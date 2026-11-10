import Foundation
import ARKit
import RealityKit
import Combine
import simd

/// Telemetry packet sent at 60 FPS over UDP to the Windows `r1_mocap` Python pipeline.
struct TeleopFramePacket: Codable {
    let timestamp: Double
    let frameNumber: Int
    let lidarAvailable: Bool
    let bodyTracked: Bool
    let estimatedHeightScale: Float
    let rootPosition: [Float]          // [x, y, z] in meters (ARKit world space)
    let rootOrientation: [Float]       // [qx, qy, qz, qw]
    let centerLidarDepthM: Float       // Direct LiDAR depth at screen center (meters)
    let lidarGrid8x6: [Float]          // Downsampled 8x6 metric LiDAR depth grid (meters)
    /// Dictionary of ARKit joint name -> [x, y, z, qx, qy, qz, qw] in root-relative metric space
    let joints: [String: [Float]]
    /// Direct `r1_mocap` compatible landmarks in camera frame (x right, y down, z forward in meters)
    /// Format: landmark_name -> [x_cam, y_cam, z_cam, u_2d, v_2d, lidar_depth_m]
    let r1Landmarks: [String: [Float]]
}

final class ARBodyLiDARManager: NSObject, ObservableObject, ARSessionDelegate {
    // MARK: - Published UI State
    @Published var isSupported: Bool = true
    @Published var lidarActive: Bool = false
    @Published var bodyDetected: Bool = false
    @Published var fps: Int = 0
    @Published var centerDepthMeters: Float = 0.0
    @Published var trackedJointCount: Int = 0
    @Published var statusMessage: String = "Initializing ARKit + LiDAR..."
    @Published var streamAll91Joints: Bool = false

    let streamer = UDPStreamer()
    let arView = ARView(frame: .zero)

    // Mapping from r1_mocap landmark names (map_to_r1.py & map_full_body.py) to ARKit ARSkeleton3D joints
    private let r1ToARKitJointMap: [(r1Name: String, arkitName: String)] = [
        ("nose", "head_joint"),
        ("left_shoulder", "left_arm_joint"),
        ("right_shoulder", "right_arm_joint"),
        ("left_elbow", "left_forearm_joint"),
        ("right_elbow", "right_forearm_joint"),
        ("left_wrist", "left_hand_joint"),
        ("right_wrist", "right_hand_joint"),
        ("left_hip", "left_upLeg_joint"),
        ("right_hip", "right_upLeg_joint"),
        ("left_knee", "left_leg_joint"),
        ("right_knee", "right_leg_joint"),
        ("left_ankle", "left_foot_joint"),
        ("right_ankle", "right_foot_joint"),
        ("left_heel", "left_foot_joint"),
        ("right_heel", "right_foot_joint"),
        ("left_foot_index", "left_toes_joint"),
        ("right_foot_index", "right_toes_joint")
    ]

    private let coreRoboticsJoints: Set<String> = [
        "root", "hips_joint",
        "spine_1_joint", "spine_4_joint", "spine_7_joint",
        "neck_1_joint", "head_joint",
        "left_shoulder_1_joint", "left_arm_joint", "left_forearm_joint", "left_hand_joint",
        "right_shoulder_1_joint", "right_arm_joint", "right_forearm_joint", "right_hand_joint",
        "left_upLeg_joint", "left_leg_joint", "left_foot_joint", "left_toes_joint",
        "right_upLeg_joint", "right_leg_joint", "right_foot_joint", "right_toes_joint"
    ]

    // 3D visualization entities inside RealityKit ARView
    private var skeletonAnchor = AnchorEntity()
    private var jointEntities: [String: ModelEntity] = [:]

    private var frameCounter: Int = 0
    private var fpsFrameCount: Int = 0
    private var lastFpsTimestamp: TimeInterval = 0

    override init() {
        super.init()
        arView.session.delegate = self
        arView.scene.addAnchor(skeletonAnchor)
    }

    func startSession() {
        guard ARBodyTrackingConfiguration.isSupported else {
            isSupported = false
            statusMessage = "ARBodyTrackingConfiguration is not supported on this device."
            return
        }

        let config = ARBodyTrackingConfiguration()
        config.automaticSkeletonScaleEstimationEnabled = true
        config.isAutoFocusEnabled = true

        // Enable iPhone 13 Pro Max LiDAR Scanner depth fusion
        if ARBodyTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics.insert(.smoothedSceneDepth)
            lidarActive = true
        } else if ARBodyTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
            lidarActive = true
        } else {
            lidarActive = false
        }

        arView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        statusMessage = lidarActive
            ? "LiDAR + 3D Body Tracking Active (60Hz)"
            : "3D Body Tracking Active (No LiDAR frame semantics)"
    }

    func stopSession() {
        arView.session.pause()
    }

    // MARK: - ARSessionDelegate (60 FPS callback)
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        frameCounter &+= 1
        fpsFrameCount &+= 1

        if frame.timestamp - lastFpsTimestamp >= 1.0 {
            let currentFps = fpsFrameCount
            fpsFrameCount = 0
            lastFpsTimestamp = frame.timestamp
            DispatchQueue.main.async {
                self.fps = currentFps
            }
        }

        let depthBuffer = (frame.smoothedSceneDepth ?? frame.sceneDepth)?.depthMap

        // 1. Sample LiDAR Depth Map (smoothedSceneDepth or sceneDepth)
        var centerDepth: Float = 0.0
        var gridDepth: [Float] = []
        if let depthMap = depthBuffer {
            let sampled = Self.sampleLiDARDepth(depthMap: depthMap)
            centerDepth = sampled.center
            gridDepth = sampled.grid8x6
        }

        // 2. Find ARBodyAnchor
        let bodyAnchor = frame.anchors.compactMap { $0 as? ARBodyAnchor }.first

        var jointsPayload: [String: [Float]] = [:]
        var r1Payload: [String: [Float]] = [:]
        var rootPos: [Float] = [0, 0, 0]
        var rootQuat: [Float] = [0, 0, 0, 1]
        var scaleFactor: Float = 1.0
        let hasBody = (bodyAnchor != nil)

        if let body = bodyAnchor {
            scaleFactor = Float(body.estimatedScaleFactor)
            let worldTransform = body.transform
            let camInv = simd_inverse(frame.camera.transform)
            let imgRes = frame.camera.imageResolution

            rootPos = [
                Self.round4(worldTransform.columns.3.x),
                Self.round4(worldTransform.columns.3.y),
                Self.round4(worldTransform.columns.3.z)
            ]
            let rq = simd_quatf(worldTransform)
            rootQuat = [
                Self.round4(rq.imag.x),
                Self.round4(rq.imag.y),
                Self.round4(rq.imag.z),
                Self.round4(rq.real)
            ]

            let skeleton = body.skeleton
            let definition = skeleton.definition
            let jointNames = definition.jointNames
            let modelTransforms = skeleton.jointModelTransforms

            var nameToIndex: [String: Int] = [:]
            nameToIndex.reserveCapacity(jointNames.count)

            for (idx, jointName) in jointNames.enumerated() {
                nameToIndex[jointName] = idx
                if !streamAll91Joints && !coreRoboticsJoints.contains(jointName) {
                    continue
                }
                let jt = modelTransforms[idx]
                let pos = jt.columns.3
                let quat = simd_quatf(jt)

                jointsPayload[jointName] = [
                    Self.round4(pos.x),
                    Self.round4(pos.y),
                    Self.round4(pos.z),
                    Self.round4(quat.imag.x),
                    Self.round4(quat.imag.y),
                    Self.round4(quat.imag.z),
                    Self.round4(quat.real)
                ]
            }

            // Build `r1_mocap` camera-frame 3D landmarks (x right, y down, z forward in meters)
            for pair in r1ToARKitJointMap {
                guard let idx = nameToIndex[pair.arkitName] else { continue }
                let jtWorld = simd_mul(worldTransform, modelTransforms[idx])
                let worldPt = SIMD3<Float>(jtWorld.columns.3.x, jtWorld.columns.3.y, jtWorld.columns.3.z)
                let camPt4 = simd_mul(camInv, jtWorld.columns.3)

                // ARKit camera frame: +X right, +Y up, -Z forward
                // r1_mocap camera frame: +X right, +Y down, +Z forward
                let xR1 = Self.round4(camPt4.x)
                let yR1 = Self.round4(-camPt4.y)
                let zR1 = Self.round4(-camPt4.z)

                let proj = frame.camera.projectPoint(
                    worldPt,
                    orientation: .portrait,
                    viewportSize: imgRes
                )
                let uNorm = Self.round4(Float(proj.x / max(1.0, imgRes.width)))
                let vNorm = Self.round4(Float(proj.y / max(1.0, imgRes.height)))

                var rawLidarD: Float = zR1
                if let depthMap = depthBuffer, uNorm >= 0, uNorm <= 1, vNorm >= 0, vNorm <= 1 {
                    let sampledD = Self.sampleLiDARAtNormalizedUV(depthMap: depthMap, u: uNorm, v: vNorm)
                    if sampledD > 0.1 {
                        rawLidarD = sampledD
                    }
                }

                r1Payload[pair.r1Name] = [xR1, yR1, zR1, uNorm, vNorm, rawLidarD]
            }

            // Update AR overlay spheres for core joints
            updateSkeletonOverlay(bodyTransform: worldTransform, joints: jointsPayload)
        } else {
            skeletonAnchor.isEnabled = false
        }

        // Update UI metrics at 15Hz to keep main thread snappy
        if frameCounter % 4 == 0 {
            let count = jointsPayload.count
            DispatchQueue.main.async {
                self.bodyDetected = hasBody
                self.centerDepthMeters = centerDepth
                self.trackedJointCount = count
            }
        }

        // 3. Stream UDP Packet to Windows r1_mocap / Teleop Server
        if streamer.isConnected {
            let packet = TeleopFramePacket(
                timestamp: frame.timestamp,
                frameNumber: frameCounter,
                lidarAvailable: lidarActive,
                bodyTracked: hasBody,
                estimatedHeightScale: Self.round4(scaleFactor),
                rootPosition: rootPos,
                rootOrientation: rootQuat,
                centerLidarDepthM: Self.round4(centerDepth),
                lidarGrid8x6: gridDepth,
                joints: jointsPayload,
                r1Landmarks: r1Payload
            )
            streamer.send(payload: packet)
        }
    }

    // MARK: - LiDAR CVPixelBuffer Samplers
    private static func sampleLiDARAtNormalizedUV(depthMap: CVPixelBuffer, u: Float, v: Float) -> Float {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return 0.0 }

        let x = max(0, min(width - 1, Int(u * Float(width))))
        let y = max(0, min(height - 1, Int(v * Float(height))))
        let rowPtr = baseAddress.advanced(by: y * bytesPerRow)
        let val = rowPtr.assumingMemoryBound(to: Float32.self)[x]
        return (val.isNaN || val.isInfinite) ? 0.0 : round4(val)
    }

    private static func sampleLiDARDepth(depthMap: CVPixelBuffer) -> (center: Float, grid8x6: [Float]) {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else {
            return (0.0, [])
        }

        func readDepth(x: Int, y: Int) -> Float {
            let clampedX = max(0, min(width - 1, x))
            let clampedY = max(0, min(height - 1, y))
            let rowPtr = baseAddress.advanced(by: clampedY * bytesPerRow)
            let floatPtr = rowPtr.assumingMemoryBound(to: Float32.self)
            let val = floatPtr[clampedX]
            return (val.isNaN || val.isInfinite) ? 0.0 : round4(val)
        }

        let centerVal = readDepth(x: width / 2, y: height / 2)

        var grid: [Float] = []
        grid.reserveCapacity(48)
        let cols = 8
        let rows = 6
        for r in 0..<rows {
            let y = Int((Float(r) + 0.5) / Float(rows) * Float(height))
            for c in 0..<cols {
                let x = Int((Float(c) + 0.5) / Float(cols) * Float(width))
                grid.append(readDepth(x: x, y: y))
            }
        }

        return (centerVal, grid)
    }

    // MARK: - RealityKit 3D Skeleton Overlay
    private func updateSkeletonOverlay(bodyTransform: simd_float4x4, joints: [String: [Float]]) {
        skeletonAnchor.isEnabled = true
        skeletonAnchor.transform = Transform(matrix: bodyTransform)

        for (name, values) in joints where values.count >= 3 {
            let pos = SIMD3<Float>(values[0], values[1], values[2])
            if let entity = jointEntities[name] {
                entity.position = pos
            } else {
                let isEndEffector = name.contains("hand") || name.contains("foot") || name.contains("head")
                let radius: Float = isEndEffector ? 0.035 : 0.022
                let color: UIColor = isEndEffector ? .systemCyan : .systemGreen
                let mesh = MeshResource.generateSphere(radius: radius)
                let mat = SimpleMaterial(color: color, isMetallic: false)
                let entity = ModelEntity(mesh: mesh, materials: [mat])
                entity.position = pos
                skeletonAnchor.addChild(entity)
                jointEntities[name] = entity
            }
        }
    }

    @inline(__always)
    private static func round4(_ v: Float) -> Float {
        (v * 10000.0).rounded() / 10000.0
    }
}
