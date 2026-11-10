import SwiftUI
import RealityKit

struct ARViewContainer: UIViewRepresentable {
    let arView: ARView

    func makeUIView(context: Context) -> ARView {
        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {}
}

struct ContentView: View {
    @StateObject private var arManager = ARBodyLiDARManager()
    @AppStorage("windowsHostIP") private var hostIP: String = "192.168.1.100"
    @AppStorage("windowsHostPort") private var hostPort: String = "5005"
    @State private var isStreaming: Bool = false

    var body: some View {
        ZStack {
            // 1. Live ARKit + LiDAR Camera Feed & 3D Skeleton Overlay
            ARViewContainer(arView: arManager.arView)
                .ignoresSafeArea()

            // 2. Center Reticle for LiDAR Depth Readout
            Circle()
                .strokeBorder(Color.cyan.opacity(0.8), lineWidth: 1.5)
                .frame(width: 18, height: 18)

            // 3. HUD Overlay
            VStack {
                // Top Telemetry Bar
                VStack(alignment: .leading, spacing: 8) {
                    HStack(spacing: 10) {
                        StatusBadge(
                            title: arManager.lidarActive ? "LiDAR ON" : "NO LiDAR",
                            active: arManager.lidarActive,
                            activeColor: .cyan
                        )
                        StatusBadge(
                            title: arManager.bodyDetected ? "BODY LOCKED (\(arManager.trackedJointCount)J)" : "SEARCHING BODY",
                            active: arManager.bodyDetected,
                            activeColor: .green
                        )
                        Spacer()
                        Text("\(arManager.fps) FPS")
                            .font(.system(.caption, design: .monospaced).bold())
                            .padding(.horizontal, 8)
                            .padding(.vertical, 4)
                            .background(Color.black.opacity(0.65))
                            .foregroundColor(.white)
                            .clipShape(Capsule())
                    }

                    HStack {
                        Label(
                            String(format: "Center LiDAR Depth: %.2f m", arManager.centerDepthMeters),
                            systemImage: "camera.metering.center.weighted"
                        )
                        .font(.system(.caption, design: .monospaced))
                        .foregroundColor(.cyan)

                        Spacer()

                        Toggle("All 91 Joints", isOn: $arManager.streamAll91Joints)
                            .toggleStyle(.button)
                            .font(.caption2.bold())
                            .tint(.purple)
                    }
                }
                .padding(12)
                .background(.ultraThinMaterial)
                .clipShape(RoundedRectangle(cornerRadius: 16))
                .padding(.horizontal, 12)
                .padding(.top, 8)

                Spacer()

                // Bottom Windows Teleop Streamer Panel
                VStack(spacing: 10) {
                    HStack(spacing: 8) {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("WINDOWS PC IP ADDRESS")
                                .font(.system(size: 10, weight: .bold, design: .monospaced))
                                .foregroundColor(.secondary)
                            TextField("192.168.1.100", text: $hostIP)
                                .keyboardType(.decimalPad)
                                .textFieldStyle(.roundedBorder)
                                .font(.system(.subheadline, design: .monospaced))
                        }

                        VStack(alignment: .leading, spacing: 2) {
                            Text("UDP PORT")
                                .font(.system(size: 10, weight: .bold, design: .monospaced))
                                .foregroundColor(.secondary)
                            TextField("5005", text: $hostPort)
                                .keyboardType(.numberPad)
                                .textFieldStyle(.roundedBorder)
                                .frame(width: 80)
                                .font(.system(.subheadline, design: .monospaced))
                        }
                    }

                    HStack(spacing: 12) {
                        Button(action: toggleStream) {
                            HStack {
                                Image(systemName: isStreaming ? "stop.fill" : "antenna.radiowaves.left.and.right")
                                Text(isStreaming ? "Stop UDP Stream" : "Stream to Windows Robot")
                                    .fontWeight(.bold)
                            }
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 12)
                            .background(isStreaming ? Color.red : Color.blue)
                            .foregroundColor(.white)
                            .clipShape(RoundedRectangle(cornerRadius: 12))
                        }

                        if isStreaming {
                            VStack(alignment: .trailing, spacing: 2) {
                                Text("PACKETS")
                                    .font(.system(size: 9, weight: .bold, design: .monospaced))
                                    .foregroundColor(.secondary)
                                Text("\(arManager.streamer.packetsSent)")
                                    .font(.system(.caption, design: .monospaced).bold())
                                    .foregroundColor(.green)
                            }
                            .frame(minWidth: 70, alignment: .trailing)
                        }
                    }

                    if let err = arManager.streamer.lastError {
                        Text(err)
                            .font(.caption2)
                            .foregroundColor(.red)
                            .lineLimit(1)
                    }
                }
                .padding(14)
                .background(.ultraThinMaterial)
                .clipShape(RoundedRectangle(cornerRadius: 18))
                .padding(.horizontal, 12)
                .padding(.bottom, 12)
            }
        }
        .onAppear {
            UIApplication.shared.isIdleTimerDisabled = true
            arManager.startSession()
        }
        .onDisappear {
            UIApplication.shared.isIdleTimerDisabled = false
            arManager.stopSession()
        }
    }

    private func toggleStream() {
        UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder), to: nil, from: nil, for: nil)
        if isStreaming {
            arManager.streamer.disconnect()
            isStreaming = false
        } else {
            let port = UInt16(hostPort) ?? 5005
            arManager.streamer.connect(host: hostIP, port: port)
            isStreaming = true
        }
    }
}

struct StatusBadge: View {
    let title: String
    let active: Bool
    let activeColor: Color

    var body: some View {
        HStack(spacing: 5) {
            Circle()
                .fill(active ? activeColor : Color.orange)
                .frame(width: 8, height: 8)
            Text(title)
                .font(.system(size: 11, weight: .bold, design: .monospaced))
                .foregroundColor(.white)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(Color.black.opacity(0.65))
        .clipShape(Capsule())
    }
}
