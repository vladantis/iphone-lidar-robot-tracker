import Foundation
import Network

/// High-frequency, low-latency UDP client using Apple's Network.framework (`NWConnection`)
/// Streams 60 FPS JSON skeleton + LiDAR telemetry packets to the Windows Python server.
final class UDPStreamer: ObservableObject {
    @Published private(set) var isConnected: Bool = false
    @Published private(set) var packetsSent: Int = 0
    @Published private(set) var lastError: String? = nil

    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "com.robotteleop.udpstreamer", qos: .userInteractive)
    private let encoder = JSONEncoder()

    func connect(host: String, port: UInt16) {
        disconnect()

        let endpointHost = NWEndpoint.Host(host.trimmingCharacters(in: .whitespacesAndNewlines))
        guard let endpointPort = NWEndpoint.Port(rawValue: port) else {
            DispatchQueue.main.async {
                self.lastError = "Invalid port: \(port)"
            }
            return
        }

        let params = NWParameters.udp
        params.serviceClass = .interactiveVideo // Minimize Wi-Fi queuing latency

        let conn = NWConnection(host: endpointHost, port: endpointPort, using: params)
        self.connection = conn

        conn.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                guard let self = self else { return }
                switch state {
                case .ready:
                    self.isConnected = true
                    self.lastError = nil
                case .failed(let error):
                    self.isConnected = false
                    self.lastError = error.localizedDescription
                case .cancelled:
                    self.isConnected = false
                default:
                    break
                }
            }
        }

        conn.start(queue: queue)
    }

    func disconnect() {
        connection?.cancel()
        connection = nil
        DispatchQueue.main.async {
            self.isConnected = false
        }
    }

    func send<T: Encodable>(payload: T) {
        guard let conn = connection, isConnected else { return }

        queue.async { [weak self] in
            guard let self = self else { return }
            do {
                let data = try self.encoder.encode(payload)
                conn.send(content: data, completion: .contentProcessed { error in
                    if let error = error {
                        DispatchQueue.main.async {
                            self.lastError = error.localizedDescription
                        }
                    } else {
                        DispatchQueue.main.async {
                            self.packetsSent &+= 1
                        }
                    }
                })
            } catch {
                DispatchQueue.main.async {
                    self.lastError = "Encode error: \(error.localizedDescription)"
                }
            }
        }
    }
}
