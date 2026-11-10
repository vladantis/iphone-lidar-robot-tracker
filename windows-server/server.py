"""
Windows Teleoperation Server for iPhone 13 Pro Max LiDAR + ARKit 3D Body Tracking.
- Listens on UDP Port 5005 for real-time 60Hz ARKit skeleton + LiDAR packets from the iPhone.
- Retargets 3D human joints to a 16-DOF Humanoid Robot in real time.
- Serves an interactive 3D Human-to-Robot Visualizer & LiDAR Depth Dashboard at http://localhost:8080
- Includes a built-in Synthetic Demo Mode so you can test on Windows immediately before sideloading.
"""

import argparse
import asyncio
import json
import os
import socket
import time
from pathlib import Path
from typing import Dict, Set

from kinematics import ARKIT_BONE_PAIRS, HumanToRobotRetargeter, generate_demo_frame

STATIC_DIR = Path(__file__).parent / "static"


def get_local_lan_ip() -> str:
    """Detects the Windows PC's primary Wi-Fi / LAN IPv4 address."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


class TeleopState:
    def __init__(self, start_in_demo: bool = True):
        self.mode: str = "DEMO" if start_in_demo else "LIVE_UDP"
        self.retargeter = HumanToRobotRetargeter(smoothing_alpha=0.38)
        self.latest_packet: dict = {}
        self.latest_robot_angles: dict = {"radians": {}, "degrees": {}}
        self.udp_packets_received: int = 0
        self.last_udp_time: float = 0.0
        self.last_sender_addr: str = "None"
        self.fps: float = 0.0
        self._fps_counter: int = 0
        self._fps_timer: float = time.time()
        self.subscribers: Set[asyncio.Queue] = set()

    def process_incoming_frame(self, packet: dict, source: str):
        now = time.time()
        self._fps_counter += 1
        if now - self._fps_timer >= 1.0:
            self.fps = round(self._fps_counter / (now - self._fps_timer), 1)
            self._fps_counter = 0
            self._fps_timer = now

        joints = packet.get("joints", {})
        robot_angles = self.retargeter.retarget(joints)

        self.latest_packet = packet
        self.latest_robot_angles = robot_angles

        # Dispatch to any hardware robot controller hook here:
        self.on_robot_command(robot_angles["radians"], robot_angles["degrees"])

        # Broadcast combined telemetry to connected 3D Visualizer browsers
        if self.subscribers:
            envelope = {
                "source": source,
                "mode": self.mode,
                "fps": self.fps,
                "udpPackets": self.udp_packets_received,
                "sender": self.last_sender_addr,
                "frame": packet,
                "robot": robot_angles,
                "bones": ARKIT_BONE_PAIRS,
            }
            msg = json.dumps(envelope)
            dead = []
            for q in self.subscribers:
                try:
                    if q.full():
                        _ = q.get_nowait()
                    q.put_nowait(msg)
                except Exception:
                    dead.append(q)
            for q in dead:
                self.subscribers.discard(q)

    def on_robot_command(self, angles_rad: Dict[str, float], angles_deg: Dict[str, float]):
        """
        HARDWARE / SIMULATOR HOOK:
        Override or add serial / ROS2 / MuJoCo commands here.
        Example for Arduino / ESP32 Serial:
            serial_port.write((json.dumps(angles_deg) + "\\n").encode())
        """
        pass


class iPhoneUDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: TeleopState):
        self.state = state

    def datagram_received(self, data: bytes, addr):
        try:
            packet = json.loads(data.decode("utf-8"))
        except Exception:
            return

        self.state.udp_packets_received += 1
        self.state.last_udp_time = time.time()
        self.state.last_sender_addr = f"{addr[0]}:{addr[1]}"

        # Automatically switch from DEMO to LIVE_UDP when the iPhone starts streaming
        if self.state.mode == "DEMO":
            self.state.mode = "LIVE_UDP"
            print(f"\n[LIVE] First UDP packet received from iPhone at {addr[0]}:{addr[1]}! Switching to LIVE_UDP mode.")

        self.state.process_incoming_frame(packet, source="IPHONE_LIDAR")


async def demo_loop(state: TeleopState):
    """Runs at 60Hz to feed synthetic ARKit + LiDAR frames when in DEMO mode."""
    t0 = time.time()
    frame_idx = 0
    while True:
        if state.mode == "DEMO":
            frame_idx += 1
            t = time.time() - t0
            packet = generate_demo_frame(frame_idx, t)
            state.process_incoming_frame(packet, source="SYNTHETIC_DEMO")
        await asyncio.sleep(1.0 / 60.0)


async def handle_http_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, state: TeleopState, lan_ip: str, udp_port: int):
    try:
        request_line = await asyncio.wait_for(reader.readline(), timeout=5.0)
        if not request_line:
            writer.close()
            return
        parts = request_line.decode("utf-8", errors="ignore").strip().split(" ")
        if len(parts) < 2:
            writer.close()
            return
        method, path = parts[0], parts[1]

        # Read headers
        content_length = 0
        while True:
            line = await reader.readline()
            if not line or line in (b"\r\n", b"\n"):
                break
            header_str = line.decode("utf-8", errors="ignore").strip()
            if header_str.lower().startswith("content-length:"):
                content_length = int(header_str.split(":", 1)[1].strip())

        body = b""
        if content_length > 0:
            body = await reader.readexactly(content_length)

        # Route 1: Server-Sent Events (SSE) real-time 60Hz stream for the 3D visualizer
        if path == "/events":
            headers = (
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/event-stream\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: keep-alive\r\n"
                "Access-Control-Allow-Origin: *\r\n\r\n"
            )
            writer.write(headers.encode("utf-8"))
            await writer.drain()

            q: asyncio.Queue = asyncio.Queue(maxsize=2)
            state.subscribers.add(q)
            try:
                while True:
                    payload = await q.get()
                    writer.write(f"data: {payload}\n\n".encode("utf-8"))
                    await writer.drain()
            except Exception:
                pass
            finally:
                state.subscribers.discard(q)
                writer.close()
            return

        # Route 2: Toggle between DEMO and LIVE_UDP mode
        if path == "/api/mode" and method == "POST":
            try:
                req = json.loads(body.decode("utf-8"))
                new_mode = req.get("mode", "DEMO")
                if new_mode in ("DEMO", "LIVE_UDP"):
                    state.mode = new_mode
            except Exception:
                pass
            resp = json.dumps({"mode": state.mode, "lan_ip": lan_ip, "udp_port": udp_port}).encode("utf-8")
            writer.write(
                f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {len(resp)}\r\n\r\n".encode("utf-8") + resp
            )
            await writer.drain()
            writer.close()
            return

        # Route 3: Server info endpoint
        if path == "/api/info":
            resp = json.dumps({
                "mode": state.mode,
                "lan_ip": lan_ip,
                "udp_port": udp_port,
                "udp_packets": state.udp_packets_received,
                "sender": state.last_sender_addr,
            }).encode("utf-8")
            writer.write(
                f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {len(resp)}\r\n\r\n".encode("utf-8") + resp
            )
            await writer.drain()
            writer.close()
            return

        # Route 4: Serve index.html
        if path in ("/", "/index.html"):
            html_file = STATIC_DIR / "index.html"
            content = html_file.read_bytes()
            writer.write(
                f"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {len(content)}\r\n\r\n".encode("utf-8")
                + content
            )
            await writer.drain()
            writer.close()
            return

        # 404 Not Found
        writer.write(b"HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\n\r\nNot Found")
        await writer.drain()
        writer.close()
    except Exception:
        try:
            writer.close()
        except Exception:
            pass


async def main(udp_port: int, http_port: int, start_in_demo: bool):
    lan_ip = get_local_lan_ip()
    state = TeleopState(start_in_demo=start_in_demo)

    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(
        lambda: iPhoneUDPProtocol(state),
        local_addr=("0.0.0.0", udp_port),
    )

    http_server = await asyncio.start_server(
        lambda r, w: handle_http_client(r, w, state, lan_ip, udp_port),
        host="0.0.0.0",
        port=http_port,
    )

    print("=" * 72)
    print("  iPhone 13 Pro Max LiDAR -> Humanoid Robot Teleop Server (Windows)")
    print("=" * 72)
    print(f"  1. Windows Wi-Fi LAN IP : {lan_ip}   <-- Enter this IP in the iPhone app")
    print(f"  2. UDP Listener Port    : {udp_port}")
    print(f"  3. 3D Web Visualizer    : http://localhost:{http_port}")
    print(f"  4. Active Mode          : {state.mode} (auto-switches on iPhone UDP packet)")
    print("=" * 72)

    try:
        async with http_server:
            await demo_loop(state)
    finally:
        transport.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="iPhone LiDAR Human-to-Robot Teleoperation Server")
    parser.add_argument("--udp-port", type=int, default=5005, help="UDP port for iPhone stream (default: 5005)")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP port for 3D Visualizer (default: 8080)")
    parser.add_argument("--live-only", action="store_true", help="Disable synthetic demo motion on startup")
    args = parser.parse_args()

    try:
        asyncio.run(main(args.udp_port, args.http_port, start_in_demo=not args.live_only))
    except KeyboardInterrupt:
        print("\nServer stopped.")
