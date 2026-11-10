# iPhone 13 Pro Max LiDAR → Humanoid Robot Teleoperation (Windows + iOS)

Real-time **60 FPS 3D Human Motion Tracking + LiDAR Depth Fusion** on **iPhone 13 Pro Max** (`ARBodyTrackingConfiguration` + `.smoothedSceneDepth`), cross-compiled in the cloud via **GitHub Actions (`macos-14`)**, sideloaded from **Windows via Sideloadly**, and streamed over **UDP** to a **Windows Python Kinematics Server + 3D Robot Simulator**.

---

## Project Structure

```text
iphone-lidar-robot-tracker/
├── .github/workflows/
│   └── build-ios.yml                 # Cloud macOS Xcode build -> outputs unsigned .ipa artifact
├── ios-app/
│   ├── project.yml                   # XcodeGen spec (generates .xcodeproj in CI without a Mac)
│   └── Sources/
│       ├── LiDARBotTrackerApp.swift  # SwiftUI entry point
│       ├── ContentView.swift         # AR HUD, LiDAR depth reticle, IP/Port config, 91-joint toggle
│       ├── ARBodyLiDARManager.swift  # ARKit 3D Body Tracking + LiDAR CVPixelBuffer depth sampler
│       └── UDPStreamer.swift         # Low-latency 60Hz Network.framework UDP client
└── windows-server/
    ├── server.py                     # Windows UDP receiver + HTTP/SSE server (zero pip dependencies!)
    ├── kinematics.py                 # 3D Skeleton -> 16-DOF Humanoid Robot IK & EMA filter
    └── static/
        └── index.html                # Real-time Three.js 3D Human + Robot Visualizer & LiDAR Heatmap
```

---

## Step 1: Test the Windows Server & 3D Robot Simulator Right Now

No third-party Python packages are required (`server.py` uses Python's built-in `asyncio` and `socket` libraries).

```powershell
cd C:\Users\user1\.gemini\antigravity\scratch\iphone-lidar-robot-tracker\windows-server
python server.py
```

1. Open **http://localhost:8080** in your browser.
2. It starts in **Synthetic Demo Mode**, simulating a 60 FPS iPhone 13 Pro Max ARKit skeleton + 8×6 LiDAR depth grid and retargeting the motion in real time to a **16-DOF Humanoid Robot**.
3. Note the **Windows Wi-Fi LAN IP** printed in your terminal (e.g., `192.168.1.xxx`) and shown in the top-right sidebar.

---

## Step 2: Cross-Compile the iOS App in the Cloud (No Mac Needed)

Because `.github/workflows/build-ios.yml` uses **XcodeGen** (`ios-app/project.yml`), GitHub's free `macos-14` runner generates the `.xcodeproj` and compiles the release `.ipa` automatically:

1. Create a free GitHub repository (public repositories get unlimited free `macos-14` build minutes) and push this folder:
   ```powershell
   cd C:\Users\user1\.gemini\antigravity\scratch\iphone-lidar-robot-tracker
   git init
   git add .
   git commit -m "Initial commit: iPhone 13 Pro Max LiDAR Robot Tracker"
   git branch -M main
   git remote add origin https://github.com/<your-username>/<your-repo>.git
   git push -u origin main
   ```
2. Go to the **Actions** tab on your GitHub repository and click **Build Unsigned iOS IPA (for Windows Sideloadly)**.
3. Once the workflow finishes (~2–3 minutes), download the **`LiDARBotTracker-iOS-IPA`** artifact zip and extract **`LiDARBotTracker-unsigned.ipa`**.

---

## Step 3: Sideload onto iPhone 13 Pro Max from Windows

1. Install **iTunes** and **iCloud** (non-Microsoft Store versions from Apple) and **Sideloadly** (`https://sideloadly.io/`) on Windows.
2. Connect your **iPhone 13 Pro Max** to your Windows PC via USB cable and tap **Trust This Computer** on the iPhone.
3. Open **Sideloadly**:
   - Drag **`LiDARBotTracker-unsigned.ipa`** into Sideloadly.
   - Enter your **Apple ID** (any free Apple ID works; Sideloadly signs the app with a free 7-day developer certificate).
   - Click **Start**.
4. On your iPhone 13 Pro Max:
   - Go to **Settings → General → VPN & Device Management** and trust your Apple ID developer certificate.
   - On iOS 16+, enable **Settings → Privacy & Security → Developer Mode**.

---

## Step 4: Live Human-to-Robot Teleoperation

1. Make sure your **iPhone 13 Pro Max** and **Windows PC** are on the same Wi-Fi network (or connect the iPhone to your Windows PC's Mobile Hotspot for ultra-low <3 ms latency).
2. Run `python windows-server/server.py` on Windows (allow Python through Windows Firewall on UDP port `5005` if prompted).
3. Launch **LiDAR Teleop** on your iPhone 13 Pro Max:
   - Point the rear camera + LiDAR scanner at a person standing ~1.5m–3.5m away.
   - Enter the **Windows PC IP Address** shown in the server terminal and port **`5005`**.
   - Tap **Stream to Windows Robot**.
4. The Windows server automatically detects the first UDP packet and switches from `DEMO` to `LIVE_UDP` mode!
