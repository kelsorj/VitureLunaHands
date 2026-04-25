# Viture Teleop Unity App (Android, Pro Neckband)

Unity-side app that captures the hand skeleton via Viture's XR SDK, synthesizes
the 5 metacarpal joints Viture doesn't track, and streams 26-joint frames over
WebSocket to the Python retargeting service on your laptop.

## Prerequisites

- **Unity 6000.0.73f1** (current 6.0 LTS patch) with **Android Build Support** —
  Viture's `package.json` requires Unity 6. Install via
  `brew install --cask unity-hub` then
  `"/Applications/Unity Hub.app/Contents/MacOS/Unity Hub" -- --headless install
  --version 6000.0.73f1 --architecture arm64 --module android --module mac-il2cpp
  --childModules`.
- ADB on your `PATH` (comes with the Android Build Support module).
- Viture **Pro Neckband + Luma Ultra/Pro** paired and recognized by the neckband.
- Laptop and Pro Neckband on the **same WiFi** network.

The **Viture XR SDK v0.7.0** ships in `../vendor/com.viture.xr/` and is already
wired into `Packages/manifest.json` via a relative `file:` reference, so there
is no separate SDK install step.

## One-time setup

1. In **Unity Hub**, install **Unity 6000.0.27f1** with **Android Build Support**
   (includes Android SDK + OpenJDK) and **Mac Build Support (IL2CPP)**.
2. In Unity Hub, **Add → Select Folder** → pick `unity-app/`. Open it.
3. Unity will resolve packages on first open — Viture XR (loaded via the local
   path `../vendor/com.viture.xr`), XR Hands 1.5.1, XR Management 4.5.1, XR
   Interaction Toolkit 3.0.5, Newtonsoft.Json, NativeWebSocket.
4. When prompted by Unity Hub, sign in with your Unity ID and activate a free
   **Unity Personal** license — required to open the project.
4. Go to **Edit → Project Settings → XR Plug-in Management → Android tab**:
   - Enable **VITURE XR**.
5. Go to **Player Settings → Android**:
   - **Scripting Backend**: IL2CPP
   - **Target Architecture**: ARM64 only
   - **Minimum API Level**: Android 12 / API 31 or whatever Viture recommends
   - **Package Name**: e.g. `com.yourname.viture.teleop`
6. Import Viture's Starter Assets sample (for a working XR Origin prefab):
   **Window → Package Manager → VITURE XR Plugin → Samples → Import
   "Starter Assets"** (and optionally "Interaction Simulator" for editor testing).

## Scene setup

The simplest scene:

1. **File → New Scene** → save as `Assets/Scenes/Main.unity`.
2. Drag the `Assets/Samples/VITURE XR Plugin/0.7.0/Starter Assets/Prefabs/XR Origin (Viture).prefab`
   into the scene hierarchy.
3. Create an empty GameObject named `HandStreamer`, add the
   `VitureTeleop.HandStreamer` component (in `Assets/Scripts/HandStreamer.cs`).
4. Optional UI for setting the laptop IP:
   - Create a `Canvas` (Screen Space — Overlay), add a `TMP_InputField` for
     IP, a second `TMP_InputField` for port, a `Button` for Connect, and a
     `TMP_Text` for status.
   - Add the `VitureTeleop.ConfigPanel` component to the `HandStreamer`
     GameObject. Wire the four UI references in the inspector.
5. **File → Build Settings** → add this scene; switch platform to **Android**.

## Build & deploy

1. Connect the Pro Neckband via USB-C. Follow Viture's docs to enable
   developer mode / ADB on the neckband.
2. **File → Build and Run** (or **Build** → APK, then
   `adb install -r <path.apk>`).

## Running

Terminal (laptop):

```bash
cd ../service
uv run mjpython -m viture_teleop.teleop_app --port 8765
```

Find your laptop's LAN IP: `ipconfig getifaddr en0` (macOS).

On the glasses:

1. Launch the app.
2. Enter the laptop's LAN IP and port **8765**. Tap **Connect**.
3. Move your right hand in front of you (the neckband cameras look down-front) —
   the Shadow Hand in the MuJoCo viewer on the laptop should mirror you, and
   the yellow skeleton overlay should show your raw joint positions.

## Notes on the 21-vs-26 joint gap

Viture's `VitureHandSubsystem` reports **21 joints**: wrist, palm, and
proximal/intermediate/distal/tip for each finger (4 for thumb, no intermediate).
Our wire format is **26 joints**, matching Unity's `XRHandJointID` enum and
including the 5 finger metacarpals. `HandStreamer.cs` synthesizes each
metacarpal as `Lerp(wrist, proximal, 0.25)` — anatomically close to where the
MCP joint sits on the palm. The Python retargeter uses the metacarpal purely as
a MANO MCP position, so this approximation is fine.

If you want the real MCP rotation, you'd need to derive it from the wrist
+ proximal orientation; not necessary for the current sim PoC.

## Troubleshooting

- **Can't find `Viture.XR` namespace**: Package Manager didn't resolve the
  file: dependency. Check `Packages/manifest.json` and that
  `../vendor/com.viture.xr` exists relative to `Packages/manifest.json`
  (i.e. at `../../vendor/com.viture.xr` from the Unity project root).
- **"No XRHandSubsystem found" at runtime**: XR Plug-in Management has Viture
  disabled, or the starter scene's XR Origin isn't in the hierarchy.
- **WS never connects**: laptop firewall blocks port 8765, wrong IP, or not on
  the same network. `adb shell ping <laptop-ip>` from the neckband to confirm
  LAN reachability.
- **Hand flickers / jumps**: Viture's cameras are down-facing on the neckband.
  Hold your hand ~30–60 cm in front of you, palm visible. The SDK applies a
  low-pass filter (`VitureHandFilter`) internally; if it's too aggressive you
  can tune `HandFilterMode` in `Project Settings → VITURE XR`.
