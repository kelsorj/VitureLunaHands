# VitureProject — glasses-driven dexterous hand teleop (sim PoC)

Stream the 26-joint hand skeleton from Viture Luma Ultra XR glasses + Pro
Neckband → retarget onto a Shadow Hand in MuJoCo → watch the robot hand mirror
your hand in real time. A sim-only proof-of-concept, designed so a real
humanoid (eventual target: 1X Neo) can slot in later behind the same
protocol.

## Layout

```
service/                 Python teleop service (uv-managed, Python 3.11)
    src/viture_teleop/
        joints.py         Canonical 26-joint ordering (Unity XRHandJointID)
        frame.py          Pydantic wire-format model + validation
        coord_convert.py  Unity (LH, Y-up) ↔ MuJoCo (RH, Z-up) conversion
        mock_trajectory.py   Procedural open↔fist test trajectory
        mock_sender.py    WebSocket client; replays the mock trajectory
        server.py         WebSocket server; logs frames / drop rates
        retarget.py       Viture→MANO mapping + dex-retargeting wrapper
        sim.py            MuJoCo Shadow Hand loader + scene + mocap helpers
        pipeline.py       Offline pipeline: mock → retarget → sim → PNG
        teleop_app.py     Live pipeline: WS server + retarget + MuJoCo viewer
        recorder.py       Writes each incoming frame to JSONL (demo capture)
        replayer.py       Streams a recorded JSONL back over WebSocket
    tests/                21 pytest tests covering all of the above
unity-app/                Unity 2022.3 LTS Android project (Pro Neckband)
    Assets/Scripts/
        HandStreamer.cs   Reads XRHands data, sends JSON frames over WS
        ConfigPanel.cs    In-app IP/port setting
    Packages/manifest.json   Dependencies (XR Hands, OpenXR, NativeWebSocket, Newtonsoft)
    ProjectSettings/...   EditorSettings.asset + ProjectVersion.txt
    .gitignore            Unity standard ignores (Library, Temp, etc.)
    README.md             Unity install/build/deploy instructions
vendor/
    mujoco_menagerie/     Cloned; we use shadow_hand/scene_right.xml and the
                          custom scene_teleop.xml added alongside it
    dex_urdf/             Cloned; we reference robots/hands/shadow_hand/*
artifacts/                Rendered PNGs + recorded demo JSONL files
```

## Setup from a fresh clone

Prerequisites:

- **Python 3.11** + [uv](https://docs.astral.sh/uv/) (`brew install uv` on macOS)
- **Unity 2022.3 LTS** with Android Build Support — only needed if you intend
  to build the glasses-side app yourself
- The Shadow Hand assets used by the sim (Apache-2.0, ~5 MB) are committed
  under `vendor/mujoco_menagerie/shadow_hand/` and `vendor/dex_urdf/robots/hands/shadow_hand/`,
  so the **sim works out of the box** without any vendor bootstrap step.

To run the **Unity side** you also need the Viture SDK (proprietary, not
redistributable — must be obtained separately):

```bash
# 1. Sign up at https://developer.viture.com and download the Unity XR SDK
#    package for the Pro Neckband.
# 2. Drop the unzipped 'com.viture.xr' folder into vendor/, so the path
#    referenced from unity-app/Packages/manifest.json resolves:
#       vendor/com.viture.xr/package.json
```

If you skip that step, `service/` will still run end-to-end against the mock
sender or recorded JSONL files — only the live-from-glasses flow is blocked.

## Quickstart (sim-only, no glasses needed)

```bash
# First-time setup
cd service
uv sync              # pulls mujoco, dex-retargeting, torch, pinocchio, etc.

# Run the offline smoke test: mock trajectory → Shadow Hand, renders 2 PNGs
uv run python -m viture_teleop.pipeline \
    --duration 3 \
    --render-open ../artifacts/retarget_open.png \
    --render-fist ../artifacts/retarget_fist.png
open ../artifacts/retarget_open.png ../artifacts/retarget_fist.png

# Run the tests
uv run pytest -v
```

## End-to-end (with mock sender, no glasses yet)

Terminal A — the teleop app with a live MuJoCo viewer window:

```bash
cd service
# macOS: must use `mjpython` (handles Cocoa main-thread requirements).
# Linux/Windows: plain `python` is fine.
uv run mjpython -m viture_teleop.teleop_app --port 8765
```

Terminal B — pretend to be the glasses:

```bash
cd service
uv run python -m viture_teleop.mock_sender --url ws://127.0.0.1:8765 --hz 30
```

You should see the Shadow Hand open↔fist in the MuJoCo viewer at ~30 Hz,
with a yellow skeleton overlay floating above it that shows the raw Viture
26-joint input.

## Recording + replaying demos

Record every frame that arrives at the teleop app:

```bash
cd service
uv run mjpython -m viture_teleop.teleop_app --port 8765 --record-to ../artifacts/demo.jsonl
```

Then replay a recording without the hardware attached:

```bash
# terminal A — receive
uv run mjpython -m viture_teleop.teleop_app --port 8770
# terminal B — play back
uv run python -m viture_teleop.replayer --url ws://127.0.0.1:8770 --file ../artifacts/demo.jsonl
```

Each JSONL line is a complete, self-validating `Frame`. Same format whether
produced by the mock sender, the glasses, or a future logger — so any tool
that can read the wire format can read these files.

## End-to-end with glasses

1. On the laptop: `uv run python -m viture_teleop.teleop_app --port 8765` — the
   viewer window opens and the WS server is up.
2. Note your laptop's LAN IP: `ipconfig getifaddr en0` (macOS).
3. Open `unity-app/` in Unity 2022.3 LTS; follow `unity-app/README.md` to
   install Viture's SDK, build the Android APK, and deploy to the Pro
   Neckband.
4. Launch the app on the glasses, enter the laptop IP, tap Connect.
5. Move your right hand — the Shadow Hand should mirror it in the viewer.

## Wire format

One JSON message per frame, sent ~30 Hz over WebSocket.

```json
{
  "t": 1745432400.123,
  "seq": 8421,
  "hand": "right",
  "tracked": true,
  "wrist_pos": [x, y, z],
  "wrist_rot": [qx, qy, qz, qw],
  "joints": [ {"id": "palm",  "p": [x,y,z], "r": [qx,qy,qz,qw]}, ... ]
}
```

Canonical joint order is in `service/src/viture_teleop/joints.py` and matches
Unity's `XRHandJointID` enum (OpenXR order). Any sender — not just Unity — can
speak this protocol.

## What this proves vs what's still missing

Sim PoC boundaries (intentionally narrow):

- ✅ 26-joint stream → MANO → Shadow Hand driven in MuJoCo at ~30 Hz, real-time
- ✅ Coord-frame + joint-order mapping documented + tested
- ✅ Coupled-tendon actuator handling (J0 = J1+J2) for Shadow Hand
- ✅ Headless E2E test that runs in CI
- ❌ Arm / wrist IK (the wrist is attached to a fixed forearm mount)
- ❌ Bimanual
- ❌ Real humanoid (no 1X Neo SDK today; architecture leaves seams for it)
- ❌ LeRobot-format demo logging
- ❌ Gesture command layer
- ❌ Network auth / NAT traversal

Each of these is a natural next increment; the single-hand loop is deliberately
rock-solid first.
