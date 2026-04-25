from __future__ import annotations

import argparse
import asyncio
import json
import time

import websockets

from .joints import JOINT_IDS
from .mock_trajectory import trajectory_at


def _build_frame(seq: int, wall_t: float, traj_t: float) -> dict:
    pose = trajectory_at(traj_t)
    joints = [
        {"id": name, "p": list(pose[name]), "r": [0.0, 0.0, 0.0, 1.0]}
        for name in JOINT_IDS
    ]
    return {
        "t": wall_t,
        "seq": seq,
        "hand": "right",
        "tracked": True,
        "wrist_pos": list(pose["wrist"]),
        "wrist_rot": [0.0, 0.0, 0.0, 1.0],
        "joints": joints,
    }


async def run(url: str, hz: float, duration: float | None) -> None:
    period = 1.0 / hz
    seq = 0
    t0 = time.monotonic()
    async with websockets.connect(url, ping_interval=None) as ws:
        print(f"connected to {url}; sending {hz:.0f} Hz")
        next_tick = t0
        while True:
            now = time.monotonic()
            traj_t = now - t0
            if duration is not None and traj_t > duration:
                break
            frame = _build_frame(seq, wall_t=time.time(), traj_t=traj_t)
            await ws.send(json.dumps(frame))
            seq += 1
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                await asyncio.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    print(f"sent {seq} frames")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="ws://127.0.0.1:8765")
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--duration", type=float, default=None, help="seconds; infinite if omitted")
    args = p.parse_args()
    asyncio.run(run(args.url, args.hz, args.duration))


if __name__ == "__main__":
    main()
