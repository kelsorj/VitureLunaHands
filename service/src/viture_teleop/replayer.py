"""Replayer: send a recorded JSONL file through a WebSocket endpoint.

Uses each frame's `t` wall-clock field for pacing so the replay preserves the
original timing. Loops by default if the file duration is short.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time
from pathlib import Path

import websockets


async def run(url: str, jsonl_path: Path, loop: bool = True, rate: float = 1.0) -> None:
    frames: list[dict] = []
    with jsonl_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    if not frames:
        raise RuntimeError(f"no frames in {jsonl_path}")
    t0_file = float(frames[0]["t"])

    async with websockets.connect(url, ping_interval=None) as ws:
        print(f"replayer: connected to {url}, {len(frames)} frames")
        while True:
            t0_wall = time.monotonic()
            for i, raw in enumerate(frames):
                target = t0_wall + (float(raw["t"]) - t0_file) / rate
                sleep_for = target - time.monotonic()
                if sleep_for > 0:
                    await asyncio.sleep(sleep_for)
                # Rewrite t and seq so the receiver sees a fresh stream each loop.
                raw = dict(raw)
                raw["t"] = time.time()
                raw["seq"] = i  # restart seq each loop
                await ws.send(json.dumps(raw))
            if not loop:
                break
            print("replayer: looping")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="ws://127.0.0.1:8765")
    p.add_argument("--file", type=Path, required=True)
    p.add_argument("--once", action="store_true", help="don't loop")
    p.add_argument("--rate", type=float, default=1.0, help="playback speed multiplier")
    args = p.parse_args()
    asyncio.run(run(args.url, args.file, loop=not args.once, rate=args.rate))


if __name__ == "__main__":
    main()
