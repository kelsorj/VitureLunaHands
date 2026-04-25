from __future__ import annotations

import argparse
import asyncio
import json
import time
from dataclasses import dataclass

import websockets
from websockets.asyncio.server import ServerConnection

from .frame import Frame


@dataclass
class ConnStats:
    frames: int = 0
    dropped: int = 0
    last_seq: int | None = None
    t_first: float | None = None
    right_frames: int = 0
    left_frames: int = 0
    t_last_right: float | None = None
    t_last_left: float | None = None

    def observe(self, frame: Frame) -> None:
        self.frames += 1
        now = time.monotonic()
        if self.t_first is None:
            self.t_first = now
        if self.last_seq is not None and frame.seq != self.last_seq + 1:
            gap = frame.seq - self.last_seq - 1
            if gap > 0:
                self.dropped += gap
        self.last_seq = frame.seq
        if frame.hand == "right":
            self.right_frames += 1
            self.t_last_right = now
        else:
            self.left_frames += 1
            self.t_last_left = now

    def rate(self) -> float:
        if self.t_first is None or self.frames == 0:
            return 0.0
        dt = time.monotonic() - self.t_first
        return self.frames / dt if dt > 0 else 0.0

    def per_hand_rates(self) -> tuple[float, float]:
        if self.t_first is None:
            return 0.0, 0.0
        dt = time.monotonic() - self.t_first
        if dt <= 0:
            return 0.0, 0.0
        return self.right_frames / dt, self.left_frames / dt


async def handle(ws: ServerConnection, consumer=None) -> None:
    peer = ws.remote_address
    print(f"connect  {peer}")
    stats = ConnStats()
    try:
        async for msg in ws:
            try:
                raw = json.loads(msg)
                frame = Frame.model_validate(raw)
            except Exception as e:
                print(f"[{peer}] bad frame: {e}")
                continue
            stats.observe(frame)
            if consumer is not None:
                consumer(frame)
            if stats.frames % 60 == 0:
                r_hz, l_hz = stats.per_hand_rates()
                print(
                    f"[{peer}] total={stats.frames} dropped={stats.dropped} "
                    f"right={stats.right_frames}@{r_hz:.1f}Hz "
                    f"left={stats.left_frames}@{l_hz:.1f}Hz"
                )
    except websockets.ConnectionClosed:
        pass
    finally:
        r_hz, l_hz = stats.per_hand_rates()
        print(
            f"disconnect {peer}; total={stats.frames} dropped={stats.dropped} "
            f"right={stats.right_frames}@{r_hz:.1f}Hz left={stats.left_frames}@{l_hz:.1f}Hz"
        )


async def run(host: str, port: int, consumer=None) -> None:
    async def _h(ws):
        await handle(ws, consumer=consumer)

    async with websockets.serve(_h, host, port, max_size=2**20):
        print(f"listening on ws://{host}:{port}")
        await asyncio.Future()  # run forever


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8765)
    args = p.parse_args()
    asyncio.run(run(args.host, args.port))


if __name__ == "__main__":
    main()
