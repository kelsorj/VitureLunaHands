"""Frame recorder.

Attaches as a consumer to the WebSocket server and appends every received
Frame as a JSON line. The resulting JSONL file replays cleanly through
`viture_teleop.replayer`, or can be post-processed into LeRobot format later.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time
from pathlib import Path
from typing import TextIO

from .frame import Frame
from .server import run as server_run


class JsonlRecorder:
    def __init__(self, out_path: Path) -> None:
        self.out_path = out_path
        self._f: TextIO | None = None
        self.count = 0
        self._t_start: float | None = None

    def open(self) -> None:
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self.out_path.open("w", encoding="utf-8")
        self._t_start = time.monotonic()
        print(f"recorder: writing to {self.out_path}")

    def __call__(self, frame: Frame) -> None:
        if self._f is None:
            self.open()
        self._f.write(frame.model_dump_json())
        self._f.write("\n")
        self.count += 1
        if self.count % 60 == 0:
            self._f.flush()

    def close(self) -> None:
        if self._f is not None:
            self._f.flush()
            self._f.close()
            self._f = None
        dur = (time.monotonic() - (self._t_start or 0.0)) if self._t_start else 0.0
        print(f"recorder: {self.count} frames in {dur:.2f}s ({self.out_path})")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--out", type=Path, required=True, help="path to .jsonl file")
    args = p.parse_args()

    rec = JsonlRecorder(args.out)
    try:
        asyncio.run(server_run(args.host, args.port, consumer=rec))
    except KeyboardInterrupt:
        pass
    finally:
        rec.close()


if __name__ == "__main__":
    main()
