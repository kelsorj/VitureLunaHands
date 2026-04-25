from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from viture_teleop.frame import Frame
from viture_teleop.mock_sender import _build_frame
from viture_teleop.recorder import JsonlRecorder


def test_recorder_writes_valid_jsonl(tmp_path: Path) -> None:
    rec = JsonlRecorder(tmp_path / "demo.jsonl")
    for i in range(10):
        frame = Frame.model_validate(_build_frame(seq=i, wall_t=float(i), traj_t=i * 0.03))
        rec(frame)
    rec.close()
    assert rec.count == 10

    with (tmp_path / "demo.jsonl").open() as f:
        lines = [line.rstrip() for line in f if line.strip()]
    assert len(lines) == 10
    for line in lines:
        parsed = json.loads(line)
        Frame.model_validate(parsed)  # round-trip validation
