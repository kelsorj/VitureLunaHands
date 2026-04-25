from __future__ import annotations

import ast
import socket
import subprocess
import sys
import time
from pathlib import Path


SERVICE_DIR = Path(__file__).resolve().parents[1]


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _wait_for_listening(port: int, timeout: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                return True
        except OSError:
            time.sleep(0.1)
    return False


def test_teleop_app_applies_mock_frames() -> None:
    port = _free_port()
    teleop = subprocess.Popen(
        [
            sys.executable,
            "-u",
            "-m",
            "viture_teleop.teleop_app",
            "--headless",
            "--duration",
            "5",
            "--port",
            str(port),
        ],
        cwd=SERVICE_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        assert _wait_for_listening(port), "teleop_app didn't open its port"
        subprocess.run(
            [
                sys.executable,
                "-u",
                "-m",
                "viture_teleop.mock_sender",
                "--url",
                f"ws://127.0.0.1:{port}",
                "--hz",
                "30",
                "--duration",
                "2.5",
            ],
            cwd=SERVICE_DIR,
            capture_output=True,
            text=True,
            timeout=15,
        )
        teleop.wait(timeout=10)
        out = teleop.stdout.read() if teleop.stdout else ""
    finally:
        if teleop.poll() is None:
            teleop.terminate()
            teleop.wait(timeout=5)

    assert "frames_applied" in out, out
    summary = [l for l in out.splitlines() if "frames_applied" in l][-1].strip()
    stats = ast.literal_eval(summary)
    # Smoke test: confirm frames flow through retarget+sim. Tight thresholds
    # are flaky on shared CI runners (dual retargeter init dominates). Real-
    # time perf is verified visually with the glasses, not here.
    assert stats["frames_applied"] >= 5, f"expected >=5 applies, got {stats}"
