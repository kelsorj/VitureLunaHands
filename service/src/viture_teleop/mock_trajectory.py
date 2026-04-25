from __future__ import annotations

import math

import numpy as np

from .joints import JOINT_IDS


# Approximate open-hand skeleton in Unity-style coordinates (meters).
# Left-handed, Y up, X right, Z forward (relative to the wrist).
# Wrist at origin, palm normal pointing along -Z, fingers along +Y.
_FINGER_BASE_X = {  # lateral offset per finger at metacarpal
    "thumb": -0.03,
    "index": -0.025,
    "middle": 0.0,
    "ring": 0.025,
    "little": 0.045,
}
_FINGER_LEN = {  # metacarpal→tip segment total (meters)
    "thumb": 0.08,
    "index": 0.10,
    "middle": 0.11,
    "ring": 0.10,
    "little": 0.085,
}
# Proportion of total length from wrist at metacarpal→proximal→mid→distal→tip
_FINGER_SEGS = {
    "thumb": (0.15, 0.50, 0.80, 1.00),           # no intermediate
    "index": (0.15, 0.45, 0.70, 0.88, 1.00),
    "middle": (0.15, 0.45, 0.70, 0.88, 1.00),
    "ring": (0.15, 0.45, 0.70, 0.88, 1.00),
    "little": (0.15, 0.45, 0.70, 0.88, 1.00),
}


def _open_hand_pose() -> dict[str, tuple[float, float, float]]:
    pose: dict[str, tuple[float, float, float]] = {
        "palm":  (0.0, 0.04, 0.0),
        "wrist": (0.0, 0.00, 0.0),
    }
    for finger in ("thumb", "index", "middle", "ring", "little"):
        x = _FINGER_BASE_X[finger]
        total = _FINGER_LEN[finger]
        segs = _FINGER_SEGS[finger]
        names = _finger_joint_names(finger)
        for name, frac in zip(names, segs):
            pose[name] = (x, frac * total, 0.0)
    return pose


def _fist_pose() -> dict[str, tuple[float, float, float]]:
    """Curl fingers toward palm by rotating each finger segment around its base."""
    pose = _open_hand_pose()
    for finger in ("thumb", "index", "middle", "ring", "little"):
        names = _finger_joint_names(finger)
        base = pose[names[0]]
        for name in names[1:]:
            x, y, z = pose[name]
            dy = y - base[1]
            # Rotate around X axis by -pi/2 (fingers curl toward palm, -Z)
            y_new = base[1]
            z_new = -dy
            pose[name] = (x, y_new, z_new)
    return pose


def _finger_joint_names(finger: str) -> tuple[str, ...]:
    if finger == "thumb":
        return (
            "thumb_metacarpal",
            "thumb_proximal",
            "thumb_distal",
            "thumb_tip",
        )
    return (
        f"{finger}_metacarpal",
        f"{finger}_proximal",
        f"{finger}_intermediate",
        f"{finger}_distal",
        f"{finger}_tip",
    )


def trajectory_at(t_seconds: float) -> dict[str, tuple[float, float, float]]:
    """Blend between open hand and fist, 1 Hz sinusoid.

    Returns a dict mapping each of the 26 joint names to (x, y, z)."""
    alpha = 0.5 - 0.5 * math.cos(2 * math.pi * t_seconds)  # 0..1..0 per second
    open_p = _open_hand_pose()
    fist_p = _fist_pose()
    return {
        name: tuple(
            (1 - alpha) * open_p[name][i] + alpha * fist_p[name][i] for i in range(3)
        )
        for name in JOINT_IDS
    }
