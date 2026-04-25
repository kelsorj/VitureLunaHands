from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np
from dex_retargeting.retargeting_config import RetargetingConfig

from .joints import JOINT_INDEX


DEFAULT_URDF_ROOT = (
    Path(__file__).resolve().parents[3]
    / "vendor"
    / "dex_urdf"
    / "robots"
    / "hands"
)


def _default_config_path(hand: str = "right") -> Path:
    import dex_retargeting

    fname = "shadow_hand_left.yml" if hand == "left" else "shadow_hand_right.yml"
    return Path(dex_retargeting.__file__).parent / "configs" / "teleop" / fname


# Viture's 26-joint skeleton -> MANO's 21-joint convention.
#
# MANO indexes 4 joints per finger: MCP (knuckle, base), PIP (mid), DIP (distal),
# TIP. OpenXR exposes 5 anatomical points per finger:
#   metacarpal -> palm bone (NOT a joint that bends)
#   proximal   -> the MCP joint (knuckle)
#   intermediate -> the PIP joint (middle)
#   distal     -> the DIP joint
#   tip        -> the fingertip
#
# So MANO's MCP/PIP/DIP/TIP map to OpenXR's proximal/intermediate/distal/tip.
# Drop the metacarpal — it's effectively a palm point.
#
# The thumb has only 3 joints anatomically; OpenXR's thumb_metacarpal is at the
# CMC (carpometacarpal, base of thumb) which fills MANO's index 1.
_MANO_SOURCE_NAMES: tuple[str, ...] = (
    "wrist",                                                                  # 0
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",        # 1..4
    "index_proximal", "index_intermediate", "index_distal", "index_tip",      # 5..8
    "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",  # 9..12
    "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",          # 13..16
    "little_proximal", "little_intermediate", "little_distal", "little_tip",  # 17..20
)

_VITURE_TO_MANO_IDX: np.ndarray = np.array(
    [JOINT_INDEX[n] for n in _MANO_SOURCE_NAMES], dtype=np.int64
)
assert _VITURE_TO_MANO_IDX.shape == (21,)


def viture_positions_to_mano(positions_26x3: np.ndarray) -> np.ndarray:
    """Remap a (26, 3) Viture-ordered position array to (21, 3) MANO order."""
    return positions_26x3[_VITURE_TO_MANO_IDX, :]


class ShadowRetargeter:
    """Thin wrapper over dex-retargeting's Shadow Hand teleop config."""

    def __init__(
        self,
        hand: str = "right",
        config_path: Path | None = None,
        urdf_root: Path = DEFAULT_URDF_ROOT,
        low_pass_alpha: float | None = 0.7,
    ) -> None:
        """
        Args:
            low_pass_alpha: smoothing for the temporal LP filter (0..1). The
                bundled YAML uses 0.2 which produces ~200 ms of perceptible
                lag at 25 Hz — fingers visibly trail a quick fist. We default
                to 0.7 (much snappier) but expose the knob for tuning. Set to
                None to leave the YAML default in place; set to 1.0 to bypass
                the filter entirely.
        """
        self.hand = hand
        config_path = config_path or _default_config_path(hand)
        if not urdf_root.exists():
            raise FileNotFoundError(
                f"URDF root not found: {urdf_root}. Did you clone vendor/dex_urdf?"
            )
        RetargetingConfig.set_default_urdf_dir(str(urdf_root))
        cfg = RetargetingConfig.load_from_file(str(config_path))
        if low_pass_alpha is not None:
            cfg.low_pass_alpha = float(low_pass_alpha)
        self.retargeting = cfg.build()

    @property
    def joint_names(self) -> list[str]:
        return list(self.retargeting.joint_names)

    def retarget(self, mano_joints_21x3: np.ndarray) -> np.ndarray:
        assert mano_joints_21x3.shape == (21, 3), mano_joints_21x3.shape
        # Vector retargeting expects ref_value = task_pos - origin_pos per pair.
        indices = self.retargeting.optimizer.target_link_human_indices
        retargeting_type = self.retargeting.optimizer.retargeting_type
        if retargeting_type == "POSITION":
            ref_value = mano_joints_21x3[indices, :]
        else:  # "VECTOR" or "DEXPILOT"
            origin = mano_joints_21x3[indices[0, :], :]
            task = mano_joints_21x3[indices[1, :], :]
            ref_value = task - origin
        return self.retargeting.retarget(ref_value.astype(np.float32))


def fill_actuator_targets(
    urdf_joint_names: Sequence[str],
    joint_angles: np.ndarray,
    mj_actuator_names: Sequence[str],
    ctrl: np.ndarray,
    prefix: str = "rh_A_",
) -> None:
    """Write per-URDF-joint angles into the MuJoCo `ctrl` slots for one hand.

    Only actuators whose name starts with `prefix` are written; others left
    untouched so the other hand's targets are preserved.

    Shadow Hand's MuJoCo model uses coupled tendons for the distal-most two
    joints of each non-thumb finger, exposed as a single 'J0' actuator whose
    command is the *sum* of the underlying J1+J2 joint angles.
    """
    assert ctrl.shape == (len(mj_actuator_names),), ctrl.shape
    name_to_value = {n: float(v) for n, v in zip(urdf_joint_names, joint_angles)}
    for i, act in enumerate(mj_actuator_names):
        if not act.startswith(prefix):
            continue
        core = act[len(prefix):]
        if core.endswith("J0") and not core.endswith("THJ0"):
            base = core[:-2]
            ctrl[i] = name_to_value.get(f"{base}J1", 0.0) + name_to_value.get(
                f"{base}J2", 0.0
            )
        else:
            ctrl[i] = name_to_value.get(core, 0.0)


def actuator_targets_from_joint_angles(
    urdf_joint_names: Sequence[str],
    joint_angles: np.ndarray,
    mj_actuator_names: Sequence[str],
    prefix: str = "rh_A_",
) -> np.ndarray:
    """Single-hand convenience wrapper: returns a fresh ctrl array."""
    ctrl = np.zeros(len(mj_actuator_names), dtype=np.float64)
    fill_actuator_targets(
        urdf_joint_names, joint_angles, mj_actuator_names, ctrl, prefix=prefix
    )
    return ctrl
