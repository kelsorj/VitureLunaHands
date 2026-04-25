"""Coordinate-frame utilities.

The Viture SDK reports joint positions in Unity's convention:
  - left-handed, +X right, +Y up, +Z forward, meters.

MuJoCo uses a right-handed frame: +X right, +Y forward, +Z up. The mapping
that aligns "physically right/up/forward" between the two frames is:

    mj = (ux, uz, uy)

This permutation has det = -1, i.e. it's a reflection — exactly what's needed
to convert from a left-handed system (Unity) to a right-handed one (MuJoCo).
Without that handedness flip, "user reaches forward" pulls the rendered hand
*toward* the camera and "thumbs up" rotates the sim hand thumbs-down; with it,
both work without any conjugate hack.
"""

from __future__ import annotations

import numpy as np


def unity_positions_to_mujoco(positions: np.ndarray) -> np.ndarray:
    """Apply the Unity→MuJoCo remap to an (N, 3) array of positions."""
    assert positions.ndim == 2 and positions.shape[1] == 3
    out = np.empty_like(positions)
    out[:, 0] = positions[:, 0]
    out[:, 1] = positions[:, 2]
    out[:, 2] = positions[:, 1]
    return out


# Change-of-basis matrix matching unity_positions_to_mujoco: (ux,uy,uz) -> (ux,uz,uy).
# det = -1 (reflection), which performs the LH→RH handedness flip that Unity
# (left-handed) and MuJoCo (right-handed) require.
_UNITY_TO_MJ_M = np.array(
    [[1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0],
     [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


def unity_quat_to_mujoco_quat(unity_xyzw: np.ndarray) -> np.ndarray:
    """Convert a Unity (x,y,z,w) quaternion into MuJoCo (w,x,y,z) order,
    accounting for the +Y-up ↔ +Z-up axis remap. Unity's XRHands subsystem
    returns quaternions in OpenXR (right-handed) convention, not Unity's
    internal LH convention, so a pure basis-change is sufficient — no
    handedness conjugate needed. Falls back to identity for a degenerate
    (zero-norm) input.
    """
    from scipy.spatial.transform import Rotation

    q = np.asarray(unity_xyzw, dtype=np.float64)
    if float(np.linalg.norm(q)) < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    R_u = Rotation.from_quat(q).as_matrix()
    R_m = _UNITY_TO_MJ_M @ R_u @ _UNITY_TO_MJ_M.T
    x, y, z, w = Rotation.from_matrix(R_m).as_quat()
    return np.array([w, x, y, z], dtype=np.float64)


def _safe_normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return v
    return v / n


def world_to_hand_local(
    positions_26x3: np.ndarray,
    wrist_idx: int,
    middle_proximal_idx: int,
    index_proximal_idx: int,
    little_proximal_idx: int,
    hand: str = "right",
) -> np.ndarray:
    """Re-express world-frame joints in the wrist's anatomical frame.

    The retargeter (and our mock trajectory) assume joints in a canonical
    hand-local frame: wrist at origin, +Y along fingers (wrist → middle_proximal),
    +X toward the pinky side of the palm (for the right hand). This function
    builds that basis from the joint geometry and projects all 26 joints into
    it. Required for Viture-streamed data; harmless on mock data because mock
    is already in this frame (rotation matrix degenerates to identity).
    """
    wrist = positions_26x3[wrist_idx]
    forward = _safe_normalize(positions_26x3[middle_proximal_idx] - wrist)
    side = _safe_normalize(
        positions_26x3[little_proximal_idx] - positions_26x3[index_proximal_idx]
    )
    if np.allclose(forward, 0) or np.allclose(side, 0):
        return positions_26x3 - wrist  # degenerate frame; fall back to translate-only

    # Canonical hand-local frame (matches mock_trajectory): wrist at origin,
    # +X = pinky side, +Y = fingers, +Z = back of hand. Same formula for both
    # hands — the per-hand mirror is applied downstream by dex-retargeting's
    # OPERATOR2MANO_RIGHT/LEFT matrices, so we shouldn't pre-flip here.
    up = _safe_normalize(np.cross(side, forward))
    right_axis = _safe_normalize(np.cross(forward, up))

    # Rows are the world-frame components of each local axis.
    # local_v = M @ (world_v - wrist) → first row · v gives the x (right) component.
    M = np.stack([right_axis, forward, up], axis=0)
    return (positions_26x3 - wrist) @ M.T
