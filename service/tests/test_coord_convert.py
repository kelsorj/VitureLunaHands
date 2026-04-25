from __future__ import annotations

import numpy as np

from viture_teleop.coord_convert import unity_positions_to_mujoco


def test_shape_preserved() -> None:
    pts = np.random.default_rng(0).random((10, 3))
    out = unity_positions_to_mujoco(pts)
    assert out.shape == pts.shape


def test_axis_mapping() -> None:
    # Unity +X (right)   → MuJoCo +X
    # Unity +Y (up)      → MuJoCo +Z (up)
    # Unity +Z (forward) → MuJoCo +Y (forward)
    # Permutation has det=-1, performing the LH→RH handedness flip.
    unity = np.array(
        [
            [1.0, 0.0, 0.0],  # +X
            [0.0, 1.0, 0.0],  # +Y
            [0.0, 0.0, 1.0],  # +Z
        ]
    )
    mj = unity_positions_to_mujoco(unity)
    np.testing.assert_allclose(mj[0], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(mj[1], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(mj[2], [0.0, 1.0, 0.0])


def test_rejects_non_3d_vectors() -> None:
    import pytest

    with pytest.raises(AssertionError):
        unity_positions_to_mujoco(np.zeros((5, 2)))
    with pytest.raises(AssertionError):
        unity_positions_to_mujoco(np.zeros(3))
