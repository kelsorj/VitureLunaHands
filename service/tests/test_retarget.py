from __future__ import annotations

import numpy as np
import pytest

from viture_teleop.frame import Frame
from viture_teleop.mock_sender import _build_frame
from viture_teleop.retarget import (
    ShadowRetargeter,
    actuator_targets_from_joint_angles,
    viture_positions_to_mano,
)
from viture_teleop.sim import actuator_names, load_model


@pytest.fixture(scope="module")
def retargeter() -> ShadowRetargeter:
    return ShadowRetargeter()


def _mano_at(t: float) -> np.ndarray:
    f = Frame.model_validate(_build_frame(seq=0, wall_t=t, traj_t=t))
    return viture_positions_to_mano(f.joint_positions())


def test_viture_to_mano_shape() -> None:
    f = Frame.model_validate(_build_frame(seq=0, wall_t=0.0, traj_t=0.0))
    mano = viture_positions_to_mano(f.joint_positions())
    assert mano.shape == (21, 3)


def test_retargeter_outputs_24_joints(retargeter: ShadowRetargeter) -> None:
    angles = retargeter.retarget(_mano_at(0.0))
    assert angles.shape == (24,)
    assert np.all(np.isfinite(angles))


def test_retargeter_responds_to_input(retargeter: ShadowRetargeter) -> None:
    retargeter.retargeting.reset()
    a_open = retargeter.retarget(_mano_at(0.0))
    for _ in range(5):  # let the low-pass filter settle toward fist
        a_fist = retargeter.retarget(_mano_at(0.5))
    assert not np.allclose(a_open, a_fist, atol=1e-3), (
        "retargeter output should change between open and fist"
    )


def test_actuator_mapping_couples_j0() -> None:
    urdf_names = ["FFJ1", "FFJ2", "WRJ1"]
    angles = np.array([0.3, 0.4, 0.1])
    mj_names = ["rh_A_FFJ0", "rh_A_WRJ1"]
    ctrl = actuator_targets_from_joint_angles(urdf_names, angles, mj_names)
    assert ctrl[0] == pytest.approx(0.7)  # FFJ1 + FFJ2
    assert ctrl[1] == pytest.approx(0.1)  # WRJ1


def test_ctrl_stays_within_actuator_bounds(retargeter: ShadowRetargeter) -> None:
    model, _ = load_model()
    mj_acts = actuator_names(model)
    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]
    retargeter.retargeting.reset()
    for t in np.linspace(0.0, 1.0, 10):
        angles = retargeter.retarget(_mano_at(float(t)))
        ctrl = actuator_targets_from_joint_angles(
            retargeter.joint_names, angles, mj_acts
        )
        # Clamp is acceptable; we just want values reasonable, not totally wild
        assert np.all(ctrl >= lo - 0.1)
        assert np.all(ctrl <= hi + 0.1)
