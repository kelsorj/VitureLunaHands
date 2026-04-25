from __future__ import annotations

import pytest
from pydantic import ValidationError

from viture_teleop.frame import Frame
from viture_teleop.joints import JOINT_IDS
from viture_teleop.mock_sender import _build_frame


def _frame_dict() -> dict:
    return _build_frame(seq=0, wall_t=0.0, traj_t=0.0)


def test_parse_well_formed_frame() -> None:
    f = Frame.model_validate(_frame_dict())
    assert f.seq == 0
    assert f.hand == "right"
    assert len(f.joints) == 26


def test_joint_positions_shape() -> None:
    f = Frame.model_validate(_frame_dict())
    p = f.joint_positions()
    assert p.shape == (26, 3)


def test_reject_wrong_joint_count() -> None:
    d = _frame_dict()
    d["joints"] = d["joints"][:25]
    with pytest.raises(ValidationError):
        Frame.model_validate(d)


def test_reject_wrong_joint_name() -> None:
    d = _frame_dict()
    d["joints"][0]["id"] = "not_a_real_joint"
    with pytest.raises(ValidationError):
        Frame.model_validate(d)


def test_reject_bad_hand() -> None:
    d = _frame_dict()
    d["hand"] = "middle"
    with pytest.raises(ValidationError):
        Frame.model_validate(d)


def test_joint_order_matches_spec() -> None:
    d = _frame_dict()
    names = [j["id"] for j in d["joints"]]
    assert tuple(names) == JOINT_IDS
