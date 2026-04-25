"""End-to-end offline pipeline: mock trajectory -> MANO -> retargeter -> MuJoCo sim.

Bypasses the network for fast local validation. Useful as a smoke test and as a
development aid while iterating on retargeting quality.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from .coord_convert import unity_positions_to_mujoco
from .frame import Frame
from .joints import JOINT_IDS, JOINT_INDEX
from .mock_sender import _build_frame
from .retarget import (
    ShadowRetargeter,
    actuator_targets_from_joint_angles,
    viture_positions_to_mano,
)
from .sim import actuator_names, load_model, skeleton_mocap_ids
from .teleop_app import OPERATOR2MANO, HUMAN_TO_ROBOT_SCALE


SKELETON_ORIGIN = np.array([-0.30, -0.35, 0.18], dtype=np.float64)


def _drive(
    frame: Frame,
    retargeter: ShadowRetargeter,
    mj_acts: list[str],
    mocap_slots: dict[str, dict[str, int]],
    data: mujoco.MjData,
) -> None:
    viture_pos = frame.joint_positions().astype(np.float64)
    op_pos = viture_pos - viture_pos[JOINT_INDEX["wrist"]]
    mano_pos = (op_pos @ OPERATOR2MANO[frame.hand].T) * HUMAN_TO_ROBOT_SCALE
    mano = viture_positions_to_mano(mano_pos.astype(np.float32))
    angles = retargeter.retarget(mano)
    ctrl = actuator_targets_from_joint_angles(retargeter.joint_names, angles, mj_acts)
    data.ctrl[:] = ctrl

    wrist_idx = JOINT_IDS.index("wrist")
    rel_pos = viture_pos - viture_pos[wrist_idx]
    mj_rel = unity_positions_to_mujoco(rel_pos)
    hand_slots = mocap_slots.get(frame.hand, {})
    for i, name in enumerate(JOINT_IDS):
        slot = hand_slots.get(name)
        if slot is not None:
            data.mocap_pos[slot] = SKELETON_ORIGIN + mj_rel[i]


def run_offline(duration: float = 5.0, hz: float = 30.0) -> dict:
    model, data = load_model()
    mj_acts = actuator_names(model)
    mocap_slots = skeleton_mocap_ids(model)
    retargeter = ShadowRetargeter()

    seq = 0
    period = 1.0 / hz
    traj_t = 0.0

    openness: list[float] = []
    steps_between = max(1, int(round(period / model.opt.timestep)))

    while traj_t < duration:
        frame_dict = _build_frame(seq=seq, wall_t=traj_t, traj_t=traj_t)
        frame = Frame.model_validate(frame_dict)
        _drive(frame, retargeter, mj_acts, mocap_slots, data)
        for _ in range(steps_between):
            mujoco.mj_step(model, data)

        ffj0 = data.ctrl[mj_acts.index("rh_A_FFJ0")]
        mfj0 = data.ctrl[mj_acts.index("rh_A_MFJ0")]
        rfj0 = data.ctrl[mj_acts.index("rh_A_RFJ0")]
        lfj0 = data.ctrl[mj_acts.index("rh_A_LFJ0")]
        openness.append(float(ffj0 + mfj0 + rfj0 + lfj0))
        seq += 1
        traj_t += period
        if not np.all(np.isfinite(data.qpos)):
            raise RuntimeError(f"sim diverged at seq={seq}")

    openness_arr = np.array(openness, dtype=np.float32)
    return {
        "frames": seq,
        "openness_min": float(openness_arr.min()),
        "openness_max": float(openness_arr.max()),
        "openness_span": float(openness_arr.max() - openness_arr.min()),
    }


def render_png(out: Path, traj_t: float, settle_steps: int = 600) -> None:
    """Render a single frame at a given trajectory time for visual verification."""
    from PIL import Image

    model, data = load_model()
    mj_acts = actuator_names(model)
    mocap_slots = skeleton_mocap_ids(model)
    retargeter = ShadowRetargeter()

    frame = Frame.model_validate(_build_frame(seq=0, wall_t=traj_t, traj_t=traj_t))
    _drive(frame, retargeter, mj_acts, mocap_slots, data)
    for _ in range(settle_steps):  # lets PD controllers settle toward targets
        mujoco.mj_step(model, data)
    # Re-apply mocap after stepping (mj_step can't change mocap, but scene needs
    # data.mocap_pos at render time).
    _drive(frame, retargeter, mj_acts, mocap_slots, data)

    r = mujoco.Renderer(model, height=480, width=640)
    r.update_scene(data)
    Image.fromarray(r.render()).save(out)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--render-open", type=Path, default=None)
    p.add_argument("--render-fist", type=Path, default=None)
    args = p.parse_args()

    stats = run_offline(duration=args.duration, hz=args.hz)
    print(stats)

    if args.render_open is not None:
        render_png(args.render_open, traj_t=0.0)
        print(f"rendered open-hand PNG: {args.render_open}")
    if args.render_fist is not None:
        render_png(args.render_fist, traj_t=0.5)
        print(f"rendered fist PNG: {args.render_fist}")


if __name__ == "__main__":
    main()
