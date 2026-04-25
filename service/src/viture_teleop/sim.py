from __future__ import annotations

import argparse
import math
import os
import time
from pathlib import Path

import mujoco
import numpy as np


_SHADOW_DIR = (
    Path(__file__).resolve().parents[3]
    / "vendor"
    / "mujoco_menagerie"
    / "shadow_hand"
)
DEFAULT_MODEL = _SHADOW_DIR / "scene_teleop.xml"
BASIC_SCENE = _SHADOW_DIR / "scene_right.xml"


def model_path() -> Path:
    env = os.environ.get("VITURE_SHADOW_HAND_SCENE")
    return Path(env) if env else DEFAULT_MODEL


def load_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    path = model_path()
    if not path.exists():
        raise FileNotFoundError(
            f"Shadow Hand scene not found at {path}. "
            "Did you clone mujoco_menagerie under vendor/?"
        )
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    return model, data


def actuator_names(model: mujoco.MjModel) -> list[str]:
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
    ]


def skeleton_mocap_ids(model: mujoco.MjModel) -> dict[str, dict[str, int]]:
    """Map hand ('right'|'left') -> {joint_name: mocap_index}.

    Bodies named 'skel_rh_<joint>' / 'skel_lh_<joint>' route to right/left.
    """
    mapping: dict[str, dict[str, int]] = {"right": {}, "left": {}}
    for body_id in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if not name or not name.startswith("skel_"):
            continue
        mocap_id = int(model.body_mocapid[body_id])
        if mocap_id < 0:
            continue
        rest = name[len("skel_"):]
        if rest.startswith("rh_"):
            mapping["right"][rest[3:]] = mocap_id
        elif rest.startswith("lh_"):
            mapping["left"][rest[3:]] = mocap_id
    return mapping


def body_id(model: mujoco.MjModel, name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError(f"body '{name}' not found in model")
    return bid


def set_body_pose(
    model: mujoco.MjModel,
    body_id_: int,
    pos: np.ndarray,
    quat_wxyz: np.ndarray | None = None,
) -> None:
    """Override a fixed body's parent-relative pose. Call mj_forward after."""
    model.body_pos[body_id_] = pos
    if quat_wxyz is not None:
        model.body_quat[body_id_] = quat_wxyz


def _swept_targets(model: mujoco.MjModel, t: float) -> np.ndarray:
    """Each actuator swept by a sin wave, phase-shifted, scaled to its ctrlrange."""
    phases = np.linspace(0.0, 2.0 * math.pi, model.nu, endpoint=False)
    s = 0.5 + 0.5 * np.sin(2.0 * math.pi * 0.3 * t + phases)
    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]
    return lo + s * (hi - lo)


def run_headless(duration: float = 3.0, verbose: bool = False) -> dict:
    model, data = load_model()
    steps = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        data.ctrl[:] = _swept_targets(model, data.time)
        mujoco.mj_step(model, data)
        steps += 1
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise RuntimeError(f"sim diverged at step {steps}")
    wall = time.monotonic() - t0
    stats = {
        "nu": int(model.nu),
        "nq": int(model.nq),
        "steps": steps,
        "wall_s": round(wall, 3),
        "sim_time": round(float(data.time), 3),
        "realtime_ratio": round(float(data.time / wall), 3) if wall > 0 else 0.0,
    }
    if verbose:
        print("actuators:")
        for name, (lo, hi) in zip(
            actuator_names(model), model.actuator_ctrlrange
        ):
            print(f"  {name:<20s}  [{lo:+.3f}, {hi:+.3f}]")
    return stats


def run_viewer() -> None:
    import mujoco.viewer

    model, data = load_model()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            data.ctrl[:] = _swept_targets(model, data.time)
            mujoco.mj_step(model, data)
            viewer.sync()


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--mode",
        choices=("headless", "viewer"),
        default="headless",
        help="headless runs a fixed-duration smoke test; viewer opens a window",
    )
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    if args.mode == "headless":
        stats = run_headless(duration=args.duration, verbose=args.verbose)
        print(stats)
    else:
        run_viewer()


if __name__ == "__main__":
    main()
