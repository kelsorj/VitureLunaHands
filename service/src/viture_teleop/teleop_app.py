"""End-to-end teleop service.

Main thread runs the MuJoCo passive viewer + sim stepping. A background thread
runs the WebSocket server and drops the latest Frame into a shared slot.
"""

from __future__ import annotations

import argparse
import asyncio
import threading
import time
from dataclasses import dataclass

import mujoco
import mujoco.viewer  # imported up here so 'mujoco' stays a module-global in run()
import numpy as np

from .coord_convert import (
    unity_positions_to_mujoco,
    unity_quat_to_mujoco_quat,
    world_to_hand_local,
)

# Operator-frame → MANO-frame basis change. The op frame is Unity-style head
# coords (+X right, +Y up, +Z forward). dex-retargeting's bundled
# OPERATOR2MANO_LEFT does not match our coordinate convention (input ends up
# with fingers along -Z_MANO, optimizer saturates), so we use the RIGHT matrix
# for both hands. This works because the LEFT Shadow Hand URDF is itself a Y-
# mirror of the right URDF, and a real left hand's geometry (thumb on +X side
# of body) is the X-mirror of a right hand's — the two mirrors cancel.
_OP2MANO = np.array(
    [[0, 0, -1],
     [-1, 0, 0],
     [0, 1, 0]],
    dtype=np.float64,
)
OPERATOR2MANO = {"right": _OP2MANO, "left": _OP2MANO}

# Pre-scale human MANO joints before retargeting. Shadow Hand URDF fingers are
# ~19 cm extended, while a typical human index finger reaches ~10 cm from the
# wrist. With the YAML's scaling_factor=1.2, raw input maps to a ~12 cm target —
# the optimizer can only match by curling the robot fingers permanently, so
# open vs fist barely differs. Multiplying the input by 1.5 gives an effective
# 1.8x scale, which produces fully-extended fingers for an open hand and clear
# flex for a fist. Tune per user if their hands are noticeably bigger/smaller.
HUMAN_TO_ROBOT_SCALE = 1.5
from .frame import Frame
from .joints import JOINT_IDS, JOINT_INDEX
from .recorder import JsonlRecorder
from .retarget import (
    ShadowRetargeter,
    fill_actuator_targets,
    viture_positions_to_mano,
)
from .server import run as server_run
from .sim import actuator_names, body_id, load_model, set_body_pose, skeleton_mocap_ids


@dataclass
class LatestFrameSlot:
    """Thread-safe single-slot for the most recent frame."""

    _lock: threading.Lock = threading.Lock()
    _frame: Frame | None = None
    _updated_at: float = 0.0

    def set(self, frame: Frame) -> None:
        with self._lock:
            self._frame = frame
            self._updated_at = time.monotonic()

    def get(self) -> tuple[Frame | None, float]:
        with self._lock:
            return self._frame, self._updated_at


def _server_thread(
    host: str,
    port: int,
    slot: LatestFrameSlot,
    recorder: JsonlRecorder | None,
) -> None:
    def consume(frame: Frame) -> None:
        slot.set(frame)
        if recorder is not None:
            recorder(frame)

    asyncio.run(server_run(host, port, consumer=consume))


# Common per-hand world-space offset added to the (Unity → MuJoCo)-converted
# wrist position. Same anchor for both hands so the user's natural left/right
# hand separation (encoded in head-frame wrist_pos) determines world spacing —
# this lets them clap their hands together or hold them apart and have the
# rendered hands match. Tweak Y to sit forward of where you want the user to
# stand and Z to set "shoulder height" in the world.
WRIST_ANCHOR_COMMON = np.array([0.0, 0.40, 0.30], dtype=np.float64)
WRIST_ANCHOR = {"right": WRIST_ANCHOR_COMMON, "left": WRIST_ANCHOR_COMMON}

# Initial body position to spawn each forearm at startup so the two hands
# don't overlap before the first frame arrives.
INITIAL_FOREARM_POS = {
    "right": np.array([ 0.25, 0.40, 0.30], dtype=np.float64),
    "left":  np.array([-0.25, 0.40, 0.30], dtype=np.float64),
}
# Where the raw skeleton overlay (yellow/cyan dots) is drawn for each hand.
SKELETON_ORIGIN = {
    "right": np.array([ 0.50, 0.40, 0.30], dtype=np.float64),
    "left":  np.array([-0.50, 0.40, 0.30], dtype=np.float64),
}
ACTUATOR_PREFIX = {"right": "rh_A_", "left": "lh_A_"}

# Position of each hand's wrist body in its forearm-local frame (from the
# Menagerie Shadow Hand MJCF). Used to compensate so we set body_pos such
# that the *wrist* lands at the user's wrist position (not the elbow).
WRIST_LOCAL_OFFSET = np.array([0.01, 0.0, 0.21301], dtype=np.float64)


def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two (w,x,y,z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=np.float64)


def _quat_conj_wxyz(q: np.ndarray) -> np.ndarray:
    """Conjugate of a unit quaternion (= inverse rotation)."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _rotate_vec_by_quat_wxyz(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Apply unit quaternion (w,x,y,z) rotation to a 3-vector."""
    w, x, y, z = q
    r = np.array([x, y, z], dtype=np.float64)
    return v + 2.0 * np.cross(r, np.cross(r, v) + w * v)


def _apply_frame_to_sim(
    frame: Frame,
    retargeters: dict[str, ShadowRetargeter],
    mj_actuator_names_: list[str],
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mocap_slots: dict[str, dict[str, int]],
    forearm_body_ids: dict[str, int],
    forearm_default_quat: dict[str, np.ndarray],
    rest_quat_inv_mj: dict[str, np.ndarray | None],
    persistent_ctrl: np.ndarray,
) -> None:
    if not frame.tracked:
        return
    hand = frame.hand
    retargeter = retargeters.get(hand)
    if retargeter is None:
        return

    viture_pos_unity = frame.joint_positions().astype(np.float64)  # (26,3)

    # Translate so the wrist is at origin, then convert from the Unity head
    # frame (+X right, +Y up, +Z forward) to MANO (palm-normal/thumb/fingers).
    # No per-hand mirror here — see OPERATOR2MANO comment.
    wrist_p = viture_pos_unity[JOINT_INDEX["wrist"]]
    op_pos = viture_pos_unity - wrist_p
    mano_pos = (op_pos @ OPERATOR2MANO[hand].T) * HUMAN_TO_ROBOT_SCALE
    mano = viture_positions_to_mano(mano_pos.astype(np.float32))
    angles = retargeter.retarget(mano)
    fill_actuator_targets(
        retargeter.joint_names,
        angles,
        mj_actuator_names_,
        persistent_ctrl,
        prefix=ACTUATOR_PREFIX[hand],
    )
    data.ctrl[:] = persistent_ctrl

    # Drive the forearm body's world pose from the user's wrist pose, anchored
    # at a per-hand "shoulder" so the hand reaches forward from the user.
    wrist_unity = np.array(frame.wrist_pos, dtype=np.float64).reshape(1, 3)
    wrist_target_mj = unity_positions_to_mujoco(wrist_unity)[0] + WRIST_ANCHOR[hand]
    user_quat_mj = unity_quat_to_mujoco_quat(np.array(frame.wrist_rot, dtype=np.float64))
    # Capture rest orientation on the very first frame per hand. Unity's
    # XR Hands subsystem reports a non-identity wrist quat at rest for the
    # left hand (and slight offsets for right). We treat the first frame as
    # the calibration "rest" pose and express every subsequent rotation as a
    # delta from it. Result: rest pose lands on REF; rotations apply on top.
    if rest_quat_inv_mj[hand] is None:
        rest_quat_inv_mj[hand] = _quat_conj_wxyz(user_quat_mj)
    delta_mj = _quat_mul_wxyz(user_quat_mj, rest_quat_inv_mj[hand])
    # body_quat = delta * REF: hand starts at REF orientation in world, then
    # the user's wrist-rotation delta from rest is applied in world frame.
    composed_quat = _quat_mul_wxyz(delta_mj, forearm_default_quat[hand])
    # The forearm's body anchor sits at the elbow, not the wrist; offset
    # backwards along the rotated forearm so the wrist (palm root) lands on
    # wrist_target_mj. Without this, translating the wrist visibly "rotates"
    # the forearm because the elbow leads.
    rotated_offset = _rotate_vec_by_quat_wxyz(WRIST_LOCAL_OFFSET, composed_quat)
    body_pos = wrist_target_mj - rotated_offset
    set_body_pose(model, forearm_body_ids[hand], body_pos, quat_wxyz=composed_quat)
    mujoco.mj_forward(model, data)

    # Skeleton overlay (per hand): raw joints in Unity→MuJoCo axes, anchored
    # next to the rendered Shadow Hand so you can compare raw vs retargeted.
    wrist_pos = viture_pos_unity[JOINT_INDEX["wrist"]]
    rel_pos = viture_pos_unity - wrist_pos
    mj_rel = unity_positions_to_mujoco(rel_pos)
    origin = SKELETON_ORIGIN[hand]
    hand_slots = mocap_slots.get(hand, {})
    for i, name in enumerate(JOINT_IDS):
        slot = hand_slots.get(name)
        if slot is None:
            continue
        data.mocap_pos[slot] = origin + mj_rel[i]


def run(
    host: str = "0.0.0.0",
    port: int = 8765,
    stale_after_s: float = 0.5,
    headless: bool = False,
    duration: float | None = None,
    render_to: str | None = None,
    loop_hz: float = 30.0,
    record_to: str | None = None,
) -> dict:
    from pathlib import Path as _Path

    # Open the WS port first so callers don't wait through model + retargeter
    # init (the latter loads pinocchio twice and can take several seconds).
    slot = LatestFrameSlot()
    recorder = JsonlRecorder(_Path(record_to)) if record_to else None
    srv_thread = threading.Thread(
        target=_server_thread, args=(host, port, slot, recorder), daemon=True
    )
    srv_thread.start()

    model, data = load_model()
    mj_names = actuator_names(model)
    mocap_slots = skeleton_mocap_ids(model)
    forearm_ids = {
        "right": body_id(model, "rh_forearm"),
        "left": body_id(model, "lh_forearm"),
    }
    # Reference forearm orientation per hand: fingers point forward (-Y_world,
    # away from camera), palm faces down (-Z_world), thumbs medial (right
    # toward -X, left toward +X). User wrist rotation composes onto this so
    # "hands at rest, palms down" renders as this reference pose.
    #
    # For Shadow Hand right URDF: local +Z = fingers, +Y = thumb, +X = palm
    # normal. Mapping local→world (palm-normal→-Z, thumb→-X, fingers→-Y)
    # gives rotation matrix [[0,-1,0],[0,0,-1],[-1,0,0]] → quat (w,x,y,z) =
    # (0.5, 0.5, -0.5, -0.5).
    REF_QUAT_RIGHT = np.array([-0.5, 0.5, -0.5, -0.5], dtype=np.float64)
    REF_QUAT_LEFT  = np.array([-0.5, 0.5, -0.5, -0.5], dtype=np.float64)
    forearm_default_quat = {"right": REF_QUAT_RIGHT, "left": REF_QUAT_LEFT}
    # Rest-pose calibration cache (populated on first frame per hand).
    rest_quat_inv_mj: dict[str, np.ndarray | None] = {"right": None, "left": None}
    # Both forearms default to origin in their MJCF; place them apart at
    # startup so they don't self-collide before any tracking data has arrived.
    # Once frames flow, body_pos is overwritten from frame.wrist_pos.
    set_body_pose(model, forearm_ids["right"], INITIAL_FOREARM_POS["right"])
    set_body_pose(model, forearm_ids["left"],  INITIAL_FOREARM_POS["left"])
    mujoco.mj_forward(model, data)
    retargeters = {
        "right": ShadowRetargeter(hand="right"),
        "left": ShadowRetargeter(hand="left"),
    }
    # Cold-start the optimizers with a dummy MANO pose so the first real frame
    # doesn't pay 5+ seconds of solver convergence and starve the loop.
    _dummy_mano = np.zeros((21, 3), dtype=np.float32)
    _dummy_mano[:, 1] = np.linspace(0.0, 0.10, 21)  # arbitrary non-degenerate
    for r in retargeters.values():
        r.retarget(_dummy_mano)
        r.retarget(_dummy_mano)
    persistent_ctrl = np.zeros(model.nu, dtype=np.float64)

    mode = "headless" if headless else "viewer"
    print(f"teleop_app[{mode}]: ws://{host}:{port} ; loop={loop_hz:.0f}Hz")

    loop_dt = 1.0 / loop_hz
    steps_per_loop = max(1, int(round(loop_dt / model.opt.timestep)))

    last_applied_seq: dict[str, int] = {"right": -1, "left": -1}
    applied = 0
    t0 = time.monotonic()

    last_log_time = t0
    log_iters = 0
    log_applies = 0
    # Latest per-hand snapshot for the once-per-second debug line.
    debug_last: dict[str, dict] = {"right": {}, "left": {}}

    def _tick() -> None:
        nonlocal applied, last_log_time, log_iters, log_applies
        log_iters += 1
        frame, updated_at = slot.get()
        if frame is not None and frame.seq != last_applied_seq[frame.hand]:
            if time.monotonic() - updated_at < stale_after_s:
                _apply_frame_to_sim(
                    frame,
                    retargeters,
                    mj_names,
                    model,
                    data,
                    mocap_slots,
                    forearm_ids,
                    forearm_default_quat,
                    rest_quat_inv_mj,
                    persistent_ctrl,
                )
                last_applied_seq[frame.hand] = frame.seq
                applied += 1
                log_applies += 1
                # Capture a snapshot for the periodic debug log.
                jp = frame.joint_positions()
                wp = jp[JOINT_INDEX["wrist"]]
                tip = jp[JOINT_INDEX["index_tip"]]
                # Distance wrist→index_tip indicates fist (small) vs open (large).
                tip_dist = float(np.linalg.norm(tip - wp))
                pref = ACTUATOR_PREFIX[frame.hand]
                ffj0 = persistent_ctrl[mj_names.index(f"{pref}FFJ0")]
                thj1 = persistent_ctrl[mj_names.index(f"{pref}THJ1")]
                debug_last[frame.hand] = {
                    "wpos": wp,
                    "wrot": np.array(frame.wrist_rot, dtype=np.float64),
                    "tip_dist": tip_dist,
                    "FFJ0": float(ffj0),
                    "THJ1": float(thj1),
                }
        for _ in range(steps_per_loop):
            mujoco.mj_step(model, data)
        now = time.monotonic()
        if now - last_log_time >= 1.0:
            def _fmt(d: dict) -> str:
                if not d:
                    return "-"
                wp = d["wpos"]
                wr = d.get("wrot", np.zeros(4))
                return (
                    f"wp=({wp[0]:+.2f},{wp[1]:+.2f},{wp[2]:+.2f}) "
                    f"wr=({wr[0]:+.2f},{wr[1]:+.2f},{wr[2]:+.2f},{wr[3]:+.2f}) "
                    f"tip_d={d['tip_dist']:.3f} FFJ0={d['FFJ0']:+.2f} THJ1={d['THJ1']:+.2f}"
                )
            print(
                f"iters/s={log_iters} applies/s={log_applies} "
                f"R_seq={last_applied_seq['right']} L_seq={last_applied_seq['left']}\n"
                f"  R: {_fmt(debug_last['right'])}\n"
                f"  L: {_fmt(debug_last['left'])}"
            )
            last_log_time = now
            log_iters = 0
            log_applies = 0

    if headless:
        while duration is None or (time.monotonic() - t0) < duration:
            loop_start = time.monotonic()
            _tick()
            sleep_for = loop_dt - (time.monotonic() - loop_start)
            if sleep_for > 0:
                time.sleep(sleep_for)
        if render_to:
            from PIL import Image

            r = mujoco.Renderer(model, height=480, width=640)
            r.update_scene(data)
            Image.fromarray(r.render()).save(render_to)
            print(f"teleop_app: rendered final frame to {render_to}")
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                loop_start = time.monotonic()
                _tick()
                viewer.sync()
                sleep_for = loop_dt - (time.monotonic() - loop_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)

    if recorder is not None:
        recorder.close()

    return {
        "wall_s": round(time.monotonic() - t0, 3),
        "frames_applied": applied,
        "last_seq": last_applied_seq,
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--render-to", default=None)
    p.add_argument("--record-to", default=None, help="append each incoming frame to this JSONL file")
    args = p.parse_args()
    stats = run(
        host=args.host,
        port=args.port,
        headless=args.headless,
        duration=args.duration,
        render_to=args.render_to,
        record_to=args.record_to,
    )
    print(stats)


if __name__ == "__main__":
    main()
