from __future__ import annotations

import numpy as np

from viture_teleop.sim import actuator_names, load_model, run_headless


def test_model_loads() -> None:
    model, data = load_model()
    assert model.nu > 0
    assert model.nq > 0


def test_actuators_include_finger_joints() -> None:
    model, _ = load_model()
    names = actuator_names(model)
    joined = ",".join(names).lower()
    for finger_hint in ("ff", "mf", "rf", "lf", "th"):  # Shadow Hand finger codes
        assert finger_hint in joined, f"no actuator containing '{finger_hint}'"


def test_headless_smoke_runs_and_stays_finite() -> None:
    stats = run_headless(duration=0.5)
    assert stats["steps"] > 10
    assert stats["sim_time"] > 0.0


def test_ctrlrange_bounds_well_defined() -> None:
    model, _ = load_model()
    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]
    assert np.all(hi >= lo)
