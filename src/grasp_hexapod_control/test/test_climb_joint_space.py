"""Joint-space swing segments (stage.joint_space) for compact climb stages.

Baseline: config/climb_front_v16_1.json, stage MID_PLATFORM_285, leg 5 (rm),
knots 1 -> 2, joint knots = exact IK of the Cartesian knots.
Mirrors test_climb_joint_space.cpp.
"""

import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path[:0] = [str(SCRIPTS)]

from climb_mode import ClimbMode
from control import GraspController
from kinematics import GraspKinematic, Q_STAND
from utils.climb import select_compact_climb_side

CONFIG = Path(__file__).resolve().parents[1] / "config/climb_front_v16_1.json"
STAGE = "MID_PLATFORM_285"
LEG, FROM, TO = 5, 1, 2


def exact_ik(leg, base_target):
    """Iterative DLS IK of one leg from the stand posture (hip frame via base_to_hip)."""

    kin = GraspKinematic()
    base = np.zeros((6, 3))
    base[leg] = base_target
    hip_target = kin.base_to_hip(base)[leg]
    q = Q_STAND[leg].copy()
    for _ in range(500):
        error = hip_target - kin.forward_leg(leg, q)
        if np.linalg.norm(error) < 1e-14:
            break
        jac = kin.jacobian_leg(leg, q)
        q = q + jac.T @ np.linalg.solve(jac @ jac.T + 1e-10 * np.eye(3), error)
    return q


def stage_of(doc):
    return next(s for s in doc["stages"] if s["name"] == STAGE)


def make_doc(to_offset=(0.0, 0.0, 0.0)):
    doc = json.loads(CONFIG.read_text())
    stage = stage_of(doc)
    slot = stage["active_legs"].index(LEG)
    for knot in (FROM, TO):
        stage["active_base_velocities_m_s"][knot][slot] = [0.0, 0.0, 0.0]
    knots = np.asarray(stage["active_base_knots_m"])
    q_from = exact_ik(LEG, knots[FROM, slot])
    q_to = exact_ik(LEG, knots[TO, slot] + np.asarray(to_offset))
    stage["joint_space"] = {
        "legs": [LEG],
        "from_knot": FROM,
        "to_knot": TO,
        "q_knots_rad": [[q_from.tolist()], [q_to.tolist()]],
    }
    return doc


def validate(doc):
    ClimbMode(None)._validate_config(doc)


def test_accepts_exact_ik_knots_and_zero_velocities():
    doc = make_doc()
    validate(doc)
    doc2 = make_doc((0.0005, 0.0, 0.0))
    stage_of(doc2)["joint_space"]["q_velocities_rad_s"] = [[[0.0] * 3], [[0.0] * 3]]
    validate(doc2)


def test_configs_without_joint_space_still_validate():
    validate(json.loads(CONFIG.read_text()))


@pytest.mark.parametrize("mutate,needle", [
    (lambda js, st: js.update(legs=[0]), "not an active leg"),
    (lambda js, st: js.update(legs=[LEG, LEG]), "duplicate leg"),
    (lambda js, st: js["q_knots_rad"].append(js["q_knots_rad"][1]), "q_knots_rad shape mismatch"),
    (lambda js, st: js.update(from_knot=2), "knot range"),
    (lambda js, st: js.update(to_knot=99), "knot range"),
    (lambda js, st: js["q_knots_rad"][1][0].__setitem__(2, 2.5), "outside joint limits"),
    (lambda js, st: js.update(q_velocities_rad_s=[[[0.0] * 3], [[0.0, 0.0, 0.1]]]),
     "end velocities must be zero"),
    (lambda js, st: st["active_base_velocities_m_s"][FROM].__setitem__(
        st["active_legs"].index(LEG), [0.0, 0.01, 0.0]),
     "base velocities must be zero"),
])
def test_rejects_invalid_joint_space(mutate, needle):
    doc = make_doc()
    stage = stage_of(doc)
    mutate(stage["joint_space"], stage)
    with pytest.raises(ValueError, match="invalid joint_space") as error:
        validate(doc)
    assert needle in str(error.value)


def test_rejects_five_millimetre_continuity_error():
    with pytest.raises(ValueError, match="boundary FK does not match"):
        validate(make_doc((0.005, 0.0, 0.0)))


def test_rejects_non_base_piecewise_stage():
    doc = make_doc()
    stage = stage_of(doc)
    stage["anchor_curve"] = "piecewise_quintic"
    del stage["active_base_knots_m"], stage["active_base_velocities_m_s"]
    with pytest.raises(ValueError, match="requires piecewise_base_quintic"):
        validate(doc)


def test_right_mirror_rejects_joint_space():
    side = json.loads((CONFIG.parent / "climb_compact.json").read_text())
    side["stages"][1]["joint_space"] = {"legs": [0]}
    with pytest.raises(ValueError, match="no right-side mirror"):
        select_compact_climb_side(side, "right")


def replay(doc):
    controller = GraspController(1.0 / 30.0, False, False)
    config = select_compact_climb_side(doc, "left")
    q = Q_STAND.copy()
    controller.enter_climb(q, config, 0, None, False)
    ticks = []
    while controller.climb_mode.state == ClimbMode.RUNNING and len(ticks) < 40000:
        q = controller.update(q.copy(), np.zeros(4)).copy()
        override = controller.joint_override[LEG]
        for other, value in enumerate(controller.joint_override):
            if other != LEG:
                assert value is None
        ticks.append((
            q.copy(), controller.climb_mode.stage_index, controller.climb_mode.phase_time,
            None if override is None else np.array(override),
        ))
    final = controller.climb_mode.state
    controller.abort_climb()
    assert all(value is None for value in controller.joint_override)
    return ticks, final


def reference_q(js, duration, elapsed):
    q0 = np.asarray(js["q_knots_rad"][0][0])
    q1 = np.asarray(js["q_knots_rad"][1][0])
    u = elapsed / duration
    w = u ** 3 * (10.0 - 15.0 * u + 6.0 * u ** 2)
    return q0 * (1.0 - w) + q1 * w


def test_runtime_tracks_joint_knots_and_leaves_other_legs_unchanged():
    with_joint = make_doc()
    without = copy.deepcopy(with_joint)
    del stage_of(without)["joint_space"]
    a, state_a = replay(with_joint)
    b, state_b = replay(without)
    assert state_a == state_b == ClimbMode.DONE
    assert len(a) == len(b)

    stage = stage_of(with_joint)
    stage_index = [s["name"] for s in with_joint["stages"]].index(STAGE)
    cumulative = np.cumsum(stage["segment_durations_s"])
    t_from, t_to = float(cumulative[FROM - 1]), float(cumulative[TO - 1])
    duration = stage["segment_durations_s"][FROM]
    in_range = 0
    worst_joint = worst_other = 0.0
    for tick, ((qa, sa, pa, oa), (qb, _, _, _)) in enumerate(zip(a, b)):
        expected_in_range = sa == stage_index and t_from <= pa <= t_to
        assert (oa is not None) == expected_in_range, tick
        if expected_in_range:
            in_range += 1
            ref = reference_q(stage["joint_space"], duration, pa - t_from)
            worst_joint = max(worst_joint, float(np.max(np.abs(qa[LEG] - ref))))
            assert np.max(np.abs(qa[LEG] - ref)) <= 1e-9, tick
            assert np.max(np.abs(oa - ref)) <= 1e-9, tick
        others = [leg for leg in range(6) if leg != LEG]
        diff = float(np.max(np.abs(qa[others] - qb[others])))
        worst_other = max(worst_other, diff)
        assert diff <= 1e-12, tick
    assert in_range > 20
    print(f"joint-space ticks={len(a)} in_range={in_range} "
          f"max|q_des-q_ref|={worst_joint:.3e} max_other_leg_diff={worst_other:.3e}")
