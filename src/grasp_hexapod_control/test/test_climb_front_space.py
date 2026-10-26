import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from build_climb_front import base_points, world_from_base
from climb_mode import ClimbMode
from kinematics import GraspKinematic, Q_STAND

SPACE_CONFIG = ROOT / "src/grasp_hexapod_control/config/climb_front_space.json"
SIDE_CONFIG = ROOT / "src/grasp_hexapod_control/config/climb_compact.json"


def test_front_world_curve_lifts_at_fixed_x_and_terminal_q_matches_final_pose():
    config = json.loads(SPACE_CONFIG.read_text())
    mode = ClimbMode(None)
    mode._validate_config(config)
    stage_index = next(i for i, row in enumerate(config["stages"])
                       if row["name"] == "MID_PLATFORM_285")
    stage = config["stages"][stage_index]
    knots = np.asarray(stage["anchor_knots"], dtype=float)
    assert stage["anchor_curve"] == "piecewise_quintic"
    assert stage["pose_curve"] == "piecewise_quintic"
    assert "active_base_knots_m" not in stage
    assert "active_base_velocities_m_s" not in stage

    mode.config = config
    mode.stage_index = stage_index
    previous_z = knots[0, 2, 2]
    for fraction in (.12, .48, .98):
        time_s = stage["segment_durations_s"][0] * fraction
        mode.phase_time = time_s
        pose, anchors, _ = mode._stage_reference()
        assert pose[0] > stage["pose_start"][0]
        assert anchors[2, 0] == pytest.approx(.080, abs=1e-12)
        assert anchors[5, 0] == pytest.approx(.080, abs=1e-12)
        assert anchors[2, 2] > previous_z
        previous_z = anchors[2, 2]
        assert np.allclose(anchors[[0, 1, 3, 4]], knots[0, [0, 1, 3, 4]], atol=1e-12)
    assert pose[2] >= .204

    pose_knots = np.asarray(stage["pose_knots"], dtype=float)
    knot_times = np.r_[0.0, np.cumsum(stage["segment_durations_s"])]
    for time_s, expected_pose in zip(knot_times, pose_knots):
        mode.phase_time = float(time_s)
        pose_at_knot, _, _ = mode._stage_reference()
        assert np.allclose(pose_at_knot, expected_pose, rtol=0.0, atol=1e-12)

    final = config["stages"][-1]
    q = np.asarray(config["terminal_q_rad"], dtype=float)
    fk_world = (np.column_stack((GraspKinematic().forward_base(q), np.ones(6)))
                @ world_from_base(np.asarray(final["pose_end"], dtype=float)).T)[:, :3]
    assert np.max(np.linalg.norm(
        fk_world - np.asarray(final["anchor_knots"][-1], dtype=float), axis=1
    )) <= .002


def test_side_active_world_piecewise_curve_remains_rejected():
    side = json.loads(SIDE_CONFIG.read_text())
    candidate = copy.deepcopy(side)
    stage = next(row for row in candidate["stages"] if row["active_legs"])
    stage["anchor_curve"] = "piecewise_quintic"
    stage["pose_curve"] = "quintic_full_stage"
    stage.pop("active_base_knots_m", None)
    stage.pop("active_base_velocities_m_s", None)
    with pytest.raises(ValueError, match="invalid compact stage fields: " + stage["name"]):
        ClimbMode(None)._validate_config(candidate)


def test_front_space_config_remains_rejected_for_hardware_execution():
    config = json.loads(SPACE_CONFIG.read_text())
    with pytest.raises(ValueError, match="front compact climbing is simulation-only"):
        ClimbMode(None).enter(Q_STAND, config, hardware_execution=True)


def test_pose_knots_reject_wrong_shape_and_discontinuous_endpoints():
    config = json.loads(SPACE_CONFIG.read_text())
    index = next(i for i, row in enumerate(config["stages"])
                 if row["name"] == "MID_PLATFORM_285")
    stage_name = config["stages"][index]["name"]
    mutations = (
        lambda row: row.__setitem__("pose_knots", row["pose_knots"][:-1]),
        lambda row: row["pose_knots"][0].__setitem__(0, row["pose_knots"][0][0] + .001),
        lambda row: row["pose_knots"][-1].__setitem__(2, row["pose_knots"][-1][2] + .001),
    )
    for mutate in mutations:
        candidate = copy.deepcopy(config)
        mutate(candidate["stages"][index])
        with pytest.raises(ValueError, match="invalid compact stage fields: " + stage_name):
            ClimbMode(None)._validate_config(candidate)
