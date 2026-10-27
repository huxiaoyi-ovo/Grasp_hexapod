import copy
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from climb_mode import ClimbMode
from control import GraspController
from kinematics import JOINT_VELOCITY_LIMIT, Q_STAND
from tools.optimize_climb_front import _geometry_sha256

ROOT = Path(__file__).resolve().parents[3]
FAST = ROOT / "src/grasp_hexapod_control/config/climb_front_fast.json"
SIDE = ROOT / "src/grasp_hexapod_control/config/climb_compact.json"
EVIDENCE = ROOT / "src/docs/evidence/climb_front_fast_20260923"
STRUCTURAL = EVIDENCE / "structural_candidate_41.json"
FAST_AUDIT = EVIDENCE / "cpu_fast_41_audit.json"
TERMINAL_EVIDENCE = ROOT / "src/docs/evidence/climb_front_terminal_20260924"
BEFORE_TERMINAL = TERMINAL_EVIDENCE / "before_climb_front_fast.json"


def load_fast():
    return json.loads(FAST.read_text())


def test_fast_config_has_only_frozen_merges_and_rebuilt_boundaries():
    config = load_fast()
    ClimbMode(None)._validate_config(config)
    assert config["climb_orientation"] == "front"
    assert config["simulation_joint_velocity_limit_rad_s"] == pytest.approx(6.0)
    assert config["stage_count"] == 41
    assert "surface_receipt" not in config
    assert config["visual_validation_deferred_for_sim_finish"] == [
        stage["name"] for stage in config["stages"]
    ]
    mapped = config["optimization"]["stage_map"]
    assert [item["candidate"] for item in mapped] == [
        "IJ_MID_DIRECT", "LM_MID_RELAND", "OP_MID_BODY_LIFT",
        "R_FRONT_BODY_LIFT", "BODY_B_FRONT_INWARD", "RB_RESET_BODY_LIFT", "AE_AF_BACK_PAIR",
        "AGAB_BACK_PAIR", "AGCD_BACK_PAIR",
    ]
    for stage in config["stages"]:
        active = np.asarray(stage["active_legs"], dtype=int)
        if len(active):
            assert np.asarray(stage["active_base_knots_m"]).shape == (
                len(stage["anchor_knots"]), len(active), 3
            )


def test_uniform_retime_matches_receipt_and_preserves_geometry():
    # This receipt belongs to the immutable pre-terminal fast candidate.
    fast = json.loads(BEFORE_TERMINAL.read_text())
    structural = json.loads(STRUCTURAL.read_text())
    audit = json.loads(FAST_AUDIT.read_text())
    assert audit["state"] == "DONE"
    assert audit["ticks"] == 1310
    assert audit["candidate_config_sha256"] == __import__("hashlib").sha256(BEFORE_TERMINAL.read_bytes()).hexdigest()
    assert audit["geometry_sha256"] == _geometry_sha256(structural)

    geometry_fields = (
        "pose_start", "pose_end", "anchor_knots", "active_base_knots_m",
        "active_base_velocities_m_s",
    )
    assert len(fast["stages"]) == len(structural["stages"]) == 41
    metadata = {item["stage"]: item for item in audit["retime_metadata"]}
    for fast_stage, source_stage in zip(fast["stages"], structural["stages"]):
        assert fast_stage["name"] == source_stage["name"]
        for field in geometry_fields:
            assert fast_stage.get(field) == source_stage.get(field)
        timing = metadata[fast_stage["name"]]
        assert fast_stage["segment_durations_s"] == timing["retimed_durations_s"]
        scales = np.asarray(fast_stage["segment_durations_s"]) / np.asarray(timing["original_durations_s"])
        assert np.allclose(scales, scales[0], atol=1e-12, rtol=0.0)

    for stage_name in ("BODY_B_FRONT_INWARD", "AH_BACK_LOW_FIRST_SEGMENT"):
        fast_mode, structural_mode = ClimbMode(None), ClimbMode(None)
        fast_mode.config, structural_mode.config = fast, structural
        fast_mode.stage_index = next(i for i, s in enumerate(fast["stages"])
                                     if s["name"] == stage_name)
        structural_mode.stage_index = fast_mode.stage_index
        fast_stage, source_stage = fast["stages"][fast_mode.stage_index], structural["stages"][fast_mode.stage_index]
        for normalized_phase in (0.0, .15, .5, .85, 1.0):
            fast_mode.phase_time = normalized_phase * sum(fast_stage["segment_durations_s"])
            structural_mode.phase_time = normalized_phase * sum(source_stage["segment_durations_s"])
            fast_pose, fast_anchors, _ = fast_mode._stage_reference()
            source_pose, source_anchors, _ = structural_mode._stage_reference()
            assert np.allclose(fast_pose, source_pose, atol=1e-12, rtol=0.0)
            assert np.allclose(fast_anchors, source_anchors, atol=1e-12, rtol=0.0)

    merge = next(stage for stage in fast["optimization"]["stage_map"]
                 if stage["candidate"] == "BODY_B_FRONT_INWARD")
    assert merge["source_stages"] == ["BODY_B", "FRONT_INWARD"]
    assert merge["active_legs"] == [1, 4]
    assert merge["clearance_m"] == pytest.approx(.020)
    assert merge["segment_durations_s"] == pytest.approx([.525, .725, .2375])
    merged_stage = next(stage for stage in fast["stages"] if stage["name"] == "BODY_B_FRONT_INWARD")
    assert merged_stage["pose_start"] == next(stage for stage in structural["stages"]
                                               if stage["name"] == "BODY_B_FRONT_INWARD")["pose_start"]
    assert merged_stage["pose_end"] == next(stage for stage in structural["stages"]
                                             if stage["name"] == "BODY_B_FRONT_INWARD")["pose_end"]


def test_terminal_diagonal_descent_restores_air_knots_and_only_moves_rear_landing():
    config = load_fast()
    before = json.loads(BEFORE_TERMINAL.read_text())
    ClimbMode(None)._validate_config(config)
    assert config["stage_count"] == before["stage_count"] == 41
    old_stages, new_stages = before["stages"], config["stages"]
    ah_index = next(i for i, stage in enumerate(old_stages)
                    if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    assert old_stages[:ah_index] == new_stages[:ah_index]
    old_ah, ah = old_stages[ah_index], new_stages[ah_index]
    old_hold, hold = old_stages[-1], new_stages[-1]
    for field in ("pose_start", "pose_end", "segment_durations_s", "settle_s",
                  "active_legs", "pose_curve", "anchor_curve"):
        assert ah[field] == old_ah[field]
    old_anchors = np.asarray(old_ah["anchor_knots"], dtype=float)
    anchors = np.asarray(ah["anchor_knots"], dtype=float)
    assert np.array_equal(anchors[1:3, [0, 3]], old_anchors[1:3, [0, 3]])
    assert np.array_equal(anchors[:, [1, 2, 4, 5]], old_anchors[:, [1, 2, 4, 5]])
    assert anchors[2, 0, 0] == pytest.approx(.19)
    assert anchors[2, 3, 0] == pytest.approx(.19)
    assert anchors[3, [0, 3], 0] == pytest.approx([.205, .205])
    assert config["terminal_refinement"]["rear_descent"] == "diagonal_forward_descent"
    assert config["terminal_refinement"]["rear_descent_dx_m"] == pytest.approx(.015)
    assert config["terminal_refinement"]["rear_actual_apex_world_z_m"] == pytest.approx(
        .16575113102445194
    )
    final_anchors = np.asarray(hold["anchor_knots"], dtype=float)
    assert np.allclose(final_anchors[:, [0, 3]],
                       np.broadcast_to(anchors[3, [0, 3]], final_anchors[:, [0, 3]].shape),
                       atol=1e-12, rtol=0.0)
    assert old_hold["pose_start"] == hold["pose_start"]
    assert old_hold["pose_end"] == hold["pose_end"]
    assert config["terminal_refinement"]["terminal_ik_residual_m"] < 1e-5


def test_terminal_patch_check_rejects_old_shared_edge_and_accepts_new_rear_anchors():
    config = load_fast()
    from build_climb_front import MESH, TRANSLATION, load_triangles

    audit_path = TERMINAL_EVIDENCE / "front_terminal_audit.py"
    spec = importlib.util.spec_from_file_location("front_terminal_audit", audit_path)
    audit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(audit)
    triangles = load_triangles(MESH) + TRANSLATION

    old_edge_feet = (
        (.175291609, .027767099, .147628544),
        (.176119995, -.165887699, .147927417),
    )
    for foot in old_edge_feet:
        nearest = audit.inspect_nearest_patch(triangles, foot)
        assert nearest
        assert not any(item["inside_broad_patch_with_10mm_margin"] for item in nearest)
        assert min(item["clearance_inward_from_broad_patch_edge_m"]
                   for item in nearest) < .010

    hold = next(stage for stage in config["stages"] if stage["name"] == "FINAL_HOLD")
    anchors = np.asarray(hold["anchor_knots"][-1], dtype=float)
    for leg in (0, 3):
        nearest = audit.inspect_nearest_patch(triangles, anchors[leg])
        assert any(item["inside_broad_patch_with_10mm_margin"] for item in nearest)
        assert min(item["clearance_inward_from_broad_patch_edge_m"]
                   for item in nearest) >= .010


def test_front_simulation_cap_is_fail_closed_and_does_not_pollute_side_session():
    fast = load_fast()
    for bad in (True, 0.0, 8.01):
        invalid = copy.deepcopy(fast)
        invalid["simulation_joint_velocity_limit_rad_s"] = bad
        with pytest.raises(ValueError, match="velocity cap"):
            ClimbMode(None)._validate_config(invalid)

    controller = GraspController(1.0 / 30.0, False, False)
    side = json.loads(SIDE.read_text())
    controller.enter_climb(Q_STAND, side, hardware_execution=False)
    original = controller.climb_mode.config
    with pytest.raises(ValueError, match="simulation-only"):
        controller.enter_climb(Q_STAND, fast, hardware_execution=True)
    assert controller.climb_mode.config is original
    assert controller.climb_mode.hardware_execution is False
    assert controller.climb_mode.state == ClimbMode.RUNNING

    # Bypass only the collision proxy to expose the step guard.  The retained
    # side session must still use its shared 4 rad/s cap, not fast's 6 rad/s.
    controller.collision_guard = lambda candidate, current: candidate
    controller.foot_desired_base[:] = controller.foot_init_base + .5
    q_next = controller.cal_joint_poses(Q_STAND)
    assert np.max(np.abs(q_next - Q_STAND)) == pytest.approx(
        JOINT_VELOCITY_LIMIT * controller.dt
    )

    front = GraspController(1.0 / 30.0, False, False)
    front.enter_climb(Q_STAND, fast, hardware_execution=False)
    front.collision_guard = lambda candidate, current: candidate
    front.foot_desired_base[:] = front.foot_init_base + .5
    q_front = front.cal_joint_poses(Q_STAND)
    assert np.max(np.abs(q_front - Q_STAND)) > np.max(JOINT_VELOCITY_LIMIT) * front.dt
    assert np.max(np.abs(q_front - Q_STAND)) <= 6.0 * front.dt + 1e-12

    approach = GraspController(1.0 / 30.0, False, False)
    approach.collision_guard = lambda candidate, current: candidate
    approach.foot_desired_base[:] = approach.foot_init_base + .5
    q_approach = approach.cal_joint_poses(Q_STAND)
    assert np.max(np.abs(q_approach - Q_STAND)) == pytest.approx(
        np.max(JOINT_VELOCITY_LIMIT) * approach.dt
    )
