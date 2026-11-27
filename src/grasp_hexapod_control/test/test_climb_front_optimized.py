import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from build_climb_front import (
    MESH, TRANSLATION, base_points, foot_sphere_signed_gap, load_triangles,
    solve_ik, world_from_base,
)
from climb_mode import ClimbMode
from control import GraspController
from kinematics import FOOT_RADIUS, GraspKinematic, JOINT_LOWER, JOINT_UPPER, Q_CLIMB_PREP
from audit_climb_front_landing import PatchClassifier, gap_in_interval, rectangle_patch_check, same_component

CONFIG = ROOT / "src/grasp_hexapod_control/config/climb_front_optimized.json"
SOURCE = ROOT / "src/grasp_hexapod_control/config/climb_front_space.json"
SIDE = ROOT / "src/grasp_hexapod_control/config/climb_compact.json"
V6_CONFIG = ROOT / "src/docs/evidence/climb_front_rework_20261026/v6/climb_front_optimized_v6.json"
V8_CONFIG = ROOT / "src/docs/evidence/climb_front_rework_20261026/v8/climb_front_optimized_v8.json"
DT = 1.0 / 30.0


def load_config():
    return json.loads(CONFIG.read_text())


def stage_index(config, name):
    return next(i for i, stage in enumerate(config["stages"]) if stage["name"] == name)


def entry_q(stage):
    q, residual = solve_ik(
        GraspKinematic(), np.asarray(stage["pose_start"], dtype=float),
        np.asarray(stage["anchor_knots"][0], dtype=float), Q_CLIMB_PREP,
        tolerance_m=1e-5,
    )
    assert residual <= 1e-5
    return q


def apply_current_reference(mode):
    base, anchors, _ = mode._stage_reference()
    mode._apply_reference(base, anchors)


def test_v6_body_relative_stages_bind_endpoints_and_hold_fixed_supports():
    config = load_config()
    ClimbMode(None)._validate_config(config)
    assert config["front_optimized_source"]["revision"] == 10
    assert config["settle_gate"]["preview_time_only_stage_advance"] is False
    assert config["settle_gate"]["tracking_errors_diagnostic_only"] is False
    assert config["settle_gate"]["max_foot_target_error_m"] == pytest.approx(.020)
    assert all(stage["settle_s"] >= .25 for stage in config["stages"])

    mode = ClimbMode(None)
    for name in ("MID_PLATFORM_285", "RB_DIRECT_PLATFORM_225", "LB_PLATFORM_225"):
        index = stage_index(config, name)
        stage = config["stages"][index]
        assert stage["anchor_curve"] == "piecewise_base_quintic"
        assert stage["pose_curve"] == "piecewise_quintic"
        active = np.asarray(stage["active_legs"], dtype=int)
        fixed = np.asarray([leg for leg in range(6) if leg not in active], dtype=int)
        base_knots = np.asarray(stage["active_base_knots_m"], dtype=float)
        anchors = np.asarray(stage["anchor_knots"], dtype=float)
        assert base_knots.shape == (len(anchors), len(active), 3)
        assert np.allclose(anchors[:, fixed], anchors[0, fixed], atol=1e-12, rtol=0.0)
        for knot, pose, base in zip(anchors, stage["pose_knots"], base_knots):
            expected = (np.column_stack((base, np.ones(len(active))))
                        @ world_from_base(np.asarray(pose, dtype=float)).T)[:, :3]
            assert np.allclose(knot[active], expected, atol=1e-9, rtol=0.0)
        mode.config, mode.stage_index = config, index
        mode.phase_time = .5 * stage["segment_durations_s"][0]
        _, composed, _ = mode._stage_reference()
        assert np.allclose(composed[fixed], anchors[0, fixed], atol=1e-12, rtol=0.0)
        assert not np.allclose(composed[active], anchors[0, active], atol=1e-9, rtol=0.0)


def test_v5_candidate_continuity_source_fingerprints_and_terminal_goal():
    config, source = load_config(), json.loads(SOURCE.read_text())
    assert len(config["stages"]) == 19
    for candidate, original in ((0, 0), (1, 1), *zip(range(3, 10), range(4, 11))):
        actual = copy.deepcopy(config["stages"][candidate]); actual.pop("settle_s")
        expected = copy.deepcopy(source["stages"][original]); expected.pop("settle_s")
        assert actual == expected
    for previous, current in zip(config["stages"], config["stages"][1:]):
        assert np.allclose(previous["pose_end"], current["pose_start"], atol=1e-12, rtol=0.0)
        assert np.allclose(previous["anchor_knots"][-1], current["anchor_knots"][0], atol=1e-12, rtol=0.0)
    final = config["stages"][-1]
    assert np.allclose(final["pose_end"][:3], [.450, -.063, .300], atol=1e-12, rtol=0.0)
    assert [stage["active_legs"] for stage in config["stages"][15:18]] == [[4, 2], [0, 1, 5], [3]]
    assert config["stages"][16]["name"] == "TRIPOD_LB_LF_RM_ADVANCE"
    assert config["stages"][17]["name"] == "RB_FINAL_ADVANCE"
    assert np.allclose(np.asarray(config["stages"][16]["anchor_knots"])[0, [2, 3, 4]],
                       np.asarray(config["stages"][16]["anchor_knots"])[-1, [2, 3, 4]])
    assert np.allclose(np.asarray(config["stages"][17]["anchor_knots"])[:, 0],
                       np.asarray(config["stages"][17]["anchor_knots"])[0, 0])
    q = np.asarray(config["terminal_q_rad"], dtype=float)
    desired = base_points(final["pose_end"], final["anchor_knots"][-1])
    assert np.max(np.linalg.norm(GraspKinematic().forward_base(q) - desired, axis=1)) <= .002


def test_v6_mid_departure_orders_clearance_before_high_preadvance():
    config = load_config()
    index = stage_index(config, "MID_PLATFORM_285")
    stage = config["stages"][index]
    active = np.asarray(stage["active_legs"], dtype=int)
    anchors = np.asarray(stage["anchor_knots"], dtype=float)
    poses = np.asarray(stage["pose_knots"], dtype=float)
    base_knots = np.asarray(stage["active_base_knots_m"], dtype=float)
    velocities = np.asarray(stage["active_base_velocities_m_s"], dtype=float)
    assert len(config["stages"]) == 19
    assert stage["segment_durations_s"] == pytest.approx([.60, .60, .75, .50, .70, .40])
    assert poses[1, 0] == pytest.approx(poses[0, 0])
    assert np.allclose(anchors[1, active, 0], anchors[0, active, 0] - .020, atol=1e-12, rtol=0.0)
    assert np.allclose(anchors[1, active, 1], anchors[0, active, 1], atol=1e-12, rtol=0.0)
    assert np.allclose(anchors[1, active, 2], .095, atol=1e-12, rtol=0.0)
    for pose, base, knot in zip(poses, base_knots, anchors):
        expected = (np.column_stack((base, np.ones(len(active))))
                    @ world_from_base(pose).T)[:, :3]
        assert np.allclose(knot[active], expected, atol=1e-9, rtol=0.0)
    departure_world_velocity = world_from_base(poses[1])[:3, :3] @ velocities[1].T
    assert np.allclose(departure_world_velocity[:2], 0.0, atol=1e-12, rtol=0.0)
    assert np.allclose(departure_world_velocity[2], .050, atol=1e-12, rtol=0.0)

    mode = ClimbMode(None)
    mode.config, mode.stage_index = config, index
    triangles = load_triangles(MESH) + TRANSLATION
    gaps = []
    for phase_time in np.arange(0.0, 1.20 + 1e-12, DT):
        mode.phase_time = float(phase_time)
        pose, world_anchors, _ = mode._stage_reference()
        if phase_time <= .60 + 1e-12:
            assert pose[0] <= poses[0, 0] + 1e-12
        gaps.append(min(foot_sphere_signed_gap(triangles, world_anchors[leg]) for leg in active))
    assert min(gaps) >= .015


def test_v7_lb_polar_path_keeps_annulus_radius_above_v6_crossing():
    config = load_config()
    v6 = json.loads(V6_CONFIG.read_text())
    kin = GraspKinematic()

    def sampled_radii(candidate):
        index = stage_index(candidate, "LB_PLATFORM_225")
        stage = candidate["stages"][index]
        leg = stage["active_legs"][0]
        mode = ClimbMode(None)
        mode.config, mode.stage_index = candidate, index
        radii = []
        for phase_time in np.arange(0.0, sum(stage["segment_durations_s"]) + 1e-12, DT):
            mode.phase_time = float(phase_time)
            pose, anchors, _ = mode._stage_reference()
            point_base = base_points(pose, np.asarray(anchors, dtype=float)[[leg]])[0]
            point_hip = (kin.hip_from_base[leg] @ np.r_[point_base, 1.0])[:3]
            rho = np.linalg.norm(point_hip[:2])
            radii.append(float(np.hypot(rho - .05236, point_hip[2])))
        return radii

    stage = config["stages"][stage_index(config, "LB_PLATFORM_225")]
    velocities = np.asarray(stage["active_base_velocities_m_s"], dtype=float)
    assert stage["segment_durations_s"] == pytest.approx([1.10, .55, .45, .45, .55, .40])
    assert np.allclose(velocities[[0, 5, 6]], 0.0, atol=1e-12, rtol=0.0)
    assert np.all(np.linalg.norm(velocities[1:5], axis=2) <= .040 + 1e-12)
    assert min(sampled_radii(v6)) < .090
    assert min(sampled_radii(config)) >= .090


def test_v8_final_lf_target_stays_on_one_flat_upper_component_under_forward_lag():
    """Screen the physical flat support component, not arbitrary STL triangles."""
    config = json.loads(V8_CONFIG.read_text())
    goal = np.asarray(config["stages"][-1]["anchor_knots"][-1], dtype=float)[1]
    assert np.allclose(goal, [.550, .025, .1964999976], atol=1e-9, rtol=0.0)
    triangles = load_triangles(MESH) + TRANSLATION
    old_screen = rectangle_patch_check(PatchClassifier(triangles), goal,
                                       sphere_gap_m=foot_sphere_signed_gap(triangles, goal))
    assert not old_screen["safe_patch"]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    flat = np.flatnonzero(np.abs(normals[:, 2]) > .999)
    assert len(flat)

    # Connected coplanar faces define the physical patch; only unshared edges
    # of that component contribute to the planar outer boundary.
    rounded = np.round(triangles[flat], decimals=9)
    edge_map = {}
    for local_index, triangle in enumerate(rounded):
        for first, second in ((0, 1), (1, 2), (2, 0)):
            edge = tuple(sorted((tuple(triangle[first]), tuple(triangle[second]))))
            edge_map.setdefault(edge, []).append(local_index)
    neighbours = [set() for _ in flat]
    for owners in edge_map.values():
        if len(owners) == 2:
            first, second = owners
            neighbours[first].add(second); neighbours[second].add(first)

    def containing_faces(point_xy):
        matches = []
        for local_index, triangle in enumerate(triangles[flat]):
            origin = triangle[0]
            matrix = np.column_stack((triangle[1, :2] - origin[:2], triangle[2, :2] - origin[:2]))
            determinant = np.linalg.det(matrix)
            if abs(determinant) <= 1e-12:
                continue
            uv = np.linalg.solve(matrix, point_xy - origin[:2])
            if np.all(uv >= -1e-9) and uv.sum() <= 1.0 + 1e-9:
                matches.append(local_index)
        return matches

    def point_segment_distance(point, start, end):
        direction = end - start
        fraction = np.clip(np.dot(point - start, direction) / np.dot(direction, direction), 0.0, 1.0)
        return float(np.linalg.norm(point - (start + fraction * direction)))

    def component_and_inset(point):
        point_xy = np.asarray(point[:2], dtype=float)
        seeds = containing_faces(point_xy)
        assert seeds
        component = set(seeds[:1]); pending = list(component)
        while pending:
            current = pending.pop()
            for neighbour in neighbours[current]:
                if neighbour not in component:
                    component.add(neighbour); pending.append(neighbour)
        boundary = []
        for edge, owners in edge_map.items():
            if len(owners) == 1 and owners[0] in component:
                boundary.append(np.asarray(edge, dtype=float)[:, :2])
        assert boundary
        return component, min(point_segment_distance(point_xy, edge[0], edge[1]) for edge in boundary)

    lagged = goal.copy(); lagged[0] -= .040
    component, inset = component_and_inset(goal)
    lag_component, lag_inset = component_and_inset(lagged)
    assert component == lag_component
    assert inset >= .015
    assert lag_inset >= .015
    assert foot_sphere_signed_gap(triangles, goal) >= -.0005
    assert foot_sphere_signed_gap(triangles, lagged) >= -.0005
    # Both centres are directly over the flat patch at the physical contact height.
    for point in (goal, lagged):
        surface = point.copy(); surface[2] -= FOOT_RADIUS
        assert containing_faces(surface[:2])
        assert surface[2] == pytest.approx(.1899999976, abs=1e-9)


def test_v9_terminal_regions_are_robust_and_match_surface_axis_diagnostics():
    config = load_config()
    assert config["front_optimized_source"]["revision"] == 10
    expected_xy = np.array(((.280, .105), (.555, .085), (.425, .100),
                            (.335, -.165), (.618, -.15998477371966988), (.400, -.275)))
    final = np.asarray(config["stages"][-1]["anchor_knots"][-1], dtype=float)
    assert np.allclose(final[:, :2], expected_xy, atol=1e-12, rtol=0.0)
    triangles = load_triangles(MESH) + TRANSLATION
    classifier = PatchClassifier(triangles)
    details = [rectangle_patch_check(classifier, point,
                                     sphere_gap_m=foot_sphere_signed_gap(triangles, point))
               for point in final]
    assert all(item["safe_patch"] for item in details)
    assert all(item["face_normal_z"] >= .90 for item in details)
    assert all(item["component_area_m2"] >= .020 for item in details)
    assert all(item["rectangle_boundary_distance_m"] > .015 for item in details)
    q = np.asarray(config["terminal_q_rad"], dtype=float)
    axes = (world_from_base(config["stages"][-1]["pose_end"])[:3, :3]
            @ GraspKinematic().terminal_axes_base(q).T).T
    normals = np.asarray([item["face_normal_world"] for item in details])
    degrees = np.degrees(np.arccos(np.clip(np.abs(np.einsum("ij,ij->i", axes, normals)), -1., 1.)))
    assert np.all(degrees <= 35.0)
    assert np.min(np.minimum(q - JOINT_LOWER, JOINT_UPPER - q)) > .02


def test_landing_audit_uses_component_identity_positive_normals_and_signed_gap_bounds():
    flat = np.array(([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
                     [[1., 0., 0.], [1., 1., 0.], [0., 1., 0.]],
                     [[2., 0., 0.], [2., 1., 0.], [3., 0., 0.]]))
    classifier = PatchClassifier(flat)
    first = classifier.describe([.20, .20, .0065])
    second = classifier.describe([.75, .75, .0065])
    assert first["nearest_triangle_index"] != second["nearest_triangle_index"]
    assert same_component(first, second)
    assert not gap_in_interval(.003871)
    downward = rectangle_patch_check(classifier, [2.20, .20, .0065], extra_xy_m=0.0)
    assert downward["face_normal_z"] < 0.0
    assert not downward["safe_patch"]


def test_false_flag_front_checkpoint_holds_lag_and_times_out_without_hardware():
    config = load_config()
    index = stage_index(config, "MID_PLATFORM_285")
    stage = config["stages"][index]
    q_start = entry_q(stage)
    controller = GraspController(DT, enable_link_collision_check=False, climb_timeout_uses_wall_time=True)
    controller.enter_climb(q_start, config, index, index, hardware_execution=False)
    mode = controller.climb_mode
    mode.phase_time = stage["segment_durations_s"][0]
    apply_current_reference(mode)
    controller.update(q_start + .5, np.zeros(4))
    assert mode.state == ClimbMode.RUNNING
    assert mode.phase_time == pytest.approx(stage["segment_durations_s"][0])
    assert mode.last_phase_hold
    assert mode.stage_elapsed_time == pytest.approx(DT)

    mode.phase_time = sum(stage["segment_durations_s"])
    mode.stage_elapsed_time = sum(stage["segment_durations_s"]) + config["settle_gate"]["timeout_s"]
    apply_current_reference(mode)
    controller.update(q_start + .5, np.zeros(4))
    assert mode.state == ClimbMode.FAILED
    assert "TIMEOUT_NOT_CONTACT_PROOF" in mode.failure_reason


def test_false_flag_terminal_persistence_resets_and_front_hardware_stays_rejected():
    config = load_config()
    index = stage_index(config, "MID_PLATFORM_285")
    stage = config["stages"][index]
    q_start = entry_q(stage)
    controller = GraspController(DT, enable_link_collision_check=False, climb_timeout_uses_wall_time=False)
    controller.enter_climb(q_start, config, index, index, hardware_execution=False)
    mode = controller.climb_mode
    mode.phase_time = sum(stage["segment_durations_s"])
    apply_current_reference(mode)
    q_terminal = entry_q({"pose_start": stage["pose_end"], "anchor_knots": [stage["anchor_knots"][-1]]})
    controller.update(q_terminal, np.zeros(4))
    assert mode.settle_time == pytest.approx(DT)
    controller.update(q_start + .5, np.zeros(4))
    assert mode.settle_time == 0.0
    assert mode.state == ClimbMode.RUNNING
    with pytest.raises(ValueError, match="front compact climbing is simulation-only"):
        ClimbMode(None).enter(Q_CLIMB_PREP, config, hardware_execution=True)


def test_side_time_only_progression_is_unchanged():
    config = json.loads(SIDE.read_text())
    controller = GraspController(DT, enable_link_collision_check=False, climb_timeout_uses_wall_time=False)
    controller.enter_climb(Q_CLIMB_PREP, config, hardware_execution=False)
    mode = controller.climb_mode
    duration = sum(config["stages"][0]["segment_durations_s"])
    mode.phase_time = duration
    controller.update(Q_CLIMB_PREP + .5, np.zeros(4))
    # The existing SIDE preview remains elapsed-time only: lagged feedback
    # does not add a checkpoint hold or mutate its configuration semantics.
    assert mode.stage_index == 1
