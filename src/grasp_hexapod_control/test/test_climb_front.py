import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from climb_mode import ClimbMode
from kinematics import GraspKinematic, Q_STAND
from build_climb_front import (
    FOOT_RADIUS, P0, TRANSLATION, build, base_points, foot_sphere_signed_gap,
    ground_pair_sphere_sweep, load_triangles, preflight_synthetic_stage_entry,
    quintic_weight, require_ground_pair_sphere_sweep,
    require_ground_sphere_clearance, surface_height, world_from_base,
)


def test_front_inward_prefix_uses_stl_landings_and_low_apex():
    config = build(stop_after="FRONT_INWARD")

    ClimbMode(None)._validate_config(config)
    assert config["prefix_preview_only"] is True
    assert config["p0"]["base"][5] == pytest.approx(-np.pi / 2.0)
    assert [stage["name"] for stage in config["stages"]] == [
        "PREP", "FRONT_LOW", "BODY_A", "MID_GROUND_SHIFT", "BODY_B",
        "FRONT_INWARD", "PREVIEW_HOLD",
    ]
    assert config["stages"][-1]["name"] == "PREVIEW_HOLD"
    assert config["stages"][-1]["pose_start"] == config["stages"][-1]["pose_end"]

    stages = {stage["name"]: stage for stage in config["stages"]}
    assert stages["PREP"]["pose_end"][3] == pytest.approx(.15)
    assert stages["BODY_A"]["pose_end"][3] == pytest.approx(.12)
    assert stages["BODY_B"]["pose_end"][3] == pytest.approx(.25)
    triangles = load_triangles(Path("src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL"))
    for name, target_x in (("FRONT_LOW", .19), ("FRONT_INWARD", .29)):
        knots = np.asarray(stages[name]["anchor_knots"])
        assert np.allclose(knots[-1][[1, 4], 0], target_x)
        for leg in (1, 4):
            assert knots[-1, leg, 2] == pytest.approx(surface_height(triangles, target_x, knots[-1, leg, 1]))
        assert np.allclose(
            knots[1:3, [1, 4], 2],
            max(knots[0, [1, 4], 2].max(), knots[-1, [1, 4], 2].max()) + .020,
        )
    assert stages["FRONT_LOW"]["segment_durations_s"] == [1.35, .85, .2375]
    assert stages["FRONT_INWARD"]["segment_durations_s"] == [.525, .725, .2375]

    with pytest.raises(ValueError, match="simulation-only"):
        ClimbMode(None).enter(Q_STAND, config, hardware_execution=True)


def test_front_refoot_prefix_keeps_synchronized_base_knots_and_stl_receipt():
    config = build(stop_after="FRONT_REFOOT")
    ClimbMode(None)._validate_config(config)
    stages = config["stages"]
    assert config["prefix_preview_only"] is True
    assert [stage["name"] for stage in stages[-7:]] == [
        "MID_BACK_ROLL_SYNC", "BACK_NEG17_SYNC", "MID_ZERO_SYNC",
        "BACK_NEG09_SYNC", "MID_FORWARD_SYNC", "FRONT_REFOOT", "PREVIEW_HOLD",
    ]

    sync_names = [
        "MID_BACK_ROLL_SYNC", "BACK_NEG17_SYNC", "MID_ZERO_SYNC",
        "BACK_NEG09_SYNC", "MID_FORWARD_SYNC", "FRONT_REFOOT",
    ]
    by_name = {stage["name"]: stage for stage in stages}
    assert by_name["BACK_NEG09_SYNC"]["pose_end"][0] == pytest.approx(.029)
    assert by_name["MID_FORWARD_SYNC"]["pose_end"][0] == pytest.approx(.039)
    assert by_name["FRONT_REFOOT"]["pose_start"] == by_name["FRONT_REFOOT"]["pose_end"]
    for name in sync_names:
        stage = by_name[name]
        active = np.asarray(stage["active_legs"], dtype=int)
        knots = np.asarray(stage["anchor_knots"], dtype=float)
        base_knots = np.asarray(stage["active_base_knots_m"], dtype=float)
        durations = np.asarray(stage["segment_durations_s"], dtype=float)
        fractions = np.r_[0.0, np.cumsum(durations) / durations.sum()]
        for index, fraction in enumerate(fractions):
            weight = quintic_weight(fraction)
            pose = ((1.0 - weight) * np.asarray(stage["pose_start"])
                    + weight * np.asarray(stage["pose_end"]))
            world = (
                np.column_stack((base_knots[index], np.ones(len(active))))
                @ world_from_base(pose).T
            )[:, :3]
            assert np.allclose(world, knots[index, active], atol=1e-9)
        apex = max(knots[0, active, 2].max(), knots[-1, active, 2].max()) + .020
        assert np.allclose(knots[1:3, active, 2], apex)

    front = by_name["FRONT_REFOOT"]
    front_knots = np.asarray(front["anchor_knots"], dtype=float)
    triangles = load_triangles(Path("src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL"))
    assert np.allclose(front_knots[-1, [1, 4], 0], .34)
    for leg in (1, 4):
        y = front_knots[-1, leg, 1]
        assert front_knots[-1, leg, 2] == pytest.approx(surface_height(triangles, .34, y))
        for angle in np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False):
            surface_height(triangles, .34 + .0185 * np.cos(angle), y + .0185 * np.sin(angle))


def test_synthetic_entry_preflight_rejects_stale_joint_or_previous_reference():
    kinematic = GraspKinematic()
    anchors = (
        np.column_stack((kinematic.forward_base(Q_STAND), np.ones(6)))
        @ world_from_base(P0).T
    )[:, :3]
    target = base_points(P0, anchors)

    assert np.allclose(
        preflight_synthetic_stage_entry(kinematic, P0, anchors, Q_STAND, target),
        target,
    )
    stale_pose = P0.copy()
    stale_pose[0] += .01
    with pytest.raises(ValueError, match="FK mismatch"):
        preflight_synthetic_stage_entry(kinematic, stale_pose, anchors, Q_STAND, target)
    with pytest.raises(ValueError, match="previous-reference mismatch"):
        preflight_synthetic_stage_entry(kinematic, P0, anchors, Q_STAND, target + .003)


def test_ground_targets_use_local_stl_sphere_gap_not_bbox_or_xy_ring_proxy():
    config = build(stop_after="FRONT_REFOOT")
    receipts = config["surface_receipt"]["ground_sphere_clearance"]
    mid_at_eight_cm = [
        receipt for receipt in receipts
        if receipt["legs"] == [2, 5] and receipt["x_m"] == pytest.approx(.08)
    ]
    assert len(mid_at_eight_cm) == 1
    assert mid_at_eight_cm[0]["min_signed_sphere_gap_m"] > .026

    triangles = load_triangles(Path("src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL")) + TRANSLATION
    mid_y = np.asarray(config["p0"]["anchors_world_m"])[[2, 5], 1]
    gaps_at_twelve_cm = [
        foot_sphere_signed_gap(triangles, np.array((.12, y, FOOT_RADIUS)))
        for y in mid_y
    ]
    assert min(gaps_at_twelve_cm) > .022
    back_y = np.asarray(config["p0"]["anchors_world_m"])[[0, 3], 1]
    gaps_at_back_three_cm = [
        foot_sphere_signed_gap(triangles, np.array((.03, y, FOOT_RADIUS)))
        for y in back_y
    ]
    assert min(gaps_at_back_three_cm) > .11

    penetrating_triangle = np.array([[
        (-.01, -.01, FOOT_RADIUS), (.01, -.01, FOOT_RADIUS), (.0, .01, FOOT_RADIUS),
    ]])
    with pytest.raises(ValueError, match="ground foot sphere intersects"):
        require_ground_sphere_clearance(penetrating_triangle, np.array((.0, .0, FOOT_RADIUS)))


def test_ground_pair_sweep_rejects_the_mid_vertical_lift_hidden_by_valid_endpoints():
    config = build(stop_after="FRONT_REFOOT")
    triangles = load_triangles(Path("src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL")) + TRANSLATION
    anchors = np.asarray(config["p0"]["anchors_world_m"], dtype=float)
    legs = [2, 5]

    def vertical_ground_pair(x):
        start = anchors.copy()
        start[legs, 0] = x
        end = start.copy()
        air = start.copy()
        air[legs, 2] = .0865
        knots = [start, air, air.copy(), end]
        base_knots = [base_points(P0, knot[legs]) for knot in knots]
        return knots, base_knots

    knots, base_knots = vertical_ground_pair(.08)
    receipt = ground_pair_sphere_sweep(
        triangles, P0, P0, knots, base_knots, [.6, .6, .6], legs,
    )
    assert receipt["min_signed_sphere_gap_m"] > .008
    assert receipt["first_below_negative_half_mm"] is None
    for x in (.10, .12):
        knots, base_knots = vertical_ground_pair(x)
        with pytest.raises(ValueError, match="ground pair sphere sweep enters"):
            require_ground_pair_sphere_sweep(
                triangles, P0, P0, knots, base_knots, [.6, .6, .6], legs,
            )

    sweeps = config["surface_receipt"]["ground_pair_sweep_clearance"]
    mid_forward = next(receipt for receipt in sweeps if receipt["stage"] == "MID_FORWARD_SYNC")
    assert mid_forward["min_signed_sphere_gap_m"] > .011


def test_mid_low_prefix_preserves_c13_and_uses_the_frozen_five_knot_path():
    c13 = build(stop_after="FRONT_REFOOT")
    config = build(stop_after="MID_LOW")
    ClimbMode(None)._validate_config(config)
    assert config["prefix_preview_only"] is True

    # The prior C1--F dictionaries are an immutable boundary for this batch.
    assert config["stages"][:12] == c13["stages"][:-1]
    by_name = {stage["name"]: stage for stage in config["stages"]}
    assert [stage["name"] for stage in config["stages"][-3:]] == [
        "P_BODY_PREP", "Q_MID_LOW", "PREVIEW_HOLD",
    ]
    assert by_name["P_BODY_PREP"]["pose_end"] == pytest.approx(
        [.1017, P0[1], .155, .15, 0.0, -np.pi / 2.0]
    )

    q = by_name["Q_MID_LOW"]
    active = np.asarray(q["active_legs"], dtype=int)
    knots = np.asarray(q["anchor_knots"], dtype=float)
    base_knots = np.asarray(q["active_base_knots_m"], dtype=float)
    assert active.tolist() == [2, 5]
    assert q["segment_durations_s"] == [1.2, 1.0, 1.0, .5]
    assert np.allclose(q["active_base_velocities_m_s"], 0.0)
    assert np.allclose(knots[1, active, (0, 2)], [[.08, .0865], [.08, .0865]])
    assert np.allclose(knots[2, active, (0, 2)], [[.14, .12], [.14, .12]])
    assert np.allclose(knots[3, active, 0], .17)

    triangles = load_triangles(Path("src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL"))
    for index, leg in enumerate(active):
        surface = surface_height(triangles, .19, knots[4, leg, 1])
        assert knots[3, leg, 2] == pytest.approx(surface + .010)
        assert knots[4, leg, 0] == pytest.approx(.19)
        assert knots[4, leg, 2] == pytest.approx(surface)
        for angle in np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False):
            surface_height(
                triangles, .19 + .0185 * np.cos(angle),
                knots[4, leg, 1] + .0185 * np.sin(angle),
            )
    for index in range(5):
        world = (
            np.column_stack((base_knots[index], np.ones(len(active))))
            @ world_from_base(np.asarray(q["pose_start"], dtype=float)).T
        )[:, :3]
        assert np.allclose(world, knots[index, active], atol=1e-9)

    q_sweep = next(
        receipt for receipt in config["surface_receipt"]["ground_pair_sweep_clearance"]
        if receipt["stage"] == "Q_MID_LOW"
    )
    assert q_sweep["first_below_negative_half_mm"] is None
    assert q_sweep["min_signed_sphere_gap_m"] >= -.0005


def test_front_pair_first_segment_pose_curve_reaches_pose_end_at_lift_boundary_only():
    config = build(stop_after="FRONT_REFOOT")
    index = next(
        i for i, stage in enumerate(config["stages"])
        if stage["name"] == "MID_FORWARD_SYNC"
    )
    stage = copy.deepcopy(config["stages"][index])
    active = np.asarray(stage["active_legs"], dtype=int)
    durations = np.asarray(stage["segment_durations_s"], dtype=float)
    knots = np.asarray(stage["anchor_knots"], dtype=float)
    pose_start = np.asarray(stage["pose_start"], dtype=float)
    pose_end = np.asarray(stage["pose_end"], dtype=float)
    cumulative = np.r_[0.0, np.cumsum(durations)]
    first_segment_poses = [
        pose_start * (1.0 - quintic_weight(min(time / durations[0], 1.0)))
        + pose_end * quintic_weight(min(time / durations[0], 1.0))
        for time in cumulative
    ]
    stage["pose_curve"] = "quintic_first_segment"
    stage["active_base_knots_m"] = [
        base_points(pose, knot[active]).tolist()
        for pose, knot in zip(first_segment_poses, knots)
    ]
    candidate = copy.deepcopy(config)
    candidate["stages"][index] = stage
    ClimbMode(None)._validate_config(candidate)

    mode = ClimbMode(None)
    mode.config = candidate
    mode.stage_index = index
    mode.phase_time = float(durations[0])
    pose, anchors, _ = mode._stage_reference()
    fixed = [leg for leg in range(6) if leg not in active]
    assert np.allclose(pose, pose_end, atol=1e-12)
    assert np.allclose(anchors[fixed], knots[0, fixed], atol=1e-12)

    side = json.loads(Path("src/grasp_hexapod_control/config/climb_compact.json").read_text())
    side_pair = next(stage for stage in side["stages"] if stage["name"] == "PAIR")
    side_pair["pose_curve"] = "quintic_first_segment"
    with pytest.raises(ValueError, match="invalid compact stage fields: PAIR"):
        ClimbMode(None)._validate_config(side)


def test_full_front_candidate_uses_exact_stl_tail_and_two_second_hold():
    from control import GraspController
    from build_climb_front import require_surface_sphere_clearance

    config = build()
    ClimbMode(None)._validate_config(config)
    assert config["prefix_preview_only"] is False
    assert config["stages"][-1]["name"] == "FINAL_HOLD"
    assert config["stages"][-1]["segment_durations_s"] == [2.0]
    assert config["stages"][-1]["pose_start"] == config["stages"][-1]["pose_end"]

    by_name = {stage["name"]: stage for stage in config["stages"]}
    assert by_name["MID_GROUND_SHIFT"]["anchor_knots"][1][2][2] == pytest.approx(.0265)
    assert by_name["MID_GROUND_SHIFT"]["segment_durations_s"] == pytest.approx([.475, .95, .475])
    assert by_name["AD_RB_075_SYNC"]["segment_durations_s"] == pytest.approx(
        [.790625, 1.078125, .790625]
    )
    assert by_name["AG2_MID_235_SYNC"]["anchor_knots"][-1][2][0] == pytest.approx(.235)
    ah = by_name["AH_BACK_LOW_FIRST_SEGMENT"]
    assert ah["pose_curve"] == "quintic_first_segment"

    triangles = load_triangles(Path("src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL")) + TRANSLATION
    final_anchors = np.asarray(config["stages"][-1]["anchor_knots"][-1], dtype=float)
    for anchor in final_anchors:
        assert foot_sphere_signed_gap(triangles, anchor) >= -.0005
    with pytest.raises(ValueError, match="surface foot sphere enters"):
        require_surface_sphere_clearance(
            triangles, np.array((.24, final_anchors[5, 1], surface_height(
                triangles - TRANSLATION, .24, final_anchors[5, 1],
            ))),
        )

    kinematic = GraspKinematic()
    terminal = np.asarray(config["terminal_q_rad"], dtype=float)
    target = base_points(np.asarray(ah["pose_end"], dtype=float), final_anchors)
    assert np.max(np.linalg.norm(kinematic.forward_base(terminal) - target, axis=1)) < 1.e-5
    assert GraspController(1.0 / 30.0, False, False)._link_collision_free(terminal).all()


def test_ah_first_segment_base_knots_follow_pose_end_after_lift():
    config = build()
    index = next(i for i, stage in enumerate(config["stages"])
                 if stage["name"] == "AH_BACK_LOW_FIRST_SEGMENT")
    stage = config["stages"][index]
    active = np.asarray(stage["active_legs"], dtype=int)
    knots = np.asarray(stage["anchor_knots"], dtype=float)
    base_knots = np.asarray(stage["active_base_knots_m"], dtype=float)
    durations = np.asarray(stage["segment_durations_s"], dtype=float)
    pose_start = np.asarray(stage["pose_start"], dtype=float)
    pose_end = np.asarray(stage["pose_end"], dtype=float)
    for time, base_knot, world_knot in zip(np.r_[0.0, np.cumsum(durations)], base_knots, knots):
        weight = quintic_weight(min(time / durations[0], 1.0))
        pose = (1.0 - weight) * pose_start + weight * pose_end
        world = (np.column_stack((base_knot, np.ones(len(active)))) @ world_from_base(pose).T)[:, :3]
        assert np.allclose(world, world_knot[active], atol=1e-9)
    assert np.allclose(
        (1.0 - quintic_weight(1.0)) * pose_start + quintic_weight(1.0) * pose_end,
        pose_end,
    )
