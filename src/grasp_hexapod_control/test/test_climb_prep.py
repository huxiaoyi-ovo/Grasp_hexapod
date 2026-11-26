#!/usr/bin/env python3
"""行走站姿 Q_STAND、攀爬准备姿态 Q_CLIMB_PREP 与准备过渡（抬脚换位）测试。"""

from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from climb_mode import ClimbMode
from control import GraspController, MissionStateMachine
from kinematics import (
    FOOT_RADIUS,
    JOINT_AXIS_SIGNS,
    JOINT_LOWER,
    JOINT_UPPER,
    Q_CLIMB_PREP,
    Q_STAND,
    GraspKinematic,
)
from utils import NavigationState
from utils.climb import derive_compact_approach_geometry, select_compact_climb_side

DT = 1.0 / 30.0
ZERO = np.zeros(4)


def _settle_reset(controller, q_start):
    """B 回行走站姿：跑完 2 s 回正，返回最后一帧关节目标。"""
    controller.reset_to_stand(q_start)
    q = q_start.copy()
    for _ in range(80):
        q = controller.update(q, ZERO)
        if not controller.reset_active:
            break
    assert not controller.reset_active
    return q


def _run_transition(controller, q_start):
    """逐帧执行准备过渡（理想伺服），返回含起点的关节序列。"""
    ok, reason = controller.begin_climb_prep(q_start)
    assert ok, reason
    frames = [q_start.copy()]
    q = q_start.copy()
    while controller.climb_prep_active:
        q = controller.update(q, ZERO)
        frames.append(q.copy())
    return np.asarray(frames)


def test_walking_stance_literal_matches_its_geometric_definition():
    kinematic = GraspKinematic()
    sign = np.array([1.0, -1.0, 0.0, -1.0, 1.0, 0.0])
    beta = sign * np.deg2rad(-10.0)
    expected = np.column_stack(
        (0.125 * np.cos(beta), 0.125 * np.sin(beta), np.full(6, -0.115))
    )
    assert np.max(np.abs(kinematic.forward(Q_STAND) - expected)) < 1e-9
    # 机身高度约 96 mm（旧站姿 70 mm）。
    feet = kinematic.forward_base(Q_STAND)
    assert FOOT_RADIUS - feet[:, 2].mean() == pytest.approx(0.09625, abs=1e-4)


def test_climb_prep_is_the_old_default_stand_bit_for_bit():
    old_knee = np.deg2rad(43.035535181)
    old_ankle = np.deg2rad(-98.531183330)
    old_stand = np.tile(np.array([0.0, old_knee, old_ankle]), (6, 1)) / JOINT_AXIS_SIGNS
    assert np.array_equal(Q_CLIMB_PREP, old_stand)
    controller = GraspController(DT)
    assert np.array_equal(controller.q_init, Q_STAND)
    assert np.array_equal(controller.q_climb_prep, Q_CLIMB_PREP)
    assert np.array_equal(
        controller.foot_climb_prep_base,
        controller.kinematic.forward_base(Q_CLIMB_PREP),
    )
    assert controller.base_height_at_climb_prep == pytest.approx(
        FOOT_RADIUS - controller.foot_climb_prep_base[:, 2].mean()
    )
    assert controller.base_height_at_climb_prep == pytest.approx(0.06988468, abs=1e-7)


def test_climb_prep_transition_reaches_q_climb_prep_with_planned_swing_and_holds():
    controller = GraspController(DT, enable_link_collision_check=True)
    q_start = _settle_reset(controller, Q_STAND + 0.05)
    assert np.allclose(q_start, Q_STAND, atol=1e-12)
    q_start = Q_STAND.copy()
    frames = _run_transition(controller, q_start)

    s1, s2, s3, s4, s5 = controller.climb_prep_segment_frames
    assert (s1, s2, s3, s4, s5) == (18, 3, 18, 3, 36)
    assert len(frames) == 1 + sum((s1, s2, s3, s4, s5))
    assert np.array_equal(frames[-1], Q_CLIMB_PREP)
    assert controller.climb_prep_posture_held
    assert not controller.climb_prep_active

    # 逐帧关节速度、关节余量、整机碰撞。
    assert np.max(np.abs(np.diff(frames, axis=0))) / DT <= 3.0
    assert np.min(np.minimum(frames[1:] - JOINT_LOWER, JOINT_UPPER - frames[1:])) >= 0.05
    assert all(controller._link_collision_free(q).all() for q in frames[1:])

    group_a, group_b = [0, 1, 5], [2, 3, 4]
    s1_end = s1
    s3_start = s1 + s2
    s3_end = s3_start + s3
    # S1：B 组关节不动；S3：A 组停在 S1 末姿态。
    assert np.array_equal(frames[1:s1_end + 1, group_b], np.repeat(q_start[group_b][None], s1, 0))
    assert np.array_equal(
        frames[s3_start:s3_end + 1, group_a],
        np.repeat(frames[s1_end, group_a][None], s3 + 1, 0),
    )
    # 摆动腿：峰值净空 >= 29 mm，u=0.1..0.9 内不低于起点地面 0.5 mm。
    feet = np.asarray([controller.kinematic.forward_base(q) for q in frames])
    ground_z = feet[0, :, 2]
    for legs, start, count in ((group_a, 0, s1), (group_b, s3_start, s3)):
        height = feet[start:start + count + 1, :, 2][:, legs] - ground_z[legs]
        assert height.max(axis=0).min() >= 0.029
        lo = int(np.ceil(0.1 * count))
        hi = int(np.floor(0.9 * count))
        assert height[lo:hi + 1].min() >= -0.0005

    # 过渡结束后即使有行走指令也保持准备姿态，步态不写关节覆盖。
    q = frames[-1].copy()
    for _ in range(40):
        q = controller.update(q, np.array([0.1, 0.1, 0.0, 0.0]))
        assert np.array_equal(q, Q_CLIMB_PREP)
        assert all(override is None for override in controller.joint_override)
    assert not controller.approach_mode.gait_started
    assert controller.climb_prep_posture_held

    # B：取消保持，回到行走站姿。
    controller.reset_to_stand(q)
    assert not controller.climb_prep_posture_held
    for _ in range(80):
        q = controller.update(q, ZERO)
        if not controller.reset_active:
            break
    assert not controller.reset_active
    assert np.array_equal(q, Q_STAND)


def test_climb_prep_foot_displacement_is_small_and_mostly_lateral():
    controller = GraspController(DT)
    frames = _run_transition(controller, Q_STAND.copy())
    start = controller.kinematic.forward_base(frames[0])
    end = controller.kinematic.forward_base(frames[-1])
    displacement = np.linalg.norm((end - start)[:, :2], axis=1)
    lateral = np.abs((end - start)[:, 0])
    assert displacement.max() < 0.030
    assert (lateral >= 0.9 * displacement).all()


def test_begin_climb_prep_rejects_bad_preconditions_without_changing_state():
    controller = GraspController(DT)
    q = Q_STAND.copy()

    controller.mode = controller.CLIMB
    ok, reason = controller.begin_climb_prep(q)
    assert not ok and "APPROACH" in reason
    controller.mode = controller.APPROACH

    controller.reset_active = True
    ok, reason = controller.begin_climb_prep(q)
    assert not ok and "reset" in reason
    controller.reset_active = False

    controller.approach_mode.gait_started = True
    ok, reason = controller.begin_climb_prep(q)
    assert not ok and "idle" in reason
    controller.approach_mode.gait_started = False
    controller.approach_mode.transfer_active = True
    ok, reason = controller.begin_climb_prep(q)
    assert not ok and "idle" in reason
    controller.approach_mode.transfer_active = False

    bad = q.copy()
    bad[0, 0] = np.nan
    ok, reason = controller.begin_climb_prep(bad)
    assert not ok and "finite" in reason

    assert not controller.climb_prep_active
    assert not controller.climb_prep_posture_held

    ok, _ = controller.begin_climb_prep(q)
    assert ok and controller.climb_prep_active
    ok, reason = controller.begin_climb_prep(q)
    assert not ok and "already active" in reason
    assert controller.climb_prep_active

    # B 取消进行中的过渡。
    controller.reset_to_stand(q)
    assert not controller.climb_prep_active
    assert not controller.climb_prep_posture_held


def test_enter_climb_and_abort_consume_the_held_posture():
    controller = GraspController(DT)
    _run_transition(controller, Q_STAND.copy())
    assert controller.climb_prep_posture_held
    controller.abort_climb()
    assert not controller.climb_prep_posture_held

    controller = GraspController(DT)
    _run_transition(controller, Q_STAND.copy())
    controller.enter_climb(Q_CLIMB_PREP, hardware_execution=True)
    assert not controller.climb_prep_posture_held
    assert not controller.climb_prep_active


def test_infeasible_climb_prep_schedule_fails_closed(monkeypatch):
    controller = GraspController(DT)
    # 起点关节贴近限位：余量门拒绝，且不改变状态。
    q = Q_STAND.copy()
    q[0, 1] = JOINT_LOWER[0, 1] + 0.01
    ok, reason = controller.begin_climb_prep(q)
    assert not ok
    assert reason
    assert not controller.climb_prep_active
    assert not controller.climb_prep_posture_held


def _p0_pose(controller, geometry):
    """左侧固定接近位姿在 pv 系（= 配置世界系）中的 base 位姿，即 P0。"""
    xiaolan = np.eye(4)
    xiaolan[:3, 3] = controller._xiaolan_translation
    return xiaolan @ geometry["targets"]["left"]


def _navigation(controller, left, stamp):
    xiaolan = np.eye(4)
    xiaolan[:3, 3] = controller._xiaolan_translation
    return NavigationState(
        stamp=stamp,
        valid=True,
        landing_confirmed=True,
        pv_from_base=controller._base_pose_world.copy(),
        pv_from_xiaolan=xiaolan,
        pv_boundary=np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]]),
    )


def test_full_mission_reaches_climb_entry_through_the_climb_prep_transition():
    controller = GraspController(DT)
    config = controller.climb_mode._load_config()
    left = select_compact_climb_side(config, "left")
    geometry = derive_compact_approach_geometry(config)
    for side in ("left", "right"):
        controller.approach_mode.configure_fixed_approach(
            geometry["targets"][side],
            target_side=side,
            xiaolan_keepout_polygon_xy_m=geometry["xiaolan_keepout_polygon_xy_m"],
            xiaolan_body_clearance_m=0.13,
        )
    # 机器人已站在 P0（pv 系即配置世界系）：接近立即就绪，随后经准备过渡进入攀爬。
    controller._xiaolan_translation = np.asarray(config["xiaolan_translation"], dtype=float)
    controller._base_pose_world = _p0_pose(controller, geometry)
    stamp = 1.0
    plan = controller.start_full_mission(
        _navigation(controller, left, stamp), left, approach_timeout_s=30.0
    )
    assert not plan.failed, plan.reason

    q = Q_STAND.copy()
    states = []
    prep_started_at = None
    prep_done_at = None
    for index in range(600):
        stamp += DT
        q = controller.update(q, ZERO, _navigation(controller, left, stamp))
        state = controller.mission.state
        if not states or states[-1] != state:
            states.append(state)
        if prep_started_at is None and controller.climb_prep_active:
            prep_started_at = index
        if prep_started_at is not None and prep_done_at is None and not controller.climb_prep_active:
            prep_done_at = index
        if state in (MissionStateMachine.CLIMB, MissionStateMachine.FAILED):
            break
    assert controller.mission.state == MissionStateMachine.CLIMB, controller.mission.reason
    assert states == [
        MissionStateMachine.APPROACH,
        MissionStateMachine.PREPARE_CLIMB,
        MissionStateMachine.CLIMB,
    ]
    assert prep_started_at is not None and prep_done_at is not None
    assert prep_done_at - prep_started_at + 1 >= sum(controller.climb_prep_segment_frames) - 1
    # 进入攀爬时机器人已在 Q_CLIMB_PREP，准备保持标志已被消耗。
    assert controller.climb_mode.state == ClimbMode.RUNNING
    assert not controller.climb_prep_posture_held
    assert controller.mission.last_prepare_joint_error_rad <= 1e-9


def test_mission_fails_with_the_reason_when_climb_prep_is_infeasible():
    controller = GraspController(DT)
    config = controller.climb_mode._load_config()
    left = select_compact_climb_side(config, "left")
    geometry = derive_compact_approach_geometry(config)
    for side in ("left", "right"):
        controller.approach_mode.configure_fixed_approach(
            geometry["targets"][side],
            target_side=side,
            xiaolan_keepout_polygon_xy_m=geometry["xiaolan_keepout_polygon_xy_m"],
            xiaolan_body_clearance_m=0.13,
        )
    controller._xiaolan_translation = np.asarray(config["xiaolan_translation"], dtype=float)
    controller._base_pose_world = _p0_pose(controller, geometry)
    stamp = 1.0
    controller.start_full_mission(_navigation(controller, left, stamp), left)
    q = Q_STAND.copy()
    q[0, 1] = JOINT_LOWER[0, 1] + 0.01
    for _ in range(20):
        stamp += DT
        controller.update(q, ZERO, _navigation(controller, left, stamp))
        if controller.mission.state == MissionStateMachine.FAILED:
            break
    assert controller.mission.state == MissionStateMachine.FAILED
    assert "climb prep" in controller.mission.reason
    assert not controller.climb_prep_active
