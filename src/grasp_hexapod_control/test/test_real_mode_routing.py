#!/usr/bin/env python3
"""纯CPU回归：实机模式仲裁和公共关节目标安全门。"""

import importlib.util
import json
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
ROS_STUB_MODULES = (
    "rospy",
    "geometry_msgs",
    "geometry_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
)


def _install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logwarn_throttle = lambda *args, **kwargs: None
    rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: 1.0)
    )
    sys.modules.setdefault("rospy", rospy)

    geometry = types.ModuleType("geometry_msgs.msg")
    geometry.PolygonStamped = type("PolygonStamped", (), {})
    geometry.PoseStamped = type("PoseStamped", (), {})
    sys.modules.setdefault("geometry_msgs", types.ModuleType("geometry_msgs"))
    sys.modules.setdefault("geometry_msgs.msg", geometry)

    sensor = types.ModuleType("sensor_msgs.msg")
    sensor.Imu = type("Imu", (), {})
    sensor.JointState = type("JointState", (), {})
    sensor.Joy = type("Joy", (), {})
    sys.modules.setdefault("sensor_msgs", types.ModuleType("sensor_msgs"))
    sys.modules.setdefault("sensor_msgs.msg", sensor)

    std = types.ModuleType("std_msgs.msg")
    std.Bool = type("Bool", (), {})
    std.Float64MultiArray = type("Float64MultiArray", (), {})
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
    sys.modules.setdefault("std_msgs.msg", std)


_missing_module = object()
_previous_ros_modules = {
    name: sys.modules.get(name, _missing_module)
    for name in ROS_STUB_MODULES
}
try:
    _install_ros_stubs()
    RUN_REAL_SPEC = importlib.util.spec_from_file_location(
        "run_real_for_test", SCRIPTS / "run_real.py"
    )
    RUN_REAL = importlib.util.module_from_spec(RUN_REAL_SPEC)
    RUN_REAL_SPEC.loader.exec_module(RUN_REAL)
finally:
    # 仅在run_real导入期间使用替身，避免污染同一pytest进程中的ROS测试。
    for _name, _module in _previous_ros_modules.items():
        if _module is _missing_module:
            sys.modules.pop(_name, None)
        else:
            sys.modules[_name] = _module

from climb_mode import ClimbMode
from control import COLLISION_MARGIN, LINK_COLLISION_RADII, GraspController
from kinematics import JOINT_LOWER, JOINT_UPPER, Q_STAND


class _Mission:
    def __init__(self):
        self.cancelled = False

    def cancel(self, reason):
        del reason
        self.cancelled = True


class _Dock:
    def __init__(self):
        self.active = True
        self.exited = False

    def exit(self):
        self.exited = True
        self.active = False


class _ButtonController:
    APPROACH = "approach"
    CLIMB = "climb"
    DOCK = "dock"

    def __init__(self):
        self.mode = self.APPROACH
        self.reset_active = True
        self.mission = _Mission()
        self.dock_mode = _Dock()
        self.aborted = False

    def abort_climb(self):
        self.aborted = True


def _button_node():
    node = object.__new__(RUN_REAL.RosControlNode)
    node.local_execution = False
    node.button_a = 0
    node.button_b = 1
    node.button_x = 2
    node.button_y = 3
    node.enable_real_climb = False
    node.enable_real_dock = False
    node.max_joy_age = 0.2
    node.state = node.HOLD
    node.controller = _ButtonController()
    node.manual_override = True
    node.command = np.ones(4, dtype=np.float64)
    node.real_climb_monitor_active = False
    node.real_climb_speed_diagnostic = None
    return node


def test_run_real_launch_enables_climb_by_default():
    launch = ET.parse(SCRIPTS.parent / "launch" / "run_real.launch")
    arguments = {
        argument.attrib["name"]: argument.attrib.get("default")
        for argument in launch.findall("arg")
    }
    assert arguments["enable_real_climb"] == "true"


def test_real_launches_enable_dock_by_default():
    for launch_name in ("run_real.launch", "control_stack.launch"):
        launch = ET.parse(SCRIPTS.parent / "launch" / launch_name)
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in launch.findall("arg")
        }
        assert arguments["enable_real_dock"] == "true"


def test_dual_board_frame_waits_until_all_six_legs_are_new():
    q_cur = np.zeros((6, 3), dtype=np.float64)
    previous = np.full(6, 9.9, dtype=np.float64)
    current = np.full(6, 10.0, dtype=np.float64)
    ready, complete = RUN_REAL.RosControlNode._feedback_frame_state(
        q_cur, current, previous, now=10.05, max_feedback_age=0.15
    )
    assert ready
    assert complete

    # 左板三条腿已进入下一轮、右板仍停留在上轮时不得重复推进控制器。
    partial = current.copy()
    for leg_name in ("lf", "lm", "lb"):
        partial[RUN_REAL.RosControlNode.LEG_INDEX[leg_name]] = 10.04
    ready, complete = RUN_REAL.RosControlNode._feedback_frame_state(
        q_cur, partial, current, now=10.06, max_feedback_age=0.15
    )
    assert ready
    assert not complete


def test_dual_board_feedback_issue_identifies_stale_board_and_leg():
    q_cur = np.zeros((6, 3), dtype=np.float64)
    stamps = np.full(6, 20.0, dtype=np.float64)
    stamps[RUN_REAL.RosControlNode.LEG_INDEX["rf"]] = 19.7
    ready, complete = RUN_REAL.RosControlNode._feedback_frame_state(
        q_cur, stamps, np.zeros(6), now=20.1, max_feedback_age=0.15
    )
    assert not ready
    assert not complete
    issue = RUN_REAL.RosControlNode._feedback_issue(
        q_cur, stamps, now=20.1, max_feedback_age=0.15
    )
    assert "right[rf=stale(0.400s)]" in issue
    assert "snapshot_skew=0.300s" in issue


def test_real_climb_speed_diagnostic_uses_feedback_timestamps():
    node = object.__new__(RUN_REAL.RosControlNode)
    node.local_execution = False
    node.real_climb_speed_diagnostic = None
    node.controller = types.SimpleNamespace(
        mode="climb",
        CLIMB="climb",
        last_update_velocity_limit_clip_count=0,
        last_update_collision_guard_hold_count=0,
    )
    q_start = np.zeros((6, 3), dtype=np.float64)
    q_next = np.full((6, 3), 0.2, dtype=np.float64)
    stamp_start = np.full(6, 30.0, dtype=np.float64)
    stamp_next = stamp_start + np.array(
        [0.1, 0.2, 0.25, 0.4, 0.5, 1.0], dtype=np.float64
    )
    node._record_real_climb_speed_diagnostic(
        "C1", q_start, q_start, stamp_start, 30.0
    )
    node._record_real_climb_speed_diagnostic(
        "C1", q_next, q_next, stamp_next, 31.0
    )
    item = node.real_climb_speed_diagnostic
    assert np.isclose(item["peak_command_speed_rad_s"], 0.2)
    assert np.isclose(item["peak_measured_speed_rad_s"], 2.0)


def test_x_y_disabled_requests_and_conflicts_do_not_start():
    node = _button_node()
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 0, 1, 0]), True, Q_STAND
    )
    assert node.state == node.HOLD
    assert node.controller.mode == node.controller.APPROACH

    calls = []
    node._start_real_climb = lambda *args: calls.append("x")
    node._start_real_dock = lambda *args: calls.append("y")
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 0, 1, 1]), True, Q_STAND
    )
    assert calls == []


def test_button2_routes_only_to_climb_and_button3_only_to_dock():
    node = _button_node()
    calls = []
    node._start_real_climb = lambda *args: calls.append("x")
    node._start_real_dock = lambda *args: calls.append("y")

    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 0, 1, 0]), True, Q_STAND
    )
    assert calls == ["x"]

    calls.clear()
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 0, 0, 1]), True, Q_STAND
    )
    assert calls == ["y"]


def test_b_cleans_mission_climb_and_dock_before_reset():
    node = _button_node()
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 1, 0, 0]), False, Q_STAND
    )
    assert node.state == node.RESETTING
    assert node.controller.mission.cancelled
    assert node.controller.aborted
    assert node.controller.dock_mode.exited
    assert not node.manual_override
    assert np.array_equal(node.command, np.zeros(4))


def test_climb_done_moves_to_hold_then_y_routes_to_dock_request():
    node = _button_node()
    node.state = node.RUNNING
    node.controller.mode = node.controller.CLIMB
    node.controller.climb_mode = types.SimpleNamespace(
        state=ClimbMode.DONE,
        failure_reason="",
    )
    node.real_climb_monitor_active = True
    node.controller.update = lambda *args: Q_STAND.copy()
    monitor_calls = []
    node._monitor_real_climb = lambda: monitor_calls.append(True)
    node._hold_motion = lambda *args, **kwargs: None
    node._make_command = lambda axes: np.zeros(4)
    node.control_source = "teleop"
    node._update_control(
        Q_STAND, np.empty(0), np.zeros(4), 1.0, 1.0, feedback_ready=True
    )
    assert node.state == node.HOLD
    assert monitor_calls == [True]

    calls = []
    node._start_real_dock = lambda *args: calls.append("dock")
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 0, 0, 1]), True, Q_STAND
    )
    assert calls == ["dock"]


def test_climb_safety_persistence_counts_one_30hz_update_per_frame():
    node = _button_node()
    node.state = node.RUNNING
    node.real_climb_monitor_active = True
    node.controller.mode = node.controller.CLIMB
    node.controller.climb_mode = types.SimpleNamespace(state=ClimbMode.RUNNING)
    hold_calls = []
    node.controller.hold_climb = lambda: hold_calls.append(True)
    node.real_climb_persistence_frames = 3
    node.climb_bad_frames = 0
    node.climb_good_frames = 0
    node._real_climb_observation = lambda: (False, "relative RTK position error")
    for _ in range(2):
        RUN_REAL.RosControlNode._monitor_real_climb(node)
    assert node.state == node.RUNNING
    assert hold_calls == []
    RUN_REAL.RosControlNode._monitor_real_climb(node)
    assert node.state == node.HOLD
    assert hold_calls == [True]


class _DiagnosticReplayController:
    APPROACH = "approach"
    CLIMB = "climb"

    def __init__(self):
        self.mode = self.APPROACH
        self.climb_mode = types.SimpleNamespace(base_pose=None)
        self.entered_hardware_execution = None

    def enter_climb(self, q_cur, hardware_execution):
        del q_cur
        self.entered_hardware_execution = hardware_execution
        self.mode = self.CLIMB
        self.climb_mode = types.SimpleNamespace(
            base_pose=np.zeros(5, dtype=np.float64),
            state=ClimbMode.RUNNING,
        )


def _diagnostic_replay_node(navigation_valid, imu_valid):
    node = _button_node()
    node.enable_real_climb = True
    node.controller = _DiagnosticReplayController()
    node.navigation = types.SimpleNamespace(
        motion_snapshot=lambda: types.SimpleNamespace(valid=navigation_valid)
    )
    node.imu = types.SimpleNamespace(
        snapshot=lambda: {
            "valid": imu_valid,
            "rotation": np.eye(3),
        }
    )
    return node


def test_x_starts_diagnostic_replay_without_imu_or_rtk_monitoring():
    node = _diagnostic_replay_node(navigation_valid=False, imu_valid=False)
    RUN_REAL.RosControlNode._start_real_climb(node, Q_STAND, True)
    assert node.state == node.RUNNING
    assert node.controller.entered_hardware_execution is True
    assert not node.real_climb_monitor_active


def test_x_enables_optional_monitoring_when_imu_and_rtk_are_fresh():
    node = _diagnostic_replay_node(navigation_valid=True, imu_valid=True)
    RUN_REAL.RosControlNode._start_real_climb(node, Q_STAND, True)
    assert node.state == node.RUNNING
    assert node.real_climb_monitor_active
    assert node.climb_start_navigation is not None
    assert node.climb_start_imu_rotation is not None
    assert node.climb_start_planned_pose is not None


def test_y_starts_dock_without_imu_and_rejects_running_climb():
    class DockController:
        APPROACH = "approach"
        CLIMB = "climb"
        DOCK = "dock"

        def __init__(self, climb_state):
            self.mode = self.APPROACH
            self.climb_mode = types.SimpleNamespace(state=climb_state)
            self.entered_q_cur = None

        def enter_dock(self, q_cur):
            self.entered_q_cur = q_cur.copy()
            self.mode = self.DOCK

    node = _button_node()
    node.enable_real_dock = True
    node.controller = DockController(climb_state=ClimbMode.DONE)
    ensured = []
    node._ensure_dock_mode = lambda: ensured.append(True)
    RUN_REAL.RosControlNode._start_real_dock(node, Q_STAND, True)
    assert node.state == node.RUNNING
    assert ensured == [True]
    assert np.array_equal(node.controller.entered_q_cur, Q_STAND)

    node = _button_node()
    node.enable_real_dock = True
    node.controller = DockController(climb_state=ClimbMode.RUNNING)
    node._ensure_dock_mode = lambda: (_ for _ in ()).throw(AssertionError())
    RUN_REAL.RosControlNode._start_real_dock(node, Q_STAND, True)
    assert node.state == node.HOLD
    assert node.controller.entered_q_cur is None


def test_inactive_optional_monitoring_never_holds_diagnostic_replay():
    node = _diagnostic_replay_node(navigation_valid=False, imu_valid=False)
    node.controller.enter_climb(Q_STAND, hardware_execution=True)
    node.state = node.RUNNING
    node.real_climb_monitor_active = False
    hold_calls = []
    node.controller.hold_climb = lambda: hold_calls.append(True)
    RUN_REAL.RosControlNode._monitor_real_climb(node)
    assert node.state == node.RUNNING
    assert hold_calls == []


def test_dock_target_guard_rejects_and_keeps_feedback_pose():
    class Dock:
        active = False

        def __init__(self):
            self.failed_reason = ""

        def enter(self, feet):
            del feet
            self.active = True

        def exit(self):
            self.active = False

        def update(self, state):
            del state
            return types.SimpleNamespace(joint_positions=np.full((6, 3), 9.0))

        def fail_execution(self, reason):
            self.failed_reason = reason

    controller = GraspController(1.0 / 30.0)
    dock = Dock()
    controller.attach_dock_mode(dock)
    controller.enter_dock(Q_STAND)
    q_des = controller.update(Q_STAND, np.zeros(4), dock_robot_state={})
    assert np.array_equal(q_des, Q_STAND)
    assert "joint limits" in dock.failed_reason


def test_l_shaped_ankle_keeps_the_30mm_teleop_swing_moving():
    controller = GraspController(1.0 / 30.0)
    assert np.isclose(controller.approach_mode.step_height, 0.030)
    q_cur = Q_STAND.copy()
    former_false_positive = False
    clearance = (
        LINK_COLLISION_RADII[0]
        + LINK_COLLISION_RADII[2]
        + COLLISION_MARGIN
    )
    for _ in range(5):
        q_cur = controller.update(q_cur, np.array([0.20, 0.0, 0.0, 0.0]))
        points = controller.kinematic.link_points_base(q_cur)
        former_false_positive |= bool(np.any([
            controller._segment_distance(
                points[leg_index, 0], points[leg_index, 1],
                points[leg_index, 2], points[leg_index, 3],
            ) < clearance
            for leg_index in range(6)
        ]))
        assert controller.last_update_collision_guard_hold_count == 0
    assert former_false_positive
    assert controller._same_leg_collision_free(
        controller.kinematic.collision_points_base(q_cur)
    ).all()


def test_l_shaped_ankle_rejects_a_real_deep_fold_within_joint_limits():
    controller = GraspController(1.0 / 30.0)
    q_candidate = Q_STAND.copy()
    q_candidate[0] = [-0.10728169, -1.79814372, -2.04202932]
    assert (q_candidate >= JOINT_LOWER).all()
    assert (q_candidate <= JOINT_UPPER).all()
    collision_points = controller.kinematic.collision_points_base(q_candidate)
    assert not controller._same_leg_collision_free(collision_points)[0]
    assert not controller._link_collision_free(q_candidate)[0]


def test_runtime_climb_settle_gate_uses_15mm_foot_error_and_80mrad_joint_gate():
    config = json.loads(
        (SCRIPTS.parent / "config" / "climb_compact.json").read_text()
    )
    settle_gate = config["settle_gate"]
    assert settle_gate["max_foot_target_error_m"] == 0.015
    assert settle_gate["entry_max_joint_error_rad"] == 0.08
    assert settle_gate["max_joint_tracking_error_rad"] == 0.08


def test_active_climb_feet_use_base_relative_paths_and_audited_clearance():
    config = json.loads(
        (SCRIPTS.parent / "config" / "climb_compact.json").read_text()
    )
    for stage in config["stages"]:
        if not stage["active_legs"]:
            continue
        assert stage["anchor_curve"] in (
            "piecewise_base_quintic",
            "relative_base_high_step",
        )

    pair = np.asarray(config["stages"][3]["active_base_knots_m"])
    pair_clearance = np.max(pair[:, :, 2], axis=0) - np.maximum(
        pair[0, :, 2], pair[-1, :, 2]
    )
    assert (pair_clearance >= 0.10).all()

    rm_symmetry = np.asarray(
        config["stages"][17]["active_base_knots_m"]
    )
    rm_clearance = np.max(rm_symmetry[:, 0, 2]) - max(
        rm_symmetry[0, 0, 2], rm_symmetry[-1, 0, 2]
    )
    assert np.isclose(rm_clearance, 0.04)


def test_hardware_climb_never_advances_on_time_without_settled_feedback():
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode
    mode.phase_time = sum(mode.config["stages"][0]["segment_durations_s"])
    controller.update(Q_STAND + 0.5, np.zeros(4))
    assert mode.stage_index == 0
    assert mode.state == ClimbMode.RUNNING
    assert mode.settle_time == 0.0


def test_climb_observation_uses_start_frame_relative_transforms():
    start_planned = ClimbMode._world_from_base(
        np.array([0.4, -0.2, 0.3, 0.15, -0.2])
    )
    current_base = np.array([0.46, -0.23, 0.31, 0.20, -0.16])
    current_planned = ClimbMode._world_from_base(current_base)
    node = object.__new__(RUN_REAL.RosControlNode)
    node.climb_start_planned_pose = start_planned
    node.climb_start_imu_rotation = start_planned[:3, :3]
    shared_start = np.eye(4)
    shared_start[:3, 3] = [1.0, -0.4, 0.2]
    shared_current = np.eye(4)
    shared_current[:3, 3] = [-0.7, 0.9, -0.1]
    node.climb_start_navigation = types.SimpleNamespace(
        pv_from_base=shared_start @ start_planned,
        pv_from_xiaolan=shared_start,
    )
    node.navigation = types.SimpleNamespace(
        motion_snapshot=lambda: types.SimpleNamespace(
            valid=True,
            pv_from_base=shared_current @ current_planned,
            pv_from_xiaolan=shared_current,
        )
    )
    node.imu = types.SimpleNamespace(
        snapshot=lambda: {
            "valid": True,
            "rotation": current_planned[:3, :3],
            "angular_velocity": np.zeros(3),
        }
    )
    node.controller = types.SimpleNamespace(
        climb_mode=types.SimpleNamespace(base_pose=current_base)
    )
    node.real_climb_max_position_error = 1e-9
    node.real_climb_max_orientation_error = 1e-9
    node.real_climb_max_angular_speed = 1e-9
    assert RUN_REAL.RosControlNode._real_climb_observation(node) == (True, "")
