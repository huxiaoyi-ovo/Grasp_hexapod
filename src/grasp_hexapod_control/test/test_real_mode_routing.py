#!/usr/bin/env python3
"""纯CPU回归：实机模式仲裁和公共关节目标安全门。"""

import importlib.util
from pathlib import Path
import sys
import types

import numpy as np


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))


def _install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logwarn_throttle = lambda *args, **kwargs: None
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


_install_ros_stubs()
RUN_REAL_SPEC = importlib.util.spec_from_file_location(
    "run_real_for_test", SCRIPTS / "run_real.py"
)
RUN_REAL = importlib.util.module_from_spec(RUN_REAL_SPEC)
RUN_REAL_SPEC.loader.exec_module(RUN_REAL)

from climb_mode import ClimbMode
from control import GraspController
from kinematics import Q_STAND


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
    return node


def test_x_y_defaults_are_disabled_and_conflicting_requests_do_not_start():
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
