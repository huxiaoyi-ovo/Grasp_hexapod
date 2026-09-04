#!/usr/bin/env python3
"""纯CPU回归：实机模式仲裁和公共关节目标安全门。"""

import importlib.util
import json
from pathlib import Path
import sys
import threading
from threading import Condition, Lock
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest


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
    rospy.loginfo_throttle = lambda *args, **kwargs: None
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
from kinematics import JOINT_LOWER, JOINT_UPPER, JOINT_VELOCITY_LIMIT, Q_STAND
from utils.climb import select_compact_climb_side
from validate_climb_preview import replay, strict_contract


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
    node.axis_gripper = 6
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


def _bt_node():
    """不初始化ROS的BT服务/租约测试节点。"""

    node = _button_node()
    node.bt_condition = Condition(Lock())
    node.bt_request = None
    node.bt_hold_deadline = 0.0
    node.bt_hold_active = False
    node.bt_hold_lease_s = 0.15
    node.bt_remote = None
    node.gripper_act_proxy = None
    node._switch_mode_response = lambda success, message: types.SimpleNamespace(
        success=success, message=message
    )
    node.local_execution = True
    return node


def _bt_request(mode):
    return {
        "mode": mode,
        "started": True,
        "final": None,
        "waiters": 0,
        "dock_clamped": False,
    }


def test_bt_mode_validation_and_unsupported_modes_fail_closed(monkeypatch):
    node = _bt_node()
    monkeypatch.setattr(RUN_REAL.rospy, "is_shutdown", lambda: False,
                        raising=False)

    unknown = RUN_REAL.RosControlNode._switch_mode_callback(
        node, types.SimpleNamespace(target_mode="bogus")
    )
    unsupported = RUN_REAL.RosControlNode._switch_mode_callback(
        node, types.SimpleNamespace(target_mode="spin_search")
    )

    assert not unknown.success and "unknown mode" in unknown.message
    assert not unsupported.success
    assert unsupported.message == "executor not implemented: spin_search"


def test_bt_same_mode_waiters_share_terminal_and_different_mode_is_busy(monkeypatch):
    node = _bt_node()
    monkeypatch.setattr(RUN_REAL.rospy, "is_shutdown", lambda: False,
                        raising=False)
    replies = []

    def call(mode):
        replies.append(RUN_REAL.RosControlNode._switch_mode_callback(
            node, types.SimpleNamespace(target_mode=mode)
        ))

    first = threading.Thread(target=call, args=("home",))
    second = threading.Thread(target=call, args=("home",))
    first.start()
    second.start()
    for _ in range(100):
        with node.bt_condition:
            if node.bt_request is not None and node.bt_request["waiters"] == 2:
                break
        threading.Event().wait(0.001)
    busy = RUN_REAL.RosControlNode._switch_mode_callback(
        node, types.SimpleNamespace(target_mode="walk")
    )
    request = node.bt_request
    RUN_REAL.RosControlNode._finish_bt_request(node, request, True, "done")
    first.join(timeout=1.0)
    second.join(timeout=1.0)

    assert not busy.success and "busy: home" in busy.message
    assert sorted((item.success, item.message) for item in replies) == [
        (True, "done"), (True, "done")
    ]
    assert node.bt_request is None


def test_b_aborts_bt_before_existing_reset_path():
    node = _bt_node()
    request = _bt_request("climb")
    node.bt_request = request
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 1, 0, 0]), False, Q_STAND
    )

    assert request["final"] == (False, "aborted by B")
    assert node.state == node.RESETTING
    assert node.controller.mission.cancelled


def test_bt_hold_only_freezes_active_request_and_resumes_climb_without_reentry():
    node = _bt_node()
    node.bt_request = _bt_request("climb")
    node.controller.mode = node.controller.CLIMB
    calls = []
    node.controller.hold_climb = lambda: calls.append("hold")
    node.controller.resume_climb = lambda: calls.append("resume")
    node.bt_hold_deadline = 2.0

    assert RUN_REAL.RosControlNode._bt_hold_is_active(node, 1.9)
    assert RUN_REAL.RosControlNode._bt_hold_is_active(node, 1.95)
    assert not RUN_REAL.RosControlNode._bt_hold_is_active(node, 2.01)
    assert calls == ["hold", "resume"]


def test_bt_hold_freezes_approach_without_running_legacy_cancel_path():
    node = _bt_node()
    request = _bt_request("approach")
    node.bt_request = request
    node.bt_hold_deadline = 2.0
    node.state = node.RUNNING
    node.controller = types.SimpleNamespace(
        APPROACH="approach",
        CLIMB="climb",
        DOCK="dock",
        mode="approach",
        q_des=Q_STAND.copy(),
        update=lambda *args: pytest.fail("BT HOLD must not advance ApproachMode"),
    )

    held = RUN_REAL.RosControlNode._update_control(
        node, Q_STAND, np.empty(0), np.zeros(4), 0.0, 1.0,
        feedback_ready=True,
    )

    assert np.array_equal(held, Q_STAND)
    assert node.state == node.RUNNING


def test_bt_gripper_act_maps_response_and_fails_when_missing_or_raising():
    node = _bt_node()
    calls = []
    node.gripper_act_proxy = lambda action: calls.append(action) or types.SimpleNamespace(
        success=True, message="open confirmed"
    )
    assert RUN_REAL.RosControlNode._bt_actuate_gripper(node, "open") == (
        True, "open confirmed"
    )
    assert calls == ["open"]

    node.gripper_act_proxy = None
    assert RUN_REAL.RosControlNode._bt_actuate_gripper(node, "open") == (
        False, "gripper_act service is unavailable"
    )
    node.gripper_act_proxy = lambda action: (_ for _ in ()).throw(
        RuntimeError("offline")
    )
    ok, message = RUN_REAL.RosControlNode._bt_actuate_gripper(node, "clamp")
    assert not ok and "offline" in message


def test_bt_home_resets_before_calling_gripper_act():
    node = _bt_node()
    actions = []
    node.gripper_act_proxy = lambda action: actions.append(action) or types.SimpleNamespace(
        success=True, message="open confirmed"
    )
    request = _bt_request("home")
    request["started"] = False
    node.bt_request = request

    RUN_REAL.RosControlNode._start_bt_request(node, request, Q_STAND)
    assert node.state == node.RESETTING
    assert actions == []

    node.state = node.HOLD
    node.controller.reset_active = False
    RUN_REAL.RosControlNode._finish_bt_mode_if_terminal(node, request)
    assert actions == ["open"]
    assert request["final"] == (True, "open confirmed")


def test_bt_dock_clamps_only_after_dock_terminal_hold():
    node = _bt_node()
    actions = []
    node.gripper_act_proxy = lambda action: actions.append(action) or types.SimpleNamespace(
        success=True, message="clamp confirmed"
    )
    request = _bt_request("dock")
    node.bt_request = request
    node.controller.dock_mode = types.SimpleNamespace(
        SUCCESS="success", FAILED="failed", state="success", reason=""
    )
    node.state = node.RUNNING

    RUN_REAL.RosControlNode._finish_bt_mode_if_terminal(node, request)
    assert actions == []
    assert request["final"] is None

    node.state = node.HOLD
    RUN_REAL.RosControlNode._finish_bt_mode_if_terminal(node, request)
    assert actions == ["clamp"]
    assert request["final"] == (True, "clamp confirmed")


def test_bt_interfaces_register_for_local_execution(monkeypatch):
    node = object.__new__(RUN_REAL.RosControlNode)
    node.subscribers = []
    node.local_execution = True
    events = []
    monkeypatch.setattr(
        RUN_REAL.rospy, "ServiceProxy",
        lambda name, kind: events.append(("proxy", name, kind)) or object(),
        raising=False,
    )
    monkeypatch.setattr(
        RUN_REAL.rospy, "Service",
        lambda name, kind, callback: events.append(("service", name, kind)) or object(),
        raising=False,
    )
    monkeypatch.setattr(
        RUN_REAL.rospy, "Subscriber",
        lambda name, kind, callback, **kwargs: events.append(("sub", name, kind)) or object(),
        raising=False,
    )
    remote_type, string_type, switch_type, response_type, gripper_type = (
        object(), object(), object(), object(), object(),
    )

    RUN_REAL.RosControlNode._register_bt_interfaces(
        node, remote_type, string_type, switch_type, response_type, gripper_type
    )

    assert ("proxy", "/grasp_hexapod/gripper_act", gripper_type) in events
    assert ("service", "/grasp_hexapod/switch_mode", switch_type) in events
    assert ("sub", "/grasp_hexapod/remote_cmd", remote_type) in events
    assert ("sub", "/grasp_hexapod/hold_motion", string_type) in events
    assert len(node.subscribers) == 2


def test_bt_activity_blocks_manual_dpad_gripper_publish():
    node = _bt_node()
    node.local_execution = False
    published = []
    node.gripper_pub = types.SimpleNamespace(publish=lambda message: published.append(message))
    request = _bt_request("approach")
    node.bt_request = request
    node.state = node.RUNNING
    node.controller = types.SimpleNamespace(
        APPROACH="approach",
        CLIMB="climb",
        DOCK="dock",
        mode="approach",
        q_des=Q_STAND.copy(),
        climb_mode=types.SimpleNamespace(state=ClimbMode.IDLE),
        approach_mode=types.SimpleNamespace(
            approach_plan=types.SimpleNamespace(
                failed=False, ready_for_climb=False, reason=""
            )
        ),
        update=lambda *args: Q_STAND.copy(),
    )
    node.navigation = types.SimpleNamespace(snapshot=lambda: "navigation")
    node.control_source = "navigation"
    node._warn_hardware_climb_phase_hold = lambda: None
    node._info_hardware_climb_active_trace = lambda: None
    node._make_command = lambda axes: np.zeros(4)
    axes = np.zeros(7)
    axes[node.axis_gripper] = 1.0

    RUN_REAL.RosControlNode._update_control(
        node, Q_STAND, axes, np.zeros(4), 1.0, 1.0, feedback_ready=True,
    )

    assert published == []


def test_bt_hold_lease_rejects_a_single_30hz_tick_or_less():
    for value in (0.0, 1.0 / 30.0, float("nan")):
        with pytest.raises(ValueError):
            RUN_REAL.RosControlNode._bt_hold_lease(value)
    assert RUN_REAL.RosControlNode._bt_hold_lease(0.15) == 0.15


def test_bt_terminal_mapping_for_auto_modes_and_gripper_failures():
    node = _bt_node()
    node.state = node.HOLD
    node.controller.reset_active = False
    home = _bt_request("home")
    node.gripper_act_proxy = lambda action: types.SimpleNamespace(
        success=False, message="open rejected"
    )
    node.bt_request = home
    RUN_REAL.RosControlNode._finish_bt_mode_if_terminal(node, home)
    assert home["final"] == (False, "open rejected")

    approach = _bt_request("approach")
    node.bt_request = approach
    node.controller.approach_mode = types.SimpleNamespace(
        approach_plan=types.SimpleNamespace(
            failed=False, ready_for_climb=True, reason=""
        )
    )
    node.state = node.RUNNING
    RUN_REAL.RosControlNode._finish_bt_mode_if_terminal(node, approach)
    assert approach["final"] == (True, "ready for climb")
    assert node.state == node.HOLD


def test_bt_automatic_approach_advances_without_fresh_joy():
    node = _bt_node()
    request = _bt_request("approach")
    node.bt_request = request
    node.state = node.RUNNING
    updates = []
    node.controller = types.SimpleNamespace(
        APPROACH="approach",
        CLIMB="climb",
        DOCK="dock",
        mode="approach",
        q_des=Q_STAND.copy(),
        climb_mode=types.SimpleNamespace(state=ClimbMode.IDLE),
        approach_mode=types.SimpleNamespace(
            approach_plan=types.SimpleNamespace(
                failed=False, ready_for_climb=False, reason=""
            )
        ),
        update=lambda *args: updates.append(args) or Q_STAND.copy(),
    )
    node.navigation = types.SimpleNamespace(snapshot=lambda: "navigation")
    node.control_source = "navigation"
    node._warn_hardware_climb_phase_hold = lambda: None
    node._info_hardware_climb_active_trace = lambda: None
    node._make_command = lambda axes: np.zeros(4)

    RUN_REAL.RosControlNode._update_control(
        node, Q_STAND, np.empty(0), np.zeros(4), 0.0, 1.0,
        feedback_ready=True,
    )

    assert node.state == node.RUNNING
    assert updates and updates[0][2] == "navigation"


def _bt_walk_node():
    node = _bt_node()
    request = _bt_request("walk")
    node.bt_request = request
    node.state = node.RUNNING
    commands = []
    stopped = []
    node.controller = types.SimpleNamespace(
        APPROACH="approach",
        CLIMB="climb",
        DOCK="dock",
        mode="approach",
        q_des=Q_STAND.copy(),
        climb_mode=types.SimpleNamespace(state=ClimbMode.IDLE),
        approach_mode=types.SimpleNamespace(
            cancel_autonomous_approach=lambda reason: stopped.append(reason),
        ),
        hold_climb=lambda: stopped.append("hold_climb"),
        update=lambda q_cur, command, *args: commands.append(command.copy()) or Q_STAND.copy(),
    )
    node.control_source = "navigation"
    node._make_command = lambda axes: np.array([0.12, -0.03, 0.0, 0.4])
    node._warn_hardware_climb_phase_hold = lambda: None
    node._info_hardware_climb_active_trace = lambda: None
    return node, request, commands, stopped


def test_bt_walk_stale_joy_zeros_command_and_fails_service():
    node, request, commands, stopped = _bt_walk_node()

    RUN_REAL.RosControlNode._update_control(
        node, Q_STAND, np.ones(4), np.zeros(4), 0.1, 1.0,
        feedback_ready=True,
    )

    assert commands and np.array_equal(commands[-1], np.zeros(4))
    assert request["final"] == (False, "joystick lost")
    assert node.state == node.HOLD
    assert stopped == ["joystick lost", "hold_climb"]


def test_bt_walk_fresh_joy_keeps_raw_command_path():
    node, request, commands, stopped = _bt_walk_node()
    node.bt_remote = types.SimpleNamespace(mode="walk", reset_edge=False)

    RUN_REAL.RosControlNode._update_control(
        node, Q_STAND, np.ones(4), np.zeros(4), 1.0, 1.0,
        feedback_ready=True,
    )

    np.testing.assert_allclose(commands[-1], [0.12, -0.03, 0.0, 0.4])
    assert request["final"] is None
    assert stopped == []


def test_run_real_launch_enables_climb_by_default():
    launch = ET.parse(SCRIPTS.parent / "launch" / "run_real.launch")
    arguments = {
        argument.attrib["name"]: argument.attrib.get("default")
        for argument in launch.findall("arg")
    }
    assert arguments["enable_real_climb"] == "true"
    assert arguments["enable_link_collision_check"] == "true"


def test_real_navigation_uses_current_climb_geometry_source():
    source = Path(RUN_REAL.__file__).read_text()

    assert "derive_compact_approach_geometry(compact)" in source
    assert "navigation_route_config" not in source
    assert "approach_navigation.yaml" not in source


def test_navigation_snapshot_rejects_pose_skew_and_stale_boundary():
    navigation = object.__new__(RUN_REAL.NavigationInput)
    navigation.max_age = 0.5
    navigation.max_pose_skew = 0.2
    navigation.max_boundary_age = 1.0
    navigation.lock = Lock()
    navigation.base_stamp = 1.0
    navigation.xiaolan_stamp = 0.7
    navigation.boundary_stamp = 1.0
    navigation.pv_from_base = np.eye(4)
    navigation.pv_from_xiaolan = np.eye(4)
    navigation.pv_boundary = np.array(
        [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]
    )
    navigation.landing_confirmed = True

    assert not navigation.snapshot().valid
    navigation.xiaolan_stamp = 1.0
    assert navigation.snapshot().valid
    navigation.boundary_stamp = -0.1
    assert not navigation.snapshot().valid


def _load_compact_config():
    return json.loads(
        (SCRIPTS.parent / "config" / "climb_compact.json").read_text()
    )


def _assert_nested_allclose(actual, expected):
    if isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in expected:
            _assert_nested_allclose(actual[key], expected[key])
    elif isinstance(expected, list):
        assert len(actual) == len(expected)
        for actual_item, expected_item in zip(actual, expected):
            _assert_nested_allclose(actual_item, expected_item)
    elif isinstance(expected, float):
        assert np.isclose(actual, expected, rtol=0.0, atol=1e-12)
    else:
        assert actual == expected


def test_compact_left_selection_is_independent_and_unchanged():
    source = _load_compact_config()
    selected = select_compact_climb_side(source, "left")
    assert selected == source
    assert selected is not source
    selected["p0"]["base"][0] += 1.0
    assert selected["p0"]["base"][0] != source["p0"]["base"][0]


def test_compact_right_selection_mirrors_runtime_fields_only():
    source = _load_compact_config()
    source_before = json.dumps(source, sort_keys=True)
    right = select_compact_climb_side(source, "right")
    assert json.dumps(source, sort_keys=True) == source_before
    center_x = source["xiaolan_translation"][0]
    order = [3, 4, 5, 0, 1, 2]
    index_map = {0: 3, 1: 4, 2: 5, 3: 0, 4: 1, 5: 2}

    assert np.array_equal(right["p0"]["q_rad"], source["p0"]["q_rad"])
    assert np.isclose(right["p0"]["base"][0], 2.0 * center_x - source["p0"]["base"][0])
    assert np.isclose(right["p0"]["base"][3], -source["p0"]["base"][3])
    np.testing.assert_allclose(
        np.asarray(right["p0"]["anchors_world_m"]),
        np.column_stack((
            2.0 * center_x - np.asarray(source["p0"]["anchors_world_m"])[order, 0],
            np.asarray(source["p0"]["anchors_world_m"])[order, 1:],
        )),
    )
    np.testing.assert_allclose(
        np.asarray(right["terminal_q_rad"]),
        -np.asarray(source["terminal_q_rad"])[order],
    )
    assert right["settle_gate"] == source["settle_gate"]

    for original, mirrored in zip(source["stages"], right["stages"]):
        assert np.isclose(mirrored["pose_start"][0], 2.0 * center_x - original["pose_start"][0])
        assert np.isclose(mirrored["pose_end"][0], 2.0 * center_x - original["pose_end"][0])
        assert np.isclose(mirrored["pose_start"][3], original["pose_start"][3])
        assert np.isclose(mirrored["pose_end"][3], original["pose_end"][3])
        assert np.isclose(mirrored["pose_start"][4], -original["pose_start"][4])
        assert np.isclose(mirrored["pose_end"][4], -original["pose_end"][4])
        original_knots = np.asarray(original["anchor_knots"])
        expected_knots = original_knots[:, order, :].copy()
        expected_knots[:, :, 0] = 2.0 * center_x - expected_knots[:, :, 0]
        np.testing.assert_allclose(mirrored["anchor_knots"], expected_knots)
        assert mirrored["active_legs"] == [
            index_map[index] for index in original["active_legs"]
        ]
        if "active_base_knots_m" in original:
            np.testing.assert_allclose(
                np.asarray(mirrored["active_base_knots_m"])[:, :, 0],
                -np.asarray(original["active_base_knots_m"])[:, :, 0],
            )
        else:
            assert "active_base_knots_m" not in mirrored
        assert mirrored["segment_durations_s"] == original["segment_durations_s"]
        assert mirrored["settle_s"] == original["settle_s"]

    assert right["stages"][17]["name"] == "LM_LEFT_SYMMETRY"
    assert right["visual_validation_deferred_for_sim_finish"][17] == "LM_LEFT_SYMMETRY"
    assert source["front_v1_receipt"] == right["front_v1_receipt"]
    assert source["source_traces"] == right["source_traces"]


def test_compact_right_selection_is_involutive_and_rejects_invalid_side():
    source = _load_compact_config()
    restored = select_compact_climb_side(
        select_compact_climb_side(source, "right"), "right"
    )
    _assert_nested_allclose(restored, source)
    with pytest.raises(ValueError, match="left or right"):
        select_compact_climb_side(source, "RIGHT")


def test_compact_left_and_right_full_replays_are_kinematically_symmetric():
    source = _load_compact_config()
    left = select_compact_climb_side(source, "left")
    right = select_compact_climb_side(source, "right")
    strict_contract(left)

    left_report = replay(left, strict=True)
    right_report = replay(right, strict=False)

    assert left_report["state"] == right_report["state"] == ClimbMode.DONE
    assert left_report["ticks"] == right_report["ticks"]
    assert np.isclose(
        left_report["min_joint_margin_rad"],
        right_report["min_joint_margin_rad"],
        atol=2e-8,
    )
    assert np.isclose(
        left_report["global_peak_command_speed_rad_s"],
        right_report["global_peak_command_speed_rad_s"],
        atol=2e-8,
    )


def test_climb_side_launch_and_real_routing_are_explicit():
    launch_dir = SCRIPTS.parent / "launch"
    for name in (
        "control_stack.launch",
        "run_real.launch",
        "run_sim_ros.launch",
        "run_sim_ros_climb.launch",
    ):
        root = ET.parse(launch_dir / name).getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in root.findall("arg")
        }
        assert arguments["climb_side"] == "left"
    sim_root = ET.parse(launch_dir / "run_sim_ros.launch").getroot()
    sim_nodes = [item for item in sim_root.iter("node") if item.attrib.get("type") == "run_sim.py"]
    assert len(sim_nodes) == 4
    assert all("--climb-side $(arg climb_side)" in item.attrib["args"] for item in sim_nodes)
    climb_root = ET.parse(launch_dir / "run_sim_ros_climb.launch").getroot()
    assert any(
        item.attrib.get("name") == "climb_side"
        and item.attrib.get("value") == "$(arg climb_side)"
        for item in climb_root.iter("arg")
    )
    control_root = ET.parse(launch_dir / "control_stack.launch").getroot()
    assert any(
        item.attrib.get("name") == "climb_side"
        and item.attrib.get("value") == "$(arg climb_side)"
        for item in control_root.iter("param")
    )
    real_root = ET.parse(launch_dir / "run_real.launch").getroot()
    assert any(
        item.attrib.get("name") == "climb_side"
        and item.attrib.get("value") == "$(arg climb_side)"
        for item in real_root.iter("arg")
    )
    source = (SCRIPTS / "run_real.py").read_text()
    assert 'rospy.get_param("~climb_side", "left")' in source
    assert "select_compact_climb_side(config, climb_side)" in source
    assert "--full-mission requires --climb-side left" in (
        SCRIPTS / "run_sim.py"
    ).read_text()


def test_real_launches_enable_dock_by_default():
    for launch_name in ("run_real.launch", "control_stack.launch"):
        launch = ET.parse(SCRIPTS.parent / "launch" / launch_name)
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in launch.findall("arg")
        }
        assert arguments["enable_real_dock"] == "true"
        assert arguments["max_feedback_age"] == "0.30"
        assert arguments["max_feedback_skew"] == "0.20"


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


def test_real_main_polls_four_times_per_control_step_without_changing_frame_gate():
    source = (SCRIPTS / "run_real.py").read_text()
    assert "self.poll_rate_hz = self.rate_hz * 4.0" in source
    assert "rate = rospy.Rate(node.poll_rate_hz)" in source
    assert "and (feedback_stamp > last_control_feedback_stamp).all()" in source


def test_dual_board_frame_rejects_excessive_snapshot_skew():
    q_cur = np.zeros((6, 3), dtype=np.float64)
    stamps = np.array([10.00, 10.01, 10.02, 10.11, 10.12, 10.13])
    ready, complete = RUN_REAL.RosControlNode._feedback_frame_state(
        q_cur,
        stamps,
        np.zeros(6),
        now=10.14,
        max_feedback_age=0.15,
        max_feedback_skew=0.10,
    )
    assert not ready
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


def test_local_compact_arm_holds_entry_and_a_is_ignored():
    node = object.__new__(RUN_REAL.RosControlNode)
    node.local_execution = True
    node.HOLD = RUN_REAL.RosControlNode.HOLD
    node.WAIT_B = RUN_REAL.RosControlNode.WAIT_B
    node.state = node.WAIT_B
    node.command = np.ones(4)
    node.controller = types.SimpleNamespace(
        q_des=np.zeros((6, 3)), reset_active=True
    )
    node.max_joy_age = 0.2
    node.button_a = 0
    node.button_b = 1
    node.button_x = 2
    node.button_y = 3
    node._read = RUN_REAL.RosControlNode._read
    node._process_buttons = RUN_REAL.RosControlNode._process_buttons.__get__(node)

    RUN_REAL.RosControlNode.arm_local_climb(node, Q_STAND)
    assert node.state == node.HOLD
    assert node.local_climb_armed
    assert np.array_equal(node.local_climb_entry_q, Q_STAND)

    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([1, 0, 0, 0]), True, Q_STAND
    )
    assert node.local_climb_armed
    held = RUN_REAL.RosControlNode._update_control(
        node, Q_STAND + 0.1, np.empty(0), np.zeros(4), 1.0, 1.0
    )
    assert np.array_equal(held, Q_STAND)
    with pytest.raises(ValueError, match="finite"):
        RUN_REAL.RosControlNode.arm_local_climb(
            node, np.full((6, 3), np.nan)
        )


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
    terminal_flushes = []
    node._monitor_real_climb = lambda: monitor_calls.append(True)
    node._flush_real_climb_speed_diagnostic = lambda reason: terminal_flushes.append(reason)
    node._hold_motion = lambda *args, **kwargs: None
    node._make_command = lambda axes: np.zeros(4)
    node.control_source = "teleop"
    node._update_control(
        Q_STAND, np.empty(0), np.zeros(4), 1.0, 1.0, feedback_ready=True
    )
    assert node.state == node.HOLD
    assert monitor_calls == [True]
    assert terminal_flushes == ["terminal"]

    node._update_control(
        Q_STAND, np.empty(0), np.zeros(4), 1.0, 1.0, feedback_ready=True
    )
    assert terminal_flushes == ["terminal"]

    calls = []
    node._start_real_dock = lambda *args: calls.append("dock")
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 0, 0, 1]), True, Q_STAND
    )
    assert calls == ["dock"]


def test_dock_terminal_logs_only_on_first_running_to_hold(monkeypatch):
    node = _button_node()
    node.state = node.RUNNING
    node.controller = types.SimpleNamespace(
        APPROACH="approach",
        CLIMB="climb",
        DOCK="dock",
        mode="dock",
        reset_active=False,
        dock_mode=types.SimpleNamespace(
            state="DONE",
            TERMINAL_STATES=("DONE",),
            reason="alignment complete",
        ),
        update=lambda *args: Q_STAND.copy(),
    )
    node._dock_lock_confirmed = lambda: False
    node._warn_hardware_climb_phase_hold = lambda: None
    node._info_hardware_climb_active_trace = lambda: None
    node._make_command = lambda axes: np.zeros(4)
    node._hold_motion = lambda *args, **kwargs: None
    node.control_source = "teleop"
    messages = []
    monkeypatch.setattr(
        RUN_REAL.rospy, "loginfo", lambda *args: messages.append(args)
    )

    for _ in range(2):
        node._update_control(
            Q_STAND, np.empty(0), np.zeros(4), 1.0, 1.0,
            feedback_ready=True,
        )

    terminal_messages = [
        item for item in messages if item[0].startswith("DockMode terminal HOLD")
    ]
    assert node.state == node.HOLD
    assert terminal_messages == [("DockMode terminal HOLD: %s", "alignment complete")]


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
        self.climb_mode = types.SimpleNamespace(
            base_pose=None,
            _load_config=lambda: {"settle_gate": {"max_foot_target_error_m": 0.02}},
        )
        self.entered_hardware_execution = None
        self.entered_config = None

    def enter_climb(self, q_cur, config=None, hardware_execution=False):
        del q_cur
        self.entered_hardware_execution = hardware_execution
        self.entered_config = config
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
    node.local_climb_armed = True
    node.local_climb_entry_q = Q_STAND.copy()
    RUN_REAL.RosControlNode._start_real_climb(node, Q_STAND, True)
    assert node.state == node.RUNNING
    assert node.controller.entered_hardware_execution is True
    assert node.controller.entered_config["settle_gate"]["max_foot_target_error_m"] == 0.02
    assert not node.local_climb_armed
    assert node.local_climb_entry_q is None
    assert not node.real_climb_monitor_active


def test_real_climb_override_gate_and_validation():
    node = _diagnostic_replay_node(navigation_valid=False, imu_valid=False)
    node.climb_foot_gate_m = 0.05
    RUN_REAL.RosControlNode._start_real_climb(node, Q_STAND, True)
    assert node.controller.entered_hardware_execution is True
    assert node.controller.entered_config["settle_gate"]["max_foot_target_error_m"] == 0.05
    for value in (0.0, float("nan"), 0.101):
        with pytest.raises(ValueError):
            RUN_REAL.RosControlNode._climb_foot_gate_m(value)


def test_b_cancels_local_compact_arm_before_normal_reset():
    node = _button_node()
    node.local_execution = True
    node.local_climb_armed = True
    node.local_climb_entry_q = Q_STAND.copy()
    RUN_REAL.RosControlNode._process_buttons(
        node, np.array([0, 1, 0, 0]), False, Q_STAND
    )
    assert not node.local_climb_armed
    assert node.local_climb_entry_q is None
    assert node.state == node.RESETTING


def test_x_enables_optional_monitoring_when_imu_and_rtk_are_fresh():
    node = _diagnostic_replay_node(navigation_valid=True, imu_valid=True)
    RUN_REAL.RosControlNode._start_real_climb(node, Q_STAND, True)
    assert node.state == node.RUNNING
    assert node.real_climb_monitor_active
    assert node.climb_start_navigation is not None
    assert node.climb_start_imu_rotation is not None
    assert node.climb_start_planned_pose is not None


def test_y_starts_dock_without_requiring_climb_completion():
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
    ensured = []
    node._ensure_dock_mode = lambda: ensured.append(True)
    RUN_REAL.RosControlNode._start_real_dock(node, Q_STAND, True)
    assert node.state == node.RUNNING
    assert ensured == [True]
    assert np.array_equal(node.controller.entered_q_cur, Q_STAND)


def test_b_then_y_uses_saved_climb_terminal_joints():
    class Dock:
        active = False

        def __init__(self):
            self.current = None
            self.terminal = None

        def enter(self, current, terminal):
            self.current = current.copy()
            self.terminal = terminal.copy()
            self.active = True

    controller = GraspController(1.0 / 30.0)
    dock = Dock()
    controller.attach_dock_mode(dock)
    saved_terminal = controller.climb_terminal_q.copy()

    controller.reset_to_stand(Q_STAND)
    controller.reset_active = False
    controller.enter_dock(Q_STAND)

    assert np.array_equal(dock.current, Q_STAND)
    assert np.array_equal(dock.terminal, saved_terminal)
    assert not np.array_equal(saved_terminal[2], Q_STAND[2])
    assert not np.array_equal(saved_terminal[5], Q_STAND[5])


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


def test_dock_invalid_joint_target_rejects_and_keeps_feedback_pose():
    class Dock:
        active = False

        def __init__(self):
            self.failed_reason = ""

        def enter(self, current, terminal):
            del current, terminal
            self.active = True

        def exit(self):
            self.active = False

        def update(self, state):
            del state
            return types.SimpleNamespace(
                joint_positions=np.full((6, 3), np.nan),
                foot_positions_base=None,
            )

        def fail_execution(self, reason):
            self.failed_reason = reason

    controller = GraspController(1.0 / 30.0)
    dock = Dock()
    controller.attach_dock_mode(dock)
    controller.enter_dock(Q_STAND)
    q_des = controller.update(Q_STAND, np.zeros(4), dock_robot_state={})
    assert np.array_equal(q_des, Q_STAND)
    assert "non-finite" in dock.failed_reason


def test_dock_foot_target_uses_the_shared_dls_chain():
    class Dock:
        active = False

        def __init__(self, target_feet):
            self.target_feet = target_feet
            self.failed_reason = ""

        def enter(self, current, terminal):
            del current, terminal
            self.active = True

        def exit(self):
            self.active = False

        def update(self, state):
            del state
            return types.SimpleNamespace(
                joint_positions=None,
                foot_positions_base=self.target_feet,
            )

        def fail_execution(self, reason):
            self.failed_reason = reason

    controller = GraspController(1.0 / 30.0)
    feet_target = controller.kinematic.forward_base(Q_STAND)
    feet_target[0, 0] += 0.001
    dock = Dock(feet_target)
    controller.attach_dock_mode(dock)
    controller.enter_dock(Q_STAND)

    q_des = controller.update(Q_STAND, np.zeros(4), dock_robot_state={})

    assert np.array_equal(controller.foot_desired_base, feet_target)
    assert not np.array_equal(q_des[0], Q_STAND[0])
    assert np.allclose(q_des[1:], Q_STAND[1:], atol=1e-12)
    assert dock.failed_reason == ""


def test_ordinary_walking_uses_40mm_swing_and_lands_before_stop():
    controller = GraspController(1.0 / 30.0)
    mode = controller.approach_mode
    assert np.isclose(mode.step_height, 0.040)
    assert np.isclose(mode.phase_duration, 0.70)
    q_cur = Q_STAND.copy()
    maximum_lift = 0.0
    minimum_joint_margin = np.inf
    for _ in range(25):
        q_cur = controller.update(q_cur, np.array([0.20, 0.0, 0.0, 0.0]))
        ground_z = controller.foot_init_base[:, 2] - mode.body_height_offset
        maximum_lift = max(
            maximum_lift,
            float(np.max(controller.foot_desired_base[:, 2] - ground_z)),
        )
        minimum_joint_margin = min(
            minimum_joint_margin,
            float(np.min(q_cur - JOINT_LOWER)),
            float(np.min(JOINT_UPPER - q_cur)),
        )
        assert controller.last_update_collision_guard_hold_count == 0
    assert maximum_lift >= 0.0399
    assert minimum_joint_margin > 0.02

    for _ in range(30):
        q_cur = controller.update(q_cur, np.zeros(4))
        if not mode.gait_started:
            break
    assert not mode.gait_started
    assert not mode.transfer_active
    assert np.allclose(mode.requested_command, 0.0)
    assert np.allclose(mode.active_phase_command, 0.0)
    assert np.allclose(
        controller.foot_desired_base[:, 2],
        controller.foot_init_base[:, 2],
    )


def test_walking_command_switch_is_slew_limited_and_latches_active_phase():
    controller = GraspController(1.0 / 30.0)
    mode = controller.approach_mode
    q_cur = Q_STAND.copy()
    commands = (
        [np.array([0.20, 0.0, 0.0, 0.0])] * 8
        + [np.array([-0.20, 0.0, 0.0, 0.0])] * 8
        + [np.array([0.0, 0.20, 0.0, 1.2])] * 8
    )
    previous_requested_command = mode.requested_command.copy()
    previous_target = controller.foot_desired_base.copy()
    previous_q = q_cur.copy()
    maximum_target_step = 0.0
    maximum_joint_speed = 0.0
    minimum_joint_margin = np.inf
    active_endpoint = None
    requested_changed_while_active = False

    for index, command in enumerate(commands):
        q_cur = controller.update(q_cur, command)
        requested_command = mode.requested_command.copy()
        linear_acceleration = np.linalg.norm(
            requested_command[:2] - previous_requested_command[:2]
        ) / controller.dt
        yaw_acceleration = abs(
            requested_command[3] - previous_requested_command[3]
        ) / controller.dt
        assert linear_acceleration <= mode.max_linear_acceleration + 1e-10
        assert yaw_acceleration <= mode.max_yaw_acceleration + 1e-10
        assert (
            np.linalg.norm(requested_command[:2])
            + mode.nominal_foot_radius * abs(requested_command[3])
            <= mode.max_foot_planar_speed + 1e-10
        )
        if index == 0:
            assert np.array_equal(
                mode.active_phase_command, requested_command
            )
            assert requested_command[0] < command[0]
        if index == 1:
            active_endpoint = mode.swing_target_base.copy()
        if index >= 8 and mode.stance_group_index == 0:
            assert np.array_equal(mode.swing_target_base, active_endpoint)
            requested_changed_while_active |= not np.array_equal(
                requested_command, mode.active_phase_command
            )
        maximum_target_step = max(
            maximum_target_step,
            float(np.max(np.linalg.norm(
                controller.foot_desired_base - previous_target,
                axis=1,
            ))),
        )
        maximum_joint_speed = max(
            maximum_joint_speed,
            float(np.max(np.abs(q_cur - previous_q))) / controller.dt,
        )
        minimum_joint_margin = min(
            minimum_joint_margin,
            float(np.min(q_cur - JOINT_LOWER)),
            float(np.min(JOINT_UPPER - q_cur)),
        )
        previous_requested_command = requested_command
        previous_target = controller.foot_desired_base.copy()
        previous_q = q_cur.copy()

    assert active_endpoint is not None
    assert requested_changed_while_active
    assert maximum_target_step <= 0.012
    assert maximum_joint_speed <= float(np.max(JOINT_VELOCITY_LIMIT)) + 1e-10
    assert minimum_joint_margin > 0.02
    assert np.max(np.abs(q_cur - Q_STAND)) > 0.01


def test_84_frame_abrupt_command_witness_remains_continuous_and_safe():
    controller = GraspController(1.0 / 30.0)
    q_cur = Q_STAND.copy()
    commands = (
        [np.array([0.20, 0.0, 0.0, 0.0])] * 18
        + [np.array([-0.20, 0.0, 0.0, 0.0])] * 18
        + [np.array([0.0, 0.20, 0.0, 1.2])] * 18
        + [np.zeros(4)] * 30
    )
    maximum_target_step = 0.0
    velocity_clips = 0
    minimum_joint_margin = np.inf
    offline_link_collision_frames = 0

    for command in commands:
        previous_target = controller.foot_desired_base.copy()
        q_cur = controller.update(q_cur, command)
        maximum_target_step = max(
            maximum_target_step,
            float(np.max(np.linalg.norm(
                controller.foot_desired_base - previous_target,
                axis=1,
            ))),
        )
        velocity_clips += controller.last_update_velocity_limit_clip_count
        minimum_joint_margin = min(
            minimum_joint_margin,
            float(np.min(q_cur - JOINT_LOWER)),
            float(np.min(JOINT_UPPER - q_cur)),
        )
        offline_link_collision_frames += int(
            not controller._link_collision_free(q_cur).all()
        )

    assert len(commands) == 84
    assert maximum_target_step <= 0.013
    assert velocity_clips <= 7
    assert minimum_joint_margin > 0.02
    assert offline_link_collision_frames == 0
    assert not controller.approach_mode.gait_started
    assert not controller.approach_mode.transfer_active


def test_approach_skips_full_link_scan_but_climb_retains_it(monkeypatch):
    controller = GraspController(
        1.0 / 30.0,
        enable_link_collision_check=True,
    )
    assert controller.enable_workspace_check
    calls = []

    def scan(joint_angles):
        calls.append(np.asarray(joint_angles).copy())
        return np.ones(6, dtype=bool)

    monkeypatch.setattr(controller, "_link_collision_free", scan)
    q_candidate = Q_STAND.copy()
    assert np.array_equal(
        controller.collision_guard(q_candidate, Q_STAND),
        q_candidate,
    )
    assert calls == []

    controller.mode = controller.CLIMB
    assert np.array_equal(
        controller.collision_guard(q_candidate, Q_STAND),
        q_candidate,
    )
    assert len(calls) == 1


def test_rejected_approach_candidate_keeps_phase_and_transfer_clocks(monkeypatch):
    controller = GraspController(1.0 / 30.0)
    mode = controller.approach_mode
    desired_before = controller.foot_desired_base.copy()
    original_foot_collision_free = controller._foot_collision_free
    monkeypatch.setattr(
        controller,
        "_foot_collision_free",
        lambda candidate: np.zeros(6, dtype=bool),
    )
    assert not controller._commit_workspace_candidate(desired_before)
    assert np.array_equal(controller.foot_desired_base, desired_before)
    monkeypatch.setattr(
        controller, "_foot_collision_free", original_foot_collision_free
    )

    q_cur = controller.update(Q_STAND, np.array([0.20, 0.0, 0.0, 0.0]))
    phase_time = mode.phase_time
    velocity = mode.foot_velocity_xy.copy()
    monkeypatch.setattr(
        controller, "_commit_workspace_candidate", lambda candidate: False
    )
    controller.update(q_cur, np.array([0.20, 0.0, 0.0, 0.0]))
    assert mode.phase_time == phase_time
    assert np.array_equal(mode.foot_velocity_xy, velocity)
    assert mode._commit_candidate(controller.foot_desired_base.copy()) is False

    mode.transfer_active = True
    mode.transfer_time = 0.0
    transfer_velocity = mode.foot_velocity_xy.copy()
    controller.update(q_cur, np.array([0.20, 0.0, 0.0, 0.0]))
    assert mode.transfer_time == 0.0
    assert np.array_equal(mode.foot_velocity_xy, transfer_velocity)


def test_l_shaped_ankle_rejects_a_real_deep_fold_within_joint_limits():
    controller = GraspController(1.0 / 30.0)
    q_candidate = Q_STAND.copy()
    q_candidate[0] = [-0.10728169, -1.79814372, -2.04202932]
    assert (q_candidate >= JOINT_LOWER).all()
    assert (q_candidate <= JOINT_UPPER).all()
    collision_points = controller.kinematic.collision_points_base(q_candidate)
    assert not controller._same_leg_collision_free(collision_points)[0]
    assert not controller._link_collision_free(q_candidate)[0]


def test_runtime_climb_settle_gate_uses_20mm_foot_error_and_80mrad_joint_gate():
    config = json.loads(
        (SCRIPTS.parent / "config" / "climb_compact.json").read_text()
    )
    settle_gate = config["settle_gate"]
    assert settle_gate["max_foot_target_error_m"] == 0.020
    assert settle_gate["entry_max_joint_error_rad"] == 0.08
    assert settle_gate["max_joint_tracking_error_rad"] == 0.08
    assert settle_gate["timeout_s"] == 5.0


def test_real_launch_uses_temporary_5cm_gate_and_control_stack_keeps_2cm():
    launches = {}
    for name in ("run_real.launch", "control_stack.launch"):
        root = ET.parse(SCRIPTS.parent / "launch" / name).getroot()
        launches[name] = {
            item.attrib["name"]: item.attrib.get("default")
            for item in root.findall("arg")
        }
    assert launches["run_real.launch"]["climb_foot_gate_m"] == "0.05"
    assert launches["control_stack.launch"]["climb_foot_gate_m"] == "0.02"
    control_root = ET.parse(
        SCRIPTS.parent / "launch" / "control_stack.launch"
    ).getroot()
    assert any(
        item.attrib.get("name") == "climb_foot_gate_m"
        and item.attrib.get("value") == "$(arg climb_foot_gate_m)"
        for item in control_root.iter("param")
    )


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
    assert mode.last_phase_hold


def test_hardware_climb_endpoint_phase_hold_clears_when_feedback_settles():
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode
    duration = sum(mode.config["stages"][0]["segment_durations_s"])
    mode.phase_time = duration
    controller.update(Q_STAND + 0.5, np.zeros(4))
    assert mode.last_phase_hold
    controller.q_des = Q_STAND.copy()
    controller.foot_desired_base[:] = controller.kinematic.forward_base(Q_STAND)
    controller.update(Q_STAND, np.zeros(4))
    assert not mode.last_phase_hold


def test_hardware_climb_uses_foot_task_error_not_joint_tracking_for_phase():
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode

    # 模拟负载下存在关节稳态偏差，但实际 FK 足端仍在当前任务目标。
    controller.q_des = Q_STAND + 0.5
    controller.foot_desired_base[:] = controller.kinematic.forward_base(Q_STAND)
    controller.update(Q_STAND, np.zeros(4))

    assert mode.last_tracking_error_rad > mode.config["settle_gate"][
        "max_joint_tracking_error_rad"
    ]
    assert mode.last_foot_target_error_m <= mode.config["settle_gate"][
        "max_foot_target_error_m"
    ]
    assert mode.phase_time == controller.dt


def test_hardware_climb_only_holds_fk_error_at_segment_checkpoints():
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode
    command = np.zeros(4)

    controller.update(Q_STAND, command)
    moving_phase = mode.phase_time
    # C1 has one segment: a large FK error inside it must not interrupt motion.
    q_des = controller.update(Q_STAND + 0.5, command)
    assert mode.last_foot_target_error_m > mode.config["settle_gate"][
        "max_foot_target_error_m"
    ]
    assert mode.phase_time > moving_phase
    assert np.isfinite(q_des).all()

    # C2 RM has an existing lift checkpoint. There the same error holds exactly
    # at the knot, preserving the checkpoint reference until feedback recovers.
    mode.stage_index = 1
    mode.phase = mode.stage_names[1]
    mode.phase_time = mode.config["stages"][1]["segment_durations_s"][0]
    mode.stage_elapsed_time = mode.phase_time
    base, anchors, _ = mode._stage_reference()
    mode._apply_reference(base, anchors, sync_previous=True)
    reference_before_hold = controller.foot_desired_base.copy()
    controller.update(Q_STAND + 0.5, command)
    assert mode.last_foot_target_error_m > mode.config["settle_gate"][
        "max_foot_target_error_m"
    ]
    assert mode.phase_time == mode.config["stages"][1]["segment_durations_s"][0]
    assert np.array_equal(controller.foot_desired_base, reference_before_hold)

    controller.q_des = Q_STAND.copy()
    controller.foot_desired_base[:] = controller.kinematic.forward_base(Q_STAND)
    controller.update(Q_STAND, command)
    assert mode.last_foot_target_error_m <= mode.config["settle_gate"][
        "max_foot_target_error_m"
    ]
    assert mode.phase_time > mode.config["stages"][1]["segment_durations_s"][0]


def test_hardware_climb_diagnostics_keep_all_feet_and_motors():
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode
    mode.stage_index = 3
    mode.phase = mode.stage_names[mode.stage_index]
    mode.phase_time = 0.2
    mode.stage_elapsed_time = 0.7
    controller.foot_desired_base[:] = controller.kinematic.forward_base(Q_STAND)
    controller.foot_desired_base[0, 0] += 0.03
    controller.foot_desired_base[1, 1] += 0.04
    controller.q_des = Q_STAND.copy()
    controller.q_des[2, 1] += 0.10
    controller.q_des[4, 2] += 0.12

    mode._update_tracking_diagnostics(Q_STAND)

    assert mode.last_worst_foot_leg == "lf"
    assert mode.last_worst_joint_leg == "rf"
    assert mode.last_worst_joint_name == "ankle"
    assert tuple(name for name, _ in mode.last_feet_over_gate) == ("lb", "lf")
    assert np.allclose(
        tuple(value for _, value in mode.last_feet_over_gate), (0.03, 0.04)
    )
    assert tuple(name for name, _ in mode.last_joints_over_tracking_gate) == (
        "lm_knee",
        "rf_ankle",
    )
    assert np.allclose(
        tuple(value for _, value in mode.last_joints_over_tracking_gate),
        (0.10, 0.12),
    )
    summary = mode.tracking_diagnostic_summary()
    assert "feet_over_gate=lb=0.03,lf=0.04" in summary
    assert "feet_over_gate_base_link_xyz=lb[actual_base_xyz_m=" in summary
    assert ";lf[actual_base_xyz_m=" in summary
    assert "error_xyz_m=(-0.03,0,0)" in summary
    assert "error_xyz_m=(0,-0.04,0)" in summary
    assert "worst_motor=rf_ankle" in summary
    assert "motors_over_0.08rad=lm_knee=0.1,rf_ankle=0.12" in summary
    active_trace = mode.active_leg_diagnostic_summary()
    assert "diagnostic_stage=PAIR diagnostic_phase_time_s=0.2" in active_trace
    assert "stage_duration_s=3.57 diagnostic_stage_elapsed_s=0.7" in active_trace
    assert "active_legs=rb,rf" in active_trace
    assert "rb[actual_base_xyz_m=" in active_trace
    assert ";rf[actual_base_xyz_m=" in active_trace


def test_hardware_climb_timeout_includes_foot_and_motor_diagnostics(monkeypatch):
    import climb_mode

    monotonic_time = [10.0]
    monkeypatch.setattr(climb_mode.time, "monotonic", lambda: monotonic_time[0])
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode
    stage = mode.config["stages"][0]
    mode.stage_elapsed_time = (
        sum(stage["segment_durations_s"])
        + mode.config["settle_gate"]["timeout_s"]
    )
    monotonic_time[0] += 0.01
    controller.update(Q_STAND + 0.5, np.zeros(4))

    assert mode.state == ClimbMode.FAILED
    assert "worst_foot=" in mode.failure_reason
    assert "worst_motor=" in mode.failure_reason
    assert "feet_over_gate=" in mode.failure_reason
    assert "motors_over_0.08rad=" in mode.failure_reason


def test_run_real_logs_hardware_climb_phase_hold_diagnostics(monkeypatch):
    warnings = []
    monkeypatch.setattr(
        RUN_REAL.rospy,
        "logwarn_throttle",
        lambda *args: warnings.append(args),
    )
    node = object.__new__(RUN_REAL.RosControlNode)
    node.controller = types.SimpleNamespace(
        mode="climb",
        CLIMB="climb",
        climb_mode=types.SimpleNamespace(
            state=ClimbMode.RUNNING,
            hardware_execution=True,
            last_phase_hold=True,
            last_collision_guard_hold=True,
            phase="LM_GROUND_SHIFT1",
            tracking_diagnostic_summary=lambda: (
                "worst_foot=lm foot_target_error_m=0.0309 "
                "feet_over_gate=lm=0.0309 worst_motor=lm_knee "
                "motors_over_0.08rad=lm_knee=0.1"
            ),
        ),
    )

    RUN_REAL.RosControlNode._warn_hardware_climb_phase_hold(node)

    assert warnings == [(
        0.5,
        "CLIMB PHASE HOLD: source=%s stage=%s %s collision_guard_hold=%s",
        "hardware_feedback",
        "LM_GROUND_SHIFT1",
        "worst_foot=lm foot_target_error_m=0.0309 "
        "feet_over_gate=lm=0.0309 worst_motor=lm_knee "
        "motors_over_0.08rad=lm_knee=0.1",
        "true",
    )]


def test_run_real_logs_active_leg_trace_without_phase_hold(monkeypatch):
    infos = []
    monkeypatch.setattr(
        RUN_REAL.rospy,
        "loginfo_throttle",
        lambda *args: infos.append(args),
    )
    node = object.__new__(RUN_REAL.RosControlNode)
    node.local_execution = True
    node.controller = types.SimpleNamespace(
        mode="climb",
        CLIMB="climb",
        climb_mode=types.SimpleNamespace(
            state=ClimbMode.RUNNING,
            hardware_execution=True,
            phase="RB_RF_HIGH_C",
            last_diagnostic_stage_name="RB_RF_HIGH_C",
            active_leg_diagnostic_summary=lambda: (
                "base_link diagnostic_phase_time_s=0.2 stage_duration_s=1 "
                "diagnostic_stage_elapsed_s=0.2 active_legs=rb,rf "
                "active_leg_base_link_xyz=rb[actual_base_xyz_m=(0,0,0),"
                "desired_base_xyz_m=(0,0,0),error_xyz_m=(0,0,0)];"
                "rf[actual_base_xyz_m=(0,0,0),desired_base_xyz_m=(0,0,0),"
                "error_xyz_m=(0,0,0)]"
            ),
        ),
    )

    RUN_REAL.RosControlNode._info_hardware_climb_active_trace(node)

    assert infos[0][0] == 1.0
    assert infos[0][2] == "isaac_sim_feedback"
    assert infos[0][3] == "RB_RF_HIGH_C"
    assert "base_link diagnostic_phase_time_s=0.2" in infos[0][4]


def test_hardware_climb_collision_hold_freezes_trajectory_phase():
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode
    controller.last_update_collision_guard_hold_count = 1
    controller.update(Q_STAND, np.zeros(4))
    assert mode.phase_time == 0.0


def test_hardware_climb_endpoint_requires_configured_persistence():
    config = json.loads(
        (SCRIPTS.parent / "config" / "climb_compact.json").read_text()
    )
    ideal = GraspController(1.0 / 30.0)
    ideal.enter_climb(Q_STAND, config, end_stage_index=0)
    q_end = Q_STAND.copy()
    while ideal.climb_mode.state == ClimbMode.RUNNING:
        q_end = ideal.update(q_end, np.zeros(4))

    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(
        Q_STAND,
        config,
        end_stage_index=0,
        hardware_execution=True,
    )
    mode = controller.climb_mode
    duration = sum(config["stages"][0]["segment_durations_s"])
    mode.phase_time = duration
    mode.stage_elapsed_time = duration
    base, anchors, _ = mode._stage_reference()
    mode._apply_reference(base, anchors, sync_previous=True)
    controller.q_des = q_end.copy()
    required_frames = int(np.ceil(
        config["settle_gate"]["persistence_s"] / controller.dt
    ))

    for _ in range(required_frames - 1):
        controller.update(q_end, np.zeros(4))
        assert mode.state == ClimbMode.RUNNING
    controller.update(q_end, np.zeros(4))
    assert mode.state == ClimbMode.DONE


def test_hardware_climb_timeout_uses_wall_time_while_phase_is_frozen(monkeypatch):
    import climb_mode

    monotonic_time = [10.0]
    monkeypatch.setattr(climb_mode.time, "monotonic", lambda: monotonic_time[0])
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode
    stage = mode.config["stages"][0]
    duration = sum(stage["segment_durations_s"])
    mode.stage_elapsed_time = (
        duration + mode.config["settle_gate"]["timeout_s"]
    )
    monotonic_time[0] += 0.01
    controller.update(Q_STAND + 0.5, np.zeros(4))
    assert mode.state == ClimbMode.FAILED
    assert mode.phase_time == controller.dt


def test_hardware_climb_elapsed_time_uses_monotonic_running_time_only(monkeypatch):
    import climb_mode

    monotonic_time = [100.0]
    monkeypatch.setattr(climb_mode.time, "monotonic", lambda: monotonic_time[0])
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode

    monotonic_time[0] = 100.4
    mode.update(np.zeros(4), Q_STAND)
    assert np.isclose(mode.stage_elapsed_time, 0.4)

    mode.hold()
    monotonic_time[0] = 140.0
    mode.update(np.zeros(4), Q_STAND)
    assert np.isclose(mode.stage_elapsed_time, 0.4)

    mode.resume()
    monotonic_time[0] = 140.25
    mode.update(np.zeros(4), Q_STAND)
    assert np.isclose(mode.stage_elapsed_time, 0.65)


def test_simulated_feedback_gates_use_controller_time_not_wall_time(monkeypatch):
    import climb_mode

    monotonic_time = [100.0]
    monkeypatch.setattr(climb_mode.time, "monotonic", lambda: monotonic_time[0])
    controller = GraspController(
        1.0 / 30.0,
        climb_timeout_uses_wall_time=False,
    )
    controller.enter_climb(Q_STAND, hardware_execution=True)
    mode = controller.climb_mode

    monotonic_time[0] = 1000.0
    mode.update(np.zeros(4), Q_STAND)
    assert np.isclose(mode.stage_elapsed_time, controller.dt)

    mode.hold()
    monotonic_time[0] = 2000.0
    mode.update(np.zeros(4), Q_STAND)
    assert np.isclose(mode.stage_elapsed_time, controller.dt)

    mode.resume()
    monotonic_time[0] = 3000.0
    mode.update(np.zeros(4), Q_STAND)
    assert np.isclose(mode.stage_elapsed_time, 2.0 * controller.dt)


def test_isaac_compact_paths_enable_real_feedback_gates():
    source = (SCRIPTS / "run_sim.py").read_text()
    real_source = (SCRIPTS / "run_real.py").read_text()
    assert source.count("hardware_execution=True") >= 2
    assert "climb_hardware_execution=True" in source
    assert "climb_timeout_uses_wall_time=False" in source
    assert "climb_timeout_uses_wall_time=not self.local_execution" in real_source


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
