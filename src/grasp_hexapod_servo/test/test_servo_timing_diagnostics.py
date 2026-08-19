#!/usr/bin/env python3
"""Pure-CPU regression for per-board Servo timing diagnostics."""

import importlib.util
from pathlib import Path
import sys
import types


SCRIPTS = Path(__file__).parents[1] / "scripts"


def _load_servo_module():
    rospy = types.ModuleType("rospy")
    rospy.loginfo = lambda *args, **kwargs: None
    sys.modules["rospy"] = rospy

    sensor = types.ModuleType("sensor_msgs.msg")
    sensor.JointState = type("JointState", (), {})
    sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
    sys.modules["sensor_msgs.msg"] = sensor

    std = types.ModuleType("std_msgs.msg")
    std.Float64MultiArray = type("Float64MultiArray", (), {})
    std.Header = type("Header", (), {})
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std

    hiwonder = types.ModuleType("hiwonder_servo_controller")
    sys.modules["hiwonder_servo_controller"] = hiwonder

    spec = importlib.util.spec_from_file_location(
        "servo_for_timing_test", SCRIPTS / "servo.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SERVO = _load_servo_module()


def _node():
    node = object.__new__(SERVO.ServoSideNode)
    node.side = "left"
    node.servo_ids = (1, 2, 3)
    node.servo_rate_hz = 30.0
    node._timing_window_started = 0.0
    node._timing_callbacks = 0
    node._timing_max_loop_s = 0.0
    node._timing_overruns = 0
    node._timing_read_retries = {servo_id: 0 for servo_id in node.servo_ids}
    node._timing_read_failures = {servo_id: 0 for servo_id in node.servo_ids}
    return node


def test_read_retry_and_failure_are_counted_per_servo_id():
    node = _node()
    calls = []
    node.control = types.SimpleNamespace(
        get_servo_position=lambda servo_id: calls.append(servo_id) or None
    )

    assert node._read_position(2) is None
    assert calls == [2, 2]
    assert node._timing_read_retries == {1: 0, 2: 1, 3: 0}
    assert node._timing_read_failures == {1: 0, 2: 1, 3: 0}


def test_timing_diagnostic_reports_one_second_summary_and_resets(monkeypatch):
    node = _node()
    messages = []
    monkeypatch.setattr(SERVO.time, "monotonic", lambda: 1.0)
    monkeypatch.setattr(SERVO.rospy, "loginfo", lambda *args: messages.append(args))
    node._timing_read_retries[2] = 3
    node._timing_read_failures[3] = 1

    node._record_timing_diagnostics(0.95)

    assert len(messages) == 1
    assert "side=%s actual_hz=%.2f" in messages[0][0]
    assert messages[0][1] == "left"
    assert messages[0][-2:] == ("2=3", "3=1")
    assert node._timing_callbacks == 0
    assert node._timing_max_loop_s == 0.0
    assert node._timing_overruns == 0
    assert node._timing_read_retries == {1: 0, 2: 0, 3: 0}
    assert node._timing_read_failures == {1: 0, 2: 0, 3: 0}
