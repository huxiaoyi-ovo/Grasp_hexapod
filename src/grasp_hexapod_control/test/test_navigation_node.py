#!/usr/bin/env python3
"""RTK/IMU ROS适配节点的数据准入门测试。"""

import importlib.util
from pathlib import Path
import sys
import types

import numpy as np


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
MODULES = (
    "rospy",
    "geometry_msgs",
    "geometry_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
)
MISSING = object()
previous = {name: sys.modules.get(name, MISSING) for name in MODULES}

rospy = types.ModuleType("rospy")
geometry = types.ModuleType("geometry_msgs.msg")
geometry.Point32 = type("Point32", (), {})
geometry.PolygonStamped = type("PolygonStamped", (), {})
geometry.PoseStamped = type("PoseStamped", (), {})
sensor = types.ModuleType("sensor_msgs.msg")
sensor.Imu = type("Imu", (), {})
sensor.NavSatFix = type(
    "NavSatFix",
    (),
    {"COVARIANCE_TYPE_UNKNOWN": 0},
)
sensor.NavSatStatus = type(
    "NavSatStatus",
    (),
    {"STATUS_NO_FIX": -1, "STATUS_FIX": 0, "STATUS_GBAS_FIX": 2},
)
try:
    sys.modules["rospy"] = rospy
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geometry
    sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
    sys.modules["sensor_msgs.msg"] = sensor
    spec = importlib.util.spec_from_file_location(
        "navigation_node_for_test", SCRIPTS / "navigation_node.py"
    )
    NAVIGATION_NODE = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(NAVIGATION_NODE)
finally:
    for name, module in previous.items():
        if module is MISSING:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


def _node():
    node = object.__new__(NAVIGATION_NODE.RtkImuNavigationNode)
    node.max_fix_age = 0.5
    node.max_imu_age = 0.2
    node.max_horizontal_std = 0.05
    node.max_imu_orientation_std = np.deg2rad(5.0)
    node.require_known_covariance = True
    node.require_gbas_fix = True
    node.require_known_imu_orientation_covariance = True
    return node


def _stamp(value=1.0):
    return types.SimpleNamespace(to_sec=lambda: value)


def test_fix_gate_requires_gbas_and_bounded_covariance():
    message = types.SimpleNamespace(
        status=types.SimpleNamespace(status=0),
        header=types.SimpleNamespace(stamp=_stamp()),
        latitude=30.0,
        longitude=120.0,
        altitude=10.0,
        position_covariance_type=1,
        position_covariance=[0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.001],
    )
    valid, reason = _node()._fix_valid(message, now=1.1)
    assert not valid and "GBAS" in reason
    message.status.status = 2
    assert _node()._fix_valid(message, now=1.1) == (True, "")
    message.position_covariance[0] = 0.01
    valid, reason = _node()._fix_valid(message, now=1.1)
    assert not valid and "uncertainty" in reason


def test_imu_gate_rejects_unknown_or_excessive_orientation_covariance():
    message = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=_stamp()),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    valid, reason = _node()._imu_valid(message, now=1.1)
    assert not valid and "unknown" in reason
    message.orientation_covariance = [
        np.deg2rad(8.0) ** 2, 0.0, 0.0,
        0.0, np.deg2rad(2.0) ** 2, 0.0,
        0.0, 0.0, np.deg2rad(2.0) ** 2,
    ]
    valid, reason = _node()._imu_valid(message, now=1.1)
    assert not valid and "uncertainty" in reason
    message.orientation_covariance[0] = np.deg2rad(2.0) ** 2
    assert _node()._imu_valid(message, now=1.1) == (True, "")
