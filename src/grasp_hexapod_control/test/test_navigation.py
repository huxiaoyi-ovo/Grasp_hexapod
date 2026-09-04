#!/usr/bin/env python3
"""双RTK/IMU局部定位的纯CPU单元测试。"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from utils.navigation import (
    NavigationCalibration,
    RelativePoseEstimator,
    quaternion_from_rotation,
    rotation_from_quaternion,
    rotation_from_rpy,
)


def _calibration_mapping():
    return {
        "installation_calibrated": True,
        "panel_origin_geodetic_deg_m": [30.0, 120.0, 10.0],
        "panel_yaw_from_east_deg": 0.0,
        "enu_from_imu_reference_rpy_deg": [0.0, 0.0, 0.0],
        "imu_from_base_rpy_deg": [0.0, 0.0, 0.0],
        "robot_rtk_antenna_in_base_m": [0.0, 0.0, 0.0],
        "panel_from_xiaolan_rpy_deg": [0.0, 0.0, 0.0],
        "xiaolan_rtk_antenna_in_xiaolan_m": [0.0, 0.0, 0.0],
        "pv_boundary_xy_m": [
            [-1.0, -1.0],
            [1.0, -1.0],
            [1.0, 1.0],
            [-1.0, 1.0],
        ],
    }


def test_placeholder_navigation_config_cannot_be_used_as_real_calibration():
    path = SCRIPTS.parent / "config" / "navigation_rtk_imu.yaml"
    with path.open() as file:
        mapping = yaml.safe_load(file)
    assert mapping["installation_calibrated"] is False
    with pytest.raises(ValueError, match="installation_calibrated"):
        NavigationCalibration.from_mapping(mapping)


def test_relative_estimator_uses_east_north_axes_and_imu_yaw():
    calibration = NavigationCalibration.from_mapping(_calibration_mapping())
    estimator = RelativePoseEstimator(calibration)
    origin = calibration.panel_origin_geodetic_deg_m
    yaw = np.deg2rad(25.0)
    quaternion = quaternion_from_rotation(rotation_from_rpy(0.0, 0.0, yaw))
    pv_from_base, pv_from_xiaolan = estimator.estimate(
        [origin[0], origin[1] + 1e-5, origin[2]],
        origin,
        quaternion,
    )
    assert pv_from_base[0, 3] > 0.9
    assert abs(pv_from_base[1, 3]) < 0.01
    assert np.allclose(pv_from_base[:3, :3], rotation_from_rpy(0.0, 0.0, yaw))
    assert np.allclose(pv_from_xiaolan, np.eye(4), atol=1e-8)


def test_estimator_compensates_both_rtk_antenna_lever_arms():
    mapping = _calibration_mapping()
    mapping["robot_rtk_antenna_in_base_m"] = [0.4, 0.0, 0.0]
    mapping["xiaolan_rtk_antenna_in_xiaolan_m"] = [0.2, 0.0, 0.0]
    calibration = NavigationCalibration.from_mapping(mapping)
    estimator = RelativePoseEstimator(calibration)
    origin = calibration.panel_origin_geodetic_deg_m
    identity_quaternion = [0.0, 0.0, 0.0, 1.0]
    pv_from_base, pv_from_xiaolan = estimator.estimate(
        origin, origin, identity_quaternion
    )
    assert np.allclose(pv_from_base[:3, 3], [-0.4, 0.0, 0.0], atol=1e-8)
    assert np.allclose(pv_from_xiaolan[:3, 3], [-0.2, 0.0, 0.0], atol=1e-8)


def test_panel_yaw_maps_north_to_panel_x_and_aligns_robot_heading():
    mapping = _calibration_mapping()
    mapping["panel_yaw_from_east_deg"] = 90.0
    calibration = NavigationCalibration.from_mapping(mapping)
    estimator = RelativePoseEstimator(calibration)
    origin = calibration.panel_origin_geodetic_deg_m
    north_facing = quaternion_from_rotation(
        rotation_from_rpy(0.0, 0.0, np.deg2rad(90.0))
    )
    pv_from_base, _ = estimator.estimate(
        [origin[0] + 1e-5, origin[1], origin[2]],
        origin,
        north_facing,
    )
    assert pv_from_base[0, 3] > 1.0
    assert abs(pv_from_base[1, 3]) < 0.01
    assert np.allclose(pv_from_base[:3, :3], np.eye(3), atol=1e-12)


def test_quaternion_rotation_round_trip():
    expected = rotation_from_rpy(0.2, -0.3, 1.1)
    quaternion = quaternion_from_rotation(expected)
    actual = rotation_from_quaternion(quaternion)
    assert np.allclose(actual, expected, atol=1e-12)


def test_navigation_launch_is_wired_to_standard_sensor_topics():
    launch = ET.parse(
        SCRIPTS.parent / "launch" / "navigation_rtk_imu.launch"
    ).getroot()
    arguments = {
        item.attrib["name"]: item.attrib.get("default")
        for item in launch.findall("arg")
    }
    assert arguments["robot_fix_topic"] == "/grasp_hexapod/rtk/fix"
    assert arguments["xiaolan_fix_topic"] == "/grasp_hexapod/xiaolan/rtk/fix"
    assert arguments["imu_topic"] == "/grasp_hexapod/imu"
    node = launch.find("node")
    assert node is not None and node.attrib["type"] == "navigation_node.py"

    real_launch = ET.parse(
        SCRIPTS.parent / "launch" / "run_real.launch"
    ).getroot()
    real_arguments = {
        item.attrib["name"]: item.attrib.get("default")
        for item in real_launch.findall("arg")
    }
    assert real_arguments["start_rtk_imu_navigation"] == "false"
    assert "navigation_route_config" not in real_arguments
    assert real_arguments["navigation_boundary_margin_m"] == "0.03"
    assert real_arguments["navigation_xiaolan_body_clearance_m"] == "0.13"
    includes = [
        item for item in real_launch.findall("include")
        if item.attrib.get("file", "").endswith("navigation_rtk_imu.launch")
    ]
    assert len(includes) == 1
    assert includes[0].attrib.get("if") == "$(arg start_rtk_imu_navigation)"
