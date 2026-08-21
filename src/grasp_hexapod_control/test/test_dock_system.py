#!/usr/bin/env python3
"""CPU contract checks for the bottom-USB DOCK perception chain."""

import importlib
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np
import yaml


PACKAGE = Path(__file__).parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))


def test_dock_system_yaml_is_the_complete_tag36h11_geometry_source():
    config = yaml.safe_load((PACKAGE / "config" / "dock_system.yaml").read_text())
    assert config["tag_family"] == "tag36h11"
    assert config["tag_ids"] == [0, 1, 2, 3]
    assert config["tag_size_m"] == 0.040
    assert {item["id"] for item in config["standalone_tags"]} == {0, 1, 2, 3}
    assert all(item["size"] == 0.040 for item in config["standalone_tags"])
    assert config["tag_bundles"] == []
    assert config["publish_tf"] is True
    assert config["real_calibrated"] is True
    assert set(config["pin_from_tag_m"]) == {"0", "1", "2", "3"}


def test_climb_config_saves_the_default_dock_entry_joint_pose():
    config = yaml.safe_load((PACKAGE / "config" / "climb_compact.json").read_text())
    terminal = np.asarray(config["terminal_q_rad"], dtype=float)
    initial = np.asarray(config["p0"]["q_rad"], dtype=float)
    assert terminal.shape == (6, 3)
    assert np.isfinite(terminal).all()
    assert not np.array_equal(terminal[2], initial[2])
    assert not np.array_equal(terminal[5], initial[5])


def test_real_launch_starts_only_the_bottom_usb_dock_chain_by_default():
    real = ET.parse(PACKAGE / "launch" / "run_real.launch")
    args = {item.attrib["name"]: item.attrib.get("default") for item in real.findall("arg")}
    assert args["start_dock_perception"] == "true"
    assert args["dock_detections_topic"] == "/dock/tag_detections"
    assert args["dock_image_topic"] == "/dock_camera/image_raw"
    assert args["dock_camera_info_topic"] == "/dock_camera/camera_info"
    assert args["dock_require_real_calibrated"] == "true"
    perception = real.find(
        "include[@file='$(find grasp_hexapod_control)/launch/dock_tag_system.launch']"
    )
    assert perception is not None
    assert perception.attrib["if"] == "$(arg start_dock_perception)"
    forwarded = {
        item.attrib["name"]: item.attrib["value"]
        for item in perception.findall("arg")
    }
    assert forwarded["detections_topic"] == "$(arg dock_detections_topic)"
    assert forwarded["camera_frame_id"] == "$(arg dock_camera_frame_id)"
    assert forwarded["dock_system_config"] == "$(arg dock_system_config)"

    dock = ET.parse(PACKAGE / "launch" / "dock_tag_system.launch")
    nodes = {(item.attrib["pkg"], item.attrib["type"], item.attrib.get("ns"))
             for item in dock.findall(".//node")}
    assert ("usb_cam", "usb_cam_node", "dock_camera") in nodes
    assert ("nodelet", "nodelet", None) in nodes
    assert ("apriltag_ros", "apriltag_ros_continuous_node", "dock") in nodes
    assert dock.find(".//include[@file='$(find image_proc)/launch/image_proc.launch']") is not None
    detector = dock.find("node[@name='apriltag']")
    assert detector.find("remap[@from='image_rect']").attrib["to"] == "/dock_camera/image_rect_color"


def test_simulation_imports_shared_dock_geometry_and_topics():
    source = (SCRIPTS / "run_sim_dock.py").read_text()
    assert "from dock_mode import (" in source
    assert "LOCK_FROM_CAMERA," in source
    assert "PIN_FROM_TAG," in source
    assert "TAG_SIZE," in source
    assert '"/dock_camera/image_raw"' in source
    assert '"/dock_camera/camera_info"' in source


def test_dock_mode_reuses_the_controller_kinematics_and_existing_topics():
    source = (SCRIPTS / "dock_mode.py").read_text()
    assert "self.controller.kinematic.forward_base" in source
    assert "self.controller._smooth_step" in source
    assert "solve_joints" not in source
    assert "JointReader" not in source
    assert "JointTrajectory" not in source
    assert "/dock_mode6" not in source
    assert 'detections_topic="/dock/tag_detections"' in source


def test_uncalibrated_dock_configuration_blocks_y_without_override():
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda name, default=None: default
    rospy.logwarn = rospy.logwarn_throttle = rospy.loginfo = lambda *args, **kwargs: None
    rospy.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 10.0))
    rospy.Subscriber = lambda *args, **kwargs: None
    previous_rospy = sys.modules.get("rospy")
    try:
        sys.modules["rospy"] = rospy
        run_real = importlib.import_module("run_real")
        node = object.__new__(run_real.RosControlNode)
        node.enable_real_dock = True
        node.dock_require_real_calibrated = True
        node.dock_allow_uncalibrated = False
        node.dock_system_config = "unused"
        node.HOLD = run_real.RosControlNode.HOLD
        node.RUNNING = run_real.RosControlNode.RUNNING
        node.state = node.HOLD
        node.command = __import__("numpy").zeros(4)
        node.controller = types.SimpleNamespace(
            climb_mode=types.SimpleNamespace(state="done"),
            enter_dock=lambda q: (_ for _ in ()).throw(AssertionError()),
        )
        node._ensure_dock_mode = lambda: (_ for _ in ()).throw(AssertionError())
        fake_dock = types.ModuleType("dock_mode")
        fake_dock.load_dock_system = lambda path: {"real_calibrated": False}
        previous = sys.modules.get("dock_mode")
        sys.modules["dock_mode"] = fake_dock
        run_real.RosControlNode._start_real_dock(node, __import__("numpy").zeros((6, 3)), True)
    finally:
        if previous_rospy is None:
            sys.modules.pop("rospy", None)
        else:
            sys.modules["rospy"] = previous_rospy
        if previous is None:
            sys.modules.pop("dock_mode", None)
        else:
            sys.modules["dock_mode"] = previous
    assert node.state == node.HOLD


def test_lock_confirmation_must_be_newer_than_this_y_session_and_fresh():
    run_real = importlib.import_module("run_real")
    node = object.__new__(run_real.RosControlNode)
    node.dock_session_started_at = 10.0
    node.dock_lock_confirmation_max_age = 0.5
    node.lock_confirmation = types.SimpleNamespace(
        snapshot_with_time=lambda: (True, 9.9)
    )
    original_time = run_real.rospy.Time
    run_real.rospy.Time = types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_sec=lambda: 10.2)
    )
    try:
        assert run_real.RosControlNode._dock_lock_confirmed(node) is None
        node.lock_confirmation = types.SimpleNamespace(
            snapshot_with_time=lambda: (True, 9.6)
        )
        assert run_real.RosControlNode._dock_lock_confirmed(node) is None
        node.lock_confirmation = types.SimpleNamespace(
            snapshot_with_time=lambda: (True, 10.3)
        )
        assert run_real.RosControlNode._dock_lock_confirmed(node) is None
        node.lock_confirmation = types.SimpleNamespace(
            snapshot_with_time=lambda: (True, 10.1)
        )
        assert run_real.RosControlNode._dock_lock_confirmed(node) is True
    finally:
        run_real.rospy.Time = original_time
