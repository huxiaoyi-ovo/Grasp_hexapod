#!/usr/bin/env python3
"""Isolated-master Phase-1 visual transport check; never starts hardware nodes.

The caller supplies ROS_MASTER_URI/ROS_LOG_DIR.  This starts only the compiled
bt_control_node and phase1_visual_sequence, while FakeHardware supplies joint
feedback and the gripper service.
"""
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import rospy
from grasp_hexapod_msgs.msg import ModeStatus
from grasp_hexapod_msgs.srv import SwitchMode
from std_msgs.msg import String
from xiaolan_yolo_detection.msg import PerceptionStatus, XiaolanTarget

sys.path.insert(0, str(Path(__file__).parent))
from fake_hardware_e2e import FakeHardware, call_mode, record_targets  # noqa: E402


def main():
    log_dir = Path(os.environ.get("PHASE1_E2E_LOG_DIR", "/tmp/grasp_phase1_visual_e2e"))
    log_dir.mkdir(parents=True, exist_ok=True)
    node = subprocess.Popen([
        "rosrun", "grasp_hexapod_bt_control", "bt_control_node",
        "_visual_guidance_enabled:=true", "_enable_real_climb:=false",
        "_enable_real_dock:=false"], stdout=(log_dir / "node.log").open("w"),
        stderr=subprocess.STDOUT, env=os.environ.copy())
    try:
        fake = FakeHardware()
        statuses = []
        rospy.Subscriber("/grasp_hexapod/mode_status", ModeStatus,
                         lambda m: statuses.append((m.mode, m.status, m.message)),
                         queue_size=20)
        target = rospy.Publisher("/xiaolan/target", XiaolanTarget, queue_size=1)
        health_pub = rospy.Publisher("/xiaolan/perception_status",
                                     PerceptionStatus, queue_size=1)
        deadline = time.time() + 15
        while target.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(.05)
        assert target.get_num_connections(), "bt_control_node did not subscribe /xiaolan/target"
        deadline = time.time() + 10
        while not any(m == "" and s == "idle" and
                      message == "phase1 joint feedback ready"
                      for m, s, message in statuses) and time.time() < deadline:
            rospy.sleep(.05)
        assert any(m == "" and s == "idle" and
                   message == "phase1 joint feedback ready"
                   for m, s, message in statuses), statuses

        phase = {"value": "absent", "held_stamp": None}
        running = True

        def publish_health(state, stamp):
            message = PerceptionStatus(state=state, reason=(
                "NO_DETECTION" if state == PerceptionStatus.NO_TARGET
                else "TARGET_VALID"))
            message.header.stamp = stamp
            message.rgb_stamp = stamp
            message.depth_stamp = stamp
            message.header.frame_id = "camera_color_optical_frame"
            health_pub.publish(message)

        def publish_loop():
            rate = rospy.Rate(15)
            while running and not rospy.is_shutdown():
                now = rospy.Time.now()
                if phase["value"] == "absent":
                    publish_health(PerceptionStatus.NO_TARGET, now)
                else:
                    msg = XiaolanTarget(detected=True, x_target=0.,
                                        z_target=(.9 if phase["value"] == "far"
                                                  else float(.60)))
                    if phase["value"] == "held_near":
                        msg.header.stamp = phase["held_stamp"]
                    else:
                        msg.header.stamp = now
                    msg.header.frame_id = "camera_color_optical_frame"
                    target.publish(msg)
                    # Keep current health source stamps separate from a held target.
                    publish_health(PerceptionStatus.TARGET_VALID, now)
                rate.sleep()
        publisher = threading.Thread(target=publish_loop, daemon=True)
        publisher.start()
        seq = subprocess.Popen(["rosrun", "grasp_hexapod_bt_control", "phase1_visual_sequence"],
                               stdout=(log_dir / "sequence.log").open("w"),
                               stderr=subprocess.STDOUT, env=os.environ.copy())
        deadline = time.time() + 15
        while not any(m == "spin_search" and s == "running" for m, s, _ in statuses) and time.time() < deadline:
            rospy.sleep(.05)
        assert any(m == "spin_search" and s == "running" for m, s, _ in statuses), statuses
        with record_targets(fake):
            time.sleep(.8)
        span = float(__import__("numpy").ptp(__import__("numpy").array(fake.walk_targets).reshape(len(fake.walk_targets), -1), axis=0).max())
        assert span > 1e-5, f"search targets did not vary: {span}"
        phase["value"] = "far"
        deadline = time.time() + 15
        while not any(m == "approach" and s == "running" for m, s, _ in statuses) and time.time() < deadline:
            rospy.sleep(.05)
        assert any(m == "approach" and s == "running" for m, s, _ in statuses), statuses
        with record_targets(fake):
            time.sleep(.8)
        approach_span = float(__import__("numpy").ptp(__import__("numpy").array(fake.walk_targets).reshape(len(fake.walk_targets), -1), axis=0).max())
        assert approach_span > 1e-5, f"approach targets did not vary: {approach_span}"
        phase["value"] = "near"
        assert seq.wait(timeout=20) == 0, "phase1 sequence failed"
        assert any(m == "spin_search" and s == "success" for m, s, _ in statuses), statuses
        assert any(m == "approach" and s == "success" and "coarse approach completed" in message
                   for m, s, message in statuses), statuses
        assert not any(m in ("climb", "dock") and s == "running" for m, s, _ in statuses), statuses
        phase["value"] = "absent"
        loss_ok, loss_message = call_mode(
            rospy.ServiceProxy("/grasp_hexapod/switch_mode", SwitchMode),
            "approach", timeout_s=5)
        assert loss_ok is False and "visual target lost" in loss_message, (loss_ok, loss_message)

        switch = rospy.ServiceProxy("/grasp_hexapod/switch_mode", SwitchMode)
        hold_pub = rospy.Publisher("/grasp_hexapod/hold_motion", String, queue_size=5)

        # A pause cannot reuse a pre-pause near sample.  Keep publishing one
        # old source stamp after the lease expires, then supply fresh frames.
        phase["value"] = "far"
        pause_result = {}
        pause_thread = threading.Thread(
            target=lambda: pause_result.update(zip(("success", "message"),
                                                   call_mode(switch, "approach", 8))),
            daemon=True)
        pause_thread.start()
        rospy.sleep(.4)
        phase["held_stamp"] = rospy.Time.now()
        phase["value"] = "held_near"
        for _ in range(12):
            hold_pub.publish(String(data="phase1 pause"))
            rospy.sleep(.03)
        rospy.sleep(.6)
        assert pause_thread.is_alive(), "old held sample completed after resume"
        phase["value"] = "near"
        pause_thread.join(timeout=5)
        assert not pause_thread.is_alive(), "fresh post-resume samples did not finish approach"
        assert pause_result["success"] is True, pause_result

        # Feedback loss triggers the visual onNodeHold failure rather than a
        # permanently running service.  No targets are emitted while feedback
        # is invalid; a subsequent explicit request works after restoration.
        phase["value"] = "far"
        feedback_result = {}
        with record_targets(fake):
            feedback_thread = threading.Thread(
                target=lambda: feedback_result.update(
                    zip(("success", "message"),
                        call_mode(switch, "approach", 5))),
                daemon=True)
            feedback_thread.start()
            rospy.sleep(.5)
            feedback_loss_started = time.monotonic()
            with fake.lock:
                fake.feedback_enabled = False
            # Ignore the already-in-flight feedback frame, then require the
            # target stream to stop over a second complete watchdog window.
            rospy.sleep(.4)
            with fake.lock:
                target_count_before_loss = len(fake.walk_targets)
            rospy.sleep(.4)
            with fake.lock:
                target_count_after_loss = len(fake.walk_targets)
        feedback_thread.join(timeout=3)
        assert not feedback_thread.is_alive(), "feedback-loss approach did not return"
        feedback_loss_elapsed = time.monotonic() - feedback_loss_started
        assert feedback_result["success"] is False, feedback_result
        assert "joint feedback lost" in feedback_result["message"], feedback_result
        assert feedback_loss_elapsed < 3.0, feedback_loss_elapsed
        assert target_count_after_loss == target_count_before_loss, (
            target_count_before_loss, target_count_after_loss)
        with fake.lock:
            fake.feedback_enabled = True
        rospy.sleep(.4)
        phase["value"] = "near"
        restored_ok, restored_message = call_mode(switch, "approach", timeout_s=5)
        assert restored_ok is True, restored_message
        print(
            f"search_span={span:.8f} approach_span={approach_span:.8f} "
            f"loss={loss_message} feedback={feedback_result['message']} "
            f"feedback_s={feedback_loss_elapsed:.3f} "
            f"restored={restored_message}")
        return 0
    finally:
        running = False if "running" in locals() else False
        if "publisher" in locals():
            publisher.join(timeout=2)
        if "seq" in locals() and seq.poll() is None:
            seq.terminate()
            try:
                seq.wait(timeout=5)
            except subprocess.TimeoutExpired:
                seq.kill()
                seq.wait(timeout=5)
        node.terminate()
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
