#!/usr/bin/env python3
"""Isolated RGB-D -> Target3DNode -> visual-control Phase-1 bench check.

The caller owns an isolated ROS master.  This script launches only the bench
(bt_control_node, fake feedback, and Target3DNode), then publishes synthetic
detection/depth inputs.  It never publishes /xiaolan/target directly.
"""
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rosgraph
import rospy
import yaml
from cv_bridge import CvBridge
from grasp_hexapod_msgs.msg import ModeStatus
from grasp_hexapod_msgs.srv import SwitchMode
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from xiaolan_yolo_detection.msg import PerceptionStatus, XiaolanDetection, XiaolanTarget

sys.path.insert(0, str(Path(__file__).parent))
from fake_hardware_e2e import call_mode  # noqa: E402


LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")


class TargetRecorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.targets = []
        self.health = []
        self.joint_targets = {leg: [] for leg in LEG_NAMES}
        rospy.Subscriber("/xiaolan/target", XiaolanTarget, self.target_cb,
                         queue_size=20)
        rospy.Subscriber("/xiaolan/perception_status", PerceptionStatus,
                         self.health_cb, queue_size=20)
        for leg in LEG_NAMES:
            rospy.Subscriber(f"/{leg}_des", Float64MultiArray,
                             lambda message, name=leg: self.joint_cb(name, message),
                             queue_size=20)

    def target_cb(self, message):
        with self.lock:
            self.targets.append((message.header.stamp.to_sec(), message.detected,
                                 message.x_target, message.z_target))

    def health_cb(self, message):
        received = rospy.Time.now().to_sec()
        rgb_stamp = message.rgb_stamp.to_sec()
        depth_stamp = message.depth_stamp.to_sec()
        with self.lock:
            self.health.append({
                "state": message.state,
                "reason": message.reason,
                "received_s": received,
                "header_stamp": message.header.stamp.to_sec(),
                "rgb_stamp": rgb_stamp,
                "depth_stamp": depth_stamp,
                "detection_hz": message.detection_hz,
                "depth_hz": message.depth_hz,
                "target_hz": message.target_hz,
                "rgb_age_s": received - rgb_stamp if rgb_stamp > 0. else None,
                "depth_age_s": received - depth_stamp if depth_stamp > 0. else None,
            })

    def joint_cb(self, leg, message):
        if len(message.data) == 10:
            with self.lock:
                self.joint_targets[leg].append(tuple(message.data[1:4]))

    def mark(self):
        with self.lock:
            return {leg: len(values) for leg, values in self.joint_targets.items()}

    def spans_since(self, mark):
        with self.lock:
            samples = {
                leg: np.asarray(values[mark[leg]:], dtype=float)
                for leg, values in self.joint_targets.items()
            }
        return {
            leg: (float(np.ptp(values, axis=0).max()) if values.size else 0.0)
            for leg, values in samples.items()
        }


def wait_until(predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        rospy.sleep(0.05)
    raise AssertionError(f"timeout waiting for {description}")


def main():
    log_dir = Path(os.environ.get("PHASE1_PERCEPTION_E2E_LOG_DIR",
                                  "/tmp/grasp_phase1_perception_e2e"))
    log_dir.mkdir(parents=True, exist_ok=True)
    # This bench covers the coarse-only path: disable the tag handoff and keep
    # the 0.60 m stop that the depth sweep below asserts.
    config = yaml.safe_load((Path(__file__).resolve().parents[1] / "config"
                             / "phase1_visual.yaml").read_text())
    config.update(tag_guidance_enabled=False, visual_stop_distance_m=0.60)
    config_path = log_dir / "phase1_visual_coarse_only.yaml"
    config_path.write_text(yaml.safe_dump(config))
    bench = subprocess.Popen(
        ["roslaunch", "grasp_hexapod_bt_control", "phase1_visual_bench.launch",
         "start_depth:=true", f"config:={config_path}"],
        stdout=(log_dir / "bench.log").open("w"), stderr=subprocess.STDOUT,
        env=os.environ.copy())
    try:
        rospy.init_node("phase1_perception_e2e")
        recorder = TargetRecorder()
        detection_pub = rospy.Publisher("/xiaolan/detection", XiaolanDetection,
                                        queue_size=1)
        depth_pub = rospy.Publisher("/camera/depth/image_raw", Image,
                                    queue_size=1)
        aligned_info_pub = rospy.Publisher("/camera/depth/camera_info",
                                           CameraInfo, queue_size=1, latch=True)
        color_info_pub = rospy.Publisher("/camera/color/camera_info",
                                         CameraInfo, queue_size=1, latch=True)
        switch = rospy.ServiceProxy("/grasp_hexapod/switch_mode", SwitchMode)
        statuses = []
        rospy.Subscriber("/grasp_hexapod/mode_status", ModeStatus,
                         lambda message: statuses.append(
                             (message.mode, message.status, message.message)),
                         queue_size=30)
        switch.wait_for_service(timeout=20)
        wait_until(
            lambda: any(mode == "" and status == "idle" and
                        message == "phase1 joint feedback ready"
                        for mode, status, message in statuses),
            10, "controller-owned phase1 feedback ready status")
        wait_until(lambda: detection_pub.get_num_connections() > 0, 15,
                   "Target3DNode detection subscription")
        master = rosgraph.Master(rospy.get_name())
        publishers = dict(master.getSystemState()[0])
        assert publishers.get("/xiaolan/target") == ["/xiaolan_target_3d_node"], publishers.get("/xiaolan/target")
        assert publishers.get("/xiaolan/perception_status") == ["/xiaolan_target_3d_node"], publishers.get("/xiaolan/perception_status")

        phase = {"value": "absent", "depth": 0.9}
        bridge = CvBridge()

        def publish_sample(depth_m, mismatch=False, zero_depth=False,
                           detected=True):
            stamp = rospy.Time.now()
            frame = "camera_color_optical_frame"
            info = CameraInfo(width=32, height=32)
            info.header.stamp = stamp
            info.header.frame_id = frame
            info.K = [100.0, 0.0, 16.0, 0.0, 100.0, 16.0, 0.0, 0.0, 1.0]
            aligned_info_pub.publish(info)
            color_info_pub.publish(info)
            image = bridge.cv2_to_imgmsg(
                np.full((32, 32), 0.0 if zero_depth else depth_m,
                        dtype=np.float32), "32FC1")
            image.header.stamp = stamp
            image.header.frame_id = "wrong_depth_frame" if mismatch else frame
            depth_pub.publish(image)
            detection = XiaolanDetection(
                detected=detected, confidence=.95, bbox_x=11, bbox_y=11,
                bbox_width=10, bbox_height=10, center_u=16, center_v=16,
                image_width=32, image_height=32,
                reason="DETECTED" if detected else "NO_DETECTION")
            detection.header.stamp = stamp
            detection.header.frame_id = frame
            detection_pub.publish(detection)

        running = True

        def publisher_loop():
            rate = rospy.Rate(15)
            while running and not rospy.is_shutdown():
                current = phase["value"]
                if current == "absent":
                    publish_sample(phase["depth"], detected=False)
                elif current == "aligned":
                    publish_sample(phase["depth"])
                elif current == "zero":
                    publish_sample(phase["depth"], zero_depth=True)
                elif current == "mismatch":
                    publish_sample(phase["depth"], mismatch=True)
                elif current == "missing":
                    # Detection deliberately has no matching depth frame.
                    stamp = rospy.Time.now()
                    detection = XiaolanDetection(
                        detected=True, confidence=.95, bbox_x=11, bbox_y=11,
                        bbox_width=10, bbox_height=10, center_u=16, center_v=16,
                        image_width=32, image_height=32, reason="DETECTED")
                    detection.header.stamp = stamp
                    detection.header.frame_id = "camera_color_optical_frame"
                    detection_pub.publish(detection)
                rate.sleep()

        thread = threading.Thread(target=publisher_loop, daemon=True)
        thread.start()
        wait_until(
            lambda: any(sample["state"] == PerceptionStatus.NO_TARGET
                        for sample in recorder.health),
            10, "Target3DNode fresh healthy NO_TARGET status")
        sequence = subprocess.Popen(
            ["rosrun", "grasp_hexapod_bt_control", "phase1_visual_sequence",
             "_controller_node:=/bt_control_node"],
            stdout=(log_dir / "sequence.log").open("w"),
            stderr=subprocess.STDOUT, env=os.environ.copy())
        wait_until(lambda: any(mode == "spin_search" and status == "running"
                               for mode, status, _ in statuses), 20,
                   "spin_search running")
        search_start = recorder.mark()
        rospy.sleep(0.8)
        search_spans = recorder.spans_since(search_start)
        assert max(search_spans.values()) > 1e-5, search_spans

        phase["value"] = "aligned"
        wait_until(lambda: any(mode == "approach" and status == "running"
                               for mode, status, _ in statuses), 20,
                   "approach running")
        approach_start = recorder.mark()
        steady_window_start = rospy.Time.now().to_sec()
        rospy.sleep(2.5)
        with recorder.lock:
            steady_health_candidates = [
                sample for sample in recorder.health
                if sample["state"] == PerceptionStatus.TARGET_VALID and
                sample["received_s"] >= steady_window_start + 2.0]
        assert steady_health_candidates, (
            "no TARGET_VALID health sample after stable far-input window")
        steady_health = max(steady_health_candidates,
                            key=lambda sample: sample["received_s"])
        steady_rates = {
            "detection_hz": steady_health["detection_hz"],
            "depth_hz": steady_health["depth_hz"],
            "target_hz": steady_health["target_hz"],
            "rgb_age_s": steady_health["rgb_age_s"],
            "depth_age_s": steady_health["depth_age_s"],
            "state": steady_health["state"],
        }
        assert all(10.0 <= steady_rates[key] <= 20.0
                   for key in ("detection_hz", "depth_hz", "target_hz")), \
            steady_rates
        assert all(age is not None and -.05 <= age <= .25
                   for age in (steady_rates["rgb_age_s"],
                               steady_rates["depth_age_s"])), steady_rates
        for depth_m in (0.9, 0.8, 0.7, 0.6, float(.60)):
            phase["depth"] = depth_m
            rospy.sleep(0.35)
        assert sequence.wait(timeout=20) == 0, "visual sequence failed"
        approach_spans = recorder.spans_since(approach_start)
        assert max(approach_spans.values()) > 1e-5, approach_spans
        assert any(mode == "approach" and status == "success" and
                   "coarse approach completed" in message
                   for mode, status, message in statuses), statuses
        assert not any(mode in ("climb", "dock") and status == "running"
                       for mode, status, _ in statuses), statuses
        with recorder.lock:
            valid_targets = [sample for sample in recorder.targets if sample[1]]
        assert valid_targets, "Target3DNode never published a valid target"
        target_stamp, _, target_x, target_z = valid_targets[-1]
        target_age = rospy.Time.now().to_sec() - target_stamp
        assert 0.0 <= target_age <= 0.25, target_age
        assert abs(target_x) < 0.03 and 0.45 <= target_z <= 0.62, (
            target_x, target_z)

        def assert_perception_failure(ok, message, phase_name):
            assert ok is False, (phase_name, ok, message)
            assert ("perception startup unavailable" in message or
                    "perception stale/unusable" in message), (phase_name, message)
            assert "visual target lost" not in message, (phase_name, message)

        phase["value"] = "zero"
        zero_ok, zero_message = call_mode(switch, "approach", timeout_s=5)
        assert_perception_failure(zero_ok, zero_message, "zero-depth")
        phase["value"] = "missing"
        missing_ok, missing_message = call_mode(switch, "approach", timeout_s=5)
        assert_perception_failure(missing_ok, missing_message, "missing-depth")
        phase["value"] = "mismatch"
        mismatch_ok, mismatch_message = call_mode(switch, "approach", timeout_s=5)
        assert_perception_failure(mismatch_ok, mismatch_message, "frame-mismatch")

        phase["value"] = "aligned"
        phase["depth"] = float(.60)
        retry_ok, retry_message = call_mode(switch, "approach", timeout_s=5)
        assert retry_ok is True and "coarse approach completed" in retry_message, retry_message
        print(
            f"search_spans={search_spans} approach_spans={approach_spans} "
            f"target_age_s={target_age:.3f} x={target_x:.4f} z={target_z:.4f} "
            f"zero={zero_message} missing={missing_message} "
            f"mismatch={mismatch_message} retry={retry_message}")
        with recorder.lock:
            health_valid = [sample for sample in recorder.health
                            if sample["state"] == PerceptionStatus.TARGET_VALID]
            health_latest = recorder.health[-1] if recorder.health else None
        assert health_valid, "Target3DNode never published healthy TARGET_VALID status"
        assert health_latest is not None, "Target3DNode emitted no perception status"
        print(f"health_steady_far={steady_rates}")
        print(
            "health_postfault_recovery=" + str({key: health_latest[key] for key in
                                                ("state", "reason", "detection_hz",
                                                 "depth_hz", "target_hz", "rgb_age_s",
                                                 "depth_age_s")}) +
            f" healthy_valid_count={len(health_valid)}")
        return 0
    finally:
        running = False if "running" in locals() else False
        if "thread" in locals():
            thread.join(timeout=2)
        if "sequence" in locals() and sequence.poll() is None:
            sequence.terminate()
            try:
                sequence.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sequence.kill()
                sequence.wait(timeout=5)
        bench.terminate()
        try:
            bench.wait(timeout=10)
        except subprocess.TimeoutExpired:
            bench.kill()
            bench.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
