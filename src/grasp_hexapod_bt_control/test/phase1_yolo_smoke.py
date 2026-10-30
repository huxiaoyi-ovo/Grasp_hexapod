#!/usr/bin/env python3
"""Isolated CPU smoke test for the real ROS YoloNode and best.onnx.

The caller supplies an isolated ROS master.  This script launches only the
existing detector (ONNX CPU backend), publishes synthetic blank RGB frames, and
verifies the published XiaolanDetection schema/timestamps.  It is not an
accuracy or camera test.
"""
import argparse
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from xiaolan_yolo_detection.msg import XiaolanDetection


def wait_until(predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        rospy.sleep(.02)
    raise AssertionError(f"timeout waiting for {description}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--python", required=True,
                        help="isolated interpreter with onnxruntime")
    parser.add_argument("--model", required=True)
    parser.add_argument("--log-dir", default="/tmp/grasp_phase1_yolo_smoke")
    parser.add_argument("--fresh-samples", type=int, default=15)
    args = parser.parse_args()
    if args.fresh_samples < 1:
        raise ValueError("--fresh-samples must be positive")

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    launch = subprocess.Popen(
        ["roslaunch", "xiaolan_yolo_detection", "xiaolan_yolo.launch",
         "start_yolo:=true", "start_3d:=false", "start_visualizer:=false",
         "backend:=onnx", f"yolo_python:={os.path.abspath(args.python)}",
         f"model_path:={Path(args.model).resolve()}"],
        stdout=(log_dir / "yolo.launch.log").open("w"),
        stderr=subprocess.STDOUT, env=os.environ.copy())
    try:
        rospy.init_node("phase1_yolo_smoke")
        received = []
        rospy.Subscriber("/xiaolan/detection", XiaolanDetection,
                         lambda message: received.append((time.monotonic(), message)),
                         queue_size=50)
        publisher = rospy.Publisher("/camera/color/image_raw", Image,
                                    queue_size=1)
        wait_until(lambda: publisher.get_num_connections() > 0, 20,
                   "YoloNode RGB subscription")
        bridge = CvBridge()
        source_stamps = set()
        rate = rospy.Rate(20)
        blank = np.zeros((64, 64, 3), dtype=np.uint8)
        deadline = time.monotonic() + 30
        while len(received) < args.fresh_samples and time.monotonic() < deadline:
            message = bridge.cv2_to_imgmsg(blank, "bgr8")
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "camera_color_optical_frame"
            source_stamps.add(message.header.stamp.to_nsec())
            publisher.publish(message)
            rate.sleep()
        fresh = [item for item in received
                 if item[1].header.stamp.to_nsec() in source_stamps]
        assert len(fresh) >= args.fresh_samples, len(fresh)
        assert all(message.header.frame_id == "camera_color_optical_frame"
                   and message.image_width == 64 and message.image_height == 64
                   and not message.detected and message.reason == "NO_DETECTION"
                   and math.isfinite(message.inference_ms) and message.inference_ms > 0.0
                   for _, message in fresh[:args.fresh_samples])

        stale_count = len(received)
        stale = bridge.cv2_to_imgmsg(blank, "bgr8")
        stale.header.stamp = rospy.Time.from_sec(rospy.Time.now().to_sec() - 1.0)
        stale.header.frame_id = "camera_color_optical_frame"
        stale_stamp = stale.header.stamp.to_nsec()
        publisher.publish(stale)
        wait_until(lambda: any(message.header.stamp.to_nsec() == stale_stamp
                               for _, message in received[stale_count:]), 5,
                   "stale-frame detector output")
        stale_output = next(message for _, message in received[stale_count:]
                            if message.header.stamp.to_nsec() == stale_stamp)
        assert stale_output.reason.startswith("STALE_OR_INVALID_RGB_TIMESTAMP"), stale_output.reason
        assert stale_output.inference_ms == 0.0, stale_output.inference_ms

        observed_rate = ((len(fresh) - 1) /
                         (fresh[-1][0] - fresh[0][0]) if len(fresh) > 1 else 0.0)
        inference_ms = [message.inference_ms for _, message in fresh[:args.fresh_samples]]
        print(
            f"fresh={len(fresh)} rate_hz={observed_rate:.3f} "
            f"inference_ms_min={min(inference_ms):.3f} "
            f"inference_ms_max={max(inference_ms):.3f} "
            f"provider=CPUExecutionProvider stale_reason={stale_output.reason}")
        return 0
    finally:
        launch.terminate()
        try:
            launch.wait(timeout=10)
        except subprocess.TimeoutExpired:
            launch.kill()


if __name__ == "__main__":
    raise SystemExit(main())
