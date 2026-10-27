"""Source association must tolerate independent ROS publisher sequences."""
import runpy
import threading
import unittest
from collections import OrderedDict
from pathlib import Path
from unittest.mock import patch

import numpy as np
import rospy
from cv_bridge import CvBridge
from xiaolan_yolo_detection.msg import XiaolanDetection, XiaolanTarget

MODULE = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                           'scripts/xiaolan_visualizer.py'))
Visualizer = MODULE['XiaolanVisualizer']


class VisualizerTest(unittest.TestCase):
    def test_independent_sequences_preserve_target_overlay(self):
        node = Visualizer.__new__(Visualizer)
        node.lock, node.bridge = threading.Lock(), CvBridge()
        node.colors, node.detections = OrderedDict(), OrderedDict()
        node.latest_rgb = node.latest_detection = node.latest_target = None
        node.info, node.detail = None, ''
        node.timeout, node.slop, node.live_dt = .5, .05, .25
        node.tolerance, node.last_display = .03, -1
        rgb = node.bridge.cv2_to_imgmsg(np.zeros((480, 848, 3), np.uint8), 'bgr8')
        det = XiaolanDetection(detected=True, image_width=848, image_height=480)
        target = XiaolanTarget(detected=True, status='CENTER', x_target=-.005, z_target=.869)
        for msg, seq in [(rgb, 3000), (det, 1954), (target, 1955)]:
            msg.header.stamp = rospy.Time(100, 123)
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.header.seq = seq
        node.rgb_cb(rgb)
        node.detection_cb(det)
        node.target_cb(target)
        with patch.dict(Visualizer.render.__globals__, render_rgb=lambda *args: args):
            rendered, _ = node.render()
            self.assertIs(rendered[2], target)
            # A different source timestamp or frame must still be rejected.
            target.header.stamp = rospy.Time(100, 124)
            rendered, _ = node.render()
            self.assertIsNone(rendered[2])
            target.header.stamp = det.header.stamp
            target.header.frame_id = 'other_camera'
            rendered, _ = node.render()
            self.assertIsNone(rendered[2])
