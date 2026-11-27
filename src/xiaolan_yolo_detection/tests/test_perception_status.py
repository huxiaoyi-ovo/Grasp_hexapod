"""Health status is derived from current source inputs, not watchdog time."""
import runpy
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from xiaolan_yolo_detection.msg import PerceptionStatus, XiaolanDetection

MODULE = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                           'scripts/xiaolan_target_3d_node.py'))
Target3DNode = MODULE['Target3DNode']


class PerceptionStatusTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        for target, replacement in [('time.monotonic', lambda: self.now),
                                    ('rospy.Time.now', lambda: rospy.Time.from_sec(self.now)),
                                    ('rospy.get_param', lambda key, default=None: default),
                                    ('rospy.Publisher', Mock()), ('rospy.Subscriber', Mock())]:
            p = patch(target, replacement)
            p.start()
            self.addCleanup(p.stop)
        self.node = Target3DNode()
        self.node.publish = Mock()

    def inputs(self, detected=False, depth_stamp=None):
        det = XiaolanDetection(detected=detected, reason='DETECTED' if detected else 'NO_DETECTION', image_width=16, image_height=16,
                               bbox_x=2, bbox_y=2, bbox_width=5, bbox_height=5,
                               center_u=4, center_v=4, inference_ms=3.)
        det.header.stamp = rospy.Time.from_sec(self.now-.01)
        det.header.frame_id = 'camera_color_optical_frame'
        info = CameraInfo(width=16, height=16)
        info.header = det.header
        info.K = [100, 0, 4, 0, 100, 4, 0, 0, 1]
        depth = CvBridge().cv2_to_imgmsg(np.full((16, 16), 600, np.uint16), '16UC1')
        depth.header = det.header
        if depth_stamp is not None:
            depth.header.stamp = rospy.Time.from_sec(depth_stamp)
        return det, depth, info

    def test_no_detection_is_healthy_only_with_depth_and_source_stamps(self):
        det, depth, info = self.inputs()
        self.node.process(det, depth, info, info)
        self.assertEqual(self.node.health_state, PerceptionStatus.NO_TARGET)
        self.node.publish_health(self.now, True)
        status = self.node.health_pub.publish.call_args[0][0]
        self.assertEqual(status.rgb_stamp, det.header.stamp)
        self.assertEqual(status.depth_stamp, depth.header.stamp)
        self.assertEqual(status.header.stamp, det.header.stamp)

    def test_missing_depth_and_mismatch_are_faults(self):
        det, depth, info = self.inputs()
        self.node.process(det, None, info, info)
        self.assertNotEqual(self.node.health_state, PerceptionStatus.NO_TARGET)
        det, depth, info = self.inputs(depth_stamp=self.now-.2)
        self.node.process(det, depth, info, info)
        self.assertNotEqual(self.node.health_state, PerceptionStatus.NO_TARGET)

    def test_inference_fault_does_not_mask_and_next_frame_recovers(self):
        det, depth, info = self.inputs()
        det.reason = 'INFERENCE_FAILURE'
        self.node.process(det, depth, info, info)
        self.assertEqual(self.node.health_state, PerceptionStatus.FAULT)
        self.now += .1
        det, depth, info = self.inputs(detected=True)
        self.node.process(det, depth, info, info)
        self.assertIn(self.node.health_state, (PerceptionStatus.TARGET_PENDING, PerceptionStatus.TARGET_VALID))

    def test_watchdog_does_not_refresh_source_or_target_rate(self):
        det, depth, info = self.inputs(detected=True)
        self.node.process(det, depth, info, info)
        stamp = self.node.last_detection.header.stamp
        self.now += .1
        self.node.watchdog(self.now)
        self.node.publish_health(self.now, True)
        status = self.node.health_pub.publish.call_args[0][0]
        self.assertEqual(status.rgb_stamp, stamp)
        self.assertLessEqual(status.target_hz, .5)


if __name__ == '__main__':
    unittest.main()
