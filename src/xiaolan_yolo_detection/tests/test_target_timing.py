"""NX inference cadence must not be mistaken for a lost target."""
import runpy
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from xiaolan_yolo_detection.msg import XiaolanDetection

MODULE = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                           'scripts/xiaolan_target_3d_node.py'))
Target3DNode = MODULE['Target3DNode']


class TargetTimingTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        for target, replacement in [
                ('time.monotonic', lambda: self.now),
                ('rospy.Time.now', lambda: rospy.Time.from_sec(self.now)),
                ('rospy.get_param', lambda key, default=None: default),
                ('rospy.Publisher', Mock()), ('rospy.Subscriber', Mock())]:
            p = patch(target, replacement)
            p.start()
            self.addCleanup(p.stop)
        self.node = Target3DNode()
        self.node.publish = Mock()

    def frame(self, interval=.32, age=.35, detected=True, x=0):
        self.now += interval
        det = XiaolanDetection(detected=detected, confidence=.95,
                               bbox_x=4, bbox_y=4, bbox_width=9, bbox_height=9,
                               center_u=8+x, center_v=8, image_width=32, image_height=32)
        det.header.stamp = rospy.Time.from_sec(self.now-age)
        det.header.frame_id = 'camera_color_optical_frame'
        info = CameraInfo(width=32, height=32)
        info.header = det.header
        info.K = [100, 0, 8, 0, 100, 8, 0, 0, 1]
        depth = CvBridge().cv2_to_imgmsg(np.full((32, 32), 600, np.uint16), '16UC1')
        depth.header = det.header
        self.node.process(det, depth, info, info)
        return self.node.publish.call_args[0]

    def test_three_fps_stays_valid_after_confirmation(self):
        self.assertEqual(self.frame()[3], 'CONFIRMING: MEASURED')
        for interval in [.30, .36, .28, .40, .32, .35]:
            self.assertEqual(self.frame(interval=interval)[3], 'VALID: MEASURED')
            self.node.watchdog(self.now+.01)
            self.assertTrue(self.node.filter.active)

    def test_slow_inference_uses_result_age_budget(self):
        self.assertEqual(self.frame(age=.60)[3], 'CONFIRMING: MEASURED')
        self.assertEqual(self.frame(age=.65)[3], 'VALID: MEASURED')
        self.assertEqual(self.frame(age=.90)[3], 'STALE_RGB_RESULT')
        self.assertFalse(self.node.filter.active)

    def test_stream_gap_requires_confirmation_again(self):
        self.frame(); self.frame()
        self.now += .51
        self.node.watchdog(self.now)
        self.assertEqual(self.node.publish.call_args[0][3], 'DETECTION_STREAM_TIMEOUT')
        self.assertEqual(self.frame()[3], 'CONFIRMING: MEASURED')
        # Even without a watchdog tick, a late frame cannot continue old confirmation.
        self.assertEqual(self.frame(interval=.6)[3], 'CONFIRMING: MEASURED')

    def test_lost_target_cannot_extend_hold(self):
        self.frame(); self.frame()
        self.assertEqual(self.frame(interval=.1, detected=False)[3], 'HELD: NO_DETECTION')
        self.now += .16
        self.node.watchdog(self.now)
        self.assertEqual(self.node.publish.call_args[0][3], 'HOLD_EXPIRED')
        self.assertFalse(self.node.filter.active)
        self.frame(); self.frame()
        self.assertEqual(self.frame(detected=False)[3], 'NOT_FOUND: HOLD_EXPIRED')

    def test_source_age_expires_even_with_no_new_input(self):
        self.frame(age=.65); self.frame(age=.65)
        self.now += .16
        self.node.watchdog(self.now)
        self.assertEqual(self.node.publish.call_args[0][3], 'STALE_RGB_RESULT')
        self.assertIsNone(self.node.publish.call_args[0][0])
        self.assertEqual(self.frame(interval=.17, age=.65)[3], 'VALID: MEASURED')

    def test_hold_expiry_resets_history_even_when_source_is_also_old(self):
        self.frame(age=.65); self.frame(age=.65)
        self.assertEqual(self.frame(interval=.1, age=.65, detected=False)[3], 'HELD: NO_DETECTION')
        self.now += .16
        self.node.watchdog(self.now)
        self.assertEqual(self.node.publish.call_args[0][3], 'HOLD_EXPIRED')
        self.assertFalse(self.node.filter.active)

    def test_future_and_out_of_order_frames_are_rejected(self):
        self.assertEqual(self.frame(age=-.1)[3], 'INVALID_RGB_TIMESTAMP')
        self.frame(); self.frame()
        self.assertEqual(self.frame(interval=.1, age=.55)[3], 'OUT_OF_ORDER_RGB')
        self.assertFalse(self.node.filter.active)
