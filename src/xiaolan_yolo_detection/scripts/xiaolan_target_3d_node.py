#!/usr/bin/env python3
"""RGB YOLO identity + color-aligned depth location; existing target interface."""
import copy
import json
import threading
import time
from collections import deque
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from xiaolan_yolo_detection.msg import XiaolanDetection, XiaolanTarget
from xiaolan_yolo_detection.yolo_core import depth_median, StableTarget


def validate_alignment(det, depth, aligned_info, color_info):
    """Fail closed on raw-depth inputs. Matching size alone is insufficient."""
    if aligned_info is None:
        raise ValueError('CAMERA_INFO_MISSING')
    expected = (det.image_width, det.image_height, det.header.frame_id)
    if not det.header.frame_id:
        raise ValueError('EMPTY_RGB_FRAME')
    for obj in (depth, aligned_info) + ((color_info,) if color_info is not None else ()):
        if (obj.width, obj.height, obj.header.frame_id) != expected:
            raise ValueError('NOT_COLOR_ALIGNED: dimensions/frame differ')
    for info in (aligned_info,) + ((color_info,) if color_info is not None else ()):
        if len(info.K) != 9 or not np.isfinite(info.K).all() or info.K[0] <= 0 or info.K[4] <= 0:
            raise ValueError('INVALID_INTRINSICS')
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
            raise ValueError('ROI/BINNING_NOT_SUPPORTED')
    if color_info is not None and not np.allclose(aligned_info.K, color_info.K, rtol=1e-4, atol=1e-3):
        raise ValueError('ALIGNED_DEPTH_K_DIFFERS_FROM_COLOR_K')


class Target3DNode:
    def __init__(self):
        p = rospy.get_param
        self.slop = float(p('~depth_sync_tolerance', .025))
        self.wait = float(p('~depth_wait_timeout', .08))
        self.timeout = float(p('~target_timeout', .5))
        self.max_result_age = float(p('~max_result_age', .8))
        self.hold_time = float(p('~max_hold_seconds', .25))
        self.roi = int(p('~roi_size', 9))
        self.min_valid = int(p('~min_valid_depth_pixels', 5))
        self.min_z, self.max_z = float(p('~min_depth', .2)), float(p('~max_depth', 1.))
        self.center = float(p('~center_tolerance_m', .03))
        confirm, lost = int(p('~confirm_frames', 2)), int(p('~lost_tolerance_frames', 3))
        max_x, max_z, alpha = float(p('~max_x_jump_m', .15)), float(p('~max_z_jump_m', .15)), float(p('~ema_alpha', .4))
        vals = [self.slop, self.wait, self.timeout, self.max_result_age, self.hold_time,
                self.min_z, self.max_z, self.center, max_x, max_z, alpha]
        if (not np.isfinite(vals).all() or not 0 <= self.slop <= .05 or self.wait < 0 or
                self.timeout <= 0 or self.max_result_age <= 0 or self.hold_time < 0 or self.roi < 1 or self.roi % 2 == 0 or
                self.min_valid < 1 or not 0 < self.min_z < self.max_z or
                self.center <= 0 or confirm < 1 or lost < 0 or min(max_x, max_z) <= 0 or not 0 < alpha <= 1):
            raise ValueError('Invalid 3D/filter parameters')
        self.filter = StableTarget(confirm, lost, max_x, max_z, alpha)
        self.bridge, self.lock = CvBridge(), threading.Lock()
        cache_size = int(p('~depth_cache_size', 45))
        if cache_size < 1:
            raise ValueError('depth_cache_size must be positive')
        self.depths = deque(maxlen=cache_size)
        self.pending = None
        self.serial = 0
        self.aligned_info = self.color_info = None
        self.last_good_time = None
        self.last_input = time.monotonic()
        self.last_source_stamp = -1
        self.last_watchdog = self.last_log = 0.
        self.pub = rospy.Publisher('/xiaolan/target', XiaolanTarget, queue_size=1)
        self.diag = rospy.Publisher('/xiaolan/target_debug', String, queue_size=1)
        self.subs = [
            rospy.Subscriber(p('~detection_topic', '/xiaolan/detection'), XiaolanDetection, self.detection_cb, queue_size=1),
            rospy.Subscriber(p('~aligned_depth_topic', '/camera/aligned_depth_to_color/image_raw'), Image, self.depth_cb, queue_size=1, buff_size=2**24),
            rospy.Subscriber(p('~aligned_info_topic', '/camera/aligned_depth_to_color/camera_info'), CameraInfo, self.aligned_cb, queue_size=1),
            rospy.Subscriber(p('~color_info_topic', '/camera/color/camera_info'), CameraInfo, self.color_cb, queue_size=1),
        ]

    def depth_cb(self, msg):
        with self.lock:
            self.depths.append(msg)

    def aligned_cb(self, msg):
        with self.lock:
            self.aligned_info = msg

    def color_cb(self, msg):
        with self.lock:
            self.color_info = msg

    def detection_cb(self, msg):
        with self.lock:
            self.serial += 1
            self.pending = (self.serial, msg, time.monotonic())

    def publish(self, value, source, current, reason, dt=None, depth_stats=None):
        out = XiaolanTarget(status='NOT_FOUND')
        out.header = copy.deepcopy((source or current).header) if (source or current) is not None else out.header
        if value is not None and source is not None:
            out.detected = True
            out.x_target, out.z_target = map(float, value)
            for field in ('bbox_x','bbox_y','bbox_width','bbox_height','center_u','center_v'):
                setattr(out, field, getattr(source, field))
            out.area = float(source.bbox_width*source.bbox_height)
            out.status = 'CENTER' if abs(out.x_target) <= self.center else ('LEFT' if out.x_target < 0 else 'RIGHT')
        self.pub.publish(out)
        confidence = float(source.confidence if source is not None else current.confidence if current is not None else 0.)
        self.diag.publish(String(data=json.dumps(dict(reason=reason, confidence=confidence,
            depth_rgb_dt=dt, status=out.status, x=out.x_target, z=out.z_target,
            rgb_age_s=(rospy.Time.now()-current.header.stamp).to_sec() if current is not None else None,
            depth_stats=depth_stats))))
        if time.monotonic()-self.last_log >= 2.:
            rospy.loginfo('3D confidence=%.3f X=%+.3f Z=%.3f status=%s reason=%s depth_rgb_dt=%s',
                          confidence, out.x_target, out.z_target, out.status, reason, dt)
            self.last_log = time.monotonic()
            if depth_stats is not None and depth_stats['reason'] != 'MEASURED':
                rospy.logwarn('Depth ROI: %s', json.dumps(depth_stats))

    def process(self, det, depth, ai, ci):
        now = time.monotonic()
        # Confirmation follows the input cadence, not the lifetime of held data.
        if now-self.last_input > self.timeout:
            self.filter.reset()
            self.last_good_time = None
        self.last_input = now
        age = (rospy.Time.now()-det.header.stamp).to_sec()
        reason = None
        if det.header.stamp.to_nsec() <= 0 or age < -.05:
            reason = 'INVALID_RGB_TIMESTAMP'
        elif det.header.stamp.to_nsec() <= self.last_source_stamp:
            reason = 'OUT_OF_ORDER_RGB'
        elif age > self.max_result_age:
            reason = 'STALE_RGB_RESULT'
        if reason is not None:
            self.filter.reset()
            self.last_good_time = None
            self.publish(None, None, det, reason)
            return
        self.last_source_stamp = det.header.stamp.to_nsec()
        measurement, reason, dt = None, det.reason or 'NO_DETECTION', None
        depth_stats = {}
        if det.detected:
            if depth is None:
                reason = 'NO_ALIGNED_DEPTH_WITHIN_TOLERANCE'
            else:
                try:
                    validate_alignment(det, depth, ai, ci)
                    dt = (depth.header.stamp-det.header.stamp).to_sec()
                    arr = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
                    box = (det.bbox_x, det.bbox_y, det.bbox_width, det.bbox_height)
                    z = depth_median(arr, depth.encoding, box, (det.center_u, det.center_v),
                                     self.roi, self.min_valid, self.min_z, self.max_z,
                                     diagnostics=depth_stats)
                    reason = depth_stats['reason']
                    if z is not None:
                        x = (det.center_u-ai.K[2])*z/ai.K[0]
                        measurement, reason = (x, z), 'MEASURED'
                except Exception as error:
                    # A calibration/format error must never retain valid control data.
                    self.filter.reset()
                    self.publish(None, None, det, str(error), dt)
                    return
        value, source, state = self.filter.update(measurement, det)
        if state == 'VALID':
            self.last_good_time = now
        if measurement is not None and state in ('HELD', 'NOT_FOUND'):
            reason = 'JUMP_REJECTED'
        if state == 'HELD' and (self.last_good_time is None or
                now-self.last_good_time > self.hold_time or
                (rospy.Time.now()-source.header.stamp).to_sec() > self.max_result_age):
            self.filter.reset()
            self.last_good_time = None
            value, source, state = None, None, 'NOT_FOUND'
            reason = 'HOLD_EXPIRED'
        self.publish(value, source, det, state+': '+reason, dt, depth_stats or None)

    def watchdog(self, now):
        reason = None
        if now-self.last_input > self.timeout:
            reason = 'DETECTION_STREAM_TIMEOUT'
        elif self.filter.active:
            if self.filter.misses and (self.last_good_time is None or
                    now-self.last_good_time > self.hold_time):
                reason = 'HOLD_EXPIRED'
            elif (rospy.Time.now()-self.filter.payload.header.stamp).to_sec() > self.max_result_age:
                reason = 'STALE_RGB_RESULT'
        if reason is not None:
            # Expiring a published result does not break a continuing input stream.
            # Keep confirmation history until an actual gap/loss or invalid input.
            if reason != 'STALE_RGB_RESULT':
                self.filter.reset()
                self.last_good_time = None
            self.publish(None, None, None, reason)

    def run(self):
        handled = -1
        while not rospy.is_shutdown():
            now = time.monotonic()
            with self.lock:
                pending = self.pending
                depths = list(self.depths)
                ai, ci = self.aligned_info, self.color_info
            if pending is not None and pending[0] != handled:
                serial, det, received = pending
                nearest = min(depths, key=lambda d: abs(d.header.stamp.to_nsec()-det.header.stamp.to_nsec())) if depths else None
                if nearest is not None and abs((nearest.header.stamp-det.header.stamp).to_sec()) > self.slop:
                    nearest = None
                if not det.detected or nearest is not None or now-received >= self.wait:
                    handled = serial
                    self.process(det, nearest, ai, ci)
            if now-self.last_watchdog >= .1:
                self.watchdog(now)
                self.last_watchdog = now
            time.sleep(.005)


if __name__ == '__main__':
    rospy.init_node('xiaolan_target_3d_node')
    Target3DNode().run()
