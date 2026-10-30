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
from xiaolan_yolo_detection.msg import XiaolanDetection, XiaolanTarget, PerceptionStatus
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


def validate_depth_source(depth, now, max_age):
    if depth.encoding not in ('16UC1', '32FC1'):
        raise ValueError('UNSUPPORTED_DEPTH_ENCODING')
    stamp = depth.header.stamp
    age = (now-stamp).to_sec()
    if stamp.to_nsec() <= 0 or age < -.05:
        raise ValueError('INVALID_DEPTH_TIMESTAMP')
    if age > max_age:
        raise ValueError('STALE_DEPTH_STREAM')


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
        self.health_hz = float(p('~health_publish_hz', 10.))
        self.diagnostics_period = float(p('~diagnostics_period_s', 2.))
        if not np.isfinite([self.health_hz, self.diagnostics_period]).all() or self.health_hz <= 0 or self.diagnostics_period <= 0:
            raise ValueError('Invalid health parameters')
        self.last_health_publish = self.last_health_log = 0.
        self.last_health_key = None
        self.last_detection = self.last_depth = None
        self.last_inference_ms = 0.
        self.window = deque()
        self.window_start = time.monotonic()
        self.last_summary = {}
        self.health_state = PerceptionStatus.WAITING
        self.health_reason = 'WAITING_FOR_INPUTS'
        self.pub = rospy.Publisher('/xiaolan/target', XiaolanTarget, queue_size=1)
        self.diag = rospy.Publisher('/xiaolan/target_debug', String, queue_size=1)
        self.health_pub = rospy.Publisher('/xiaolan/perception_status', PerceptionStatus, queue_size=1)
        self.subs = [
            rospy.Subscriber(p('~detection_topic', '/xiaolan/detection'), XiaolanDetection, self.detection_cb, queue_size=1),
            rospy.Subscriber(p('~aligned_depth_topic', '/camera/aligned_depth_to_color/image_raw'), Image, self.depth_cb, queue_size=1, buff_size=2**24),
            rospy.Subscriber(p('~aligned_info_topic', '/camera/aligned_depth_to_color/camera_info'), CameraInfo, self.aligned_cb, queue_size=1),
            rospy.Subscriber(p('~color_info_topic', '/camera/color/camera_info'), CameraInfo, self.color_cb, queue_size=1),
        ]

    def depth_cb(self, msg):
        with self.lock:
            self.depths.append(msg)
            self.last_depth = msg
            self.window.append(('depth', time.monotonic()))

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

    def _rate(self, kind, now):
        with self.lock:
            while self.window and now-self.window[0][1] > 2.:
                self.window.popleft()
            elapsed = now-self.window_start
            if not self.window or elapsed < .1:
                return 0.
            elapsed = min(2., elapsed)
            return sum(item[0] == kind for item in self.window)/elapsed

    def set_health(self, state, reason, det=None, depth=None):
        self.health_state, self.health_reason = state, reason
        if det is not None:
            self.last_detection = det
            self.last_inference_ms = float(det.inference_ms)
        if depth is not None:
            self.last_depth = depth

    def publish_health(self, now, force=False):
        key = (self.health_state, self.health_reason)
        if not force and key == self.last_health_key and now-self.last_health_publish < 1./self.health_hz:
            return
        det, depth = self.last_detection, self.last_depth
        out = PerceptionStatus(state=self.health_state, reason=self.health_reason,
                               detection_hz=self._rate('detection', now), depth_hz=self._rate('depth', now),
                               target_hz=self._rate('target', now), inference_ms=self.last_inference_ms)
        if det is not None:
            out.rgb_stamp = det.header.stamp
            out.header.frame_id = det.header.frame_id
        if depth is not None:
            out.depth_stamp = depth.header.stamp
        if det is not None and depth is not None:
            out.header.stamp = min(det.header.stamp, depth.header.stamp)
        self.health_pub.publish(out)
        summary = self.last_summary
        source_age = (rospy.Time.now()-summary['source'].header.stamp).to_sec() if summary.get('source') is not None else None
        input_age = (rospy.Time.now()-summary['current'].header.stamp).to_sec() if summary.get('current') is not None else None
        if key != self.last_health_key and now-self.last_health_log >= .5:
            rospy.loginfo('3D state=%d reason=%s X=%s Z=%s source_age=%s input_age=%s skew=%s infer_ms=%.1f det_hz=%.1f depth_hz=%.1f target_hz=%.1f', out.state, out.reason, summary.get('x'), summary.get('z'), source_age, input_age, summary.get('skew'), out.inference_ms, out.detection_hz, out.depth_hz, out.target_hz)
            self.last_health_log = now
        elif now-self.last_health_log >= self.diagnostics_period:
            rospy.loginfo('3D summary state=%d reason=%s X=%s Z=%s source_age=%s input_age=%s skew=%s infer_ms=%.1f det_hz=%.1f depth_hz=%.1f target_hz=%.1f', out.state, out.reason, summary.get('x'), summary.get('z'), source_age, input_age, summary.get('skew'), out.inference_ms, out.detection_hz, out.depth_hz, out.target_hz)
            self.last_health_log = now
        self.last_health_key, self.last_health_publish = key, now

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
        self.last_summary = dict(x=out.x_target, z=out.z_target, reason=reason,
                                 source=source, current=current, skew=dt)

    def process(self, det, depth, ai, ci):
        now = time.monotonic()
        with self.lock:
            self.window.append(('detection', now))
        self.last_detection = det
        if depth is not None:
            self.last_depth = depth
        self.last_inference_ms = float(det.inference_ms)
        # Every processed frame reclassifies health; an old fault cannot stick.
        self.health_state, self.health_reason = PerceptionStatus.WAITING, 'PROCESSING'
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
            self.set_health(PerceptionStatus.FAULT, reason, det, depth)
            return
        self.last_source_stamp = det.header.stamp.to_nsec()
        measurement, reason, dt = None, det.reason or 'NO_DETECTION', None
        depth_stats = {}
        if not det.detected:
            if det.reason != 'NO_DETECTION':
                reason = det.reason or 'INFERENCE_FAILURE'
                self.set_health(PerceptionStatus.FAULT, reason, det, depth)
            elif depth is None:
                reason = 'NO_ALIGNED_DEPTH_WITHIN_TOLERANCE'
                self.set_health(PerceptionStatus.FAULT, reason, det, depth)
            else:
                try:
                    validate_depth_source(depth, rospy.Time.now(), self.max_result_age)
                    validate_alignment(det, depth, ai, ci)
                    self.set_health(PerceptionStatus.NO_TARGET, 'NO_DETECTION', det, depth)
                except Exception as error:
                    reason = str(error)
                    self.set_health(PerceptionStatus.FAULT, reason, det, depth)
        else:
            if depth is None:
                reason = 'NO_ALIGNED_DEPTH_WITHIN_TOLERANCE'
                self.set_health(PerceptionStatus.FAULT, reason, det, depth)
            else:
                try:
                    validate_depth_source(depth, rospy.Time.now(), self.max_result_age)
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
                    self.set_health(PerceptionStatus.FAULT, str(error), det, depth)
                    return
        value, source, state = self.filter.update(measurement, det)
        if state == 'VALID':
            self.last_good_time = now
            with self.lock:
                self.window.append(('target', now))
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
        if det.detected and self.health_state != PerceptionStatus.FAULT:
            self.set_health(PerceptionStatus.TARGET_VALID if state == 'VALID' else PerceptionStatus.TARGET_PENDING,
                            state+'_'+reason, det, depth)

    def watchdog(self, now):
        reason = None
        depth_stale = self.last_depth is not None and (self.last_depth.header.stamp.to_nsec() <= 0 or
                (rospy.Time.now()-self.last_depth.header.stamp).to_sec() > self.max_result_age)
        if self.last_detection is None:
            reason = 'DETECTION_STREAM_TIMEOUT'
        elif self.last_depth is None:
            reason = 'DEPTH_STREAM_TIMEOUT'
        elif now-self.last_input > self.timeout:
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
            self.set_health(PerceptionStatus.FAULT, 'DEPTH_STREAM_TIMEOUT' if depth_stale else reason)
        elif depth_stale:
            self.set_health(PerceptionStatus.FAULT, 'DEPTH_STREAM_TIMEOUT')
        elif self.last_detection is None or self.last_depth is None:
            self.set_health(PerceptionStatus.WAITING, 'WAITING_FOR_INPUTS')
        self.publish_health(now)

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
                # Negative health can use the latest validated depth stream;
                # target geometry remains constrained to the synch tolerance.
                health_depth = nearest
                if not det.detected and depths:
                    health_depth = depths[-1]
                if (det.detected and (nearest is not None or now-received >= self.wait)) or (
                        not det.detected and (health_depth is not None or now-received >= self.wait)):
                    handled = serial
                    self.process(det, nearest if det.detected else health_depth, ai, ci)
            if now-self.last_watchdog >= .1:
                self.watchdog(now)
                self.last_watchdog = now
            time.sleep(.005)


if __name__ == '__main__':
    rospy.init_node('xiaolan_target_3d_node')
    Target3DNode().run()
