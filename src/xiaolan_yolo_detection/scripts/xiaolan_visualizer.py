#!/usr/bin/env python3
"""YOLO-chain RGB preview. All bbox/center/header coordinates are RGB.

Display association is approximate and NEVER feeds detection or control.
No depth-to-RGB scaling, projection or depth heatmap is used here.
"""
import json
import os
import threading
import time
from collections import OrderedDict
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from xiaolan_yolo_detection.msg import XiaolanDetection, XiaolanTarget


def header_key(msg):
    # rospy assigns seq independently for each publisher. Only the source
    # timestamp and optical frame survive RGB -> detection -> target publishing.
    return msg.header.stamp.to_nsec(), msg.header.frame_id


def render_rgb(image, detection=None, target=None, info=None, tolerance=.03, notice='', dt=None):
    image = image.copy()
    h, w = image.shape[:2]
    calibrated = info is not None and info.width == w and info.height == h and info.K[0] > 0
    cx = int(round(info.K[2])) if calibrated else w//2
    has_target = target is not None and target.detected
    status = target.status if has_target else 'NOT_FOUND'
    color = (0, 230, 0) if status == 'CENTER' else (0, 210, 255) if status in ('LEFT','RIGHT') else (0,0,255)
    if has_target and calibrated and np.isfinite(target.z_target) and target.z_target > 0:
        radius = info.K[0]*tolerance/target.z_target
        left, right = int(np.clip(cx-radius, 0, w-1)), int(np.clip(cx+radius, 0, w-1))
        tint = image.copy()
        cv2.rectangle(tint, (left,0), (right,h-1), (0,180,0), -1)
        cv2.addWeighted(tint, .12, image, .88, 0, dst=image)
        cv2.line(image, (left,0), (left,h-1), (0,160,0), 1)
        cv2.line(image, (right,0), (right,h-1), (0,160,0), 1)
    cv2.line(image, (cx,0), (cx,h-1), (255,180,0), 2)
    box = detection if detection is not None and detection.detected else None
    if box is not None:
        x, y, bw, bh = box.bbox_x, box.bbox_y, box.bbox_width, box.bbox_height
        point = (box.center_u, box.center_v)
        cv2.rectangle(image, (x,y), (x+bw-1,y+bh-1), color, 2)
        cv2.circle(image, point, 5, color, -1)
        cv2.line(image, (cx,point[1]), point, color, 2)
    confidence = '%.3f' % detection.confidence if detection is not None else '--'
    metrics = 'X_target=%+.3f m  Z_target=%.3f m' % (target.x_target, target.z_target) if has_target else 'X_target=--  Z_target=--'
    labels = [status+' | confidence='+confidence, metrics,
              notice or ('YOLO BOX / 3D NOT CONFIRMED' if box is not None and not has_target else 'RGB TARGET'),
              'Display dt=%s ms | %s | Q: close' % ('--' if dt is None else '%.1f' % (1000*dt),
                       'optical axis' if calibrated else 'image center (no CameraInfo)')]
    tint = image.copy()
    cv2.rectangle(tint, (0,0), (w-1,min(135,h-1)), (0,0,0), -1)
    cv2.addWeighted(tint,.65,image,.35,0,dst=image)
    for i, text in enumerate(labels):
        cv2.putText(image, text, (10,26+i*29), cv2.FONT_HERSHEY_SIMPLEX,
                    .6 if i < 2 else .45, color if i == 0 else (255,255,255), 1, cv2.LINE_AA)
    return image


class XiaolanVisualizer:
    def __init__(self):
        p = rospy.get_param
        cv2.setNumThreads(1)
        self.max_fps = float(p('~visualization_max_fps', 5.))
        if not np.isfinite(self.max_fps) or self.max_fps <= 0:
            raise ValueError('visualization_max_fps must be positive')
        self.timeout = float(p('~target_timeout', .5))
        self.slop = float(p('~rgb_sync_tolerance', .05))
        self.live_dt = float(p('~visualization_max_dt', .25))
        self.tolerance = float(p('~center_tolerance_m', .03))
        if not np.isfinite([self.timeout,self.slop,self.live_dt,self.tolerance]).all() or min(self.timeout,self.live_dt) <= 0 or min(self.slop,self.tolerance) < 0:
            raise ValueError('Invalid display timing/tolerance')
        self.show_window = bool(p('~show_window', True)) and bool(os.environ.get('DISPLAY'))
        self.lock, self.bridge = threading.Lock(), CvBridge()
        self.colors, self.detections = OrderedDict(), OrderedDict()
        self.latest_rgb = self.latest_detection = self.latest_target = None
        self.info = None
        self.detail = ''
        self.last_display = -1
        self.window = 'Xiaolan YOLO RGB'
        self.pub = rospy.Publisher('~image', Image, queue_size=1)
        self.subs = [
            rospy.Subscriber(p('~rgb_topic','/camera/color/image_raw'), Image, self.rgb_cb, queue_size=1,buff_size=2**24),
            rospy.Subscriber(p('~detection_topic','/xiaolan/detection'), XiaolanDetection,self.detection_cb,queue_size=1),
            rospy.Subscriber('/xiaolan/target',XiaolanTarget,self.target_cb,queue_size=1),
            rospy.Subscriber(p('~aligned_info_topic','/camera/aligned_depth_to_color/camera_info'),CameraInfo,self.info_cb,queue_size=1),
            rospy.Subscriber('/xiaolan/target_debug',String,self.debug_cb,queue_size=1),
        ]

    def rgb_cb(self,msg):
        with self.lock:
            self.latest_rgb = (msg,time.monotonic())
            self.colors[header_key(msg)] = self.latest_rgb
            while len(self.colors)>30:
                self.colors.popitem(last=False)

    def detection_cb(self,msg):
        with self.lock:
            self.latest_detection = (msg,time.monotonic())
            self.detections[header_key(msg)] = self.latest_detection
            while len(self.detections)>30:
                self.detections.popitem(last=False)

    def target_cb(self,msg):
        with self.lock:
            self.latest_target = (msg,time.monotonic())

    def info_cb(self,msg):
        with self.lock:
            self.info = msg

    def debug_cb(self,msg):
        try:
            detail = json.loads(msg.data).get('reason','')
        except (ValueError,TypeError):
            return
        with self.lock:
            self.detail = detail

    def render(self):
        now = time.monotonic()
        with self.lock:
            rgb, det, target, info, notice = self.latest_rgb,self.latest_detection,self.latest_target,self.info,self.detail
            colors = list(self.colors.values())
            detections = dict(self.detections)
        if rgb is None:
            return None,None
        det = det[0] if det and now-det[1] <= self.timeout else None
        target = target[0] if target and now-target[1] <= self.timeout else None
        # Held measurements retain their original header and corresponding bbox/confidence.
        if target is not None and target.detected:
            corresponding = detections.get(header_key(target))
            if corresponding is not None and now-corresponding[1] <= self.timeout:
                det = corresponding[0]
            else:
                target = None
                notice = 'TARGET SOURCE FRAME EXPIRED'
        dt = None
        if det is not None:
            nearest = min(colors,key=lambda item: abs(item[0].header.stamp.to_nsec()-det.header.stamp.to_nsec()))
            nearest_dt = (nearest[0].header.stamp-det.header.stamp).to_sec()
            if abs(nearest_dt) <= self.slop and nearest[0].header.stamp.to_nsec() > self.last_display:
                rgb = nearest
            dt = (rgb[0].header.stamp-det.header.stamp).to_sec()
            if abs(dt) > self.live_dt or rgb[0].header.frame_id != det.header.frame_id or (rgb[0].width,rgb[0].height)!=(det.image_width,det.image_height):
                det = target = None
                notice = 'OVERLAY EXPIRED / RGB FRAME MISMATCH'
        if now-rgb[1] > self.timeout:
            det = target = None
            notice = 'STALE RGB'
        if info is not None and info.header.frame_id != rgb[0].header.frame_id:
            info = None
        bgr = self.bridge.imgmsg_to_cv2(rgb[0],'bgr8')
        output = render_rgb(bgr,det,target,info,self.tolerance,notice,dt)
        self.last_display = max(self.last_display,rgb[0].header.stamp.to_nsec())
        return output,rgb[0].header

    def run(self):
        visible_once = False
        if self.show_window:
            cv2.namedWindow(self.window,cv2.WINDOW_NORMAL)
        try:
            while not rospy.is_shutdown():
                try:
                    image,header = self.render()
                    if image is not None:
                        msg = self.bridge.cv2_to_imgmsg(image,'bgr8')
                        msg.header = header
                        self.pub.publish(msg)
                        if self.show_window:
                            cv2.imshow(self.window,image)
                except (CvBridgeError,cv2.error,ValueError) as error:
                    rospy.logwarn_throttle(2.,'Visualizer: %s',error)
                if self.show_window:
                    if cv2.waitKey(1)&255 in (27,ord('q'),ord('Q')):
                        break
                    visible = cv2.getWindowProperty(self.window,cv2.WND_PROP_VISIBLE)
                    if visible >= 1:
                        visible_once = True
                    elif visible == 0 and visible_once:
                        break
                time.sleep(1.0/self.max_fps)
        finally:
            if self.show_window:
                cv2.destroyAllWindows()


if __name__=='__main__':
    rospy.init_node('xiaolan_visualizer')
    XiaolanVisualizer().run()
