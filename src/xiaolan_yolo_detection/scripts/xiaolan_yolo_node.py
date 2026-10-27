#!/usr/bin/env python3
"""Latest-frame YOLO detector with TensorRT GPU and ONNX CPU backends."""
import os
import threading
import time
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from xiaolan_yolo_detection.msg import XiaolanDetection
from xiaolan_yolo_detection.yolo_core import Letterbox, best_box


class YoloNode:
    def __init__(self):
        model = os.path.expanduser(rospy.get_param('~model_path'))
        if not os.path.isfile(model):
            raise ValueError('Missing trained one-class model: '+model+'; see docs/YOLO_SETUP.md')
        self.conf = float(rospy.get_param('~conf_threshold', .5))
        self.hz = float(rospy.get_param('~max_fps', 15.))
        self.max_age = float(rospy.get_param('~max_input_age', .5))
        size = int(rospy.get_param('~input_size', 416))
        if not 0 < self.conf <= 1 or self.hz <= 0 or self.max_age <= 0 or size <= 0:
            raise ValueError('Invalid inference parameters')
        if int(rospy.get_param('~max_det', 1)) != 1:
            raise ValueError('This single-target node requires max_det=1')
        cv2.setNumThreads(1)
        backend = rospy.get_param('~backend', 'tensorrt')
        if backend == 'tensorrt':
            from xiaolan_yolo_detection.tensorrt_runtime import TensorRTSession
            engine = os.path.expanduser(rospy.get_param('~engine_path'))
            self.session = TensorRTSession(engine, model, size)
            self.infer = self.session.infer
            rospy.loginfo('YOLO backend=TensorRT GPU FP16 engine=%s', engine)
        elif backend == 'onnx':
            import onnxruntime as ort
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = int(rospy.get_param('~intra_op_num_threads', 4))
            opts.inter_op_num_threads = int(rospy.get_param('~inter_op_num_threads', 1))
            if min(opts.intra_op_num_threads, opts.inter_op_num_threads) < 1:
                raise ValueError('ORT thread counts must be >=1')
            opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            opts.add_session_config_entry('session.intra_op.allow_spinning', '0')
            opts.add_session_config_entry('session.inter_op.allow_spinning', '0')
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(model, sess_options=opts, providers=['CPUExecutionProvider'])
            inp = self.session.get_inputs()
            if len(inp) != 1 or inp[0].shape != [1, 3, size, size] or inp[0].type != 'tensor(float)':
                raise ValueError('Export static FP32 batch=1 model at configured input_size')
            if len(self.session.get_outputs()) != 1:
                raise ValueError('Expected detection-only raw output, not segmentation/end-to-end')
            self.input_name = inp[0].name
            self.infer = lambda tensor: self.session.run(None, {self.input_name: tensor})[0]
            rospy.loginfo('YOLO backend=ONNX Runtime CPU')
        else:
            raise ValueError('backend must be tensorrt or onnx')
        self.pre = Letterbox(size)
        self.pre.tensor.fill(0)
        warm = self.infer(self.pre.tensor)
        best_box(warm, (1, 1, 0, 0, size, size), self.conf)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.counter = 0
        self.pub = rospy.Publisher(rospy.get_param('~detection_topic', '/xiaolan/detection'), XiaolanDetection, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~rgb_topic', '/camera/color/image_raw'), Image,
                                    self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        with self.lock:
            self.counter += 1
            self.latest = (self.counter, msg, time.monotonic())

    def run(self):
        last = -1
        count, dropped, log_time = 0, 0, time.monotonic()
        while not rospy.is_shutdown():
            started = time.monotonic()
            with self.lock:
                current = self.latest
            if current is None or current[0] == last:
                time.sleep(.005)
                continue
            last, msg, received = current
            out = XiaolanDetection(header=msg.header, image_width=msg.width, image_height=msg.height)
            try:
                age = (rospy.Time.now()-msg.header.stamp).to_sec()
                if msg.header.stamp.to_nsec() == 0 or age < -.05 or age > self.max_age or started-received > self.max_age:
                    raise ValueError('STALE_OR_INVALID_RGB_TIMESTAMP age_s=%.3f local_wait_s=%.3f' %
                                     (age, started-received))
                bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                tensor, mapping = self.pre.prepare(bgr)
                t0 = time.monotonic()
                raw = self.infer(tensor)
                out.inference_ms = (time.monotonic()-t0)*1000
                box = best_box(raw, mapping, self.conf)
                out.reason = 'NO_DETECTION'
                if box:
                    out.bbox_x, out.bbox_y, out.bbox_width, out.bbox_height, out.confidence = box
                    out.center_u = out.bbox_x+(out.bbox_width-1)//2
                    out.center_v = out.bbox_y+(out.bbox_height-1)//2
                    out.detected, out.reason = True, 'DETECTED'
            except Exception as error:
                out.reason = str(error)
                rospy.logwarn_throttle(2., 'YOLO: %s', error)
            self.pub.publish(out)
            if out.inference_ms > 0:
                count += 1
            else:
                dropped += 1
            elapsed = time.monotonic()-log_time
            if elapsed >= 2.:
                rospy.loginfo('YOLO inference_ms=%.1f FPS=%.1f dropped=%d confidence=%.3f detected=%s reason=%s',
                              out.inference_ms, count/elapsed, dropped, out.confidence, out.detected, out.reason)
                count, dropped, log_time = 0, 0, time.monotonic()
            time.sleep(max(0., 1/self.hz-(time.monotonic()-started)))


if __name__ == '__main__':
    rospy.init_node('xiaolan_yolo_node')
    try:
        YoloNode().run()
    except Exception as error:
        rospy.logfatal('Cannot start YOLO: %s', error)
        raise
