#!/usr/bin/env python3
"""Approach closed-loop check: perception -> commands -> body motion -> perception.

Isolated master only; starts the compiled bt_control_node (fake servo boards,
no hardware). A kinematic world model closes the loop:

* body pose: integrated from the commanded joints (ideal servo, q_cur = q_des)
  by rigid 2-D registration of the stance feet between frames (no-slip leg
  odometry; stance = feet within 2 mm of the lowest foot);
* /xiaolan/target + /xiaolan/perception_status: Xiaolan front-face centre in
  camera_color_optical_frame, with latency, noise, FOV and depth limits;
* /xiaolan/tag_detections: the tag pose in the camera frame (tag mount yaw
  error, camera pitch, FOV/range/incidence visibility);
* /grasp_hexapod/imu: body yaw.

The tag reference is "taught" in the same model at the climb start pose P0
(offset 0), exactly as record_tag_reference.py would print it. Then
home -> spin_search -> approach runs from a start pose, and the final body
pose is compared with P0.

Geometry (world: X toward Xiaolan, Y left, Z up) comes from
config/climb_front_v16_1.json: Xiaolan origin (0.45, -0.03), box
x +-0.355, y -0.373..0.307, height 0.248; P0 base (-0.1, -0.0677), yaw -90 deg.
The camera extrinsic is a model parameter. The 0.02 m forward offset matches
the measured ~0.15 m camera-to-tag depth when the front legs touch Xiaolan.

Simulation-only evidence: ideal servo, no slip, synthetic perception. It does
not prove hardware behaviour.

Usage (an isolated master must already run, e.g. roscore -p 11313):
    ROS_MASTER_URI=http://localhost:11313 python3 approach_closed_loop_e2e.py \
        --case far_offset [--out /tmp/x.npz]
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import rospy
import yaml
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from grasp_hexapod_msgs.msg import ModeStatus
from grasp_hexapod_msgs.srv import SwitchMode
from sensor_msgs.msg import Imu
from xiaolan_yolo_detection.msg import PerceptionStatus, XiaolanTarget

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "grasp_hexapod_control" / "scripts"))
from fake_hardware_e2e import FakeHardware, call_mode  # noqa: E402
from kinematics import FOOT_RADIUS, GraspKinematic, Q_STAND  # noqa: E402

FRAME = "camera_color_optical_frame"
# Xiaolan near face and the climb start pose (climb_front_v16_1.json).
FACE_X = 0.45 - 0.355
FACE_Y = (-0.03 - 0.373, -0.03 + 0.307)
FACE_TOP = 0.248
P0 = np.array([-0.1, -0.0677, -math.pi / 2])
# Camera model: 848x480, roughly 90 x 65 deg.
IMG_W, IMG_H, FX, FY = 848, 480, 424.0, 377.0

CASES = {
    # name: (start offset from P0 in the P0 body frame: back [m], left [m], yaw [deg])
    "straight": (0.70, 0.00, 0.0),
    "far_offset": (0.90, 0.12, 20.0),
    "search": (0.80, -0.10, 70.0),   # Xiaolan outside the FOV: spin_search must turn
    "tag_dropout": (0.70, 0.05, -10.0),  # tag hidden below 0.24 m depth -> blind finish
}


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def quat_from_matrix(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return x, y, z, w


class World:
    """Kinematic world model; base pose (X, Y, psi) with psi = yaw of base +x."""

    def __init__(self, args):
        self.kin = GraspKinematic()
        self.args = args
        th = math.radians(args.camera_pitch_deg)
        # Optical axes in the base frame (x right, y forward, z up).
        z_opt = np.array([0.0, math.cos(th), -math.sin(th)])
        x_opt = np.array([1.0, 0.0, 0.0])
        self.r_base_opt = np.column_stack([x_opt, np.cross(z_opt, x_opt), z_opt])
        self.cam_base = np.array([0.0, args.camera_forward_m, args.camera_up_m])
        # Tag frame in the world at a square view: x right (-Y), y down, z into the face.
        r_square = np.column_stack([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        self.r_w_tag = rot_z(math.radians(args.tag_mount_yaw_deg)) @ r_square
        self.tag_w = np.array([FACE_X, P0[1] + args.tag_left_m, args.tag_height_m])
        self.target_w = np.array([FACE_X, 0.5 * sum(FACE_Y), 0.5 * FACE_TOP])
        feet = self.kin.forward_base(Q_STAND)
        self.stand_height = FOOT_RADIUS - float(np.mean(feet[:, 2]))
        self.lock = threading.Lock()
        self.pose = P0.copy()
        self.height = self.stand_height
        self.prev_feet = None
        self.history = []  # (time, pose, height)
        self.trace = []

    def place(self, pose):
        with self.lock:
            self.pose = np.array(pose, dtype=float)
            self.prev_feet = None
            self.history = []

    def camera(self, pose, height):
        r = rot_z(pose[2])
        cam_w = np.array([pose[0], pose[1], height]) + r @ self.cam_base
        r_w_opt = r @ self.r_base_opt
        return cam_w, r_w_opt

    def in_camera(self, point_w, pose, height):
        cam_w, r_w_opt = self.camera(pose, height)
        return r_w_opt.T @ (point_w - cam_w)

    def project(self, p):
        if p[2] <= 1e-3:
            return None
        return FX * p[0] / p[2] + IMG_W / 2, FY * p[1] / p[2] + IMG_H / 2

    def step(self, q, now):
        """Advance the body pose from the commanded joints (ideal servo)."""
        feet = self.kin.forward_base(q)
        with self.lock:
            if self.prev_feet is not None:
                zmin = min(feet[:, 2].min(), self.prev_feet[:, 2].min())
                legs = [i for i in range(6)
                        if feet[i, 2] < zmin + 0.002 and self.prev_feet[i, 2] < zmin + 0.002]
                if len(legs) >= 3:
                    a, b = self.prev_feet[legs, :2], feet[legs, :2]
                    ca, cb = a.mean(0), b.mean(0)
                    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
                    r = vt.T @ u.T
                    if np.linalg.det(r) < 0:
                        vt[-1] *= -1
                        r = vt.T @ u.T
                    t = cb - r @ ca  # feet: b = R a + t in base coordinates
                    # The body moved by the inverse transform, expressed in the old base frame.
                    ri = r.T
                    d_base = -ri @ t
                    dpsi = math.atan2(ri[1, 0], ri[0, 0])
                    rw = rot_z(self.pose[2])[:2, :2]
                    self.pose[:2] += rw @ d_base
                    self.pose[2] += dpsi
                    self.height = FOOT_RADIUS - float(np.mean(feet[legs, 2]))
            self.prev_feet = feet
            self.history.append((now, self.pose.copy(), self.height))
            if len(self.history) > 120:
                self.history = self.history[-120:]
            self.trace.append((now, *self.pose, self.height))

    def sample(self, latency):
        """Pose `latency` seconds ago (nearest recorded frame)."""
        with self.lock:
            if not self.history:
                return None
            target = self.history[-1][0] - latency
            for stamp, pose, height in reversed(self.history):
                if stamp <= target:
                    return stamp, pose.copy(), height
            return self.history[0][0], self.history[0][1].copy(), self.history[0][2]

    def tag_reference(self):
        """What record_tag_reference.py prints at P0 (offset 0): x, z, yaw."""
        p = self.in_camera(self.tag_w, P0, self.stand_height)
        cam_w, r_w_opt = self.camera(P0, self.stand_height)
        r = r_w_opt.T @ self.r_w_tag
        return float(p[0]), float(p[2]), math.atan2(r[2, 0], r[0, 0])

    def tag_view(self, pose, height):
        p = self.in_camera(self.tag_w, pose, height)
        if not (self.args.tag_min_range_m <= p[2] <= self.args.tag_max_range_m):
            return None
        cam_w, r_w_opt = self.camera(pose, height)
        r = r_w_opt.T @ self.r_w_tag
        # Incidence: angle between the tag normal and the line of sight.
        if abs(math.acos(max(-1.0, min(1.0, r[:, 2] @ (p / np.linalg.norm(p)))))) > math.radians(60):
            return None
        half = self.args.tag_size_m / 2
        for cx, cy in ((-half, -half), (half, -half), (half, half), (-half, half)):
            uv = self.project(p + r @ np.array([cx, cy, 0.0]))
            if uv is None or not (0 <= uv[0] <= IMG_W and 0 <= uv[1] <= IMG_H):
                return None
        return p, r

    def target_view(self, pose, height):
        p = self.in_camera(self.target_w, pose, height)
        uv = self.project(p)
        if uv is None or not (0 <= uv[0] <= IMG_W and 0 <= uv[1] <= IMG_H):
            return None
        if not (0.20 <= p[2] <= self.args.yolo_max_depth_m):
            return None
        return p, uv


def write_config(args, world, path):
    base = yaml.safe_load(open(HERE.parent / "config" / "phase1_visual.yaml"))
    x, z, yaw = world.tag_reference()
    base.update(tag_guidance_enabled=True, tag_guidance_id=args.tag_id,
                tag_ref_x_m=x, tag_p0_z_m=z, tag_ref_yaw_rad=yaw, tag_imu_yaw_sign=1.0)
    path.write_text(yaml.safe_dump(base))
    return x, z, yaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=sorted(CASES), default="far_offset")
    parser.add_argument("--camera-pitch-deg", type=float, default=3.0)
    parser.add_argument("--camera-forward-m", type=float, default=0.02)
    parser.add_argument("--camera-up-m", type=float, default=0.035)
    parser.add_argument("--tag-left-m", type=float, default=0.04)
    parser.add_argument("--tag-height-m", type=float, default=0.13)
    parser.add_argument("--tag-mount-yaw-deg", type=float, default=1.5)
    parser.add_argument("--tag-size-m", type=float, default=0.05)
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-min-range-m", type=float, default=0.10)
    parser.add_argument("--tag-max-range-m", type=float, default=0.90)
    parser.add_argument("--yolo-max-depth-m", type=float, default=2.5)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--timeout-s", type=float, default=240.0)
    parser.add_argument("--no-imu", action="store_true",
                        help="publish no IMU (the robot may run without one; only the blind finish needs it)")
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    if args.case == "tag_dropout":
        args.tag_min_range_m = 0.24
    rng = np.random.default_rng(args.seed)
    world = World(args)

    log_dir = Path(os.environ.get("APPROACH_LOOP_LOG_DIR", tempfile.mkdtemp(prefix="approach_loop_")))
    log_dir.mkdir(parents=True, exist_ok=True)
    cfg = log_dir / "phase1_visual_taught.yaml"
    ref = write_config(args, world, cfg)
    subprocess.check_call(["rosparam", "load", str(cfg), "/grasp_hexapod_bt_control"])
    node = subprocess.Popen(
        ["rosrun", "grasp_hexapod_bt_control", "bt_control_node",
         "__name:=grasp_hexapod_bt_control", "_visual_guidance_enabled:=true",
         "_enable_real_climb:=false", "_enable_real_dock:=false", "_approach_fake:=false"],
        stdout=(log_dir / "node.log").open("w"), stderr=subprocess.STDOUT, env=os.environ.copy())
    running = True
    try:
        fake = FakeHardware()
        statuses = []
        rospy.Subscriber("/grasp_hexapod/mode_status", ModeStatus,
                         lambda m: statuses.append((time.time(), m.mode, m.status, m.message)),
                         queue_size=50)
        target_pub = rospy.Publisher("/xiaolan/target", XiaolanTarget, queue_size=1)
        health_pub = rospy.Publisher("/xiaolan/perception_status", PerceptionStatus, queue_size=1)
        tag_pub = rospy.Publisher("/xiaolan/tag_detections", AprilTagDetectionArray, queue_size=1)
        imu_pub = rospy.Publisher("/grasp_hexapod/imu", Imu, queue_size=5)
        switch = rospy.ServiceProxy("/grasp_hexapod/switch_mode", SwitchMode)
        switch.wait_for_service(timeout=20)
        integrate = {"on": False}
        counts = {"target": 0, "tag": 0}

        def physics_loop():
            rate = rospy.Rate(30)
            while running and not rospy.is_shutdown():
                if integrate["on"]:
                    with fake.lock:
                        q = fake.q_des.copy()
                    world.step(q, rospy.Time.now().to_sec())
                rate.sleep()

        def imu_loop():
            rate = rospy.Rate(50)
            while running and not rospy.is_shutdown() and not args.no_imu:
                with world.lock:
                    psi = float(world.pose[2])
                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "imu_link"
                msg.orientation.z, msg.orientation.w = math.sin(psi / 2), math.cos(psi / 2)
                imu_pub.publish(msg)
                rate.sleep()

        def yolo_loop():
            rate = rospy.Rate(15)
            while running and not rospy.is_shutdown():
                sample = world.sample(0.12) if integrate["on"] else None
                now = rospy.Time.now()
                stamp = now
                status = PerceptionStatus(state=PerceptionStatus.NO_TARGET, reason="NO_DETECTION")
                if sample is not None:
                    stamp = rospy.Time.from_sec(sample[0])
                    view = world.target_view(sample[1], sample[2])
                    status.rgb_stamp = status.depth_stamp = stamp
                    if view is not None:
                        p, uv = view
                        msg = XiaolanTarget(detected=True,
                                            x_target=float(p[0] + rng.normal(0, 0.005)),
                                            z_target=float(p[2] + rng.normal(0, 0.008)),
                                            center_u=int(uv[0]), center_v=int(uv[1]), status="TARGET_VALID")
                        msg.header.stamp = stamp
                        msg.header.frame_id = FRAME
                        target_pub.publish(msg)
                        counts["target"] += 1
                        status.state, status.reason = PerceptionStatus.TARGET_VALID, "TARGET_VALID"
                # The node requires header.stamp == min(rgb_stamp, depth_stamp).
                status.rgb_stamp = status.depth_stamp = status.header.stamp = stamp
                status.header.frame_id = FRAME
                health_pub.publish(status)
                rate.sleep()

        def tag_loop():
            rate = rospy.Rate(15)
            while running and not rospy.is_shutdown():
                sample = world.sample(0.07) if integrate["on"] else None
                arr = AprilTagDetectionArray()
                arr.header.stamp = rospy.Time.now()
                arr.header.frame_id = FRAME
                if sample is not None:
                    view = world.tag_view(sample[1], sample[2])
                    arr.header.stamp = rospy.Time.from_sec(sample[0])
                    if view is not None:
                        p, r = view
                        noise = rot_z(rng.normal(0, math.radians(0.3)))
                        r = r @ np.array([[noise[0, 0], 0, noise[0, 1]], [0, 1, 0], [noise[1, 0], 0, noise[1, 1]]])
                        det = AprilTagDetection(id=[args.tag_id], size=[args.tag_size_m])
                        pose = PoseWithCovarianceStamped()
                        pose.header = arr.header
                        pose.pose.pose.position.x = float(p[0] + rng.normal(0, 0.001))
                        pose.pose.pose.position.y = float(p[1] + rng.normal(0, 0.001))
                        pose.pose.pose.position.z = float(p[2] + rng.normal(0, 0.002))
                        qx, qy, qz, qw = quat_from_matrix(r)
                        o = pose.pose.pose.orientation
                        o.x, o.y, o.z, o.w = qx, qy, qz, qw
                        det.pose = pose
                        arr.detections.append(det)
                        counts["tag"] += 1
                tag_pub.publish(arr)
                rate.sleep()

        threads = [threading.Thread(target=f, daemon=True) for f in (physics_loop, imu_loop, yolo_loop, tag_loop)]
        for th in threads:
            th.start()

        deadline = time.time() + 15
        while not any(m == "" and s == "idle" and msg == "phase1 joint feedback ready"
                      for _, m, s, msg in statuses) and time.time() < deadline:
            rospy.sleep(0.05)
        ok, message = call_mode(switch, "home", timeout_s=60)
        assert ok, f"home failed: {message}"
        back, left, dyaw = CASES[args.case]
        r0 = rot_z(P0[2])[:2, :2]
        # P0 body frame: +y forward (toward Xiaolan), +x right.
        start_xy = P0[:2] + r0 @ np.array([-left, -back])
        start = np.array([start_xy[0], start_xy[1], P0[2] + math.radians(dyaw)])
        world.place(start)
        integrate["on"] = True
        rospy.sleep(0.5)
        t0 = time.time()
        results = {}
        for mode, limit in (("spin_search", 90.0), ("approach", args.timeout_s)):
            ok, message = call_mode(switch, mode, timeout_s=limit)
            results[mode] = (ok, message, time.time() - t0)
            print(f"[loop] {mode}: ok={ok} t={time.time() - t0:.1f}s msg={message}")
            if not ok:
                break
        rospy.sleep(1.0)
        integrate["on"] = False
        with world.lock:
            final = world.pose.copy()
        err_w = final[:2] - P0[:2]
        err_body = rot_z(P0[2])[:2, :2].T @ err_w  # in the P0 body frame: x right, y forward
        yaw_err = math.degrees((final[2] - P0[2] + math.pi) % (2 * math.pi) - math.pi)
        print(f"[loop] case={args.case} start=({start[0]:.3f},{start[1]:.3f},{math.degrees(start[2]):.1f}deg) "
              f"taught_ref x={ref[0]:.4f} z={ref[1]:.4f} yaw={math.degrees(ref[2]):.2f}deg")
        print(f"[loop] final error vs P0: right={err_body[0] * 1000:.1f} mm forward={err_body[1] * 1000:.1f} mm "
              f"yaw={yaw_err:.2f} deg; perception msgs target={counts['target']} tag={counts['tag']}")
        events = [(round(t - t0, 2), m, s, msg) for t, m, s, msg in statuses if t >= t0]
        for e in events:
            print("[status]", e)
        if args.out:
            np.savez(args.out, trace=np.array(world.trace), p0=P0, start=start, final=final,
                     face_x=FACE_X, tag=world.tag_w, target=world.target_w)
        passed = (results.get("approach", (False,))[0] is True and abs(err_body[0]) <= 0.015
                  and abs(err_body[1]) <= 0.020 and abs(yaw_err) <= 3.0)
        print(f"[loop] {'PASS' if passed else 'FAIL'} (budget right 15 mm, forward 20 mm, yaw 3 deg)")
        return 0 if passed else 1
    finally:
        running = False
        node.terminate()
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
