#!/usr/bin/env python3
"""Hardware-free controller checks for perception-health gating."""
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rospy
from grasp_hexapod_msgs.msg import ModeStatus
from grasp_hexapod_msgs.srv import SwitchMode
from xiaolan_yolo_detection.msg import PerceptionStatus

from fake_hardware_e2e import FakeHardware, call_mode, record_targets


def wait_until(predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        rospy.sleep(.03)
    raise AssertionError(f"timeout waiting for {description}")


def target_span(fake):
    with fake.lock:
        values = np.asarray(fake.walk_targets, dtype=float)
    if not values.size:
        return 0.0
    return float(np.ptp(values.reshape(values.shape[0], -1), axis=0).max())


def wait_target_convergence(fake, window_size=60, timeout_s=3.0):
    started = time.monotonic()
    while time.monotonic() - started < timeout_s and not rospy.is_shutdown():
        with fake.lock:
            values = np.asarray(fake.walk_targets[-window_size:], dtype=float)
        if values.shape[0] >= window_size:
            span = float(np.ptp(values.reshape(values.shape[0], -1), axis=0).max())
            if span <= 1e-4:
                return time.monotonic() - started, span
        rospy.sleep(.03)
    raise AssertionError(
        f"targets did not converge across trailing {window_size} samples "
        f"within {timeout_s:.1f}s; latest span={target_span(fake):.8g}")


def main():
    log_dir = Path(os.environ.get(
        "PHASE1_INPUT_HEALTH_E2E_LOG_DIR",
        "/tmp/grasp_phase1_input_health_e2e"))
    log_dir.mkdir(parents=True, exist_ok=True)
    rospy.init_node("phase1_input_health_e2e")
    fake = FakeHardware(initialize_node=False)
    node = subprocess.Popen(
        ["rosrun", "grasp_hexapod_bt_control", "bt_control_node",
         "_visual_guidance_enabled:=true", "_enable_real_climb:=false",
         "_enable_real_dock:=false"],
        stdout=(log_dir / "node.log").open("w"), stderr=subprocess.STDOUT,
        env=os.environ.copy())
    statuses = []
    status_lock = threading.Lock()
    rospy.Subscriber(
        "/grasp_hexapod/mode_status", ModeStatus,
        lambda message: _append_status(statuses, status_lock, message),
        queue_size=30)
    health_pub = rospy.Publisher("/xiaolan/perception_status",
                                 PerceptionStatus, queue_size=1)
    switch = rospy.ServiceProxy("/grasp_hexapod/switch_mode", SwitchMode)
    phase = {"value": "silent", "stamp": None}
    phase_lock = threading.Lock()
    publishing = True
    health_thread = None
    mode_thread = None
    try:
        switch.wait_for_service(timeout=15)
        wait_until(
            lambda: any(mode == "" and status == "idle" and
                        message == "phase1 joint feedback ready"
                        for mode, status, message in _status_snapshot(statuses, status_lock)),
            10, "controller-owned joint feedback ready")
        ok, message = call_mode(switch, "home", timeout_s=20)
        assert ok is True, (ok, message)

        # A: No perception packet ever arrives; commands stay at idle hold and
        # the mode exits with the explicit startup-health reason.
        no_health_status_mark = len(_status_snapshot(statuses, status_lock))
        with record_targets(fake):
            started = time.monotonic()
            no_health_ok, no_health_message = call_mode(
                switch, "spin_search", timeout_s=5)
            no_health_elapsed = time.monotonic() - started
            no_health_span = target_span(fake)
        assert no_health_ok is False, (no_health_ok, no_health_message)
        assert "perception startup unavailable" in no_health_message, no_health_message
        assert no_health_elapsed < 4.0, no_health_elapsed
        assert no_health_span <= 1e-4, no_health_span
        wait_until(
            lambda: any(mode == "spin_search" and status == "failed"
                        for mode, status, _ in _status_snapshot(
                            statuses, status_lock)[no_health_status_mark:]),
            2, "post-request spin_search failed ModeStatus")

        def health_loop():
            rate = rospy.Rate(15)
            while publishing and not rospy.is_shutdown():
                with phase_lock:
                    current, frozen = phase["value"], phase["stamp"]
                if current != "silent":
                    source = rospy.Time.now() if current == "fresh" else frozen
                    if source is not None:
                        message = PerceptionStatus(
                            state=PerceptionStatus.NO_TARGET,
                            reason="NO_DETECTION")
                        message.header.stamp = source
                        message.rgb_stamp = source
                        message.depth_stamp = source
                        message.header.frame_id = "camera_color_optical_frame"
                        health_pub.publish(message)
                        if current == "fresh":
                            with phase_lock:
                                phase["stamp"] = source
                rate.sleep()

        health_thread = threading.Thread(target=health_loop, daemon=True)
        with phase_lock:
            phase["value"] = "fresh"
        health_thread.start()
        wait_until(lambda: _phase_stamp(phase, phase_lock) is not None,
                   3, "first fresh synthetic NO_TARGET health packet")

        # B: Healthy NO_TARGET permits search. Repeating an old source stamp
        # must not renew health freshness or the failure deadline.
        with record_targets(fake):
            mode_result = {}
            healthy_search_status_mark = len(
                _status_snapshot(statuses, status_lock))

            def run_search():
                mode_result["success"], mode_result["message"] = call_mode(
                    switch, "spin_search", timeout_s=5)

            mode_thread = threading.Thread(target=run_search, daemon=True)
            mode_thread.start()
            wait_until(
                lambda: any(mode == "spin_search" and status == "running"
                            for mode, status, _ in _status_snapshot(
                                statuses, status_lock)[healthy_search_status_mark:]),
                5, "healthy NO_TARGET spin_search")
            rospy.sleep(.8)
            healthy_search_span = target_span(fake)
            frozen_started = time.monotonic()
            with phase_lock:
                phase["stamp"] = rospy.Time.now()
                phase["value"] = "frozen"
                frozen_source = phase["stamp"]
            mode_thread.join(timeout=4)
            assert not mode_thread.is_alive(), "frozen-source health did not fail search"
            frozen_elapsed = time.monotonic() - frozen_started
            frozen_ok = mode_result.get("success")
            frozen_message = mode_result.get("message", "")
        assert healthy_search_span > 1e-5, healthy_search_span
        assert frozen_ok is False, (frozen_ok, frozen_message)
        assert "perception stale/unusable" in frozen_message, frozen_message
        wait_until(
            lambda: any(mode == "spin_search" and status == "failed"
                        for mode, status, _ in _status_snapshot(
                            statuses, status_lock)[healthy_search_status_mark:]),
            2, "post-request frozen-health failed ModeStatus")

        # C: Restoring health is not an implicit new command. An explicit
        # request starts a new run, which home then preempts cleanly.
        with record_targets(fake):
            settle_wait_s, frozen_settled_span = wait_target_convergence(
                fake, window_size=60, timeout_s=3.0)
        # The failed status has already been observed. Mark after it so only
        # events caused by restoration can satisfy the no-autostart check.
        restore_status_mark = len(_status_snapshot(statuses, status_lock))
        with record_targets(fake):
            with phase_lock:
                phase["value"] = "fresh"
            rospy.sleep(.7)
            restored_without_request_span = target_span(fake)
        assert restored_without_request_span <= 1e-4, restored_without_request_span
        assert not any(mode and status == "running"
                       for mode, status, _ in _status_snapshot(
                           statuses, status_lock)[restore_status_mark:]), \
            _status_snapshot(statuses, status_lock)[restore_status_mark:]

        with record_targets(fake):
            resumed = {}
            resumed_status_mark = len(_status_snapshot(statuses, status_lock))

            def run_resumed_search():
                resumed["success"], resumed["message"] = call_mode(
                    switch, "spin_search", timeout_s=10)

            mode_thread = threading.Thread(target=run_resumed_search, daemon=True)
            mode_thread.start()
            wait_until(
                lambda: any(mode == "spin_search" and status == "running"
                            for mode, status, _ in _status_snapshot(
                                statuses, status_lock)[resumed_status_mark:]),
                5, "explicit post-restore spin_search")
            rospy.sleep(.7)
            resumed_span = target_span(fake)
            home_ok, home_message = call_mode(switch, "home", timeout_s=20)
            mode_thread.join(timeout=5)
            assert not mode_thread.is_alive(), "home did not terminate explicit search"
        assert resumed_span > 1e-5, resumed_span
        assert home_ok is True, (home_ok, home_message)
        assert resumed.get("success") is False, resumed
        assert "preempted by home" in resumed.get("message", ""), resumed
        print(
            f"no_health_s={no_health_elapsed:.3f} no_health_span={no_health_span:.8g} "
            f"healthy_search_span={healthy_search_span:.8g} frozen_source={frozen_source.to_sec():.6f} "
            f"frozen_elapsed_s={frozen_elapsed:.3f} frozen_failure={frozen_message} "
            f"settle_wait_s={settle_wait_s:.3f} baseline_span={frozen_settled_span:.8g} "
            f"restored_without_request_span={restored_without_request_span:.8g} "
            f"explicit_search_span={resumed_span:.8g} home={home_message}")
        return 0
    finally:
        publishing = False
        if health_thread is not None:
            health_thread.join(timeout=2)
        if mode_thread is not None and mode_thread.is_alive():
            mode_thread.join(timeout=1)
        node.send_signal(signal.SIGINT)
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()
            node.wait(timeout=5)


def _append_status(statuses, lock, message):
    with lock:
        statuses.append((message.mode, message.status, message.message))


def _status_snapshot(statuses, lock):
    with lock:
        return list(statuses)


def _phase_stamp(phase, lock):
    with lock:
        return phase["stamp"]


if __name__ == "__main__":
    raise SystemExit(main())
