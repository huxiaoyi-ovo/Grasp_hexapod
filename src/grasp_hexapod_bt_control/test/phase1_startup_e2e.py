#!/usr/bin/env python3
"""Isolated Phase-1 sequence preflight and missing-feedback lifecycle checks."""
import os
import subprocess
import sys
import time
from pathlib import Path

import rospy
from grasp_hexapod_msgs.msg import ModeStatus

sys.path.insert(0, str(Path(__file__).parent))
from fake_hardware_e2e import FakeHardware, LEG_NAMES  # noqa: E402


def wait_until(predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return
        rospy.sleep(.03)
    raise AssertionError(f"timeout waiting for {description}")


def start_node(log_dir, visual):
    command = ["rosrun", "grasp_hexapod_bt_control", "bt_control_node"]
    if visual:
        command.extend([
            "_visual_guidance_enabled:=true", "_enable_real_climb:=false",
            "_enable_real_dock:=false"])
    return subprocess.Popen(command, stdout=(log_dir / "node.log").open("w"),
                            stderr=subprocess.STDOUT, env=os.environ.copy())


def run_sequence(log_dir):
    return subprocess.Popen(
        ["rosrun", "grasp_hexapod_bt_control", "phase1_visual_sequence"],
        stdout=(log_dir / "sequence.log").open("w"),
        stderr=subprocess.STDOUT, env=os.environ.copy())


def stop_node(node):
    node.terminate()
    try:
        node.wait(timeout=5)
    except subprocess.TimeoutExpired:
        node.kill()
        node.wait(timeout=5)
    rospy.sleep(.3)


def main():
    log_root = Path(os.environ.get("PHASE1_STARTUP_E2E_LOG_DIR",
                                   "/tmp/grasp_phase1_startup_e2e"))
    log_root.mkdir(parents=True, exist_ok=True)
    rospy.init_node("phase1_startup_e2e")
    fake = FakeHardware(initialize_node=False)
    statuses = []
    rospy.Subscriber("/grasp_hexapod/mode_status", ModeStatus,
                     lambda message: statuses.append(
                         (message.mode, message.status, message.message)),
                     queue_size=50)

    with fake.lock:
        fake.feedback_enabled = True
        fake.legs_seen.clear()
        legacy_gripper_calls = len(fake.gripper_calls)
    legacy_log = log_root / "legacy"
    legacy_log.mkdir(exist_ok=True)
    legacy = start_node(legacy_log, visual=False)
    try:
        rospy.wait_for_service("/grasp_hexapod/switch_mode", timeout=10)
        sequence = run_sequence(legacy_log)
        assert sequence.wait(timeout=5) == 3, "legacy preflight exit"
        with fake.lock:
            assert len(fake.gripper_calls) == legacy_gripper_calls
            assert not fake.legs_seen
    finally:
        stop_node(legacy)

    with fake.lock:
        fake.feedback_enabled = False
        fake.legs_seen.clear()
        no_feedback_gripper_calls = len(fake.gripper_calls)
    rospy.sleep(.4)
    no_feedback_log = log_root / "no_feedback"
    no_feedback_log.mkdir(exist_ok=True)
    no_feedback = start_node(no_feedback_log, visual=True)
    try:
        rospy.wait_for_service("/grasp_hexapod/switch_mode", timeout=10)
        started = time.monotonic()
        sequence = run_sequence(no_feedback_log)
        assert sequence.wait(timeout=3) == 1, "missing-feedback sequence exit"
        no_feedback_elapsed = time.monotonic() - started
        assert no_feedback_elapsed < 2.0, no_feedback_elapsed
        with fake.lock:
            fake.feedback_enabled = True
        rospy.sleep(.6)
        with fake.lock:
            assert len(fake.gripper_calls) == no_feedback_gripper_calls
            assert not fake.legs_seen
        assert not any(mode == "spin_search" and status == "running"
                       for mode, status, _ in statuses)
    finally:
        stop_node(no_feedback)

    with fake.lock:
        fake.feedback_enabled = True
        fake.legs_seen.clear()
    active_log = log_root / "feedback_lost_during_home"
    active_log.mkdir(exist_ok=True)
    active = start_node(active_log, visual=True)
    try:
        rospy.wait_for_service("/grasp_hexapod/switch_mode", timeout=10)
        spin_status_start = len(statuses)
        wait_until(
            lambda: any(mode == "" and status == "idle" and
                        message == "phase1 joint feedback ready"
                        for mode, status, message in statuses[spin_status_start:]),
            10, "controller-owned phase1 feedback ready status")
        sequence = run_sequence(active_log)
        wait_until(lambda: set(LEG_NAMES) <= fake.legs_seen, 10,
                   "first home joint targets")
        with fake.lock:
            fake.feedback_enabled = False
        loss_started = time.monotonic()
        assert sequence.wait(timeout=3) == 1, "home feedback-loss sequence exit"
        home_loss_elapsed = time.monotonic() - loss_started
        assert home_loss_elapsed < 2.0, home_loss_elapsed
        with fake.lock:
            fake.feedback_enabled = True
        rospy.sleep(1.0)
        assert not any(mode == "spin_search" and status == "running"
                       for mode, status, _ in statuses[spin_status_start:])
    finally:
        stop_node(active)

    print(
        f"no_feedback_s={no_feedback_elapsed:.3f} "
        f"home_feedback_loss_s={home_loss_elapsed:.3f} "
        f"statuses={len(statuses)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
