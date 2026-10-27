#!/usr/bin/env python3
"""小蓝左右直线接近决策与导航失效落脚测试。"""

from pathlib import Path
import json
import sys

import numpy as np


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from approach_mode import ApproachPlan
from control import GraspController, MissionStateMachine
from kinematics import Q_STAND
from utils import NavigationState
from utils.climb import derive_compact_approach_geometry


BOUNDARY = np.array(
    [[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]],
    dtype=np.float64,
)
XIAOLAN_KEEP_OUT = np.array(
    [[-0.355, -0.373], [0.355, -0.373], [0.355, 0.307], [-0.355, 0.307]],
    dtype=np.float64,
)


def _navigation(base_xy, valid=True):
    pv_from_base = np.eye(4, dtype=np.float64)
    pv_from_base[:2, 3] = base_xy
    return NavigationState(
        stamp=1.0,
        valid=valid,
        landing_confirmed=True,
        pv_from_base=pv_from_base,
        pv_from_xiaolan=np.eye(4, dtype=np.float64),
        pv_boundary=BOUNDARY,
    )


def _configured_controller():
    controller = GraspController(1.0 / 30.0)
    with (SCRIPTS.parent / "config" / "climb_compact.json").open() as file:
        compact = json.load(file)
    geometry = derive_compact_approach_geometry(compact)
    for side in ("left", "right"):
        controller.approach_mode.configure_fixed_approach(
            geometry["targets"][side],
            target_side=side,
            xiaolan_keepout_polygon_xy_m=geometry[
                "xiaolan_keepout_polygon_xy_m"
            ],
            xiaolan_body_clearance_m=0.13,
        )
    return controller


def test_approach_geometry_tracks_current_compact_p0_and_climb_mesh():
    with (SCRIPTS.parent / "config" / "climb_compact.json").open() as file:
        compact = json.load(file)

    geometry = derive_compact_approach_geometry(compact)

    expected_left = np.asarray(compact["p0"]["base"][:3]) - np.asarray(
        compact["xiaolan_translation"]
    )
    assert np.allclose(geometry["targets"]["left"][:3, 3], expected_left)
    assert np.isclose(geometry["targets"]["right"][0, 3], -expected_left[0])
    assert np.allclose(
        geometry["targets"]["right"][1:3, 3], expected_left[1:3]
    )
    assert np.allclose(
        geometry["xiaolan_keepout_polygon_xy_m"], XIAOLAN_KEEP_OUT,
        atol=2e-8,
    )


def test_approach_selects_nearest_direct_left_or_right_target():
    controller = _configured_controller()

    left = controller.start_autonomous_approach(_navigation([-0.9, 0.0]))
    assert not left.failed
    assert left.target_side == "left"
    assert left.waypoints_pv.shape == (1, 2)

    controller.approach_mode.cancel_autonomous_approach()
    right = controller.start_autonomous_approach(_navigation([0.9, 0.0]))
    assert not right.failed
    assert right.target_side == "right"
    assert right.waypoints_pv.shape == (1, 2)


def test_requested_opposite_side_is_rejected_instead_of_planning_detour():
    controller = _configured_controller()

    result = controller.start_autonomous_approach(
        _navigation([0.9, 0.0]),
        target_side="left",
    )

    assert result.failed
    assert "no direct left" in result.reason


def test_known_panel_boundary_can_reject_both_fixed_targets():
    controller = _configured_controller()
    navigation = _navigation([0.0, -0.6])
    navigation.pv_boundary = np.array(
        [[-0.75, -1.0], [0.75, -1.0], [0.75, 1.0], [-0.75, 1.0]]
    )

    result = controller.start_autonomous_approach(navigation)

    assert result.failed
    assert "panel/Xiaolan constraints" in result.reason


def test_navigation_failure_finishes_current_swing_before_failed_state():
    controller = GraspController(1.0 / 30.0)
    mode = controller.approach_mode
    q_cur = controller.update(
        Q_STAND.copy(), np.array([0.12, 0.0, 0.0, 0.0])
    )
    assert mode.gait_started
    mode.approach_plan = ApproachPlan(active=True, state="translate")
    mode.navigation_state = _navigation([0.0, 0.0])
    controller.mission.state = MissionStateMachine.APPROACH

    controller.mission.update(q_cur, _navigation([0.0, 0.0], valid=False))
    assert controller.mission.state == MissionStateMachine.FAIL_LANDING

    for _ in range(60):
        q_cur = controller.update(q_cur, np.zeros(4), None)
        if controller.mission.state == MissionStateMachine.FAILED:
            break

    assert controller.mission.state == MissionStateMachine.FAILED
    assert controller.mission.reason == "navigation data became invalid"
    assert not mode.gait_started
    assert not mode.transfer_active
