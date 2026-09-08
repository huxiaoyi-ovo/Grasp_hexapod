import copy
import json
from pathlib import Path
import sys
import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(SCRIPTS / 'tools'))
from climb_mode import ClimbMode
from control import GraspController
from kinematics import Q_STAND
from rebuild_climb_tail import build, continuous_swings


def curve(velocities=None, t=1.0):
    mode = ClimbMode(None); mode.phase_time = t
    knots = np.array([[[0., 0., 0.]], [[1., 0., 0.]], [[1., 1., 0.]]])
    return mode._piecewise(knots, [1., 1.], velocities)


def velocity(velocities, t, h=1.e-5):
    return (curve(velocities, t + h) - curve(velocities, t - h)) / (2*h)


def acceleration(velocities, t, h=2.e-4):
    return (curve(velocities, t + h) - 2*curve(velocities, t) + curve(velocities, t-h)) / h**2


def test_hermite_knot_velocity_and_zero_acceleration():
    v = np.array([[[0.,0.,0.]], [[.2,.2,0.]], [[0.,0.,0.]]])
    assert np.allclose(velocity(v, 1.), v[1], atol=2.e-5)
    assert np.linalg.norm(acceleration(v, 1.)) < 5.e-3
    assert np.allclose(curve(v, 0.), [[[0.,0.,0.]]])
    assert np.allclose(curve(v, 2.), [[[1.,1.,0.]]])
    assert np.linalg.norm(velocity(v, 0.)) < 2.e-5
    assert np.linalg.norm(velocity(v, 2.)) < 2.e-5


def test_absent_velocity_preserves_smoothstep():
    expected = curve(None, .5)
    assert np.allclose(expected, [[[.5,0.,0.]]])


def test_velocity_shape_and_nan_rejected():
    cfg = json.loads((SCRIPTS.parent / 'config/climb_compact.json').read_text())
    stage = next(s for s in cfg['stages'] if s['anchor_curve'] == 'piecewise_base_quintic' and len(s['segment_durations_s']) > 1)
    bad = copy.deepcopy(cfg); target = bad['stages'][cfg['stages'].index(stage)]
    target['active_base_velocities_m_s'] = [[[0., 0., 0.]]]
    with pytest.raises(ValueError): ClimbMode(None)._validate_config(bad)
    bad = copy.deepcopy(cfg); target = bad['stages'][cfg['stages'].index(stage)]
    target['active_base_velocities_m_s'] = np.full_like(np.asarray(stage['active_base_knots_m']), np.nan).tolist()
    with pytest.raises(ValueError): ClimbMode(None)._validate_config(bad)


def test_internal_hermite_checkpoint_holds_on_feedback_mismatch():
    config = json.loads((SCRIPTS.parent / 'config/climb_compact.json').read_text())
    stage = config['stages'][1]
    stage['active_base_velocities_m_s'] = np.zeros_like(
        np.asarray(stage['active_base_knots_m'], dtype=np.float64)
    ).tolist()
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, config, hardware_execution=True)
    mode = controller.climb_mode
    mode.stage_index = 1
    mode.phase = mode.stage_names[1]
    mode.phase_time = stage['segment_durations_s'][0]
    mode.stage_elapsed_time = mode.phase_time
    pose, anchors, _ = mode._stage_reference()
    mode._apply_reference(pose, anchors, sync_previous=True)
    reference_before_hold = controller.foot_desired_base.copy()
    controller.update(Q_STAND + .5, np.zeros(4))
    assert mode.last_phase_hold
    assert mode.phase_time == stage['segment_durations_s'][0]
    assert np.array_equal(controller.foot_desired_base, reference_before_hold)


def continuous_config():
    source = json.loads((ROOT / 'src/docs/evidence/climb_flow_20260908/baseline_config.json').read_text())
    return continuous_swings(build(source))


def test_continuous_air_marker_rejects_non_lm_or_landing_boundary():
    config = continuous_config()
    ClimbMode(None)._validate_config(config)
    bad = copy.deepcopy(config)
    bad['stages'][23]['active_legs'] = [0]
    with pytest.raises(ValueError):
        ClimbMode(None)._validate_config(bad)
    bad = copy.deepcopy(config)
    bad['stages'][25]['continuous_air_transition'] = True
    with pytest.raises(ValueError):
        ClimbMode(None)._validate_config(bad)
    bad = copy.deepcopy(config)
    bad['stages'][23]['continuous_air_transition'] = 'yes'
    with pytest.raises(ValueError):
        ClimbMode(None)._validate_config(bad)


def test_continuous_air_transition_skips_duplicate_endpoint_but_mismatch_holds():
    config = continuous_config()
    controller = GraspController(1.0 / 30.0)
    controller.enter_climb(Q_STAND, config, hardware_execution=True)
    mode = controller.climb_mode
    c24 = config['stages'][23]
    mode.stage_index = 23
    mode.phase = mode.stage_names[23]
    mode.phase_time = sum(c24['segment_durations_s'])
    mode.stage_elapsed_time = mode.phase_time
    pose, anchors, _ = mode._stage_reference()
    mode._apply_reference(pose, anchors, sync_previous=True)
    mode.update(np.zeros(4), Q_STAND + .5)
    assert mode.last_phase_hold
    assert mode.stage_index == 23
    assert mode.phase_time == sum(c24['segment_durations_s'])

    mode._update_tracking_diagnostics = lambda unused: setattr(mode, 'last_settled', True)
    mode.settle_time = mode._effective_settle_required(c24) - controller.dt
    mode.update(np.zeros(4), Q_STAND)
    assert mode.stage_index == 24
    assert mode.phase_time == controller.dt
    c25_start = np.asarray(config['stages'][24]['active_base_knots_m'][0][0])
    assert not np.allclose(controller.foot_desired_base[2], c25_start)
