#!/usr/bin/env python3
"""Focused contracts for the bounded C20--C22 LM level-and-lift candidate."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import types

import numpy as np

PACKAGE = Path(__file__).parents[1]
SCRIPTS = PACKAGE / 'scripts'
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / 'tools')]
from climb_mode import ClimbMode
from kinematics import GraspKinematic
from validate_climb_preview import strict_contract

TOOL = SCRIPTS / 'tools' / 'rebuild_climb_lm_level.py'
SPEC = importlib.util.spec_from_file_location('rebuild_climb_lm_level', TOOL)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
BASE = PACKAGE.parents[0] / 'docs/evidence/climb_concurrency_20260908/candidate_config.json'


def candidate():
    base = json.loads(BASE.read_text())
    assert hashlib.sha256(BASE.read_bytes()).hexdigest() == BUILDER.BASE_SHA256
    item = BUILDER.build_v2(base)
    ClimbMode(None)._validate_config(item)
    return base, item


def by_name(config, name):
    return next(stage for stage in config['stages'] if stage['name'] == name)


def test_lm_level_preload_and_first_lift_hold_height():
    _, item = candidate()
    preload, lift, air = (by_name(item, name) for name in (
        'BODY_PRELOAD_LM', 'LM_LIFT', 'BODY_ADVANCE_LM_AIR'))
    assert preload['segment_durations_s'] == [.5]
    assert np.allclose(preload['pose_start'][2:4], [.226, .16])
    assert np.allclose(preload['pose_end'][2:4], [.226, .16])
    assert np.array_equal(preload['anchor_knots'][0], preload['anchor_knots'][-1])
    assert lift['pose_curve'] == 'quintic_first_segment'
    assert lift['segment_durations_s'] == [1.2, .45]
    assert np.allclose(lift['pose_start'], preload['pose_end'])
    assert np.allclose(lift['pose_end'][2:4], [.226, 0.])
    assert np.allclose(air['pose_start'], lift['pose_end'])


def test_lm_level_preserves_boundary_supports_terminal_and_fk():
    base, item = candidate()
    names = [stage['name'] for stage in item['stages']]
    c23 = names.index('LM_LEFT_FINAL_LAND')
    assert item['stages'][c23:] == base['stages'][c23:]
    assert item['terminal_q_rad'] == base['terminal_q_rad']
    for previous, following in zip(item['stages'], item['stages'][1:]):
        assert np.allclose(previous['pose_end'], following['pose_start'], rtol=0, atol=1e-12)
        assert np.allclose(previous['anchor_knots'][-1], following['anchor_knots'][0], rtol=0, atol=1e-12)
    terminal = np.asarray(item['terminal_q_rad'])
    final = item['stages'][-1]
    inverse = np.linalg.inv(ClimbMode._world_from_base(final['pose_end']))
    expected = (np.c_[final['anchor_knots'][-1], np.ones(6)] @ inverse.T)[:, :3]
    assert np.max(np.abs(GraspKinematic().forward_base(terminal) - expected)) <= 1e-5


def test_lm_level_continuous_air_velocity_and_strict_contract():
    _, item = candidate()
    lift, air = (by_name(item, name) for name in ('LM_LIFT', 'BODY_ADVANCE_LM_AIR'))
    rotation_end = ClimbMode._world_from_base(lift['pose_end'])[:3, :3]
    rotation_start = ClimbMode._world_from_base(air['pose_start'])[:3, :3]
    end = rotation_end @ np.asarray(lift['active_base_velocities_m_s'])[-1, 0]
    start = rotation_start @ np.asarray(air['active_base_velocities_m_s'])[0, 0]
    assert np.linalg.norm(end) > 1e-9
    assert np.allclose(end, start, rtol=0, atol=1e-9)
    strict_contract(item)


def test_dock_deterministic_sequence_is_bounded_cpu_diagnostic():
    _, item = candidate()
    rospy = types.ModuleType('rospy')
    tf2_ros = types.ModuleType('tf2_ros')
    tf = types.ModuleType('tf')
    transformations = types.ModuleType('tf.transformations')
    transformations.quaternion_matrix = lambda _: np.eye(4)
    tf.transformations = transformations
    previous = {name: sys.modules.get(name) for name in ('rospy', 'tf2_ros', 'tf', 'tf.transformations', 'dock_mode')}
    try:
        sys.modules.update({'rospy': rospy, 'tf2_ros': tf2_ros, 'tf': tf,
                            'tf.transformations': transformations})
        sys.modules.pop('dock_mode', None)
        defaults = BUILDER.dock_mode_defaults()
        report = BUILDER.dock_continuous_report(item)
    finally:
        for name, module in previous.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module
    assert np.isclose(report['body_raise_height_m'], defaults.BODY_RAISE_HEIGHT_M)
    assert np.isclose(report['search_radius_m'], defaults.TAG_SEARCH_RADIUS_M)
    assert np.isclose(report['search_speed_m_s'], defaults.TAG_SEARCH_SPEED_M_S)
    assert report['sample_count'] > 0
    assert report['min_joint_margin_rad'] > .02
    assert report['min_true_com_six_foot_support_m'] > .01
    assert report['max_ik_residual_m'] <= 1e-5
    assert 'not a support-surface' in report['evidence_boundary']
