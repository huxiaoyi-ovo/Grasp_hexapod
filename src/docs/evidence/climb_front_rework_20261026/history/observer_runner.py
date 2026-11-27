#!/usr/bin/env python3
"""Observation-only wrapper for one frozen visible Isaac preview."""
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path('/home/artrc/Grasp_hexapod')
SCRIPTS = ROOT / 'src/grasp_hexapod_control/scripts'
sys.path.insert(0, str(SCRIPTS))
import run_sim  # Isaac Gym is intentionally imported by run_sim before any torch import.

OUT = Path('/tmp/grasp_front_v4_gpu_20261026T123301Z/actual_samples.jsonl')
original = run_sim._update_climb_metric

def observed(metric, controller, q_current, root_position, root_quaternion_xyzw, dt):
    result = original(metric, controller, q_current, root_position, root_quaternion_xyzw, dt)
    mode = controller.climb_mode
    anchors = None
    if mode is not None and mode.state == mode.RUNNING:
        try:
            _, anchors, _ = mode._stage_reference()
            anchors = np.asarray(anchors, dtype=float).tolist()
        except ValueError:
            anchors = None
    row = {
        'stage_index': None if mode is None else mode.stage_index,
        'stage_name': None if mode is None else mode.phase,
        'phase_time_s': None if mode is None else float(mode.phase_time),
        'actual_q_rad': np.asarray(q_current, dtype=float).reshape(6, 3).tolist(),
        'actual_root_xyz': np.asarray(root_position, dtype=float).tolist(),
        'actual_root_quaternion_xyzw': np.asarray(root_quaternion_xyzw, dtype=float).tolist(),
        'planned_base_pose': None if mode is None else np.asarray(mode.base_pose, dtype=float).tolist(),
        'planned_anchors': anchors,
    }
    with OUT.open('a', encoding='utf-8') as file:
        file.write(json.dumps(row, allow_nan=False) + '\n')
    return result

run_sim._update_climb_metric = observed
sys.argv = ['run_sim.py', '--climb-start', '--climb-config', str(ROOT / 'src/grasp_hexapod_control/config/climb_front_optimized.json'), '--climb-speed', '1', '--climb-joint-speed', '1.8', '--climb-metrics', '/tmp/grasp_front_v4_gpu_20261026T123301Z/metrics.json']
run_sim.main()
