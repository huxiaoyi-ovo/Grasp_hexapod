#!/usr/bin/env python3
"""Four-pair terminal knee STL comparison for revised AH and old FINAL_HOLD."""
import importlib.util
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / 'src/grasp_hexapod_control/scripts'
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / 'tools')]

EVIDENCE = Path(__file__).resolve().parent
OLD_CONFIG = EVIDENCE / 'before_climb_front_fast.json'
CURRENT_CONFIG = ROOT / 'src/grasp_hexapod_control/config/climb_front_fast.json'
OLD_CPU = ROOT / 'src/docs/evidence/climb_front_fast_20260923/cpu_fast_41_audit.json'
CURRENT_LOCAL = EVIDENCE / 'rejected_x205_apex10_local.json'
OUT = EVIDENCE / 'terminal_knee_diagnosis.json'
HELPER = ROOT / 'src/docs/evidence/climb_front_fast_20260923/front_back_spacing_visual_mesh_audit.py'

spec = importlib.util.spec_from_file_location('terminal_mesh_audit', HELPER)
mesh_audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mesh_audit)
mesh_audit.EVIDENCE_DIR = EVIDENCE
mesh_audit.PAIR_CACHE_PATH = EVIDENCE / 'terminal_knee_diagnosis_pair_cache.json'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit_pose(label, config_path, q, pose, q_source):
    config = json.loads(Path(config_path).read_text())
    items = []
    for left, right in ((('lb', 'knee'), ('lm', 'knee')),
                        (('rb', 'knee'), ('rm', 'knee'))):
        left_pair = {'leg': left[0], 'link': left[1]}
        right_pair = {'leg': right[0], 'link': right[1]}
        result = mesh_audit.check_pair(
            mesh_audit.scene, q, pose, left_pair, right_pair)
        items.append({
            'pair': [left_pair, right_pair],
            'mesh_collision': result['mesh_collision'],
            'triangle_hit': result['triangle_hit'],
            'triangle_test_count': result['triangle_test_count'],
            'broad_phase_candidates': result['broad_phase_candidates'],
            'pair_status': result['pair_status'],
            'components_nonempty': result['components_nonempty'],
            'cache_reused': result['cache_reused'],
        })
    return {
        'label': label,
        'config_sha256': sha256(config_path),
        'config_stage_count': config['stage_count'],
        'q_source': q_source,
        'q_rad': np.asarray(q, dtype=float).tolist(),
        'base_pose': np.asarray(pose, dtype=float).tolist(),
        'queries': items,
    }


def main():
    old_cfg = json.loads(OLD_CONFIG.read_text())
    old_cpu = json.loads(OLD_CPU.read_text())
    current_cfg = json.loads(CURRENT_CONFIG.read_text())
    current_local = json.loads(CURRENT_LOCAL.read_text())

    current_stage = next(s for s in current_cfg['stages']
                         if s['name'] == 'AH_BACK_LOW_FIRST_SEGMENT')
    current_q = np.asarray(current_local['last_q_rad'], dtype=float)
    current_pose = np.asarray(current_stage['pose_end'], dtype=float)
    current = audit_pose(
        'revised_x205_AH_DONE', CURRENT_CONFIG, current_q, current_pose,
        'rejected_x205_apex10_local.json last_q_rad after state DONE; pose from candidate AH pose_end',
    )

    old_hold = next(s for s in old_cfg['stages'] if s['name'] == 'FINAL_HOLD')
    old_q = np.asarray(old_cpu['boundaries']['FINAL_HOLD']['exit_q_rad'], dtype=float)
    old_pose = np.asarray(old_hold['pose_end'], dtype=float)
    old = audit_pose(
        'original_d15bf_FINAL_HOLD_EXIT', OLD_CONFIG, old_q, old_pose,
        'cpu_fast_41_audit.json boundaries.FINAL_HOLD.exit_q_rad; pose from before_climb_front_fast.json FINAL_HOLD pose_end',
    )
    out = {
        'current_candidate_config_sha256': sha256(CURRENT_CONFIG),
        'historical_source_config_sha256': sha256(OLD_CONFIG),
        'historical_cpu_audit_sha256': sha256(OLD_CPU),
        'visual_mesh_sha256': mesh_audit.visual_mesh_hashes(),
        'selected_terminal_states': [current, old],
        'boundary': 'Four selected exact triangle-intersection queries only. No continuous route, containment, contact, load, stability or hardware proof.',
    }
    OUT.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    mesh_audit.scene = mesh_audit.default_visual_scene(ROOT)
    main()
