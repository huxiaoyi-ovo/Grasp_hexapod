#!/usr/bin/env python3
"""Resolve capsule guard pairs at the first revised AH warning using selected STL pairs."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / 'src/grasp_hexapod_control/scripts'
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / 'tools')]
from control import (BODY_COLLISION_RADIUS, BODY_COLLISION_Z_MAX,
                     BODY_COLLISION_Z_MIN, COLLISION_MARGIN,
                     LINK_COLLISION_RADII, GraspController)

EVIDENCE = Path(__file__).resolve().parent
REPLAY = EVIDENCE / 'local_ah_replay.json'
OUT = EVIDENCE / 'ah_knee_window_x205_mesh_audit.json'
HELPER = ROOT / 'src/docs/evidence/climb_front_fast_20260923/front_back_spacing_visual_mesh_audit.py'
NAMES = ('lb', 'lf', 'lm', 'rb', 'rf', 'rm')
LINKS = ('thigh', 'knee', 'ankle')

spec = importlib.util.spec_from_file_location('old_visual_mesh_audit', HELPER)
mesh_audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mesh_audit)
mesh_audit.EVIDENCE_DIR = EVIDENCE
mesh_audit.PAIR_CACHE_PATH = EVIDENCE / 'ah_first_capsule_mesh_pair_cache.json'


def main():
    replay = json.loads(REPLAY.read_text())
    samples = replay['known_interference_window_samples']
    if not samples:
        raise RuntimeError('local replay has no AH phase >= 1.5 s samples')
    records = []
    for sample in samples:
        q = np.asarray(sample['q_rad'], dtype=float)
        pose = np.asarray(sample['pose'], dtype=float)
        for left, right in (({'leg': 'lb', 'link': 'knee'}, {'leg': 'lm', 'link': 'knee'}),
                            ({'leg': 'rb', 'link': 'knee'}, {'leg': 'rm', 'link': 'knee'})):
            result = mesh_audit.check_pair(mesh_audit.scene, q, pose, left, right)
            records.append({
                'tick': sample['tick'],
                'phase_time_s': sample['phase_time_s'],
                'q_rad': sample['q_rad'],
                'pose': sample['pose'],
                'pair': [left, right],
                'mesh_collision': result['mesh_collision'],
                'triangle_hit': result['triangle_hit'],
                'triangle_test_count': result['triangle_test_count'],
                'broad_phase_candidates': result['broad_phase_candidates'],
                'pair_status': result['pair_status'],
                'components_nonempty': result['components_nonempty'],
                'cache_reused': result['cache_reused'],
            })
            if result['mesh_collision']:
                receipt = {
                    'candidate_config_sha256': replay['candidate_config_sha256'],
                    'source_cpu_audit_sha256': replay['source_cpu_audit_sha256'],
                    'stage': replay['stage'],
                    'window_start_phase_s': replay['known_interference_window_start_s'],
                    'pair_query_count': len(records),
                    'samples': records,
                    'visual_mesh_sha256': mesh_audit.visual_mesh_hashes(),
                    'conclusion': 'real triangle intersection confirmed; stopped at first colliding sampled pair',
                    'evidence_boundary': 'Only LB-LM and RB-RM knee visual STL triangle checks at the discrete 30 Hz AH window samples; not continuous clearance or hardware evidence.',
                }
                OUT.write_text(json.dumps(receipt, indent=2) + '\n')
                print(json.dumps({'collision': True, 'tick': sample['tick'],
                                  'phase_time_s': sample['phase_time_s'],
                                  'pair': [left, right], 'triangle_hit': result['triangle_hit'],
                                  'pair_query_count': len(records), 'output': str(OUT)}, indent=2))
                return 4
    receipt = {
        'candidate_config_sha256': replay['candidate_config_sha256'],
        'source_cpu_audit_sha256': replay['source_cpu_audit_sha256'],
        'stage': replay['stage'],
        'window_start_phase_s': replay['known_interference_window_start_s'],
        'pair_query_count': len(records),
        'samples': records,
        'visual_mesh_sha256': mesh_audit.visual_mesh_hashes(),
        'all_sampled_pairs_clear': all(not item['mesh_collision'] for item in records),
        'evidence_boundary': 'Only LB-LM and RB-RM knee visual STL triangle checks at the discrete 30 Hz AH window samples; not continuous clearance or hardware evidence.',
    }
    OUT.write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps({'collision': False, 'pair_query_count': len(records),
                      'samples': len(samples), 'output': str(OUT)}, indent=2))
    return 0


if __name__ == '__main__':
    mesh_audit.scene = mesh_audit.default_visual_scene(ROOT)
    raise SystemExit(main())
