import hashlib
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path('/home/artrc/Grasp_hexapod')
SCRIPTS = ROOT / 'src/grasp_hexapod_control/scripts'
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / 'tools')]
from climb_mode import ClimbMode
from control import (BODY_COLLISION_RADIUS, BODY_COLLISION_Z_MAX,
                     BODY_COLLISION_Z_MIN, COLLISION_MARGIN,
                     LINK_COLLISION_RADII, GraspController)
from kinematics import Q_STAND
from utils.climb_collision import EPSILON_M, default_visual_scene

DT = 1.0 / 30.0
NAMES = ('lb', 'lf', 'lm', 'rb', 'rf', 'rm')
LINKS = ('thigh', 'knee', 'ankle')
EVIDENCE_DIR = ROOT / 'src/docs/evidence/climb_front_fast_20260923'
PAIR_CACHE_PATH = EVIDENCE_DIR / 'back_knee_spacing_visual_mesh_pair_cache.json'
VISUAL_MESH_PATHS = [ROOT / f'src/grasp_hexapod_description/meshes/{name}_link.STL'
                     for name in ('body', 'knee', 'ankle')]


def visual_mesh_hashes():
    return {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in VISUAL_MESH_PATHS}


def pair_cache_key(q, pose, left, right):
    payload = {'q_rad': np.asarray(q, dtype=float).tolist(),
               'base_pose': np.asarray(pose, dtype=float).tolist(),
               'pair': [left, right], 'visual_mesh_sha256': visual_mesh_hashes()}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def mesh_components(scene, q, pose, leg=None, link=None):
    world_from_base = ClimbMode._world_from_base(pose)
    if leg is None:
        components = scene.robot_components(q, world_from_base, leg_indices=[], include_body=True)
        return [item for item in components if item.mesh.link_name == 'body']
    leg_index = NAMES.index(leg) if isinstance(leg, str) else int(leg)
    components = scene.robot_components(q, world_from_base, leg_indices=[leg_index], include_body=False)
    return [item for item in components if item.mesh.link_name == link]


def check_pair(scene, q, pose, left, right):
    key = pair_cache_key(q, pose, left, right)
    cached = json.loads(PAIR_CACHE_PATH.read_text()) if PAIR_CACHE_PATH.exists() else {'checks': {}}
    if key in cached['checks']:
        result = dict(cached['checks'][key]['result'])
        result['cache_reused'] = True
        return result
    left_mesh = mesh_components(scene, q, pose, left.get('leg'), left['link'])
    right_mesh = mesh_components(scene, q, pose, right.get('leg'), right['link'])
    if not left_mesh or not right_mesh:
        raise RuntimeError(f"missing visual mesh components: {left}, {right}")
    left_filtered = filter_by_other_component_bounds(left_mesh, right_mesh)
    right_filtered = filter_by_other_component_bounds(right_mesh, left_mesh)
    left_triangles = sum(len(item.triangles) for item in left_filtered)
    right_triangles = sum(len(item.triangles) for item in right_filtered)
    if not left_triangles or not right_triangles:
        result_collision, hit, broad, tests = False, None, 0, 0
        status = 'conservative_triangle_AABB_prefilter_excluded_all_pairs'
    else:
        result = scene.cached_active_component_collision(left_filtered, right_filtered)
        result_collision, hit = bool(result.collision), result.hit
        broad, tests = int(result.broad_phase_candidates), int(result.narrow_phase_tests)
        status = ('triangle_candidates_checked' if tests > 0 else
                  'broadphase_excluded_all_triangle_pairs; collision result is clear')
    output = {
        'pair': [left, right],
        'mesh_checked': True,
        'pair_query_count': 1,
        'components_nonempty': {'left': len(left_mesh), 'right': len(right_mesh)},
        'pair_status': status,
        'filtered_triangle_counts': {'left': left_triangles, 'right': right_triangles},
        'triangle_test_count': tests,
        'broad_phase_candidates': broad,
        'mesh_collision': result_collision,
        'triangle_hit': None if hit is None else hit.__dict__,
        'cache_reused': False,
    }
    cached['checks'][key] = {
        'q_rad': np.asarray(q, dtype=float).tolist(),
        'base_pose': np.asarray(pose, dtype=float).tolist(),
        'pair': [left, right], 'visual_mesh_sha256': visual_mesh_hashes(),
        'result': {k: v for k, v in output.items() if k != 'cache_reused'},
    }
    PAIR_CACHE_PATH.write_text(json.dumps(cached, indent=2) + '\n')
    return output


def filter_by_other_component_bounds(components, other_components):
    """Apply only conservative triangle-AABB rejection; retain original indices."""
    output = []
    for component in components:
        triangles = component.triangles
        minimum = np.min(triangles, axis=1)
        maximum = np.max(triangles, axis=1)
        mask = np.zeros(len(triangles), dtype=bool)
        for other in other_components:
            mask |= (np.all(maximum + EPSILON_M >= other.bounds_min, axis=1)
                     & np.all(minimum <= other.bounds_max + EPSILON_M, axis=1))
        if np.any(mask):
            indices = component.component.triangle_indices[mask]
            filtered_component = replace(
                component.component, triangle_indices=indices,
                bounds_min=np.min(triangles[mask], axis=(0, 1)),
                bounds_max=np.max(triangles[mask], axis=(0, 1)),
            )
            output.append(replace(
                component, component=filtered_component, triangles=triangles[mask],
                bounds_min=filtered_component.bounds_min,
                bounds_max=filtered_component.bounds_max,
            ))
    return output


def capsule_body_gaps(controller, q, legs):
    points = controller.kinematic.link_points_base(q)
    start = np.array((0.0, 0.0, BODY_COLLISION_Z_MIN))
    end = np.array((0.0, 0.0, BODY_COLLISION_Z_MAX))
    gaps = []
    for leg in legs:
        for link_index in (1, 2):
            distance = controller._segment_distance(
                points[leg, link_index], points[leg, link_index + 1], start, end)
            required = (BODY_COLLISION_RADIUS + LINK_COLLISION_RADII[link_index]
                        + COLLISION_MARGIN)
            gaps.append({
                'leg': NAMES[leg], 'link': LINKS[link_index],
                'proxy_gap_m': float(distance - required),
            })
    return gaps


def local_stage_replay(config, audit, stage_name):
    index = next(i for i, stage in enumerate(config['stages']) if stage['name'] == stage_name)
    boundary = audit['boundaries'][stage_name]
    entry_q = np.asarray(boundary['entry_q_rad'], dtype=float)
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    controller.enter_climb(entry_q, config, index, index, hardware_execution=False)
    mode = controller.climb_mode
    q = entry_q.copy()
    points = []
    _, _, total = mode._stage_reference()
    ticks = int(round(total / DT))
    for _ in range(ticks + 1):
        pose, _, _ = mode._stage_reference()
        gaps = capsule_body_gaps(controller, q, (1, 4) if stage_name == 'FRONT_INWARD' else (2, 5))
        for gap in gaps:
            points.append({
                **gap, 'phase_time_s': float(mode.phase_time),
                'q_rad': q.tolist(), 'base_pose': pose.tolist(),
            })
        if mode.phase_time >= total:
            break
        q = controller.update(q, np.zeros(4))
    if not points:
        raise RuntimeError(f'no local stage replay samples for {stage_name}')
    return min(points, key=lambda item: item['proxy_gap_m'])


def local_first_tick(config, audit, stage_name):
    index = next(i for i, stage in enumerate(config['stages']) if stage['name'] == stage_name)
    entry_q = np.asarray(audit['boundaries'][stage_name]['entry_q_rad'], dtype=float)
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    controller.enter_climb(entry_q, config, index, index, hardware_execution=False)
    q = controller.update(entry_q.copy(), np.zeros(4))
    mode = controller.climb_mode
    pose, _, _ = mode._stage_reference()
    return q, pose, float(mode.phase_time)


def save_evidence(records, audit, candidate_path, complete=False):
    unique_records = []
    seen_records = set()
    for record in records:
        identity = (record['label'], json.dumps(record.get('q_rad'), separators=(',', ':')),
                    json.dumps(record.get('base_pose'), separators=(',', ':')))
        if identity not in seen_records:
            seen_records.add(identity)
            unique_records.append(record)
    records = unique_records
    baseline_path = ROOT / 'src/grasp_hexapod_control/config/climb_front.json'
    mesh_paths = [ROOT / f'src/grasp_hexapod_description/meshes/{name}_link.STL'
                  for name in ('body', 'knee', 'ankle')]
    mesh_hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in mesh_paths}
    repair_checks = [item for record in records if 'rear_knee_pairs' in record['label']
                     for item in record['pair_checks']]
    body_checks = [item for record in records if 'body_proxy_witness' in record['label']
                   for item in record['pair_checks']]
    all_checks = [item for record in records for item in record['pair_checks']]
    required_complete = (
        len(repair_checks) == 4 and len(body_checks) == 8
        and sum('worst_body_proxy_gap' in item['label'] for item in records) == 2
        and all(item.get('mesh_checked') and item.get('pair_query_count') == 1
                and all(item.get('components_nonempty', {}).values()) for item in all_checks)
    )
    out = {
        'method': 'Explicit selected visual-mesh triangle intersection queries at exact candidate 30 Hz witnesses, using conservative triangle-AABB prefilter and original triangle indices; no whole-robot or Xiaolan scan.',
        'candidate_cpu_audit': {'state': audit['state'], 'ticks': audit['ticks'],
                                'zero_velocity_clips': sum(
                                    int(row.get('velocity_clips', 0))
                                    for row in audit['stage_rows'].values())},
        'baseline_config_sha256': hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
        'candidate_config_sha256': hashlib.sha256(candidate_path.read_bytes()).hexdigest(),
        'visual_mesh_sha256': mesh_hashes,
        'required_counts': {
            'rear_knee_pairs_checked': len(repair_checks),
            'body_witness_pairs_checked': len(body_checks),
            'body_worst_phase_pairs_checked': sum('worst_body_proxy_gap' in item['label'] for item in records),
            'all_required_pairs_checked': bool(complete and required_complete),
        },
        'records': records,
        'boundary': 'A clear result covers triangle intersection at the sampled pose, with triangle AABB rejection where narrow-phase count is zero. cached_active_component_collision uses components_only and does not perform point-in-solid containment. Samples do not prove continuous clearance, contact, load, stability, simulator collision filtering, or hardware safety.',
    }
    out_path = ROOT / 'src/docs/evidence/climb_front_fast_20260923/back_knee_spacing_visual_mesh_audit.json'
    out_path.write_text(json.dumps(out, indent=2) + '\n')
    return out


def main():
    candidate_path = EVIDENCE_DIR / 'front_back_spacing_candidate.json'
    audit_path = EVIDENCE_DIR / 'front_back_spacing_audit.json'
    config = json.loads(candidate_path.read_text())
    audit_doc = json.loads(audit_path.read_text())
    audit = audit_doc['candidate']
    global audit_doc_gate
    audit_doc_gate = audit_doc['gate']
    if audit['state'] != 'DONE' or audit['ticks'] != 2737:
        raise RuntimeError('cached candidate audit does not match frozen successful replay')

    # All earlier pair results are already durable in PAIR_CACHE_PATH. Do not
    # depend on temporary streaming output or repeat completed mesh queries.
    prior_records = []

    # One known old real visual knee collision guards the conservative AABB
    # prefilter. It must retain the baseline N-exit hit.
    old_path = EVIDENCE_DIR / 'capsule_proxy_visual_mesh_audit.json'
    old_doc = json.loads(old_path.read_text())
    baseline_n = next(item for item in old_doc['records'] if item['label'] == 'baseline_N_exit')
    baseline_pair = baseline_n['pairs'][0]['legs']
    prefilter_regression = check_pair(
        scene, np.asarray(baseline_n['q_rad']), np.asarray(baseline_n['base_pose']),
        {'leg': baseline_pair[0], 'link': 'knee'},
        {'leg': baseline_pair[1], 'link': 'knee'},
    )
    if not prefilter_regression['mesh_collision']:
        raise RuntimeError('conservative triangle-AABB prefilter lost known baseline knee collision')
    print(json.dumps({'baseline_prefilter_regression': prefilter_regression}, indent=2), flush=True)

    records = []
    # Query the worst body-proxy gap first so the first mesh result reaches the
    # caller promptly. Each query uses the cached stage-entry q, never cold IK.
    early_worst_records = list(prior_records)
    for stage_name in ('FRONT_INWARD', 'Q_MID_LOW'):
        worst = local_stage_replay(config, audit, stage_name)
        label = f'candidate_{stage_name}_worst_body_proxy_gap'
        prior = next((item for item in early_worst_records if item['label'] == label), None)
        if prior is not None:
            if (not np.allclose(prior['q_rad'], worst['q_rad'], rtol=0.0, atol=1e-10)
                    or not np.allclose(prior['base_pose'], worst['base_pose'], rtol=0.0, atol=1e-10)):
                raise RuntimeError(f'cached prior mesh sample changed for {stage_name}')
            print(json.dumps({'reused_prior_pair': label, 'result': prior['pair_checks'][0]}, indent=2), flush=True)
            continue
        result = check_pair(
            scene, np.asarray(worst['q_rad']), np.asarray(worst['base_pose']),
            {'leg': None, 'link': 'body'}, {'leg': worst['leg'], 'link': worst['link']},
        )
        record = {'label': label,
                  'source_stage': stage_name,
                  'note': 'local 30 Hz replay from cached stage entry; minimum body-link capsule proxy gap',
                  'phase_time_s': worst['phase_time_s'], 'q_rad': worst['q_rad'],
                  'base_pose': worst['base_pose'], 'worst_proxy_pair': {
                      'leg': worst['leg'], 'link': worst['link'],
                      'proxy_gap_m': worst['proxy_gap_m']},
                  'pair_checks': [result]}
        early_worst_records.append(record)
        print(json.dumps({'first_worst_body_mesh_result': record}, indent=2), flush=True)
        save_evidence(early_worst_records, audit, candidate_path)
        if result['mesh_collision']:
            baseline_path = ROOT / 'src/grasp_hexapod_control/config/climb_front.json'
            out_path = ROOT / 'src/docs/evidence/climb_front_fast_20260923/back_knee_spacing_visual_mesh_audit.json'
            out_path.write_text(json.dumps({
                'method': 'Explicit selected visual mesh query at exact candidate witness; stopped at first real collision.',
                'baseline_config_sha256': hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
                'candidate_config_sha256': hashlib.sha256(candidate_path.read_bytes()).hexdigest(),
                'records': [record],
                'boundary': 'A triangle collision at this sampled pose is a real visual-mesh intersection. Simulation evidence only; no hardware conclusion.',
            }, indent=2) + '\n')
            print('STOP: exact visual mesh collision confirmed; remaining pairs were not queried.', flush=True)
            return

    repair_states = {}
    for stage_name in ('LM_MID_RELAND', 'OP_MID_BODY_LIFT'):
        repair_states[stage_name] = local_first_tick(config, audit, stage_name)

    front = audit['stage_rows']['FRONT_INWARD']['capsule_witness']
    low = audit['stage_rows']['Q_MID_LOW']['capsule_witness']
    controller = GraspController(DT, enable_link_collision_check=False,
                                 climb_timeout_uses_wall_time=False)
    for stage_name, note, pairs in (
        ('LM_MID_RELAND', 'candidate repaired LM witness; LB-LM and RB-RM knee pairs',
         ((0, 2), (3, 5))),
        ('OP_MID_BODY_LIFT', 'candidate repaired OP witness; LB-LM and RB-RM knee pairs',
         ((0, 2), (3, 5))),
    ):
        q0, pose, phase = repair_states[stage_name]
        record = {'label': f'candidate_{stage_name}_rear_knee_pairs', 'source_stage': stage_name,
                  'note': note, 'phase_time_s': phase, 'q_rad': q0.tolist(),
                  'base_pose': pose.tolist(), 'pair_checks': []}
        for left_leg, right_leg in pairs:
            pair_result = check_pair(
                scene, q0, pose,
                {'leg': NAMES[left_leg], 'link': 'knee'},
                {'leg': NAMES[right_leg], 'link': 'knee'},
            )
            record['pair_checks'].append(pair_result)
            records_now = [*records, record, *early_worst_records]
            save_evidence(records_now, audit, candidate_path)
            print(json.dumps({'completed_pair': record['label'], **pair_result}, indent=2), flush=True)
        records.append(record)

    for stage_name, witness, target_legs in (
        ('FRONT_INWARD', front, (1, 4)), ('Q_MID_LOW', low, (2, 5)),
    ):
        q0 = np.asarray(witness['q_rad'], dtype=float)
        pose = np.asarray(witness['base_pose'], dtype=float)
        phase = float(witness['phase_time_s'])
        record = {'label': f'candidate_{stage_name}_body_proxy_witness',
                  'source_stage': stage_name, 'note': 'cached first body capsule proxy warning witness',
                  'phase_time_s': phase, 'q_rad': q0.tolist(), 'base_pose': pose.tolist(),
                  'proxy_gaps_m': capsule_body_gaps(controller, q0, target_legs), 'pair_checks': []}
        for leg in target_legs:
            for link in ('knee', 'ankle'):
                pair_result = check_pair(
                    scene, q0, pose, {'leg': None, 'link': 'body'},
                    {'leg': NAMES[leg], 'link': link},
                )
                record['pair_checks'].append(pair_result)
                records_now = [*records, record, *early_worst_records]
                save_evidence(records_now, audit, candidate_path)
                print(json.dumps({'completed_pair': record['label'], **pair_result}, indent=2), flush=True)
        records.append(record)

    records.extend(early_worst_records)
    final = save_evidence(records, audit, candidate_path, complete=True)
    final['prefilter_regression'] = {
        'case': 'baseline_N_exit', 'expected_mesh_collision': True,
        'observed_mesh_collision': prefilter_regression['mesh_collision'],
        'triangle_hit': prefilter_regression['triangle_hit'],
        'narrow_phase_tests': prefilter_regression['triangle_test_count'],
    }
    (EVIDENCE_DIR / 'back_knee_spacing_visual_mesh_audit.json').write_text(json.dumps(final, indent=2) + '\n')
    print(json.dumps(final, indent=2))


if __name__ == '__main__':
    scene = default_visual_scene(ROOT)
    main()
