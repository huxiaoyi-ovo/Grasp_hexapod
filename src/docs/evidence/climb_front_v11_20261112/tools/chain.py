"""Chain-editing helpers for stage-wise front-climb edits (simulation-only planning; model diagnostics).

rebuild(stages, start) re-propagates pose_start/anchor continuity from stage `start` on:
fixed legs take the previous end anchors, an active leg's first knot takes its previous end anchor,
interior/end world knots are kept, and base-relative knots are recomputed in the body pose at each knot
time (identical rule to the v10 builder). Stages whose inputs are unchanged are left byte-identical."""
import copy, json, sys
from pathlib import Path
import numpy as np

REPO = Path('/home/artrc/Grasp_hexapod')
sys.path[:0] = [str(REPO / 'src/grasp_hexapod_control/scripts'), str(REPO / 'src/grasp_hexapod_control/scripts/tools')]
from build_climb_front import MESH, base_points, load_triangles, surface_height  # noqa: E402
from climb_mode import ClimbMode  # noqa: E402
TRI = load_triangles(MESH)


def world_rot(pose):
    from build_climb_front import world_from_base
    return world_from_base(np.asarray(pose, float))[:3, :3]


def smooth(t):
    return t ** 3 * (10 - 15 * t + 6 * t * t)


def base_from_world(stage, cap=.040, keep_velocities=False):
    ps, pe = np.array(stage['pose_start']), np.array(stage['pose_end'])
    durs = np.array(stage['segment_durations_s'])
    tk = np.r_[0, np.cumsum(durs)] / durs.sum()
    k = np.array(stage['anchor_knots'])
    act = stage['active_legs']
    if stage.get('pose_curve') == 'piecewise_quintic':
        poses = np.array(stage['pose_knots'])            # runtime: pose at knot i is pose_knots[i]
    else:
        poses = [ps * (1 - smooth(t)) + pe * smooth(t) for t in tk]
    base = np.array([base_points(p, kk[act]) for p, kk in zip(poses, k)])
    times = np.r_[0, np.cumsum(durs)]
    vel = np.zeros_like(base)
    for i in range(1, len(base) - 1):
        v = (base[i + 1] - base[i - 1]) / (times[i + 1] - times[i - 1])
        n = np.linalg.norm(v, axis=1, keepdims=True)
        vel[i] = v * np.minimum(1.0, cap / np.maximum(n, 1e-15))
    stage['active_base_knots_m'] = base.tolist()
    if not (keep_velocities and np.shape(stage.get('active_base_velocities_m_s')) == base.shape):
        stage['active_base_velocities_m_s'] = vel.tolist()


def rebuild(stages, start):
    for i in range(max(start, 1), len(stages)):
        prev, s = stages[i - 1], stages[i]
        pe_prev = np.array(prev['pose_end'])
        end_prev = np.array(prev['anchor_knots'][-1])
        k = np.array(s['anchor_knots'])
        k0 = k.copy()
        fixed = [l for l in range(6) if l not in s['active_legs']]
        k[:, fixed] = end_prev[fixed]
        k[0] = end_prev
        if np.allclose(k, k0, rtol=0, atol=1e-12) and np.allclose(s['pose_start'], pe_prev, rtol=0, atol=1e-12):
            continue                                   # unchanged inputs: keep the stage byte-identical
        if s['pose_start'] == s['pose_end'] and not np.allclose(s['pose_start'], pe_prev):
            s['pose_end'] = pe_prev.tolist()          # body-fixed stage follows the new body
        s['pose_start'] = pe_prev.tolist()
        if 'pose_knots' in s:
            pk = np.array(s['pose_knots']); pk[0] = pe_prev; s['pose_knots'] = pk.tolist()
        s['anchor_knots'] = k.tolist()
        if s.get('anchor_curve') == 'piecewise_base_quintic':
            base_from_world(s, keep_velocities=True)
    return stages


def swing_stage(name, ps, pe, start_anchors, active, targets_xy, apex_z, durs, lift_fwd=0.0):
    """Lift-cross-land world swing for `active` legs; landing z from the exact STL envelope."""
    a = np.array(start_anchors, float)
    tgt = np.array([[x, y, surface_height(TRI, x, y)] for x, y in targets_xy])
    k = np.repeat(a[None], 4, 0)
    apex = np.broadcast_to(np.asarray(apex_z, float), (len(active),))
    for j, l in enumerate(active):
        k[1, l] = [a[l, 0] + lift_fwd, a[l, 1], apex[j]]
        k[2, l] = [tgt[j, 0], tgt[j, 1], apex[j]]
        k[3, l] = tgt[j]
    s = dict(name=name, pose_start=list(map(float, ps)), pose_end=list(map(float, pe)), active_legs=list(active),
             anchor_knots=k.tolist(), segment_durations_s=list(map(float, durs)),
             anchor_curve='piecewise_base_quintic', pose_curve='quintic_full_stage', settle_s=.15)
    base_from_world(s)
    return s


def validate(cfg):
    cfg['stage_count'] = len(cfg['stages'])
    ClimbMode(None)._validate_config(cfg)
    return cfg
