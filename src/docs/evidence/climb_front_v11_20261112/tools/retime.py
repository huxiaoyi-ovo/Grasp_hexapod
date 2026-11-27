"""User item 2: air fast, touchdown gentle.
For every swing stage: (1) insert a pre-contact knot `pre` metres above the touchdown point along the
approach direction; the descent apex->pre is acceleration-limited (<= a_desc) and keeps a small downward
speed v_pre at the pre-contact knot (base-relative Hermite stages); the last `pre` metres take t_touch s.
(2) every other (lift/cross/descent) segment is rescaled so its peak reference joint rate equals `cap`
(never below a_desc's minimum or t_min). Body-only stages are untouched.
Simulation-only planning; reference rates are model numbers, not measured servo speed."""
import copy, json, sys
import numpy as np
import chain
from refspeed import profile
sys.path[:0] = ['/home/artrc/grasp_sync_backups/claude_front_opt_20261110']
from geom import apply, inv_rigid, world_from_base, base_to_hip, leg_ik  # noqa: E402
from climb_mode import ClimbMode  # noqa: E402

PRE, T_TOUCH, A_DESC, V_PRE, CAP, T_MIN = .008, .25, 1.5, .02, 2.75, .25
CAP_HI = 3.3


def pose_at(stage, t):
    m = ClimbMode(None); m.config = {'stages': [stage]}; m.stage_index = 0; m.phase_time = t
    return m._stage_reference()[0]


def insert_precontact(s):
    act = s['active_legs']
    k = np.array(s['anchor_knots']); d = np.array(s['segment_durations_s'])
    last, prev = k[-1, act], k[-2, act]
    dist = np.linalg.norm(prev - last, axis=1)
    if dist.max() <= PRE + .004:
        return False
    u = 1 - PRE / dist.max()                                   # fraction of the last segment
    t_split = d[:-1].sum() + u * d[-1]
    pose_split = pose_at(s, t_split)
    knot = k[-1].copy()
    dirs = (prev - last) / np.maximum(dist, 1e-9)[:, None]
    knot[act] = last + dirs * np.minimum(PRE, dist)[:, None]
    k = np.concatenate((k[:-1], knot[None], k[-1:]))
    s['anchor_knots'] = k.tolist()
    desc = dist.max() - PRE
    s['segment_durations_s'] = list(d[:-1]) + [max(np.sqrt(5.77 * desc / A_DESC), T_MIN), T_TOUCH]
    if 'pose_knots' in s:
        pk = np.array(s['pose_knots']); s['pose_knots'] = np.concatenate((pk[:-1], pose_split[None], pk[-1:])).tolist()
    if s['anchor_curve'] == 'piecewise_base_quintic':
        v = np.array(s['active_base_velocities_m_s'])
        R = world_from_base(np.array(s['pose_end']))[:3, :3]
        vpre = -(dirs @ R) * V_PRE                              # slow downward approach, base frame
        s['active_base_velocities_m_s'] = np.concatenate((v[:-1], vpre[None], v[-1:])).tolist()
        chain.base_from_world(s, keep_velocities=True)
    return True


def rescale_air(cfg, idx, iters=3):
    s = cfg['stages'][idx]
    n = len(s['segment_durations_s'])
    fixed_tail = 2 if s.get('_pre') else 0                     # descent + touchdown keep their limits
    for _ in range(iters):
        ts, qs, rate = profile(cfg, s['name'])
        d = np.array(s['segment_durations_s']); cum = np.r_[0, np.cumsum(d)]
        for j in range(n - fixed_tail):
            sel = (ts[1:] > cum[j]) & (ts[1:] <= cum[j + 1])
            if not sel.any():
                continue
            peak = rate[sel[:len(rate)]].max()
            if peak < CAP:
                d[j] = max(T_MIN, d[j] * peak / CAP)             # speed slow air segments up to CAP
            elif peak > CAP_HI:
                d[j] = d[j] * peak / CAP_HI                      # never let a segment exceed CAP_HI
        if fixed_tail:                                          # descent never faster than the rate cap either
            j = n - 2; sel = (ts[1:] > cum[j]) & (ts[1:] <= cum[j + 1])
            if sel.any():
                d[j] = max(d[j], d[j] * rate[sel[:len(rate)]].max() / CAP)
        s['segment_durations_s'] = d.tolist()
        if s['anchor_curve'] == 'piecewise_base_quintic':
            chain.base_from_world(s, keep_velocities=True)


def main(src, dst, slow=()):
    cfg = json.load(open(src)); out = copy.deepcopy(cfg); st = out['stages']
    before = sum(sum(s['segment_durations_s']) for s in st)
    for i, s in enumerate(st):
        if not s['active_legs']:
            continue
        s['_pre'] = insert_precontact(s)
    for i, s in enumerate(st):
        if s['active_legs'] and s['name'] not in slow:
            rescale_air(out, i)
    for s in st:
        s.pop('_pre', None)
    after = sum(sum(s['segment_durations_s']) for s in st)
    out.setdefault('v11_edits', []).append(dict(step='RETIME', change='pre-contact %.0f mm / %.2f s touchdown, descent accel <= %.1f m/s2 with %.2f m/s pre-contact speed, air segments at %.1f rad/s reference cap' % (PRE * 1000, T_TOUCH, A_DESC, V_PRE, CAP)))
    chain.validate(out); json.dump(out, open(dst, 'w'), indent=1)
    print('nominal motion time %.1f s -> %.1f s' % (before, after))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3:])
