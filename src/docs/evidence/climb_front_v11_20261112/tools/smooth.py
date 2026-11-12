"""User: smoother, faster steps. Every swing stage becomes a body-relative Hermite curve whose interior knots
carry natural pass-through velocities (central differences, scaled by `k`), so the foot only stops at
lift-off and touchdown; the pre-contact knot keeps a slow downward approach. Simulation-only planning."""
import json, sys
import numpy as np
import chain


def smooth_stage(s, k=.8, v_pre=.02):
    if not s['active_legs']:
        return
    s['anchor_curve'] = 'piecewise_base_quintic'
    s.pop('active_base_velocities_m_s', None)
    chain.base_from_world(s)                                   # base knots at each knot's body pose
    b = np.array(s['active_base_knots_m']); d = np.array(s['segment_durations_s']); t = np.r_[0, np.cumsum(d)]
    v = np.zeros_like(b)
    tk = t / t[-1]
    ps, pe = np.array(s['pose_start']), np.array(s['pose_end'])
    poses = np.array(s['pose_knots']) if s.get('pose_curve') == 'piecewise_quintic' else \
        [ps * (1 - chain.smooth(x)) + pe * chain.smooth(x) for x in tk]
    for i in range(1, len(b) - 1):
        vb = k * (b[i + 1] - b[i - 1]) / (t[i + 1] - t[i - 1])
        R = chain.world_rot(poses[i])
        vw = vb @ R.T
        if i >= 2:
            vw[:, 2] = np.maximum(vw[:, 2], 0.0)                # after the lift knot: never cut down early (riser clearance)
        v[i] = vw @ R
    if len(b) >= 3:                                            # pre-contact knot: slow approach toward touchdown
        dirn = b[-1] - b[-2]; n = np.linalg.norm(dirn, axis=1, keepdims=True)
        v[-2] = dirn / np.maximum(n, 1e-9) * v_pre
    s['active_base_velocities_m_s'] = v.tolist()


if __name__ == '__main__':
    cfg = json.load(open(sys.argv[1]))
    keep = set(sys.argv[3:])                                    # stages whose source curve is kept
    for s in cfg['stages']:
        if s['name'] not in keep:
            smooth_stage(s)
    cfg.setdefault('v11_edits', []).append(dict(step='SMOOTH', change='all swings body-relative Hermite with pass-through knot velocities; stop only at lift-off/touchdown'))
    chain.validate(cfg); json.dump(cfg, open(sys.argv[2], 'w'), indent=1)
