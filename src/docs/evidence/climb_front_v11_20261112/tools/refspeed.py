"""Reference joint-rate profile of one stage at 30 Hz (runtime _stage_reference + closed-form IK)."""
import json, sys, numpy as np
sys.path[:0] = ['/home/artrc/grasp_sync_backups/claude_front_opt_20261110', '/home/artrc/Grasp_hexapod/src/grasp_hexapod_control/scripts']
from geom import apply, inv_rigid, world_from_base, base_to_hip, leg_ik, margins
from climb_mode import ClimbMode
L = ('lb', 'lf', 'lm', 'rb', 'rf', 'rm'); J = ('thigh', 'knee', 'ankle')


def profile(cfg, name, dt=1 / 30):
    m = ClimbMode(None); m.config = cfg; m.stage_index = [s['name'] for s in cfg['stages']].index(name)
    s = cfg['stages'][m.stage_index]; T = sum(s['segment_durations_s'])
    qs, ts = [], np.arange(0, T + 1e-9, dt)
    for t in ts:
        m.phase_time = t; pose, anchors, _ = m._stage_reference()
        q, ok = leg_ik(base_to_hip(apply(inv_rigid(world_from_base(pose)), anchors)[None]))
        qs.append(q[0])
    qs = np.array(qs); rate = np.abs(np.diff(qs, axis=0)) / dt
    return ts, qs, rate


if __name__ == '__main__':
    cfg = json.load(open(sys.argv[1]))
    for name in sys.argv[2:]:
        ts, qs, rate = profile(cfg, name)
        i, l, j = np.unravel_index(np.argmax(rate), rate.shape)
        mg = margins(qs).min(axis=(1, 2))
        print(f'{name}: peak ref rate {rate.max():.2f} rad/s at t={ts[i]:.2f}s {L[l]} {J[j]}; min margin {mg.min():.3f} at t={ts[np.argmin(mg)]:.2f}')
