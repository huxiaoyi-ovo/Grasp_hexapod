"""Slip test: stretch every stage whose body translation exceeds a peak-acceleration limit
(quintic: a_peak = 5.77 d / T^2) so T >= sqrt(5.77 d / a_lim). Durations scale uniformly within the stage."""
import json, sys
import numpy as np
import chain
cfg = json.load(open(sys.argv[1])); a_lim = float(sys.argv[3]); changed = []
for s in cfg['stages']:
    d = np.linalg.norm(np.array(s['pose_end'][:3]) - np.array(s['pose_start'][:3]))
    T = sum(s['segment_durations_s']); Tmin = np.sqrt(5.77 * d / a_lim) if d > 1e-6 else 0
    if Tmin > T:
        s['segment_durations_s'] = (np.array(s['segment_durations_s']) * Tmin / T).tolist()
        if s['anchor_curve'] == 'piecewise_base_quintic':
            chain.base_from_world(s, keep_velocities=True)
        changed.append((s['name'], round(T, 2), round(Tmin, 2)))
cfg.setdefault('v11_edits', []).append(dict(step='BODY_ACCEL', change='body translation peak accel <= %.2f m/s2' % a_lim))
chain.validate(cfg); json.dump(cfg, open(sys.argv[2], 'w'), indent=1)
print('stretched:', changed); print('total %.1f s' % sum(sum(s['segment_durations_s']) for s in cfg['stages']))
