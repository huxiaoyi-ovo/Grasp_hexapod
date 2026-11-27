"""Merge body-only stages into the following swing stage (body moves during the swing), removing a stop.
Usage: merge_body.py in out NAME [NAME ...]"""
import json, sys
import numpy as np
import chain
cfg = json.load(open(sys.argv[1])); st = cfg['stages']
for name in sys.argv[3:]:
    i = [s['name'] for s in st].index(name); b, s = st[i], st[i + 1]
    assert not b['active_legs'] and s['active_legs']
    s['pose_start'] = b['pose_start']
    if 'pose_knots' in s:
        pk = np.array(s['pose_knots']); pk[0] = b['pose_start']; s['pose_knots'] = pk.tolist()
    k = np.array(s['anchor_knots']); k[0] = np.array(b['anchor_knots'][0]); s['anchor_knots'] = k.tolist()
    d = np.array(s['segment_durations_s']); Tb = sum(b['segment_durations_s'])
    s['segment_durations_s'] = (d * max(1.0, (d.sum() + .5 * Tb) / d.sum())).tolist()   # keep at least half the body time
    if s['anchor_curve'] == 'piecewise_base_quintic':
        chain.base_from_world(s, keep_velocities=True)
    s['name'] = s['name']
    del st[i]
cfg.setdefault('v11_edits', []).append(dict(step='MERGE', change='body-only stages merged into next swing: ' + ','.join(sys.argv[3:])))
chain.validate(cfg); json.dump(cfg, open(sys.argv[2], 'w'), indent=1)
print('stages', len(st))
