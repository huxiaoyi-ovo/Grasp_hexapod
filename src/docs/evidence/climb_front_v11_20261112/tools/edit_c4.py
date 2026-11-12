"""v11 step 1b: C4 moves RF only to the right wing centre; LF stays on its wide-face centre from C3.
C8's LF lift knot is moved above LF's new start. Simulation-only planning edit."""
import copy, json, sys
import numpy as np
import chain
from chain import surface_height, TRI
src = json.load(open(sys.argv[1])); out = copy.deepcopy(src); st = out['stages']
rf_x, apex = float(sys.argv[3]), float(sys.argv[4])
body_x = float(sys.argv[5]) if len(sys.argv) > 5 else None
i4 = [s['name'] for s in st].index('FRONT_TO_029')
s = st[i4]; k = np.array(s['anchor_knots'])
start = np.array(st[i4 - 1]['anchor_knots'][-1])
k[:, 1] = start[1]                                         # LF fixed
rf = start[4]; tgt = np.array([rf_x, rf[1], surface_height(TRI, rf_x, rf[1])])
k[1, 4] = [rf[0], rf[1], apex]; k[2, 4] = [tgt[0], tgt[1], apex]; k[3, 4] = tgt
s['active_legs'] = [4]; s['anchor_knots'] = k.tolist(); s['name'] = 'RF_TO_WING'
if body_x is not None:
    pe = list(s['pose_end']); pe[0] = body_x; s['pose_end'] = pe
s.pop('active_base_velocities_m_s', None)
chain.base_from_world(s)
chain.rebuild(st, i4 + 1)
i5 = i4 + 1
st[i5]['segment_durations_s'] = [0.7]
i8 = [x['name'] for x in st].index('FRONT_TO_034')
k8 = np.array(st[i8]['anchor_knots']); lf0 = k8[0, 1]
k8[1, 1, :2] = lf0[:2]                                     # lift straight above the new LF start
st[i8]['anchor_knots'] = k8.tolist(); chain.base_from_world(st[i8], keep_velocities=True)
out.setdefault('v11_edits', []).append(dict(step='C4', change='RF only to wing (%.3f,%.3f) with body x -> %s; LF stays at face centre; C5 .70 s; C8 LF lift above new start' % (tgt[0], tgt[1], body_x)))
chain.validate(out)
json.dump(out, open(sys.argv[2], 'w'), indent=1)
print('RF target', np.round(tgt, 4), 'C8 LF knots', np.round(k8[:, 1], 3).tolist())
