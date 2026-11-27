"""User: second-to-last (RB) and last (LB) climbing legs should not tuck inward; keep the shank close to the
landing-surface normal. New footholds RB (.212,-.225), LB (.205,.105) on planar first-face regions; the last
approach knots follow; later stages start from the new anchors (their lift knots above the new start)."""
import json, sys
import numpy as np
import chain
from chain import surface_height, TRI
out = json.load(open(sys.argv[1])); st = out['stages']; names = [s['name'] for s in st]
tg = {3: (.212, -.225), 0: (float(sys.argv[3]) if len(sys.argv) > 3 else .225, .105)}
for name, leg, side in (('RB_DIRECT_PLATFORM_225', 3, -1), ('LB_PLATFORM_225', 0, 1)):
    s = st[names.index(name)]; k = np.array(s['anchor_knots'])
    x, y = tg[leg]; t = np.array((x, y, surface_height(TRI, x, y)))
    k[-3, leg] = t + (0, side * .010, .035)
    k[-2, leg] = t + (0, 0, .020)
    k[-1, leg] = t
    s['anchor_knots'] = k.tolist(); chain.base_from_world(s, keep_velocities=True)
    chain.rebuild(st, names.index(name) + 1)
for name, leg in (('TRIPOD_LB_LF_RM_ADVANCE', 0), ('RB_FINAL_ADVANCE', 3)):
    s = st[names.index(name)]; k = np.array(s['anchor_knots'])
    k[1, leg, :2] = k[0, leg, :2] + .4 * (k[-1, leg, :2] - k[0, leg, :2])   # rise while drawing toward the target
    s['anchor_knots'] = k.tolist()
    if s['anchor_curve'] == 'piecewise_base_quintic':
        chain.base_from_world(s, keep_velocities=True)
out['v11_edits'].append(dict(step='REAR_OUT', change='RB foothold -> (.212,-.225), LB -> (.205,.105): shank near surface normal'))
chain.validate(out); json.dump(out, open(sys.argv[2], 'w'), indent=1)
