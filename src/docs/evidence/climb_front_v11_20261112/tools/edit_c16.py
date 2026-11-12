"""v11 step 3: C16 LM final target (.425,.100) -> (.400,.100): thigh near its .698 rad limit and only 11 mm
from the top planar-region front edge. Later stages follow (LM fixed)."""
import json, sys
import numpy as np
import chain
from chain import surface_height, TRI
out = json.load(open(sys.argv[1])); st = out['stages']
i = [s['name'] for s in st].index('DIAGONAL_RF_LM_ADVANCE'); s = st[i]
x, y = float(sys.argv[3]), float(sys.argv[4])
k = np.array(s['anchor_knots']); k[2, 2, :2] = (x, y); k[-1, 2] = (x, y, surface_height(TRI, x, y))
for j in range(1, len(k) - 1):
    if j >= 2: k[j, 2, :2] = (x, y)
s['anchor_knots'] = k.tolist()
if s['anchor_curve'] == 'piecewise_base_quintic':
    chain.base_from_world(s, keep_velocities=True)
chain.rebuild(st, i + 1)
out['v11_edits'].append(dict(step='C16', change='LM final (.425,.100) -> (%.3f,%.3f)' % (x, y)))
chain.validate(out); json.dump(out, open(sys.argv[2], 'w'), indent=1)
print(np.round(k[:, 2], 3).tolist())
