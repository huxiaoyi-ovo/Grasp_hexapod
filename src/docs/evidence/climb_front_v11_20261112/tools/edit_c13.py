"""C13 front-pair intermediate footholds (.48) sit 8 mm in front of the top's transverse slot (x~.43-.47):
move LF to (.491,.029) (centre of .472-.51) and RF to (.52,-.165). Later stages start from the new anchors."""
import json, sys
import numpy as np
import chain
from chain import surface_height, TRI
out = json.load(open(sys.argv[1])); st = out['stages']; names = [s['name'] for s in st]
i = names.index('FRONT_DIRECT_FINAL_480'); s = st[i]; k = np.array(s['anchor_knots'])
for leg, x in ((1, float(sys.argv[3])), (4, float(sys.argv[4]))):
    y = k[-1, leg, 1]; z = surface_height(TRI, x, y)
    for j in range(2, len(k)):
        k[j, leg, 0] = x
    k[-1, leg, 2] = z
s['anchor_knots'] = k.tolist()
if s['anchor_curve'] == 'piecewise_base_quintic':
    chain.base_from_world(s, keep_velocities=True)
chain.rebuild(st, i + 1)
for name, leg in (('DIAGONAL_RF_LM_ADVANCE', 4), ('TRIPOD_LB_LF_RM_ADVANCE', 1)):
    t = st[names.index(name)]; kk = np.array(t['anchor_knots']); kk[1, leg, :2] = kk[0, leg, :2]; t['anchor_knots'] = kk.tolist()
    if t['anchor_curve'] == 'piecewise_base_quintic':
        chain.base_from_world(t, keep_velocities=True)
out['v11_edits'].append(dict(step='C13', change='front intermediate LF x %s RF x %s (clear of transverse slot)' % (sys.argv[3], sys.argv[4])))
chain.validate(out); json.dump(out, open(sys.argv[2], 'w'), indent=1)
