"""v11 step 2 (user item 1): before the last pair climbs, RM advances along the right wing while the body
pre-shifts forward, so RB lifts with the COM well ahead of the LB-RM diagonal. LB ground relay .155->.14.
Later stages only follow the new start states (RM's later lift knot sits above its new start)."""
import copy, json, sys
import numpy as np
import chain
src = json.load(open(sys.argv[1])); out = copy.deepcopy(src); st = out['stages']
lbx, rmx, bx, bz, bp = map(float, sys.argv[3:8])
names = [s['name'] for s in st]
i13 = names.index('LB_GROUND_TO_155')
k = np.array(st[i13]['anchor_knots']); k[2:, 0, 0] = lbx; st[i13]['anchor_knots'] = k.tolist(); st[i13]['name'] = 'LB_GROUND_RELAY'
ps = np.array(st[i13]['pose_end']); pe = ps.copy(); pe[0], pe[2], pe[3] = bx, bz, bp
a = np.array(st[i13]['anchor_knots'][-1])
rm = chain.swing_stage('RM_WING_FORWARD_BODY_PRESHIFT', ps, pe, a, [5], [(rmx, a[5, 1])], a[5, 2] + .015, [0.45, 0.6, 0.45])
kr = np.array(rm['anchor_knots']); kr[1:3, 5, 1] -= .02          # swing slightly outward: away from the hip pocket
rm['anchor_knots'] = kr.tolist(); chain.base_from_world(rm)
st.insert(i13 + 1, rm)
chain.rebuild(st, i13 + 2)
i17 = [s['name'] for s in st].index('TRIPOD_LB_LF_RM_ADVANCE')
k17 = np.array(st[i17]['anchor_knots']); k17[1, 5, :2] = k17[0, 5, :2]; st[i17]['anchor_knots'] = k17.tolist()
out.setdefault('v11_edits', []).append(dict(step='REAR', change='LB relay x %.3f; insert RM wing forward to %.3f with body pre-shift to (%.3f, z %.3f, pitch %.2f) before RB climbs' % (lbx, rmx, bx, bz, bp)))
chain.validate(out); json.dump(out, open(sys.argv[2], 'w'), indent=1)
print('stages', len(st))
