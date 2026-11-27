"""v11 step 2 (user item 1): before the last pair climbs, move the middle pair straight to its final top
footholds while the body advances; later stages no longer move the middle legs. Simulation-only edit."""
import copy, json, sys
import numpy as np
import chain
src = json.load(open(sys.argv[1])); out = copy.deepcopy(src); st = out['stages']
bx, bz, bp, apex = map(float, sys.argv[3:7])
lm_xy, rm_xy = (.40, .100), (.400, -.275)
i13 = [s['name'] for s in st].index('LB_GROUND_TO_155')
ps = np.array(st[i13]['pose_end']); pe = ps.copy(); pe[0], pe[2], pe[3] = bx, bz, bp
mid = chain.swing_stage('MID_PAIR_TO_FINAL_TOP', ps, pe, st[i13]['anchor_knots'][-1], [2, 5], [lm_xy, rm_xy], apex,
                        [0.55, 0.75, 0.45])
st.insert(i13 + 1, mid)
for name, keep in (('DIAGONAL_RF_LM_ADVANCE', [4]), ('TRIPOD_LB_LF_RM_ADVANCE', [0, 1])):
    s = st[[x['name'] for x in st].index(name)]
    s['active_legs'] = keep
    s.pop('active_base_velocities_m_s', None)
chain.rebuild(st, i13 + 2)
for name in ('DIAGONAL_RF_LM_ADVANCE', 'TRIPOD_LB_LF_RM_ADVANCE'):
    s = st[[x['name'] for x in st].index(name)]
    if s['anchor_curve'] == 'piecewise_base_quintic':
        chain.base_from_world(s)
s16 = st[[x['name'] for x in st].index('DIAGONAL_RF_LM_ADVANCE')]; s16['name'] = 'RF_FINAL_ADVANCE'
s17 = st[[x['name'] for x in st].index('TRIPOD_LB_LF_RM_ADVANCE')]; s17['name'] = 'LB_LF_FINAL_ADVANCE'
out.setdefault('v11_edits', []).append(dict(step='MID', change='insert MID_PAIR_TO_FINAL_TOP after C13: LM->%s RM->%s body x %.3f z %.3f pitch %.2f; C16 RF only, C17 LB+LF only' % (lm_xy, rm_xy, bx, bz, bp)))
fb = np.array(st[-1]['anchor_knots'][-1])
chain.validate(out)
json.dump(out, open(sys.argv[2], 'w'), indent=1)
print('stages', len(st), 'final LM/RM', np.round(fb[[2, 5]], 3).tolist())
