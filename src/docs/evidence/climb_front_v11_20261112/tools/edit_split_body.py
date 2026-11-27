"""RB stays outward (.212,-.225) until its final move; split the final body advance so RB is not dragged to
full extension: TRIPOD ends at body x=bx, RB_FINAL_ADVANCE carries the body bx -> .45 while RB swings."""
import json, sys
import numpy as np
import chain
out = json.load(open(sys.argv[1])); st = out['stages']; names = [s['name'] for s in st]
bx = float(sys.argv[3])
tri = st[names.index('TRIPOD_LB_LF_RM_ADVANCE')]
final_pose = np.array(tri['pose_end'])
pk = np.array(tri['pose_knots']); pk[pk[:, 0] > bx, 0] = bx; tri['pose_knots'] = pk.tolist()
pe = final_pose.copy(); pe[0] = bx; tri['pose_end'] = pe.tolist()
if tri['anchor_curve'] == 'piecewise_base_quintic':
    chain.base_from_world(tri, keep_velocities=True)
rb = st[names.index('RB_FINAL_ADVANCE')]
rb['pose_end'] = final_pose.tolist()
chain.rebuild(st, names.index('RB_FINAL_ADVANCE'))
rb['pose_end'] = final_pose.tolist()
chain.rebuild(st, names.index('RB_FINAL_ADVANCE') + 1)
out['v11_edits'].append(dict(step='SPLIT', change='TRIPOD body end x %.3f; RB_FINAL carries body to .45' % bx))
chain.validate(out); json.dump(out, open(sys.argv[2], 'w'), indent=1)
print([ (s['name'], round(s['pose_start'][0],3), round(s['pose_end'][0],3)) for s in st[-4:]])
