"""v11 step 2b: C14 RB / C15 LB high arcs: apex knots 30 mm lower and 10 mm further outward (C14 body-cylinder
proxy warning), C15 lift segment 1.1 -> 1.5 s (LB now starts from x=.14)."""
import copy, json, sys
import numpy as np
import chain
out = json.load(open(sys.argv[1])); st = out['stages']; names = [s['name'] for s in st]
for name, leg, out_sign in (('RB_DIRECT_PLATFORM_225', 3, -1), ('LB_PLATFORM_225', 0, 1)):
    s = st[names.index(name)]; k = np.array(s['anchor_knots'])
    k[1:3, leg, 2] -= .030; k[1:3, leg, 1] += out_sign * .010
    s['anchor_knots'] = k.tolist()
    if name.startswith('LB'):
        d = list(s['segment_durations_s']); d[0] = 1.5; s['segment_durations_s'] = d
    chain.base_from_world(s, keep_velocities=True)
out['v11_edits'].append(dict(step='REAR2', change='C14/C15 apex knots -30 mm and 10 mm outward; C15 lift 1.5 s'))
chain.validate(out); json.dump(out, open(sys.argv[2], 'w'), indent=1)
