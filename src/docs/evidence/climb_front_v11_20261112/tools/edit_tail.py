"""v11 step 2 (user item 1): rebuild the tail after FRONT_DIRECT_FINAL_480.
Order: LB ground relay -> RB ground relay -> six-support body raise/advance -> middle pair straight to final
top footholds -> RB straight to final -> LB straight to final -> body level/advance with front pair to
final -> hold. Simulation-only planning; all numbers are model diagnostics."""
import copy, json, sys
import numpy as np
import chain
from chain import surface_height, TRI

YAW = -np.pi / 2
P = dict(rear_ground_x=.14, body1=(.33, -.068, .24, .45), apex_lm=.228, apex_rm=.215, mid_fwd=.03,
         rb_final=(.335, -.165), lb_final=(.28, .105), lm_final=(.40, .100), rm_final=(.400, -.275),
         lf_final=(.555, .085), rf_final=(.618, -.160), body_final=(.45, -.063, .30, 0.0))


def pose(x, y, z, p):
    return np.array((x, y, z, p, 0., YAW))


def body_stage(name, ps, pe, anchors, T):
    a = np.array(anchors, float)
    return dict(name=name, pose_start=list(map(float, ps)), pose_end=list(map(float, pe)), active_legs=[],
                anchor_knots=[a.tolist(), a.tolist()], segment_durations_s=[float(T)],
                anchor_curve='piecewise_quintic', pose_curve='quintic_full_stage', settle_s=.15)


def ground_relay(name, ps, anchors, leg, x, T=(.5, .7, .45)):
    a = np.array(anchors, float); k = np.repeat(a[None], 4, 0)
    k[1, leg, 2] = a[leg, 2] + .020
    k[2, leg] = [x, a[leg, 1], a[leg, 2] + .020]
    k[3, leg] = [x, a[leg, 1], a[leg, 2]]
    s = dict(name=name, pose_start=list(map(float, ps)), pose_end=list(map(float, ps)), active_legs=[leg],
             anchor_knots=k.tolist(), segment_durations_s=list(T), anchor_curve='piecewise_quintic',
             pose_curve='quintic_full_stage', settle_s=.15)
    return s


def high_step(name, ps, pe, anchors, legs, targets, apex, out_y, T):
    """Lift up and laterally outward (away from the hip pocket), cross high, come in, land along +z."""
    a = np.array(anchors, float)
    tg = np.array([[x, y, surface_height(TRI, x, y)] for x, y in targets])
    k = np.repeat(a[None], 5, 0)
    for j, l in enumerate(legs):
        side = np.sign(a[l, 1] - ps[1])
        k[1, l] = [a[l, 0] + .02, a[l, 1] + side * out_y, apex]
        k[2, l] = [tg[j, 0], tg[j, 1] + side * out_y, apex]
        k[3, l] = [tg[j, 0], tg[j, 1], tg[j, 2] + .030]
        k[4, l] = tg[j]
    s = dict(name=name, pose_start=list(map(float, ps)), pose_end=list(map(float, pe)), active_legs=list(legs),
             anchor_knots=k.tolist(), segment_durations_s=list(map(float, T)),
             anchor_curve='piecewise_base_quintic', pose_curve='quintic_full_stage', settle_s=.15)
    chain.base_from_world(s)
    return s


def main(src_path, out_path, **kw):
    P.update(kw)
    src = json.load(open(src_path)); out = copy.deepcopy(src); st = out['stages']
    i12 = [s['name'] for s in st].index('FRONT_DIRECT_FINAL_480')
    head = st[:i12 + 1]
    p = np.array(head[-1]['pose_end']); a = np.array(head[-1]['anchor_knots'][-1])
    tail = []

    def add(s):
        nonlocal p, a
        tail.append(s); p = np.array(s['pose_end']); a = np.array(s['anchor_knots'][-1])
    add(ground_relay('LB_GROUND_RELAY', p, a, 0, P['rear_ground_x']))
    add(ground_relay('RB_GROUND_RELAY', p, a, 3, P['rear_ground_x']))
    add(body_stage('BODY_RAISE_ADVANCE', p, pose(*P['body1']), a, 1.2))
    add(chain.swing_stage('MID_PAIR_TO_FINAL_TOP', p, p, a, [2, 5], [P['lm_final'], P['rm_final']],
                          [P['apex_lm'], P['apex_rm']], [0.55, 0.75, 0.45], lift_fwd=P['mid_fwd']))
    add(high_step('RB_TO_FINAL', p, p, a, [3], [P['rb_final']], .27, .05, [1.0, .9, .5, .45]))
    add(high_step('LB_TO_FINAL', p, p, a, [0], [P['lb_final']], .27, .05, [1.0, .9, .5, .45]))
    pf = pose(*P['body_final'])
    add(body_stage('BODY_TO_CENTRE', p, pf, a, 1.5))
    add(chain.swing_stage('RF_TO_FINAL', p, p, a, [4], [P['rf_final']], .226, [0.5, 0.8, 0.45]))
    add(chain.swing_stage('LF_TO_FINAL', p, p, a, [1], [P['lf_final']], .226, [0.5, 0.8, 0.45]))
    add(body_stage('STAND_FINAL_HOLD', p, p, a, 2.0))
    out['stages'] = head + tail
    out['v11_edits'] = out.get('v11_edits', []) + [dict(step='TAIL', change='rear ground relays to x=%.3f, body raise to %s, mids/rears straight to finals, then centre' % (P['rear_ground_x'], P['body1']))]
    # terminal q: from the final anchors at the final pose (validator only checks shape)
    chain.validate(out)
    json.dump(out, open(out_path, 'w'), indent=1)
    print('stages', len(out['stages']), 'tail', [s['name'] for s in tail])


if __name__ == '__main__':
    kw = {}
    for arg in sys.argv[3:]:
        k, v = arg.split('=')
        kw[k] = tuple(map(float, v.split(','))) if ',' in v else float(v)
    main(sys.argv[1], sys.argv[2], **kw)
