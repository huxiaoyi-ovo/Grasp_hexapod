"""v11 step 1: swap v10 C2/C3 so the middle pair advances first, then land the front pair at the
centre of the 12 deg first-landing face (x~.24) instead of its lower edge (x=.190).
C4 starts from the new front anchors; its end state and every stage from C5 on stay byte-identical.
Simulation-only planning edit; geometry/COM numbers are model diagnostics."""
import copy, hashlib, json, sys
from pathlib import Path
import numpy as np

REPO = Path('/home/artrc/Grasp_hexapod')
sys.path[:0] = [str(REPO / 'src/grasp_hexapod_control/scripts'), str(REPO / 'src/grasp_hexapod_control/scripts/tools')]
from build_climb_front import MESH, TRANSLATION, base_points, load_triangles, surface_height, world_from_base  # noqa
from kinematics import GraspKinematic  # noqa
from climb_mode import ClimbMode  # noqa
SRC = REPO / 'src/grasp_hexapod_control/config/climb_front_optimized.json'
OUT = Path(__file__).with_name('v11_c2.json')
K = GraspKinematic()
TRI = load_triangles(MESH)


def smooth(t):
    return t ** 3 * (10 - 15 * t + 6 * t * t)


def base_stage(name, ps, pe, knots_w, active, durs, cap=.040):
    """piecewise_base_quintic stage; base knots = world knots in the body pose at each knot time."""
    knots_w = np.asarray(knots_w, float)
    tk = np.r_[0, np.cumsum(durs)] / sum(durs)
    poses = [ps * (1 - smooth(t)) + pe * smooth(t) for t in tk]
    base = np.array([base_points(p, k[active]) for p, k in zip(poses, knots_w)])
    times = np.r_[0, np.cumsum(durs)]
    vel = np.zeros_like(base)
    for i in range(1, len(base) - 1):
        v = (base[i + 1] - base[i - 1]) / (times[i + 1] - times[i - 1])
        n = np.linalg.norm(v, axis=1, keepdims=True)
        vel[i] = v * np.minimum(1.0, cap / np.maximum(n, 1e-15))
    return dict(name=name, pose_start=ps.tolist(), pose_end=pe.tolist(), active_legs=list(active),
                anchor_knots=knots_w.tolist(), segment_durations_s=list(map(float, durs)),
                anchor_curve='piecewise_base_quintic', pose_curve='quintic_full_stage', settle_s=.15,
                active_base_knots_m=base.tolist(), active_base_velocities_m_s=vel.tolist())


def com_margin(pose, feet, support):
    """Model COM margin (m) at a static pose, q from DLS-free closed form via kinematics IK fallback."""
    from build_climb_front import solve_ik, support_margin
    q = solve_ik(K, pose, feet, np.array(json.loads(SRC.read_text())['p0']['q_rad']))
    q = q[0] if isinstance(q, tuple) else q
    com = (world_from_base(pose) @ np.r_[K.center_of_mass_base(q), 1])[:2]
    return support_margin(com, feet[support]), q


def main(lf_x=.242, lf_y=.100, rf_x=.212, rf_y=-.1647, body_x=-.05, apex=.19, lift_s=1.4, fwd=.05, z1=.17):
    src = json.loads(SRC.read_text())
    st = src['stages']
    c1, c2, c3, c4 = st[0], st[1], st[2], st[3]
    pose1 = np.array(c1['pose_end'])                        # (-.12, y, .15, .15)
    a0 = np.array(c2['anchor_knots'][0])                     # P0 feet
    # --- new C2: middle pair to x=-.020 (old C3 world swing), body -.12 -> body_x, attitude kept
    poseA = pose1.copy(); poseA[0] = body_x
    kA = np.array(c3['anchor_knots']); kA[:, [1, 4]] = a0[[1, 4]]; kA[:, [0, 3]] = a0[[0, 3]]
    mid = copy.deepcopy(c3); mid['name'] = 'MID_GROUND_FIRST'
    mid['pose_start'], mid['pose_end'], mid['anchor_knots'] = pose1.tolist(), poseA.tolist(), kA.tolist()
    # --- new C3: front pair from P0 to the face centre, body fixed
    endA = kA[-1]
    tgt = np.array([[x, y, surface_height(TRI, x, y)] for x, y in ((lf_x, lf_y), (rf_x, rf_y))])
    k = np.repeat(endA[None], 4, 0)
    k[1, [1, 4]] = endA[[1, 4]] + [fwd, 0, z1 - endA[1, 2]]     # rise while moving forward (avoid hip pocket)
    k[2, [1, 4]] = np.c_[tgt[:, :2], np.full(2, apex)]
    k[3, [1, 4]] = tgt
    front = base_stage('FRONT_TO_FACE_CENTRE', poseA, poseA, k, [1, 4], [lift_s, 1.0, .30])
    # --- C4: same end, start from the new front anchors
    k4 = np.array(c4['anchor_knots']); k4[0, [1, 4]] = tgt
    k4[1, [1, 4]] = np.c_[tgt[:, :2], np.array(c4['anchor_knots'])[1, [1, 4], 2]]
    for kk in k4:
        kk[[2, 5]] = endA[[2, 5]]
    pe4 = np.array(c4['pose_end'])
    c4n = base_stage(c4['name'], poseA, pe4, k4, [1, 4], c4['segment_durations_s'])
    assert np.allclose(k4[-1], np.array(c4['anchor_knots'])[-1])
    out = copy.deepcopy(src)
    out['stages'] = [c1, mid, front, c4n] + st[4:]
    out['v11_edits'] = [dict(step='C2', change='mid pair first; front pair lands at planar face centres LF(%.3f,%.3f) RF(%.3f,%.3f); C4 start follows' % (lf_x, lf_y, rf_x, rf_y),
                             source_sha256=hashlib.sha256(SRC.read_bytes()).hexdigest())]
    ClimbMode(None)._validate_config(out)
    # diagnostics: COM for the front lift (support mids+rears) and C4 lift
    m3, _ = com_margin(poseA, endA, [0, 2, 3, 5])
    print('front-lift COM margin at body x=%.3f: %.1f mm' % (body_x, m3 * 1000))
    print('front target', np.round(tgt, 4).tolist())
    OUT.write_text(json.dumps(out, indent=1) + '\n')
    print('wrote', OUT, hashlib.sha256(OUT.read_bytes()).hexdigest()[:12])


if __name__ == '__main__':
    main(*map(float, sys.argv[1:]))
