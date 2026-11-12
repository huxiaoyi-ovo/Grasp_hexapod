"""Hardware-speed retime: replay the full chain through GraspController with the shared 4 rad/s step cap
(hardware_execution semantics proxied by setting the front cap to 4.0 and the full_mission foot gate), record
each stage's peak raw (pre-clip) joint rate, and stretch only the stages that exceed `cap*margin`.
Iterates until a replay finishes with zero clips. CPU controller model only; not servo/load evidence."""
import copy, json, sys
from pathlib import Path
import numpy as np
REPO = Path('/home/artrc/Grasp_hexapod')
sys.path[:0] = [str(REPO / 'src/grasp_hexapod_control/scripts'), str(REPO / 'src/grasp_hexapod_control/scripts/tools')]
from control import GraspController  # noqa: E402
from kinematics import Q_STAND  # noqa: E402
import chain  # noqa: E402

DT = 1 / 30


def replay(cfg):
    c = GraspController(DT, enable_link_collision_check=False, climb_timeout_uses_wall_time=False)
    c.enter_climb(Q_STAND, cfg, 0, len(cfg['stages']) - 1, hardware_execution=False)
    m = c.climb_mode; q = Q_STAND.copy(); peak = {}; clips = {}; fk = {}; tick = 0
    while m.state == m.RUNNING and tick < 30000:
        name = cfg['stages'][m.stage_index]['name']
        prev_ref = c.foot_desired_base_prev.copy(); old = q.copy()
        q = c.update(q, np.zeros(4)); tick += 1
        inv = c.kinematic.damped_inverse_jacobian(old)
        raw = 16.0 * np.einsum('lij,lj->li', inv, c.kinematic.base_to_hip(c.foot_desired_base) - c.kinematic.forward(old)) \
            + np.einsum('lij,lj->li', inv, (c.kinematic.base_to_hip(c.foot_desired_base) - c.kinematic.base_to_hip(prev_ref)) / DT)
        peak[name] = max(peak.get(name, 0), float(np.abs(raw).max()))
        clips[name] = clips.get(name, 0) + int(c.last_update_velocity_limit_clip_count)
        e = np.linalg.norm(c.kinematic.hip_to_base(c.kinematic.forward(q)) - c.foot_desired_base, axis=1).max()
        fk[name] = max(fk.get(name, 0), float(e))
    return m.state, tick, peak, clips, fk


def main(src, dst, cap=4.0, margin=.9):
    cfg = json.load(open(src)); cfg = copy.deepcopy(cfg)
    cfg['simulation_joint_velocity_limit_rad_s'] = cap
    for it in range(8):
        state, tick, peak, clips, fk = replay(cfg)
        n = sum(clips.values())
        print(f'iter {it}: state {state} ticks {tick} ({tick / 30:.1f} s) clips {n} max fk {max(fk.values()) * 1000:.1f} mm')
        stretched = []
        for s in cfg['stages']:
            p = peak.get(s['name'], 0)
            if p > cap * margin:
                f = p / (cap * margin)
                s['segment_durations_s'] = (np.array(s['segment_durations_s']) * f).tolist()
                if s['anchor_curve'] == 'piecewise_base_quintic':
                    chain.base_from_world(s, keep_velocities=True)
                stretched.append((s['name'], round(f, 2)))
        if not stretched and n == 0 and state == 'DONE':
            break
        print('   stretched', stretched)
    cfg['simulation_joint_velocity_limit_rad_s'] = json.load(open(src))['simulation_joint_velocity_limit_rad_s']
    cfg.setdefault('v11_edits', []).append(dict(step='HW_RETIME', change='stages stretched until a %.1f rad/s step-cap replay has zero clips (peak raw <= %.0f%% of cap)' % (cap, margin * 100)))
    chain.validate(cfg); json.dump(cfg, open(dst, 'w'), indent=1)
    print('plan total %.1f s' % sum(sum(s['segment_durations_s']) for s in cfg['stages']))
    for s in cfg['stages']:
        print(f"  {s['name'][:26]:26s} peak {peak.get(s['name'], 0):4.2f} rad/s  fk {fk.get(s['name'], 0) * 1000:5.1f} mm")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
