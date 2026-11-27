import json, sys
from pathlib import Path
import numpy as np
ROOT=Path('/home/artrc/Grasp_hexapod');OUT=Path('/tmp/grasp_front_v9_wpNdZp');SCRIPTS=ROOT/'src/grasp_hexapod_control/scripts';sys.path.insert(0,str(SCRIPTS))
import run_sim
original=run_sim._update_climb_metric
def values(x): return None if x is None else np.asarray(x,dtype=float).tolist()
def observed(metric,controller,q_current,root_position,root_quaternion_xyzw,dt):
 result=original(metric,controller,q_current,root_position,root_quaternion_xyzw,dt);mode=controller.climb_mode
 row={'stage_index':None if mode is None else int(mode.stage_index),'stage_name':None if mode is None else mode.phase,'phase_time_s':None if mode is None else float(mode.phase_time),'mode_state':None if mode is None else mode.state,'actual_q_rad':values(np.asarray(q_current).reshape(6,3)),'actual_root_xyz':values(root_position),'actual_root_quaternion_xyzw':values(root_quaternion_xyzw),'applied_planned_base_pose':None if mode is None else values(mode.base_pose),'applied_mode_anchors_world':None if mode is None else values(mode.anchors_world),'last_foot_errors_m':None if mode is None else values(mode.last_foot_errors_m),'last_phase_hold':None if mode is None else bool(mode.last_phase_hold),'settle_time_s':None if mode is None else float(mode.settle_time),'stage_elapsed_time_s':None if mode is None else float(mode.stage_elapsed_time),'dt_s':float(dt)}
 with (OUT/'actual_samples.jsonl').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
 return result
run_sim._update_climb_metric=observed
sys.argv=['run_sim.py','--climb-start','--climb-config',str(ROOT/'src/grasp_hexapod_control/config/climb_front_optimized.json'),'--climb-speed','1','--climb-joint-speed','1.8','--climb-metrics',str(OUT/'metrics.json')]
run_sim.main()
