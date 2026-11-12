import json,sys
from pathlib import Path
import numpy as np
ROOT=Path('/home/artrc/Grasp_hexapod');OUT=Path('/tmp/grasp_front_v10_pidudS');S=ROOT/'src/grasp_hexapod_control/scripts';sys.path.insert(0,str(S))
import run_sim
from isaacgym import gymapi,gymtorch
orig=run_sim._update_climb_metric;force_tensor=None;indices=None;mapping=None
def v(x):return None if x is None else np.asarray(x,dtype=float).tolist()
def obs(metric,controller,q,root,quat,dt):
 global force_tensor,indices,mapping
 frame=sys._getframe(1).f_locals
 if force_tensor is None:
  gym,sim,env,actor=frame['gym'],frame['sim'],frame['env'],frame['actor']
  names=gym.get_actor_rigid_body_names(env,actor);mapping={n:gym.find_actor_rigid_body_index(env,actor,n,gymapi.DOMAIN_SIM) for n in names if n.endswith('_foot_link')};need=['lb_foot_link','lf_foot_link','lm_foot_link','rb_foot_link','rf_foot_link','rm_foot_link']
  if any(mapping.get(n,-1)<0 for n in need):raise RuntimeError('foot net-force binding failed: '+repr(mapping))
  force_tensor=gymtorch.wrap_tensor(gym.acquire_net_contact_force_tensor(sim));indices=[mapping[n] for n in need]
 result=orig(metric,controller,q,root,quat,dt);gym=frame['gym'];sim=frame['sim'];gym.refresh_net_contact_force_tensor(sim);mode=controller.climb_mode
 row={'stage_index':int(mode.stage_index),'stage_name':mode.phase,'phase_time_s':float(mode.phase_time),'mode_state':mode.state,'actual_q_rad':v(np.asarray(q).reshape(6,3)),'actual_root_xyz':v(root),'actual_root_quaternion_xyzw':v(quat),'applied_planned_base_pose':v(mode.base_pose),'applied_mode_anchors_world':v(mode.anchors_world),'last_foot_errors_m':v(mode.last_foot_errors_m),'net_contact_force_world_N':v(force_tensor[indices].cpu().numpy()),'foot_force_mapping':mapping,'force_provenance':'Isaac net contact force, last physics substep; simulator diagnostic only'}
 with (OUT/'actual_samples.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
 return result
run_sim._update_climb_metric=obs;sys.argv=['run_sim.py','--climb-start','--climb-config',str(ROOT/'src/grasp_hexapod_control/config/climb_front_optimized.json'),'--climb-speed','1','--climb-joint-speed','1.8','--climb-metrics',str(OUT/'metrics.json')];run_sim.main()
