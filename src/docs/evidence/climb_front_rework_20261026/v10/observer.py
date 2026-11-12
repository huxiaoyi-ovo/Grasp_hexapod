import json,sys
from pathlib import Path
import numpy as np
ROOT=Path('/home/artrc/Grasp_hexapod');OUT=Path('/tmp/grasp_front_v10_forces_valid_t41L0I');S=ROOT/'src/grasp_hexapod_control/scripts';sys.path.insert(0,str(S))
import run_sim
from isaacgym import gymapi,gymtorch
orig=run_sim._update_climb_metric;ft=None;idx=None;meta={};prepared=0
def val(x):return None if x is None else np.asarray(x,dtype=float).tolist()
def observed(metric,controller,q,root,quat,dt):
 global ft,idx,prepared,meta
 main=sys._getframe(1).f_locals;mode=controller.climb_mode
 if ft is None:
  assert main['control_frame']==0 and main['physics_frame']==0 and mode.stage_index==0 and mode.phase_time==0.0
  gym,sim,env,actor=main['gym'],main['sim'],main['env'],main['actor'];assert prepared==0
  gym.prepare_sim(sim);prepared=1
  raw=gym.acquire_net_contact_force_tensor(sim);ft=gymtorch.wrap_tensor(raw);assert tuple(ft.shape)==(gym.get_sim_rigid_body_count(sim),3)
  need=['lb_foot_link','lf_foot_link','lm_foot_link','rb_foot_link','rf_foot_link','rm_foot_link'];names=gym.get_actor_rigid_body_names(env,actor);mp={n:gym.find_actor_rigid_body_index(env,actor,n,gymapi.DOMAIN_SIM) for n in need};assert all(n in names and mp[n]>=0 for n in need)
  idx=[mp[n] for n in need];meta={'prepared_once':prepared,'mapping':mp,'sim_rigid_body_count':int(gym.get_sim_rigid_body_count(sim)),'actor_rigid_body_names':names,'collection':str(main.get('sim_params',None))}
 result=orig(metric,controller,q,root,quat,dt);main['gym'].refresh_net_contact_force_tensor(main['sim']);f=ft[idx].cpu().numpy();assert np.isfinite(f).all()
 row={'stage_index':int(mode.stage_index),'stage_name':mode.phase,'phase_time_s':float(mode.phase_time),'mode_state':mode.state,'actual_q_rad':val(np.asarray(q).reshape(6,3)),'actual_root_xyz':val(root),'actual_root_quaternion_xyzw':val(quat),'applied_planned_base_pose':val(mode.base_pose),'applied_mode_anchors_world':val(mode.anchors_world),'last_foot_errors_m':val(mode.last_foot_errors_m),'foot_net_contact_force_world_N':val(f),'force_metadata':meta,'force_boundary':'last control-observed physics step; simulator diagnostic only'}
 with (OUT/'actual_samples.jsonl').open('a') as h:h.write(json.dumps(row)+'\n')
 return result
run_sim._update_climb_metric=observed;sys.argv=['run_sim.py','--climb-start','--climb-config',str(ROOT/'src/grasp_hexapod_control/config/climb_front_optimized.json'),'--climb-speed','1','--climb-joint-speed','1.8','--climb-metrics',str(OUT/'metrics.json')];run_sim.main()
