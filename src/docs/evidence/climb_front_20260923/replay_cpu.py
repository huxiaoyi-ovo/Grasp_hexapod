import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];sys.path[:0]=[str(ROOT/'src/grasp_hexapod_control/scripts'),str(ROOT/'src/grasp_hexapod_control/scripts/tools')]
from control import GraspController
from climb_mode import ClimbMode
from kinematics import Q_STAND,JOINT_LOWER,JOINT_UPPER
from build_climb_front import (world_from_base,support_margin,foot_sphere_signed_gap,load_triangles,TRANSLATION)
cfg=json.load(open(ROOT/'src/grasp_hexapod_control/config/climb_front.json'));names=('lb','lf','lm','rb','rf','rm');tri=load_triangles(ROOT/'src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL')+TRANSLATION;ctrl=GraspController(1/30,False,False);kin=ctrl.kinematic;ctrl.enter_climb(Q_STAND,cfg,0,len(cfg['stages'])-1);q=Q_STAND.copy();tail_start=next(i for i,s in enumerate(cfg['stages']) if s['name']=='R_FRONT_40_SYNC');out={'entry':'P0/Q_STAND','tail_start_index':tail_start,'stages':{},'first_failure':None};last=None;entry=None;ticks=0
while ctrl.climb_mode.state==ClimbMode.RUNNING and ticks<10000:
 idx=ctrl.climb_mode.stage_index; st=cfg['stages'][idx];name=st['name'];active=np.asarray(st['active_legs'],int);fixed=np.array([i for i in range(6) if i not in active]);tail=idx>=tail_start
 if name!=last:
  last=name;entry=q.copy();out['stages'][name]={'index':idx,'tail_gate':tail,'max_foot_error_m':0.,'min_joint_margin_rad':1e9,'min_support_margin_m':1e9,'max_fixed_thigh_delta_rad':0.,'raw_peak_rad_s':0.,'raw_unclipped_peak_rad_s':0.,'raw_unclipped_source':None,'clips':0,'min_sphere_gap_m':1e9,'tail_link_clear':True}
 old=q.copy(); prev_ref=ctrl.foot_desired_base_prev.copy();q=ctrl.update(q,np.zeros(4));ticks+=1; hip_des=kin.base_to_hip(ctrl.foot_desired_base); hip_prev=kin.base_to_hip(prev_ref); hip_actual_before=kin.forward(old); pinv=kin.damped_inverse_jacobian(old); raw=16.0*np.einsum('lij,lj->li',pinv,hip_des-hip_actual_before)+np.einsum('lij,lj->li',pinv,(hip_des-hip_prev)*30.0); raw_idx=np.unravel_index(np.argmax(abs(raw)),raw.shape);row=out['stages'][name];actual=kin.hip_to_base(kin.forward(q));err=np.linalg.norm(actual-ctrl.foot_desired_base,axis=1);row['max_foot_error_m']=max(row['max_foot_error_m'],float(err.max())); raw_value=float(abs(raw[raw_idx]));
 if raw_value>row['raw_unclipped_peak_rad_s']: row['raw_unclipped_peak_rad_s']=raw_value;row['raw_unclipped_source']={'leg':int(raw_idx[0]),'joint':int(raw_idx[1]),'value_rad_s':raw_value}
 m=np.minimum(q-JOINT_LOWER,JOINT_UPPER-q);row['min_joint_margin_rad']=min(row['min_joint_margin_rad'],float(m.min()));speed=float(abs(q-old).max()*30);row['raw_peak_rad_s']=max(row['raw_peak_rad_s'],speed);row['clips']+=int(ctrl.last_update_velocity_limit_clip_count);p=ctrl.climb_mode.base_pose;a=ctrl.climb_mode.anchors_world;com=(world_from_base(p)@np.r_[kin.center_of_mass_base(q),1])[:3];row['min_support_margin_m']=min(row['min_support_margin_m'],support_margin(com[:2],a[fixed]))
 if tail:
  row['max_fixed_thigh_delta_rad']=max(row['max_fixed_thigh_delta_rad'],float(abs(q[fixed,0]-entry[fixed,0]).max()))
  for leg in active:row['min_sphere_gap_m']=min(row['min_sphere_gap_m'],foot_sphere_signed_gap(tri,a[leg]))
  row['tail_link_clear'] &= bool(ctrl._link_collision_free(q).all())
 # Existing prefix keeps the agreed broad 15mm diagnostic gate; frozen tail uses tight gates.
 bad=(row['min_joint_margin_rad']<=0 or row['clips'] or row['max_foot_error_m']>.015)
 if tail:bad |= (row['max_foot_error_m']>.010 or row['min_support_margin_m']<.010 or row['max_fixed_thigh_delta_rad']>.08 or row['raw_peak_rad_s']>3.8 or row['min_sphere_gap_m']<-.0005 or not row['tail_link_clear'])
 if bad:
  out['first_failure']={'stage':name,'stage_time_s':float(ctrl.climb_mode.phase_time),'row':row,'q_rad':q.tolist(),'foot_error_m':err.tolist(),'anchors_world_m':a.tolist()};break
out['ticks']=ticks;out['state']=ctrl.climb_mode.state;out['terminal_q_rad']=q.tolist();out['terminal_pose']=ctrl.climb_mode.base_pose.tolist();out['final_anchors_world_m']=ctrl.climb_mode.anchors_world.tolist();Path(ROOT/'src/docs/evidence/climb_front_20260923/cpu_replay.json').write_text(json.dumps(out,indent=2));print(json.dumps({'state':out['state'],'ticks':ticks,'first_failure':out['first_failure'],'last':last,'last_row':out['stages'].get(last)},indent=2))
