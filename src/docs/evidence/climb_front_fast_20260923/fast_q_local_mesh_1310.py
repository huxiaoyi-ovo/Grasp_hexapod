import hashlib, json, sys
from pathlib import Path
import numpy as np
ROOT=Path('/home/artrc/Grasp_hexapod')
EVIDENCE=ROOT/'src/docs/evidence/climb_front_fast_20260923'
sys.path.insert(0,str(EVIDENCE))
import front_back_spacing_visual_mesh_audit as mesh_audit
from utils.climb_collision import default_visual_scene
mesh_audit.scene=default_visual_scene(ROOT)
config_path=ROOT/'src/grasp_hexapod_control/config/climb_front_fast.json'
audit_path=EVIDENCE/'cpu_fast_41_audit.json'
out_path=EVIDENCE/'cpu_fast_q_local_mesh_1310.json'
config=json.loads(config_path.read_text()); audit=json.loads(audit_path.read_text())
q_witness=audit['stage_rows']['Q_MID_LOW']['capsule_witness']
q=np.asarray(q_witness['q_rad']); pose=np.asarray(q_witness['base_pose'])
record={'label':'fast_Q_MID_LOW_capsule_witness','phase_time_s':q_witness['phase_time_s'],'q_rad':q.tolist(),'base_pose':pose.tolist(),'pair_checks':[]}
for leg in ('lm','rm'):
 for link in ('knee','ankle'):
  result=mesh_audit.check_pair(mesh_audit.scene,q,pose,{'leg':None,'link':'body'},{'leg':leg,'link':link})
  record['pair_checks'].append(result)
  out={'candidate_config_sha256':hashlib.sha256(config_path.read_bytes()).hexdigest(),'audit_sha256':hashlib.sha256(audit_path.read_bytes()).hexdigest(),'records':[record],'boundary':'Discrete sampled visual-mesh triangle intersection only. Containment and continuous clearance are not evaluated; no contact/load/stability/hardware claim.'}
  out_path.write_text(json.dumps(out,indent=2)+'\n')
  print(json.dumps({'completed_pair':result,'phase_time_s':record['phase_time_s'],'leg':leg,'link':link},indent=2),flush=True)
  if result['mesh_collision']:
   print('STOP: triangle collision; remaining pairs not queried.',flush=True)
   raise SystemExit(3)
worst=mesh_audit.local_stage_replay(config,audit,'Q_MID_LOW')
worst_pair={'leg':worst['leg'],'link':worst['link']}
if abs(worst['phase_time_s']-record['phase_time_s'])>1e-9 or worst_pair not in [{'leg':c['pair'][1]['leg'],'link':c['pair'][1]['link']} for c in record['pair_checks']]:
 result=mesh_audit.check_pair(mesh_audit.scene,np.asarray(worst['q_rad']),np.asarray(worst['base_pose']),{'leg':None,'link':'body'},worst_pair)
 record2={'label':'fast_Q_MID_LOW_worst_body_proxy_gap','phase_time_s':worst['phase_time_s'],'q_rad':worst['q_rad'],'base_pose':worst['base_pose'],'worst_proxy_pair':worst_pair,'proxy_gap_m':worst['proxy_gap_m'],'pair_checks':[result]}
 out['records'].append(record2); out_path.write_text(json.dumps(out,indent=2)+'\n')
 print(json.dumps({'completed_worst_pair':record2},indent=2),flush=True)
 if result['mesh_collision']:
  print('STOP: triangle collision; remaining batch not queried.',flush=True); raise SystemExit(3)
else:
 out['worst_sample_reuses_initial_witness']=True
 out['worst_sample']={'phase_time_s':worst['phase_time_s'],'pair':worst_pair,'proxy_gap_m':worst['proxy_gap_m']}
 out_path.write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
