#!/bin/bash
# usage: cpu.sh cfg.json out.json [nrows]
python3 -c "
import sys; sys.path.insert(0,'/home/artrc/Grasp_hexapod/src/docs/evidence/climb_front_direct_20260924')
import cpu_direct_audit; from pathlib import Path
cpu_direct_audit.run(Path('$1').resolve(), Path('$2').resolve())" > /dev/null 2>&1
python3 -c "
import json,numpy as np,sys
sys.path[:0]=['/home/artrc/Grasp_hexapod/src/grasp_hexapod_control/scripts']
from kinematics import JOINT_LOWER,JOINT_UPPER
r=json.load(open('$2'));f=r['first_failure'];print('CPU',r['state'],r['ticks'],f and (f['stage'],f['reason'],round(f['stage_time_s'],2)))
if f and f['reason']=='joint_limit':
  q=np.array(f['q_rad']);m=np.minimum(q-JOINT_LOWER,JOINT_UPPER-q);i,j=np.unravel_index(np.argmin(m),m.shape);print('  limit',('lb','lf','lm','rb','rf','rm')[i],('thigh','knee','ankle')[j])
for k,v in list(r['stage_rows'].items())[:${3:-30}]:
  c=v['min_com_margin_m']; g=v['min_active_sphere_gap_m']; print(f\"  {k[:24]:24s} COM {('%.1f'%(c*1000)) if c is not None else '-':>6s} mm  margin {v['min_joint_margin_rad']:.3f}  fk {v['max_fk_error_m']*1000:4.1f} mm  gap {('%.1f'%(g*1000)) if g is not None else '-':>6s} mm  clips {v['velocity_clips']} capsule {'WARN' if v['capsule_proxy_first_warning'] else '-'}\")"
