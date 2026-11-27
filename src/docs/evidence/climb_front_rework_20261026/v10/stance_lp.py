import json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
ROOT=Path('/home/artrc/Grasp_hexapod');S=ROOT/'src/grasp_hexapod_control/scripts';sys.path[:0]=[str(S),str(S/'tools')]
from kinematics import GraspKinematic,ESTIMATED_LEG_LINK_MASSES,ESTIMATED_LEG_LINK_COMS,ESTIMATED_TOTAL_MASS
from build_climb_front import world_from_base
cpu=json.loads(Path('/tmp/grasp_front_v10_pidudS/c17_suffix_cpu.json').read_text());cfg=json.loads((ROOT/'src/grasp_hexapod_control/config/climb_front_optimized.json').read_text());rows=[r for r in cpu['tick_samples'] if r['stage_index']==16];kin=GraspKinematic();contacts=(2,4,3);normals=np.array([[0,0,1],[.0871027,-.0348995,.995588],[-.2079118,0,.9781476]]);normals/=np.linalg.norm(normals,axis=1)[:,None];out=[]
def tangents(n):
 a=np.array([1.,0,0]);a-=a@n*n;a/=np.linalg.norm(a);return a,np.cross(n,a)
def grav(q,pose,leg):
 R=world_from_base(pose)[:3,:3]
 def potential(x):return 4.482*9.81*0 # own only below
 def own(x):
  com=kin.link_com_positions_base(x)[leg];return float(np.sum((R@com.T).T[:,2]*ESTIMATED_LEG_LINK_MASSES)*9.81)
 g=np.zeros(3);e=1e-5
 for j in range(3):
  qp=q.copy();qm=q.copy();qp[leg,j]+=e;qm[leg,j]-=e;g[j]=(own(qp)-own(qm))/(2*e)
 return g
for f in np.linspace(0,1,5):
 r=min(rows,key=lambda x:abs(x['phase_time_s']-2.2*f));q=np.asarray(r['q_rad']);pose=np.asarray(r['base_pose']);_,anchors,_=None,None,None
 # C17 fixed anchors are config entry for legs LM/RF/RB.
 anchors=np.asarray(cfg['stages'][16]['anchor_knots'][0]);R=world_from_base(pose)[:3,:3];com=(world_from_base(pose)@np.r_[kin.center_of_mass_base(q),1])[:3];Aeq=np.zeros((6,10));beq=np.r_[np.array([0,0,4.482*9.81]),np.zeros(3)]
 for i,leg in enumerate(contacts):
  n=normals[i];t1,t2=tangents(n);B=np.column_stack((n,t1,t2));Aeq[:3,3*i:3*i+3]=B;Aeq[3:,3*i:3*i+3]=np.array([[0,-(anchors[leg]-com)[2],(anchors[leg]-com)[1]],[(anchors[leg]-com)[2],0,-(anchors[leg]-com)[0]],[-(anchors[leg]-com)[1],(anchors[leg]-com)[0],0]])@B
 Aub=[];bub=[]
 for i,leg in enumerate(contacts):
  # friction inner pyramid and effort envelope
  for s in (-1,1):
   a=np.zeros(10);a[3*i+1]=s;a[3*i]=-.8/np.sqrt(2);Aub.append(a);bub.append(0)
   a=np.zeros(10);a[3*i+2]=s;a[3*i]=-.8/np.sqrt(2);Aub.append(a);bub.append(0)
  J=R@kin.jacobian(q)[leg];g=grav(q,pose,leg)
  for j in range(3):
   a=np.zeros(10);a[3*i:3*i+3]=-J[:,j]@np.column_stack((normals[i],*tangents(normals[i])));a[9]=-1.2;Aub.append(a);bub.append(-g[j]);Aub.append(-a);bub.append(g[j])
 bounds=[]
 for i in range(3):bounds += [(0,None),(None,None),(None,None)]
 bounds += [(0,None)]
 res=linprog(np.r_[np.zeros(9),1],A_ub=Aub,b_ub=bub,A_eq=Aeq,b_eq=beq,bounds=bounds,method='highs');out.append({'fraction':float(f),'success':bool(res.success),'t':None if not res.success else float(res.x[9]),'forces':None if not res.success else res.x[:9].reshape(3,3).tolist()})
Path('/tmp/grasp_front_v10_pidudS/stance_lp.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
