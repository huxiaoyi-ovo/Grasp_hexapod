import json,sys
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
ROOT=Path('/home/artrc/Grasp_hexapod');S=ROOT/'src/grasp_hexapod_control/scripts';sys.path[:0]=[str(S),str(S/'tools')]
from kinematics import GraspKinematic,ESTIMATED_LEG_LINK_MASSES
from build_climb_front import world_from_base
CPU=Path('/tmp/grasp_front_v10_pidudS/c17_suffix_cpu.json');CFG=ROOT/'src/grasp_hexapod_control/config/climb_front_optimized.json';OUT=Path('/tmp/grasp_front_v10_pidudS')
kin=GraspKinematic();contacts=(2,4,3);normals=np.array([[0,0,1],[.0871027,-.0348995,.995588],[-.2079118,0,.9781476]]);normals/=np.linalg.norm(normals,axis=1)[:,None]
def tang(n):
 a=np.array([1.,0,0]);a-=a@n*n;a/=np.linalg.norm(a);return a,np.cross(n,a)
def own_g(q,R,leg):
 def V(x):return float(np.sum((R@kin.link_com_positions_base(x)[leg].T).T[:,2]*ESTIMATED_LEG_LINK_MASSES)*9.81)
 g=np.zeros(3);e=1e-5
 for j in range(3):
  qp=q.copy();qm=q.copy();qp[leg,j]+=e;qm[leg,j]-=e;g[j]=(V(qp)-V(qm))/(2*e)
 return g
def main():
 cpu=json.loads(CPU.read_text());cfg=json.loads(CFG.read_text());rows=[r for r in cpu['tick_samples'] if r['stage_index']==16];checks=[];out=[]
 for frac in np.linspace(0,1,5):
  row=min(rows,key=lambda r:abs(r['phase_time_s']-2.2*frac));q=np.asarray(row['q_rad']);pose=np.asarray(row['base_pose']);W=world_from_base(pose);R=W[:3,:3];anchors=np.asarray(cfg['stages'][16]['anchor_knots'][0]);Js=[]
  for leg in contacts:
   J=R@kin.base_from_hip[leg,:3,:3]@kin.jacobian(q)[leg];fd=np.empty((3,3));e=1e-6
   for j in range(3):
    qp=q.copy();qm=q.copy();qp[leg,j]+=e;qm[leg,j]-=e;fd[:,j]=(R@(kin.forward_base(qp)[leg]-kin.forward_base(qm)[leg]))/(2*e)
   checks.append({'fraction':float(frac),'leg':int(leg),'max_jacobian_error_m_per_rad':float(np.max(abs(J-fd)))});Js.append(J)
  Aeq=np.zeros((6,10));beq=np.r_[np.array([0,0,4.482*9.81]),np.zeros(3)];com=(W@np.r_[kin.center_of_mass_base(q),1])[:3];Aub=[];bub=[]
  for i,leg in enumerate(contacts):
   B=np.column_stack((normals[i],*tang(normals[i])));Aeq[:3,3*i:3*i+3]=B;d=anchors[leg]-com;Aeq[3:,3*i:3*i+3]=np.array([[0,-d[2],d[1]],[d[2],0,-d[0]],[-d[1],d[0],0]])@B
   for k in (1,2):
    for sign in (-1,1):
     a=np.zeros(10);a[3*i+k]=sign;a[3*i]=-.8/np.sqrt(2);Aub.append(a);bub.append(0)
   g=own_g(q,R,leg);J=Js[i]
   for j in range(3):
    jf=J[:,j]@B
    for sign in (-1,1):
     a=np.zeros(10);a[3*i:3*i+3]=-sign*jf;a[9]=-1.2;Aub.append(a);bub.append(-sign*g[j])
  for leg in set(range(6))-set(contacts):
   for gj in abs(own_g(q,R,leg)):
    a=np.zeros(10);a[9]=-1.2;Aub.append(a);bub.append(-gj)
  res=linprog(np.r_[np.zeros(9),1],A_ub=Aub,b_ub=bub,A_eq=Aeq,b_eq=beq,bounds=[(0,None),(None,None),(None,None)]*3+[(0,None)],method='highs')
  out.append({'fraction':float(frac),'success':bool(res.success),'t':None if not res.success else float(res.x[9]),'forces_local_n_t1_t2_N':None if not res.success else res.x[:9].reshape(3,3).tolist(),'forces_world_N':None if not res.success else [(np.column_stack((normals[i],*tang(normals[i])))@res.x[3*i:3*i+3]).tolist() for i in range(3)]})
 unit=linprog([1],A_ub=[[-1.2]],b_ub=[-.6],bounds=[(0,None)],method='highs')
 result={'jacobian_checks':checks,'max_jacobian_error':max(x['max_jacobian_error_m_per_rad'] for x in checks),'torque_sign_unit_min_t':float(unit.x[0]),'samples':out,'pass':max(x['max_jacobian_error_m_per_rad'] for x in checks)<1e-6 and abs(unit.x[0]-.5)<1e-9 and all(x['success'] and x['t']<=1 for x in out)};OUT.joinpath('stance_lp_corrected.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'pass':result['pass'],'max_jacobian_error':result['max_jacobian_error'],'unit_t':result['torque_sign_unit_min_t'],'t':[x['t'] for x in out]}))
if __name__=='__main__':main()
