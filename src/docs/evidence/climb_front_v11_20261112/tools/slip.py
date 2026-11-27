import json, sys, numpy as np
L=('lb','lf','lm','rb','rf','rm')
def slips(mf, cf):
    m=json.load(open(mf)); ps=m['per_stage']; st=json.load(open(cf))['stages']; out=[]
    for i in range(len(ps)-1):
        a0=np.array(ps[i]['stage_start_anchor_world_xyz_m']); f0=np.array(ps[i]['stage_start_actual_world_foot_xyz_m'])
        a1=np.array(ps[i+1]['stage_start_anchor_world_xyz_m']); f1=np.array(ps[i+1]['stage_start_actual_world_foot_xyz_m'])
        sup=[l for l in range(6) if l not in st[i]['active_legs']]
        out.append((st[i]['name'], float(((f1-a1)-(f0-a0))[sup,0].mean()*1000)))
    return out, ps[-1]['end_root_position_error_m']*1000
if __name__=='__main__':
    for mf,cf in zip(sys.argv[1::2], sys.argv[2::2]):
        s,r=slips(mf,cf); print(mf.split('/')[-1], 'final root err %.1f mm, total mean slip %.1f mm'%(r,sum(x for _,x in s)))
        print('   ', ' '.join(f'{n[:10]}:{v:+.1f}' for n,v in s if abs(v)>1.5))
