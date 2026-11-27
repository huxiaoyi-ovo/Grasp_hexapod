import importlib.util,json,sys
from pathlib import Path
import numpy as np
ROOT=Path('/home/artrc/Grasp_hexapod');OUT=Path('/tmp/grasp_front_v10_pidudS');S=ROOT/'src/grasp_hexapod_control/scripts';sys.path[:0]=[str(S),str(S/'tools')]
from utils.climb_collision import ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,default_visual_scene
cfg=json.loads((ROOT/'src/grasp_hexapod_control/config/climb_front_optimized.json').read_text());cpu=json.loads((OUT/'c17_suffix_cpu.json').read_text());T=np.eye(4);T[:3,3]=cfg['xiaolan_translation'];scene=default_visual_scene(ROOT,world_from_xiaolan=T);assert np.array_equal(scene.world_from_xiaolan,T);assert np.allclose(scene.xiaolan_mesh.triangles,scene.xiaolan_mesh_local.triangles+T[:3,3],atol=1e-12)
sp=importlib.util.spec_from_file_location('h',ROOT/'src/docs/evidence/climb_front_fast_20260923/front_back_spacing_visual_mesh_audit.py');h=importlib.util.module_from_spec(sp);sp.loader.exec_module(h);h.PAIR_CACHE_PATH=OUT/'cad_cache.json';out=[]
def add(x):
 out.append(x)
 if x['hit']:Path(OUT/'cad.json').write_text(json.dumps({'status':'HIT','queries':out},indent=2)+'\n');raise SystemExit
for idx,legs,times in ((16,(0,1,5),(.65,1.75)),(17,(3,),(.65,1.75))):
 rows=[r for r in cpu['tick_samples'] if r['stage_index']==idx];st=cfg['stages'][idx];lo=st['segment_durations_s'][0];hi=sum(st['segment_durations_s'][:-1])
 for leg in legs:
  for t in times:
   r=min([x for x in rows if lo-1e-9<=x['phase_time_s']<=hi+1e-9],key=lambda x:abs(x['phase_time_s']-t));assert leg in st['active_legs']
   for link in ('knee','ankle'):
    c=h.mesh_components(scene,np.asarray(r['q_rad']),np.asarray(r['base_pose']),leg,link);assert c
    if link=='ankle':assert ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX in {z.component.component_index for z in c}
    z=scene.components_vs_xiaolan(c);add({'stage':st['name'],'leg':leg,'link':link,'t':r['phase_time_s'],'hit':bool(z.collision),'tri':None if z.hit is None else z.hit.__dict__})
assert len(out)==16
Path(OUT/'cad.json').write_text(json.dumps({'status':'CLEAR','queries':out,'note':'16 active-air exact queries; remaining neighbor budget omitted only because no additional changed-leg collision selector was needed'},indent=2)+'\n');print('CLEAR',len(out))
