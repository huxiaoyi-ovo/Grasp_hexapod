"""Offline-only fixture regeneration from the frozen Python implementation."""
import sys,json,cv2,numpy as np
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
sys.path.insert(0,str(root/'test/reference'))
import dock_mode_python as d
d.rospy.logwarn_throttle=lambda *a,**k: None
out=root/'test/ring_fixtures';out.mkdir(exist_ok=True)
images=[]
inputs=[('full',d._synthetic_ring_image()),('offset',d._synthetic_ring_image(center=(610,390),outer_diameter=300)),('cropped',d._synthetic_ring_image(center=(610,390),outer_diameter=300)[:,:625]),('three',d._synthetic_ring_image(center=(1600,360),outer_diameter=900)),('blank',np.full((300,300),255,np.uint8))]
for i in range(3):
 p=out/('real_%d.png'%i)
 if p.exists(): inputs.append(('real_%d'%i,cv2.imread(str(p),0)))
for name,image in inputs:
 detected=d.detect_concentric_rings(image);name+='.png';cv2.imwrite(str(out/name),image)
 item=dict(file=name,valid=detected.valid)
 if detected.valid:item.update(center=detected.center_px.tolist(),diameter=detected.major_diameter_px,matched=detected.matched_boundaries)
 images.append(item)
class Controller:
 dt=.1
 def __init__(self):
  self.feet=np.zeros((6,3));self.feet[:,2]=np.linspace(-.07,-.05,6)
  self.kinematic=type('K',(),{'forward_base':lambda _,q:self.feet.copy()})()
 def _sync_actual_feet(self,q):return self.feet.copy()
 @staticmethod
 def _smooth_step(x):return 10*x**3-15*x**4+6*x**5
class Perception:
 def reset(self):pass
 def latest(self):return d.PerceptionResult(reason='lost ring')
scenarios=[]
for confirmation in [False,True]:
 c=Controller();mode=d.DockMode(c,Perception(),require_lock_confirmation=confirmation)
 initial=c.feet.copy();q=np.zeros((6,3));mode.enter(q,np.ones((6,3)))
 frames=[]
 for i in range(250):
  pose=[.025,-.01,-.03] if i<30 else [0.,0.,-.03]
  if 15<=i<23:pose=None
  state={'joints':q,'lock_confirmed':i<10 or i>120}
  if pose is not None:state['lock_from_pin']=d.transform(pose)
  r=mode.update(state)
  frames.append(dict(pose=pose,confirmed=state['lock_confirmed'],state=r.state,feet=None if r.foot_positions_base is None else r.foot_positions_base.tolist(),joints=None if r.joint_positions is None else r.joint_positions.tolist(),request_lock=r.request_lock))
  if r.foot_positions_base is not None:c.feet=r.foot_positions_base.copy()
  if r.success:break
 assert mode.state==mode.SUCCESS
 scenarios.append(dict(confirmation=confirmation,initial_feet=initial.tolist(),frames=frames))
(out/'expected.json').write_text(json.dumps(dict(images=images,scenarios=scenarios),ensure_ascii=False,indent=2)+'\n')
print('fixtures:',len(images),'images',sum(len(s['frames']) for s in scenarios),'state frames')
