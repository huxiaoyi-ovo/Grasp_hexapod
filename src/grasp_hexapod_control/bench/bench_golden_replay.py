import sys, time
sys.path.insert(0, "/home/artrc/guangfu/Grasp_hexapod/src/grasp_hexapod_control/scripts")
import numpy as np
from control import GraspController
from kinematics import Q_STAND

DT = 1.0 / 30.0
controller = GraspController(DT, enable_link_collision_check=False,
                             climb_timeout_uses_wall_time=False)
q_start = Q_STAND + np.deg2rad(np.array([[10,-12,8],[-9,11,-7],[12,-10,9],
                                         [-11,9,-8],[9,-13,11],[-10,8,-9]]))
command = np.zeros(4)
controller.reset_to_stand(q_start)
q_cur = q_start.copy()
# 预热
for _ in range(50):
    q_cur = controller.update(q_cur, command)

REPEATS = 5
FRAMES = 825
t0 = time.perf_counter()
for r in range(REPEATS):
    if r != 0:
        controller = GraspController(DT, enable_link_collision_check=False,
                                     climb_timeout_uses_wall_time=False)
        controller.reset_to_stand(q_start)
        q_cur = q_start.copy()
    command = np.zeros(4)
    for idx in range(FRAMES):
        if 75 <= idx < 375:
            command = np.array([0.05, 0.10, 0.0, 0.15])
        elif 375 <= idx < 675:
            command = np.array([-0.08, -0.05, 0.008, -0.20])
        else:
            command = np.zeros(4)
        q_cur = controller.update(q_cur, command)
t1 = time.perf_counter()
print(f"python_per_frame_us={(t1 - t0) / (REPEATS * FRAMES) * 1e6:.1f}")
