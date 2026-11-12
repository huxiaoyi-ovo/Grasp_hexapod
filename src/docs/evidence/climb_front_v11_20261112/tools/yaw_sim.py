"""Run the repository run_sim.py unchanged, but place Xiaolan rotated by YAW_DEG about the robot start
(world xy of P0). Equivalent to a robot start heading error of -YAW_DEG. Test harness only.
Usage: YAW_DEG=3 python3 yaw_sim.py <run_sim args...>"""
import os, sys
import numpy as np
SCRIPTS = '/home/artrc/Grasp_hexapod/src/grasp_hexapod_control/scripts'
sys.path.insert(0, SCRIPTS)
sys.argv[0] = os.path.join(SCRIPTS, 'run_sim.py')
import run_sim  # noqa: E402

YAW = np.radians(float(os.environ.get('YAW_DEG', '0')))
CENTRE = np.array((-0.1, -0.067694497636))
_orig = run_sim.add_static_stl_triangle_mesh


def rotated(gym, sim, mesh_path, position, friction_scale=1.0):
    gymapi = run_sim.gymapi
    data = mesh_path.read_bytes()
    n = int(np.frombuffer(data, '<u4', 1, 80)[0])
    rec = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    v = np.frombuffer(data, rec, n, 84)['vertices'].reshape(-1, 3).astype(np.float64)
    v += (position.x, position.y, position.z)
    c, s = np.cos(YAW), np.sin(YAW)
    xy = v[:, :2] - CENTRE
    v[:, :2] = CENTRE + xy @ np.array(((c, s), (-s, c)))
    v = np.ascontiguousarray(v.astype(np.float32))
    tri = np.arange(n * 3, dtype=np.uint32).reshape(-1, 3)
    p = gymapi.TriangleMeshParams()
    p.nb_vertices, p.nb_triangles = v.shape[0], tri.shape[0]
    p.transform.p = gymapi.Vec3(0, 0, 0)
    p.static_friction, p.dynamic_friction = 1.0 * friction_scale, .8 * friction_scale
    gym.add_triangle_mesh(sim, v.ravel(), tri.ravel(), p)
    print(f'Xiaolan loaded ROTATED by {np.degrees(YAW):+.2f} deg about P0: {n} triangles')


run_sim.add_static_stl_triangle_mesh = rotated
run_sim.main()
