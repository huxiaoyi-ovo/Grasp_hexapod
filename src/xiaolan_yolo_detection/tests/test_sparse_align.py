#!/usr/bin/env python3
"""depth_median_sparse (unaligned depth + extrinsics) vs. the aligned reference."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from xiaolan_yolo_detection.yolo_core import depth_median, depth_median_sparse  # noqa: E402

W, H = 848, 480
KC = np.array([[424., 0, 424.], [0, 424., 240.], [0, 0, 1.]])
KD = np.array([[420., 0, 426.], [0, 420., 238.], [0, 0, 1.]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def render(plane_z_c, r_cd, t_cd, tilt=0.0):
    """Depth image (depth camera) of a plane z_c = plane_z_c + tilt * x_c in the colour frame."""
    v, u = np.mgrid[0:H, 0:W].astype(float)
    ray_d = np.stack(((u-KD[0, 2])/KD[0, 0], (v-KD[1, 2])/KD[1, 1], np.ones_like(u)))
    ray_c = np.tensordot(r_cd, ray_d, axes=1)  # direction in the colour frame
    # point = s*ray_d (depth frame) -> p_c = s*ray_c + t ; solve p_c.z = z0 + tilt*p_c.x
    s = (plane_z_c + tilt*t_cd[0] - t_cd[2]) / (ray_c[2] - tilt*ray_c[0])
    return (s * 1000.).astype(np.uint16)  # depth z in the depth frame = s (ray_d.z = 1)


def aligned_reference(plane_z_c, tilt=0.0):
    v, u = np.mgrid[0:H, 0:W].astype(float)
    rx = (u-KC[0, 2])/KC[0, 0]
    return ((plane_z_c / (1 - tilt*rx)) * 1000.).astype(np.uint16)


class SparseAlignTest(unittest.TestCase):
    def check(self, z, r_cd, t_cd, center, tilt=0.0):
        box = (center[0]-60, center[1]-40, 120, 80)
        depth = render(z, r_cd, t_cd, tilt)
        stats = {}
        got = depth_median_sparse(depth, '16UC1', KD, KC, r_cd, t_cd, box, center,
                                  min_z=.2, max_z=2.5, diagnostics=stats)
        ref = depth_median(aligned_reference(z, tilt), '16UC1', box, center, min_z=.2, max_z=2.5)
        self.assertIsNotNone(got, stats)
        self.assertEqual(stats['reason'], 'MEASURED')
        self.assertGreaterEqual(stats['valid_pixels'], 20)
        self.assertAlmostEqual(got, ref, delta=0.004)
        return got

    def test_fronto_parallel_with_baseline(self):
        for z in (0.25, 0.6, 1.0, 2.0):
            got = self.check(z, np.eye(3), np.array([0.032, 0.0, 0.001]), (424, 240))
            self.assertAlmostEqual(got, z, delta=0.003)

    def test_off_centre_rotated_and_tilted(self):
        self.check(0.8, rot_y(np.radians(0.7)), np.array([-0.03, 0.002, 0.0]), (250, 300), tilt=0.15)
        self.check(0.4, rot_y(np.radians(-0.5)), np.array([0.03, 0.0, 0.002]), (700, 120), tilt=-0.2)

    def test_no_depth_reports_reason(self):
        stats = {}
        got = depth_median_sparse(np.zeros((H, W), np.uint16), '16UC1', KD, KC, np.eye(3),
                                  np.array([0.03, 0, 0]), (300, 200, 100, 80), (350, 240),
                                  diagnostics=stats)
        self.assertIsNone(got)
        self.assertEqual(stats['reason'], 'NO_VALID_DEPTH')


if __name__ == '__main__':
    unittest.main()
