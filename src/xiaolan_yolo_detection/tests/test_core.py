import unittest
import numpy as np
from xiaolan_yolo_detection.yolo_core import Letterbox, best_box, depth_median, StableTarget

class CoreTest(unittest.TestCase):
    def test_letterbox_restore_and_top_nms(self):
        tensor, mapping = Letterbox(416).prepare(np.zeros((480,640,3),np.uint8))
        self.assertEqual(tensor.shape,(1,3,416,416))
        raw=np.array([[[208,208,130,65,.9],[208,208,130,65,.7]]],np.float32)
        box=best_box(raw,mapping,.5)
        self.assertEqual(box[:4],(220,190,200,100))
        self.assertAlmostEqual(box[4],.9)
        self.assertEqual(best_box(raw.transpose(0,2,1),mapping,.5),box)
        self.assertIsNone(best_box(raw,mapping,.95))
        with self.assertRaises(ValueError): best_box(np.zeros((1,84,100)),mapping,.5)

    def test_roi_median_units_holes_and_strict_range(self):
        depth=np.full((40,40),900,np.uint16)
        depth[16:25,16:25]=500
        depth[20,20]=0
        args=((0,0,40,40),(20,20))
        self.assertAlmostEqual(depth_median(depth,'16UC1',*args),.5)
        metres=depth.astype(np.float32)/1000
        metres[20,20]=np.nan
        self.assertAlmostEqual(depth_median(metres,'32FC1',*args),.5)
        for val in [0.,.2,1.,float('nan')]:
            metres[16:25,16:25]=val
            self.assertIsNone(depth_median(metres,'32FC1',*args))
        with self.assertRaises(ValueError): depth_median(depth,'16UC1',(0,0,50,40),(20,20))

    def test_depth_failure_diagnostics(self):
        args = ((0, 0, 9, 9), (4, 4))
        for raw, reason, observed in [(0, 'NO_VALID_DEPTH', None),
                                      (1500, 'DEPTH_OUT_OF_RANGE', 1.5),
                                      (500, 'MEASURED', .5)]:
            stats = {}
            result = depth_median(np.full((9, 9), raw, np.uint16),
                                  '16UC1', *args, diagnostics=stats)
            self.assertEqual(stats['reason'], reason)
            self.assertEqual(stats['roi_pixels'], 81)
            if observed is None:
                self.assertIsNone(stats['observed_median_m'])
            else:
                self.assertAlmostEqual(stats['observed_median_m'], observed, places=6)
            self.assertEqual(result is not None, reason == 'MEASURED')
        depth = np.zeros((9, 9), np.float32)
        depth[4, 4] = .5
        stats = {}
        self.assertIsNone(depth_median(depth, '32FC1', *args, diagnostics=stats))
        self.assertEqual(stats['reason'], 'INSUFFICIENT_DEPTH_PIXELS')
        self.assertEqual(stats['valid_pixels'], 1)

    def test_confirmation_gate_ema_loss_recovery(self):
        f=StableTarget()
        self.assertEqual(f.update((0,.5))[2],'CONFIRMING')
        val,_,state=f.update((.1,.6))
        self.assertEqual(state,'VALID')
        np.testing.assert_allclose(val,[.04,.54])
        for _ in range(3): self.assertEqual(f.update((.9,.9))[2],'HELD')
        self.assertEqual(f.update(None)[2],'NOT_FOUND')
        self.assertEqual(f.update((-.2,.4))[2],'CONFIRMING')
        self.assertEqual(f.update((-.2,.4))[2],'VALID')
        f.reset()
        f.update((0,.5));f.update(None)
        self.assertEqual(f.update((0,.5))[2],'CONFIRMING')

if __name__=='__main__': unittest.main()
