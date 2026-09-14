"""Opt-in integration checks: XIAOLAN_TEST_GPU=1 with the GPU Python launcher."""
import os
from pathlib import Path
import unittest
import numpy as np


@unittest.skipUnless(os.environ.get('XIAOLAN_TEST_GPU') == '1', 'requires local TensorRT engine and GPU')
class GPUIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from xiaolan_yolo_detection.tensorrt_runtime import TensorRTSession
        cls.models = Path(__file__).resolve().parents[1] / 'models'
        cls.session = TensorRTSession(cls.models/'best.engine', cls.models/'best.onnx', 416)

    @classmethod
    def tearDownClass(cls):
        cls.session.close()
        cls.session.close()

    def test_gpu_output_matches_cpu_on_blank_frame(self):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1
        cpu = ort.InferenceSession(str(self.models/'best.onnx'), sess_options=opts,
                                   providers=['CPUExecutionProvider'])
        x = np.zeros((1, 3, 416, 416), np.float32)
        expected = cpu.run(None, {cpu.get_inputs()[0].name: x})[0]
        actual = self.session.infer(x).copy()
        self.assertEqual(actual.shape, expected.shape)
        self.assertTrue(np.isfinite(actual).all())
        # FP16 changes rounding; score agreement matters even on a negative frame.
        self.assertLess(float(np.max(np.abs(actual[:, 4, :]-expected[:, 4, :]))), .02)

    def test_repeated_inference_and_buffer_validation(self):
        x = np.zeros((1, 3, 416, 416), np.float32)
        first = self.session.infer(x).copy()
        np.testing.assert_array_equal(first, self.session.infer(x))
        for invalid in [x.astype(np.float64), x[:, :, ::2, :], x[:, :, :, ::-1]]:
            with self.assertRaises(ValueError):
                self.session.infer(invalid)

    def test_wrong_input_size_is_rejected(self):
        from xiaolan_yolo_detection.tensorrt_runtime import TensorRTSession
        with self.assertRaisesRegex(ValueError, 'input dimensions'):
            TensorRTSession(self.models/'best.engine', self.models/'best.onnx', 320)
