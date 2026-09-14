"""Static TensorRT 8 GPU inference using the installed CUDA driver (no PyCUDA)."""
import atexit
import ctypes as C
import hashlib
import json
from pathlib import Path
import numpy as np


class TensorRTSession:
    def __init__(self, engine_path, onnx_path, size):
        import tensorrt as trt
        if trt.__version__.split('.')[0] != '8':
            raise RuntimeError('This JetPack 5 backend requires TensorRT 8.x')
        path = Path(engine_path)
        metadata = json.loads(path.with_suffix(path.suffix + '.json').read_text())
        digest = hashlib.sha256(Path(onnx_path).read_bytes()).hexdigest()
        if metadata['onnx_sha256'] != digest or metadata['tensorrt'] != trt.__version__:
            raise ValueError('TensorRT engine/model version mismatch; rebuild the engine')
        self.cuda = C.CDLL('libcuda.so.1')
        signatures = {
            'cuInit': [C.c_uint],
            'cuDevicePrimaryCtxRetain': [C.POINTER(C.c_void_p), C.c_int],
            'cuCtxPushCurrent_v2': [C.c_void_p],
            'cuCtxPopCurrent_v2': [C.POINTER(C.c_void_p)],
            'cuDevicePrimaryCtxRelease_v2': [C.c_int],
            'cuMemAlloc_v2': [C.POINTER(C.c_uint64), C.c_size_t],
            'cuMemFree_v2': [C.c_uint64],
            'cuMemcpyHtoD_v2': [C.c_uint64, C.c_void_p, C.c_size_t],
            'cuMemcpyDtoH_v2': [C.c_void_p, C.c_uint64, C.c_size_t],
            'cuCtxSynchronize': [],
        }
        for name, args in signatures.items():
            fn = getattr(self.cuda, name)
            fn.argtypes, fn.restype = args, C.c_int
        self._call('cuInit', 0)
        self.cu_context = C.c_void_p()
        self._call('cuDevicePrimaryCtxRetain', C.byref(self.cu_context), 0)
        self._call('cuCtxPushCurrent_v2', self.cu_context)
        self.buffers = []
        self.context = self.engine = self.runtime = None
        self.closed = False
        try:
            self.logger = trt.Logger(trt.Logger.WARNING)
            trt.init_libnvinfer_plugins(self.logger, '')
            self.runtime = trt.Runtime(self.logger)
            self.engine = self.runtime.deserialize_cuda_engine(path.read_bytes())
            if self.engine is None:
                raise RuntimeError('Cannot load TensorRT engine; rebuild on this NX')
            if self.engine.has_implicit_batch_dimension or self.engine.num_bindings != 2:
                raise ValueError('Expected one static input and one output')
            self.input_index = next(i for i in range(2) if self.engine.binding_is_input(i))
            self.output_index = 1-self.input_index
            if self.engine.binding_is_input(self.output_index):
                raise ValueError('Expected a single output')
            shapes = [tuple(self.engine.get_binding_shape(i)) for i in range(2)]
            if shapes[self.input_index] != (1, 3, size, size):
                raise ValueError('Engine input dimensions differ from input_size')
            for i in range(2):
                if self.engine.get_binding_dtype(i) != trt.float32 or min(shapes[i]) <= 0:
                    raise ValueError('Expected static FP32 input/output bindings')
                ptr = C.c_uint64()
                self._call('cuMemAlloc_v2', C.byref(ptr), int(np.prod(shapes[i]))*4)
                self.buffers.append(ptr)
            self.input_shape = shapes[self.input_index]
            self.output = np.empty(shapes[self.output_index], dtype=np.float32)
            self.context = self.engine.create_execution_context()
            if self.context is None:
                raise RuntimeError('Cannot create TensorRT execution context')
            atexit.register(self.close)
        except Exception:
            self.close()
            raise

    def _call(self, name, *args):
        status = getattr(self.cuda, name)(*args)
        if status != 0:
            raise RuntimeError('%s failed with CUDA error %d' % (name, status))

    def infer(self, tensor):
        if self.closed:
            raise RuntimeError('TensorRT session is closed')
        if tensor.shape != self.input_shape or tensor.dtype != np.float32 or not tensor.flags.c_contiguous:
            raise ValueError('TensorRT input must be contiguous FP32 with the configured shape')
        self._call('cuMemcpyHtoD_v2', self.buffers[self.input_index], tensor.ctypes.data, tensor.nbytes)
        if not self.context.execute_v2([p.value for p in self.buffers]):
            raise RuntimeError('TensorRT GPU execution failed')
        self._call('cuCtxSynchronize')
        self._call('cuMemcpyDtoH_v2', self.output.ctypes.data, self.buffers[self.output_index], self.output.nbytes)
        return self.output

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.context = self.engine = self.runtime = None
        for ptr in self.buffers:
            self._call('cuMemFree_v2', ptr)
        popped = C.c_void_p()
        self._call('cuCtxPopCurrent_v2', C.byref(popped))
        self._call('cuDevicePrimaryCtxRelease_v2', 0)
