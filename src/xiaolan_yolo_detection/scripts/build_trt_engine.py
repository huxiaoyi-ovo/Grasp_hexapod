#!/usr/bin/env python3
"""Build a local TensorRT FP16 engine from the static single-class ONNX model."""
import argparse
import hashlib
import json
from pathlib import Path
import tensorrt as trt


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=package/'models/best.onnx')
    parser.add_argument('--output', type=Path, default=package/'models/best.engine')
    args = parser.parse_args()
    logger = trt.Logger(trt.Logger.INFO)
    trt.init_libnvinfer_plugins(logger, '')
    builder = trt.Builder(logger)
    if not builder.platform_has_fast_fp16:
        raise RuntimeError('GPU does not support fast FP16')
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    onnx_parser = trt.OnnxParser(network, logger)
    data = args.model.read_bytes()
    if not onnx_parser.parse(data):
        raise RuntimeError('\n'.join(str(onnx_parser.get_error(i)) for i in range(onnx_parser.num_errors)))
    if network.num_inputs != 1 or network.num_outputs != 1:
        raise ValueError('Expected one input and one output')
    shape = tuple(network.get_input(0).shape)
    if len(shape) != 4 or shape[:2] != (1, 3) or shape[2] != shape[3] or min(shape) <= 0:
        raise ValueError('Expected static batch=1 NCHW input')
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)
    config.set_flag(trt.BuilderFlag.FP16)
    engine = builder.build_serialized_network(network, config)
    if engine is None:
        raise RuntimeError('TensorRT engine build failed')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix+'.tmp')
    temporary.write_bytes(bytes(engine))
    temporary.replace(args.output)
    args.output.with_suffix(args.output.suffix+'.json').write_text(json.dumps({
        'onnx_sha256': hashlib.sha256(data).hexdigest(), 'tensorrt': trt.__version__,
        'input_shape': shape, 'precision': 'FP16', 'device': 'Jetson Xavier NX',
    }, indent=2))
    print('Built TensorRT FP16 engine:', args.output)


if __name__ == '__main__':
    main()
