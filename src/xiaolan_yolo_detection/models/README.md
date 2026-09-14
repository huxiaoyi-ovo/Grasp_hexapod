`best.onnx` 是已训练的单类别 YOLO11n，类别 0 为 xiaolan。
输入为静态 FP32、batch=1、416×416，输出为原始检测头，不含 NMS。

`best.engine` 和 `best.engine.json` 由 `scripts/build_trt_engine.py` 在本机生成，
用于 TensorRT FP16 GPU 推理，并校验源模型 SHA256 和 TensorRT 版本。
引擎不提交到仓库；更换模型、设备或 TensorRT 版本后应重新构建。
