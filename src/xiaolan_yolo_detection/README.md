# Xiaolan YOLO Detection

基于 **ROS Noetic、YOLO ONNX 和对齐深度图**的小蓝平台单目标检测与定位项目。在 Xavier NX 上默认使用 TensorRT FP16 GPU 推理，保留 ONNX Runtime CPU 后端，输出目标检测框、相机坐标系下的横向偏移 X 和深度 Z，并提供实时可视化与训练图片采集工具。

本包负责感知，不启动运动控制器；定位结果使用 `xiaolan_yolo_detection/XiaolanTarget` 消息，可供已有控制器订阅。

## Grasp_hexapod 本机部署

`XiaolanDetection` 和 `XiaolanTarget` 消息均由本包提供。源模型为包内 `models/best.onnx`，
本机 GPU 引擎为 `models/best.engine`。引擎必须在目标设备上构建，不提交到仓库。

在 `~/Grasp_hexapod` 下安装和编译：

```bash
/usr/bin/python3 -m venv --without-pip --system-site-packages .venv/xiaolan-ort
/usr/bin/python3 -m pip --python .venv/xiaolan-ort/bin/python3 install -r src/xiaolan_yolo_detection/requirements/runtime.txt
./build_release.sh --pkg xiaolan_yolo_detection -j2
/usr/bin/python3 src/xiaolan_yolo_detection/scripts/setup_gpu_runtime.py
.venv/jetson-gpu/bin/python3 src/xiaolan_yolo_detection/scripts/build_trt_engine.py
```

以上环境创建方式适用于本机缺少 `ensurepip`、已有支持 `--python` 的 pip 的情况。
依赖安装到虚拟环境，不替换系统 Python 包。GPU 安装脚本仅支持本机的 JetPack 5 /
L4T R35、aarch64、Python 3.8，从 NVIDIA r35.6 软件源下载并校验 SHA256，
把 TensorRT 8.5.2、CUDA 11.4 和 cuDNN 8.6 运行库解包到 `.venv/jetson-gpu/`，无需 sudo。
GPU 启动失败会明确报错，不会自动退回 CPU。更换 ONNX 模型后必须重新构建引擎；
启动时会校验 ONNX SHA256 和 TensorRT 版本。

2026-09-14 本机 Xavier NX 验证：首次 FP16 引擎构建约 17 分钟，生成约 7 MB
引擎。6 张真实图像的 CPU/GPU 检测框最多相差 1 像素，置信度差异小于 0.001。
连续 60 秒 ROS 测试中 896 帧全部有效检测和定位，约 15 FPS；三个 20 秒窗口的
GPU 推理耗时中位数为 13–17 ms，最大值 31.4 ms。该结果对应当前场景、15 FPS
相机和 416 输入，不代表所有场景的精度或最高吞吐量。15 项测试（含实际 GPU）通过。

连接 Orbbec 相机后，终端一启动相机：

```bash
cd ~/Grasp_hexapod
source devel/setup.bash
roslaunch xiaolan_yolo_detection xiaolan_orbbec_camera.launch
```

终端二启动检测和定位（默认无窗口，有桌面时可加 `show_window:=true`）：

```bash
cd ~/Grasp_hexapod
./launch_xiaolan_yolo.sh
```

也可以直接执行 `roslaunch xiaolan_yolo_detection xiaolan_yolo.launch`，默认使用
本工作空间 `.venv/jetson-gpu/bin/python3`（自动设置本地 GPU 库路径）。若使用 install 空间或其他目录的环境，
通过 `yolo_python:=/absolute/path/to/python3` 指定解释器。

终端三查看结果：

```bash
source ~/Grasp_hexapod/devel/setup.bash
rostopic echo /xiaolan/target
rostopic echo /xiaolan/target_debug
```

按 Ctrl+C 停止对应终端。真实检测精度、帧率和深度对齐需要连接相机后验证。

## 功能

- 单类别检测：类别 `0` 为 `xiaolan`，每帧最多保留一个目标。
- TensorRT FP16 GPU 推理，可选 ONNX Runtime CPU 后端，默认输入为 `416 × 416`。
- 根据 RGB 时间戳匹配对齐深度图，在检测框中心区域取有效深度中值。
- 支持连续帧确认、位置跳变过滤、短时丢失保持和 EMA 平滑。
- 发布检测框、定位结果、诊断信息和可视化图像。
- 提供图片采集、YOLO 数据集整理及基础数值逻辑测试。

处理流程：

```text
RGB 图像 → YOLO 检测 → 匹配对齐深度 → 中心区域深度中值
                                      ↓
可视化 ← 检测框与定位结果 ← 时序过滤 ← 计算 X / Z
```

## 目录结构

```text
xiaolan_yolo_detection/
├── config/xiaolan_yolo.yaml       # 检测、深度同步与滤波参数
├── launch/xiaolan_yolo.launch \
    # 启动检测、定位和可视化节点
├── models/best.onnx               # 随仓库提供的 ONNX 模型
├── msg/XiaolanDetection.msg       # 二维检测消息
├── msg/XiaolanTarget.msg          # 三维定位消息
├── scripts/
│   ├── xiaolan_yolo_node.py \
      # ONNX 检测节点
│   ├── xiaolan_target_3d_node.py  # 深度定位节点
│   ├── xiaolan_visualizer.py \
     # 可视化节点
│   ├── xiaolan_image_collector.py # 图片采集工具
│   └── prepare_xiaolan_dataset.py # 数据集整理工具
├── src/xiaolan_yolo_detection/    # 图像预处理、深度与滤波逻辑
├── requirements/runtime.txt      # 隔离推理环境依赖
├── tests/test_core.py \
           # 基础单元测试
└── docs/
    ├── TRAINING.md               # 数据采集、训练与导出说明
    └── ANALYSIS.md               # 设计与复用说明
```

## 环境要求

- ROS Noetic、Python 3 和 catkin 工作空间。
- ROS 依赖：`rospy`、`std_msgs`、`sensor_msgs`、`cv_bridge`、消息生成工具。
- 系统 Python 可使用 NumPy 和 OpenCV；推理依赖安装在独立虚拟环境中。
- 提供 RGB、与 RGB 对齐的深度图及相机内参的相机，例如 RealSense D435i。

## 安装与编译

### 1. 获取代码

首次安装时执行；如果本地已有项目，跳过克隆：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/LIULEIHUZHOU/xiaolan_yolo_detection.git
```

在工作空间中编译：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 2. 创建独立推理环境

以下命令要求本机已安装 `python3-venv`，并且虚拟环境内可使用 pip：

```bash
python3 -m venv --system-site-packages ~/catkin_ws/xiaolan-ort
~/catkin_ws/xiaolan-ort/bin/python3 -m pip install -r \
  ~/catkin_ws/src/xiaolan_yolo_detection/requirements/runtime.txt
```

依赖版本为 `numpy==1.24.4`、`onnxruntime==1.16.3` 和 `onnx==1.15.0`。其中 ONNX 用于模型检查；推理使用 ONNX Runtime。虚拟环境继承系统的 ROS、CvBridge 和 OpenCV，避免直接升级 ROS 系统 Python 的依赖。运行检测不需要安装 PyTorch 或 Ultralytics。

## 启动

### 1. 启动相机

本工作空间使用 Orbbec Gemini 330 系列时，运行：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch xiaolan_yolo_detection xiaolan_orbbec_camera.launch
```

该入口使用 848×480、15 FPS 和深度对齐配置，关闭软件深度去噪，降低本机采集延迟。
可通过 `camera_fps` 和 `enable_noise_removal_filter` 覆盖；本机原 30 FPS 加软件去噪
配置实测 RGB 延迟约 0.50–0.63 秒，超过检测的 0.5 秒图像有效期。
同时关闭图像传输插件，避免
Theora 将 `16UC1` 深度转为 `bgr8` 引发刷屏报错；原始 RGB、深度和内参仍然发布。
当前配置使用 Orbbec 的 `/camera/depth/image_raw` 与 `/camera/depth/camera_info`，
必须配合 `depth_registration:=true`。下面 RealSense 入口需要把配置中的这两个话题
改为对应的 `/camera/aligned_depth_to_color/...`。

如果已安装的 RealSense 驱动提供标准启动文件，可运行：

```bash
source /opt/ros/noetic/setup.bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

相机驱动需自行安装和配置。若使用已有相机启动流程，确保输出以下话题：

| 输入话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 检测图像 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 与 RGB 对齐的深度 |
| `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 定位使用的内参 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 收到后用于内参一致性检查 |

深度支持 `16UC1`（毫米）和 `32FC1`（米）。必须使用对齐深度，不能直接用原始深度话题替代。

### 2. 启动检测与定位

在另一个终端运行：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch xiaolan_yolo_detection xiaolan_yolo.launch \
  backend:=onnx \
  yolo_python:="$HOME/catkin_ws/xiaolan-ort/bin/python3" \
  show_window:=true
```

无桌面环境将 `show_window` 改为 `false`；仍会发布可视化图像话题。

| 启动参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model_path` | 包内 `models/best.onnx` | ONNX 模型路径 |
| `config` | 包内 `config/xiaolan_yolo.yaml` | 参数文件路径 |
| `yolo_python` | 包目录下 `../../.venv/jetson-gpu/bin/python3` | 自动加载本地 TensorRT/CUDA 运行库的解释器入口 |
| `backend` | `tensorrt` | GPU 推理；显式设置 `onnx` 可使用 CPU |
| `engine_path` | 包内 `models/best.engine` | 本机生成的 TensorRT FP16 引擎 |
| `start_yolo` | `true` | 启用检测节点 |
| `start_3d` | `true` | 启用定位节点 |
| `start_visualizer` | `true` | 启用可视化节点 |
| `show_window` | `false` | 显示 OpenCV 窗口 |

`start_yolo:=false` 可用于配合录制的检测消息调试定位与显示，本身不会进行目标识别。

### 3. 查看结果

```bash
rostopic echo /xiaolan/target
rostopic echo /xiaolan/target_debug
rostopic hz /xiaolan/detection
rostopic hz /xiaolan/target
rosrun rqt_image_view rqt_image_view /xiaolan_visualizer/image
```

| 输出话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/xiaolan/detection` | `xiaolan_yolo_detection/XiaolanDetection` | 检测状态、置信度、检测框、中心像素、推理耗时及原因 |
| `/xiaolan/target` | `xiaolan_yolo_detection/XiaolanTarget` | 供下游使用的目标定位结果 |
| `/xiaolan/target_debug` | `std_msgs/String` | 定位诊断信息 |
| `/xiaolan_visualizer/image` | `sensor_msgs/Image` | 叠加检测与定位信息的图像 |

检测框坐标和消息时间戳对应原始 RGB 图像。X、Z 使用**相机光学坐标系**：X 向右为正、向左为负，Z 为前方深度，单位为米；未转换到机器人 `base_link` 坐标系。

不要同时运行占用 `/xiaolan/detection` 同名话题的旧检测链，尤其是使用不同消息类型的节点。

## 主要配置

在 [config/xiaolan_yolo.yaml](config/xiaolan_yolo.yaml) 中修改：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_size` | `416` | 必须与模型静态输入尺寸一致 |
| `conf_threshold` | `0.5` | 检测置信度阈值 |
| `max_det` | `1` | 当前实现仅支持一个目标 |
| `max_fps` | `15.0` | 推理处理频率上限，不代表实测帧率 |
| `intra_op_num_threads` | `2` | ONNX Runtime 算子内部线程数；关闭空闲忙等以减轻 CPU 争用 |
| `visualization_max_fps` | `5.0` | 预览刷新率上限；可视化 OpenCV 固定为单线程，降低 NX 与远程桌面的负载 |
| `depth_sync_tolerance` | `0.025` | RGB 与深度时间戳容差，秒 |
| `roi_size` | `9` | 检测框中心深度采样区域边长，像素 |
| `min_valid_depth_pixels` | `5` | 深度区域最少有效像素数 |
| `min_depth` / `max_depth` | `0.20` / `1.00` | 有效深度范围，严格排除边界，米 |
| `center_tolerance_m` | `0.03` | 居中判断的横向偏移容差，米 |
| `confirm_frames` | `2` | 目标确认所需连续有效帧数 |
| `lost_tolerance_frames` | `3` | 短时丢失容忍帧数 |
| `max_hold_seconds` | `0.25` | 丢失或跳变时，自上次有效测量起允许复用旧结果的最长时间；不用于重置连续有效检测，秒 |
| `ema_alpha` | `0.4` | 位置平滑系数 |
| `target_timeout` | `0.5` | 检测消息中断阈值；超过后清空确认状态，秒 |
| `max_result_age` | `0.8` | 从 RGB 采集到定位结果的总有效期，包含 CPU 推理耗时，秒 |

CPU 对比启动（本机 GPU 解释器也包含 ONNX Runtime）：

```bash
roslaunch xiaolan_yolo_detection xiaolan_yolo.launch backend:=onnx
```

## 模型要求

仓库已包含 `models/best.onnx`。本 README 不提供该模型的训练指标或真机精度结论，部署时需结合实际场景验证。

替换模型时，应满足节点当前支持的格式：

- 单类别 YOLOv8 / YOLO11 原始检测输出，类别 `0` 为 `xiaolan`。
- 静态 FP32 输入，默认形状为 `[1, 3, 416, 416]`。
- 单个输出张量，形状为 `[1, 5, N]` 或 `[1, N, 5]`。
- 导出时不内置 NMS；不支持直接加载通用 COCO 多类别、分割或端到端后处理模型。

如改用 480 输入，需要同时替换为 480 静态模型并修改 `input_size`。数据标注、训练与导出流程见 [训练说明](docs/TRAINING.md)。

## 图片采集与数据整理

相机启动并加载工作空间环境后，运行：

```bash
rosrun xiaolan_yolo_detection xiaolan_image_collector.py \
  _save_dir:="$HOME/xiaolan_dataset/raw"
```

在采集窗口按 **空格**保存图片，按 **Q** 或 **Esc** 退出。该工具需要桌面显示环境。

图片存放于 `raw/`，对应的 YOLO 标签放在 `labels_raw/`，图片与标签使用相同文件名主体。先预览整理结果：

```bash
python3 ~/catkin_ws/src/xiaolan_yolo_detection/scripts/prepare_xiaolan_dataset.py \
  --root "$HOME/xiaolan_dataset" --dry-run
```

确认后移除 `--dry-run` 执行。脚本按正负样本分别随机划分约 70% / 20% / 10% 的训练、验证、测试集，并生成 `xiaolan.yaml`。若有 labelImg 的 `classes.txt` 元数据，可加 `--exclude-classes-txt`。

**缺失标签会被补为空标签并作为负样本；请先完成正样本标注。** 执行时会清空并重建 `images/{train,val,test}` 和 `labels/{train,val,test}` 生成目录。连续采集图片还需按场次检查集合划分，避免相邻帧跨集合造成评估偏差。

## 测试

在 Grasp_hexapod 工作空间根目录运行测试，需要先编译消息，并可导入 NumPy 和 OpenCV：

```bash
source devel/setup.bash
/usr/bin/python3 -m unittest discover -s src/xiaolan_yolo_detection/tests -v
```

完成 GPU 安装和引擎构建后，可额外运行实际 GPU 集成测试（普通测试默认跳过这些项）：

```bash
source devel/setup.bash
XIAOLAN_TEST_GPU=1 .venv/jetson-gpu/bin/python3 -m unittest discover -s src/xiaolan_yolo_detection/tests -v
```

测试覆盖检测框坐标还原、单目标选择、深度单位及无效值处理、低帧率连续确认、丢失保持到期、结果过期、断流和乱序拒绝。测试不替代相机连接、模型识别精度或实时性能的真机验证。

## 常见问题

YOLO 日志中的 `FPS` 只统计实际执行推理的帧，`dropped` 是该日志周期内未执行推理的帧数。
时间戳警告附带 `age_s`（图像采集至检查时的总延迟）和 `local_wait_s`
（图像回调到推理线程的等待时间），便于区分上游图像延迟与本节点等待。

若有检测框但定位失败，查看 `/xiaolan/target_debug` 中的 `depth_stats`：
`NO_VALID_DEPTH` 表示中心 ROI 无正数有限深度；`DEPTH_OUT_OF_RANGE` 表示深度全部
超出配置的严格范围 `(0.20, 1.00)` 米；`INSUFFICIENT_DEPTH_PIXELS` 表示范围内像素
不足。`observed_median_m` 给出筛选前的正数深度中值，`valid_pixels` 给出范围内像素数。
应根据实际距离和测量结果调整范围，或检查相机深度空洞，不要仅凭检测置信度判断定位有效。

| 现象 | 排查方向 |
| --- | --- |
| 找不到 `XiaolanTarget` 消息 | 重新编译本包，并执行 `source devel/setup.bash` |
| 提示找不到 `tensorrt` 或 CUDA 库 | 执行本机 GPU 安装步骤，并使用 `.venv/jetson-gpu/bin/python3` 入口 |
| 引擎缺失或版本/模型不匹配 | 用 GPU 解释器重新运行 `scripts/build_trt_engine.py` |
| 提示找不到 `onnxruntime` | 检查 `yolo_python` 是否指向已安装依赖的虚拟环境 |
| `STALE_OR_INVALID_RGB_TIMESTAMP` / `STALE_RGB_RESULT` | 检测前 RGB 有效期为 `max_input_age`（0.5 秒），定位总有效期为 `max_result_age`（0.8 秒）；检查相机延迟和 CPU 推理耗时，诊断话题 `rgb_age_s` 可查看定位时帧龄 |
| `OUT_OF_ORDER_RGB` / `INVALID_RGB_TIMESTAMP` | 检查重复、倒退、零时间戳或超前的 RGB 时间戳，以及 ROS 时钟配置 |
| 模型输入或输出格式报错 | 检查单类别、静态尺寸、FP32 和无内置 NMS 的导出要求 |
| 有检测框但没有有效定位 | 检查对齐深度、内参、时间戳匹配以及深度是否在配置范围内 |
| 没有弹出窗口 | 检查 `show_window:=true` 和桌面环境，或订阅可视化图像话题 |
| 处理频率偏低 | 检查推理耗时、图像输入频率，尝试关闭窗口并调整 ORT 线程数 |

## 后续更新到 GitHub

在项目根目录执行：

```bash
git add .
git commit -m "描述本次修改"
git push
```
