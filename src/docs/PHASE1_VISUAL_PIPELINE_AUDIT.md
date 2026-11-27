# Phase-1 小蓝视觉粗接近：离线全链路审计（2026-10-30）

## 结论与边界

本批次闭合“前方相机检测/深度定位 → 视觉锁定 → 粗接近 → HOLD”的离线 ROS
接口验证。它**不**授权真实相机、串口舵机、带载步行、攀爬、接触、边界净空或
识别准确率。流程止于粗接近，成功消息为
`coarse approach completed (AprilTag alignment pending)`；不会进入 climb/dock。

本机 CPU 真实 ONNX Runtime 烟雾测试对空白合成 RGB 输出 `NO_DETECTION`。这证明
模型文件、ROS 节点和 CPU 后端可执行，不证明小蓝正样本识别、距离精度、时延或
GPU/TensorRT 部署。

## 节点、话题、坐标和命令

| 段 | ROS 接口 | 数据/责任 |
|---|---|---|
| RGB 检测 | `/camera/color/image_raw` (`sensor_msgs/Image`) → `/xiaolan/detection` (`XiaolanDetection`) | `YoloNode` 保留 RGB Header；ONNX/TensorRT 检测框在 RGB 像素坐标。 |
| 深度定位 | detection + `/camera/depth/image_raw` + depth/color `CameraInfo` → `/xiaolan/target` (`XiaolanTarget`) | `Target3DNode` 要求尺寸/frame 对齐的深度 ROI；有效深度严格在 `(0.2,1.0) m`。 |
| 粗接近 | `/xiaolan/target` → `spin_search` / `approach` → `/<leg>_des` | C++ 只经现有 controller 更新关节目标；不直接发串口或 `cmd_vel`。 |

`x_target/z_target` 是相机光学坐标：`+x` 右、`+z` 前。控制向量索引为
`(right,forward,up,yaw)`；本路径输出 `(0,v,0,w)`，目标 `+x` 产生负 `yaw`。
这是固定相机光学假设，尚无相机到机身外参、热更新或 CAD 攀爬起点转换；也没有
障碍物、板边界、足端净空或攀爬起点几何证明。

## 状态机、时序与安全门

手动 sequence 为 `home -> spin_search -> approach -> HOLD`，默认不自动启动。搜索
需 3 个不同、居中、新 source stamp；大方位误差先转向且禁止前进。光学深度
`z <= .60 m` 且居中后，先等步态排空，再以持续新鲜观测稳定 `.25 s` 才粗完成。

诊断起点：目标年龄 `.25 s`、丢失超时 `1 s`、搜索/接近超时各 `60 s`、最大前进
`.03 m/s`、最大 yaw `.15 rad/s`、相机光学深度停距 `.60 m`（不是机器人净空或
预攀爬姿态）。深度上限 `1 m` 意味着 1 m 外无用于锁定的有效测量，不存在“远处先锁”
保证。感知 `max_fps=15` 是节流，不是新鲜度证明；
控制反馈约 30 Hz、poll 120 Hz；watchdog/hold 也不能替代 source freshness。

回调仅缓存；控制线程只接受有限、非未来、post-run/post-resume、非空 frame、`z>0`
且在年龄窗内样本。重复/乱序 callback 为 held 并置零；同一已接受样本多次控制读取
可在年龄窗内维持命令但不增加确认。未来、NaN、坏/零深度不污染水位；暂停恢复
重设 freshness floor，不能复用暂停前锁定/settle。

视觉启用时，空闲 `ModeStatus` 报告 `phase1 waiting for joint feedback` 或
`phase1 joint feedback ready`。这只表示控制节点的六腿反馈健康门，不表示 home、
接触、载荷、姿态或物理可用。缺反馈会失败活动 Phase-1 请求；恢复不自动启动 search。

## 启动、命名空间与驱动边界

PC 离线检查只用 `phase1_visual_bench.launch`：它启动 `/bt_control_node`、受保护的
fake feedback/gripper，且可选 `Target3DNode`；不启动 serial、servo、joy、相机或
合成 target。bench 手动 sequence 必须传 `_controller_node:=/bt_control_node`；生产
默认参数是 `/grasp_hexapod_bt_control`。`phase1_visual.launch` 包含
`control_only.launch` 并会带起 servo/joy，PC 离线验证不得使用。自动
`start_sequence:=true` 在反馈未 ready 时可安全 fail-fast；默认手动启动并在 ready 后重试。

驱动包本批次只读。静态接口为每腿 `Float64MultiArray` 固定 10 元素：索引 0
`power` 上力标志（控制节点写 1），索引 1--3 为弧度关节位置。实际 driver 无命令年龄 watchdog；
HOLD 不是扭矩关闭/急停，缓存最后目标仍可能保持上力。

## 证据与未闭合项

证据位于 `src/docs/evidence/phase1_visual_20261030/`，含 C2 startup、直接视觉、
RGB-D/Target3D、gtest、CPU 单测和 ONNX smoke 的原始收据及 SHA-256 清单。感知
unittest 是 **12 passed + 3 skipped**（`Ran 15`），不是“15 passed”。C 感知启动
竞态失败收据保留在 `/tmp/grasp_phase1_perception_e2e_20261030T005744` 和 `...005841`；
C2 改为等待控制节点自产生的 feedback-ready 状态。首次 smoke 的解释器符号链接错误
收据保留在 `/tmp/grasp_phase1_yolo_smoke_20261030T010557`；修正为保留 venv 路径后，
`...010636` 通过。

尚待：Jetson/TensorRT、真实相机时间戳/对齐/正样本统计、相机外参、边界/障碍/净空、
带载舵机与急停、AprilTag 精对齐及一切攀爬接触/载荷/稳定性结论。另有既存包装限制：
`xiaolan_yolo_detection` 的 CMake 安装目录包含不存在的 `docs`，本批未验证 install-space
或 clean Jetson 部署。不得把本审计称为硬件验证。

| 离线项目 | 最终收据结果（均为 synthetic/host-only） |
|---|---|
| gtest | VisualGuidance 9/9；ModeFramework 13/13；Routing 7/7。 |
| startup | 无反馈 `.269 s`、home 中反馈丢失 `.515 s` 均失败返回。 |
| 直接视觉 | 搜索/接近 target span 非零；运行中反馈丢失 `.800 s` 返回。 |
| RGB-D | Target3D 末样本 age `.030 s`，`x=.0000 m`、`z=.5501 m`；坏深度/帧失配均 fail-stop。 |
| ONNX CPU | 空白帧 15 个，`14.996 Hz`，`23.066–29.447 ms`，均为 `NO_DETECTION`。 |
