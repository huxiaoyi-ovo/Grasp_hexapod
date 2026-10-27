# 单模型YOLO RGB-D感知部署

该ROS 1 Noetic包只加载一次`models/xiaolan_unified_seg.pt`，模型同时输出：
`board`、`platform_robot`、`front_feature`、`top_green_feature`。

## 安装

```bash
source /opt/ros/noetic/setup.bash
python3 -m pip install --user -r ~/Grasp_hexapod/src/hexapod_perception/requirements.txt
cd ~/Grasp_hexapod
catkin_make
source devel/setup.bash
```

仓库已包含模型，无需修改模型路径。默认使用CPU；CUDA环境可在启动时传入
`device:=0`。

## 相机前置条件

启动Orbbec相机并确保：

- `/camera/color/image_raw`、`/camera/depth/image_raw`分辨率相同；
- 深度已开启D2C对齐；
- `/camera/color/camera_info`有效；
- TF中存在`base_link <- camera_color_optical_frame`。

## 启动与检查

```bash
roslaunch hexapod_perception perception.launch
rostopic echo /hexapod_perception/status
rostopic echo /hexapod_perception/xiaolan_pose
rostopic echo /hexapod_perception/board_pose
```

主要输出为`board_pose`、`xiaolan_pose`、对应的`*_position`以及JSON
状态话题`/hexapod_perception/status`。位置单位为米，姿态为四元数，默认表达在
`base_link`。

模型SHA-256：
`4c213640dc81177f2375d395226c15cd1e840f43dae5a5770e1989b039a27feb`。

训练使用491张联合标注图，其中缺失类别由原三个教师模型补充伪标签。部署前仍需
在最终相机、光照和贴装角度下进行实机回归测试。
