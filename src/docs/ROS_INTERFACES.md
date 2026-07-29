# ROS接口与自动接近约定

## 坐标系

- `pv_map`：固定在光伏板上的导航坐标系，边界和全局位姿都使用它。
- `base_link`：六足机身坐标系，`+x`向右、`+y`向前、`+z`向上。
- `xiaolan_base`：小蓝机身坐标系。
- ROS位姿和边界必须带时间戳与`frame_id=pv_map`。

控制器内部的变换命名为`target_from_source`：

```text
point_pv = pv_from_base @ point_base
point_pv = pv_from_xiaolan @ point_xiaolan
```

## 协作者输出话题

| 默认话题 | ROS类型 | 含义 |
|---|---|---|
| `/grasp_hexapod/navigation/base_pose` | `geometry_msgs/PoseStamped` | 六足在`pv_map`中的位姿 |
| `/grasp_hexapod/navigation/xiaolan_pose` | `geometry_msgs/PoseStamped` | 小蓝在`pv_map`中的位姿和朝向 |
| `/grasp_hexapod/navigation/pv_boundary` | `geometry_msgs/PolygonStamped` | 光伏板可落脚区域边界 |
| `/grasp_hexapod/landing_confirmed` | `std_msgs/Bool` | 六足已经稳定落板 |

话题名可以在`run_real.launch`中重映射。位姿超过`max_pose_age`、坐标系
不正确或边界少于三个点时，`NavigationState.valid=False`，自动接近不得启动。

RTK天线坐标到`base_link`/`xiaolan_base`的外参、RTK与IMU融合、小蓝朝向
估计以及光伏板边界识别由感知侧完成。控制器不直接解释经纬度。

## 内部接口

`run_real.py`把ROS消息转换为：

```python
NavigationState(
    stamp,
    valid,
    landing_confirmed,
    pv_from_base,
    pv_from_xiaolan,
    pv_boundary,
)
```

Isaac Gym后续也构造相同结构，因此自动接近算法不区分仿真和实机。

## 自动接近配置

自动接近前必须测定两个固定变换：

```text
xiaolan_from_left_base
xiaolan_from_right_base
```

它们分别表示六足处于小蓝左/右侧标准攀爬准备位姿时，目标`base_link`
在`xiaolan_base`中的位姿。未配置这两个变换时，自动接近会明确拒绝启动。

## ApproachMode固定策略

1. 将左右标准接近位姿转换到`pv_map`。
2. 用六足安全包络检查两条直线路径是否位于光伏板边界内。
3. 按行走距离、转角和边界余量评分，选择可行代价最低的一侧。
4. 先原地对齐目标偏航，再全向直线平移。
5. 每帧检查六个计划落脚点的光伏板边界余量。
6. 到达目标后给零指令，让当前摆动组落地并六足停稳。
7. 输出`ready_for_climb=True`，由总状态机决定是否切换`CLIMB`。

这里不使用RRT或A*，也不承担障碍物绕行。若两条固定路径均不满足边界
安全条件，模式返回失败并保持停止，不能选择一条“最接近可行”的危险路径。
