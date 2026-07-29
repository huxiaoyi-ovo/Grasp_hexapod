"""固定六足支撑下的视觉对接模式接口。

功能定位
--------
DockMode由两个主要模块组成：

1. DockPerception
   负责视觉感知，持续读取和更新插销相对卡紧机构的位姿。
2. DockAdjustment
   根据最新相对位姿生成机身的相对运动指令。

DockMode还负责记录进入对接时的六个固定足端，并把机身相对运动反变换为
六个base_link足端目标。工作空间、碰撞、DLS和关节执行仍由control.py负责。

坐标与单位
----------
项目统一使用+x向右、+y向前、+z向上，长度单位m，角度单位rad。
齐次变换命名A_from_B表示把B坐标转换到A坐标：

    point_A = A_from_B @ point_B

底部相机后置安装，卡紧机构位于机身底部中心，两者不是同一个坐标系。
感知模块必须考虑相机、AprilTag、插销和卡紧机构之间的固定外参，最终输出
robot_dock_from_pin，而不能把camera_from_tag直接作为对接误差。

协作者需要实现
--------------
感知模块：

    获取或接收视觉数据
    检测AprilTag或其他对接目标
    计算插销相对卡紧机构的位姿
    持续更新位姿、时间戳、有效性和置信度
    处理观测丢失、过期和明显异常

调整模块：

    根据相对位姿判断机身需要如何调整
    输出一次性的机身相对运动变换
    管理对接任务的运行、成功、失败和卡紧请求
    根据底层目标接受/拒绝反馈调整后续指令

机身反变换：

    进入DOCK时记录六个固定足端
    累积调整模块给出的机身相对运动
    调用项目GraspKinematic提供的坐标变换能力
    计算六个足端在新base_link中的目标坐标
    将足端候选交给control.py检查和执行

开发边界
--------
DockMode负责感知、机身相对运动决策和固定足端反变换，但不得：

    直接修改controller.foot_desired_base
    计算或发送18个关节角
    访问舵机ID和舵机SDK
    绕过control.py的工作空间、碰撞和限位保护

相机采集、ROS订阅或仿真真值可以采用不同适配方式，但都必须转换为相同的
DockPerceptionResult；仿真和实机必须复用相同的DockAdjustment。
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class DockPerceptionResult:
    """感知模块输出的最新对接相对位姿。"""

    # 插销相对卡紧机构的4×4齐次变换。
    robot_dock_from_pin: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )
    valid: bool = False
    timestamp: float = 0.0
    confidence: float = 0.0


@dataclass
class DockMotionCommand:
    """调整模块输出给control.py的一次性相对运动指令。"""

    # 当前机身目标到下一机身目标的4×4相对变换。
    # control.py只消费一次，并负责累积、平滑、限速和可行性检查。
    body_motion: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )
    active: bool = False
    success: bool = False
    failed: bool = False
    request_lock: bool = False
    reason: str = ""


@dataclass
class DockTarget:
    """DockMode完成机身反变换后交给control.py的足端候选。"""

    # 六个固定足端在期望base_link中的位置，shape=(6,3)，单位m。
    foot_positions_base: np.ndarray = field(
        default_factory=lambda: np.zeros((6, 3), dtype=np.float64)
    )
    active: bool = False
    success: bool = False
    failed: bool = False
    request_lock: bool = False
    reason: str = ""


class DockPerception:
    """协作者实现的对接视觉感知模块。"""

    def update(self, sensor_input) -> DockPerceptionResult:
        """读取本周期视觉输入并返回最新相对位姿。"""

        # TODO(collaborator):
        # 实现视觉检测、坐标变换和观测状态更新。
        raise NotImplementedError("Dock perception is not implemented yet")


class DockAdjustment:
    """协作者实现的机身相对运动指令生成模块。"""

    def reset(self):
        """开始新任务时清空内部状态。"""

        # TODO(collaborator): 初始化调整算法内部状态。

    def update(
        self,
        perception: DockPerceptionResult,
        target_accepted: bool,
        reject_reason: str,
    ) -> DockMotionCommand:
        """根据最新对接位姿和底层反馈生成一次相对运动指令。"""

        # TODO(collaborator):
        # 实现对接调整决策和任务结果判断。
        raise NotImplementedError("Dock adjustment is not implemented yet")


class DockMode:
    """组合感知、调整和固定足端机身反变换的对接模式入口。"""

    def __init__(self, controller):
        # 只保留公共运动学对象，不允许通过它绕过control.py的安全检查。
        self.dt = controller.dt
        self.kinematic = controller.kinematic
        self.perception = DockPerception()
        self.adjustment = DockAdjustment()
        self.active = False
        self.foot_anchors_reference = None
        self.reference_from_body_target = np.eye(
            4,
            dtype=np.float64,
        )

    def enter(self, foot_positions_base):
        """开始新任务，并记录进入DOCK时的六个固定足端。"""
        self.foot_anchors_reference = np.asarray(
            foot_positions_base,
            dtype=np.float64,
        ).reshape(6, 3).copy()
        self.reference_from_body_target[:] = np.eye(4)
        self.active = True
        self.adjustment.reset()

    def solve_body_motion(
        self,
        body_motion,
    ) -> np.ndarray:
        """将一次机身相对运动转换成六个固定足端的base_link目标。

        TODO(collaborator):
        累积body_motion，并根据foot_anchors_reference完成机身刚体反变换。
        可以调用self.kinematic中的坐标变换能力，但不得计算或发送关节角。
        返回值必须是shape=(6,3)、单位m的足端候选。
        """
        raise NotImplementedError(
            "Dock body-motion transform is not implemented yet"
        )

    def update(
        self,
        sensor_input,
        target_accepted=True,
        reject_reason="",
    ) -> DockTarget:
        """完成感知和调整，并输出六个固定足端的候选目标。"""
        perception = self.perception.update(sensor_input)
        motion_command = self.adjustment.update(
            perception,
            target_accepted,
            reject_reason,
        )
        foot_positions_base = self.solve_body_motion(
            motion_command.body_motion
        )
        return DockTarget(
            foot_positions_base=foot_positions_base,
            active=motion_command.active,
            success=motion_command.success,
            failed=motion_command.failed,
            request_lock=motion_command.request_lock,
            reason=motion_command.reason,
        )

    def exit(self):
        """结束或取消当前对接任务。"""
        self.active = False
        self.foot_anchors_reference = None
