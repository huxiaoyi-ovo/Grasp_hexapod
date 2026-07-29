"""固定六个足端、仅调整机身位姿的视觉对接模式。

DockMode由对接协作者实现，内部只包含两个核心部分：

1. 底部相机感知
    - 订阅机身底部后置相机的图像；需要时同时订阅相机内参。
    - 从图像中识别AprilTag或插销，持续估计插销相对相机的位姿。
    - 相机不在机身中心，卡紧机构位于机身底部中心，因此必须结合标定外参，
      把相机观测转换成“插销相对卡紧机构”的位姿，不能直接使用图像误差。
    - 保存时间戳、置信度和是否有效；图像过期、识别失败时不得继续累积运动。
    - 仿真和实机应使用相同的感知输出定义，图像来源由各自启动端配置。

2. 相对移动目标输出
    - 根据最新相对位姿计算机身本周期需要平移和旋转的微小增量。
    - 进入DOCK时记录六个足端支撑位置；机身移动期间足端在环境中保持不动。
    - 对接模式所需的机身运动解算在本文件中实现：把视觉误差转换为目标机身
      位姿，再在六个足端固定的条件下反求新base_link坐标系中的足端坐标。
    - 可以调用kinematics.py已有的坐标变换、正运动学和雅可比等基础函数；
      对接专用的刚体反变换和误差控制函数由协作者继续在本文件中补充。
    - 根据位置和姿态误差判断继续调整、请求锁紧、成功或失败。
    - control.py拒绝候选时保持上一次已接受目标，不能在错误目标上继续积分。

数据和坐标约定：
    +x向右、+y向前、+z向上；位置单位m，角度单位rad。
    A_from_B表示把B坐标系中的量转换到A坐标系。
    视觉链路应明确包含相机、AprilTag、插销和卡紧机构之间的标定变换。

DockMode.update()必须向control.py提供：
    foot_positions_base：shape=(6,3)的足端候选，位于base_link坐标系。
    active、success、failed、request_lock：当前对接状态。
    reason：识别失效、调整失败或底层拒绝等原因。

职责边界：
    DockMode负责相机订阅、图像处理、相对位姿估计和机身移动目标生成。
    DockMode输出的是六个足端坐标，而不是关节角；control.py负责工作空间、
    碰撞、关节限位、DLS关节解算和唯一模式管理。
    DockMode不直接操作Isaac Gym、舵机SDK或发送关节命令，也不绕过control.py
    修改最终足端目标。视觉对准后仍需等待卡紧机构反馈才能宣布对接成功。
"""


class DockMode:
    """最小占位接口；视觉感知和调整逻辑由对接协作者实现。"""

    def __init__(self, controller):
        self.controller = controller
        self.active = False
        self.foot_anchors_base = None

    def enter(self, foot_positions_base):
        """进入DOCK并记录六个固定足端。"""
        self.foot_anchors_base = foot_positions_base.copy()
        self.active = True

    def update(
        self,
        robot_state,
        target_accepted=True,
        reject_reason="",
    ):
        """读取最新相机结果并输出本周期的固定足端候选。"""
        raise NotImplementedError("DockMode is not implemented yet")

    def exit(self):
        """退出DOCK并清空固定足端。"""
        self.active = False
        self.foot_anchors_base = None
