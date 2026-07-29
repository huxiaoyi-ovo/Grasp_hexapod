"""攀爬模式接口，目前尚未实现具体攀爬状态机。

计划功能：
    根据平台几何、IMU和足端接触信息，规划单腿落脚、支撑转移以及机身
    高度和姿态，最终稳定到达小蓝背部。
计划输入：
    当前足端/关节状态、IMU姿态与角速度、六足接触状态和平台观测。
计划输出：
    六个足端目标、机身目标、攀爬阶段以及成功/失败状态。
结构：
    对齐 -> 前腿探测 -> 逐腿攀爬 -> 机身转移 -> 后腿登顶 -> 稳定。
边界：
    不直接调用Isaac Gym、ROS或舵机SDK，关节解算继续由control.py完成。
"""


class ClimbMode:
    """攀爬模式接口；具体接触状态机将在下一阶段实现。"""

    def __init__(self, controller):
        self.controller = controller

    def update(self, command):
        raise NotImplementedError("Climb mode is not implemented yet")
