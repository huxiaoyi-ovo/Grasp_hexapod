#!/usr/bin/env python3
"""遥控器节点：把 /joy 原始按键/摇杆转换为语义命令 RemoteCmd。

行为树通过 /grasp_hexapod/remote_cmd 消费（WaitResetCommand 消费
reset_edge(B)、WaitMotionEnabled 消费 enable_edge(A)，manual_climb/dock
消费 climb/dock 边沿，manual_walk 消费摇杆速度）。本节点纯做语义转换，
不含任何运动控制逻辑（行走/攀爬/对接由控制栈承担）。

按键映射（与 run_real.py 一致）：
    A=使能(enable_edge)  B=复位(reset_edge)  X=攀爬(climb_edge)  Y=对接(dock_edge)
轴映射：
    axis_right=0（右，scale -1） axis_forward=1（前） axis_yaw=3（偏航）
    死区外 -> mode="walk" 且输出 vx/vy/vyaw（限幅）；死区内 -> mode="idle"。

用法：
    rosrun grasp_hexapod_bt remote_control.py
    python3 remote_control.py --selftest     # 离线自检（不依赖 ROS）
"""

import argparse

DEFAULTS = dict(
    button_a=0, button_b=1, button_x=2, button_y=3,
    axis_right=0, axis_forward=1, axis_yaw=3,
    right_scale=-1.0, forward_scale=1.0, yaw_scale=1.0,
    deadzone=0.10, max_vx=0.3, max_vy=0.3, max_vyaw=0.5,
)


class RemoteMapper:
    """joy -> RemoteCmd 字段的纯逻辑映射（可离线测试）。

    边沿事件只在按键"抬起->按下"时置 True 一帧；速度经死区与限幅。
    """

    def __init__(self, **kwargs):
        cfg = dict(DEFAULTS)
        cfg.update(kwargs)
        self.cfg = cfg
        self.prev_buttons = [False] * 16

    @staticmethod
    def _read(values, index):
        if 0 <= index < len(values):
            return values[index]
        return 0.0

    def _axis(self, axes, index, scale, limit):
        raw = self._read(axes, index) * scale
        if abs(raw) < self.cfg["deadzone"]:
            return 0.0
        return max(-limit, min(limit, raw))

    def _edge(self, buttons, index):
        pressed = bool(self._read(buttons, index))
        edge = pressed and not self.prev_buttons[index]
        if index < len(self.prev_buttons):
            self.prev_buttons[index] = pressed
        return edge

    def map(self, buttons, axes):
        """输入一组 joy buttons/axes，输出 RemoteCmd 字段 dict。"""
        reset_edge = self._edge(buttons, self.cfg["button_b"])
        enable_edge = self._edge(buttons, self.cfg["button_a"])
        climb_edge = self._edge(buttons, self.cfg["button_x"])
        dock_edge = self._edge(buttons, self.cfg["button_y"])

        vx = self._axis(axes, self.cfg["axis_forward"],
                        self.cfg["forward_scale"], self.cfg["max_vx"])
        vy = self._axis(axes, self.cfg["axis_right"],
                        self.cfg["right_scale"], self.cfg["max_vy"])
        vyaw = self._axis(axes, self.cfg["axis_yaw"],
                          self.cfg["yaw_scale"], self.cfg["max_vyaw"])

        if climb_edge:
            mode = "climb"
        elif dock_edge:
            mode = "dock"
        elif vx or vy or vyaw:
            mode = "walk"
        else:
            mode = "idle"
        return {"mode": mode, "vx": vx, "vy": vy, "vyaw": vyaw,
                "reset_edge": reset_edge, "enable_edge": enable_edge,
                "climb_edge": climb_edge, "dock_edge": dock_edge}


def run_node(args):
    import rospy
    from sensor_msgs.msg import Joy
    from grasp_hexapod_msgs.msg import RemoteCmd

    rospy.init_node("remote_control")
    cfg = {k: type(v)(rospy.get_param("~" + k, v)) for k, v in DEFAULTS.items()}
    publish_hz = float(rospy.get_param("~publish_hz", 10.0))

    mapper = RemoteMapper(**cfg)
    pub = rospy.Publisher("/grasp_hexapod/remote_cmd", RemoteCmd, queue_size=2)
    state = {"cmd": None, "dirty": False}

    def on_joy(message):
        state["cmd"] = mapper.map(message.buttons, message.axes)
        state["dirty"] = True

    def publish(_event):
        if state["cmd"] is None:
            return
        cmd = dict(state["cmd"])
        # 边沿事件只发布一次
        if state["dirty"]:
            state["dirty"] = False
        else:
            for key in ("reset_edge", "enable_edge", "climb_edge", "dock_edge"):
                cmd[key] = False
        msg = RemoteCmd()
        msg.mode = cmd["mode"]
        msg.vx = cmd["vx"]
        msg.vy = cmd["vy"]
        msg.vyaw = cmd["vyaw"]
        msg.reset_edge = cmd["reset_edge"]
        msg.enable_edge = cmd["enable_edge"]
        msg.climb_edge = cmd["climb_edge"]
        msg.dock_edge = cmd["dock_edge"]
        pub.publish(msg)

    rospy.Subscriber("/joy", Joy, on_joy, queue_size=1)
    rospy.Timer(rospy.Duration(1.0 / publish_hz), publish)
    rospy.loginfo("遥控器节点就绪: /joy -> /grasp_hexapod/remote_cmd "
                  "(A=使能 B=复位 X=攀爬 Y=对接, 摇杆=行走)")
    rospy.spin()


def selftest():
    mapper = RemoteMapper()
    A, B, X, Y = 0, 1, 2, 3

    # --- 1. A 键使能边沿（只触发一帧） ---
    cmd = mapper.map([True, False, False, False], [0.0] * 8)
    assert cmd["enable_edge"] and cmd["mode"] == "idle", cmd
    cmd = mapper.map([True, False, False, False], [0.0] * 8)  # 按住不放
    assert not cmd["enable_edge"], cmd
    print("[OK] A 键使能边沿（按住不重复触发）")

    # --- 2. B 键复位边沿 ---
    cmd = mapper.map([False, False, False, False], [0.0] * 8)  # 抬起
    assert not any((cmd["reset_edge"], cmd["enable_edge"]))
    cmd = mapper.map([False, True, False, False], [0.0] * 8)
    assert cmd["reset_edge"], cmd
    print("[OK] B 键复位边沿")

    # --- 3. 摇杆前推 -> walk，vx>0 ---
    cmd = mapper.map([False] * 4, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert cmd["mode"] == "walk" and cmd["vx"] > 0 and cmd["vy"] == 0, cmd
    print("[OK] 摇杆前推: mode={} vx={:.2f}".format(cmd["mode"], cmd["vx"]))

    # --- 4. 死区内 -> idle，速度 0 ---
    cmd = mapper.map([False] * 4, [0.0, 0.05, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0])
    assert cmd["mode"] == "idle" and cmd["vx"] == 0 and cmd["vyaw"] == 0, cmd
    print("[OK] 死区: mode={} 速度清零".format(cmd["mode"]))

    # --- 5. 限幅 ---
    cmd = mapper.map([False] * 4, [0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert cmd["vx"] == 0.3, cmd
    cmd = mapper.map([False] * 4, [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0])
    assert abs(cmd["vyaw"]) == 0.5, cmd
    print("[OK] 速度限幅: vx={:.2f} vyaw={:.2f}".format(cmd["vx"], abs(cmd["vyaw"])))

    # --- 6. 右轴 scale=-1 -> vy 反号 ---
    cmd = mapper.map([False] * 4, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert cmd["vy"] < 0, cmd
    print("[OK] 右轴符号统一: vy={:.2f}".format(cmd["vy"]))

    # --- 7. X/Y 键攀爬/对接边沿 ---
    cmd = mapper.map([False] * 4, [0.0] * 8)
    cmd = mapper.map([False, False, True, False], [0.0] * 8)
    assert cmd["climb_edge"] and cmd["mode"] == "climb", cmd
    cmd = mapper.map([False] * 4, [0.0] * 8)
    cmd = mapper.map([False, False, False, True], [0.0] * 8)
    assert cmd["dock_edge"] and cmd["mode"] == "dock", cmd
    print("[OK] X/Y 键: climb/dock 边沿")

    print("selftest 全部通过")


def main():
    parser = argparse.ArgumentParser(description="遥控器语义化节点（/joy -> RemoteCmd）")
    parser.add_argument("--selftest", action="store_true", help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()

    if args.selftest:
        selftest()
        return

    run_node(args)


if __name__ == "__main__":
    main()
