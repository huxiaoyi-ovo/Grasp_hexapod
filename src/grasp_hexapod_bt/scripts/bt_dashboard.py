#!/usr/bin/env python3
"""行为树 Web 实时看板 v2：完整树可视化 + 模式控制 + 模拟注入 + LoRa 地面站仿真。

单进程 ThreadingHTTPServer（默认 0.0.0.0:8080，参数 ~port/~host），页面
自包含（无 rosbridge / 无外部 CDN，离线可用），1s 轮询 /state.json，载荷
未变不重渲染。本节点**不再依赖 sim_manual**，模拟发布/模式服务调用全部
自持。

页签：
    主控台      同屏四合一（/remote 滚动定位到远端链路面板）：
                左侧行为树完整树（订阅 /grasp_hexapod/bt_state，无数据时显示
                静态完整树模板灰显，运行时 import hexapod_bt 生成主链+遥控
                测试链全节点结构，失败回退内嵌 JSON 副本；颜色 SUCCESS 绿 /
                FAILURE 红 / RUNNING 蓝呼吸灯 / 当前节点青色描边 / INVALID
                灰暗，活跃路径高亮）。
                右侧双列控制栏：
                  控制列：模式控制（直接调 /grasp_hexapod/switch_mode 8 模式
                    + /grasp_hexapod/gripper_act，抢占式仲裁，行为树/遥控
                    模式下均可手动切换）；模拟模式服务（可选托管
                    switch_mode：**不立刻完成**——每次调用固定等待 N 秒
                    （默认 5s，~sim_mode_delay / 页面可调）后返回完成，
                    不同模式互相抢占；真实执行端在跑时占用探测拒绝开启）；
                    模拟注入开关（可选周期发布通过
                    状态自检：传感器自检 -> /grasp_hexapod/sensor_health
                    2Hz 全在线+编码器帧、落地确认 -> encoder_state
                    landed=true、RTK 精准 -> /fix 对角 0.01；一次性按钮：
                    确认落地/未落地/RTK 良好/超限/分路异常。与真实驱动
                    同话题，仅仿真/调试用）。
                  远端列（LoRa 地面站仿真）：PTY 虚拟串口（默认，pty.openpty()
                    建虚拟串口对，本节点持 master，真实 lora_node 从 slave
                    收发，命令组帧 "CMD,HEX,<OP>,NOW*CK\r\n" 字节和 mod 256
                    校验，完整走真实串口协议，不影响真实 lora_node）/话题
                    直发（备选，勿与真实 lora_node 同启）/自动手动放行
                    （手动=按 active_phase 点亮命令按钮等待点击；自动=收到
                    状态反馈后 2s 自动发送当前步骤放行命令，每阶段一次）+
                    机器人状态（bt_state 阶段/任务 + STA 回传历史）。
    虚拟手柄    原始帧发布 /joy（sensor_msgs/Joy，与实体手柄同话题同格式，
                axes[8]/buttons[11]），语义转换仍在 remote_control*。

接口：
    GET  /            页面（主控台）
    GET  /remote      同一页面，滚动定位到远端链路面板
    GET  /state.json  全量快照 JSON（bt/sim/mode/lora/remote + server_now）
    GET  /joy.json    手柄链路状态
    POST /mode        {"mode":"climb"}            -> switch_mode 服务
    POST /gripper     {"action":"open"}           -> gripper_act 服务
    POST /sim         {"toggle":"sensors"} | {"once":"landed"|...}
    POST /lora_tx     {"op":"DEPLOY"}             -> 经当前链路发 LoRa 命令
    POST /lora_cfg    {"mode":"pty"|"topic"|"off"} | {"spawn":true|false}
    POST /joy         {"axes":[...],"buttons":[...]} 即时发布 /joy

用法：
    rosrun grasp_hexapod_bt bt_dashboard.py                      # 默认 8080，PTY 自动开
    rosrun grasp_hexapod_bt bt_dashboard.py _port:=9000 _lora_mode:=topic
    python3 bt_dashboard.py --selftest                           # 离线自检（不依赖 ROS）

性能设计：/state.json 按版本号缓存（任何子状态变化才重新序列化），HTTP
keep-alive 复用连接；2s 倒计时由后端 fire_at 时间戳 + 前端时钟偏移推算，
不增加轮询频率。
"""

import argparse
import collections
import json
import os
import pty
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# 常量（话题/服务/口径与 run_real_bt.py、lora_node.py、sim_manual.py 一致）
# ---------------------------------------------------------------------------
BT_STATE_TOPIC = "/grasp_hexapod/bt_state"
LORA_COMMAND_TOPIC = "/lora/command"
LORA_STATUS_TOPIC = "/lora/status"
SENSOR_HEALTH_TOPIC = "/grasp_hexapod/sensor_health"
ENCODER_TOPIC = "/grasp_hexapod/encoder_state"
FIX_TOPIC = "/fix"
REMOTE_CMD_TOPIC = "/grasp_hexapod/remote_cmd"
JOY_TOPIC = "/joy"                  # 虚拟手柄与实体手柄同话题
SWITCH_MODE_SERVICE = "/grasp_hexapod/switch_mode"
GRIPPER_SERVICE = "/grasp_hexapod/gripper_act"

SENSOR_NAMES = ("imu", "gps", "rtk", "servo", "stereo", "mono")
BT_STALE_S = 5.0                    # bt_state 超时 -> 未连接横幅（树转灰显）
JOY_STALE_S = 1.0                   # 手柄页停报 -> watchdog 补发全零帧（松手）
LORA_OPS = ("RECOVER", "RELEASE", "DEPLOY", "HOIST_DONE", "HOME", "ABORT",
            "PAUSE", "RESUME", "BOGUS")
AUTO_DELAY_S = 2.0                  # 自动模式：状态反馈后延迟发送秒数

MODE_DEFS = [                       # 模式控制页按钮（label 用 hexapod_bt 口径）
    {"mode": "home", "label": "回到初始姿态", "full": "回到初始姿态(含复位)"},
    {"mode": "walk", "label": "行走", "full": "行走(连续)"},
    {"mode": "climb", "label": "攀爬", "full": "攀爬到小蓝上"},
    {"mode": "dock", "label": "对接夹紧", "full": "对接夹紧(tag导引+抬腿+夹爪)"},
    {"mode": "spin_search", "label": "自转搜索", "full": "自转搜索小蓝"},
    {"mode": "release", "label": "释放小蓝", "full": "释放小蓝(夹爪open)"},
    {"mode": "approach", "label": "接近导航", "full": "接近导航到攀爬点(RTK粗导航+tag精导航)"},
]

REMOTE_CMDS = [                     # 远端控制台命令按钮
    {"op": "RECOVER", "label": "回收任务", "desc": "CMD,HEX,RECOVER 放行 WaitTaskCommand，开始抓取回收任务"},
    {"op": "RELEASE", "label": "释放任务", "desc": "CMD,HEX,RELEASE 放行 WaitTaskCommand，开始释放小蓝任务"},
    {"op": "DEPLOY", "label": "下放开始", "desc": "CMD,HEX,DEPLOY 放行 WaitDeployment（绞盘下放门）"},
    {"op": "HOIST_DONE", "label": "拉升完成", "desc": "CMD,HEX,HOIST_DONE 放行 WaitWinchHoisted（回收完成门）"},
    {"op": "HOME", "label": "恢复初始", "desc": "CMD,HEX,HOME 放行 WaitHomeCmd（恢复初始姿态门）"},
    {"op": "PAUSE", "label": "暂停", "desc": "CMD,HEX,PAUSE 一级暂停：挂起任务停走等待（RESUME 可恢复）"},
    {"op": "RESUME", "label": "继续", "desc": "CMD,HEX,RESUME 解除一级暂停，任务从原阶段继续"},
    {"op": "ABORT", "label": "急停打断", "desc": "CMD,HEX,ABORT 二级打断：终止任务走失败回退（home 尽力 + FAILED）"},
]

# active_phase 前缀 -> 放行动作（channel: lora=LoRa 命令 / sim=模拟话题）。
# WaitTaskCommand 不自动发任务（由用户点 RECOVER/RELEASE 起任务）；
# "执行 " 前缀 = RunMode 阻塞在 switch_mode 服务，等待其自然返回。
PASS_MAP = [
    ("WaitTaskCommand", None, "等待地面任务命令（手动点 RECOVER / RELEASE 起任务）"),
    ("WaitSensorsReady", ("sim", "sensors_ok"), "注入全健康帧"),
    ("IsSensorDataOk", ("sim", "sensors_ok"), "注入全健康帧"),
    ("IsAbortRequested", None, "任务已打断：失败回退中（home 尽力 + 上报 FAILED），不可恢复"),
    ("WaitDeployment", ("lora", "DEPLOY"), "发送 DEPLOY"),
    ("IsLandingConfirmed", ("sim", "landed"), "注入确认落地"),
    ("WaitRtkPrecise", ("sim", "rtk_good"), "注入良好 /fix"),
    ("WaitWinchHoisted", ("lora", "HOIST_DONE"), "发送 HOIST_DONE"),
    ("WaitHomeCmd", ("lora", "HOME"), "发送 HOME"),
]

# 手动模式下 active_phase 前缀 -> 点亮的命令按钮 + 提示（ops 为空表示去模拟注入页）
SUGGEST_MAP = [
    ("WaitTaskCommand", ("RECOVER", "RELEASE"), "等待任务命令：选择 RECOVER(回收) 或 RELEASE(释放)"),
    ("WaitDeployment", ("DEPLOY",), "等待绞盘下放：发送 DEPLOY 放行"),
    ("WaitWinchHoisted", ("HOIST_DONE",), "等待绞盘回收完成：发送 HOIST_DONE 放行"),
    ("WaitHomeCmd", ("HOME",), "等待恢复初始命令：发送 HOME 放行"),
    ("任务暂停监护", ("RESUME",), "任务暂停中：发送 RESUME 从原阶段继续（ABORT 则终止任务）"),
    ("IsLandingConfirmed", (), "等待落地确认：到「模拟注入」页点确认落地，或开自动模式"),
    ("WaitRtkPrecise", (), "等待 RTK 精准：到「模拟注入」页开 RTK 精准，或开自动模式"),
    ("WaitSensorsReady", (), "等待传感器上线：到「模拟注入」页开传感器自检，或开自动模式"),
    ("IsSensorDataOk", (), "传感器数据异常：到「模拟注入」页恢复健康帧，或开自动模式"),
]


def resolve_pass(phase):
    """active_phase -> (channel, value, label)；无可放行动作时 channel=None。"""
    if not phase:
        return None, None, "未收到行为树数据"
    for prefix, act, label in PASS_MAP:
        if phase.startswith(prefix):
            if act is None:
                return None, None, label
            return act[0], act[1], label
    if phase.startswith("执行 "):
        return None, None, "模式执行中：等待 switch_mode 服务返回（可在模式控制页手动切换/抢占）"
    return None, None, "当前阶段无需放行: {}".format(phase)


def suggest_for_phase(phase):
    """active_phase -> 手动模式建议 {ops, hint}；无建议返回 None。"""
    if not phase:
        return None
    for prefix, ops, hint in SUGGEST_MAP:
        if phase.startswith(prefix):
            return {"ops": list(ops), "hint": hint}
    if phase.startswith("执行 "):
        return {"ops": [], "hint": "模式执行中：等待 switch_mode 服务返回"}
    return None


class SimModeService:
    """模拟 switch_mode 提供者：请求后**固定延迟**再返回完成（不立刻完成）。

    - 完成时刻 = 请求时刻 + delay_s（默认 5s，可随时调整，对新请求生效）。
    - 新模式请求抢占执行中的模式：旧请求按失败返回（与真实模式服务
      抢占语义一致）。
    开启前探测服务占用：已被真实执行端 / sim_feedback / sim_manual 提供时
    拒绝开启。_on_switch 返回 (success, message)，由 factory 适配为服务响应。
    """

    def __init__(self, log=None, notify=None, now=time.time, delay_s=5.0,
                 service_factory=None, service_name=SWITCH_MODE_SERVICE):
        self._log = log or LogBuf()
        self._notify = notify or (lambda: None)
        self._now = now
        self._factory = service_factory
        self._service_name = service_name
        self._lock = threading.Lock()
        self.enabled = False
        self.delay_s = float(delay_s)
        self._svc = None
        self._gen = 0
        self.current = None            # {"mode","deadline","gen"}
        self._probe_cache = (0.0, None)

    # ---- 开关 ----
    def set_enabled(self, on):
        on = bool(on)
        if on == self.enabled:
            return True, "模拟模式服务已是{}".format("开启" if on else "关闭")
        if on:
            if self._factory is None:
                return False, "ROS 服务不可用（未在 ROS 环境运行？）"
            if self._probe_occupied():
                return False, ("switch_mode 已被其他节点提供"
                               "（真实执行端/sim_feedback/sim_manual），"
                               "先停它再开启模拟模式服务")
            self._svc = self._factory(self._on_switch)
            self.enabled = True
            self._log.add(True, "模拟模式服务已开启：switch_mode 请求后固定"
                                "等待 {:.0f}s 再返回完成".format(self.delay_s))
        else:
            self.enabled = False
            if self._svc is not None:
                try:
                    self._svc.shutdown("sim mode off")
                except Exception:  # noqa: BLE001
                    pass
                self._svc = None
            with self._lock:
                self.current = None        # 等待中的请求按抢占失败返回
            self._log.add(True, "模拟模式服务已关闭")
        self._notify()
        return True, "模拟模式服务已{}".format("开启" if on else "关闭")

    def set_delay(self, delay_s):
        try:
            delay = max(0.0, float(delay_s))
        except (TypeError, ValueError):
            return False, "延迟须为数值（秒）"
        with self._lock:
            self.delay_s = delay
        self._log.add(True, "模拟模式延迟已设为 {:.0f}s（对新请求生效）".format(
            delay))
        self._notify()
        return True, "模拟模式延迟已设为 {:.0f}s（对新请求生效）".format(delay)

    def _probe_occupied(self):
        now_t = self._now()
        cached_t, cached = self._probe_cache
        if now_t - cached_t < 5.0 and cached is not None:
            return cached
        occupied = False
        try:
            import rosgraph.masterapi
            master = rosgraph.masterapi.Master("bt_dash_probe")
            occupied = any(name == self._service_name
                           for name, _ in master.getSystemState()[2])
        except Exception:  # noqa: BLE001 —— 无 master 时视为未占用（factory 已判空）
            occupied = False
        self._probe_cache = (now_t, occupied)
        return occupied

    # ---- 服务处理（阻塞） ----
    def _on_switch(self, req):
        mode = (req.target_mode or "").strip()
        with self._lock:
            self._gen += 1
            gen = self._gen
            preempted = self.current["mode"] if self.current else None
            deadline = self._now() + self.delay_s
            self.current = {"mode": mode, "deadline": deadline, "gen": gen}
        self._notify()
        if preempted:
            self._log.add(True, "模拟模式 {} 被抢占（新请求 {}）".format(
                preempted, mode))
        while True:
            time.sleep(0.05)
            with self._lock:
                cur = self.current
                if cur is None or cur["gen"] != gen:
                    return False, "模式 {} 被抢占".format(mode)
                if self._now() >= cur["deadline"]:
                    self.current = None
                    break
        self._notify()
        self._log.add(True, "模拟模式 {} 完成（等待 {:.0f}s）".format(
            mode, self.delay_s))
        return True, "模拟完成: {}（等待 {:.0f}s）".format(mode, self.delay_s)

    def view(self):
        with self._lock:
            cur = ({"mode": self.current["mode"],
                    "deadline": self.current["deadline"]}
                   if self.current else None)
            return {"on": self.enabled, "delay_s": self.delay_s,
                    "current": cur}


# ---------------------------------------------------------------------------
# LoRa 线协议编解码（与 src/reference/lora/scripts/lora_node.py 完全同款）
# ---------------------------------------------------------------------------
class LoRaCodec:
    """串口帧编解码：'<body>*CK\\r\\n'，CK = body 字节和 mod 256 两位大写十六进制。"""

    def __init__(self, enable_checksum=True):
        self.enable_checksum = bool(enable_checksum)
        self.buf = bytearray()
        self.dropped = 0
        self.lines = 0

    @staticmethod
    def checksum(body):
        total = 0
        for byte in body:
            total = (total + byte) % 256
        return "{:02X}".format(total)

    @staticmethod
    def build_frame(body):
        text = body if isinstance(body, str) else body.decode("utf-8", "ignore")
        return "{}*{}\r\n".format(
            text, LoRaCodec.checksum(text.encode("utf-8"))).encode("utf-8")

    def feed(self, chunk):
        """喂入一块字节，返回本次解出的净帧列表（已去校验，处理分片/粘包）。"""
        self.buf.extend(chunk if isinstance(chunk, bytes) else chunk.encode("utf-8"))
        frames = []
        while True:
            idx = self.buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self.buf[:idx])
            del self.buf[:idx + 1]
            self.lines += 1
            text = line.decode("utf-8", "ignore").rstrip("\r")
            frame = self._validate(text)
            if frame is not None:
                frames.append(frame)
        return frames

    def _validate(self, line):
        if not line:
            return None
        if self.enable_checksum:
            if "*" not in line:
                self.dropped += 1
                return None
            body, ck = line.rsplit("*", 1)
            if not body or len(ck) != 2:
                self.dropped += 1
                return None
            if LoRaCodec.checksum(body.encode("utf-8")) != ck.upper():
                self.dropped += 1
                return None
            return body
        return line.rstrip("*").strip()


# ---------------------------------------------------------------------------
# 静态完整树模板（无数据时灰显完整树；运行时从 hexapod_bt 生成，失败回退）
# ---------------------------------------------------------------------------
FALLBACK_TEMPLATES_JSON = (
    "{\"主链\":{\"tree_name\":\"主链\",\"nodes\":["
    "{\"name\":\"任务失败回退\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":0,\"is_leaf\":false},"
    "{\"name\":\"主流程_带安全监视\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"紧急打断监护_每tick\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"传感器恢复超时\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":false},"
    "{\"name\":\"IsSensorDataOk\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":3,\"is_leaf\":true},"
    "{\"name\":\"任务暂停监护_可恢复\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":false},"
    "{\"name\":\"任务阶段序列\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":3,\"is_leaf\":false},"
    "{\"name\":\"WaitTaskCommand\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":4,\"is_leaf\":true},"
    "{\"name\":\"SafetyInit 安全初始化\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":4,\"is_leaf\":false},"
    "{\"name\":\"传感器上线超时\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":5,\"is_leaf\":false},"
    "{\"name\":\"WaitSensorsReady\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"执行 回到初始姿态(含复位)\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":5,\"is_leaf\":true},"
    "{\"name\":\"DeployAndLand 下放落地\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":4,\"is_leaf\":false},"
    "{\"name\":\"下放等待超时\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":5,\"is_leaf\":false},"
    "{\"name\":\"WaitDeployment\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"落地超时\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":5,\"is_leaf\":false},"
    "{\"name\":\"IsLandingConfirmed\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"释放或回收分流\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":4,\"is_leaf\":false},"
    "{\"name\":\"释放分支_释放小蓝\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":5,\"is_leaf\":false},"
    "{\"name\":\"IsReleaseMission\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"执行 释放小蓝 ⑪（夹爪open）\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"ReportStatus\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"WaitWinchHoisted\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"WaitHomeCmd\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"执行 回到初始姿态(含复位)\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"ReportStatus\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"回收分支_抓取回收\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":5,\"is_leaf\":false},"
    "{\"name\":\"IsRecoveryMission\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"ReportStatus\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"定位导航_带RTK精度监视\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":false},"
    "{\"name\":\"RTK等待超时\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":7,\"is_leaf\":false},"
    "{\"name\":\"WaitRtkPrecise\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":8,\"is_leaf\":true},"
    "{\"name\":\"定位导航步骤\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":7,\"is_leaf\":false},"
    "{\"name\":\"执行 自转搜索小蓝 ㉖\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":8,\"is_leaf\":true},"
    "{\"name\":\"执行 接近导航到攀爬点 ㉗（RTK粗导航+tag精导航）\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":8,\"is_leaf\":true},"
    "{\"name\":\"执行 攀爬到小蓝上 ㉘\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"执行 对接夹紧 ㉙㉚（tag导引+抬腿+夹爪clamp+确认）\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"ReportStatus\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"WaitWinchHoisted\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"WaitHomeCmd\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"执行 回到初始姿态(含复位)\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"ReportStatus\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":6,\"is_leaf\":true},"
    "{\"name\":\"ReportStatus\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":4,\"is_leaf\":true},"
    "{\"name\":\"失败处理\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"回到初始姿态(尽力)\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":false},"
    "{\"name\":\"执行 回到初始姿态(含复位)\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":3,\"is_leaf\":true},"
    "{\"name\":\"home不可用也继续\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":3,\"is_leaf\":true},"
    "{\"name\":\"ReportStatus\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true}"
    "]},"
    "\"遥控测试链\":{\"tree_name\":\"遥控测试链\",\"nodes\":["
    "{\"name\":\"遥控器测试链\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":0,\"is_leaf\":false},"
    "{\"name\":\"测试模式_home\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"遥控选择home\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"执行 回到初始姿态(含复位)\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"测试模式_walk\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"遥控选择walk\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"执行 行走\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"测试模式_climb\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"遥控选择climb\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"执行 攀爬到小蓝上 ㉘\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"测试模式_dock\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"遥控选择dock\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"执行 对接夹紧 ㉙㉚（tag导引+抬腿+夹爪clamp+确认）\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"测试模式_spin_search\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"遥控选择spin_search\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"执行 自转搜索小蓝 ㉖\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"测试模式_release\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":false},"
    "{\"name\":\"遥控选择release\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"执行 释放小蓝 ⑪（夹爪open）\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":2,\"is_leaf\":true},"
    "{\"name\":\"空闲待命\",\"status\":\"INVALID\",\"feedback\":\"\",\"depth\":1,\"is_leaf\":true}"
    "]}}"
)


def load_tree_templates():
    """生成两棵树的静态完整模板 dict；hexapod_bt/py_trees 不可用时用内嵌副本。"""
    try:
        import hexapod_bt
        ctx = hexapod_bt.FakeBridge(script={"mission": "recover"})
        trees = [hexapod_bt.snapshot_tree(
                     hexapod_bt.build_hexapod_tree(ctx), tree_name="主链"),
                 hexapod_bt.snapshot_tree(
                     hexapod_bt.build_remote_test_tree(ctx), tree_name="遥控测试链")]
        return {t["tree_name"]: {"tree_name": t["tree_name"], "nodes": t["nodes"]}
                for t in trees}
    except Exception:  # noqa: BLE001 —— 离线/无 py_trees 环境兜底
        return json.loads(FALLBACK_TEMPLATES_JSON)


# ---------------------------------------------------------------------------
# 小工具：线程安全日志缓冲 / 行为树快照容器
# ---------------------------------------------------------------------------
class LogBuf:
    def __init__(self, maxlen=25, notify=None):
        self._deq = collections.deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._notify = notify or (lambda: None)

    def add(self, ok, msg, **extra):
        entry = {"t": time.time(), "ok": bool(ok), "msg": msg}
        entry.update(extra)
        with self._lock:
            self._deq.append(entry)
        self._notify()

    def list(self):
        with self._lock:
            return list(self._deq)


class BtState:
    """最新行为树快照（订阅回调写，HTTP 读，跨线程）。"""

    _WAITING = {"tree_name": "", "root_status": "", "mission_status": "",
                "active_phase": "", "active_feedback": "", "nodes": []}

    def __init__(self, notify=None):
        self._notify = notify or (lambda: None)
        self._lock = threading.Lock()
        self.snapshot = None
        self.stamp = 0.0

    def update(self, snapshot):
        with self._lock:
            self.snapshot = snapshot
            self.stamp = time.time()
        self._notify()

    def view(self):
        with self._lock:
            snap, stamp = self.snapshot, self.stamp
        stale = snap is not None and time.time() - stamp > BT_STALE_S
        if snap is None or stale:
            data = dict(self._WAITING)
            data.update({"received": stamp if snap else 0,
                         "waiting": True, "stale": stale})
            return data
        data = dict(snap)
        data["received"] = stamp
        data["waiting"] = False
        data["stale"] = False
        return data


# ---------------------------------------------------------------------------
# PTY 虚拟串口传输（模拟真实串口：lora_node 打开 slave 路径收发）
# ---------------------------------------------------------------------------
class PtyTransport:
    """pty 串口对：master 由本节点持有读写，slave 路径给真实 lora_node 打开。

    持有 slave fd 防止对端关闭时 master 读抛 EIO；创建后立即对线路置 raw
    （关回显/输出处理），字节流行为等同真实串口。
    """

    def __init__(self):
        self.master_fd, self.slave_fd = pty.openpty()
        tty.setraw(self.slave_fd)          # 关闭 ECHO/OPOST，防止帧被回显/改写
        self.path = os.ttyname(self.slave_fd)

    def read(self, timeout=0.1):
        """读 master（lora_node 写向串口的上行帧字节）；超时返回 b''。"""
        ready, _, _ = select.select([self.master_fd], [], [], timeout)
        if not ready:
            return b""
        return os.read(self.master_fd, 512)

    def write(self, data):
        view = memoryview(data)
        while view:
            n = os.write(self.master_fd, view)
            view = view[n:]

    def close(self):
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


class FakeTransport:
    """离线假串口：写入记录 + 可注入上行字节（自检用）。"""

    def __init__(self):
        self.written = []
        self.to_read = bytearray()
        self.path = "/dev/fake_pty"

    def read(self, timeout=0.1):
        chunk = bytes(self.to_read)
        del self.to_read[:]
        return chunk

    def write(self, data):
        self.written.append(bytes(data))

    def close(self):
        pass


# ---------------------------------------------------------------------------
# LoRa 链路（PTY 虚拟串口 / 话题直发双模式）
# ---------------------------------------------------------------------------
class LoraLink:
    """地面站侧 LoRa 链路仿真。

    pty 模式：组帧写 master -> 真实 lora_node 从 slave 读串口校验解码 ->
    /lora/command -> 行为树；行为树 STA -> /lora/status -> lora_node 写串口
    -> master 读到并解码。topic 模式：直接发布 /lora/command（下行），
    上行 STA 由 /lora/status 话题订阅侧喂回 handle_body()。
    """

    def __init__(self, log=None, notify=None, transport_factory=None,
                 topic_publish=None, now=time.time):
        self._log = log or (lambda *a, **k: None)
        self._notify = notify or (lambda: None)
        self._tf = transport_factory          # 测试注入；None = 真实 PtyTransport
        self._topic_publish = topic_publish   # f(body) -> connected（topic 模式）
        self._now = now
        self._lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self.mode = "off"                     # off | pty | topic
        self.transport = None
        self._reader_stop = threading.Event()
        self._reader_thread = None
        self.codec = LoRaCodec()
        self.frames = collections.deque(maxlen=40)
        self.sta = collections.deque(maxlen=12)
        self.tx_count = 0
        self.rx_count = 0
        self.last_rx_t = 0.0
        self._node_proc = None

    # ---- 模式切换 ----
    def set_mode(self, mode):
        if mode not in ("off", "pty", "topic"):
            return False, "未知链路模式: {}".format(mode)
        with self._lock:
            if mode == self.mode:
                return True, "链路模式已是 {}".format(mode)
            self.mode = mode
        if mode == "pty":
            self._open_pty()
            with self._lock:
                path = self.transport.path if self.transport else ""
            self._log.add(True, "LoRa 链路切换为 PTY 虚拟串口（{}）".format(path))
            return True, "PTY 虚拟串口已开启: {}".format(path)
        self._close_pty()
        self._log.add(True, "LoRa 链路切换为{}".format(
            "话题直发（/lora/command）" if mode == "topic" else "关闭"))
        return True, "链路模式: {}".format(mode)

    def _open_pty(self):
        with self._lock:
            if self.transport is not None:
                return
            self.transport = (self._tf() if self._tf else PtyTransport())
            self._reader_stop.clear()
            self._reader_thread = threading.Thread(
                target=self._reader, daemon=True, name="lora-pty-reader")
            self._reader_thread.start()

    def _close_pty(self):
        """关闭 PTY：slave 路径即将失效，lora_node 子进程一并停止。"""
        with self._lock:
            transport = self.transport
            self.transport = None
        self._reader_stop.set()
        if transport is not None:
            transport.close()
        self._kill_lora_node()

    # ---- 串口读线程 ----
    def _reader(self):
        while not self._reader_stop.is_set():
            with self._lock:
                transport = self.transport
            if transport is None:
                return
            try:
                chunk = transport.read(0.1)
            except OSError:
                time.sleep(0.2)
                continue
            if not chunk:
                time.sleep(0.02)      # 假串口即刻返回空，避免忙轮询
                continue
            for body in self.codec.feed(chunk):
                self.handle_body(body, via="串口")

    # ---- 收发 ----
    def send_cmd(self, op, param="NOW"):
        if op not in LORA_OPS:
            return False, "未知 LoRa 命令: {}".format(op)
        body = "CMD,HEX,{},{}".format(op, param)
        with self._lock:
            mode = self.mode
            transport = self.transport
        frame = LoRaCodec.build_frame(body)
        ck = LoRaCodec.checksum(body.encode("utf-8"))
        if mode == "pty":
            if transport is None:
                return False, "PTY 未开启（远端控制台选择 PTY 模式）"
            with self._tx_lock:
                try:
                    transport.write(frame)
                except OSError as exc:
                    return False, "串口写入失败: {}".format(exc)
            with self._lock:
                self.tx_count += 1
                self.frames.append({"t": self._now(), "dir": "tx",
                                    "body": body, "ck": ck, "via": "串口"})
            self._notify()
            return True, "已发送 {}*{}（PTY 串口 -> lora_node）".format(body, ck)
        if mode == "topic":
            if self._topic_publish is None:
                return False, "话题直发不可用（无 ROS 发布器）"
            connected = self._topic_publish(body)
            with self._lock:
                self.tx_count += 1
                self.frames.append({"t": self._now(), "dir": "tx",
                                    "body": body, "ck": ck, "via": "话题"})
            self._notify()
            return True, "已发布 {} {}（话题直发）".format(
                LORA_COMMAND_TOPIC, body) + ("" if connected else "（无订阅者，运行器未启动？）")
        return False, "LoRa 链路未开启（远端控制台选择 PTY 或话题直发）"

    def handle_body(self, body, via="串口"):
        """处理一帧上行净帧（STA 回传）：记日志/解析/通知（串口与话题共用）。"""
        ck = LoRaCodec.checksum(body.encode("utf-8"))
        with self._lock:
            self.rx_count += 1
            self.last_rx_t = self._now()
            self.frames.append({"t": self._now(), "dir": "rx",
                                "body": body, "ck": ck, "via": via})
            fields = body.split(",")
            if len(fields) >= 3 and fields[0] == "STA":
                self.sta.append({"t": self._now(), "status": fields[2],
                                 "x": fields[3] if len(fields) > 3 else "",
                                 "y": fields[4] if len(fields) > 4 else ""})
        self._notify()

    # ---- lora_node 子进程（可选自动拉起） ----
    def spawn_lora_node(self, on):
        if not on:
            self._kill_lora_node()
            return True, "已停止 lora_node"
        with self._lock:
            mode, transport = self.mode, self.transport
        if mode != "pty" or transport is None:
            return False, "需先开启 PTY 虚拟串口再拉起 lora_node"
        if self._node_proc is not None and self._node_proc.poll() is None:
            return True, "lora_node 已在运行"
        script = _find_lora_script()
        if script is None:
            return False, "未找到 lora 包的 lora_node.py（workspace 未编译？）"
        self._node_proc = subprocess.Popen(
            [sys.executable, script, "_port:=" + transport.path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._log.add(True, "已拉起 lora_node（{} _port:={}）".format(
            script, transport.path))
        return True, "lora_node 已启动（串口 {}）".format(transport.path)

    def _kill_lora_node(self):
        proc = self._node_proc
        self._node_proc = None
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()

    def shutdown(self):
        self._close_pty()

    # ---- 视图 ----
    def view(self):
        with self._lock:
            node_on = (self._node_proc is not None
                       and self._node_proc.poll() is None)
            last_rx = self.last_rx_t
            return {
                "mode": self.mode,
                "pty_open": self.transport is not None,
                "slave_path": self.transport.path if self.transport else "",
                "lora_node_on": node_on,
                "tx": self.tx_count, "rx": self.rx_count,
                "dropped": self.codec.dropped,
                "last_rx_age_s": (self._now() - last_rx) if last_rx else None,
                "frames": list(self.frames)[-14:],
                "sta": list(self.sta),
            }


def _find_lora_script():
    try:
        import rospkg
        path = os.path.join(rospkg.RosPack().get_path("lora"),
                            "scripts", "lora_node.py")
        if os.path.isfile(path):
            return path
    except Exception:  # noqa: BLE001
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in ("../../reference/lora/scripts/lora_node.py",):
        cand = os.path.normpath(os.path.join(here, rel))
        if os.path.isfile(cand):
            return cand
    return None


# ---------------------------------------------------------------------------
# 模拟注入发布器（需求4：开关周期发布，通过状态自检）
# ---------------------------------------------------------------------------
class SimInjector:
    """模拟话题发布器：开关启用后周期发布；一次性按钮单帧发布。

    发布口径与 sim_manual.py 一致（健康帧六路全在线、编码器落地角 135、
    /fix 对角协方差 0.01）。publish 可注入（自检/离线），默认走 ROS。
    """

    TOGGLES = {
        "sensors": "传感器自检通过（sensor_health 2Hz 全在线 + 编码器帧）",
        "landed": "落地确认（开=landed true / 关=false，随编码器帧 2Hz）",
        "rtk": "RTK 精准（/fix 对角 0.01 1Hz）",
    }
    ONCES = ("sensors_ok", "landed", "not_landed", "rtk_good", "rtk_bad",
             "sensor_bad_imu", "sensor_bad_gps", "sensor_bad_rtk",
             "sensor_bad_servo", "sensor_bad_stereo", "sensor_bad_mono",
             "task_bogus")

    def __init__(self, publish=None, lora_send=None, log=None, notify=None,
                 now=time.time):
        self._publish = publish or self._ros_publish   # f(topic, msg) -> connected
        self._lora = lora_send                          # f(op) -> (ok,msg)
        self._log = log or LogBuf(notify=notify or (lambda: None))
        self._notify = notify or (lambda: None)
        self._now = now
        self._lock = threading.Lock()
        self._toggle = {"sensors": False, "landed": False, "rtk": False}
        self._encoder_landed = False   # 编码器帧携带的落地状态（最近一次设置）
        self._tick_count = 0

    # ---- 消息构造（ROS 类型延迟导入） ----
    @staticmethod
    def _health_arr(bad=None):
        from grasp_hexapod_msgs.msg import SensorHealth, SensorHealthArray
        arr = SensorHealthArray()
        for name in SENSOR_NAMES:
            h = SensorHealth()
            h.name, h.online = name, True
            if bad and name == bad:
                h.fresh, h.freq_hz, h.age_s = False, 0.0, 9.99
                h.reason = "看板注入异常"
            else:
                h.fresh, h.freq_hz, h.age_s = True, 50.0, 0.02
                h.reason = ""
            arr.sensors.append(h)
        return arr

    @staticmethod
    def _encoder(landed):
        from grasp_hexapod_msgs.msg import EncoderState
        m = EncoderState()
        m.landed = landed
        m.angle = 135.0 if landed else 45.0
        m.reason = "看板注入:已落地" if landed else "看板注入:未落地"
        return m

    @staticmethod
    def _fix(cov):
        from sensor_msgs.msg import NavSatFix
        m = NavSatFix()
        m.status.status = 0
        m.position_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return m

    def _ros_publish(self, topic, msg):
        import rospy
        registry = getattr(self, "_pub_registry", None)
        if registry is None:
            registry = self._pub_registry = {}
        pub = registry.get(topic)
        if pub is None:
            pub = rospy.Publisher(topic, type(msg), queue_size=5)
            registry[topic] = pub
        deadline = time.time() + 0.6
        while pub.get_num_connections() == 0 and time.time() < deadline:
            time.sleep(0.05)
        pub.publish(msg)
        return pub.get_num_connections() > 0

    # ---- 周期发布循环（0.5s 一拍：传感器/编码器 2Hz，RTK 1Hz） ----
    # 注：IsSensorDataOk 的健康报告要求 sensor_health 与 encoder_state 同时
    # 有帧，故「传感器自检」开关一并发布编码器帧（落地状态=最近一次设置）。
    def tick(self):
        with self._lock:
            sensors = self._toggle["sensors"]
            landed_on = self._toggle["landed"]
            rtk = self._toggle["rtk"]
            encoder_landed = self._encoder_landed
            self._tick_count += 1
            rtk_due = self._tick_count % 2 == 0
        try:
            if sensors:
                self._publish(SENSOR_HEALTH_TOPIC, self._health_arr())
            if sensors or landed_on:
                self._publish(ENCODER_TOPIC, self._encoder(encoder_landed))
            if rtk and rtk_due:
                self._publish(FIX_TOPIC, self._fix(0.01))
        except Exception as exc:  # noqa: BLE001
            self._log.add(False, "周期发布失败: {}".format(exc))

    # ---- 接口 ----
    def toggle(self, key):
        if key not in self._toggle:
            return False, "未知开关: {}".format(key)
        with self._lock:
            self._toggle[key] = not self._toggle[key]
            on = self._toggle[key]
            if key == "landed":
                self._encoder_landed = on     # 开=已落地 / 关=未落地
        self._log.add(True, "{} 已{}".format(
            self.TOGGLES[key], "开启" if on else "关闭"))
        if on:
            self.tick()                       # 立即发一拍，不等下一周期
        return True, "{} {}".format(
            self.TOGGLES[key], "开启" if on else "关闭")

    def once(self, action):
        if action not in self.ONCES:
            return False, "未知模拟动作: {}".format(action)
        try:
            if action == "sensors_ok":
                c1 = self._publish(SENSOR_HEALTH_TOPIC, self._health_arr())
                with self._lock:
                    landed = self._encoder_landed
                c2 = self._publish(ENCODER_TOPIC, self._encoder(landed))
                connected = c1 and c2
                msg = "已发布六路全健康帧 + 编码器帧" + (
                    "" if connected else "（无订阅者）")
            elif action == "landed":
                with self._lock:
                    self._encoder_landed = True
                connected = self._publish(ENCODER_TOPIC, self._encoder(True))
                msg = "已发布确认落地（landed=true，一帧缓存生效；持续帧跟随此状态）" + (
                    "" if connected else "（无订阅者）")
            elif action == "not_landed":
                with self._lock:
                    self._encoder_landed = False
                connected = self._publish(ENCODER_TOPIC, self._encoder(False))
                msg = "已发布未落地（landed=false）" + (
                    "" if connected else "（无订阅者）")
            elif action == "rtk_good":
                connected = self._publish(FIX_TOPIC, self._fix(0.01))
                msg = "已发布 /fix 对角协方差 0.01（良好）" + (
                    "" if connected else "（无订阅者）")
            elif action == "rtk_bad":
                connected = self._publish(FIX_TOPIC, self._fix(9.0))
                msg = "已发布 /fix 对角协方差 9.0（超限，触发 RTK 停走）" + (
                    "" if connected else "（无订阅者）")
            elif action.startswith("sensor_bad_"):
                name = action[len("sensor_bad_"):]
                connected = self._publish(SENSOR_HEALTH_TOPIC,
                                          self._health_arr(bad=name))
                msg = "已注入 {} 不新鲜帧（停走保持；恢复点全健康帧）".format(name) + (
                    "" if connected else "（无订阅者）")
            elif action == "task_bogus":
                if self._lora is None:
                    return False, "LoRa 链路未配置（远端控制台有 BOGUS 按钮）"
                ok, msg = self._lora("BOGUS")
                self._log.add(ok, msg)
                return ok, msg
            else:
                return False, "未知模拟动作: {}".format(action)
        except Exception as exc:  # noqa: BLE001
            msg = "发布失败: {}".format(exc)
            self._log.add(False, msg)
            return False, msg
        self._log.add(True, msg)
        return True, msg

    def view(self):
        with self._lock:
            return {"toggles": dict(self._toggle), "log": self._log.list()}


# ---------------------------------------------------------------------------
# 模式控制（需求3：直接调 switch_mode / gripper_act，行为树/遥控模式下均可）
# ---------------------------------------------------------------------------
class ModeControl:
    def __init__(self, caller=None, log=None, notify=None):
        self._caller = caller or self._ros_call   # f(kind, value) -> (ok, msg)
        self._log = log or LogBuf(notify=notify or (lambda: None))
        self._notify = notify or (lambda: None)
        self._lock = threading.Lock()
        self.busy = False

    def request(self, kind, value):
        with self._lock:
            if self.busy:
                return False, "上一个模式请求仍在执行（服务阻塞式，稍候）"
            self.busy = True
        self._notify()

        def _run():
            try:
                ok, msg = self._caller(kind, value)
            except Exception as exc:  # noqa: BLE001
                ok, msg = False, "调用异常: {}".format(exc)
            with self._lock:
                self.busy = False
            self._log.add(ok, "{}({}) → {}".format(
                "switch_mode" if kind == "mode" else "gripper_act",
                value, msg))
        threading.Thread(target=_run, daemon=True).start()
        return True, "已提交 {}({})（服务阻塞式，结果见日志）".format(
            "switch_mode" if kind == "mode" else "gripper_act", value)

    @staticmethod
    def _ros_call(kind, value):
        import rospy
        if kind == "mode":
            from grasp_hexapod_msgs.srv import SwitchMode
            service, srv, field = SWITCH_MODE_SERVICE, SwitchMode, "target_mode"
        else:
            from grasp_hexapod_msgs.srv import GripperAct
            service, srv, field = GRIPPER_SERVICE, GripperAct, "action"
        try:
            rospy.wait_for_service(service, timeout=5.0)
        except Exception:  # noqa: BLE001
            return False, "服务不可用: {}（先启动模式执行端或 sim 托管）".format(service)
        proxy = rospy.ServiceProxy(service, srv)
        resp = proxy(**{field: value})
        return bool(resp.success), resp.message or ""

    def view(self):
        with self._lock:
            busy = self.busy
        return {"busy": busy, "log": self._log.list()}


# ---------------------------------------------------------------------------
# 自动放行状态机（需求5：反馈后 2s 自动发送当前步骤的通过命令）
# ---------------------------------------------------------------------------
class AutoPilot:
    """自动/手动放行核。

    反馈源：active_phase 变化（bt_state）与 STA 帧到达（串口/话题）。
    自动模式：反馈 -> 2s 倒计时 -> 按 resolve_pass 放行当前步骤
    （lora 走链路、sim 走模拟话题）；每个「阶段进入」只自动发一次
    （阶段离开再进入视为新回合）。tick() 由后台线程 10Hz 驱动。
    """

    def __init__(self, executor, log=None, notify=None, now=time.time,
                 delay_s=AUTO_DELAY_S):
        self._exec = executor                  # f(channel, value) -> (ok, msg)
        self._log = log or LogBuf(notify=notify or (lambda: None))
        self._notify = notify or (lambda: None)
        self._now = now
        self.delay_s = delay_s
        self._lock = threading.Lock()
        self.auto_on = False
        self.phase = ""
        self._episode = 0
        self._fired = collections.deque(maxlen=8)   # (episode, channel, value)
        self.pending = None

    # ---- 反馈入口 ----
    def on_phase(self, phase):
        with self._lock:
            changed = phase != self.phase
            if changed:
                self._episode += 1
                self.phase = phase
                self.pending = None
        if changed:
            self._feedback("阶段变化")

    def on_sta(self):
        self._feedback("STA 回传")

    def set_auto(self, on):
        with self._lock:
            self.auto_on = bool(on)
            self.pending = None
        self._log.add(True, "自动放行已{}".format("开启" if on else "关闭"))
        if on:
            self._feedback("自动开启")
        return True, "自动放行 {}".format("开启" if on else "关闭")

    def _feedback(self, src):
        with self._lock:
            if not self.auto_on:
                return
            episode, phase = self._episode, self.phase
        channel, value, label = resolve_pass(phase)
        if channel is None:
            with self._lock:
                if self.phase == phase:
                    self.pending = None
            return
        with self._lock:
            if (episode, channel, value) in self._fired:
                return
            self.pending = {
                "episode": episode, "phase": phase,
                "channel": channel, "value": value, "label": label,
                "fire_at": self._now() + self.delay_s,
            }
        self._log.add(True, "反馈到达（{}），{:.0f}s 后自动{}（{}）".format(
            src, self.delay_s, label,
            "LoRa 串口" if channel == "lora" else "模拟话题"))

    # ---- 10Hz tick：到点执行 / 阶段已变则取消 ----
    def tick(self):
        with self._lock:
            pending = self.pending
            if pending is None:
                return
            if not self.auto_on:
                self.pending = None
                return
            if self.phase != pending["phase"]:
                self.pending = None
                return
            if self._now() < pending["fire_at"]:
                return
            self._fired.append((pending["episode"], pending["channel"],
                                pending["value"]))
            self.pending = None
        try:
            ok, msg = self._exec(pending["channel"], pending["value"])
        except Exception as exc:  # noqa: BLE001
            ok, msg = False, "执行异常: {}".format(exc)
        self._log.add(ok, "自动放行: {}".format(msg))

    def view(self):
        with self._lock:
            pending = dict(self.pending) if self.pending else None
            return {"auto_on": self.auto_on, "pending": pending}


# ---------------------------------------------------------------------------
# 核心聚合（state.json 装配 + 版本号载荷缓存）
# ---------------------------------------------------------------------------
class DashboardCore:
    def __init__(self):
        self._ver = 0
        self._payload = None
        self._key = None
        self.bt = BtState(notify=self.bump)
        self.remote_cmd = {"mode": "", "t": 0.0}
        self.lora_log = LogBuf(notify=self.bump)
        self.lora = LoraLink(log=self.lora_log, notify=self.bump)
        self.sim_log = LogBuf(notify=self.bump)
        self.sim = SimInjector(log=self.sim_log, notify=self.bump)
        self.mode_log = LogBuf(notify=self.bump)
        self.modes = ModeControl(log=self.mode_log, notify=self.bump)
        self.auto_log = LogBuf(notify=self.bump)
        self.auto = AutoPilot(executor=self._auto_exec, log=self.auto_log,
                              notify=self.bump)
        self.sim_modes = SimModeService(
            log=self.mode_log, notify=self.bump)

    def bump(self):
        self._ver += 1

    def _auto_exec(self, channel, value):
        if channel == "lora":
            return self.lora.send_cmd(value)
        if channel == "sim":
            return self.sim.once(value)
        return False, "未知放行通道: {}".format(channel)

    def on_remote_cmd(self, mode):
        self.remote_cmd = {"mode": mode, "t": time.time()}
        self.bump()

    def payload(self):
        bt_view = self.bt.view()
        key = (self._ver, id(self.bt.snapshot), bt_view["stale"],
               bt_view["waiting"])
        if key == self._key:
            return self._payload
        remote = self.auto.view()
        phase = bt_view["active_phase"]
        remote.update({
            "suggest": None if remote["auto_on"] else suggest_for_phase(phase),
            "phase": phase,
            "mission": bt_view["mission_status"],
            "root": bt_view["root_status"],
            "log": self.auto_log.list(),
        })
        data = {
            "server_now": time.time(),
            "bt": bt_view,
            "sim": self.sim.view(),
            "mode": dict(self.modes.view(),
                         sim_service=self.sim_modes.view()),
            "lora": self.lora.view(),
            "remote": remote,
            "remote_cmd": ({"mode": self.remote_cmd["mode"],
                            "age_s": time.time() - self.remote_cmd["t"]}
                           if self.remote_cmd["mode"] else None),
        }
        self._payload = json.dumps(data, ensure_ascii=False)
        self._key = key
        return self._payload


# ---------------------------------------------------------------------------
# 虚拟手柄 -> /joy（沿用 v1：POST /joy 每帧即时发布 + 停报 watchdog 松手）
# ---------------------------------------------------------------------------
JOY_AXES_LEN = 8
JOY_BUTTONS_LEN = 11


def _parse_joy_body(body):
    if not isinstance(body, dict):
        return None, None, "载荷不是 JSON 对象"
    axes, buttons = body.get("axes"), body.get("buttons")
    if not isinstance(axes, list) or not isinstance(buttons, list):
        return None, None, "axes/buttons 必须是数组"
    if len(axes) > 16 or len(buttons) > 32:
        return None, None, "axes/buttons 超长"
    try:
        axes = [max(-1.0, min(1.0, float(a))) for a in axes]
        buttons = [1 if b else 0 for b in buttons]
    except (TypeError, ValueError):
        return None, None, "axes/buttons 含非数值"
    return axes, buttons, ""


class JoyLink:
    def __init__(self, pub, log=None, now_fn=time.time, make_msg=None):
        self._pub = pub
        self._log = log
        self._now = now_fn
        self._make_msg = make_msg or self._default_msg
        self._lock = threading.Lock()
        self._last_rx = 0.0
        self._active = False

    @staticmethod
    def _default_msg(axes, buttons):
        import rospy
        from sensor_msgs.msg import Joy
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = axes
        msg.buttons = buttons
        return msg

    def _publish_joy(self, axes, buttons):
        self._pub.publish(self._make_msg(axes, buttons))

    def update(self, axes, buttons):
        try:
            self._publish_joy(axes, buttons)
        except Exception as exc:  # noqa: BLE001
            return False, "发布失败: {}".format(exc)
        with self._lock:
            self._last_rx = self._now()
            self._active = True
        return True, "已发布 /joy axes={} buttons={}".format(
            len(axes), len(buttons))

    def watchdog(self):
        with self._lock:
            stale = self._active and self._now() - self._last_rx > JOY_STALE_S
        if not stale:
            return
        try:
            self._publish_joy([0.0] * JOY_AXES_LEN, [0] * JOY_BUTTONS_LEN)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._active = False
        if self._log:
            self._log("虚拟手柄停止上报，已补发全零帧（摇杆回中+按键松开）")

    def view(self):
        with self._lock:
            now = self._now()
            active = self._active and now - self._last_rx <= JOY_STALE_S
            age = (now - self._last_rx) if self._last_rx else None
        try:
            subs = int(self._pub.get_num_connections())
        except AttributeError:
            subs = 0
        return {"active": active, "age_s": age, "subscribers": subs}


# ---------------------------------------------------------------------------
# 页面（自包含 HTML/CSS/JS；模板 JSON 注入占位符）
# ---------------------------------------------------------------------------
_PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<title>Hexapod 行为树 · 实时看板</title>
<style>
  :root { color-scheme: light; }
  * { box-sizing: border-box; }
  body { margin:0; background:#f1f5f9; color:#1e293b;
         font: 14px/1.5 "SF Mono", Consolas, "Noto Sans Mono CJK SC", monospace; }
  header { padding:10px 16px; background:#ffffff; border-bottom:2px solid #e2e8f0;
           display:flex; flex-wrap:wrap; gap:10px; align-items:center; }
  header .title { font-weight:700; font-size:16px; margin-right:auto; }
  .chip { padding:3px 10px; border-radius:12px; font-size:13px; font-weight:700;
          border:1px solid transparent; white-space:nowrap; }
  .chip small { font-weight:400; opacity:.85; }
  .st { display:inline-block; width:9px; height:9px; border-radius:50%;
        margin-right:5px; vertical-align:middle; }
  #staleBar { display:none; background:#ffedd5; color:#9a3412; padding:4px 16px; font-size:12px; }

  #tabs { display:flex; background:#ffffff; border-bottom:1px solid #e2e8f0;
          overflow-x:auto; }
  #tabs button { padding:10px 18px; background:transparent; color:#94a3b8;
                 border:none; border-right:1px solid #ffffff;
                 border-bottom:2px solid transparent; white-space:nowrap;
                 font:inherit; font-size:14px; font-weight:700; cursor:pointer; }
  #tabs button.act { color:#c2410c; background:#fff7ed; border-bottom-color:#f97316; }
  .view { padding:14px 16px 60px; }

  /* ---- 行为树页 ---- */
  #viewBt .legend { display:flex; gap:14px; flex-wrap:wrap; padding:6px 12px;
        background:#ffffff; border:1px solid #e2e8f0; border-radius:8px;
        font-size:12px; color:#94a3b8; margin-bottom:10px; }
  #phaseBox { padding:10px 14px; background:#ffffff; border:1px solid #e2e8f0;
              border-radius:8px; margin-bottom:10px; }
  #phaseBox .label { color:#94a3b8; font-size:12px; }
  #phaseName { font-size:18px; font-weight:700; margin:2px 0; }
  #phaseFb { color:#475569; font-size:13px; min-height:18px; }
  #treeSel { display:flex; gap:8px; margin:0 0 8px; align-items:center; }
  #treeSel span { color:#94a3b8; font-size:12px; }
  #treeSel button { padding:3px 12px; border-radius:6px; cursor:pointer;
        background:#ffffff; color:#1e293b; border:1px solid #e2e8f0; font:inherit; font-size:12px; }
  #treeSel button.act { background:#ea580c; border-color:#c2410c; color:#ffffff; }
  #waiting { display:none; padding:24px; text-align:center; color:#94a3b8;
             font-size:14px; border:1px dashed #e2e8f0; border-radius:8px; margin-bottom:8px; }
  #treeWrap { padding:4px 2px; overflow:auto; }
  ul.tree, ul.tree ul { list-style:none; margin:0; padding:0 0 0 20px; position:relative; }
  ul.tree { padding-left:4px; }
  ul.tree li { position:relative; padding:3px 0 3px 16px; }
  ul.tree li::before { content:""; position:absolute; left:0; top:0; bottom:0;
                       width:1px; background:#e2e8f0; }
  ul.tree li:after  { content:""; position:absolute; left:0; top:16px;
                      width:12px; height:1px; background:#e2e8f0; }
  ul.tree li:last-child::before { bottom:auto; height:16px; }
  .node { display:inline-block; max-width:660px; padding:4px 10px 5px 8px;
          border-radius:6px; border:1px solid #e2e8f0; border-left:3px solid #64748b;
          background:#ffffff; }
  .node .nm { font-weight:600; }
  .node .fb { color:#94a3b8; font-size:12px; margin-top:1px; white-space:normal; }
  .node .dot { display:inline-block; width:9px; height:9px; border-radius:50%;
               margin-right:6px; vertical-align:middle; }
  .node .dot.pulse { animation: pulse 1.6s ease-in-out infinite; }
  @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:.35; } }
  @media (prefers-reduced-motion: reduce) { .node .dot.pulse { animation:none; } }
  .node.r-running { border-left-color:#f97316; background:rgba(249,115,22,.12);
                    border-color:#fdba74; }
  .node.r-success { border-left-color:#22c55e; background:rgba(34,197,94,.10); }
  .node.r-failure { border-left-color:#ef4444; background:rgba(239,68,68,.12); }
  .node.r-invalid { border-left-color:#475569; opacity:.5; }
  .node.onpath { border-color:#fdba74; }
  .node.dimmed { opacity:.42; }
  .node.cur { border-color:#2563eb; border-left-color:#2563eb;
              box-shadow:0 0 0 2px rgba(37,99,235,.40);
              background:rgba(37,99,235,.08); opacity:1; }
  .node .curtag { display:none; margin-left:8px; padding:0 6px; border-radius:8px;
                  font-size:11px; background:#1d4ed8; color:#dbeafe; }
  .node.cur .curtag { display:inline-block; }
  .branch-mark { color:#64748b; font-size:11px; margin-right:3px; }

  /* ---- 主控台布局：树为主区 + 右侧悬浮岛式控制面板（控制列 / 远端列） ---- */
  .mainLayout { display:flex; gap:14px; align-items:flex-start; }
  #treeCol { flex:1 1 auto; min-width:0; }
  /* 悬浮岛：四边 16px 外边距、圆角 20、单层低透明度柔和投影；
     随页面滚动吸附在视口上方 16px（窄屏单列布局自动取消吸附） */
  #sideCol { flex:0 0 768px; display:flex; gap:14px; align-items:flex-start;
             max-height:calc(100vh - 280px); overflow-y:auto;
             margin:16px; padding:6px 18px;
             background:linear-gradient(180deg,#ffffff,#f8fafc);
             border:1px solid #1e293b;
             border-radius:20px;
             box-shadow:0 8px 24px rgba(15,23,42,.12);
             position:sticky; top:16px;
             scrollbar-width:thin; scrollbar-color:#cbd5e1 transparent; }
  #sideCol::-webkit-scrollbar { width:8px; }
  #sideCol::-webkit-scrollbar-thumb { background:#cbd5e1; border-radius:4px; }
  .sideInner { flex:1 1 0; min-width:0; display:flex; flex-direction:column; gap:12px; }
  /* 岛内分区：卡片去底色描边，改用分隔线，保持“一整块浮动面板”的观感 */
  #sideCol .card { margin-bottom:0; background:transparent; border:none;
                   border-radius:0; padding:6px 0 12px;
                   border-bottom:1px solid #1e293b; }
  #sideCol .sideInner .card:last-child { border-bottom:none; }
  #sideCol .card h3 { margin-bottom:6px; }
  #sideA .modeGrid { grid-template-columns:repeat(2, 1fr); }
  @media (max-width:1420px) { #sideCol { flex-basis:660px; } }
  @media (max-width:1200px) {
    .mainLayout { flex-direction:column; }
    #sideCol { flex:none; width:100%; max-height:none; position:static; }
  }
  @media (max-width:760px) { #sideCol { flex-direction:column; } }

  /* ---- 通用按钮/卡片 ---- */
  .card { background:#ffffff; border:1px solid #e2e8f0; border-radius:10px;
          padding:12px 14px; margin-bottom:14px; }
  .card h3 { margin:0 0 10px; font-size:14px; color:#1e293b; }
  .card h3 small { color:#94a3b8; font-weight:400; font-size:12px; }
  button.b { padding:6px 14px; border-radius:8px; cursor:pointer;
        background:#ffffff; color:#1e293b; border:1px solid #e2e8f0;
        font:inherit; font-size:13px; }
  button.b:hover { border-color:#fb923c; }
  button.b:disabled { opacity:.45; cursor:not-allowed; }
  button.b.warn { border-color:#fecaca; color:#dc2626; }
  button.b.on { background:#166534; border-color:#22c55e; }
  .btnrow { display:flex; flex-wrap:wrap; gap:8px; }
  .note { color:#64748b; font-size:11px; margin:6px 0 0; }
  .logbox { max-height:240px; overflow:auto; font-size:12px; }
  .logbox .t { color:#64748b; margin-right:6px; }
  .logbox .bad { color:#dc2626; }
  .logbox .ok { color:#16a34a; }

  /* ---- 模式控制页 ---- */
  .modeGrid { display:grid; grid-template-columns:repeat(auto-fill,minmax(150px,1fr)); gap:10px; }
  button.mode { padding:14px 10px; border-radius:10px; cursor:pointer;
        background:#ffffff; color:#1e293b; border:2px solid #e2e8f0;
        font:inherit; font-weight:700; font-size:15px; }
  button.mode small { display:block; font-weight:400; font-size:11px; color:#94a3b8; margin-top:3px; }
  button.mode:hover { border-color:#fb923c; }
  button.mode:active { transform:scale(.97); }
  button.mode:disabled { opacity:.4; cursor:not-allowed; }
  button.mode.busy { border-color:#f59e0b; }

  /* ---- 模拟注入页 ---- */
  .simrow { display:flex; align-items:center; gap:10px; padding:10px 0;
            border-bottom:1px dashed #e2e8f0; }
  .simrow:last-child { border-bottom:none; }
  .simrow .desc { flex:1; min-width:0; }
  .simrow .desc b { font-size:13px; }
  .simrow .desc span { display:block; color:#64748b; font-size:11px; }
  .sw { width:52px; height:26px; border-radius:13px; background:#e2e8f0;
        border:1px solid #475569; position:relative; cursor:pointer; flex:none;
        transition:background .2s; }
  .sw::after { content:""; position:absolute; top:2px; left:2px; width:20px;
        height:20px; border-radius:50%; background:#94a3b8; transition:left .2s,background .2s; }
  .sw.on { background:#166534; border-color:#22c55e; }
  .sw.on::after { left:28px; background:#dcfce7; }

  /* ---- 远端控制台 ---- */
  .linkstat { display:flex; flex-wrap:wrap; gap:8px; margin:8px 0; }
  .pathline { background:#ffffff; border:1px solid #e2e8f0; border-radius:6px;
              padding:6px 8px; font-size:12px; color:#ea580c; word-break:break-all; }
  .rbtns { display:flex; flex-wrap:wrap; gap:10px; }
  button.cmd { flex:1 1 130px; padding:12px 10px; border-radius:10px; cursor:pointer;
        background:#ffffff; color:#1e293b; border:2px solid #e2e8f0;
        font:inherit; font-weight:700; font-size:14px; text-align:center; }
  button.cmd small { display:block; font-weight:400; font-size:11px; color:#94a3b8; margin-top:2px; }
  button.cmd:hover { border-color:#fb923c; }
  button.cmd:disabled { opacity:.4; cursor:not-allowed; }
  button.cmd.ready { border-color:#f59e0b; box-shadow:0 0 0 2px rgba(245,158,11,.4);
        background:rgba(245,158,11,.12); animation: pulse 1.6s ease-in-out infinite; }
  button.cmd .abadge { display:none; margin-left:6px; padding:0 6px; border-radius:8px;
        font-size:11px; background:#1d4ed8; color:#dbeafe; }
  button.cmd.armed .abadge { display:inline-block; }
  #autoBox { display:flex; gap:12px; align-items:center; flex-wrap:wrap;
             padding:10px 12px; border-radius:8px; background:#ffffff;
             border:1px solid #e2e8f0; margin-bottom:10px; }
  #autoText { flex:1; min-width:200px; font-size:13px; color:#475569; }
  #autoText b { color:#ea580c; }
  .staTable { width:100%; border-collapse:collapse; font-size:12px; }
  .staTable th, .staTable td { text-align:left; padding:3px 6px;
        border-bottom:1px solid #e2e8f0; }
  .staTable th { color:#94a3b8; font-weight:400; }
  .flog { font-size:11px; line-height:1.7; }
  .flog .tx { color:#ea580c; }
  .flog .rx { color:#16a34a; }
  .flog .t { color:#64748b; margin-right:5px; }

  /* ---- 虚拟手柄页 ---- */
  body.joy { overflow:hidden; overscroll-behavior:none;
             display:flex; flex-direction:column;
             height:100vh; height:100dvh; }
  body.joy footer { display:none; }
  body.joy #staleBar { display:none !important; }
  body.joy header .chip { display:none; }
  #viewJoy { display:none; flex:1; min-height:0; flex-direction:column;
             gap:12px; padding:12px; width:100%; max-width:760px;
             margin:0 auto; }
  #joyStatus { display:flex; flex-wrap:wrap; gap:8px; align-items:center; }
  #joyBtns { display:flex; gap:12px; justify-content:center; flex-wrap:wrap; }
  #joyDpad { display:grid; grid-template-columns:repeat(3, 58px);
             grid-template-rows:repeat(3, 46px); gap:5px; justify-content:center; }
  button.dp { border-radius:10px; background:#ffffff; color:#1e293b;
              border:2px solid #e2e8f0; font:inherit; font-weight:700;
              font-size:16px; line-height:1.1; touch-action:none;
              user-select:none; -webkit-user-select:none;
              -webkit-tap-highlight-color:transparent; cursor:pointer; }
  button.dp small { display:block; font-weight:400; font-size:10px; color:#94a3b8; }
  button.dp.on { background:#166534; border-color:#22c55e; }
  button.dp[data-dpad=up] { grid-column:2; grid-row:1; }
  button.dp[data-dpad=left] { grid-column:1; grid-row:2; }
  button.dp[data-dpad=right] { grid-column:3; grid-row:2; }
  button.dp[data-dpad=down] { grid-column:2; grid-row:3; }
  button.pad { min-width:68px; min-height:68px; border-radius:50%;
               background:#ffffff; color:#1e293b; border:2px solid #e2e8f0;
               font:inherit; font-weight:700; font-size:18px; line-height:1.15;
               touch-action:none; user-select:none; -webkit-user-select:none;
               -webkit-tap-highlight-color:transparent; cursor:pointer; }
  button.pad small { display:block; font-weight:400; font-size:11px; color:#94a3b8; }
  button.pad.on { background:#166534; border-color:#22c55e; }
  #joySticks { flex:1; min-height:0; display:flex; align-items:stretch;
               justify-content:space-around; gap:16px; }
  .stickBox { display:flex; flex-direction:column; align-items:center;
              justify-content:flex-end; gap:8px; min-width:0; }
  .stick { position:relative; flex:0 1 auto; min-height:0;
           height:min(38vmin, 250px); aspect-ratio:1/1; border-radius:50%;
           background:#ffffff; border:2px solid #e2e8f0; touch-action:none;
           user-select:none; -webkit-user-select:none;
           -webkit-tap-highlight-color:transparent; }
  .stick::before { content:""; position:absolute; inset:0; margin:auto;
                   width:70%; height:70%; border-radius:50%;
                   border:1px dashed #e2e8f0; }
  .knob { position:absolute; left:50%; top:50%; width:44%; height:44%;
          margin:-22% 0 0 -22%; border-radius:50%; background:#e2e8f0;
          border:2px solid #64748b; pointer-events:none; transition:transform .08s; }
  .stick.live .knob { background:#ea580c; border-color:#fb923c; transition:none; }
  .stickLabel { color:#94a3b8; font-size:12px; white-space:nowrap; }
  .stickLabel span { color:#1e293b; }
  #joyHint { color:#64748b; font-size:11px; text-align:center; }

  #toast { position:fixed; top:12px; right:12px; padding:8px 14px; border-radius:8px;
           background:#f0fdf4; border:1px solid #86efac; color:#166534; font-size:13px;
           opacity:0; transition:opacity .25s; pointer-events:none; max-width:60vw; z-index:9; }
  #toast.show { opacity:1; }
  #toast.bad { background:#fef2f2; border-color:#fca5a5; color:#b91c1c; }
  footer { position:fixed; bottom:0; right:8px; color:#475569; font-size:11px;
           background:#f1f5f9; padding:0 4px; }
</style>
</head>
<body>
<header>
  <span class="title">Hexapod 行为树 · 实时看板</span>
  <span id="chipRoot" class="chip">树状态 —</span>
  <span id="chipMission" class="chip">任务结果 —</span>
  <span id="chipLink" class="chip"><small>LoRa —</small></span>
  <span id="chipTime" class="chip"><small>—</small></span>
</header>
<nav id="tabs">
  <button id="tabBt" class="act">主控台</button>
  <button id="tabJoy">虚拟手柄</button>
</nav>
<div id="staleBar">⚠ 行为树数据超过 5 秒未更新（bt_state 停止？运行器是否在运行）——树转灰显</div>

<div id="viewBt" class="view">
  <div class="legend">
    <span><span class="st" style="background:#f97316"></span>RUNNING 执行中</span>
    <span><span class="st" style="background:#22c55e"></span>SUCCESS 已通过</span>
    <span><span class="st" style="background:#ef4444"></span>FAILURE 未通过</span>
    <span><span class="st" style="background:#2563eb"></span>当前节点</span>
    <span><span class="st" style="background:#64748b"></span>INVALID 未访问</span>
  </div>
  <div id="phaseBox">
    <div class="label">当前运行阶段</div>
    <div id="phaseName">等待数据…</div>
    <div id="phaseFb"></div>
  </div>
  <div class="mainLayout">
    <div id="treeCol">
      <div id="treeSel"><span>树：</span>
        <button data-tree="主链" class="act">主链（任务）</button>
        <button data-tree="遥控测试链">遥控测试链</button>
      </div>
      <div id="waiting">等待 /grasp_hexapod/bt_state……<br>
        灰显为完整树模板。请启动运行器：<b>run_real_bt.py</b>（实机）/ <b>bt_mock_world.py</b>（联调）。</div>
      <div id="treeWrap"></div>
    </div>
    <aside id="sideCol">
     <div class="sideInner" id="sideA">
      <div class="card">
        <h3>模式控制 <small>/grasp_hexapod/switch_mode · 抢占式仲裁</small></h3>
        <p class="note">行为树/遥控模式下均可手动切换（进行中请求被立即终结，WAIT_B 安全门只接受 home）；响应为阻塞式最终结果。</p>
        <div id="modeGrid" class="modeGrid"></div>
        <div class="btnrow" style="margin-top:10px">
          <button class="b" data-gripper="open">夹爪 open（松开）</button>
          <button class="b" data-gripper="clamp">夹爪 clamp（夹紧）</button>
        </div>
        <div class="simrow" style="margin-top:10px"><div class="desc"><b>模拟模式服务</b>
          <span>看板托管 switch_mode：每次调用<b>固定等待 N 秒后返回完成</b>（绝不立刻成功）；
          不同模式互相抢占。真实执行端 / sim_feedback 在跑时请勿开启。</span></div>
          <div class="sw" id="simModeSw"></div></div>
        <div class="btnrow" style="margin-top:8px; align-items:center">
          <label style="font-size:13px">延迟
            <input type="number" id="simModeDelay" min="0" max="600" step="1"
                   style="width:64px; background:#ffffff; color:#1e293b;
                          border:1px solid #e2e8f0; border-radius:6px; padding:3px 6px"> 秒
          </label>
          <button class="b" id="simModeDelayBtn">应用延迟</button>
        </div>
        <p class="note" id="simModeNote"></p>
      </div>
      <div class="card">
        <h3>请求日志 <small id="remoteCmdChip"></small></h3>
        <div id="modeLog" class="logbox"></div>
      </div>
      <div class="card">
        <h3>模拟注入 <small>发布模拟信息通过状态自检（仅仿真/调试用）</small></h3>
        <p class="note">开关周期发布（传感器 2Hz / 编码器 2Hz / RTK 1Hz），停用即停发；一次性按钮单帧生效。真实驱动在运行时会与真实帧互相覆盖。</p>
        <div class="simrow"><div class="desc"><b>传感器自检通过</b>
          <span>/sensor_health 六路全在线 2Hz + 编码器帧 → 放行 IsSensorDataOk / WaitSensorsReady（自检门需两者同时有帧）</span></div>
          <div class="sw" data-sim-toggle="sensors"></div></div>
        <div class="simrow"><div class="desc"><b>落地确认</b>
          <span>/encoder_state 开=landed true（135°）/ 关=false，随传感器帧 2Hz → 放行 IsLandingConfirmed</span></div>
          <div class="sw" data-sim-toggle="landed"></div></div>
        <div class="simrow"><div class="desc"><b>RTK 精准</b>
          <span>/fix 对角协方差 0.01 m²（阈值 0.04）1Hz → 放行 WaitRtkPrecise</span></div>
          <div class="sw" data-sim-toggle="rtk"></div></div>
      </div>
      <div class="card">
        <h3>一次性注入</h3>
        <div class="btnrow">
          <button class="b" data-sim-once="sensors_ok">全健康帧</button>
          <button class="b" data-sim-once="landed">确认落地</button>
          <button class="b" data-sim-once="not_landed">未落地</button>
          <button class="b" data-sim-once="rtk_good">RTK 良好帧</button>
          <button class="b warn" data-sim-once="rtk_bad">RTK 超限帧</button>
          <button class="b warn" data-sim-once="sensor_bad_imu">imu 异常</button>
          <button class="b warn" data-sim-once="sensor_bad_gps">gps 异常</button>
          <button class="b warn" data-sim-once="sensor_bad_rtk">rtk 异常</button>
          <button class="b warn" data-sim-once="sensor_bad_servo">servo 异常</button>
          <button class="b warn" data-sim-once="sensor_bad_stereo">stereo 异常</button>
          <button class="b warn" data-sim-once="sensor_bad_mono">mono 异常</button>
        </div>
        <p class="note">分路异常 → IsSensorDataOk 停走保持；恢复点「全健康帧」或开持续开关。</p>
      </div>
      <div class="card">
        <h3>注入日志</h3>
        <div id="simLog" class="logbox"></div>
      </div>
     </div><!-- /sideA -->
     <div class="sideInner" id="sideB">
      <div class="card" id="loraLinkCard">
        <h3>LoRa 链路 <small>地面站仿真（/remote 定位到此）</small></h3>
        <div class="btnrow">
          <button class="b" data-link="pty">PTY 虚拟串口</button>
          <button class="b" data-link="topic">话题直发</button>
          <button class="b warn" data-link="off">关闭</button>
        </div>
        <div id="ptyBox" style="display:none; margin-top:10px;">
          <div class="pathline" id="ptyPath">—</div>
          <p class="note">真实 lora_node 从上述虚拟串口收发（完整串口协议+校验）。启动命令：<br>
            <span id="spawnCmd" style="color:#ea580c"></span></p>
          <label style="font-size:13px;cursor:pointer">
            <input type="checkbox" id="spawnChk"> 自动拉起 lora_node（子进程，随看板退出）
          </label>
        </div>
        <p class="note" id="linkNote"></p>
        <div class="linkstat">
          <span id="lstatTx" class="chip">TX 0</span>
          <span id="lstatRx" class="chip">RX 0</span>
          <span id="lstatDrop" class="chip">丢帧 0</span>
          <span id="lstatLast" class="chip">最近RX —</span>
        </div>
      </div>
      <div class="card">
        <h3>发布模式 <small>命令经当前 LoRa 链路下发</small></h3>
        <div id="autoBox">
          <div class="sw" id="autoSw"></div>
          <b style="font-size:14px" id="autoLabel">手动模式</b>
          <div id="autoText">命令按钮中<b>亮起</b>的为当前步骤可发命令，点击发布</div>
        </div>
        <div class="rbtns" id="cmdBtns"></div>
        <p class="note" id="remoteHint"></p>
        <div class="btnrow" style="margin-top:8px">
          <button class="b warn" id="bogusBtn">发送非法命令 BOGUS（测试失败回退）</button>
        </div>
      </div>
      <div class="card">
        <h3>机器人状态 <small>bt_state + STA 回传</small></h3>
        <div style="font-size:13px; margin-bottom:8px;">
          阶段 <b id="rPhase" style="color:#ea580c">—</b> ·
          任务 <b id="rMission" style="color:#16a34a">—</b> ·
          根 <b id="rRoot">—</b>
        </div>
        <table class="staTable"><thead><tr><th>时间</th><th>STA 状态</th><th>x</th><th>y</th></tr></thead>
          <tbody id="staBody"></tbody></table>
      </div>
      <div class="card">
        <h3>帧日志 <small>含校验和原文</small></h3>
        <div id="frameLog" class="logbox flog"></div>
      </div>
      <div class="card">
        <h3>放行日志</h3>
        <div id="autoLog" class="logbox"></div>
      </div>
     </div><!-- /sideB -->
    </aside>
  </div>
</div>

<div id="viewJoy">
  <div id="joyStatus">
    <span id="joyLink" class="chip"><small>/joy 状态 —</small></span>
    <span id="joySend" class="chip"><small>上报 —</small></span>
  </div>
  <div id="joyBtns">
    <button class="pad" data-btn="0">A<small>使能</small></button>
    <button class="pad" data-btn="1">B<small>复位</small></button>
    <button class="pad" data-btn="2">X<small>攀爬</small></button>
    <button class="pad" data-btn="3">Y<small>对接</small></button>
  </div>
  <div id="joyDpad">
    <button class="dp" data-dpad="up">▲<small>上</small></button>
    <button class="dp" data-dpad="left">◀<small>左</small></button>
    <button class="dp" data-dpad="right">▶<small>右</small></button>
    <button class="dp" data-dpad="down">▼<small>下</small></button>
  </div>
  <div id="joySticks">
    <div class="stickBox">
      <div class="stick" id="stickL"><div class="knob"></div></div>
      <div class="stickLabel">左摇杆 · 行走 <span id="valL">(0.00, 0.00)</span></div>
    </div>
    <div class="stickBox">
      <div class="stick" id="stickR"><div class="knob"></div></div>
      <div class="stickLabel">右摇杆 · 转向 <span id="valR">(0.00, 0.00)</span></div>
    </div>
  </div>
  <div id="joyHint">原始帧发布到 /joy（axes[8] buttons[11]：轴 0/1=左摇杆、3=右摇杆转向、
    6/7=十字键，方向与实机一致：左/前=+1 右/后=-1；键 A/B/X/Y=使能/复位/攀爬/对接）<br>
    键盘调试：WASD=左摇杆，Q/E=转向，方向键=十字键，1/2/3/4=A/B/X/Y</div>
</div>
<div id="toast"></div>
<footer>自动刷新 1s · bt_dashboard v2</footer>
<script>
"use strict";
var COLORS = { RUNNING:"#f97316", SUCCESS:"#22c55e", FAILURE:"#ef4444", INVALID:"#64748b" };
var MARKS  = { RUNNING:"●", SUCCESS:"✓", FAILURE:"✗", INVALID:"·" };
var TEMPLATES = __TEMPLATES_JSON__;
var MODE_DEFS = __MODES_JSON__;
var REMOTE_CMDS = __CMDS_JSON__;

function esc(s){ return (s==null?"":String(s)).replace(/[&<>"']/g,
  function(c){ return {"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]; }); }
function hms(t){ return t ? new Date(t*1000).toLocaleTimeString() : "--:--:--"; }
function setChip(el, label, value, color){
  el.innerHTML = "<span class='st' style='background:" + (color || COLORS[value] || "#64748b") +
    "'></span>" + label + " " + esc(value||"—");
}
var toastTimer = null;
function toast(msg, ok){
  var t = document.getElementById("toast");
  t.textContent = msg;
  t.className = "show" + (ok ? "" : " bad");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(function(){ t.className = ""; }, 2600);
}
function post(path, body){
  return fetch(path, { method: "POST", headers: { "Content-Type": "application/json" },
                       body: JSON.stringify(body) })
    .then(function(r){ return r.json(); });
}

/* ---- 页签切换（主控台=树+模式控制+模拟注入+远端链路同屏；/remote 滚动定位） ---- */
var VIEWS = ["bt","joy"];
var TAB_OF = { bt:"tabBt", joy:"tabJoy" };
var BOX_OF = { bt:"viewBt", joy:"viewJoy" };
var curView = "bt", joyActive = false;
function showView(name){
  if (VIEWS.indexOf(name) < 0) name = "bt";
  var wasJoy = joyActive;
  joyActive = (name === "joy");
  VIEWS.forEach(function(v){
    var el = document.getElementById(BOX_OF[v]);
    el.style.display = (v === name) ? (v === "joy" ? "flex" : "block") : "none";
    if (v !== "joy") el.classList.toggle("view", true);
  });
  VIEWS.forEach(function(v){
    document.getElementById(TAB_OF[v]).className = (v === name) ? "act" : "";
  });
  document.body.className = joyActive ? "joy" : "";
  curView = name;
  if (wasJoy && !joyActive) joyRelease(true);
}
VIEWS.forEach(function(v){
  document.getElementById(TAB_OF[v]).addEventListener("click", function(){ showView(v); });
});

/* ---- 行为树页：完整树渲染（实时数据优先，无数据用静态模板灰显） ---- */
var treeChoice = "主链";
document.querySelectorAll("#treeSel button").forEach(function(b){
  b.addEventListener("click", function(){
    treeChoice = b.dataset.tree;
    document.querySelectorAll("#treeSel button").forEach(function(x){
      x.className = (x === b) ? "act" : ""; });
    lastTreeSig = "";           // 强制重渲染
    pollOnce();
  });
});
function buildHierarchy(nodes){
  var root = { n: nodes[0], children: [] }, stack = [root];
  for (var i = 1; i < nodes.length; i++){
    var nd = nodes[i];
    while (stack.length > 1 && stack[stack.length-1].n.depth >= nd.depth) stack.pop();
    var item = { n: nd, children: [] };
    stack[stack.length-1].children.push(item);
    stack.push(item);
  }
  return root;
}
function markPath(item, activeName){
  var hit = (item.n.status === "RUNNING" && item.n.name === activeName);
  item.children.forEach(function(c){ if (markPath(c, activeName)) hit = true; });
  item.onpath = hit;
  return hit;
}
function nodeHtml(item, hasActive){
  var n = item.n;
  var cls = "node r-" + String(n.status||"INVALID").toLowerCase();
  if (hasActive){ cls += item.onpath ? " onpath" : " dimmed"; }
  if (n.status === "RUNNING" && n.name === window.__activeName) cls += " cur";
  var mark = item.children.length ? "<span class='branch-mark'>▾</span>" : "";
  var dot = "<span class='dot" + (n.status==="RUNNING" ? " pulse" : "") +
    "' style='background:" + (COLORS[n.status]||"#64748b") + "'></span>";
  var fb = n.feedback ? "<div class='fb'>" + esc(n.feedback) + "</div>" : "";
  return "<li><div class='" + cls + "'>" + mark + dot +
    "<span class='nm'>" + esc(n.name) + "</span>" +
    "<span class='curtag'>当前</span>" + fb + "</div>" +
    (item.children.length
      ? "<ul>" + item.children.map(function(c){ return nodeHtml(c, hasActive); }).join("") + "</ul>"
      : "") + "</li>";
}
var lastTreeSig = "", lastScrollPhase = "";
function renderTree(nodes, activeName){
  var w = document.getElementById("treeWrap");
  if (!nodes || !nodes.length){ w.innerHTML = ""; return; }
  var sig = nodes.map(function(n){ return n.name + "|" + n.status; }).join(";") + "#" + activeName;
  if (sig === lastTreeSig) return;
  lastTreeSig = sig;
  window.__activeName = activeName;
  var root = buildHierarchy(nodes);
  var hasActive = nodes.some(function(n){ return n.status === "RUNNING"; });
  if (hasActive) markPath(root, activeName);
  w.innerHTML = "<ul class='tree'>" + nodeHtml(root, hasActive) + "</ul>";
  if (activeName && activeName !== lastScrollPhase){
    lastScrollPhase = activeName;
    var cur = w.querySelector(".node.cur");
    if (cur && cur.scrollIntoView) cur.scrollIntoView({block:"center", behavior:"smooth"});
  }
}
function renderBt(d){
  var bt = d.bt || {};
  document.getElementById("waiting").style.display = bt.waiting ? "block" : "none";
  document.getElementById("staleBar").style.display = bt.stale ? "block" : "none";
  setChip(document.getElementById("chipRoot"), "树状态", bt.root_status);
  setChip(document.getElementById("chipMission"),
          "任务结果", bt.mission_status || "—",
          (bt.mission_status === "FAILED") ? "#ef4444" :
          ((bt.mission_status === "DONE") ? "#22c55e" : undefined));
  document.getElementById("chipTime").innerHTML = "<small>" + hms(bt.received) + "</small>";
  document.getElementById("phaseName").textContent =
    bt.active_phase || (bt.waiting ? "等待数据…（灰显完整树模板）" : "待命 / 无运行节点");
  document.getElementById("phaseFb").textContent = bt.active_feedback || "";
  var nodes = null;
  if (!bt.waiting && bt.nodes && bt.nodes.length){
    nodes = bt.nodes;
    document.getElementById("treeSel").style.display =
      (TEMPLATES[bt.tree_name] ? "none" : "flex");
  } else {
    var tp = TEMPLATES[treeChoice];
    nodes = tp ? tp.nodes : null;
    document.getElementById("treeSel").style.display = "flex";
  }
  document.querySelectorAll("#treeSel button").forEach(function(x){
    x.className = (x.dataset.tree === treeChoice) ? "act" : ""; });
  renderTree(nodes, bt.active_phase);
}

/* ---- 模式控制页 ---- */
(function(){
  var host = document.getElementById("modeGrid"), html = "";
  MODE_DEFS.forEach(function(m){
    html += "<button class='mode' data-mode='" + m.mode + "' title='" + esc(m.full) +
      "'>" + esc(m.label) + "<small>" + esc(m.mode) + "</small></button>";
  });
  host.innerHTML = html;
  host.addEventListener("click", function(ev){
    var b = ev.target.closest("button.mode");
    if (!b || b.disabled) return;
    post("mode", { mode: b.dataset.mode }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
  document.querySelectorAll("button[data-gripper]").forEach(function(b){
    b.addEventListener("click", function(){
      post("gripper", { action: b.dataset.gripper }).then(function(res){
        toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
      }).catch(function(){ toast("请求失败", false); });
    });
  });
  document.getElementById("simModeSw").addEventListener("click", function(){
    var ss = (window.__lastData && window.__lastData.mode &&
              window.__lastData.mode.sim_service) || {};
    post("mode_sim", { on: !ss.on }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
  document.getElementById("simModeDelayBtn").addEventListener("click", function(){
    var v = parseFloat(document.getElementById("simModeDelay").value);
    if (isNaN(v) || v < 0){ toast("延迟须为非负数值（秒）", false); return; }
    post("mode_sim", { delay: v }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
})();
function renderModes(d){
  var busy = d.mode && d.mode.busy;
  document.querySelectorAll("button.mode").forEach(function(b){
    b.disabled = !!busy; b.classList.toggle("busy", !!busy); });
  var ss = (d.mode && d.mode.sim_service) || {};
  var sw = document.getElementById("simModeSw");
  if (sw) sw.classList.toggle("on", !!ss.on);
  var delayInput = document.getElementById("simModeDelay");
  if (delayInput && document.activeElement !== delayInput && ss.delay_s != null){
    delayInput.value = ss.delay_s;
  }
  var log = (d.mode && d.mode.log) || [];
  document.getElementById("modeLog").innerHTML = log.slice(-12).map(function(e){
    return "<div><span class='t'>" + hms(e.t) + "</span>" +
      (e.ok ? "<span class='ok'>✓</span> " : "<span class='bad'>✗</span> ") +
      esc(e.msg) + "</div>";
  }).join("");
  var rc = d.remote_cmd;
  document.getElementById("remoteCmdChip").textContent = rc
    ? "最近 remote_cmd: " + rc.mode + "（" + Math.max(0, Math.round(rc.age_s)) + "s 前）" : "";
}

/* ---- 模拟注入页 ---- */
document.querySelectorAll("[data-sim-toggle]").forEach(function(sw){
  sw.addEventListener("click", function(){
    post("sim", { toggle: sw.dataset.simToggle }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
});
document.querySelectorAll("[data-sim-once]").forEach(function(b){
  b.addEventListener("click", function(){
    post("sim", { once: b.dataset.simOnce }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
});
function renderSim(d){
  var sim = d.sim || {}, tg = sim.toggles || {};
  document.querySelectorAll("[data-sim-toggle]").forEach(function(sw){
    sw.classList.toggle("on", !!tg[sw.dataset.simToggle]); });
  var log = sim.log || [];
  document.getElementById("simLog").innerHTML = log.slice(-12).map(function(e){
    return "<div><span class='t'>" + hms(e.t) + "</span>" +
      (e.ok ? "<span class='ok'>✓</span> " : "<span class='bad'>✗</span> ") +
      esc(e.msg) + "</div>";
  }).join("");
}

/* ---- 远端控制台 ---- */
var clockOffset = 0;             // server_now - Date.now()/1000
(function(){
  var host = document.getElementById("cmdBtns"), html = "";
  REMOTE_CMDS.forEach(function(c){
    html += "<button class='cmd' data-op='" + c.op + "' title='" + esc(c.desc) +
      "'>" + esc(c.label) + "<small>" + c.op + "</small>" +
      "<span class='abadge'>自动</span></button>";
  });
  host.innerHTML = html;
  host.addEventListener("click", function(ev){
    var b = ev.target.closest("button.cmd");
    if (!b || b.disabled) return;
    post("lora_tx", { op: b.dataset.op }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
  document.getElementById("bogusBtn").addEventListener("click", function(){
    post("lora_tx", { op: "BOGUS" }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
  document.querySelectorAll("button[data-link]").forEach(function(b){
    b.addEventListener("click", function(){
      post("lora_cfg", { mode: b.dataset.link }).then(function(res){
        toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
      }).catch(function(){ toast("请求失败", false); });
    });
  });
  document.getElementById("autoSw").addEventListener("click", function(){
    post("remote_auto", {}).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
  document.getElementById("spawnChk").addEventListener("change", function(){
    post("lora_cfg", { spawn: this.checked }).then(function(res){
      toast(res.ok ? "✔ " + res.msg : "✘ " + res.msg, res.ok); pollOnce();
    }).catch(function(){ toast("请求失败", false); });
  });
})();
function renderRemote(d){
  var lora = d.lora || {}, r = d.remote || {};
  clockOffset = d.server_now - Date.now()/1000;
  document.querySelectorAll("button[data-link]").forEach(function(b){
    b.classList.toggle("on", b.dataset.link === lora.mode); });
  var ptyOn = lora.mode === "pty";
  document.getElementById("ptyBox").style.display = ptyOn ? "block" : "none";
  if (ptyOn){
    document.getElementById("ptyPath").textContent =
      "虚拟串口（lora_node 端口）: " + (lora.slave_path || "—");
    document.getElementById("spawnCmd").textContent =
      "rosrun lora lora_node.py _port:=" + (lora.slave_path || "/dev/pts/N");
  }
  var chk = document.getElementById("spawnChk");
  chk.disabled = !ptyOn;
  chk.checked = !!lora.lora_node_on;
  document.getElementById("linkNote").textContent =
    lora.mode === "topic" ? "话题直发：直接发布 /lora/command（勿与真实 lora_node 同时启用）" :
    lora.mode === "off" ? "链路已关闭：命令按钮不可用" : "";
  document.getElementById("lstatTx").textContent = "TX " + (lora.tx || 0);
  document.getElementById("lstatRx").textContent = "RX " + (lora.rx || 0);
  document.getElementById("lstatDrop").textContent = "丢帧 " + (lora.dropped || 0);
  document.getElementById("lstatLast").textContent = lora.last_rx_age_s != null
    ? "最近RX " + lora.last_rx_age_s.toFixed(1) + "s 前" : "最近RX —";
  var lc = { off:"#64748b", pty:"#22c55e", topic:"#f59e0b" }[lora.mode] || "#64748b";
  setChip(document.getElementById("chipLink"), "LoRa",
          lora.mode === "pty" ? "PTY" : (lora.mode === "topic" ? "直发" : "关"), lc);
  // 自动/手动
  var sw = document.getElementById("autoSw");
  sw.classList.toggle("on", !!r.auto_on);
  document.getElementById("autoLabel").textContent = r.auto_on ? "自动模式" : "手动模式";
  // 命令按钮：链路开启才可用；手动模式建议亮起；自动模式待发标记
  var suggest = (!r.auto_on && r.suggest) ? r.suggest.ops : [];
  var armedOp = (r.auto_on && r.pending && r.pending.channel === "lora")
    ? r.pending.value : null;
  document.querySelectorAll("button.cmd").forEach(function(b){
    b.disabled = lora.mode === "off";
    b.classList.toggle("ready", suggest.indexOf(b.dataset.op) >= 0);
    b.classList.toggle("armed", b.dataset.op === armedOp);
  });
  document.getElementById("remoteHint").textContent =
    (!r.auto_on && r.suggest) ? r.suggest.hint : (r.auto_on
      ? "自动放行：收到状态反馈后 " + AUTO_DELAY_STR + "s 自动发送当前步骤的通过命令（LoRa 步骤走串口，传感器步骤走模拟话题）"
      : "");
  // 机器人状态 + STA 历史
  document.getElementById("rPhase").textContent = r.phase || "—";
  document.getElementById("rMission").textContent = r.mission || "—";
  document.getElementById("rRoot").textContent = r.root || "—";
  var sta = lora.sta || [];
  document.getElementById("staBody").innerHTML = sta.slice(-8).reverse().map(function(s){
    return "<tr><td>" + hms(s.t) + "</td><td><b>" + esc(s.status) + "</b></td><td>" +
      esc(s.x) + "</td><td>" + esc(s.y) + "</td></tr>";
  }).join("");
  // 帧日志
  var frames = lora.frames || [];
  document.getElementById("frameLog").innerHTML = frames.slice(-12).reverse().map(function(f){
    return "<div class='" + f.dir + "'><span class='t'>" + hms(f.t) + "</span>" +
      (f.dir === "tx" ? "↑TX " : "↓RX ") + esc(f.body) + " <b>*" + esc(f.ck) + "</b>（" +
      esc(f.via) + "）</div>";
  }).join("");
  // 放行日志
  var alog = r.log || [];
  document.getElementById("autoLog").innerHTML = alog.slice(-12).map(function(e){
    return "<div><span class='t'>" + hms(e.t) + "</span>" +
      (e.ok ? "<span class='ok'>✓</span> " : "<span class='bad'>✗</span> ") +
      esc(e.msg) + "</div>";
  }).join("");
  renderAutoText(d);
}
var AUTO_DELAY_STR = "__AUTO_DELAY__";
function renderAutoText(d){
  var r = d.remote || {}, box = document.getElementById("autoText");
  if (!r.auto_on){
    box.innerHTML = "命令按钮中<b>亮起</b>的为当前步骤可发命令，点击发布";
    return;
  }
  if (!r.pending){
    box.innerHTML = "自动放行已开启：<b>等待状态反馈</b>…";
    return;
  }
  var p = r.pending;
  var remain = p.fire_at - (Date.now()/1000 + clockOffset);
  var via = p.channel === "lora" ? "LoRa 串口" : "模拟话题";
  box.innerHTML = "反馈已到（" + esc(p.phase) + "）：<b>" +
    (remain > 0 ? remain.toFixed(1) + "s" : "正在") + "</b>后自动发送 <b>" +
    esc(p.value) + "</b>（" + via + "）";
}
/* ---- 模拟模式服务：执行中剩余时间（deadline 为服务端绝对时刻） ---- */
function renderSimModeText(d){
  var ss = (d.mode && d.mode.sim_service) || {};
  var el = document.getElementById("simModeNote");
  if (!el) return;
  if (ss.on && ss.current){
    var remain = ss.current.deadline - (Date.now()/1000 + clockOffset);
    el.innerHTML = "模拟执行中：<b>" + esc(ss.current.mode) + "</b> · 剩余 " +
      (remain > 0 ? remain.toFixed(0) + "s" : "即将完成");
  } else {
    el.textContent = "";
  }
}
setInterval(function(){                 // 倒计时平滑刷新（不重新拉数据）
  if (curView === "bt" && window.__lastData){
    renderAutoText(window.__lastData);
    renderSimModeText(window.__lastData);
  }
}, 200);

/* ---- 主渲染 + 1s 轮询（载荷未变跳过） ---- */
function render(d){
  window.__lastData = d;
  renderBt(d);
  renderModes(d);
  renderSim(d);
  renderRemote(d);
  renderSimModeText(d);
}
var inflight = false, lastPayload = "";
function pollOnce(){
  if (inflight) return;
  inflight = true;
  fetch("state.json", { cache: "no-store" })
    .then(function(r){ return r.text(); })
    .then(function(txt){
      inflight = false;
      if (txt === lastPayload) return;
      lastPayload = txt;
      render(JSON.parse(txt));
    })
    .catch(function(){ inflight = false; });
}
setInterval(pollOnce, 1000);

/* ---- 虚拟手柄（/joy 原始帧，与实体手柄同话题同格式） ---- */
var joyState = { axes:[0,0,0,0,0,0,0,0], buttons:[0,0,0,0,0,0,0,0,0,0,0] };
function padFrame(){ return { axes: joyState.axes.slice(), buttons: joyState.buttons.slice() }; }
function makeStick(el, axH, axV, valEl){
  var knob = el.querySelector(".knob");
  function apply(dx, dy){
    dx = Math.max(-1, Math.min(1, dx));
    dy = Math.max(-1, Math.min(1, dy));
    var R = el.getBoundingClientRect().width * 0.28;
    knob.style.transform = "translate(" + (dx*R).toFixed(1) + "px," + (dy*R).toFixed(1) + "px)";
    joyState.axes[axH] = -dx;      // 与实机一致：左推=+1、右推=-1
    joyState.axes[axV] = -dy;      // 屏幕向上推 = +1（前推为正）
    if (valEl) valEl.textContent = "(" + (-dx).toFixed(2) + ", " + (-dy).toFixed(2) + ")";
  }
  function fromEvent(e){
    var r = el.getBoundingClientRect(), R = r.width / 2;
    apply((e.clientX - r.left - R) / R, (e.clientY - r.top - R) / R);
  }
  el.addEventListener("pointerdown", function(e){
    try { el.setPointerCapture(e.pointerId); } catch (err) {}
    el.classList.add("live");
    fromEvent(e);
    e.preventDefault();
  });
  el.addEventListener("pointermove", function(e){
    if (!el.classList.contains("live")) return;
    fromEvent(e);
    e.preventDefault();
  });
  function end(){ el.classList.remove("live"); apply(0, 0); }
  el.addEventListener("pointerup", end);
  el.addEventListener("pointercancel", end);
  return { release: function(){
    el.classList.remove("live");
    knob.style.transform = "translate(0px, 0px)";
    if (valEl) valEl.textContent = "(0.00, 0.00)";
  } };
}
var stickL = makeStick(document.getElementById("stickL"), 0, 1, document.getElementById("valL"));
var stickR = makeStick(document.getElementById("stickR"), 3, 4, document.getElementById("valR"));
(function(){
  document.querySelectorAll("button.pad").forEach(function(b){
    var idx = parseInt(b.dataset.btn, 10);
    function press(on){
      joyState.buttons[idx] = on ? 1 : 0;
      b.classList.toggle("on", on);
    }
    b.addEventListener("pointerdown", function(e){
      try { b.setPointerCapture(e.pointerId); } catch (err) {}
      press(true); e.preventDefault();
    });
    b.addEventListener("pointerup", function(){ press(false); });
    b.addEventListener("pointercancel", function(){ press(false); });
    b.addEventListener("contextmenu", function(e){ e.preventDefault(); });
  });
})();
var dpadState = { up:false, down:false, left:false, right:false };
function dpadApply(){
  joyState.axes[6] = dpadState.left ? 1 : (dpadState.right ? -1 : 0);
  joyState.axes[7] = dpadState.up ? 1 : (dpadState.down ? -1 : 0);
}
(function(){
  document.querySelectorAll("button.dp").forEach(function(b){
    var key = b.dataset.dpad;
    function press(on){
      dpadState[key] = on;
      b.classList.toggle("on", on);
      dpadApply();
    }
    b.addEventListener("pointerdown", function(e){
      try { b.setPointerCapture(e.pointerId); } catch (err) {}
      press(true); e.preventDefault();
    });
    b.addEventListener("pointerup", function(){ press(false); });
    b.addEventListener("pointercancel", function(){ press(false); });
    b.addEventListener("contextmenu", function(e){ e.preventDefault(); });
  });
})();
var KEYMAP = { w:"ax1+", s:"ax1-", a:"ax0-", d:"ax0+",
               arrowup:"dpup", arrowdown:"dpdown",
               arrowleft:"dpleft", arrowright:"dpright",
               q:"ax3-", e:"ax3+", "1":"b0", "2":"b1", "3":"b2", "4":"b3" };
var keyDown = {};
function keyRebuild(){
  function axis(pos, neg){ return (keyDown[pos] ? 0.9 : 0) - (keyDown[neg] ? 0.9 : 0); }
  joyState.axes[0] = axis("ax0-", "ax0+");
  joyState.axes[1] = axis("ax1+", "ax1-");
  joyState.axes[3] = axis("ax3-", "ax3+");
  for (var i = 0; i < 4; i++) joyState.buttons[i] = keyDown["b" + i] ? 1 : 0;
  ["up","down","left","right"].forEach(function(dd){
    if (("dp" + dd) in keyDown) dpadState[dd] = !!keyDown["dp" + dd];
  });
  dpadApply();
}
document.addEventListener("keydown", function(e){
  var k = KEYMAP[e.key.toLowerCase()];
  if (!k || keyDown[k]) return;
  e.preventDefault();
  keyDown[k] = true;
  keyRebuild();
});
document.addEventListener("keyup", function(e){
  var k = KEYMAP[e.key.toLowerCase()];
  if (!k) return;
  keyDown[k] = false;
  keyRebuild();
});
var joySendOk = null;
function postJoy(body, opts){
  var init = { method: "POST", headers: { "Content-Type": "application/json" },
               body: JSON.stringify(body) };
  if (opts) Object.keys(opts).forEach(function(k){ init[k] = opts[k]; });
  return fetch("joy", init)
    .then(function(r){ return r.json(); })
    .then(function(res){ joySendOk = true; return res; })
    .catch(function(){ joySendOk = false; return null; });
}
function joyRelease(send){
  joyState.axes = [0,0,0,0,0,0,0,0];
  joyState.buttons = [0,0,0,0,0,0,0,0,0,0,0];
  stickL.release();
  stickR.release();
  document.querySelectorAll("button.pad.on, button.dp.on").forEach(function(b){
    b.classList.remove("on");
  });
  dpadState = { up:false, down:false, left:false, right:false };
  keyDown = {};
  if (send) postJoy(padFrame(), { keepalive: true });
}
setInterval(function(){
  if (joyActive) postJoy(padFrame());
}, 50);
window.addEventListener("pagehide", function(){ if (joyActive) joyRelease(true); });
document.addEventListener("visibilitychange", function(){
  if (document.hidden && joyActive) joyRelease(true);
});
setInterval(function(){
  if (!joyActive) return;
  fetch("joy.json", { cache: "no-store" })
    .then(function(r){ return r.json(); })
    .then(function(st){
      var subs = st.subscribers || 0;
      document.getElementById("joyLink").innerHTML =
        "<span class='st' style='background:" + (subs > 0 ? "#22c55e" : "#f59e0b") +
        "'></span>/joy 订阅 " + subs + (subs > 0 ? "（链路通）" : "（未启动遥控链）");
      document.getElementById("joySend").innerHTML =
        "<span class='st' style='background:" + (joySendOk ? "#f97316" : "#ef4444") +
        "'></span>上报 " + (joySendOk ? "正常 ~20Hz" : "失败");
    })
    .catch(function(){});
}, 1000);

/* ---- 启动：/remote 滚动定位到远端链路面板 ---- */
showView("bt");
if (location.pathname.indexOf("remote") >= 0){
  setTimeout(function(){
    var lc = document.getElementById("loraLinkCard");
    if (lc && lc.scrollIntoView) lc.scrollIntoView({ block: "start", behavior: "smooth" });
  }, 600);
}
pollOnce();
</script>
</body>
</html>
"""


def build_page():
    templates = load_tree_templates()
    return (_PAGE_TEMPLATE
            .replace("__TEMPLATES_JSON__", json.dumps(templates, ensure_ascii=False))
            .replace("__MODES_JSON__", json.dumps(MODE_DEFS, ensure_ascii=False))
            .replace("__CMDS_JSON__", json.dumps(REMOTE_CMDS, ensure_ascii=False))
            .replace("__AUTO_DELAY__", "{:.0f}".format(AUTO_DELAY_S)))


_PAGE_CACHE = None


def get_page():
    global _PAGE_CACHE
    if _PAGE_CACHE is None:
        _PAGE_CACHE = build_page()
    return _PAGE_CACHE


# ---------------------------------------------------------------------------
# HTTP 服务
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    core = None          # DashboardCore（run/selftest 注入）
    joy = None           # JoyLink
    protocol_version = "HTTP/1.1"

    def _respond(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):  # noqa: BLE001
            pass

    def _json(self, code, obj):
        self._respond(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                      "application/json; charset=utf-8")

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html", "/remote"):
            self._respond(200, get_page().encode("utf-8"),
                          "text/html; charset=utf-8")
        elif path == "/state.json":
            if self.core is None:
                self._json(503, {"ok": False, "msg": "core unavailable"})
                return
            self._respond(200, self.core.payload().encode("utf-8"),
                          "application/json; charset=utf-8")
        elif path == "/joy.json":
            if self.joy is None:
                self._json(503, {"ok": False, "msg": "joy unavailable"})
                return
            self._json(200, self.joy.view())
        else:
            self.send_error(404, "not found")

    def _body(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(length).decode("utf-8") or "{}"), None
        except Exception:  # noqa: BLE001
            return None, "请求体不是合法 JSON"

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path not in ("/joy", "/mode", "/gripper", "/sim", "/lora_tx",
                        "/lora_cfg", "/remote_auto", "/mode_sim"):
            self.send_error(404, "not found")
            return
        body, err = self._body()
        if err:
            self._json(400, {"ok": False, "msg": err})
            return
        if self.core is None and path != "/joy":
            self._json(503, {"ok": False, "msg": "core unavailable"})
            return
        if path == "/joy":
            axes, buttons, jerr = _parse_joy_body(body)
            if jerr:
                self._json(400, {"ok": False, "msg": jerr})
                return
            if self.joy is None:
                self._json(503, {"ok": False, "msg": "joy unavailable"})
                return
            ok, msg = self.joy.update(axes, buttons)
            self._json(200 if ok else 503, {"ok": ok, "msg": msg})
            return
        if path == "/mode":
            mode = str(body.get("mode") or "")
            if not any(m["mode"] == mode for m in MODE_DEFS):
                self._json(400, {"ok": False, "msg": "未知模式: {}".format(mode)})
                return
            ok, msg = self.core.modes.request("mode", mode)
            self._json(200 if ok else 409, {"ok": ok, "msg": msg})
            return
        if path == "/mode_sim":
            if "delay" in body:
                ok, msg = self.core.sim_modes.set_delay(body.get("delay"))
                code = 200 if ok else 400
            else:
                ok, msg = self.core.sim_modes.set_enabled(bool(body.get("on")))
                code = 200 if ok else 409
            self._json(code, {"ok": ok, "msg": msg})
            return
        if path == "/gripper":
            action = str(body.get("action") or "")
            if action not in ("open", "clamp"):
                self._json(400, {"ok": False, "msg": "action 须为 open/clamp"})
                return
            ok, msg = self.core.modes.request("gripper", action)
            self._json(200 if ok else 409, {"ok": ok, "msg": msg})
            return
        if path == "/sim":
            toggle = body.get("toggle")
            once = body.get("once")
            if toggle:
                ok, msg = self.core.sim.toggle(str(toggle))
            elif once:
                ok, msg = self.core.sim.once(str(once))
            else:
                ok, msg = False, "需要 toggle 或 once 字段"
            self._json(200 if ok else 400, {"ok": ok, "msg": msg})
            return
        if path == "/lora_tx":
            ok, msg = self.core.lora.send_cmd(str(body.get("op") or ""))
            self._json(200 if ok else 400, {"ok": ok, "msg": msg})
            return
        if path == "/lora_cfg":
            if "mode" in body:
                ok, msg = self.core.lora.set_mode(str(body["mode"]))
            elif "spawn" in body:
                ok, msg = self.core.lora.spawn_lora_node(bool(body["spawn"]))
            else:
                ok, msg = False, "需要 mode 或 spawn 字段"
            self._json(200 if ok else 400, {"ok": ok, "msg": msg})
            return
        if path == "/remote_auto":
            ok, msg = self.core.auto.set_auto(not self.core.auto.auto_on)
            self._json(200, {"ok": ok, "msg": msg})
            return

    def log_message(self, *args):   # 静默访问日志
        pass


# ---------------------------------------------------------------------------
# ROS 接线
# ---------------------------------------------------------------------------
def run():
    import rospy
    from std_msgs.msg import String
    from grasp_hexapod_msgs.msg import BtStateArray, RemoteCmd
    from bt_monitor import snapshot_from_msg

    rospy.init_node("bt_dashboard", anonymous=True)
    port = int(rospy.get_param("~port", 8080))
    host = rospy.get_param("~host", "0.0.0.0")
    lora_mode = rospy.get_param("~lora_mode", "pty")

    core = DashboardCore()

    def _sim_mode_factory(handler):
        from grasp_hexapod_msgs.srv import SwitchMode, SwitchModeResponse

        def _handler(req):
            ok, msg = handler(req)
            return SwitchModeResponse(success=ok, message=msg)

        return rospy.Service(SWITCH_MODE_SERVICE, SwitchMode, _handler)

    core.sim_modes._factory = _sim_mode_factory
    core.sim_modes.delay_s = max(
        0.0, float(rospy.get_param("~sim_mode_delay", 5.0)))

    # LoRa 话题直发发布器（懒建）
    def _lora_topic_publish(body):
        pub = getattr(core, "_lora_cmd_pub", None)
        if pub is None:
            pub = core._lora_cmd_pub = rospy.Publisher(
                LORA_COMMAND_TOPIC, String, queue_size=5)
        deadline = time.time() + 0.6
        while pub.get_num_connections() == 0 and time.time() < deadline:
            time.sleep(0.05)
        pub.publish(String(data=body))
        return pub.get_num_connections() > 0

    core.lora._topic_publish = _lora_topic_publish

    from sensor_msgs.msg import Joy as JoyMsg
    joy_pub = rospy.Publisher(JOY_TOPIC, JoyMsg, queue_size=10)
    Handler.core = core
    Handler.joy = JoyLink(joy_pub, log=rospy.loginfo)

    def on_bt_state(msg):
        core.bt.update(snapshot_from_msg(msg))
        core.auto.on_phase(msg.active_phase)

    def on_lora_status(msg):       # STA 上行：话题直发模式经此回传；
        if core.lora.mode == "topic":   # PTY 模式由串口侧解码，勿重复计
            core.lora.handle_body(msg.data, via="话题")
            core.auto.on_sta()

    def on_remote_cmd(msg):
        core.on_remote_cmd(msg.mode)

    rospy.Subscriber(BT_STATE_TOPIC, BtStateArray, on_bt_state, queue_size=10)
    rospy.Subscriber(LORA_STATUS_TOPIC, String, on_lora_status, queue_size=5)
    rospy.Subscriber(REMOTE_CMD_TOPIC, RemoteCmd, on_remote_cmd, queue_size=5)
    rospy.Timer(rospy.Duration(0.5), lambda _e: Handler.joy.watchdog())

    if lora_mode in ("pty", "topic"):
        core.lora.set_mode(lora_mode)

    # 后台循环：模拟注入周期发布（2Hz）+ 自动放行 tick（10Hz）
    stop = threading.Event()

    def _loop_sim():
        while not stop.wait(0.5):
            try:
                core.sim.tick()
            except Exception:  # noqa: BLE001
                pass

    def _loop_auto():
        while not stop.wait(0.1):
            try:
                core.auto.tick()
            except Exception:  # noqa: BLE001
                pass

    threading.Thread(target=_loop_sim, daemon=True, name="sim-tick").start()
    threading.Thread(target=_loop_auto, daemon=True, name="auto-tick").start()

    try:
        httpd = ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        rospy.logfatal("无法监听 %s:%d（%s）。多为残留旧看板进程占用："
                       "ps aux | grep bt_dashboard 找到后 kill，或 "
                       "rosrun grasp_hexapod_bt bt_dashboard.py _port:=9000 换端口",
                       host, port, exc)
        sys.exit(1)
    pty_path = core.lora.transport.path if core.lora.transport else "未开启"
    rospy.loginfo("bt_dashboard v2 就绪：http://%s:%d （/remote 直达远端控制台）"
                  "；LoRa 链路=%s（虚拟串口 %s，rosrun lora lora_node.py "
                  "_port:=%s）；模拟注入/模式控制自持，无 sim_manual 依赖",
                  _lan_ip(), port, core.lora.mode, pty_path, pty_path)
    import signal
    rospy.on_shutdown(lambda: (stop.set(), core.lora.shutdown(), threading.Thread(
        target=httpd.shutdown, daemon=True).start()))
    signal.signal(signal.SIGTERM,
                  lambda *_a: rospy.signal_shutdown("sigterm"))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        core.lora.shutdown()
        httpd.server_close()
        rospy.signal_shutdown("bt_dashboard exit")


def _lan_ip():
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except Exception:  # noqa: BLE001
        return "0.0.0.0"


# ---------------------------------------------------------------------------
# 离线自检
# ---------------------------------------------------------------------------
def _wait_until(cond, timeout=3.0, step=0.05):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return True
        time.sleep(step)
    return cond()


def selftest():
    """离线：codec/模板/映射/状态机/PTY 回环/HTTP 往返（不依赖 ROS master）。"""
    import urllib.request

    # 1. LoRaCodec：与 lora_node 同款协议（"ABC"→C6），组帧/解帧/坏帧丢弃
    assert LoRaCodec.checksum(b"ABC") == "C6"
    codec = LoRaCodec()
    frame = LoRaCodec.build_frame("CMD,HEX,DEPLOY,NOW")
    body = "CMD,HEX,DEPLOY,NOW"
    assert frame == ("{}*{}\r\n".format(body, LoRaCodec.checksum(
        body.encode())).encode())
    # 分片喂入仍可解出
    assert codec.feed(frame[:5]) == []
    assert codec.feed(frame[5:]) == [body]
    bad = LoRaCodec()
    bad.feed(b"HELLO*FF\r\n")            # 校验错 → 丢弃
    assert bad.dropped == 1 and bad.lines == 1
    print("[OK] LoRaCodec 组帧/分片解帧/坏帧丢弃")

    # 2. 静态完整树模板：两棵树、深度单调、主链节点数与兜底副本一致
    templates = load_tree_templates()
    assert set(templates) == {"主链", "遥控测试链"}
    for name, tp in templates.items():
        nodes = tp["nodes"]
        assert nodes[0]["depth"] == 0
        for prev, cur in zip(nodes, nodes[1:]):
            assert cur["depth"] <= prev["depth"] + 1, (name, cur["name"])
    fb = json.loads(FALLBACK_TEMPLATES_JSON)
    assert len(templates["主链"]["nodes"]) == len(fb["主链"]["nodes"])
    assert len(templates["遥控测试链"]["nodes"]) == len(fb["遥控测试链"]["nodes"])
    assert [n["name"] for n in templates["主链"]["nodes"]] == \
           [n["name"] for n in fb["主链"]["nodes"]]
    print("[OK] 树模板（主链 %d 节点 / 遥控测试链 %d 节点，兜底副本一致）" % (
        len(templates["主链"]["nodes"]), len(templates["遥控测试链"]["nodes"])))

    # 3. 放行/建议映射
    ch, val, _ = resolve_pass("WaitDeployment")
    assert (ch, val) == ("lora", "DEPLOY")
    assert resolve_pass("IsLandingConfirmed")[:2] == ("sim", "landed")
    assert resolve_pass("WaitRtkPrecise")[:2] == ("sim", "rtk_good")
    assert resolve_pass("WaitWinchHoisted")[:2] == ("lora", "HOIST_DONE")
    assert resolve_pass("WaitHomeCmd")[:2] == ("lora", "HOME")
    assert resolve_pass("WaitTaskCommand")[0] is None       # 任务由用户起
    assert resolve_pass("执行 home")[0] is None             # 模式执行等待
    assert resolve_pass("")[0] is None
    assert suggest_for_phase("WaitDeployment")["ops"] == ["DEPLOY"]
    assert suggest_for_phase("WaitTaskCommand")["ops"] == ["RECOVER", "RELEASE"]
    assert suggest_for_phase("IsLandingConfirmed")["ops"] == []
    assert suggest_for_phase("执行 xxx") is not None
    assert suggest_for_phase("") is None
    print("[OK] 放行/建议映射")

    # 3c. SimModeService：固定延迟完成（不立刻成功）/ 抢占 / 运行中改延迟 / 关闭
    class _FakeSvcHandle:
        def shutdown(self, reason=""):
            pass

    class _Req:
        def __init__(self, m):
            self.target_mode = m

    clock4 = [1000.0]
    sms = SimModeService(log=LogBuf(), now=lambda: clock4[0], delay_s=5.0,
                         service_factory=lambda h: _FakeSvcHandle())
    ok, _ = sms.set_enabled(True)
    assert ok and sms.view()["on"] is True
    ok, msg = sms.set_enabled(True)
    assert ok and "已是" in msg                   # 重复开启幂等
    res = {}

    def _call(mode, store):
        store[mode] = sms._on_switch(_Req(mode))

    # 1) 固定延迟：home 请求后 5s 才返回（绝不立刻完成）
    th1 = threading.Thread(target=_call, args=("home", res))
    th1.start()
    time.sleep(0.15)
    assert sms.view()["current"]["mode"] == "home"   # 执行中
    clock4[0] = 1003.0
    time.sleep(0.1)
    assert res.get("home") is None                   # 5s 未到，仍阻塞
    clock4[0] = 1005.5
    assert _wait_until(lambda: res.get("home") is not None, timeout=2.0)
    assert res["home"][0] is True and "home" in res["home"][1]

    # 2) 抢占：climb 执行中被 home 抢占，按失败返回
    res2 = {}
    th2 = threading.Thread(target=_call, args=("climb", res2))
    th2.start()
    time.sleep(0.15)
    assert sms.view()["current"]["mode"] == "climb"
    res3 = {}
    th3 = threading.Thread(target=_call, args=("home", res3))
    th3.start()
    time.sleep(0.15)
    th2.join(2.0)
    assert res2["climb"][0] is False and "抢占" in res2["climb"][1]
    clock4[0] = 1011.0                               # home(1005.5+5) 到点
    assert _wait_until(lambda: res3.get("home") is not None and
                       res3["home"][0] is True, timeout=2.0)

    # 3) 运行中改延迟：新请求按新延迟完成
    ok, _ = sms.set_delay(1.0)
    assert ok and sms.view()["delay_s"] == 1.0
    res4 = {}
    th4 = threading.Thread(target=_call, args=("dock", res4))
    th4.start()
    time.sleep(0.15)
    clock4[0] += 0.5
    time.sleep(0.1)
    assert res4.get("dock") is None                  # 1s 未到
    clock4[0] += 1.0
    assert _wait_until(lambda: res4.get("dock") is not None, timeout=2.0)
    assert res4["dock"][0] is True
    ok, _ = sms.set_enabled(False)
    assert ok and sms.view()["on"] is False
    assert sms.view()["current"] is None
    print("[OK] SimModeService 固定延迟完成/抢占/运行中改延迟/关闭")

    # 4. AutoPilot：反馈->2s 倒计时->执行；每回合一次；阶段变化取消/重臂
    clock = [0.0]
    executed = []

    def fake_exec(channel, value):
        executed.append((channel, value))
        return True, "已发送 {}".format(value)

    ap = AutoPilot(executor=fake_exec, log=LogBuf(),
                   now=lambda: clock[0], delay_s=2.0)
    ap.set_auto(True)
    assert ap.pending is None or ap.pending["phase"] == ""   # 无数据不臂
    ap.pending = None
    ap.on_phase("WaitDeployment")          # 反馈 -> 2s 后发 DEPLOY
    assert ap.pending is not None and ap.pending["fire_at"] == 2.0
    clock[0] = 1.0
    ap.tick()
    assert executed == []                  # 未到点
    clock[0] = 2.0
    ap.tick()
    assert executed == [("lora", "DEPLOY")]
    ap.on_sta()                            # 已发过（本回合）-> 不重臂
    assert ap.pending is None
    ap.on_phase("IsLandingConfirmed")      # 传感器步骤 -> sim 通道
    clock[0] = 4.0
    ap.tick()
    assert ("sim", "landed") in executed
    ap.on_phase("执行 回到初始姿态")       # 模式执行 -> 无 pending
    assert ap.pending is None
    ap.on_phase("WaitHomeCmd")             # clock=4.0 -> fire_at=6.0
    clock[0] = 5.9
    ap.tick()
    assert ("lora", "HOME") not in executed   # 未到点（差 0.1s）
    clock[0] = 8.0
    ap.tick()
    assert ("lora", "HOME") in executed
    ap.set_auto(False)
    ap.on_phase("WaitDeployment")
    assert ap.pending is None              # 关自动不臂
    print("[OK] AutoPilot 反馈->2s->自动放行（每回合一次/阶段切换/开关）")

    # 5. LoraLink + 假串口：TX 组帧写入、RX 解码、STA 历史
    fake = FakeTransport()
    link = LoraLink(log=LogBuf(), transport_factory=lambda: fake)
    ok, msg = link.set_mode("pty")
    assert ok and "PTY" in msg
    ok, _ = link.send_cmd("DEPLOY")
    assert ok and fake.written and fake.written[-1] == LoRaCodec.build_frame(
        "CMD,HEX,DEPLOY,NOW")
    fake.to_read.extend(LoRaCodec.build_frame("STA,HEX,LANDED,1.00,2.00"))
    assert _wait_until(lambda: link.view()["rx"] == 1)
    view = link.view()
    assert view["sta"] and view["sta"][-1]["status"] == "LANDED"
    assert view["tx"] == 1 and view["mode"] == "pty"
    ok, _ = link.send_cmd("HOME")
    assert ok and view["tx"] + 1 == link.view()["tx"]
    link.set_mode("off")
    ok, msg = link.send_cmd("HOME")
    assert not ok and "未开启" in msg
    print("[OK] LoraLink 假串口 TX/RX/STA/模式切换")

    # 5b. 真实 PTY 回环：模拟 lora_node 打开 slave 收发（含 raw 无回显）
    real = LoraLink(log=LogBuf())
    ok, _ = real.set_mode("pty")
    assert ok
    path = real.transport.path
    peer = os.open(path, os.O_RDWR | os.O_NOCTTY)     # 扮演 lora_node
    try:
        ok, _ = real.send_cmd("RECOVER")
        assert ok
        r, _, _ = select.select([peer], [], [], 2.0)
        assert r
        got = os.read(peer, 256)
        assert got == LoRaCodec.build_frame("CMD,HEX,RECOVER,NOW")
        os.write(peer, LoRaCodec.build_frame("STA,HEX,DONE,0.00,0.00"))
        assert _wait_until(lambda: real.view()["rx"] == 1)
        assert real.view()["sta"][-1]["status"] == "DONE"
        assert real.view()["rx"] == 1        # 无回显（否则会把 TX 帧当 RX）
    finally:
        os.close(peer)
        real.shutdown()
    print("[OK] 真实 PTY 虚拟串口回环（帧透传/raw 无回显/STA 解码）")

    # 6. SimInjector：开关状态流转 + 一次性动作（消息构造需工作空间消息包）
    published = []
    sim = SimInjector(publish=lambda topic, msg: published.append((topic, msg)) or True,
                      log=LogBuf())
    ok, _ = sim.toggle("sensors")
    assert ok and sim.view()["toggles"]["sensors"] is True
    sim.toggle("sensors")
    assert sim.view()["toggles"]["sensors"] is False
    try:
        from grasp_hexapod_msgs.msg import EncoderState, SensorHealthArray
        from sensor_msgs.msg import NavSatFix
        ok, _ = sim.once("landed")
        assert ok and published[-1][0] == ENCODER_TOPIC
        assert published[-1][1].landed is True and published[-1][1].angle == 135.0
        ok, _ = sim.once("rtk_good")
        assert published[-1][0] == FIX_TOPIC
        assert published[-1][1].position_covariance[0] == 0.01
        ok, _ = sim.once("sensor_bad_imu")
        assert published[-1][0] == SENSOR_HEALTH_TOPIC
        bad = [h for h in published[-1][1].sensors if h.name == "imu"]
        assert bad and bad[0].fresh is False
        ok, _ = sim.once("zzz")
        assert not ok
        print("[OK] SimInjector 开关/一次性（消息口径与 sim_manual 一致）")
    except ImportError:
        print("[SKIP] SimInjector 消息构造（无 ROS 环境消息包）")

    # 7. ModeControl：后台线程调用 + 日志
    calls = []

    def fake_caller(kind, value):
        calls.append((kind, value))
        time.sleep(0.05)
        return True, "fake ok"

    mc = ModeControl(caller=fake_caller, log=LogBuf())
    ok, _ = mc.request("mode", "climb")
    assert ok and mc.view()["busy"] is True
    ok, _ = mc.request("mode", "dock")
    assert not ok                       # 忙时拒绝
    assert _wait_until(lambda: not mc.view()["busy"])
    assert calls == [("mode", "climb")]
    assert any("climb" in e["msg"] for e in mc.view()["log"])
    print("[OK] ModeControl 异步调用/忙拒绝/日志")

    # 8. DashboardCore 载荷：结构齐全 + 版本缓存
    core = DashboardCore()
    core.lora.shutdown()
    p1 = core.payload()
    assert core.payload() is p1          # 无变化零序列化
    core.bt.update({"tree_name": "主链", "root_status": "RUNNING",
                    "mission_status": "", "active_phase": "WaitDeployment",
                    "active_feedback": "", "nodes": []})
    p2 = core.payload()
    assert p2 is not p1
    data = json.loads(p2)
    for key in ("server_now", "bt", "sim", "mode", "lora", "remote"):
        assert key in data, key
    assert data["bt"]["active_phase"] == "WaitDeployment"
    assert data["remote"]["suggest"]["ops"] == ["DEPLOY"]
    assert data["mode"]["sim_service"]["on"] is False
    assert data["mode"]["sim_service"]["delay_s"] == 5.0
    fake_tx = FakeTransport()          # 链路可发（无任务计时钩子，仅验证收发）
    core.lora._tf = lambda: fake_tx
    core.lora.set_mode("pty")
    ok, _ = core.lora.send_cmd("RECOVER")
    assert ok and fake_tx.written
    core.lora.shutdown()
    print("[OK] DashboardCore 载荷结构/版本缓存/链路发送")

    # 9. HTTP 往返：页面标记 + state.json + POST 各端点 + 坏请求
    try:
        from grasp_hexapod_msgs.msg import EncoderState  # noqa: F401
        have_msgs = True
    except ImportError:
        have_msgs = False
    core2 = DashboardCore()
    fake2 = FakeTransport()
    core2.lora._tf = lambda: fake2
    core2.lora.set_mode("pty")
    core2.sim._publish = lambda topic, msg: True     # 免 ROS 发布器
    core2.modes._caller = fake_caller
    Handler.core = core2
    Handler.joy = JoyLink(_FakeJoyPub(), make_msg=lambda a, b: (list(a), list(b)))
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        base = "http://127.0.0.1:{}".format(port)
        for path, marks in (
                ("/", ["viewBt", "sideCol", "sideA", "sideB", "mainLayout",
                       "modeGrid", "data-gripper", "viewJoy",
                       "makeStick", "data-sim-toggle", "data-sim-once",
                       "data-op", "autoSw", "loraLinkCard",
                       "spawnChk", "treeWrap",
                       "fetch(\"state.json\""]),
                ("/remote", ["loraLinkCard", "makeStick"])):
            page = urllib.request.urlopen(base + path, timeout=5).read().decode("utf-8")
            for mark in marks:
                assert mark in page, (path, mark)
        assert "__TEMPLATES_JSON__" not in page and "__AUTO_DELAY__" not in page

        st = json.loads(urllib.request.urlopen(base + "/state.json",
                                               timeout=5).read().decode("utf-8"))
        assert st["lora"]["mode"] == "pty" and st["remote"]["auto_on"] is False

        def _post(path_, payload):
            req = urllib.request.Request(
                base + path_, data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"}, method="POST")
            try:
                return json.loads(urllib.request.urlopen(
                    req, timeout=5).read().decode())
            except urllib.error.HTTPError as err:      # noqa: F821
                return {"__code__": err.code}

        resp = _post("/lora_tx", {"op": "DEPLOY"})
        assert resp["ok"] and fake2.written
        resp = _post("/lora_tx", {"op": "NOPE"})
        assert resp.get("__code__") == 400 and not resp.get("ok")
        resp = _post("/sim", {"toggle": "sensors"})
        assert resp["ok"] and core2.sim.view()["toggles"]["sensors"] is True
        resp = _post("/sim", {"once": "landed"})
        assert resp.get("ok") or not have_msgs   # 无 ROS 消息包时构造失败可接受
        resp = _post("/mode", {"mode": "walk"})
        assert resp["ok"]
        assert _wait_until(lambda: any("walk" in e["msg"]
                                       for e in core2.modes.view()["log"]))
        resp = _post("/mode", {"mode": "zzz"})
        assert resp.get("__code__") == 400 and not resp.get("ok")
        resp = _post("/remote_auto", {})
        assert resp["ok"] and core2.auto.view()["auto_on"] is True
        resp = _post("/mode_sim", {"on": True})
        assert resp.get("__code__") == 409 and not resp.get("ok")   # 离线无 ROS 服务工厂
        resp = _post("/mode_sim", {"on": False})
        assert resp["ok"]
        resp = _post("/lora_cfg", {"mode": "off"})
        assert resp["ok"] and core2.lora.mode == "off"

        req = urllib.request.Request(
            base + "/joy", data=json.dumps(
                {"axes": [0.2, -0.8], "buttons": [0, 1, 0]}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        resp = json.loads(urllib.request.urlopen(req, timeout=5).read().decode())
        assert resp["ok"] is True
        try:
            urllib.request.urlopen(urllib.request.Request(
                base + "/joy", data=json.dumps({"axes": "bad"}).encode("utf-8"),
                headers={"Content-Type": "application/json"}, method="POST"),
                timeout=5)
            raise AssertionError("坏手柄帧应 400")
        except urllib.error.HTTPError as err:
            assert err.code == 400
        print("[OK] HTTP 往返（页面/state.json/各 POST 端点/坏请求拒绝）")
    finally:
        httpd.shutdown()
        httpd.server_close()
        core2.lora.shutdown()

    # 10. JoyLink：透传/停报补零/空闲停发（沿用 v1 语义）
    joy_pub = _FakeJoyPub()
    clock2 = [100.0]
    joy = JoyLink(joy_pub, log=print, now_fn=lambda: clock2[0],
                  make_msg=lambda a, b: {"axes": list(a), "buttons": list(b)})
    ok, _ = joy.update([0.5, -0.5, 0, 0], [0, 1, 0])
    assert ok and len(joy_pub.frames) == 1
    joy.watchdog()
    assert len(joy_pub.frames) == 1
    clock2[0] += 2.0
    joy.watchdog()
    assert len(joy_pub.frames) == 2
    assert joy_pub.frames[1]["axes"] == [0.0] * JOY_AXES_LEN
    assert joy.view()["active"] is False
    print("[OK] JoyLink 帧透传/停报补零（松手）/空闲停发")

    print("selftest 全部通过")


class _FakeJoyPub:
    def __init__(self):
        self.frames = []

    def publish(self, msg):
        self.frames.append(msg)

    def get_num_connections(self):
        return 2


def main():
    parser = argparse.ArgumentParser(
        description="行为树 Web 实时看板 v2（完整树/模式控制/模拟注入/LoRa 地面站仿真）")
    parser.add_argument("--selftest", action="store_true",
                        help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        selftest()
        return
    run()


if __name__ == "__main__":
    main()
