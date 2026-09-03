#!/usr/bin/env python3
"""六足机器人行为树（py_trees 可运行实现）——统一模式执行版。

架构（对应 src/docs/BT_INTERFACES.md）：
    - 所有运动/任务动作统一为“模式”：home / walk / climb / dock /
      spin_search / release / approach / tag_nav；连续性子动作（tag 导引
      到充电桩、六腿抬起、夹爪夹紧、结束确认等）都在对应模式内部由执行
      节点自动完成，树中**一个模式只有一个 RunMode 节点**。
    - ~/switch_mode（SwitchMode.srv）：请求携带目标模式，自动执行该模式
      完整流程，响应即【最终结果】（bool success + string message）。不再
      需要单独的 ModeResult 查询服务。
    - 桥接 switch_mode() 返回 (state, message)，state ∈ RUNNING/SUCCESS/
      FAILED（RUNNING = 阻塞式服务尚未返回 / 模式执行中），供树非阻塞轮询。
    - 夹爪夹紧/松开：~/gripper_act（GripperAct.srv open/clamp），由
      release / dock 模式内部调用并折入最终结果，不出现在树中。
    - home（回到初始姿态，含复位）替换遥控 B 复位/回站/A 使能；主链不含
      遥控。遥控为独立测试树 build_remote_test_tree（每功能可直接切换）。

用法：
    python3 hexapod_bt.py --selftest                 # 离线自检
    python3 hexapod_bt.py --print-tree               # 主树结构
    python3 hexapod_bt.py --tick [--mission release] # 软件 tick 演示
    python3 hexapod_bt.py --remote-test dock         # 遥控测试树演示
"""

import argparse
import time

import py_trees
from py_trees.common import Status
from py_trees.decorators import Timeout


MODE_LABELS = {
    "home": "回到初始姿态(含复位)",
    "walk": "行走",
    "climb": "攀爬到小蓝上 ㉘",
    "dock": "对接夹紧 ㉙㉚（tag导引+抬腿+夹爪clamp+确认）",
    "spin_search": "自转搜索小蓝 ㉖",
    "release": "释放小蓝 ⑪（夹爪open）",
    "approach": "粗导航到可视tag ㉗",
    "tag_nav": "tag精导航到攀爬点",
}


# --------------------------------------------------------------------------
# 桥接上下文
# --------------------------------------------------------------------------
class BridgeContext:
    """行为树桥接句柄（实机由调用方实现；离线用 FakeBridge）。

    动作方法统一返回协议：True=完成，False=进行中，None=失败/放弃
    （返回 None 时树走失败回退分支）。
    """

    def __init__(self):
        self.status_log = []
        self.mission_mode = None  # None / "release" / "recover"

    # ---- 传感器（统一自检） ----
    def sensor_health(self):
        """全部传感器健康报告（IsSensorDataOk/WaitSensorsReady 消费）。

        订阅 /grasp_hexapod/sensor_health（SensorHealthArray，
        sensor_health_monitor 发布）+ 订阅 /grasp_hexapod/encoder_state
        （EncoderState，持续发布），其 normal 字段映射为 "encoder" 条目。
        返回 {name: {online,fresh,freq_hz,age_s,reason}}。
        """
        raise NotImplementedError

    def is_landing_confirmed(self):
        """⑩/㉔ 编码器落地判断（订阅 /grasp_hexapod/encoder_state，持续反馈）：

        landed → True；not_landed → False；normal=false → None（失败）。"""
        raise NotImplementedError

    def rtk_covariance_ok(self):
        """RTK 定位精度：/fix position_covariance 对角最大 ≤ 阈值（默认 0.04 m²）。"""
        raise NotImplementedError

    def hold_motion(self, reason):
        """零速保持（机器人停下），供 WaitRtkPrecise 协方差超限时调用。"""
        raise NotImplementedError

    # ---- 模式执行（统一服务，返回最终结果） ----
    def switch_mode(self, target_mode):
        """执行目标模式（调 ~/switch_mode，SwitchMode.srv）。

        服务自动执行该模式的完整连续性流程并返回【最终结果】。桥接方法返回
        (state, message)：
          state ∈ RUNNING（服务尚未返回/模式执行中）/ SUCCESS（成功）/
                  FAILED（失败，message 为问题）
        切换**幂等**：已在目标模式时返回其当前状态、不重复触发。
        """
        raise NotImplementedError

    # ---- 任务命令 / LoRa ----
    def receive_task_command(self):
        """⑤/⑲ 订阅 /lora/command：CMD,HEX,RELEASE,…→"release"；
        CMD,HEX,RECOVER,…→"recover"；无任务→None；其他→非法。"""
        raise NotImplementedError

    def wait_deployment(self, dt):
        """⑨ 订阅 /lora/command 的 CMD,HEX,DEPLOY,…。True=下放开始。"""
        raise NotImplementedError

    def wait_winch_hoisted(self, dt):
        """⑫/㉜ 订阅 /lora/command 的 CMD,HEX,HOIST_DONE,…。True=回收完成。"""
        raise NotImplementedError

    def report_status(self, status, dt=0.0):
        """LoRa 状态上报：发布 /lora/status STA,HEX,<status>,<x>,<y>。"""
        self.status_log.append(status)

    # ---- 遥控测试链专用（主链不使用） ----
    def read_remote_cmd(self):
        """遥控语义命令（仅 build_remote_test_tree 使用）。

        订阅 /grasp_hexapod/remote_cmd（RemoteCmd），返回
        {"mode": "idle|home|walk|climb|dock|spin_search|release", ...}。
        """
        raise NotImplementedError


# --------------------------------------------------------------------------
# 叶子节点
# --------------------------------------------------------------------------
class IsSensorDataOk(py_trees.behaviour.Behaviour):
    """统一传感器数据自检（每 tick）：任一传感器不 fresh 即中断主流程。"""

    def __init__(self, ctx, name="IsSensorDataOk"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        report = self.ctx.sensor_health()
        bad = [
            "{}({})".format(name, entry.get("reason") or "数据异常")
            for name, entry in report.items()
            if not entry.get("fresh", False)
        ]
        if bad:
            self.feedback_message = "传感器异常: " + ",".join(sorted(bad))
            return Status.FAILURE
        self.feedback_message = "传感器数据全部正常"
        return Status.SUCCESS


class WaitSensorsReady(py_trees.behaviour.Behaviour):
    """传感器上线门禁（SafetyInit 一次性启动检查）：全部 online 才放行。"""

    def __init__(self, ctx, name="WaitSensorsReady"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        report = self.ctx.sensor_health()
        offline = [name for name, entry in report.items() if not entry.get("online", False)]
        if offline:
            self.feedback_message = "等待传感器上线: " + ",".join(sorted(offline))
            return Status.RUNNING
        self.feedback_message = "传感器链路全部在线"
        return Status.SUCCESS


class WaitTaskCommand(py_trees.behaviour.Behaviour):
    """⑤/⑲ 等待任务命令：写入 ctx.mission_mode 后放行。"""

    def __init__(self, ctx, name="WaitTaskCommand"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        cmd = self.ctx.receive_task_command()
        if cmd is None:
            self.feedback_message = "等待地面任务命令（⑤/⑲）"
            return Status.RUNNING
        if cmd in ("release", "recover"):
            self.ctx.mission_mode = cmd
            self.feedback_message = "收到任务命令: {}".format(cmd)
            return Status.SUCCESS
        self.feedback_message = "非法任务命令: {}".format(cmd)
        return Status.FAILURE


class CheckMissionMode(py_trees.behaviour.Behaviour):
    """任务模式分流条件：ctx.mission_mode == mode。"""

    def __init__(self, ctx, mode, name=None):
        super().__init__(name or "CheckMissionMode_{}".format(mode))
        self.ctx = ctx
        self.mode = mode

    def update(self):
        return Status.SUCCESS if self.ctx.mission_mode == self.mode else Status.FAILURE


class CheckRemoteCmd(py_trees.behaviour.Behaviour):
    """遥控测试链：当前遥控命令 == mode 时才放行（主链不使用）。"""

    def __init__(self, ctx, mode, name=None):
        super().__init__(name or "CheckRemoteCmd_{}".format(mode))
        self.ctx = ctx
        self.mode = mode

    def update(self):
        cmd = self.ctx.read_remote_cmd()
        if cmd and cmd.get("mode") == self.mode:
            self.feedback_message = "遥控选择: {}".format(self.mode)
            return Status.SUCCESS
        return Status.FAILURE


class WaitRtkPrecise(py_trees.behaviour.Behaviour):
    """RTK 协方差监护：超限 → hold_motion 停走（RUNNING，暂停非中止）。"""

    def __init__(self, ctx, name="WaitRtkPrecise"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        if self.ctx.rtk_covariance_ok():
            self.feedback_message = "RTK 协方差达标"
            return Status.SUCCESS
        self.ctx.hold_motion("rtk_covariance")
        self.feedback_message = "RTK 协方差超限，停走等待恢复"
        return Status.RUNNING


class RunMode(py_trees.behaviour.Behaviour):
    """执行一个模式：调 ctx.switch_mode（统一 ~/switch_mode 服务）。

    该服务自动执行模式的完整连续性流程（如 dock 内含 tag导引+抬腿+夹爪
    clamp+结束确认），响应即最终结果。桥接把阻塞式服务包装为三态：
    RUNNING→RUNNING、SUCCESS→SUCCESS、FAILED→FAILURE（问题进 feedback）。
    切换幂等由桥接保证，反应式父级复检不会重复触发。
    """

    def __init__(self, ctx, mode, name=None):
        super().__init__(name or "RunMode_{}".format(mode))
        self.ctx = ctx
        self.mode = mode

    def update(self):
        state, message = self.ctx.switch_mode(self.mode)
        if state == "SUCCESS":
            self.feedback_message = "模式 {} 完成".format(self.mode)
            return Status.SUCCESS
        if state == "FAILED":
            self.feedback_message = "模式 {} 失败: {}".format(self.mode, message)
            return Status.FAILURE
        self.feedback_message = "模式 {} 执行中".format(self.mode)
        return Status.RUNNING


class WaitDeployment(py_trees.behaviour.Behaviour):
    """⑨ 等待绞盘下放·六足下降开始。"""

    def __init__(self, ctx, name="WaitDeployment"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        return _action_status(self.ctx.wait_deployment(self.dt))


class IsLandingConfirmed(py_trees.behaviour.Behaviour):
    """⑩/㉔ 编码器落地判断（topic 持续反馈三态）。"""

    def __init__(self, ctx, name="IsLandingConfirmed"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        result = self.ctx.is_landing_confirmed()
        if result is None:
            self.feedback_message = "编码器故障"
            return Status.FAILURE
        if result:
            self.feedback_message = "编码器确认落地"
            return Status.SUCCESS
        self.feedback_message = "未落地，继续等待"
        return Status.RUNNING


class ReportStatus(py_trees.behaviour.Behaviour):
    def __init__(self, ctx, status, name="ReportStatus"):
        super().__init__(name)
        self.ctx = ctx
        # 不能用 self.status（与 py_trees Behaviour.status 冲突）。
        self.report_value = status

    def update(self):
        self.ctx.report_status(self.report_value, self.dt)
        return Status.SUCCESS


class WaitWinchHoisted(py_trees.behaviour.Behaviour):
    def __init__(self, ctx, name="WaitWinchHoisted"):
        super().__init__(name)
        self.ctx = ctx

    def update(self):
        return _action_status(self.ctx.wait_winch_hoisted(self.dt))


# --------------------------------------------------------------------------
# 树构建
# --------------------------------------------------------------------------
def _action_status(result):
    """True=完成(SUCCESS)；False=进行中(RUNNING)；None=失败(FAILURE)。"""
    if result is None:
        return Status.FAILURE
    return Status.SUCCESS if result else Status.RUNNING


def _run_mode(ctx, mode):
    """单模式执行：一个 RunMode 节点（一模式一节点）。"""
    label = MODE_LABELS.get(mode, mode)
    return RunMode(ctx, mode, name="执行 {}".format(label))


def build_hexapod_tree(ctx, deploy_timeout_s=120.0, landing_timeout_s=120.0,
                       comms_timeout_s=10.0, rtk_wait_timeout_s=60.0):
    """构建主链完整行为树（无遥控）。

    结构：
      任务失败回退（Selector）
      ├─ 主流程_带安全监视（Sequence memory=False：IsSensorDataOk 每 tick 复检）
      │   └─ 任务阶段序列（memory=True）
      │       ├─ WaitTaskCommand ⑤/⑲
      │       ├─ SafetyInit：上线门禁 → home 回到初始姿态(含复位)
      │       ├─ DeployAndLand ⑨ → ⑩/㉔
      │       ├─ 释放/回收分流（Selector）
      │       │   ├─ 释放分支：RunMode("release") → RELEASED → ⑫
      │       │   └─ 回收分支：LANDED → [spin_search→approach(RTK监护)]
      │       │            → RunMode("tag_nav") → RunMode("climb")
      │       │            → RunMode("dock") → CLAMPED → ㉜
      │       └─ ReportStatus DONE ㊱
      └─ 失败处理：home(尽力) → ReportStatus FAILED
    """

    # ---- SafetyInit：传感器上线 -> home 回到初始姿态(含复位) ----
    safety_init = py_trees.composites.Sequence(
        name="SafetyInit 安全初始化", memory=True, children=[
            Timeout(
                name="传感器上线超时",
                child=WaitSensorsReady(ctx),
                duration=comms_timeout_s,
            ),
            _run_mode(ctx, "home"),
        ])

    # ---- DeployAndLand：⑨ 绞盘下放 -> ⑩/㉔ 落地确认 ----
    deploy_and_land = py_trees.composites.Sequence(
        name="DeployAndLand 下放落地", memory=True, children=[
            Timeout(
                name="下放等待超时",
                child=WaitDeployment(ctx),
                duration=deploy_timeout_s,
            ),
            Timeout(
                name="落地超时",
                child=IsLandingConfirmed(ctx),
                duration=landing_timeout_s,
            ),
        ])

    # ---- 释放分支（⑤ 任务）：release 模式内部完成（含夹爪 open） ----
    release_branch = py_trees.composites.Sequence(
        name="释放分支_释放小蓝", memory=True, children=[
            CheckMissionMode(ctx, "release", name="IsReleaseMission"),
            _run_mode(ctx, "release"),
            ReportStatus(ctx, status="RELEASED"),
            WaitWinchHoisted(ctx),
        ])

    # ---- 定位导航（spin_search + approach）带 RTK 协方差监护 ----
    # 外层 memory=False（反应式）：WaitRtkPrecise 每 tick 复检，协方差超限即
    # hold_motion 停走并暂停内层；内层 memory=True 保证每模式只触发一次。
    locate_and_nav = py_trees.composites.Sequence(
        name="定位导航_带RTK精度监视", memory=False, children=[
            Timeout(
                name="RTK等待超时",
                child=WaitRtkPrecise(ctx),
                duration=rtk_wait_timeout_s,
            ),
            py_trees.composites.Sequence(
                name="定位导航步骤", memory=True, children=[
                    _run_mode(ctx, "spin_search"),
                    _run_mode(ctx, "approach"),
                ]),
        ])

    # ---- 回收分支（⑲ 任务）：tag_nav → climb → dock ----
    recover_branch = py_trees.composites.Sequence(
        name="回收分支_抓取回收", memory=True, children=[
            CheckMissionMode(ctx, "recover", name="IsRecoveryMission"),
            ReportStatus(ctx, status="LANDED"),        # ㉕ 落地状态回传
            locate_and_nav,                            # ㉖㉗ 搜索/定位/粗导航
            _run_mode(ctx, "tag_nav"),                 # 识别tag→到达攀爬点
            _run_mode(ctx, "climb"),                   # ㉘ 攀爬(含姿态准备)
            _run_mode(ctx, "dock"),                    # ㉙㉚ 对接(导引+抬腿+夹爪)
            ReportStatus(ctx, status="CLAMPED"),       # ㉛ 回传夹紧完成
            WaitWinchHoisted(ctx),                     # ㉜ 拉升绞盘回收
        ])

    # ---- 释放/回收分流 ----
    mission_branch = py_trees.composites.Selector(
        name="释放或回收分流", memory=False, children=[
            release_branch,
            recover_branch,
        ])

    # ---- 主流程：IsSensorDataOk 每 tick 复检 ----
    mission_flow = py_trees.composites.Sequence(
        name="主流程_带安全监视", memory=False, children=[
            IsSensorDataOk(ctx),
            py_trees.composites.Sequence(
                name="任务阶段序列", memory=True, children=[
                    WaitTaskCommand(ctx),
                    safety_init,
                    deploy_and_land,
                    mission_branch,
                    ReportStatus(ctx, status="DONE"),   # ㊱ 完成通知
                ]),
        ])

    # ---- 失败处理：home 尽力回初始姿态(含复位)，无论成败都上报 FAILED ----
    failure_handling = py_trees.composites.Sequence(
        name="失败处理", memory=True, children=[
            py_trees.composites.Selector(
                name="回到初始姿态(尽力)", memory=False, children=[
                    _run_mode(ctx, "home"),
                    py_trees.behaviours.Success(name="home不可用也继续"),
                ]),
            ReportStatus(ctx, status="FAILED"),
        ])

    root = py_trees.composites.Selector(
        name="任务失败回退", memory=False, children=[
            mission_flow,
            failure_handling,
        ])
    root.context = ctx
    return root


def build_remote_test_tree(ctx):
    """遥控独立测试链：每个功能可直接切换（RunMode），不必走完整流程。

    主链不包含本树。walk 为连续模式（RunMode 保持 RUNNING 直到遥控切走）；
    其余为一次性模式。空闲（未选中任何功能）时落在末尾 Success 待命节点。
    """
    children = []
    for mode in ("home", "walk", "climb", "dock", "spin_search", "release"):
        children.append(py_trees.composites.Sequence(
            name="测试模式_{}".format(mode), memory=False, children=[
                CheckRemoteCmd(ctx, mode, name="遥控选择{}".format(mode)),
                _run_mode(ctx, mode),
            ]))
    children.append(py_trees.behaviours.Success(name="空闲待命"))
    root = py_trees.composites.Selector(
        name="遥控器测试链", memory=False, children=children)
    root.context = ctx
    return root


# --------------------------------------------------------------------------
# 离线假实现
# --------------------------------------------------------------------------
SENSOR_NAMES = ("imu", "gps", "rtk", "servo", "stereo", "mono", "encoder")


def _healthy(name):
    return {"online": True, "fresh": True, "freq_hz": 50.0,
            "age_s": 0.02, "reason": ""}


class FakeBridge(BridgeContext):
    """可脚本化的假桥接：switch_mode 按时间线返回 (state, message)。

    script 特殊键：
      mission           "release"/"recover"（任务命令内容）
      task_cmd_value    非法任务命令字符串
      switch_fail_modes [mode, ...]          切这些模式立即 FAILED
      mode_fail         {mode: {"at": t, "message": str}}  到时刻返回 FAILED
      mode_never        [mode, ...]          模式结果永 RUNNING
      sensor_bad / sensor_offline / cov_bad_windows / cov_always_bad
      remote_target     "home|walk|climb|dock|spin_search|release"
    times 键：
      task_cmd, deploy, landing, winch_done, 以及各模式完成时刻（mode_<mode>）
    """

    def __init__(self, script=None):
        super().__init__()
        self.t = 0.0
        self.times = {
            "task_cmd": 0.5,
            "deploy": 5.0, "landing": 8.0,
            "winch_done": 70.0,
            "mode_home": 2.0,
            "mode_release": 10.0,
            "mode_spin_search": 11.0,
            "mode_approach": 20.0,
            "mode_tag_nav": 22.0,
            "mode_climb": 40.0,
            "mode_dock": 60.0,
        }
        self.mission_mode_cmd = "recover"
        self.task_cmd_value = None
        self.switch_fail_modes = []
        self.mode_fail = {}
        self.mode_never = []
        self.sensor_bad = None
        self.sensor_offline = None
        self.cov_bad_windows = []
        self.cov_always_bad = False
        self.remote_target = "idle"
        self.switch_log = []          # (t, mode) 记录实际切换进入
        self._active_mode = None
        if script:
            self.times.update(script)
            for key in ("mission", "task_cmd_value", "switch_fail_modes",
                        "mode_fail", "mode_never", "sensor_bad", "sensor_offline",
                        "cov_bad_windows", "cov_always_bad", "remote_target"):
                if key in script:
                    setattr(self, "mission_mode_cmd" if key == "mission" else key,
                            script[key])

    def _done(self, key):
        return self.t >= self.times[key]

    def _mode_done(self, mode):
        return self.t >= self.times.get("mode_" + mode, 1e9)

    # ---- 传感器 ----
    def sensor_health(self):
        report = {name: _healthy(name) for name in SENSOR_NAMES}
        if self.sensor_bad and self.sensor_bad in report:
            entry = report[self.sensor_bad]
            entry.update({"fresh": False, "freq_hz": 0.0,
                          "age_s": 9.99, "reason": "频率 0Hz"})
        if self.sensor_offline and self.sensor_offline in report:
            entry = report[self.sensor_offline]
            entry.update({"online": False, "fresh": False, "freq_hz": 0.0,
                          "age_s": 9.99, "reason": "离线/无消息"})
        return report

    def is_landing_confirmed(self):
        if not self.sensor_health()["encoder"]["fresh"]:
            return None
        return self._done("landing")

    def rtk_covariance_ok(self):
        if self.cov_always_bad:
            return False
        return not any(s <= self.t <= e for s, e in self.cov_bad_windows)

    def hold_motion(self, reason):
        pass

    # ---- 模式执行（返回最终结果 state/message） ----
    def switch_mode(self, target_mode):
        if target_mode in self.switch_fail_modes:
            return ("FAILED", "模式 {} 被拒绝".format(target_mode))
        # 幂等：已在目标模式不重复记录；切到新模式则记录一次
        if self._active_mode != target_mode:
            self._active_mode = target_mode
            self.switch_log.append((self.t, target_mode))
        if target_mode in self.mode_fail:
            cfg = self.mode_fail[target_mode]
            if self.t >= cfg.get("at", 0.0):
                return ("FAILED", cfg.get("message", "失败"))
        if target_mode in self.mode_never:
            return ("RUNNING", "")
        if self._mode_done(target_mode):
            return ("SUCCESS", "")
        return ("RUNNING", "")

    # ---- 任务/LoRa ----
    def receive_task_command(self):
        if not self._done("task_cmd"):
            return None
        return self.task_cmd_value or self.mission_mode_cmd

    def wait_deployment(self, dt):
        return self._done("deploy")

    def wait_winch_hoisted(self, dt):
        return self._done("winch_done")

    # ---- 遥控（仅测试链） ----
    def read_remote_cmd(self):
        return {"mode": self.remote_target,
                "vx": 0.0, "vy": 0.0, "vyaw": 0.0,
                "reset_edge": False, "enable_edge": False,
                "climb_edge": False, "dock_edge": False}


# --------------------------------------------------------------------------
# 自检与演示
# --------------------------------------------------------------------------
def print_tree(tree):
    def walk(node, depth=0):
        print("  " * depth + "- " + node.name)
        for child in node.children:
            walk(child, depth + 1)

    walk(tree)


def run_until_done(tree, ctx, dt=0.5, max_steps=400, wall_sleep=0.0):
    """推进树到终态。wall_sleep>0 时配合真实时钟 Timeout 使用。"""
    status = Status.RUNNING
    step = 0
    while status == Status.RUNNING and step < max_steps:
        ctx.t += dt
        for node in tree.iterate():
            node.dt = dt
        if wall_sleep:
            time.sleep(wall_sleep)
        tree.tick_once()
        status = tree.status
        step += 1
    return status


def _tick(tree, ctx, dt):
    ctx.t += dt
    for node in tree.iterate():
        node.dt = dt
    tree.tick_once()
    return tree.status


def selftest():
    """离线验证：单模式节点、最终结果由 switch_mode 返回、失败回退、遥控测试链。"""
    # --- 1. 回收任务正常推进 ---
    ctx = FakeBridge()
    tree = build_hexapod_tree(ctx)
    status = run_until_done(tree, ctx)
    assert status == Status.SUCCESS, "recover: {}".format(status)
    assert ctx.status_log == ["LANDED", "CLAMPED", "DONE"], ctx.status_log
    switched = [m for _, m in ctx.switch_log]
    assert switched == ["home", "spin_search", "approach",
                        "tag_nav", "climb", "dock"], switched
    print("[OK] 回收任务: 模式序列 =", switched, "状态上报 =", ctx.status_log)

    # --- 2. 释放任务正常推进（release 模式内部完成夹爪 open） ---
    ctx2 = FakeBridge(script={"mission": "release"})
    tree2 = build_hexapod_tree(ctx2)
    status2 = run_until_done(tree2, ctx2)
    assert status2 == Status.SUCCESS, "release: {}".format(status2)
    assert ctx2.status_log == ["RELEASED", "DONE"], ctx2.status_log
    switched2 = [m for _, m in ctx2.switch_log]
    assert switched2 == ["home", "release"], switched2
    print("[OK] 释放任务: 模式序列 =", switched2, "状态上报 =", ctx2.status_log)

    # --- 3. 模式执行被拒绝（home）-> 失败回退 ---
    ctx3 = FakeBridge(script={"switch_fail_modes": ["home"]})
    tree3 = build_hexapod_tree(ctx3)
    status3 = run_until_done(tree3, ctx3)
    assert status3 == Status.SUCCESS and ctx3.status_log == ["FAILED"], ctx3.status_log
    print("[OK] 切换被拒绝回退: 状态上报 =", ctx3.status_log)

    # --- 4. dock 模式最终结果 FAILED（夹爪夹紧失败）-> 失败回退 ---
    ctx4 = FakeBridge(script={"mode_fail": {"dock": {"at": 40.0,
                                                     "message": "夹爪受限"}}})
    tree4 = build_hexapod_tree(ctx4)
    status4 = run_until_done(tree4, ctx4)
    assert status4 == Status.SUCCESS and ctx4.status_log == ["LANDED", "FAILED"], (
        ctx4.status_log)
    print("[OK] dock 最终结果失败回退: 状态上报 =", ctx4.status_log)

    # --- 5. release 最终结果 FAILED（夹爪 open 失败）-> 失败回退 ---
    ctx5 = FakeBridge(script={"mission": "release",
                              "mode_fail": {"release": {"at": 6.0,
                                                        "message": "夹爪离线"}}})
    tree5 = build_hexapod_tree(ctx5)
    status5 = run_until_done(tree5, ctx5)
    assert status5 == Status.SUCCESS and ctx5.status_log == ["FAILED"], ctx5.status_log
    print("[OK] release 最终结果失败回退: 状态上报 =", ctx5.status_log)

    # --- 6. tag_nav 导航失败 -> 失败回退 ---
    ctx6 = FakeBridge(script={"mode_fail": {"tag_nav": {"at": 21.0,
                                                        "message": "未识别到tag"}}})
    tree6 = build_hexapod_tree(ctx6)
    status6 = run_until_done(tree6, ctx6)
    assert status6 == Status.SUCCESS and ctx6.status_log == ["LANDED", "FAILED"], (
        ctx6.status_log)
    print("[OK] tag_nav 最终结果失败回退: 状态上报 =", ctx6.status_log)

    # --- 7. 传感器数据异常（任一）-> 失败回退 ---
    for sensor in SENSOR_NAMES:
        ctx7 = FakeBridge(script={"sensor_bad": sensor})
        tree7 = build_hexapod_tree(ctx7)
        status7 = run_until_done(tree7, ctx7)
        assert status7 == Status.SUCCESS and ctx7.status_log == ["FAILED"], (
            "{} {}".format(sensor, ctx7.status_log))
    print("[OK] 传感器数据异常回退")

    # --- 8. 传感器通信离线 -> 上线门禁超时 -> 失败回退 ---
    ctx8 = FakeBridge(script={"sensor_offline": "gps"})
    tree8 = build_hexapod_tree(ctx8, comms_timeout_s=0.2)
    status8 = run_until_done(tree8, ctx8, wall_sleep=0.05)
    assert status8 == Status.SUCCESS and ctx8.status_log == ["FAILED"], ctx8.status_log
    print("[OK] 传感器通信离线回退: 状态上报 =", ctx8.status_log)

    # --- 9. 落地超时 -> 失败回退 ---
    ctx9 = FakeBridge(script={"landing": 1e9})
    tree9 = build_hexapod_tree(ctx9, landing_timeout_s=0.2)
    status9 = run_until_done(tree9, ctx9, wall_sleep=0.05)
    assert status9 == Status.SUCCESS and ctx9.status_log == ["FAILED"], ctx9.status_log
    print("[OK] 落地超时回退: 状态上报 =", ctx9.status_log)

    # --- 10. 下放等待超时 -> 失败回退 ---
    ctx10 = FakeBridge(script={"deploy": 1e9})
    tree10 = build_hexapod_tree(ctx10, deploy_timeout_s=0.2)
    status10 = run_until_done(tree10, ctx10, wall_sleep=0.05)
    assert status10 == Status.SUCCESS and ctx10.status_log == ["FAILED"], ctx10.status_log
    print("[OK] 下放等待超时回退: 状态上报 =", ctx10.status_log)

    # --- 11. RTK 协方差超限 -> 停走 -> 恢复继续到 DONE ---
    ctx11 = FakeBridge(script={"cov_bad_windows": [(21.5, 23.0)]})
    tree11 = build_hexapod_tree(ctx11)
    status11 = run_until_done(tree11, ctx11)
    assert status11 == Status.SUCCESS and ctx11.status_log == ["LANDED", "CLAMPED", "DONE"], (
        ctx11.status_log)
    print("[OK] RTK 协方差超限停走恢复后继续: 状态上报 =", ctx11.status_log)

    # --- 12. 非法任务命令 -> 失败回退 ---
    ctx12 = FakeBridge(script={"task_cmd_value": "BOGUS"})
    tree12 = build_hexapod_tree(ctx12)
    status12 = run_until_done(tree12, ctx12)
    assert status12 == Status.SUCCESS and ctx12.status_log == ["FAILED"], ctx12.status_log
    print("[OK] 非法任务命令回退: 状态上报 =", ctx12.status_log)

    # --- 13. 遥控测试链：每个功能单独可切 ---
    for mode in ("home", "climb", "dock", "spin_search", "release", "walk"):
        ctx13 = FakeBridge(script={"remote_target": mode})
        tree13 = build_remote_test_tree(ctx13)
        if mode == "walk":
            # walk 连续模式：选中时保持 RUNNING
            status13 = _tick(tree13, ctx13, 0.5)
            assert status13 == Status.RUNNING, status13
            ctx13.remote_target = "idle"
            status13 = _tick(tree13, ctx13, 0.5)
            assert status13 == Status.SUCCESS, status13  # 空闲待命
            print("[OK] 遥控测试链 walk: 保持直至切出")
            continue
        status13 = run_until_done(tree13, ctx13, max_steps=300)
        assert status13 == Status.SUCCESS, "{} {}".format(mode, status13)
        switched13 = [m for _, m in ctx13.switch_log]
        assert switched13 == [mode], "{} {}".format(mode, switched13)
        print("[OK] 遥控测试链单切: {} -> SUCCESS".format(mode))

    print("selftest 全部通过")


def main():
    parser = argparse.ArgumentParser(description="六足行为树 py_trees 统一模式执行版")
    parser.add_argument("--selftest", action="store_true", help="离线自检")
    parser.add_argument("--print-tree", action="store_true", help="打印主树结构")
    parser.add_argument("--tick", action="store_true", help="主树软件 tick 演示")
    parser.add_argument("--mission", choices=["release", "recover"], default="recover",
                        help="tick 演示的任务模式")
    parser.add_argument("--remote-test", metavar="MODE",
                        help="遥控测试树演示：home|walk|climb|dock|spin_search|release")
    parser.add_argument("--render-dot", metavar="PATH", help="渲染树状图为 SVG（需 graphviz）")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return

    if args.remote_test:
        ctx = FakeBridge(script={"remote_target": args.remote_test})
        tree = build_remote_test_tree(ctx)
        print("遥控测试树演示（目标 {}）：".format(args.remote_test))
        status = Status.RUNNING
        step = 0
        while status == Status.RUNNING and step < 300:
            ctx.t += 0.5
            for node in tree.iterate():
                node.dt = 0.5
            tree.tick_once()
            status = tree.status
            step += 1
        print("最终状态: {}，模式记录: {}".format(
            status.name, [m for _, m in ctx.switch_log]))
        return

    ctx = FakeBridge(script={"mission": args.mission})
    tree = build_hexapod_tree(ctx)

    if args.print_tree:
        print_tree(tree)
        return

    if args.tick:
        print("主树 tick 演示（{} 任务，0.5s/步）：".format(args.mission))
        status = Status.RUNNING
        step = 0
        while status == Status.RUNNING and step < 300:
            ctx.t += 0.5
            for node in tree.iterate():
                node.dt = 0.5
            tree.tick_once()
            status = tree.status
            step += 1
            if step % 10 == 0:
                print("  t={:5.1f}s status={} log={}".format(
                    ctx.t, status.name, ctx.status_log))
        print("最终状态: {}，状态上报: {}".format(status.name, ctx.status_log))
        return

    if args.render_dot:
        import os
        from py_trees.display import render_dot_tree
        out = args.render_dot
        directory, name = os.path.split(out)
        if not directory:
            directory = "."
        if not name.endswith(".svg"):
            name += ".svg"
        files = render_dot_tree(tree, name=name, target_directory=directory)
        print("已生成:", files)
        return

    print_tree(tree)
    print("\n用法：--selftest | --print-tree | --tick [--mission release] | "
          "--remote-test MODE | --render-dot <路径>")


if __name__ == "__main__":
    main()
