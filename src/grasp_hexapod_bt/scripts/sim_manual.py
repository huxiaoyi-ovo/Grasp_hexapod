#!/usr/bin/env python3
"""手动单步模拟节点：实物缺失时的**唯一手动模拟源**（替代旧版看板内置注入）。

dashboard 面板按钮 → /grasp_hexapod/sim_inject 服务（动作 id）→ 本节点发布
对应话题/应答服务，让行为树**一次只通过当前这一小步**（不整段跳过）。

与 sim_feedback 的分工（同接口会互相覆盖/服务冲突，**勿同时运行**）：
    sim_feedback  自动时间线回归（按下放/落地/模式完成时刻自动推进）
    sim_manual    手动单步联调（每一步都由人点击放行，可控点更细）

接口（标准名不变，契约见 src/docs/BT_INTERFACES.md）：
    服务(托管)  sim_inject   /grasp_hexapod/sim_inject   （看板按钮调用，常驻）
                switch_mode  /grasp_hexapod/switch_mode  （开关式挂载，探测占用）
                gripper_act  /grasp_hexapod/gripper_act  （开关式挂载）
    话题(发布)  encoder_state /grasp_hexapod/encoder_state
                sensor_health /grasp_hexapod/sensor_health
                fix           /fix
                lora_command  /lora/command
                remote_cmd    /grasp_hexapod/remote_cmd
                sim_state     /grasp_hexapod/sim_state   （面板状态 JSON，2Hz）
    话题(订阅)  bt_state     /grasp_hexapod/bt_state     （active_phase 供 step_next）

按钮动作注册表（ACTIONS/GROUPS/PHASE_HINTS）是本节点与 bt_dashboard 面板的
**单一数据源**（dashboard 同目录 import）。

用法：
    rosrun grasp_hexapod_bt sim_manual.py                    # mission=recover
    rosrun grasp_hexapod_bt sim_manual.py _mission:=release
    python3 sim_manual.py --selftest                         # 离线自检（不依赖 ROS）
"""

import argparse
import collections
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# 动作注册表（bt_dashboard 面板按钮与本节点路由的单一数据源）
# ---------------------------------------------------------------------------
SENSOR_NAMES = ("imu", "gps", "rtk", "servo", "stereo", "mono")
MODES = ("home", "walk", "climb", "dock", "spin_search", "release",
         "approach", "tag_nav")
REMOTE_MODES = ("idle", "home", "walk", "climb", "dock", "spin_search",
                "release")

SWITCH_MODE_SERVICE = "/grasp_hexapod/switch_mode"
GRIPPER_ACT_SERVICE = "/grasp_hexapod/gripper_act"
SIM_INJECT_SERVICE = "/grasp_hexapod/sim_inject"
SIM_STATE_TOPIC = "/grasp_hexapod/sim_state"
BT_STATE_TOPIC = "/grasp_hexapod/bt_state"
MODE_STEP_TIMEOUT_S = 300.0        # 模式单步确认最长等待（超时返回失败）
GRIPPER_STEP_TIMEOUT_S = 120.0     # 夹爪单步确认最长等待

GROUPS = ["⋆ 手动单步", "① 任务命令", "② 传感器", "③ 下放·落地·编码器",
          "④ 模式执行·夹爪", "⑤ RTK", "⑥ 绞盘回收", "⑦ 遥控测试链",
          "⑧ 指定模式失败"]

ACTIONS = {
    "step_next": {"label": "▶ 通过当前步骤", "group": "⋆ 手动单步",
                  "kind": "once", "cls": "hi",
                  "note": "按 active_phase 自动识别当前卡点并只放行这一小步"
                          "（任务命令按 ~mission 注入）"},
    "task_recover": {"label": "RECOVER 回收", "group": "① 任务命令", "kind": "once",
                     "note": "CMD,HEX,RECOVER → /lora/command（锁存，单击一次）"},
    "task_release": {"label": "RELEASE 释放", "group": "① 任务命令", "kind": "once",
                     "note": "CMD,HEX,RELEASE → /lora/command（锁存，单击一次）"},
    "task_bogus": {"label": "非法命令 BOGUS", "group": "① 任务命令", "kind": "once",
                   "cls": "warn",
                   "note": "注入非法任务命令 → 测试 WaitTaskCommand 失败回退路径"},
    "sensors_ok": {"label": "全健康帧（单次）", "group": "② 传感器", "kind": "once",
                   "note": "sensor_health 六路全 fresh + 编码器未落地帧"
                          "（落地门由「确认落地」独立放行）"},
    "hold_toggle": {"label": "持续保持 5Hz", "group": "② 传感器", "kind": "toggle",
                    "bind": "hold",
                    "note": "重发 健康帧+编码器未落地帧+/fix，对抗真实 monitor 覆盖；结束记得关"},
    "deploy": {"label": "DEPLOY 下放开始", "group": "③ 下放·落地·编码器", "kind": "once",
               "note": "CMD,HEX,DEPLOY → /lora/command"},
    "landed": {"label": "确认落地", "group": "③ 下放·落地·编码器", "kind": "once",
               "note": "encoder_state landed=true（一帧缓存永久生效）"},
    "not_landed": {"label": "未落地", "group": "③ 下放·落地·编码器", "kind": "once",
                   "note": "landed=false：落地确认保持等待（RUNNING）"},
    "mode_service_toggle": {"label": "手动模式服务", "group": "④ 模式执行·夹爪",
                            "kind": "toggle", "bind": "mode_service",
                            "note": "挂载 switch_mode；与 sim_feedback/实机 mode_server 互斥"},
    "mode_step_toggle": {"label": "模式单步确认", "group": "④ 模式执行·夹爪",
                         "kind": "toggle",
                         "note": "开启后每次 switch_mode 阻塞等待面板确认才放行（一步一点；需手动模式服务）"},
    "mode_confirm_ok": {"label": "确认模式成功", "group": "④ 模式执行·夹爪",
                        "kind": "once",
                        "note": "放行当前单步等待的模式（SUCCESS）"},
    "mode_confirm_fail": {"label": "确认模式失败", "group": "④ 模式执行·夹爪",
                          "kind": "once", "cls": "warn",
                          "note": "当前等待的模式返回失败（走失败回退路径）"},
    "mode_fail_next": {"label": "下次模式失败", "group": "④ 模式执行·夹爪", "kind": "once",
                       "cls": "warn",
                       "note": "装填：下一次 switch_mode 返回失败（一次性，测试失败回退）"},
    "gripper_service_toggle": {"label": "手动夹爪服务", "group": "④ 模式执行·夹爪",
                               "kind": "toggle", "bind": "gripper",
                               "note": "挂载 gripper_act open/clamp（模式内部调用）"},
    "gripper_step_toggle": {"label": "夹爪单步确认", "group": "④ 模式执行·夹爪",
                            "kind": "toggle",
                            "note": "开启后夹爪 open/clamp 阻塞等待面板确认（需手动夹爪服务）"},
    "gripper_confirm_ok": {"label": "确认夹爪成功", "group": "④ 模式执行·夹爪",
                           "kind": "once",
                           "note": "放行当前单步等待的夹爪动作"},
    "gripper_confirm_fail": {"label": "确认夹爪失败", "group": "④ 模式执行·夹爪",
                             "kind": "once", "cls": "warn",
                             "note": "当前等待的夹爪动作返回失败"},
    "gripper_fail_open": {"label": "下次 open 失败", "group": "④ 模式执行·夹爪",
                          "kind": "once", "cls": "warn",
                          "note": "装填：下一次夹爪 open 返回失败（一次性，测试 release 失败路径）"},
    "gripper_fail_clamp": {"label": "下次 clamp 失败", "group": "④ 模式执行·夹爪",
                           "kind": "once", "cls": "warn",
                           "note": "装填：下一次夹爪 clamp 返回失败（一次性，测试 dock 失败路径）"},
    "rtk_good": {"label": "良好协方差 /fix", "group": "⑤ RTK", "kind": "once",
                 "note": "对角 0.01 m²（阈值 0.04）→ 解除 RTK 停走"},
    "rtk_bad": {"label": "协方差超限 /fix", "group": "⑤ RTK", "kind": "once",
                "cls": "warn",
                "note": "对角 9.0 m² → 触发 RTK 停走等待（60s 超时路径）；恢复点\"良好协方差\""},
    "hoist_done": {"label": "HOIST_DONE 回收完成", "group": "⑥ 绞盘回收", "kind": "once",
                   "note": "CMD,HEX,HOIST_DONE → /lora/command"},
    "home_cmd": {"label": "HOME 恢复初始", "group": "⑥ 绞盘回收", "kind": "once",
                 "note": "CMD,HEX,HOME → /lora/command（恢复初始命令，放行 WaitHomeCmd 门）"},
}
for _mode in REMOTE_MODES:
    ACTIONS["remote_" + _mode] = {
        "label": "remote: " + _mode, "group": "⑦ 遥控测试链", "kind": "once",
        "note": "remote_cmd.mode={}（仅 remote_test 运行器生效）".format(_mode)}
for _name in SENSOR_NAMES:
    ACTIONS["sensor_bad_" + _name] = {
        "label": _name + " 异常", "group": "② 传感器", "kind": "once",
        "cls": "warn",
        "note": "注入 {} 不新鲜帧 → 触发 IsSensorDataOk 停走保持（真实 monitor 在跑时约 0.2s 后被覆盖，瞬时毛刺测试用；恢复点\"全健康帧\"）".format(_name)}
for _mode in MODES:
    ACTIONS["mode_fail_" + _mode] = {
        "label": "fail: " + _mode, "group": "⑧ 指定模式失败", "kind": "once",
        "cls": "warn",
        "note": "装填：switch_mode({}) 返回失败（一次性，比\"下次模式失败\"更精准）".format(_mode)}

# active_phase 前缀 -> 建议按钮 + 提示文案（看板据此高亮"当前卡点"）
PHASE_HINTS = [
    {"match": "WaitTaskCommand", "actions": ["step_next", "task_recover", "task_release"],
     "tip": "等待地面任务命令（⑤/⑲）→ ▶通过当前步骤 或点 RECOVER / RELEASE 注入"},
    {"match": "WaitSensorsReady", "actions": ["step_next", "sensors_ok", "hold_toggle"],
     "tip": "等待传感器上线（启动门禁）→ ▶通过当前步骤 注入全健康帧"},
    {"match": "IsSensorDataOk", "actions": ["step_next", "sensors_ok", "hold_toggle"],
     "tip": "传感器数据异常停走中 → ▶通过当前步骤 注入全健康帧恢复"},
    {"match": "WaitDeployment", "actions": ["step_next", "deploy"],
     "tip": "等待绞盘下放开始（⑨）→ ▶通过当前步骤 注入 DEPLOY"},
    {"match": "IsLandingConfirmed", "actions": ["step_next", "landed"],
     "tip": "等待编码器确认落地（⑩/㉔）→ ▶通过当前步骤 注入确认落地"},
    {"match": "WaitRtkPrecise", "actions": ["step_next", "rtk_good"],
     "tip": "RTK 协方差超限停走等待 → ▶通过当前步骤 注入良好 /fix 解锁"},
    {"match": "WaitWinchHoisted", "actions": ["step_next", "hoist_done"],
     "tip": "等待绞盘回收完成（⑫/㉜）→ ▶通过当前步骤 注入 HOIST_DONE"},
    {"match": "WaitHomeCmd", "actions": ["step_next", "home_cmd"],
     "tip": "等待地面恢复初始命令 → ▶通过当前步骤 注入 HOME"},
    {"match": "执行 ", "actions": ["step_next", "mode_service_toggle", "mode_step_toggle"],
     "tip": "模式执行中（阻塞于 switch_mode 服务）；无执行器时开手动模式服务，逐步调试加开「模式单步确认」"},
]

# step_next 的 phase 前缀 -> 动作映射（None 表示按 mission 动态取任务命令）
STEP_PREFIXES = [
    ("WaitTaskCommand", None),
    ("WaitSensorsReady", "sensors_ok"),
    ("IsSensorDataOk", "sensors_ok"),
    ("WaitDeployment", "deploy"),
    ("IsLandingConfirmed", "landed"),
    ("WaitRtkPrecise", "rtk_good"),
    ("WaitWinchHoisted", "hoist_done"),
    ("WaitHomeCmd", "home_cmd"),
]


def resolve_step_action(phase, mission):
    """step_next 纯函数：active_phase 前缀 -> (action, reason)。

    action 为 None 时 reason 说明不可放行的原因（调用方据此返回失败提示）。
    RunMode 阶段（"执行 " 前缀）交由调用方结合单步状态处理，不在此映射。
    """
    if not phase:
        return None, "未收到行为树阶段（bt_state 无数据，运行器未启动？）"
    for prefix, action in STEP_PREFIXES:
        if phase.startswith(prefix):
            if action is None:
                return ("task_recover" if mission == "recover"
                        else "task_release"), ""
            return action, ""
    if phase.startswith("执行 "):
        return None, "__mode__"        # 需结合模式服务/单步状态（节点层处理）
    return None, "无法识别当前阶段: {}".format(phase)


# ---------------------------------------------------------------------------
# 纯逻辑状态核（无 ROS，供节点组合与离线自检）
# ---------------------------------------------------------------------------
class ManualState:
    """开关/装填/单步等待队列/日志/视图（线程安全，与发布解耦）。"""

    def __init__(self, mission="recover"):
        assert mission in ("recover", "release")
        self.mission = mission
        self._lock = threading.Lock()
        self._log = collections.deque(maxlen=8)
        self._view = None
        self._view_t = 0.0
        # 开关
        self.hold_on = False
        self.hold_since = 0.0
        self.mode_step = False
        self.gripper_step = False
        # 装填（一次性失败）
        self.mode_fail_next = False
        self.mode_fail_set = set()
        self.gripper_fail_armed = {"open": False, "clamp": False}
        # 单步等待（switch_mode 阻塞在节点层，这里管队列与放行）
        self.pending = collections.OrderedDict()   # pid -> entry
        self._pending_seq = 0
        self.gripper_pending = None                # entry or None
        # 行为树快照缓存（step_next 用）
        self.active_phase = ""
        self.root_status = ""

    # ---- 日志 ----
    def log_result(self, action, ok, msg):
        with self._lock:
            self._log.append({"t": time.time(), "action": action,
                              "ok": ok, "msg": msg})
            self._view = None              # 立即失效缓存，日志即时上屏
        return (ok, msg)

    # ---- phase 缓存 ----
    def update_phase(self, active_phase, root_status):
        with self._lock:
            self.active_phase = active_phase or ""
            self.root_status = root_status or ""

    # ---- 模式装填/消耗 ----
    def arm_mode_fail(self, mode=None):
        with self._lock:
            if mode is None:
                self.mode_fail_next = True
                return True, "已装填：下一次 switch_mode 返回失败（一次性）"
            self.mode_fail_set.add(mode)
            return True, "已装填：switch_mode({}) 返回失败（一次性）".format(mode)

    def _consume_mode_fail(self, mode):
        """命中即清除（next 优先级低于指定模式）。"""
        if mode in self.mode_fail_set:
            self.mode_fail_set.discard(mode)
            return True
        if self.mode_fail_next:
            self.mode_fail_next = False
            return True
        return False

    # ---- switch_mode 单步流（entry["event"] 由节点层等待） ----
    def switch_mode_enter(self, target_mode):
        """返回 ("fail"|"instant", msg) 或 ("pending", pid, entry)。"""
        with self._lock:
            if self._consume_mode_fail(target_mode):
                return ("fail", "手动注入: 模式 {} 失败（一次性装填）".format(
                    target_mode))
            if not self.mode_step:
                return ("instant", "手动模式服务: 即时成功 ({})".format(target_mode))
            self._pending_seq += 1
            pid = self._pending_seq
            entry = {"mode": target_mode, "ok": None, "msg": "",
                     "since": time.time(), "event": threading.Event()}
            self.pending[pid] = entry
            self._view = None
            return ("pending", pid, entry)

    def switch_mode_leave(self, pid, entry):
        """单步等待结束（放行/超时）后的收尾；超时补默认失败。"""
        with self._lock:
            self.pending.pop(pid, None)
            self._view = None
        if entry["ok"] is None:
            entry["ok"], entry["msg"] = False, "单步确认超时({:.0f}s)".format(
                MODE_STEP_TIMEOUT_S)

    def confirm_mode(self, ok):
        """放行最早的单步等待请求；返回 (handled, msg)。"""
        with self._lock:
            if not self.pending:
                return False, "当前无待确认模式"
            pid, entry = next(iter(self.pending.items()))
            del self.pending[pid]
            entry["ok"] = ok
            entry["msg"] = ("面板确认: {} 成功".format(entry["mode"]) if ok
                            else "面板确认: {} 失败（手动拒绝）".format(entry["mode"]))
            entry["event"].set()
            self._view = None
            return True, "已确认模式 {} → {}".format(
                entry["mode"], "SUCCESS" if ok else "FAILURE")

    def drain_pending(self, msg="服务已关闭"):
        """关闭模式服务时排掉全部阻塞请求（按失败放行）。"""
        with self._lock:
            entries = list(self.pending.values())
            self.pending.clear()
            for entry in entries:
                entry["ok"], entry["msg"] = False, msg
                entry["event"].set()
            self._view = None
            return len(entries)

    # ---- 夹爪装填/单步流 ----
    def arm_gripper_fail(self, action):
        with self._lock:
            self.gripper_fail_armed[action] = True
        return True, "已装填：下一次夹爪 {} 返回失败（一次性，需手动夹爪服务）".format(action)

    def gripper_enter(self, action):
        """返回 ("fail"|"instant", msg) 或 ("pending", entry)。"""
        with self._lock:
            if self.gripper_fail_armed.get(action):
                self.gripper_fail_armed[action] = False
                return ("fail", "手动注入: {} 失败（一次性装填）".format(action))
            if not self.gripper_step:
                return ("instant", "手动夹爪服务: {} 即时成功".format(action))
            if self.gripper_pending is not None:
                return ("fail", "已有待确认夹爪动作（{}），先确认再触发".format(
                    self.gripper_pending["action"]))
            entry = {"action": action, "ok": None, "msg": "",
                     "since": time.time(), "event": threading.Event()}
            self.gripper_pending = entry
            self._view = None
            return ("pending", entry)

    def gripper_leave(self, entry):
        with self._lock:
            if self.gripper_pending is entry:
                self.gripper_pending = None
            self._view = None
        if entry["ok"] is None:
            entry["ok"], entry["msg"] = False, "夹爪单步确认超时({:.0f}s)".format(
                GRIPPER_STEP_TIMEOUT_S)

    def confirm_gripper(self, ok):
        with self._lock:
            entry = self.gripper_pending
            if entry is None:
                return False, "当前无待确认夹爪动作"
            self.gripper_pending = None
            entry["ok"] = ok
            entry["msg"] = ("面板确认: {} 成功".format(entry["action"]) if ok
                            else "面板确认: {} 失败（手动拒绝）".format(entry["action"]))
            entry["event"].set()
            self._view = None
            return True, "已确认夹爪 {} → {}".format(
                entry["action"], "SUCCESS" if ok else "FAILURE")

    # ---- 面板状态视图（≤5s 刷新；内容不变保持对象身份以命中载荷缓存） ----
    def view(self, occupied_mode=False, occupied_gripper=False):
        now = time.time()
        with self._lock:
            if self._view is None or now - self._view_t >= 5.0:
                first = next(iter(self.pending.values()), None)
                pending = (None if first is None else
                           {"mode": first["mode"], "since": first["since"],
                            "count": len(self.pending)})
                gp = self.gripper_pending
                view = {"available": True,
                        "mission": self.mission,
                        "phase": self.active_phase,
                        "log": list(self._log),
                        "hold": {"on": self.hold_on, "since": self.hold_since},
                        "mode_service": {
                            "on": False,          # 托管状态由节点层回填
                            "occupied": occupied_mode,
                            "fail_armed": (self.mode_fail_next
                                           or bool(self.mode_fail_set)),
                            "fail_armed_modes": sorted(self.mode_fail_set),
                            "step": self.mode_step,
                            "pending": pending},
                        "gripper": {
                            "on": False,          # 托管状态由节点层回填
                            "occupied": occupied_gripper,
                            "step": self.gripper_step,
                            "pending": (None if gp is None else
                                        {"action": gp["action"],
                                         "since": gp["since"]}),
                            "fail_armed": dict(self.gripper_fail_armed)}}
                if view != self._view:
                    self._view = view
                self._view_t = now
            return self._view


def known_action(action):
    """动作 id 是否可路由（静态匹配，节点 inject 与自检共用）。"""
    return (action in ACTIONS
            or action.startswith("sensor_bad_")
            or action.startswith("remote_")
            or action.startswith("mode_fail_"))


# ---------------------------------------------------------------------------
# ROS 节点（发布/服务托管包装 ManualState）
# ---------------------------------------------------------------------------
class ManualNode:
    """按钮动作 → 发布标准话题/挂载服务（与实机源同帧格式）。

    发布器按需懒建；发布前等待订阅者建连（≤0.6s），未连上仍发出并提示。
    switch_mode 是阻塞式服务，无法用话题注入"模式完成"，故提供可选的
    即时成功/单步确认服务端（与 sim_feedback / 实机 mode_server 互斥）。
    """

    def __init__(self, mission="recover"):
        import rospy
        self.rospy = rospy
        self.core = ManualState(mission)
        self._pubs = {}
        self._hold_timer = None
        self._mode_svc = None
        self._gripper_svc = None
        self._occupied_cache = {"mode": (0.0, None), "gripper": (0.0, None)}

    # ---- 动作路由（sim_inject 服务入口） ----
    def inject(self, action):
        try:
            if action == "step_next":
                return self._step_next(action)
            if action == "task_recover":
                return self._lora(action, "RECOVER")
            if action == "task_release":
                return self._lora(action, "RELEASE")
            if action == "task_bogus":
                return self._lora(action, "BOGUS")
            if action == "deploy":
                return self._lora(action, "DEPLOY")
            if action == "hoist_done":
                return self._lora(action, "HOIST_DONE")
            if action == "home_cmd":
                return self._lora(action, "HOME")
            if action == "landed":
                return self._encoder_state(action, True, "手动注入:已落地")
            if action == "not_landed":
                return self._encoder_state(action, False, "手动注入:未落地")
            if action == "sensors_ok":
                return self._health_all(action)
            if action.startswith("sensor_bad_"):
                return self._sensor_bad(action, action[len("sensor_bad_"):])
            if action == "rtk_good":
                return self._fix_cov(action, 0.01, "良好")
            if action == "rtk_bad":
                return self._fix_cov(action, 9.0, "超限")
            if action == "hold_toggle":
                return self._hold_toggle(action)
            if action == "mode_service_toggle":
                return self._mode_service_toggle(action)
            if action == "mode_step_toggle":
                return self._mode_step_toggle(action)
            if action in ("mode_confirm_ok", "mode_confirm_fail"):
                ok, msg = self.core.confirm_mode(action == "mode_confirm_ok")
                return self.core.log_result(action, ok, msg)
            if action == "mode_fail_next":
                ok, msg = self.core.arm_mode_fail(None)
                return self.core.log_result(action, ok, msg)
            if action.startswith("mode_fail_"):
                mode = action[len("mode_fail_"):]
                if mode not in MODES:
                    return self.core.log_result(
                        action, False, "未知模式: {}".format(mode))
                ok, msg = self.core.arm_mode_fail(mode)
                return self.core.log_result(action, ok, msg)
            if action == "gripper_service_toggle":
                return self._gripper_service_toggle(action)
            if action == "gripper_step_toggle":
                return self._gripper_step_toggle(action)
            if action in ("gripper_confirm_ok", "gripper_confirm_fail"):
                ok, msg = self.core.confirm_gripper(
                    action == "gripper_confirm_ok")
                return self.core.log_result(action, ok, msg)
            if action == "gripper_fail_open":
                ok, msg = self.core.arm_gripper_fail("open")
                return self.core.log_result(action, ok, msg)
            if action == "gripper_fail_clamp":
                ok, msg = self.core.arm_gripper_fail("clamp")
                return self.core.log_result(action, ok, msg)
            if action.startswith("remote_"):
                return self._remote(action, action[len("remote_"):])
            return self.core.log_result(action, False,
                                        "未知动作: {}".format(action))
        except Exception as exc:  # noqa: BLE001
            return self.core.log_result(action, False,
                                        "注入失败: {}".format(exc))

    # ---- 智能放行：当前卡点一步通过 ----
    def _step_next(self, action):
        phase = self.core.active_phase
        act, reason = resolve_step_action(phase, self.core.mission)
        if act is not None:
            return self.inject(act)
        if reason == "__mode__":
            with self.core._lock:
                pending = bool(self.core.pending)
                hosted = self._mode_svc is not None
            if hosted and pending:
                return self.inject("mode_confirm_ok")
            if hosted:
                return self.core.log_result(
                    action, True,
                    "模式服务为即时成功模式，无需确认（逐步调试请开「模式单步确认」）")
            return self.core.log_result(
                action, False,
                "模式服务未开启：先点「手动模式服务」，逐步调试再加开「模式单步确认」")
        return self.core.log_result(action, False, reason)

    # ---- 发布基础设施 ----
    def _publish(self, topic, msg_type, msg):
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self.rospy.Publisher(topic, msg_type, queue_size=5)
            self._pubs[topic] = pub
        deadline = time.time() + 0.6
        while pub.get_num_connections() == 0 and time.time() < deadline:
            time.sleep(0.05)
        pub.publish(msg)
        return pub.get_num_connections() > 0

    @staticmethod
    def _sent_warn(connected):
        return "" if connected else "（无订阅者，运行器可能未启动）"

    # ---- 各动作实现 ----
    def _lora(self, action, op):
        from std_msgs.msg import String
        connected = self._publish("/lora/command", String,
                                  String(data="CMD,HEX,{},MANUAL".format(op)))
        return self.core.log_result(
            action, True, "已发布 /lora/command CMD,HEX,{}{}".format(
                op, self._sent_warn(connected)))

    def _encoder_state(self, action, landed, reason):
        from grasp_hexapod_msgs.msg import EncoderState
        m = EncoderState()
        m.landed = landed
        m.angle, m.reason = (135.0 if landed else 45.0), reason
        connected = self._publish("/grasp_hexapod/encoder_state", EncoderState, m)
        return self.core.log_result(
            action, True,
            "已发布 encoder_state landed={} {}{}".format(
                landed, reason, self._sent_warn(connected)))

    def _publish_health_all(self):
        """发布六路全健康帧 + 编码器帧（不记日志，供单次/持续复用）。

        落地位固定 False：传感器门与落地门解耦——落地与否只由
        「确认落地/未落地」按钮决定，保证"一次一小步"可独立控制。
        """
        from grasp_hexapod_msgs.msg import EncoderState, SensorHealth, SensorHealthArray
        arr = SensorHealthArray()
        for name in SENSOR_NAMES:
            h = SensorHealth()
            h.name, h.online, h.fresh = name, True, True
            h.freq_hz, h.age_s, h.reason = 50.0, 0.02, ""
            arr.sensors.append(h)
        enc = EncoderState()
        enc.landed = False
        enc.angle, enc.reason = 45.0, "手动注入:未落地（落地由确认按钮放行）"
        c1 = self._publish("/grasp_hexapod/sensor_health", SensorHealthArray, arr)
        c2 = self._publish("/grasp_hexapod/encoder_state", EncoderState, enc)
        return c1 and c2

    def _health_all(self, action):
        ok = self._publish_health_all()
        return self.core.log_result(
            action, True,
            "已发布六路全健康帧 + 编码器未落地帧" + self._sent_warn(ok))

    def _sensor_bad(self, action, name):
        """单传感器不新鲜帧（其余路保持健康），触发 IsSensorDataOk 停走保持。"""
        from grasp_hexapod_msgs.msg import SensorHealth, SensorHealthArray
        if name not in SENSOR_NAMES:
            return self.core.log_result(action, False,
                                        "未知传感器: {}".format(name))
        arr = SensorHealthArray()
        for sensor in SENSOR_NAMES:
            h = SensorHealth()
            h.name, h.online = sensor, True
            if sensor == name:
                h.fresh, h.freq_hz, h.age_s = False, 0.0, 9.99
                h.reason = "手动注入异常"
            else:
                h.fresh, h.freq_hz, h.age_s = True, 50.0, 0.02
                h.reason = ""
            arr.sensors.append(h)
        connected = self._publish("/grasp_hexapod/sensor_health",
                                  SensorHealthArray, arr)
        return self.core.log_result(
            action, True,
            "已注入 {} 不新鲜帧（停走保持；恢复点\"全健康帧\"）".format(name)
            + self._sent_warn(connected))

    def _publish_fix(self, cov=0.01):
        from sensor_msgs.msg import NavSatFix
        m = NavSatFix()
        m.status.status = 0
        m.position_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return self._publish("/fix", NavSatFix, m)

    def _fix_cov(self, action, cov, label):
        connected = self._publish_fix(cov)
        return self.core.log_result(
            action, True,
            "已发布 /fix 对角协方差 {} m²（{}）".format(cov, label)
            + self._sent_warn(connected))

    def _remote(self, action, mode):
        from grasp_hexapod_msgs.msg import RemoteCmd
        m = RemoteCmd()
        m.mode = mode
        connected = self._publish("/grasp_hexapod/remote_cmd", RemoteCmd, m)
        return self.core.log_result(
            action, True, "已发布 remote_cmd.mode={}{}".format(
                mode, self._sent_warn(connected)))

    # ---- 持续保持（5Hz 重发，对抗真实 monitor 覆盖） ----
    def _hold_toggle(self, action):
        with self.core._lock:
            self.core.hold_on = not self.core.hold_on
            on = self.core.hold_on
            if on:
                self.core.hold_since = time.time()
                self._hold_timer = self.rospy.Timer(
                    self.rospy.Duration(0.2), self._hold_tick)
                msg = "持续保持已开启（5Hz：健康帧+落地+/fix）"
            else:
                self.core.hold_since = 0.0
                if self._hold_timer is not None:
                    self._hold_timer.shutdown()
                    self._hold_timer = None
                msg = "持续保持已关闭"
            self.core._view = None
        return self.core.log_result(action, True, msg)

    def _hold_tick(self, _event):           # 不记日志
        try:
            self._publish_health_all()
            self._publish_fix()
        except Exception:  # noqa: BLE001
            pass

    # ---- 开关（模式单步 / 夹爪单步） ----
    def _mode_step_toggle(self, action):
        with self.core._lock:
            self.core.mode_step = not self.core.mode_step
            on = self.core.mode_step
            self.core._view = None
        return self.core.log_result(
            action, True,
            "模式单步确认已开启（每次 switch_mode 等面板确认；超时 {:.0f}s）".format(
                MODE_STEP_TIMEOUT_S) if on else "模式单步确认已关闭（恢复即时成功）")

    def _gripper_step_toggle(self, action):
        with self.core._lock:
            self.core.gripper_step = not self.core.gripper_step
            on = self.core.gripper_step
            self.core._view = None
        return self.core.log_result(
            action, True,
            "夹爪单步确认已开启（open/clamp 等面板确认；超时 {:.0f}s）".format(
                GRIPPER_STEP_TIMEOUT_S) if on else "夹爪单步确认已关闭（恢复即时成功）")

    # ---- 托管服务：switch_mode / gripper_act ----
    def on_switch_mode(self, req):
        """即时成功（默认）或单步阻塞等待面板确认（一步一点）。"""
        from grasp_hexapod_msgs.srv import SwitchModeResponse
        result = self.core.switch_mode_enter(req.target_mode)
        if result[0] == "fail":
            return SwitchModeResponse(success=False, message=result[1])
        if result[0] == "instant":
            return SwitchModeResponse(success=True, message=result[1])
        _, pid, entry = result
        entry["event"].wait(MODE_STEP_TIMEOUT_S)
        self.core.switch_mode_leave(pid, entry)
        return SwitchModeResponse(success=entry["ok"],
                                  message=entry["msg"] or (
                                      "面板确认: {}".format(req.target_mode)))

    def on_gripper_act(self, req):
        from grasp_hexapod_msgs.srv import GripperActResponse
        action = (req.action or "").lower()
        result = self.core.gripper_enter(action)
        if result[0] in ("fail", "instant"):
            return GripperActResponse(success=result[0] == "instant",
                                      message=result[1])
        entry = result[1]
        entry["event"].wait(GRIPPER_STEP_TIMEOUT_S)
        self.core.gripper_leave(entry)
        return GripperActResponse(success=entry["ok"], message=entry["msg"])

    def _mode_service_toggle(self, action):
        return self._service_toggle(
            action, "mode",
            SWITCH_MODE_SERVICE, "SwitchMode", self.on_switch_mode,
            on_msg="手动模式服务已开启：switch_mode 即时返回 success",
            off_msg="手动模式服务已关闭")

    def _gripper_service_toggle(self, action):
        return self._service_toggle(
            action, "gripper",
            GRIPPER_ACT_SERVICE, "GripperAct", self.on_gripper_act,
            on_msg="手动夹爪服务已开启：gripper_act open/clamp 待应答",
            off_msg="手动夹爪服务已关闭")

    def _service_toggle(self, action, key, service, srv_name, handler,
                        on_msg, off_msg):
        import grasp_hexapod_msgs.srv as srv_mod
        with self.core._lock:
            svc = getattr(self, "_" + key + "_svc")
            if svc is None:
                if self._probe_service(service, key):
                    ok, msg = False, ("{} 已被其他节点提供（sim_feedback/实机？），"
                                      "先停它再开手动服务".format(service))
                else:
                    setattr(self, "_" + key + "_svc", self.rospy.Service(
                        service, getattr(srv_mod, srv_name), handler))
                    ok, msg = True, on_msg
            else:
                svc.shutdown("sim_manual off")
                setattr(self, "_" + key + "_svc", None)
                ok, msg = True, off_msg
                if key == "mode":
                    drained = self.core.drain_pending("手动模式服务已关闭")
                    if drained:
                        msg += "（{} 个待确认请求已按失败放行）".format(drained)
                elif key == "gripper":
                    entry = None
                    with self.core._lock:
                        entry = self.core.gripper_pending
                        if entry is not None:
                            entry["ok"], entry["msg"] = False, "手动夹爪服务已关闭"
                            entry["event"].set()
                    if entry is not None:
                        msg += "（1 个待确认请求已按失败放行）"
            self.core._view = None
        return self.core.log_result(action, ok, msg)   # 出锁后再写日志（锁不可重入）

    def _probe_service(self, service, key):
        """探测服务是否已被其他节点提供（True/False/None=未知；5s 缓存）。

        key ∈ {"mode", "gripper"}：对应 _<key>_svc 托管句柄与占用缓存键。
        """
        if getattr(self, "_{}_svc".format(key), None) is not None:
            return False                     # 自己托管着 → 视为未占用
        now = time.time()
        cached_t, cached = self._occupied_cache.get(key, (0.0, None))
        if now - cached_t < 5.0:
            return cached
        try:
            import rosgraph.masterapi
            master = rosgraph.masterapi.Master("sim_manual_probe")
            occupied = any(name == service
                           for name, _ in master.getSystemState()[2])
        except Exception:  # noqa: BLE001
            occupied = None
        self._occupied_cache[key] = (now, occupied)
        return occupied

    # ---- 面板状态视图（回填托管状态 + 占用探测） ----
    def view(self):
        view = dict(self.core.view(
            occupied_mode=self._probe_service(SWITCH_MODE_SERVICE, "mode") or False,
            occupied_gripper=self._probe_service(GRIPPER_ACT_SERVICE, "gripper") or False))
        mode_svc = dict(view["mode_service"])
        mode_svc["on"] = self._mode_svc is not None
        gripper_svc = dict(view["gripper"])
        gripper_svc["on"] = self._gripper_svc is not None
        view["mode_service"], view["gripper"] = mode_svc, gripper_svc
        return view


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def run():
    import rospy
    from std_msgs.msg import String
    from grasp_hexapod_msgs.msg import BtStateArray
    from grasp_hexapod_msgs.srv import SimInject

    rospy.init_node("sim_manual")
    mission = rospy.get_param("~mission", "recover")
    node = ManualNode(mission)

    def on_inject(req):
        ok, msg = node.inject(req.action)
        rospy.loginfo("[sim_manual] %s %s → %s", "✓" if ok else "✗",
                      req.action, msg)
        from grasp_hexapod_msgs.srv import SimInjectResponse
        return SimInjectResponse(success=ok, message=msg)

    def on_bt_state(msg):
        node.core.update_phase(msg.active_phase, msg.root_status)

    rospy.Service(SIM_INJECT_SERVICE, SimInject, on_inject)
    rospy.Subscriber(BT_STATE_TOPIC, BtStateArray, on_bt_state, queue_size=10)

    pub_state = rospy.Publisher(SIM_STATE_TOPIC, String, queue_size=5)

    def pub_sim_state(_event=None):
        try:
            pub_state.publish(String(data=json.dumps(
                node.view(), ensure_ascii=False)))
        except Exception:  # noqa: BLE001
            pass

    rospy.Timer(rospy.Duration(0.5), pub_sim_state)   # 2Hz：内容未变载荷相同
    rospy.loginfo("sim_manual 就绪（mission=%s）：看板按钮 → %s；勿与 sim_feedback "
                  "同接口同时运行", mission, SIM_INJECT_SERVICE)
    rospy.spin()


def selftest():
    """离线：注册表一致性 / step_next 映射 / 状态机（装填·单步·超时）/ 视图。"""
    # 1. 注册表一致性：分组合法、id 唯一、提示引用存在、生成集合完整
    ids = list(ACTIONS)
    assert len(ids) == len(set(ids))
    for act in ACTIONS.values():
        assert act["group"] in GROUPS, act
    for hint in PHASE_HINTS:
        for aid in hint["actions"]:
            assert aid in ACTIONS, aid
    assert set(a for a in ACTIONS if a.startswith("sensor_bad_")) == set(
        "sensor_bad_" + n for n in SENSOR_NAMES)
    assert set(a for a in ACTIONS if a.startswith("remote_")) == set(
        "remote_" + m for m in REMOTE_MODES)
    assert set(a for a in ACTIONS
               if a.startswith("mode_fail_") and a != "mode_fail_next") == set(
        "mode_fail_" + m for m in MODES)
    print("[OK] ACTIONS/PHASE_HINTS 注册表一致（%d 个动作）" % len(ACTIONS))

    # 2. step_next 映射（纯函数）
    assert resolve_step_action("IsLandingConfirmed", "recover") == ("landed", "")
    assert resolve_step_action("WaitDeployment", "release") == ("deploy", "")
    assert resolve_step_action("WaitTaskCommand", "recover")[0] == "task_recover"
    assert resolve_step_action("WaitTaskCommand", "release")[0] == "task_release"
    assert resolve_step_action("WaitSensorsReady", "recover") == ("sensors_ok", "")
    assert resolve_step_action("WaitRtkPrecise", "recover") == ("rtk_good", "")
    assert resolve_step_action("WaitWinchHoisted", "recover") == ("hoist_done", "")
    assert resolve_step_action("执行 自转搜索小蓝 ㉖", "recover") == (None, "__mode__")
    assert resolve_step_action("", "recover")[0] is None
    assert resolve_step_action("CheckRemoteCmd_home", "recover")[0] is None
    for prefix, action in STEP_PREFIXES:   # 每个 step 前缀都能解析出动作
        act, reason = resolve_step_action(prefix, "recover")
        assert act is not None and act in ACTIONS, (prefix, reason)
    print("[OK] step_next 映射（%d 个前缀）" % len(STEP_PREFIXES))

    # 3. 状态机：装填（next/指定）→ 消耗；单步 阻塞→确认→放行；超时
    core = ManualState("recover")
    core.update_phase("IsLandingConfirmed", "RUNNING")
    core.arm_mode_fail("climb")
    assert core.switch_mode_enter("home")[0] == "instant"      # 未开单步
    core.mode_step = True
    assert core.switch_mode_enter("climb")[0] == "fail"        # 指定装填命中
    result = core.switch_mode_enter("home")                    # 进入单步等待
    assert result[0] == "pending"
    view = core.view()
    assert view["mode_service"]["pending"]["mode"] == "home"
    assert view["mode_service"]["step"] is True
    handled, msg = core.confirm_mode(True)
    assert handled and "home" in msg
    assert core.view()["mode_service"]["pending"] is None

    core.arm_mode_fail(None)                                   # 下一次任意
    result = core.switch_mode_enter("dock")
    assert result[0] == "fail" and "dock" in result[1]

    # 超时路径（缩短超时）
    global MODE_STEP_TIMEOUT_S
    _bak = MODE_STEP_TIMEOUT_S
    MODE_STEP_TIMEOUT_S = 0.2
    result = core.switch_mode_enter("spin_search")
    assert result[0] == "pending"
    _, pid, entry = result
    entry["event"].wait(MODE_STEP_TIMEOUT_S)     # 模拟节点层等待
    core.switch_mode_leave(pid, entry)
    assert entry["ok"] is False and "超时" in entry["msg"]
    MODE_STEP_TIMEOUT_S = _bak

    # 空确认 / 排空
    assert core.confirm_mode(True)[0] is False
    core.switch_mode_enter("tag_nav")
    assert core.drain_pending() == 1
    print("[OK] 模式装填/单步/超时/排空状态机")

    # 4. 夹爪状态机：装填→命中；单步 阻塞→确认
    core2 = ManualState("release")
    core2.arm_gripper_fail("clamp")
    assert core2.gripper_enter("clamp")[0] == "fail"
    assert core2.gripper_enter("open")[0] == "instant"
    core2.gripper_step = True
    result = core2.gripper_enter("open")
    assert result[0] == "pending"
    entry = result[1]
    assert core2.view()["gripper"]["pending"]["action"] == "open"
    handled, msg = core2.confirm_gripper(False)
    assert handled and "FAILURE" in msg and "open" in msg
    entry["event"].wait(0.1)
    assert entry["ok"] is False
    assert core2.view()["gripper"]["pending"] is None
    assert core2.confirm_gripper(True)[0] is False
    print("[OK] 夹爪装填/单步状态机")

    # 5. 视图缓存：内容未变保持对象身份
    v1 = core2.view()
    assert core2.view() is v1
    core2.log_result("test", True, "x")
    assert core2.view() is not v1
    print("[OK] 视图缓存（未变零重建）")

    print("selftest 全部通过")


def main():
    parser = argparse.ArgumentParser(
        description="手动单步模拟节点（实物缺失时的手动模拟源）")
    parser.add_argument("--selftest", action="store_true",
                        help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        selftest()
        return
    run()


if __name__ == "__main__":
    main()
