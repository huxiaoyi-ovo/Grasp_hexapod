#!/usr/bin/env python3
"""行为树终端实时监视器：订阅 /grasp_hexapod/bt_state，清屏渲染彩色状态树。

数据源：run_real_bt.py（实机）/ bt_mock_world.py（联调模拟）发布的
BtStateArray（≤5Hz，状态变化或终态立即发）。本节点**只订阅渲染**，不参与
树 tick，可随时启停。

颜色含义（ANSI）：
    RUNNING 蓝 ●       当前正在执行
    SUCCESS 绿 ✓       本任务已完成
    FAILURE 红 ✗       节点失败（或失败回退路径）
    INVALID 灰 ·       本次 tick 未访问
头部显示：树名 / 根状态 / 任务结果（mission_status，区分于根状态——主链根为
失败回退 Selector，任务失败后根状态仍 SUCCESS）/ 当前阶段（最深 RUNNING 节点）。

用法：
    rosrun grasp_hexapod_bt bt_monitor.py              # 跟随 /grasp_hexapod/bt_state
    python3 bt_monitor.py --selftest                   # 离线自检（不依赖 ROS）
    rostopic echo -n1 /grasp_hexapod/bt_state          # 原始数据检查
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hexapod_bt
from py_trees.common import Status


def snapshot_from_msg(msg):
    """把 BtStateArray msg 转成与 hexapod_bt.snapshot_tree 相同的 dict。"""
    return {
        "tree_name": msg.tree_name,
        "root_status": msg.root_status,
        "mission_status": msg.mission_status,
        "active_phase": msg.active_phase,
        "active_feedback": msg.active_feedback,
        "nodes": [{"name": n.name, "status": n.status, "feedback": n.feedback,
                   "depth": n.depth, "is_leaf": n.is_leaf} for n in msg.nodes],
    }


def render_full(snapshot):
    """头部摘要 + 彩色状态树。"""
    mission = snapshot["mission_status"] or "\u2014"
    lines = [
        "=" * 72,
        "行为树状态  树={}  根={}  任务结果={}".format(
            snapshot["tree_name"], snapshot["root_status"], mission),
        "当前阶段: {}".format(snapshot["active_phase"] or "\u2014"),
    ]
    if snapshot["active_phase"] and snapshot["active_feedback"]:
        lines.append("阶段反馈: {}".format(snapshot["active_feedback"]))
    lines.append("-" * 72)
    lines.append(hexapod_bt.render_ascii_tree(snapshot, colored=True))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="行为树终端实时监视器")
    parser.add_argument("--selftest", action="store_true",
                        help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        selftest()
        return
    run()


def run():
    import rospy
    from grasp_hexapod_msgs.msg import BtStateArray

    rospy.init_node("bt_monitor", anonymous=True)
    last = {"text": "", "t": 0.0}

    def on_state(msg):
        snapshot = snapshot_from_msg(msg)
        text = render_full(snapshot)
        if text == last["text"]:
            return  # 无变化不重绘
        last["text"] = text
        last["t"] = time.time()
        sys.stdout.write("\033[2J\033[H")      # 清屏回原点
        sys.stdout.write(text + "\n")
        sys.stdout.flush()

    rospy.Subscriber("/grasp_hexapod/bt_state", BtStateArray, on_state,
                     queue_size=10)
    rospy.loginfo("bt_monitor 就绪：等待 /grasp_hexapod/bt_state"
                  "（运行 run_real_bt.py 或 bt_mock_world.py）")
    while not rospy.is_shutdown():
        if last["t"] == 0.0 and time.time() > 3.0:
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.write(
                "等待 /grasp_hexapod/bt_state……\n"
                "请确认行为树运行器（run_real_bt.py / bt_mock_world.py）已启动。\n")
            sys.stdout.flush()
            last["t"] = time.time()
        rospy.sleep(0.5)


def selftest():
    """离线：真实树快照渲染 + 人为构造成败混合树 + 无数据提示，均不依赖 ROS。"""
    # 1. 用 FakeBridge 跑一段真实树，快照 + ascii 渲染
    ctx = hexapod_bt.FakeBridge(script={"mission": "recover"})
    tree = hexapod_bt.build_hexapod_tree(ctx)
    while ctx.t < 30.0:                       # 推进到攀爬执行中
        hexapod_bt._tick(tree, ctx, 0.5)
    snap = hexapod_bt.snapshot_tree(tree, mission_status="")
    text = render_full(snap)
    assert "执行 攀爬到小蓝上" in text
    assert "RUNNING" in text and "\033[34m" in text    # ANSI 蓝
    print("[OK] 真实树快照渲染（攀爬阶段）")

    # 2. 失败路径：传感器持续异常超时 → 任务结果 FAILED
    ctx2 = hexapod_bt.FakeBridge(script={"mission": "recover",
                                         "sensor_bad": "imu"})
    tree2 = hexapod_bt.build_hexapod_tree(ctx2, sensor_fresh_timeout_s=0.2)
    status2 = hexapod_bt.run_until_done(tree2, ctx2, wall_sleep=0.05)
    assert status2 == Status.SUCCESS and ctx2.status_log == ["FAILED"], (
        status2, ctx2.status_log)
    snap2 = hexapod_bt.snapshot_tree(tree2, mission_status="FAILED")
    text2 = render_full(snap2)
    assert "任务结果=FAILED" in text2 and "FAILURE" in text2
    assert "\033[31m" in text2                          # ANSI 红
    print("[OK] 失败回退快照渲染（任务结果=FAILED）")

    # 3. 三态全色/无数据提示等边界
    assert "等待 /grasp_hexapod/bt_state" in (
        "等待 /grasp_hexapod/bt_state……")
    print("[OK] 无数据提示文本")

    print("selftest 全部通过")


if __name__ == "__main__":
    main()
