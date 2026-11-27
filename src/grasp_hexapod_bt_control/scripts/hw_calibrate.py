#!/usr/bin/env python3
"""实机 approach 标定向导（终端交互）。

用法（full_mission.launch 已在运行时）：
    rosrun grasp_hexapod_bt_control hw_calibrate.py               # 全部步骤
    rosrun grasp_hexapod_bt_control hw_calibrate.py --step teach  # 只做某一步
    rosrun grasp_hexapod_bt_control hw_calibrate.py --from topics # 从某一步往后

步骤：tag 配置 -> 话题检查 -> YOLO 检查 -> IMU 方向 -> P0 示教 -> 偏航-距离检查。
每一步都可以跳过（输入 s）或退出（输入 q）。写配置前自动备份原文件。
写入的参数在控制节点启动时读取：标定完必须重启 full_mission.launch。

说明见 docs/HW_CALIBRATION.md。只读传感器话题，不发任何运动指令。
"""
import argparse
import math
import re
import shutil
import sys
import time
from pathlib import Path

import rospy



def package_dir():
    """与 launch 的 $(find grasp_hexapod_bt_control) 相同的包目录。"""
    try:
        import rospkg
        return Path(rospkg.RosPack().get_path("grasp_hexapod_bt_control"))
    except Exception:  # noqa: BLE001  未 source 工作空间时退回源码位置
        return Path(__file__).resolve().parents[1]


PKG_DIR = package_dir()
DEFAULT_YAML = PKG_DIR / "config" / "phase1_visual.yaml"
DEFAULT_LAUNCH = PKG_DIR / "launch" / "full_mission.launch"
FRAME = "camera_color_optical_frame"
TAG_TOPIC = "/xiaolan/tag_detections"
TARGET_TOPIC = "/xiaolan/target"
STATUS_TOPIC = "/xiaolan/perception_status"
IMU_TOPIC = "/grasp_hexapod/imu"
FEEDBACK_TOPIC = "/lb_pos"
MAX_X_AXIS_OUT_OF_PLANE = 0.5  # 与控制器 kMaxTagXAxisOutOfPlane 相同


class Quit(Exception):
    pass


class Restart(Exception):
    """配置已改、需要重启 launch：结束本次运行。"""


def say(text=""):
    print(text, flush=True)


def ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    try:
        answer = input(f"{prompt}{suffix}: ").strip()
    except EOFError:
        raise Quit()
    if answer.lower() == "q":
        raise Quit()
    return answer if answer else ("" if default is None else str(default))


def ask_number(prompt, default, cast, low, high):
    """反复询问直到得到 [low, high] 内的数字。"""
    while True:
        text = ask(prompt, default)
        try:
            value = cast(text)
        except ValueError:
            say(f"  请输入数字（{low}–{high}）")
            continue
        if low <= value <= high:
            return value
        say(f"  超出范围（{low}–{high}）")


def wait_enter(prompt):
    """回车继续；s 跳过本步（返回 False）；q 退出。"""
    answer = ask(prompt + "（回车继续，s 跳过，q 退出）")
    return answer.lower() != "s"


def yes(prompt, default="y"):
    return ask(prompt + " (y/n)", default).lower().startswith("y")


# ---------------------------------------------------------------- 配置文件读写

def read_yaml_value(path, key):
    match = re.search(rf"^{re.escape(key)}:[ \t]*([^#\n]+)", path.read_text(), re.M)
    return match.group(1).strip() if match else None


def write_yaml_values(path, values, backup_done):
    text = path.read_text()
    for key, value in values.items():
        pattern = rf"^({re.escape(key)}:[ \t]*)([^#\n]*?)([ \t]*(#.*)?)$"
        if not re.search(pattern, text, re.M):
            raise RuntimeError(f"{path} 中找不到 {key}")
        text = re.sub(pattern, lambda m: f"{m.group(1)}{value}{m.group(3)}", text, count=1, flags=re.M)
    backup(path, backup_done)
    path.write_text(text)


def read_launch_default(path, arg):
    match = re.search(rf'<arg name="{re.escape(arg)}" default="([^"]*)"', path.read_text())
    return match.group(1) if match else None


def write_launch_default(path, arg, value, backup_done):
    text = path.read_text()
    new, count = re.subn(rf'(<arg name="{re.escape(arg)}" default=")[^"]*(")',
                         lambda m: f"{m.group(1)}{value}{m.group(2)}", text, count=1)
    if count != 1:
        raise RuntimeError(f"{path} 中找不到 arg {arg}")
    backup(path, backup_done)
    path.write_text(new)


def backup(path, backup_done):
    if path in backup_done:
        return
    target = path.with_name(path.name + time.strftime(".bak_%Y%m%d_%H%M%S"))
    shutil.copy2(path, target)
    backup_done.add(path)
    say(f"  已备份 {path.name} -> {target.name}")


# ---------------------------------------------------------------- 话题采样

def tag_yaw(q):
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(n) or n < 1e-6:
        return None
    x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + z * w)
    r20 = 2.0 * (x * z - y * w)
    if abs(r10) > MAX_X_AXIS_OUT_OF_PLANE:
        return None
    return math.atan2(r20, r00)


def collect_tags(tag_id, count, timeout_s):
    """收集 count 个新鲜、有效的 (x, z, yaw)；返回 (样本, 其他 ID, 坐标系是否正确)。"""
    from apriltag_ros.msg import AprilTagDetectionArray
    samples, other_ids, last, frames = [], set(), [0.0], set()

    def callback(msg):
        for det in msg.detections:
            if len(det.id) != 1:
                continue
            if det.id[0] != tag_id:
                other_ids.add(det.id[0])
                continue
            # 与控制器 mode_approach.cpp 相同：检测自带时间戳时用它自己的 header
            #（含坐标系），否则用数组的 header。
            header = det.pose.header if not det.pose.header.stamp.is_zero() else msg.header
            stamp = header.stamp.to_sec()
            if stamp <= last[0]:
                continue
            frames.add(header.frame_id)
            p, yaw = det.pose.pose.pose.position, tag_yaw(det.pose.pose.pose.orientation)
            if yaw is None or not p.z > 0.0:
                continue
            last[0] = stamp
            samples.append((p.x, p.z, yaw))

    sub = rospy.Subscriber(TAG_TOPIC, AprilTagDetectionArray, callback, queue_size=10)
    deadline = time.time() + timeout_s
    while len(samples) < count and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)
    sub.unregister()
    bad = frames - {FRAME}
    if bad:
        say(f"  ✗ tag 坐标系是 {sorted(bad)}，控制器只接受 {FRAME}，approach 会拒收所有 tag。"
            "检查 full_mission.launch 里检测器订阅的图像话题。")
        return [], other_ids, False
    return samples[:count], other_ids, True


def stats(samples):
    n = len(samples)
    mx = sum(s[0] for s in samples) / n
    mz = sum(s[1] for s in samples) / n
    myaw = math.atan2(sum(math.sin(s[2]) for s in samples) / n, sum(math.cos(s[2]) for s in samples) / n)

    def std(values):
        return math.sqrt(sum(v * v for v in values) / (len(values) - 1)) if len(values) > 1 else 0.0

    return (mx, mz, myaw, std([s[0] - mx for s in samples]), std([s[1] - mz for s in samples]),
            std([math.atan2(math.sin(s[2] - myaw), math.cos(s[2] - myaw)) for s in samples]))


def topic_rate(topic, msg_type, seconds=2.0):
    count = [0]
    sub = rospy.Subscriber(topic, msg_type, lambda _m: count.__setitem__(0, count[0] + 1), queue_size=50)
    time.sleep(seconds)
    sub.unregister()
    return count[0] / seconds


def imu_yaw(timeout_s=2.0):
    from sensor_msgs.msg import Imu
    try:
        msg = rospy.wait_for_message(IMU_TOPIC, Imu, timeout=timeout_s)
    except rospy.ROSException:
        return None
    q = msg.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------- 各步骤

def stance_error():
    """当前关节反馈与行走站姿 Q_STAND 的最大偏差（rad）；读不到返回 None。"""
    from sensor_msgs.msg import JointState
    try:
        import rospkg
        sys.path.insert(0, str(Path(rospkg.RosPack().get_path("grasp_hexapod_control")) / "scripts"))
        from kinematics import Q_STAND
    except Exception:  # noqa: BLE001
        return None
    worst = 0.0
    for index, leg in enumerate(("lb", "lf", "lm", "rb", "rf", "rm")):
        try:
            msg = rospy.wait_for_message(f"/{leg}_pos", JointState, timeout=1.0)
        except rospy.ROSException:
            return None
        if len(msg.position) < 3:
            return None
        worst = max(worst, max(abs(msg.position[j] - Q_STAND[index][j]) for j in range(3)))
    return worst


def step_tag_config(ctx):
    say("\n== 1/6 tag 配置 ==")
    yaml_id = read_yaml_value(ctx.yaml, "tag_guidance_id")
    launch_id = read_launch_default(ctx.launch, "tag_id")
    launch_size = read_launch_default(ctx.launch, "tag_size_m")
    say(f"  当前：yaml tag_guidance_id={yaml_id}，launch tag_id={launch_id}，tag_size_m={launch_size}")
    tag_id = ask_number("  小蓝正面用于接近的 tag 编号（tag36h11）", yaml_id, int, 0, 586)
    if launch_id is None or launch_size is None:
        say(f"  ✗ {ctx.launch} 里找不到 tag_id / tag_size_m 参数，无法自动设置。")
        raise Quit()
    size = ask_number("  tag 黑框外边长（米，例如 0.05）", launch_size, float, 0.01, 0.30)
    ctx.tag_id = tag_id
    changed = False
    if str(tag_id) != str(yaml_id):
        write_yaml_values(ctx.yaml, {"tag_guidance_id": tag_id}, ctx.backups)
        changed = True
    if str(tag_id) != str(launch_id):
        write_launch_default(ctx.launch, "tag_id", tag_id, ctx.backups)
        changed = True
    if abs(size - float(launch_size)) > 1e-9:
        write_launch_default(ctx.launch, "tag_size_m", f"{size:.4f}", ctx.backups)
        changed = True
    if changed:
        say("  已写入。检测器只在启动时加载 tag 编号和边长，需要重启 full_mission.launch。")
        say("  roslaunch 自带的 ROS master 会随 launch 一起重启，本向导必须先退出：")
        say("    1) 终端 A：Ctrl-C，再重新 roslaunch ... full_mission.launch show_windows:=false")
        say("    2) 终端 B：rosrun grasp_hexapod_bt_control hw_calibrate.py --from topics")
        raise Restart()
    else:
        say("  配置一致，无需改动。")


def step_topics(ctx):
    say("\n== 2/6 话题检查（机器人可以不动）==")
    from apriltag_ros.msg import AprilTagDetectionArray
    from sensor_msgs.msg import Imu, JointState
    from xiaolan_yolo_detection.msg import PerceptionStatus, XiaolanTarget
    checks = [(STATUS_TOPIC, PerceptionStatus, "YOLO 健康状态", True),
              (TARGET_TOPIC, XiaolanTarget, "YOLO 目标（看到小蓝才有）", False),
              (TAG_TOPIC, AprilTagDetectionArray, "AprilTag 检测数组", True),
              (IMU_TOPIC, Imu, "IMU（只有盲走收尾需要）", False),
              (FEEDBACK_TOPIC, JointState, "舵机关节反馈", True)]
    ok = True
    for topic, msg_type, label, required in checks:
        hz = topic_rate(topic, msg_type)
        mark = "OK " if hz > 1.0 else ("缺失" if required else "无  ")
        say(f"  [{mark}] {topic:32s} {hz:5.1f} Hz  {label}")
        ok = ok and (hz > 1.0 or not required)
        if topic == IMU_TOPIC:
            ctx.has_imu = hz > 1.0
    if not ok:
        say("  有必需话题缺失：确认 full_mission.launch 正常运行（相机、YOLO、tag 检测器、舵机节点）。")
        if not yes("  仍然继续？", "n"):
            raise Quit()


def step_yolo(ctx):
    say("\n== 3/6 YOLO 检查 ==")
    from xiaolan_yolo_detection.msg import XiaolanTarget
    for distance in ("1.0", "2.0"):
        if not wait_enter(f"  把机器人放在小蓝正前方约 {distance} m，相机对着小蓝"):
            continue
        got, no_depth = [], [0]

        def on_target(m):
            if not m.detected:
                return
            if math.isfinite(m.z_target) and m.z_target > 0.0:
                got.append((m.x_target, m.z_target))
            else:
                no_depth[0] += 1

        sub = rospy.Subscriber(TARGET_TOPIC, XiaolanTarget, on_target, queue_size=50)
        time.sleep(3.0)
        sub.unregister()
        if not got:
            if no_depth[0]:
                say(f"  ✗ {distance} m：检出了小蓝但没有有效深度（{no_depth[0]} 帧）。approach 会因 "
                    "TARGET_PENDING 失败：检查深度相机、距离是否超出 max_depth（xiaolan_yolo.yaml）。")
            else:
                say(f"  ✗ {distance} m：3 秒内没有检出小蓝。approach 无法从这个距离开始。")
                say("    检查：画面里小蓝是否完整、光线、模型是否加载（看 xiaolan_yolo_node 日志）。")
            continue
        xs = [g[0] for g in got]
        zs = [g[1] for g in got]
        say(f"  ✓ {distance} m：{len(got) / 3.0:.1f} Hz 检出，深度 z={sum(zs) / len(zs):.2f} m，"
            f"横向 x={sum(xs) / len(xs):+.3f} m")


def step_imu(ctx):
    say("\n== 4/6 IMU 方向 ==")
    if ctx.has_imu is False or imu_yaw() is None:
        say("  没有 IMU 数据：跳过。tag 一直看得见时 approach 不需要 IMU；")
        say("  tag 中途丢失时会以 'imu unavailable for blind finish' 安全停下。")
        return
    if not wait_enter("  机器人放平静止"):
        return
    y0 = imu_yaw()
    if not wait_enter("  从上往下看，用手把机器人逆时针转约 30 度，放稳"):
        return
    y1 = imu_yaw()
    delta = math.degrees(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))
    say(f"  IMU 偏航变化 {delta:+.1f} 度")
    if abs(delta) < 10.0:
        say("  变化太小，无法判断，跳过（转得不够或 IMU 无输出）。")
        return
    if not yes("  确认刚才是从上往下看【逆时针】转的？"):
        say("  方向不确定：不写入，请重做这一步（--step imu）。")
        return
    sign = 1.0 if delta > 0 else -1.0
    current = read_yaml_value(ctx.yaml, "tag_imu_yaw_sign")
    say(f"  tag_imu_yaw_sign 应为 {sign:+.0f}（当前 {current}）")
    if float(current) != sign:
        write_yaml_values(ctx.yaml, {"tag_imu_yaw_sign": f"{sign:.1f}"}, ctx.backups)
        ctx.changed_params = True
        say("  已写入。")


def step_teach(ctx):
    say("\n== 5/6 P0 示教 ==")
    if ctx.tag_id is None:
        ctx.tag_id = int(read_yaml_value(ctx.yaml, "tag_guidance_id"))
    say("  先在看板点 home，让机器人站成行走站姿；然后把机器人摆到攀爬起点 P0：")
    say("    - 正对小蓝正面，机身侧边与立面垂直（不要歪）；")
    say("    - 机身中心线对准小蓝正面中心（攀爬配置里偏右约 5 mm，可视为居中）；")
    say("    - 两只前脚脚尖离小蓝立面约 2 cm（机身中心离立面约 19.5 cm）；")
    say("    - 机身静止、六条腿都着地。")
    while True:
        if not wait_enter("  摆好了"):
            return
        error = stance_error()
        if error is None:
            say("  ⚠ 读不到六条腿的关节反馈，无法确认站姿。")
        elif error > 0.10:
            say(f"  ⚠ 当前关节离行走站姿差 {error:.2f} rad：请先在看板点 home，站好后再摆到 P0。")
            if yes("  重新摆放？"):
                continue
        else:
            say(f"  站姿正确（离行走站姿 {error:.3f} rad）。")
        say(f"  采集 tag {ctx.tag_id} 的 30 帧……")
        samples, others, frame_ok = collect_tags(ctx.tag_id, 30, 15.0)
        if not frame_ok:
            return
        if len(samples) < 30:
            say(f"  ✗ 只收到 {len(samples)} 帧 tag {ctx.tag_id}。" +
                (f" 画面里看到的是其他编号 {sorted(others)}，回到第 1 步改编号。" if others else
                 " 确认 tag 在画面内、检测器在运行（话题检查）。"))
            if not yes("  重试？"):
                return
            continue
        mx, mz, myaw, sx, sz, syaw = stats(samples)
        say(f"  结果：x={mx:+.4f} m  z={mz:.4f} m  yaw={math.degrees(myaw):+.2f} 度")
        say(f"  抖动：x {sx * 1000:.1f} mm  z {sz * 1000:.1f} mm  yaw {math.degrees(syaw):.2f} 度")
        if sx > 0.003 or sz > 0.003 or math.degrees(syaw) > 0.5:
            say("  ⚠ 抖动偏大（期望 x/z ≤3 mm，yaw ≤0.5 度）：机器人是否静止、光线是否稳定？")
            if yes("  重新采集？"):
                continue
        if not (0.08 <= mz <= 0.35):
            say(f"  ⚠ 深度 {mz:.3f} m 不像 P0（期望约 0.15-0.20 m），确认位置。")
            if yes("  重新采集？"):
                continue
        if yes("  写入 phase1_visual.yaml？"):
            write_yaml_values(ctx.yaml, {"tag_ref_x_m": f"{mx:.4f}", "tag_p0_z_m": f"{mz:.4f}",
                                         "tag_ref_yaw_rad": f"{myaw:.4f}"}, ctx.backups)
            ctx.changed_params = True
            ctx.taught_yaw = myaw
            say("  已写入。")
        return


def step_yaw_check(ctx):
    say("\n== 6/6 偏航-距离检查（可选，检查 tag 打印/尺寸/标定是否有问题）==")
    if ctx.tag_id is None:
        ctx.tag_id = int(read_yaml_value(ctx.yaml, "tag_guidance_id"))
    yaws = []
    for distance in ("0.30", "0.45"):
        if not wait_enter(f"  沿直线后退到相机离 tag 约 {distance} m，不要转身"):
            continue
        samples, _, frame_ok = collect_tags(ctx.tag_id, 15, 10.0)
        if not frame_ok:
            return
        if len(samples) < 10:
            say(f"  ✗ {distance} m 收不到足够的 tag 帧（{len(samples)}）")
            continue
        _, mz, myaw, *_ = stats(samples)
        say(f"  {distance} m：z={mz:.3f} m  yaw={math.degrees(myaw):+.2f} 度")
        if abs(mz - float(distance)) > 0.08:
            say(f"  ⚠ 实测深度 {mz:.2f} m 与要求的 {distance} m 差太多，这一点不计入（是否没有后退？）")
            continue
        yaws.append(myaw)
    if ctx.taught_yaw is not None:
        yaws.append(ctx.taught_yaw)
    if len(yaws) >= 2:
        diffs = [math.atan2(math.sin(y - yaws[0]), math.cos(y - yaws[0])) for y in yaws]
        spread = math.degrees(max(diffs) - min(diffs))
        verdict = "OK" if spread <= 1.0 else "偏大：可能 tag 打印变形、边长不对或相机标定误差，先报告再上机"
        say(f"  偏航随距离的变化 {spread:.2f} 度（≤1 度为正常）：{verdict}")


STEPS = {"tag": step_tag_config, "topics": step_topics, "yolo": step_yolo,
         "imu": step_imu, "teach": step_teach, "yawcheck": step_yaw_check}


class Context:
    def __init__(self, yaml_path, launch_path):
        self.yaml, self.launch = yaml_path, launch_path
        self.backups = set()
        self.tag_id = None
        self.has_imu = None
        self.taught_yaw = None
        self.changed_params = False


def main():
    parser = argparse.ArgumentParser(description="实机 approach 标定向导")
    parser.add_argument("--step", choices=sorted(STEPS), help="只运行某一步")
    parser.add_argument("--from", dest="start", choices=sorted(STEPS), help="从某一步开始往后运行")
    parser.add_argument("--yaml", type=Path, default=DEFAULT_YAML)
    parser.add_argument("--launch", type=Path, default=DEFAULT_LAUNCH)
    args = parser.parse_args(rospy.myargv()[1:])
    ctx = Context(args.yaml, args.launch)
    say("实机 approach 标定向导：只读传感器，不会让机器人运动。任何提示下输入 q 退出。")
    say(f"配置文件：{ctx.yaml}\nlaunch：  {ctx.launch}")
    rospy.init_node("hw_calibrate", anonymous=True, disable_signals=True)
    order = list(STEPS)
    if args.step:
        steps = [args.step]
    elif args.start:
        steps = order[order.index(args.start):]
    else:
        steps = order
    try:
        for name in steps:
            STEPS[name](ctx)
    except Restart:
        say("\n本次运行结束（等待重启 launch）。")
        return 0
    except (Quit, KeyboardInterrupt):
        say("\n已退出。")
    if ctx.backups:
        say("\n本次改动的文件：" + "，".join(p.name for p in ctx.backups) + "（原文件已备份为 .bak_时间戳）")
        say("当前标定值（请保存或发给开发同学入库）：")
        for key in ("tag_guidance_id", "tag_ref_x_m", "tag_p0_z_m", "tag_ref_yaw_rad", "tag_imu_yaw_sign"):
            say(f"  {key}: {read_yaml_value(ctx.yaml, key)}")
        say(f"  launch tag_id: {read_launch_default(ctx.launch, 'tag_id')}  "
            f"tag_size_m: {read_launch_default(ctx.launch, 'tag_size_m')}")
        say("注意：这些文件受 git 管理。以后 git pull 前先 git stash，pull 完再 git stash pop，"
            "否则 pull 会因本地改动失败。")
    if ctx.changed_params or ctx.backups:
        say("下一步：Ctrl-C 停掉 full_mission.launch 并重新启动，让控制节点读取新参数。")
        say("然后把机器人放到 1-2 m 外，在看板上依次点 home -> spin_search -> approach（不要点 climb）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
