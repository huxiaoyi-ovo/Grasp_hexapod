#!/usr/bin/env python3
"""Per-process resource profile for the xiaolan_yolo detection chain.

Diagnostics only. Launch xiaolan_orbbec_camera.launch and xiaolan_yolo.launch
beforehand, then run this against the same master. It subscribes ONLY the small
/xiaolan/* topics (never the camera raw streams, so the probe itself does not
add connections to the camera driver) and samples /proc for CPU%, RSS, threads
and mapped library origins. Writes one JSON plus a raw tegrastats log.
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import threading
import time
import xmlrpc.client

import rospy

NODE_LABELS = (
    ('/camera/camera', 'camera'),
    ('/xiaolan_yolo_node', 'yolo'),
    ('/xiaolan_target_3d_node', 'td3'),
    ('/xiaolan_visualizer', 'visualizer'),
    # Rest of full_mission.launch (missing ones are reported and skipped), so
    # one run gives the whole on-robot load.
    ('/xiaolan_apriltag_detector', 'apriltag'),
    ('/camera/color/tag_image_proc_manager', 'image_proc'),
    ('/grasp_hexapod_bt_control', 'bt_control'),
    ('/servo_left_node', 'servo_left'),
    ('/servo_right_node', 'servo_right'),
    ('/dock_camera/usb_cam', 'dock_camera'),
    ('/bt_dashboard', 'dashboard'),
    ('/grasp_hexapod_remote_control', 'remote'),
)
MAP_KEYS = ('site-packages/numpy', 'dist-packages/numpy', 'cv2/',
            'libopencv_core.so', 'libnvinfer.so', 'libcudnn', 'torch')


def resolve_pids():
    import rosgraph
    import rosnode
    want = dict(NODE_LABELS)
    out = {}
    try:
        master = rosgraph.Master('resource_profile')
        nodes = sorted(rosnode.get_node_names())
    except Exception as error:
        print('ERROR: cannot reach ROS master: %s' % error, file=sys.stderr)
        return out, []
    for name in nodes:
        label = want.get(name)
        if label is None or label in out:
            continue
        try:
            uri = master.lookupNode(name)
            code, _msg, pid = xmlrpc.client.ServerProxy(uri, allow_none=True).getPid(name)
            if code == 1 and pid:
                out[label] = int(pid)
        except Exception:
            pass
    return out, nodes


def read_proc(pid):
    data = open('/proc/%d/stat' % pid).read()
    rest = data[data.rindex(')') + 2:].split()
    ticks = int(rest[11]) + int(rest[12])
    rss = threads = None
    for line in open('/proc/%d/status' % pid):
        if line.startswith('VmRSS:'):
            rss = int(line.split()[1])
        elif line.startswith('Threads:'):
            threads = int(line.split()[1])
    cmd = open('/proc/%d/cmdline' % pid, 'rb').read().replace(b'\0', b' ').decode()[:200]
    return ticks, rss, threads, cmd


def map_summary(pid):
    found = {}
    try:
        with open('/proc/%d/maps' % pid) as handle:
            for line in handle:
                for key in MAP_KEYS:
                    if key in line:
                        found.setdefault(key, set()).add(line.split()[-1])
    except (IOError, OSError):
        pass
    return {key: sorted(paths) for key, paths in found.items()}


def pct(values, q):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


class Monitors:
    def __init__(self):
        self.lock = threading.Lock()
        self.detection = []
        self.target = []
        self.status = []

    def clear(self):
        with self.lock:
            self.detection, self.target, self.status = [], [], []

    def on_detection(self, msg):
        with self.lock:
            self.detection.append((rospy.get_rostime().to_sec(), msg.header.stamp.to_sec(),
                                   bool(msg.detected), float(msg.confidence), msg.reason))

    def on_target(self, msg):
        age = (rospy.get_rostime() - msg.header.stamp).to_sec()
        with self.lock:
            self.target.append((rospy.get_rostime().to_sec(), bool(msg.detected), age, msg.status))

    def on_status(self, msg):
        with self.lock:
            self.status.append((int(msg.state), msg.reason))


def summarize(mon, window, pids, node_snapshot, args, t0_wall):
    out = dict(label=args.label, window_s=window, warmup_s=args.warmup,
               wall_clock=t0_wall, node_snapshot=node_snapshot, pids={})
    for label, pid in pids.items():
        try:
            out['pids'][label] = dict(pid=pid, cmdline=read_proc(pid)[3], maps=map_summary(pid))
        except (IOError, OSError, ValueError, IndexError):
            out['pids'][label] = dict(pid=pid, cmdline=None, maps={})
    with mon.lock:
        detection, target, status = list(mon.detection), list(mon.target), list(mon.status)
    det_detected = [row for row in detection if row[2]]
    reasons = {}
    for row in detection:
        reasons[row[4]] = reasons.get(row[4], 0) + 1
    states = {}
    for state, reason in status:
        states[state] = states.get(state, 0) + 1
    fault_reasons = {}
    for state, reason in status:
        if state == 4:
            fault_reasons[reason] = fault_reasons.get(reason, 0) + 1
    # Watchdog NOT_FOUND targets carry a zero header stamp; exclude them from latency.
    ages = [row[2] for row in target if 0 < row[2] < 5.]
    out['messages'] = dict(
        detection_hz=len(detection) / window,
        detection_detected_ratio=(len(det_detected) / len(detection)) if detection else None,
        confidence_median=(statistics.median([row[3] for row in det_detected])
                           if det_detected else None),
        detection_reasons=reasons,
        target_hz=len(target) / window,
        target_detected_ratio=(sum(1 for row in target if row[1]) / len(target)) if target else None,
        target_latency_samples=len(ages),
        target_latency_p50_s=pct(ages, .50),
        target_latency_p99_s=pct(ages, .99),
        target_status_counts={key: sum(1 for row in target if row[3] == key)
                              for key in sorted(set(row[3] for row in target))},
        state_distribution=states,
        fault_reasons=fault_reasons)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--warmup', type=float, default=10.)
    ap.add_argument('--window', type=float, default=60.)
    ap.add_argument('--interval', type=float, default=1.)
    ap.add_argument('--label', default='run')
    ap.add_argument('--out', required=True, help='output JSON path; tegrastats log gets .tegrastats.log')
    args = ap.parse_args()

    from xiaolan_yolo_detection.msg import PerceptionStatus, XiaolanDetection, XiaolanTarget
    rospy.init_node('resource_profile', disable_signals=True)
    mon = Monitors()
    subs = [
        rospy.Subscriber('/xiaolan/detection', XiaolanDetection, mon.on_detection, queue_size=20),
        rospy.Subscriber('/xiaolan/target', XiaolanTarget, mon.on_target, queue_size=20),
        rospy.Subscriber('/xiaolan/perception_status', PerceptionStatus, mon.on_status, queue_size=20),
    ]
    time.sleep(args.warmup)
    pids, node_snapshot = resolve_pids()
    missing = [label for _name, label in NODE_LABELS if label not in pids]
    if missing:
        print('WARNING: nodes not found on master: %s' % ', '.join(missing), file=sys.stderr)
    clk = os.sysconf('SC_CLK_TCK')
    prev, rss_first, thread_max = {}, {}, {}
    for label, pid in pids.items():
        try:
            prev[label] = read_proc(pid)[0]
            rss_first[label] = read_proc(pid)[1]
        except (IOError, OSError, ValueError, IndexError):
            prev[label] = None
    mon.clear()
    tegrastats = subprocess.Popen(['tegrastats', '--interval', '1000'],
                                  stdout=open(args.out + '.tegrastats.log', 'w'),
                                  stderr=subprocess.DEVNULL)
    window_start = time.monotonic()
    last = window_start
    cpu = {label: [] for label in pids}
    rss_last, rss_min, rss_max = {}, {}, {}
    while time.monotonic() - window_start < args.window:
        time.sleep(args.interval)
        now = time.monotonic()
        dt = now - last
        last = now
        for label, pid in pids.items():
            try:
                ticks, rss, threads, _cmd = read_proc(pid)
            except (IOError, OSError, ValueError, IndexError):
                continue
            if prev.get(label) is not None:
                cpu[label].append((ticks - prev[label]) / clk / dt * 100.)
            prev[label] = ticks
            rss_last[label] = rss
            if rss is not None:
                rss_min[label] = rss if label not in rss_min else min(rss_min[label], rss)
                rss_max[label] = rss if label not in rss_max else max(rss_max[label], rss)
            if threads is not None:
                thread_max[label] = threads if label not in thread_max else max(thread_max[label], threads)
    window = time.monotonic() - window_start
    tegrastats.terminate()
    tegrastats.wait(timeout=5)

    out = summarize(mon, window, pids, node_snapshot, args, time.strftime('%Y-%m-%dT%H:%M:%S'))
    out['processes'] = {}
    for label in pids:
        samples = cpu.get(label, [])
        out['processes'][label] = dict(
            cpu_pct_avg=sum(samples) / len(samples) if samples else None,
            cpu_pct_max=max(samples) if samples else None,
            cpu_samples=len(samples),
            rss_kb_first=rss_first.get(label), rss_kb_last=rss_last.get(label),
            rss_kb_min=rss_min.get(label), rss_kb_max=rss_max.get(label),
            threads_max=thread_max.get(label))
    env = dict(loadavg=open('/proc/loadavg').read().strip())
    try:
        env['nvpmodel_status'] = open('/var/lib/nvpmodel/status').read().strip()
    except (IOError, OSError):
        pass
    try:
        freqs = [open(p).read().strip() for p in
                 ['/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq' % i for i in range(os.cpu_count())]]
        env['cpu_cur_freq_khz'] = freqs
    except (IOError, OSError):
        pass
    out['env'] = env
    with open(args.out, 'w') as handle:
        json.dump(out, handle, indent=1, sort_keys=True)
        handle.write('\n')

    print('== %s == window %.1f s, nodes found: %s, missing: %s' %
          (args.label, window, ', '.join(pids) or '-', ', '.join(missing) or '-'))
    header = '%-12s %6s %8s %8s %9s %9s %5s' % ('label', 'pid', 'cpu%avg', 'cpu%max', 'rssMB_1st', 'rssMB_last', 'thr')
    print(header)
    for label in NODE_LABELS:
        proc = out['processes'].get(label[1])
        if proc is None or proc['cpu_pct_avg'] is None:
            continue
        print('%-12s %6d %8.1f %8.1f %9.1f %9.1f %5s' %
              (label[1], proc and pids[label[1]], proc['cpu_pct_avg'], proc['cpu_pct_max'],
               (proc['rss_kb_first'] or 0) / 1024., (proc['rss_kb_last'] or 0) / 1024.,
               proc['threads_max']))
    total = sum(p['cpu_pct_avg'] for p in out['processes'].values()
                if p is not None and p.get('cpu_pct_avg') is not None)
    print('%-12s %6s %8.1f   (sum of the processes above, %% of one core)' % ('TOTAL', '', total))
    msg = out['messages']
    print('detection %.2f Hz (detected %.0f%%, conf_med %s), target %.2f Hz (detected %.0f%%), '
          'latency(n=%s) p50 %s ms / p99 %s ms' %
          (msg['detection_hz'], 100 * (msg['detection_detected_ratio'] or 0),
           ('%.3f' % msg['confidence_median']) if msg['confidence_median'] is not None else '--',
           msg['target_hz'], 100 * (msg['target_detected_ratio'] or 0), msg['target_latency_samples'],
           ('%.0f' % (1000 * msg['target_latency_p50_s'])) if msg['target_latency_p50_s'] is not None else '--',
           ('%.0f' % (1000 * msg['target_latency_p99_s'])) if msg['target_latency_p99_s'] is not None else '--'))
    print('states %s; faults %s' % (msg['state_distribution'], msg['fault_reasons']))
    for sub in subs:
        sub.unregister()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
