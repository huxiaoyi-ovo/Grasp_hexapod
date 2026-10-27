#!/usr/bin/env python3
"""Summarize saved 30 Hz CPU thigh samples over uninterrupted fixed-foot intervals."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
CONFIG = HERE / "rear_body_advance_candidate.json"
CPU = HERE / "cpu_rear_body_advance.json"
OUTPUT = HERE / "fixed_contact_intervals.json"
LEGS = ("LB", "LF", "LM", "RB", "RF", "RM")
DT = 1.0 / 30.0


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    config_bytes, cpu_bytes = CONFIG.read_bytes(), CPU.read_bytes()
    config, cpu = json.loads(config_bytes), json.loads(cpu_bytes)
    config_sha, cpu_sha = sha(CONFIG), sha(CPU)
    if cpu.get("state") != "DONE" or cpu.get("config_sha256_before") != config_sha:
        raise ValueError("CPU receipt does not bind the saved candidate")
    if len(cpu["tick_samples"]) != 873:
        raise ValueError("saved CPU receipt does not have the expected continuous route ticks")

    stages = {stage["name"]: stage for stage in config["stages"]}
    stage_stats = {}
    for name, stage in stages.items():
        knots = stage["anchor_knots"]
        active = set(stage["active_legs"])
        for leg in range(6):
            if leg in active:
                continue
            xyz = [knot[leg] for knot in knots]
            max_anchor_delta = max(
                (sum((point[i] - xyz[0][i]) ** 2 for i in range(3)) ** .5 for point in xyz),
                default=0.0,
            )
            if max_anchor_delta > 1e-10:
                raise ValueError(f"config fixed anchor moved in {name}/{LEGS[leg]}")
            stage_stats[(name, leg)] = {"stage": name, "leg": LEGS[leg],
                                        "fixed_sample_count": 0, "thigh_min_rad": None,
                                        "thigh_max_rad": None, "thigh_span_rad": None,
                                        "peak_sampled_abs_thigh_rate_rad_s": 0.0,
                                        "world_anchor_xyz_m": xyz[0],
                                        "anchor_knot_max_delta_m": max_anchor_delta}

    stage_values = {key: [] for key in stage_stats}
    intervals = []
    open_interval = [None] * 6

    def finish(leg):
        rows = open_interval[leg]
        if not rows:
            return
        q = [float(row["q_rad"][leg][0]) for row in rows]
        anchor = stages[rows[0]["stage"]]["anchor_knots"][0][leg]
        max_anchor_delta = 0.0
        stage_names = []
        for row in rows:
            name = row["stage"]
            if not stage_names or stage_names[-1] != name:
                stage_names.append(name)
            xyz = stages[name]["anchor_knots"][0][leg]
            delta = sum((xyz[i] - anchor[i]) ** 2 for i in range(3)) ** .5
            max_anchor_delta = max(max_anchor_delta, delta)
        rates = []
        for prev, cur, q0, q1 in zip(rows, rows[1:], q, q[1:]):
            dt = (int(cur["tick"]) - int(prev["tick"])) * DT
            if dt <= 0:
                raise ValueError("CPU ticks are not strictly increasing")
            rates.append(abs(q1 - q0) / dt)
        intervals.append({
            "leg": LEGS[leg], "start_tick": rows[0]["tick"], "end_tick": rows[-1]["tick"],
            "start_stage": rows[0]["stage"], "end_stage": rows[-1]["stage"],
            "stage_sequence": stage_names, "sample_count": len(rows),
            "thigh_min_rad": min(q), "thigh_max_rad": max(q),
            "thigh_span_rad": max(q) - min(q),
            "peak_sampled_abs_thigh_rate_rad_s": max(rates, default=0.0),
            "fixed_world_anchor_xyz_m": anchor,
            "world_anchor_max_delta_from_interval_start_m": max_anchor_delta,
            "anchor_constant_within_1e_10m": max_anchor_delta <= 1e-10,
        })
        open_interval[leg] = None

    per_stage_samples = {key: [] for key in stage_stats}
    for row in cpu["tick_samples"]:
        active = set(int(i) for i in row["active_legs"])
        name = row["stage"]
        for leg in range(6):
            if leg in active:
                finish(leg)
                continue
            if open_interval[leg] is None:
                open_interval[leg] = []
            open_interval[leg].append(row)
            per_stage_samples[(name, leg)].append(row)
    for leg in range(6):
        finish(leg)

    for key, rows in per_stage_samples.items():
        if not rows:
            continue
        stat = stage_stats[key]
        q = [float(row["q_rad"][key[1]][0]) for row in rows]
        rates = [abs(q1 - q0) / ((int(r1["tick"]) - int(r0["tick"])) * DT)
                 for r0, r1, q0, q1 in zip(rows, rows[1:], q, q[1:])]
        stat.update({"fixed_sample_count": len(rows), "thigh_min_rad": min(q),
                     "thigh_max_rad": max(q), "thigh_span_rad": max(q) - min(q),
                     "peak_sampled_abs_thigh_rate_rad_s": max(rates, default=0.0)})

    result = {
        "source_config_sha256": config_sha, "source_cpu_receipt_sha256": cpu_sha,
        "tick_rate_hz": 30, "sample_count": len(cpu["tick_samples"]),
        "method": "Saved CPU q_rad[:,0] samples grouped over each uninterrupted non-active-foot interval; intervals continue across stage boundaries and end only when that leg is active.",
        "stage_fixed_leg_thigh_statistics": list(stage_stats.values()),
        "uninterrupted_fixed_contact_intervals": intervals,
        "summary": {"interval_count": len(intervals),
                    "max_interval_thigh_span_rad": max((r["thigh_span_rad"] for r in intervals), default=0.0),
                    "max_interval_peak_sampled_abs_thigh_rate_rad_s": max(
                        (r["peak_sampled_abs_thigh_rate_rad_s"] for r in intervals), default=0.0),
                    "all_interval_anchors_constant": all(r["anchor_constant_within_1e_10m"] for r in intervals),
                    "all_stage_fixed_anchors_constant": all(
                        r["anchor_knot_max_delta_m"] <= 1e-10 for r in stage_stats.values())},
        "boundary": "Sampled controller/kinematic thigh motion and planned anchor constancy only; no joint torque, measured contact, load, slip, stability, or physical support claim. No per-stage .08 rad rejection threshold is applied.",
    }
    OUTPUT.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(OUTPUT), "summary": result["summary"],
                      "config_sha256": config_sha, "cpu_sha256": cpu_sha}, indent=2))


if __name__ == "__main__":
    main()
