#!/usr/bin/env python3
"""Build the bounded C2 speed candidate from the frozen f8da plan."""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from climb_mode import ClimbMode
from rebuild_climb_tail import original_tangents, stage_pose, world_from_base

BASE = ROOT / "src/docs/evidence/climb_lm_level_20260909/candidate_v2.json"
BASE_SHA256 = "f8da102282bffef261bfa0a2fe94b07558121f6c4bb38bc97c1c8c1c3061440f"


def require(value, detail):
    if not value:
        raise AssertionError(detail)


def rebuild_active_world(stage):
    """Synchronize C2's active RM world knots with its actual body pose."""
    times = np.r_[0.0, np.cumsum(stage["segment_durations_s"])]
    base_knots = np.asarray(stage["active_base_knots_m"], dtype=np.float64)
    for knot_index, time_s in enumerate(times):
        transform = world_from_base(stage_pose(stage, float(time_s)))
        for active_index, leg in enumerate(stage["active_legs"]):
            stage["anchor_knots"][knot_index][leg] = (
                transform @ np.r_[base_knots[knot_index, active_index], 1.0]
            )[:3].tolist()


def build(base):
    """Return the exact f8da deep copy with only C2's approved fields changed."""
    candidate = copy.deepcopy(base)
    stage = candidate["stages"][1]
    require(stage["name"] == "RM" and stage["active_legs"] == [5],
            "expected C2 RM-only stage")
    stage["active_base_knots_m"][2][0][2] -= .020
    stage["segment_durations_s"][1] = 1.7
    rebuild_active_world(stage)
    original_tangents(stage)
    require(candidate["stages"][0] == base["stages"][0]
            and candidate["stages"][2:] == base["stages"][2:],
            "only C2 may change")
    require({key: candidate[key] for key in candidate if key != "stages"}
            == {key: base[key] for key in base if key != "stages"},
            "top-level fields changed")
    ClimbMode(None)._validate_config(candidate)
    return candidate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=BASE)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raw = args.input.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == BASE_SHA256,
            "input must be frozen f8da evidence")
    candidate = build(json.loads(raw))
    args.output.write_text(json.dumps(candidate, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
