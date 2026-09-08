"""Shared C1--C35 segment-speed contract for compact-climb diagnostics.

These limits are offline trajectory-planning gates.  They are not contact,
load, friction, stability, or hardware authorization evidence.
"""

import numpy as np


SEMANTIC_NAMES = (
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("CRITICAL_BODY_TRANSFER",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER"),
    ("CRITICAL_BODY_TRANSFER",),
    ("SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    ("BODY",),
)

MAJOR_BODY_INDICES = frozenset((0, 6, 16, 34))
FROZEN_PRELOAD_INDEX = 22
FROZEN_LB_LOW_STEP_INDEX = 19
FROZEN_LF_LOW_STEP_INDEX = 21
LEFT_TRANSFER_BODY_MINIMUM_DURATION_S = 1.20
# Full-chain PhysX-informed first-pass floors for body and critical-body
# stages.  They are simulation preview durations, not hardware speed limits.
PHYSX_BODY_MINIMUM_DURATIONS = {
    0: 1.80,
    2: 0.80,
    6: 2.00,
    9: 0.80,
    16: 1.20,
    18: 0.80,
    20: LEFT_TRANSFER_BODY_MINIMUM_DURATION_S,
    22: 0.50,
    24: 3.00,
    28: 0.80,
    31: 0.80,
    34: 1.50,
}

# The active 33-stage tail is keyed by names so historical 36-stage baseline
# diagnostics retain their original positional semantic map.
TAIL_SEMANTICS_BY_NAME = {
    "LM_LEFT_FINAL_LAND": ("TOUCHDOWN",),
    "RB_RF_DIRECT_FINAL": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_REPOSITION": ("BODY",),
    "LM_DIRECT_FINAL": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "RM_DIRECT_FINAL": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LB_LF_DIRECT_FINAL": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_DOCK_FINAL": ("BODY",),
}


def stage_specs(stage_index, stage):
    """Return immutable semantic/target/hard/minimum entries for one stage."""

    if stage["name"] == "STAND_FINAL_HOLD":
        return ()

    if stage["name"] in TAIL_SEMANTICS_BY_NAME:
        semantics = TAIL_SEMANTICS_BY_NAME[stage["name"]]
    elif not 0 <= stage_index < len(SEMANTIC_NAMES):
        return ()
    else:
        semantics = SEMANTIC_NAMES[stage_index]
    durations = stage["segment_durations_s"]
    if len(semantics) != len(durations):
        raise ValueError("retime semantic shape mismatch: " + stage["name"])
    active_count = len(stage["active_legs"])
    result = []
    for segment_index, semantic in enumerate(semantics):
        if semantic in ("SWING_LIFT", "SWING_TRANSFER"):
            target, hard = ((2.7, 3.2) if active_count == 2 else (3.0, 3.4))
            minimum = 0.35 if semantic == "SWING_LIFT" else 0.45
        elif semantic == "TOUCHDOWN":
            target, hard, minimum = 1.4, 1.8, 0.35
        elif semantic == "CRITICAL_BODY_TRANSFER":
            target, hard, minimum = 1.8, 2.4, 0.80
        elif semantic == "BODY":
            target, hard = 1.9, 2.4
            minimum = 0.80 if stage_index in MAJOR_BODY_INDICES else 0.60
            if stage["name"] == "BODY_REPOSITION":
                minimum = 1.60
            elif stage["name"] == "BODY_DOCK_FINAL":
                minimum = 1.50
            if stage_index == FROZEN_PRELOAD_INDEX:
                minimum = 0.50
        else:
            raise ValueError("unknown retime semantic: " + semantic)
        minimum = max(
            minimum, PHYSX_BODY_MINIMUM_DURATIONS.get(stage_index, 0.0)
        )
        result.append({
            "segment_index": segment_index,
            "semantic": semantic,
            "duration_s": float(durations[segment_index]),
            "target_rad_s": target,
            "hard_gate_rad_s": hard,
            "minimum_duration_s": minimum,
        })
    return result


def segment_for_time(stage_index, stage, time_s):
    """Classify a stage-local command time, including its settling samples."""

    specs = stage_specs(stage_index, stage)
    if not specs:
        return None
    cumulative = np.cumsum(stage["segment_durations_s"])
    segment_index = min(int(np.searchsorted(cumulative, time_s, side="right")),
                        len(specs) - 1)
    return specs[segment_index]


def speed_report(stage_index, stage):
    """Create stable per-segment report rows used by all validators."""

    return [{
        **spec,
        "measured_peak_rad_s": 0.0,
        "worst_source": None,
    } for spec in stage_specs(stage_index, stage)]


def update_speed_report(rows, segment_index, speed_rad_s, source):
    """Retain the largest absolute joint-speed sample and its source."""

    row = rows[segment_index]
    if speed_rad_s > row["measured_peak_rad_s"]:
        row["measured_peak_rad_s"] = float(speed_rad_s)
        row["worst_source"] = source


def assert_speed_report(rows, require):
    """Apply the semantic hard gates without weakening other validator gates."""

    for row in rows:
        require(
            row["measured_peak_rad_s"] <= row["hard_gate_rad_s"],
            row["worst_source"] or {
                "metric": "segment_speed_rad_s",
                "semantic": row["semantic"],
                "actual": row["measured_peak_rad_s"],
                "threshold": row["hard_gate_rad_s"],
            },
        )
