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

# The executable 32-stage compact plan is identified by stage name.  Candidate
# builders may merge stages, so its semantic speed contract must not follow a
# positional index after a stage-count change.
SEMANTICS_BY_NAME = {
    "PREP": ("BODY",),
    "RM": ("SWING_LIFT", "SWING_TRANSFER", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY": ("BODY",),
    "PAIR": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LB_LF_GROUND_SHIFT": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LM_GROUND_SHIFT": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY2": ("BODY",),
    "RM_HIGH_C": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "RB_RF_HIGH_C": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY3": ("BODY",),
    "RB_RF_SHIFT1": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LM_GROUND_SHIFT1": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "RM_PRE_ADVANCE": ("SWING_LIFT", "SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LB_LF_BODY_ADVANCE_HIGH_STEP": ("CRITICAL_BODY_TRANSFER",),
    "RB_RF_TOP_INWARD_PAIR": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LM_EDGE_STAGE": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_A": ("BODY",),
    "RM_RIGHT_SYMMETRY": ("SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_LEFT_TRANSFER_PREP": ("BODY",),
    "LB_LOW_STEP": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_RIGHT_BEFORE_LF": ("BODY",),
    "LF_LOW_STEP": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_PRELOAD_LM": ("BODY",),
    "LM_LIFT": ("SWING_LIFT", "SWING_TRANSFER"),
    "BODY_ADVANCE_LM_AIR": ("CRITICAL_BODY_TRANSFER",),
    "LM_LEFT_FINAL_LAND": ("TOUCHDOWN",),
    "RB_RF_DIRECT_FINAL": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "RM_DIRECT_FINAL": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_REPOSITION": ("BODY",),
    "LB_LF_DIRECT_FINAL": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "BODY_DOCK_FINAL": ("BODY",),
    "RB_BODY_ADVANCE": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LM_RM_PRE_ADVANCE": ("SWING_LIFT", "SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LM_RM_BODY_TRANSFER": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LM_BODY_TRANSFER": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "RM_BODY_REPOSITION": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
    "LB_LF_DOCK_TRANSFER": ("SWING_LIFT", "SWING_TRANSFER", "TOUCHDOWN"),
}

BODY_MINIMUM_BY_NAME = {
    "PREP": 1.80, "BODY": .80, "BODY2": 2.00, "BODY3": .80,
    "BODY_A": 1.20, "BODY_LEFT_TRANSFER_PREP": .80,
    "BODY_RIGHT_BEFORE_LF": LEFT_TRANSFER_BODY_MINIMUM_DURATION_S,
    "BODY_PRELOAD_LM": .50, "BODY_ADVANCE_LM_AIR": 3.00,
    "BODY_REPOSITION": 1.60, "BODY_DOCK_FINAL": 1.50,
}


def stage_specs(stage_index, stage):
    """Return immutable semantic/target/hard/minimum entries for one stage."""

    if stage["name"] == "STAND_FINAL_HOLD":
        return ()

    if stage["name"] in SEMANTICS_BY_NAME:
        semantics = SEMANTICS_BY_NAME[stage["name"]]
    elif stage["name"] in TAIL_SEMANTICS_BY_NAME:
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
            minimum = BODY_MINIMUM_BY_NAME.get(stage["name"],
                                                0.80 if stage_index in MAJOR_BODY_INDICES else 0.60)
        else:
            raise ValueError("unknown retime semantic: " + semantic)
        floor = (BODY_MINIMUM_BY_NAME.get(stage["name"], 0.0)
                 if stage["name"] in SEMANTICS_BY_NAME else
                 PHYSX_BODY_MINIMUM_DURATIONS.get(stage_index, 0.0))
        minimum = max(minimum, floor)
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
