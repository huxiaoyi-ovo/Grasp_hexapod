import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(SCRIPTS / "tools"))

from climb_mode import ClimbMode
from rebuild_climb_concurrent import build_case, main as builder_main
from retime_climb import allowed_difference
from utils.climb_retime import stage_specs


SOURCE = {
    "RB_BODY_ADVANCE": (9, 10),
    "LM_RM_PRE_ADVANCE": (11, 12),
    "LM_RM_BODY_TRANSFER": (15, 17),
    "RM_BODY_REPOSITION": (27, 28),
    "LB_LF_DOCK_TRANSFER": (29, 30),
}
CRITICAL_LEFT = ("BODY_LEFT_TRANSFER_PREP", "LB_LOW_STEP",
                 "BODY_RIGHT_BEFORE_LF", "LF_LOW_STEP", "BODY_PRELOAD_LM",
                 "LM_LIFT", "BODY_ADVANCE_LM_AIR", "LM_LEFT_FINAL_LAND")


@pytest.fixture(scope="module")
def baseline():
    return json.loads((ROOT / "src/docs/evidence/climb_concurrency_20260908/baseline_config.json").read_text())


@pytest.mark.parametrize("case", ("A", "B", "C", "AC", "ABC"))
def test_merged_boundaries_fixed_anchors_and_untouched_stages(baseline, case):
    candidate = build_case(baseline, case)
    ClimbMode(None)._validate_config(candidate)
    original = {stage["name"]: stage for stage in baseline["stages"]}
    merged = {stage["name"]: stage for stage in candidate["stages"]
              if stage["name"] in SOURCE}
    for name, stage in merged.items():
        start, end = SOURCE[name]
        source_start, source_end = baseline["stages"][start], baseline["stages"][end]
        assert np.allclose(stage["pose_start"], source_start["pose_start"], rtol=0., atol=1e-12)
        assert np.allclose(stage["pose_end"], source_end["pose_end"], rtol=0., atol=1e-12)
        assert np.allclose(stage["anchor_knots"][0], source_start["anchor_knots"][0], rtol=0., atol=1e-12)
        assert np.allclose(stage["anchor_knots"][-1], source_end["anchor_knots"][-1], rtol=0., atol=1e-12)
        fixed = [leg for leg in range(6) if leg not in stage["active_legs"]]
        for knot in stage["anchor_knots"]:
            assert np.allclose(np.asarray(knot)[fixed], np.asarray(source_start["anchor_knots"][0])[fixed], rtol=0., atol=1e-12)
        velocity = np.asarray(stage["active_base_velocities_m_s"])
        assert np.allclose(velocity[[0, -1]], 0., rtol=0., atol=1e-12)
        assert np.any(np.abs(velocity[1:-1]) > 0.)
    consumed = {index for span in SOURCE.values() for index in range(span[0], span[1] + 1)}
    for index, stage in enumerate(baseline["stages"]):
        if index not in consumed:
            assert next(item for item in candidate["stages"] if item["name"] == stage["name"]) == stage
    for name in CRITICAL_LEFT:
        assert next(item for item in candidate["stages"] if item["name"] == name) == original[name]
    assert candidate["stages"][-1]["pose_end"] == baseline["stages"][-1]["pose_end"]
    assert candidate["stages"][-1]["anchor_knots"][-1] == baseline["stages"][-1]["anchor_knots"][-1]


def test_explicit_air_node_mappings(baseline):
    candidate = build_case(baseline, "ABC")
    stages = {stage["name"]: stage for stage in candidate["stages"]}
    lm_rm = stages["LM_RM_PRE_ADVANCE"]
    source_lm, source_rm = baseline["stages"][11], baseline["stages"][12]
    assert np.allclose(np.asarray(lm_rm["anchor_knots"])[[0, 2, 3, 4], 2],
                       np.asarray(source_lm["anchor_knots"])[[0, 1, 2, 3], 2], rtol=0., atol=1e-12)
    assert np.allclose(np.asarray(lm_rm["anchor_knots"])[:, 5],
                       np.asarray(source_rm["anchor_knots"])[:, 5], rtol=0., atol=1e-12)
    lm_rm_body = stages["LM_RM_BODY_TRANSFER"]
    source_lm, source_rm = baseline["stages"][15], baseline["stages"][17]
    assert np.allclose(np.asarray(lm_rm_body["anchor_knots"])[:, 2],
                       np.asarray(source_lm["anchor_knots"])[:, 2], rtol=0., atol=1e-12)
    expected = np.asarray(source_rm["anchor_knots"])[[0, 1, -1], 5]
    extra = expected[-1].copy(); extra[2] = max(expected[1, 2], expected[-1, 2] + .03)
    assert np.allclose(np.asarray(lm_rm_body["anchor_knots"])[:, 5],
                       np.vstack((expected[:2], extra, expected[-1])), rtol=0., atol=1e-12)


def test_b1_keeps_rm_stage_and_uses_only_lm_body_transfer(baseline):
    candidate = build_case(baseline, "AB1C")
    merged = next(stage for stage in candidate["stages"]
                  if stage["name"] == "LM_BODY_TRANSFER")
    lm_source = baseline["stages"][15]
    assert merged["active_legs"] == [2]
    assert merged["segment_durations_s"] == [.38, .95, .54]
    assert np.allclose(np.asarray(merged["anchor_knots"])[:, 2],
                       np.asarray(lm_source["anchor_knots"])[:, 2], rtol=0., atol=1e-12)
    assert next(stage for stage in candidate["stages"]
                if stage["name"] == "RM_RIGHT_SYMMETRY") == baseline["stages"][17]


def test_named_retime_freezes_survive_stage_index_changes(baseline):
    candidate = build_case(baseline, "AC")
    assert next(i for i, stage in enumerate(candidate["stages"])
                if stage["name"] == "LB_LOW_STEP") != 19
    changed = copy.deepcopy(candidate)
    next(stage for stage in changed["stages"] if stage["name"] == "LB_LOW_STEP")["segment_durations_s"][0] += .01
    with pytest.raises(AssertionError, match="LB_LOW_STEP"):
        allowed_difference(candidate, changed)
    changed = copy.deepcopy(candidate)
    next(stage for stage in changed["stages"] if stage["name"] == "LM_RM_PRE_ADVANCE")["segment_durations_s"][0] += .01
    with pytest.raises(AssertionError, match="continuous swing timing"):
        allowed_difference(candidate, changed)


def test_named_speed_contract_is_index_independent(baseline):
    candidate = build_case(baseline, "ABC")
    for stage in candidate["stages"]:
        if stage["name"] == "STAND_FINAL_HOLD":
            continue
        assert [row["semantic"] for row in stage_specs(0, stage)] == [
            row["semantic"] for row in stage_specs(999, stage)]


def test_builder_rejects_nonfrozen_baseline(tmp_path, monkeypatch, baseline):
    altered = copy.deepcopy(baseline)
    altered["stage_count"] = 0
    input_path = tmp_path / "altered.json"
    input_path.write_text(json.dumps(altered))
    monkeypatch.setattr(sys, "argv", ["builder", "--input", str(input_path),
                                       "--case", "A", "--output", str(tmp_path / "out.json"),
                                       "--report", str(tmp_path / "report.json")])
    with pytest.raises(ValueError, match="frozen 68f"):
        builder_main()
