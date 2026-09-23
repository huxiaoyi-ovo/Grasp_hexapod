#!/usr/bin/env python3
"""Focused structural contracts for the bounded C2 RM speed candidate."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

PACKAGE = Path(__file__).parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]

from climb_mode import ClimbMode
from validate_climb_preview import strict_contract

TOOL = SCRIPTS / "tools" / "rebuild_climb_c2_speed.py"
ACTIVE_CONFIG = PACKAGE / "config" / "climb_compact.json"
SPEC = importlib.util.spec_from_file_location("rebuild_climb_c2_speed", TOOL)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


def candidate():
    raw = BUILDER.BASE.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == BUILDER.BASE_SHA256
    base = json.loads(raw)
    item = BUILDER.build(base)
    ClimbMode(None)._validate_config(item)
    return base, item


def test_c2_only_changes_approved_knot_duration_world_and_tangent_fields():
    base, item = candidate()
    assert item["stages"][0] == base["stages"][0]
    assert item["stages"][2:] == base["stages"][2:]
    assert {key: item[key] for key in item if key != "stages"} == {
        key: base[key] for key in base if key != "stages"
    }
    before, after = base["stages"][1], item["stages"][1]
    assert after["segment_durations_s"] == [before["segment_durations_s"][0], 1.7,
                                               before["segment_durations_s"][2], before["segment_durations_s"][3]]
    expected = np.asarray(before["active_base_knots_m"], dtype=float).copy()
    expected[2, 0, 2] -= .020
    assert np.allclose(after["active_base_knots_m"], expected, rtol=0., atol=1e-12)
    for key in before:
        if key not in ("segment_durations_s", "active_base_knots_m",
                       "anchor_knots", "active_base_velocities_m_s"):
            assert after[key] == before[key]


def test_c2_boundary_fixed_anchors_and_internal_hermite_velocity():
    base, item = candidate()
    before, after = base["stages"][1], item["stages"][1]
    anchors = np.asarray(after["anchor_knots"], dtype=float)
    frozen = [leg for leg in range(6) if leg not in after["active_legs"]]
    assert np.allclose(anchors[:, frozen], np.asarray(before["anchor_knots"])[0, frozen], rtol=0., atol=1e-12)
    assert np.allclose(anchors[[0, -1], 5], np.asarray(before["anchor_knots"])[[0, -1], 5], rtol=0., atol=1e-12)
    velocity = np.asarray(after["active_base_velocities_m_s"], dtype=float)
    assert np.allclose(velocity[[0, -1]], 0., rtol=0., atol=1e-12)
    assert np.all(np.linalg.norm(velocity[1:-1], axis=2) > 0.)
    strict_contract(item)


def test_c2_contract_rejects_unapproved_duration_or_geometry():
    _, item = candidate()
    bad_duration = json.loads(json.dumps(item))
    bad_duration["stages"][1]["segment_durations_s"][0] -= .01
    with pytest.raises(AssertionError, match="C2 RM speed contract"):
        strict_contract(bad_duration)
    bad_geometry = json.loads(json.dumps(item))
    bad_geometry["stages"][1]["active_base_knots_m"][1][0][0] += .001
    with pytest.raises(AssertionError, match="C2 RM speed contract"):
        strict_contract(bad_geometry)


def test_active_merge_keeps_only_the_upstream_receipt_gate_override():
    active = json.loads(ACTIVE_CONFIG.read_text())
    strict_contract(active)
    assert active["front_v1_receipt"]["settle_gate"]["max_foot_target_error_m"] == .04
    assert active["front_v1_receipt"]["settle_gate"]["timeout_s"] == 5.0
    unexpected_gate = json.loads(json.dumps(active))
    unexpected_gate["front_v1_receipt"]["settle_gate"]["max_foot_target_error_m"] = .041
    with pytest.raises(AssertionError, match="concurrent front receipt gate identity"):
        strict_contract(unexpected_gate)
