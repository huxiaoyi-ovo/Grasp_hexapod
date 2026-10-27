import copy
import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
TOOLS = SCRIPTS / "tools"
EVIDENCE = ROOT / "src/docs/evidence/climb_front_direct_20260924"
CONFIG_PATH = ROOT / "src/grasp_hexapod_control/config/climb_front_direct.json"
ACCEPTED_SHA = "4421aff133035f358e69ed33ddaa649d580750d868f2318736efc17e797bf60c"
sys.path[:0] = [str(SCRIPTS), str(TOOLS)]

from climb_mode import ClimbMode  # noqa: E402
from kinematics import Q_STAND  # noqa: E402
import replan_climb_front as builder  # noqa: E402


def load_config():
    data = CONFIG_PATH.read_bytes()
    assert hashlib.sha256(data).hexdigest() == ACCEPTED_SHA
    return json.loads(data)


def test_default_builder_reproduces_accepted_bytes_without_overwriting_snapshot(tmp_path):
    output = tmp_path / "regenerated.json"
    subprocess.run([sys.executable, str(TOOLS / "replan_climb_front.py"),
                    "--output", str(output)], cwd=ROOT, check=True,
                    capture_output=True, text=True)
    assert output.read_bytes() == (EVIDENCE / "rear_body_advance_candidate.json").read_bytes()
    assert hashlib.sha256(output.read_bytes()).hexdigest() == ACCEPTED_SHA


def test_accepted_route_is_continuous_and_never_reverses_an_active_foot():
    config = load_config()
    ClimbMode(None)._validate_config(config)
    assert config["stage_count"] == 21
    assert config["simulation_only"] is True
    assert config["simulation_candidate_only"] is True
    assert config["climb_orientation"] == "front"
    assert config["xiaolan_translation"] == pytest.approx([.45, -.03, 0.0])
    for first, second in zip(config["stages"], config["stages"][1:]):
        assert np.allclose(first["pose_end"], second["pose_start"], atol=1e-12, rtol=0)
        assert np.allclose(first["anchor_knots"][-1], second["anchor_knots"][0],
                           atol=1e-12, rtol=0)
    for stage in config["stages"]:
        if stage["name"] != "PREP":
            assert stage["pose_end"][0] >= stage["pose_start"][0] - 1e-12
        assert stage["pose_start"][5] == pytest.approx(-np.pi / 2, abs=1e-12)
        assert stage["pose_end"][5] == pytest.approx(-np.pi / 2, abs=1e-12)
    for stage in config["stages"]:
        knots = np.asarray(stage["anchor_knots"], dtype=float)
        active = set(stage["active_legs"])
        for leg in range(6):
            if leg not in active:
                assert np.allclose(knots[:, leg], knots[0, leg], atol=1e-12, rtol=0)
        for leg in active:
            assert knots[-1, leg, 0] > knots[0, leg, 0], (stage["name"], leg)

    with pytest.raises(ValueError, match="front compact climbing is simulation-only"):
        ClimbMode(None).enter(Q_STAND, config=config, hardware_execution=True)


@pytest.mark.parametrize("corruption, message", [
    ("ground", "low-ground landing"),
    ("front", "STL landing failed"),
])
def test_builder_rejects_cached_air_height_before_any_ik(corruption, message, monkeypatch):
    fast = json.loads(builder.FAST.read_bytes())
    receipt = json.loads(builder.ENDPOINTS.read_bytes())
    receipt = copy.deepcopy(receipt)
    if corruption == "ground":
        row = next(row for row in receipt["stages"] if any(
            np.asarray(row["world_anchors_target_m"])[builder.LEGS.index(name.upper()), 0] < .17792
            and abs(np.asarray(row["world_anchors_target_m"])[builder.LEGS.index(name.upper()), 2] - .0065) < 1e-6
            for name in row["active_legs"]
        ))
        name = next(name for name in row["active_legs"]
                    if np.asarray(row["world_anchors_target_m"])[builder.LEGS.index(name.upper()), 0] < .17792
                    and abs(np.asarray(row["world_anchors_target_m"])[builder.LEGS.index(name.upper()), 2] - .0065) < 1e-6)
        row["world_anchors_target_m"][builder.LEGS.index(name.upper())][2] = .0265
    else:
        row = next(row for row in receipt["stages"] if any(
            np.asarray(row["world_anchors_target_m"])[builder.LEGS.index(name.upper()), 0] > .30
            for name in row["active_legs"]
        ))
        name = next(name for name in row["active_legs"]
                    if np.asarray(row["world_anchors_target_m"])[builder.LEGS.index(name.upper()), 0] > .30)
        row["world_anchors_target_m"][builder.LEGS.index(name.upper())][2] += .020

    def forbidden_ik(*_args, **_kwargs):
        raise AssertionError("invalid contact height reached IK")

    monkeypatch.setattr(builder, "solve_ik", forbidden_ik)
    with pytest.raises(ValueError, match=message):
        builder.build(fast, receipt)


def test_cpu_mesh_and_gpu_receipts_bind_the_accepted_geometry_and_patch_samples():
    config = load_config()
    cpu = json.loads((EVIDENCE / "cpu_rear_body_advance.json").read_text())
    mesh = json.loads((EVIDENCE / "mesh_rear_body_advance_audit.json").read_text())
    manifest = json.loads((EVIDENCE / "gpu_rear_body_advance_start_manifest.json").read_text())
    gpu = json.loads((EVIDENCE / "gpu_rear_body_advance_summary.json").read_text())
    patches = json.loads((EVIDENCE / "gpu_rear_body_advance_actual_foot_stl_audit.json").read_text())
    manifest_bytes = (EVIDENCE / "gpu_rear_body_advance_start_manifest.json").read_bytes()
    metrics_sha = hashlib.sha256((EVIDENCE / "gpu_rear_body_advance_metrics.json").read_bytes()).hexdigest()
    assert cpu["state"] == "DONE" and cpu["config_sha256_before"] == ACCEPTED_SHA
    assert len(cpu["tick_samples"]) == 873
    assert mesh["candidate"]["config_sha256"] == ACCEPTED_SHA
    assert mesh["candidate"]["cpu_audit_sha256"] == hashlib.sha256(
        (EVIDENCE / "cpu_rear_body_advance.json").read_bytes()).hexdigest()
    assert mesh["inherited_prefix"]["queries_repeated"] == 0
    assert mesh["summary"]["real_triangle_hits"] == 0
    assert mesh["summary"]["total_queries"] == 46
    assert manifest["candidate_config_sha256"] == ACCEPTED_SHA
    assert gpu["candidate_config_sha256"] == ACCEPTED_SHA
    assert gpu["prelaunch_manifest_sha256"] == hashlib.sha256(manifest_bytes).hexdigest()
    assert gpu["gpu_metrics_sha256"] == metrics_sha
    assert gpu["final_state"] == "DONE"
    assert all(gate["result"] == "PASS" for gate in gpu["preview_gates"])
    assert patches["acceptance_pass"] is True
    assert patches["gpu_start_manifest_sha256"] == hashlib.sha256(manifest_bytes).hexdigest()
    assert patches["gpu_metrics_sha256"] == metrics_sha
    assert patches["first_mid_contact_entry"]["acceptance_pass"] is True
    assert all(f["signed_sphere_gap_pass_minus_0_5_to_plus_2_mm"]
               and f["nearest_surface_normal_pass_dot_0_995"]
               for f in patches["feet"])
    assert all(f["broad_patch_inside_10mm_margin"]
               for f in patches["feet"] if f["leg"] in ("LB", "RB", "LM", "RM"))
    assert config["stage_count"] == 21
