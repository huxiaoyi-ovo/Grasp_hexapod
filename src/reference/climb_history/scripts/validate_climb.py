"""Standalone M1A/M1B/M1C plus M2/M3 offline-readiness validation.

Run from the repository root as:
    python3 src/grasp_hexapod_control/scripts/tools/validate_climb.py
"""

import copy
import json
import hashlib
import math
from pathlib import Path
import struct
import tempfile
import sys

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))
ROOT = SCRIPTS_DIR.parents[2]

from tools import analyze_climb as analyze
from climb_mode import ClimbMode
from utils import climb
from utils import climb_collision as visual_collision
import control
from tools import plan_climb_trajectory as visual_planner
from tools import plan_climb_back_trajectory as back_planner
from tools import plan_climb_back_sim_finish as sim_finish
import kinematics as k
from utils import points_in_polygon


TOL = 1e-12

EXPECTED_SEQUENCE_NAMES = (
    "pair_first_rm_retract",
    "pair_first_rb_rf_extend",
    "pair_first_combined",
    "rm_first_then_pair",
)
EXPECTED_P0_STRATEGY_NAMES = (
    "rm_retract",
    "rb_rf_extend",
    "combined",
    "uncommitted_rm_first_sequence",
)
PAIR_FIRST_GROUPS = (("rb", "rf"), ("rm",))
RM_FIRST_GROUPS = (("rm",), ("rb", "rf"))
XIAOLAN_CAD_CONFIG_KEYS = {
    "status",
    "stl_sha256",
    "internal_features_policy",
    "upper_surfaces_policy",
    "upper_surfaces_height_range_m",
    "negative_x",
    "positive_x",
    "selector_tolerance_semantics",
}
XIAOLAN_CAD_SELECTOR_KEYS = {
    "normal",
    "plane_offset_m",
    "normal_tolerance",
    "plane_offset_tolerance_m",
    "transition_anchors_xy_m",
}
XIAOLAN_MESH_PATH = (
    SCRIPTS_DIR.parents[1]
    / "grasp_hexapod_description"
    / "meshes"
    / "xiaolan"
    / "base_link_xiaolan.STL"
)
XIAOLAN_STL_SHA256 = (
    "e61d489ab5d2b9e0e02fc8b1671cbddcd2e7cc49ffd48d1d2fed7541b42580be"
)
XIAOLAN_SELECTED_AREA_M2 = 0.0411512349894029
XIAOLAN_OUTER_AREA_M2 = 0.04415124099462368
XIAOLAN_INTERNAL_AREA_M2 = 0.0030000152353977984
EXPECTED_M2_PHYSICAL_PATHS = (
    "unknown.task_frame_origin_climb_m",
    "unknown.task_frame_y_positive_reference",
    "unknown.xiaolan_orientation_climb",
    "unknown.deck_height_tolerance_m",
    "unknown.deck_height_reference",
    "unknown.deck_edge_survey_climb",
    "unknown.deck_safe_polygon_uv_climb",
    "unknown.deck_normal_survey_climb",
    "unknown.body_collision_envelope_m",
    "unknown.bottom_camera_collision_envelope_m",
    "unknown.motor_collision_envelope_m",
    "unknown.measured_foot_geometry",
    "unknown.real_mass_kg",
    "unknown.real_com_uncertainty_m",
    "unknown.friction_range",
    "unknown.lx15d_loaded_tracking_error_rad",
    "unknown.lx15d_backlash_rad",
    "unknown.lx15d_loaded_speed_rad_s",
    "unknown.lx15d_board_skew_rad",
)


def require(condition, name):
    if not condition:
        raise AssertionError("M1A validation failed: " + name)


def require_close(actual, expected, name, tol=TOL):
    require(
        np.allclose(actual, expected, rtol=0.0, atol=tol),
        name,
    )


def check_common(q, name_prefix):
    model = k.GraspKinematic()
    transforms = model.link_transforms_base(q)
    require(transforms.shape == (6, 4, 4, 4), name_prefix + " transform shape")
    require(np.all(np.isfinite(transforms)), name_prefix + " transform finiteness")

    require(
        np.allclose(transforms[:, :, 3, :], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=TOL),
        name_prefix + " homogeneous bottom rows",
    )
    rotations = transforms[:, :, :3, :3]
    identity = np.eye(3, dtype=np.float64)
    require(
        np.allclose(
            np.matmul(rotations.swapaxes(-1, -2), rotations),
            identity,
            rtol=0.0,
            atol=TOL,
        ),
        name_prefix + " rotation orthonormality",
    )
    require(
        np.allclose(
            np.linalg.det(rotations),
            1.0,
            rtol=0.0,
            atol=TOL,
        ),
        name_prefix + " proper rotations",
    )

    require_close(
        transforms[:, :, :3, 3],
        model.link_points_base(q),
        name_prefix + " transform origins vs link points",
    )
    require_close(
        transforms[:, 3, :3, 3],
        model.forward_base(q),
        name_prefix + " foot origins vs forward_base",
    )

    link_coms = model.link_com_positions_base(q)
    require(link_coms.shape == (6, 4, 3), name_prefix + " link COM shape")
    require(np.all(np.isfinite(link_coms)), name_prefix + " link COM finiteness")
    require_close(
        link_coms[:, 3],
        transforms[:, 3, :3, 3],
        name_prefix + " zero-local-offset foot COM",
    )

    axes = model.terminal_axes_base(q)
    require(axes.shape == (6, 3), name_prefix + " terminal axis shape")
    require(np.all(np.isfinite(axes)), name_prefix + " terminal axis finiteness")
    require_close(
        np.linalg.norm(axes, axis=1),
        np.ones(6, dtype=np.float64),
        name_prefix + " terminal axis unit norm",
    )
    require(
        np.all(axes[:, 2] < -0.999),
        name_prefix + " terminal axes point below base",
    )

    com = model.center_of_mass_base(q)
    require(com.shape == (3,), name_prefix + " COM shape")
    require(np.all(np.isfinite(com)), name_prefix + " COM finiteness")

    singular_values = model.jacobian_min_singular_values(q)
    require(singular_values.shape == (6,), name_prefix + " singular value shape")
    require(np.all(np.isfinite(singular_values)), name_prefix + " singular value finiteness")
    require(np.all(singular_values > 0.0), name_prefix + " positive singular values")

    margins = model.joint_limit_margins(q)
    require(margins.shape == (6, 3), name_prefix + " limit margin shape")
    require(np.all(np.isfinite(margins)), name_prefix + " limit margin finiteness")
    require(np.all(margins > 0.0), name_prefix + " positive limit margins")


require_close(
    k.ESTIMATED_TOTAL_MASS,
    4.482,
    "estimated total mass",
)
require_close(
    k.ESTIMATED_TOTAL_MASS,
    k.ESTIMATED_BASE_MASS + 6.0 * np.sum(k.ESTIMATED_LEG_LINK_MASSES),
    "estimated total mass includes six fixed foot masses",
)

check_common(k.Q_STAND, "Q_STAND")

expected_forward_base = np.array(
    [
        [-0.09698546701054667, -0.1679811808930615, -0.06338468052793655],
        [-0.09698603091855472, 0.1679808553169389, -0.06338468052793655],
        [-0.1939705735429649, 0.0000004749691728617622, -0.06338468052793655],
        [0.09698533763006974, -0.1679812555911260, -0.06338468052793655],
        [0.09698477371966989, 0.1679815811631059, -0.06338468052793655],
        [0.1939705735435137, 0.0000003255733608508978, -0.06338468052793655],
    ],
    dtype=np.float64,
)
require_close(
    k.GraspKinematic().forward_base(k.Q_STAND),
    expected_forward_base,
    "Q_STAND forward_base",
)

expected_com = np.array(
    [0.000323783507785, 0.002154715763596, 0.045125794381923],
    dtype=np.float64,
)
require_close(
    k.GraspKinematic().center_of_mass_base(k.Q_STAND),
    expected_com,
    "Q_STAND estimated COM",
)

expected_singular_values = np.full(
    (6,),
    0.047151804576541,
    dtype=np.float64,
)
require_close(
    k.GraspKinematic().jacobian_min_singular_values(k.Q_STAND),
    expected_singular_values,
    "Q_STAND minimum singular values",
)

offsets = np.deg2rad(
    np.array(
        [
            [3.0, -4.0, 5.0],
            [-2.0, 3.0, -4.0],
            [4.0, -2.0, 3.0],
            [-3.0, 4.0, -2.0],
            [2.0, -3.0, 4.0],
            [-4.0, 2.0, -3.0],
        ],
        dtype=np.float64,
    )
)
q_offset = k.Q_STAND + offsets
check_common(q_offset, "Q_STAND+offsets")

below_limit = k.Q_STAND.copy()
below_limit[0, 0] = k.JOINT_LOWER[0, 0] - 1e-9
margins = k.GraspKinematic().joint_limit_margins(below_limit)
expected_margin = below_limit[0, 0] - k.JOINT_LOWER[0, 0]
require(margins[0, 0] == expected_margin, "below-limit exact margin value")
require(margins[0, 0] < 0.0, "below-limit margin is negative")


# 检查攀爬配置和几何支撑计算。

config_path = SCRIPTS_DIR.parent / "config" / "climb.json"
require(config_path.is_file(), "repository climb.json exists")
config = climb.load_climb_config(config_path)
require(
    config["schema_version"] == climb.CLIMB_CONFIG_SCHEMA_VERSION,
    "climb config schema version",
)
require(config["status"] == "UNRESOLVED", "climb config status is UNRESOLVED")
require(
    config["units"] == {"length": "m", "angle": "rad"},
    "climb config units are m and rad",
)
require_close(
    config["known"]["deck_height_nominal_m"],
    0.150,
    "climb config nominal deck height",
)
require(
    len(config["unresolved"]) > 0,
    "climb config unresolved list is nonempty",
)
require(not climb.climb_config_ready(config), "UNRESOLVED config is not ready")

invalid_ready = dict(config)
invalid_ready["status"] = "READY"
require(
    not climb.climb_config_ready(invalid_ready),
    "READY config with unresolved list is rejected without file I/O",
)
try:
    climb.validate_climb_config(invalid_ready)
    raise AssertionError(
        "M1B validation failed: READY+unresolved direct dictionary must raise"
    )
except ValueError:
    pass

missing_unresolved = dict(config)
missing_unresolved["unresolved"] = config["unresolved"][:-1]
try:
    climb.validate_climb_config(missing_unresolved)
    raise AssertionError(
        "M1B validation failed: unlisted null config value must raise"
    )
except ValueError:
    pass

require(
    "deploy_contact_load_threshold" not in str(config),
    "config never invents a contact/load threshold",
)
require(
    "lora_uncertainty_m" not in str(config),
    "LoRa transport is not assigned spatial uncertainty",
)

surface = climb.build_surface_frame(
    "horizontal",
    np.zeros(3, dtype=np.float64),
    np.array([0.0, 0.0, 1.0], dtype=np.float64),
    np.array([1.0, 0.0, 0.0], dtype=np.float64),
    np.array(
        [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
        dtype=np.float64,
    ),
)
require_close(
    surface.basis_climb,
    np.eye(3, dtype=np.float64),
    "horizontal surface identity basis",
)
surface_points = np.array(
    [[0.2, -0.3, 0.5], [-1.1, 0.4, -0.2]],
    dtype=np.float64,
)
surface_uv, surface_height = climb.project_points_to_surface(
    surface,
    surface_points,
)
surface_round_trip = climb.surface_points_from_uv(
    surface,
    surface_uv,
    surface_height,
)
require_close(
    surface_round_trip,
    surface_points,
    "surface projection/inverse round-trip",
)
_, positive_height = climb.project_points_to_surface(
    surface,
    np.array([[0.0, 0.0, 0.2]], dtype=np.float64),
)
_, negative_height = climb.project_points_to_surface(
    surface,
    np.array([[0.0, 0.0, -0.3]], dtype=np.float64),
)
require_close(positive_height, [0.2], "positive signed height")
require_close(negative_height, [-0.3], "negative signed height")

terminal_angles = climb.terminal_axis_surface_angles(
    np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0]], dtype=np.float64),
    np.array([0.0, 0.0, 1.0], dtype=np.float64),
)
require_close(
    terminal_angles,
    np.array([0.0, np.pi / 2.0], dtype=np.float64),
    "terminal axis surface angles",
)

square_hull_points = np.array(
    [
        [-1.0, -1.0],
        [1.0, -1.0],
        [1.0, 1.0],
        [-1.0, 1.0],
        [0.0, 0.0],
        [0.0, -1.0],
        [1.0, 0.0],
        [-1.0, 0.0],
        [0.0, 1.0],
        [-1.0, -1.0],
    ],
    dtype=np.float64,
)
square_hull = climb.convex_hull_2d(square_hull_points)
require_close(
    square_hull,
    np.array(
        [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
        dtype=np.float64,
    ),
    "square hull is four CCW corners",
)
require_close(
    climb.signed_polygon_margin(np.array([0.0, 0.0]), square_hull),
    1.0,
    "signed center margin",
)
require_close(
    climb.signed_polygon_margin(np.array([1.5, 0.0]), square_hull),
    -0.5,
    "signed outside margin",
)

square_contacts = np.array(
    [
        [-1.0, -1.0, 0.0],
        [1.0, -1.0, 0.0],
        [1.0, 1.0, 0.0],
        [-1.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)
square_support = climb.gravity_projected_support(
    np.array([0.0, 0.0, 0.2], dtype=np.float64),
    square_contacts,
    np.array([0.0, 0.0, -9.81], dtype=np.float64),
)
require(square_support.valid, "square gravity support is valid")
require_close(
    square_support.raw_margin_m,
    1.0,
    "square gravity raw margin",
)
square_support_uncertain = climb.gravity_projected_support(
    np.array([0.0, 0.0, 0.2], dtype=np.float64),
    square_contacts,
    np.array([0.0, 0.0, -9.81], dtype=np.float64),
    uncertainty_radius_m=0.2,
)
require_close(
    square_support_uncertain.robust_margin_m,
    0.8,
    "square gravity robust margin",
)
require(
    square_support_uncertain.robust_inside,
    "square gravity support robustly inside",
)

collinear_contacts = np.array(
    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
    dtype=np.float64,
)
collinear_support = climb.gravity_projected_support(
    np.array([0.5, 0.0, 0.1], dtype=np.float64),
    collinear_contacts,
    np.array([0.0, 0.0, -9.81], dtype=np.float64),
)
require(not collinear_support.valid, "collinear contacts are invalid")
require(
    not collinear_support.robust_inside,
    "collinear contacts are never robustly inside",
)
require(
    np.isneginf(collinear_support.raw_margin_m)
    and np.isneginf(collinear_support.robust_margin_m),
    "collinear support margins are -inf",
)
require(
    len(collinear_support.reason) > 0,
    "invalid support has a reason",
)

kinematic = k.GraspKinematic()
feet = kinematic.forward_base(k.Q_STAND)
com = kinematic.center_of_mass_base(k.Q_STAND)
gravity_down = np.array([0.0, 0.0, -9.81], dtype=np.float64)

quad_support = climb.gravity_projected_support(
    com,
    feet[[5, 1, 0, 2]],
    gravity_down,
)
require(quad_support.valid, "rm/lf/lb/lm gravity support is valid")
require_close(
    quad_support.raw_margin_m,
    0.09495618344188111,
    "rm/lf/lb/lm Q_STAND raw margin",
)

far_tripod_support = climb.gravity_projected_support(
    com,
    feet[[1, 0, 2]],
    gravity_down,
)
require(far_tripod_support.valid, "far tripod gravity support is valid")
require_close(
    far_tripod_support.raw_margin_m,
    -0.09730953608913526,
    "far tripod Q_STAND raw margin",
)
require(
    not far_tripod_support.robust_inside,
    "far tripod is not robustly inside",
)

stand_terminal_axes = kinematic.terminal_axes_base(k.Q_STAND)
stand_terminal_errors = climb.terminal_axis_surface_angles(
    stand_terminal_axes,
    np.array([0.0, 0.0, 1.0], dtype=np.float64),
)
require(
    np.all(np.isfinite(stand_terminal_errors)),
    "Q_STAND terminal-axis surface errors are finite",
)
require(
    np.max(stand_terminal_errors) < 0.002,
    "Q_STAND terminal-axis surface errors max < 0.002 rad",
)


# 检查离线分析候选项和必需输入路径。

sequence_specs = climb.climb_sequence_specs()
require(len(sequence_specs) == 4, "four candidate sequence specs")
require(
    [spec.name for spec in sequence_specs] == list(EXPECTED_SEQUENCE_NAMES),
    "candidate sequence ids and order",
)
require(
    len({spec.name for spec in sequence_specs}) == 4,
    "candidate sequence ids are unique",
)
require(
    [spec.p0_strategy for spec in sequence_specs]
    == list(EXPECTED_P0_STRATEGY_NAMES),
    "p0 strategy names",
)
for spec, expected_groups in zip(
    sequence_specs,
    (
        PAIR_FIRST_GROUPS,
        PAIR_FIRST_GROUPS,
        PAIR_FIRST_GROUPS,
        RM_FIRST_GROUPS,
    ),
):
    require(
        tuple(spec.initial_platform_groups) == expected_groups,
        "pair/rm-first platform-group semantics",
    )
    for group in spec.initial_platform_groups:
        for leg_name in group:
            require(
                leg_name in climb.CLIMB_LEG_NAMES,
                "candidate sequence leg names are valid",
            )
    require(
        set(spec.__dataclass_fields__)
        == {"name", "p0_strategy", "initial_platform_groups"},
        "candidate spec has no invented P0 reseat/trajectory fields",
    )

require(
    tuple(climb.M2_REQUIRED_PHYSICAL_PATHS) == EXPECTED_M2_PHYSICAL_PATHS,
    "M2 required physical paths match contract",
)
require(
    tuple(climb.M3_REQUIRED_PHYSICAL_PATHS) == EXPECTED_M2_PHYSICAL_PATHS,
    "M3 required physical paths match contract at this stage",
)

config_snapshot = json.loads(json.dumps(config))

m2_presence = climb.offline_input_presence(config, "M2")
require(m2_presence.milestone == "M2", "M2 presence milestone")
require(m2_presence.status == "UNRESOLVED", "M2 presence is UNRESOLVED")
require(
    tuple(m2_presence.missing_paths) == EXPECTED_M2_PHYSICAL_PATHS,
    "M2 exact missing-path order",
)
m3_presence = climb.offline_input_presence(config, "M3")
require(m3_presence.milestone == "M3", "M3 presence milestone")
require(m3_presence.status == "UNRESOLVED", "M3 presence is UNRESOLVED")
require(
    tuple(m3_presence.missing_paths) == EXPECTED_M2_PHYSICAL_PATHS,
    "M3 exact missing-path order",
)
for presence in (m2_presence, m3_presence):
    require(
        "not schema/value validation" in presence.note
        and "certification" in presence.note
        and "deployment approval" in presence.note
        and "contact/load proof" in presence.note,
        "input-presence note states its limits",
    )

try:
    climb.offline_input_presence(config, "M4")
    raise AssertionError(
        "M2/M3 validation failed: unknown milestone must be rejected"
    )
except ValueError:
    pass

try:
    climb.offline_input_presence({"schema_version": 1}, "M2")
    raise AssertionError(
        "M2/M3 validation failed: invalid config must be rejected first"
    )
except ValueError:
    pass


# 检查离线分析配置中的未解决项。

offline_analysis = config["offline_analysis"]
require(
    isinstance(offline_analysis, dict)
    and offline_analysis.get("status") == "UNRESOLVED",
    "offline_analysis status is UNRESOLVED",
)
require(
    "blocked by required physical inputs" in offline_analysis["blocked_by"],
    "offline_analysis states search/certification is blocked",
)
require(
    offline_analysis["candidate_sequence_ids"]
    == list(EXPECTED_SEQUENCE_NAMES),
    "offline_analysis candidate sequence ids and order",
)
require(
    offline_analysis["m2_gate"]
    == "physical_input_presence_then_joint_search_and_full_sample_certification",
    "offline_analysis M2 gate string",
)
require(
    offline_analysis["m3_gate"]
    == "continuous_C4_C5_corridor_with_far_tripod_fallback",
    "offline_analysis M3 gate string",
)
require(
    set(offline_analysis)
    == {
        "status",
        "blocked_by",
        "candidate_sequence_ids",
        "m2_gate",
        "m3_gate",
    },
    "offline_analysis stays compact without invented fields",
)
climb.validate_climb_config(config)


# 检查离线基础报告。

report = analyze.build_baseline_report(config)
json.dumps(report, allow_nan=False, sort_keys=True)
require(
    report["overall_status"] == "UNRESOLVED",
    "baseline overall status is UNRESOLVED",
)
require(
    report["readiness_gates"]["M2"]["status"] == "UNRESOLVED"
    and report["readiness_gates"]["M3"]["status"] == "UNRESOLVED",
    "baseline M2/M3 gates are UNRESOLVED",
)
require_close(
    report["model_baseline"][
        "hip_origin_x_delta_rm_minus_mean_rb_rf_m"
    ],
    0.0425,
    "baseline hip-origin x delta",
)
require_close(
    report["known_nominal_values"]["hip_origin_delta_m"],
    0.0425,
    "configured hip-origin delta remains distinct",
)
expected_foot_delta = float(
    k.GraspKinematic().forward_base(k.Q_STAND)[5, 0]
    - 0.5
    * (
        k.GraspKinematic().forward_base(k.Q_STAND)[3, 0]
        + k.GraspKinematic().forward_base(k.Q_STAND)[4, 0]
    )
)
require_close(
    report["model_baseline"][
        "foot_center_x_delta_rm_minus_mean_rb_rf_m"
    ],
    expected_foot_delta,
    "baseline foot-center x delta matches Q_STAND",
)
require(
    abs(expected_foot_delta - 0.096985) < 1e-6,
    "baseline foot-center x delta is about 0.096985 m",
)
require(
    abs(
        report["known_nominal_values"]["foot_center_delta_nominal_m"]
        - expected_foot_delta
    ) < 1e-6,
    "configured nominal foot-center delta matches the Q_STAND model",
)
require(
    max(
        report["known_nominal_values"][
            "rm_motor_envelope_rough_range_m"
        ]
    )
    < report["known_nominal_values"]["foot_center_delta_nominal_m"],
    "motor-envelope observation is not conflated with foot-center delta",
)
require_close(
    report["geometry_only_support"]["near_four"]["raw_margin_m"],
    0.09495618344188111,
    "baseline near-four raw support margin",
)
require_close(
    report["geometry_only_support"]["far_tripod"]["raw_margin_m"],
    -0.09730953608913526,
    "baseline far-tripod raw support margin",
)
require(
    not report["geometry_only_support"]["far_tripod"][
        "inside_raw_geometry"
    ],
    "baseline far tripod COM is outside raw model geometry",
)
for support_name in ("near_four", "far_tripod"):
    support_report = report["geometry_only_support"][support_name]
    require(
        support_report["applied_uncertainty_radius_m"] == 0.0
        and support_report["uncertainty_status"]
        == "REAL_COM_UNCERTAINTY_UNRESOLVED",
        "baseline support reports zero applied and unresolved real uncertainty",
    )
    require(
        "robust_margin_m" not in support_report
        and "robust_inside" not in support_report,
        "baseline does not label zero-uncertainty margins as robust",
    )
require(
    [candidate["name"] for candidate in report["candidate_sequences"]]
    == list(EXPECTED_SEQUENCE_NAMES),
    "baseline candidate sequence ids and order",
)


def reject_forbidden_claim_strings(value):
    forbidden = {
        "PASS",
        "CERTIFIED",
        "CONTACT",
        "LOAD",
        "REAL-ROBOT AUTHORIZATION",
    }
    if isinstance(value, dict):
        for item in value.values():
            reject_forbidden_claim_strings(item)
    elif isinstance(value, list):
        for item in value:
            reject_forbidden_claim_strings(item)
    elif isinstance(value, str):
        require(
            value.strip().upper() not in forbidden,
            "baseline contains no forbidden status/claim text",
        )


reject_forbidden_claim_strings(report)
require(
    any("no trajectory search" in item for item in report["limitations"])
    and any(
        "not a contact/load/friction/stability proof"
        in item
        for item in report["limitations"]
    )
    and any("no real-robot authorization" in item for item in report["limitations"]),
    "baseline states explicit claim limitations",
)

require(
    config == config_snapshot,
    "offline helpers and baseline builder do not mutate config",
)


# 检查小蓝低斜面的 CAD 配置和提取结果。

xiaolan_cad = config["known"][analyze.XIAOLAN_CAD_CONFIG_KEY]
require(
    set(xiaolan_cad) == XIAOLAN_CAD_CONFIG_KEYS,
    "xiaolan CAD config object has exactly the contract fields",
)
require(
    xiaolan_cad["status"] == "USER_CONFIRMED_MODEL_GEOMETRY",
    "xiaolan CAD config status is USER_CONFIRMED_MODEL_GEOMETRY",
)
require(
    xiaolan_cad["stl_sha256"] == XIAOLAN_STL_SHA256,
    "xiaolan CAD config STL SHA-256",
)
require(
    xiaolan_cad["internal_features_policy"]
    == "IGNORE_FOR_FIRST_PASS_LANDING_REGION",
    "xiaolan internal-loop policy",
)
require(
    xiaolan_cad["upper_surfaces_policy"] == "NO_CONTACT_OBSTACLE",
    "xiaolan upper-surface policy",
)
require_close(
    xiaolan_cad["upper_surfaces_height_range_m"],
    [0.18, 0.19],
    "xiaolan upper-surface height range",
)
require(
    "numerical" in xiaolan_cad["selector_tolerance_semantics"]
    and "not physical safety margins" in xiaolan_cad[
        "selector_tolerance_semantics"
    ],
    "selector tolerances are numerical matching tolerances only",
)
for side_name, side_sign, expected_anchors in (
    (
        "negative_x",
        -1.0,
        [[-0.1445341, 0.1724553], [-0.2084391, -0.2241534]],
    ),
    (
        "positive_x",
        1.0,
        [[0.1445341, 0.1724553], [0.2084391, -0.2241534]],
    ),
):
    selector = xiaolan_cad[side_name]
    require(
        set(selector) == XIAOLAN_CAD_SELECTOR_KEYS,
        side_name + " selector has exactly the contract fields",
    )
    require_close(
        selector["normal"],
        [side_sign * 0.20791157, 0.0, 0.97814763],
        side_name + " selector normal",
    )
    require_close(
        selector["plane_offset_m"],
        0.19515595,
        side_name + " selector plane offset",
    )
    require(
        selector["normal_tolerance"] == 2e-5
        and selector["plane_offset_tolerance_m"] == 2e-6,
        side_name + " selector numerical tolerances",
    )
    require(
        selector["transition_anchors_xy_m"] == expected_anchors,
        side_name + " transition anchors",
    )

require(XIAOLAN_MESH_PATH.is_file(), "repository Xiaolan STL exists")
cad_report = analyze.build_xiaolan_cad_report(config, XIAOLAN_MESH_PATH)
require(
    cad_report["status"] == "MODEL_GEOMETRY_READY",
    "CAD geometry report is MODEL_GEOMETRY_READY",
)
json.dumps(cad_report, allow_nan=False, sort_keys=True)

for side_name in ("negative_x", "positive_x"):
    side_report = cad_report[side_name]
    require(
        side_report["selected_triangle_count"] == 43,
        side_name + " selected triangle count",
    )
    require_close(
        side_report["selected_triangle_area_m2"],
        XIAOLAN_SELECTED_AREA_M2,
        side_name + " selected triangle area",
        tol=1e-12,
    )
    require_close(
        side_report["plane_normal"],
        xiaolan_cad[side_name]["normal"],
        side_name + " normalized plane normal matches selector",
        tol=xiaolan_cad[side_name]["normal_tolerance"],
    )
    require_close(
        side_report["plane_offset_m"],
        xiaolan_cad[side_name]["plane_offset_m"],
        side_name + " plane offset matches selector",
        tol=xiaolan_cad[side_name]["plane_offset_tolerance_m"],
    )
    require(
        abs(side_report["tilt_from_horizontal_deg"] - 12.0) < 0.01,
        side_name + " tilt is about 12 degrees",
    )

    if side_name == "negative_x":
        require_close(
            side_report["bounds_3d_m"]["min"],
            [-0.27207911, -0.25747350, 0.14168364],
            "negative-x lower bounds",
            tol=1e-7,
        )
        require_close(
            side_report["bounds_3d_m"]["max"],
            [-0.14143395, 0.17374368, 0.16945313],
            "negative-x upper bounds",
            tol=1e-7,
        )
    else:
        require_close(
            side_report["bounds_3d_m"]["min"],
            [0.14143395, -0.25747350, 0.14168364],
            "positive-x lower bounds",
            tol=1e-7,
        )
        require_close(
            side_report["bounds_3d_m"]["max"],
            [0.27207911, 0.17374368, 0.16945313],
            "positive-x upper bounds",
            tol=1e-7,
        )

    require(
        side_report["boundary_loop_count"] == 2,
        side_name + " has exactly two boundary loops",
    )
    loops_by_kind = {
        loop["kind"]: loop for loop in side_report["boundary_loops"]
    }
    outer_loop = loops_by_kind["PLANNER_OUTER"]
    internal_loop = loops_by_kind["IGNORED_INTERNAL"]
    require(
        outer_loop["vertex_count"] == 25,
        side_name + " outer loop has 25 edges",
    )
    require_close(
        outer_loop["projected_area_m2"],
        XIAOLAN_OUTER_AREA_M2,
        side_name + " outer loop projected area",
        tol=1e-12,
    )
    require(
        internal_loop["vertex_count"] == 18,
        side_name + " internal loop has 18 edges",
    )
    require_close(
        internal_loop["projected_area_m2"],
        XIAOLAN_INTERNAL_AREA_M2,
        side_name + " internal loop projected area",
        tol=1e-12,
    )
    require(
        side_report["ignored_internal_loop_count"] == 1,
        side_name + " one ignored internal loop",
    )
    require_close(
        side_report["planner_outer_boundary_area_m2"],
        outer_loop["projected_area_m2"],
        side_name + " planner area equals outer-loop area",
        tol=0.0,
    )
    require(
        abs(
            side_report["planner_outer_boundary_area_m2"]
            - (
                outer_loop["projected_area_m2"]
                - internal_loop["projected_area_m2"]
            )
        )
        > 1e-9,
        side_name + " planner area does not subtract the ignored loop",
    )
    require(
        side_report["step_high_segment_count"] == 10
        and side_report["outer_drop_segment_count"] == 15,
        side_name + " step-high and outer-drop segment counts",
    )

    outer_vertices = [
        tuple(round(float(value), 7) for value in vertex)
        for vertex in outer_loop["vertices_3d_m"]
    ]
    outer_edge_set = set()
    for index in range(len(outer_vertices)):
        first = outer_vertices[index]
        second = outer_vertices[(index + 1) % len(outer_vertices)]
        outer_edge_set.add(tuple(sorted((first, second))))
    covered_edges = set()
    for segment in (
        side_report["step_high_segments_3d_m"]
        + side_report["outer_drop_segments_3d_m"]
    ):
        first = tuple(round(float(value), 7) for value in segment[0])
        second = tuple(round(float(value), 7) for value in segment[1])
        edge = tuple(sorted((first, second)))
        require(
            edge not in covered_edges,
            side_name + " outer edge classified more than once",
        )
        covered_edges.add(edge)
    require(
        covered_edges == outer_edge_set,
        side_name + " step/drop segments cover all 25 outer edges once",
    )

negative_side = cad_report["negative_x"]
positive_side = cad_report["positive_x"]
negative_vertices = {
    tuple(round(float(value), 7) for value in vertex)
    for vertex in negative_side["planner_outer_boundary_vertices_3d_m"]
}
positive_vertices = {
    (-x, y, z)
    for x, y, z in (
        tuple(round(float(value), 7) for value in vertex)
        for vertex in positive_side["planner_outer_boundary_vertices_3d_m"]
    )
}
require(
    negative_vertices == positive_vertices,
    "negative/positive outer boundaries mirror in x",
)
require_close(
    negative_side["planner_outer_boundary_area_m2"],
    positive_side["planner_outer_boundary_area_m2"],
    "mirrored planner areas match",
    tol=0.0,
)
require(
    negative_side["selected_triangle_count"]
    == positive_side["selected_triangle_count"]
    and negative_side["step_high_segment_count"]
    == positive_side["step_high_segment_count"]
    and negative_side["outer_drop_segment_count"]
    == positive_side["outer_drop_segment_count"],
    "mirrored side counts match",
)

normals, vertices = analyze.read_binary_stl(XIAOLAN_MESH_PATH)
negative_extraction = analyze.extract_xiaolan_side(
    vertices,
    normals,
    xiaolan_cad["negative_x"],
)
plane_normal = negative_extraction.plane_normal
plane_offset = negative_extraction.plane_offset_m
sample_x = -0.18
sample_y = -0.0036
sample_z = (
    plane_offset
    - plane_normal[0] * sample_x
    - plane_normal[1] * sample_y
) / plane_normal[2]
sample_distances = analyze.xiaolan_plane_distance_report(
    negative_extraction,
    [sample_x, sample_y, sample_z],
)
require(
    sample_distances["inside_planner_polygon"],
    "user sample is inside the planner polygon",
)
require(
    math.isfinite(sample_distances["outer_drop_distance_m"])
    and sample_distances["outer_drop_distance_m"] > 0.0
    and math.isfinite(sample_distances["step_high_distance_m"])
    and sample_distances["step_high_distance_m"] > 0.0,
    "user sample has finite positive raw edge-class distances",
)

internal_uv = negative_extraction.internal_loops_2d_m[0]
internal_sample_x = -0.211
internal_sample_y = 0.04
internal_sample_z = (
    plane_offset
    - plane_normal[0] * internal_sample_x
    - plane_normal[1] * internal_sample_y
) / plane_normal[2]
projection_normal = negative_extraction.projection_normal
u_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
v_axis = np.cross(projection_normal, u_axis)
internal_sample_uv = np.array(
    [
        internal_sample_x * u_axis[0]
        + internal_sample_y * u_axis[1]
        + internal_sample_z * u_axis[2],
        internal_sample_x * v_axis[0]
        + internal_sample_y * v_axis[1]
        + internal_sample_z * v_axis[2],
    ],
    dtype=np.float64,
)
require(
    points_in_polygon(
        internal_sample_uv.reshape(1, 2),
        internal_uv,
    )[0],
    "test point is inside the ignored internal loop",
)
internal_sample_distances = analyze.xiaolan_plane_distance_report(
    negative_extraction,
    [internal_sample_x, internal_sample_y, internal_sample_z],
)
require(
    internal_sample_distances["inside_planner_polygon"],
    "point inside the ignored internal loop remains planner-inside",
)

outside_x = 0.0
outside_y = 0.0
outside_z = (
    plane_offset
    - plane_normal[0] * outside_x
    - plane_normal[1] * outside_y
) / plane_normal[2]
try:
    analyze.xiaolan_plane_distance_report(
        negative_extraction,
        [outside_x, outside_y, outside_z],
    )
    raise AssertionError(
        "M2 CAD validation failed: outside point must be rejected"
    )
except ValueError as exc:
    require("outside" in str(exc), "outside point rejection message")

bad_hash_config = json.loads(json.dumps(config))
bad_hash_config["known"][analyze.XIAOLAN_CAD_CONFIG_KEY]["stl_sha256"] = (
    "0" * 64
)
try:
    analyze.build_xiaolan_cad_report(bad_hash_config, XIAOLAN_MESH_PATH)
    raise AssertionError(
        "M2 CAD validation failed: corrupted hash must fail"
    )
except ValueError as exc:
    require("SHA-256" in str(exc), "corrupted hash failure message")

with tempfile.TemporaryDirectory() as temp_dir:
    malformed_path = Path(temp_dir) / "malformed.stl"
    malformed_path.write_bytes(
        b"\x00" * 84
        + struct.pack("<I", 2)
        + b"\x00" * 50
    )
    try:
        analyze.read_binary_stl(malformed_path)
        raise AssertionError(
            "M2 CAD validation failed: malformed STL metadata must fail"
        )
    except ValueError as exc:
        require("size mismatch" in str(exc), "malformed STL failure message")

missing_selector_config = json.loads(json.dumps(config))
del missing_selector_config["known"][
    analyze.XIAOLAN_CAD_CONFIG_KEY
]["negative_x"]
try:
    analyze.build_xiaolan_cad_report(
        missing_selector_config,
        XIAOLAN_MESH_PATH,
    )
    raise AssertionError(
        "M2 CAD validation failed: missing selector must fail"
    )
except ValueError as exc:
    require("negative_x" in str(exc), "missing selector failure message")

full_report = analyze.build_baseline_report(config, XIAOLAN_MESH_PATH)
json.dumps(full_report, allow_nan=False, sort_keys=True)
require(
    full_report["xiaolan_cad_geometry"]["status"]
    == "MODEL_GEOMETRY_READY",
    "full baseline CAD geometry status",
)
require(
    full_report["overall_status"] == "UNRESOLVED",
    "full baseline overall status remains UNRESOLVED",
)
reject_forbidden_claim_strings(full_report)


def reject_planning_claim_keys(value):
    forbidden_keys = {
        "ik",
        "trajectory",
        "safety_margin_m",
        "contact_threshold",
        "foot_target",
        "body_pose",
        "search_bounds",
        "certification",
        "authorization",
    }
    if isinstance(value, dict):
        for key, child in value.items():
            require(
                key.lower() not in forbidden_keys,
                "CAD report contains no planning/authorization claim key",
            )
            reject_planning_claim_keys(child)
    elif isinstance(value, list):
        for child in value:
            reject_planning_claim_keys(child)


reject_planning_claim_keys(cad_report)
require(
    "Raw CAD geometry extraction only" in cad_report["note"],
    "CAD report states raw-geometry-only scope",
)


# 路径碰撞只使用可视 STL，不使用较窄的 URDF 踝关节碰撞盒。
require(
    all(visual_collision.triangle_intersection_regressions()),
    "visual triangle intersection regressions including coplanar/tolerance cases",
)
visual_scene = visual_collision.default_visual_scene(
    ROOT
)
for mesh in (visual_scene.body_mesh, *visual_scene.link_meshes.values(), visual_scene.xiaolan_mesh):
    require(len(mesh.triangles) > 0, "visual STL has triangles: " + mesh.link_name)
    require(len(mesh.sha256) == 64, "visual STL hash: " + mesh.link_name)
    for component in mesh.components:
        subset = mesh.triangles[component.triangle_indices]
        require(
            np.all(subset >= component.bounds_min - TOL)
            and np.all(subset <= component.bounds_max + TOL),
            "visual component envelope contains triangles: " + mesh.link_name,
        )
require(
    len(visual_scene.link_meshes["ankle"].components) > 1,
    "ankle visual STL is component-split rather than replaced by a fat box",
)
ankle_mesh = visual_scene.link_meshes["ankle"]
footpad_definition = visual_collision.require_ankle_visual_footpad_binding(
    ankle_mesh
)
require(
    ankle_mesh.sha256
    == visual_collision.ANKLE_VISUAL_FOOTPAD_MESH_SHA256
    and footpad_definition.component_index
    == visual_collision.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX
    and visual_collision.ANKLE_VISUAL_FOOTPAD_SEMANTICS
    == "physical_terminal_foot_or_footpad_visual",
    "ankle visual footpad identity is bound to the audited mesh hash",
)
footpad_triangles = ankle_mesh.triangles[
    footpad_definition.triangle_indices
]
footpad_vertices = np.unique(
    np.round(
        footpad_triangles.reshape(-1, 3),
        visual_collision.ROUND_DECIMALS,
    ),
    axis=0,
)
require(
    len(footpad_triangles) == 1124 and len(footpad_vertices) == 564,
    "ankle visual footpad audited triangle and vertex counts",
)
require_close(
    footpad_definition.bounds_min,
    [0.10282626003026962, -0.08663489669561386, -0.006750310771167278],
    "ankle visual footpad audited local minimum",
    tol=2e-12,
)
require_close(
    footpad_definition.bounds_max,
    [0.12308488041162491, -0.06637626141309738, 0.006249630358070135],
    "ankle visual footpad audited local maximum",
    tol=2e-12,
)
foot_center_ankle = k.FOOT_OFFSET_ANKLE
foot_axis_ankle = foot_center_ankle / np.linalg.norm(foot_center_ankle)
footpad_distance = min(
    math.sqrt(
        visual_collision.point_triangle_distance_squared(
            foot_center_ankle, triangle
        )
    )
    for triangle in footpad_triangles
)
component_nine = next(
    component
    for component in ankle_mesh.components
    if component.component_index == 9
)
component_nine_distance = min(
    math.sqrt(
        visual_collision.point_triangle_distance_squared(
            foot_center_ankle, triangle
        )
    )
    for triangle in ankle_mesh.triangles[component_nine.triangle_indices]
)
relative_footpad = footpad_vertices - foot_center_ankle
footpad_axial = relative_footpad @ foot_axis_ankle
outward_radius = np.linalg.norm(
    relative_footpad[footpad_axial >= 0.0], axis=1
)
require(
    0.0 <= k.FOOT_RADIUS - footpad_distance < 5e-5
    and component_nine_distance > k.FOOT_RADIUS
    and np.max(np.abs(outward_radius - k.FOOT_RADIUS)) < 5e-6
    and np.min(footpad_axial) < -0.016,
    "ankle component 10 matches the 6.5 mm foot while component 9 does not overlap it",
)
world_from_base = np.eye(4, dtype=np.float64)
visual_components = visual_scene.robot_components(k.Q_STAND, world_from_base)
require(len(visual_components) > 0, "visual link transforms produce finite components")
for component in visual_components:
    require(np.all(np.isfinite(component.triangles)), "transformed visual triangles finite")
transformed_footpad = visual_scene.transformed_robot_component(
    k.Q_STAND,
    world_from_base,
    "ankle",
    0,
    visual_collision.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
)
filtered_footpad = next(
    component
    for component in visual_components
    if component.mesh.link_name == "ankle"
    and component.leg_index == 0
    and component.component.component_index
    == visual_collision.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX
)
require_component_fields = (
    transformed_footpad.mesh.sha256 == filtered_footpad.mesh.sha256
    and np.array_equal(
        transformed_footpad.component.triangle_indices,
        filtered_footpad.component.triangle_indices,
    )
    and np.array_equal(
        transformed_footpad.triangles, filtered_footpad.triangles
    )
)
require(
    require_component_fields,
    "exact transformed-component lookup equals full robot filtering",
)

# 排除脚垫时不能隐藏同腿的其他部件。
footpad_contact_mesh = visual_collision.visual_mesh_from_triangles(
    "xiaolan", transformed_footpad.triangles[192:193]
)
footpad_contact_scene = visual_collision.VisualCollisionScene(
    visual_scene.body_mesh,
    visual_scene.link_meshes,
    footpad_contact_mesh,
)
footpad_default_hit = footpad_contact_scene.components_vs_xiaolan(
    (transformed_footpad,)
)
footpad_exact_clear = footpad_contact_scene.components_vs_xiaolan(
    (transformed_footpad,),
    exclude_components=(("ankle", 0, 10),),
)
footpad_wrong_exclusion = footpad_contact_scene.components_vs_xiaolan(
    (transformed_footpad,),
    exclude_components=(("ankle", 0, 9),),
)
require(
    footpad_default_hit.collision
    and footpad_default_hit.hit.left_component == 10
    and not footpad_exact_clear.collision
    and footpad_wrong_exclusion.collision
    and footpad_contact_scene.component_xiaolan_triangle_hits(
        transformed_footpad
    )
    == (0,),
    "footpad default hit, exact component exclusion, and deterministic all-hit query",
)
default_footpad_cache = visual_planner.build_phase_cache(
    footpad_contact_scene,
    k.Q_STAND,
    world_from_base,
    (0,),
)
excluded_footpad_cache = visual_planner.build_phase_cache(
    footpad_contact_scene,
    k.Q_STAND,
    world_from_base,
    (0,),
    back_planner.VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS,
)
default_footpad_cached_result = visual_planner.cached_phase_visual(
    footpad_contact_scene,
    default_footpad_cache,
    k.Q_STAND,
    world_from_base,
)
excluded_footpad_cached_result = visual_planner.cached_phase_visual(
    footpad_contact_scene,
    excluded_footpad_cache,
    k.Q_STAND,
    world_from_base,
    back_planner.VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS,
)
require(
    not default_footpad_cached_result[0]
    and default_footpad_cached_result[1]["visual_hit"]["left_component"]
    == visual_collision.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX
    and excluded_footpad_cached_result == (True, {})
    and excluded_footpad_cache["xiaolan_component_exclusions"]
    == back_planner.VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS
    and back_planner._classified_surface_hits(
        footpad_contact_scene,
        "low",
        footpad_contact_scene.component_xiaolan_triangle_hits(
            transformed_footpad
        ),
        {"valid": True},
        frozenset((0,)),
        frozenset(),
    )[0],
    "front cache transfers only exact footpad contact to independent classification",
)
static_footpad_front_cache = visual_planner.build_phase_cache(
    footpad_contact_scene,
    k.Q_STAND,
    world_from_base,
    (1,),
    back_planner.VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS,
)
static_footpad_back_cache = (
    back_planner._cache_static_visual_footpad_hits(
        footpad_contact_scene,
        static_footpad_front_cache,
        k.Q_STAND,
        world_from_base,
    )
)
cached_static_hits = static_footpad_back_cache[
    back_planner.STATIC_VISUAL_FOOTPAD_HITS_CACHE_KEY
]
active_footpad = footpad_contact_scene.transformed_robot_component(
    k.Q_STAND,
    world_from_base,
    "ankle",
    1,
    visual_collision.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
)
require(
    set(cached_static_hits) == {0, 2, 3, 4, 5}
    and cached_static_hits[0]
    == footpad_contact_scene.component_xiaolan_triangle_hits(
        transformed_footpad
    )
    and 1 not in cached_static_hits
    and back_planner._visual_footpad_hits(
        footpad_contact_scene,
        static_footpad_back_cache,
        k.Q_STAND,
        world_from_base,
        1,
    )
    == footpad_contact_scene.component_xiaolan_triangle_hits(
        active_footpad
    ),
    "back cache stores only direct-equivalent static footpad hit tuples",
)
require(
    back_planner._classified_surface_hits(
        footpad_contact_scene,
        "low",
        cached_static_hits[0],
        {"valid": True},
        frozenset((0,)),
        frozenset(),
    )[0]
    and not back_planner._classified_surface_hits(
        footpad_contact_scene,
        None,
        cached_static_hits[0],
        {"valid": False},
        frozenset((0,)),
        frozenset(),
    )[0],
    "cached static footpad hits are reclassified for every phase policy",
)
transformed_component_nine = visual_scene.transformed_robot_component(
    k.Q_STAND, world_from_base, "ankle", 0, 9
)
component_nine_contact_mesh = visual_collision.visual_mesh_from_triangles(
    "xiaolan", transformed_component_nine.triangles[:1]
)
component_nine_scene = visual_collision.VisualCollisionScene(
    visual_scene.body_mesh,
    visual_scene.link_meshes,
    component_nine_contact_mesh,
)
require(
    component_nine_scene.components_vs_xiaolan(
        (transformed_component_nine,),
        exclude_components=(("ankle", 0, 10),),
    ).collision
    and component_nine_scene.components_vs_xiaolan(
        (transformed_component_nine,),
        exclude_components=back_planner.VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS,
    ).collision,
    "strict and fast-sim footpad exclusions leave component 9 as a hard hit",
)
coincident_visual_result = visual_scene.robot_vs_xiaolan(
    k.Q_STAND,
    world_from_base,
)
require(
    coincident_visual_result.collision
    and coincident_visual_result.hit is not None
    and coincident_visual_result.narrow_phase_tests > 0
    and coincident_visual_result.visual_narrow_phase_used,
    "robot-vs-Xiaolan collision is found by visual triangle narrow phase",
)
xiaolan_far = np.eye(4, dtype=np.float64)
xiaolan_far[:3, 3] = [0.0, 0.0, 10.0]
translated_visual_scene = visual_collision.default_visual_scene(
    ROOT,
    world_from_xiaolan=xiaolan_far,
)
require_close(
    translated_visual_scene.world_from_xiaolan,
    xiaolan_far,
    "Xiaolan world transform is stored explicitly",
)
require(
    not translated_visual_scene.robot_vs_xiaolan(k.Q_STAND, world_from_base).collision,
    "Xiaolan world translation changes visual collision result",
)

# 防止部件完全穿入小蓝却没有表面相交。
tetrahedron = np.array(
    [
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
    ],
    dtype=np.float64,
)
tetra_mesh = visual_collision.visual_mesh_from_triangles("xiaolan", tetrahedron)
contained_mesh = visual_collision.visual_mesh_from_triangles(
    "test_component",
    np.array([[[0.1, 0.1, 0.1], [0.2, 0.1, 0.1], [0.1, 0.2, 0.1]]]),
)
contained_component = visual_collision.TransformedComponent(
    contained_mesh,
    contained_mesh.components[0],
    None,
    contained_mesh.triangles,
    contained_mesh.components[0].bounds_min,
    contained_mesh.components[0].bounds_max,
)
containment_scene = visual_collision.VisualCollisionScene(
    contained_mesh,
    {},
    tetra_mesh,
    np.eye(4, dtype=np.float64),
)
require(
    visual_collision.point_in_closed_mesh(np.array([0.1, 0.1, 0.1]), tetra_mesh)
    and not visual_collision.point_in_closed_mesh(np.array([1.1, 0.1, 0.1]), tetra_mesh)
    and containment_scene.components_vs_xiaolan((contained_component,)).collision,
    "closed Xiaolan containment flags a fully enclosed visual component",
)

foot = visual_scene.foot_spheres(k.Q_STAND, world_from_base)[0]
foot_contact_mesh = visual_collision.visual_mesh_from_triangles(
    "xiaolan",
    np.array(
        [[
            foot.center + [-foot.radius_m, -foot.radius_m, -foot.radius_m],
            foot.center + [foot.radius_m, -foot.radius_m, -foot.radius_m],
            foot.center + [-foot.radius_m, foot.radius_m, -foot.radius_m],
        ]],
        dtype=np.float64,
    ),
)
foot_scene = visual_collision.VisualCollisionScene(
    visual_scene.body_mesh,
    visual_scene.link_meshes,
    foot_contact_mesh,
    np.eye(4, dtype=np.float64),
)
foot_hit = foot_scene.feet_vs_xiaolan(k.Q_STAND, world_from_base)
foot_clear_transform = np.eye(4, dtype=np.float64)
foot_clear_transform[2, 3] = 0.03
foot_clear_scene = visual_collision.VisualCollisionScene(
    visual_scene.body_mesh,
    visual_scene.link_meshes,
    foot_contact_mesh,
    foot_clear_transform,
)
require(
    foot_hit.collision
    and foot_hit.hit.left_link == "foot"
    and not foot_clear_scene.feet_vs_xiaolan(k.Q_STAND, world_from_base).collision
    and not foot_scene.feet_vs_xiaolan(
        k.Q_STAND,
        world_from_base,
        ignore_links=(("foot", foot.leg_index),),
    ).collision,
    "visual analytic foot sphere detects contact, separation, and exact ignore",
)
self_visual_result = visual_scene.self_collision(k.Q_STAND, world_from_base)
reference_self_visual_result = visual_scene.self_collision_reference(k.Q_STAND, world_from_base)
require(
    not self_visual_result.collision
    and self_visual_result.visual_narrow_phase_used,
    "Q_STAND visual self query excludes only connected mechanical neighbours",
)
require(
    self_visual_result == reference_self_visual_result,
    "vectorized visual self AABB sieve preserves literal-loop result and counts",
)
require(
    not hasattr(visual_collision, "URDF_COLLISION_GEOMETRY"),
    "visual path API exposes no URDF collision acceptance geometry",
)


# 检查规划记录在所有采样点都通过前保持 BLOCKED。
trace_path = SCRIPTS_DIR.parent / "config" / "climb_trace.json"
trace = json.loads(trace_path.read_text())
require(
    trace["schema"] == "SIMULATION_ONLY_VISUAL_MODEL_TRACE"
    and trace["status"] in {"MODEL_PATH_FOUND", "BLOCKED", "PARTIAL_STAGE_COMPLETE"}
    and trace["URDF_COLLISION_GEOMETRY_NOT_USED"]
    and trace["GROUND_GAIT_WORKSPACE_NOT_USED_AS_CLIMB_GATE"]
    and trace["REAL_CLEARANCE_UNRESOLVED"],
    "CC011 trace has model-only visual-CAD semantics",
)
require(trace["leg_order"] == list(k.LEG_NAMES), "CC011 trace leg order")
require_close(
    np.asarray(trace["world_from_xiaolan"]),
    np.array([[1.0, 0.0, 0.0, 0.45], [0.0, 1.0, 0.0, -0.03], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
    "CC011 Xiaolan world transform",
)
require(
    trace["low_surface"]["source"] == "CAD fitted low-platform plane and usable 2D polygon"
    and "selector_triangle_ids" not in trace["low_surface"],
    "CC011 low contact success is plane/polygon based, not raw triangle-id based",
)
require(
    trace["p0_preflight"]["base_pose_world"][0] == -0.08
    and trace["p0_preflight"]["clear"]
    and trace["fixed_poses_world"]["PREP"][0] == -0.06
    and trace["fixed_poses_world"]["PREP"][2:] == [0.15, -0.15]
    and trace["fixed_poses_world"]["BODY"][0] == -0.05
    and trace["fixed_poses_world"]["BODY"][2:] == [0.15, -0.05],
    "CC011 uses the one fixed P0/PREP/BODY pose contract",
)
requirements_by_name = {entry["name"]: entry for entry in trace["stage_requirements"]}
require(
    requirements_by_name["P0_TO_PREP"]["minimum_samples"] >= 31
    and requirements_by_name["RM_SWING"]["minimum_samples"] >= 41
    and requirements_by_name["PAIR_SWING"]["minimum_samples"] >= 41
    and requirements_by_name["PAIR_SWING"]["active_legs"] == ["rb", "rf"]
    and requirements_by_name["PAIR_SWING"]["synchronized_common_time"]
    and requirements_by_name["RM_SWING"]["waypoint_templates"] == 4
    and requirements_by_name["PAIR_SWING"]["waypoint_templates"] == 4
    and all(entry["visual_components_and_self_required"] for entry in requirements_by_name.values()),
    "CC011 trace requires dense visual checks and synchronized pair swing",
)
require(
    trace["prep_anchor_policy"] == "all six P0 football centers fixed in world for every PREP sample",
    "CC011 PREP preserves all six world anchors",
)
require(trace["verification_leaves_are_not_optimized_poses"], "CC011 records verification leaves separately from trajectory parameters")

# 防止节点位姿、关节角和固定足端坐标不一致。
def require_trace_node_transform(node_name):
    node = trace["nodes"].get(node_name)
    if node is None:
        return
    require(
        {"base_world", "pitch_rad", "q_rad", "anchors_world_m"}.issubset(node),
        "CC011 " + node_name + " node has pose/q/anchor receipt fields",
    )
    pose = visual_planner.pose_matrix(*node["base_world"], node["pitch_rad"])
    q_node = np.asarray(node["q_rad"], dtype=np.float64)
    anchors_node = np.asarray(node["anchors_world_m"], dtype=np.float64)
    require(q_node.shape == (6, 3) and anchors_node.shape == (6, 3), "CC011 " + node_name + " node shapes")
    require_close(
        visual_planner.world_points(pose, k.GraspKinematic().forward_base(q_node)),
        anchors_node,
        "CC011 " + node_name + " node pose matches transformed anchors",
        tol=2e-6,
    )


for _node_name in ("P0", "PREP", "RM", "BODY", "PAIR"):
    require_trace_node_transform(_node_name)

selected_low = trace["low_surface"]["selected"]
require(selected_low["outer_polygon_and_x_section_checked"], "CC011 low goals record outer-polygon and x-section check")
contact_local = np.asarray(selected_low["contact_points_local_m"], dtype=np.float64)
centre_local = np.asarray(selected_low["goals_local_m"], dtype=np.float64)
support_normal = np.asarray(selected_low.get("support_plane_normal_local", trace["low_surface"]["normal_local"]), dtype=np.float64)
require_close(centre_local - contact_local, k.FOOT_RADIUS * support_normal, "CC011 every low goal is offset from fitted low-platform plane", tol=2e-10)
require_close(contact_local @ support_normal, np.full(3, negative_extraction.plane_offset_m), "CC011 low contact projections lie on fitted plane", tol=2e-10)


def require_component_sequence_equal(left, right, label):
    require(len(left) == len(right), label + " count")
    for index, (first, second) in enumerate(zip(left, right)):
        require(
            first.mesh.link_name == second.mesh.link_name
            and first.leg_index == second.leg_index
            and first.component.component_index == second.component.component_index
            and np.array_equal(first.component.triangle_indices, second.component.triangle_indices)
            and np.array_equal(first.triangles, second.triangles)
            and np.array_equal(first.bounds_min, second.bounds_min)
            and np.array_equal(first.bounds_max, second.bounds_max),
            label + " component " + str(index),
        )


def require_cached_phase_equivalence(node_name, active_legs):
    node = trace["nodes"].get(node_name)
    if node is None:
        return
    pose = visual_planner.pose_matrix(*node["base_world"], node["pitch_rad"])
    q = np.asarray(node["q_rad"], dtype=np.float64)
    full_components = visual_scene.robot_components(q, pose)
    active_components = visual_scene.robot_components(q, pose, active_legs, include_body=False)
    expected_active = tuple(component for component in full_components if component.leg_index in active_legs)
    require_component_sequence_equal(active_components, expected_active, "CC011 selective " + node_name)
    cache = visual_planner.build_phase_cache(visual_scene, q, pose, active_legs)
    explicit_empty_cache = visual_planner.build_phase_cache(
        visual_scene, q, pose, active_legs, ()
    )
    require(
        cache["xiaolan_component_exclusions"] == ()
        and explicit_empty_cache["xiaolan_component_exclusions"] == ()
        and cache["static_xiaolan"]
        == explicit_empty_cache["static_xiaolan"]
        and visual_planner.cached_phase_visual(
            visual_scene, cache, q, pose
        )
        == visual_planner.cached_phase_visual(
            visual_scene, explicit_empty_cache, q, pose, ()
        ),
        "CC011 default-empty cache exclusion preserves old front behavior "
        + node_name,
    )
    full_component_collision = visual_scene.self_collision(q, pose).collision
    cached_component_collision = cache["static_self"].collision or visual_scene.cached_active_component_collision(cache["static_components"], active_components).collision
    require(full_component_collision == cached_component_collision, "CC011 cached component collision matches full " + node_name)
    all_feet = visual_scene.foot_spheres(q, pose)
    active_feet = tuple(all_feet[index] for index in active_legs)
    full_foot_collision = visual_scene.feet_vs_components(all_feet, full_components).collision
    static_foot_component = visual_scene.feet_vs_components(cache["static_feet"], cache["static_components"])
    require(
        cache["static_foot_components"] == static_foot_component,
        "CC011 phase cache retains its one-time static foot/component result " + node_name,
    )
    require(
        cache["static_foot_pair"] == visual_planner._first_foot_pair_collision(cache["static_feet"]),
        "CC011 phase cache retains its one-time static foot-pair result " + node_name,
    )
    cached_foot_collision = (
        cache["static_foot_components"].collision
        or visual_scene.feet_vs_components(cache["static_feet"], active_components).collision
        or visual_scene.feet_vs_components(active_feet, cache["static_components"] + active_components).collision
    )
    require(full_foot_collision == cached_foot_collision, "CC011 cached bidirectional foot/component collision matches full " + node_name)


# 检查 RM 和双腿同步阶段的静态/活动部件缓存。
require_cached_phase_equivalence("PREP", (visual_planner.RM,))
require_cached_phase_equivalence("BODY", (visual_planner.RB, visual_planner.RF))

stage_by_name = {entry["name"]: entry for entry in trace["stages"]}
if trace["status"] == "MODEL_PATH_FOUND":
    for _name, _minimum in (("P0_TO_PREP", 31), ("RM_SWING", 41), ("BODY_TRANSFER", 31), ("PAIR_SWING", 41)):
        require(_name in stage_by_name, "CC011 successful trace contains " + _name)
        require(stage_by_name[_name]["visual_sample_count"] >= _minimum, "CC011 " + _name + " records all dense visual samples")
    require(all(trace["nodes"][name] is not None for name in ("P0", "PREP", "RM", "BODY", "PAIR")), "CC011 successful trace has all endpoint nodes")
    require(stage_by_name["PAIR_SWING"]["synchronized_common_time"] and trace["nodes"]["PAIR"]["synchronized_common_time"], "CC011 pair receipt is synchronized")
    for leg in ("rm", "rb", "rf"):
        node_name = "RM" if leg == "rm" else "PAIR"
        contact = trace["nodes"][node_name]["goal_contact"][leg]
        require(contact["plane_error_m"] <= 1e-8 and contact["polygon_inside"], "CC011 " + leg + " endpoint reaches fitted low plane")
elif trace["status"] == "BLOCKED":
    require("blocked" in trace and trace["blocked"].get("earliest_witness") is not None, "CC011 BLOCKED trace preserves earliest layered rejection witness")
else:
    require(trace["status"] == "PARTIAL_STAGE_COMPLETE", "CC011 non-success trace is an explicit stop receipt")
    require(trace["stop_after"] in {"PREP", "RM", "BODY"}, "CC011 partial receipt has a non-pair stop stage")
planner_config = json.loads(config_path.read_text())
_triangles, planner_extraction, planner_normal, _ids = visual_planner.selector_geometry(planner_config)
low_triangle_ids = frozenset(int(index) for index in _ids)
top_triangle_ids = back_planner._top_triangle_ids(visual_scene)
low_probe = next(iter(low_triangle_ids))
top_probe = min(top_triangle_ids)
side_probe = next(
    index
    for index, triangle in enumerate(visual_scene.xiaolan_mesh_local.triangles)
    if index not in low_triangle_ids
    and index not in top_triangle_ids
    and abs(
        np.cross(
            triangle[1] - triangle[0], triangle[2] - triangle[0]
        )[2]
    )
    < 0.5
    * np.linalg.norm(
        np.cross(
            triangle[1] - triangle[0], triangle[2] - triangle[0]
        )
    )
)
valid_contact_probe = {"valid": True}
require(
    back_planner._classified_surface_hits(
        visual_scene,
        "low",
        (low_probe,),
        valid_contact_probe,
        low_triangle_ids,
        top_triangle_ids,
    )[0]
    and not back_planner._classified_surface_hits(
        visual_scene,
        None,
        (low_probe,),
        {"valid": False},
        low_triangle_ids,
        top_triangle_ids,
    )[0]
    and not back_planner._classified_surface_hits(
        visual_scene,
        "low",
        (side_probe,),
        valid_contact_probe,
        low_triangle_ids,
        top_triangle_ids,
    )[0]
    and not back_planner._classified_surface_hits(
        visual_scene,
        "low",
        (-1,),
        valid_contact_probe,
        low_triangle_ids,
        top_triangle_ids,
    )[0],
    "back visual footpad low contact accepts only classified selector facets",
)
require(
    back_planner._classified_surface_hits(
        visual_scene,
        "cad",
        (top_probe,),
        valid_contact_probe,
        low_triangle_ids,
        top_triangle_ids,
    )[0]
    and not back_planner._classified_surface_hits(
        visual_scene,
        "cad",
        (side_probe,),
        valid_contact_probe,
        low_triangle_ids,
        top_triangle_ids,
    )[0],
    "back CAD contact accepts only upward support-like facets",
)
cad_anchor_start = np.zeros((6, 3), dtype=np.float64)
cad_anchor_start[back_planner.RM] = [0.1, -0.1, 0.19]
cad_anchor_target = cad_anchor_start.copy()
cad_anchor_target[back_planner.RM] = [0.2, -0.1, 0.19]
cad_stage = back_planner._make_stage(
    "CAD_TARGET_PROBE",
    np.zeros(5),
    np.zeros(5),
    (
        cad_anchor_start,
        cad_anchor_start,
        cad_anchor_target,
        cad_anchor_target,
    ),
    (back_planner.RM,),
    tuple(range(5)),
    {back_planner.RM: "cad"},
    {back_planner.RM: "cad"},
)
cad_at_start = back_planner._stage_contact_state(
    "cad",
    cad_anchor_start[back_planner.RM],
    cad_stage,
    back_planner.RM,
    planner_extraction,
    planner_normal,
    back_planner._top_triangles(visual_scene),
    back_planner._visual_contact_expected_anchor(
        cad_stage, back_planner.RM, 0.0
    ),
)
cad_start_against_final = back_planner._stage_contact_state(
    "cad",
    cad_anchor_start[back_planner.RM],
    cad_stage,
    back_planner.RM,
    planner_extraction,
    planner_normal,
    back_planner._top_triangles(visual_scene),
)
cad_at_target = back_planner._stage_contact_state(
    "cad",
    cad_anchor_target[back_planner.RM],
    cad_stage,
    back_planner.RM,
    planner_extraction,
    planner_normal,
    back_planner._top_triangles(visual_scene),
)
cad_before_target = back_planner._stage_contact_state(
    "cad",
    cad_anchor_target[back_planner.RM] + [2e-6, 0.0, 0.0],
    cad_stage,
    back_planner.RM,
    planner_extraction,
    planner_normal,
    back_planner._top_triangles(visual_scene),
)
require(
    cad_at_start["valid"]
    and not cad_start_against_final["valid"]
    and cad_at_target["valid"]
    and not cad_before_target["valid"]
    and back_planner._allowed_contacts(cad_stage, 1e-12)[back_planner.RM]
    == "cad"
    and back_planner.RM
    not in back_planner._allowed_contacts(cad_stage, 2e-12)
    and back_planner.RM not in back_planner._allowed_contacts(cad_stage, 1.5)
    and back_planner._allowed_contacts(cad_stage, 2.5)[back_planner.RM]
    == "cad",
    "back CAD policy uses the fixed start only at t=0 and final thereafter",
)
cad_static_stage = back_planner._make_stage(
    "CAD_STATIC_PROBE",
    np.zeros(5),
    np.zeros(5),
    (cad_anchor_target, cad_anchor_target),
    (),
    tuple(range(6)),
    {back_planner.RM: "cad"},
    {back_planner.RM: "cad"},
)
cad_static_start = back_planner._stage_contact_state(
    "cad",
    cad_anchor_target[back_planner.RM],
    cad_static_stage,
    back_planner.RM,
    planner_extraction,
    planner_normal,
    back_planner._top_triangles(visual_scene),
    back_planner._visual_contact_expected_anchor(
        cad_static_stage, back_planner.RM, 0.0
    ),
)
cad_static_terminal = back_planner._stage_contact_state(
    "cad",
    cad_anchor_target[back_planner.RM],
    cad_static_stage,
    back_planner.RM,
    planner_extraction,
    planner_normal,
    back_planner._top_triangles(visual_scene),
)
require(
    cad_static_start["valid"] and cad_static_terminal["valid"],
    "back static CAD contact remains valid at start and terminal",
)
planner_goal = visual_planner.low_goals(planner_extraction, planner_normal)
planner_y = planner_goal["robot_center_y_local_m"] - 0.03
planner_model = k.GraspKinematic()
planner_p0 = visual_planner.pose_matrix(-0.08, planner_y, visual_planner.P0_Z, 0.0)
planner_anchors = visual_planner.world_points(planner_p0, planner_model.forward_base(k.Q_STAND))
planner_end = visual_planner.pose_matrix(-0.06, planner_y, 0.15, -0.15)
planner_q, planner_residual = visual_planner.solve_anchors(
    planner_model,
    planner_anchors,
    planner_end,
    k.Q_STAND,
)
require(
    planner_residual <= 1e-6
    and np.min(planner_model.joint_limit_margins(planner_q)) > 0.221,
    "CC011 DLS uses hip-frame error for known PREP endpoint",
)


# 后半段记录从 PAIR 节点开始，不能改动前半段记录。
back_path = SCRIPTS_DIR.parent / "config" / "climb_back_trace.json"
back = json.loads(back_path.read_text())
require(
    back["schema"] == "SIMULATION_ONLY_BACK_HALF_VISUAL_MODEL_TRACE_V1"
    and back["simulation_only"]
    and back["model_only_not_contact_or_load_proof"],
    "back trace simulation-only schema",
)
require(
    back["source_trace"]["path"] == "climb_trace.json"
    and back["source_trace"]["required_node"] == "PAIR"
    and back["source_trace"]["sha256"]
    == hashlib.sha256(trace_path.read_bytes()).hexdigest(),
    "back trace binds exact source PAIR trace",
)
require(
    back["mesh_sha256"]["xiaolan"]
    == hashlib.sha256(XIAOLAN_MESH_PATH.read_bytes()).hexdigest(),
    "back trace binds Xiaolan STL",
)
require(
    back["URDF_COLLISION_GEOMETRY_NOT_USED"]
    and back["CONTROLLER_CAPSULE_COLLISION_DIAGNOSTIC_ONLY"]
    and back["fixed_CAD_target_is_planned_contact_not_contact_proof"]
    and back["REAL_CLEARANCE_UNRESOLVED"],
    "back trace preserves collision authority and real-clearance boundary",
)
require(
    back["rotation_order"] == "T_Ry_pitch_Rx_roll"
    and back["verification_leaves_are_not_optimized_poses"],
    "back trace rotation and verification-leaf semantics",
)

# 检查横滚和俯仰角的旋转顺序。
rotation_probe = np.array((0.2, -0.1, 0.3, 0.17, -0.23))
probe_transform = back_planner.pose_matrix(rotation_probe)
cr, sr = np.cos(rotation_probe[3]), np.sin(rotation_probe[3])
cp, sp = np.cos(rotation_probe[4]), np.sin(rotation_probe[4])
expected_rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
expected_ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
require_close(
    probe_transform[:3, :3],
    expected_ry @ expected_rx,
    "back trace T Ry pitch Rx roll rotation order",
)
require_close(
    probe_transform[:3, 3],
    rotation_probe[:3],
    "back trace pose translation",
)

back_stages = back_planner.fixed_stages(trace, planner_extraction, planner_normal)
require(
    tuple(stage["name"] for stage in back_stages) == back_planner.STAGE_NAMES
    and back["configured_stage_order"] == list(back_planner.STAGE_NAMES),
    "back trace fixed stage order has no search candidates",
)


def validate_sim_trace_receipt(
    candidate,
    strict_bytes,
    source_bytes,
    strict_trace,
    scene,
    all_stages,
    model,
):
    selected = all_stages[-2:]
    expected_names = tuple(stage["name"] for stage in selected)
    if expected_names != sim_finish.SIM_STAGE_NAMES:
        raise ValueError("sim fixed stage suffix mismatch")
    if (
        candidate.get("schema")
        != "SIMULATION_ONLY_BACK_HALF_FAST_FINISH_V1"
        or candidate.get("status")
        not in {"SIM_CANDIDATE_PATH_FOUND", "BLOCKED"}
        or candidate.get("simulation_candidate_only") is not True
        or candidate.get("dense_visual_all_leaves") is not False
        or candidate.get("visual_validation_deferred_to_isaac") is not True
        or candidate.get("strict_prefix_stop") != "B1"
        or candidate.get("configured_stage_order")
        != list(sim_finish.SIM_STAGE_NAMES)
        or candidate.get("controller_analytic_guards_diagnostic_only")
        is not True
        or candidate.get("sim_contact_engineering_tolerance_m")
        != sim_finish.SIM_CONTACT_ENGINEERING_TOLERANCE_M
    ):
        raise ValueError("sim trace schema or authority boundary mismatch")
    source_strict = candidate.get("source_strict_trace", {})
    if (
        source_strict.get("path") != "climb_back_trace.json"
        or source_strict.get("sha256")
        != hashlib.sha256(strict_bytes).hexdigest()
        or source_strict.get("status") != "PARTIAL_STAGE_COMPLETE"
        or source_strict.get("stop_after") != "B1"
        or strict_trace.get("status") != "PARTIAL_STAGE_COMPLETE"
        or strict_trace.get("stop_after") != "B1"
        or candidate.get("source_front_trace_sha256")
        != hashlib.sha256(source_bytes).hexdigest()
    ):
        raise ValueError("sim strict-prefix binding mismatch")
    expected_thresholds = back_planner._trace_contract(
        source_bytes, scene, all_stages
    )["thresholds"]
    if candidate.get("numeric_thresholds") != expected_thresholds:
        raise ValueError("sim numeric threshold mismatch")
    diagnostics = candidate.get("deferred_visual_diagnostics")
    if diagnostics != list(sim_finish.DEFERRED_VISUAL_DIAGNOSTICS):
        raise ValueError("sim deferred visual provenance mismatch")
    intended = diagnostics[1]
    if (
        intended["plane_error_m"]
        >= sim_finish.SIM_CONTACT_ENGINEERING_TOLERANCE_M
        or intended["within_sim_contact_engineering_tolerance"] is not True
        or intended["not_rechecked_in_this_run"] is not True
        or diagnostics[0]["not_rechecked_in_this_run"] is not True
    ):
        raise ValueError("sim deferred contact diagnostic mismatch")

    fixed = candidate.get("fixed_endpoints")
    if not isinstance(fixed, dict) or set(fixed) != set(expected_names):
        raise ValueError("sim fixed endpoint keys mismatch")
    nodes = candidate.get("nodes")
    if (
        not isinstance(nodes, dict)
        or nodes.get("B1") != strict_trace["nodes"]["B1"]
    ):
        raise ValueError("sim B1 source node mismatch")
    records = candidate.get("stages")
    if not isinstance(records, list) or not records or len(records) > 2:
        raise ValueError("sim stage records mismatch")

    maximum = {
        "max_ik_residual_m": back_planner.MAX_RESIDUAL_M,
        "max_adjacent_joint_delta_rad": back_planner.MAX_JOINT_STEP_RAD,
        "max_active_foot_step_m": back_planner.MAX_FOOT_STEP_M,
        "max_base_translation_step_m": back_planner.MAX_BASE_STEP_M,
        "max_roll_step_rad": back_planner.MAX_ANGLE_STEP_RAD,
        "max_pitch_step_rad": back_planner.MAX_ANGLE_STEP_RAD,
    }
    minimum = {
        "min_joint_margin_rad": back_planner.MIN_JOINT_MARGIN_RAD,
        "min_support_margin_m": back_planner.MIN_SUPPORT_MARGIN_M,
    }
    expected_metric_keys = {*maximum, *minimum, "min_sigma"}
    clear_count = 0
    for index, record in enumerate(records):
        stage = selected[index]
        endpoint = fixed[stage["name"]]
        count = record.get("numeric_leaf_count")
        metrics = record.get("metrics")
        if (
            record.get("name") != stage["name"]
            or record.get("minimum_samples") != stage["minimum_samples"]
            or not isinstance(count, int)
            or isinstance(count, bool)
            or count < stage["minimum_samples"]
            or count > back_planner.MAX_SAMPLES
            or record.get("numeric_hard_gates_all_leaves") is not True
            or record.get("analytic_guard_is_diagnostic_not_path_authority")
            is not True
            or record.get("dense_visual_all_leaves") is not False
            or record.get("visual_validation_deferred_to_isaac") is not True
            or not isinstance(metrics, dict)
            or set(metrics) != expected_metric_keys
            or not np.allclose(
                record.get("pose_start"),
                stage["pose_start"],
                rtol=0.0,
                atol=1e-12,
            )
            or not np.allclose(
                record.get("pose_end"),
                stage["pose_end"],
                rtol=0.0,
                atol=1e-12,
            )
            or not np.allclose(
                record.get("anchor_knots_world_m"),
                stage["anchor_knots"],
                rtol=0.0,
                atol=1e-12,
            )
            or not np.allclose(
                endpoint.get("pose_world"),
                stage["pose_end"],
                rtol=0.0,
                atol=1e-12,
            )
            or not np.allclose(
                endpoint.get("anchors_world_m"),
                stage["anchor_knots"][-1],
                rtol=0.0,
                atol=1e-12,
            )
            or not np.allclose(
                endpoint.get("anchor_knots_world_m"),
                stage["anchor_knots"],
                rtol=0.0,
                atol=1e-12,
            )
        ):
            raise ValueError("sim stage fixed numeric receipt mismatch")
        for name, value in metrics.items():
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError("sim stage metric is invalid")
        if any(metrics[name] > limit + 1e-12 for name, limit in maximum.items()):
            raise ValueError("sim stage maximum metric failed")
        if any(metrics[name] < limit - 1e-12 for name, limit in minimum.items()):
            raise ValueError("sim stage minimum metric failed")
        runs = record.get("analytic_guard_false_runs")
        if (
            not isinstance(runs, list)
            or record.get("analytic_guard_all_clear") is not (not runs)
        ):
            raise ValueError("sim analytic diagnostic mismatch")
        if record.get("clear") is True:
            clear_count += 1
            node = nodes.get(stage["name"])
            if not isinstance(node, dict):
                raise ValueError("sim clear endpoint node missing")
            pose = np.asarray(node.get("pose_world"), dtype=np.float64)
            q_value = np.asarray(node.get("q_rad"), dtype=np.float64)
            anchors = np.asarray(node.get("anchors_world_m"), dtype=np.float64)
            if (
                pose.shape != (5,)
                or q_value.shape != (6, 3)
                or anchors.shape != (6, 3)
                or not np.allclose(
                    pose, stage["pose_end"], rtol=0.0, atol=1e-12
                )
                or not np.allclose(
                    anchors,
                    stage["anchor_knots"][-1],
                    rtol=0.0,
                    atol=1e-12,
                )
                or not np.allclose(
                    q_value,
                    record.get("q_end_rad"),
                    rtol=0.0,
                    atol=1e-12,
                )
                or np.max(
                    np.abs(
                        back_planner.world_points(
                            back_planner.pose_matrix(pose),
                            model.forward_base(q_value),
                        )
                        - anchors
                    )
                )
                > 1e-6
            ):
                raise ValueError("sim endpoint FK receipt mismatch")
        elif index != len(records) - 1:
            raise ValueError("sim failed stage must be final")

    if candidate["status"] == "SIM_CANDIDATE_PATH_FOUND":
        if (
            clear_count != 2
            or len(records) != 2
            or set(nodes) != {"B1", *sim_finish.SIM_STAGE_NAMES}
            or "blocked" in candidate
        ):
            raise ValueError("sim candidate success receipt mismatch")
    else:
        failed = records[-1]
        blocked = candidate.get("blocked")
        if (
            failed.get("clear") is not False
            or clear_count != len(records) - 1
            or blocked
            != {
                "stage": failed["name"],
                "reason": failed["reason"],
                "earliest_witness": failed["witness"],
            }
        ):
            raise ValueError("sim BLOCKED witness mismatch")


rm_b_stage = back_stages[back_planner.STAGE_NAMES.index("RM_B")]
rm_b_geometry = rm_b_stage["trajectory_geometry"]
require_close(
    rm_b_stage["anchor_knots"][:, back_planner.RM],
    np.array(
        (
            (0.42, -0.13, 0.1965),
            (0.465, -0.13, 0.2065),
            (0.51, -0.13, 0.1965),
        )
    ),
    "back RM_B fixed two-segment low-arc knots",
)
rm_b_other_legs = [
    leg for leg in range(6) if leg != back_planner.RM
]
require(
    rm_b_stage["segments"] == 2
    and np.array_equal(rm_b_stage["pose_start"], rm_b_stage["pose_end"])
    and np.array_equal(
        rm_b_stage["anchor_knots"][:, rm_b_other_legs],
        np.broadcast_to(
            rm_b_stage["anchor_knots"][0, rm_b_other_legs],
            rm_b_stage["anchor_knots"][:, rm_b_other_legs].shape,
        ),
    )
    and rm_b_geometry["curve"]
    == "two_segment_piecewise_quintic_smoothstep"
    and rm_b_geometry["segment_count"] == 2
    and rm_b_geometry["active_leg"] == "rm"
    and rm_b_geometry["base_pose_fixed"] is True
    and rm_b_geometry["other_anchors_fixed"] is True,
    "back RM_B cannot fall back to the generic lift template",
)
require_close(
    np.asarray(rm_b_geometry["anchor_knots_world_m"]),
    rm_b_stage["anchor_knots"],
    "back RM_B trajectory receipt binds every anchor knot",
)
lf_c1_stage = back_stages[back_planner.STAGE_NAMES.index("LF_C1")]
lf_c1_geometry = lf_c1_stage["trajectory_geometry"]
require_close(
    lf_c1_stage["pose_end"],
    [
        0.23,
        float(trace["nodes"]["PAIR"]["base_world"][1]) - 0.03,
        0.19,
        0.10,
        -0.20,
    ],
    "back LF_C1 fixed C2 endpoint pose",
)
require_close(
    lf_c1_stage["anchor_knots"][-1, back_planner.LF],
    [0.19864857479269682, 0.12028708155300927, 0.15273468388201647],
    "back LF_C1 fixed low target remains unchanged",
    tol=2e-10,
)
require(
    np.array_equal(
        lf_c1_stage["anchor_knots"][-1, [0, 2, 3, 4, 5]],
        lf_c1_stage["anchor_knots"][0, [0, 2, 3, 4, 5]],
    )
    and lf_c1_stage["segments"] == 3
    and lf_c1_geometry["anchor_curve"]
    == "three_segment_piecewise_quintic_smoothstep"
    and lf_c1_geometry["pose_curve"]
    == "single_quintic_smoothstep_over_full_stage"
    and lf_c1_geometry["segment_count"] == 3
    and lf_c1_geometry["active_leg"] == "lf"
    and lf_c1_geometry["final_anchors_unchanged_by_pose_fix"] is True,
    "back LF_C1 retains the existing generator and all other anchors",
)
require_close(
    np.asarray(lf_c1_geometry["pose_end_world"]),
    lf_c1_stage["pose_end"],
    "back LF_C1 trajectory receipt binds final pose",
)
require_close(
    np.asarray(lf_c1_geometry["anchor_knots_world_m"]),
    lf_c1_stage["anchor_knots"],
    "back LF_C1 trajectory receipt binds unchanged anchors",
)

# 继续检查只接受当前记录允许的失败形态。
back_resume_scene = visual_collision.default_visual_scene(
    ROOT, back_planner.WORLD_FROM_XIAOLAN
)
sim_trace_path = (
    SCRIPTS_DIR.parent
    / "config"
    / "climb_back_sim_trace.json"
)
sim_trace = json.loads(sim_trace_path.read_text())
validate_sim_trace_receipt(
    sim_trace,
    back_path.read_bytes(),
    trace_path.read_bytes(),
    back,
    back_resume_scene,
    back_stages,
    k.GraspKinematic(),
)
require(
    sim_trace["status"] in {"SIM_CANDIDATE_PATH_FOUND", "BLOCKED"}
    and sim_trace["status"] != "MODEL_PATH_FOUND"
    and sim_trace["dense_visual_all_leaves"] is False
    and sim_trace["simulation_candidate_only"] is True,
    "fast finish cannot masquerade as a strict dense model path",
)


def require_sim_trace_rejected(name, mutate):
    candidate = copy.deepcopy(sim_trace)
    mutate(candidate)
    try:
        validate_sim_trace_receipt(
            candidate,
            back_path.read_bytes(),
            trace_path.read_bytes(),
            back,
            back_resume_scene,
            back_stages,
            k.GraspKinematic(),
        )
    except (ValueError, KeyError, TypeError):
        return
    raise AssertionError("M1A validation failed: sim accepted " + name)


require_sim_trace_rejected(
    "MODEL_PATH_FOUND status",
    lambda value: value.__setitem__("status", "MODEL_PATH_FOUND"),
)
require_sim_trace_rejected(
    "changed strict-prefix hash",
    lambda value: value["source_strict_trace"].__setitem__(
        "sha256", "0" * 64
    ),
)
require_sim_trace_rejected(
    "changed fixed D4 endpoint",
    lambda value: value["fixed_endpoints"]["D4_LB_LOW"][
        "pose_world"
    ].__setitem__(0, 9.0),
)
require_sim_trace_rejected(
    "numeric leaf count below minimum",
    lambda value: value["stages"][0].__setitem__(
        "numeric_leaf_count", 1
    ),
)
require_sim_trace_rejected(
    "pretended dense visual validation",
    lambda value: value.__setitem__("dense_visual_all_leaves", True),
)
require_sim_trace_rejected(
    "changed deferred visual diagnostic",
    lambda value: value["deferred_visual_diagnostics"][1].__setitem__(
        "plane_error_m", 0.0
    ),
)
rm_high_stage = back_stages[back_planner.STAGE_NAMES.index("RM_HIGH_C")]
rm_geometry = rm_high_stage["trajectory_geometry"]
require(
    back_planner.RM_HIGH_C_LIFT == 0.02
    and back_planner.RM_HIGH_C_TARGET_FACET_ID == 18028
    and rm_geometry["lift_m"] == back_planner.RM_HIGH_C_LIFT
    and rm_geometry["target_upward_facet_id"]
    == back_planner.RM_HIGH_C_TARGET_FACET_ID,
    "back RM HIGH_C fixed normal-path constants",
)
require_close(
    np.asarray(rm_geometry["source_low_surface_normal_world"]),
    planner_normal,
    "back RM HIGH_C source low-surface normal",
)
require_close(
    np.asarray(rm_geometry["target_upward_normal_world"]),
    back_planner.RM_HIGH_C_TARGET_NORMAL,
    "back RM HIGH_C audited target-facet normal",
)
require_close(
    np.asarray(rm_geometry["anchor_knots_world_m"]),
    rm_high_stage["anchor_knots"],
    "back RM HIGH_C receipt geometry binds all four knots",
)
target_triangle = back_resume_scene.xiaolan_mesh_local.triangles[
    back_planner.RM_HIGH_C_TARGET_FACET_ID
]
target_normal = np.cross(
    target_triangle[1] - target_triangle[0],
    target_triangle[2] - target_triangle[0],
)
target_normal /= np.linalg.norm(target_normal)
if target_normal[2] < 0.0:
    target_normal *= -1.0
require_close(
    target_normal,
    back_planner.RM_HIGH_C_TARGET_NORMAL,
    "back RM HIGH_C triangle 18028 oriented upward normal",
)
target_center_local = back_planner.base_points(
    back_planner.WORLD_FROM_XIAOLAN,
    rm_high_stage["anchor_knots"][-1, back_planner.RM][None],
)[0]
target_contact_local = (
    target_center_local - k.FOOT_RADIUS * target_normal
)
require(
    visual_collision.point_triangle_distance_squared(
        target_contact_local, target_triangle
    ) <= 1e-12,
    "back RM HIGH_C target center minus radius normal reaches triangle 18028",
)
require(
    back_planner._allowed_contacts(rm_high_stage, 1e-12)[back_planner.RM]
    == "low"
    and back_planner.RM
    not in back_planner._allowed_contacts(rm_high_stage, 2e-12),
    "back RM HIGH_C permits the source contact only at t=0",
)

retry_back = back_planner._prepare_resume_artifact(back, back_stages)
resume_clear_count = len(retry_back["stages"])
resume_requested = (
    back_planner.STAGE_NAMES[resume_clear_count]
    if resume_clear_count < len(back_planner.STAGE_NAMES)
    else "FULL"
)
resume_last_name = back_planner.STAGE_NAMES[resume_clear_count - 1]
expected_failed_history = copy.deepcopy(back.get("failed_attempt_history", []))
if back["status"] == "BLOCKED":
    expected_failed_history.append({
        "stage_record": back["stages"][-1],
        "blocked": back["blocked"],
    })
resume_state, resume_count, resume_elapsed = back_planner._resume_preflight(
    retry_back,
    resume_requested,
    trace_path.read_bytes(),
    back_resume_scene,
    back_stages,
    k.GraspKinematic(),
)
require(
    resume_count == resume_clear_count
    and back_stages[resume_count - 1]["name"] == resume_last_name
    and resume_elapsed == back["planner_elapsed_s"]
    and retry_back["status"] == "PARTIAL_STAGE_COMPLETE"
    and "blocked" not in retry_back
    and retry_back.get("failed_attempt_history", [])
    == expected_failed_history
    and retry_back["fixed_stage_trajectory_geometry"]
    == back_planner._fixed_stage_trajectory_geometry(back_stages)
    and retry_back["fixed_targets"]
    == back_planner._fixed_targets(back_stages)
    and np.array_equal(
        resume_state["q"], np.asarray(back["nodes"][resume_last_name]["q_rad"])
    )
    and np.array_equal(
        resume_state["anchors"],
        np.asarray(back["nodes"][resume_last_name]["anchors_world_m"]),
    ),
    "back current clear prefix passes resume artifact and FK preflight",
)

# 只检查已确认的数值和支撑结果，不运行可视碰撞。
rm_b_model = k.GraspKinematic()
rm_b_samples = back_planner._adaptive_samples(
    rm_b_model,
    np.asarray(back["nodes"]["BODY_A"]["q_rad"]),
    rm_b_stage,
)
rm_b_max_residual = max(sample["residual_m"] for sample in rm_b_samples)
rm_b_min_joint_margin = min(
    float(np.min(rm_b_model.joint_limit_margins(sample["q_rad"])))
    for sample in rm_b_samples
)
rm_b_min_support_margin = math.inf
rm_b_max_joint_step = 0.0
rm_b_max_foot_step = 0.0
for index, sample in enumerate(rm_b_samples):
    world_com = back_planner.world_points(
        sample["transform"],
        rm_b_model.center_of_mass_base(sample["q_rad"])[None],
    )[0]
    support = climb.gravity_projected_support(
        world_com,
        sample["anchors"][list(rm_b_stage["support_legs"])],
        np.array((0.0, 0.0, -9.81)),
    )
    rm_b_min_support_margin = min(
        rm_b_min_support_margin, float(support.raw_margin_m)
    )
    if index:
        previous = rm_b_samples[index - 1]
        rm_b_max_joint_step = max(
            rm_b_max_joint_step,
            float(np.max(np.abs(sample["q_rad"] - previous["q_rad"]))),
        )
        rm_b_max_foot_step = max(
            rm_b_max_foot_step,
            float(
                np.linalg.norm(
                    sample["anchors"][back_planner.RM]
                    - previous["anchors"][back_planner.RM]
                )
            ),
        )
numeric_tolerance = 1e-12
require(
    len(rm_b_samples) == 45
    and rm_b_max_residual <= back_planner.MAX_RESIDUAL_M + numeric_tolerance
    and rm_b_max_joint_step
    <= back_planner.MAX_JOINT_STEP_RAD + numeric_tolerance
    and rm_b_max_foot_step
    <= back_planner.MAX_FOOT_STEP_M + numeric_tolerance
    and rm_b_min_joint_margin
    >= back_planner.MIN_JOINT_MARGIN_RAD - numeric_tolerance
    and rm_b_min_support_margin
    >= back_planner.MIN_SUPPORT_MARGIN_M - numeric_tolerance,
    "back RM_B fixed low arc has 45 numeric verification leaves",
)

lf_c1_samples = back_planner._adaptive_samples(
    rm_b_model,
    np.asarray(back["nodes"]["RM_B"]["q_rad"]),
    lf_c1_stage,
)
lf_c1_max_residual = 0.0
lf_c1_max_joint_step = 0.0
lf_c1_max_foot_step = 0.0
lf_c1_max_base_step = 0.0
lf_c1_max_roll_step = 0.0
lf_c1_max_pitch_step = 0.0
lf_c1_min_joint_margin = math.inf
lf_c1_min_support_margin = math.inf
for index, sample in enumerate(lf_c1_samples):
    lf_c1_max_residual = max(
        lf_c1_max_residual, float(sample["residual_m"])
    )
    lf_c1_min_joint_margin = min(
        lf_c1_min_joint_margin,
        float(np.min(rm_b_model.joint_limit_margins(sample["q_rad"]))),
    )
    world_com = back_planner.world_points(
        sample["transform"],
        rm_b_model.center_of_mass_base(sample["q_rad"])[None],
    )[0]
    support = climb.gravity_projected_support(
        world_com,
        sample["anchors"][list(lf_c1_stage["support_legs"])],
        np.array((0.0, 0.0, -9.81)),
    )
    lf_c1_min_support_margin = min(
        lf_c1_min_support_margin, float(support.raw_margin_m)
    )
    if not index:
        continue
    previous = lf_c1_samples[index - 1]
    lf_c1_max_joint_step = max(
        lf_c1_max_joint_step,
        float(np.max(np.abs(sample["q_rad"] - previous["q_rad"]))),
    )
    lf_c1_max_foot_step = max(
        lf_c1_max_foot_step,
        float(
            np.linalg.norm(
                sample["anchors"][back_planner.LF]
                - previous["anchors"][back_planner.LF]
            )
        ),
    )
    lf_c1_max_base_step = max(
        lf_c1_max_base_step,
        float(np.linalg.norm(sample["pose"][:3] - previous["pose"][:3])),
    )
    lf_c1_max_roll_step = max(
        lf_c1_max_roll_step,
        abs(float(sample["pose"][3] - previous["pose"][3])),
    )
    lf_c1_max_pitch_step = max(
        lf_c1_max_pitch_step,
        abs(float(sample["pose"][4] - previous["pose"][4])),
    )
require(
    len(lf_c1_samples) == 140
    and lf_c1_max_residual
    <= back_planner.MAX_RESIDUAL_M + numeric_tolerance
    and lf_c1_max_joint_step
    <= back_planner.MAX_JOINT_STEP_RAD + numeric_tolerance
    and lf_c1_max_foot_step
    <= back_planner.MAX_FOOT_STEP_M + numeric_tolerance
    and lf_c1_max_base_step
    <= back_planner.MAX_BASE_STEP_M + numeric_tolerance
    and lf_c1_max_roll_step
    <= back_planner.MAX_ANGLE_STEP_RAD + numeric_tolerance
    and lf_c1_max_pitch_step
    <= back_planner.MAX_ANGLE_STEP_RAD + numeric_tolerance
    and lf_c1_min_joint_margin
    >= back_planner.MIN_JOINT_MARGIN_RAD - numeric_tolerance
    and lf_c1_min_support_margin
    >= back_planner.MIN_SUPPORT_MARGIN_M - numeric_tolerance,
    "back LF_C1 fixed C2 endpoint has 140 numeric verification leaves",
)


def require_resume_rejected(name, mutate):
    candidate = copy.deepcopy(back)
    mutate(candidate)
    try:
        candidate = back_planner._prepare_resume_artifact(
            candidate, back_stages
        )
        back_planner._resume_preflight(
            candidate,
            resume_requested,
            trace_path.read_bytes(),
            back_resume_scene,
            back_stages,
            k.GraspKinematic(),
        )
    except (KeyError, TypeError, ValueError):
        return
    raise AssertionError("M1A validation failed: resume accepted " + name)


def require_prepared_resume_rejected(name, mutate):
    candidate = copy.deepcopy(retry_back)
    mutate(candidate)
    try:
        back_planner._resume_preflight(
            candidate,
            resume_requested,
            trace_path.read_bytes(),
            back_resume_scene,
            back_stages,
            k.GraspKinematic(),
        )
    except (KeyError, TypeError, ValueError):
        return
    raise AssertionError("M1A validation failed: resume accepted " + name)


require_prepared_resume_rejected(
    "changed RM_B fixed midpoint",
    lambda value: value["fixed_stage_trajectory_geometry"]["RM_B"]
    ["anchor_knots_world_m"][1][back_planner.RM].__setitem__(0, 0.466),
)
require_prepared_resume_rejected(
    "changed RM_B segment semantics",
    lambda value: value["fixed_stage_trajectory_geometry"]["RM_B"].__setitem__(
        "curve", "generic_lift_template"
    ),
)
require_prepared_resume_rejected(
    "changed LF_C1 fixed C2 pose contract",
    lambda value: value["fixed_stage_trajectory_geometry"]["LF_C1"]
    ["pose_end_world"].__setitem__(2, 0.18),
)


legacy_lf_witness = {
    "sample": 58,
    "self_hit": None,
    "xiaolan_hit": {
        "left_link": "body",
        "left_leg": None,
        "left_component": 0,
        "left_triangle": 25554,
        "right_link": "xiaolan",
        "right_leg": None,
        "right_component": 0,
        "right_triangle": 17896,
    },
}
legacy_lf_attempt_matches = []
for index, attempt in enumerate(back.get("failed_attempt_history", [])):
    record = attempt.get("stage_record", {})
    blocked = attempt.get("blocked", {})
    if (
        record.get("name") == "LF_C1"
        and record.get("clear") is False
        and record.get("reason") == "visual_stl"
        and record.get("elapsed_s") == 6084.070828396001
        and record.get("visual_sample_count") == 59
        and record.get("witness") == legacy_lf_witness
        and blocked
        == {
            "stage": "LF_C1",
            "reason": "visual_stl",
            "earliest_witness": legacy_lf_witness,
        }
    ):
        legacy_lf_attempt_matches.append((index, attempt))
require(
    back["status"] == "PARTIAL_STAGE_COMPLETE"
    and len(legacy_lf_attempt_matches) == 1,
    "back uniquely identifies the archived legacy LF_C1 failure",
)
legacy_lf_attempt_index, legacy_lf_attempt = legacy_lf_attempt_matches[0]
lf_c1_record = back["stages"][back_planner.STAGE_NAMES.index("LF_C1")]
legacy_lf_resume_matches = []
resume_history = back.get("resume_history", [])
for index, entry in enumerate(resume_history):
    if entry.get("from_stage") != "RM_B":
        continue
    prior = entry.get("prior_elapsed_s")
    added = entry.get("added_elapsed_s")
    if (
        not isinstance(prior, (int, float))
        or not isinstance(added, (int, float))
        or not math.isfinite(prior)
        or not math.isfinite(added)
        or added < lf_c1_record["elapsed_s"]
    ):
        continue
    cumulative = prior + added
    prior_continuous = index == 0 or math.isclose(
        resume_history[index - 1]["prior_elapsed_s"]
        + resume_history[index - 1]["added_elapsed_s"],
        prior,
        rel_tol=0.0,
        abs_tol=1e-12,
    )
    if index + 1 < len(resume_history):
        next_entry = resume_history[index + 1]
        stop_is_lf_c1 = (
            next_entry.get("from_stage") == "LF_C1"
            and math.isclose(
                cumulative,
                next_entry.get("prior_elapsed_s", math.nan),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        )
    else:
        stop_is_lf_c1 = (
            back.get("stop_after") == "LF_C1"
            and math.isclose(
                cumulative,
                back.get("planner_elapsed_s", math.nan),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        )
    if prior_continuous and stop_is_lf_c1:
        legacy_lf_resume_matches.append((index, entry, cumulative))
require(
    len(legacy_lf_resume_matches) == 1,
    "back uniquely identifies the legacy LF_C1 migration resume interval",
)
legacy_lf_resume_index, legacy_lf_resume, legacy_lf_cumulative = (
    legacy_lf_resume_matches[0]
)

post_lf_c1 = copy.deepcopy(back)
lf_c1_count = back_planner.STAGE_NAMES.index("LF_C1") + 1
post_lf_c1["stages"] = post_lf_c1["stages"][:lf_c1_count]
post_lf_c1["nodes"] = {
    name: value
    for name, value in post_lf_c1["nodes"].items()
    if name == "PAIR_SOURCE" or name in back_planner.STAGE_NAMES[:lf_c1_count]
}
post_lf_c1["failed_attempt_history"] = post_lf_c1[
    "failed_attempt_history"
][:(legacy_lf_attempt_index + 1)]
post_lf_c1["resume_history"] = post_lf_c1["resume_history"][
    :(legacy_lf_resume_index + 1)
]
post_lf_c1["planner_elapsed_s"] = legacy_lf_cumulative
post_lf_c1["status"] = "PARTIAL_STAGE_COMPLETE"
post_lf_c1["stop_after"] = "LF_C1"
post_lf_state, post_lf_count, _ = back_planner._resume_preflight(
    post_lf_c1,
    "LB_HOVER",
    trace_path.read_bytes(),
    back_resume_scene,
    back_stages,
    k.GraspKinematic(),
)
require(
    post_lf_count == lf_c1_count
    and np.array_equal(
        post_lf_state["q"], np.asarray(back["nodes"]["LF_C1"]["q_rad"])
    ),
    "back controlled post-migration artifact stops at LF_C1",
)

legacy_lf_blocked = copy.deepcopy(post_lf_c1)
legacy_lf_blocked["failed_attempt_history"] = legacy_lf_blocked[
    "failed_attempt_history"
][:legacy_lf_attempt_index]
legacy_lf_blocked["resume_history"] = legacy_lf_blocked["resume_history"][
    :legacy_lf_resume_index
]
legacy_lf_blocked["stages"][-1] = legacy_lf_attempt["stage_record"]
legacy_lf_blocked["blocked"] = legacy_lf_attempt["blocked"]
legacy_lf_blocked["nodes"].pop("LF_C1")
legacy_lf_blocked["status"] = "BLOCKED"
legacy_lf_blocked["stop_after"] = "LF_C1"
legacy_lf_blocked["planner_elapsed_s"] = legacy_lf_resume[
    "prior_elapsed_s"
]
legacy_lf_blocked["fixed_targets"]["LF_C1"]["pose_world"][2] = 0.18
legacy_lf_blocked["fixed_targets"]["LF_C1"]["pose_world"][3] = 0.15
legacy_lf_blocked["fixed_stage_trajectory_geometry"].pop("LF_C1")
legacy_lf_prepared = back_planner._prepare_resume_artifact(
    legacy_lf_blocked, back_stages
)
legacy_lf_state, legacy_lf_count, _ = back_planner._resume_preflight(
    legacy_lf_prepared,
    "LF_C1",
    trace_path.read_bytes(),
    back_resume_scene,
    back_stages,
    k.GraspKinematic(),
)
require(
    legacy_lf_count == back_planner.STAGE_NAMES.index("LF_C1")
    and legacy_lf_prepared["status"] == "PARTIAL_STAGE_COMPLETE"
    and "blocked" not in legacy_lf_prepared
    and legacy_lf_prepared["fixed_targets"]
    == back_planner._fixed_targets(back_stages)
    and legacy_lf_prepared["fixed_stage_trajectory_geometry"]
    == back_planner._fixed_stage_trajectory_geometry(back_stages)
    and np.array_equal(
        legacy_lf_state["q"],
        np.asarray(back["nodes"]["RM_B"]["q_rad"]),
    ),
    "back controlled legacy LF_C1 BLOCKED artifact safely migrates",
)


def require_legacy_lf_resume_rejected(name, mutate):
    candidate = copy.deepcopy(legacy_lf_blocked)
    mutate(candidate)
    try:
        candidate = back_planner._prepare_resume_artifact(
            candidate, back_stages
        )
        back_planner._resume_preflight(
            candidate,
            "LF_C1",
            trace_path.read_bytes(),
            back_resume_scene,
            back_stages,
            k.GraspKinematic(),
        )
    except (KeyError, TypeError, ValueError):
        return
    raise AssertionError("M1A validation failed: resume accepted " + name)


def make_legacy_lf_partial(value):
    value["stages"] = value["stages"][:-1]
    value["status"] = "PARTIAL_STAGE_COMPLETE"
    value["stop_after"] = "RM_B"
    value.pop("blocked")


def change_legacy_lf_witness(value):
    value["stages"][-1]["witness"]["xiaolan_hit"][
        "right_triangle"
    ] = 17897
    value["blocked"]["earliest_witness"] = copy.deepcopy(
        value["stages"][-1]["witness"]
    )


require_legacy_lf_resume_rejected(
    "legacy LF_C1 PARTIAL does not migrate", make_legacy_lf_partial
)
require_legacy_lf_resume_rejected(
    "changed legacy LF_C1 pose",
    lambda value: value["fixed_targets"]["LF_C1"]["pose_world"].__setitem__(
        3, 0.14
    ),
)
require_legacy_lf_resume_rejected(
    "changed legacy LF_C1 source anchor",
    lambda value: value["nodes"]["RM_B"]["anchors_world_m"][0].__setitem__(
        0, value["nodes"]["RM_B"]["anchors_world_m"][0][0] + 1e-4
    ),
)
require_legacy_lf_resume_rejected(
    "changed legacy LF_C1 target anchor",
    lambda value: value["fixed_targets"]["LF_C1"][
        "goal_world_m"
    ].__setitem__(0, value["fixed_targets"]["LF_C1"]["goal_world_m"][0] + 1e-4),
)
require_legacy_lf_resume_rejected(
    "changed legacy LF_C1 witness", change_legacy_lf_witness
)


require_resume_rejected(
    "stale source hash",
    lambda value: value["source_trace"].__setitem__("sha256", "0" * 64),
)
require_resume_rejected(
    "changed stage order",
    lambda value: value["configured_stage_order"].__setitem__(0, "BODY2"),
)
require_resume_rejected(
    "changed fixed endpoint",
    lambda value: value["nodes"][resume_last_name]["pose_world"].__setitem__(
        0, value["nodes"][resume_last_name]["pose_world"][0] + 0.001
    ),
)
require_resume_rejected(
    "changed endpoint q and FK",
    lambda value: value["nodes"][resume_last_name]["q_rad"][0].__setitem__(
        0, value["nodes"][resume_last_name]["q_rad"][0][0] + 0.01
    ),
)
require_resume_rejected(
    "changed endpoint anchors and FK",
    lambda value: value["nodes"][resume_last_name]["anchors_world_m"][0].__setitem__(
        0,
        value["nodes"][resume_last_name]["anchors_world_m"][0][0] + 1e-4,
    ),
)
require_resume_rejected(
    "changed footpad binding",
    lambda value: value["visual_footpad_component_binding"].__setitem__(
        "component_index", 9
    ),
)
require_resume_rejected(
    "insufficient visual sample count",
    lambda value: value["stages"][0].__setitem__(
        "visual_sample_count", value["stages"][0]["minimum_samples"] - 1
    ),
)
require_resume_rejected(
    "failed recorded IK residual",
    lambda value: value["stages"][0]["metrics"].__setitem__(
        "max_ik_residual_m", back_planner.MAX_RESIDUAL_M + 1e-6
    ),
)
require_resume_rejected(
    "failed recorded support margin",
    lambda value: value["stages"][0]["metrics"].__setitem__(
        "min_support_margin_m", 0.0
    ),
)
require_resume_rejected(
    "changed active legs",
    lambda value: value["stages"][0].__setitem__("active_legs", []),
)
require_resume_rejected(
    "failed terminal visual footpad classification",
    lambda value: value["stages"][0]["terminal_contacts"]["rb"]
    ["visual_footpad_hit_classification"].__setitem__(
        "all_hit_facets_match_planned_support_surface", False
    ),
)


def blocked_retry_fixture():
    candidate = copy.deepcopy(retry_back)
    failed = {
        "name": resume_requested,
        "clear": False,
        "reason": "validator_fixture_failure",
        "witness": {"detail": "validator_fixture"},
    }
    candidate["stages"].append(failed)
    candidate["status"] = "BLOCKED"
    candidate["stop_after"] = resume_requested
    candidate["blocked"] = {
        "stage": resume_requested,
        "reason": failed["reason"],
        "earliest_witness": copy.deepcopy(failed["witness"]),
    }
    return candidate


def require_blocked_resume_rejected(name, mutate):
    candidate = blocked_retry_fixture()
    mutate(candidate)
    try:
        candidate = back_planner._prepare_resume_artifact(
            candidate, back_stages
        )
        back_planner._resume_preflight(
            candidate,
            resume_requested,
            trace_path.read_bytes(),
            back_resume_scene,
            back_stages,
            k.GraspKinematic(),
        )
    except (KeyError, TypeError, ValueError):
        return
    raise AssertionError(
        "M1A validation failed: blocked resume accepted " + name
    )


require_blocked_resume_rejected(
    "changed failed stage",
    lambda value: value["blocked"].__setitem__("stage", "not_next_stage"),
)
require_blocked_resume_rejected(
    "changed failed reason",
    lambda value: value["blocked"].__setitem__("reason", "visual_stl"),
)
require_blocked_resume_rejected(
    "changed failed witness",
    lambda value: value["blocked"]["earliest_witness"].__setitem__(
        "detail", "changed"
    ),
)


def append_second_failed_record(value):
    value["stages"].append(copy.deepcopy(value["stages"][-1]))


require_blocked_resume_rejected(
    "multiple failed records", append_second_failed_record
)
fixed = retry_back["fixed_targets"]
require_close(
    np.asarray(fixed["HIGH_C"]["rm"]),
    [0.299434, -0.130227, 0.190408],
    "back fixed RM HIGH_C shoulder target",
)
require_close(
    np.asarray(fixed["HIGH_C"]["rb"]),
    [0.269434, -0.220227, 0.184628],
    "back fixed RB HIGH_C shoulder target",
)
require_close(
    np.asarray(fixed["HIGH_C"]["rf"]),
    [0.325, 0.07, 0.1965],
    "back fixed RF HIGH_C top target",
)
require(
    fixed["HIGH_C"]["surface_policy"]
    == {"rm": "cad", "rb": "cad", "rf": "top"}
    and fixed["HIGH_C"]["fixed_CAD_target_is_planned_contact_not_contact_proof"],
    "back HIGH_C labels RM/RB non-top and RF top without claiming contact proof",
)
require_close(
    np.asarray(fixed["right_high_final_world_m"]["rb"]),
    [0.43, -0.18, 0.1965],
    "back fixed RB high target",
)
require_close(
    np.asarray(fixed["right_high_final_world_m"]["rf"]),
    [0.425, 0.02, 0.1965],
    "back fixed RF high target",
)
require_close(
    np.asarray(fixed["right_high_final_world_m"]["rm"]),
    [0.51, -0.13, 0.1965],
    "back fixed RM high target",
)
pair_y = float(trace["nodes"]["PAIR"]["base_world"][1])
require_close(
    np.asarray(fixed["LF_C1"]["pose_world"]),
    [0.23, pair_y - 0.03, 0.19, 0.10, -0.20],
    "back fixed LF_C1 pose",
)
require_close(
    np.asarray(fixed["LF_C1"]["goal_world_m"]),
    [0.19864857479269682, 0.12028708155300927, 0.15273468388201647],
    "back fixed LF_C1 goal remains unchanged",
    tol=2e-10,
)
require_close(
    np.asarray(fixed["B1"]["pose_world"]),
    [0.27, pair_y - 0.03, 0.22, 0.15, -0.20],
    "back fixed B1 pose",
)
require_close(
    np.asarray(fixed["D4"]["pose_world"]),
    [0.25, pair_y - 0.02, 0.215, 0.0, -0.15],
    "back fixed D4 pose",
)
require_close(
    np.asarray(fixed["D4"]["lb_goal_world_m"]),
    [0.19864857479269682, -0.23067608077320254, 0.15273462939636057],
    "back corrected LB low target",
    tol=2e-10,
)
require_close(
    np.asarray(fixed["M2_DONE"]["pose_world"]),
    [0.30, pair_y - 0.01, 0.24, 0.0, -0.10],
    "back fixed M2 DONE pose",
)

source_back_node = back["nodes"]["PAIR_SOURCE"]
require_close(
    np.asarray(source_back_node["q_rad"]),
    np.asarray(trace["nodes"]["PAIR"]["q_rad"]),
    "back source q equals front PAIR q",
)
require_close(
    np.asarray(source_back_node["anchors_world_m"]),
    np.asarray(trace["nodes"]["PAIR"]["anchors_world_m"]),
    "back source anchors equal front PAIR anchors",
)

back_stage_by_name = {stage["name"]: stage for stage in back["stages"]}
completed_names = [stage["name"] for stage in back["stages"] if stage["clear"]]
require(
    completed_names == list(back_planner.STAGE_NAMES[: len(completed_names)]),
    "back receipt contains a strict completed prefix",
)
thresholds = back["thresholds"]
for stage_name in completed_names:
    stage = back_stage_by_name[stage_name]
    metrics = stage["metrics"]
    require(
        stage["visual_sample_count"] >= stage["minimum_samples"],
        "back " + stage_name + " records every visual verification leaf",
    )
    require(
        metrics["max_ik_residual_m"] <= thresholds["max_ik_residual_m"]
        and metrics["max_adjacent_joint_delta_rad"]
        <= thresholds["max_adjacent_joint_delta_rad"]
        and metrics["max_active_foot_step_m"]
        <= thresholds["max_active_foot_step_m"]
        and metrics["max_base_translation_step_m"]
        <= thresholds["max_base_translation_step_m"]
        and metrics["max_roll_step_rad"]
        <= thresholds["max_roll_or_pitch_step_rad"]
        and metrics["max_pitch_step_rad"]
        <= thresholds["max_roll_or_pitch_step_rad"]
        and metrics["min_joint_margin_rad"]
        >= thresholds["min_joint_margin_rad"]
        and metrics["min_support_margin_m"]
        >= thresholds["min_geometry_support_margin_m"],
        "back " + stage_name + " dense numeric and support gates",
    )
    node = back["nodes"][stage_name]
    node_pose = back_planner.pose_matrix(np.asarray(node["pose_world"]))
    require_close(
        back_planner.world_points(
            node_pose,
            k.GraspKinematic().forward_base(np.asarray(node["q_rad"])),
        ),
        np.asarray(node["anchors_world_m"]),
        "back " + stage_name + " node transform matches anchors",
        tol=2e-6,
    )

if back["status"] == "BLOCKED":
    require(
        back.get("blocked", {}).get("earliest_witness") is not None,
        "back BLOCKED receipt retains first witness",
    )
elif back["status"] == "PARTIAL_STAGE_COMPLETE":
    require(
        back["stop_after"] in back_planner.STAGE_NAMES
        and completed_names[-1] == back["stop_after"],
        "back partial receipt stops at a completed fixed stage",
    )
else:
    require(
        back["status"] == "MODEL_PATH_FOUND_BACK_HALF"
        and completed_names == list(back_planner.STAGE_NAMES),
        "back success requires every fixed stage",
    )


# 紧凑运行记录只能在原有阶段后追加模拟阶段。
compact_path = SCRIPTS_DIR.parent / "config" / "climb_compact.json"
compact = json.loads(compact_path.read_text())
require(
    compact["schema"] == "SIMULATION_ONLY_CLIMB_COMPACT_V2"
    and compact["simulation_only"]
    and compact["simulation_candidate_only"]
    and compact["stage_count"] == 35,
    "compact V2 simulation-only schema and stage count",
)
source_bindings = compact["source_traces"]
require(
    source_bindings["front"]["sha256"]
    == hashlib.sha256(trace_path.read_bytes()).hexdigest()
    and source_bindings["strict_back_prefix"]["sha256"]
    == hashlib.sha256(back_path.read_bytes()).hexdigest()
    and source_bindings["strict_back_prefix"]["stop_after"] == "B1"
    and source_bindings["sim_finish"]["sha256"]
    == hashlib.sha256(sim_trace_path.read_bytes()).hexdigest(),
    "compact V2 binds front, strict-prefix, and sim-finish SHA256",
)
require_close(
    np.asarray(compact["xiaolan_translation"]),
    np.asarray(trace["world_from_xiaolan"])[:3, 3],
    "compact Xiaolan translation",
)
legacy = compact["front_v1_receipt"]
require(compact["p0"] == legacy["p0"], "compact P0 is the preserved V1 receipt")
require(
    compact["settle_gate"]
    == {
        "command_tracking_only_not_contact_proof": True,
        "entry_max_joint_error_rad": 0.08,
        "max_foot_target_error_m": 0.010,
        "max_joint_tracking_error_rad": 0.08,
        "persistence_s": 0.25,
        "preview_time_only_stage_advance": True,
        "timeout_s": 3.0,
        "tracking_errors_diagnostic_only": True,
    }
    and sim_finish.SIM_SETTLE_MAX_FOOT_TARGET_ERROR_M == 0.010,
    "compact uses the 10 mm simulation-only foot-target settle tolerance",
)
compact_p0_q = np.asarray(compact["p0"]["q_rad"])
compact_p0_base = np.asarray(compact["p0"]["base"])
compact_p0_pose = visual_planner.pose_matrix(
    *compact_p0_base[:3], compact_p0_base[3]
)
require_close(compact_p0_q, k.Q_STAND, "compact P0 is Q_STAND")
require_close(
    np.asarray(compact["p0"]["anchors_world_m"]),
    visual_planner.world_points(
        compact_p0_pose, k.GraspKinematic().forward_base(compact_p0_q)
    ),
    "compact P0 FK anchors",
    tol=2e-10,
)
require(
    legacy["durations_s"]
    == {
        "PREP": 6.0,
        "RM": [2.5, 4.0, 2.5, 2.0],
        "BODY": 4.0,
        "PAIR": [3.0, 8.0, 3.0],
    }
    and legacy["settle_gate"]["persistence_s"] == 0.25
    and legacy["settle_gate"]["max_foot_target_error_m"] == 0.005
    and legacy["settle_gate"]["entry_max_joint_error_rad"] == 0.08
    and legacy["settle_gate"]["command_tracking_only_not_contact_proof"],
    "compact preserves V1 durations and command-tracking settle semantics",
)
expected_front_stages = sim_finish._front_compact_stages(trace, legacy)
preview_back_stages = sim_finish._preview_back_stages(
    back_stages, planner_normal
)
expected_names = [
    "PREP",
    "RM",
    "BODY",
    "PAIR",
    *[stage["name"] for stage in preview_back_stages],
]
compact_stages = compact["stages"]
require(
    [stage["name"] for stage in compact_stages] == expected_names,
    "compact V2 exact 26-stage order",
)
require(
    back_planner._json_value(compact_stages[:4])
    == back_planner._json_value(expected_front_stages),
    "compact front four stages are numerically V1-equivalent",
)
back_q_nodes = {**back["nodes"], **sim_trace["nodes"]}
expected_q = np.asarray(back["nodes"]["PAIR_SOURCE"]["q_rad"])
compact_model = k.GraspKinematic()
for index, fixed_stage in enumerate(preview_back_stages, start=4):
    compact_stage = compact_stages[index]
    expected_durations = (
        [2.0] * fixed_stage["segments"]
        if fixed_stage["active_legs"]
        else [4.0]
    )
    require_close(
        np.asarray(compact_stage["pose_start"]),
        fixed_stage["pose_start"],
        "compact " + fixed_stage["name"] + " pose start",
    )
    require_close(
        np.asarray(compact_stage["pose_end"]),
        fixed_stage["pose_end"],
        "compact " + fixed_stage["name"] + " pose end",
    )
    require_close(
        np.asarray(compact_stage["anchor_knots"]),
        fixed_stage["anchor_knots"],
        "compact " + fixed_stage["name"] + " anchor knots",
    )
    if fixed_stage["name"] in back_q_nodes:
        expected_q = np.asarray(
            back_q_nodes[fixed_stage["name"]]["q_rad"]
        )
    else:
        expected_q, endpoint_residual = visual_planner.solve_anchors(
            compact_model,
            fixed_stage["anchor_knots"][-1],
            back_planner.pose_matrix(fixed_stage["pose_end"]),
            expected_q,
            1e-8,
        )
        require(
            endpoint_residual <= 1e-6,
            "compact " + fixed_stage["name"] + " endpoint IK",
        )
    require_close(
        np.asarray(compact_stage["expected_q_end"]),
        expected_q,
        "compact " + fixed_stage["name"] + " endpoint q",
    )
    require(
        compact_stage["active_legs"] == list(fixed_stage["active_legs"])
        and compact_stage["segment_durations_s"] == expected_durations
        and compact_stage["settle_s"] == 0.5
        and compact_stage["pose_curve"] == "quintic_full_stage"
        and compact_stage["anchor_curve"] == "piecewise_quintic",
        "compact " + fixed_stage["name"] + " fixed playback semantics",
    )

previous_pose = np.array((*compact_p0_base[:3], 0.0, compact_p0_base[3]))
previous_anchors = np.asarray(compact["p0"]["anchors_world_m"])
for stage in compact_stages:
    require_close(
        np.asarray(stage["pose_start"]),
        previous_pose,
        "compact " + stage["name"] + " pose boundary continuity",
    )
    require_close(
        np.asarray(stage["anchor_knots"])[0],
        previous_anchors,
        "compact " + stage["name"] + " anchor boundary continuity",
    )
    transform = ClimbMode._world_from_base(stage["pose_end"])
    require_close(
        transform,
        back_planner.pose_matrix(np.asarray(stage["pose_end"])),
        "compact " + stage["name"] + " T Ry pitch Rx roll",
    )
    previous_pose = np.asarray(stage["pose_end"])
    previous_anchors = np.asarray(stage["anchor_knots"])[-1]
require_close(
    np.asarray(compact_stages[3]["pose_end"]),
    np.asarray(compact_stages[4]["pose_start"]),
    "compact front/back boundary pose",
)
require_close(
    np.asarray(compact_stages[3]["anchor_knots"])[-1],
    np.asarray(compact_stages[4]["anchor_knots"])[0],
    "compact front/back boundary anchors",
)
require_close(
    np.asarray(compact_stages[3]["expected_q_end"]),
    np.asarray(back["nodes"]["PAIR_SOURCE"]["q_rad"]),
    "compact front/back boundary q",
    tol=1e-6,
)
require(
    [stage["name"] for stage in compact_stages[-15:]]
    == list(sim_finish.PREVIEW_SUFFIX_NAMES),
    "compact uses the pair-first C21-C26 preview suffix",
)
compact_total_s = sum(
    sum(stage["segment_durations_s"]) + stage["settle_s"]
    for stage in compact_stages
)
require_close(compact_total_s, 199.0, "compact total movement plus settle duration")

# 检查控制器在所有阶段的理想跟踪结果。
smoke_controller = control.GraspController(0.02)
smoke_q = compact_p0_q.copy()
smoke_controller.enter_climb(smoke_q, compact)
smoke_mode = smoke_controller.climb_mode
require(
    smoke_mode.state == smoke_mode.RUNNING
    and smoke_mode.phase == "PREP"
    and smoke_mode.stage_index == 0,
    "compact V2 enter starts PREP",
)
require_close(
    smoke_controller.foot_desired_base,
    smoke_controller.foot_desired_base_prev,
    "compact enter synchronizes feed-forward target",
)
require_close(smoke_controller.q_des, smoke_q, "compact enter synchronizes q")
hold_indices = {0, 12, 24}
held = set()
visited = []
endpoint_joint_error = 0.0
endpoint_foot_error = 0.0
previous_index = smoke_mode.stage_index
for _ in range(20000):
    if smoke_mode.phase not in visited:
        visited.append(smoke_mode.phase)
    if smoke_mode.stage_index in hold_indices and smoke_mode.stage_index not in held:
        frozen_time = smoke_mode.phase_time
        frozen_target = smoke_controller.foot_desired_base.copy()
        smoke_controller.hold_climb()
        smoke_q = smoke_controller.update(smoke_q, np.zeros(4)).copy()
        require(
            smoke_mode.state == smoke_mode.HOLD
            and smoke_mode.phase_time == frozen_time
            and np.array_equal(
                smoke_controller.foot_desired_base, frozen_target
            ),
            "compact front/middle/back HOLD freezes time and target",
        )
        smoke_controller.resume_climb()
        held.add(smoke_mode.stage_index)
    smoke_q = smoke_controller.update(smoke_q, np.zeros(4)).copy()
    require(np.all(np.isfinite(smoke_q)), "compact smoke q remains finite")
    if smoke_mode.stage_index != previous_index or smoke_mode.state == smoke_mode.DONE:
        completed = compact_stages[previous_index]
        endpoint_joint_error = max(
            endpoint_joint_error,
            float(
                np.max(
                    np.abs(
                        smoke_q - np.asarray(completed["expected_q_end"])
                    )
                )
            ),
        )
        actual_feet = smoke_controller.kinematic.hip_to_base(
            smoke_controller.kinematic.forward(smoke_q)
        )
        endpoint_foot_error = max(
            endpoint_foot_error,
            float(
                np.max(
                    np.linalg.norm(
                        actual_feet - smoke_controller.foot_desired_base,
                        axis=1,
                    )
                )
            ),
        )
        previous_index = smoke_mode.stage_index
    if smoke_mode.state in (smoke_mode.DONE, smoke_mode.FAILED):
        break
require(
    smoke_mode.state == smoke_mode.DONE
    and visited == expected_names
    and held == hold_indices
    and endpoint_joint_error
    <= compact["settle_gate"]["max_joint_tracking_error_rad"]
    and endpoint_foot_error
    <= compact["settle_gate"]["max_foot_target_error_m"],
    "compact perfect tracking reaches DONE through all stages and hold points",
)
require_close(
    smoke_q,
    np.asarray(compact_stages[-1]["expected_q_end"]),
    "compact final expected q",
    tol=2e-8,
)
require_close(
    smoke_controller.foot_desired_base,
    smoke_controller.kinematic.hip_to_base(
        smoke_controller.kinematic.forward(smoke_q)
    ),
    "compact final expected feet",
    tol=2e-8,
)

entry_reject_controller = control.GraspController(0.02)
try:
    entry_reject_controller.enter_climb(compact_p0_q + 0.09, compact)
except ValueError:
    pass
else:
    raise AssertionError(
        "M1A validation failed: compact entry joint gate accepted error"
    )

for flag_name in (
    "preview_time_only_stage_advance",
    "tracking_errors_diagnostic_only",
):
    bad_flag_compact = copy.deepcopy(compact)
    bad_flag_compact["settle_gate"][flag_name] = 1
    try:
        control.GraspController(0.02).enter_climb(
            compact_p0_q, bad_flag_compact
        )
    except ValueError:
        pass
    else:
        raise AssertionError(
            "M1A validation failed: compact accepted non-boolean " + flag_name
        )

# 检查预览模式按模拟时间推进，不把跟踪误差当作阻塞条件。
preview_controller = control.GraspController(0.02)
preview_controller.enter_climb(compact_p0_q, compact)
preview_mode = preview_controller.climb_mode
preview_mode.phase_time = sum(compact_stages[0]["segment_durations_s"])
for _ in range(
    math.ceil(compact_stages[0]["settle_s"] / preview_controller.dt) + 1
):
    preview_mode.update(np.zeros(4), compact_p0_q + 0.2)
require(
    preview_mode.state == preview_mode.RUNNING
    and preview_mode.stage_index == 1
    and preview_mode.phase == "RM"
    and preview_mode.last_settled is False
    and preview_mode.failure_reason == "",
    "compact preview advances by simulation time despite large diagnostics",
)

# 检查运行诊断值会刷新，并在超时时保留跟踪信息。
diagnostic_controller = control.GraspController(0.02)
diagnostic_mode = diagnostic_controller.climb_mode
require(
    diagnostic_mode.last_tracking_error_rad == 0.0
    and diagnostic_mode.last_foot_target_error_m == 0.0
    and diagnostic_mode.last_settled is False,
    "compact tracking diagnostics have finite initial values",
)
tracking_gate_compact = copy.deepcopy(compact)
tracking_gate_compact["settle_gate"][
    "preview_time_only_stage_advance"
] = False
tracking_gate_compact["settle_gate"][
    "tracking_errors_diagnostic_only"
] = False
diagnostic_controller.enter_climb(compact_p0_q, tracking_gate_compact)
diagnostic_mode.update(np.zeros(4), compact_p0_q)
require(
    math.isfinite(diagnostic_mode.last_tracking_error_rad)
    and math.isfinite(diagnostic_mode.last_foot_target_error_m)
    and isinstance(diagnostic_mode.last_settled, bool),
    "compact tracking diagnostics refresh on update",
)
diagnostic_mode.phase_time = (
    sum(compact_stages[0]["segment_durations_s"])
    + compact["settle_gate"]["timeout_s"]
)
diagnostic_mode.update(np.zeros(4), compact_p0_q + 0.2)
require(
    diagnostic_mode.state == diagnostic_mode.FAILED
    and math.isfinite(diagnostic_mode.last_tracking_error_rad)
    and math.isfinite(diagnostic_mode.last_foot_target_error_m)
    and diagnostic_mode.last_settled is False
    and "tracking_error_rad=" in diagnostic_mode.failure_reason
    and "foot_target_error_m=" in diagnostic_mode.failure_reason,
    "compact timeout exposes finite tracking diagnostics without contact proof",
)

print(
    "PASS: M1 foundation, M2/M3 readiness, and M2 CAD geometry "
    "validation (not M2 feasibility certification)"
)
