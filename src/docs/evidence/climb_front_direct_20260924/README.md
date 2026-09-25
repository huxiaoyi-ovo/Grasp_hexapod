# Accepted 21-stage direct front route

Current simulation candidate: `rear_body_advance_candidate.json`, SHA-256
`4421aff133035f358e69ed33ddaa649d580750d868f2318736efc17e797bf60c`.
The route reduces the accepted fast predecessor from 41 stages / 43.6667 s to
21 stages / 29.1 s (about 49% fewer stages and 33% less simulated control time). Four
source spans are merged to remove redundant backtracking. Final support keeps
the first M touchdown at x=.205 m, terminal M at x=.235 m, terminal B at
x=.225 m, and advances the final body pose to x=.318 m, z=.245 m to preserve
rear/middle knee clearance.

Run the visible preview with the recorded command:

```bash
/home/artrc/miniconda3/bin/conda run --no-capture-output -n grasp_hexapod python3 -u src/grasp_hexapod_control/scripts/run_sim.py --climb-start --climb-config src/grasp_hexapod_control/config/climb_front_direct.json --climb-speed 1 --climb-joint-speed 1.8 --climb-metrics /tmp/climb_front_direct_preview.json
```

Accepted evidence is bound to the same config SHA: `cpu_rear_body_advance.json`
(DONE, 873 ticks), inherited v2 prefix mesh receipt `mesh_mid_inset_audit.json`,
46 new AH mesh queries in `mesh_rear_body_advance_audit.json` (zero hits), and
the visible GPU manifest/log/metrics/summary plus
`gpu_rear_body_advance_actual_foot_stl_audit.json`. All five simulator preview
gates passed. Actual simulator feet passed the named `FRONT_TO_040` LM/RM entry
and `FINAL_HOLD` six-foot patch checks for sampled normal, sphere gap, and
required broad-patch inset. `fixed_contact_intervals.json` summarizes saved
CPU thigh samples across complete fixed-foot intervals and confirms planned
anchors stayed constant.

The earlier a7 and v2 candidates failed final rear-patch acceptance; the
rear-only x=.225 candidate hit a rear/middle knee pair in the AH mesh window.
Those receipts remain preserved and are not accepted. The current result is a
bounded simulation/CAD candidate: discrete mesh samples and Isaac feet do not
prove continuous clearance, real contact, load, friction, stability, or
hardware readiness. The front route remains rejected for hardware execution.

# Front direct-route endpoint planning notes

## Pre-builder closure

### Landing height source

- **Symptom / root cause:** v4–v6 reported false endpoint feasibility because `cached_z` searched every anchor knot and could select an air knot. At front x `.40/.48`, the air height was `.2165 m` while the endpoint landing height was `.1965 m`; at mid x `.235`, it was `.165316 m` instead of `.160316 m`; ground x `.055` was read as `.0265 m` instead of `.0065 m`. Those receipts are retained and marked `invalid_cached_air_height`.
- **Fix / reusable rule:** v7/v8 use only the named source stage's final anchor knot for Xiaolan contacts, explicit `z=.0065 m` for ground contacts, and check the foot-sphere signed gap before static IK. The endpoint x/y must match the requested leg and the Xiaolan gap must be within `[-0.5,+2] mm`.
- **Regression / adoption:** v8 records 20 landing-height checks, all passing; observed Xiaolan gaps range from `-0.142 mm` to numerical zero. The new builder must call the same endpoint-only check before any IK solve. A missing endpoint match, wrong surface gap, or ground z other than `.0065 m` rejects the state.
- **Evidence boundary / technique:** this catches incorrect contact height cheaply and locally. It does not prove the intervening foot path, whole-robot collision freedom, load support, friction, stability, or hardware behavior. Applicable when reusing a contact from an existing stage; fail closed instead of guessing from an arbitrary knot.

### Fixed-thigh measurement across stage boundaries

- **Symptom / root cause:** the old per-stage `.08 rad` fixed-thigh comparison depends on where a continuous contact interval is split. The 41-stage `cpu_structural_41_audit.json` contains `q_thigh_samples`; regrouping samples by uninterrupted fixed-foot support shows LM and RM each change about `.7031 rad` across PREP→BODY_A, with peak sampled rate about `1.9647 rad/s`. The stage threshold describes segmentation, not support, slip, or a physical limit.
- **Fix / reusable rule:** for the new front plan, preserve and report both per-stage thigh deltas and cumulative range/rate over each complete fixed-contact interval. Do not reject a route solely because a contact interval crosses `.08 rad`; retain hard IK limits, fixed world anchors, support COM, self-link, speed/clip, and surface checks.
- **Regression / adoption:** the continuous CPU receipt must group contiguous non-active leg samples across stage boundaries, report each interval's thigh min/max/span and peak rate, and confirm world anchors remain constant. Do not add a relanding just to reset a per-stage delta.
- **Evidence boundary / technique:** the old CPU samples are model/controller diagnostics, not measurements of physical loading or slip. This rule applies to the new front route only; side planning is unchanged. It avoids segmentation-driven extra steps while retaining joint-limit and actual path gates.

## v8 endpoint baseline

Source fast config SHA-256: `847faeec51f258179090c59f736b773b02273e4662a0fc0bde200bd5e4ebf67c`.
The corrected-height seeded static probe has 24 states, with minimum entry/exit COM support margin `11.95 mm`, minimum joint margin `0.02424 rad`, maximum IK residual `0.529 mm`, and no failing endpoint. This is not continuous CPU or collision evidence. See `endpoint_probe_v8.py` and `endpoint_probe_v8.json`.
