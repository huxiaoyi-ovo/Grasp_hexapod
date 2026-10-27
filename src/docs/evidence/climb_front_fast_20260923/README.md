# Front fast climb candidate: structural, local mesh, and one GPU preview

This directory records an independent simulation candidate. The preserved input `config/climb_front.json` remains the 50-stage baseline (SHA-256 `bac3f74e8e2904a04df9e2ddebdaf308831119033ddabc342fd2b02561c16aee`). The fast candidate is 41 stages after nine bounded merges. Its SHA-256 is `d15bf4d9f9a8feca09ebb1290502176e539f5b56c8b751ce9f9d0161f6601898` and it remains simulation-only.

| Diagnostic | Baseline | Fast candidate |
| --- | ---: | ---: |
| Stages | 50 | 41 |
| Simulated duration | 103.4667 s | 43.6667 s |
| Planned command cap | 4 rad/s | 6 rad/s |
| Isaac DOF speed multiplier | 1.2 (4.8 rad/s) | 1.8 (7.2 rad/s) |

The duration difference is -59.8 s (-57.8%). These are preview settings and simulated timing, not real servo speed measurements.

## Changes and CPU evidence

The candidate keeps the eight earlier merges and adds `BODY_B_FRONT_INWARD` from `BODY_B + FRONT_INWARD`, active legs `[1,4]`, the original 20 mm apex, and full-stage quintic pose from the source start to source end. Other merges are `IJ_MID_DIRECT`, `LM_MID_RELAND`, `OP_MID_BODY_LIFT`, `R_FRONT_BODY_LIFT`, `RB_RESET_BODY_LIFT`, `AE_AF_BACK_PAIR`, `AGAB_BACK_PAIR`, and `AGCD_BACK_PAIR`.

The K and N rear world-x landings changed from 0.03 to 0.00 m and 0.05 to 0.01 m respectively; downstream reset targets were synchronized. The earlier old-N rear knee intersection is separately documented by the pre-fix baseline mesh witness; all four specified post-fix rear knee pairs were explicitly queried and clear at their sampled poses. The exact-pair body checks and known-hit AABB-prefilter regression are in `back_knee_spacing_visual_mesh_audit.json` and its pair cache. A clear local triangle query does not establish containment or continuous-path clearance. The selected 1310-tick audit retains one Q_MID_LOW capsule proxy witness; its four body-to-LM/RM knee/ankle pairs were checked at 1.200 s, and the worst-gap body-to-LM-knee pair was checked again at 1.2667 s. The local triangle AABB/broadphase rejected all candidates; see `cpu_fast_q_local_mesh_1310.json`.

`cpu_structural_41_audit.json` records the structural candidate at 2711 ticks. `cpu_fast_41_audit.json` records the selected 1310-tick schedule, 41-stage durations, zero clips, geometry SHA, and candidate config SHA. The selected timing reconstructs the recorded slowdown schedule as a speed-margin heuristic; the original audit run did not capture the later candidate hash, so the receipt labels this binding `reconstructed_from_recorded_timing`. The exact config now matches that timing receipt byte-for-byte. The earlier 1274-tick v1 snapshot and its summary remain separate.

To reproduce the final timing JSON from the archived structural candidate and recorded timing metadata without rerunning a controller audit:

```bash
python3 src/grasp_hexapod_control/scripts/tools/optimize_climb_front.py \
  --baseline src/grasp_hexapod_control/config/climb_front.json \
  --output /tmp/front_fast_reconstructed.json \
  --audit src/docs/evidence/climb_front_fast_20260923/cpu_structural_41_audit.json \
  --stage-map /tmp/front_fast_reconstructed_map.json \
  --structural-output src/docs/evidence/climb_front_fast_20260923/structural_candidate_41.json \
  --fast-audit /tmp/front_fast_reconstructed_audit.json \
  --retime-only \
  --recorded-retime-metadata src/docs/evidence/climb_front_fast_20260923/cpu_fast_41_audit.json \
  --align-recorded-only
sha256sum /tmp/front_fast_reconstructed.json src/grasp_hexapod_control/config/climb_front_fast.json
```

The receipt must remain marked as reconstructed rather than claiming an audit-time config hash. The fast timing rule scales every non-hold segment in a stage by one factor, preserving stage geometry and normalized reference curves. `FINAL_HOLD` remains 2 seconds; non-hold settle is 0.10 seconds. The 5.2 rad/s value is a soft speed-margin estimate, not a CPU acceptance gate.

## Focused test result

Tier 0 Python compilation and config validation passed. The first two-suite pytest run had 10 passed and 2 failed due to test assertion implementation errors (optional geometry keys on inactive stages and NumPy cap comparison). After correcting those test assertions, both affected tests passed in a targeted rerun. No simulation/controller behavior failed. This is not recorded as one clean 12-test run.

## GPU preview

One headless Isaac preview ran from P0 through C41 `FINAL_HOLD`; command, pre-start config/runtime hashes, rates, and gains are in `gpu_invocation.txt`. Raw console and metrics are `gpu_console.log` and `gpu_metrics.json`. The run reached `DONE`, reason `none`, at 43.6667 seconds of simulated control time. All five console preview gates passed:

- final hold joint drift: 0.00089407 mrad, limit 20 mrad;
- final foot target error: 0.001080 m, limit 0.015 m;
- final root position error: 0.015679 m, limit 0.050 m;
- global root position error: 0.021287 m, limit 0.200 m;
- minimum joint-limit margin: 0.011683 rad, limit 0 rad.

Additional requested diagnostics also passed: global max foot error 11.732 mm (MID_GROUND_SHIFT), max root error 21.287 mm (AE_AF_BACK_PAIR), max support-foot drift 9.239 mm (Q_MID_LOW), zero target velocity clips, and max controller target speed 5.0312 rad/s (Z_FRONT_48_SYNC). The minimum joint margin was 0.011683 rad at Q_MID_LOW. Summary is in `gpu_summary.json`.

The six world foot centers in `final_actual_foot_stl_audit.json` come from `stage_start_actual_world_foot_xyz_m` at C41 `FINAL_HOLD` entry, not the last simulation frame. Their local static-STL sphere gaps are approximately -0.019 mm to +0.0004 mm. This is a local hold-entry geometry check only.

## Evidence boundary

The simulator still filters robot actor self-collision. These artifacts do not prove full-route robot self-collision clearance, triangle containment, continuous mesh clearance, physical contact, load support, friction, stability, or hardware readiness. This is simulation evidence only and does not authorize hardware execution.

## Superseded terminal geometry

The `d15bf4...` result above is the preserved historical x = 0.190 m rear terminal candidate. A later read-only review found LB/RB at the 12-degree slope / 25-degree bevel boundary with zero inward edge margin. It is superseded for terminal-patch purposes by the separate simulation candidate SHA `847faeec51f258179090c59f736b773b02273e4662a0fc0bde200bd5e4ebf67c` in `src/docs/evidence/climb_front_terminal_20260924/`. Historical config, metrics, and prior conclusions remain unchanged; consult the new directory for the diagonal-descent correction and its evidence limits.
