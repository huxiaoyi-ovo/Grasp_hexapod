# Front terminal landing correction

## Accepted simulation candidate

`config/climb_front_fast.json` is SHA-256 `847faeec51f258179090c59f736b773b02273e4662a0fc0bde200bd5e4ebf67c`. It retains the 41-stage, nine-merge, 43.6667 s front-fast plan and changes only the rear-leg tail geometry. The original 50-stage config remains untouched.

The terminal rear landing is at world x = 0.205 m and actual STL-derived z = 0.15393948 m. The two rear air knots are restored exactly from the frozen d15bf source: knot 1 x = 0.13 m and knot 2 x = 0.19 m, both z = 0.16575113 m. The final 15 mm of rear travel is a diagonal forward descent from x = 0.19 m to x = 0.205 m. AH body pose, stage duration, all prefix stages, other leg anchors, and HOLD body pose/duration are unchanged. Terminal IK residual is 9.37e-7 m.

The full reproducible configuration-generation command is:

```bash
python3 src/grasp_hexapod_control/scripts/tools/refine_climb_front_terminal.py \
  --input src/docs/evidence/climb_front_terminal_20260924/before_climb_front_fast.json \
  --output /tmp/climb_front_diagonal_forward_reconstructed.json
sha256sum /tmp/climb_front_diagonal_forward_reconstructed.json src/grasp_hexapod_control/config/climb_front_fast.json
```

Both generated and accepted config hashes are `847faeec51f258179090c59f736b773b02273e4662a0fc0bde200bd5e4ebf67c`. This regenerates the JSON and terminal IK; it does not rerun CPU or GPU audits.

## Targeted collision and patch evidence

The x = 0.210 m vertical-landing variants had real LB–LM and RB–RM knee mesh intersections in the high crossing. Endpoint IK and capsule-proxy probes were feasible; no exact x = 0.210 m terminal STL query was made. Lowering the air apex from +15 mm to +10 mm did not remove those crossings. Exact terminal knee-pair queries for the original d15bf pose and the later x = 0.205 m candidate were both clear. At x = 0.205 m with the original high air knots restored and a diagonal final descent, a seeded 30 Hz AH replay completed in 68 ticks with zero velocity clips and no capsule warnings: maximum foot error 6.400 mm, minimum joint margin 0.13993 rad, minimum support COM margin 41.56 mm, and minimum active-foot sphere gap -0.142 mm.

The measured AH window from phase 1.5 s through landing contains 23 discrete controller samples. Both rear knee pairs were checked at every sample (46 selected visual-STL pair queries); all were clear. This is a discrete, pair-specific check, not a continuous or whole-robot mesh certification. Evidence is in `local_ah_replay_diagonal_forward.json`, `ah_knee_window_x205_mesh_audit.json`, and `ah_first_capsule_mesh_pair_cache.json`.

At C41 `FINAL_HOLD` entry, the six actual world foot centers are taken from `stage_start_actual_world_foot_xyz_m` in the GPU metrics. Exact nearest-point checks show:

- LB and RB are on the 12-degree broad slope, with inward edge clearances 12.418 mm and 12.993 mm. Their nearest triangle normals align with the intended slope normal.
- LM and RM are on the same broad slope; LF and RF are on the horizontal top patch with upward normals.
- All six signed sphere gaps are between -0.0186 mm and +0.0004 mm. These geometry checks exclude gross suspension or penetration at this sampled hold entry; they do not prove physical contact or support load.
- The regression rejects the previous LB/RB shared-edge points, whose edge clearance was zero, and accepts the new planned rear anchors with more than 10 mm margin.

The patch receipt is `final_actual_foot_stl_audit.json`; candidate binding comes from the prelaunch `gpu_start_manifest.json`, since the metrics file itself has no config SHA field.

## One full headless preview

The single authorized run used 240 Hz physics and 30 Hz controller/actuator updates, `--climb-speed 1`, and `--climb-joint-speed 1.8` (7.2 rad/s simulated DOF cap). The front command cap is 6 rad/s; gains and URDF effort were unchanged. Config and runtime SHA-256 values and the exact command are in `gpu_start_manifest.json`; raw output is `gpu_console.log` and metrics are `gpu_metrics.json`.

The run reached `DONE` at C41 `FINAL_HOLD`, reason `none`, in 43.6667 simulated seconds. All five preview gates passed. Additional global checks also passed: maximum kinematic foot-target error 11.732 mm, root position error 21.287 mm, support-foot drift 9.239 mm, zero velocity clips, and peak target speed 5.031 rad/s. Minimum joint-limit margin was 0.011683 rad. The separate maximum world-foot-anchor error was 22.035 mm at `AD_RB_075_SYNC` and is retained as a distinct tracking diagnostic. Summary is `gpu_summary.json`.

## Focused checks and preserved rejected candidates

Tier 0 Python compilation and config validation passed. The front and fast focused suites passed 13 tests; the later patch-boundary regression passed separately (1 test), not as a combined 14-test run.

Rejected candidate and witness files remain separate: `rejected_apex15_*`, `rejected_x210_apex10_*`, and `rejected_x205_apex10_*`. The first x = 0.210 m / 15 mm apex aggregate receipt was overwritten by the later x = 0.210 m / 10 mm query; its distinct high-crossing witness and triangle hit remain in `ah_first_capsule_mesh_pair_cache.json` and the stopped-run log, not in a preserved aggregate JSON. The x = 0.205 m / 10 mm apex midair collisions are retained in `rejected_x205_apex10_air_mesh.json`. `terminal_knee_diagnosis.json` covers only the original d15bf and x = 0.205 m terminal poses; both queried poses were clear. The x = 0.210 m endpoint was not STL-queried.

## Evidence boundary

These are kinematic, visual-STL, and Isaac simulation diagnostics. Actor self-collision remains filtered by the simulator. The selected checks do not establish continuous whole-route collision freedom, mesh containment, physical contact, load, friction, stability, or hardware readiness.
