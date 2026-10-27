# Front climb evidence, 2026-09-23

`config_snapshot.json` is the generated front-only route used by the one retained nominal headless GPU run. `cpu_replay.json` is the full P0/Q_STAND inherited-reference CPU replay. `gpu_metrics.json` is the retained machine-readable simulator output; `receipt.json` binds commands and input hashes. `final_actual_foot_stl_audit.json` performs six local foot-sphere-to-static-STL queries at `FINAL_HOLD` stage start.

The original headless console was not retained by its tool session. Consequently this directory does not reconstruct its hold-drift or `PREVIEW VERDICT` text. The metrics record `final_state: DONE`, `final_reason: none`, and all 50 per-stage rows, but these remain simulation diagnostics and are not real contact, support, load, friction, stability, or hardware evidence.

## Visual preview

```bash
conda run --no-capture-output -n grasp_hexapod python3 src/grasp_hexapod_control/scripts/run_sim.py \
  --climb-start --climb-config src/grasp_hexapod_control/config/climb_front.json
```

This opens the existing Isaac viewer and does not replace the retained headless evidence.

## Viewer gate excerpt

`viewer_console_excerpt.log` contains the retained **final excerpt**, rather than a complete viewer log, from a separate interactive preview using the same config SHA and nominal 240/30/30 Hz, joint-speed `1.2`, climb-speed `1.0` settings. Its five printed simulator gates passed. It supplements, and does not replace, the headless `gpu_metrics.json` receipt.
