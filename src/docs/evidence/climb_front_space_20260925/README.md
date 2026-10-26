# Front space candidate v5

Status: **CPU replay DONE; simulation diagnostics only**. Tier 0 compilation/config validation/builder reproduction and four focused tests passed. One continuous 30 Hz CPU replay completed all 19 stages. No GPU or hardware run occurred. v1 through v4 CPU receipts remain preserved.

- Source config SHA-256: `4421aff133035f358e69ed33ddaa649d580750d868f2318736efc17e797bf60c`
- Candidate config SHA-256: `990ee38578d538150ba0b87aadf08f4890be7f902b81010e5b6959eeecd78a78`
- Runtime SHA-256: `225009277f1498d031409e25787056ab003d4c6a74d6380cfc847195690541a6`
- 19 stages; reference duration including settles `28.278799 s`; CPU replay 855 ticks (28.500 controller seconds).
- Sole v5 geometry/body change: `RB_GROUND_TO_155` endpoint x `.205 -> .225 m`; z, roll, pitch, feet, and timing unchanged. `LB_GROUND_TO_155` starts from the propagated `.225 m` body pose and retains its old endpoint.

## v5 symptom-to-adoption closure

- **Symptom:** v4 entered `LB_GROUND_TO_155` with COM support margin `0.427 mm` after RB landed at x `.155 m`.
- **Root cause:** RB's landing created the next support arrangement, while LB began unloading before the body had completed its existing forward travel; the next-stage support margin was not established at the preceding stage boundary.
- **Fix:** complete that existing `20 mm` body advance in C14 by moving its endpoint x to `.225 m`. This adds no stage, step, or return motion; C15 inherits the new start pose. CPU v5 completed all stages.
- **Reusable rule:** check the support set available after the next stage unloads a foot at the preceding stage exit; active-support margin alone can miss an imminent weak incoming support.
- **Regression check:** the four focused config/runtime tests pass, and the continuous CPU replay covers C14-to-C15 stage propagation; C15 minimum sampled COM margin is `0.015755 m`.
- **Evidence boundary:** CPU COM is a model diagnostic. DONE does not establish real contact, load transfer, friction, stability, or hardware safety.
- **Reusable technique:** evaluate incoming support margin at each stage entry using the post-unload support set, alongside the current stage's active/fixed-foot checks.
- **Adoption:** when adding the future CAD audit script, emit incoming support margins at every stage entry so preceding-stage pose changes can be reviewed against the next unload.

## v5 CPU stage metrics

| Stage | Min joint margin (rad) | Raw peak (rad/s) | Clips | Min COM margin (m) |
|---|---:|---:|---:|---:|
| PREP | 0.374306 | 4.022803 | 0 | — |
| FRONT_LOW | 0.046864 | 4.307314 | 0 | 0.020975 |
| BODY_A | 0.016990 | 3.618232 | 0 | — |
| M_GROUND_FIRST_MINUS02 | 0.025670 | 3.084209 | 0 | 0.096663 |
| FRONT_TO_029 | 0.381442 | 4.846416 | 0 | 0.015511 |
| SIX_SUPPORT_BODY_ADVANCE | 0.326497 | 5.179058 | 0 | — |
| BACK_TO_MINUS09 | 0.127269 | 3.237461 | 0 | 0.012665 |
| MID_GROUND_TO_080 | 0.127683 | 4.255676 | 0 | 0.096695 |
| FRONT_TO_034 | 0.438049 | 4.124045 | 0 | 0.048800 |
| BODY_TO_105 | 0.266012 | 5.332778 | 0 | — |
| BACK_TO_055 | 0.102005 | 2.303539 | 0 | 0.015484 |
| MID_PLATFORM_285 | 0.051873 | 2.975929 | 0 | 0.069510 |
| FRONT_TO_0400 | 0.090312 | 2.163643 | 0 | 0.099493 |
| RB_GROUND_TO_155 | 0.120397 | 1.642334 | 0 | 0.042130 |
| LB_GROUND_TO_155 | 0.218616 | 1.880085 | 0 | 0.015755 |
| FRONT_TO_0480 | 0.202699 | 2.156175 | 0 | 0.051930 |
| RB_PLATFORM_225 | 0.043598 | 3.518109 | 0 | 0.031707 |
| LB_PLATFORM_225 | 0.053053 | 3.134307 | 0 | 0.036882 |
| FINAL_HOLD | 0.397633 | 0.040125 | 0 | — |

Terminal IK residual is `4.07e-6 m`, terminal joint margin `0.397583 rad`. Landing sphere-gap tolerance remains CAD numeric tolerance only. `build_receipt.json` binds source/config/runtime/builder/tests and the CPU result.

## 2026-09-25 v6 pause checkpoint

The frozen v6 candidate changes only the C12 RM air-point heights: knot 2 `.180 -> .183 m` and knot 3 `.205 -> .208 m`. Config SHA-256 is `321a1b762d99659be988defa5bfdc27ae9672fc215bb351db9260a8cfd9bc333`. Its CPU replay is DONE: 855 ticks, 19 stages, 28.5 controller seconds. C12 sampled crossing minimum is `16.960 mm` planned-anchor gap and `17.155 mm` FK gap under the planned root; this exceeds the 15 mm minimum but not the 20 mm preferred margin. Selected exact mesh review is incomplete and no GPU run occurred. The user requested pause; resume the pending CAD review only under a new continuation. No commit or push was made.

## v6 integration status

The tracked `climb_front_space.json` is the frozen v6 candidate
`321a1b762d99659be988defa5bfdc27ae9672fc215bb351db9260a8cfd9bc333`.
`cpu_space_v6.json` binds this SHA before and after its 855-tick, 19-stage
CPU replay (`DONE`). The preceding v5 narrative, metrics, mesh cache, and
`space_audit_v5.json` remain historical evidence bound to
`990ee38578d538150ba0b87aadf08f4890be7f902b81010e5b6959eeecd78a78`;
they are not v6 CAD-clearance evidence. `build_receipt.json` preserves its
build-time status and historical ownership. Selected exact CAD review remains
incomplete; no GPU or hardware run is recorded for v6.
