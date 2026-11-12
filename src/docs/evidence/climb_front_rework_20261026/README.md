# Front-climb rework diagnostics through v10

## Latest v10 truth — NOT accepted for foot support or hardware

Frozen v10 is [`de855ddfd3ac8df9f25f32d5c497601ffc0f452fb44e2bdeec9e9931bbe5e725`](v10/manifest.json). Its one valid visible GPU diagnostic reached C19 `DONE` in 39.3 s with true exit `0` and all five preview gates PASS, but it is not accepted: LB and LF had zero mapped simulator foot-normal force over 60 DONE samples, and LB's independent visual signed gap was `+2.20919 mm` (above the `+2 mm` limit). Front hardware remains blocked.

The immutable [v10 manifest](v10/manifest.json) and [recorded observability report](v10/observability/report.md) preserve the valid run and its simulator-only boundary. The next dependency is a guarded-touchdown design using deployable joint feedback plus IMU and fresh/aligned robot/Xiaolan RTK-over-LoRa; real IMU/RTK calibration, rate, latency, noise, and failure behavior remain pending user data.

## Historical v4 snapshot and earlier candidates

All artifacts here are experimental failure evidence, simulation/CAD diagnostics only, and do not authorize hardware. Current v4 (`7ae297…1cea5`) is rejected: visible GPU reached DONE but preview root gates failed (end `.639931 m`, global `.682526 m`); C14 first degraded (root 85 mm, foot 53.6 mm, 60 clips) and C15 attitude reached 116 degrees. Hold/end-foot at a fallen pose is not success.

User constraints retained for the next Sol-owned replan: globally minimize unnecessary steps; one-shot final feet when feasible; concurrency only with physical stability; ample mid/late clearance and patch margins; final actual CAD-bbox XY `(.45,-.063)`; high lifts while supports remain contacted and extend body within reach/torque/load limits; body/hip-relative active-foot planning with world anchors only for fixed support/contact constraints; use verified SIDE retract/tilt/air-foot preadvance mechanisms. No perception, SLAM, RL, or hardware execution. Simulation truth is diagnostic only; deployable signals remain joint feedback, IMU, RTK/LoRa.

CAD audit corrections: default identity Xiaolan frame produced false object routing; explicit T plus vertex/bounds assertions fixed the audit, not the route. Initial MID object selection targeted supporting RF (`mk[1]`) rather than active M (`mk[2]`); the hash-bound ankle component 10 is physical footpad and cannot be globally ignored. Corrected active-air queries were clear; raw support-footpad intersection remains history. Geometric insets/patches are not contact/load proof. No physical fix is claimed closed; global replan remains open.

## Historical status ledger (2026-10-26)

| Candidate | Hash | Status and bounded evidence |
| --- | --- | --- |
| Canonical direct 21 | `4421aff…` | Accepted simulation candidate: latest visible 29.1 s replay passed all five preview gates; still simulation-only, not contact/load/stability or hardware proof. |
| Original space 19 v6 | `321a1b…` | CPU-only: 855-tick DONE; selected CAD review pending; no GPU/hardware claim. |
| Optimized v1 | `c1e586…` | Rejected: CPU 979-tick DONE, but GPU C17 same-side fore+mid failure; end/global root errors `.34864/.349016`. |
| Optimized v2 | `35fe…` | Rejected: CPU MID ankle failure. |
| Optimized v3 | `9124…` | Rejected: MID local pass, then RB continuous-ankle failure. |
| Current optimized v4 | `7ae2970e1f4ee6ec5044b5b9eee877e04b00ac40afb997222fad885ceca1cea5` | Rejected: local CPU tails plus 55 corrected scoped CAD queries passed, but visible GPU first degraded in C14 and fell later. |

Original accepted/default configurations remain unchanged. The immutable `history/current_v4_config.json` and `history/current_v4_builder.py` snapshots are failure evidence; no physical fix is claimed from their local checks or any archive item.

## Runtime-gate correction and next boundary

SIDE simulation also has `settle_gate.preview_time_only_stage_advance: true`; it is not already simulation-feedback-gated. Its hardware branch has the 20 mm FK checkpoint, `.25 s` persistence, and `5 s` timeout; with the simulation flag false, only the terminal FK gate is selected. Neither establishes contact or load. Proposed next adoption: a FRONT simulation tracking gate while the hardware guard remains rejected. Future deployable multisignal work may use joint feedback, IMU, and RTK/LoRa; simulator truth remains diagnostic only.

The unresolved v4 diagnosis is physical contact plus reference-coordinate/handoff, not merely timing or IK: actual C14 LM gap was about 5 mm while ideal body-q static force LP was feasible; that is no proof of torque insufficiency or a single cause. Actual C14 body held near `(.21,-.051,.239)` versus plan `(.29,-.0677,.259)`. A five-foot polygon margin was positive while some intended supports were away from the surface. Raw actual-q/root samples are archived; a new body-relative replan must first close this diagnosis.

## v5 archived CPU rejection; v6 departure-order screen

v5 is rejected by CPU, separately from v4's visible-GPU C14/C15 fall. Its single continuous replay stopped at C11 `MID_PLATFORM_285`, tick 461, on actual active-foot sphere gap `-1.148276 mm`; its v5 config/builder, raw CPU receipt, console, and baseline-test witness are immutable under `v5/`. The original `history/` directory and its 12-entry manifest were not changed.

v6 is an open, model-screened correction only: before the prior high preadvance, C11 will retract both active feet 20 mm in world x and raise them to world z `.095` with body x held `.130`, then retain the later v5 path. The 25-sample planned screen reports a 15.611 mm minimum sphere gap, maximum IK residual `8.90e-6 m`, and minimum margin `.044976 rad`. It remains neither contact/load/stability evidence nor hardware authorization; CPU then CAD/GPU review are still required.

## v6 C15 limit rejection; v7 hip-polar screen

v6 clears the earlier C11 departure gate but is rejected by its cached local CPU replay at C15 `LB_PLATFORM_225`: LB ankle reached `2.094 rad` at tick 409. Its immutable config/builder/raw CPU/manifest are under `v6/`; this is distinct from v4's GPU fall and v5's C11 sphere-gap rejection.

v7 is an open C15-only hip-polar reuse screen: initial high theta-zero retreat, high terminal-theta turn, `.35/.65` polar arc, normal-offset approach, and unchanged terminal support. The model screen is not a contact/load/stability claim; one cached-entry CPU replay is required before any CAD/GPU review.

## v7 GPU rejection; v8 same-flat final-LF allocation screen

v7 (`b2351359…`) is rejected by its one compatible visible GPU replay despite its bounded CPU/CAD evidence: C18 `BACK_PAIR_CENTER` held at a feedback checkpoint and timed out after 5.4667 s (RM target error `.0480423 m`, LM `.0296214 m`); C19 was not reached. Its immutable raw builder/config, CPU/CAD/GPU/preflight artifacts and hashes are in `v7/`. This is distinct from v4's earlier GPU fall.

v8 is a one-foot allocation screen only. It replaces final LF's lower-broad target `(.618,.033986,.17045)` with CAD projection of the shorter preferred-body offset `(+.100,+.088)` from CAD centre: `(.550,.025,.1964999976)`, on the same flat upper face as the observed v7 actual LF. All body poses, timing, other feet, feedback gates, and runtime remain frozen. The nominal and `x-.040` lag points require same-component, normal-z `>.999`, surface/gap, and planar inset `>=15 mm` evidence. This is geometry/model screening, not contact, load, friction, stability, or hardware proof.

## v8 GPU completion rejected by actual-foot geometry

v8 (`661def8c…`) passed its focused generation/test, hash-bound C17 suffix CPU, and 16 selected C17/C18 CAD witnesses. Its one visible GPU run at diagnostic joint-speed `1.8` reached C19 `DONE` at 39.3 s and printed PASS for all five preview gates. That does not make it accepted: exact final actual-foot classification found nonpenetrating gaps but failed the 15 mm inset/upward-face diagnostic. LF was still only `9.925 mm` from its flat-component boundary; LB's nearest component was vertical and RB's was a tiny edge component. Final root XY was offset `(-.035328,.002026) m` from CAD centre. The raw q/root observer and exact nominal/actual face/component/inset report remain under `/tmp/grasp_front_v8_sdRyAI/`. This is a simulation-geometry diagnosis only, not a contact/load/stability or hardware conclusion; no rerun, retime, or automatic route change follows.

## v9 global terminal-region allocation screen

v8 raw evidence is immutable in `v8/`; it remains rejected. v9 does not retain LB's legacy terminal target: its 75.9-degree terminal foot-axis diagnostic was poor. The candidate keeps the 19-stage sequence and all early motion frozen, but allocates the complete terminal state to robust large-patch XY regions under a full along-route/lateral uncertainty rectangle. The reusable offline auditor rejects the v8 LF box that crossed a hole and requires connected coplanar component identity, positive-up normals, area, planar inset, and signed near-surface gap bounds. CAD/simulation geometry remains diagnostic only, never proof of contact, load, friction, stability, or hardware safety.

## v10 contact-order screen

v9 is rejected: its GPU route reached DONE but LB's actual final signed gap was `+2.0805 mm`, exceeding the unchanged `+2 mm` bound. The direct-C15 LB alternative is also rejected: static IK passed, but exact LB ankle/LM knee collision occurred at its full-theta high knot. v10 keeps all v9 terminal patches, body trajectory, timing, and 19 stages, but moves LB with LF/RM in C17 and RB alone in C18. This is a contact-order diagnostic screen only; it remains neither contact/load/stability nor hardware proof.

## Current v10 truth ledger — preview completed, foot support NOT accepted

The frozen v10 configuration is `de855ddfd3ac8df9f25f32d5c497601ffc0f452fb44e2bdeec9e9931bbe5e725`; the matching runtime snapshot is `4a3ec853a8868e911e8e6d6b4b80845afef53b4b24207ce2859882bb03035db1`. One valid visible GPU diagnostic completed C19 `DONE` in 39.3 s with true exit `0` and all five preview gates PASS. This is not acceptance: final component/area/positive-up/inset diagnostics pass, but the independent full visual sphere-gap gate fails at LB `+2.20919 mm` (limit `+2 mm`). LF's `+.98208 mm` visual gap is within the geometry interval but is not a force result.

The observer's last 60 DONE samples have simulator-only mapped foot-normal means LB `0`, LF `0`, LM `17.7693`, RB `13.5293`, RF `11.1398`, RM `1.6335` N; their summed world-z force is `43.9559 N`, close to model gravity `43.9694 N`. Thus four legs carry the observed simulation load while LB/LF do not. Geometry/FK, simulator force, and preview completion must remain separate evidence layers; none proves real contact, load, friction, stability, or hardware readiness. Independent rigid-body foot world positions were not logged, so FK-vs-PhysX foot-origin comparison is unavailable without an unauthorized rerun.

Immutable v10 snapshots and raw receipts are in `v10/` with `manifest.json`. The initial stance LP is explicitly INVALID_FORMULATION (not physical infeasibility), and the first tensor observer is explicitly INVALID_OBSERVER (interrupted before valid `prepare_sim` ordering). The valid observer prepared the simulation once before tensor acquisition and records simulator truth only. The recorded-data report at `/tmp/grasp_front_v10_observability_C0rI5U/report.md` finds LB/LF zero force despite mean FK reference norms `.364/.203 mm`; it intentionally defines no deployable contact threshold.

The next dependency is a Sol-owned guarded touchdown/load-probe architecture using only real joint feedback, IMU, and fresh/aligned robot/Xiaolan RTK-over-LoRa packets with candidate/persistence/timeout/failure semantics. Existing navigation inputs publish at configured 10 Hz but are not connected to the controller, no real calibration or latency/noise/failure model is known, and simulator GT cannot become a transition input. Stop gait-point micro-tuning until that evidence model is frozen; front hardware remains rejected.
