# Repository Guide

## Workspace Rules

- This is a ROS Noetic catkin workspace; packages live under `src/`.
- `grasp_hexapod_description`: URDF, meshes, model configuration, RViz/Gazebo launch files.
- `grasp_hexapod_control`: controller, approach/climb modes, Isaac Gym and real-robot entry points, shared utilities.
- `grasp_hexapod_servo`: verified LX-15D ROS driver and serial protocol. It is externally owned and read-only for current control cleanup; do not edit, format, move, or include it in broad refactors unless the user explicitly reopens that scope.
- Keep non-runtime reference code in `src/reference/`. Put Python in package `scripts/`; reserve package `src/` for future C++ only. Keep runtime/core commands and `validate_climb_preview.py` in `scripts/`, reusable modules in `scripts/utils/`, and offline analyzer/planner/history-validator commands in `scripts/tools/`.
- Use two-space XML indentation. Preserve generated URDF link, joint, and mesh names unless regenerating the complete export. Keep ROS directories lowercase: `config/`, `launch/`, `meshes/`, `urdf/`. Update every `$(find ...)` reference after a package rename.
- Treat generated CAD/URDF artifacts carefully: make focused edits, avoid formatting churn, and document manual edits that a future export must repeat.

## Build, Validation, and Git

Run from the repository root.

```bash
catkin_make
source devel/setup.bash
roslaunch grasp_hexapod_description display.launch
roslaunch grasp_hexapod_description gazebo.launch
roslaunch --nodes src/grasp_hexapod_description/launch/display.launch
roslaunch --nodes src/grasp_hexapod_description/launch/gazebo.launch
```

`catkin_make` builds the workspace; `roslaunch --nodes` checks launch syntax and package references when available. There is no dedicated test suite. Validate relevant changes with `catkin_make`, launch checks, and RViz/Gazebo. For URDF edits, also run:

```bash
check_urdf src/grasp_hexapod_description/urdf/grasp_hexapod.urdf
```

Confirm mesh paths resolve and joint names match `config/joint_names_grasp_hexapod.yaml`.

Use `test` as the integration branch. Create feature branches from and target PRs to `test`; DO NOT develop on `main`. Use short imperative commits, list validation commands, and identify regenerated CAD exports.

## Conversation Handoff

- `HANDOFF.md` is the transient cross-conversation state. A new session MUST read it after `AGENTS.md` and `PROJECT_MEMORY.md`; before handoff, update it. It never overrides safety rules or stable facts.

## Climbing Preview Source of Truth

| ID | Runtime stage | Required action |
| --- | --- | --- |
| C1 | `PREP` | Prepare the compact climbing posture. |
| C2 | `RM` | Move RM from the low support; third base knot is 20 mm lower and crossing is 1.70 s. |
| C3 | `BODY` | Align the body for the paired transfer. |
| C4 | `PAIR` | Move the right pair from the low support. |
| C5 | `LB_LF_GROUND_SHIFT` | Advance LB/LF on the ground. |
| C6 | `LM_GROUND_SHIFT` | Advance LM on the ground. |
| C7 | `BODY2` | Body pitch -0.56 rad prepares RM clearance. |
| C8 | `RM_HIGH_C` | RM high swing; its two air base knots use the verified -5 mm correction. |
| C9 | `RB_RF_HIGH_C` | Move the right pair to the high support. |
| C10 | `RB_BODY_ADVANCE` | Advance the body while RB swings to its next support. |
| C11 | `LM_RM_PRE_ADVANCE` | Lift and pre-advance LM and RM together. |
| C12 | `LB_LF_BODY_ADVANCE_HIGH_STEP` | Move LB/LF high while advancing the body. |
| C13 | `RB_RF_TOP_INWARD_PAIR` | Move the right pair inward on the upper support. |
| C14 | `LM_BODY_TRANSFER` | Transfer the body while LM swings; RM stays fixed. |
| C15 | `RM_RIGHT_SYMMETRY` | Set RM on the right-side symmetric support. |
| C16 | `BODY_LEFT_TRANSFER_PREP` | Prepare the body for the left transfer. |
| C17 | `LB_LOW_STEP` | LB lift reaches roll -pi/12 before transfer. |
| C18 | `BODY_RIGHT_BEFORE_LF` | Restore the body while LF remains fixed. |
| C19 | `LF_LOW_STEP` | LF lift reaches roll +.16 rad before transfer. |
| C20 | `BODY_PRELOAD_LM` | Fixed-foot x preload for .5 s at z .226 m and roll .16 rad; no dip or levelling. |
| C21 | `LM_LIFT` | LM rises while the body levels over 1.2 s, then retains the .45 s transfer. |
| C22 | `BODY_ADVANCE_LM_AIR` | Airborne LM body advance starts from the level z .226 m pose and reconnects the original endpoint. |
| C23 | `LM_LEFT_FINAL_LAND` | LM retained intermediate landing. |
| C24 | `RB_RF_DIRECT_FINAL` | RB/RF paired final swing with synchronized body pose. |
| C25 | `RM_BODY_REPOSITION` | Reposition the body while RM swings to its final support. |
| C26 | `LB_LF_DOCK_TRANSFER` | Transfer LB/LF and dock the body to the final pose. |
| C27 | `STAND_FINAL_HOLD` | Hold the final stand. |

Active SHA `5f8edbc7ba46e5776e800cdfb7b786234ea50e844cf617aaf20e6bbd5489fcce`; the 27-stage plan retains the eight critical left-transfer stages and merges five bounded swing/body blocks with nonzero internal Hermite reference velocity. C2 uses the approved 20 mm lower third RM base knot and 1.70 s crossing; C20--C22 retain the no-dip LM level-and-lift contract. Nominal and `.75` friction/`.10 s` RM-delay GPU previews pass all five gates. Simulation diagnostics only.

## Climbing Change and Preview Protocol

- Before planning or modifying any climb stage, read and apply
  `CLIMB_ENGINEERING_RULES.md`. Every corrected planning error must be closed
  as `symptom -> root cause -> fix -> reusable rule -> regression check ->
  evidence boundary -> reusable technique -> adoption` before the next planning
  batch begins. Append a technique card with applicability, failure signs, and
  measured benefit; then promote repeated checks into preflight assertions or
  tools. A local fix that only records history but does not improve the next
  planning cycle is incomplete.
- Owner: Sol performs physical reasoning, stage semantics, safety boundaries, and final acceptance. Terra may perform bounded repository inspection, mechanical edits/generation, and basic validation under a precise contract.
- Before editing, inspect the complete affected stage chain, support relation, boundary continuity, and target final state. Find the physical cause, then make the smallest change that closes all downstream dependencies.
- Prefer the smallest stage-order or coordinate change that resolves observed behavior. If reachability or support angle is poor, prefer multiple small leg/body motions to one large leg motion.
- Before an Isaac Gym preview, run only syntax, configuration-load, and a few critical IK checks. Let the user inspect motion promptly; use the failed stage name for the next targeted change.
- Current CPU/DLS posture gates prefer axis <=20 degrees and normally use
  <=30 degrees. The active plan's early maximum known-axis is 34.266 degrees.
  The active replay minimum margin is .051379 rad. These are
  kinematic diagnostics, not proof that the entire route is physically safe or
  globally validated.
- Unless explicitly requested, DO NOT run dense whole-robot STL collision scans, long planning searches, or repeated full regressions before the user reviews the preview.
- Use `--climb-speed` only as an explicit preview-time diagnostic multiplier: default `1.0`, range `0.25`–`4.0`; it divides planned durations and MUST NOT change the physics timestep.
- Use `--climb-joint-speed` only for the simulated joint velocity limit: default `1.2`, range `0.5`–`5.0`. The default maps the URDF `4 rad/s` limit to about `4.8 rad/s`; values above `1.2` are diagnostics-only. It does not change planned footholds, durations, or controller gains; current approach/climb control uses the shared `Kp=100`, `Kd=.8`, URDF effort, and velocity multiplier `1.2` baseline.
- The APPROACH `0.45 s` reference is one complete synchronized arc. Compact climbing instead uses vertical lift -> horizontal crossing -> vertical landing; its individual-leg periods are selected by distance and clearance in the `0.45`–`0.85 s` range and are not claimed to be identical to APPROACH.
- Use `--joint-speed` for the ordinary APPROACH simulation joint velocity limit: default `1.2`, range `0.5`–`5.0`. It scales only the URDF `4 rad/s` velocity field; stiffness, damping, effort, gait timing, commands, and compact climbing remain unchanged. Values above `1.2` exceed the LX-15D 7.4 V official no-load-speed candidate and are diagnostics-only.
- Isaac controller and DOF-target writes both default to 30 Hz, with 240 Hz physics, matching the real servo-node cadence. The shared `4 rad/s` controller step cap is a command guard, not the physical servo speed. Real loaded speed still requires joint-feedback measurement.
- Stage-range replay is available with inclusive aliases, e.g. `--climb-from C13 --climb-to C15`; optional `--climb-metrics path.json` records joint tracking plus actual-root/world-foot/support-drift simulator truth. These fields are diagnostics only and never contact/load/stability proof or deployable transition inputs.
- Preserve preview/certification separation. Complete collision, tracking, sensing, persistence, and safety validation only after the preview gait stabilizes and before real use.

## Active Worker Routing

- The only active routing is **Sol + Terra**. Luna, Claude Code, and DeepSeek are neither active routes nor fallbacks in this repository.
- Sol owns requirements, architecture, physical/safety reasoning, the frozen task contract, and final acceptance. Sol works directly on tightly coupled tasks when delegation overhead would exceed the saved work.
- Terra is the single bounded execution worker for repetitive inspection, changes across more than three files, mechanical refactors, tests, and evidence collection when the task can be specified completely in advance.
- Before the first repository write in every user turn, Sol MUST record `execution_owner: Sol` or `execution_owner: Terra` in the frozen contract with a one-line reason. A continuation such as “继续” does not inherit the previous owner when its objective, approved files, or validation scope changes.
- Set `execution_owner: Terra` before editing whenever the planned or actual batch spans more than three files, or contains separable mechanical implementation, test updates, documentation synchronization, or evidence collection. Once Terra owns the batch, Sol MUST NOT use `apply_patch` or perform duplicate implementation/validation; Sol reviews the returned diff and sends contract-scoped corrections back to the same Terra thread.
- Sol may write directly only when the batch is at most three files and is genuinely trivial or inseparable from Sol-owned architecture, physical/safety reasoning, contract definition, or final acceptance. The contract MUST state this exception explicitly. If a Sol-owned batch grows past either boundary, stop before the next write, re-freeze the target tree, and route the remaining execution to Terra.
- Use the `terra` profile in `~/.codex/agents/terra.toml`, one thread, and one writer. Do not run a separate audit worker and then duplicate its implementation in Sol; reuse the same thread for contract-scoped corrections.
- Every Terra contract MUST state objective, approved files, invariants, prohibited changes, validation tiers, expected concise evidence, and stop condition. Terra must stop instead of redesigning or expanding scope.
- Sol MUST set Terra reasoning from the frozen contract: `low` for read-only scans or obvious repetitive edits; `medium` for routine implementation and targeted tests; `high` for coupled multi-file refactors or difficult diagnosis; `xhigh` only for exceptional bounded implementation where extra reasoning is justified. Omitted effort defaults to `medium`. Architecture, physical/safety decisions, and open-ended design remain in Sol.
- Sol reviews the approved-path diff, safety boundaries, and returned evidence. Do not repeat Terra's successful mechanical validation unless relevant inputs changed or the evidence is incomplete.
- Long tasks report at least every 60 seconds. Optimize in order: correctness and real-robot safety, execution efficiency, minimum Sol usage, then total computation.
- `CLAUDE.md` and `CLAUDE_CODE_HANDOFF.md` are historical compatibility artifacts only. They grant no authority, define no active routing, and do not override this section.

## Hardware, Sensors, and Evidence Boundaries

- These are fixed project facts unless the user explicitly changes them; then update `PROJECT_MEMORY.md` in the same change.
- All 18 joint actuators are Hiwonder LX-15D bus servos. Real hardware provides joint-position feedback only; DO NOT assume reliable motor torque/current feedback for contact detection.
- Each physical foot has a hemispherical lower contact surface and cylindrical upper part, not a full sphere. Plan ground and Xiaolan contact in the hemisphere's central region. Final-leg-axis angle to local support normal is a pose-quality metric only, never measured contact patch, contact force, or verified support.
- Sensors: forward-top body Orbbec Gemini 336L depth camera, IMU, RTK receiver, fixed bottom USB camera primarily for docking. DO NOT assume unspecified foot-force/contact or other real sensors.
- Xiaolan is fixed to the photovoltaic panel and has its own RTK. Robot and Xiaolan exchange RTK positions through LoRa.
- First Xiaolan landing region in `base_link_xiaolan.STL`: broad slightly inclined side surface at Xiaolan-frame height about 0.14–0.16 m, not narrow horizontal `z=0.150 m` strips. Negative-x CAD plane: `-0.20791*x + 0.97815*z = 0.19516`; outward normal `[-0.20791, 0, 0.97815]` (about 12 degrees from horizontal). Positive-x side mirrors it. First planning model need not exclude small internal features; erode only Xiaolan outer edge and step boundary toward about 0.18–0.19 m upper surfaces. Avoid upper surfaces where possible, but they are not forbidden: an independently extracted obstacle-free upper-support component is allowed only as penalized late-stage fallback if low-surface-only is infeasible. CAD geometry is authorized for model-level search; real clearance/tolerance margins remain unresolved.
- Distinguish lateral hip/motor protrusion from foot protrusion. Rough 50–60 mm is `rm` thigh-motor/hip mounting beyond `rb/rf`; model hip-origin difference is 42.5 mm (`85 mm` vs `42.5 mm`), with motor-envelope/reference-point differences explaining the rough measurement. Nominal Q_STAND foot-center difference is about 96.99 mm (`rm≈193.97 mm`, `rb/rf≈96.99 mm`), a planning input only until preparation pose is surveyed. DO NOT assume the first paired feet are naturally closest to the edge; offline preparation-pose search MUST consider mounting-envelope clearance and foot-center reach.
- Current climbing assumes known required pose, predefined climbing start pose, and fixed known relevant Xiaolan geometry/initial relative pose. Scope is climbing gait, climbing state machine, sensor-based motion adjustment, and event/transition detection. DO NOT add visual perception, camera inputs, SLAM, or map building.
- Deployable climbing may use real joint feedback, IMU, and RTK/LoRa only. Gemini 336L and bottom USB camera are out of scope for current climbing decisions.
- Use IMU continuously in every climbing phase. Define planned body-attitude trajectory/envelope and expected angular-rate/acceleration behavior. IMU attitude is the primary deployable indicator of physically normal command progress and body settling, never sole evidence of foot contact or load support.
- Throughout climbing, combine robot/Xiaolan RTK/LoRa for centimeter-class phase-level estimate of `base_link` and posture-dependent estimated COM against Xiaolan known frame, edges, and safe deck. Apply calibrated antenna lever-arm correction from robot IMU attitude and Xiaolan orientation. RTK gates require valid/fresh solution, timestamp alignment, uncertainty margins, persistence, and IMU/joint-motion consistency. DO NOT use RTK as a fast foot-contact sensor.
- As development progresses, simulation perception MUST match real sensor availability, frames, update rates, latency, noise, and failure behavior. Inputs used by shared controller or phase transitions MUST be available on the real robot.
- Simulation-only ground truth (root/object poses, foot world poses, PhysX contact forces/points, collision pairs, surface normals) is diagnostics, labeling, validation, and auxiliary safety assessment only. It MUST NOT be the primary observation or sole deployable transition condition.
- Real physics is a hard constraint for every climb motion and event judgement. Explicitly reason about support contacts, load paths/redistribution, COM stability, friction/slip, impact, structural/foot compliance, LX-15D backlash/tracking error, actuator saturation, sensor noise/rate/latency, and plausible failures. Separate observable quantities from model inferences. NEVER declare contact, load support, stability, or phase completion solely from ideal kinematic target or elapsed time; require conservative deployable multi-signal or persistence evidence appropriate to the observable signals.

## Isaac Gym Learning and Development

- The user develops this project personally. Act as tutor, technical guide, and code reviewer.
- Final target and teaching reference: hand-written controller architecture in `src/reference/expert.py`. Guide the user to independently rebuild and adapt it for this robot.
- Retain the reference concepts: command flow, forward kinematics, damped-Jacobian IK, body-motion interpolation, desired-foot planning, tripod support/swing state machine, workspace checks, inter-leg collision checks, and contact/transition criteria. Adapt geometry, transforms, joint signs/offsets, indexing, safe workspace, collision thresholds, and transition criteria to the grasp URDF; DO NOT copy constants blindly.
- DO NOT introduce RL training, rewards, observations, PPO, or a `legged_gym` runtime dependency.
- Teach concepts and tradeoffs before implementation; tie each lesson to current code/model. Use small verifiable exercises with objective, API hints, expected behavior, and validation. Check understanding of ownership, frames, tensor/array shape, units, control semantics, and single versus multi-environment behavior.
- DO NOT provide full copy-paste solutions by default. Give progressively stronger hints when needed. After user implementation, review exact code and explain issues/tradeoffs so the user can correct it.
- Diagnose from code, URDF, logs, and runtime output; explain cause, not only the fix. Follow model inspection and Isaac Gym interface toward the standalone closed-loop `expert.py`-style controller and deployment. Teach tensors/multi-environment only when they directly support that controller.
- At every stage, identify the corresponding `src/reference/expert.py` function/control concept, what remains, and what must adapt. This reference-controller direction overrides generic sinusoidal gaits, unrelated architectures, and RL unless the user explicitly revokes it.
- Preserve the tutoring workflow unless the user requests implementation. Then follow Active Worker Routing; give code for the user to type only when they explicitly choose manual learning.
- Treat an implementation code block as primary teaching material: put concise explanation beside relevant lines/sections, including purpose, current contract/risk, and next dependency. Follow it only with compact validation target and immediate next step.
- Prefer the smallest sufficient implementation: direct readable flow, no premature abstraction, configuration layers, boilerplate validation, or broad defensive exception handling. Add checks/structure only when preventing plausible interface, resource, joint-index, frame, unit, or unsafe-control failure. Keep strictness for kinematics, frames, units, indexing, and control semantics.
- DO NOT add `try`/`finally`, wrappers, factories, or generalized utilities by habit; require a concrete repeated responsibility or clear near-term dependency.

## Active Compact Curve (2026-09-08)

The executable compact plan is 27 stages (`C1`--`C27`), SHA
`5f8edbc7ba46e5776e800cdfb7b786234ea50e844cf617aaf20e6bbd5489fcce`. It merges RB/body, LM/RM pre-advance, LM/body while RM remains fixed,
RM/body reposition, and LB/LF/body dock; the eight left-transfer stages remain
byte-identical to the 68f baseline. C2/C8 and tail air knots retain the
verified 5 mm lower clearance geometry; C2 separately lowers its third RM base knot 20 mm. C21/C22 share the approved world-frame
air velocity; C23 remains an ordinary landing with existing persistence and
checkpoint gates. Curve reference velocity is not measured loaded-servo
velocity; CPU/CAD and GPU preview results remain simulation diagnostics only,
never hardware authorization.

## LM 调平同步抬脚（当前）

当前默认 SHA `f8da102282bffef261bfa0a2fe94b07558121f6c4bb38bc97c1c8c1c3061440f` 从冻结 6761 派生。C20 固定足短 x 预载 `.5 s`，保持 `z=.226 m`、roll `.16 rad`；C21 在 LM 抬起首 `1.2 s` 调平到 roll `0`，高度不降，随后保留 `.45 s`；C22 接回原终点，C23 之后和终态冻结。CPU 局部、generic、DOCK 路径诊断与名义/stress GPU 证据在 `src/docs/evidence/climb_lm_level_20260909/`；均不授权硬件。

## C2 RM 提速（当前）

当前默认 SHA `5f8edbc7ba46e5776e800cdfb7b786234ea50e844cf617aaf20e6bbd5489fcce` 只将 C2 第三个 RM active-base knot 降 `20 mm` 并将第二段 `2.10 -> 1.70 s`，其余阶段与运行控制冻结。120/30 Hz DLS 和 9 点局部 visual CAD 通过；一处 crossing 中点无 STL XY 投影，保留 `null` 而非净空结论。名义与 stress GPU 均通过五门。证据位于 `src/docs/evidence/climb_c2_speed_20260909/`，均不授权硬件。
