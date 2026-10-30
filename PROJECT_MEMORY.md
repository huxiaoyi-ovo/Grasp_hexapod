# Grasp Hexapod Project Memory

## Purpose

This file stores stable physical facts and the accepted current simulation snapshot. `AGENTS.md` owns workflow and the executable stage map.

## Real Robot

- The robot has 18 Hiwonder LX-15D bus servos. Only joint-position feedback is available; torque/current is not a contact sensor.
- A foot has a hemispherical lower contact surface and cylindrical upper section. Terminal leg-axis angle to the local support normal is a kinematic posture metric, not contact area, force, or load proof.
- Sensors: top-forward Orbbec Gemini 336L, IMU, robot RTK, and a fixed bottom USB docking camera. Do not assume other sensors.
- Xiaolan is stationary on the photovoltaic panel, has its own RTK, and exchanges RTK positions with the robot over LoRa.

## Xiaolan Geometry

- The first landing region is the broad inclined side surface in `base_link_xiaolan.STL`, approximately 0.14–0.16 m high, not narrow horizontal `z=0.150 m` strips.
- Negative-x plane: `-0.20791*x + 0.97815*z = 0.19516`; outward normal `[-0.20791, 0, 0.97815]`; bounds `x=-0.27208..-0.14143 m`, `y=-0.25747..0.17374 m`, `z=0.14168..0.16945 m`. The positive-x side is mirrored.
- Ignore internal small features in the first model. Erode only outer edges and the step boundary to the 0.18–0.19 m upper surfaces. Upper support components are penalized late-stage fallbacks, not normal targets.
- The observed 50–60 mm side protrusion is the `rm` thigh-motor/hip region. Model hip-origin difference is 42.5 mm; nominal Q_STAND foot-center difference is about 96.99 mm. Keep motor-envelope and foot-center geometry distinct.

## Deployability Boundary

- Current climbing assumes a known fixed start pose and known Xiaolan geometry/relative pose. Cameras, visual perception, SLAM, and mapping are out of scope.
- Deployable inputs are joint position, IMU, and RTK/LoRa only. IMU is continuous attitude/settling evidence, never contact/load proof. RTK is phase progress only and needs freshness, alignment, uncertainty and persistence gates.
- Simulation truth is diagnostic only. It must not be a primary controller input or sole phase gate. Real motion decisions require support/load-path, friction/slip, impact/compliance, backlash/tracking, saturation, and sensor-failure reasoning.

## Current Simulation Preview Snapshot

- 2026-10-27 planning directive: experimental front-route planning may explicitly retain intentionally airborne feet until a support is needed or available; it must declare a conservative planned support set rather than require all six feet to land in every phase. These roles are not contact, load, stability, or hardware evidence. The user has fixed the coarse high/low boundary criterion as the external step/riser, so no semantic-choice question remains; exact planning polygon, conservative height, and tolerance remain a geometry task. This planning abstraction does not alter the real Xiaolan CAD facts above. Future simulation diagnostics must include initial-pose/landing-offset robustness and a normal-tripod-walk tail; deployable sensing and hardware rejection remain unchanged.

- 2026-10-27 top-visible Xiaolan CAD review (geometry only): the primary exposed back flat is G2 at about `190 mm`. Major exposed upper wings G4/G6 are each `5.384 deg` inclined and sample-visible over about `171--190 mm` / `176.6--189.8 mm`. Raw upward G1/G3 at `184.5/182.5 mm` are sampled `0%` top-visible beneath G2 and are not external platforms. The separate outer first-landing G5 is the `12 deg` surface at about `141--169 mm`. At local `y=+.106 m`, the G2-to-G5 outer-slope top-envelope difference is `20.548994 mm` at distinct X positions; the local section contains both riser and bevel faces, is not a universal height or a confirmed pair of user-intended back platforms, and does not uniquely locate the external divider. There is no fixed `40 mm` back-step fact. The user-supplied external-riser criterion removes the semantic-choice blocker; exact geometry location, planning polygon, height, and tolerance remain open. The STL is unchanged.

- Current compact candidate SHA-256 `af3dca695cf61279af7e41a79595ce0d5b2c8415c0d8de8e7db8df61fc1b944c`: C20/C22 complete planned height and roll at their 1.4 s lift end and hold through foot transfer; their transfer durations are `.50/.55 s`. C20 ends at `-pi/12`, C21 returns level in 1.2 s, C22 retains `+.16 rad`, and existing FK knot gates remain the progression check; this is not IMU-attitude proof. C22 LF landing moves `(+.010,0,+.002126) m`, fixed through C34 takeoff. C23/C24 add `+.005 m` body x and C25 returns to its old endpoint. CPU strict replay is `2084` ticks with `.030634 rad` global margin; true-COM C23 terminal future support `.030072 m` and C24/C25 air support `.029662 m`. Current GPU PhysX C1--C36 is `DONE` in `74.3333 s` with all five preview gates passing; metrics are retained in `src/docs/evidence/climb_speed_20260908/` and remain simulation diagnostics only.

- Historical pre-lift-roll baseline: SHA `d603f9f4405d2bfffd55faf85cbbf011aa08ded18f6714dd92be574a86482277`, its `2497`-tick replay, and its targeted C7--C9 GPU metrics are not current-candidate evidence.
- The compact plan has one source JSON and a launch/runtime `climb_side=left|right` selector. The old `2497`-tick left/right mirror values are historical baseline evidence; mirror behavior must be rerun for SHA `771236...` before it is claimed current. Right-side GPU PhysX and hardware remain untested and unauthorized.

- Active compact SHA `67616b7056329c0f65723962befa5166c4d162c45ca0b85fc79aad516aed9edf` has 27 stages, merging five bounded swing/body blocks while retaining the eight left-transfer stages and final body/feet byte-identically from 68f. C23's retained LM intermediate support remains the final LM anchor; C21/C22 carry approved continuous air-reference velocity and eligible swing interiors use stored Hermite tangents. C2/C8 and tail air-knot lowering remain trajectory/workspace tradeoffs. Nominal and `.75` friction/`.10 s` RM-delay GPU previews reach DONE with five preview gates; the stress maximum foot-target error is `.03051465 m`, and remains simulation-only rather than a full-trajectory robustness certificate. Evidence is `src/docs/evidence/climb_concurrency_20260908/`.
- Active config, Isaac and `control_stack` use a `20 mm` FK foot-task gate; `run_real.launch` temporarily applies an in-memory `50 mm` override. The gate blocks progression only at existing segment knots and the final endpoint, never inside a segment; final persistence remains at least `.25 s`, timeout remains nominal duration plus `5 s`, and `.08 rad` joint tracking is diagnostic only. A 1 Hz `base_link` actual/desired/signed-error trace distinguishes target geometry from following error diagnostically, not as contact, load, or causal proof.
- The last full-chain CUDA extrema (`C5` tracking/foot, `C25` root, `C33` world-foot, `C7` support drift) belong to prior SHA `46bd1bed...` and are historical after the C7--C8 edit. Current targeted C7--C9 CUDA gives C8 minimum joint margin `.029766 rad` and C7 support-foot drift `.02191 m`; a current-SHA full-chain rerun is still required. These are simulation diagnostics only, not contact/load/stability or hardware authorization.
- Controller and replay run at 30 Hz with 240 Hz physics; target-step guard is `4 rad/s`. Current control uses shared Approach `Kp=100`, `Kd=.8`, URDF effort and velocity multiplier `1.2`. Loaded real behavior still needs suspended feedback measurement.
- A small historical lesson remains: prior 54/56/59-stage trials are superseded simulation history, including the rejected LM contact deletion. They are not current trajectory evidence.
- Current PhysX contact calibration keeps `contact_offset=20 mm`, `rest_offset=0`, robot collision thickness `1 mm`, friction offset threshold `1 mm`, and friction correlation distance `0.5 mm`; ground/Xiaolan friction remains `1.5/1.2` and `1.0/0.8`.

- Active LM 调平 SHA `f8da102282bffef261bfa0a2fe94b07558121f6c4bb38bc97c1c8c1c3061440f` 从冻结 6761 派生：C20 `.5 s` 固定足短 x 预载保持 `.226 m/.16 rad`，C21 用 `1.2+.45 s` 在 LM 离地时调平且不降身，C22 接回原终点，C23 后和 terminal 完全冻结。局部五支撑 `.03079897 m`、关节 `.13666168 rad`、generic DONE 1743 ticks；名义和 `.75` friction/`.10 s` RM-delay GPU 均 PASS 五门，stress 的 RM-delay 仅在 `LM_LEFT_FINAL_LAND` 激活一次。只属仿真诊断。DOCK 正常 terminal 入口的 40 mm 上升加 20 mm 搜索圆为 243 点模型抽样，未覆盖 TF 预对准/下降；任意 HOLD Y 入口和自动 mission 未 enter DockMode 仍是未解决路由风险。

- Active C2 提速 SHA `5f8edbc7ba46e5776e800cdfb7b786234ea50e844cf617aaf20e6bbd5489fcce` 从 f8da 派生：只将 RM 第三个 base knot 降 `.020 m`，第二段 `2.10 -> 1.70 s`，并重建 world knots/tangents。120/30 Hz DLS、局部 CAD、名义与 `.75` friction/`.10 s` RM-delay GPU 均通过五门；全流程 `61.166667 -> 60.766667 s`。C1--C4 headless native timing 显示 PhysX `simulate` 随 C1 `4.066 ms` 到 C4 `5.625 ms` 每步增加，fetch `.003903 s`、controller `.435807 s`，不支持 metrics 增长或渲染归因，也未修复实时播放。所有结果仍是仿真/播放诊断。
