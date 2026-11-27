# Phase-1 deployment review (offline only)

Date: 2026-10-30.  This is a source/build and isolated ROS transport review;
it did not start production launch, physical camera, serial servo, joystick,
simulation, or hardware.

## Flow checked

`xiaolan_yolo_node` plus aligned-depth `xiaolan_target_3d_node` publish the
generated `XiaolanTarget` (`header`, `detected`, optical-frame `x_target` and
`z_target`, box fields, and `status`) on `/xiaolan/target`.  The opt-in C++
branch consumes it through `home -> spin_search -> coarse approach -> HOLD`.
It never makes the robot ready for climb, dock, or AprilTag completion.

Static launch expansion confirms that production `phase1_visual.launch` starts
`joy_node`, both servo nodes, remote control, and the control node.  It was
not run.  The hardware-free bench defaults to `/bt_control_node` and
`/phase1_fake_feedback`; its depth/sequence variant additionally starts only
Target3D and the sequence helper.

## Review outcomes

Tier 0 parsed current Python/launch/YAML source and passed `git diff --check`.
The Xiaolan CMake install rule still references a missing `docs/` source
directory, so install-space/clean-Jetson packaging is not verified.

Tier 1 narrow build and C++ tests passed: 9 visual-guidance, 13 mode-framework,
and 7 routing tests.  Perception unit tests ran 15 total: 12 passed and 3
skipped.  Tier 2 fresh-master fake-feedback runs passed: startup loss handling
(`.317 s` no feedback, `.515 s` home loss), direct visual motion spans
`.53333467/.53642655`, and RGB-D derived target age `.046 s`, `x=0`,
`z=.5502`.  These are transport/controller evidence only, not camera,
recognition, calibration, loaded-servo, clearance, contact, or climb evidence.

Seven code-level deployment findings were recorded; no runtime fix was made in
this review. Severity is deployment priority, not a hardware incident claim:

- **P1 — perception-health missing during search.** With no callbacks, search
  commands yaw for 60 s. A healthy explicit `NOT_FOUND` must be distinguished
  from an absent/dead perception stream before direct deployment.
- **P2 — stopping boundary has no tolerance.**
- **P2 — target frame is not validated.**
- **P2 — blind-search yaw bypasses the configured maximum.**
- **P2 — asynchronous status handoff is unsynchronised.** This is a static
  data-race finding, not a TSAN-observed crash.
- **P2 — deployment wrapper does not forward `left_port`, `right_port`, or
  `baudrate` into `control_only.launch`.**  A `--dump-params` probe supplied
  `/dev/example`, `/dev/example2`, and `57600`, yet retained `/dev/ttyTHS0`,
  `/dev/ttyACM0`, and `115200` on the two servo nodes.
- **P2 — Xiaolan install rule references a nonexistent source `docs/` dir.**

- At `z=double(float(.60))`, `VisualGuidance::Approaching` returns forward
  `7.15256e-09` rather than settling and eventually fails its 60 s timeout.
- `search_yaw_rate=.3` yields yaw `.3` although `max_yaw_rate=.15`.
- A valid target with frame `map` is accepted and commands positive forward
  motion, rather than requiring the camera-optical frame.
- With no target callbacks at all, search commands positive yaw through 59 s
  and fails only after the 60 s search timeout.  The normal launch defaults can
  leave camera/perception off, so a sequence can rotate blindly while the
  controller only knows joint-feedback health.

Static inspection also finds an unsynchronised `last_finished_*` handoff:
`bt_control_node.cpp:503-505` write it under `bt_mutex_`, whereas
`publishModeStatus` reads it at `:599-614` without that lock while the node
uses an asynchronous spinner.  This review did not add TSAN or refactor it.

The port-forward probe accepts the override and exits zero, but `--dump-params`
retains the serial defaults above. `phase1_visual.launch:10-16` forwards only
visual/climb/dock arguments; `control_only.launch` owns the actual serial
arguments at `:83-85` and passes them at `:167-170`.

See `validation.txt` for exact commands, provenance hashes, and raw receipt
locations.  The pre-existing CPU ONNX smoke is referenced only; it was not
rerun because its model/runtime inputs did not change.
