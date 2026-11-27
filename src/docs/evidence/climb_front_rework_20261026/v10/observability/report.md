# v10 recorded DONE-hold observability analysis

Inputs are the last 60 C19 `DONE` samples from the valid single GPU run.  The
input sample SHA is `111d3c20ced1e92e2d768acdfd497f066318179e0477b08139f5de4265c78c5a`;
the frozen configuration SHA is `de855ddfd3ac8df9f25f32d5c497601ffc0f452fb44e2bdeec9e9931bbe5e725`.

For every saved row, desired base-frame feet were reconstructed only as
`inverse(applied_planned_base_pose) * applied_mode_anchors_world`; actual feet
were FK from logged actual `q`.  No desired joint target was logged or inferred.
The table reports mean FK error norm / mean error projected onto the actual
CAD-face normal expressed in the planned base / mean saved simulator net force
projected onto that face normal:

| foot | FK norm (mm) | normal residual (mm) | normal force (N) |
| --- | ---: | ---: | ---: |
| LB | 0.364 | -0.229 | 0.000 |
| LF | 0.203 | -0.080 | 0.000 |
| LM | 2.549 | 1.534 | 17.769 |
| RB | 1.406 | 1.132 | 13.529 |
| RF | 1.154 | 1.042 | 11.140 |
| RM | 0.106 | 0.034 | 1.634 |

This unperturbed recorded hold does not select a joint/FK-only completion or
contact discriminator: two zero-force feet coexist with sub-millimetre
reference/FK residuals.  That observation is simulator diagnostic evidence,
not a real-sensor threshold or contact/load claim.

Saved root truth was nearly stationary: mean XY `(.415284, -.057181) m`,
mean 30 Hz XY finite-difference speed proxy `2.925e-6 m/s`, and mean quaternion
finite-difference angular-speed proxy `1.638e-5 rad/s`.  Root truth,
quaternion, and net contact force are simulator-only diagnostics.  The
requested independent foot rigid-body world poses were not logged; no late
rerun was made.  The record contains no IMU/RTK calibration, rate, latency,
noise, failure model, or desired joint target, so none is inferred here.

The machine-readable per-sample aggregate/statistics report is `analysis.json`
(SHA-256 `76e21e4ed434a772dc1330a282b83e33d3ebadc4158a4105974dd49ac9122677`).
