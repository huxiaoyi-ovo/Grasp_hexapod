# Compact climbing speed update

The active compact plan is `af3dca695cf61279af7e41a79595ce0d5b2c8415c0d8de8e7db8df61fc1b944c` and preserves all 36 stages, footholds,
pose geometry, joint limits, 30 Hz/240 Hz timing, PD gains, and collision guard.
It reduces C1--C35 GPU replay time from 85.2333 s to 70.0333 s (-17.8334%).
The full C1--C36 replay is 89.5333 s to 74.3333 s (-16.98%).

The change uses seven fixed-foot body-stage persistence overrides, shorter
verified body/transfer/touchdown durations, and no new trajectory shape. C20
and C22 retain their 1.4 s lift so planned body roll is complete before their
foot transfer; their transfer durations are 0.50 s and 0.55 s. No nonzero-speed
corner blend was added: the existing knot screen found one 2.3 degree corner at
a touchdown checkpoint and every other corner at least 18.9 degrees, with many
near 90 degrees. Changing them would require new clearance and checkpoint
evidence.

The refined GPU replay reached `DONE` with all five preview gates passing:
terminal foot/root/global-root errors `.00108306/.0267842/.0267845 m`, minimum
joint margin `.0298461 rad`, maximum FK foot-target error `.011175966 m`, maximum
support-foot drift `.013348896 m`, and zero velocity clips. Compared with the
baseline, terminal root error increases about `2.8 mm`, maximum FK foot-target
error about `.51 mm`, and support drift is effectively unchanged
(`.0133482345 -> .0133488956 m`). CPU capsule-guard
replay reached `DONE` with zero guard holds and `.0300747 rad` minimum margin.

The default retimer now audits and preserves the accepted durations while still
running its existing 30 Hz hard-speed and active-foot tracking checks. Use
`--explore-durations` to explicitly generate a new duration proposal; this does
not relax any hard gate. Evidence files and hashes are in
`src/docs/evidence/climb_speed_20260908/`.

All results are model and simulator diagnostics. They do not authorize loaded
hardware motion or establish contact, load support, friction, stability, actual
servo velocity, or IMU attitude completion.
