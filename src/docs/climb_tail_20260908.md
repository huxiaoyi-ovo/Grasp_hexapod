# Compact climb tail, 2026-09-08

This is a historical 33-stage candidate, superseded by the active 32-stage
curve documented in `climb_flow_20260908.md`.  Its C1--C24 plan retained
its body path while the airborne LM endpoint rises by 15 mm in base z.  C26 retains
the established LM intermediate support, C27 moves RB/RF together while the
body reaches the raised RM-ready pose, C28 moves RM to its inward-adjusted
final support, and C29 is the required fixed-foot body move.  C30 gives LM a
separate 30 mm base-z lift, high transfer, and touchdown at its original final
support.  C31 moves LB/LF together, C32 docks the body, and C33 holds.

The earlier 32-stage one-swing LM alternative was rejected in this candidate:
placing LM directly on its final support at C26 produced a 45.41 degree final
leg-axis angle, above the 30 degree gate.  The restored C26 intermediate
support has a 27.20 degree axis and 38.24 mm hip-foot horizontal radius; the
new C30 final LM landing at the C29 body pose has an 18.02 degree axis.  This
adds one physically necessary LM swing but removes three stages from the
previous 36-stage baseline.

The 33-stage CPU checks pass: strict 30 Hz replay takes 2078 ticks with a
0.030634 rad minimum joint margin and stored-terminal FK error of
1.36e-11 m.  Sparse interior foot clearances are 30.47 mm (C26 LM), 42.23 mm
(C28 RM), and 30.07 mm (C30 LM); the nine altered active-leg non-foot visual
CAD samples and the capsule guard pass.  Under model-only roll/pitch +/-2
degree perturbations the smallest sampled joint margin is 0.02219 rad and
the smallest true-COM support-polygon margin is 63.77 mm.  These are
kinematic/model diagnostics, not contact, load, friction, stability, or
hardware authorization.

For SHA `a4b417fe2263971577fd274fd6af4b861fbf6a9b555c6f62f25013a2512906fd`,
GPU nominal reaches DONE with all five preview gates: motion 69.8333 s and
full 74.1333 s.  The `.75` friction / `.10 s` RM-delay stress probe also
reaches DONE with all five gates: motion 70.0667 s and full 74.3667 s.  This
is only 0.2 s faster than the prior full baseline, so the adopted benefit is
stage simplification and clearance rather than a material speed claim.
