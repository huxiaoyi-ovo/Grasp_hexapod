# Front climb v11 (2026-11-12) — stage-wise robustness edits of v10 and hardware freeze

Deployed config: `src/grasp_hexapod_control/config/climb_front_v11.json`
(SHA-256 `1683c12cf8f36626d789267f6258c1cac6628940916e84ba3f3820078797c6a8`).
Source: `climb_front_optimized.json` (v10, SHA `de855ddf…`). All numbers are CPU/C++/Isaac
simulation diagnostics; none proves contact, load, friction, stability or hardware safety.

## Edits on top of v10 (in order; `front_v11_provenance.edits` in the config)
1. C2/C3 swapped: middle pair advances first; front pair lands on the planar centres of the 12° face
   LF (.242,.100) / RF (.212,−.165) instead of its lower edge (.190). Front-lift COM 21→33 mm.
2. C4 moves RF only to the right wing (.30,−.165) while the body advances (old LF target sat on the
   face/top step, +x tolerance 5 mm). C5 .70 s, C8 ×1.5.
3. Before the last pair climbs: LB ground relay .148, RM advances along the wing to .315 with a body
   pre-shift to (.28, z .22, pitch .40). RB-lift COM 18.6→53.8 mm. Rear high arcs −30 mm / +10 mm outward.
4. C16 LM final .425→.40 (thigh near limit, 11 mm from the planar edge). Joint margin .005→.05+.
5. Rear legs not tucked in: RB (.212,−.225) shank-to-normal 31°→6°; LB (.24,.105) 30°→13°;
   final body advance split (.41 then .45) so RB is not dragged to full extension.
6. Retime: pre-contact knot 8 mm above touchdown, descent accel ≤1.5 m/s², slow touchdown;
   air segments at 2.75–3.3 rad/s reference; flat-ground swings use pass-through (non-stop) Hermite
   curves, step-crossing swings keep their verified segmented paths.
7. C13 front intermediate footholds moved off the top transverse slot: LF (.491,.029), RF (.52,−.165).
8. Hardware retime: stages stretched until a 4 rad/s step-cap replay has zero clips (5 stages).
9. `terminal_q_rad` regenerated from the final anchors; simulation cap set to 4.0 rad/s.

## Verification of the frozen file
| check | result |
|---|---|
| Python validator / CPU 30 Hz replay | DONE, 1179 ticks, 0 clips at 4 rad/s |
| C++ vs Python per frame (`tools/cpp_replay.cpp`) | 1179 ticks both, max 5.0e-6 rad |
| C++ hardware mode (feedback checkpoints, 0.04 m gate, 4 rad/s) | DONE, 39.3 s, peak 3.57 rad/s |
| terminal_q_rad vs replay final q | 2.4e-6 rad |
| Isaac nominal (`--climb-joint-speed 1.2`) | DONE 39.3 s, five preview gates PASS |

Start-error sweeps (Isaac, robot start error relative to Xiaolan; yaw via `tools/yaw_sim.py`, which
rotates the Xiaolan mesh at run time without changing the repository): see `sweep_summary.json`.

## Tools (`tools/`)
`landing_tolerance.py` (per-landing distance to step/edge/slot), `chain.py` (stage-chain re-propagation),
`edit_*.py` (each edit), `retime.py`, `smooth.py`, `hw_retime.py`, `cpu.sh`, `gpu_check.sh`,
`envelope*.sh`, `yaw_sim.py`, `slip.py`, `refspeed.py`, `cpp_replay.cpp`. They expect the original
prototype paths; they are archived for provenance, not as maintained repository tools.

## Open
Accumulated support slip (~3 cm nominal) limits backward start tolerance; RB parks on the ~36 mm-wide
right face; +3° yaw fails at C13; ~6 s of stage-end settle persistence (until foot contact sensors).
