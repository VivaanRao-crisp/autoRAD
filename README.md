# autoRAD - Main concept (secondary reference cable) - bench-rig prototype

**Start here:** `docs/design_spec.md` (what to build), `docs/buy_list.md`, `docs/demo_sheet.md`,
`docs/img/board_template_1to1.pdf`. Checks: `python3 bench_check.py --template`. History: `iteration_log.md`, `CHANGELOG.md`.

**Print files:** `exports/stl/` (ready for the slicer), `exports/step/` (for Fusion / Inventor), list in `parts_list.md`.
Made by `cad/bench_parts.py`; checked by `cad/check_stl.py` and `cad/bench_assembly.py`. Needs `pip install cadquery trimesh networkx scipy matplotlib`.

---
*Older material below (design study of architectures A-E and the full-attachment option D) is kept for the report.*

All values are PRELIMINARY. Nothing here is validated; only prototype tests can do that.
Design reasoning: `docs/main_design.md` (rev 2).

## Run
```
python3 kinematics.py
```
Needs only the Python standard library for now. CadQuery and cq_gears are needed from the parts step.

## Files
- `params.py` - every dimension and design variable (units in comments, TODOs marked)
- `kinematics.py` - angle mapping, bell-crank geometry, carriage force balance, slot (cam) profile,
  pressure angle, PASS/FAIL checks, plus a short reference table for the spool+gear architecture A
- `docs/main_design.md` - design study: 5 architectures, weighted comparison, recommendation, risks
- `docs/buildability.md` - build review; option D chosen (see CLAUDE.md)
- `kinematics_D.py` + `docs/kinematics_D_report.md` - exact link-mechanism check of D (run with `--plots` for `docs/img/D_*.png`)
- `iteration_log.md` - design changes from the D build onward (CLAUDE.md rule 4); earlier history in `CHANGELOG.md`
- `parts/` - empty; one script per part later. `out/` - generated files (git-ignored)

## What kinematics.py prints
1. Bell-crank geometry for ΔS = 60 / 70 / 80 mm: L_LINE, P, real swept envelope (width, height), pressure angle
2. Layout trade-off: N_REEVE x Y0 -> width, height, pressure angle, R0, R_load, slot-wall load
3. Angle, angular rate, line angle, carriage position and grip separation vs travel
4. Carriage force balance (pin, spring, friction, cable tension down/up, slot-wall load, pressure
   angle) for the ΔS x F_hand grid, with the first slack point if any
5. Slot centre line for the right arm (arm frame, origin at the pivot), easing and dwell marked
6. Checks: hand spacing, cable never slack, R0 and R_load side by side, housing width, pressure angle, angle range
7. Reference: architecture A ratios and stage counts

## The math (also in code comments)
- Each handle rotates 30 deg over 300 mm (linear + easing), then dwells to 400 mm.
- k = 300 / 0.5236 rad = 573 mm of cable per rad of handle; carriage: dy/dβ = k / N_REEVE = 72 mm/rad at n = 8.
- Hand torque T = F · L_LINE · sin θ (θ > 0 closing). Cable tension = (spring + pin ± friction) / N_REEVE.
- Profile: linear with a quadratic easing over the last EASE_FRACTION of the map stroke (zero
  angular speed entering the dwell). Pin force = 2T · N_REEVE · dβ/ds.
- Slot point = Rot(−β) · (pin − pivot), pin = (0, Y0 + s/N_REEVE), pivot = (P/2, 0).
- Pressure angle: tan α = (dy/dβ + P/2) / y_pin. Slot-wall force = T / |r_pin × n̂| (drive, easing, dwell).
- Envelope: slot centre line thickened by slot/2 + rim, rotated by β(s) for all s, plus pivot
  boss, carriage and walls -> housing width and height.
- Geometry: L_LINE = ΔS / (2(sin θ_open − sin θ_closed)), P = S_closed − 2 L_LINE sin θ_closed.

## Current status of the checks (python3 kinematics.py, 2026-11-10, rev 3 baseline n = 8, Y0 = 80)
Hand spacing PASS (106.0), cable never slack PASS, **R0 = 5.6 N PASS / R_load = 39.9 N PASS (goal TODO)**,
housing width (swept envelope) 172 mm vs 170 **FAIL by 2 mm**, height 213 mm above pivots (no target),
pressure angle 54.8 deg PASS (limit 55, assumption), angle range PASS.
The rev-2 width "proxy" (139 mm) was wrong: the real swept envelope of the rev-2 layout was 292 mm.

## Assumptions I made (please confirm or correct)
1. F_HAND = 200 N per hand, vertical, equal both hands (Q5 unknown).
2. Friction modelled as (1 − η)(spring + |pin|) opposing carriage motion (η from item 10).
3. Reference cable hangs parallel to the main cable from an anchor above (1:1 payout).
4. Pulley 30 mm for 1.5 mm cable (rule-of-thumb, not a standard).
5. Pressure-angle limit 55 deg for a rolling follower (test on prototype).
6. Housing width = computed swept envelope of the slotted arms (±5 mm at best), not a proxy.
7. Angle-vs-travel profile linear_eased, EASE_FRACTION = 0.2 (quadratic blend into the dwell).
8. THETA_CLOSED_DEG = +3 deg bias so the hand torque is always closing (docs Section 6).
9. N_REEVE = 8, Y0_PIN_MM = 80 mm: the width / pressure-angle compromise (docs 5.1). 7 pulleys.
10. Efficiency 0.97 per pulley and 0.90 for carriage + roller -> 0.73 overall at n = 8.
11. Envelope model: arm rim 4 mm outside the slot, 2 mm clearance, 3 mm housing wall, pivot boss R20.
12. Spring preload 30 N (20 N left the no-load up stroke at 1.8 N cable tension with 8 parts).

## Open questions
Answered in rev 2: Q1-Q4, Q7-Q10 (see docs Section 1.4). Still open:
- Q5 stack loads (TODO), Q6 machine details (TODO)
- Q11 R_load goal: 39.9 N at ΔS = 70 vs the 10 %-of-stack goal of 40 N. ΔS = 60 gives ~36 N.
- Q12 housing height: 213 mm above the pivots at n = 8, Y0 = 80.
- Q13 is 172 x 213 mm at a 55 deg pressure angle acceptable for E? If <= 170 mm is hard at a
  conventional pressure angle, architecture C (~149 mm) is the answer. The first printed slotted
  arm is the decision test.
- Q14 are 7 pulleys (n = 8) acceptable, or n = 6 (5 pulleys, 188 mm, 59 deg)?
- Pivot bolts: M6/M8 allowed (answered).
