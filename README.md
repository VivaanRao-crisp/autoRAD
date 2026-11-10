# autoRAD - parametric CAD-by-script (architecture E: reeved cable + carriage + slotted bell-crank handles)

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
- `parts/` - empty; one script per part later. `out/` - generated files (git-ignored)

## What kinematics.py prints
1. Bell-crank geometry for ΔS = 60 / 70 / 80 mm: L_LINE, pivot spacing P, width proxy, hand torques
2. Angle, line angle, carriage position and grip separation vs travel
3. Carriage force balance (pin, spring, friction, cable tension on down/up stroke) for the
   ΔS × F_hand grid, with the first slack point if any
4. Slot centre line for the right arm (arm frame, origin at the pivot) and pressure angle
5. Checks: hand spacing, cable never slack, R0, R_load, housing width, pressure angle, angle range
6. Reference: architecture A ratios and stage counts

## The math (also in code comments)
- Each handle rotates 30 deg over 300 mm (linear), then dwells to 400 mm.
- k = 300 / 0.5236 rad = 573 mm of cable per rad of handle; carriage: dy/dβ = k / N_REEVE = 143 mm/rad.
- Hand torque T = F · L_LINE · sin θ (θ > 0 closing). Pin force = 2T / (dy/dβ). Cable tension =
  (spring + pin ± friction) / N_REEVE.
- Slot point = Rot(−β) · (pin − pivot), pin = (0, Y0 + s/N_REEVE), pivot = (P/2, 0).
- Geometry: L_LINE = ΔS / (2(sin θ_open − sin θ_closed)), P = S_closed − 2 L_LINE sin θ_closed.

## Current status of the checks (python3 kinematics.py, 2026-11-10)
Hand spacing PASS (106.0), cable never slack PASS, R0 = 9.0 N PASS, R_load = 40.7 N vs 40 N
**FAIL (marginal, goal TODO)**, housing width 139 mm PASS, pressure angle 52 deg PASS, angle range PASS.

## Assumptions I made (please confirm or correct)
1. F_HAND = 200 N per hand, vertical, equal both hands (Q5 unknown).
2. Efficiency ETA_MECH = 0.8; friction modelled as (1 − η)(spring + |pin|) opposing carriage motion.
3. Reference cable hangs parallel to the main cable from an anchor above (1:1 payout).
4. Pulley 30 mm for 1.5 mm cable (rule-of-thumb, not a standard).
5. Pressure-angle limit 55 deg for a rolling follower (test on prototype).
6. Housing width proxy = P + 2 × 20 mm pivot boss.
7. Linear angle-vs-travel profile (PROFILE = "linear").
8. THETA_CLOSED_DEG = +3 deg bias so the hand torque is always closing (docs Section 6).
9. Y0_PIN_MM = 150 mm (pin height at closed) to keep the pressure angle under 55 deg.

## Open questions
Answered in rev 2: Q1-Q4, Q7-Q10 (see docs Section 1.4). Still open:
- Q5 stack loads (TODO), Q6 machine details (TODO)
- Q11 R_load goal: 40.7 N at ΔS = 70 vs the 10 %-of-stack goal of 40 N. ΔS = 60 gives 36 N.
- Q12 housing height: carriage pin reaches 250 mm above the pivots at 400 mm travel (N_REEVE = 4).
- Pivot bolts: M6/M8 wanted for the two handle pivots (brief lists M3/M4).
