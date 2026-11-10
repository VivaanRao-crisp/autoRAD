# Changelog

| date | change | reason |
|------|--------|--------|
| 2026-11-10 | Added docs/main_design.md (architecture study, 5 concepts, recommendation E) | Step 2 of the project brief: design study before CAD |
| 2026-11-10 | docs/main_design.md rev 2: B total 225 -> 235; Finding 2 wording (stack is on the main cable) | review corrections (Part A) |
| 2026-11-10 | docs: reviewer answers Q1-Q10 folded in; new Q11 (R_load margin), Q12 (housing height) | review Part B |
| 2026-11-10 | docs Section 6: carriage force balance; symmetric +-15 deg geometry goes slack at s = 0-100 mm under load; fix (b) offset geometry theta +3 -> +33 deg chosen | review Part C: hand torque changed sign in rev 1 |
| 2026-11-10 | params.py: replaced apex-pivot geometry with bell-crank (S_CLOSED, DELTA_S, THETA_CLOSED, L_LINE, P), added N_REEVE, carriage, spring, F_HAND, ETA_MECH, slot/roller, goals; spool/gear values kept as reference | architecture E selected |
| 2026-11-10 | params.py: Y0_PIN_MM 120 -> 150 | pressure angle at closed was 58 deg, over the 55 deg assumption |
| 2026-11-10 | kinematics.py: slot_profile_mm, pressure_angle_deg, carriage force balance, R0/R_load, new PASS/FAIL checks; gear planning cut to a reference table | review Part D |
| 2026-11-10 | README.md: run instructions, assumptions, check status, open questions updated | keep docs in step with code |
| 2026-11-10 | docs rev 3: width proxy replaced by swept envelope; rev-2 layout confirmed at 292 mm wide | review item 1 (proxy ignored the slotted arms) |
| 2026-11-10 | kinematics.py: arm_envelope_mm, housing_width_mm, housing_height_above_pivots_mm, layout scan (set_layout), slot_wall_force_n, eased profile, pin force from dbeta/ds; R0 and R_load printed together | review items 1, 2, 4, 5 |
| 2026-11-10 | params.py: N_REEVE 4 -> 8, Y0_PIN_MM 150 -> 80, PROFILE linear_eased + EASE_FRACTION 0.2, ETA_PULLEY/ETA_SLIDE replace ETA_MECH, envelope params (ARM_RIM, HOUSING_WALL, ARM_CLEARANCE, CARRIAGE_PULLEYS_PER_ROW), SPRING_PRELOAD 20 -> 30 N | width/pressure-angle trade-off (docs 5.1); 8 parts divided the spring preload to 1.8 N on the up stroke |
| 2026-11-10 | docs: compactness re-scored (E 3->2, C 4->5, D 3->4), totals E 375 / D 365 / C 360; E kept as primary conditionally, first slotted-arm print is the E-vs-C decision test; M6/M8 pivots allowed | review items 3, 6 |
| 2026-11-10 | docs/buildability.md: build review of E, C, D+anchor spring; recommends D on a shared E/D platform as the first prototype | review request: most likely to work in 3 weeks, not best on paper |
| 2026-11-10 | kinematics_D.py + docs/kinematics_D_report.md + docs/img/D_*.png; D geometry corrected in params.py (arm 70.3, link 70, 25 deg); iteration_log.md started per CLAUDE.md | verify D before CAD (CLAUDE.md rule 5) |
| 2026-11-10 | check3d_D.py + report addendum (3D stack, grips vs housing, reeving lanes); params.py header/legacy/bearing/stack/lane values | review of kinematics_D_report.md items 1-3 |
| 2026-11-10 | docs/design_freeze.md (NOT FROZEN), CLAUDE.md and params.py updated; check3d_D.py: lanes with F0, force directions, y-layout, width budget | freeze run | 
| 2026-11-10 | docs/cad_build_guide.md; params.py D_ARM 72.05, psi0 38, PULLEY_WIDTH 7; CLAUDE.md CAD route | team builds CAD in GUI tool |
