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
