# autoRAD - parametric CAD-by-script (Step 1: parameters + kinematics)

All values are PRELIMINARY. Nothing here is validated; only prototype tests can do that.

## Run
```
python3 kinematics.py
```
Needs only the Python standard library for now. CadQuery and cq_gears are needed from Step 2 (parts).

## Files
- `params.py` - every dimension and design variable (units in comments, TODOs marked)
- `kinematics.py` - handle angle, spool rotation, ratio, gear-stage suggestion, PASS/FAIL checks
- `parts/` - empty; one script per part later. `out/` - generated STEP/STL (git-ignored)

## The math (also in code comments)
- Each handle rotates (110 - 50) / 2 = 30 deg. Mapping is linear over 300 mm, clamped at 110 deg after.
- Spool rotation = payout / r. Ratio = spool rotation / handle rotation, e.g. r = 12: (300/12) / 0.5236 rad = 47.7.
- Stages: smallest n with (max stage ratio)^n >= ratio. Tooth counts: 12-tooth pinions, last stage absorbs rounding.
- Grip separation = pivot_spacing + 2 * 127 * sin(25 deg) = 107.3 mm (>= 106 needed, only 1.3 mm margin).

## Assumptions I made (please confirm or correct)
1. Cable payout equals attachment travel (1:1, no pulley multiplier).
2. Spool radius is the effective radius at the cable (no cable thickness/layering).
3. Both handles pivot about one common point (pivot spacing = 0). This makes the hand-spacing check optimistic or exact depending on the real layout.
4. Final stage drives two meshing sector gears, 1:1 between handles, so the spool->one-handle ratio is the whole ratio.
5. Spool candidates step by 3 mm (12, 15, ... 30).
6. Gear planning placeholders: module 1.0 mm, min 12 teeth, max 6:1 per stage, equal pinions per stage.
7. Housing-width check is a PROXY (largest gear or spool diameter + 2 walls + 2 clearances). It ignores handles, bearings, mount and spring, so a PASS means little yet.
8. "Handle angle range" check only tests the formula endpoints and hard-stop clamp, not any mechanism.

## Open questions
1. Is pivot spacing really 0, or do the handles sit on separate axes? What value?
2. Does the cable route 1:1 or via a pulley (changes payout per mm)?
3. Does the 300 mm map stroke stay fixed, or should the ratio be re-chosen per user within 250-400 mm (see table 3)?
4. Definition of "actuation resistance <= 10 N" (force at the handle pull, at the cable, return-spring force, friction?).
5. Gear module, pressure angle, face width, and printed material for FDM gears.
6. Housing orientation: which dimension is "width" (along the handle spread?) so the width check can be real.
7. Is the 1.3 mm hand-spacing margin acceptable given grip diameter 32 mm and the angle tolerance of a printed hard stop?
8. Is "grip centre ~127 mm" measured along the handle axis or straight-line from the pivot?
9. Should the final-stage tooth count be allowed to have a non-integer ratio error (currently within ~1%)?

## Observed in the first run
Candidate radii 12-15 mm need 3 stages; 18-30 mm need 2 stages (at the 6:1 per-stage placeholder).
