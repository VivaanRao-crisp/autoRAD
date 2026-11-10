# autoRAD bench rig — assembly, test and demo sheet

Only your measurements count. Fill in the tables; photograph every state and every problem.

## A. Assembly (about 1 h)
1. Drill the board from the 1:1 template (check the 100 mm bar first). Cut the two slots and the notch.
2. Front: handle R on the pivot R bolt (plain, no spacer). Handle L on its 20.9 mm spacer on pivot L. Tighten the nuts only until the handles swing freely.
3. Links: link R onto handle R's joint pin with a washer; link L onto handle L's joint pin. Bring both free ends to the centre line.
4. Carriage: from the back, push the carriage pin bolt through the carriage plate, slot 1, the 8.3 mm spacer, link R, a washer, link L; nut on the front. Guide bolt through slot 2, washer, nut (loose).
5. Axle: flange, bearing M1, flange, washer, post, washer, flange, bearing M2, flange, nuts.
6. Back standoffs: F1 (37.5) with its bearing and flanges at (−14, 141.7); F0 (21.5) at (−30, 141.7); dead end (23) at (−6, 95).
7. Cord: tie at the dead end; route exactly as design_spec §5; over the notch at x = −38 to the anchor.
8. Band from the carriage hook to the band screw. Pull the carriage up to the top of slot 1 (closed) — the band should just hold it.
9. Grips: screw the dowels to the grip pads.
10. By hand: push the carriage down 75 mm. Handles must open to 110° and the cord must stay taut. If anything binds, loosen that nut first.

## B. Setup for the test
Clamp the board vertically to a table edge, handles hanging free below the edge. Tie the anchor
end of the cord to a fixed point roughly in line with the notch (above the board, ≥ 400 mm away).
Mark the cord at the notch with tape at the closed position.

Two equivalent ways to run it: **(a)** keep the anchor fixed and slide the board down along the
table edge (what the real attachment does); **(b)** keep the board clamped and pull the cord's
anchor end away by hand. Use (b) for measurements, (a) for the demo video.

## C. Measurements (fill in)

**C1 Angle vs travel** (protractor between the two dowels; travel = cord pulled past the tape mark)

| cord payout [mm] | model included angle | measured angle | left handle from vertical | right handle from vertical |
|---|---|---|---|---|
| 0 | 50.0° | | | |
| 60 | 62.9° | | | |
| 120 | 75.0° | | | |
| 180 | 86.6° | | | |
| 240 | 98.2° | | | |
| 300 | 110.0° | | | |

Symmetry = difference between the left and right columns (model: 0; accept ≤ 3°).

**C2 Return**: release the cord slowly from 300 mm. Does the carriage return fully to the top of
slot 1? Does the cord stay taut on the way? Repeat 10 times; count failures.

**C3 Cord force** (spring balance or kitchen scale in the cord, no load on the grips): at 0, 150,
300 mm. Model: ~2–5 N (depends on the band you chose). Record the band used.

**C4 Load**: hang 1 kg then 2 kg on each grip (bag of sugar, water bottle). At closed and at
300 mm: do the handles hold their angle? Does the cord slacken? Does anything flex or creak?
Do NOT exceed 2 kg per grip on the first print.

**C5 Problems found** (this is the iteration section of your report): what broke, rubbed, bound,
or did not fit; what you changed (part, dimension, why); the v2 result.

## D. Demo script (2 minutes)
1. Show the board closed: handles at 50°, cord taut, band holding the carriage.
2. Slide the board down 300 mm against the fixed anchor (or pull the cord): handles open
   symmetrically to 110°. Point at the 4 cord parts and the carriage moving 1/4 as far.
3. Slide back up: the band rewinds, the handles close.
4. Show the angle-vs-travel table next to the plot `docs/img/D_angle_vs_travel.png`.
5. Say what is simplified: bench rig, no housing, low load, bearings as pulleys — and what the full
   attachment would add (design_spec §9).
