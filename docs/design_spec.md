# autoRAD bench rig — final design spec (hand off to CAD and manufacturing)

Date 2026-11-10. This is the frozen proof-of-concept of the **Main concept**: a frame-referenced
secondary cable, paid out as the attachment travels down, opens two handles symmetrically from
50° to 110° over 300 mm of travel, and a return band closes them again. Built as a low-load
**bench rig on one board** (your slide 12), not as a machine-mountable attachment.

Every number comes from `bench_check.py` (run output in `out/bench_check_run.txt`). All 9 checks
pass. Nothing is validated by test: the first print will show what needs changing.

Frames: **x outward (right handle at +x), y up, pivot line y = 0, centre line x = 0.** Board front
face (handles side) is z = 0; the board occupies z −12 … 0; the cord system is behind it (z < −12).

---

## 1. Master parameter table (enter these as Fusion User Parameters / Inventor Parameters)

| Name | Value | Note |
|---|---|---|
| S_CLOSED | 106 mm | grip-centre spacing at closed |
| DELTA_S | 70 mm | spacing change closed → open |
| THETA_C | 3° | pivot→grip line from vertical at closed |
| BETA_MAX | 30° | handle rotation |
| N_REEVE | 4 | cord parts |
| STROKE | 300 mm | attachment travel for full opening |
| ARM_A | 125.9 mm | drive arm, pivot → joint |
| LINK_C | 120.0 mm | link, joint → carriage pin |
| PSI0 | 42° | drive arm elevation above the inward horizontal at closed |
| HUB_R | 12 mm | pivot hub radius |
| PIVOT_D | 8 mm | M8 bolt |
| PIN_D | 5 mm | M5: joint pins, carriage pin, guide bolt, bearing axles, standoffs |
| PLATE_T | 8 mm | handles and carriage |
| LINK_T / LINK_W | 6 / 12 mm | |
| ARM_W | 16 mm | |
| BOSS_R | 8 mm | joint bosses |
| BRG | 625ZZ: OD 16, bore 5, width 5 | used directly as pulleys |
| FLANGE_D | 20 mm | printed flange washers, 1 mm |
| BOARD | 200 × 290 × 12 mm | bottom edge at y = −20 |
| CLEAR_PIN | 0.3 mm | all pin holes = pin + 0.3 |
| GAP | 0.3 mm | between stacked plates |

Derived (write as equations):

| Name | Equation | Value |
|---|---|---|
| L_LINE | DELTA_S / (2 (sin(THETA_C + BETA_MAX) − sin THETA_C)) | 71.09 mm |
| P | S_CLOSED − 2 L_LINE sin THETA_C | 98.56 mm |
| PIVOT_X | P / 2 | 49.28 mm |
| JOINT (closed, right) | (PIVOT_X − ARM_A cos PSI0, ARM_A sin PSI0) | (−44.3, 84.2) |
| PIN_Y (closed) | JOINT_Y + √(LINK_C² − JOINT_X²) | 195.8 mm |
| CAR_TRAVEL | PIN_Y(0°) − PIN_Y(30°) | 75.0 mm |

## 2. Printed parts (5 models, 16 pieces, PETG or PLA, ~9 h total at 0.28 mm layers)

### P1 Handle (×2: print one, mirror it for the left) — ~2.7 h each
One flat plate, 8 mm thick, printed lying flat. Origin = pivot bore. Right handle shown; left = mirror (x → −x).

| Feature | Dimensions |
|---|---|
| Hub | circle R12, bore Ø8.3 |
| Drive arm | bar 16 wide from the hub to the joint boss; joint at **125.9 mm, 138° from +x** (i.e. up-and-inward); joint boss R8, bore Ø5.3 |
| Long arm | bar 16 wide from the hub to the grip centre at **71.09 mm, 273° (−87°)** from +x (3° outboard of straight down) |
| Grip pad | 24 × 60 rectangle centred on the grip centre, long axis **25° from vertical**, top end toward the centre line; two Ø4.3 holes at ±20 mm along the axis (wood screws into the dowel) |
| Angle between the two arms at the hub | 225° (drive arm at 138°, long arm at 273°) |

Fillet the inside corners R5. The plate spans about 118 × 176 mm — fits a 200 × 200 bed on the diagonal.

### P2 Link (×2, identical) — ~0.6 h each
Bar 12 wide × 6 thick, **120.0 mm between hole centres**, both holes Ø5.3, end bosses R8.

### P3 Carriage (×1) — ~1.3 h
8 mm plate, printed lying on its back face, with one post standing up (the post becomes the
axle holder). Carriage frame: origin = carriage pin hole, x outward, y up. The plate sits
**behind the board** (z −20 … −12); the post extends further back to z −52.

| Feature | x | y | note |
|---|---|---|---|
| Vertical bar | −4 … +8 | −10 … 65 | |
| Horizontal bar | −32 … +8 | 55 … 65 | |
| Post | −19 … −9 | 35 … 55 | extruded 32 mm from the plate's back face (to z −52) |
| Carriage pin hole | 0 | 0 | Ø5.3 through the plate |
| Guide bolt hole | 0 | 60 | Ø5.3 through the plate |
| Axle hole | through the post, along x | 45 | Ø5.3, centred 24 mm behind the plate's back face (z −44) |
| Band hook | top edge of the horizontal bar at x = +8 | 65 | a notch or a hole Ø4 |

### P4 Tube / standoff (×6: one model, length is the parameter)
Cylinder OD 24 (handle spacer) or OD 10 (standoffs), bore Ø5.3 (Ø8.3 for the handle spacer).

| Use | Bore | OD | Length |
|---|---|---|---|
| Left-handle spacer (board face → handle L) | 8.3 | 24 | **20.9** |
| Carriage-pin spacer (board face → link R) | 5.3 | 10 | **8.3** |
| F1 standoff (board back → bearing) | 5.3 | 10 | **37.5** |
| F0 standoff | 5.3 | 10 | **21.5** |
| Dead-end standoff | 5.3 | 10 | **23.0** |
| Spare F0/F1 standoff | | | one of each |

### P5 Flange washer (×6, plus spares)
Disc Ø20 × 1 mm, bore Ø5.3. Two per bearing (M1, M2, F1), one each side.

## 3. Board (plywood 12 mm or acrylic 10 mm), 200 × 290

Use the 1:1 template `docs/img/board_template_1to1.pdf` (A4, print at 100 %, verify the 100 mm
scale bar). Coordinates in the board frame (bottom-left corner at (−100, −20)):

| Feature | Position | Size |
|---|---|---|
| Pivot R / L | (+49.28, 0) / (−49.28, 0) | Ø8.3 through |
| Slot 1 (carriage pin) | x = 0, y 117.6 … 198.9 | 5.3 wide |
| Slot 2 (guide bolt) | x = 0, y 177.6 … 258.9 | 5.3 wide |
| F1 bolt | (−14, 141.7) | Ø5.3 |
| F0 bolt | (−30, 141.7) | Ø5.3 |
| Dead-end bolt | (−6, 95) | Ø5.3 |
| Band screw | (+20, 262) | wood screw, back face |
| Anchor-cord notch | top edge at x = −38 | 3 mm deep |

Bolt heads on the **front** face must be flat (countersunk or button head, ≤ 3.5 mm proud): the
right link passes over the F0/F1 heads with 4.8 mm to spare.

## 4. Stack and assembly order (z from the board's front face)

| z range | part | on |
|---|---|---|
| 0 … 8 | handle R | directly on the board, M8 pivot bolt at (+49.28, 0) |
| 8.3 … 14.3 | link R | on handle R's joint pin (M5), washer between |
| 14.6 … 20.6 | link L | on handle L's joint pin, washer between |
| 20.9 … 28.9 | handle L | on the 20.9 mm spacer tube, M8 pivot bolt at (−49.28, 0) |
| 0 … 8.3 | carriage-pin spacer tube | on the carriage pin, between the board and link R |
| −20 … −12 | carriage plate | behind the board, pin bolt through slot 1, guide bolt through slot 2 |
| −36 / −52 | cord lane planes (near / far) | moving bearings on the carriage axle at x −22 and −6; F0 at (−30, 141.7) z −36; F1 at (−14, 141.7) z −52; dead end at (−6, 95) z −36 |

Grips: Ø20 dowel × 130, screwed flat onto each grip pad (the two grips end up 21 mm apart in z — accepted for the bench).

**Carriage pin bolt (M5 × 50):** head → carriage plate → slot 1 → spacer 8.3 → link R → washer → link L → nut.
**Guide bolt (M5 × 30):** head → carriage plate → slot 2 → washer → nut (loose enough to slide).
**Axle (M5 × 35):** nut → flange → M1 → flange → washer → post → washer → flange → M2 → flange → nut.
**F1 (M5 × 70):** flat head on the front → board → standoff 37.5 → flange → bearing → flange → nut. **F0 (M5 × 50):** same with standoff 21.5. **Dead end (M5 × 45):** board → standoff 23 → nut; tie the cord above the nut.

## 5. Cord routing (braided cord ~1.5 mm, ~1.5 m)

Tie at the dead end (−6, 95, z −36) → up to M2 (x −6) on the near lane (z −36) → over M2 → down
the far lane (−6, z −52) → under F1 (x −14, z −52) → up the far lane (−22, z −52) → over M1
(x −22) → down the near lane (−22, z −36) → under F0 (x −30, z −36) → up the anchor lane
(x −38, z −36) → over the notch in the top edge → to the fixed anchor.

Every part is vertical, each pulls the carriage down; 4 parts → the carriage moves 1/4 of the
cord payout. Return: rubber band / bungee from the carriage hook (back face, y ≈ 260 at closed)
to the band screw at (+20, 262). Pick a band that just holds the carriage up at closed; the model
needs only 2–5 N in the cord.

## 6. Bought parts — see `docs/buy_list.md`

## 7. Reference positions (VERIFIED, `bench_check.py`) — check your CAD against these

| carriage travel | cord payout (= attachment travel) | β | included angle | carriage pin y | joint R (x, y) |
|---|---|---|---|---|---|
| 0 | 0 | 0.00° | 50.0° | 195.8 | (−44.3, 84.2) |
| 15 | 60 | 6.45° | 62.9° | 180.8 | (−53.2, 73.2) |
| 30 | 120 | 12.49° | 75.0° | 165.8 | (−60.3, 62.0) |
| 45 | 180 | 18.32° | 86.6° | 150.8 | (−66.0, 50.6) |
| 60 | 240 | 24.10° | 98.2° | 135.8 | (−70.5, 38.7) |
| 75 | 300 | 29.99° | 110.0° | 120.8 | (−73.9, 26.2) |

Mapping is within +1.0° of linear (1.7 % of range). Transmission angle ≥ 64°. Minimum 3D
clearance in the front stack 3.6 mm (0.3 mm plate gaps by design); right arm to left pivot bolt
8.5 mm; carriage post to F1 flange 4 mm at the stop; moving-to-fixed bearing gap 8 mm.

## 8. Motion study in Fusion / Inventor

Joints: pivot R and L (revolute, z axis at (±49.28, 0), 0 … 30°); joint R and L (revolute at the
joint bosses); carriage pin (revolute, both links to the carriage); carriage ↔ board (slider along y,
0 … −75 mm). Drive the slider; the handles must follow with no other constraints. If the loop
doesn't close, a length is wrong.

## 8a. STL export (2026-11-10) — two differences from §2, and open points

Print files: `exports/stl/` (print orientation), `exports/step/` (design frame), list in `parts_list.md`.

1. **P1 handle:** the joint hole has a 90° countersink Ø11 on the back face. Handle R lies flat on the
   board, so the joint pin cannot have a head or nut behind it. Use **M5 × 25 countersunk** screws for the
   two joint pins, head flush in the handle, nut on the link side. The left handle is the same part turned over.
2. **P3 carriage:** the band hook is two Ø4 holes at (4, 54.5) and (4, −5.5) in the carriage frame. Use the
   lower one: from the top of the carriage the band would be ~12 mm long at closed and ~77 mm at the stop.

Open, not changed (assembly check, `out/bench_assembly_run.txt`): right arm to the left-handle spacer
0.51 mm at full open; flange washers and the post overlap by 0.5 mm per side (leave out the two washers
beside the post); F1 / F0 bearings sit 1 mm behind the lane plane (standoffs 37.5 / 21.5 do not allow for
the 1 mm flange); wall beside the carriage pin hole 1.35 mm; handle bed footprint 167 × 167 mm.

## 9. What is deliberately simplified (say so in the report)

No housing, no guards, not machine-mountable, no spool and no gears (replaced by a 4-part reeved
cord: lower friction than a 19–48:1 printed gear train), grips Ø20 not Ø32, grips offset 21 mm in z,
handles' hand clearance not addressed (no hands in the bench test), overtravel beyond 300 mm not
protected (keep the pull ≤ 300 mm), bearing pulleys Ø16 with a 1.5 mm cord. Loads at 200 N/hand
(link 68 N, carriage 108 N, cord 43 N) are model values only; the bench runs at ≤ 2 kg per grip.
