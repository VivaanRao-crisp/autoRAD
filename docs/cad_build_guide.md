# autoRAD — CAD build guide for the team (build in your GUI CAD, fully parametric)

Written for: six students building the parametric model in the institute CAD tool. The numbers
come from `kinematics_D.py` and `check3d_D.py` (run on 2026-11-10 with ψ0 = 38°, pulley width
7 mm). **Preliminary and unvalidated.** Tags: VERIFIED = computed by those scripts;
ASSUMPTION = a choice to confirm or change; TODO = not decided.

## 0. Rules for everyone

1. **One master parameter table (section 1).** In your CAD tool create these as global
   parameters / user parameters / equations (Fusion "User Parameters", SolidWorks "Global
   Variables" or "Equations", Creo "Parameters", FreeCAD spreadsheet). Every sketch dimension
   refers to a name, never a typed number. This is what makes the model parametric.
2. **Axes:** X = outward (right handle at +X), Y = up, Z = stacking direction along the pivot axes.
   Pivot line Y = 0, centre line X = 0. Rotation that **opens the right handle is counter-clockwise
   as seen with X right, Y up**. Right handle at +X, left is the mirror (X → −X).
3. **Fits (FDM, PETG):** pin hole = pin + 0.3 mm (`CLEAR_PIN`); bearing seat = bearing OD + 0.1 mm
   (`CLEAR_BRG`); plate-to-plate gap 0.3 mm; minimum wall 1.6 mm (use 2 mm); no overhang > 45°.
4. **Naming:** `autoRAD_<part>_<R|L>`. Name the origin of every part at its main hole centre
   (pivot bore for arms, pin bore for links and carriage) so mates snap on.
5. **Check every result in section 5 against your model.** If yours differs by more than
   0.2 mm, find out why before printing.

## 1. Master parameter table

Independent parameters (type these in):

| Name | Value | Unit | Tag / source |
|---|---|---|---|
| `S_CLOSED` | 106 | mm | grip-centre spacing at closed (brief: hand 96 + 10) |
| `DELTA_S` | 70 | mm | change in grip spacing, closed → open |
| `THETA_C` | 3 | deg | pivot→grip line angle from vertical at closed |
| `BETA_MAX` | 30 | deg | handle rotation closed → open |
| `N_REEVE` | 8 | – | cable parts |
| `STROKE_MAP` | 300 | mm | attachment travel for full opening |
| `ARM_A` | 72.05 | mm | drive arm, pivot → joint. VERIFIED (re-solved for 37.5 mm travel) |
| `LINK_C` | 70.0 | mm | link, joint → carriage pin. VERIFIED |
| `PSI0` | 38 | deg | drive arm elevation above the inward horizontal at closed. VERIFIED |
| `HUB_R` | 10 | mm | pivot hub radius. ASSUMPTION |
| `PIVOT_D` | 8 | mm | pivot bolt (M8). see issue I1 |
| `JOINT_D` | 5 | mm | arm/link joint pin (M5). ASSUMPTION |
| `CAR_PIN_D` | 8 | mm | carriage pin (M8). ASSUMPTION |
| `ARM_W` | 16 | mm | drive-arm plate width. ASSUMPTION |
| `ARM_T` | 8 | mm | drive-arm / long-arm plate thickness. ASSUMPTION |
| `LINK_W` | 12 | mm | link width. ASSUMPTION |
| `LINK_T` | 6 | mm | link thickness. ASSUMPTION |
| `JOINT_BOSS_R` | 8 | mm | boss radius at joint pins. ASSUMPTION |
| `PLATE_GAP` | 0.3 | mm | gap between stacked moving plates |
| `PUL_D` | 30 | mm | pulley diameter (moving and fixed). ASSUMPTION |
| `PUL_W` | 7 | mm | pulley width (625 bearing 5 + 2 × 1 mm flange). ASSUMPTION |
| `BRG_OD / ID / W` | 16 / 5 / 5 | mm | 625ZZ default, **confirm before printing** |
| `FAIR_PUL_D` | 20 | mm | fairlead pulley F0. bearing TODO |
| `ROD_D` | 5 | mm | guide rods |
| `ROD_BUSH_OD` | 12 | mm | printed bush around each rod. ASSUMPTION |
| `GRIP_D` | 32 | mm | bought grip tube |
| `GRIP_LEN` | 130 | mm | bought grip tube |
| `CABLE_D` | 1.5 | mm | steel cable |
| `WALL` | 3 | mm | housing plate thickness for the side plates (top/bottom/front/back 10 mm) |
| `CABLE_Z` | 43 | mm | z of the moving-pulley axle. ASSUMPTION |
| `FIXED_ROW_Y` | 81.5 | mm | y of fixed pulley axles. VERIFIED (5 mm gap at the stop) |

Derived (write them as equations, so the sheet stays parametric):

| Name | Equation | Value |
|---|---|---|
| `L_LINE` | `DELTA_S / (2*(sin(THETA_C+BETA_MAX) - sin(THETA_C)))` | 71.09 mm |
| `P` | `S_CLOSED - 2*L_LINE*sin(THETA_C)` | 98.56 mm |
| `PIVOT_X` | `P/2` | 49.28 mm |
| `JOINT_R_CLOSED` | `(PIVOT_X - ARM_A*cos(PSI0), ARM_A*sin(PSI0))` | (−7.50, 44.36) |
| `PIN_Y_CLOSED` | `JOINT_Y + sqrt(LINK_C^2 - JOINT_X^2)` | 113.96 mm |
| `CAR_TRAVEL` | `PIN_Y_CLOSED - PIN_Y_OPEN` | 37.5 mm (= 300 / 8) |
| `BELL_ANGLE` | angle between drive arm and pivot→grip line | 131° |
| `TUBE_OFFSET` | `25° - THETA_C`: tube axis vs pivot→grip line | 22° |

The kinematic link between the two moving parts is the loop: **arm joint → link → carriage pin**.
If your assembly sketch is built from `ARM_A`, `LINK_C`, `PSI0` and `P`, the whole mechanism
re-solves when you change them. That is the parametric demonstration.

## 2. Per-part build sheets (8 essential parts for the prototype)

Print orientation rule: plates lie flat (layers in-plane), pins and bearings sit in holes.
Print-time figures are rough (15 cm³/h, ±50 %) and are not VERIFIED.

### Part 1 — Drive arm R (and L = mirror), ~1.5 h each
Plate, thickness `ARM_T`, width `ARM_W` along its axis. **Origin = pivot bore.**
- Pivot bore Ø `PIVOT_D + CLEAR_PIN` through the hub, hub radius `HUB_R`.
- Joint boss: radius `JOINT_BOSS_R`, bore Ø `JOINT_D + CLEAR_PIN`, centre at distance `ARM_A`
  from the pivot, direction at **142° (= 180° − PSI0)** measured CCW from +X at closed (right arm).
- A 2-hole bolt pattern on the hub face for the long handle arm (TODO: 2 × M4 at ±6 mm from the
  pivot on the long-arm axis).
- z: **arm R z 0–8**, **arm L z 20.9–28.9** (see the stack in section 4).
- Opens CCW (right). Stop: a tab for the 110° stop (TODO: location after the frame exists).

### Part 2 — Hub extension (×2), ~0.3 h
Cylinder radius `HUB_R`, bore `PIVOT_D + CLEAR_PIN`, length **10.45 mm**, attached to the face of
each drive arm hub so the long arm sits at the common grip plane z = 14.45. VERIFIED arithmetic
(common plane = centre of the 28.9 mm stack). Print as part of the arm if it fits, else separate.

### Part 3 — Long handle arm R / L (bell-crank leg + grip seat), ~1.5 h each
Plate `ARM_T` thick, **origin = pivot bore**, centred at z = 14.45 (z 10.45–18.45).
- Leg from the pivot to the grip centre: length `L_LINE` = 71.09 mm at **273° (−87°)** at closed,
  i.e. 3° outboard of straight down. The two legs of the bell-crank (this one and the drive arm)
  make the included angle `BELL_ANGLE` = 131° at the pivot.
- Grip seat: bought tube Ø`GRIP_D` × `GRIP_LEN`; tube axis is tilted `TUBE_OFFSET` = 22° from the
  pivot→grip line (25° from vertical at closed), upper end toward the centre line. Seat = a
  Ø`GRIP_D + CLEAR_PIN` socket or a bracket of your choice (TODO: tube fixing, e.g. a bolt through
  the tube).
- **Open issue I2:** hand clearance to the hub is −12.9 mm in the R35 assumption (see section 6).

### Part 4 — Link (×2: R and L are identical, flipped), ~0.4 h each
Plate `LINK_T` × `LINK_W`, **length `LINK_C` = 70.0 mm between hole centres**, both bores
Ø `JOINT_D + CLEAR_PIN` (arm end) and Ø `CAR_PIN_D + CLEAR_PIN` (carriage end), end boss radius
`JOINT_BOSS_R`. z: **link R 8.3–14.3**, **link L 14.6–20.6**.

### Part 5 — Carriage, ~4 h
One block holding: the moving pulley axle, the two rod bushes, the carriage pin, the spring hook.
- Moving pulley axle: along **X** at height `PIN_Y + 40` above the pin (ASSUMPTION), z = `CABLE_Z`,
  carries 4 pulleys at **x = −45, −15, +15, +45** (spacing 30 = `PUL_D`).
- Carriage pin: M8 along **Z** on the centre line, through both links (z 8.3–20.6), supported at
  the front (z 0–8) and recessed behind link L (1 mm). **I3: cantilever ~20 mm, confirm strength.**
- Rod bushes: 2 × Ø`ROD_BUSH_OD` at **x = ±56**, z = `CABLE_Z`, sliding on the Ø5 rods
  (bush bore Ø5 + `CLEAR_PIN` = 5.3 mm).
- Spring hook: on the centre line, **above** the pulley axle (the spring sits between M2 and M3
  at x = 0, z = 43, in the 23 mm gap).
- Block width 97 mm over the pulleys (3 × 30 + 7). Carriage travel **37.5 mm along −Y**
  (moves **down** as cable pays out).

### Part 6 — Moving pulley (×4), ~0.6 h each
Ø`PUL_D`, width `PUL_W`, bearing seat Ø`BRG_OD + CLEAR_BRG`, depth `BRG_W`, bearing in each; cable
groove for `CABLE_D` (groove width 2.2 mm, depth 1.5 mm — ASSUMPTION), flange 1 mm each side.

### Part 7 — Fixed pulley (×3) and fairlead pulley (×1)
Same pulley as Part 6 for F1 (x −30, z 58), F2 (x 0, z 28), F3 (x +30, z 58), axles along **Z**
at y = `FIXED_ROW_Y` = 81.5. Fairlead F0: Ø`FAIR_PUL_D` at (x −45, z 18), axle along **X**, y 81.5.
Dead end: a clamp at (x +45, z 28), y 81.5. **F0 bearing: TODO (OD ≤ 12).**

### Part 8 — Frame (front plate + back plate + standoffs), ~6–8 h
Proposal (ASSUMPTION, the only part I couldn't derive): two flat plates in the XY plane, **front
plate** z −10…0 and **back plate** z 63.5…73.5, joined by standoffs; plates cover y from −15 to
+190 and x ±74.5 (149 wide), bolted with M4 standoffs in the corners. The front plate carries the
two pivot bolts (M8 at x ±49.28, y 0), the two Ø5 rods at x ±56 (rods span y 40…150, **TODO length
~150**) and the closed/open carriage stops; the back plate carries the F1/F2/F3/F0 axles and the
dead-end clamp. Eye for the carabiner on the top edge. Anchor-cable fairlead hole at (x −45, z 8)
in the top plate. **This is the part to design with the most care.**

Minimum bought-parts list: 625ZZ × 8 (7 pulleys + F0, size TODO), M8 bolts × 3 (pivots + carriage
pin), M5 pins × 2 (joints), Ø5 rod × 2, extension spring (30 N preload, ~0.1 N/mm — **real value
TODO**), 1.5 mm cable ~3 m, ferrules, M3/M4 screws, grip tube × 2, carabiner, strap.

## 3. Mate sheet for the assembly and motion study

| # | Joint | Parts | Axis | Type | Limit |
|---|---|---|---|---|---|
| 1 | Pivot R | frame ↔ drive arm R (+ hub ext + long arm R fixed to it) | Z at (+49.28, 0) | revolute | 0…30° CCW |
| 2 | Pivot L | frame ↔ drive arm L (+ hub ext + long arm L) | Z at (−49.28, 0) | revolute | 0…30° CW |
| 3 | Joint R | drive arm R ↔ link R | Z at JOINT_R | revolute | free |
| 4 | Joint L | drive arm L ↔ link L | Z at JOINT_L | revolute | free |
| 5 | Carriage pin R/L | link R, link L ↔ carriage | Z on the centre line | revolute ×2 | free |
| 6 | Carriage slide | carriage ↔ rods/frame | Y | slider | 0…37.5 mm downward from closed |
| 7 | Pulleys | pulley ↔ axle | own axis | revolute | free (motion study may fix) |
| 8 | Handle sync | — | — | enforced by the carriage + two links | |

Drive the carriage (joint 6) from 0 to 37.5 mm; the handles must follow without further
constraints. **If the motion study stalls, the link or arm length is wrong** — the whole loop
should close with zero extra constraints.

Reference positions to compare with your CAD (all VERIFIED, `kinematics_D.py`):

| carriage travel | attachment travel s | β (each handle) | included angle | carriage pin y | joint R (x, y) | grip R centre (x, y) |
|---|---|---|---|---|---|---|
| 0 | 0 | 0.00° | 50.00° | 113.96 | (−7.50, 44.36) | (53.0, −71.0) |
| 10 | 80 | 8.68° | 67.37° | 103.96 | (−13.54, 35.28) | (63.7, −69.6) |
| 20 | 160 | 16.62° | 83.25° | 93.96 | (−17.81, 26.26) | (73.2, −67.0) |
| 30 | 240 | 24.28° | 98.56° | 83.96 | (−20.71, 17.09) | (81.9, −63.2) |
| 37.5 | 300 | 30.00° | 110.00° | 76.46 | (−22.07, 10.03) | (88.0, −59.6) |

The mapping is not exactly linear: **max deviation +1.44° (2.4 % of 60°) at s = 108 mm**, within
the 2.5 % limit. In a physical test the handles lead the straight line by up to about 1.4°
around s = 100 mm.

## 4. z-stack (every plate and pin)

| z [mm] | 0–8 | 8.3–14.3 | 14.6–20.6 | 20.9–28.9 | 10.45–18.45 |
|---|---|---|---|---|---|
| part | drive arm R | link R | link L | drive arm L | both long arms (common grip plane z = 14.45) |

Joint pins: R joint pin spans arm R + link R (z −3…17.3), L joint pin spans link L + arm L
(z 11.6…31.9). **Do not run any joint pin through the far arm.** Minimum real clearance in the
3D check: 3.6 mm; the 0.3 mm stacked gaps are by design.

## 5. Checks to do in your CAD (acceptance)

| Check | Expected |
|---|---|
| Hand spacing at closed (grip centre to grip centre) | 106.0 mm |
| Included angle at carriage travel 0 / 37.5 mm | 50° / 110° |
| Carriage travel for 30° | 37.5 mm |
| Link angle from vertical, closed / open | 6.1° / 18.4° |
| Minimum transmission angle arm–link (angle between the two parts at the joint) | 58.1° at closed (limit 30°) |
| Interference, whole mechanism at 5 carriage positions | none, except the intended pin fits and the 0.3 mm gaps |
| Mass estimate of the printed parts | TODO: from the CAD, needed for the report |
| Housing outer size | 149 wide × ~71.5 deep (with 3 mm plates) × ~190–200 tall; see I4 |

## 6. Known issues you must handle

- **I1 — Pivot bolt vs bearing.** The pivots are Ø8 bolts, but a 625ZZ has a 5 mm bore. My earlier
  "hub R10 = bearing 16/2 + 2" rationale was for the pulleys, not the pivots. For the pivots use
  a plain printed hub on the M8 bolt (wall around the bolt 5.8 mm at R10), **or** a 608 bearing
  (8 × 22 × 7), which needs hub R ≥ 13. **Decide.**
- **I2 — Hand clearance.** Grip tube vs hub 0.6 mm; the hand envelope (R35 assumption) overlaps
  the hub by 12.9 mm at ΔS = 70. Test with a mock-up hub and tube before the final print, or
  accept it for the bench prototype (no hands involved).
- **I3 — Carriage pin cantilever.** M8 pin supported from the front only. If the first test shows
  deflection, add a rear bracket with a cut-out for F2.
- **I4 — Depth 71.5 mm** (not 70): 7 mm pulleys, 3 mm plates, cable z 24.5–61.5 and the anchor
  lane. Plate thickness 10 mm for the front/back plates in section 2 adds to this: ~91 mm
  outside. Choose: 3 mm back/front plates or live with it. **Needs a decision from the team.**
- **I5 — F0 (fairlead) pulley bearing and the anchor-spring tube (v2) are TODO.**
- **I6 — Extension spring and 625ZZ not yet confirmed as available.**
- **I7 — Loads:** peak link force 119 N, carriage 227 N, at 200 N/hand (assumption). The first
  tests should use a few kg. R0 5.4 N, R_load 38.4 N (goal 40) are model values only.

## 7. Work split for six people

| Person | Parts / task | Output |
|---|---|---|
| 1 (lead) | Master parameter table, **assembly sketch loop** (arm → link → carriage), mate sheet, motion study | assembly + animation 50° → 110° |
| 2 | Part 1 drive arms, Part 2 hub extension | STEP + STL |
| 3 | Part 3 long arms + grip seats, Part 4 links, joint pins | STEP + STL |
| 4 | Part 5 carriage, Part 6 moving pulleys | STEP + STL |
| 5 | Part 7 fixed/fairlead pulleys, Part 8 frame plates + standoffs, stops | STEP + STL |
| 6 | Drawings, exploded + section views, mass, strength/weight calcs, report + slides, bench test sheet | drawings, report |

Order of printing: arms and links first (shortest, show the mapping), then the carriage and
pulleys, then the frame. Print each part as soon as its owner has checked it against section 5.

## 8. Bench test sheet (fill in; only measured values count)

Pull the cable by hand from a fixed anchor; hold the frame. At attachment travel 0, 80, 160, 240,
300 mm (= carriage 0, 10, 20, 30, 37.5): measure the angle between the handles with a protractor
(target 50, 67.4, 83.3, 98.6, 110°) and the cable tension with a spring balance (R0 model 4.8–5.4 N).
Then hang 2 kg and 5 kg per grip at closed and open: record the angle and whether the handles creep
or the cable slackens. Record every problem found, what you changed, and the v2 result.
