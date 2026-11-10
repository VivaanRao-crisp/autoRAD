# autoRAD — Main concept design study (travel-actuated opening V-bar)

Revision 3 (2026-11-10). Changes from rev 2: housing-width proxy replaced by a computed swept
envelope (the reviewer's ~290 mm figure is confirmed); layout trade-off table added; baseline
layout changed to N_REEVE = 8, Y0 = 80 mm; easing zone added before the dwell; slot-wall loads
added; R0 and R_load reported side by side; compactness re-scored with real envelopes; M6/M8
pivots allowed.

Status: **design study, nothing validated.** Every number is a first-pass hand calculation or a
`kinematics.py` output. TODO = unknown. ASSUMPTION = my choice; challenge it.

Notation: `s` = attachment travel [mm], `β` = handle rotation from closed [deg], `θ` = angle of
the pivot→grip-centre line from vertical [deg, outward +], `T` = torque on one handle [N·m],
`F` = vertical hand force per hand [N], `P` = pivot spacing [mm], `ΔS` = change in grip-centre
separation [mm], `n` = N_REEVE, `Y0` = pin height above the pivot line at closed [mm].

---

## 1. Problem restatement, assumptions, open questions

### 1.1 Problem in one paragraph
A removable attachment for a cable machine. It hangs from the existing carabiner. Two grip
handles form a V. As the user pushes the attachment down, the handles must open symmetrically
from 50° to 110° included angle over the first 300 mm of travel, then hold 110° (dwell) up to
400 mm. On the way up they must close again. The motion must be driven only by the attachment's
travel relative to the fixed frame, with no electronics and no manual adjustment during use.

### 1.2 Four findings that shape the design (read these first)

**Finding 1 — the hand load dominates, not the gear ratio.**
With both handles pivoting at the apex and the grip 127 mm from the pivot, a vertical push `F`
creates `T = F · L · sin β` = 10.7–20.8 N·m per handle at 200 N/hand. Over the mapping stroke the
cable pays out 300 mm while each handle turns 0.5236 rad:

    k = 300 / 0.5236 = 573 mm of cable per radian of handle rotation
    cable tension (ideal) = T_both / 0.573 m   → 37 N closed … 73 N open

No transmission fixes this; only the handle geometry does. **Resolved:** bell-crank handles with
the grip hanging almost under its pivot at the closed end (θ = +3° → +33°), worst torque
7.7 N·m per handle at ΔS = 70, always closing (Section 5.2).

**Finding 2 — "hold 110° beyond 300 mm" needs a dwell, not a hard stop.**
A blocked mechanism would make the reference cable take the user's extra push as a jolt (the
main cable still carries the stack). **Resolved:** dwell as a straight section of the slot cam,
now entered through an easing zone so there is no force step (Section 6.5).

**Finding 3 — the hand torque must never reverse sign.**
Rev 1's symmetric ±15° geometry made the load open the handles in the first half of the stroke
and the cable went slack (Section 6). **Resolved:** θ ≥ +3° throughout.

**Finding 4 (rev 3) — the slot cam makes architecture E wide.**
The slot is the pin path seen from the arm; it spans ~30° of arc at a radius of Y0 … Y0 + s/n.
At the closed position that arc sticks out sideways. With the rev-2 layout (n = 4, Y0 = 150) the
far end of the slot sits 141 mm from the centre line and the housing is **292 mm** wide — the
reviewer's estimate is confirmed. The pin being 49 mm inboard of the pivot also adds to the
pressure angle: `tan α = (dy/dβ + P/2) / y_pin`. Width and pressure angle trade against each
other at ~3.5° per 10 mm (Section 5.1). **Partly resolved:** n = 8 and Y0 = 80 give 172 mm ×
213 mm at a 55° pressure angle. E cannot reach 150–170 mm at a conventional (≤ 45°) pressure
angle; that needs ≥ 210 mm. Architecture C can reach ~150 mm (Section 3).

### 1.3 Assumptions (ASSUMPTION = mine; change in `params.py`)
| # | Assumption | Why it matters |
|---|---|---|
| A1 | Hand force per hand `F_HAND_N` = 200 N, vertical, equal on both hands. | Sets every internal load. TODO measure (Q5). |
| A2 | Reference cable anchored above, parallel to the main cable, payout = travel. | Offset error small (Section 7.6). |
| A3 | Efficiency: 0.97 per ball-bearing pulley, 0.90 for carriage guide + roller together → `ETA_MECH` = 0.90 × 0.97^(n−1) = 0.73 at n = 8. Friction modelled as (1 − η)(spring + \|pin\|) opposing carriage motion. | Rough. Test 9 measures the real R0. |
| A4 | 1.5 mm cable; pulley Ø30 (≈ 20× cable, guidance not standard). TODO. | Pulleys size the carriage and the housing top. |
| A5 | Handle swing plane = frontal plane. If tilted by τ, torques scale by cos τ. TODO. | |
| A6 | Profile `linear_eased`, EASE_FRACTION = 0.2 (last 60 mm blend to zero angular speed). | Removes the step into the dwell. |
| A7 | Pressure-angle limit 55° for a ball-bearing roller in a printed slot. Not a standard; test 2 checks it. | Sets Y0 for a given width. |
| A8 | Envelope model: arm = slot centre line thickened by slot/2 + 4 mm rim, plus a Ø40 pivot boss; housing = widest swept part + 2 mm clearance + 3 mm wall per side. ±5 mm precision at best. | Replaces the rev-2 proxy. |
| A9 | Carriage pulleys 4 side by side on one axle (150 mm wide); fixed pulleys likewise at the top. | Height. |

### 1.4 Open questions — status
| # | Question | Answer / status |
|---|---|---|
| Q1 | Pivots may move apart? | Yes. Width target 150–170 mm. |
| Q2 | Hand separation change? | ΔS = 70 mm baseline (106 → 176), 60/70/80 shown. |
| Q3 | Dwell required? | Yes. 300 mm map, dwell to 400 mm. |
| Q4 | Mapping profile? | Linear for v1 (now linear + 20 % easing); one parameter. |
| Q5 | Stack loads? | Unknown; F_HAND = 200 N assumed. Tests start at a few kg. TODO. |
| Q6 | Which machine? | Unknown, TODO. |
| Q7 | Actuation resistance | R0 ≤ 10 N no-load; R_load ≤ ~10 % of stack (TODO confirm). |
| Q8 | Swivel? | None in housing for v1. |
| Q9 | Spring? | Extension spring (E). |
| Q10 | Failure bias? | Closed; done by the +3° offset. |
| Q11 | R_load margin: 39.9 N vs 40 N goal at ΔS = 70. | Open. ΔS = 60 gives ~36 N. |
| Q12 | Housing height: 213 mm above the pivots at n = 8. | Open. |
| Q13 (new) | **Is 172 mm × 213 mm at a 55° pressure angle acceptable for E, or is ≤ 170 mm hard?** If hard at a conventional pressure angle, C is the answer (Section 4). | Open — decides the architecture. |
| Q14 (new) | Are 7 pulleys + 7 bearings (n = 8) acceptable, or should n = 6 (5 pulleys, 188 mm, 59°) be the baseline? | Open. |
| — | Pivot bolts | **M6/M8 allowed** (reviewer). |

### 1.5 Definition of "actuation resistance" (accepted, Q7)
`R` = tension in the reference cable during the down stroke. Always reported as a pair:
- **R0** (no hand load; spring + friction only), goal ≤ 10 N. Baseline: **5.6 N**.
- **R_load** (design hand load 200 N/hand), goal ≤ 10 % of stack (TODO). Baseline: **39.9 N**.

---

## 2. Architectures

(Unchanged from rev 2 except where noted; see rev 2 for the full text of A–D.)

**A. Spool + compound spur train** — 2–3 stages, sector gears for symmetry, slip clutch needed
for overtravel, output teeth carry the hand torque (m ≥ 2). ≈ 15 parts.

**B. Spool + worm** — single stage but η ≈ 0.26–0.34 and the cable must drive *against* the hand
torque through it. Not worth it.

**C. Threaded drum / barrel cam + runner + two links** — drum turns 2.4–3.2 times, runner moves
~40 mm on a 12–20 mm lead (η ≈ 0.5), links to 50 mm short arms on the bell cranks. Dwell = a
zero-lead section of the barrel-cam groove. Clock spring, or an extension spring on a cord wound
on a second drum section. **Real envelope (rev 3, hand estimate):** the 50 mm short arm pointing
30° inboard at closed swings to vertical at open, so the only outboard material is the pivot
boss: width ≈ P + 2 × 20 + 2 × 5 ≈ **149 mm**; height ≈ 150–170 mm above the pivots (drum,
runner travel, links — TODO layout). ≈ 12 parts.

**D. Reeved cable + carriage + two links** — as E without the slots; compact like C (~150 mm)
and efficient, but no dwell and no way to hold the handles against the closing torque in a
lost-motion section.

**E. Reeved cable + carriage + slotted bell-crank handles** — selected in rev 2. Rev 3 adds:
- Slot = `Rot(−β(s)) · (pin − pivot)`, pin = (0, Y0 + s/n), pivot = (P/2, 0). The slot's far end
  at closed is `Rot(−30°) · (−P/2, Y0 + 400/n)` outboard of the pivot. For n = 8, Y0 = 80:
  (22.3, 137.2) → 49.3 + 22.3 + 9.2 rim = 80.8 mm from the centre line → width 172 mm.
  For n = 4, Y0 = 150: (82.3, 241.2) → 140.8 mm → **292 mm** (rev 2 error).
- Pressure angle: `tan α = (dy/dβ + P/2) / y_pin` (the P/2 term is the arm material at the pin
  moving down as the arm opens). At closed, n = 8, Y0 = 80: (64.6 + 49.3)/80 → 55°.
- Pin force `= 2 T · n · dβ/ds`: peak 218 N at s ≈ 240 mm (n = 8). Cable tension is
  unchanged by n; pin, roller and slot loads scale with n.
- Parts: n − 1 = 7 pulleys + 7 bearings, carriage, 2 guide rods, 2 slotted arms, pin + roller,
  spring, dead-end clamp, housing (≈ 16 incl. pulleys).

---

## 3. Weighted comparison (re-scored with real envelopes)

Scores 1–5, weights sum to 100. Changes from rev 2 marked (old → new).

| Criterion | Wt | A | B | C | D | E |
|---|---|---|---|---|---|---|
| Meets mapping incl. dwell | 15 | 2 | 2 | 5 | 2 | 5 |
| Actuation force (R0 / R_load) | 20 | 3 | 1 | 2 | 5 | 4 |
| Compactness (real envelope) | 10 | 3 | 4 | 4→**5** (149 × ~160) | 3→**4** (~150) | 3→**2** (172 × 213 at 55°) |
| Part count & cost | 10 | 2 | 4 | 4 | 3 | 4→**3** (7 pulleys) |
| Printability | 10 | 3 | 2 | 3 | 4 | 5→**4** (55° slot) |
| Robustness under hand load | 10 | 2 | 2 | 3 | 4 | 4→**3** (157 N wall, 218 N pin) |
| Ease of assembly | 5 | 2 | 3 | 4 | 3 | 4→**3** (8-part reeving) |
| Ease of return | 5 | 3 | 1 | 3 | 5 | 5 |
| Safety | 15 | 3 | 3 | 4 | 3 | 4 |
| **Total /500** | | **260** | **235** | **360** | **365** | **375** |

Check E: 75 + 80 + 20 + 30 + 40 + 30 + 15 + 25 + 60 = 375. Check C: 75 + 40 + 50 + 40 + 30 + 30
+ 20 + 15 + 60 = 360. D: 30 + 100 + 40 + 30 + 40 + 40 + 15 + 25 + 45 = 365.

**Does the ranking change?** No: E 375 > D 365 > C 360 > A > B. **But the margin between E and C
has gone from 70 to 15 points, which is inside the noise of my scoring.** D is only ahead of C
because it has no dwell problem *scored* — it fails the dwell requirement outright (score 2 on a
weight-15 criterion) and is not a candidate. The real decision is E vs C.

---

## 4. Recommendation (rev 3)

**Primary: E, conditionally.** Baseline n = 8, Y0 = 80 mm, ΔS = 70: 172 mm wide, 213 mm above
the pivots, pressure angle 55° → 46° over the drive, R0 = 5.6 N / R_load = 39.9 N, cable never
slack. It is the only architecture that meets the dwell, the R0 goal and (just) the R_load goal.

**The condition:** the first printed part is the slotted arm + roller pin on a test fixture
(build step 2), loaded to 7.7 N·m. If the slot wall at 46–55° pressure angle shows wear or
spreading after a few hundred cycles, or the team decides ≤ 170 mm is hard at a conventional
pressure angle, **switch to C** — nothing else in the project changes (same bell-crank geometry,
same housing top, same anchor).

**Fallback: C.** 149 mm wide, compact, dwell for free, but R0 ≈ 10 N (at the goal) and R_load
≈ 60 N (ideal 27 N / η ≈ 0.5 + spring), i.e. 1.5× the goal; printed thread under ~190 N axial
load; needs a rotary return (clock spring or cord-on-drum + extension spring).

What would change my mind right now: a hard width limit ≤ 170 mm **and** a pressure-angle
limit ≤ 45° together → C. A relaxed width (≥ 210 mm) → E at Y0 = 120 (43°, 137 N wall) becomes
comfortable and the condition above disappears.

---

## 5. First-pass layout for E

### 5.1 Width / pressure-angle / height trade-off (ΔS = 70, F = 200 N)
From `kinematics.py` section 2. Width and height are housing outsides; wall = peak slot-wall
normal force on one arm; η = 0.90 × 0.97^(n−1).

| n | Y0 [mm] | η | width [mm] | height [mm] | pressure angle | wall max [N] | R0 / R_load [N] |
|---|---|---|---|---|---|---|---|
| 4 | 50 | 0.82 | 192 | 233 | 74° | 104 | 8.8 / 39.8 |
| 4 | 120 | 0.82 | 262 | 303 | 56° | 77 | 8.8 / 39.8 |
| 4 | 150 (rev 2) | 0.82 | **292** | 333 | 50° | 71 | 8.8 / 39.8 |
| 4 | 180 | 0.82 | 322 | 363 | 45° | 68 | 8.8 / 39.8 |
| 6 | 50 | 0.77 | 158 | 200 | 70° | 148 | 5.5 / 38.3 |
| 6 | 80 | 0.77 | 188 | 230 | 59° | 123 | 5.5 / 38.3 |
| 6 | 120 | 0.77 | 228 | 270 | 48° | 107 | 5.5 / 38.3 |
| 8 | 50 | 0.73 | 160 | 183 | 66° | 190 | 4.0 / 38.3 |
| **8** | **80** | **0.73** | **172** | **213** | **55°** | **157** | **5.6 / 39.9** |
| 8 | 120 | 0.73 | 212 | 253 | 43° | 137 | 5.6 / 39.9 |
| 8 | 150 | 0.73 | 242 | 283 | 37° | 129 | 5.6 / 39.9 |

(R0/R_load in the 4- and 6-part rows are with the 20 N preload of rev 2; the 8-part baseline
uses 30 N, Section 5.4.) Rules of thumb from the table: **+10 mm Y0 = +10 mm width, −3.5° pressure
angle, +10 mm height**; going from n = 4 to 8 saves ~90 mm of width at equal pressure angle but
costs 4 pulleys and 9 points of efficiency. A larger θ_closed (10–15°) saves a further 10–20 mm
but raises R_load to 47–55 N; not taken.

**Can E meet 150–170 mm?** Only at a pressure angle ≥ 55° (n = 8, Y0 ≤ 78) or ≥ 66° (160 mm).
At ≤ 45° it needs ≥ 210 mm. The baseline sits 2 mm over the target — inside the ±5 mm envelope
precision, but reported as FAIL by the check.

### 5.2 Handle geometry — offset bell crank (unchanged values)
θ = +3° → +33°, `L_LINE = ΔS / (2(sin θ_open − sin θ_closed))`, `P = S_closed − 2 L_LINE sin θ_closed`.

| ΔS | S closed → open | L_LINE | P | T closed | T open (F = 200 N) | width (n = 8, Y0 = 80) |
|---|---|---|---|---|---|---|
| 60 | 106 → 166 | 60.9 | 99.6 | 0.64 | 6.64 N·m | 172 |
| **70** | 106 → 176 | 71.1 | 98.6 | 0.74 | 7.74 N·m | 172 |
| 80 | 106 → 186 | 81.3 | 97.5 | 0.85 | 8.85 N·m | 171 |

ΔS barely affects the envelope (the slot geometry depends on P, which changes by 2 mm); it sets
the torque and therefore R_load (36 / 40 / 45 N) and the wall loads.

### 5.3 Slot profile with easing
Profile `g(u)` for stroke fraction `u`, easing fraction `e` = 0.2:

    g = m u                         for u ≤ 1 − e          (constant slope, m = 1/(1 − e/2) = 1.111)
    g = 1 − m (1 − u)² / (2e)       for u > 1 − e          (slope falls linearly to 0 at u = 1)

β = 30° · g. The slope in the constant part is 11 % steeper than pure linear so that 110° is still
reached exactly at 300 mm, now with zero angular speed. Slot centre line (arm frame, n = 8,
Y0 = 80, ΔS = 70):

| s | 0 | 50 | 100 | 150 | 200 | 240 (ease start) | 275 | 300 | 350 | 400 |
|---|---|---|---|---|---|---|---|---|---|---|
| x | −49.3 | −40.7 | −30.5 | −18.9 | −5.9 | 5.4 | 13.3 | 16.1 | 19.2 | 22.3 |
| y | 80.0 | 90.6 | 100.3 | 108.7 | 115.8 | 120.5 | 123.8 | 126.4 | 131.8 | 137.2 |
| pressure angle | 55° | 53° | 51° | 49° | 47° | 46° | 61° | 90° | dwell | dwell |

Slot length ≈ 75 mm drive + 12.5 mm dwell; width 10.3 mm (Ø10 roller + 0.3). The pressure angle
rises through the easing zone, but the pin force falls to zero at the same time, so the wall load
stays bounded (Section 6.5).

### 5.4 Spring (rev 3)
Extension spring on the carriage, **preload 30 N**, rate 0.10 N/mm → 35 N at the end of the
dwell (50 mm carriage travel). With 8 parts the cable sees (spring ± friction)/8:
no-load down 4.8 → 5.6 N (= R0), no-load up 2.7 → 3.2 N (≥ 2 N margin). The rev-2 20 N preload
gave 1.8 N on the up stroke — too close to slack. **R0 = 5.6 N / R_load = 39.9 N.**

### 5.5 Pivots and synchronisation
M6 or M8 pivot bolts (allowed by the reviewer) in two bushings or ball bearings per handle.
Synchronisation = one pin in two mirrored slots.

### 5.6–5.8 Cable, mount, guarding — unchanged from rev 2
Anchor strap above (Q6 TODO); fairlead; ≥ 150 mm free cable; all cable runs enclosed; closed stop
on the carriage; open stop = dwell + carriage upper stop; handle exit slots with ≥ 5 mm clearance.
Reeving at n = 8: 4 carriage pulleys on one axle (150 mm wide), 3 fixed pulleys + dead end on the
housing top.

---

## 6. Carriage force balance, sign change, easing and wall loads

### 6.1–6.2 Reviewer's estimate and model — unchanged from rev 2
Symmetric ±15° geometry: 42 N up on the carriage vs 20 N spring at closed → slack. Model:
`N·T_c = F_spring + F_pin ± F_friction`, `F_pin = 2T · n · dβ/ds`, friction = (1 − η)(spring + |pin|).

### 6.3 Results — baseline n = 8, Y0 = 80, ΔS = 70, spring 30 N + 0.1 N/mm, η = 0.73
| F | s | T [N·m] | pin [N] | spring [N] | friction [N] | T_c down [N] | T_c up [N] | wall [N] | α |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 30.0 | 8.2 | 4.8 | 2.7 | 0 | 55° |
| 0 | 400 | 0 | 0 | 35.0 | 9.5 | **5.6 = R0** | 3.2 | 0 | — |
| 200 | 0 | 0.74 | 23 | 30.0 | 14.5 | 8.4 | 4.8 | 20 | 55° |
| 200 | 100 | 3.47 | 108 | 31.2 | 37.9 | 22.1 | 12.6 | 85 | 51° |
| 200 | 200 | 6.06 | 188 | 32.5 | 60.2 | 35.1 | 20.0 | 139 | 47° |
| 200 | 225 | 6.68 | 207 | 32.8 | 65.5 | 38.2 | 21.8 | 150 | 46° |
| 200 | ~240 | 7.0 | **218** | 33.0 | 67 | **39.9 = R_load** | 22.6 | **157** | 46° |
| 200 | 250 | 7.26 | 188 | 33.1 | 60.2 | 35.1 | 20.1 | 142 | 49° |
| 200 | 275 | 7.62 | 99 | 33.4 | 36.0 | 21.0 | 12.0 | 101 | 61° |
| 200 | 300 | 7.74 | 1 | 33.8 | 9.3 | 5.4 | 3.1 | 66 | 90° |
| 200 | 400 | 7.74 | 0 | 35.0 | 9.5 | 5.6 | 3.2 | 60 | dwell |

First slack: never, for ΔS ∈ {60, 70, 80} × F ∈ {0, 100, 200}. **R0 = 5.6 N / R_load = 39.9 N.**

### 6.4 Fixes for the sign change — unchanged conclusion
(a) stiff spring: R0 25–31 N, rejected; **(b) offset +3° → +33°: chosen**; (c) detent: rejected;
(d) opening-everywhere: rejected. Rev 2 Section 6.4 has the arithmetic.

### 6.5 Easing zone: cable-tension curve and dwell side load (new)
Without easing (rev 2) the pin force was 108 N up to s = 300 and 0 after it: the cable tension
stepped from 40.7 N to 8 N in zero distance, which the user would feel as a clunk and the slot
end would take as an impact. With `EASE_FRACTION` = 0.2 the pin force follows dβ/ds down to zero
over the last 60 mm:

| s [mm] | 200 | 225 | 240 | 250 | 275 | 300 | 325 |
|---|---|---|---|---|---|---|---|
| T_c down, rev 2 (no easing) | 30.8 | 33.4 | 35.0 | 35.8 | 38.3 | 40.7 → 8.2 | 8.6 |
| T_c down, rev 3 (eased) | 35.1 | 38.2 | 39.9 | 35.1 | 21.0 | 5.4 | 5.4 |

The peak moves earlier (s ≈ 240) and is slightly higher (the constant-slope part is 11 %
steeper), then falls smoothly to the dwell value. TODO: the quadratic blend has a slope
discontinuity at u = 0.8 (jerk, not a force step); a cubic/cosine blend is one line in `profile()`.

**Slot-wall load.** One formula covers drive, easing and dwell: the wall force acts along the slot
normal through the pin, and its moment about the pivot balances the hand torque,
`F_wall = T / |r_pin × n̂|`.
- Drive section: equals (F_pin/2)/cos α; peak **157 N at s ≈ 240 (α = 46°)**.
- Dwell: the normal is horizontal, the lever is simply y_pin: F_wall = 7.74 N·m / 0.118–0.130 m =
  **66 → 60 N per arm, horizontal, inward on the pin**. The two arms' loads cancel at the pin when
  the hands are equal; an unequal load puts the difference into the carriage guide rods.
- Pin bending: 218 N over a 12 mm span → σ ≈ 104 MPa for M4 (steel OK, TODO grade); M5 halves it.

---

## 7. Calculations to verify by hand (updated where rev 3 changed them)

7.1 Angle vs travel: β = 30 · g(s/300); at s = 150: g = 1.111 × 0.5 = 0.556 → β = 16.7°, included 83.3°.
7.2 k = 573 mm/rad; dy/dβ = k/n = 71.6 mm/rad (n = 8); in the constant-slope part the effective value is 71.6/1.111 = 64.5.
7.3 Pin force at s = 240: T = 200 × 0.0711 × sin(3° + 26.7°) = 7.05 N·m; dβ/ds = 1.111 × 0.5236/300 = 1.94 × 10⁻³ rad/mm; F_pin = 2 × 7050 × 8 × 1.94 × 10⁻³ = 219 N ✓.
7.4 Pressure angle at closed: tan α = (64.5 + 49.3)/80 = 1.42 → 54.9° ✓.
7.5 Envelope: far slot end at closed = Rot(−30°)(−49.3, 130) = (22.3, 137.2); x = 49.3 + 22.3 + 9.2 = 80.8; width = 2(80.8 + 2 + 3) = 171.6 ✓. Rev 2: Rot(−30°)(−49.3, 250) = (82.3, 241.2) → 140.8 → 292 ✓.
7.6 Anchor offset: 100 mm at 1 m → −0.4 % payout. Unchanged.
7.7 Spring: (30 + 0.27 × 30)/8 = 4.8 N down, (30 − 8.2)/8 = 2.7 N up at closed ✓.
7.8 Dwell wall load: 7.74 / 0.130 = 60 N ✓.
7.9 C envelope: short arm tip at closed 49.3 − 50 sin 30° = 24 mm from the centre line, at open 49 mm; outboard extent = boss → 149 mm.

---

## 8. Top 10 risks and tests (rev 3 changes marked)

| # | Risk | Test |
|---|---|---|
| 1 | Hand torque higher than assumed → R_load felt | Spring balance in the reference cable during pushdowns at 3 stack settings from a few kg. |
| 2 | **Slot wall wear/spreading at 46–55° pressure angle, 157 N (go/no-go for E)** | 500-cycle rig at 7.7 N·m per handle; measure slot width; inspect roller. |
| 3 | Carriage binds (8-part reeving, 4 pulleys on one axle, unequal hand loads) | One-sided 100 N load; measure R0 either way. |
| 4 | Cable jumps a pulley (up-stroke tension only 2.7 N no-load) | 50 fast releases; inspect. Raise preload if needed. |
| 5 | Jolt or jerk at the easing/dwell transition; inertia not modelled | 400 and 450 mm strokes; accelerometer on the housing. |
| 6 | Asymmetry from print tolerance | Protractor jig at 5 stroke points; ≤ 2°. |
| 7 | Housing top / eye fails across layers | Static hang to 2× intended max stack (TODO) with a safety strap. |
| 8 | Anchor slips or cables snag | 100 reps at 3 heights on the real machine. |
| 9 | R0 > 10 N once assembled (model 5.6 N with η = 0.73 assumed) | Spring balance, no hand load, 0–100 % stroke + dwell. |
| 10 | Torque sign flips at closed because of tolerance (+3° bias) | 5 kg per grip at closed with the cable slack; handles must stay closed. |

---

## 9. CAD part list and build order — unchanged from rev 2
except: `pulley` ×7 (n = 8); `carriage` carries 4 pulleys on one axle; step 2 (slotted arm + pin
test fixture) is now the **decision test** between E and C.

*End of study. Nothing above is validated; Section 8 is the validation plan.*
