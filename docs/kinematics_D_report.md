# Architecture D — kinematics verification report

Date 2026-11-10. Script: `kinematics_D.py` (run `python3 kinematics_D.py --plots`). Inputs:
`params.py`. Plots in `docs/img/D_*.png`. Nothing here is validated by test; this is the
check CLAUDE.md rule 5 asks for before any geometry is generated.

Tags: **VERIFIED** = computed by `kinematics_D.py` from the formulas shown; **ASSUMPTION** = my
choice; **TODO** = unknown.

## 0. What was checked and the outcome in one table

| Item | Start value (buildability.md) | Result | Status |
|---|---|---|---|
| Carriage travel for 30° | 37.6 mm with arm 70 / link 60 / arm horizontal | **32.4 mm** (stop at s = 259 mm) | did not close — corrected, §5 |
| Mapping linearity | "linear-ish, a few %" | start geometry +8.2° (13.6 %); corrected geometry **−0.37° (0.6 %)** | corrected |
| Carriage direction on payout | up (implied) | **down** — inward arm joint moves down when opening | did not close — corrected, §5 |
| Carriage width | 4 pulleys in a row, 150 mm, inside a 149 mm housing | 150 > 149; **2 rows of 2 = 70 mm** | did not close — corrected, §5 |
| Torque sign | never reverses | θ = 3° → 33°, T ≥ 0 at all F | VERIFIED PASS |
| Transmission angle | — | min **67.8°** (limit 30°) | VERIFIED PASS |
| R0 / R_load | ~5–9 N / ~40 N | **5.4 N / 41.5 N** (goals ≤ 10 / ≤ 40) | PASS / **FAIL by 1.5 N** |
| Cable never slack | — | min tension 2.73 N (≥ 2 N margin) | VERIFIED PASS |
| Housing width / height | ~149 / ~190 | **148.6 mm / 235.5 mm** (201 above, 34 below the pivot line) | VERIFIED (layout ASSUMPTION) |
| Interference margins (2D) | — | all positive, min **14.4 mm** | VERIFIED PASS |
| 3D stack (addendum §9) | arm R, arm L, link R, link L | joint pins collide with the far arm by −9.6 mm; **re-stacked** arm R, link R, link L, arm L → min 3.6 mm | corrected |
| Grips / hands vs hubs (addendum §10) | — | hand envelope −12.9 mm into the R10 hub; joint exposed at open | **open — needs a decision** |
| Reeving lanes (addendum §11) | 2 × 2 in plane | cannot be reeved; crossed-axle layout: 8 lanes, 0 fleet; carriage 98 mm, depth 65 mm | corrected |

## 1. Model

Right handle, housing frame, x outward, y up, pivot O = (P/2, 0), P = 98.56 mm (from
`params.pivot_spacing_mm`, ΔS = 70, θ_closed = 3°).

    joint    A = O + a · (cos α, sin α),   α = 180° − ψ0 + β        (ψ0 = arm elevation above the inward horizontal at closed)
    pin      C = (0, y_c),   y_c = A_y + √(c² − A_x²)                (link of length c, carriage pin ABOVE the joint — ASSUMPTION)
    travel   x_carriage(β) = y_c(0) − y_c(β)                         (positive DOWN)
    mapping  y_c(s) = y_c(0) − s/n  → β(s) solved by bisection, n = 8
    stop     s_stop = n · x_carriage(30°)

Direction: opening is a CCW rotation of the right handle (the grip below the pivot moves
outward). A joint inboard of the pivot has velocity ω × r = ω(−A_y', A_x') with A_x' < 0, so its
vertical velocity is negative: the joint moves **down** when the handle opens. The link therefore
pulls the carriage down and the carriage must move down as cable pays out: fixed pulleys below the
carriage, return spring pulling up. (buildability.md §1 had the joint rising — sign error.)

Loads (statics, frictionless joints, both handles equal):

    T      = F · L_LINE · sin(θ),   θ = θ_closed + β,   L_LINE = 71.1 mm
    F_link = T / (a · sin μ)          μ = angle between arm O→A and link A→C
    F_pin  = 2 F_link · cos λ         λ = link angle from vertical;  + = pushes carriage UP (closing)
    check  F_pin = 2 T · n · dβ/ds    (virtual work — agrees with statics to 0.1 N at every point)
    n T_c  = F_spring + F_pin ± F_fric,   F_fric = (1 − η)(F_spring + |F_pin|),   η = 0.90 × 0.97⁷ = 0.727

The friction model is the one in main_design.md A3 (ASSUMPTION). Spring: 30 N preload +
0.10 N/mm × carriage travel (params; "as bought, real value TBD" per CLAUDE.md).

Final geometry (VERIFIED, `kinematics_D.py` scan over ψ0 ∈ {0…30°} × c ∈ {50…100} with a solved
for 37.5 mm of travel each time; chosen for the smallest mapping error with margins ≥ 15 mm):

| Parameter | Value | Tag |
|---|---|---|
| Arm a (pivot → link joint) | **70.3 mm** | VERIFIED (solved; 70.33 gives exactly 300.0, 70.3 gives 299.8) |
| Link c | **70.0 mm** | VERIFIED (scan) |
| Arm elevation at closed ψ0 | **25°** above inward horizontal (→ −5° at open) | VERIFIED (scan) |
| Pivot spacing P | 98.56 mm | VERIFIED (`params.pivot_spacing_mm`) |
| L_LINE | 71.1 mm | VERIFIED (`params.line_length_mm`) |
| Carriage travel at 30° | 37.48 mm | VERIFIED |
| Stop at s | 299.8 mm (target 300) | VERIFIED |
| Pin height y_c: closed / stop | 98.2 / 60.7 mm above the pivot line | VERIFIED |
| Joint position: closed / stop | (−14.4, 29.7) / (−20.8, −6.1) mm | VERIFIED |

## 2. Check 1 — handle angle vs carriage travel (5 mm steps)

| carriage [mm] | s [mm] | β [°] | included [°] | linear target [°] | deviation [°] |
|---|---|---|---|---|---|
| 0 | 0 | 0.00 | 50.00 | 50.00 | +0.00 |
| 5 | 40 | 4.04 | 58.08 | 58.00 | +0.08 |
| 10 | 80 | 8.01 | 66.01 | 66.00 | +0.01 |
| 15 | 120 | 11.94 | 73.87 | 74.00 | −0.13 |
| 20 | 160 | 15.86 | 81.73 | 82.00 | −0.27 |
| 25 | 200 | 19.82 | 89.64 | 90.00 | −0.36 |
| 30 | 240 | 23.83 | 97.67 | 98.00 | −0.33 |
| 35 | 280 | 27.93 | 105.86 | 106.00 | −0.14 |
| 37.48 | 299.8 | 30.00 | 110.00 | 109.97 | +0.03 |

50° → 110° over 299.8 mm: VERIFIED. **Max deviation from linear −0.37° at s = 213 mm = 0.6 %
of the 60° range** (1 mm scan). The start geometry gave +8.18° (13.6 %). Plot: `docs/img/D_angle_vs_travel.png`.

## 3. Check 2 — torque sign

θ = 3° + β runs 3.0° → 33.0°; T = F · 0.0711 · sin θ.

| F [N/hand] | T min [N·m] | T max [N·m] | sign change? |
|---|---|---|---|
| 0 | 0.000 | 0.000 | no (zero) |
| 100 | +0.372 (closed) | +3.872 (stop) | no |
| 200 | +0.744 (closed) | +7.744 (stop) | no |

VERIFIED PASS: the hand load always drives the handles closed; the link is always in compression
and the carriage always pushed up against the cable. Plot: `docs/img/D_torque_vs_travel.png`.

## 4. Check 3 — mechanism loads (every 25 mm, F = 0 / 100 / 200 N per hand)

Columns: T hand torque per handle; μ transmission angle; F_link per link (compression); F_pin on
the carriage (up); T_c cable tension on the down / up stroke. Spring 30.0 → 33.7 N; friction
(1 − η)(spring + |pin|).

| s [mm] | T₁₀₀ | F_link₁₀₀ | F_pin₁₀₀ | T_c down₁₀₀ | T_c up₁₀₀ | T₂₀₀ | μ [°] | F_link₂₀₀ | F_pin₂₀₀ | spring | fric₂₀₀ | T_c down₂₀₀ | T_c up₂₀₀ |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.37 | 5.4 | 10.6 | 6.47 | 3.69 | 0.74 | 76.9 | 10.9 | 21.3 | 30.0 | 14.0 | 8.16 | 4.66 |
| 25 | 0.69 | 9.9 | 19.3 | 7.89 | 4.51 | 1.37 | 80.5 | 19.8 | 38.6 | 30.3 | 18.8 | 10.96 | 6.26 |
| 50 | 0.99 | 14.2 | 27.6 | 9.27 | 5.29 | 1.99 | 83.9 | 28.4 | 55.2 | 30.6 | 23.4 | 13.66 | 7.80 |
| 75 | 1.30 | 18.5 | 35.7 | 10.61 | 6.06 | 2.59 | 87.2 | 36.9 | 71.5 | 30.9 | 27.9 | 16.29 | 9.31 |
| 100 | 1.60 | 22.7 | 43.8 | 11.94 | 6.82 | 3.19 | 89.6 | 45.4 | 87.5 | 31.2 | 32.4 | 18.90 | 10.80 |
| 125 | 1.89 | 26.9 | 51.8 | 13.26 | 7.58 | 3.78 | 86.5 | 53.9 | 103.6 | 31.6 | 36.9 | 21.51 | 12.29 |
| 150 | 2.18 | 31.2 | 59.9 | 14.60 | 8.34 | 4.37 | 83.6 | 62.5 | 119.8 | 31.9 | 41.4 | 24.13 | 13.79 |
| 175 | 2.47 | 35.6 | 68.2 | 15.96 | 9.12 | 4.94 | 80.7 | 71.2 | 136.3 | 32.2 | 46.0 | 26.81 | 15.32 |
| 200 | 2.76 | 40.1 | 76.6 | 17.36 | 9.92 | 5.51 | 78.0 | 80.2 | 153.2 | 32.5 | 50.7 | 29.55 | 16.88 |
| 225 | 3.04 | 44.7 | 85.3 | 18.80 | 10.74 | 6.08 | 75.3 | 89.4 | 170.7 | 32.8 | 55.5 | 32.38 | 18.50 |
| 250 | 3.32 | 49.5 | 94.4 | 20.29 | 11.59 | 6.64 | 72.7 | 99.0 | 188.8 | 33.1 | 60.5 | 35.31 | 20.17 |
| 275 | 3.60 | 54.4 | 103.8 | 21.84 | 12.48 | 7.20 | 70.2 | 108.8 | 207.7 | 33.4 | 65.8 | 38.37 | 21.92 |
| 299.8 | 3.87 | 59.5 | 113.7 | 23.45 | 13.40 | 7.74 | 67.8 | 119.0 | 227.3 | 33.7 | 71.2 | 41.54 | 23.73 |

F = 0: F_link = F_pin = 0; T_c down 4.77 → 5.37 N, up 2.73 → 3.07 N.

| Result | Value | Goal | Status |
|---|---|---|---|
| **R0** (no load, down stroke max) | **5.37 N** | ≤ 10 N | VERIFIED PASS |
| **R_load** (200 N/hand, down stroke max) | **41.54 N** | ≤ 10 % × 400 N = 40 N (TODO confirm goal) | **FAIL by 1.5 N** |
| Minimum cable tension, any case (no-load up stroke at closed) | 2.73 N | ≥ 2 N margin | VERIFIED PASS |
| Peak link force | 119 N compression | — | VERIFIED |
| Peak carriage force | 227 N | — | VERIFIED |

Note on R_load: the two ways to meet 40 N are ΔS = 60 (torque −14 %, estimated ~36 N — not run
here) or a lighter carriage spring (the spring contributes 33.7 × 1.27 / 8 = 5.4 N of the 41.5).

**Carriage stop load** (v2 with the anchor spring; the cable pulls the carriage down onto the stop):

    stop reaction = n · T_anchor − F_spring(stop) − F_pin(stop)

| Anchor tension | F = 0 | F = 100 N | F = 200 N |
|---|---|---|---|
| 50 N (preload, ASSUMPTION) | 366 N | 253 N | 139 N |
| 70 N (ASSUMPTION; rate TODO) | 526 N | 413 N | 299 N |

The stop load is highest with **no** hand load (the hand torque pushes the carriage *off* the
stop). v1 without the anchor spring: once at the stop the cable simply goes taut; the stop load
is 8 × whatever extra tension the user applies — not computable, and the reason v1 testing must
stay within 300 mm or at low loads. Plot: `docs/img/D_cable_tension_vs_travel.png`.

## 5. Check 4 — geometry, envelope, interference

| Quantity | Value | Tag |
|---|---|---|
| Arm / link / arm angle at closed | 70.3 mm / 70.0 mm / 25° | VERIFIED |
| Pivot spacing P | 98.56 mm | VERIFIED |
| Carriage travel | 37.48 mm (s/8) | VERIFIED |
| Transmission angle μ (arm–link) | min **67.8°** at the stop; 76.9° closed; peaks at 89.6° (s = 100) | VERIFIED PASS (limit 30°) |
| Link angle from vertical | 11.9° closed → 17.2° open | VERIFIED |
| Housing width | **148.6 mm** = 2 × (49.3 + 20 boss + 2 + 3) | VERIFIED; carriage 70 mm (2 × Ø30 + 10) does not govern |
| Housing height | **235.5 mm**: top 201.2 above the pivot line, bottom 34.3 below | VERIFIED under the layout ASSUMPTION below |
| Layout | carriage pin at 98.2 → 60.7; carriage body ±10 mm; 2 pulley rows above the pin (pitch 40); fixed pulley rows at y = +25.7 and −14.3 (below the carriage's lowest row), in a z-plane behind the arms; spring location **TODO** (beside the carriage or above it adds height) | ASSUMPTION |
| z-stack | arm R 0–8, arm L 8.3–16.3, link R 16.6–22.6, link L 22.9–28.9 mm; cable/pulleys behind at z ≥ 30 | ASSUMPTION (D_ARM_THICK_MM, 0.3 gap) |

Interference margins (2D, in the model; negative would be a collision):

| s [mm] | arm → carriage body | arm joint → opposite pivot bolt (M8) | link → opposite pivot bolt | arm joint past the centre line |
|---|---|---|---|---|
| 0 | 50.5 | 33.8 | 35.8 | 14.4 (z-stacked, by design) |
| 75 | 49.7 | 25.9 | 27.9 | 17.8 |
| 150 | 49.1 | 19.8 | 21.8 | 19.9 |
| 225 | 48.8 | 16.5 | 18.5 | 20.9 |
| 299.8 | 43.8 | 17.2 | 19.2 | 20.8 |

Minimum margin **14.4 mm** (the joint crossing the centre line, which the z-stack allows). Arm
plate width 16 mm and link width 12 mm are ASSUMPTIONS used for these margins. Not checked here:
the long handle arms vs the fixed pulleys below the pivot line (different z-plane by the layout
assumption), the guide rods, and the 3D carriage — those belong to the assembly interference check.
Plot: `docs/img/D_transmission_angle_vs_travel.png`.

## 6. Check 5 — values in buildability.md that did not close

1. **Joint direction.** buildability §1 wrote the arm tip rising with the handle opening; it
   falls (§1 above). Consequence: the carriage moves **down** on cable payout, so the fixed
   pulleys go below the carriage and the return spring pulls up. The housing therefore extends
   ~34 mm below the pivot line (fixed pulleys behind the arms).
2. **Carriage travel.** "arm 70, link 60 → 37.6 mm" was computed with the wrong direction.
   Correct value for that geometry: y_c(0) = 56.31, y_c(30°) = 23.92 → **32.39 mm**, stop at
   8 × 32.39 = 259 mm. Corrected geometry: a = 70.3, c = 70, ψ0 = 25° → 37.48 mm, stop at 299.8 mm.
3. **Mapping.** "linear-ish, a few %" was 13.6 % for the start geometry; 0.6 % after correction.
4. **Carriage width.** 4 pulleys on one axle = 4 × 30 + 3 × 10 = 150 mm, which cannot fit in a
   149 mm housing. Corrected: 2 rows of 2 → 70 mm wide, +40 mm of height. (CLAUDE.md still says
   "4 pulleys on one axle"; it should be updated.)
5. **Housing height** "~190 mm" → 235.5 mm with the corrected layout (201 above + 34 below).
6. **R_load** "~38–40 N" → 41.5 N with the corrected link geometry (the first-pass number used
   E's dβ/ds; D's is slightly higher near the stop).

buildability.md itself has not been edited; a correction note pointing here was appended.

## 7. Assumptions carried (listed as requested)

- A-D1 The link attaches on the **inward** side of the pivot and runs **up** to the carriage pin
  on the centre line; the pin is the upper root of the link circle.
- A-D2 Carriage pin on the centre line (x = 0), shared by both links; links in separate z-planes.
- A-D3 Friction model A3 with η = 0.727 (0.90 slide × 0.97⁷ pulleys); the link pins are included in the 0.90.
- A-D4 Spring 30 N + 0.10 N/mm (CLAUDE.md "as bought, real value TBD").
- A-D5 Layout: pulley rows pitch 40 mm, carriage body ±10 mm around the pin, fixed pulleys
  below the carriage behind the arms, spring location TODO.
- A-D6 Arm plate 16 × 8 mm, link 12 mm wide, pivot bolt M8, boss R20 — for margins only.
- A-D7 Anchor-spring tension 50 N (preload) and 70 N for the stop-load table; rate TODO.
- A-D8 F_HAND = 200 N/hand, equal hands, vertical (Q5 open).
- matplotlib (already installed, v3.9.4) was used for the plots; no new library was installed.

## 8. Summary

**Closed:** 50° → 110° over 299.8 mm (−0.37° max from linear); torque always closing; cable
never slack (min 2.73 N); transmission angle ≥ 67.8°; R0 = 5.4 N; housing 148.6 mm wide; all
interference margins ≥ 14.4 mm; statics and virtual-work pin forces agree.

**Did not close:** the start geometry (travel 32.4 mm, mapping 13.6 % off, joint direction
reversed, 4-in-a-row carriage too wide, height ~190); R_load = 41.5 N vs the 40 N goal (goal TODO;
ΔS = 60 or a lighter spring would close it).

**Changed in params.py (architecture-D block, all tagged):** `D_ARM_MM` 70.0 → **70.3**
(VERIFIED), `D_LINK_MM` 60 → **70.0** (VERIFIED), `D_ARM_ANGLE_CLOSED_DEG` 0 → **25.0**
(VERIFIED); new ASSUMPTION/TODO entries for pulleys per row (2), arm/link widths, carriage
half-height, pivot bolt, anchor-spring tensions, transmission-angle limit. E and C values untouched.

---

# Addendum (rev 2 of this report) — 3D stack, grips vs housing, reeving lanes

Script: `check3d_D.py` (run `python3 check3d_D.py`; output in `out/check3d_D_run.txt`). Reads the
verified link geometry from `kinematics_D.py` and the new D_* values in `params.py`. Verified
geometry (arm 70.3 / link 70.0 / 25° / P 98.56) was **not** changed; proposals are marked.

Clearance model: plates = segment + half-width + end bosses, extruded over a z-range; pins =
cylinders along z. 3D clearance = plan distance if the z-ranges overlap, otherwise
√(max(plan, 0)² + z-gap²). Pins span the plates they join plus 3 mm overhang each side
(`D_PIN_OVERHANG_MM`, ASSUMPTION). Pins: pivot M8, joint M5, carriage M8 (ASSUMPTION). Hub R10
(= 625 bearing OD 16/2 + 2 mm wall, ASSUMPTION), joint boss R8, arm plate 16 × 8, link 12 × 6,
plate gap 0.3. Carriage positions checked: 0, 10, 20, 30, 37.48 mm.

## 9. Check 6 — 3D clearances, stack as reported vs re-stacked

**A) Stack as reported** (arm R 0–8, arm L 8.3–16.3, link R 16.6–22.6, link L 22.9–28.9):
the right joint pin must reach from arm R (z ≤ 8) to link R (z ≥ 16.6) and therefore passes
through arm L's plane; mirror for the left. True 3D clearance of that pair:

| pair | x = 0 | 10 | 20 | 30 | 37.48 | min |
|---|---|---|---|---|---|---|
| arm_R / joint_L (and arm_L / joint_R) | +1.7 | −0.0 | −4.1 | **−9.6** | −6.9 | **COLLISION** |
| all other non-joined pairs | ≥ 16.3 | | | | | OK |
| adjacent plates (design gap) | 0.3 | | | | | by design |

VERIFIED: the reported stack **collides** from ~10 mm of carriage travel onward, by up to 9.6 mm.

**B) Re-stacked** (`D_STACK`: arm R 0–8, link R 8.3–14.3, link L 14.6–20.6, arm L 20.9–28.9):
each joint pin now joins adjacent planes; the carriage pin passes through both links (adjacent).

| pair | min over the 5 positions | note |
|---|---|---|
| arm_R / joint_L, arm_L / joint_R | **3.6** | z gap between the pin overhang and the far arm |
| arm_R / arm_L | 12.9 | different planes, plan overlap allowed |
| arm_R / link_L, arm_L / link_R | 6.6 | |
| arm / opposite pivot bolt, link / opposite pivot bolt | 16.3 | plan distance, same as the 2D check |
| joint_R / joint_L | 23.9 | |
| carriage_pin / anything not joined | ≥ 52.8 | |
| adjacent plates | 0.3 | design gap |

VERIFIED PASS: no collision; the smallest real clearance is 3.6 mm (≥ `D_CLEARANCE_MIN_MM` = 2).
Both tables in full in `out/check3d_D_run.txt`.

**Joggle / spacer so both grips end in one plane.** Arm R plate centre z = 4.0, arm L centre
z = 24.9 → common grip plane z = **14.45** (centre of the 28.9 mm stack). Proposal: each pivot
hub is extended in z to the grip plane and the long handle arm (a separate printed plate, 8 mm,
centred at z = 14.45) bolts onto the hub face — a **10.45 mm hub extension on each side**
(`D_GRIP_SPACER_R_MM`, `D_GRIP_SPACER_L_MM`, VERIFIED arithmetic). A printed S-joggle in an 8 mm
PETG plate is the alternative, but a straight hub + bolted arm is stronger across layers and
lets the long arm be reprinted alone. The two hubs are at x = ±49.3, so their extended z-ranges
(R: 0 → 18.45, L: 10.45 → 28.9) cannot meet.

## 10. Check 7 — grip tubes and long arms vs hubs and housing (closed and open)

Tube Ø32 × 130 (CLAUDE.md), centred on the grip centre (L_LINE 71.1 at θ = 3° / 33°), tube axis
at 25° / 55° from vertical, upper end toward the centre line. Hand envelope: R35 around the tube
axis over the 96 mm hand breadth (`HAND_ENVELOPE_RADIUS_MM`, **ASSUMPTION — TODO measure a fist
on a Ø32 tube**). Housing bottom plate proposed at y = +15 above the pivot line (`D_HOUSING_BOTTOM_Y_MM`).

| case | hub | tube top (x, y) | tube → own hub | tube → other hub | **hand → own hub** | hand → housing bottom (+15) | tube → bottom |
|---|---|---|---|---|---|---|---|
| closed | R20 (boss as drawn) | (25.5, −12.1) | **−9.4** | 39.8 | **−22.9** | 7.5 | 11.1 |
| closed | R10 (minimum) | (25.5, −12.1) | 0.6 | 49.8 | **−12.9** | 7.5 | 11.1 |
| open | R20 | (34.8, −22.3) | **−9.4** | 51.0 | **−22.9** | 12.1 | 21.3 |
| open | R10 | (34.8, −22.3) | 0.6 | 61.0 | **−12.9** | 12.1 | 21.3 |

VERIFIED (under the assumptions): with the R20 boss the tube itself hits the hub by 9.4 mm; with
the smallest possible hub (R10) the tube just clears (0.6 mm) but the **hand envelope overlaps the
hub by 12.9 mm** at both ends of the stroke. The long arms never reach the housing if the housing
bottom is at y ≥ +15 (the hubs hang below it as ears); a housing extending below the pivot line
(as in §5, bottom at −34) would collide with the tube tops (−12 / −22) and the hands.

Why this is structural, not a layout slip: for a low-torque bell crank (θ_closed ≈ 0) the grip
depth below the pivot ≈ L_LINE ≈ ΔS/0.985, i.e. **the grips sit about one ΔS below the pivots**.
ΔS = 70 puts the hands 71 mm below the pivot axes, inside the pivot hubs' reach.

**Also found:** the arm/link joint is at y = −6.1 at full open — below any housing bottom that
clears the hands, i.e. an exposed moving joint next to the fingers.

Proposals (not applied — they change verified geometry; your call):

| Option | What | Effect | Price |
|---|---|---|---|
| P1 | Raise the arm elevation ψ0 so the joint stays inside the housing: ψ0 = 40°, c = 70 → a = 72.7 (re-solved) | joint y 46.8 → 12.6; fits a housing bottom at +10 | mapping error +1.7° (2.9 %), μ min 55°, R_load 37.8 N. ψ0 = 45° keeps the joint above +15 but the links then collide with the opposite hub (−2.9 mm) and the error is +2.5° |
| P2 | Increase ΔS to 90 (grips 91 mm deep; S 106 → 196) | tube → hub +13.5, hand → hub **+5.2** with R10 | T_max 9.96 N·m (+29 %), ideal cable tension 34.8 N, R_load ≈ 53 N (goal 40) |
| P2' | ΔS = 80 | hand → hub −4.1 (still touching) | T_max 8.85, R_load ≈ 47 N |
| P3 | Keep ΔS = 70, hub R10, and **test the hand clearance on a 1–2 h mock-up** (hub + tube segment) before changing anything | decides whether the R35 assumption is pessimistic | none if it passes; otherwise P2 |

My recommendation: P3 first (cheapest fact), P1 regardless (an exposed joint beside the hands
is not acceptable), then P2 only if the mock-up fails. The `R_load` goal is already marginal at
ΔS = 70 and would be clearly missed at ΔS = 90.

## 11. Reeving lanes — every part its own (x, z)

The 2 × 2 in-plane layout from the buildability note **cannot be reeved**: a fixed and a moving
pulley at the same x in the same plane have identical tangent lanes, so the cable leaving the
moving pulley returns to the fixed one on the lane it arrived on — a closed loop. Real blocks
avoid this with sheaves at different z, which gives every part a fleet (out-of-plane) angle of
Δz / part length ≈ 10 / 60 mm ≈ 9°, far beyond the usual ≤ 1.5° guidance (TODO source) for our
short parts.

Layout that works with **zero fleet angle** (ASSUMPTION, checked geometrically in `check3d_D.py`):
moving pulleys on the carriage with axles along **x** (pulley planes y-z), fixed pulleys with
axles along **z** (planes x-y). A part is then exactly vertical when its x equals a moving
pulley's x = x_fixed ± R_f and its z equals a fixed pulley's z = z_moving ± R_m. With all pulleys
Ø30, width 8 (`PULLEY_WIDTH_MM`, ASSUMPTION: 625 bearing + 1.5 mm flanges), carriage axle at
z = 40 (`D_CABLE_Z_CENTRE_MM`):

| part | from → to | x | z |
|---|---|---|---|
| 1 | anchor → M1 (x −45) | −45 | 25 |
| 2 | M1 → F1 (x −30, z 55) | −45 | 55 |
| 3 | F1 → M2 (x −15) | −15 | 55 |
| 4 | M2 → F2 (x 0, z 25) | −15 | 25 |
| 5 | F2 → M3 (x +15) | +15 | 25 |
| 6 | M3 → F3 (x +30, z 55) | +15 | 55 |
| 7 | F3 → M4 (x +45) | +45 | 55 |
| 8 | M4 → dead end (housing top) | +45 | 25 |

VERIFIED: 8 distinct lanes; no lane passes through any pulley body (moving bodies x ± 4 at
z 25–55; fixed bodies x ± 15 at z ± 4); fleet angle 0 at every pulley. Entry fairlead at
(−45, 25) and dead end at (+45, 25) on the housing top; the three fixed pulleys sit in one row
(y ≈ 50 in the §5 layout, i.e. **above the pivot line** — the housing no longer needs to extend
below it for the cable).

Consequences:
- **Carriage pulley block width = 3 × Ø_fixed + pulley width = 98 mm** (x −45 … +45 plus 8), not
  70. `D_CARRIAGE_WIDTH_MM` is set to this derived value with a note; 68 mm is reachable only with
  Ø20 fixed pulleys, which cannot carry a 625 bearing (OD 16) — bearing TODO for that option.
- **Depth:** the cable system spans z 21 … 59 (38 mm: fixed pulleys at z 25 and 55, ± 4). The
  arm/link stack is z 0 … 28.9. The fixed pulley at (x 0, z 25) shares z with arm L (20.9–28.9) but
  is above it in y (bottom at 35 vs arm top 31 — 4 mm, tight; raise the fixed row to y ≥ 55).
  Total = 59 + 2 × 3 mm walls = **65 mm, i.e. 5 mm over the 60 mm depth**. Ways back to 60: Ø26
  moving pulleys (z spacing −4), 6 mm arm plates (stack −4), or accept 65.
- No z-offset *within* a row is needed — the two fixed-pulley z-planes (25 and 55) are set by the
  moving pulley diameter, not by a row offset.

## 12. Summary of this addendum

**Closed:** re-stacked order arm R / link R / link L / arm L clears everything (min 3.6 mm);
spacer 10.45 mm per side brings both grips to z = 14.45; reeving: 8 vertical lanes, no pulley
intersections, zero fleet angle; `params.py` now carries bearing (625ZZ default), pin, boss,
stack, grip-plane, lane and carriage-width values, all tagged; architecture-E values marked
legacy; header fixed.

**Did not close:** (1) the reported stack collides (−9.6 mm) — replaced; (2) the 2 × 2 in-plane
reeving cannot be reeved — replaced by the crossed-axle layout; (3) carriage width is 98 mm, not
the 70 requested — explained above; (4) depth 65 mm vs 60; (5) **grip tube / hand envelope vs
pivot hub: −9.4 mm (R20) / hand −12.9 mm (R10)** under the R35 hand assumption; (6) the arm joint
is exposed below the pivot line at full open. Items 5–6 need your decision (§10 proposals); nothing
verified was changed.
