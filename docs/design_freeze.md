# Architecture D — design freeze run (2026-11-10)

**Status: NOT FROZEN.** Three stop conditions failed (Section 1). Everything else passed. Values
below are from `kinematics_D.py` and `check3d_D.py` with `params.py` as applied in this run
(arm elevation 40°, link 70, arm re-solved to **72.75 mm**, hub R10, ΔS 70). Run outputs:
`out/kinematics_D_run.txt`, `out/check3d_D_run.txt`; plots `docs/img/D_*.png`.
Tags: VERIFIED = computed by the named script; ASSUMPTION = my choice; TODO = unknown.

## 1. Stop conditions

| Condition | Limit | Result | Status | Proposed fix |
|---|---|---|---|---|
| Mapping error | ≤ 2.5 % of 60° | **+1.72° = 2.86 %** at s = 110 mm | **FAIL** | ψ0 = 38°: a = 72.05, error +1.43° = 2.39 %, joint at open y = 10.0 (needs the front-bay guard at y ≥ +12, see 5.3). ψ0 = 37°: 2.17 %, joint 8.7. Table in 2.2. |
| Any 3D clearance | ≥ 2 mm | min real clearance 3.6 mm (0.3 mm plate gaps are by design) | PASS | — |
| Moving/fixed pulley gap at the stop | ≥ 5 mm | 5.0 mm (fixed row placed from this limit) | PASS | — |
| Carriage + rods + spring fit the width | inner 143 | rods: 130 mm PASS; **spring beside the carriage: 154 mm FAIL** | **FAIL** | Spring **above** the carriage at x = 0, z = 43, in the 22 mm gap between M2 and M3: 130 mm, PASS. |
| Net cable force | 8 T down | 8 parts, each carriage ↔ fixed end, all pull down | PASS | — |
| Depth (asked 65–70) | — | **72.0 mm** with 8 mm pulleys and 3 mm plates | out of range | 7 mm pulleys (1 mm flanges) → 70.0; or accept 72. |

## 2. Kinematics (`kinematics_D.py`, VERIFIED)

### 2.1 Geometry as applied
| Item | Value | Tag |
|---|---|---|
| Arm a | 72.75 mm (re-solved for 37.50 mm travel) | VERIFIED |
| Link c | 70.0 mm | reviewer |
| Arm elevation at closed ψ0 | 40° (→ +10° at open; joint y 46.8 → 12.6) | reviewer |
| Pivot spacing P | 98.56 mm; L_LINE 71.1; θ 3° → 33° | VERIFIED |
| Carriage travel / stop | 37.50 mm / s = 300.0 mm | VERIFIED |
| Carriage pin y | 116.5 (closed) → 79.0 (stop) | VERIFIED |
| Transmission angle | min 55.3° at closed, 81.4° at open (limit 30) | VERIFIED PASS |

### 2.2 Mapping (5 mm steps of carriage travel) and the ψ0 trade
| carriage mm | 0 | 5 | 10 | 15 | 20 | 25 | 30 | 35 | 37.5 |
|---|---|---|---|---|---|---|---|---|---|
| included ° | 50.0 | 59.0 | 67.7 | 76.0 | 84.0 | 91.7 | 99.1 | 106.3 | 110.0 |
| linear ° | 50.0 | 58.0 | 66.0 | 74.0 | 82.0 | 90.0 | 98.0 | 106.0 | 110.0 |

(values from `out/kinematics_D_run.txt` section 1; max +1.72° at s = 110 mm)

| ψ0 | a | max error | % | joint y at open | R_load | μ min |
|---|---|---|---|---|---|---|
| 34 | 71.01 | +0.92 | 1.53 | 5.0 | 39.4 | 63.9 |
| 36 | 71.48 | +1.17 | 1.95 | 7.5 | 38.9 | 61.0 |
| 37 | 71.75 | +1.30 | 2.17 | 8.7 | 38.6 | 59.6 |
| **38** | **72.05** | **+1.43** | **2.39** | **10.0** | **38.4** | **58.1** |
| 39 | 72.39 | +1.57 | 2.62 | 11.3 | 38.1 | 56.7 |
| 40 (applied) | 72.75 | +1.72 | 2.86 | 12.6 | 37.8 | 55.3 |

### 2.3 Loads (η = 0.727, spring 30 N + 0.1 N/mm, friction (1−η)(spring+|pin|), F = 200 N/hand)
| | value |
|---|---|
| Torque | always closing, +0.74 → +7.74 N·m per handle; no sign change at F = 0/100/200 |
| **R0 / R_load** | **5.37 N / 37.83 N** (goals ≤ 10 / ≤ 40) — both PASS |
| Min cable tension (no-load up stroke) | 2.73 N (≥ 2) |
| Peak link force / carriage force | 119 N / 227 N (unchanged by ψ0 within 1 %) |
| Carriage stop load (v2 anchor spring 50 / 70 N) | 366 / 526 N at F = 0 (worst case), 162 / 322 N at F = 200 |

## 3. 3D clearances (`check3d_D.py`, stack arm R 0–8 / link R 8.3–14.3 / link L 14.6–20.6 / arm L 20.9–28.9)
Carriage travel 0, 10, 20, 30, 37.5 mm; pins pivot M8, joint M5, carriage M8; hub R10, joint boss R8.

| pair | min clearance over the 5 positions |
|---|---|
| arm_R / joint_L and arm_L / joint_R (pin overhang to far arm, z) | 3.6 |
| arm_R / arm_L | 12.9 |
| arm / opposite pivot bolt, link / opposite pivot bolt | ≥ 17.7 (plan) |
| joint_R / joint_L | ≥ 23 |
| carriage pin / anything not joined | ≥ 48 |
| adjacent plates | 0.3 (design gap) |

VERIFIED PASS (full table in `out/check3d_D_run.txt`). The reported stack (rev 1) would still
collide (−2.7 mm with the R10 hub) and is dropped.

## 4. Reeving (`check3d_D.py`, VERIFIED geometry; layout ASSUMPTION)
Moving pulleys M1–M4 Ø30 on the carriage axle **along x** at x = −45, −15, +15, +45, axle z = 43.
Fixed pulleys Ø30 with axles **along z** in one row at y = 84: F1 (−30, z 58), F2 (0, z 28),
F3 (+30, z 58). Fairlead pulley F0 Ø20 (axle along x) at (x −45, z 18) and the dead end at
(x +45, z 28) are **at the fixed-pulley end**; the anchor cable enters through the top plate at
(x −45, z 8) and runs down in front of the stack to F0.

| part | between | x | z | force on carriage |
|---|---|---|---|---|
| 1 | F0 → M1 | −45 | 28 | T down |
| 2 | M1 → F1 | −45 | 58 | T down |
| 3 | F1 → M2 | −15 | 58 | T down |
| 4 | M2 → F2 | −15 | 28 | T down |
| 5 | F2 → M3 | +15 | 28 | T down |
| 6 | M3 → F3 | +15 | 58 | T down |
| 7 | F3 → M4 | +45 | 58 | T down |
| 8 | M4 → dead end | +45 | 28 | T down |

Net **8 T down**; 8 distinct lanes; no lane through a pulley body; zero fleet angle at every pulley.
Report section 11 (rev 2 addendum) said "dead end on the housing top" — **corrected here**: fairlead
pulley and dead end are at the fixed end, below the carriage.

### y-layout
| item | y [mm] above the pivot line |
|---|---|
| carriage pin | 116.5 (closed) → 79.0 (stop) |
| moving axle (= pin + 40, `D_MOVING_AXLE_ABOVE_PIN_MM`, ASSUMPTION) | 156.5 → 119.0 |
| fixed row axle (F0–F3, dead end) | 84.0 |
| moving/fixed body gap at the stop | **5.0** (= limit) |
| F2 (x 0, z 24–32) vs arm L (z 20.9–28.9) | 14.8 mm 3D; vs links z-gap 3.4; vs carriage pin (1 mm back overhang) z-gap 2.4 |
| F0 vs hub L | share z, but 59 mm apart in y (F0 at y 84, hub at y 0): clear |

## 5. Housing
### 5.1 Width
Inner needed = block 98 + 2 × (2 gap + 12 rod bush) + 2 × 2 wall clearance = **130 mm**, rods at
x = ±57 (z = 43). Spring beside the carriage would need 154 → **above the carriage at x = 0,
z = 43** (22 mm gap between M2 and M3) instead. Available inner 143 (149 outer, 3 mm walls):
13 mm spare. **Final width: 149 mm** (unchanged; could drop to 136).

### 5.2 Depth
Stack z 0–28.9 (front clearance 2 → inner from −2); cable system z 24–62 (F1/F3 bodies to 62);
anchor lane z 8. Inner −2 … 64 → **72.0 mm outer with 3 mm plates**. 70.0 with 7 mm pulleys.

### 5.3 Height
Top plate outer y = 191.5 (moving pulley top at closed 171.5 + 10 clearance + 10 plate).
Cable-bay bottom plate: inner 59, outer 49 (covers z 24–64 only). Front bay (z −2 … 24,
arms and links) needs a guard down to y = +15 (hands, `D_HOUSING_BOTTOM_Y_MM`) with the joint at
12.6 — **2.4 mm below the guard: another reason to take ψ0 = 38 (joint 10.0) and the guard at +12**.
Hub ears to y = −10. **Overall: ≈ 200 mm from hub bottom to top plate**, grips below that.

## 6. Part list (quantities, key dimensions; print all PETG unless noted)
| # | part | qty | key dimensions | tag / TODO |
|---|---|---|---|---|
| 1 | drive arm R / L (bell-crank plate with hub and joint boss) | 1 + 1 (mirror) | a 72.75 (or 72.05), ψ0 40° (or 38°), 16 × 8, hub R10 bore for 625 bearing | VERIFIED geometry; **ψ0 open** |
| 2 | hub extension / spacer | 2 | 10.45 mm to z = 14.45, R10 | VERIFIED |
| 3 | long handle arm R / L (bolted to the hub face) | 1 + 1 | L_LINE 71.1 at θ 3°, 8 mm plate; grip tube seat | shape TODO |
| 4 | link | 2 | 70.0 c-c, 12 × 6, M5 holes | VERIFIED |
| 5 | carriage (pulley block + pin support from the front + rod bushes + spring hook) | 1 | block 98 wide, axle 40 above the pin, bushes at x ±57, pin M8 held at z 0–8 | ASSUMPTION layout |
| 6 | moving pulley Ø30 × 8 (625 bearing) | 4 | lanes x ±15, ±45 | bearing TODO confirm |
| 7 | fixed pulley Ø30 × 8 | 3 | at y 84: (−30, z 58), (0, z 28), (+30, z 58) | |
| 8 | fairlead pulley Ø20 × 8 | 1 | (x −45, z 18), y 84 | **bearing OD ≤ 12 TODO** |
| 9 | housing top plate (eye, fairlead hole at (−45, 8), rod seats, spring anchor) | 1 | 149 × 72 × 10 | eye: bolt or steel eye TODO |
| 10 | housing bottom plate, cable bay | 1 | 149 × 40 × 10, fixed-pulley axles, dead end | |
| 11 | front-bay guard | 1 | down to y +15 (or +12), cut-outs for hubs | |
| 12 | side plates / standoffs | 2 + n | height ≈ 200, depth 72 | standoff count TODO |
| 13 | carriage stops (closed, open) | 2 | at carriage y 116.5 / 79.0 | |
| 14 | cable dead-end clamp | 1 | at (+45, 28), y 84 | |
| 15 | grip tube Ø32 × 130 | 2 | bought | |
| 16 | anchor spring tube + plunger (v2) | 1 + 1 | preload ≥ 50 N | v2 only; rate TODO |
| Bought | 625ZZ ×7 (+1 small for F0), M8 pivots ×2, M5 joint pins ×2, M8 carriage pin, Ø5 rods ×2 (~150), extension spring 30 N + 0.1 N/mm, 1.5 mm cable ~3 m, M3/M4 screws, carabiner, strap | | | spring and bearings TODO |

## 7. Remaining TODOs (blocking before parts)
1. **Decide ψ0: 38° (2.39 %, joint 10.0, guard at +12) vs 40° as applied (2.86 %).**
2. Hand envelope: R35 is an assumption; hand → hub still **−12.9 mm** at ΔS = 70 (unchanged by this run). Mock-up test (hub + tube) or ΔS → 90 (report §10). Not a stop condition here, but unresolved.
3. Pulley width 7 vs 8 (depth 70 vs 72). F0 bearing (OD ≤ 12). Spring actual rate/preload. 625ZZ confirmation.
4. Carriage pin support from the front only (cantilever 20 mm, ~90 MPa on M8): confirm or add a rear bracket at z > 32 with a cut-out for F2.
5. F_HAND (200 N) and machine data (Q5, Q6) still unknown.
