# autoRAD — Buildability review of E, C and D+anchor-spring

Date 2026-11-10. Companion to `docs/main_design.md` rev 3. Question answered here: **which
option is most likely to work as a printed prototype in ~3 weeks** for two students with one FDM
printer and hand tools — not which is best on paper.

Nothing here is validated. Print times, hours and material numbers are estimates with the
assumptions stated; expect ±50 % on time.

Common assumptions (ASSUMPTION unless marked):
- Print rate 15 cm³/h (0.4 nozzle, 0.2 mm layers, PETG). Fine features (threads, slots) at 0.12 mm: ~8 cm³/h.
- Housings as 3 mm shells with a 60 mm depth; split into body + top + cover for printing.
- Grips Ø32 × 130 hollow, long arms, stops, clamps and fairlead are the same for all three options
  (~14 h of printing, ~10 parts) and are not repeated in the comparison.
- Hand load 200 N/hand; PETG E ≈ 2 GPa, ν ≈ 0.4 — TODO datasheet; yield TODO (typically a few tens of MPa).
- Housing widths are the swept-envelope values: **E = 172 mm** (n = 8, Y0 = 80); **C ≈ 149 mm**
  and **D ≈ 149 mm** (arms never swing outboard of the pivot; width = P + 2 × 20 boss + 10). A
  common E/D platform would be 172 mm.

---

## 1. The three options as they would actually be built

**E — reeved cable (8 parts) + carriage + slotted bell-crank arms.** The 110° hold is a straight
dwell section in each slot; the roller pin on the carriage runs in both slots.

**C — threaded drum / barrel cam + runner + two links.** Drum Ø40, 2-start trapezoidal thread
(lead ~17 mm) plus a cable groove on the same shaft; the last turn of the groove has zero lead
(dwell). Return by clock spring, or by a cord on a second drum section pulled by an extension spring.

**D + anchor spring — reeved cable (8 parts) + carriage + two links + hard stop at 110°; the
reference cable's frame anchor is a preloaded spring.** Geometry: arm 70 mm pointing inward,
link 60 mm, carriage travel 37.6 mm for 30° (n = 8). Overtravel: when the carriage reaches its
stop at 300 mm the cable tension rises to the anchor-spring preload (≥ 50 N, must exceed the
40 N loaded tension) and then climbs with the spring rate over the remaining 100 mm of travel.
The user feels a **soft wall of ~50–70 N** beyond 300 mm instead of a free dwell.
Variant worth noting: the series spring could live *inside* the carriage (pulley block and link
yoke joined by a ~250 N preloaded die spring, 12.5 mm travel) — enclosed and machine-independent,
but a much stiffer spring and a two-piece carriage. Not costed below; TODO if the anchor is a problem.

---

## 2. Comparison

### 2.1 Printed parts and print time (mechanism-specific parts only; common parts add ~14 h)

| | E | C | D + anchor spring |
|---|---|---|---|
| Housing body + top + cover | 172 × 213 × 60 → ~24 h | 149 × 170 × 60 → ~18 h | 149 × 190 × 60 → ~19 h (172 wide if shared with E: ~22 h) |
| Drive parts | 7 pulleys (~2.5 h), carriage 1-row 4-pulley block (~4 h), 2 slotted arms 8 mm thick (~5.5 h each at fine layers → 11 h), roller spacer | drum with thread + groove (~8 h at fine layers), nut/runner (~3 h), 2 links (~1 h), 2 short arms (~3 h), cord drum / spring cover (~2 h) | 7 pulleys (~2.5 h), carriage (~4 h), 2 link arms (~3 h), 2 links (~1 h), anchor spring tube + plunger (~2 h) |
| Fixture for the decision test | slot/roller fixture (~2 h) | thread coupons ×3 (~3 h) | — |
| **Mechanism print time** | **~44 h** | **~38 h** | **~32 h** |
| **Printed parts (mechanism)** | 3 + 7 + 1 + 2 + 1 + 1 = 15 | 3 + 1 + 1 + 2 + 2 + 1 = 10 | 3 + 7 + 1 + 2 + 2 + 2 = 17 |
| Reprint cost of the risky part | slotted arm: 5.5 h each, both must match | drum: 8 h | link arm: 1.5 h; anchor tube: 1 h |

Total with common parts: E ~58 h, C ~52 h, D ~46 h. One printer running ~8 h/day: 6–7 days of
printing before any reprints. That is a third of the 3 weeks for every option, so **reprint cost
of the part most likely to fail** matters more than the total.

### 2.2 Bought parts

| | E | C | D + anchor spring |
|---|---|---|---|
| Bearings | 7 pulley bearings + 1 roller bearing (Ø10 OD) + 4 pivot bearings/bushings | 2 drum bearings + 1 thrust bearing (or washers) + 4 pivot | 7 pulley bearings + 4 pivot |
| Shafts/pins | 2 × M8 pivots, 2 × Ø5 guide rods (~150 mm), 1 × M4 roller pin, 4 pulley axles (M4/M5) | 1 × M8 drum axle, 4 × M4 link pins, 2 × M8 pivots | as E minus roller pin, plus 4 × M4 link pins |
| Springs | 1 extension spring: 30 N preload, ~0.1 N/mm, 50 mm travel — **hard to buy as specified**; realistic: stiffer (0.3–0.5 N/mm) with 20–30 N initial tension, R0 then ~9 N | clock spring (tape-measure salvage?) **or** extension spring + 1 mm cord | carriage spring as E **plus** anchor spring: ≥ 50 N preload, 100 mm travel — a long extension spring (~200 mm) or a bungee; preload set by a stop in the tube |
| Cable | 1.5 mm × ~2.5 m (8 parts) + 4 crimps/clamps | 1.5 mm × ~1 m + 2 crimps | as E |
| Other | M3/M4 screws (~25), strap, carabiner | screws (~15), strap, carabiner | screws (~25), strap, carabiner |
| **Line items** | ~11 | ~9 | ~12 |
| **Hardest to source** | 30 N / 0.1 N/mm spring | clock spring (if used) | anchor spring with 100 mm travel |

### 2.3 Hardest tolerances

| | E | C | D + anchor spring |
|---|---|---|---|
| 1 | Slot width: 10.3 mm nominal; FDM slots print 0.2–0.4 mm narrow → test coupon needed; both arms must match within ~0.2 mm or the handles are asymmetric | Printed 2-start trapezoidal thread, 17 mm lead: fit and surface decide friction (η 0.4–0.6); needs 2–3 coupons | Link pin holes (Ø4 + 0.3): easy; links are tolerance-forgiving |
| 2 | Two guide rods parallel over 100 mm, carriage bushes 0.3 mm: binding if off by > 0.2 mm/100 mm | Drum axial play (thrust face) ≤ 0.3 mm or the runner position wanders → handle angle error ~0.5° per 0.3 mm | Same guide-rod issue as E |
| 3 | Roller pin perpendicular to the carriage (both slots share it) | Cable groove pitch 2 mm, must not let the cable cross-wind | Anchor spring preload stop: must be > 40 N and repeatable |
| 4 | 4 pulleys on one axle: axle straightness, pulley spacing vs fixed pulleys (cable runs must be vertical) | Dwell transition in the groove (zero lead) must be smooth or the runner clicks | Same 4-pulley axle issue as E |

### 2.4 Likely failure points (with the arithmetic that worries me)

**E**
- Slot wall contact: Ø10 steel roller on an 8 mm PETG wall at 157 N is a line contact.
  Hertz estimate p₀ = √(F·E′/(π·L·R)) = √(157 × 2381/(π × 8 × 5)) ≈ **55 MPa** — at or above
  typical PETG yield (TODO datasheet). Expect local denting and creep where the user pauses at the
  loaded end. A 12 mm wall and a Ø16 roller bring it to ~35 MPa; still marginal.
- Pressure angle 46–55°: stick-slip if the roller bearing is dry or the slot is tight.
- Carriage binding on printed bushes; cable jumping a pulley at 2.7 N up-stroke tension.
- Roller pin M4 at 218 N: ~100 MPa bending — fine for steel, but the printed pin seats see
  218/(4 × 8) ≈ 7 MPa bearing, OK.

**C**
- Thread friction unknown until printed: R0 modelled ~10 N, could be 15–20 N; R_load ~60 N.
- Thread/nut wear under 190 N axial (spread over several turns → low stress, but PETG-on-PETG dry).
- Drum thrust face: 190 N on a printed face without a thrust bearing → creep; buy the bearing.
- Clock spring: salvage springs are unknown stiffness; cord-on-drum return adds a cord that can tangle.
- Cable jumping the groove when slack (single-layer groove, no flanges between turns).

**D + anchor spring**
- Mapping lag: if the real hand torque makes the cable tension exceed the anchor preload, the
  anchor spring starts extending *during* the normal stroke and the handles lag. Fix is a higher
  preload, which raises the soft-wall force. Everything hangs on the unknown F_HAND (Q5).
- Loads during overtravel: 8 parts × 50–70 N = **400–560 N** on the carriage stop and on the
  housing top plate (fixed pulley axles + dead end). A 10 mm PETG top plate: M ≈ 560 × 0.04/4 =
  5.6 N·m, σ ≈ 6M/(bh²) = 5.6 MPa — fine if designed for it; a 5 mm plate would not be.
- Links: 110–125 N each; M4 pins in printed holes, bearing stress ~3.4 MPa — negligible. This is
  the most forgiving drive of the three.
- Anchor hardware is machine-specific and external: strap slips, spring tube knocks the frame,
  the gym may object.

### 2.5 Teammate-hours to assemble and tune (two students; excludes printing and CAD)

| | E | C | D + anchor spring |
|---|---|---|---|
| Assembly | reeving 8 parts + tensioning 3–4 h; carriage/rods 2–3 h; arms/pivots 2 h → **8–9 h** | drum/nut/bearings 2 h; links/arms 2 h; cable + return 2 h → **6 h** | reeving 3–4 h; carriage/rods 2–3 h; arms/links 1 h; anchor tube 1 h → **8 h** |
| Tuning | slot coupon + 1–2 arm reprints (each 11 h of printing, 1 h of work); carriage alignment; spring → **8–12 h** plus printer time | thread coupons, drum fit, groove, return spring → **8–12 h** plus 8 h drum reprints | spring preload on the machine 3–4 h; link lengths 1 h; carriage alignment 2 h → **6–8 h** |
| **Total** | **16–21 h** | **14–18 h** | **14–16 h** |
| Serial bottleneck | arm reprints (11 h each pair) | drum reprints (8 h each) | none over 2 h |

### 2.6 What could go wrong in testing

| | E | C | D + anchor spring |
|---|---|---|---|
| Bench, no load | R0 above 10 N from pulley + roller friction; carriage sticks at one end | R0 above 10 N from the thread; return spring too weak to rewind through the thread | R0 ~5–9 N; soft wall at 300 mm feels like a stop (it is) |
| On machine, a few kg | slot clicks at the easing/dwell transition; asymmetry from slot mismatch | runner clicks at the groove dwell; cable crosses in the groove | anchor spring extends early if preload set low; strap moves |
| On machine, design load | **slot wall dents under sustained load; roller marks the wall** | thread friction rises with load; R_load felt as ~60 N | **if F_HAND > assumed, handles lag during the stroke**; top plate deflects |
| Graceful degradation? | No — a damaged slot changes the mapping | Partly — a worn thread gets stiffer but keeps working | **Yes — without the anchor spring it still works for strokes ≤ 300 mm** (the cable simply goes taut at the stop; acceptable at a few kg on the bench) |

---

## 3. Recommendation: most likely to work

**Build D + anchor spring, on a platform that can also take E's arms.**

Why D:
- Its drive (links, pins) has the lowest stresses of the three by an order of magnitude and is
  insensitive to print tolerance. Nothing in it needs a coupon.
- Its risky element (the anchor spring) is cheap to iterate (1–2 h printed parts, swap a spring)
  and can be **left off for the first tests**: strokes ≤ 300 mm work without it.
- Shortest serial path: no 8–11 h reprint loop between tuning iterations.
- Same reeving and carriage as E. Print the housing at 172 mm, and the E slotted arms become a
  1-day experiment on a working machine instead of the thing the whole prototype depends on.

Why not E first: the slot contact stress (~55 MPa line contact) and the 46–55° pressure angle
put the one part that defines the mapping at risk of creep and wear, and each iteration costs
11 h of printing. E is the better *design*; it is not the better *first prototype*.

Why not C: fewest parts and the smallest housing, but every uncertainty sits in one 8-hour
printed part (the drum), its friction is unknown until printed, R_load ≈ 60 N misses the goal,
and the rotary return needs a spring you cannot buy to a number.

**What we lose by choosing D:**
1. The hard dwell. Beyond 300 mm the user meets a soft wall of ~50 N rising to ~70 N (a step from
   5.6 N to 50 N when unloaded, ~40 N to 50 N when loaded). Requirement Q3 is met only in the sense
   that the handles stay at 110°; the "free" overtravel is gone.
2. The mapping is only linear-ish (link kinematics, a few % deviation) and cannot be shaped by a
   parameter — the cam-profile story and the easing zone go.
3. 400–560 N on the carriage stop and housing top during overtravel; the top plate must be
   designed for it (≥ 10 mm, in-plane layers).
4. An external, machine-specific part at the anchor that needs setting up per machine and can be
   seen and bumped.
5. Dependence on F_HAND: the preload must exceed the real loaded tension; if the gym loads are
   higher than 200 N/hand the handles lag until the preload is raised.
6. If the housing is shared with E it stays at 172 mm; a D-only housing would be ~149 mm.

Decision tests, in order: (1) bench R0 with the links (target ≤ 10 N); (2) on-machine mapping at
a few kg with the anchor spring; (3) raise the load until either the handles lag (preload too
low) or the top plate flexes; (4) if time remains, fit the E arms and run the slot test.

*Stop. Waiting for review.*

---

## Correction note (2026-11-10, after `kinematics_D.py`)

Six values in §1–2 for option D did not close when the link mechanism was modelled exactly; see
`docs/kinematics_D_report.md` §6. In short: the inward arm joint moves **down** on opening (not
up), so the carriage moves down on cable payout; arm 70 / link 60 / horizontal arm gives 32.4 mm of
travel (stop at 259 mm) and a 13.6 % non-linear mapping — corrected to arm 70.3 / link 70 / 25°
elevation (37.5 mm, 0.6 %); 4 carriage pulleys in a row (150 mm) do not fit a 149 mm housing —
2 rows of 2; housing height ~235 mm, not ~190; R_load 41.5 N, not ~40. The text above is left as
written so the correction is traceable.
