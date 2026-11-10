# autoRAD — Main concept design study (travel-actuated opening V-bar)

Revision 2 (2026-11-10). Changes from rev 1: corrections from review (Part A), reviewer's answers
to the open questions folded in (Part B), new Section 6 on the carriage force balance and the
torque sign-change problem (Part C), geometry changed from symmetric ±15° to a +3° → +33° offset.

Status: **design study, nothing validated.** Every number below is a first-pass hand calculation
or a `kinematics.py` output from the preliminary requirements. TODO = unknown. ASSUMPTION = my
choice; challenge it.

Notation: `s` = attachment travel [mm], `β` = handle rotation from closed [deg], `θ` = angle of
the pivot→grip-centre line from vertical [deg, outward +], `T` = torque on one handle [N·m],
`F` = vertical hand force per hand [N], `P` = pivot spacing [mm], `ΔS` = change in grip-centre
separation [mm].

---

## 1. Problem restatement, assumptions, open questions

### 1.1 Problem in one paragraph
A removable attachment for a cable machine. It hangs from the existing carabiner. Two grip
handles form a V. As the user pushes the attachment down, the handles must open symmetrically
from 50° to 110° included angle over the first 300 mm of travel, then hold 110° (dwell) up to
400 mm. On the way up they must close again. The motion must be driven only by the attachment's
travel relative to the fixed frame, with no electronics and no manual adjustment during use.

### 1.2 Three findings that shape the design (read these first)

**Finding 1 — the hand load dominates, not the gear ratio.**
In the geometry as first specified (both handles pivoting at the apex, grip centre 127 mm from
the pivot, handles hanging down-and-out), a vertical push `F` on a grip creates a torque about
the pivot of `T = F · L · sin β`. With an ASSUMED 200 N per hand:

| β | T per handle | T both handles |
|---|---|---|
| 25° (closed) | 10.7 N·m | 21.5 N·m |
| 40° (mid) | 16.3 N·m | 32.6 N·m |
| 55° (open) | 20.8 N·m | 41.6 N·m |

This torque tries to **close** the handles. Whatever transmission we choose, the reference cable
must react it. Over the mapping stroke the cable pays out 300 mm while each handle turns
0.5236 rad, so

    k = 300 mm / 0.5236 rad = 573 mm of cable per radian of handle rotation
    cable tension (ideal) = T_both / 0.573 m      → 37 N closed … 73 N open

The ≤ 10 N actuation goal cannot be met by any transmission while this torque exists; only the
**handle geometry** (Section 5.2) can reduce it. **Resolved for v1:** bell-crank handles with the
grip hanging almost directly under its pivot at the closed end (θ = +3° → +33°). This cuts the
worst torque from 20.8 to 7.7 N·m per handle (ΔS = 70) and keeps it always closing.

**Finding 2 — the "hold 110° beyond 300 mm" requirement needs a dwell, not a hard stop.**
If the handles hit a stop while the attachment keeps moving down, the carriage (or spool) stops,
the reference cable can no longer pay out, and it goes taut. The main cable still carries the
stack; the reference cable now takes **the user's extra push against a rigidly blocked
mechanism**, as a jolt into the anchor, the housing and the user's wrists. **Resolved for v1:**
keep the dwell (Q3), implemented as a straight section of the slot cam (Section 5.3).

**Finding 3 — the hand torque must never reverse sign (found by the reviewer).**
The rev-1 geometry (θ = −15° → +15°) made the hand load *open* the handles in the first half
of the stroke. The carriage is held only by the cable (tension only) and the spring, so an
opening load drives the carriage ahead of the cable, the cable goes slack and the handles open
under load before the user moves. Section 6 shows the numbers (42 N up against a 20 N spring at
the closed end) and the fix: offset the geometry so θ ≥ +3° throughout. The torque is then always
closing, the cable is always in tension, and the handles are biased closed if the cable is ever
released (Q10).

### 1.3 Assumptions (ASSUMPTION = mine; change in `params.py`)
| # | Assumption | Why it matters |
|---|---|---|
| A1 | Hand force per hand `F_HAND_N` = 200 N, vertical, equal on both hands. | Sets every internal load. TODO measure (Q5). |
| A2 | Reference cable anchored to the frame **above** the attachment, hanging parallel to the main cable, payout = travel (1:1). | Cable tension adds to the felt load; anchor offset error is small (Section 7.6). |
| A3 | Friction: overall mechanical efficiency `ETA_MECH` = 0.8 for carriage + pulleys + roller. Modelled as a friction force = (1 − η) × (spring + \|pin force\|) opposing carriage motion. | Rough. Prototype test 9 measures the real R0. |
| A4 | 1.5 mm steel cable; pulley diameter 30 mm (≈ 20× cable diameter, common wire-rope guidance, not a standard). TODO confirm. | Pulley size drives housing size. |
| A5 | Handle swing plane = frontal plane; vertical hand force produces the torques above. If the V plane is tilted by τ, torque scales by cos τ. TODO confirm with the ergonomic study. | |
| A6 | Linear angle-vs-travel map for v1 (Q4). `PROFILE` in `params.py` is the single switch for other profiles. | |
| A7 | Pressure-angle limit 55° for a ball-bearing roller in a printed slot. | Not a standard; prototype test 2 checks wear. |
| A8 | Housing width proxy = P + 2 × 20 mm pivot boss. | Real width comes from the CAD layout. |

### 1.4 Open questions — status after review
| # | Question | Answer / status |
|---|---|---|
| Q1 | May the pivots move apart (bell crank)? | **Yes.** Width target relaxed to 150–170 mm. |
| Q2 | How much should the hands separate? | **ΔS = 70 mm baseline** (106 → 176 mm), parameter; 60/70/80 shown. |
| Q3 | Is the dwell required? | **Yes.** Full open at 300 mm, dwell to 400 mm, both parameters. |
| Q4 | Linear mapping? | **Linear for v1**; profile is one parameter. |
| Q5 | Stack loads? | **Unknown.** Keep F_HAND = 200 N as the design assumption. Prototype tests start at a few kg. TODO. |
| Q6 | Which machine (pulley height, carabiner gate, cable, anchor options)? | **Unknown, TODO.** Anchor bracket is the last part to design. |
| Q7 | Actuation-resistance definition | **Accepted:** R0 ≤ 10 N no-load; R_load ≤ ~10 % of stack load (TODO confirm). |
| Q8 | Swivel? | **None in the housing for v1**; add at the anchor if twist shows in testing. |
| Q9 | Spring type? | **Extension spring is fine.** |
| Q10 | Failure bias if the cable releases? | **Biased closed, no pinch points.** The +3° offset does this. |

New questions raised by this revision:
- Q11 The R_load goal (10 % of a 400 N stack = 40 N) is hit almost exactly at ΔS = 70 (40.7 N). Is ΔS = 60 (36 N) acceptable, or is the goal soft?
- Q12 Housing height: with N_REEVE = 4 and the pin starting 150 mm above the pivots, the carriage pin reaches 250 mm above the pivots at 400 mm travel. Is a ~300 mm tall housing acceptable? N_REEVE = 6 saves ~35 mm at the cost of two more pulleys (Section 5.1).

### 1.5 Definition of "actuation resistance" (accepted, Q7)
`R` = tension in the reference cable during the down stroke = extra force the user must push
beyond the stack.
- **R0 (no-load)**: bench measurement, spring balance on the reference cable, no hand load.
  Goal **R0 ≤ 10 N** over the full stroke.
- **R_load**: with the design hand load. Goal ≤ 10 % of the stack load (TODO confirm).

---

## 2. Architectures

All five share: housing hangs from the carabiner, reference cable enters at the top, handles
exit at the bottom through guarded slots, closed and open end stops, return spring.
"Ratio" below is the required spool-to-handle reduction `i = k / r_spool` (k = 573 mm/rad), or
the equivalent for non-rotary stages.

### A. Spool + compound spur-gear train (the baseline)

```
 cable ──► [spool r] ── shaft ── pinion 1 ─► gear 1 / pinion 2 ─► gear 2 / pinion 3 ─► sector gear (handle R)
                                                                                   └─► meshes with sector gear (handle L)
 clock spring on the spool shaft rewinds.
```
- Equations: `i = 573 / r_spool`; r = 12 → 47.7, r = 24 → 23.9, r = 30 → 19.1. 2 stages for r ≥ 18 mm, 3 stages for r ≤ 15 (at a 6:1 per-stage placeholder).
- Symmetry: final sector gears on the two handle pivots mesh with each other (1:1 mirror).
- Part count: spool, clock spring, 2–3 compound gears, 2 sectors, 3–4 shafts, 6–8 bearings/bushings, housing (≈ 15 parts + fasteners).
- Printability: good; cq_gears makes the gears. Output-stage teeth carry the full hand torque: with m = 1 and T = 5 N·m a simple Lewis estimate gives ~70 MPa root stress (too high for PLA/PETG, TODO material data); m = 2–3 is needed at the output.
- Friction/backlash: 2–3 meshes ≈ 0.85–0.9 overall. Backlash is taken up in one direction because the spring and the hand torque load the train the same way.
- Overtravel: needs a slip clutch at the spool or a lost-motion stage (extra parts).
- Failure modes: output-tooth breakage under hand load; clock spring fatigue; shaft bosses wearing oval.
- Verdict: works, well understood, but heaviest option and leaves Finding 2 unsolved.

### B. Spool + worm → worm wheel on a handle

```
 cable ──► [spool] ── same shaft ── worm ─► worm wheel on handle R ─► sector gears sync handle L
```
- Equations: single stage, `i = z_wheel` for a 1-start worm: 20–48 teeth. Efficiency
  `η = tan λ / tan(λ + φ)`, φ = atan µ = 11.3° for µ = 0.2. A printable 1-start worm has
  λ ≈ 4–6° → **η ≈ 0.26–0.34**, self-locking (λ < φ).
- Self-locking does not help: during the down stroke the cable must drive the wheel **against**
  the hand torque through that 26–34 % efficiency, so R_load ≈ 3× the ideal value. The spring must also rewind through it.
- Symmetry: sector gears between handles.
- Part count: low (spool, worm, wheel, 2 sectors, spring, 4 bearings).
- Printability: printed worms work at module ≥ 1.5 but wear fast dry.
- Overtravel: needs a slip clutch (same as A).
- Verdict: **not worth it.**

### C. Threaded drum (lead-screw / barrel cam) + runner + two links ("umbrella")

```
 cable ──► [drum: cable groove + helical thread on one shaft] ── nut/runner slides axially
           runner ── link ─► handle R (short arm)
                  └─ link ─► handle L (short arm)        clock spring on drum rewinds
```
- Equations: drum turns `N = 300 / (2π r)`; runner travel `x = N · lead`. For 40 mm runner
  travel: r = 15 → 3.18 turns, lead 12.6 mm; r = 25 → 1.91 turns, lead 20.9 mm. Lead angle on a
  20 mm thread 11°–18° → **η ≈ 0.48–0.58** (µ = 0.2). Equal arm/link length `a`: runner position
  `y = 2a cos β`, `dy/dβ = 2a sin β` (non-linear).
- Overtravel: cut as a **barrel-cam groove**, the last turn(s) can be circumferential = dwell.
- Symmetry: inherent — one runner on the centre line, two mirrored links.
- Loads: runner axial force `= T_both / (dy/dβ)`; at β = 55°, a = 50 mm: ≈ 500 N on a printed thread (original geometry), ≈ 190 N with the Section 5.2 geometry. Drum needs a thrust bearing.
- Part count: drum, nut/runner, 2 links, clock spring, 2 radial + 1 thrust bearing, 4 link pins, housing (≈ 12).
- Printability: 2-start trapezoidal thread, lead 12–20 mm, prints well vertically.
- Friction/backlash: thread backlash ~0.3 mm → < 1° at the handle; friction is the weakness.
- Failure modes: thread wear/stripping, clock spring sourcing, cable jumping the groove if slack.
- Verdict: simple and compact, solves the dwell, but pays ~2× in actuation force.

### D. Reeved cable (block) + sliding carriage + two links — no rotation at all

```
 frame anchor
     │
     ▼ cable enters housing top
  ┌──┴──────────────┐
  │ fixed pulley ●   │  ◄── dead end
  │   ╲   ╱  ╲   ╱   │   n = 4 cable parts between fixed pulleys and carriage
  │    ● ──── ●      │  carriage (2 pulleys) moves UP as cable pays out
  │    │ carriage │  │   extension spring pulls carriage DOWN (stores cable)
  │   ╱         ╲    │
  │ link         link│  links to the handles' short arms (bell cranks)
  ├──○───────────○───┤  handle pivots
  │ │ handle     │   │
```
- Equations: carriage travel `x = s / n`. n = 4 → 75 mm (100 mm at 400 mm); n = 3 → 100 mm; n = 6 → 50 mm. Carriage force `= n · T_cable`.
- Symmetry: inherent (central carriage, mirrored links).
- Part count: 3 pulleys + 3 bearings (n = 4), carriage, 2 guide rods, 2 links, 4 pins, extension spring, dead-end clamp, housing (≈ 14).
- Friction/backlash: pulleys on ball bearings ≈ 0.97 each, link pins ≈ 0.97, carriage guide 0.9–0.95 → **η ≈ 0.8–0.85**. Zero backlash.
- Overtravel: **no natural dwell.** A free slot (lost motion) does not hold the handles against a closing hand torque. Needs a slip or a changed requirement.
- Failure modes: cable jumping a pulley when slack, carriage binding on printed rails, spring fatigue.
- Verdict: lowest force and simplest loads, but no dwell. Fixed by E.

### E. Reeved cable + carriage + **slotted bell-crank handles** (D with a built-in cam dwell) — selected

```
  same reeving and carriage as D, but the carriage carries ONE transverse pin
  (M4 bolt + small ball bearing as a roller). Each handle is a bell crank whose
  short arm, inside the housing, has a PROFILED SLOT. The pin runs in both slots.

        carriage ──●── pin (centre line)
                  ╱ ╲
       slot in   ╱   ╲   slot in
       arm L    ╱     ╲  arm R
             ○─┘       └─○   pivots (spacing P)
             │           │
          handle L    handle R  (long arms + grips, outside the housing)
```
- Equations: the slot is **the path of the pin as seen from the handle** while the handle
  follows the required β(s):

      pin (housing frame)   = (0, Y0 + s/n)
      slot point (handle R) = Rot(−β(s)) · (pin − pivot_R),   pivot_R = (+P/2, 0)

  Any β(s) can be cut into the slot, including the dwell (slot section parallel to the carriage
  travel for s > 300 mm). The slot walls hold the handle positively in both directions.
- Pressure angle (slot normal vs carriage travel): computed numerically in `kinematics.py`;
  52° at closed falling to 41° at full open for n = 4, Y0 = 150 (Section 5.3).
- Symmetry: inherent and rigid — one pin, two mirrored slots. No sync gears.
- Loads: pin force `= T_both / (dy/dβ)`, dy/dβ = 143 mm/rad (n = 4). Section 6 gives the values.
- Part count: as D minus 2 links/4 pins, plus 1 pin + roller (≈ 12).
- Friction: as D, with the pin roller instead of link pins → **η ≈ 0.8** (A3).
- Printability: the slotted arms are flat plates — ideal for FDM; the slot profile is generated parametrically.
- Failure modes: slot wall wear or spreading under load (arms ≥ 8 mm thick, PETG), pin bending (M4 at 300 N over a 12 mm span ≈ 143 MPa — OK for steel, TODO grade; actual loads are ~110 N with the Section 5.2 geometry), carriage binding.
- Verdict: fewest moving parts that still satisfy Findings 1–3; the best force budget.

Also considered and rejected without a full score: a two-stage planetary (compact variant of A,
same problems), and a cable-capstan reduction (very low friction, but needs non-standard cord and
has no dwell).

---

## 3. Weighted comparison

Scores 1 (poor) – 5 (excellent). Weights sum to 100. Totals re-checked in rev 2 (B corrected
225 → 235; A, C, D, E unchanged).

| Criterion | Wt | A spur | B worm | C drum+links | D block+links | E block+slots |
|---|---|---|---|---|---|---|
| Meets mapping incl. dwell / overtravel | 15 | 2 | 2 | 5 | 2 | 5 |
| Actuation force (R0 and R_load) | 20 | 3 | 1 | 2 | 5 | 4 |
| Compactness (width, height) | 10 | 3 | 4 | 4 | 3 | 3 |
| Part count & cost | 10 | 2 | 4 | 4 | 3 | 4 |
| Printability (FDM, tolerances) | 10 | 3 | 2 | 3 | 4 | 5 |
| Robustness under hand load | 10 | 2 | 2 | 3 | 4 | 4 |
| Ease of assembly / adjustment | 5 | 2 | 3 | 4 | 3 | 4 |
| Ease of return (spring) | 5 | 3 | 1 | 3 | 5 | 5 |
| Safety (guarding, failure modes) | 15 | 3 | 3 | 4 | 3 | 4 |
| **Weighted total /500** | | **260** | **235** | **350** | **355** | **420** |

Check (B): 15·2 + 20·1 + 10·4 + 10·4 + 10·2 + 10·2 + 5·3 + 5·1 + 15·3 = 30+20+40+40+20+20+15+5+45 = 235.

How to read this honestly:
- **E vs D** is decided by the dwell criterion, now confirmed required (Q3).
- **C vs D** is a genuine close call: C is more compact with a free dwell; D has half the force.
- A scores low mainly for part count, output-tooth loads and the missing dwell. If the instructor wants gears as course content, A is defensible with a slip clutch and m ≥ 2 at the output.
- B is out.

---

## 4. Recommendation

**Primary: E — reeved cable (n = 4) + spring-returned carriage + slotted bell-crank handles,
with the offset geometry of Section 5.2 (θ = +3° → +33°, ΔS = 70 mm).**
No gears, threads or clock spring; symmetric by construction; the angle-vs-travel profile and
the dwell are a single printed slot generated from `params.py`; hand torque always closing so the
cable is never slack and the handles fail closed; R0 ≈ 9 N (goal ≤ 10), R_load ≈ 41 N at the
assumed 200 N/hand (goal ≈ 40, see Q11).

**Fallback: C — threaded drum / barrel cam + runner + links.** A different family, so it does
not share E's risks (carriage binding, slot wear, housing height). The offset geometry of
Section 5.2 applies to C as well.

What would make me change my mind:
- Measured hand loads far above 200 N → the R_load goal cannot be met by E at ΔS = 70; reduce ΔS or revisit the goal (Q11).
- Housing height limit below ~280 mm above the pivots (Q12) → N_REEVE = 6, or C.
- Slot wear in prototype test 2 → C.
- Instructor wants gears → A, with a slip clutch.

---

## 5. First-pass layout for E

### 5.1 Overall arrangement (starting values; all in `params.py`)

```
                 carabiner eye (printed, M8 bolt through, or steel eye — TODO)
                 reference-cable fairlead (smooth printed bush, radius ≥ 5 mm)
      ┌──────────────────────────────────────┐  ─┐
      │ fixed pulley (Ø30)        dead end    │   │  ~40 mm
      │                                        │   │
      │   carriage: 2× pulley Ø30, guide rods  │   │  carriage travel 100 mm (400 mm stroke)
      │   pin + roller on centre line          │   │  + 10 mm margin each end
      │   extension spring to housing bottom   │   │
      │                                        │   │
      │   slotted short arms, pin 150→250 mm   │   │  ≈ 270 mm above pivots total
      ├───○────────────────────────────────○───┤  ─┘  pivots, spacing P ≈ 99 mm
      │   │  guard skirt / handle exit slots │   │
      └───┼────────────────────────────────┼───┘
          │ long arm                        │
         (grip Ø32 × 130)               (grip)
```

| Item | Value | Note |
|---|---|---|
| Reeving parts `N_REEVE` | 4 | 3 pulleys. n = 6: pressure angle 44° and ~35 mm shorter, but 5 pulleys. |
| Carriage travel | 75 mm (300 mm) / 100 mm (400 mm) + 10 mm margins | sets housing height |
| Pin start height `Y0_PIN_MM` | 150 mm above the pivot line | 120 mm gave a 58° pressure angle |
| Pulley diameter | 30 mm (A4) | TODO bearing size |
| Carriage guide | 2 × Ø4 steel rods (TODO: plain M4 rod is acceptable for v1) in printed bushes, 0.3 mm clearance | printed rails only if rods unavailable |
| Pin | M4 bolt, ball-bearing roller OD 10 mm (TODO size) | slot width 10.3 mm |
| Slot arm thickness | ≥ 8 mm, PETG, 100 % perimeters near the slot | wall ≥ 2 mm rule |
| Housing width proxy | P + 2 × 20 = **139 mm** | target ≤ 170 |
| Housing height above pivots | ≈ 270 mm | **tall; Q12** |

### 5.2 Handle geometry — offset bell crank (rev 2)
Each handle is a bell crank. The **grip tube** keeps the required angle (25° → 55° from
vertical). The **pivot-to-grip-centre line** (length `L_LINE`, angle θ from vertical) is a
separate choice. A vertical hand force gives `T = F · L_LINE · sin θ`: θ > 0 closes, θ < 0 opens.

Rev 1 used θ = −15° → +15° (grip under the pivot at mid-stroke). That minimises |T| but makes the
load *open* the handles in the first half — Finding 3, analysed in Section 6. **Rev 2 uses
θ = +3° → +33°**: the grip hangs 3° outboard of vertical at closed, so the hand torque is always
closing (small at closed, largest at open). Equations:

    L_LINE = ΔS / (2 (sin θ_open − sin θ_closed))
    P      = S_closed − 2 L_LINE sin θ_closed
    T(s)   = F · L_LINE · sin θ(s),   θ(s) = θ_closed + β(s)

| ΔS | S closed → open | L_LINE | P | width proxy | T closed | T open (per handle, F = 200 N) |
|---|---|---|---|---|---|---|
| 60 | 106 → 166 | 60.9 | 99.6 | 139.6 | 0.64 N·m | 6.64 N·m |
| **70** | 106 → 176 | 71.1 | 98.6 | 138.6 | 0.74 N·m | 7.74 N·m |
| 80 | 106 → 186 | 81.3 | 97.5 | 137.5 | 0.85 N·m | 8.85 N·m |
| rev-1 symmetric, ΔS = 70 | 106 → 176 | 67.6 | 141 | 181 | −3.5 N·m (opening) | +3.5 N·m |
| original (P = 0, L = 127) | 107 → 208 | 127 | 0 | — | 10.7 N·m | 20.8 N·m |

Note the pivot spacing is now *less* than the closed hand spacing (grips hang slightly outboard),
so the housing gets narrower, not wider: ≈ 139 mm, inside the original 140 mm target even before
the relaxation. The price is a 2× larger peak torque than the symmetric geometry, carried by the
pin and slot (Section 6.3) — still 2.7× lower than the original apex-pivot geometry.

Hand-spacing check: S_closed = 106 mm is an input, so it passes by construction.

### 5.3 Slot profile (the "cam")
`kinematics.slot_profile_mm()` returns the right-arm slot centre line in the arm frame
(origin at the pivot, x outward, y up, frame = housing frame at closed):

    pin − pivot = (−P/2, Y0 + s/N_REEVE);   slot(s) = Rot(−β(s)) · (pin − pivot)

Baseline (ΔS = 70, n = 4, Y0 = 150), every 50 mm of travel:

| s | 0 | 50 | 100 | 150 | 200 | 250 | 300 | 350 | 400 |
|---|---|---|---|---|---|---|---|---|---|
| x [mm] | −49.3 | −34.9 | −18.1 | 0.9 | 22.1 | 45.1 | 69.8 | 76.1 | 82.3 |
| y [mm] | 150.0 | 166.2 | 180.9 | 193.9 | 204.8 | 213.4 | 219.5 | 230.3 | 241.1 |
| pressure angle | 52° | 50° | 48° | 46° | 44° | 42° | dwell | dwell | dwell |

The slot is ~155 mm long: 130 mm of drive section and 25 mm of straight dwell. Slot width =
roller OD + 0.3 mm. Slot ends are the end stops, backed by hard stops on the carriage so the
printed slot ends do not take impact. TODO (v2 of the profile): ease the last ~10 % of the drive
section into the dwell so the handle does not stop with a velocity step.

### 5.4 Spring (rev 2: unchanged values, now justified by Section 6)
Extension spring, carriage to housing bottom, pulls the carriage down (stores cable).
`SPRING_PRELOAD_N` = 20 N at closed, `SPRING_RATE_N_PER_MM` = 0.10 → 30 N at the end of the
dwell (100 mm). With the offset geometry the hand torque only ever adds to the spring, so the
spring's only jobs are (a) keeping the cable taut with no hand load and (b) closing the handles
against friction on the way up. Resulting no-load tension: T_cable = (spring + friction)/4 =
6.0 N at closed → **R0 = 9.0 N** at the end of the dwell. Goal ≤ 10 N: PASS, with little margin.
A constant-force spring would flatten this to ~6 N. TODO: catalogue selection; no part number assumed.

### 5.5 Pivots and synchronisation
Each handle pivots on an M6 or M8 bolt (TODO: the brief lists M3/M4; M4 is marginal for a 200 N
hand load in bending — I ask to allow M6/M8 for the two pivots) in two bushings or small ball
bearings. Synchronisation is the single pin in the two mirrored slots; nothing else is needed.

### 5.6 Reference cable: anchor, routing, snag control
- Anchor: webbing loop or hook strap around the frame upright / top-pulley bracket, as close to the main cable's exit as possible so the two cables hang parallel. Offset error: 100 mm offset at 1 m height costs 0.4 % of payout (Section 7.6). TODO: machine-specific (Q6).
- Entry: smooth printed fairlead on the housing top, radius ≥ 5 mm; ≥ 150 mm of free cable before the first pulley so the housing can swing ±15° without fouling.
- Snagging: all cable runs enclosed; the spring keeps the cable taut so it cannot jump a pulley. Pulley flanges ≥ 1.5 × cable diameter.
- Swivel (Q8): none in v1; add at the anchor if twist shows up.

### 5.7 Mount to the existing carabiner (no drilling)
A printed eye (or a steel eye bolt — TODO allowed?) on the housing top, sized for the machine's
carabiner (TODO measure, Q6). The stack load goes from the eye through the housing top plate into
the pivot bosses; the print orientation must keep this load path in-plane (TODO orientation study).

### 5.8 Guarding and end stops
- All moving parts inside the housing; handles exit through slots with ≥ 5 mm clearance and a soft skirt against finger entry. No pinch points between the handles themselves: at closed the grips are 106 mm apart (Q10).
- Closed stop: carriage lower hard stop (spring-loaded against it at rest).
- Open stop: slot dwell + carriage upper hard stop.

---

## 6. Carriage force balance and the torque sign-change problem (Part C)

### 6.1 Independent check of the reviewer's estimate
Symmetric geometry, ΔS = 60, θm = 15°: L_LINE = 60 / (4 sin 15°) = 58.0 mm. At the closed end
θ = −15°: T = 200 × 0.058 × sin 15° = 3.0 N·m per handle, **opening**; 6.0 N·m for both.
dy/dβ = (300/4) / 0.5236 = 143 mm/rad, so the pin pushes the carriage **up** with
6.0 / 0.143 = **42 N**. Spring at closed: 20 N. **I agree**: the carriage is driven up by 22 N
net before friction, the cable goes slack and the handles open under load before the user moves.
It is worse than the estimate suggests on the up stroke, where friction also acts against the
spring (Section 6.2).

### 6.2 Model
Forces on the carriage (positive = down, the spring's direction):

    spring   F_s(s)  = preload + rate · (y_pin − Y0)                     [always down]
    pin      F_p(s)  = 2 T(s) / (dy/dβ),  T = F · L_LINE · sin θ(s)       [down if T closing]
                       (zero in the dwell: slot parallel to the travel)
    friction F_f(s)  = (1 − η)(F_s + |F_p|), opposes carriage motion      [ASSUMPTION A3]
    cable    N · T_c = F_s + F_p + F_f   (down stroke, carriage moving up)
             N · T_c = F_s + F_p − F_f   (up stroke, carriage moving down)

Slack = T_c below the 2 N margin in either direction. Quasi-static; inertia ignored (TODO: the
carriage is light, but the handles plus the user's hands are not — test 5).

### 6.3 Results — symmetric geometry (rev 1) vs offset geometry (rev 2)
Spring 20 N + 0.10 N/mm, η = 0.8, N = 4. Full tables with 25 mm steps are printed by
`python3 kinematics.py` (section 3); the offset-geometry grid is the code's default, the
symmetric grid is obtained by setting `THETA_CLOSED_DEG = -15`.

**Symmetric θ = −15° → +15° (rev 1): cable tension T_c [N], down / up stroke**

| ΔS | F | s = 0 | 75 | 150 | 225 | 300 | first slack |
|---|---|---|---|---|---|---|---|
| 60 | 0 | 6.0 / 4.0 | 6.6 / 4.4 | 7.1 / 4.8 | 7.7 / 5.1 | 8.2 / 5.5 | never |
| 60 | 100 | 1.8 / **−2.3** | 4.5 / 1.2 | 7.1 / 4.8 | 10.9 / 7.2 | 14.5 / 9.7 | **s = 0** |
| 60 | 200 | **−2.4 / −8.6** | 2.3 / **−2.0** | 7.1 / 4.8 | 14.0 / 9.3 | 20.8 / 13.9 | **s = 0** |
| 70 | 100 | 1.1 / **−3.3** | 4.1 / 0.7 | 7.1 / 4.8 | 11.4 / 7.6 | 15.6 / 10.4 | **s = 0** |
| 70 | 200 | **−3.8 / −10.7** | 1.6 / **−3.0** | 7.1 / 4.8 | 15.1 / 10.1 | 22.9 / 15.3 | **s = 0** |
| 80 | 100 | 0.4 / **−4.4** | 3.7 / 0.2 | 7.1 / 4.8 | 11.9 / 7.9 | 16.6 / 11.1 | **s = 0** |
| 80 | 200 | **−5.2 / −12.8** | 0.9 / **−4.1** | 7.1 / 4.8 | 16.1 / 10.8 | 25.0 / 16.7 | **s = 0** |

Slack from the closed end up to ~s = 100 mm at every non-zero hand load, in both directions.
Pin force at closed: −42 / −49 / −56 N (ΔS = 60/70/80) at F = 200 N.

**Offset θ = +3° → +33° (rev 2), ΔS = 70: pin force, friction and T_c [N]**

| F | s | T per handle | pin | spring | friction | T_c down | T_c up |
|---|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 20.0 | 4.0 | 6.0 | 4.0 |
| 0 | 400 | 0 | 0 | 30.0 | 6.0 | **9.0** (= R0) | 6.0 |
| 100 | 0 | 0.37 | 5 | 20.0 | 5.0 | 7.6 | 5.0 |
| 100 | 300 | 3.87 | 54 | 27.5 | 16.3 | 24.5 | 16.3 |
| 200 | 0 | 0.74 | 10 | 20.0 | 6.1 | 9.1 | 6.1 |
| 200 | 150 | 4.39 | 61 | 23.8 | 17.0 | 25.5 | 17.0 |
| 200 | 300 | 7.74 | 108 | 27.5 | 27.1 | **40.7** (= R_load) | 27.1 |
| 200 | 400 | 7.74 | 0 (dwell) | 30.0 | 6.0 | 9.0 | 6.0 |

First slack: **never**, for all ΔS in {60, 70, 80} and F in {0, 100, 200}. Minimum tension is
the no-load up stroke at closed, 4.0 N.

R0 and R_load summary (offset geometry): R0 = 9.0 N for every ΔS; R_load = 36.1 / 40.7 / 45.3 N
for ΔS = 60 / 70 / 80 (peak just before the dwell). Pin force peak 93 / 108 / 124 N.

### 6.4 Fixes evaluated
**(a) Stiffer spring, keep the symmetric geometry.** Preload needed so T_c ≥ 2 N on the up
stroke at closed (rate 0.10): 73 / 84 / 94 N for ΔS = 60 / 70 / 80. Resulting
R0 = 25 / 28 / 31 N — **fails the 10 N goal by ~3×** — and R_load = 37 / 42 / 47 N, i.e. no better
than the offset geometry at the loaded end. The spring also has to be pulled by the user on every
rep with no hand load. Rejected.

**(b) Offset geometry, torque never changes sign.** Grip under the pivot at the closed end,
plus a +3° bias so the closing torque is strictly positive and tolerances cannot flip it
(θ = 0° exactly would give zero closing torque at rest). Values (ΔS = 70): L_LINE = 71.1 mm,
P = 98.6 mm, width proxy 139 mm, T_max = 7.74 N·m per handle, pin force 108 N, cable tension
40.7 N with friction (24.5 N ideal + spring + friction). R0 unchanged at 9.0 N. For reference,
θ = 0° → 30° exactly gives P = 106, width 146, T_max = 7.0 N·m, R_load = 37.6 N.
**Chosen.** It costs a 2× larger peak torque than symmetric, in exchange for: cable always taut,
R0 unchanged, narrower housing, handles biased closed on cable release (Q10), and no extra parts.

**(c) Detent or latch at the closed position.** A detent cannot tell "cable pulling" from
"hand load pushing": at the start of the stroke both act in the same direction on the carriage.
A detent stiff enough to hold 42–56 N would also hold against the cable, so the handles would
not open at all until the detent broke away — and the opening load persists to s ≈ 100–150 mm,
not just at s = 0. A cable-tension-triggered release would be a new mechanism with its own
failure modes. Rejected.

**(d) Other options considered.**
- *Make the torque opening everywhere* (θ = −30° → 0°): the cable would then only restrain, and
  with the cable released the handles would fly open under load — opposite to Q10, and the
  spring would still need to beat the opening load on the up stroke. Rejected.
- *Hybrid: small offset + slightly stiffer spring*: not needed; the +3° offset already gives
  ≥ 4 N minimum tension. Keep the spring at 20 N and revisit after prototype test 9.
- *Reduce ΔS*: lowers all loads linearly (ΔS = 60: R_load 36 N) and is the first knob to turn if
  Q11 says 40 N is a hard limit.

### 6.5 Comparison of fixes (ΔS = 70, F = 200 N)

| Fix | R0 [N] | R_load [N] | housing width proxy | extra parts | main risk | printability |
|---|---|---|---|---|---|---|
| (a) stiff spring, symmetric | 28 | 42 | 181 mm | 0 | fails R0 goal; big spring | same |
| **(b) offset +3° → +33°** | **9** | **41** | **139 mm** | **0** | 2× peak slot load vs symmetric | same (slot a bit longer) |
| (c) detent at closed | 9 (until breakaway) | 42 | 181 mm | 2–4 | handles may never open; wear | worse |
| (d) opening everywhere | 9 | ~30 | ≥ 200 mm | 0 | handles fly open on cable release | same |

**Recommendation: (b).** What would change my mind: a measured hand load well above 200 N
(peak slot/pin loads scale with it — then reduce ΔS first), or prototype evidence that the slot
walls cannot take ~110 N at a 45–52° pressure angle (then C, with the same offset geometry).

---

## 7. Calculations to verify by hand

### 7.1 Angle vs travel (linear map, dwell 300 → 400 mm)
| s [mm] | 0 | 75 | 150 | 225 | 300 | 375 |
|---|---|---|---|---|---|---|
| included angle | 50° | 65° | 80° | 95° | 110° | 110° |
| per handle β | 0° | 7.5° | 15° | 22.5° | 30° | 30° |
| θ (line angle) | 3° | 10.5° | 18° | 25.5° | 33° | 33° |
| carriage y above pivots (n = 4) | 150 | 168.8 | 187.5 | 206.2 | 225 | 243.8 |
| grip separation (ΔS = 70) | 106 | 124.5 | 142.5 | 159.8 | 176 | 176 |

### 7.2 The virtual-work constant
k = 300 / (30·π/180) = 573 mm per rad = 10.0 mm of travel per degree of handle rotation.
Cable tension (ideal) = T_both / 0.573. Carriage force = n × cable tension; dy/dβ = k/n = 143 mm/rad.

### 7.3 Hand torque and loads — three geometries (F = 200 N/hand, η = 0.8)
| | original (P = 0, L = 127) | symmetric ±15°, ΔS = 70 | **offset +3° → +33°, ΔS = 70** |
|---|---|---|---|
| T per handle at closed | 10.7 N·m closing | 3.5 N·m **opening** | 0.74 N·m closing |
| T per handle, max | 20.8 N·m | 3.5 N·m | 7.74 N·m |
| pin force, max (n = 4) | 291 N | ±49 N | 108 N |
| cable tension ideal / with friction and spring | 73 / ~100 N | 12 / 23 N (slack at closed!) | 27 / 41 N |
| R0 | ~9 N | 9 N | 9 N |
| cable slack under load? | no | **yes, s = 0–100 mm** | no |

### 7.4 Slot pressure angle
Numerical, `kinematics.pressure_angle_deg()`. Baseline n = 4, Y0 = 150: 52° (closed) → 41°
(open). Alternatives: n = 4, Y0 = 120 → 58°; n = 6, Y0 = 150 → 44°; n = 6, Y0 = 120 → 50°.
Slot normal force per arm ≈ (pin/2) / cos α = 54 / cos 45° ≈ 76 N at the loaded end.

### 7.5 Spring and no-load resistance
F_spring 20 → 30 N over 100 mm; friction (1 − 0.8) × F_spring = 4 → 6 N; T_c = (F_spring + friction)/4 = 6.0 → 9.0 N = R0.
Up stroke no-load: (F_spring − friction)/4 = 4.0 → 6.0 N ≥ 2 N margin.

### 7.6 Anchor offset error (cable not parallel)
payout = √((h+300)² + x²) − √(h² + x²). For x = 100 mm: h = 300 → 292 mm (−2.7 %); h = 1000 → 299 mm (−0.4 %).

### 7.7 Pin bending (M4, 12 mm span, simply supported)
At 300 N (original geometry): M = 0.9 N·m, σ = 32M/(πd³) ≈ 143 MPa. At the design 108 N: ≈ 52 MPa. Steel OK (TODO bolt grade).

### 7.8 For the fallback C
Drum r = 20: 2.39 turns over 300 mm (3.18 over 400); 40 mm runner travel → lead 16.8 mm; lead
angle on a 20 mm thread 14.9°; η ≈ 0.54; runner force at full open with a = c = 50 mm:
T_both / 0.082 = 15.5 / 0.082 ≈ 190 N (offset geometry) / 500 N (original).

### 7.9 For reference, A
Output sector torque 7.7 N·m on z = 60: m = 2, b = 10 → Ft = 128 N, Lewis-type stress ≈ 21 MPa (plausible for PETG, TODO data); m = 1 → ~110 MPa (fails).

---

## 8. Top 10 risks and the prototype test for each

| # | Risk | Test on the printed prototype |
|---|---|---|
| 1 | Hand torque higher than assumed (F > 200 N, or the user pushes outward) → R_load felt as heavy resistance | Load cell / spring balance in the reference cable during real pushdowns at 3 stack settings, starting at a few kg; compare with Section 6.3. |
| 2 | Slot walls wear or spread (pressure angle 41–52°, ~110 N pin load) | 500-cycle bench rig with weights giving 7.7 N·m on each handle; measure slot width before/after. |
| 3 | Carriage binds on guides (tolerances, side load if hands unequal) | Push each handle alone with the other free; measure R0 with a one-sided 100 N load. |
| 4 | Cable jumps a pulley when the user lifts fast (spring too weak; up-stroke tension only 4 N no-load) | Drop-release test: let the attachment rise at max machine speed 50 times; inspect. |
| 5 | Jolt at the dwell transition or the carriage hard stop; inertia of handles + hands not in the model | Run 400 mm and 450 mm strokes; feel / accelerometer on the housing. |
| 6 | Asymmetry from print tolerance (one slot off by 0.3 mm) | Measure both handle angles at 5 stroke points with a protractor jig; spec ≤ 2° difference. |
| 7 | Housing top plate / carabiner eye fails under stack load across layers | Static hang test to 2× max intended stack (TODO value) with a safety strap. |
| 8 | Reference-cable anchor slips or the cable snags the main cable | Mount on the real machine; 100 reps at 3 heights; check anchor position after. |
| 9 | R0 > 10 N once assembled (friction stack-up; the model has 1 N of margin) | Spring balance on the reference cable, no hand load, at 0/25/50/75/100 % stroke and in the dwell. |
| 10 | Torque sign flips at the closed end because of print tolerance (+3° bias too small) → handles creep open under load at rest | Hang 5 kg per grip at closed with the cable slack; handles must stay closed. Increase `THETA_CLOSED_DEG` if not. |

---

## 9. CAD part list (names only) and build order

Parts (one script each in `parts/`):
1. `housing_top` (carabiner eye, fairlead, fixed pulley seat, dead end)
2. `housing_body` (guide-rod seats, pivot bosses, spring anchor, handle exit guards)
3. `housing_cover`
4. `pulley` (×3, same part)
5. `carriage` (two pulley seats, guide bushes, pin seat, spring hook)
6. `handle_arm_slotted_R` and `_L` (mirrored; slot generated from `kinematics.slot_profile_mm`)
7. `handle_long_arm_R` / `_L` (grip mount; may merge with 6 if it prints)
8. `grip` (×2)
9. `pin_roller_spacer`
10. `end_stop_closed`, `end_stop_open`
11. `cable_dead_end_clamp`
12. `anchor_strap_bracket` (machine-specific, last — Q6)
Bought: 3 pulley bearings, 1 roller bearing, 2 pivot bolts + bushings, 2 guide rods, M3/M4
screws, extension spring, 1.5 mm cable + ferrules, carabiner.

Build order:
1. `params.py` / `kinematics.py` as now (done in rev 2).
2. `handle_arm_slotted` + `pin_roller_spacer` + a simple test fixture → **first print**, check the pin runs the whole slot by hand, run test 10.
3. `carriage` + `pulley` + guide rods → bench-test the reeving ratio and spring with a tape measure.
4. `housing_body` / `top` / `cover` → assemble, measure R0 (test 9).
5. Grips, guards, stops → first on-machine test at a few kg (tests 1, 5, 8).
6. Iterate: slot profile easing, spring, pressure angle.

---

*End of study. Nothing above is validated; the tests in Section 8 are the validation plan.*
