# autoRAD — Main concept design study (travel-actuated opening V-bar)

Status: **design study, nothing validated.** Every number below is a first-pass hand calculation
from the preliminary requirements. Items marked TODO are unknown. Items marked ASSUMPTION are
my choices and should be challenged.

Notation used throughout: `s` = attachment travel [mm], `β` = angle of one handle from the
centre line [deg], `T` = torque on one handle [N·m], `F` = user force per hand [N].

---

## 1. Problem restatement, assumptions, open questions

### 1.1 Problem in one paragraph
A removable attachment for a cable machine. It hangs from the existing carabiner. Two grip
handles form a V. As the user pushes the attachment down, the handles must open symmetrically
from 50° to 110° included angle over the first 300 mm of travel, then hold 110° for any extra
travel. On the way up they must close again. The motion must be driven only by the attachment's
travel relative to the fixed frame, with no electronics and no manual adjustment during use.

### 1.2 Two findings that change the problem (read these first)

**Finding 1 — the hand load dominates, not the gear ratio.**
In the geometry as specified (both handles pivoting at the apex, grip centre 127 mm from the
pivot, handles hanging down-and-out), a vertical push `F` on a grip creates a torque about the
pivot of

    T = F · L · sin β        (L = 127 mm, β = 25° closed … 55° open)

With an ASSUMED 200 N per hand (≈ 40 kg stack shared by two hands — TODO: confirm typical
pushdown loads with gym data):

| β | T per handle | T both handles |
|---|---|---|
| 25° (closed) | 10.7 N·m | 21.5 N·m |
| 40° (mid) | 16.3 N·m | 32.6 N·m |
| 55° (open) | 20.8 N·m | 41.6 N·m |

This torque tries to **close** the handles (a hanging handle pushed down behaves like a pendulum).
Whatever transmission we choose, the reference cable must react it. By virtual work, over the
mapping stroke the cable pays out 300 mm while each handle turns 0.5236 rad, so

    k = 300 mm / 0.5236 rad = 573 mm of cable per radian of handle rotation
    cable tension (ideal, no friction) = T_both / 0.573 m

i.e. 37 N at closed and **73 N at open**, before friction. With a 50 %-efficient stage that is
~150 N. The ≤ 10 N actuation goal cannot be met by any transmission while this torque exists.
It can only be met by changing the **handle geometry** so the hand's force line passes near the
pivot (Section 5.2 shows how; it cuts the torque by about 4×). This is the most important design
decision in the project.

**Finding 2 — the "hold 110° beyond 300 mm" requirement needs a dwell, not a hard stop.**
If the handles simply hit a stop while the attachment keeps moving down, the spool (or carriage)
stops, the reference cable can no longer pay out, and it goes taut: the reference cable then
carries the user's full push plus the stack, with a jolt, and something breaks (anchor, cable,
housing or the user's wrists). Every architecture below therefore needs one of:
(a) a cam **dwell** (the drive keeps moving, the output does not),
(b) a lost-motion element that still positively holds the handles under load, or
(c) a slip clutch, or
(d) a changed requirement (Section 1.4, Q3).

### 1.3 Assumptions (ASSUMPTION = mine; change in `params.py`)
| # | Assumption | Why it matters |
|---|---|---|
| A1 | User force per hand `F_HAND` = 200 N, vertical. | Sets every internal load. TODO measure. |
| A2 | Reference cable anchored to the frame **above** the attachment, hanging parallel to the main cable, payout = travel (1:1). | Tension then adds to the felt load; anchor offset error is small (Section 6.6). |
| A3 | Friction coefficient printed-on-printed, dry: µ ≈ 0.2 (TODO: measure; PLA/PETG dry sliding values vary widely). | Drives worm/screw efficiency estimates. |
| A4 | Ball-bearing pulley efficiency ≈ 0.97 each; plain printed sliding joints ≈ 0.85–0.9 per joint. | Used in the force estimates. These are rough engineering guesses, not data. |
| A5 | Handle swing plane is the frontal plane (hands spread sideways), so pivot axes are horizontal front-to-back and vertical hand force produces the torque in Finding 1. If the V plane is tilted by angle `τ`, torque scales by cos τ. | TODO: confirm with the ergonomic study. |
| A6 | 1.5 mm steel cable; sheave diameter ≥ ~20× cable diameter for life (common wire-rope guidance, not a standard; TODO confirm for the chosen cable). | Pulley ≥ 30 mm. |
| A7 | Linear angle-vs-travel mapping is wanted (50 + 60·s/300). | Nothing in the brief requires linear; see Q4. |

### 1.4 Open questions (answers change the design)
1. **Handle geometry**: may the pivots be moved apart and the grip positioned under the pivot (Section 5.2)? This changes the "127 mm from pivot" and "housing ≤ 140 mm" numbers.
2. **How much should the hands separate?** Specified geometry gives 107 → 208 mm between grip centres. If 60–80 mm of separation change is enough ergonomically, the loads and the housing width both drop (Section 5.2 table).
3. **Is the dwell at 110° really required?** Alternatives: map 400 mm → 110° (a 250 mm user reaches 87°), or let long-stroke users exceed 110°. A dwell costs a cam; a hard stop alone is unsafe (Finding 2).
4. Linear mapping, or is a profile (slow start, fast finish, or vice versa) better for the triceps movement?
5. Typical and maximum stack loads for the target users (A1).
6. Which machine(s)? Height of the top pulley, where a strap can anchor, carabiner size. The reference-cable anchor design depends on this entirely.
7. Definition of actuation resistance — proposal in Section 1.5.
8. Does the attachment have to swivel about the vertical (as a V-bar does on its ring)? A swivel twists the reference cable.
9. Is a tape-measure style clock spring acceptable as "a spring", or only coil springs?
10. Safe failure mode preference: if the reference cable detaches, handles snap closed (closing-torque geometry) or snap open (opening-torque geometry). Both are ugly; which is less bad?

### 1.5 Proposed definition of "actuation resistance" (requirement says TBD)
Define `R` = tension in the reference cable during the down stroke = extra force the user must
push beyond the stack. Two values:
- **R0 (no-load)**: measured on the bench by pulling the reference cable with a spring balance
  with no hand load. Target **R0 ≤ 10 N** over the full stroke. This is what the spring and
  friction cost.
- **R_load**: with the design hand load applied. `R_load ≈ T_both / (0.573 · η) + R0`. Target:
  TODO — propose ≤ 10 % of the stack load, so the user barely notices it.

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
- Equations: `i = 573 / r_spool`; r = 12 → 47.7, r = 20 → 28.7, r = 30 → 19.1. From
  `kinematics.py`: 2 stages for r ≥ 18 mm (e.g. 12/59 + 12/58 at r = 24), 3 stages for r ≤ 15.
- Symmetry: final sector gears on the two handle pivots mesh with each other (1:1 mirror).
- Part count: spool, clock spring, 2–3 compound gears, 2 sectors, 3–4 shafts, 6–8 bearings/bushings, housing (≈ 15 parts + fasteners).
- Printability: good; cq_gears makes the gears. Output-stage teeth must carry the full hand torque: with m = 1 and T = 5 N·m, tooth root stress by a simple Lewis estimate is ~70 MPa (too high for PLA/PETG, TODO material data); m = 2–3 is needed at the output, which grows the housing.
- Friction/backlash: 2–3 meshes ≈ 0.85–0.9 overall. Backlash is taken up in one direction because the spring and the hand torque both load the train the same way; it shows as free play only when unloaded.
- Overtravel: needs a slip clutch at the spool or a lost-motion stage (extra parts, not shown).
- Failure modes: output-tooth breakage under hand load; clock spring fatigue; shaft bosses wearing oval.
- Verdict: works, well understood, but it is the heaviest option and still leaves Findings 1 and 2 unsolved.

### B. Spool + worm → worm wheel on a handle

```
 cable ──► [spool] ── same shaft ── worm ─► worm wheel on handle R ─► sector gears sync handle L
```
- Equations: single stage, `i = z_wheel` for a 1-start worm: 20–48 teeth. Efficiency
  `η = tan λ / tan(λ + φ)`, φ = atan µ = 11.3° for µ = 0.2. A printable 1-start worm has
  λ ≈ 4–6° → **η ≈ 0.26–0.34**, and it is self-locking (λ < φ).
- Self-locking sounds attractive (the hand cannot back-drive the spool) but it does not help:
  during the down stroke the cable must drive the wheel **against** the hand torque through that
  26–34 % efficiency, so R_load ≈ 3× the ideal 73 N ≈ 220 N. The spring must also rewind through it.
- Symmetry: sector gears between handles.
- Part count: low (spool, worm, wheel, 2 sectors, spring, 4 bearings).
- Printability: printed worms work at module ≥ 1.5 but wear fast dry.
- Overtravel: needs a slip clutch (same as A).
- Verdict: **not worth it.** The one place a worm's self-locking would pay off (holding the hand load on the up stroke) is not where the problem is.

### C. Threaded drum (lead-screw / barrel cam) + runner + two links ("umbrella")

```
 cable ──► [drum: cable groove + helical thread on one shaft] ── nut/runner slides axially
           runner ── link ─► handle R (short arm)
                  └─ link ─► handle L (short arm)        clock spring on drum rewinds
```
- Equations: drum turns `N = 300 / (2π r)`; runner travel `x = N · lead`. For 40 mm runner
  travel: r = 15 → 3.18 turns, lead 12.6 mm; r = 25 → 1.91 turns, lead 20.9 mm. Lead angle on a
  20 mm thread: 11°–18° → **η ≈ 0.48–0.58** (A3). Link geometry with equal arm/link length `a`:
  runner position `y = 2a cos β`, so `dy/dβ = 2a sin β` (non-linear: slow at first, fast later).
- Overtravel: if the thread is cut as a **barrel-cam groove** instead of a true thread, the last
  turn(s) can be circumferential (zero lead) = a dwell. Handles hold 110° while the drum keeps
  turning. This is the cleanest answer to Finding 2.
- Symmetry: inherent — one runner on the centre line, two mirrored links.
- Loads: runner axial force `= T_both / (dy/dβ)`; at β = 55°, a = 50 mm: 41.6 / 0.082 ≈ 500 N
  on a printed thread (original geometry), ≈ 120 N with the Section 5.2 geometry. Drum needs a
  thrust bearing.
- Part count: drum, nut/runner, 2 links, clock spring, 2 radial + 1 thrust bearing, 4 link pins, housing (≈ 12).
- Printability: 2-start trapezoidal thread, lead 12–20 mm, prints well vertically; nut is easy.
- Friction/backlash: thread backlash ~0.3 mm → < 1° at the handle; friction is the weakness (η ≈ 0.5).
- Failure modes: thread wear/stripping under the hand load, clock spring sourcing, cable jumping the groove if slack.
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
- Equations: carriage travel `x = s / n`. n = 4 → 75 mm (100 mm for a 400 mm stroke); n = 3 →
  100 mm; n = 6 → 50 mm. Carriage force `= n · T_cable`. Link geometry as in C.
- Symmetry: inherent (central carriage, mirrored links).
- Part count: 3 pulleys + 3 bearings (n = 4), carriage, 2 guide rods or printed rails, 2 links,
  4 pins, extension spring, cable clamp/dead end, housing (≈ 14).
- Friction/backlash: pulleys on ball bearings ≈ 0.97 each (3 → 0.91), link pins ≈ 0.97, carriage
  guide 0.9–0.95 → **η ≈ 0.8–0.85**. Zero backlash (cable is always in tension).
- Overtravel: **no natural dwell.** A free slot (lost motion) does not work with a closing hand
  torque — the handles would fall back from the stop by the slot length. Needs a slip or a
  changed requirement (Q3).
- Failure modes: cable jumping a pulley when slack, carriage binding on printed rails, spring fatigue.
- Verdict: lowest force and simplest loads, but no dwell. Fixed by E.

### E. Reeved cable + carriage + **slotted bell-crank handles** (D with a built-in cam dwell) — my pick

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
- Equations: the slot is simply **the path of the pin as seen from the handle** while the handle
  follows the required β(s):

      pin (housing frame)   = (0, y0 + s/n)
      slot point (handle R) = Rot(−β(s)) · (pin − pivot_R),   pivot_R = (+P/2, 0)

  Any β(s) can be cut into the slot: linear, eased, and a dwell (straight slot section parallel
  to the carriage travel for s > 300 mm). The slot walls hold the handle positively in both
  directions, so the dwell works under a closing hand torque — this is what D lacks.
- Pressure angle (angle between slot normal and carriage travel): `tan α ≈ (dy/dβ) / r_slot`
  with dy/dβ = 143 mm/rad for n = 4. r_slot = 120 mm → 50°, 150 mm → 44°, 180 mm → 39°. High
  for a cam, acceptable with a rolling follower; this is the main thing to test.
- Symmetry: inherent and rigid — one pin, two mirrored slots. No sync gears.
- Loads: pin force `= T_both / (dy/dβ)` = 41.6 / 0.143 ≈ 290 N (original geometry) or
  ≈ 70 N (Section 5.2 geometry). Cable tension = pin force / n.
- Part count: as D minus 2 links/4 pins, plus 1 pin + roller (≈ 12).
- Friction: as D, with the pin roller instead of link pins → **η ≈ 0.8**.
- Printability: the slotted arms are flat plates — ideal for FDM; the slot profile is generated parametrically (a good "CAD as a design tool" story for the report).
- Failure modes: slot wall wear or spreading under load (make arms ≥ 8 mm thick, PETG), pin bending (M4 at 300 N over a 12 mm span ≈ 140 MPa — OK for steel, TODO), carriage binding.
- Verdict: fewest moving parts that still satisfy both Findings; the best force budget.

Also considered and rejected without a full score: a two-stage planetary (a compact variant of
A, same problems), and a cable-capstan reduction (3 mm drum driving a 70 mm sector with cord —
very low friction but needs non-standard cord and still has no dwell).

---

## 3. Weighted comparison

Scores 1 (poor) – 5 (excellent). Weights sum to 100.

| Criterion | Wt | A spur | B worm | C drum+links | D block+links | E block+slots |
|---|---|---|---|---|---|---|
| Meets mapping incl. dwell / overtravel | 15 | 2 | 2 | 5 | 2 | 5 |
| Actuation force (R0 and R_load) | 20 | 3 | 1 | 2 | 5 | 4 |
| Compactness (≤ 140 mm wide, height) | 10 | 3 | 4 | 4 | 3 | 3 |
| Part count & cost | 10 | 2 | 4 | 4 | 3 | 4 |
| Printability (FDM, tolerances) | 10 | 3 | 2 | 3 | 4 | 5 |
| Robustness under hand load | 10 | 2 | 2 | 3 | 4 | 4 |
| Ease of assembly / adjustment | 5 | 2 | 3 | 4 | 3 | 4 |
| Ease of return (spring) | 5 | 3 | 1 | 3 | 5 | 5 |
| Safety (guarding, failure modes) | 15 | 3 | 3 | 4 | 3 | 4 |
| **Weighted total /500** | | **260** | **225** | **350** | **355** | **420** |

How to read this honestly:
- **E vs D (355 vs 420)** is decided almost entirely by the dwell criterion. If Q3 is answered
  "no dwell needed", D and E are equal and D is slightly simpler.
- **C vs D** is a genuine close call: C is more compact and has the dwell for free; D has half the
  actuation force. I weighted force at 20 because the brief's ≤ 10 N goal is already hard.
- A scores low mainly for part count, output-tooth loads and the missing dwell, not because gears
  are bad. If you already have cq_gears working and want a gear train for the course content, A
  is defensible — but it does not solve Finding 2.
- B is out.
- Height is scored under compactness: D/E are the tallest (Section 5.1); C is the shortest.

---

## 4. Recommendation

**Primary: E — reeved cable (n = 4) + spring-returned carriage + slotted bell-crank handles,
with the handle geometry of Section 5.2.**
Reasons: no gears, no threads, no clock spring; symmetric by construction; the angle-vs-travel
profile and the dwell are a single printed slot generated from `params.py`; lowest internal loads
and the only option with a credible path to R0 ≤ 10 N.

**Fallback: C — threaded drum / barrel cam + runner + links.** A different family, so it does
not share E's risks (carriage binding, slot wear, housing height). Use it if the prototype
shows the slotted arms wearing or the housing becoming too tall.

What would make me change my mind:
- Q3 answered "no dwell" → go to **D** (drop the slots, use links).
- Hand loads measured far below 200 N, or the team accepts a ~50 N actuation force → **C** becomes
  the most compact and I would take it.
- Housing height limit < ~250 mm above the pivots → C.
- Instructor wants gears as course content → A, with a slip clutch and m ≥ 2 at the output.

---

## 5. First-pass layout for E

### 5.1 Overall arrangement (dimensions are starting points; all go in `params.py`)

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
      │   slotted short arms, r_slot 120–150   │   │  ~ 180–200 mm above pivots total
      ├───○────────────────────────────────○───┤  ─┘  pivots, spacing P
      │   │  guard skirt / handle exit slots │   │
      └───┼────────────────────────────────┼───┘
          │ long arm                        │
         (grip Ø32 × 130)               (grip)
```

| Item | Value | Note |
|---|---|---|
| Reeving parts `N_REEVE` | 4 | 3 pulleys. n = 6 shortens the housing by 33 mm but adds 2 pulleys. |
| Carriage travel | 75 mm (300 mm) / 100 mm (400 mm) + 10 mm margins | sets housing height |
| Pulley diameter | 30 mm (A6) | bore for the bearing; TODO bearing size |
| Pulley bearings | small ball bearing, 3 or 4 mm bore | TODO size; bushings as plan B |
| Carriage guide | 2 × Ø4 steel rods (TODO: plain M4 rod is acceptable for v1) in printed bushes, 0.3 mm clearance | printed rails only if rods unavailable |
| Pin | M4 bolt, ball-bearing roller | roller OD ≈ 10–12 mm; slot width = OD + 0.3 |
| Slot arm thickness | ≥ 8 mm, PETG, 100 % perimeters near the slot | wall ≥ 2 mm rule |
| r_slot (pin distance from pivot) | 120 → 170 mm over the stroke (pin starts ~120 mm above the pivot line) | pressure angle 50° → 40° |
| Housing width | P + 2 × (pivot boss ≈ 20) | see 5.2 |
| Housing height above pivots | ≈ 190 mm | **exceeds a V-bar; flag** |

### 5.2 Handle geometry — reducing the hand torque (recommended change to the requirement)
Make each handle a bell crank and move the pivots apart. Let the line from pivot to grip centre
have length `L_LINE` and angle `θ` from vertical (outward positive). Keep the **grip tube**
angle as specified (25° → 55° from vertical), but let θ sweep symmetrically `−θm → +θm` so the
grip hangs directly under the pivot at mid-stroke. Then, independent of θm:

    T_max  = F · ΔS / 4                 (ΔS = change in grip-centre separation)
    P      = S_closed + ΔS / 2          (pivot spacing)
    L_LINE = ΔS / (4 sin θm)

| ΔS | S closed → open | P | L_LINE (θm = 15°) | T_max per handle (F = 200 N) | ideal cable tension |
|---|---|---|---|---|---|
| 101 (as specified) | 106 → 207 | 157 | 98 | 5.1 N·m | 18 N |
| 80 | 106 → 186 | 146 | 77 | 4.0 N·m | 14 N |
| 60 | 106 → 166 | 136 | 58 | 3.0 N·m | 10.5 N |
| 40 | 106 → 146 | 126 | 39 | 2.0 N·m | 7 N |
| original (P = 0, L = 127) | 107 → 208 | 0 | 127 | **20.8 N·m** | **73 N** |

So the 140 mm housing target and the full 101 mm separation change conflict: either relax the
width to ≈ 170–180 mm (P = 157 + bosses) or accept ΔS ≈ 60 mm at P = 136. **I recommend
ΔS = 60–80 mm and a housing width of 150–170 mm**, pending the ergonomic answer to Q2. The
torque drops 4–7×; everything downstream (pin, slot, carriage, cable, anchor) shrinks with it.
Note the torque now changes sign at mid-stroke (opening in the first half, closing in the second);
the slot walls handle both.

Hand-spacing check stays ≥ 106 mm by construction (S_closed is the input).

### 5.3 Slot profile (the "cam")
Generated numerically from `params.py` in the next step: for s = 0 … 400 mm, β(s) = 25° +
30°·min(s/300, 1) (plus a short easing over the last ~10 % so the handle does not hit the dwell
with a velocity step), `y(s) = Y0 + s/N_REEVE`, slot point = Rot(−β)·(pin − pivot). Slot width =
roller OD + 0.3 mm. Slot ends are the end stops (closed end and the end of the dwell), backed by
a hard stop on the carriage so the slot ends do not carry impact.

### 5.4 Spring
Extension spring from the carriage to the housing bottom (pulls the carriage down = stores cable).
Requirement: cable stays taut on the up stroke at any speed, i.e. `F_spring ≥ n · T_cable,min +
friction`. Target T_cable ≈ 5 N → F_spring ≈ 20 N at closed, ≈ 30 N at full open (k ≈ 0.1 N/mm
over 100 mm, preload 20 N). This gives R0 ≈ 7.5 N / 0.8 ≈ **9 N at full open** — just inside
the goal; a constant-force spring would make it ~6 N. TODO: spring selection from a catalogue;
no part number assumed.

### 5.5 Pivots and synchronisation
Each handle pivots on an M6 or M8 bolt (TODO: the brief lists M3/M4; M4 is marginal for a 200 N
hand load in bending through a 12 mm wide boss — I would ask to allow M6/M8 for the two pivots)
in two bushings or small ball bearings. Synchronisation is the single pin in the two mirrored
slots; nothing else is needed.

### 5.6 Reference cable: anchor, routing, snag control
- Anchor: webbing loop or hook strap around the frame upright / top-pulley bracket, as close to
  the main cable's exit as possible so the two cables hang parallel. Offset error: 100 mm offset
  at 1 m height costs 0.4 % of payout (Section 6.6) — negligible. TODO: machine-specific.
- Entry: smooth printed fairlead on the housing top, radius ≥ 5 mm; a short free length of
  cable (≥ 150 mm) before the first pulley so the housing can swing ±15° without fouling.
- Snagging: inside the housing all cable runs are enclosed; the spring keeps the cable taut so
  it cannot jump a pulley. Pulley grooves with side flanges ≥ 1.5 × cable diameter.
- Swivel (Q8): if the attachment must swivel, the reference cable will twist; a swivel link at
  the anchor (TODO: standard hardware?) solves it.

### 5.7 Mount to the existing carabiner (no drilling)
A printed eye (or a steel eye bolt — TODO allowed?) on the housing top, sized for a typical gym
carabiner (TODO: measure the machine's carabiner gate opening). The main cable load (stack) goes
straight through the housing top plate into the pivot bosses; the housing must be printed with
the layer direction chosen so this load is not across layers (TODO: orientation study in CAD).

### 5.8 Guarding and end stops
- All moving parts (pulleys, carriage, slotted arms, spring) inside the housing; handles exit
  through slots with ≥ 5 mm clearance, with a soft skirt to prevent finger entry.
- Closed stop: carriage lower hard stop (spring-loaded against it at rest).
- Open stop: slot dwell + carriage upper hard stop.

---

## 6. Calculations to verify by hand

### 6.1 Angle vs travel (linear map, dwell after 300 mm)
| s [mm] | 0 | 75 | 150 | 225 | 300 | 450 |
|---|---|---|---|---|---|---|
| included angle | 50° | 65° | 80° | 95° | 110° | 110° |
| per handle β | 25° | 32.5° | 40° | 47.5° | 55° | 55° |
| carriage y − Y0 (n = 4) | 0 | 18.75 | 37.5 | 56.25 | 75 | 100 (dwell) |

### 6.2 The virtual-work constant
k = 300 / (30·π/180) = 573 mm per rad = 10.0 mm of travel per degree of handle rotation.
Cable tension (ideal) = T_both / 0.573. Carriage force = n × cable tension.

### 6.3 Hand torque and loads — two geometries (F = 200 N/hand)
| | original (P = 0, L = 127) | bell-crank, ΔS = 60, θm = 15° |
|---|---|---|
| T per handle, max | 20.8 N·m | 3.0 N·m |
| pin force (n = 4, dy/dβ = 143 mm/rad) | 290 N | 42 N |
| cable tension ideal / with η = 0.8 | 73 / 91 N | 10.5 / 13 N |
| R_load incl. R0 ≈ 9 N | ~100 N | ~22 N |

### 6.4 Slot pressure angle
tan α = (dy/dβ)/r_slot = 143 / r_slot → r = 120: 50°, 150: 44°, 180: 39°. Compare n = 6 (95 mm/rad):
r = 120: 39°. Choose r_slot ≥ 120 and verify wear on the prototype.

### 6.5 Spring and no-load resistance
F_spring 20 → 30 N over 100 mm; T_cable = F_spring/4 = 5 → 7.5 N; R0 = T_cable / η ≈ 6 → 9 N.

### 6.6 Anchor offset error (cable not parallel)
payout = √((h+300)² + x²) − √(h² + x²). For x = 100 mm: h = 300 → 292 mm (−2.7 %); h = 1000 → 299 mm (−0.4 %).

### 6.7 Pin bending (M4, 300 N, 12 mm span, simply supported)
M = F·l/4 = 0.9 N·m; σ = 32M/(πd³) ≈ 143 MPa. Steel OK (TODO: bolt grade); with the Section 5.2
geometry the load is 4–7× lower.

### 6.8 For the fallback C
Drum r = 20: 2.39 turns over 300 mm (3.18 over 400); 40 mm runner travel → lead 16.8 mm; lead
angle on a 20 mm thread 14.9°; η ≈ 0.54; runner force at β = 55° with a = c = 50 mm:
T_both / 0.082 → 500 N (original geometry) / 73 N (ΔS = 60 geometry).

### 6.9 For reference, A
Output sector torque 5 N·m on z = 60: m = 1, b = 8 → ~70 MPa (fails PLA — TODO data); m = 2,
b = 10 → ~14 MPa (plausible). Original geometry 20.8 N·m: m = 3, b = 12 → ~21 MPa.

---

## 7. Top 10 risks and the prototype test for each

| # | Risk | Test on the printed prototype |
|---|---|---|
| 1 | Hand torque much higher than assumed → cable tension felt as heavy resistance | Load cell / spring balance in the reference cable during real pushdowns at 3 stack settings; compare with Section 6.3. |
| 2 | Slot walls wear or spread (high pressure angle) | 500-cycle bench rig with a 5 N·m weight hung on each handle; measure slot width before/after. |
| 3 | Carriage binds on guides (printed tolerances, side load if hands unequal) | Push each handle alone with the other free; measure R0 with one-sided 100 N load. |
| 4 | Cable jumps a pulley when the user lifts fast (spring too weak) | Drop-release test: let the attachment rise at max machine speed 50 times; inspect. |
| 5 | Overtravel jolt (dwell transition too abrupt, or carriage hits its stop) | Run 400 mm and 450 mm strokes; feel/accelerometer on the housing. |
| 6 | Asymmetry from print tolerance (one slot off by 0.3 mm) | Measure both handle angles at 5 stroke points with a protractor jig; spec ≤ 2° difference. |
| 7 | Housing top plate / carabiner eye fails under stack load across layers | Static hang test to 2× max stack (TODO value) with a safety strap. |
| 8 | Reference-cable anchor slips or the cable angle snags the main cable | Mount on the real machine; 100 reps at 3 heights; check anchor position after. |
| 9 | R0 > 10 N once assembled (friction stack-up) | Spring balance on the reference cable, no hand load, at 0/25/50/75/100 % stroke. |
| 10 | Handles snap closed/open if the reference cable detaches under load | Deliberate release at low stack with hands clear; decide if a secondary stop/damper is needed. |

---

## 8. CAD part list (names only) and build order

Parts (one script each in `parts/`):
1. `housing_top` (carabiner eye, fairlead, fixed pulley seat, dead end)
2. `housing_body` (guide-rod seats, pivot bosses, spring anchor, handle exit guards)
3. `housing_cover`
4. `pulley` (×3, same part)
5. `carriage` (two pulley seats, guide bushes, pin seat, spring hook)
6. `handle_arm_slotted_R` and `_L` (mirrored; slot generated from params)
7. `handle_long_arm_R` / `_L` (grip mount; may merge with 6 if it prints)
8. `grip` (×2)
9. `pin_roller_spacer`
10. `end_stop_closed`, `end_stop_open`
11. `cable_dead_end_clamp`
12. `anchor_strap_bracket` (machine-specific, last)
Bought: 3 bearings for pulleys, 1 roller bearing, 2 pivot bolts + bushings, 2 guide rods, M3/M4
screws, extension spring, 1.5 mm cable + ferrules, carabiner.

Build order:
1. `params.py` update (bell-crank geometry, reeving, slot parameters) and `kinematics.py` slot-profile function.
2. `handle_arm_slotted` + `pin_roller_spacer` + a simple test fixture → **first print**, check the pin runs the whole slot by hand.
3. `carriage` + `pulley` + guide rods → bench-test the reeving ratio and spring with a tape measure.
4. `housing_body` / `top` / `cover` → assemble, measure R0 (risk 9).
5. Grips, guards, stops → first on-machine test at low stack (risks 1, 5, 8).
6. Iterate: slot profile, spring, pressure angle.

---

*End of study. Nothing above is validated; the tests in Section 7 are the validation plan.*
