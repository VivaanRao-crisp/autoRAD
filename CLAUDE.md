# autoRAD — Claude Code project context

Course project (DES649, IIT Kanpur). Goal: a **3D-printed functional prototype** of a removable
cable-machine attachment (two-handle V-bar) whose handles open from 50° to 110° included angle
over the first 300 mm of travel, hold 110° up to 400 mm, and close on the way back. No
electronics, no manual adjustment. Driven only by the attachment's travel relative to the frame.

Read first: `docs/main_design.md` (rev 3) and `docs/buildability.md`. Both are first-pass studies;
**nothing is validated.** `kinematics.py` and `params.py` already exist in the repo — reuse them.

## What we are building (decision made)

**Option D + anchor spring** (see `buildability.md` §1–3, verified in `docs/kinematics_D_report.md`
and `docs/design_freeze.md` — **status NOT FROZEN**, see design_freeze.md §1 for the open stop conditions):
- Reeved cable, n = 8 parts. **Crossed axles:** 4 moving pulleys Ø30 on the carriage with the axle
  along X (x = ±15, ±45, axle z 43); 3 fixed pulleys Ø30 with axles along Z in one row at y 84
  (F1 (−30, z 58), F2 (0, z 28), F3 (+30, z 58)); fairlead pulley F0 Ø20 at (−45, z 18) and the dead
  end at (+45, z 28) **at the fixed end, below the carriage**. Every part vertical, zero fleet angle,
  net 8 T down. (The earlier "4 pulleys on one axle = 150 mm" and "2 × 2 in plane" are void.)
- Carriage block 98 mm wide, moves **DOWN** on cable payout; two Ø5 guide rods at x = ±57, z 43;
  return extension spring **above** the carriage at x 0, z 43 (30 N preload, ~0.1 N/mm as bought; TBD).
- Bell-crank drive arms a = 72.75 mm (ψ0 = 40°; 72.05 at 38° proposed) pointing inward, links
  70 mm to the carriage pin on the centre line. Carriage travel 37.5 mm for 30°. Hard stop at 110°.
- **z-stack (CAD Y axis = pivot axis direction here):** arm R 0–8, link R 8.3–14.3, link L 14.6–20.6,
  arm L 20.9–28.9; both long handle arms at the common plane z = 14.45 via 10.45 mm hub extensions.
  Pivot hubs R10 on M8 bolts with 625 bearings; joint pins M5; carriage pin M8.
- Reference cable anchored to the frame through a preloaded spring (>= 50 N, must exceed the 38 N
  loaded tension), spring tube + plunger. **v1 is built WITHOUT the anchor spring; v2 adds it.**
- Grips are BOUGHT (Ø32 tube/dowel × 130). Model only a placeholder cylinder for the assembly.
  **Open:** hand envelope (R35 assumption) overlaps the pivot hub by 12.9 mm at ΔS = 70 — mock-up test
  before parts (design_freeze.md §7).
- Housing: **flat plates + standoffs**, not a shell. Width 149 (inner 143, 130 needed), depth 72
  (70 with 7 mm pulleys), height ≈ 200 from hub bottom to top plate (top plate outer y 191.5, cable-bay
  bottom plate 49–59, front-bay guard down to y +15 / +12). Top plate >= 10 mm, printed flat.
  TODO confirm whether any plate is laser-cut instead.
- E (slotted arms) and C (threaded drum) are **CAD-only alternatives**, modelled later if time
  remains; not printed. Their `params.py` values are marked LEGACY.

## Working rules (important)

1. **Never invent numbers.** Anything not in the docs is `ASSUMPTION` or `TODO` in `params.py`
   with a comment. Test results come only from the user's measurements; never fabricate data.
2. **`params.py` is the single source of truth.** All CAD is generated from it. No magic numbers
   in part scripts.
3. Units mm, N, degrees. Z up. Pivot line = Y 0, centre line = X 0 (as in the docs).
4. Log every design change in `iteration_log.md`: date, what changed, why, which part to reprint.
5. Before printing anything, verify the kinematics (below). Do not generate part geometry from
   unverified link lengths.
6. Keep the repo private; do not upload designs anywhere (patent consultation pending).

## CAD stack and layout

Use **build123d** (fall back to CadQuery). One script per part so a single part can be reprinted.

```
cad/params.py            # shared parameters
cad/parts/<name>.py      # one file per part, builds and exports
cad/assembly.py          # places all parts, interference check
exports/step/  exports/stl/
docs/  iteration_log.md  parts_list.md   # parts_list: name, qty, material, est. print h, orientation
```

## Print and fit rules (FDM, PETG, 0.4 nozzle)

- Draft quality: 0.28 mm layers, 3 walls, 25–40 % infill. No fine features needed in D.
- Clearances: pin holes nominal + 0.3 mm; bearing seats nominal + 0.1 mm (press fit) — expose as
  params; sliding fit for carriage bushes + 0.3 mm. TODO tune after the first coupon.
- Min wall 1.6 mm; avoid overhangs > 45°; orient every part so layers are NOT across its main load
  (links in-plane, plates flat).
- Every part exports STL in print orientation. Record the orientation in `parts_list.md`.
- Bearing size is a PARAMETER (default 625ZZ: 5×16×5). Confirm with the user before printing pulleys.

## Acceptance checks per part

- Builds from `params.py` with no manual edits; exports valid STEP and watertight STL.
- Bounding box matches `parts_list.md`.
- Assembly script reports interference between all parts (zero except intended press fits)
  at closed, 150 mm, 300 mm, and 400 mm of travel.
- Render PNGs of each part and of the assembly (closed and open) into `docs/img/`.

## Deliverables Claude Code feeds (course submission)

CAD files (STEP + scripts), exploded and section views (images; final drawings done separately in
the institute CAD), mechanism animation (GIF/MP4 of 50° → 110° vs. travel), parts list with print
time and material, interference results, kinematic plots, `iteration_log.md`.
