# autoRAD bench rig — printed parts list

Generated 2026-11-10 by `cad/bench_parts.py` from `params.py` (values of `docs/design_spec.md` §2).
STL = print orientation (already lying on the bed, z up). STEP = design frame, for import into Fusion / Inventor.
**Nothing is validated.** Print times are the estimates from `design_spec.md` (0.28 mm layers), not measured.

| File (`exports/stl/`) | Print qty | Size on the bed [mm] | Volume [cm³] | Orientation | Est. time each |
|---|---|---|---|---|---|
| `P1_handle.stl` | 2 | 166.9 × 167.2 × 8.0 | 34.05 | flat, countersink face down | ~2.7 h |
| `P2_link.stl` | 2 | 136.0 × 16.0 × 6.0 | 9.76 | flat | ~0.6 h |
| `P3_carriage.stl` | 1 | 40.0 × 75.0 × 40.0 | 16.67 | plate flat on its board-side face, post standing up | ~1.3 h |
| `P4a_spacer_handleL.stl` | 1 | Ø24 × 20.9, bore 8.3 | 8.32 | standing | — |
| `P4b_spacer_carriage_pin.stl` | 1 | Ø10 × 8.3, bore 5.3 | 0.47 | standing | — |
| `P4c_standoff_F1.stl` | 1 (+1 spare) | Ø10 × 37.5, bore 5.3 | 2.12 | standing | — |
| `P4d_standoff_F0.stl` | 1 (+1 spare) | Ø10 × 21.5, bore 5.3 | 1.21 | standing | — |
| `P4e_standoff_dead_end.stl` | 1 | Ø10 × 23.0, bore 5.3 | 1.30 | standing | — |
| `P5_flange_washer.stl` | 6 (+spares) | Ø20 × 1.0, bore 5.3 | 0.29 | flat | — |
| `optional_plain_pulley.stl` | 0 (3 only if no 625ZZ bearings) | Ø16 × 7.0, bore 5.3 | 1.25 | standing | — |

Total printed volume for one rig (no spares): about 119 cm³ solid-equivalent. Material PETG or PLA.

Notes
- **The left handle is the same part as the right one, turned over.** Print `P1_handle.stl` twice; do not mirror it in the slicer.
- The handle needs a bed of at least 170 × 170 mm. It is already turned 13.5° in plane to fit the smallest square.
- The carriage is handed (the post is on the left when you look at the front of the board). Do not mirror it.
- No supports needed. The axle hole in the carriage post is a Ø5.3 horizontal hole; it bridges without support.
- Slicer: 0.28 mm layers, 3 walls, 25–40 % infill (CLAUDE.md). Print one link first and try an M5 bolt in it before the rest.

Regenerate and re-check after any change to `params.py`:

```
python cad/bench_parts.py        # exports/stl, exports/step
python cad/check_stl.py          # measures the STL files against design_spec.md
python cad/bench_assembly.py     # interference at 6 carriage positions (--export writes the assembly STEP)
python cad/render_bench.py       # docs/img/bench_*.png
```
