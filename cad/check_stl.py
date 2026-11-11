"""cad/check_stl.py - acceptance check of the exported STL files (the files that go to the slicer).

Run from the repo root (after cad/bench_parts.py):   python cad/check_stl.py

Reads ONLY exports/stl/*.stl and measures them by slicing the mesh. The expected values below are typed
from docs/design_spec.md on purpose, so this check does not share code or parameters with the generator.
Needs trimesh and numpy.
"""
import math
import os
import sys

import numpy as np
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL = os.path.join(ROOT, "exports", "stl")
TOL = 0.05          # [mm] allowed difference between a measured and an expected dimension
ROUND_TOL = 0.03    # [mm] a section loop is a circle if its radius varies less than this
FAILS = []


def report(label, measured, expected, tol=TOL):
    ok = abs(measured - expected) <= tol
    if not ok:
        FAILS.append(label)
    print("   %-58s %9.3f  (spec %8.3f)  %s" % (label, measured, expected, "PASS" if ok else "FAIL"))


def flag(label, ok, detail=""):
    if not ok:
        FAILS.append(label)
    print("   %-58s %s  %s" % (label, "PASS" if ok else "FAIL", detail))


def load(name):
    """Load one STL and check that it is a closed, consistently oriented, single solid."""
    m = trimesh.load_mesh(os.path.join(STL, name + ".stl"))
    print("\n%s.stl  (%d triangles, bbox %.2f x %.2f x %.2f, volume %.2f cm3)"
          % (name, len(m.faces), m.extents[0], m.extents[1], m.extents[2], m.volume / 1000.0))
    flag("watertight, consistent winding, one body, volume > 0",
         m.is_watertight and m.is_winding_consistent and m.body_count == 1 and m.volume > 0)
    flag("sits on the bed (lowest z = 0)", abs(m.bounds[0][2]) < 1e-6)
    return m


def loops(mesh, origin, normal):
    """Closed loops [(n, 3) arrays] where the plane cuts the mesh."""
    sec = mesh.section(plane_origin=origin, plane_normal=normal)
    return [] if sec is None else [np.asarray(d) for d in sec.discrete]


def circles(mesh, origin, normal):
    """Round loops in a section: list of (centre xyz, diameter)."""
    n = np.asarray(normal, float)
    found = []
    for pts in loops(mesh, origin, normal):
        c = pts[:-1].mean(axis=0) if np.allclose(pts[0], pts[-1]) else pts.mean(axis=0)
        r = np.linalg.norm((pts - c) - np.outer((pts - c) @ n, n), axis=1)
        if r.std() < ROUND_TOL and r.mean() < 15.0:
            found.append((c, 2.0 * r.mean()))
    return found


def pick(found, dia, tol=0.1):
    """Circles of one diameter."""
    return [c for c, d in found if abs(d - dia) <= tol]


def dist(a, b):
    return float(np.linalg.norm(np.asarray(a)[:2] - np.asarray(b)[:2]))


def angle_at(o, a, b):
    """Angle [deg] from o->a to o->b, counter-clockwise seen from +z, 0 .. 360."""
    va, vb = np.asarray(a)[:2] - np.asarray(o)[:2], np.asarray(b)[:2] - np.asarray(o)[:2]
    return math.degrees(math.atan2(va[0] * vb[1] - va[1] * vb[0], va @ vb)) % 360.0


def check_handle():
    m = load("P1_handle")
    report("plate thickness", m.extents[2], 8.0)
    mid = circles(m, [0, 0, 4.0], [0, 0, 1])
    pivot, joint, screws = pick(mid, 8.3), pick(mid, 5.3), pick(mid, 4.3)
    flag("holes found: 1 pivot 8.3, 1 joint 5.3, 2 screw 4.3", (len(pivot), len(joint), len(screws)) == (1, 1, 2),
         "found %d / %d / %d" % (len(pivot), len(joint), len(screws)))
    o, j = pivot[0], joint[0]
    grip = 0.5 * (screws[0] + screws[1])
    report("pivot -> joint (ARM_A)", dist(o, j), 125.9)
    report("pivot -> grip centre (L_LINE)", dist(o, grip), 71.09)
    report("screw holes apart (2 x 20)", dist(screws[0], screws[1]), 40.0)
    # the STL lies countersink-down, i.e. it is seen from the FRONT of the right handle: drive arm -> long arm = 273 - 138 = 135 deg CCW
    report("angle drive arm -> long arm, CCW (273 - 138)", angle_at(o, j, grip), 135.0)
    # grip axis vs pivot->grip line: 25 deg from vertical, line at 3 deg from vertical -> 22 deg; top end toward the drive-arm side
    top = screws[0] if dist(screws[0], j) < dist(screws[1], j) else screws[1]
    a = angle_at(grip, o, top)
    report("grip axis vs pivot->grip line (25 - 3)", min(a, 360.0 - a), 22.0, 0.1)
    flag("grip top leans toward the centre line (drive-arm side)", a < 180.0, "CCW angle %.2f" % a)
    hub = [np.linalg.norm(pts[:, :2] - o[:2], axis=1) for pts in loops(m, [0, 0, 4.0], [0, 0, 1])]
    report("hub radius (smallest outline distance from the pivot)", min(r.min() for r in hub if r.min() > 5.0), 12.0)
    csk = pick(circles(m, [0, 0, 0.5], [0, 0, 1]), 11.0 - 2 * 0.5, 0.1)
    flag("countersink on the bed face of the joint hole (dia 10.0 at z 0.5)", len(csk) == 1 and dist(csk[0], j) < 0.02)
    top_hole = pick(circles(m, [0, 0, 7.5], [0, 0, 1]), 5.3)
    flag("joint hole is plain 5.3 at the other face", len(top_hole) == 1)
    print("   bed footprint %.1f x %.1f mm" % (m.extents[0], m.extents[1]))


def check_link():
    m = load("P2_link")
    report("thickness", m.extents[2], 6.0)
    report("overall length (120 + 2 x R8)", max(m.extents[:2]), 136.0)
    report("overall width (2 x R8)", min(m.extents[:2]), 16.0)
    h = pick(circles(m, [0, 0, 3.0], [0, 0, 1]), 5.3)
    flag("two 5.3 holes", len(h) == 2)
    report("hole centre distance (LINK_C)", dist(h[0], h[1]), 120.0)
    mid = 0.5 * (h[0] + h[1])
    axis = (h[1] - h[0])[:2] / dist(h[0], h[1])
    width = [abs((pts[:, :2] - mid[:2]) @ np.array([-axis[1], axis[0]])) for pts in loops(m, [mid[0], mid[1], 3.0], [axis[0], axis[1], 0])]
    report("bar width at mid length", 2 * max(w.max() for w in width), 12.0)


def check_carriage():
    m = load("P3_carriage")
    report("height on the bed (plate 8 + post 32)", m.extents[2], 40.0)
    report("footprint x (bar -32 .. +8)", m.extents[0], 40.0)
    report("footprint y (-10 .. 65)", m.extents[1], 75.0)
    pl = circles(m, [0, 0, 4.0], [0, 0, 1])
    m5, band = pick(pl, 5.3), pick(pl, 4.0)
    flag("plate holes: 2 x 5.3 (pin, guide), 2 x 4.0 (band)", (len(m5), len(band)) == (2, 2), "found %d / %d" % (len(m5), len(band)))
    pin, guide = sorted(m5, key=lambda c: c[1])
    report("pin -> guide hole", dist(pin, guide), 60.0)
    report("pin hole to the plate's bottom edge", pin[1] - m.bounds[0][1], 10.0)
    # on the bed the part is seen from BEHIND the board: +x here = -x in the carriage frame
    report("pin hole to the bar's thin side (frame x = +8)", pin[0] - m.bounds[0][0], 8.0)
    report("pin hole to the far end of the horizontal bar (frame x = -32)", m.bounds[1][0] - pin[0], 32.0)
    post = [pts for pts in loops(m, [0, 0, 20.0], [0, 0, 1])]
    flag("only the post is cut at z = 20", len(post) == 1)
    lo, hi = post[0].min(axis=0), post[0].max(axis=0)
    report("post, frame x -19 (print x from the pin)", hi[0] - pin[0], 19.0)
    report("post, frame x -9", lo[0] - pin[0], 9.0)
    report("post, y 35 above the pin", lo[1] - pin[1], 35.0)
    report("post, y 55 above the pin", hi[1] - pin[1], 55.0)
    flag("post is on frame -x: on the LEFT seen from the front of the board", lo[0] - pin[0] > 0, "(print +x = frame -x)")
    ax = pick(circles(m, [pin[0] + 14.0, 0, 0], [1, 0, 0]), 5.3)
    flag("axle hole 5.3 through the post, along x", len(ax) == 1)
    report("axle hole height above the pin (y 45)", ax[0][1] - pin[1], 45.0)
    report("axle hole behind the plate's back face (24)", ax[0][2] - 8.0, 24.0)
    for label, c, exp in (("upper band hole", max(band, key=lambda c: c[1]), (4.0, 54.5)), ("lower band hole", min(band, key=lambda c: c[1]), (4.0, -5.5))):
        report(label + " frame x", -(c[0] - pin[0]), exp[0])
        report(label + " frame y", c[1] - pin[1], exp[1])
    left_wall = (pin[0] - m.bounds[0][0]) - 5.3 / 2          # toward frame +x
    print("   NOTE wall beside the pin hole: %.2f mm on the frame -x side, %.2f mm on the +x side" % (4.0 - 5.3 / 2, left_wall))


def check_tube(name, od, bore, length):
    m = load(name)
    report("length", m.extents[2], length)
    report("outside diameter", max(m.extents[:2]), od)
    c = pick(circles(m, [0, 0, length / 2.0], [0, 0, 1]), bore)
    flag("bore %.1f, concentric" % bore, len(c) == 1 and dist(c[0], m.bounds.mean(axis=0)) < 0.02)


def main():
    check_handle()
    check_link()
    check_carriage()
    for name, od, bore, length in (("P4a_spacer_handleL", 24.0, 8.3, 20.9), ("P4b_spacer_carriage_pin", 10.0, 5.3, 8.3),
                                   ("P4c_standoff_F1", 10.0, 5.3, 37.5), ("P4d_standoff_F0", 10.0, 5.3, 21.5),
                                   ("P4e_standoff_dead_end", 10.0, 5.3, 23.0), ("P5_flange_washer", 20.0, 5.3, 1.0),
                                   ("optional_plain_pulley", 16.0, 5.3, 7.0)):
        check_tube(name, od, bore, length)
    print("\nRESULT: %s" % ("ALL CHECKS PASS" if not FAILS else "%d FAILED: %s" % (len(FAILS), "; ".join(FAILS))))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
