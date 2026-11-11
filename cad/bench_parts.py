"""cad/bench_parts.py - the 5 printed models of the BENCH RIG (docs/design_spec.md section 2) as solids.

Run from the repo root:   python cad/bench_parts.py        (writes exports/stl, exports/step, parts_list.md)

Every dimension comes from params.py, kinematics_D.py or bench_check.py - no numbers are typed here.
Nothing is validated; only the print and the bench test are.

Frames
  handle   : origin = pivot bore, x outward, y up, RIGHT handle at closed, back face (board side) at z = 0.
  link     : origin = joint hole, second hole on +x, z 0 .. thickness.
  carriage : origin = carriage pin hole, x outward, y up (seen from the FRONT of the board),
             z = 0 is the face that slides on the board's back face, the body is at z < 0.
STEP files keep these design frames. STL files are turned into print orientation (flat, z >= 0).
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import cadquery as cq          # noqa: E402
import params as p             # noqa: E402
import kinematics_D as kd      # noqa: E402
import bench_check as bc       # noqa: E402

Z = cq.Vector(0, 0, 1)
STL_TOL_MM = 0.01              # chord tolerance of the STL facets
STL_ANG_TOL = 0.1              # angular tolerance [rad]


# ------------------------------------------------------------------ small builders
def pin_hole_d(pin_d):
    """Printed hole diameter for a pin: pin + clearance."""
    return pin_d + p.BENCH_PIN_CLEARANCE_MM


def cyl(d, z0, z1, x=0.0, y=0.0):
    """Cylinder of diameter d along z from z0 to z1, axis at (x, y)."""
    return cq.Solid.makeCylinder(d / 2.0, z1 - z0, cq.Vector(x, y, z0), Z)


def prism(points, z0, z1):
    """Closed polygon extruded from z0 to z1."""
    return cq.Workplane("XY").workplane(offset=z0).polyline(points).close().extrude(z1 - z0).val()


def box(x, y, z):
    """Axis-aligned box from three (lo, hi) pairs."""
    return prism([(x[0], y[0]), (x[1], y[0]), (x[1], y[1]), (x[0], y[1])], z[0], z[1])


def capsule(a, b, width, z0, z1):
    """Bar of the given width from point a to point b with full round ends (radius = width / 2)."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    wp = cq.Workplane("XY").workplane(offset=z0).center(0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))
    return wp.slot2D(length + width, width, ang).extrude(z1 - z0).val()


def fuse(*shapes):
    """Union of solids, with coplanar faces merged."""
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def concave_vertical_edges(solid, z_mid, probe=0.3, n=72, limit=0.55):
    """Vertical straight edges that are INSIDE corners: more than `limit` of a small circle around
    the edge is inside the solid (a convex corner gives < 0.5, a tangent seam exactly 0.5)."""
    found = []
    for e in solid.Edges():
        if e.geomType() != "LINE":
            continue
        a, b = e.startPoint(), e.endPoint()
        if abs(a.x - b.x) > 1e-6 or abs(a.y - b.y) > 1e-6:
            continue
        inside = sum(solid.isInside(cq.Vector(a.x + probe * math.cos(2 * math.pi * i / n),
                                              a.y + probe * math.sin(2 * math.pi * i / n), z_mid)) for i in range(n))
        if inside / n > limit:
            found.append(e)
    return found


# ------------------------------------------------------------------ handle geometry (points, right handle, closed)
def handle_points():
    """Key points of the right handle in the handle frame [mm]: joint, grip centre, pad axis u, pad normal v."""
    ang = math.radians(180.0 - p.D_ARM_ANGLE_CLOSED_DEG)
    joint = (p.D_ARM_MM * math.cos(ang), p.D_ARM_MM * math.sin(ang))
    th = math.radians(p.THETA_CLOSED_DEG)
    grip = (p.L_LINE_MM * math.sin(th), -p.L_LINE_MM * math.cos(th))
    tube = math.radians(p.ANGLE_INCLUDED_CLOSED_DEG / 2.0)          # grip axis from vertical, top toward the centre line
    u = (-math.sin(tube), math.cos(tube))
    v = (math.cos(tube), math.sin(tube))
    return joint, grip, u, v


def pad_point(grip, u, v, along, across):
    """Point on the grip pad: `along` the pad axis and `across` it, from the grip centre."""
    return (grip[0] + along * u[0] + across * v[0], grip[1] + along * u[1] + across * v[1])


# ------------------------------------------------------------------ P1 handle
def handle():
    """P1: one-piece bell-crank handle plate (hub + drive arm + long arm + grip pad). Right handle;
    the left handle is the SAME part turned over. Returns (solid, number of filleted inside corners)."""
    t = p.D_ARM_THICK_MM
    w = p.D_ARM_WIDTH_MM
    assert abs(w / 2.0 - p.D_JOINT_BOSS_RADIUS_MM) < 1e-9, "joint boss radius must equal half the arm width"
    joint, grip, u, v = handle_points()
    hl, hw = p.BENCH_GRIP_PAD_L_MM / 2.0, p.BENCH_GRIP_PAD_W_MM / 2.0
    pad = prism([pad_point(grip, u, v, s1 * hl, s2 * hw) for s1, s2 in ((1, 1), (1, -1), (-1, -1), (-1, 1))], 0.0, t)
    body = fuse(cyl(2 * p.D_HUB_RADIUS_MM, 0.0, t), capsule((0, 0), joint, w, 0.0, t), capsule((0, 0), grip, w, 0.0, t), pad)
    corners = concave_vertical_edges(body, t / 2.0)
    body = body.fillet(p.BENCH_HANDLE_FILLET_R_MM, corners)
    # bores
    d_joint = pin_hole_d(p.D_JOINT_PIN_MM)
    cuts = [cyl(pin_hole_d(p.D_PIVOT_BOLT_MM), -1.0, t + 1.0),
            cyl(d_joint, -1.0, t + 1.0, joint[0], joint[1])]
    for s in (+1, -1):
        hx, hy = pad_point(grip, u, v, s * p.BENCH_GRIP_SCREW_OFFSET_MM, 0.0)
        cuts.append(cyl(p.BENCH_GRIP_SCREW_HOLE_D_MM, -1.0, t + 1.0, hx, hy))
    # countersink on the back face (z = 0) of the joint hole, so the joint screw head is flush with the board side
    r_top = p.BENCH_JOINT_CSK_D_MM / 2.0
    depth = (r_top - d_joint / 2.0) / math.tan(math.radians(p.BENCH_JOINT_CSK_ANGLE_DEG / 2.0))
    cuts.append(cq.Solid.makeCone(r_top, d_joint / 2.0, depth, cq.Vector(joint[0], joint[1], 0.0), Z))
    return body.cut(*cuts).clean(), len(corners)


# ------------------------------------------------------------------ P2 link
def link():
    """P2: link bar with two end bosses and two pin holes."""
    c, w, t, r = p.D_LINK_MM, p.D_LINK_WIDTH_MM, p.D_LINK_THICK_MM, p.D_JOINT_BOSS_RADIUS_MM
    body = fuse(box((0.0, c), (-w / 2.0, w / 2.0), (0.0, t)), cyl(2 * r, 0.0, t), cyl(2 * r, 0.0, t, c, 0.0))
    d = pin_hole_d(p.D_JOINT_PIN_MM)
    return body.cut(cyl(d, -1.0, t + 1.0), cyl(d, -1.0, t + 1.0, c, 0.0)).clean()


# ------------------------------------------------------------------ P3 carriage
def carriage():
    """P3: carriage plate (vertical bar + horizontal bar) with the axle post, in the carriage frame.
    The post's outline is part of the plate and continues BENCH_POST_DEPTH_MM behind its back face."""
    L = bc.layout()
    rects = bc.carriage_outline(L)
    t = p.BENCH_CARRIAGE_T_MM
    post_key = [k for k in rects if k.startswith("post")][0]
    z_plate = (-t, 0.0)
    z_post = (-t - p.BENCH_POST_DEPTH_MM, 0.0)
    body = fuse(box(rects["vertical bar"]["x"], rects["vertical bar"]["y"], z_plate),
                box(rects["horizontal bar"]["x"], rects["horizontal bar"]["y"], z_plate),
                box(rects[post_key]["x"], rects[post_key]["y"], z_post))
    d = pin_hole_d(p.D_CARRIAGE_PIN_MM)
    cuts = [cyl(d, -t - 1.0, 1.0),                                         # carriage pin
            cyl(d, -t - 1.0, 1.0, 0.0, p.BENCH_GUIDE2_ABOVE_PIN_MM)]       # guide bolt 2
    for hx, hy in (p.BENCH_BAND_HOLE_UPPER_XY_MM, p.BENCH_BAND_HOLE_LOWER_XY_MM):
        cuts.append(cyl(p.BENCH_BAND_HOLE_D_MM, -t - 1.0, 1.0, hx, hy))
    # axle hole through the post, along x
    px = rects[post_key]["x"]
    z_axle = L["z_axle"] - L["zb"]                                         # board z -> carriage z
    assert abs(z_axle + t + p.BENCH_AXLE_BEHIND_PLATE_MM) < 1e-9
    cuts.append(cq.Solid.makeCylinder(d / 2.0, px[1] - px[0] + 2.0, cq.Vector(px[0] - 1.0, p.BENCH_AXLE_ABOVE_PIN_MM, z_axle),
                                      cq.Vector(1, 0, 0)))
    return body.cut(*cuts).clean()


# ------------------------------------------------------------------ P4 tubes, P5 flange washer
def tube(od, bore, length):
    """P4 / P5: plain tube standing on its end face."""
    return cyl(od, 0.0, length).cut(cyl(bore, -1.0, length + 1.0)).clean()


def tube_table():
    """The tube family: name -> (outside dia, bore, length, quantity, use). Lengths as in bench_check.print_layout."""
    L = bc.layout()
    m5, m8 = pin_hole_d(p.D_JOINT_PIN_MM), pin_hole_d(p.D_PIVOT_BOLT_MM)
    so = p.BENCH_STANDOFF_OD_MM
    half_w = p.BENCH_PULLEY_W_MM / 2.0
    return {
        "P4a_spacer_handleL": (p.BENCH_SPACER_OD_MM, m8, p.D_STACK["arm_L"][0], 1, "board face -> left handle, on pivot L"),
        "P4b_spacer_carriage_pin": (so, m5, p.D_STACK["link_R"][0], 1, "board face -> link R, on the carriage pin"),
        "P4c_standoff_F1": (so, m5, -L["z_far"] - p.BOARD_T_MM - half_w, 1, "board back -> fixed bearing F1"),
        "P4d_standoff_F0": (so, m5, -L["z_near"] - p.BOARD_T_MM - half_w, 1, "board back -> fairlead bearing F0"),
        "P4e_standoff_dead_end": (so, m5, -L["z_near"] - p.BOARD_T_MM - p.BENCH_DEAD_END_CORD_GAP_MM, 1, "board back -> dead-end nut"),
        "P5_flange_washer": (p.BENCH_FLANGE_D_MM, m5, p.BENCH_FLANGE_T_MM, 6, "one each side of bearings M1, M2, F1"),
        "optional_plain_pulley": (p.BENCH_PULLEY_D_MM, m5, p.PULLEY_WIDTH_MM, 0, "ONLY if 625ZZ bearings cannot be bought (buy_list.md)"),
    }


# ------------------------------------------------------------------ print orientation
def to_bed(shape):
    """Move a shape so its lowest point is z = 0 and its bounding box starts at x = y = 0."""
    bb = shape.BoundingBox()
    return shape.translate(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin))


def smallest_square_angle(shape, step_deg=0.5):
    """In-plane rotation [deg] that makes the larger side of the XY bounding box smallest (fits the smallest bed)."""
    pts = [(v.X, v.Y) for v in shape.Vertices()]
    for e in shape.Edges():                                   # sample curved edges too
        pts += [(q.x, q.y) for q in (e.positionAt(i / 16.0) for i in range(17))]
    best = (1e9, 0.0)
    a = 0.0
    while a < 180.0:
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        xs = [x * c - y * s for x, y in pts]
        ys = [x * s + y * c for x, y in pts]
        side = max(max(xs) - min(xs), max(ys) - min(ys))
        if side < best[0] - 1e-9:
            best = (side, a)
        a += step_deg
    return best[1]


def print_orientation(name, shape):
    """Shape turned as it lies on the printer bed. Returns (shape, note)."""
    if name == "P1_handle":
        ang = smallest_square_angle(shape)
        return to_bed(shape.rotate(cq.Vector(0, 0, 0), Z, ang)), "flat, countersink face DOWN, turned %.1f deg in plane to fit the smallest bed" % ang
    if name == "P3_carriage":
        # half turn about y: the face that slides on the board lies on the bed, the post stands up (a rotation, not a mirror)
        return to_bed(shape.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 180.0)), "plate flat on its board-side face, post standing up"
    if name == "P2_link":
        return to_bed(shape), "flat"
    return to_bed(shape), "standing on an end face"


# ------------------------------------------------------------------ build + export
def build_all():
    """All printed models in their design frames: name -> (solid, quantity, description)."""
    h, n_fillets = handle()
    parts = {
        "P1_handle": (h, 2, "one-piece handle; print 2 identical, turn one over for the left (%d inside corners filleted)" % n_fillets),
        "P2_link": (link(), 2, "link, %.1f mm between hole centres" % p.D_LINK_MM),
        "P3_carriage": (carriage(), 1, "carriage plate with axle post"),
    }
    for name, (od, bore, length, qty, use) in tube_table().items():
        parts[name] = (tube(od, bore, length), qty, "OD %.1f, bore %.1f, length %.1f - %s" % (od, bore, length, use))
    return parts


def export_all(parts, out_root=ROOT):
    """Write exports/step/<name>.step (design frame) and exports/stl/<name>.stl (print orientation). Returns rows."""
    stl_dir, step_dir = os.path.join(out_root, "exports", "stl"), os.path.join(out_root, "exports", "step")
    os.makedirs(stl_dir, exist_ok=True)
    os.makedirs(step_dir, exist_ok=True)
    rows = []
    for name, (solid, qty, desc) in parts.items():
        printed, note = print_orientation(name, solid)
        cq.exporters.export(cq.Workplane(obj=solid), os.path.join(step_dir, name + ".step"))
        cq.exporters.export(cq.Workplane(obj=printed), os.path.join(stl_dir, name + ".stl"),
                            tolerance=STL_TOL_MM, angularTolerance=STL_ANG_TOL)
        bb = printed.BoundingBox()
        rows.append(dict(name=name, qty=qty, desc=desc, note=note, bbox=(bb.xlen, bb.ylen, bb.zlen), volume=solid.Volume()))
    return rows


def main():
    parts = build_all()
    rows = export_all(parts)
    print("name                      | qty | bbox in print orientation [mm] | volume [cm3] | orientation")
    for r in rows:
        print("%-25s | %3d | %6.1f x %6.1f x %5.1f       | %7.2f      | %s"
              % (r["name"], r["qty"], r["bbox"][0], r["bbox"][1], r["bbox"][2], r["volume"] / 1000.0, r["note"]))
    return rows


if __name__ == "__main__":
    main()
