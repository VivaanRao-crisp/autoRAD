"""cad/bench_assembly.py - places the printed parts, the board and the bought hardware, then checks interference.

Run from the repo root:
    python cad/bench_assembly.py            (interference + clearance table at 6 carriage positions)
    python cad/bench_assembly.py --export   (also writes exports/step/assembly_closed.step and assembly_open.step)

Board frame (docs/design_spec.md): x outward, y up, pivot line y = 0, board front face z = 0, board body z -12 .. 0.
Bought hardware is modelled as plain cylinders (params.BENCH_HW_*) - envelopes for the check, not drawings.
A pair is a COLLISION if the two solids share volume; otherwise the smallest distance between them is reported.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cadquery as cq          # noqa: E402
import params as p             # noqa: E402
import kinematics_D as kd      # noqa: E402
import bench_check as bc       # noqa: E402
import bench_parts as bp       # noqa: E402

O = cq.Vector(0, 0, 0)
X, Y, Z = cq.Vector(1, 0, 0), cq.Vector(0, 1, 0), cq.Vector(0, 0, 1)
VOLUME_EPS_MM3 = 1e-3          # shared volume above this = collision
TOUCH_EPS_MM = 1e-4            # distance below this = the two parts touch (by design: stacked faces)
BOLT_D = {"M5": p.D_JOINT_PIN_MM, "M8": p.D_PIVOT_BOLT_MM}


def beta_at(travel_mm):
    """Handle opening [deg] at a carriage travel [mm]."""
    return kd.beta_of_s(travel_mm * p.D_N_REEVE)


def x_cyl(d, x0, x1, y, z):
    """Cylinder of diameter d along x from x0 to x1, axis at (y, z)."""
    return cq.Solid.makeCylinder(d / 2.0, x1 - x0, cq.Vector(x0, y, z), X)


def x_ring(od, bore, x0, x1, y, z):
    """Ring (bearing or flange) with its axis along x."""
    return x_cyl(od, x0, x1, y, z).cut(x_cyl(bore, x0 - 1.0, x1 + 1.0, y, z))


def z_ring(od, bore, z0, z1, x, y):
    """Ring (tube, bearing or flange) with its axis along z."""
    return bp.cyl(od, z0, z1, x, y).cut(bp.cyl(bore, z0 - 1.0, z1 + 1.0, x, y))


# ------------------------------------------------------------------ fixed bodies
def board(L):
    """The board with its two pivot holes, two slots, three M5 holes and the anchor-cord notch."""
    w, t, y0, top = p.BOARD_W_MM, p.BOARD_T_MM, p.BOARD_BOTTOM_Y_MM, L["top"]
    body = bp.box((-w / 2, w / 2), (y0, top), (-t, 0.0))
    ox = kd.pivot_x()
    d8, d5, sw = bp.pin_hole_d(p.D_PIVOT_BOLT_MM), bp.pin_hole_d(p.D_JOINT_PIN_MM), p.BENCH_SLOT_W_MM
    cuts = [bp.cyl(d8, -t - 1, 1, ox, 0.0), bp.cyl(d8, -t - 1, 1, -ox, 0.0),
            bp.cyl(d5, -t - 1, 1, L["xF1"], L["yf"]), bp.cyl(d5, -t - 1, 1, L["xF0"], L["yf"]),
            bp.cyl(d5, -t - 1, 1, L["x2"], p.BENCH_DEAD_END_Y_MM)]
    for s0, s1 in (L["slot1"], L["slot2"]):
        cuts.append(bp.box((-sw / 2, sw / 2), (s0, s1), (-t - 1, 1)))
    n = p.BENCH_ANCHOR_NOTCH_DEPTH_MM
    cuts.append(bp.box((L["x_anchor"] - n / 2, L["x_anchor"] + n / 2), (top - n, top + 1), (-t - 1, 1)))
    return body.cut(*cuts).clean()


def fixed_bodies(L, parts):
    """Board, pivot bolts, left-handle spacer, F1 / F0 / dead-end stacks behind the board."""
    b = {"board": board(L)}
    ox, zb = kd.pivot_x(), L["zb"]
    tubes = bp.tube_table()
    hw, ht = p.BENCH_HW_M8_WASHER_D_MM, p.BENCH_HW_M8_WASHER_T_MM
    hd, hh, nh = p.BENCH_HW_M8_HEAD_D_MM, p.BENCH_HW_M8_HEAD_H_MM, p.BENCH_HW_M8_NUT_H_MM
    for name, x, z_face in (("pivot_R", ox, p.D_STACK["arm_R"][1]), ("pivot_L", -ox, p.D_STACK["arm_L"][1])):
        z_head = z_face + ht                                   # washer, then the hex head, on the front of the handle
        z_end = z_head - p.BENCH_HW_BOLT_LEN_MM[name]
        b[name + "_bolt"] = bp.fuse(bp.cyl(BOLT_D["M8"], z_end, z_head, x, 0.0), bp.cyl(hd, z_head, z_head + hh, x, 0.0),
                                    z_ring(hw, BOLT_D["M8"], z_face, z_head, x, 0.0))
        b[name + "_nut"] = bp.fuse(z_ring(hw, BOLT_D["M8"], zb - ht, zb, x, 0.0), z_ring(hd, BOLT_D["M8"], zb - ht - nh, zb - ht, x, 0.0))
    b["spacer_handleL"] = parts["P4a_spacer_handleL"][0].translate(cq.Vector(-ox, 0.0, 0.0))
    # stacks behind the board, built toward -z
    m5, R, w, ft = BOLT_D["M5"], p.BENCH_PULLEY_D_MM, p.BENCH_PULLEY_W_MM, p.BENCH_FLANGE_T_MM
    nd, nhh = p.BENCH_HW_M5_NUT_D_MM, p.BENCH_HW_M5_NUT_H_MM
    hole = bp.pin_hole_d(m5)
    for name, x, y, key, bearing in (("F1", L["xF1"], L["yf"], "P4c_standoff_F1", True), ("F0", L["xF0"], L["yf"], "P4d_standoff_F0", True),
                                     ("dead_end", L["x2"], p.BENCH_DEAD_END_Y_MM, "P4e_standoff_dead_end", False)):
        length = tubes[key][2]
        z = zb - length
        b[name + "_standoff"] = z_ring(tubes[key][0], tubes[key][1], z, zb, x, y)
        if bearing:
            b[name + "_flange_a"] = z_ring(p.BENCH_FLANGE_D_MM, hole, z - ft, z, x, y)
            b[name + "_bearing"] = z_ring(R, m5, z - ft - w, z - ft, x, y)
            b[name + "_flange_b"] = z_ring(p.BENCH_FLANGE_D_MM, hole, z - 2 * ft - w, z - ft - w, x, y)
            z = z - 2 * ft - w
        b[name + "_nut"] = z_ring(nd, m5, z - nhh, z, x, y)
        # flat head on the FRONT face, drawn proud by the largest height the spec allows
        b[name + "_bolt"] = bp.fuse(bp.cyl(m5, -p.BENCH_HW_BOLT_LEN_MM[name], 0.0, x, y),
                                    bp.cyl(p.BENCH_HW_M5_HEAD_D_MM, 0.0, p.BENCH_HW_M5_HEAD_H_MM, x, y))
    return b


# ------------------------------------------------------------------ moving bodies
def carriage_bodies(L, parts, yc):
    """Carriage with pin bolt, pin spacer, guide bolt, axle, two bearings and four flanges, at carriage pin height yc."""
    b = {}
    zb, t = L["zb"], p.BENCH_CARRIAGE_T_MM
    m5 = BOLT_D["M5"]
    hd, hh = p.BENCH_HW_M5_HEAD_D_MM, p.BENCH_HW_M5_HEAD_H_MM
    nd, nh = p.BENCH_HW_M5_NUT_D_MM, p.BENCH_HW_M5_NUT_H_MM
    wd, wt = p.BENCH_HW_M5_WASHER_D_MM, p.BENCH_HW_M5_WASHER_T_MM
    z_back = zb - t
    b["carriage"] = parts["P3_carriage"][0].translate(cq.Vector(0.0, yc, zb))
    # carriage pin: head behind the plate, through slot 1, spacer, link R, link L, nut in front
    b["carriage_pin_bolt"] = bp.fuse(bp.cyl(m5, z_back, z_back + p.BENCH_HW_BOLT_LEN_MM["carriage_pin"], 0.0, yc),
                                     bp.cyl(hd, z_back - hh, z_back, 0.0, yc))
    b["carriage_pin_nut"] = z_ring(nd, m5, p.D_STACK["link_L"][1], p.D_STACK["link_L"][1] + nh, 0.0, yc)
    b["spacer_carriage_pin"] = parts["P4b_spacer_carriage_pin"][0].translate(cq.Vector(0.0, yc, 0.0))
    # guide bolt 2
    yg = yc + p.BENCH_GUIDE2_ABOVE_PIN_MM
    b["guide_bolt"] = bp.fuse(bp.cyl(m5, z_back, z_back + p.BENCH_HW_BOLT_LEN_MM["guide"], 0.0, yg), bp.cyl(hd, z_back - hh, z_back, 0.0, yg))
    b["guide_nut"] = bp.fuse(z_ring(wd, m5, 0.0, wt, 0.0, yg), z_ring(nd, m5, wt, wt + nh, 0.0, yg))
    # axle along x with the two moving bearings at their nominal lane positions
    ya, za = yc + p.BENCH_AXLE_ABOVE_PIN_MM, L["z_axle"]
    w, ft, hole = p.BENCH_PULLEY_W_MM, p.BENCH_FLANGE_T_MM, bp.pin_hole_d(m5)
    xm = 0.5 * (L["x1"] + L["x2"])
    half = p.BENCH_HW_BOLT_LEN_MM["axle"] / 2.0
    b["axle_bolt"] = x_cyl(m5, xm - half, xm + half, ya, za)
    for name, x in (("M1", L["x1"]), ("M2", L["x2"])):
        b[name + "_bearing"] = x_ring(p.BENCH_PULLEY_D_MM, m5, x - w / 2, x + w / 2, ya, za)
        b[name + "_flange_out"] = x_ring(p.BENCH_FLANGE_D_MM, hole, x - w / 2 - ft, x - w / 2, ya, za)
        b[name + "_flange_in"] = x_ring(p.BENCH_FLANGE_D_MM, hole, x + w / 2, x + w / 2 + ft, ya, za)
    return b


def linkage_bodies(parts, beta, yc):
    """Both handles (with dowels and joint screws) and both links at handle opening beta [deg], carriage pin at yc."""
    b = {}
    ox = kd.pivot_x()
    ax, ay = kd.arm_joint(beta)                                 # right joint, board frame
    h = parts["P1_handle"][0]
    lk = parts["P2_link"][0]
    t = p.D_ARM_THICK_MM
    zR, zL = p.D_STACK["arm_R"], p.D_STACK["arm_L"]
    zlR, zlL = p.D_STACK["link_R"], p.D_STACK["link_L"]

    def as_right(s):
        return s.rotate(O, Z, beta).translate(cq.Vector(ox, 0.0, zR[0]))

    def as_left(s):                                             # the same part turned over (half turn about y)
        return s.rotate(O, Z, beta).rotate(O, Y, 180.0).translate(cq.Vector(-ox, 0.0, zL[1]))

    b["handle_R"], b["handle_L"] = as_right(h), as_left(h)
    # dowels: on the front face of each grip pad
    _joint, grip, u, _v = bp.handle_points()
    gd, gl = p.BENCH_GRIP_D_MM, p.BENCH_GRIP_LEN_MM
    for name, zc, place in (("dowel_R", t + gd / 2.0, as_right), ("dowel_L", -gd / 2.0, as_left)):
        start = cq.Vector(grip[0] - u[0] * gl / 2.0, grip[1] - u[1] * gl / 2.0, zc)
        b[name] = place(cq.Solid.makeCylinder(gd / 2.0, gl, start, cq.Vector(u[0], u[1], 0.0)))
    # links
    ang_R = math.degrees(math.atan2(yc - ay, 0.0 - ax))
    ang_L = math.degrees(math.atan2(yc - ay, 0.0 + ax))
    b["link_R"] = lk.rotate(O, Z, ang_R).translate(cq.Vector(ax, ay, zlR[0]))
    b["link_L"] = lk.rotate(O, Z, ang_L).translate(cq.Vector(-ax, ay, zlL[0]))
    # joint screws: countersunk head flush in the handle, nyloc nut on the link side
    m5, nd, nh = BOLT_D["M5"], p.BENCH_HW_M5_NUT_D_MM, p.BENCH_HW_M5_NUT_H_MM
    ln = p.BENCH_HW_BOLT_LEN_MM["joint"]
    ch = (p.BENCH_HW_M5_CSK_HEAD_D_MM - m5) / 2.0
    b["joint_R_screw"] = bp.fuse(bp.cyl(m5, zR[0], zR[0] + ln, ax, ay),
                                 cq.Solid.makeCone(p.BENCH_HW_M5_CSK_HEAD_D_MM / 2.0, m5 / 2.0, ch, cq.Vector(ax, ay, zR[0]), Z))
    b["joint_R_nut"] = z_ring(nd, m5, zlR[1], zlR[1] + nh, ax, ay)
    b["joint_L_screw"] = bp.fuse(bp.cyl(m5, zL[1] - ln, zL[1], -ax, ay),
                                 cq.Solid.makeCone(m5 / 2.0, p.BENCH_HW_M5_CSK_HEAD_D_MM / 2.0, ch, cq.Vector(-ax, ay, zL[1] - ch), Z))
    b["joint_L_nut"] = z_ring(nd, m5, zlL[0] - nh, zlL[0], -ax, ay)
    return b


# rigid groups: bodies in one group never move against each other
GROUPS = {"handle_R": "hR", "dowel_R": "hR", "handle_L": "hL", "dowel_L": "hL"}


def group_of(name, fixed_names, carriage_names):
    if name in fixed_names:
        return "fixed"
    if name in carriage_names:
        return "carriage"
    return GROUPS.get(name, name)


def assembly(travel_mm, parts=None, L=None):
    """All bodies at a carriage travel [mm]: dict name -> solid, plus the sets of fixed and carriage names."""
    parts = bp.build_all() if parts is None else parts
    L = bc.layout() if L is None else L
    yc = L["yc0"] - travel_mm
    fixed = fixed_bodies(L, parts)
    car = carriage_bodies(L, parts, yc)
    lnk = linkage_bodies(parts, beta_at(travel_mm), yc)
    bodies = {}
    bodies.update(fixed)
    bodies.update(car)
    bodies.update(lnk)
    return bodies, set(fixed), set(car)


# ------------------------------------------------------------------ pair check
def boxes_near(b1, b2, margin):
    """True if two bounding boxes are closer than margin in every axis."""
    return (b1.xmin - margin <= b2.xmax and b2.xmin - margin <= b1.xmax and
            b1.ymin - margin <= b2.ymax and b2.ymin - margin <= b1.ymax and
            b1.zmin - margin <= b2.zmax and b2.zmin - margin <= b1.zmax)


def check_pairs(bodies, fixed_names, carriage_names, first):
    """Yield (a, b, shared volume, distance) for every pair of bodies that can interact at this position."""
    names = list(bodies)
    bbs = {n: bodies[n].BoundingBox() for n in names}
    for i, a in enumerate(names):
        for c in names[i + 1:]:
            same = group_of(a, fixed_names, carriage_names) == group_of(c, fixed_names, carriage_names)
            if same and not first:
                continue                                        # rigid against each other: checked once
            if not boxes_near(bbs[a], bbs[c], p.D_CLEARANCE_MIN_MM):
                continue
            vol = bodies[a].intersect(bodies[c]).Volume()
            dist = 0.0 if vol > VOLUME_EPS_MM3 else bodies[a].distance(bodies[c])
            yield a, c, vol, dist


def run_check():
    """Interference and clearance over the carriage positions. Returns (collisions, tight) as dicts keyed by pair."""
    parts, L = bp.build_all(), bc.layout()
    collisions, tight, touching = {}, {}, set()
    travels = p.BENCH_CHECK_TRAVELS_MM
    for k, tr in enumerate(travels):
        bodies, fx, cr = assembly(tr, parts, L)
        n_pairs = 0
        for a, c, vol, dist in check_pairs(bodies, fx, cr, first=(k == 0)):
            n_pairs += 1
            key = (a, c)
            if vol > VOLUME_EPS_MM3:
                if vol > collisions.get(key, (0.0, 0.0))[0]:
                    collisions[key] = (vol, tr)
            elif dist < TOUCH_EPS_MM:
                touching.add(key)
            elif dist < p.D_CLEARANCE_MIN_MM:
                if dist < tight.get(key, (1e9, 0.0))[0]:
                    tight[key] = (dist, tr)
        print("carriage travel %5.1f mm (cord %3.0f mm, included angle %6.2f deg): %d bodies, %d close pairs examined"
              % (tr, tr * p.D_N_REEVE, p.ANGLE_INCLUDED_CLOSED_DEG + 2 * beta_at(tr), len(bodies), n_pairs))
    print("\nCOLLISIONS (shared volume): %d" % len(collisions))
    for (a, c), (vol, tr) in sorted(collisions.items(), key=lambda kv: -kv[1][0]):
        print("   %-22s x %-22s  %8.2f mm3  worst at carriage travel %.0f" % (a, c, vol, tr))
    print("\nCLEARANCES under %.1f mm (not touching): %d" % (p.D_CLEARANCE_MIN_MM, len(tight)))
    for (a, c), (d, tr) in sorted(tight.items(), key=lambda kv: kv[1][0]):
        print("   %-22s - %-22s  %5.2f mm  at carriage travel %.0f" % (a, c, d, tr))
    print("\nPairs that touch face to face (stacked by design): %d" % len(touching))
    for a, c in sorted(touching):
        print("   %s / %s" % (a, c))
    return collisions, tight


def export_assembly():
    """Write the closed and the open assembly as STEP (every body a named solid)."""
    out = os.path.join(ROOT, "exports", "step")
    os.makedirs(out, exist_ok=True)
    parts, L = bp.build_all(), bc.layout()
    for label, tr in (("closed", p.BENCH_CHECK_TRAVELS_MM[0]), ("open", p.BENCH_CHECK_TRAVELS_MM[-1])):
        bodies, _fx, _cr = assembly(tr, parts, L)
        asm = cq.Assembly(name="autoRAD_bench_" + label)
        for name, solid in bodies.items():
            asm.add(solid, name=name)
        asm.save(os.path.join(out, "assembly_%s.step" % label))
        print("written exports/step/assembly_%s.step (%d bodies)" % (label, len(bodies)))


if __name__ == "__main__":
    run_check()
    if "--export" in sys.argv:
        export_assembly()
