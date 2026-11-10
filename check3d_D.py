"""check3d_D.py - 3D clearance checks for architecture D (pins, plates, bosses, grips, cable lanes).

Run:  python3 check3d_D.py
Reads params.py and the verified link geometry from kinematics_D.py. Writes nothing.

Bodies are modelled as:
  plate  = 2D capsule (segment + half-width) plus end bosses (circles), extruded over a z-range
  pin    = circle (radius) extruded over a z-range (axis along z)
3D clearance between two such bodies:
  dz  = gap between the z-ranges (0 if they overlap)
  dxy = 2D distance between the shapes minus radii / half-widths (negative = overlap in plan)
  clearance = dxy if dz == 0 (they share z: plan distance is the real one)
            = sqrt(max(dxy,0)^2 + dz^2) otherwise  (if dxy < 0 the z gap alone separates them)
A negative clearance is a collision in the model.
"""
import math
import params as p
import kinematics_D as kd


# ------------------------------------------------------------------ 2D helpers
def seg_seg_dist(a, b, c, d, n=120):
    """Min distance between segments a-b and c-d (sampled; fine for clearance tables)."""
    best = 1e9
    for i in range(n + 1):
        px, py = a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n
        best = min(best, kd.point_segment_dist(px, py, c[0], c[1], d[0], d[1]))
    return best


def z_gap(z1, z2):
    """Gap between z-ranges (lo, hi); 0 if they overlap."""
    return max(0.0, max(z1[0], z2[0]) - min(z1[1], z2[1]))


def combine(dxy, dz):
    """3D clearance from plan clearance and z gap (see module docstring)."""
    if dz == 0.0:
        return dxy
    return math.hypot(max(dxy, 0.0), dz)


# ------------------------------------------------------------------ bodies at a given carriage travel
def beta_for_travel(travel_mm):
    """Handle opening [deg] for a carriage travel [mm] (travel = s / n)."""
    return kd.beta_of_s(travel_mm * p.D_N_REEVE)


def bodies(travel_mm, stack):
    """Dict of bodies at a carriage travel. stack = dict name -> (z_lo, z_hi) for arm_R, link_R, link_L, arm_L.
    Pins span the planes they connect plus D_PIN_OVERHANG_MM each side."""
    beta = beta_for_travel(travel_mm)
    ox = kd.pivot_x()
    ax, ay = kd.arm_joint(beta)
    yc = kd.pin_y(beta)
    oh = p.D_PIN_OVERHANG_MM
    hub_r = p.D_HUB_RADIUS_MM
    jb = p.D_JOINT_BOSS_RADIUS_MM
    b = {}
    # plates: ("seg", a, b, half_w, bosses[(centre, r)], z)
    b["arm_R"] = ("seg", (ox, 0.0), (ax, ay), p.D_ARM_WIDTH_MM / 2, [((ox, 0.0), hub_r), ((ax, ay), jb)], stack["arm_R"])
    b["arm_L"] = ("seg", (-ox, 0.0), (-ax, ay), p.D_ARM_WIDTH_MM / 2, [((-ox, 0.0), hub_r), ((-ax, ay), jb)], stack["arm_L"])
    b["link_R"] = ("seg", (ax, ay), (0.0, yc), p.D_LINK_WIDTH_MM / 2, [((ax, ay), jb), ((0.0, yc), jb)], stack["link_R"])
    b["link_L"] = ("seg", (-ax, ay), (0.0, yc), p.D_LINK_WIDTH_MM / 2, [((-ax, ay), jb), ((0.0, yc), jb)], stack["link_L"])
    # pins: ("pin", centre, r, z)
    zall = (min(v[0] for v in stack.values()) - oh, max(v[1] for v in stack.values()) + oh)
    b["pivot_R"] = ("pin", (ox, 0.0), p.D_PIVOT_BOLT_MM / 2, zall)
    b["pivot_L"] = ("pin", (-ox, 0.0), p.D_PIVOT_BOLT_MM / 2, zall)
    zr = (min(stack["arm_R"][0], stack["link_R"][0]) - oh, max(stack["arm_R"][1], stack["link_R"][1]) + oh)
    zl = (min(stack["arm_L"][0], stack["link_L"][0]) - oh, max(stack["arm_L"][1], stack["link_L"][1]) + oh)
    b["joint_R"] = ("pin", (ax, ay), p.D_JOINT_PIN_MM / 2, zr)
    b["joint_L"] = ("pin", (-ax, ay), p.D_JOINT_PIN_MM / 2, zl)
    b["carriage_pin"] = ("pin", (0.0, yc), p.D_CARRIAGE_PIN_MM / 2, zall)
    return b


def plan_clearance(b1, b2):
    """Plan (2D) clearance between two bodies, negative = overlap."""
    if b1[0] == "pin" and b2[0] == "pin":
        return math.hypot(b1[1][0] - b2[1][0], b1[1][1] - b2[1][1]) - b1[2] - b2[2]
    if b1[0] == "seg" and b2[0] == "pin":
        d = kd.point_segment_dist(b2[1][0], b2[1][1], b1[1][0], b1[1][1], b1[2][0], b1[2][1]) - b1[3] - b2[2]
        for (cx, cy), r in b1[4]:
            d = min(d, math.hypot(cx - b2[1][0], cy - b2[1][1]) - r - b2[2])
        return d
    if b1[0] == "pin":
        return plan_clearance(b2, b1)
    # seg vs seg
    d = seg_seg_dist(b1[1], b1[2], b2[1], b2[2]) - b1[3] - b2[3]
    for (cx, cy), r in b1[4]:
        d = min(d, kd.point_segment_dist(cx, cy, b2[1][0], b2[1][1], b2[2][0], b2[2][1]) - r - b2[3])
    for (cx, cy), r in b2[4]:
        d = min(d, kd.point_segment_dist(cx, cy, b1[1][0], b1[1][1], b1[2][0], b1[2][1]) - r - b1[3])
    for (c1, r1) in b1[4]:
        for (c2, r2) in b2[4]:
            d = min(d, math.hypot(c1[0] - c2[0], c1[1] - c2[1]) - r1 - r2)
    return d


# pairs that are joints by design (touch on purpose) and are skipped
JOINED = {("arm_R", "pivot_R"), ("arm_L", "pivot_L"), ("arm_R", "joint_R"), ("link_R", "joint_R"),
          ("arm_L", "joint_L"), ("link_L", "joint_L"), ("link_R", "carriage_pin"), ("link_L", "carriage_pin")}


def clearance_table(stack, label):
    """Print the min 3D clearance of every non-joined pair at the 5 carriage positions."""
    names = ["arm_R", "arm_L", "link_R", "link_L", "pivot_R", "pivot_L", "joint_R", "joint_L", "carriage_pin"]
    pairs = [(a, c) for i, a in enumerate(names) for c in names[i + 1:] if (a, c) not in JOINED and (c, a) not in JOINED]
    travels = p.D_CHECK_TRAVELS_MM
    print("\n%s" % label)
    print("   stack (z lo-hi):  " + ", ".join("%s %.1f-%.1f" % (k, v[0], v[1]) for k, v in stack.items()))
    print("   pair                       | " + " | ".join("x=%5.2f" % t for t in travels) + " |  min  | note")
    worst = []
    for a, c in pairs:
        row = []
        for t in travels:
            b = bodies(t, stack)
            dxy = plan_clearance(b[a], b[c])
            dz = z_gap(b[a][-1], b[c][-1])
            row.append(combine(dxy, dz))
        m = min(row)
        note = "COLLISION" if m < 0 else ("tight" if m < p.D_CLEARANCE_MIN_MM else "")
        if m < p.D_CLEARANCE_MIN_MM:
            worst.append((a, c, m))
        print("   %-26s | " % (a + " / " + c) + " | ".join("%7.1f" % v for v in row) + " | %5.1f | %s" % (m, note))
    return worst


# ------------------------------------------------------------------ grip tube and hand vs housing
def grip_tube(beta_deg):
    """Grip tube axis segment (top end, bottom end) [mm] for the right handle and the hand portion."""
    ox = kd.pivot_x()
    th = math.radians(p.THETA_CLOSED_DEG + beta_deg)
    cx, cy = ox + p.L_LINE_MM * math.sin(th), -p.L_LINE_MM * math.cos(th)
    tube = math.radians(p.ANGLE_INCLUDED_CLOSED_DEG / 2 + beta_deg)      # tube angle from vertical
    ux, uy = -math.sin(tube), math.cos(tube)                               # along the tube, upward-inboard
    half = p.D_GRIP_LENGTH_MM / 2
    hh = p.HAND_BREADTH_P95_MALE_MM / 2
    return ((cx + ux * half, cy + uy * half), (cx - ux * half, cy - uy * half),
            (cx + ux * hh, cy + uy * hh), (cx - ux * hh, cy - uy * hh))


def grip_checks():
    """Clearances of the grip tube and the hand envelope against hubs and the housing bottom, closed and open."""
    print("\nGrip tube (dia %.0f x %.0f) and hand envelope (R%.0f around the tube over the hand breadth, ASSUMPTION) vs hub and housing"
          % (p.GRIP_DIAMETER_MM, p.D_GRIP_LENGTH_MM, p.HAND_ENVELOPE_RADIUS_MM))
    ox = kd.pivot_x()
    rt = p.GRIP_DIAMETER_MM / 2
    print("   case   | hub R | tube top (x,y) | tube->own hub | tube->other hub | hand->own hub | hand->housing bottom y=%+.0f | tube->bottom"
          % p.D_HOUSING_BOTTOM_Y_MM)
    for label, beta in (("closed", 0.0), ("open", p.ANGLE_PER_HANDLE_DEG)):
        top, bot, htop, hbot = grip_tube(beta)
        for hub in (p.PIVOT_BOSS_MM, p.D_HUB_RADIUS_MM):
            d_own = kd.point_segment_dist(ox, 0, top[0], top[1], bot[0], bot[1]) - rt - hub
            d_oth = kd.point_segment_dist(-ox, 0, top[0], top[1], bot[0], bot[1]) - rt - hub
            h_own = kd.point_segment_dist(ox, 0, htop[0], htop[1], hbot[0], hbot[1]) - p.HAND_ENVELOPE_RADIUS_MM - hub
            h_bot = p.D_HOUSING_BOTTOM_Y_MM - (max(htop[1], hbot[1]) + p.HAND_ENVELOPE_RADIUS_MM)
            t_bot = p.D_HOUSING_BOTTOM_Y_MM - (top[1] + rt)
            print("   %-6s | %5.0f | (%5.1f,%6.1f) | %13.1f | %15.1f | %13.1f | %27.1f | %11.1f"
                  % (label, hub, top[0], top[1], d_own, d_oth, h_own, h_bot, t_bot))


# ------------------------------------------------------------------ reeving lanes, forces, y-layout
def lane_layout():
    """Pulley positions and the 8 cable parts with their (x, z) lanes and force direction on the carriage.

    Layout (ASSUMPTION, checked here): moving pulleys M1..M4 on the carriage axle along X (planes y-z),
    fixed pulleys F1..F3 with axles along Z (planes x-y) in ONE row BELOW the carriage, fairlead pulley F0
    (axle along X, dia D_FAIRLEAD_PULLEY_DIAMETER_MM) and the dead end also at the fixed-pulley end.
    Every part runs vertically between the carriage and the fixed end, so each part pulls the carriage DOWN.
    """
    Rf = p.D_FIXED_PULLEY_DIAMETER_MM / 2
    Rm = p.D_MOVING_PULLEY_DIAMETER_MM / 2
    R0 = p.D_FAIRLEAD_PULLEY_DIAMETER_MM / 2
    zm = p.D_CABLE_Z_CENTRE_MM
    zf, zb = zm - Rm, zm + Rm                                   # front / back lane planes
    xs = [-3 * Rf, -Rf, Rf, 3 * Rf]                             # M1..M4
    fixed = {"F1": (-2 * Rf, zb), "F2": (0.0, zf), "F3": (2 * Rf, zb)}   # (x, z), axle along z
    f0 = (xs[0], zf - R0)                                       # F0 axle (x, z): up-lane at zf, down-lane at zf - 2 R0
    parts = [("1 F0 -> M1", xs[0], zf), ("2 M1 -> F1", xs[0], zb), ("3 F1 -> M2", xs[1], zb),
             ("4 M2 -> F2", xs[1], zf), ("5 F2 -> M3", xs[2], zf), ("6 M3 -> F3", xs[2], zb),
             ("7 F3 -> M4", xs[3], zb), ("8 M4 -> dead end", xs[3], zf)]
    return xs, fixed, f0, parts, (zf, zb)


def reeving_lanes():
    """Print the lanes, the force direction of every part, and the pulley-body check."""
    Rf = p.D_FIXED_PULLEY_DIAMETER_MM / 2
    Rm = p.D_MOVING_PULLEY_DIAMETER_MM / 2
    R0 = p.D_FAIRLEAD_PULLEY_DIAMETER_MM / 2
    w = p.PULLEY_WIDTH_MM
    zm = p.D_CABLE_Z_CENTRE_MM
    xs, fixed, f0, parts, (zf, zb) = lane_layout()
    print("\nReeving: moving M1..M4 dia %.0f (axle along x) at x = %s, axle z = %.0f; fixed F1..F3 dia %.0f (axle along z) at (x, z) = %s;"
          % (2 * Rm, xs, zm, 2 * Rf, fixed))
    print("   fairlead pulley F0 dia %.0f (axle along x) at (x %.0f, z %.0f), anchor cable comes DOWN at (x %.0f, z %.0f) in front of the stack;"
          % (2 * R0, f0[0], f0[1], f0[0], zf - 2 * R0))
    print("   F0, F1..F3 and the dead end are all at the FIXED-PULLEY END (below the carriage). Fairlead hole in the top plate at (x %.0f, z %.0f)." % (f0[0], zf - 2 * R0))
    print("   part               |   x   |   z   | runs between                | force on carriage | through a pulley body?")
    ok_all, n_down = True, 0
    for name, x, z in parts:
        hits = []
        for i, mx in enumerate(xs):
            if abs(x - mx) < w / 2 - 1e-9 and zm - Rm < z < zm + Rm:
                hits.append("M%d" % (i + 1))
        for k, (fx, fz) in fixed.items():
            if fx - Rf < x < fx + Rf and abs(z - fz) < w / 2 - 1e-9:
                hits.append(k)
        if abs(x - f0[0]) < w / 2 - 1e-9 and f0[1] - R0 < z < f0[1] + R0 and z != zf:
            hits.append("F0")
        ok_all = ok_all and not hits
        n_down += 1
        print("   %-18s | %5.0f | %5.0f | carriage <-> fixed end       | T DOWN            | %s" % (name, x, z, ", ".join(hits) if hits else "no"))
    lanes = {(x, z) for _, x, z in parts}
    print("   net force on carriage = %d x T DOWN -> %s; distinct lanes %d/8 -> %s; no cable through a pulley body -> %s"
          % (n_down, "PASS" if n_down == 8 else "FAIL", len(lanes), "PASS" if len(lanes) == 8 else "FAIL", "PASS" if ok_all else "FAIL"))
    return xs, fixed, f0, (zf, zb)


def y_layout():
    """y of the moving axle (closed / stop), the fixed row, the pulley-body gap at the stop, and the clearances
    of the fixed pulleys to the arm L plate, the links and the carriage pin. Returns a dict."""
    Rf = p.D_FIXED_PULLEY_DIAMETER_MM / 2
    Rm = p.D_MOVING_PULLEY_DIAMETER_MM / 2
    yc0, yc1 = kd.pin_y(0.0), kd.pin_y(p.ANGLE_PER_HANDLE_DEG)
    ym0, ym1 = yc0 + p.D_MOVING_AXLE_ABOVE_PIN_MM, yc1 + p.D_MOVING_AXLE_ABOVE_PIN_MM
    yf = ym1 - Rm - p.D_PULLEY_GAP_AT_STOP_MM - Rf
    gap_stop = (ym1 - Rm) - (yf + Rf)
    xs, fixed, f0, _parts, (zf, zb) = lane_layout()
    w = p.PULLEY_WIDTH_MM
    # arm L top edge near x = 0 (arm L shares z with F2 if their z-ranges overlap)
    beta = 0.0
    ax, ay = kd.arm_joint(beta)
    ox = kd.pivot_x()
    f2 = fixed["F2"]
    d_f2_armL = kd.point_segment_dist(f2[0], yf, -ox, 0.0, -ax, ay) - Rf - p.D_ARM_WIDTH_MM / 2
    zgap_f2_armL = c_zgap((f2[1] - w / 2, f2[1] + w / 2), p.D_STACK["arm_L"])
    cl_f2_armL = combine(d_f2_armL, zgap_f2_armL)
    # F2 vs links (z) and vs carriage pin (z, with the back overhang)
    zgap_f2_links = c_zgap((f2[1] - w / 2, f2[1] + w / 2), (p.D_STACK["link_R"][0], p.D_STACK["link_L"][1]))
    zgap_f2_pin = c_zgap((f2[1] - w / 2, f2[1] + w / 2), (p.D_STACK["link_R"][0] - p.D_PIN_OVERHANG_MM, p.D_STACK["link_L"][1] + p.D_CARRIAGE_PIN_BACK_OVERHANG_MM))
    # F0 (front) vs hub L (z-extended to the grip plane) and vs arm R joint boss
    zgap_f0_hubL = c_zgap((f0[1] - 10.0 - 0.75, f0[1] + 10.0), (p.D_GRIP_PLANE_Z_MM - p.D_GRIP_SPACER_L_MM, p.D_STACK["arm_L"][1]))
    top = ym0 + Rm + p.TOP_CLEARANCE_MM + p.D_HOUSING_PLATE_MM
    bottom_inner = yf - Rf - p.TOP_CLEARANCE_MM
    bottom = bottom_inner - p.D_HOUSING_PLATE_MM
    out = dict(yc0=yc0, yc1=yc1, ym0=ym0, ym1=ym1, yf=yf, gap_stop=gap_stop, cl_f2_armL=cl_f2_armL,
               zgap_f2_links=zgap_f2_links, zgap_f2_pin=zgap_f2_pin, zgap_f0_hubL=zgap_f0_hubL,
               top=top, bottom_inner=bottom_inner, bottom=bottom, joint_open_y=kd.arm_joint(p.ANGLE_PER_HANDLE_DEG)[1])
    print("\ny-layout (pivot line y = 0): carriage pin y %.1f (closed) -> %.1f (stop); moving axle = pin + %.0f: %.1f -> %.1f;"
          % (yc0, yc1, p.D_MOVING_AXLE_ABOVE_PIN_MM, ym0, ym1))
    print("   fixed row axle y = %.1f (F0, F1, F2, F3 and dead end); moving/fixed body gap at the stop = %.1f mm (min %.0f) -> %s"
          % (yf, gap_stop, p.D_PULLEY_GAP_AT_STOP_MM, "PASS" if gap_stop >= p.D_PULLEY_GAP_AT_STOP_MM else "FAIL"))
    print("   F2 (x 0, z %.0f) vs arm L: 3D clearance %.1f | F2 vs links z-gap %.1f | F2 vs carriage pin (back overhang %.0f) z-gap %.1f | F0 vs hub L: share z, y-separated by %.1f mm"
          % (f2[1], cl_f2_armL, zgap_f2_links, p.D_CARRIAGE_PIN_BACK_OVERHANG_MM, zgap_f2_pin, yf - p.D_FAIRLEAD_PULLEY_DIAMETER_MM / 2 - p.D_HUB_RADIUS_MM))
    print("   housing: top plate outer y %.1f, bottom plate inner y %.1f / outer %.1f (joint at open y %.1f; hands need bottom >= %+.0f)"
          % (top, bottom_inner, bottom, out["joint_open_y"], p.D_HOUSING_BOTTOM_Y_MM))
    return out


def c_zgap(z1, z2):
    return z_gap(z1, z2)


def width_budget():
    """Inner width needed: carriage block + clearance + rod bushes + wall clearance, spring beside or above."""
    half_block = p.D_CARRIAGE_WIDTH_MM / 2
    rod_centre = half_block + p.ARM_CLEARANCE_MM + p.D_ROD_BUSH_OD_MM / 2
    inner_rods = 2 * (rod_centre + p.D_ROD_BUSH_OD_MM / 2 + p.ARM_CLEARANCE_MM)
    inner_spring_beside = 2 * (rod_centre + p.D_ROD_BUSH_OD_MM / 2 + p.ARM_CLEARANCE_MM + p.D_SPRING_OD_MM + p.ARM_CLEARANCE_MM)
    inner_avail = p.HOUSING_WIDTH_MM_D - 2 * p.HOUSING_WALL_MM
    print("\nWidth budget (inner): block %.0f + 2 x (%.0f gap + %.0f bush) + 2 x %.0f wall clearance = %.0f mm with rods at x = +/-%.0f"
          % (p.D_CARRIAGE_WIDTH_MM, p.ARM_CLEARANCE_MM, p.D_ROD_BUSH_OD_MM, p.ARM_CLEARANCE_MM, inner_rods, rod_centre))
    print("   spring BESIDE the carriage (dia %.0f): %.0f mm needed; spring ABOVE the carriage at x = 0, z = %.0f between M2 and M3 (gap %.0f): %.0f mm needed"
          % (p.D_SPRING_OD_MM, inner_spring_beside, p.D_CABLE_Z_CENTRE_MM, p.D_FIXED_PULLEY_DIAMETER_MM - p.PULLEY_WIDTH_MM, inner_rods))
    print("   available inner width = %.0f - 2 x %.0f = %.0f -> rods: %s, spring beside: %s, spring above: %s"
          % (p.HOUSING_WIDTH_MM_D, p.HOUSING_WALL_MM, inner_avail, "PASS" if inner_rods <= inner_avail else "FAIL",
             "PASS" if inner_spring_beside <= inner_avail else "FAIL", "PASS" if inner_rods <= inner_avail else "FAIL"))
    return inner_rods, inner_spring_beside, inner_avail, rod_centre


def main():
    print("check3d_D.py - a = %.1f, c = %.1f, psi0 = %.1f, P = %.2f; pins: pivot M%.0f, joint M%.0f, carriage M%.0f; hub R%.0f (boss R%.0f), joint boss R%.0f"
          % (p.D_ARM_MM, p.D_LINK_MM, p.D_ARM_ANGLE_CLOSED_DEG, p.PIVOT_SPACING_MM, p.D_PIVOT_BOLT_MM, p.D_JOINT_PIN_MM,
             p.D_CARRIAGE_PIN_MM, p.D_HUB_RADIUS_MM, p.PIVOT_BOSS_MM, p.D_JOINT_BOSS_RADIUS_MM))
    w1 = clearance_table(p.D_STACK_REPORTED, "A) stack as reported (arm R, arm L, link R, link L)")
    w2 = clearance_table(p.D_STACK, "B) re-stacked (arm R, link R, link L, arm L)")
    for lab, w in (("A", w1), ("B", w2)):
        print("   %s: pairs under %.0f mm: %s" % (lab, p.D_CLEARANCE_MIN_MM, ", ".join("%s/%s %.1f" % t for t in w) if w else "none"))
    zc = 0.5 * (p.D_STACK["arm_R"][0] + p.D_STACK["arm_L"][1])
    print("\nJoggle / spacer: arm R plate centre z = %.2f, arm L centre z = %.2f -> common grip plane z = %.2f;"
          % (0.5 * sum(p.D_STACK["arm_R"]), 0.5 * sum(p.D_STACK["arm_L"]), zc))
    print("   spacer under the right long arm = %.2f mm, under the left = %.2f mm (hub extended in z, long arm bolted on at the common plane)"
          % (zc - 0.5 * sum(p.D_STACK["arm_R"]), 0.5 * sum(p.D_STACK["arm_L"]) - zc))
    grip_checks()
    reeving_lanes()
    y_layout()
    width_budget()
    zf = p.D_CABLE_Z_CENTRE_MM - p.D_MOVING_PULLEY_DIAMETER_MM / 2
    z_back = p.D_CABLE_Z_CENTRE_MM + p.D_MOVING_PULLEY_DIAMETER_MM / 2 + p.PULLEY_WIDTH_MM / 2
    z_front = min(0.0, zf - p.D_FAIRLEAD_PULLEY_DIAMETER_MM - p.CABLE_DIAMETER_MM / 2)
    print("\nDepth: stack z 0..%.1f, cable z %.1f..%.1f, anchor down-lane z %.1f -> inner %.1f..%.1f, outer depth %.1f mm with %.0f mm plates"
          % (p.D_STACK["arm_L"][1], zf - p.PULLEY_WIDTH_MM / 2, z_back, zf - p.D_FAIRLEAD_PULLEY_DIAMETER_MM,
             z_front - p.ARM_CLEARANCE_MM, z_back + p.ARM_CLEARANCE_MM, (z_back + p.ARM_CLEARANCE_MM) - (z_front - p.ARM_CLEARANCE_MM) + 2 * p.HOUSING_WALL_MM, p.HOUSING_WALL_MM))


if __name__ == "__main__":
    main()
