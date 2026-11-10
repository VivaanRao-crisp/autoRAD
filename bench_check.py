"""bench_check.py - final check of the BENCH RIG (one board, 4 cable parts, cord system behind the board).

Run:  python3 bench_check.py            (prints everything)
      python3 bench_check.py --template (also writes docs/img/board_template_1to1.pdf, A4, true scale)

Reuses kinematics_D.py (link mechanism) and check3d_D.py (3D stack clearances) with the bench values
in params.py. Adds the board layout: slots, standoff positions, cord lanes, extents, and the
carriage shape. Nothing here is validated; only the print and the bench test are.
"""
import math
import sys
import params as p
import kinematics_D as kd
import check3d_D as c3

kd.ETA_D = p.BENCH_ETA


# ------------------------------------------------------------------ layout numbers
def layout():
    """All derived bench-rig positions [mm] in the board frame (x outward, y up from the pivot line)."""
    yc0, yc1 = kd.pin_y(0.0), kd.pin_y(p.ANGLE_PER_HANDLE_DEG)
    R = p.BENCH_PULLEY_D_MM / 2
    axle0, axle1 = yc0 + p.BENCH_AXLE_ABOVE_PIN_MM, yc1 + p.BENCH_AXLE_ABOVE_PIN_MM
    yf = axle1 - R - p.BENCH_LANE_GAP_AT_STOP_MM - R                 # fixed bearings' axle height
    x1, x2 = p.BENCH_MOVING_X_MM                                     # moving bearings
    xF1 = 0.5 * (x1 + x2)                                            # fixed bearing F1: tangents at x1, x2
    xF0 = x1 - R                                                     # fairlead bearing F0: tangents at x1 - 2R (anchor) and x1
    x_anchor = x1 - 2 * R
    zb = -p.BOARD_T_MM                                               # back face of the board
    z_plate = (zb - p.BENCH_CARRIAGE_T_MM, zb)                       # carriage plate
    z_axle = zb - p.BENCH_CARRIAGE_T_MM - 24.0                       # moving axle: post 24 mm behind the plate (ASSUMPTION)
    z_near, z_far = z_axle + R, z_axle - R                           # the two lane planes
    slot1 = (yc1 - p.BENCH_SLOT_W_MM / 2 - 0.5, yc0 + p.BENCH_SLOT_W_MM / 2 + 0.5)
    g2 = p.BENCH_GUIDE2_ABOVE_PIN_MM
    slot2 = (yc1 + g2 - p.BENCH_SLOT_W_MM / 2 - 0.5, yc0 + g2 + p.BENCH_SLOT_W_MM / 2 + 0.5)
    top = p.BOARD_BOTTOM_Y_MM + p.BOARD_H_MM
    return dict(yc0=yc0, yc1=yc1, axle0=axle0, axle1=axle1, yf=yf, x1=x1, x2=x2, xF1=xF1, xF0=xF0,
                x_anchor=x_anchor, zb=zb, z_plate=z_plate, z_axle=z_axle, z_near=z_near, z_far=z_far,
                slot1=slot1, slot2=slot2, top=top, R=R)


def carriage_outline(L):
    """Carriage plate outline in the carriage frame (origin = carriage pin, x outward, y up), as named rectangles.
    Gamma shape: vertical bar beside the pin, horizontal bar on top, post hanging down from the bar to the axle.
    The post sits between the two moving bearings; nothing of the plate may be at the fixed standoffs' x
    (xF0, xF1) below the horizontal bar, because those standoffs pass the plate's plane when the carriage is at the stop.
    """
    x1, x2, R = L["x1"], L["x2"], L["R"]
    w = p.BENCH_PULLEY_W_MM
    post_x = (x1 + w / 2 + 0.5, x2 - w / 2 - 0.5)                   # between the bearings, 0.5 mm washers
    h = p.BENCH_AXLE_ABOVE_PIN_MM
    return {
        "vertical bar": dict(x=(-4.0, 8.0), y=(-10.0, h + 20.0)),
        "horizontal bar": dict(x=(x1 - R - 2.0, 8.0), y=(h + 10.0, h + 20.0)),
        "post (hangs to the axle, 32 mm deep in z)": dict(x=post_x, y=(h - 10.0, h + 10.0)),
        "axle": dict(x=(x1 - R, x2 + R), y=(h, h)),
        "guide bolt 2": dict(x=(0.0, 0.0), y=(p.BENCH_GUIDE2_ABOVE_PIN_MM, p.BENCH_GUIDE2_ABOVE_PIN_MM)),
    }


# ------------------------------------------------------------------ checks
def print_kinematics():
    s_stop = kd.stroke_at_stop()
    print("BENCH RIG  a = %.1f, c = %.1f, psi0 = %.0f deg, P = %.2f, L_line = %.2f, n = %d, eta = %.2f"
          % (p.D_ARM_MM, p.D_LINK_MM, p.D_ARM_ANGLE_CLOSED_DEG, p.PIVOT_SPACING_MM, p.L_LINE_MM, p.D_N_REEVE, kd.ETA_D))
    print("carriage travel %.2f mm -> stop at s = %.1f mm" % (kd.carriage_travel(30), s_stop))
    print("\n1) Mapping (carriage travel 0..75 in 15 mm steps = attachment travel 0..300 in 60 mm steps)")
    print("   carriage | s | beta | included | linear | dev | joint R (x, y) | pin y")
    worst = (0.0, 0.0)
    for s in kd.srange(1.0):
        d = kd.included_deg(s) - kd.linear_target_deg(s)
        if abs(d) > abs(worst[0]):
            worst = (d, s)
    for s in [i * 60.0 for i in range(6)]:
        b = kd.beta_of_s(s)
        ax, ay = kd.arm_joint(b)
        print("   %6.2f | %3.0f | %5.2f | %6.2f | %6.2f | %+5.2f | (%6.1f, %5.1f) | %6.1f"
              % (s / p.D_N_REEVE, s, b, kd.included_deg(s), kd.linear_target_deg(s), kd.included_deg(s) - kd.linear_target_deg(s), ax, ay, kd.pin_y(b)))
    print("   max deviation %+.2f deg at s = %.0f = %.2f %% of 60 deg -> %s (limit 2.5)" % (worst[0], worst[1], 100 * abs(worst[0]) / 60, kd.tag(abs(worst[0]) / 60 <= 0.025)))
    mus = [kd.transmission_angle_deg(kd.beta_of_s(s)) for s in kd.srange(1.0)]
    print("   transmission angle min %.1f deg -> %s; torque sign: T(closed) = %+.2f, T(open) = %+.2f N m at 200 N/hand (always closing)"
          % (min(mus), kd.tag(min(mus) >= 30), kd.hand_torque_nm(0, 200), kd.hand_torque_nm(s_stop, 200)))
    print("   loads at 200 N/hand (reference only, bench runs at a few kg): link %.0f N, carriage %.0f N, cord %.1f N; at 20 N/hand: cord %.1f N"
          % (max(kd.link_force_n(s, 200) for s in kd.srange(1.0)), max(kd.pin_force_n(s, 200) for s in kd.srange(1.0)),
             max(kd.cable_tension_n(s, 200, "down") for s in kd.srange(1.0)), max(kd.cable_tension_n(s, 20, "down") for s in kd.srange(1.0))))


def print_layout(L):
    print("\n2) Board layout (board %.0f x %.0f x %.0f, bottom edge y = %.0f, top edge y = %.0f; front face z = 0, back face z = %.0f)"
          % (p.BOARD_W_MM, p.BOARD_H_MM, p.BOARD_T_MM, p.BOARD_BOTTOM_Y_MM, L["top"], L["zb"]))
    print("   pivots (M8 through the board): (+/-%.2f, 0)" % kd.pivot_x())
    print("   carriage pin y: %.1f (closed) -> %.1f (stop); board slot 1 at x 0, y %.1f .. %.1f, width %.1f" % (L["yc0"], L["yc1"], L["slot1"][0], L["slot1"][1], p.BENCH_SLOT_W_MM))
    print("   guide bolt 2 y: %.1f -> %.1f; board slot 2 at x 0, y %.1f .. %.1f" % (L["yc0"] + p.BENCH_GUIDE2_ABOVE_PIN_MM, L["yc1"] + p.BENCH_GUIDE2_ABOVE_PIN_MM, L["slot2"][0], L["slot2"][1]))
    print("   moving axle y: %.1f -> %.1f at z %.1f; bearings at x %.0f and %.0f; lanes at z %.1f (near) and %.1f (far)"
          % (L["axle0"], L["axle1"], L["z_axle"], L["x1"], L["x2"], L["z_near"], L["z_far"]))
    print("   fixed row y = %.1f: F1 at x %.0f (lane plane z %.1f), F0 at x %.0f (z %.1f); dead end screw at (x %.0f, y %.0f, z %.1f); anchor cord lane x %.0f, z %.1f, over the top edge"
          % (L["yf"], L["xF1"], L["z_far"], L["xF0"], L["z_near"], L["x2"], p.BENCH_DEAD_END_Y_MM, L["z_near"], L["x_anchor"], L["z_near"]))
    print("   standoff lengths from the back face: F1 %.1f, F0 %.1f, dead end %.1f (bearing/cord centred on the lane plane)"
          % (-(L["z_far"]) - p.BOARD_T_MM - p.BENCH_PULLEY_W_MM / 2, -(L["z_near"]) - p.BOARD_T_MM - p.BENCH_PULLEY_W_MM / 2, -(L["z_near"]) - p.BOARD_T_MM - 1.0))
    print("   cord lanes (x, z), each part vertical between carriage bearings and the fixed row, all pull the carriage DOWN:")
    parts = [("1 F0 -> M1", L["x1"], L["z_near"]), ("2 M1 -> F1", L["x1"], L["z_far"]), ("3 F1 -> M2", L["x2"], L["z_far"]), ("4 M2 -> dead end", L["x2"], L["z_near"])]
    for n, x, z in parts:
        print("      %-18s x %6.1f  z %6.1f" % (n, x, z))
    print("   distinct lanes %d/4; anchor lane (%.0f, %.1f) is a 5th, outside the bearings" % (len({(x, z) for _, x, z in parts}), L["x_anchor"], L["z_near"]))


def print_clearances(L):
    print("\n3) Clearances")
    worst = c3.clearance_table(p.D_STACK, "   3a) front stack, 5 carriage positions (handles, links, pins)")
    print("   flagged under %.0f mm: %s" % (p.D_CLEARANCE_MIN_MM, ", ".join("%s/%s %.1f" % t for t in worst) if worst else "none (0.3 mm plate gaps are by design)"))
    ox = kd.pivot_x()
    body = min(kd.point_segment_dist(-ox, 0, ox, 0, *kd.arm_joint(kd.beta_of_s(s))) - p.D_ARM_WIDTH_MM / 2 - p.D_PIVOT_BOLT_MM / 2 for s in kd.srange(1.0))
    print("   3b) right arm body to the LEFT pivot bolt (same bolt passes all planes): min %.1f mm -> %s" % (body, kd.tag(body >= p.D_CLEARANCE_MIN_MM)))
    # fixed standoff bolt heads on the front (z 0..3.5) vs link R crossing them at closed
    heads_z = 3.5
    zgap = p.D_STACK["link_R"][0] - heads_z
    print("   3c) F0/F1/dead-end bolt heads on the front face (z 0..%.1f, flat heads) vs link R (z %.1f+): z gap %.1f mm -> %s"
          % (heads_z, p.D_STACK["link_R"][0], zgap, kd.tag(zgap >= 2.0)))
    # link R vs the standoff bolt shanks: shanks are inside the board, so no conflict; report the crossing x for information
    yc0 = L["yc0"]
    ax, ay = kd.arm_joint(0.0)
    xr = ax * (yc0 - L["yf"]) / (yc0 - ay)
    print("   3d) link R crosses y = %.0f (fixed row) at x = %.1f at closed; standoffs at x %.0f and %.0f are BEHIND the board, so no conflict" % (L["yf"], xr, L["xF0"], L["xF1"]))
    # carriage post vs F1 at the stop
    post_bottom = L["yc1"] + p.BENCH_AXLE_ABOVE_PIN_MM - 10.0
    f1_top = L["yf"] + L["R"] + 2.0   # flange washer dia 20
    print("   3e) carriage post bottom (y %.1f at the stop) vs F1 flange top (y %.1f): gap %.1f mm -> %s" % (post_bottom, f1_top, post_bottom - f1_top, kd.tag(post_bottom - f1_top >= 2)))
    gap = (L["axle1"] - L["R"]) - (L["yf"] + L["R"])
    print("   3f) moving/fixed bearing gap at the stop %.1f mm (set to %.0f) -> %s" % (gap, p.BENCH_LANE_GAP_AT_STOP_MM, kd.tag(gap >= 5)))
    # carriage vertical bar (x -4..8) vs F1 standoff (xF1 +/- 5) and dead end (x2 +/- 2.5)
    vb = (-4.0, 8.0)
    d_f1 = L["xF1"] + 5.0
    print("   3g) carriage vertical bar x %.0f..%.0f vs F1 standoff edge x %.1f: %.1f mm; vs F0 standoff edge x %.1f: %.1f mm -> %s"
          % (vb[0], vb[1], d_f1, vb[0] - d_f1, L["xF0"] + 5.0, vb[0] - (L["xF0"] + 5.0), kd.tag(vb[0] - d_f1 >= 2)))
    lowest = L["yc1"] - 10.0
    print("   3h) carriage lowest point at the stop y %.1f vs dead-end screw y %.0f: %.1f mm -> %s" % (lowest, p.BENCH_DEAD_END_Y_MM, lowest - p.BENCH_DEAD_END_Y_MM, kd.tag(lowest - p.BENCH_DEAD_END_Y_MM >= 5)))
    # extents on the board
    xs = [abs(kd.arm_joint(kd.beta_of_s(s))[0]) + p.D_JOINT_BOSS_RADIUS_MM for s in kd.srange(1.0)]
    print("   3i) extents: joints reach |x| %.1f (board half-width %.0f); carriage top at closed y %.1f (board top %.0f); handle hub bottom y %.0f (board bottom %.0f)"
          % (max(xs), p.BOARD_W_MM / 2, L["yc0"] + p.BENCH_AXLE_ABOVE_PIN_MM + 20, L["top"], -p.D_HUB_RADIUS_MM, p.BOARD_BOTTOM_Y_MM))
    ok = max(xs) + 5 <= p.BOARD_W_MM / 2 and L["yc0"] + p.BENCH_AXLE_ABOVE_PIN_MM + 20 + 5 <= L["top"]
    print("       fits the board -> %s" % kd.tag(ok))


def print_handle():
    print("\n4) One-piece handle plate (handle frame = board frame at closed, origin = pivot bore; RIGHT handle, LEFT is the mirror)")
    ang = 180.0 - p.D_ARM_ANGLE_CLOSED_DEG
    jx, jy = kd.arm_joint(0.0)
    ox = kd.pivot_x()
    th = math.radians(p.THETA_CLOSED_DEG)
    gx, gy = p.L_LINE_MM * math.sin(th), -p.L_LINE_MM * math.cos(th)
    print("   hub: R%.0f, bore %.1f (M8 + 0.3); drive arm: %.0f wide, to the joint boss R%.0f bore %.1f at %.1f mm, %.0f deg from +x (joint at (%.2f, %.2f))"
          % (p.D_HUB_RADIUS_MM, p.D_PIVOT_BOLT_MM + 0.3, p.D_ARM_WIDTH_MM, p.D_JOINT_BOSS_RADIUS_MM, p.D_JOINT_PIN_MM + 0.3, p.D_ARM_MM, ang, jx - ox, jy))
    print("   long arm: %.0f wide, to the grip centre at %.2f mm, %.0f deg from +x (grip centre at (%.2f, %.2f))" % (p.D_ARM_WIDTH_MM, p.L_LINE_MM, -90 + p.THETA_CLOSED_DEG, gx, gy))
    print("   grip pad: 24 x 60 mm rectangle centred on the grip centre, long axis at %.0f deg from vertical (toward the centre line at the top); two 4.3 mm holes at +/-20 mm along it for wood screws into the dowel"
          % (p.ANGLE_INCLUDED_CLOSED_DEG / 2))
    print("   bell-crank angle between the two arms at the hub: %.1f deg; plate thickness %.0f; LEFT handle spacer tube: bore 8.3, OD 24, length %.1f (z 0..%.1f)"
          % (ang - (-90 + p.THETA_CLOSED_DEG), p.D_ARM_THICK_MM, p.D_STACK["arm_L"][0], p.D_STACK["arm_L"][0]))


def template(L):
    """A4 1:1 drilling template (PDF): board outline, holes, slots, edge notch, 100 mm scale bar."""
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    os.makedirs("docs/img", exist_ok=True)
    fig = plt.figure(figsize=(210 / 25.4, 297 / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-105, 105); ax.set_ylim(p.BOARD_BOTTOM_Y_MM - 3.5, p.BOARD_BOTTOM_Y_MM - 3.5 + 297); ax.set_aspect("equal"); ax.axis("off")
    ax.add_patch(Rectangle((-p.BOARD_W_MM / 2, p.BOARD_BOTTOM_Y_MM), p.BOARD_W_MM, p.BOARD_H_MM, fill=False, lw=1.0))
    ox = kd.pivot_x()
    holes = [((ox, 0), 8.3, "pivot R M8"), ((-ox, 0), 8.3, "pivot L M8"), ((L["xF1"], L["yf"]), 5.3, "F1 M5"), ((L["xF0"], L["yf"]), 5.3, "F0 M5"),
             ((L["x2"], p.BENCH_DEAD_END_Y_MM), 5.3, "dead end M5"), ((20, L["top"] - 8), 4.0, "band screw")]
    for (x, y), d, name in holes:
        ax.add_patch(Circle((x, y), d / 2, fill=False, lw=0.8)); ax.plot([x - 4, x + 4], [y, y], "k-", lw=0.4); ax.plot([x, x], [y - 4, y + 4], "k-", lw=0.4)
        ax.text(x + 6, y + 3, "%s\n(%.1f, %.1f)" % (name, x, y), fontsize=6)
    for (y0, y1), name in ((L["slot1"], "slot 1"), (L["slot2"], "slot 2")):
        ax.add_patch(Rectangle((-p.BENCH_SLOT_W_MM / 2, y0), p.BENCH_SLOT_W_MM, y1 - y0, fill=False, lw=0.8))
        ax.text(5, 0.5 * (y0 + y1), "%s  x 0, y %.1f..%.1f, w %.1f" % (name, y0, y1, p.BENCH_SLOT_W_MM), fontsize=6)
    ax.plot([L["x_anchor"], L["x_anchor"]], [L["top"] - 6, L["top"]], "k-", lw=1.2); ax.text(L["x_anchor"] - 30, L["top"] - 12, "anchor cord notch x %.0f" % L["x_anchor"], fontsize=6)
    ax.plot([-100, -100], [p.BOARD_BOTTOM_Y_MM, L["top"]], "k:", lw=0.5)
    ax.plot([-90, 10], [p.BOARD_BOTTOM_Y_MM + 10, p.BOARD_BOTTOM_Y_MM + 10], "k-", lw=2); ax.text(-90, p.BOARD_BOTTOM_Y_MM + 13, "100 mm scale bar - check with a ruler after printing (print at 100 %, no 'fit to page')", fontsize=7)
    ax.plot([-100, 100], [0, 0], "k--", lw=0.4); ax.text(60, 2, "pivot line y = 0", fontsize=6)
    ax.text(-100, L["top"] + 2, "autoRAD bench rig - board drilling template, 1:1, A4 portrait. Board %.0f x %.0f, bottom-left corner at (%.0f, %.0f). Front face (handles side) shown." % (p.BOARD_W_MM, p.BOARD_H_MM, -p.BOARD_W_MM / 2, p.BOARD_BOTTOM_Y_MM), fontsize=6)
    fig.savefig("docs/img/board_template_1to1.pdf")
    print("\ntemplate written: docs/img/board_template_1to1.pdf")


def main():
    L = layout()
    print_kinematics()
    print_layout(L)
    print_clearances(L)
    print_handle()
    print("\n5) Carriage plate (carriage frame, origin = carriage pin, mm):")
    for k, v in carriage_outline(L).items():
        print("   %-40s x %6.1f .. %6.1f   y %6.1f .. %6.1f" % (k, v["x"][0], v["x"][1], v["y"][0], v["y"][1]))
    if "--template" in sys.argv:
        template(L)


if __name__ == "__main__":
    main()
