"""kinematics_D.py - verification of architecture D (links + hard stop) before any CAD.

Run:  python3 kinematics_D.py          (prints the tables; add --plots to save PNGs to docs/img/)
Reads params.py. Does not change kinematics.py (architectures E and C stay as they were).

Geometry, right handle, housing frame (x outward, y up), pivot O = (P/2, 0):
  drive arm: joint A = O + a * (cos(alpha), sin(alpha)),  alpha = 180 deg - psi0 + beta
             (psi0 = D_ARM_ANGLE_CLOSED_DEG above the inward horizontal, beta = handle opening, CCW +)
  link:      carriage pin C = (0, y_c) with |C - A| = c, the root ABOVE the joint
             y_c = A_y + sqrt(c^2 - A_x^2)
  carriage:  y_c(s) = y_c(0) - s / n   (moves DOWN as the cable pays out; see report, check 5)
  beta(s):   solved numerically from y_c(beta) = y_c(0) - s / n
Loads (statics, frictionless joints):
  hand torque on one handle        T = F * L_LINE * sin(theta),  theta = THETA_CLOSED + beta
  link force (along the link)      F_link = T / (a * sin(mu)),   mu = angle between arm and link
  carriage force from both links   F_pin = 2 * F_link * cos(lambda),  lambda = link angle from vertical
                                   (positive = pushes the carriage UP = closing direction)
  cross-check by virtual work      F_pin = 2 T (dbeta/ds) n
  cable tension (model A3)         n T_c = spring + F_pin +/- friction,  friction = (1 - eta)(spring + |F_pin|)
"""
import math
import sys
import params as p

ETA_D = p.ETA_SLIDE * p.ETA_PULLEY ** (p.D_N_REEVE - 1)     # same model as main_design.md A3: 0.90 x 0.97^7


# ------------------------------------------------------------------ geometry
def pivot_x():
    """x of the right pivot [mm] = P / 2."""
    return p.PIVOT_SPACING_MM / 2.0


def arm_joint(beta_deg, a=None):
    """Position (x, y) [mm] of the arm/link joint for handle opening beta [deg]."""
    a = p.D_ARM_MM if a is None else a
    alpha = math.radians(180.0 - p.D_ARM_ANGLE_CLOSED_DEG + beta_deg)
    return pivot_x() + a * math.cos(alpha), a * math.sin(alpha)


def pin_y(beta_deg, a=None, c=None):
    """Carriage pin height y_c [mm] for opening beta: y_c = A_y + sqrt(c^2 - A_x^2) (link above joint)."""
    c = p.D_LINK_MM if c is None else c
    ax, ay = arm_joint(beta_deg, a)
    if c * c < ax * ax:
        raise ValueError("link too short: |A_x| = %.1f > c = %.1f" % (abs(ax), c))
    return ay + math.sqrt(c * c - ax * ax)


def carriage_travel(beta_deg, a=None, c=None):
    """Carriage travel [mm] from closed, positive downward: y_c(0) - y_c(beta)."""
    return pin_y(0.0, a, c) - pin_y(beta_deg, a, c)


def stroke_at_stop():
    """Attachment travel s [mm] at which the handles reach the 110 deg stop: n * travel(30 deg)."""
    return p.D_N_REEVE * carriage_travel(p.ANGLE_PER_HANDLE_DEG)


def beta_of_s(s_mm):
    """Handle opening beta [deg] for attachment travel s [mm], by bisection; clamped at the stop."""
    target = min(s_mm, stroke_at_stop()) / p.D_N_REEVE
    lo, hi = 0.0, p.ANGLE_PER_HANDLE_DEG
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if carriage_travel(mid) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def included_deg(s_mm):
    """Included angle [deg] = closed + 2 beta."""
    return p.ANGLE_INCLUDED_CLOSED_DEG + 2.0 * beta_of_s(s_mm)


def linear_target_deg(s_mm):
    """Linear mapping target [deg] over STROKE_MAP_MM."""
    return p.ANGLE_INCLUDED_CLOSED_DEG + (p.ANGLE_INCLUDED_OPEN_DEG - p.ANGLE_INCLUDED_CLOSED_DEG) * min(s_mm / p.STROKE_MAP_MM, 1.0)


def dbeta_ds(s_mm, ds=0.5):
    """d(beta)/ds [rad/mm], central difference, 0 at the stop."""
    s_stop = stroke_at_stop()
    s1, s2 = max(s_mm - ds, 0.0), min(s_mm + ds, s_stop)
    if s2 <= s1:
        return 0.0
    return math.radians(beta_of_s(s2) - beta_of_s(s1)) / (s2 - s1)


# ------------------------------------------------------------------ angles and loads
def transmission_angle_deg(beta_deg):
    """Angle [deg] between the arm (O->A) and the link (A->C); 90 is ideal."""
    ax, ay = arm_joint(beta_deg)
    ox = pivot_x()
    ux, uy = ax - ox, ay                      # arm direction
    vx, vy = 0.0 - ax, pin_y(beta_deg) - ay   # link direction
    cosang = (ux * vx + uy * vy) / (math.hypot(ux, uy) * math.hypot(vx, vy))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
    return min(ang, 180.0 - ang)


def link_angle_from_vertical_deg(beta_deg):
    """Angle [deg] of the link from vertical."""
    ax, ay = arm_joint(beta_deg)
    return math.degrees(math.atan2(abs(ax), pin_y(beta_deg) - ay))


def theta_deg(s_mm):
    """Pivot->grip line angle [deg] = THETA_CLOSED + beta."""
    return p.THETA_CLOSED_DEG + beta_of_s(s_mm)


def hand_torque_nm(s_mm, f_hand_n):
    """Hand torque on one handle [N m], + = closing: F * L_LINE * sin(theta)."""
    return f_hand_n * p.L_LINE_MM / 1000.0 * math.sin(math.radians(theta_deg(s_mm)))


def link_force_n(s_mm, f_hand_n):
    """Force in one link [N] (compression when the hand torque is closing): T / (a sin mu)."""
    beta = beta_of_s(s_mm)
    mu = math.radians(transmission_angle_deg(beta))
    return hand_torque_nm(s_mm, f_hand_n) / (p.D_ARM_MM / 1000.0 * math.sin(mu))


def pin_force_n(s_mm, f_hand_n):
    """Force on the carriage from both links [N], + = up (closing direction): 2 F_link cos(lambda)."""
    beta = beta_of_s(s_mm)
    return 2.0 * link_force_n(s_mm, f_hand_n) * math.cos(math.radians(link_angle_from_vertical_deg(beta)))


def pin_force_virtual_work_n(s_mm, f_hand_n):
    """Cross-check: F_pin = 2 T (dbeta/ds) n."""
    return 2.0 * hand_torque_nm(s_mm, f_hand_n) * 1000.0 * dbeta_ds(s_mm) * p.D_N_REEVE


def spring_force_n(s_mm):
    """Return spring on the carriage [N], pulls UP (toward closed): preload + rate * carriage travel."""
    travel = min(s_mm, stroke_at_stop()) / p.D_N_REEVE
    return p.SPRING_PRELOAD_N + p.SPRING_RATE_N_PER_MM * travel


def friction_n(s_mm, f_hand_n):
    """Friction on the carriage [N], model A3: (1 - eta) (spring + |pin|)."""
    return (1.0 - ETA_D) * (spring_force_n(s_mm) + abs(pin_force_n(s_mm, f_hand_n)))


def cable_tension_n(s_mm, f_hand_n, direction):
    """Reference-cable tension [N]: (spring + pin +/- friction) / n. Down stroke: friction adds."""
    f = friction_n(s_mm, f_hand_n)
    if direction == "up":
        f = -f
    return (spring_force_n(s_mm) + pin_force_n(s_mm, f_hand_n) + f) / p.D_N_REEVE


def stop_load_n(anchor_tension_n, f_hand_n):
    """Carriage hard-stop reaction [N] once at the stop: n T_anchor - spring - pin (cable pulls down)."""
    s = stroke_at_stop()
    return p.D_N_REEVE * anchor_tension_n - spring_force_n(s) - pin_force_n(s, f_hand_n)


# ------------------------------------------------------------------ envelope and interference
def carriage_half_w():
    """Carriage half-width [mm]: pulleys side by side on one axle."""
    per_row = p.D_CARRIAGE_PULLEYS_PER_ROW
    return (per_row * p.PULLEY_DIAMETER_MM + (per_row - 1) * p.PULLEY_GAP_MM) / 2.0


def carriage_rows():
    """Pulley rows (axles) on the carriage."""
    moving = p.D_N_REEVE // 2
    return (moving + p.D_CARRIAGE_PULLEYS_PER_ROW - 1) // p.D_CARRIAGE_PULLEYS_PER_ROW


def housing_width_mm():
    """Housing outside width [mm] = 2 (widest of pivot boss / carriage + clearance + wall)."""
    half = max(pivot_x() + p.PIVOT_BOSS_MM, carriage_half_w())
    return 2.0 * (half + p.ARM_CLEARANCE_MM + p.HOUSING_WALL_MM)


def housing_extent_mm():
    """(top, bottom) [mm] of the housing relative to the pivot line. ASSUMPTION layout:
    carriage pulley rows above the pin; fixed pulley rows below the lowest carriage position."""
    rows = carriage_rows()
    y_top_pin = pin_y(0.0)
    y_low_pin = pin_y(p.ANGLE_PER_HANDLE_DEG)
    top = y_top_pin + p.D_CARRIAGE_HALF_H_MM + rows * (p.PULLEY_DIAMETER_MM + p.PULLEY_GAP_MM) + p.TOP_CLEARANCE_MM + p.HOUSING_WALL_MM
    fixed_first = y_low_pin - p.D_CARRIAGE_HALF_H_MM - p.PULLEY_GAP_MM - p.PULLEY_DIAMETER_MM / 2.0
    fixed_last = fixed_first - (rows - 1) * (p.PULLEY_DIAMETER_MM + p.PULLEY_GAP_MM)
    bottom = fixed_last - p.PULLEY_DIAMETER_MM / 2.0 - p.ARM_CLEARANCE_MM - p.HOUSING_WALL_MM
    return top, bottom, fixed_first, fixed_last


def point_rect_dist(px, py, x0, x1, y0, y1):
    """Distance [mm] from a point to an axis-aligned rectangle (0 inside)."""
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def segment_rect_dist(ax, ay, bx, by, x0, x1, y0, y1, samples=200):
    """Min distance [mm] between segment A-B and a rectangle, by sampling."""
    return min(point_rect_dist(ax + (bx - ax) * i / samples, ay + (by - ay) * i / samples, x0, x1, y0, y1)
               for i in range(samples + 1))


def point_segment_dist(px, py, ax, ay, bx, by):
    """Distance [mm] from a point to segment A-B."""
    vx, vy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def interference_margins(s_mm):
    """Clearances [mm] at stroke s (negative = collision in the model):
    arm->carriage body, arm joint->opposite pivot bolt, link->opposite pivot bolt, arm-tip crossing of centre line."""
    beta = beta_of_s(s_mm)
    ox = pivot_x()
    ax, ay = arm_joint(beta)
    yc = pin_y(beta)
    hw, hh = carriage_half_w(), p.D_CARRIAGE_HALF_H_MM
    arm_to_carriage = segment_rect_dist(ox, 0.0, ax, ay, -hw, hw, yc - hh, yc + hh) - p.D_ARM_WIDTH_MM / 2.0
    joint_to_bolt = math.hypot(ax + ox, ay) - p.D_ARM_WIDTH_MM / 2.0 - p.D_PIVOT_BOLT_MM / 2.0
    link_to_bolt = point_segment_dist(-ox, 0.0, ax, ay, 0.0, yc) - p.D_LINK_WIDTH_MM / 2.0 - p.D_PIVOT_BOLT_MM / 2.0
    crossing = -ax if ax < 0 else 0.0
    return arm_to_carriage, joint_to_bolt, link_to_bolt, crossing


# ------------------------------------------------------------------ report
def tag(ok):
    return "PASS" if ok else "FAIL"


def srange(step):
    s_stop = stroke_at_stop()
    pts = [i * step for i in range(int(s_stop // step) + 1)]
    if pts[-1] < s_stop - 1e-9:
        pts.append(s_stop)
    return pts


def main(save_plots=False):
    s_stop = stroke_at_stop()
    print("kinematics_D.py - architecture D check. a = %.1f, c = %.1f, psi0 = %.1f deg, P = %.1f, L_line = %.1f, n = %d, eta = %.3f"
          % (p.D_ARM_MM, p.D_LINK_MM, p.D_ARM_ANGLE_CLOSED_DEG, p.PIVOT_SPACING_MM, p.L_LINE_MM, p.D_N_REEVE, ETA_D))
    print("pin y at closed %.2f, at 30 deg %.2f -> carriage travel %.2f mm -> stop at s = %.1f mm (target %.0f)"
          % (pin_y(0), pin_y(30), carriage_travel(30), s_stop, p.STROKE_MAP_MM))

    print("\n1) Handle angle vs carriage travel (5 mm steps)")
    print("   carriage mm |  s mm | beta | included | linear target | deviation")
    worst = (0.0, 0.0)
    for s in srange(1.0):
        dev = included_deg(s) - linear_target_deg(s)
        if abs(dev) > abs(worst[0]):
            worst = (dev, s)
    for s in srange(5.0 * p.D_N_REEVE):
        print("   %10.2f | %5.1f | %5.2f | %8.2f | %13.2f | %+6.2f"
              % (s / p.D_N_REEVE, s, beta_of_s(s), included_deg(s), linear_target_deg(s), included_deg(s) - linear_target_deg(s)))
    print("   max deviation from linear: %+.2f deg at s = %.0f mm = %.1f %% of the 60 deg range" % (worst[0], worst[1], 100 * abs(worst[0]) / 60.0))

    print("\n2) Torque sign (theta = %.0f + beta)" % p.THETA_CLOSED_DEG)
    for F in p.F_HAND_STUDY_N:
        ts = [hand_torque_nm(s, F) for s in srange(1.0)]
        print("   F = %3.0f N: T min %+.3f N m, max %+.3f N m, theta %.1f .. %.1f deg -> %s"
              % (F, min(ts), max(ts), theta_deg(0), theta_deg(s_stop), tag(min(ts) >= 0.0)))

    print("\n3) Loads (eta = %.3f, spring %.0f N + %.2f N/mm, friction = (1-eta)(spring+|pin|))" % (ETA_D, p.SPRING_PRELOAD_N, p.SPRING_RATE_N_PER_MM))
    print("   s mm | F | T Nm | mu deg | F_link | F_pin (statics) | F_pin (virt.work) | spring | fric | T_c down | T_c up")
    for F in p.F_HAND_STUDY_N:
        for s in srange(25.0):
            print("   %5.1f | %3.0f | %5.2f | %6.1f | %6.1f | %15.1f | %17.1f | %6.1f | %4.1f | %8.2f | %6.2f"
                  % (s, F, hand_torque_nm(s, F), transmission_angle_deg(beta_of_s(s)), link_force_n(s, F), pin_force_n(s, F),
                     pin_force_virtual_work_n(s, F), spring_force_n(s), friction_n(s, F),
                     cable_tension_n(s, F, "down"), cable_tension_n(s, F, "up")))
    r0 = max(cable_tension_n(s, 0.0, "down") for s in srange(1.0))
    rl = max(cable_tension_n(s, p.F_HAND_N, "down") for s in srange(1.0))
    tmin = min(cable_tension_n(s, F, "up") for s in srange(1.0) for F in p.F_HAND_STUDY_N)
    print("   R0 = %.2f N (goal <= %.0f -> %s) | R_load = %.2f N (goal <= %.0f -> %s) | min tension any case %.2f N (>= %.0f -> %s)"
          % (r0, p.R0_MAX_N, tag(r0 <= p.R0_MAX_N), rl, p.R_LOAD_FRACTION_MAX * p.STACK_LOAD_N,
             tag(rl <= p.R_LOAD_FRACTION_MAX * p.STACK_LOAD_N), tmin, p.CABLE_TENSION_MIN_N, tag(tmin >= p.CABLE_TENSION_MIN_N)))
    print("   Carriage stop load (v2, anchor spring; cable pulls the carriage DOWN onto the stop):")
    for Ta in p.D_ANCHOR_TENSION_STUDY_N:
        print("     anchor tension %.0f N: " % Ta + ", ".join("F = %.0f -> %.0f N" % (F, stop_load_n(Ta, F)) for F in p.F_HAND_STUDY_N))
    print("     v1 (no anchor spring): cable goes taut at the stop; stop load = 8 x whatever tension the user applies - not computable")

    print("\n4) Geometry")
    mus = [transmission_angle_deg(beta_of_s(s)) for s in srange(1.0)]
    top, bottom, f1, f2 = housing_extent_mm()
    print("   arm %.1f mm, link %.1f mm, arm angle at closed %.1f deg, P = %.2f mm, carriage travel %.2f mm, stop at s = %.1f mm"
          % (p.D_ARM_MM, p.D_LINK_MM, p.D_ARM_ANGLE_CLOSED_DEG, p.PIVOT_SPACING_MM, carriage_travel(30), s_stop))
    print("   transmission angle min %.1f deg (at closed %.1f, open %.1f), limit %.0f -> %s"
          % (min(mus), mus[0], mus[-1], p.D_TRANSMISSION_ANGLE_MIN_DEG, tag(min(mus) >= p.D_TRANSMISSION_ANGLE_MIN_DEG)))
    print("   link angle from vertical: closed %.1f deg, open %.1f deg" % (link_angle_from_vertical_deg(0), link_angle_from_vertical_deg(30)))
    print("   housing width %.1f mm (carriage %.0f wide, %d rows; pivot boss R%.0f)" % (housing_width_mm(), 2 * carriage_half_w(), carriage_rows(), p.PIVOT_BOSS_MM))
    print("   housing top %.1f above pivot line, bottom %.1f below -> height %.1f; fixed pulley rows at y = %.1f, %.1f (ASSUMPTION layout)"
          % (top, -bottom, top - bottom, f1, f2))
    print("   pin y: closed %.2f, stop %.2f" % (pin_y(0), pin_y(30)))
    print("   interference margins [mm] (negative = collision in the model):")
    print("   s mm | arm->carriage body | joint->opposite pivot bolt | link->opposite pivot bolt | arm tip past centre line (z-stacked)")
    for s in (0.0, 75.0, 150.0, 225.0, s_stop):
        m = interference_margins(s)
        print("   %5.1f | %18.1f | %26.1f | %25.1f | %5.1f" % (s, m[0], m[1], m[2], m[3]))

    if save_plots:
        save_plots_png()


def save_plots_png():
    """Save the four requested plots to docs/img/ (matplotlib, already installed)."""
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs("docs/img", exist_ok=True)
    ss = srange(1.0)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(ss, [included_deg(s) for s in ss], label="mechanism (a=%.1f, c=%.1f)" % (p.D_ARM_MM, p.D_LINK_MM))
    ax.plot(ss, [linear_target_deg(s) for s in ss], "--", label="linear target")
    ax.set_xlabel("attachment travel s [mm]"); ax.set_ylabel("included angle [deg]"); ax.grid(True); ax.legend()
    ax.set_title("D: handle angle vs travel"); fig.tight_layout(); fig.savefig("docs/img/D_angle_vs_travel.png", dpi=120)
    fig, ax = plt.subplots(figsize=(6, 4))
    for F in p.F_HAND_STUDY_N:
        ax.plot(ss, [hand_torque_nm(s, F) for s in ss], label="F = %.0f N/hand" % F)
    ax.set_xlabel("attachment travel s [mm]"); ax.set_ylabel("hand torque per handle [N m], + = closing"); ax.grid(True); ax.legend()
    ax.set_title("D: hand torque vs travel"); fig.tight_layout(); fig.savefig("docs/img/D_torque_vs_travel.png", dpi=120)
    fig, ax = plt.subplots(figsize=(6, 4))
    for F in p.F_HAND_STUDY_N:
        ax.plot(ss, [cable_tension_n(s, F, "down") for s in ss], label="F = %.0f N, down stroke" % F)
        ax.plot(ss, [cable_tension_n(s, F, "up") for s in ss], ":", label="F = %.0f N, up stroke" % F)
    ax.axhline(p.R0_MAX_N, color="grey", lw=0.8); ax.set_xlabel("attachment travel s [mm]"); ax.set_ylabel("reference-cable tension [N]")
    ax.grid(True); ax.legend(fontsize=7); ax.set_title("D: cable tension vs travel"); fig.tight_layout(); fig.savefig("docs/img/D_cable_tension_vs_travel.png", dpi=120)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(ss, [transmission_angle_deg(beta_of_s(s)) for s in ss])
    ax.axhline(p.D_TRANSMISSION_ANGLE_MIN_DEG, color="red", lw=0.8, label="limit %.0f deg" % p.D_TRANSMISSION_ANGLE_MIN_DEG)
    ax.set_xlabel("attachment travel s [mm]"); ax.set_ylabel("transmission angle arm-link [deg]"); ax.grid(True); ax.legend()
    ax.set_title("D: transmission angle vs travel"); fig.tight_layout(); fig.savefig("docs/img/D_transmission_angle_vs_travel.png", dpi=120)
    print("\nplots saved to docs/img/")


if __name__ == "__main__":
    main(save_plots="--plots" in sys.argv)
