"""kinematics.py - angle mapping, slot profile, carriage force balance, envelope and checks for autoRAD.

Run:  python3 kinematics.py
All inputs come from params.py. These are preliminary calculations, not validation.

Sign conventions (right-hand handle, looking at the user's frontal plane):
  x = outward (+ to the right), y = up. Pivot of the right handle at (+P/2, 0).
  The carriage pin sits on the centre line at (0, y_pin), y_pin = Y0 + s / N_REEVE.
  Opening = the grip (below the pivot) moves outward = counter-clockwise rotation of the handle.
  Pin force: positive = DOWN on the carriage (closing direction, same sense as the spring).

Layout variables N_REEVE and Y0_PIN_MM are read from params at call time, so set_layout() can
override them for the trade-off table (restore_layout() puts the baseline back).
"""
import math
import params as p

_BASELINE = (p.N_REEVE, p.Y0_PIN_MM)


def set_layout(n_reeve, y0_mm):
    """Override the reeving count and pin height (for the trade-off scan)."""
    p.N_REEVE = n_reeve
    p.Y0_PIN_MM = y0_mm
    p.DY_DBETA_MM_PER_RAD = p.K_MM_PER_RAD / n_reeve
    p.ETA_MECH = p.eta_mech(n_reeve)
    p.CARRIAGE_WIDTH_MM = p.carriage_width_mm(n_reeve)


def restore_layout():
    """Put the baseline layout from params.py back."""
    set_layout(*_BASELINE)


# ------------------------------------------------------------------ angle mapping
def stroke_fraction(stroke_mm):
    """Fraction 0..1 of the mapping stroke, clamped (the dwell holds 1.0)."""
    return min(max(stroke_mm / p.STROKE_MAP_MM, 0.0), 1.0)


def profile(u):
    """Opening fraction g(u) for stroke fraction u (0..1). One parameter selects the profile.

    linear:        g = u
    linear_eased:  g = m u                      for u <= 1 - e     (constant slope m)
                   g = 1 - m (1 - u)^2 / (2 e)  for u >  1 - e     (slope falls linearly to 0 at u = 1)
                   continuity at u = 1 - e gives m = 1 / (1 - e/2).  g(1) = 1, g'(1) = 0.
    """
    if p.PROFILE == "linear" or p.EASE_FRACTION <= 0.0:
        return u
    if p.PROFILE == "linear_eased":
        e = p.EASE_FRACTION
        m = 1.0 / (1.0 - e / 2.0)
        if u <= 1.0 - e:
            return m * u
        return 1.0 - m * (1.0 - u) ** 2 / (2.0 * e)
    raise NotImplementedError("TODO: profile '%s' not defined" % p.PROFILE)


def handle_rotation_deg(stroke_mm):
    """Rotation of ONE handle from closed [deg] = 30 deg * profile(fraction)."""
    return p.ANGLE_PER_HANDLE_DEG * profile(stroke_fraction(stroke_mm))


def dbeta_ds_rad_per_mm(stroke_mm, ds=0.5):
    """Handle angular rate per mm of travel [rad/mm], central difference (0 in the dwell)."""
    s1 = max(stroke_mm - ds, 0.0)
    s2 = min(stroke_mm + ds, p.STROKE_DWELL_END_MM)
    if s2 <= s1:
        return 0.0
    return math.radians(handle_rotation_deg(s2) - handle_rotation_deg(s1)) / (s2 - s1)


def included_angle_deg(stroke_mm):
    """Included angle between the grip tubes [deg] = closed angle + 2 * handle rotation."""
    return p.ANGLE_INCLUDED_CLOSED_DEG + 2.0 * handle_rotation_deg(stroke_mm)


def theta_deg(stroke_mm):
    """Angle of the pivot->grip-centre line from vertical [deg], outward positive."""
    return p.THETA_CLOSED_DEG + handle_rotation_deg(stroke_mm)


def carriage_y_mm(stroke_mm):
    """Height of the carriage pin above the pivot line [mm]: y = Y0 + s / N_REEVE (no clamp: dwell)."""
    s = min(max(stroke_mm, 0.0), p.STROKE_DWELL_END_MM)
    return p.Y0_PIN_MM + s * p.CABLE_PAYOUT_PER_MM_TRAVEL / p.N_REEVE


# ------------------------------------------------------------------ hand spacing and torque
def grip_separation_mm(stroke_mm, delta_s_mm=p.DELTA_S_MM):
    """Grip-centre separation [mm] = P + 2 L sin(theta)."""
    L = p.line_length_mm(delta_s_mm)
    return p.pivot_spacing_mm(delta_s_mm) + 2.0 * L * math.sin(math.radians(theta_deg(stroke_mm)))


def hand_torque_nm(stroke_mm, f_hand_n, delta_s_mm=p.DELTA_S_MM):
    """Torque on ONE handle from a vertical hand force [N m], positive = CLOSING.

    T = F * L_LINE * sin(theta)   (theta < 0 means the grip is inboard of the pivot -> opening)
    """
    L = p.line_length_mm(delta_s_mm) / 1000.0
    return f_hand_n * L * math.sin(math.radians(theta_deg(stroke_mm)))


# ------------------------------------------------------------------ carriage force balance
def pin_force_n(stroke_mm, f_hand_n, delta_s_mm=p.DELTA_S_MM):
    """Axial force on the carriage from BOTH handles [N], positive = down (closing).

    Virtual work: F_pin * dy = 2 * T * dbeta  ->  F_pin = 2 T (dbeta/ds) / (dy/ds),  dy/ds = 1/N_REEVE.
    Goes to zero smoothly through the easing zone and is zero in the dwell.
    """
    t_nmm = hand_torque_nm(stroke_mm, f_hand_n, delta_s_mm) * 1000.0
    return 2.0 * t_nmm * dbeta_ds_rad_per_mm(stroke_mm) * p.N_REEVE / p.CABLE_PAYOUT_PER_MM_TRAVEL


def spring_force_n(stroke_mm):
    """Extension-spring force on the carriage [N], always down: preload + rate * carriage travel."""
    return p.SPRING_PRELOAD_N + p.SPRING_RATE_N_PER_MM * (carriage_y_mm(stroke_mm) - p.Y0_PIN_MM)


def friction_force_n(stroke_mm, f_hand_n, delta_s_mm=p.DELTA_S_MM):
    """Friction on the carriage [N] (ASSUMPTION): (1 - eta) * (spring + |pin|), opposes motion."""
    return (1.0 - p.ETA_MECH) * (spring_force_n(stroke_mm)
                                 + abs(pin_force_n(stroke_mm, f_hand_n, delta_s_mm)))


def cable_tension_n(stroke_mm, f_hand_n, direction, delta_s_mm=p.DELTA_S_MM):
    """Reference-cable tension [N] from the carriage force balance.

    Cable parts pull the carriage UP with N_REEVE * T.  Spring and (closing) pin force pull down.
      down stroke (carriage moving up, friction acts down):  N T = spring + pin + friction
      up stroke   (carriage moving down, friction acts up):  N T = spring + pin - friction
    A result <= 0 means the cable is slack: the hand load drives the carriage ahead of the cable.
    """
    f = friction_force_n(stroke_mm, f_hand_n, delta_s_mm)
    if direction == "up":
        f = -f
    return (spring_force_n(stroke_mm) + pin_force_n(stroke_mm, f_hand_n, delta_s_mm) + f) / p.N_REEVE


def stroke_points(step_mm=p.STROKE_TABLE_STEP_MM):
    """Stroke values [mm] 0 .. STROKE_DWELL_END in steps."""
    n = int(round(p.STROKE_DWELL_END_MM / step_mm))
    return [i * step_mm for i in range(n + 1)]


def first_slack_stroke_mm(f_hand_n, delta_s_mm=p.DELTA_S_MM):
    """First stroke [mm] where cable tension < CABLE_TENSION_MIN_N (either direction), or None."""
    for s in stroke_points(5.0):
        if min(cable_tension_n(s, f_hand_n, "down", delta_s_mm),
               cable_tension_n(s, f_hand_n, "up", delta_s_mm)) < p.CABLE_TENSION_MIN_N:
            return s
    return None


def r0_max_n():
    """No-load actuation resistance R0 [N]: max cable tension on the down stroke with F = 0."""
    return max(cable_tension_n(s, 0.0, "down") for s in stroke_points(5.0))


def r_load_max_n(f_hand_n=p.F_HAND_N, delta_s_mm=p.DELTA_S_MM):
    """Loaded actuation resistance [N]: max cable tension on the down stroke at the design hand load."""
    return max(cable_tension_n(s, f_hand_n, "down", delta_s_mm) for s in stroke_points(5.0))


# ------------------------------------------------------------------ slot profile (the cam)
def rotate(x, y, angle_deg):
    """Rotate the vector (x, y) counter-clockwise by angle_deg."""
    c, s = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    return c * x - s * y, s * x + c * y


def pin_rel_pivot_mm(stroke_mm, delta_s_mm=p.DELTA_S_MM):
    """Pin position relative to the right pivot, housing frame: (-P/2, y_pin)."""
    return -p.pivot_spacing_mm(delta_s_mm) / 2.0, carriage_y_mm(stroke_mm)


def slot_point_mm(stroke_mm, delta_s_mm=p.DELTA_S_MM):
    """Slot centre-line point (x, y) [mm] for the RIGHT arm, in the arm frame (origin = pivot,
    frame = housing frame at closed). The arm opens by rotating CCW by beta, so the pin, seen from
    the arm, is the housing-frame pin rotated by -beta:
        slot = Rot(-beta) . (pin - pivot)
    The left arm is the mirror image (x -> -x).
    """
    x, y = pin_rel_pivot_mm(stroke_mm, delta_s_mm)
    return rotate(x, y, -handle_rotation_deg(stroke_mm))


def slot_profile_mm(step_mm=5.0, delta_s_mm=p.DELTA_S_MM):
    """List of (stroke, x, y) slot centre-line points for the right arm, 0 .. STROKE_DWELL_END."""
    return [(s, ) + slot_point_mm(s, delta_s_mm) for s in stroke_points(step_mm)]


def slot_tangent_housing(stroke_mm, delta_s_mm=p.DELTA_S_MM, ds=0.5):
    """Unit tangent of the slot at the pin, expressed in the housing frame at stroke s."""
    s1 = max(stroke_mm - ds, 0.0)
    s2 = min(stroke_mm + ds, p.STROKE_DWELL_END_MM)
    x1, y1 = slot_point_mm(s1, delta_s_mm)
    x2, y2 = slot_point_mm(s2, delta_s_mm)
    tx, ty = rotate(x2 - x1, y2 - y1, handle_rotation_deg(stroke_mm))
    norm = math.hypot(tx, ty)
    return tx / norm, ty / norm


def pressure_angle_deg(stroke_mm, delta_s_mm=p.DELTA_S_MM):
    """Pressure angle [deg] = angle between the slot normal and the carriage travel (vertical)
    = angle of the slot tangent from horizontal. 90 deg in the dwell (no drive, by design)."""
    tx, ty = slot_tangent_housing(stroke_mm, delta_s_mm)
    return math.degrees(math.atan2(abs(ty), abs(tx)))


def slot_wall_force_n(stroke_mm, f_hand_n, delta_s_mm=p.DELTA_S_MM):
    """Normal force of ONE arm's slot wall on the roller [N].

    Frictionless slot: the wall force acts along the slot normal through the pin. Its moment about
    the pivot must balance the hand torque:  T = F_wall * |r_pin x n|  (r_pin = pin - pivot).
    Valid in the drive section (where it equals (F_pin/2)/cos(alpha)), in the easing and in the
    dwell (where the normal is horizontal and the lever is simply y_pin).
    """
    tx, ty = slot_tangent_housing(stroke_mm, delta_s_mm)
    nx, ny = -ty, tx
    rx, ry = pin_rel_pivot_mm(stroke_mm, delta_s_mm)
    lever_mm = abs(rx * ny - ry * nx)
    t_nmm = hand_torque_nm(stroke_mm, f_hand_n, delta_s_mm) * 1000.0
    return abs(t_nmm) / lever_mm


# ------------------------------------------------------------------ swept envelope
def arm_envelope_mm(delta_s_mm=p.DELTA_S_MM, step_mm=5.0):
    """Bounding box (xmin, xmax, ymin, ymax) [mm, housing frame] swept by the RIGHT arm over the
    whole stroke. The arm is modelled as the slot centre line thickened by (slot/2 + rim) plus the
    pivot boss. For each stroke s the whole slot is rotated by beta(s) about the pivot.
    """
    r_rim = p.SLOT_WIDTH_MM / 2.0 + p.ARM_RIM_MM
    px = p.pivot_spacing_mm(delta_s_mm) / 2.0
    slot = [(x, y) for _, x, y in slot_profile_mm(step_mm, delta_s_mm)]
    xmin, xmax = px - p.PIVOT_BOSS_MM, px + p.PIVOT_BOSS_MM
    ymin, ymax = -p.PIVOT_BOSS_MM, p.PIVOT_BOSS_MM
    for s in stroke_points(step_mm):
        beta = handle_rotation_deg(s)
        for x, y in slot:
            hx, hy = rotate(x, y, beta)
            hx += px
            xmin, xmax = min(xmin, hx - r_rim), max(xmax, hx + r_rim)
            ymin, ymax = min(ymin, hy - r_rim), max(ymax, hy + r_rim)
    return xmin, xmax, ymin, ymax


def housing_width_mm(delta_s_mm=p.DELTA_S_MM):
    """Housing outside width [mm] = 2 x (widest of: arm sweep, carriage, pivot boss) + clearance + walls."""
    _, xmax, _, _ = arm_envelope_mm(delta_s_mm)
    half = max(xmax, p.CARRIAGE_WIDTH_MM / 2.0)
    return 2.0 * (half + p.ARM_CLEARANCE_MM + p.HOUSING_WALL_MM)


def housing_height_above_pivots_mm(delta_s_mm=p.DELTA_S_MM):
    """Housing height above the pivot line [mm]: the taller of the arm sweep and the cable stack
    (pin at its highest + carriage pulley rows + fixed pulley + top clearance), plus the wall."""
    _, _, _, ymax_arm = arm_envelope_mm(delta_s_mm)
    rows = p.carriage_pulley_rows(p.N_REEVE)
    stack = (carriage_y_mm(p.STROKE_DWELL_END_MM) + rows * (p.PULLEY_DIAMETER_MM + p.PULLEY_GAP_MM)
             + p.PULLEY_DIAMETER_MM + p.TOP_CLEARANCE_MM)
    return max(ymax_arm, stack) + p.HOUSING_WALL_MM


def drive_section_points():
    """Stroke points in the constant-slope part of the drive (before the easing zone)."""
    s_end = p.STROKE_MAP_MM * (1.0 - p.EASE_FRACTION)
    return [s for s in stroke_points(5.0) if s <= s_end]


# ------------------------------------------------------------------ architecture A reference (not selected)
def required_ratio(spool_radius_mm):
    """Spool->handle reduction for a spool drive: ratio = k / r_spool (k = 573 mm/rad)."""
    return p.K_MM_PER_RAD / spool_radius_mm


def stages_needed(ratio):
    """Smallest n with (max ratio per stage)^n >= ratio."""
    n = 1
    while p.GEAR_MAX_STAGE_RATIO ** n < ratio:
        n += 1
    return n


# ------------------------------------------------------------------ report helpers
def tag(ok):
    """PASS / FAIL text."""
    return "PASS" if ok else "FAIL"


def layout_summary(delta_s_mm=p.DELTA_S_MM):
    """Dict of the numbers the trade-off table needs for the current layout."""
    xmin, xmax, _, ymax = arm_envelope_mm(delta_s_mm)
    return dict(
        width=housing_width_mm(delta_s_mm), height=housing_height_above_pivots_mm(delta_s_mm),
        arm_x_out=xmax, arm_x_in=xmin, arm_y=ymax,
        pa=max(pressure_angle_deg(s, delta_s_mm) for s in drive_section_points()),
        r0=r0_max_n(), rload=r_load_max_n(p.F_HAND_N, delta_s_mm),
        wall=max(slot_wall_force_n(s, p.F_HAND_N, delta_s_mm) for s in stroke_points(5.0)),
        slack=first_slack_stroke_mm(p.F_HAND_N, delta_s_mm), eta=p.ETA_MECH)


def print_geometry():
    """Table: bell-crank geometry and real envelope for each DELTA_S in the study (baseline layout)."""
    print("1) Bell-crank geometry, baseline layout N_reeve = %d, Y0 = %.0f mm (S_closed = %.0f, theta %.0f -> %.0f deg, F = %.0f N)"
          % (p.N_REEVE, p.Y0_PIN_MM, p.S_CLOSED_MM, p.THETA_CLOSED_DEG, p.THETA_OPEN_DEG, p.F_HAND_N))
    print("   dS mm | L_line | pivot P | S open | T open | arm x out | arm x in | width | height | pressure")
    for dS in p.DELTA_S_STUDY_MM:
        d = layout_summary(dS)
        print("   %5.0f | %6.1f | %7.1f | %6.1f | %6.2f | %9.1f | %8.1f | %5.0f | %6.0f | %5.1f"
              % (dS, p.line_length_mm(dS), p.pivot_spacing_mm(dS), grip_separation_mm(p.STROKE_MAP_MM, dS),
                 hand_torque_nm(p.STROKE_MAP_MM, p.F_HAND_N, dS), d["arm_x_out"], d["arm_x_in"],
                 d["width"], d["height"], d["pa"]))
    print("   (arm x = extreme of the RIGHT arm from the centre line over the stroke; width/height are housing outsides)")


def print_tradeoff():
    """Table: width / height / pressure angle / R0 / R_load / wall load for every layout in the study."""
    print("\n2) Layout trade-off, dS = %.0f mm, F = %.0f N (eta = 0.90 x 0.97^pulleys)" % (p.DELTA_S_MM, p.F_HAND_N))
    print("   n | Y0  | eta  | width | height | arm x out | pressure | R0  | R_load | wall max | slack | width <= %.0f"
          % p.HOUSING_WIDTH_MAX_MM)
    for n in p.N_REEVE_STUDY:
        for y0 in p.Y0_STUDY_MM:
            set_layout(n, y0)
            d = layout_summary()
            print("   %d | %3.0f | %.2f | %5.0f | %6.0f | %9.1f | %8.1f | %4.1f | %6.1f | %8.0f | %5s | %s"
                  % (n, y0, d["eta"], d["width"], d["height"], d["arm_x_out"], d["pa"], d["r0"], d["rload"],
                     d["wall"], "none" if d["slack"] is None else "%.0f" % d["slack"],
                     tag(d["width"] <= p.HOUSING_WIDTH_MAX_MM)))
    restore_layout()


def print_angle_table():
    """Table: angle, theta, carriage position vs travel."""
    print("\n3) Angle vs travel (profile %s, ease %.0f%%, map %.0f mm, dwell to %.0f mm, N_reeve = %d)"
          % (p.PROFILE, 100 * p.EASE_FRACTION, p.STROKE_MAP_MM, p.STROKE_DWELL_END_MM, p.N_REEVE))
    print("   s mm | included | per handle | dbeta/ds deg/mm | theta | carriage y | separation")
    for s in stroke_points(25.0)[::2]:
        print("   %4.0f | %8.1f | %10.1f | %15.3f | %5.1f | %10.1f | %10.1f"
              % (s, included_angle_deg(s), handle_rotation_deg(s), math.degrees(dbeta_ds_rad_per_mm(s)),
                 theta_deg(s), carriage_y_mm(s), grip_separation_mm(s)))


def print_force_balance():
    """Table: carriage force balance, cable tension and slot wall load for the study grid."""
    print("\n4) Carriage force balance, cable tension [N] (eta = %.2f, spring %.0f N + %.2f N/mm)"
          % (p.ETA_MECH, p.SPRING_PRELOAD_N, p.SPRING_RATE_N_PER_MM))
    print("   pin: + = down (closing). T_down / T_up = cable tension on down / up stroke. wall = one arm's slot-wall load.")
    for dS in p.DELTA_S_STUDY_MM:
        for F in p.F_HAND_STUDY_N:
            slack = first_slack_stroke_mm(F, dS)
            print("   dS = %.0f mm, F_hand = %3.0f N  ->  first slack at: %s"
                  % (dS, F, ("%.0f mm" % slack) if slack is not None else "never"))
            print("      s mm | T_hand | pin | spring | fric | T_down | T_up | wall | pressure")
            for s in stroke_points(25.0):
                print("      %4.0f | %+6.2f | %+5.0f | %6.1f | %4.1f | %6.1f | %5.1f | %4.0f | %5.1f"
                      % (s, hand_torque_nm(s, F, dS), pin_force_n(s, F, dS), spring_force_n(s),
                         friction_force_n(s, F, dS), cable_tension_n(s, F, "down", dS),
                         cable_tension_n(s, F, "up", dS), slot_wall_force_n(s, F, dS), pressure_angle_deg(s, dS)))


def print_slot():
    """Table: slot centre-line points for the baseline DELTA_S."""
    print("\n5) Slot centre line, right arm, arm frame (origin = pivot), dS = %.0f mm, slot width %.1f mm"
          % (p.DELTA_S_MM, p.SLOT_WIDTH_MM))
    print("   s mm |    x    |    y    | r from pivot | pressure angle")
    for s, x, y in slot_profile_mm(step_mm=25.0):
        note = "  (dwell)" if s > p.STROKE_MAP_MM else ("  (easing)" if s > p.STROKE_MAP_MM * (1 - p.EASE_FRACTION) else "")
        print("   %4.0f | %7.1f | %7.1f | %12.1f | %6.1f%s" % (s, x, y, math.hypot(x, y), pressure_angle_deg(s), note))


def print_checks():
    """PASS/FAIL checks. R0 and R_load are always shown together."""
    print("\n6) Checks (baseline: N_reeve = %d, Y0 = %.0f, dS = %.0f)" % (p.N_REEVE, p.Y0_PIN_MM, p.DELTA_S_MM))
    s_closed = grip_separation_mm(0.0)
    print("   Hand spacing closed = %.1f mm, need >= %.1f -> %s"
          % (s_closed, p.HAND_SPACING_MIN_MM, tag(s_closed >= p.HAND_SPACING_MIN_MM - 1e-6)))
    slack_any = [F for F in p.F_HAND_STUDY_N if first_slack_stroke_mm(F) is not None]
    print("   Cable never slack (T >= %.0f N) for F_hand in %s -> %s%s"
          % (p.CABLE_TENSION_MIN_N, p.F_HAND_STUDY_N, tag(not slack_any),
             ("  (slack at F = %s)" % slack_any) if slack_any else ""))
    r0, rl = r0_max_n(), r_load_max_n()
    lim = p.R_LOAD_FRACTION_MAX * p.STACK_LOAD_N
    print("   R0 (no load) = %.1f N, goal <= %.0f -> %s   |   R_load (F = %.0f N) = %.1f N, goal <= %.0f -> %s (goal TODO)"
          % (r0, p.R0_MAX_N, tag(r0 <= p.R0_MAX_N), p.F_HAND_N, rl, lim, tag(rl <= lim)))
    w, h = housing_width_mm(), housing_height_above_pivots_mm()
    print("   Housing width (swept envelope) = %.0f mm, target <= %.0f -> %s;  height above pivots = %.0f mm (no target yet, Q12)"
          % (w, p.HOUSING_WIDTH_MAX_MM, tag(w <= p.HOUSING_WIDTH_MAX_MM), h))
    pa = max(pressure_angle_deg(s) for s in drive_section_points())
    print("   Pressure angle max (constant-slope section) = %.1f deg, limit %.0f -> %s"
          % (pa, p.PRESSURE_ANGLE_MAX_DEG, tag(pa <= p.PRESSURE_ANGLE_MAX_DEG)))
    a0, a1, a2 = included_angle_deg(0), included_angle_deg(p.STROKE_MAP_MM), included_angle_deg(p.STROKE_DWELL_END_MM)
    ok = abs(a0 - p.ANGLE_INCLUDED_CLOSED_DEG) < 1e-6 and abs(a1 - p.ANGLE_INCLUDED_OPEN_DEG) < 1e-6 and abs(a2 - a1) < 1e-6
    print("   Handle angle range: %.0f / %.0f / %.0f deg at 0 / %.0f / %.0f mm -> %s"
          % (a0, a1, a2, p.STROKE_MAP_MM, p.STROKE_DWELL_END_MM, tag(ok)))


def print_arch_a_reference():
    """Short reference table for the spool + gear-train architecture (not selected)."""
    print("\n7) Reference only - architecture A (spool + gears, not selected): ratio = %.0f / r" % p.K_MM_PER_RAD)
    for r in p.spool_radius_candidates():
        print("   r = %2.0f mm: ratio %.1f, %d stages at max %.0f:1 per stage"
              % (r, required_ratio(r), stages_needed(required_ratio(r)), p.GEAR_MAX_STAGE_RATIO))


def main():
    """Print all tables and checks."""
    print("autoRAD kinematics (architecture E) - PRELIMINARY, not a validation\n")
    print_geometry()
    print_tradeoff()
    print_angle_table()
    print_force_balance()
    print_slot()
    print_checks()
    print_arch_a_reference()


if __name__ == "__main__":
    main()
