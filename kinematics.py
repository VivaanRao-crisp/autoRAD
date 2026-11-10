"""kinematics.py - angle mapping, slot profile, carriage force balance and checks for autoRAD.

Run:  python3 kinematics.py
All inputs come from params.py. These are preliminary calculations, not validation.

Sign conventions (right-hand handle, looking at the user's frontal plane):
  x = outward (+ to the right), y = up. Pivot of the right handle at (+P/2, 0).
  The carriage pin sits on the centre line at (0, y_pin), y_pin = Y0 + s / N_REEVE.
  Opening = the grip (below the pivot) moves outward = counter-clockwise rotation of the handle.
  Pin force: positive = DOWN on the carriage (closing direction, same sense as the spring).
"""
import math
import params as p


# ------------------------------------------------------------------ angle mapping
def stroke_fraction(stroke_mm):
    """Fraction 0..1 of the mapping stroke, clamped (the dwell holds 1.0)."""
    return min(max(stroke_mm / p.STROKE_MAP_MM, 0.0), 1.0)


def profile(fraction):
    """Map stroke fraction -> opening fraction. v1 linear; other profiles go here (one parameter)."""
    if p.PROFILE == "linear":
        return fraction
    raise NotImplementedError("TODO: profile '%s' not defined" % p.PROFILE)


def handle_rotation_deg(stroke_mm):
    """Rotation of ONE handle from closed [deg] = 30 deg * profile(fraction)."""
    return p.ANGLE_PER_HANDLE_DEG * profile(stroke_fraction(stroke_mm))


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

    Virtual work: F_pin * dy = 2 * T * d(beta)  ->  F_pin = 2 T / (dy/dbeta).
    In the dwell the slot runs parallel to the carriage, so the axial force is zero.
    """
    if stroke_mm > p.STROKE_MAP_MM:      # s = STROKE_MAP is the last point of the drive section
        return 0.0
    return 2.0 * hand_torque_nm(stroke_mm, f_hand_n, delta_s_mm) / (p.DY_DBETA_MM_PER_RAD / 1000.0)


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


def stroke_points():
    """Stroke values [mm] used in the tables: 0 .. STROKE_DWELL_END in STROKE_TABLE_STEP steps."""
    n = int(round(p.STROKE_DWELL_END_MM / p.STROKE_TABLE_STEP_MM))
    return [i * p.STROKE_TABLE_STEP_MM for i in range(n + 1)]


def first_slack_stroke_mm(f_hand_n, delta_s_mm=p.DELTA_S_MM):
    """First stroke [mm] where the cable tension drops below CABLE_TENSION_MIN_N (either direction), or None."""
    for s in stroke_points():
        if min(cable_tension_n(s, f_hand_n, "down", delta_s_mm),
               cable_tension_n(s, f_hand_n, "up", delta_s_mm)) < p.CABLE_TENSION_MIN_N:
            return s
    return None


def r0_max_n():
    """No-load actuation resistance R0 [N]: max cable tension on the down stroke with F = 0."""
    return max(cable_tension_n(s, 0.0, "down") for s in stroke_points())


def r_load_max_n(f_hand_n=p.F_HAND_N, delta_s_mm=p.DELTA_S_MM):
    """Loaded actuation resistance [N]: max cable tension on the down stroke at the design hand load."""
    return max(cable_tension_n(s, f_hand_n, "down", delta_s_mm) for s in stroke_points())


# ------------------------------------------------------------------ slot profile (the cam)
def rotate(x, y, angle_deg):
    """Rotate the vector (x, y) counter-clockwise by angle_deg."""
    c, s = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    return c * x - s * y, s * x + c * y


def slot_point_mm(stroke_mm, delta_s_mm=p.DELTA_S_MM):
    """Slot centre-line point (x, y) [mm] for the RIGHT arm, in the arm frame (origin = pivot,
    frame = housing frame at closed). The arm opens by rotating CCW by phi, so the pin, seen from
    the arm, is the housing-frame pin rotated by -phi:
        slot = Rot(-phi) . (pin - pivot),   pin - pivot = (-P/2, y_pin)
    The left arm is the mirror image (x -> -x).
    """
    P = p.pivot_spacing_mm(delta_s_mm)
    return rotate(-P / 2.0, carriage_y_mm(stroke_mm), -handle_rotation_deg(stroke_mm))


def slot_profile_mm(step_mm=5.0, delta_s_mm=p.DELTA_S_MM):
    """List of (stroke, x, y) slot centre-line points for the right arm, 0 .. STROKE_DWELL_END."""
    pts = []
    s = 0.0
    while s <= p.STROKE_DWELL_END_MM + 1e-9:
        x, y = slot_point_mm(s, delta_s_mm)
        pts.append((s, x, y))
        s += step_mm
    return pts


def pressure_angle_deg(stroke_mm, delta_s_mm=p.DELTA_S_MM, ds=0.5):
    """Pressure angle [deg] = angle between the slot normal and the carriage travel (vertical).

    Computed numerically: the slot tangent (in the arm frame) is d(slot)/ds; rotate it back to the
    housing frame. The angle of that tangent from HORIZONTAL equals the normal's angle from VERTICAL.
    In the dwell the tangent is vertical -> 90 deg (no drive, by design).
    """
    if stroke_mm + ds > p.STROKE_DWELL_END_MM:   # at the very end, difference backwards
        stroke_mm -= ds
    x1, y1 = slot_point_mm(stroke_mm, delta_s_mm)
    x2, y2 = slot_point_mm(stroke_mm + ds, delta_s_mm)
    tx, ty = rotate(x2 - x1, y2 - y1, handle_rotation_deg(stroke_mm))
    return math.degrees(math.atan2(abs(ty), abs(tx)))


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


def print_geometry():
    """Table: bell-crank geometry for each DELTA_S in the study."""
    print("1) Bell-crank geometry (S_closed = %.0f mm, theta %.0f -> %.0f deg, F_hand = %.0f N)"
          % (p.S_CLOSED_MM, p.THETA_CLOSED_DEG, p.THETA_OPEN_DEG, p.F_HAND_N))
    print("   dS mm | L_line | pivot P | S open | width | T closed | T open (per handle, N m)")
    for dS in p.DELTA_S_STUDY_MM:
        print("   %5.0f | %6.1f | %7.1f | %6.1f | %5.1f | %+8.2f | %+6.2f"
              % (dS, p.line_length_mm(dS), p.pivot_spacing_mm(dS), grip_separation_mm(p.STROKE_MAP_MM, dS),
                 p.housing_width_mm(dS), hand_torque_nm(0.0, p.F_HAND_N, dS),
                 hand_torque_nm(p.STROKE_MAP_MM - 1e-6, p.F_HAND_N, dS)))


def print_angle_table():
    """Table: angle, theta, carriage position vs travel."""
    print("\n2) Angle vs travel (map %.0f mm, dwell to %.0f mm, N_reeve = %d)"
          % (p.STROKE_MAP_MM, p.STROKE_DWELL_END_MM, p.N_REEVE))
    print("   s mm | included | per handle | theta | carriage y | separation")
    for s in stroke_points()[::3]:
        print("   %4.0f | %8.1f | %10.1f | %5.1f | %10.1f | %10.1f"
              % (s, included_angle_deg(s), handle_rotation_deg(s), theta_deg(s),
                 carriage_y_mm(s), grip_separation_mm(s)))


def print_force_balance():
    """Table: carriage force balance and cable tension for the study grid."""
    print("\n3) Carriage force balance, cable tension [N] (eta = %.2f, spring %.0f N + %.2f N/mm)"
          % (p.ETA_MECH, p.SPRING_PRELOAD_N, p.SPRING_RATE_N_PER_MM))
    print("   pin force: + = down (closing).  T_down / T_up = cable tension on down / up stroke.")
    for dS in p.DELTA_S_STUDY_MM:
        for F in p.F_HAND_STUDY_N:
            slack = first_slack_stroke_mm(F, dS)
            print("   dS = %.0f mm, F_hand = %3.0f N  ->  first slack at: %s"
                  % (dS, F, ("%.0f mm" % slack) if slack is not None else "never"))
            print("      s mm | T_hand | pin | spring | fric | T_down | T_up")
            for s in stroke_points()[::2]:
                print("      %4.0f | %+6.2f | %+5.0f | %6.1f | %4.1f | %6.1f | %5.1f"
                      % (s, hand_torque_nm(s, F, dS), pin_force_n(s, F, dS), spring_force_n(s),
                         friction_force_n(s, F, dS), cable_tension_n(s, F, "down", dS),
                         cable_tension_n(s, F, "up", dS)))


def print_slot():
    """Table: slot centre-line points and pressure angle for the baseline DELTA_S."""
    print("\n4) Slot centre line, right arm, arm frame (origin = pivot), dS = %.0f mm, slot width %.1f mm"
          % (p.DELTA_S_MM, p.SLOT_WIDTH_MM))
    print("   s mm |    x    |    y    | r from pivot | pressure angle")
    for s, x, y in slot_profile_mm(step_mm=25.0):
        print("   %4.0f | %7.1f | %7.1f | %12.1f | %6.1f%s"
              % (s, x, y, math.hypot(x, y), pressure_angle_deg(s),
                 "  (dwell)" if s >= p.STROKE_MAP_MM else ""))


def print_checks():
    """PASS/FAIL checks."""
    print("\n5) Checks")
    s_closed = grip_separation_mm(0.0)
    print("   Hand spacing closed = %.1f mm, need >= %.1f -> %s"
          % (s_closed, p.HAND_SPACING_MIN_MM, tag(s_closed >= p.HAND_SPACING_MIN_MM - 1e-6)))
    slack_any = [F for F in p.F_HAND_STUDY_N if first_slack_stroke_mm(F) is not None]
    print("   Cable never slack (T >= %.0f N) for F_hand in %s, dS = %.0f -> %s%s"
          % (p.CABLE_TENSION_MIN_N, p.F_HAND_STUDY_N, p.DELTA_S_MM, tag(not slack_any),
             ("  (slack at F = %s)" % slack_any) if slack_any else ""))
    r0 = r0_max_n()
    print("   R0 (no load) max = %.1f N, goal <= %.0f -> %s" % (r0, p.R0_MAX_N, tag(r0 <= p.R0_MAX_N)))
    rl = r_load_max_n()
    lim = p.R_LOAD_FRACTION_MAX * p.STACK_LOAD_N
    print("   R_load max = %.1f N at F_hand = %.0f N, goal <= %.0f%% of %.0f N stack = %.0f N -> %s (TODO confirm goal)"
          % (rl, p.F_HAND_N, 100 * p.R_LOAD_FRACTION_MAX, p.STACK_LOAD_N, lim, tag(rl <= lim)))
    w = p.HOUSING_WIDTH_MM
    print("   Housing width proxy = %.1f mm, target <= %.0f -> %s" % (w, p.HOUSING_WIDTH_MAX_MM, tag(w <= p.HOUSING_WIDTH_MAX_MM)))
    pa = max(pressure_angle_deg(s) for s in stroke_points() if s < p.STROKE_MAP_MM)
    print("   Pressure angle max (drive section) = %.1f deg, limit %.0f -> %s"
          % (pa, p.PRESSURE_ANGLE_MAX_DEG, tag(pa <= p.PRESSURE_ANGLE_MAX_DEG)))
    a0, a1, a2 = included_angle_deg(0), included_angle_deg(p.STROKE_MAP_MM), included_angle_deg(p.STROKE_DWELL_END_MM)
    ok = abs(a0 - p.ANGLE_INCLUDED_CLOSED_DEG) < 1e-9 and abs(a1 - p.ANGLE_INCLUDED_OPEN_DEG) < 1e-9 and abs(a2 - a1) < 1e-9
    print("   Handle angle range: %.0f / %.0f / %.0f deg at 0 / %.0f / %.0f mm -> %s"
          % (a0, a1, a2, p.STROKE_MAP_MM, p.STROKE_DWELL_END_MM, tag(ok)))


def print_arch_a_reference():
    """Short reference table for the spool + gear-train architecture (not selected)."""
    print("\n6) Reference only - architecture A (spool + gears, not selected): ratio = %.0f / r" % p.K_MM_PER_RAD)
    for r in p.spool_radius_candidates():
        print("   r = %2.0f mm: ratio %.1f, %d stages at max %.0f:1 per stage"
              % (r, required_ratio(r), stages_needed(required_ratio(r)), p.GEAR_MAX_STAGE_RATIO))


def main():
    """Print all tables and checks."""
    print("autoRAD kinematics (architecture E) - PRELIMINARY, not a validation\n")
    print_geometry()
    print_angle_table()
    print_force_balance()
    print_slot()
    print_checks()
    print_arch_a_reference()


if __name__ == "__main__":
    main()
