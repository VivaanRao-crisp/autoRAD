"""kinematics.py - handle angle, spool rotation, gear ratio and checks for autoRAD.

Run:  python3 kinematics.py
All inputs come from params.py. Results are preliminary calculations, not validation.
"""
import math
import params as p


# ------------------------------------------------------------------ handle motion
def included_angle_deg(stroke_mm):
    """Included angle between handles [deg] at a given attachment travel [mm].

    Linear map: fraction f = stroke / STROKE_MAP_MM, clamped to 0..1 (hard stop at 110 deg).
        angle = closed + f * (open - closed)
    """
    f = min(max(stroke_mm / p.STROKE_MAP_MM, 0.0), 1.0)
    return p.ANGLE_INCLUDED_CLOSED_DEG + f * (p.ANGLE_INCLUDED_OPEN_DEG - p.ANGLE_INCLUDED_CLOSED_DEG)


def handle_rotation_deg(stroke_mm):
    """Rotation of ONE handle from its closed position [deg] (half the included-angle change)."""
    return (included_angle_deg(stroke_mm) - p.ANGLE_INCLUDED_CLOSED_DEG) / 2.0


# ------------------------------------------------------------------ spool and ratio
def spool_rotation_rad(stroke_mm, spool_radius_mm):
    """Spool rotation [rad] for a given travel.

    Cable payout = stroke * CABLE_PAYOUT_PER_MM_TRAVEL; arc length = radius * angle, so
        theta_spool = payout / radius
    """
    return stroke_mm * p.CABLE_PAYOUT_PER_MM_TRAVEL / spool_radius_mm


def required_ratio(spool_radius_mm, stroke_mm=None):
    """Total reduction ratio spool -> one handle (spool turns per handle turn).

        ratio = theta_spool(full stroke) / theta_handle(full opening)
    Example r = 12 mm: (300/12) rad / (30 deg = 0.5236 rad) = 25 / 0.5236 = 47.7
    """
    if stroke_mm is None:
        stroke_mm = p.STROKE_MAP_MM
    handle_rad = math.radians(p.ANGLE_PER_HANDLE_DEG)
    return spool_rotation_rad(stroke_mm, spool_radius_mm) / handle_rad


# ------------------------------------------------------------------ gear train planning
def stages_needed(ratio):
    """Smallest number of stages n so that (max ratio per stage)^n >= ratio. Minimum 1."""
    n = 1
    while p.GEAR_MAX_STAGE_RATIO ** n < ratio:
        n += 1
    return n


def plan_gear_train(ratio):
    """Suggest tooth counts [(pinion, gear), ...] for a compound reduction train.

    Method: n stages, every pinion has GEAR_MIN_TEETH. Stages 1..n-1 get ratio^(1/n)
    (rounded to whole teeth). The LAST stage is sized to make up whatever is left, so the
    rounding error does not pile up. Returns (stages, achieved_ratio).
    """
    n = stages_needed(ratio)
    z_p = p.GEAR_MIN_TEETH
    stages = []
    achieved = 1.0
    for _ in range(n - 1):
        z_g = round(z_p * ratio ** (1.0 / n))
        stages.append((z_p, z_g))
        achieved *= z_g / z_p
    z_g_last = round(z_p * ratio / achieved)
    stages.append((z_p, z_g_last))
    achieved *= z_g_last / z_p
    return stages, achieved


def stage_centre_distance_mm(z_pinion, z_gear):
    """Centre distance of a spur pair [mm] = module * (z1 + z2) / 2."""
    return p.GEAR_MODULE_MM * (z_pinion + z_gear) / 2.0


def largest_gear_diameter_mm(stages):
    """Largest pitch diameter [mm] in the train = module * teeth."""
    return p.GEAR_MODULE_MM * max(z_g for _, z_g in stages)


# ------------------------------------------------------------------ checks
def grip_centre_separation_mm(included_deg):
    """Distance between the two grip centres [mm].

    Each grip centre sits GRIP_CENTRE_FROM_PIVOT_MM from its pivot, each handle at half the
    included angle from the centre line (handles symmetric). Pivots are PIVOT_SPACING apart:
        separation = pivot_spacing + 2 * L * sin(included / 2)
    """
    half = math.radians(included_deg / 2.0)
    return p.HANDLE_PIVOT_SPACING_MM + 2.0 * p.GRIP_CENTRE_FROM_PIVOT_MM * math.sin(half)


def check_hand_spacing():
    """Return (passed, separation_mm) at the closed angle."""
    sep = grip_centre_separation_mm(p.ANGLE_INCLUDED_CLOSED_DEG)
    return sep >= p.HAND_SPACING_MIN_MM, sep


def housing_width_proxy_mm(spool_radius_mm, stages):
    """PROXY for housing width [mm] (no layout yet): widest round part + 2 walls + 2 clearances."""
    widest = max(2.0 * spool_radius_mm, largest_gear_diameter_mm(stages))
    return widest + 2.0 * p.MIN_WALL_MM + 2.0 * p.MOVING_CLEARANCE_MM


def check_angle_range():
    """Return (passed, details): 0 travel -> closed angle, full map -> open angle,
    beyond the map -> still the open angle (hard stop)."""
    a0 = included_angle_deg(0.0)
    a1 = included_angle_deg(p.STROKE_MAP_MM)
    a2 = included_angle_deg(p.STROKE_MAP_MM * p.STROKE_BEYOND_MAP_PERCENT / 100.0)
    ok = (abs(a0 - p.ANGLE_INCLUDED_CLOSED_DEG) < 1e-9
          and abs(a1 - p.ANGLE_INCLUDED_OPEN_DEG) < 1e-9
          and abs(a2 - p.ANGLE_INCLUDED_OPEN_DEG) < 1e-9)
    return ok, (a0, a1, a2)


def tag(ok):
    """PASS / FAIL text."""
    return "PASS" if ok else "FAIL"


# ------------------------------------------------------------------ command-line report
def main():
    """Print all tables and checks."""
    print("autoRAD kinematics - PRELIMINARY, not a validation\n")

    print("1) Handle angle vs attachment travel (map stroke = %.0f mm)" % p.STROKE_MAP_MM)
    print("   % stroke | travel mm | included deg | per-handle deg")
    for pct in p.STROKE_CHECK_PERCENTS + (p.STROKE_BEYOND_MAP_PERCENT,):
        s = p.STROKE_MAP_MM * pct / 100.0
        print("   %8d | %9.1f | %12.1f | %14.1f" % (pct, s, included_angle_deg(s), handle_rotation_deg(s)))

    print("\n2) Spool, ratio, gear train per candidate spool radius")
    print("   r mm | spool rev | ratio | stages | teeth (pinion/gear per stage)  | achieved | err %  | widest part+walls mm | width")
    rows = []
    for r in p.spool_radius_candidates():
        rev = spool_rotation_rad(p.STROKE_MAP_MM, r) / (2 * math.pi)
        ratio = required_ratio(r)
        stages, got = plan_gear_train(ratio)
        w = housing_width_proxy_mm(r, stages)
        teeth = " ".join("%d/%d" % s for s in stages)
        err = 100.0 * (got - ratio) / ratio
        wtxt = tag(w <= p.HOUSING_WIDTH_MAX_MM)
        print("   %4.0f | %9.2f | %5.1f | %6d | %-30s | %8.2f | %+5.1f | %20.1f | %s"
              % (r, rev, ratio, len(stages), teeth, got, err, w, wtxt))
        rows.append((r, w))
    print("   (module = %.2f mm, min pinion = %d teeth, max stage ratio = %.1f; all TODO placeholders)"
          % (p.GEAR_MODULE_MM, p.GEAR_MIN_TEETH, p.GEAR_MAX_STAGE_RATIO))

    print("\n3) Ratio sensitivity to the mapping stroke (design range)")
    print("   r mm | " + " | ".join("%4.0f mm" % s for s in
          (p.STROKE_RANGE_MIN_MM, p.STROKE_MAP_MM, p.STROKE_RANGE_MAX_MM)))
    for r in p.spool_radius_candidates():
        vals = [required_ratio(r, s) for s in (p.STROKE_RANGE_MIN_MM, p.STROKE_MAP_MM, p.STROKE_RANGE_MAX_MM)]
        print("   %4.0f | " % r + " | ".join("%7.1f" % v for v in vals))

    print("\n4) Checks")
    ok, sep = check_hand_spacing()
    print("   Hand spacing: grip-centre separation closed = %.1f mm, need >= %.1f mm -> %s"
          % (sep, p.HAND_SPACING_MIN_MM, tag(ok)))
    print("     (assumes pivot spacing = %.1f mm, TODO)" % p.HANDLE_PIVOT_SPACING_MM)
    ok, (a0, a1, a2) = check_angle_range()
    print("   Handle angle range: 0 mm -> %.1f, %.0f mm -> %.1f, %d%% -> %.1f deg -> %s"
          % (a0, p.STROKE_MAP_MM, a1, p.STROKE_BEYOND_MAP_PERCENT, a2, tag(ok)))
    n_ok = sum(1 for _, w in rows if w <= p.HOUSING_WIDTH_MAX_MM)
    print("   Housing width (PROXY, limit %.0f mm): %d of %d radii pass -> see column above"
          % (p.HOUSING_WIDTH_MAX_MM, n_ok, len(rows)))
    print("   Actuation resistance <= %.0f N: TODO (definition TBD)" % p.ACTUATION_RESISTANCE_MAX_N)


if __name__ == "__main__":
    main()
