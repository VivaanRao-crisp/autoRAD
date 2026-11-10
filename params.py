"""params.py - every dimension and design variable for autoRAD.

ALL VALUES ARE PRELIMINARY. Units: mm, degrees, N unless the comment says otherwise.
Items marked TODO are unknown and need a decision from the team.
"""
import math

# ---------------------------------------------------------------- handle motion
ANGLE_INCLUDED_CLOSED_DEG = 50.0    # [deg] included angle between the two handles, closed
ANGLE_INCLUDED_OPEN_DEG = 110.0     # [deg] included angle, fully open

# ---------------------------------------------------------------- stroke mapping
STROKE_MAP_MM = 300.0               # [mm] attachment travel that maps to full opening
STROKE_RANGE_MIN_MM = 250.0         # [mm] design range, lower end
STROKE_RANGE_MAX_MM = 400.0         # [mm] design range, upper end
STROKE_CHECK_PERCENTS = (0, 25, 50, 75, 100)  # [% of mapping stroke] points printed in the table
STROKE_BEYOND_MAP_PERCENT = 150     # [% of mapping stroke] extra point to show the hard stop holds
# TODO: cable payout per mm of attachment travel. 1.0 assumes the reference cable runs straight
# from the frame anchor to the spool (no pulley/multiplier). Confirm with the real cable routing.
CABLE_PAYOUT_PER_MM_TRAVEL = 1.0    # [-]

# ---------------------------------------------------------------- grip / handle
GRIP_DIAMETER_MM = 32.0             # [mm]
GRIP_LENGTH_MIN_MM = 115.0          # [mm]
GRIP_LENGTH_MAX_MM = 150.0          # [mm]
GRIP_CENTRE_FROM_PIVOT_MM = 127.0   # [mm] distance pivot -> grip centre, along the handle
# TODO: distance between the two handle pivot axes. 0 = both handles pivot about one common
# point (pure V). Real value depends on the gear layout.
HANDLE_PIVOT_SPACING_MM = 0.0       # [mm]

# ---------------------------------------------------------------- hand-spacing check
HAND_BREADTH_P95_MALE_MM = 96.0     # [mm] 95th percentile male hand breadth (given in brief)
HAND_CLEARANCE_MM = 10.0            # [mm] extra clearance between hands (given in brief)

# ---------------------------------------------------------------- housing / printing
HOUSING_WIDTH_MAX_MM = 140.0        # [mm]
MIN_WALL_MM = 2.0                   # [mm] FDM minimum wall
MOVING_CLEARANCE_MM = 0.3           # [mm] clearance on moving fits
# TODO: actual housing width needs a layout. Until then kinematics.py uses a PROXY:
# largest gear (or spool) diameter + 2 walls + 2 clearances.

# ---------------------------------------------------------------- spool
SPOOL_RADIUS_MIN_MM = 12.0          # [mm] candidate range, low end
SPOOL_RADIUS_MAX_MM = 30.0          # [mm] candidate range, high end
SPOOL_RADIUS_STEP_MM = 3.0          # [mm] step between candidates (my choice, change freely)
# TODO: radius is taken as the effective radius at which cable leaves the spool
# (ignores cable thickness and layering).

# ---------------------------------------------------------------- gear train (planning only)
GEAR_MODULE_MM = 1.0                # [mm] TODO: pick for FDM strength; 1.0 is a placeholder
GEAR_MIN_TEETH = 12                 # [-] TODO: min pinion teeth; avoids undercut at 20 deg PA
GEAR_MAX_STAGE_RATIO = 6.0          # [-] TODO: max reduction per stage; a design preference
# TODO: pressure angle, face width, and how the final stage drives both handles symmetrically
# (assumed: two meshing sector gears, 1:1 between handles, so only the spool->one-handle
# ratio matters).

# ---------------------------------------------------------------- performance goals
ACTUATION_RESISTANCE_MAX_N = 10.0   # [N] goal; TODO: definition of "actuation resistance" is TBD

# ================================================================ derived values
ANGLE_PER_HANDLE_DEG = (ANGLE_INCLUDED_OPEN_DEG - ANGLE_INCLUDED_CLOSED_DEG) / 2.0
# each handle swings half of the change in included angle: (110 - 50) / 2 = 30 deg

HAND_SPACING_MIN_MM = HAND_BREADTH_P95_MALE_MM + HAND_CLEARANCE_MM   # 96 + 10 = 106 mm


def spool_radius_candidates():
    """Return the list of candidate spool radii [mm] from MIN to MAX in STEP increments."""
    radii = []
    r = SPOOL_RADIUS_MIN_MM
    while r <= SPOOL_RADIUS_MAX_MM + 1e-9:
        radii.append(round(r, 6))
        r += SPOOL_RADIUS_STEP_MM
    return radii
