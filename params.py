"""params.py - every dimension and design variable for autoRAD (architecture E).

ALL VALUES ARE PRELIMINARY. Units: mm, degrees, N unless the comment says otherwise.
Items marked TODO are unknown and need a decision from the team.
Architecture E = reeved reference cable + spring-returned carriage + slotted bell-crank handles.
See docs/main_design.md for the reasoning behind each value.
"""
import math

# ---------------------------------------------------------------- handle motion (requirement)
ANGLE_INCLUDED_CLOSED_DEG = 50.0    # [deg] included angle between the two grip tubes, closed
ANGLE_INCLUDED_OPEN_DEG = 110.0     # [deg] included angle, fully open

# ---------------------------------------------------------------- stroke mapping (requirement)
STROKE_MAP_MM = 300.0               # [mm] attachment travel that maps to full opening
STROKE_DWELL_END_MM = 400.0         # [mm] handles stay open (dwell) up to here, then carriage hard stop
STROKE_RANGE_MIN_MM = 250.0         # [mm] design range of user strokes, lower end
STROKE_RANGE_MAX_MM = 400.0         # [mm] design range, upper end
STROKE_TABLE_STEP_MM = 25.0         # [mm] step used in printed tables
PROFILE = "linear"                  # angle-vs-travel profile, v1 = linear. TODO: others later
# TODO: cable payout per mm of attachment travel. 1.0 assumes the reference cable hangs parallel
# to the main cable from an anchor above. Confirm with the real routing.
CABLE_PAYOUT_PER_MM_TRAVEL = 1.0    # [-]

# ---------------------------------------------------------------- grip
GRIP_DIAMETER_MM = 32.0             # [mm]
GRIP_LENGTH_MIN_MM = 115.0          # [mm]
GRIP_LENGTH_MAX_MM = 150.0          # [mm]

# ---------------------------------------------------------------- hand-spacing (requirement)
HAND_BREADTH_P95_MALE_MM = 96.0     # [mm] 95th percentile male hand breadth (given in brief)
HAND_CLEARANCE_MM = 10.0            # [mm] extra clearance (given in brief)

# ---------------------------------------------------------------- bell-crank handle geometry
# Each handle is a bell crank. The GRIP TUBE angle follows the included-angle requirement above.
# The line from the handle pivot to the grip centre is a separate design choice:
#   length L_LINE, angle THETA from vertical (outward positive). THETA sweeps THETA_CLOSED ->
#   THETA_CLOSED + ANGLE_PER_HANDLE. A vertical hand force F makes a torque F * L_LINE * sin(THETA)
#   about the pivot: THETA > 0 closes the handle, THETA < 0 opens it (docs, Part C).
S_CLOSED_MM = 106.0                 # [mm] grip-centre separation at closed (= hand-spacing minimum)
DELTA_S_MM = 70.0                   # [mm] change in grip-centre separation closed -> open (baseline, Q2)
DELTA_S_STUDY_MM = (60.0, 70.0, 80.0)   # [mm] values compared in the printed tables
THETA_CLOSED_DEG = 3.0              # [deg] pivot->grip line angle at closed. +3 = small CLOSING bias
                                    #       (fix (b) in docs Part C). -15 reproduces the old symmetric case.
PIVOT_BOSS_MM = 20.0                # [mm] housing material outboard of each pivot axis (proxy, TODO layout)
HOUSING_WIDTH_MAX_MM = 170.0        # [mm] relaxed target 150-170 (Q1)

# ---------------------------------------------------------------- hand load (ASSUMPTION)
F_HAND_N = 200.0                    # [N] design vertical force per hand. TODO: measure (Q5)
F_HAND_STUDY_N = (0.0, 100.0, 200.0)    # [N] values compared in the printed tables

# ---------------------------------------------------------------- reeving and carriage
N_REEVE = 4                         # [-] cable parts supporting the carriage (3 pulleys)
PULLEY_DIAMETER_MM = 30.0           # [mm] ASSUMPTION: >= ~20 x cable dia for 1.5 mm cable. TODO confirm
CABLE_DIAMETER_MM = 1.5             # [mm] steel cable (brief)
CARRIAGE_MARGIN_MM = 10.0           # [mm] extra carriage travel at each end before the hard stops
Y0_PIN_MM = 150.0                   # [mm] height of the carriage pin above the pivot line at closed
                                    #      (120 gave a 58 deg pressure angle at closed; 150 brings it under 55)
ETA_MECH = 0.8                      # [-] ASSUMPTION: overall mechanical efficiency carriage+pulleys+roller

# ---------------------------------------------------------------- return spring (extension)
SPRING_PRELOAD_N = 20.0             # [N] spring force on the carriage at the closed stop
SPRING_RATE_N_PER_MM = 0.10         # [N/mm] TODO: pick from a catalogue, no part number assumed
CABLE_TENSION_MIN_N = 2.0           # [N] "never slack" margin used in the PASS/FAIL check

# ---------------------------------------------------------------- slot / pin
ROLLER_OD_MM = 10.0                 # [mm] ball-bearing roller on the pin. TODO bearing size
MOVING_CLEARANCE_MM = 0.3           # [mm] clearance on moving fits (brief)
SLOT_WIDTH_MM = ROLLER_OD_MM + MOVING_CLEARANCE_MM
PRESSURE_ANGLE_MAX_DEG = 55.0       # [deg] ASSUMPTION: limit for a rolling follower; test on prototype
MIN_WALL_MM = 2.0                   # [mm] FDM minimum wall (brief)

# ---------------------------------------------------------------- performance goals (Q7)
R0_MAX_N = 10.0                     # [N] no-load reference-cable tension, max over the stroke
R_LOAD_FRACTION_MAX = 0.10          # [-] loaded tension <= 10 % of stack load. TODO confirm
STACK_LOAD_N = 2 * F_HAND_N         # [N] stack load implied by the hand-load assumption

# ---------------------------------------------------------------- architecture A reference (NOT selected)
# Kept so kinematics.py can still print the spool/gear comparison from the design study.
SPOOL_RADIUS_MIN_MM = 12.0          # [mm]
SPOOL_RADIUS_MAX_MM = 30.0          # [mm]
SPOOL_RADIUS_STEP_MM = 6.0          # [mm]
GEAR_MODULE_MM = 1.0                # [mm] placeholder
GEAR_MIN_TEETH = 12                 # [-] placeholder
GEAR_MAX_STAGE_RATIO = 6.0          # [-] placeholder

# ================================================================ derived values
ANGLE_PER_HANDLE_DEG = (ANGLE_INCLUDED_OPEN_DEG - ANGLE_INCLUDED_CLOSED_DEG) / 2.0   # 30 deg
HAND_SPACING_MIN_MM = HAND_BREADTH_P95_MALE_MM + HAND_CLEARANCE_MM                    # 106 mm

# cable per radian of handle rotation (virtual-work constant k, docs 6.2):  573 mm/rad
K_MM_PER_RAD = STROKE_MAP_MM * CABLE_PAYOUT_PER_MM_TRAVEL / math.radians(ANGLE_PER_HANDLE_DEG)

# carriage
CARRIAGE_TRAVEL_MAP_MM = STROKE_MAP_MM / N_REEVE            # 75 mm at full opening
CARRIAGE_TRAVEL_DWELL_MM = STROKE_DWELL_END_MM / N_REEVE    # 100 mm at end of dwell
DY_DBETA_MM_PER_RAD = K_MM_PER_RAD / N_REEVE                # 143 mm of carriage per rad of handle

THETA_OPEN_DEG = THETA_CLOSED_DEG + ANGLE_PER_HANDLE_DEG


def line_length_mm(delta_s_mm=DELTA_S_MM):
    """Pivot-to-grip-centre length L_LINE [mm] that gives the wanted separation change.

    S = P + 2 L sin(theta)  ->  delta_S = 2 L (sin(theta_open) - sin(theta_closed)).
    """
    return delta_s_mm / (2.0 * (math.sin(math.radians(THETA_OPEN_DEG))
                                - math.sin(math.radians(THETA_CLOSED_DEG))))


def pivot_spacing_mm(delta_s_mm=DELTA_S_MM):
    """Distance between the two handle pivot axes P [mm]:  P = S_closed - 2 L sin(theta_closed)."""
    return S_CLOSED_MM - 2.0 * line_length_mm(delta_s_mm) * math.sin(math.radians(THETA_CLOSED_DEG))


def housing_width_mm(delta_s_mm=DELTA_S_MM):
    """Housing width proxy [mm] = pivot spacing + a boss outboard of each pivot."""
    return pivot_spacing_mm(delta_s_mm) + 2.0 * PIVOT_BOSS_MM


L_LINE_MM = line_length_mm()
PIVOT_SPACING_MM = pivot_spacing_mm()
HOUSING_WIDTH_MM = housing_width_mm()


def spool_radius_candidates():
    """Candidate spool radii [mm] for the architecture-A reference table."""
    radii = []
    r = SPOOL_RADIUS_MIN_MM
    while r <= SPOOL_RADIUS_MAX_MM + 1e-9:
        radii.append(round(r, 6))
        r += SPOOL_RADIUS_STEP_MM
    return radii
