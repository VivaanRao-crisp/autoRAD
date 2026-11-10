"""params.py - every dimension and design variable for autoRAD. Single source of truth (CLAUDE.md).

ALL VALUES ARE PRELIMINARY. Units: mm, degrees, N. Z up in CAD; in these 2D kinematics x = outward,
y = up (pivot line y = 0, centre line x = 0), z = stacking direction along the pivot axes.
Tags in comments: VERIFIED (computed by the named script) / ASSUMPTION (a choice) / TODO (unknown).

SELECTED: architecture D = reeved reference cable (8 parts) + carriage + two links + hard stop
(docs/buildability.md, docs/kinematics_D_report.md). Architecture E (slotted arms) and C (threaded
drum) are LEGACY, CAD-only alternatives; their values are kept so kinematics.py still runs.
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
PROFILE = "linear_eased"            # "linear" or "linear_eased" (linear, then a quadratic blend to zero
                                    # angular speed at STROKE_MAP so the dwell is entered without a step)
EASE_FRACTION = 0.2                 # [-] last fraction of the mapping stroke used for the blend (0 = pure linear)
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
PIVOT_BOSS_MM = 10.0                # [mm] reviewer decision (freeze run): hub R10 = bearing OD 16 / 2 + 2 mm wall (was 20)
HOUSING_WIDTH_MAX_MM = 170.0        # [mm] relaxed target 150-170 (Q1)
ARM_RIM_MM = 4.0                    # [mm] arm material outside the slot edge (>= MIN_WALL; ASSUMPTION 4 for strength)
HOUSING_WALL_MM = 3.0               # [mm] housing side wall
ARM_CLEARANCE_MM = 2.0              # [mm] gap between a moving arm and the housing wall

# ---------------------------------------------------------------- hand load (ASSUMPTION)
F_HAND_N = 200.0                    # [N] design vertical force per hand. TODO: measure (Q5)
F_HAND_STUDY_N = (0.0, 100.0, 200.0)    # [N] values compared in the printed tables

# ---------------------------------------------------------------- LEGACY architecture E: reeving, carriage, pin height
# (read by kinematics.py only; architecture D uses the D_* block below)
N_REEVE = 8                         # [-] cable parts supporting the carriage (N_REEVE - 1 pulleys)
PULLEY_DIAMETER_MM = 30.0           # [mm] ASSUMPTION: >= ~20 x cable dia for 1.5 mm cable. TODO confirm
CABLE_DIAMETER_MM = 1.5             # [mm] steel cable (brief)
CARRIAGE_MARGIN_MM = 10.0           # [mm] extra carriage travel at each end before the hard stops
Y0_PIN_MM = 80.0                    # [mm] height of the carriage pin above the pivot line at closed
                                    #      (rev 3 trade-off, docs 5.1: each +10 mm = +10 mm width, -3.5 deg pressure angle)
N_REEVE_STUDY = (4, 6, 8)           # [-] layouts compared in the trade-off table
Y0_STUDY_MM = (50.0, 80.0, 120.0, 150.0, 180.0)   # [mm] pin heights compared in the trade-off table
PULLEY_GAP_MM = 10.0                # [mm] gap between side-by-side pulleys / pulley rows
CARRIAGE_PULLEYS_PER_ROW = 4        # [-] moving pulleys side by side on one axle (4 x 30 + 3 x 10 = 150 mm wide)
TOP_CLEARANCE_MM = 10.0             # [mm] space above the fixed pulleys inside the housing
ETA_PULLEY = 0.97                   # [-] ASSUMPTION: efficiency per ball-bearing pulley
ETA_SLIDE = 0.90                    # [-] ASSUMPTION: carriage guide + roller-in-slot, together

# ---------------------------------------------------------------- return spring (extension)
SPRING_PRELOAD_N = 12.0             # [N] bench rig: rubber band / bungee pull on the carriage at closed. ASSUMPTION, pick by test
                                    #     (history: 30 N extension spring for the 8-part full attachment)
SPRING_RATE_N_PER_MM = 0.05         # [N/mm] ASSUMPTION: soft band; +3.75 N over the 75 mm travel
CABLE_TENSION_MIN_N = 2.0           # [N] "never slack" margin used in the PASS/FAIL check

# ---------------------------------------------------------------- LEGACY architecture E: slot / roller
ROLLER_OD_MM = 10.0                 # [mm] ball-bearing roller on the pin. TODO bearing size
MOVING_CLEARANCE_MM = 0.3           # [mm] clearance on moving fits (brief)
SLOT_WIDTH_MM = ROLLER_OD_MM + MOVING_CLEARANCE_MM
PRESSURE_ANGLE_MAX_DEG = 55.0       # [deg] ASSUMPTION: limit for a rolling follower; test on prototype
MIN_WALL_MM = 2.0                   # [mm] FDM minimum wall (brief)

# ---------------------------------------------------------------- performance goals (Q7)
R0_MAX_N = 10.0                     # [N] no-load reference-cable tension, max over the stroke
R_LOAD_FRACTION_MAX = 0.10          # [-] loaded tension <= 10 % of stack load. TODO confirm
STACK_LOAD_N = 2 * F_HAND_N         # [N] stack load implied by the hand-load assumption

# ---------------------------------------------------------------- architecture D (SELECTED, CLAUDE.md): links + hard stop
# Checked by kinematics_D.py; see docs/kinematics_D_report.md. Tags: VERIFIED / ASSUMPTION / TODO.
# Geometry (right handle, housing frame, x outward, y up, pivot at (P/2, 0)):
#   drive arm of length D_ARM_MM points INWARD from the pivot, elevated D_ARM_ANGLE_CLOSED_DEG above the
#   inward horizontal at closed; link of length D_LINK_MM from the arm joint UP to the carriage pin on the
#   centre line. Opening = CCW rotation of the right handle -> the inward joint moves DOWN -> the carriage
#   moves DOWN as the cable pays out (fixed pulleys below the carriage, return spring pulls it up).
D_ARM_MM = 125.9                    # [mm] VERIFIED bench_check.py: re-solved for 75.0 mm carriage travel (n = 4), 42 deg elevation
                                    #      (history: 70.0 start -> 70.3 at 25 deg -> 72.7 at 40 deg)
D_LINK_MM = 120.0                   # [mm] VERIFIED bench_check.py scan (n = 4): mapping error 1.7 %, transmission angle >= 64 deg
D_ARM_ANGLE_CLOSED_DEG = 42.0       # [deg] VERIFIED bench_check.py scan: 8.5 mm between the arm body and the opposite pivot bolt at open
D_N_REEVE = 4                       # [-] bench rig: 4 cable parts, 2 moving + 1 fixed bearing + fairlead bearing
D_CARRIAGE_PULLEYS_PER_ROW = 2      # [-] ASSUMPTION: 2 rows of 2 (4 in a row = 150 mm > 149 mm housing, buildability error)
D_ARM_WIDTH_MM = 16.0               # [mm] ASSUMPTION: drive-arm plate width, used for interference margins
D_ARM_THICK_MM = 8.0                # [mm] ASSUMPTION: plate thickness; arms stacked in z with 0.3 mm gap
D_LINK_WIDTH_MM = 12.0              # [mm] ASSUMPTION: link width, used for interference margins
D_CARRIAGE_HALF_H_MM = 10.0         # [mm] ASSUMPTION: carriage body half-height around the pin
D_PIVOT_BOLT_MM = 8.0               # [mm] M8 pivot bolt (allowed by reviewer)
D_ANCHOR_SPRING_PRELOAD_N = 50.0    # [N] v2 only (CLAUDE.md). ASSUMPTION: must exceed R_load. Rate: TODO
D_ANCHOR_TENSION_STUDY_N = (50.0, 70.0)   # [N] cable tensions at which the carriage stop load is reported
D_TRANSMISSION_ANGLE_MIN_DEG = 30.0 # [deg] flag limit requested by the reviewer
D_CARRIAGE_TRAVEL_MM = 75.0         # [mm] VERIFIED bench_check.py: 300 / 4

# -- bought bearing (CLAUDE.md: parameter, default 625ZZ; confirm with the user before printing pulleys)
PULLEY_BEARING_OD_MM = 16.0         # [mm] ASSUMPTION default 625ZZ
PULLEY_BEARING_ID_MM = 5.0          # [mm] ASSUMPTION default 625ZZ
PULLEY_BEARING_WIDTH_MM = 5.0       # [mm] ASSUMPTION default 625ZZ
PULLEY_WIDTH_MM = 7.0               # [mm] ASSUMPTION (user default, freeze run): bearing width 5 + 2 x 1 mm flanges

# -- pins, bosses, plates (check3d_D.py)
D_JOINT_PIN_MM = 5.0                # [mm] M5 arm/link joint pin
D_CARRIAGE_PIN_MM = 5.0             # [mm] bench rig: M5 carriage pin (also the front guide bolt through board slot 1)
D_HUB_RADIUS_MM = 12.0              # [mm] bench rig: plain bore on the M8 pivot bolt, no bearing (wall 7.85 mm)
D_JOINT_BOSS_RADIUS_MM = 8.0        # [mm] ASSUMPTION: boss around the joint pin on arm and link (= arm half width)
D_PIN_OVERHANG_MM = 3.0             # [mm] ASSUMPTION: pin head / nut beyond the plates it connects
D_LINK_THICK_MM = 6.0               # [mm] ASSUMPTION: link plate thickness
D_PLATE_GAP_MM = 0.3                # [mm] clearance between stacked moving plates (brief)
D_CLEARANCE_MIN_MM = 2.0            # [mm] ASSUMPTION: flag any 3D clearance below this
D_CHECK_TRAVELS_MM = (0.0, 18.75, 37.5, 56.25, 75.0)   # [mm] carriage positions checked (0..75 mm)

# -- z-stack (z lo, hi) [mm], VERIFIED check3d_D.py (see docs/kinematics_D_report.md section 9)
D_STACK_REPORTED = {"arm_R": (0.0, 8.0), "arm_L": (8.3, 16.3), "link_R": (16.6, 22.6), "link_L": (22.9, 28.9)}  # rev 1 report, FAILS
D_STACK = {"arm_R": (0.0, 8.0), "link_R": (8.3, 14.3), "link_L": (14.6, 20.6), "arm_L": (20.9, 28.9)}           # VERIFIED (bench: board face = z 0)
D_GRIP_PLANE_Z_MM = 14.45           # [mm] VERIFIED check3d_D.py: common plane of both long arms = centre of the stack
D_GRIP_SPACER_R_MM = 10.45          # [mm] VERIFIED check3d_D.py: hub extension from arm R plate centre (z 4.0) to the grip plane
D_GRIP_SPACER_L_MM = 10.45          # [mm] VERIFIED check3d_D.py: hub extension from arm L plate centre (z 24.9) to the grip plane

# -- grips, hands, housing bottom (check3d_D.py)
D_GRIP_LENGTH_MM = 130.0            # [mm] CLAUDE.md: bought tube dia 32 x 130
HAND_ENVELOPE_RADIUS_MM = 35.0      # [mm] ASSUMPTION: knuckle envelope radius around the tube axis. TODO measure
D_HOUSING_BOTTOM_Y_MM = 15.0        # [mm] ASSUMPTION/PROPOSAL: housing bottom plate above the pivot line; hubs as ears below

# -- y / z layout of the cable system (check3d_D.py y_layout)
D_MOVING_AXLE_ABOVE_PIN_MM = 40.0   # [mm] ASSUMPTION: carriage pulley axle above the carriage pin
D_PULLEY_GAP_AT_STOP_MM = 5.0       # [mm] reviewer: min gap between moving and fixed pulley bodies at the stop
D_FAIRLEAD_PULLEY_DIAMETER_MM = 20.0   # [mm] ASSUMPTION: F0 turns the anchor cable at the fixed end; bearing with OD <= 12: TODO
D_CARRIAGE_PIN_BACK_OVERHANG_MM = 4.0  # [mm] bench rig: M5 nut in front of link L
D_HOUSING_PLATE_MM = 10.0           # [mm] CLAUDE.md: top/bottom plates >= 10 mm, printed flat
D_GUIDE_ROD_MM = 5.0                # [mm] CLAUDE.md: two dia-5 guide rods
D_ROD_BUSH_OD_MM = 12.0             # [mm] ASSUMPTION: printed bush around each rod on the carriage
D_SPRING_OD_MM = 10.0               # [mm] ASSUMPTION: extension spring outside diameter. TODO from the bought spring
HOUSING_WIDTH_MM_D = 149.0          # [mm] reviewer: available outer width; inner 143 >= 130 needed (VERIFIED check3d_D.py width_budget)
D_FIXED_ROW_Y_MM = 84.0             # [mm] VERIFIED check3d_D.py y_layout: fixed-pulley / F0 / dead-end axle height (5 mm gap at the stop)
D_HOUSING_TOP_Y_MM = 191.5          # [mm] VERIFIED check3d_D.py: top plate outer face above the pivot line
D_HOUSING_DEPTH_MM = 72.0           # [mm] VERIFIED check3d_D.py: outer depth with 8 mm pulleys and 3 mm plates (70 with 7 mm pulleys)
D_SPRING_POSITION = "above carriage, x 0, z 43"   # VERIFIED check3d_D.py: 'beside' needs 154 mm inner, does not fit

# -- reeving lanes (check3d_D.py): moving pulleys axle along x, fixed pulleys axle along z, all parts vertical
D_MOVING_PULLEY_DIAMETER_MM = 30.0  # [mm] = PULLEY_DIAMETER_MM; sets the z spacing of the fixed pulleys
D_FIXED_PULLEY_DIAMETER_MM = 30.0   # [mm] = PULLEY_DIAMETER_MM; sets the x spacing of the moving pulleys
D_CABLE_Z_CENTRE_MM = 43.0          # [mm] ASSUMPTION: z of the moving-pulley axle (43: F2 clears the links and the carriage pin by >= 2 mm)
D_CARRIAGE_WIDTH_MM = 3 * D_FIXED_PULLEY_DIAMETER_MM + PULLEY_WIDTH_MM   # [mm] VERIFIED check3d_D.py = 98 for dia 30.
                                    # Reviewer asked for 70 (from the 2 x 2 in-plane idea); that layout cannot be
                                    # reeved (closed loop) - see report section 11. 68 mm is reachable with dia-20 fixed pulleys.

# ---------------------------------------------------------------- BENCH RIG (final prototype, 2026-11-10) - everything on one board
# Board in the XY plane, front face z = 0 (handles and links in front, z > 0), board body z -BOARD_T..0,
# carriage plate and the whole cord system BEHIND the board (z < -BOARD_T). Pivot line y = 0, centre line x = 0.
BOARD_W_MM = 200.0                  # [mm] ASSUMPTION: plywood or acrylic, fits an A4 1:1 template
BOARD_H_MM = 290.0                  # [mm] ASSUMPTION
BOARD_T_MM = 12.0                   # [mm] ASSUMPTION: 12 mm plywood (10 mm acrylic also fine)
BOARD_BOTTOM_Y_MM = -20.0           # [mm] bottom edge relative to the pivot line (hub R12 + 8 margin)
BENCH_PULLEY_D_MM = 16.0            # [mm] 625ZZ bearing used directly as the pulley (OD 16, width 5)
BENCH_PULLEY_W_MM = 5.0             # [mm]
BENCH_FLANGE_D_MM = 20.0            # [mm] printed flange washers either side of each bearing, 1 mm thick
BENCH_CORD_D_MM = 1.5               # [mm] ASSUMPTION: braided cord / fishing line (low load)
BENCH_AXLE_ABOVE_PIN_MM = 45.0      # [mm] ASSUMPTION: moving-bearing axle above the carriage pin (post hangs from the plate)
BENCH_MOVING_X_MM = (-22.0, -6.0)   # [mm] x of the two moving bearings (one lane pattern, all parts vertical)
BENCH_LANE_GAP_AT_STOP_MM = 8.0     # [mm] ASSUMPTION: min gap moving-bearing bottom to fixed-bearing top at the stop
BENCH_CARRIAGE_T_MM = 8.0           # [mm] carriage plate thickness
BENCH_GUIDE2_ABOVE_PIN_MM = 60.0    # [mm] second guide bolt above the carriage pin (through board slot 2)
BENCH_SLOT_W_MM = 5.3               # [mm] board slots for the two M5 guide bolts (5 + 0.3)
BENCH_DEAD_END_Y_MM = 95.0          # [mm] dead-end screw height (below the carriage's lowest point)
BENCH_GRIP_D_MM = 20.0              # [mm] ASSUMPTION: wooden dowel, screwed to the grip pad (brief says 32: bench only)
BENCH_GRIP_LEN_MM = 130.0           # [mm]
BENCH_RETURN = "rubber band / bungee from the carriage plate to a screw at the top edge"   # ASSUMPTION, no spring spec
BENCH_ETA = 0.90 * 0.97 ** 3        # [-] ASSUMPTION: 3 bearings; same model as A3

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
CARRIAGE_TRAVEL_MAP_MM = STROKE_MAP_MM / N_REEVE            # at full opening
CARRIAGE_TRAVEL_DWELL_MM = STROKE_DWELL_END_MM / N_REEVE    # at end of dwell
DY_DBETA_MM_PER_RAD = K_MM_PER_RAD / N_REEVE                # mm of carriage per rad of handle (linear part)

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


def eta_mech(n_reeve=N_REEVE):
    """Overall efficiency [-] = slide efficiency x pulley efficiency ^ (number of pulleys)."""
    return ETA_SLIDE * ETA_PULLEY ** (n_reeve - 1)


def carriage_pulley_rows(n_reeve=N_REEVE):
    """Rows (axles) of pulleys on the carriage: n/2 moving pulleys, CARRIAGE_PULLEYS_PER_ROW per axle."""
    moving = n_reeve // 2
    return (moving + CARRIAGE_PULLEYS_PER_ROW - 1) // CARRIAGE_PULLEYS_PER_ROW


def carriage_width_mm(n_reeve=N_REEVE):
    """Carriage width [mm]: pulleys side by side on one axle, plus gaps."""
    per_row = min(n_reeve // 2, CARRIAGE_PULLEYS_PER_ROW)
    return per_row * PULLEY_DIAMETER_MM + (per_row - 1) * PULLEY_GAP_MM


L_LINE_MM = line_length_mm()
PIVOT_SPACING_MM = pivot_spacing_mm()
ETA_MECH = eta_mech()
CARRIAGE_WIDTH_MM = carriage_width_mm()


def spool_radius_candidates():
    """Candidate spool radii [mm] for the architecture-A reference table."""
    radii = []
    r = SPOOL_RADIUS_MIN_MM
    while r <= SPOOL_RADIUS_MAX_MM + 1e-9:
        radii.append(round(r, 6))
        r += SPOOL_RADIUS_STEP_MM
    return radii
