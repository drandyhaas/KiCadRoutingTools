"""
Default routing parameter values.

This module defines the default values for routing parameters, used by both
the CLI (route.py) and the GUI (swig_gui.py) to ensure consistency.
"""

# Track/Via parameters
TRACK_WIDTH = 0.3  # mm
CLEARANCE = 0.25  # mm
VIA_SIZE = 0.5  # mm
VIA_DRILL = 0.3  # mm

# Grid parameters
GRID_STEP = 0.1  # mm

# Via-obstacle diagonal expansion margin: how far a via's keep-out reaches
# diagonally on the routing grid. MUST be identical between the per-net obstacle
# cache (_collect_via_obstacles) and the full obstacle map, or incremental
# rip/rebuild desyncs (see obstacle_map.py). Coincidentally equals CLEARANCE but
# is a distinct concept; centralized so the call sites can't drift.
DIAGONAL_MARGIN = 0.25  # mm

# Allowance subtracted from mm-exact PLACEMENT validation thresholds so that
# grid-quantized copper (endpoints rounded to the routing grid) is not
# rejected for sub-resolution noise. This is a placement tolerance, NOT the
# DRC grading margin (check_drc --clearance-margin): placement must demand
# (nearly) full clearance or it ships real grazes (#339).
PLACEMENT_QUANTIZATION_MARGIN = 0.005  # mm

# Clearance slack the #189/#339 via-in-pad UNBLOCK refit tolerates before it
# shrinks/rejects a rescue via. The tight PLACEMENT_QUANTIZATION_MARGIN stranded
# pads (ulx3s -11, butterstick -7) because a via grazing by a few tens of um was
# shrunk/rejected until the unblock failed. Tolerate up to this much sub-clearance
# at PLACEMENT and let the post-route via-nudge (nudge_grazing_vias) move the
# residual graze to full clearance instead -- a nudged via stays connected; a
# stranded pad does not. Matches check_drc's default grading margin.
UNBLOCK_REFIT_MARGIN = 0.05  # mm

# Cost parameters
VIA_COST = 50
VIA_PROXIMITY_COST = 10
TURN_COST = 1000
STUB_PROXIMITY_COST = 0.2
TRACK_PROXIMITY_COST = 0.0

# Distance parameters
STUB_PROXIMITY_RADIUS = 2.0  # mm
NECKDOWN_LENGTH = 2.5  # mm of narrow track from the pad on neck-down routes (issue #72)
NECKDOWN_TAPER_LENGTH = 0.5  # mm narrow->wide width taper (0 = abrupt)
TRACK_PROXIMITY_DISTANCE = 2.0  # mm
BGA_PROXIMITY_RADIUS = 7.0  # mm
BGA_PROXIMITY_COST = 0.2

# Vertical alignment attraction
VERTICAL_ATTRACTION_RADIUS = 1.0  # mm
VERTICAL_ATTRACTION_COST = 0.0

# Ripped route avoidance
RIPPED_ROUTE_AVOIDANCE_RADIUS = 1.0  # mm
RIPPED_ROUTE_AVOIDANCE_COST = 0.1

# Impedance routing
IMPEDANCE_DEFAULT = 50  # ohms

# Crossing penalty
CROSSING_PENALTY = 1000.0

# Probe iterations
MAX_PROBE_ITERATIONS = 5000

# Length matching
LENGTH_MATCH_TOLERANCE = 0.1  # mm
MEANDER_AMPLITUDE = 1.0  # mm

# Time matching (alternative to length matching)
TIME_MATCHING = False  # If True, match propagation time instead of length
TIME_MATCH_TOLERANCE = 1.0  # ps

# GND via placement for single-ended routing
ADD_GND_VIAS = False  # If True, add GND vias near signal vias
GND_VIA_NET = "GND"  # Net name for GND vias
GND_VIA_DISTANCE = 2.0  # mm - max distance from signal via to GND via

# Algorithm parameters
MAX_ITERATIONS = 200000
HEURISTIC_WEIGHT = 1.9
PROXIMITY_HEURISTIC_FACTOR = 0.02
MAX_RIPUP = 3
# Phase 3 tap rip-up abandon metric (#85 arbitration); documented in
# docs/rip-up-reroute.md "Abandon metrics". Must match phase3_routing.ABANDON_METRICS.
RIPUP_ABANDON_METRIC = 'stranded'
RIPUP_ABANDON_METRIC_CHOICES = ('stranded', 'total-pads', 'complete-nets',
                                'congestion', 'history', 'weighted',
                                'probe', 'weighted-probe')

# Layer direction preference (0=horizontal, 1=vertical, 255=none)
# Alternates H/V starting with horizontal on top layer
DIRECTION_PREFERENCE_COST = 50  # Cost penalty for non-preferred direction (0 = disabled)

# Bus routing - auto-detection and parallel routing of grouped nets
BUS_DETECTION_RADIUS = 5.0  # mm - max endpoint distance to form bus
BUS_MIN_NETS = 2  # Minimum nets to form a bus
BUS_ATTRACTION_RADIUS = 5.0  # mm - attraction radius from neighbor track
BUS_ATTRACTION_BONUS = 5000  # Cost bonus for staying near neighbor

# Guide corridor - route selected nets through a user-drawn polyline (issue #7)
GUIDE_CORRIDOR_ENABLED = False
GUIDE_CORRIDOR_LAYER = "User.1"  # User layer the guide polyline is drawn on
GUIDE_CORRIDOR_SPACING = 0.0  # mm; 0 = waypoints only at drawn segment endpoints (else subdivide long segments)

# Keepout zone - keep routed tracks out of a user-drawn polygon (issue #27)
KEEPOUT_ENABLED = False
KEEPOUT_LAYER = "User.2"  # User layer the keepout polygon is drawn on

# Clearance parameters
ROUTING_CLEARANCE_MARGIN = 1.0
HOLE_TO_HOLE_CLEARANCE = 0.20  # mm - JLC "Via Hole-to-Hole Spacing" (edge-to-edge),
                               # the floor that governs router-placed via drills.
                               # (JLCPCB's pad-hole-to-hole is a separate, larger
                               # 0.45 mm; not modelled here -- this value targets
                               # via spacing. list_nets._FAB_FLOORS 'hole_to_hole'.)
                               # Routing AND check_drc default to this so a bare run
                               # never places/passes vias closer than is manufacturable.
NPTH_TO_TRACK_CLEARANCE = 0.20  # mm - JLC "NPTH to Track" fab floor: minimum copper
                                # (track) to NPTH mounting-hole edge. The drill removes
                                # any copper closer, so a track can't be routed/graded
                                # nearer than this regardless of the (smaller) routing
                                # clearance. Used by the NPTH track keep-out + check_drc
                                # track-hole check (issue #233).
BOARD_EDGE_CLEARANCE = 0.0  # mm

# Default layers
DEFAULT_LAYERS = ['F.Cu', 'B.Cu']

# Ordering strategy
DEFAULT_ORDERING_STRATEGY = "mps"

# BGA Fanout defaults
BGA_TRACK_WIDTH = 0.3  # mm
BGA_CLEARANCE = 0.25  # mm
BGA_VIA_SIZE = 0.5  # mm
BGA_VIA_DRILL = 0.3  # mm
BGA_EXIT_MARGIN = 0.5  # mm
BGA_DIFF_PAIR_GAP = 0.1  # mm

# QFN Fanout defaults
QFN_TRACK_WIDTH = 0.1  # mm
QFN_CLEARANCE = 0.1  # mm
QFN_EXTENSION = 0.1  # mm - extension past pad edge before bend

# Differential Pair defaults
DIFF_PAIR_WIDTH = 0.3  # mm track width for differential pairs (GUI diff tab
# default). #381 D4: matches route_diff.py's --track-width CLI default (0.3); the
# old 0.1 made GUI/plan diff runs 3x narrower than the equivalent CLI command.
# Consumed ONLY by the GUI diff tab -- the CLI/engine diff width default comes
# from route_diff.py's argparse and batch_route_diff_pairs' TRACK_WIDTH, so this
# change is GUI-only and does not move any CLI behavior.
DIFF_PAIR_GAP = 0.101  # mm gap between P and N traces
DIFF_PAIR_MIN_TURNING_RADIUS = 0.2  # mm
DIFF_PAIR_MAX_SETBACK_ANGLE = 45.0  # degrees
DIFF_PAIR_MAX_TURN_ANGLE = 180.0  # degrees
DIFF_PAIR_CHAMFER_EXTRA = 1.5  # multiplier for meander chamfers
DIFF_PAIR_CENTERLINE_SETBACK = 0.0  # mm - 0 = auto (2x P-N spacing)

# Plane routing defaults (route_planes.py)
PLANE_ZONE_CLEARANCE = 0.2  # mm - zone fill clearance from other copper
PLANE_MIN_THICKNESS = 0.1  # mm - minimum zone copper thickness
PLANE_EDGE_CLEARANCE = 0.5  # mm - zone clearance from board edge
PLANE_MAX_SEARCH_RADIUS = 10.0  # mm - max radius to search for via position
PLANE_MAX_VIA_REUSE_RADIUS = 1.0  # mm - max radius to reuse existing via
PLANE_PAD_STRAP_RADIUS = 1.5  # mm - max distance to strap a plane pad to an
                              # adjacent already-connected same-net pad instead
                              # of drilling another via (issue #349)
PLANE_MAX_RIP_NETS = 3  # max blocker nets to rip up
PLANE_TRACK_VIA_CLEARANCE = 0.8  # mm - clearance from track center to other nets' via centers
SAME_NET_PAD_CLEARANCE = -1.0  # mm - edge-to-edge clearance between via and same-net pads
                               # when placing plane stitching vias. -1 disables (allow via-in-pad).
                               # Any value >= 0 forces vias to be placed outside same-net pads
                               # with that much edge-to-edge clearance.

# Fine-pitch tap escalation (plane_pad_tap.py, issues #99/#104)
# When a tap can't be placed at the nominal clearance/grid on dense fine-pitch
# QFN/LQFP/BGA pads, the router escalates to a finer grid and steps the clearance
# DOWN toward the manufacturing floor (list_nets.fab_floors for the board's layer
# count), narrowing the tap track to the fab track floor. There is deliberately
# NO hard-coded "fine clearance"/"fine track" magic number -- the floor is the
# fab limit, and the ladder stops at the first clearance that routes (issue #226).
FINE_PITCH_NEIGHBOR_DIST = 0.65  # mm - same-component neighbor spacing => fine-pitch
FINE_PITCH_MIN_PAD_DIM = 0.35    # mm - pad min dimension below this => fine-pitch
FINE_TAP_GRID_STEP = 0.05        # mm - fine routing grid for tight tap retries
FINE_TAP_CLEARANCE_STEPS = 4     # clearance steps from nominal down to the fab floor
FINE_TAP_SEARCH_RADIUS = 3.0     # mm - cap on NEW-via search during the fine retry
                                 # (a far new via at fine width butterflies neighbours;
                                 # far EXISTING vias are reached via the distant-trace path)

# Repair disconnected planes defaults (route_disconnected_planes.py)
REPAIR_MAX_TRACK_WIDTH = 2.0  # mm - maximum track width for connections
REPAIR_MIN_TRACK_WIDTH = 0.2  # mm - minimum track width for connections
REPAIR_ANALYSIS_GRID_STEP = 0.5  # mm - grid step for connectivity analysis


# GUI-specific ranges (min, max, increment, digits)
# These define the SpinCtrl ranges for the GUI
PARAM_RANGES = {
    # 4 digits: fab-floor widths/drills carry a 4th decimal (e.g. a 6-layer
    # min track 0.0762mm). At digits=3 the SpinCtrlDouble rounded 0.0762 -> 0.076,
    # so the GUI routed BELOW the floor and every such track tripped a
    # track-width DRC the CLI (full precision) does not (#362).
    'track_width': {'min': 0.05, 'max': 25.0, 'inc': 0.05, 'digits': 4},
    'clearance': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 4},
    'via_size': {'min': 0.2, 'max': 2.0, 'inc': 0.05, 'digits': 4},
    'via_drill': {'min': 0.1, 'max': 1.5, 'inc': 0.05, 'digits': 4},
    'grid_step': {'min': 0.01, 'max': 1.0, 'inc': 0.01, 'digits': 4},
    'via_cost': {'min': 1, 'max': 1000},
    'max_iterations': {'min': 1000, 'max': 100000000},
    'heuristic_weight': {'min': 1.0, 'max': 10.0, 'inc': 0.1, 'digits': 1},
    'proximity_heuristic_factor': {'min': 0.0, 'max': 0.2, 'inc': 0.01, 'digits': 2},
    'turn_cost': {'min': 0, 'max': 10000},
    'direction_preference_cost': {'min': 0, 'max': 10000},
    'max_ripup': {'min': 0, 'max': 50},
    'stub_proximity_radius': {'min': 0.0, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'stub_proximity_cost': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'neckdown_length': {'min': 0.0, 'max': 50.0, 'inc': 0.5, 'digits': 1},
    'neckdown_taper_length': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'via_proximity_cost': {'min': 0.0, 'max': 100.0, 'inc': 1.0, 'digits': 1},
    'track_proximity_distance': {'min': 0.0, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'track_proximity_cost': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'routing_clearance_margin': {'min': 0.5, 'max': 2.0, 'inc': 0.1, 'digits': 1},
    'hole_to_hole_clearance': {'min': 0.0, 'max': 1.0, 'inc': 0.05, 'digits': 3},
    'board_edge_clearance': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 3},
    'bga_proximity_radius': {'min': 0.0, 'max': 20.0, 'inc': 0.5, 'digits': 1},
    'bga_proximity_cost': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'vertical_attraction_radius': {'min': 0.0, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'vertical_attraction_cost': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'ripped_route_avoidance_radius': {'min': 0.0, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'ripped_route_avoidance_cost': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'impedance': {'min': 10, 'max': 200, 'inc': 1, 'digits': 0},
    'crossing_penalty': {'min': 0.0, 'max': 10000.0, 'inc': 100.0, 'digits': 0},
    'max_probe_iterations': {'min': 100, 'max': 100000},
    'length_match_tolerance': {'min': 0.01, 'max': 5.0, 'inc': 0.01, 'digits': 2},
    'meander_amplitude': {'min': 0.1, 'max': 10.0, 'inc': 0.1, 'digits': 1},
    'time_match_tolerance': {'min': 0.1, 'max': 50.0, 'inc': 0.1, 'digits': 1},
    'gnd_via_distance': {'min': 0.5, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    # Fanout parameters
    'exit_margin': {'min': 0.1, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'diff_pair_gap': {'min': 0.05, 'max': 5.0, 'inc': 0.01, 'digits': 2},
    'qfn_extension': {'min': 0.05, 'max': 10.0, 'inc': 0.05, 'digits': 2},
    # Differential pair routing parameters
    'diff_pair_width': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'diff_pair_min_turning_radius': {'min': 0.05, 'max': 2.0, 'inc': 0.05, 'digits': 2},
    'diff_pair_max_setback_angle': {'min': 10.0, 'max': 90.0, 'inc': 5.0, 'digits': 0},
    'diff_pair_max_turn_angle': {'min': 45.0, 'max': 360.0, 'inc': 15.0, 'digits': 0},
    'diff_pair_chamfer_extra': {'min': 1.0, 'max': 3.0, 'inc': 0.1, 'digits': 1},
    'diff_pair_centerline_setback': {'min': 0.0, 'max': 10.0, 'inc': 0.1, 'digits': 1},  # 0 = auto
    # Plane routing parameters
    'plane_zone_clearance': {'min': 0.05, 'max': 2.0, 'inc': 0.05, 'digits': 2},
    'plane_min_thickness': {'min': 0.05, 'max': 1.0, 'inc': 0.05, 'digits': 2},
    'plane_edge_clearance': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'plane_max_search_radius': {'min': 1.0, 'max': 50.0, 'inc': 1.0, 'digits': 1},
    'plane_max_via_reuse_radius': {'min': 0.0, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'plane_max_rip_nets': {'min': 1, 'max': 10},
    'same_net_pad_clearance': {'min': 0.0, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    # Repair planes parameters
    'repair_max_track_width': {'min': 0.1, 'max': 10.0, 'inc': 0.1, 'digits': 3},
    'repair_min_track_width': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'repair_analysis_grid_step': {'min': 0.1, 'max': 2.0, 'inc': 0.1, 'digits': 1},
    # Bus routing parameters
    'bus_detection_radius': {'min': 0.5, 'max': 100.0, 'inc': 0.5, 'digits': 1},
    'bus_attraction_radius': {'min': 0.5, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'bus_attraction_bonus': {'min': 0, 'max': 10000},
    'bus_min_nets': {'min': 2, 'max': 20},
}
