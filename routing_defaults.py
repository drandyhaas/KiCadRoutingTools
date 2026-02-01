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

# Cost parameters
VIA_COST = 50
VIA_PROXIMITY_COST = 10
TURN_COST = 1000
STUB_PROXIMITY_COST = 0.2
TRACK_PROXIMITY_COST = 0.0

# Distance parameters
STUB_PROXIMITY_RADIUS = 2.0  # mm
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

# Layer direction preference (0=horizontal, 1=vertical, 255=none)
# Alternates H/V starting with horizontal on top layer
DIRECTION_PREFERENCE_COST = 50  # Cost penalty for non-preferred direction (0 = disabled)

# Bus routing - auto-detection and parallel routing of grouped nets
BUS_DETECTION_RADIUS = 5.0  # mm - max endpoint distance to form bus
BUS_MIN_NETS = 2  # Minimum nets to form a bus
BUS_ATTRACTION_RADIUS = 5.0  # mm - attraction radius from neighbor track
BUS_ATTRACTION_BONUS = 5000  # Cost bonus for staying near neighbor

# Clearance parameters
ROUTING_CLEARANCE_MARGIN = 1.0
HOLE_TO_HOLE_CLEARANCE = 0.2  # mm
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
DIFF_PAIR_WIDTH = 0.1  # mm track width for differential pairs
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
PLANE_MAX_RIP_NETS = 3  # max blocker nets to rip up

# Repair disconnected planes defaults (route_disconnected_planes.py)
REPAIR_MAX_TRACK_WIDTH = 2.0  # mm - maximum track width for connections
REPAIR_MIN_TRACK_WIDTH = 0.2  # mm - minimum track width for connections
REPAIR_ANALYSIS_GRID_STEP = 0.5  # mm - grid step for connectivity analysis


# GUI-specific ranges (min, max, increment, digits)
# These define the SpinCtrl ranges for the GUI
PARAM_RANGES = {
    'track_width': {'min': 0.05, 'max': 25.0, 'inc': 0.05, 'digits': 2},
    'clearance': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'via_size': {'min': 0.2, 'max': 2.0, 'inc': 0.05, 'digits': 2},
    'via_drill': {'min': 0.1, 'max': 1.5, 'inc': 0.05, 'digits': 2},
    'grid_step': {'min': 0.01, 'max': 1.0, 'inc': 0.01, 'digits': 2},
    'via_cost': {'min': 1, 'max': 1000},
    'max_iterations': {'min': 1000, 'max': 100000000},
    'heuristic_weight': {'min': 1.0, 'max': 10.0, 'inc': 0.1, 'digits': 1},
    'proximity_heuristic_factor': {'min': 0.0, 'max': 0.2, 'inc': 0.01, 'digits': 2},
    'turn_cost': {'min': 0, 'max': 10000},
    'direction_preference_cost': {'min': 0, 'max': 10000},
    'max_ripup': {'min': 0, 'max': 50},
    'stub_proximity_radius': {'min': 0.0, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'stub_proximity_cost': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'via_proximity_cost': {'min': 0.0, 'max': 100.0, 'inc': 1.0, 'digits': 1},
    'track_proximity_distance': {'min': 0.0, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'track_proximity_cost': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'routing_clearance_margin': {'min': 0.5, 'max': 2.0, 'inc': 0.1, 'digits': 1},
    'hole_to_hole_clearance': {'min': 0.0, 'max': 1.0, 'inc': 0.05, 'digits': 2},
    'board_edge_clearance': {'min': 0.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
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
    # Repair planes parameters
    'repair_max_track_width': {'min': 0.1, 'max': 10.0, 'inc': 0.1, 'digits': 1},
    'repair_min_track_width': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'repair_analysis_grid_step': {'min': 0.1, 'max': 2.0, 'inc': 0.1, 'digits': 1},
    # Bus routing parameters
    'bus_detection_radius': {'min': 0.5, 'max': 100.0, 'inc': 0.5, 'digits': 1},
    'bus_attraction_radius': {'min': 0.5, 'max': 10.0, 'inc': 0.5, 'digits': 1},
    'bus_attraction_bonus': {'min': 0, 'max': 10000},
    'bus_min_nets': {'min': 2, 'max': 20},
}
