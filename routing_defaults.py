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
TRACK_PROXIMITY_COST = 0.2

# Distance parameters
STUB_PROXIMITY_RADIUS = 2.0  # mm
TRACK_PROXIMITY_DISTANCE = 2.0  # mm
BGA_PROXIMITY_RADIUS = 7.0  # mm
BGA_PROXIMITY_COST = 0.2

# Vertical alignment attraction
VERTICAL_ATTRACTION_RADIUS = 1.0  # mm
VERTICAL_ATTRACTION_COST = 0.1

# Impedance routing
IMPEDANCE_DEFAULT = 50  # ohms

# Crossing penalty
CROSSING_PENALTY = 1000.0

# Probe iterations
MAX_PROBE_ITERATIONS = 5000

# Length matching
LENGTH_MATCH_TOLERANCE = 0.1  # mm
MEANDER_AMPLITUDE = 1.0  # mm

# Algorithm parameters
MAX_ITERATIONS = 200000
HEURISTIC_WEIGHT = 1.9
MAX_RIPUP = 3

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


# GUI-specific ranges (min, max, increment, digits)
# These define the SpinCtrl ranges for the GUI
PARAM_RANGES = {
    'track_width': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'clearance': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'via_size': {'min': 0.2, 'max': 2.0, 'inc': 0.05, 'digits': 2},
    'via_drill': {'min': 0.1, 'max': 1.5, 'inc': 0.05, 'digits': 2},
    'grid_step': {'min': 0.01, 'max': 1.0, 'inc': 0.01, 'digits': 2},
    'via_cost': {'min': 1, 'max': 1000},
    'max_iterations': {'min': 1000, 'max': 1000000},
    'heuristic_weight': {'min': 1.0, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'turn_cost': {'min': 0, 'max': 10000},
    'max_ripup': {'min': 0, 'max': 10},
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
    'impedance': {'min': 10, 'max': 200, 'inc': 1, 'digits': 0},
    'crossing_penalty': {'min': 0.0, 'max': 10000.0, 'inc': 100.0, 'digits': 0},
    'max_probe_iterations': {'min': 100, 'max': 100000},
    'length_match_tolerance': {'min': 0.01, 'max': 5.0, 'inc': 0.01, 'digits': 2},
    'meander_amplitude': {'min': 0.1, 'max': 10.0, 'inc': 0.1, 'digits': 1},
    # Fanout parameters
    'exit_margin': {'min': 0.1, 'max': 5.0, 'inc': 0.1, 'digits': 1},
    'diff_pair_gap': {'min': 0.05, 'max': 1.0, 'inc': 0.01, 'digits': 2},
}
