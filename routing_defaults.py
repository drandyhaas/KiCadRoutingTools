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


# GUI-specific ranges (min, max, increment, digits)
# These define the SpinCtrl ranges for the GUI
PARAM_RANGES = {
    'track_width': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'clearance': {'min': 0.05, 'max': 5.0, 'inc': 0.05, 'digits': 2},
    'via_size': {'min': 0.2, 'max': 2.0, 'inc': 0.05, 'digits': 2},
    'via_drill': {'min': 0.1, 'max': 1.5, 'inc': 0.05, 'digits': 2},
    'grid_step': {'min': 0.01, 'max': 1.0, 'inc': 0.01, 'digits': 3},
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
}
