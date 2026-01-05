"""
Constants for BGA fanout routing.
"""

# Position comparison tolerances (mm)
POSITION_TOLERANCE = 0.001  # For comparing if two points are the same
EDGE_PAD_TOLERANCE = 0.01   # For determining if a pad is on the edge
FANOUT_DETECTION_TOLERANCE = 0.05  # For detecting existing fanouts

# Via parameters
DEFAULT_VIA_SIZE = 0.3
DEFAULT_VIA_DRILL = 0.2
VIA_PROXIMITY_TOLERANCE = 0.1  # For finding nearby vias

# Track parameters
DEFAULT_TRACK_WIDTH = 0.1
DEFAULT_CLEARANCE = 0.1
DEFAULT_DIFF_PAIR_GAP = 0.101
DEFAULT_EXIT_MARGIN = 0.5

# Iteration limits
MAX_REBALANCE_ITERATIONS = 100
MAX_EDGE_PAIR_ITERATIONS = 50

# Jog parameters
JOG_LENGTH_DIVISOR = 4  # Jog length = pitch / JOG_LENGTH_DIVISOR
