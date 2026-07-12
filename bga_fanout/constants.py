"""
Constants for BGA fanout routing.
"""

# Position comparison tolerances (mm)
POSITION_TOLERANCE = 0.001  # For comparing if two points are the same
EDGE_PAD_TOLERANCE = 0.01   # For determining if a pad is on the edge
FANOUT_DETECTION_TOLERANCE = 0.05  # For detecting existing fanouts

# Fallback via diameter for the clearance-placement aux tools
# (place_fanout_clearance.py / animate_fanout_clearance.py) when a board's own
# via size can't be read. The MAIN fanout defaults live in routing_defaults.BGA_*;
# this is intentionally a small fallback, not those defaults.
DEFAULT_VIA_SIZE = 0.3
VIA_PROXIMITY_TOLERANCE = 0.1  # For finding nearby vias

# Iteration limits
MAX_REBALANCE_ITERATIONS = 100
MAX_EDGE_PAIR_ITERATIONS = 50

# Jog parameters
JOG_LENGTH_DIVISOR = 4  # Jog length = pitch / JOG_LENGTH_DIVISOR
