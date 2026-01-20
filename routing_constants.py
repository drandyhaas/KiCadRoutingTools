"""
Routing constants used throughout the codebase.

Centralizes magic numbers and default values for consistency.
"""

# Default layer stack for 4-layer boards
DEFAULT_4_LAYER_STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']

# Power net detection patterns (used for auto-exclusion from component routing)
POWER_NET_EXCLUSION_PATTERNS = [
    '*GND*', '*VCC*', '*VDD*', '+*V', '-*V', 'unconnected-*', ''
]

# Via placement multipliers
VIA_EXCLUSION_MARGIN_MULTIPLIER = 1.5  # Multiplied by via_size for pad exclusion zones
VIA_CLOSE_RADIUS_MULTIPLIER = 2.5     # Default multiplier for close via search radius
VIA_SKIP_RADIUS_MULTIPLIER = 2.0      # Multiplier for skipping existing vias

# Spatial indexing
DEFAULT_SPATIAL_INDEX_CELL_SIZE = 2.0  # mm - for DRC checking
ORPHAN_STUB_CELL_SIZE = 0.5           # mm - for orphan stub detection

# Tolerances (mm)
ORPHAN_STUB_VIA_TOLERANCE = 0.15      # Distance to consider endpoint near a via
ORPHAN_STUB_PAD_TOLERANCE = 0.5       # Distance to consider endpoint near a pad

# Plane region routing
MIN_OPEN_SPACE_CLEARANCE_CELLS = 2    # Minimum clearance in grid cells for open space routing
PROXIMITY_COST_MULTIPLIER = 1000      # Multiplier for proximity cost calculation

# Resistance analysis (plane_resistance.py)
RAY_CAST_LENGTH = 1000.0              # mm - ray length for polygon width sampling

# IPC-2152 formula constants for max current calculation
IPC_2152_EXPONENT_A = 0.44            # Area exponent
IPC_2152_EXPONENT_B = 0.725           # Temperature rise exponent
IPC_2152_K_INTERNAL = 0.024           # k-value for internal layers
IPC_2152_K_EXTERNAL = 0.048           # k-value for external layers

# Polygon/zone geometry tolerances (mm)
POLYGON_BUFFER_DISTANCE = 0.01        # Buffer distance for polygon shrinking
POLYGON_EDGE_TOLERANCE = 0.001        # Tolerance for edge-sharing detection

# BGA/component detection (mm)
BGA_EDGE_DETECTION_TOLERANCE = 0.01   # Tolerance for detecting pad alignment
BGA_DEFAULT_EDGE_TOLERANCE = 1.6      # Default edge tolerance for BGA pitch detection
