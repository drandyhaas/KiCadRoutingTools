"""
Shared base utilities for PCB routing.

This module contains the core position and geometry utilities used by all routing modules.
The bulk of routing functionality has been split into:
- connectivity.py: Endpoint finding, stub analysis, MST algorithms
- net_queries.py: Pad/net queries, MPS ordering, route length calculations
- route_modification.py: Add/remove routes, segment cleanup
"""

import math
from typing import Tuple

from kicad_parser import Segment, POSITION_DECIMALS


def pos_key(x: float, y: float) -> Tuple[float, float]:
    """
    Normalize coordinates for position-based lookups.

    Use this consistently when building position sets or checking position membership
    to avoid floating-point comparison issues.
    """
    return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS))


def segment_length(seg: Segment) -> float:
    """Calculate the length of a single segment."""
    return math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)
