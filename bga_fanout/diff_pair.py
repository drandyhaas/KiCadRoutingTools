"""
Differential pair detection and routing utilities.

Functions for finding and processing differential pairs in BGA footprints.
"""

from typing import Dict, List
import fnmatch

from kicad_parser import Footprint
from net_queries import extract_diff_pair_base
from bga_fanout.types import DiffPairPads


def find_differential_pairs(footprint: Footprint,
                           diff_pair_patterns: List[str]) -> Dict[str, DiffPairPads]:
    """
    Find all differential pairs in a footprint matching the given patterns.

    Args:
        footprint: The footprint to search
        diff_pair_patterns: Glob patterns for nets to treat as diff pairs

    Returns:
        Dict mapping base_name to DiffPairPads
    """
    pairs: Dict[str, DiffPairPads] = {}

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        # Check if this net matches any diff pair pattern
        matched = any(fnmatch.fnmatch(pad.net_name, pattern)
                     for pattern in diff_pair_patterns)
        if not matched:
            continue

        # Try to extract diff pair info
        result = extract_diff_pair_base(pad.net_name)
        if result is None:
            continue

        base_name, is_p = result

        if base_name not in pairs:
            pairs[base_name] = DiffPairPads(base_name=base_name)

        if is_p:
            pairs[base_name].p_pad = pad
        else:
            pairs[base_name].n_pad = pad

    # Filter to only complete pairs
    complete_pairs = {k: v for k, v in pairs.items() if v.is_complete}

    return complete_pairs
