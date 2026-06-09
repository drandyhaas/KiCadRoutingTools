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
    # Key by (base_name, suffix style) so nets only pair within the same naming
    # convention (e.g. CLK+ pairs with CLK-, never with an unrelated CLK_N)
    pairs: Dict[tuple, DiffPairPads] = {}

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

        base_name, is_p, style = result
        key = (base_name, style)

        if key not in pairs:
            pairs[key] = DiffPairPads(base_name=base_name)

        if is_p:
            pairs[key].p_pad = pad
        else:
            pairs[key].n_pad = pad

    # Filter to only complete pairs, keyed by base name (disambiguate with the
    # suffix style in the unlikely case two conventions share a base name)
    complete_pairs: Dict[str, DiffPairPads] = {}
    for (base_name, style), pair in pairs.items():
        if not pair.is_complete:
            continue
        name = base_name if base_name not in complete_pairs else f"{base_name}({style})"
        complete_pairs[name] = pair

    return complete_pairs
