"""
Differential pair detection and routing utilities.

Functions for finding and processing differential pairs in BGA footprints.
"""
from __future__ import annotations

from typing import Dict, List

from kicad_parser import Footprint
from net_queries import extract_diff_pair_base, matches_diff_pair_patterns
from bga_fanout.types import DiffPairPads


def find_differential_pairs(footprint: Footprint,
                           diff_pair_patterns: List[str]) -> Dict[str, DiffPairPads]:
    """
    Find all differential pairs in a footprint matching the given patterns.

    A pair is selected when *either* half matches the patterns (see
    matches_diff_pair_patterns), so a glob that only catches one half (e.g.
    '*_P') or an explicit base name still pulls in the complete pair and both
    siblings escape together. (issue #120)

    Args:
        footprint: The footprint to search
        diff_pair_patterns: Glob patterns for nets to treat as diff pairs

    Returns:
        Dict mapping base_name to DiffPairPads
    """
    # Key by (base_name, suffix style) so nets only pair within the same naming
    # convention (e.g. CLK+ pairs with CLK-, never with an unrelated CLK_N)
    pairs: Dict[tuple, DiffPairPads] = {}
    matched_keys = set()

    # Collect all diff-pair halves, regardless of pattern, so a pattern that
    # only catches one half can still pull in its sibling below.
    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
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

        if matches_diff_pair_patterns(pad.net_name, base_name, diff_pair_patterns):
            matched_keys.add(key)

    # Filter to only complete pairs whose key was matched, keyed by base name
    # (disambiguate with the suffix style in the unlikely case two conventions
    # share a base name)
    complete_pairs: Dict[str, DiffPairPads] = {}
    for key, pair in pairs.items():
        if key not in matched_keys:
            continue
        if not pair.is_complete:
            continue
        base_name, style = key
        name = base_name if base_name not in complete_pairs else f"{base_name}({style})"
        complete_pairs[name] = pair

    return complete_pairs
