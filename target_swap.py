"""Target swap optimization using Hungarian algorithm.

This module handles the optimization of target assignments for swappable diff pairs.
It uses the Hungarian algorithm (via scipy.optimize.linear_sum_assignment) to find
the optimal assignment of sources to targets that minimizes total routing cost.

Cost factors:
- Distance between source and target centroids
- Layer penalty (via cost) if source and target are on different layers
- Crossing penalty if assignments would cause route crossings
"""

import math
from typing import Dict, List, Optional, Tuple, Set, Callable, TYPE_CHECKING

try:
    from scipy.optimize import linear_sum_assignment
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

from kicad_parser import PCBData, Pad
from routing_config import DiffPair, GridRouteConfig
from routing_utils import find_connected_segment_positions, pos_key
from chip_boundary import (
    ChipBoundary, build_chip_list, identify_chip_for_point,
    compute_far_side, compute_boundary_position, crossings_from_boundary_order,
    generate_boundary_debug_labels
)


def segments_cross(p1: Tuple[float, float], p2: Tuple[float, float],
                   p3: Tuple[float, float], p4: Tuple[float, float]) -> bool:
    """
    Check if line segment (p1->p2) properly crosses line segment (p3->p4).

    Uses CCW (counter-clockwise) orientation test.
    Returns True only for proper intersection (not endpoint touching).
    """
    def ccw(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> bool:
        """Check if three points are in counter-clockwise order."""
        return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

    # Two segments cross if they straddle each other
    return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))


def get_source_centroid(endpoint: Tuple) -> Tuple[float, float]:
    """
    Get centroid (midpoint of P and N) for a source endpoint.

    Endpoint format: (p_gx, p_gy, n_gx, n_gy, layer_idx, p_x, p_y, n_x, n_y)
    For source, we use the original float coordinates (indices 5-8).
    """
    p_x, p_y = endpoint[5], endpoint[6]
    n_x, n_y = endpoint[7], endpoint[8]
    return ((p_x + n_x) / 2, (p_y + n_y) / 2)


def get_target_centroid(endpoint: Tuple) -> Tuple[float, float]:
    """
    Get centroid (midpoint of P and N) for a target endpoint.

    Endpoint format: (p_gx, p_gy, n_gx, n_gy, layer_idx, p_x, p_y, n_x, n_y)
    """
    p_x, p_y = endpoint[5], endpoint[6]
    n_x, n_y = endpoint[7], endpoint[8]
    return ((p_x + n_x) / 2, (p_y + n_y) / 2)


def build_cost_matrix(
    pair_data: List[Tuple[str, DiffPair, List, List]],
    config: GridRouteConfig,
    pcb_data: PCBData,
    use_boundary_ordering: bool = False
) -> Tuple[List[List[float]], List[str]]:
    """
    Build N x N cost matrix for optimal target assignment.

    Args:
        pair_data: List of (pair_name, DiffPair, sources, targets) tuples
                   sources/targets format: (p_gx, p_gy, n_gx, n_gy, layer_idx, p_x, p_y, n_x, n_y)
        config: Routing configuration with via_cost
        pcb_data: PCB data for chip boundary detection
        use_boundary_ordering: If True, use chip boundary ordering for crossing detection.
                              Otherwise use Euclidean segment crossing test (default).

    Returns:
        (cost_matrix, pair_names) where:
        - cost_matrix[i][j] = cost if source i connects to target j
        - pair_names = list of pair names in matrix order

    Cost components:
        1. Distance: Euclidean distance between source and target centroids
        2. Layer penalty: via_cost if source layer != target layer
        3. Crossing penalty: Heavy cost for assignments that cross other assignments
           (using chip boundary ordering for accurate detection)
    """
    n = len(pair_data)
    pair_names = [pd[0] for pd in pair_data]

    # Get crossing penalty from config (default 100.0)
    crossing_penalty = getattr(config, 'target_swap_crossing_penalty', 1000.0)

    # Extract source/target centroids and layers
    source_centroids = []
    source_layers = []
    target_centroids = []
    target_layers = []

    for name, pair, sources, targets in pair_data:
        # Use first source/target (typically there's only one of each)
        src = sources[0]
        tgt = targets[0]
        source_centroids.append(get_source_centroid(src))
        source_layers.append(src[4])  # layer_idx
        target_centroids.append(get_target_centroid(tgt))
        target_layers.append(tgt[4])  # layer_idx

    # Build chip list and boundary positions if using boundary ordering
    chips = []
    source_chips = []
    target_chips = []
    boundary_positions: Dict[Tuple[int, int], Optional[Tuple[float, float]]] = {}

    if use_boundary_ordering:
        chips = build_chip_list(pcb_data)

        # Identify which chip each source/target belongs to
        source_chips = [identify_chip_for_point(c, chips) for c in source_centroids]
        target_chips = [identify_chip_for_point(c, chips) for c in target_centroids]

        # Precompute boundary positions for each (source_i, target_j) assignment
        for i in range(n):
            src_chip = source_chips[i]
            for j in range(n):
                tgt_chip = target_chips[j]

                if src_chip and tgt_chip and src_chip != tgt_chip:
                    src_far, tgt_far = compute_far_side(src_chip, tgt_chip)
                    # Use OPPOSITE traversal directions so that when both chips are
                    # "unrolled" into vertical lines, they face each other.
                    # With opposite directions, the crossing check works correctly:
                    # same geometric order → opposite boundary order → crossing detected
                    src_pos = compute_boundary_position(src_chip, source_centroids[i], src_far, clockwise=True)
                    tgt_pos = compute_boundary_position(tgt_chip, target_centroids[j], tgt_far, clockwise=False)
                    boundary_positions[(i, j)] = (src_pos, tgt_pos)

    # Initialize cost matrix with distance and layer penalties
    cost_matrix = [[0.0] * n for _ in range(n)]

    for i in range(n):
        src_c = source_centroids[i]
        src_layer = source_layers[i]

        for j in range(n):
            tgt_c = target_centroids[j]
            tgt_layer = target_layers[j]

            # Distance cost (Euclidean)
            dist = math.sqrt((src_c[0] - tgt_c[0])**2 + (src_c[1] - tgt_c[1])**2)

            # Layer penalty (via cost if layers differ)
            layer_cost = config.via_cost if src_layer != tgt_layer else 0

            cost_matrix[i][j] = dist + layer_cost

    # Add crossing penalties
    # For each (i,j) assignment, check if it crosses any other potential (k,l) assignment
    for i in range(n):
        for j in range(n):
            crossing_count = 0

            if use_boundary_ordering:
                # Use chip boundary ordering for crossing detection
                pos_ij = boundary_positions.get((i, j))
                if pos_ij is not None:
                    for k in range(n):
                        if k == i:
                            continue
                        for l in range(n):
                            if l == j:
                                continue

                            pos_kl = boundary_positions.get((k, l))
                            if pos_kl is None:
                                continue

                            # Check if same chip pair
                            if (source_chips[i] == source_chips[k] and
                                target_chips[j] == target_chips[l]):
                                if crossings_from_boundary_order(
                                    pos_ij[0], pos_ij[1], pos_kl[0], pos_kl[1]
                                ):
                                    crossing_count += 1
                else:
                    # Fall back to segment crossing for this assignment
                    src_i = source_centroids[i]
                    tgt_j = target_centroids[j]
                    for k in range(n):
                        if k == i:
                            continue
                        src_k = source_centroids[k]
                        for l in range(n):
                            if l == j:
                                continue
                            tgt_l = target_centroids[l]
                            if segments_cross(src_i, tgt_j, src_k, tgt_l):
                                crossing_count += 1
            else:
                # Use Euclidean segment crossing (default)
                src_i = source_centroids[i]
                tgt_j = target_centroids[j]
                for k in range(n):
                    if k == i:
                        continue
                    src_k = source_centroids[k]
                    for l in range(n):
                        if l == j:
                            continue
                        tgt_l = target_centroids[l]
                        if segments_cross(src_i, tgt_j, src_k, tgt_l):
                            crossing_count += 1

            # Add normalized crossing penalty
            if crossing_count > 0:
                cost_matrix[i][j] += crossing_penalty * crossing_count / max(1, (n - 1))

    return cost_matrix, pair_names


def compute_optimal_assignment(
    pair_data: List[Tuple[str, DiffPair, List, List]],
    config: GridRouteConfig,
    pcb_data: PCBData,
    use_boundary_ordering: bool = False
) -> Optional[Dict[str, str]]:
    """
    Compute optimal target swaps using Hungarian algorithm.

    Args:
        pair_data: List of (pair_name, DiffPair, sources, targets) for swappable pairs
        config: Routing configuration
        pcb_data: PCB data for chip boundary detection
        use_boundary_ordering: If True, use chip boundary ordering for crossing detection

    Returns:
        Dictionary mapping pair_name -> target_pair_name for swaps,
        or None if no swaps improve the assignment.
        Only includes pairs where assignment differs from original (diagonal).
    """
    if not HAS_SCIPY:
        print("  Warning: scipy not available, cannot compute optimal assignment")
        return None

    if len(pair_data) < 2:
        return None

    cost_matrix, pair_names = build_cost_matrix(pair_data, config, pcb_data, use_boundary_ordering)
    n = len(pair_names)

    # Compute original (diagonal) cost - each source connects to its own target
    original_cost = sum(cost_matrix[i][i] for i in range(n))

    # Compute optimal assignment using Hungarian algorithm
    import numpy as np
    cost_array = np.array(cost_matrix)
    row_ind, col_ind = linear_sum_assignment(cost_array)
    optimal_cost = sum(cost_matrix[row_ind[i]][col_ind[i]] for i in range(n))

    print(f"  Original assignment cost: {original_cost:.2f}")
    print(f"  Optimal assignment cost:  {optimal_cost:.2f}")

    # Only apply if optimal is strictly better
    if optimal_cost >= original_cost:
        print("  No beneficial swaps found")
        return None

    # Build swap dictionary for non-diagonal assignments
    swaps = {}
    for i in range(n):
        src_pair = pair_names[row_ind[i]]
        tgt_pair = pair_names[col_ind[i]]
        if src_pair != tgt_pair:
            swaps[src_pair] = tgt_pair

    improvement = original_cost - optimal_cost
    num_swaps = len(swaps) // 2  # Each swap involves 2 pairs
    print(f"  Improvement: {improvement:.2f} ({num_swaps} swap pair(s))")

    return swaps if swaps else None


def find_pad_in_positions(pads: List[Pad], positions: Set[Tuple[float, float]],
                          threshold: float = 0.5) -> Optional[Pad]:
    """Find a pad that's near any position in the stub chain."""
    for pad in pads:
        for pos in positions:
            if abs(pad.global_x - pos[0]) < threshold and abs(pad.global_y - pos[1]) < threshold:
                return pad
    return None


def apply_single_swap(
    pcb_data: PCBData,
    p1_name: str,
    p1_pair: DiffPair,
    p1_targets: List,
    p2_name: str,
    p2_pair: DiffPair,
    p2_targets: List,
    target_swaps: Dict[str, str],
    target_swap_info: List[Dict]
) -> bool:
    """
    Apply a single pairwise target swap to pcb_data.

    Modifies segments, vias, and pads to swap net assignments between
    p1's targets and p2's targets.

    Args:
        pcb_data: PCB data to modify in place
        p1_name: Name of first diff pair
        p1_pair: DiffPair object for first pair
        p1_targets: Target endpoints for first pair
        p2_name: Name of second diff pair
        p2_pair: DiffPair object for second pair
        p2_targets: Target endpoints for second pair
        target_swaps: Dict to update with swap mapping
        target_swap_info: List to append swap details for output file

    Returns:
        True if swap was applied successfully
    """
    # Record the swap
    target_swaps[p1_name] = p2_name
    target_swaps[p2_name] = p1_name

    print(f"\nApplying target swap: {p1_name} <-> {p2_name}")

    # Target tuple: (p_gx, p_gy, n_gx, n_gy, layer_idx, p_x, p_y, n_x, n_y)
    p1_tgt = p1_targets[0]
    p2_tgt = p2_targets[0]
    p1_p_pos = (p1_tgt[5], p1_tgt[6])  # P target position for pair 1
    p1_n_pos = (p1_tgt[7], p1_tgt[8])  # N target position for pair 1
    p2_p_pos = (p2_tgt[5], p2_tgt[6])  # P target position for pair 2
    p2_n_pos = (p2_tgt[7], p2_tgt[8])  # N target position for pair 2

    # Find all segment positions connected to each target stub
    p1_p_positions = find_connected_segment_positions(pcb_data, p1_p_pos[0], p1_p_pos[1], p1_pair.p_net_id)
    p1_n_positions = find_connected_segment_positions(pcb_data, p1_n_pos[0], p1_n_pos[1], p1_pair.n_net_id)
    p2_p_positions = find_connected_segment_positions(pcb_data, p2_p_pos[0], p2_p_pos[1], p2_pair.p_net_id)
    p2_n_positions = find_connected_segment_positions(pcb_data, p2_n_pos[0], p2_n_pos[1], p2_pair.n_net_id)

    # Swap net IDs in pcb_data segments at target positions
    p1_p_seg_count = 0
    p1_n_seg_count = 0
    p2_p_seg_count = 0
    p2_n_seg_count = 0

    for seg in pcb_data.segments:
        seg_positions = {pos_key(seg.start_x, seg.start_y),
                        pos_key(seg.end_x, seg.end_y)}
        # p1 target: p1_pair.p_net_id -> p2_pair.p_net_id
        if seg.net_id == p1_pair.p_net_id and seg_positions & p1_p_positions:
            seg.net_id = p2_pair.p_net_id
            p1_p_seg_count += 1
        elif seg.net_id == p1_pair.n_net_id and seg_positions & p1_n_positions:
            seg.net_id = p2_pair.n_net_id
            p1_n_seg_count += 1
        # p2 target: p2_pair.p_net_id -> p1_pair.p_net_id
        elif seg.net_id == p2_pair.p_net_id and seg_positions & p2_p_positions:
            seg.net_id = p1_pair.p_net_id
            p2_p_seg_count += 1
        elif seg.net_id == p2_pair.n_net_id and seg_positions & p2_n_positions:
            seg.net_id = p1_pair.n_net_id
            p2_n_seg_count += 1

    print(f"  Swapped segments: {p1_name} target: {p1_p_seg_count}P+{p1_n_seg_count}N, {p2_name} target: {p2_p_seg_count}P+{p2_n_seg_count}N")

    # Swap net IDs in pcb_data vias at target positions
    p1_p_via_count = 0
    p1_n_via_count = 0
    p2_p_via_count = 0
    p2_n_via_count = 0

    for via in pcb_data.vias:
        via_pos = pos_key(via.x, via.y)
        if via.net_id == p1_pair.p_net_id and via_pos in p1_p_positions:
            via.net_id = p2_pair.p_net_id
            p1_p_via_count += 1
        elif via.net_id == p1_pair.n_net_id and via_pos in p1_n_positions:
            via.net_id = p2_pair.n_net_id
            p1_n_via_count += 1
        elif via.net_id == p2_pair.p_net_id and via_pos in p2_p_positions:
            via.net_id = p1_pair.p_net_id
            p2_p_via_count += 1
        elif via.net_id == p2_pair.n_net_id and via_pos in p2_n_positions:
            via.net_id = p1_pair.n_net_id
            p2_n_via_count += 1

    if p1_p_via_count + p1_n_via_count + p2_p_via_count + p2_n_via_count > 0:
        print(f"  Swapped vias: {p1_name} target: {p1_p_via_count}P+{p1_n_via_count}N, {p2_name} target: {p2_p_via_count}P+{p2_n_via_count}N")

    # Find pads connected to the target stubs
    p1_p_pads = pcb_data.pads_by_net.get(p1_pair.p_net_id, [])
    p1_n_pads = pcb_data.pads_by_net.get(p1_pair.n_net_id, [])
    p2_p_pads = pcb_data.pads_by_net.get(p2_pair.p_net_id, [])
    p2_n_pads = pcb_data.pads_by_net.get(p2_pair.n_net_id, [])

    p1_p_pad = find_pad_in_positions(p1_p_pads, p1_p_positions)
    p1_n_pad = find_pad_in_positions(p1_n_pads, p1_n_positions)
    p2_p_pad = find_pad_in_positions(p2_p_pads, p2_p_positions)
    p2_n_pad = find_pad_in_positions(p2_n_pads, p2_n_positions)

    # Swap pad net IDs
    if p1_p_pad and p2_p_pad:
        p1_p_pad.net_id, p2_p_pad.net_id = p2_p_pad.net_id, p1_p_pad.net_id
        p1_p_pad.net_name, p2_p_pad.net_name = p2_p_pad.net_name, p1_p_pad.net_name
        print(f"  Swapped P pads: {p1_p_pad.component_ref}:{p1_p_pad.pad_number} <-> {p2_p_pad.component_ref}:{p2_p_pad.pad_number}")
    if p1_n_pad and p2_n_pad:
        p1_n_pad.net_id, p2_n_pad.net_id = p2_n_pad.net_id, p1_n_pad.net_id
        p1_n_pad.net_name, p2_n_pad.net_name = p2_n_pad.net_name, p1_n_pad.net_name
        print(f"  Swapped N pads: {p1_n_pad.component_ref}:{p1_n_pad.pad_number} <-> {p2_n_pad.component_ref}:{p2_n_pad.pad_number}")

    # Update pads_by_net dictionary
    if p1_p_pad:
        if p1_pair.p_net_id in pcb_data.pads_by_net:
            pcb_data.pads_by_net[p1_pair.p_net_id] = [p for p in pcb_data.pads_by_net[p1_pair.p_net_id] if p != p1_p_pad]
        pcb_data.pads_by_net.setdefault(p2_pair.p_net_id, []).append(p1_p_pad)
    if p2_p_pad:
        if p2_pair.p_net_id in pcb_data.pads_by_net:
            pcb_data.pads_by_net[p2_pair.p_net_id] = [p for p in pcb_data.pads_by_net[p2_pair.p_net_id] if p != p2_p_pad]
        pcb_data.pads_by_net.setdefault(p1_pair.p_net_id, []).append(p2_p_pad)
    if p1_n_pad:
        if p1_pair.n_net_id in pcb_data.pads_by_net:
            pcb_data.pads_by_net[p1_pair.n_net_id] = [p for p in pcb_data.pads_by_net[p1_pair.n_net_id] if p != p1_n_pad]
        pcb_data.pads_by_net.setdefault(p2_pair.n_net_id, []).append(p1_n_pad)
    if p2_n_pad:
        if p2_pair.n_net_id in pcb_data.pads_by_net:
            pcb_data.pads_by_net[p2_pair.n_net_id] = [p for p in pcb_data.pads_by_net[p2_pair.n_net_id] if p != p2_n_pad]
        pcb_data.pads_by_net.setdefault(p1_pair.n_net_id, []).append(p2_n_pad)

    # Store swap info for output file writing
    # IMPORTANT: Exclude overlapping positions from p2 to prevent double-swapping in output file
    # When positions overlap, a segment would be swapped twice (758->780, then 780->758) and end up
    # back at its original net. By excluding overlaps from p2, each segment is swapped exactly once.
    p2_p_positions_no_overlap = p2_p_positions - p1_p_positions
    p2_n_positions_no_overlap = p2_n_positions - p1_n_positions

    target_swap_info.append({
        'p1_name': p1_name,
        'p2_name': p2_name,
        'p1_p_net_id': p1_pair.p_net_id,
        'p1_n_net_id': p1_pair.n_net_id,
        'p2_p_net_id': p2_pair.p_net_id,
        'p2_n_net_id': p2_pair.n_net_id,
        'p1_p_positions': p1_p_positions,
        'p1_n_positions': p1_n_positions,
        'p2_p_positions': p2_p_positions_no_overlap,  # Excludes overlapping positions
        'p2_n_positions': p2_n_positions_no_overlap,  # Excludes overlapping positions
        'p1_p_pad': p1_p_pad,
        'p1_n_pad': p1_n_pad,
        'p2_p_pad': p2_p_pad,
        'p2_n_pad': p2_n_pad,
    })

    return True


def apply_target_swaps(
    pcb_data: PCBData,
    swappable_pairs: List[Tuple[str, DiffPair]],
    config: GridRouteConfig,
    get_endpoints_func: Callable[[DiffPair], Tuple[List, List, Optional[str]]],
    use_boundary_ordering: bool = False
) -> Tuple[Dict[str, str], List[Dict]]:
    """
    Main entry point for target swap optimization.

    Computes optimal target assignment using Hungarian algorithm and applies
    beneficial swaps to pcb_data.

    Args:
        pcb_data: PCB data to modify in place
        swappable_pairs: List of (pair_name, DiffPair) that can have targets swapped
        config: Routing configuration
        get_endpoints_func: Function to get endpoints for a DiffPair
                           Returns (sources, targets, error_string)
        use_boundary_ordering: If True, use chip boundary ordering for crossing detection

    Returns:
        (target_swaps, target_swap_info) where:
        - target_swaps: Dict mapping pair_name -> swapped_target_pair_name
        - target_swap_info: List of dicts with swap details for output file writing
    """
    target_swaps: Dict[str, str] = {}
    target_swap_info: List[Dict] = []

    if len(swappable_pairs) < 2:
        return target_swaps, target_swap_info

    # Gather endpoint data for all swappable pairs
    pair_data: List[Tuple[str, DiffPair, List, List]] = []
    for pair_name, pair in swappable_pairs:
        sources, targets, error = get_endpoints_func(pair)
        if error or not sources or not targets:
            print(f"  Skipping {pair_name}: {error or 'no endpoints'}")
            continue
        pair_data.append((pair_name, pair, sources, targets))

    if len(pair_data) < 2:
        print("  Not enough valid pairs for target swap optimization")
        return target_swaps, target_swap_info

    print(f"\nComputing optimal target assignment for {len(pair_data)} pairs...")

    # Compute optimal swaps using Hungarian algorithm
    optimal_swaps = compute_optimal_assignment(pair_data, config, pcb_data, use_boundary_ordering)

    if not optimal_swaps:
        return target_swaps, target_swap_info

    # Build lookup for pair_data by name
    pair_lookup = {name: (pair, sources, targets)
                   for name, pair, sources, targets in pair_data}

    # Apply each swap pair (only process each pair once)
    processed = set()
    for src_name, tgt_name in optimal_swaps.items():
        if src_name in processed or tgt_name in processed:
            continue

        processed.add(src_name)
        processed.add(tgt_name)

        p1_pair, p1_sources, p1_targets = pair_lookup[src_name]
        p2_pair, p2_sources, p2_targets = pair_lookup[tgt_name]

        apply_single_swap(
            pcb_data,
            src_name, p1_pair, p1_targets,
            tgt_name, p2_pair, p2_targets,
            target_swaps, target_swap_info
        )

    return target_swaps, target_swap_info


def generate_debug_boundary_labels(
    pcb_data: PCBData,
    swappable_pairs: List[Tuple[str, DiffPair]],
    get_endpoints_func: Callable[[DiffPair], Tuple[List, List, Optional[str]]]
) -> List[dict]:
    """
    Generate debug labels showing boundary position ordering for visualization.

    Args:
        pcb_data: PCB data for chip detection
        swappable_pairs: List of (pair_name, DiffPair) to label
        get_endpoints_func: Function to get endpoints for a DiffPair

    Returns:
        List of label dicts with 'text', 'x', 'y', 'layer' keys
    """
    labels = []

    # Gather endpoints
    source_centroids = []
    target_centroids = []
    pair_names = []

    for pair_name, pair in swappable_pairs:
        sources, targets, error = get_endpoints_func(pair)
        if error or not sources or not targets:
            continue
        src = sources[0]
        tgt = targets[0]
        source_centroids.append(get_source_centroid(src))
        target_centroids.append(get_target_centroid(tgt))
        pair_names.append(pair_name)

    if len(source_centroids) < 2:
        return labels

    # Build chip list and identify chips
    chips = build_chip_list(pcb_data)

    # Find the two main chips (source and target)
    src_chip = identify_chip_for_point(source_centroids[0], chips)
    tgt_chip = identify_chip_for_point(target_centroids[0], chips)

    if not src_chip or not tgt_chip or src_chip == tgt_chip:
        return labels

    src_far, tgt_far = compute_far_side(src_chip, tgt_chip)

    # Generate labels for source positions (numbered by order)
    # Source uses clockwise, target uses counter-clockwise (opposite directions)
    # so that when "unrolled" the two lines face each other
    src_positions = []
    for i, centroid in enumerate(source_centroids):
        pos = compute_boundary_position(src_chip, centroid, src_far, clockwise=True)
        src_positions.append((pos, centroid, pair_names[i]))

    # Sort and number
    src_sorted = sorted(src_positions, key=lambda x: x[0])
    for order_num, (pos, centroid, name) in enumerate(src_sorted, start=1):
        from chip_boundary import _project_to_boundary
        projected, edge = _project_to_boundary(centroid, src_chip.bounds)
        # Rotate labels on top/bottom edges by 90 degrees
        angle = 90 if edge in ('top', 'bottom') else 0
        labels.append({
            'text': f"S{order_num}",
            'x': projected[0],
            'y': projected[1],
            'layer': "User.6",
            'angle': angle
        })

    # Generate labels for target positions (counter-clockwise, opposite of source)
    tgt_positions = []
    for i, centroid in enumerate(target_centroids):
        pos = compute_boundary_position(tgt_chip, centroid, tgt_far, clockwise=False)
        tgt_positions.append((pos, centroid, pair_names[i]))

    tgt_sorted = sorted(tgt_positions, key=lambda x: x[0])
    for order_num, (pos, centroid, name) in enumerate(tgt_sorted, start=1):
        from chip_boundary import _project_to_boundary
        projected, edge = _project_to_boundary(centroid, tgt_chip.bounds)
        # Rotate labels on top/bottom edges by 90 degrees
        angle = 90 if edge in ('top', 'bottom') else 0
        labels.append({
            'text': f"T{order_num}",
            'x': projected[0],
            'y': projected[1],
            'layer': "User.6",
            'angle': angle
        })

    return labels
