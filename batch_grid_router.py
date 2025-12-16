"""
Batch PCB Router using Rust-accelerated A* - Routes multiple nets sequentially.

Usage:
    python batch_grid_router.py input.kicad_pcb output.kicad_pcb net1 net2 net3 ...

Requires the Rust router module. Build it with:
    cd rust_router && cargo build --release
    cp target/release/grid_router.dll grid_router.pyd  # Windows
    cp target/release/libgrid_router.so grid_router.so  # Linux
"""

import sys
import os
import time
import fnmatch
import random
import math
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field

from kicad_parser import (
    parse_kicad_pcb, PCBData, Segment, Via, Pad,
    auto_detect_bga_exclusion_zones, find_components_by_type, detect_package_type
)
from kicad_writer import generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr


@dataclass
class DiffPair:
    """Represents a differential pair with P and N nets."""
    base_name: str  # Common name without _P/_N suffix
    p_net_id: Optional[int] = None
    n_net_id: Optional[int] = None
    p_net_name: Optional[str] = None
    n_net_name: Optional[str] = None

    @property
    def is_complete(self) -> bool:
        return self.p_net_id is not None and self.n_net_id is not None

# Add rust_router directory to path for importing the compiled module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

# Import Rust router
try:
    import grid_router
    from grid_router import GridObstacleMap, GridRouter
    version = getattr(grid_router, '__version__', 'unknown')
    print(f"Using Rust router v{version}")
except ImportError as e:
    print("ERROR: Rust router module not found!")
    print("Build it with:")
    print("  cd rust_router && cargo build --release")
    print("  cp target/release/grid_router.dll grid_router.pyd  # Windows")
    print("  cp target/release/libgrid_router.so grid_router.so  # Linux")
    sys.exit(1)


@dataclass
class GridRouteConfig:
    """Configuration for grid-based routing."""
    track_width: float = 0.1  # mm
    clearance: float = 0.1  # mm between tracks
    via_size: float = 0.3  # mm via outer diameter
    via_drill: float = 0.2  # mm via drill
    grid_step: float = 0.1  # mm grid resolution
    via_cost: int = 25  # grid steps equivalent penalty for via
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'B.Cu'])
    max_iterations: int = 100000
    heuristic_weight: float = 1.5
    # BGA exclusion zones (auto-detected from PCB) - vias blocked inside these areas
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = field(default_factory=list)
    stub_proximity_radius: float = 1.0  # mm - radius around stubs to penalize
    stub_proximity_cost: float = 3.0  # mm equivalent cost at stub center
    # Direction search order: "forward", "backwards", or "random"
    direction_order: str = "forward"
    # Differential pair routing parameters
    diff_pair_gap: float = 0.1  # mm - gap between P and N traces (center-to-center = track_width + gap)
    min_diff_pair_centerline_setback: float = 0.4  # mm - minimum distance in front of stubs to start centerline route
    max_diff_pair_centerline_setback: float = 0.4  # mm - maximum distance (searches in range min to max)
    debug_layers: bool = False  # Output raw A* path on User.9, simplified path on User.8


class GridCoord:
    """Utilities for converting between float (mm) and integer grid coordinates."""
    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step
        self.inv_step = 1.0 / grid_step

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert float mm coordinates to integer grid coordinates."""
        return (round(x * self.inv_step), round(y * self.inv_step))

    def to_float(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert integer grid coordinates to float mm coordinates."""
        return (gx * self.grid_step, gy * self.grid_step)

    def to_grid_dist(self, dist_mm: float) -> int:
        """Convert a distance in mm to grid units."""
        return round(dist_mm * self.inv_step)


def extract_diff_pair_base(net_name: str) -> Optional[Tuple[str, bool]]:
    """
    Extract differential pair base name and polarity from net name.

    Looks for common differential pair naming conventions:
    - name_P / name_N
    - nameP / nameN
    - name+ / name-

    Returns (base_name, is_positive) or None if not a diff pair.
    """
    if not net_name:
        return None

    # Try _P/_N suffix (most common for LVDS)
    if net_name.endswith('_P'):
        return (net_name[:-2], True)
    if net_name.endswith('_N'):
        return (net_name[:-2], False)

    # Try P/N suffix without underscore
    if net_name.endswith('P') and len(net_name) > 1:
        # Check it's not just ending in P as part of name
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], True)
    if net_name.endswith('N') and len(net_name) > 1:
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], False)

    # Try +/- suffix
    if net_name.endswith('+'):
        return (net_name[:-1], True)
    if net_name.endswith('-'):
        return (net_name[:-1], False)

    return None


def find_differential_pairs(pcb_data: PCBData, patterns: List[str]) -> Dict[str, DiffPair]:
    """
    Find all differential pairs in the PCB matching the given glob patterns.

    Args:
        pcb_data: PCB data with net information
        patterns: Glob patterns for nets to treat as diff pairs (e.g., '*lvds*')

    Returns:
        Dict mapping base_name to DiffPair with complete P/N pairs
    """
    pairs: Dict[str, DiffPair] = {}

    # Collect all net names from pcb_data
    for net_id, net in pcb_data.nets.items():
        net_name = net.name
        if not net_name or net_id == 0:
            continue

        # Check if this net matches any diff pair pattern
        matched = any(fnmatch.fnmatch(net_name, pattern) for pattern in patterns)
        if not matched:
            continue

        # Try to extract diff pair info
        result = extract_diff_pair_base(net_name)
        if result is None:
            continue

        base_name, is_p = result

        if base_name not in pairs:
            pairs[base_name] = DiffPair(base_name=base_name)

        if is_p:
            pairs[base_name].p_net_id = net_id
            pairs[base_name].p_net_name = net_name
        else:
            pairs[base_name].n_net_id = net_id
            pairs[base_name].n_net_name = net_name

    # Filter to only complete pairs
    complete_pairs = {k: v for k, v in pairs.items() if v.is_complete}

    return complete_pairs


def find_connected_groups(segments: List[Segment], tolerance: float = 0.01) -> List[List[Segment]]:
    """Find groups of connected segments using union-find."""
    if not segments:
        return []

    n = len(segments)
    parent = list(range(n))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    # Check all pairs for shared endpoints
    for i in range(n):
        for j in range(i + 1, n):
            si, sj = segments[i], segments[j]
            pts_i = [(si.start_x, si.start_y), (si.end_x, si.end_y)]
            pts_j = [(sj.start_x, sj.start_y), (sj.end_x, sj.end_y)]
            for pi in pts_i:
                for pj in pts_j:
                    if abs(pi[0] - pj[0]) < tolerance and abs(pi[1] - pj[1]) < tolerance:
                        union(i, j)
                        break

    # Group segments by their root
    groups: Dict[int, List[Segment]] = {}
    for i in range(n):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


def find_stub_free_ends(segments: List[Segment], pads: List[Pad], tolerance: float = 0.05) -> List[Tuple[float, float, str]]:
    """
    Find the free ends of a segment group (endpoints not connected to other segments or pads).

    Args:
        segments: Connected group of segments
        pads: Pads that might be connected to the segments
        tolerance: Distance tolerance for considering points as connected

    Returns:
        List of (x, y, layer) tuples for free endpoints
    """
    if not segments:
        return []

    # Count how many times each endpoint appears
    endpoint_counts: Dict[Tuple[float, float, str], int] = {}
    for seg in segments:
        key_start = (round(seg.start_x, 3), round(seg.start_y, 3), seg.layer)
        key_end = (round(seg.end_x, 3), round(seg.end_y, 3), seg.layer)
        endpoint_counts[key_start] = endpoint_counts.get(key_start, 0) + 1
        endpoint_counts[key_end] = endpoint_counts.get(key_end, 0) + 1

    # Get pad positions
    pad_positions = [(round(p.global_x, 3), round(p.global_y, 3)) for p in pads]

    # Free ends are endpoints that appear only once AND are not near a pad
    free_ends = []
    for (x, y, layer), count in endpoint_counts.items():
        if count == 1:
            # Check if near a pad
            near_pad = False
            for px, py in pad_positions:
                if abs(x - px) < tolerance and abs(y - py) < tolerance:
                    near_pad = True
                    break
            if not near_pad:
                free_ends.append((x, y, layer))

    return free_ends


def get_stub_direction(segments: List[Segment], stub_x: float, stub_y: float, stub_layer: str,
                       tolerance: float = 0.05) -> Tuple[float, float]:
    """
    Find the direction a stub is pointing (from pad toward free end).

    Args:
        segments: List of segments for the net
        stub_x, stub_y: Position of the stub free end
        stub_layer: Layer of the stub
        tolerance: Distance tolerance for matching endpoints

    Returns:
        (dx, dy): Normalized direction vector pointing in the stub direction
    """
    # Find the segment that has this stub endpoint
    for seg in segments:
        if seg.layer != stub_layer:
            continue

        # Check if start matches stub position
        if abs(seg.start_x - stub_x) < tolerance and abs(seg.start_y - stub_y) < tolerance:
            # Direction from end to start (toward the free end)
            dx = seg.start_x - seg.end_x
            dy = seg.start_y - seg.end_y
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                return (dx / length, dy / length)
            return (0, 0)

        # Check if end matches stub position
        if abs(seg.end_x - stub_x) < tolerance and abs(seg.end_y - stub_y) < tolerance:
            # Direction from start to end (toward the free end)
            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                return (dx / length, dy / length)
            return (0, 0)

    # Fallback - no matching segment found
    return (0, 0)


def get_net_endpoints(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                      use_stub_free_ends: bool = False) -> Tuple[List, List, str]:
    """
    Find source and target endpoints for a net, considering segments, pads, and existing vias.

    Args:
        pcb_data: PCB data
        net_id: Net ID to find endpoints for
        config: Grid routing configuration
        use_stub_free_ends: If True, use only stub free ends (for diff pairs).
                           If False, use all segment endpoints (for single-ended).

    Returns:
        (sources, targets, error_message)
        - sources: List of (gx, gy, layer_idx, orig_x, orig_y)
        - targets: List of (gx, gy, layer_idx, orig_x, orig_y)
        - error_message: None if successful, otherwise describes why routing can't proceed
    """
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    # Case 1: Multiple segment groups
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments)
        if len(groups) >= 2:
            groups.sort(key=len, reverse=True)
            source_segs = groups[0]
            target_segs = groups[1]

            if use_stub_free_ends:
                # For diff pairs: use only stub free ends (tips not connected to pads)
                source_free_ends = find_stub_free_ends(source_segs, net_pads)
                target_free_ends = find_stub_free_ends(target_segs, net_pads)

                sources = []
                for x, y, layer in source_free_ends:
                    layer_idx = layer_map.get(layer)
                    if layer_idx is not None:
                        gx, gy = coord.to_grid(x, y)
                        sources.append((gx, gy, layer_idx, x, y))

                targets = []
                for x, y, layer in target_free_ends:
                    layer_idx = layer_map.get(layer)
                    if layer_idx is not None:
                        gx, gy = coord.to_grid(x, y)
                        targets.append((gx, gy, layer_idx, x, y))
            else:
                # For single-ended: use all segment endpoints
                sources = []
                for seg in source_segs:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                targets = []
                for seg in target_segs:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        targets.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            targets.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

            if sources and targets:
                return sources, targets, None

    # Case 2: One segment group + unconnected pads
    if len(net_segments) >= 1 and len(net_pads) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            # Check if any pad is NOT connected to the segment group
            seg_group = groups[0]
            seg_points = set()
            for seg in seg_group:
                seg_points.add((round(seg.start_x, 3), round(seg.start_y, 3)))
                seg_points.add((round(seg.end_x, 3), round(seg.end_y, 3)))

            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, 3), round(pad.global_y, 3))
                # Check if pad is near any segment point
                connected = False
                for sp in seg_points:
                    if abs(pad_pos[0] - sp[0]) < 0.05 and abs(pad_pos[1] - sp[1]) < 0.05:
                        connected = True
                        break
                if not connected:
                    unconnected_pads.append(pad)

            if unconnected_pads:
                # Use all segment endpoints as source, unconnected pad(s) as target
                sources = []
                for seg in seg_group:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                targets = []
                for pad in unconnected_pads:
                    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                    for layer in pad.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

                # Add existing vias as endpoints - they can be routed to on any layer
                # Determine if via is near stub (add to sources) or near unconnected pads (add to targets)
                # Note: All vias are through-hole, connecting ALL routing layers
                for via in net_vias:
                    gx, gy = coord.to_grid(via.x, via.y)
                    # Check if via is near any unconnected pad
                    near_unconnected = False
                    for pad in unconnected_pads:
                        if abs(via.x - pad.global_x) < 0.1 and abs(via.y - pad.global_y) < 0.1:
                            near_unconnected = True
                            break
                    # Add via as endpoint on ALL routing layers (vias are through-hole)
                    for layer in config.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            if near_unconnected:
                                targets.append((gx, gy, layer_idx, via.x, via.y))
                            else:
                                sources.append((gx, gy, layer_idx, via.x, via.y))

                if sources and targets:
                    return sources, targets, None

    # Case 3: No segments, just pads - route between pads
    if len(net_segments) == 0 and len(net_pads) >= 2:
        # Use first pad as source, rest as targets
        sources = []
        pad = net_pads[0]
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        for layer in pad.layers:
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                sources.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

        targets = []
        for pad in net_pads[1:]:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            for layer in pad.layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

        if sources and targets:
            return sources, targets, None

    # Case 4: Single segment, check if it connects two pads already
    if len(net_segments) == 1 and len(net_pads) >= 2:
        # Segment already connects pads - nothing to route
        return [], [], "Net has 1 segment connecting pads - already routed"

    # Determine why we can't route
    if len(net_segments) == 0 and len(net_pads) < 2:
        return [], [], f"Net has no segments and only {len(net_pads)} pad(s) - need at least 2 endpoints"
    if len(net_segments) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            return [], [], "Net segments are already connected (single group) with no unconnected pads"

    return [], [], f"Cannot determine endpoints: {len(net_segments)} segments, {len(net_pads)} pads"


def get_all_unrouted_net_ids(pcb_data: PCBData) -> List[int]:
    """
    Find all net IDs in the PCB that have unrouted stubs (multiple disconnected segment groups).
    These are nets that have partial routing but aren't fully connected.
    """
    unrouted_ids = []

    # Group segments by net ID
    net_segments: Dict[int, List[Segment]] = {}
    for seg in pcb_data.segments:
        if seg.net_id not in net_segments:
            net_segments[seg.net_id] = []
        net_segments[seg.net_id].append(seg)

    # Check each net for multiple disconnected groups
    for net_id, segments in net_segments.items():
        if net_id == 0:  # Skip unassigned net
            continue
        if len(segments) < 2:
            continue
        groups = find_connected_groups(segments)
        if len(groups) >= 2:
            # Net has multiple disconnected stub groups = unrouted
            unrouted_ids.append(net_id)

    return unrouted_ids


def get_stub_endpoints(pcb_data: PCBData, net_ids: List[int]) -> List[Tuple[float, float]]:
    """Get centroid positions of unrouted net stubs for proximity avoidance."""
    stubs = []
    for net_id in net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        if len(net_segments) < 2:
            continue
        groups = find_connected_groups(net_segments)
        if len(groups) < 2:
            continue
        for group in groups:
            points = []
            for seg in group:
                points.append((seg.start_x, seg.start_y))
                points.append((seg.end_x, seg.end_y))
            if points:
                cx = sum(p[0] for p in points) / len(points)
                cy = sum(p[1] for p in points) / len(points)
                stubs.append((cx, cy))
    return stubs


def get_net_stub_centroids(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float]]:
    """
    Get centroids of each connected stub group for a net.
    Returns list of (x, y) centroids, typically 2 for a 2-point net.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        return []
    groups = find_connected_groups(net_segments)
    if len(groups) < 2:
        return []

    centroids = []
    for group in groups:
        points = []
        for seg in group:
            points.append((seg.start_x, seg.start_y))
            points.append((seg.end_x, seg.end_y))
        if points:
            cx = sum(p[0] for p in points) / len(points)
            cy = sum(p[1] for p in points) / len(points)
            centroids.append((cx, cy))
    return centroids


def get_net_routing_endpoints(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float]]:
    """
    Get the two routing endpoints for a net, for MPS conflict detection.

    This handles multiple cases:
    1. Two stub groups -> use stub centroids
    2. One stub group + unconnected pads -> use stub centroid + pad centroid
    3. No stubs, just pads -> use pad positions

    Returns list of (x, y) positions, typically 2 for source and target.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])

    # Case 1: Multiple stub groups - use their centroids
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments)
        if len(groups) >= 2:
            centroids = []
            for group in groups:
                points = []
                for seg in group:
                    points.append((seg.start_x, seg.start_y))
                    points.append((seg.end_x, seg.end_y))
                if points:
                    cx = sum(p[0] for p in points) / len(points)
                    cy = sum(p[1] for p in points) / len(points)
                    centroids.append((cx, cy))
            return centroids[:2]

    # Case 2: One stub group + pads - find unconnected pads
    if len(net_segments) >= 1 and len(net_pads) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            # Get stub centroid
            group = groups[0]
            seg_points = set()
            for seg in group:
                seg_points.add((round(seg.start_x, 3), round(seg.start_y, 3)))
                seg_points.add((round(seg.end_x, 3), round(seg.end_y, 3)))

            stub_pts = []
            for seg in group:
                stub_pts.append((seg.start_x, seg.start_y))
                stub_pts.append((seg.end_x, seg.end_y))
            stub_cx = sum(p[0] for p in stub_pts) / len(stub_pts)
            stub_cy = sum(p[1] for p in stub_pts) / len(stub_pts)

            # Find unconnected pads
            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, 3), round(pad.global_y, 3))
                connected = False
                for sp in seg_points:
                    if abs(pad_pos[0] - sp[0]) < 0.05 and abs(pad_pos[1] - sp[1]) < 0.05:
                        connected = True
                        break
                if not connected:
                    unconnected_pads.append(pad)

            if unconnected_pads:
                # Compute centroid of unconnected pads
                pad_cx = sum(p.global_x for p in unconnected_pads) / len(unconnected_pads)
                pad_cy = sum(p.global_y for p in unconnected_pads) / len(unconnected_pads)
                return [(stub_cx, stub_cy), (pad_cx, pad_cy)]

    # Case 3: No stubs, just pads - use first pad and centroid of rest
    if len(net_segments) == 0 and len(net_pads) >= 2:
        first_pad = net_pads[0]
        other_pads = net_pads[1:]
        other_cx = sum(p.global_x for p in other_pads) / len(other_pads)
        other_cy = sum(p.global_y for p in other_pads) / len(other_pads)
        return [(first_pad.global_x, first_pad.global_y), (other_cx, other_cy)]

    return []


def compute_mps_net_ordering(pcb_data: PCBData, net_ids: List[int],
                              center: Tuple[float, float] = None) -> List[int]:
    """
    Compute optimal net routing order using Maximum Planar Subset (MPS) algorithm.

    The MPS approach identifies crossing conflicts between nets and orders them
    so that non-conflicting nets are routed first. This reduces routing failures
    caused by earlier routes blocking later ones.

    Algorithm:
    1. For each net, find its two stub endpoint centroids
    2. Project all endpoints onto a circular boundary centered on the routing region
    3. Assign each endpoint an angular position on the boundary
    4. Detect crossing conflicts: nets A and B cross if their endpoints alternate
       on the boundary (A1, B1, A2, B2 or B1, A1, B2, A2 ordering)
    5. Build a conflict graph where edges connect crossing nets
    6. Use greedy algorithm: repeatedly select net with fewest active conflicts,
       add to result, and remove its neighbors from consideration for this round
    7. Continue until all nets are ordered (multiple rounds/layers)

    Args:
        pcb_data: PCB data with segments
        net_ids: List of net IDs to order
        center: Optional center point for angular projection (auto-computed if None)

    Returns:
        Ordered list of net IDs, with least-conflicting nets first
    """
    import math

    # Step 1: Get routing endpoints for each net (stubs, pads, or both)
    net_endpoints = {}  # net_id -> [(x1, y1), (x2, y2)]
    for net_id in net_ids:
        endpoints = get_net_routing_endpoints(pcb_data, net_id)
        if len(endpoints) >= 2:
            # Take the first two endpoints (source and target)
            net_endpoints[net_id] = endpoints[:2]

    if not net_endpoints:
        print("MPS: No nets with valid routing endpoints found")
        return list(net_ids)

    # Step 2: Compute center if not provided (centroid of all endpoints)
    if center is None:
        all_points = []
        for endpoints in net_endpoints.values():
            all_points.extend(endpoints)
        if all_points:
            center = (
                sum(p[0] for p in all_points) / len(all_points),
                sum(p[1] for p in all_points) / len(all_points)
            )
        else:
            center = (0, 0)

    # Step 3: Compute angular position for each endpoint
    def angle_from_center(point: Tuple[float, float]) -> float:
        """Compute angle from center to point in radians [0, 2*pi)."""
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        ang = math.atan2(dy, dx)
        if ang < 0:
            ang += 2 * math.pi
        return ang

    # For each net, get angles of both endpoints and normalize order
    net_angles = {}  # net_id -> (angle1, angle2) where angle1 < angle2
    for net_id, endpoints in net_endpoints.items():
        a1 = angle_from_center(endpoints[0])
        a2 = angle_from_center(endpoints[1])
        # Normalize: always store with smaller angle first
        if a1 > a2:
            a1, a2 = a2, a1
        net_angles[net_id] = (a1, a2)

    # Step 4: Detect crossing conflicts
    # Two nets cross if their intervals on the circle interleave
    # Net A with (a1, a2) and Net B with (b1, b2) cross if:
    #   a1 < b1 < a2 < b2  OR  b1 < a1 < b2 < a2
    def nets_cross(net_a: int, net_b: int) -> bool:
        """Check if two nets have crossing paths on the circular boundary."""
        a1, a2 = net_angles[net_a]
        b1, b2 = net_angles[net_b]

        # Check interleaving: one net's interval partially overlaps the other's
        # a1 < b1 < a2 < b2 means A starts, B starts, A ends, B ends = crossing
        if a1 < b1 < a2 < b2:
            return True
        if b1 < a1 < b2 < a2:
            return True
        return False

    # Build conflict graph
    net_list = list(net_angles.keys())
    conflicts = {net_id: set() for net_id in net_list}

    for i, net_a in enumerate(net_list):
        for net_b in net_list[i+1:]:
            if nets_cross(net_a, net_b):
                conflicts[net_a].add(net_b)
                conflicts[net_b].add(net_a)

    # Count total conflicts for reporting
    total_conflicts = sum(len(c) for c in conflicts.values()) // 2
    print(f"MPS: {len(net_list)} nets with {total_conflicts} crossing conflicts detected")

    # Step 5: Greedy ordering - repeatedly pick net with fewest active conflicts
    ordered = []
    remaining = set(net_list)
    round_num = 0

    while remaining:
        round_num += 1
        round_winners = []
        round_losers = set()
        active_conflicts = {net_id: len(conflicts[net_id] & remaining)
                           for net_id in remaining}

        # Process this round: pick nets with minimal conflicts
        round_remaining = set(remaining)
        while round_remaining:
            # Find net with minimum active conflicts among round_remaining
            min_conflicts = float('inf')
            best_net = None
            for net_id in round_remaining:
                # Active conflicts = conflicts with nets still in round_remaining
                active = len(conflicts[net_id] & round_remaining)
                if active < min_conflicts:
                    min_conflicts = active
                    best_net = net_id

            if best_net is None:
                break

            # This net wins this round
            round_winners.append(best_net)
            round_remaining.discard(best_net)

            # All its conflicting neighbors in round_remaining become losers
            for loser in conflicts[best_net] & round_remaining:
                round_losers.add(loser)
                round_remaining.discard(loser)

        # Add winners to ordered list, then losers go to next round
        ordered.extend(round_winners)
        remaining = round_losers

        if round_winners:
            net_names = [pcb_data.nets[nid].name for nid in round_winners[:3]]
            suffix = f"... (+{len(round_winners)-3} more)" if len(round_winners) > 3 else ""
            print(f"MPS Round {round_num}: {len(round_winners)} nets selected "
                  f"({', '.join(net_names)}{suffix})")

    # Add any nets that weren't in net_angles (no valid endpoints) at the end
    # These are nets we couldn't determine routing endpoints for
    nets_without_endpoints = [nid for nid in net_ids if nid not in net_angles]
    if nets_without_endpoints:
        print(f"MPS: {len(nets_without_endpoints)} nets without valid endpoints appended at end")
        ordered.extend(nets_without_endpoints)

    return ordered


def build_base_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                            nets_to_route: List[int],
                            extra_clearance: float = 0.0) -> GridObstacleMap:
    """Build base obstacle map with static obstacles (BGA zones, pads, pre-existing tracks/vias).

    Excludes all nets that will be routed (nets_to_route) - their stubs will be added
    per-net in the routing loop (excluding the current net being routed).

    Args:
        extra_clearance: Additional clearance to add for routing (e.g., for diff pair centerline routing)
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}
    nets_to_route_set = set(nets_to_route)

    obstacles = GridObstacleMap(num_layers)

    # Set BGA exclusion zones - block vias AND tracks on ALL layers
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
        for layer_idx in range(num_layers):
            for gx in range(gmin_x, gmax_x + 1):
                for gy in range(gmin_y, gmax_y + 1):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)

    # Precompute grid expansions (with extra clearance)
    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance + extra_clearance))

    # Add segments as obstacles (excluding nets we'll route - their stubs added per-net)
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid)

    # Add pads as obstacles (excluding nets we'll route - their pads added per-net)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance)

    return obstacles


def add_net_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig,
                                extra_clearance: float = 0.0):
    """Add a net's stub segments as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid)


def add_net_pads_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0):
    """Add a net's pads as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance)


def add_net_vias_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0):
    """Add a net's vias as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance + extra_clearance))

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid)


def add_same_net_via_clearance(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig):
    """Add via-via clearance blocking for same-net vias.

    This blocks only via placement (not track routing) near existing vias on the same net,
    enforcing DRC via-via clearance even within a single net.
    """
    coord = GridCoord(config.grid_step)

    # Via-via clearance: center-to-center distance must be >= via_size + clearance
    # So we block via placement within this radius of existing vias
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        # Only block via placement, not track routing (tracks can pass through same-net vias)
        for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)


def _add_segment_obstacle(obstacles: GridObstacleMap, seg, coord: GridCoord,
                          layer_idx: int, expansion_grid: int, via_block_grid: int):
    """Add a segment as obstacle to the map."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham line with expansion
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
        for ex in range(-via_block_grid, via_block_grid + 1):
            for ey in range(-via_block_grid, via_block_grid + 1):
                if ex*ex + ey*ey <= via_block_grid * via_block_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)

        if gx == gx2 and gy == gy2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            gx += sx
        if e2 < dx:
            err += dx
            gy += sy


def _add_via_obstacle(obstacles: GridObstacleMap, via, coord: GridCoord,
                      num_layers: int, via_track_expansion_grid: int, via_via_expansion_grid: int):
    """Add a via as obstacle to the map."""
    gx, gy = coord.to_grid(via.x, via.y)
    # Block cells for track routing
    for ex in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
        for ey in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
            if ex*ex + ey*ey <= via_track_expansion_grid * via_track_expansion_grid:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
    # Block cells for via placement
    for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
        for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                obstacles.add_blocked_via(gx + ex, gy + ey)


def _add_pad_obstacle(obstacles: GridObstacleMap, pad, coord: GridCoord,
                      layer_map: Dict[str, int], config: GridRouteConfig,
                      extra_clearance: float = 0.0):
    """Add a pad as obstacle to the map."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)

    # Rectangular expansion for track clearance
    half_x_mm = pad.size_x / 2 + config.clearance + extra_clearance
    half_y_mm = pad.size_y / 2 + config.clearance + extra_clearance
    expand_x = coord.to_grid_dist(half_x_mm)
    expand_y = coord.to_grid_dist(half_y_mm)

    for ex in range(-expand_x, expand_x + 1):
        for ey in range(-expand_y, expand_y + 1):
            for layer in pad.layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    # Via blocking near pads
    if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
        via_clear_mm = config.via_size / 2 + config.clearance
        via_expand_x = int((pad.size_x / 2 + via_clear_mm) / config.grid_step)
        via_expand_y = int((pad.size_y / 2 + via_clear_mm) / config.grid_step)
        for ex in range(-via_expand_x, via_expand_x + 1):
            for ey in range(-via_expand_y, via_expand_y + 1):
                obstacles.add_blocked_via(gx + ex, gy + ey)


def add_routed_path_obstacles(obstacles: GridObstacleMap, path: List[Tuple[int, int, int]],
                               config: GridRouteConfig):
    """Add a newly routed path as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    expansion_mm = config.track_width / 2 + config.clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = config.via_size / 2 + config.track_width / 2 + config.clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        if layer1 != layer2:
            # Via - add via obstacle
            for ex in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
                for ey in range(-via_track_expansion_grid, via_track_expansion_grid + 1):
                    if ex*ex + ey*ey <= via_track_expansion_grid * via_track_expansion_grid:
                        for layer_idx in range(num_layers):
                            obstacles.add_blocked_cell(gx1 + ex, gy1 + ey, layer_idx)
            for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                    if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                        obstacles.add_blocked_via(gx1 + ex, gy1 + ey)
        else:
            # Segment on same layer - add track obstacle using Bresenham
            dx = abs(gx2 - gx1)
            dy = abs(gy2 - gy1)
            sx = 1 if gx1 < gx2 else -1
            sy = 1 if gy1 < gy2 else -1
            err = dx - dy

            gx, gy = gx1, gy1
            while True:
                for ex in range(-expansion_grid, expansion_grid + 1):
                    for ey in range(-expansion_grid, expansion_grid + 1):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer1)
                for ex in range(-via_block_grid, via_block_grid + 1):
                    for ey in range(-via_block_grid, via_block_grid + 1):
                        if ex*ex + ey*ey <= via_block_grid * via_block_grid:
                            obstacles.add_blocked_via(gx + ex, gy + ey)

                if gx == gx2 and gy == gy2:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    gx += sx
                if e2 < dx:
                    err += dx
                    gy += sy


def add_stub_proximity_costs(obstacles: GridObstacleMap, unrouted_stubs: List[Tuple[float, float]],
                              config: GridRouteConfig):
    """Add stub proximity costs to the obstacle map."""
    coord = GridCoord(config.grid_step)
    stub_radius_grid = coord.to_grid_dist(config.stub_proximity_radius)
    stub_cost_grid = int(config.stub_proximity_cost * 1000 / config.grid_step)

    for stub_x, stub_y in unrouted_stubs:
        gcx, gcy = coord.to_grid(stub_x, stub_y)
        for dx in range(-stub_radius_grid, stub_radius_grid + 1):
            for dy in range(-stub_radius_grid, stub_radius_grid + 1):
                dist_sq = dx * dx + dy * dy
                if dist_sq <= stub_radius_grid * stub_radius_grid:
                    dist = (dist_sq ** 0.5)
                    proximity = 1.0 - (dist / stub_radius_grid) if stub_radius_grid > 0 else 1.0
                    cost = int(proximity * stub_cost_grid)
                    obstacles.set_stub_proximity(gcx + dx, gcy + dy, cost)


def build_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                       exclude_net_id: int, unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> GridObstacleMap:
    """Build Rust obstacle map from PCB data (legacy function for compatibility)."""
    # Build base map excluding just this net
    obstacles = build_base_obstacle_map(pcb_data, config, [exclude_net_id])

    # Add stub proximity costs
    if unrouted_stubs:
        add_stub_proximity_costs(obstacles, unrouted_stubs, config)

    # Add same-net via clearance blocking (for DRC - vias can't be too close even on same net)
    add_same_net_via_clearance(obstacles, pcb_data, exclude_net_id, config)

    return obstacles


def route_net(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
              unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> Optional[dict]:
    """Route a single net using the Rust router."""
    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Extract grid-only coords for routing
    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Build obstacles
    obstacles = build_obstacle_map(pcb_data, config, net_id, unrouted_stubs)

    # Add source and target positions as allowed cells to override BGA zone blocking
    # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    # NOTE: Must pass layer to only allow override on the specific layer of the endpoint
    for gx, gy, layer in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy, layer)

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    # Determine direction order
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order == "backwards":
        start_backwards = True
    else:  # "forward" or default
        start_backwards = False

    # Set up first and second direction based on order
    if start_backwards:
        first_sources, first_targets = targets_grid, sources_grid
        second_sources, second_targets = sources_grid, targets_grid
        first_label, second_label = "backwards", "forward"
    else:
        first_sources, first_targets = sources_grid, targets_grid
        second_sources, second_targets = targets_grid, sources_grid
        first_label, second_label = "forward", "backwards"

    # Try first direction, then second if first fails
    reversed_path = False
    total_iterations = 0

    path, iterations = router.route_multi(obstacles, first_sources, first_targets, config.max_iterations)
    total_iterations += iterations

    if path is None:
        print(f"No route found after {iterations} iterations ({first_label}), trying {second_label}...")
        path, iterations = router.route_multi(obstacles, second_sources, second_targets, config.max_iterations)
        total_iterations += iterations
        if path is not None:
            reversed_path = not start_backwards  # True if we ended up going backwards

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return None

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    # If path was found in reverse direction, swap sources/targets for connection logic
    if reversed_path:
        sources, targets = targets, sources

    # Find which source/target the path actually connects to
    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    # Convert path to segments and vias
    new_segments = []
    new_vias = []

    # Add connecting segment from original start to first path point if needed
    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
                    net_id=net_id
                )
                new_segments.append(seg)

    # Add connecting segment from last path point to original end if needed
    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,  # Include raw path for incremental obstacle updates
    }


def route_net_with_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              obstacles: GridObstacleMap) -> Optional[dict]:
    """Route a single net using pre-built obstacles (for incremental routing)."""
    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Add source and target positions as allowed cells to override BGA zone blocking
    # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    # NOTE: Must pass layer to only allow override on the specific layer of the endpoint
    for gx, gy, layer in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy, layer)

    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    # Determine direction order
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order == "backwards":
        start_backwards = True
    else:
        start_backwards = False

    if start_backwards:
        first_sources, first_targets = targets_grid, sources_grid
        second_sources, second_targets = sources_grid, targets_grid
        first_label, second_label = "backwards", "forward"
    else:
        first_sources, first_targets = sources_grid, targets_grid
        second_sources, second_targets = targets_grid, sources_grid
        first_label, second_label = "forward", "backwards"

    reversed_path = False
    total_iterations = 0

    path, iterations = router.route_multi(obstacles, first_sources, first_targets, config.max_iterations)
    total_iterations += iterations

    if path is None:
        print(f"No route found after {iterations} iterations ({first_label}), trying {second_label}...")
        path, iterations = router.route_multi(obstacles, second_sources, second_targets, config.max_iterations)
        total_iterations += iterations
        if path is not None:
            reversed_path = not start_backwards

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return {'failed': True, 'iterations': total_iterations}

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    if reversed_path:
        sources, targets = targets, sources

    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    new_segments = []
    new_vias = []

    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            via = Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=net_id
            )
            new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
                    net_id=net_id
                )
                new_segments.append(seg)

    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.track_width,
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
    }


def _segments_cross(seg1: Segment, seg2: Segment) -> Optional[Tuple[float, float]]:
    """Check if two segments cross (not just touch at endpoints). Returns crossing point or None."""
    # Line 1: P1 + t*(P2-P1), Line 2: P3 + u*(P4-P3)
    x1, y1 = seg1.start_x, seg1.start_y
    x2, y2 = seg1.end_x, seg1.end_y
    x3, y3 = seg2.start_x, seg2.start_y
    x4, y4 = seg2.end_x, seg2.end_y

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None  # Parallel

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

    # Check if intersection is strictly inside both segments (not at endpoints)
    eps = 0.001  # Small margin to exclude endpoint touches
    if eps < t < (1 - eps) and eps < u < (1 - eps):
        cross_x = x1 + t * (x2 - x1)
        cross_y = y1 + t * (y2 - y1)
        return (cross_x, cross_y)
    return None


def fix_self_intersections(segments: List[Segment], existing_segments: List[Segment] = None,
                           max_short_length: float = 1.0) -> List[Segment]:
    """Fix self-intersections by moving endpoints of short connector segments.

    When a short connector segment crosses a longer segment (either new or existing),
    we move the endpoint of the short segment to eliminate the crossing.

    Args:
        segments: New segments from routing
        existing_segments: Existing segments of the same net to check crossings against
        max_short_length: Maximum length for a segment to be considered "short"
    """
    if not segments:
        return segments

    # Combine existing segments by layer for cross-checking
    existing_by_layer = {}
    if existing_segments:
        for seg in existing_segments:
            if seg.layer not in existing_by_layer:
                existing_by_layer[seg.layer] = []
            existing_by_layer[seg.layer].append(seg)

    # Process each layer separately
    layer_segments = {}
    for seg in segments:
        if seg.layer not in layer_segments:
            layer_segments[seg.layer] = []
        layer_segments[seg.layer].append(seg)

    result_segments = []

    for layer, layer_segs in layer_segments.items():
        # Get existing segments on this layer
        existing_on_layer = existing_by_layer.get(layer, [])

        # Find all crossings involving short NEW segments
        # Map: new segment index -> (other segment, crossing point)
        segments_to_modify = {}

        for i, seg in enumerate(layer_segs):
            seg_len = math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)
            if seg_len > max_short_length:
                continue

            # This is a short new segment - check if it crosses any EXISTING segment
            for existing in existing_on_layer:
                cross_pt = _segments_cross(seg, existing)
                if cross_pt:
                    segments_to_modify[i] = (existing, cross_pt)
                    break

            # Also check crossings with LONG new segments on the same layer
            if i not in segments_to_modify:
                for j, other in enumerate(layer_segs):
                    if i == j:
                        continue
                    other_len = math.sqrt((other.end_x - other.start_x)**2 + (other.end_y - other.start_y)**2)
                    if other_len <= max_short_length:
                        continue  # Only check against longer segments
                    cross_pt = _segments_cross(seg, other)
                    if cross_pt:
                        segments_to_modify[i] = (other, cross_pt)
                        break

        # Process segments - modify short segments that cross existing ones
        for i, seg in enumerate(layer_segs):
            if i in segments_to_modify:
                existing, cross_pt = segments_to_modify[i]

                # Move the short segment's endpoint to an ENDPOINT of the existing segment
                # (not the crossing point, which would create a gap)
                # Choose the existing endpoint closest to the crossing point
                dist_cross_to_ex_start = math.sqrt((cross_pt[0] - existing.start_x)**2 +
                                                    (cross_pt[1] - existing.start_y)**2)
                dist_cross_to_ex_end = math.sqrt((cross_pt[0] - existing.end_x)**2 +
                                                  (cross_pt[1] - existing.end_y)**2)

                if dist_cross_to_ex_end < dist_cross_to_ex_start:
                    snap_x, snap_y = existing.end_x, existing.end_y
                else:
                    snap_x, snap_y = existing.start_x, existing.start_y

                # Determine which endpoint of the short seg to move (the one closer to crossing)
                dist_start_to_cross = math.sqrt((seg.start_x - cross_pt[0])**2 + (seg.start_y - cross_pt[1])**2)
                dist_end_to_cross = math.sqrt((seg.end_x - cross_pt[0])**2 + (seg.end_y - cross_pt[1])**2)

                if dist_start_to_cross < dist_end_to_cross:
                    # Move start to existing endpoint
                    new_seg = Segment(
                        start_x=snap_x, start_y=snap_y,
                        end_x=seg.end_x, end_y=seg.end_y,
                        width=seg.width, layer=seg.layer, net_id=seg.net_id
                    )
                else:
                    # Move end to existing endpoint
                    new_seg = Segment(
                        start_x=seg.start_x, start_y=seg.start_y,
                        end_x=snap_x, end_y=snap_y,
                        width=seg.width, layer=seg.layer, net_id=seg.net_id
                    )
                result_segments.append(new_seg)
            else:
                result_segments.append(seg)

    return result_segments


def collapse_appendices(segments: List[Segment], existing_segments: List[Segment] = None,
                        max_appendix_length: float = 1.0, vias: List[Via] = None) -> List[Segment]:
    """Collapse short appendix segments by moving dead-end vertices to junction points.

    An appendix is a short segment where one endpoint is a dead-end (degree 1) and
    the other endpoint is a junction (degree >= 2). We collapse it by moving the
    dead-end to nearly coincide with the junction (offset by 0.001mm).

    Only collapses segments where the dead-end doesn't connect to existing segments or vias.
    Also fixes self-intersections where new segments cross existing segments.
    """
    if not segments:
        return segments

    # First fix self-intersections with existing segments
    segments = fix_self_intersections(segments, existing_segments, max_appendix_length)

    # Build map of existing segment endpoints by layer (store actual coordinates for proximity check)
    existing_endpoints = {}
    if existing_segments:
        for seg in existing_segments:
            if seg.layer not in existing_endpoints:
                existing_endpoints[seg.layer] = []
            existing_endpoints[seg.layer].append((seg.start_x, seg.start_y))
            existing_endpoints[seg.layer].append((seg.end_x, seg.end_y))

    # Build map of via locations by layer (store actual coordinates and size for proximity check)
    via_locations = {}
    if vias:
        all_copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
        for via in vias:
            # Through-hole vias connect all layers
            if via.layers and 'F.Cu' in via.layers and 'B.Cu' in via.layers:
                via_layers = all_copper_layers
            elif via.layers:
                via_layers = via.layers
            else:
                via_layers = all_copper_layers
            via_size = getattr(via, 'size', 0.6)  # Default via size if not available
            for layer in via_layers:
                if layer not in via_locations:
                    via_locations[layer] = []
                via_locations[layer].append((via.x, via.y, via_size))

    # Process each layer separately
    layer_segments = {}
    for seg in segments:
        if seg.layer not in layer_segments:
            layer_segments[seg.layer] = []
        layer_segments[seg.layer].append(seg)

    result_segments = []

    def point_near_any(px, py, points_list, tolerance):
        """Check if point is within tolerance of any point in list."""
        for ex, ey in points_list:
            if math.sqrt((px - ex)**2 + (py - ey)**2) < tolerance:
                return True
        return False

    def point_near_any_via(px, py, vias_list):
        """Check if point is within via_size/4 of any via in list."""
        for vx, vy, via_size in vias_list:
            tolerance = via_size / 4
            if math.sqrt((px - vx)**2 + (py - vy)**2) < tolerance:
                return True
        return False

    for layer, layer_segs in layer_segments.items():
        layer_existing = existing_endpoints.get(layer, [])
        layer_vias = via_locations.get(layer, [])

        # Build endpoint degree map from NEW segments only
        endpoint_counts = {}
        for seg in layer_segs:
            start_key = (round(seg.start_x, 4), round(seg.start_y, 4))
            end_key = (round(seg.end_x, 4), round(seg.end_y, 4))
            endpoint_counts[start_key] = endpoint_counts.get(start_key, 0) + 1
            endpoint_counts[end_key] = endpoint_counts.get(end_key, 0) + 1

        # Find and collapse appendices
        for seg in layer_segs:
            length = math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)

            if length > max_appendix_length:
                result_segments.append(seg)
                continue

            start_key = (round(seg.start_x, 4), round(seg.start_y, 4))
            end_key = (round(seg.end_x, 4), round(seg.end_y, 4))
            start_degree = endpoint_counts.get(start_key, 0)
            end_degree = endpoint_counts.get(end_key, 0)

            # Check if endpoints connect to existing segments (with proximity tolerance) or vias
            # Use track width / 4 as proximity tolerance for segments, via size / 4 for vias
            proximity_tol = seg.width / 4
            start_connects_existing = point_near_any(seg.start_x, seg.start_y, layer_existing, proximity_tol) or point_near_any_via(seg.start_x, seg.start_y, layer_vias)
            end_connects_existing = point_near_any(seg.end_x, seg.end_y, layer_existing, proximity_tol) or point_near_any_via(seg.end_x, seg.end_y, layer_vias)

            # Appendix: one end is dead-end (degree 1, not connected to existing/vias),
            # other is junction (degree >= 2 OR connected to existing/vias)
            if (start_degree == 1 and not start_connects_existing and
                (end_degree >= 2 or end_connects_existing)):
                # Collapse: move start to nearly coincide with end (junction point)
                new_seg = Segment(
                    start_x=seg.end_x + 0.001,
                    start_y=seg.end_y,
                    end_x=seg.end_x,
                    end_y=seg.end_y,
                    width=seg.width,
                    layer=seg.layer,
                    net_id=seg.net_id
                )
                result_segments.append(new_seg)
            elif (end_degree == 1 and not end_connects_existing and
                  (start_degree >= 2 or start_connects_existing)):
                # Collapse: move end to nearly coincide with start (junction point)
                new_seg = Segment(
                    start_x=seg.start_x,
                    start_y=seg.start_y,
                    end_x=seg.start_x + 0.001,
                    end_y=seg.start_y,
                    width=seg.width,
                    layer=seg.layer,
                    net_id=seg.net_id
                )
                result_segments.append(new_seg)
            else:
                # Not an appendix or connects to existing - keep as is
                result_segments.append(seg)

    return result_segments


def fix_diff_pair_clearance(p_segments: List[Segment], n_segments: List[Segment],
                             clearance: float = 0.1, track_width: float = 0.1) -> Tuple[List[Segment], List[Segment]]:
    """Fix P-N clearance violations by shifting connector segments perpendicular.

    When P and N connector segments are too close, shift them perpendicular to their
    direction to increase clearance while maintaining connectivity.
    """
    if not p_segments or not n_segments:
        return p_segments, n_segments

    min_center_dist = clearance + track_width  # Center-to-center distance needed
    eps = 0.001  # Tolerance for coordinate matching

    def point_to_segment_dist(px, py, s):
        """Distance from point to segment, returns (distance, closest_x, closest_y)."""
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        len_sq = dx * dx + dy * dy
        if len_sq < 0.0001:
            return math.sqrt((px - s.start_x)**2 + (py - s.start_y)**2), s.start_x, s.start_y
        t = max(0, min(1, ((px - s.start_x) * dx + (py - s.start_y) * dy) / len_sq))
        cx = s.start_x + t * dx
        cy = s.start_y + t * dy
        return math.sqrt((px - cx)**2 + (py - cy)**2), cx, cy

    def segment_min_dist_info(s1, s2):
        """Find min distance and the closest points on each segment."""
        min_d = float('inf')
        best_t1, best_x2, best_y2 = 0.5, 0, 0
        for t1 in [i * 0.1 for i in range(11)]:
            x1 = s1.start_x + t1 * (s1.end_x - s1.start_x)
            y1 = s1.start_y + t1 * (s1.end_y - s1.start_y)
            d, cx, cy = point_to_segment_dist(x1, y1, s2)
            if d < min_d:
                min_d = d
                best_t1, best_x2, best_y2 = t1, cx, cy
        return min_d, best_t1, best_x2, best_y2

    def points_match(x1, y1, x2, y2):
        """Check if two points are the same within tolerance."""
        return abs(x1 - x2) < eps and abs(y1 - y2) < eps

    def update_connected_endpoints(seg_list, old_x, old_y, new_x, new_y):
        """Update endpoints of segments that were connected to (old_x, old_y)."""
        for s in seg_list:
            if points_match(s.start_x, s.start_y, old_x, old_y):
                s.start_x, s.start_y = new_x, new_y
            if points_match(s.end_x, s.end_y, old_x, old_y):
                s.end_x, s.end_y = new_x, new_y

    # Group segments by layer
    p_by_layer = {}
    n_by_layer = {}
    for s in p_segments:
        p_by_layer.setdefault(s.layer, []).append(s)
    for s in n_segments:
        n_by_layer.setdefault(s.layer, []).append(s)

    # Find and fix clearance violations on short connector segments
    for layer in set(p_by_layer.keys()) & set(n_by_layer.keys()):
        p_layer = p_by_layer[layer]
        n_layer = n_by_layer[layer]

        # Check segments and shift them perpendicular if they violate clearance
        for seg_list, other_list in [(p_layer, n_layer), (n_layer, p_layer)]:
            for seg in seg_list:
                seg_len = math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)
                if seg_len > 2.0 or seg_len < 0.02:
                    continue

                for other in other_list:
                    dist, t_closest, other_x, other_y = segment_min_dist_info(seg, other)
                    deficit = min_center_dist - dist
                    if deficit > 0.001:  # Fix any real violation
                        # Compute direction from closest point on other to our segment
                        closest_x = seg.start_x + t_closest * (seg.end_x - seg.start_x)
                        closest_y = seg.start_y + t_closest * (seg.end_y - seg.start_y)
                        # Vector from other segment's closest point to our closest point
                        away_dx = closest_x - other_x
                        away_dy = closest_y - other_y
                        away_len = math.sqrt(away_dx**2 + away_dy**2)
                        if away_len > 0.001:
                            # Normalize and compute shift amount
                            away_dx /= away_len
                            away_dy /= away_len
                            shift = deficit + 0.02  # Extra margin

                            # Save old endpoints
                            old_start_x, old_start_y = seg.start_x, seg.start_y
                            old_end_x, old_end_y = seg.end_x, seg.end_y

                            # Shift entire segment perpendicular (away from other)
                            seg.start_x += away_dx * shift
                            seg.start_y += away_dy * shift
                            seg.end_x += away_dx * shift
                            seg.end_y += away_dy * shift

                            # Update connected segments to maintain connectivity
                            update_connected_endpoints(seg_list, old_start_x, old_start_y,
                                                      seg.start_x, seg.start_y)
                            update_connected_endpoints(seg_list, old_end_x, old_end_y,
                                                      seg.end_x, seg.end_y)

    return p_segments, n_segments


def add_route_to_pcb_data(pcb_data: PCBData, result: dict) -> None:
    """Add routed segments and vias to PCB data for subsequent routes to see."""
    new_segments = result['new_segments']
    if not new_segments:
        return

    # Get all unique net_ids from new segments
    net_ids = set(s.net_id for s in new_segments)

    # Get new vias for appendix checking
    new_vias = result.get('new_vias', [])

    # Process each net separately for same-net cleanup
    cleaned_segments = []
    for net_id in net_ids:
        net_segs = [s for s in new_segments if s.net_id == net_id]
        existing_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        # Include both new vias and existing vias for this net
        net_vias = [v for v in new_vias if v.net_id == net_id]
        net_vias.extend([v for v in pcb_data.vias if v.net_id == net_id])
        cleaned = collapse_appendices(net_segs, existing_segments, vias=net_vias)
        cleaned_segments.extend(cleaned)

    # For diff pairs (2 nets), also fix P-N clearance
    if len(net_ids) == 2:
        net_list = list(net_ids)
        p_segs = [s for s in cleaned_segments if s.net_id == net_list[0]]
        n_segs = [s for s in cleaned_segments if s.net_id == net_list[1]]
        p_segs, n_segs = fix_diff_pair_clearance(p_segs, n_segs)
        # Re-run self-intersection fix since P-N clearance shift may create same-net crossings
        p_existing = [s for s in pcb_data.segments if s.net_id == net_list[0]]
        n_existing = [s for s in pcb_data.segments if s.net_id == net_list[1]]
        p_segs = fix_self_intersections(p_segs, p_existing)
        n_segs = fix_self_intersections(n_segs, n_existing)
        cleaned_segments = p_segs + n_segs

    # Filter out very short (degenerate) segments
    def seg_len(s):
        return math.sqrt((s.end_x - s.start_x)**2 + (s.end_y - s.start_y)**2)
    cleaned_segments = [s for s in cleaned_segments if seg_len(s) > 0.01]

    for seg in cleaned_segments:
        pcb_data.segments.append(seg)
    for via in result['new_vias']:
        pcb_data.vias.append(via)
    # Update result so output file also gets cleaned segments
    result['new_segments'] = cleaned_segments


def get_diff_pair_endpoints(pcb_data: PCBData, p_net_id: int, n_net_id: int,
                             config: GridRouteConfig) -> Tuple[List, List, str]:
    """
    Find source and target endpoints for a differential pair.

    Returns:
        (sources, targets, error_message)
        - sources: List of (p_gx, p_gy, n_gx, n_gy, layer_idx)
        - targets: List of (p_gx, p_gy, n_gx, n_gy, layer_idx)
        - error_message: None if successful, otherwise describes why routing can't proceed
    """
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    # Get endpoints for P and N nets separately
    # Use stub free ends for diff pairs to get the actual stub tips
    p_sources, p_targets, p_error = get_net_endpoints(pcb_data, p_net_id, config, use_stub_free_ends=True)
    n_sources, n_targets, n_error = get_net_endpoints(pcb_data, n_net_id, config, use_stub_free_ends=True)

    if p_error:
        return [], [], f"P net: {p_error}"
    if n_error:
        return [], [], f"N net: {n_error}"

    if not p_sources or not p_targets:
        return [], [], "P net has no valid source/target endpoints"
    if not n_sources or not n_targets:
        return [], [], "N net has no valid source/target endpoints"

    # Determine source vs target based on proximity between P and N endpoints.
    # The "source" side is where P and N endpoints are closest together,
    # and "target" side is where the other P and N endpoints are closest.
    # This ensures P and N agree on which end is source vs target.

    # Combine all P endpoints and all N endpoints (ignoring the source/target
    # classification from get_net_endpoints since it may differ between P and N)
    p_all = p_sources + p_targets
    n_all = n_sources + n_targets

    def find_closest_pair(p_endpoints, n_endpoints):
        """Find the P and N endpoints that are closest to each other on same layer."""
        best_p, best_n = None, None
        best_dist = float('inf')
        for p in p_endpoints:
            p_gx, p_gy, p_layer = p[0], p[1], p[2]
            for n in n_endpoints:
                n_gx, n_gy, n_layer = n[0], n[1], n[2]
                if n_layer != p_layer:
                    continue
                dist = abs(p_gx - n_gx) + abs(p_gy - n_gy)
                if dist < best_dist:
                    best_dist = dist
                    best_p, best_n = p, n
        return best_p, best_n, best_dist

    # Find the closest P-N pair - this becomes one end (we'll call it "source")
    p_src, n_src, src_dist = find_closest_pair(p_all, n_all)
    if p_src is None or n_src is None:
        return [], [], "Could not find matching P and N endpoints on same layer"

    # Remove the matched endpoints from consideration
    p_remaining = [p for p in p_all if p != p_src]
    n_remaining = [n for n in n_all if n != n_src]

    if not p_remaining or not n_remaining:
        return [], [], "Not enough endpoints for source and target"

    # Find the closest P-N pair from remaining - this becomes the other end ("target")
    p_tgt, n_tgt, tgt_dist = find_closest_pair(p_remaining, n_remaining)
    if p_tgt is None or n_tgt is None:
        return [], [], "Could not find matching P and N target endpoints on same layer"

    # Build the paired source and target tuples
    paired_sources = [(
        p_src[0], p_src[1],  # P grid coords
        n_src[0], n_src[1],  # N grid coords
        p_src[2],  # layer
        p_src[3], p_src[4],  # P original coords
        n_src[3], n_src[4]  # N original coords
    )]

    paired_targets = [(
        p_tgt[0], p_tgt[1],  # P grid coords
        n_tgt[0], n_tgt[1],  # N grid coords
        p_tgt[2],  # layer
        p_tgt[3], p_tgt[4],  # P original coords
        n_tgt[3], n_tgt[4]  # N original coords
    )]

    if not paired_sources:
        return [], [], "Could not match P and N source endpoints"
    if not paired_targets:
        return [], [], "Could not match P and N target endpoints"

    return paired_sources, paired_targets, None


def simplify_path(path):
    """
    Simplify a path by removing intermediate points on straight lines.
    Only keeps corner points and endpoints.

    Args:
        path: List of (gx, gy, layer) grid coordinates

    Returns:
        Simplified path with collinear points removed
    """
    if len(path) <= 2:
        return list(path)

    result = [path[0]]

    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        next_pt = path[i + 1]

        # Keep if layer changes
        if prev[2] != curr[2] or curr[2] != next_pt[2]:
            result.append(curr)
            continue

        # Direction vectors
        dx1 = curr[0] - prev[0]
        dy1 = curr[1] - prev[1]
        dx2 = next_pt[0] - curr[0]
        dy2 = next_pt[1] - curr[1]

        # Normalize (handle zero case)
        len1 = math.sqrt(dx1*dx1 + dy1*dy1) or 1
        len2 = math.sqrt(dx2*dx2 + dy2*dy2) or 1

        # Check if directions are the same (collinear)
        ndx1, ndy1 = dx1/len1, dy1/len1
        ndx2, ndy2 = dx2/len2, dy2/len2

        # Not collinear if directions differ (keep this corner point)
        if abs(ndx1 - ndx2) > 0.01 or abs(ndy1 - ndy2) > 0.01:
            result.append(curr)

    result.append(path[-1])
    return result


def smooth_path_for_diffpair(path, min_segment_length=3):
    """
    Smooth a path by removing short zigzag segments that cause diff pair crossing.

    For differential pairs, short alternating segments (e.g., diagonal-horizontal-diagonal)
    cause perpendicular offsets to swing wildly. This function removes intermediate points
    that create very short segments.

    Args:
        path: List of (gx, gy, layer) grid coordinates (already simplified)
        min_segment_length: Minimum segment length in grid units to keep

    Returns:
        Smoothed path with short zigzag segments removed
    """
    if len(path) <= 2:
        return list(path)

    result = [path[0]]

    i = 1
    while i < len(path) - 1:
        curr = path[i]

        # Check segment lengths
        prev = result[-1]
        next_pt = path[i + 1]

        # Distance from previous kept point to current
        dist_prev = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)

        # Distance from current to next
        dist_next = math.sqrt((next_pt[0] - curr[0])**2 + (next_pt[1] - curr[1])**2)

        # Keep if layer changes
        if prev[2] != curr[2] or curr[2] != next_pt[2]:
            result.append(curr)
            i += 1
            continue

        # Skip if both segments are short (part of a zigzag)
        if dist_prev < min_segment_length and dist_next < min_segment_length:
            # Skip this point, let it connect directly
            i += 1
            continue

        result.append(curr)
        i += 1

    result.append(path[-1])
    return result


def create_parallel_path_float(centerline_path, coord, sign, spacing_mm=0.1, start_dir=None, end_dir=None):
    """
    Create a path parallel to centerline using floating-point perpendicular offsets.

    Uses bisector-based offsets at corners for smooth parallel paths.

    Args:
        centerline_path: List of (gx, gy, layer) grid coordinates
        coord: GridCoord converter for grid<->float
        sign: +1 for one track, -1 for the other
        spacing_mm: Distance from centerline in mm
        start_dir: Optional (dx, dy) direction to use at start point for perpendicular calc
        end_dir: Optional (dx, dy) direction to use at end point for perpendicular calc

    Returns:
        List of (x, y, layer) floating-point coordinates
    """
    if len(centerline_path) < 2:
        return [(coord.to_float(p[0], p[1])[0], coord.to_float(p[0], p[1])[1], p[2])
                for p in centerline_path]

    result = []

    for i in range(len(centerline_path)):
        gx, gy, layer = centerline_path[i]
        x, y = coord.to_float(gx, gy)

        use_corner_scale = True  # Only apply corner scaling for bisector calculations
        if i == 0:
            # First point: use start_dir if provided, else perpendicular to first segment
            if start_dir is not None:
                dx, dy = start_dir
                use_corner_scale = False  # Provided direction, no corner scaling
            else:
                next_x, next_y = coord.to_float(centerline_path[1][0], centerline_path[1][1])
                dx, dy = next_x - x, next_y - y
                use_corner_scale = False  # Single segment, no corner scaling
        elif i == len(centerline_path) - 1:
            # Last point: use end_dir if provided, else perpendicular to last segment
            if end_dir is not None:
                dx, dy = end_dir
                use_corner_scale = False  # Provided direction, no corner scaling
            else:
                prev_x, prev_y = coord.to_float(centerline_path[i-1][0], centerline_path[i-1][1])
                dx, dy = x - prev_x, y - prev_y
                use_corner_scale = False  # Single segment, no corner scaling
        else:
            # Corner: use bisector of incoming and outgoing directions
            prev = centerline_path[i-1]
            next_pt = centerline_path[i+1]

            if prev[2] != layer or next_pt[2] != layer:
                # Layer change - use incoming direction
                prev_x, prev_y = coord.to_float(prev[0], prev[1])
                dx, dy = x - prev_x, y - prev_y
            else:
                prev_x, prev_y = coord.to_float(prev[0], prev[1])
                next_x, next_y = coord.to_float(next_pt[0], next_pt[1])

                dx_in, dy_in = x - prev_x, y - prev_y
                dx_out, dy_out = next_x - x, next_y - y

                len_in = math.sqrt(dx_in*dx_in + dy_in*dy_in) or 1
                len_out = math.sqrt(dx_out*dx_out + dy_out*dy_out) or 1

                # Bisector direction (sum of unit vectors)
                dx = dx_in/len_in + dx_out/len_out
                dy = dy_in/len_in + dy_out/len_out

                if abs(dx) < 0.01 and abs(dy) < 0.01:
                    dx, dy = dx_in, dy_in

        # Normalize and compute perpendicular offset
        length = math.sqrt(dx*dx + dy*dy) or 1
        ndx, ndy = dx/length, dy/length

        # Corner compensation: scale offset by 2/length to maintain perpendicular distance
        # When summing two unit vectors, length = 2*cos(θ/2) where θ is angle between them
        # To maintain spacing_mm perpendicular to each segment, multiply by 2/length
        # Cap the scaling to avoid extreme miter extensions at very sharp corners (>135° turn)
        # Only apply at actual corners (bisector calculations), not at endpoints
        if use_corner_scale:
            corner_scale = min(2.0 / length, 3.0) if length > 0.1 else 1.0
        else:
            corner_scale = 1.0

        perp_x = -ndy * sign * spacing_mm * corner_scale
        perp_y = ndx * sign * spacing_mm * corner_scale

        result.append((x + perp_x, y + perp_y, layer))

    return result


def route_diff_pair_with_obstacles(pcb_data: PCBData, diff_pair: DiffPair,
                                    config: GridRouteConfig,
                                    obstacles: GridObstacleMap) -> Optional[dict]:
    """
    Route a differential pair using centerline + offset approach.

    1. Routes a single centerline path using A* (GridRouter)
    2. Simplifies the path by removing collinear points
    3. Creates P and N paths using perpendicular offsets from centerline
    """
    p_net_id = diff_pair.p_net_id
    n_net_id = diff_pair.n_net_id

    # Find endpoints
    sources, targets, error = get_diff_pair_endpoints(pcb_data, p_net_id, n_net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Get P and N source/target coordinates
    src = sources[0]
    tgt = targets[0]

    p_src_gx, p_src_gy = src[0], src[1]
    n_src_gx, n_src_gy = src[2], src[3]
    src_layer = src[4]

    p_tgt_gx, p_tgt_gy = tgt[0], tgt[1]
    n_tgt_gx, n_tgt_gy = tgt[2], tgt[3]
    tgt_layer = tgt[4]

    # Get original stub positions (in mm)
    p_src_x, p_src_y = src[5], src[6]
    n_src_x, n_src_y = src[7], src[8]
    p_tgt_x, p_tgt_y = tgt[5], tgt[6]
    n_tgt_x, n_tgt_y = tgt[7], tgt[8]

    # Calculate spacing from actual P-N stub distance (use average of source and target)
    src_pn_dist = math.sqrt((p_src_x - n_src_x)**2 + (p_src_y - n_src_y)**2)
    tgt_pn_dist = math.sqrt((p_tgt_x - n_tgt_x)**2 + (p_tgt_y - n_tgt_y)**2)
    avg_pn_dist = (src_pn_dist + tgt_pn_dist) / 2
    spacing_mm = avg_pn_dist / 2  # Half-spacing for each track from centerline
    print(f"  P-N spacing: src={src_pn_dist:.3f}mm, tgt={tgt_pn_dist:.3f}mm, using={avg_pn_dist:.3f}mm (offset={spacing_mm:.3f}mm)")

    # Get segments for P and N nets to find stub directions
    p_segments = [s for s in pcb_data.segments if s.net_id == p_net_id]
    n_segments = [s for s in pcb_data.segments if s.net_id == n_net_id]

    src_layer_name = layer_names[src_layer]
    tgt_layer_name = layer_names[tgt_layer]

    # Get stub directions at source and target
    p_src_dir = get_stub_direction(p_segments, p_src_x, p_src_y, src_layer_name)
    n_src_dir = get_stub_direction(n_segments, n_src_x, n_src_y, src_layer_name)
    p_tgt_dir = get_stub_direction(p_segments, p_tgt_x, p_tgt_y, tgt_layer_name)
    n_tgt_dir = get_stub_direction(n_segments, n_tgt_x, n_tgt_y, tgt_layer_name)

    # Average P and N directions at each end (they should be similar for a diff pair)
    src_dir_x = (p_src_dir[0] + n_src_dir[0]) / 2
    src_dir_y = (p_src_dir[1] + n_src_dir[1]) / 2
    tgt_dir_x = (p_tgt_dir[0] + n_tgt_dir[0]) / 2
    tgt_dir_y = (p_tgt_dir[1] + n_tgt_dir[1]) / 2

    # Normalize the averaged directions
    src_dir_len = math.sqrt(src_dir_x*src_dir_x + src_dir_y*src_dir_y)
    tgt_dir_len = math.sqrt(tgt_dir_x*tgt_dir_x + tgt_dir_y*tgt_dir_y)
    if src_dir_len > 0:
        src_dir_x /= src_dir_len
        src_dir_y /= src_dir_len
    if tgt_dir_len > 0:
        tgt_dir_x /= tgt_dir_len
        tgt_dir_y /= tgt_dir_len

    # Calculate centerline midpoints between P and N stubs
    center_src_x = (p_src_x + n_src_x) / 2
    center_src_y = (p_src_y + n_src_y) / 2
    center_tgt_x = (p_tgt_x + n_tgt_x) / 2
    center_tgt_y = (p_tgt_y + n_tgt_y) / 2

    # Get setback range
    min_setback = config.min_diff_pair_centerline_setback
    max_setback = config.max_diff_pair_centerline_setback

    print(f"  Centerline setback: {min_setback}mm to {max_setback}mm")
    print(f"  Source direction: ({src_dir_x:.2f}, {src_dir_y:.2f}), target direction: ({tgt_dir_x:.2f}, {tgt_dir_y:.2f})")

    # Generate candidate points along the setback line (from min to max distance)
    # Sample at grid_step intervals
    num_samples = max(1, int((max_setback - min_setback) / config.grid_step) + 1)

    center_sources = []
    center_targets = []

    allow_radius = 2
    for i in range(num_samples):
        if num_samples > 1:
            setback = min_setback + (max_setback - min_setback) * i / (num_samples - 1)
        else:
            setback = min_setback

        # Calculate source candidate
        if src_dir_len > 0:
            src_x = center_src_x + src_dir_x * setback
            src_y = center_src_y + src_dir_y * setback
            src_gx, src_gy = coord.to_grid(src_x, src_y)

            # Check if blocked
            if not obstacles.is_blocked(src_gx, src_gy, src_layer):
                center_sources.append((src_gx, src_gy, src_layer))
                # Add allowed cells around this point
                for dx in range(-allow_radius, allow_radius + 1):
                    for dy in range(-allow_radius, allow_radius + 1):
                        obstacles.add_allowed_cell(src_gx + dx, src_gy + dy)
                obstacles.add_source_target_cell(src_gx, src_gy, src_layer)

        # Calculate target candidate
        if tgt_dir_len > 0:
            tgt_x = center_tgt_x + tgt_dir_x * setback
            tgt_y = center_tgt_y + tgt_dir_y * setback
            tgt_gx, tgt_gy = coord.to_grid(tgt_x, tgt_y)

            # Check if blocked
            if not obstacles.is_blocked(tgt_gx, tgt_gy, tgt_layer):
                center_targets.append((tgt_gx, tgt_gy, tgt_layer))
                # Add allowed cells around this point
                for dx in range(-allow_radius, allow_radius + 1):
                    for dy in range(-allow_radius, allow_radius + 1):
                        obstacles.add_allowed_cell(tgt_gx + dx, tgt_gy + dy)
                obstacles.add_source_target_cell(tgt_gx, tgt_gy, tgt_layer)

    # Remove duplicate grid coordinates
    center_sources = list(dict.fromkeys(center_sources))
    center_targets = list(dict.fromkeys(center_targets))

    if not center_sources:
        print(f"  No valid source candidates found (all blocked in setback range)")
        return {'failed': True, 'iterations': 0}
    if not center_targets:
        print(f"  No valid target candidates found (all blocked in setback range)")
        return {'failed': True, 'iterations': 0}

    print(f"  Found {len(center_sources)} source candidates, {len(center_targets)} target candidates")

    # Create router for centerline
    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    path, iterations = router.route_multi(obstacles, center_sources, center_targets, config.max_iterations)
    total_iterations = iterations

    if path is None:
        # Try reverse direction
        print(f"No route found after {iterations} iterations, trying backwards...")
        path, iter2 = router.route_multi(obstacles, center_targets, center_sources, config.max_iterations)
        total_iterations += iter2
        if path is not None:
            path = list(reversed(path))

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return {'failed': True, 'iterations': total_iterations}

    # Count layer transitions (via changes) in the path
    layer_transitions = 0
    for i in range(1, len(path)):
        if path[i][2] != path[i-1][2]:
            layer_transitions += 1

    # Limit to single via transition for diff pairs
    if layer_transitions > 1:
        print(f"Route found but has {layer_transitions} layer transitions (max 1 allowed for diff pairs)")
        return {'failed': True, 'iterations': total_iterations}

    # Simplify path by removing collinear points
    simplified_path = simplify_path(path)
    print(f"Route found in {total_iterations} iterations, path: {len(path)} -> {len(simplified_path)} points")

    # Store paths for debug output (converted to float coordinates)
    raw_astar_path = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], layer)
                      for gx, gy, layer in path]
    simplified_path_float = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], layer)
                             for gx, gy, layer in simplified_path]

    # Add turn segments at start and end to face the connector stubs
    # This makes the centerline turn to align with stub direction before connecting
    # Use a shorter turn length to reduce connector segment length
    turn_length_mm = setback * 0.3  # Length of the turn segment in mm
    turn_length_grid = int(turn_length_mm / config.grid_step)

    if len(simplified_path) >= 2 and turn_length_grid > 0:
        # Add turn segment at start (facing source stubs)
        first_gx, first_gy, first_layer = simplified_path[0]
        # Move backward along stub direction to create turn point
        turn_start_gx = first_gx - int(src_dir_x * turn_length_grid)
        turn_start_gy = first_gy - int(src_dir_y * turn_length_grid)
        simplified_path.insert(0, (turn_start_gx, turn_start_gy, first_layer))

        # Add turn segment at end (facing target stubs)
        last_gx, last_gy, last_layer = simplified_path[-1]
        # Move toward stubs (opposite of tgt_dir which points outward from stubs)
        turn_end_gx = last_gx - int(tgt_dir_x * turn_length_grid)
        turn_end_gy = last_gy - int(tgt_dir_y * turn_length_grid)
        simplified_path.append((turn_end_gx, turn_end_gy, last_layer))

        print(f"  Added turn segments: path now {len(simplified_path)} points")


    # Determine which side of the centerline P is on
    # Use the first segment direction of the simplified path
    if len(simplified_path) >= 2:
        first_gx, first_gy, _ = simplified_path[0]
        second_gx, second_gy, _ = simplified_path[1]
        first_x, first_y = coord.to_float(first_gx, first_gy)
        second_x, second_y = coord.to_float(second_gx, second_gy)
        path_dir_x = second_x - first_x
        path_dir_y = second_y - first_y
    else:
        path_dir_x, path_dir_y = 1.0, 0.0

    # Vector from source midpoint to P source position
    src_midpoint_x = (p_src_x + n_src_x) / 2
    src_midpoint_y = (p_src_y + n_src_y) / 2
    to_p_dx = p_src_x - src_midpoint_x
    to_p_dy = p_src_y - src_midpoint_y

    # Cross product: determines which side of the path direction P is on at source
    src_cross = path_dir_x * to_p_dy - path_dir_y * to_p_dx
    src_p_sign = +1 if src_cross >= 0 else -1

    # Calculate target polarity using path direction at target end
    # Path arrives at target, so use negative of last segment direction
    if len(simplified_path) >= 2:
        last_gx1, last_gy1, _ = simplified_path[-2]
        last_gx2, last_gy2, _ = simplified_path[-1]
        last_cx1, last_cy1 = coord.to_float(last_gx1, last_gy1)
        last_cx2, last_cy2 = coord.to_float(last_gx2, last_gy2)
        last_len = math.sqrt((last_cx2 - last_cx1)**2 + (last_cy2 - last_cy1)**2)
        if last_len > 0.001:
            tgt_path_dir_x = (last_cx2 - last_cx1) / last_len
            tgt_path_dir_y = (last_cy2 - last_cy1) / last_len
        else:
            tgt_path_dir_x, tgt_path_dir_y = path_dir_x, path_dir_y
    else:
        tgt_path_dir_x, tgt_path_dir_y = path_dir_x, path_dir_y

    # Vector from target midpoint to P target position
    tgt_midpoint_x = (p_tgt_x + n_tgt_x) / 2
    tgt_midpoint_y = (p_tgt_y + n_tgt_y) / 2
    tgt_to_p_dx = p_tgt_x - tgt_midpoint_x
    tgt_to_p_dy = p_tgt_y - tgt_midpoint_y

    # Cross product: determines which side of the path direction P is on at target
    tgt_cross = tgt_path_dir_x * tgt_to_p_dy - tgt_path_dir_y * tgt_to_p_dx
    tgt_p_sign = +1 if tgt_cross >= 0 else -1

    # Check if polarity differs between source and target
    polarity_swap_needed = (src_p_sign != tgt_p_sign)

    # Check if path has layer changes (vias)
    has_layer_change = False
    for i in range(len(simplified_path) - 1):
        if simplified_path[i][2] != simplified_path[i + 1][2]:
            has_layer_change = True
            break

    # Choose polarity based on whether we have vias for the swap
    if polarity_swap_needed and has_layer_change:
        # Use TARGET polarity - the swap happens on the source/approach side (F.Cu)
        # This keeps the target/exit side (In1.Cu) clean with no crossover needed
        p_sign = tgt_p_sign
        print(f"  Polarity swap with vias: using TARGET polarity p_sign={p_sign}")
    elif polarity_swap_needed and not has_layer_change:
        # WARNING: Polarity swap needed but no layer change (no vias) to perform the swap
        # This will result in crossing tracks on the same layer
        p_sign = src_p_sign
        print(f"  WARNING: Polarity swap needed but no vias - tracks will cross!")
    else:
        # Use source polarity (no swap needed)
        p_sign = src_p_sign
        print(f"  Source polarity: p_sign={p_sign}")

    if polarity_swap_needed:
        print(f"  (src_p_sign={src_p_sign}, tgt_p_sign={tgt_p_sign}, has_vias={has_layer_change})")

    n_sign = -p_sign

    # Create P and N paths using perpendicular offsets from centerline
    # Use stub directions at endpoints so perpendicular offsets align with stub P-N axis
    start_stub_dir = (src_dir_x, src_dir_y)
    end_stub_dir = (-tgt_dir_x, -tgt_dir_y)  # Negate because we arrive at target stubs
    p_float_path = create_parallel_path_float(simplified_path, coord, sign=p_sign, spacing_mm=spacing_mm,
                                               start_dir=start_stub_dir, end_dir=end_stub_dir)
    n_float_path = create_parallel_path_float(simplified_path, coord, sign=n_sign, spacing_mm=spacing_mm,
                                               start_dir=start_stub_dir, end_dir=end_stub_dir)

    # Fix the first/last points to align with actual stub positions
    # This ensures connectors are parallel (same direction) rather than converging
    # Calculate where each path should start/end based on stub positions + stub direction
    connector_length = turn_length_mm  # Distance from stub to turn point
    connector_spread = 0.05  # Spread connector endpoints by this fraction to add clearance margin
    min_track_clearance = config.track_width + config.clearance  # Min center-to-center distance

    if p_float_path and n_float_path and len(p_float_path) >= 2:
        # Calculate P-N offset vectors at source and target
        src_pn_dx = (n_src_x - p_src_x) / 2  # Half-offset from centerline to each track
        src_pn_dy = (n_src_y - p_src_y) / 2
        tgt_pn_dx = (n_tgt_x - p_tgt_x) / 2
        tgt_pn_dy = (n_tgt_y - p_tgt_y) / 2

        # Replace first points with positions directly in front of each stub
        # Spread outward slightly to add clearance margin at transition to centerline
        p_float_path[0] = (p_src_x + src_dir_x * connector_length - src_pn_dx * connector_spread,
                          p_src_y + src_dir_y * connector_length - src_pn_dy * connector_spread,
                          p_float_path[0][2])
        n_float_path[0] = (n_src_x + src_dir_x * connector_length + src_pn_dx * connector_spread,
                          n_src_y + src_dir_y * connector_length + src_pn_dy * connector_spread,
                          n_float_path[0][2])

        # Replace last points with positions directly in front of each target stub
        p_float_path[-1] = (p_tgt_x + tgt_dir_x * connector_length - tgt_pn_dx * connector_spread,
                           p_tgt_y + tgt_dir_y * connector_length - tgt_pn_dy * connector_spread,
                           p_float_path[-1][2])
        n_float_path[-1] = (n_tgt_x + tgt_dir_x * connector_length + tgt_pn_dx * connector_spread,
                           n_tgt_y + tgt_dir_y * connector_length + tgt_pn_dy * connector_spread,
                           n_float_path[-1][2])

        # Check connector endpoints against opposite track and shorten if too close
        def point_to_path_dist(px, py, path):
            """Min distance from point to any segment in path (skipping endpoints)."""
            min_dist = float('inf')
            for i in range(1, len(path) - 1):
                x1, y1, _ = path[i]
                x2, y2, _ = path[i + 1]
                seg_len_sq = (x2 - x1)**2 + (y2 - y1)**2
                if seg_len_sq > 0.0001:
                    t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / seg_len_sq))
                    dist = math.sqrt((px - x1 - t * (x2 - x1))**2 + (py - y1 - t * (y2 - y1))**2)
                    min_dist = min(min_dist, dist)
            return min_dist

        def check_and_shorten_connectors(p_path, n_path, p_idx, n_idx,
                                         p_base, n_base, dir_x, dir_y, pn_dx, pn_dy):
            """Check clearance and shorten connector if needed. Returns True if shortened."""
            p_x, p_y, p_layer = p_path[p_idx]
            n_x, n_y, n_layer = n_path[n_idx]
            min_dist = min(point_to_path_dist(p_x, p_y, n_path),
                          point_to_path_dist(n_x, n_y, p_path))
            deficit = min_track_clearance - min_dist
            if deficit > 0.001:
                new_len = max(0.01, connector_length - deficit * 3)
                p_path[p_idx] = (p_base[0] + dir_x * new_len - pn_dx * connector_spread,
                                p_base[1] + dir_y * new_len - pn_dy * connector_spread, p_layer)
                n_path[n_idx] = (n_base[0] + dir_x * new_len + pn_dx * connector_spread,
                                n_base[1] + dir_y * new_len + pn_dy * connector_spread, n_layer)

        # Check source and target connectors
        check_and_shorten_connectors(p_float_path, n_float_path, 0, 0,
                                     (p_src_x, p_src_y), (n_src_x, n_src_y),
                                     src_dir_x, src_dir_y, src_pn_dx, src_pn_dy)
        check_and_shorten_connectors(p_float_path, n_float_path, -1, -1,
                                     (p_tgt_x, p_tgt_y), (n_tgt_x, n_tgt_y),
                                     tgt_dir_x, tgt_dir_y, tgt_pn_dx, tgt_pn_dy)

    # Fix via positions at layer changes to be perpendicular to centerline direction
    # This ensures P and N vias form a line perpendicular to the centerline path
    # Also ensure minimum via-via spacing is met
    # When vias are on a bend, use the average of incoming and outgoing perpendiculars
    min_via_spacing = config.via_size + config.clearance  # Minimum center-to-center distance

    if p_float_path and n_float_path and len(simplified_path) >= 2:
        for i in range(len(simplified_path) - 1):
            gx1, gy1, layer1 = simplified_path[i]
            gx2, gy2, layer2 = simplified_path[i + 1]

            if layer1 != layer2:
                # Layer change detected - calculate centerline position
                cx, cy = coord.to_float(gx1, gy1)

                # Get incoming direction (from previous point to via)
                if i > 0:
                    prev_gx, prev_gy, _ = simplified_path[i - 1]
                    prev_x, prev_y = coord.to_float(prev_gx, prev_gy)
                    in_dir_x, in_dir_y = cx - prev_x, cy - prev_y
                    in_len = math.sqrt(in_dir_x**2 + in_dir_y**2)
                    if in_len > 0.001:
                        in_dir_x, in_dir_y = in_dir_x / in_len, in_dir_y / in_len
                    else:
                        in_dir_x, in_dir_y = 1.0, 0.0
                else:
                    in_dir_x, in_dir_y = None, None

                # Get outgoing direction (from via to next point after layer change)
                if i + 2 < len(simplified_path):
                    next_gx, next_gy, _ = simplified_path[i + 2]
                    next_x, next_y = coord.to_float(next_gx, next_gy)
                    out_dir_x, out_dir_y = next_x - cx, next_y - cy
                    out_len = math.sqrt(out_dir_x**2 + out_dir_y**2)
                    if out_len > 0.001:
                        out_dir_x, out_dir_y = out_dir_x / out_len, out_dir_y / out_len
                    else:
                        out_dir_x, out_dir_y = None, None
                else:
                    out_dir_x, out_dir_y = None, None

                # Calculate perpendicular direction for via-via axis
                # If both directions available, use average of their perpendiculars
                if in_dir_x is not None and out_dir_x is not None:
                    # Perpendicular to incoming: (-in_dir_y, in_dir_x)
                    # Perpendicular to outgoing: (-out_dir_y, out_dir_x)
                    # Average them
                    perp_x = (-in_dir_y + -out_dir_y) / 2
                    perp_y = (in_dir_x + out_dir_x) / 2
                    # Normalize the average
                    perp_len = math.sqrt(perp_x**2 + perp_y**2)
                    if perp_len > 0.001:
                        perp_x, perp_y = perp_x / perp_len, perp_y / perp_len
                    else:
                        perp_x, perp_y = -in_dir_y, in_dir_x
                    print(f"  Via at bend: in=({in_dir_x:.2f},{in_dir_y:.2f}), out=({out_dir_x:.2f},{out_dir_y:.2f}), avg_perp=({perp_x:.2f},{perp_y:.2f})")
                elif in_dir_x is not None:
                    perp_x, perp_y = -in_dir_y, in_dir_x
                elif out_dir_x is not None:
                    perp_x, perp_y = -out_dir_y, out_dir_x
                else:
                    perp_x, perp_y = 1.0, 0.0

                # Use larger spacing for vias if needed to meet:
                # 1. Minimum via-via clearance
                # 2. Track-to-opposite-via clearance (with 15% safety margin for diagonal segments)
                track_via_clearance = (config.clearance + config.track_width / 2 + config.via_size / 2) * 1.15
                min_via_spacing_for_track = track_via_clearance - spacing_mm
                via_spacing = max(spacing_mm, min_via_spacing / 2, min_via_spacing_for_track)

                # Calculate P and N via positions along the via-via axis (at wider spacing)
                # These are at TARGET polarity positions initially
                p_via_x = cx + perp_x * p_sign * via_spacing
                p_via_y = cy + perp_y * p_sign * via_spacing
                n_via_x = cx + perp_x * n_sign * via_spacing
                n_via_y = cy + perp_y * n_sign * via_spacing

                # For polarity swap: swap via positions to match SOURCE polarity
                # Centerline uses TARGET polarity, but vias need to be at SOURCE positions
                # so the F.Cu approach side works correctly
                if polarity_swap_needed:
                    p_via_x, n_via_x = n_via_x, p_via_x
                    p_via_y, n_via_y = n_via_y, p_via_y
                    print(f"  Swapped via positions for polarity swap (vias at SOURCE positions)")

                # Calculate approach positions on the line perpendicular to INCOMING direction
                # through the INNER via. Outer approach is track-via clearance from inner via.
                if in_dir_x is not None and out_dir_x is not None:
                    # Perpendicular to incoming direction
                    in_perp_x, in_perp_y = -in_dir_y, in_dir_x

                    # Determine which via is inner (on inside of bend)
                    # Use current (possibly swapped) via positions
                    cross = in_dir_x * out_dir_y - in_dir_y * out_dir_x
                    if cross < 0:
                        # Turning right - inner is P if p_sign < 0, else N
                        inner_via_x = p_via_x if p_sign < 0 else n_via_x
                        inner_via_y = p_via_y if p_sign < 0 else n_via_y
                        inner_is_p = (p_sign < 0)
                    else:
                        # Turning left - inner is P if p_sign > 0, else N
                        inner_via_x = p_via_x if p_sign > 0 else n_via_x
                        inner_via_y = p_via_y if p_sign > 0 else n_via_y
                        inner_is_p = (p_sign > 0)

                    # For polarity swap: vias are swapped, so flip inner_is_p and inner_via
                    # The via labeled "P" is now at what was N's position and vice versa
                    if polarity_swap_needed:
                        inner_is_p = not inner_is_p
                        # Also update inner_via to match the flipped inner_is_p
                        if inner_is_p:
                            inner_via_x, inner_via_y = p_via_x, p_via_y
                        else:
                            inner_via_x, inner_via_y = n_via_x, n_via_y
                        print(f"  Flipped inner_is_p for polarity swap: inner_is_p={inner_is_p}")

                    # track_via_clearance already calculated above with 10% safety margin
                    diff_pair_spacing = spacing_mm * 2

                    # Calculate direction from inner via to outer via (for correct perpendicular sign)
                    if inner_is_p:
                        outer_via_x, outer_via_y = n_via_x, n_via_y
                    else:
                        outer_via_x, outer_via_y = p_via_x, p_via_y
                    inner_to_outer_x = outer_via_x - inner_via_x
                    inner_to_outer_y = outer_via_y - inner_via_y

                    # Project onto in_perp to get the correct sign for perpendicular offset
                    perp_dot = inner_to_outer_x * in_perp_x + inner_to_outer_y * in_perp_y
                    perp_sign = 1 if perp_dot >= 0 else -1

                    # Outer approach is on debug line, track-via clearance from inner via
                    # Inner approach is on debug line, diff pair spacing from outer approach
                    if inner_is_p:
                        # N is outer - place on debug line, track-via clearance from inner (P) via
                        n_old_x = inner_via_x + in_perp_x * perp_sign * track_via_clearance
                        n_old_y = inner_via_y + in_perp_y * perp_sign * track_via_clearance
                        # P is inner - place on debug line, diff pair spacing from N approach (toward center)
                        p_old_x = n_old_x - in_perp_x * perp_sign * diff_pair_spacing
                        p_old_y = n_old_y - in_perp_y * perp_sign * diff_pair_spacing
                    else:
                        # P is outer - place on debug line, track-via clearance from inner (N) via
                        p_old_x = inner_via_x + in_perp_x * perp_sign * track_via_clearance
                        p_old_y = inner_via_y + in_perp_y * perp_sign * track_via_clearance
                        # N is inner - place on debug line, diff pair spacing from P approach (toward center)
                        n_old_x = p_old_x - in_perp_x * perp_sign * diff_pair_spacing
                        n_old_y = p_old_y - in_perp_y * perp_sign * diff_pair_spacing

                    # Calculate EXIT positions using OUTGOING perpendicular through inner via
                    out_perp_x, out_perp_y = -out_dir_y, out_dir_x

                    # Project inner_to_outer onto out_perp for correct sign
                    out_perp_dot = inner_to_outer_x * out_perp_x + inner_to_outer_y * out_perp_y
                    out_perp_sign = 1 if out_perp_dot >= 0 else -1

                    # Same logic for exit: outer is track-via clearance from inner via,
                    # inner is diff pair spacing from outer
                    if inner_is_p:
                        # N is outer - place on outgoing debug line, track-via clearance from inner (P) via
                        n_exit_x = inner_via_x + out_perp_x * out_perp_sign * track_via_clearance
                        n_exit_y = inner_via_y + out_perp_y * out_perp_sign * track_via_clearance
                        # P is inner - place on outgoing debug line, diff pair spacing from N exit (toward center)
                        p_exit_x = n_exit_x - out_perp_x * out_perp_sign * diff_pair_spacing
                        p_exit_y = n_exit_y - out_perp_y * out_perp_sign * diff_pair_spacing
                    else:
                        # P is outer - place on outgoing debug line, track-via clearance from inner (N) via
                        p_exit_x = inner_via_x + out_perp_x * out_perp_sign * track_via_clearance
                        p_exit_y = inner_via_y + out_perp_y * out_perp_sign * track_via_clearance
                        # N is inner - place on outgoing debug line, diff pair spacing from P exit (toward center)
                        n_exit_x = p_exit_x - out_perp_x * out_perp_sign * diff_pair_spacing
                        n_exit_y = p_exit_y - out_perp_y * out_perp_sign * diff_pair_spacing
                else:
                    # No bend - use averaged perpendicular for both approach and exit
                    p_old_x = cx + perp_x * p_sign * spacing_mm
                    p_old_y = cy + perp_y * p_sign * spacing_mm
                    n_old_x = cx + perp_x * n_sign * spacing_mm
                    n_old_y = cy + perp_y * n_sign * spacing_mm
                    p_exit_x, p_exit_y = p_old_x, p_old_y
                    n_exit_x, n_exit_y = n_old_x, n_old_y

                # Update position at index i to approach position (track approaches via from here on layer1)
                p_float_path[i] = (p_old_x, p_old_y, layer1)
                n_float_path[i] = (n_old_x, n_old_y, layer1)

                # Insert via position after old position (short transition segment on layer1)
                p_float_path.insert(i + 1, (p_via_x, p_via_y, layer1))
                n_float_path.insert(i + 1, (n_via_x, n_via_y, layer1))

                # Update what was i+1 (now i+2) to via position on layer2
                p_float_path[i + 2] = (p_via_x, p_via_y, layer2)
                n_float_path[i + 2] = (n_via_x, n_via_y, layer2)

                # Insert exit position after via on layer2 (using outgoing perpendicular)
                p_float_path.insert(i + 3, (p_exit_x, p_exit_y, layer2))
                n_float_path.insert(i + 3, (n_exit_x, n_exit_y, layer2))

                # Check if segment from exit to continuation passes too close to opposite via
                # If so, add a jog point: go further in outgoing direction, then back to continuation
                p_jog_added = False
                n_jog_added = False
                if in_dir_x is not None and out_dir_x is not None and not polarity_swap_needed:
                    p_cont_x, p_cont_y, p_cont_layer = p_float_path[i + 4]
                    n_cont_x, n_cont_y, n_cont_layer = n_float_path[i + 4]

                    # Check P exit->cont segment vs N via (when N is inner)
                    if not inner_is_p:
                        seg_dx = p_cont_x - p_exit_x
                        seg_dy = p_cont_y - p_exit_y
                        seg_len_sq = seg_dx**2 + seg_dy**2
                        if seg_len_sq > 0.001:
                            t = max(0, min(1, ((n_via_x - p_exit_x) * seg_dx + (n_via_y - p_exit_y) * seg_dy) / seg_len_sq))
                            closest_x = p_exit_x + t * seg_dx
                            closest_y = p_exit_y + t * seg_dy
                            dist_to_nvia = math.sqrt((closest_x - n_via_x)**2 + (closest_y - n_via_y)**2)

                            if dist_to_nvia < track_via_clearance:
                                # Need jog: go in outgoing direction past the N via, then back to continuation
                                exit_to_nvia_x = n_via_x - p_exit_x
                                exit_to_nvia_y = n_via_y - p_exit_y
                                proj_dist = exit_to_nvia_x * out_dir_x + exit_to_nvia_y * out_dir_y
                                perp_dist = abs(exit_to_nvia_x * out_dir_y - exit_to_nvia_y * out_dir_x)

                                if perp_dist < track_via_clearance:
                                    extra_dist = math.sqrt(track_via_clearance**2 - perp_dist**2)
                                    jog_dist = proj_dist + extra_dist + config.via_size
                                else:
                                    jog_dist = track_via_clearance

                                jog_x = p_exit_x + out_dir_x * jog_dist
                                jog_y = p_exit_y + out_dir_y * jog_dist
                                p_float_path.insert(i + 4, (jog_x, jog_y, layer2))
                                p_jog_added = True
                                print(f"  Added P jog at ({jog_x:.2f},{jog_y:.2f}) to clear N via")

                    # Check N exit->cont segment vs P via (when P is inner)
                    else:
                        seg_dx = n_cont_x - n_exit_x
                        seg_dy = n_cont_y - n_exit_y
                        seg_len_sq = seg_dx**2 + seg_dy**2
                        if seg_len_sq > 0.001:
                            t = max(0, min(1, ((p_via_x - n_exit_x) * seg_dx + (p_via_y - n_exit_y) * seg_dy) / seg_len_sq))
                            closest_x = n_exit_x + t * seg_dx
                            closest_y = n_exit_y + t * seg_dy
                            dist_to_pvia = math.sqrt((closest_x - p_via_x)**2 + (closest_y - p_via_y)**2)

                            if dist_to_pvia < track_via_clearance:
                                exit_to_pvia_x = p_via_x - n_exit_x
                                exit_to_pvia_y = p_via_y - n_exit_y
                                proj_dist = exit_to_pvia_x * out_dir_x + exit_to_pvia_y * out_dir_y
                                perp_dist = abs(exit_to_pvia_x * out_dir_y - exit_to_pvia_y * out_dir_x)

                                if perp_dist < track_via_clearance:
                                    extra_dist = math.sqrt(track_via_clearance**2 - perp_dist**2)
                                    jog_dist = proj_dist + extra_dist + config.via_size
                                else:
                                    jog_dist = track_via_clearance

                                jog_x = n_exit_x + out_dir_x * jog_dist
                                jog_y = n_exit_y + out_dir_y * jog_dist
                                n_float_path.insert(i + 4, (jog_x, jog_y, layer2))
                                n_jog_added = True
                                print(f"  Added N jog at ({jog_x:.2f},{jog_y:.2f}) to clear P via")

                # Check if continuation points are too close to the opposite via
                # Account for jog insertion: continuation is at i+5 if jog added, else i+4
                # For non-polarity-swap: if P is outer, check P_cont vs N via; if N is outer, check N_cont vs P via
                if in_dir_x is not None and out_dir_x is not None and not polarity_swap_needed:
                    p_cont_idx = i + 5 if p_jog_added else i + 4
                    n_cont_idx = i + 5 if n_jog_added else i + 4
                    p_cont_x, p_cont_y, p_cont_layer = p_float_path[p_cont_idx]
                    n_cont_x, n_cont_y, n_cont_layer = n_float_path[n_cont_idx]

                    # Get continuation direction from next point
                    p_next_idx = p_cont_idx + 1
                    if p_next_idx < len(p_float_path):
                        p_next_x, p_next_y, _ = p_float_path[p_next_idx]
                        ext_dx = p_next_x - p_cont_x
                        ext_dy = p_next_y - p_cont_y
                        ext_len = math.sqrt(ext_dx*ext_dx + ext_dy*ext_dy)
                        if ext_len > 0.001:
                            p_cont_dir_x = ext_dx / ext_len
                            p_cont_dir_y = ext_dy / ext_len
                        else:
                            p_cont_dir_x, p_cont_dir_y = out_dir_x, out_dir_y
                    else:
                        p_cont_dir_x, p_cont_dir_y = out_dir_x, out_dir_y

                    if not inner_is_p:
                        # N is inner, P is outer - check if P continuation is too close to N via
                        p_cont_to_nvia_dist = math.sqrt((p_cont_x - n_via_x)**2 + (p_cont_y - n_via_y)**2)
                        if p_cont_to_nvia_dist < track_via_clearance:
                            shortfall = track_via_clearance - p_cont_to_nvia_dist + config.via_size
                            p_cont_x += p_cont_dir_x * shortfall
                            p_cont_y += p_cont_dir_y * shortfall
                            p_float_path[p_cont_idx] = (p_cont_x, p_cont_y, p_cont_layer)
                            print(f"  Pre-adjusted P cont for N via clearance: moved by {shortfall:.3f}mm")
                    else:
                        # P is inner, N is outer - check if N continuation is too close to P via
                        n_cont_to_pvia_dist = math.sqrt((n_cont_x - p_via_x)**2 + (n_cont_y - p_via_y)**2)
                        if n_cont_to_pvia_dist < track_via_clearance:
                            # Get N continuation direction
                            n_next_idx = n_cont_idx + 1
                            if n_next_idx < len(n_float_path):
                                n_next_x, n_next_y, _ = n_float_path[n_next_idx]
                                n_ext_dx = n_next_x - n_cont_x
                                n_ext_dy = n_next_y - n_cont_y
                                n_ext_len = math.sqrt(n_ext_dx*n_ext_dx + n_ext_dy*n_ext_dy)
                                if n_ext_len > 0.001:
                                    n_cont_dir_x = n_ext_dx / n_ext_len
                                    n_cont_dir_y = n_ext_dy / n_ext_len
                                else:
                                    n_cont_dir_x, n_cont_dir_y = out_dir_x, out_dir_y
                            else:
                                n_cont_dir_x, n_cont_dir_y = out_dir_x, out_dir_y
                            shortfall = track_via_clearance - n_cont_to_pvia_dist + config.via_size
                            n_cont_x += n_cont_dir_x * shortfall
                            n_cont_y += n_cont_dir_y * shortfall
                            n_float_path[n_cont_idx] = (n_cont_x, n_cont_y, n_cont_layer)
                            print(f"  Pre-adjusted N cont for P via clearance: moved by {shortfall:.3f}mm")

                # For polarity swap: check if N exit->cont passes too close to P via
                # N doesn't do the arc detour, so add a jog if needed
                if polarity_swap_needed and in_dir_x is not None and out_dir_x is not None:
                    n_cont_x_check, n_cont_y_check, n_cont_layer_check = n_float_path[i + 4]

                    # Check N exit->cont segment vs P via
                    seg_dx = n_cont_x_check - n_exit_x
                    seg_dy = n_cont_y_check - n_exit_y
                    seg_len_sq = seg_dx**2 + seg_dy**2
                    if seg_len_sq > 0.001:
                        t = max(0, min(1, ((p_via_x - n_exit_x) * seg_dx + (p_via_y - n_exit_y) * seg_dy) / seg_len_sq))
                        closest_x = n_exit_x + t * seg_dx
                        closest_y = n_exit_y + t * seg_dy
                        dist_to_pvia = math.sqrt((closest_x - p_via_x)**2 + (closest_y - p_via_y)**2)

                        if dist_to_pvia < track_via_clearance:
                            # Need jog for N: go in outgoing direction past P via
                            exit_to_pvia_x = p_via_x - n_exit_x
                            exit_to_pvia_y = p_via_y - n_exit_y
                            proj_dist = exit_to_pvia_x * out_dir_x + exit_to_pvia_y * out_dir_y
                            perp_dist = abs(exit_to_pvia_x * out_dir_y - exit_to_pvia_y * out_dir_x)

                            if perp_dist < track_via_clearance:
                                extra_dist = math.sqrt(track_via_clearance**2 - perp_dist**2)
                                jog_dist = proj_dist + extra_dist + config.via_size
                            else:
                                jog_dist = track_via_clearance

                            jog_x = n_exit_x + out_dir_x * jog_dist
                            jog_y = n_exit_y + out_dir_y * jog_dist
                            n_float_path.insert(i + 4, (jog_x, jog_y, layer2))
                            print(f"  Added N jog at ({jog_x:.2f},{jog_y:.2f}) to clear P via (polarity swap)")

                            # After jog insertion, check if subsequent N path points pass too close to P via
                            # The jog is at i+4, original continuation is now at i+5
                            n_indices_to_remove = []
                            n_check_start_idx = i + 5

                            for n_check_idx in range(n_check_start_idx, min(n_check_start_idx + 5, len(n_float_path))):
                                nx, ny, nlayer = n_float_path[n_check_idx]
                                dist_to_pvia = math.sqrt((nx - p_via_x)**2 + (ny - p_via_y)**2)
                                if dist_to_pvia < track_via_clearance:
                                    n_indices_to_remove.append(n_check_idx)
                                    print(f"  Removing N path point at ({nx:.3f},{ny:.3f}) - too close to P via (dist={dist_to_pvia:.3f}mm)")

                            # Remove problematic points in reverse order
                            for idx in reversed(n_indices_to_remove):
                                n_float_path.pop(idx)

                            if n_indices_to_remove:
                                print(f"  Removed {len(n_indices_to_remove)} N path points that were too close to P via")

                # For polarity swap: P needs to detour around N via
                # First segment goes at 45 deg towards incoming side and towards N via,
                # until one track_via_clearance below the N via
                if polarity_swap_needed:
                    # N via is the one we need to go around
                    n_via_pos_x, n_via_pos_y = n_via_x, n_via_y

                    # Via axis: direction from P via to N via (normalized)
                    to_n_x = n_via_pos_x - p_via_x
                    to_n_y = n_via_pos_y - p_via_y
                    via_axis_len = math.sqrt(to_n_x*to_n_x + to_n_y*to_n_y)
                    if via_axis_len > 0.001:
                        via_axis_x = to_n_x / via_axis_len
                        via_axis_y = to_n_y / via_axis_len
                    else:
                        via_axis_x, via_axis_y = 1, 0

                    # Two perpendicular options to via axis
                    perp1_x, perp1_y = via_axis_y, -via_axis_x
                    perp2_x, perp2_y = -via_axis_y, via_axis_x

                    # Get the continuation point to determine which side to detour on
                    # After p_exit insertion (i+3), the original continuation is at i+4
                    p_cont_for_perp_x, p_cont_for_perp_y, _ = p_float_path[i + 4]

                    # Vector from P via to continuation point
                    to_cont_x = p_cont_for_perp_x - p_via_x
                    to_cont_y = p_cont_for_perp_y - p_via_y

                    # Choose perpendicular that points AWAY from continuation
                    # The detour (via -> detour1 -> detour2) goes in +via_perp direction
                    # The return segment (detour3 -> p_cont) should NOT cross the detour
                    # So via_perp should point away from p_cont (negative dot product with to_cont)
                    dot1 = perp1_x * to_cont_x + perp1_y * to_cont_y
                    if dot1 < 0:
                        via_perp_x, via_perp_y = perp1_x, perp1_y
                    else:
                        via_perp_x, via_perp_y = perp2_x, perp2_y


                    # ===== CIRCULAR ARC DETOUR AROUND N VIA =====
                    # Instead of rectangular detour, use circular arc around N via

                    # Arc parameters
                    arc_radius = 1.25 * track_via_clearance  # Radius of arc around N via
                    num_arc_segments = 20  # Number of segments to approximate arc
                    track_track_clearance = config.clearance + config.track_width

                    # Get P continuation info for stopping condition
                    p_cont_x, p_cont_y, p_cont_layer = p_float_path[i + 4]
                    n_cont_x, n_cont_y, _ = n_float_path[i + 4]

                    # Pre-check: if p_cont is too close to N via, move it along P continuation direction
                    # This prevents messy post-hoc geometry fixes
                    pcont_to_nvia_dist = math.sqrt((p_cont_x - n_via_pos_x)**2 + (p_cont_y - n_via_pos_y)**2)
                    if pcont_to_nvia_dist < track_via_clearance:
                        # Need to extend p_cont. Get P continuation direction using lookahead.
                        min_ext_lookahead = arc_radius * 2
                        ext_dir_x, ext_dir_y = out_dir_x, out_dir_y  # Default fallback

                        if i + 5 < len(p_float_path):
                            # Walk along path to get averaged direction
                            total_dx, total_dy = 0.0, 0.0
                            accumulated_dist = 0.0
                            curr_x, curr_y = p_cont_x, p_cont_y

                            for j in range(i + 5, len(p_float_path)):
                                next_x, next_y, _ = p_float_path[j]
                                seg_dx = next_x - curr_x
                                seg_dy = next_y - curr_y
                                seg_len = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)

                                if seg_len > 0.001:
                                    total_dx += seg_dx
                                    total_dy += seg_dy
                                    accumulated_dist += seg_len
                                    curr_x, curr_y = next_x, next_y

                                if accumulated_dist >= min_ext_lookahead:
                                    break

                            total_len = math.sqrt(total_dx*total_dx + total_dy*total_dy)
                            if total_len > 0.001:
                                ext_dir_x = total_dx / total_len
                                ext_dir_y = total_dy / total_len

                        # Move p_cont along this direction until it has clearance from N via
                        # Use via diameter as extra margin so it scales with design
                        shortfall = track_via_clearance - pcont_to_nvia_dist + config.via_size
                        p_cont_x += ext_dir_x * shortfall
                        p_cont_y += ext_dir_y * shortfall
                        p_float_path[i + 4] = (p_cont_x, p_cont_y, p_cont_layer)
                        print(f"  Pre-adjusted p_cont for N via clearance: moved by {shortfall:.3f}mm along continuation")

                    # N continuation direction (for distance check)
                    n_cont_dx = n_cont_x - n_exit_x
                    n_cont_dy = n_cont_y - n_exit_y
                    n_cont_len = math.sqrt(n_cont_dx*n_cont_dx + n_cont_dy*n_cont_dy)
                    if n_cont_len > 0.001:
                        n_cont_dir_x = n_cont_dx / n_cont_len
                        n_cont_dir_y = n_cont_dy / n_cont_len
                    else:
                        n_cont_dir_x, n_cont_dir_y = out_dir_x, out_dir_y

                    # P continuation direction (for intersection check)
                    # Use averaged direction over a minimum lookahead distance to avoid
                    # sensitivity to tiny path segments near the via
                    min_lookahead = arc_radius * 2  # At least 2x arc radius
                    p_cont_dir_x, p_cont_dir_y = n_cont_dir_x, n_cont_dir_y  # Default fallback

                    if i + 5 < len(p_float_path):
                        # Walk along path accumulating displacement until we've traveled min_lookahead
                        total_dx = 0.0
                        total_dy = 0.0
                        accumulated_dist = 0.0
                        curr_x, curr_y = p_cont_x, p_cont_y

                        for j in range(i + 5, len(p_float_path)):
                            next_x, next_y, _ = p_float_path[j]
                            seg_dx = next_x - curr_x
                            seg_dy = next_y - curr_y
                            seg_len = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)

                            if seg_len > 0.001:
                                total_dx += seg_dx
                                total_dy += seg_dy
                                accumulated_dist += seg_len
                                curr_x, curr_y = next_x, next_y

                            if accumulated_dist >= min_lookahead:
                                break

                        # Use total displacement as direction
                        total_len = math.sqrt(total_dx*total_dx + total_dy*total_dy)
                        if total_len > 0.001:
                            p_cont_dir_x = total_dx / total_len
                            p_cont_dir_y = total_dy / total_len

                    # Starting angle: direction from N via to P via
                    start_angle = math.atan2(p_via_y - n_via_pos_y, p_via_x - n_via_pos_x)


                    # Determine rotation direction based on via_perp
                    # via_perp points away from continuation, so we rotate TOWARD continuation
                    # Cross product of via_axis and via_perp tells us rotation direction
                    # via_axis × via_perp > 0 means via_perp is counterclockwise from via_axis
                    cross_sign = via_axis_x * via_perp_y - via_axis_y * via_perp_x
                    # We want to go in the OPPOSITE direction of via_perp (toward continuation)
                    if cross_sign > 0:
                        angle_step = -2 * math.pi / num_arc_segments  # Clockwise
                    else:
                        angle_step = 2 * math.pi / num_arc_segments   # Counterclockwise

                    # First point: from P via, go toward N via until arc_radius from N via
                    # This is the point where the arc starts
                    arc_start_x = n_via_pos_x + arc_radius * math.cos(start_angle)
                    arc_start_y = n_via_pos_y + arc_radius * math.sin(start_angle)

                    # Generate arc points
                    arc_points = [(arc_start_x, arc_start_y)]
                    final_target_x, final_target_y = p_cont_x, p_cont_y  # Where to connect after arc

                    for seg_idx in range(1, num_arc_segments + 1):
                        angle = start_angle + seg_idx * angle_step
                        arc_x = n_via_pos_x + arc_radius * math.cos(angle)
                        arc_y = n_via_pos_y + arc_radius * math.sin(angle)

                        # Check stopping conditions BEFORE adding this point

                        # Condition 1: Check if we intersect the P continuation line
                        # P continuation line goes through p_cont in direction p_cont_dir
                        # Check if segment from last arc point to this arc point crosses that line
                        last_x, last_y = arc_points[-1]

                        # Vector from p_cont to last arc point
                        to_last_x = last_x - p_cont_x
                        to_last_y = last_y - p_cont_y
                        # Vector from p_cont to current arc point
                        to_curr_x = arc_x - p_cont_x
                        to_curr_y = arc_y - p_cont_y

                        # Signed distances to the P continuation line (perpendicular distance)
                        # Using cross product: distance = (point - line_point) × line_dir
                        dist_last = to_last_x * p_cont_dir_y - to_last_y * p_cont_dir_x
                        dist_curr = to_curr_x * p_cont_dir_y - to_curr_y * p_cont_dir_x

                        # If signs differ, we crossed the line
                        if dist_last * dist_curr < 0:
                            # Find intersection point
                            # Parametric: last + t*(curr - last) crosses line at t where distance = 0
                            t = dist_last / (dist_last - dist_curr)
                            intersect_x = last_x + t * (arc_x - last_x)
                            intersect_y = last_y + t * (arc_y - last_y)

                            # Check if intersection is "ahead" of arc start in P continuation direction
                            # This prevents false crossings when arc starts very close to the P cont line
                            int_to_start_x = intersect_x - arc_start_x
                            int_to_start_y = intersect_y - arc_start_y
                            ahead_dist = int_to_start_x * p_cont_dir_x + int_to_start_y * p_cont_dir_y

                            # Also check if intersection is ahead of p_cont (the continuation point)
                            int_to_pcont_x = intersect_x - p_cont_x
                            int_to_pcont_y = intersect_y - p_cont_y
                            ahead_of_pcont = int_to_pcont_x * p_cont_dir_x + int_to_pcont_y * p_cont_dir_y

                            # Need intersection to be meaningfully ahead (not just barely past start)
                            # and roughly aligned with p_cont (not way behind it)
                            min_ahead_dist = arc_radius * 0.5  # Must be at least half arc_radius ahead
                            if ahead_dist < min_ahead_dist or ahead_of_pcont < -arc_radius * 2:
                                # Intersection is too close to start or behind p_cont - ignore this crossing
                                pass  # Will fall through to add arc point normally
                            else:
                                # Valid crossing - check if far enough from N track
                                int_to_nexit_x = intersect_x - n_exit_x
                                int_to_nexit_y = intersect_y - n_exit_y
                                int_dot = int_to_nexit_x * n_cont_dir_x + int_to_nexit_y * n_cont_dir_y
                                int_dist = abs(int_to_nexit_x * n_cont_dir_y - int_to_nexit_y * n_cont_dir_x)

                                # Only stop if intersection is on opposite side of n_exit OR far enough from N track
                                if int_dot <= 0 or int_dist >= track_track_clearance:
                                    arc_points.append((intersect_x, intersect_y))
                                    final_target_x, final_target_y = intersect_x, intersect_y
                                    print(f"  Arc stopped at P continuation intersection after {seg_idx} segments")
                                    break
                                else:
                                    # Intersection too close to N track, continue arc
                                    print(f"  P continuation intersection at ({intersect_x:.3f},{intersect_y:.3f}) too close to N track (dist={int_dist:.3f}mm), continuing arc")

                        # Condition 2: Check if we're within diff_pair_spacing of N track
                        # N track goes from n_exit in direction n_cont_dir
                        # Only check if arc point is on the correct side (in the n_cont_dir direction from n_exit)
                        arc_to_nexit_x = arc_x - n_exit_x
                        arc_to_nexit_y = arc_y - n_exit_y

                        # Check if arc point is in the direction of N continuation (dot product > 0)
                        dot_to_n_cont = arc_to_nexit_x * n_cont_dir_x + arc_to_nexit_y * n_cont_dir_y

                        if dot_to_n_cont > 0:
                            # Arc point is on the side where the N track goes - check diff pair spacing
                            dist_to_n_track = abs(arc_to_nexit_x * n_cont_dir_y - arc_to_nexit_y * n_cont_dir_x)
                            if dist_to_n_track < diff_pair_spacing:
                                # Interpolate to find exact point at diff_pair_spacing
                                # Get distance of last arc point to N track
                                last_to_nexit_x = last_x - n_exit_x
                                last_to_nexit_y = last_y - n_exit_y
                                dist_last = abs(last_to_nexit_x * n_cont_dir_y - last_to_nexit_y * n_cont_dir_x)

                                if dist_last > dist_to_n_track:  # Sanity check - we're getting closer
                                    # Interpolate: find t where dist = diff_pair_spacing
                                    t = (dist_last - diff_pair_spacing) / (dist_last - dist_to_n_track)
                                    interp_x = last_x + t * (arc_x - last_x)
                                    interp_y = last_y + t * (arc_y - last_y)
                                    arc_points.append((interp_x, interp_y))
                                    print(f"  Arc stopped at diff pair spacing after {seg_idx} segments (interpolated to dist={diff_pair_spacing:.3f}mm)")
                                else:
                                    print(f"  Arc stopped at diff pair spacing after {seg_idx} segments (dist={dist_to_n_track:.3f}mm)")
                                break

                        arc_points.append((arc_x, arc_y))

                    # Now we have the arc points. Need to insert them into the path.
                    # The path currently has:
                    # i+2: (p_via_x, p_via_y, layer2) - the via position
                    # i+3: (p_exit_x, p_exit_y, layer2) - the exit position (short segment from via)
                    # i+4: (p_cont_x, p_cont_y, layer2) - the continuation point
                    
                    # Check if the segment from last arc point to p_cont passes too close to N via
                    p_cont_was_extended = False
                    orig_p_cont_x, orig_p_cont_y = p_cont_x, p_cont_y  # Save original before potential extension
                    if len(arc_points) > 0:
                        last_arc_x, last_arc_y = arc_points[-1]
                        # Calculate minimum distance from N via to segment (last_arc -> p_cont)
                        seg_dx = p_cont_x - last_arc_x
                        seg_dy = p_cont_y - last_arc_y
                        seg_len_sq = seg_dx**2 + seg_dy**2
                        if seg_len_sq > 0.001:
                            # Project N via onto segment
                            t = max(0, min(1, ((n_via_pos_x - last_arc_x) * seg_dx + (n_via_pos_y - last_arc_y) * seg_dy) / seg_len_sq))
                            closest_x = last_arc_x + t * seg_dx
                            closest_y = last_arc_y + t * seg_dy
                            dist_to_nvia = math.sqrt((closest_x - n_via_pos_x)**2 + (closest_y - n_via_pos_y)**2)
                            if dist_to_nvia < track_via_clearance:
                                # Segment passes too close to N via, extend p_cont away from N via
                                shortfall = track_via_clearance - dist_to_nvia + 0.05  # Extra margin
                                # Move p_cont perpendicular to segment, away from N via
                                pcont_to_nvia_x = p_cont_x - n_via_pos_x
                                pcont_to_nvia_y = p_cont_y - n_via_pos_y
                                pcont_nvia_dist = math.sqrt(pcont_to_nvia_x**2 + pcont_to_nvia_y**2)
                                if pcont_nvia_dist > 0.001:
                                    away_x = pcont_to_nvia_x / pcont_nvia_dist
                                    away_y = pcont_to_nvia_y / pcont_nvia_dist
                                    p_cont_x += away_x * shortfall
                                    p_cont_y += away_y * shortfall
                                    p_float_path[i + 4] = (p_cont_x, p_cont_y, p_cont_layer)
                                    p_cont_was_extended = True
                                    print(f"  Extended p_cont for N via clearance (segment was {dist_to_nvia:.3f}mm from via, need {track_via_clearance:.3f}mm)")

                    # If p_cont was extended, add a return point to bring P back parallel to N
                    return_point = None
                    if p_cont_was_extended:
                        # Calculate target position: on the original P track line
                        # The original P track goes through orig_p_cont in direction p_cont_dir

                        # Distance along P continuation from orig_p_cont to extended p_cont (projected)
                        pcont_to_orig_x = p_cont_x - orig_p_cont_x
                        pcont_to_orig_y = p_cont_y - orig_p_cont_y
                        proj_dist = pcont_to_orig_x * p_cont_dir_x + pcont_to_orig_y * p_cont_dir_y
                        # Add a bit more distance to ensure we're past the extended section
                        return_along_dist = proj_dist + track_via_clearance

                        # Return point: on original P line at return_along_dist from orig_p_cont
                        return_x = orig_p_cont_x + p_cont_dir_x * return_along_dist
                        return_y = orig_p_cont_y + p_cont_dir_y * return_along_dist

                        return_point = (return_x, return_y, p_cont_layer)
                        print(f"  Adding return point at ({return_x:.3f}, {return_y:.3f}) to restore parallel tracks")

                    # Check if the segment from last arc point to p_cont would violate N track clearance
                    # If so, extend p_cont further along the P continuation direction
                    if len(arc_points) > 0:
                        last_arc_x, last_arc_y = arc_points[-1]
                        
                        # Check if p_cont is on the "wrong side" of n_exit (in n_cont_dir direction)
                        # and too close to N track
                        pcont_to_nexit_x = p_cont_x - n_exit_x
                        pcont_to_nexit_y = p_cont_y - n_exit_y
                        pcont_dot = pcont_to_nexit_x * n_cont_dir_x + pcont_to_nexit_y * n_cont_dir_y
                        pcont_dist = abs(pcont_to_nexit_x * n_cont_dir_y - pcont_to_nexit_y * n_cont_dir_x)
                        
                        if pcont_dot > 0 and pcont_dist < track_track_clearance:
                            # p_cont is too close to N track, need to extend it along P continuation
                            # Move perpendicular to N track for clearance (more reliable than along p_cont_dir)
                            shortfall = track_track_clearance - pcont_dist + 0.05  # Small margin

                            # Perpendicular to n_cont_dir, pointing away from N track
                            # Determine which perpendicular direction to use based on current p_cont position
                            n_perp_x, n_perp_y = -n_cont_dir_y, n_cont_dir_x
                            # Check if this perpendicular points away from N track (same side as p_cont relative to N line)
                            test_dot = pcont_to_nexit_x * n_perp_x + pcont_to_nexit_y * n_perp_y
                            if test_dot < 0:
                                n_perp_x, n_perp_y = -n_perp_x, -n_perp_y

                            new_pcont_x = p_cont_x + shortfall * n_perp_x
                            new_pcont_y = p_cont_y + shortfall * n_perp_y
                            print(f"  Extended p_cont from ({p_cont_x:.3f},{p_cont_y:.3f}) to ({new_pcont_x:.3f},{new_pcont_y:.3f}) for N track clearance (moved {shortfall:.3f}mm)")
                            p_cont_x, p_cont_y = new_pcont_x, new_pcont_y
                            p_float_path[i + 4] = (p_cont_x, p_cont_y, p_cont_layer)

                    # We want to replace i+3 (exit) with: arc_start, then arc points, then final_target
                    # The segment from p_via to arc_start goes toward N via

                    # Build the detour points list
                    detour_points = []
                    for ax, ay in arc_points:
                        detour_points.append((ax, ay, layer2))

                    # Update the path
                    # i+2 stays as via
                    # i+3 becomes arc_start (first point going toward N via)
                    # Then insert arc points
                    # Final point connects to continuation

                    p_float_path[i + 3] = (arc_points[0][0], arc_points[0][1], layer2)

                    # Insert remaining arc points after i+3
                    insert_idx = i + 4
                    for j, (ax, ay) in enumerate(arc_points[1:]):
                        p_float_path.insert(insert_idx + j, (ax, ay, layer2))

                    # Insert return point after p_cont if we extended it
                    if return_point is not None:
                        # p_cont is now at index i + 4 + len(arc_points) - 1 = i + 3 + len(arc_points)
                        # Insert return point after p_cont
                        return_insert_idx = i + 4 + len(arc_points)
                        p_float_path.insert(return_insert_idx, return_point)

                    # Update the continuation point if we found an intersection
                    if final_target_x != p_cont_x or final_target_y != p_cont_y:
                        # The final arc point IS the new connection point
                        # It will connect to the original p_cont
                        pass

                    print(f"  Circular arc detour: {len(arc_points)} points around N via at radius {arc_radius:.3f}mm")

                    # After arc insertion, check if any subsequent path points pass too close to N via
                    # These are remnants of the original path that may have zigzagged through the via area
                    # p_cont is now at index i + 3 + len(arc_points) - 1 = i + 2 + len(arc_points)
                    p_cont_idx = i + 2 + len(arc_points)

                    # Check points after p_cont for N via clearance violations
                    # Keep track of indices to remove (points that are too close to N via)
                    indices_to_remove = []
                    check_start_idx = p_cont_idx + 1

                    for check_idx in range(check_start_idx, min(check_start_idx + 5, len(p_float_path))):
                        px, py, player = p_float_path[check_idx]
                        dist_to_nvia = math.sqrt((px - n_via_pos_x)**2 + (py - n_via_pos_y)**2)
                        if dist_to_nvia < track_via_clearance:
                            indices_to_remove.append(check_idx)
                            print(f"  Removing path point at ({px:.3f},{py:.3f}) - too close to N via (dist={dist_to_nvia:.3f}mm)")

                    # Remove problematic points in reverse order to maintain correct indices
                    for idx in reversed(indices_to_remove):
                        p_float_path.pop(idx)

                    if indices_to_remove:
                        print(f"  Removed {len(indices_to_remove)} path points that were too close to N via")

                    # Path already updated in arc detour code above

    # Convert floating-point paths to segments and vias
    new_segments = []
    new_vias = []

    # DEBUG: Use different layers to visualize different segment types
    DEBUG_LAYERS = False
    DEBUG_CENTERLINE_LAYER = 'In3.Cu'  # Centerline parallel path segments
    DEBUG_CONNECTOR_LAYER = 'In4.Cu'   # Connectors from stubs to centerline

    def float_path_to_geometry(float_path, net_id, original_start, original_end):
        """Convert floating-point path (x, y, layer) to segments and vias."""
        segs = []
        vias = []

        def get_layer_name(layer_idx):
            """Get layer name, handling special debug indices (-4=In4.Cu, -5=In5.Cu, -6=In6.Cu, -7=In7.Cu)."""
            if layer_idx == -4:
                return 'In4.Cu'
            elif layer_idx == -5:
                return 'In5.Cu'
            elif layer_idx == -6:
                return 'In6.Cu'
            elif layer_idx == -7:
                return 'In7.Cu'
            else:
                return layer_names[layer_idx]

        # Add connecting segment from original start if needed
        if original_start and len(float_path) > 0:
            first_x, first_y, first_layer = float_path[0]
            orig_x, orig_y = original_start
            if abs(orig_x - first_x) > 0.001 or abs(orig_y - first_y) > 0.001:
                seg_layer = DEBUG_CONNECTOR_LAYER if DEBUG_LAYERS else get_layer_name(first_layer)
                segs.append(Segment(
                    start_x=orig_x, start_y=orig_y,
                    end_x=first_x, end_y=first_y,
                    width=config.track_width,
                    layer=seg_layer,
                    net_id=net_id
                ))

        # Convert path
        for i in range(len(float_path) - 1):
            x1, y1, layer1 = float_path[i]
            x2, y2, layer2 = float_path[i + 1]

            # Check if this is a real layer change (not debug layer transition)
            real_layer1 = layer1 if layer1 >= 0 else 1  # Map debug layers to In1.Cu for via logic
            real_layer2 = layer2 if layer2 >= 0 else 1

            if real_layer1 != real_layer2:
                vias.append(Via(
                    x=x1, y=y1,
                    size=config.via_size,
                    drill=config.via_drill,
                    layers=[get_layer_name(layer1), get_layer_name(layer2)],
                    net_id=net_id
                ))
            elif abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                seg_layer = DEBUG_CENTERLINE_LAYER if DEBUG_LAYERS else get_layer_name(layer1)
                segs.append(Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=seg_layer,
                    net_id=net_id
                ))

        # Add connecting segment to original end if needed
        if original_end and len(float_path) > 0:
            last_x, last_y, last_layer = float_path[-1]
            orig_x, orig_y = original_end
            if abs(orig_x - last_x) > 0.001 or abs(orig_y - last_y) > 0.001:
                seg_layer = DEBUG_CONNECTOR_LAYER if DEBUG_LAYERS else layer_names[last_layer]
                segs.append(Segment(
                    start_x=last_x, start_y=last_y,
                    end_x=orig_x, end_y=orig_y,
                    width=config.track_width,
                    layer=seg_layer,
                    net_id=net_id
                ))

        return segs, vias

    # Get original coordinates for P and N nets
    p_start = (sources[0][5], sources[0][6]) if sources else None
    p_end = (targets[0][5], targets[0][6]) if targets else None
    n_start = (sources[0][7], sources[0][8]) if sources else None
    n_end = (targets[0][7], targets[0][8]) if targets else None

    # Convert P path
    p_segs, p_vias = float_path_to_geometry(p_float_path, p_net_id, p_start, p_end)
    new_segments.extend(p_segs)
    new_vias.extend(p_vias)

    # Convert N path
    n_segs, n_vias = float_path_to_geometry(n_float_path, n_net_id, n_start, n_end)
    new_segments.extend(n_segs)
    new_vias.extend(n_vias)

    # Convert float paths back to grid format for return value
    p_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in p_float_path]
    n_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in n_float_path]

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(simplified_path),
        'p_path': p_path,
        'n_path': n_path,
        'raw_astar_path': raw_astar_path,
        'simplified_path': simplified_path_float,
    }


def batch_route(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None,
                direction_order: str = None,
                ordering_strategy: str = "inside_out",
                disable_bga_zones: bool = False,
                track_width: float = 0.1,
                clearance: float = 0.1,
                via_size: float = 0.3,
                via_drill: float = 0.2,
                grid_step: float = 0.1,
                via_cost: int = 25,
                max_iterations: int = 100000,
                heuristic_weight: float = 1.5,
                stub_proximity_radius: float = 1.0,
                stub_proximity_cost: float = 3.0,
                diff_pair_patterns: Optional[List[str]] = None,
                diff_pair_gap: float = 0.1,
                min_diff_pair_centerline_setback: float = 0.4,
                max_diff_pair_centerline_setback: float = 0.4,
                debug_layers: bool = False) -> Tuple[int, int, float]:
    """
    Route multiple nets using the Rust router.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route
        layers: List of copper layers to route on (must be specified - cannot auto-detect
                which layers are ground planes vs signal layers)
        bga_exclusion_zones: Optional list of BGA exclusion zones (auto-detected if None)
        direction_order: Direction search order - "forward", "backwards", or "random"
                        (None = use GridRouteConfig default)
        ordering_strategy: Net ordering strategy:
            - "inside_out": Sort BGA nets by distance from BGA center (default)
            - "mps": Use Maximum Planar Subset algorithm to minimize crossing conflicts
            - "original": Keep nets in original order
        track_width: Track width in mm (default: 0.1)
        clearance: Clearance between tracks in mm (default: 0.1)
        via_size: Via outer diameter in mm (default: 0.3)
        via_drill: Via drill size in mm (default: 0.2)
        grid_step: Grid resolution in mm (default: 0.1)
        via_cost: Penalty for placing a via in grid steps (default: 25)
        max_iterations: Max A* iterations before giving up (default: 100000)
        heuristic_weight: A* heuristic weight, higher=faster but less optimal (default: 1.5)
        stub_proximity_radius: Radius around stubs to penalize in mm (default: 1.0)
        stub_proximity_cost: Cost penalty near stubs in mm equivalent (default: 3.0)
        diff_pair_patterns: Glob patterns for nets to route as differential pairs
        diff_pair_gap: Gap between P and N traces in differential pairs (default: 0.1mm)
        debug_layers: Output raw A* path on User.9, simplified path on User.8

    Returns:
        (successful_count, failed_count, total_time)
    """
    print(f"Loading {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    # Layers must be specified - we can't auto-detect which are ground planes
    if layers is None:
        layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']  # Default 4-layer signal stack
    print(f"Using {len(layers)} routing layers: {layers}")

    # Auto-detect BGA exclusion zones if not specified
    if disable_bga_zones:
        bga_exclusion_zones = []
        print("BGA exclusion zones disabled")
    elif bga_exclusion_zones is None:
        bga_exclusion_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.5)
        if bga_exclusion_zones:
            bga_components = find_components_by_type(pcb_data, 'BGA')
            print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s):")
            for i, (fp, zone) in enumerate(zip(bga_components, bga_exclusion_zones)):
                print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f})")
        else:
            print("No BGA components detected - no exclusion zones needed")

    config_kwargs = dict(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        via_cost=via_cost,
        layers=layers,
        max_iterations=max_iterations,
        heuristic_weight=heuristic_weight,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius,
        stub_proximity_cost=stub_proximity_cost,
        diff_pair_gap=diff_pair_gap,
        min_diff_pair_centerline_setback=min_diff_pair_centerline_setback,
        max_diff_pair_centerline_setback=max_diff_pair_centerline_setback,
        debug_layers=debug_layers,
    )
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    config = GridRouteConfig(**config_kwargs)

    # Find differential pairs if patterns provided
    diff_pairs: Dict[str, DiffPair] = {}
    diff_pair_net_ids = set()  # Net IDs that are part of differential pairs
    if diff_pair_patterns:
        diff_pairs = find_differential_pairs(pcb_data, diff_pair_patterns)
        if diff_pairs:
            print(f"\nFound {len(diff_pairs)} differential pairs:")
            for pair_name, pair in list(diff_pairs.items())[:5]:
                print(f"  {pair_name}: {pair.p_net_name} / {pair.n_net_name}")
            if len(diff_pairs) > 5:
                print(f"  ... and {len(diff_pairs) - 5} more")
            # Track which net IDs are part of pairs
            for pair in diff_pairs.values():
                diff_pair_net_ids.add(pair.p_net_id)
                diff_pair_net_ids.add(pair.n_net_id)
        else:
            print(f"\nNo differential pairs found matching patterns: {diff_pair_patterns}")

    # Find net IDs - check both pcb.nets and pads_by_net
    net_ids = []
    for net_name in net_names:
        net_id = None
        # First check pcb.nets
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        # If not found, check pads_by_net (for nets not in pcb.nets)
        if net_id is None:
            for nid, pads in pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.net_name == net_name:
                        net_id = nid
                        break
                if net_id is not None:
                    break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_ids.append((net_name, net_id))

    if not net_ids:
        print("No valid nets to route!")
        return 0, 0, 0.0

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        # Use Maximum Planar Subset algorithm to minimize crossing conflicts
        print(f"\nUsing MPS ordering strategy...")
        all_net_ids = [nid for _, nid in net_ids]
        ordered_ids = compute_mps_net_ordering(pcb_data, all_net_ids)
        # Rebuild net_ids in the new order
        id_to_name = {nid: name for name, nid in net_ids}
        net_ids = [(id_to_name[nid], nid) for nid in ordered_ids if nid in id_to_name]

    elif ordering_strategy == "inside_out" and bga_exclusion_zones:
        # Sort nets inside-out from BGA center(s) for better escape routing
        # Only applies to nets that have pads inside a BGA zone
        def pad_in_bga_zone(pad):
            """Check if a pad is inside any BGA zone."""
            for zone in bga_exclusion_zones:
                if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                    return True
            return False

        def get_min_distance_to_bga_center(net_id):
            """Get minimum distance from any BGA pad of this net to its BGA center."""
            pads = pcb_data.pads_by_net.get(net_id, [])
            if not pads:
                return float('inf')

            min_dist = float('inf')
            for zone in bga_exclusion_zones:
                center_x = (zone[0] + zone[2]) / 2
                center_y = (zone[1] + zone[3]) / 2
                for pad in pads:
                    # Only consider pads that are inside this BGA zone
                    if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                        dist = ((pad.global_x - center_x) ** 2 + (pad.global_y - center_y) ** 2) ** 0.5
                        min_dist = min(min_dist, dist)
            return min_dist

        # Separate BGA nets from non-BGA nets
        bga_nets = []
        non_bga_nets = []
        for net_name, net_id in net_ids:
            pads = pcb_data.pads_by_net.get(net_id, [])
            has_bga_pad = any(pad_in_bga_zone(pad) for pad in pads)
            if has_bga_pad:
                bga_nets.append((net_name, net_id))
            else:
                non_bga_nets.append((net_name, net_id))

        # Sort BGA nets inside-out, keep non-BGA nets in original order
        bga_nets.sort(key=lambda x: get_min_distance_to_bga_center(x[1]))
        net_ids = bga_nets + non_bga_nets

        if bga_nets:
            print(f"\nSorted {len(bga_nets)} BGA nets inside-out ({len(non_bga_nets)} non-BGA nets unchanged)")

    elif ordering_strategy == "original":
        print(f"\nUsing original net order (no sorting)")

    # Separate single-ended nets from differential pairs
    single_ended_nets = []
    diff_pair_ids_to_route = []  # (pair_name, pair) tuples
    processed_pair_net_ids = set()

    for net_name, net_id in net_ids:
        if net_id in diff_pair_net_ids and net_id not in processed_pair_net_ids:
            # Find the differential pair this net belongs to
            for pair_name, pair in diff_pairs.items():
                if pair.p_net_id == net_id or pair.n_net_id == net_id:
                    diff_pair_ids_to_route.append((pair_name, pair))
                    processed_pair_net_ids.add(pair.p_net_id)
                    processed_pair_net_ids.add(pair.n_net_id)
                    break
        elif net_id not in diff_pair_net_ids:
            single_ended_nets.append((net_name, net_id))

    total_routes = len(single_ended_nets) + len(diff_pair_ids_to_route)
    print(f"\nRouting {total_routes} items ({len(single_ended_nets)} single-ended nets, {len(diff_pair_ids_to_route)} differential pairs)...")
    print("=" * 60)

    results = []
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0

    # Build base obstacle map once (excludes all nets we're routing)
    all_net_ids_to_route = [nid for _, nid in net_ids]
    print("Building base obstacle map...")
    base_start = time.time()
    base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route)
    base_elapsed = time.time() - base_start
    print(f"Base obstacle map built in {base_elapsed:.2f}s")

    # Build separate base obstacle map with extra clearance for diff pair centerline routing
    # Extra clearance = spacing from centerline to each track (at least one track width)
    diff_pair_extra_clearance = (config.track_width + config.diff_pair_gap) / 2 + config.track_width
    print(f"Building diff pair obstacle map (extra clearance: {diff_pair_extra_clearance:.3f}mm)...")
    dp_base_start = time.time()
    diff_pair_base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route, diff_pair_extra_clearance)
    dp_base_elapsed = time.time() - dp_base_start
    print(f"Diff pair obstacle map built in {dp_base_elapsed:.2f}s")

    # Track which nets have been routed (their segments/vias are now in pcb_data)
    routed_net_ids = []
    remaining_net_ids = list(all_net_ids_to_route)

    # Get ALL unrouted nets in the PCB for stub proximity costs
    # (not just the ones we're routing in this batch)
    all_unrouted_net_ids = set(get_all_unrouted_net_ids(pcb_data))
    print(f"Found {len(all_unrouted_net_ids)} unrouted nets in PCB for stub proximity")

    route_index = 0

    # Route differential pairs first (they're more constrained)
    for pair_name, pair in diff_pair_ids_to_route:
        route_index += 1
        print(f"\n[{route_index}/{total_routes}] Routing diff pair {pair_name}")
        print(f"  P: {pair.p_net_name} (id={pair.p_net_id})")
        print(f"  N: {pair.n_net_name} (id={pair.n_net_id})")
        print("-" * 40)

        start_time = time.time()

        # Clone diff pair base obstacles (with extra clearance for centerline routing)
        obstacles = diff_pair_base_obstacles.clone()

        # Add previously routed nets' segments/vias/pads as obstacles (with extra clearance)
        for routed_id in routed_net_ids:
            add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (excluding both P and N)
        other_unrouted = [nid for nid in remaining_net_ids
                         if nid != pair.p_net_id and nid != pair.n_net_id]
        for other_net_id in other_unrouted:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)

        # Add stub proximity costs for ALL unrouted nets in PCB (not just current batch)
        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                   if nid != pair.p_net_id and nid != pair.n_net_id
                                   and nid not in routed_net_ids]
        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
        if unrouted_stubs:
            add_stub_proximity_costs(obstacles, unrouted_stubs, config)

        # Add same-net via clearance for both P and N
        add_same_net_via_clearance(obstacles, pcb_data, pair.p_net_id, config)
        add_same_net_via_clearance(obstacles, pcb_data, pair.n_net_id, config)

        # Route the differential pair
        result = route_diff_pair_with_obstacles(pcb_data, pair, config, obstacles)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']
            add_route_to_pcb_data(pcb_data, result)
            if pair.p_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.p_net_id)
            if pair.n_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.n_net_id)
            routed_net_ids.append(pair.p_net_id)
            routed_net_ids.append(pair.n_net_id)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            failed += 1
            total_iterations += iterations

    # Route single-ended nets
    for net_name, net_id in single_ended_nets:
        route_index += 1
        print(f"\n[{route_index}/{total_routes}] Routing {net_name} (id={net_id})")
        print("-" * 40)

        start_time = time.time()

        # Clone base obstacles
        obstacles = base_obstacles.clone()

        # Add previously routed nets' segments/vias/pads as obstacles (from pcb_data)
        for routed_id in routed_net_ids:
            add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
            add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config)
            add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (not the current net)
        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
        for other_net_id in other_unrouted:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config)

        # Add stub proximity costs for ALL unrouted nets in PCB (not just current batch)
        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                   if nid != net_id and nid not in routed_net_ids]
        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
        if unrouted_stubs:
            add_stub_proximity_costs(obstacles, unrouted_stubs, config)

        # Add same-net via clearance blocking (for DRC - vias can't be too close even on same net)
        add_same_net_via_clearance(obstacles, pcb_data, net_id, config)

        # Route the net using the prepared obstacles
        result = route_net_with_obstacles(pcb_data, net_id, config, obstacles)
        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']
            add_route_to_pcb_data(pcb_data, result)
            remaining_net_ids.remove(net_id)
            routed_net_ids.append(net_id)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            failed += 1
            total_iterations += iterations

    print("\n" + "=" * 60)
    print(f"Routing complete: {successful} successful, {failed} failed")
    print(f"Total time: {total_time:.2f}s")
    print(f"Total iterations: {total_iterations}")

    if results:
        print(f"\nWriting output to {output_file}...")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()

        routing_text = ""
        for result in results:
            for seg in result['new_segments']:
                routing_text += generate_segment_sexpr(
                    (seg.start_x, seg.start_y), (seg.end_x, seg.end_y),
                    seg.width, seg.layer, seg.net_id
                ) + "\n"
            for via in result['new_vias']:
                routing_text += generate_via_sexpr(
                    via.x, via.y, via.size, via.drill,
                    via.layers, via.net_id
                ) + "\n"

        # Add debug paths if enabled (using gr_line for User layers)
        if debug_layers:
            print("Adding debug paths to User.8 (simplified) and User.9 (raw A*)")
            for result in results:
                # Raw A* path on User.9
                raw_path = result.get('raw_astar_path', [])
                if len(raw_path) >= 2:
                    for i in range(len(raw_path) - 1):
                        x1, y1, _ = raw_path[i]
                        x2, y2, _ = raw_path[i + 1]
                        if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                            routing_text += generate_gr_line_sexpr(
                                (x1, y1), (x2, y2),
                                0.05, "User.9"  # Thin line
                            ) + "\n"

                # Simplified path on User.8
                simplified_path = result.get('simplified_path', [])
                if len(simplified_path) >= 2:
                    for i in range(len(simplified_path) - 1):
                        x1, y1, _ = simplified_path[i]
                        x2, y2, _ = simplified_path[i + 1]
                        if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                            routing_text += generate_gr_line_sexpr(
                                (x1, y1), (x2, y2),
                                0.15, "User.8"  # Thicker line
                            ) + "\n"

        last_paren = content.rfind(')')
        new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(new_content)

        print(f"Successfully wrote {output_file}")

    return successful, failed, total_time


def expand_net_patterns(pcb_data: PCBData, patterns: List[str]) -> List[str]:
    """
    Expand wildcard patterns to matching net names.

    Patterns can include * and ? wildcards (fnmatch style).
    Example: "Net-(U2A-DATA_*)" matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.

    Returns list of unique net names in sorted order for patterns,
    preserving order of non-pattern names.
    """
    # Collect net names from both pcb.nets and pads_by_net
    all_net_names = set(net.name for net in pcb_data.nets.values())
    # Also include net names from pads (for nets not in pcb.nets)
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.net_name:
                all_net_names.add(pad.net_name)
                break  # Only need one pad's net_name per net
    all_net_names = list(all_net_names)
    result = []
    seen = set()

    for pattern in patterns:
        if '*' in pattern or '?' in pattern:
            # It's a wildcard pattern - find all matching nets
            matches = sorted([name for name in all_net_names if fnmatch.fnmatch(name, pattern)])
            if not matches:
                print(f"Warning: Pattern '{pattern}' matched no nets")
            else:
                print(f"Pattern '{pattern}' matched {len(matches)} nets")
            for name in matches:
                if name not in seen:
                    result.append(name)
                    seen.add(name)
        else:
            # Literal net name
            if pattern not in seen:
                result.append(pattern)
                seen.add(pattern)

    return result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Batch PCB Router - Routes multiple nets using Rust-accelerated A*",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Wildcard patterns supported:
  "Net-(U2A-DATA_*)"  - matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.
  "Net-(*CLK*)"       - matches any net containing CLK

Examples:
  python batch_grid_router.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_*)"
  python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering mps

Differential pair routing:
  python batch_grid_router.py input.kicad_pcb output.kicad_pcb "*lvds*" --diff-pairs "*lvds*"

  The --diff-pairs option specifies patterns for nets that should be routed as differential pairs.
  Nets matching these patterns with _P/_N, P/N, or +/- suffixes will be routed together
  maintaining constant spacing (controlled by --diff-pair-gap).
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", help="Output KiCad PCB file")
    parser.add_argument("net_patterns", nargs="+", help="Net names or wildcard patterns to route")
    # Ordering and strategy options
    parser.add_argument("--ordering", "-o", choices=["inside_out", "mps", "original"],
                        default="mps",
                        help="Net ordering strategy: mps (default, crossing conflicts), inside_out, or original")
    parser.add_argument("--direction", "-d", choices=["forward", "backwards", "random"],
                        default=None,
                        help="Direction search order for each net route")
    parser.add_argument("--no-bga-zones", action="store_true",
                        help="Disable BGA exclusion zone detection (allows routing through BGA areas)")
    parser.add_argument("--layers", "-l", nargs="+",
                        default=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                        help="Routing layers to use (default: F.Cu In1.Cu In2.Cu B.Cu)")

    # Track and via geometry
    parser.add_argument("--track-width", type=float, default=0.1,
                        help="Track width in mm (default: 0.1)")
    parser.add_argument("--clearance", type=float, default=0.1,
                        help="Clearance between tracks in mm (default: 0.1)")
    parser.add_argument("--via-size", type=float, default=0.3,
                        help="Via outer diameter in mm (default: 0.3)")
    parser.add_argument("--via-drill", type=float, default=0.2,
                        help="Via drill size in mm (default: 0.2)")

    # Router algorithm parameters
    parser.add_argument("--grid-step", type=float, default=0.1,
                        help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--via-cost", type=int, default=25,
                        help="Penalty for placing a via in grid steps (default: 25)")
    parser.add_argument("--max-iterations", type=int, default=100000,
                        help="Max A* iterations before giving up (default: 100000)")
    parser.add_argument("--heuristic-weight", type=float, default=1.5,
                        help="A* heuristic weight, higher=faster but less optimal (default: 1.5)")

    # Stub proximity penalty
    parser.add_argument("--stub-proximity-radius", type=float, default=1.5,
                        help="Radius around stubs to penalize routing in mm (default: 1.5)")
    parser.add_argument("--stub-proximity-cost", type=float, default=2.0,
                        help="Cost penalty near stubs in mm equivalent (default: 2.0)")

    # Differential pair routing
    parser.add_argument("--diff-pairs", "-D", nargs="+",
                        help="Glob patterns for nets to route as differential pairs (e.g., '*lvds*')")
    parser.add_argument("--diff-pair-gap", type=float, default=0.1,
                        help="Gap between P and N traces of differential pairs in mm (default: 0.1)")
    parser.add_argument("--min-diff-pair-centerline-setback", type=float, default=0.4,
                        help="Minimum distance in front of stubs to start centerline route in mm (default: 0.4)")
    parser.add_argument("--max-diff-pair-centerline-setback", type=float, default=0.4,
                        help="Maximum distance in front of stubs to start centerline route in mm (default: 0.4)")

    # Debug options
    parser.add_argument("--debug-layers", action="store_true",
                        help="Output raw A* path on User.9, simplified path on User.8 for debugging")

    args = parser.parse_args()

    # Load PCB to expand wildcards
    print(f"Loading {args.input_file} to expand net patterns...")
    pcb_data = parse_kicad_pcb(args.input_file)
    net_names = expand_net_patterns(pcb_data, args.net_patterns)

    if not net_names:
        print("No nets matched the given patterns!")
        sys.exit(1)

    print(f"Routing {len(net_names)} nets: {net_names[:5]}{'...' if len(net_names) > 5 else ''}")

    batch_route(args.input_file, args.output_file, net_names,
                direction_order=args.direction,
                ordering_strategy=args.ordering,
                disable_bga_zones=args.no_bga_zones,
                layers=args.layers,
                track_width=args.track_width,
                clearance=args.clearance,
                via_size=args.via_size,
                via_drill=args.via_drill,
                grid_step=args.grid_step,
                via_cost=args.via_cost,
                max_iterations=args.max_iterations,
                heuristic_weight=args.heuristic_weight,
                stub_proximity_radius=args.stub_proximity_radius,
                stub_proximity_cost=args.stub_proximity_cost,
                diff_pair_patterns=args.diff_pairs,
                diff_pair_gap=args.diff_pair_gap,
                min_diff_pair_centerline_setback=args.min_diff_pair_centerline_setback,
                max_diff_pair_centerline_setback=args.max_diff_pair_centerline_setback,
                debug_layers=args.debug_layers)
