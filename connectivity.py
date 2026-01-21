"""
Connectivity analysis utilities for PCB routing.

Functions for finding endpoints, analyzing stubs, computing MST paths,
and tracking segment connectivity.
"""

import math
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import PCBData, Segment, Via, Pad, Zone
from routing_config import GridRouteConfig, GridCoord
from routing_utils import pos_key, segment_length, POSITION_DECIMALS, build_layer_map
from geometry_utils import UnionFind
from routing_constants import BGA_DEFAULT_EDGE_TOLERANCE, BGA_EDGE_DETECTION_TOLERANCE


def _get_pad_coords(p) -> Tuple[float, float]:
    """Get x, y coordinates from Pad object or dict."""
    if hasattr(p, 'global_x'):
        return p.global_x, p.global_y
    elif isinstance(p, dict):
        return p['x'], p['y']
    else:
        raise ValueError(f"Unknown pad type: {type(p)}")


def find_farthest_pad_pair(pads) -> Tuple[int, int]:
    """
    Find the two farthest pads/endpoints by Manhattan distance.

    Manhattan distance is used because PCB routing follows horizontal/vertical
    paths, making it a better estimate of actual route length than Euclidean.

    Args:
        pads: List of Pad objects or dicts with 'x'/'y' keys (must have at least 2)

    Returns:
        (idx_a, idx_b): Indices of the two farthest pads
    """
    if len(pads) < 2:
        raise ValueError("Need at least 2 pads to find farthest pair")

    max_dist = -1
    best_pair = (0, 1)

    for i in range(len(pads)):
        for j in range(i + 1, len(pads)):
            x1, y1 = _get_pad_coords(pads[i])
            x2, y2 = _get_pad_coords(pads[j])
            dist = abs(x1 - x2) + abs(y1 - y2)
            if dist > max_dist:
                max_dist = dist
                best_pair = (i, j)

    return best_pair


def find_closest_point_on_segments(
    segments: List[Segment],
    target_x: float,
    target_y: float,
    target_layers: List[str]
) -> Tuple[float, float, str, float]:
    """
    Find the closest point on any segment to a target location.

    For multi-point net routing, this finds where to tap into an existing
    track to reach an additional pad.

    Args:
        segments: List of routed segments to search
        target_x, target_y: Target location coordinates
        target_layers: Preferred layers for the target (from pad.layers)

    Returns:
        (x, y, layer, distance): Closest point coordinates, its layer, and distance
        Returns None if no segments provided
    """
    if not segments:
        return None

    best_point = None
    best_dist = float('inf')
    best_layer = None

    for seg in segments:
        # Project target point onto segment line
        sx, sy = seg.start_x, seg.start_y
        ex, ey = seg.end_x, seg.end_y

        # Vector from start to end
        dx = ex - sx
        dy = ey - sy
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq < 1e-10:
            # Degenerate segment (point)
            px, py = sx, sy
        else:
            # Parameter t for projection onto line
            t = ((target_x - sx) * dx + (target_y - sy) * dy) / seg_len_sq
            # Clamp to segment bounds
            t = max(0.0, min(1.0, t))
            px = sx + t * dx
            py = sy + t * dy

        # Distance from projected point to target
        dist = math.sqrt((px - target_x) ** 2 + (py - target_y) ** 2)

        # Prefer same-layer connections (reduce via cost implicitly)
        layer_bonus = 0.0 if seg.layer in target_layers else 0.5

        effective_dist = dist + layer_bonus

        if effective_dist < best_dist:
            best_dist = effective_dist
            best_point = (px, py)
            best_layer = seg.layer

    return (best_point[0], best_point[1], best_layer, best_dist) if best_point else None


def calculate_stub_length(pcb_data, net_id: int) -> float:
    """
    Calculate the total length of existing stub segments for a net.

    Stubs are the pre-existing track segments that connect from pads to the
    routing endpoints. For accurate length matching, we need to include these
    in the total pad-to-pad length.

    Args:
        pcb_data: PCB data containing existing segments
        net_id: Net ID to calculate stub length for

    Returns:
        Total stub length in mm
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    total = 0.0
    for seg in net_segments:
        total += segment_length(seg)
    return total


def is_edge_stub(pad_x: float, pad_y: float, bga_zones: List) -> bool:
    """Check if a pad is on the outer row/column of any BGA zone.

    Args:
        pad_x, pad_y: Pad position to check (not stub endpoint)
        bga_zones: List of BGA zones, either:
                   - 4-tuple: (min_x, min_y, max_x, max_y) - uses default tolerance
                   - 5-tuple: (min_x, min_y, max_x, max_y, edge_tolerance)

    Returns:
        True if pad is inside a BGA zone AND in the outer row/column
    """
    for zone in bga_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        # Use per-zone tolerance if available, else default (0.5mm margin + 1mm pitch * 1.1)
        edge_tolerance = zone[4] if len(zone) > 4 else BGA_DEFAULT_EDGE_TOLERANCE
        # Check if pad is INSIDE this zone (with small tolerance for floating point)
        inside_tolerance = BGA_EDGE_DETECTION_TOLERANCE
        if (pad_x >= min_x - inside_tolerance and pad_x <= max_x + inside_tolerance and
            pad_y >= min_y - inside_tolerance and pad_y <= max_y + inside_tolerance):
            # Check if it's on an edge (within tolerance of min/max)
            on_left = abs(pad_x - min_x) <= edge_tolerance
            on_right = abs(pad_x - max_x) <= edge_tolerance
            on_bottom = abs(pad_y - min_y) <= edge_tolerance
            on_top = abs(pad_y - max_y) <= edge_tolerance
            if on_left or on_right or on_bottom or on_top:
                return True
    return False


def find_connected_groups(segments: List[Segment], tolerance: float = 0.01) -> List[List[Segment]]:
    """Find groups of connected segments using union-find with spatial hashing.

    Uses O(n) spatial hashing instead of O(n²) pairwise comparison.
    """
    if not segments:
        return []

    n = len(segments)
    uf = UnionFind()

    # Spatial hash: map rounded coordinates to (segment_index, x, y) tuples
    # Use tolerance-based grid cells
    inv_tol = 1.0 / tolerance
    endpoint_map: Dict[Tuple[int, int], List[Tuple[int, float, float]]] = {}

    # First pass: build spatial hash with actual coordinates
    for i, seg in enumerate(segments):
        for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
            key = (int(x * inv_tol), int(y * inv_tol))
            if key not in endpoint_map:
                endpoint_map[key] = []
            endpoint_map[key].append((i, x, y))

    # Second pass: check each endpoint against neighbors (handles bucket boundary issues)
    for i, seg in enumerate(segments):
        for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
            gx, gy = int(x * inv_tol), int(y * inv_tol)
            # Check current cell and 8 neighbors
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    key = (gx + dx, gy + dy)
                    if key in endpoint_map:
                        for j, ox, oy in endpoint_map[key]:
                            if i < j:  # Avoid duplicate checks
                                if abs(x - ox) < tolerance and abs(y - oy) < tolerance:
                                    uf.union(i, j)

    # Group segments by their root
    groups: Dict[int, List[Segment]] = {}
    for i in range(n):
        root = uf.find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


def _point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    """Check if a point (x, y) is inside a polygon using ray casting algorithm."""
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def is_net_connected_via_zones(
    segments: List[Segment],
    vias: List[Via],
    pads: List[Pad],
    zones: List[Zone],
    routing_layers: List[str] = None
) -> bool:
    """
    Check if all pads of a net are connected through zones/planes.

    Uses union-find to track connectivity through:
    - Segments connecting points on same layer
    - Vias connecting points across layers
    - Zones connecting all points inside them on the zone's layer
    - Through-hole pads connecting across all layers

    Args:
        segments: Track segments for this net
        vias: Vias for this net
        pads: Pads for this net (must have at least 2)
        zones: Zones/planes for this net
        routing_layers: List of routing layers (for expanding *.Cu pads)

    Returns:
        True if all pads are connected, False otherwise
    """
    if len(pads) < 2:
        return True  # 0 or 1 pad is trivially connected

    if not zones:
        return False  # No zones, can't be connected via zones

    # Import expand_pad_layers locally to avoid circular import
    from net_queries import expand_pad_layers

    uf = UnionFind()

    # Detect all copper layers
    copper_layers = set()
    for seg in segments:
        if seg.layer.endswith('.Cu'):
            copper_layers.add(seg.layer)
    for via in vias:
        if via.layers:
            for layer in via.layers:
                if layer.endswith('.Cu'):
                    copper_layers.add(layer)
    for zone in zones:
        if zone.layer.endswith('.Cu'):
            copper_layers.add(zone.layer)
    if routing_layers:
        copper_layers.update(routing_layers)
    if not copper_layers:
        copper_layers = {'F.Cu', 'B.Cu'}

    all_copper_layers = sorted(copper_layers)

    # Collect all points: (x, y, layer, point_id)
    all_points = []
    point_id = 0

    # Add segment endpoints
    for seg in segments:
        start_id = point_id
        all_points.append((seg.start_x, seg.start_y, seg.layer, start_id))
        point_id += 1
        end_id = point_id
        all_points.append((seg.end_x, seg.end_y, seg.layer, end_id))
        point_id += 1
        uf.union(start_id, end_id)  # Connect segment's endpoints

    # Add vias (connect all layers at one location)
    for via in vias:
        via_layers = all_copper_layers if (via.layers and 'F.Cu' in via.layers and 'B.Cu' in via.layers) else (via.layers or all_copper_layers)
        via_ids = []
        for layer in via_layers:
            all_points.append((via.x, via.y, layer, point_id))
            via_ids.append(point_id)
            point_id += 1
        for vid in via_ids[1:]:
            uf.union(via_ids[0], vid)

    # Add pads (through-hole pads connect across layers)
    pad_ids = []
    for pad in pads:
        expanded_layers = expand_pad_layers(pad.layers, all_copper_layers)
        this_pad_ids = []
        for layer in expanded_layers:
            if layer in copper_layers:
                all_points.append((pad.global_x, pad.global_y, layer, point_id))
                this_pad_ids.append(point_id)
                pad_ids.append(point_id)
                point_id += 1
        for pid in this_pad_ids[1:]:
            uf.union(this_pad_ids[0], pid)

    # Connect points through zones
    for zone in zones:
        zone_layer = zone.layer
        points_on_layer = [(x, y, layer, pid) for x, y, layer, pid in all_points if layer == zone_layer]
        points_in_zone = [pid for x, y, layer, pid in points_on_layer if _point_in_polygon(x, y, zone.polygon)]
        if len(points_in_zone) > 1:
            for pid in points_in_zone[1:]:
                uf.union(points_in_zone[0], pid)

    # Connect nearby points on same layer (tolerance-based)
    tolerance = 0.02
    for i, (x1, y1, l1, id1) in enumerate(all_points):
        for j, (x2, y2, l2, id2) in enumerate(all_points):
            if i < j and l1 == l2:
                if abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance:
                    uf.union(id1, id2)

    # Check if all pads are in the same component
    if not pad_ids:
        return True
    first_root = uf.find(pad_ids[0])
    return all(uf.find(pid) == first_root for pid in pad_ids)


def get_zone_connected_pad_groups(
    segments: List[Segment],
    vias: List[Via],
    pads: List[Pad],
    zones: List[Zone],
    routing_layers: List[str] = None
) -> Dict[int, int]:
    """
    Get connected component membership for each pad based on zone/plane connectivity.

    Returns a dict mapping pad index to component ID. Pads with the same component ID
    are already connected through zones/tracks/vias and don't need MST edges between them.

    Args:
        segments: Track segments for this net
        vias: Vias for this net
        pads: Pads for this net
        zones: Zones/planes for this net
        routing_layers: List of routing layers (for expanding *.Cu pads)

    Returns:
        Dict mapping pad index (0-based) to component ID
    """
    if len(pads) < 2:
        return {i: 0 for i in range(len(pads))}

    from net_queries import expand_pad_layers

    uf = UnionFind()

    # Detect all copper layers
    copper_layers = set()
    for seg in segments:
        if seg.layer.endswith('.Cu'):
            copper_layers.add(seg.layer)
    for via in vias:
        if via.layers:
            for layer in via.layers:
                if layer.endswith('.Cu'):
                    copper_layers.add(layer)
    for zone in zones:
        if zone.layer.endswith('.Cu'):
            copper_layers.add(zone.layer)
    if routing_layers:
        copper_layers.update(routing_layers)
    if not copper_layers:
        copper_layers = {'F.Cu', 'B.Cu'}

    all_copper_layers = sorted(copper_layers)

    # Collect all points with pad index tracking
    all_points = []  # (x, y, layer, point_id)
    point_id = 0
    pad_point_ids: Dict[int, List[int]] = {}  # pad_index -> list of point_ids

    # Add segment endpoints
    for seg in segments:
        start_id = point_id
        all_points.append((seg.start_x, seg.start_y, seg.layer, start_id))
        point_id += 1
        end_id = point_id
        all_points.append((seg.end_x, seg.end_y, seg.layer, end_id))
        point_id += 1
        uf.union(start_id, end_id)

    # Add vias
    for via in vias:
        via_layers = all_copper_layers if (via.layers and 'F.Cu' in via.layers and 'B.Cu' in via.layers) else (via.layers or all_copper_layers)
        via_ids = []
        for layer in via_layers:
            all_points.append((via.x, via.y, layer, point_id))
            via_ids.append(point_id)
            point_id += 1
        for vid in via_ids[1:]:
            uf.union(via_ids[0], vid)

    # Add pads with index tracking
    for pad_idx, pad in enumerate(pads):
        expanded_layers = expand_pad_layers(pad.layers, all_copper_layers)
        this_pad_ids = []
        for layer in expanded_layers:
            if layer in copper_layers:
                all_points.append((pad.global_x, pad.global_y, layer, point_id))
                this_pad_ids.append(point_id)
                point_id += 1
        for pid in this_pad_ids[1:]:
            uf.union(this_pad_ids[0], pid)
        pad_point_ids[pad_idx] = this_pad_ids

    # Connect points through zones
    for zone in zones:
        zone_layer = zone.layer
        points_on_layer = [(x, y, layer, pid) for x, y, layer, pid in all_points if layer == zone_layer]
        points_in_zone = [pid for x, y, layer, pid in points_on_layer if _point_in_polygon(x, y, zone.polygon)]
        if len(points_in_zone) > 1:
            for pid in points_in_zone[1:]:
                uf.union(points_in_zone[0], pid)

    # Connect nearby points on same layer
    tolerance = 0.02
    for i, (x1, y1, l1, id1) in enumerate(all_points):
        for j, (x2, y2, l2, id2) in enumerate(all_points):
            if i < j and l1 == l2:
                if abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance:
                    uf.union(id1, id2)

    # Map each pad index to its component root
    pad_components = {}
    for pad_idx, point_ids in pad_point_ids.items():
        if point_ids:
            pad_components[pad_idx] = uf.find(point_ids[0])
        else:
            # Pad has no valid layers - give it a unique component
            pad_components[pad_idx] = -pad_idx - 1

    return pad_components


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
        key_start = (round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS), seg.layer)
        key_end = (round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS), seg.layer)
        endpoint_counts[key_start] = endpoint_counts.get(key_start, 0) + 1
        endpoint_counts[key_end] = endpoint_counts.get(key_end, 0) + 1

    # Get pad positions
    pad_positions = [(round(p.global_x, POSITION_DECIMALS), round(p.global_y, POSITION_DECIMALS)) for p in pads]

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


def get_stub_segments(pcb_data: PCBData, net_id: int, stub_x: float, stub_y: float,
                      stub_layer: str, tolerance: float = 0.05) -> List[Segment]:
    """
    Get all segments that form a stub (from free end back to pad).

    Walks backwards from the stub free end through connected segments until
    reaching a pad or the segment chain ends.

    Args:
        pcb_data: PCB data containing segments and pads
        net_id: Net ID of the stub
        stub_x, stub_y: Position of the stub free end
        stub_layer: Layer of the stub
        tolerance: Distance tolerance for matching endpoints

    Returns:
        List of segments forming the stub, ordered from free end to pad
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id and s.layer == stub_layer]
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    pad_positions = [(p.global_x, p.global_y) for p in net_pads]

    result = []
    visited = set()
    current_x, current_y = stub_x, stub_y

    while True:
        # Find segment connected at current position
        found = None
        for seg in net_segments:
            if id(seg) in visited:
                continue

            # Check if start matches current position
            if abs(seg.start_x - current_x) < tolerance and abs(seg.start_y - current_y) < tolerance:
                found = seg
                next_x, next_y = seg.end_x, seg.end_y
                break

            # Check if end matches current position
            if abs(seg.end_x - current_x) < tolerance and abs(seg.end_y - current_y) < tolerance:
                found = seg
                next_x, next_y = seg.start_x, seg.start_y
                break

        if found is None:
            break

        result.append(found)
        visited.add(id(found))

        # Check if next position is at a pad (we've reached the pad end)
        at_pad = False
        for px, py in pad_positions:
            if abs(next_x - px) < tolerance and abs(next_y - py) < tolerance:
                at_pad = True
                break

        if at_pad:
            break

        current_x, current_y = next_x, next_y

    return result


def get_stub_vias(pcb_data: PCBData, net_id: int, stub_segments: List[Segment],
                  tolerance: float = 0.05) -> List:
    """
    Get vias that are part of a stub (e.g., pad vias from layer switching).

    Finds vias that are located at segment endpoints along the stub path.

    Args:
        pcb_data: PCB data containing vias
        net_id: Net ID of the stub
        stub_segments: List of segments forming the stub (from get_stub_segments)
        tolerance: Distance tolerance for matching positions

    Returns:
        List of Via objects that are part of the stub
    """
    if not stub_segments:
        return []

    # Collect all unique positions along the stub segments only
    # Don't add all pad positions - only positions actually on this stub path
    positions = set()
    for seg in stub_segments:
        positions.add((seg.start_x, seg.start_y))
        positions.add((seg.end_x, seg.end_y))

    # Find vias at these positions
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
    stub_vias = []
    for via in net_vias:
        for px, py in positions:
            if abs(via.x - px) < tolerance and abs(via.y - py) < tolerance:
                stub_vias.append(via)
                break

    return stub_vias


def calculate_stub_via_barrel_length(stub_vias: List, stub_layer: str, pcb_data) -> float:
    """
    Calculate via barrel length for stub vias, using the stub layer as one endpoint.

    The barrel length is from the stub's layer to the pad's layer (where the via
    connects), not the via's full span.

    Args:
        stub_vias: List of Via objects in the stub
        stub_layer: Layer the stub segments are on
        pcb_data: PCBData with stackup info and pads

    Returns:
        Total via barrel length in mm
    """
    if not stub_vias or not pcb_data or not hasattr(pcb_data, 'get_via_barrel_length'):
        return 0.0

    total = 0.0
    for via in stub_vias:
        if not via.layers or len(via.layers) < 2:
            continue

        # Find pad at this via position to determine the other layer
        pad_layer = None
        net_pads = pcb_data.pads_by_net.get(via.net_id, [])
        for pad in net_pads:
            if abs(pad.global_x - via.x) < 0.05 and abs(pad.global_y - via.y) < 0.05:
                # Find copper layer from pad's layers (filter out mask/paste layers and wildcards)
                for layer in pad.layers:
                    if layer.endswith('.Cu') and not layer.startswith('*'):
                        pad_layer = layer
                        break
                break

        if pad_layer and pad_layer != stub_layer:
            # Calculate barrel from stub layer to pad layer
            total += pcb_data.get_via_barrel_length(stub_layer, pad_layer)
        elif stub_layer in via.layers:
            # No pad found or same layer - use via's other layer
            other_layer = via.layers[1] if via.layers[0] == stub_layer else via.layers[0]
            total += pcb_data.get_via_barrel_length(stub_layer, other_layer)

    return total


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
    # Import expand_pad_layers locally to avoid circular import
    from net_queries import expand_pad_layers

    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

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
                seg_points.add((round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS)))
                seg_points.add((round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS)))

            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, POSITION_DECIMALS), round(pad.global_y, POSITION_DECIMALS))
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
                    # Expand wildcard layers like "*.Cu" to actual routing layers
                    expanded_layers = expand_pad_layers(pad.layers, config.layers)
                    for layer in expanded_layers:
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
        # Expand wildcard layers like "*.Cu" to actual routing layers
        expanded_layers = expand_pad_layers(pad.layers, config.layers)
        for layer in expanded_layers:
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                sources.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

        targets = []
        for pad in net_pads[1:]:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            # Expand wildcard layers like "*.Cu" to actual routing layers
            expanded_layers = expand_pad_layers(pad.layers, config.layers)
            for layer in expanded_layers:
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


def get_multipoint_net_pads(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig
) -> Optional[List[Tuple]]:
    """
    Check if a net has 3+ unconnected endpoints (multi-point net) and return stub endpoints.

    Multi-point nets need special handling - they can't be routed with a single
    A* path. Instead, they need incremental routing: closest pair first, then
    tap into existing track for remaining pads.

    Handles two cases:
    1. No segments, 3+ pads -> return pad positions
    2. 3+ disconnected segment groups -> return stub endpoints (free ends)

    Args:
        pcb_data: PCB data
        net_id: Net ID to check
        config: Grid routing configuration

    Returns:
        List of endpoint info if multi-point: [(gx, gy, layer_idx, orig_x, orig_y, endpoint_obj), ...]
        None if not a multi-point net
    """
    # Import expand_pad_layers locally to avoid circular import
    from net_queries import expand_pad_layers

    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])

    # Case 1: No segments and 3+ pads
    if len(net_segments) == 0 and len(net_pads) >= 3:
        pad_info = []
        for pad in net_pads:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            # Expand wildcard layers like "*.Cu" to actual routing layers
            expanded_layers = expand_pad_layers(pad.layers, config.layers)
            # Use FIRST layer for MST calculation (one entry per physical pad)
            # The tap routing will create targets on ALL layers for through-hole pads
            for layer in expanded_layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    pad_info.append((gx, gy, layer_idx, pad.global_x, pad.global_y, pad))
                    break  # Only one entry per physical pad for MST
            else:
                # Fallback: if no layers matched, use first routing layer
                if config.layers:
                    pad_info.append((gx, gy, 0, pad.global_x, pad.global_y, pad))
        return pad_info if len(pad_info) >= 3 else None

    # Case 2: Check for 3+ disconnected segment groups
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments)
        if len(groups) >= 3:
            # Find stub free ends for each group (endpoints not touching pads)
            endpoint_info = []
            for group in groups:
                free_ends = find_stub_free_ends(group, net_pads)
                if free_ends:
                    # Use the first free end from each group
                    x, y, layer = free_ends[0]
                    layer_idx = layer_map.get(layer)
                    if layer_idx is not None:
                        # Create a simple object to hold endpoint info
                        endpoint_info.append((
                            coord.to_grid(x, y)[0],
                            coord.to_grid(x, y)[1],
                            layer_idx,
                            x,
                            y,
                            {'x': x, 'y': y, 'layer': layer, 'layers': [layer]}  # dict with layers attr for compatibility
                        ))
            return endpoint_info if len(endpoint_info) >= 3 else None

    return None


def normalize_endpoints_by_component(
    pcb_data: PCBData,
    sources: List,
    targets: List,
    net_id: int
) -> Tuple[List, List]:
    """
    Normalize source/target endpoints so that source is always from the
    alphabetically-first component. This ensures consistent ordering across
    all nets for crossing detection and distance calculations.

    Args:
        pcb_data: PCB data with pad information
        sources: List of source endpoints (gx, gy, layer_idx, orig_x, orig_y)
        targets: List of target endpoints (gx, gy, layer_idx, orig_x, orig_y)
        net_id: Net ID for looking up pads

    Returns:
        (normalized_sources, normalized_targets) - may be swapped if needed
    """
    if not sources or not targets:
        return sources, targets

    net_pads = pcb_data.pads_by_net.get(net_id, [])
    if len(net_pads) < 2:
        return sources, targets

    # Find which component each endpoint is connected to
    def find_component_for_endpoint(endpoint):
        """Find the component_ref for the pad nearest to this endpoint."""
        ex, ey = endpoint[3], endpoint[4]  # orig_x, orig_y
        for pad in net_pads:
            if abs(pad.global_x - ex) < 1.0 and abs(pad.global_y - ey) < 1.0:
                return pad.component_ref
        return None

    src_component = find_component_for_endpoint(sources[0])
    tgt_component = find_component_for_endpoint(targets[0])

    if src_component and tgt_component:
        # Sort alphabetically - first component is source
        if src_component > tgt_component:
            # Swap so alphabetically-first component is source
            return targets, sources

    return sources, targets


def get_stub_endpoints(pcb_data: PCBData, net_ids: List[int]) -> List[Tuple[float, float, str]]:
    """Get free end positions of unrouted net stubs for proximity avoidance.

    Returns list of (x, y, layer) tuples - includes layer for same-layer filtering.
    """
    stubs = []
    for net_id in net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        if len(net_segments) < 2:
            continue
        groups = find_connected_groups(net_segments)
        if len(groups) < 2:
            continue
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        for group in groups:
            free_ends = find_stub_free_ends(group, net_pads)
            if free_ends:
                # Include all free ends with their layers (for same-layer filtering)
                for fe_x, fe_y, fe_layer in free_ends:
                    stubs.append((fe_x, fe_y, fe_layer))
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


def segments_intersect(a1: Tuple[float, float], a2: Tuple[float, float],
                       b1: Tuple[float, float], b2: Tuple[float, float]) -> bool:
    """Check if line segment a1-a2 intersects line segment b1-b2.

    Uses the counter-clockwise orientation test for robust intersection detection.
    """
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    # Check if segments share an endpoint (not a real crossing)
    eps = 0.001
    for p1 in [a1, a2]:
        for p2 in [b1, b2]:
            if abs(p1[0] - p2[0]) < eps and abs(p1[1] - p2[1]) < eps:
                return False

    return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) and (ccw(a1, a2, b1) != ccw(a1, a2, b2))


def compute_mst_edges(points: List[Tuple[float, float]], use_manhattan: bool = False) -> List[Tuple[int, int, float]]:
    """Compute minimum spanning tree edges between points using Prim's algorithm.

    Args:
        points: List of (x, y) coordinates
        use_manhattan: If True, use Manhattan distance; if False, use Euclidean

    Returns:
        List of (idx_a, idx_b, distance) tuples representing MST edges.
        The edges are in the order they were added to the tree (not sorted by length).
    """
    if len(points) < 2:
        return []
    if len(points) == 2:
        if use_manhattan:
            dist = abs(points[0][0] - points[1][0]) + abs(points[0][1] - points[1][1])
        else:
            dist = math.sqrt((points[0][0] - points[1][0])**2 + (points[0][1] - points[1][1])**2)
        return [(0, 1, dist)]

    # Prim's algorithm
    in_tree = {0}  # Start with first point
    edges = []

    while len(in_tree) < len(points):
        best_edge = None
        best_dist = float('inf')

        for i in in_tree:
            for j in range(len(points)):
                if j in in_tree:
                    continue
                if use_manhattan:
                    dist = abs(points[i][0] - points[j][0]) + abs(points[i][1] - points[j][1])
                else:
                    dist = math.sqrt((points[i][0] - points[j][0])**2 +
                                    (points[i][1] - points[j][1])**2)
                if dist < best_dist:
                    best_dist = dist
                    best_edge = (i, j, dist)

        if best_edge:
            in_tree.add(best_edge[1])
            edges.append(best_edge)

    return edges


def compute_mst_segments(points: List[Tuple[float, float]]) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Compute minimum spanning tree segments between points using Prim's algorithm.

    Returns list of (point1, point2) tuples representing MST edges.
    Uses Euclidean distance.
    """
    edges = compute_mst_edges(points, use_manhattan=False)
    return [(points[e[0]], points[e[1]]) for e in edges]


def get_net_mst_segments(pcb_data: PCBData, net_id: int) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Get MST segments representing the routing path for a net.

    For 2-pad nets: returns single segment between pads.
    For 3+ pad nets: returns MST segments connecting all pads.

    Uses stub endpoints if available, otherwise pad positions.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])

    # Case 1: Has stubs - use stub free ends
    if net_segments:
        groups = find_connected_groups(net_segments)
        if len(groups) >= 2:
            # Multiple stub groups - get free end of each
            points = []
            for group in groups:
                free_ends = find_stub_free_ends(group, net_pads)
                if free_ends:
                    points.append((free_ends[0][0], free_ends[0][1]))
                else:
                    # Fallback to centroid
                    pts = []
                    for seg in group:
                        pts.append((seg.start_x, seg.start_y))
                        pts.append((seg.end_x, seg.end_y))
                    cx = sum(p[0] for p in pts) / len(pts)
                    cy = sum(p[1] for p in pts) / len(pts)
                    points.append((cx, cy))
            return compute_mst_segments(points)

    # Case 2: No stubs, just pads - use pad positions
    if len(net_pads) >= 2:
        points = [(pad.global_x, pad.global_y) for pad in net_pads]
        return compute_mst_segments(points)

    return []


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
                seg_points.add((round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS)))
                seg_points.add((round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS)))

            stub_pts = []
            for seg in group:
                stub_pts.append((seg.start_x, seg.start_y))
                stub_pts.append((seg.end_x, seg.end_y))
            stub_cx = sum(p[0] for p in stub_pts) / len(stub_pts)
            stub_cy = sum(p[1] for p in stub_pts) / len(stub_pts)

            # Find unconnected pads
            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, POSITION_DECIMALS), round(pad.global_y, POSITION_DECIMALS))
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


def find_connected_segment_positions(pcb_data: PCBData, start_x: float, start_y: float,
                                      net_id: int, tolerance: float = 0.1,
                                      layer: str = None) -> set:
    """
    Find all segment endpoint positions connected to a starting position for a given net.

    Uses BFS to traverse the segment chain from the starting position.
    Returns a set of (x, y) tuples for all endpoints in the connected stub chain.

    Args:
        layer: Optional layer name to filter segments. When specified, only segments
               on this layer are considered. This prevents incorrectly connecting
               stubs that share XY coordinates but are on different layers.
    """
    # Get all segments for this net (optionally filtered by layer)
    if layer:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id and s.layer == layer]
    else:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]

    # Build adjacency: position -> list of connected positions
    adjacency = {}
    for seg in net_segments:
        start = pos_key(seg.start_x, seg.start_y)
        end = pos_key(seg.end_x, seg.end_y)
        if start not in adjacency:
            adjacency[start] = []
        if end not in adjacency:
            adjacency[end] = []
        adjacency[start].append(end)
        adjacency[end].append(start)

    # BFS from start position
    start_key = pos_key(start_x, start_y)
    visited = set()
    queue = [start_key]

    while queue:
        pos = queue.pop(0)
        if pos in visited:
            continue
        visited.add(pos)
        for neighbor in adjacency.get(pos, []):
            if neighbor not in visited:
                queue.append(neighbor)

    return visited


def find_connected_segments(pcb_data: PCBData, start_x: float, start_y: float,
                            net_id: int) -> List:
    """
    Find all segments connected to a starting position for a given net.

    Uses BFS to traverse the segment chain from the starting position.
    Returns a list of Segment objects in the connected stub chain.

    This differs from find_connected_segment_positions in that it returns
    the actual segment objects, not just their positions. This allows
    unambiguous identification of which segments belong to which stub chain,
    even when two chains share a common endpoint position.
    """
    # Get all segments for this net
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]

    # Build adjacency: position -> list of segments touching that position
    pos_to_segments = {}
    for seg in net_segments:
        start = pos_key(seg.start_x, seg.start_y)
        end = pos_key(seg.end_x, seg.end_y)
        if start not in pos_to_segments:
            pos_to_segments[start] = []
        if end not in pos_to_segments:
            pos_to_segments[end] = []
        pos_to_segments[start].append(seg)
        pos_to_segments[end].append(seg)

    # BFS from start position, collecting segments
    start_key = pos_key(start_x, start_y)
    visited_positions = set()
    visited_segments = set()
    queue = [start_key]

    while queue:
        pos = queue.pop(0)
        if pos in visited_positions:
            continue
        visited_positions.add(pos)

        for seg in pos_to_segments.get(pos, []):
            if id(seg) in visited_segments:
                continue
            visited_segments.add(id(seg))

            # Add the other endpoint of this segment to the queue
            other_end = pos_key(seg.end_x, seg.end_y) if pos_key(seg.start_x, seg.start_y) == pos else pos_key(seg.start_x, seg.start_y)
            if other_end not in visited_positions:
                queue.append(other_end)

    # Return the actual segment objects
    return [s for s in net_segments if id(s) in visited_segments]
