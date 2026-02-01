"""
Bus detection module for identifying groups of nets that should be routed together.

A bus is a group of nets where:
1. Source endpoints are physically clustered together
2. Target endpoints are also physically clustered together

This module detects such groups and orders the nets by physical position
for routing from the middle outward.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import math

from kicad_parser import PCBData
from connectivity import get_net_routing_endpoints


@dataclass
class BusGroup:
    """Represents a group of nets that should be routed as a bus."""
    name: str
    net_ids: List[int] = field(default_factory=list)
    # Source and target positions per net (parallel lists with net_ids)
    source_positions: List[Tuple[float, float]] = field(default_factory=list)
    target_positions: List[Tuple[float, float]] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.net_ids)


def detect_bus_groups(
    pcb_data: PCBData,
    net_ids: List[int],
    detection_radius: float = 2.0,
    min_nets: int = 2,
) -> List[BusGroup]:
    """
    Detect bus groups by finding nets where EITHER all sources are within radius
    of each other OR all targets are within radius of each other.

    Args:
        pcb_data: PCB data with net information
        net_ids: List of net IDs to analyze for bus grouping
        detection_radius: Maximum distance (mm) - either all sources or all targets
                         must be within this distance of each other
        min_nets: Minimum number of nets to form a bus (default 2)

    Returns:
        List of BusGroup objects, each containing nets that form a bus
    """
    # Get endpoints for each net
    net_endpoints: Dict[int, Tuple[Tuple[float, float], Tuple[float, float]]] = {}

    for net_id in net_ids:
        endpoints = get_net_routing_endpoints(pcb_data, net_id)
        if len(endpoints) >= 2:
            net_endpoints[net_id] = (endpoints[0], endpoints[1])

    if len(net_endpoints) < min_nets:
        return []

    bus_groups = []
    bus_counter = 0
    remaining = set(net_endpoints.keys())

    while len(remaining) >= min_nets:
        source_positions = {nid: net_endpoints[nid][0] for nid in remaining}
        target_positions = {nid: net_endpoints[nid][1] for nid in remaining}

        # Find largest clique from sources OR targets
        source_clique = _find_largest_clique(source_positions, detection_radius, min_nets)
        target_clique = _find_largest_clique(target_positions, detection_radius, min_nets)

        # Use whichever is larger
        if len(source_clique) >= len(target_clique):
            best_bus_nets = source_clique
        else:
            best_bus_nets = target_clique

        if len(best_bus_nets) >= min_nets:
            bus_counter += 1
            bus = BusGroup(name=f"bus_{bus_counter}")

            # Order nets by physical position
            ordered_nets = _order_nets_by_position(best_bus_nets, net_endpoints)

            for net_id in ordered_nets:
                bus.net_ids.append(net_id)
                bus.source_positions.append(net_endpoints[net_id][0])
                bus.target_positions.append(net_endpoints[net_id][1])

            bus_groups.append(bus)

            # Remove these nets from consideration
            for nid in best_bus_nets:
                remaining.discard(nid)
        else:
            # No valid bus found, done
            break

    return bus_groups


def _all_within_radius(
    net_ids: List[int],
    positions: Dict[int, Tuple[float, float]],
    radius: float
) -> bool:
    """Check if all positions are within radius of each other (pairwise)."""
    for i, nid1 in enumerate(net_ids):
        x1, y1 = positions[nid1]
        for nid2 in net_ids[i+1:]:
            x2, y2 = positions[nid2]
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            if dist > radius:
                return False
    return True


def _find_largest_clique(
    positions: Dict[int, Tuple[float, float]],
    radius: float,
    min_size: int
) -> List[int]:
    """
    Find the largest group where all members are within radius of each other.

    Uses greedy approach: start with closest pair, add items that are within
    radius of all existing members.
    """
    if len(positions) < min_size:
        return []

    items = list(positions.keys())

    # Find all pairs within radius
    edges = []
    for i, id1 in enumerate(items):
        x1, y1 = positions[id1]
        for id2 in items[i+1:]:
            x2, y2 = positions[id2]
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            if dist <= radius:
                edges.append((dist, id1, id2))

    if not edges:
        return []

    # Sort by distance (closest first)
    edges.sort()

    # Try building cliques starting from each edge
    best_clique = []
    for _, id1, id2 in edges:
        clique = [id1, id2]

        # Try adding other items
        for other in items:
            if other in clique:
                continue
            ox, oy = positions[other]
            # Check if within radius of all clique members
            all_close = True
            for member in clique:
                mx, my = positions[member]
                if math.sqrt((ox - mx) ** 2 + (oy - my) ** 2) > radius:
                    all_close = False
                    break
            if all_close:
                clique.append(other)

        if len(clique) > len(best_clique):
            best_clique = clique

    return best_clique if len(best_clique) >= min_size else []


def _order_nets_by_position(
    net_ids: List[int],
    net_endpoints: Dict[int, Tuple[Tuple[float, float], Tuple[float, float]]]
) -> List[int]:
    """
    Order nets by physical position (left-to-right or top-to-bottom).

    Determines the primary axis of the bus (horizontal or vertical) and
    sorts nets accordingly.

    Args:
        net_ids: List of net IDs to order
        net_endpoints: Dict mapping net ID to (source, target) positions

    Returns:
        Ordered list of net IDs
    """
    if len(net_ids) <= 1:
        return list(net_ids)

    # Get source positions for all nets
    sources = [(nid, net_endpoints[nid][0]) for nid in net_ids]

    # Determine primary axis by looking at spread in X vs Y
    xs = [p[0] for _, p in sources]
    ys = [p[1] for _, p in sources]

    x_spread = max(xs) - min(xs)
    y_spread = max(ys) - min(ys)

    # Sort by the axis with less spread (perpendicular to bus direction)
    # If bus runs horizontally (large X spread), sort by Y position
    # If bus runs vertically (large Y spread), sort by X position
    if x_spread > y_spread:
        # Bus runs horizontally, sort by Y
        sources.sort(key=lambda item: item[1][1])
    else:
        # Bus runs vertically, sort by X
        sources.sort(key=lambda item: item[1][0])

    return [nid for nid, _ in sources]


def get_bus_routing_order(bus: BusGroup) -> List[int]:
    """
    Get the order in which bus nets should be routed.

    Routes from the middle outward, alternating sides.
    Example for 5 nets [A, B, C, D, E] ordered by position:
    Route order: [C, B, D, A, E] (middle, left, right, left, right)

    Args:
        bus: BusGroup with nets ordered by physical position

    Returns:
        List of net IDs in routing order
    """
    n = len(bus.net_ids)
    if n == 0:
        return []
    if n == 1:
        return list(bus.net_ids)

    middle = n // 2
    order = [bus.net_ids[middle]]

    for i in range(1, n):
        left_idx = middle - i
        right_idx = middle + i

        if left_idx >= 0:
            order.append(bus.net_ids[left_idx])
        if right_idx < n:
            order.append(bus.net_ids[right_idx])

    return order


def get_attraction_neighbor(
    bus: BusGroup,
    net_id: int,
    routed_paths: Dict[int, List[Tuple[int, int, int]]]
) -> Optional[List[Tuple[int, int, int]]]:
    """
    Get the path of the already-routed neighbor that this net should attract to.

    Args:
        bus: BusGroup containing the net
        net_id: Net ID being routed
        routed_paths: Dict mapping net ID to routed path [(gx, gy, layer), ...]

    Returns:
        Path of the neighbor to attract to, or None if no neighbor routed yet
    """
    if net_id not in bus.net_ids:
        return None

    idx = bus.net_ids.index(net_id)

    # Check left neighbor first
    if idx > 0:
        left_neighbor = bus.net_ids[idx - 1]
        if left_neighbor in routed_paths:
            return routed_paths[left_neighbor]

    # Check right neighbor
    if idx < len(bus.net_ids) - 1:
        right_neighbor = bus.net_ids[idx + 1]
        if right_neighbor in routed_paths:
            return routed_paths[right_neighbor]

    return None
