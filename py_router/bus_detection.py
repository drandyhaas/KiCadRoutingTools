"""
Bus detection module for identifying groups of nets that should be routed together.

A bus is a group of nets where:
1. Source endpoints are physically clustered together
2. Target endpoints are also physically clustered together

This module detects such groups and orders the nets by physical position
for routing from the middle outward.
"""
from __future__ import annotations

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
    # Which endpoint type formed the clique (determines routing direction)
    clique_endpoint: str = "source"  # "source" or "target"
    # Declared by the user (--bus-nets) rather than detected: net_ids is the
    # side-by-side order the members must keep along the whole run, and each
    # member is attracted to its own lane of the planned corridor.
    ordered: bool = False

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

        # Use whichever is larger, and track which endpoint formed the clique
        if len(source_clique) >= len(target_clique):
            best_bus_nets = source_clique
            clique_endpoint = "source"
        else:
            best_bus_nets = target_clique
            clique_endpoint = "target"

        if len(best_bus_nets) >= min_nets:
            # Both-end coherence (#296 R9): a one-ended clique at a BGA sweeps
            # up EVERYTHING fanning out of the package (ottercast: a 21-net
            # "bus" mixing the SDC0 river with power nets, local caps and LEDs
            # bound for different corners -- one planned corridor for all of
            # them is noise for most). Sub-cluster the clique by the OTHER
            # endpoint (greedy centroid clustering at 2x the detection radius,
            # destinations spread wider than sources along connectors) and
            # emit only destination-coherent subgroups; the leftovers are not
            # buses and route normally.
            other = 1 if clique_endpoint == "source" else 0
            clusters: List[List[int]] = []
            for nid in best_bus_nets:
                px, py = net_endpoints[nid][other]
                placed = False
                for cl in clusters:
                    # math.fsum, not the builtin sum() (#493): Python 3.12
                    # switched sum() to compensated summation, so a float sum
                    # differs by an ULP or two between KiCad's bundled python
                    # and the system one. These centroids are compared against
                    # a radius below, so an ULP can flip cluster membership and
                    # change which nets are treated as a bus -- i.e. the routing
                    # would depend on the interpreter. fsum is exactly rounded
                    # on every version.
                    cx = math.fsum(net_endpoints[m][other][0] for m in cl) / len(cl)
                    cy = math.fsum(net_endpoints[m][other][1] for m in cl) / len(cl)
                    if math.hypot(px - cx, py - cy) <= 2.0 * detection_radius:
                        cl.append(nid)
                        placed = True
                        break
                if not placed:
                    clusters.append([nid])

            for cl in clusters:
                if len(cl) < min_nets:
                    continue
                bus_counter += 1
                bus = BusGroup(name=f"bus_{bus_counter}",
                               clique_endpoint=clique_endpoint)
                # Order nets by physical position
                ordered_nets = _order_nets_by_position(cl, net_endpoints)
                for net_id in ordered_nets:
                    bus.net_ids.append(net_id)
                    bus.source_positions.append(net_endpoints[net_id][0])
                    bus.target_positions.append(net_endpoints[net_id][1])
                bus_groups.append(bus)

            # Remove EVERY clique member from consideration (emitted or not:
            # re-considering the leftovers would just re-form the same clique).
            for nid in best_bus_nets:
                remaining.discard(nid)
        else:
            # No valid bus found, done
            break

    return bus_groups


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

    # Sort by the axis with larger spread to get physical ordering
    if x_spread >= y_spread:
        # Sort by X (left to right)
        sources.sort(key=lambda item: item[1][0])
    else:
        # Sort by Y (top to bottom)
        sources.sort(key=lambda item: item[1][1])

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


#: The axes a net's run is measured along: x, y and both diagonals. Every
#: direction is within 22.5 deg of one of them. Measuring along x + y alone
#: (#1284) lost every net running along the -45 deg diagonal, where x + y is
#: constant: a zero-length run, or a direction picked by sub-mm jitter.
_RUN_AXES = ((1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, -1.0))


def _run_endpoints(pads):
    """The two pads spanning a net's run: the farthest-apart of the pairs
    extreme along each of :data:`_RUN_AXES`. Exact for a two-pad net, within
    cos(22.5 deg) of the true span otherwise, and O(n) -- a pad-heavy net
    the caller did not declare as power still reaches the filter. The pair
    has no meaningful order; a net has no direction."""
    best, best_d2 = None, -1.0
    for ax, ay in _RUN_AXES:
        lo = min(pads, key=lambda p: ax * p.global_x + ay * p.global_y)
        hi = max(pads, key=lambda p: ax * p.global_x + ay * p.global_y)
        d2 = (hi.global_x - lo.global_x) ** 2 + (hi.global_y - lo.global_y) ** 2
        if d2 > best_d2:
            best, best_d2 = (lo, hi), d2
    return best


def filter_bus_groups_geometric(pcb_data, bus_groups, config,
                                min_members: int = 3,
                                min_run_mm: float = 5.0,
                                cos_cone: float = 0.9,
                                max_len_ratio: float = 1.5):
    """Strict geometric bus definition (default on; KICAD_BUS_STRICT=0
    reverts to raw clique detection): a group is a bus only if its members
    genuinely TRAVEL TOGETHER. Four rules, no names, no component identity:

      1. >=3 members (pairs belong to the diff router or aren't worth
         bundling),
      2. parallel travel: member runs (:func:`_run_endpoints`) within a
         cos>0.9 cone either way round, since a net has no direction, at
         length ratio <=1.5 (largest coherent subset kept --
         clustered sources + this cone geometrically imply clustered
         destinations, subsuming any shared-component test),
      3. run length >= 5mm (local hops gain nothing; also retires the
         decoupling-cap clusters without a passive-component carve-out),
      4. no power nets (config.power_net_widths).

    A member with fewer than 2 pads has no run and is dropped too. Diff-pair
    members are NOT filtered here: the *_P/*_N pair map is the caller's
    concern.

    Raw clique detection on ottercast declared 31 groups/118 members and
    planned corridors for decoupling clusters and power rails; these rules
    yield 6 groups/27 members -- the four real buses (SDC0, WL_SDIO,
    BT_PCM, BT_UART) plus small co-traveling GPIO/cap bundles a human
    would also lane together.
    """
    import os as _os
    if _os.environ.get('KICAD_BUS_STRICT', '1') in ('0', 'off', 'false'):
        return bus_groups
    import math as _math
    power_ids = set(getattr(config, 'power_net_widths', {}) or {})
    out = []
    for bus in bus_groups:
        vecs = {}
        for nid in bus.net_ids:
            if nid in power_ids:
                continue
            pads = pcb_data.pads_by_net.get(nid, [])
            if len(pads) < 2:
                continue
            a, b = _run_endpoints(pads)
            dx, dy = b.global_x - a.global_x, b.global_y - a.global_y
            L = _math.hypot(dx, dy)
            if L >= min_run_mm:
                vecs[nid] = (dx / L, dy / L, L)
        best = []
        for nid, (ux, uy, L) in vecs.items():
            cone = [m for m, (vx, vy, M) in vecs.items()
                    if abs(ux * vx + uy * vy) > cos_cone
                    and max(L, M) / min(L, M) <= max_len_ratio]
            if len(cone) > len(best):
                best = cone
        if len(best) < min_members:
            continue
        if len(best) < len(bus.net_ids):
            kept = [nid for nid in bus.net_ids if nid in set(best)]
            bus.net_ids = kept
        out.append(bus)
    return out


def bus_attraction_context(
    net_id: int,
    bus_net_to_group: Optional[Dict[int, BusGroup]],
    bus_corridors: Optional[Dict[str, List[Tuple[int, int, int]]]],
    routed_paths: Optional[Dict[int, List[Tuple[int, int, int]]]] = None
) -> Tuple[Optional[List[Tuple[int, int, int]]], bool]:
    """(attraction_path, reverse_direction) for a bus member.

    The one place that encodes the priority every routing site must share:
    an ordered bus member's own LANE of the planned corridor first (keyed
    (group.name, net_id)), else the group's PLANNED corridor (one centerline
    for the whole group, guide included), else an already-routed neighbor's path; route from the
    clustered endpoints when the clique was target-based. Non-members get
    (None, False). Callers pass whatever paths dict they track (the main
    loop's sampled bus paths, the reroute loop's routed_net_paths) or None.
    """
    bg = (bus_net_to_group or {}).get(net_id)
    if bg is None:
        return None, False
    corridors = bus_corridors or {}
    # An ordered (declared) bus: the member's own lane of the corridor first.
    attr = (corridors.get((bg.name, net_id)) or corridors.get(bg.name)
            or get_attraction_neighbor(bg, net_id, routed_paths or {}))
    return attr, (bg.clique_endpoint == "target")


def declared_bus_groups(pcb_data: PCBData, specs: List[List[str]]) -> List[BusGroup]:
    """The buses the user declared (--bus-nets "A,B,C", repeatable): one
    ordered BusGroup per list, its members in the list's order -- the order
    they keep side by side along their common run. An unknown net name or a
    net named twice is an error (a silent drop would reorder the lanes)."""
    by_name = {net.name: nid for nid, net in pcb_data.nets.items()}
    groups: List[BusGroup] = []
    seen: Dict[int, int] = {}
    for gi, names in enumerate(specs):
        ids: List[int] = []
        for name in names:
            if name not in by_name:
                raise ValueError(f"--bus-nets: no net named {name!r}")
            nid = by_name[name]
            if nid in seen:
                raise ValueError(f"--bus-nets: net {name!r} declared twice")
            seen[nid] = gi
            ids.append(nid)
        if len(ids) < 2:
            raise ValueError(f"--bus-nets: a bus needs two nets or more, got {names}")
        ends = [get_net_routing_endpoints(pcb_data, nid) for nid in ids]
        groups.append(BusGroup(
            name=f"declared_bus_{gi}", net_ids=ids,
            source_positions=[e[0] if e else (0.0, 0.0) for e in ends],
            target_positions=[e[1] if len(e) > 1 else (0.0, 0.0) for e in ends],
            clique_endpoint="source", ordered=True))
    return groups


def net_points(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float]]:
    """Every place a net has copper to join (its pads, its tracks' ends), mm."""
    pts = [(p.global_x, p.global_y) for p in pcb_data.pads_by_net.get(net_id, [])]
    for seg in pcb_data.segments:
        if seg.net_id == net_id:
            pts += [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]
    return pts


def net_span(pcb_data: PCBData, net_id: int
             ) -> Tuple[float, Optional[Tuple[Tuple[float, float], Tuple[float, float]]]]:
    """A net's whole run: its two farthest-apart points and their distance
    (mm). A multipoint member's routing endpoints may be a short local edge;
    an ordered bus's corridor follows the run instead."""
    pts = sorted(set(net_points(pcb_data, net_id)))
    best = (0.0, None)
    for i, a in enumerate(pts):
        for b in pts[i + 1:]:
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            if d > best[0]:
                best = (d, (a, b))
    return best


def ordered_bus_representative(pcb_data: PCBData, bus: BusGroup) -> int:
    """The member whose run is longest (ties: the earlier in the order): an
    ordered bus's corridor is probed along it, so every member's lane runs as
    far as the farthest-reaching member does."""
    return max(bus.net_ids, key=lambda nid: (net_span(pcb_data, nid)[0], -bus.net_ids.index(nid)))


def lane_offsets(n: int, pitch: float, centre: Optional[int] = None) -> List[float]:
    """Sideways offset of each of n lanes from the corridor's centreline, in
    the corridor's left-hand normal, one pitch apart: lane i at
    (i - centre) * pitch, centre the lane the centreline is (by default the
    middle, (n-1)/2; an ordered bus's corridor is its representative's lane)."""
    c = (n - 1) / 2.0 if centre is None else centre
    return [(i - c) * pitch for i in range(n)]


def _tangent(path: List[Tuple[int, int, int]], i: int, window: int) -> Tuple[float, float]:
    """The unit direction of the path at point i, over +-window points (a
    smoothed tangent, so a 45 degree jog does not swing the lanes)."""
    a = path[max(0, i - window)]
    b = path[min(len(path) - 1, i + window)]
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    if d == 0:
        return (1.0, 0.0)
    return (dx / d, dy / d)


def offset_path(path: List[Tuple[int, int, int]], offset: float,
                window: int = 4) -> List[Tuple[int, int, int]]:
    """The dense (gx, gy, layer) path moved sideways by offset grid cells
    along its left-hand normal (-ty, tx), each point keeping its layer;
    consecutive duplicates dropped. offset 0 returns the path unchanged."""
    if offset == 0 or len(path) < 2:
        return list(path)
    out: List[Tuple[int, int, int]] = []
    for i, (x, y, layer) in enumerate(path):
        tx, ty = _tangent(path, i, window)
        p = (int(round(x - ty * offset)), int(round(y + tx * offset)), layer)
        if not out or out[-1] != p:
            out.append(p)
    return out


def lane_sign(path: List[Tuple[int, int, int]], ends_grid: List[List[Tuple[int, int]]]) -> int:
    """+1 if the members' ends, taken in the declared order, run along the
    corridor's left-hand normal at the end of the corridor nearer to them,
    -1 if against it (then the lanes are mirrored, so member 0 takes the
    side its own pads are on). ends_grid[i] is member i's endpoints in grid
    coordinates. Both corridor ends vote; a tie keeps +1."""
    if len(path) < 2 or not ends_grid:
        return 1
    n = len(ends_grid)
    mid = (n - 1) / 2.0
    vote = 0.0
    for k in (0, len(path) - 1):
        cx, cy, _ = path[k]
        tx, ty = _tangent(path, k, 4)
        nx, ny = -ty, tx
        for i, ends in enumerate(ends_grid):
            if not ends:
                continue
            ex, ey = min(ends, key=lambda e: math.hypot(e[0] - cx, e[1] - cy))
            vote += (i - mid) * ((ex - cx) * nx + (ey - cy) * ny)
    return -1 if vote < 0 else 1


def lane_paths(path: List[Tuple[int, int, int]], net_ids: List[int], pitch: float,
               ends_grid: List[List[Tuple[int, int]]],
               centre: Optional[int] = None) -> Dict[int, List[Tuple[int, int, int]]]:
    """Each member's own lane of an ordered bus: the corridor's centreline
    offset by its lane (lane_offsets, pitch in grid cells; centre the index
    of the member the corridor was probed along), mirrored by lane_sign so
    the declared order meets the members' copper the right way round.
    ends_grid[i] is member i's copper points in grid cells.
    {net_id: dense (gx, gy, layer) path}."""
    sign = lane_sign(path, ends_grid)
    return {nid: offset_path(path, sign * off)
            for nid, off in zip(net_ids, lane_offsets(len(net_ids), pitch, centre))}


class Lane(list):
    """An ordered bus member's lane: a dense (gx, gy, layer) attraction path
    whose direction is the member's, not the corridor probe's. The router's
    attraction rewards only moves ALONG the path's direction, so a lane is
    turned to each route's direction of travel (LaneOrientedRouter)."""


class LaneOrientedRouter:
    """A GridRouter whose attraction path (a Lane) is set, before each
    route, pointing from that route's sources towards its targets -- forward
    and backward probes alike, multipoint edges in either sense. Everything
    else is the wrapped router's."""

    def __init__(self, router, lane: List[Tuple[int, int, int]]):
        self._router = router
        self._ways = (list(lane), list(reversed(lane)))
        self._current = 0
        router.set_attraction_path(self._ways[0])

    def _orient(self, sources, targets) -> None:
        if not sources or not targets or len(self._ways[0]) < 2:
            return
        sx = math.fsum(p[0] for p in sources) / len(sources)
        sy = math.fsum(p[1] for p in sources) / len(sources)
        tx = math.fsum(p[0] for p in targets) / len(targets)
        ty = math.fsum(p[1] for p in targets) / len(targets)
        head, tail = self._ways[0][0], self._ways[0][-1]
        along = (math.hypot(head[0] - sx, head[1] - sy) + math.hypot(tail[0] - tx, tail[1] - ty))
        against = (math.hypot(tail[0] - sx, tail[1] - sy) + math.hypot(head[0] - tx, head[1] - ty))
        way = 0 if along <= against else 1
        if way != self._current:
            self._router.set_attraction_path(self._ways[way])
            self._current = way

    def route_with_frontier(self, obstacles, sources, targets, *args, **kwargs):
        self._orient(sources, targets)
        return self._router.route_with_frontier(obstacles, sources, targets, *args, **kwargs)

    def route_multi(self, obstacles, sources, targets, *args, **kwargs):
        self._orient(sources, targets)
        return self._router.route_multi(obstacles, sources, targets, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._router, name)
