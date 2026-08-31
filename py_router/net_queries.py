"""
Net query utilities for PCB routing.

Functions for querying pads, nets, differential pairs, and computing
MPS (Maximum Planar Subset) net ordering.
"""
from __future__ import annotations

import math
import difflib
import fnmatch
import re
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Set, Union


def split_net_patterns(patterns: List[str],
                       known_net_names: Optional[Set[str]] = None) -> Tuple[List[str], List[str]]:
    """
    Split filter patterns into (include_patterns, exclude_patterns).

    A leading "!" normally marks an exclusion pattern, but the active-low net
    naming convention (e.g. !RESET, !CS, !MIX_BYPASS) means a net can legitimately
    have a name that starts with "!". To keep such nets routable by literal name
    (issue #177):

    - "\\!FOO" is an *escaped literal* -- the leading backslash is stripped and
      "!FOO" is treated as an inclusion pattern.
    - "!FOO" is an exclusion of "FOO", *unless* "!FOO" verbatim names a known net,
      in which case it is treated as a literal inclusion of that active-low net.
    - everything else is an inclusion pattern.

    Args:
        patterns: Raw filter patterns.
        known_net_names: Net names that exist on the board; a "!"-prefixed pattern
            matching one verbatim is treated as a literal include rather than an
            exclusion. Defaults to empty (every "!"-pattern is an exclusion).

    Returns:
        (include_patterns, exclude_patterns) with the leading "!"/"\\" removed.
    """
    known = known_net_names or set()
    include_patterns: List[str] = []
    exclude_patterns: List[str] = []
    for p in patterns:
        if p.startswith('\\!'):
            include_patterns.append(p[1:])       # escaped literal "!FOO"
        elif p.startswith('!') and p not in known:
            exclude_patterns.append(p[1:])       # genuine exclusion of "FOO"
        else:
            include_patterns.append(p)           # include (covers literal "!FOO" nets)
    return include_patterns, exclude_patterns


def net_pattern_matches(net_name: str, pattern: str) -> bool:
    """Match one net name against one fnmatch pattern, sheet-path aware (#493).

    KiCad qualifies a net with the sheet it was declared on, so the board's net
    is '/GND' or '/Management Interface/VSMPS', not 'GND'. A plain
    `fnmatch(net_name, pattern)` therefore makes the unqualified spelling people
    actually write -- `--nets '*' '!GND'` -- match NOTHING, so the exclusion
    silently no-ops and the net gets routed anyway. That is issue #292
    (core1106_cam routed its plane nets as traces) and it recurred on eth_tap,
    whose step-1 fanout shipped 267 '/GND' + 171 '/3V3' segments out of 750 from
    a command that asked to exclude both.

    So: a pattern that names NO path ('GND', 'VCC*') is understood as naming the
    net itself and is matched against the trailing path component as well. A
    pattern that DOES carry a path ('/GND', '/Sheet/*') is taken literally and
    only ever full-matched, so an explicit spelling stays exact.

        net_pattern_matches('/GND', 'GND')                  -> True
        net_pattern_matches('/Analog/GND', 'GND')           -> True
        net_pattern_matches('/GND_A', 'GND')                -> False
        net_pattern_matches('/GND', '/GND')                 -> True
        net_pattern_matches('/Analog/GND', '/GND')          -> False  (path given)
    """
    if fnmatch.fnmatch(net_name, pattern):
        return True
    if '/' not in pattern and '/' in net_name:
        return fnmatch.fnmatch(net_name.rsplit('/', 1)[-1], pattern)
    return False


def matches_net_filter(net_name: str, patterns: List[str]) -> bool:
    """
    Check if a net name matches a list of filter patterns.

    Patterns can include * and ? wildcards (fnmatch style).
    Patterns starting with "!" are exclusion patterns (but see split_net_patterns
    for how active-low net names like "!RESET" stay selectable by literal name).

    Logic:
    - If there are inclusion patterns (not starting with !), net must match at least one
    - If there are exclusion patterns (starting with !), net must not match any
    - If only exclusion patterns are provided, all non-excluded nets match

    Examples:
        matches_net_filter("GND", ["*"])           -> True
        matches_net_filter("GND", ["*", "!GND"])   -> False
        matches_net_filter("VCC", ["*", "!GND"])   -> True
        matches_net_filter("NET1", ["!GND", "!VCC"]) -> True (no inclusion = match all)
        matches_net_filter("GND", ["!GND", "!VCC"])  -> False
        matches_net_filter("!RESET", ["!RESET"])     -> True (literal active-low net)
        matches_net_filter("!RESET", ["\\!RESET"])   -> True (escaped literal)

    Args:
        net_name: The net name to check
        patterns: List of patterns (may include wildcards and ! prefix for exclusion)

    Returns:
        True if the net should be included, False otherwise
    """
    if not patterns:
        return True  # No filter = include all

    # Treat a "!"-pattern that verbatim names the net under test as a literal
    # include, so active-low nets (!RESET, !MIX_BYPASS) match by name (issue #177).
    include_patterns, exclude_patterns = split_net_patterns(patterns, {net_name})

    # Check exclusion first: if net matches any exclude pattern, reject it
    if exclude_patterns:
        for pattern in exclude_patterns:
            if net_pattern_matches(net_name, pattern):
                return False

    # Check inclusion: if there are include patterns, must match at least one
    if include_patterns:
        for pattern in include_patterns:
            if net_pattern_matches(net_name, pattern):
                return True
        return False  # Has include patterns but didn't match any

    # No include patterns (only exclusions) and didn't match any exclusion
    return True

from kicad_parser import PCBData, Segment, Via, Pad
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from chip_boundary import (
    build_chip_list, identify_chip_for_point, compute_far_side,
    compute_boundary_position, crossings_from_boundary_order
)
from routing_utils import pos_key, segment_length
from connectivity import (
    find_connected_groups, find_stub_free_ends, get_net_routing_endpoints,
    get_net_mst_segments, segments_intersect
)


@dataclass
class MPSResult:
    """Extended result from MPS net ordering with conflict and layer information."""
    ordered_ids: List[int]                                    # Ordered net IDs
    conflicts: Dict[int, Set[int]]                            # unit_id -> set of conflicting unit_ids (layer-filtered)
    unit_layers: Dict[int, Tuple[Set[str], Set[str]]]         # unit_id -> (src_layers, tgt_layers)
    unit_to_nets: Dict[int, List[int]]                        # unit_id -> [net_ids]
    unit_names: Dict[int, str]                                # unit_id -> display name
    round_assignments: Dict[int, int]                         # unit_id -> round_number (1-indexed)
    num_rounds: int
    geometric_conflicts: Dict[int, Set[int]] = None           # All crossings regardless of layer (for swap checking)


def calculate_route_length(segments: List[Segment], vias: List[Via] = None, pcb_data=None) -> float:
    """
    Calculate the total length of a routed path.

    Args:
        segments: List of Segment objects making up the route
        vias: Optional list of Via objects for via barrel length calculation
        pcb_data: Optional PCBData with stackup info for via barrel lengths

    Returns:
        Total route length in mm (sum of all segment lengths + via barrel lengths)
    """
    total = 0.0
    for seg in segments:
        total += segment_length(seg)

    # Add via barrel lengths if pcb_data with stackup is provided
    if vias and pcb_data and hasattr(pcb_data, 'get_via_barrel_length'):
        for via in vias:
            if via.layers and len(via.layers) >= 2:
                barrel_len = pcb_data.get_via_barrel_length(via.layers[0], via.layers[1])
                total += barrel_len

    return total


def calculate_via_barrel_length(vias: List[Via], pcb_data) -> float:
    """
    Calculate total via barrel length for a list of vias.

    Args:
        vias: List of Via objects
        pcb_data: PCBData with stackup info

    Returns:
        Total via barrel length in mm
    """
    if not vias or not pcb_data or not hasattr(pcb_data, 'get_via_barrel_length'):
        return 0.0

    total = 0.0
    for via in vias:
        if via.layers and len(via.layers) >= 2:
            total += pcb_data.get_via_barrel_length(via.layers[0], via.layers[1])
    return total


def net_copper_length(pcb_data: PCBData, net_id: int,
                      include_vias: bool = True) -> float:
    """Total copper length of ONE net already on a board, in mm (#489 §7).

    The supported way to ask "how long is this net?" from a parsed board --
    `calculate_route_length` takes a SEGMENT LIST, not a PCBData + net id, and
    every caller that guessed otherwise (the review-routed-board recipe did)
    raised. Sums the net's segments plus, with include_vias, its via barrels
    from the stackup, matching KiCad's measurement.

    NOTE this is TOTAL net copper, not a pin-to-pin path: on a multipoint or
    fly-by net (or one with stubs) it exceeds every actual signal path. Use
    `pin_pair_path_length` when the number has to mean electrical delay.
    """
    segments = [s for s in pcb_data.segments if s.net_id == net_id]
    vias = [v for v in pcb_data.vias if v.net_id == net_id] if include_vias else None
    return calculate_route_length(segments, vias, pcb_data)


def net_copper_lengths(pcb_data: PCBData, net_ids=None,
                       include_vias: bool = True) -> Dict[int, float]:
    """{net_id: total copper length mm} in ONE pass over the board's copper.

    Same measurement as `net_copper_length`, but for many nets at once -- a
    length-match group audit would otherwise rescan every segment per net.
    net_ids=None covers every net that has copper.
    """
    wanted = set(net_ids) if net_ids is not None else None
    segs_by: Dict[int, List[Segment]] = {}
    vias_by: Dict[int, List[Via]] = {}
    for s in pcb_data.segments:
        if wanted is None or s.net_id in wanted:
            segs_by.setdefault(s.net_id, []).append(s)
    if include_vias:
        for v in pcb_data.vias:
            if wanted is None or v.net_id in wanted:
                vias_by.setdefault(v.net_id, []).append(v)
    out = {}
    for nid in (wanted if wanted is not None else set(segs_by) | set(vias_by)):
        out[nid] = calculate_route_length(segs_by.get(nid, []),
                                         vias_by.get(nid), pcb_data)
    return out


def _pad_holds_point(pad: Pad, x: float, y: float, tol: float = 0.02) -> bool:
    """Is (x, y) inside the pad's copper (bbox test + tolerance)?"""
    hx = pad.size_x / 2 + tol
    hy = pad.size_y / 2 + tol
    return abs(x - pad.global_x) <= hx and abs(y - pad.global_y) <= hy


def pin_pair_path_length(pcb_data: PCBData, net_id: int,
                         pad_a: Pad, pad_b: Pad,
                         tolerance: float = 0.02) -> Optional[float]:
    """Shortest copper path length between TWO PADS of one net, in mm (#489 §7).

    The pin-pair ("from-to") measurement length matching actually needs, and
    what `net_copper_length` cannot give: on a multipoint net -- a fly-by DDR
    address bus, a daisy-chained clock, anything with a stub -- total net copper
    is the sum of every branch and matches no signal path at all.

    Walks a WEIGHTED graph over the net's own copper: each segment contributes
    an edge of its own length, coincident endpoints and T-junctions (an endpoint
    landing on another segment's interior) join at zero cost, and a via joins
    the layers it spans at its barrel length from the stackup. Pads attach to
    any endpoint or T-point that falls inside their copper.

    Returns None when the two pads are not joined by copper (an unrouted or
    broken net) -- distinguish that from 0.0, which means pad-to-pad direct
    contact. Zone/pour connections are NOT traversed: a net that reaches its
    pads only through a plane has no track path and returns None.
    """
    segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if not segments:
        return None

    # Point interning: endpoints within `tolerance` are ONE node (the same
    # coincidence rule check_connected unions at). Bucketing on round(x/tol)
    # would NOT do -- two points 2 um apart straddling a bucket edge land in
    # different buckets, which silently breaks the walk at a joint.
    q = max(tolerance, 1e-9)
    _buckets: Dict[Tuple[int, int, str], List[Tuple[float, float, int]]] = {}
    _node_xy: List[Tuple[float, float, str]] = []

    def key(x: float, y: float, layer: str) -> int:
        bx, by = int(math.floor(x / q)), int(math.floor(y / q))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for (ex, ey, nid) in _buckets.get((bx + dx, by + dy, layer), ()):
                    if math.hypot(ex - x, ey - y) <= tolerance:
                        return nid
        nid = len(_node_xy)
        _node_xy.append((x, y, layer))
        _buckets.setdefault((bx, by, layer), []).append((x, y, nid))
        return nid

    adjacency: Dict[int, List[Tuple[int, float]]] = {}

    def add_edge(n1, n2, w: float):
        if n1 == n2:
            return
        adjacency.setdefault(n1, []).append((n2, w))
        adjacency.setdefault(n2, []).append((n1, w))

    copper_layers = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    from kicad_parser import pad_is_plated_through

    # Vias and the two pads are DISCS of copper, not points: a track ends
    # anywhere inside them, not on their centre (real boards miss the centre by
    # tens of um). Each disc is a landing zone -- any segment node inside it
    # joins at no cost, and a segment passing THROUGH it gets split there so a
    # via mid-run is reachable.
    discs = []  # (cx, cy, radius, layers, hit_test, kind, obj)
    for v in pcb_data.vias:
        if v.net_id != net_id or len(v.layers or ()) < 2:
            continue
        span = list(v.layers)
        if copper_layers and span[0] in copper_layers and span[1] in copper_layers:
            i, j = copper_layers.index(span[0]), copper_layers.index(span[1])
            spanned = copper_layers[min(i, j):max(i, j) + 1]
        else:
            spanned = span
        radius = max((getattr(v, 'size', 0) or 0) / 2, tolerance)
        discs.append((v.x, v.y, radius, spanned,
                      lambda x, y, _v=v, _r=radius: math.hypot(x - _v.x, y - _v.y) <= _r + tolerance,
                      'via', v))
    for pad in (pad_a, pad_b):
        pad_layers = [l for l in expand_pad_layers(pad.layers, copper_layers)]
        radius = max(pad.size_x, pad.size_y) / 2
        discs.append((pad.global_x, pad.global_y, radius, pad_layers,
                      lambda x, y, _p=pad: _pad_holds_point(_p, x, y, tolerance),
                      'pad', pad))

    # Cut each segment at every T-junction (another segment's endpoint on its
    # interior) and at every disc it passes through, then chain the pieces.
    endpoints_by_layer: Dict[str, List[Tuple[float, float]]] = {}
    for s in segments:
        endpoints_by_layer.setdefault(s.layer, []).extend(
            [(s.start_x, s.start_y), (s.end_x, s.end_y)])

    for s in segments:
        length = segment_length(s)
        if length < 1e-9:
            continue
        ux = (s.end_x - s.start_x) / length
        uy = (s.end_y - s.start_y) / length
        cuts = [0.0, length]

        def _project(px: float, py: float, max_perp: float):
            """Along-segment distance where (px, py) meets this segment, or
            None when it is farther off the line than max_perp."""
            t = (px - s.start_x) * ux + (py - s.start_y) * uy
            if t <= tolerance or t >= length - tolerance:
                return None
            perp = abs(-(px - s.start_x) * uy + (py - s.start_y) * ux)
            return t if perp <= max_perp else None

        for (px, py) in endpoints_by_layer.get(s.layer, ()):
            t = _project(px, py, tolerance)
            if t is not None:
                cuts.append(t)
        for (cx, cy, radius, dlayers, _hit, _kind, _obj) in discs:
            if s.layer not in dlayers:
                continue
            t = _project(cx, cy, radius + tolerance)
            if t is not None:
                cuts.append(t)

        cuts.sort()
        for t1, t2 in zip(cuts, cuts[1:]):
            if t2 - t1 < 1e-9:
                continue
            n1 = key(s.start_x + ux * t1, s.start_y + uy * t1, s.layer)
            n2 = key(s.start_x + ux * t2, s.start_y + uy * t2, s.layer)
            add_edge(n1, n2, t2 - t1)

    # Attach each disc: one synthetic node per layer it spans, joined at no cost
    # to every segment node inside its copper on that layer. A via's barrel then
    # links its own per-layer nodes at the length KiCad measures (charged once
    # across the span, so a mid-span layer change isn't billed twice); a plated
    # pad barrel links its layers at no cost.
    def _new_node(x: float, y: float, layer: str) -> int:
        nid = len(_node_xy)
        _node_xy.append((x, y, layer))
        return nid

    track_nodes = list(adjacency.keys())
    pad_nodes: Dict[int, List[int]] = {}
    for (cx, cy, radius, dlayers, hit, kind, obj) in discs:
        layer_nodes: Dict[str, int] = {}
        for layer in dlayers:
            hub = _new_node(cx, cy, layer)
            joined = False
            for node in track_nodes:
                nx, ny, nlayer = _node_xy[node]
                if nlayer != layer:
                    continue
                if hit(nx, ny):
                    add_edge(hub, node, 0.0)
                    joined = True
            if joined or kind == 'pad':
                layer_nodes[layer] = hub

        if kind == 'via':
            barrel = 0.0
            if hasattr(pcb_data, 'get_via_barrel_length'):
                barrel = pcb_data.get_via_barrel_length(dlayers[0], dlayers[-1])
            per_step = barrel / max(1, len(dlayers) - 1)
            for l1, l2 in zip(dlayers, dlayers[1:]):
                if l1 in layer_nodes and l2 in layer_nodes:
                    add_edge(layer_nodes[l1], layer_nodes[l2], per_step)
                elif l1 in layer_nodes or l2 in layer_nodes:
                    # An intermediate layer with no copper of this net still has
                    # to be walked THROUGH -- keep the chain intact.
                    for l in (l1, l2):
                        layer_nodes.setdefault(l, _new_node(cx, cy, l))
                    add_edge(layer_nodes[l1], layer_nodes[l2], per_step)
        else:
            hubs = list(layer_nodes.values())
            if pad_is_plated_through(obj) and len(hubs) > 1:
                for other in hubs[1:]:
                    add_edge(hubs[0], other, 0.0)
            pad_nodes[id(obj)] = hubs

    starts = pad_nodes.get(id(pad_a)) or []
    goals = set(pad_nodes.get(id(pad_b)) or [])
    if not starts or not goals:
        return None

    import heapq
    dist = {n: 0.0 for n in starts}
    heap = [(0.0, i, n) for i, n in enumerate(starts)]
    heapq.heapify(heap)
    counter = len(starts)
    while heap:
        d, _, node = heapq.heappop(heap)
        if d > dist.get(node, float('inf')) + 1e-12:
            continue
        if node in goals:
            return d
        for nxt, w in adjacency.get(node, ()):
            nd = d + w
            if nd < dist.get(nxt, float('inf')) - 1e-12:
                dist[nxt] = nd
                heapq.heappush(heap, (nd, counter, nxt))
                counter += 1
    return None


def routable_pad_count(pcb_data: PCBData, net_id: int, off_board=None) -> int:
    """Number of the net's pads that sit ON the board -- the pads the router can
    actually reach as endpoints. Off-board pads (#291) are dropped before
    routing, so they don't count. Pass a shared `off_board` predicate
    (check_drc.make_off_board_test) to avoid rebuilding it per net."""
    pads = pcb_data.pads_by_net.get(net_id, ())
    if off_board is None:
        return len(pads)
    return sum(1 for p in pads if not off_board(p.global_x, p.global_y))


def filter_routable_nets(pcb_data: PCBData, net_ids: List[Tuple[str, int]]
                         ) -> List[Tuple[str, int]]:
    """Drop nets with <2 ON-BOARD pads from a [(name, id), ...] routing list and
    warn LOUDLY, listing them -- a 0/1-endpoint net can never complete a
    connection, so attempting it only wastes the router (bus_pirate5 ground for
    ~3h on nets whose pads were all read as off-board). Counts on-board pads so
    it catches BOTH genuinely <2-pad nets AND nets whose pads are off-board /
    outside a mis-parsed outline. Returns the kept list."""
    from check_drc import make_off_board_test
    off_board = make_off_board_test(pcb_data.board_info)
    skipped = [(nm, nid) for (nm, nid) in net_ids
               if routable_pad_count(pcb_data, nid, off_board) < 2]
    if skipped:
        print("\n" + "=" * 64)
        print(f"WARNING: {len(skipped)} net(s) have fewer than 2 routable (on-board) "
              f"pads and CANNOT be routed -- skipping them (a connection needs "
              f">=2 endpoints):")
        for nm, nid in skipped:
            total = len(pcb_data.pads_by_net.get(nid, ()))
            onb = routable_pad_count(pcb_data, nid, off_board)
            extra = f" ({total-onb} off-board)" if onb < total else ""
            print(f"    - {nm} (net {nid}): {onb} on-board pad(s){extra}")
        print("  If a net you expected to route is here, its pads may be off-board "
              "or the Edge.Cuts outline mis-parsed -- check the board outline.")
        print("=" * 64)
        skip_ids = {nid for _nm, nid in skipped}
        return [(nm, nid) for (nm, nid) in net_ids if nid not in skip_ids]
    return net_ids


def log_net_health(pcb_data: PCBData, log=print) -> Tuple[int, int, int]:
    """Emit one warning line per problematic net -- for the GUI Log tab (and CLI).

    Flags, per net (skipping KiCad's `unconnected-*` single-pin nets):
      * fewer than 2 ON-BOARD pads  -> unroutable (a route needs >=2 endpoints);
      * some (but not all) pads off the board edge -> likely a placement mistake.
    Also emits board-level parse warnings first: no Edge.Cuts outline, or an
    outline so mis-parsed that most pads read as off-board (the bus_pirate5 class,
    where a bad inner contour made every pad look off-board). Call at board load
    so issues surface before routing. Returns (n_unroutable, n_offboard, n_parse).
    """
    from check_drc import make_off_board_test
    bi = pcb_data.board_info
    n_parse = 0
    if not getattr(bi, 'board_bounds', None):
        n_parse += 1
        log("WARNING: board outline (Edge.Cuts) did not parse -- board bounds "
            "unknown; off-board / edge checks unavailable.")
    off_board = make_off_board_test(bi)
    all_pads = [(p.global_x, p.global_y)
                for pads in pcb_data.pads_by_net.values() for p in pads]
    if off_board and all_pads:
        frac = sum(1 for x, y in all_pads if off_board(x, y)) / len(all_pads)
        if frac > 0.4:
            n_parse += 1
            log(f"WARNING: {frac*100:.0f}% of pads read as OFF-BOARD -- the Edge.Cuts "
                f"outline is probably mis-parsed (bad cutout / open contour); the "
                f"per-net warnings below may be spurious until the outline is fixed.")

    n_unroutable = n_offboard = 0
    for nid, net in pcb_data.nets.items():
        if nid <= 0 or not net.name or net.name.lower().startswith('unconnected-'):
            continue
        pads = pcb_data.pads_by_net.get(nid, [])
        n_off = sum(1 for p in pads if off_board(p.global_x, p.global_y)) if off_board else 0
        n_on = len(pads) - n_off
        if n_on < 2:
            n_unroutable += 1
            extra = f" ({n_off} off-board)" if n_off else ""
            log(f"WARNING: net '{net.name}' has {n_on} on-board pad(s) of "
                f"{len(pads)}{extra} -- cannot be routed (needs >=2 endpoints).")
        elif n_off:
            n_offboard += 1
            log(f"WARNING: net '{net.name}' has {n_off}/{len(pads)} pad(s) off the "
                f"board edge.")
    return n_unroutable, n_offboard, n_parse


def expand_net_patterns(pcb_data: PCBData, patterns: List[str],
                        exclude_unconnected: bool = True) -> List[str]:
    """
    Expand wildcard patterns to matching net names.

    Patterns can include * and ? wildcards (fnmatch style).
    Example: "Net-(U2A-DATA_*)" matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.

    Patterns starting with "!" are exclusion patterns - they remove matching
    nets from the result. Process order matters: include patterns add nets,
    exclude patterns remove them.
    Example: "*" "!GND" "!VCC" - all nets except GND and VCC

    A "!"-pattern that verbatim names an existing net is treated as a literal
    inclusion of that (active-low) net, and "\\!FOO" is an escaped literal "!FOO",
    so active-low nets like !RESET / !MIX_BYPASS stay routable by name (issue #177).

    Args:
        pcb_data: PCB data with nets and pads
        patterns: List of net name patterns (may include wildcards)
        exclude_unconnected: If True (default), exclude "unconnected-*" nets

    Returns list of unique net names in sorted order for patterns,
    preserving order of non-pattern names.
    """
    # Collect net names from both pcb.nets and pads_by_net
    all_net_names = set(net.name for net in pcb_data.nets.values() if net.name)
    # Also include net names from pads (for nets not in pcb.nets)
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.net_name:
                all_net_names.add(pad.net_name)
                break  # Only need one pad's net_name per net

    # Filter out unconnected nets (KiCad pins not connected in schematic)
    # and empty net names. Literal-name lookups ("does not exist" check,
    # did-you-mean) still see the unfiltered set, so explicitly naming an
    # 'unconnected-*' net works without a spurious warning (#513 item 7).
    unfiltered_net_names = set(all_net_names)
    if exclude_unconnected:
        all_net_names = {name for name in all_net_names
                        if name and not name.lower().startswith('unconnected-')}
        # #513 item 7: an 'unconnected-*' net with >=2 pads is NOT always a
        # true no-connect -- reversible footprints (XIAO, ProMicro) get their
        # doubled pin positions auto-named this way and the pads DO need a
        # trace (klein_kb shipped 4 such nets open). But USB-shield tabs share
        # the same shape and are correctly left unrouted (#479, joined by the
        # connector shell) -- so warn with names rather than auto-route.
        _multi_nc = []
        for _nid, _net in pcb_data.nets.items():
            _nm = _net.name or ''
            if not _nm.lower().startswith('unconnected-'):
                continue
            _np = len(pcb_data.pads_by_net.get(_nid, []))
            if _np >= 2:
                _multi_nc.append((_nm, _np))
        if _multi_nc:
            _lst = ', '.join(f"'{n}' ({c} pads)" for n, c in sorted(_multi_nc)[:8])
            print(f"WARNING: {len(_multi_nc)} 'unconnected-*' net(s) with >=2 pads "
                  f"excluded from the wildcard: {_lst}. If these are a reversible "
                  f"footprint's doubled pins they DO need a trace -- route them "
                  f"explicitly by naming them in --nets. Shield/mounting tabs "
                  f"joined mechanically can stay unrouted (#513 item 7).")

    known_net_names = unfiltered_net_names
    all_net_names = list(all_net_names)

    # #513 item 19: an exclusion-ONLY pattern list means "everything else".
    # A bare "!GND" used to match ZERO nets ("No nets matched the given
    # patterns!") instead of the obvious intent -- imply the '*' inclusion.
    def _is_exclusion(rp):
        return (rp.startswith('!') and not rp.startswith('\\!')
                and rp not in known_net_names)
    if patterns and all(_is_exclusion(rp) for rp in patterns):
        print("Exclusion-only net patterns given: implying '*' "
              "(everything except the exclusions)")
        patterns = ['*'] + list(patterns)
    result = []
    seen = set()
    excluded = set()

    def _did_you_mean(name: str) -> str:
        """Suggest real nets a mistyped literal probably meant - hierarchical
        names carry a leading slash ('/GND'), which users routinely drop, and
        a silently no-op exclusion defeats the coverage invariant (issue #292:
        core1106_cam routed its plane nets as traces because '!GND' matched
        nothing while the board's net is '/GND')."""
        want = name.lstrip('/')
        close = sorted(n for n in known_net_names
                       if n.lstrip('/') == want or n.rsplit('/', 1)[-1] == want)
        return f" (did you mean {', '.join(repr(c) for c in close[:3])}?)" if close else ""

    for raw_pattern in patterns:
        # Classify the pattern: exclusion ("!FOO"), escaped literal ("\!FOO"), or
        # plain include. A "!FOO" that verbatim names a real net is a literal
        # active-low include, not an exclusion (issue #177).
        is_exclude = False
        if raw_pattern.startswith('\\!'):
            pattern = raw_pattern[1:]                      # escaped literal "!FOO"
        elif raw_pattern.startswith('!') and raw_pattern not in known_net_names:
            is_exclude = True
            pattern = raw_pattern                          # handled in exclusion branch
        else:
            pattern = raw_pattern                          # include (covers literal "!FOO" nets)

        # Check for exclusion pattern (starts with !)
        # Both branches below resolve through net_pattern_matches, so the
        # unqualified spelling ('!GND') excludes the board's sheet-qualified
        # '/GND' instead of silently matching nothing (#292/#493).
        if is_exclude:
            exclude_pattern = pattern[1:]  # Remove the !
            matches = [name for name in all_net_names
                       if net_pattern_matches(name, exclude_pattern)]
            is_wildcard = '*' in exclude_pattern or '?' in exclude_pattern
            if matches:
                label = (f"Exclusion pattern '!{exclude_pattern}'" if is_wildcard
                         else f"Exclusion '!{exclude_pattern}'")
                print(f"{label} matched {len(matches)} net(s): "
                      f"{', '.join(sorted(matches)[:5])}"
                      f"{' ...' if len(matches) > 5 else ''}")
                # #513 item 19: the #292 trailing-path-component heuristic can
                # sweep up a DISTINCT net that merely shares the leaf name --
                # wrass_audio_card's '!GND' also excluded '/Expansion/GND' (a
                # separate 2-pad net, never tied to main GND), which then
                # silently got zero copper. Warn when an unqualified literal
                # exclusion matched extra differently-qualified nets.
                if not is_wildcard:
                    _exact = {exclude_pattern, '/' + exclude_pattern.lstrip('/')}
                    _others = [m for m in matches if m not in _exact
                               and m.lstrip('/') != exclude_pattern.lstrip('/')]
                    if _others:
                        print(f"  WARNING: unqualified exclusion "
                              f"'!{exclude_pattern}' ALSO excluded "
                              f"{len(_others)} distinct net(s) sharing the "
                              f"leaf name: {', '.join(sorted(_others)[:5])}"
                              f"{' ...' if len(_others) > 5 else ''}. If any "
                              f"is a separate net that needs routing, exclude "
                              f"by fully-qualified name instead (#513 item 19).")
            else:
                label = ("Exclusion pattern" if is_wildcard else "Exclusion")
                print(f"WARNING: {label} '!{exclude_pattern}' matched no nets"
                      f"{_did_you_mean(exclude_pattern)}")
            if not is_wildcard:
                # Keep the literal in `excluded` too, so a later literal include
                # of the same spelling stays suppressed even when it named no net.
                excluded.add(exclude_pattern)
            for name in matches:
                excluded.add(name)
                if name in seen:
                    result.remove(name)
                    seen.remove(name)
        else:
            matches = sorted([name for name in all_net_names
                              if net_pattern_matches(name, pattern)
                              and name not in excluded])
            if '*' in pattern or '?' in pattern:
                if not matches:
                    print(f"Warning: Pattern '{pattern}' matched no nets")
                else:
                    print(f"Pattern '{pattern}' matched {len(matches)} nets")
                for name in matches:
                    if name not in seen:
                        result.append(name)
                        seen.add(name)
            elif matches:
                # Literal that resolves to real net(s) -- including the
                # sheet-qualified form of an unqualified spelling.
                for name in matches:
                    if name not in seen:
                        result.append(name)
                        seen.add(name)
            elif pattern in known_net_names:
                # Literal naming a real net that the wildcard pool filtered out
                # (an 'unconnected-*' no-connect): explicit naming includes it
                # without the spurious does-not-exist warning (#513 item 7).
                if pattern not in seen and pattern not in excluded:
                    result.append(pattern)
                    seen.add(pattern)
            else:
                # Literal naming no net: preserved as-is (callers may pass names
                # this board does not carry) with the long-standing warning.
                if pattern not in seen and pattern not in excluded:
                    result.append(pattern)
                    seen.add(pattern)
                    print(f"WARNING: Net '{pattern}' does not exist on this board"
                          f"{_did_you_mean(pattern)}")

    return result


# --------------------------------------------------------- component filters

_REF_GLOB_CHARS = '*?['


@dataclass
class ComponentNetSelection:
    """What :func:`nets_for_components` resolved a set of ref patterns to.

    `unmatched_patterns` is the field that exists so a typo'd reference can be
    reported as such: selecting nets by component and getting nothing back is
    otherwise indistinguishable from "this component has no routable nets".
    """
    net_names: List[str]           # selected nets, sorted
    net_ids: List[int]             # the same nets as ids, sorted
    matched_refs: List[str]        # footprint references the patterns matched
    unmatched_patterns: List[str]  # patterns that matched no footprint at all
    excluded_names: List[str]      # nets dropped by `exclude_patterns`, sorted


def component_ref_matches(ref: str, pattern: str, match: str = 'glob') -> bool:
    """Match one footprint reference against one pattern.

    Both modes send a pattern carrying an fnmatch metacharacter (``*?[``) through
    fnmatch. They differ only in what a BARE token means:

      ``glob``       bare token is an EXACT reference -- 'U1' does NOT match 'U10'
      ``substring``  bare token is a substring        -- 'U1' matches 'U10', 'U100'

    ``substring`` is the GUI Comp Filter's long-standing behaviour, and it is the
    right reading there: it narrows a visible list as you type. ``glob`` is what
    the CLI has always done, where the filter decides what copper gets placed and
    a silently-included extra footprint is a real hazard.

    Matching is case-insensitive, via ``fnmatchcase`` on upper-cased inputs so a
    pattern resolves identically on Windows (where plain ``fnmatch`` case-folds
    against the platform rules) and on POSIX (where it does not).
    """
    if not ref or not pattern:
        return False
    ref_u, pat_u = ref.upper(), pattern.upper()
    if any(c in pattern for c in _REF_GLOB_CHARS):
        return fnmatch.fnmatchcase(ref_u, pat_u)
    if match == 'substring':
        return pat_u in ref_u
    return ref_u == pat_u


def suggest_component_refs(refs, pattern: str, limit: int = 3) -> str:
    """" (did you mean 'U3', 'U4'?)" for a reference pattern that matched nothing."""
    if not pattern or not refs:
        return ""
    close = difflib.get_close_matches(pattern.upper(),
                                      {r.upper(): r for r in refs}, n=limit, cutoff=0.6)
    by_upper = {r.upper(): r for r in refs}
    names = [by_upper[c] for c in close if c in by_upper]
    return f" (did you mean {', '.join(repr(n) for n in names)}?)" if names else ""


def nets_for_components(pcb_data: PCBData,
                        patterns,
                        *,
                        mode: str = 'any',
                        match: str = 'glob',
                        exclude_patterns: Optional[List[str]] = None
                        ) -> ComponentNetSelection:
    """Nets touching the footprints matched by `patterns` (issue #537).

    This is the single implementation of "the nets of these components". It
    replaced four divergent ones that disagreed about the only question that
    matters -- whether 'U1' also means U10, U12 and U100 -- so the same request
    selected different nets on the CLI, in the GUI, and in a replayed plan.
    Callers pick the policy explicitly via `match` (see
    :func:`component_ref_matches`) instead of inheriting whichever loop they
    happened to be written next to.

    mode:
      ``any``       net has >=1 pad on a matched footprint (the historical
                    behaviour of every call site, and the default)
      ``between``   net reaches >=2 DISTINCT matched footprints -- the wires
                    running between the selected parts. Note this is stricter
                    than ">=2 matched pads": two pads of one net on a single
                    matched footprint run between nothing.
      ``internal``  EVERY pad of the net is on a matched footprint -- the nets
                    that do not leave the selected block. Beware that this is
                    near-empty for a SINGLE selected footprint, which reads as a
                    bug rather than as the correct answer it is.

    `exclude_patterns` (e.g. POWER_NET_EXCLUSION_PATTERNS) drops matching nets
    from the result and reports them in `excluded_names`, so a caller can say
    what it dropped rather than dropping it silently.
    """
    if isinstance(patterns, str):
        patterns = [patterns]
    patterns = [p.strip() for p in (patterns or []) if p and p.strip()]
    if mode not in ('any', 'between', 'internal'):
        raise ValueError(f"nets_for_components: bad mode {mode!r}")
    if match not in ('glob', 'substring'):
        raise ValueError(f"nets_for_components: bad match {match!r}")
    if not patterns:
        return ComponentNetSelection([], [], [], [], [])

    # Every reference the board carries. `footprints` is the authoritative list,
    # but pads carry `component_ref` independently, so union both -- a pattern is
    # only "matched nothing" if it misses everything a pad could name.
    all_refs = set(pcb_data.footprints or {})
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.component_ref:
                all_refs.add(pad.component_ref)

    matched_refs: Set[str] = set()
    unmatched: List[str] = []
    for pattern in patterns:
        hits = {r for r in all_refs if component_ref_matches(r, pattern, match)}
        if hits:
            matched_refs |= hits
        else:
            unmatched.append(pattern)

    selected: List[Tuple[str, int]] = []
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id <= 0 or not pads:
            continue
        on_matched = [p for p in pads if p.component_ref in matched_refs]
        if not on_matched:
            continue
        if mode == 'between':
            if len({p.component_ref for p in on_matched}) < 2:
                continue
        elif mode == 'internal':
            if len(on_matched) != len(pads):
                continue
        net = pcb_data.nets.get(net_id)
        name = (net.name if net and net.name else None) or pads[0].net_name
        if name:
            selected.append((name, net_id))

    excluded: List[str] = []
    if exclude_patterns:
        kept = []
        for name, net_id in selected:
            if any(p and fnmatch.fnmatchcase(name.upper(), p.upper())
                   for p in exclude_patterns):
                excluded.append(name)
            else:
                kept.append((name, net_id))
        selected = kept

    return ComponentNetSelection(
        net_names=sorted({n for n, _ in selected}),
        net_ids=sorted({i for _, i in selected}),
        matched_refs=sorted(matched_refs),
        unmatched_patterns=unmatched,
        excluded_names=sorted(set(excluded)),
    )


def identify_power_nets(pcb_data: PCBData,
                        patterns: List[str],
                        widths: List[float]) -> Dict[int, float]:
    """
    Identify power nets and map them to track widths based on pattern matching.

    Each pattern in `patterns` corresponds to a width in `widths` at the same index.
    Nets are matched against patterns in order; the first matching pattern determines
    the width. This allows priority ordering (e.g., specific nets before wildcards).

    Args:
        pcb_data: Parsed PCB data with net information
        patterns: Glob patterns for power nets (e.g., ['*GND*', '*VCC*', '+*V'])
        widths: Track widths in mm corresponding to each pattern

    Returns:
        Dict mapping net_id to track width for matched nets

    Example:
        identify_power_nets(pcb, ['*GND*', '*VCC*'], [0.4, 0.5])
        # Returns {5: 0.4, 12: 0.4, 7: 0.5} if nets 5,12 match GND and 7 matches VCC
    """
    if len(patterns) != len(widths):
        raise ValueError(f"patterns ({len(patterns)}) and widths ({len(widths)}) must have same length")

    power_net_widths: Dict[int, float] = {}

    # Collect all net names and IDs
    for net_id, net in pcb_data.nets.items():
        if not net.name or net_id == 0:
            continue

        # Check patterns in order - first match wins
        for pattern, width in zip(patterns, widths):
            if fnmatch.fnmatch(net.name, pattern):
                power_net_widths[net_id] = width
                break

    return power_net_widths


# Ground-net family recognized for GND return-via placement (issue #379). A
# board's ground is spelled many ways; an exact 'GND' match silently disabled
# --gnd-vias on '/GND' (KiCad's hierarchical sheet-path form), 'GNDA', 'DGND',
# 'GND_D', 'GND1', etc. These suffix-style names (A/D/P + GND) do not start with
# 'GND', so they need an explicit set alongside the 'GND*' prefix test.
_GND_SUFFIX_NAMES = frozenset({'AGND', 'DGND', 'PGND'})


def _gnd_base(name: str) -> str:
    """Net name normalized for GND matching: hierarchical sheet-path prefix
    dropped ('/sheet/GND' -> 'GND', '/GND' -> 'GND'), upper-cased."""
    return name.rsplit('/', 1)[-1].upper()


def is_ground_net_name(name: str) -> bool:
    """True if a net name is in the GND family (GND, /GND, GNDA, AGND, DGND,
    PGND, GND_D, GND1, ...); the sheet-path prefix is stripped first (#379).

    KNOWN and deliberately not widened: this does NOT match `VSS`, while
    `list_nets.find_power_nets` calls VSS ground. No tracked board names a net
    `VSS*`, so the divergence is structurally untestable here -- and widening a
    predicate five other call sites already depend on, to fix a case no fixture
    can exercise, is how the "second copy drifts" lesson gets earned rather than
    read. Consumers that care must say which spelling they mean.
    """
    b = _gnd_base(name)
    return b.startswith('GND') or b in _GND_SUFFIX_NAMES


#: Pin-FUNCTION names that mean "supply" (#705). Lifted verbatim from the
#: closure that used to live inside `analyze_power_paths.get_power_net_
#: recommendations`, where it was unreachable to every other caller -- so the
#: placement side had no pin-level power predicate at all and issue #705's
#: channel 2 would have had to copy it. It is a power-AND-GROUND table: it was
#: written for a current-summing tool that wanted both sides of the supply, and
#: a caller that means "rail, not return" must exclude ground itself.
POWER_PIN_KEYWORDS = ('VCC', 'VDD', 'VSS', 'GND', 'VCCA', 'VSSA', 'VDDA',
                      'VDDPLL', 'VCCPLL', 'GNDPLL', 'VRH', 'VRL', 'AVDD',
                      'AVSS')


def is_supply_pintype(pintype: str) -> bool:
    """Is this pad's `pintype` a supply pin the tool should grade?

    Token-split, and BOTH halves are load-bearing. KiCad writes compound
    pintypes, and every compound spelling in the tracked corpus is
    `X+no_connect` -- 557 `passive+no_connect`, 63 `tri_state+no_connect`, 3
    `power_in+no_connect`. So:

    * `pintype in ('power_in', 'power_out')` DROPS `power_in+no_connect`, which
      is the right answer for the wrong reason -- it would drop a
      `power_in+anything_else` too;
    * `pintype.startswith('power_in')` GRADES it, which is wrong: a pin the
      designer marked no-connect is uncovered by definition and would be
      reported forever.

    Measured, that one pad is the whole difference between "12 of 12 covered"
    and "1 uncovered" on `glasgow_revC` U30 (pad B10, `VPP_FAST`), the cleanest
    board in the corpus -- so the decision deserves a named predicate rather
    than being a side effect of `in` versus `startswith`.
    """
    tokens = (pintype or '').split('+')
    return (('power_in' in tokens or 'power_out' in tokens)
            and 'no_connect' not in tokens)


def is_supply_pinfunction(pinfunction: str, keywords=None) -> bool:
    """Is this pad's `pinfunction` a supply name? Exact match or prefix.

    Prefix is load-bearing: real boards spell the pin `VCC_14`, `GND_7`,
    `VCC_16`. `keywords` overrides the default table wholesale (the intent's
    `decaps.pin_functions`), because the failure mode is a DEFAULT entry that is
    wrong for a board and an add-only override cannot remove one.
    """
    fn = (pinfunction or '').upper()
    if not fn:
        return False
    kws = POWER_PIN_KEYWORDS if keywords is None else tuple(keywords)
    return any(fn == kw or fn.startswith(kw) for kw in kws)


def is_power_pin(pinfunction: str, pintype: str, keywords=None) -> bool:
    """Either channel says supply. The form `analyze_power_paths` re-exports.

    Behaviour-preserving against the closure this replaces on every pad of
    every tracked board: the old form tested `pintype in ('power_in',
    'power_out')`, and the two disagree only on a compound pintype that is not
    `X+no_connect`, of which the corpus has none.
    """
    return (is_supply_pintype(pintype)
            or is_supply_pinfunction(pinfunction, keywords))


#: Net-NAME shapes that mean power or ground. Moved here from
#: `placement.groups._POWER_NET`, unchanged, so the pin rule's rail-net fallback
#: and the block-prefix grouper cannot drift apart.
#:
#: It is a PREFIX test on the sheet-path leaf, and it is loose on purpose for
#: its original job (rejecting a net as a block-name source). Read as a supply
#: test it over-matches -- the `\\d` and `pwr` and `vref` alternatives make
#: `40M_P`, `32K_N`, `2V5_3V3` and `VREF` all "power" -- and it UNDER-matches
#: the rails that matter most: it does not match `RAM_VDD`, `RAM_VDDQ` or
#: `IOVDD`, because those do not START with a supply token. Measured, that
#: under-match is why `orangecrab_ext_pll` abstains instead of grading 17 chips
#: on ground, which is the true answer -- so it is disclosed, not widened.
_POWER_NET_RE = re.compile(r'^(a?gnd|dgnd|pgnd|vcc|vdd|vss|vee|vbus|vin|vout|'
                           r'vref|pwr|\+|-|\d)', re.I)


def is_power_net_name(name: str) -> bool:
    """True if a net name LOOKS like a rail or a return, by its leaf."""
    leaf = (name or '').split('/')[-1]
    return bool(leaf) and bool(_POWER_NET_RE.match(leaf))


def resolve_gnd_net_id(pcb_data: PCBData,
                       preferred_name: Optional[str] = None
                       ) -> Tuple[Optional[int], Optional[str]]:
    """Resolve the board's ground net for GND return-via placement (issue #379).

    Returns ``(net_id, net_name)``, or ``(None, None)`` if no ground net is
    found. Priority: an explicit ``preferred_name`` (e.g. ``--gnd-via-net``,
    matched ignoring the sheet-path prefix) > an exact 'GND' > the GND family,
    picking the shortest family name (then lowest net id) so a plain 'GND' beats
    'GNDA'/'GNDD' deterministically. net 0 (unconnected) is never a ground net.
    """
    nets = [(nid, net) for nid, net in pcb_data.nets.items()
            if nid != 0 and net.name]
    if preferred_name:
        want = _gnd_base(preferred_name)
        for nid, net in nets:
            if _gnd_base(net.name) == want:
                return nid, net.name
    exact = sorted((nid, net) for nid, net in nets if _gnd_base(net.name) == 'GND')
    if exact:
        return exact[0][0], exact[0][1].name
    fam = [(nid, net) for nid, net in nets if is_ground_net_name(net.name)]
    if fam:
        fam.sort(key=lambda kv: (len(kv[1].name), kv[0]))
        return fam[0][0], fam[0][1].name
    return None, None


def _ground_family_net_ids(pcb_data: PCBData) -> List[Tuple[int, str]]:
    """[(net_id, name)] of every ground-family net, deterministically ordered."""
    fam = [(nid, net.name) for nid, net in pcb_data.nets.items()
           if nid != 0 and net.name and is_ground_net_name(net.name)]
    fam.sort(key=lambda kv: (len(kv[1]), kv[1], kv[0]))
    return fam


def ground_domain_bridges(pcb_data: PCBData) -> List[Dict]:
    """Components that deliberately TIE two ground nets together.

    A split-ground board joins its domains at exactly one point, through a
    ferrite, a 0 Ohm link or a net tie. Such a part touches BOTH grounds, so it
    must not be used to decide which domain a signal returns to -- otherwise the
    two domains look like one through it.

    Detected structurally (no datasheet needed): a populated part with exactly
    two electrically-distinct pads, one on each of two different ground-family
    nets. Net ties are included via footprint.net_tie_groups. Returns
    [{'ref', 'net_ids'}].
    """
    fam_ids = {nid for nid, _ in _ground_family_net_ids(pcb_data)}
    bridges = []
    for ref, fp in sorted(pcb_data.footprints.items()):
        if fp.dnp:
            continue  # a no-pop link is an OPEN: it bridges nothing
        pad_nets = {p.net_id for p in fp.pads if p.net_id}
        if len(pad_nets) != 2:
            continue
        if pad_nets <= fam_ids:
            bridges.append({'ref': ref, 'net_ids': sorted(pad_nets)})
    return bridges


def resolve_ground_domains(pcb_data: PCBData) -> Dict[int, Set[str]]:
    """{ground net_id: {component refs returning to it}} (#489 §5).

    `resolve_gnd_net_id` resolves ONE board-global ground, so on a split-ground
    board with AGND + DGND and no plain 'GND' it picks one arbitrarily and the
    return-via features stitch every signal via -- analog included -- to that one
    net. That is worse than doing nothing: it can bridge the very domains the
    designer built the split to separate, while connectivity and DRC both stay
    green.

    Each ground-family net is a domain, seeded with the components that touch it.
    Bridge parts (see ground_domain_bridges) are excluded so a domain's membership
    does not leak across the split. A component touching two domains directly
    (not through a bridge) is left out of both -- it is genuinely ambiguous, and
    guessing is what caused the bug.
    """
    cached = getattr(pcb_data, '_ground_domains', None)
    if cached is not None:
        return cached

    fam = _ground_family_net_ids(pcb_data)
    bridge_refs = {b['ref'] for b in ground_domain_bridges(pcb_data)}

    touches: Dict[str, Set[int]] = {}
    for nid, _name in fam:
        for pad in pcb_data.pads_by_net.get(nid, ()) or ():
            ref = pad.component_ref
            if not ref or ref in bridge_refs:
                continue
            touches.setdefault(ref, set()).add(nid)

    domains: Dict[int, Set[str]] = {nid: set() for nid, _ in fam}
    for ref, nids in touches.items():
        if len(nids) == 1:
            domains[next(iter(nids))].add(ref)
    pcb_data._ground_domains = domains
    return domains


def resolve_return_net_id(pcb_data: PCBData, net_id: Optional[int] = None,
                          preferred_name: Optional[str] = None
                          ) -> Tuple[Optional[int], Optional[str]]:
    """The ground net a given signal net should return to (#489 §5).

    Same contract as `resolve_gnd_net_id`, and IDENTICAL to it whenever the board
    has 0 or 1 ground domains (the overwhelmingly common case) or the caller
    passes `preferred_name` (--gnd-via-net still wins outright).

    With several ground domains, the signal's own endpoint components decide:
    the domain shared by the components its pads sit on. Ambiguous or unknown
    falls back to the board-global answer, so behaviour never gets worse than
    before -- it just stops being confidently wrong on the boards where grounding
    matters most.
    """
    if preferred_name:
        return resolve_gnd_net_id(pcb_data, preferred_name)

    domains = resolve_ground_domains(pcb_data)
    if net_id is None or len([d for d in domains if domains[d]]) < 2:
        return resolve_gnd_net_id(pcb_data)

    refs = {p.component_ref for p in pcb_data.pads_by_net.get(net_id, ()) or ()
            if p.component_ref}
    if not refs:
        return resolve_gnd_net_id(pcb_data)

    hits = [nid for nid, members in sorted(domains.items()) if refs & members]
    if len(hits) == 1:
        net = pcb_data.nets.get(hits[0])
        return hits[0], (net.name if net else None)
    return resolve_gnd_net_id(pcb_data)


def describe_ground_domains(pcb_data: PCBData,
                            preferred_name: Optional[str] = None
                            ) -> Optional[str]:
    """A one-shot warning when a board has SEVERAL ground domains and the user
    has not disambiguated, or None when there is nothing to say (#489 §5)."""
    if preferred_name:
        return None
    domains = {nid: refs for nid, refs in resolve_ground_domains(pcb_data).items() if refs}
    if len(domains) < 2:
        return None
    parts = []
    for nid, refs in sorted(domains.items(), key=lambda kv: -len(kv[1])):
        net = pcb_data.nets.get(nid)
        parts.append(f"{net.name if net else nid} ({len(refs)} components)")
    bridges = ground_domain_bridges(pcb_data)
    bridge_txt = ""
    if bridges:
        bridge_txt = (" Tied by " +
                      ", ".join(b['ref'] for b in bridges[:4]) +
                      (", ..." if len(bridges) > 4 else "") + ".")
    return (f"NOTE: {len(domains)} ground domains found: {'; '.join(parts)}."
            f"{bridge_txt} Return vias are matched to each signal's own domain "
            f"from its endpoint components; pass --gnd-via-net to force one net "
            f"for the whole board.")


def gnd_candidate_names(pcb_data: PCBData) -> List[str]:
    """Ground-ish net names on the board, for a 'no GND net found' diagnostic."""
    return sorted({net.name for nid, net in pcb_data.nets.items()
                   if nid != 0 and net.name
                   and ('GND' in net.name.upper()
                        or _gnd_base(net.name) in _GND_SUFFIX_NAMES)})


def extract_diff_pair_base(net_name: str) -> Optional[Tuple[str, bool, str]]:
    """
    Extract differential pair base name and polarity from net name.

    Looks for common differential pair naming conventions:
    - name_P / name_N
    - nameP / nameN
    - name+ / name-
    - name_t / name_c (true/complement, common for DDR; case-insensitive)
    - name_t_X / name_c_X (true/complement with channel suffix, e.g., DQS0_t_A)
    - name_TA / name_CA (true/complement, no separator before channel char)
    - nameDP / nameDM, nameDPLUS / nameDMINUS (USB data lines)

    Returns (base_name, is_positive, style) or None if not a diff pair.
    style identifies the suffix convention ('_t', '_P', 'P', '+', 'DP') - nets
    only form a pair when both use the same convention, so e.g. /CLK+ does not
    pair with an unrelated /CLK_N net.
    """
    import re

    if not net_name:
        return None

    # KiCad auto-names a netless pin's net 'Net-(<ref>-<pin>)'. The trailing ')'
    # buries the polarity suffix (e.g. Net-(U12-USB_D+) ends in '+)'), so peel a
    # matching wrapper before applying the suffix rules (issue #91, bitaxe USB).
    is_auto_name = net_name.startswith('Net-(') and net_name.endswith(')')
    if is_auto_name:
        net_name = net_name[5:-1]

    # In a 'Net-(<ref>-<pin path>)' auto-name the pin path encodes the chip's
    # internal signal path, e.g. 'U12-GPIO19/U1RTS/.../USB_D-'. The diff marker
    # lives in the final path segment, but the per-pin prefix differs between the
    # two halves (GPIO19 vs GPIO20), so the full path never pairs. Use the leaf
    # so USB_D+/USB_D- pair (issue #181). Only for auto-names: user-named
    # hierarchical nets like '/bank1/CLK_N' must keep their full path so they
    # don't pair across banks. KiCad escapes '/' in net names as '{slash}', so
    # split on either form.
    if is_auto_name:
        path = net_name.replace('{slash}', '/')
        if '/' in path:
            net_name = path.rsplit('/', 1)[-1]

    # NOTE (issue #192): the DDR true/complement (_t_/_c_) rules are checked LAST,
    # AFTER the explicit polarity-suffix rules below (+/-, _P/_N, P/N, USB,
    # indexed). A `_t_`/`_c_` can appear mid-name as a section/channel letter
    # (e.g. TARGET_C_SENSE+), and a real polarity suffix must win over it -- else
    # the infix is misread as DDR complement and both halves key to different
    # bases with the same polarity, so the pair is silently dropped.

    # USB data lines: DPLUS / DMINUS (case-insensitive). The D is kept in the
    # base so the two halves pair. Reject a letter before the D (e.g. an
    # unrelated word ending in 'dplus') so this only fires on real USB names.
    dpm_match = re.match(r'^(.*?)D(PLUS|MINUS)$', net_name, re.IGNORECASE)
    if dpm_match and (not dpm_match.group(1) or not dpm_match.group(1)[-1].isalpha()):
        return (dpm_match.group(1) + 'D', dpm_match.group(2).upper() == 'PLUS', 'DP')

    # USB data lines: DP / DM / DN (case-insensitive, e.g. USB_DP / USB_DM,
    # USB_DP / USB_DN, D+ aliases). P is positive; M and N are negative (issue
    # #143: tigard names its USB pair /USB_DP + /USB_DN). Same letter-boundary
    # guard so words like 'LCDP' or 'SDN' don't match.
    dpm_match = re.match(r'^(.*?)D([PMN])$', net_name, re.IGNORECASE)
    if dpm_match and (not dpm_match.group(1) or not dpm_match.group(1)[-1].isalpha()):
        return (dpm_match.group(1) + 'D', dpm_match.group(2).upper() == 'P', 'DP')

    # Indexed P/N pair: name_P0/name_N0, name_P1/name_N1 (issue #143: daisho's
    # FE_CLK pairs CLK_P0/CLK_N0). Keep the trailing index in the base so each
    # index pairs only with its own twin, never across indices.
    idx_match = re.match(r'^(.+)_([PN])(\d+)$', net_name)
    if idx_match:
        base = idx_match.group(1) + '_X' + idx_match.group(3)
        return (base, idx_match.group(2) == 'P', '_P')

    # Try _P/_N suffix (most common for LVDS)
    if net_name.endswith('_P'):
        return (net_name[:-2], True, '_P')
    if net_name.endswith('_N'):
        return (net_name[:-2], False, '_P')

    # Try P/N suffix without underscore. Accept a digit, underscore, OR uppercase
    # letter before the final P/N so SerDes/PCIe names pair (issue #151: TXP/TXN,
    # SSRXP/SSRXN, REFCLKP/REFCLKN). A lowercase letter (e.g. WAKEn) is rejected,
    # and pairing still needs both siblings to exist, so stray names don't pair.
    if net_name.endswith('P') and len(net_name) > 1:
        if net_name[-2] in '0123456789_' or net_name[-2].isupper():
            return (net_name[:-1], True, 'P')
    if net_name.endswith('N') and len(net_name) > 1:
        if net_name[-2] in '0123456789_' or net_name[-2].isupper():
            return (net_name[:-1], False, 'P')

    # Try +/- suffix. Reject the KiCad 'Net-(<ref>-+)' / 'Net-(<ref>--)' form,
    # where the '+'/'-' is a 2-terminal passive's pad name (buzzers, etc.) sitting
    # right after the ref-pad '-' separator -- those are DC polarity terminals,
    # not a coupled signal pair (issue #181). A genuine pair is 'FOO+' / 'FOO-',
    # never 'FOO-+' / 'FOO--'.
    if net_name.endswith('+') and not net_name.endswith('-+'):
        return (net_name[:-1], True, '+')
    if net_name.endswith('-') and not net_name.endswith('--'):
        return (net_name[:-1], False, '+')

    # +/- with a suffix AFTER the sign, e.g. /D+_L and /D-_L (issue #290,
    # dilemma's split-keyboard USB pair). Only an underscore-led suffix
    # qualifies -- a bare mid-name '-' is ordinary hyphenation (3V3-MCU), not
    # polarity. Same passive-terminal guard as above ('BZ1--_L' must not pair).
    pm_match = re.match(r'^(.+?)([+-])(_[A-Za-z0-9_]*)$', net_name)
    if pm_match and pm_match.group(1)[-1] not in '+-':
        return (pm_match.group(1) + pm_match.group(3), pm_match.group(2) == '+', '+')

    # DDR true/complement, _t/_c -- checked LAST so a mid-name _t_/_c_ section
    # letter never shadows a real +/- or _P/_N pair (issue #192). Case-insensitive
    # (CK_t/CK_c, CK_T/CK_C). With an explicit channel separator: DQS0_t_A / DQS0_c_A
    tc_match = re.match(r'^(.+)_([tc])_(.+)$', net_name, re.IGNORECASE)
    if tc_match:
        base = tc_match.group(1) + '_X_' + tc_match.group(3)  # Keep suffix in base for pairing
        is_positive = tc_match.group(2).lower() == 't'
        return (base, is_positive, '_t')

    # No separator before the channel char: DQS0_TA / DQS0_CA, CK_T0 / CK_C0.
    # Requires a trailing char so plain CK_t / CK_c falls through to the next rule.
    tc_match = re.match(r'^(.+)_([tc])([A-Za-z0-9])$', net_name, re.IGNORECASE)
    if tc_match:
        base = tc_match.group(1) + '_X' + tc_match.group(3)  # Keep channel in base for pairing
        is_positive = tc_match.group(2).lower() == 't'
        return (base, is_positive, '_t')

    # Plain _t / _c suffix (DDR style, e.g., CK_t / CK_c), case-insensitive
    if net_name[-2:].lower() == '_t':
        return (net_name[:-2], True, '_t')
    if net_name[-2:].lower() == '_c':
        return (net_name[:-2], False, '_t')

    return None


def matches_diff_pair_patterns(net_name: str, base_name: str, patterns: List[str]) -> bool:
    """
    Return True if any glob pattern selects this diff-pair half.

    Matches the pattern against the full net name, its leaf (the last
    '/'-separated segment), the diff-pair base name, and the base name's leaf.
    This lets a glob that only catches one half (e.g. '*_P') or an explicit
    base name (e.g. '/DVI_CK') select the whole pair, even for hierarchical
    (slash-separated) net names where '*_P' would otherwise miss the '_N'
    sibling and a leaf/base name would never equal the full path. (issue #120)
    """
    candidates = (net_name, net_name.rsplit('/', 1)[-1],
                  base_name, base_name.rsplit('/', 1)[-1])
    return any(fnmatch.fnmatch(candidate, pattern)
               for pattern in patterns for candidate in candidates)


def find_differential_pairs(pcb_data: PCBData, patterns: List[str]) -> Dict[str, DiffPairNet]:
    """
    Find all differential pairs in the PCB matching the given glob patterns.

    A pair is selected when *either* half matches the patterns (see
    matches_diff_pair_patterns), so '*_P', '*_N', or an explicit base name all
    pull in the complete pair.

    Args:
        pcb_data: PCB data with net information
        patterns: Glob patterns for nets to treat as diff pairs (e.g., '*lvds*')

    Returns:
        Dict mapping base_name to DiffPair with complete P/N pairs
    """
    # Key by (base_name, suffix style) so nets only pair within the same naming
    # convention (e.g. /CLK+ pairs with /CLK-, never with an unrelated /CLK_N)
    pairs: Dict[Tuple[str, str], DiffPairNet] = {}
    matched_keys: Set[Tuple[str, str]] = set()

    # Collect all diff-pair halves, regardless of pattern, so a pattern that
    # only catches one half can still pull in its sibling below.
    for net_id, net in pcb_data.nets.items():
        net_name = net.name
        if not net_name or net_id == 0:
            continue

        # Try to extract diff pair info
        result = extract_diff_pair_base(net_name)
        if result is None:
            continue

        base_name, is_p, style = result
        key = (base_name, style)

        if key not in pairs:
            pairs[key] = DiffPairNet(base_name=base_name)

        if is_p:
            pairs[key].p_net_id = net_id
            pairs[key].p_net_name = net_name
        else:
            pairs[key].n_net_id = net_id
            pairs[key].n_net_name = net_name

        if matches_diff_pair_patterns(net_name, base_name, patterns):
            matched_keys.add(key)

    # Filter to only complete pairs whose key was matched, keyed by base name
    # (disambiguate with the suffix style in the unlikely case two conventions
    # share a base name)
    complete_pairs: Dict[str, DiffPairNet] = {}
    for key, pair in pairs.items():
        if key not in matched_keys:
            continue
        if not pair.is_complete:
            continue
        base_name, style = key
        name = base_name if base_name not in complete_pairs else f"{base_name}({style})"
        complete_pairs[name] = pair

    return complete_pairs


def find_single_ended_nets(
    pcb_data: PCBData,
    patterns: List[str],
    exclude_net_ids: Set[int] = None
) -> List[Tuple[str, int]]:
    """
    Find all single-ended nets matching the given glob patterns.

    Args:
        pcb_data: PCB data with net information
        patterns: Glob patterns for nets (e.g., 'Net-(U2A-BE*)')
        exclude_net_ids: Net IDs to exclude (e.g., diff pair net IDs)

    Returns:
        List of (net_name, net_id) tuples for matching single-ended nets
    """
    if exclude_net_ids is None:
        exclude_net_ids = set()

    result = []
    for net_id, net in pcb_data.nets.items():
        net_name = net.name
        if not net_name or net_id == 0:
            continue

        # Skip excluded nets (e.g., diff pair nets)
        if net_id in exclude_net_ids:
            continue

        # Check if this net matches any pattern
        matched = any(fnmatch.fnmatch(net_name, pattern) for pattern in patterns)
        if matched:
            result.append((net_name, net_id))

    return result


def expand_pad_layers(pad_layers: List[str], routing_layers: List[str]) -> List[str]:
    """
    Expand wildcard layer specifications to actual layer names.

    KiCad uses "*.Cu" to mean all copper layers for through-hole pads.
    This function expands such wildcards to the actual routing layers.

    Args:
        pad_layers: List of layer names from the pad (may include wildcards like "*.Cu")
        routing_layers: List of actual routing layer names (e.g., ["F.Cu", "In1.Cu", "B.Cu"])

    Returns:
        List of expanded layer names (no wildcards)
    """
    expanded = []
    for layer in pad_layers:
        if layer == "*.Cu":
            # Expand to all copper routing layers
            expanded.extend(routing_layers)
        elif layer in ("F&B.Cu", "F&B"):
            # KiCad's OTHER copper layer-set token: front and back only, never
            # the inners. It ends in ".Cu" but is not a layer, so the
            # `endswith` branch below used to pass it through VERBATIM -- and
            # "F&B.Cu" matches no real layer name, so every consumer scoping by
            # this silently gave such a pad NO copper layer at all: unroutable
            # to the router, invisible to the connectivity and clearance
            # scopes. The bare "F&B" spelling is accepted because
            # obstacle_map.py, plane_fill_model.py and pcb_modification.py all
            # already accept both for zone layer tokens.
            expanded.extend(["F.Cu", "B.Cu"])
        elif layer.endswith(".Cu"):
            # Regular copper layer
            expanded.append(layer)
        # Skip non-copper layers like "*.Mask", "*.Paste", etc.
    # Remove duplicates while preserving routing_layers order (deterministic)
    unique = set(expanded)
    layer_order = {layer: i for i, layer in enumerate(routing_layers)}
    # Tie-break on the layer NAME: every layer absent from routing_layers maps
    # to the same fallback rank, so they tied and fell back to `unique` order --
    # and `unique` is a set of STRINGS, whose order CPython randomizes per
    # process. Layer order feeds pad expansion and per-layer passes.
    return sorted(unique, key=lambda l: (layer_order.get(l, len(routing_layers)), l))


def get_all_unrouted_net_ids(pcb_data: PCBData) -> List[int]:
    """
    Find all net IDs in the PCB that need routing.

    A net is unrouted if it has 2+ pads and they're not all connected.
    This includes:
    1. Nets with multiple disconnected segment groups (partial routing)
    2. Nets with 2+ pads but no segments (completely unrouted)
    3. Nets with 2+ pads but only 1 segment group (stub connects to 1 pad only)
    """
    unrouted_ids = set()

    # Group segments by net ID
    net_segments: Dict[int, List[Segment]] = {}
    for seg in pcb_data.segments:
        if seg.net_id not in net_segments:
            net_segments[seg.net_id] = []
        net_segments[seg.net_id].append(seg)
    # #545 F12: via-AWARE grouping (same fix as the 'Could not find pads for
    # both stub groups' site below): without vias= a route that changes
    # layers falls apart into per-layer fragments and a fully-routed net
    # reads as 'unrouted' here (ordering/proximity-avoidance impact only).
    net_vias_by_id: Dict[int, list] = {}
    for _v in pcb_data.vias:
        net_vias_by_id.setdefault(_v.net_id, []).append(_v)

    # Check each net with 2+ pads
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == 0:  # Skip unassigned net
            continue
        if len(pads) < 2:  # Single-pad nets don't need routing
            continue

        segments = net_segments.get(net_id, [])
        if not segments:
            # No segments at all - completely unrouted
            unrouted_ids.add(net_id)
        else:
            # Check if segments form multiple disconnected groups
            groups = find_connected_groups(segments,
                                           vias=net_vias_by_id.get(net_id, []))
            if len(groups) >= 2:
                # Multiple disconnected stub groups = unrouted
                unrouted_ids.add(net_id)
            elif len(groups) == 1 and len(pads) > 1:
                # Only 1 segment group but multiple pads - stub connects to some but not all
                unrouted_ids.add(net_id)

    return list(unrouted_ids)


def get_chip_pad_positions(pcb_data: PCBData, net_ids: List[int], min_pads: int = 4) -> List[Tuple[float, float, str]]:
    """Pad positions acting as PSEUDO-STUBS for proximity avoidance: pads of
    fine-pitch chip packages (BGA/QFN/QFP) whose net has NOT yet escaped the
    pad -- the future escape needs the surrounding space, so routing is
    discouraged near it.

    #585 item 8 added two gates. Both are ON by default (f785a7e's shipped
    behaviour) and independently disablable with =0. They were briefly defaulted
    OFF on a two-board 2x2 whose spread turned out comparable to its own noise;
    two corpus A/Bs then measured ON BETTER -- by 3 nets on 148 boards, and by 2
    on 129 boards with smoothing restored (7 boards better, 4 worse). They stay
    SEPARATE knobs because the two are wildly asymmetric in reach: on spartan6's
    final board the retire gate removes 95% of emitters (667 -> 33 pads) while
    the package gate removes a third (667 -> 448).

    - `KICAD_FINE_PITCH_PSEUDO_STUBS` (default ON) -- emit only for BGA/QFN/QFP. Introduced
      to stop 4-pad capacitors and dense connectors emitting fields, reasoning
      that "coarse pads need no escape protection". But the field protects
      ARRIVAL as much as escape, and without it nets could not REACH multi-pin
      headers: spartan6_6layer went 2 -> 7 incomplete nets across this commit,
      three of four casualties on 24-pin headers H1/H5/H7 (package 'OTHER'),
      and the same class took cubesat_backplane's /H2-* nets.

      Its stacking rationale is also weaker than it looks under the shipped
      default: composition is MAX (KICAD_PROXIMITY_SUM unset), so N overlapping
      fields cost the max, not the sum -- a connector cannot inflate cost by
      concentration unless an opt-in sum/zoned/softcap mode is selected.

    - `KICAD_PSEUDO_STUB_RETIRE` (default ON) -- drop the proxy once same-net copper reaches
      the pad, on the grounds that the real stub endpoint takes over as the
      proximity signal. Plausible, but it retires on ANY attachment: a header
      pad with one stub on it stops defending the corridor the REST of its
      route still needs.

    Kept as knobs rather than reverted because the trade is real and
    board-dependent: a "re-admit dense connectors" variant regressed glasgow
    while helping lpddr4 (#585 item 8). The likely missing piece is MAGNITUDE,
    not membership -- the radius is a flat STUB_PROXIMITY_RADIUS (2.0mm)
    whether the pad is a 0.5mm BGA ball or a 2.54mm header pin, so re-admitted
    headers got a BGA-sized keep-out. A pitch-scaled radius would settle it.

    Args:
        pcb_data: PCB data
        net_ids: List of unrouted net IDs to get chip pads for
        min_pads: Minimum pads for a footprint to be considered (default: 4)

    Returns:
        List of (x, y, layer) tuples for chip pad positions.
    """
    import env_knobs
    import numpy as _np
    from kicad_parser import detect_package_type

    net_id_set = set(net_ids)

    # Sorted for deterministic output order. Footprints never move during a
    # run, so the chip-ref list is board-static -- memoized per (min_pads,
    # gate) on pcb_data (2026-08-14 profiling: this function was 95s of the
    # orangecrab step, called 1,464x with a full package-detect + segment
    # scan and a 570M-iteration retire genexpr each time).
    _fine_only = env_knobs.FINE_PITCH_PSEUDO_STUBS
    _refs_memo = getattr(pcb_data, '_chip_refs_memo', None)
    if _refs_memo is None:
        _refs_memo = pcb_data._chip_refs_memo = {}
    _rk = (min_pads, _fine_only)
    chip_refs = _refs_memo.get(_rk)
    if chip_refs is None:
        chip_refs = _refs_memo[_rk] = sorted(
            ref for ref, footprint in pcb_data.footprints.items()
            if footprint.pads and len(footprint.pads) >= min_pads
            and (not _fine_only
                 or detect_package_type(footprint) in ('BGA', 'QFN', 'QFP')))

    if not chip_refs:
        return []

    # Same-net attachment points for the "already escaped" test: segment
    # endpoints and via centers. Rebuilt once per COPPER EPOCH (the
    # add/remove_route choke-point counter) for ALL nets as float64 arrays;
    # per-call subsets just index the dict. The vectorized retire test below
    # runs the same float64 ops elementwise, so decisions are bit-identical
    # to the scalar genexpr it replaces.
    _epoch = getattr(pcb_data, '_copper_epoch', 0)
    _apm = getattr(pcb_data, '_attach_pts_memo', None)
    if _apm is None or _apm[0] != _epoch:
        _raw: dict = {}
        for seg in pcb_data.segments:
            _raw.setdefault(seg.net_id, []).append((seg.start_x, seg.start_y))
            _raw[seg.net_id].append((seg.end_x, seg.end_y))
        for via in pcb_data.vias:
            _raw.setdefault(via.net_id, []).append((via.x, via.y))
        _apm = pcb_data._attach_pts_memo = (
            _epoch,
            {nid: _np.array(pts, dtype=_np.float64)
             for nid, pts in _raw.items()})
    attach_points = _apm[1]

    chip_pads = []
    for ref in chip_refs:
        footprint = pcb_data.footprints[ref]
        for pad in footprint.pads:
            # Only include pads for nets we're tracking
            if pad.net_id not in net_id_set:
                continue
            # Use first copper layer from pad's layers
            pad_layer = None
            for layer in pad.layers:
                if layer.endswith('.Cu') and not layer.startswith('*'):
                    pad_layer = layer
                    break
            if not pad_layer:
                continue
            # Skip pads that already have an escape (stub end / via in reach)
            pts = (attach_points.get(pad.net_id)
                   if env_knobs.PSEUDO_STUB_RETIRE else None)
            if pts is not None and len(pts):
                reach = max(pad.size_x, pad.size_y) / 2.0 + 0.05
                reach_sq = reach * reach
                px, py = pad.global_x, pad.global_y
                dx = pts[:, 0] - px
                dy = pts[:, 1] - py
                if bool((dx * dx + dy * dy <= reach_sq).any()):
                    continue
            chip_pads.append((pad.global_x, pad.global_y, pad_layer))

    return chip_pads


def find_pad_nearest_to_position(pcb_data: PCBData, net_id: int, x: float, y: float) -> Optional[Pad]:
    """Find the pad for a given net that is nearest to the specified position."""
    pads = pcb_data.pads_by_net.get(net_id, [])
    if not pads:
        return None

    best_pad = None
    best_dist = float('inf')
    for pad in pads:
        dist = (pad.global_x - x) ** 2 + (pad.global_y - y) ** 2
        if dist < best_dist:
            best_dist = dist
            best_pad = pad

    return best_pad


def find_containing_or_nearest_bga_zone(
    point: Tuple[float, float],
    bga_zones: List[Tuple[float, float, float, float]]
) -> Optional[Tuple[float, float, float, float]]:
    """
    Find the BGA zone containing a point, or the nearest zone if outside all zones.

    Args:
        point: (x, y) position
        bga_zones: List of (min_x, min_y, max_x, max_y) BGA exclusion zones

    Returns:
        The containing/nearest BGA zone, or None if no zones provided
    """
    if not bga_zones:
        return None

    x, y = point

    # First check if inside any zone
    for zone in bga_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return zone

    # Not inside any zone - find nearest
    best_zone = None
    best_dist = float('inf')

    for zone in bga_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        # Distance to bounding box
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < best_dist:
            best_dist = dist
            best_zone = zone

    return best_zone


def compute_routing_aware_distance(
    target_free_end: Tuple[float, float],
    source_chip_center: Tuple[float, float],
    bga_zone: Tuple[float, float, float, float]
) -> float:
    """
    Compute the shortest path distance from target stub free end to source chip center,
    routing around the BGA zone (not through it).

    The algorithm:
    1. Determine which edge of the BGA the target stub is on/near
    2. Compute two candidate paths: around each corner of the BGA edge
    3. Return the shorter path distance

    Args:
        target_free_end: (x, y) position of target stub free end
        source_chip_center: (x, y) position of source chip center
        bga_zone: (min_x, min_y, max_x, max_y) BGA exclusion zone

    Returns:
        Routing-aware distance in mm
    """
    tx, ty = target_free_end
    sx, sy = source_chip_center
    min_x, min_y, max_x, max_y = bga_zone[:4]

    def point_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    # Get BGA corners
    corners = {
        'top_left': (min_x, min_y),
        'top_right': (max_x, min_y),
        'bottom_left': (min_x, max_y),
        'bottom_right': (max_x, max_y)
    }

    # Determine which edge the target stub is on/nearest to
    dist_to_left = abs(tx - min_x)
    dist_to_right = abs(tx - max_x)
    dist_to_top = abs(ty - min_y)
    dist_to_bottom = abs(ty - max_y)

    min_dist = min(dist_to_left, dist_to_right, dist_to_top, dist_to_bottom)

    # Determine candidate corners based on stub edge position
    if min_dist == dist_to_top:
        # Stub on top edge - can go around top-left or top-right corner
        corner1, corner2 = corners['top_left'], corners['top_right']
    elif min_dist == dist_to_bottom:
        # Stub on bottom edge
        corner1, corner2 = corners['bottom_left'], corners['bottom_right']
    elif min_dist == dist_to_left:
        # Stub on left edge
        corner1, corner2 = corners['top_left'], corners['bottom_left']
    else:  # dist_to_right
        # Stub on right edge
        corner1, corner2 = corners['top_right'], corners['bottom_right']

    # Path 1: target -> corner1 -> source
    dist1 = point_distance(target_free_end, corner1) + point_distance(corner1, source_chip_center)

    # Path 2: target -> corner2 -> source
    dist2 = point_distance(target_free_end, corner2) + point_distance(corner2, source_chip_center)

    return min(dist1, dist2)


def get_unit_routing_info(
    pcb_data: PCBData,
    unit_net_ids: List[int]
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    Get target stub free end and source chip center for a routing unit.

    For diff pairs, averages the P and N positions.

    Args:
        pcb_data: PCB data
        unit_net_ids: List of net IDs (1 for single net, 2 for diff pair)

    Returns:
        (target_free_end, source_chip_center) or None if not determinable
    """
    target_free_ends = []
    source_chip_centers = []

    for net_id in unit_net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        net_pads = pcb_data.pads_by_net.get(net_id, [])

        if len(net_segments) < 2:
            continue

        # Via-AWARE grouping: without vias= a route that changes layers
        # falls apart into per-layer fragments, and a fully-routed coupled
        # net then warns "Could not find pads for both stub groups" and
        # loses its MPS unit info (ordering-only impact, but noisy and
        # wrong). Same call shape as get_net_endpoints Case 1.
        net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) < 2:
            continue

        # Find which pad connects to each stub group
        def get_group_pad(group_segs):
            """Find the pad connected to this stub group."""
            group_points = set()
            for seg in group_segs:
                group_points.add(pos_key(seg.start_x, seg.start_y))
                group_points.add(pos_key(seg.end_x, seg.end_y))
            for pad in net_pads:
                pad_pos = pos_key(pad.global_x, pad.global_y)
                for gp in group_points:
                    if abs(pad_pos[0] - gp[0]) < 0.05 and abs(pad_pos[1] - gp[1]) < 0.05:
                        return pad
            return None

        # Get pad for each group.
        #
        # groups[:2] took whichever two groups owned the earliest segments in
        # the board -- and the GUI and CLI hold identical copper in different
        # order. Rank by COPPER first (more segments, then more length, geometry
        # last) so "the two main stub groups" is a property of the net, not of
        # the file. Same rule as the source/target ranking in
        # connectivity.get_net_endpoints.
        def _grp_rank(g):
            total = 0.0
            anchor = None
            for _s in g:
                total += math.hypot(_s.end_x - _s.start_x, _s.end_y - _s.start_y)
                a = (min((_s.start_x, _s.start_y), (_s.end_x, _s.end_y)),
                     max((_s.start_x, _s.start_y), (_s.end_x, _s.end_y)), _s.layer)
                if anchor is None or a < anchor:
                    anchor = a
            return (-len(g), -total, anchor)

        group_pads = []
        for group in sorted(groups, key=_grp_rank)[:2]:
            pad = get_group_pad(group)
            if pad:
                group_pads.append((pad.component_ref, group, pad))

        if len(group_pads) < 2:
            net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
            print(f"WARNING: Could not find pads for both stub groups in {net_name} (found {len(group_pads)} pads)")
            continue

        # Sort by component_ref alphabetically for consistent source/target assignment
        # First component (alphabetically) = source, second = target
        # Tie-break beyond component_ref: both stubs of a net frequently land on
        # the SAME component, so the alphabetical sort tied and fell back to the
        # order the groups were discovered in -- board order again. Add the pad
        # number and position so source/target assignment is geometric.
        group_pads.sort(key=lambda x: (x[0], str(x[2].pad_number),
                                       x[2].global_x, x[2].global_y))
        source_segs = group_pads[0][1]
        target_segs = group_pads[1][1]
        source_pad = group_pads[0][2]

        # Get target stub free end (or pad position if no stub free end)
        target_free = find_stub_free_ends(target_segs, net_pads)
        if target_free:
            target_free_ends.append((target_free[0][0], target_free[0][1]))
        else:
            # Use the target pad position as fallback
            target_pad = group_pads[1][2]  # target pad from earlier
            target_free_ends.append((target_pad.global_x, target_pad.global_y))

        # Get source stub free end (or pad position if no stub free end)
        source_free = find_stub_free_ends(source_segs, net_pads)
        if source_free:
            source_chip_centers.append((source_free[0][0], source_free[0][1]))
        else:
            # Use the source pad position as fallback
            source_pad = group_pads[0][2]  # source pad from earlier
            source_chip_centers.append((source_pad.global_x, source_pad.global_y))

    if not target_free_ends or not source_chip_centers:
        return None

    # Average for diff pairs
    avg_target = (
        sum(p[0] for p in target_free_ends) / len(target_free_ends),
        sum(p[1] for p in target_free_ends) / len(target_free_ends)
    )
    avg_source = (
        sum(p[0] for p in source_chip_centers) / len(source_chip_centers),
        sum(p[1] for p in source_chip_centers) / len(source_chip_centers)
    )

    return (avg_target, avg_source)


def _build_mps_unit_mappings(
    pcb_data: PCBData,
    net_ids: List[int],
    diff_pairs: Dict
) -> Tuple[Dict[int, int], Dict[int, List[int]], Dict[int, str], List[int]]:
    """
    Build mapping from net_id to unit_id for MPS ordering.

    Diff pair P/N nets are grouped into a single unit.

    Returns:
        Tuple of (net_to_unit, unit_to_nets, unit_names, unit_ids)
    """
    net_to_unit = {}  # net_id -> unit_id
    unit_to_nets = {}  # unit_id -> [net_ids]
    unit_names = {}  # unit_id -> display name

    if diff_pairs:
        for pair_name, pair in diff_pairs.items():
            if pair.p_net_id in net_ids or pair.n_net_id in net_ids:
                # Use P net ID as the canonical unit ID
                unit_id = pair.p_net_id
                net_to_unit[pair.p_net_id] = unit_id
                net_to_unit[pair.n_net_id] = unit_id
                unit_to_nets[unit_id] = [pair.p_net_id, pair.n_net_id]
                unit_names[unit_id] = pair_name

    # Add single nets (not part of any diff pair)
    for net_id in net_ids:
        if net_id not in net_to_unit:
            net_to_unit[net_id] = net_id
            unit_to_nets[net_id] = [net_id]
            unit_names[net_id] = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"

    # Get unique unit IDs from the net_ids list
    unit_ids = []
    seen_units = set()
    for net_id in net_ids:
        unit_id = net_to_unit.get(net_id, net_id)
        if unit_id not in seen_units:
            seen_units.add(unit_id)
            unit_ids.append(unit_id)

    return net_to_unit, unit_to_nets, unit_names, unit_ids


def _compute_mps_unit_endpoints(
    pcb_data: PCBData,
    unit_ids: List[int],
    unit_to_nets: Dict[int, List[int]]
) -> Dict[int, List[Tuple[float, float]]]:
    """
    Compute routing endpoints for each unit.

    For diff pairs, averages P and N endpoints.

    Returns:
        Dict mapping unit_id to [source_endpoint, target_endpoint]
    """
    unit_endpoints = {}
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])

        if len(unit_net_ids) == 2:
            # Diff pair: combine P and N endpoints
            p_endpoints = get_net_routing_endpoints(pcb_data, unit_net_ids[0])
            n_endpoints = get_net_routing_endpoints(pcb_data, unit_net_ids[1])
            if len(p_endpoints) >= 2 and len(n_endpoints) >= 2:
                src = ((p_endpoints[0][0] + n_endpoints[0][0]) / 2,
                       (p_endpoints[0][1] + n_endpoints[0][1]) / 2)
                tgt = ((p_endpoints[1][0] + n_endpoints[1][0]) / 2,
                       (p_endpoints[1][1] + n_endpoints[1][1]) / 2)
                unit_endpoints[unit_id] = [src, tgt]
        else:
            # Single net
            endpoints = get_net_routing_endpoints(pcb_data, unit_id)
            if len(endpoints) >= 2:
                unit_endpoints[unit_id] = endpoints[:2]

    return unit_endpoints


def _compute_mps_center(
    unit_endpoints: Dict[int, List[Tuple[float, float]]],
    center: Tuple[float, float] = None
) -> Tuple[float, float]:
    """Compute center point for angular projection if not provided."""
    if center is not None:
        return center

    all_points = []
    for endpoints in unit_endpoints.values():
        all_points.extend(endpoints)
    if all_points:
        return (
            sum(p[0] for p in all_points) / len(all_points),
            sum(p[1] for p in all_points) / len(all_points)
        )
    return (0, 0)


def _compute_mps_unit_layers(
    pcb_data: PCBData,
    unit_ids: List[int],
    unit_to_nets: Dict[int, List[int]]
) -> Dict[int, Tuple[Set[str], Set[str]]]:
    """
    Build layer information for each unit from stub segments.

    Returns:
        Dict mapping unit_id to (source_layers, target_layers)
    """
    unit_layers = {}
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])
        src_layers = set()
        tgt_layers = set()

        for net_id in unit_net_ids:
            net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
            if net_segments:
                # #545 F12: via-aware grouping (a layer-changing stub is ONE
                # group, not per-layer fragments), and rank the two main
                # groups by COPPER like every other consumer -- [0]/[1] in
                # board order differs between the GUI and CLI fronts.
                _nv = [v for v in pcb_data.vias if v.net_id == net_id]
                stub_groups = find_connected_groups(net_segments, vias=_nv)
                if len(stub_groups) >= 2:
                    def _sg_rank(g):
                        total = 0.0
                        anchor = None
                        for _s in g:
                            total += math.hypot(_s.end_x - _s.start_x,
                                                _s.end_y - _s.start_y)
                            a = (min((_s.start_x, _s.start_y),
                                     (_s.end_x, _s.end_y)),
                                 max((_s.start_x, _s.start_y),
                                     (_s.end_x, _s.end_y)), _s.layer)
                            if anchor is None or a < anchor:
                                anchor = a
                        return (-len(g), -total, anchor)
                    stub_groups = sorted(stub_groups, key=_sg_rank)
                    for seg in stub_groups[0]:
                        src_layers.add(seg.layer)
                    for seg in stub_groups[1]:
                        tgt_layers.add(seg.layer)

        # If no stub layers found, use pad layers as fallback
        if not src_layers and not tgt_layers:
            for net_id in unit_net_ids:
                net_pads = pcb_data.pads_by_net.get(net_id, [])
                for pad in net_pads:
                    for layer in pad.layers:
                        if layer.endswith('.Cu') and not layer.startswith('*'):
                            src_layers.add(layer)
                            tgt_layers.add(layer)

        unit_layers[unit_id] = (src_layers, tgt_layers)

    return unit_layers


def _compute_mps_unit_distances(
    pcb_data: PCBData,
    unit_list: List[int],
    unit_to_nets: Dict[int, List[int]],
    unit_endpoints: Dict[int, List[Tuple[float, float]]],
    bga_exclusion_zones: List[Tuple[float, float, float, float]]
) -> Dict[int, float]:
    """
    Compute routing-aware distances for each unit.

    Used as secondary ordering: shorter routes first within same conflict count.

    Returns:
        Dict mapping unit_id to distance
    """
    unit_distances = {}
    for unit_id in unit_list:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])
        routing_info = get_unit_routing_info(pcb_data, unit_net_ids)

        if routing_info and bga_exclusion_zones:
            target_free_end, source_chip_center = routing_info
            bga_zone = find_containing_or_nearest_bga_zone(target_free_end, bga_exclusion_zones)

            if bga_zone:
                unit_distances[unit_id] = compute_routing_aware_distance(
                    target_free_end, source_chip_center, bga_zone
                )
            else:
                dx = source_chip_center[0] - target_free_end[0]
                dy = source_chip_center[1] - target_free_end[1]
                unit_distances[unit_id] = math.sqrt(dx * dx + dy * dy)
        else:
            if unit_id in unit_endpoints:
                endpoints = unit_endpoints[unit_id]
                dx = endpoints[1][0] - endpoints[0][0]
                dy = endpoints[1][1] - endpoints[0][1]
                unit_distances[unit_id] = math.sqrt(dx * dx + dy * dy)
            else:
                unit_distances[unit_id] = float('inf')

    return unit_distances


def _greedy_order_mps_units(
    unit_list: List[int],
    conflicts: Dict[int, Set[int]],
    unit_distances: Dict[int, float],
    unit_names: Dict[int, str],
    reverse_rounds: bool
) -> Tuple[List[int], Dict[int, int], int]:
    """
    Order units using greedy algorithm: pick unit with fewest conflicts.

    Returns:
        Tuple of (ordered_units, round_assignments, num_rounds)
    """
    all_rounds = []
    round_assignments = {}
    remaining = set(unit_list)
    round_num = 0

    while remaining:
        round_num += 1
        round_winners = []
        round_losers = set()

        round_remaining = set(remaining)
        while round_remaining:
            best_unit = min(
                round_remaining,
                key=lambda uid: (len(conflicts[uid] & round_remaining), unit_distances.get(uid, 0), uid)
            )

            round_winners.append(best_unit)
            round_remaining.discard(best_unit)
            round_assignments[best_unit] = round_num

            for loser in conflicts[best_unit] & round_remaining:
                round_losers.add(loser)
                round_remaining.discard(loser)

        all_rounds.append((round_winners, round_num))
        remaining = round_losers

    if reverse_rounds:
        all_rounds = list(reversed(all_rounds))
        print("MPS: Reversing round order (routing most-conflicting groups first)")

    # Build ordered list, sort each round by distance
    ordered_units = []
    for round_winners, orig_round_num in all_rounds:
        # Tie-break on uid: sorting by distance alone is STABLE, so units at
        # equal distance kept round_winners order. That order is itself
        # deterministic, but any upstream wobble in a float distance flips
        # neighbouring units and reorders the whole round -- and routing order
        # is board-wide. uid is a stable identity, so equal-distance units now
        # have one canonical order on both fronts.
        sorted_winners = sorted(round_winners,
                                key=lambda uid: (unit_distances.get(uid, 0), uid))
        ordered_units.extend(sorted_winners)
        if sorted_winners:
            winner_names = [unit_names.get(uid, f"Net {uid}") for uid in sorted_winners]
            names_str = ", ".join(winner_names)
            print(f"MPS Round {orig_round_num}: {len(sorted_winners)} units: {names_str}")

    return ordered_units, round_assignments, round_num


def compute_mps_net_ordering(pcb_data: PCBData, net_ids: List[int],
                              center: Tuple[float, float] = None,
                              diff_pairs: Dict = None,
                              use_boundary_ordering: bool = True,
                              bga_exclusion_zones: List[Tuple[float, float, float, float]] = None,
                              reverse_rounds: bool = False,
                              crossing_layer_check: bool = True,
                              return_extended_info: bool = False,
                              use_segment_intersection: bool = None) -> Union[List[int], MPSResult]:
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
        diff_pairs: Optional dict of pair_name -> DiffPair. If provided, P and N nets
                    of each pair are treated as a single routing unit.
        use_boundary_ordering: If True, use chip boundary unrolling for crossing detection.
                              This respects the physical constraint that routes can't go
                              through BGA chips. Default is False (use angular projection).
        return_extended_info: If True, return MPSResult with conflict and layer info
                             instead of just ordered IDs. Used for MPS-aware layer swaps.
        use_segment_intersection: If True, use MST segment intersection for crossing detection.
                                 If None (default), auto-detect: use segment intersection when
                                 no net endpoints are on chips.

    Returns:
        If return_extended_info=False: Ordered list of net IDs, with least-conflicting nets first
        If return_extended_info=True: MPSResult with full conflict/layer/round info
    """
    # Step 1: Build unit mappings (group diff pair P/N nets)
    net_to_unit, unit_to_nets, unit_names, unit_ids = _build_mps_unit_mappings(
        pcb_data, net_ids, diff_pairs
    )

    # Step 2: Get routing endpoints for each unit
    unit_endpoints = _compute_mps_unit_endpoints(pcb_data, unit_ids, unit_to_nets)

    if not unit_endpoints:
        print("MPS: No units with valid routing endpoints found")
        return list(net_ids)

    # Step 3: Compute center if not provided
    center = _compute_mps_center(unit_endpoints, center)

    # Step 4: Compute positions for crossing detection
    # Methods: boundary ordering (BGA), segment intersection (non-BGA), or angular (fallback)
    unit_boundary_info = {}  # unit_id -> (src_pos, tgt_pos, src_chip, tgt_chip) or None
    unit_angles = {}  # unit_id -> (angle1, angle2)
    unit_mst_segments = {}  # unit_id -> list of (p1, p2) segments

    # Build MST segments for each unit (used for segment intersection method)
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])
        all_segments = []
        for net_id in unit_net_ids:
            all_segments.extend(get_net_mst_segments(pcb_data, net_id))
        if all_segments:
            unit_mst_segments[unit_id] = all_segments

    if use_boundary_ordering:
        # Use chip boundary ordering - respects physical constraint that routes can't go through chips
        chips = build_chip_list(pcb_data)

        for unit_id, endpoints in unit_endpoints.items():
            src_chip = identify_chip_for_point(endpoints[0], chips)
            tgt_chip = identify_chip_for_point(endpoints[1], chips)

            if src_chip and tgt_chip and src_chip != tgt_chip:
                # Normalize source/target by component reference (alphabetically)
                if src_chip.reference > tgt_chip.reference:
                    endpoints = [endpoints[1], endpoints[0]]
                    src_chip, tgt_chip = tgt_chip, src_chip
                    unit_endpoints[unit_id] = endpoints

                src_far, tgt_far = compute_far_side(src_chip, tgt_chip)
                src_pos = compute_boundary_position(src_chip, endpoints[0], src_far, clockwise=True)
                tgt_pos = compute_boundary_position(tgt_chip, endpoints[1], tgt_far, clockwise=False)
                unit_boundary_info[unit_id] = (src_pos, tgt_pos, src_chip, tgt_chip)

    # Auto-detect segment intersection mode if not specified
    if use_segment_intersection is None:
        any_in_bga = False
        if bga_exclusion_zones:
            for unit_id, endpoints in unit_endpoints.items():
                for ep in endpoints:
                    for zone in bga_exclusion_zones:
                        min_x, min_y, max_x, max_y = zone[:4]
                        if min_x <= ep[0] <= max_x and min_y <= ep[1] <= max_y:
                            any_in_bga = True
                            break
                    if any_in_bga:
                        break
                if any_in_bga:
                    break

        use_segment_intersection = not any_in_bga and len(unit_mst_segments) > 0
        if use_segment_intersection:
            print("MPS: Using segment intersection method (no nets on BGA chips)")

    # Compute angular positions (fallback method)
    def angle_from_center(point: Tuple[float, float]) -> float:
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        ang = math.atan2(dy, dx)
        if ang < 0:
            ang += 2 * math.pi
        return ang

    for unit_id, endpoints in unit_endpoints.items():
        if not use_boundary_ordering or unit_boundary_info.get(unit_id) is None:
            a1 = angle_from_center(endpoints[0])
            a2 = angle_from_center(endpoints[1])
            if a1 > a2:
                a1, a2 = a2, a1
            unit_angles[unit_id] = (a1, a2)

    # Step 5: Build layer information for each unit
    unit_layers = _compute_mps_unit_layers(pcb_data, unit_ids, unit_to_nets)

    # Step 6: Detect crossing conflicts
    def units_cross_geometric(unit_a: int, unit_b: int) -> bool:
        """Check if two units have crossing paths (geometric only, ignores layers)."""
        # Use segment intersection method if enabled (takes priority)
        if use_segment_intersection:
            segs_a = unit_mst_segments.get(unit_a, [])
            segs_b = unit_mst_segments.get(unit_b, [])
            # Check if any segment from A crosses any segment from B
            for seg_a in segs_a:
                for seg_b in segs_b:
                    if segments_intersect(seg_a[0], seg_a[1], seg_b[0], seg_b[1]):
                        return True
            return False

        # Use boundary ordering for BGA chips
        info_a = unit_boundary_info.get(unit_a)
        info_b = unit_boundary_info.get(unit_b)
        if use_boundary_ordering and info_a is not None and info_b is not None:
            # Only compare if same chip pair
            if (info_a[2], info_a[3]) != (info_b[2], info_b[3]):
                return False  # Different chip pairs, can't directly cross
            # Check boundary order inversion
            return crossings_from_boundary_order(info_a[0], info_a[1], info_b[0], info_b[1])

        # Use angular method (default or fallback)
        if unit_a in unit_angles and unit_b in unit_angles:
            a1, a2 = unit_angles[unit_a]
            b1, b2 = unit_angles[unit_b]
            return (a1 < b1 < a2 < b2) or (b1 < a1 < b2 < a2)

        return False

    def units_share_layer(unit_a: int, unit_b: int) -> bool:
        """Check if two units share at least one layer.

        If either unit has no layer info (no stubs/unrouted), assume they could
        share any layer and return True.
        """
        a_src, a_tgt = unit_layers.get(unit_a, (set(), set()))
        b_src, b_tgt = unit_layers.get(unit_b, (set(), set()))
        a_all = a_src | a_tgt
        b_all = b_src | b_tgt
        # If either has no layer info, assume potential conflict on any layer
        if not a_all or not b_all:
            return True
        return bool(a_all & b_all)

    # Build conflict graphs - both geometric (all crossings) and layer-filtered
    unit_list = list(unit_endpoints.keys())
    geometric_conflicts = {unit_id: set() for unit_id in unit_list}
    conflicts = {unit_id: set() for unit_id in unit_list}

    for i, unit_a in enumerate(unit_list):
        for unit_b in unit_list[i+1:]:
            if units_cross_geometric(unit_a, unit_b):
                # Always add to geometric conflicts
                geometric_conflicts[unit_a].add(unit_b)
                geometric_conflicts[unit_b].add(unit_a)
                # Only add to layer-filtered conflicts if they share a layer (or layer check disabled)
                if not crossing_layer_check or units_share_layer(unit_a, unit_b):
                    conflicts[unit_a].add(unit_b)
                    conflicts[unit_b].add(unit_a)

    # Count total conflicts for reporting
    total_conflicts = sum(len(c) for c in conflicts.values()) // 2
    num_diff_pairs = sum(1 for uid in unit_list if len(unit_to_nets.get(uid, [])) == 2)
    num_single = len(unit_list) - num_diff_pairs
    if num_diff_pairs > 0:
        print(f"MPS: {num_diff_pairs} diff pairs + {num_single} single nets = {len(unit_list)} units with {total_conflicts} crossing conflicts")
    else:
        print(f"MPS: {len(unit_list)} nets with {total_conflicts} crossing conflicts detected")

    # Step 7: Compute route distances for each unit
    unit_distances = _compute_mps_unit_distances(
        pcb_data, unit_list, unit_to_nets, unit_endpoints, bga_exclusion_zones
    )

    # Step 8: Greedy ordering - pick units with fewest conflicts
    ordered_units, round_assignments, round_num = _greedy_order_mps_units(
        unit_list, conflicts, unit_distances, unit_names, reverse_rounds
    )

    # Expand ordered units back to net IDs
    ordered = []
    for unit_id in ordered_units:
        ordered.extend(unit_to_nets.get(unit_id, [unit_id]))

    # Add any nets that weren't in unit_angles (no valid endpoints) at the end
    # These are nets we couldn't determine routing endpoints for
    ordered_set = set(ordered)
    nets_without_endpoints = [nid for nid in net_ids if nid not in ordered_set]
    if nets_without_endpoints:
        print(f"MPS: {len(nets_without_endpoints)} nets without valid endpoints appended at end")
        ordered.extend(nets_without_endpoints)

    if return_extended_info:
        return MPSResult(
            ordered_ids=ordered,
            conflicts=conflicts,
            unit_layers=unit_layers,
            unit_to_nets=unit_to_nets,
            unit_names=unit_names,
            round_assignments=round_assignments,
            num_rounds=round_num,
            geometric_conflicts=geometric_conflicts
        )
    return ordered
