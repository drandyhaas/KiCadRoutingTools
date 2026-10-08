"""
Blocker detection for copper plane via placement and repair.

Identifies which net is blocking a via placement or a route. The rip-up
EXECUTION machinery that used to live here (try_place_via_with_ripup,
_settle_ripped_nets) was deleted with route_planes' tap loop (#562); the
detection survivors serve repair_planes, single_ended_routing and
routing_diagnostics.
"""
from __future__ import annotations

from typing import List, Dict, Tuple, Optional, Set

from kicad_parser import PCBData
from routing_config import GridRouteConfig, GridCoord
from bresenham_utils import walk_line



def find_via_position_blocker(
    via_x: float,
    via_y: float,
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int,
    protected_net_ids: Optional[Set[int]] = None,
    quiet: bool = False
) -> Optional[int]:
    """
    Find the net that is blocking via placement at a specific position.

    Checks segments and vias from other nets to find what's blocking
    the given via position.

    Args:
        via_x, via_y: Position where via placement is blocked
        pcb_data: PCB data with all segments/vias
        config: Routing configuration
        exclude_net_id: Net ID to exclude (the target net)
        protected_net_ids: Set of net IDs that should never be identified as blockers

    Returns:
        Net ID of the closest non-protected blocker, or None if no blocker found
    """
    best_blocker = None
    best_dist_sq = float('inf')
    best_protected_blocker = None
    best_protected_dist_sq = float('inf')
    protected = protected_net_ids or set()

    # Check segments
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        dist_sq = _point_to_segment_dist_sq(via_x, via_y, seg.start_x, seg.start_y, seg.end_x, seg.end_y)
        clearance_needed = config.via_size / 2 + seg.width / 2 + config.clearance
        if dist_sq < clearance_needed ** 2:
            if seg.net_id in protected:
                if dist_sq < best_protected_dist_sq:
                    best_protected_dist_sq = dist_sq
                    best_protected_blocker = seg.net_id
            elif dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_blocker = seg.net_id

    # Check vias
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        dx = via.x - via_x
        dy = via.y - via_y
        dist_sq = dx * dx + dy * dy
        clearance_needed = config.via_size / 2 + via.size / 2 + config.clearance
        if dist_sq < clearance_needed ** 2:
            if via.net_id in protected:
                if dist_sq < best_protected_dist_sq:
                    best_protected_dist_sq = dist_sq
                    best_protected_blocker = via.net_id
            elif dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_blocker = via.net_id

    # Report protected blocker if it's closer than any non-protected blocker
    if (not quiet and best_protected_blocker is not None
            and best_protected_dist_sq < best_dist_sq):
        net = pcb_data.nets.get(best_protected_blocker)
        blocker_name = net.name if net else f"net_{best_protected_blocker}"
        print(f"blocked by {blocker_name} (protected, cannot rip)...", end=" ")

    return best_blocker


def find_route_blocker_from_frontier(
    blocked_cells: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int,
    protected_net_ids: Optional[Set[int]] = None
) -> Optional[int]:
    """
    Find the net most responsible for blocking a route based on frontier data.

    Uses the blocked_cells from route_with_frontier to identify which net's
    segments/vias are blocking the most cells on the search frontier.

    Args:
        blocked_cells: List of (gx, gy, layer) cells from route_with_frontier
        pcb_data: PCB data with all segments/vias
        config: Routing configuration
        exclude_net_id: Net ID to exclude (the target net)
        protected_net_ids: Set of net IDs that should never be identified as blockers

    Returns:
        Net ID of the top non-protected blocker, or None if no blocker found
    """
    if not blocked_cells:
        return None

    coord = GridCoord(config.grid_step)
    blocked_set = set(blocked_cells)
    protected = protected_net_ids or set()

    # Frontier bbox prefilter (#225): every membership test below is on layer 0,
    # so only layer-0 frontier cells can ever match, and a segment/via contributes
    # only if its clearance-expanded window overlaps that frontier. The router's
    # blocked frontier is local, but this function used to walk EVERY board track
    # and via (with a per-cell window scan), costing ~27s of rip-up handling on
    # daisho. Skipping copper whose expanded bbox can't reach the frontier is an
    # exact necessary-condition cull -- the counts (and the chosen blocker) are
    # unchanged. No layer-0 frontier cell -> nothing can match (was: empty tally).
    _l0x = [gx for (gx, gy, l) in blocked_set if l == 0]
    if not _l0x:
        return None
    _l0y = [gy for (gx, gy, l) in blocked_set if l == 0]
    bb_min_x, bb_max_x = min(_l0x), max(_l0x)
    bb_min_y, bb_max_y = min(_l0y), max(_l0y)

    # Count how many blocked cells each net is responsible for (including protected)
    net_block_count: Dict[int, int] = {}

    # Check segments
    # expansion = existing_track_half + clearance + routing_track_half. Size the
    # existing-track half from the segment's ACTUAL width (a wide/diff-pair trace
    # is responsible for more blocked cells than the default width implies), so
    # the rip-up heuristic attributes blockage to the right net. Mirrors #172.
    routing_half = config.track_width / 2

    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue

        # Get layer index (assume single layer routing, layer 0)
        layer_idx = 0

        seg_half = (seg.width if getattr(seg, 'width', 0) and seg.width > 0
                    else config.track_width) / 2
        expansion_grid = max(1, coord.to_grid_dist(seg_half + config.clearance + routing_half))

        # Trace along segment and check for blocked cells
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        # Skip segments whose expanded window can't reach the frontier bbox.
        if (max(gx1, gx2) + expansion_grid < bb_min_x or
                min(gx1, gx2) - expansion_grid > bb_max_x or
                max(gy1, gy2) + expansion_grid < bb_min_y or
                min(gy1, gy2) - expansion_grid > bb_max_y):
            continue

        count = 0
        for gx, gy in walk_line(gx1, gy1, gx2, gy2):
            # Check expansion around this point
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    cell = (gx + ex, gy + ey, layer_idx)
                    if cell in blocked_set:
                        count += 1

        if count > 0:
            net_block_count[seg.net_id] = net_block_count.get(seg.net_id, 0) + count

    # Check vias - size the keep-out from each via's ACTUAL size (a fanout
    # via-in-pad is larger than config.via_size), same rationale as the segments.
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue

        via_r = (via.size if getattr(via, 'size', 0) and via.size > 0
                 else config.via_size) / 2
        via_expansion_grid = max(1, coord.to_grid_dist(via_r + config.track_width / 2 + config.clearance))

        gx, gy = coord.to_grid(via.x, via.y)
        # Skip vias whose expanded window can't reach the frontier bbox.
        if (gx + via_expansion_grid < bb_min_x or gx - via_expansion_grid > bb_max_x or
                gy + via_expansion_grid < bb_min_y or gy - via_expansion_grid > bb_max_y):
            continue
        count = 0
        for ex in range(-via_expansion_grid, via_expansion_grid + 1):
            for ey in range(-via_expansion_grid, via_expansion_grid + 1):
                if ex * ex + ey * ey <= via_expansion_grid * via_expansion_grid:
                    # Vias block all layers, but for single-layer routing check layer 0
                    cell = (gx + ex, gy + ey, 0)
                    if cell in blocked_set:
                        count += 1

        if count > 0:
            net_block_count[via.net_id] = net_block_count.get(via.net_id, 0) + count

    if not net_block_count:
        return None

    # Find top blocker overall (for diagnostics) and top non-protected blocker (for ripping)
    top_blocker = max(net_block_count.keys(), key=lambda k: net_block_count[k])

    # Check if top blocker is protected
    if top_blocker in protected:
        net = pcb_data.nets.get(top_blocker)
        blocker_name = net.name if net else f"net_{top_blocker}"
        print(f"blocked by {blocker_name} (protected, cannot rip)...", end=" ")

        # Find top non-protected blocker
        non_protected = {k: v for k, v in net_block_count.items() if k not in protected}
        if non_protected:
            return max(non_protected.keys(), key=lambda k: non_protected[k])
        return None

    return top_blocker


def _point_to_segment_dist_sq(px: float, py: float,
                              ax: float, ay: float,
                              bx: float, by: float) -> float:
    """Squared distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-12:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def _restored_piece_collides(seg: Optional[Dict], via: Optional[Dict],
                             plane_vias: List[Dict], plane_segments: List[Dict],
                             via_size: float, clearance: float,
                             config=None, piece_net: Optional[int] = None,
                             plane_net: Optional[int] = None) -> bool:
    """Issue #88.1: return True if a to-be-restored segment or via would
    overlap newly-placed plane copper (plane vias/segments placed this run).

    Plane stitching vias span all layers, so a restored via or any restored
    segment (on any layer) that comes within (via_size/2 + own_radius +
    clearance) of a plane via center is a short. Restored segments are also
    tested against plane segments on the same layer. Collision-free pieces are
    restored verbatim; colliding pieces are left ripped (so the net falls into
    the ripped-nets set to be re-routed rather than shorted onto plane copper).

    #1136: given `config` and both nets -- the restored piece's (`piece_net`)
    and the plane copper's (`plane_net`) -- each pair is priced at
    `config.pair_clearance` from the base `clearance`: the two classes, then
    the .kicad_dru rule (the stack for via-via, the layer they meet on
    otherwise, the #735 track rule for track-track). Without them, or on a
    board that declares no class and no rule, at the flat `clearance`.
    """
    via_r = via_size / 2.0
    if (config is None or piece_net is None or plane_net is None
            or config.pair_clearance_inert()):
        def _clr(layer, kind):
            return clearance
    else:
        _memo = {}

        def _clr(layer, kind):
            v = _memo.get((layer, kind))
            if v is None:
                v = _memo[(layer, kind)] = config.pair_clearance(
                    piece_net, plane_net, layer, kind=kind, base=clearance)
            return v

    if via is not None:
        # Restored via vs plane vias (via-via, all layers).
        vr = via.get('size', via_size) / 2.0
        thresh = via_r + vr + _clr(None, 'stack')
        thresh_sq = thresh * thresh
        for pv in plane_vias:
            if (via['x'] - pv['x']) ** 2 + (via['y'] - pv['y']) ** 2 < thresh_sq:
                return True
        # Restored via vs plane SEGMENTS (the barrel spans all layers, so a
        # segment on any layer counts). The original #88.1 call sites only
        # restored against stitching VIAS so this was never needed; the #329
        # tap restore also checks against the tap's new TRACE copper --
        # without this, a restored via sat on 13 fresh +3V3 trace segments
        # (glasgow /IO_Banks/DA2, 0707b wave set1).
        for ps in plane_segments:
            ps_half_w = ps.get('width', 0.2) / 2.0
            v_thresh = vr + ps_half_w + _clr(ps.get('layer'), 'layer')
            if _point_to_segment_dist_sq(via['x'], via['y'],
                                         ps['start'][0], ps['start'][1],
                                         ps['end'][0], ps['end'][1]) < v_thresh * v_thresh:
                return True
        return False

    if seg is not None:
        sx0, sy0 = seg['start'][0], seg['start'][1]
        sx1, sy1 = seg['end'][0], seg['end'][1]
        half_w = seg.get('width', 0.2) / 2.0
        # Restored segment vs plane vias (via copper, all layers).
        thresh = via_r + half_w + _clr(seg.get('layer'), 'layer')
        thresh_sq = thresh * thresh
        for pv in plane_vias:
            if _point_to_segment_dist_sq(pv['x'], pv['y'], sx0, sy0, sx1, sy1) < thresh_sq:
                return True
        # Restored segment vs plane segments on the same layer (seg-seg).
        seg_layer = seg.get('layer')
        for ps in plane_segments:
            if ps.get('layer') != seg_layer:
                continue
            ps_half_w = ps.get('width', 0.2) / 2.0
            s_thresh = half_w + ps_half_w + _clr(seg_layer, 'track')
            s_thresh_sq = s_thresh * s_thresh
            # Endpoint sampling covers the short axis-overlap case; an X
            # CROSSING has all four endpoints far apart, so also test true
            # intersection (#329 restore checks restored signal traces
            # against the tap's new trace copper, where crossings happen).
            px0, py0 = ps['start'][0], ps['start'][1]
            px1, py1 = ps['end'][0], ps['end'][1]
            if (_point_to_segment_dist_sq(px0, py0, sx0, sy0, sx1, sy1) < s_thresh_sq or
                    _point_to_segment_dist_sq(px1, py1, sx0, sy0, sx1, sy1) < s_thresh_sq or
                    _point_to_segment_dist_sq(sx0, sy0, px0, py0, px1, py1) < s_thresh_sq or
                    _point_to_segment_dist_sq(sx1, sy1, px0, py0, px1, py1) < s_thresh_sq):
                return True
            from geometry_utils import segments_intersect_2d
            if segments_intersect_2d((sx0, sy0), (sx1, sy1), (px0, py0), (px1, py1)):
                return True
        return False

    return False



# (try_place_via_with_ripup / _settle_ripped_nets /
#  ViaPlacementResult / _re_add_pad_obstacles_for_net DELETED,
#  review dead-code 2: their only consumer was route_planes'
#  Step-9 tap/rip loop, removed in 8c72da7/1080c97. The live
#  survivors of this module are find_via_position_blocker,
#  find_route_blocker_from_frontier, _restored_piece_collides and
#  _point_to_segment_dist_sq, used by repair_planes /
#  single_ended_routing / routing_diagnostics.)
