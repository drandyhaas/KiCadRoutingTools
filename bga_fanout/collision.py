"""
Collision detection utilities for BGA fanout routing.
"""

import math
from typing import Tuple, List, Dict, Set

import numpy as np

from bga_fanout.constants import POSITION_TOLERANCE


def check_segment_collision(seg1_start: Tuple[float, float], seg1_end: Tuple[float, float],
                            seg2_start: Tuple[float, float], seg2_end: Tuple[float, float],
                            clearance: float) -> bool:
    """Check if two segments are too close."""
    # Horizontal segments
    if abs(seg1_start[1] - seg1_end[1]) < POSITION_TOLERANCE and abs(seg2_start[1] - seg2_end[1]) < POSITION_TOLERANCE:
        if abs(seg1_start[1] - seg2_start[1]) < clearance:
            x1_min, x1_max = min(seg1_start[0], seg1_end[0]), max(seg1_start[0], seg1_end[0])
            x2_min, x2_max = min(seg2_start[0], seg2_end[0]), max(seg2_start[0], seg2_end[0])
            if x1_max > x2_min - clearance and x2_max > x1_min - clearance:
                return True

    # Vertical segments
    if abs(seg1_start[0] - seg1_end[0]) < POSITION_TOLERANCE and abs(seg2_start[0] - seg2_end[0]) < POSITION_TOLERANCE:
        if abs(seg1_start[0] - seg2_start[0]) < clearance:
            y1_min, y1_max = min(seg1_start[1], seg1_end[1]), max(seg1_start[1], seg1_end[1])
            y2_min, y2_max = min(seg2_start[1], seg2_end[1]), max(seg2_start[1], seg2_end[1])
            if y1_max > y2_min - clearance and y2_max > y1_min - clearance:
                return True

    # Check endpoint proximity
    for p1 in [seg1_start, seg1_end]:
        for p2 in [seg2_start, seg2_end]:
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
            if dist < clearance:
                return True

    # Check for general segment intersection (handles diagonal crossings)
    # Using cross product method to detect intersection
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(seg2_start, seg2_end, seg1_start)
    d2 = cross(seg2_start, seg2_end, seg1_end)
    d3 = cross(seg1_start, seg1_end, seg2_start)
    d4 = cross(seg1_start, seg1_end, seg2_end)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True

    # Also check point-to-segment distance for near-misses with clearance
    def point_to_segment_dist(px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        proj_x, proj_y = x1 + t * dx, y1 + t * dy
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    # Check distance from each endpoint of seg1 to seg2
    for p in [seg1_start, seg1_end]:
        if point_to_segment_dist(p[0], p[1], seg2_start[0], seg2_start[1], seg2_end[0], seg2_end[1]) < clearance:
            return True
    # Check distance from each endpoint of seg2 to seg1
    for p in [seg2_start, seg2_end]:
        if point_to_segment_dist(p[0], p[1], seg1_start[0], seg1_start[1], seg1_end[0], seg1_end[1]) < clearance:
            return True

    return False


def _track_bbox_arrays(tracks: List[Dict]):
    """Axis-aligned bounding box (xmin/xmax/ymin/ymax) of every track, as arrays."""
    n = len(tracks)
    sx = np.empty(n); sy = np.empty(n); ex = np.empty(n); ey = np.empty(n)
    for i, t in enumerate(tracks):
        sx[i], sy[i] = t['start']
        ex[i], ey[i] = t['end']
    return (np.minimum(sx, ex), np.maximum(sx, ex),
            np.minimum(sy, ey), np.maximum(sy, ey))


def _encode_pair_ids(tracks: List[Dict]) -> "np.ndarray":
    """Map each track's pair_id to an int code (>=0 for a truthy pair_id, -1 for none)."""
    codes = np.empty(len(tracks), dtype=np.int64)
    mapping: Dict = {}
    for i, t in enumerate(tracks):
        v = t.get('pair_id')
        if not v:
            codes[i] = -1
        else:
            c = mapping.get(v)
            if c is None:
                c = len(mapping)
                mapping[v] = c
            codes[i] = c
    return codes


def find_colliding_pairs(tracks: List[Dict], min_spacing: float) -> Set[str]:
    """Find all differential pairs or single-ended nets involved in collisions.

    Returns set of identifiers (pair_id for diff pairs, or 'net_<net_id>' for single-ended).

    The cheap, exact filters (same layer, different net, different pair) and a
    clearance-inflated bounding-box overlap test are evaluated with numpy across
    all O(n^2) pairs at once, replacing the Python double loop. The bbox test is
    a *necessary* condition for `check_segment_collision` to fire (two segments
    within `min_spacing` must have bboxes overlapping within that margin), so the
    exact per-segment check still decides every surviving candidate - the result
    set is identical to the scalar version.
    """
    colliding_pairs = set()
    n = len(tracks)
    if n < 2:
        return colliding_pairs

    xmin, xmax, ymin, ymax = _track_bbox_arrays(tracks)
    nets = np.array([t['net_id'] for t in tracks])
    existing = np.array([bool(t.get('is_existing')) for t in tracks])
    pair_codes = _encode_pair_ids(tracks)

    # Collisions only happen within a layer; bucket first so the O(m^2) pair
    # enumeration is per-layer rather than over the whole board.
    layer_buckets: Dict = {}
    for i, t in enumerate(tracks):
        layer_buckets.setdefault(t['layer'], []).append(i)

    for idx_list in layer_buckets.values():
        m = len(idx_list)
        if m < 2:
            continue
        idxs = np.array(idx_list)  # ascending -> idxs[ii] < idxs[jj] for ii<jj
        ii, jj = np.triu_indices(m, k=1)
        a = idxs[ii]
        b = idxs[jj]

        mask = ~existing[a]
        mask &= nets[a] != nets[b]
        same_pair = (pair_codes[a] >= 0) & (pair_codes[a] == pair_codes[b])
        mask &= ~same_pair
        # Clearance-inflated bbox overlap (necessary condition for a collision).
        mask &= (xmax[a] >= xmin[b] - min_spacing) & (xmax[b] >= xmin[a] - min_spacing)
        mask &= (ymax[a] >= ymin[b] - min_spacing) & (ymax[b] >= ymin[a] - min_spacing)

        sel = np.nonzero(mask)[0]
        for i, j in zip(a[sel].tolist(), b[sel].tolist()):
            t1 = tracks[i]
            t2 = tracks[j]
            if check_segment_collision(t1['start'], t1['end'],
                                        t2['start'], t2['end'],
                                        min_spacing):
                # Use pair_id if available, otherwise use net_id as identifier
                if t1.get('pair_id'):
                    colliding_pairs.add(t1['pair_id'])
                elif not t1.get('is_existing'):
                    colliding_pairs.add(f"net_{t1['net_id']}")
                if t2.get('pair_id'):
                    colliding_pairs.add(t2['pair_id'])
                elif not t2.get('is_existing'):
                    colliding_pairs.add(f"net_{t2['net_id']}")
    return colliding_pairs


def get_track_identifier(track: Dict) -> str:
    """Get the identifier for a track (pair_id or net_XXX for single-ended)."""
    if track.get('pair_id'):
        return track['pair_id']
    elif track.get('net_id') and not track.get('is_existing'):
        return f"net_{track['net_id']}"
    return None


def tracks_match_identifier(track: Dict, identifier: str) -> bool:
    """Check if a track matches the given identifier."""
    if identifier.startswith('net_'):
        net_id = int(identifier[4:])
        return track.get('net_id') == net_id and not track.get('pair_id')
    else:
        return track.get('pair_id') == identifier


def find_collision_partners(identifier: str, tracks: List[Dict], min_spacing: float) -> Set[str]:
    """Find all pairs/nets that the given identifier collides with on the same layer."""
    partners = set()
    id_idx = [i for i, t in enumerate(tracks) if tracks_match_identifier(t, identifier)]
    other_idx = [i for i, t in enumerate(tracks)
                 if not tracks_match_identifier(t, identifier) and not t.get('is_existing')]
    if not id_idx or not other_idx:
        return partners

    xmin, xmax, ymin, ymax = _track_bbox_arrays(tracks)
    o = np.array(other_idx)
    o_layer = np.array([tracks[k]['layer'] for k in other_idx], dtype=object)

    for i in id_idx:
        pt = tracks[i]
        # Same layer + clearance-inflated bbox overlap is a necessary condition;
        # the exact check below decides each surviving candidate.
        mask = (o_layer == pt['layer'])
        mask &= (xmax[i] >= xmin[o] - min_spacing) & (xmax[o] >= xmin[i] - min_spacing)
        mask &= (ymax[i] >= ymin[o] - min_spacing) & (ymax[o] >= ymin[i] - min_spacing)
        for k in o[np.nonzero(mask)[0]].tolist():
            ot = tracks[k]
            if check_segment_collision(pt['start'], pt['end'],
                                        ot['start'], ot['end'],
                                        min_spacing):
                other_id = get_track_identifier(ot)
                if other_id:
                    partners.add(other_id)
    return partners
