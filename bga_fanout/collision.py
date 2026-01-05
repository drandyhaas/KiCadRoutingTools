"""
Collision detection utilities for BGA fanout routing.
"""

import math
from typing import Tuple, List, Dict, Set

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


def find_colliding_pairs(tracks: List[Dict], min_spacing: float) -> Set[str]:
    """Find all differential pairs or single-ended nets involved in collisions.

    Returns set of identifiers (pair_id for diff pairs, or 'net_<net_id>' for single-ended).
    """
    colliding_pairs = set()
    for i, t1 in enumerate(tracks):
        # Skip existing tracks (we can't move them)
        if t1.get('is_existing'):
            continue

        for j, t2 in enumerate(tracks[i+1:], i+1):
            # Skip if different layers
            if t1['layer'] != t2['layer']:
                continue
            # Skip if same net
            if t1['net_id'] == t2['net_id']:
                continue
            # Skip if same pair (P and N of same diff pair)
            if t1.get('pair_id') and t1.get('pair_id') == t2.get('pair_id'):
                continue

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
    id_tracks = [t for t in tracks if tracks_match_identifier(t, identifier)]
    other_tracks = [t for t in tracks if not tracks_match_identifier(t, identifier) and not t.get('is_existing')]

    for pt in id_tracks:
        for ot in other_tracks:
            if pt['layer'] != ot['layer']:
                continue
            if check_segment_collision(pt['start'], pt['end'],
                                        ot['start'], ot['end'],
                                        min_spacing):
                other_id = get_track_identifier(ot)
                if other_id:
                    partners.add(other_id)
    return partners
