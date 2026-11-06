"""
Track generation and collision detection for BGA fanout routing.

Functions for generating track segments from routes and detecting collisions.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

from kicad_parser import PCBData
from bga_fanout.types import create_track, route_uid, FanoutRoute
from bga_fanout.collision import check_segment_collision, spacing_to
from bga_fanout.constants import POSITION_TOLERANCE


def detect_collisions(
    tracks: List[Dict],
    existing_tracks: List[Dict],
    min_spacing: float,
    max_pairs: int = 5
) -> Tuple[int, List[Tuple[Dict, Dict]]]:
    """
    Detect collisions between tracks.

    Args:
        tracks: New tracks to check
        existing_tracks: Existing tracks to check against
        min_spacing: Minimum required spacing between tracks
        max_pairs: Maximum number of collision pairs to return

    Returns:
        Tuple of (collision_count, list of collision pairs)
    """
    collision_count = 0
    collision_pairs = []

    for i, t1 in enumerate(tracks):
        # Check against other new tracks
        for j, t2 in enumerate(tracks[i+1:], i+1):
            if t1['layer'] != t2['layer']:
                continue
            if t1['net_id'] == t2['net_id']:
                continue
            # Skip collision check for tracks from the same differential pair
            if t1.get('pair_id') and t1.get('pair_id') == t2.get('pair_id'):
                continue
            if check_segment_collision(t1['start'], t1['end'],
                                       t2['start'], t2['end'],
                                       min_spacing):
                collision_count += 1
                if len(collision_pairs) < max_pairs:
                    collision_pairs.append((t1, t2))

        # Also check against existing tracks
        for t2 in existing_tracks:
            if t1['layer'] != t2['layer']:
                continue
            if t1['net_id'] == t2['net_id']:
                continue
            if check_segment_collision(t1['start'], t1['end'],
                                       t2['start'], t2['end'],
                                       spacing_to(t2, min_spacing)):
                collision_count += 1
                if len(collision_pairs) < max_pairs:
                    collision_pairs.append((t1, t2))

    return collision_count, collision_pairs


def pair_self_collisions(tracks: List[Dict], pair_spacing: float) -> List[Tuple[Dict, Dict]]:
    """The tracks of one differential pair that come closer to their partner
    than `pair_spacing`, the pair's own centreline spacing (#1684).

    detect_collisions leaves a pair's two legs out, since they run closer than
    two strangers may, and layer reassignment (its remedy) moves both legs
    together. So P and N could cross and still validate as no collisions."""
    hits = []
    for i, t1 in enumerate(tracks):
        if not t1.get('pair_id'):
            continue
        for t2 in tracks[i + 1:]:
            if t1.get('pair_id') == t2.get('pair_id') \
                    and t1['net_id'] != t2['net_id'] and t1['layer'] == t2['layer'] \
                    and check_segment_collision(t1['start'], t1['end'],
                                                t2['start'], t2['end'],
                                                pair_spacing - 1e-6):
                hits.append((t1, t2))
    return hits


def board_copper_tracks(pcb_data: PCBData, layers: List[str], track_width: float,
                        bounds: Tuple[float, float, float, float] = None,
                        clearance: float = 0.0) -> List[Dict]:
    """The copper already on the board, as existing-track dicts for the
    collision checks: every segment, and every via as a zero-length track on
    each of `layers` it has copper on.

    Each carries `spacing_extra` = (its width - `track_width`) / 2: the checks'
    `min_spacing` (track_width + clearance) is sized for two of this run's
    tracks, so a via or a wider track needs that much more between centrelines
    (`collision.spacing_to`). `bounds` (xmin, ymin, xmax, ymax) keeps only the
    copper that reaches into it.

    #1551: the board's copper used to enter the checks only with
    check_for_previous (the "skip pads that already have fanout" option), and
    its vias never did, so a fanout on a board that already carried copper
    near the array validated its escapes against nothing but each other.

    #1677: the board's Edge.Cuts rings stand here too, one zero-width track of
    no net per edge on each of `layers`, held at the edge clearance, so a
    channel escape across a cut-out is a collision like any other.
    """
    from check_drc import BoardEdgeGate, laid_copper_edge_clearance
    from geometry_utils import via_copper_layers

    def _inside(x0, y0, x1, y1, h):
        return bounds is None or (x1 + h >= bounds[0] and x0 - h <= bounds[2]
                                  and y1 + h >= bounds[1] and y0 - h <= bounds[3])

    out = []
    layer_set = set(layers)
    for seg in pcb_data.segments:
        if seg.layer not in layer_set:
            continue
        if not _inside(min(seg.start_x, seg.end_x), min(seg.start_y, seg.end_y),
                       max(seg.start_x, seg.end_x), max(seg.start_y, seg.end_y),
                       seg.width / 2):
            continue
        t = create_track((seg.start_x, seg.start_y), (seg.end_x, seg.end_y),
                         seg.width, seg.layer, seg.net_id, is_existing=True)
        t['spacing_extra'] = (seg.width - track_width) / 2
        out.append(t)
    for via in pcb_data.vias:
        if not _inside(via.x, via.y, via.x, via.y, via.size / 2):
            continue
        for layer in via_copper_layers(via, layers):
            t = create_track((via.x, via.y), (via.x, via.y), via.size, layer,
                             via.net_id, is_existing=True)
            t['spacing_extra'] = (via.size - track_width) / 2
            out.append(t)
    edges = BoardEdgeGate(pcb_data, laid_copper_edge_clearance(pcb_data, clearance))
    extra = edges.clearance - clearance - track_width / 2
    for ring in edges.rings:
        for (x0, y0), (x1, y1) in zip(ring, list(ring[1:]) + [ring[0]]):
            if not _inside(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), 0.0):
                continue
            for layer in layers:
                t = create_track((x0, y0), (x1, y1), 0.0, layer, -1, is_existing=True)
                t['spacing_extra'] = extra
                out.append(t)
    return out


def generate_tracks_from_routes(
    routes: List[FanoutRoute],
    track_width: float,
    top_layer: str,
) -> Tuple[List[Dict], int, int]:
    """
    Generate track segments from routes.

    Converts FanoutRoute objects into track dictionaries ready for output.

    Args:
        routes: List of FanoutRoute objects
        track_width: Width of tracks
        top_layer: Name of the top layer (for reporting)

    Returns:
        Tuple of (tracks, edge_count, inner_count)
    """
    tracks = []
    edge_count = 0
    inner_count = 0
    neighbor_count = 0

    for route in routes:
        # Neighbor connection: direct link to adjacent same-net pad
        if getattr(route, 'neighbor_connection', False):
            # Simple direct connection from this pad to neighbor pad
            tracks.append(create_track(route.pad_pos, route.exit_pos, track_width,
                                       route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
            neighbor_count += 1
            continue

        if route.is_edge:
            # Edge pad: Check if stub_end differs from pad_pos (differential pair convergence)
            if route.pair_id and (abs(route.stub_end[0] - route.pad_pos[0]) > POSITION_TOLERANCE or
                                   abs(route.stub_end[1] - route.pad_pos[1]) > POSITION_TOLERANCE):
                # Differential edge pair: 45° stub to converge, then straight to exit
                # 45° stub: pad -> stub_end (convergence point)
                tracks.append(create_track(route.pad_pos, route.stub_end, track_width,
                                           route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                # Straight segment: stub_end -> exit
                tracks.append(create_track(route.stub_end, route.exit_pos, track_width,
                                           route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
            else:
                # Single-ended edge pad: direct segment to exit
                tracks.append(create_track(route.pad_pos, route.exit_pos, track_width,
                                           route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
            # Add jog segment(s)
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append(create_track(route.exit_pos, route.jog_extension, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                    # Then the 45° jog from extension point
                    tracks.append(create_track(route.jog_extension, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append(create_track(route.exit_pos, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
            edge_count += 1
        else:
            # Inner pad: 45° stub + channel segment + jog
            # For half-edge pairs: pad -> channel_point -> stub_end -> exit

            if route.channel_point:
                # Half-edge inner pad: tent shape with channel segment
                # Path: pad -> channel_point -> channel_point2 -> stub_end -> exit
                #
                # First 45°: pad -> channel_point (entry to channel)
                tracks.append(create_track(route.pad_pos, route.channel_point, track_width,
                                           route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                # Straight segment in channel: channel_point -> channel_point2 (1 pitch)
                if route.channel_point2:
                    tracks.append(create_track(route.channel_point, route.channel_point2, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                    # Second 45°: channel_point2 -> stub_end (exit from channel)
                    tracks.append(create_track(route.channel_point2, route.stub_end, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                else:
                    # Fallback if no channel_point2 (shouldn't happen)
                    tracks.append(create_track(route.channel_point, route.stub_end, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                # Straight segment: stub_end -> exit
                tracks.append(create_track(route.stub_end, route.exit_pos, track_width,
                                           route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
            else:
                # Normal inner pad: single 45° stub to channel, then straight to exit
                # Check for zero-length stubs (pad is exactly on channel)
                dx = abs(route.stub_end[0] - route.pad_pos[0])
                dy = abs(route.stub_end[1] - route.pad_pos[1])
                has_stub = dx >= POSITION_TOLERANCE or dy >= POSITION_TOLERANCE

                if has_stub:
                    # 45-degree stub: pad -> stub_end
                    tracks.append(create_track(route.pad_pos, route.stub_end, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))

                if route.pre_channel_jog:
                    # Jogged route: stub_end -> jog_point -> exit
                    # Jog segment: stub_end -> pre_channel_jog
                    tracks.append(create_track(route.stub_end, route.pre_channel_jog, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                    # Channel segment: pre_channel_jog -> exit
                    tracks.append(create_track(route.pre_channel_jog, route.exit_pos, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                else:
                    # Normal route: stub_end -> exit
                    tracks.append(create_track(route.stub_end, route.exit_pos, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))

            # Jog segment(s): exit -> jog_end
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append(create_track(route.exit_pos, route.jog_extension, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                    # Then the 45° jog from extension point
                    tracks.append(create_track(route.jog_extension, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append(create_track(route.exit_pos, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id,
                                       route_uid=route_uid(route)))
            inner_count += 1

    print(f"  Generated {len(tracks)} track segments")
    print(f"    Edge pads: {edge_count} (direct H/V on {top_layer})")
    print(f"    Inner pads: {inner_count} (45° stub + channel)")
    if neighbor_count > 0:
        print(f"    Neighbor links: {neighbor_count} (direct pad-to-pad)")

    return tracks, edge_count, inner_count
