"""
Track generation and collision detection for BGA fanout routing.

Functions for generating track segments from routes and detecting collisions.
"""

from typing import Dict, List, Tuple

from kicad_parser import PCBData
from bga_fanout.types import create_track, FanoutRoute
from bga_fanout.collision import check_segment_collision
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
                                       min_spacing):
                collision_count += 1
                if len(collision_pairs) < max_pairs:
                    collision_pairs.append((t1, t2))

    return collision_count, collision_pairs


def convert_segments_to_tracks(pcb_data: PCBData) -> List[Dict]:
    """
    Convert PCB segments to track format for collision checking.

    Args:
        pcb_data: PCB data containing segments

    Returns:
        List of track dictionaries
    """
    existing_tracks = []
    for seg in pcb_data.segments:
        existing_tracks.append(create_track(
            (seg.start_x, seg.start_y), (seg.end_x, seg.end_y),
            seg.width, seg.layer, seg.net_id, is_existing=True
        ))
    return existing_tracks


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

    for route in routes:
        if route.is_edge:
            # Edge pad: Check if stub_end differs from pad_pos (differential pair convergence)
            if route.pair_id and (abs(route.stub_end[0] - route.pad_pos[0]) > POSITION_TOLERANCE or
                                   abs(route.stub_end[1] - route.pad_pos[1]) > POSITION_TOLERANCE):
                # Differential edge pair: 45° stub to converge, then straight to exit
                # 45° stub: pad -> stub_end (convergence point)
                tracks.append(create_track(route.pad_pos, route.stub_end, track_width,
                                           route.layer, route.net_id, route.pair_id))
                # Straight segment: stub_end -> exit
                tracks.append(create_track(route.stub_end, route.exit_pos, track_width,
                                           route.layer, route.net_id, route.pair_id))
            else:
                # Single-ended edge pad: direct segment to exit
                tracks.append(create_track(route.pad_pos, route.exit_pos, track_width,
                                           route.layer, route.net_id, route.pair_id))
            # Add jog segment(s)
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append(create_track(route.exit_pos, route.jog_extension, track_width,
                                               route.layer, route.net_id, route.pair_id))
                    # Then the 45° jog from extension point
                    tracks.append(create_track(route.jog_extension, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id))
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append(create_track(route.exit_pos, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id))
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
                                           route.layer, route.net_id, route.pair_id))
                # Straight segment in channel: channel_point -> channel_point2 (1 pitch)
                if route.channel_point2:
                    tracks.append(create_track(route.channel_point, route.channel_point2, track_width,
                                               route.layer, route.net_id, route.pair_id))
                    # Second 45°: channel_point2 -> stub_end (exit from channel)
                    tracks.append(create_track(route.channel_point2, route.stub_end, track_width,
                                               route.layer, route.net_id, route.pair_id))
                else:
                    # Fallback if no channel_point2 (shouldn't happen)
                    tracks.append(create_track(route.channel_point, route.stub_end, track_width,
                                               route.layer, route.net_id, route.pair_id))
                # Straight segment: stub_end -> exit
                tracks.append(create_track(route.stub_end, route.exit_pos, track_width,
                                           route.layer, route.net_id, route.pair_id))
            else:
                # Normal inner pad: single 45° stub to channel, then straight to exit
                # Check for zero-length stubs (pad is exactly on channel)
                dx = abs(route.stub_end[0] - route.pad_pos[0])
                dy = abs(route.stub_end[1] - route.pad_pos[1])
                has_stub = dx >= POSITION_TOLERANCE or dy >= POSITION_TOLERANCE

                if has_stub:
                    # 45-degree stub: pad -> stub_end
                    tracks.append(create_track(route.pad_pos, route.stub_end, track_width,
                                               route.layer, route.net_id, route.pair_id))

                if route.pre_channel_jog:
                    # Jogged route: stub_end -> jog_point -> exit
                    # Jog segment: stub_end -> pre_channel_jog
                    tracks.append(create_track(route.stub_end, route.pre_channel_jog, track_width,
                                               route.layer, route.net_id, route.pair_id))
                    # Channel segment: pre_channel_jog -> exit
                    tracks.append(create_track(route.pre_channel_jog, route.exit_pos, track_width,
                                               route.layer, route.net_id, route.pair_id))
                else:
                    # Normal route: stub_end -> exit
                    tracks.append(create_track(route.stub_end, route.exit_pos, track_width,
                                               route.layer, route.net_id, route.pair_id))

            # Jog segment(s): exit -> jog_end
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append(create_track(route.exit_pos, route.jog_extension, track_width,
                                               route.layer, route.net_id, route.pair_id))
                    # Then the 45° jog from extension point
                    tracks.append(create_track(route.jog_extension, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id))
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append(create_track(route.exit_pos, route.jog_end, track_width,
                                               route.layer, route.net_id, route.pair_id))
            inner_count += 1

    print(f"  Generated {len(tracks)} track segments")
    print(f"    Edge pads: {edge_count} (direct H/V on {top_layer})")
    print(f"    Inner pads: {inner_count} (45° stub + channel)")

    return tracks, edge_count, inner_count
