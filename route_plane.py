"""
Copper Plane Generator - Creates filled zones with via stitching to net pads.

Creates a solid copper zone on a specified layer, places vias near target pads,
and routes short traces to connect vias to pads when direct placement is blocked.

Usage:
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu
"""

import sys
import os
import re
import argparse
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass

# Add rust_router to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

from kicad_parser import parse_kicad_pcb, PCBData, Pad, Via, Segment
from kicad_writer import generate_via_sexpr, generate_segment_sexpr, generate_zone_sexpr
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map

try:
    from grid_router import GridObstacleMap
except ImportError:
    print("Error: grid_router module not found. Please build the Rust router first.")
    print("Run: python build_router.py")
    sys.exit(1)


@dataclass
class ZoneInfo:
    """Information about a copper zone/pour."""
    net_id: int
    net_name: str
    layer: str


def extract_zones(pcb_file: str) -> List[ZoneInfo]:
    """Extract zone information from a KiCad PCB file.

    Args:
        pcb_file: Path to the .kicad_pcb file

    Returns:
        List of ZoneInfo objects for each zone found
    """
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    zones = []
    # Pattern to match zone blocks and extract net_id, net_name, and layer
    zone_pattern = r'\(zone\s+\(net\s+(\d+)\)\s+\(net_name\s+"([^"]+)"\)\s+\(layer\s+"([^"]+)"\)'

    for match in re.finditer(zone_pattern, content):
        zones.append(ZoneInfo(
            net_id=int(match.group(1)),
            net_name=match.group(2),
            layer=match.group(3)
        ))

    return zones


def check_existing_zones(zones: List[ZoneInfo], target_layer: str, target_net_name: str,
                          target_net_id: int, verbose: bool = False) -> Tuple[bool, bool]:
    """Check for existing zones on the target layer.

    Args:
        zones: List of existing zones
        target_layer: Layer we want to create zone on
        target_net_name: Net name we want
        target_net_id: Net ID we want
        verbose: Print verbose output

    Returns:
        (should_create_zone, should_continue) tuple
        - should_create_zone: False if zone already exists on same net
        - should_continue: False if zone exists on different net (error condition)
    """
    for zone in zones:
        if zone.layer == target_layer:
            if zone.net_id == target_net_id or zone.net_name == target_net_name:
                # Same net - warn and skip zone creation but continue with vias
                print(f"Warning: Zone already exists on {target_layer} for net '{zone.net_name}' (ID {zone.net_id})")
                print("  Skipping zone creation, but will place vias to connect to existing zone.")
                return (False, True)
            else:
                # Different net - error
                print(f"Error: Zone already exists on {target_layer} for DIFFERENT net '{zone.net_name}' (ID {zone.net_id})")
                print(f"  Cannot create {target_net_name} zone on same layer. Aborting.")
                return (False, False)

    # No existing zone on this layer
    return (True, True)


def resolve_net_id(pcb_data: PCBData, net_name: str) -> Optional[int]:
    """Resolve a net name to its ID.

    Args:
        pcb_data: Parsed PCB data
        net_name: Net name to look up

    Returns:
        Net ID if found, None otherwise
    """
    # Check in pcb.nets
    for net in pcb_data.nets.values():
        if net.name == net_name:
            return net.net_id

    # Check in pads_by_net (some nets may only appear in pad definitions)
    for net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            if pad.net_name == net_name:
                return net_id

    return None


def identify_target_pads(
    pcb_data: PCBData,
    net_id: int,
    plane_layer: str
) -> List[Dict]:
    """
    Identify pads that need via connections to the plane layer.

    Returns list of dicts with pad info and connection type:
    - "through_hole": Through-hole pad - can connect on any layer, no via needed
    - "direct": SMD pad on plane layer - zone connects directly, no via needed
    - "via_needed": SMD pad on opposite layer - needs via + trace
    """
    target_pads = []
    pads = pcb_data.pads_by_net.get(net_id, [])

    for pad in pads:
        # Check if pad has drill (through-hole)
        if pad.drill > 0:
            # Through-hole pad - directly connects to all layers including plane
            target_pads.append({
                'pad': pad,
                'type': 'through_hole',
                'needs_via': False,
                'needs_trace': False
            })
        elif plane_layer in pad.layers or "*.Cu" in pad.layers:
            # SMD pad on plane layer - direct zone connection
            target_pads.append({
                'pad': pad,
                'type': 'direct',
                'needs_via': False,
                'needs_trace': False
            })
        else:
            # SMD pad NOT on plane layer - needs via
            # Get the pad's actual layer for trace routing
            pad_layer = None
            for layer in pad.layers:
                if layer.endswith('.Cu') and not layer.startswith('*'):
                    pad_layer = layer
                    break

            target_pads.append({
                'pad': pad,
                'type': 'via_needed',
                'needs_via': True,
                'needs_trace': True,  # May need trace if via can't be at pad center
                'pad_layer': pad_layer
            })

    return target_pads


def build_via_obstacle_map(
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int
) -> GridObstacleMap:
    """
    Build obstacle map for via placement.

    Blocks:
    - Existing vias (all nets - via-via clearance)
    - Pads on all layers (except target net pads)
    - Tracks on all layers (except target net)
    - Board edge clearance
    - Through-hole pad drills (hole-to-hole clearance)
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)

    obstacles = GridObstacleMap(num_layers)

    # Precompute grid expansions
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))
    via_track_expansion_grid = max(1, coord.to_grid_dist(config.via_size / 2 + config.track_width / 2 + config.clearance))

    # Add existing vias as obstacles (including same net - can't place via too close to another)
    for via in pcb_data.vias:
        gx, gy = coord.to_grid(via.x, via.y)
        # Block via placement within via-via clearance
        for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)

    # Add existing segments as obstacles (via can't overlap with tracks)
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_via_obstacle(obstacles, seg, coord, via_track_expansion_grid)

    # Add pads as obstacles (excluding target net pads)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            _add_pad_via_obstacle(obstacles, pad, coord, config)

    # Add board edge via blocking
    _add_board_edge_via_obstacles(obstacles, pcb_data, config)

    # Add hole-to-hole clearance blocking for existing drills
    _add_drill_hole_via_obstacles(obstacles, pcb_data, config, exclude_net_id)

    return obstacles


def _add_segment_via_obstacle(obstacles: GridObstacleMap, seg: Segment,
                               coord: GridCoord, expansion_grid: int):
    """Add a segment as via blocking obstacle."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Use Bresenham-style line blocking
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1

    x, y = gx1, gy1
    if dx > dy:
        err = dx / 2
        while x != gx2:
            _block_via_circle(obstacles, x, y, expansion_grid)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != gy2:
            _block_via_circle(obstacles, x, y, expansion_grid)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    _block_via_circle(obstacles, gx2, gy2, expansion_grid)


def _block_via_circle(obstacles: GridObstacleMap, cx: int, cy: int, radius: int):
    """Block via placement in a circular region."""
    for ex in range(-radius, radius + 1):
        for ey in range(-radius, radius + 1):
            if ex*ex + ey*ey <= radius * radius:
                obstacles.add_blocked_via(cx + ex, cy + ey)


def _add_pad_via_obstacle(obstacles: GridObstacleMap, pad: Pad,
                           coord: GridCoord, config: GridRouteConfig):
    """Add a pad as via blocking obstacle."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)

    # Use pad size + clearance
    pad_radius = max(pad.size_x, pad.size_y) / 2
    expansion = coord.to_grid_dist(pad_radius + config.via_size / 2 + config.clearance)

    _block_via_circle(obstacles, gx, gy, expansion)


def _add_board_edge_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig):
    """Block via placement near board edges."""
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return

    coord = GridCoord(config.grid_step)
    min_x, min_y, max_x, max_y = board_bounds

    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    via_edge_clearance = edge_clearance + config.via_size / 2
    via_expand = coord.to_grid_dist(via_edge_clearance)

    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    grid_margin = via_expand + 5

    # Block edges
    for gx in range(gmin_x - grid_margin, gmax_x + grid_margin + 1):
        for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
            # Outside board + margin
            if gx < gmin_x + via_expand or gx > gmax_x - via_expand or \
               gy < gmin_y + via_expand or gy > gmax_y - via_expand:
                obstacles.add_blocked_via(gx, gy)


def _add_drill_hole_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig, exclude_net_id: int):
    """Block via placement near existing drill holes."""
    if config.hole_to_hole_clearance <= 0:
        return

    coord = GridCoord(config.grid_step)

    # Collect drill holes
    drill_holes = []

    for via in pcb_data.vias:
        drill_holes.append((via.x, via.y, via.drill))

    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            if pad.drill > 0:
                drill_holes.append((pad.global_x, pad.global_y, pad.drill))

    for hx, hy, drill_dia in drill_holes:
        required_dist = drill_dia / 2 + config.via_drill / 2 + config.hole_to_hole_clearance
        expand = coord.to_grid_dist(required_dist)
        gx, gy = coord.to_grid(hx, hy)
        _block_via_circle(obstacles, gx, gy, expand)


def find_existing_via_nearby(
    pad: Pad,
    existing_vias: List[Tuple[float, float]],
    max_search_radius: float
) -> Optional[Tuple[float, float]]:
    """
    Find an existing via on the target net within search radius of the pad.

    Args:
        pad: The pad to connect
        existing_vias: List of (x, y) positions of existing vias on target net
        max_search_radius: Maximum distance to search

    Returns:
        (x, y) position of nearest existing via, or None if none found
    """
    best_via = None
    best_dist_sq = max_search_radius * max_search_radius

    for vx, vy in existing_vias:
        dx = vx - pad.global_x
        dy = vy - pad.global_y
        dist_sq = dx * dx + dy * dy
        if dist_sq <= best_dist_sq:
            best_dist_sq = dist_sq
            best_via = (vx, vy)

    return best_via


def find_via_position(
    pad: Pad,
    obstacles: GridObstacleMap,
    coord: GridCoord,
    max_search_radius: float
) -> Optional[Tuple[float, float]]:
    """
    Find the closest valid position for a via near a pad.

    Uses spiral search outward from pad center, checking clearances.

    Returns:
        (x, y) position for via, or None if no valid position found
    """
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # Try pad center first
    if not obstacles.is_via_blocked(pad_gx, pad_gy):
        return (pad.global_x, pad.global_y)

    # Spiral search outward
    max_radius_grid = coord.to_grid_dist(max_search_radius)

    # Search in expanding rings
    for radius in range(1, max_radius_grid + 1):
        best_pos = None
        best_dist_sq = float('inf')

        # Check all points at this Manhattan radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                # Skip if not on the ring edge
                if abs(dx) != radius and abs(dy) != radius:
                    continue

                gx, gy = pad_gx + dx, pad_gy + dy

                # Check if via can be placed here
                if obstacles.is_via_blocked(gx, gy):
                    continue

                # Check Euclidean distance
                dist_sq = dx * dx + dy * dy
                if dist_sq < best_dist_sq:
                    best_dist_sq = dist_sq
                    best_pos = coord.to_float(gx, gy)

        if best_pos:
            return best_pos

    return None  # No valid position within search radius


def block_via_position(obstacles: GridObstacleMap, via_x: float, via_y: float,
                        coord: GridCoord, hole_to_hole_clearance: float, via_drill: float):
    """Block the area around a newly placed via for hole-to-hole clearance.

    Args:
        obstacles: The obstacle map to update
        via_x, via_y: Position of the placed via
        coord: Grid coordinate converter
        hole_to_hole_clearance: Minimum clearance between drill holes
        via_drill: Drill diameter of the via
    """
    gx, gy = coord.to_grid(via_x, via_y)
    # Required distance: (this_drill/2) + (other_drill/2) + clearance
    # Since we're placing vias of the same size, it's: via_drill + clearance
    required_dist = via_drill + hole_to_hole_clearance
    expand = coord.to_grid_dist(required_dist)
    _block_via_circle(obstacles, gx, gy, expand)


def create_via_to_pad_trace(
    via_pos: Tuple[float, float],
    pad: Pad,
    pad_layer: str,
    net_id: int,
    track_width: float
) -> Optional[Dict]:
    """
    Create a direct trace from via position to pad center.

    Returns segment dict or None if via is at pad center.
    """
    # Check if via is at pad center (within tolerance)
    if abs(via_pos[0] - pad.global_x) < 0.001 and abs(via_pos[1] - pad.global_y) < 0.001:
        return None  # Via is at pad center, no trace needed

    return {
        'start': via_pos,
        'end': (pad.global_x, pad.global_y),
        'width': track_width,
        'layer': pad_layer,
        'net_id': net_id
    }


def write_plane_output(
    input_file: str,
    output_file: str,
    zone_sexpr: Optional[str],
    new_vias: List[Dict],
    new_segments: List[Dict]
) -> bool:
    """Write the complete output file with zone (optional), vias, and traces."""
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Build routing text
    elements = []

    # Add zone if provided
    if zone_sexpr:
        elements.append(zone_sexpr)

    # Add vias
    for via in new_vias:
        elements.append(generate_via_sexpr(
            via['x'], via['y'], via['size'], via['drill'],
            via['layers'], via['net_id']
        ))

    # Add segments
    for seg in new_segments:
        elements.append(generate_segment_sexpr(
            seg['start'], seg['end'],
            seg['width'], seg['layer'], seg['net_id']
        ))

    if not elements:
        # Nothing to add, just copy the file
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    routing_text = '\n'.join(elements)

    # Insert before final paren
    last_paren = content.rfind(')')
    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def create_plane(
    input_file: str,
    output_file: str,
    net_name: str,
    plane_layer: str,
    via_size: float = 0.3,
    via_drill: float = 0.2,
    track_width: float = 0.2,
    clearance: float = 0.1,
    zone_clearance: float = 0.2,
    min_thickness: float = 0.1,
    grid_step: float = 0.1,
    max_search_radius: float = 2.0,
    hole_to_hole_clearance: float = 0.2,
    all_layers: List[str] = None,
    verbose: bool = False,
    dry_run: bool = False
) -> Tuple[int, int, int]:
    """
    Create a copper plane zone and place vias to connect target pads.

    Returns:
        (vias_placed, traces_added, pads_needing_vias)
    """
    if all_layers is None:
        all_layers = ['F.Cu', 'B.Cu']

    # Step 1: Load PCB and find net
    if verbose:
        print(f"Loading PCB from {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    net_id = resolve_net_id(pcb_data, net_name)
    if net_id is None:
        print(f"Error: Net '{net_name}' not found in PCB")
        return (0, 0, 0)

    if verbose:
        print(f"Found net '{net_name}' with ID {net_id}")

    # Step 2: Check for existing zones on target layer
    existing_zones = extract_zones(input_file)
    should_create_zone, should_continue = check_existing_zones(
        existing_zones, plane_layer, net_name, net_id, verbose
    )

    if not should_continue:
        # Zone exists on different net - abort
        return (0, 0, 0)

    # Step 3: Get board bounds for zone polygon
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        print("Error: Could not determine board bounds")
        return (0, 0, 0)

    min_x, min_y, max_x, max_y = board_bounds
    if verbose:
        print(f"Board bounds: ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})")

    # Create zone polygon from board bounds (only used if creating new zone)
    zone_polygon = [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y)
    ]

    # Step 4: Identify target pads for via placement
    target_pads = identify_target_pads(pcb_data, net_id, plane_layer)

    pads_through_hole = sum(1 for p in target_pads if p['type'] == 'through_hole')
    pads_direct = sum(1 for p in target_pads if p['type'] == 'direct')
    pads_need_via = sum(1 for p in target_pads if p['type'] == 'via_needed')

    if verbose:
        print(f"\nPad analysis for net '{net_name}':")
        print(f"  Through-hole pads (no via needed): {pads_through_hole}")
        print(f"  SMD pads on {plane_layer} (no via needed): {pads_direct}")
        print(f"  SMD pads on other layers (via needed): {pads_need_via}")

    # Step 5: Collect existing vias on target net (for reuse)
    existing_net_vias: List[Tuple[float, float]] = []
    for via in pcb_data.vias:
        if via.net_id == net_id:
            existing_net_vias.append((via.x, via.y))

    if verbose and existing_net_vias:
        print(f"  Existing vias on net '{net_name}': {len(existing_net_vias)}")

    # Step 6: Build obstacle map for via placement
    config = GridRouteConfig(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        hole_to_hole_clearance=hole_to_hole_clearance,
        layers=all_layers
    )

    coord = GridCoord(grid_step)

    if pads_need_via > 0:
        if verbose:
            print("\nBuilding obstacle map for via placement...")
        obstacles = build_via_obstacle_map(pcb_data, config, net_id)
    else:
        obstacles = None

    # Step 7: Place vias near each target pad (or reuse existing)
    new_vias = []
    new_segments = []
    vias_placed = 0
    vias_reused = 0
    traces_added = 0
    failed_vias = 0

    # Track all available vias (existing + newly placed) for reuse
    available_vias = list(existing_net_vias)

    for pad_info in target_pads:
        if not pad_info['needs_via']:
            continue

        pad = pad_info['pad']
        pad_layer = pad_info.get('pad_layer')

        if verbose:
            print(f"  Finding via position for pad {pad.component_ref}.{pad.pad_number}...", end=" ")

        # First check if there's an existing or already-placed via nearby
        existing_via = find_existing_via_nearby(pad, available_vias, max_search_radius)

        if existing_via:
            # Reuse existing via - just create trace to connect
            via_pos = existing_via
            vias_reused += 1

            if pad_layer:
                trace = create_via_to_pad_trace(via_pos, pad, pad_layer, net_id, track_width)
                if trace:
                    new_segments.append(trace)
                    traces_added += 1
                    if verbose:
                        dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5
                        print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), trace {dist:.2f}mm")
                else:
                    if verbose:
                        print(f"reusing via at pad center ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
            else:
                if verbose:
                    print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
        else:
            # Need to place a new via
            via_pos = find_via_position(pad, obstacles, coord, max_search_radius)

            if via_pos:
                # Create via
                new_vias.append({
                    'x': via_pos[0],
                    'y': via_pos[1],
                    'size': via_size,
                    'drill': via_drill,
                    'layers': ['F.Cu', 'B.Cu'],  # Through-hole via
                    'net_id': net_id
                })
                vias_placed += 1

                # Add to available vias for potential reuse by other pads
                available_vias.append(via_pos)

                # Block this via position for hole-to-hole clearance
                block_via_position(obstacles, via_pos[0], via_pos[1], coord,
                                   hole_to_hole_clearance, via_drill)

                # Create trace if via not at pad center
                if pad_layer:
                    trace = create_via_to_pad_trace(via_pos, pad, pad_layer, net_id, track_width)
                    if trace:
                        new_segments.append(trace)
                        traces_added += 1
                        if verbose:
                            dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5
                            print(f"new via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), trace {dist:.2f}mm")
                    else:
                        if verbose:
                            print(f"new via at pad center ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
                else:
                    if verbose:
                        print(f"new via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
            else:
                failed_vias += 1
                if verbose:
                    print(f"FAILED - no valid position within {max_search_radius}mm")

    # Step 8: Generate zone polygon (only if we should create one)
    zone_sexpr = None
    if should_create_zone:
        zone_sexpr = generate_zone_sexpr(
            net_id=net_id,
            net_name=net_name,
            layer=plane_layer,
            polygon_points=zone_polygon,
            clearance=zone_clearance,
            min_thickness=min_thickness,
            direct_connect=True
        )

    # Step 9: Write output
    if verbose:
        print(f"\nResults:")
        if should_create_zone:
            print(f"  Zone created on {plane_layer}")
        else:
            print(f"  Using existing zone on {plane_layer}")
        print(f"  New vias placed: {vias_placed}")
        print(f"  Existing vias reused: {vias_reused}")
        print(f"  Traces added: {traces_added}")
        if failed_vias > 0:
            print(f"  Failed via placements: {failed_vias}")

    if dry_run:
        print("\nDry run - no output file written")
    else:
        if verbose:
            print(f"\nWriting output to {output_file}...")
        if write_plane_output(input_file, output_file, zone_sexpr, new_vias, new_segments):
            print(f"Output written to {output_file}")
            print("Note: Open in KiCad and press 'B' to refill zones")
        else:
            print("Error writing output file")

    return (vias_placed, traces_added, pads_need_via)


def main():
    parser = argparse.ArgumentParser(
        description="Create copper zone with via stitching to net pads",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu
    python route_plane.py input.kicad_pcb output.kicad_pcb --net VCC --layer In2.Cu --verbose
    python route_plane.py input.kicad_pcb output.kicad_pcb --net GND --layer B.Cu --dry-run
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", help="Output KiCad PCB file")

    # Required options
    parser.add_argument("--net", "-n", required=True, help="Net name for the plane (e.g., GND, VCC)")
    parser.add_argument("--layer", "-l", required=True, help="Copper layer for the zone (e.g., B.Cu, In1.Cu)")

    # Via and track geometry
    parser.add_argument("--via-size", type=float, default=0.3, help="Via outer diameter in mm (default: 0.3)")
    parser.add_argument("--via-drill", type=float, default=0.2, help="Via drill size in mm (default: 0.2)")
    parser.add_argument("--track-width", type=float, default=0.2, help="Track width for via-to-pad connections in mm (default: 0.2)")
    parser.add_argument("--clearance", type=float, default=0.1, help="Clearance in mm (default: 0.1)")

    # Zone options
    parser.add_argument("--zone-clearance", type=float, default=0.2, help="Zone clearance from other copper in mm (default: 0.2)")
    parser.add_argument("--min-thickness", type=float, default=0.1, help="Minimum zone copper thickness in mm (default: 0.1)")

    # Algorithm options
    parser.add_argument("--grid-step", type=float, default=0.1, help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--max-search-radius", type=float, default=2.0, help="Max radius to search for valid via position in mm (default: 2.0)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=0.2, help="Minimum clearance between drill holes in mm (default: 0.2)")
    parser.add_argument("--all-layers", nargs="+", default=['F.Cu', 'B.Cu'],
                        help="All copper layers for via span (default: F.Cu B.Cu)")

    # Debug options
    parser.add_argument("--verbose", "-v", action="store_true", help="Print detailed output")
    parser.add_argument("--dry-run", action="store_true", help="Analyze without writing output")

    args = parser.parse_args()

    create_plane(
        input_file=args.input_file,
        output_file=args.output_file,
        net_name=args.net,
        plane_layer=args.layer,
        via_size=args.via_size,
        via_drill=args.via_drill,
        track_width=args.track_width,
        clearance=args.clearance,
        zone_clearance=args.zone_clearance,
        min_thickness=args.min_thickness,
        grid_step=args.grid_step,
        max_search_radius=args.max_search_radius,
        hole_to_hole_clearance=args.hole_to_hole_clearance,
        all_layers=args.all_layers,
        verbose=args.verbose,
        dry_run=args.dry_run
    )


if __name__ == "__main__":
    main()
