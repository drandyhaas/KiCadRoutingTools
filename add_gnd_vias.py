"""
GND via placement for single-ended routing.

Adds GND vias near signal vias for return current paths.
"""

import math
from typing import List, Dict

from kicad_parser import PCBData, Via
from routing_config import GridRouteConfig, GridCoord
from grid_router import GridObstacleMap


def add_gnd_vias_near_signal_vias(
    results: List[Dict],
    pcb_data: PCBData,
    gnd_net_name: str,
    gnd_via_distance: float,
    config: GridRouteConfig,
    obstacles: GridObstacleMap,
    coord: GridCoord
) -> List[Via]:
    """
    Add GND vias near signal vias that don't already have a nearby GND via or pad.

    Places GND vias as close as possible to each signal via, checking from minimum
    viable distance outward up to gnd_via_distance.

    Args:
        results: List of routing results with 'new_vias' entries
        pcb_data: PCB data with existing vias and pads
        gnd_net_name: Net name for GND (e.g., "GND")
        gnd_via_distance: Maximum distance to place GND via from signal via (mm)
        config: Routing config with via size, drill, clearance, layers
        obstacles: Current obstacle map with all routed tracks/vias
        coord: Grid coordinate converter

    Returns:
        List of new GND Via objects to add
    """
    # Find GND net ID
    gnd_net_id = None
    for net_id, net in pcb_data.nets.items():
        if net.name == gnd_net_name:
            gnd_net_id = net_id
            break

    if gnd_net_id is None:
        print(f"Warning: GND net '{gnd_net_name}' not found, skipping GND via placement")
        return []

    # Collect all signal vias from routing results (non-GND vias)
    signal_vias = []
    for result in results:
        for via in result.get('new_vias', []):
            if via.net_id != gnd_net_id:
                signal_vias.append(via)

    if not signal_vias:
        return []

    # Collect existing GND vias from PCB data and routing results
    existing_gnd_positions = [(v.x, v.y) for v in pcb_data.vias if v.net_id == gnd_net_id]
    for result in results:
        for via in result.get('new_vias', []):
            if via.net_id == gnd_net_id:
                existing_gnd_positions.append((via.x, via.y))

    # Also include through-hole GND pads (they act as GND vias)
    for pad in pcb_data.pads_by_net.get(gnd_net_id, []):
        if pad.drill and pad.drill > 0:  # Through-hole pad
            existing_gnd_positions.append((pad.global_x, pad.global_y))

    # Minimum distance from signal via center to GND via center
    # (signal via radius + clearance + GND via radius)
    min_distance = config.via_size / 2 + config.clearance + config.via_size / 2

    new_gnd_vias = []
    placed_gnd_positions = []  # Track mm positions of GND vias we're adding

    # Via-to-via minimum clearance (center-to-center)
    via_via_min_dist = config.via_size + config.clearance

    # Grid cells to check for via clearance
    # The obstacle map already expands tracks by track_width/2 + clearance.
    # We only need to check cells within via_drill/2 (the actual hole).
    via_check_radius_grid = max(1, coord.to_grid_dist(config.via_drill / 2))

    def is_via_position_clear(x_mm: float, y_mm: float, skip_via_x: float, skip_via_y: float) -> bool:
        """Check if a via can be placed at the given position.

        Args:
            x_mm, y_mm: Proposed GND via position
            skip_via_x, skip_via_y: Signal via position to skip when checking via clearance
        """
        gx, gy = coord.to_grid(x_mm, y_mm)

        # Check track clearance on all layers (via must not overlap tracks)
        for layer_idx in range(len(config.layers)):
            for dx in range(-via_check_radius_grid, via_check_radius_grid + 1):
                for dy in range(-via_check_radius_grid, via_check_radius_grid + 1):
                    if dx * dx + dy * dy <= via_check_radius_grid * via_check_radius_grid:
                        if obstacles.is_blocked(gx + dx, gy + dy, layer_idx):
                            return False

        # Manual via-to-via clearance check (skip the signal via we're placing near)
        for via in pcb_data.vias:
            if abs(via.x - skip_via_x) < 0.01 and abs(via.y - skip_via_y) < 0.01:
                continue  # Skip the signal via we're placing a GND via for
            dist = math.sqrt((x_mm - via.x)**2 + (y_mm - via.y)**2)
            if dist < via_via_min_dist:
                return False

        # Also check vias from routing results (the signal vias we're working with)
        for result in results:
            for via in result.get('new_vias', []):
                if abs(via.x - skip_via_x) < 0.01 and abs(via.y - skip_via_y) < 0.01:
                    continue  # Skip the target signal via
                dist = math.sqrt((x_mm - via.x)**2 + (y_mm - via.y)**2)
                if dist < via_via_min_dist:
                    return False

        # Check clearance from through-hole pads
        for net_id, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if pad.drill and pad.drill > 0:
                    dist = math.sqrt((x_mm - pad.global_x)**2 + (y_mm - pad.global_y)**2)
                    min_pad_dist = pad.drill / 2 + config.clearance + config.via_drill / 2
                    if dist < min_pad_dist:
                        return False

        # Check against GND vias we're placing in this batch
        for px, py in placed_gnd_positions:
            dist = math.sqrt((x_mm - px)**2 + (y_mm - py)**2)
            if dist < via_via_min_dist:
                return False

        return True

    # Try many angles for better coverage (every 15 degrees = 24 directions)
    angles_deg = list(range(0, 360, 15))

    for sig_via in signal_vias:
        sx, sy = sig_via.x, sig_via.y

        # Check if there's already a GND via/pad within gnd_via_distance
        has_nearby_gnd = False
        for gx, gy in existing_gnd_positions + placed_gnd_positions:
            dist = math.sqrt((sx - gx)**2 + (sy - gy)**2)
            if dist <= gnd_via_distance:
                has_nearby_gnd = True
                break

        if has_nearby_gnd:
            continue

        # Try to place a GND via as close as possible to signal via
        # Search all angles at each distance, find the closest valid position
        best_pos = None
        distance_step = config.grid_step / 2  # Finer step for better placement

        distance = min_distance
        while distance <= gnd_via_distance:
            # Try all angles at this distance
            for angle_deg in angles_deg:
                angle_rad = math.radians(angle_deg)
                gnd_x = sx + distance * math.cos(angle_rad)
                gnd_y = sy + distance * math.sin(angle_rad)

                if is_via_position_clear(gnd_x, gnd_y, sx, sy):
                    # Found a valid position at closest possible distance
                    best_pos = (gnd_x, gnd_y)
                    break

            if best_pos is not None:
                break  # Found a position, stop searching

            distance += distance_step

        if best_pos:
            gnd_via = Via(
                x=best_pos[0],
                y=best_pos[1],
                size=config.via_size,
                drill=config.via_drill,
                layers=config.layers,
                net_id=gnd_net_id,
                free=True  # Prevent KiCad auto-assignment
            )
            new_gnd_vias.append(gnd_via)
            placed_gnd_positions.append(best_pos)
            existing_gnd_positions.append(best_pos)

    if new_gnd_vias:
        print(f"Added {len(new_gnd_vias)} GND vias near signal vias")

    return new_gnd_vias


def add_gnd_vias_to_existing_board(
    pcb_data: PCBData,
    gnd_net_name: str,
    gnd_via_distance: float,
    config: GridRouteConfig,
    obstacles: GridObstacleMap,
    coord: GridCoord
) -> List[Via]:
    """
    Add GND vias near existing signal vias in a PCB that don't already have a nearby GND via.

    This function works with existing board vias, not routing results.

    Args:
        pcb_data: PCB data with existing vias and pads
        gnd_net_name: Net name for GND (e.g., "GND")
        gnd_via_distance: Maximum distance to place GND via from signal via (mm)
        config: Routing config with via size, drill, clearance, layers
        obstacles: Obstacle map built from the board
        coord: Grid coordinate converter

    Returns:
        List of new GND Via objects to add
    """
    # Find GND net ID
    gnd_net_id = None
    for net_id, net in pcb_data.nets.items():
        if net.name == gnd_net_name:
            gnd_net_id = net_id
            break

    if gnd_net_id is None:
        print(f"Warning: GND net '{gnd_net_name}' not found, skipping GND via placement")
        return []

    # Collect all signal vias from the board (non-GND, non-power vias)
    signal_vias = [v for v in pcb_data.vias if v.net_id != gnd_net_id and v.net_id != 0]

    if not signal_vias:
        print("No signal vias found in board")
        return []

    print(f"Checking {len(signal_vias)} signal vias for GND via placement")

    # Collect existing GND vias from PCB data
    existing_gnd_positions = [(v.x, v.y) for v in pcb_data.vias if v.net_id == gnd_net_id]

    # Also include through-hole GND pads (they act as GND vias)
    for pad in pcb_data.pads_by_net.get(gnd_net_id, []):
        if pad.drill and pad.drill > 0:  # Through-hole pad
            existing_gnd_positions.append((pad.global_x, pad.global_y))

    # Minimum distance from signal via center to GND via center
    # (signal via radius + clearance + GND via radius)
    min_distance = config.via_size / 2 + config.clearance + config.via_size / 2

    # Debug: Print clearance parameters
    print(f"GND via placement parameters:")
    print(f"  via_size={config.via_size}mm, via_drill={config.via_drill}mm, clearance={config.clearance}mm")
    print(f"  grid_step={config.grid_step}mm")
    print(f"  min_distance (center-to-center)={min_distance:.3f}mm")

    new_gnd_vias = []
    placed_gnd_positions = []

    # Via-to-via minimum clearance (center-to-center) for batch placement
    via_via_min_dist = config.via_size + config.clearance

    # Grid cells to check for via clearance
    # The obstacle map already expands tracks by track_width/2 + clearance.
    # We only need to check cells within via_drill/2 (the actual hole), not the full
    # via pad, since the obstacle map expansion already accounts for clearance.
    via_check_radius_grid = max(1, coord.to_grid_dist(config.via_drill / 2))

    def is_via_position_clear(x_mm: float, y_mm: float, sig_via_x: float, sig_via_y: float) -> tuple:
        """Check if a via can be placed at the given position.

        Returns: (is_clear, reason) where reason explains why it's blocked if not clear.

        Note: We do NOT use obstacles.is_via_blocked() here because that blocks
        positions within via_size+clearance of ALL vias, including the signal via
        we're placing near. Since we already enforce min_distance (which equals
        via_size+clearance), we just need to check track clearance and manually
        check other vias.
        """
        gx, gy = coord.to_grid(x_mm, y_mm)

        # Check all layers for track clearance
        # The obstacle map expands tracks by track_width/2 + clearance, so we only
        # need to check cells within the via drill radius
        for layer_idx in range(len(config.layers)):
            for dx in range(-via_check_radius_grid, via_check_radius_grid + 1):
                for dy in range(-via_check_radius_grid, via_check_radius_grid + 1):
                    if dx * dx + dy * dy <= via_check_radius_grid * via_check_radius_grid:
                        if obstacles.is_blocked(gx + dx, gy + dy, layer_idx):
                            return (False, f"track_blocked_layer{layer_idx}_at_({dx},{dy})")

        # Check via-to-via clearance for OTHER vias (skip the signal via we're targeting)
        for via in pcb_data.vias:
            # Skip the signal via we're placing a GND via near
            if abs(via.x - sig_via_x) < 0.01 and abs(via.y - sig_via_y) < 0.01:
                continue
            dist = math.sqrt((x_mm - via.x)**2 + (y_mm - via.y)**2)
            if dist < via_via_min_dist:
                return (False, f"too_close_to_via({dist:.2f}mm)")

        # Check clearance from through-hole pads
        for net_id, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if pad.drill and pad.drill > 0:
                    dist = math.sqrt((x_mm - pad.global_x)**2 + (y_mm - pad.global_y)**2)
                    min_pad_dist = pad.drill / 2 + config.clearance + config.via_drill / 2
                    if dist < min_pad_dist:
                        return (False, f"too_close_to_th_pad({dist:.2f}mm)")

        # Check against GND vias we're placing in this batch
        for px, py in placed_gnd_positions:
            dist = math.sqrt((x_mm - px)**2 + (y_mm - py)**2)
            if dist < via_via_min_dist:
                return (False, f"too_close_to_batch_gnd_via({dist:.2f}mm)")

        return (True, "clear")

    # Try many angles for better coverage (every 15 degrees = 24 directions)
    angles_deg = list(range(0, 360, 15))

    skipped_has_gnd = 0
    skipped_no_space = 0
    placement_distances = []  # Track distances where GND vias were placed

    for sig_via in signal_vias:
        sx, sy = sig_via.x, sig_via.y

        # Check if there's already a GND via/pad within gnd_via_distance
        has_nearby_gnd = False
        nearest_gnd_dist = float('inf')
        for gx, gy in existing_gnd_positions + placed_gnd_positions:
            dist = math.sqrt((sx - gx)**2 + (sy - gy)**2)
            nearest_gnd_dist = min(nearest_gnd_dist, dist)
            if dist <= gnd_via_distance:
                has_nearby_gnd = True
                break

        if has_nearby_gnd:
            skipped_has_gnd += 1
            continue

        # Try to place a GND via as close as possible to signal via
        best_pos = None
        distance_step = config.grid_step / 2  # Finer step for better placement

        distance = min_distance
        while distance <= gnd_via_distance:
            # Try all angles at this distance
            for angle_deg in angles_deg:
                angle_rad = math.radians(angle_deg)
                gnd_x = sx + distance * math.cos(angle_rad)
                gnd_y = sy + distance * math.sin(angle_rad)

                is_clear, reason = is_via_position_clear(gnd_x, gnd_y, sx, sy)
                if is_clear:
                    # Found a valid position at closest possible distance
                    best_pos = (gnd_x, gnd_y)
                    break

            if best_pos is not None:
                break  # Found a position, stop searching

            distance += distance_step

        if best_pos is None:
            skipped_no_space += 1
        else:
            actual_dist = math.sqrt((best_pos[0] - sx)**2 + (best_pos[1] - sy)**2)
            placement_distances.append(actual_dist)

            gnd_via = Via(
                x=best_pos[0],
                y=best_pos[1],
                size=config.via_size,
                drill=config.via_drill,
                layers=config.layers,
                net_id=gnd_net_id,
                free=True
            )
            new_gnd_vias.append(gnd_via)
            placed_gnd_positions.append(best_pos)
            existing_gnd_positions.append(best_pos)

    # Summary
    if new_gnd_vias:
        print(f"\nAdded {len(new_gnd_vias)} GND vias near signal vias")
        if placement_distances:
            avg_dist = sum(placement_distances) / len(placement_distances)
            min_placed = min(placement_distances)
            max_placed = max(placement_distances)
            print(f"  Placement distances: min={min_placed:.3f}mm, avg={avg_dist:.3f}mm, max={max_placed:.3f}mm")
            print(f"  (min_distance theoretical={min_distance:.3f}mm)")
    if skipped_has_gnd > 0:
        print(f"  {skipped_has_gnd} signal vias already had nearby GND via/pad")
    if skipped_no_space > 0:
        print(f"  {skipped_no_space} signal vias had no clear space for GND via within {gnd_via_distance}mm")
    if not new_gnd_vias and skipped_has_gnd == len(signal_vias):
        print("All signal vias already have nearby GND vias/pads")

    return new_gnd_vias
