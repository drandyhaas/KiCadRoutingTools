"""
GND via placement for single-ended routing.

Adds GND vias near signal vias for return current paths.
"""
from __future__ import annotations

import math
from typing import List, Dict

from kicad_parser import PCBData, Via, pad_drill_capsule
from geometry_utils import point_to_segment_distance
from routing_config import GridRouteConfig, GridCoord
import rust_alloc  # noqa: F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads
from grid_router import GridObstacleMap


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

    # Minimum distance from signal via center to GND via center: larger of the
    # copper ring clearance (via_size + clearance) and the drill hole-to-hole
    # minimum (via_drill + hole_to_hole_clearance), so the paired GND via clears
    # the signal via on both measures (issue #125).
    min_distance = max(config.via_size + config.clearance,
                       config.via_drill + config.hole_to_hole_clearance)

    # Debug: Print clearance parameters
    print(f"GND via placement parameters:")
    print(f"  via_size={config.via_size}mm, via_drill={config.via_drill}mm, clearance={config.clearance}mm")
    print(f"  grid_step={config.grid_step}mm")
    print(f"  min_distance (center-to-center)={min_distance:.3f}mm")

    new_gnd_vias = []
    placed_gnd_positions = []

    # Via-to-via minimum clearance (center-to-center) for batch placement: larger
    # of copper ring (via_size + clearance) and drill hole-to-hole (via_drill +
    # hole_to_hole_clearance), so a thin-ring via can't land inside the drill
    # hole-to-hole minimum (issue #125).
    via_via_min_dist = max(config.via_size + config.clearance,
                           config.via_drill + config.hole_to_hole_clearance)

    # Grid cells to check for via clearance
    # The obstacle map already expands tracks by track_width/2 + clearance.
    # We only need to check cells within via_drill/2 (the actual hole), not the full
    # via pad, since the obstacle map expansion already accounts for clearance.
    # Ceil (not floor) the hole radius so this circular clear-check doesn't miss the
    # outermost ring and let the GND-via hole land ~1 cell into a keep-out (#154 class).
    # The obstacle map expands each obstacle by (its_half + clearance +
    # TRACK_width/2) -- the body it models is a TRACK. A via's copper ring is
    # via_size/2 wide, so scanning only the DRILL radius under-enforced by
    # (via_size - track_width)/2 ~ 0.19mm and shipped 35-99um sub-clearance
    # grazes against tracks and SMD pads (Andy's bitaxe DRC2.rpt: every
    # violation was a stitching via). Scan the full ring shortfall.
    _ring_shortfall = max(config.via_drill / 2,
                          config.via_size / 2 - config.track_width / 2)
    via_check_radius_grid = max(1, coord.to_grid_dist_safe(_ring_shortfall))

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

        # Check drill hole-to-hole clearance from through-hole pad drills (a
        # physical drill-to-drill minimum -> hole_to_hole_clearance, NOT the copper
        # clearance, which under-enforces it; issue #125 PAD-DRILL-VIA-DRILL).
        for net_id, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if pad.drill and pad.drill > 0:
                    # Measure to the drill's real CAPSULE (a slot is a milled slot,
                    # not a round hole): distance to the capsule axis minus its
                    # radius. Round drills degenerate to the old centre distance.
                    (p1x, p1y), (p2x, p2y), prad = pad_drill_capsule(pad)
                    dist = point_to_segment_distance(x_mm, y_mm, p1x, p1y, p2x, p2y) - prad
                    min_pad_dist = config.hole_to_hole_clearance + config.via_drill / 2
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
                # Snap to the routing grid: the clearance check works on
                # grid CELLS, so an off-grid candidate can sit up to half a
                # cell (25um at 0.05) inside an obstacle the check calls
                # clear -- the residual 12-22um stitching-via overlaps.
                gnd_x = round((sx + distance * math.cos(angle_rad))
                              / config.grid_step) * config.grid_step
                gnd_y = round((sy + distance * math.sin(angle_rad))
                              / config.grid_step) * config.grid_step

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
                # Through-hole span like every other routed via (see above).
                layers=["F.Cu", "B.Cu"],
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
