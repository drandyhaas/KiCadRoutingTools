"""
Core placement algorithms for spacing components within the board boundary.

place_components_initially: Connectivity-driven incremental placement that
minimizes airwire crossings and total wire length.

place_components_evenly: Simple row-based packing (largest-area-first fallback).
"""

import math
import os
import sys
from typing import List, Dict, Tuple, Set

from kicad_parser import PCBData, local_to_global
from connectivity import segments_intersect, compute_mst_edges
from placement.parser import extract_courtyard_bboxes, extract_locked_refs
from placement.utility import compute_footprint_bbox_local, snap_to_grid

_rust_placer_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '..', 'rust_placer')
if _rust_placer_dir not in sys.path:
    sys.path.insert(0, _rust_placer_dir)
from placement_scorer import find_best_candidate as _rust_find_best_candidate


def _get_rotated_local_bounds(ref, fp, courtyard_bboxes, verbose):
    """Get courtyard bounds relative to footprint origin, rotated to global orientation.

    Returns (rmin_x, rmin_y, rmax_x, rmax_y) — how far the courtyard extends
    from the footprint origin in each direction after applying rotation.
    """
    if ref in courtyard_bboxes:
        lmin_x, lmin_y, lmax_x, lmax_y = courtyard_bboxes[ref]
        if verbose:
            w, h = lmax_x - lmin_x, lmax_y - lmin_y
            print(f"  {ref}: courtyard {w:.1f} x {h:.1f} mm")
    else:
        lmin_x, lmin_y, lmax_x, lmax_y = compute_footprint_bbox_local(fp)
        if verbose:
            w, h = lmax_x - lmin_x, lmax_y - lmin_y
            print(f"  {ref}: pad bbox {w:.1f} x {h:.1f} mm (no courtyard)")

    # Rotate local bounds by footprint rotation
    rot = fp.rotation % 360
    if abs(rot) < 0.01:
        return lmin_x, lmin_y, lmax_x, lmax_y

    angle = math.radians(-rot)  # KiCad uses negated angles
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    corners = [
        (lmin_x, lmin_y), (lmax_x, lmin_y),
        (lmin_x, lmax_y), (lmax_x, lmax_y),
    ]
    rotated_xs = [x * cos_a - y * sin_a for x, y in corners]
    rotated_ys = [x * sin_a + y * cos_a for x, y in corners]

    return min(rotated_xs), min(rotated_ys), max(rotated_xs), max(rotated_ys)


def _rects_overlap(a_min_x, a_min_y, a_max_x, a_max_y,
                   b_min_x, b_min_y, b_max_x, b_max_y, clearance):
    """Check if two axis-aligned rectangles (given by min/max coords) overlap with clearance."""
    return not (a_max_x + clearance <= b_min_x or
                b_max_x + clearance <= a_min_x or
                a_max_y + clearance <= b_min_y or
                b_max_y + clearance <= a_min_y)


def _overlaps_any(gmin_x, gmin_y, gmax_x, gmax_y, placed_rects, clearance):
    """Check if a rectangle overlaps any already-placed rectangle."""
    for pmin_x, pmin_y, pmax_x, pmax_y in placed_rects:
        if _rects_overlap(gmin_x, gmin_y, gmax_x, gmax_y,
                          pmin_x, pmin_y, pmax_x, pmax_y, clearance):
            return True
    return False


def _get_rotated_local_bounds_for_rotation(ref, fp, courtyard_bboxes, rotation):
    """Like _get_rotated_local_bounds but takes an arbitrary rotation angle."""
    if ref in courtyard_bboxes:
        lmin_x, lmin_y, lmax_x, lmax_y = courtyard_bboxes[ref]
    else:
        lmin_x, lmin_y, lmax_x, lmax_y = compute_footprint_bbox_local(fp)

    rot = rotation % 360
    if abs(rot) < 0.01:
        return lmin_x, lmin_y, lmax_x, lmax_y

    angle = math.radians(-rot)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    corners = [
        (lmin_x, lmin_y), (lmax_x, lmin_y),
        (lmin_x, lmax_y), (lmax_x, lmax_y),
    ]
    rotated_xs = [x * cos_a - y * sin_a for x, y in corners]
    rotated_ys = [x * sin_a + y * cos_a for x, y in corners]

    return min(rotated_xs), min(rotated_ys), max(rotated_xs), max(rotated_ys)


def _build_component_net_map(pcb_data):
    """Build mapping from component reference to its set of connected net IDs."""
    comp_nets = {}
    for ref, fp in pcb_data.footprints.items():
        nets = set()
        for pad in fp.pads:
            if pad.net_id > 0:
                nets.add(pad.net_id)
        comp_nets[ref] = nets
    return comp_nets


def _compute_target_position(ref, placed_positions, comp_nets, board_center):
    """Compute weighted centroid target for placement based on connectivity."""
    my_nets = comp_nets.get(ref, set())
    if not my_nets:
        return board_center

    weighted_x = 0.0
    weighted_y = 0.0
    total_weight = 0

    for placed_ref, (px, py, _rot) in placed_positions.items():
        shared = my_nets & comp_nets.get(placed_ref, set())
        if not shared:
            continue
        weight = len(shared)
        weighted_x += px * weight
        weighted_y += py * weight
        total_weight += weight

    if total_weight == 0:
        return board_center

    return (weighted_x / total_weight, weighted_y / total_weight)


def _compute_pad_globals(fp, origin_x, origin_y, rotation):
    """Compute global (x, y, net_id) for all connected pads at a candidate placement."""
    result = []
    for pad in fp.pads:
        if pad.net_id <= 0:
            continue
        gx, gy = local_to_global(origin_x, origin_y, rotation,
                                  pad.local_x, pad.local_y)
        result.append((gx, gy, pad.net_id))
    return result


def _build_airwires_for_nets(net_pad_positions, involved_nets):
    """Build MST-based airwire segments for the specified nets.

    Returns list of ((x1,y1), (x2,y2), net_id) airwire segments.
    """
    airwires = []
    for net_id in involved_nets:
        points = net_pad_positions.get(net_id, [])
        if len(points) < 2:
            continue
        if len(points) == 2:
            airwires.append((points[0], points[1], net_id))
        else:
            edges = compute_mst_edges(points, use_manhattan=False)
            for i, j, _dist in edges:
                airwires.append((points[i], points[j], net_id))
    return airwires


def _count_crossings(new_airwires, existing_airwires):
    """Count intersections between new and existing airwires from different nets."""
    crossings = 0
    for a1, a2, net_a in new_airwires:
        for b1, b2, net_b in existing_airwires:
            if net_a == net_b:
                continue
            if segments_intersect(a1, a2, b1, b2):
                crossings += 1
    return crossings


def _compute_total_airwire_length(airwires):
    """Sum of Euclidean lengths of all airwire segments."""
    total = 0.0
    for (x1, y1), (x2, y2), _ in airwires:
        total += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return total


def place_components_initially(
    pcb_data: PCBData,
    pcb_file: str,
    grid_step: float = 0.1,
    clearance: float = 0.25,
    board_edge_clearance: float = 0.55,
    crossing_penalty: float = 10.0,
    verbose: bool = False
) -> List[Dict]:
    """
    Place components using connectivity-driven incremental placement.

    Places components in order of descending pin count. Each component is
    positioned to minimize airwire crossings and total airwire length
    relative to already-placed components. Tries 4 rotations (0/90/180/270).

    Locked components are kept in place and treated as obstacles.
    Components with no pads are skipped.
    """
    bounds = pcb_data.board_info.board_bounds
    if bounds is None:
        print("ERROR: No board boundary (Edge.Cuts) found in PCB file!")
        sys.exit(1)

    min_x, min_y, max_x, max_y = bounds
    board_w = max_x - min_x
    board_h = max_y - min_y
    board_center = ((min_x + max_x) / 2, (min_y + max_y) / 2)
    print(f"Board bounds: ({min_x:.1f}, {min_y:.1f}) to ({max_x:.1f}, {max_y:.1f})"
          f"  [{board_w:.1f} x {board_h:.1f} mm]")

    # Usable area with edge margin
    margin = max(clearance, board_edge_clearance)
    usable_min_x = min_x + margin
    usable_min_y = min_y + margin
    usable_max_x = max_x - margin
    usable_max_y = max_y - margin

    if verbose:
        print(f"Usable area: ({usable_min_x:.1f}, {usable_min_y:.1f}) to"
              f" ({usable_max_x:.1f}, {usable_max_y:.1f})")

    # Extract courtyard bounding boxes and locked status
    courtyard_bboxes = extract_courtyard_bboxes(pcb_file)
    locked_refs = extract_locked_refs(pcb_file)

    if locked_refs:
        print(f"Locked components (not moved): {', '.join(sorted(locked_refs))}")

    # Build obstacle list from locked components
    collision_clearance = max(0, clearance - grid_step)
    placed_rects: List[Tuple[float, float, float, float]] = []
    placed_positions: Dict[str, Tuple[float, float, float]] = {}

    for ref in locked_refs:
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        rmin_x, rmin_y, rmax_x, rmax_y = _get_rotated_local_bounds(
            ref, fp, courtyard_bboxes, verbose=False)
        placed_rects.append((fp.x + rmin_x, fp.y + rmin_y,
                             fp.x + rmax_x, fp.y + rmax_y))
        placed_positions[ref] = (fp.x, fp.y, fp.rotation)

    # Build component net map and initial pad positions from locked components
    comp_nets = _build_component_net_map(pcb_data)
    net_pad_positions: Dict[int, List[Tuple[float, float]]] = {}
    for ref in locked_refs:
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        for pad in fp.pads:
            if pad.net_id > 0:
                net_pad_positions.setdefault(pad.net_id, []).append(
                    (pad.global_x, pad.global_y))

    # Build initial airwires from locked components
    existing_airwires = _build_airwires_for_nets(
        net_pad_positions, set(net_pad_positions.keys()))

    # Sort unlocked components by connected pin count (most pins first)
    components_to_place = []
    skipped = []
    for ref, fp in pcb_data.footprints.items():
        if ref in locked_refs:
            continue
        if not fp.pads:
            skipped.append(ref)
            continue
        pin_count = len([p for p in fp.pads if p.net_id > 0])
        components_to_place.append({
            'reference': ref,
            'footprint': fp,
            'pin_count': pin_count,
        })

    components_to_place.sort(
        key=lambda x: (x['pin_count'], len(x['footprint'].pads)), reverse=True)

    if skipped:
        print(f"Skipped (no pads): {', '.join(sorted(skipped))}")

    print(f"Placing {len(components_to_place)} components"
          f" ({len(locked_refs)} locked)...")

    # Placement loop
    candidate_rotations = [0, 90, 180, 270]
    search_radius = 20.0  # mm
    candidate_step = 2.0  # mm
    placements = []

    # Airwires in flat format for Rust: (x1, y1, x2, y2, net_id)
    flat_airwires = [(a[0], a[1], b[0], b[1], n)
                     for (a, b, n) in existing_airwires]

    for idx, item in enumerate(components_to_place):
        ref = item['reference']
        fp = item['footprint']

        # Target position
        if idx == 0:
            target_x, target_y = board_center
        else:
            target_x, target_y = _compute_target_position(
                ref, placed_positions, comp_nets, board_center)

        # Pre-compute bounds for all 4 rotations and pad locals
        bounds_by_rotation = [
            _get_rotated_local_bounds_for_rotation(
                ref, fp, courtyard_bboxes, rot)
            for rot in candidate_rotations
        ]
        pad_locals = [(p.local_x, p.local_y, p.net_id)
                      for p in fp.pads if p.net_id > 0]

        result = _rust_find_best_candidate(
            pad_locals=pad_locals,
            bounds_by_rotation=bounds_by_rotation,
            target_x=target_x, target_y=target_y,
            search_radius=search_radius,
            candidate_step=candidate_step,
            grid_step=grid_step,
            usable_min_x=usable_min_x, usable_min_y=usable_min_y,
            usable_max_x=usable_max_x, usable_max_y=usable_max_y,
            placed_rects=placed_rects,
            collision_clearance=collision_clearance,
            net_pad_positions=net_pad_positions,
            existing_airwires=flat_airwires,
            crossing_penalty=crossing_penalty,
            is_first=(idx == 0),
            board_center_x=board_center[0],
            board_center_y=board_center[1],
        )

        if result is not None:
            origin_x, origin_y, rotation, best_score, pad_globals, new_airwires = result
        else:
            print(f"  WARNING: No valid position found for {ref},"
                  f" placing at board center")
            origin_x = snap_to_grid(board_center[0], grid_step)
            origin_y = snap_to_grid(board_center[1], grid_step)
            rotation = fp.rotation
            best_score = 0.0
            pad_globals = [(gx, gy, p.net_id)
                           for p in fp.pads if p.net_id > 0
                           for gx, gy in [local_to_global(
                               origin_x, origin_y, rotation,
                               p.local_x, p.local_y)]]
            new_airwires = []

        # Look up bounds for chosen rotation
        rot_idx = candidate_rotations.index(int(rotation))
        rmin_x, rmin_y, rmax_x, rmax_y = bounds_by_rotation[rot_idx]

        # Update state
        placements.append({
            'reference': ref,
            'new_x': origin_x,
            'new_y': origin_y,
            'new_rotation': rotation,
        })
        placed_rects.append((origin_x + rmin_x, origin_y + rmin_y,
                             origin_x + rmax_x, origin_y + rmax_y))
        placed_positions[ref] = (origin_x, origin_y, rotation)

        for gx, gy, net_id in pad_globals:
            net_pad_positions.setdefault(net_id, []).append((gx, gy))
        involved_nets = {net_id for _, _, net_id in pad_globals}
        flat_airwires = [aw for aw in flat_airwires
                         if aw[4] not in involved_nets]
        flat_airwires.extend(new_airwires)

        if verbose:
            print(f"  {ref:>6s}: ({origin_x:.2f}, {origin_y:.2f})"
                  f" rot={rotation:.0f}"
                  f"  score={best_score:.1f}"
                  f"  pins={item['pin_count']}")

    # Final stats (convert flat airwires to Python format for stats helpers)
    final_airwires = [((x1, y1), (x2, y2), n)
                      for x1, y1, x2, y2, n in flat_airwires]
    total_crossings = _count_crossings(final_airwires, final_airwires) // 2
    total_length = _compute_total_airwire_length(final_airwires)
    print(f"Placement complete: {total_crossings} total crossings,"
          f" {total_length:.1f}mm total airwire length")

    return placements


def place_components_evenly(
    pcb_data: PCBData,
    pcb_file: str,
    grid_step: float = 0.1,
    clearance: float = 0.25,
    board_edge_clearance: float = 0.0,
    verbose: bool = False
) -> List[Dict]:
    """
    Place all unlocked components evenly within the board boundary.

    Locked components are kept in place and treated as obstacles.
    Unlocked components are sorted largest-first and placed in rows,
    left-to-right, top-to-bottom, avoiding locked component areas.

    Args:
        pcb_data: Parsed PCB data
        pcb_file: Path to PCB file (for courtyard/locked extraction)
        grid_step: Grid resolution in mm
        clearance: Minimum gap between components in mm
        verbose: Print detailed placement info

    Returns:
        List of placement dicts with keys: reference, new_x, new_y, new_rotation
    """
    bounds = pcb_data.board_info.board_bounds
    if bounds is None:
        print("ERROR: No board boundary (Edge.Cuts) found in PCB file!")
        sys.exit(1)

    min_x, min_y, max_x, max_y = bounds
    board_w = max_x - min_x
    board_h = max_y - min_y
    print(f"Board bounds: ({min_x:.1f}, {min_y:.1f}) to ({max_x:.1f}, {max_y:.1f})"
          f"  [{board_w:.1f} x {board_h:.1f} mm]")

    # Usable area with edge margin
    margin = max(clearance, board_edge_clearance)
    usable_min_x = min_x + margin
    usable_min_y = min_y + margin
    usable_max_x = max_x - margin
    usable_max_y = max_y - margin
    usable_w = usable_max_x - usable_min_x
    usable_h = usable_max_y - usable_min_y

    if verbose:
        print(f"Usable area: ({usable_min_x:.1f}, {usable_min_y:.1f}) to"
              f" ({usable_max_x:.1f}, {usable_max_y:.1f})"
              f"  [{usable_w:.1f} x {usable_h:.1f} mm]")

    # Extract courtyard bounding boxes and locked status
    courtyard_bboxes = extract_courtyard_bboxes(pcb_file)
    locked_refs = extract_locked_refs(pcb_file)

    if locked_refs:
        print(f"Locked components (not moved): {', '.join(sorted(locked_refs))}")

    # Build obstacle list from locked components as global (min_x, min_y, max_x, max_y)
    placed_rects: List[Tuple[float, float, float, float]] = []
    for ref in locked_refs:
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        rmin_x, rmin_y, rmax_x, rmax_y = _get_rotated_local_bounds(
            ref, fp, courtyard_bboxes, verbose=False)
        placed_rects.append((fp.x + rmin_x, fp.y + rmin_y,
                             fp.x + rmax_x, fp.y + rmax_y))

    # Compute rotated local bounds for unlocked footprints
    footprint_sizes = []
    skipped = []
    for ref, fp in pcb_data.footprints.items():
        if ref in locked_refs:
            continue
        if not fp.pads:
            skipped.append(ref)
            continue
        rmin_x, rmin_y, rmax_x, rmax_y = _get_rotated_local_bounds(
            ref, fp, courtyard_bboxes, verbose)
        w = rmax_x - rmin_x
        h = rmax_y - rmin_y
        area = w * h
        footprint_sizes.append({
            'reference': ref,
            'footprint': fp,
            'rmin_x': rmin_x,
            'rmin_y': rmin_y,
            'rmax_x': rmax_x,
            'rmax_y': rmax_y,
            'width': w,
            'height': h,
            'area': area,
            'rotation': fp.rotation,
        })

    # Sort largest-first for better packing
    footprint_sizes.sort(key=lambda x: x['area'], reverse=True)

    if skipped:
        print(f"Skipped (no pads): {', '.join(sorted(skipped))}")

    print(f"Placing {len(footprint_sizes)} components ({len(locked_refs)} locked)...")
    if verbose and footprint_sizes:
        largest = footprint_sizes[0]
        smallest = footprint_sizes[-1]
        print(f"  Largest:  {largest['reference']} ({largest['width']:.1f} x {largest['height']:.1f} mm)")
        print(f"  Smallest: {smallest['reference']} ({smallest['width']:.1f} x {smallest['height']:.1f} mm)")

    # Row-based placement with collision avoidance
    # cursor_x, cursor_y track the top-left corner of where the next bounding box goes
    # Use reduced clearance for collision detection to account for grid snap rounding
    collision_clearance = max(0, clearance - grid_step)
    placements = []
    cursor_x = usable_min_x
    cursor_y = usable_min_y
    row_max_height = 0.0
    row_num = 1
    row_count = 0

    for item in footprint_sizes:
        w = item['width']
        h = item['height']
        rmin_x = item['rmin_x']
        rmin_y = item['rmin_y']
        rmax_x = item['rmax_x']
        rmax_y = item['rmax_y']

        # Try placing in current position, advance if it overlaps locked components
        placed = False
        while not placed:
            # Check if component fits in current row
            right_edge = cursor_x + w
            if right_edge > usable_max_x and row_count > 0:
                # Start new row
                cursor_y += row_max_height + clearance
                cursor_x = usable_min_x
                row_max_height = 0.0
                row_num += 1
                row_count = 0

            # Compute footprint origin so bounding box top-left is at cursor
            origin_x = snap_to_grid(cursor_x - rmin_x, grid_step)
            origin_y = snap_to_grid(cursor_y - rmin_y, grid_step)

            # Global bounds from snapped origin
            gmin_x = origin_x + rmin_x
            gmin_y = origin_y + rmin_y
            gmax_x = origin_x + rmax_x
            gmax_y = origin_y + rmax_y

            if _overlaps_any(gmin_x, gmin_y, gmax_x, gmax_y,
                             placed_rects, collision_clearance):
                # Skip past the overlap — advance cursor
                cursor_x += grid_step * 10  # jump forward in larger steps
                if cursor_x + w > usable_max_x:
                    cursor_y += row_max_height + clearance if row_max_height > 0 else clearance
                    cursor_x = usable_min_x
                    row_max_height = 0.0
                    row_num += 1
                    row_count = 0
                # Safety: if we've gone past the board, just place it
                if cursor_y + h > usable_max_y + board_h:
                    placed = True  # give up avoiding overlaps
            else:
                placed = True

        # Final placement position
        origin_x = snap_to_grid(cursor_x - rmin_x, grid_step)
        origin_y = snap_to_grid(cursor_y - rmin_y, grid_step)

        placements.append({
            'reference': item['reference'],
            'new_x': origin_x,
            'new_y': origin_y,
            'new_rotation': item['rotation'],
        })
        placed_rects.append((origin_x + rmin_x, origin_y + rmin_y,
                             origin_x + rmax_x, origin_y + rmax_y))

        if verbose:
            print(f"  {item['reference']:>6s}: ({origin_x:.2f}, {origin_y:.2f})"
                  f"  [{w:.1f} x {h:.1f} mm]  row {row_num}")

        # Advance cursor based on actual snapped position, not theoretical width
        cursor_x = (origin_x + rmax_x) + clearance
        row_max_height = max(row_max_height, (origin_y + rmax_y) - cursor_y)
        row_count += 1

    # Check if we overflowed the board
    final_bottom = cursor_y + row_max_height
    if final_bottom > usable_max_y:
        overflow = final_bottom - usable_max_y
        print(f"WARNING: Components overflow board boundary by {overflow:.1f} mm vertically"
              f" ({row_num} rows)")
    else:
        print(f"Placed in {row_num} row(s), {usable_max_y - final_bottom:.1f} mm vertical space remaining")

    return placements
