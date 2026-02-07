"""
Core placement algorithm for spacing components within the board boundary.

Sorts by area and places in rows left-to-right, top-to-bottom,
skipping locked components and avoiding their occupied areas.
"""

import math
import sys
from typing import List, Dict, Tuple

from kicad_parser import PCBData
from placement.parser import extract_courtyard_bboxes, extract_locked_refs
from placement.utility import compute_footprint_bbox_local, snap_to_grid


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
