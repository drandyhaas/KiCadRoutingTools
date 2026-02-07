"""
Core placement algorithm for spacing components within the board boundary.

Sorts by area and places in rows left-to-right, top-to-bottom.
"""

import sys
from typing import List, Dict

from kicad_parser import PCBData
from placement.parser import extract_courtyard_bboxes
from placement.utility import compute_footprint_bbox_local, snap_to_grid


def place_components_evenly(
    pcb_data: PCBData,
    pcb_file: str,
    grid_step: float = 0.1,
    clearance: float = 0.25,
    verbose: bool = False
) -> List[Dict]:
    """
    Place all components evenly within the board boundary.

    Sorts components largest-first and places them in rows, left-to-right,
    top-to-bottom within the usable board area.

    Args:
        pcb_data: Parsed PCB data
        pcb_file: Path to PCB file (for courtyard extraction)
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
    margin = clearance
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

    # Extract courtyard bounding boxes from file (these define the real component boundary)
    courtyard_bboxes = extract_courtyard_bboxes(pcb_file)

    # Compute bounding boxes for all footprints
    footprint_sizes = []
    for ref, fp in pcb_data.footprints.items():
        if ref in courtyard_bboxes:
            w, h = courtyard_bboxes[ref]
            if verbose:
                print(f"  {ref}: using courtyard bbox {w:.1f} x {h:.1f} mm")
        else:
            w, h = compute_footprint_bbox_local(fp)
            if verbose:
                print(f"  {ref}: using pad bbox {w:.1f} x {h:.1f} mm (no courtyard)")
        # Swap width/height if rotated 90 or 270 degrees
        rot = fp.rotation % 360
        if abs(rot - 90) < 1 or abs(rot - 270) < 1:
            w, h = h, w

        area = w * h
        footprint_sizes.append({
            'reference': ref,
            'footprint': fp,
            'width': w,
            'height': h,
            'area': area,
            'rotation': fp.rotation,  # preserve existing rotation
        })

    # Sort largest-first for better packing
    footprint_sizes.sort(key=lambda x: x['area'], reverse=True)

    print(f"Placing {len(footprint_sizes)} components...")
    if verbose and footprint_sizes:
        largest = footprint_sizes[0]
        smallest = footprint_sizes[-1]
        print(f"  Largest:  {largest['reference']} ({largest['width']:.1f} x {largest['height']:.1f} mm)")
        print(f"  Smallest: {smallest['reference']} ({smallest['width']:.1f} x {smallest['height']:.1f} mm)")

    # Row-based placement: left-to-right, top-to-bottom
    placements = []
    cursor_x = usable_min_x
    cursor_y = usable_min_y
    row_max_height = 0.0
    row_num = 1
    row_count = 0

    for item in footprint_sizes:
        w = item['width']
        h = item['height']

        # Center of component needs to be offset by half its size
        half_w = w / 2
        half_h = h / 2

        # Check if component fits in current row
        right_edge = cursor_x + w

        if right_edge > usable_max_x and row_count > 0:
            # Start new row
            cursor_y += row_max_height + clearance
            cursor_x = usable_min_x
            row_max_height = 0.0
            row_num += 1
            row_count = 0

        # Place component center at grid-snapped position
        cx = snap_to_grid(cursor_x + half_w, grid_step)
        cy = snap_to_grid(cursor_y + half_h, grid_step)

        placements.append({
            'reference': item['reference'],
            'new_x': cx,
            'new_y': cy,
            'new_rotation': item['rotation'],
        })

        if verbose:
            print(f"  {item['reference']:>6s}: ({cx:.2f}, {cy:.2f})"
                  f"  [{w:.1f} x {h:.1f} mm]  row {row_num}")

        # Advance cursor
        cursor_x += w + clearance
        row_max_height = max(row_max_height, h)
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
