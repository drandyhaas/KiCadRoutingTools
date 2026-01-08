"""
DRC Checker - Find overlapping tracks and vias between different nets.
"""

import sys
import argparse
import math
import fnmatch
from typing import List, Tuple, Set, Optional
from kicad_parser import parse_kicad_pcb, Segment, Via, Pad
from geometry_utils import (
    point_to_segment_distance,
    closest_point_on_segment,
    segment_to_segment_closest_points,
)
from routing_utils import expand_pad_layers


def matches_any_pattern(name: str, patterns: List[str]) -> bool:
    """Check if a net name matches any of the given patterns (fnmatch style)."""
    for pattern in patterns:
        if fnmatch.fnmatch(name, pattern):
            return True
    return False


def segment_to_segment_distance(seg1: Segment, seg2: Segment) -> float:
    """Calculate minimum distance between two segments."""
    dist, _, _ = segment_to_segment_closest_points(seg1, seg2)
    return dist


def segments_cross(seg1: Segment, seg2: Segment, tolerance: float = 0.001) -> Tuple[bool, Optional[Tuple[float, float]]]:
    """Check if two segments on the same layer cross each other.

    Returns (True, intersection_point) if they cross, (False, None) otherwise.
    Segments that share an endpoint are not considered crossing.
    """
    if seg1.layer != seg2.layer:
        return False, None

    x1, y1 = seg1.start_x, seg1.start_y
    x2, y2 = seg1.end_x, seg1.end_y
    x3, y3 = seg2.start_x, seg2.start_y
    x4, y4 = seg2.end_x, seg2.end_y

    # Check if segments share an endpoint (not a crossing)
    def points_equal(ax, ay, bx, by):
        return abs(ax - bx) < tolerance and abs(ay - by) < tolerance

    if (points_equal(x1, y1, x3, y3) or points_equal(x1, y1, x4, y4) or
        points_equal(x2, y2, x3, y3) or points_equal(x2, y2, x4, y4)):
        return False, None

    # Direction vectors
    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3

    # Cross product of direction vectors
    cross = dx1 * dy2 - dy1 * dx2

    if abs(cross) < 1e-10:
        # Parallel segments - no crossing
        return False, None

    # Solve for parameters t and u where:
    # (x1, y1) + t * (dx1, dy1) = (x3, y3) + u * (dx2, dy2)
    dx3, dy3 = x3 - x1, y3 - y1
    t = (dx3 * dy2 - dy3 * dx2) / cross
    u = (dx3 * dy1 - dy3 * dx1) / cross

    # Check if intersection is within both segments (exclusive of endpoints)
    eps = 0.001  # Small margin to exclude near-endpoint intersections
    if eps < t < 1 - eps and eps < u < 1 - eps:
        # Calculate intersection point
        ix = x1 + t * dx1
        iy = y1 + t * dy1
        return True, (ix, iy)

    return False, None


def check_segment_overlap(seg1: Segment, seg2: Segment, clearance: float, clearance_margin: float = 0.10):
    """Check if two segments on the same layer violate clearance.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).
                         Violations smaller than clearance * clearance_margin are ignored.

    Returns:
        (has_violation, overlap, closest_pt1, closest_pt2)
    """
    if seg1.layer != seg2.layer:
        return False, 0.0, None, None

    # Required distance is half-widths plus clearance
    required_dist = seg1.width / 2 + seg2.width / 2 + clearance
    actual_dist, pt1, pt2 = segment_to_segment_closest_points(seg1, seg2)
    overlap = required_dist - actual_dist

    # Use clearance-based tolerance (10% of clearance by default)
    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap, pt1, pt2
    return False, 0.0, None, None


def check_via_segment_overlap(via: Via, seg: Segment, clearance: float, clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if a via overlaps with a segment on any common layer.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).
    """
    # Standard through-hole vias go through ALL copper layers, not just the ones listed
    # Only skip non-copper layers
    if not seg.layer.endswith('.Cu'):
        return False, 0.0

    required_dist = via.size / 2 + seg.width / 2 + clearance
    actual_dist = point_to_segment_distance(via.x, via.y,
                                            seg.start_x, seg.start_y,
                                            seg.end_x, seg.end_y)
    overlap = required_dist - actual_dist

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_via_via_overlap(via1: Via, via2: Via, clearance: float, clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if two vias overlap.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).
    """
    # All vias are through-hole, so they always potentially conflict
    required_dist = via1.size / 2 + via2.size / 2 + clearance
    actual_dist = math.sqrt((via1.x - via2.x)**2 + (via1.y - via2.y)**2)
    overlap = required_dist - actual_dist

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_pad_segment_overlap(pad: Pad, seg: Segment, clearance: float,
                               routing_layers: List[str],
                               clearance_margin: float = 0.10) -> Tuple[bool, float, Optional[Tuple[float, float]]]:
    """Check if a segment is too close to a pad on the same layer.

    Args:
        pad: Pad object with global_x, global_y, size_x, size_y, layers
        seg: Segment to check against
        clearance: Minimum clearance in mm
        routing_layers: List of routing layer names (for expanding *.Cu wildcards)
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm, closest_point_on_segment)
    """
    # Expand pad layers (handles *.Cu wildcards)
    expanded_layers = expand_pad_layers(pad.layers, routing_layers)

    # Check if segment is on a layer the pad is on
    if seg.layer not in expanded_layers:
        return False, 0.0, None

    # Calculate distance from pad center to segment
    dist_to_center = point_to_segment_distance(
        pad.global_x, pad.global_y,
        seg.start_x, seg.start_y,
        seg.end_x, seg.end_y
    )

    # For rectangular pads, use the larger dimension for conservative check
    # More accurate would be to check against the actual rectangle, but this is good enough
    pad_half_size = max(pad.size_x, pad.size_y) / 2

    # Required distance: pad half-size + segment half-width + clearance
    required_dist = pad_half_size + seg.width / 2 + clearance
    overlap = required_dist - dist_to_center

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        # Get closest point on segment to pad center
        closest_pt = closest_point_on_segment(
            pad.global_x, pad.global_y,
            seg.start_x, seg.start_y,
            seg.end_x, seg.end_y
        )
        return True, overlap, closest_pt

    return False, 0.0, None


def check_pad_via_overlap(pad: Pad, via: Via, clearance: float,
                          routing_layers: List[str],
                          clearance_margin: float = 0.10) -> Tuple[bool, float]:
    """Check if a via is too close to a pad.

    Args:
        pad: Pad object
        via: Via to check against
        clearance: Minimum clearance in mm
        routing_layers: List of routing layer names (for expanding *.Cu wildcards)
        clearance_margin: Fraction of clearance to use as tolerance (default 0.10 = 10%).

    Returns:
        (has_violation, overlap_mm)
    """
    # Expand pad layers (handles *.Cu wildcards)
    expanded_layers = expand_pad_layers(pad.layers, routing_layers)

    # Vias are through-hole, so they conflict with pads on any copper layer
    if not any(layer.endswith('.Cu') for layer in expanded_layers):
        return False, 0.0

    # Distance from pad center to via center
    dist = math.sqrt((pad.global_x - via.x)**2 + (pad.global_y - via.y)**2)

    # For rectangular pads, use the larger dimension for conservative check
    pad_half_size = max(pad.size_x, pad.size_y) / 2

    # Required distance: pad half-size + via half-size + clearance
    required_dist = pad_half_size + via.size / 2 + clearance
    overlap = required_dist - dist

    tolerance = clearance * clearance_margin
    if overlap > tolerance:
        return True, overlap

    return False, 0.0


def write_debug_lines(pcb_file: str, violations: List[dict], clearance: float, layer: str = "User.7"):
    """Write debug lines to PCB file showing violation locations.

    Adds gr_line elements connecting closest points of violating segments.
    """
    import uuid

    # Read the PCB file
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Generate gr_line elements for segment-segment violations
    debug_lines = []
    print(f"\nDebug lines (center-to-center distance, required clearance = {clearance}mm):")
    for v in violations:
        if v['type'] == 'segment-segment' and 'closest_pt1' in v and v['closest_pt1']:
            pt1 = v['closest_pt1']
            pt2 = v['closest_pt2']
            dist = math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
            # Track width is typically 0.1mm, so required center-to-center = 0.1 + clearance = 0.2mm
            required = 0.1 + clearance  # half-width + half-width + clearance = track_width + clearance
            violation_amt = required - dist
            print(f"  {v['net1']} <-> {v['net2']}: dist={dist:.4f}mm, required={required:.3f}mm, violation={violation_amt:.4f}mm")
            print(f"    from ({pt1[0]:.4f}, {pt1[1]:.4f}) to ({pt2[0]:.4f}, {pt2[1]:.4f})")

            line = f'''\t(gr_line
\t\t(start {pt1[0]:.6f} {pt1[1]:.6f})
\t\t(end {pt2[0]:.6f} {pt2[1]:.6f})
\t\t(stroke
\t\t\t(width 0.05)
\t\t\t(type solid)
\t\t)
\t\t(layer "{layer}")
\t\t(uuid "{uuid.uuid4()}")
\t)'''
            debug_lines.append(line)

    if not debug_lines:
        print(f"No debug lines to write")
        return

    # Insert before the final closing paren
    debug_text = '\n'.join(debug_lines)
    last_paren = content.rfind(')')
    new_content = content[:last_paren] + '\n' + debug_text + '\n' + content[last_paren:]

    with open(pcb_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"\nWrote {len(debug_lines)} debug line(s) to layer {layer}")


def run_drc(pcb_file: str, clearance: float = 0.1, net_patterns: Optional[List[str]] = None,
            debug_output: bool = False, quiet: bool = False):
    """Run DRC checks on the PCB file.

    Args:
        pcb_file: Path to the KiCad PCB file
        clearance: Minimum clearance in mm
        net_patterns: Optional list of net name patterns (fnmatch style) to focus on.
                     If provided, only checks involving at least one matching net are reported.
        debug_output: If True, write debug lines to User.7 layer showing violation locations
        quiet: If True, only print a summary line unless there are violations
    """
    if quiet and net_patterns:
        # Print a brief summary line in quiet mode
        print(f"Checking {', '.join(net_patterns)} for DRC...", end=" ", flush=True)
    elif not quiet:
        print(f"Loading {pcb_file}...")

    pcb_data = parse_kicad_pcb(pcb_file)

    if not quiet:
        print(f"Found {len(pcb_data.segments)} segments and {len(pcb_data.vias)} vias")

    # Helper to check if a net_id matches the filter patterns
    def net_matches_filter(net_id: int) -> bool:
        if net_patterns is None:
            return True  # No filter, include all
        net_info = pcb_data.nets.get(net_id, None)
        if net_info is None:
            return False
        return matches_any_pattern(net_info.name, net_patterns)

    # Helper to check if a violation involves at least one matching net
    def violation_matches_filter(net1_str: str, net2_str: str) -> bool:
        if net_patterns is None:
            return True
        return matches_any_pattern(net1_str, net_patterns) or matches_any_pattern(net2_str, net_patterns)

    if net_patterns and not quiet:
        print(f"Filtering to nets matching: {net_patterns}")

    # Group segments and vias by net
    segments_by_net = {}
    for seg in pcb_data.segments:
        if seg.net_id not in segments_by_net:
            segments_by_net[seg.net_id] = []
        segments_by_net[seg.net_id].append(seg)

    vias_by_net = {}
    for via in pcb_data.vias:
        if via.net_id not in vias_by_net:
            vias_by_net[via.net_id] = []
        vias_by_net[via.net_id].append(via)

    violations = []

    # Check segment-to-segment violations (different nets only)
    if not quiet:
        print("\nChecking segment-to-segment clearances...")
    net_ids = list(segments_by_net.keys())

    # If filtering, only check pairs where at least one net matches
    if net_patterns:
        matching_seg_nets = [n for n in net_ids if net_matches_filter(n)]
        if not quiet:
            print(f"  Found {len(matching_seg_nets)} matching segment nets out of {len(net_ids)}")
    else:
        matching_seg_nets = None

    for i, net1 in enumerate(net_ids):
        net1_matches = matching_seg_nets is None or net1 in matching_seg_nets
        for net2 in net_ids[i+1:]:
            if net1 == net2:
                continue
            # Skip if neither net matches the filter
            net2_matches = matching_seg_nets is None or net2 in matching_seg_nets
            if not net1_matches and not net2_matches:
                continue
            for seg1 in segments_by_net[net1]:
                for seg2 in segments_by_net[net2]:
                    has_violation, overlap, pt1, pt2 = check_segment_overlap(seg1, seg2, clearance)
                    if has_violation:
                        net1_name = pcb_data.nets.get(net1, None)
                        net2_name = pcb_data.nets.get(net2, None)
                        net1_str = net1_name.name if net1_name else f"net_{net1}"
                        net2_str = net2_name.name if net2_name else f"net_{net2}"
                        violations.append({
                            'type': 'segment-segment',
                            'net1': net1_str,
                            'net2': net2_str,
                            'layer': seg1.layer,
                            'overlap_mm': overlap,
                            'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                            'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                            'closest_pt1': pt1,
                            'closest_pt2': pt2,
                        })
                    # Also check for segment crossings (different nets)
                    crosses, cross_point = segments_cross(seg1, seg2)
                    if crosses:
                        net1_name = pcb_data.nets.get(net1, None)
                        net2_name = pcb_data.nets.get(net2, None)
                        net1_str = net1_name.name if net1_name else f"net_{net1}"
                        net2_str = net2_name.name if net2_name else f"net_{net2}"
                        violations.append({
                            'type': 'segment-crossing',
                            'net1': net1_str,
                            'net2': net2_str,
                            'layer': seg1.layer,
                            'cross_point': cross_point,
                            'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                            'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                        })

    # Check for same-net segment crossings (self-intersections)
    if not quiet:
        print("Checking for same-net segment crossings...")
    for net_id in net_ids:
        if matching_seg_nets is not None and net_id not in matching_seg_nets:
            continue
        segs = segments_by_net[net_id]
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                seg1, seg2 = segs[i], segs[j]
                crosses, cross_point = segments_cross(seg1, seg2)
                if crosses:
                    net_name = pcb_data.nets.get(net_id, None)
                    net_str = net_name.name if net_name else f"net_{net_id}"
                    violations.append({
                        'type': 'segment-crossing-same-net',
                        'net1': net_str,
                        'net2': net_str,
                        'layer': seg1.layer,
                        'cross_point': cross_point,
                        'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                        'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                    })

    # Check via-to-segment violations (different nets only)
    if not quiet:
        print("Checking via-to-segment clearances...")
    via_net_ids = list(vias_by_net.keys())

    # Pre-compute matching via nets
    if net_patterns:
        matching_via_nets = set(n for n in via_net_ids if net_matches_filter(n))
        matching_seg_net_set = set(matching_seg_nets) if matching_seg_nets else set()
        if not quiet:
            print(f"  Found {len(matching_via_nets)} matching via nets out of {len(via_net_ids)}")
    else:
        matching_via_nets = None
        matching_seg_net_set = None

    for via_net in via_net_ids:
        via_net_matches = matching_via_nets is None or via_net in matching_via_nets
        for seg_net in net_ids:
            if via_net == seg_net:
                continue
            # Skip if neither net matches
            seg_net_matches = matching_seg_net_set is None or seg_net in matching_seg_net_set
            if not via_net_matches and not seg_net_matches:
                continue
            for via in vias_by_net[via_net]:
                for seg in segments_by_net.get(seg_net, []):
                    has_violation, overlap = check_via_segment_overlap(via, seg, clearance)
                    if has_violation:
                        via_net_name = pcb_data.nets.get(via_net, None)
                        seg_net_name = pcb_data.nets.get(seg_net, None)
                        via_net_str = via_net_name.name if via_net_name else f"net_{via_net}"
                        seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg_net}"
                        violations.append({
                            'type': 'via-segment',
                            'net1': via_net_str,
                            'net2': seg_net_str,
                            'layer': seg.layer,
                            'overlap_mm': overlap,
                            'via_loc': (via.x, via.y),
                            'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                        })

    # Check via-to-via violations (all nets, including same-net)
    if not quiet:
        print("Checking via-to-via clearances...")
    for i, net1 in enumerate(via_net_ids):
        net1_matches = matching_via_nets is None or net1 in matching_via_nets
        for net2 in via_net_ids[i+1:]:
            # Skip if neither net matches
            net2_matches = matching_via_nets is None or net2 in matching_via_nets
            if not net1_matches and not net2_matches:
                continue
            for via1 in vias_by_net[net1]:
                for via2 in vias_by_net[net2]:
                    # Skip if same via (can happen with same-net checking)
                    if via1 is via2:
                        continue
                    has_violation, overlap = check_via_via_overlap(via1, via2, clearance)
                    if has_violation:
                        net1_name = pcb_data.nets.get(net1, None)
                        net2_name = pcb_data.nets.get(net2, None)
                        net1_str = net1_name.name if net1_name else f"net_{net1}"
                        net2_str = net2_name.name if net2_name else f"net_{net2}"
                        violations.append({
                            'type': 'via-via' if net1 != net2 else 'via-via-same-net',
                            'net1': net1_str,
                            'net2': net2_str,
                            'overlap_mm': overlap,
                            'loc1': (via1.x, via1.y),
                            'loc2': (via2.x, via2.y),
                        })
        # Also check same-net via pairs (only if this net matches filter)
        if net1_matches and net1 in vias_by_net:
            vias_list = vias_by_net[net1]
            for j in range(len(vias_list)):
                for k in range(j + 1, len(vias_list)):
                    via1 = vias_list[j]
                    via2 = vias_list[k]
                    has_violation, overlap = check_via_via_overlap(via1, via2, clearance)
                    if has_violation:
                        net1_name = pcb_data.nets.get(net1, None)
                        net1_str = net1_name.name if net1_name else f"net_{net1}"
                        violations.append({
                            'type': 'via-via-same-net',
                            'net1': net1_str,
                            'net2': net1_str,
                            'overlap_mm': overlap,
                            'loc1': (via1.x, via1.y),
                            'loc2': (via2.x, via2.y),
                        })

    # Get routing layers for pad layer expansion
    routing_layers = list(set(seg.layer for seg in pcb_data.segments if seg.layer.endswith('.Cu')))
    if not routing_layers:
        routing_layers = ['F.Cu', 'B.Cu']  # Fallback

    # Check pad-to-segment violations (different nets only)
    if not quiet:
        print("Checking pad-to-segment clearances...")

    # Group pads by net
    pads_by_net = pcb_data.pads_by_net

    # Pre-compute matching pad nets
    pad_net_ids = list(pads_by_net.keys())
    if net_patterns:
        matching_pad_nets = set(n for n in pad_net_ids if net_matches_filter(n))
        if not quiet:
            print(f"  Found {len(matching_pad_nets)} matching pad nets out of {len(pad_net_ids)}")
    else:
        matching_pad_nets = None

    for pad_net in pad_net_ids:
        pad_net_matches = matching_pad_nets is None or pad_net in matching_pad_nets
        for seg_net in net_ids:
            if pad_net == seg_net:
                continue  # Same net - skip
            # Skip if neither net matches
            seg_net_matches = matching_seg_net_set is None or seg_net in matching_seg_net_set
            if not pad_net_matches and not seg_net_matches:
                continue
            for pad in pads_by_net.get(pad_net, []):
                for seg in segments_by_net.get(seg_net, []):
                    has_violation, overlap, closest_pt = check_pad_segment_overlap(
                        pad, seg, clearance, routing_layers
                    )
                    if has_violation:
                        pad_net_name = pcb_data.nets.get(pad_net, None)
                        seg_net_name = pcb_data.nets.get(seg_net, None)
                        pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                        seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg_net}"
                        violations.append({
                            'type': 'pad-segment',
                            'net1': pad_net_str,
                            'net2': seg_net_str,
                            'layer': seg.layer,
                            'overlap_mm': overlap,
                            'pad_loc': (pad.global_x, pad.global_y),
                            'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                            'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                            'closest_pt': closest_pt,
                        })

    # Check pad-to-via violations (different nets only)
    if not quiet:
        print("Checking pad-to-via clearances...")

    for pad_net in pad_net_ids:
        pad_net_matches = matching_pad_nets is None or pad_net in matching_pad_nets
        for via_net in via_net_ids:
            if pad_net == via_net:
                continue  # Same net - skip
            # Skip if neither net matches
            via_net_matches = matching_via_nets is None or via_net in matching_via_nets
            if not pad_net_matches and not via_net_matches:
                continue
            for pad in pads_by_net.get(pad_net, []):
                for via in vias_by_net.get(via_net, []):
                    has_violation, overlap = check_pad_via_overlap(
                        pad, via, clearance, routing_layers
                    )
                    if has_violation:
                        pad_net_name = pcb_data.nets.get(pad_net, None)
                        via_net_name = pcb_data.nets.get(via_net, None)
                        pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                        via_net_str = via_net_name.name if via_net_name else f"net_{via_net}"
                        violations.append({
                            'type': 'pad-via',
                            'net1': pad_net_str,
                            'net2': via_net_str,
                            'overlap_mm': overlap,
                            'pad_loc': (pad.global_x, pad.global_y),
                            'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                            'via_loc': (via.x, via.y),
                        })

    # Report violations
    if quiet:
        if violations:
            print(f"FAILED ({len(violations)} violations)")
        else:
            print("OK")
            return violations

    # Print detailed results (always for non-quiet, or when violations in quiet mode)
    if not quiet or violations:
        print("\n" + "=" * 60 if not quiet else "=" * 60)
        if violations:
            print(f"FOUND {len(violations)} DRC VIOLATIONS:\n")

            # Group by type
            by_type = {}
            for v in violations:
                t = v['type']
                if t not in by_type:
                    by_type[t] = []
                by_type[t].append(v)

            for vtype, vlist in by_type.items():
                print(f"\n{vtype.upper()} violations ({len(vlist)}):")
                print("-" * 40)
                for v in vlist[:20]:  # Show first 20 of each type
                    if vtype == 'segment-segment':
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Seg1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})-({v['loc1'][2]:.2f},{v['loc1'][3]:.2f})")
                        print(f"    Seg2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})-({v['loc2'][2]:.2f},{v['loc2'][3]:.2f})")
                    elif vtype == 'via-segment':
                        print(f"  Via:{v['net1']} <-> Seg:{v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-via':
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})")
                        print(f"    Via2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})")
                    elif vtype in ('segment-crossing', 'segment-crossing-same-net'):
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Layer: {v['layer']}, Cross at: ({v['cross_point'][0]:.3f},{v['cross_point'][1]:.3f})")
                        print(f"    Seg1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})-({v['loc1'][2]:.2f},{v['loc1'][3]:.2f})")
                        print(f"    Seg2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})-({v['loc2'][2]:.2f},{v['loc2'][3]:.2f})")
                    elif vtype == 'pad-segment':
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Seg:{v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'pad-via':
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Via:{v['net2']}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")

                if len(vlist) > 20:
                    print(f"  ... and {len(vlist) - 20} more")
        else:
            print("NO DRC VIOLATIONS FOUND!")

        print("=" * 60)

    # Write debug lines if requested
    if debug_output and violations:
        write_debug_lines(pcb_file, violations, clearance)

    return violations


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Check PCB for DRC violations (clearance errors)')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--clearance', '-c', type=float, default=0.1,
                        help='Minimum clearance in mm (default: 0.1)')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Optional net name patterns to focus on (fnmatch wildcards supported, e.g., "*lvds*")')
    parser.add_argument('--debug-lines', '-d', action='store_true',
                        help='Write debug lines to User.7 layer showing violation locations')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Only print a summary line unless there are violations')

    args = parser.parse_args()

    violations = run_drc(args.pcb, args.clearance, args.nets, args.debug_lines, args.quiet)
    sys.exit(1 if violations else 0)
