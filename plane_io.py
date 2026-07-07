"""
I/O utilities for copper plane generation.

Handles reading zone information from PCB files and writing plane output.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional

from kicad_parser import PCBData, parse_kicad_pcb, _unescape_kicad_string
from kicad_writer import (generate_via_sexpr, generate_segment_sexpr, move_copper_text_to_silkscreen,
                          move_copper_graphics_to_silkscreen, add_teardrops_to_pads)


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
    # KiCad 9: (zone (net <id>) (net_name "name") (layer "layer"))
    zone_pattern = r'\(zone\s+\(net\s+(\d+)\)\s+\(net_name\s+"([^"]+)"\)\s+\(layer\s+"([^"]+)"\)'

    for match in re.finditer(zone_pattern, content):
        zones.append(ZoneInfo(
            net_id=int(match.group(1)),
            net_name=match.group(2),
            layer=match.group(3)
        ))

    if not zones:
        # KiCad 10: (zone (net "name") (layer "layer")) - no numeric ID, no net_name line
        zone_pattern_v10 = r'\(zone\s+\(net\s+"([^"]+)"\)\s+\(layer\s+"([^"]+)"\)'
        for match in re.finditer(zone_pattern_v10, content):
            zones.append(ZoneInfo(
                net_id=0,  # No numeric ID in KiCad 10
                net_name=_unescape_kicad_string(match.group(1)),
                layer=match.group(2)
            ))

    return zones


def check_existing_zones(zones: List[ZoneInfo], target_layer: str, target_net_name: str,
                          target_net_id: int, verbose: bool = False) -> Tuple[bool, bool, Optional[ZoneInfo]]:
    """Check for existing zones on the target layer.

    Args:
        zones: List of existing zones
        target_layer: Layer we want to create zone on
        target_net_name: Net name we want
        target_net_id: Net ID we want
        verbose: Print verbose output

    Returns:
        (should_create_zone, should_continue, zone_to_replace) tuple
        - should_create_zone: True to create/replace zone
        - should_continue: False if zone exists on different net (error condition)
        - zone_to_replace: ZoneInfo of existing zone to replace, or None
    """
    for zone in zones:
        if zone.layer == target_layer:
            if zone.net_id == target_net_id or zone.net_name == target_net_name:
                # Same net - replace existing zone with our parameters
                print(f"Note: Replacing existing zone on {target_layer} for net '{zone.net_name}' with new parameters")
                return (True, True, zone)
            else:
                # Different net - error
                print(f"Error: Zone already exists on {target_layer} for DIFFERENT net '{zone.net_name}' (ID {zone.net_id})")
                print(f"  Cannot create {target_net_name} zone on same layer. Aborting.")
                return (False, False, None)

    # No existing zone on this layer
    return (True, True, None)


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


def filter_nets_from_content(content: str, net_ids_to_exclude: List[int],
                             net_names_to_exclude: List[str] = None) -> str:
    """
    Filter out segments and vias for specific net IDs from PCB file content.

    Args:
        content: Raw PCB file content
        net_ids_to_exclude: List of net IDs to remove
        net_names_to_exclude: For KiCad 10, list of net names to remove

    Returns:
        Filtered content with those nets' segments and vias removed
    """
    if not net_ids_to_exclude and not net_names_to_exclude:
        return content

    net_id_set = set(net_ids_to_exclude) if net_ids_to_exclude else set()
    net_name_set = set(net_names_to_exclude) if net_names_to_exclude else set()
    lines = content.split('\n')
    result_lines = []

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Check if this starts a segment or via (may be multi-line)
        if stripped == '(segment' or stripped.startswith('(segment ') or \
           stripped == '(via' or stripped.startswith('(via '):
            # Collect all lines of this element
            element_lines = [line]
            open_parens = line.count('(') - line.count(')')
            while open_parens > 0 and i + 1 < len(lines):
                i += 1
                element_lines.append(lines[i])
                open_parens += lines[i].count('(') - lines[i].count(')')

            # Check if any line contains net ID to exclude
            element_text = '\n'.join(element_lines)
            # Try KiCad 9 format: (net <id>)
            net_match = re.search(r'\(net\s+(\d+)\)', element_text)
            if net_match:
                element_net_id = int(net_match.group(1))
                if element_net_id in net_id_set:
                    # Skip this element entirely
                    i += 1
                    continue
            elif net_name_set:
                # KiCad 10: (net "name")
                net_match_v10 = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"\)', element_text)
                # The file stores the ESCAPED name; the exclude set holds the
                # parser's unescaped names -- without unescaping, every
                # backslash-named ripped net evaded this filter and its stale
                # copper shipped OVERLAPPING the tap via route_planes placed in
                # the vacated spot (neo6502 GND-via-on-/GPIO9\OE2#, found by
                # the #319 DRC attribution; #312/#264 escaping family).
                if net_match_v10 and _unescape_kicad_string(net_match_v10.group(1)) in net_name_set:
                    i += 1
                    continue

            # Keep this element
            result_lines.extend(element_lines)
        else:
            result_lines.append(line)

        i += 1

    return '\n'.join(result_lines)


def filter_zones_from_content(content: str, zones_to_remove: List[Tuple[int, str]],
                              zone_names_to_remove: List[Tuple[str, str]] = None) -> str:
    """
    Filter out zones for specific (net_id, layer) pairs from PCB file content.

    Args:
        content: Raw PCB file content
        zones_to_remove: List of (net_id, layer) tuples to remove
        zone_names_to_remove: For KiCad 10, list of (net_name, layer) tuples to remove

    Returns:
        Filtered content with those zones removed
    """
    if not zones_to_remove and not zone_names_to_remove:
        return content

    # Build sets for fast lookup
    remove_set = set(zones_to_remove) if zones_to_remove else set()
    remove_name_set = set(zone_names_to_remove) if zone_names_to_remove else set()
    lines = content.split('\n')
    result_lines = []

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Check if this starts a zone (may be multi-line)
        if stripped == '(zone' or stripped.startswith('(zone '):
            # Collect all lines of this zone element
            element_lines = [line]
            open_parens = line.count('(') - line.count(')')
            while open_parens > 0 and i + 1 < len(lines):
                i += 1
                element_lines.append(lines[i])
                open_parens += lines[i].count('(') - lines[i].count(')')

            # Extract net_id and layer from the zone
            element_text = '\n'.join(element_lines)
            layer_match = re.search(r'\(layer\s+"([^"]+)"\)', element_text)
            # Try KiCad 9: (net <id>)
            net_match = re.search(r'\(net\s+(\d+)\)', element_text)

            if net_match and layer_match:
                zone_net_id = int(net_match.group(1))
                zone_layer = layer_match.group(1)
                if (zone_net_id, zone_layer) in remove_set:
                    # Skip this zone entirely
                    i += 1
                    continue
            elif layer_match and remove_name_set:
                # KiCad 10: (net "name")
                net_match_v10 = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"\)', element_text)
                if net_match_v10:
                    # Unescape: remove_name_set holds parser display names.
                    zone_net_name = _unescape_kicad_string(net_match_v10.group(1))
                    zone_layer = layer_match.group(1)
                    if (zone_net_name, zone_layer) in remove_name_set:
                        i += 1
                        continue

            # Keep this zone
            result_lines.extend(element_lines)
        else:
            result_lines.append(line)

        i += 1

    return '\n'.join(result_lines)


def write_plane_output(
    input_file: str,
    output_file: str,
    zone_sexpr: Optional[str],
    new_vias: List[Dict],
    new_segments: List[Dict],
    exclude_net_ids: List[int] = None,
    zones_to_replace: List[Tuple[int, str]] = None,
    add_teardrops: bool = False,
    net_id_to_name: Dict[int, str] = None
) -> bool:
    """Write the complete output file with zone (optional), vias, and traces.

    Args:
        input_file: Path to input PCB file
        output_file: Path to output PCB file
        zone_sexpr: Zone S-expression to add (or None)
        new_vias: List of via dicts with x, y, size, drill, layers, net_id
        new_segments: List of segment dicts with start, end, width, layer, net_id
        exclude_net_ids: Optional list of net IDs to exclude from output
                         (their segments/vias will be filtered out)
        zones_to_replace: Optional list of (net_id, layer) tuples for zones to
                          remove before adding new zones
        add_teardrops: Add teardrop settings to all pads

    Returns:
        True if successful, False otherwise
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)
    content = move_copper_graphics_to_silkscreen(content)

    # Add teardrops to all pads if requested
    if add_teardrops:
        print("Adding teardrop settings to pads...")
        content, teardrop_count = add_teardrops_to_pads(content)
        if teardrop_count > 0:
            print(f"  Added teardrops to {teardrop_count} pads")
        else:
            print("  All pads already have teardrop settings")

    # Filter out zones to be replaced
    if zones_to_replace:
        content = filter_zones_from_content(content, zones_to_replace)

    # Filter out excluded nets if specified.
    # Issue #88.1: on KiCad 10 boards, segments/vias reference nets by NAME
    # ((net "GND")) rather than numeric id, so filtering by id alone silently
    # leaves ripped nets' copper in the output - which then shorts against the
    # plane vias placed in the cleared spots. When a net_id->name map is
    # available, also exclude by name so KiCad 10 ripped nets are truly removed.
    if exclude_net_ids:
        exclude_names = None
        if net_id_to_name:
            exclude_names = [net_id_to_name[nid] for nid in exclude_net_ids
                             if nid in net_id_to_name]
        content = filter_nets_from_content(content, exclude_net_ids,
                                           net_names_to_exclude=exclude_names)

    # Build routing text
    elements = []

    # Add zone if provided
    if zone_sexpr:
        elements.append(zone_sexpr)

    # Add vias
    for via in new_vias:
        via_net_name = net_id_to_name.get(via['net_id']) if net_id_to_name else None
        elements.append(generate_via_sexpr(
            via['x'], via['y'], via['size'], via['drill'],
            via['layers'], via['net_id'], net_name=via_net_name
        ))

    # Add segments
    for seg in new_segments:
        seg_net_name = net_id_to_name.get(seg['net_id']) if net_id_to_name else None
        elements.append(generate_segment_sexpr(
            seg['start'], seg['end'],
            seg['width'], seg['layer'], seg['net_id'], net_name=seg_net_name
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


def _pt_seg_dist_sq(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Squared distance from point (px,py) to segment (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return (px - x1) ** 2 + (py - y1) ** 2
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = x1 + t * dx, y1 + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def _remove_vias_at_positions(content: str, positions: List[Tuple[float, float]],
                              tol: float = 2e-3) -> Tuple[str, int]:
    """Remove `(via ...)` elements whose `(at x y)` matches any of `positions`.

    Used to drop plane stitching vias that would short against restored signal
    copper (issue #88). Matching is positional with a small tolerance because
    vias are written with fixed 6-decimal precision.
    """
    if not positions:
        return content, 0
    lines = content.split('\n')
    result_lines: List[str] = []
    removed = 0
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if stripped == '(via' or stripped.startswith('(via '):
            element_lines = [line]
            open_parens = line.count('(') - line.count(')')
            while open_parens > 0 and i + 1 < len(lines):
                i += 1
                element_lines.append(lines[i])
                open_parens += lines[i].count('(') - lines[i].count(')')
            element_text = '\n'.join(element_lines)
            m = re.search(r'\(at\s+(-?[\d.]+)\s+(-?[\d.]+)', element_text)
            drop = False
            if m:
                vx, vy = float(m.group(1)), float(m.group(2))
                for px, py in positions:
                    if abs(vx - px) < tol and abs(vy - py) < tol:
                        drop = True
                        break
            if drop:
                removed += 1
                i += 1
                continue
            result_lines.extend(element_lines)
        else:
            result_lines.append(line)
        i += 1
    return '\n'.join(result_lines), removed


def _remove_segments_at(content: str, segs: List[Dict], tol: float = 2e-3) -> Tuple[str, int]:
    """Remove `(segment ...)` elements whose start/end (either orientation) and
    layer match any of `segs`. Used to drop plane connection/region traces that
    would short against restored signal copper (#140), mirroring
    `_remove_vias_at_positions` for the segment case.
    """
    if not segs:
        return content, 0
    targets = [((s['start'][0], s['start'][1]), (s['end'][0], s['end'][1]), s['layer']) for s in segs]

    def _match(ax, ay, bx, by, layer) -> bool:
        for (tx0, ty0), (tx1, ty1), tlayer in targets:
            if tlayer != layer:
                continue
            fwd = abs(ax - tx0) < tol and abs(ay - ty0) < tol and abs(bx - tx1) < tol and abs(by - ty1) < tol
            rev = abs(ax - tx1) < tol and abs(ay - ty1) < tol and abs(bx - tx0) < tol and abs(by - ty0) < tol
            if fwd or rev:
                return True
        return False

    lines = content.split('\n')
    result_lines: List[str] = []
    removed = 0
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if stripped == '(segment' or stripped.startswith('(segment '):
            element_lines = [line]
            open_parens = line.count('(') - line.count(')')
            while open_parens > 0 and i + 1 < len(lines):
                i += 1
                element_lines.append(lines[i])
                open_parens += lines[i].count('(') - lines[i].count(')')
            element_text = '\n'.join(element_lines)
            ms = re.search(r'\(start\s+(-?[\d.]+)\s+(-?[\d.]+)', element_text)
            me = re.search(r'\(end\s+(-?[\d.]+)\s+(-?[\d.]+)', element_text)
            ml = re.search(r'\(layer\s+"?([^")\s]+)', element_text)
            drop = False
            if ms and me and ml:
                drop = _match(float(ms.group(1)), float(ms.group(2)),
                              float(me.group(1)), float(me.group(2)), ml.group(1))
            if drop:
                removed += 1
                i += 1
                continue
            result_lines.extend(element_lines)
        else:
            result_lines.append(line)
        i += 1
    return '\n'.join(result_lines), removed


def restore_failed_reroute_nets(
    input_file: str,
    output_file: str,
    broken_net_ids: List[int],
    plane_vias: List[Dict],
    net_id_to_name: Optional[Dict[int, str]],
    via_size: float,
    clearance: float,
    plane_segments: Optional[List[Dict]] = None,
) -> Tuple[List[int], int, int]:
    """Issue #88: restore the ORIGINAL trace of ripped nets that failed to
    re-route, so they are never left disconnected (strictly worse than the
    input board).

    The pristine geometry is re-read from ``input_file`` because ``pcb_data``
    has the ripped nets removed in memory. Plane stitching vias that were placed
    in the cleared spots are removed where they would short against the restored
    copper, so the net is genuinely reconnected (as in the input) rather than
    shorted to the plane.

    Returns ``(restored_net_ids, plane_vias_removed)``.
    """
    orig = parse_kicad_pcb(input_file)

    restored_segs: List[Dict] = []
    restored_vias: List[Dict] = []
    restored_net_ids: List[int] = []
    for nid in broken_net_ids:
        segs = [s for s in orig.segments if s.net_id == nid]
        vias = [v for v in orig.vias if v.net_id == nid]
        if not segs and not vias:
            continue  # nothing was ever routed for this net; can't restore
        restored_net_ids.append(nid)
        for s in segs:
            restored_segs.append({'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
                                  'width': s.width, 'layer': s.layer, 'net_id': nid})
        for v in vias:
            restored_vias.append({'x': v.x, 'y': v.y, 'size': v.size, 'drill': v.drill,
                                  'layers': v.layers, 'net_id': nid})

    if not restored_net_ids:
        return [], 0, 0

    # A plane via that lands within (via_r + own_radius + clearance) of restored
    # copper is an electrical short between the signal net and the plane net.
    # Such vias must be removed so restoring the trace reconnects rather than
    # shorts. Plane vias span all layers, so segments on any layer are tested.
    via_r = via_size / 2.0
    remove_positions: List[Tuple[float, float]] = []
    for pv in plane_vias:
        pvx, pvy = pv['x'], pv['y']
        collides = False
        for rv in restored_vias:
            thresh = via_r + rv['size'] / 2.0 + clearance
            if (pvx - rv['x']) ** 2 + (pvy - rv['y']) ** 2 < thresh * thresh:
                collides = True
                break
        if not collides:
            for rs in restored_segs:
                thresh = via_r + rs['width'] / 2.0 + clearance
                if _pt_seg_dist_sq(pvx, pvy, rs['start'][0], rs['start'][1],
                                   rs['end'][0], rs['end'][1]) < thresh * thresh:
                    collides = True
                    break
        if collides:
            remove_positions.append((pvx, pvy))

    # A plane *segment* laid in the ripped net's vacated corridor likewise shorts
    # the restored copper (it stays even after a colliding via is removed — the
    # dominant DRC source on dense boards, #140). Drop plane segments that come
    # within (own_half + restored_half + clearance) of the restored copper.
    from rip_up_reroute import _seg_seg_dist_sq
    remove_segs: List[Dict] = []
    for ps in (plane_segments or []):
        (psx0, psy0), (psx1, psy1) = ps['start'], ps['end']
        ph = ps['width'] / 2.0
        collides = False
        for rs in restored_segs:
            if rs['layer'] != ps['layer']:
                continue
            thresh = ph + rs['width'] / 2.0 + clearance
            if _seg_seg_dist_sq(psx0, psy0, psx1, psy1,
                                rs['start'][0], rs['start'][1],
                                rs['end'][0], rs['end'][1]) < thresh * thresh:
                collides = True
                break
        if not collides:
            for rv in restored_vias:  # restored via spans all layers
                thresh = ph + rv['size'] / 2.0 + clearance
                if _pt_seg_dist_sq(rv['x'], rv['y'], psx0, psy0, psx1, psy1) < thresh * thresh:
                    collides = True
                    break
        if collides:
            remove_segs.append(ps)

    with open(output_file, 'r', encoding='utf-8') as f:
        content = f.read()

    content, vias_removed = _remove_vias_at_positions(content, remove_positions)
    content, segs_removed = _remove_segments_at(content, remove_segs)

    elements: List[str] = []
    for v in restored_vias:
        nm = net_id_to_name.get(v['net_id']) if net_id_to_name else None
        elements.append(generate_via_sexpr(v['x'], v['y'], v['size'], v['drill'],
                                           v['layers'], v['net_id'], net_name=nm))
    for s in restored_segs:
        nm = net_id_to_name.get(s['net_id']) if net_id_to_name else None
        elements.append(generate_segment_sexpr(s['start'], s['end'], s['width'],
                                               s['layer'], s['net_id'], net_name=nm))

    if elements:
        routing_text = '\n'.join(elements)
        last_paren = content.rfind(')')
        if last_paren != -1:
            content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)

    return restored_net_ids, vias_removed, segs_removed
