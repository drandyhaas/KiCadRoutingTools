"""
KiCad PCB Writer - Writes routing results to .kicad_pcb files.
"""
from __future__ import annotations

import re
import uuid
from typing import List, Dict, Tuple, Optional

from kicad_parser import Pad, is_kicad_10
from routing_utils import pos_key, POSITION_DECIMALS


def _escape_net_name(name: Optional[str]) -> Optional[str]:
    """Escape a net name for a quoted KiCad S-expr token — the exact inverse of
    kicad_parser._unescape_kicad_string (backslash then quote).

    The parser stores each net's DISPLAY name unescaped, but keys name-based net
    dedup on the RAW file text (kicad_parser.extract_nets). So a net whose name
    contains a literal backslash or quote -- e.g. `/GPIO24\\SPI1_RX(MISO)` -- must
    be re-escaped on write, or route.py emits `(net "/GPIO24\\SPI1...")` with a
    SINGLE backslash where the original board had it DOUBLE-escaped. The two raw
    tokens then parse to two different synthetic net_ids for one net, so the
    routed segment lands on a duplicate net and check_drc flags a phantom
    pad-segment/via short against the pad (still on the original id). Compounds
    every route.py step (issue #312, neo6502). Escape backslash FIRST, then quote.
    """
    if name is None:
        return None
    return name.replace('\\', '\\\\').replace('"', '\\"')


def move_copper_text_to_silkscreen(content: str) -> str:
    """
    Move gr_text elements from copper layers to silkscreen layers.

    Text on F.Cu is moved to F.SilkS, text on B.Cu is moved to B.SilkS.
    This prevents text from interfering with routing while preserving it visually.

    Args:
        content: Raw PCB file content

    Returns:
        Modified content with text layers changed
    """
    count = 0

    # Find all gr_text blocks by finding "(gr_text" and matching to closing paren
    i = 0
    result_parts = []
    last_end = 0

    while True:
        # Find next gr_text
        start = content.find('(gr_text', i)
        if start == -1:
            break

        # Find the matching closing paren by counting nesting
        depth = 0
        end = start
        for j in range(start, len(content)):
            if content[j] == '(':
                depth += 1
            elif content[j] == ')':
                depth -= 1
                if depth == 0:
                    end = j + 1
                    break

        # Extract the gr_text block
        block = content[start:end]

        # Check if it's on F.Cu or B.Cu
        layer_match = re.search(r'\(layer\s+"(F\.Cu|B\.Cu)"\)', block)
        if layer_match:
            layer = layer_match.group(1)
            new_layer = 'F.SilkS' if layer == 'F.Cu' else 'B.SilkS'
            new_block = block.replace(f'(layer "{layer}")', f'(layer "{new_layer}")')

            # Add content before this block, then the modified block
            result_parts.append(content[last_end:start])
            result_parts.append(new_block)
            last_end = end
            count += 1

        i = end

    # Add remaining content
    result_parts.append(content[last_end:])

    if count > 0:
        print(f"  Moved {count} text element(s) from copper layers to silkscreen")

    return ''.join(result_parts)


# Graphic primitives that make up copper logos/artwork (no net). Functional copper
# is zones / segments / vias / pads — never standalone graphics — so moving these
# off copper is safe and only affects decoration.
_COPPER_GRAPHIC_TAGS = (
    'fp_poly', 'gr_poly', 'fp_line', 'gr_line', 'fp_circle', 'gr_circle',
    'fp_arc', 'gr_arc', 'fp_rect', 'gr_rect', 'fp_curve', 'gr_curve',
)


def move_copper_graphics_to_silkscreen(content: str) -> str:
    """Move graphic primitives (logos / artwork drawn as polys, lines, arcs, ...)
    from copper layers to silkscreen, mirroring move_copper_text_to_silkscreen:
    F.Cu -> F.SilkS, B.Cu -> B.SilkS.

    A net-less copper graphic (e.g. a copper OSHW logo) is not modelled as an
    obstacle by the router, so plane pours and routed copper run straight over it
    and short against it (orangecrab: one F.Cu logo shorted ~30 plane traces/vias).
    Relocating it to silkscreen preserves it visually while taking it out of copper.
    """
    count = 0
    for tag in _COPPER_GRAPHIC_TAGS:
        token = '(' + tag
        tlen = len(token)
        result_parts = []
        last_end = 0
        i = 0
        while True:
            start = content.find(token, i)
            if start == -1:
                break
            # Require a token boundary so '(gr_line' doesn't match '(gr_linex'.
            nxt = content[start + tlen: start + tlen + 1]
            if nxt and (nxt.isalnum() or nxt == '_'):
                i = start + tlen
                continue
            depth = 0
            end = start
            for j in range(start, len(content)):
                if content[j] == '(':
                    depth += 1
                elif content[j] == ')':
                    depth -= 1
                    if depth == 0:
                        end = j + 1
                        break
            block = content[start:end]
            layer_match = re.search(r'\(layer\s+"(F\.Cu|B\.Cu)"\)', block)
            if layer_match:
                layer = layer_match.group(1)
                new_layer = 'F.SilkS' if layer == 'F.Cu' else 'B.SilkS'
                block = block.replace(f'(layer "{layer}")', f'(layer "{new_layer}")')
                count += 1
            result_parts.append(content[last_end:start])
            result_parts.append(block)
            last_end = end
            i = end
        result_parts.append(content[last_end:])
        content = ''.join(result_parts)

    if count > 0:
        print(f"  Moved {count} copper graphic(s)/logo(s) to silkscreen")

    return content


def generate_segment_sexpr(start: Tuple[float, float], end: Tuple[float, float],
                           width: float, layer: str, net_id: int,
                           net_name: str = None) -> str:
    """Generate KiCad S-expression for a track segment.

    Args:
        net_name: If provided, output KiCad 10 format (net "name") instead of (net id).
    """
    net_str = f'(net "{_escape_net_name(net_name)}")' if net_name is not None else f'(net {net_id})'
    return f'''	(segment
		(start {start[0]:.6f} {start[1]:.6f})
		(end {end[0]:.6f} {end[1]:.6f})
		(width {width})
		(layer "{layer}")
		{net_str}
		(uuid "{uuid.uuid4()}")
	)'''


def generate_gr_line_sexpr(start: Tuple[float, float], end: Tuple[float, float],
                           width: float, layer: str) -> str:
    """Generate KiCad S-expression for a graphic line (for non-copper layers)."""
    return f'''	(gr_line
		(start {start[0]:.6f} {start[1]:.6f})
		(end {end[0]:.6f} {end[1]:.6f})
		(stroke
			(width {width})
			(type solid)
		)
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
	)'''


def generate_via_sexpr(x: float, y: float, size: float, drill: float,
                       layers: List[str], net_id: int, free: bool = False,
                       net_name: str = None) -> str:
    """Generate KiCad S-expression for a via.

    Args:
        free: If True, adds (free yes) to prevent KiCad from auto-assigning net based on overlapping tracks.
        net_name: If provided, output KiCad 10 format (net "name") instead of (net id).
    """
    layers_str = '" "'.join(layers)
    free_str = "\n\t\t(free yes)" if free else ""
    net_str = f'(net "{_escape_net_name(net_name)}")' if net_name is not None else f'(net {net_id})'
    # KiCad 10 adds structured tenting/covering/plugging fields after layers
    if net_name is not None:
        tenting_str = "\n\t\t(tenting (front yes) (back yes))"
    else:
        tenting_str = ""
    return f'''	(via
		(at {x:.6f} {y:.6f})
		(size {size})
		(drill {drill})
		(layers "{layers_str}"){tenting_str}{free_str}
		{net_str}
		(uuid "{uuid.uuid4()}")
	)'''


def generate_gr_text_sexpr(text: str, x: float, y: float, layer: str,
                           size: float = 0.5, angle: float = 0) -> str:
    """Generate KiCad S-expression for a graphic text label."""
    return f'''	(gr_text "{text}"
		(at {x:.6f} {y:.6f} {angle})
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
		(effects
			(font
				(size {size} {size})
				(thickness 0.1)
			)
		)
	)'''


def generate_zone_sexpr(
    net_id: int,
    net_name: str,
    layer: str,
    polygon_points: List[Tuple[float, float]],
    clearance: float = 0.2,
    min_thickness: float = 0.1,
    thermal_gap: float = 0.2,
    thermal_bridge_width: float = 0.2,
    direct_connect: bool = True,
    use_net_name: bool = False
) -> str:
    """Generate KiCad S-expression for a filled copper zone.

    Args:
        net_id: Net ID (e.g., 690 for GND)
        net_name: Net name string (e.g., "GND")
        layer: Copper layer (e.g., "B.Cu", "In1.Cu")
        polygon_points: List of (x, y) coordinates defining zone boundary
        clearance: Clearance from other copper in mm
        min_thickness: Minimum copper thickness in mm
        thermal_gap: Gap for thermal relief spokes
        thermal_bridge_width: Width of thermal bridges
        direct_connect: If True, use solid/direct pad connections; if False, use thermal relief
        use_net_name: If True, output KiCad 10 format (net "name") instead of (net id)

    Returns:
        S-expression string for the zone
    """
    # Format polygon points
    pts_lines = []
    for x, y in polygon_points:
        pts_lines.append(f"(xy {x:.6f} {y:.6f})")
    pts_str = " ".join(pts_lines)

    # Connect pads mode: "yes" for direct solid connection, omit for thermal relief
    if direct_connect:
        connect_pads_str = f'''(connect_pads yes
		(clearance {clearance})
	)'''
    else:
        connect_pads_str = f'''(connect_pads
		(clearance {clearance})
	)'''

    # KiCad 10: (net "name"), no (net_name ...) line; KiCad 9: (net id) + (net_name "name")
    if use_net_name:
        net_lines = f'(net "{_escape_net_name(net_name)}")'
    else:
        net_lines = f'(net {net_id})\n\t\t(net_name "{_escape_net_name(net_name)}")'

    # KiCad 10 removes (filled_areas_thickness no) and adds (island_removal_mode 0) in fill
    if use_net_name:
        fill_block = f'''(fill yes
			(thermal_gap {thermal_gap})
			(thermal_bridge_width {thermal_bridge_width})
			(island_removal_mode 0)
		)'''
        extra_zone_props = ''
    else:
        fill_block = f'''(fill yes
			(thermal_gap {thermal_gap})
			(thermal_bridge_width {thermal_bridge_width})
		)'''
        extra_zone_props = '\n\t\t(filled_areas_thickness no)'

    return f'''	(zone
		{net_lines}
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
		(hatch edge 0.5)
		{connect_pads_str}
		(min_thickness {min_thickness}){extra_zone_props}
		{fill_block}
		(polygon
			(pts
				{pts_str}
			)
		)
	)'''


def add_tracks_to_pcb(input_path: str, output_path: str, tracks: List[Dict],
                      net_id_to_name: Dict[int, str] = None) -> bool:
    """
    Add track segments to a PCB file.

    Args:
        input_path: Path to original .kicad_pcb file
        output_path: Path for output file
        tracks: List of track dicts with keys: start, end, width, layer, net_id
        net_id_to_name: For KiCad 10, mapping of synthetic net IDs to net names

    Returns:
        True if successful
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Match the input board's net-token format (see add_tracks_and_vias_to_pcb):
    # name-only refs only for name-net boards, numeric (net <id>) for KiCad 9.
    from kicad_parser import board_uses_name_nets, detect_kicad_version, extract_nets
    if board_uses_name_nets(content):
        if net_id_to_name is None:
            nets, _ = extract_nets(content, detect_kicad_version(content))
            net_id_to_name = {nid: n.name for nid, n in nets.items()}
    else:
        net_id_to_name = None

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)
    # Move copper logos/artwork to silkscreen too (net-less copper graphics would
    # otherwise short against routed/poured copper).
    content = move_copper_graphics_to_silkscreen(content)

    # Generate segment S-expressions
    segments = []
    for track in tracks:
        track_net_name = net_id_to_name.get(track['net_id']) if net_id_to_name else None
        seg = generate_segment_sexpr(
            track['start'],
            track['end'],
            track['width'],
            track['layer'],
            track['net_id'],
            net_name=track_net_name
        )
        segments.append(seg)

    routing_text = '\n'.join(segments)

    if not routing_text.strip():
        print("Warning: No routing elements to add")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    # Find the last closing parenthesis
    last_paren = content.rfind(')')

    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    # Insert routing before the final closing paren
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    # Write output file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def add_tracks_and_vias_to_pcb(input_path: str, output_path: str,
                               tracks: List[Dict], vias: List[Dict] = None,
                               remove_vias: List[Dict] = None,
                               net_id_to_name: Dict[int, str] = None) -> bool:
    """
    Add track segments and vias to a PCB file, optionally removing existing vias.

    Args:
        input_path: Path to original .kicad_pcb file
        output_path: Path for output file
        tracks: List of track dicts with keys: start, end, width, layer, net_id
        vias: List of via dicts with keys: x, y, size, drill, layers, net_id
        remove_vias: List of via dicts with keys: x, y (position to match for removal)

    Returns:
        True if successful
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Keep the output's net-token format consistent with the input board. Only
    # emit name-only (net "name") refs when the board actually uses them; for a
    # KiCad 9 numeric board write (net <id>) and IGNORE any name map the caller
    # passed (the fanout tools pass one unconditionally - issue #80 fix went the
    # other way and injected KiCad-10 name nets into KiCad-9 boards, producing a
    # hybrid file KiCad 9 reads as net-less).
    from kicad_parser import board_uses_name_nets, detect_kicad_version, extract_nets
    if board_uses_name_nets(content):
        if net_id_to_name is None:
            nets, _ = extract_nets(content, detect_kicad_version(content))
            net_id_to_name = {nid: n.name for nid, n in nets.items()}
    else:
        net_id_to_name = None

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)
    # Move copper logos/artwork to silkscreen too (net-less copper graphics would
    # otherwise short against routed/poured copper).
    content = move_copper_graphics_to_silkscreen(content)

    # Remove existing vias if specified
    if remove_vias:
        import re
        removed_count = 0
        for via_to_remove in remove_vias:
            x, y = via_to_remove['x'], via_to_remove['y']
            # Match via at this position
            # KiCad format: (via\n\t\t(at X Y)\n\t\t(size ...)\n\t\t(drill ...)\n\t\t(layers ...)\n\t\t(net ...)\n\t\t(uuid ...)\n\t)
            # Build pattern that matches the multi-line via block
            x_str = f"{x:.6f}".rstrip('0').rstrip('.')
            y_str = f"{y:.6f}".rstrip('0').rstrip('.')

            # Also try integer format if coordinates are whole numbers
            x_patterns = [re.escape(x_str)]
            y_patterns = [re.escape(y_str)]
            if x == int(x):
                x_patterns.append(str(int(x)))
            if y == int(y):
                y_patterns.append(str(int(y)))

            # Build pattern that matches via block with any of the coordinate formats
            for x_pat in x_patterns:
                for y_pat in y_patterns:
                    # Match entire via block from opening to closing parenthesis
                    # Use non-greedy match for content between (at ...) and final )
                    pattern = rf'\t\(via\s*\n\s*\(at\s+{x_pat}\s+{y_pat}\)[\s\S]*?\n\t\)'
                    new_content = re.sub(pattern, '', content)
                    if new_content != content:
                        content = new_content
                        removed_count += 1
                        break
                else:
                    continue
                break
        if removed_count > 0:
            print(f"  Removed {removed_count} vias from file")

    elements = []

    # Generate segment S-expressions
    for track in tracks:
        track_net_name = net_id_to_name.get(track['net_id']) if net_id_to_name else None
        seg = generate_segment_sexpr(
            track['start'],
            track['end'],
            track['width'],
            track['layer'],
            track['net_id'],
            net_name=track_net_name
        )
        elements.append(seg)

    # Generate via S-expressions
    if vias:
        for via in vias:
            via_net_name = net_id_to_name.get(via['net_id']) if net_id_to_name else None
            v = generate_via_sexpr(
                via['x'],
                via['y'],
                via['size'],
                via['drill'],
                via['layers'],
                via['net_id'],
                via.get('free', False),
                net_name=via_net_name
            )
            elements.append(v)

    routing_text = '\n'.join(elements)

    if not routing_text.strip():
        print("Warning: No routing elements to add")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    # Find the last closing parenthesis
    last_paren = content.rfind(')')

    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    # Insert routing before the final closing paren
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    # Write output file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def modify_segment_layers(content: str, segment_mods: List[Dict]) -> Tuple[str, int]:
    """
    Modify the layer of existing segments in the KiCad PCB content.

    Args:
        content: KiCad PCB file content
        segment_mods: List of dicts with keys:
            - start: (x, y) tuple
            - end: (x, y) tuple
            - net_id: int
            - old_layer: str (optional, for verification)
            - new_layer: str

    Returns:
        (modified_content, count_of_modified_segments)
    """
    if not segment_mods:
        return content, 0

    # Build a lookup for segment modifications
    def coord_key(x, y):
        return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS))

    # Per-net lookups: KiCad 9 segments carry a numeric net id, KiCad 10 segments
    # carry the net NAME, so index mods under both. Values are LISTS in append
    # order so chained swaps resolve to the final layer (last mod wins per net).
    mod_lookup_by_id = {}
    mod_lookup_by_name = {}
    # Coordinate-only fallback (net changed between mod recording and writing,
    # e.g. target swaps reassigned the stub's net).
    mod_lookup_by_coords = {}
    for mod in segment_mods:
        start_key = coord_key(mod['start'][0], mod['start'][1])
        end_key = coord_key(mod['end'][0], mod['end'][1])
        # Also store reverse order since segment endpoints can be swapped
        for ckey in ((start_key, end_key), (end_key, start_key)):
            mod_lookup_by_id.setdefault(ckey + (mod['net_id'],), []).append(mod)
            if mod.get('net_name'):
                mod_lookup_by_name.setdefault(ckey + (mod['net_name'],), []).append(mod)
            mod_lookup_by_coords.setdefault(ckey, []).append(mod)

    count = 0

    # Pattern to match segment blocks - handle both KiCad 9 (net <id>) and KiCad 10 (net "name")
    segment_pattern = re.compile(
        r'(\(segment\s*\n?\s*'
        r'\(start\s+([\d.-]+)\s+([\d.-]+)\)\s*\n?\s*'
        r'\(end\s+([\d.-]+)\s+([\d.-]+)\)\s*\n?\s*'
        r'\(width\s+[\d.]+\)\s*\n?\s*'
        r'\(layer\s+")([^"]+)("\)\s*\n?\s*'
        r'\(net\s+(?:(\d+)|"([^"]*)")\))',
        re.MULTILINE
    )

    def replace_layer(match):
        nonlocal count
        full_match = match.group(0)
        start_x = float(match.group(2))
        start_y = float(match.group(3))
        end_x = float(match.group(4))
        end_y = float(match.group(5))
        layer = match.group(6)
        net_id = int(match.group(8)) if match.group(8) else None
        net_name = match.group(9)  # KiCad 10: (net "name")

        start_key = coord_key(start_x, start_y)
        end_key = coord_key(end_x, end_y)
        coord_only_key = (start_key, end_key)

        # First try an exact per-net match (numeric id for KiCad 9 files, net
        # name for KiCad 10 files). Take the LAST mod so chained swaps land on
        # the final layer. Matching by net matters: two nets can carry
        # identical-geometry stub segments (stacked fanout escapes), and the
        # coordinate-only fallback then applies one net's mod to the other's
        # segment, leaving copper on the wrong layer mid-run (issue #264).
        mods = None
        if net_id is not None:
            mods = mod_lookup_by_id.get(coord_only_key + (net_id,))
        if mods is None and net_name is not None:
            mods = mod_lookup_by_name.get(coord_only_key + (net_name,))
        mod = mods[-1] if mods else None
        # Fallback: coordinate match with the net left unmatched (the net was
        # reassigned between recording and writing, e.g. by target swaps). Only
        # accept mods whose old_layer equals THIS segment's current layer, so a
        # twin segment of another net (different layer) never steals the mod.
        if mod is None and coord_only_key in mod_lookup_by_coords:
            cands = [m for m in mod_lookup_by_coords[coord_only_key]
                     if m.get('old_layer') == layer]
            mod = cands[-1] if cands else None

        if mod:
            new_layer = mod['new_layer']
            if layer != new_layer:
                count += 1
                # Replace the layer in the match
                return full_match.replace(f'(layer "{layer}")', f'(layer "{new_layer}")')
        return full_match

    result = segment_pattern.sub(replace_layer, content)
    return result, count


def swap_segment_nets_at_positions(content: str, positions: set,
                                   old_net_id: int, new_net_id: int,
                                   layer: str = None,
                                   old_net_name: str = None,
                                   new_net_name: str = None) -> Tuple[str, int]:
    """
    Swap net IDs of segments that have endpoints at the given positions.

    Args:
        content: KiCad file content
        positions: Set of position keys to match
        old_net_id: Net ID to replace
        new_net_id: Net ID to replace with
        layer: Optional layer name to filter segments. When specified, only segments
               on this layer are swapped. This prevents incorrectly swapping
               stubs that share XY coordinates but are on different layers.
        old_net_name: For KiCad 10, the net name to match/replace
        new_net_name: For KiCad 10, the replacement net name

    Returns (modified_content, count_of_swapped_segments).
    """
    use_names = old_net_name is not None and new_net_name is not None

    if use_names:
        # KiCad 10: match (net "name")
        segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width[^)]*\)\s+\(layer\s+"?([^")]+)"?\).*?\(net\s+"([^"]*)"\)'
    else:
        # KiCad 9: match (net <id>)
        segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width[^)]*\)\s+\(layer\s+"?([^")]+)"?\).*?\(net\s+(\d+)\)'

    count = 0

    def replace_net(match):
        nonlocal count
        start_x, start_y = float(match.group(1)), float(match.group(2))
        end_x, end_y = float(match.group(3)), float(match.group(4))
        seg_layer = match.group(5)

        start_key = pos_key(start_x, start_y)
        end_key = pos_key(end_x, end_y)

        # Check if this segment has endpoints in our position set and correct net
        if use_names:
            seg_net_name = match.group(6)
            matches_net = seg_net_name == old_net_name
        else:
            seg_net_id = int(match.group(6))
            matches_net = seg_net_id == old_net_id

        if matches_net and (start_key in positions or end_key in positions):
            if layer is None or seg_layer == layer:
                count += 1
                if use_names:
                    return match.group(0).replace(f'(net "{old_net_name}")', f'(net "{new_net_name}")')
                else:
                    return match.group(0).replace(f'(net {old_net_id})', f'(net {new_net_id})')
        return match.group(0)

    result = re.sub(segment_pattern, replace_net, content, flags=re.DOTALL)
    return result, count


def remove_segments_from_content(content: str, segments: List,
                                 net_id_to_name: Dict = None) -> Tuple[str, int]:
    """Delete whole ``(segment ...)`` blocks matching the given segments.

    Used by the dead-end sweep (issue #84) to strip original input-file copper
    that prune_dead_end_segments flagged as a dead end -- copper the writer would
    otherwise carry into the output verbatim. A segment is matched by its
    unordered endpoint pair plus layer and net, which uniquely identifies it
    (a duplicate with the same endpoints/layer/net is interchangeable). The whole
    block (including its trailing uuid) is removed, not just the net token.

    Returns ``(modified_content, count_removed)``.
    """
    if not segments:
        return content, 0

    use_names = net_id_to_name is not None and is_kicad_10(content)

    def seg_key(p1, p2, layer, net_token):
        return (frozenset((p1, p2)), layer, net_token)

    targets = set()
    for s in segments:
        net_token = (net_id_to_name.get(s.net_id) if use_names else s.net_id)
        targets.add(seg_key(pos_key(s.start_x, s.start_y),
                            pos_key(s.end_x, s.end_y), s.layer, net_token))

    start_re = re.compile(r'\(start\s+([\d.-]+)\s+([\d.-]+)\)')
    end_re = re.compile(r'\(end\s+([\d.-]+)\s+([\d.-]+)\)')
    layer_re = re.compile(r'\(layer\s+"?([^")]+)"?\)')
    net_name_re = re.compile(r'\(net\s+"([^"]*)"\)')
    net_id_re = re.compile(r'\(net\s+(\d+)\)')

    # Scan top-level (segment ...) blocks with a quote-aware paren matcher. A
    # plain nested-paren regex breaks on KiCad v10 auto-net names that contain
    # parentheses, e.g. (net "Net-(R2-Pad1)") -- the inner parens are inside a
    # string and must not count toward block nesting.
    out = []
    pos = 0
    n = len(content)
    count = 0
    while True:
        j = content.find('(segment', pos)
        if j < 0:
            out.append(content[pos:])
            break
        out.append(content[pos:j])
        depth = 0
        in_str = False
        k = j
        while k < n:
            ch = content[k]
            if in_str:
                if ch == '"':
                    in_str = False
            elif ch == '"':
                in_str = True
            elif ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    k += 1
                    break
            k += 1
        block = content[j:k]
        ms, me, ml = start_re.search(block), end_re.search(block), layer_re.search(block)
        keep = True
        if ms and me and ml:
            if use_names:
                mn = net_name_re.search(block)
                net_token = mn.group(1) if mn else None
            else:
                mn = net_id_re.search(block)
                net_token = int(mn.group(1)) if mn else None
            key = seg_key(pos_key(float(ms.group(1)), float(ms.group(2))),
                          pos_key(float(me.group(1)), float(me.group(2))),
                          ml.group(1), net_token)
            if key in targets:
                keep = False
                count += 1
        if keep:
            out.append(block)
        pos = k
    return ''.join(out), count


def remove_vias_from_content(content: str, vias: List,
                             net_id_to_name: Dict = None) -> Tuple[str, int]:
    """Delete whole ``(via ...)`` blocks matching the given vias.

    Via twin of remove_segments_from_content: strips original input-file vias
    of a ripped/re-routed net that are no longer on the final board (#103 rip-
    existing; without this the old via AND its reroute's replacement both ship,
    stacking same-net drill pairs). Matched by position + net token; duplicates
    are interchangeable. Returns ``(modified_content, count_removed)``.
    """
    if not vias:
        return content, 0

    use_names = net_id_to_name is not None and is_kicad_10(content)

    targets = set()
    for v in vias:
        net_token = (net_id_to_name.get(v.net_id) if use_names else v.net_id)
        targets.add((pos_key(v.x, v.y), net_token))

    at_re = re.compile(r'\(at\s+([\d.-]+)\s+([\d.-]+)\)')
    net_name_re = re.compile(r'\(net\s+"([^"]*)"\)')
    net_id_re = re.compile(r'\(net\s+(\d+)\)')

    out = []
    pos = 0
    n = len(content)
    count = 0
    while True:
        j = content.find('(via', pos)
        # skip non-via tokens sharing the prefix: (vias allowed) in rule areas
        while j >= 0 and j + 4 < n and content[j + 4] not in ' \t\r\n(':
            j = content.find('(via', j + 4)
        if j < 0:
            out.append(content[pos:])
            break
        out.append(content[pos:j])
        depth = 0
        in_str = False
        k = j
        while k < n:
            ch = content[k]
            if in_str:
                if ch == '"':
                    in_str = False
            elif ch == '"':
                in_str = True
            elif ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    k += 1
                    break
            k += 1
        block = content[j:k]
        ma = at_re.search(block)
        keep = True
        if ma:
            if use_names:
                mn = net_name_re.search(block)
                net_token = mn.group(1) if mn else None
            else:
                mn = net_id_re.search(block)
                net_token = int(mn.group(1)) if mn else None
            if (pos_key(float(ma.group(1)), float(ma.group(2))), net_token) in targets:
                keep = False
                count += 1
        if keep:
            out.append(block)
        pos = k
    return ''.join(out), count


def swap_via_nets_at_positions(content: str, positions: set,
                               old_net_id: int, new_net_id: int,
                               tolerance: float = 0.02,
                               old_net_name: str = None,
                               new_net_name: str = None) -> Tuple[str, int]:
    """
    Swap net IDs of vias that are at the given positions.

    Uses tolerance-based matching to handle slight coordinate differences
    between vias and segment endpoints (e.g., original vias may be slightly
    off-grid from routed segments).

    Args:
        old_net_name: For KiCad 10, the net name to match/replace
        new_net_name: For KiCad 10, the replacement net name

    Returns (modified_content, count_of_swapped_vias).
    """
    use_names = old_net_name is not None and new_net_name is not None

    if use_names:
        via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\).*?\(net\s+"([^"]*)"\)'
    else:
        via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\).*?\(net\s+(\d+)\)'

    count = 0

    def is_near_any_position(x, y, positions, tol):
        """Check if (x, y) is within tolerance of any position in the set."""
        for px, py in positions:
            if abs(x - px) < tol and abs(y - py) < tol:
                return True
        return False

    def replace_net(match):
        nonlocal count
        via_x, via_y = float(match.group(1)), float(match.group(2))

        if use_names:
            via_net_name = match.group(3)
            matches_net = via_net_name == old_net_name
        else:
            via_net_id = int(match.group(3))
            matches_net = via_net_id == old_net_id

        if matches_net and is_near_any_position(via_x, via_y, positions, tolerance):
            count += 1
            if use_names:
                return match.group(0).replace(f'(net "{old_net_name}")', f'(net "{new_net_name}")')
            else:
                return match.group(0).replace(f'(net {old_net_id})', f'(net {new_net_id})')
        return match.group(0)

    result = re.sub(via_pattern, replace_net, content, flags=re.DOTALL)
    return result, count


def add_teardrops_to_pads(content: str,
                          best_length_ratio: float = 0.5,
                          max_length: float = 1.0,
                          best_width_ratio: float = 1.0,
                          max_width: float = 2.0,
                          curved_edges: bool = False,
                          filter_ratio: float = 0.9,
                          allow_two_segments: bool = True,
                          prefer_zone_connections: bool = True) -> tuple[str, int]:
    """
    Add teardrop settings to all pads that don't already have them.

    Teardrops are added before the (uuid ...) line in each pad definition.

    Args:
        content: KiCad PCB file content
        best_length_ratio: Target length as ratio of track width (default 0.5)
        max_length: Maximum teardrop length in mm (default 1.0)
        best_width_ratio: Target width as ratio of pad size (default 1.0)
        max_width: Maximum teardrop width in mm (default 2.0)
        curved_edges: Use curved teardrop edges (default False)
        filter_ratio: Min ratio of teardrop to pad size to apply (default 0.9)
        allow_two_segments: Allow teardrops with two segments (default True)
        prefer_zone_connections: Prefer zone connections over teardrops (default True)

    Returns:
        (modified_content, count_of_pads_with_teardrops_added)
    """
    curved_str = "yes" if curved_edges else "no"
    two_seg_str = "yes" if allow_two_segments else "no"
    zone_str = "yes" if prefer_zone_connections else "no"

    teardrop_block = f'''(teardrops
				(best_length_ratio {best_length_ratio})
				(max_length {max_length})
				(best_width_ratio {best_width_ratio})
				(max_width {max_width})
				(curved_edges {curved_str})
				(filter_ratio {filter_ratio})
				(enabled yes)
				(allow_two_segments {two_seg_str})
				(prefer_zone_connections {zone_str})
			)
			'''

    count = 0
    result_parts = []
    last_end = 0
    i = 0

    while True:
        # Find next pad definition
        pad_start = content.find('(pad ', i)
        if pad_start == -1:
            break

        # Find the end of this pad block by counting parens
        depth = 0
        pad_end = pad_start
        for j in range(pad_start, len(content)):
            if content[j] == '(':
                depth += 1
            elif content[j] == ')':
                depth -= 1
                if depth == 0:
                    pad_end = j + 1
                    break

        pad_block = content[pad_start:pad_end]

        # Check if this pad already has teardrops
        if '(teardrops' not in pad_block:
            # Find the (uuid line to insert before it
            uuid_pos = pad_block.find('(uuid ')
            if uuid_pos != -1:
                # Insert teardrop block before uuid
                new_pad_block = pad_block[:uuid_pos] + teardrop_block + pad_block[uuid_pos:]
                result_parts.append(content[last_end:pad_start])
                result_parts.append(new_pad_block)
                last_end = pad_end
                count += 1

        i = pad_end

    # Add remaining content
    result_parts.append(content[last_end:])

    return ''.join(result_parts), count


def swap_pad_nets_in_content(content: str, pad1: Pad, pad2: Pad) -> str:
    """
    Swap the net assignments of two pads in the KiCad PCB file content.

    Finds each pad by its component reference and pad number, then swaps their (net ...) declarations.
    """
    def find_pad_net_in_footprint(content: str, component_ref: str, pad_number: str) -> Optional[Tuple[int, int, str]]:
        """Find the (net id "name") part of a pad and return (start, end, match_text)."""
        fp_start_pattern = r'\(footprint\s+"[^"]*"'

        for fp_match in re.finditer(fp_start_pattern, content):
            fp_start = fp_match.start()
            # Find the end of this footprint block
            depth = 0
            fp_end = fp_start
            for i, char in enumerate(content[fp_start:], fp_start):
                if char == '(':
                    depth += 1
                elif char == ')':
                    depth -= 1
                    if depth == 0:
                        fp_end = i + 1
                        break

            fp_text = content[fp_start:fp_end]

            # Check if this footprint has the right Reference
            ref_pattern = rf'\(property\s+"Reference"\s+"{re.escape(component_ref)}"'
            if not re.search(ref_pattern, fp_text):
                continue

            # Find the pad with this number in this footprint
            pad_pattern = rf'\(pad\s+"{re.escape(pad_number)}"\s+\w+\s+\w+'
            for pad_match in re.finditer(pad_pattern, fp_text):
                pad_start_rel = pad_match.start()
                # Find end of this pad block
                depth = 0
                pad_end_rel = pad_start_rel
                for i, char in enumerate(fp_text[pad_start_rel:], pad_start_rel):
                    if char == '(':
                        depth += 1
                    elif char == ')':
                        depth -= 1
                        if depth == 0:
                            pad_end_rel = i + 1
                            break

                pad_text = fp_text[pad_start_rel:pad_end_rel]

                # Find the (net id "name") or (net "name") in this pad
                net_match = re.search(r'\(net\s+\d+\s+"[^"]*"\)', pad_text)
                if not net_match:
                    # KiCad 10: (net "name") with no numeric ID
                    net_match = re.search(r'\(net\s+"[^"]*"\)', pad_text)
                if net_match:
                    # Convert to absolute positions in content
                    abs_start = fp_start + pad_start_rel + net_match.start()
                    abs_end = fp_start + pad_start_rel + net_match.end()
                    return (abs_start, abs_end, net_match.group())

        return None

    # Find both pads' net declarations
    result1 = find_pad_net_in_footprint(content, pad1.component_ref, pad1.pad_number)
    result2 = find_pad_net_in_footprint(content, pad2.component_ref, pad2.pad_number)

    if not result1 or not result2:
        print(f"  WARNING: Could not find pad(s) to swap nets")
        if not result1:
            print(f"    Missing: {pad1.component_ref} pad {pad1.pad_number}")
        if not result2:
            print(f"    Missing: {pad2.component_ref} pad {pad2.pad_number}")
        return content

    start1, end1, net1_text = result1
    start2, end2, net2_text = result2

    # Swap the net declarations (replace higher position first to preserve indices)
    if start1 < start2:
        content = content[:start2] + net1_text + content[end2:]
        content = content[:start1] + net2_text + content[end1:]
    else:
        content = content[:start1] + net2_text + content[end1:]
        content = content[:start2] + net1_text + content[end2:]

    return content
