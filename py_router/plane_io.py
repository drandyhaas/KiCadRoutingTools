"""
I/O utilities for copper plane generation.

Handles reading zone information from PCB files and writing plane output.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Iterable, List, Dict, Tuple, Optional

from kicad_parser import (PCBData, parse_kicad_pcb, _unescape_kicad_string,
                          layer_list_tokens)
from kicad_writer import (generate_via_sexpr, generate_segment_sexpr, via_net_name,
                          move_copper_text_to_silkscreen,
                          move_copper_graphics_to_silkscreen, add_teardrops_to_pads,
                          add_teardrops_to_vias,
    strip_zero_length_edge_cuts)
# E3: the one guarded squared-distance kernel (length_sq < 1e-10 degenerate guard).
from geometry_utils import point_to_segment_dist_sq as _pt_seg_dist_sq


@dataclass
class ZoneInfo:
    """Information about a copper zone/pour."""
    net_id: int
    net_name: str
    layer: str
    in_footprint: bool = False  # #478: owned by a footprint, not the board
    # Fill priority of the EXISTING pour. Needed to coexist with a foreign-net
    # pour deterministically: KiCad fills higher priority first and only lower
    # ones pull back, so a new plane sharing the layer takes one ABOVE the highest
    # incumbent (see shared_layer_zone_priority -- priorities are non-negative,
    # so it cannot go below).
    priority: int = 0
    # A KiCad zone can span SEVERAL layers -- `(layers "F.Cu" "B.Cu")` is ONE
    # physical pour, which the reader expands into one ZoneInfo per layer. `span`
    # is that pour's full layer set and `uuid` identifies it, so a consumer can
    # tell "this is only on my layer" (safe to replace) from "this also pours
    # other layers" (deleting it would destroy copper nobody asked to touch).
    uuid: str = ''
    span: Tuple[str, ...] = ()
    # The parser Zone this entry was read from (#1329): its outline and fill
    # settings, so a replacement can say what it changes
    # (zone_replacement_changes). None for an entry built by hand.
    zone: object = field(default=None, compare=False, repr=False)


def zone_infos(zones) -> List[ZoneInfo]:
    """One ZoneInfo per parser Zone, each carrying its pour's full layer
    SPAN. The reader emits one Zone per copper layer, and the entries of a
    multi-layer `(layers ...)` pour share a uuid, which is how the span is
    recovered. Both the file path (extract_zones) and route_planes' live-board
    path read zones through this, so the two fronts decide alike."""
    zones = list(zones or ())
    span_by_uuid: Dict[str, List[str]] = {}
    for z in zones:
        if getattr(z, 'uuid', ''):
            span_by_uuid.setdefault(z.uuid, []).append(z.layer)
    return [ZoneInfo(net_id=z.net_id, net_name=z.net_name, layer=z.layer,
                     in_footprint=getattr(z, 'in_footprint', False),
                     priority=int(getattr(z, 'priority', 0) or 0),
                     uuid=getattr(z, 'uuid', '') or '',
                     span=tuple(span_by_uuid.get(getattr(z, 'uuid', '') or '',
                                                 [z.layer])),
                     zone=z)
            for z in zones]


def extract_zones(pcb_file: str) -> List[ZoneInfo]:
    """Extract zone information from a KiCad PCB file.

    Args:
        pcb_file: Path to the .kicad_pcb file

    Returns:
        List of ZoneInfo objects for each zone found
    """
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # #369 A12: delegate to kicad_parser's block-based zone reader instead of
    # a fixed-token-order regex. The regex demanded a single (layer "...") in
    # exact position, so a multi-layer (layers "In1.Cu" "In2.Cu") pour was
    # INVISIBLE here -- check_existing_zones then answered "create" and
    # route_planes poured a full-layer zone over another net's existing pour
    # (a mass short after refill). The parser reader emits one Zone per copper
    # layer, handles v5/v6 single-line zones, unquoted layers, and unescaped
    # net names, and is the same model route_planes' other path already uses.
    from kicad_parser import extract_zones as _parser_extract_zones
    from kicad_parser import extract_nets, detect_kicad_version
    _, name_to_id = extract_nets(content, detect_kicad_version(content))
    from kicad_parser import pour_zones
    # POURS only: a teardrop (#1164) is routing output, so a GND teardrop on
    # F.Cu must not read as "GND already pours on F.Cu" (route_planes kept
    # the pour it was asked to create) or as an incumbent whose priority a
    # new plane must outrank.
    parsed = pour_zones(_parser_extract_zones(content, name_to_id))

    # Each PHYSICAL pour's full layer span: consumers need it to know that
    # deleting the pour would also remove copper on layers they were not
    # asked to touch.
    return zone_infos(parsed)


def check_existing_zones(zones: List[ZoneInfo], target_layer: str, target_net_name: str,
                          target_net_id: int, verbose: bool = False,
                          call_layers: Optional[Iterable[str]] = None
                          ) -> Tuple[bool, bool, Optional[ZoneInfo], List[ZoneInfo]]:
    """Check for existing zones on the target layer.

    A foreign-net pour on the layer is NOT fatal. KiCad allows zones of different
    nets to share a layer -- it fills them by priority and holds clearance
    between nets -- and this function used to abort the whole run over it, which
    left `route_planes` unable to add a plane to any board that already had one
    on that layer (and, worse, the CLI aborted while the GUI path deliberately
    did not: "in KiCad, zones with different nets may coexist on a layer").

    So the foreign pour is kept, ours is created alongside, and the caller gives
    ours a fill priority ABOVE the incumbent -- see `shared_layer_zone_priority`.
    That picks a definite winner for the overlap (the plane the user just asked
    for) instead of leaving KiCad to tie-break on UUIDs. Both nets still keep
    clearance from each other, so coexisting is not a short.

    A same-net pour is replaced when this call pours the net on EVERY layer the
    pour covers (#1299). `call_layers` is that set: every layer the call pours
    `target_net_name` on. A `(layers "F.Cu" "B.Cu")` pour is ONE zone, so a
    call that pours the net on both replaces it, and one that pours only B.Cu
    keeps it. Deciding one layer at a time, as this did before, kept it in
    both cases and stacked a second zone on each layer even when the run
    poured them all. The writer (`filter_zones_from_content`) applies the same
    rule: it drops a pour only when all of its layers are in the remove set.

    Args:
        zones: List of existing zones
        target_layer: Layer we want to create zone on
        target_net_name: Net name we want
        target_net_id: Net ID we want
        verbose: Print verbose output
        call_layers: every layer this call pours `target_net_name` on
            (`target_layer` is always counted); None = `target_layer` alone

    Returns:
        (should_create_zone, should_continue, zone_to_replace, coexisting_zones)
        - should_create_zone: True to create/replace zone
        - should_continue: kept for callers; now always True (no abort case left)
        - zone_to_replace: ZoneInfo of the SAME-NET zone to replace, or None --
          None also when the same-net pour covers a layer this call does not
          pour the net on, since deleting it would remove copper there
        - coexisting_zones: pours already on this layer that stay, and that the
          new plane must therefore out-prioritize: other nets' pours, plus a
          same-net multi-layer pour that cannot be replaced
    """
    poured = set(call_layers or ()) | {target_layer}
    replaced = replaced_pours(zones, target_layer, target_net_name,
                              target_net_id, call_layers)
    zone_to_replace = replaced[0] if replaced else None
    coexist: List[ZoneInfo] = []
    keep_span: List[ZoneInfo] = []

    for zone in zones:
        # #478: footprint-owned pours are the footprint's business -- a small
        # shield/thermal pour must neither veto a whole plane layer nor be
        # "replaced" (deleted out of its footprint) by plane creation. KiCad
        # fills both; zone priority + clearance arbitrate the overlap.
        if getattr(zone, 'in_footprint', False):
            continue
        if zone.layer != target_layer:
            continue
        if zone.net_id == target_net_id or zone.net_name == target_net_name:
            # Same net, and not replaceable: it covers a layer this call does
            # not pour the net on, so deleting it would take that copper too.
            if not any(zone is r for r in replaced):
                keep_span.append(zone)
                coexist.append(zone)
        else:
            coexist.append(zone)

    if zone_to_replace is not None:
        others = sorted(set(zone_to_replace.span or ()) - {target_layer})
        whole = (f" (ONE pour that also covers {', '.join(others)}, which this "
                 f"run pours too, so it goes as a whole)" if others else "")
        print(f"Note: Replacing existing zone on {target_layer} for net "
              f"'{zone_to_replace.net_name}'{whole}; what the replacement changes "
              f"is listed once the new pour is built (--skip-existing-zones "
              f"keeps the existing pour)")

    for zone in keep_span:
        others = ", ".join(sorted(set(zone.span) - poured))
        print(f"WARNING: net '{zone.net_name}' already has a pour on {target_layer}, "
              f"but it is ONE multi-layer zone that also covers {others}, which "
              f"this run does not pour for '{zone.net_name}', so it cannot be "
              f"replaced without destroying that copper. Keeping it and pouring "
              f"the new plane over it at a higher fill priority (same net, so no "
              f"short). To replace it outright, pour '{zone.net_name}' on "
              f"{others} in the same run, or delete the pour in KiCad.")

    foreign = [z for z in coexist if z not in keep_span]
    if foreign:
        names = ", ".join(sorted({f"'{z.net_name}'" for z in foreign}))
        print(f"WARNING: {target_layer} already carries {len(foreign)} pour(s) for "
              f"other net(s) {names}; creating the '{target_net_name}' plane "
              f"alongside them (KiCad allows it and holds clearance between nets). "
              f"The new plane takes the HIGHER fill priority, so where they "
              f"overlap the existing pour(s) pull back around it. Point "
              f"--plane-layers at a free layer if you want the opposite.")

    return (True, True, zone_to_replace, coexist)


def replaced_pours(zones: List[ZoneInfo], target_layer: str, target_net_name: str,
                   target_net_id: int, call_layers: Optional[Iterable[str]] = None
                   ) -> List[ZoneInfo]:
    """Every existing pour that replacing `target_net_name`'s pour on
    `target_layer` removes: the board's (not a footprint's) same-net pours on
    the layer whose every layer this call pours the net on (see
    check_existing_zones, which decides with this). The writer removes all of
    them, so the disclosure must describe all of them (#1329)."""
    poured = set(call_layers or ()) | {target_layer}
    return [z for z in zones
            if not getattr(z, 'in_footprint', False)
            and z.layer == target_layer
            and (z.net_id == target_net_id or z.net_name == target_net_name)
            and set(getattr(z, 'span', ()) or (z.layer,)) <= poured]


#: How a person reads `Zone.pad_connection` (#1329).
_PAD_CONNECTION_WORDS = {'thermal': 'thermal relief', 'solid': 'solid',
                         'none': 'not connected',
                         'thru_hole_only': 'thermal relief on through-hole pads only'}
#: ... and `Zone.island_removal_mode`.
_ISLAND_WORDS = {0: 'always removed', 1: 'never removed',
                 2: 'removed below a minimum area'}


def _outline_words(points, holes=()) -> str:
    if not points:
        return 'unknown'
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    cut = f", {len(holes)} cut-out(s)" if holes else ""
    return (f"{len(points)}-vertex outline ({min(xs):.2f}, {min(ys):.2f})"
            f"..({max(xs):.2f}, {max(ys):.2f}){cut}")


def zone_replacement_changes(old_zone, new_zones: List[Dict],
                             outline_why: str = '') -> List[Dict]:
    """What replacing the existing pour `old_zone` (a parser Zone) with the
    pour(s) `new_zones` (create_plane's zone dicts for the same net and layer)
    changes, setting by setting, each with WHY the new value is what it is
    (#1329). Replacing used to say only "with new parameters", so a designer's
    thermal-relief pour became a solid one with no word about it.

    Returns [{setting, old, new, why}], one per setting that changes; [] when
    nothing does. A setting the old pour does not record reads 'unknown'.
    """
    from kicad_writer import ZONE_THERMAL_GAP, ZONE_THERMAL_BRIDGE_WIDTH
    if old_zone is None or not new_zones:
        return []
    new = new_zones[0]
    rows: List[Dict] = []

    def num(v):
        return 'unknown' if v is None else f"{v:g} mm"

    def differ(a, b):
        return a is None or b is None or abs(a - b) > 1e-6

    old_pc = getattr(old_zone, 'pad_connection', None)
    new_pc = 'thermal' if new.get('thermal_relief') else 'solid'
    if old_pc != new_pc:
        rows.append(dict(
            setting='pad connection',
            old=_PAD_CONNECTION_WORDS.get(old_pc, old_pc or 'unknown'),
            new=_PAD_CONNECTION_WORDS[new_pc],
            why=('route_planes connects pads solid unless --thermal-relief '
                 '(Planes tab: Thermal relief pad connections)'
                 if new_pc == 'solid' else '--thermal-relief')))
    old_cl, new_cl = getattr(old_zone, 'clearance', None), new.get('clearance')
    if new_cl is not None and differ(old_cl, new_cl):
        rows.append(dict(setting='clearance', old=num(old_cl), new=num(new_cl),
                         why="this run's zone clearance: --zone-clearance, "
                             "else the routed clearance"))
    old_mt, new_mt = getattr(old_zone, 'min_thickness', None), new.get('min_thickness')
    if new_mt is not None and differ(old_mt, new_mt):
        rows.append(dict(setting='minimum width', old=num(old_mt), new=num(new_mt),
                         why='--min-thickness'))
    # Listed for a solid pour too: the file keeps them, and they are what a
    # later switch to thermal relief gets.
    old_sp = (getattr(old_zone, 'thermal_gap', None),
              getattr(old_zone, 'thermal_bridge_width', None))
    new_sp = (ZONE_THERMAL_GAP, ZONE_THERMAL_BRIDGE_WIDTH)
    if any(differ(a, b) for a, b in zip(old_sp, new_sp)):
        rows.append(dict(setting='thermal gap / spoke width',
                         old=f"{num(old_sp[0])} / {num(old_sp[1])}",
                         new=f"{num(new_sp[0])} / {num(new_sp[1])}",
                         why="route_planes' fixed spoke geometry"
                             + ("; unused while pads connect solid"
                                if new_pc == 'solid' else '')))
    old_poly = list(getattr(old_zone, 'polygon', None) or ())
    old_holes = list(getattr(old_zone, 'holes', None) or ())
    new_polys = [list(z.get('polygon_points') or ()) for z in new_zones]
    same_outline = (len(new_polys) == 1 and not old_holes
                    and len(old_poly) == len(new_polys[0])
                    and all(abs(a[0] - b[0]) < 1e-4 and abs(a[1] - b[1]) < 1e-4
                            for a, b in zip(old_poly, new_polys[0])))
    if not same_outline:
        new_words = (_outline_words(new_polys[0]) if len(new_polys) == 1 else
                     f"{len(new_polys)} regions")
        rows.append(dict(setting='outline', old=_outline_words(old_poly, old_holes),
                         new=new_words, why=outline_why or 'the new pour'))
    old_isl = getattr(old_zone, 'island_removal_mode', 0) or 0
    if old_isl != 0:
        rows.append(dict(setting='isolated islands',
                         old=_ISLAND_WORDS.get(old_isl, str(old_isl)),
                         new=_ISLAND_WORDS[0],
                         why="route_planes' pours always remove them"))
    old_pr = int(getattr(old_zone, 'priority', 0) or 0)
    new_pr = int(new.get('priority', 0) or 0)
    if old_pr != new_pr:
        rows.append(dict(setting='fill priority', old=str(old_pr), new=str(new_pr),
                         why='one above another net\'s pour on the layer'
                             if new_pr > old_pr else 'the run\'s priority'))
    return rows


def format_zone_replacement(net_name: str, layers: List[str],
                            rows: List[Dict]) -> List[str]:
    """Console lines for one replaced pour (#1329): the settings it changes,
    each with why, and how to keep the pour instead."""
    head = (f"Replaced '{net_name}' pour on {', '.join(layers)} "
            f"(--skip-existing-zones keeps it):")
    if not rows:
        return [head + " its settings are unchanged"]
    out = [head + f" {len(rows)} setting(s) changed"]
    for r in rows:
        out.append(f"    {r['setting']}: {r['old']} -> {r['new']}  [{r['why']}]")
    return out


def shared_layer_zone_priority(foreign_zones: List[ZoneInfo]) -> int:
    """Fill-priority offset for a new plane sharing a layer with other nets' pours.

    One ABOVE the highest incumbent, so KiCad -- which fills highest priority
    first and only pulls LOWER-priority zones back -- resolves every overlap in
    favour of the plane just requested, deterministically. Without it both sit at
    priority 0 and the fill hinges on UUID order, the exact nondeterminism
    `zone_overlap_priorities` exists to remove.

    Upward, not downward: KiCad zone priority is NON-NEGATIVE. A "one below the
    incumbent" value of -1 is written but then read back as 0 (the parsers match
    `\\d+`), so it silently did nothing -- measured on megadesk before this was
    turned around. Going up also leaves the user's own zone untouched.

    Returns 0 when nothing contends, so boards that never had the ambiguity are
    byte-for-byte unchanged.
    """
    if not foreign_zones:
        return 0
    return max(int(getattr(z, 'priority', 0) or 0) for z in foreign_zones) + 1


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

    # Character-based, quote-aware block scan (#523). The old LINE-based
    # collector counted parens per line, so a board where one element's
    # closer and the next element's opener share a line -- the ')\t(segment'
    # joins the KiCad-oracle's ''.join splice emitted (legal s-expr, KiCad
    # reads it fine) -- never terminated its count: it swallowed every
    # following element to EOF as ONE "element" and removed the lot when the
    # FIRST net token matched, eating foreign-net copper and the file's root
    # paren (the apple2e_mmu truncation: a board KiCad could not load while
    # check_drc/check_connected graded it normally). A char walk with
    # in-string tracking is immune to line layout, and parens inside quoted
    # net names like (net "Net-(R2-Pad1)") never count.
    out = []
    pos = 0
    n = len(content)
    depth = 0
    in_str = False
    esc = False
    i = 0
    while i < n:
        ch = content[i]
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
            i += 1
            continue
        if ch == '"':
            in_str = True
            i += 1
            continue
        if ch == '(':
            if depth == 1:
                tok = None
                if content.startswith('(segment', i) and \
                        i + 8 < n and content[i + 8] in ' \t\r\n(':
                    tok = 'segment'
                elif content.startswith('(via', i) and \
                        i + 4 < n and content[i + 4] in ' \t\r\n(':
                    tok = 'via'
                if tok is not None:
                    # Quote-aware walk to this block's own closer.
                    d2 = 0
                    s2 = False
                    e2 = False
                    k = i
                    while k < n:
                        c2 = content[k]
                        if s2:
                            if e2:
                                e2 = False
                            elif c2 == '\\':
                                e2 = True
                            elif c2 == '"':
                                s2 = False
                        elif c2 == '"':
                            s2 = True
                        elif c2 == '(':
                            d2 += 1
                        elif c2 == ')':
                            d2 -= 1
                            if d2 == 0:
                                k += 1
                                break
                        k += 1
                    block = content[i:k]
                    remove = False
                    # Try KiCad 9 format: (net <id>)
                    net_match = re.search(r'\(net\s+(\d+)\)', block)
                    if net_match:
                        remove = int(net_match.group(1)) in net_id_set
                    elif net_name_set:
                        # KiCad 10: (net "name"). The file stores the ESCAPED
                        # name; the exclude set holds the parser's unescaped
                        # names -- without unescaping, every backslash-named
                        # ripped net evaded this filter and its stale copper
                        # shipped OVERLAPPING the tap via route_planes placed
                        # in the vacated spot (neo6502 GND-via-on-/GPIO9\OE2#,
                        # found by the #319 DRC attribution; #312/#264
                        # escaping family).
                        m10 = re.search(
                            r'\(net\s+"((?:[^"\\]|\\.)*)"\)', block)
                        remove = bool(
                            m10 and _unescape_kicad_string(m10.group(1))
                            in net_name_set)
                    if remove:
                        # Trim the removed block's own leading indent from
                        # the kept prefix, and swallow the trailing newline
                        # only when the rest of its line is blank -- so a
                        # removed block leaves neither a blank line nor a
                        # stray tab, and joined-line inputs stay balanced.
                        pre = i
                        while pre > pos and content[pre - 1] in '\t ':
                            pre -= 1
                        out.append(content[pos:pre])
                        k2 = k
                        while k2 < n and content[k2] in '\t ':
                            k2 += 1
                        if k2 < n and content[k2] == '\n':
                            k = k2 + 1
                        pos = k
                    i = k
                    continue
            depth += 1
        elif ch == ')':
            depth -= 1
        i += 1
    out.append(content[pos:])
    return ''.join(out)


def _zone_layer_span(element_text: str, copper_layers=None) -> set:
    """Every copper layer a zone block declares -- the span the parser reads
    (kicad_parser.extract_zones), so the writer removes exactly the pours the
    engine decided to replace.

    Handles both spellings: `(layer "B.Cu")` and the multi-layer
    `(layers "F.Cu" "B.Cu")`, which is ONE physical pour -- with the names
    quoted or bare (KiCad 5 / 6), and KiCad 6's layer-set tokens expanded as
    pcbnew loads them: `F&B.Cu` is F.Cu + B.Cu, `*.Cu` every copper layer of
    the board (`copper_layers`), `*In.Cu` its inner ones (#1299: kept
    literally, they could never be in a remove set, so a pour the engine
    replaced stayed in the file under its replacement). Read from the zone
    HEADER only: each fill polygon carries a `(layer ...)` tag of its own.
    """
    hdr = element_text
    for tok in ('(polygon', '(filled_polygon', '(keepout'):
        i = hdr.find(tok)
        if i != -1:
            hdr = hdr[:i]
    m = re.search(r'\(layers\s+([^()]*)\)', hdr)
    if m:
        tokens = layer_list_tokens(m.group(1))
    else:
        m = re.search(r'\(layer\s+"?([^")\s]+)"?\)', hdr)
        tokens = [m.group(1)] if m else []
    span = set()
    for t in tokens:
        if t == 'F&B.Cu':
            span |= {'F.Cu', 'B.Cu'}
        elif t == '*.Cu':
            span |= set(copper_layers or ())
        elif t == '*In.Cu':
            span |= {l for l in (copper_layers or ()) if l not in ('F.Cu', 'B.Cu')}
        elif t.endswith('.Cu'):
            span.add(t)
    return span


def filter_zones_from_content(content: str, zones_to_remove: List[Tuple[int, str]],
                              zone_names_to_remove: List[Tuple[str, str]] = None) -> str:
    """
    Filter out zones for specific (net_id, layer) pairs from PCB file content.

    A zone is removed only when EVERY layer it pours is in the remove set, so a
    multi-layer pour is never deleted to "replace" one of its layers.

    Args:
        content: Raw PCB file content
        zones_to_remove: List of (net_id, layer) tuples to remove
        zone_names_to_remove: For KiCad 10, list of (net_name, layer) tuples to
            remove -- REQUIRED for a name-based board, where zones carry
            (net "NAME") and the numeric matcher can never fire

    Returns:
        Filtered content with those zones removed
    """
    if not zones_to_remove and not zone_names_to_remove:
        return content

    # Build sets for fast lookup
    remove_set = set(zones_to_remove) if zones_to_remove else set()
    remove_name_set = set(zone_names_to_remove) if zone_names_to_remove else set()
    copper_layers = None   # the board's copper stack, read once if a zone says *.Cu
    lines = content.split('\n')
    result_lines = []

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Check if this starts a zone (may be multi-line)
        if stripped == '(zone' or stripped.startswith('(zone '):
            # #478: never remove a footprint-owned zone (nested >= 2 tabs) --
            # replacing a board plane must not excise a same-net pour out of a
            # footprint's own block.
            _fp_nested = line[:len(line) - len(stripped)].count('\t') >= 2
            # Collect all lines of this zone element
            element_lines = [line]
            open_parens = line.count('(') - line.count(')')
            while open_parens > 0 and i + 1 < len(lines):
                i += 1
                element_lines.append(lines[i])
                open_parens += lines[i].count('(') - lines[i].count(')')

            # Extract net and layer SPAN from the zone. A pour may declare one
            # layer -- (layer "B.Cu") -- or several -- (layers "F.Cu" "B.Cu"),
            # which is ONE physical zone. Matching only the singular spelling
            # meant a multi-layer pour was never removable here at all (megadesk's
            # GND pour), the same blind spot #369 A12 fixed on the READ side.
            element_text = '\n'.join(element_lines)
            if copper_layers is None and re.search(r'\*(?:In)?\.Cu', element_text):
                from kicad_parser import _layer_table
                copper_layers = _layer_table(content)[1]
            zone_layers = _zone_layer_span(element_text, copper_layers)
            # Try KiCad 9: (net <id>)
            net_match = re.search(r'\(net\s+(\d+)\)', element_text)

            # #1164: a teardrop area is routing output on the track's net, not
            # a pour, so replacing that net's pour on this layer must not take
            # it (the parser's own test, kicad_parser.extract_zones).
            _teardrop = re.search(r'\(attr\s*\(teardrop[\s)]',
                                  element_text) is not None

            def _removable(keys, want) -> bool:
                # Remove only when EVERY layer this pour covers is being replaced.
                # Otherwise deleting it would take copper on a layer nobody asked
                # to touch (a GND pour on F.Cu+B.Cu replaced "for B.Cu").
                return (bool(keys) and keys <= want and not _fp_nested
                        and not _teardrop)

            if net_match and zone_layers:
                zone_net_id = int(net_match.group(1))
                if _removable({(zone_net_id, l) for l in zone_layers}, remove_set):
                    # Skip this zone entirely
                    i += 1
                    continue
            elif zone_layers and remove_name_set:
                # KiCad 10: (net "name")
                net_match_v10 = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"\)', element_text)
                if net_match_v10:
                    # Unescape: remove_name_set holds parser display names.
                    zone_net_name = _unescape_kicad_string(net_match_v10.group(1))
                    if _removable({(zone_net_name, l) for l in zone_layers},
                                  remove_name_set):
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
    net_id_to_name: Dict[int, str] = None,
    removed_segments: List = None
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
        removed_segments: SPECIFIC input-board Segment objects to delete (#508
                          finding 2 / #484 class: an in-memory pass removed a
                          non-excluded net's input copper from pcb_data; the
                          writer must strip its text or board != file)

    Returns:
        True if successful, False otherwise
    """
    # Seed the output's sibling .kicad_pro before the board exists (#513 item
    # 12): a kill between this write and the post-write floor writeback must
    # not strand the DRC floor (stock-netclass fallback, the #441/#338 class).
    from fix_kicad_drc_settings import seed_project_for_output
    seed_project_for_output(output_file, input_file)
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    if removed_segments:
        from kicad_writer import remove_segments_from_content
        content, _nrs = remove_segments_from_content(
            content, removed_segments, net_id_to_name=net_id_to_name)
        if _nrs:
            print(f"  Stripped {_nrs} in-memory-removed input segment(s) "
                  f"from the output")

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)
    content = move_copper_graphics_to_silkscreen(content)
    content = strip_zero_length_edge_cuts(content)

    # Add teardrops to all pads if requested. Pads are never ADDED by a routing
    # run, so this can run on the input text; the VIA pass has to wait until this
    # run's own stitching vias are in the content (see below, #489 §9).
    if add_teardrops:
        print("Adding teardrop settings to pads...")
        content, teardrop_count = add_teardrops_to_pads(content)
        if teardrop_count > 0:
            print(f"  Added teardrops to {teardrop_count} pads")
        else:
            print("  All pads already have teardrop settings")

    # Filter out zones to be replaced. SAME #88.1 class as the net filter just
    # below, which was fixed long before: on a KiCad 10 board a zone carries
    # (net "NAME"), so the numeric (net <id>) matcher here never matched and the
    # "replaced" zone silently SHIPPED alongside its replacement -- two same-net
    # pours on one layer, while the log said "Replacing existing zone".
    # Reproduced on megadesk (version 20260206): a single-net GND run on B.Cu left
    # the original 4-point GND pour sitting next to the new one.
    if zones_to_replace:
        zone_names = None
        if net_id_to_name:
            zone_names = [(net_id_to_name[nid], layer)
                          for nid, layer in zones_to_replace
                          if nid in net_id_to_name]
        content = filter_zones_from_content(content, zones_to_replace,
                                            zone_names_to_remove=zone_names)

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

    # Add vias. A reused via carries its own protection spec; a NEW stitching
    # via emits no token and inherits the board's `(setup ...)` policy, exactly
    # as a via placed in KiCad does (see add_tracks_and_vias_to_pcb).
    for via in new_vias:
        elements.append(generate_via_sexpr(
            via['x'], via['y'], via['size'], via['drill'],
            via['layers'], via['net_id'],
            # #749 D: the ONE resolver -- net 0 is absent from every map, and a
            # numeric ref emitted next to a spec used to be a via the parser
            # could not read back (#748).
            net_name=via_net_name(via['net_id'], net_id_to_name),
            tenting_attrs=via.get('tenting_attrs'),
            via_type=via.get('via_type')
        ))

    # Add segments
    for seg in new_segments:
        seg_net_name = net_id_to_name.get(seg['net_id']) if net_id_to_name else None
        elements.append(generate_segment_sexpr(
            seg['start'], seg['end'],
            seg['width'], seg['layer'], seg['net_id'], net_name=seg_net_name
        ))

    if not elements:
        # Nothing to add, just copy the file -- but the via teardrop pass still
        # has to run over the board's existing vias.
        if add_teardrops:
            content, _vtd = add_teardrops_to_vias(content)
            print(f"  Added teardrops to {_vtd} vias" if _vtd
                  else "  No vias needed teardrops (none present, or all already set)")
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

    # Via teardrops LAST, so this run's own stitching vias get them too -- those
    # are the vias that most need one (#489 §9). Running before the insert above
    # covered only the vias already on the board (measured: 63 of 81 on megadesk).
    if add_teardrops:
        new_content, via_teardrop_count = add_teardrops_to_vias(new_content)
        if via_teardrop_count > 0:
            print(f"  Added teardrops to {via_teardrop_count} vias")
        else:
            print("  No vias needed teardrops (none present, or all already set)")

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def _remove_vias_at_positions(content: str, positions: List[Tuple[float, float]],
                              tol: float = 2e-3,
                              net_ids: Optional[List[int]] = None,
                              net_names: Optional[List[Optional[str]]] = None
                              ) -> Tuple[str, int]:
    """Remove `(via ...)` elements whose `(at x y)` matches any of `positions`.

    Used to drop plane stitching vias that would short against restored signal
    copper (issue #88). Matching is positional with a small tolerance because
    vias are written with fixed 6-decimal precision.

    net_ids / net_names (parallel to positions, #313/#344): when given, a via
    is dropped only if its net ALSO matches -- so a via-nudge rewrite that
    re-emits ONE net's moved via can't collaterally delete a DIFFERENT net's
    via that happens to sit within `tol` of the old position (net-agnostic
    positional removal was a silent +open-net path). KiCad 9 vias carry a
    numeric `(net N)`, KiCad 10 vias carry the net NAME `(net "/FOO")` -- the
    id-only guard read every name-based via as net None, never matched, and
    turned removal into a no-op: the moved via was re-emitted WITHOUT removing
    the original, shipping a stacked duplicate and the very graze the nudge
    was clearing (#344 C6.1/GPIO26, same file-format class as #264). The net
    guard therefore only skips on a POSITIVE mismatch of whichever form the
    file carries; a via whose net cannot be compared falls back to positional
    matching (pre-#313 behavior).
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
            nm = re.search(r'\(net\s+(\d+)', element_text)
            v_net = int(nm.group(1)) if nm else None
            v_net_name = None
            if v_net is None:
                nn = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"', element_text)
                if nn:
                    from kicad_parser import _unescape_kicad_string
                    v_net_name = _unescape_kicad_string(nn.group(1))
            drop = False
            if m:
                vx, vy = float(m.group(1)), float(m.group(2))
                for k, (px, py) in enumerate(positions):
                    if abs(vx - px) < tol and abs(vy - py) < tol:
                        if (net_ids is not None and v_net is not None
                                and v_net != net_ids[k]):
                            continue  # right spot, wrong net: leave it
                        if (net_names is not None and v_net_name is not None
                                and net_names[k] is not None
                                and v_net_name != net_names[k]):
                            continue  # name-based file, wrong net name
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
    partial_copper: Optional[Tuple[List[Dict], List[Dict]]] = None,
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
                                  'layers': v.layers, 'net_id': nid,
                                  # These vias came OFF the board and go back on;
                                  # carry their tenting/plugging spec with them
                                  # or the restore silently re-tents them (#489 §8).
                                  'tenting_attrs': dict(getattr(v, 'tenting_attrs', {}) or {}),
                                  # ...and its blind/micro type word.
                                  'via_type': getattr(v, 'via_type', 'through')})

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

    # A PARTIALLY-restored net (b2557cd settle) already has its kept pieces in
    # the output as emitted copper; restoring the full original on top would
    # duplicate every piece at identical coordinates and resurrect the dropped
    # colliding ones. Strip the net's emitted partial copper first -- the full
    # original restore replaces it.
    if partial_copper is not None:
        p_segs, p_vias = partial_copper
        p_segs = [x for x in p_segs if x.get('net_id') in restored_net_ids]
        p_vias = [x for x in p_vias if x.get('net_id') in restored_net_ids]
        if p_vias:
            content, _n = _remove_vias_at_positions(
                content, [(v['x'], v['y']) for v in p_vias])
        if p_segs:
            content, _n = _remove_segments_at(content, p_segs)

    content, vias_removed = _remove_vias_at_positions(content, remove_positions)
    content, segs_removed = _remove_segments_at(content, remove_segs)

    elements: List[str] = []
    for v in restored_vias:
        # #749 C: this is a RESTORE -- these vias came off the board -- so they
        # get their real spec back, and one that carried none keeps inheriting
        # the board's `(setup ...)` rather than gaining a token. Both halves
        # matter: handing {} back used to re-stamp the via with front+back
        # tenting it never had.
        elements.append(generate_via_sexpr(v['x'], v['y'], v['size'], v['drill'],
                                           v['layers'], v['net_id'],
                                           net_name=via_net_name(v['net_id'],
                                                                 net_id_to_name),
                                           tenting_attrs=v.get('tenting_attrs'),
                                           inherit_when_unspecified=True,
                                           via_type=v.get('via_type')))
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
