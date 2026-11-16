"""
KiCad Routing Tools - make the live board match a result board file (IPC).

A tab that runs a CLI chain on a snapshot of the board (the Autorouter tab,
the AI Placement tab's runs) ends with a result .kicad_pcb, and "Apply
Result to Board" makes the open board match it. The result board is the
authority: what the result has, the live board gets; what the live board has
and the result does not, goes. Item by item:

  footprint poses   moved, flipped and turned to the result's pose, and its
                    KiCad lock mirrored (#1054); a footprint that draws the
                    board outline is never moved (#829) and is named
  tracks and vias   a live item the result carries UNCHANGED is kept (its
                    uuid, and with it its KiCad group, #1279); every other
                    live item is removed and the result's are created
  zones             the board-level copper pours: a live pour the result
                    carries unchanged is kept, the rest are replaced by pours
                    built from the result's (outline, holes, layers, net,
                    priority, clearance, thickness, pad connection, islands)
  design rules      the result project's rule floors and net classes,
                    lowered into the live board's project file (only-lower,
                    as every IPC writeback), and the result's
                    `kicad_routing_tools` records

The ipc-migration twin of main's module of the same name, which writes a
pcbnew BOARD. The contract is the same (`plan_apply` lists, `run_apply`
carries out, one function per kind of item); what kipy cannot do is said
where it applies:

  - every change rides ONE commit, so the whole Apply is one undo step;
  - created tracks and vias take new uuids and go on in the parse's order
    (tracks, then vias), not the result's own uuids and file order: kipy
    has no uuid setter on create and no insertion order;
  - a zone is rebuilt from the result's parse rather than copied with its
    fill (kipy cannot load a second board), and KiCad refills the zones
    after the commit; rule areas and teardrops stay as the live board has
    them;
  - the rules go into the project FILE, as every IPC writeback: kipy cannot
    write the live design settings, so KiCad uses them after the project is
    reloaded.

Nothing here builds UI. The caller lists the plan (`plan_apply`) in its
confirmation box, then calls `run_apply`.
"""

import json
import os


# ------------------------------------------------------------- footprints

def fp_is_back(fp) -> bool:
    """True when the live footprint sits on the back side. kipy models the
    side as the footprint's `layer` (BL_F_Cu / BL_B_Cu), the IPC analogue of
    SWIG's IsFlipped()."""
    from kicad_ipc_adapter import _ensure_kipy
    try:
        return fp.layer == _ensure_kipy().BoardLayer.BL_B_Cu
    except Exception:                                           # noqa: BLE001
        return False


def live_footprints_by_key(board):
    """{PCBData key: live kipy footprint}, in one enumeration (#726).

    Keyed the way PCBData keys are: two blocks sharing a reference are `TP4`
    and `TP4~2`, so a map keyed on the bare reference finds neither.
    `disambiguate_references` over the board's own footprint order is the
    same function both parse paths use. One enumeration, not a lookup per
    part: kipy has no FindFootprintByReference."""
    from kicad_parser import _fp_reference, disambiguate_references
    live_fps = list(board.get_footprints())
    raw = []
    for fp in live_fps:
        r = _fp_reference(fp)
        if not r:
            try:
                r = "#" + str(fp.id.value)
            except Exception:                                   # noqa: BLE001
                r = "?"
        raw.append(r)
    return dict(zip(disambiguate_references(raw), live_fps))


def pose_moves(board, result_pcb, live=None):
    """([(ref, live_fp, parsed_fp)] whose pose differs, [outline owners not
    moved], [refs only in the result]).

    #829: a footprint that draws the board's own outline is skipped: moving it
    would resize the board. This applier writes poses through the IPC commit
    and never goes through the CLI writer, so the CLI-side guard does not
    reach it; and the move list is a diff of the RESULT against the LIVE
    board, so a live board that drifted from the staged input produces moves
    the CLI never proposed. Resolved by the PCBData key (#726)."""
    from kicad_ipc_adapter import _vec_xy_mm
    if live is None:
        live = live_footprints_by_key(board)
    moves, outline_skipped, unreachable = [], [], []
    for ref, parsed in sorted(result_pcb.footprints.items()):
        if getattr(parsed, 'owns_board_outline', False):
            outline_skipped.append(ref)
            continue
        fp = live.get(ref)
        if fp is None:
            unreachable.append(ref)    # never invent parts
            continue
        live_x, live_y = _vec_xy_mm(fp.position)
        live_rot = float(getattr(fp.orientation, "degrees", 0.0)) % 360
        parsed_back = str(parsed.layer or "F.Cu").startswith("B")
        if (abs(live_x - parsed.x) > 1e-4 or abs(live_y - parsed.y) > 1e-4
                or abs((live_rot - (parsed.rotation or 0) % 360 + 180) % 360 - 180) > 1e-3
                or fp_is_back(fp) != parsed_back):
            moves.append((ref, fp, parsed))
    return moves, outline_skipped, unreachable


def lock_changes(board, result_pcb, live=None):
    """[(ref, live_fp, want_locked)] for every footprint whose KiCad lock
    differs from the result board's (#1054), in both directions. An outline
    owner is included: a lock moves nothing."""
    from kicad_parser import kipy_locked
    if live is None:
        live = live_footprints_by_key(board)
    changes = []
    for ref, parsed in sorted(result_pcb.footprints.items()):
        fp = live.get(ref)
        if fp is None:
            continue
        want = bool(getattr(parsed, 'locked', False))
        if kipy_locked(fp) != want:
            changes.append((ref, fp, want))
    return changes


def apply_pose(fp, parsed):
    """Give the kipy footprint FP the parsed pose (side, position, angle). The
    caller queues `commit.update(fp)`. kipy has no Flip(): the side IS the
    footprint's layer."""
    from kicad_ipc_adapter import _ensure_kipy, vec_mm
    parsed_back = str(parsed.layer or "F.Cu").startswith("B")
    if fp_is_back(fp) != parsed_back:
        BoardLayer = _ensure_kipy().BoardLayer
        fp.layer = BoardLayer.BL_B_Cu if parsed_back else BoardLayer.BL_F_Cu
    fp.position = vec_mm(parsed.x, parsed.y)
    try:
        from kipy.geometry import Angle
        fp.orientation = Angle.from_degrees(float(parsed.rotation or 0))
    except Exception:                                           # noqa: BLE001
        pass  # leave orientation if the API shape differs


# ------------------------------------------------------------ tracks/vias

def copper_plan(board, result_pcb):
    """What making the live board's tracks and vias the result's copper takes.

    A live track, arc or via the result carries UNCHANGED -- same geometry,
    layer, width and net, and for a via the same size, drill, layer pair and
    type -- is KEPT rather than removed and recreated (#1279): recreating gave
    every item a new uuid, which took it out of any KiCad group (a `sense:`
    group lost its members) and brought an unchanged arc back as chords.
    Changes nothing. Returns {'kept', 'removed', 'arcs_removed',
    'tracks_added', 'vias_added'} counts plus the items behind them
    ('_kept', '_removed', '_add_segs', '_add_vias')."""
    from kicad_ipc_adapter import layer_name_for, _vec_xy_mm
    from kicad_parser import mm_to_iu, _arc_to_segments, kipy_via_type

    def name_for(net_id):
        net = result_pcb.nets.get(net_id)
        return net.name if net is not None else ""

    def live_net_name(item):
        net = getattr(item, "net", None)
        return (getattr(net, "name", "") or "") if net is not None else ""

    # Keys compare in integer nanometres, the resolution both boards hold.
    def seg_key(a, b, width_nm, layer, net_name):
        return (min(a, b), max(a, b), width_nm, layer, net_name)

    def via_key(x, y, size, drill, layers, vtype, net_name):
        return (mm_to_iu(x), mm_to_iu(y), mm_to_iu(size), mm_to_iu(drill),
                tuple(layers or ()), vtype or 'through', net_name)

    # The result's copper, by key. Copper GRAPHICS parse into segments too
    # (#337 board gr_*, #908 footprint fp_*): they are drawings, not tracks.
    want_segs, want_vias = {}, {}
    for seg in result_pcb.segments:
        if getattr(seg, "graphic", False):
            continue
        k = seg_key((mm_to_iu(seg.start_x), mm_to_iu(seg.start_y)),
                    (mm_to_iu(seg.end_x), mm_to_iu(seg.end_y)),
                    mm_to_iu(seg.width), seg.layer, name_for(seg.net_id))
        want_segs.setdefault(k, []).append(seg)
    for via in result_pcb.vias:
        k = via_key(via.x, via.y, via.size, via.drill, via.layers,
                    getattr(via, 'via_type', 'through'), name_for(via.net_id))
        want_vias.setdefault(k, []).append(via)

    # The live copper, keyed the way the kipy builder reads it: an arc's
    # pieces chorded by the same _arc_to_segments the result's parse chords
    # it with.
    items = []
    for t in list(board.get_tracks()):
        net = live_net_name(t)
        width = int(getattr(t, "width", 0) or 0)
        layer = layer_name_for(getattr(t, "layer", None))
        sx, sy = _vec_xy_mm(t.start)
        ex, ey = _vec_xy_mm(t.end)
        mid = getattr(t, "mid", None)
        if mid is not None:
            mx, my = _vec_xy_mm(mid)
            keys = [seg_key((mm_to_iu(p0[0]), mm_to_iu(p0[1])),
                            (mm_to_iu(p1[0]), mm_to_iu(p1[1])), width, layer, net)
                    for p0, p1 in _arc_to_segments((sx, sy), (mx, my), (ex, ey))]
        else:
            keys = [seg_key((mm_to_iu(sx), mm_to_iu(sy)),
                            (mm_to_iu(ex), mm_to_iu(ey)), width, layer, net)]
        items.append((t, 'seg', keys, mid is not None))
    for v in list(board.get_vias()):
        vx, vy = _vec_xy_mm(v.position)
        drill = getattr(getattr(v, "padstack", None), "drill", None)
        top = layer_name_for(getattr(drill, "start_layer", None)) or "F.Cu"
        bot = layer_name_for(getattr(drill, "end_layer", None)) or "B.Cu"
        items.append((v, 'via', [via_key(
            vx, vy, (getattr(v, "diameter", 0) or 0) / 1e6,
            (getattr(v, "drill_diameter", 0) or 0) / 1e6,
            [top, bot], kipy_via_type(v), live_net_name(v))], False))

    kept, removed = [], []
    for it, kind, keys, is_arc in items:
        pool = want_segs if kind == 'seg' else want_vias
        need = {}
        for k in keys:
            need[k] = need.get(k, 0) + 1
        if keys and all(len(pool.get(k, ())) >= n for k, n in need.items()):
            kept.append((it, kind, [pool[k].pop() for k in keys]))
        else:
            removed.append((it, is_arc))
    add_segs = [s for segs in want_segs.values() for s in segs]
    add_vias = [v for vias in want_vias.values() for v in vias]
    return {'kept': len(kept), 'removed': len(removed),
            'arcs_removed': sum(1 for _it, is_arc in removed if is_arc),
            'tracks_added': len(add_segs), 'vias_added': len(add_vias),
            '_kept': kept, '_removed': [it for it, _a in removed],
            '_add_segs': add_segs, '_add_vias': add_vias}


def apply_copper(commit, result_pcb, plan):
    """Queue PLAN (`copper_plan`) on COMMIT: the removals, the result's
    protection spec on each kept via, and the created tracks and vias. A
    created item takes its KiCad lock from the result's `locked` (#521); a
    sense hold is `held_by`, never a lock. Returns the public counts."""
    from kicad_ipc_adapter import make_track, make_via, set_item_locked
    from kicad_parser import kipy_set_via_protection

    def name_for(net_id):
        net = result_pcb.nets.get(net_id)
        return net.name if net is not None else None

    for it in plan['_removed']:
        commit.remove(it)
    for it, kind, taken in plan['_kept']:
        spec = getattr(taken[0], "tenting_attrs", None)
        if kind == 'via' and spec:
            # Keep the result's tenting/plugging spec (#489 s8); an empty
            # spec leaves the via inheriting the board's, as it was.
            kipy_set_via_protection(it, spec)
            commit.update(it)
    for seg in plan['_add_segs']:
        t = make_track(commit.net_map, seg.start_x, seg.start_y,
                       seg.end_x, seg.end_y, seg.width, seg.layer,
                       net_name=name_for(seg.net_id))
        # #521: user-pinned copper must come back pinned.
        set_item_locked(t, bool(getattr(seg, "locked", False)))
        commit.add(t)
    for via in plan['_add_vias']:
        layers = getattr(via, "layers", None) or ["F.Cu", "B.Cu"]
        v = make_via(commit.net_map, via.x, via.y, via.size, via.drill,
                     top_layer=layers[0], bottom_layer=layers[-1],
                     net_name=name_for(via.net_id),
                     # #489 s8 / #962: the via's own protection spec.
                     tenting_attrs=getattr(via, "tenting_attrs", None),
                     via_type=getattr(via, "via_type", None))
        set_item_locked(v, bool(getattr(via, "locked", False)))  # #521
        commit.add(v)
    print(f"Apply: {plan['kept']} unchanged track(s)/via(s) kept, "
          f"{plan['removed']} removed, {plan['tracks_added']} track(s) and "
          f"{plan['vias_added']} via(s) added")
    return public_counts(plan)


def public_counts(plan):
    """A copper or zone plan without its items (what stats and lines show)."""
    return {k: v for k, v in (plan or {}).items() if not k.startswith('_')}


def disclose_via_protection(result_pcb, what="apply"):
    """The vias whose protection spec kipy could not write, named (#962)."""
    from kicad_ipc_adapter import disclose_unwritten_via_protection

    def name_for(net_id):
        net = result_pcb.nets.get(net_id)
        return net.name if net is not None else None
    disclose_unwritten_via_protection(result_pcb.vias, name_for, result_pcb, what)


# ------------------------------------------------------------------ zones

def _nm_ring(points):
    from kicad_parser import mm_to_iu
    return tuple((mm_to_iu(x), mm_to_iu(y)) for x, y in points or ())


def _zone_signature(entries):
    """What makes two pours the same pour: net, copper layers, outline and
    holes to the nanometre, priority and the fill settings, over the per-layer
    Zone entries of ONE zone (both readers emit one entry per copper layer).
    Fills are not compared."""
    z = entries[0]

    def mm(v):
        return None if v is None else round(float(v), 6)
    return (z.net_name or "", tuple(sorted(e.layer for e in entries)),
            _nm_ring(z.polygon),
            tuple(sorted(_nm_ring(h) for h in (getattr(z, 'holes', None) or ()))),
            int(z.priority or 0), int(z.island_removal_mode or 0),
            mm(z.island_area_min), mm(z.clearance), mm(z.min_thickness),
            z.pad_connection, mm(z.thermal_gap), mm(z.thermal_bridge_width))


def _group_pours(zones):
    """[[Zone entries of one pour], ...]: board-level, non-teardrop entries
    grouped by uuid (in order), a uuid-less entry alone."""
    groups, by_uuid = [], {}
    for z in zones or ():
        if getattr(z, 'in_footprint', False) or getattr(z, 'teardrop', False):
            continue
        u = getattr(z, 'uuid', '') or ''
        if u and u in by_uuid:
            by_uuid[u].append(z)
            continue
        g = [z]
        groups.append(g)
        if u:
            by_uuid[u] = g
    return groups


def _live_pours(board):
    """[(kipy zone, [Zone entries])] of the live board-level copper pours,
    read by the same reader the kipy builder uses."""
    from kicad_parser import _kipy_zone_entries

    def resolve_net(net):
        return 0, (getattr(net, "name", "") or "") if net is not None else ""
    out = []
    for z in list(board.get_zones()):
        try:
            entries = _kipy_zone_entries(z, resolve_net)
        except Exception:                                       # noqa: BLE001
            entries = []
        entries = [e for e in entries if not getattr(e, 'teardrop', False)]
        if entries:
            out.append((z, entries))
    return out


def zone_plan(board, result_pcb):
    """{'keep': [(live, result entries)], 'add': [result entries],
    'remove': [live]} -- the live pours the result carries unchanged, the
    result's pours the live board lacks, and the live pours the result does
    not have. Rule areas and teardrops are not pours and are left alone."""
    result_groups = _group_pours(getattr(result_pcb, 'zones', None))
    pool = {}
    for g in result_groups:
        pool.setdefault(_zone_signature(g), []).append(g)
    keep, remove = [], []
    for lz, entries in _live_pours(board):
        twins = pool.get(_zone_signature(entries))
        if twins:
            keep.append((lz, twins.pop()))
        else:
            remove.append(lz)
    left = {id(g) for gs in pool.values() for g in gs}
    add = [g for g in result_groups if id(g) in left]    # in the result's order
    return {'keep': keep, 'add': add, 'remove': remove}


_CONNECT_STYLES = {'none': 'ZCS_NONE', 'thermal': 'ZCS_THERMAL',
                   'solid': 'ZCS_FULL', 'thru_hole_only': 'ZCS_PTH_THERMAL'}
_ISLAND_MODES = {0: 'IRM_ALWAYS', 1: 'IRM_NEVER', 2: 'IRM_AREA'}


def make_pour(net_map, entries):
    """A kipy copper Zone built from one result pour's Zone ENTRIES: its
    outline and holes, every copper layer, net, priority, clearance, minimum
    thickness, pad connection and spokes, and island removal -- the fields
    the text parser reads and `_zone_signature` compares."""
    from kicad_ipc_adapter import _zone_types, _mm_to_nm, layer_id_for
    Zone, ZoneType, PolyLine, PolyLineNode, PolygonWithHoles = _zone_types()
    z0 = entries[0]

    def polyline(points):
        pl = PolyLine()
        for x, y in points:
            pl.append(PolyLineNode.from_xy(_mm_to_nm(x), _mm_to_nm(y)))
        return pl
    pwh = PolygonWithHoles()
    pwh.outline = polyline(z0.polygon)
    for h in getattr(z0, 'holes', None) or ():
        try:
            pwh.add_hole(polyline(h))
        except Exception:                                       # noqa: BLE001
            pwh.proto.holes.append(polyline(h).proto)
    zone = Zone()
    zone.type = ZoneType.ZT_COPPER
    zone.outline = pwh
    zone.layers = [layer_id_for(e.layer) for e in entries]
    if z0.clearance is not None:
        zone.clearance = _mm_to_nm(z0.clearance)
    if z0.min_thickness is not None:
        zone.min_thickness = _mm_to_nm(z0.min_thickness)
    if z0.priority:
        zone.priority = int(z0.priority)
    try:
        from kipy.proto.board.board_types_pb2 import (ZoneConnectionStyle,
                                                       IslandRemovalMode)
        cs = zone.proto.copper_settings
        if z0.pad_connection in _CONNECT_STYLES:
            cs.connection.zone_connection = ZoneConnectionStyle.Value(
                _CONNECT_STYLES[z0.pad_connection])
        spokes = cs.connection.thermal_spokes
        if z0.thermal_gap is not None:
            spokes.gap.value_nm = _mm_to_nm(z0.thermal_gap)
        if z0.thermal_bridge_width is not None:
            spokes.width.value_nm = _mm_to_nm(z0.thermal_bridge_width)
        cs.island_mode = IslandRemovalMode.Value(
            _ISLAND_MODES.get(int(z0.island_removal_mode or 0), 'IRM_ALWAYS'))
        if z0.island_area_min:
            cs.min_island_area = int(round(float(z0.island_area_min) * 1e12))
    except Exception as e:                                      # noqa: BLE001
        print(f"Warning: zone fill settings not all set on this kipy ({e})")
    if z0.net_name:
        net = net_map.resolve(net_name=z0.net_name)
        if net is not None:
            zone.net = net
    return zone


def apply_zones(commit, plan):
    """Queue ZONE_PLAN on COMMIT: remove, and add a pour built from each
    result pour the live board lacks. Returns counts."""
    for lz in plan['remove']:
        commit.remove(lz)
    for entries in plan['add']:
        commit.add(make_pour(commit.net_map, entries))
    return {'kept': len(plan['keep']), 'removed': len(plan['remove']),
            'added': len(plan['add'])}


# ---------------------------------------------------------- design rules

#: The project rule keys a result's floors are read from (the live writer's
#: own set, fix_kicad_drc_settings.apply_targets_to_board).
_RULE_KEYS = ('min_clearance', 'min_track_width', 'min_via_diameter',
              'min_through_hole_diameter', 'min_hole_to_hole',
              'min_copper_edge_clearance', 'min_hole_clearance',
              'min_via_annular_width', 'min_connection', 'min_via_drill')


def result_rules(result_path):
    """(targets, severities, classes) the result's sibling .kicad_pro holds:
    its rule floors and Default class clearance as apply_targets_to_project
    targets, its relaxed check severities, and {class name: clearance} of its
    other net classes. ({}, {}, {}) when it has no project."""
    pro = os.path.splitext(result_path)[0] + '.kicad_pro'
    try:
        with open(pro, encoding='utf-8') as fh:
            proj = json.load(fh)
    except (OSError, ValueError):
        return {}, {}, {}
    ds = (proj.get('board') or {}).get('design_settings') or {}
    rules = ds.get('rules') or {}
    targets = {k: float(rules[k]) for k in _RULE_KEYS
               if isinstance(rules.get(k), (int, float))}
    classes = {}
    for nc in (proj.get('net_settings') or {}).get('classes') or []:
        if isinstance(nc, dict) and isinstance(nc.get('clearance'), (int, float)):
            classes[nc.get('name')] = float(nc['clearance'])
    if 'Default' in classes:
        targets['class_clearance'] = classes.pop('Default')
    sev = {k: v for k, v in (ds.get('rule_severities') or {}).items()
           if v in ('ignore', 'warning')}
    return targets, sev, classes


def apply_rules(result_path, fix_drc=True, board_path=None):
    """Lower the live board's PROJECT FILE to the result project's floors and
    classes, and carry the result's records into it. Returns change strings.

    FIX_DRC is the dialog's "Fix DRC settings after routing" box: unticked,
    the floors, classes and severities are left as they are (#693), and only
    the records travel. The IPC twin of main's live writer: kipy cannot write
    the live design settings, so the file is written and KiCad uses it once
    the project is reloaded (said on the console, as every IPC writeback)."""
    if board_path is None:
        from kicad_ipc_adapter import get_board_full_path
        board_path = get_board_full_path()
    if not board_path:
        return []
    pro = os.path.splitext(board_path)[0] + '.kicad_pro'
    if not os.path.isfile(pro):
        return []
    import tool_records
    changes = []
    try:
        proj = tool_records.load_project(pro)
    except (OSError, ValueError) as e:
        print(f"(rules not carried: {e})")
        return []
    targets, sev, classes = result_rules(result_path)
    if fix_drc and (targets or sev or classes):
        from fix_kicad_drc_settings import apply_targets_to_project
        # Each other class to the result's own value, only-lower: no ceiling
        # binds, and the result's clearance is the class's "descent".
        changes += apply_targets_to_project(
            proj, targets, sev,
            clamp_nondefault_netclasses=bool(classes), class_ceiling=1e6,
            class_descents={c: (v,) for c, v in classes.items()}) or []
    try:
        ns = tool_records.effective_namespace(result_path)
        if ns and ns != proj.get(tool_records.NAMESPACE):
            proj[tool_records.NAMESPACE] = json.loads(json.dumps(ns))
            changes.append('kicad_routing_tools records')
    except Exception as e:                                      # noqa: BLE001
        print(f"(records not carried: {e})")
    if changes:
        tool_records.stamp_revision(proj)
        tmp = pro + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(proj, f, indent=2)
            f.write('\n')
        os.replace(tmp, pro)
        print(f"Apply: {len(changes)} design-rule change(s) written to {pro}; "
              f"reload the project so KiCad uses them (the IPC API cannot "
              f"change them in the live editor)")
    return changes


# -------------------------------------------------------------- the whole

NOTHING_TO_CHANGE = ("Nothing to change: the open board already matches the "
                     "result -- its parts, locks, tracks, vias and zones.")


class ApplyPlan:
    """What Apply will change, computed against the live board as it is now."""

    def __init__(self, result_path, result_pcb, moves, outline_skipped,
                 unreachable, locks, copper, zones, rules, fix_drc=True):
        self.result_path = result_path
        self.result_pcb = result_pcb
        self.moves = moves
        self.outline_skipped = outline_skipped
        self.unreachable = unreachable
        self.locks = locks
        self.copper = copper
        self.zones = zones
        self.rules = rules          # the result's project declares floors
        self.fix_drc = fix_drc      # ... and the dialog may lower the live ones

    @property
    def has_changes(self):
        """Does Apply change the board: a part moved, locked or unlocked, a
        track or via removed or added, a zone removed or added? Design rules
        alone do not count: they are only-lowered to the result's, and a run
        that changes no copper has no new floor to bring."""
        c, z = self.copper or {}, self.zones or {}
        return bool(self.moves or self.locks
                    or c.get('removed') or c.get('tracks_added') or c.get('vias_added')
                    or z.get('remove') or z.get('add'))

    def lines(self):
        """The confirmation box's text, one paragraph per kind of change.
        Only what is non-zero is named; a plan that changes nothing says so
        in one line (the callers then show it with OK alone, not Continue)."""
        if not self.has_changes:
            return [NOTHING_TO_CHANGE]

        def refs(rs):
            return ", ".join(rs[:30]) + (", ..." if len(rs) > 30 else "")

        def counts(parts):
            return ", ".join(f"{n} {what}" for n, what in parts if n)
        out = []
        if self.moves:
            out.append(f"{len(self.moves)} footprint(s) will move: "
                       f"{refs([r for r, _f, _p in self.moves])}")
        to_lock = [r for r, _f, want in self.locks if want]
        to_unlock = [r for r, _f, want in self.locks if not want]
        if to_lock:
            out.append(f"{len(to_lock)} footprint(s) will be locked, as the result "
                       f"has them: {refs(to_lock)}")
        if to_unlock:
            out.append(f"{len(to_unlock)} footprint(s) will be unlocked, as the "
                       f"result has them: {refs(to_unlock)}")
        if self.outline_skipped:
            out.append(f"Not moved: {', '.join(self.outline_skipped)} -- these draw "
                       f"the board outline (#829).")
        if self.unreachable:
            out.append(f"Not applied: {len(self.unreachable)} part(s) only in the "
                       f"result: {refs(self.unreachable)}")
        if self.copper is not None:
            c = self.copper
            what = counts([(c['removed'], "removed (live items the result does not have)"),
                           (c['tracks_added'], "track(s) added"),
                           (c['vias_added'], "via(s) added"),
                           (c['kept'], "unchanged kept")])
            if c['removed'] or c['tracks_added'] or c['vias_added']:
                line = f"Tracks and vias: {what}."
                if c.get('arcs_removed'):
                    line += (f" {c['arcs_removed']} of the removed items are arcs; "
                             f"the result's copper in their place is short "
                             f"straight segments.")
                out.append(line)
        if self.zones is not None:
            z = self.zones
            if z['remove'] or z['add']:
                what = counts([(len(z['remove']), "removed"),
                               (len(z['add']), "added from the result"),
                               (len(z['keep']), "unchanged kept")])
                out.append(f"Zones: {what}"
                           + (", refilled by KiCad after the change (rule areas "
                              "and teardrops are left as they are)"
                              if z['add'] else "") + ".")
        if self.rules and self.fix_drc:
            out.append("Design rules: the project's floors and net classes are "
                       "lowered to the result project's, as a routing step does "
                       "(written to the project file; reload the project so "
                       "KiCad uses them).")
        elif self.rules:
            out.append("Design rules: not changed -- 'Fix DRC settings after "
                       "routing' is unticked, so the live board keeps its floors "
                       "and KiCad may grade the result's fine copper against them.")
        return out


def plan_apply(board, result_path, *, copper=True, zones=True, rules=True,
               fix_drc=True):
    """An ApplyPlan for making BOARD match the result at RESULT_PATH. FIX_DRC
    is the dialog's "Fix DRC settings after routing" box (apply_rules)."""
    from kicad_parser import parse_kicad_pcb
    result_pcb = parse_kicad_pcb(result_path)
    live = live_footprints_by_key(board)
    moves, outline_skipped, unreachable = pose_moves(board, result_pcb, live)
    locks = lock_changes(board, result_pcb, live)
    cop = copper_plan(board, result_pcb) if copper else None
    zp = zone_plan(board, result_pcb) if zones else None
    has_rules = rules and any(result_rules(result_path))
    return ApplyPlan(result_path, result_pcb, moves, outline_skipped,
                     unreachable, locks, cop, zp, has_rules, fix_drc=fix_drc)


def apply_poses_and_copper(board, result_pcb, moves, locks=(), copper=None,
                           zones=None, message="KiCadRoutingTools: apply result"):
    """Poses, locks, COPPER (a `copper_plan`) and ZONES (a `zone_plan`) in ONE
    IPC commit, so the board is never left half-applied and the whole Apply is
    one undo step. Returns (copper counts or None, zone counts or None)."""
    from kicad_ipc_adapter import begin_commit, set_item_locked
    cstats = zstats = None
    with begin_commit(board, message) as commit:
        updated = {}
        for _ref, fp, parsed in moves:
            apply_pose(fp, parsed)
            updated[id(fp)] = fp
        # #1054: the result board's KiCad locks, in both directions.
        for _ref, fp, want in locks:
            set_item_locked(fp, want)
            updated[id(fp)] = fp
        for fp in updated.values():
            commit.update(fp)
        if copper is not None:
            cstats = apply_copper(commit, result_pcb, copper)
        if zones is not None:
            zstats = apply_zones(commit, zones)
    if copper is not None:
        disclose_via_protection(result_pcb)
    return cstats, zstats


def run_apply(board, plan):
    """Carry out PLAN on BOARD. Returns a stats dict."""
    c, z = apply_poses_and_copper(board, plan.result_pcb, plan.moves, plan.locks,
                                  copper=plan.copper, zones=plan.zones)
    stats = {'moved': len(plan.moves), 'locks': len(plan.locks)}
    if c is not None:
        stats['copper'] = c
    if z is not None:
        stats['zones'] = z
        if z['added']:
            # The pours are built, not copied with their fills: KiCad fills them.
            try:
                board.refill_zones()
            except Exception as e:                              # noqa: BLE001
                print(f"(zone refill not run: {e}; press B in KiCad to refill)")
    stats['rules'] = apply_rules(plan.result_path, fix_drc=plan.fix_drc)
    return stats
