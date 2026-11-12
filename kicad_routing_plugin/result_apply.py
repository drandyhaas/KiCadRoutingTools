"""
KiCad Routing Tools - make the live board match a result board file.

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
  zones             the board-level zones (pours, keep-outs, teardrops; a
                    footprint's own zones travel with it): a live zone the
                    result carries unchanged is kept and takes the result's
                    fill, the rest are replaced by copies of the result's,
                    fills included -- not refilled, so the board grades as
                    the result file does
  design rules      the result project's rule floors and Default net class,
                    lowered onto the live board's design settings (only-
                    lower, as every GUI step's writeback), and the result's
                    `kicad_routing_tools` records, so KiCad's own save writes
                    the project the result was graded against

Every function takes the live BOARD and pcbnew; nothing here builds UI. The
caller lists the plan (`plan_apply`) in its confirmation box, then calls
`run_apply`.
"""

import json
import os


# ------------------------------------------------------------- footprints

def pose_moves(board, result_pcb, pcbnew):
    """([(ref, live_fp, parsed_fp)] whose pose differs, [outline owners not
    moved], [refs only in the result]).

    #829: a footprint that draws the board's own outline is skipped: moving it
    would resize the board. This applier writes poses with SetPosition /
    SetOrientationDegrees / Flip and never goes through the CLI writer, so
    the CLI-side guard does not reach it; and the move list is a diff of the
    RESULT against the LIVE board, so a live board that drifted from the
    staged input produces moves the CLI never proposed. Resolved by the
    PCBData key, never the bare reference (#726)."""
    from .gui_utils import live_footprints_by_key
    live = live_footprints_by_key(board)
    moves, outline_skipped, unreachable = [], [], []
    for ref, parsed in sorted(result_pcb.footprints.items()):
        if getattr(parsed, 'owns_board_outline', False):
            outline_skipped.append(ref)
            continue
        fp = live.get(ref) or board.FindFootprintByReference(ref)
        if fp is None:
            unreachable.append(ref)    # never invent parts
            continue
        pos = fp.GetPosition()
        live_x, live_y = pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)
        live_rot = fp.GetOrientationDegrees() % 360
        parsed_back = str(parsed.layer or "F.Cu").startswith("B")
        if (abs(live_x - parsed.x) > 1e-4 or abs(live_y - parsed.y) > 1e-4
                or abs((live_rot - (parsed.rotation or 0) % 360 + 180) % 360 - 180) > 1e-3
                or bool(fp.IsFlipped()) != parsed_back):
            moves.append((ref, fp, parsed))
    return moves, outline_skipped, unreachable


def lock_changes(board, result_pcb):
    """[(ref, live_fp, want_locked)] for every footprint whose KiCad lock
    differs from the result board's (#1054), in both directions. An outline
    owner is included: a lock moves nothing."""
    from .gui_utils import live_footprints_by_key
    live = live_footprints_by_key(board)
    changes = []
    for ref, parsed in sorted(result_pcb.footprints.items()):
        fp = live.get(ref) or board.FindFootprintByReference(ref)
        if fp is None:
            continue
        want = bool(getattr(parsed, 'locked', False))
        if bool(fp.IsLocked()) != want:
            changes.append((ref, fp, want))
    return changes


def apply_pose(fp, parsed, pcbnew):
    from kicad_parser import mm_to_iu
    parsed_back = str(parsed.layer or "F.Cu").startswith("B")
    if fp.IsFlipped() != parsed_back:
        fp.Flip(fp.GetPosition(), False)
    fp.SetPosition(pcbnew.VECTOR2I(mm_to_iu(parsed.x), mm_to_iu(parsed.y)))
    fp.SetOrientationDegrees(parsed.rotation or 0)


# ------------------------------------------------------------ tracks/vias

def apply_copper(board, result_pcb, pcbnew, dry_run=False, order_board=None):
    """Make the live board's tracks and vias the result board's copper.

    A live track, arc or via the result carries UNCHANGED -- same geometry,
    layer, width and net, and for a via the same size, drill, layer pair and
    type -- is KEPT rather than removed and recreated (#1279): recreating gave
    every item a new uuid, which took it out of any KiCad group (a `sense:`
    group lost its members) and brought an unchanged arc back as chords. A
    kept via takes the result's protection spec. A created item takes its
    KiCad lock from the result's `locked` (#521); a sense hold is `held_by`,
    never a lock. RemoveNative, never Remove (see the SWIG-registry warning
    in swig_gui).

    Returns {'kept', 'removed', 'tracks_added', 'vias_added', 'nets'}, where
    'nets' is [(item, net code)] for every item the result decides the net
    of, so a caller re-pins them after BuildConnectivity (which reassigns a
    netcode over a stale fill). DRY_RUN counts and changes nothing.
    ORDER_BOARD, the result loaded by pcbnew, gives the order the created
    items go on in (its own); without it they keep the parse's order."""
    from kicad_parser import mm_to_iu, pcbnew_track_copper
    from .swig_gui import _build_layer_mappings
    from .gui_utils import apply_via_protection, set_via_layers
    name_to_id, _ = _build_layer_mappings()

    # Result-board net ids can differ from live net codes; map by name.
    def live_net_code(net_id):
        net = result_pcb.nets.get(net_id)
        if net is not None:
            live = board.FindNet(net.name)
            if live is not None:
                return live.GetNetCode()
        return net_id

    def result_net_name(net_id):
        net = result_pcb.nets.get(net_id)
        return net.name if net is not None else ''

    # Keys compare in internal units, the resolution both boards hold.
    def seg_key(s, net_name):
        a = (mm_to_iu(s.start_x), mm_to_iu(s.start_y))
        b = (mm_to_iu(s.end_x), mm_to_iu(s.end_y))
        return (min(a, b), max(a, b), mm_to_iu(s.width), s.layer, net_name)

    def via_key(v, net_name):
        return (mm_to_iu(v.x), mm_to_iu(v.y), mm_to_iu(v.size),
                mm_to_iu(v.drill), tuple(v.layers or ()),
                getattr(v, 'via_type', 'through'), net_name)

    # The result's copper, by key. Copper GRAPHICS parse into segments too
    # (#337 board gr_*, #908 footprint fp_*): they are drawings, not tracks.
    want_segs, want_vias = {}, {}
    for seg in result_pcb.segments:
        if getattr(seg, "graphic", False):
            continue
        want_segs.setdefault(seg_key(seg, result_net_name(seg.net_id)), []).append(seg)
    for via in result_pcb.vias:
        want_vias.setdefault(via_key(via, result_net_name(via.net_id)), []).append(via)

    # The live copper through the extractor every live read uses, so an arc's
    # pieces are chorded exactly as the result's parse chords them; `owners`
    # maps each piece back to its one live item.
    owners = {}
    live_segs, live_vias = pcbnew_track_copper(board, owners=owners)
    items = {}
    for s in live_segs:
        it = owners[id(s)]
        items.setdefault(id(it), (it, 'seg', []))[2].append(seg_key(s, it.GetNetname()))
    for v in live_vias:
        it = owners[id(v)]
        items[id(it)] = (it, 'via', [via_key(v, it.GetNetname())])

    kept, removed = [], []
    for it, kind, keys in items.values():
        pool = want_segs if kind == 'seg' else want_vias
        need = {}
        for k in keys:
            need[k] = need.get(k, 0) + 1
        if all(len(pool.get(k, ())) >= n for k, n in need.items()):
            kept.append((it, kind, [pool[k].pop() for k in keys]))
        else:
            removed.append(it)
    add_segs = [s for segs in want_segs.values() for s in segs]
    add_vias = [v for vias in want_vias.values() for v in vias]
    out = {'kept': len(kept), 'removed': len(removed),
           'tracks_added': len(add_segs), 'vias_added': len(add_vias), 'nets': []}
    if dry_run:
        return out

    for it in removed:
        board.RemoveNative(it)
    for it, kind, taken in kept:
        out['nets'].append((it, it.GetNetCode()))
        if kind == 'via':
            # Keep the result's tenting/plugging spec (#489 §8); an empty spec
            # leaves the via inheriting the board's, as it was.
            apply_via_protection(it, getattr(taken[0], "tenting_attrs", None))
    def make_track(seg):
        track = pcbnew.PCB_TRACK(board)
        # mm_to_iu, never FromMM(round(...)) (#493 item 5 / #362).
        track.SetStart(pcbnew.VECTOR2I(mm_to_iu(seg.start_x), mm_to_iu(seg.start_y)))
        track.SetEnd(pcbnew.VECTOR2I(mm_to_iu(seg.end_x), mm_to_iu(seg.end_y)))
        track.SetWidth(mm_to_iu(seg.width))
        track.SetLayer(name_to_id.get(seg.layer, pcbnew.F_Cu))
        track.SetNetCode(live_net_code(seg.net_id))
        # #521: user-pinned copper must come back pinned.
        track.SetLocked(bool(getattr(seg, "locked", False)))
        return track

    def make_via(via):
        pcb_via = pcbnew.PCB_VIA(board)
        pcb_via.SetPosition(pcbnew.VECTOR2I(mm_to_iu(via.x), mm_to_iu(via.y)))
        pcb_via.SetWidth(mm_to_iu(via.size))
        pcb_via.SetDrill(mm_to_iu(via.drill))
        pcb_via.SetNetCode(live_net_code(via.net_id))
        if getattr(via, "layers", None) and len(via.layers) >= 2:
            set_via_layers(pcb_via, name_to_id.get(via.layers[0], pcbnew.F_Cu),
                           name_to_id.get(via.layers[-1], pcbnew.B_Cu),
                           getattr(via, "via_type", None))
        pcb_via.SetLocked(bool(getattr(via, "locked", False)))  # #521
        # Keep a via's own tenting/plugging spec (#489 §8).
        apply_via_protection(pcb_via, getattr(via, "tenting_attrs", None))
        return pcb_via

    # The live board ends up holding its tracks and vias in the RESULT's
    # order, and a created item takes the result item's own uuid. KiCad's
    # zone filler is not order-free at the nanometre (measured: a board
    # refilled with its tracks reversed moves one vertex 1 nm), and KiCad
    # saves tracks sorted by type, layer and uuid. So a refill in the session
    # after Apply is the refill of the result file opened in KiCad, and the
    # applied board, saved, lists its tracks as the result saved by KiCad.
    # A kept item is moved to its place (out of the list and back, its group
    # restored: BOARD::Remove takes an item out of its group); its uuid and
    # group membership are unchanged. Without ORDER_BOARD: created items in
    # the parse's order, after the kept ones.
    found = _file_order(order_board, pcbnew) if order_board is not None else {}
    _n2i, id_to_name = _build_layer_mappings()
    tail = 10 ** 9
    seq = []
    for it, _kind, _taken in kept:
        hits = found.get(_live_order_key(it, id_to_name, pcbnew)) or []
        index, uuid = hits.pop(0) if hits else (-1, None)
        seq.append((index, 'kept', it, uuid))
    uuids = {it.m_Uuid.AsString() for it, _k, _t in kept}
    for n, (kind, item) in enumerate([('seg', s) for s in add_segs]
                                     + [('via', v) for v in add_vias]):
        hits = found.get(_parsed_order_key(kind, item, mm_to_iu)) or []
        index, uuid = hits.pop(0) if hits else (tail + n, None)
        seq.append((index, kind, item, uuid))
    seq.sort(key=lambda r: r[0])
    for index, kind, item, uuid in seq:
        if kind == 'kept':
            if found and index >= 0:
                # The result's twin may carry another uuid: a step ripped
                # this copper and laid it again, identically. The item takes
                # the result's uuid, as the result has it.
                mine = item.m_Uuid.AsString()
                new = uuid if uuid and uuid != mine and uuid not in uuids else None
                if new:
                    uuids.discard(mine)
                    uuids.add(new)
                _move_to_end(board, item, pcbnew, new_uuid=new)
            continue
        made = make_track(item) if kind == 'seg' else make_via(item)
        if uuid and uuid not in uuids:
            made.SetUuid(pcbnew.KIID(uuid))
            uuids.add(uuid)
        _append(board, made, pcbnew)
        out['nets'].append((made, made.GetNetCode()))
    print(f"Apply: {len(kept)} unchanged track(s)/via(s) kept, {len(removed)} "
          f"removed, {len(add_segs)} track(s) and {len(add_vias)} via(s) added")
    return out


def _append(board, item, pcbnew):
    """board.Add, at the END of the board's list: the Python Add takes no
    mode and INSERTs at the front, which reverses a run of added items."""
    item.thisown = 0            # what board.Add does: the board owns it now
    board.AddNative(item, pcbnew.ADD_MODE_APPEND)


def _parsed_order_key(kind, item, nm):
    """An order key for a PARSED segment or via (see _file_order)."""
    if kind == 'seg':
        a, b = (nm(item.start_x), nm(item.start_y)), (nm(item.end_x), nm(item.end_y))
        return ('seg', min(a, b), max(a, b), nm(item.width), item.layer)
    return ('via', nm(item.x), nm(item.y), nm(item.size))


def _live_order_key(t, id_to_name, pcbnew):
    """The order key of a pcbnew track or via (see _parsed_order_key)."""
    if t.GetClass() == 'PCB_VIA':
        p = t.GetPosition()
        return ('via', p.x, p.y, t.GetWidth(pcbnew.F_Cu))
    if t.GetClass() == 'PCB_TRACK':
        a = (t.GetStart().x, t.GetStart().y)
        b = (t.GetEnd().x, t.GetEnd().y)
        return ('seg', min(a, b), max(a, b), t.GetWidth(),
                id_to_name.get(t.GetLayer(), t.GetLayerName()))
    return None                 # an arc: the parse chords it


def _file_order(result_board, pcbnew):
    """{order key: [(index, uuid), ...]} of the result board's tracks and
    vias, in the order the board holds them (its file's)."""
    from .swig_gui import _build_layer_mappings
    _n2i, id_to_name = _build_layer_mappings()
    found = {}
    for i, t in enumerate(result_board.GetTracks()):
        key = _live_order_key(t, id_to_name, pcbnew)
        if key is not None:
            found.setdefault(key, []).append((i, t.m_Uuid.AsString()))
    return found


def _move_to_end(board, item, pcbnew, new_uuid=None):
    """Move a live board ITEM to the end of the board's list, keeping the
    item itself and its group; NEW_UUID, when given, is set while the item
    is off the board, so the board's own uuid index follows it."""
    group = item.GetParentGroup()
    board.RemoveNative(item)
    item.ClearFlags(pcbnew.STRUCT_DELETED)
    if new_uuid:
        item.SetUuid(pcbnew.KIID(new_uuid))
    _append(board, item, pcbnew)
    if group is not None and item.GetParentGroup() is None:
        group.AddItem(item)


def repin_nets(pinned):
    """Give every (item, net code) of PINNED its net back: BuildConnectivity
    over a fill reassigns a touching item's netcode to the zone's (#362)."""
    fixed = 0
    for it, code in pinned:
        try:
            if it.GetNetCode() != code:
                it.SetNetCode(code)
                fixed += 1
        except Exception:                                       # noqa: BLE001
            pass
    return fixed


# ------------------------------------------------------------------ zones

def _board_zones(board):
    """The board-level zones (a footprint's own zones are not in Zones())."""
    return list(board.Zones())


def _zone_signature(z):
    """What makes two zones the same zone: net, layers, rule-area kind,
    priority, outline and the fill settings. Fills are not compared -- a kept
    zone takes the result's."""
    def get(name, *args):
        try:
            v = getattr(z, name)(*args)
            return str(v)
        except Exception:                                       # noqa: BLE001
            return None
    try:
        ol = z.Outline()
        outline = tuple((ol.CVertex(i).x, ol.CVertex(i).y)
                        for i in range(ol.TotalVertices()))
    except Exception:                                           # noqa: BLE001
        outline = None
    try:
        layers = tuple(int(ly) for ly in z.GetLayerSet().Seq())
    except Exception:                                           # noqa: BLE001
        layers = (z.GetLayer(),)
    return (z.GetNetname(), layers, outline,
            get('GetIsRuleArea'), get('GetAssignedPriority'), get('GetZoneName'),
            get('GetLocalClearance'), get('GetMinThickness'), get('GetPadConnection'),
            get('GetThermalReliefGap'), get('GetThermalReliefSpokeWidth'),
            get('GetIslandRemovalMode'), get('GetFillMode'))


def zone_plan(board, result_board):
    """{'keep': [(live, result)], 'add': [result], 'remove': [live]} -- the
    live board-level zones the result carries unchanged, the result's zones
    the live board lacks, and the live zones the result does not have."""
    result_zones = _board_zones(result_board)
    pool = {}
    for rz in result_zones:
        pool.setdefault(_zone_signature(rz), []).append(rz)
    keep, remove = [], []
    for lz in _board_zones(board):
        twins = pool.get(_zone_signature(lz))
        if twins:
            keep.append((lz, twins.pop()))
        else:
            remove.append(lz)
    left = {id(rz) for zs in pool.values() for rz in zs}
    add = [rz for rz in result_zones if id(rz) in left]  # in the result's order
    return {'keep': keep, 'add': add, 'remove': remove}


def _copy_fill(dst, src):
    for ly in src.GetLayerSet().Seq():
        dst.SetFilledPolysList(ly, src.GetFilledPolysList(ly))
    dst.SetIsFilled(src.IsFilled())
    try:
        dst.SetNeedRefill(False)
    except Exception:                                           # noqa: BLE001
        pass


def apply_zones(board, plan, pcbnew):
    """Carry out ZONE_PLAN: keep (taking the result's fill), remove, and add a
    copy of each result zone with its net mapped by name. Returns counts."""
    for lz, rz in plan['keep']:
        _copy_fill(lz, rz)
    for lz in plan['remove']:
        board.RemoveNative(lz)
    added = []
    for rz in plan['add']:
        nz = pcbnew.ZONE(rz)          # the copy carries outline, settings and fill
        nz.SetParent(board)
        _append(board, nz, pcbnew)              # in the result's order
        net = board.FindNet(rz.GetNetname()) if rz.GetNetCode() else None
        nz.SetNetCode(net.GetNetCode() if net is not None else 0)
        _copy_fill(nz, rz)
        added.append(nz)
    return {'kept': len(plan['keep']), 'removed': len(plan['remove']),
            'added': len(added)}


# ---------------------------------------------------------- design rules

#: The project rule keys a result's floors are read from (the live writer's
#: own set, fix_kicad_drc_settings.apply_targets_to_board).
_RULE_KEYS = ('min_clearance', 'min_track_width', 'min_via_diameter',
              'min_through_hole_diameter', 'min_hole_to_hole',
              'min_copper_edge_clearance', 'min_hole_clearance',
              'min_via_annular_width', 'min_connection', 'min_via_drill')


def result_rules(result_path):
    """(targets, severities, classes) the result's sibling .kicad_pro holds:
    its rule floors and Default class clearance as apply_targets_to_board
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


def apply_rules(board, result_path, fix_drc=True):
    """Lower the live board's design settings to the result project's, and
    carry the result's records onto the board. Returns change strings.

    FIX_DRC is the dialog's "Fix DRC settings after routing" box: unticked,
    the floors, classes and severities are left as they are (#693, the
    rule every GUI writer of them keeps), and only the records travel."""
    changes = []
    targets, sev, classes = result_rules(result_path)
    if fix_drc and (targets or sev):
        from fix_kicad_drc_settings import apply_targets_to_board
        changes += apply_targets_to_board(board, targets, sev) or []
    if fix_drc and classes:
        # Each other class to the result's own value, only-lower: no ceiling
        # binds, and the result's clearance is the class's "descent".
        from fix_kicad_drc_settings import clamp_nondefault_netclasses_on_board
        changes += clamp_nondefault_netclasses_on_board(
            board, {}, class_ceiling=1e6,
            class_descents={c: (v,) for c, v in classes.items()}) or []
    try:
        import tool_records
        ns = tool_records.effective_namespace(result_path)
        if ns:
            tool_records.write_live_namespace(board, ns)
            changes.append('kicad_routing_tools records')
    except Exception as e:                                      # noqa: BLE001
        print(f"(records not carried: {e})")
    return changes


# -------------------------------------------------------------- the whole

class ApplyPlan:
    """What Apply will change, computed against the live board as it is now."""

    def __init__(self, result_path, result_pcb, result_board, moves, outline_skipped,
                 unreachable, locks, copper, zones, rules, fix_drc=True):
        self.result_path = result_path
        self.result_pcb = result_pcb
        self.result_board = result_board
        self.moves = moves
        self.outline_skipped = outline_skipped
        self.unreachable = unreachable
        self.locks = locks
        self.copper = copper
        self.zones = zones
        self.rules = rules          # the result's project declares floors
        self.fix_drc = fix_drc      # ... and the dialog may lower the live ones

    def lines(self):
        """The confirmation box's text, one paragraph per kind of change."""
        def refs(rs):
            return ", ".join(rs[:30]) + (", ..." if len(rs) > 30 else "")
        out = [f"{len(self.moves)} footprint(s) will move"
               + (f": {refs([r for r, _f, _p in self.moves])}" if self.moves else ".")]
        to_lock = [r for r, _f, want in self.locks if want]
        to_unlock = [r for r, _f, want in self.locks if not want]
        if to_lock:
            out.append(f"{len(to_lock)} footprint(s) will be LOCKED, as the result "
                       f"has them: {refs(to_lock)}")
        if to_unlock:
            out.append(f"{len(to_unlock)} footprint(s) will be UNLOCKED, as the "
                       f"result has them: {refs(to_unlock)}")
        if self.outline_skipped:
            out.append(f"NOT moved: {', '.join(self.outline_skipped)} -- these draw "
                       f"the board outline (#829).")
        if self.unreachable:
            out.append(f"NOT applied: {len(self.unreachable)} part(s) only in the "
                       f"result: {refs(self.unreachable)}")
        if self.copper is not None:
            c = self.copper
            out.append(f"Tracks and vias: {c['kept']} unchanged kept, {c['removed']} "
                       f"live item(s) the result does not have REMOVED, "
                       f"{c['tracks_added']} track(s) and {c['vias_added']} via(s) "
                       f"added. Replaced arcs come back as short straight segments.")
        if self.zones is not None:
            z = self.zones
            out.append(f"Zones: {len(z['keep'])} unchanged kept, {len(z['remove'])} "
                       f"removed, {len(z['add'])} added from the result, with the "
                       f"result's fills (not refilled; press B in KiCad to refill).")
        if self.rules and self.fix_drc:
            out.append("Design rules: the live board's floors and net classes are "
                       "lowered to the result project's, as a routing step does.")
        elif self.rules:
            out.append("Design rules: NOT changed -- 'Fix DRC settings after "
                       "routing' is unticked, so the live board keeps its floors "
                       "and KiCad may grade the result's fine copper against them.")
        return out


def plan_apply(board, result_path, pcbnew, *, copper=True, zones=True, rules=True,
               fix_drc=True):
    """An ApplyPlan for making BOARD match the result at RESULT_PATH. FIX_DRC
    is the dialog's "Fix DRC settings after routing" box (apply_rules)."""
    from kicad_parser import parse_kicad_pcb
    result_pcb = parse_kicad_pcb(result_path)
    moves, outline_skipped, unreachable = pose_moves(board, result_pcb, pcbnew)
    locks = lock_changes(board, result_pcb)
    cop = apply_copper(board, result_pcb, pcbnew, dry_run=True) if copper else None
    result_board = pcbnew.LoadBoard(result_path) if zones else None
    zp = zone_plan(board, result_board) if zones else None
    has_rules = rules and any(result_rules(result_path))
    return ApplyPlan(result_path, result_pcb, result_board, moves, outline_skipped,
                     unreachable, locks, cop, zp, has_rules, fix_drc=fix_drc)


def run_apply(board, plan, pcbnew):
    """Carry out PLAN on BOARD. Returns a stats dict."""
    for _ref, fp, parsed in plan.moves:
        apply_pose(fp, parsed, pcbnew)
    for _ref, fp, want in plan.locks:
        fp.SetLocked(want)
    stats = {'moved': len(plan.moves), 'locks': len(plan.locks)}
    pinned = []
    if plan.copper is not None:
        c = apply_copper(board, plan.result_pcb, pcbnew, order_board=plan.result_board)
        pinned = c.pop('nets')
        stats['copper'] = c
    if plan.zones is not None:
        stats['zones'] = apply_zones(board, plan.zones, pcbnew)
    stats['rules'] = apply_rules(board, plan.result_path, fix_drc=plan.fix_drc)
    board.BuildConnectivity()
    stats['repinned'] = repin_nets(pinned)
    if stats['repinned']:
        board.BuildConnectivity()
    try:
        pcbnew.Refresh()
    except Exception:                                           # noqa: BLE001
        pass
    return stats
