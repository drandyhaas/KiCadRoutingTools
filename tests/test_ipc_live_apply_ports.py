#!/usr/bin/env python3
"""main's live-board GUI fixes, as the IPC front applies them.

main's SWIG front gained, since the last sync, live-board behaviour that this
branch has to reach through kipy instead of pcbnew. Each arm here drives the
IPC code with REAL kipy objects (kicad-python's own Track / ArcTrack / Via /
Zone / FootprintInstance) on a recording stand-in board, so what is asserted
is what the adapter would hand KiCad over the socket:

  A. a re-created blind / buried / micro via keeps its TYPE (#1268) and its
     span; an untyped via is through whatever pair it names, as the CLI
     writer emits it
  B. `set_item_locked` (the write half of kipy_locked) reaches the proto on
     every item kind (#521 / #1054)
  C. the Planes apply: "Skip existing zones" UNTICKED replaces the net's
     single-layer pour on the layer (a78cc7d2) and pours beside a multi-layer
     one; teardrops and rule areas are no pour (#1164); #448's NPTH-slot
     keep-out is a RULE AREA, not the net-less copper zone this loop used to
     build; each pour carries the engine's priority and pad connection
  C2. #1299: a multi-layer pour the run re-pours on every layer it covers is
     replaced, once; one re-poured on only some of its layers is kept
  D. the footprint mover resolves a duplicated reference's second block by
     its PCBData key `TP4~2` (#726) and does not move a part that draws the
     board outline (#829)
  E. the Placement tab's Apply mirrors the result board's KiCad locks in both
     directions (#1054) and KEEPS live copper the result carries unchanged
     instead of recreating it, so a `sense:` group keeps its members (#1279)
  F. the dialog's re-sync patches footprint and pad poses IN PLACE (#362),
     keeping the Pad objects pads_by_net shares
  G. #1556: every strip in the adapter is counted, layer-keyed and
     width-preferring (`strip_listed_tracks`, kicad_writer.match_stack_members)
     -- the routing apply, and the Planes apply's finalize strips, which it
     used to drop on the floor; no endpoint-SET strip is left in the adapter
  H. the project writeback takes every keyword the shared writeback
     deciders hand it (fanout_fix_kwargs' `class_ceiling` / `hole_clearance`,
     #1529/#1568, and plane_fix_kwargs'): a missing one is a TypeError the
     writeback's own except swallows, so the floors are silently never written

The SWIG gates for C/C2/E (test_skip_existing_zones_gui,
test_1299_multilayer_zone_replace_gui, test_placement_apply_locks) drive
pcbnew boards and cannot run here. What
this file does NOT cover is the socket itself: a running KiCad's answer to
these commits.

Needs kipy and wx (KiCad's bundled python); self-skips (77) without them.

    python3 tests/test_ipc_live_apply_ports.py
"""
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import kipy.board_types as bt                                 # noqa: F401
    import wx                                                     # noqa: F401
except Exception as _e:                                          # noqa: BLE001
    import headless_plan
    headless_plan.reexec_into_kicad(os.path.abspath(__file__))
    print(f"SKIP: needs kipy + wx (KiCad's python): {type(_e).__name__}: {_e}")
    sys.exit(77)

from kipy.board_types import (ArcTrack, BoardLayer, FootprintInstance,  # noqa: E402
                              Net, Track, Via, Zone)
from kipy.geometry import Vector2                                # noqa: E402
from kipy.proto.board.board_types_pb2 import ViaType, ZoneType   # noqa: E402
from kipy.proto.common.types import LockedState                  # noqa: E402

import kicad_ipc_adapter as A                                    # noqa: E402
from kicad_parser import kipy_locked, kipy_via_type, parse_kicad_pcb  # noqa: E402

FAILS = []


def check(cond, msg, detail=''):
    print(f"  {'ok  ' if cond else 'FAIL'}  {msg}"
          + (f"  -- {detail}" if (detail and not cond) else ''))
    if not cond:
        FAILS.append(msg)
    return cond


def nm(mm):
    return int(round(mm * 1e6))


def vec(x, y):
    return Vector2.from_xy(nm(x), nm(y))


class Board:
    """The slice of kipy.board.Board the adapter calls, recording commits."""

    def __init__(self, nets=(), footprints=(), tracks=(), vias=(), zones=()):
        self.nets = {}
        for name in nets:
            n = Net()
            n.name = name
            self.nets[name] = n
        self.footprints = list(footprints)
        self.tracks = list(tracks)
        self.vias = list(vias)
        self.zones = list(zones)
        self.created, self.removed, self.updated, self.pushed = [], [], [], []

    def net(self, name):
        return self.nets[name]

    def get_nets(self):
        return list(self.nets.values())

    def get_footprints(self):
        return list(self.footprints)

    def get_tracks(self):
        return list(self.tracks)

    def get_vias(self):
        return list(self.vias)

    def get_zones(self):
        return list(self.zones)

    def begin_commit(self):
        return object()

    def create_items(self, items):
        self.created.extend(items)

    def remove_items(self, items):
        self.removed.extend(items)

    def update_items(self, items):
        self.updated.extend(items)

    def push_commit(self, _h, message=''):
        self.pushed.append(message)

    def drop_commit(self, _h):
        self.pushed.append('<dropped>')


def is_in(item, items):
    return any(item is x for x in items)


# ---------------------------------------------------------------------------
class _NoNets:
    def resolve(self, **_k):
        return None


def test_via_types():
    print("A. make_via keeps a via's type, and the type decides the span")
    want = {'blind': ('F.Cu', 'In1.Cu', ViaType.VT_BLIND),
            'buried': ('In1.Cu', 'In2.Cu', ViaType.VT_BURIED),
            'micro': ('F.Cu', 'In1.Cu', ViaType.VT_MICRO)}
    for vt, (top, bot, enum) in want.items():
        v = A.make_via(_NoNets(), 1, 2, 0.4, 0.2, top, bot, via_type=vt)
        d = v.padstack.drill
        check(v.type == enum and A.layer_name_for(d.start_layer) == top
              and A.layer_name_for(d.end_layer) == bot,
              f"a {vt} via comes back {vt} on {top}-{bot}",
              f"{ViaType.Name(v.type)} {A.layer_name_for(d.start_layer)}-"
              f"{A.layer_name_for(d.end_layer)}")
        check(kipy_via_type(v) == vt, f"and the builder's reader reads it {vt}",
              kipy_via_type(v))
    for vt in (None, 'through'):
        v = A.make_via(_NoNets(), 1, 2, 0.6, 0.3, 'F.Cu', 'In1.Cu', via_type=vt)
        d = v.padstack.drill
        check(v.type == ViaType.VT_THROUGH
              and (A.layer_name_for(d.start_layer),
                   A.layer_name_for(d.end_layer)) == ('F.Cu', 'B.Cu'),
              f"via_type={vt!r} naming F.Cu-In1.Cu is a THROUGH via, as "
              f"kicad_writer emits it (no type word)")


def test_set_item_locked():
    print("B. set_item_locked reaches the proto on every item kind")
    for cls in (Track, ArcTrack, Via, FootprintInstance):
        it = cls()
        A.set_item_locked(it, True)
        locked = it.proto.locked == LockedState.LS_LOCKED and kipy_locked(it)
        A.set_item_locked(it, False)
        check(locked and not kipy_locked(it)
              and it.proto.locked == LockedState.LS_UNLOCKED,
              f"{cls.__name__}: locks and unlocks in the proto")

    class _NoProperty:           # a kipy release without the `locked` property
        def __init__(self):
            self.proto = Track().proto
    it = _NoProperty()
    A.set_item_locked(it, True)
    check(it.proto.locked == LockedState.LS_LOCKED,
          "a class without the property is locked through its proto")


# ---------------------------------------------------------------------------
def _zone(board, net, layers, ztype=ZoneType.ZT_COPPER, name=''):
    z = Zone()
    z.type = ztype
    z.layers = [A.layer_id_for(n) for n in layers]
    if net:
        z.net = board.net(net)
    if name:
        z.name = name
    return z


def test_planes_apply():
    print("C. the Planes apply: replace, teardrops and rule areas, keep-outs")
    b = Board(nets=('GND', 'SIG'))
    gnd_b = _zone(b, 'GND', ['B.Cu'])
    gnd_tear = _zone(b, 'GND', ['F.Cu'], ZoneType.ZT_TEARDROP)
    sig_in1 = _zone(b, 'SIG', ['In1.Cu'])
    rule = _zone(b, None, ['F.Cu', 'B.Cu'], ZoneType.ZT_RULE_AREA, 'npth_slot_1')
    sig_multi = _zone(b, 'SIG', ['F.Cu', 'B.Cu'])
    b.zones = [gnd_b, gnd_tear, sig_in1, rule, sig_multi]
    sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
    new = [dict(net_name='GND', layer='B.Cu', polygon_points=sq,
                replace_existing=True),
           dict(net_name='GND', layer='B.Cu', polygon_points=sq,
                replace_existing=True),          # a split's second polygon
           dict(net_name='SIG', layer='In1.Cu', polygon_points=sq),
           dict(net_name='GND', layer='F.Cu', polygon_points=sq),
           dict(net_name='SIG', layer='B.Cu', polygon_points=sq,
                replace_existing=True, priority=2, thermal_relief=True),
           dict(keepout=True, name='npth_slot_1', layers=['F.Cu', 'B.Cu'],
                polygon_points=sq),
           dict(keepout=True, name='npth_slot_2', layers=['F.Cu', 'B.Cu'],
                polygon_points=sq)]
    pcb = SimpleNamespace(nets={})
    c = A.apply_planes_results(b, pcb_data=pcb, new_zones=new)
    check(c['zones_replaced'] == 1 and is_in(gnd_b, b.removed)
          and len(b.removed) == 1,
          "the GND pour on B.Cu is replaced, ONCE, though two polygons ask",
          f"{c} removed={len(b.removed)}")
    check(c['zones_skipped'] == 2 and not is_in(sig_in1, b.removed),
          "an existing pour without replace_existing is kept and the new one "
          "skipped; an existing keep-out NAME skips the duplicate", f"{c}")
    added = [z for z in b.created if isinstance(z, Zone)]
    gnd_f = [z for z in added if z.type == ZoneType.ZT_COPPER
             and A.layer_name_for(z.layers[0]) == 'F.Cu']
    check(len(gnd_f) == 1 and not is_in(gnd_tear, b.removed),
          "a GND TEARDROP on F.Cu is not 'GND already pours on F.Cu' (#1164): "
          "the F.Cu pour is added and the teardrop left alone")
    ko = [z for z in added if z.type == ZoneType.ZT_RULE_AREA]
    check(len(ko) == 1 and ko[0].name == 'npth_slot_2'
          and ko[0].proto.rule_area_settings.keepout_copper
          and not ko[0].proto.rule_area_settings.keepout_tracks
          and sorted(A.layer_name_for(l) for l in ko[0].layers) == ['B.Cu', 'F.Cu'],
          "#448's NPTH-slot keep-out is a copper-pour keep-out RULE AREA on its "
          "layers (it used to be a net-less copper zone on F.Cu)")
    sig_b = [z for z in added if z.type == ZoneType.ZT_COPPER
             and z.net.name == 'SIG']
    from kipy.proto.board.board_types_pb2 import ZoneConnectionStyle as ZCS
    check(len(sig_b) == 1 and not is_in(sig_multi, b.removed)
          and sig_b[0].priority == 2,
          "a MULTI-layer SIG pour is neither replaced nor a duplicate: the new "
          "B.Cu pour goes beside it at the engine's priority, as route_planes "
          "pours it")
    conn = sig_b[0].connection if sig_b else None
    gnd_conn = gnd_f[0].connection if gnd_f else None
    check(conn is not None and conn.zone_connection == ZCS.ZCS_THERMAL
          and conn.thermal_spokes.gap == nm(0.2)
          and conn.thermal_spokes.width == nm(0.2)
          and gnd_conn is not None and gnd_conn.zone_connection == ZCS.ZCS_FULL,
          "pads connect solid unless the run asked for thermal relief, whose "
          "spokes are the CLI writer's 0.2 mm gap and width")
    check(c['zones'] == 5, "five zones added: two GND B.Cu polygons, GND F.Cu, "
          "SIG B.Cu, one keep-out", f"{c['zones']}")
    check(A.existing_zone_keys(Board(zones=[gnd_tear, rule])) == set(),
          "existing_zone_keys counts no teardrop and no rule area")
    from kipy.proto.board.board_types_pb2 import TeardropType
    td_copper = _zone(b, 'GND', ['B.Cu'])
    td_copper.proto.copper_settings.teardrop.type = TeardropType.TDT_VIA_PAD
    check(A.existing_zone_keys(Board(zones=[td_copper])) == set(),
          "nor a teardrop that comes as a COPPER zone with teardrop settings "
          "(the form kicad_parser's kipy read flags)")


def test_planes_multilayer_replace():
    print("C2. #1299: a multi-layer pour the run re-pours on EVERY layer it "
          "covers is replaced, once")
    b = Board(nets=('VCC', 'GND'))
    vcc_fb = _zone(b, 'VCC', ['F.Cu', 'B.Cu'])
    gnd_3 = _zone(b, 'GND', ['F.Cu', 'In1.Cu', 'B.Cu'])
    b.zones = [vcc_fb, gnd_3]
    sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
    new = [dict(net_name='VCC', layer='F.Cu', polygon_points=sq, replace_existing=True),
           dict(net_name='VCC', layer='B.Cu', polygon_points=sq, replace_existing=True),
           # GND re-poured on two of its three layers: the pour stays
           dict(net_name='GND', layer='F.Cu', polygon_points=sq, replace_existing=True),
           dict(net_name='GND', layer='B.Cu', polygon_points=sq, replace_existing=True)]
    c = A.apply_planes_results(b, pcb_data=SimpleNamespace(nets={}), new_zones=new)
    check(is_in(vcc_fb, b.removed) and sum(1 for z in b.removed if z is vcc_fb) == 1
          and c['zones_replaced'] == 1,
          "the VCC F.Cu+B.Cu pour, re-poured on both, is removed ONCE though it is "
          "listed under each of its layers", f"{c} removed={len(b.removed)}")
    check(not is_in(gnd_3, b.removed),
          "the GND pour re-poured on two of its three layers is kept (plane_io "
          "drops a pour only when every layer it covers is replaced)")
    check(c['zones'] == 4 and c['zones_skipped'] == 0,
          "all four new pours are added", f"{c}")


# ---------------------------------------------------------------------------
def _fp(ref, x, y, locked=False, uid=None):
    fp = FootprintInstance()
    fp.reference_field.text.value = ref
    fp.position = vec(x, y)
    fp.layer = BoardLayer.BL_F_Cu
    fp.locked = locked
    if uid:
        fp.proto.id.value = uid
    return fp


def test_footprint_moves():
    print("D. the footprint mover: #726 keys, #829 outline owners")
    tp_a, tp_b, u2 = _fp('TP4', 1, 1), _fp('TP4', 2, 2), _fp('U2', 3, 3)
    b = Board(footprints=[tp_a, tp_b, u2])
    pcb = SimpleNamespace(nets={}, footprints={
        'U2': SimpleNamespace(owns_board_outline=True)})
    applied = []
    n = A.apply_footprint_moves(
        b, [{'reference': 'TP4~2', 'new_x': 9, 'new_y': 9, 'new_rotation': 0},
            {'reference': 'U2', 'new_x': 5, 'new_y': 5, 'new_rotation': 0},
            {'reference': 'NOPE', 'new_x': 5, 'new_y': 5, 'new_rotation': 0}],
        pcb_data=pcb, applied_out=applied)
    check(A._vec_xy_mm(tp_b.position) == (9.0, 9.0)
          and A._vec_xy_mm(tp_a.position) == (1.0, 1.0),
          "TP4~2 moves the SECOND TP4 block, and the first stays")
    check(A._vec_xy_mm(u2.position) == (3.0, 3.0),
          "a part that draws the board outline is not moved (#829)")
    check(n == 1 and [p['reference'] for p in applied] == ['TP4~2'],
          "one move applied and reported", f"{n} {applied}")


# ---------------------------------------------------------------------------
RESULT = """(kicad_pcb (version 20241229) (generator "pcbnew")
 (general (thickness 1.6))
 (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal)
  (2 "B.Cu" signal) (25 "Edge.Cuts" user))
 (setup)
 (net 0 "") (net 1 "GND") (net 2 "SIG")
 (gr_rect (start -5 -5) (end 60 30) (stroke (width 0.1) (type solid))
  (fill none) (layer "Edge.Cuts"))
%(fps)s
 (segment (start 0 0) (end 5 0) (width 0.2) (layer "F.Cu") (net "SIG")
  (uuid "00000000-0000-0000-0000-00000000000a"))
 (segment (start 0 1) (end 6 1) (width 0.2) (layer "F.Cu") (net "SIG")
  (uuid "00000000-0000-0000-0000-00000000000b"))
 (arc (start 0 3) (mid 2.5 4) (end 5 3) (width 0.2) (layer "F.Cu") (net "SIG")
  (uuid "00000000-0000-0000-0000-00000000000c"))
 (segment (start 0 5) (end 5 5) (width 0.2) (layer "F.Cu") (net "GND")
  (locked yes) (uuid "00000000-0000-0000-0000-00000000000d"))
 (via (at 7 0) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "SIG")
  (tenting (front yes) (back yes))
  (uuid "00000000-0000-0000-0000-00000000000e"))
 (via blind (at 8 0) (size 0.4) (drill 0.2) (layers "F.Cu" "In1.Cu")
  (net "SIG") (uuid "00000000-0000-0000-0000-00000000000f"))
)
"""


def _fp_block(ref, x, y, locked):
    return (f' (footprint "L:R" {"(locked yes) " if locked else ""}'
            f'(layer "F.Cu") (at {x} {y})\n'
            f'  (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
            f'(effects (font (size 1 1) (thickness 0.15)))))')


def test_placement_apply():
    print("E. the Placement tab's Apply: locks (#1054), unchanged copper kept "
          "(#1279)")
    from kicad_routing_plugin.placement_gui import PlacementTab
    #           ref     live pose, live lock   result pose, result lock
    plan = {'U9':   ((10, 10), False, (10, 10), True),    # lock in place
            'C3':   ((20, 10), False, (22, 10), True),    # move AND lock
            'P1':   ((30, 10), True, (32, 10), False),    # unlock AND move
            'BUS1': ((40, 10), True, (40, 10), True)}     # control
    fps = '\n'.join(_fp_block(r, *p[2], p[3]) for r, p in plan.items())
    td = tempfile.mkdtemp(prefix='ipcapply_')
    path = os.path.join(td, 'result.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(RESULT % {'fps': fps})
    result = parse_kicad_pcb(path)
    import shutil
    shutil.rmtree(td, ignore_errors=True)

    live_fps = {r: _fp(r, *p[0], locked=p[1]) for r, p in plan.items()}
    b = Board(nets=('GND', 'SIG'), footprints=list(live_fps.values()))

    def track(x0, y0, x1, y1, net):
        t = Track()
        t.start, t.end, t.width = vec(x0, y0), vec(x1, y1), nm(0.2)
        t.layer, t.net = BoardLayer.BL_F_Cu, b.net(net)
        return t
    seg_a = track(0, 0, 5, 0, 'SIG')          # unchanged -> kept
    seg_b = track(0, 1, 5, 1, 'SIG')          # result lengthened it -> replaced
    seg_x = track(0, 7, 5, 7, 'SIG')          # not in the result -> removed
    arc = ArcTrack()
    arc.start, arc.mid, arc.end = vec(0, 3), vec(2.5, 4), vec(5, 3)
    arc.width, arc.layer, arc.net = nm(0.2), BoardLayer.BL_F_Cu, b.net('SIG')
    via_1 = A.make_via(A.NetMap(b), 7, 0, 0.6, 0.3, net_name='SIG')
    via_x = A.make_via(A.NetMap(b), 9, 0, 0.6, 0.3, net_name='SIG')
    b.tracks = [seg_a, seg_b, seg_x, arc]
    b.vias = [via_1, via_x]

    tab = SimpleNamespace(_live_by_key=PlacementTab._live_by_key,
                          _fp_is_back=PlacementTab._fp_is_back)
    moves = PlacementTab._pose_moves(
        SimpleNamespace(**vars(tab)), b, result)
    locks = PlacementTab._lock_changes(tab, b, result)
    check(sorted(r for r, _f, _p in moves) == ['C3', 'P1'],
          "the pose diff names the two moved parts", f"{[m[0] for m in moves]}")
    check(sorted((r, w) for r, _f, w in locks)
          == [('C3', True), ('P1', False), ('U9', True)],
          "the lock pass names U9 (lock, NO move -- invisible to a pose diff), "
          "C3 (lock) and P1 (unlock), and not the control",
          f"{[(r, w) for r, _f, w in locks]}")
    tracks_added, vias_added = PlacementTab._apply_ipc(
        tab, b, result, moves, True, locks)
    got = {r: kipy_locked(f) for r, f in live_fps.items()}
    check(got == {r: p[3] for r, p in plan.items()},
          "every live lock now equals the result's", f"{got}")
    check(A._vec_xy_mm(live_fps['C3'].position) == (22.0, 10.0)
          and A._vec_xy_mm(live_fps['U9'].position) == (10.0, 10.0),
          "C3 moved, U9 locked in place")
    check(all(is_in(live_fps[r], b.updated) for r in ('U9', 'C3', 'P1'))
          and not is_in(live_fps['BUS1'], b.updated)
          and sum(1 for x in b.updated if x is live_fps['C3']) == 1,
          "each changed footprint is updated once, the control not at all")

    check(not any(is_in(x, b.removed) for x in (seg_a, arc, via_1)),
          "the unchanged track, ARC and via are KEPT (same objects, so a "
          "sense: group keeps its members and the arc stays an arc)")
    check(all(is_in(x, b.removed) for x in (seg_b, seg_x, via_x))
          and len(b.removed) == 3,
          "the changed track and the copper the result lacks are removed",
          f"{len(b.removed)} removed")
    new_t = [x for x in b.created if isinstance(x, Track)]
    new_v = [x for x in b.created if isinstance(x, Via)]
    check(tracks_added == 2 and len(new_t) == 2 and vias_added == 1
          and len(new_v) == 1,
          "two tracks (the lengthened one, the new one) and one via added",
          f"{tracks_added}/{vias_added}")
    locked_new = [t for t in new_t if kipy_locked(t)]
    check(len(locked_new) == 1 and A._vec_xy_mm(locked_new[0].start)[1] == 5.0,
          "the result's (locked yes) track comes back LOCKED (#521)")
    check(new_v and new_v[0].type == ViaType.VT_BLIND
          and A.layer_name_for(new_v[0].padstack.drill.end_layer) == 'In1.Cu',
          "the result's blind via comes back blind on F.Cu-In1.Cu (#1268)")
    from kicad_parser import kipy_via_protection_live
    spec = kipy_via_protection_live(via_1) or {}
    check(is_in(via_1, b.updated) and 'tenting' in spec,
          "the kept via takes the result's protection spec (#489)", f"{spec}")


# ---------------------------------------------------------------------------
def test_pose_sync():
    print("F. the dialog's re-sync patches poses in place (#362)")
    from kicad_routing_plugin.routing_dialog import _sync_footprint_poses

    def fp(x, rot, pads):
        return SimpleNamespace(x=x, y=0.0, rotation=rot, layer='F.Cu',
                               locked=False, pads=pads)

    def pad(gx, sx):
        return SimpleNamespace(global_x=gx, global_y=0.0, hole_x=None,
                               hole_y=None, size_x=sx, size_y=1.0, rotation=0,
                               rect_rotation=0, polygons=None, layers=['F.Cu'])
    p1, p2 = pad(1.0, 0.5), pad(2.0, 0.5)
    old = SimpleNamespace(footprints={'C1': fp(1.0, 0, [p1, p2]),
                                      'R1': fp(5.0, 0, [pad(5.0, 0.5)])})
    new = SimpleNamespace(footprints={
        'C1': fp(11.0, 90, [pad(11.0, 1.0), pad(11.0, 1.0)]),
        'R1': fp(6.0, 0, [pad(6.0, 0.5), pad(7.0, 0.5)])})
    n = _sync_footprint_poses(old, new)
    c1 = old.footprints['C1']
    check(n == 2 and c1.x == 11.0 and c1.rotation == 90,
          "the footprint pose is the fresh build's")
    check(c1.pads[0] is p1 and p1.global_x == 11.0 and p1.size_x == 1.0,
          "pads patched IN PLACE (the objects pads_by_net shares), size swap "
          "included")
    check(old.footprints['R1'].pads[0].global_x == 5.0,
          "a footprint whose pad count changed keeps its pads (no mis-aligned "
          "patch)")

    # #726 on a board with a duplicated reference -- the three properties
    # tests/gui_parity/test_726_gui_sync.py pins for main's SWIG sync, on
    # esp_prog's two `Ref*` blocks (`Ref*` and `Ref*~2`).
    board = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    cached, fresh = parse_kicad_pcb(board), parse_kicad_pcb(board)
    twin = [k for k in cached.footprints if k.endswith('~2')]
    if not check(len(twin) == 1, "precondition: esp_prog carries one "
                 "duplicated reference", f"{twin}"):
        return
    twin = twin[0]
    first = twin.split('~')[0]

    def pose(pcb, k):
        f = pcb.footprints[k]
        return (f.x, f.y, f.rotation,
                tuple((p.global_x, p.global_y) for p in f.pads))
    before = {k: pose(cached, k) for k in cached.footprints}
    _sync_footprint_poses(cached, fresh)
    check(all(pose(cached, k) == before[k] for k in cached.footprints),
          "1. a no-op sync is a true no-op on a duplicate-carrying board")
    fp2 = fresh.footprints[twin]
    fp2.x += 1.0
    for p in fp2.pads:
        p.global_x += 1.0
    _sync_footprint_poses(cached, fresh)
    check(pose(cached, twin)[0] == before[twin][0] + 1.0
          and pose(cached, first) == before[first],
          f"2. a move of {twin} reaches {twin}, and {first} stays put")
    check(all(abs(a[0] - b[0] - 1.0) < 1e-9 for a, b in
              zip(pose(cached, twin)[3], before[twin][3]))
          and pose(cached, first)[3] == before[first][3],
          "3. the pads follow the footprint they belong to")


def _track(board, x0, y0, x1, y1, w, layer, net):
    t = Track()
    t.start, t.end = vec(x0, y0), vec(x1, y1)
    t.width = nm(w)
    t.layer = layer
    t.net = board.net(net)
    return t


def test_live_strip_counted():
    print("G. #1556: the adapter's strips are counted and width-preferring")
    from kicad_parser import Segment, Via as PVia
    F, B = BoardLayer.BL_F_Cu, BoardLayer.BL_B_Cu
    pcb = SimpleNamespace(nets={1: SimpleNamespace(name='N')})

    def seg(w, layer='F.Cu', x1=12.0):
        return Segment(start_x=10.0, start_y=10.0, end_x=x1, end_y=10.0,
                       width=w, layer=layer, net_id=1)

    b = Board(nets=('N',))
    thin = _track(b, 10, 10, 12, 10, 0.0762, F, 'N')
    wide = _track(b, 12, 10, 10, 10, 0.1, F, 'N')
    back = _track(b, 10, 10, 12, 10, 0.1, B, 'N')
    b.tracks = [thin, wide, back]
    c = A.apply_routing_results(b, {'segments_to_remove': [seg(0.1)]},
                                pcb_data=pcb)
    check(c['removed'] == 1 and is_in(wide, b.removed),
          "a same-span stack loses the LISTED member (0.1), not the first",
          f"{c} removed={len(b.removed)}")
    check(not is_in(thin, b.removed) and not is_in(back, b.removed),
          "the 0.0762 keeper and the same span on B.Cu stay")

    b = Board(nets=('N',))
    t1 = _track(b, 10, 10, 12, 10, 0.1, F, 'N')
    t2 = _track(b, 10, 10, 12, 10, 0.1, F, 'N')
    b.tracks = [t1, t2]
    c = A.apply_routing_results(b, {'segments_to_remove': [seg(0.1), seg(0.1, x1=13)]},
                                pcb_data=pcb)
    check(c['removed'] == 1 and c['segments_unmatched'] == 1,
          "counted: one listing of a two-member stack removes one; a listing "
          "with no member removes nothing and is reported", f"{c}")

    b = Board(nets=('N',))
    keep = _track(b, 10, 10, 12, 10, 0.0762, F, 'N')
    gone = _track(b, 10, 10, 12, 10, 0.1, F, 'N')
    v = Via()
    v.position = vec(20, 20)
    v.net = b.net('N')
    b.tracks, b.vias = [keep, gone], [v]
    c = A.apply_planes_results(
        b, pcb_data=pcb,
        strip_segments=[seg(0.1), PVia(x=20.0, y=20.0, size=0.6, drill=0.3,
                                       layers=['F.Cu', 'B.Cu'], net_id=1)])
    check(c.get('stripped') == 2 and is_in(gone, b.removed) and is_in(v, b.removed)
          and not is_in(keep, b.removed),
          "the Planes apply deletes the finalize's strips (segment by width, "
          "via by position), which it used to ignore", f"{c}")

    src = open(os.path.join(ROOT, 'kicad_ipc_adapter.py'), encoding='utf-8').read()
    body = src[src.index('def strip_listed_tracks'):]
    check(body.count('commit.remove(t)') == 0,
          "no adapter strip matches tracks by a set of endpoint keys any more")


def test_writeback_takes_shared_keywords():
    print("H. the project writeback takes the shared deciders' keywords")
    import inspect
    from fix_kicad_drc_settings import fanout_fix_kwargs, plane_fix_kwargs
    params = set(inspect.signature(A.write_drc_settings_to_project).parameters)
    fan = fanout_fix_kwargs('bga', [{}], [{}], clearance=0.15, track_width=0.15,
                            via_size=0.3, via_drill=0.2, clearance_ceiling=0.15,
                            hole_clearance=0.2)
    pln = plane_fix_kwargs([{}], [{}], clearance=0.15, hole_to_hole=0.25,
                           track_width=0.15, via_size=0.3, via_drill=0.2)
    # What the tabs add on top: the Planes tab's edge / thermal / class keys.
    extra = {'edge_clearance', 'keep_thermal', 'clamp_nondefault_netclasses',
             'class_ceiling', 'hole_clearance'}
    missing = (set(fan) | set(pln) | extra) - params
    check(not missing, "write_drc_settings_to_project accepts every keyword",
          f"missing {sorted(missing)}")


def main():
    for t in (test_via_types, test_set_item_locked, test_planes_apply,
              test_planes_multilayer_replace, test_footprint_moves,
              test_placement_apply, test_pose_sync, test_live_strip_counted,
              test_writeback_takes_shared_keywords):
        t()
    if FAILS:
        print(f"\n{len(FAILS)} check(s) FAILED")
        return 1
    print("\nPASS  main's live-board fixes reach KiCad through the IPC adapter")
    return 0


if __name__ == '__main__':
    sys.exit(main())
