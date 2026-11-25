#!/usr/bin/env python3
"""The bus step's tie pass and its packs (awx/field_ties.py, route_bus._supply_ties, route_bus.plan_boxed).

  python3 tests/test_622_supply_ties_packs.py

After a group's lanes are laid the step ties, by copper, the supply pads beneath its parts that only a pour joins --
and finishes its PACKS: the other parts a lane's net has a pad on (a termination pack, a pull-up). A pack's rail pads
and its own stubs are laid in one pass, planned together. A tie that walls a pad in, or leaves a net no way in the
route step's own plan, is taken out by itself; the rest of the pass stands.

1. clear_of: of the copper a pass kept, the pieces with copper near a point go and the others stay; a rail run pad to
   pad is a piece a SPAN (`cut`), so the span beside the point goes and the rail's other spans stay;
2. without_outer_pours: a net poured on an outer layer alone keeps its pour -- unless it is named (a pack's rail);
   a net poured on an inner layer too loses its outer pour either way; an inner pour is never taken;
3. _free_ends: a net's pads no copper joins and its tracks' ends that stand on nothing of the net; not an end on a
   pad, on a via or on another track;
4. lay_after's pads: a pack's pad on a poured net that no copper joins (its rail side: that net's outer pour is not
   read), a pack's pad on a lane's net with no copper yet (its stub: the pass plans its nets together); not a pad
   copper already joins, not a pad of a part that is no pack;
5. the step's router buses are opt-in (BUS_ROUTER=1);
6. plan_boxed: on a board with one net's pad walled round by other pads and another net free, the route step's plan
   has no way for the first and the check names it alone;
7. whole_ends.own_parts -- the parts whose pads are no copper in front of a lane's end, the lane's own: the
   destination from a tooth and the source from a berth where the bus ends on a small part; none where it ends on an
   array of pads, whose pads stand in front of a lane as any copper does.
"""
import os
import sys
import tempfile
import types
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import field_ties as ft  # noqa: E402
import route_bus as rb  # noqa: E402
from kicad_parser import BoardInfo  # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg, make_via  # noqa: E402

fails = []


def check(cond, msg):
    print(('ok   ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


def T(x1, y1, x2, y2, net=1, layer='B.Cu'):
    return {'start': (x1, y1), 'end': (x2, y2), 'width': 0.1, 'layer': layer, 'net_id': net}


def V(x, y, net=1):
    return {'x': x, 'y': y, 'size': 0.25, 'drill': 0.15, 'layers': ['F.Cu', 'B.Cu'], 'net_id': net}


# ---- 1. clear_of
a = [T(0, 0, 1, 0), T(1, 0, 1, 1)]                 # one piece: two tracks sharing an end, a via at its far end
va = [V(1, 1)]
b = [T(5, 0, 6, 0)]                                # another piece of the same net, far off
c = [T(1, 1, 2, 1, net=2)]                         # another net's track from the same point: its own piece
tr, vs = ft.clear_of(a + b + c, va, [(0.2, 0.3)], 0.5)
check(tr == b + c and vs == [], '1. the piece with copper near the point goes whole, its via with it; a far piece and '
      'another net\'s track from the same point stay')
tr, vs = ft.clear_of(a + b + c, va, [(9, 9)], 0.5)
check(tr == a + b + c and vs == va, '1. no point near: everything stays')
rail = [T(0, 0, 0, 1), T(0, 1, 0, 2), T(0, 2, 0, 3)]         # a rail run through pads at (0, 1) and (0, 2)
at_pad = lambda nid, x, y: nid == 1 and abs(x) < 1e-6 and y in (1, 2)       # noqa: E731
tr, _vs = ft.clear_of(rail, [], [(0.3, 0.4)], 0.5)
check(tr == [], '1. a rail run pad to pad is one piece as drawn: a point beside one span takes it all')
tr, _vs = ft.clear_of(rail, [], [(0.3, 0.4)], 0.5, cut=at_pad)
check(tr == rail[1:], '1. ...and a piece a span with its pads named: the span beside the point goes, the others stay')

# ---- 2. without_outer_pours
ZONE = '\t(zone\n\t\t(net {i})\n\t\t(net_name "{n}")\n\t\t(layer "{l}")\n\t\t(polygon (pts (xy 0 0) (xy 1 0) (xy 1 1)))\n\t)'
zones = [(1, 'RAIL', 'B.Cu'), (2, 'GND', 'F.Cu'), (2, 'GND', 'In1.Cu'), (3, 'VIO', 'F.Cu')]


def text():
    return '(kicad_pcb\n\t(version 20241229)\n' + '\n'.join(ZONE.format(i=i, n=n, l=l) for i, n, l in zones) + '\n)\n'


def left(also=()):
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(text())
        n = ft.without_outer_pours(p, ['F.Cu', 'B.Cu'], also)
        return n, sorted((nm, sorted(ls)[0]) for _a, _b, nm, ls in ft._zone_blocks(open(p, encoding='utf-8').read()))


n, z = left()
check(n == 1 and z == [('GND', 'In1.Cu'), ('RAIL', 'B.Cu'), ('VIO', 'F.Cu')],
      '2. a net poured on an inner layer too loses its outer pour; one poured on an outer layer alone keeps it')
n, z = left({'RAIL'})
check(n == 2 and z == [('GND', 'In1.Cu'), ('VIO', 'F.Cu')],
      '2. ...unless it is named: its outer pour goes too, the other outer-only net\'s stays')

# ---- 3. _free_ends
p1 = make_pad(1, 0.0, 0.0, ref='U1', num='1', net_name='S', size_x=0.3, size_y=0.3)       # a track ends on it
p2 = make_pad(1, 5.0, 0.0, ref='R1', num='1', net_name='S', size_x=0.3, size_y=0.3)       # no copper joins it
fps = {'U1': types.SimpleNamespace(reference='U1', pads=[p1]), 'R1': types.SimpleNamespace(reference='R1', pads=[p2])}
segs = [make_seg(0.0, 0.0, 1.0, 0.0, net_id=1), make_seg(1.0, 0.0, 2.0, 0.0, net_id=1),   # ...on to a via
        make_seg(2.0, 0.0, 2.0, 1.5, net_id=1, layer='B.Cu')]                             # ...and a run ending free
pcb = make_pcb(nets={1: make_net(1, 'S', pads=[p1, p2])}, footprints=fps, segments=segs,
               vias=[make_via(2.0, 0.0, net_id=1)], zones=[])
got = sorted(rb._free_ends(pcb, 1))
check(got == [(2.0, 1.5), (5.0, 0.0)], f'3. the pad no copper joins and the run\'s free end; not the end on a pad, on '
      f'a via or on another track -- {got}')

# ---- 4. lay_after's pads
rail_net, lane_net, other = 1, 2, 3
pk = [make_pad(rail_net, 10.0, 0.0, ref='RN1', num='1', net_name='RAIL', layers=('B.Cu',)),
      make_pad(rail_net, 10.0, 0.5, ref='RN1', num='2', net_name='RAIL', layers=('B.Cu',)),     # a track ends on it
      make_pad(lane_net, 11.0, 0.0, ref='RN1', num='8', net_name='LANE', layers=('B.Cu',)),
      make_pad(other, 11.0, 0.5, ref='RN1', num='7', net_name='OTHER', layers=('B.Cu',))]
loose = [make_pad(rail_net, 20.0, 0.0, ref='C9', num='1', net_name='RAIL', layers=('B.Cu',))]   # no pack's pad
board = make_pcb(nets={rail_net: make_net(rail_net, 'RAIL'), lane_net: make_net(lane_net, 'LANE'),
                       other: make_net(other, 'OTHER')},
                 footprints={'RN1': types.SimpleNamespace(reference='RN1', pads=pk),
                             'C9': types.SimpleNamespace(reference='C9', pads=loose)},
                 segments=[make_seg(10.0, 0.5, 9.0, 0.5, net_id=rail_net, layer='B.Cu')], vias=[],
                 zones=[types.SimpleNamespace(net_id=rail_net, layer='B.Cu')],
                 board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'In1.Cu', 'B.Cu'],
                                      board_bounds=(0.0, 0.0, 30.0, 10.0)))
seen = {}


def fake_pass(field, nets, pads, wayouts, **kw):
    seen.update(nets=set(nets), pads=sorted(f'{p.component_ref}.{p.pad_number}' for p in pads), kw=kw)
    return {'tied': [], 'segments': 0, 'vias': 0, 'exit': 0, 'tracks': [], 'vs': [], 'strayed': 0}


_load, _pass = ft._load, ft._pass
ft._load, ft._pass = (lambda _path: board), fake_pass
try:
    ft.lay_after('board', [], packs=['RN1'], lanes=['LANE'])
    check(seen.get('pads') == ['RN1.1', 'RN1.8'] and seen.get('nets') == {'RAIL', 'LANE'},
          f'4. the pack\'s rail pad no copper joins and its stub pad on a lane\'s net; not the rail pad a track ends '
          f'on, a pad of another net, or a pad of a part that is no pack -- {seen.get("pads")}')
    check(seen.get('kw', {}).get('bare_nets') == {'RAIL'} and seen['kw'].get('together') is True
          and seen['kw'].get('bare_outer') is True,
          '4. ...the rail\'s outer pour not read, the pass\'s nets planned together')
    seen.clear()
    ft.lay_after('board', [], packs=['RN1'])
    check(seen.get('pads') == ['RN1.1'] and seen['kw'].get('together') is False,
          '4. with no lane named the stub is no pad of the pass, and its nets are not planned together')
    seen.clear()
    check(ft.lay_after('board', []) is None and not seen, '4. with no pack there is no pad and no pass')
finally:
    ft._load, ft._pass = _load, _pass

# ---- 5. the router buses are opt-in
import awx_settings  # noqa: E402
with awx_settings.given({k: v for k, v in awx_settings.environ().items() if k != 'BUS_ROUTER'}):
    off = rb._router_on()
with awx_settings.given(dict(awx_settings.environ(), BUS_ROUTER='1')):
    on = rb._router_on()
check(off is False and on is True, '5. the router buses the step finds are routed by it only with BUS_ROUTER=1')

# ---- 6. plan_boxed
U = lambda: str(uuid.uuid4())  # noqa: E731


def write_board(path, pads, w=20, h=30):
    """a board: `pads` [(ref, x, y, net name)] (through-hole, 0.8 mm) and the outline (0, 0)-(w, h)"""
    nid = {n_: i + 1 for i, n_ in enumerate(sorted({p[3] for p in pads}))}
    L = ['(kicad_pcb', '\t(version 20241229)', '\t(generator "pcbnew")', '\t(generator_version "9.0")',
         '\t(general (thickness 1.6) (legacy_teardrops no))', '\t(paper "A4")',
         '\t(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (5 "F.SilkS" user) (1 "F.Mask" user) (3 "B.Mask" user)'
         ' (25 "Edge.Cuts" user) (35 "F.Fab" user))',
         '\t(setup (pad_to_mask_clearance 0))', '\t(net 0 "")']
    L += [f'\t(net {i} "{n_}")' for n_, i in nid.items()]
    for ref, x, y, net in pads:
        L += [f'\t(footprint "synth:P" (layer "F.Cu") (uuid "{U()}") (at {x} {y})',
              f'\t\t(property "Reference" "{ref}" (at 0 -1.2 0) (layer "F.SilkS") (uuid "{U()}") (effects (font (size 0.5 0.5) (thickness 0.08))))',
              f'\t\t(pad "1" thru_hole circle (at 0 0) (size 0.8 0.8) (drill 0.4) (layers "*.Cu" "*.Mask") (net {nid[net]} "{net}") (uuid "{U()}"))',
              '\t)']
    for (x1, y1), (x2, y2) in (((0, 0), (w, 0)), ((w, 0), (w, h)), ((w, h), (0, h)), ((0, h), (0, 0))):
        L.append(f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.1) (type default)) (layer "Edge.Cuts") (uuid "{U()}"))')
    L.append(')')
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')


ring = [(round(10 + 0.9 * i, 2), round(10 + 0.9 * j, 2)) for i in range(-2, 3) for j in range(-2, 3)
        if max(abs(i), abs(j)) == 2]               # sixteen pads round (10, 10), a tenth of a millimetre between them
pads = [(f'W{k}', x, y, 'WALL') for k, (x, y) in enumerate(ring)] \
    + [('A1', 10, 10, 'A'), ('A2', 10, 20, 'A'), ('B1', 4, 4, 'B'), ('B2', 16, 4, 'B')]
import rules  # noqa: E402
_active = rules.active
rules.active = lambda: types.SimpleNamespace(clearance=0.15, track=0.15, via_size=0.5, via_drill=0.3)
try:
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'in.kicad_pcb')
        write_board(b, pads)
        got = rb.plan_boxed(b)
        check(got == {'A'}, f'6. the net whose pad the other pads wall round has no way in the route step\'s plan, '
              f'the free net has one -- {got}')
        check(sorted(os.listdir(td)) == ['in.kicad_pcb'], '6. ...and the check leaves nothing beside the board')
finally:
    rules.active = _active

# ---- 7. own_parts
import whole_ends as we  # noqa: E402


def part(ref, pts):
    return types.SimpleNamespace(reference=ref, layer='F.Cu', x=0.0, y=0.0, rotation=0.0, footprint_name='synth:P',
                                 pads=[make_pad(k + 1, x, y, ref=ref, num=str(k + 1), net_name=f'N{k}',
                                                size_x=0.3, size_y=0.3) for k, (x, y) in enumerate(pts)])


fps7 = {'S': part('S', [(0.5 * i, 0.5 * j) for i in range(5) for j in range(5)]),
        'A': part('A', [(10 + 0.8 * i, 0.8 * j) for i in range(4) for j in range(4)]),
        'R': part('R', [(20.0, 0.0), (21.0, 0.0)]),
        'H': part('H', [(30.0 + 2.54 * i, 0.0) for i in range(8)])}
board7 = make_pcb(footprints=fps7, zones=[])
st_small = {'sref': 'S', 'dref': 'R', 'pcb': board7}
st_row = {'sref': 'S', 'dref': 'H', 'pcb': board7}
st_array = {'sref': 'S', 'dref': 'A', 'pcb': board7}
st_both = {'sref': 'S', 'dref': 'R', 'dests': ['R', 'A'], 'pcb': board7}
check(we.own_parts(st_small, 0) == {'R'} and we.own_parts(st_small, 1) == {'S'}
      and we.own_parts(st_row, 0) == {'H'} and we.own_parts(st_row, 1) == {'S'},
      '7. a bus to a part of two pads, or to a row of pins: the destination from a tooth, the source from a berth')
check(we.own_parts(st_array, 0) == frozenset() and we.own_parts(st_array, 1) == frozenset()
      and we.own_parts(st_both, 0) == frozenset(),
      '7. a bus to an array of pads -- or to a group with one in it -- names none')

print(f'{"FAILED: " + str(len(fails)) if fails else "all checks passed"}')
sys.exit(1 if fails else 0)
