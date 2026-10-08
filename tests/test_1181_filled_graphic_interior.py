#!/usr/bin/env python3
"""#1181: a FILLED copper graphic is copper inside too -- an obstacle there, and
graded there.

A closed filled shape was parsed into its perimeter segments and nothing else,
so its interior blocked nothing and graded nothing. The router put a 0.6 mm
+3V3 via 0.26 mm inside One-Air-Max USB1's filled shield rect, and check_drc
graded it clean while KiCad reported `shorting_items`. A second leak let the
via launder itself: a track or via that merely TOUCHED a graphic granted it
its net (`graphic_effective_nets`' mutable arm), so the short was waived as
"same net" -- KiCad gives a footprint's copper no net at all.

KiCad 10.0.6 on esp_prog with probe copper in U2's filled SOT-89 tab reports a
violation for each of: a /+3.3V via at the tab centre (`clearance`), one
straddling its east edge (`shorting_items`), a /+3.3V track inside it
(`clearance`), and a via of pad 2's own net inside it (`shorting_items`, which
this repo accepts as a #995 own-copper row).

Checks:
  1. The parse: the tab's segments share one `graphic_ring`; a stroked
     (unfilled) graphic carries none.
  2. The obstacle map: the tab interior blocks a /+3.3V track and via, before
     and after prepare; pad 2's own net keeps its pad and the interior (#908's
     lift at region scale).
  3. A shape whose own pads give it TWO untied nets is lifted for neither.
  4. check_drc counts the three foreign probes, publishes the own-net via as a
     #995 row, and leaves the base board as it was.
  5. Board-level art: a via INSIDE a filled shape grants it nothing; one
     touching it from outside still does (#337). A footprint's copper takes
     no net from touching copper unless the part declares a net tie.

    python3 tests/test_1181_filled_graphic_interior.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb, Segment, Pad, Net, PCBData, Footprint, Via  # noqa: E402
from check_drc import (run_drc, filled_graphic_shapes, filled_graphic_lift_nets,  # noqa: E402
                       graphic_own_pad_nets, graphic_effective_nets)
from kicad_writer import generate_via_sexpr, generate_segment_sexpr          # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
TAB_CENTRE = (135.59, 96.69)
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


pcb = quiet(parse_kicad_pcb, BOARD)
n2i = {n.name: i for i, n in pcb.nets.items()}

print("1. the parse")
tab = [s for s in pcb.segments if getattr(s, 'graphic', False) and s.owner_ref == 'U2']
rings = {id(s.graphic_ring) for s in tab}
check("U2's tab: 8 segments, one shared ring",
      len(tab) == 8 and len(rings) == 1 and tab[0].graphic_ring is not None
      and len(tab[0].graphic_ring) == 8, f"{len(tab)} segs, {len(rings)} ring(s)")
shapes = [sh for sh in filled_graphic_shapes(pcb) if sh.owner_ref == 'U2']
check("one FilledGraphic for it, containing the tab centre",
      len(shapes) == 1 and shapes[0].contains(*TAB_CENTRE)
      and not shapes[0].contains(138.0, 96.69))
line = Segment(start_x=0, start_y=0, end_x=1, end_y=0, width=0.2, layer='F.Cu',
               net_id=0, graphic=True, graphic_kind='line')
check("a stroked line carries no ring", line.graphic_ring is None)

print("2. the obstacle map")
from routing_config import GridRouteConfig                               # noqa: E402
from obstacle_map import build_base_obstacle_map, GridCoord, build_layer_map  # noqa: E402
from routing_context import prepare_obstacles_inplace                    # noqa: E402
cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.2, via_size=0.6,
                      via_drill=0.3, grid_step=0.05)
coord = GridCoord(cfg.grid_step)


def blocked(net, at, nets=None, prepare=True):
    nets = nets or [n2i[net]]
    obs = quiet(build_base_obstacle_map, pcb, cfg, nets)
    if prepare:
        quiet(prepare_obstacles_inplace, obs, pcb, cfg, n2i[net], nets, [], {},
              build_layer_map(cfg.layers), {})
    gx, gy = coord.to_grid(*at)
    return obs.is_via_blocked(gx, gy), obs.is_blocked(gx, gy, 0)


check("tab interior blocks a /+3.3V via and track (base)",
      blocked('/+3.3V', TAB_CENTRE, prepare=False) == (True, True))
check("... and after prepare for /+3.3V",
      blocked('/+3.3V', TAB_CENTRE) == (True, True))
batch = [n2i['/+3.3V'], n2i['Net-(C1-Pad1)'], n2i['GND']]
check("... and for GND in a batch with pad 2's net",
      blocked('GND', TAB_CENTRE, nets=batch) == (True, True))
check("pad 2's own net keeps the interior (one own-pad net: lifted)",
      blocked('Net-(C1-Pad1)', TAB_CENTRE, nets=batch) == (False, False))
pad2 = [p for p in pcb.footprints['U2'].pads if p.pad_number == '2'][0]
check("pad 2's own net reaches its pad",
      blocked('Net-(C1-Pad1)', (pad2.global_x, pad2.global_y), nets=batch)[1] is False)

print("3. a shape between two untied nets is lifted for neither")
p1 = Pad(pad_number='1', net_id=1, net_name='/A', global_x=0.0, global_y=0.0,
         local_x=0, local_y=0, size_x=0.6, size_y=0.6, shape='rect',
         layers=['F.Cu'], drill=0, pad_type='smd', component_ref='U1')
p2 = Pad(pad_number='2', net_id=2, net_name='/B', global_x=4.0, global_y=0.0,
         local_x=4, local_y=0, size_x=0.6, size_y=0.6, shape='rect',
         layers=['F.Cu'], drill=0, pad_type='smd', component_ref='U1')
ring = ((0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (0.0, 1.0))   # corners on the pad centres
segs = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.1,
                layer='F.Cu', net_id=0, graphic=True, owner_ref='U1',
                graphic_kind='poly', graphic_filled=True, graphic_ring=ring)
        for a, b in zip(ring, ring[1:] + ring[:1])]
fp = Footprint(reference='U1', footprint_name='L:X', x=0, y=0, rotation=0,
               layer='F.Cu', pads=[p1, p2])
syn = PCBData(board_info=None, nets={1: Net(1, '/A'), 2: Net(2, '/B')},
              footprints={'U1': fp}, vias=[], segments=segs,
              pads_by_net={1: [p1], 2: [p2]})
own = graphic_own_pad_nets(syn)
touched = set().union(*own.values()) if own else set()
sh = filled_graphic_shapes(syn)[0]
check("both nets lift some perimeter edge", touched == {1, 2}, str(touched))
check("the interior is lifted for neither",
      filled_graphic_lift_nets(sh, own, syn.footprints) == frozenset())
fp.net_tie_groups = [['1', '2']]
check("... unless a declared tie bridges them",
      filled_graphic_lift_nets(sh, graphic_own_pad_nets(syn), syn.footprints) == frozenset({1, 2}))

print("4. check_drc on esp_prog probes")
CASES = {
    'via33_center': (('via', 135.59, 96.69, '/+3.3V'), 'via-segment'),
    'via33_straddle': (('via', 136.64, 96.69, '/+3.3V'), 'via-segment'),
    'trk33_inside': (('seg', 134.9, 96.69, 136.3, 96.69, '/+3.3V'), 'segment-segment'),
    'viaC1_center': (('via', 135.59, 96.69, 'Net-(C1-Pad1)'), None),
}
tmp = tempfile.mkdtemp(prefix='t1181_')
try:
    base_rows = quiet(run_drc, BOARD, quiet=True, print_summary=False)
    base_counted = [v for v in base_rows if not v.get('accepted')]
    base_own = [v for v in base_rows if v.get('accepted') == 'footprint-own-copper']
    for name, (it, want) in CASES.items():
        out = os.path.join(tmp, name + '.kicad_pcb')
        subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'copy_board.py'),
                        BOARD, out], check=True, capture_output=True)
        if it[0] == 'via':
            sx = generate_via_sexpr(it[1], it[2], 0.6, 0.3, ['F.Cu', 'B.Cu'], 0, net_name=it[3])
        else:
            sx = generate_segment_sexpr((it[1], it[2]), (it[3], it[4]), 0.25, 'F.Cu', 0,
                                        net_name=it[5])
        txt = open(out, encoding='utf-8').read()
        i = txt.rstrip().rfind(')')
        open(out, 'w', encoding='utf-8').write(txt[:i] + sx + '\n)\n')
        rows = quiet(run_drc, out, quiet=True, print_summary=False)
        counted = [v for v in rows if not v.get('accepted') and v['type'] != 'via-in-paste']
        new = counted[len(base_counted):] if len(counted) > len(base_counted) else []
        if want:
            check(f"{name}: one counted {want} against Polygon(U2)",
                  len(counted) == len(base_counted) + 1 and len(new) == 1
                  and new[0]['type'] == want and new[0].get('item2') == 'Polygon(U2)',
                  str([(v['type'], v['net1'], v['net2']) for v in counted]))
        else:
            own_rows = [v for v in rows if v.get('accepted') == 'footprint-own-copper']
            check(f"{name}: not counted, published as a #995 own-copper row",
                  len(counted) == len(base_counted) and len(own_rows) == len(base_own) + 1,
                  f"{len(counted)} counted, {len(own_rows)} own rows")
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print("5. board-level art: inside is the short, touching is a joint")
art_ring = ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0))
art = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.1,
               layer='F.Cu', net_id=5, graphic=True, graphic_kind='rect',
               graphic_filled=True, graphic_ring=art_ring)
       for a, b in zip(art_ring, art_ring[1:] + art_ring[:1])]
inside = Via(x=0.2, y=1.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=6)
outside = Via(x=-0.25, y=1.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=7)
b = PCBData(board_info=None, nets={5: Net(5, 'S'), 6: Net(6, 'I'), 7: Net(7, 'O')},
            footprints={}, vias=[inside, outside], segments=art, pads_by_net={})
eff = graphic_effective_nets(b)[id(art[0])]
check("a via inside the filled art does not give it its net", 6 not in eff, str(eff))
check("a via touching it from outside still does (#337)", 7 in eff, str(eff))
# A footprint's copper takes no net from a touching track -- unless the part
# declares a net tie, whose copper exists to short nets (KiCad reports no
# contact with it: kintex's NT* bridges, cheapmesh's tied AE1 antenna).
tie_pad = Pad(pad_number='1', net_id=8, net_name='T', global_x=10.0, global_y=0.0,
                     local_x=0, local_y=0, size_x=0.5, size_y=0.5, shape='rect',
                     layers=['F.Cu'], drill=0, pad_type='smd', component_ref='NT1')
bar = Segment(start_x=10.0, start_y=0.0, end_x=11.0, end_y=0.0, width=0.2,
              layer='F.Cu', net_id=0, graphic=True, owner_ref='NT1')
trk = Segment(start_x=11.0, start_y=0.0, end_x=12.0, end_y=0.0, width=0.2,
              layer='F.Cu', net_id=9)
nt = Footprint(reference='NT1', footprint_name='L:NetTie', x=10, y=0, rotation=0,
               layer='F.Cu', pads=[tie_pad])
tb = PCBData(board_info=None, nets={8: Net(8, 'T'), 9: Net(9, 'U')},
             footprints={'NT1': nt}, vias=[], segments=[bar, trk],
             pads_by_net={8: [tie_pad]})
check("an untied part's copper takes no net from a touching track",
      9 not in graphic_effective_nets(tb)[id(bar)])
nt.net_tie_groups = [['1']]
check("a net-tie part's copper does", 9 in graphic_effective_nets(tb)[id(bar)])

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")
