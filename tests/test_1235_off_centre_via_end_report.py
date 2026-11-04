#!/usr/bin/env python3
"""#1235: check_weird reports a track that ends on a via off its centre.

A track end inside a same-net via's copper but off its centre is connected,
so DRC and connectivity pass it, and nothing reported it. KiCad's length
measure reads such an end differently from ours (before 10.0.4 a via joins
only the tracks ending at its centre; from 10.0.4 the run is cut at the via
pad and measured to the centre), which is how a pair matched to 0.0006 mm
shipped 1.58 mm of skew (#1583 part B).

Rows, on synthetic boards:
  1. the definition (connectivity.off_centre_via_ends): an end inside the
     via's copper and off its centre is found with its offset; an end on the
     centre, an end past the via's radius, a foreign-net end and an end on a
     layer the via does not carry are not;
  2. check_weird reports it as advisory: not in the headline or the exit
     code (blocking_findings), named with its offset in the report;
  3. an end that is also a same-net pad's terminal names the pad.

    python3 tests/test_1235_off_centre_via_end_report.py
"""
import io
import os
import sys
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import (BoardInfo, Net, Pad, PCBData, Segment,   # noqa: E402
                          StackupLayer, Via)
from connectivity import off_centre_via_ends                        # noqa: E402
from check_weird import (check_weird, print_report,                 # noqa: E402
                         blocking_findings, ADVISORY_CATEGORIES)

NET, FOREIGN = 1, 2
LAYERS = ['F.Cu', 'In1.Cu', 'B.Cu']
failures = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  [{detail}]" if detail and not ok else ''))
    if not ok:
        failures.append(name)


def seg(x1, y1, x2, y2, layer='F.Cu', net=NET, w=0.2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def via(x, y, net=NET, layers=('F.Cu', 'B.Cu')):
    return Via(x=x, y=y, size=0.6, drill=0.3, layers=list(layers), net_id=net)


def pad(x, y, ref, num='1', net=NET, layer='F.Cu', size=0.6):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=size, size_y=size, shape='rect',
               layers=[layer], net_id=net, net_name='/SIG' if net == NET else '/OTHER',
               rotation=0.0, drill=0.0)


def board(segments, vias, pads):
    stack = [StackupLayer(name='F.Cu', layer_type='copper', thickness=0.035),
             StackupLayer(name='core', layer_type='core', thickness=0.7),
             StackupLayer(name='In1.Cu', layer_type='copper', thickness=0.035),
             StackupLayer(name='core', layer_type='core', thickness=0.7),
             StackupLayer(name='B.Cu', layer_type='copper', thickness=0.035)]
    nets = {NET: Net(net_id=NET, name='/SIG', pads=[p for p in pads if p.net_id == NET]),
            FOREIGN: Net(net_id=FOREIGN, name='/OTHER',
                         pads=[p for p in pads if p.net_id == FOREIGN])}
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    return PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 1: 'In1.Cu', 31: 'B.Cu'},
                             copper_layers=LAYERS, stackup=stack),
        nets=nets, footprints={}, vias=list(vias), segments=list(segments),
        pads_by_net=by_net, zones=[])


def ends(pcb):
    return off_centre_via_ends(pcb.segments, pcb.vias, LAYERS)


def advisory(pcb):
    return [f for f in check_weird(pcb, tolerance=0)[0]
            if f['category'] == 'off-centre-via-end']


# Pad A (F.Cu) at x=0 and pad B (B.Cu) at x=10; a via at (5, 0) carries the
# layer change. F.Cu runs A -> via, B.Cu runs via -> B.
A, B = pad(0, 0, 'U1'), pad(10, 0, 'U2', layer='B.Cu')
B_CU = seg(5, 0, 10, 0, layer='B.Cu')

print("1. the definition")
off = seg(0, 0, 5.1, 0.1)                 # ends 0.141 mm off the via centre
rows = ends(board([off, B_CU], [via(5, 0)], [A, B]))
check("an end inside the via, off its centre, is found",
      len(rows) == 1 and rows[0][0] is off and rows[0][1] == 1
      and abs(rows[0][3] - 0.1414) < 1e-3, rows)
check("an end on the centre is not",
      not ends(board([seg(0, 0, 5, 0), B_CU], [via(5, 0)], [A, B])))
check("an end past the via's radius is not",
      not ends(board([seg(0, 0, 4.6, 0), B_CU], [via(5, 0)], [A, B])))
check("a foreign net's end is not",
      not ends(board([seg(0, 0, 5.1, 0.1, net=FOREIGN), B_CU], [via(5, 0)], [A, B])))
blind = via(5, 0, layers=('In1.Cu', 'B.Cu'))     # row 1's via spans F.Cu
check("an end on a layer the via does not carry is not",
      not ends(board([seg(0, 0, 5.1, 0.1), B_CU], [blind], [A, B])))

print("2. check_weird reports it as advisory")
found = advisory(board([seg(0, -0.25, 5.0, -0.25), B_CU], [via(5, 0)], [A, B]))
check("check_weird reports the end", len(found) == 1, found)
check("the category is advisory", 'off-centre-via-end' in ADVISORY_CATEGORIES
      and not blocking_findings(found))
buf = io.StringIO()
with redirect_stdout(buf):
    print_report(found, [])
out = buf.getvalue()
check("the report says nothing counted, and names the end",
      'NO WEIRD THINGS FOUND' in out and 'advisory, not counted: off-centre-via-end x1' in out
      and '0.2500mm off the centre' in out, out)

print("3. an end on a pad names the pad")
C = pad(5.15, 0, 'C1', size=0.4)          # a same-net pad the end lands on
found = advisory(board([seg(0, 0, 5.15, 0), B_CU], [via(5, 0)], [A, B, C]))
check("the finding names the pad", len(found) == 1 and 'also on pad C1.1' in found[0]['detail'],
      found)

print()
if failures:
    print(f"FAILED: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("all #1235 off-centre via end report checks passed")
