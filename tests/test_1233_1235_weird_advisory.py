#!/usr/bin/env python3
"""#1233-#1235: check_weird's ADVISORY categories, and that they stay advisory.

Synthetic Segment/Via/Pad boards, in the style of tests/test_check_weird.py:

1. off-angle-corner (#1233): a 30 degree turn is flagged, a 135 degree turn
   (an acute corner) is flagged, 45 and 90 are not, 45.9 is within the 1
   degree tolerance, a joint on a same-net pad is not a corner, a T is not a
   two-segment joint, and the chords of one arc track (one shared uuid) are
   not corners.
2. short-fragment (#1234): a 0.05 mm segment mid-route is flagged; one
   entering a pad or a via is not; 0.1 mm is not under the floor; the floor
   moves with `fragment_floor`; arc chords are not fragments.
3. off-centre-via-end (#1235): a track ending 0.1 mm from a via's centre is
   flagged once per layer; one ending on the centre is not; one beyond the
   via's copper is not; one on a layer the via does not span is not.
4. They are ADVISORY: check_weird() returns them only with advisory=True, the
   CLI prints them and exits 0 when they are all it found, and --json counts
   them; a non-advisory finding still exits 1.

    python3 tests/test_1233_1235_weird_advisory.py
"""
import json
import os
import subprocess
import sys
import tempfile
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import Pad, Segment, Via, Net, BoardInfo, PCBData  # noqa: E402
from check_weird import (check_weird, advisory_findings,  # noqa: E402
                         ADVISORY_CATEGORIES, CATEGORIES)

RUN_ALL_FAST_OK = True

NET = 1
NAME = '/TEST'
FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _pad(x, y, size=0.6, num='1', ref='U1'):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=size, size_y=size, shape='circle',
               layers=['F.Cu'], net_id=NET, net_name=NAME, rotation=0.0, drill=0.0)


def _seg(x1, y1, x2, y2, layer='F.Cu', uuid=''):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=0.2,
                   layer=layer, net_id=NET, uuid=uuid)


def _via(x, y, layers=('F.Cu', 'B.Cu')):
    return Via(x=x, y=y, size=0.6, drill=0.3, layers=list(layers), net_id=NET)


def _pcb(segments, vias=(), pads=()):
    return PCBData(
        board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
        nets={NET: Net(net_id=NET, name=NAME, pads=list(pads))},
        footprints={}, vias=list(vias), segments=list(segments),
        pads_by_net={NET: list(pads)}, zones=[])


def _count(pcb, cat, **kw):
    return sum(1 for f in advisory_findings(pcb, **kw) if f['category'] == cat)


def _turn(deg, pads=()):
    import math
    r = math.radians(deg)
    return _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 5 + 3 * math.cos(r), 3 * math.sin(r))],
                pads=pads)


def main():
    # ---- 1. off-angle-corner --------------------------------------------------
    for deg, want in ((30, 1), (135, 1), (45, 0), (90, 0), (45.9, 0), (47.5, 1)):
        check(f'1. a {deg} degree turn -> {want}',
              _count(_turn(deg), 'off-angle-corner') == want)
    check('1. a 30 degree joint on a same-net pad is not a corner',
          _count(_turn(30, pads=[_pad(5, 0)]), 'off-angle-corner') == 0)
    tee = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 10, 0), _seg(5, 0, 7, 1.2)])
    check('1. a T (three segments) is not a two-segment corner',
          _count(tee, 'off-angle-corner') == 0)
    arc = _pcb([_seg(0, 0, 1, 0.1, uuid='arc-1'), _seg(1, 0.1, 2, 0.4, uuid='arc-1')])
    check('1. chords of one arc track are not corners', _count(arc, 'off-angle-corner') == 0)
    arc_end = _pcb([_seg(-3, 0, 0, 0), _seg(0, 0, 1, 0.1, uuid='arc-1'),
                    _seg(1, 0.1, 2, 0.4, uuid='arc-1')])
    check('1. a line meeting an arc chord is not a corner either',
          _count(arc_end, 'off-angle-corner') == 0)

    # ---- 2. short-fragment ----------------------------------------------------
    mid = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 5.05, 0), _seg(5.05, 0, 9, 0)])
    check('2. a 0.05 mm segment mid-route is a fragment', _count(mid, 'short-fragment') == 1)
    padded = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 5.05, 0)], pads=[_pad(5.05, 0)])
    check('2. one entering a pad is not', _count(padded, 'short-fragment') == 0)
    via = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 5.05, 0)], vias=[_via(5.05, 0)])
    check('2. one entering a via is not', _count(via, 'short-fragment') == 0)
    at_floor = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 5.1, 0), _seg(5.1, 0, 9, 0)])
    check('2. 0.1 mm is not under the 0.1 floor', _count(at_floor, 'short-fragment') == 0)
    check('2. the floor moves with fragment_floor',
          _count(at_floor, 'short-fragment', fragment_floor=0.2) == 1)
    chords = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 5.05, 0.01, uuid='arc-2'),
                   _seg(5.05, 0.01, 5.1, 0.03, uuid='arc-2')])
    check('2. arc chords are not fragments', _count(chords, 'short-fragment') == 0)

    # ---- 3. off-centre-via-end ------------------------------------------------
    two = _pcb([_seg(0, 0, 5.1, 0), _seg(5.1, 0, 9, 3, layer='B.Cu')], vias=[_via(5, 0)])
    check('3. ends 0.1 mm off the centre, on two layers, are two findings',
          _count(two, 'off-centre-via-end') == 2)
    centred = _pcb([_seg(0, 0, 5, 0)], vias=[_via(5, 0)])
    check('3. an end on the centre is not', _count(centred, 'off-centre-via-end') == 0)
    beyond = _pcb([_seg(0, 0, 4.5, 0)], vias=[_via(5, 0)])
    check("3. an end beyond the via's copper is not", _count(beyond, 'off-centre-via-end') == 0)
    blind = _pcb([_seg(0, 0, 5.1, 0, layer='B.Cu')], vias=[_via(5, 0, layers=('F.Cu', 'In1.Cu'))])
    blind.board_info.copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    check('3. an end on a layer the via does not span is not',
          _count(blind, 'off-centre-via-end') == 0)

    # ---- 4. advisory ------------------------------------------------------------
    check('4. the advisory set is registered in CATEGORIES',
          set(ADVISORY_CATEGORIES) <= set(CATEGORIES))
    board = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 5 + 2.6, 1.5), _seg(5 + 2.6, 1.5, 9, 1.5)],
                 pads=[_pad(0, 0), _pad(9, 1.5, num='2', ref='U2')])
    plain, _ = check_weird(board)
    asked, _ = check_weird(board, advisory=True)
    check('4. check_weird() returns no advisory finding by default',
          not any(f['category'] in ADVISORY_CATEGORIES for f in plain), str(plain))
    check('4. ...and returns them with advisory=True',
          Counter(f['category'] for f in asked)['off-angle-corner'] == 2, str(asked))

    text = ('(kicad_pcb (version 20240202) (generator "t")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (net 0 "") (net 1 "/A")\n'
            ' (footprint "L:P" (layer "F.Cu") (at 100 100) (property "Reference" "U1")\n'
            '  (pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A")))\n'
            ' (footprint "L:P" (layer "F.Cu") (at 109 101.5) (property "Reference" "U2")\n'
            '  (pad "1" smd circle (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A")))\n'
            ' (segment (start 100 100) (end 105 100) (width 0.2) (layer "F.Cu") (net 1))\n'
            ' (segment (start 105 100) (end 107.6 101.5) (width 0.2) (layer "F.Cu") (net 1))\n'
            ' (segment (start 107.6 101.5) (end 109 101.5) (width 0.2) (layer "F.Cu") (net 1))\n'
            ')\n')
    d = tempfile.mkdtemp()
    path, js = os.path.join(d, 'b.kicad_pcb'), os.path.join(d, 'w.json')
    with open(path, 'w') as fh:
        fh.write(text)
    tool = os.path.join(ROOT, 'py_router', 'check_weird.py')
    r = subprocess.run([sys.executable, tool, path, '--json', js], capture_output=True, text=True)
    check('4. the CLI exits 0 when only advisory findings exist', r.returncode == 0,
          r.stdout[-400:] + r.stderr[-400:])
    check('4. ...and prints them under ADVISORY', 'ADVISORY (2' in r.stdout, r.stdout[-400:])
    with open(js) as fh:
        doc = json.load(fh)
    check('4. --json counts them', doc['counts'].get('off-angle-corner') == 2, str(doc['counts']))
    r = subprocess.run([sys.executable, tool, path, '--no-advisory', '--json', js],
                       capture_output=True, text=True)
    with open(js) as fh:
        doc = json.load(fh)
    check('4. --no-advisory skips them', doc['counts'].get('off-angle-corner') == 0,
          str(doc['counts']))
    with open(path, 'w') as fh:
        fh.write(text.replace(')\n)\n', ')\n (segment (start 105 100) (end 105 103) (width 0.2)'
                                        ' (layer "F.Cu") (net 1))\n)\n', 1))
    r = subprocess.run([sys.executable, tool, path], capture_output=True, text=True)
    check('4. a non-advisory finding (a dangling spur) still exits 1', r.returncode == 1,
          r.stdout[-400:])
    for p in (path, js):
        os.remove(p)
    os.rmdir(d)

    print(f"\n{'ALL PASS' if not FAILS else '%d FAIL' % len(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
