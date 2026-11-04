#!/usr/bin/env python3
"""#1633: an edge part seated on the exact end of its `along_edge_band` is
written INSIDE the band.

`edge_seat.project` clamps a part pulled past a seat's free stretch onto the
stretch's end exactly -- a declared band's end, say -- and place_simple wrote
every pose rounded to 4 decimals. Half the time that rounding went outward,
by under 50 nm, and check_floorplan, which measures at 1 nm, graded the
seat's own band an ERROR: `J1 sits at 18% along the 46.00mm west edge,
outside the declared band 17%-18% (0.00mm past its nearer end)` -- a seed
that cannot be DONE, ranked below every clean one, and a message that showed
neither why nor by how much.

Checks:
  1. `edge_seat.written_pose` rounds a coordinate that would land past its
     stretch's end into the stretch, at either end and on either axis, and
     leaves every other coordinate at the nearest step (the old rounding);
  2. place_simple on the issue's board, one seed, with each band end rounding
     OUTWARD (failed 2 edge_connector errors on main) and with the same bands
     moved under 100 nm so each end rounds INWARD (passed on main): 0
     edge_connector errors in both;
  3. the band message prints the percentages in the digits that tell the
     part's from the nearer end's, and a sub-micron `past` in nm.
"""
import contextlib
import io
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, HERE)

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import edge_seat                               # noqa: E402
from placement import floorplan as fp                         # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
GROUPS = os.path.join(ROOT, 'docs', 'examples', 'tigard.groups.json')
# a 0.5 mm band 8 mm along each 46 mm edge, as the fractions an author would
# write; `out` has each end's 5th decimal rounding away from the band, `in`
# the same ends moved under 100 nm, rounding into it
BANDS = {'out': {'J1': (0.168484, 0.179357), 'J5': (0.168484, 0.179357),
                 'J7': (0.16849, 0.179356)},
         'in': {'J1': (0.168485, 0.179356), 'J5': (0.168485, 0.179356),
                'J7': (0.168489, 0.179355)}}


def test_written_pose():
    seat = edge_seat.EdgeSeat('J', 'south', 0, '', {
        0.0: edge_seat.Line(10.0, [(1.00002, 5.00004), (7.0, 9.0)])})
    # on the stretch's ends exactly: 1.00002 rounds to 1.0, outside, so it is
    # written one step in; 5.00004 rounds to 5.0, inside, and is kept
    assert edge_seat.written_pose(seat, 1.00002, 10.0, 0.0) == (1.0001, 10.0)
    assert edge_seat.written_pose(seat, 5.00004, 10.0, 0.0) == (5.0, 10.0)
    seat_hi = edge_seat.EdgeSeat('J', 'south', 0, '', {
        0.0: edge_seat.Line(10.0, [(1.0, 5.00006)])})
    assert edge_seat.written_pose(seat_hi, 5.00006, 10.0, 0.0) == (5.0, 10.0), \
        edge_seat.written_pose(seat_hi, 5.00006, 10.0, 0.0)
    # mid-stretch and the across-edge coordinate: the nearest step, as before
    assert edge_seat.written_pose(seat, 3.123456, 10.000049, 0.0) == (3.1235, 10.0)
    # axis 1 (an east or west edge): y is the coordinate along it
    seat_y = edge_seat.EdgeSeat('J', 'west', 1, '', {
        90.0: edge_seat.Line(2.0, [(3.00003, 4.0)])})
    assert edge_seat.written_pose(seat_y, 2.000049, 3.00003, 90.0) == (2.0, 3.0001)
    # a turn the seat does not have, and a stretch narrower than a step:
    # the nearest step
    assert edge_seat.written_pose(seat_y, 2.0, 3.00003, 0.0) == (2.0, 3.0)
    seat_thin = edge_seat.EdgeSeat('J', 'south', 0, '', {
        0.0: edge_seat.Line(10.0, [(1.00002, 1.00004)])})
    assert edge_seat.written_pose(seat_thin, 1.00002, 10.0, 0.0) == (1.0, 10.0)
    print('  PASS: written_pose rounds a stretch end into its stretch, and every '
          'other coordinate to the nearest step')


def _grade(intent_path, board):
    with contextlib.redirect_stdout(io.StringIO()):
        return fp.grade(fp.load_intent(intent_path), parse_kicad_pcb(board), board)


def test_place_simple(td):
    evidence(BOARD, 'fixture board')
    evidence(GROUPS, 'groups file')
    emit = os.path.join(td, 'emit.json')
    check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), BOARD,
           '--emit-intent', emit], accept=True)
    placed = {}
    for arm, bands in BANDS.items():
        doc = json.load(open(evidence(emit, 'emitted intent'), encoding='utf-8'))
        assert sorted(c['ref'] for c in doc['edge_connectors']) == sorted(bands), \
            doc['edge_connectors']
        for c in doc['edge_connectors']:
            f0, f1 = bands[c['ref']]
            c['along_edge_band'] = {'from': f0, 'to': f1}
        ipath = os.path.join(td, f'{arm}.json')
        json.dump(doc, open(ipath, 'w', encoding='utf-8'), indent=1)
        out = os.path.join(td, arm, 'placed.kicad_pcb')
        os.makedirs(os.path.dirname(out))
        check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_placer', 'place_simple.py'),
               BOARD, out, '--groups', GROUPS, '--intent', ipath, '--seed', '0'], accept=True)
        bad = [v.message for v in _grade(ipath, evidence(out, f'{arm} placed board')).errors
               if v.rule == 'edge_connector']
        assert not bad, (arm, bad)
        placed[arm] = (ipath, out)
    print('  PASS: seed 0 grades its own bands clean, each end rounding outward '
          '(2 errors on main) and inward')
    return placed


def test_message(td, placed):
    ipath, out = placed['out']
    graded = _grade(ipath, out)
    row = next(r for r in graded.edge_seating if r['ref'] == 'J1')
    frac = (row['along_edge_offset_mm'] + row['span_mm'] / 2.0) / row['span_mm']
    # J1's band moved to end 4e-6 of the 46 mm edge (184 nm, +-75 nm for the
    # row's 4-decimal rounding) short of where J1 stands
    doc = json.load(open(ipath, encoding='utf-8'))
    for c in doc['edge_connectors']:
        if c['ref'] == 'J1':
            c['along_edge_band'] = ({'from': frac - 0.05, 'to': frac - 4e-6})
    tight = os.path.join(td, 'tight.json')
    json.dump(doc, open(tight, 'w', encoding='utf-8'), indent=1)
    msgs = [v for v in _grade(tight, out).errors
            if v.rule == 'edge_connector' and v.ref == 'J1']
    assert len(msgs) == 1, [v.message for v in msgs]
    v = msgs[0]
    assert 'nm past its nearer end' in v.message, v.message
    sits = v.message.split(' sits at ')[1].split('%')[0]
    band = v.message.split('declared band ')[1].split(' (')[0]
    hi = band.split('-')[1].rstrip('%')
    assert sits != hi and len(sits.split('.')[-1]) == len(hi.split('.')[-1]), v.message
    print(f'  PASS: the band message reads the miss: {v.message}')


def main():
    test_written_pose()
    with tempfile.TemporaryDirectory() as td:
        placed = test_place_simple(td)
        test_message(td, placed)
    print('test_1633_band_end_rounding: all passed')


if __name__ == '__main__':
    main()
