#!/usr/bin/env python3
"""A part seated on one edge walls another edge's seat line where it stands
wherever it seats (`edge_seat._corners`).

A seat line is read against the parts the anneal does not move, so a part
seated on the ADJACENT edge near a corner was invisible to it. CM5_MINIMA's
HDMI receptacle J501, seated on the south edge (origin anywhere in x
89.50-94.10), covers x 85.70-98.00 over y 68.8-80.7 wherever it stands, and
the west edge's J101 and USB701 were still given their line up to y 76.0;
the anneal put them there, and of seeds 0-3 and 7 three ended with J501 and
J101 or USB701 illegal (after 3 anneals each) and one seed was judged. Each
line is now cut where its part would overlap what a part on another edge
covers at every pose its seat allows (`_covered`), to a fixed point; a part
left no position is placed as any other part, with a reason naming the parts
that walled it; and a part `_crowding` drops walls nothing. On CM5 none of
the five seeds is then illegal and all five are judged.

Checks, on small boards built here (60 x 40 mm, south-west corner):
  1. the west part's line stops where the south part's courtyard is,
     wherever that one stands (y 34-40): it ends at 34 - 4 - SEAT_GAP_MM;
  2. two parts that wall each other wherever both stand: the later declared
     is unseated, the reason naming the other, which keeps its whole line;
  3. a part on the other face is not walled (the south part has no drilled
     pads, so it covers nothing on B);
  4. a south part `_crowding` drops (it does not fit beside one declared
     before it) walls nothing: the west line is whole again;
  5. NEGATIVE CONTROL: with `_corners` cutting nothing, the west line of
     check 1 runs into the corner (to y 37.3), where the west part overlaps
     the south part at every pose that one has.

    python3 tests/test_edge_seat_corners.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
from unittest import mock

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
import pose_score                                              # noqa: E402
from placement import edge_seat, floorplan                     # noqa: E402
from placement.mating import facing_edge                       # noqa: E402
from run_utils import evidence                                 # noqa: E402

failures = []
CLEARANCE, EDGE = 0.2, 0.2
GAP = edge_seat.SEAT_GAP_MM
#: the local face that looks west at a quarter turn of 90
WEST_FACE = next(f for f in ('+x', '-x', '+y', '-y') if facing_edge(f, 90.0) == 'west')


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def footprint(ref, at, crt, pads=(), side='F', locked=False):
    """A courtyard-only part: `crt` (x0, y0, x1, y1) local, a pad at each x
    of `pads` (y 0)."""
    x0, y0, x1, y1 = crt
    return (f'  (footprint "Test:Part"{" (locked yes)" if locked else ""} '
            f'(layer "{side}.Cu") (at {at})\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) (layer "{side}.SilkS"))\n'
            f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) (layer "{side}.CrtYd"))\n'
            + ''.join(f'    (pad "{i + 1}" smd rect (at {x} 0) (size 1 1) '
                      f'(layers "{side}.Cu" "{side}.Mask"))\n' for i, x in enumerate(pads))
            + '  )\n')


#: the south part: 14 mm along its edge, 6 deep, pads at x +-5
SOUTH = ((-7, -3, 7, 3), (-5, 5))
#: a west part: 8 mm along its edge at a quarter turn, 10 deep
WEST = ((-4, -5, 4, 5), (-2, 2))
#: a locked wall along the south edge from x 20 to the east edge: the south
#: part's origin can stand only in x 5.7-13, so its courtyard always covers
#: x 6-12.7, y 34-40
SOUTH_WALL = footprint('W1', '40 36', (-20, -4, 20, 4), (0,), locked=True)


def board(path, parts):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t_edge_corner")\n  (net 0 "")\n'
                 '  (gr_rect (start 0 0) (end 60 40) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return evidence(path, 'synthetic board')


def row(ref, edge, face):
    return {'ref': ref, 'edge': edge, 'class': 'edge_receptacle', 'mating_face': face}


def seat(path, rows):
    """`(state, seats, unseated, crowded)`."""
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
           'edge_connectors': rows}
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=CLEARANCE,
                                      board_edge_clearance=EDGE)
        movable = {r for r, p in state.parts.items() if not p.locked}
        crowded = []
        out, _notes, unseated = edge_seat.seats(
            state, pcb, path, floorplan.intent_from_dict(doc), movable, crowded=crowded)
    return state, out, unseated, crowded


def line_end(out, ref, rot=90.0):
    ln = out[ref].lines.get(rot) if ref in out else None
    return max(f1 for _f0, f1 in ln.free) if ln else None


def overlaps_everywhere(state, seats, a, a_at, b):
    """Whether `a` at y `a_at` on its line overlaps `b` at both ends of b's
    line (and so at every pose b's seat allows)."""
    pa = state.parts[a]
    la = seats[a].lines[90.0]
    ra = pa.rect(la.coord, a_at, 90.0)
    lb = seats[b].lines[0.0]
    ends = [x for f in lb.free for x in f]
    for x in (min(ends), max(ends)):
        rb = state.parts[b].rect(x, lb.coord, 0.0)
        if min(ra[2], rb[2]) - max(ra[0], rb[0]) <= 0 or min(ra[3], rb[3]) - max(ra[1], rb[1]) <= 0:
            return False
    return True


def main():
    tmp = tempfile.mkdtemp(prefix='t_edge_corner_')
    try:
        one = board(os.path.join(tmp, 'one.kicad_pcb'),
                    [SOUTH_WALL, footprint('S', '30 20 0', *SOUTH),
                     footprint('A', '40 20 90', *WEST)])
        rows = [row('S', 'south', '+y'), row('A', 'west', WEST_FACE)]

        # 1. the west line stops at the south part
        state, out, unseated, _c = seat(one, rows)
        end = line_end(out, 'A')
        s_free = out['S'].lines[0.0].free if 'S' in out else ()
        check('1. both seated, the south part in x 5.7-13',
              sorted(out) == ['A', 'S'] and len(s_free) == 1
              and abs(s_free[0][0] - 5.705) < 0.05 and abs(s_free[0][1] - 13.0) < 0.05,
              f'{sorted(out)} {s_free} {unseated}')
        check('1. the west line ends at y 34 - 4 - SEAT_GAP_MM',
              end is not None and abs(end - (30.0 - GAP)) < 1e-6, f'{end}')
        check('1. and nowhere on it does the west part overlap the south part everywhere',
              end is not None and not overlaps_everywhere(state, out, 'A', end, 'S'))
        p = state.parts['A']
        check('1. the west part stands on its line', 'A' in out and
              any(f0 - 1e-9 <= p.y <= f1 + 1e-9 for f0, f1 in out['A'].lines[90.0].free),
              f'y {p.y}')

        # 2. two parts that wall each other: the later declared goes
        two = board(os.path.join(tmp, 'two.kicad_pcb'),
                    [SOUTH_WALL, footprint('W2', '5 15', (-5, -15, 5, 15), (0,), locked=True),
                     footprint('S', '30 20 0', *SOUTH), footprint('B', '40 20 90', *WEST)])
        _st, out, unseated, _c = seat(two, [row('S', 'south', '+y'), row('B', 'west', WEST_FACE)])
        why = unseated.get('B', '')
        check('2. the later declared is unseated, the reason names the other',
              sorted(out) == ['S'] and 'S' in why and 'another edge' in why,
              f'{sorted(out)} {unseated}')
        check('2. the one kept keeps its whole line',
              'S' in out and abs(out['S'].lines[0.0].free[-1][1] - 13.0) < 0.05,
              str(out.get('S')))
        _st, out, unseated, _c = seat(two, [row('B', 'west', WEST_FACE), row('S', 'south', '+y')])
        check('2. declared the other way round, the other is unseated',
              sorted(out) == ['B'] and 'B' in unseated.get('S', ''), f'{sorted(out)} {unseated}')

        # 3. the other face is not walled: its line is the line it has uncut
        face = board(os.path.join(tmp, 'face.kicad_pcb'),
                     [SOUTH_WALL, footprint('S', '30 20 0', *SOUTH),
                      footprint('C', '40 20 90', *WEST, side='B')])
        face_rows = [row('S', 'south', '+y'), row('C', 'west', None)]
        _st, out, _u, _c = seat(face, face_rows)
        with mock.patch.object(edge_seat, '_corners', lambda st, seats: (seats, {})):
            _st, uncut, _u, _c = seat(face, face_rows)
        check('3. a B-side west part keeps the line it has uncut, past y 34',
              'C' in out and out['C'].lines == uncut['C'].lines
              and (line_end(out, 'C') or 0) > 34.0, f"{out.get('C')}")

        # 4. a south part `_crowding` drops walls nothing
        crowd = board(os.path.join(tmp, 'crowd.kicad_pcb'),
                      [footprint('W1', '38 36', (-22, -4, 22, 4), (0,), locked=True),
                       footprint('S1', '30 20 0', (-3, -3, 3, 3), (-1, 1)),
                       footprint('S2', '30 10 0', *SOUTH), footprint('A', '40 20 90', *WEST)])
        _st, out, _u, crowded = seat(crowd, [row('S1', 'south', '+y'), row('S2', 'south', '+y'),
                                             row('A', 'west', WEST_FACE)])
        end = line_end(out, 'A')
        check('4. S2 is dropped by crowding, S1 and A seated',
              sorted(out) == ['A', 'S1'] and [c.get('unseated') for c in crowded] == [['S2']],
              f'{sorted(out)} {crowded}')
        check('4. the west line is whole again (S1 covers nothing wherever it stands)',
              end is not None and end > 37.0, f'{end}')

        # 5. negative control: no cut, and the west line runs into the corner
        with mock.patch.object(edge_seat, '_corners', lambda st, seats: (seats, {})):
            state, out, _u, _c = seat(one, rows)
        end = line_end(out, 'A')
        check('5. NEGATIVE CONTROL: uncut, the line reaches y 37.3 and overlaps S everywhere',
              end is not None and end > 37.0 and overlaps_everywhere(state, out, 'A', end, 'S'),
              f'{end}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
