#!/usr/bin/env python3
"""#1692: where the outline under an edge part is not straight, the seat reads
that outline, so a part seated at its body floor passes check_assembly's gate;
or the seat says it could not, naming the part.

The floor (`floorplan.body_clearing_overhang`) is a straight edge's number. Off
one, the band reads the part's distance from the outline where it stands, on its
courtyard unless the edge is straight under the body, so a part at that band
could still have its body through the board. On #1603's fixture, redrawn with a
bent east edge, with a synthetic body at board depth (none of this asks KiCad):

  1. on an arc, a chamfer and a round corner, every position of the seat's line
     passes the gate (`heights.body_in_board`, what check_assembly reads);
  2. its band still grades: nowhere past the declared `max`;
  3. a plain rectangle and a straight stretch of a non-rectangular outline seat
     where they always did;
  4. a band `max` too tight to clear the body keeps a seat, and the note names
     the part, what it reads and what stopped it; a pad floor likewise;
  5. a board with no model read is not asked.

    python3 tests/test_1692_bent_edge_body_floor.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from placement import heights  # noqa: E402
from test_1603_body_in_board import FIXTURE, T, box, low, quiet  # noqa: E402

RUN_ALL_TIMEOUT = 900
#: where the synthetic body at board depth starts along J3's own +x: its pads end
#: at 2.54, so this keeps them a millimetre inside it
BODY_X0 = 3.54
RULE = ('(version 1)\n(rule "edge_launch"\n  (condition "A.Type == \'Pad\' && '
        'A.memberOfFootprint(\'J3\')")\n  (constraint edge_clearance (min 0mm)))\n')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def line(a, b):
    return (f'\t(gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
            f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n')


def arc(a, m, b):
    return (f'\t(gr_arc (start {a[0]} {a[1]}) (mid {m[0]} {m[1]}) (end {b[0]} {b[1]}) '
            f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n')


WEST = line((5, 5), (5, 25))
OUTLINES = {
    'a rectangle': [WEST, line((5, 25), (31, 25)), line((31, 5), (5, 5)), line((31, 5), (31, 25))],
    'an arc': [WEST, line((5, 25), (30, 25)), line((30, 5), (5, 5)),
               arc((30, 5), (31, 15), (30, 25))],
    'a deeper arc': [WEST, line((5, 25), (28.75, 25)), line((28.75, 5), (5, 5)),
                     arc((28.75, 5), (31, 15), (28.75, 25))],
    'a chamfer': [WEST, line((5, 25), (31, 25)), line((27, 5), (5, 5)),
                  line((27, 5), (31, 15)), line((31, 15), (31, 25))],
    'a round corner': [WEST, line((5, 25), (31, 25)), line((21, 5), (5, 5)),
                       arc((21, 5), (28.0711, 7.9289), (31, 15)), line((31, 15), (31, 25))],
    'a small chamfer far off': [WEST, line((5, 25), (29.5, 25)), line((31, 5), (5, 5)),
                                line((31, 5), (31, 23.5)), line((31, 23.5), (29.5, 25))],
}


def drawn(tmp, name, items, dru=None, body_drawn=False):
    """The fixture piled, with `items` for its outline."""
    with open(FIXTURE, encoding='utf-8', newline='') as f:
        t = f.read()
    t, n = re.subn(r'\t\(gr_line\n(?:\t\t[^\n]*\n|\t\t\t[^\n]*\n)*?\t\t\(layer "Edge.Cuts"\)\n'
                   r'[^\n]*\n\t\)\n', '', t)
    assert n == 4, n
    if body_drawn:
        # J3's Fab drawing is concave, so unread: a plain box 0.5 mm short of its
        # courtyard makes it the shape the floor reads
        at = t.index('(footprint "Connector_Coaxial:SMA_Amphenol')
        head, tail = t[:at], t[at:]
        tail, n = re.subn(r'\t\t\(fp_line\n(?:\t\t\t[^\n]*\n|\t\t\t\t[^\n]*\n)*?\t\t\t\(layer "F\.Fab"\)\n'
                          r'[^\n]*\n\t\t\)\n', '', tail)
        assert n, 'J3 Fab lines'
        k = tail.index('\t\t(fp_line')
        t = (head + tail[:k] + '\t\t(fp_rect (start -3 -5.08) (end 13.97 5.08) (stroke (width 0.1) '
             '(type default)) (fill no) (layer "F.Fab"))\n' + tail[k:])
    i = t.rindex('\t(embedded_fonts')
    d = os.path.join(tmp, re.sub(r'\W+', '_', name))
    os.makedirs(d)
    src = os.path.join(d, 'src.kicad_pcb')
    with open(src, 'w', encoding='utf-8', newline='') as f:
        f.write(t[:i] + ''.join(items) + t[i:])
    pile = os.path.join(d, 'pile.kicad_pcb')
    run_utils.check([sys.executable, '-X', 'utf8', '-B',
                     os.path.join(REPO, '.claude/skills/plan-pcb-placement/scripts/make_unplaced.py'),
                     src, pile], accept=True)
    run_utils.evidence(pile, name)
    if dru:
        with open(os.path.join(d, 'pile.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(dru)
    return pile


def modelled(x0, read=True):
    def make(pcb, *_a, **_k):
        if not read:
            return heights.Heights(note='no heights')
        return heights._load({'top': {'J3': 4.34, 'J1': 8.54, 'J2': 8.54, 'C1': 0.5, 'C2': 0.5},
                              'covers': {}, 'bodies': {},
                              'depth': {'J3': heights._depth_raster(
                                  low(box(x0, 15.24, -4.76, 4.76, -3.58, 4.34)), T)},
                              'frame_fits': 1}, pcb, FIXTURE, {})
    return make


def seat(path, x0=BODY_X0, band=None, read=True):
    """`edge_seat.seats` with J3 on the east edge, at `band`, else the floor the
    emit gives it: `(seat, notes, why not, the board, its heights, the band)`."""
    from kicad_parser import parse_kicad_pcb
    from placement import edge_seat, floorplan
    pcb = quiet(parse_kicad_pcb, path)
    real = heights.for_board
    make = modelled(x0, read)
    heights.for_board = lambda p, *_a, **_k: make(p)
    try:
        doc = quiet(floorplan.emit_intent, pcb, path)
        floor = next((c.get('overhang_mm') for c in doc.get('edge_connectors') or ()
                      if c.get('ref') == 'J3'), None)
        band = band or floor or {'min': 10.83, 'max': 11.43}
        doc['edge_connectors'] = [c for c in doc.get('edge_connectors') or ()
                                  if c.get('ref') != 'J3']
        doc['edge_connectors'].append({'ref': 'J3', 'edge': 'east', 'overhang_mm': band})
        intent = floorplan.intent_from_dict(doc, 'test')
        clearance, edge = floorplan.resolved_floors(path)
        st = quiet(edge_seat.seat_state, pcb, path, intent, {}, clearance, edge)
        with contextlib.redirect_stdout(io.StringIO()):
            got, notes, why = edge_seat.seats(st, pcb, path, intent, {'J3'})
        probe = edge_seat._Probe(st, pcb, path, {}, floorplan.outline_state(pcb, path))
    finally:
        heights.for_board = real
    return got.get('J3'), notes, why, pcb, make(pcb), band, probe


def poses_of(s):
    """Both ends and the middle of each stretch of each turn of a seat."""
    out = []
    for rot, ln in s.lines.items():
        for f0, f1 in ln.free:
            for a in (f0, (f0 + f1) / 2.0, f1):
                out.append((a, ln.coord, rot) if s.axis == 0 else (ln.coord, a, rot))
    return out


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    tmp = tempfile.mkdtemp(prefix='t1692_')
    try:
        boards = {name: drawn(tmp, name, items) for name, items in OUTLINES.items()}
        print('on a bent edge, the seat clears the gate:')
        for name in ('an arc', 'a deeper arc', 'a chamfer', 'a round corner'):
            s, notes, why, pcb, h, band, probe = seat(boards[name])
            check(f'{name}: it seats', s is not None, str(why))
            if s is None:
                continue
            read = [heights.body_in_board(h, pcb, {'J3': p}) for p in poses_of(s)]
            bad = [(p, next(r for r in c['rows'] if r['ref'] == 'J3'))
                   for p, c in zip(poses_of(s), read) if c['gating_refs']]
            check(f'...every position of its line passes the gate', not bad, str(bad[:2]))
            over = [probe.band('J3', 'east', p) for p in poses_of(s)]
            check(f'...and its band still grades (max {band["max"]})',
                  max(over) <= band['max'] + 1e-6, str(over))
            check('...and its note carries no warning',
                  not any('WARNING' in n for n in notes), str(notes))

        print('the emit\'s band leaves room for the courtyard the seat reads there:')
        arced = drawn(tmp, 'arc, body drawn', OUTLINES['a deeper arc'], body_drawn=True)
        square = drawn(tmp, 'rectangle, body drawn', OUTLINES['a rectangle'], body_drawn=True)
        s, notes, why, pcb, h, band, probe = seat(arced)
        _s, _n, _w, _p, _h, plain, _pr = seat(square)
        check('a courtyard 0.5 mm past the drawn body: the same floor as on a rectangle, and '
              '`max` that much wider only off one',
              band['min'] == plain['min'] and abs(plain['max'] - plain['min'] - 0.6) < 0.002
              and abs(band['max'] - plain['max'] - 0.5) < 0.002, f'{band} vs {plain}')
        check('...and it seats clear of the gate, in its band',
              s is not None and not any(heights.body_in_board(h, pcb, {'J3': p})['gating_refs']
                                        for p in poses_of(s))
              and not any('WARNING' in n for n in notes)
              and max(probe.band('J3', 'east', p) for p in poses_of(s)) <= band['max'] + 1e-6,
              str(notes) + str(why))

        print('where the band is the right number already:')
        rect, *_r = seat(boards['a rectangle'])
        check('a rectangle seats', rect is not None)
        rect_coord = next(iter(rect.lines.values())).coord
        far, *_r = seat(boards['a small chamfer far off'])
        check('a straight stretch of another outline seats at the same depth',
              far is not None and abs(next(iter(far.lines.values())).coord - rect_coord) < 0.01,
              str(far and far.lines))
        s, _n, _w, pcb, h, _b, probe = seat(boards['a rectangle'])
        check('on a rectangle the seat never reads the gate',
              probe.body_inside('J3', (10.0, 15.0, 0.0)) is None
              and heights.body_in_board(h, pcb, {'J3': (10.0, 15.0, 0.0)})['gating_refs'] == ['J3'])

        print('what it cannot clear, it says:')
        s, notes, why, pcb, h, band, _p = seat(boards['a deeper arc'], band={'min': 10.83, 'max': 10.9})
        text = ' '.join(n for n in notes if 'J3' in n)
        check("a tight `max`: it keeps a seat, as the band alone reads it", s is not None, str(why))
        check('...the note names the part, what the gate reads, and that max',
              'edge connector J3' in text and 'WARNING' in text and 'check_assembly gates' in text
              and 'its overhang_mm.max 10.90mm' in text, text)
        check('...the warning is the truth: it does gate there',
              s is not None and any(heights.body_in_board(h, pcb, {'J3': p})['gating_refs']
                                    for p in poses_of(s)))
        ruled = drawn(tmp, 'pads', OUTLINES['a deeper arc'], dru=RULE)
        s, notes, why, _pcb, _h, _b, _p = seat(ruled, x0=2.54, band={'min': 11.83, 'max': 20.0})
        text = ' '.join(n for n in notes if 'J3' in n) + str(why)
        check('a pad floor no line can reach names itself in its reason',
              s is None and "its pad copper's edge clearance stops it" in text, text)
        s, notes, why, _pcb, _h, _b, _p = seat(boards['an arc'], x0=2.54,
                                               band={'min': 9.0, 'max': 20.0})
        text = ' '.join(n for n in notes if 'J3' in n)
        # #1773: the pad that stops it, read where the body would clear: there
        # it leaves the board, which no edge_clearance rule can free
        check('a pad floor that stops a cleared line: a seat, and the warning names it',
              s is not None and 'WARNING' in text and "its pad copper's edge clearance stops it: "
              "pad 2, 0.20mm from the fab edge floor" in text
              and 'where it clears, pad 2 would leave the board by' in text
              and 'No edge_clearance rule frees a pad on or past the outline' in text, text)

        print('no model read:')
        s, notes, *_r = seat(boards['an arc'], read=False, band={'min': 10.83, 'max': 11.43})
        check('the seat stands where the band puts it, with nothing to say',
              s is not None and not any('WARNING' in n for n in notes), str(notes))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
