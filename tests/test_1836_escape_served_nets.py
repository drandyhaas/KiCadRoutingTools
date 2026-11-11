"""#1836: a face's nets that END on the neighbour across from it need no lane.

`escape._blocked_span` charges a neighbour's pad copper against a face
without asking where the face's nets go, so a resistor array straight across
from the IC pins it serves read the IC as the wall it had to get past, and
`check_floorplan --health` said to move the part the nets are meant to reach.

The board below puts a 12-pad part U1 (east column E1..E6, west column
W1..W6) beside a different neighbour, one per case:

  * FULL     RN1 straight across U1's east face, a pad of each of E1..E6 in
             the escape band. Every east net is served there: no deficit, and
             RN1 is reported as `served_by`, never as a part to move.
  * PARTIAL  the same, but RN1 carries only E1..E3; its other three pads are
             foreign nets. E4..E6 still need lanes, and RN1 still covers the
             whole span, so the face stays short by those and RN1 is still a
             blocker -- the charge is kept for the nets that do not end on it.
  * REVERSED RN1 carries E1..E6 in the opposite order, so E1's pad is at the
             far end of the face from U1's E1 pin. Only a target across
             from the net's own pin -- within the band depth along the face,
             no shallower than 45 degrees -- serves it: E3 and E4 do, E1
             and E6 (2.2 mm apart along the face) do not.
  * FAR PAD  a two-terminal R1 whose E3 pad lies BEYOND the escape band and
             whose near pad is a foreign net. Reaching the far pad means
             passing the near one, so E3 is not served there.

A real board closes it: tigard at its own Default class (0.15 / 0.15), where
the four series arrays facing U5 / U6 read short with U5 / U6 as the blocker.

    python3 tests/test_1836_escape_served_nets.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import escape                                  # noqa: E402
from run_utils import evidence                                # noqa: E402

FAILS = []
#: lane 0.3 mm, so the escape band is 4 x 0.3 = 1.2 mm off each face
RULES = dict(track_width=0.15, clearance=0.15)


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def board(path, case):
    """U1 at (10, 10): pads 0.6 x 0.3 at local x +-1.5, y -1.25..1.25 step
    0.5, so its copper box runs x 8.2..11.8 and the east band x 11.8..13.0.
    The neighbour per `case` sits in that band."""
    nets = ['E%d' % k for k in range(1, 7)] + ['W%d' % k for k in range(1, 7)]
    nets += ['F%d' % k for k in range(1, 7)]
    nid = {n: i + 1 for i, n in enumerate(nets)}

    def pad(num, x, y, net, size='0.6 0.3'):
        return (f'    (pad "{num}" smd rect (at {x} {y}) (size {size})'
                f' (layers "F.Cu") (net {nid[net]} "{net}"))\n')
    ys = [-1.25 + 0.5 * i for i in range(6)]
    upads = ''.join(pad(i + 1, 1.5, y, 'E%d' % (i + 1)) for i, y in enumerate(ys))
    upads += ''.join(pad(i + 7, -1.5, y, 'W%d' % (i + 1)) for i, y in enumerate(ys))
    # the far ends of the west nets, well away from anything
    wpads = ''.join(pad(i + 1, 0, 0.5 * i, 'W%d' % (i + 1)) for i in range(6))
    if case == 'full':
        npads = ''.join(pad(i + 1, 0, y, 'E%d' % (i + 1)) for i, y in enumerate(ys))
        nref, nat = 'RN1', '12.6 10'
    elif case == 'reversed':
        npads = ''.join(pad(i + 1, 0, y, 'E%d' % (6 - i)) for i, y in enumerate(ys))
        nref, nat = 'RN1', '12.6 10'
    elif case == 'partial':
        npads = ''.join(pad(i + 1, 0, y, ('E%d' if i < 3 else 'F%d') % (i + 1))
                        for i, y in enumerate(ys))
        nref, nat = 'RN1', '12.6 10'
    else:                                         # far pad
        # near pad F1 at x 12.6 (in the band), far pad E1 at x 14.4 (beyond
        # it), both on the row of U1's E1 pad; a long body so the near pad
        # covers the whole span
        npads = (pad(1, 0, 0, 'F1', size='0.6 3.0')
                 + pad(2, 1.8, -0.25, 'E3'))
        nref, nat = 'R1', '12.6 10'
    # the far ends of the east nets that do not end on the neighbour
    epads = ''.join(pad(i + 1, 0, 0.5 * i, 'E%d' % (i + 1)) for i in range(6))
    epads += ''.join(pad(i + 7, 1, 0.5 * i, 'F%d' % (i + 1)) for i in range(6))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n'
                 + ''.join(f'  (net {nid[n]} "{n}")\n' for n in nets) +
                 '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))'
                 ' (layer "Edge.Cuts"))\n'
                 '  (footprint "t:SOIC" (layer "F.Cu") (at 10 10)\n'
                 '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
                 + upads + '  )\n'
                 f'  (footprint "t:NEIGHBOUR" (layer "F.Cu") (at {nat})\n'
                 f'    (property "Reference" "{nref}" (at 0 0) (layer "F.SilkS"))\n'
                 + npads + '  )\n'
                 '  (footprint "t:FARW" (layer "F.Cu") (at 2 20)\n'
                 '    (property "Reference" "X1" (at 0 0) (layer "F.SilkS"))\n'
                 + wpads + '  )\n'
                 '  (footprint "t:FARE" (layer "F.Cu") (at 30 20)\n'
                 '    (property "Reference" "X2" (at 0 0) (layer "F.SilkS"))\n'
                 + epads + '  )\n'
                 ')\n')
    return path


def east(path):
    pcb = parse_kicad_pcb(path)
    with contextlib.redirect_stdout(io.StringIO()):
        led = escape.escape_ledger(pcb, pcb_file=path, refs=['U1'], **RULES)
    p = led[0]
    return p, next(f for f in p.faces if f.face == 'east'), pcb


def names(pcb, ids):
    return sorted(pcb.nets[n].name for n in (ids or ()))


def main(td):
    p, f, pcb = east(board(os.path.join(td, 'full.kicad_pcb'), 'full'))
    face = names(pcb, f.nets)
    ok('FULL: the east face carries E-nets and is blocked end to end by RN1',
       len(face) >= 4 and all(n.startswith('E') for n in face)
       and f.blocked_mm > 2.5 and f.supply == 0 and 'RN1' in f.blockers,
       (face, f.blocked_mm, f.supply, f.blockers))
    ok('FULL: every net on the face ends on RN1 and is served there',
       names(pcb, getattr(f, 'served_nets', None)) == face
       and getattr(f, 'served_by', None) == ('RN1',),
       (face, names(pcb, getattr(f, 'served_nets', None)),
        getattr(f, 'served_by', None)))
    ok('FULL: no deficit on any layer count, and the part is not short',
       f.deficit == 0 and f.deficit_floor == 0 and p.worst is None,
       (f.deficit, f.deficit_floor))
    d = next(x for x in p.to_dict()['faces'] if x['face'] == 'east')
    ok('FULL: the row publishes served, served_by and lane_demand',
       d.get('served') == len(face) and d.get('served_by') == ['RN1']
       and d.get('lane_demand') == 0 and d['demand'] == len(face), d)

    p, f, pcb = east(board(os.path.join(td, 'partial.kicad_pcb'), 'partial'))
    face = names(pcb, f.nets)
    near = [n for n in face if n in ('E1', 'E2', 'E3')]
    rest = [n for n in face if n not in near]
    ok('PARTIAL: the nets RN1 carries are served, the others are not',
       near and rest and names(pcb, getattr(f, 'served_nets', None)) == near,
       (face, names(pcb, getattr(f, 'served_nets', None))))
    ok('PARTIAL: the nets that do not end on RN1 are still short past it',
       (f.supply, f.deficit) == (0, len(rest)) and 'RN1' in f.blockers,
       (f.demand, f.supply, f.deficit, f.blockers))

    p, f, pcb = east(board(os.path.join(td, 'reversed.kicad_pcb'), 'reversed'))
    face = names(pcb, f.nets)
    got = names(pcb, getattr(f, 'served_nets', None))
    # copper edge to copper edge along the face: E3/E4 0.2 mm apart, E1/E6
    # 2.2 mm -- past the 1.2 mm band depth
    ok('REVERSED: a net whose RN1 pad is across from its own pin is served, '
       'one whose RN1 pad is at the far end of the face is not',
       {'E3', 'E4'} <= set(got) and not {'E1', 'E6'} & set(got)
       and {'E3', 'E4'} <= set(face),
       (face, got))

    p, f, pcb = east(board(os.path.join(td, 'far.kicad_pcb'), 'far'))
    face = names(pcb, f.nets)
    ok('FAR PAD: an E3 pad beyond the band, behind a foreign pad, serves nothing',
       'E3' in face and not getattr(f, 'served_nets', ())
       and f.deficit == len(face) and 'R1' in f.blockers,
       (face, names(pcb, getattr(f, 'served_nets', None)), f.deficit,
        f.blockers))

    # the real board the issue measured, at its own Default class
    tg = os.path.join(td, 'tg')
    os.makedirs(tg)
    src = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
    dst = os.path.join(tg, 'tigard.kicad_pcb')
    shutil.copyfile(evidence(src, 'tigard board'), dst)
    with open(os.path.join(tg, 'tigard.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump({'net_settings': {'classes': [{
            'name': 'Default', 'clearance': 0.15, 'track_width': 0.15,
            'via_diameter': 0.5, 'via_drill': 0.3}]}}, fh)
    tp = parse_kicad_pcb(dst)
    with contextlib.redirect_stdout(io.StringIO()):
        led = escape.escape_ledger(tp, pcb_file=dst, fine_at_board_rules=True)
    rows = {(x.ref, x.face): x for q in led for x in q.faces}
    for ref, ic in (('RN1', 'U5'), ('RN2', 'U5'), ('RN5', 'U6'), ('RN6', 'U6')):
        x = rows[(ref, 'east')]
        ok(f'tigard {ref} east: its nets end on {ic}, so it is not short '
           f'and {ic} is named as served_by',
           x.deficit == 0 and ic in getattr(x, 'served_by', ()) and x.demand == 3,
           (x.demand, x.supply, x.deficit, getattr(x, 'served_by', ()), x.blockers))
    short_blockers = {b for q in led for x in q.faces if x.deficit
                      for b in x.blockers}
    ok('tigard: no face is short with a blocker that serves EVERY one of its nets',
       not [(x.ref, x.face) for q in led for x in q.faces
            if x.deficit and getattr(x, 'served_nets', ()) and len(getattr(x, 'served_nets', ())) == x.demand],
       sorted(short_blockers))


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as td:
        main(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
