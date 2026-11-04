"""#1425: place_simple reports how many far-face parts stand under a ball
grid's field, and how many of them in the outer rows a fan-out escapes through.

A back-face part centred inside a BGA's ball field takes a via or a track lane
of the fan-out, the outer rows most of all, and place_simple's JSON had no key
for it. `placement.escape.ball_field_census` counts them (by ring in from the
field's edge) and place_simple prints, reports (`bga_field`) and writes them to
`judge.json`.

    python3 tests/test_1425_ball_field_census.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _bga(ref, x, y, n=7, pitch=1.0):
    """An n x n ball grid, locked, on F.Cu."""
    pads = ''.join(
        f'    (pad "{chr(65 + i)}{j + 1}" smd circle (at {(j - (n - 1) / 2) * pitch:g}'
        f' {(i - (n - 1) / 2) * pitch:g}) (size 0.4 0.4) (layers "F.Cu")'
        f' (net {1 + (i * n + j) % 5} "N{1 + (i * n + j) % 5}"))\n'
        for i in range(n) for j in range(n))
    hw = n * pitch / 2 + 0.3
    return (f'  (footprint "t:BGA" (locked yes) (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-hw} {-hw}) (end {hw} {hw}) (stroke (width 0.05)'
            f' (type default)) (layer "F.CrtYd"))\n' + pads + '  )\n')


def _cap(ref, x, y, layer='B.Cu'):
    return (f'  (footprint "t:C" (locked yes) (layer "{layer}") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "{layer[0]}.SilkS"))\n'
            '    (fp_rect (start -0.8 -0.4) (end 0.8 0.4) (stroke (width 0.05)'
            f' (type default)) (layer "{layer[0]}.CrtYd"))\n'
            f'    (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "{layer}")'
            ' (net 1 "N1"))\n'
            f'    (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "{layer}")'
            ' (net 2 "N2")))\n')


def _board(path):
    # balls span x 17..23, y 12..18 (7 x 7, 1 mm): ring k = k mm in from an edge
    parts = (_bga('U1', 20, 15)
             + _cap('C1', 17.0, 15.0)      # ring 0: under the outermost column
             + _cap('C2', 18.5, 15.0)      # ring 2 (1.5 mm in -> 2)
             + _cap('C3', 20.0, 15.0)      # ring 3, the field's centre
             + _cap('C4', 33.0, 5.0)       # outside the field
             + _cap('C5', 20.0, 13.0, 'F.Cu'))   # inside the field, but the SAME face
    nets = ''.join(f'  (net {i} "N{i}")\n' for i in range(1, 6))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n' + nets
                 + '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n' + parts + ')\n')
    return path


def main():
    from placement.escape import ball_field_census
    with tempfile.TemporaryDirectory(prefix='t1425_') as td:
        board = _board(os.path.join(td, 'b.kicad_pcb'))
        rows = ball_field_census(parse_kicad_pcb(board))
        ok('one ball grid has parts under its field', len(rows) == 1, rows)
        r = rows[0] if rows else {}
        ok('the far-face parts centred inside it are counted (C1, C2, C3)',
           r.get('refs') == ['C1', 'C2', 'C3'] and r.get('parts') == 3, r.get('refs'))
        ok('a part outside the field, and one on the BGA\'s own face, are not',
           'C4' not in r.get('refs', ()) and 'C5' not in r.get('refs', ()))
        ok('rings are pitches in from the field\'s edge',
           r.get('rings') == [1, 0, 1, 1], r.get('rings'))
        ok('the outer 4 rows hold all three; the outer 2 hold one',
           r.get('outer') == 3 and ball_field_census(
               parse_kicad_pcb(board), rings=2)[0]['outer'] == 1, r.get('outer'))

        groups = os.path.join(td, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'groups': [
                {'name': 'all', 'refs': ['U1', 'C1', 'C2', 'C3', 'C4', 'C5']}]}, fh)
        intent = os.path.join(td, 'intent.json')
        with open(intent, 'w', encoding='utf-8') as fh:
            json.dump({'kind': 'floorplan-intent', 'schema': 1}, fh)
        out, js = os.path.join(td, 'out.kicad_pcb'), os.path.join(td, 'r.json')
        res = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), board, out,
                     '--groups', groups, '--intent', intent, '--seed', '0', '--json', js],
                    accept=True)
        evidence(js, 'the place_simple report')
        rep = json.load(open(js))
        ok('the report carries the census', rep.get('bga_field') and
           rep['bga_field'][0]['outer'] == 3, rep.get('bga_field'))
        ok('and the run prints it',
           'U1 (7x7 balls at 1 mm): 3 B.Cu part(s) centred under its ball field, '
           '3 in its outer 4 rows' in res.stdout, res.stdout[-600:])
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
