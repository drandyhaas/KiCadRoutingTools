"""#2029: which pins of a part cannot fan out to a via, and what walls them in.

U1 is a row of four pads (nets S1..S4, each also on X1 far away). F1 is a
fence of another net's F.Cu copper round it, closed on three sides 0.6 mm from
the pads. On the fourth (north) side the pocket is `depth` mm deep:

    0.6  no 0.6 mm via fits anywhere a track can reach: every pin CAGED,
         walled by F1.
    1.8  one row of two via sites fits: each pin reaches a site alone (not
         caged), but two are CROWDED when all fan out at once.
    none the fence removed: every pin escapes, exit 0.

A 4-layer board with a GND pour over In1.Cu leaves In1.Cu out of the default
layers, as `--layer-costs` would ban it.

    python3 tests/test_2029_fanout_check.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from check_fanout import check_fanout                         # noqa: E402
from run_utils import check                                   # noqa: E402

FAILS = []
CLI = os.path.join(ROOT, 'py_tools', 'check_fanout.py')
PRO = {'net_settings': {'classes': [{'name': 'Default', 'track_width': 0.2,
                                     'clearance': 0.2, 'via_diameter': 0.6,
                                     'via_drill': 0.3}]}}


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def board(path, depth=None, inner=False):
    """U1's pads 0.25 x 0.6 at x 10, 10.5, 11, 11.5, y 10 on F.Cu; F1's
    0.4 mm walls 0.6 mm west, east and south of them and `depth` mm north
    (None: no fence). `inner`: In1.Cu and In2.Cu, a GND pour over In1.Cu."""
    nets = ''.join(f'  (net {k} "S{k}")\n' for k in range(1, 5))
    nets += '  (net 5 "FENCE")\n  (net 6 "GND")\n'
    upads = ''.join(f'    (pad "{k}" smd rect (at {(k - 1) * 0.5:g} 0) (size 0.25 0.6)'
                    f' (layers "F.Cu") (net {k} "S{k}"))\n' for k in range(1, 5))
    xpads = ''.join(f'    (pad "{k}" smd rect (at 0 {k - 1}) (size 0.3 0.3)'
                    f' (layers "F.Cu") (net {k} "S{k}"))\n' for k in range(1, 5))
    fence = ''
    if depth is not None:
        t = 0.4
        xl, xr = 9.875 - 0.6 - t / 2, 11.625 + 0.6 + t / 2
        yt, yb = 9.7 - depth - t / 2, 10.3 + 0.6 + t / 2
        walls = [((xl + xr) / 2, yt, xr - xl + t, t), ((xl + xr) / 2, yb, xr - xl + t, t),
                 (xl, (yt + yb) / 2, t, yb - yt), (xr, (yt + yb) / 2, t, yb - yt)]
        fence = ('  (footprint "t:FENCE" (layer "F.Cu") (at 0 0)\n'
                 '    (property "Reference" "F1" (at 0 0) (layer "F.SilkS"))\n'
                 + ''.join(f'    (pad "W{i}" smd rect (at {x:g} {y:g}) (size {w:g} {h:g})'
                           f' (layers "F.Cu") (net 5 "FENCE"))\n'
                           for i, (x, y, w, h) in enumerate(walls)) + '  )\n')
    layers = ('(0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
              '(31 "B.Cu" signal)' if inner else '(0 "F.Cu" signal) (31 "B.Cu" signal)')
    pour = ('  (zone (net 6) (net_name "GND") (layer "In1.Cu") (hatch edge 0.5)\n'
            '    (polygon (pts (xy 0 0) (xy 40 0) (xy 40 30) (xy 0 30))))\n'
            if inner else '')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 f'  (layers {layers} (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n' + nets +
                 '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))'
                 ' (layer "Edge.Cuts"))\n'
                 '  (footprint "t:ROW" (layer "F.Cu") (at 10 10)\n'
                 '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
                 + upads + '  )\n'
                 '  (footprint "t:FAR" (layer "F.Cu") (at 30 20)\n'
                 '    (property "Reference" "X1" (at 0 0) (layer "F.SilkS"))\n'
                 + xpads + '  )\n' + fence + pour + ')\n')
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    return path


def verdicts(r):
    return [p['verdict'] for p in r['pins']]


def main(td):
    caged = board(os.path.join(td, 'caged.kicad_pcb'), depth=0.6)
    r = check_fanout(caged, 'U1')
    ok('a pocket no via fits in: every pin caged', verdicts(r) == ['caged'] * 4, r['pins'])
    ok('and walled by the fence', all(p['walls'] == ['F1'] for p in r['pins']), r['pins'])
    check([sys.executable, '-X', 'utf8', CLI, caged, '-c', 'U1'],
          refuse='walled by F1', code=1)
    print('  PASS  the CLI exits 1 naming the wall')

    band = board(os.path.join(td, 'band.kicad_pcb'), depth=1.8)
    r = check_fanout(band, 'U1')
    v = verdicts(r)
    ok('a band with two via sites for four pins: none caged, two crowded',
       sorted(v) == ['crowded', 'crowded', 'escapes', 'escapes'], v)
    g = r['groups']
    ok('one group of all four, two fit, the fence named',
       len(g) == 1 and len(g[0]['pins']) == 4 and g[0]['exits'] == 2
       and g[0]['walls'][:1] == ['F1'] and not g[0]['window_edge'], g)

    free = board(os.path.join(td, 'free.kicad_pcb'))
    r = check_fanout(free, 'U1')
    ok('the fence removed: every pin escapes', verdicts(r) == ['escapes'] * 4, r['pins'])
    res = check([sys.executable, '-X', 'utf8', CLI, free, '-c', 'U1', '--json'],
                accept=True)
    d = json.loads(res.stdout)
    ok('--json carries counts, pins and layers',
       d['counts'] == {'escapes': 4} and d['layers'] == ['F.Cu', 'B.Cu'], d['counts'])

    four = board(os.path.join(td, 'four.kicad_pcb'), inner=True)
    r = check_fanout(four, 'U1')
    ok('a GND pour over In1.Cu leaves that layer out by default',
       r['layers'] == ['F.Cu', 'In2.Cu', 'B.Cu'] and r['planes'] == {'In1.Cu': ['GND']},
       (r['layers'], r['planes']))

    check([sys.executable, '-X', 'utf8', CLI, free, '-c', 'NOSUCH'],
          refuse="no part 'NOSUCH' on this board", code=2)
    check([sys.executable, '-X', 'utf8', CLI, free, '-c', 'U1', '--layers', 'F.Cu'],
          refuse='needs two allowed layers', code=2)
    check([sys.executable, '-X', 'utf8', CLI, free, '-c', 'U1', '--layers', 'B.Cu', 'In1.Cu'],
          refuse='not copper layers of this board: In1.Cu', code=2)
    check([sys.executable, '-X', 'utf8', CLI, free, '-c', 'U1', '--layers', 'B.Cu', 'F.Cu'],
          accept=True)
    print('  PASS  usage errors exit 2 with their reason')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as td:
        main(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
