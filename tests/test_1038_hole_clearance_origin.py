#!/usr/bin/env python3
"""#1038: check_drc's AUTOMATIC copper-to-hole floor reads the floor the board
DECLARED, not only the project's current (relaxed) rule.

A pour/repair DRC writeback clamps `rules.min_hole_clearance` DOWN to the
clearance a step routed at (run 32: 0.25 -> 0.1), while route.py keeps
routing to the ORIGINAL 0.25 (`fab_floor_origin`, via
obstacle_map.resolve_hole_clearance) and says so on every call. check_drc's
`--hole-clearance 0` ("auto") read only the rules, so a board violating its
own declared hole clearance graded clean by default.

Synthetic (always runs): an NPTH hole with a track 0.22 mm off its wall,
graded at --clearance 0.1:
  * project rules 0.1 + fab_floor_origin 0.25 -> TRACK-HOLE, graded_at says
    hole_clearance 0.25 from fab_floor_origin;
  * the same board with NO origin -> clean at the 0.20 fab floor (control:
    the origin, and nothing else, is what raised it).
Repro (self-skips with 77 when wk/run32 is absent): routed_c3 now reports
J5's two TRACK-HOLE items by default.

    python3 tests/test_1038_hole_clearance_origin.py
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from run_utils import check, evidence  # noqa: E402

CHECK_DRC = os.path.join(ROOT, 'py_router', 'check_drc.py')
RUN32 = os.path.join(ROOT, 'wk', 'run32', 'routed_c3.kicad_pcb')

fails = []


def ok(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def _edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'(layer "Edge.Cuts") (width 0.1))' for i in range(4))


# NPTH hole at (15,15), drill 2.1 -> wall at r=1.05. Track width 0.2 whose
# edge sits 0.22 mm off the wall: clean at the 0.20 fab floor, 0.03 short of
# a declared 0.25.
Y = 15 + 1.05 + 0.22 + 0.1
BOARD = f'''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "SIG")
{_edge(0, 0, 30, 30)}
 (footprint "t:mh" (layer "F.Cu") (at 15 15)
  (property "Reference" "H1" (at 0 -2) (layer "F.SilkS"))
  (pad "" np_thru_hole circle (at 0 0) (size 2.1 2.1) (drill 2.1)
       (layers "F&B.Cu" "*.Mask"))
 )
 (segment (start 10 {Y}) (end 20 {Y}) (width 0.2) (layer "F.Cu") (net 1) (uuid "s"))
)'''


def _stage(tmp, name, origin):
    d = os.path.join(tmp, name)
    os.makedirs(d)
    pcb = os.path.join(d, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    proj = {'board': {'design_settings': {'rules': {'min_hole_clearance': 0.1}}}}
    if origin is not None:
        proj['kicad_routing_tools'] = {
            'fab_floor_origin': {'min_hole_clearance': origin}}
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(proj, f)
    return pcb


def _grade(pcb, tmp, tag, violating):
    out = os.path.join(tmp, f'{tag}.json')
    argv = [sys.executable, '-X', 'utf8', CHECK_DRC, pcb, '--clearance', '0.1',
            '--json', out]
    if violating:
        r = check(argv, refuse='TRACK-HOLE', code=1)
    else:
        r = check(argv, accept=True)
    evidence(out, f'{tag} check_drc --json')
    with open(out, encoding='utf-8') as f:
        return json.load(f), r


def main():
    with tempfile.TemporaryDirectory() as tmp:
        doc, _r = _grade(_stage(tmp, 'origin', 0.25), tmp, 'origin', True)
        ga = doc.get('graded_at', {})
        ok('origin 0.25 -> the 0.22 band is a track-hole violation',
           doc.get('by_type', {}).get('track-hole', 0) == 1, doc.get('by_type'))
        ok('graded_at.hole_clearance is the declared 0.25',
           abs(ga.get('hole_clearance', 0) - 0.25) < 1e-9, ga)
        ok('graded_at.hole_clearance_source names fab_floor_origin',
           ga.get('hole_clearance_source') == 'fab_floor_origin', ga)

        doc, _r = _grade(_stage(tmp, 'plain', None), tmp, 'plain', False)
        ga = doc.get('graded_at', {})
        ok('control: no origin -> clean at the 0.20 fab floor',
           doc.get('violations') == 0, doc.get('by_type'))
        ok('control: graded_at says 0.20 from the fab floor',
           abs(ga.get('hole_clearance', 0) - 0.20) < 1e-9
           and ga.get('hole_clearance_source') == 'fab floor', ga)

    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1

    # The issue's own repro, when the run-32 assets are staged.
    if not os.path.isfile(RUN32):
        print('SKIP: wk/run32/routed_c3.kicad_pcb absent -- the synthetic '
              'checks above passed; the repro was not run')
        return 77
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'c3.json')
        check([sys.executable, '-X', 'utf8', CHECK_DRC, RUN32,
               '--clearance', '0.1', '--json', out],
              refuse='TRACK-HOLE', code=1)
        evidence(out, 'routed_c3 check_drc --json')
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
        ok('routed_c3 default grade: 2 track-hole (J5)',
           doc.get('by_type', {}).get('track-hole') == 2, doc.get('by_type'))
        ok('routed_c3 graded at the declared 0.25 from fab_floor_origin',
           doc['graded_at'].get('hole_clearance_source') == 'fab_floor_origin'
           and abs(doc['graded_at'].get('hole_clearance', 0) - 0.25) < 1e-9,
           doc['graded_at'])
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
