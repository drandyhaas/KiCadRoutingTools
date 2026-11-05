#!/usr/bin/env python3
"""#1603, with KiCad: KiCad's own stock SMA_Amphenol_132289_EdgeMount model,
measured by kicad-cli, on the fixture tests/test_1603_body_in_board.py
grades with a synthetic one.

  1. J3 at the designer's pose (its pads end on the east edge, its flange
     and barrel past it): check_assembly reads no body through the board and
     calls the board buildable;
  2. J3 where place_simple seated it from an intent with `overhang_mm.min`
     0 (x 16.53, wholly on the board): NOT BUILDABLE, naming the body
     through the board;
  3. the intent emitted off the designer's board gives J3 a band floor near
     12 mm, not 0, and the emit off the pile, in J3's own frame, the same
     (#1662);
  4. the real model's depth reads the body (#1659): moved in from its drawn
     pose, 0.05 mm does not gate, and 0.15, 0.25, 0.5 and 0.7 mm do;
  5. placed from that intent (the board piled, one seed) at the fab edge floor,
     its pads end level with its body, so that floor stops it short of its band:
     no seat, and the warning names the floor (#1663);
  6. with a `.kicad_dru` rule binding its pads J3 is seated with its body off the
     board: check_floorplan and check_assembly both pass, and J3 stands within
     0.1 mm of the designer's pose.

Needs kicad-cli and KiCad's stock 3D models (placement/heights.py's own
export). Without them it exits 77 with a `SKIP:` line.

    python3 tests/test_1603_body_in_board_kicad.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 900
SKIP_EXIT = 77
FIXTURE = os.path.join(TESTS, 'fixtures', '1603', 'edge_sma.kicad_pcb')
PY = [sys.executable, '-X', 'utf8', '-B']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def tool(*a):
    return PY + [os.path.join(REPO, *a[0].split('/'))] + [str(x) for x in a[1:]]


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    from model_export import resolve_cli
    cli, why = resolve_cli()
    if not cli:
        print(f'SKIP: no kicad-cli ({why})')
        return SKIP_EXIT
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    from placement import heights
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(FIXTURE)
    h = heights.for_board(pcb, FIXTURE)
    if h.height('J3') is None:
        print(f"SKIP: kicad-cli resolved no model of the stock SMA jack J3 ({h.note})")
        return SKIP_EXIT
    tmp = tempfile.mkdtemp(prefix='t1603k_')
    try:
        print('the designer\'s pose:')
        r = run_utils.check(tool('py_tools/check_assembly.py', FIXTURE), accept=True)
        check('buildable, no body through the board',
              'VERDICT: buildable' in r.stdout and 'BODY THROUGH THE BOARD' not in r.stdout,
              ' | '.join(ln.strip() for ln in r.stdout.splitlines() if 'VERDICT' in ln))

        print('wholly on the board, where place_simple seated it from `min 0`:')
        with open(FIXTURE, encoding='utf-8') as f:
            text = f.read()
        placed = os.path.join(tmp, 'placed.kicad_pcb')
        with open(placed, 'w', encoding='utf-8') as f:
            f.write(text.replace('(at 28.46 15)', '(at 16.53 10.2634)', 1))
        r = run_utils.check(tool('py_tools/check_assembly.py', placed),
                            refuse='BODY THROUGH THE BOARD (1 part(s) GATE)', code=4)
        line = next((ln for ln in r.stdout.splitlines() if ln.strip().startswith('J3 ')), '')
        m = re.search(r'J3 ([\d.]+)mm2', line)
        check('NOT BUILDABLE, J3 about 12 mm through the board',
              'VERDICT: NOT BUILDABLE' in r.stdout and m and float(m.group(1)) > 50, line.strip())

        print('the intent emitted off the designer\'s board:')
        intent = os.path.join(tmp, 'intent.json')
        run_utils.check(tool('py_tools/check_floorplan.py', FIXTURE, '--emit-intent', intent),
                        accept=True)
        run_utils.evidence(intent, 'the emitted intent')
        with open(intent, encoding='utf-8') as f:
            doc = json.load(f)
        j3 = next((c for c in doc['edge_connectors'] if c['ref'] == 'J3'), {})
        band = j3.get('overhang_mm') or {}
        check("J3's band floor is near 12 mm, read off its model",
              j3.get('edge') == 'east' and 11.4 < band.get('min', 0) <= 11.93
              and doc['context']['basis'].get('edge_connectors[J3].overhang_mm.min')
              == 'derived_default', str(j3))

        print('moved in from its drawn pose:')
        for inb, gates in ((0.05, False), (0.15, True), (0.25, True), (0.5, True), (0.7, True)):
            c = heights.body_in_board(h, pcb, {'J3': (28.46 - inb, 15.0, 0.0)})
            row = next((r for r in c['rows'] if r['ref'] == 'J3'), {})
            check(f'{inb} mm in ' + ('gates' if gates else 'does not gate'),
                  ('J3' in c['gating_refs']) == gates and abs(row.get('inside_mm', 0) - inb) < 0.01,
                  str(row))

        print('placed from it:')
        pile = os.path.join(tmp, 'pile.kicad_pcb')
        run_utils.check(tool('.claude/skills/plan-pcb-placement/scripts/make_unplaced.py',
                             FIXTURE, pile), accept=True)
        run_utils.evidence(pile, 'the pile')
        emitted = os.path.join(tmp, 'pile_intent.json')
        r = run_utils.check(tool('py_tools/check_floorplan.py', pile, '--emit-intent', emitted,
                                 '--allow-unplaced'), accept=True)
        run_utils.evidence(emitted, 'the intent emitted off the pile')
        with open(emitted, encoding='utf-8') as f:
            row = next((c for c in json.load(f)['edge_connectors'] if c['ref'] == 'J3'), {})
        check('off the pile J3 gets the same band, in its own frame, with no edge',
              row.get('overhang_mm') == band and 'edge' not in row, str(row))
        check('...and an OWED line naming it',
              any(ln.strip().startswith('OWED: J3 ') and 'board depth' in ln
                  for ln in r.stdout.splitlines()), '')
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as f:
            json.dump({'schema': 1, 'groups': [{'name': 'all', 'role': 'core',
                                                'refs': ['C1', 'C2', 'J1', 'J2', 'J3']}]}, f)
        out = os.path.join(tmp, 'floor.kicad_pcb')
        r = run_utils.check(tool('py_placer/place_simple.py', pile, out, '--groups', groups,
                                 '--intent', intent, '--seed', 0), accept=True)
        line = next((ln.strip() for ln in r.stdout.splitlines() if 'WARNING: J3' in ln), '')
        check('at the fab edge floor it has no seat, and the warning names that floor',
              'has no seat there' in line and '0.20mm from the fab edge floor' in line
              and 'Lowering the band will not help' in line, line)

        print('...with a rule binding its pads:')
        ruled = os.path.join(tmp, 'ruled')
        os.makedirs(ruled)
        rule_pile = os.path.join(ruled, 'pile.kicad_pcb')
        run_utils.check(tool('py_router/copy_board.py', pile, rule_pile), accept=True)
        with open(os.path.join(ruled, 'pile.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write('(version 1)\n(rule "edge_launch"\n  (condition "A.Type == \'Pad\' && '
                    'A.memberOfFootprint(\'J3\')")\n  (constraint edge_clearance (min 0mm)))\n')
        pile = rule_pile
        out = os.path.join(ruled, 's0.kicad_pcb')
        r = run_utils.check(tool('py_placer/place_simple.py', pile, out, '--groups', groups,
                                 '--intent', intent, '--seed', 0), accept=True)
        run_utils.evidence(out, 'the placed board')
        lines = [ln.strip() for ln in r.stdout.splitlines()
                 if ln.strip().startswith(('floorplan:', 'assembly:'))]
        check('place_simple grades its own seed floorplan-clean and buildable',
              any(ln.startswith('floorplan: no error') for ln in lines)
              and any(ln.startswith('assembly: buildable') for ln in lines), ' | '.join(lines))
        with contextlib.redirect_stdout(io.StringIO()):
            j = parse_kicad_pcb(out).footprints['J3']
        check("J3 stands within 0.1 mm of the designer's pose, its body off the board",
              abs(j.x - 28.46) <= heights.DEPTH_EDGE_TOL_MM + 0.001 and abs(j.y - 15.0) < 8.0,
              f'({j.x}, {j.y})')
        run_utils.check(tool('py_tools/check_assembly.py', out, '--intent', intent,
                             '--baseline', pile), accept=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
