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
     12 mm, not 0;
  4. placed from that intent (the board piled, one seed), J3 is seated with
     its body off the board: check_floorplan and check_assembly both pass,
     and J3 stands within a cell of the designer's pose.

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

        print('placed from it:')
        pile = os.path.join(tmp, 'pile.kicad_pcb')
        run_utils.check(tool('.claude/skills/plan-pcb-placement/scripts/make_unplaced.py',
                             FIXTURE, pile), accept=True)
        run_utils.evidence(pile, 'the pile')
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as f:
            json.dump({'schema': 1, 'groups': [{'name': 'all', 'role': 'core',
                                                'refs': ['C1', 'C2', 'J1', 'J2', 'J3']}]}, f)
        out = os.path.join(tmp, 's0.kicad_pcb')
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
        check("J3 stands within a cell of the designer's pose, its body off the board",
              abs(j.x - 28.46) <= heights.CELL_MM + 0.01 and abs(j.y - 15.0) < 8.0,
              f'({j.x}, {j.y})')
        run_utils.check(tool('py_tools/check_assembly.py', out, '--intent', intent,
                             '--baseline', pile), accept=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
