#!/usr/bin/env python3
"""#1771: edge_closeup says when a part read on its nearest edge is in a corner.

A bare `REF` is rendered from the edge its pads are nearest. tigard's J7 (a
1.00 mm side-entry header, mouth south) traded into the south-west corner
sits 0.255 mm from west and 0.400 mm from south, so it was shot side-on from
the west with nothing in the line but `(nearest)`. Now a part whose
next-nearest edge is within CORNER_MARGIN_MM of its nearest says
`corner: also N mm from <edge>` (`corner` in the JSON rows), and a named edge
(`J7:south`, what plan-pcb-placement's edge step now passes) says nothing.

Cases:
  1. (no KiCad) corner_edge on rects beside one edge, in a corner, and just
     past the margin;
  2. (kicad-cli) the corner J7, bare and named, and J7 where tigard has it
     (beside the south edge only). Exits 77 (a skip, not a pass) without
     kicad-cli.

    python3 -X utf8 tests/test_1771_edge_closeup_corner.py
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_tools', 'py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                              # noqa: E402

RUN_ALL_TIMEOUT = 600
BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
TOOL = os.path.join(ROOT, 'py_tools', 'edge_closeup.py')
COPY = os.path.join(ROOT, 'py_router', 'copy_board.py')
POSE = run_utils.tool('place_pose.py')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def unit_rows():
    import edge_closeup as E
    b = (0.0, 0.0, 100.0, 50.0)
    check('a part beside one edge is no corner',
          E.corner_edge((40, 45, 60, 49), b) is None)
    got = E.corner_edge((0.255, 45, 5, 49.6), b)
    check('the issue\'s corner J7: nearest west, also 0.40 mm from south',
          E.nearest_edge((0.255, 45, 5, 49.6), b) == 'west'
          and got is not None and got[0] == 'south' and abs(got[1] - 0.4) < 1e-9,
          str(got))
    check('a next-nearest edge just past the margin is not a corner',
          E.corner_edge((0.2, 45, 5, 50 - 0.2 - E.CORNER_MARGIN_MM - 0.01), b) is None)
    check('...and one just inside it is',
          E.corner_edge((0.2, 45, 5, 50 - 0.2 - E.CORNER_MARGIN_MM + 0.01), b) is not None)


def _cli():
    from kicad_oracle import find_kicad_cli
    return find_kicad_cli(warn=False)


def render_rows():
    run_utils.evidence(BOARD, 'tracked board')
    with tempfile.TemporaryDirectory() as td:
        t = os.path.join(td, 't.kicad_pcb')
        c = os.path.join(td, 'c.kicad_pcb')
        subprocess.run([sys.executable, '-X', 'utf8', COPY, BOARD, t],
                       capture_output=True, cwd=ROOT)
        run_utils.evidence(t, 'tigard copy')
        # J7 trades places with H3, the south-west corner's M3 hole (the #1771 rig)
        run_utils.check([sys.executable, '-X', 'utf8', POSE, t, c,
                         'set', 'H3', '39.795', '72.825',
                         'set', 'J7', '33.655', '72.825', '--rot', '0',
                         '--allow-routed'], accept=True)
        run_utils.evidence(c, 'the corner board')
        js = os.path.join(td, 'rows.json')
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, c, 'J7', 'J7:south',
                             '--out-dir', os.path.join(td, 'e'), '--json', js],
                            accept=True)
        out = r.stdout
        rows = json.load(open(run_utils.evidence(js, 'rows'), encoding='utf-8'))
        bare, named = rows[0], rows[1]
        check('bare J7 in the corner: read west, says it is a corner by south',
              bare['edge'] == 'west' and (bare.get('corner') or {}).get('edge') == 'south',
              str(bare.get('corner')))
        check('...on the console too',
              'corner: also 0.40 mm from south' in out, out[-600:])
        check('named J7:south says no corner', named.get('corner') is None,
              str(named.get('corner')))
        js2 = os.path.join(td, 'rows2.json')
        run_utils.check([sys.executable, '-X', 'utf8', TOOL, t, 'J7',
                         '--out-dir', os.path.join(td, 'e2'), '--json', js2],
                        accept=True)
        row = json.load(open(run_utils.evidence(js2, 'rows'), encoding='utf-8'))[0]
        check('J7 where tigard has it (south edge only): no corner',
              row['edge'] == 'south' and row.get('corner') is None, str(row))


def main():
    unit_rows()
    if not _cli():
        print('SKIP: kicad-cli not found; case 2 not run')
        return 77 if not FAILS else 1
    render_rows()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
