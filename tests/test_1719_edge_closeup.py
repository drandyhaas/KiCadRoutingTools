#!/usr/bin/env python3
"""#1719: an edge part's mouth is rendered close up, at a stated scale.

plan-pcb-placement's edge check judged which way each edge connector opens
from one whole-board `kicad-cli pcb render` per edge. That frames the whole
board, so on a ~155 mm board the scale is ~5.6 px/mm and a 1.00 mm
side-entry header's mouth-out and mouth-in renders look the same.
`py_tools/edge_closeup.py` renders each named part centred, from outside the
edge its pads are nearest, at a stated scale (default 40 px/mm), measuring
kicad-cli's zoom-1 scale rather than assuming it.

Cases:
  1. (no KiCad) the edge a part is read on, and the refusals: an unknown
     edge name and an unknown reference exit 2 for that reason;
  2. (kicad-cli) a 1.00 mm side-entry header at a board's south edge, mouth
     out, and the same board with that header turned 180 degrees (mouth into
     the board): each render is taken at the stated 40 px/mm, the zoom
     follows from the measured zoom-1 scale, its model resolves, and the two
     renders differ around the part by more than a few specks (the contacts
     in its opening against a plain back wall).

Case 2 exits 77 (a skip, not a pass) when kicad-cli is absent.

    python3 tests/test_1719_edge_closeup.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_tools', 'py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                              # noqa: E402

RUN_ALL_TIMEOUT = 600
BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
REF = 'J7'                  # a 4-pin 1.00 mm side-entry header, south edge
TOOL = os.path.join(ROOT, 'py_tools', 'edge_closeup.py')
COPY = os.path.join(ROOT, 'py_router', 'copy_board.py')
POSE = run_utils.tool('place_pose.py')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def test_edges_and_refusals():
    import edge_closeup as E
    b = (0.0, 0.0, 100.0, 50.0)
    check('a part by the south edge reads south, and so on',
          [E.nearest_edge(r, b) for r in ((40, 45, 60, 49), (40, 1, 60, 5),
                                          (1, 20, 5, 30), (95, 20, 99, 30))]
          == ['south', 'north', 'west', 'east'])
    check('each edge is rendered from the side standing outside it',
          E.SIDES == {'north': 'back', 'south': 'front', 'west': 'left',
                      'east': 'right'}, E.SIDES)
    run_utils.evidence(BOARD, 'tracked board')
    with tempfile.TemporaryDirectory() as tmp:
        run_utils.check([sys.executable, TOOL, BOARD, f'{REF}:up',
                         '--out-dir', tmp],
                        refuse='the edge is one of', code=2)
        run_utils.check([sys.executable, TOOL, BOARD, 'NOPE99',
                         '--out-dir', tmp],
                        refuse='NOPE99: no such part', code=2)
    check('an unknown edge and an unknown part are refused for that reason',
          True)


def _diff_count(a, b, half=160):
    """Pixels that differ in a window around the image centre (the part)."""
    import numpy as np
    from PIL import Image
    x = np.asarray(Image.open(a).convert('RGB')).astype(int)
    y = np.asarray(Image.open(b).convert('RGB')).astype(int)
    h, w = x.shape[:2]
    win = (slice(h // 2 - half, h // 2 + half),
           slice(w // 2 - half, w // 2 + half))
    d = np.abs(x[win] - y[win]).sum(axis=2) > 30
    return int(d.sum())


def test_mouth_out_and_in_differ_at_the_stated_scale():
    from kicad_oracle import find_kicad_cli
    if not find_kicad_cli(warn=False):
        print('SKIP: kicad-cli not found')
        sys.exit(77)
    with tempfile.TemporaryDirectory() as tmp:
        out_b = os.path.join(tmp, 'out.kicad_pcb')
        in_b = os.path.join(tmp, 'in.kicad_pcb')
        run_utils.check([sys.executable, COPY, BOARD, out_b], accept=True)
        run_utils.check([sys.executable, POSE, out_b, in_b, 'rotate', REF,
                         '180', '--allow-routed', '--force'], accept=True)
        rows, pngs = {}, {}
        for tag, board in (('out', out_b), ('in', in_b)):
            for scale in (40.0,):
                d = os.path.join(tmp, f'{tag}_{scale:g}')
                js = os.path.join(tmp, f'{tag}_{scale:g}.json')
                run_utils.check([sys.executable, TOOL, board, REF,
                                 '--out-dir', d, '--px-per-mm', str(scale),
                                 '--json', js], accept=True)
                with open(js, encoding='utf-8') as fh:
                    (row,) = json.load(fh)
                rows[tag, scale] = row
                pngs[tag, scale] = run_utils.evidence(row['png'], 'render')
        r = rows['out', 40.0]
        check('read on the south edge, rendered from the front',
              (r['edge'], r['side']) == ('south', 'front'), r)
        check('rendered at the stated 40 px/mm, the zoom following the '
              'measured zoom-1 scale',
              r['px_per_mm'] == 40.0 and 10 < r['px_per_mm_at_zoom1'] < 30
              and abs(r['zoom'] * r['px_per_mm_at_zoom1'] - 40.0) < 0.1, r)
        check('its model resolves, so the render shows its body',
              r['model'] == 'resolved', r)
        near = _diff_count(pngs['out', 40.0], pngs['in', 40.0])
        check('mouth out and mouth in differ plainly around the part',
              near >= 400, near)


def main():
    for t in (test_edges_and_refusals,
              test_mouth_out_and_in_differ_at_the_stated_scale):
        print(t.__name__)
        t()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASSED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
