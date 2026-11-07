#!/usr/bin/env python3
"""#1720: cover_facts reports the air gaps inside a connector's own model.

A cover's `underside_mm` is the socket's card height less what hangs below
the card, and when the socket's drawing cannot be had the only source left
on the board is the socket's 3D model. cover_facts reported a connector's
top and the LOWEST point of its raised models per raster cell, and the
raster holds one value per cell, so a slot -- air between plastic below and
plastic above -- was in neither. `heights._gap_raster` casts a vertical ray
at each cell through the model's surfaces (one facing up ends solid, one
facing down starts it) and keeps each cell's lowest air gap; cover_facts
groups those cells by floor and ceiling and prints each group as the
model's own geometry.

Cases:
  1. (no KiCad) a lower housing with an upper housing over half its width
     and 0.8 mm of air between: the gap is read at floor 1.0, ceiling 1.8,
     over exactly the upper housing's 10 x 2 mm, and one group; a single
     solid box, and two boxes stacked with no air, read no gap;
  2. (kicad-cli) a side-entry header's stock model, whose mouth is a cavity
     under its top wall: cover_facts lists a gap in its model, ceiling over
     floor, inside the part's own frame, in the text and the JSON.

Case 2 exits 77 (a skip, not a pass) when kicad-cli is absent.

    python3 tests/test_1720_model_gaps.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_tools', 'py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import numpy as np                                            # noqa: E402

import run_utils                                              # noqa: E402

RUN_ALL_TIMEOUT = 600
BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
REF = 'J7'                     # a side-entry header: its mouth is a cavity
TOOL = os.path.join(ROOT, 'py_tools', 'cover_facts.py')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _box(x0, y0, z0, x1, y1, z1):
    """An axis-aligned box's 12 triangles and each one's outward normal's z
    (the `q`, `nh` that heights' measurement hands its rasters)."""
    v = np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1)
                  for x in (x0, x1)], float)
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3),
             (0, 4, 6, 2), (1, 3, 7, 5)]
    t = v[np.array([tri for a, b, c, d in quads
                    for tri in ((a, b, c), (a, c, d))])]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1)[:, None]
    centre = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    n[np.einsum('ij,ij->i', n, t.mean(1) - centre) < 0] *= -1
    return t, n[:, 2]


def _model(*boxes):
    parts = [_box(*b) for b in boxes]
    return (np.concatenate([p[0] for p in parts]),
            np.concatenate([p[1] for p in parts]))


def test_synthetic_slot():
    from placement.heights import _gap_raster
    import cover_facts as CF
    q, nh = _model((0, 0, 0, 10, 4, 1.0), (0, 0, 1.8, 10, 2, 3.0))
    raster = _gap_raster(q, nh)
    rows = CF._gaps(raster) if raster else []
    check('a slot between two housings is one gap group', len(rows) == 1,
          rows)
    if rows:
        r = rows[0]
        check('its floor and ceiling are the housings\' faces',
              (r['floor_mm'], r['ceiling_mm'], r['gap_mm'])
              == (1.0, 1.8, 0.8), r)
        check('over exactly the upper housing (10 x 2 mm)',
              r['area_mm2'] == 20.0
              and r['extent_local'] == [0.0, 0.0, 10.0, 2.0], r)
    q, nh = _model((0, 0, 0, 10, 4, 1.0))
    check('one solid box has no gap', _gap_raster(q, nh) is None)
    q, nh = _model((0, 0, 0, 10, 4, 1.0), (0, 0, 1.0, 10, 4, 3.0))
    check('two boxes stacked with no air have no gap',
          _gap_raster(q, nh) is None)


def test_side_entry_mouth():
    from kicad_oracle import find_kicad_cli
    if not find_kicad_cli(warn=False):
        print('SKIP: kicad-cli not found')
        sys.exit(77)
    run_utils.evidence(BOARD, 'tracked board')
    with tempfile.TemporaryDirectory() as tmp:
        js = os.path.join(tmp, 'facts.json')
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, BOARD,
                             '--refs', REF, '--json', js], accept=True)
        with open(run_utils.evidence(js, 'facts'), encoding='utf-8') as fh:
            (row,) = json.load(fh)['connectors']
        gaps = row.get('model_gaps') or []
        check('the header\'s model has a gap', bool(gaps), row)
        if gaps:
            g = gaps[0]
            check('ceiling over floor, both above the face, under the '
                  'part\'s top',
                  0 < g['floor_mm'] < g['ceiling_mm'] <= row['height_mm'],
                  (g, row['height_mm']))
            check('the text says it is the model\'s',
                  "gap in its model (the model's, not the datasheet's)"
                  in r.stdout, r.stdout[-800:])


def main():
    for t in (test_synthetic_slot, test_side_entry_mouth):
        print(t.__name__)
        t()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASSED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
