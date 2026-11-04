#!/usr/bin/env python3
"""#1635: a thin slice of a body through the board gates by its length.

`body_in_board` gated from 4 mm^2 of a part's 3D model at board depth inside
the outline. #1603's stock edge-mount SMA jack moved 0.7 mm in from its drawn
pose stands 0.74 mm inside the edge across 6.5 mm and read 2.0 mm^2 -- "a
model drawn low, never gates", buildable -- while 0.1 mm further in it read
4.5 mm^2 and was NOT BUILDABLE. The gate now also reads the slice's LENGTH
(`DEPTH_MIN_SPAN_MM`, the longest side of its box in the part's own frame),
and a row under the gate says what it measured, not a cause.

With #1603's synthetic body measurement (no KiCad needed):

  1. the jack 0.25, 0.5 and 0.7 mm in gates, each under 4 mm^2, by its span;
  2. at its drawn pose it reads nothing;
  3. a 1 x 1 mm and a 1 x 2.5 mm model drawn low (the longest the tracked
     boards have) are listed and do not gate; a 5.25 x 0.5 mm lip does;
  4. `depth_past_edge` gives a floor for a long thin body under 4 mm^2;
  5. check_assembly prints the span and the depth, never "drawn low".

    python3 tests/test_1635_body_in_board_span.py
"""
import contextlib
import io
import os
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
from test_1603_body_in_board import (DESIGNED, FIXTURE, T, assembly, at,  # noqa: E402
                                     box, low, quiet, synthetic)

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, FIXTURE)
    h = synthetic(pcb)
    print('the jack moved in from its drawn pose:')
    for inb in (0.25, 0.5, 0.7):
        c = heights.body_in_board(h, pcb, {'J3': (DESIGNED[0] - inb, DESIGNED[1], 0.0)})
        row = next((r for r in c['rows'] if r['ref'] == 'J3'), {})
        check(f'{inb} mm in: under {heights.DEPTH_MIN_MM2:g} mm^2 and gates by its span',
              row.get('area_mm2', 99) < heights.DEPTH_MIN_MM2
              and row.get('span_mm', 0) >= heights.DEPTH_MIN_SPAN_MM
              and 'J3' in c['gating_refs']
              and abs(row.get('inside_mm', 0) - inb) < heights.CELL_MM + 0.05, str(row))
    c = heights.body_in_board(h, pcb)
    check('at its drawn pose it reads nothing', not c['rows'], str(c['rows']))

    print('small bodies drawn low, and a long thin lip:')
    extra = {'C1': heights._depth_raster(low(box(1.0, 2.0, 1.0, 2.0, -1.2, 0.5)), T),
             'C2': heights._depth_raster(low(box(-0.5, 0.5, -1.25, 1.25, -1.2, 0.5)), T)}
    c = heights.body_in_board(synthetic(pcb, extra=extra), pcb)
    rows = {r['ref']: r for r in c['rows']}
    check('a 1 x 1 mm body is listed and does not gate',
          'C1' in rows and not rows['C1']['gates'], str(rows.get('C1')))
    check('a 1 x 2.5 mm body is listed and does not gate',
          'C2' in rows and 2.5 <= rows['C2']['span_mm'] < heights.DEPTH_MIN_SPAN_MM
          and not rows['C2']['gates'], str(rows.get('C2')))
    lip = {'C1': heights._depth_raster(low(box(-2.625, 2.625, -0.25, 0.25, -1.2, 0.5)), T)}
    hl = synthetic(pcb, extra=lip)
    c = heights.body_in_board(hl, pcb)
    rows = {r['ref']: r for r in c['rows']}
    check('a 5.25 x 0.5 mm lip inside the outline gates',
          'C1' in rows and rows['C1']['area_mm2'] < heights.DEPTH_MIN_MM2
          and rows['C1']['gates'], str(rows.get('C1')))
    check('...and depth_past_edge gives it a floor', heights.depth_past_edge(
        hl, pcb, 'C1', 'east') is not None)

    print('check_assembly:')
    tmp = tempfile.mkdtemp(prefix='t1635_')
    try:
        path = at(os.path.join(tmp, 'in07.kicad_pcb'), (DESIGNED[0] - 0.7, DESIGNED[1], 0.0))
        hp = synthetic(quiet(parse_kicad_pcb, path))
        rc, doc, text = assembly(path, hp, tmp)
        line = next((ln for ln in text.splitlines() if ln.strip().startswith('J3 ')), '')
        check('the jack 0.7 mm in is NOT BUILDABLE', rc == 4 and doc['buildable'] is False
              and doc['body_in_board_count'] == 1, f"rc {rc}")
        check('...and its row says what was measured',
              'mm inside the outline' in line and 'drawn low' not in text, line.strip())
        path = at(os.path.join(tmp, 'c1.kicad_pcb'), DESIGNED)
        rc, doc, text = assembly(path, synthetic(quiet(parse_kicad_pcb, path), extra=extra),
                                 tmp)
        line = next((ln for ln in text.splitlines() if ln.strip().startswith('C1 ')), '')
        check('a row under the gate says so without naming a cause',
              rc == 0 and 'does not gate' in line and 'drawn low' not in line, line.strip())
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    with contextlib.redirect_stderr(io.StringIO()):
        rc = main()
    sys.exit(rc)
