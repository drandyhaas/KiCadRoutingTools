#!/usr/bin/env python3
"""#1634: the emitter reads an overhang as a DISTANCE, and a part whose 3D
model has body at board depth past its edge is an edge part.

`emit_intent` decided how a part overhanging the outline is declared on
`courtyard_overhang`, a ranking sum over each side and off-board corner the
courtyard crosses. On #1603's fixture (a stock edge-mount SMA, J3, 11.93 mm
past the east edge) a 2 mm chamfer at the FAR corner made it read 35.79,
over the band cap: J3 lost its edge and its #1603 floor ("DAMAGE"), and
place_simple seated it through the board. Named by a part number with no
class, J3 lost them too ("no edge is claimed").

Here, with #1603's synthetic body measurement (no KiCad needed):

  1. the chamfered board's sum reads ~3x the distance (the discriminator);
  2. on the chamfered board J3 keeps its east edge and its floor, uncapped;
  3. named by part number, J3 is given the east edge and the floor by its
     model's body at board depth -- and with no body measured it is not
     (the control), so the edge comes from the model, not from the name.

    python3 tests/test_1634_emit_overhang_distance.py
"""
import os
import re
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
from test_1603_body_in_board import FIXTURE, quiet, synthetic  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def variants(tmp):
    """The fixture with a 2 mm chamfer at the corner farthest from J3, and
    the fixture with J3 named by its part number."""
    with open(FIXTURE, encoding='utf-8', newline='') as f:
        t = f.read()
    a = t.replace('(start 5 5)\n\t\t(end 31 5)', '(start 7 5)\n\t\t(end 31 5)')
    a = a.replace('(start 5 25)\n\t\t(end 5 5)', '(start 5 25)\n\t\t(end 5 7)')
    assert a.count('(start 7 5)') == 1 and a.count('(end 5 7)') == 1
    i = a.index('\t(gr_line')
    j = a.index('\t(gr_line', i + 5)
    c = re.sub(r'\(start [^)]*\)', '(start 5 7)', a[i:j], count=1)
    c = re.sub(r'\(end [^)]*\)', '(end 7 5)', c, count=1)
    c = re.sub(r'\(uuid "[^"]*"\)', '(uuid "00000000-0000-4000-8000-000000001634")', c)
    chamfer = os.path.join(tmp, 'chamfer.kicad_pcb')
    with open(chamfer, 'w', encoding='utf-8', newline='') as f:
        f.write(a[:i] + c + a[i:])
    old = '(footprint "Connector_Coaxial:SMA_Amphenol_132289_EdgeMount"'
    assert t.count(old) == 1
    name = os.path.join(tmp, 'name.kicad_pcb')
    with open(name, 'w', encoding='utf-8', newline='') as f:
        f.write(t.replace(old, '(footprint "RF:132289"', 1))
    return chamfer, name


def emit(path, h):
    from kicad_parser import parse_kicad_pcb
    from placement.floorplan import emit_intent
    pcb = quiet(parse_kicad_pcb, path)
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: h(pcb)
    try:
        doc = quiet(emit_intent, pcb, path)
    finally:
        heights.for_board = real
    return pcb, next((c for c in doc.get('edge_connectors') or ()
                      if c.get('ref') == 'J3'), {})


def floored(row):
    band = row.get('overhang_mm') or {}
    return (row.get('edge') == 'east' and not row.get('overhang_capped')
            and 11.5 < band.get('min', 0) < 11.93
            and 11.93 <= band.get('max', 0) < 13.0)


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    tmp = tempfile.mkdtemp(prefix='t1634_')
    try:
        chamfer, name = variants(tmp)
        for p in (chamfer, name):
            run_utils.evidence(p, os.path.basename(p))
        print('the chamfered outline:')
        pcb, row = emit(chamfer, synthetic)
        from placement.connector_geometry import (
            ConnectorGeometry, courtyard_overhang, courtyard_overhang_past)
        from placement import quench
        st = quiet(quench.QuenchState, pcb, chamfer, 0.2, 0.2)
        geo = ConnectorGeometry(pcb, chamfer)
        rect = st.parts['J3'].rect()
        summed = courtyard_overhang(geo, rect)
        past, _others = courtyard_overhang_past(geo, rect, 'east')
        check('the ranking sum reads ~3x the distance past the edge',
              summed > 3 * past - 0.5 and abs(past - 11.93) < 0.05,
              f'sum {summed:.3f}, past {past:.3f}')
        check('J3 keeps its east edge and its #1603 floor, uncapped', floored(row),
              str({k: row.get(k) for k in ('edge', 'overhang_mm', 'overhang_capped',
                                           'observed_overhang_mm')}))
        print('J3 named by its part number:')
        _pcb, row = emit(name, synthetic)
        check('its model gives it the east edge and the floor', floored(row),
              str({k: row.get(k) for k in ('edge', 'overhang_mm')}))
        check('...and the note says why', '3D model' in (row.get('note') or ''),
              row.get('note'))
        _pcb, row = emit(name, lambda _p: heights.Heights(note='no heights'))
        check('control: with no body measured, the class-less part claims no edge',
              'edge' not in row and (row.get('overhang_mm') or {}).get('min') == 0.0,
              str({k: row.get(k) for k in ('edge', 'overhang_mm')}))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
