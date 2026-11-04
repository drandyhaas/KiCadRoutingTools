#!/usr/bin/env python3
"""#1369: a pour's resolved zone clearance lowers the DRC floor only when the
step draws a pour, and only for a real ball matrix.

Two ways the writeback lowered a floor no shipped copper needed:

  * `route_planes --skip-existing-zones` that KEEPS every zone draws nothing,
    yet its resolved zone clearance (tightened to thread a BGA via lattice) was
    recorded in the clearance ledger and written to the Default class.
  * `detect_package_type` calls every LGA/CSP/WLP a 'BGA' (so their interior
    lands stay clear of routes), and the pour-clearance resolution took each as
    a via-lattice field: an 8-pad perimeter LGA lowered the board's clearance
    (and printed a false "its plane balls are islands" NOTE) for a lattice it
    does not have.

    python3 tests/test_1369_pour_floor_follows_drawn_pour.py
"""
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check, evidence, tool        # noqa: E402

import clearance_ledger                            # noqa: E402
from route_planes import _resolve_zone_clearance   # noqa: E402

BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run32', 'placed_v2.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']


def pad(i, j, pitch):
    return SimpleNamespace(global_x=10 + i * pitch, global_y=10 + j * pitch,
                           local_x=i * pitch, local_y=j * pitch)


def board_with(name, pads, via_at, pitch):
    fp = SimpleNamespace(footprint_name=name, pads=pads, reference='U1')
    vias = [SimpleNamespace(x=via_at[0], y=via_at[1], size=0.25, net_id=2)]
    return SimpleNamespace(
        footprints={'U1': fp}, nets={2: SimpleNamespace(name='GND')},
        vias=vias,
        board_info=SimpleNamespace(
            copper_layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']))


def default_class_clearance(pro):
    with open(evidence(pro, 'project'), encoding='utf-8') as f:
        d = json.load(f)
    for c in d['net_settings']['classes']:
        if c['name'] == 'Default':
            return c['clearance']


def unit_half():
    pitch = 0.65
    # a real 4x4 matrix with a plane-net via in it tightens the pour ...
    matrix = board_with('pkg:BGA-16',
                        [pad(i, j, pitch) for i in range(4) for j in range(4)],
                        (10 + pitch, 10 + pitch), pitch)
    clearance_ledger.reset()
    zc = _resolve_zone_clearance(None, 0.2, 0.15, matrix, 0.4, ['GND'])
    assert zc < 0.2, zc
    # ... and records it unless the caller defers the record
    assert clearance_ledger.effective(0.2) < 0.2
    clearance_ledger.reset()
    zc = _resolve_zone_clearance(None, 0.2, 0.15, matrix, 0.4, ['GND'],
                                 record=False)
    assert zc < 0.2 and abs(clearance_ledger.effective(0.2) - 0.2) < 1e-9
    # a 2x6 perimeter land pattern (an LGA-12) has no interior balls: the same
    # via inside it asks nothing of the pour
    lga = board_with('Package_LGA:LGA-12_2x2mm_P0.5mm',
                     [pad(i, j, pitch) for i in range(6) for j in range(2)],
                     (10 + pitch, 10 + pitch / 2), pitch)
    clearance_ledger.reset()
    zc = _resolve_zone_clearance(None, 0.2, 0.15, lga, 0.4, ['GND'])
    assert abs(zc - 0.2) < 1e-9, f'a perimeter LGA tightened the pour to {zc}'
    assert abs(clearance_ledger.effective(0.2) - 0.2) < 1e-9


def integration_half():
    evidence(BOARD, 'plane board')
    tmp = tempfile.mkdtemp(prefix='t1369_')
    try:
        inb = os.path.join(tmp, 'in.kicad_pcb')
        p1 = os.path.join(tmp, 'p1.kicad_pcb')
        p2 = os.path.join(tmp, 'p2.kicad_pcb')
        check(PY + [tool('copy_board.py'), BOARD, inb], accept=True)
        check(PY + [tool('route_planes.py'), inb, p1, '--nets', 'GND', '+3V3',
                    '--plane-layers', 'In1.Cu', 'In2.Cu', '--zone-clearance',
                    '0.2'], accept=True)
        before = default_class_clearance(os.path.join(tmp, 'p1.kicad_pro'))
        r = check(PY + [tool('route_planes.py'), p1, p2, '--nets', 'GND',
                        '+3V3', '--plane-layers', 'In1.Cu', 'In2.Cu',
                        '--skip-existing-zones'], accept=True)
        assert 'already exists' in r.stdout, 'zones were not kept (broken test)'
        after = default_class_clearance(os.path.join(tmp, 'p2.kicad_pro'))
        assert before == after, (
            f'a pour that kept every zone moved the Default class '
            f'{before} -> {after}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    unit_half()
    print('unit half ok')
    integration_half()
    print('PASS')
