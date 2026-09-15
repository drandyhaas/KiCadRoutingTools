#!/usr/bin/env python3
"""Run-26 body seating regression, updated to the #961 geometry contract.

Original fixtures stay intact; body measurement now applies to every explicit
edge entry and missing geometry is unmeasured. Class alone imposes no seating.
"""

import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan as fp  # noqa: E402

RUN_ALL_FAST_OK = True

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')

#: A 20 x 10 board; U1 at (2, 5) with two pads 1.6 mm inboard of the west edge
#: and a drawn body (on the layer the test picks) reaching the edge exactly.
SYNTH = '''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "/A")
 (gr_rect (start 0 0) (end 20 10) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "test:RCPT" (layer "F.Cu") (uuid "fp-u1") (at 2 5)
   (property "Reference" "U1" (at 0 0 0))
%s   (pad "1" smd rect (at -0.1 -1) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A") (uuid "p1"))
   (pad "2" smd rect (at -0.1 1) (size 0.6 0.6) (layers "F.Cu") (net 0 "") (uuid "p2"))
 )
)
'''
BODY = '   (fp_rect (start -2 -3) (end 1.5 3) (stroke (width 0.1) (type solid)) (fill no) (layer "%s") (uuid "b1"))\n'


def _grade(board, entry):
    intent = fp.intent_from_dict({'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND,
                                  'units': 'mm', 'edge_connectors': [entry]})
    return fp.grade(intent, parse_kicad_pcb(board), board)


def _seat(result, ref):
    return [v for v in result.violations
            if v.rule == 'edge_connector' and v.ref == ref
            and 'edge_clearance_mm' in v.measured]


def _synth(body_layer):
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w', encoding='utf-8') as fh:
        fh.write(SYNTH % (BODY % body_layer if body_layer else ''))
    return path


def main():
    # #961 supersedes the old class-dependent pad/courtyard fallback.
    # Keep the original public and synthetic fixtures, change the expectation.
    base = {'ref': 'USB1', 'edge': 'west', 'overhang_mm': {'min': 0., 'max': .65}}
    for extra in ({}, {'class': 'edge_receptacle'},
                  {'context': {'mount_mode': 'edge_mount'}}, {'max_setback_mm': .5}):
        result = _grade(ESP, dict(base, **extra))
        row = result.edge_seating[0]
        assert row['body_overhang_basis'] == 'F.Fab', row
        assert row['body_overhang_mm'] == row['body_setback_mm'] == 0., row
        assert not [v for v in result.errors if v.rule == 'edge_connector'], result.errors
    # Naming an edge does not mandate flush seating or nearest-edge identity.
    east = _grade(ESP, dict(base, edge='east'))
    row = east.edge_seating[0]
    assert row['body_overhang_mm'] == 0 and row['body_setback_mm'] > 20, row
    assert row['measurements']['body_setback']['disposition'] == 'not_declared'
    seat = _grade(ESP, dict(base, edge='east', max_setback_mm=.5))
    assert seat.edge_seating[0]['measurements']['body_setback']['disposition'] == 'fail'
    assert any('body_setback_mm' in v.measured for v in seat.errors), seat.errors
    for layer in (None, 'F.SilkS', 'F.Fab'):
        path = _synth(layer)
        try:
            entry = dict(base, ref='U1', max_setback_mm=.5)
            result = _grade(path, entry)
            row = result.edge_seating[0]
            if layer is None:
                assert row['body_overhang_mm'] is None and not result.complete
                assert row['measurements']['body_setback']['disposition'] == 'unmeasured'
                assert any('unmeasured' in v.message for v in result.errors)
            else:
                assert row['body_overhang_basis'] == layer, row
                assert row['body_overhang_mm'] == row['body_setback_mm'] == 0., row
                assert row['measurements']['body_setback']['disposition'] == 'pass'
        finally:
            os.unlink(path)
    print('ALL PASS: drawn body for every entry, explicit seating, no pad fallback')
    return 0


if __name__ == '__main__':
    sys.exit(main())
