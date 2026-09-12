#!/usr/bin/env python3
"""The edge-connector SEAT is measured on the drawn body, not the pad box.

`rule_edge_connector`'s seat conjunct asked "is this part's mating face at the
edge?" and answered it with the courtyard rect -- on a courtyard-less library
the pad bounding box. A receptacle's pads sit well inboard of its opening by
construction, so run 26 graded a micro-USB socket whose fab body is flush
with the west edge as "seated 1.30mm from the nearest edge with no overhang"
and had to waive the brief's four USB1 clauses by name. Measured on the
tracked esp_prog fixture (same pose): drawn fab body x = 114.0 = the outline,
pad copper from x = 115.6.

For an `edge_receptacle` (or a brief row carrying `mount_mode: edge_mount`)
the seat is now measured on the drawn body when the library drew one (fab,
else silk); every other entry, and the OVERHANG conjunct, keep the courtyard.
The NEAREST-EDGE conjunct reads the same basis: run 27's replay measured the
same socket's pad box 1.6 mm from the west edge and 1.3 mm from the south, so
on the courtyard it read "nearest the south edge but declared on the west" on
every one of ten seeds -- a fixed part, so no seed could pass.

Run:
    python3 tests/test_run26_edge_seat_body_basis.py
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
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # --- 0: the fixture is what the docstring says (anti-vacuity) ----------
    pcb = parse_kicad_pcb(ESP)
    from placement.body import board_bodies
    rect, src = fp.drawn_body_rect(board_bodies(pcb, ESP).get('USB1'),
                                   pcb.footprints['USB1'])
    x0 = pcb.board_info.board_bounds[0]
    check('esp_prog USB1 draws a fab body flush with the west edge',
          src == 'fab' and rect is not None and abs(rect[0] - x0) < 1e-6,
          f'{src} {rect} edge {x0}')
    pad_x = min(p.global_x - p.size_x / 2 for p in pcb.footprints['USB1'].pads
                if p.pad_type != 'np_thru_hole')
    check('and its pad copper is well inboard of that body',
          pad_x - x0 > 1.0, f'{pad_x - x0:.2f} mm')

    # --- 1: the real board, declared as the brief declares it --------------
    base = {'ref': 'USB1', 'class': 'edge_receptacle', 'edge': 'west',
            'overhang_mm': {'min': 0.0, 'max': 0.65}}
    r = _grade(ESP, dict(base))
    check('an edge_receptacle flush by its body raises no seat finding',
          _seat(r, 'USB1') == [], f'{[v.message for v in _seat(r, "USB1")]}')
    # The same pose, judged on the courtyard as before: a plain entry armed
    # by max_setback_mm keeps the old currency and DOES fire -- the change is
    # scoped to the receptacle class, not a loosening of the seat rule.
    plain = {'ref': 'USB1', 'edge': 'west', 'max_setback_mm': 0.5,
             'overhang_mm': {'min': 0.0, 'max': 0.65}}
    r2 = _grade(ESP, plain)
    s2 = _seat(r2, 'USB1')
    check('a plain entry is still measured on the courtyard and fires',
          len(s2) == 1 and s2[0].measured.get('basis') == 'courtyard'
          and s2[0].measured['edge_clearance_mm'] > 1.0,
          f'{[(v.measured, v.message[:60]) for v in s2]}')
    # A brief row: no class, but mount_mode carried in context -> body basis.
    brief_row = {'ref': 'USB1', 'edge': 'west', 'max_setback_mm': 0.5,
                 'overhang_mm': {'min': 0.0, 'max': 0.65},
                 'context': {'mount_mode': 'edge_mount'}}
    r3 = _grade(ESP, brief_row)
    check('mount_mode: edge_mount selects the body basis too',
          _seat(r3, 'USB1') == [], f'{[v.message for v in _seat(r3, "USB1")]}')
    # Anti-vacuity for the class arm: push the tolerance below the body's
    # own 0.00 mm is impossible, so instead demand a seat the body cannot
    # meet from the OTHER side -- an east claim on a west-flush part.
    east = dict(base, edge='east')
    r4 = _grade(ESP, east)
    check('the body basis is a measurement, not a bypass: an east claim on a '
          'west-flush part still fails the edge rule',
          any(v.rule == 'edge_connector' and v.ref == 'USB1'
              for v in r4.violations))

    # --- 1b: the NEAREST-EDGE conjunct reads the same basis (run 27) -----
    def _near(result, ref):
        return [v for v in result.violations
                if v.rule == 'edge_connector' and v.ref == ref
                and 'edge' in v.measured
                and 'edge_clearance_mm' not in v.measured]

    b = pcb.board_info.board_bounds
    pads = [p for p in pcb.footprints['USB1'].pads
            if p.pad_type != 'np_thru_hole']
    south_gap = b[3] - max(p.global_y + p.size_y / 2 for p in pads)
    west_gap = min(p.global_x - p.size_x / 2 for p in pads) - b[0]
    check('anti-vacuity: USB1 pad box lies nearer the south edge than the west',
          south_gap < west_gap, f'south {south_gap:.2f} west {west_gap:.2f}')
    check('an edge_receptacle flush by its body is nearest its declared edge',
          _near(r, 'USB1') == [], f'{[v.message for v in _near(r, "USB1")]}')
    n2 = _near(r2, 'USB1')
    check('a plain entry still reads the courtyard, and misreads south',
          len(n2) == 1 and n2[0].measured.get('edge') == 'south'
          and n2[0].measured.get('basis') == 'courtyard',
          f'{[(v.measured, v.message[:60]) for v in n2]}')
    check('mount_mode: edge_mount selects the body basis for nearest-edge too',
          _near(r3, 'USB1') == [], f'{[v.message for v in _near(r3, "USB1")]}')
    check('and the east claim is refused by nearest-edge on the body, not only by the seat',
          any(v.measured.get('edge') == 'west' for v in _near(r4, 'USB1')),
          f'{[v.measured for v in _near(r4, "USB1")]}')

    # --- 2: synthetic: no drawn body -> courtyard; silk body -> body:silk --
    for layer, want_basis, want_hit in (
            (None, 'courtyard', True),      # pads only: 1.6 mm inboard, fires
            ('F.SilkS', 'body:silk', False),  # silk box reaches the edge
            ('F.Fab', 'body:fab', False)):
        path = _synth(layer)
        try:
            entry = {'ref': 'U1', 'class': 'edge_receptacle', 'edge': 'west',
                     'overhang_mm': {'min': 0.0, 'max': 0.65}}
            rs = _grade(path, entry)
            hits = _seat(rs, 'U1')
            if want_hit:
                check(f'synthetic [{layer}]: seat fires on the {want_basis}',
                      len(hits) == 1 and hits[0].measured['basis'] == want_basis
                      and abs(hits[0].measured['edge_clearance_mm'] - 1.6) < 0.05,
                      f'{[(v.measured, v.message[:70]) for v in hits]}')
            else:
                check(f'synthetic [{layer}]: a body at the edge seats it ({want_basis})',
                      hits == [], f'{[(v.measured, v.message[:70]) for v in hits]}')
                # and the courtyard reading is still reported beside it when
                # the rule DOES fire elsewhere -- assert the basis by grading
                # the same board with the tolerance made impossible to meet.
                tight = dict(entry, max_setback_mm=-1.0)
                tight.pop('class')
                tight['context'] = {'mount_mode': 'edge_mount'}
                rt = _grade(path, tight)
                ht = _seat(rt, 'U1')
                check(f'synthetic [{layer}]: the finding names its basis',
                      len(ht) == 1 and ht[0].measured['basis'] == want_basis
                      and abs(ht[0].measured['courtyard_clearance_mm'] - 1.6) < 0.05,
                      f'{[v.measured for v in ht]}')
        finally:
            os.unlink(path)

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
