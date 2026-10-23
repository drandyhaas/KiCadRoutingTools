#!/usr/bin/env python3
"""#1331: a DC power jack and a hyphen-named or qualified-ref switch get an
edge class.

`part_class.classify_part` reads the footprint name against substring tables
and the reference against a leading prefix. Three gp14 parts fell through
(M16): One-Air-Max's DC1 `DC-IN-SMD_DC-045A-13A` (no `jack` in the name),
its PWR_SW1 `SW-TH_TK-6580A8-1` (the table had `sw_`, not `sw-`, and the
prefix read `PWR`), and CM5's SW501 `SW-SMD_TS-1010A-B` only through its
`SW` prefix. So `--emit-intent --declare-classes` declared none of them.

The short stems are matched with no letter on either side
(`EDGE_RECEPTACLE_WORDS`, `EDGE_ACTUATOR_WORDS`), so the `dc_in` inside an
`Converter_DCDC_Intel` QFN is not a jack and Samtec's `TSW-114` header is
not a switch (both bare substrings hit those on the corpus), and the actuator
prefix is also read on the designator after a qualifier (`_designator`),
which nothing else reads, so a `TP_GND1` stays a test point. Over the 500 tracked and stress boards (70812
footprints) the new rules change the class of four parts, all real: one
`DC-IN-TH` jack, two `SW-PB` pushbuttons and two `SW-SMD` push encoders.

Checks:
  1. classify_part on a real board: DC1 is an edge receptacle, PWR_SW1 and
     a hyphen-named SW-SMD part are edge actuators, a qualified `*_SW`
     reference on a non-switch footprint is an actuator by convention only;
     an Intel DC-DC QFN, a DC-DC module, a TSW-114 header and TP_GND1 are
     not edge parts of those kinds.
  2. check_floorplan --emit-intent --declare-classes declares DC1 with its
     class, and PWR_SW1 (overhanging its edge) with its class.

    python3 tests/test_1331_edge_class_names.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement.part_class import classify_part                 # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402

failures = []

# (reference, footprint, x, y): every part two copper pads on F.Cu.
PARTS = (
    ('DC1', 'OneAir:DC-IN-SMD_DC-045A-13A', 20, 20),
    ('PWR_SW1', 'OneAir:SW-TH_TK-6580A8-1', 30, 0.5),
    ('SW501', 'EasyEDA:SW-SMD_TS-1010A-B', 40, 20),
    ('PWR_SW2', 'Resistor_SMD:R_0603_1608Metric', 50, 20),
    ('U1', 'DCDC:Converter_DCDC_Intel_QFN-24_4x6mm_P0.65mm_ThermalVias', 20, 30),
    ('U2', 'Converter_DCDC:DC-DC_Module_TRACO_TSR-1', 30, 30),
    ('J9', 'PCM_EEZ_connectors:TSW-114-07-F-D', 40, 30),
    ('TP_GND1', 'TestPoint:TestPoint_Pad_D1.0mm', 50, 30),
)
WANT = {'DC1': 'edge_receptacle', 'PWR_SW1': 'edge_actuator',
        'SW501': 'edge_actuator', 'PWR_SW2': 'edge_actuator',
        'U1': None, 'U2': None, 'J9': 'connector_affinity',
        'TP_GND1': 'testpoint'}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board_text():
    out = ['(kicad_pcb (version 20241229) (generator "t1331")\n',
           '  (net 0 "")\n',
           '  (gr_rect (start 0 0) (end 60 40) (layer "Edge.Cuts"))\n']
    for ref, name, x, y in PARTS:
        out.append(
            f'  (footprint "{name}" (layer "F.Cu") (at {x} {y} 0)\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start -2 -1.5) (end 2 1.5) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu" "F.Mask"))\n'
            f'    (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu" "F.Mask")))\n')
    out.append(')\n')
    return ''.join(out)


def main():
    tmp = tempfile.mkdtemp(prefix='t1331_')
    try:
        path = os.path.join(tmp, 'classes.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(board_text())
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(evidence(path, 'synthetic board'))

        # 1. The classes.
        for ref, want in WANT.items():
            got = classify_part(pcb.footprints[ref], ref)
            check(f'{ref} {pcb.footprints[ref].footprint_name} -> {want}',
                  got.name == want, f'{got.name} {got.evidence}')
        ev = classify_part(pcb.footprints['PWR_SW2'], 'PWR_SW2')
        check('a qualified designator is evidence by convention only',
              ev.confidence == 'low'
              and "designator 'SW' in 'PWR_SW2'" in ev.evidence[0],
              str(ev))

        # 2. The emitter declares them.
        intent = os.path.join(tmp, 'intent.json')
        run_check([sys.executable, '-X', 'utf8', 'py_tools/check_floorplan.py',
                   path, '--emit-intent', intent, '--declare-classes'],
                  accept=True)
        doc = json.load(open(evidence(intent, 'emitted intent'),
                             encoding='utf-8'))
        rows = {c['ref']: c for c in doc.get('edge_connectors', [])}
        check('DC1 is declared an edge receptacle',
              (rows.get('DC1') or {}).get('class') == 'edge_receptacle',
              str(rows.get('DC1')))
        check('PWR_SW1, overhanging its edge, is declared an edge actuator',
              (rows.get('PWR_SW1') or {}).get('class') == 'edge_actuator',
              str(rows.get('PWR_SW1')))
        check('the near misses are not declared edge parts',
              not {'U1', 'U2'} & set(rows)
              and (rows.get('J9') or {}).get('class') != 'edge_actuator',
              str(sorted(rows)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
