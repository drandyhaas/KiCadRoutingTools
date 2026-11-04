#!/usr/bin/env python3
"""#1643: the fill model keeps a pour off Edge.Cuts at KiCad's edge gap.

The model kept every Edge.Cuts item max(zone clearance, the zone net's
class, the board's min_copper_edge_clearance) away (#1564). Probed on
kicad-cli 10.0.6 (`pcb drc --refill-zones --save-board`, then the saved
fill's distance to a 40x30 outline), KiCad's gap is the edge clearance
ALONE:

  zone  class  board edge  .kicad_dru                         gap
  0.2   0.2    0.05        -                                  0.05
  0.2   0.2    0.30        -                                  0.30
  0.1   0.1    0.30        -                                  0.30
  0.3   0.1    0.05        -                                  0.05  zone above both
  0.1   0.4    0.05        -                                  0.05  class above both
  0.2   0.2    0.05        edge_clearance 0.4                 0.40
  0.2   0.2    0.05        edge_clearance 0.4 (layer F.Cu)    0.40 F.Cu, 0.05 B.Cu
  0.2   0.2    0.05        edge_clearance 0.4, A.NetName GND  0.40
  0.2   0.2    0.05        edge_clearance 0.4, A.Type Zone    0.40
  0.2   0.2    0.30        edge_clearance 0.1                 0.10  a rule BELOW the board's
  0.2   0.2    0.05        clearance 0.5                      0.05  not an edge constraint
  0.2   0.2    0.05        - (edge line 0.5 wide)             0.05  from the line's centre
  0.2   0.2    0           -                                  0     to the edge
  0.2   0.2    (no key)    -                                  0.50  pcbnew's factory value
  0.2   0.2    (no .kicad_pro at all)                         0.50

An interior cutout is kept the same gap (0.05 / 0 / 0.30 at a 0.05 / 0 /
0.30 rule, probed the same way; tests/test_fill_model_pad_override.py pins
the model's half).

Each row is pinned here as a consequence the router reads: the outline is
redrawn as two halves joined by a 4 mm isthmus, whose F.Cu fill is one
piece when the isthmus is wider than 2 x gap + min_thickness and two when it
is narrower (by 0.08 mm either way). The model must agree on every board,
on any host. With kicad-cli present, KiCad refills the same boards and must
agree too, so the table is re-probed wherever KiCad is installed; without
it the table stands as recorded and the file says it was not re-probed.

    python3 tests/test_1643_zone_edge_gap.py [case-letter ...]
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from kicad_oracle import find_kicad_cli  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from plane_fill_model import ZoneFillModel  # noqa: E402
import run_utils  # noqa: E402

FIXTURE = os.path.join(TESTS, 'fixtures', '1643', 'edge_gap.kicad_pcb')
MIN_THICKNESS = 0.25
MARGIN = 0.08

#: (case, zone clearance, Default class clearance, board edge rule,
#:  .kicad_dru body or None, Edge.Cuts line width, KiCad's F.Cu gap)
CASES = [
    ('A', 0.2, 0.2, 0.05, None, 0.1, 0.05),
    ('B', 0.2, 0.2, 0.30, None, 0.1, 0.30),
    ('C', 0.1, 0.1, 0.30, None, 0.1, 0.30),
    ('D', 0.3, 0.1, 0.05, None, 0.1, 0.05),
    ('E', 0.1, 0.4, 0.05, None, 0.1, 0.05),
    ('F', 0.2, 0.2, 0.05, '(rule e (constraint edge_clearance (min 0.4mm)))', 0.1, 0.40),
    ('G', 0.2, 0.2, 0.05, '(rule e (layer F.Cu) (constraint edge_clearance (min 0.4mm)))',
     0.1, 0.40),
    ('G2', 0.2, 0.2, 0.05, '(rule e (layer B.Cu) (constraint edge_clearance (min 0.4mm)))',
     0.1, 0.05),
    ('H', 0.2, 0.2, 0.05, "(rule e (constraint edge_clearance (min 0.4mm)) "
                          "(condition \"A.NetName == 'GND'\"))", 0.1, 0.40),
    ('M', 0.2, 0.2, 0.05, "(rule e (constraint edge_clearance (min 0.4mm)) "
                          "(condition \"A.Type == 'Zone'\"))", 0.1, 0.40),
    ('I', 0.2, 0.2, 0.30, '(rule e (constraint edge_clearance (min 0.1mm)))', 0.1, 0.10),
    ('J', 0.2, 0.2, 0.05, '(rule c (constraint clearance (min 0.5mm)))', 0.1, 0.05),
    ('K', 0.2, 0.2, 0.05, None, 0.5, 0.05),
    ('L', 0.2, 0.2, 0.0, None, 0.1, 0.0),
    ('Q', 0.2, 0.2, 'no key', None, 0.1, 0.50),
    ('R', 0.2, 0.2, 'no project', None, 0.1, 0.50),
]

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def outline(neck):
    """Two 18x30 halves joined at y=15 by a 4 mm isthmus `neck` wide."""
    a, b = 15 - neck / 2, 15 + neck / 2
    return [(0, 0), (18, 0), (18, a), (22, a), (22, 0), (40, 0), (40, 30),
            (22, 30), (22, b), (18, b), (18, 30), (0, 30)]


def stage(d, zc, cc, edge, dru, width, neck):
    text = open(FIXTURE, encoding='utf-8').read()
    text, n = re.subn(r'\t\(gr_line\n(?:\t\t.*\n)*?\t\)\n', '', text)
    assert n == 4, f'fixture: {n} outline lines, expected 4'
    pts = outline(neck)
    lines = ''.join(
        f'\t(gr_line (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) '
        f'(stroke (width {width}) (type default)) (layer "Edge.Cuts"))\n'
        for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]))
    text = text.replace('\t(zone\n', lines + '\t(zone\n', 1)
    text, n = re.subn(r'(\(connect_pads yes\n\s*)\(clearance [\d.]+\)',
                      lambda m: f'{m.group(1)}(clearance {zc})', text)
    assert n == 2, f'fixture: {n} zone clearances, expected 2'
    pcb = os.path.join(d, 'b.kicad_pcb')
    open(pcb, 'w', encoding='utf-8').write(text)
    if edge != 'no project':
        rules = {} if edge == 'no key' else {"min_copper_edge_clearance": edge}
        json.dump({"meta": {"filename": "b.kicad_pro", "version": 3},
                   "board": {"design_settings": {"rules": rules}},
                   "net_settings": {"classes": [{"name": "Default", "clearance": cc,
                                                 "track_width": 0.2, "via_diameter": 0.6,
                                                 "via_drill": 0.3}], "meta": {"version": 4}}},
                  open(os.path.join(d, 'b.kicad_pro'), 'w'), indent=2)
    if dru:
        open(os.path.join(d, 'b.kicad_dru'), 'w').write('(version 1)\n' + dru + '\n')
    return pcb


def model_whole(pcb):
    pcb_data = parse_kicad_pcb(pcb)
    zone = next(z for z in pcb_data.zones if z.layer == 'F.Cu')
    m = ZoneFillModel(pcb_data, zone)
    assert m.ok, 'fixture: the model did not build'
    left, right = m.query_component(9.0, 5.0), m.query_component(31.0, 5.0)
    assert left and right, f'fixture: no fill in a half ({left}, {right})'
    return left == right


def kicad_whole(cli, pcb):
    subprocess.run([cli, 'pcb', 'drc', '--refill-zones', '--save-board', '--format', 'json',
                    '-o', os.path.join(os.path.dirname(pcb), 'drc.json'), pcb],
                   capture_output=True, timeout=300)
    n = len(re.findall(r'\(filled_polygon\s+\(layer "F\.Cu"\)', open(pcb, encoding='utf-8').read()))
    assert n in (1, 2), f'KiCad left {n} F.Cu fill pieces'
    return n == 1


def main(only=()):
    run_utils.evidence(FIXTURE, 'the #1643 fixture board')
    cli = find_kicad_cli(warn=False)
    tmp = tempfile.mkdtemp(prefix='t1643_')
    try:
        for name, zc, cc, edge, dru, width, gap in CASES:
            if only and name not in only:
                continue
            for want, neck in ((True, 2 * gap + MIN_THICKNESS + MARGIN),
                               (False, 2 * gap + MIN_THICKNESS - MARGIN)):
                d = os.path.join(tmp, f'{name}_{"whole" if want else "split"}')
                os.makedirs(d)
                pcb = stage(d, zc, cc, edge, dru, width, neck)
                what = (f'{name}: zone {zc} class {cc} edge {edge}'
                        + (f' dru {dru}' if dru else '') + (f' line {width}' if width != 0.1 else '')
                        + f', isthmus {neck:.2f} -> {"one piece" if want else "two"}')
                got = model_whole(pcb)
                check(f'model  {what}', got == want, f'model reads it {"whole" if got else "split"}')
                if cli:
                    got = kicad_whole(cli, pcb)
                    check(f'KiCad  {what}', got == want,
                          f'KiCad fills it {"whole" if got else "split"}: the table is stale')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    if not cli:
        print('  (kicad-cli not present: the recorded table was not re-probed)')
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main(set(sys.argv[1:])))
