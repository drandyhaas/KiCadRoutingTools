#!/usr/bin/env python3
"""#1624: check_impedance reads a coplanar gap on KiCad's fill, not a grid's.

The audit used to read every gap on the fill MODEL, a raster. Two things
made a correct line a broken promise there, and board_score blocked on it:
the model stamped at the pour's own zone clearance (nothing published the
project's class map to it, though KiCad fills at max(zone, class)), and its
read-off subtracted `cell/2 + step/2`, so a true 0.20 read 0.15 or 0.20 by
where the grid fell under the track -- the 0.15 a tolerance miss.

Gated on one 30 mm line in a class at 0.2, its pour's zone at 0.12, the gap
declared 0.2, the line set where the old read-off gave 0.15:

  A. A stored KiCad fill (fractured, as KiCad writes it) at 0.2 reads 0.200
     at exact distance, source 'stored'; the promise holds.
  B. A stored fill at 0.10 still breaks the promise (a wrong gap fails).
  C. A stored fill over the trace reads 0 and the report says to refill.
  D. No stored fill: the fill model (gap_source 'model', and a note saying
     so; no refill is run), at the project's class, reads within half a cell
     of 0.2 at two grid offsets; the promise holds.
  E. The CLI exits 0 on A and 1 on B, naming the net.

    python3 tests/test_1624_coplanar_gap_kicad_fill.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check as run_check, evidence             # noqa: E402

failures = []
WIDTH = 0.3
GAP = 0.2


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def fill_block(yc, gap):
    """One fractured filled_polygon, as KiCad writes it: the outer ring, a
    zero-width slit in to the hole kept around the line, and back out."""
    hw = WIDTH / 2
    hx0, hx1 = 105 - hw - gap, 135 + hw + gap
    hy0, hy1 = yc - hw - gap, yc + hw + gap
    pts = [(100.5, 100.5), (139.5, 100.5), (139.5, 129.5), (100.5, 129.5),
           (100.5, yc), (hx0, yc), (hx0, hy1), (hx1, hy1), (hx1, hy0),
           (hx0, hy0), (hx0, yc), (100.5, yc)]
    xy = ' '.join(f'(xy {x:.4f} {y:.4f})' for x, y in pts)
    return f' (filled_polygon (layer "F.Cu") (pts {xy}))'


def build(d, yc=115.02, stored_gap=None, outline_y=100.0):
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, 'b.kicad_pcb')
    fp = lambda ref, x: (
        f' (footprint "T:{ref}" (layer "F.Cu") (at {x} {yc})'
        f' (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes)'
        f' (effects (font (size 1 1) (thickness 0.15))))'
        f' (attr smd) (pad "1" smd rect (at 0 0) (size 0.2 0.2) (layers "F.Cu")'
        f' (net 2 "/RF")))')
    fill = fill_block(yc, stored_gap) if stored_gap is not None else ''
    zone = (f' (zone (net 1) (net_name "GND") (layer "F.Cu")'
            f' (uuid "00000000-0000-0000-0000-000000001624") (hatch edge 0.5)'
            f' (connect_pads (clearance 0.12)) (min_thickness 0.1)'
            f' (filled_areas_thickness no) (fill yes (thermal_gap 0.5)'
            f' (thermal_bridge_width 0.5))'
            f' (polygon (pts (xy 100 {outline_y}) (xy 140 {outline_y}) (xy 140 130)'
            f' (xy 100 130))){fill})'
            # the reference plane under the line, so only the gap is graded
            f'\n (zone (net 1) (net_name "GND") (layer "B.Cu")'
            f' (uuid "00000000-0000-0000-0000-000000002624") (hatch edge 0.5)'
            f' (connect_pads (clearance 0.12)) (min_thickness 0.1)'
            f' (filled_areas_thickness no) (fill yes (thermal_gap 0.5)'
            f' (thermal_bridge_width 0.5))'
            f' (polygon (pts (xy 100 {outline_y}) (xy 140 {outline_y}) (xy 140 130)'
            f' (xy 100 130))))')
    edges = ''.join(
        f' (gr_line (start {a} {b}) (end {c} {e}) (layer "Edge.Cuts")'
        f' (stroke (width 0.05) (type solid)))\n'
        for a, b, c, e in ((100, outline_y, 140, outline_y), (140, outline_y, 140, 130),
                           (140, 130, 100, 130), (100, 130, 100, outline_y)))
    text = '\n'.join([
        '(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")',
        ' (general (thickness 0.27)) (paper "A4")',
        ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))',
        ' (setup (stackup (layer "F.Cu" (type "copper") (thickness 0.035))',
        '   (layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4")'
        ' (epsilon_r 4.5) (loss_tangent 0.02))',
        '   (layer "B.Cu" (type "copper") (thickness 0.035))) (pad_to_mask_clearance 0))',
        ' (net 0 "") (net 1 "GND") (net 2 "/RF")',
        fp('J1', 105), fp('J2', 135),
        f' (segment (start 105 {yc}) (end 135 {yc}) (width {WIDTH}) (layer "F.Cu")'
        f' (net 2) (uuid "00000000-0000-0000-0000-00000000aaaa"))',
        zone, edges + ')', ''])
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    cls = lambda n, c: dict(name=n, clearance=c, track_width=0.2, via_diameter=0.6,
                            via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
                            priority=(2147483647 if n == 'Default' else 0))
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.1}}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cls('Default', 0.16), cls('RF', 0.2)],
                                    'meta': {'version': 4},
                                    'netclass_patterns': [{'netclass': 'RF',
                                                           'pattern': '/RF'}]}}, f)
    return evidence(path, 'the fixture board')


def measure(path, fill_source='auto'):
    from kicad_parser import parse_kicad_pcb
    import check_impedance as ci
    pcb = parse_kicad_pcb(path)
    nid = next(i for i, n in pcb.nets.items() if n.name == '/RF')
    rep = ci.analyze_impedance(pcb, net_patterns=['/RF'],
                               net_declared_gaps={nid: GAP},
                               fill_source=fill_source)
    return rep, rep.nets[0]


def main():
    tmp = tempfile.mkdtemp(prefix='t1624_')
    try:
        # A. stored fill at the declared gap
        a = build(os.path.join(tmp, 'a'), stored_gap=GAP)
        rep, n = measure(a)
        check('A: the gap is read on the stored fill', rep.gap_source == 'stored',
              rep.gap_source)
        check('A: every sample reads 0.200 at exact distance',
              bool(n.gap_samples) and all(abs(g - GAP) < 1e-6 for g in n.gap_samples),
              f'{min(n.gap_samples or [0]):.4f}..{max(n.gap_samples or [0]):.4f}')
        check('A: the promise holds', not n.promise_broken,
              f'off-target {n.length_gap_wrong_mm:.2f} mm')

        # B. a really wrong gap still fails
        b = build(os.path.join(tmp, 'b'), stored_gap=0.10)
        rep, n = measure(b)
        check('B: a stored 0.10 gap reads 0.10',
              bool(n.gap_samples) and abs(sorted(n.gap_samples)[len(n.gap_samples) // 2] - 0.10) < 1e-6)
        check('B: and breaks the promise', n.promise_broken,
              f'off-target {n.length_gap_wrong_mm:.2f} of {n.routed_length_mm:.2f} mm')

        # C. a stored fill over the trace
        c = build(os.path.join(tmp, 'c'), stored_gap=-0.05)
        rep, n = measure(c)
        check('C: fill over the trace reads 0', n.gap_samples and max(n.gap_samples) == 0.0)
        check('C: and the report says to refill',
              any('predates the copper' in s for s in rep.notes), str(rep.notes))

        # D. the model alone, at two grid offsets
        for tag, oy in (('d0', 100.0), ('d1', 100.02)):
            d = build(os.path.join(tmp, tag), outline_y=oy)
            rep, n = measure(d)
            check(f'D[{oy}]: no fill stored -> the model, never a refill',
                  rep.gap_source == 'model', rep.gap_source)
            check(f'D[{oy}]: the model reads within half a cell of 0.2',
                  bool(n.gap_samples) and all(abs(g - GAP) <= 0.026 for g in n.gap_samples),
                  f'{min(n.gap_samples or [0]):.4f}..{max(n.gap_samples or [0]):.4f}')
            check(f'D[{oy}]: the promise holds', not n.promise_broken,
                  f'off-target {n.length_gap_wrong_mm:.2f} mm')
            check(f'D[{oy}]: the report says the model was read',
                  any('fill MODEL' in s for s in rep.notes), str(rep.notes))

        # E. the CLI
        tool = os.path.join(ROOT, 'py_tools', 'check_impedance.py')
        run_check([sys.executable, '-X', 'utf8', tool, a, '--coplanar-gap', '0.2',
                   '--nets', '/RF'], accept=True)
        check('E: the CLI accepts A', True)
        run_check([sys.executable, '-X', 'utf8', tool, b, '--coplanar-gap', '0.2',
                   '--nets', '/RF'], code=1, refuse='Nets whose declared gap was NOT achieved')
        check('E: the CLI refuses B for the gap', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"FAIL: {len(failures)}: {failures}")
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
