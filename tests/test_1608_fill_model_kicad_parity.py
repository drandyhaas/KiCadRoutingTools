#!/usr/bin/env python3
"""#1608: the shared fill model pours where KiCad's fill pours, no further.

plane_fill_model.ZoneFillModel's default mode (read by the connectivity
grader, the oracle, the region connector, the plane repairs, the BGA fanout,
the router's pour launches and check_impedance) poured where KiCad's filler
does not, two ways:

* it kept no HOLE clearance: KiCad keeps the fill max(clearance, hole
  clearance) from every foreign drill, at the board's min_hole_clearance,
  else pcbnew's factory 0.25 -- and an NPTH's hole only the hole clearance
  away, so a declared 0 pours to the hole wall;
* its min-thickness opening eroded with a 1-cell structuring element (a plus
  at min_thickness 0.1), which passed a 45-degree channel down to ~0.08 mm
  and missed axis-aligned ones up to ~0.14 mm.

The model now stamps the hole knock-outs and computes KiCad's deflated fill
(the opening's erosion) exactly at sample points half a cell apart, joining
two of them when they are 8-adjacent -- their min_thickness discs overlap, so
every join it makes is one KiCad's deflate-then-inflate makes. Its widened
probe answers with the NEAREST labelled cell, not the smallest label in the
window (a pad in a walled-off pocket took the flood's label).

Every KiCad number below was measured on kicad-cli / pcbnew 10.0.3 (refill,
then the filled polygons): the track-end pinch boards join at a neck of
0.100 and not at 0.096 at every angle; the via pairs (zone clearance 0.1,
hole clearance 0.25) join at a neck of 0.099 and not at 0.098 at
min_thickness 0.1, and down to 0.216 but not at 0.214 at 0.25; the
knock-out distances are those of the probe board below. What each case pins (all fail on the pre-#1608 model):

* #1581's repro: U1's GND pocket, walled off by four via-in-pad pins with
  0.086 mm 45-degree necks, is not joined to the flood;
* the same pins at a = 0.72 with min_hole_clearance 0.4: not joined (KiCad:
  the drill-to-drill gap is 0.018 mm); at 0.05 it is;
* a pinch neck between two track ends of 0.09 mm (under min_thickness 0.1)
  never joins, at 0, 22.5 and 45 degrees; one of 0.1 or 0.13 mm always does;
* two vias whose hole knock-outs leave a neck of exactly min_thickness 0.1
  (0.8 mm apart, drill 0.2, hole clearance 0.25: a BGA dogbone field) join,
  as KiCad joins them, and at 0.096 do not; at min_thickness 0.25 a neck of
  0.23 joins (KiCad's inflate rejoins the two halves of its deflated fill)
  and one of 0.21 does not;
* the knock-out around a via, a plated pad and an NPTH sits where KiCad's
  does, at hole clearance 0.4, 0 and undeclared (factory 0.25);
* the scipy-free path (KiCad's bundled Python: the GUI) labels the same
  components as scipy.

    python3 -X utf8 tests/test_1608_fill_model_kicad_parity.py
"""
import json
import math
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

import numpy as np                                              # noqa: E402

import plane_fill_model as pfm                                  # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from test_1581_graze_prune_pour_only_join import (              # noqa: E402
    write_board, CX, CY, STUB_VIA)

FAILURES = []


def check(cond, what, detail=''):
    print(('  ok    ' if cond else '  FAIL  ') + what
          + ('' if cond or not detail else '  -- ' + detail))
    if not cond:
        FAILURES.append(what)


HEAD = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
        ' (general (thickness 1.6))\n (paper "A4")\n'
        ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
        ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n (net 1 "GND")\n'
        ' (net 2 "/X")\n')


def _fp(ref, x, y, pad):
    return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x:.4f} {y:.4f})\n'
            f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
            f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
            f'    (attr smd)\n    {pad}\n  )\n')


def _gnd_pad(size=0.5):
    return (f'(pad "1" smd rect (at 0 0) (size {size} {size}) (layers "F.Cu") '
            f'(net 1 "GND"))')


def _tail(w, h, zc, mth):
    b = (f'  (zone (net 1) (net_name "GND") (layer "F.Cu") (hatch edge 0.5) '
         f'(connect_pads yes (clearance {zc}))\n'
         f'    (min_thickness {mth}) (filled_areas_thickness no) '
         f'(fill yes (thermal_gap 0.2) (thermal_bridge_width 0.2))\n'
         f'    (polygon (pts (xy 0.5 0.5) (xy {w - 0.5} 0.5) (xy {w - 0.5} {h - 0.5}) '
         f'(xy 0.5 {h - 0.5}))))\n')
    for x1, y1, x2, y2 in ((0, 0, w, 0), (w, 0, w, h), (w, h, 0, h), (0, h, 0, 0)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    return b + ')\n'


def _project(path, hole):
    rules = {"min_clearance": 0.05}
    if hole is not None:
        rules["min_hole_clearance"] = hole
    with open(path[:-len('kicad_pcb')] + 'kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({"board": {"design_settings": {"rules": rules}},
                   "net_settings": {"classes": [{"name": "Default",
                                                 "clearance": 0.05}]}}, f)


def pinch_board(path, rings, zc=0.2, mth=0.1, hole=0.05):
    """A GND pad in each foreign-net square ring whose one gap has a fill
    neck `w`, the ring turned by `theta`; a GND pad in the open flood at
    (3, 3). `rings` = [(w, theta, vias)]: the gap is between two track ends
    (copper gap w + 2 zc), or with `vias` between two foreign 0.45/0.2 vias
    the ring's tracks end on (the neck set by the larger of their copper and
    hole knock-outs). Returns the centres."""
    tw, half = 0.2, 1.5
    vko = max(0.225 + zc, 0.1 + hole)
    body, centres = '', []
    for k, (w, th, vias) in enumerate(rings):
        cx, cy = 8.0 + 6.0 * k + 0.0173 * k, 8.0 + 0.0291 * k
        centres.append((cx, cy))
        e = (w / 2 + vko) if vias else ((w + 2 * zc) / 2 + tw / 2)
        a = math.radians(th)

        def rot(px, py):
            return (cx + px * math.cos(a) - py * math.sin(a),
                    cy + px * math.sin(a) + py * math.cos(a))
        for p, q in (((-half, -half), (half, -half)), ((half, half), (-half, half)),
                     ((-half, half), (-half, -half)), ((half, -half), (half, -e)),
                     ((half, e), (half, half))):
            (x1, y1), (x2, y2) = rot(*p), rot(*q)
            body += (f'  (segment (start {x1:.6f} {y1:.6f}) (end {x2:.6f} {y2:.6f}) '
                     f'(width {0.1 if vias else tw}) (layer "F.Cu") (net 2))\n')
        if vias:
            for yy in (-e, e):
                vx, vy = rot(half, yy)
                body += (f'  (via (at {vx:.6f} {vy:.6f}) (size 0.45) (drill 0.2) '
                         f'(layers "F.Cu" "B.Cu") (net 2))\n')
        body += _fp(f'A{k}', cx, cy, _gnd_pad())
    body += _fp('B0', 3.0, 3.0, _gnd_pad())
    with open(path, 'w', encoding='utf-8') as f:
        f.write(HEAD + body + _tail(8.0 + 6.0 * len(rings), 16.0, zc, mth))
    _project(path, hole)
    return centres


#: (label, kind, x, size, drill) of the knock-out probe board, all at y = 10
PROBES = [('via 0.45/0.3', 'via', 10, 0.45, 0.3), ('via 0.6/0.3', 'via', 15, 0.6, 0.3),
          ('plated pad 1.0/0.6', 'th', 20, 1.0, 0.6),
          ('NPTH 1.0', 'npth', 30, 1.0, 1.0), ('NPTH 1.0 in a 1.4 pad', 'npth', 35, 1.4, 1.0)]
#: KiCad 10.0.3's fill edge, mm from each probe's centre (zone clearance 0.2),
#: by the board's min_hole_clearance (None = undeclared: factory 0.25)
KICAD_EDGE = {0.4: [0.5505, 0.5505, 0.7005, 0.9005, 0.9005],
              0.0: [0.4255, 0.5005, 0.7005, 0.5005, 0.9005],
              None: [0.4255, 0.5005, 0.7005, 0.7505, 0.9005]}


def probe_board(path, hole):
    body = _fp('G1', 5, 5, _gnd_pad())
    for i, (_n, kind, x, s, d) in enumerate(PROBES):
        if kind == 'via':
            body += (f'  (via (at {x} 10) (size {s}) (drill {d}) '
                     f'(layers "F.Cu" "B.Cu") (net 2))\n')
        elif kind == 'th':
            body += _fp(f'P{i}', x, 10, f'(pad "1" thru_hole circle (at 0 0) '
                        f'(size {s} {s}) (drill {d}) (layers "*.Cu") (net 2 "/X"))')
        else:
            body += _fp(f'H{i}', x, 10, f'(pad "" np_thru_hole circle (at 0 0) '
                        f'(size {s} {s}) (drill {d}) (layers "*.Cu" "*.Mask"))')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(HEAD + body + _tail(50, 20, 0.2, 0.1))
    if hole is None:
        pro = path[:-len('kicad_pcb')] + 'kicad_pro'
        if os.path.exists(pro):
            os.remove(pro)
    else:
        _project(path, hole)
    return path


def fill_edge(m, x, y):
    """Distance from (x, y) to the nearest fill cell centre of model `m`."""
    ii, jj = np.nonzero(m.labels)
    px = m.x0 + (ii + 0.5) * m.cell
    py = m.y0 + (jj + 0.5) * m.cell
    near = (abs(px - x) < 2) & (abs(py - y) < 2)
    return float(np.hypot(px[near] - x, py[near] - y).min())


def joined(m, a, b, size=0.4):
    ca, cb = m.query_component(*a, size=size), m.query_component(*b, size=size)
    return bool(ca) and ca == cb


def same_partition(a, b):
    if ((a > 0) != (b > 0)).any():
        return False
    pairs = np.unique(np.stack([a[a > 0], b[b > 0]]), axis=1)
    return (len(np.unique(pairs[0])) == pairs.shape[1]
            == len(np.unique(pairs[1])))


def main():
    tmp = tempfile.mkdtemp(prefix='krt_1608_')

    def zone_of(pcb):
        return next(z for z in pcb.zones if z.layer == 'F.Cu'
                    and pcb.nets[z.net_id].name == 'GND')

    print("#1581's repro: four via-in-pad pins, 0.086 mm necks at 45 degrees")
    pcb = parse_kicad_pcb(write_board(os.path.join(tmp, 'ring.kicad_pcb'),
                                      0.45, False, 0.15, control=True))
    m = pfm.ZoneFillModel(pcb, zone_of(pcb))
    for size in (0.0, 0.4):
        check(not joined(m, (CX, CY), STUB_VIA, size),
              f"U1's GND pocket is not joined to the flood (probe {size} mm)",
              'KiCad pours nothing through the necks')
    check(joined(m, (130.0, 104.0), (108.0, 104.0)),
          'G3 and G2 in the open flood are joined')

    print('\nhole clearance (the same pins at a = 0.72: a 0.468 mm copper gap)')
    for hole, want in ((0.4, False), (0.05, True)):
        pcb = parse_kicad_pcb(write_board(os.path.join(tmp, f'h{hole}.kicad_pcb'),
                                          0.72, False, hole))
        check(abs(pfm.kicad_fill_hole_clearance(pcb) - hole) < 1e-9,
              f'min_hole_clearance {hole} is read')
        check(joined(pfm.ZoneFillModel(pcb, zone_of(pcb)), (CX, CY), STUB_VIA) == want,
              f'at min_hole_clearance {hole} the pocket is '
              + ('joined, as KiCad joins it' if want else 'not joined: KiCad '
                 'pours nothing through a 0.018 mm drill-to-drill gap'))

    print('\nthe min-thickness opening (pinch necks, zone clearance 0.2, '
          'min_thickness 0.1)')
    rings = [(w, th, False) for w in (0.09, 0.1, 0.13) for th in (0, 22.5, 45)]
    path = os.path.join(tmp, 'pinch.kicad_pcb')
    centres = pinch_board(path, rings)
    pcb = parse_kicad_pcb(path)
    z = zone_of(pcb)
    m = pfm.ZoneFillModel(pcb, z)
    for (w, th, _v), c in zip(rings, centres):
        check(joined(m, c, (3.0, 3.0)) == (w >= 0.1),
              f'a {w} mm neck between track ends at {th} degrees is '
              + ('joined' if w >= 0.1 else 'not joined') + ', as in KiCad')

    print('\nvia pairs at the threshold (0.8 mm dogbone vias, 0.2 mm drill, '
          'hole clearance 0.25, zone clearance 0.1)')
    for mth, cases in ((0.1, ((0.1, True), (0.096, False))),
                       (0.25, ((0.23, True), (0.21, False)))):
        rings = [(w, th, True) for w, _j in cases for th in (0, 30)]
        path = os.path.join(tmp, f'vias_{mth}.kicad_pcb')
        centres = pinch_board(path, rings, zc=0.1, mth=mth, hole=0.25)
        vpcb = parse_kicad_pcb(path)
        mv = pfm.ZoneFillModel(vpcb, zone_of(vpcb))
        for (w, th, _v), c in zip(rings, centres):
            want = dict(cases)[w]
            check(joined(mv, c, (3.0, 3.0)) == want,
                  f'min_thickness {mth}: a {w} mm neck between hole '
                  f'knock-outs at {th} degrees is '
                  + ('joined' if want else 'not joined') + ', as in KiCad')
    if pfm._HAS_SCIPY:
        pfm._HAS_SCIPY = False
        try:
            m2 = pfm.ZoneFillModel(pcb, z)
        finally:
            pfm._HAS_SCIPY = True
        check(same_partition(m.labels, m2.labels),
              'the scipy-free path (the GUI) labels the same components')
        from scipy import ndimage
        rng = np.random.default_rng(1608)
        ok = True
        for _ in range(60):
            g = rng.random(tuple(rng.integers(1, 40, 2))) < rng.uniform(0.3, 0.7)
            for eight in (False, True):
                a, na = pfm._label_components(g, eight)
                b, nb = ndimage.label(g, structure=np.ones((3, 3)) if eight else None)
                ok &= na == nb and same_partition(a, b)
        check(ok, '_label_components matches ndimage.label, 4- and 8-connected')
    else:
        print('  (scipy absent: the scipy parity checks are skipped)')

    print('\nknock-outs around holes (KiCad 10.0.3 fill edges, cell-quantised)')
    for hole in (0.4, 0.0, None):
        pcb = parse_kicad_pcb(probe_board(
            os.path.join(tmp, f'probe_{hole}.kicad_pcb'), hole))
        m = pfm.ZoneFillModel(pcb, zone_of(pcb))
        got = [fill_edge(m, x, 10.0) for _n, _k, x, _s, _d in PROBES]
        bad = [f'{n}: {g:.3f} vs {k:.3f}' for (n, *_r), g, k
               in zip(PROBES, got, KICAD_EDGE[hole]) if not (k - 0.01 < g < k + 0.04)]
        check(not bad, f'min_hole_clearance {hole}: every fill edge is KiCad\'s',
              '; '.join(bad))
    check(pfm.kicad_fill_hole_clearance(pcb) == pfm.KICAD_FACTORY_HOLE_CLEARANCE,
          'with no project the model fills at the factory 0.25')

    print()
    if FAILURES:
        print('FAILED: %d check(s): %s' % (len(FAILURES), FAILURES))
        return 1
    print('PASS: the fill model pours where KiCad pours')
    return 0


if __name__ == '__main__':
    sys.exit(main())
