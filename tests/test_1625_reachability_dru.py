#!/usr/bin/env python3
"""#1625: check_reachability prices each foreign object at its PAIR's value.

The field priced every foreign object at ONE number per foreign net (its
class) and never read the `.kicad_dru`, so a pad ringed by four GND pads with
0.40 mm slots read CAGED (class 0.16: 0.1 + 2 x 0.16 = 0.42 needed) while a
rule `A.NetClass == 'HS' && B.Type == 'Pad'` at 0.1 made the slot legal, and
route.py routed it with KiCad's DRC clean. Each object is now priced through
check_drc's resolver (`placement.legality.PadClearanceModel`, which reads the
class/type-scoped and layer rules), against the routed net's own track:

  1. the rule on disk, F.Cu only: PASSABLE, the throat 0.40 vs 0.30 needed,
     and the knobs line names the rule;
  2. control, no rule: CAGED at the class value, as before (0.42 needed);
  3. the routed net's OWN class is part of the pair (KiCad's max of the two
     classes): with the GND class at 0.12 and the routed net's at 0.16 the
     same slot reads CAGED, where pricing the foreign class alone said
     PASSABLE.

No routing, no KiCad; the board is built here.

    python3 tests/test_1625_reachability_dru.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
TOOL = os.path.join(REPO, 'py_tools', 'check_reachability.py')
RULE = ("(rule hs_to_pads (condition \"A.NetClass == 'HS' && B.Type == 'Pad'\") "
        "(constraint clearance (min 0.1mm)))")
FAILS = []


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def fp(ref, x, y, net, name, w, h):
    return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
            f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
            f'    (attr smd)\n    (pad "1" smd rect (at 0 0) (size {w} {h}) '
            f'(layers "F.Cu") (net {net} "{name}")))\n')


def build(d, dru=None, gnd_class=None):
    """S1.1 (/SIG, class HS at 0.16) in a pinwheel of four GND pads, slots
    0.40 wide; `gnd_class` puts GND in a class of that clearance."""
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n'
         ' (net 0 "") (net 1 "GND") (net 2 "/SIG")\n')
    b += fp('S1', 120, 115, 2, '/SIG', .3, .3) + fp('S2', 123, 115, 2, '/SIG', .6, .6)
    a, t, s, L = .28, .5, .40, .9

    def wall(ref, x0, y0, x1, y1):
        return fp(ref, round((x0 + x1) / 2, 4), round((y0 + y1) / 2, 4), 1, 'GND',
                  round(x1 - x0, 4), round(y1 - y0, 4))
    cx, cy = 120, 115
    b += wall('R1', cx - a + s, cy - a - t, cx + a + L, cy - a)
    b += wall('R2', cx + a, cy - a + s, cx + a + t, cy + a + L)
    b += wall('R3', cx - a - L, cy + a, cx + a - s, cy + a + t)
    b += wall('R4', cx - a - t, cy - a - L, cx - a, cy + a - s)
    for x1, y1, x2, y2 in ((110, 105, 140, 105), (140, 105, 140, 125),
                           (140, 125, 110, 125), (110, 125, 110, 105)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')

    def cl(n, c=0.16):
        return dict(name=n, clearance=c, track_width=0.1, via_diameter=0.3,
                    via_drill=0.15, wire_width=6, bus_width=12, line_style=0,
                    priority=(2147483647 if n == 'Default' else 0))
    classes = [cl('Default', gnd_class or 0.16), cl('HS')]
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.1,
                                                           'min_track_width': 0.1}}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': classes, 'meta': {'version': 4},
                                    'netclass_patterns': [{'netclass': 'HS',
                                                           'pattern': '/SIG'}]}},
                  f, indent=1)
    if dru:
        with open(pcb[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write(f'(version 1)\n{dru}\n')
    return pcb


ARGS = ['--pad', 'S1.1', '--track', '0.1', '--via', '0.3', '--layers', 'F.Cu']


def main():
    tmp = tempfile.mkdtemp(prefix='t1625_')
    try:
        pcb = build(os.path.join(tmp, 'rule'), RULE)
        js = os.path.join(tmp, 'rule.json')
        r = run_utils.check(PY + [TOOL, pcb] + ARGS + ['--json-out', js], accept=True)
        with open(run_utils.evidence(js, 'check_reachability --json-out'),
                  encoding='utf-8') as f:
            doc = json.load(f)
        ok('1. the rule on disk: PASSABLE', doc['verdict'] == 'PASSABLE', doc['verdict'])
        ok('1. the throat is the 0.40 slot, 0.30 needed at the rule',
           abs((doc['gap_mm'] or 0) - 0.40) < 0.011
           and abs((doc['gap_need_mm'] or 0) - 0.30) < 1e-6,
           (doc['gap_mm'], doc['gap_need_mm']))
        ok('1. the knobs line names the rule',
           "pairs priced by rule 'hs_to_pads': 0.1mm" in r.stdout, r.stdout[-600:])
        ok('1. and the JSON carries it',
           doc.get('clearance_sources') == {"rule 'hs_to_pads'": [0.1]},
           doc.get('clearance_sources'))

        pcb = build(os.path.join(tmp, 'norule'))
        run_utils.check(PY + [TOOL, pcb] + ARGS, refuse='VERDICT    CAGED', code=1)
        ok('2. control, no rule: CAGED at the class value', True)

        pcb = build(os.path.join(tmp, 'ownclass'), gnd_class=0.12)
        run_utils.check(PY + [TOOL, pcb] + ARGS, refuse='VERDICT    CAGED', code=1)
        ok("3. the routed net's own 0.16 class binds against GND's 0.12", True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
