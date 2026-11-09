#!/usr/bin/env python3
"""#1627: check_complete --authored-from judges each object against the floor
that binds IT in the authored project.

It compared every via with the one Board Setup minimum and never read the
authored `.kicad_dru`, so a via a class rule explicitly allows (KiCad 0
errors, check_drc 0) read UNSOUND, and with a covered and an uncovered via it
counted both. Now each track, via and hole is held to the Board Setup value
unless an AUTHORED rule sets its minimum (resolved as check_drc resolves a
size floor); the count is the objects below their own floor, and the ones a
rule allows are reported as `measured_kept` with `held_by_rule` and the rule.

Fixture: /SIG (class HS) and /OTH (Default), Board Setup ring 0.13, a rule
`A.NetClass == 'HS'` -> annular_width 0.07, vias at ring 0.075. The authored
board is the same board without its vias. No routing, no KiCad.

    python3 tests/test_1627_authored_rule_floors.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools'),
          os.path.join(REPO, 'py_placer')):
    sys.path.insert(0, p)
from check_complete import fab_floor_integrity  # noqa: E402

FAILS = []
RULE = "(rule hs_ring (condition \"A.NetClass == 'HS'\") (constraint annular_width (min 0.07mm)))"


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def fp(ref, x, y, side, net, name):
    return (f'  (footprint "T:{ref}" (layer "{side}") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
            f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
            f'    (attr smd)\n    (pad "1" smd rect (at 0 0) (size 1 1) '
            f'(layers "{side}") (net {net} "{name}")))\n')


def build(path, vias, dru=None):
    """`vias`: the nets (1 /SIG, 2 /OTH) that carry a ring-0.075 via."""
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n'
         ' (net 0 "") (net 1 "/SIG") (net 2 "/OTH")\n')
    b += (fp('A1', 110, 106, 'F.Cu', 1, '/SIG') + fp('A2', 130, 106, 'B.Cu', 1, '/SIG')
          + fp('C1', 110, 114, 'F.Cu', 2, '/OTH') + fp('C2', 130, 114, 'B.Cu', 2, '/OTH'))
    for i, n in enumerate(vias):
        b += (f'  (via (at 120 {106 if n == 1 else 114}) (size 0.35) (drill 0.2) '
              f'(layers "F.Cu" "B.Cu") (net {n}) '
              f'(uuid "00000000-0000-0000-0000-00000000000{i + 1}"))\n')
    for x1, y1, x2, y2 in ((100, 100, 140, 100), (140, 100, 140, 120),
                           (140, 120, 100, 120), (100, 120, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')

    def cl(n):
        return dict(name=n, clearance=0.2, track_width=0.2, via_diameter=0.6,
                    via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
                    priority=(2147483647 if n == 'Default' else 0))
    rules = dict(min_clearance=0.1, min_track_width=0.15, min_via_diameter=0.3,
                 min_through_hole_diameter=0.2, min_via_annular_width=0.13,
                 min_hole_to_hole=0.2, min_copper_edge_clearance=0.2)
    stem = path[:-len('.kicad_pcb')]
    with open(stem + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': rules}},
                   'meta': {'filename': os.path.basename(stem) + '.kicad_pro',
                            'version': 3},
                   'net_settings': {'classes': [cl('Default'), cl('HS')],
                                    'meta': {'version': 4},
                                    'netclass_patterns': [{'netclass': 'HS',
                                                           'pattern': '/SIG'}]}},
                  f, indent=1)
    if dru:
        with open(stem + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write(f'(version 1)\n{dru}\n')
    return path


def ring(r):
    return next((x for x in r.get('relaxed', [])
                 if x['key'] == 'min_via_annular_width'), None)


def kept(r):
    return next((x for x in r.get('measured_kept', [])
                 if x['key'] == 'min_via_annular_width'), None)


def main():
    tmp = tempfile.mkdtemp(prefix='t1627_')
    try:
        def arm(name, vias, authored_rule, graded_rule=None):
            d = os.path.join(tmp, name)
            os.makedirs(d)
            a = build(os.path.join(d, 'authored.kicad_pcb'), [], authored_rule)
            b = build(os.path.join(d, 'routed.kicad_pcb'), vias, graded_rule)
            return fab_floor_integrity(b, a, hole_grader=lambda *_: {'count': 0})

        r = arm('covered', [1], RULE)
        ok('covered: a via the authored rule allows is not relaxed', ring(r) is None,
           json.dumps(r.get('relaxed')))
        k = kept(r)
        ok('covered: it is reported as held by that rule',
           k and k.get('held_by_rule') == 1 and k.get('rules') == ['hs_ring'],
           json.dumps(r.get('measured_kept')))

        r = arm('uncovered', [2], RULE)
        ok('uncovered: a via no rule covers stays relaxed, 1 below',
           ring(r) and ring(r)['violations'] == 1, json.dumps(r.get('relaxed')))

        r = arm('both', [1, 2], RULE)
        ok('both: only the uncovered via counts (1 below, not 2)',
           ring(r) and ring(r)['violations'] == 1
           and ring(r).get('held_by_rule') == 1, json.dumps(r.get('relaxed')))

        r = arm('no_rule', [1], None)
        ok('control, no rule: the via is relaxed at the Board Setup value',
           ring(r) and ring(r)['violations'] == 1, json.dumps(r.get('relaxed')))

        r = arm('chain_rule', [1], None, graded_rule=RULE)
        ok('a rule on the GRADED board only cannot excuse its own copper',
           ring(r) and ring(r)['violations'] == 1, json.dumps(r.get('relaxed')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
