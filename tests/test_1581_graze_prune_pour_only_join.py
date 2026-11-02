#!/usr/bin/env python3
"""#1581: the graze prune never trusts a pour-only join KiCad's fill may not make.

prune_grazing_segments drops a segment that grazes foreign copper when the net
is no worse connected without it. A removal that SPLITS the net's copper
leaves one side joined to the rest only by the pour, and the prune graded that
join with the fill MODEL. The model pours channels KiCad does not: its
min-thickness opening uses a 1-cell structuring element (a plus shape at
min_thickness 0.1), which passes a 45-degree gap down to ~0.07 mm, and it keeps
no hole clearance from a foreign drill.

The board is the issue's repro, built here: U1's GND pad ringed by four
via-in-pad signal pins 0.45 mm away, the pin-to-pin copper gap at 45 degrees
0.086 mm after the 0.1 zone clearance on each side -- under the zone's 0.1
min_thickness, so KiCad pours nothing through it. The earlier step's fanout
stub runs out through that gap to a via in the open flood, 0.093 mm from the
pins: a graze at --clearance 0.1. Without the fix the prune removes it (the
model joins the pad's fill pocket to the flood), route.py ships GND open and
the in-run oracle cannot put it back (checked against the pre-fix pass when
this test was written).

The control is G3, a GND pad in the open flood with a stub that grazes a
foreign pad on its way to a via: KiCad's fill surely joins G3, so the gate
must still let that stub go.

The hole half: at a = 0.72 the copper gap (0.468) passes a conservative model
without hole clearance, but with min_hole_clearance 0.4 the drill-to-drill gap
is 0.018 mm and KiCad pours nothing (checked with kicad-cli 10.0.3).

    python3 -X utf8 tests/test_1581_graze_prune_pour_only_join.py
"""
import json
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

CX, CY = 120.0, 115.0
D = 1.2 / math.sqrt(2)          # the stub's via, beyond the pins at 45 deg
STUB_VIA = (round(CX + D, 4), round(CY + D, 4))


def board_text(a, stub, control=False):
    """The issue's t.py: a GND pad ringed by four signal pins `a` mm away."""
    def fp(ref, x, y, pads, layer='F.Cu'):
        p = ''.join(f'    (pad "{n}" smd {sh} (at {px} {py}) (size {s} {s}) '
                    f'(layers "{layer}") (net {nid} "{nm}"))\n'
                    for n, sh, px, py, s, nid, nm in pads)
        return (f'  (footprint "T:{ref}" (layer "{layer}") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr smd)\n{p}  )\n')

    def zone(layer):
        return (f'  (zone (net 1) (net_name "GND") (layer "{layer}") (hatch edge 0.5) '
                f'(connect_pads yes (clearance 0.1))\n'
                f'    (min_thickness 0.1) (filled_areas_thickness no) '
                f'(fill yes (thermal_gap 0.2) (thermal_bridge_width 0.2))\n'
                f'    (polygon (pts (xy 100.5 100.5) (xy 139.5 100.5) '
                f'(xy 139.5 129.5) (xy 100.5 129.5))))\n')
    names = ['GND', '/S1', '/S2', '/S3', '/S4', '/X', '/TP']
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) '
         '(2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n')
    b += ''.join(f' (net {i + 1} "{n}")\n' for i, n in enumerate(names))
    b += fp('G2', 108, 104, [('1', 'rect', 0, 0, 1, 1, 'GND')])
    b += ('  (segment (start 108 104) (end 110 104) (width 0.1) (layer "F.Cu") (net 1))\n'
          '  (via (at 110 104) (size 0.35) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1))\n')
    b += fp('U1', CX, CY, [('1', 'rect', 0, 0, .25, 1, 'GND'),
                           ('2', 'circle', a, 0, .35, 2, '/S1'),
                           ('3', 'circle', -a, 0, .35, 3, '/S2'),
                           ('4', 'circle', 0, a, .35, 4, '/S3'),
                           ('5', 'circle', 0, -a, .35, 5, '/S4')])
    for i, (x, y, ex, ey, nid, nm) in enumerate(
            [(a, 0, 8, 0, 2, '/S1'), (-a, 0, -8, 0, 3, '/S2'),
             (0, a, 0, 8, 4, '/S3'), (0, -a, 0, -8, 5, '/S4')]):
        b += (f'  (via (at {CX + x} {CY + y}) (size 0.35) (drill 0.2) '
              f'(layers "F.Cu" "B.Cu") (net {nid}))\n'
              f'  (segment (start {CX + x} {CY + y}) (end {CX + ex} {CY + ey}) '
              f'(width 0.1) (layer "B.Cu") (net {nid}))\n')
        b += fp(f'P{i + 1}', CX + ex, CY + ey, [('1', 'rect', 0, 0, .4, nid, nm)],
                layer='B.Cu')
    if stub:
        b += (f'  (segment (start {CX} {CY}) (end {STUB_VIA[0]} {STUB_VIA[1]}) '
              f'(width 0.1) (layer "F.Cu") (net 1))\n')
    b += (f'  (via (at {STUB_VIA[0]} {STUB_VIA[1]}) (size 0.35) (drill 0.2) '
          f'(layers "F.Cu" "B.Cu") (net 1))\n')
    b += fp('TP1', CX, CY, [('1', 'rect', 0, 0, .2, 7, '/TP')], layer='B.Cu')
    b += (fp('X1', 104, 126, [('1', 'rect', 0, 0, .5, 6, '/X')])
          + fp('X2', 112, 126, [('1', 'rect', 0, 0, .5, 6, '/X')]))
    if control:
        # G3 in the open flood; its stub grazes X3 (0.08 mm < 0.1) on the way
        # to its via.
        b += fp('G3', 130, 104, [('1', 'rect', 0, 0, 1, 1, 'GND')])
        b += ('  (segment (start 130 104) (end 132 104) (width 0.1) (layer "F.Cu") (net 1))\n'
              '  (via (at 132 104) (size 0.35) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1))\n')
        b += fp('X3', 131, 104.23, [('1', 'rect', 0, 0, .2, 6, '/X')])
    b += zone('F.Cu') + zone('In1.Cu')
    for x1, y1, x2, y2 in ((100, 100, 140, 100), (140, 100, 140, 130),
                           (140, 130, 100, 130), (100, 130, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    return b + ')\n'


def write_board(path, a, stub, hole, control=False, project=True):
    """Write the board, and (project=True) its project: Default class 0.09,
    so the stub (0.093 mm from the pins) is legal on this board."""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(board_text(a, stub, control))
    if project:
        with open(path[:-len('kicad_pcb')] + 'kicad_pro', 'w', encoding='utf-8') as f:
            json.dump({"board": {"design_settings": {"rules": {
                "min_clearance": 0.09, "min_hole_clearance": hole}}},
                "net_settings": {"classes": [
                    {"name": "Default", "clearance": 0.09, "track_width": 0.1,
                     "via_diameter": 0.35, "via_drill": 0.2}]}}, f)
    return path


FAILURES = []


def check(cond, what, detail=''):
    print(('  ok    ' if cond else '  FAIL  ') + what
          + ('' if cond or not detail else '  -- ' + detail))
    if not cond:
        FAILURES.append(what)


def main():
    from kicad_parser import parse_kicad_pcb
    from pcb_modification import prune_grazing_segments
    from plane_fill_model import (ZoneFillModel, get_zone_model,
                                  trusted_zone_model, fill_hole_clearance)

    tmp = tempfile.mkdtemp(prefix='krt_1581_')

    def load(name, *args, **kw):
        pcb = parse_kicad_pcb(write_board(os.path.join(tmp, name), *args, **kw))
        gnd = next(n for n, v in pcb.nets.items() if v.name == 'GND')
        fz = next(z for z in pcb.zones if z.net_id == gnd and z.layer == 'F.Cu')
        return pcb, gnd, fz

    def joined(m, a=(CX, CY), b=STUB_VIA):
        ca, cb = m.query_component(*a), m.query_component(*b)
        return bool(ca) and ca == cb

    print('the mechanism (preconditions -- if one fails the fixture no longer '
          'tests the bug)')
    pcb, gnd, fz = load('ring.kicad_pcb', 0.45, True, 0.15, control=True)
    check(joined(get_zone_model(pcb, fz)),
          "the default model joins U1's GND pocket to the flood through the "
          "0.086 mm gap")
    check(not joined(trusted_zone_model(pcb, fz, {})),
          'the trusted (conservative) model does not: KiCad pours nothing '
          'there')
    check(joined(trusted_zone_model(pcb, fz, {}), (130.0, 104.0), (132.0, 104.0)),
          'the trusted model joins G3 to the flood: it is not blind to open fill')

    def stubs():
        return {(round(s.start_x, 3), round(s.start_y, 3))
                for s in pcb.segments if s.net_id == gnd}

    print('\nthe pass')
    check({(CX, CY), (130.0, 104.0)} <= stubs(), 'both stubs are on the board')
    n, _nets, removed = prune_grazing_segments(
        [], pcb, {gnd}, clearance=0.1, check_foreign_segments=True)
    check((CX, CY) in stubs(),
          "U1's stub is kept: it is the pad's only join KiCad makes",
          'removed %d segment(s)' % n)
    check((130.0, 104.0) not in stubs() and n == 1,
          "G3's grazing stub is still removed: KiCad's fill surely joins G3",
          'removed %d segment(s): %s' % (n, [(s.start_x, s.start_y) for s in removed]))

    print('\nhole clearance')
    pcb, gnd, fz = load('hole.kicad_pcb', 0.72, False, 0.4)
    check(abs(fill_hole_clearance(pcb) - 0.4) < 1e-9,
          'the project\'s min_hole_clearance is read (0.4)',
          str(fill_hole_clearance(pcb)))
    check(joined(ZoneFillModel(pcb, fz, conservative=True, hole_clearance=0.0)),
          'without hole clearance the 0.468 mm copper gap passes the '
          'conservative model')
    check(not joined(trusted_zone_model(pcb, fz, {})),
          'with it (0.018 mm drill-to-drill gap) the trusted model does not '
          'join the pocket, as KiCad does not')
    pcb, gnd, fz = load('hole_lo.kicad_pcb', 0.72, False, 0.05)
    check(joined(trusted_zone_model(pcb, fz, {})),
          'at min_hole_clearance 0.05 it does join (KiCad does too)')
    pcb, _g, _z = load('noproj.kicad_pcb', 0.72, False, 0.05, project=False)
    check(abs(fill_hole_clearance(pcb) - 0.25) < 1e-9,
          'a board with no project fills at pcbnew\'s factory 0.25',
          str(fill_hole_clearance(pcb)))

    print()
    if FAILURES:
        print('FAILED: %d check(s): %s' % (len(FAILURES), FAILURES))
        return 1
    print('PASS: the graze prune keeps a join only KiCad-sure fill would replace')
    return 0


if __name__ == '__main__':
    sys.exit(main())
