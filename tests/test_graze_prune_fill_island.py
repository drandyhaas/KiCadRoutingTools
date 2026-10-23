#!/usr/bin/env python3
"""The graze prune never drops a fill island's only link.

prune_grazing_segments drops a segment that grazes foreign copper when the net
is no worse connected without it. It grades that with a zone-credit VALIDATOR
(0.25 mm clear of foreign copper or no pour credit), which is conservative for
the pad it would over-credit and the opposite for the copper being removed: a
tap whose end lies in a narrow fill island loses its end's credit, reads as a
dead end, the island's pad reads as already open, and the removal looks free.

Measured on a GND ball of a 0.5 mm csBGA, which sits in an F.Cu fill
island between the RAM fanout tracks: the router reached it with a short F.Cu
tail through a gap too narrow for fill, and this pass dropped the tail in every
route step (route.py's #209 gate: "post-routing cleanup DISCONNECTED ... GND
-> 3 pad(s) now unconnected"). The ball shipped open, KiCad agreeing.

The board below is that case in miniature. GND pad P sits in a box of SIG
track whose top has a 0.26 mm opening: under the 0.3 mm a 0.1 mm-clearance,
0.1 mm-thick fill needs, over the 0.3 - 0.04 a 0.1 mm track can thread, so
the GND track that enters it grazes both sides. Its end is 0.24 mm from a wall:
fill reaches it, the validator does not. Q and R are GND pads in the main fill.

Without the fix the pass drops the tap and P opens (checked against the
pre-fix pass when this test was written).

    python3 -X utf8 tests/test_graze_prune_fill_island.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

WALLS = [((8.5, 8.5), (8.5, 11.5)), ((11.5, 8.5), (11.5, 11.5)),
         ((8.5, 11.5), (11.5, 11.5)), ((8.5, 8.5), (9.77, 8.5)),
         ((10.23, 8.5), (11.5, 8.5))]
TAP = ((10.0, 6.0), (10.0, 8.75))   # from pad Q, through the opening
#: A GND stub at R, written BEFORE the tap. check_net_connectivity unions a fill
#: component's points onto its FIRST point, and analyze_conn_excluding drops
#: every edge of an excluded segment's points -- so were the tap's start that
#: hub, excluding it would split the main fill and block the removal for a
#: reason unrelated to the island (on the measured GND an earlier segment is).
STUB = ((4.0, 4.0), (6.0, 4.0))


def _seg(a, b, w, net):
    return ('  (segment (start %g %g) (end %g %g) (width %g) (layer "F.Cu") (net %d))\n'
            % (a[0], a[1], b[0], b[1], w, net))


BOARD = ('''(kicad_pcb (version 20241229) (generator "pcbnew") (general (thickness 1.6))
 (paper "A4")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (37 "F.SilkS" user "F.Silkscreen")
  (39 "F.Mask" user) (44 "Edge.Cuts" user))
 (setup)
 (net 0 "") (net 1 "GND") (net 2 "SIG")
 (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts"))
 (footprint "L:P" (layer "F.Cu") (at 0 0)
  (property "Reference" "U1" (at 0 -3 0) (layer "F.SilkS") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
  (pad "P" smd rect (at 10 10) (size 0.5 0.5) (layers "F.Cu" "F.Mask") (net 1 "GND"))
  (pad "Q" smd rect (at 10 6) (size 0.5 0.5) (layers "F.Cu" "F.Mask") (net 1 "GND"))
  (pad "R" smd rect (at 4 4) (size 0.5 0.5) (layers "F.Cu" "F.Mask") (net 1 "GND"))
  (pad "S" smd rect (at 16 16) (size 0.5 0.5) (layers "F.Cu" "F.Mask") (net 2 "SIG")))
''' + ''.join(_seg(a, b, 0.2, 2) for a, b in WALLS) + _seg(STUB[0], STUB[1], 0.2, 1)
       + _seg(TAP[0], TAP[1], 0.1, 1) + '''
 (zone (net 1) (net_name "GND") (layer "F.Cu") (uuid "00000000-0000-0000-0000-0000000000aa") (hatch edge 0.5)
  (connect_pads yes (clearance 0.1)) (min_thickness 0.1) (filled_areas_thickness no)
  (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
  (polygon (pts (xy 1 1) (xy 19 1) (xy 19 19) (xy 1 19))))
)
''')

FAILURES = []


def check(cond, what, detail=''):
    print(('  ok    ' if cond else '  FAIL  ') + what
          + ('' if cond or not detail else '  -- ' + detail))
    if not cond:
        FAILURES.append(what)


def main():
    from kicad_parser import parse_kicad_pcb
    from check_connected import check_net_connectivity, make_real_fill_validator
    from pcb_modification import prune_grazing_segments

    path = os.path.join(tempfile.mkdtemp(prefix='krt_graze_island_'), 'island.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD)
    pcb = parse_kicad_pcb(path)
    gnd = next(n for n, v in pcb.nets.items() if v.name == 'GND')
    pads = pcb.pads_by_net[gnd]
    zones = [z for z in pcb.zones if z.net_id == gnd]

    def gnd_segments():
        return [s for s in pcb.segments if s.net_id == gnd]

    def tap_segments():
        return [s for s in gnd_segments() if abs(s.start_y - s.end_y) > 1.0]

    def p_open(**kw):
        r = check_net_connectivity(gnd, gnd_segments(), [], pads, zones,
                                   pcb_data=pcb, **kw)
        return any(abs(x - 10.0) < 1e-6 and abs(y - 10.0) < 1e-6
                   for (x, y, _l, _r) in (r.get('disconnected_pads') or []))

    print('the case (preconditions -- if one fails the fixture no longer tests the bug)')
    check(len(tap_segments()) == 1, 'one GND tap into the box')
    check(not p_open(), 'graded with the pour, P is connected through the tap')
    check(p_open(zone_credit_validator=make_real_fill_validator(pcb, gnd)),
          "under the validator P is ALREADY open: the tap's end is denied credit")
    check(not p_open() and p_open(zone_credit_validator=make_real_fill_validator(pcb, gnd)),
          'so the two models disagree about exactly this pad')

    print('\nthe pass')
    n, _nets, _strip = prune_grazing_segments(
        [], pcb, {gnd}, clearance=0.1, check_foreign_segments=True)
    check(len(tap_segments()) == 1 and n == 0,
          'the grazing tap is kept: it is the island\'s only link',
          'removed %d segment(s)' % n)
    check(not p_open(), 'P is still connected after the prune')

    print()
    if FAILURES:
        print('FAILED: %d check(s): %s' % (len(FAILURES), FAILURES))
        return 1
    print('PASS: the graze prune keeps a fill island\'s only link')
    return 0


if __name__ == '__main__':
    sys.exit(main())
