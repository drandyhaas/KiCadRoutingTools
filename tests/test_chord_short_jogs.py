#!/usr/bin/env python3
"""chord_short_jogs: hypotenuse-fill same-net short 135-deg jogs.

A grid jog's outer cap routinely lands EXACTLY tangent to adjacent same-net
copper; KiCad's connection_width checker then reports a sub-minimum web
around the float-dust micro-void (measured live: 0.079 mm on copper that is
provably >= 0.2 mm wide everywhere). The chord makes the jog convex so the
tangency class cannot form, and its sliver triangle cannot legally contain
foreign copper.

    python3 tests/test_chord_short_jogs.py
"""
import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Segment
from pcb_modification import chord_short_jogs

NET = 7


def _seg(x1, y1, x2, y2, net=NET, w=0.2, layer='F.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def _pcb(segs):
    return SimpleNamespace(segments=list(segs))


def run():
    passed = failed = 0

    def check(name, cond):
        nonlocal passed, failed
        passed += bool(cond)
        failed += not cond
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    # The live ADCFE /VREF2 jog: long E approach, 0.2 straight + 0.2828
    # diagonal down to a via; the diagonal's cap sits exactly tangent to the
    # E approach's edge (KiCad flagged 0.079 mm there).
    east = _seg(31.5, 59.4, 27.7, 59.4)
    n_arm = _seg(27.7, 59.4, 27.7, 59.2)          # 1.0 w
    ne_arm = _seg(27.7, 59.2, 27.9, 59.0)         # 1.41 w
    pcb = _pcb([east, n_arm, ne_arm])
    results = [{'net_id': NET, 'new_segments': [n_arm, ne_arm], 'new_vias': []}]
    n = chord_short_jogs(results, pcb, {NET})
    check("one chord for the live jog", n == 1)
    g = [s for s in pcb.segments if s not in (east, n_arm, ne_arm)]
    check("chord joined the board", len(g) == 1)
    if g:
        g = g[0]
        eps = {(round(g.start_x, 3), round(g.start_y, 3)),
               (round(g.end_x, 3), round(g.end_y, 3))}
        check("chord spans the jog's far endpoints",
              eps == {(27.7, 59.4), (27.9, 59.0)})
        check("chord at the narrower arm's width", g.width == 0.2)
        check("chord on the jog's layer + net", g.layer == 'F.Cu' and g.net_id == NET)
    check("chord lands in the write-list as a cleanup result",
          any(r.get('cleanup') == 'jog_chord' for r in results))

    # A REAL elbow (long arms at 135 deg) is untouched.
    a = _seg(0, 0, 3, 0)
    b = _seg(0, 0, -2.12, 2.12)
    pcb = _pcb([a, b])
    results = [{'net_id': NET, 'new_segments': [a, b], 'new_vias': []}]
    check("long-arm 135 elbow untouched", chord_short_jogs(results, pcb, {NET}) == 0)

    # 90-deg jogs are untouched (no tangency class there).
    a = _seg(0, 0, 0.2, 0)
    b = _seg(0.2, 0, 0.2, 0.2)
    pcb = _pcb([a, b])
    results = [{'net_id': NET, 'new_segments': [a, b], 'new_vias': []}]
    check("90-deg jog untouched", chord_short_jogs(results, pcb, {NET}) == 0)

    # A purely-input jog is left alone (this run did not make it).
    a = _seg(0, 0, 0.2, 0)
    b = _seg(0.2, 0, 0.34, -0.14)
    pcb = _pcb([a, b])
    results = [{'net_id': NET, 'new_segments': [], 'new_vias': []}]
    check("input-only jog untouched", chord_short_jogs(results, pcb, {NET}) == 0)

    # If the hypotenuse already exists as copper, no duplicate is added.
    a = _seg(0, 0, 0.2, 0)
    b = _seg(0.2, 0, 0.34, -0.14)
    hyp = _seg(0, 0, 0.34, -0.14)
    pcb = _pcb([a, b, hyp])
    results = [{'net_id': NET, 'new_segments': [a, b, hyp], 'new_vias': []}]
    check("existing hypotenuse not duplicated",
          chord_short_jogs(results, pcb, {NET}) == 0)

    # Out-of-scope net untouched.
    a = _seg(0, 0, 0.2, 0, net=9)
    b = _seg(0.2, 0, 0.34, -0.14, net=9)
    pcb = _pcb([a, b])
    results = [{'net_id': 9, 'new_segments': [a, b], 'new_vias': []}]
    check("out-of-scope net untouched", chord_short_jogs(results, pcb, {NET}) == 0)

    # Determinism: same input -> same chords, twice.
    def build():
        a = _seg(31.5, 59.4, 27.7, 59.4)
        b = _seg(27.7, 59.4, 27.7, 59.2)
        c = _seg(27.7, 59.2, 27.9, 59.0)
        pcb = _pcb([a, b, c])
        res = [{'net_id': NET, 'new_segments': [b, c], 'new_vias': []}]
        chord_short_jogs(res, pcb, {NET})
        return sorted((s.start_x, s.start_y, s.end_x, s.end_y, s.width)
                      for s in pcb.segments)
    check("deterministic emission", build() == build())

    print(f"{passed}/{passed + failed} chord_short_jogs tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run())
