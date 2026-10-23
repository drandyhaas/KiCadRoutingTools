#!/usr/bin/env python3
"""Two shortcuts in the quench's candidate gate, held to the forms they replace.

* `RuleAreaKeepouts._near` answers a pad far from a keep-out from a cached
  floor: the clearance at the centre of its `NEAR_CELL_MM` cell, less the
  distance to that centre (a signed distance moves no faster than its point).
  It must answer exactly as the walk of the area's rings does -- for pads far
  from the area, beside it, inside it and inside a hole in it.
* `floorplan.nearest_rail_cap(pin, caps, stop_at=L)` returns the first cap
  within L: within L exactly when the true minimum is, and the minimum itself
  otherwise. Without `stop_at` it is the minimum `(gap, ref)` as before. The
  quench's tether gate passes its limit there, because it asks only whether
  some cap is within it.

    python3 tests/test_quench_gate_bounds.py
"""
import math
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan, legality  # noqa: E402

FAILS = []


def check(ok, what):
    print(f"  {'PASS' if ok else 'FAIL'}  {what}")
    if not ok:
        FAILS.append(what)


def near_by_walk(ko, ai, rect):
    """`_near` as it was: the bbox miss, then one walk of the rings."""
    bx = ko.areas[ai]['bbox']
    m = ko.band + ko.grid_step
    x0, y0, x1, y1 = rect[:4]
    if x1 < bx[0] - m or x0 > bx[2] + m or y1 < bx[1] - m or y0 > bx[3] + m:
        return False
    reach = max(math.hypot(x1 - x0, y1 - y0), ko.grid_step * math.sqrt(2.0)) / 2.0
    return ko._clear(ko.areas[ai], (x0 + x1) / 2.0,
                     (y0 + y1) / 2.0) < ko.band + reach + 1e-9


def keepout_floor():
    path = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
    pcb = parse_kicad_pcb(path)
    ko = legality.RuleAreaKeepouts.for_board(pcb, 0.2, path)
    check(ko.active and len(ko.areas) == 1, "glasgow_revC has one keep-out area")
    # A synthetic area with a hole, so pads inside the region and inside the
    # hole are asked too.
    outer = [(10.0, 10.0), (40.0, 10.0), (40.0, 30.0), (10.0, 30.0)]
    hole = [(20.0, 15.0), (30.0, 15.0), (30.0, 25.0), (20.0, 25.0)]
    ko.areas.append({'index': len(ko.areas), 'outer': outer, 'holes': [hole],
                     'rings': [outer, hole], 'covered': ko.areas[0]['covered'],
                     'bbox': (10.0, 10.0, 40.0, 30.0)})
    bx = pcb.board_info.board_bounds
    rng = random.Random(1)
    walks = [0]
    clear = ko._clear

    def counted(a, x, y):
        walks[0] += 1
        return clear(a, x, y)
    ko._clear = counted
    asked = same = 0
    for ai in range(len(ko.areas)):
        ab = ko.areas[ai]['bbox']
        for k in range(6000):
            # half anywhere on the board, half around the area's own box
            if k % 2:
                lo_x, lo_y, hi_x, hi_y = ab[0] - 6, ab[1] - 6, ab[2] + 6, ab[3] + 6
            else:
                lo_x, lo_y, hi_x, hi_y = bx
            cx, cy = rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y)
            w, h = rng.uniform(0.1, 3.0), rng.uniform(0.1, 3.0)
            rect = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
            asked += 1
            got = ko._near(ai, rect)
            ko._clear = clear
            want = near_by_walk(ko, ai, rect)
            ko._clear = counted
            same += (got == want)
    check(same == asked, f"_near answers as the ring walk on {same}/{asked} pads "
                         f"(both areas, holes included)")
    # One part's visit: 2000 pad rects within 3 mm of the centre of the
    # synthetic area's hole, 5 mm from its edges -- inside the area's box, so
    # the box test cannot answer. Each 1 mm cell is walked once; every pad
    # after is answered by its floor.
    walks[0] = 0
    ko._cell_clear.clear()
    agree = 0
    for _ in range(2000):
        cx, cy = 25.0 + rng.uniform(-3, 3), 20.0 + rng.uniform(-3, 3)
        rect = (cx - 0.4, cy - 0.3, cx + 0.4, cy + 0.3)
        got = ko._near(1, rect)
        ko._clear = clear
        agree += got == near_by_walk(ko, 1, rect)
        ko._clear = counted
    check(agree == 2000 and 0 < walks[0] <= 49,
          f"a clustered visit: {walks[0]} ring walks for 2000 pads, all answered alike")


def rail_cap_stop():
    pcb = parse_kicad_pcb(os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb'))
    fps = pcb.footprints
    caps_on = {}
    for ref, fp in fps.items():
        if ref.startswith('C'):
            for p in fp.pads:
                if p.net_id > 0:
                    caps_on.setdefault(p.net_id, set()).add(ref)
    pins = [(p, sorted(caps_on[p.net_id])) for ref, fp in sorted(fps.items())
            if ref.startswith('U') for p in fp.pads
            if p.net_id > 0 and len(caps_on.get(p.net_id, ())) >= 2]
    check(len(pins) > 50, f"glasgow_revC: {len(pins)} IC pins with two or more caps on their net")
    rng = random.Random(2)
    exact = stops = cases = 0
    for pin, refs in pins:
        caps = [fps[r] for r in refs]
        rng.shuffle(caps)
        gaps = [(g, c.reference) for c in caps
                if (g := floorplan._pin_gap(pin, c, pin.net_id)) is not None]
        want = min(gaps) if gaps else None
        exact += floorplan.nearest_rail_cap(pin, caps) == want
        if want is None:
            continue
        for lim in (want[0] - 0.1, want[0], want[0] + 0.1,
                    rng.uniform(want[0] - 2, want[0] + 2)):
            got = floorplan.nearest_rail_cap(pin, iter(caps), stop_at=lim)
            cases += 1
            if want[0] <= lim:
                stops += got is not None and got[0] <= lim
            else:
                stops += got == want
    check(exact == len(pins), f"without stop_at: the minimum (gap, ref), {exact}/{len(pins)}")
    check(stops == cases, f"with stop_at: within the limit exactly when the minimum is, "
                          f"else the minimum ({stops}/{cases})")


def main():
    keepout_floor()
    rail_cap_stop()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
