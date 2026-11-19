#!/usr/bin/env python3
"""#2004: the KiCad-oracle's rescue-ladder copper is graded as written.

The oracle's weld escalation and its exact-fill strap commit a
`net_rescue._attempt_edge` result straight to the board. That rescue routes a
fine-grid A* whose map rounds an off-grid foreign track, and an exact-fill
endpoint seeds a cell beside the fill's edge, so the copper as written
could sit inside clearance of a track the map called clear. On zynq_ad9364
a 0.0889 GND strap shipped 7 and 14 um inside 0.09 of an off-grid
ETH_RXD0 fanout track. Both paths now refuse such copper through
`kicad_oracle.rescue_copper_clear` (the main link path's
`wide_route_clear` measurement), as the main path already declines a
grazing link.

Checks, on an in-repo board with an off-grid foreign track laid in a clear
spot, no routing:
  1. a strap 14 um inside clearance of it is refused
  2. the same strap moved clear of it passes
  3. two separate clean segments pass: the gate measures each segment and
     invents no leg between them (that leg would cross the track)
  4. mixed widths are graded each at its own width
  5. both oracle paths that commit rescue copper call the gate
  6. `wide_route_clear` refuses a leg that crosses a foreign track mid-span
     (it measured endpoint distances only, so a crossing read as clear)
  7. a round pad is measured as its circle (check_drc's pad shape), not its
     bounding square, which refused legal copper beside it
"""
import contextlib
import io
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from kicad_parser import parse_kicad_pcb, Segment                 # noqa: E402
from routing_config import GridRouteConfig                        # noqa: E402
from kicad_oracle import rescue_copper_clear                      # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
LAYER = 'B.Cu'
CLR = 0.09
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def clear_spot(pcb, need=2.0):
    """A point on LAYER with no copper, pad or edge within `need` mm."""
    x0, y0, x1, y1 = pcb.board_info.board_bounds
    pads = [p for ps in pcb.pads_by_net.values() for p in ps]
    best = None
    y = y0 + need
    while y < y1 - need:
        x = x0 + need
        while x < x1 - need:
            d = min([math.hypot(p.global_x - x, p.global_y - y)
                     - max(p.size_x, p.size_y) / 2 for p in pads] + [99])
            for s in pcb.segments:
                if s.layer == LAYER:
                    d = min(d, math.hypot((s.start_x + s.end_x) / 2 - x,
                                          (s.start_y + s.end_y) / 2 - y)
                            - math.hypot(s.end_x - s.start_x,
                                         s.end_y - s.start_y) / 2)
            for v in pcb.vias:
                d = min(d, math.hypot(v.x - x, v.y - y) - v.size / 2)
            if d > need and (best is None or d > best[0]):
                best = (d, x, y)
            x += 0.5
        y += 0.5
    return best


def seg(x1, y1, x2, y2, w, net):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=LAYER, net_id=net)


def main():
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    gnd = next(i for i, n in pcb.nets.items() if n.name.strip('/') == 'GND')
    other = next(i for i, n in pcb.nets.items()
                 if i not in (0, gnd) and n.pads)
    spot = clear_spot(pcb)
    expect(spot is not None, "the board has a clear spot for the fixture")
    if spot is None:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    _, cx, cy = spot
    # The foreign track: off-grid, along x at y = cy + 0.000291
    fy = cy + 0.000291
    pcb.segments.append(seg(cx - 1.0 + 0.004879, fy, cx + 1.0 + 0.004879, fy,
                            0.12, other))
    cfg = GridRouteConfig(clearance=CLR, track_width=0.0889)
    w = 0.0889

    # 1. the strap's copper edge 0.0759 from the track's: 14 um inside 0.09
    off = 0.06 + w / 2 + 0.0759
    graze = [seg(cx - 0.5, fy - off, cx + 0.5, fy - off, w, gnd)]
    expect(not rescue_copper_clear(graze, pcb, gnd, cfg),
           "a strap 14 um inside clearance of an off-grid track is refused")

    # 2. moved clear (0.095 copper gap)
    off2 = 0.06 + w / 2 + 0.095
    clean = [seg(cx - 0.5, fy - off2, cx + 0.5, fy - off2, w, gnd)]
    expect(rescue_copper_clear(clean, pcb, gnd, cfg),
           "the same strap 0.095 clear of it passes")

    # 3. two separate clean segments either side of the track: a leg joining
    #    them would cross it, and none is invented
    two = [seg(cx - 0.5, fy - off2, cx, fy - off2, w, gnd),
           seg(cx, fy + off2, cx + 0.5, fy + off2, w, gnd)]
    expect(rescue_copper_clear(two, pcb, gnd, cfg),
           "two separate clean segments pass (no leg invented between them)")

    # 4. mixed widths: the wide one is graded at its own width
    wide_off = 0.06 + 0.25 / 2 + 0.05          # 0.05 gap at 0.25 wide
    mixed = clean + [seg(cx - 0.5, fy + wide_off, cx + 0.5, fy + wide_off,
                         0.25, gnd)]
    expect(not rescue_copper_clear(mixed, pcb, gnd, cfg),
           "a wide segment is graded at its own width (0.05 gap refused)")
    expect(rescue_copper_clear([], pcb, gnd, cfg),
           "no segments: nothing to refuse")

    # 6. wide_route_clear itself: a leg that CROSSES the track mid-span is
    #    refused (its four endpoint distances alone read it as clear)
    from plane_region_connector import wide_route_clear
    expect(not wide_route_clear([(cx, fy - 0.3, LAYER), (cx, fy + 0.3, LAYER)],
                                w, pcb, gnd, cfg),
           "wide_route_clear refuses a leg crossing a foreign track mid-span")

    # 7. a round pad is measured as its circle, not its bounding square:
    #    copper on the pad's 45-degree diagonal, inside the square but 0.12
    #    clear of the circle, passes; 0.05 from the circle is refused
    rp = next(q for ps in pcb.pads_by_net.values() for q in ps
              if q.shape == 'circle' and q.net_id not in (0, gnd)
              and not getattr(q, 'polygons', None))
    R = rp.size_x / 2

    def diag_seg(gap):
        dd = (R + w / 2 + gap) / math.sqrt(2)
        mx, my = rp.global_x + dd, rp.global_y + dd
        return [Segment(start_x=mx - 0.01, start_y=my + 0.01,
                        end_x=mx + 0.01, end_y=my - 0.01, width=w,
                        layer='F.Cu', net_id=gnd)]
    expect(rescue_copper_clear(diag_seg(0.12), pcb, gnd, cfg),
           f"copper 0.12 clear of round pad {rp.component_ref}.{rp.pad_number}'s "
           f"circle passes (it is inside the pad's bounding square)")
    expect(not rescue_copper_clear(diag_seg(0.05), pcb, gnd, cfg),
           "copper 0.05 from the circle is refused")

    # 5. both oracle paths that commit rescue copper call the gate
    src = open(os.path.join(ROOT, 'py_router', 'kicad_oracle.py'),
               encoding='utf-8').read()
    calls = re.findall(r"rescue_copper_clear\(\s*_esc2?\.get\('new_segments'\)",
                       src)
    expect(len(calls) == 2,
           f"the weld escalation and the exact-fill strap both gate ({len(calls)} call(s))")

    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: #2004 oracle rescue copper graded as written")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
