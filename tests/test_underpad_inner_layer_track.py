#!/usr/bin/env python3
"""The under-pad escape keeps its vias off tracks on the layers it does not route.

  python3 tests/test_underpad_inner_layer_track.py

A BGA fanned out on F.Cu and B.Cu of a 4-layer board still drills through
In1/In2: every via's barrel meets the copper there. The channel engine's
via-in-pad check (#370 B4) and the base obstacle map's out-of-config copper
counted it; the under-pad engine dropped every track on a layer outside its
escape set before its via checks ever saw it, and laid a via on an In2.Cu
VCC_3V3 track (zynq_ad9364, the bus step's source fan -- a short).

On ulx3s U1 (4 copper layers), under-pad on F.Cu + B.Cu:
  1. without the inner track, a chosen ball gets a via (the case is live:
     the site really is where the engine puts a via);
  2. a foreign track laid on In2.Cu through that via's centre, and the same
     fanout again: no via of another net stands within via ring + track +
     clearance of it.

Uses kicad_files/ulx3s.kicad_pcb; skips cleanly if absent.
"""
import contextlib
import io
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402

BOARD = os.path.join(ROOT, "kicad_files", "ulx3s.kicad_pcb")
LAYERS = ["F.Cu", "B.Cu"]                  # the escape set: the inner layers are NOT routed
PARAMS = dict(track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2,
              escape_method='underpad', plane_drop='off')
TRACK_W = 0.2


def fan(pcb):
    with contextlib.redirect_stdout(io.StringIO()):
        return generate_bga_fanout(pcb.footprints["U1"], pcb, layers=list(LAYERS), **PARAMS)


def seg_dist(px, py, s):
    dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - s.start_x) * dx + (py - s.start_y) * dy) / L2))
    return math.hypot(px - (s.start_x + t * dx), py - (s.start_y + t * dy))


def main():
    print("=" * 60)
    print("under-pad escape vs a track on a layer it does not route")
    print("=" * 60)
    if not os.path.exists(BOARD):
        print(f"  [SKIP] board not present: {BOARD}")
        return 0
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    assert "In2.Cu" in pcb.board_info.copper_layers, pcb.board_info.copper_layers
    _t, vias0, _r, _f = fan(pcb)
    # a via OFF the ball centre is the under-pad engine's own site choice; any
    # via will do -- the one nearest the array's centre, away from the edge rows
    fp = pcb.footprints["U1"]
    cx = sum(p.global_x for p in fp.pads) / len(fp.pads)
    cy = sum(p.global_y for p in fp.pads) / len(fp.pads)
    live = bool(vias0)
    print(f"  [{'PASS' if live else 'FAIL'}] without the inner track the fanout lays vias ({len(vias0)})")
    if not live:
        return 1
    v0 = min(vias0, key=lambda v: (math.hypot(v['x'] - cx, v['y'] - cy), v['x'], v['y']))
    # a foreign net: any net with no pad on U1 (so the fanout never routes it)
    u1_nets = {p.net_id for p in fp.pads}
    foreign = min(nid for nid in pcb.nets if nid and nid not in u1_nets)
    track = Segment(v0['x'] - 0.6, v0['y'], v0['x'] + 0.6, v0['y'], TRACK_W, "In2.Cu", foreign)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb2 = parse_kicad_pcb(BOARD)
    pcb2.segments.append(track)
    _t, vias1, _r, failed1 = fan(pcb2)
    need = PARAMS['via_size'] / 2 + TRACK_W / 2 + PARAMS['clearance'] - 1e-6
    hits = [v for v in vias1 if v['net_id'] != foreign and seg_dist(v['x'], v['y'], track) < need]
    ok = not hits
    print(f"  [{'PASS' if ok else 'FAIL'}] with an In2.Cu track of net {foreign} through "
          f"({v0['x']:.3f}, {v0['y']:.3f}): no via within {need:.3f} mm of it"
          + ('' if ok else f" -- {len(hits)} do: " + ', '.join(f"net {v['net_id']} at ({v['x']:.3f}, {v['y']:.3f})"
                                                                   for v in hits[:4])))
    print(f"  (the ball's net {v0['net_id']}: "
          f"{'refused' if v0['net_id'] in set(failed1) else 'escaped elsewhere'})")
    print("ALL PASS" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
