#!/usr/bin/env python3
"""#1729: a pair's centreline map reserves each layer's own member half-pitch.

A member rides (layer width + gap) / 2 off the centreline, so the centreline
map stamps every obstacle with that offset. One value for every layer cannot
serve them all: the call's, (track_width + gap) / 2, let the outer member on
a layer wider than the track width reach into a neighbour's clearance (it was
then necked alone), and the widest layer's reserved room on a narrower layer
that no member there needs -- on an impedance stackup whose outer microstrip
is far wider than its stripline, a pair could no longer leave its pads.

A two-layer board, the outer layer 0.45 mm wide and the other at the 0.15
track width, gap 0.15: half-pitch 0.30 on the wide layer, 0.15 on the other.
Around a foreign track, a foreign pad on each layer, a foreign via and the
board edge, a cell the narrow layer's member clears at its own pitch is open
on that layer, while the cell the same distance off on the wide layer stays
blocked -- and a cell the call's pitch would open there is blocked too
(the wide layer keeps #1729's reservation). FAILS on a90b210c: every narrow
layer cell is blocked at the wide layer's half-pitch.

  * the value: a plain float, the call's half-pitch, when every layer shares
    one width; otherwise each layer's own, read through `extra_on_layer`,
    with the call's as its number (what a via barrel, across the stack,
    keeps);
  * hole stamps: each layer reaches its own member's far edge at a 45-degree
    bend; a map that bakes no layer width is unchanged;
  * the map: the rows above.

    python3 tests/test_1729_centreline_per_layer.py
"""
import contextlib
import io
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence  # noqa: E402

TW, GAP, CLR, EDGE, STEP = 0.15, 0.15, 0.15, 0.2, 0.05
WIDE = 0.45
HP_WIDE, HP_NARROW = (WIDE + GAP) / 2, (TW + GAP) / 2

BOARD = """(kicad_pcb (version 20240108) (generator "t")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "FOREIGN")
  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts"))
  (segment (start 5 10) (end 15 10) (width 0.2) (layer "F.Cu") (net 1))
  (segment (start 5 10) (end 15 10) (width 0.2) (layer "B.Cu") (net 1))
  (via (at 15 16) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (footprint "T:P" (layer "F.Cu") (at 10 4)
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "FOREIGN"))
    (pad "2" smd rect (at 0 0) (size 1 1) (layers "B.Cu") (net 1 "FOREIGN")))
)
"""


def _cfg(layer_widths, reserve=True):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=TW, diff_pair_gap=GAP,
                           clearance=CLR, grid_step=STEP, board_edge_clearance=EDGE,
                           via_size=0.6, via_drill=0.3,
                           layer_widths=layer_widths, reserve_layer_widths=reserve)


def t_value(td):
    from diff_pair_routing import centreline_extra_clearance
    from routing_config import extra_on_layer
    v = centreline_extra_clearance(_cfg({}))
    assert type(v) is float and v == (TW + GAP) / 2, repr(v)
    v = centreline_extra_clearance(_cfg({'F.Cu': TW, 'B.Cu': TW}))
    assert type(v) is float and v == (TW + GAP) / 2, repr(v)
    v = centreline_extra_clearance(_cfg({'F.Cu': WIDE}))
    assert abs(float(v) - HP_NARROW) < 1e-12, repr(v)
    assert abs(extra_on_layer(v, 'F.Cu') - HP_WIDE) < 1e-12, repr(v)
    assert abs(extra_on_layer(v, 'B.Cu') - HP_NARROW) < 1e-12, repr(v)
    assert extra_on_layer(0.2, 'F.Cu') == 0.2
    print("  ok  the call's half-pitch as a plain float with one width; per layer otherwise")


def t_hole_stamps(td):
    from diff_pair_routing import centreline_extra_clearance
    from obstacle_map import _hole_stamp_widths
    cfg = _cfg({'F.Cu': WIDE})
    mitre = 1.0 / math.cos(math.radians(22.5))
    reach = {i: w / 2 + ex for (w, ex), idxs
             in _hole_stamp_widths(cfg, centreline_extra_clearance(cfg)).items()
             for i in idxs}
    want = {0: WIDE / 2 + HP_WIDE * mitre, 1: TW / 2 + HP_NARROW * mitre}
    assert all(abs(reach[i] - want[i]) < 1e-9 for i in want), (reach, want)
    se = _cfg({'F.Cu': WIDE}, reserve=False)
    assert _hole_stamp_widths(se, 0.05) == {(TW + 2 * 0.05 * (mitre - 1), 0.05): [0, 1]}, \
        _hole_stamp_widths(se, 0.05)
    print("  ok  hole stamps: each layer reaches its own member's far edge; "
          "a map with no baked width unchanged")


def t_map(td):
    from diff_pair_routing import centreline_extra_clearance
    from kicad_parser import parse_kicad_pcb
    from obstacle_map import build_base_obstacle_map
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(path, 'the staged board'))
    cfg = _cfg({'F.Cu': WIDE})
    with contextlib.redirect_stdout(io.StringIO()):
        obs = build_base_obstacle_map(pcb, cfg, [], centreline_extra_clearance(cfg))

    def g(mm):
        return int(round(mm / STEP))
    # (what, wide-layer cell, narrow-layer cell). The wide layer's cell is
    # inside its own reach and outside the reach the call's half-pitch gives
    # it; the narrow layer's is outside its own reach and inside the reach
    # the wide layer's half-pitch gives it.
    rows = [
        # track: reach reserve/2 + 0.1 + clr + half-pitch -- wide 0.775
        # (call's 0.625), narrow 0.475 (at the wide pitch 0.625)
        ('foreign track', (10.0, 10.70), (10.0, 10.55)),
        # pad: reach track/2 + clr + half-pitch past its edge (y 4.5) --
        # wide 0.525 (0.375), narrow 0.375 (0.525)
        ('foreign pad', (10.0, 4.95), (10.0, 4.95)),
        # via: reach via/2 + reserve/2 + clr + half-pitch in whole cells --
        # wide 20 (17), narrow 14 (17)
        ('foreign via', (15.0 - 0.90, 16.0), (15.0 - 0.80, 16.0)),
        # edge: reach edge + track/2 + half-pitch from x = 0 -- wide 0.575
        # (0.425), narrow 0.425 (0.575)
        ('board edge', (0.50, 15.0), (0.50, 15.0)),
    ]
    for what, (wx, wy), (nx, ny) in rows:
        assert obs.is_blocked(g(wx), g(wy), 0), \
            f"{what}: the wide layer's cell at ({wx}, {wy}) is open"
        assert not obs.is_blocked(g(nx), g(ny), 1), (
            f"{what}: the narrow layer's cell at ({nx}, {ny}) is blocked -- "
            f"reserved at the wide layer's half-pitch")
    print("  ok  the map: past a track, a pad, a via and the board edge the narrow "
          "layer is held at its own half-pitch, the wide layer at its own")


CASES = [t_value, t_hole_stamps, t_map]


def main():
    want = set(sys.argv[1:])
    for fn in CASES:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='t1729pl_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
