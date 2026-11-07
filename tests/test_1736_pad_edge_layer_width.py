#!/usr/bin/env python3
"""#1736: route_diff's maps stamp pads and the board edge at the layer's width.

The diff engine's maps reserve each layer's impedance width in their track
and via stamps (`reserve_layer_widths`), since its pose router has no track
margin to carry a width on. Pads and the board edge stamped the call's
`--track-width`, so a member at its layer width could come
(layer width - track width) / 2 inside a foreign pad's clearance or the edge
clearance, and the graze neck then narrowed it beside the pad.

A two-layer board, F.Cu 0.45 mm wide, B.Cu at the 0.15 track width. Past a
foreign pad on each layer and past the board edge, on the map with no pair
offset (the connector and hybrid-leg base) and on a map that bakes no layer
width (the single-ended engine's):
  * on F.Cu a cell the layer width's track would graze is blocked;
  * on B.Cu, at the call's width, the same cell is open;
  * a map that bakes no width stamps the call's width on both layers.

    python3 tests/test_1736_pad_edge_layer_width.py
"""
import contextlib
import io
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

BOARD = """(kicad_pcb (version 20240108) (generator "t")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "FOREIGN")
  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts"))
  (footprint "T:P" (layer "F.Cu") (at 10 4)
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "FOREIGN"))
    (pad "2" smd rect (at 0 0) (size 1 1) (layers "B.Cu") (net 1 "FOREIGN")))
)
"""

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _cfg(reserve):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=TW, diff_pair_gap=GAP,
                           clearance=CLR, grid_step=STEP, board_edge_clearance=EDGE,
                           via_size=0.6, via_drill=0.3,
                           layer_widths={'F.Cu': WIDE}, reserve_layer_widths=reserve)


def main():
    from kicad_parser import parse_kicad_pcb
    from obstacle_map import build_base_obstacle_map
    td = tempfile.mkdtemp(prefix='t1736pe_')
    try:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(BOARD)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(evidence(path, 'the staged board'))
            diff_map = build_base_obstacle_map(pcb, _cfg(True), [])
            se_map = build_base_obstacle_map(pcb, _cfg(False), [])
    finally:
        shutil.rmtree(td, ignore_errors=True)

    def g(mm):
        return int(round(mm / STEP))

    # The pad's edge is at y = 4.5: the keep-out reaches width/2 + clearance
    # past it -- 0.375 at the layer width, 0.225 at the call's. The board
    # edge's reaches edge clearance + width/2 from x = 0 -- 0.425 and 0.275.
    # Each cell sits halfway between the two reaches.
    rows = [('foreign pad', (10.0, 4.80)), ('board edge', (0.35, 15.0))]
    for what, (x, y) in rows:
        check(f"{what}: on the wide layer the cell a layer-width member would graze "
              f"is blocked", diff_map.is_blocked(g(x), g(y), 0))
        check(f"{what}: on the layer at the call's width the same cell is open",
              not diff_map.is_blocked(g(x), g(y), 1))
        check(f"{what}: a map that bakes no layer width stamps the call's width",
              not se_map.is_blocked(g(x), g(y), 0) and not se_map.is_blocked(g(x), g(y), 1))

    if FAILS:
        print(f"\nFAIL: {len(FAILS)} row(s): {FAILS}")
        return 1
    print("\npads and the board edge reserve the layer's width on the diff engine's maps. OK")
    return 0


if __name__ == '__main__':
    sys.exit(main())
