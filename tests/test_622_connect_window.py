#!/usr/bin/env python3
"""awx/connect.connect's search WINDOW keeps a via's reach inside it at the lane's outermost point.

  python3 tests/test_622_connect_window.py

make_local_window installs the window's bounds as the board's, so the obstacle builders keep every via its edge
clearance off the window's edge. connect() built the window as its points and a margin -- 0.6 by default, the braid's
route_lane -- which is less than a via's half size and the board-edge clearance (0.225 + 0.4 at sdi_mipi_conv's rules):
a via at the lane's outermost point had no cell (SDI_D2 on four layers, its change 1.4 mm short of its berth, the
lane's easternmost point: refused both rounds). On a bare board at those rules, with route_lane's margin, this pins:

1. a lane from F.Cu to B.Cu at ONE point -- its only via site the window's centre and its outermost point at once --
   is connected, by a via there;
2. a lane whose change is at its start, the outermost point of its points along x: the via stands at the start itself,
   not a grid step in from it.
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('ROUTE_LAYERS', None)
import connect as cn  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = ('(kicad_pcb (version 20240108) (generator "test")\n  (general (thickness 1.6))\n'
         '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
         '  (net 0 "")\n  (net 1 "A")\n'
         '  (gr_rect (start 0 0) (end 50 50) (stroke (width 0.05) (type solid)) (fill none) (layer "Edge.Cuts"))\n)\n')

fails = []


def check(cond, msg):
    print(('PASS ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, 'b.kicad_pcb')
    open(p, 'w').write(BOARD)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(p)
    cfg = cn.make_config(pcb, 0.15, 0.1, 0.45, 0.2, grid_step=0.025, board_edge_clearance=0.4)
    if not (cfg.via_size / 2 + cfg.board_edge_clearance > 0.6):
        raise SystemExit(f'BROKEN TEST: a via\'s edge reach {cfg.via_size / 2 + cfg.board_edge_clearance} is within '
                         f'the margin it was drawn for')
    # 1.
    with contextlib.redirect_stdout(io.StringIO()):
        r = cn.connect(pcb, 1, (25.0, 25.0), 'F.Cu', (25.0, 25.0), 'B.Cu', cfg, margin=0.6)
    vias = [(round(v.x, 3), round(v.y, 3)) for v in r[1]] if r else None
    check(vias == [(25.0, 25.0)], f'1: F.Cu to B.Cu at one point, by a via there: {vias}')
    # 2.
    with contextlib.redirect_stdout(io.StringIO()):
        r = cn.connect(pcb, 1, (20.0, 25.0), 'F.Cu', (25.0, 25.0), 'B.Cu', cfg, margin=0.6)
    vias = [(round(v.x, 3), round(v.y, 3)) for v in r[1]] if r else None
    check(vias == [(20.0, 25.0)], f'2: the change at the start, the outermost point: {vias}')
print('=' * 60)
print(f'{len(fails)} FAILED' if fails else 'ALL PASS')
sys.exit(1 if fails else 0)
