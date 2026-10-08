#!/usr/bin/env python3
"""The oracle's exact-fill strap never starts inside the run's board-edge band
(#1168).

sonde_xilinx declares `min_copper_edge_clearance 0.01`; route.py pins 0.2 and
routes at it, but writes the project only at the end, so the oracle's exact
refill (inside the run) still filled to 0.01 and its seeds -- fill interior
points inset by track_half + 0.05 -- started at y = 67.759 on a board edge at
67.31. A seed cell overrides the static edge keep-out, so the strap ended at
(130.1, 67.8): SEGMENT-BOARD-EDGE 0.028, KiCad copper_edge_clearance 0.1725
against a 0.2 rule.

Rows: `seeds_clear_of_edge` drops exactly the seeds inside edge + track_half
(the issue's seed among them) and keeps the rest, against an Edge.Cuts ring and
against bare board bounds; with no edge floor nothing is dropped; the tier
passes the obstacle map's own edge value (board_edge_clearance, else
clearance) to both exact-fill calls.

    python3 tests/test_1168_oracle_seed_edge.py
"""
import ast
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_oracle import seeds_clear_of_edge, _seed_edge_clearance   # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


EDGE_Y = 67.31
OUTLINE = [(100.0, 40.0), (160.0, 40.0), (160.0, EDGE_Y), (100.0, EDGE_Y)]
TRACK_HALF = 0.635 / 2
EDGE = 0.2
need = EDGE_Y - EDGE - TRACK_HALF          # 66.7925: the last legal centre y
seeds = [(130.1, 67.759, 'B.Cu'),          # the issue's seed (cell 67.8)
         (130.1, need + 0.01, 'B.Cu'),     # just inside the band
         (130.1, need - 0.01, 'B.Cu'),     # just clear of it
         (130.1, 60.0, 'B.Cu')]
ring = NS(board_info=NS(board_outlines=[OUTLINE], board_outline=OUTLINE,
                        board_cutouts=[], board_bounds=(100, 40, 160, EDGE_Y)))
kept = seeds_clear_of_edge(seeds, ring, EDGE, TRACK_HALF)
check('against the Edge.Cuts ring: the band is dropped, the rest kept',
      kept == seeds[2:], str(kept))
bare = NS(board_info=NS(board_outlines=[], board_outline=[], board_cutouts=[],
                        board_bounds=(100, 40, 160, EDGE_Y)))
check('against bare board bounds: the same',
      seeds_clear_of_edge(seeds, bare, EDGE, TRACK_HALF) == seeds[2:])
check('the issue seed is dropped at the run floor but would pass at the '
      "project's 0.01 (the refill's rule)",
      seeds[0] not in kept
      and seeds[0] in seeds_clear_of_edge(seeds[:1], ring, 0.01, TRACK_HALF))
check('no edge floor: nothing dropped',
      seeds_clear_of_edge(seeds, ring, 0.0, TRACK_HALF) == seeds)
check("the seed band mirrors the obstacle map's edge value",
      _seed_edge_clearance(NS(board_edge_clearance=0.2, clearance=0.1)) == 0.2
      and _seed_edge_clearance(NS(board_edge_clearance=0.0, clearance=0.1)) == 0.1)

src = open(os.path.join(ROOT, 'py_router', 'kicad_oracle.py'), encoding='utf-8').read()
calls = [n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)
         and getattr(n.func, 'id', '') == '_exact_fill_endpoints']
check('both exact-fill calls pass the edge floor',
      len(calls) == 2 and all(any(k.arg == 'edge_clearance' for k in c.keywords)
                              for c in calls), str([c.lineno for c in calls]))

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
