#!/usr/bin/env python3
"""A GND return via (add_gnd_vias) clears foreign tracks EXACTLY, not to the
grid's resolution.

The placement checked each candidate on the obstacle map's CELLS: a track is
stamped as the cells whose centres lie inside its keep-out, and the via as a
disc of cells, so both edges are quantised to the grid and a via could sit up
to half a cell into a track the cells called clear. At a 0.1 mm grid a 0.45 mm
return via landed 0.006-0.010 mm inside the clearance of a signal track that
widens from 0.081 to 0.100 mm on In2.Cu (and 0.015 mm inside a 0.25 mm power
trunk's). A site the grid accepts is now measured once more, exactly: the
via's ring against every foreign track and pad, at the clearance check_drc
grades the pair at.

The geometry is the case's: the signal track's taper and bend, as routed, and
a signal via of that net placed so the search's FIRST candidate (distance
0.54, angle 0, snapped to the grid) is the case's site (144.90, 93.30), which
grazes the track's 0.100 mm segment (0.0095 mm at the 0.0889 mm the board was
graded at, 0.0107 at this step's 0.09). On the routed board the cells accepted
that site; here the map is EMPTY, standing for cells that accept it, so the
exact measurement is the only thing between the site and the via. The config
is the one route_planes builds for this step (all six copper layers, via
0.45/0.2, track 0.0889, clearance 0.09, grid 0.1).

Checks:
  A. precondition: the case's site grazes the track, measured here with
     check_drc's own via-to-segment test
  B. a return via is still placed for the signal via
  C. every placed via clears every foreign track exactly

    python3 tests/test_gnd_via_exact_clearance.py
"""
import contextlib
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Via, Net, PCBData, BoardInfo, Segment       # noqa: E402
from routing_config import GridRouteConfig, GridCoord               # noqa: E402
from obstacle_map import GridObstacleMap   # noqa: E402  (puts rust_router on sys.path)
from add_gnd_vias import add_gnd_vias_to_existing_board             # noqa: E402
from check_drc import check_via_segment_overlap                     # noqa: E402

FAILS = []
CLEARANCE = 0.09
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']
SIG, GND = 1, 2
# The signal track as routed: (x1, y1, x2, y2, width), all on In2.Cu.
TRACK = [
    (142.5000, 93.2000, 144.3000, 93.2000, 0.0762),
    (144.3000, 93.2000, 144.6000, 93.5000, 0.0762),
    (144.6000, 93.5000, 144.6167, 93.5167, 0.08096),
    (144.6167, 93.5167, 144.6333, 93.5333, 0.08572),
    (144.6333, 93.5333, 144.6500, 93.5500, 0.09048),
    (144.6500, 93.5500, 144.6667, 93.5667, 0.09524),
    (144.6667, 93.5667, 144.8000, 93.7000, 0.1),
    (144.8000, 93.7000, 145.4667, 93.7000, 0.1),
    (145.4667, 93.7000, 145.5500, 93.7000, 0.09524),
    (145.5500, 93.7000, 145.6333, 93.7000, 0.09048),
]
SITE = (144.90, 93.30)


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def worst_overlap(pcb, via):
    """check_drc's via-to-segment overlap, the worst over foreign tracks."""
    worst = 0.0
    for s in pcb.segments:
        if s.net_id == via.net_id:
            continue
        bad, ov = check_via_segment_overlap(via, s, CLEARANCE, clearance_margin=0.0)
        if bad:
            worst = max(worst, ov)
    return worst


def main():
    segs = [Segment(start_x=a, start_y=b, end_x=c, end_y=d, width=w, layer='In2.Cu',
                    net_id=SIG) for a, b, c, d, w in TRACK]
    # 0.54 to the left of the site: the first candidate the search tries
    sig_via = Via(x=SITE[0] - 0.54, y=SITE[1], size=0.25, drill=0.15,
                  layers=['F.Cu', 'B.Cu'], net_id=SIG)
    pcb = PCBData(board_info=BoardInfo(layers={}, copper_layers=list(LAYERS),
                                       board_bounds=(140.0, 90.0, 150.0, 97.0)),
                  nets={0: Net(net_id=0, name='', pads=[]),
                        SIG: Net(net_id=SIG, name='SIG', pads=[]),
                        GND: Net(net_id=GND, name='GND', pads=[])},
                  footprints={}, vias=[sig_via], segments=segs,
                  pads_by_net={0: [], SIG: [], GND: []})
    cfg = GridRouteConfig(via_size=0.45, via_drill=0.2, track_width=0.0889,
                          clearance=CLEARANCE, grid_step=0.1, layers=list(LAYERS),
                          hole_to_hole_clearance=0.25)
    obstacles = GridObstacleMap(len(LAYERS))        # cells that accept every site
    with contextlib.redirect_stdout(io.StringIO()):
        placed = add_gnd_vias_to_existing_board(pcb, 'GND', 2.0, cfg, obstacles,
                                                GridCoord(cfg.grid_step))

    print('A. the case site')
    site = Via(x=SITE[0], y=SITE[1], size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'],
               net_id=GND)
    check('precondition: the site grazes the track', worst_overlap(pcb, site) > 0.005,
          f'{worst_overlap(pcb, site):.4f} mm')
    print('B. placement')
    check('a return via is placed', len(placed) == 1, f'{len(placed)}')
    print('C. exact clearance')
    for v in placed:
        check(f'the via at ({v.x:.2f}, {v.y:.2f}) clears every foreign track',
              worst_overlap(pcb, v) <= 1e-6, f'overlap {worst_overlap(pcb, v):.4f} mm')


if __name__ == '__main__':
    main()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')
