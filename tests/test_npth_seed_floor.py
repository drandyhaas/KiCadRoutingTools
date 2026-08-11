#!/usr/bin/env python3
"""Issue #390: plane-repair route seeds must honor the NPTH-to-track floor.

Region-join pseudo-anchors and KiCad-oracle island seeds are stamped
source/target cells, which OVERRIDE the obstacle map's NPTH drill keep-out.
The fill-validity margin ladder relaxed below the fab floor, approving a seed
on real zone fill 0.15mm from an NPTH hole edge -- the 0.2-wide GNDR strap
terminating there shipped a copper-to-hole violation (crkbd vs rEXSW1).

Now `npth_floor_ok` enforces drill/2 + NPTH_TO_TRACK_CLEARANCE + track_half
non-relaxably inside `_real_fill_point` and `_island_seed_points`.

    python3 tests/test_npth_seed_floor.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from plane_region_connector import npth_floor_ok, _real_fill_point
from kicad_oracle import _island_seed_points
from routing_defaults import NPTH_TO_TRACK_CLEARANCE


def _npth(x, y, drill):
    return SimpleNamespace(pad_type='np_thru_hole', drill=drill,
                           global_x=x, global_y=y, hole_x=None, hole_y=None,
                           net_id=0, layers=['*.Cu', '*.Mask'],
                           size_x=drill, size_y=drill, shape='circle',
                           rect_rotation=0.0)


def _board(npths):
    return SimpleNamespace(pads_by_net={0: list(npths)}, vias=[], segments=[],
                           zones=[])


def run():
    fails = []

    def check(name, cond):
        print(('PASS' if cond else 'FAIL') + f': {name}')
        if not cond:
            fails.append(name)

    # NPTH drill 1.9 at origin; floor for a 0.2-wide track = 0.95+0.2+0.1=1.25
    pcb = _board([_npth(0.0, 0.0, 1.9)])
    check('inside the floor rejected', not npth_floor_ok(1.20, 0.0, pcb, 0.1))
    check('outside the floor accepted', npth_floor_ok(1.30, 0.0, pcb, 0.1))
    check('width widens the floor',
          npth_floor_ok(1.30, 0.0, pcb, 0.1) and not npth_floor_ok(1.30, 0.0, pcb, 0.25))

    # _real_fill_point: NPTH floor is NOT relaxable by a small margin. Zone
    # square (-5..5); point on fill 1.2 from the hole center passes the
    # relaxed margin geometry but must still be rejected.
    poly = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
    ok = _real_fill_point((1.20, 0.0), 4, pcb, [poly], 'B.Cu', 0.15,
                          npth_track_half=0.1)
    check('_real_fill_point enforces the floor at relaxed margin', not ok)
    ok = _real_fill_point((2.0, 0.0), 4, pcb, [poly], 'B.Cu', 0.15,
                          npth_track_half=0.1)
    check('_real_fill_point keeps clear fill valid', ok)

    # _island_seed_points: bare fill cells inside the floor are dropped, the
    # rest survive; the raw reported point is dropped too when violating.
    pcb2 = _board([_npth(0.0, 0.0, 1.9)])
    step = 0.25
    cells = {(4, 0), (8, 0)}  # (1.0, 0) inside floor; (2.0, 0) clear
    pts = _island_seed_points(cells, step, pcb2, 4, 'B.Cu',
                              ['F.Cu', 'B.Cu'], (1.0, 0.0), track_half=0.1)
    xs = {round(p[0], 2) for p in pts}
    check('violating fill cell dropped', 1.0 not in xs)
    check('clear fill cell kept', 2.0 in xs)
    pts = _island_seed_points(cells, step, pcb2, 4, 'B.Cu',
                              ['F.Cu', 'B.Cu'], (2.0, 0.0), track_half=0.1)
    check('clear reported point kept',
          any(round(p[0], 2) == 2.0 for p in pts))

    _d11_obstacle_map_reads_the_board(check)

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


# --- D11: the ROUTER's NPTH keep-out must read min_hole_clearance too --------
#
# obstacle_map priced NPTH holes at a hardcoded
# max(config.clearance, NPTH_TO_TRACK_CLEARANCE) -- a flat 0.20 fab floor that
# never read the board -- while check_drc DOES read min_hole_clearance
# (check_drc.py:2390). So on a board declaring 0.25 the router routed into a
# band its own checker then flagged. Measured: a route came within 0.2263 mm of
# BUS1's NPTH against a declared 0.25 -- a real 0.0237 mm violation,
# routing-introduced, confirmed independently by kicad-cli, and the single DRC
# failure on that board.

def _declaring_board(tmp, **rules):
    """A board file whose sibling project declares board constraints."""
    import json
    p = os.path.join(tmp, 'b.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20240108))\n')
    with open(os.path.join(tmp, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': rules}}}, f)
    return p


def _d11_obstacle_map_reads_the_board(check):
    import tempfile
    import obstacle_map
    from routing_config import GridRouteConfig

    print()
    print('D11: the NPTH track keep-out reads the board, not a constant')

    with tempfile.TemporaryDirectory() as tmp:
        quiet_dir = os.path.join(tmp, 'silent')
        os.makedirs(quiet_dir, exist_ok=True)
        declares = _declaring_board(tmp, min_hole_clearance=0.25)
        silent = _declaring_board(quiet_dir)          # declares no constraints

        cfg = GridRouteConfig()
        cfg.clearance = 0.15
        obstacle_map._HOLE_CLR_CACHE.clear()

        pcb = SimpleNamespace(source_path=declares)
        check('a board declaring 0.25 resolves to 0.25',
              obstacle_map.resolve_hole_clearance(pcb, cfg) == 0.25)

        quiet = SimpleNamespace(source_path=silent)
        check('a board declaring nothing resolves to 0.0 (fab floor applies)',
              obstacle_map.resolve_hole_clearance(quiet, cfg) == 0.0)

        check('an in-memory board with no source_path resolves to 0.0',
              obstacle_map.resolve_hole_clearance(
                  SimpleNamespace(source_path=''), cfg) == 0.0)

        cfg_x = GridRouteConfig()
        cfg_x.clearance = 0.15
        cfg_x.hole_clearance = 0.4
        check('an explicit config value wins over the board',
              obstacle_map.resolve_hole_clearance(pcb, cfg_x) == 0.4)

        # THE POINT: the clearance actually handed to the keep-out stamper.
        # 0.20 (fab) and 0.15 (routing) are both below the declared 0.25, so
        # the old expression yielded 0.20 -- 0.05mm too close, which is how a
        # 0.2263mm approach to a 0.25 hole got built.
        captured = []
        real = obstacle_map.block_track_cells_near_drills
        real_via = obstacle_map.block_via_cells_near_drills
        obstacle_map.block_track_cells_near_drills = (
            lambda o, holes, tw, clr, step, layers: captured.append(clr))
        obstacle_map.block_via_cells_near_drills = (
            lambda *a, **k: None)
        try:
            for board, label, want in ((declares, 'declared 0.25', 0.25),
                                       (silent, 'nothing declared', 0.20)):
                captured.clear()
                obstacle_map.add_drill_hole_obstacles(
                    _FakeObstacles(), _npth_board(board), cfg, set())
                check(f'NPTH keep-out priced at {want} when {label}',
                      captured and abs(captured[0] - want) < 1e-9,
                      )
                if captured:
                    print(f'        priced at {captured[0]:.4g}mm')
        finally:
            obstacle_map.block_track_cells_near_drills = real
            obstacle_map.block_via_cells_near_drills = real_via


class _FakeObstacles:
    """add_drill_hole_obstacles only passes this through to the stampers."""


def _npth_board(path):
    pad = _npth(10.0, 10.0, 2.0)
    pad.pad_number, pad.component_ref = 'H1', 'BUS1'
    pad.local_clearance = 0.0
    return SimpleNamespace(pads_by_net={0: [pad]}, vias=[], segments=[],
                           zones=[], source_path=path)


if __name__ == '__main__':
    sys.exit(run())
