#!/usr/bin/env python3
"""A fanout frame copy leaves the run's caches behind and keeps its state.

`bga_fanout`'s flipped and rotated frames work on a deep copy of the board.
Inside route.py the board also carries the run's caches -- the rescue keeps
its obstacle maps, native `GridObstacleMap` objects, in `_rescue_map_cache`
-- and the copy failed on them ("cannot pickle 'builtins.GridObstacleMap'
object"), so a fanout-rescue rung that reached a flipped or rotated array
failed every time. A cache describes the board in its own frame, so the copy
should not carry one at all (`rotate_frame.frame_copy`).

On the in-repo orangecrab board, with a real obstacle map in the cache:
  1. `flip_frame.to_front_frame` and `rotate_frame.to_axis_aligned_frame`
     both copy the board;
  2. the copy has no `_rescue_map_cache`, the original keeps its own;
  3. run state the fanout reads survives: `_copper_epoch` and the
     `_fanout_all_foreign_immovable` flag a rescue sets before it calls in.

    python3 tests/test_frame_copy_run_caches.py
"""
import contextlib
import io
import os
import sys
from collections import OrderedDict

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path[:0] = [os.path.join(REPO, 'py_router'), TESTS]

from run_utils import evidence  # noqa: E402

RUN_ALL_TIMEOUT = 300
BOARD = os.path.join(REPO, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def main():
    evidence(BOARD)
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from obstacle_map import build_base_obstacle_map
    from bga_fanout import flip_frame, rotate_frame
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    ref = max(pcb.footprints, key=lambda r: len(pcb.footprints[r].pads))
    nid = next(p.net_id for p in pcb.footprints[ref].pads if p.net_id)
    cfg = GridRouteConfig(layers=list(pcb.board_info.copper_layers), track_width=0.15,
                          clearance=0.15, grid_step=0.1)
    with contextlib.redirect_stdout(io.StringIO()):
        omap = build_base_obstacle_map(pcb, cfg, [nid])
    pcb._rescue_map_cache = OrderedDict({(nid,): (omap, None, None)})
    pcb._copper_epoch = 7
    pcb._fanout_all_foreign_immovable = True

    print(f'1. both frames copy a board whose rescue cache holds an obstacle map ({ref})')
    for name, fn in (('flip', lambda: flip_frame.to_front_frame(pcb, ref)),
                     ('rotate', lambda: rotate_frame.to_axis_aligned_frame(pcb, ref))):
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                rp = fn()[0]
            err = ''
        except Exception as ex:                                  # noqa: BLE001
            rp, err = None, f'{type(ex).__name__}: {ex}'
        check(f'{name}: the frame copy is made', rp is not None, err)
        if rp is None:
            continue
        print(f'2./3. {name}: what the copy carries')
        check(f'{name}: the copy carries no rescue cache',
              not hasattr(rp, '_rescue_map_cache'))
        check(f'{name}: the copy keeps the run state',
              getattr(rp, '_copper_epoch', None) == 7
              and getattr(rp, '_fanout_all_foreign_immovable', None) is True)
    check('the original keeps its own cache',
          pcb._rescue_map_cache[(nid,)][0] is omap)

    print('FAILED: ' + ', '.join(failures) if failures else 'ALL PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
