#!/usr/bin/env python3
"""A pad clearance override REPLACES the class/rule value (#530, decision 3).

KiCad's engine returns max(overrides) floored at rules.min_clearance BEFORE it
looks at a net class or a custom rule (drc_engine.cpp; measured on KiCad
10.0.0 by tests/oracle/constraint_agreement.py rows pad_override_below_class,
pad_override_beats_rule, pad_override_below_board_min). The tree used to
price it as max(base, override), which is right for a zone's local clearance
and wrong for a pad's -- 2932 pads on 48 corpus boards declare an override
BELOW their class.

Three checks, all on the harness's own synthetic board so check_drc is held
to the same boundary the harness measured:

  1. design_rules.override_clearance: below class wins, floored at min_clearance,
     max of two overrides, absent -> base
  2. check_drc: a track at override + eps from a pad (below the 0.2 class) is
     CLEAN; at override - eps it is flagged
  3. the router's obstacle stamp: a route cell at override + eps from that pad
     is free (was blocked under the old max semantics)
"""
import os
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

from design_rules import override_clearance  # noqa: E402
from constraint_agreement import write_board, _pad_and_track  # noqa: E402


def check(label, ok, fails):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    if not ok:
        fails.append(label)


def main():
    fails = []
    pad = SimpleNamespace(local_clearance=0.1)
    big = SimpleNamespace(local_clearance=0.5)
    none = SimpleNamespace(local_clearance=0.0)
    check('override below class replaces it', override_clearance(0.3, 0.05, pad) == 0.1, fails)
    check('override floored at the board minimum', override_clearance(0.3, 0.15, pad) == 0.15, fails)
    check('two overrides -> the larger', override_clearance(0.3, 0.05, pad, big) == 0.5, fails)
    check('no override -> base', override_clearance(0.3, 0.05, none, None) == 0.3, fails)

    from check_drc import run_drc
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'probe.kicad_pcb')
        for gap, expect_clean in ((0.1 + 0.01, True), (0.1 - 0.01, False)):
            fps, segs = _pad_and_track(gap, 0.1)
            write_board(board, footprints=fps, segments=segs, rules={'min_clearance': 0.05})
            viols = run_drc(board, clearance=0.2, quiet=True, check_sizes=False,
                            print_summary=False)
            clr = [v for v in viols if 'clearance' in str(v.get('type', ''))
                   or 'pad' in str(v.get('type', ''))]
            check(f"check_drc at override {'+' if expect_clean else '-'} eps "
                  f"({'clean' if expect_clean else 'flagged'} expected): {len(clr)} clearance violation(s)",
                  (not clr) if expect_clean else bool(clr), fails)

        # the router's obstacle stamp
        fps, segs = _pad_and_track(0.1 + 0.02, 0.1)
        write_board(board, footprints=fps, segments=[], rules={'min_clearance': 0.05})
        pcb = parse_kicad_pcb(board)
        from routing_config import GridRouteConfig
        from kicad_dru import install_layer_clearances
        from obstacle_map import build_base_obstacle_map
        cfg = GridRouteConfig(track_width=0.2, clearance=0.2, grid_step=0.05,
                              layers=['F.Cu', 'B.Cu'])
        install_layer_clearances(cfg, None, board, pcb)
        from routing_config import GridCoord
        coord = GridCoord(cfg.grid_step)
        obs = build_base_obstacle_map(pcb, cfg, nets_to_route=[2])
        # pad copper edge at y = 10.5; a 0.2 track centre at edge + 0.1 (override)
        # + 0.1 (half width) + 0.05 (one grid cell) = 10.75 must be free; under the
        # old max() semantics the pad was stamped at the 0.2 class, which blocks
        # every centre up to 10.8.
        gx, gy = coord.to_grid(10.0, 10.75)
        check('router obstacle stamp honours the override (cell free at override + 1 cell)',
              not obs.is_blocked(gx, gy, 0), fails)
        gx2, gy2 = coord.to_grid(10.0, 10.65)
        check('...and still blocks a centre inside the override', obs.is_blocked(gx2, gy2, 0), fails)

    if fails:
        print("FAIL: " + '; '.join(fails))
        return 1
    print("PASS: a pad override replaces the class/rule clearance (floored at the board "
          "minimum) in the resolver, in check_drc and in the router's stamps")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
