#!/usr/bin/env python3
"""keep_input_copper: input-file copper is read-only for the cleanup passes.

Chained flows (fanout -> route, or an external stage authoring escape stubs)
need the input file's own copper to survive a routing run verbatim - including
dead-end stubs of nets the run FAILED to route, which the #84 sweep otherwise
deletes. The flag makes every subtractive/rewriting cleanup pass treat input
copper as read-only anchors while this run's new copper is cleaned normally.

    python3 tests/test_keep_input_copper.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import PCBData, BoardInfo, Segment, Pad
from routing_config import GridRouteConfig
from pcb_modification import sweep_dead_ends
from cleanup_pipeline import run_post_route_cleanup

NET = 5


def _seg(x1, y1, x2, y2, layer='F.Cu', net=NET, width=0.2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=width,
                   layer=layer, net_id=net)


def _pad(ref, x, y, net=NET):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=0.5, size_y=0.5, shape='circle',
               layers=['F.Cu'], net_id=net, net_name='N')


def make_board():
    """pad A --a-- J --b-- pad B (live input path), J --stub-- dead tip (an
    authored input escape stub), plus a THIS-RUN dead spur off pad B."""
    pa, pb = _pad('A', 0, 0), _pad('B', 10, 0)
    a = _seg(0, 0, 5, 0)
    b = _seg(5, 0, 10, 0)
    stub = _seg(5, 0, 5, 3)            # input-file escape stub (dead end)
    run_spur = _seg(10, 0, 10, 4)      # this-run dead spur
    pcb = PCBData(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=None),
                  nets={}, footprints={},
                  vias=[], segments=[a, b, stub, run_spur],
                  pads_by_net={NET: [pa, pb]})
    results = [{'net_id': NET, 'new_segments': [run_spur], 'new_vias': []}]
    return pcb, results, a, b, stub, run_spur


def run():
    passed = failed = 0

    def check(name, cond):
        nonlocal passed, failed
        passed += bool(cond)
        failed += not cond
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    # --- sweep_dead_ends, default: BOTH dead ends removed; input stub stripped.
    pcb, results, a, b, stub, run_spur = make_board()
    n_segs, _n_vias, strip = sweep_dead_ends(results, pcb, {NET})
    check("default: both dead ends swept", n_segs == 2)
    check("default: input stub in the writer strip list", strip == [stub])
    check("default: this-run spur dropped from the write-list",
          results[0]['new_segments'] == [])

    # --- sweep_dead_ends, keep_input_copper: input stub survives everywhere;
    # this-run spur still swept.
    pcb, results, a, b, stub, run_spur = make_board()
    n_segs, _n_vias, strip = sweep_dead_ends(results, pcb, {NET},
                                             keep_input_copper=True)
    check("flag: exactly the this-run spur swept", n_segs == 1)
    check("flag: strip list empty (no input copper removed)", strip == [])
    check("flag: input stub still on the board",
          any(s is stub for s in pcb.segments))
    check("flag: this-run spur dropped from the write-list",
          results[0]['new_segments'] == [])
    check("flag: live input path untouched",
          any(s is a for s in pcb.segments) and any(s is b for s in pcb.segments))

    # --- full pipeline: strip lists stay empty by construction under the flag,
    # and this-run copper is still cleaned.
    pcb, results, a, b, stub, run_spur = make_board()
    cfg = GridRouteConfig(grid_step=0.1, clearance=0.15, via_size=0.6,
                          track_width=0.2, via_drill=0.3,
                          layers=['F.Cu', 'B.Cu'])
    out = run_post_route_cleanup(results, pcb, {NET}, cfg,
                                 keep_input_copper=True)
    check("pipeline flag: input_strip_segments empty",
          out.input_strip_segments == [])
    check("pipeline flag: input_strip_vias empty", out.input_strip_vias == [])
    check("pipeline flag: input stub survives the whole pipeline",
          any(s is stub for s in pcb.segments))
    check("pipeline flag: this-run spur still cleaned",
          results[0]['new_segments'] == [])

    # --- full pipeline, default: the input stub is stripped (the #84 baseline
    # this flag opts out of - guards against the flag accidentally becoming
    # default-on).
    pcb, results, a, b, stub, run_spur = make_board()
    out = run_post_route_cleanup(results, pcb, {NET}, cfg)
    check("pipeline default: input stub stripped",
          any(s is stub for s in out.input_strip_segments))

    print(f"{passed}/{passed + failed} keep_input_copper tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run())
