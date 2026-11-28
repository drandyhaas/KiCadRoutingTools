#!/usr/bin/env python3
"""#1842: a micro, blind or buried via is an end of its net only on the layers
it joins.

Three places offered every via of a net as an end on EVERY routing layer:
the anchor split a net rescue aims its search with, the tap points of a
multi-point net, and "this pad is reached on every layer" for a pad with a
same-net via in it. A search seeded on a layer the via does not reach laid
copper that joined nothing (measured on a 4-layer board routed with micro
vias: 15 rescue routes undone, 3 nets left open). A through via reads as
before.

Checks, on the 8-layer fixture board (tests/fixtures/1842) with a micro via
F.Cu-In1.Cu added to one net in memory:
  1. `connectivity.get_net_endpoints_anchor_split` offers the via's cell on
     F.Cu and In1.Cu only, and on all eight layers once the via is a
     through via;
  2. `single_ended_routing.get_all_segment_tap_points` likewise;
  3. `single_ended_routing._pad_all_layer_reach` is False for a pad with the
     micro via in it and True with a through via.
"""
import contextlib
import io
import os
import sys
from dataclasses import replace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))
sys.path.insert(0, TESTS)
from run_utils import evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb, Via  # noqa: E402

BOARD = os.path.join(TESTS, 'fixtures', '1842', 'hdi_repro.kicad_pcb')
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu', 'B.Cu']
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def main():
    from connectivity import get_net_endpoints_anchor_split
    from routing_config import GridRouteConfig, GridCoord
    from single_ended_routing import get_all_segment_tap_points, _pad_all_layer_reach
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(BOARD, 'the fixture board'))
    cfg = GridRouteConfig(track_width=0.1, clearance=0.1, via_size=0.46, via_drill=0.2,
                          grid_step=0.1, layers=list(STACK))
    coord = GridCoord(0.1)
    net = next(n for n in pcb.nets.values() if n.name == 'S01')
    pads = pcb.pads_by_net[net.net_id]
    ok(len(pads) == 2 and not pcb.vias, f"S01 has {len(pads)} pads and the board no via")
    a, b = pads
    # beside pad a, clear of it
    micro = Via(x=round(a.global_x + 0.4, 4), y=round(a.global_y + 0.4, 4), size=0.25,
                drill=0.1, layers=['F.Cu', 'In1.Cu'], net_id=net.net_id, via_type='micro')
    through = replace(micro, via_type='through', layers=['F.Cu', 'B.Cu'])
    cell = coord.to_grid(micro.x, micro.y)

    def split_layers(via):
        pcb.vias[:] = [via]
        try:
            sa, sb, err = get_net_endpoints_anchor_split(
                pcb, net.net_id, cfg, (a.global_x, a.global_y), (b.global_x, b.global_y))
        finally:
            pcb.vias[:] = []
        return err, sorted({STACK[e[2]] for e in sa + sb if (e[0], e[1]) == cell},
                           key=STACK.index)

    print("1. the anchor split")
    err, got = split_layers(micro)
    ok(not err and got == ['F.Cu', 'In1.Cu'],
       f"the micro via's cell is an end on its two layers: {got}")
    err, got = split_layers(through)
    ok(not err and got == STACK, f"a through via's on every layer: {len(got)} of {len(STACK)}")

    print("2. the tap points")
    taps = sorted((STACK[t[2]] for t in get_all_segment_tap_points(
        [], coord, STACK, vias=[micro])), key=STACK.index)
    n_through = len(get_all_segment_tap_points([], coord, STACK, vias=[through]))
    ok(taps == ['F.Cu', 'In1.Cu'] and n_through == len(STACK),
       f"the micro via taps on {taps}, a through via on {n_through} layers")

    print("3. a pad with a via in it")
    reach = {}
    for name, via in (('micro', micro), ('through', through)):
        pcb.vias[:] = [replace(via, x=a.global_x, y=a.global_y)]
        try:
            reach[name] = _pad_all_layer_reach(pcb, a)
        finally:
            pcb.vias[:] = []
    ok(reach == {'micro': False, 'through': True},
       f"reached on every layer: {reach}")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
