#!/usr/bin/env python3
"""
Test for issue #265: stub layer-swap optimization for multi-point nets.

Multi-point nets (3+ unconnected endpoints) used to be skipped by
`apply_single_ended_layer_swaps` with a "not yet supported" warning, so a net
whose fanout stubs sat on several layers always burned layer-change vias at
route time. The conservative collapse moves every movable stub onto one
common layer when the whole endpoint set allows it.

Scenarios (synthetic boards):
  1. Three dangling stubs on In1/In1/F.Cu collapse to F.Cu (2 moves, 0 new
     vias beats 1 move + 1 new pad via).
  2. Same board with can_swap_to_top_layer=False collapses to In1 instead,
     drilling the one pad via the F.Cu stub needs.
  3. A committed pad-to-pad trace (immovable) forces the collapse direction:
     the movable stubs join ITS layer, the committed copper never moves.
  4. Endpoints with no feasible common layer (committed In1 copper + bare
     F.Cu-only SMD pads) are left completely untouched.
  5. A layer already compatible with every endpoint (through-hole pads) means
     no swap at all.

Run:
    python3 tests/test_multipoint_layer_swap.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Segment, Via, Pad, Net
from routing_config import GridRouteConfig
from layer_swap_optimization import apply_single_ended_layer_swaps

MP_NET = 1


def _cfg():
    c = GridRouteConfig()
    c.layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    c.track_width = 0.1
    c.clearance = 0.15
    c.grid_step = 0.05
    c.via_size = 0.45
    c.via_drill = 0.3
    return c


def _pad(ref, num, x, y, layers=None, drill=0.0, pad_type=''):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5,
               shape='circle', layers=layers or ['F.Cu'], net_id=MP_NET,
               net_name='MP', drill=drill, pad_type=pad_type)


def _seg(x1, y1, x2, y2, layer):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=0.1, layer=layer, net_id=MP_NET)


def _via(x, y):
    return Via(x=x, y=y, size=0.45, drill=0.3,
               layers=['F.Cu', 'B.Cu'], net_id=MP_NET)


def _pcb(segments, vias, pads):
    return PCBData(
        footprints={}, segments=segments, vias=vias,
        nets={MP_NET: Net(MP_NET, 'MP')},
        board_info=None, pads_by_net={MP_NET: pads},
    )


def _mixed_stub_board():
    """Pads A/B: F.Cu pads with via-in-pad + dangling In1.Cu stubs.
    Pad C: F.Cu pad with a dangling F.Cu stub (no via). 3 endpoint groups
    spanning In1.Cu + F.Cu."""
    segments = [
        _seg(0.0, 0.0, 1.0, 0.0, 'In1.Cu'),    # A stub
        _seg(10.0, 0.0, 9.0, 0.0, 'In1.Cu'),   # B stub
        _seg(5.0, 5.0, 5.0, 4.0, 'F.Cu'),      # C stub
    ]
    vias = [_via(0.0, 0.0), _via(10.0, 0.0)]
    pads = [_pad('U1', '1', 0.0, 0.0),
            _pad('U2', '1', 10.0, 0.0),
            _pad('U3', '1', 5.0, 5.0)]
    return _pcb(segments, vias, pads)


def _run(pcb, cfg, can_swap_to_top=True):
    mods, swap_vias, swap_segs = [], [], []
    n = apply_single_ended_layer_swaps(
        pcb, cfg, [('MP', MP_NET)], can_swap_to_top, mods, swap_vias,
        verbose=True, all_swap_segments=swap_segs)
    return n, mods, swap_vias


def main():
    cfg = _cfg()
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- 1: collapse to F.Cu (fewest new vias wins over fewest moves) ------
    print("\nScenario 1: mixed In1/In1/F.Cu stubs, top layer allowed")
    pcb = _mixed_stub_board()
    n, mods, swap_vias = _run(pcb, cfg)
    layers = {s.layer for s in pcb.segments}
    check("two stubs moved", n == 2)
    check("all stubs on F.Cu", layers == {'F.Cu'})
    check("no new pad vias", len(swap_vias) == 0)
    check("modifications recorded for the writer", len(mods) == 2 and
          all(m['new_layer'] == 'F.Cu' for m in mods))

    # -- 2: top layer forbidden -> collapse to In1 with a pad via ----------
    print("\nScenario 2: same board, can_swap_to_top_layer=False")
    pcb = _mixed_stub_board()
    n, mods, swap_vias = _run(pcb, cfg, can_swap_to_top=False)
    layers = {s.layer for s in pcb.segments}
    check("one stub moved", n == 1)
    check("all stubs on In1.Cu", layers == {'In1.Cu'})
    check("pad via drilled at the F.Cu pad", len(swap_vias) == 1 and
          abs(swap_vias[0].x - 5.0) < 1e-6 and abs(swap_vias[0].y - 5.0) < 1e-6)

    # -- 3: committed copper forces the collapse direction -----------------
    print("\nScenario 3: committed In1 trace + two F.Cu stubs")
    # THT pads A-B already connected by an In1 trace with a dangling branch
    # (its free end is the group's endpoint, but the copper is committed);
    # C and D are F.Cu pads with dangling F.Cu stubs.
    segments = [
        _seg(0.0, 0.0, 5.0, 0.0, 'In1.Cu'),
        _seg(5.0, 0.0, 10.0, 0.0, 'In1.Cu'),
        _seg(5.0, 0.0, 5.0, 2.0, 'In1.Cu'),   # dangling branch off the trunk
        _seg(4.0, 6.0, 4.0, 5.0, 'F.Cu'),     # C stub
        _seg(6.0, -6.0, 6.0, -5.0, 'F.Cu'),   # D stub
    ]
    pads = [_pad('J1', '1', 0.0, 0.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole'),
            _pad('J1', '2', 10.0, 0.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole'),
            _pad('U3', '1', 4.0, 6.0),
            _pad('U4', '1', 6.0, -6.0)]
    pcb = _pcb(segments, [], pads)
    n, mods, swap_vias = _run(pcb, cfg)
    in1 = [s for s in pcb.segments if s.layer == 'In1.Cu']
    check("both F.Cu stubs moved to In1.Cu", n == 2 and len(in1) == 5)
    check("committed trace untouched",
          all(s.layer == 'In1.Cu' for s in pcb.segments[:3]))
    check("pad vias drilled at both moved F.Cu pads", len(swap_vias) == 2)

    # -- 4: no feasible common layer -> leave everything alone -------------
    print("\nScenario 4: committed In1 copper vs bare F.Cu-only pads")
    segments = [
        _seg(0.0, 0.0, 5.0, 0.0, 'In1.Cu'),
        _seg(5.0, 0.0, 10.0, 0.0, 'In1.Cu'),
        _seg(5.0, 0.0, 5.0, 2.0, 'In1.Cu'),   # dangling branch = group endpoint
    ]
    pads = [_pad('J1', '1', 0.0, 0.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole'),
            _pad('J1', '2', 10.0, 0.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole'),
            _pad('U3', '1', 4.0, 6.0),        # bare SMD, F.Cu only
            _pad('U4', '1', 6.0, -6.0)]       # bare SMD, F.Cu only
    pcb = _pcb(segments, [], pads)
    before = [(s.start_x, s.start_y, s.layer) for s in pcb.segments]
    n, mods, swap_vias = _run(pcb, cfg)
    after = [(s.start_x, s.start_y, s.layer) for s in pcb.segments]
    check("nothing moved", n == 0 and before == after)
    check("no vias, no modifications", not swap_vias and not mods)

    # -- 5: a compatible common layer already exists -> no-op --------------
    print("\nScenario 5: THT pad + two In1 stubs (In1 already fits all)")
    segments = [
        _seg(0.0, 0.0, 1.0, 0.0, 'In1.Cu'),
        _seg(10.0, 0.0, 9.0, 0.0, 'In1.Cu'),
    ]
    vias = [_via(0.0, 0.0), _via(10.0, 0.0)]
    pads = [_pad('U1', '1', 0.0, 0.0),
            _pad('U2', '1', 10.0, 0.0),
            _pad('J1', '1', 5.0, 5.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole')]
    pcb = _pcb(segments, vias, pads)
    n, mods, swap_vias = _run(pcb, cfg)
    check("no swap applied", n == 0 and not mods and not swap_vias)
    check("stubs still on In1.Cu",
          all(s.layer == 'In1.Cu' for s in pcb.segments))

    # -- 6: issue #429 -- lone THT pad + dangling stub -> MOVABLE ----------
    # A through-hole pad spans every copper layer, so check_net_connectivity's
    # pad_components lists one entry per layer for that single pad, all at the
    # SAME (x, y). _net_connected_pad_locs must NOT mistake that per-layer
    # expansion for >=2 mutually-connected pads: a bare THT pad (connector)
    # connected to nothing else is committed to NOTHING, so a fanout stub off it
    # is safe to relayer (the pad reaches all layers). Before the fix the stub
    # read as pinned/immovable.
    print("\nScenario 6: issue #429 -- lone THT pad + dangling stub is movable")
    from layer_swap_optimization import (
        _net_connected_pad_locs, _stub_free_end_is_open, STUB_POSITION_TOLERANCE)
    from stub_layer_switching import get_stub_info

    # The bug only bites when the net's copper spans >=2 layers, so the THT pad's
    # *.Cu expansion lands on multiple layers (one pad_components entry per layer,
    # same root). Here J1 is a lone THT connector pad with a dangling In1 stub;
    # U2 is a separate, UNCONNECTED F.Cu pad+stub that just puts F.Cu into the
    # net's layer set. J1 must still read as connected to nothing.
    seg_j1 = _seg(0.0, 0.0, 2.0, 0.0, 'In1.Cu')   # dangling In1 stub off the THT pad
    seg_u2 = _seg(20.0, 0.0, 22.0, 0.0, 'F.Cu')   # unrelated F.Cu stub (2nd layer)
    tht = _pad('J1', '1', 0.0, 0.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole')
    u2 = _pad('U2', '1', 20.0, 0.0, layers=['F.Cu'])
    pcb = _pcb([seg_j1, seg_u2], [], [tht, u2])
    conn = _net_connected_pad_locs(pcb, MP_NET, STUB_POSITION_TOLERANCE)
    check("lone THT pad's per-layer expansion is not counted as connected copper",
          (0.0, 0.0) not in conn)
    stub = get_stub_info(pcb, MP_NET, 2.0, 0.0, 'In1.Cu')
    check("stub off the THT pad is recognized", stub is not None and bool(stub.segments))
    movable = _stub_free_end_is_open(pcb, stub, STUB_POSITION_TOLERANCE, conn)
    check("stub off a bare THT pad is MOVABLE (relayerable)", movable is True)

    # Guard: a stub whose pad IS genuinely connected to another pad stays pinned.
    seg_a = _seg(0.0, 0.0, 5.0, 0.0, 'In1.Cu')   # committed A<->B trace on In1
    seg_b = _seg(5.0, 0.0, 10.0, 0.0, 'In1.Cu')
    branch = _seg(5.0, 0.0, 5.0, 2.0, 'In1.Cu')  # dangling branch off the trunk
    pa = _pad('J2', '1', 0.0, 0.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole')
    pb = _pad('J2', '2', 10.0, 0.0, layers=['*.Cu'], drill=0.8, pad_type='thru_hole')
    pcb2 = _pcb([seg_a, seg_b, branch], [], [pa, pb])
    conn2 = _net_connected_pad_locs(pcb2, MP_NET, STUB_POSITION_TOLERANCE)
    check("two THT pads joined by committed copper ARE connected",
          (0.0, 0.0) in conn2 and (10.0, 0.0) in conn2)
    stub2 = get_stub_info(pcb2, MP_NET, 5.0, 2.0, 'In1.Cu')
    movable2 = _stub_free_end_is_open(pcb2, stub2, STUB_POSITION_TOLERANCE, conn2)
    check("committed pad-to-pad copper stays IMMOVABLE", movable2 is False)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
