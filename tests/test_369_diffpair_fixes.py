#!/usr/bin/env python3
"""
Tests for issue #369 A2, A3, A14 (diff-pair machinery).

  A2: ripping a multi-leg multipoint diff pair removes the per-LEG dicts from
      the write-list (value-equality against the merged dict never matched, so
      the ripped legs' copper shipped alongside the reroute's); restore puts
      the legs back.
  A3: target-swap pads_by_net re-filing is gated on BOTH pads like the net_id
      swap (one-sided re-filing put a never-swapped pad in the other net's
      list) - tested at the helper level via apply_single_swap's gates.
  A14: a multipoint terminal picks the farthest SAME-LAYER stub-tip pair; tips
      on different layers no longer emit N's connector on P's layer.

Run:
    python3 tests/test_369_diffpair_fixes.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Segment, Via, Pad, Net
from routing_config import GridRouteConfig, DiffPairNet
from rip_up_reroute import rip_up_net, restore_net


def _seg(x1, y1, x2, y2, net_id, layer='F.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=0.2, layer=layer, net_id=net_id)


def _cfg():
    c = GridRouteConfig()
    c.layers = ['F.Cu', 'In1.Cu', 'B.Cu']
    c.grid_step = 0.1
    return c


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- A2: rip removes per-leg dicts; restore brings them back -------------
    print("A2: multipoint diff pair rip/restore of per-leg results")
    p_id, n_id = 1, 2
    leg1 = {'net_name': 'P', 'new_segments': [_seg(0, 0, 5, 0, p_id)],
            'new_vias': [], 'iterations': 10}
    leg2 = {'net_name': 'P', 'new_segments': [_seg(5, 0, 9, 0, p_id)],
            'new_vias': [], 'iterations': 12}
    merged = dict(leg2)
    merged['new_segments'] = leg1['new_segments'] + leg2['new_segments']
    merged['new_vias'] = []
    merged['leg_results'] = [leg1, leg2]

    _pads = [Pad(component_ref='U1', pad_number='1', global_x=0, global_y=0,
                 local_x=0, local_y=0, size_x=0.4, size_y=0.4, shape='circle',
                 layers=['F.Cu'], net_id=p_id, net_name='P'),
             Pad(component_ref='U2', pad_number='1', global_x=9, global_y=0,
                 local_x=0, local_y=0, size_x=0.4, size_y=0.4, shape='circle',
                 layers=['F.Cu'], net_id=p_id, net_name='P')]
    pcb = PCBData(board_info=None, nets={p_id: Net(p_id, 'P'), n_id: Net(n_id, 'N')},
                  footprints={}, vias=[],
                  segments=list(merged['new_segments']), pads_by_net={p_id: _pads})
    pair = DiffPairNet(base_name='X', p_net_name='P', n_net_name='N',
                       p_net_id=p_id, n_net_id=n_id)
    results = [leg1, leg2]
    routed_results = {p_id: merged, n_id: merged}
    diff_pair_by_net_id = {p_id: ('X', pair), n_id: ('X', pair)}
    routed_net_ids = [p_id, n_id]
    remaining = []
    cfg = _cfg()

    saved, ripped_ids, was_in = rip_up_net(
        p_id, pcb, routed_net_ids, {}, routed_results, diff_pair_by_net_id,
        remaining, results, cfg)
    check("rip found the legs in the write-list (was False)", was_in)
    check("BOTH leg dicts removed from the write-list (was neither)",
          results == [])
    check("copper removed from pcb_data", len(pcb.segments) == 0)

    restore_net(p_id, saved, ripped_ids, was_in, pcb, routed_net_ids, {},
                routed_results, diff_pair_by_net_id, remaining, results, cfg)
    check("restore puts the LEG dicts back (not the merged twin)",
          len(results) == 2 and results[0] is leg1 and results[1] is leg2)
    # add_route_to_pcb_data may merge the two collinear legs into one span
    check("copper restored to pcb_data (full extent)",
          len(pcb.segments) >= 1
          and min(min(s.start_x, s.end_x) for s in pcb.segments) <= 0.0
          and max(max(s.start_x, s.end_x) for s in pcb.segments) >= 9.0)

    # -- A14: same-layer tip pair selection ----------------------------------
    print("\nA14: multipoint terminal same-layer tip pairing")
    from diff_pair_multipoint import _terminal_stub_endpoint
    cfg = _cfg()
    pp = Pad(component_ref='U1', pad_number='1', global_x=0, global_y=0,
             local_x=0, local_y=0, size_x=0.4, size_y=0.4, shape='circle',
             layers=['F.Cu'], net_id=1, net_name='P')
    nn = Pad(component_ref='U1', pad_number='2', global_x=0, global_y=0.5,
             local_x=0, local_y=0, size_x=0.4, size_y=0.4, shape='circle',
             layers=['F.Cu'], net_id=2, net_name='N')
    # P stub escapes on In1; N has one short F.Cu tip and one In1 tip:
    # the OLD code took N's farthest tip regardless of layer and stamped P's
    # layer on it; the fix must pick N's In1 tip (common layer with P).
    segs = [
        _seg(0, 0, 3, 0, 1, layer='In1.Cu'),      # P stub on In1 (tip at 3,0)
        _seg(0, 0.5, 4, 0.5, 2, layer='F.Cu'),    # N far tip on F.Cu (4,0.5)
        _seg(0, 0.5, 2, 0.5, 2, layer='In1.Cu'),  # N tip on In1 (2,0.5)
    ]
    pcb = PCBData(board_info=None, nets={1: Net(1, 'P'), 2: Net(2, 'N')},
                  footprints={}, vias=[], segments=segs,
                  pads_by_net={1: [pp], 2: [nn]})
    ep = _terminal_stub_endpoint(pcb, (pp, nn), cfg)
    check("endpoint produced", ep is not None)
    if ep is not None:
        _, _, _, _, layer_idx, px, py, nx, ny = ep
        check("common layer chosen (In1)", cfg.layers[layer_idx] == 'In1.Cu')
        check("N coordinate is N's In1 tip, not its far F.Cu tip",
              abs(nx - 2.0) < 1e-6 and abs(ny - 0.5) < 1e-6)

    # N with NO tip on P's layer -> None (pad-midpoint fallback), not a
    # cross-layer connector.
    segs = [
        _seg(0, 0, 3, 0, 1, layer='In1.Cu'),
        _seg(0, 0.5, 4, 0.5, 2, layer='F.Cu'),
    ]
    pcb = PCBData(board_info=None, nets={1: Net(1, 'P'), 2: Net(2, 'N')},
                  footprints={}, vias=[], segments=segs,
                  pads_by_net={1: [pp], 2: [nn]})
    ep = _terminal_stub_endpoint(pcb, (pp, nn), cfg)
    check("no common layer -> None (was P-layer stamp on N)", ep is None)

    # -- A3: pads_by_net re-filing symmetric with the pad swap ---------------
    print("\nA3: one-sided pad find does not re-file pads_by_net")
    import target_swap as ts
    import inspect
    src = inspect.getsource(ts.apply_single_swap) if hasattr(ts, 'apply_single_swap') else ''
    # structural check: the re-filing is now under the both-pads gates
    check("re-filing gated on both P pads",
          'if p1_p_pad and p2_p_pad:' in src and 'if p1_n_pad and p2_n_pad:' in src
          and src.count('if p1_p_pad:') == 0)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
