#!/usr/bin/env python3
"""
Regression test for issue #221 (route_diff via leak in the single-ended-blocker
layer swap).

`apply_diff_pair_layer_swaps` Phase 7 tries to clear a diff pair's via-transition
layer by *moving a blocking single-ended net's stub to another layer*. The move
is applied speculatively (the blocker stub is re-layered and, if it was on F.Cu,
a pad via is created) so the subsequent diff-pair `validate_swap` can be tested
against the cleared layer. When that validation FAILS the speculative move is
reverted with `revert_stub_layer_switch`.

The bug: the caller recorded the speculative via/segment mods in the output
aggregates (`all_swap_vias`, `all_segment_modifications`) BEFORE the validation,
and `revert_stub_layer_switch` only undoes `pcb_data` -- it has no handle on the
aggregates. So every reverted attempt leaked its pad via into the written board.
A single-ended net that blocks several pairs (or is probed across several
candidate layers) accumulated 5-10 coincident pad vias, which is the bulk of the
board's VIA-DRILL-HOLE (stacked drills), plus via-body VIA-SEGMENT / PAD-VIA
grazes from those phantom vias. On butterstick this was 173 VIA-DRILL-HOLE.

The fix defers recording into the aggregates until AFTER the diff-pair swap is
confirmed valid, so a reverted attempt leaves nothing behind.

This test forces the Phase-7 revert path deterministically (blocker present,
single-ended swap valid, diff-pair validation always failing) and asserts the
output aggregates stay empty -- with the bug they would contain leaked vias.

Run:
    python3 tests/test_layer_swap_via_leak.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Segment, Net
from routing_config import GridRouteConfig, DiffPairNet
from stub_layer_switching import StubInfo
import layer_swap_optimization as lso


LAYERS = ['F.Cu', 'In1.Cu', 'B.Cu']
P_NET, N_NET, BLOCKER_NET = 1, 2, 3


def _cfg():
    c = GridRouteConfig()
    c.layers = LAYERS
    c.via_size = 0.45
    c.via_drill = 0.20
    c.clearance = 0.1
    c.track_width = 0.1
    c.bga_exclusion_zones = []
    return c


def _seg(net_id, layer, x0, y0, x1, y1):
    return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1,
                   width=0.1, layer=layer, net_id=net_id)


def main():
    cfg = _cfg()

    # The blocker single-ended net lives on F.Cu (the diff pair's target layer),
    # so moving it off F.Cu mints a pad via -- the exact thing that leaked.
    blocker_seg = _seg(BLOCKER_NET, 'F.Cu', 5.0, 5.0, 5.0, 6.0)
    pcb = PCBData(
        footprints={}, segments=[blocker_seg], vias=[],
        nets={P_NET: Net(P_NET, '/P'), N_NET: Net(N_NET, '/N'),
              BLOCKER_NET: Net(BLOCKER_NET, '/BLOCKER')},
        board_info=None, pads_by_net={},
    )

    pair = DiffPairNet(base_name='PAIR', p_net_id=P_NET, n_net_id=N_NET,
                       p_net_name='/P', n_net_name='/N')

    # source on In1.Cu (idx 1), target on F.Cu (idx 0) -> pair "needs a via",
    # which is what drives it into the Phase-7 blocker-clearing logic.
    sources = [(0.0, 0.0, 0.0, 0.2, 1, 0.0, 0.0, 0.0, 0.2)]
    targets = [(1.0, 0.0, 1.0, 0.2, 0, 1.0, 0.0, 1.0, 0.2)]

    def fake_endpoints(pcb_data, p_net_id, n_net_id, config):
        return sources, targets, None

    def fake_get_stub_info(pcb_data, net_id, x, y, layer, *a, **k):
        # Pair stubs: present so the source/target legs are considered.
        return StubInfo(net_id=net_id, x=x, y=y, layer=layer, segments=[],
                        pad_x=x, pad_y=y, has_pad_via=False)

    # One blocker on F.Cu, owning a real segment so the speculative switch mints
    # a pad via and records a segment mod (both must be rolled back on revert).
    blocker_stub = StubInfo(net_id=BLOCKER_NET, x=5.0, y=6.0, layer='F.Cu',
                            segments=[blocker_seg], pad_x=5.0, pad_y=5.0,
                            has_pad_via=False)

    def fake_blocking_nets(p_stub, n_stub, layer, pcb_data, diff_ids):
        return [BLOCKER_NET]

    def fake_se_stub(pcb_data, net_id, layer, config):
        return blocker_stub

    lso.get_diff_pair_endpoints = fake_endpoints
    lso.get_stub_info = fake_get_stub_info
    lso.collect_stubs_by_layer = lambda *a, **k: {}
    lso.collect_stub_endpoints_by_layer = lambda *a, **k: {}
    lso._find_blocking_single_ended_nets = fake_blocking_nets
    lso._get_single_ended_stub_on_layer = fake_se_stub
    lso._validate_single_ended_swap = lambda *a, **k: True
    # Diff-pair validation always fails -> every speculative blocker move is
    # reverted. This is the exact path that leaked.
    lso.validate_swap = lambda *a, **k: (False, 'forced-fail')

    all_segment_modifications = []
    all_swap_vias = []

    lso.apply_diff_pair_layer_swaps(
        pcb, cfg,
        diff_pair_ids_to_route_set=[('PAIR', pair)],
        diff_pairs={'PAIR': pair},
        can_swap_to_top_layer=True,
        all_segment_modifications=all_segment_modifications,
        all_swap_vias=all_swap_vias,
    )

    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    check("reverted blocker swap leaks no via into all_swap_vias",
          all_swap_vias == [],
          f"all_swap_vias={[(v.x, v.y, v.net_id) for v in all_swap_vias]}")
    check("reverted blocker swap leaks no segment mod",
          all_segment_modifications == [],
          f"n_mods={len(all_segment_modifications)}")
    check("pcb_data.vias clean after revert (sanity)",
          pcb.vias == [],
          f"pcb.vias={len(pcb.vias)}")
    check("blocker segment restored to F.Cu after revert (sanity)",
          blocker_seg.layer == 'F.Cu',
          f"layer={blocker_seg.layer}")

    print("\n" + "=" * 60)
    passed = sum(1 for _, ok in results if ok)
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"\n{passed}/{len(results)} checks passed")
    print("=" * 60)
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
