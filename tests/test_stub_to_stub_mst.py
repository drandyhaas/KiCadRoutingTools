#!/usr/bin/env python3
"""The multipoint MST must be able to ask stub-to-stub, not only pad-to-pad.

The defect this guards (test-board run 5, journal [9]): each copper island
contributed exactly ONE terminal (a via, else free_ends[0]), so an
island-to-island gap of 0.6mm between two NON-representative stub ends was
structurally invisible -- net_forensics showed the gap with EMPTY walls, the
router's MST realized the edge at a walled far pad instead, and the join had to
be hand-authored into the board file. Islands now emit every free end (capped,
position-sorted); compute_component_mst_edges is component-keyed (#317), so the
extra terminals never create intra-island edges -- they only let the inter-island
edge land on the true closest approach.

Also pinned: multipoint CLASSIFICATION counts islands+copperless pads (units),
not terminals -- a 2-island net with two dangling ends per island must NOT
become "multipoint" just because it now has 4 terminals.
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
# #522 moved the engine modules out of the repo root into py_router/ (and the
# placement family into py_placer/, the leaf tools into py_tools/).
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
          os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import make_pad, make_seg, make_net, make_pcb  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
import connectivity  # noqa: E402


def _pcb_three_units():
    """Island A: pad + elbow stub whose free end reaches toward island B's
    stub; island B: pad + stub; plus one bare pad (third unit). The two stub
    free ends are 0.6mm apart; the pads are ~10mm apart."""
    net = 7
    pads = [
        make_pad(net_id=net, x=0.0, y=0.0, num='1', net_name='N'),
        make_pad(net_id=net, x=10.0, y=0.0, ref='U2', num='1', net_name='N'),
        make_pad(net_id=net, x=5.0, y=8.0, ref='U3', num='1', net_name='N'),
    ]
    segs = [
        # Island A: from pad1 east then an elbow north -- free end at (4.0, 2.0)
        make_seg(0.0, 0.0, 4.0, 0.0, net_id=net),
        make_seg(4.0, 0.0, 4.0, 2.0, net_id=net),
        # Island B: from pad2 west then an elbow north -- free end at (4.6, 2.0)
        make_seg(10.0, 0.0, 4.6, 0.0, net_id=net),
        make_seg(4.6, 0.0, 4.6, 2.0, net_id=net),
    ]
    return make_pcb(nets={net: make_net(net, 'N')}, segments=segs,
                    pads_by_net={net: pads}), net


def test_mst_edge_lands_on_the_stub_gap():
    pcb, net = _pcb_three_units()
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    info = connectivity.get_multipoint_net_pads(pcb, net, cfg)
    assert info, "net not classified multipoint (3 units exist)"
    positions = [(t[3], t[4]) for t in info]
    comps, _, _, _ = connectivity.get_terminal_component_info(pcb, net, info)
    edges = connectivity.compute_component_mst_edges(positions, comps)
    # The A<->B edge must be realized at the 0.6mm stub gap, not the 10mm pads.
    best = min(edges, key=lambda e: e[2])
    (ax, ay), (bx, by) = positions[best[0]], positions[best[1]]
    got = {(ax, ay), (bx, by)}
    assert got == {(4.0, 2.0), (4.6, 2.0)}, \
        f"closest inter-island edge realized at {got}, not the stub gap"
    assert abs(best[2] - 0.6) < 1e-6, f"gap distance {best[2]}, wanted 0.6"
    print("  PASS: inter-island MST edge lands on the 0.6mm stub-to-stub gap")


def test_two_island_net_stays_non_multipoint():
    net = 7
    pads = [
        make_pad(net_id=net, x=0.0, y=0.0, num='1', net_name='N'),
        make_pad(net_id=net, x=10.0, y=0.0, ref='U2', num='1', net_name='N'),
    ]
    segs = [
        make_seg(0.0, 0.0, 3.0, 0.0, net_id=net),
        make_seg(3.0, 0.0, 3.0, 1.5, net_id=net),   # island A: 2 free-end arms
        make_seg(10.0, 0.0, 7.0, 0.0, net_id=net),
        make_seg(7.0, 0.0, 7.0, 1.5, net_id=net),   # island B: 2 free-end arms
    ]
    pcb = make_pcb(nets={net: make_net(net, 'N')}, segments=segs,
                   pads_by_net={net: pads})
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    info = connectivity.get_multipoint_net_pads(pcb, net, cfg)
    assert info is None, \
        f"2-island net reclassified as multipoint by terminal count: {info}"
    print("  PASS: 2-island net stays a simple net (units, not terminals)")


if __name__ == '__main__':
    test_mst_edge_lands_on_the_stub_gap()
    test_two_island_net_stays_non_multipoint()
