#!/usr/bin/env python3
"""Issue #72: Case-1 copper groups must expose their VIAS as route terminals.

get_net_endpoints Case 1 (two+ copper groups) emitted only segment endpoints,
each bound to its segment's layer. A group whose segments all sit on layers
OUTSIDE config.layers -- the canonical shape: an F.Cu pad stub ending in an
escape via, routed with --layers In2/In3 to duck under an HV corridor --
contributed no endpoint at all and the net was unroutable, even though its
via is reachable on every inner layer. Case 2 has added vias for years.

    python3 tests/test_group_vias_as_terminals.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rust_router"))

from connectivity import get_net_endpoints


def _seg(x1, y1, x2, y2, layer="F.Cu", net=5):
    return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                           width=0.1, layer=layer, net_id=net)


def _via(x, y, net=5):
    return SimpleNamespace(x=x, y=y, size=0.25, drill=0.15, net_id=net,
                           layers=["F.Cu", "B.Cu"])


def _pad(x, y, net=5, ref="J1"):
    return SimpleNamespace(global_x=x, global_y=y, net_id=net,
                           pad_type="smd", size_x=0.2, size_y=0.7,
                           drill=0.0, layers=["F.Cu"], component_ref=ref,
                           pad_number="1", shape="rect", rect_rotation=0.0,
                           roundrect_rratio=0.0, polygons=None)


def main():
    # Two groups, both entirely on F.Cu, each ending in a via:
    #   group A: pad(10,10) + stub to via(12,10)
    #   group B: pad(40,40) + stub to via(38,40)
    pads = [_pad(10, 10, ref="J1"), _pad(40, 40, ref="R2")]
    segs = [_seg(10, 10, 12, 10), _seg(40, 40, 38, 40)]
    vias = [_via(12, 10), _via(38, 40)]
    net = SimpleNamespace(name="THERM", net_id=5, pads=pads)
    pcb = SimpleNamespace(nets={5: net}, pads_by_net={5: pads},
                          segments=segs, vias=vias, zones=[], footprints=[],
                          board_outlines=[], board_info=None)

    # Inner layers only: F.Cu segment endpoints all drop out; only the vias
    # can carry terminals.
    cfg = SimpleNamespace(layers=["In2.Cu", "In3.Cu"], grid_step=0.05)
    sources, targets, err = get_net_endpoints(pcb, 5, cfg)
    assert err is None, f"unexpected error: {err}"
    assert sources and targets, \
        "via-anchored groups must produce terminals on inner layers (#72)"
    src_xy = {(round(s[3], 2), round(s[4], 2)) for s in sources}
    tgt_xy = {(round(t[3], 2), round(t[4], 2)) for t in targets}
    assert src_xy in ({(12.0, 10.0)}, {(38.0, 40.0)}), src_xy
    assert tgt_xy in ({(12.0, 10.0)}, {(38.0, 40.0)}) and tgt_xy != src_xy
    layer_idxs = {s[2] for s in sources} | {t[2] for t in targets}
    assert layer_idxs == {0, 1}, \
        f"via terminals must exist on every routing layer, got {layer_idxs}"

    print("OK: #72 Case-1 group vias serve as terminals on all routing layers")


if __name__ == "__main__":
    main()
