#!/usr/bin/env python3
"""Issue #71: --rip-existing-nets must force a CONNECTED net back into the
route set.

filter_already_routed graded any fully-connected net "Already fully connected"
and skipped it BEFORE the rip patterns were consulted, so a connected-but-DRC-
violating net (e.g. routed at 0.25mm from an HV corridor that requires 0.35)
was unrippable: the only way to re-route it was hand-deleting its copper from
the board text. force_route_patterns short-circuits the connectivity grade for
rip-matched nets; route.py then pre-strips their copper (pre-route rip) so the
router replans from scratch and the writer's removal lists drop the originals.

    python3 tests/test_rip_forces_reroute.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from routing_common import filter_already_routed


def _pad(x, y, net, ref="R1"):
    return SimpleNamespace(global_x=x, global_y=y, net_id=net,
                           pad_type="smd", size_x=0.6, size_y=0.6,
                           drill=0.0, layers=["F.Cu"],
                           component_ref=ref, pad_number="1",
                           shape="rect", rect_rotation=0.0,
                           roundrect_rratio=0.0, polygons=None)


def _seg(x1, y1, x2, y2, net, layer="F.Cu"):
    return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                           width=0.15, layer=layer, net_id=net)


def _board():
    # net 7 "VIOLATOR": two pads joined by one segment => fully connected
    pads = [_pad(0.0, 0.0, 7), _pad(5.0, 0.0, 7)]
    segs = [_seg(0.0, 0.0, 5.0, 0.0, 7)]
    net = SimpleNamespace(name="VIOLATOR", net_id=7, pads=pads)
    return SimpleNamespace(
        nets={7: net}, pads_by_net={7: pads}, segments=segs, vias=[],
        zones=[], footprints=[], board_outlines=[])


def main():
    pcb = _board()
    net_ids = [("VIOLATOR", 7)]
    cfg = SimpleNamespace(layers=["F.Cu", "B.Cu"])

    to_route, skipped = filter_already_routed(pcb, net_ids, cfg)
    assert to_route == [], "baseline: connected net must be skipped"
    assert skipped and skipped[0][0] == "VIOLATOR"

    to_route, skipped = filter_already_routed(
        pcb, net_ids, cfg, force_route_patterns=["VIOLATOR"])
    assert to_route == [("VIOLATOR", 7)], \
        "rip-matched connected net must re-enter the route set (#71)"

    to_route, _ = filter_already_routed(
        pcb, net_ids, cfg, force_route_patterns=["OTHER_*"])
    assert to_route == [], "non-matching pattern must not force-route"

    print("OK: #71 rip forces connected net back into the route set")


if __name__ == "__main__":
    main()
