#!/usr/bin/env python3
"""
Regression test for issue #165: route_diff connector/setback segments grazing a
foreign/partner pad.

The pair's connector segments are clearance-checked during routing against an
obstacle map that excludes BOTH halves of the pair, so a connector can graze the
PARTNER net's pad (e.g. a USB-C P connector cutting across the adjacent N pad)
completely unseen. `_connector_grazes_foreign_copper` re-validates the final
geometry against every foreign pad using the SAME geometry + tolerance as
check_drc, so it rejects a real graze WITHOUT false-rejecting tight-but-legal
packing that sits exactly at clearance (the over-rejection that sank the first
attempt).

Run:
    python3 tests/test_diff_connector_graze.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import Pad, Segment, Via, PCBData
from routing_config import GridRouteConfig
from diff_pair_routing import _connector_grazes_foreign_copper

P_NET, N_NET = 10, 20
CLEARANCE = 0.2
WIDTH = 0.2
PAD_HALF = 0.15  # 0.3mm square pad
# Track edge must clear pad edge by (width/2 + clearance); centerline distance
# from pad center for a just-legal track = PAD_HALF + WIDTH/2 + CLEARANCE.
LEGAL_CENTERLINE = PAD_HALF + WIDTH / 2 + CLEARANCE  # 0.45mm
TOLERANCE = CLEARANCE * 0.05  # check_drc default --clearance-margin = 0.01mm


def _pad(net_id, net_name, x, y):
    return Pad('J1', 'A7', x, y, 0, 0, 2 * PAD_HALF, 2 * PAD_HALF, 'rect',
               ['F.Cu'], net_id, net_name, 0, None, 0.0, None, 0.25, 0.0, None)


def _seg(net_id, y):
    return Segment(start_x=-1.0, start_y=y, end_x=1.0, end_y=y,
                   width=WIDTH, layer='F.Cu', net_id=net_id)


def _cfg():
    cfg = GridRouteConfig()
    cfg.clearance = CLEARANCE
    cfg.layers = ['F.Cu', 'B.Cu']
    return cfg


def _pcb(pads_by_net):
    return PCBData(footprints={}, nets={}, segments=[], vias=[],
                   board_info=None, pads_by_net=pads_by_net)


def main():
    cfg = _cfg()
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    # 1. Real graze against the PARTNER net's pad must be rejected.
    pcb = _pcb({N_NET: [_pad(N_NET, '/USB_DN', 0.0, 0.0)]})
    y = LEGAL_CENTERLINE - 0.15  # 0.15mm into the required clearance band
    r = _connector_grazes_foreign_copper([_seg(P_NET, y)], pcb, P_NET, N_NET, cfg)
    check("real partner-pad graze rejected", r is not None,
          f"overlap={r[3]:.3f}mm" if r else "NOT detected")

    # 2. A just-legal track sitting exactly at clearance must NOT be rejected.
    r = _connector_grazes_foreign_copper([_seg(P_NET, LEGAL_CENTERLINE)], pcb, P_NET, N_NET, cfg)
    check("track exactly at clearance passes", r is None,
          "false reject" if r else "")

    # 3. The connector landing on its OWN-net pad must be ignored.
    pcb_own = _pcb({P_NET: [_pad(P_NET, '/USB_DP', 0.0, 0.0)]})
    r = _connector_grazes_foreign_copper([_seg(P_NET, 0.0)], pcb_own, P_NET, N_NET, cfg)
    check("own-net pad ignored (connector lands on it)", r is None,
          "wrongly rejected own pad" if r else "")

    # 4. Tolerance boundary matches check_drc: overlap <= tol passes, > tol rejects.
    below = LEGAL_CENTERLINE - (TOLERANCE - 0.001)  # overlap just under tolerance
    above = LEGAL_CENTERLINE - (TOLERANCE + 0.001)  # overlap just over tolerance
    r_below = _connector_grazes_foreign_copper([_seg(P_NET, below)], pcb, P_NET, N_NET, cfg)
    r_above = _connector_grazes_foreign_copper([_seg(P_NET, above)], pcb, P_NET, N_NET, cfg)
    check("overlap just under DRC tolerance passes", r_below is None)
    check("overlap just over DRC tolerance rejects", r_above is not None)

    # 5. N segment grazing the partner P pad (symmetric case).
    pcb_p = _pcb({P_NET: [_pad(P_NET, '/USB_DP', 0.0, 0.0)]})
    r = _connector_grazes_foreign_copper([_seg(N_NET, LEGAL_CENTERLINE - 0.15)],
                                      pcb_p, P_NET, N_NET, cfg)
    check("N segment grazing partner P pad rejected", r is not None,
          f"overlap={r[3]:.3f}mm" if r else "NOT detected")

    # 6. P segment grazing the PARTNER net's via (e.g. its underpad escape via).
    def _via(net_id, x, y):
        return Via(x=x, y=y, size=0.45, drill=0.25, layers=['F.Cu', 'B.Cu'], net_id=net_id)
    via_clear = 0.45 / 2 + WIDTH / 2 + CLEARANCE  # 0.525mm center-to-center
    pcb_v = PCBData(footprints={}, nets={}, segments=[], vias=[_via(N_NET, 0.0, 0.0)],
                    board_info=None, pads_by_net={})
    r = _connector_grazes_foreign_copper([_seg(P_NET, via_clear - 0.12)], pcb_v, P_NET, N_NET, cfg)
    check("P segment grazing partner via rejected", r is not None and r[0] == 'via',
          f"kind={r[0]} overlap={r[3]:.3f}mm" if r else "NOT detected")

    # 7. P segment near its OWN-net via must be ignored (it connects to it).
    pcb_vo = PCBData(footprints={}, nets={}, segments=[], vias=[_via(P_NET, 0.0, 0.0)],
                     board_info=None, pads_by_net={})
    r = _connector_grazes_foreign_copper([_seg(P_NET, via_clear - 0.12)], pcb_vo, P_NET, N_NET, cfg)
    check("own-net via ignored", r is None, "wrongly rejected own via" if r else "")

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
