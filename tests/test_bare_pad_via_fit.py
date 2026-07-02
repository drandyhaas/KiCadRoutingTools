#!/usr/bin/env python3
"""
Regression test for issue #241 (route_diff bare-pad target swap drops diff-pair
pad vias below clearance).

`apply_diff_pair_layer_swaps` fans a bare F.Cu connector pad onto an inner layer
by dropping a through-via on EACH of the pair's P and N pads (the bare-pad target
swap, `apply_bare_pad_target_via`). At a tight connector pad pitch (e.g. SYZYGY /
DDR headers at 0.5mm) two 0.45mm via bodies inherently overlap (VIA-VIA) and graze
the neighbouring cap / partner pads (PAD-VIA). The existing
`via_barrel_clear_of_foreign_copper` guard only rejects a via PUNCHING THROUGH
foreign copper (a short), not a sub-clearance graze - so on butterstick the swap
shipped 2 VIA-VIA between the /SYZYGY2.TX2 and /SYZYGY2.TX3 partner pad vias.

The fix adds `_bare_pad_pair_vias_fit`, mirroring the multipoint path's already-
present `_fans_fit` guard: it re-validates the two new vias at check_drc clearance
(body AND drill hole-to-hole) against each other, the existing foreign vias, and
the foreign + PARTNER pads. If they don't fit the swap is undone and the pair
routes to the bare pads instead (connectivity preserved - verified on butterstick:
26/40 pairs routed both with and without the guard, VIA-VIA 2 -> 0).

This test exercises the helper directly:
  - two vias at a 0.5mm pad pitch -> collide -> fit=False (via-via)
  - the same two vias spaced 0.8mm apart -> fit=True
  - a single pad via grazing a foreign cap pad -> fit=False (pad-via)
  - a single pad via grazing the PARTNER pad -> fit=False (pad-via)

Run:
    python3 tests/test_bare_pad_via_fit.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Via, Pad, Net
from routing_config import GridRouteConfig
import layer_swap_optimization as lso

P_NET, N_NET, FOREIGN_NET = 1, 2, 99


def _cfg():
    c = GridRouteConfig()
    c.layers = ['F.Cu', 'In1.Cu', 'B.Cu']
    c.via_size = 0.45
    c.via_drill = 0.20
    c.clearance = 0.1
    c.hole_to_hole_clearance = 0.20
    return c


def _via(x, y, net_id, cfg):
    return Via(x=x, y=y, size=cfg.via_size, drill=cfg.via_drill,
               layers=['F.Cu', 'B.Cu'], net_id=net_id)


def _pad(ref, num, x, y, net_id, net_name):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5, shape='rect',
               layers=['F.Cu'], net_id=net_id, net_name=net_name)


def _pcb(vias=None, pads_by_net=None):
    return PCBData(
        footprints={}, segments=[], vias=list(vias or []),
        nets={P_NET: Net(P_NET, '/PAIR_P'), N_NET: Net(N_NET, '/PAIR_N'),
              FOREIGN_NET: Net(FOREIGN_NET, '+2V5')},
        board_info=None, pads_by_net=pads_by_net or {},
    )


def main():
    cfg = _cfg()
    checks = []

    def check(name, cond):
        checks.append((name, cond))
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    # 1) two pad vias at a 0.5mm diff-pair pad pitch collide (via-via): the
    #    TX2/TX3 case. 0.45 body + 0.45 body + 0.1 clearance needs 0.55mm centres.
    via_p = _via(0.0, 0.0, P_NET, cfg)
    via_n = _via(0.5, 0.0, N_NET, cfg)
    pcb = _pcb(vias=[via_p, via_n])
    fit, why = lso._bare_pad_pair_vias_fit(pcb, [via_p, via_n], cfg)
    check("0.5mm-pitch pad vias rejected (via-via)", (not fit) and 'via-via' in why)

    # 2) the same two vias spaced 0.8mm apart fit cleanly.
    via_p2 = _via(0.0, 0.0, P_NET, cfg)
    via_n2 = _via(0.8, 0.0, N_NET, cfg)
    pcb2 = _pcb(vias=[via_p2, via_n2])
    fit2, _ = lso._bare_pad_pair_vias_fit(pcb2, [via_p2, via_n2], cfg)
    check("0.8mm-pitch pad vias accepted", fit2)

    # 3) a single pad via grazing a FOREIGN cap pad is rejected (pad-via). The
    #    via sits at its own pad; a +2V5 cap pad edge sits ~0.1mm from the body.
    via_a = _via(0.0, 0.0, P_NET, cfg)
    foreign_pad = _pad('C38', '1', 0.55, 0.0, FOREIGN_NET, '+2V5')
    pcb3 = _pcb(vias=[via_a], pads_by_net={FOREIGN_NET: [foreign_pad]})
    fit3, why3 = lso._bare_pad_pair_vias_fit(pcb3, [via_a], cfg)
    check("pad via grazing a foreign pad rejected (pad-via)",
          (not fit3) and 'pad-via' in why3)

    # 4) a single pad via grazing the PARTNER pad is rejected (the C2P_CLK case).
    #    Own-net pads are excluded, but the partner net's pad IS checked.
    via_b = _via(0.0, 0.0, P_NET, cfg)
    partner_pad = _pad('J4', '36', 0.55, 0.0, N_NET, '/PAIR_N')
    pcb4 = _pcb(vias=[via_b], pads_by_net={N_NET: [partner_pad]})
    fit4, why4 = lso._bare_pad_pair_vias_fit(pcb4, [via_b], cfg)
    check("pad via grazing the partner pad rejected (pad-via)",
          (not fit4) and 'pad-via' in why4)

    # 5) the via legitimately sitting on its OWN-net pad does not self-reject.
    via_c = _via(0.0, 0.0, P_NET, cfg)
    own_pad = _pad('U4', 'N4', 0.0, 0.0, P_NET, '/PAIR_P')
    pcb5 = _pcb(vias=[via_c], pads_by_net={P_NET: [own_pad]})
    fit5, _ = lso._bare_pad_pair_vias_fit(pcb5, [via_c], cfg)
    check("via on its own-net pad accepted (no self-reject)", fit5)

    # 6) (#282) a new pad via whose drill overlaps the pair's OWN existing
    #    fanout via (same net!) is rejected: hole-to-hole is net-independent.
    #    butterstick CK1: new via 0.125mm from the ball via-in-pad, 0.2 drills.
    via_d = _via(0.0, 0.0, P_NET, cfg)
    own_fanout_via = Via(x=0.125, y=0.0, size=0.4, drill=0.2,
                         layers=['F.Cu', 'B.Cu'], net_id=P_NET)
    pcb6 = _pcb(vias=[via_d, own_fanout_via])
    fit6, why6 = lso._bare_pad_pair_vias_fit(pcb6, [via_d], cfg)
    check("same-net drill overlap rejected (hole-to-hole, #282)",
          (not fit6) and 'hole-to-hole' in why6)

    # 7) (#282) apply_bare_pad_target_via REUSES an existing same-net through
    #    via whose barrel covers the pad centre instead of drilling a new hole:
    #    returns via=None and anchors the stub at the existing via.
    from stub_layer_switching import apply_bare_pad_target_via
    ball_via = Via(x=0.125, y=0.0, size=0.4, drill=0.2,
                   layers=['F.Cu', 'B.Cu'], net_id=P_NET)
    pcb7 = _pcb(vias=[ball_via])
    n_vias_before = len(pcb7.vias)
    via7, stub7 = apply_bare_pad_target_via(
        pcb7, P_NET, 0.0, 0.0, 'In1.Cu', 5.0, 0.0, cfg)
    check("overlapping same-net via reused (no new hole, #282)",
          via7 is None and len(pcb7.vias) == n_vias_before
          and (stub7.start_x, stub7.start_y) == (ball_via.x, ball_via.y))

    # 8) with no reusable via nearby, a new via IS created at the pad.
    pcb8 = _pcb(vias=[Via(x=0.9, y=0.0, size=0.4, drill=0.2,
                          layers=['F.Cu', 'B.Cu'], net_id=P_NET)])
    via8, stub8 = apply_bare_pad_target_via(
        pcb8, P_NET, 0.0, 0.0, 'In1.Cu', 5.0, 0.0, cfg)
    check("no reusable via -> new via at the pad",
          via8 is not None and (via8.x, via8.y) == (0.0, 0.0))

    passed = sum(1 for _, c in checks if c)
    print("=" * 60)
    print(f"{passed}/{len(checks)} checks passed")
    print("=" * 60)
    return 0 if passed == len(checks) else 1


if __name__ == '__main__':
    sys.exit(main())
