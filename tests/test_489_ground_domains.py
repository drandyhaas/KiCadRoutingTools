#!/usr/bin/env python3
"""#489 §5: all ground nets collapsed to one, so return vias crossed domains.

`resolve_gnd_net_id` resolves ONE board-global ground. On a split-ground board it
picks one arbitrarily, and both return-via features stitch EVERY signal via --
analog included -- to that net. That is worse than doing nothing: it can bridge
the very domains the split exists to separate, while connectivity and DRC both
report green.

Covers:
  * a single-ground board resolves EXACTLY as before, for every net (the
    no-regression property that matters for the corpus)
  * a split-ground board keeps AGND/DGND apart, and each signal's return domain
    comes from its endpoint components
  * the single-point bridge (ferrite / 0R link / net tie) is detected and excluded
    from domain seeding, so membership does not leak across the split
  * a DNP link is NOT a bridge (a no-pop part is an open)
  * an explicit net name still pins every via board-wide
  * add_gnd_vias places each companion via on the signal's OWN domain, and
    same-domain coverage is what counts as "already has a return"

Run with:  python3 tests/test_489_ground_domains.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "rust_router"))

from kicad_parser import BoardInfo, Footprint, Zone  # noqa: E402
from synth import make_pad, make_via, make_net, make_pcb  # noqa: E402
from net_queries import (resolve_gnd_net_id, resolve_ground_domains,  # noqa: E402
                         ground_domain_bridges, resolve_return_net_id,
                         describe_ground_domains)

GND, AGND, ASIG, DSIG = 1, 2, 3, 4


def _fp(ref, pads, dnp=False):
    return Footprint(reference=ref, footprint_name='F', x=0.0, y=0.0, rotation=0.0,
                     layer='F.Cu', pads=pads, dnp=dnp)


def _board(*, split=True, bridge='R1', bridge_dnp=False):
    """A tiny board: digital half on GND, analog half on AGND, optionally tied
    by a single 2-pad part. U1 is digital, U2 analog."""
    nets = {GND: make_net(GND, 'GND'), DSIG: make_net(DSIG, '/DATA')}
    pads_by_net = {}
    fps = {}

    d_gnd = make_pad(net_id=GND, x=0, y=0, ref='U1', num='2')
    d_sig = make_pad(net_id=DSIG, x=1, y=0, ref='U1', num='1')
    fps['U1'] = _fp('U1', [d_gnd, d_sig])
    pads_by_net[GND] = [d_gnd]
    pads_by_net[DSIG] = [d_sig, make_pad(net_id=DSIG, x=8, y=0, ref='U3', num='1')]
    fps['U3'] = _fp('U3', [pads_by_net[DSIG][1]])

    if split:
        nets[AGND] = make_net(AGND, 'AGND')
        nets[ASIG] = make_net(ASIG, '/MIC')
        a_gnd = make_pad(net_id=AGND, x=20, y=0, ref='U2', num='2')
        a_sig = make_pad(net_id=ASIG, x=21, y=0, ref='U2', num='1')
        fps['U2'] = _fp('U2', [a_gnd, a_sig])
        pads_by_net[AGND] = [a_gnd]
        pads_by_net[ASIG] = [a_sig, make_pad(net_id=ASIG, x=28, y=0, ref='U4', num='1')]
        fps['U4'] = _fp('U4', [pads_by_net[ASIG][1]])
        if bridge:
            b1 = make_pad(net_id=GND, x=10, y=0, ref=bridge, num='1')
            b2 = make_pad(net_id=AGND, x=10.5, y=0, ref=bridge, num='2')
            fps[bridge] = _fp(bridge, [b1, b2], dnp=bridge_dnp)
            pads_by_net[GND].append(b1)
            pads_by_net[AGND].append(b2)

    info = BoardInfo(layers={0: 'F.Cu', 1: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                     board_bounds=(-5, -5, 35, 15))
    return make_pcb(nets=nets, footprints=fps, pads_by_net=pads_by_net,
                    board_info=info)


def run():
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # ------------------------------------------- single ground: nothing changes
    one = _board(split=False)
    gid, gname = resolve_gnd_net_id(one)
    check((gid, gname) == (GND, 'GND'), f"single-ground resolve: got {gid}, {gname}")
    for nid in one.nets:
        check(resolve_return_net_id(one, nid)[0] == gid,
              f"single-ground board: net {nid} must resolve to the same net as "
              f"resolve_gnd_net_id -- no behaviour change on the common board")
    check(describe_ground_domains(one) is None,
          "a single-ground board must not emit a multi-domain note")
    check(len([d for d, refs in resolve_ground_domains(one).items() if refs]) == 1,
          "a single-ground board has one domain")

    # ------------------------------------------------ split ground: two domains
    b = _board()
    domains = {nid: refs for nid, refs in resolve_ground_domains(b).items() if refs}
    check(set(domains) == {GND, AGND}, f"expected GND+AGND domains, got {domains}")
    check(domains[GND] == {'U1'}, f"digital domain should be U1, got {domains[GND]}")
    check(domains[AGND] == {'U2'}, f"analog domain should be U2, got {domains[AGND]}")

    # The bridge part must be found and kept OUT of both domains -- otherwise it
    # links them and every signal looks like it belongs to one ground.
    bridges = ground_domain_bridges(b)
    check([x['ref'] for x in bridges] == ['R1'],
          f"the single-point tie should be detected, got {bridges}")
    check('R1' not in domains[GND] and 'R1' not in domains[AGND],
          "a bridge part must not be a member of either domain")

    # ------------------------------- per-signal return domain from its endpoints
    check(resolve_return_net_id(b, DSIG)[1] == 'GND',
          f"a digital net must return to GND, got {resolve_return_net_id(b, DSIG)}")
    check(resolve_return_net_id(b, ASIG)[1] == 'AGND',
          f"an analog net must return to AGND, got {resolve_return_net_id(b, ASIG)} "
          f"-- stitching it to GND is the filed bug")

    # An explicit name still pins everything, board-wide.
    check(resolve_return_net_id(b, ASIG, 'GND')[1] == 'GND',
          "an explicit --gnd-via-net must override per-domain matching")

    note = describe_ground_domains(b)
    check(note and 'AGND' in note and 'R1' in note,
          f"the multi-domain note should name the domains and the tie, got {note!r}")
    check(describe_ground_domains(b, 'GND') is None,
          "no note when the user has already disambiguated")

    # ------------------------------------------------ a DNP link is not a bridge
    nopop = _board(bridge_dnp=True)
    check(ground_domain_bridges(nopop) == [],
          "a DNP series link is an OPEN and must not count as a ground tie")
    # With no bridge part populated, its pads still touch both nets, so the part
    # is ambiguous rather than a member -- domains stay separate either way.
    nd = {nid: refs for nid, refs in resolve_ground_domains(nopop).items() if refs}
    check(set(nd) == {GND, AGND},
          f"domains must stay separate with a DNP link, got {nd}")

    # -------------------------------------------- no ground family at all -> None
    bare = make_pcb(nets={DSIG: make_net(DSIG, '/DATA')},
                    pads_by_net={DSIG: [make_pad(net_id=DSIG)]},
                    board_info=BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu']))
    check(resolve_return_net_id(bare, DSIG) == (None, None),
          "a board with no ground net must still resolve to (None, None)")

    # ------------------------------------ end-to-end: which net do vias land on?
    try:
        from routing_config import GridRouteConfig, GridCoord
        from obstacle_map import build_base_obstacle_map
        from add_gnd_vias import add_gnd_vias_to_existing_board
    except Exception as e:                                   # pragma: no cover
        print(f"  (skipped placement check: {e})")
    else:
        # Each ground domain poured on two layers: a return via must join two
        # (#1667).
        def _poured(board):
            board.zones = [Zone(net_id=nid, net_name=name, layer=lay,
                                polygon=[(-5, -5), (35, -5), (35, 15), (-5, 15)])
                           for nid, name in ((GND, 'GND'), (AGND, 'AGND'))
                           for lay in ('F.Cu', 'B.Cu')]
            return board
        e2e = _poured(_board())
        # One via per signal net, far from any ground copper of either domain.
        e2e.vias = [make_via(4.0, 5.0, net_id=DSIG, size=0.45, drill=0.25,
                             layers=['F.Cu', 'B.Cu']),
                    make_via(24.0, 5.0, net_id=ASIG, size=0.45, drill=0.25,
                             layers=['F.Cu', 'B.Cu'])]
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.1,
                              track_width=0.15, clearance=0.15, via_size=0.45,
                              via_drill=0.25, hole_to_hole_clearance=0.25)
        coord = GridCoord(cfg.grid_step)
        obstacles = build_base_obstacle_map(e2e, cfg, [])
        placed = add_gnd_vias_to_existing_board(e2e, '', 3.0, cfg, obstacles, coord)
        by_net = {}
        for v in placed:
            by_net.setdefault(v.net_id, 0)
            by_net[v.net_id] += 1
        check(by_net.get(AGND, 0) >= 1,
              f"the analog via must get an AGND return via, got {by_net} -- "
              f"placing it on GND is the domain bridge this fixes")
        check(by_net.get(GND, 0) >= 1,
              f"the digital via must get a GND return via, got {by_net}")

        # Explicit pin: everything on GND, including the analog via.
        e2e2 = _poured(_board())
        e2e2.vias = list(e2e.vias)
        pinned = add_gnd_vias_to_existing_board(
            e2e2, 'GND', 3.0, cfg, build_base_obstacle_map(e2e2, cfg, []), coord)
        check(pinned and all(v.net_id == GND for v in pinned),
              f"an explicit net must pin every return via to it, got "
              f"{[v.net_id for v in pinned]}")

    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} check(s) FAILED")
        return False
    print("PASS  #489 §5 ground domains (split kept apart, single-ground unchanged)")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
