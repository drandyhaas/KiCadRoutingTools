#!/usr/bin/env python3
"""The #444 chain seam re-ask never replaces a committed leg with copper that
does not join the leg's pads (#1303 follow-up).

`seam_reask_chain_leg` runs on a leg that is already on the board. Its
connectivity gate (`_hybrid_route_connects`) checked the board's copper PLUS
the candidate -- and the board still held the leg's own old copper, which
joins the pads by itself. So any candidate passed, an empty one included:
daisy_pedal_board /USBCONN-D's leg 1 was "replaced" by no copper (score
18.2 -> 9.0), route_diff demoted the pair ("no coupled trunk copper kept",
member pads disconnected) and route.py later routed it single-ended, 0 mm
coupled with 10.2 mm of skew. The gate now leaves out the copper the
candidate replaces.

Driven on a committed leg (P and N between two pad pairs, never side by side) with
the REAL connectivity gate and a stubbed leg router that offers
  - nothing for either net (refused: the pads would be left unjoined),
  - a shorter side-by-side route that joins the pads (taken).

    python3 tests/test_1303_chain_reask_keeps_copper.py
"""
import contextlib
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import diff_pair_routing as dpr                                 # noqa: E402
import pcb_modification                                          # noqa: E402
from kicad_parser import (BoardInfo, Net, Pad, PCBData,          # noqa: E402
                          Segment)
from routing_config import GridRouteConfig                      # noqa: E402
from types import SimpleNamespace                               # noqa: E402

failures = []
P, N = 1, 2


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def seg(net, pts, layer='F.Cu'):
    return [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.1,
                    layer=layer, net_id=net) for a, b in zip(pts, pts[1:])]


def pad(net, x, y):
    return Pad(pad_number='1', net_id=net, net_name=f'D{"_P" if net == P else "_N"}',
               shape='rect', global_x=x, global_y=y, local_x=0.0, local_y=0.0,
               size_x=0.15, size_y=0.15, layers=['F.Cu'], drill=0.0,
               pad_type='smd', component_ref='U1')


# the committed leg, as on daisy: P and N never side by side (less than a
# pitch alongside, so the split gate has no coupled run to guard and the
# connectivity gate alone stands between the leg and an empty candidate)
OLD = (seg(P, [(0, 0), (0, 3), (8, 3), (8, 0)])
       + seg(N, [(0, 0.3), (4, -3.0), (8, 0.3)]))
SHORT = {P: seg(P, [(0, 0), (8, 0)]), N: seg(N, [(0, 0.3), (8, 0.3)])}
EMPTY = {P: [], N: []}


def run(offer):
    info = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    pads = {P: [pad(P, 0, 0), pad(P, 8, 0)], N: [pad(N, 0, 0.3), pad(N, 8, 0.3)]}
    pcb = PCBData(board_info=info, nets={P: Net(net_id=P, name='D_P'),
                                         N: Net(net_id=N, name='D_N')},
                  footprints={}, segments=list(OLD), vias=[], pads_by_net=pads)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.1, diff_pair_gap=0.1,
                          clearance=0.1, grid_step=0.1)
    pair = SimpleNamespace(p_net_id=P, n_net_id=N)
    leg = {'_leg_terms': ((pads[P][0], pads[N][0]), (pads[P][1], pads[N][1])),
           'new_segments': list(OLD), 'new_vias': []}
    saved = (dpr._route_hybrid_leg, dpr._pn_tracks_cross_full,
             dpr._connector_grazes_foreign_copper)
    dpr._route_hybrid_leg = lambda _pcb, net, *a, **k: (list(offer[net]), [], 1, [])
    dpr._pn_tracks_cross_full = lambda *a, **k: False
    dpr._connector_grazes_foreign_copper = lambda *a, **k: None
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            replaced = dpr.seam_reask_chain_leg(pcb, pair, cfg, None, ['F.Cu', 'B.Cu'], leg)
    finally:
        (dpr._route_hybrid_leg, dpr._pn_tracks_cross_full,
         dpr._connector_grazes_foreign_copper) = saved
    return replaced, leg, pcb, out.getvalue()


def main():
    import env_knobs
    # grouped-placement removed the knob (66522c20): the re-ask is always on
    if not getattr(env_knobs, 'SEAM_REASK', True):
        print('  (KICAD_SEAM_REASK=0: the re-ask is disabled, nothing to drive)')
        print('PASS')
        return 0
    from check_connected import check_net_connectivity

    replaced, leg, pcb, log = run(EMPTY)
    check('an empty re-ask of a committed leg is refused', not replaced, log.strip())
    check('...the leg keeps its copper', leg['new_segments'] == OLD)
    for nid in (P, N):
        r = check_net_connectivity(nid, [s for s in pcb.segments if s.net_id == nid],
                                   [], pcb.pads_by_net[nid])
        check(f'...and net {nid} stays joined on the board', bool(r.get('connected')))

    replaced, leg, pcb, log = run(SHORT)
    check('a shorter re-ask that joins the pads is taken',
          replaced and len(leg['new_segments']) == 2, log.strip())

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
