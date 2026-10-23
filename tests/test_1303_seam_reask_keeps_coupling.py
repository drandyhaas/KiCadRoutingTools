#!/usr/bin/env python3
"""#1303: a #444 seam re-ask never splits a coupled pair -- and never refuses
one that only runs it alongside at a looser pitch.

Both re-asks -- the hybrid's joint whole-connection A* and the chain re-ask
of a committed multipoint leg -- priced a candidate by length, vias and
`_split_len` (copper farther than 1 mm from its partner). On imx6ulz DQS0 the
joint re-ask swapped a coupled middle for P on In2.Cu and N on B.Cu (0 mm
coupled) because the composed pair's long legs scored worse (185.0 -> 100.4).

The first gate measured coupling at 1.5x the routed gap and refused any loss
of coupled LENGTH. A re-ask is one single-ended A* per net attracted to its
partner, so it runs alongside at the routing clearance or wider: nearly every
candidate read 0 mm coupled, every re-ask on a coupled middle was refused,
and the composed detours shipped (13 corpus boards lost nets).

Now a candidate is refused when it SPLITS the pair
(`diff_pair_routing.seam_splits_pair`): the share of its route alongside the
partner -- KiCad's coupling geometry, parallel facing spans on one layer at
any distance -- falls below half the composed route's.

The chain re-ask is driven here on a synthetic leg: a coupled 4 mm middle
with long split legs (the shape that made the old score take anything
shorter), and a stubbed leg router that offers two short routes that are
  - on different layers (split -- refused),
  - on one layer but never parallel (split -- refused),
  - side by side at the gap (taken),
  - side by side at a 0.5 mm and a 1.5 mm gap (taken: the case the first
    gate refused).

    python3 tests/test_1303_seam_reask_keeps_coupling.py
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
               size_x=0.1, size_y=0.1, layers=['F.Cu', 'B.Cu'], drill=0.0,
               pad_type='smd', component_ref='U1')


def board():
    info = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    pads = {P: [pad(P, 0, 0), pad(P, 4, 0)], N: [pad(N, -5, 0.2), pad(N, 9, 0.2)]}
    return PCBData(board_info=info, nets={P: Net(net_id=P, name='D_P'),
                                          N: Net(net_id=N, name='D_N')},
                   footprints={}, segments=[], vias=[], pads_by_net=pads)


# the committed leg: a 4 mm coupled middle at y=10 with long split legs (N's
# outer legs on B.Cu, so nothing but the middle faces P)
BASE_P = seg(P, [(0, 0), (0, 10), (4, 10), (4, 0)])
BASE_N = (seg(N, [(-5, 0.2), (-5, 10.2), (0, 10.2)], 'B.Cu')
          + seg(N, [(0, 10.2), (4, 10.2)])
          + seg(N, [(4, 10.2), (9, 10.2), (9, 0.2)], 'B.Cu'))
UNCOUPLED = {P: seg(P, [(0, 0), (4, 0)]), N: seg(N, [(-5, 0.2), (9, 0.2)], 'B.Cu')}
ASKEW = {P: seg(P, [(0, 0), (4, 0)]), N: seg(N, [(-5, 0.2), (2, 3.2), (9, 0.2)])}
COUPLED = {P: seg(P, [(0, 0), (4, 0)]), N: seg(N, [(-5, 0.2), (9, 0.2)])}
LOOSE = {P: seg(P, [(0, 0), (4, 0)]), N: seg(N, [(-5, 0.6), (9, 0.6)])}
WIDE = {P: seg(P, [(0, 0), (4, 0)]), N: seg(N, [(-5, 1.6), (9, 1.6)])}


def run(offer):
    pcb = board()
    pcb.segments = list(BASE_P + BASE_N)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.1, diff_pair_gap=0.1,
                          clearance=0.1, grid_step=0.1)
    pair = SimpleNamespace(p_net_id=P, n_net_id=N)
    pp, nn = pcb.pads_by_net[P], pcb.pads_by_net[N]
    leg = {'_leg_terms': ((pp[0], nn[0]), (pp[1], nn[1])),
           'new_segments': list(BASE_P + BASE_N), 'new_vias': []}
    saved = (dpr._route_hybrid_leg, dpr._pn_tracks_cross_full,
             dpr._connector_grazes_foreign_copper, dpr._hybrid_route_connects,
             pcb_modification.add_route_to_pcb_data,
             pcb_modification.remove_route_from_pcb_data)
    dpr._route_hybrid_leg = lambda _pcb, net, *a, **k: (list(offer[net]), [], 1, [])
    dpr._pn_tracks_cross_full = lambda *a, **k: False
    dpr._connector_grazes_foreign_copper = lambda *a, **k: None
    dpr._hybrid_route_connects = lambda *a, **k: True
    pcb_modification.add_route_to_pcb_data = lambda *a, **k: None
    pcb_modification.remove_route_from_pcb_data = lambda *a, **k: None
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            replaced = dpr.seam_reask_chain_leg(pcb, pair, cfg, None, ['F.Cu', 'B.Cu'], leg)
    finally:
        (dpr._route_hybrid_leg, dpr._pn_tracks_cross_full,
         dpr._connector_grazes_foreign_copper, dpr._hybrid_route_connects,
         pcb_modification.add_route_to_pcb_data,
         pcb_modification.remove_route_from_pcb_data) = saved
    return replaced, leg, out.getvalue()


def main():
    def share(offer):
        al, ln = dpr.seam_coupling(offer[P] + offer[N], P, N)
        return al / ln

    base_al, base_len = dpr.seam_coupling(BASE_P + BASE_N, P, N)
    check('seam_coupling: the committed leg carries its 4 mm coupled middle on a '
          '34 mm route', abs(base_al - 4.0) < 1e-6 and abs(base_len - 34.0) < 1e-6,
          f'{base_al}, {base_len}')
    check('seam_coupling: P and N on different layers run 0 mm alongside',
          share(UNCOUPLED) == 0.0)
    check('seam_coupling: one layer but never parallel is not alongside',
          share(ASKEW) == 0.0)
    check('seam_coupling: side by side at the gap, 4 of 14 mm alongside',
          abs(share(COUPLED) - 4 / 14) < 1e-6, str(share(COUPLED)))
    for name, offer in (('0.5 mm', LOOSE), ('1.5 mm', WIDE)):
        check(f'seam_coupling: side by side at a {name} gap is alongside too',
              abs(share(offer) - 4 / 14) < 1e-6, str(share(offer)))
    if env_knobs_off():
        print('  (KICAD_SEAM_REASK=0: the re-ask is disabled, nothing to drive)')
    else:
        for name, offer in (('on different layers', UNCOUPLED),
                            ('on one layer but never parallel', ASKEW)):
            replaced, leg, log = run(offer)
            check(f'a shorter re-ask {name} SPLITS the pair and is refused',
                  not replaced and leg['new_segments'] == BASE_P + BASE_N, log.strip())
            check('...and says so', 'chain seam re-ask rejected: it splits the pair '
                                    '(12% -> 0% of the route alongside its partner)' in log,
                  log.strip())
        for name, offer in (('at the gap', COUPLED), ('at a 0.5 mm gap', LOOSE),
                            ('at a 1.5 mm gap', WIDE)):
            replaced, leg, log = run(offer)
            check(f'a shorter re-ask side by side {name} is taken',
                  replaced and len(leg['new_segments']) == 2, log.strip())
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


def env_knobs_off():
    import env_knobs
    # grouped-placement removed the knob (66522c20): the re-ask is always on
    return not getattr(env_knobs, 'SEAM_REASK', True)


if __name__ == '__main__':
    sys.exit(main())
