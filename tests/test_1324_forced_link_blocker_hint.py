#!/usr/bin/env python3
"""The #103 blocker hint for a failed forced oracle link anchors on the link (#1324).

A plane net's exact-fill oracle link (#572) the finalize could not weld joins
the final reconciliation as a forced edge: route_oracle_links searches between
the link's OWN two endpoints. When that search failed with "no rippable
blockers found", preexisting_blocker_hint attributed the blockers around the
net's DERIVED endpoints (get_net_endpoints) -- for GND, two of its pads. On
multichannel_mixer it named Net-(IC5A-IN-), 0.2 mm from a GND pad and 29.9 mm
from the strap, while the stuck cells were Net-(IC4B-IN-)'s and
Net-(IC2B-OUT)'s; following the hint ripped the wrong net and the strap
failed again.

Pinned here:
  1. route_oracle_links' failed result names the link it failed on
     (`failed_link`), on each failure path (a search failure, a zero-copper
     "success", an endpoint layer outside the stack);
  2. preexisting_blocker_hint(anchors=...) names the copper boxing the
     anchors, and not a net that only sits by the net's pads -- the board:
     GND's pads with NET_FAR beside one, and the strap's endpoints far away
     with NET_NEAR across them;
  3. the single-ended loop hands the failed link to the hint.

    python3 tests/test_1324_forced_link_blocker_hint.py
"""
import ast
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import RoutingBoard                                  # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from routing_config import GridRouteConfig                      # noqa: E402
from run_utils import evidence                                  # noqa: E402

FAILS = []
STRAP = ((30.0, 20.0), (31.0, 20.0))


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def board(tmp):
    b = RoutingBoard(w=40, h=30)
    b.pad('G1', 5, 5, 'GND')
    b.pad('G2', 10, 5, 'GND')
    b.pad('F1', 5, 8, 'NET_FAR')            # beside GND's pad G1
    b.pad('F2', 3, 12, 'NET_FAR')
    b.seg(5, 5.6, 5, 8, 'NET_FAR', w=0.2)
    b.pad('N1', 25, 25, 'NET_NEAR')         # across the strap
    b.pad('N2', 35, 25, 'NET_NEAR')
    b.seg(25, 25, 30.5, 20.5, 'NET_NEAR', w=0.2)
    b.seg(30.5, 20.5, 35, 25, 'NET_NEAR', w=0.2)
    path = os.path.join(tmp, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(b.text())
    return parse_kicad_pcb(evidence(path, 'synthetic board'))


def failed_link_paths():
    print('1. route_oracle_links names the link it failed on')
    import single_ended_routing as ser
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    link = ((STRAP[0][0], STRAP[0][1], 'F.Cu', 'zone'),
            (STRAP[1][0], STRAP[1][1], 'F.Cu', 'pad'))
    real = ser.route_net_with_obstacles
    try:
        for label, stub in (
                ('search failure', lambda *a, **k: {'failed': True,
                                                    'iterations': 9}),
                ('zero-copper success', lambda *a, **k: {
                    'new_segments': [], 'new_vias': [], 'iterations': 1})):
            ser.route_net_with_obstacles = stub
            r = ser.route_oracle_links(None, 1, cfg, None, [link])
            check(f'{label}: failed, naming the link',
                  r.get('failed') and r.get('failed_link') == STRAP,
                  str(r.get('failed_link')))
    finally:
        ser.route_net_with_obstacles = real
    off = ((STRAP[0][0], STRAP[0][1], 'In5.Cu', 'zone'), link[1])
    r = ser.route_oracle_links(None, 1, cfg, None, [off])
    check('layer outside the stack: failed, naming the link',
          r.get('failed') and r.get('failed_link') == STRAP,
          str(r.get('failed_link')))


def hint_anchors(tmp):
    print('2. the hint attributes at the anchors it is given')
    from routing_diagnostics import preexisting_blocker_hint
    pcb = board(tmp)
    gnd = next(i for i, n in pcb.nets.items() if n.name == 'GND')
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    # One frontier cell on an inner layer: the frontier tally reads layer 0
    # only, so every name below comes from the anchors' 1 mm ring.
    cells = [(300, 200, 1)]
    _t, at_link = preexisting_blocker_hint(cells, cfg, pcb, gnd,
                                           return_names=True,
                                           anchors=list(STRAP))
    check('anchored on the strap: NET_NEAR, not NET_FAR',
          at_link == ['NET_NEAR'], str(at_link))
    _t, at_pads = preexisting_blocker_hint(cells, cfg, pcb, gnd,
                                           return_names=True,
                                           anchors=[(5.0, 5.0), (10.0, 5.0)])
    check("anchored on GND's pads: NET_FAR (the old reading)",
          at_pads == ['NET_FAR'], str(at_pads))


def loop_passes_link():
    print('3. the single-ended loop hands the failed link to the hint')
    tree = ast.parse(open(os.path.join(ROOT, 'py_router', 'single_ended_loop.py'),
                          encoding='utf-8').read())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == 'preexisting_blocker_hint']
    kw = [{k.arg for k in c.keywords} for c in calls]
    check('every call passes anchors=', calls and all('anchors' in k for k in kw),
          f'{len(calls)} call(s), keywords {kw}')


def main():
    failed_link_paths()
    with tempfile.TemporaryDirectory() as tmp:
        hint_anchors(tmp)
    loop_passes_link()
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
