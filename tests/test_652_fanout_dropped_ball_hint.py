#!/usr/bin/env python3
"""Issue #652: a fanout-dropped ball is diagnosed as one, not as 'no rippable
blockers found'.

A ball the fanout drops is removed from the output, and every later routing step
fails its net with `no rippable blockers found` -- which is TRUE and useless. It
invites retries that can never work (rip authority, force-reroute, smaller vias)
because the ball has no escape stub for any of them to act on. Measured on
orangecrab: EXT_PLL+ and LED_R shipped unrouted in ~15 route-step variants across
an entire campaign, including `--rip-existing-nets '*'` and `--force-reroute`,
before the cause was traced back to the fanout log.

The maintainer's closing note (2026-08-18) scopes what is left: the rescue ladder
shipped, "the DIAGNOSIS this issue asks for still does not exist ... Keeping open
for the attribution."

WHY IT IS RE-DERIVED FROM GEOMETRY. The fanout's `unescaped_nets` has ZERO
consuming code -- only tests asserting the key exists and skill-doc prose. The
step persists DRC settings and one float; `protected_nets.PRO_NAMESPACE` holds
three keys, none of them a fanout ledger. So a later `route.py` PROCESS cannot be
told, only shown. #472 settled the same point for the bare-ball zone exemption:
"board-state-driven, so no sidecar file is load-bearing".

WHY NOT `auto_detect_bga_exclusion_zones`. It walks
`find_components_by_type(pcb_data, 'BGA')` only. Measured on this repo's boards:
routed_output gets 3 zones (IC1, U3, U1) and NONE for its QFN-76 U2 -- the part
that drops balls -- and tigard gets ZERO zones. A zone-keyed diagnosis would be
silent on exactly the parts this is for. `entombed_bare_pads` uses the
footprint's own pad bounding box instead, so it works for any package.

MEASURED on tracked boards (every net, no failure filter -- the hint additionally
requires the net to have already failed):

    tigard                 85 nets   ->  2 entombed-bare pads  (U3)
    routed_output         526 nets   -> 185                    (IC1, U1, U2, U3)
    glasgow_revC          251 nets   ->  51                    (U1, U30)
    qfn_underpad_coupling   9 nets   ->   0

Of routed_output's 185, 137 are nets with NO copper anywhere on the board and 48
have copper elsewhere but NONE within 0.5 mm of the pad -- zero near-misses at
the 0.05 mm reach, which is the number that would say the tolerance is wrong.

A DISCLOSED LIMIT: on a board whose pours have not been created yet, a
plane-destined net (GND) with an entombed pad matches, because there is no zone
to consult and no copper attached. Once the pour exists `pcb_data.zones` answers
and the hint goes quiet -- `test_a_pad_served_by_a_POUR_is_not_bare` pins that.
The hint only fires for a net that has ALREADY failed to route, where saying "this
pad has no escape" is fair either way.

Run: python3 tests/test_652_fanout_dropped_ball_hint.py
"""
import io
import contextlib
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kicad_parser import BoardInfo, Footprint, Zone       # noqa: E402
from synth import make_pad, make_pcb, make_net, make_seg, make_via  # noqa: E402
from routing_common import entombed_bare_pads             # noqa: E402
from routing_diagnostics import fanout_dropped_ball_hint  # noqa: E402

X, WALL = 7, 9
CHECKS = []


def check(name, ok, detail=''):
    CHECKS.append((name, bool(ok)))
    print(('  PASS: ' if ok else '  FAIL: ') + name
          + (' -- ' + detail if detail else ''))


def _package(net_of_centre=X, *, attach=None, zone=False, drill=0.0,
             centre_net_pads=2, pitch=0.5, n=7, neighbour_drill=0.0):
    """An n x n fine-pitch package whose CENTRE ball is on `net_of_centre`.

    n = 7 at 0.5 mm pitch puts the centre ball two full rings inside the field,
    which is what `entombed_bare_pads` means by entombed. `attach` optionally
    drops a segment endpoint on that ball, `zone` optionally floods it.
    """
    pads, by_net = [], {}
    nid = 1000
    for i in range(n):
        for j in range(n):
            centre = (i == n // 2 and j == n // 2)
            net = net_of_centre if centre else (nid := nid + 1)
            p = make_pad(net, 5.0 + (i - n // 2) * pitch,
                         5.0 + (j - n // 2) * pitch, ref='U1',
                         num=f'{i}{j}', net_name='X' if centre else f'n{net}',
                         size_x=0.25, size_y=0.25)
            p.drill = drill if centre else neighbour_drill
            if not centre and neighbour_drill:
                # Through-hole neighbours block EVERY layer, so the caged ball
                # cannot escape by dropping a via either -- which is what makes
                # the failure "nothing rippable" rather than "try another
                # layer". It is also the real shape of an entombed ball.
                p.pad_type = 'thru_hole'
                p.layers = ['*.Cu']
            pads.append(p)
            by_net.setdefault(net, []).append(p)
    # X needs a second pad so it is a routable net at all (#472: a single-pad
    # net's ball is trivially bare and disabled two zones spuriously).
    for k in range(centre_net_pads - 1):
        far = make_pad(net_of_centre, 12.0 + k, 5.0, ref='U2', num=f'{k}',
                       net_name='X', size_x=0.3, size_y=0.3)
        pads.append(far)
        by_net.setdefault(net_of_centre, []).append(far)
    fp = Footprint(reference='U1', footprint_name='test:QFN', x=5.0, y=5.0,
                   rotation=0.0, layer='F.Cu', pads=pads)
    segs = list(attach or [])
    zones = []
    if zone:
        zones.append(Zone(net_id=net_of_centre, net_name='X', layer='F.Cu',
                          polygon=[(0.0, 0.0), (14.0, 0.0), (14.0, 10.0),
                                   (0.0, 10.0)]))
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 14.0, 10.0))
    return make_pcb(nets={net_of_centre: make_net(net_of_centre, 'X')},
                    footprints={'U1': fp}, segments=segs,
                    pads_by_net=by_net, board_info=bi, zones=zones)


def main():
    # 1. THE CASE. A bare ball two rings inside a fine-pitch field.
    pcb = _package()
    bare = entombed_bare_pads(pcb, [X])
    check('an entombed bare ball is found', len(bare) == 1,
          f'{[(p.pad_number, r) for p, r in bare]}')
    hint, verdict = fanout_dropped_ball_hint(pcb, None, X, 'X',
                                             return_verdict=True)
    check('the hint names the pad and the remedy',
          'fanout-dropped ball' in hint and 'U1.33' in hint
          and 'escape-method underpad' in hint, hint[:90])
    check('the verdict is structured, not prose',
          (verdict or {}).get('verdict') == 'fanout_dropped'
          and verdict.get('pad') == 'U1.33', str(verdict))

    # --- the controls it must stay silent on -----------------------------
    # 2. copper AT the ball (the #666 rescue healed it, or the fanout escaped).
    healed = _package(attach=[make_seg(5.0, 5.0, 5.0, 8.0, net_id=X,
                                       width=0.1, layer='F.Cu')])
    check('a ball with an escape stub is NOT a fanout drop',
          not entombed_bare_pads(healed, [X])
          and fanout_dropped_ball_hint(healed, None, X, 'X') == '')

    # 3. a via on the ball (the ordinary under-pad escape).
    vias = _package()
    vias.vias.append(make_via(5.0, 5.0, net_id=X, size=0.25, drill=0.15))
    check('a ball with an escape VIA is NOT a fanout drop',
          not entombed_bare_pads(vias, [X]))

    # 4. served by a POUR. The #472 closure this generalises never consulted
    #    zones, and its entombment filter selects exactly the interior region
    #    where plane-served balls live -- so this was a live false positive.
    poured = _package(zone=True)
    check('a pad served by a POUR is not bare',
          not entombed_bare_pads(poured, [X]),
          'zones were not consulted')

    # 5. a single-pad net. Trivially bare; counting it disabled U6/U7's zones
    #    spuriously on ottercast (#472).
    lone = _package(centre_net_pads=1)
    check('a single-pad net is not reported', not entombed_bare_pads(lone, [X]))

    # 6. a THROUGH-HOLE ball. Its barrel already reaches every layer.
    th = _package(drill=0.3)
    check('a drilled pad is not reported', not entombed_bare_pads(th, [X]))

    # 7. an OUTER-RING ball. Reachable through the edge band; only entombed
    #    balls are unreachable by construction.
    outer = _package()
    edge = [p for p in outer.footprints['U1'].pads
            if p.net_id != X and abs(p.global_x - 3.5) < 1e-9][0]
    check('an outer-ring bare ball is not reported',
          not any(p is edge for p, _r in entombed_bare_pads(
              outer, [edge.net_id, X])),
          'the edge band exemption is gone')

    # 8. a board with no package at all.
    tiny = make_pcb(nets={X: make_net(X, 'X')},
                    board_info=BoardInfo(layers={}, copper_layers=['F.Cu'],
                                         board_bounds=(0, 0, 10, 10)),
                    pads_by_net={X: [make_pad(X, 1.0, 1.0, ref='R1', num='1'),
                                     make_pad(X, 2.0, 1.0, ref='R1', num='2')]})
    check('a board with no pad field reports nothing',
          not entombed_bare_pads(tiny, [X]))

    # 9. END TO END: the key reaches JSON_SUMMARY and the GUI mirror.
    from route import batch_route
    caged = _package(neighbour_drill=0.3)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        _ok, _fail, _t, results_data = batch_route(
            'synthetic', '', ['X'], layers=['F.Cu', 'B.Cu'], clearance=0.2,
            track_width=0.2, via_size=0.5, via_drill=0.3, grid_step=0.1,
            ordering_strategy='original', final_reconcile=False,
            max_rip_up_count=0, return_results=True, pcb_data=caged)
    out = buf.getvalue()
    summaries = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    summary = json.loads(summaries[-1]) if summaries else {}
    fd = summary.get('fanout_dropped') or []
    check("summary['fanout_dropped'] names the net",
          any(r.get('net') == 'X' and r.get('pad') == 'U1.33' for r in fd),
          f'fanout_dropped={fd}; failed_single={summary.get("failed_single")}')
    check('the hint is printed for a human too',
          'fanout-dropped ball' in out,
          'the diagnosis is data-only, so the console still says only '
          '"no rippable blockers"')
    # The invariant test_boxed_in_summary states for its own key: the GUI reads
    # results_data, not the JSON line, so a key that lands in one and not the
    # other is invisible on one of the two fronts.
    rd = results_data if isinstance(results_data, dict) else {}
    check('results_data mirrors it for the GUI',
          bool(rd.get('fanout_dropped'))
          and rd.get('fanout_dropped') == summary.get('fanout_dropped'),
          f'results_data={rd.get("fanout_dropped")} -- an empty match on both '
          f'sides is a vacuous pass, so a non-empty list is required')

    bad = [n for n, ok in CHECKS if not ok]
    print(f"\n{'FAIL' if bad else 'PASS'}  #652 fanout-dropped diagnosis: "
          f"{len(CHECKS) - len(bad)}/{len(CHECKS)} checks")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
