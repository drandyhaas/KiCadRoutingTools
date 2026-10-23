#!/usr/bin/env python3
"""#1362: a pair whose own fanout escapes lead away from its far terminal is
not electrically short, however close its pads are.

route_diff defers a 2-terminal pair to the single-ended member fallback when
the pad-to-pad distance is under the coupled floor. That ignored the pair's
escape copper: on ulx3s FPDI_D0 (U1.A16 -> C42, 3.94 mm) the escapes run
5.3 mm east of the balls while the caps sit north, so the fallback joined one
member through its ball via and the other along its escape (0 mm coupled,
7.4 mm skew at the diff step). The short test now also measures each member
along the run its own copper commits it to (`member_run_mm`): that copper's
length plus the gap from its free tips. Copper only lengthens the run, so the
change can only turn a short verdict into a coupled attempt.

Fixture: P/N pads 0.3 mm apart at U1, their partners 1.2 mm north at J1 (under
the 3.0 mm floor at width 0.15 / gap 0.15). Arms:
  ESCAPED  -- both members carry a 3 mm escape running SOUTH, away from J1:
              not short; route_diff says why and routes the pair coupled.
  BARE     -- no copper (negative control): short, deferred as before.
  TOWARD   -- a 0.4 mm escape running NORTH, toward J1: still short (the
              run stays under the floor), deferred as before.
The GUI's `diff_pair_is_short` must give the router's verdict on every arm.

    python3 tests/test_1362_escape_aware_short_pair.py
"""
import io
import json
import os
import re
import sys
from contextlib import redirect_stdout

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_HERE, _ROOT, os.path.join(_ROOT, 'py_router'), os.path.join(_ROOT, 'py_tools')):
    sys.path.insert(0, _p)

from kicad_parser import BoardInfo
from synth import make_net, make_pad, make_pcb, make_seg

P, N = 1, 2
WIDTH = GAP = 0.15

CHECKS = []


def check(name, ok, detail=''):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f"  [{detail}]" if detail else ''))


def _board(escape):
    """escape: None, 'away' (3 mm south) or 'toward' (0.4 mm north)."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 12.0, 10.0))
    pads = {
        P: [make_pad(P, 6.0, 3.0, ref='U1', num='1', net_name='/D_P', size_x=0.2, size_y=0.2),
            make_pad(P, 6.0, 1.8, ref='J1', num='1', net_name='/D_P', size_x=0.2, size_y=0.2)],
        N: [make_pad(N, 6.3, 3.0, ref='U1', num='2', net_name='/D_N', size_x=0.2, size_y=0.2),
            make_pad(N, 6.3, 1.8, ref='J1', num='2', net_name='/D_N', size_x=0.2, size_y=0.2)],
    }
    segs = []
    if escape == 'away':
        segs = [make_seg(6.0, 3.0, 6.0, 6.0, net_id=P, width=WIDTH),
                make_seg(6.3, 3.0, 6.3, 6.0, net_id=N, width=WIDTH)]
    elif escape == 'toward':
        segs = [make_seg(6.0, 3.0, 6.0, 2.6, net_id=P, width=WIDTH),
                make_seg(6.3, 3.0, 6.3, 2.6, net_id=N, width=WIDTH)]
    return make_pcb(nets={P: make_net(P, '/D_P'), N: make_net(N, '/D_N')},
                    segments=segs, pads_by_net=pads, board_info=bi)


def _route(pcb):
    from route_diff import batch_route_diff_pairs
    buf = io.StringIO()
    with redirect_stdout(buf):
        res = batch_route_diff_pairs(
            'synthetic', '', ['/D_*'], layers=['F.Cu', 'B.Cu'],
            clearance=0.1, track_width=WIDTH, diff_pair_gap=GAP,
            via_size=0.5, via_drill=0.3, grid_step=0.05,
            return_results=True, pcb_data=pcb)
    out = buf.getvalue()
    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    return res, out, (json.loads(m[-1]) if m else None)


def _verdicts(pcb):
    from diff_pair_multipoint import (get_diff_pair_terminals, leg_electrically_short,
                                      pair_leg_electrically_short, member_run_mm,
                                      diff_pair_is_short)
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(track_width=WIDTH, diff_pair_gap=GAP, clearance=0.1,
                          grid_step=0.05, layers=['F.Cu', 'B.Cu'])
    terms = get_diff_pair_terminals(pcb, P, N)
    runs = [member_run_mm(pcb, terms[0][k].net_id, terms[0][k], terms[1][k], cfg)
            for k in (0, 1)]
    return {'terms': len(terms),
            'pad_short': leg_electrically_short(terms[0], terms[1], cfg),
            'short': pair_leg_electrically_short(pcb, terms[0], terms[1], cfg),
            'gui_short': diff_pair_is_short(pcb, P, N, WIDTH, GAP, None),
            'runs': [round(r, 3) for r in runs]}


def main():
    print("#1362 escape-aware electrically-short test")

    # Preconditions: the fixture is what the test claims -- one 2-terminal
    # pair, short pad to pad at this geometry (floor 3.0 mm, span 1.2 mm).
    from diff_pair_multipoint import diff_pair_min_coupled_length
    from routing_config import GridRouteConfig
    floor = diff_pair_min_coupled_length(GridRouteConfig(
        track_width=WIDTH, diff_pair_gap=GAP, grid_step=0.05, layers=['F.Cu', 'B.Cu']))
    check("precondition: coupled floor is 3.0 mm", abs(floor - 3.0) < 1e-9, f"{floor}")

    arms = {}
    for arm in ('away', None, 'toward'):
        v = _verdicts(_board(arm))
        arms[arm] = v
        check(f"precondition [{arm}]: one 2-terminal pair, short pad to pad",
              v['terms'] == 2 and v['pad_short'], str(v))

    a, b, t = arms['away'], arms[None], arms['toward']
    check("ESCAPED: each member's run counts its 3 mm escape (>= floor)",
          min(a['runs']) >= floor and all(abs(r - 7.2) < 1e-6 for r in a['runs']),
          str(a['runs']))
    check("ESCAPED: not electrically short", a['short'] is False)
    check("BARE (control): run is the pad-to-pad 1.2 mm", b['runs'] == [1.2, 1.2], str(b['runs']))
    check("BARE (control): still short", b['short'] is True)
    check("TOWARD (control): a run under the floor stays short",
          t['short'] is True and max(t['runs']) < floor, str(t['runs']))
    for arm, v in arms.items():
        check(f"GUI diff_pair_is_short agrees with the router [{arm}]",
              v['gui_short'] == v['short'])

    # End to end through route_diff: the verdict decides the pair's path.
    (_ok, _f, _t, _rd), out, js = _route(_board('away'))
    check("ESCAPED: route_diff ran to a JSON_SUMMARY", js is not None)
    js = js or {}
    check("ESCAPED: route_diff says WHY it does not defer",
          'Short pad to pad, but its own escape copper' in out)
    check("ESCAPED: not deferred as electrically short",
          'Electrically short (' not in out
          and '/D' not in (js.get('single_ended_diff_pairs') or []))
    check("ESCAPED: routed as a coupled pair from its escape tips",
          '/D' in (js.get('routed_diff_pairs') or []), str(js.get('routed_diff_pairs')))

    (_ok, _f, _t, _rd), out_b, js_b = _route(_board(None))
    js_b = js_b or {}
    check("BARE (control): deferred as electrically short, as before",
          'Electrically short (' in out_b
          and '/D' in (js_b.get('single_ended_diff_pairs') or [])
          and 'Short pad to pad' not in out_b)

    failed = [n for n, ok in CHECKS if not ok]
    print(f"{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
    if failed:
        print("FAILED: " + "; ".join(failed))
        sys.exit(1)
    print("ALL PASS")


if __name__ == '__main__':
    main()
