#!/usr/bin/env python3
"""#1871: a multi-leg pair carries the self-graze marker of ANY of its legs.

The hybrid's self-graze fallback marks the leg it laid (`selfgraze`).
`_route_terminal_set` merged a multi-leg pair's legs as a copy of the LAST
leg, so the pair carried that leg's marker alone: a pair whose earlier leg
took the fallback had no marker, `judge_selfgraze_on_copper` skipped it, and
it was recorded protected with no judgement of the copper it shipped.

The merged result now carries the largest marker of any leg, so the
judgement reads the whole pair.

Rows:
  1. `_route_terminal_set` (the four things it calls to order, route and
     polish the legs replaced by stand-ins, so its own merge is what runs):
     the marker of an earlier leg reaches the merged result; the last
     leg's still does; two marked legs give the larger, whichever comes
     last; no marked leg gives no key;
  2. such a pair through the judgement: P and N 0.1 mm apart under a
     0.25 mm clearance stay out of the record and are listed; 1.0 mm apart
     they are admitted.

    python3 tests/test_1871_selfgraze_marker_any_leg.py
"""
import os
import sys
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from synth import make_seg, make_net, make_pcb             # noqa: E402
from kicad_parser import BoardInfo                         # noqa: E402
import diff_pair_multipoint as dpm                         # noqa: E402
import diff_pair_routing                                   # noqa: E402

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def merged_of(markers):
    """`_route_terminal_set`'s merged result for legs carrying `markers`
    (None: a leg with no marker), in leg order."""
    legs = []
    for m in markers:
        leg = {'new_segments': [], 'new_vias': [], 'iterations': 1,
               'hybrid_escape': True}
        if m is not None:
            leg['selfgraze'] = m
        legs.append(leg)
    real = (dpm.candidate_terminal_chains, dpm._route_chain_attempt,
            dpm.build_diff_pair_leg_obstacles, diff_pair_routing.seam_reask_chain_leg)
    dpm.candidate_terminal_chains = lambda terminals: [list(terminals)]
    dpm._route_chain_attempt = lambda state, pair, name, chain: (legs, [])
    dpm.build_diff_pair_leg_obstacles = lambda *a, **k: None
    diff_pair_routing.seam_reask_chain_leg = lambda *a, **k: None
    state = SimpleNamespace(
        pcb_data=None, config=SimpleNamespace(layers=['F.Cu', 'B.Cu']),
        base_obstacles=None, routed_net_ids=set(), remaining_net_ids=set(),
        all_unrouted_net_ids=set(), gnd_net_id=None, track_proximity_cache={},
        layer_map={})
    pair = SimpleNamespace(p_net_id=1, n_net_id=2)
    try:
        leg_results, merged, _se = dpm._route_terminal_set(
            state, pair, 'X', [('a', 'b'), ('c', 'd'), ('e', 'f')])
    finally:
        (dpm.candidate_terminal_chains, dpm._route_chain_attempt,
         dpm.build_diff_pair_leg_obstacles,
         diff_pair_routing.seam_reask_chain_leg) = real
    assert leg_results is legs and merged is not None, 'the stand-in legs were not merged'
    return merged


def merge_rows():
    print("1. the marker a multi-leg pair's merged result carries")
    m = merged_of([2, None])
    ok(m.get('selfgraze') == 2, f"an earlier leg's marker reaches it: {m.get('selfgraze')}")
    m = merged_of([None, 3])
    ok(m.get('selfgraze') == 3, f"the last leg's still does: {m.get('selfgraze')}")
    m = merged_of([4, 1])
    ok(m.get('selfgraze') == 4, f"two marked legs give the larger, not the last: "
                                f"{m.get('selfgraze')}")
    m = merged_of([None, None])
    ok('selfgraze' not in m, f"no marked leg, no key: {m.get('selfgraze')}")
    ok(m.get('is_diff_pair') is True and m.get('is_multipoint') is True
       and len(m.get('leg_results') or ()) == 2, "the rest of the merge is as it was")
    return merged_of([2, None])


def judged_rows(merged):
    print("2. such a pair through the judgement on shipped copper")
    from routing_config import GridRouteConfig
    from route_diff import (judge_selfgraze_on_copper, protection_candidates,
                            unprotected_pair_rows)
    cfg = GridRouteConfig(clearance=0.25, layers=['F.Cu', 'B.Cu'])
    pair = SimpleNamespace(p_net_id=1, n_net_id=2, p_net_name='X_P', n_net_name='X_N')
    pairs = [('X', pair)]

    def board(gap):
        segs = [make_seg(0, 0, 10, 0, net_id=1),
                make_seg(0, 0.2 + gap, 10, 0.2 + gap, net_id=2)]
        return make_pcb(nets={1: make_net(1, 'X_P'), 2: make_net(2, 'X_N')},
                        segments=segs, vias=[], pads_by_net={1: [], 2: []},
                        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                                             copper_layers=['F.Cu', 'B.Cu'],
                                             board_bounds=(-1.0, -1.0, 14.0, 3.0)),
                        zones=[])

    res = {1: dict(merged), 2: dict(merged)}
    pcb = board(0.1)
    short = judge_selfgraze_on_copper(res, pcb, cfg, pairs)
    prot = protection_candidates(res, pcb, pairs=pairs)
    rows = unprotected_pair_rows(pairs, prot, res, None, None, short, {'X'})
    ok(short.get('X', 0) >= 1 and prot == {}
       and [r['reason'] for r in rows] == ['self-graze fallback'],
       f"0.1 mm apart: judged, left out and listed: {short}, {rows}")
    res = {1: dict(merged), 2: dict(merged)}
    pcb = board(1.0)
    short = judge_selfgraze_on_copper(res, pcb, cfg, pairs)
    prot = protection_candidates(res, pcb, pairs=pairs)
    ok(short == {} and prot == {'X_P': 'diff-pair', 'X_N': 'diff-pair'},
       f"1.0 mm apart: judged and admitted: {prot}")


def main():
    try:
        merged = merge_rows()
        judged_rows(merged)
    except AssertionError as exc:
        print(f"  [BROKEN TEST] {exc}")
        return 1
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
