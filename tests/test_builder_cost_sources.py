"""The obstacle builders price a net with the same soft-cost sources everywhere.

Rows:
  - a net is not priced against its OWN track-proximity entry (a multipoint
    net's Phase 3 taps were pushed off its own main route), nor its river
    siblings', and nothing is copied when nothing is dropped; the same view
    is handed out again while its sources are unchanged (the merge memo is
    keyed on its identity) and a new one once any of them changes; an entry
    that prices nothing is not dropped at all;
  - stub-proximity sources: every unrouted net, a multipoint net whose taps
    are still pending although its Phase 1 route is in, and a pre-existing net
    ripped this run and not yet back; never the net being routed, never a
    routed net with nothing pending, nor one whose taps Phase 3 has routed;
    a dataclasses.replace clone carries that run state (carry_run_state);
  - Phase 3's fast builder takes the ripped-route ghost ledgers, and every
    Phase 3 call hands them over (it routed blind to pending victims'
    corridors, and differently whether length matching sent it to the slow
    builder or not);
  - a ripped-route ghost reserves the ripped net's corridor: every builder
    prices it to every other net and never to the net(s) it is routing, so a
    victim's reroute is not charged for going back;
  - the diff-pair layer-swap fallback builds every map with the main loop's
    builder, as the main loop calls it (its victim-reroute map omitted the
    still-unrouted nets' copper and the pair's own same-net rings; then the
    per-net cache stamped that copper at the single-ended clearance).

    python3 tests/test_builder_cost_sources.py
"""
import inspect
import os
import re
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import routing_context as rc  # noqa: E402


def test_own_track_proximity_entry_dropped():
    cache = {5: 'own', 6: 'other', 7: 'sibling', -1: 'bga'}
    got = rc._per_net_cost_sources(cache, (5,), sibs={7})
    assert set(got) == {6, -1}, got
    assert rc._per_net_cost_sources(cache, (9,)) is cache, "copied for nothing"
    assert set(rc._per_net_cost_sources(cache, (5, 6))) == {7, -1}
    again = rc._per_net_cost_sources(cache, (5,), sibs={7})
    assert again is got, "an unchanged view was rebuilt: the merge memo misses"
    cache[6] = 'rerouted'
    fresh = rc._per_net_cost_sources(cache, (5,), sibs={7})
    assert fresh is not got and fresh[6] == 'rerouted', fresh
    # An entry that prices nothing (every routed net's at track proximity
    # cost 0) is not worth a new dict: the run's cache itself comes back.
    import numpy as np
    from plane_fragility import fragility_cache_key
    quiet = {5: np.empty((0, 4)), 6: np.ones((2, 4)),
             fragility_cache_key(5): np.empty((0, 4))}
    assert rc._per_net_cost_sources(quiet, (5,)) is quiet


def test_stub_proximity_sources():
    config = types.SimpleNamespace(_pending_multipoint={3: object()})
    pcb = types.SimpleNamespace(_preexisting_rips={8: 'OLD'})
    ids = rc._stub_proximity_source_ids(
        config, pcb, all_unrouted_net_ids=[1, 2, 3, 4],
        routed_net_ids=[2, 3], exclude={1})
    # 1 is being routed; 2 is routed and done; 3 is routed with taps pending;
    # 4 is unrouted; 8 is a ripped pre-existing net not in the batch list.
    assert ids == [3, 4, 8], ids
    bare = rc._stub_proximity_source_ids(
        types.SimpleNamespace(), types.SimpleNamespace(), [1, 2, 3], [2], {1})
    assert bare == [3], "without pending/rips it is the old rule"
    # Once Phase 3 has routed 3's taps it is a routed net like any other,
    # although the pending dict keeps it for the ripped-net reroute.
    config._multipoint_taps_done = {3}
    ids = rc._stub_proximity_source_ids(
        config, pcb, all_unrouted_net_ids=[1, 2, 3, 4],
        routed_net_ids=[2, 3], exclude={1})
    assert ids == [4, 8], ids
    # A clone made with dataclasses.replace carries that run state.
    from dataclasses import replace
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig()
    cfg._pending_multipoint, cfg._multipoint_taps_done = {3: 0}, {3}
    clone = rc.carry_run_state(cfg, replace(cfg, max_rip_up_count=0))
    assert clone._multipoint_taps_done is cfg._multipoint_taps_done
    assert not hasattr(replace(cfg), '_pending_multipoint'), \
        "replace() copies run state now; carry_run_state is moot"


def test_phase3_builder_takes_the_ghosts():
    params = inspect.signature(rc.build_incremental_obstacles).parameters
    assert 'ripped_route_layer_costs' in params
    assert 'ripped_route_via_positions' in params
    import phase3_routing
    src = inspect.getsource(phase3_routing)
    calls = [m.start() for m in re.finditer(r'build_incremental_obstacles\(', src)]
    calls = [c for c in calls if not src[max(0, c - 40):c].rstrip().endswith('import')]
    assert len(calls) >= 7, len(calls)
    for c in calls:
        call = src[c:src.index(')', c + src[c:].index('cache') + 5) + 1]
        call = src[c:c + len(call) + 300]           # the call and its tail
        assert 'ripped_route_via_positions' in call, src[c:c + 300]


def test_own_ghost_is_never_priced_to_its_reroute():
    import numpy as np
    cfg = types.SimpleNamespace(ripped_route_avoidance_cost=0.1)
    ghosts = {5: np.ones((2, 4)), 6: np.ones((2, 4)), 7: np.ones((1, 4))}
    got = rc.filter_ripped_ghosts(ghosts, cfg, routed_net_ids=[7], own_net_ids=(5,))
    assert set(got) == {6}, got
    assert set(rc.filter_ripped_ghosts(ghosts, cfg, [7])) == {5, 6}, \
        "another net still pays 5's ghost"
    for fn, own in ((rc.build_diff_pair_obstacles, '(p_net_id, n_net_id)'),
                    (rc.build_single_ended_obstacles, '(net_id,)'),
                    (rc.build_incremental_obstacles, '(net_id,)'),
                    (rc.prepare_obstacles_inplace, '(net_id,)')):
        src = inspect.getsource(fn)
        assert src.count('filter_ripped_ghosts(') == 2, fn.__name__
        assert src.count(f'routed_net_ids, {own})') == 2, fn.__name__


def test_fallback_maps_use_the_shared_builder():
    import layer_swap_fallback
    src = inspect.getsource(layer_swap_fallback.try_fallback_layer_swap)
    assert 'clone_fresh()' not in src, "a hand-built map is back"
    assert src.count('_pair_map(') >= 4, src.count('_pair_map(')   # def + 3 uses
    # As the main loop calls it: no per-net cache, whose entries are stamped
    # at extra clearance 0 where a pair needs diff_pair_extra_clearance.
    body = src[src.index('def _pair_map('):src.index('from stub_layer_switching')]
    assert 'net_obstacles_cache=' not in body, body
    import diff_pair_loop
    import reroute_loop
    for mod in (diff_pair_loop, reroute_loop):
        msrc = inspect.getsource(mod)
        i = msrc.index('try_fallback_layer_swap(')
        assert 'ripped_route_via_positions=state.ripped_route_via_positions' in \
            msrc[i:i + 2000], mod.__name__


TESTS = [test_own_track_proximity_entry_dropped, test_stub_proximity_sources,
         test_phase3_builder_takes_the_ghosts,
         test_own_ghost_is_never_priced_to_its_reroute,
         test_fallback_maps_use_the_shared_builder]


if __name__ == '__main__':
    fails = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
