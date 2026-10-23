#!/usr/bin/env python3
"""route_diff's post-route cleanup leaves a PEELED pair's copper alone until both members connect.

A multipoint pair whose far terminals route_diff peels off the coupled chain for the single-ended
follow-up (state.diff_pair_single_ended_nets) carries copper that pass still needs: h616_lpddr4's
/DRAM/SCK (U1, two U2 ball pairs, R4, C7) had its coupled U1->R4 leg end on R4's B.Cu pads through
the stub-swap vias the writer adds after the cleanup, so the dead-end sweep saw it dangling, trimmed
all 30 of its segments, and the pair reached route.py bare (routed single-ended, 0% coupled, 22 mm
of skew). `diff_cleanup_scope` decides which pairs the cleanup may touch: a peeled pair, like a
single-ended-fallback pair, only once both its members are fully connected.

    python3 tests/test_diff_cleanup_peeled_pair.py
"""
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'py_router'))

from route_diff import diff_cleanup_scope  # noqa: E402

CHECKS = []


def check(name, ok):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")


def pair(p, n):
    return (f'pair{p}', SimpleNamespace(p_net_id=p, n_net_id=n))


PAIRS = [pair(1, 2), pair(3, 4), pair(5, 6), pair(7, 8)]
ROUTED = {1: {}, 2: {}, 3: {}, 4: {}, 5: {'se_member_fallback': True}, 6: {}}   # pair 7/8 never routed


def scope(single_ended, connected):
    return diff_cleanup_scope(PAIRS, ROUTED, single_ended, lambda nid: nid in connected)


def main():
    # a coupled pair routed whole is swept; a pair with no result is not
    s = scope({}, connected=set())
    check('a plainly routed pair is in the cleanup scope', {1, 2} <= s)
    check('a pair with no routed result is left out', not ({7, 8} & s))
    check('single-ended-fallback copper with a member open is left out', not ({5, 6} & s))
    # the peeled pair: in the follow-up set, members still open -> its copper is load-bearing
    s = scope({3: '/DRAM/SCKP', 4: '/DRAM/SCKN'}, connected={1, 2})
    check('a PEELED pair with a member open is left out (its copper is the follow-up\'s)', not ({3, 4} & s))
    check('the other pairs are unaffected by it', {1, 2} <= s)
    # ...one member connected is not enough
    s = scope({3: '/DRAM/SCKP', 4: '/DRAM/SCKN'}, connected={3})
    check('a peeled pair with ONE member connected is still left out', not ({3, 4} & s))
    # ...both connected: nothing left for the follow-up, the cleanup takes it
    s = scope({3: '/DRAM/SCKP', 4: '/DRAM/SCKN'}, connected={3, 4, 5, 6})
    check('a peeled pair with both members connected is swept', {3, 4} <= s)
    check('fallback copper with both members connected is swept', {5, 6} <= s)
    bad = [n for n, ok in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(bad)}/{len(CHECKS)} checks passed")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
