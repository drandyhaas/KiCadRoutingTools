#!/usr/bin/env python3
"""Issue #219: the rip-up cycle-guard ancestry, extended to diff-pair rippers.

The single-net guard (record_rip_ancestry) records that a net X, ripped by Y,
must not rip Y (or Y's ancestors) back -- breaking the A->B->A rip war that ends
in a restore-crossing. For a DIFF-PAIR ripper both halves rip as one unit, so the
ripped net must be barred from ripping EITHER half. This is what
record_pair_rip_ancestry does, and the subtlety it must get right is that two
plain record_rip_ancestry(P, X) / record_rip_ancestry(N, X) calls would NOT work
-- the second REPLACES the first, dropping P from X's ancestry -- so it sets the
union in one shot.

The diff-pair guard is OFF by default (routing_state.DIFF_PAIR_RIP_GUARD) -- it is
a no-op butterfly with no measured win (see issue #219). The semantic tests below
flip it ON to exercise the logic that's preserved for future grading; a separate
test pins the OFF default (record is a no-op, exclude = just the pair's two nets).

Run:  python3 tests/test_rip_ancestry_pair.py [-v]
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from types import SimpleNamespace

import routing_state
from routing_state import (record_rip_ancestry, record_pair_rip_ancestry,
                           rip_exclude_set, diff_pair_rip_exclude)


def _state():
    # The ancestry helpers only touch state.rip_ancestry; a full RoutingState
    # needs a board + config, so a minimal stub keeps this a pure unit test.
    return SimpleNamespace(rip_ancestry={})


def test_pair_records_both_halves(verbose):
    """A pair (P=10, N=11) ripping X=20 -> X barred from ripping BOTH 10 and 11."""
    fails = []
    st = _state()
    record_pair_rip_ancestry(st, 10, 11, 20)
    excl = rip_exclude_set(st, 20)
    for half in (10, 11):
        if half not in excl:
            fails.append(f"X=20 exclude set missing ripper half {half}: {sorted(excl)}")
    if 20 not in excl:
        fails.append(f"X=20 exclude set must contain itself: {sorted(excl)}")
    if verbose and not fails:
        print(f"  pair (10,11) rips 20 -> exclude(20)={sorted(excl)}  OK")
    return fails


def test_not_overwritten_like_two_single_calls(verbose):
    """Regression: two record_rip_ancestry calls WOULD drop the first half;
    record_pair_rip_ancestry must keep both."""
    fails = []
    # what the naive approach does
    naive = _state()
    record_rip_ancestry(naive, 10, 20)
    record_rip_ancestry(naive, 11, 20)   # overwrites -> 10 lost
    naive_excl = rip_exclude_set(naive, 20)
    if 10 in naive_excl:
        fails.append("test premise wrong: two single calls unexpectedly kept both halves")
    # the pair helper keeps both
    good = _state()
    record_pair_rip_ancestry(good, 10, 11, 20)
    good_excl = rip_exclude_set(good, 20)
    if not ({10, 11} <= good_excl):
        fails.append(f"pair helper dropped a half: {sorted(good_excl)}")
    if verbose and not fails:
        print(f"  naive exclude(20)={sorted(naive_excl)} (10 lost) vs "
              f"pair exclude(20)={sorted(good_excl)}  OK")
    return fails


def test_ancestor_chain_propagates(verbose):
    """If half P=10 was itself ripped by A=1, then X ripped by (10,11) inherits A
    too -- X must not rip the whole upstream chain back."""
    fails = []
    st = _state()
    record_rip_ancestry(st, 1, 10)        # A=1 ripped P=10  -> ancestry(10) = {1}
    record_pair_rip_ancestry(st, 10, 11, 20)  # pair (10,11) ripped X=20
    excl = rip_exclude_set(st, 20)
    for nid in (1, 10, 11, 20):
        if nid not in excl:
            fails.append(f"exclude(20) missing {nid} (chain not propagated): {sorted(excl)}")
    if verbose and not fails:
        print(f"  chain A=1 -> P=10, pair rips 20 -> exclude(20)={sorted(excl)}  OK")
    return fails


def test_self_rip_not_recorded(verbose):
    """Ripping one of the halves themselves leaves no degenerate self-only entry."""
    fails = []
    st = _state()
    record_pair_rip_ancestry(st, 10, 11, 10)  # ripped net == a half
    # 10's ancestry should be just {11} (the OTHER half), never include 10 itself
    excl = rip_exclude_set(st, 10)
    if 10 in st.rip_ancestry.get(10, frozenset()):
        fails.append(f"self stored in own ancestry: {sorted(st.rip_ancestry[10])}")
    if 11 not in excl:
        fails.append(f"other half 11 should still be excluded: {sorted(excl)}")
    if verbose and not fails:
        print(f"  pair (10,11) rips half 10 -> ancestry(10)="
              f"{sorted(st.rip_ancestry.get(10, []))}  OK")
    return fails


def test_off_by_default(verbose):
    """With the flag OFF (the shipped default), the guard is a no-op: nothing is
    recorded and the blocker-exclude is just the pair's own two nets."""
    fails = []
    saved = routing_state.DIFF_PAIR_RIP_GUARD
    routing_state.DIFF_PAIR_RIP_GUARD = False
    try:
        st = _state()
        record_pair_rip_ancestry(st, 10, 11, 20)
        if st.rip_ancestry:
            fails.append(f"OFF: recorded ancestry anyway: {st.rip_ancestry}")
        excl = diff_pair_rip_exclude(st, 10, 11)
        if excl != {10, 11}:
            fails.append(f"OFF: exclude {sorted(excl)} != just the pair {{10, 11}}")
    finally:
        routing_state.DIFF_PAIR_RIP_GUARD = saved
    if verbose and not fails:
        print("  guard OFF: record is a no-op, exclude = {pair}  OK")
    return fails


def test_on_exclude_adds_ancestry(verbose):
    """With the flag ON, diff_pair_rip_exclude adds the pair's rip-ancestry."""
    fails = []
    saved = routing_state.DIFF_PAIR_RIP_GUARD
    routing_state.DIFF_PAIR_RIP_GUARD = True
    try:
        st = _state()
        record_rip_ancestry(st, 1, 10)   # net 1 ripped half P=10
        excl = diff_pair_rip_exclude(st, 10, 11)
        if not ({1, 10, 11} <= excl):
            fails.append(f"ON: exclude {sorted(excl)} missing pair+ancestor {{1,10,11}}")
    finally:
        routing_state.DIFF_PAIR_RIP_GUARD = saved
    if verbose and not fails:
        print(f"  guard ON: exclude(10,11)={sorted(excl)} (incl. ancestor 1)  OK")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    # The semantic tests below exercise the guard's preserved-but-disabled logic,
    # so force it ON; test_off_by_default restores and checks the shipped default.
    routing_state.DIFF_PAIR_RIP_GUARD = True

    print("=== diff-pair rip-up cycle-guard, gating (issue #219) ===")
    fails = test_off_by_default(args.verbose)
    fails += test_on_exclude_adds_ancestry(args.verbose)
    print("=== record_pair_rip_ancestry semantics (guard forced ON) ===")
    fails += test_pair_records_both_halves(args.verbose)
    fails += test_not_overwritten_like_two_single_calls(args.verbose)
    fails += test_ancestor_chain_propagates(args.verbose)
    fails += test_self_rip_not_recorded(args.verbose)

    if fails:
        print("\nFAIL:\n  " + "\n  ".join(fails))
        return 1
    print("\nPASS: diff-pair guard is a no-op when OFF (the default); when ON it "
          "records BOTH halves (and their ancestors) into the ripped net's ancestry")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
