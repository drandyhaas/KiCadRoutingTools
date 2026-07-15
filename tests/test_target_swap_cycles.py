#!/usr/bin/env python3
"""Multi-round target-swap chains must not corrupt the target_swaps map (#380).

apply_single_swap / apply_single_ended_swap recorded each swap as a symmetric
involution (target_swaps[a]=b; target_swaps[b]=a). That is correct only for
disjoint pairwise transpositions. The multi-round crossing optimizer, however,
returns an ordered sequence that can share an index between rounds -- select
(A,B), then a later round adds (B,C) -- which realizes the 3-cycle A->B->C->A.
Applied through the involution the map became {A:B, B:C, C:B}: two keys map to
B, A is never a value, and the true permutation is unrepresentable. That
corrupt map then mis-resolved the layer-swap partner (layer_swap_fallback) and
was garbled by the `k < v` summary dedup.

record_target_swap composes transpositions into a faithful permutation map, and
summarize_target_swaps enumerates it cycle-safely. This pins both.

    python3 tests/test_target_swap_cycles.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from target_swap import record_target_swap, summarize_target_swaps


def _apply(seq):
    """Apply an ordered transposition sequence and return the recorded map."""
    ts = {}
    for a, b in seq:
        record_target_swap(ts, a, b)
    return ts


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # 1. Disjoint pairwise transpositions still yield the old symmetric involution.
    check("disjoint transpositions -> involution",
          _apply([('A', 'B'), ('C', 'D')]) == {'A': 'B', 'B': 'A', 'C': 'D', 'D': 'C'})

    # 2. The reported 3-cycle: (A,B) then (B,C) -> A->B->C->A, NOT {A:B,B:C,C:B}.
    three = _apply([('A', 'B'), ('B', 'C')])
    check("3-cycle recorded faithfully", three == {'A': 'B', 'B': 'C', 'C': 'A'})
    check("3-cycle is a bijection (every pair is some pair's new target)",
          sorted(three.keys()) == sorted(three.values()))
    check("no key collides on one target (the old corruption)",
          len(set(three.values())) == len(three))

    # 3. partner resolution (what layer_swap_fallback does: target_swaps[name]):
    #    each cycle member's partner is the pair whose ORIGINAL target it now uses.
    check("partner of A is B", three['A'] == 'B')
    check("partner of C is A (old corrupt map wrongly said B)", three['C'] == 'A')

    # 4. Applying a transposition twice cancels it (identity is dropped).
    check("double swap cancels to identity", _apply([('A', 'B'), ('A', 'B')]) == {})

    # 5. A 4-cycle from three chained rounds.
    four = _apply([('A', 'B'), ('B', 'C'), ('C', 'D')])
    check("4-cycle recorded faithfully",
          four == {'A': 'B', 'B': 'C', 'C': 'D', 'D': 'A'})

    # 6. summarize_target_swaps: 2-cycle -> one sorted pair (matches old k<v).
    check("summary 2-cycle -> single sorted pair",
          summarize_target_swaps({'A': 'B', 'B': 'A'}) == [('A', 'B')])
    check("summary 2-cycle sorts regardless of applied order",
          summarize_target_swaps({'B': 'A', 'A': 'B'}) == [('A', 'B')])
    check("summary two disjoint 2-cycles",
          sorted(summarize_target_swaps({'A': 'B', 'B': 'A', 'C': 'D', 'D': 'C'}))
          == [('A', 'B'), ('C', 'D')])

    # 7. summary of a 3-cycle lists every edge (no member dropped), in cycle order.
    summ3 = summarize_target_swaps(three)
    check("summary 3-cycle keeps all three edges",
          sorted(summ3) == [('A', 'B'), ('B', 'C'), ('C', 'A')])
    check("summary 3-cycle has no duplicates", len(summ3) == len(set(summ3)))

    # 8. empty map -> empty summary.
    check("summary empty", summarize_target_swaps({}) == [])

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  transposition chains compose faithfully (2/3/4-cycles);")
    print("        summary is cycle-safe and matches old output for involutions")
    return 0


if __name__ == '__main__':
    sys.exit(run())
