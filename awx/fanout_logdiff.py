#!/usr/bin/env python3
"""fanout_logdiff.py LOG_A LOG_B [--context N] -- where two runs of the fanout on our own ends first DECIDED differently:
two fo.log files (fanout_from_plan.py under PLAN_JUDGE=ends -- a whole_route.py round's OUTDIR/rN/fo.log) compared on
their decision lines only, the ends model's ('whole ends: ...'), the destination passes', the source realizes' and the
plans', with N lines of each around the first that differs.

What it is for: two machines may route a rung to different copper (each keeps its own plan among equal optima), and
the first decision that parts says which stage to look at. The rest of a log is noise for this: a cold cache prints
lines a warm one does not (the taut memo's), and paths differ. Exits 0 when the decisions agree, 1 when they part.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import sys

KEYS = ('whole ends:', 'destination pass', 'destination re-plan', 'destination:', 'source residue', 'plan (')


def decisions(path):
    """the decision lines of a fanout log, in order, stripped"""
    return [ln.strip() for ln in open(path, errors='replace') if any(k in ln for k in KEYS)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('log_a')
    ap.add_argument('log_b')
    ap.add_argument('--context', type=int, default=2)
    a = ap.parse_args()
    A, B = decisions(a.log_a), decisions(a.log_b)
    for i in range(max(len(A), len(B))):
        x = A[i] if i < len(A) else '(none)'
        y = B[i] if i < len(B) else '(none)'
        if x != y:
            print(f'the decisions part at decision {i + 1} of {len(A)} / {len(B)}:')
            for tag, L in (('A', A), ('B', B)):
                for j in range(max(0, i - a.context), min(len(L), i + a.context + 1)):
                    print(f' {tag} {j + 1:3d}{" >" if j == i else "  "} {L[j][:220]}')
                print()
            sys.exit(1)
    print(f'the same {len(A)} decisions')


if __name__ == '__main__':
    main()
