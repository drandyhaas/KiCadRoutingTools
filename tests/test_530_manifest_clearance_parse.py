#!/usr/bin/env python3
"""#530: the corpus graders read the routed floor off a manifest that says
either `--clearance X` (pre-rewrite) or `--clearance-ceiling X` (the recorded
reading, rewritten 2026-09-03), and grade at the MINIMUM over the routing
steps. A grader that only knew the old spelling would fall back to its
default and phantom-flag every rewritten board."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
from ab_replay_grade import route_clearance  # noqa: E402


def main():
    fails = []
    cases = [
        ("route.py a b --nets '*' --clearance 0.15 --track-width 0.2\n", "0.15", "old spelling"),
        ("route.py a b --nets '*' --clearance-ceiling 0.15 --track-width 0.2\n", "0.15", "rewritten spelling"),
        ("bga_fanout.py a -c U1 --clearance 0.09\nroute_diff.py a b --clearance-ceiling 0.1\n"
         "route.py b c --clearance-ceiling 0.2\n", "0.09", "minimum over mixed spellings"),
        ("check_drc.py a --clearance-margin 0.1\n", "0.1", "no routed floor -> the default"),
    ]
    for txt, want, name in cases:
        got = route_clearance(txt)
        if got != want:
            fails.append(f"{name}: route_clearance -> {got!r}, want {want!r}")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: route_clearance reads --clearance and --clearance-ceiling alike and grades at the minimum")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
