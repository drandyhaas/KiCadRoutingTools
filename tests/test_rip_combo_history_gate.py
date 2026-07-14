#!/usr/bin/env python3
"""Issue #376 (fix 3): shared rip-up history gate for the two reroute ladders.

single_ended_loop and reroute_loop both gate the progressive rip-up ladder on
rip_and_retry_history. The reroute twin previously did a bare, silent
``continue`` (no log) at one site and a divergently-worded log at another -- a
#315-class forked-check drift. rip_combo_already_tried centralizes the decision
AND its "Skipping N=..." message so the loops can't diverge; it returns the
canonical blocker frozenset so the check key and the later add key stay in
lockstep.

    python3 tests/test_rip_combo_history_gate.py
"""
import io
import os
import sys
from contextlib import redirect_stdout
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from polarity_swap import rip_combo_already_tried


def _blocker(net_id, name):
    return NS(net_id=net_id, net_name=name)


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    # Two single-net blockers; a diff-pair blocker maps to its P net canonical.
    b_sig = _blocker(10, "/SIG")
    b_pair_n = _blocker(21, "/DP_N")            # N side of a pair whose P is 20
    diff_pair_by_net_id = {20: ("DP", NS(p_net_id=20, n_net_id=21)),
                           21: ("DP", NS(p_net_id=20, n_net_id=21))}
    blockers = [b_sig, b_pair_n]

    # --- not in history: (False, canonicals); canonicals fold the pair to P ---
    hist = set()
    out = io.StringIO()
    with redirect_stdout(out):
        tried, canon = rip_combo_already_tried(hist, 5, "/NET", blockers, 2,
                                               diff_pair_by_net_id)
    check("fresh combo -> not tried", tried is False)
    check("canonicals fold diff-pair N to P (20, not 21)", canon == frozenset({10, 20}))
    check("no log when not tried", out.getvalue() == "")

    # --- key = (history_net_id, canonicals); present -> (True, canonicals)+log
    hist.add((5, canon))
    out = io.StringIO()
    with redirect_stdout(out):
        tried2, canon2 = rip_combo_already_tried(hist, 5, "/NET", blockers, 2,
                                                 diff_pair_by_net_id)
    check("recorded combo -> tried", tried2 is True)
    check("returns the same canonical set", canon2 == canon)
    log = out.getvalue()
    check("logs the shared 'Skipping N=' message", "Skipping N=2: already tried ripping" in log)
    check("log names the diff pair by its pair name, not net", "DP" in log and "/NET" in log)
    check("multi-blocker log is brace-wrapped", "{/SIG, DP}" in log)

    # --- same combo but a DIFFERENT history net id is independent ---
    out = io.StringIO()
    with redirect_stdout(out):
        tried3, _ = rip_combo_already_tried(hist, 99, "/OTHER", blockers, 2,
                                            diff_pair_by_net_id)
    check("different history net id -> not tried", tried3 is False)

    # --- N=1 single blocker: bare name, no braces; log=False suppresses print
    out = io.StringIO()
    with redirect_stdout(out):
        t1, c1 = rip_combo_already_tried(hist, 5, "/NET", blockers, 1,
                                         diff_pair_by_net_id)  # only b_sig
    check("N=1 fresh combo not tried (subset key differs)", t1 is False and c1 == frozenset({10}))
    hist.add((5, c1))
    out = io.StringIO()
    with redirect_stdout(out):
        t1b, _ = rip_combo_already_tried(hist, 5, "/NET", blockers, 1,
                                         diff_pair_by_net_id, log=False)
    check("N=1 recorded -> tried, but log=False prints nothing",
          t1b is True and out.getvalue() == "")

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  shared gate: canonical folding, (net,combo) keying, unified")
    print("        'Skipping N=' log, log=False suppression")
    return 0


if __name__ == "__main__":
    sys.exit(run())
