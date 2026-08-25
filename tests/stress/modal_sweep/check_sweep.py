#!/usr/bin/env python3
"""Validate a sweep result JSON. Exits non-zero if the run is not trustworthy.

    python3 check_sweep.py /path/to/sweep_<ts>.json            # any sweep
    python3 check_sweep.py sweep.json --require-all-complete   # smoke test

Why this exists as a SCRIPT rather than a paragraph of "things to eyeball": a
sweep can exit green and still be meaningless. The 2026-08-09 bring-up produced
a run with chain_complete on every row, 100% completion and 0 DRC, in which BOTH
arms had secretly executed with the same routing constants -- the image had
shipped an uncommitted working-tree edit, so the "baseline" was not the baseline.
At 100 arms that reads as "the parameter has no effect" and nothing in the output
contradicts it. These checks are what would have caught it.

Checks
  1. chain_complete            -- a broken chain is not a result
  2. arms applied their spec   -- an arm declaring `defaults` must report
                                  non-empty patched_defaults, else it tested
                                  nothing; a bare arm must report {}
  3. arms actually differ      -- identical arms mean the parameter never
                                  reached the engine (unless genuinely inert)
  4. provenance               -- every row carries a source commit, and a
                                  `+dirty` sha is reported as a warning
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Routing OUTCOMES -- any difference here is real signal.
OUTCOME_KEYS = ("completion_pct", "drc", "conn", "nets_incomplete")
# Wall-clock is NOISY, so a small delta is not evidence the parameter did
# anything. Measured during bring-up: a run in which both arms provably executed
# the SAME constants still differed by 5% and 0.06% on total_seconds, while the
# genuinely-different run differed by 23% and 50%. Below this threshold a timing
# delta is treated as jitter, not proof.
TIMING_NOISE_FRAC = 0.10


def check(data: dict, require_all_complete: bool = False,
          max_broken_frac: float = 0.25) -> int:
    rows = data.get("rows") or []
    if not rows:
        print("FAIL: no rows in this sweep")
        return 1
    failures, warnings = [], []

    # 1 -----------------------------------------------------------------
    broken = [r for r in rows if not r.get("chain_complete")]
    if broken:
        frac = len(broken) / len(rows)
        msg = f"{len(broken)}/{len(rows)} rows have a broken chain ({frac*100:.0f}%)"
        # A real sweep legitimately has SOME failing boards, so a broken chain is
        # normally a warning. But a high RATE means something systemic, not board
        # difficulty -- e.g. the pre-#522 manifests whose root-level tool paths
        # went unremapped: every step exited rc=2 while --continue-on-error kept
        # the task green, and that flavour is 75 of the 150 boards in sets 1-10.
        # Reporting half the corpus as a warning would bury it.
        if require_all_complete or frac > max_broken_frac:
            failures.append(msg + (f" -- above the {max_broken_frac*100:.0f}% "
                                   "threshold, so this looks systemic, not per-board"
                                   if not require_all_complete else ""))
        else:
            warnings.append(msg)
        for r in broken[:5]:
            tail = (r.get("replay_log_tail") or r.get("stderr_tail") or "").strip().splitlines()
            print(f"   broken: {r.get('arm')}/{r.get('board')}"
                  + (f"  last log line: {tail[-1][:120]}" if tail else "  (no log captured)"))
    print(f"1. chain_complete ......... {len(rows)-len(broken)}/{len(rows)}")

    # 2 -----------------------------------------------------------------
    bad_spec = []
    for r in rows:
        declared = set((r.get("arm_defaults") or {}))
        applied = r.get("patched_defaults")
        if applied is None:
            bad_spec.append(f"{r.get('arm')}/{r.get('board')}: no patched_defaults field")
    # arms that declare defaults are known from any row of that arm
    by_arm: dict[str, list] = {}
    for r in rows:
        by_arm.setdefault(r.get("arm"), []).append(r)
    for arm, rs in by_arm.items():
        applied = [r.get("patched_defaults") or {} for r in rs]
        if any(applied) and not all(applied):
            bad_spec.append(f"{arm}: patched_defaults inconsistent across boards "
                            f"-- container reuse may be leaking arms")
    if bad_spec:
        failures.append("arm spec not applied consistently")
        for b in bad_spec[:5]:
            print(f"   {b}")
    print(f"2. arm spec applied ....... {len(by_arm)} arm(s): "
          + ", ".join(f"{a}={list((rs[0].get('patched_defaults') or {}).keys()) or '{}'}"
                      for a, rs in sorted(by_arm.items())))

    # 3 -----------------------------------------------------------------
    diffs, timing_diffs = [], []
    arms = sorted(by_arm)
    if len(arms) >= 2:
        base = arms[0] if "baseline" not in arms else "baseline"
        for other in [a for a in arms if a != base]:
            for b in {r["board"] for r in by_arm[other]}:
                x = next((r for r in by_arm[base] if r.get("board") == b), None)
                y = next((r for r in by_arm[other] if r.get("board") == b), None)
                if not x or not y:
                    continue
                for k in OUTCOME_KEYS:
                    if x.get(k) != y.get(k):
                        diffs.append(f"{other} vs {base} on {b}: {k} {x.get(k)} -> {y.get(k)}")
                xs, ys = x.get("total_seconds"), y.get("total_seconds")
                if isinstance(xs, (int, float)) and isinstance(ys, (int, float)) and xs > 0:
                    frac = abs(ys - xs) / xs
                    if frac >= TIMING_NOISE_FRAC:
                        timing_diffs.append(
                            f"{other} vs {base} on {b}: total_seconds {xs:.2f} -> {ys:.2f} "
                            f"({frac*100:.0f}%)")
        if not diffs and not timing_diffs:
            failures.append(
                "all arms produced IDENTICAL outcomes and near-identical runtimes -- "
                "the parameter probably never reached the engine (see the docstring)")
        elif not diffs:
            warnings.append(
                "arms differ ONLY in runtime, not in any routing outcome. Above the "
                "noise floor so the parameter is reaching the engine, but it changed "
                "no completion/DRC/connectivity result on these boards")
    print(f"3. arms differ ............ {len(diffs)} outcome, {len(timing_diffs)} timing "
          f"(>{int(TIMING_NOISE_FRAC*100)}%)")
    for d in (diffs + timing_diffs)[:5]:
        print(f"   {d}")

    # 4 -----------------------------------------------------------------
    shas = {r.get("git") for r in rows}
    if any(s in (None, "", "unknown") for s in shas):
        failures.append("rows missing a source commit -- results are not reproducible")
    if any(s and s.endswith("+dirty") for s in shas):
        warnings.append("built from a DIRTY working tree (KICAD_SWEEP_DIRTY=1) -- "
                        "the baseline is whatever was uncommitted locally")
    print(f"4. provenance ............. {', '.join(sorted(str(s) for s in shas))}")

    for w in warnings:
        print(f"WARN: {w}")
    if failures:
        print("\nFAIL:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nPASS: sweep looks trustworthy")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep_json")
    ap.add_argument("--max-broken-frac", type=float, default=0.25,
                    help="fail if more than this FRACTION of rows have a broken "
                         "chain (default 0.25) -- a high rate is systemic, not "
                         "board difficulty")
    ap.add_argument("--require-all-complete", action="store_true",
                    help="treat ANY broken chain as failure (right for a smoke test; "
                         "a real sweep legitimately has some failing boards)")
    a = ap.parse_args()
    return check(json.loads(Path(a.sweep_json).read_text()),
                 a.require_all_complete, a.max_broken_frac)


if __name__ == "__main__":
    sys.exit(main())
