#!/usr/bin/env python3
"""Local dose-range prescreen: the free stage before any cloud spend.

    python3 tests/stress/modal_sweep/prescreen.py HEURISTIC_WEIGHT 1.0 1.5 2.5 3.0
    python3 tests/stress/modal_sweep/prescreen.py --env KICAD_PROXIMITY_SUM 0 1 softcap

Runs two challenging-but-fast boards (baseline verdict >= 2 so movement
shows in BOTH directions -- a perfectly-routed board hides small damage and
cannot show improvement; override with --boards) at each candidate value of
ONE knob, and prints verdict / wall per value. Purpose: find the range
where a knob makes any sense AT ALL. Division of labor, measured on first
use: the prescreen catches BLOWUPS (quality collapse, wall explosion);
mild degradation is the cloud screen's job (hw3.0 tied 0-0 locally, died
across 12 cloud boards). The funnel:

  prescreen (2 boards, local, $0)  ->  screen (12 boards, ~$0.02/arm)
  ->  full corpus (~$0.7/arm)      ->  holdout (untouched sets)

PARALLEL up to --jobs (default 4): defaults-mode values each get their own
temporary git WORKTREE (routing_defaults is per-tree state; the compiled
grid_router.so is copied into each worktree's rust_router/ -- worktrees
don't inherit build artifacts). Env-mode values need no isolation. The
main tree is never patched, so this can run alongside other local work.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))


def pick_boards(n=2):
    bv = json.loads((HERE / "board_value.json").read_text())
    costs = {s["board"]: s for s in bv.get("table", [])}

    # Andy's criterion: challenging-but-fast -- baseline must NOT route the
    # board perfectly (verdict >= 2), cheap enough for minutes-scale local
    # runs, then highest spread. (Cloud verdicts can differ from local:
    # interpreter-version divergence -- the flat-baseline warning below
    # catches a pick that routes perfectly on THIS machine.)
    def ok(b):
        s = costs.get(b, {})
        return s.get("baseline_verdict", 0) >= 2 and s.get("cost_h", 9) <= 0.15
    pool = [b for b in bv.get("screen", []) if ok(b)]
    pool += sorted((s["board"] for s in bv.get("table", [])
                    if ok(s["board"]) and s["board"] not in pool),
                   key=lambda b: (-costs[b]["spread"], costs[b]["cost_h"]))
    pool.sort(key=lambda b: (-costs[b]["spread"], costs[b]["cost_h"]))
    return pool[:n]


_WORKER = r"""
import json, os, resource, sys
sys.path.insert(0, sys.argv[1] + "/tests/stress")
import ab_replay_grade as abg
from pathlib import Path
stress, set_dir, out, board, tag = (Path(sys.argv[2]), sys.argv[3],
                                    Path(sys.argv[4]), sys.argv[5], sys.argv[6])
if board == "ddr5_testbed":
    # its manifest bakes one output into the OLD runs_set2/ dir; without this
    # remap the clobber guard aborts the chain (same fix as wave_driver's)
    abg.EXTRA_REMAPS.append(f"{stress}/runs_set2/ddr5_testbed:{out / board}")
r = abg.do_board(stress / set_dir, out, tag, board)
# CPU of the whole reaped chain (redo + tools): wall time under a parallel
# pool is contamination; CPU is the honest per-value cost (Andy).
ru = resource.getrusage(resource.RUSAGE_CHILDREN)
r["cpu_s"] = round(ru.ru_utime + ru.ru_stime, 1)
(out / f"row_{board}.json").write_text(json.dumps(r))
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("knob")
    ap.add_argument("values", nargs="+")
    ap.add_argument("--env", action="store_true",
                    help="knob is a KICAD_* env var, not a routing_defaults constant")
    ap.add_argument("--boards", default="",
                    help="comma-separated; default: 2 challenging-but-fast boards")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--stress-dir",
                    default=os.path.expanduser("~/Documents/kicad_stress_test"))
    a = ap.parse_args()

    import sweep_lib as sl
    stress = Path(a.stress_dir)
    boards = ([b.strip() for b in a.boards.split(",") if b.strip()]
              or pick_boards())
    locs = {b: s for s, b in sl.discover_boards(
        stress, [f"set{i}" for i in range(1, 26)]) if b in set(boards)}
    missing = set(boards) - set(locs)
    if missing:
        raise SystemExit(f"boards not found in corpus: {sorted(missing)}")

    def parse(v):
        try:
            return int(v)
        except ValueError:
            try:
                return float(v)
            except ValueError:
                return v

    values = [None] + [parse(v) for v in a.values]   # None = default
    tags = {v: ("default" if v is None else str(v).replace("/", "_"))
            for v in values}
    out_root = Path(tempfile.mkdtemp(prefix="kicad_prescreen_"))

    # One worktree per defaults-mode non-default value; env-mode and the
    # default value run from the main repo untouched.
    trees, made = {}, []
    for v in values:
        if v is None or a.env:
            trees[v] = REPO
            continue
        wt = out_root / f"wt_{tags[v]}"
        subprocess.run(["git", "-C", str(REPO), "worktree", "add", "--detach",
                        str(wt), "HEAD"], check=True, capture_output=True)
        made.append(wt)
        # worktrees don't inherit build artifacts (memory: grid_router.so
        # belongs in rust_router/, not the root)
        shutil.copy(REPO / "rust_router" / "grid_router.so",
                    wt / "rust_router" / "grid_router.so")
        sl.patch_routing_defaults(wt, {a.knob: v})
        trees[v] = wt

    # Workers run in their OWN process group and are tracked, so killing the
    # prescreen kills the whole tree. Learned the hard way: pkill of the
    # parent left 4 orphaned worker chains (redo + route, no "prescreen" in
    # their cmdlines) burning CPU into an abandoned tmp dir alongside the
    # next run's legitimate 4 (Andy counted 8).
    import atexit
    import signal
    live = set()

    def _reap(*_):
        for proc in list(live):
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if _:
            sys.exit(143)

    atexit.register(_reap)
    signal.signal(signal.SIGTERM, _reap)
    signal.signal(signal.SIGINT, _reap)

    def run_one(v, board):
        env = dict(os.environ)
        if a.env and v is not None:
            env[a.knob] = str(v)
        out = out_root / f"res_{tags[v]}"
        out.mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen([sys.executable, "-c", _WORKER, str(trees[v]),
                                 str(stress), f"runs_{locs[board]}", str(out),
                                 board, f"pre:{tags[v]}"], env=env,
                                start_new_session=True,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        live.add(proc)
        try:
            proc.wait()
        finally:
            live.discard(proc)
        p = out / f"row_{board}.json"
        return json.loads(p.read_text()) if p.exists() else {}

    print(f"prescreen {a.knob} on {boards} ({a.jobs} parallel)")
    results = {}
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as ex:
            futs = {ex.submit(run_one, v, b): (v, b)
                    for v in values for b in boards}
            for f in concurrent.futures.as_completed(futs):
                v, b = futs[f]
                results[(v, b)] = f.result()
                r = results[(v, b)]
                print(f"  [{tags[v]}] {b}: "
                      f"{'ok' if r.get('chain_complete') else 'BROKEN'} "
                      f"compl={r.get('completion_pct')} t={r.get('total_seconds')}s",
                      flush=True)
    finally:
        for wt in made:
            subprocess.run(["git", "-C", str(REPO), "worktree", "remove",
                            "--force", str(wt)], capture_output=True)

    rows = []
    for v in values:
        verd, wall, cpu, peak, ok = 0, 0, 0.0, 0.0, True
        for b in boards:
            r = results.get((v, b), {})
            if not r.get("chain_complete"):
                ok = False
                continue
            verd += (r.get("nets_incomplete") or 0) + (r.get("conn") or 0)
            wall += r.get("total_seconds") or 0
            cpu += r.get("cpu_s") or 0
            peak = max(peak, r.get("peak_rss_mb") or 0)
        rows.append((tags[v], verd if ok else None, cpu, peak, wall, ok))

    if rows[0][5] and rows[0][1] is not None and rows[0][1] < 2:
        print(f"\nWARN: boards route near-perfectly at the DEFAULT locally "
              f"(verdict {rows[0][1]}) -- damage and improvement both "
              "invisible; pick more challenging --boards.")
    print(f"\n{'value':<12}{'verdict':>8}{'cpu_s':>8}{'peak_mb':>9}{'wall_s':>8}")
    best = min((r[1] for r in rows if r[1] is not None), default=0)
    base_cpu, base_peak = rows[0][2], rows[0][3]
    for tag, verd, cpu, peak, wall, ok in rows:
        # Viability (Andy): >50% over the DEFAULT's measured CPU or peak RSS
        # disqualifies a value regardless of quality -- resources are part of
        # the default contract. CPU not wall (pool contention); peak is the
        # tree-sampled max across boards.
        flags = []
        if not ok:
            flags.append("CHAIN BROKEN")
        if verd is not None and verd > best + 3:
            flags.append("quality: outside sensible range?")
        if base_cpu and cpu > 1.5 * base_cpu:
            flags.append(f"NOT VIABLE cpu +{100*(cpu/base_cpu-1):.0f}%")
        # Mem threshold 2x, not 1.5x: single-run peak RSS is phase-noise
        # sensitive (non-monotone across doses -- turn-cost 300 flagged while
        # 10000 passed), so the 50-100% band is unreliable; CPU stays at 1.5x
        # (smooth and reproducible across the battery).
        if base_peak and peak > 2.0 * base_peak:
            flags.append(f"NOT VIABLE mem +{100*(peak/base_peak-1):.0f}%")
        sfx = ("  <- " + "; ".join(flags)) if flags else ""
        print(f"{tag:<12}{str(verd):>8}{cpu:>8.0f}{peak:>9.0f}{wall:>8.0f}{sfx}")
    print("\nPick 2-3 values from the sensible range for the cloud screen.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
