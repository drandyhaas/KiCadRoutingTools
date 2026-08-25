#!/usr/bin/env python3
"""Multi-set A/B wave driver: keep N boards running at ALL times, ACROSS sets.

`ab_replay_grade.py --set runs_setN` parallelises WITHIN one set, so every set's
tail drains to idle (often 1 slow board holding 4 free cores) before the next set
starts. Over 11 sets that idle time dominates. This driver builds ONE global queue
over every set and keeps the pool saturated across set boundaries.

It calls `ab_replay_grade.do_board` per board, so grading semantics, the
per-board `--clearance`, the #405 baseline subtraction and the `summary.json`
schema are all IDENTICAL to a plain `ab_replay_grade` wave -- the per-set
`summary.json` it writes is a drop-in input to `ab_replay_grade.py --compare`
and to `ab_wave_report.py`.

    # candidate wave over sets 1-11, 5 boards in flight at all times
    nohup caffeinate -s python3 -u tests/stress/ab_wave_driver.py wave \
        --out ~/Documents/kicad_stress_test/ab_main_0728a --label head --jobs 5 \
        > ~/wave.log 2>&1 &

    # re-grade an EXISTING wave with today's grader (no re-routing)
    python3 tests/stress/ab_wave_driver.py regrade \
        --out ~/Documents/kicad_stress_test/ab_main_0726a --label base --jobs 5

===============================================================================
OPERATIONAL NOTES -- read before starting a wave
===============================================================================

*   **Detach it: `nohup ... &`.** A corpus wave runs for hours. Started in the
    foreground it dies with the terminal / the agent session. Verify it actually
    detached (parent should be init):
        ps -eo pid,ppid,etime,command | grep ab_wave_driver

*   **`caffeinate -s`.** macOS sleeping mid-wave suspends every worker and
    corrupts the timing columns (and on battery can kill the run outright).
    Wrap the whole driver, not each board.

*   **Attach a Monitor** on the log so failures surface instead of being
    discovered hours later. Filter for BOTH progress and every failure shape --
    a monitor that greps only the happy path is silent through a crashloop:
        tail -f wave.log | grep -E --line-buffered \\
          "chain=BROKEN|NORESULT|ALL DONE|Traceback|Killed|MemoryError|PROGRESS"

*   **FREEZE the working tree for the whole wave.** Manifests bake TOOL paths
    absolutely, so a replay always runs whatever is checked out right now. Edit a
    routing module mid-wave and early boards ran different code than late ones --
    silently, with no marker in the output. Adding NEW files is safe; changing
    existing ones is not. For the same reason two waves must run SEQUENTIALLY
    (they share git state), while boards within a wave run in parallel.

*   **Regrade the baseline; never diff against stored/published numbers.** Every
    grader here (check_drc, the fill-aware connectivity checker, the netclass
    reconciliation) is under active development, so an old `summary.json` is a
    snapshot of code that no longer exists. Re-running the OLD wave's copper
    through TODAY's grader has moved rows in both directions and once turned a
    real -24 into an apparent -72. That is what `regrade` mode is for. It
    REWRITES the wave's `summary.json` in place, so back the original up first:
        cp setN/summary.json setN/summary_orig.json
    Cheap validation: if regrading reproduces well under ~100% of the stored
    rows, the stored table was never a usable baseline.

*   **Wave dirs are write-once.** Never re-run a wave into an existing dir: the
    sibling `.kicad_pro` DRC floor written by a prior run gets read back and
    silently changes the routing (looks like non-determinism; it isn't). Name
    them `ab_<what>_MMDD` with a same-day `a`/`b`/`c` suffix.

*   **Outputs carry random UUIDs.** Never hash or whole-file-diff `.kicad_pcb`
    outputs to judge determinism -- compare the graded counts instead.

===============================================================================
SCHEDULING
===============================================================================

*   **Order: `--order name` is the DEFAULT** -- plain corpus order, by set then
    board name (numeric sets sort 9 before 10). This is the reproducible one and
    what you want almost always: the Nth board of a wave is the same board every
    run, two waves' logs interleave comparably, and a wave killed half way
    through covers a predictable prefix instead of an arbitrary subset. Nothing
    about grading depends on order -- routing is deterministic and each board is
    independent -- so this costs only wall-clock, never correctness.

*   **`--order lpt`** is the opt-in alternative: longest-processing-time-first
    from a prior wave's `total_seconds` (`--cost-baseline`). It shortens the
    makespan, because the corpus's multi-hour board starts at t=0 instead of
    stranding 4 idle cores at the end. The cost is that run order becomes
    data-dependent: it changes whenever the cost baseline changes, so two waves
    are no longer directly comparable board-for-board. Reach for it only when
    wall-clock genuinely matters.

*   **Memory admission is OFF by default** (`--heavy-slots 0` = no limit), so
    the chosen order is honoured strictly. Set `--heavy-slots 1` on a small-RAM
    box: it caps how many boards whose prior peak exceeds `--heavy-mb` run at
    once. The corpus has ~11 boards over 2.5GB and one peaking at 6.6GB, and
    several of those together on an 8GB machine thrash swap and get workers
    OOM-killed -- surfacing as NORESULT rows hours in. When enabled, a blocked
    heavy board does NOT hold a worker slot (the scheduler starts the next
    ELIGIBLE board), so `--jobs` stay in flight -- but that IS a reordering, so
    it trades some of the reproducibility above for safety.

*   **A board never kills the wave**: a worker exception degrades to a NORESULT
    row. Per-set `summary.json` is flushed after every completion, so killing
    the driver still leaves every finished board graded on disk.
"""
import argparse
import concurrent.futures
import json
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent  # tests/stress/ -> repo root
sys.path.insert(0, str(REPO / "tests/stress"))

import ab_replay_grade as A  # noqa: E402
from _gitver import write_git_version, format_version  # noqa: E402

DEFAULT_CORPUS = Path("~/Documents/kicad_stress_test").expanduser()
FALLBACK_COST = (300.0, 500.0)  # (seconds, peak_mb) for a board with no prior wave


def load_costs(baseline_dir):
    """board -> (seconds, peak_mb) from a prior wave, for LPT + memory admission."""
    costs = {}
    if not baseline_dir:
        return costs
    for f in sorted(Path(baseline_dir).expanduser().glob("set*/summary.json")):
        for r in json.loads(f.read_text()):
            costs[r["board"]] = (r.get("total_seconds") or 0.0,
                                 r.get("peak_footprint_mb") or r.get("peak_rss_mb") or 0.0)
    return costs


def regrade_board(set_dir, out_dir, label, board):
    """Per-board equivalent of ab_replay_grade.regrade's loop body (which is
    serial); split out so the same global pool can parallelise a regrade."""
    bdir = Path(out_dir) / board
    txt = (Path(set_dir) / board / "redo_commands.sh").read_text()
    clr = A.route_clearance(txt)
    fname = A.final_output_name(txt)
    final = bdir / fname if fname else None
    done = bool(final) and final.exists()
    lp = bdir / "_replay.log"
    log_text = lp.read_text(errors="replace") if lp.exists() else ""
    res = {"board": board, "clearance": clr, "replay_rc": 0,
           "final": fname if done else None, "chain_complete": done,
           "drc": None, "conn": None, "nets_total": None, "nets_incomplete": None,
           "completion_pct": None, **A._diff_pair_stats(log_text)}
    if done:
        # Same #405 symmetric baseline resolution as ab_replay_grade.regrade.
        baseline = A.manifest_baseline(txt, bdir, Path(set_dir) / board)
        res.update(A.grade(str(final), clr, baseline=baseline))
    print(f"[{label}] {board}: chain={'ok' if done else 'BROKEN'} drc={res['drc']} "
          f"kdrc={res.get('kicad_drc')} conn={res['conn']} compl={res['completion_pct']}%",
          flush=True)
    return res


def _set_key(s):
    """Numeric set order where possible ('9' before '10'), name order otherwise
    (so 'set1monster' still sorts deterministically)."""
    return (0, int(s), "") if s.isdigit() else (1, 0, s)


def build_tasks(mode, corpus, out_root, sets, costs, heavy_mb, order="name"):
    """Global task list across every set.

    order='name' (DEFAULT): plain corpus order -- by set, then board name. This
    is the reproducible one: the Nth board of a wave is the same board every
    run, two waves interleave their logs comparably, and a partial wave covers a
    predictable prefix. Prefer it.

    order='lpt': longest-processing-time-first from --cost-baseline. Shorter
    wall-clock (the multi-hour board starts at t=0 instead of stranding idle
    cores at the end) but the run order is data-dependent, so it changes
    whenever the cost baseline changes.
    """
    tasks = []
    for s in sets:
        set_dir = corpus / f"runs_set{s}"
        out_dir = out_root / f"set{s}"
        if not set_dir.is_dir():
            print(f"[warn] no {set_dir}; skipping set{s}", flush=True)
            continue
        if mode == "wave":
            boards = sorted(d.name for d in set_dir.iterdir()
                            if not d.name.startswith(".") and (d / "redo_commands.sh").exists())
            out_dir.mkdir(parents=True, exist_ok=True)
        else:
            if not out_dir.is_dir():
                print(f"[warn] no {out_dir} to regrade; skipping set{s}", flush=True)
                continue
            # Regrade only boards that still have a manifest (skips headless-worker
            # artifacts and any stray dir).
            boards = sorted(d.name for d in out_dir.iterdir()
                            if d.is_dir() and not d.name.startswith(".")
                            and (set_dir / d.name / "redo_commands.sh").exists())
        for b in boards:
            sec, mb = costs.get(b, FALLBACK_COST)
            tasks.append({"set": s, "board": b, "set_dir": set_dir, "out_dir": out_dir,
                          "sec": sec, "mb": mb, "heavy": mb > heavy_mb})
    if order == "lpt":
        tasks.sort(key=lambda t: -t["sec"])          # longest-processing-time-first
    else:
        tasks.sort(key=lambda t: (_set_key(t["set"]), t["board"]))   # corpus order
    return tasks


def main():
    ap = argparse.ArgumentParser(
        description="Global-queue corpus wave driver (keeps --jobs boards in flight across sets)")
    ap.add_argument("mode", choices=["wave", "regrade"],
                    help="wave = replay+grade into --out; regrade = re-grade --out in place")
    ap.add_argument("--out", required=True, help="wave dir (created by wave, rewritten by regrade)")
    ap.add_argument("--corpus", default=str(DEFAULT_CORPUS), help="dir holding runs_setN/")
    ap.add_argument("--sets", default="1,2,3,4,5,6,7,8,9,10,11")
    ap.add_argument("--label", default="head", help="provenance label recorded in git_version")
    ap.add_argument("--jobs", type=int, default=5, help="boards in flight at all times")
    ap.add_argument("--order", choices=("name", "lpt"), default="name",
                    help="board order: 'name' (DEFAULT) = by set then board name, "
                         "reproducible; 'lpt' = longest-first, shorter wall-clock "
                         "but data-dependent")
    ap.add_argument("--heavy-mb", type=float, default=2500.0,
                    help="a board whose prior peak exceeds this is 'heavy' "
                         "(only used with --heavy-slots)")
    ap.add_argument("--heavy-slots", type=int, default=4,
                    help="max heavy boards concurrent (DEFAULT 4); 0 = no limit. "
                         "A LOW value serialises the tail: a corpus wave ends with "
                         "its heavy boards, so --heavy-slots 1 drains them one at a "
                         "time and the last 6 boards can take longer than the first "
                         "159. 4 keeps the pool busy while still capping peak RAM")
    ap.add_argument("--cost-baseline", default=None,
                    help="prior wave dir supplying per-board time/memory for scheduling")
    ap.add_argument("--extra-remap", action="append", default=[], metavar="OLD:NEW",
                    help="extra redo_stress_test --remap (e.g. to point a wave at a worktree)")
    args = ap.parse_args()

    corpus = Path(args.corpus).expanduser()
    out_root = Path(args.out).expanduser()
    sets = [s.strip() for s in args.sets.split(",") if s.strip()]
    A.EXTRA_REMAPS = list(args.extra_remap)  # honoured by ab_replay_grade.do_board

    costs = load_costs(args.cost_baseline)
    tasks = build_tasks(args.mode, corpus, out_root, sets, costs, args.heavy_mb,
                        order=args.order)
    if not tasks:
        sys.exit("nothing to do")

    if args.mode == "wave":
        ver = None
        for s in sets:
            d = out_root / f"set{s}"
            if d.is_dir():
                ver = write_git_version(d, REPO, label=args.label)
        if ver:
            print(f"[{args.label}] code under test: {format_version(ver)} "
                  f"(commit {ver.get('commit')})", flush=True)
    _adm = (f"<={args.heavy_slots} heavy(>{args.heavy_mb:.0f}MB) concurrent"
            if args.heavy_slots else "no memory admission")
    print(f"[{args.label}] {args.mode}: {len(tasks)} boards over sets {','.join(sets)}; "
          f"{args.jobs} in flight, order={args.order}, {_adm}", flush=True)
    if costs:
        est = sum(t["sec"] for t in tasks)
        print(f"[{args.label}] est serial cpu {est/3600:.1f}h -> ~{est/3600/args.jobs:.1f}h "
              f"at {args.jobs}x (from --cost-baseline)", flush=True)

    lock = threading.Lock()
    results = {s: [] for s in sets}

    def flush(s):
        d = out_root / f"set{s}"
        if d.is_dir():
            (d / "summary.json").write_text(
                json.dumps(sorted(results[s], key=lambda r: r["board"]), indent=2))

    def run_one(t):
        lbl = f"{args.label}/set{t['set']}"
        if args.mode == "wave":
            return A.do_board(t["set_dir"], t["out_dir"], lbl, t["board"])
        return regrade_board(t["set_dir"], t["out_dir"], lbl, t["board"])

    # ---- scheduler: keep --jobs in flight, honouring heavy admission --------
    t0 = time.time()
    pending, running, heavy_running, done_n = list(tasks), {}, 0, 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        while pending or running:
            while len(running) < args.jobs:
                pick = None
                for i, t in enumerate(pending):
                    if (t["heavy"] and args.heavy_slots
                            and heavy_running >= args.heavy_slots):
                        continue  # skip it -- a lighter board takes the slot instead
                    pick = pending.pop(i)
                    break
                if pick is None:
                    break  # only heavy boards left and the heavy slot is busy
                if pick["heavy"]:
                    heavy_running += 1
                running[ex.submit(run_one, pick)] = pick

            done, _ = concurrent.futures.wait(
                running, return_when=concurrent.futures.FIRST_COMPLETED)
            for f in done:
                t = running.pop(f)
                if t["heavy"]:
                    heavy_running -= 1
                try:
                    res = f.result()
                except BaseException as e:  # noqa: BLE001 -- one board never kills the wave
                    print(f"[{args.label}/set{t['set']}] {t['board']}: NORESULT "
                          f"({type(e).__name__}: {e})", flush=True)
                    res = {"board": t["board"], "clearance": None, "replay_rc": None,
                           "final": None, "chain_complete": False, "drc": None, "conn": None,
                           "nets_total": None, "nets_incomplete": None,
                           "completion_pct": None, "error": f"{type(e).__name__}: {e}"}
                with lock:
                    results[t["set"]].append(res)
                    flush(t["set"])  # stream to disk: a mid-run death keeps finished grades
                    done_n += 1
                print(f"[PROGRESS] {done_n}/{len(tasks)} done, {len(running)} in flight, "
                      f"{len(pending)} queued, {(time.time()-t0)/3600:.2f}h elapsed", flush=True)

    for s in sets:
        flush(s)
        if results[s]:
            ok = sum(1 for r in results[s] if r["chain_complete"])
            print(f"[{args.label}] set{s}: {ok}/{len(results[s])} chains complete", flush=True)
    print(f"[{args.label}] ALL DONE in {(time.time()-t0)/3600:.2f}h", flush=True)


if __name__ == "__main__":
    main()
