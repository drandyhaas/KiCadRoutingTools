#!/usr/bin/env python3
"""Replay whole stress-test SETS in the cloud, keep the routed boards, and A/B
them against an existing baseline wave.

    python3 tests/stress/cloud_replay_sets.py --sets set10-set19

`ab_replay_grade.py` replays a set locally and grades it; `modal_sweep/` fans
(arm x board) tasks onto rented cores. This driver is the missing third thing:
a NO-PARAMETER-CHANGE replay of the current code across many sets, which keeps
the finished `.kicad_pcb`/`.kicad_pro` and then compares to a baseline. One arm,
no overrides -- the question is not "what does this knob do" but "what does the
CURRENT engine do to these 150 boards, versus the last time we looked".

Six stages, run in order by default; pick with --only:

  plan      discover boards, price the run, check preflight. SPENDS NOTHING.
  upload    push any sets the corpus volume is missing (idempotent)
  run       the cloud replay, artifact-keeping ON
  harvest   pull rows + boards into a local wave dir shaped like an
            ab_replay_grade wave, so --compare AND --regrade both work on it
  baseline  build the baseline from the ORIGINAL recorded runs, re-graded today
  compare   per-set + aggregate A/B

Three things this is careful about, all of which have burned this repo before:

1. **The kept board travels with its `.kicad_pro`.** The project carries the DRC
   floor the chain actually routed to. A bare `.kicad_pcb` re-grades against the
   stock netclass and manufactures phantom sub-floor violations on correct copper
   (#441). The container-side copy in modal_app.py takes every sibling
   (.kicad_pro/.kicad_dru/.kicad_prl) plus the manifest.

2. **The baseline is the recorded run itself, not an archived wave.** The corpus
   gets RE-RECORDED. Sets 10-19 were, after the #562 pours-first reshape: the
   2026-07-28 `ab_main_0728a` wave replayed a `route -> planes -> repair` chain,
   while today's manifests run `planes -> route` with the repair absorbed --
   all 150 boards produce a different final output than that wave recorded. A
   board-for-board diff against it therefore mixes a different recorded PLAN in
   with the engine delta and cannot separate the two. The recorded run dir holds
   the very boards the manifest we are replaying produced, so `--baseline
   recorded` (the default) compares like with like. Pointing --baseline at a wave
   dir still works, but preflight refuses it when the chains disagree unless you
   pass --allow-chain-mismatch.

3. **Graders drift, so the baseline is re-graded, never read off an old table.**
   `--baseline recorded` re-scores the recorded boards with TODAY's grader, which
   is what makes a delta attributable to the engine. The recorded runs are
   symlinked into a scratch wave rather than copied or graded in place -- the
   corpus is not ours to rewrite, and regrade() overwrites summary.json in
   whatever directory it is given.

The arm name carries the source commit, so resuming re-uses banked rows only
within one commit; changing the code forces a real re-run instead of silently
mixing two engines into one table.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SWEEP = HERE / "modal_sweep"
sys.path.insert(0, str(SWEEP))
sys.path.insert(0, str(HERE))

DEFAULT_STRESS = Path(os.environ.get("STRESS_DIR",
                                     os.path.expanduser("~/Documents/kicad_stress_test")))
RESULTS_VOLUME = "kicad-sweep-results"
CORPUS_VOLUME = "kicad-corpus"
# Modal's published core-second price; used only to print an estimate.
USD_PER_CORE_SEC = 0.0000131


# ----------------------------------------------------------------- utilities

def sh(cmd, **kw):
    """Run a command, streaming its output. Returns the CompletedProcess."""
    print(f"  $ {' '.join(str(c) for c in cmd)}", flush=True)
    return subprocess.run([str(c) for c in cmd], **kw)


# Modal rate-limits app CREATION. Launching several arms close together -- which
# every A/B and every bisect ladder does -- gets some of them rejected outright,
# and the run then exits non-zero while BACKGROUNDED, so a half-launched batch
# looks exactly like a healthy one until you notice the missing apps. Six arms
# launched ~2s apart lost three that way; a later batch of six lost three more.
RATE_LIMIT_MARK = "rate limit exceeded"
RATE_LIMIT_TRIES = 4


def sh_capture(cmd, **kw):
    """Stream a command's output AND return it, so the caller can inspect it.

    sh() streams but discards; capture_output would hide a 30-minute run's
    progress. This does both.
    """
    print(f"  $ {' '.join(str(c) for c in cmd)}", flush=True)
    p = subprocess.Popen([str(c) for c in cmd], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, **kw)
    out = []
    for line in p.stdout:
        print(line, end="", flush=True)
        out.append(line)
    p.wait()
    return p.returncode, "".join(out)


def _num(v: str):
    """int, else float, else the raw string -- the order matters (see stage_run)."""
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        return v


def expand_sets(spec: str) -> list:
    """'set10-set19' / 'set10,set12' / 'set10-set12,set19' -> [set10, ...]."""
    out = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            ai = int(a.strip().removeprefix("set"))
            bi = int(b.strip().removeprefix("set"))
            if bi < ai:
                raise SystemExit(f"bad range {part!r}: {b} precedes {a}")
            out += [f"set{i}" for i in range(ai, bi + 1)]
        else:
            out.append(part)
    seen, uniq = set(), []
    for s in out:                       # order-preserving dedupe
        if s not in seen:
            seen.add(s)
            uniq.append(s)
    return uniq


def git_sha(short=True) -> str:
    try:
        r = subprocess.run(["git", "-C", str(REPO), "rev-parse",
                            "--short" if short else "HEAD", "HEAD"],
                           capture_output=True, text=True)
        sha = r.stdout.strip().split("\n")[0]
    except Exception:
        return "unknown"
    return sha or "unknown"


def tree_dirty() -> list:
    """Modified TRACKED files. Untracked (`??`) does not make a run
    unreproducible, and treating it as dirty cries wolf -- a stress worker
    leaving logs in the repo root flagged a clean baseline once."""
    r = subprocess.run(["git", "-C", str(REPO), "status", "--porcelain"],
                       capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln and not ln.startswith("??")]


def wave_provenance(summary_path: Path) -> dict:
    """The git_version.json sitting beside (or one level above) a summary."""
    for cand in (summary_path.parent / "git_version.json",
                 summary_path.parent.parent / "git_version.json"):
        if cand.exists():
            try:
                return json.loads(cand.read_text())
            except Exception:
                pass
    return {}


# --------------------------------------------------------------------- plan

def discover(stress: Path, sets: list):
    import sweep_lib as sl
    boards = sl.discover_boards(stress, sets)
    costs = sl.load_cost_table(stress)
    return boards, costs


def stage_plan(args, sets, stress) -> dict:
    import sweep_lib as sl
    boards, costs = discover(stress, sets)
    if not boards:
        raise SystemExit(f"no boards found for {sets} under {stress}")

    per_set = {}
    for s, b in boards:
        per_set.setdefault(s, []).append(b)

    # Price it. LLM-sourced timings measure a whole agent session (exploration
    # and retries included) and badly overestimate a replay, so they are shown
    # separately rather than folded into one confident number.
    replay_s = llm_s = 0.0
    llm_boards, longest = [], (0.0, "")
    for s, b in boards:
        e = costs.get(b, {})
        sec = e.get("seconds", 0.0) or 0.0
        if e.get("src") == "replay":
            replay_s += sec
            if sec > longest[0]:
                longest = (sec, b)
        else:
            llm_s += sec
            llm_boards.append(b)

    est_s = replay_s + 0.68 * llm_s      # README's measured replay:LLM ratio
    print(f"\n=== PLAN: {len(boards)} boards across {len(sets)} set(s) ===")
    for s in sets:
        print(f"  {s:9} {len(per_set.get(s, [])):3} boards")
    print(f"\n  estimated CPU time   {est_s/3600:6.1f} CPU-hours "
          f"({replay_s/3600:.1f} measured + {llm_s/3600:.1f} LLM-priced x0.68)")
    print(f"  estimated CPU cost   ${est_s * USD_PER_CORE_SEC:6.2f}  "
          f"(cores are cheap here; concurrency quota is the real limit)")
    print(f"  wall-clock floor     {longest[0]/60:6.1f} min  ({longest[1]}) "
          f"-- the longest single board, measured")
    if llm_boards:
        print(f"  NOTE {len(llm_boards)} board(s) priced from LLM sessions, which "
              f"overestimate a replay: {', '.join(sorted(llm_boards)[:6])}"
              f"{' ...' if len(llm_boards) > 6 else ''}")
    peak = max((costs.get(b, {}).get("peak_mb") or 0) for _, b in boards)
    print(f"  heaviest board       {peak/1024:6.1f} GB peak -- containers BURST above their "
          f"reservation, so no board needs its own tier")

    # Preflight -- every failure here is cheaper to hit now than after upload.
    problems, warnings = [], []
    for s in sets:
        if not sl.run_dirs_for(stress, s):
            problems.append(f"{s}: no runs dir under {stress}")
    if not args.no_baseline and args.baseline == "recorded":
        # Baseline is built from the recorded runs later; check it CAN be.
        from ab_replay_grade import final_output_name
        no_final = 0
        for s in sets:
            for rd in sl.run_dirs_for(stress, s)[:1]:
                for bdir in sorted(p for p in rd.iterdir() if p.is_dir()):
                    man = bdir / "redo_commands.sh"
                    if not man.exists():
                        continue
                    fn = final_output_name(man.read_text())
                    if not (fn and (bdir / fn).exists()):
                        no_final += 1
        print(f"  baseline: the recorded runs themselves, re-graded with today's grader")
        if no_final:
            warnings.append(f"{no_final} recorded board(s) have no final output "
                            f"(their chain never completed when recorded); they will "
                            f"be excluded from the paired comparison")
    elif not args.no_baseline:
        for s in sets:
            bp = baseline_summary(args, s)
            if not bp.exists():
                problems.append(f"{s}: baseline summary missing ({bp})")
            else:
                prov = wave_provenance(bp)
                if prov.get("dirty"):
                    msg = (f"{s}: baseline {prov.get('commit','?')[:9]} was built from a "
                           f"DIRTY tree -- its numbers are not reproducible from git")
                    (problems if not args.allow_dirty_baseline else warnings).append(msg)
        # A wave baseline is only comparable if it replayed the SAME chain. The
        # corpus gets re-recorded, and then a board-for-board diff silently mixes
        # a different recorded PLAN into the engine delta. Detect it by the final
        # output name, which changes when the chain is reshaped.
        from ab_replay_grade import final_output_name
        changed = tot = 0
        for s in sets:
            bp = baseline_summary(args, s)
            rds = sl.run_dirs_for(stress, s)
            if not (bp.exists() and rds):
                continue
            for r in json.loads(bp.read_text()):
                man = rds[0] / str(r.get("board")) / "redo_commands.sh"
                if not man.exists():
                    continue
                tot += 1
                cur = final_output_name(man.read_text())
                if not (r.get("final") and cur
                        and Path(str(r["final"])).name == Path(cur).name):
                    changed += 1
        if tot and changed:
            msg = (f"{changed}/{tot} baseline board(s) recorded a DIFFERENT final "
                   f"output than today's manifest produces -- that wave replayed a "
                   f"different chain, so a diff against it is NOT a pure code delta. "
                   f"Prefer --baseline recorded.")
            (warnings if changed < tot or args.allow_chain_mismatch
             else problems).append(msg)
            if changed == tot and not args.allow_chain_mismatch:
                problems.append("every board's chain differs; pass --allow-chain-mismatch "
                                "if you really want this comparison")
    dirty = tree_dirty()
    if dirty:
        warnings.append(f"working tree has {len(dirty)} modified tracked file(s); "
                        f"the cloud image ships a CLEAN HEAD unless KICAD_SWEEP_DIRTY=1, "
                        f"so the run may not test what you have locally")
    for w in warnings:
        print(f"  WARN  {w}")
    for p in problems:
        print(f"  ERROR {p}")
    if problems:
        raise SystemExit("preflight failed; nothing was spent")

    return {"boards": boards, "per_set": per_set, "est_seconds": est_s}


def baseline_summary(args, set_name: str) -> Path:
    if args.baseline == "recorded":
        return Path(args.out).expanduser() / "_baseline" / set_name / "summary.json"
    return Path(args.baseline).expanduser() / set_name / "summary.json"


def build_recorded_baseline(args, sets, stress):
    """Baseline = the ORIGINAL recorded runs, re-graded with today's grader.

    This is the right baseline for a replay, and an archived ab_* wave usually is
    not. The corpus gets RE-RECORDED (sets 10-19 were, after the #562 pours-first
    reshape: the 2026-07-28 wave replayed a `route -> planes -> repair` chain,
    while today's manifests run `planes -> route` with the repair absorbed). A
    board-for-board diff against that wave therefore mixes a different recorded
    PLAN in with the engine delta and cannot separate them.

    The recorded run dir, by contrast, holds the very boards the manifest we are
    replaying produced -- same plan, so the only moving parts are the engine and
    (removed here) the grader.

    Materialized as SYMLINKS into a scratch wave dir rather than copies: the
    corpus is precious and read-only in spirit, and regrade() rewrites
    summary.json in whatever directory you hand it -- pointed at runs_setN that
    would clobber the recorded run's own summary.
    """
    import concurrent.futures
    import sweep_lib as sl
    from ab_replay_grade import final_output_name
    root = Path(args.out).expanduser() / "_baseline"
    jobs = max(1, int(getattr(args, "jobs", 6) or 6))
    print(f"\n=== BASELINE (recorded runs, re-graded today) ===\n  {root}\n"
          f"  {len(sets)} set(s), {jobs} in parallel")

    def one_set(s):
        rds = sl.run_dirs_for(stress, s)
        if not rds:
            return f"  {s}: no runs dir; skipped"
        out_lines = []
        run_dir = rds[0]
        wave = root / s
        wave.mkdir(parents=True, exist_ok=True)
        linked = missing = 0
        for bdir in sorted(p for p in run_dir.iterdir() if p.is_dir()):
            man = bdir / "redo_commands.sh"
            if not man.exists():
                continue
            fn = final_output_name(man.read_text())
            final = bdir / fn if fn else None
            dest = wave / bdir.name
            dest.mkdir(parents=True, exist_ok=True)
            if not (final and final.exists()):
                missing += 1
                continue
            # The project/dru siblings must come along: grading a board without
            # its .kicad_pro resolves the floor from the STOCK netclass and
            # invents sub-floor violations on correct copper (#441).
            for sib in sorted(final.parent.glob(final.stem + ".*")):
                if sib.suffix not in (".kicad_pcb", ".kicad_pro", ".kicad_dru", ".kicad_prl"):
                    continue
                link = dest / sib.name
                if link.is_symlink() or link.exists():
                    link.unlink()
                link.symlink_to(sib)
            linked += 1
        out_lines.append(f"  {s:9} {linked:3} final board(s) linked"
                         + (f", {missing} with NO final (chain never completed "
                            f"in the recording)" if missing else ""))
        subprocess.run([sys.executable, str(HERE / "ab_replay_grade.py"),
                        "--regrade", str(wave), "--set", str(run_dir)],
                       stdout=subprocess.DEVNULL)
        summ = wave / "summary.json"
        if summ.exists():
            rows = json.loads(summ.read_text())
            ok = sum(1 for r in rows if r.get("chain_complete"))
            out_lines.append(f"  {s:9} regraded: {ok}/{len(rows)} complete")
            (wave / "git_version.json").write_text(json.dumps(
                {"label": "recorded-run", "source": "runs dir, re-graded today",
                 "commit": git_sha(short=False), "describe": git_sha(),
                 "dirty": bool(tree_dirty()),
                 "note": "boards are the ORIGINAL recorded outputs; the grader is today's"},
                indent=1))
        return "\n".join(out_lines)

    # Regrading a set is an independent subprocess over its own boards, so the
    # sets fan out cleanly. Results are printed in COMPLETION order but each set
    # emits its lines as one block, so the log stays readable.
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        for line in ex.map(one_set, sets):
            if line:
                print(line, flush=True)


# ------------------------------------------------------------------- upload

def corpus_has(sets: list, stress: Path) -> set:
    """Set names already present on the corpus volume, under the name the
    CONTAINERS will ask for.

    The name that matters is the one `sweep_lib.discover_boards` derives from
    the LOCAL preferred run dir -- modal_app resolves the manifest as
    `/corpus/runs_<that name>`. So "present" means that exact dir is on the
    volume, not merely something that looks like it.

    This was a substring test (`f"runs_{s}" in listing`) against the whole
    listing text, and it lied in exactly the case that matters: sets 1-5 had
    been RE-RECORDED locally as plain `runs_set1..5`, while the volume still
    held only the older `runs_set1_llm0801`. "runs_set1" is a substring of
    "runs_set1_llm0801", so upload reported all five present and pushed
    nothing -- then all 75 boards died container-side on
    FileNotFoundError: /corpus/runs_set2/ulx3s/redo_commands.sh.
    """
    import sweep_lib as sl

    r = subprocess.run(["modal", "volume", "ls", CORPUS_VOLUME, "/"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"  (could not list {CORPUS_VOLUME}: {r.stderr.strip()[:200]}); "
              f"assuming nothing is uploaded")
        return set()
    on_volume = {ln.strip() for ln in r.stdout.splitlines() if ln.strip()}
    have = set()
    for s in sets:
        # A set is present when its RUN dir is -- boards_* alone is not enough
        # to replay.
        local = sl.run_dirs_for(stress, s)
        wanted = local[0].name if local else f"runs_{s}"
        if wanted in on_volume:
            have.add(s)
    return have


def stage_upload(args, sets, stress):
    have = corpus_has(sets, stress)
    missing = [s for s in sets if s not in have]
    print(f"\n=== UPLOAD ===\n  corpus already has: "
          f"{', '.join(sorted(have)) or '(none)'}")
    if not missing:
        print("  nothing to upload")
        return
    print(f"  uploading: {', '.join(missing)}")
    cmd = [sys.executable, str(SWEEP / "upload_corpus.py"),
           "--sets", ",".join(missing), "--stress-dir", str(stress)]
    if args.dry_run:
        cmd.append("--dry-run")
    r = sh(cmd)
    if r.returncode != 0:
        raise SystemExit(f"upload failed (rc={r.returncode})")


# ---------------------------------------------------------------------- run

def arm_name(args) -> str:
    """Arm identity = label + source commit.

    The results volume is persistent and modal_app RESUMES by skipping tasks
    that already have a row. Without the commit in the name, a second run after
    a code change would silently re-use the first run's rows and report two
    engines as one -- the exact failure the sweep README calls out ("the
    baseline was not the baseline").
    """
    # --arm overrides entirely: harvesting a wave produced by ANOTHER session
    # (or another commit) must not depend on what this checkout happens to be
    # sitting on. Without it, `--only harvest` silently looks for the local
    # HEAD's arm and reports "no result rows" for a run that is right there.
    if getattr(args, "arm", ""):
        return args.arm
    # A previous stage's arm name, recorded at RUN time. HEAD moves -- a
    # concurrent session committing between `run` and `harvest` silently
    # renamed the arm this driver looks for, and harvest then reported "no
    # result rows" for a wave sitting complete on the volume. The launch-time
    # name is the truth; recompute only when there is nothing recorded.
    stamp = Path(args.out).expanduser() / "arm.txt"
    if stamp.exists():
        name = stamp.read_text().strip()
        if name:
            return name
    return f"{args.label}_{git_sha()}"


def stage_run(args, sets, stress, plan):
    arm = arm_name(args)
    arms_file = Path(args.workdir) / "arms.replay.json"
    arms_file.parent.mkdir(parents=True, exist_ok=True)
    # Record the arm name so later stages find it even if HEAD has moved.
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "arm.txt").write_text(arm + "\n")
    # One arm, no overrides: a straight replay of whatever the image ships.
    # keep_artifacts rides the ARM SPEC because that is what gets serialized into
    # the task -- a Modal container does not inherit this shell's environment, so
    # exporting the env var here would keep nothing (the smoke run proved it:
    # chain green, zero artifacts).
    spec = {"name": arm, "keep_artifacts": True,
            "note": "cloud replay"}
    # Overrides ride the ARM SPEC, not this shell's environment -- a Modal
    # container inherits nothing from here (that is what silently kept zero
    # artifacts on the first run). `env` reaches the routing subprocess;
    # `defaults` patches routing_defaults.py inside the container's own repo
    # copy, which is how a module constant is A/B'd without shared git state.
    if getattr(args, "env", None):
        spec["env"] = dict(kv.split("=", 1) for kv in args.env)
    if getattr(args, "defaults", None):
        # int BEFORE float: several routing_defaults constants land in a pyo3
        # `i32` argument (DIRECTION_PREFERENCE_COST, VIA_COST, TURN_COST, ...),
        # and pyo3 refuses a Python float there -- coercing every numeric knob
        # to float made such an arm raise TypeError on its first route step
        # instead of measuring the knob.
        spec["defaults"] = {k: _num(v)
                            for k, v in (kv.split("=", 1) for kv in args.defaults)}
    if spec.get("env") or spec.get("defaults"):
        spec["note"] = f"cloud replay; env={spec.get('env')} defaults={spec.get('defaults')}"
    arms_file.write_text(json.dumps([spec], indent=1))
    print(f"  arm spec: {json.dumps(spec)}")

    env = dict(os.environ)
    # Name the Modal app after the ARM, not just the sets. A bisect ladder
    # runs many points over the SAME sets, and naming by sets alone made
    # every point show up on the dashboard as an identical
    # "kicad-replay-set1-set5" -- so a half-launched ladder (three points
    # silently lost to Modal's app-create rate limit) looked indistinguishable
    # from a healthy one, and you cannot tell which point a running app is.
    _slug = arm if arm.startswith("kicad") else f"kicad-{arm}"
    env.setdefault("KICAD_SWEEP_NAME", f"{_slug}-{sets[0]}-{sets[-1]}")
    # No big-memory tier. Modal bills max(reserved, used) and containers burst
    # ABOVE their reservation, so pinning a heavy board to a fat tier buys no
    # capacity -- only a larger bill against every hour it runs (the sweep
    # README measured a flat 1 GB reservation at +0.3% over the pure-usage
    # floor, versus +47% for tiering and +78% for per-board peaks).

    print(f"\n=== RUN ===\n  arm: {arm}\n  artifacts: ON "
          f"(final .kicad_pcb + siblings kept per board)")
    # --ignore-excludes: board_value.json's exclude list is FITTED to the arm
    # family it was measured on -- boards where competing soft-cost arms all tied
    # carry no information FOR A SWEEP. A replay is not a sweep: every board is a
    # datum, and the baseline wave graded all of them, so dropping any would
    # silently shrink the comparison (the smoke run showed it culling 5 of
    # set10's 15 before we asked for one board).
    cmd = ["modal", "run", str(SWEEP / "modal_app.py"),
           "--arms", str(arms_file), "--sets", ",".join(sets),
           "--stress-dir", str(stress), "--ignore-excludes"]
    if args.limit:
        cmd += ["--limit", str(args.limit)]
    if args.boards:
        cmd += ["--boards", args.boards]
    if getattr(args, "exclude", ""):
        # modal_app has no exclude flag, so express it as an explicit board LIST
        # minus the exclusions. Needed because one hung board holds up every arm:
        # core64_logic replays in 4 min historically and now never terminates,
        # so every arm sits at 129/130 waiting on it.
        drop = {b.strip() for b in args.exclude.split(",") if b.strip()}
        keep = [b for _s, b in plan.get("boards", []) if b not in drop]
        if keep and not args.boards:
            cmd += ["--boards", ",".join(sorted(set(keep)))]
        print(f"  excluding {len(drop)} board(s): {', '.join(sorted(drop))}")
    for _try in range(1, RATE_LIMIT_TRIES + 1):
        rc, out = sh_capture(cmd, env=env)
        if rc == 0:
            break
        if RATE_LIMIT_MARK in out.lower() and _try < RATE_LIMIT_TRIES:
            wait = _try * 90
            print(f"  Modal app-create rate limit hit; retry {_try}/"
                  f"{RATE_LIMIT_TRIES - 1} in {wait}s (banked rows make this cheap)",
                  flush=True)
            time.sleep(wait)
            continue
        # Any other failure is real -- do not paper over it with a retry.
        raise SystemExit(f"cloud run failed (rc={rc}); banked rows are "
                         f"still on the volume -- re-run to resume")


# ------------------------------------------------------------------ harvest

def stage_harvest(args, sets, stress) -> dict:
    """Pull this arm's rows and boards into a local ab_replay_grade-shaped wave.

    Layout (per set), which is exactly what --compare and --regrade expect:
        <out>/<set>/summary.json
        <out>/<set>/<board>/<final>.kicad_pcb   (+ .kicad_pro/.kicad_dru/...)
        <out>/<set>/<board>/redo_commands.sh
    """
    arm = arm_name(args)
    out = Path(args.out).expanduser()
    raw = out / "_raw"
    if raw.exists():
        shutil.rmtree(raw)
    raw.mkdir(parents=True, exist_ok=True)

    print(f"\n=== HARVEST ===\n  arm {arm} -> {out}")
    r = sh(["modal", "volume", "get", "--force", RESULTS_VOLUME, f"/{arm}", str(raw)])
    if r.returncode != 0:
        raise SystemExit(f"volume get failed (rc={r.returncode}) -- is the arm name right? "
                         f"`modal volume ls {RESULTS_VOLUME} /`")

    # `modal volume get` may nest under the remote dir name; find the rows.
    roots = [p for p in [raw, raw / arm] if p.exists()]
    rows_by_set, artifacts = {}, 0
    for root in roots:
        for jf in sorted(root.glob("*.json")):
            try:
                row = json.loads(jf.read_text())
            except Exception as e:
                print(f"  WARN unreadable row {jf.name}: {e}")
                continue
            row = row[0] if isinstance(row, list) else row
            s = row.get("set") or jf.name.split("__")[0]
            rows_by_set.setdefault(s, []).append(row)
        adir = root / "artifacts"
        if adir.is_dir():
            for bdir in sorted(p for p in adir.iterdir() if p.is_dir()):
                if "__" not in bdir.name:
                    continue
                s, board = bdir.name.split("__", 1)
                dest = out / s / board
                dest.mkdir(parents=True, exist_ok=True)
                for f in sorted(bdir.iterdir()):
                    if f.is_file():
                        shutil.copy2(f, dest / f.name)
                artifacts += 1

    if not rows_by_set:
        raise SystemExit(f"no result rows harvested for arm {arm}")

    total = 0
    for s, rows in sorted(rows_by_set.items()):
        rows.sort(key=lambda r: r.get("board", ""))
        sdir = out / s
        sdir.mkdir(parents=True, exist_ok=True)
        (sdir / "summary.json").write_text(json.dumps(rows, indent=2))
        # Provenance beside the summary, so compare() can print it and a later
        # reader can tell which engine produced these numbers.
        prov = {"commit": git_sha(short=False), "describe": git_sha(),
                "dirty": bool(tree_dirty()), "label": args.label,
                "arm": arm, "captured": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "source": "cloud_replay_sets.py"}
        (sdir / "git_version.json").write_text(json.dumps(prov, indent=1))
        total += len(rows)
        complete = sum(1 for r in rows if r.get("chain_complete"))
        with_art = sum(1 for r in rows if r.get("artifacts"))
        print(f"  {s:9} {len(rows):3} rows, {complete:3} chain-complete, "
              f"{with_art:3} with kept boards")

    print(f"  harvested {total} row(s), {artifacts} board artifact dir(s)")

    # Re-grade the harvested boards LOCALLY, on equal terms with the baseline.
    #
    # The cloud image deliberately ships no KiCad, so ab_replay_grade's kicad-cli
    # cross-check degrades to None there and `drc_real` falls back to RAW DRC.
    # The baseline is graded locally, where kicad-cli reconciles violations that
    # are pre-existing or not real. Comparing the two directly is not an engine
    # measurement at all -- it measures which machine did the grading.
    #
    # Measured on sets 10-19 the first time this ran: the comparison reported
    # "real DRC 47 -> 87, +40 WORSE", of which +77 was this artifact across 9
    # boards, while the 8 genuine raw-DRC changes netted -37 BETTER (one board,
    # apple1_aci_card, went 44 -> 0). The headline number had the wrong SIGN.
    #
    # Keeping the boards is what makes the fix possible: grade them here, with
    # the same grader and the same kicad-cli the baseline gets.
    missing_pro = [f"{r.get('set')}/{r.get('board')}"
                   for rows in rows_by_set.values() for r in rows
                   if r.get("artifact_warning")]
    if missing_pro:
        print(f"  WARN {len(missing_pro)} board(s) flagged an artifact problem "
              f"(a .kicad_pcb without its .kicad_pro re-grades against the stock "
              f"netclass): {', '.join(missing_pro[:5])}")

    if not getattr(args, "no_local_regrade", False):
        _regrade_locally(args, sorted(rows_by_set), stress, out, "harvested wave")
    return rows_by_set


def _regrade_locally(args, sets, stress, out, what):
    """Re-grade an already-materialized wave in place, sets in parallel."""
    import concurrent.futures
    import sweep_lib as sl
    jobs = max(1, int(getattr(args, "jobs", 6) or 6))
    print(f"  re-grading the {what} locally ({len(sets)} sets, {jobs} in parallel) "
          f"so it is graded on the SAME terms as the baseline")
    # Snapshot every harvested row BEFORE the regrade overwrites summary.json.
    _pre_regrade = {}
    for s in sets:
        f = out / s / "summary.json"
        if f.exists():
            try:
                _pre_regrade[s] = {r.get("board"): r for r in json.loads(f.read_text())}
            except Exception:
                pass

    def one(s):
        rds = sl.run_dirs_for(stress, s)
        wave = out / s
        if not rds or not wave.is_dir():
            return f"    {s}: skipped (no runs dir or no wave dir)"
        subprocess.run([sys.executable, str(HERE / "ab_replay_grade.py"),
                        "--regrade", str(wave), "--set", str(rds[0])],
                       stdout=subprocess.DEVNULL)
        summ = wave / "summary.json"
        if not summ.exists():
            return f"    {s}: regrade produced no summary"
        rr = json.loads(summ.read_text())

        # Merge back rows the regrade DROPPED. --regrade rebuilds summary.json
        # from the board DIRECTORIES it finds, so any board whose artifact was
        # not kept vanishes from the results entirely -- 150 harvested rows
        # became 137, and the comparison then reported those boards as "not run"
        # when in fact they ran, completed, and their rows were on the volume.
        # Keep them, marked as cloud-graded, so they are excluded from the
        # PAIRED comparison honestly rather than deleted silently.
        pre = _pre_regrade.get(s) or {}
        have = {r.get("board") for r in rr}
        readded = []
        for b, row in sorted(pre.items()):
            if b not in have:
                row = dict(row)
                row["regraded"] = False
                rr.append(row)
                readded.append(b)
        if readded:
            rr.sort(key=lambda r: r.get("board") or "")
            summ.write_text(json.dumps(rr, indent=2))

        kc = sum(1 for r in rr if r.get("kicad_connection_width") is not None)
        return (f"    {s:9} {sum(1 for r in rr if r.get('chain_complete')):3}/"
                f"{len(rr):<3} complete, {kc:3} with the kicad-cli cross-check"
                + (f"; {len(readded)} kept as CLOUD-GRADED (no artifact to "
                   f"re-grade: {', '.join(readded[:3])}"
                   f"{'...' if len(readded) > 3 else ''})" if readded else ""))

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        for line in ex.map(one, sets):
            print(line, flush=True)


# ------------------------------------------------------------------ compare

def regrade_baseline(args, sets, stress):
    """Re-score the baseline's OWN boards with today's grader.

    Comparing a fresh run against a months-old summary conflates a code delta
    with grader drift. The baseline kept its boards, so it can simply be
    re-scored. The archive's original summary is snapshotted first and never
    thrown away.
    """
    import sweep_lib as sl
    print("\n=== REGRADE BASELINE (today's grader) ===")
    for s in sets:
        wave = Path(args.baseline).expanduser() / s
        summ = wave / "summary.json"
        if not summ.exists():
            print(f"  {s}: no baseline summary; skipped")
            continue
        snap = wave / "summary.pre_regrade.json"
        if not snap.exists():
            shutil.copy2(summ, snap)
            print(f"  {s}: snapshotted original -> {snap.name}")
        run_dirs = sl.run_dirs_for(stress, s)
        if not run_dirs:
            print(f"  {s}: no runs dir; cannot regrade (needs manifests)")
            continue
        sh([sys.executable, str(HERE / "ab_replay_grade.py"),
            "--regrade", str(wave), "--set", str(run_dirs[0])])


def stage_compare(args, sets, stress):
    out = Path(args.out).expanduser()
    print("\n=== COMPARE vs baseline ===")
    print(f"  baseline: {args.baseline}")
    if args.regrade_baseline:
        regrade_baseline(args, sets, stress)

    agg = {"sets": {}, "baseline": str(args.baseline), "new": str(out)}
    for s in sets:
        old = baseline_summary(args, s)
        new = out / s / "summary.json"
        if not new.exists():
            print(f"\n-- {s}: no new summary (not harvested?); skipped")
            continue
        if not old.exists():
            print(f"\n-- {s}: no baseline summary; skipped")
            continue
        prov = wave_provenance(old)
        print(f"\n{'='*100}\n-- {s}   baseline {prov.get('describe') or prov.get('commit','?')[:9]}"
              f"{' DIRTY' if prov.get('dirty') else ''}   "
              f"{'(re-graded today)' if args.regrade_baseline else '(as archived)'}")
        r = sh([sys.executable, str(HERE / "ab_replay_grade.py"),
                "--compare", str(old), str(new)])
        agg["sets"][s] = {"baseline_summary": str(old), "new_summary": str(new),
                          "compare_rc": r.returncode}

    # A roll-up across sets, computed from the same fields compare() grades on,
    # so a per-set win/loss does not hide in ten separate tables.
    agg["totals"] = aggregate_totals(args, sets, out)
    (out / "aggregate.json").write_text(json.dumps(agg, indent=2))
    print_aggregate(agg["totals"])
    print(f"\nwrote {out/'aggregate.json'}")


def _incompl(r):
    ni = r.get("nets_incomplete")
    return ni if ni is not None else r.get("conn")


def _drc(r):
    v = r.get("drc_real")
    return v if v is not None else r.get("drc")


def aggregate_totals(args, sets, out) -> dict:
    """Sum the graded axes over boards COMPLETE IN BOTH waves.

    Restricting to boards complete in both is what ab_replay_grade's own compare
    does: a board that fails to produce a final in one wave has no comparable
    number, and letting it contribute zero would read as an improvement.
    """
    tot = {"boards_compared": 0, "drc_old": 0, "drc_new": 0,
           "incompl_old": 0, "incompl_new": 0,
           "baseline_only_complete": [], "new_only_complete": [], "not_run": [],
           "cloud_graded_only": []}
    for s in sets:
        op, np_ = baseline_summary(args, s), out / s / "summary.json"
        if not (op.exists() and np_.exists()):
            continue
        old = {r["board"]: r for r in json.loads(op.read_text())}
        new = {r["board"]: r for r in json.loads(np_.read_text())}
        for b in sorted(set(old) | set(new)):
            o, n = old.get(b), new.get(b)
            oc = bool(o and o.get("chain_complete"))
            nc = bool(n and n.get("chain_complete"))
            if n is not None and n.get("regraded") is False:
                # Ran and completed, but has no local re-grade -- comparing it
                # against a locally-graded baseline would compare GRADERS.
                tot.setdefault("cloud_graded_only", []).append(f"{s}/{b}")
                continue
            if n is None:
                # Absent from the new wave entirely (partial run, --boards/--limit,
                # or a task that never produced a row). Calling that a REGRESSION
                # would be a lie -- it was never attempted.
                tot["not_run"].append(f"{s}/{b}")
                continue
            if oc and not nc:
                tot["baseline_only_complete"].append(f"{s}/{b}")
            elif nc and not oc:
                tot["new_only_complete"].append(f"{s}/{b}")
            if not (oc and nc):
                continue
            do, dn = _drc(o), _drc(n)
            io, inw = _incompl(o), _incompl(n)
            if None in (do, dn, io, inw):
                continue
            tot["boards_compared"] += 1
            tot["drc_old"] += do
            tot["drc_new"] += dn
            tot["incompl_old"] += io
            tot["incompl_new"] += inw
    return tot


def print_aggregate(t: dict):
    print(f"\n{'='*100}\n=== AGGREGATE over all sets ===")
    n = t["boards_compared"]
    if not n:
        print("  no boards were complete in BOTH waves -- nothing comparable")
        return
    dd = t["drc_new"] - t["drc_old"]
    di = t["incompl_new"] - t["incompl_old"]

    def arrow(d):
        return "BETTER" if d < 0 else ("worse" if d > 0 else "same")
    print(f"  boards compared (complete in both): {n}")
    print(f"  real DRC          {t['drc_old']:6} -> {t['drc_new']:6}   "
          f"{dd:+6}  {arrow(dd)}")
    print(f"  incomplete nets   {t['incompl_old']:6} -> {t['incompl_new']:6}   "
          f"{di:+6}  {arrow(di)}")
    if t["baseline_only_complete"]:
        print(f"  REGRESSED to broken chain ({len(t['baseline_only_complete'])}): "
              f"{', '.join(t['baseline_only_complete'][:8])}")
    if t.get("cloud_graded_only"):
        print(f"  cloud-graded only, excluded from the pairing "
              f"({len(t['cloud_graded_only'])}, they RAN -- their artifact was not "
              f"kept so they could not be re-graded on the baseline's terms): "
              f"{', '.join(t['cloud_graded_only'][:6])}"
              f"{' ...' if len(t['cloud_graded_only']) > 6 else ''}")
    if t.get("not_run"):
        print(f"  not run in this wave ({len(t['not_run'])}, NOT a regression): "
              f"{', '.join(t['not_run'][:8])}"
              f"{' ...' if len(t['not_run']) > 8 else ''}")
    if t["new_only_complete"]:
        print(f"  newly completing ({len(t['new_only_complete'])}): "
              f"{', '.join(t['new_only_complete'][:8])}")


# --------------------------------------------------------------------- main

STAGES = ["plan", "upload", "run", "harvest", "baseline", "compare"]


def main():
    ap = argparse.ArgumentParser(
        description="Cloud-replay stress sets, keep the boards, A/B vs a baseline.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="example:\n"
               "  %(prog)s --sets set10-set19\n"
               "  %(prog)s --sets set10-set19 --only plan\n"
               "  %(prog)s --sets set10 --boards bus_pirate5 --label smoke\n")
    ap.add_argument("--sets", default="set10-set19",
                    help="ranges and/or names: set10-set19, set10,set12 (default: %(default)s)")
    ap.add_argument("--out", default="", help="local wave dir (default: <stress>/cloud_<label>_<sha>)")
    ap.add_argument("--baseline", default="recorded",
                    help="'recorded' (default) = the ORIGINAL recorded runs, re-graded with "
                         "today's grader -- same chain, same grader, so a delta is the ENGINE. "
                         "Or a path to a wave dir holding <set>/summary.json (e.g. an ab_* "
                         "wave), which is only comparable if it replayed the same chain.")
    ap.add_argument("--allow-chain-mismatch", action="store_true",
                    help="compare against a wave that replayed a DIFFERENT chain anyway")
    ap.add_argument("--label", default="replay", help="run label, part of the arm name")
    ap.add_argument("--arm", default="",
                    help="exact arm name on the results volume, overriding <label>_<sha>. "
                         "Use it to harvest/compare a wave produced by another session or "
                         "commit (`modal volume ls kicad-sweep-results /` lists them).")
    ap.add_argument("--stress-dir", default=str(DEFAULT_STRESS))
    ap.add_argument("--no-local-regrade", action="store_true",
                    help="do NOT re-grade harvested boards locally. SAFE on a KiCad "
                         "image (the default since 2026-08-23): the containers run the "
                         "same kicad-cli grader this machine does and the two were "
                         "checked to agree, so cloud grading is not a different grader. "
                         "Do NOT use it on a --no-kicad wave or one banked before that "
                         "date -- those rows have no real drc_real to compare.")
    ap.add_argument("--env", action="append", default=[], metavar="K=V",
                    help="KICAD_* knob for this arm, repeatable. Rides the arm spec "
                         "(containers do not inherit this shell's environment).")
    ap.add_argument("--defaults", action="append", default=[], metavar="NAME=VALUE",
                    help="routing_defaults.py constant to patch for this arm, repeatable.")
    ap.add_argument("--jobs", type=int, default=6,
                    help="sets re-graded in parallel during the baseline stage (default 6)")
    ap.add_argument("--workdir", default="", help="scratch for the generated arms file")
    ap.add_argument("--only", default="", help=f"run one/some stages: {'|'.join(STAGES)} (comma-separated)")
    ap.add_argument("--dry-run", action="store_true",
                    help="plan only, and make `upload` report sizes without pushing")
    ap.add_argument("--limit", type=int, default=0, help="keep only the N cheapest boards (smoke)")
    ap.add_argument("--boards", default="", help="restrict to these board names (smoke)")
    ap.add_argument("--exclude", default="",
                    help="board names to DROP (comma-separated). One non-terminating "
                         "board stalls every arm at N-1, so exclude known hangs.")
    ap.add_argument("--regrade-baseline", action="store_true",
                    help="re-score the baseline's own boards with TODAY's grader before "
                         "comparing (snapshots the archive's summary.json first). Use this "
                         "whenever you intend to attribute a delta to CODE.")
    ap.add_argument("--allow-dirty-baseline", action="store_true",
                    help="proceed even if the baseline wave was built from a dirty tree")
    ap.add_argument("--no-baseline", action="store_true",
                    help="skip baseline checks and the compare stage entirely")
    ap.add_argument("--with-kicad", action="store_true", default=True,
                    help="(DEFAULT since 2026-08-23) build the image WITH KiCad "
                         "(kicad/kicad:10.0.0) so the oracle legs actually run. "
                         "Use --no-kicad to opt out.")
    ap.add_argument("--no-kicad", dest="with_kicad", action="store_false",
                    help="build WITHOUT KiCad (the old default). Every oracle leg "
                         "is then DEAD, not degraded -- oracle_reconnect returns "
                         "available=False as soon as find_kicad_cli() is None, so a "
                         "change acting only through the finalize audit / #589 "
                         "re-audit / oracle-summary check A/Bs as a PERFECT NULL that "
                         "reads as a measurement (#650). Only use it when you know "
                         "the change cannot touch an oracle leg and you want the "
                         "cheaper image. NOTE the arm label loses its -kc suffix, "
                         "so such an arm is not comparable with a KiCad one.")
    args = ap.parse_args()

    sets = expand_sets(args.sets)
    stress = Path(args.stress_dir).expanduser()
    if not stress.is_dir():
        raise SystemExit(f"stress dir not found: {stress}")
    if args.with_kicad:
        # The results volume RESUMES by arm name, and arm = label + sha. Two
        # waves at the SAME commit that differ only by the image would collide
        # there -- the no-KiCad rows would be re-used for the KiCad arm and the
        # two engines reported as one, the exact "the baseline was not the
        # baseline" failure arm_name() exists to prevent. Suffix the label so
        # they can never share an arm.
        if not args.label.endswith("-kc"):
            args.label += "-kc"
        os.environ["KICAD_SWEEP_WITH_KICAD"] = "1"
    if not args.out:
        args.out = str(stress / f"cloud_{args.label}_{git_sha()}")
    if not args.workdir:
        args.workdir = args.out
    stages = [s.strip() for s in args.only.split(",") if s.strip()] or list(STAGES)
    for s in stages:
        if s not in STAGES:
            raise SystemExit(f"unknown stage {s!r}; pick from {STAGES}")
    if args.no_baseline and "compare" in stages and not args.only:
        stages = [s for s in stages if s != "compare"]

    print(f"sets      : {', '.join(sets)}")
    print(f"stress    : {stress}")
    print(f"out       : {args.out}")
    print(f"stages    : {', '.join(stages)}")
    print(f"image     : {'kicad/kicad:10.0.0 (oracle legs LIVE)' if args.with_kicad else 'debian_slim (no KiCad -- oracle legs are no-ops)'}")

    plan = stage_plan(args, sets, stress) if "plan" in stages else {}
    if args.dry_run:
        print("\n--dry-run: stopping after plan; nothing was spent")
        if "upload" in stages:
            stage_upload(args, sets, stress)
        return 0
    if "upload" in stages:
        stage_upload(args, sets, stress)
    if "run" in stages:
        stage_run(args, sets, stress, plan)
    if "harvest" in stages:
        stage_harvest(args, sets, stress)
    if "baseline" in stages and not args.no_baseline and args.baseline == "recorded":
        build_recorded_baseline(args, sets, stress)
    if "compare" in stages and not args.no_baseline:
        stage_compare(args, sets, stress)
    return 0


if __name__ == "__main__":
    sys.exit(main())
