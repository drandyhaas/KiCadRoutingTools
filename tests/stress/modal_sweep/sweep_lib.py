#!/usr/bin/env python3
"""Pure-python helpers for the Modal parameter sweep -- no modal import here, so
this file is unit-testable and usable from the CLI without a Modal account.

Three things live here:
  1. arm parameter application (env / routing_defaults constants / manifest sed)
  2. the recorded-cost table used for longest-first dispatch + memory tiering
  3. task-list construction

Why longest-first matters: the sweep's wall-clock floor is the single longest
board, so a 50-minute board dispatched last adds 50 minutes to the tail. Sorting
descending by recorded cost puts the monsters in flight first.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

# Memory tiers, resized 2026-08-10 from 414 MEASURED cloud tasks (real Linux
# RSS via the fixed /proc sampler). Modal bills max(reserved, used) per
# second, so a reservation only ever ADDS cost -- it buys OOM-protection,
# not capacity (containers burst above the reservation). Measured: flat 1 GB
# lands 0.3% above the pure-usage floor (673 vs 671 GB-h on the sample),
# while per-board peak reservation costs +78% (the board max bills against
# every LIGHT arm's runtime too) and the old 4/12 sizing +47%. Peaks
# routinely exceed the reserve BY DESIGN: p50 2.1 GB, p90 5.2 GB, max 8.6 GB
# (hackrf_one, every arm) all completed in 2 GB containers via burst.
# The big tier is OOM insurance for boards that PROVE flaky under burst:
# name them in KICAD_SWEEP_BIG_BOARDS (comma-separated) for a top-up run.
DEFAULT_MEM_MB = 1024
BIG_MEM_MB = 8192
# Fallback cost for a board with no recorded timing (mid-pack, not last).
DEFAULT_COST_S = 120.0


# --------------------------------------------------------------------------
# 1. Applying an arm's parameters
# --------------------------------------------------------------------------

def patch_routing_defaults(repo: Path, overrides: dict) -> dict:
    """Rewrite module-level constants in py_router/routing_defaults.py.

    routing_defaults exposes plain module constants (TRACK_PROXIMITY_COST,
    VIA_COST, ...) with no env-override hook, so an arm that varies one has to
    edit the file -- which is exactly what the documented local A/B recipe does
    with `git stash`. Per-container patching does the same thing without any
    shared git state, which is what lets 100 arms run CONCURRENTLY instead of
    sequentially.

    Returns {name: (old, new)} for logging. Raises on an unknown constant so a
    typo in an arm spec fails loudly instead of silently testing nothing.
    """
    path = repo / "py_router" / "routing_defaults.py"
    text = path.read_text()
    applied = {}
    for name, value in (overrides or {}).items():
        # [ \t]* NOT \s* -- \s crosses newlines, so the trailing-comment group
        # would reach down and capture a comment on a LATER line, splicing it
        # onto the patched line (caught in testing: TRACK_PROXIMITY_COST came
        # out as `= 2.0  # Distance parameters`, stealing a section header two
        # lines below). Keep every quantifier here line-local.
        pat = re.compile(rf"^{re.escape(name)}[ \t]*=[ \t]*([^\n#]+?)[ \t]*(#[^\n]*)?$",
                         re.MULTILINE)
        m = pat.search(text)
        if not m:
            raise KeyError(
                f"{name} is not a module-level constant in routing_defaults.py -- "
                f"check the arm spec (an unknown name would silently test nothing)"
            )
        applied[name] = (m.group(1), repr(value))
        comment = f"  {m.group(2)}" if m.group(2) else ""
        text = text[:m.start()] + f"{name} = {value!r}{comment}" + text[m.end():]
    path.write_text(text)
    return applied


def patch_manifest(manifest_text: str, subs: list) -> str:
    """Apply [{'pattern': ..., 'replacement': ...}] to a redo_commands.sh body.

    This is the arm mechanism for parameters that are CLI FLAGS rather than
    module constants (e.g. --via-proximity-cost). Anchor patterns tightly; a
    loose one silently rewrites more of the chain than intended.
    """
    out = manifest_text
    for s in subs or []:
        out = re.sub(s["pattern"], s["replacement"], out)
    return out


def recorded_repo_prefix(manifest_text: str) -> str | None:
    """The repo path baked into a manifest's tool invocations.

    Manifests reference tools by ABSOLUTE path, so a replay runs whatever lives
    at the recorded path. In a container the repo is elsewhere, so we detect the
    recorded prefix and hand ab_replay_grade an --extra-remap for it. Detecting
    it (rather than assuming one constant) is what lets manifests recorded from
    different checkouts -- a worktree, someone else's home dir -- all replay.

    Two flavors exist: post-#522 manifests say <repo>/py_router/<tool>.py (and
    tests/ for checks), but PRE-#522 recordings (e.g. the whole llm0801 wave --
    10 of the first 12 set dirs) bake ROOT-level tool paths
    (<repo>/bga_fanout.py) with no py_router/tests component anywhere. The
    first pattern misses those entirely -- which made every such board's steps
    exit rc=2 in the container ("can't open file", no remap) while
    --continue-on-error kept the task green. The fallback anchors on the
    python3 invocation itself, so the token is always a TOOL path, never a
    corpus path; in-container the remapped root-level path is then dead and
    redo_stress_test's relocate_moved_scripts (#522/#562) rewrites it into
    py_router/ -- the same two-step dance that makes these manifests replay
    locally.
    """
    m = re.search(r"(\S+?)/(?:py_router|tests)/[A-Za-z_0-9]+\.py", manifest_text)
    if m:
        return m.group(1)
    m = re.search(r"python3\S*(?: -X \S+)? (\S+?)/[A-Za-z_0-9.]+\.py",
                  manifest_text)
    return m.group(1) if m else None


def recorded_corpus_prefix(manifest_text: str) -> str | None:
    """The stress dir baked into a manifest, from its `# cwd=` line.

    Manifests reference the INPUT board absolutely
    (`<stress>/boards_unrouted_<set>/<board>.kicad_pcb`) while intermediates are
    relative to the run dir. So a replay needs the corpus reachable at its
    recorded path, not just the repo. The `# cwd=` line is
    `<stress>/runs_<set>/<board>`, so the stress dir is its grandparent.
    """
    m = re.search(r"^#\s*cwd=(\S+)/runs_[^/\s]+/[^/\s]+\s*$", manifest_text, re.MULTILINE)
    return m.group(1) if m else None


# --------------------------------------------------------------------------
# 2. Recorded cost table (dispatch order + memory tier)
# --------------------------------------------------------------------------

def load_cost_table(stress_dir: Path) -> dict:
    """board -> {'seconds': float, 'peak_mb': float} from prior runs.

    Prefers REPLAY timings (ab_*/**/summary.json: total_seconds +
    peak_footprint_mb) because that is what this sweep actually runs. Falls back
    to LLM-run tool_exec_s, which measures the agent's whole tool session
    (including exploration and retries) and so overestimates a replay -- fine
    for ORDERING, which is all it is used for.

    peak_footprint_mb is preferred over peak_rss_mb: RSS under-reports by up to
    5x (mimalloc-retained + IOAccelerator-tagged pages, issue #419), and sizing
    containers off RSS would under-provision the heavy boards.
    """
    table: dict[str, dict] = {}
    for summary in sorted(stress_dir.glob("ab_*/*/summary.json")):
        try:
            rows = json.loads(summary.read_text())
        except Exception:
            continue
        for r in rows if isinstance(rows, list) else []:
            b = r.get("board")
            sec = r.get("total_seconds")
            if not b or not isinstance(sec, (int, float)) or sec <= 0:
                continue
            peak = r.get("peak_footprint_mb") or r.get("peak_rss_mb") or 0.0
            prev = table.get(b)
            # keep the WORST observed cost -- under-ordering a monster is the
            # expensive mistake, over-ordering a cheap board costs nothing
            if not prev or sec > prev["seconds"]:
                table[b] = {"seconds": float(sec), "peak_mb": float(peak), "src": "replay"}
    for res in sorted(stress_dir.glob("results_set*/*.json")):
        try:
            d = json.loads(res.read_text())
        except Exception:
            continue
        b = d.get("board")
        sec = (d.get("timing") or {}).get("tool_exec_s")
        if not b or b in table or not isinstance(sec, (int, float)) or sec <= 0:
            continue
        table[b] = {"seconds": float(sec), "peak_mb": 0.0, "src": "llm"}
    return table


# A board with no recorded memory but a long recorded runtime gets the big tier
# anyway: LLM-sourced rows carry no peak at all, so the 8-hour monsters would
# otherwise land in the 4 GB tier and OOM -- losing the whole board, which is the
# expensive failure. Over-provisioning a few boards costs almost nothing.
LONG_BOARD_S = 30 * 60


def memory_mb_for(board: str, costs: dict) -> int:
    """Everything rides the small tier (see the tier note above: reserving
    for predicted need measurably COSTS money under max(reserved, used)
    billing and burst covers real peaks). The big tier is opt-in OOM
    insurance for boards that proved flaky under burst."""
    import os
    big = {b.strip() for b in os.environ.get("KICAD_SWEEP_BIG_BOARDS",
                                             "").split(",") if b.strip()}
    return BIG_MEM_MB if board in big else DEFAULT_MEM_MB


def build_tasks(arms: list, boards: list, costs: dict) -> list:
    """Cartesian product of arms x boards, ordered longest-first.

    Returned tasks are dicts so they serialize cleanly to Modal.
    """
    tasks = []
    for arm in arms:
        for set_name, board in boards:
            c = costs.get(board) or {}
            tasks.append({
                "arm": arm["name"],
                "arm_spec": arm,
                "set": set_name,
                "board": board,
                "est_seconds": c.get("seconds", DEFAULT_COST_S),
                "memory_mb": memory_mb_for(board, costs),
            })
    tasks.sort(key=lambda t: -t["est_seconds"])
    return tasks


def run_dirs_for(stress_dir: Path, set_name: str) -> list:
    """Every run dir belonging to `set_name`, newest first.

    A set's runs do NOT always live in `runs_<set>`: sets 1-5 were recorded by a
    later wave into `runs_set1_llm0801` etc., while sets 6-10 use the plain name.
    Matching only the exact name silently finds half the corpus (this cost a
    debugging round: 75 of 150 boards, and the wall-clock floor came out wrong
    because the longest board was in a missed set).

    Matching is EXACT or exact-plus-underscore -- never a bare prefix, or
    `runs_set1*` would swallow runs_set10 through runs_set19.
    """
    want = f"runs_{set_name}"
    dirs = [d for d in stress_dir.glob("runs_*")
            if d.is_dir() and (d.name == want or d.name.startswith(want + "_"))]
    return sorted(dirs, key=lambda d: (d.name != want, -d.stat().st_mtime))


def discover_boards(stress_dir: Path, sets: list) -> list:
    """[(set_name, board)] for every board with a replayable manifest.

    A board without redo_commands.sh cannot be replayed -- skipped here rather
    than failing once per arm downstream. A board present in several run dirs is
    taken from the preferred one only (see run_dirs_for ordering), so it is never
    swept twice under the same arm.
    """
    out, seen = [], set()
    for s in sets:
        for run_dir in run_dirs_for(stress_dir, s):
            for d in sorted(run_dir.iterdir()):
                if not d.is_dir() or d.name.startswith(".") or d.name in seen:
                    continue
                if (d / "redo_commands.sh").exists():
                    seen.add(d.name)
                    out.append((run_dir.name[len("runs_"):], d.name))
    return out


def summarize(results: list) -> dict:
    """Fold per-(arm,board) rows into a per-arm comparison table."""
    by_arm: dict[str, list] = {}
    for r in results:
        if r:
            by_arm.setdefault(r.get("arm", "?"), []).append(r)
    table = []
    for arm, rows in sorted(by_arm.items()):
        ok = [r for r in rows if r.get("chain_complete")]
        graded = [r for r in ok if isinstance(r.get("completion_pct"), (int, float))]
        drc = [r["drc_real"] for r in ok if isinstance(r.get("drc_real"), (int, float))]
        table.append({
            "arm": arm,
            "boards": len(rows),
            "chain_complete": len(ok),
            "mean_completion_pct": round(sum(r["completion_pct"] for r in graded) / len(graded), 2)
                                   if graded else None,
            "total_drc_real": sum(drc) if drc else None,
            "cpu_hours": round(sum(r.get("total_seconds", 0.0) for r in rows) / 3600, 2),
            "failed": len(rows) - len(ok),
        })
    return {"arms": table, "n_results": len(results)}
