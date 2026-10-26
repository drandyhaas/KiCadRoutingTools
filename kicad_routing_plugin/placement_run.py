"""
KiCad Routing Tools - headless placement-run contracts (Placement tab).

The Placement tab drives Claude Code headless with the /plan-pcb-placement skill,
in its `place` or `full` mode. Unlike the "Ask AI" analysis skills,
those runs WRITE (the groups file, the intent, one placed board per seed,
fast-routed boards, a converge ledger, REPORT.md), take minutes to hours, and
must be observable from the outside while they run.

This module holds everything about such a run that is not wx: the workdir
layout, the instruction/RESULT contracts, and the read-only pollers the GUI's
monitor timer uses to answer "what is it doing right now" (ledger tail, board
artifacts, stage derivation). Intentionally wx-free so tests can import it
headless, mirroring ai_backend.py.
"""

import hashlib
import json
import os
import re
import shutil
import stat
import sys
import tempfile
import time

# ai_backend.DATASHEET_SENTENCE, mirrored: this module is imported flat by
# the tests and as a package by the GUI, and a plugin-only sibling cannot be
# imported both ways (tests/test_plugin_sibling_imports.py).
# tests/test_datasheet_scratch.py holds the two copies equal.
DATASHEET_SENTENCE = (
    "Read datasheets with WebSearch and WebFetch; a PDF datasheet reads "
    "better downloaded (curl) and opened with the Read tool, a few `pages` at "
    "a time, figures included. If you must save a downloaded file, save it "
    "only under $KRT_DATASHEET_DIR, which is deleted when this run ends."
)

# The write-capable allowlist for placement runs. Write/Edit are the reason
# this cannot ride ai_backend.CLAUDE_ALLOWED_TOOLS (read-only by convention);
# the subagent-dispatch tool stays because the skills dispatch verification
# subagents at close-out. A headless -p run cannot answer permission prompts,
# so anything the skill needs must be listed here (a refused tool errors
# visibly in the transcript).
#
# BOTH dispatch spellings (#552): Claude Code renamed the tool, and a
# permission rule matches the canonical name only. This line has said `Task`
# alone since #633; on 2.1.251 the dispatch event carries `"name":"Agent"`, so
# the close-out verification this comment claims may never have been granted.
#
# Monitor, because /plan-pcb-placement's stop rules require watching long jobs
# (never unwatched for more than 20 minutes); without it a headless run can
# only poll with Bash.
#
# WebFetch, so the grouping step can read a part's datasheet rather than a
# search snippet (Andy, 2026-10-15). Downloads go to the run's own datasheet
# folder (ai_backend.datasheet_scratch), which the runner deletes at the end.
PLACEMENT_ALLOWED_TOOLS = (
    "Bash,Read,Glob,Grep,Write,Edit,WebSearch,WebFetch,Agent,Task,TodoWrite,"
    "Monitor")

# The machine-readable completion contract, appended to the instructions.
# Same last-RESULT=-line convention as ai_plan.PLAN_RESULT_SCHEMA, parsed by
# ai_backend.extract_result_line + parse_placement_result below.
PLACEMENT_RESULT_SCHEMA = (
    'RESULT=<compact single-line JSON> with this exact schema: '
    '{"status": "complete"|"residue"|"refused", '
    '"board": "<absolute path to the final board file>", '
    '"report": "<absolute path to REPORT.md, or null>", '
    '"blocking": <final board_score blocking count as an integer, or null>, '
    '"summary": "<one line>"}')

# Tab mode -> (skill, the skill's own mode argument). Both tab modes run the
# one placement skill; the keys also name the run folders (_RUN_DIR_RE).
PLACEMENT_SKILLS = {
    "place": "plan-pcb-placement",
    "place_route": "plan-pcb-placement",
}
PLACEMENT_SKILL_MODES = {
    "place": "place",
    "place_route": "full",
}

# Backends that can drive a placement run today. The tab shows ALL backends
# in its dropdown (so the UI already communicates future harness support) but
# reverts any unsupported pick: the skills must WRITE, and e.g. opencode's
# pcb-analysis agent denies edits. Growing this tuple (plus per-backend
# allowlist handling in build_cmd) is the whole cost of adding a harness.
PLACEMENT_SUPPORTED_BACKENDS = ("claude",)

# The folder beside the board that holds one directory per run (#1057).
RUNS_DIRNAME = "krt_placement"
# `*` ignores the .gitignore itself too, so the whole folder stays out of the
# user's repository. Written once, never over a file the user already has.
_RUNS_GITIGNORE = (
    "# KiCad Routing Tools placement runs: scratch, never version it.\n*\n")
# Per-run record of the board the run was started on: several boards can
# share one folder, and a run's own files do not say which one it served.
RUN_MARKER = ".krt_run.json"
# How many runs of one board the folder keeps, the new run included.
KEEP_RUNS = 3
# Only directories named the way create_workdir names them are ever pruned,
# so nothing else a user keeps in the folder can be touched.
_RUN_DIR_RE = re.compile(
    r"^(\d{8}_\d{6})_(?:%s)(?:_(\d+))?$"
    % "|".join(sorted(PLACEMENT_SKILLS, key=len, reverse=True)))


def create_workdir(board_filename, mode):
    """Create and return the run's working directory.

    Lives next to the board file (krt_placement/<stamp>_<mode>/) so the board
    and REPORT.md survive the session and are easy to find; falls back to the
    system temp dir when the board has no on-disk file yet. The folder gets a
    .gitignore and the run a RUN_MARKER naming its board, which is what lets
    prune_runs keep the folder bounded (#1057).
    """
    base = None
    if board_filename:
        d = os.path.dirname(os.path.abspath(board_filename))
        if os.path.isdir(d):
            base = d
    if base is None:
        base = tempfile.gettempdir()
    root = os.path.join(base, RUNS_DIRNAME)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    # exist_ok=False + suffix retry: a same-second restart must NOT reuse a
    # dirty workdir (its leftover final.kicad_pcb/REPORT.md would be
    # scavenged as the new run's outputs).
    for n in range(1, 100):
        suffix = "" if n == 1 else f"_{n}"
        workdir = os.path.join(root, f"{stamp}_{mode}{suffix}")
        try:
            os.makedirs(workdir, exist_ok=False)
        except FileExistsError:
            continue
        _ensure_gitignore(root)
        _write_run_marker(workdir, board_filename, mode)
        return workdir
    raise OSError(f"could not create a fresh workdir under {base}")


def _ensure_gitignore(root):
    path = os.path.join(root, ".gitignore")
    if os.path.lexists(path):
        return
    try:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(_RUNS_GITIGNORE)
    except OSError:
        pass  # a read-only project dir still gets its run


def _write_run_marker(workdir, board_filename, mode):
    # Best effort: a run without a marker is attributed to no board, and
    # prune_runs never deletes an unattributed run.
    try:
        with open(os.path.join(workdir, RUN_MARKER), "w",
                  encoding="utf-8") as f:
            json.dump({
                "board": (os.path.abspath(board_filename)
                          if board_filename else None),
                "mode": mode,
                "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }, f)
    except OSError:
        pass


def _board_key(path):
    return os.path.normcase(os.path.abspath(path)) if path else None


def _run_board(run_dir, sole_board):
    """(attributed, board key) for one run directory.

    A run from before RUN_MARKER existed names no board. It is attributed to
    the folder's board when that board is the ONLY one beside the folder --
    it cannot have served another -- and otherwise stays unattributed.
    """
    try:
        with open(os.path.join(run_dir, RUN_MARKER), encoding="utf-8") as f:
            return True, _board_key(json.load(f).get("board"))
    except FileNotFoundError:
        pass
    except (OSError, ValueError, AttributeError):
        return False, None      # unreadable marker: leave the run alone
    if sole_board is None:
        return False, None
    return True, sole_board


def _sole_board(board_dir):
    """The one .kicad_pcb in board_dir (KiCad autosaves excluded), or None."""
    try:
        boards = [n for n in os.listdir(board_dir)
                  if n.endswith(".kicad_pcb") and not n.startswith("_autosave-")
                  and os.path.isfile(os.path.join(board_dir, n))]
    except OSError:
        return None
    return _board_key(os.path.join(board_dir, boards[0])) \
        if len(boards) == 1 else None


def _is_link(path):
    isjunction = getattr(os.path, "isjunction", None)   # Python 3.12+
    return os.path.islink(path) or bool(isjunction and isjunction(path))


def _tree_bytes(path):
    total = 0
    for dirpath, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.lstat(os.path.join(dirpath, name)).st_size
            except OSError:
                pass
    return total


def _remove_tree(path):
    """rmtree that clears read-only bits; True when the tree is gone."""
    def _retry(func, p, _exc):
        if func not in (os.unlink, os.remove, os.rmdir):
            return
        try:
            os.chmod(p, stat.S_IWRITE)
            func(p)
        except OSError:
            pass    # judged below by whether the tree is still there
    if sys.version_info >= (3, 12):
        shutil.rmtree(path, onexc=_retry)
    else:
        shutil.rmtree(path, onerror=_retry)
    return not os.path.lexists(path)


def prune_runs(workdir, board_filename, keep):
    """Delete this board's oldest run directories beyond `keep` (#1057).

    `workdir` is the run just created by create_workdir: it counts as one of
    the `keep` and is never deleted. keep <= 0 keeps every run. Only this
    board's runs are candidates (see _run_board); another board's runs in the
    same folder, unattributed runs, links and anything not named like a run
    are left alone.

    Returns {"root", "pruned": [(path, bytes)], "failed": [path],
    "kept": [(path, bytes)], "unattributed": int}.
    """
    root = os.path.dirname(os.path.abspath(workdir))
    result = {"root": root, "pruned": [], "failed": [], "kept": [],
              "unattributed": 0}
    if keep <= 0 or os.path.basename(root) != RUNS_DIRNAME:
        return result
    me = _board_key(board_filename)
    sole = _sole_board(os.path.dirname(root))
    this_run = os.path.normcase(os.path.abspath(workdir))
    runs = []
    try:
        names = os.listdir(root)
    except OSError:
        return result
    for name in names:
        m = _RUN_DIR_RE.match(name)
        path = os.path.join(root, name)
        if (m is None or os.path.normcase(path) == this_run
                or _is_link(path) or not os.path.isdir(path)):
            continue
        attributed, board = _run_board(path, sole)
        if not attributed:
            result["unattributed"] += 1
        elif board == me:
            runs.append(((m.group(1), int(m.group(2) or 1)), path))
    runs.sort(reverse=True)                     # newest first
    keep_others = [p for _k, p in runs[:keep - 1]]
    for path in [workdir] + keep_others:
        result["kept"].append((path, _tree_bytes(path)))
    for _key, path in runs[keep - 1:]:
        size = _tree_bytes(path)
        if _remove_tree(path):
            result["pruned"].append((path, size))
        else:
            # rmtree may have taken the marker before failing on an open
            # file; without it the next run could not retry this one.
            if not os.path.exists(os.path.join(path, RUN_MARKER)):
                _write_run_marker(path, board_filename, None)
            result["failed"].append(path)
    return result


def _fmt_bytes(n):
    if n >= 1024 ** 3:
        return f"{n / 1024 ** 3:.1f} GB"
    if n >= 1024 ** 2:
        return f"{n / 1024 ** 2:.1f} MB"
    return f"{n / 1024:.0f} KB"


def format_prune_note(result):
    """prune_runs' result as log lines ('' when there is nothing to say)."""
    lines = []
    root = result["root"]
    if result["pruned"]:
        kept = result["kept"]
        lines.append(
            f"Placement: removed {len(result['pruned'])} older run(s) of this "
            f"board from {root} "
            f"({_fmt_bytes(sum(b for _p, b in result['pruned']))} freed); "
            f"keeping the newest {len(kept)} "
            f"({_fmt_bytes(sum(b for _p, b in kept))}).")
    if result["failed"]:
        lines.append(
            f"Placement: could not fully remove {len(result['failed'])} old "
            f"run folder(s) (a file may be open); retried at the next run: "
            + ", ".join(result["failed"]))
    if result["unattributed"]:
        lines.append(
            f"Placement: {result['unattributed']} run folder(s) in {root} "
            f"name no board, so they are never pruned; delete them by hand "
            f"if unneeded.")
    return "".join(line + "\n" for line in lines)


def stage_inputs(workdir, snapshot_path, board_filename):
    """Copy the board snapshot (and the real board's project siblings) into
    the workdir, so the whole run needs exactly one --add-dir.

    The snapshot comes from pcbnew.SaveBoard into a temp file, which strands
    the sibling .kicad_pro/.kicad_dru (the DRC floor and per-layer clearance
    rules the route steps auto-read - see CLAUDE.md #441/#498), so those are
    taken from the real board file's directory when present.
    """
    staged = os.path.join(workdir, "input.kicad_pcb")
    shutil.copyfile(snapshot_path, staged)
    if board_filename:
        stem = os.path.splitext(os.path.abspath(board_filename))[0]
        # GUARDED, like route.py's own fallback: this module runs inside
        # KiCad's plugin loader, which does not put py_router on sys.path.
        # An unconditional import here raised ModuleNotFoundError and took
        # `stage_inputs` with it -- caught by tests/test_placement_run.py.
        try:
            from copy_board import SIBLING_EXTS
        except Exception:                                  # noqa: BLE001
            SIBLING_EXTS = (".kicad_pro", ".kicad_prl", ".kicad_dru",
                            ".design-brief.json")
        for ext in SIBLING_EXTS:
            sibling = stem + ext
            if os.path.isfile(sibling):
                shutil.copyfile(sibling, os.path.join(workdir, "input" + ext))
        # mechanical.json is a DIRECTORY file, not a stem sibling:
        # reconcile.discover_mechanical reads <board dir>/mechanical.json, and
        # the run's board is <workdir>/input.kicad_pcb. Unstaged, every
        # declared mechanical fact beside the user's board was invisible to a
        # run launched from this tab, while the same skill run in place saw it.
        try:
            from placement.reconcile import MECHANICAL_NAME
        except Exception:                                  # noqa: BLE001
            MECHANICAL_NAME = "mechanical.json"
        mech = os.path.join(os.path.dirname(stem), MECHANICAL_NAME)
        if os.path.isfile(mech):
            shutil.copyfile(mech, os.path.join(workdir, MECHANICAL_NAME))
        _stage_schematic(workdir, board_filename)
    return staged


def _stage_schematic(workdir, board_filename):
    """The project's schematic tree beside input.kicad_pcb: the root as
    input.kicad_sch (what board_context looks for beside the board) and each
    sub-sheet at its path relative to the root, so the hierarchy resolves as
    it did. Without it a run from this tab read no sheets and no drawn
    blocks, while the same skill run on the board in place read both. A
    sheet file outside the root's folder is not staged, and board_context
    names it as not found."""
    try:
        from placement.schematic import discover, tree_files
    except Exception:                                      # noqa: BLE001
        return
    try:
        root, _looked = discover(board_filename)
        if root is None:
            return
        inside, _outside = tree_files(root)
        for path, rel in inside:
            same = os.path.normcase(os.path.abspath(path)) == \
                os.path.normcase(os.path.abspath(root))
            dst = os.path.join(workdir, "input.kicad_sch" if same else rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(path, dst)
    except Exception as exc:                               # noqa: BLE001
        # An unreadable schematic must not stop the run; board_context then
        # reports none beside the board. Said on KiCad's console, not lost.
        print(f"Placement: schematic not staged ({type(exc).__name__}: "
              f"{exc})", file=sys.stderr)


def _fwd(path):
    """Forward-slash a path for prompt text (Windows-safe, shell-safe)."""
    return os.path.abspath(path).replace("\\", "/")


def build_placement_instructions(workdir, mode, extra=""):
    """The instruction text handed to the skill prompt (after the board arg)."""
    wd = _fwd(workdir)
    lines = [
        "Work headless and unattended; never wait for user input.",
        f"Use {wd} as the working directory for ALL artifacts (groups.json, "
        "the intent, scores, renders, REPORT.md). Keep the seeds in "
        f"{wd}/ with the skill's names (s<S>.kicad_pcb, s<S>.json, "
        "s<S>_route/, s<S>_fast.kicad_pcb, s<S>_fast.score.json): this GUI's "
        "candidates table reads them. Keep the converge ledger at "
        f"{wd}/ledger.jsonl.",
        f"Do not modify any file outside {wd}, and never edit "
        "input.kicad_pcb in place.",
        DATASHEET_SENTENCE,
        # #552 item 2. This run's allowlist is the write-capable one, so it is
        # the run whose CHILDREN can actually damage something -- and the
        # workdir pin above was parent-only. A subagent inherits the tools and
        # nothing else, so the pin has to be restated INTO the child, the same
        # way the skills' own <subagent_prompt> blocks are copied verbatim.
        f"If you dispatch a subagent, copy this sentence into its prompt "
        f"verbatim: it must not modify any file outside {wd} (a downloaded "
        f"datasheet may go under $KRT_DATASHEET_DIR), must never edit "
        f"input.kicad_pcb in place, and must answer with a line beginning "
        f"VERDICT= (never RESULT=, which this GUI reads as the run's own "
        f"result line).",
        f"At close-out write the final board to {wd}/final.kicad_pcb (with "
        f"sibling final.kicad_pro) and the report to {wd}/REPORT.md.",
        "If a gate refuses because the board already carries routed copper, "
        'stop and report status "refused" instead of stripping copper. On a '
        "pile, check_floorplan's unplaced gate is not a refusal of this run: "
        "pass it --allow-unplaced, as the skill says.",
        # The GUI's progress line names the newest tool command and the
        # newest ledger row, so the milestone rows are what the user sees as
        # the run's progress between commands.
        "Record every milestone board in that ledger as you go (the skill's "
        "`converge.py record` step): this GUI shows the newest row as the "
        "run's progress.",
    ]
    if extra and extra.strip():
        lines.append(extra.strip())
    lines.append("After the report, end your reply with exactly one line of "
                 "the form " + PLACEMENT_RESULT_SCHEMA)
    return " ".join(lines)


def build_placement_prompt(backend, workdir, staged_board, mode, extra=""):
    """The full skill prompt for a placement run (via backend.skill_prompt)."""
    args = f"{PLACEMENT_SKILL_MODES[mode]} {_fwd(staged_board)}"
    return backend.skill_prompt(
        PLACEMENT_SKILLS[mode], args,
        build_placement_instructions(workdir, mode, extra))


def _norm_existing(path):
    """Normalize a path from the RESULT JSON; return it if it exists."""
    if not path:
        return None
    p = os.path.normpath(str(path).replace("\\", "/").replace("/", os.sep))
    return p if os.path.isfile(p) else None


def parse_placement_result(value):
    """Parse the RESULT= JSON payload -> (outputs dict | None, [errors]).

    outputs: {status, board, report, blocking, summary}. A missing or
    non-existing board path is fatal for complete/residue (the caller falls
    back to scan_workdir_outputs); report problems are non-fatal.
    """
    errors = []
    if not value:
        return None, ["no RESULT= line in the agent's reply"]
    try:
        data = json.loads(value)
    except (json.JSONDecodeError, ValueError) as e:
        return None, [f"RESULT= payload is not valid JSON: {e}"]
    if not isinstance(data, dict):
        return None, ["RESULT= payload is not a JSON object"]
    status = data.get("status")
    if status not in ("complete", "residue", "refused"):
        return None, [f"RESULT= status {status!r} not in complete/residue/refused"]
    board = _norm_existing(data.get("board"))
    if board is None and status != "refused":
        return None, [f"RESULT= board path missing or not found: {data.get('board')!r}"]
    report = _norm_existing(data.get("report"))
    if data.get("report") and report is None:
        errors.append(f"report path not found: {data.get('report')!r}")
    blocking = data.get("blocking")
    # `isinstance(True, int)` holds, so the bool test is not redundant: a
    # `false` or a `-1` used to pass validation and reach the status line as
    # "blocking False" (#1075).
    if blocking is not None and (isinstance(blocking, bool)
                                 or not isinstance(blocking, int)
                                 or blocking < 0):
        errors.append(f"blocking is not a non-negative integer: {blocking!r}")
        blocking = None
    return {
        "status": status,
        "board": board,
        "report": report,
        "blocking": blocking,
        "summary": str(data.get("summary", "")),
    }, errors


def list_board_artifacts(workdir):
    """Top-level *.kicad_pcb files -> [(path, mtime, size)], oldest first.

    Not recursive: a seed's fast-route working files live in s<S>_route/,
    and the converge boards/<sha> store churns too much to look at.
    """
    out = []
    try:
        names = os.listdir(workdir)
    except OSError:
        return out
    for name in names:
        if not name.endswith(".kicad_pcb"):
            continue
        path = os.path.join(workdir, name)
        try:
            st = os.stat(path)
        except OSError:
            continue  # vanished mid-scan
        out.append((path, st.st_mtime, st.st_size))
    out.sort(key=lambda t: t[1])
    return out


def newest_stable_board(artifacts, prev_artifacts):
    """The newest board whose size is unchanged since the previous scan.

    A board still being written grows between ticks; requiring two scans at
    the same size keeps the preview renderer off torn files. prev_artifacts
    is the previous call's list (or None on the first tick).
    """
    prev_sizes = {p: s for p, _m, s in (prev_artifacts or [])}
    for path, _mtime, size in reversed(artifacts):
        if size > 0 and prev_sizes.get(path) == size:
            return path
    return None


# ------------------------------------------------------------ candidates
#
# The skill keeps the seeds in <workdir> (SKILL.md section 2):
#   s<S>.kicad_pcb + s<S>.json     seed S's placement and place_simple report
#   s<S>_route/                    the fast route's working files
#   s<S>_fast.kicad_pcb            the fast-routed board
#   s<S>_fast.score.json           its quick_score (blocking, quality{...})
# and writes the kept board to <workdir>/final.kicad_pcb.

_SEED_FILE_RE = re.compile(r"^s(\d+)(?:\.|_)")
_FAST_BOARD_RE = re.compile(r"^s\d+_fast\.kicad_pcb$")    # a --quick board: never shipped


def _read_score(path):
    """(blocking, vias, copper_mm, segments, floorplan, joins) from a
    quick_score / board_score JSON, or None. `floorplan` is the placement's
    own defects already summed into `blocking`: `blocking_by.floorplan` (the
    check_floorplan errors) plus `blocking_by.assembly` (check_assembly's
    count), 0 when the score graded neither; `joins` is
    `joins_needed`, the joins the open nets still need (0 at blocking 0, None
    when an older score does not say)."""
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
        q = doc.get("quality") or {}
        by = doc.get("blocking_by") or {}
        # the placement's own defects, as quick_score.judge_key reads them:
        # floorplan errors plus check_assembly's count (both 0 when ungraded)
        fp = None
        if by.get("floorplan") is not None or by.get("assembly") is not None:
            fp = int(by.get("floorplan") or 0) + int(by.get("assembly") or 0)
        blocking = int(doc["blocking"])
        joins = doc.get("joins_needed")
        return (blocking, int(q["vias"]), float(q["copper_mm"]),
                int(q["segments"]), int(fp) if fp is not None else 0,
                int(joins) if joins is not None else (0 if blocking == 0 else None))
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return None


def candidate_key(row):
    """How the skill ranks candidates (`quick_score.judge_key`): the
    floorplan errors first -- a seed check_floorplan grades with an error
    cannot be DONE as placed, so every seed that grades clean ranks above it
    (#1296) -- then blocking, then the joins the open nets still need (a
    missing 17-pad rail is 16 joins, one stranded pad 1: by vias the
    rail-less board won, #1340), then vias, copper, segments."""
    joins = row.get("joins")
    return (row.get("floorplan") or 0, row["blocking"],
            float("inf") if joins is None else joins,
            row["vias"], row["copper_mm"], row["segments"])


def scan_candidates(workdir, active=None):
    """One row per seed, from the seeds' files in <workdir> -> [row], by seed.

    row: {seed, placed, fast, blocking, vias, copper_mm, segments, floorplan,
    joins, status, mtime}. `placed`/`fast` are the board paths or None; the
    metrics are None until the fast route is scored (`floorplan`: the
    check_floorplan errors inside `blocking`, 0 when the score graded none;
    `joins`: the joins the open nets still need). status: "placing" (no finished placement
    yet), "routing" (placed, no score yet), "not judged" (placed, and not
    among the seeds `judge.json` names for the skill to fast-route),
    "illegal" (its report lists illegal parts, so it is never judged),
    "routed" (scored, with blocking nets), "clean" (scored, blocking 0),
    "best" (the best scored row). `active` = the seeds the run is placing
    right now (`active_candidates`), shown as "placing" before their boards
    exist.
    """
    rows = {}
    try:
        files = os.listdir(workdir)
    except OSError:
        files = []
    for s in sorted({int(sm.group(1)) for sm in map(_SEED_FILE_RE.match, files) if sm}):
        def p(suffix):
            path = os.path.join(workdir, f"s{s}{suffix}")
            return path if os.path.isfile(path) else None
        placed, report, fast = p(".kicad_pcb"), p(".json"), p("_fast.kicad_pcb")
        score_path = p("_fast.score.json")
        if not (placed or report or fast or score_path):
            continue                       # only a seed's route folder
        score = _read_score(score_path) if score_path else None
        mtime = max((_mtime(x) for x in (placed, report, fast, score_path) if x),
                    default=0.0)
        row = {"seed": s, "placed": placed, "fast": fast,
               "blocking": None, "vias": None, "copper_mm": None,
               "segments": None, "floorplan": None, "joins": None,
               "mtime": mtime}
        if score is not None:
            (row["blocking"], row["vias"], row["copper_mm"],
             row["segments"], row["floorplan"], row["joins"]) = score
            row["status"] = "clean" if score[0] == 0 else "routed"
        elif placed and report:
            row["status"] = "routing"
        else:
            row["status"] = "placing"
        rows[s] = row
    for s in active or ():
        if s not in rows:
            rows[s] = {"seed": s, "placed": None, "fast": None,
                       "blocking": None, "vias": None, "copper_mm": None,
                       "segments": None, "floorplan": None, "joins": None,
                       "status": "placing", "mtime": time.time()}
    out = [rows[k] for k in sorted(rows)]
    _mark_unjudged(out)
    scored = [r for r in out if r["blocking"] is not None]
    if scored:
        min(scored, key=candidate_key)["status"] = "best"
    return out


def _seed_defaults():
    """place_simple's default seeds: (scattered, clustered, judged) -- the
    seeds of each start it places for an OUT carrying {seed} with no seed
    named, and how many legal seeds of lowest `final.cost` it names for a
    judging route. place_simple is the
    authority; the import is guarded like `SIBLING_EXTS` above (KiCad's plugin
    loader does not put py_placer on sys.path) and the fallback is a copy."""
    try:
        from place_simple import CLUSTERED_SEEDS, JUDGED_SEEDS, SCATTERED_SEEDS
        return SCATTERED_SEEDS, CLUSTERED_SEEDS, JUDGED_SEEDS
    except Exception:                                      # noqa: BLE001
        return 6, 6, 6


def _named_judged(rows):
    """The seeds `judge.json` names (place_simple writes it beside the seeds'
    boards once every seed is placed), else None."""
    for r in rows:
        if r["placed"]:
            try:
                with open(os.path.join(os.path.dirname(r["placed"]), "judge.json"),
                          encoding="utf-8") as fh:
                    doc = json.load(fh)
                return {int(s) for s in doc.get("judge") or ()}
            except (OSError, ValueError, TypeError, AttributeError):
                return None
    return None


def _mark_unjudged(rows):
    """Re-label the placed, unscored rows that the skill will not judge:
    "illegal" when the seed's report lists illegal parts, "not judged" when
    `judge.json` does not name it -- or, without one, when place_simple's
    default number of legal seeds of lower `final.cost` exist. Until then,
    every unscored row stays "routing"."""
    legal = []                         # (final.cost, row) of every legal seed
    for r in rows:
        if not r["placed"] or r["status"] not in ("routing", "routed", "clean"):
            continue
        try:
            with open(os.path.splitext(r["placed"])[0] + ".json", encoding="utf-8") as fh:
                rep = json.load(fh)
        except (OSError, ValueError):
            continue
        if not isinstance(rep, dict):
            continue
        if rep.get("illegal"):
            if r["status"] == "routing":
                r["status"] = "illegal"
            continue
        cost = (rep.get("final") or {}).get("cost")
        if isinstance(cost, (int, float)):
            legal.append((cost, r))
    named = _named_judged(rows)
    if named is not None:
        for _cost, r in legal:
            if r["status"] == "routing" and r["seed"] not in named:
                r["status"] = "not judged"
        return
    judged = _seed_defaults()[2]
    if len(legal) < judged:
        return
    legal.sort(key=lambda t: t[0])
    for _cost, r in legal[judged:]:
        if r["status"] == "routing":
            r["status"] = "not judged"


def _mtime(path):
    try:
        return os.path.getmtime(path)
    except OSError:
        return 0.0


def _sha256(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
    except OSError:
        return None
    return h.hexdigest()


def mark_kept(rows, workdir, board=None):
    """Mark the candidate the run kept as "KEPT" and return it (or None).

    The kept board (`board`, else <workdir>/final.kicad_pcb) is matched to a
    candidate by the sha256 of its placement or fast-routed board. A kept
    board that is neither -- touched up with place_pose, or routed in full --
    falls back to the best scored row, the one the skill's ranking keeps.
    """
    target = board if board and os.path.isfile(board) else \
        os.path.join(workdir, "final.kicad_pcb")
    kept = None
    want = _sha256(target) if os.path.isfile(target) else None
    if want:
        for row in rows:
            if any(p and _sha256(p) == want for p in (row["placed"], row["fast"])):
                kept = row
    if kept is None:
        scored = [r for r in rows if r["blocking"] is not None]
        kept = min(scored, key=candidate_key) if scored else None
    if kept is not None:
        kept["status"] = "KEPT"
    return kept


_ACTIVE_RE = re.compile(r"(?:^|[\s/\\'\"])s(\d+|\{seed\})\.kicad_pcb\b")
_SEEDS_RE = re.compile(r"--seeds[ =]([\d,\-]+)")


def _int_opt(line, flag, default):
    m = re.search(re.escape(flag) + r"[ =](\d+)", line)
    return int(m.group(1)) if m else default


def _seed_spec(spec):
    """'0-3' / '0,2,5' / '0-2,5' -> [seeds]."""
    out = []
    for part in spec.split(","):
        lo, _, hi = part.partition("-")
        if lo.isdigit() and (not hi or hi.isdigit()):
            out.extend(range(int(lo), int(hi or lo) + 1))
    return out


def active_candidates(transcript_tail):
    """[seed] the newest place_simple command is writing: one seed
    (`s<S>.kicad_pcb`), or every seed of the parallel form
    (`s{seed}.kicad_pcb --seeds 0-3`, or with no seed named the default
    seeds, `--scattered` + `--clustered` + `--anchored` from 0). [] when it
    names no seed board."""
    for line in reversed(list(transcript_tail or ())):
        if "place_simple.py" in line:
            m = _ACTIVE_RE.search(line)
            if not m:
                return []
            if m.group(1).isdigit():
                return [int(m.group(1))]
            seeds = _SEEDS_RE.search(line)
            if seeds:
                return _seed_spec(seeds.group(1))
            one = re.search(r"--seed[ =](\d+)", line)
            if one:
                return [int(one.group(1))]
            sc, cl, _judged = _seed_defaults()
            count = (_int_opt(line, "--scattered", sc) + _int_opt(line, "--clustered", cl)
                     + _int_opt(line, "--anchored", 0))      # anchored: opt-in, none by default
            return list(range(count))
    return []


def newest_run_board(workdir):
    """The newest board of the run, as a path relative to the workdir with
    forward slashes (for derive_stage)."""
    best = (0.0, None)
    for rel in _run_boards(workdir):
        mt = _mtime(os.path.join(workdir, rel))
        if mt >= best[0]:
            best = (mt, rel)
    return best[1]


def _run_boards(workdir):
    try:
        names = os.listdir(workdir)
    except OSError:
        return
    for name in names:
        if name.endswith(".kicad_pcb"):
            yield name


def read_ledger_tail(ledger_path, offset):
    """Incrementally read complete ledger.jsonl rows -> (rows, new_offset).

    Only lines terminated by a newline are consumed, so a torn last line is
    left for the next tick. Unparseable complete lines are skipped (their
    bytes are still consumed).
    """
    rows = []
    try:
        with open(ledger_path, "rb") as f:
            f.seek(offset)
            chunk = f.read()
    except OSError:
        return rows, offset
    if not chunk:
        return rows, offset
    end = chunk.rfind(b"\n")
    if end < 0:
        return rows, offset  # only a torn partial line so far
    for line in chunk[:end].splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line.decode("utf-8", errors="replace"))
        except (json.JSONDecodeError, ValueError):
            continue
        if isinstance(row, dict):
            rows.append(row)
    return rows, offset + end + 1


# What the newest board file says the run reached (newest_run_board names
# it relative to the workdir): the locked input, a seed's placement or fast
# route, the final board.
_ARTIFACT_STAGES = (
    (re.compile(r"^s(\d+)_fast\.kicad_pcb$"), "seed {0} fast-routed"),
    (re.compile(r"^s(\d+)\.kicad_pcb$"), "seed {0} placed"),
    (re.compile(r"^fixed\.kicad_pcb$"), "fixed parts locked"),
    (re.compile(r"^final\.kicad_pcb$"), "final board written"),
    (re.compile(r"route", re.I), "routing"),
    (re.compile(r"(?:^|_)s(\d+)\.kicad_pcb$"), "placed seed {0}"),
)

# What the newest tool command says the run is doing NOW. Matched against the
# transcript tail newest-first; the first command that matches names the
# activity. Order matters: a specific pattern before its general one. `SS`
# is a seed's board or route folder: s<S>.kicad_pcb, s<S>_route, s<S>_fast.
_SS = r"(?:^|[\s/\\'\"])s(\d+)"
_COMMAND_STAGES = (
    (re.compile(r"place_simple\.py\b.*?--seeds[ =]([\d,\-]+)"), "placing seeds {0}"),
    (re.compile(r"place_simple\.py\b(?!.*--seed[ =]\d).*?s\{seed\}"), "placing the seeds"),
    (re.compile(r"place_simple\.py\b.*?" + _SS + r"\.kicad_pcb"), "placing seed {0}"),
    (re.compile(r"place_simple\.py\b.*?--seed[ =](\d+)"), "placing seed {0}"),
    (re.compile(r"place_simple\.py\b"), "placing"),
    (re.compile(r"judge_seeds\.py\b"), "fast-routing the judged seeds"),
    (re.compile(r"(?:quick_score|board_score)\.py\b.*?" + _SS + r"_fast"), "scoring seed {0}"),
    (re.compile(r"route_review\.py\b"), "reviewing the fast route"),
    (re.compile(_SS + r"_(?:route|fast)\b"), "fast-routing seed {0}"),
    (re.compile(r"\broute\.py\b.*--quick\b"), "fast-routing"),
    (re.compile(r"\b(?:route|route_planes|route_diff|bga_fanout|qfn_fanout|"
                r"place_fanout_clearance)\.py\b"), "routing"),
    (re.compile(r"board_score\.py\b"), "scoring"),
    (re.compile(r"check_groups\.py\b"), "checking the groups"),
    (re.compile(r"check_floorplan\.py\b.*--emit-intent"), "writing the intent"),
    (re.compile(r"check_floorplan\.py\b"), "checking the floorplan"),
    (re.compile(r"check_assembly\.py\b"), "checking assembly"),
    (re.compile(r"\b(?:board_brief|board_context)\.py\b"), "reading the board"),
    (re.compile(r"place_pose\.py\b"), "setting poses"),
    (re.compile(r"render_(?:placement|nets)\.py\b"), "rendering"),
    (re.compile(r"converge\.py\s+record\b"), "recording a milestone"),
    (re.compile(r"check_complete\.py\b"), "checking completion"),
)


def _row_label(row):
    """What a ledger row DID, in one line. `converge.row_label` is the AUTHORITY.

    The import is guarded the same way `SIBLING_EXTS` above is, and for the same
    reason: this module runs inside KiCad's plugin loader, which does not put
    py_placer on sys.path. The fallback below is a copy, and a copy is a thing
    that drifts -- `tests/test_904_lens_file_binding.py` compares the two on the
    same rows so it cannot drift silently.

    What it fixes: `ledger_row.get("lever") or "?"` ended the ladder at the
    first field, so an `--exhausted` declaration -- which has `lever: null` by
    construction, its whole content being the reason a person wrote -- rendered
    in the GUI as `lap 31: systemic/?`.
    """
    try:
        from converge import row_label
        return row_label(row)
    except Exception:                                      # noqa: BLE001
        lever = str(row.get("lever") or "").strip()
        if lever:
            return lever
        dec = row.get("exhausted")
        if isinstance(dec, dict) and str(dec.get("reason") or "").strip():
            return "declared exhausted: " + str(dec["reason"]).strip()
        stop = str(row.get("stop_condition") or "").strip()
        if stop:
            return "close-out: " + stop
        return "(no lever recorded)"


def _command_stage(transcript_tail):
    """The activity the newest recognised tool command names, or None."""
    for line in reversed(list(transcript_tail or ())):
        for rx, fmt in _COMMAND_STAGES:
            m = rx.search(line)
            if m:
                return fmt.format(*m.groups())
    return None


def derive_stage(transcript_tail, ledger_row, newest_artifact_name):
    """Best human answer to "what is it doing right now".

    The newest recognised tool command in the transcript says what the run is
    doing now (`placing seed 2`, `fast-routing`); the newest converge ledger
    row says the last milestone it recorded. Both are shown when both exist,
    the activity first. Without either, the newest board file's name answers,
    then a generic fallback.
    """
    activity = _command_stage(transcript_tail)
    milestone = None
    if ledger_row:
        lap = ledger_row.get("iteration")
        kind = ledger_row.get("kind") or "?"
        milestone = f"lap {lap}: {kind}/{_row_label(ledger_row)}"
    if activity and milestone:
        return f"{activity} ({milestone})"
    if activity or milestone:
        return activity or milestone
    if newest_artifact_name:
        for rx, fmt in _ARTIFACT_STAGES:
            m = rx.search(newest_artifact_name)
            if m:
                return fmt.format(*m.groups())
    return "working..."


def scan_workdir_outputs(workdir):
    """Fallback outputs when the RESULT= line is missing or unusable.

    Scavenges what the run left behind: the declared final board (else the
    best scored candidate's placement, else the newest top-level non-input
    board) and REPORT.md. Same dict shape as
    parse_placement_result, with
    status "scavenged" so callers can say so.
    """
    artifacts = [t for t in list_board_artifacts(workdir)
                 if os.path.basename(t[0]) != "input.kicad_pcb"
                 and not _FAST_BOARD_RE.match(os.path.basename(t[0]))]
    board = os.path.join(workdir, "final.kicad_pcb")
    if not os.path.isfile(board):
        # The best scored candidate's PLACEMENT (never its --quick board),
        # else the newest top-level board. No newer scan to compare sizes
        # against - the run is over, so any file is as stable as it gets.
        scored = [r for r in scan_candidates(workdir)
                  if r["blocking"] is not None and r["placed"]]
        board = (min(scored, key=candidate_key)["placed"] if scored
                 else newest_stable_board(artifacts, artifacts))
    report = os.path.join(workdir, "REPORT.md")
    if not os.path.isfile(report):
        report = None
    return {
        "status": "scavenged",
        "board": board,
        "report": report,
        "blocking": None,
        "summary": "",
    }
