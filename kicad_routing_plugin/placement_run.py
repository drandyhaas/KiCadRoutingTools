"""
KiCad Routing Tools - headless placement-run contracts (Placement tab).

The Placement tab drives Claude Code headless with the /plan-pcb-placement or
/plan-pcb-placement-and-routing skill. Unlike the "Ask AI" analysis skills,
those runs WRITE (lap boards, a converge ledger, REPORT.md, the movie), take
minutes to hours, and must be observable from the outside while they run.

This module holds everything about such a run that is not wx: the workdir
layout, the instruction/RESULT contracts, and the read-only pollers the GUI's
monitor timer uses to answer "what is it doing right now" (ledger tail, board
artifacts, stage derivation). Intentionally wx-free so tests can import it
headless, mirroring ai_backend.py.
"""

import json
import os
import re
import shutil
import tempfile
import time

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
PLACEMENT_ALLOWED_TOOLS = (
    "Bash,Read,Glob,Grep,Write,Edit,WebSearch,Agent,Task,TodoWrite")

# The machine-readable completion contract, appended to the instructions.
# Same last-RESULT=-line convention as ai_plan.PLAN_RESULT_SCHEMA, parsed by
# ai_backend.extract_result_line + parse_placement_result below.
PLACEMENT_RESULT_SCHEMA = (
    'RESULT=<compact single-line JSON> with this exact schema: '
    '{"status": "complete"|"residue"|"refused", '
    '"board": "<absolute path to the final board file>", '
    '"movie": "<absolute path to the movie, or null>", '
    '"report": "<absolute path to REPORT.md, or null>", '
    '"blocking": <final board_score blocking count as an integer, or null>, '
    '"summary": "<one line>"}')

PLACEMENT_SKILLS = {
    "place": "plan-pcb-placement",
    "place_route": "plan-pcb-placement-and-routing",
}

# Backends that can drive a placement run today. The tab shows ALL backends
# in its dropdown (so the UI already communicates future harness support) but
# reverts any unsupported pick: the skills must WRITE, and e.g. opencode's
# pcb-analysis agent denies edits. Growing this tuple (plus per-backend
# allowlist handling in build_cmd) is the whole cost of adding a harness.
PLACEMENT_SUPPORTED_BACKENDS = ("claude",)

# Driver stage ids -> human progress text ("which type of work"), from the two
# skills' driver --list output (placement_driver.py P*, loop_driver.py L*).
STAGE_LABELS = {
    "P0": "P0 gate: should placement be touched",
    "P1": "P1 seeding an unplaced board",
    "P2": "P2 locking mechanical parts",
    "P3": "P3 reconstructing the placement",
    "P4": "P4 fix loop: measure/change/verify",
    "P5": "P5 arrangement slate",
    "P6": "P6 floorplan intent",
    "P-close": "P-close: close-out + film",
    "L1": "L1 placing",
    "L2": "L2 freeze + route",
    "L3": "L3 classifying the failure",
    "L4": "L4 re-entering at the named point",
    "L5": "L5 close-out",
}

# Tolerates --stage P4 / --stage=P4 / --stage "P4" spellings.
_STAGE_RE = re.compile(r"--stage[=\s]+[\"']?(P-close|P[0-6]|L[1-5])\b")


def create_workdir(board_filename, mode):
    """Create and return the run's working directory.

    Lives next to the board file (krt_placement/<stamp>_<mode>/) so the movie
    and REPORT.md survive the session and are easy to find; falls back to the
    system temp dir when the board has no on-disk file yet.
    """
    base = None
    if board_filename:
        d = os.path.dirname(os.path.abspath(board_filename))
        if os.path.isdir(d):
            base = d
    if base is None:
        base = tempfile.gettempdir()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    # exist_ok=False + suffix retry: a same-second restart must NOT reuse a
    # dirty workdir (its leftover final.kicad_pcb/REPORT.md would be
    # scavenged as the new run's outputs).
    for n in range(1, 100):
        suffix = "" if n == 1 else f"_{n}"
        workdir = os.path.join(base, "krt_placement", f"{stamp}_{mode}{suffix}")
        try:
            os.makedirs(workdir, exist_ok=False)
            return workdir
        except FileExistsError:
            continue
    raise OSError(f"could not create a fresh workdir under {base}")


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
        from copy_board import SIBLING_EXTS   # ONE list (#711)
        for ext in SIBLING_EXTS:
            sibling = stem + ext
            if os.path.isfile(sibling):
                shutil.copyfile(sibling, os.path.join(workdir, "input" + ext))
    return staged


def _fwd(path):
    """Forward-slash a path for prompt text (Windows-safe, shell-safe)."""
    return os.path.abspath(path).replace("\\", "/")


def build_placement_instructions(workdir, mode, extra=""):
    """The instruction text handed to the skill prompt (after the board arg)."""
    wd = _fwd(workdir)
    lines = [
        "Work headless and unattended; never wait for user input.",
        f"Use {wd} as the working directory for ALL artifacts (ledger, lap "
        "boards, renders, scores, REPORT.md, journal.md, the movie). Keep the "
        f"converge ledger at {wd}/ledger.jsonl.",
        f"Do not modify any file outside {wd}, and never edit "
        "input.kicad_pcb in place.",
        # #552 item 2. This run's allowlist is the write-capable one, so it is
        # the run whose CHILDREN can actually damage something -- and the
        # workdir pin above was parent-only. A subagent inherits the tools and
        # nothing else, so the pin has to be restated INTO the child, the same
        # way the skills' own <subagent_prompt> blocks are copied verbatim.
        f"If you dispatch a subagent, copy this sentence into its prompt "
        f"verbatim: it must not modify any file outside {wd}, must never edit "
        f"input.kicad_pcb in place, and must answer with a line beginning "
        f"VERDICT= (never RESULT=, which this GUI reads as the run's own "
        f"result line).",
        f"At close-out write the final board to {wd}/final.kicad_pcb (with "
        f"sibling final.kicad_pro), the movie to {wd}/placement.mp4 (or .gif "
        f"fallback), and the report to {wd}/REPORT.md.",
        "If a gate refuses (for example the board already carries routed "
        'copper), stop and report status "refused" instead of stripping '
        "copper.",
    ]
    if mode == "place_route":
        lines.append("Run every loop_driver.py stage with --no-delegate "
                     "(single process; this is a headless run).")
    if extra and extra.strip():
        lines.append(extra.strip())
    lines.append("After the report, end your reply with exactly one line of "
                 "the form " + PLACEMENT_RESULT_SCHEMA)
    return " ".join(lines)


def build_placement_prompt(backend, workdir, staged_board, mode, extra=""):
    """The full skill prompt for a placement run (via backend.skill_prompt)."""
    return backend.skill_prompt(
        PLACEMENT_SKILLS[mode], _fwd(staged_board),
        build_placement_instructions(workdir, mode, extra))


def _norm_existing(path):
    """Normalize a path from the RESULT JSON; return it if it exists."""
    if not path:
        return None
    p = os.path.normpath(str(path).replace("\\", "/").replace("/", os.sep))
    return p if os.path.isfile(p) else None


def parse_placement_result(value):
    """Parse the RESULT= JSON payload -> (outputs dict | None, [errors]).

    outputs: {status, board, movie, report, blocking, summary}. A missing or
    non-existing board path is fatal for complete/residue (the caller falls
    back to scan_workdir_outputs); movie/report problems are non-fatal.
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
    movie = _norm_existing(data.get("movie"))
    if data.get("movie") and movie is None:
        errors.append(f"movie path not found: {data.get('movie')!r}")
    report = _norm_existing(data.get("report"))
    if data.get("report") and report is None:
        errors.append(f"report path not found: {data.get('report')!r}")
    blocking = data.get("blocking")
    if blocking is not None and not isinstance(blocking, int):
        errors.append(f"blocking is not an integer: {blocking!r}")
        blocking = None
    return {
        "status": status,
        "board": board,
        "movie": movie,
        "report": report,
        "blocking": blocking,
        "summary": str(data.get("summary", "")),
    }, errors


def list_board_artifacts(workdir):
    """Top-level *.kicad_pcb files -> [(path, mtime, size)], oldest first.

    Deliberately not recursive: the converge boards/<sha> CAS store and
    attempt archives churn far too much to preview; the interesting boards
    (input, lap*, loop_round*, placed, final) land at the top level.
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


_ARTIFACT_STAGES = (
    (re.compile(r"loop_round(\d+)_route\.log$"), "routing round {0}"),
    (re.compile(r"loop_round(\d+)_routed\.kicad_pcb$"), "routing round {0}"),
    (re.compile(r"loop_round(\d+)\.kicad_pcb$"), "placing round {0}"),
    (re.compile(r"render_lap(\d+)"), "rendering lap {0}"),
    (re.compile(r"^score.*\.json$"), "scoring the board"),
    (re.compile(r"^lap(\d+)\.kicad_pcb$"), "lap {0}"),
)


def derive_stage(transcript_tail, ledger_row, newest_artifact_name):
    """Best human answer to "what is it doing right now".

    Priority: the last driver --stage the agent invoked (the drivers are the
    skills' tape heads, so this is authoritative when present) -> the newest
    converge ledger row -> artifact-name heuristics -> a generic fallback.
    """
    for line in reversed(list(transcript_tail or ())):
        m = _STAGE_RE.search(line)
        if m:
            return STAGE_LABELS.get(m.group(1), m.group(1))
    if ledger_row:
        lap = ledger_row.get("iteration")
        kind = ledger_row.get("kind") or "?"
        lever = ledger_row.get("lever") or "?"
        return f"lap {lap}: {kind}/{lever}"
    if newest_artifact_name:
        for rx, fmt in _ARTIFACT_STAGES:
            m = rx.search(newest_artifact_name)
            if m:
                return fmt.format(*m.groups())
    return "working..."


def scan_workdir_outputs(workdir):
    """Fallback outputs when the RESULT= line is missing or unusable.

    Scavenges what the run left behind: the declared final board (else the
    newest stable non-input board), REPORT.md, and the movie (mp4 preferred,
    gif fallback). Same dict shape as parse_placement_result, with
    status "scavenged" so callers can say so.
    """
    artifacts = [t for t in list_board_artifacts(workdir)
                 if os.path.basename(t[0]) != "input.kicad_pcb"]
    board = os.path.join(workdir, "final.kicad_pcb")
    if not os.path.isfile(board):
        # No newer scan to compare sizes against - the run is over, so any
        # remaining file is as stable as it will ever get.
        board = newest_stable_board(artifacts, artifacts)
    report = os.path.join(workdir, "REPORT.md")
    if not os.path.isfile(report):
        report = None
    movie = None
    for name in ("placement.mp4", "placement.gif"):
        candidate = os.path.join(workdir, name)
        if os.path.isfile(candidate):
            movie = candidate
            break
    if movie is None:
        try:
            movies = sorted(
                (os.path.join(workdir, n) for n in os.listdir(workdir)
                 if n.endswith((".mp4", ".gif"))),
                key=lambda p: os.path.getmtime(p))
        except OSError:
            movies = []
        movie = movies[-1] if movies else None
    return {
        "status": "scavenged",
        "board": board,
        "movie": movie,
        "report": report,
        "blocking": None,
        "summary": "",
    }
