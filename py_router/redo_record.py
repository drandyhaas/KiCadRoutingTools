"""Self-recording of board-mutating tool invocations (issue #132).

The stress-test redo harness replays the exact command sequence a board run
executed. Recording used to live only in tests/stress/run_limited.sh, so it
fired only when a command was actually routed through that wrapper -- and the
LLM agent does not always do so. Having each board-mutating CLI record its OWN
invocation makes capture reliable regardless of how the tool was launched.

Each of route.py / route_diff.py / route_planes.py /
repair_planes.py / bga_fanout.py calls record_invocation() at the
top of main(). It is a no-op unless the REDO_MANIFEST env var points at a
manifest file (run_board.sh sets it to <run-dir>/redo_commands.sh; set it to
/dev/null or leave it unset to disable). The manifest format matches what
redo_stress_test.py expects: a '# cwd=<dir>' line followed by the fully-quoted
command on the next line, led by the env knobs it ran with as `NAME=value`
assignments (#1302), which bash and redo_stress_test both apply. A command
that wrote nothing -- a refusal -- is taken back out of the manifest as the
process ends (#1640).
"""
from __future__ import annotations

import atexit
import json
import os
import re
import shlex
import sys
import time

#: The recorder's own writes go through the real `open`: they are not the
#: tool's (see `_watch_process`).
_REAL_OPEN = open


def _resolve_manifest(manifest: str):
    """Pin the manifest to the board's run dir, never the tools checkout.

    Stress agents re-export REDO_MANIFEST as "$PWD/redo_commands.sh" even though
    run_board.sh already exports the absolute run-dir path. That is correct only
    while PWD is the run dir -- and agents `cd` into the repo to import
    kicad_parser, after which $PWD is the CHECKOUT. The record then lands in
    py_router/redo_commands.sh, so the board's own manifest is silently missing
    those steps (qfhmix01: 3 of 24 steps recorded into the repo, 2026-08-06) and
    the repo accumulates untracked debris.

    REDO_RUNDIR (exported by run_board.sh) is the authority: relative paths
    resolve against it, and a path that lands inside this checkout is redirected
    back to it. Returns None when the record would escape and there is no run dir
    to fall back on -- dropping the record beats corrupting the repo.
    """
    rundir = os.environ.get("REDO_RUNDIR")
    if not os.path.isabs(manifest):
        manifest = os.path.join(rundir or os.getcwd(), manifest)
    manifest = os.path.abspath(manifest)
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    try:
        escaped = os.path.commonpath([manifest, repo]) == repo
    except ValueError:      # different drives (Windows) -> cannot be inside
        escaped = False
    if escaped:
        if not rundir:
            return None
        manifest = os.path.join(rundir, os.path.basename(manifest))
    return manifest


def _timings_path(manifest: str) -> str:
    return (manifest[:-3] if manifest.endswith(".sh") else manifest) + ".timings.jsonl"


def _write_timing(manifest: str, cmd: list, cwd: str, env: dict,
                  start: float) -> None:
    """Append this command's wall-clock to a sibling timings log (issue
    #132). Kept in a SEPARATE file from the manifest so the manifest stays a
    clean replayable script; one JSON line per command, in execution order.
    Lets the original LLM run's per-command timing be compared against a
    later replay (redo_stress_test.py --timings-out)."""
    timings = _timings_path(manifest)
    try:
        rec = {"seconds": round(time.time() - start, 3), "cwd": cwd, "argv": cmd}
        if env:
            rec["env"] = env    # #1302: the knobs the step ran with
        with _REAL_OPEN(timings, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")
    except Exception:
        pass


# ---------------------------------------------------------------- refusals
# A command is recorded BEFORE it does its work, so a run that is killed part
# way is still in the manifest. The cost was that a command its tool REFUSED
# was in it too (#1640): `route.py does_not_exist.kicad_pcb x.kicad_pcb` exits
# 2 and writes nothing, and as the last command it became the pruned chain's
# final board -- the real work was pruned away and the replay died on the
# typo. The refusals are not one call site per tool (an argparse error, a
# `parser.error` after the parse, a `print` + `sys.exit(2)` deep in a main, a
# FileNotFoundError on a missing input), so the record is taken back at exit
# instead, by one rule for every self-recording tool: the process WROTE
# NOTHING -- no file its command line names changed, no file appeared in or
# left the folders they sit in (or the cwd), and no file was opened for
# writing. A command that wrote nothing has nothing a replay could need,
# whatever it exited with (a refusal, `--help`, a listing). The exit status
# is deliberately not the test: `sys.exit(main())` looks `sys.exit` up before
# `main()` runs, so a status cannot be learned from inside the tool. A
# KeyboardInterrupt, and a kill that skips `atexit`, keep the record: that is
# the killed run the early record exists for. A command taken back writes no
# timings line either.

_STATE = {"installed": False, "killed": False, "wrote": False, "records": []}


def _path_tokens(argv, cwd):
    """Each argument that may name a file (an option's `=`-joined value
    included), as an absolute path, in order."""
    out = []
    for a in argv:
        if not isinstance(a, str) or not a or len(a) > 4096 or "\0" in a:
            continue
        if a.startswith("-"):
            if "=" not in a:
                continue
            a = a.split("=", 1)[1]
            if not a:
                continue
        out.append(os.path.abspath(os.path.join(cwd, a)))
    return out


def _signature(path):
    try:
        st = os.stat(path)
    except (OSError, ValueError):
        return None
    return (st.st_mtime_ns, st.st_size, st.st_ino)


def _watch_process() -> None:
    """Learn how this process ends: interrupted, and whether it opened any
    file for writing. Once per process."""
    if _STATE["installed"]:
        return
    _STATE["installed"] = True
    import builtins
    real_hook, real_open = sys.excepthook, builtins.open

    def _hook(et, ev, tb):
        if issubclass(et, KeyboardInterrupt):
            _STATE["killed"] = True
        real_hook(et, ev, tb)

    # `open` itself, not Python's "open" audit event: an audit hook is called
    # for EVERY audited event, and one route of an in-repo board raised 8.5
    # million of them (`builtins.id`). The cost of a wrapper is one frame in
    # the traceback of an open that fails, which the comment on its line
    # explains.
    def _open(file, mode="r", *args, **kwargs):
        if (not _STATE["wrote"] and isinstance(mode, str)
                and any(c in mode for c in "wax+")):
            _STATE["wrote"] = True
        return real_open(file, mode, *args, **kwargs)  # redo_record (#1640): notes a write, then opens as asked

    sys.excepthook, builtins.open = _hook, _open
    atexit.register(_finish)


def _watched(argv, cwd):
    """{path: signature} for every file the command line names and the
    folders they sit in, the cwd's included."""
    paths = _path_tokens(argv, cwd)
    dirs = {cwd} | {os.path.dirname(p) for p in paths}
    return {p: _signature(p) for p in list(paths) + sorted(dirs)}


def _wrote_nothing(rec) -> bool:
    if _STATE["killed"] or _STATE["wrote"]:
        return False
    return all(_signature(p) == sig for p, sig in rec["watch"].items())


def _finish() -> None:
    for rec in _STATE["records"]:
        try:
            if _wrote_nothing(rec):
                retract_record(rec["manifest"], rec["written"])
            else:
                _write_timing(rec["manifest"], rec["cmd"], rec["cwd"],
                              rec["knobs"], rec["start"])
        except Exception:
            pass


def retract_record(manifest: str, written) -> bool:
    """Take back ONE record `write_record` appended: `written` is what it
    returned, (the manifest's size before, the bytes appended). The bytes are
    removed where they were written, or at their last occurrence after that
    if the file changed since; a manifest the record created is removed.
    Returns whether the record was found."""
    before, block = written
    with _REAL_OPEN(manifest, "rb") as f:
        data = f.read()
    at = before if data[before:before + len(block)] == block \
        else data.rfind(block, before)
    if at < 0:
        return False
    rest = data[:at] + data[at + len(block):]
    if not rest and before == 0:
        os.remove(manifest)
        return True
    with _REAL_OPEN(manifest, "r+b") as f:
        f.write(rest)
        f.truncate()
    return True


def _replay_knobs() -> dict:
    """The env knobs this process runs with that a replay must reproduce
    (#1302); {} when none is set, or when env_knobs cannot be imported."""
    try:
        from env_knobs import replay_env_knobs
    except Exception:
        return {}
    return replay_env_knobs()


def env_prefix_tokens(knobs: dict) -> list:
    """`NAME=value` tokens for `knobs`, each shell-quoted on the VALUE only:
    bash reads `NAME='a b' cmd` as an assignment, but a quoted `'NAME=a b'`
    as the name of the command to run."""
    return [f"{k}={shlex.quote(v)}" for k, v in knobs.items()]


_ENV_ASSIGN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*=")


def split_env_prefix(argv):
    """(env, argv): the command's leading `NAME=value` assignments, and the
    command that follows them (#1302).

    The recorder writes the env knobs a step ran with as that prefix -- the
    shell's own spelling, so `bash redo_commands.sh` applies them -- and an
    agent may type one by hand. A replayer must apply them as the child's
    ENVIRONMENT: handed to Popen as argv, `KICAD_PLANE_FRAGILITY_COST=0` was
    run as a program (PermissionError / FileNotFoundError), and a replayer
    that dropped them routed the step at the defaults instead. The parsed
    manifest keeps the prefix in argv, so every reader sees what was run;
    the analysis helpers (board_io, is_check_cmd, the pruner) are blind to
    it by construction, and anything that EXECUTES or converts a command
    splits it here."""
    env = {}
    i = 0
    while i < len(argv) and _ENV_ASSIGN.match(argv[i]):
        k, v = argv[i].split("=", 1)
        env[k] = v
        i += 1
    return env, list(argv[i:])


def record_invocation(manifest_env: str = "REDO_MANIFEST") -> None:
    """Append this process's invocation to the manifest named by manifest_env.

    Best-effort: any failure (unwritable path, etc.) is swallowed so recording
    can never affect the wrapped tool. Records
    `python3 -u -X utf8 <script> <args>` so the replayed command matches the
    RUNBOOK convention. `-u` is not cosmetic (#599): a replayed step killed
    part-way -- by the caller's command timeout, the memory watchdog, or a
    Ctrl-C -- leaves an EMPTY log with buffered stdout, so the one artifact
    that would explain the kill dies with it (usmu_smu, sets-21-27 wave).

    The env knobs the step ran with lead the line as a `NAME=value` prefix
    (#1302), the shell's own spelling and the one the routing skill tells an
    agent to type. Without them a replay ran the step at the defaults: gp14
    ecc83 routed 0 vias / 131.72 mm under KICAD_PLANE_FRAGILITY_COST=0 and
    its recorded line replayed to 2 vias / 146.10 mm.

    The record is taken back at exit when the command wrote nothing -- a
    refusal (#1640; see "refusals" above). Call it at the top of `main()`,
    before the work, so a killed run is still recorded.
    """
    manifest = os.environ.get(manifest_env)
    if not manifest or manifest == os.devnull or manifest == "/dev/null":
        return
    try:
        manifest = _resolve_manifest(manifest)
        if not manifest:
            return
        argv = list(sys.argv)
        if not argv:
            return
        cmd = ["python3", "-u", "-X", "utf8"] + argv
        knobs = _replay_knobs()
        cwd = os.getcwd()
        start = time.time()
        written = write_record(manifest, cmd, cwd, knobs)
        # After the write: a manifest it creates is no write of the tool's.
        watch = _watched(argv[1:], cwd)
        _STATE["records"].append({"manifest": manifest, "cmd": cmd, "cwd": cwd,
                                  "knobs": knobs, "start": start,
                                  "written": written, "watch": watch})
        _watch_process()
    except Exception:
        pass


def write_record(manifest: str, cmd: list, cwd: str, knobs: dict) -> None:
    """Append ONE command to `manifest` in the recorded format: the header
    when the file is new, the `# cwd=` line, then the shell-quoted command
    led by its env knobs as `NAME=value`. The one writer of the format:
    `record_invocation` uses it, and so does a runner that records the steps
    it runs into a manifest of their own (judge_seeds' route folders, #1592).
    Raises on an unwritable path; the caller decides whether that matters.
    Returns (the manifest's size before, the bytes appended), which
    `retract_record` takes back."""
    quoted = " ".join(env_prefix_tokens(knobs)
                      + [shlex.quote(a) for a in cmd])
    before = os.path.getsize(manifest) if os.path.exists(manifest) else 0
    header = ""
    if before == 0:
        header = ("#!/bin/bash\n"
                  "# Auto-recorded stress-test command manifest (issue #132).\n"
                  "# Replay with redo_stress_test.py.\n"
                  "set -e\n")
    # The line ends a text-mode append wrote (CRLF on Windows), as bytes, so
    # the record can be found again exactly.
    block = (f"{header}# cwd={shlex.quote(cwd)}\n{quoted}\n"
             .replace("\n", os.linesep).encode("utf-8"))
    with _REAL_OPEN(manifest, "ab") as f:
        f.write(block)
    return before, block
