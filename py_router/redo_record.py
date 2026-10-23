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
assignments (#1302), which bash and redo_stress_test both apply.
"""
from __future__ import annotations

import atexit
import json
import os
import shlex
import sys
import time


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


def _register_timing(manifest: str, cmd: list, cwd: str,
                     env: dict = None) -> None:
    """Append this command's wall-clock to a sibling timings log when the
    process exits (issue #132). Kept in a SEPARATE file from the manifest so the
    manifest stays a clean replayable script; one JSON line per command, in
    execution order. Lets the original LLM run's per-command timing be compared
    against a later replay (redo_stress_test.py --timings-out)."""
    timings = (manifest[:-3] if manifest.endswith(".sh") else manifest) + ".timings.jsonl"
    start = time.time()

    def _on_exit():
        try:
            rec = {"seconds": round(time.time() - start, 3), "cwd": cwd, "argv": cmd}
            if env:
                rec["env"] = env    # #1302: the knobs the step ran with
            with open(timings, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
        except Exception:
            pass

    atexit.register(_on_exit)


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
        quoted = " ".join(env_prefix_tokens(knobs)
                          + [shlex.quote(a) for a in cmd])
        header = ""
        if not os.path.exists(manifest) or os.path.getsize(manifest) == 0:
            header = ("#!/bin/bash\n"
                      "# Auto-recorded stress-test command manifest (issue #132).\n"
                      "# Replay with redo_stress_test.py.\n"
                      "set -e\n")
        cwd = os.getcwd()
        with open(manifest, "a", encoding="utf-8") as f:
            f.write(f"{header}# cwd={shlex.quote(cwd)}\n{quoted}\n")
        _register_timing(manifest, cmd, cwd, knobs)
    except Exception:
        pass
