"""Self-recording of board-mutating tool invocations (issue #132).

The stress-test redo harness replays the exact command sequence a board run
executed. Recording used to live only in tests/stress/run_limited.sh, so it
fired only when a command was actually routed through that wrapper -- and the
LLM agent does not always do so. Having each board-mutating CLI record its OWN
invocation makes capture reliable regardless of how the tool was launched.

Each of route.py / route_diff.py / route_planes.py /
route_disconnected_planes.py / bga_fanout.py calls record_invocation() at the
top of main(). It is a no-op unless the REDO_MANIFEST env var points at a
manifest file (run_board.sh sets it to <run-dir>/redo_commands.sh; set it to
/dev/null or leave it unset to disable). The manifest format matches what
redo_stress_test.py expects: a '# cwd=<dir>' line followed by the fully-quoted
command on the next line.
"""

import atexit
import json
import os
import shlex
import sys
import time


def _register_timing(manifest: str, cmd: list, cwd: str) -> None:
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
            with open(timings, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
        except Exception:
            pass

    atexit.register(_on_exit)


def record_invocation(manifest_env: str = "REDO_MANIFEST") -> None:
    """Append this process's invocation to the manifest named by manifest_env.

    Best-effort: any failure (unwritable path, etc.) is swallowed so recording
    can never affect the wrapped tool. Records `python3 -X utf8 <script> <args>`
    so the replayed command matches the RUNBOOK convention.
    """
    manifest = os.environ.get(manifest_env)
    if not manifest or manifest == os.devnull or manifest == "/dev/null":
        return
    try:
        argv = list(sys.argv)
        if not argv:
            return
        cmd = ["python3", "-X", "utf8"] + argv
        quoted = " ".join(shlex.quote(a) for a in cmd)
        header = ""
        if not os.path.exists(manifest) or os.path.getsize(manifest) == 0:
            header = ("#!/bin/bash\n"
                      "# Auto-recorded stress-test command manifest (issue #132).\n"
                      "# Replay with redo_stress_test.py.\n"
                      "set -e\n")
        cwd = os.getcwd()
        with open(manifest, "a", encoding="utf-8") as f:
            f.write(f"{header}# cwd={shlex.quote(cwd)}\n{quoted}\n")
        _register_timing(manifest, cmd, cwd)
    except Exception:
        pass
