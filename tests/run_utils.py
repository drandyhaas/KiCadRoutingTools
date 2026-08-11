"""
Shared utilities for test scripts.
"""
from __future__ import annotations

import os
import shlex
import subprocess

# Get the root directory (parent of tests/)
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)


def run(cmd: str, unbuffered: bool = False) -> None:
    """Run a command string and print output.

    Commands are executed from the project root directory, so paths like
    'python3 py_router/route.py' and 'kicad_files/...' work correctly.

    Args:
        cmd: Command string to run (will be parsed using shell-style splitting)
        unbuffered: If True, add -u flag to python commands
    """
    if unbuffered and cmd.startswith('python3 '):
        cmd = 'python3 -u ' + cmd[8:]
    print(f"\n>>> {cmd}")
    args = shlex.split(cmd)
    result = subprocess.run(args, cwd=ROOT_DIR)
    if result.returncode != 0:
        print(f"Command failed with exit code {result.returncode}")


# ---------------------------------------------------------------------------
# Refusal assertions -- "it failed" is not "it failed for the reason I meant"
# ---------------------------------------------------------------------------
#
# A NEGATIVE CONTROL THAT CANNOT DISTINGUISH ITS OWN BREAKAGE FROM THE DEFECT IT
# TESTS WILL EVENTUALLY REPORT A FALSE CLEAN. This is the same defect class the
# tools themselves keep exhibiting -- an instrument that can fail two ways and
# reports one -- and it is easier to hit in a test, because a test's failure
# path is the path nobody looks at.
#
# Observed, all in one session, every one reporting "the guard held" or "the
# feature is absent" when neither was true:
#
#   * a negative control exited 1 from a ModuleNotFoundError (the copy under
#     test lived in a temp dir with no sys.path entry) -- read as "the gate
#     refused";
#   * a probe called a function name that does not exist and reported the
#     feature missing;
#   * evidence passed as `<(echo '{...}')`, whose fd is gone by the time a
#     Windows child opens it -- read as "the stage never mentions it";
#   * a section splitter matched the wrong header and reported zero mentions;
#   * an assertion that `--deadline 0` must always produce a partial, which is
#     false on a board with no work to abandon.
#
# `must_refuse` separates the two outcomes. An ACCIDENT (import error,
# traceback, argparse usage, missing file) is reported as a BROKEN TEST, never
# as a satisfied guard.

_ACCIDENTS = (
    'Traceback (most recent call last)', 'ModuleNotFoundError', 'ImportError',
    'SyntaxError', 'IndentationError', 'NameError', 'AttributeError',
    'No such file or directory', 'unrecognized arguments',
    'error: argument', 'command not found', 'Errno 2',
)


def _accident(out: str):
    for a in _ACCIDENTS:
        if a in out:
            return a
    return None


def check(argv, *, refuse=None, code=None, accept=False, timeout=900,
          cwd=None, allow=()):
    """Run `argv`; assert it refused FOR THE STATED REASON, or accepted.

    refuse : substring the refusal must contain -- the reason, not just failure.
    code   : exact exit code, when the contract names one.
    accept : assert success instead (exit 0 and no traceback).
    allow  : accident substrings that are legitimate here (e.g. a test that
             deliberately feeds a bad argument and wants argparse's message).

    Returns the CompletedProcess. Raises AssertionError with the output on any
    mismatch -- including a failure that happened for an unrelated reason,
    which is reported as a broken test rather than a passing assertion.
    """
    r = subprocess.run([str(a) for a in argv], capture_output=True, text=True,
                       encoding='utf-8', errors='replace',
                       cwd=cwd or ROOT_DIR, timeout=timeout)
    out = (r.stdout or '') + (r.stderr or '')
    tail = out[-1200:]
    acc = _accident(out)
    if acc and acc not in allow:
        raise AssertionError(
            f"BROKEN TEST, not a satisfied guard: the command died from "
            f"{acc!r}, which is unrelated to what this check asserts. "
            f"A control that cannot tell its own breakage from the defect it "
            f"tests reports a false clean.\n  argv: {argv}\n{tail}")
    if accept:
        if r.returncode != 0:
            raise AssertionError(f"expected success, got exit "
                                 f"{r.returncode}\n  argv: {argv}\n{tail}")
        return r
    if r.returncode == 0:
        raise AssertionError(f"expected a REFUSAL, got exit 0 -- the guard did "
                             f"not hold\n  argv: {argv}\n{tail}")
    if code is not None and r.returncode != code:
        raise AssertionError(f"expected exit {code}, got {r.returncode}\n"
                             f"  argv: {argv}\n{tail}")
    if refuse and refuse not in out:
        raise AssertionError(
            f"it failed, but NOT for the stated reason. Expected the output to "
            f"mention {refuse!r}.\n  argv: {argv}\n{tail}")
    return r


def evidence(path, what='evidence'):
    """Assert a file exists and is non-empty BEFORE it is used as input.

    The `<(echo ...)` process-substitution trap: the fd is gone by the time a
    native Windows child opens it, so the stage refuses for a reason that has
    nothing to do with what is being tested, and the test reads that refusal as
    its answer. Materialise evidence to a real file and check it here.
    """
    if not os.path.isfile(path):
        raise AssertionError(f"{what} does not exist: {path} -- a check whose "
                             f"INPUT is missing tests nothing")
    if os.path.getsize(path) == 0:
        raise AssertionError(f"{what} is empty: {path}")
    return path
