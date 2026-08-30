"""
Shared utilities for test scripts.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import sys

# Get the root directory (parent of tests/)
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)

#: Where the #522 reorg put the CLIs. Same order as krt_capabilities._TOOL_DIRS,
#: which is the resolver this delegates to.
_TOOL_DIRS = ('', 'py_router', 'py_tools', 'py_placer')


def tool(name):
    """Absolute path to a shipped CLI, wherever the #522 reorg put it.

    RAISES when the tool is absent, and that is the whole point.

    Commit ee860796 moved route.py, check_drc.py, place_optimize.py and the
    rest out of the repo root into py_router/ py_tools/ py_placer/. EIGHTEEN
    tests kept spawning `os.path.join(ROOT, 'route.py')`. Python exits 2 with
    `can't open file ...: [Errno 2]` on STDERR, and those tests all assert on
    STDOUT -- so the whole class surfaced as a bare AssertionError with an
    empty message that reads exactly like a product failure. They sat red long
    enough that several of them rotted into FALSE PASSES underneath: five of
    test_soft_joint's ten checks "passed" only because the subprocess produced
    no output at all, and test_457_startup_checks' `returncode == 1` passed
    because ModuleNotFoundError also exits 1.

    So a missing tool must fail HERE, naming what it looked for and where,
    rather than three frames away inside a subprocess. A static lint cannot do
    this job: test_431_board_gates builds `os.path.join(ROOT, script)` with
    `script` a variable, so the bad path is only knowable at runtime. This
    function is the gate.
    """
    try:
        sys.path.insert(0, ROOT_DIR)
        from krt_capabilities import _tool_path       # the shipped resolver
        p = _tool_path(ROOT_DIR, name)
    except Exception:                                  # pragma: no cover
        p = os.path.join(ROOT_DIR, name)
    if not os.path.isfile(p):
        raise FileNotFoundError(
            f'{name!r} is not in this checkout. Looked in: '
            + ', '.join(repr(os.path.join(d, name) or name)
                        for d in _TOOL_DIRS)
            + '. If a tool moved, fix the caller -- do not hardcode a new path.')
    return p


def tool_env(env=None):
    """An environment whose PYTHONPATH reaches every package the reorg made.

    The sibling defect to `tool`: a subprocess probe launched with
    `PYTHONPATH=ROOT` alone cannot `import kicad_parser` or `import
    startup_checks` any more, because both moved into py_router/. The child
    dies with ModuleNotFoundError and the parent reports whatever that exit
    code happens to collide with.
    """
    env = dict(os.environ if env is None else env)
    parts = [os.path.join(ROOT_DIR, d) if d else ROOT_DIR
             for d in _TOOL_DIRS]
    prev = env.get('PYTHONPATH')
    if prev:
        parts.append(prev)
    env['PYTHONPATH'] = os.pathsep.join(parts)
    return env


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


def corpus_boards(pattern: str = 'kicad_files/*.kicad_pcb'):
    """The boards git TRACKS under kicad_files/ -- the fixed set any count or
    "silent on every board" claim is actually about.

    A plain glob of that directory is not that set. `kicad_files/` accumulates
    GENERATED boards (routed/fanout/plane outputs, several of them gitignored),
    so the same glob returns 22 entries on a clean clone, 27 on a lightly used
    one, and 33+ on a machine that has run the suite for a while. Any threshold
    pinned against it is therefore only right in ONE working copy, and it fails
    on a clean checkout and in CI -- which is a test bug that reports as a
    product bug, and reads as machine-dependent flake.

    Returns [] when git cannot answer (a tarball export, no git on PATH);
    callers must SKIP rather than grade against a set they cannot identify.

    `tests/test_orbit_fit_noop.py` discovered this and carried the only copy;
    it lives here so a second caller cannot re-derive it slightly differently.
    """
    try:
        p = subprocess.run(['git', 'ls-files', '-z', pattern],
                           cwd=ROOT_DIR, capture_output=True, text=True,
                           timeout=60)
        if p.returncode == 0:
            return sorted(os.path.join(ROOT_DIR, n)
                          for n in p.stdout.split('\0') if n)
    except Exception:                                            # noqa: BLE001
        pass
    return []
