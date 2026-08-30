"""Self-describing CMD/EXIT banner for instrument CLIs (run-3 finding B1).

Run 3's conformance watcher filed 8 traceability FAILs for one root cause:
instruments leave no record of HOW they were invoked or WHAT they exited,
so every gate had to be hand-wrapped in ``{ echo "CMD: ..."; tool; echo
"EXIT=$?"; }`` — and the hand wrapping was applied inconsistently (one
render log carried it, the next did not). The evidence chain should not
depend on operator discipline: the instrument prints its own banner.

Usage — two lines at the top of the ``__main__`` block, no re-indentation::

    if __name__ == "__main__":
        import cli_banner; cli_banner.install()
        ... existing body, whether inline or sys.exit(main()) ...

``install()``:

- prints ``CMD: <the exact command line>`` — from ``sys.orig_argv`` when
  available (Python 3.10+), which includes interpreter flags like
  ``-X utf8`` verbatim, so the line is REPLAYABLE truth rather than a
  reconstruction;
- arranges for ``EXIT=<rc>`` to be printed as the process ends, whatever
  the path out: a ``sys.exit(n)`` (argparse errors included), falling off
  the end of the script (rc 0), or an unhandled exception (rc 1, the
  traceback still prints).

Both lines go to stdout so a single ``> log 2>&1`` redirect captures the
whole evidence unit. Set ``KRT_NO_BANNER=1`` to suppress (for consumers
that parse an instrument's stdout strictly).
"""
from __future__ import annotations

import atexit
import os
import sys

_installed = False


def _quote(a: str) -> str:
    if not a or any(c in a for c in ' \t"\'!*?$&|;<>(){}'):
        return '"' + a.replace('"', '\\"') + '"'
    return a


def command_line() -> str:
    """The exact invocation, interpreter flags included when knowable."""
    argv = getattr(sys, 'orig_argv', None)
    if argv:
        head = [os.path.basename(argv[0])] + list(argv[1:])
    else:  # pre-3.10 fallback: reconstruct (loses -X flags)
        head = [os.path.basename(sys.executable)] + list(sys.argv)
    return ' '.join(_quote(a) for a in head)


def install_dash_hint() -> None:
    """Explain the one argparse failure a real net name can cause.

    Run 14 lost a lap to this. The board carried a net called ``-12V`` -- not
    exotic; every bipolar analog board has one -- and ``route.py --nets ...
    -12V ...`` died with::

        route.py: error: unrecognized arguments: -12V

    no board, no JSON_SUMMARY, exit 2. ``--nets`` is ``nargs='+'``, so argparse
    takes a leading ``-`` as the start of a flag. The same hole is in every
    name-taking list flag across route.py, route_diff.py, route_planes.py,
    route_disconnected_planes.py, check_drc.py and check_connected.py -- 30-odd
    of them -- and the fix at each call site is the same one character:
    ``--nets=-12V``.

    Rather than patch thirty parsers, this patches ``ArgumentParser.error`` once
    from ``install()``, which every instrument already calls. The GUI never
    calls ``install()``, so no GUI path changes.
    """
    import argparse
    import re
    orig = argparse.ArgumentParser.error
    if getattr(orig, '_krt_dash_hint', False):
        return

    def error(self, message):                       # noqa: D401
        try:
            m = re.search(r'unrecognized arguments:\s*(.+)$', str(message))
            bad = [t for t in (m.group(1).split() if m else [])
                   if t.startswith('-') and t != '-']
            if bad:
                t = bad[0]
                sys.stderr.write(
                    "\n  NOTE: %r begins with '-', so argparse read it as a "
                    "flag rather than a value.\n"
                    "        A net (or layer) whose NAME starts with '-' must "
                    "be attached with '=':\n"
                    "            --nets=%s          not   --nets %s\n"
                    "        Every name-taking list flag here has this shape "
                    "(--nets, --power-nets,\n"
                    "        --rip-existing-nets, --plane-layers, ...).\n\n"
                    % (t, t, t))
                sys.stderr.flush()
        except Exception:               # a hint must never mask the real error
            pass
        return orig(self, message)

    error._krt_dash_hint = True
    argparse.ArgumentParser.error = error


def install() -> None:
    """Print the CMD banner now; print EXIT=<rc> when the process ends."""
    global _installed
    if _installed or os.environ.get('KRT_NO_BANNER'):
        return
    _installed = True
    install_dash_hint()
    # Line-buffer the log (run-7 A11). Redirected to a file, stdout is
    # BLOCK-buffered, so a run that is killed -- a timeout, a stall, a closed
    # session -- loses whatever sits in the 8KB buffer. One run-7 board's
    # legalize log ended at 200 bytes that way, and the stage was read as a
    # silent no-op when it had in fact repaired 7 parts. These are minute-scale
    # tools whose logs are the evidence; a syscall per line is the right trade.
    # A tty is already line-buffered, so this only changes the redirected case.
    for _stream in (sys.stdout, sys.stderr):
        try:
            if not _stream.isatty():
                _stream.reconfigure(line_buffering=True)
        except Exception:      # non-reconfigurable wrapper: keep the banner
            pass
    print(f"CMD: {command_line()}", flush=True)
    # #653: the knobs that were actually in the environment. Printed next to
    # CMD because they are the OTHER half of "how was this invoked" -- a knob
    # exported in the launching shell changes routing exactly like a flag
    # does, but left no trace, so an env-contaminated baseline could only be
    # found by re-running it. `ENV KNOBS: none` is the clean statement, and it
    # is printed too: silence must not be the answer for both cases (the #654
    # lesson).
    try:
        import env_knobs
        print(env_knobs.env_knobs_line(), flush=True)
    except Exception:              # a diagnostic must never break the tool
        pass

    state = {'rc': 0}

    real_exit = sys.exit

    def _exit(code=0):
        state['rc'] = code if isinstance(code, int) else (0 if code is None else 1)
        real_exit(code)

    sys.exit = _exit

    real_hook = sys.excepthook

    def _hook(et, ev, tb):
        if et is SystemExit:  # raised directly, not via sys.exit()
            c = ev.code
            state['rc'] = c if isinstance(c, int) else (0 if c is None else 1)
        else:
            state['rc'] = 1
        real_hook(et, ev, tb)

    sys.excepthook = _hook

    @atexit.register
    def _print_exit():  # pragma: no cover - exercised via subprocess tests
        try:
            print(f"EXIT={state['rc']}", flush=True)
        except Exception:
            pass
