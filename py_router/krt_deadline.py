"""Wall-clock deadlines and a guaranteed JSON_SUMMARY for long-running stages.

Run 9 lost two stages to the same shape: `place_reconstruct --stages legalize`
and `route_disconnected_planes` each ran to an external `timeout(1)`, wrote no
output file, printed no `JSON_SUMMARY`, and exited with **the shell's** 124
rather than any code the tool chose. From the log they were indistinguishable
from a hang, and because both stage their output and promote it only at the end,
a kill left nothing at the output path either.

The family is wider than those two -- `EXACT_FILL_TIMEOUT` (300 s) and
`ORACLE_DRC_TIMEOUT` (240 s) both degrade to a fallback with no failure signal,
and `converge record` can fail before it execs at all. What they share:

    a long silent phase after a complete-looking report, an exit code that
    belongs to the shell rather than the tool, and staged output that leaves
    nothing behind on a kill.

This module fixes the part that is fixable from inside the process: a stage that
knows its own budget stops ITSELF, keeps the work it has done, says so in the
summary, and exits with a code it chose.

Usage -- `arm()` once, then a cooperative check in the hot loop::

    import krt_deadline
    dl = krt_deadline.arm(args.deadline, tool='place_reconstruct',
                          on_partial=lambda: report)     # None == no budget
    ...
    for violator in violators:
        if dl and dl.check('legalize'):
            break
    ...
    finally:
        krt_deadline.emit(report, deadline=dl)

`emit()` is the single print site and fires at most once per process, so calling
it from the success path AND a `finally` AND the atexit hook is safe.

WHY THIS IS NOT PART OF `cli_banner`
------------------------------------
1. `cli_banner.install()` returns early when `KRT_NO_BANNER` is set
   (`cli_banner.py:59`), and `board_score.run_tool` sets `KRT_NO_BANNER='1'` for
   every checker it shells (`board_score.py:108-119`). A summary emitter
   suppressed the same way would go dark exactly where `board_score` reads.
   Nothing here consults `KRT_NO_BANNER`.
2. The tools that NEED a deadline are largely the ones that deliberately do NOT
   carry the banner (`route_disconnected_planes.py`, `route.py`,
   `place_route_loop.py`). Folding the deadline into `cli_banner` would make
   adopting it mean adopting `CMD:`/`EXIT=` on their stdout -- a separate and
   riskier decision this repo takes seriously (`converge.py:692`: "converge's
   stdout is a JSON API").

`atexit` runs LIFO, so a hook registered by `arm()` AFTER `cli_banner.install()`
fires BEFORE the banner's `EXIT=` hook. That gives `JSON_SUMMARY:` then `EXIT=`,
which is the order every consumer wants. **That ordering is load-bearing.**

WHAT THIS CANNOT DO
-------------------
Measured on Windows, so nobody expects more than this delivers::

    external `timeout 1`, tool has NO deadline   -> shell rc 124,
                                                    0 JSON_SUMMARY lines,
                                                    0 EXIT lines
    external `timeout 5`, tool deadline 0.4s     -> rc 7, status "deadline",
                                                    partial result intact,
                                                    EXIT=7

The first row is run 9's actual case and it stays **unfixable from inside the
process**: a native win32 process cannot receive a POSIX SIGTERM, and under Git
Bash the kill resolves to `TerminateProcess`, which is uncatchable -- no handler,
no `atexit`, no flush. Anyone designing around a SIGTERM handler here is
designing for a platform that did not produce the bug.

So: **setting a budget BELOW whatever the harness allows is the whole
mechanism.** The signal handlers below only help for a polite SIGINT/SIGTERM
(interactive Ctrl+C, POSIX CI). The remaining defence for the uncatchable case is
line buffering (already done, `cli_banner.py:69-74`), the `DEADLINE:` breadcrumb
this module prints at arm time, and a progress heartbeat -- which is why
`stdout_progress` is here and not cosmetic.

DELIBERATELY NOT DONE
---------------------
* **A watchdog thread that prints and calls `os._exit`.** It fires even when the
  main thread is inside a C call, which is seductive -- and it emits state the
  main thread is mid-mutation of, skips `atexit` (losing `EXIT=`, the exact
  defect being fixed), and can interleave a partial line into stdout. Both hot
  loops here are pure Python and reach a cooperative check.
* **A `DeadlineExceeded` exception.** `route_disconnected_planes.py` is full of
  `except Exception` swallowers -- `:2474` sits directly on the hot path -- so an
  exception-based deadline would be caught and converted into a printed note,
  which is precisely the silent failure being removed. Cooperative flag checks
  and `return None` only.
* **A generous default budget.** A wall clock default would make the same board
  produce a different output on a slower machine, which this repo cares about
  specifically (`tests/test_457_determinism.py`, `redo_commands.sh` replays). The
  default is None; the budget belongs to the harness that already owns a clock.
"""
from __future__ import annotations

import json
import os
import sys
import time
from typing import Any, Callable, Dict, Optional

#: Exit code a stage returns when its OWN budget stopped it.
#:
#: Not 124 -- that is the shell's, and erasing the tool/shell distinction is the
#: confusion being removed. Not 4 -- in these tools 4 asserts a MEASURED defect
#: class (`route_disconnected_planes.py:3616-3623` advises "reconnect them, or
#: re-run without --rip-blocker-nets", which presumes a completed run), and a
#: run that never finished has measured nothing. Not 5 -- already spoken for
#: repo-wide: `converge.py:446` STUCK and `check_complete.py` UNSOUND. 7 is
#: unused anywhere, which is what a harness needs.
DEADLINE_EXIT = 7

_emitted = False
_summary_fn: Optional[Callable[[], Dict[str, Any]]] = None
_installed = False
_signalled = False
_current: Optional['Deadline'] = None


def current() -> Optional['Deadline']:
    """The budget armed for this process, if any.

    A deliberate global, for the one case threading cannot reach: work buried
    under a shared engine that no caller in the chain can pass a parameter to.
    The motivating case is `plane_fragility.register_plane_fragility`, invoked
    from `route.py:batch_route` -- so EVERY batch_route on a board KiCad cannot
    fill pays the full 300s EXACT_FILL_TIMEOUT, and the plane repair calls
    batch_route many times. That is the root cause of both non-terminations
    measured in run 9, and no signature between the CLI and that call site
    carries a budget.

    Use it only where a parameter genuinely cannot reach; everywhere else pass
    the Deadline explicitly.
    """
    return _current


class Deadline:
    """A wall-clock budget. Never construct with 0 meaning "no budget" -- use
    `arm(None)`; `--deadline 0` is a legitimate "stop immediately", which the
    tests rely on.

    `expired()` is a subtraction and a comparison -- cheap enough for a loop that
    runs once per violator or once per pad. Do NOT call it per candidate pose;
    the granularity that matters is "can the partial be described coherently",
    not "how soon do we notice".
    """

    __slots__ = ('seconds', 'started', 'tool', 'stopped_in')

    def __init__(self, seconds: Optional[float], tool: str = ''):
        self.seconds = None if seconds is None else float(seconds)
        self.started = time.monotonic()
        self.tool = tool
        self.stopped_in: Optional[str] = None

    def expired(self, reserve: float = 0.0) -> bool:
        """True once the budget (minus `reserve`) is spent, or on a signal.

        `reserve` carves a band off the end so a caller can stop its main work
        early and still have clock left for a mandatory tail step. See
        `cancel_check`.
        """
        if _signalled:
            return True
        if self.seconds is None:
            return False
        return (time.monotonic() - self.started) >= max(0.0,
                                                        self.seconds - reserve)

    def check(self, label: str, reserve: float = 0.0) -> bool:
        """`expired()`, recording WHICH phase owned the clock when it went.

        The label lands in the summary as `stopped_in`, which is the difference
        between "it timed out" and "it timed out in the legalize sweep with 88
        violators left".
        """
        if self.expired(reserve):
            if self.stopped_in is None:
                self.stopped_in = label
            return True
        return False

    def cancel_check(self, label: str = '', reserve: float = 0.0):
        """A zero-arg closure for engines that already take a `cancel_check`.

        `route_disconnected_planes` (`:762`) and `route.py` (`:370`) both accept
        one and honour it at every loop head -- so a deadline there is this
        closure, not new machinery.
        """
        def _cancel() -> bool:
            return self.check(label, reserve) if label else self.expired(reserve)
        return _cancel

    def remaining(self) -> Optional[float]:
        if self.seconds is None:
            return None
        return max(0.0, self.seconds - (time.monotonic() - self.started))

    def elapsed(self) -> float:
        return time.monotonic() - self.started

    def __repr__(self) -> str:  # pragma: no cover - diagnostics only
        cap = 'none' if self.seconds is None else f'{self.seconds:g}s'
        return f'<Deadline {self.tool} cap={cap} elapsed={self.elapsed():.1f}s>'


def arm(seconds: Optional[float], *, tool: str,
        on_partial: Optional[Callable[[], Dict[str, Any]]] = None
        ) -> Optional[Deadline]:
    """Start a budget, announce it, and arrange a partial flush if interrupted.

    Precedence: the `seconds` argument, else `$KRT_DEADLINE_S` (so a harness can
    set one budget for a whole chain with one export), else None.

    Returns None when there is no budget, so callers can write `if dl and
    dl.check(...)` and pay nothing when the feature is off.

    The `DEADLINE:` line it prints is deliberate: a log WITHOUT it is proof no
    budget was set, which is the standing diagnosis for the next 46-minute run.
    """
    if seconds is None:
        env = os.environ.get('KRT_DEADLINE_S')
        if env:
            try:
                seconds = float(env)
            except ValueError:
                print(f"krt_deadline: ignoring unparseable KRT_DEADLINE_S="
                      f"{env!r}", file=sys.stderr)
    global _current
    dl = None if seconds is None else Deadline(seconds, tool=tool)
    _current = dl
    if dl is not None:
        print(f"DEADLINE: {dl.seconds:g}s ({tool})", flush=True)
    # Install the flush hook even with NO budget. The atexit hook is what makes
    # "a JSON_SUMMARY on every path" true -- it covers early returns, argparse
    # errors and unhandled exceptions, which is why the callers need no
    # try/finally around a main() with a dozen return statements. Without this,
    # the guarantee would only exist for runs that happened to set a budget.
    if on_partial is not None:
        _install(on_partial, dl)
    return dl


def emit(report: Optional[Dict[str, Any]], *, complete: bool = True,
         status: str = 'ok', deadline: Optional[Deadline] = None) -> bool:
    """The ONE `JSON_SUMMARY:` print site. Fires at most once per process.

    Returns True if this call did the printing. A payload that will not
    serialise is reported rather than swallowed -- a summary that vanished is the
    defect this module exists to remove, so it must not reappear as an exception.
    """
    global _emitted
    if _emitted or report is None:
        return False
    _emitted = True
    doc = dict(report)
    doc.setdefault('complete', complete)
    doc.setdefault('status', status)
    if deadline is not None:
        doc.setdefault('deadline_s', deadline.seconds)
        doc.setdefault('elapsed_s', round(deadline.elapsed(), 1))
        if deadline.stopped_in:
            doc.setdefault('stopped_in', deadline.stopped_in)
    try:
        print('JSON_SUMMARY: ' + json.dumps(doc, sort_keys=True, default=str),
              flush=True)
    except Exception as exc:                                   # noqa: BLE001
        try:
            print('JSON_SUMMARY: ' + json.dumps(
                {'complete': False, 'status': 'error',
                 'error': f'summary unserialisable: {exc}'}), flush=True)
        except Exception:                                      # pragma: no cover
            pass
    return True


def seal() -> bool:
    """Declare the JSON_SUMMARY already published, WITHOUT printing one.

    For a tool whose engine prints its own summary. `route.py` and
    `route_diff.py` are the cases: `batch_route` builds a large summary and
    `print`s it directly, and it does so once per pass (the reconciliation
    self-invoke prints a second), so routing them through `emit()` -- which
    fires at most once -- would swallow every summary after the first.

    Without this the atexit flush would see `_emitted` still False on a
    perfectly successful run and publish the contentless partial report armed
    at `--deadline` time, printing a SECOND, contradicting
    `{"complete": false, "status": "incomplete"}` line after the real one. That
    is a measured defect, not a hypothetical: it shipped in
    `route_disconnected_planes` (run 11, v6_r7) and any consumer keying on the
    LAST JSON_SUMMARY read a good run as a failed one.

    Returns True if this call did the sealing.
    """
    global _emitted
    if _emitted:
        return False
    _emitted = True
    return True


def stamp(report: Dict[str, Any]) -> Dict[str, Any]:
    """Stamp THIS PROCESS's spent budget onto a summary someone else prints.

    The companion to `seal()`, and the reason neither is a new engine kwarg:
    `batch_route`'s summary is the only one `route.py` emits, and
    `place_route_loop` already refuses a `complete: false` tally
    (`place_route_loop.py:162`) while `route_summary.merge_summaries` already
    makes incompleteness sticky across passes. The one missing piece was the
    stamp itself, and it is read through `current()` rather than threaded
    through the signature -- so the GUI, which never arms a budget, is inert
    here and no parity gate is involved.

    A no-op unless a budget was armed AND actually cut the work short, so a
    summary from a run that finished inside its budget is byte-identical to
    before.

    "Cut short" is `stopped_in or expired()`, not `expired()` alone. A caller
    using a RESERVE band (`cancel_check(label, reserve=...)`, which
    route_planes and route_disconnected_planes both need so the bounded tail
    still has clock) trips its loops at `deadline - reserve` -- and if that
    tail then finishes before the wall clock runs out, the run is past its
    cancel and NOT past its deadline. Testing `expired()` alone would report
    that partial board as complete, which is precisely the confusion this
    module exists to remove. `stopped_in` is set by `Deadline.check` at the
    moment the cancel fires, so it is the durable record.
    """
    dl = _current
    if dl is None or not (dl.stopped_in or dl.expired()):
        return report
    report['complete'] = False
    report['status'] = 'deadline'
    report['deadline_s'] = dl.seconds
    report['elapsed_s'] = round(dl.elapsed(), 1)
    report['stopped_in'] = dl.stopped_in
    return report


def mark(report: Dict[str, Any], dl: Deadline, **partial: Any
         ) -> Dict[str, Any]:
    """Stamp `report` as a partial result, in the shape consumers rely on.

    `complete` is the machine gate -- read it as `.get('complete', True)` so
    pre-change logs degrade correctly. `status` is the branching discriminator:
    a cancel is not a deadline is not a crash, and encoding the reason only as a
    bool loses that.

    `status` is `"deadline"`, not `"timeout"` -- `timeout` already means
    something else in these summaries (`max_iterations`, A* budgets).
    """
    report['complete'] = False
    report['status'] = 'deadline'
    report['deadline_s'] = dl.seconds
    report['elapsed_s'] = round(dl.elapsed(), 1)
    report['stopped_in'] = dl.stopped_in
    if partial:
        report.setdefault('partial', {}).update(partial)
    return report


def stdout_progress(min_interval: float = 15.0,
                    deadline: Optional[Deadline] = None,
                    heartbeat_file: Optional[str] = None):
    """A throttled `(current, total, label)` callback for CLI progress.

    `route_disconnected_planes` already invokes `progress_callback` at ~9 sites;
    it is None on the CLI path only because it was built for the GUI. Passing
    this lights all of them up with no new print statements.

    Throttling matters: one of those sites fires per pad and would otherwise
    flood a 96-pad board's log. `PROGRESS:` is a distinct prefix that cannot
    match `route_summary.SUMMARY_RE`.

    run-23: `heartbeat_file` (default $KRT_PROGRESS_FILE) additionally
    rewrites one small JSON doc per tick, atomically -- a waiter or a
    RUN_STATE reader polls one file for liveness instead of tailing a log
    that only grows when something prints.
    """
    state = {'last': 0.0}
    hb = heartbeat_file or os.environ.get('KRT_PROGRESS_FILE') or None

    def _progress(current=0, total=0, label='') -> None:
        now = time.monotonic()
        if now - state['last'] < min_interval:
            return
        state['last'] = now
        frac = f"{current}/{total}  " if total else ''
        clock = ''
        if deadline is not None and deadline.seconds is not None:
            clock = f"  [t={deadline.elapsed():.0f}s/{deadline.seconds:g}s]"
        try:
            print(f"PROGRESS: {frac}{label}{clock}", flush=True)
        except Exception:                                      # noqa: BLE001
            pass
        if hb:
            try:
                doc = {'t': time.time(), 'current': current, 'total': total,
                       'label': label,
                       'elapsed_s': (round(deadline.elapsed(), 1)
                                     if deadline is not None else None),
                       'deadline_s': (deadline.seconds
                                      if deadline is not None else None)}
                tmp = hb + '.tmp'
                with open(tmp, 'w', encoding='utf-8') as f:
                    json.dump(doc, f)
                os.replace(tmp, hb)
            except Exception:                                  # noqa: BLE001
                pass
    return _progress


def _install(on_partial: Callable[[], Dict[str, Any]],
             dl: Optional[Deadline]) -> None:
    """atexit + signal wiring. Every registration is defensive: a missing signal
    must not stop the tool from running."""
    global _installed, _summary_fn
    _summary_fn = on_partial
    if _installed:
        return
    _installed = True

    import atexit

    @atexit.register
    def _flush() -> None:                                      # pragma: no cover
        if _emitted or _summary_fn is None:
            return
        try:
            rep = _summary_fn()
        except Exception as exc:                               # noqa: BLE001
            emit({'error': f'partial summary unavailable: {exc}'},
                 complete=False, status='error')
            return
        # Name the reason accurately: a run that died on an early return is
        # `incomplete`, not `deadline`. Conflating them would put the wrong
        # diagnosis in the one artefact that survives.
        if _signalled:
            status = 'cancelled'
        elif dl is not None and dl.expired():
            status = 'deadline'
            mark(rep, dl)
        else:
            status = 'incomplete'
        emit(rep, complete=False, status=status, deadline=dl)

    try:
        import signal

        def _on_signal(signum, _frame):                        # noqa: ANN001
            # Set a flag ONLY. Printing from a handler can interleave into a
            # half-written stdout line and corrupt the very JSON_SUMMARY a
            # scraper reads; the atexit hook does the I/O instead. A second
            # identical signal restores the default so Ctrl+C twice always
            # kills.
            global _signalled
            if _signalled:
                signal.signal(signum, signal.SIG_DFL)
                return
            _signalled = True
            sys.exit(130 if signum == getattr(signal, 'SIGINT', None)
                     else DEADLINE_EXIT)

        for _name in ('SIGTERM', 'SIGINT', 'SIGBREAK'):
            _sig = getattr(signal, _name, None)
            if _sig is None:
                continue
            try:
                signal.signal(_sig, _on_signal)
            except (ValueError, OSError, RuntimeError):
                pass          # not the main thread, or platform refuses it
    except Exception:                                          # noqa: BLE001
        pass


def reset_for_test() -> None:
    """Test hook: forget that a summary was emitted."""
    global _emitted, _installed, _summary_fn, _signalled
    _emitted = False
    _installed = False
    _summary_fn = None
    _signalled = False
