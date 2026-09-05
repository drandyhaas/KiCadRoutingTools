#!/usr/bin/env python3
"""#828: a marshalled board save says WHY it did not happen.

`ui_thread.save_board_on_ui_thread` returned one bare `False` for two facts
that call for opposite responses: the 120 s expiry (the wx main thread was not
pumping -- this machine, this moment, retryable) and a `pcbnew.SaveBoard`
exception (this board -- it will throw again). Each arm PRINTED distinctly, so
a human could tell; no caller could. `save_board_on_ui_thread_ex` returns a
`SaveStatus` beside the bool, and this test drives every arm against stubbed
`wx` / `pcbnew` modules so it runs without KiCad, from a real worker thread,
the configuration the #688 gate needs KiCad's python for.

Pinned here, in the order the issue argues them:

  1. UI thread pumping, SaveBoard ok      -> (True,  'ok')
  2. UI thread NEVER pumps                -> (False, 'timeout'), elapsed set,
     and the abandoned save does NOT run SaveBoard when the UI thread finally
     gets to it (the #688 dead-work guard, unchanged)
  3. SaveBoard raises                     -> (False, 'save_failed'), the
     exception's text in `detail`
  4. no wx at all, SaveBoard raises       -> 'save_failed', not a fourth reason
     (the issue's own note: absent wx calls straight through)
  5. the bool wrapper's contract survives: `is False` on BOTH failure arms and
     `is True` on success -- a NamedTuple in its place would be truthy and
     silently pass every `if not save_board_...` site
  6. the two failure arms are DIFFERENT reasons with different `why()` text --
     the programmatic distinction the issue says did not exist
  7. every reason is in the closed vocabulary, and `why()` names an unknown
     one loudly rather than echoing it
"""
import os
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))


class _Stubs:
    """Install stub `wx` and `pcbnew` for one scenario; restore on exit."""

    def __init__(self, pumping, save_raises=None, no_wx=False):
        self.pumping = pumping
        self.save_raises = save_raises
        self.no_wx = no_wx
        self.saved = []          # (path, aSkipSettings) per SaveBoard call
        self.queued = []         # callables CallAfter received
        self._prev = {}

    def __enter__(self):
        pcb = types.ModuleType('pcbnew')
        stubs = self

        def SaveBoard(path, board, aSkipSettings=False):
            stubs.saved.append((path, aSkipSettings))
            if stubs.save_raises is not None:
                raise stubs.save_raises
            return True
        pcb.SaveBoard = SaveBoard

        for name in ('wx', 'pcbnew'):
            self._prev[name] = sys.modules.get(name)
        sys.modules['pcbnew'] = pcb
        if self.no_wx:
            # `import wx` must FAIL, not import a real one that may be around.
            sys.modules['wx'] = None
        else:
            wx = types.ModuleType('wx')
            wx.IsMainThread = lambda: False        # we are the worker

            def CallAfter(fn):
                stubs.queued.append(fn)
                if stubs.pumping:
                    # The "UI thread": runs it shortly, off the caller's thread.
                    threading.Timer(0.05, fn).start()
            wx.CallAfter = CallAfter
            sys.modules['wx'] = wx
        return self

    def __exit__(self, *exc):
        for name, prev in self._prev.items():
            if prev is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prev
        return False


def _on_worker(fn):
    """Run `fn` on a real worker thread and return its result."""
    box = {}

    def _run():
        try:
            box['r'] = fn()
        except BaseException as e:      # noqa: BLE001 -- reported, not lost
            box['exc'] = e
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    t.join(timeout=30)
    if t.is_alive():
        raise AssertionError("BLOCKED: the helper did not return in 30s")
    if 'exc' in box:
        raise box['exc']
    return box['r']


def main():
    from ui_thread import (save_board_on_ui_thread, save_board_on_ui_thread_ex,
                           SaveStatus, SAVE_REASONS)
    fails = []
    board = object()

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # 1. pumping + ok
    with _Stubs(pumping=True) as st:
        ok, status = _on_worker(lambda: save_board_on_ui_thread_ex(
            'a.kicad_pcb', board, timeout_s=5))
        check(ok is True and status.reason == 'ok' and status.ok,
              f"pumping+ok: expected (True, 'ok'), got ({ok!r}, {status!r})")
        check(st.saved == [('a.kicad_pcb', True)],
              f"pumping+ok: SaveBoard should run once with aSkipSettings=True, "
              f"saw {st.saved!r}")
        check(_on_worker(lambda: save_board_on_ui_thread(
            'b.kicad_pcb', board, timeout_s=5)) is True,
              "bool wrapper: expected exactly True on success")

    # 2. never pumps -> timeout, and the abandoned save does no dead work
    with _Stubs(pumping=False) as st:
        t0 = time.monotonic()
        ok, status = _on_worker(lambda: save_board_on_ui_thread_ex(
            'c.kicad_pcb', board, timeout_s=0.3))
        el = time.monotonic() - t0
        check(ok is False and status.reason == 'timeout' and status.is_timeout,
              f"never-pumps: expected (False, 'timeout'), got ({ok!r}, {status!r})")
        check(status.elapsed_s is not None and 0.25 <= status.elapsed_s < 5,
              f"never-pumps: elapsed_s should be ~0.3, got {status.elapsed_s!r}")
        check(0.25 <= el < 5, f"never-pumps: returned after {el:.2f}s, expected ~0.3")
        check('0s' in status.why() or 'within' in status.why(),
              f"never-pumps: why() should say how long it waited: {status.why()!r}")
        check(st.saved == [], f"never-pumps: SaveBoard must not have run, saw {st.saved!r}")
        # The UI thread gets to it LATER: the #688 guard must skip the dead work.
        check(len(st.queued) == 1, f"never-pumps: expected one CallAfter, saw {len(st.queued)}")
        for fn in st.queued:
            fn()
        check(st.saved == [], f"never-pumps: an ABANDONED save ran SaveBoard "
                              f"when the UI thread finally pumped: {st.saved!r}")
        timeout_status = status
        check(_on_worker(lambda: save_board_on_ui_thread(
            'd.kicad_pcb', board, timeout_s=0.3)) is False,
              "bool wrapper: expected exactly False on timeout")

    # 3. pumping, SaveBoard raises -> save_failed with the exception text
    with _Stubs(pumping=True, save_raises=IOError("disk is a teapot")) as st:
        ok, status = _on_worker(lambda: save_board_on_ui_thread_ex(
            'e.kicad_pcb', board, timeout_s=5))
        check(ok is False and status.reason == 'save_failed' and not status.is_timeout,
              f"raises: expected (False, 'save_failed'), got ({ok!r}, {status!r})")
        check('teapot' in status.detail and 'OSError' in status.detail,
              f"raises: detail should carry the exception type and text: {status.detail!r}")
        check('teapot' in status.why(), f"raises: why() should carry the text: {status.why()!r}")
        failed_status = status
        check(_on_worker(lambda: save_board_on_ui_thread(
            'f.kicad_pcb', board, timeout_s=5)) is False,
              "bool wrapper: expected exactly False on a save exception")

    # 4. no wx: straight through, and a raise is still 'save_failed'
    with _Stubs(pumping=False, save_raises=RuntimeError("no"), no_wx=True) as st:
        ok, status = _on_worker(lambda: save_board_on_ui_thread_ex(
            'g.kicad_pcb', board, timeout_s=0.3))
        check(ok is False and status.reason == 'save_failed',
              f"no-wx+raise: expected 'save_failed' (not a fourth reason), got {status!r}")
        check(st.saved == [('g.kicad_pcb', True)],
              f"no-wx: SaveBoard should have been called directly, saw {st.saved!r}")
    with _Stubs(pumping=False, no_wx=True) as st:
        ok, status = _on_worker(lambda: save_board_on_ui_thread_ex(
            'h.kicad_pcb', board, timeout_s=0.3))
        check(ok is True and status.reason == 'ok',
              f"no-wx+ok: expected (True, 'ok'), got ({ok!r}, {status!r})")

    # 6. the distinction the issue says did not exist
    check(timeout_status.reason != failed_status.reason,
          "timeout and save_failed came back as the SAME reason")
    check(timeout_status.why() != failed_status.why(),
          "timeout and save_failed came back with the SAME why() text")

    # 7. closed vocabulary, loud on an unknown token
    for r in ('ok', 'timeout', 'save_failed'):
        check(r in SAVE_REASONS, f"{r!r} missing from SAVE_REASONS")
    check(set(SAVE_REASONS) == {'ok', 'timeout', 'save_failed'},
          f"SAVE_REASONS grew without this test noticing: {SAVE_REASONS!r}")
    check('unknown' in SaveStatus('bogus').why(),
          f"an unknown reason should be named as unknown: {SaveStatus('bogus').why()!r}")

    if fails:
        for f in fails:
            print(f"FAIL: {f}")
        return 1
    print("PASS: #828 -- a marshalled save reports 'timeout' apart from "
          "'save_failed', the bool wrapper keeps its False/True identity, and "
          "an abandoned save does no dead work")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
