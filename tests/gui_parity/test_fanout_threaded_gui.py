#!/usr/bin/env python3
"""#621: the fanout tab runs its escape on a WORKER THREAD, and still applies.

The fanout tab used to call the escape engine synchronously on the UI thread.
That made its Cancel button unreachable -- nothing could deliver the click
while the engine held the thread -- so the engines' cooperative `cancel_check`
had no caller and `PlanExecutor.stop()` documented fanout as a tab that "just
runs its step to completion". It now runs the engine in `_fanout_worker` and
applies the result from `_on_operation_complete`, like the planes tab.

Threading a step that used to be synchronous can fail in three ways that no
unit test sees, and this gate drives the REAL headless `RoutingDialog` and the
REAL `FanoutTab` to catch them:

1.  **The results never arrive.** `_run_*_fanout` now returns immediately, so
    anything that read its results straight after the call gets nothing. (That
    is not hypothetical: `test_fanout_rotated_gui.py` on `main` does exactly
    that, and had to be taught to wait as part of this change.)
2.  **The apply happens off the UI thread.** `_apply_fanout_results` mutates
    the live pcbnew board; doing that from the worker is a crash waiting for a
    big board. The worker must return data and nothing else.
3.  **The busy signal is released early.** `fanout_btn.IsEnabled()` is what
    `ai_plan._poll_until_idle` polls to decide a step finished, so the button
    must stay DOWN from `_begin_run` until after the results are applied. Let
    it up any sooner and the plan executor starts the next step mid-apply.

Deliberately NOT tested here: a cancel actually firing. The only real cancel is
a human clicking Cancel, and forcing one from a timer or a call counter would
measure the harness rather than the tool. What is pinned is that the plumbing a
cancel needs is in place and inert when nobody clicks -- the copper is
identical to the CLI's, which `test_fanout_cancel.py` pins at engine level.

    python3 -X utf8 tests/gui_parity/test_fanout_threaded_gui.py
"""
import glob
import os
import subprocess
import sys

# The real dialog builds about_tab, whose wxEXPAND|wxALIGN_* sizer flags trip a
# fatal assert on wx debug builds. Must be set before wx is imported.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BOARD = os.path.join(REPO, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
REF = 'U2'
# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import wx, pcbnew'],
                              capture_output=True).returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    # os.execv re-splits argv on spaces on Windows, and the
                    # interpreter lives under "Program Files".
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with wx + pcbnew found")
    sys.exit(0)


BAD = []


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        BAD.append(name)
    return bool(cond)


def main():
    import threading
    import wx
    import pcbnew
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui

    app = wx.App(False)                                          # noqa: F841
    board = pcbnew.LoadBoard(BOARD)
    pcbnew.GetBoard = lambda: board
    pcb_data = build_pcb_data_from_board(board)
    dialog = swig_gui.RoutingDialog(None, pcb_data, BOARD)

    try:
        tab = dialog.fanout_tab
        fp = pcb_data.footprints[REF]
        nets = sorted({p.net_name for p in fp.pads if p.net_id and p.net_name})

        applied = {}
        ui_thread = threading.current_thread().ident

        def _capture(tracks, vias, failed_nets=None, **kw):
            # Stop at the apply boundary: mutating the live board is not what
            # this gate is about. Record WHICH THREAD got here -- that is
            # failure mode 2.
            applied['thread'] = threading.current_thread().ident
            applied['tracks'] = tracks
            applied['vias'] = vias

        tab._apply_fanout_results = _capture

        tab.fanout_type.SetSelection(1)          # QFN
        tab._on_type_changed(None)
        tab._run_qfn_fanout(fp, nets, tab.qfn_options.get_config())

        # The call returns IMMEDIATELY now. Busy must already be asserted.
        _ok('the busy signal is asserted before the call returns',
            tab._running and not tab.fanout_btn.IsEnabled())

        # Run the real event loop until the tab reports idle -- the condition
        # ai_plan._poll_until_idle waits on. A MainLoop, not a Yield loop: the
        # tab collects the worker's result through wx.CallLater, and Yield
        # never fires wx timers on Windows (see wx_pump.py).
        from wx_pump import run_until
        if not run_until(lambda: not tab._running, 300):
            _ok('the worker finished within the pump budget', False)

        _ok('the results arrived after the worker finished',
            bool(applied.get('tracks')))
        _ok('...and the apply ran on the UI THREAD, not the worker',
            applied.get('thread') == ui_thread)
        _ok('the busy signal is released only after the apply',
            tab.fanout_btn.IsEnabled() and not tab._running)
        _ok('the Cancel button reverted to Close',
            tab.close_btn.GetLabel() == 'Close')
        _ok('nothing was left cancelled', not tab._cancel_requested)
        _ok('the worker reported no error', tab._operation_error is None)

        n = len(applied.get('tracks') or [])
        print(f"\n  ({n} track(s) emitted through the threaded path)")
    finally:
        dialog.Destroy()

    print('\nFAIL: %d check(s)' % len(BAD) if BAD else '\nOK')
    for b in BAD:
        print('  - ' + b)
    return 1 if BAD else 0


if __name__ == '__main__':
    try:
        import wx          # noqa: F401
        import pcbnew      # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))
    sys.exit(main())
