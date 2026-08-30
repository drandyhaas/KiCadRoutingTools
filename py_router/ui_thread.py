"""Run wx-backed pcbnew calls on the wx MAIN thread (issue #688).

Engine-side on purpose: the routing engine runs on the plugin's worker thread
and makes pcbnew calls of its own (the live-fill provider in
``kicad_parser.build_pcb_data_from_board``), so the guard cannot live only in
``kicad_routing_plugin``. wx is imported LAZILY -- the CLI has no wx, and there
this degrades to a direct call.

Why this module exists
----------------------
Issue #688 (Windows 11 / KiCad 10.0.5): the plugin froze with KiCad at ~0% CPU
and the dialog "Ne repond pas". A py-spy dump of the frozen process named both
sides of the deadlock:

    MainThread (idle):        _run_plugin (action_plugin.py:164)   <- ShowModal
    Thread-3 (_run_routing):  SaveBoard (pcbnew.py:9899)
                              _stage_live_board (swig_gui.py:3345)
                              batch_route (route.py:4115)

``route.py:4115`` at v0.21.2 is the plane finalize's ``stage_board_fn()``, so
the routing WORKER was calling ``pcbnew.SaveBoard`` -- a wxWidgets-backed C++
call -- while the UI thread sat in its modal loop. That is the rule
``gui_utils.ui_thread_status`` already states ("only ever touch wx from the main
thread; off-thread callers are marshalled with CallAfter"), and it explains
every symptom: it needs zones (the finalize oracle leg only runs with plane
pairs, hence "no zones -> no freeze"), it deadlocks rather than spins (0% CPU),
it is Windows-only, and the on-screen message VARIES between runs because that
label is merely the last text painted before the worker blocked -- not where it
is stuck.

There is more than one such call, which is why this is shared: fixing only the
finalize's would have moved the freeze to the next one (the live-fill provider,
reached from ``exact_unconnected`` and ``plane_fragility`` every oracle round).
"""
from __future__ import annotations

SAVE_BOARD_UI_TIMEOUT_S = 120


def _wx():
    try:
        import wx
        return wx
    except Exception:
        return None


def save_board_on_ui_thread(path, board, timeout_s=SAVE_BOARD_UI_TIMEOUT_S,
                            label=""):
    """``pcbnew.SaveBoard(path, board, aSkipSettings=True)`` on the wx MAIN
    thread. True on success, False if it could not be run.

    ``aSkipSettings`` is not optional: KiCad 10's implicit project-settings save
    merges the pre-migration on-disk project JSON with its migrated in-memory
    view and throws on any key whose type changed (KiCad 9 wrote
    ``sheet_component_classes`` as ``[]``, 10 holds an object). With no C++
    handler above a worker thread, that throw aborts ALL of KiCad.

    The timeout is half the fix and earns its keep on its own: if the UI thread
    is not pumping, the wait expires and the caller degrades instead of hanging
    forever. A skipped save costs one optional leg; a deadlock costs the session.
    """
    import pcbnew
    import threading

    wx = _wx()
    box = {}
    abandoned = threading.Event()

    def _save():
        if abandoned.is_set():
            return           # caller stopped waiting; do not do dead work
        try:
            pcbnew.SaveBoard(path, board, aSkipSettings=True)
            box['ok'] = True
        except Exception as e:      # reported to the caller, never raised here
            box['err'] = e

    if wx is None or wx.IsMainThread():
        # CLI (no wx), or already the main thread: nothing to marshal.
        _save()
    else:
        done = threading.Event()

        def _save_and_signal():
            try:
                _save()
            finally:
                done.set()

        wx.CallAfter(_save_and_signal)
        if not done.wait(timeout_s):
            abandoned.set()
            print(f"  (#688 guard{label}: the wx main thread did not run the "
                  f"board save within {timeout_s}s -- skipping rather than "
                  f"blocking)")
            return False
    if 'err' in box:
        print(f"  (#688 guard{label}: board save failed: {box['err']})")
        return False
    return bool(box.get('ok'))
