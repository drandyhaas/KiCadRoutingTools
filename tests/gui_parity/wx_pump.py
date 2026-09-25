"""Wait for a headless dialog's background work by running a REAL wx MainLoop.

The tabs hand a finished worker's result back to the UI thread through
`wx.CallLater`, which is a wx.Timer. Yield-pumping -- `wx.YieldIfNeeded()` in a
sleep loop -- does not deliver wx.Timer events outside a running event loop on
Windows (MSW), so a gate that waits that way sits out its whole budget while
the worker finished in half a second, then grades an empty result (measured:
the QFN fanout worker done at 0.5 s, `_running` still set 120 s later). macOS
delivers timers either way, which is why such gates were green there. This is
the loop test_placement_fake_run and headless_plan already use.
"""
import time

import wx


def run_until(pred, timeout_s, poll_ms=50):
    """Run wx's MainLoop until `pred()` holds or `timeout_s` passes.

    Returns True if `pred()` held, False on timeout. Safe to call repeatedly.
    """
    if pred():
        return True
    app = wx.GetApp()
    deadline = time.monotonic() + timeout_s
    state = {'ok': False}

    def _poll():
        if pred():
            state['ok'] = True
            app.ExitMainLoop()
        elif time.monotonic() >= deadline:
            app.ExitMainLoop()
        else:
            wx.CallLater(poll_ms, _poll)

    wx.CallLater(poll_ms, _poll)
    app.MainLoop()
    return state['ok']
