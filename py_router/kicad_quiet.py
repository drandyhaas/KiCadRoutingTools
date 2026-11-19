"""Keep a headless pcbnew process from opening a modal wx dialog.

pcbnew reports load warnings through wx's log. Its default target shows them
as a modal message box, which nobody dismisses in a child process, so
``pcbnew.LoadBoard`` never returns. A board that names a font the machine
lacks ("Font 'X' not found; substituting 'Noto Sans Regular'") is enough: the
KiCad refill subprocess then burns its whole timeout on every call
(rp2040-devboard: three 300 s timeouts, the plane-tap step killed at 1163 s;
the same refill with the log on stderr takes 0.8 s).

``QUIET_WX_SOURCE`` is the same code as text, for the scripts this package
writes out and runs under KiCad's python, which may not have py_router on
sys.path. Call it (or exec the source) BEFORE the first LoadBoard.
"""

QUIET_WX_SOURCE = """\
try:
    import wx
except ImportError:
    wx = None
if wx is not None:
    # wx.App resets the log target, so the target is set after it exists.
    _quiet_wx_app = wx.GetApp() or wx.App(False)
    wx.Log.SetActiveTarget(wx.LogStderr())
"""


def quiet_wx():
    """Route wx log messages to stderr; returns the wx.App (keep it alive)."""
    scope = {}
    exec(QUIET_WX_SOURCE, scope)
    return scope.get('_quiet_wx_app')
