"""Keep a headless pcbnew process from opening a modal wx dialog.

pcbnew reports load warnings through wx's log. Its default target shows them
as a modal message box, which nobody dismisses in a child process, so
``pcbnew.LoadBoard`` never returns. A board that names a font the machine
lacks ("Font 'X' not found; substituting 'Noto Sans Regular'") is enough: the
KiCad refill subprocess then burns its whole timeout on every call
(measured: three 300 s timeouts, the plane-tap step killed at 1163 s; the
same refill with the log on stderr takes 0.8 s).

``QUIET_WX_SOURCE`` is that code as text, for the scripts this package
writes out and runs under KiCad's python, which may not have py_router on
sys.path. It must run BEFORE the first LoadBoard.

It imports pcbnew before it creates a wx.App. The other order fails where a
display exists: with KiCad 10.0.6 and wxPython 4.2.5 (GTK3) on Linux,
``wx.App(False)`` followed by ``import pcbnew`` raises "SystemError:
initialization of _pcbnew raised unreported exception", so every refill child
died and the exact fill fell back to zone outlines. Headless runs never saw
it, because wx.App raises SystemExit there and no app is made.
"""

QUIET_WX_SOURCE = """\
import pcbnew
try:
    import wx
except ImportError:
    wx = None
if wx is not None:
    # wx.App resets the log target, so the target is set after it exists.
    # Without a display (headless Linux) wx.App raises SystemExit: no app can
    # exist there, so no modal either, and the child goes on without one.
    _quiet_wx_app = wx.GetApp()
    if _quiet_wx_app is None:
        try:
            _quiet_wx_app = wx.App(False)
        except (Exception, SystemExit):
            _quiet_wx_app = None
    wx.Log.SetActiveTarget(wx.LogStderr())
"""

