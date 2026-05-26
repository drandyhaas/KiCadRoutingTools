"""
Runtime check for Python dependencies that KiCad does not bundle.

KiCad ships numpy + wxPython, but not scipy or shapely. When this plugin is
installed via the KiCad PCM (Plugin and Content Manager), no pip step runs,
so we must detect missing dependencies on first invocation and offer to
install them into KiCad's bundled Python.
"""

import subprocess
import sys

import wx


REQUIRED = [
    # (pip name, import test)
    ("scipy", "from scipy.optimize import linear_sum_assignment"),
    ("shapely", "from shapely.geometry import Polygon"),
]


def _missing_packages():
    missing = []
    for pip_name, import_stmt in REQUIRED:
        try:
            exec(import_stmt, {})
        except ImportError:
            missing.append(pip_name)
    return missing


def _pip_install(packages, parent):
    """Run `python -m pip install <packages>` against the current interpreter.

    Returns (success, log_output).
    """
    cmd = [sys.executable, "-m", "pip", "install", "--upgrade", *packages]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=600
        )
    except Exception as e:
        return False, f"pip command failed to start:\n{e}"
    log = (result.stdout or "") + (result.stderr or "")
    return result.returncode == 0, log


def ensure_dependencies(parent=None):
    """Verify that scipy and shapely are importable. If not, prompt the user
    to install them via pip into KiCad's Python. Returns True if all deps are
    present (after any install), False if the user cancelled or install failed.
    """
    missing = _missing_packages()
    if not missing:
        return True

    pkg_list = ", ".join(missing)
    msg = (
        f"KiCad Routing Tools needs the following Python packages that are not "
        f"bundled with KiCad:\n\n"
        f"  {pkg_list}\n\n"
        f"Install them now into KiCad's Python?\n"
        f"({sys.executable})"
    )
    dlg = wx.MessageDialog(
        parent, msg, "Install missing dependencies",
        wx.YES_NO | wx.ICON_QUESTION,
    )
    choice = dlg.ShowModal()
    dlg.Destroy()
    if choice != wx.ID_YES:
        return False

    progress = wx.ProgressDialog(
        "Installing dependencies",
        f"Running pip install {pkg_list}...\n\nThis may take a minute.",
        maximum=100, parent=parent,
        style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE,
    )
    progress.Pulse()
    wx.Yield()

    ok, log = _pip_install(missing, parent)
    progress.Destroy()

    if not ok:
        wx.MessageBox(
            f"pip install failed.\n\n"
            f"You may need to install manually with:\n"
            f"  \"{sys.executable}\" -m pip install {pkg_list}\n\n"
            f"Output:\n{log[-2000:]}",
            "Install failed", wx.OK | wx.ICON_ERROR, parent=parent,
        )
        return False

    still_missing = _missing_packages()
    if still_missing:
        wx.MessageBox(
            "Dependencies were installed but are still not importable in this "
            "KiCad session. Please restart KiCad and try again.",
            "Restart required", wx.OK | wx.ICON_INFORMATION, parent=parent,
        )
        return False

    return True
