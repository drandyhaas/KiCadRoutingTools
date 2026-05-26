"""
Runtime check for Python dependencies that KiCad does not bundle.

KiCad ships numpy + wxPython, but not scipy or shapely. When this plugin is
installed via the KiCad PCM (Plugin and Content Manager), no pip step runs,
so we must detect missing dependencies on first invocation and offer to
install them into KiCad's bundled Python.

The list of required packages is read from `requirements.txt` at the plugin
root, so requirements.txt is the single source of truth for both the CLI
install path (install_plugin.py) and this runtime check.

The pip install runs in a worker thread so the wx event loop keeps ticking
and the progress dialog stays responsive (otherwise KiCad freezes for the
full duration of the install, which can be minutes on a slow network).
"""

import os
import re
import subprocess
import sys
import threading

import wx


# Mapping of pip package name -> import statement used to verify it. The
# specific submodule imports catch broken/partial installs (e.g. shapely
# without its native lib) better than a bare `import pkg`. Packages not in
# this dict fall back to a plain `import <pkg>`.
IMPORT_TESTS = {
    "numpy": "import numpy",
    "scipy": "from scipy.optimize import linear_sum_assignment",
    "shapely": "from shapely.geometry import Polygon",
}

# Pattern matching the package name at the start of a requirements line.
# Stops at the first version-specifier or environment-marker character.
_REQ_NAME_RE = re.compile(r"^\s*([A-Za-z0-9][A-Za-z0-9._-]*)")


def _requirements_path():
    """Path to the requirements.txt at the plugin root.

    deps_check.py lives at <plugin_dir>/kicad_routing_plugin/, so the file
    is one directory up.
    """
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "requirements.txt",
    )


def _parse_requirements(path):
    """Return the list of pip package names listed in requirements.txt.

    Skips comments, blank lines, and pip directives (`-r ...`, URLs, etc.).
    Strips version specifiers so `numpy>=1.21.0` -> `numpy`.
    """
    names = []
    if not os.path.isfile(path):
        return names
    with open(path, "r") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line or line.startswith("-") or "://" in line:
                continue
            m = _REQ_NAME_RE.match(line)
            if m:
                names.append(m.group(1))
    return names


def _required_packages():
    """Return [(pip_name, import_statement)] derived from requirements.txt."""
    return [(name, IMPORT_TESTS.get(name, f"import {name}"))
            for name in _parse_requirements(_requirements_path())]


def _missing_packages():
    missing = []
    for pip_name, import_stmt in _required_packages():
        try:
            exec(import_stmt, {})
        except ImportError:
            missing.append(pip_name)
    return missing


def _find_python_executable():
    """Return the path to the python interpreter for this process.

    Inside KiCad's embedded Python, `sys.executable` is the host C++ binary
    (`pcbnew`), not python3 — so subprocess.run([sys.executable, '-m', 'pip',
    ...]) hangs because pcbnew treats `-m pip install ...` as application
    arguments. Reconstruct the real interpreter path from `sys.prefix`.
    """
    # If sys.executable already looks like a python binary, trust it.
    exe = sys.executable or ""
    if exe and os.path.basename(exe).lower().startswith("python"):
        return exe

    # Otherwise derive from sys.prefix (the Python install dir).
    prefix = sys.prefix or sys.base_prefix
    candidates = []
    if sys.platform == "win32":
        candidates = [
            os.path.join(prefix, "python.exe"),
            os.path.join(prefix, "Scripts", "python.exe"),
        ]
    else:
        for name in (f"python{sys.version_info.major}.{sys.version_info.minor}",
                     f"python{sys.version_info.major}", "python3", "python"):
            candidates.append(os.path.join(prefix, "bin", name))
    for path in candidates:
        if os.path.isfile(path):
            return path
    # Last resort: fall back to sys.executable; will likely fail but produces
    # a clear error message in the install dialog.
    return exe


def _pip_install_threaded(packages, progress):
    """Run `python -m pip install <packages>` in a worker thread while
    pulsing the wx ProgressDialog from the main thread. Returns
    (success, log_output).
    """
    python_exe = _find_python_executable()
    cmd = [python_exe, "-m", "pip", "install", "--upgrade", *packages]

    # Result holder shared with the worker thread.
    result = {"returncode": None, "stdout": "", "stderr": "", "error": None}

    def run():
        try:
            proc = subprocess.run(
                cmd, capture_output=True, text=True, timeout=600
            )
            result["returncode"] = proc.returncode
            result["stdout"] = proc.stdout or ""
            result["stderr"] = proc.stderr or ""
        except Exception as e:
            result["error"] = repr(e)

    worker = threading.Thread(target=run, daemon=True)
    worker.start()

    # Pump the wx event loop while the worker runs. wx.MilliSleep yields
    # without blocking the GUI; Pulse() advances the indeterminate bar.
    while worker.is_alive():
        if not progress.Pulse()[0]:  # user clicked Cancel
            break
        wx.MilliSleep(150)
        wx.YieldIfNeeded()
    worker.join(timeout=5)

    if result["error"] is not None:
        return False, f"pip command failed to start: {result['error']}"
    log = result["stdout"] + result["stderr"]
    return result["returncode"] == 0, log


def ensure_dependencies(parent=None):
    """Verify that scipy and shapely are importable. If not, prompt the user
    to install them via pip into KiCad's Python. Returns True if all deps are
    present (after any install), False if the user cancelled or install failed.
    """
    missing = _missing_packages()
    if not missing:
        return True

    pkg_list = ", ".join(missing)
    python_exe = _find_python_executable()
    msg = (
        f"KiCad Routing Tools needs the following Python packages that are not "
        f"bundled with KiCad:\n\n"
        f"  {pkg_list}\n\n"
        f"Install them now into KiCad's Python?\n"
        f"({python_exe})"
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
        style=wx.PD_APP_MODAL | wx.PD_CAN_ABORT | wx.PD_AUTO_HIDE,
    )

    ok, log = _pip_install_threaded(missing, progress)
    progress.Destroy()

    if not ok:
        wx.MessageBox(
            f"pip install failed.\n\n"
            f"You may need to install manually with:\n"
            f"  \"{python_exe}\" -m pip install {pkg_list}\n\n"
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
