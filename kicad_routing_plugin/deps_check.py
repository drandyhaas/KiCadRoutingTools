"""
Runtime check for Python dependencies that KiCad does not bundle.

KiCad ships numpy + wxPython, but not scipy or shapely. When this plugin is
installed via the KiCad PCM (Plugin and Content Manager), no pip step runs,
so we must detect missing dependencies on first invocation and offer to
install them into KiCad's bundled Python.

The list of packages is read from `requirements.txt` at the plugin root, so
requirements.txt is the single source of truth for both the CLI install path
(install_plugin.py) and this runtime check.

Not every one of them BLOCKS, though (#943). `OPTIONAL_PACKAGES` names the ones
whose absence disables a feature instead of stopping the plugin; only the rest
gate `ensure_dependencies`.

The pip install runs in a worker thread so the wx event loop keeps ticking
and the progress dialog stays responsive (otherwise KiCad freezes for the
full duration of the install, which can be minutes on a slow network).

It is not offered at all on a PEP 668 interpreter (#944) -- see
`_externally_managed`.
"""

import os
import re
import subprocess
import sys
import sysconfig
import threading

import wx


# Mapping of pip package name -> import statement used to verify it. The
# specific submodule imports catch broken/partial installs (e.g. shapely
# without its native lib) better than a bare `import pkg`. Packages not in
# this dict fall back to a plain `import <pkg>`.
#
# That fallback is only right while the pip name IS the import name, and it
# fails SILENTLY when it is not (#943): Pillow imports as `PIL`, so `import
# Pillow` raised ModuleNotFoundError on every machine and the probe reported a
# package that was installed as missing -- a dialog no install could dismiss,
# in front of a plugin that then refused to open. Every name in
# requirements.txt needs an entry here unless the two names are identical, and
# `tests/test_943_optional_render_dependency.py` checks that against the
# installed distributions rather than against this comment.
IMPORT_TESTS = {
    "numpy": "import numpy",
    "scipy": "from scipy.optimize import linear_sum_assignment",
    "shapely": "from shapely.geometry import Polygon",
    "Pillow": "from PIL import Image",
}

# Packages the plugin can RUN without. They are still installed by the prompt
# below when it fires for something else -- on the PCM path this dialog is the
# only installer there is -- but a machine that lacks one gets the plugin, with
# the feature that needs it turned off.
#
# Pillow is the RASTER path and nothing else (#943): the "Make routing movie"
# checkbox (movie_recorder.py) and the placement tab's preview image
# (placement_gui.py), both of which already disable themselves on ImportError.
# Routing needs none of it -- `startup_checks.check_render_dependencies` is a
# separate gate for exactly this reason, and gating the GUI on Pillow here
# re-made, one layer up, the mistake that function was written to record.
OPTIONAL_PACKAGES = {
    "Pillow": "board renders, the placement preview and the routing movie",
}


def _optional_effect(names):
    """One line per optional package saying what turns off without it."""
    return "\n".join(
        f"Without {n}: {OPTIONAL_PACKAGES.get(n, 'a feature')} are unavailable. "
        f"Everything else works."
        for n in names
    )


# Distro package names for the PEP 668 path (#944), Debian/Ubuntu spelling.
# Fedora agrees on all but Pillow (python3-pillow), which the message SAYS
# rather than guesses from a distro sniff that would be wrong on the next one.
DISTRO_PACKAGES = {
    "numpy": "python3-numpy",
    "scipy": "python3-scipy",
    "shapely": "python3-shapely",
    "Pillow": "python3-pil",
}
FEDORA_PACKAGES = dict(DISTRO_PACKAGES, Pillow="python3-pillow")


def _distro_command(names, table):
    return " ".join(table.get(n, n.lower()) for n in names)


def _externally_managed():
    """Path of this interpreter's PEP 668 EXTERNALLY-MANAGED marker, or None.

    #944: KiCad's Linux packages run the SYSTEM interpreter, and on Ubuntu
    23.04+, Debian 12+ and Fedora 38+ that prefix is marked externally managed
    -- pip refuses every install into it, `--user` included. The one-click
    install offered below therefore cannot succeed there, and offering it
    anyway spends a progress dialog to arrive at `error:
    externally-managed-environment` in a log tail.

    A venv is exempt even when its base prefix carries the marker (that is what
    PEP 668 is FOR, and `sysconfig.get_path('stdlib')` inside a venv still
    resolves to the base stdlib, so the file would be found), hence the prefix
    test first.
    """
    if sys.prefix != getattr(sys, "base_prefix", sys.prefix):
        return None
    for key in ("stdlib", "platstdlib"):
        try:
            directory = sysconfig.get_path(key)
        except Exception:
            continue
        if directory:
            marker = os.path.join(directory, "EXTERNALLY-MANAGED")
            if os.path.isfile(marker):
                return marker
    return None


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
    """Return (blocking, optional): the pip names that do not import, split by
    whether the plugin can run without them (`OPTIONAL_PACKAGES`).
    """
    blocking, optional = [], []
    for pip_name, import_stmt in _required_packages():
        try:
            exec(import_stmt, {})
        except ImportError:
            (optional if pip_name in OPTIONAL_PACKAGES else blocking).append(pip_name)
    return blocking, optional


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


def _describe_missing(blocking, optional):
    """The missing packages as display lines, optional ones marked."""
    lines = [f"  {name}" for name in blocking]
    lines += [f"  {name}  (optional -- {OPTIONAL_PACKAGES.get(name, 'a feature')})"
              for name in optional]
    return "\n".join(lines)


def _report_externally_managed(parent, blocking, optional, marker):
    """Say what to install, instead of offering an install that cannot run.

    Direction (1) of #944, and deliberately not (2): retrying with
    `--break-system-packages` writes into a prefix the distro's own package
    manager owns, which is the user's call to make and not a plugin's to make
    silently. The command is spelled out so making it is one paste.
    """
    python_exe = _find_python_executable()
    names = blocking + optional
    debian = _distro_command(names, DISTRO_PACKAGES)
    fedora = _distro_command(names, FEDORA_PACKAGES)
    wx.MessageBox(
        "KiCad Routing Tools needs the following Python packages that are not "
        "bundled with KiCad:\n\n"
        f"{_describe_missing(blocking, optional)}\n\n"
        "This Python is managed by your distribution (PEP 668):\n"
        f"  {marker}\n\n"
        "pip cannot install into it, so no one-click install is offered. "
        "Install the distribution's own packages instead:\n\n"
        f"  sudo apt install {debian}\n"
        f"  (Fedora: sudo dnf install {fedora})\n\n"
        "If your distribution has no package for one of them, the deliberate "
        "override is:\n"
        f"  \"{python_exe}\" -m pip install --break-system-packages "
        f"{' '.join(names)}",
        "Install with your package manager", wx.OK | wx.ICON_INFORMATION,
        parent=parent,
    )


def ensure_dependencies(parent=None):
    """Verify the packages the plugin cannot run without are importable. If any
    is missing, prompt the user to install them via pip into KiCad's Python.
    Returns True if all BLOCKING deps are present (after any install), False if
    the user cancelled or the install failed.

    A missing `OPTIONAL_PACKAGES` entry never returns False and never raises a
    dialog of its own (#943) -- the feature it serves turns itself off, which is
    the behaviour the plugin already had at those call sites. It IS added to the
    pip command when the prompt fires for a blocking package anyway, so a fresh
    PCM install (which has none of them) still ends up with everything.
    """
    blocking, optional = _missing_packages()
    if not blocking:
        return True

    marker = _externally_managed()
    if marker is not None:
        _report_externally_managed(parent, blocking, optional, marker)
        return False

    to_install = blocking + optional
    pkg_list = ", ".join(to_install)
    python_exe = _find_python_executable()
    msg = (
        f"KiCad Routing Tools needs the following Python packages that are not "
        f"bundled with KiCad:\n\n"
        f"{_describe_missing(blocking, optional)}\n\n"
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

    ok, log = _pip_install_threaded(to_install, progress)
    progress.Destroy()

    still_missing, still_optional = _missing_packages()
    if not ok and still_missing:
        wx.MessageBox(
            f"pip install failed.\n\n"
            f"You may need to install manually with:\n"
            f"  \"{python_exe}\" -m pip install {pkg_list}\n\n"
            f"Output:\n{log[-2000:]}",
            "Install failed", wx.OK | wx.ICON_ERROR, parent=parent,
        )
        return False
    if not ok:
        # Only the optional half failed. Say so and carry on -- refusing here
        # would block the plugin on a package it does not need (#943).
        wx.MessageBox(
            f"Installed what the plugin needs, but pip could not install: "
            f"{', '.join(still_optional)}.\n\n"
            f"{_optional_effect(still_optional)}\n\n"
            f"To install later:\n"
            f"  \"{python_exe}\" -m pip install {' '.join(still_optional)}",
            "Some optional packages missing", wx.OK | wx.ICON_INFORMATION,
            parent=parent,
        )

    if still_missing:
        wx.MessageBox(
            "Dependencies were installed but are still not importable in this "
            "KiCad session. Please restart KiCad and try again.",
            "Restart required", wx.OK | wx.ICON_INFORMATION, parent=parent,
        )
        return False

    return True
