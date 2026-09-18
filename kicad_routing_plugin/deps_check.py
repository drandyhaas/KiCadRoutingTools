"""
Runtime check for Python dependencies that KiCad does not bundle.

KiCad ships wxPython, and -- depending on the version and platform -- may ship
neither scipy nor shapely nor numpy. KiCad 10.0.0 on macOS bundles NO numpy at
all (verified 2026-09-18: `find KiCad.app -name numpy` is empty), so do not
assume any of them is present; earlier notes here claimed numpy was bundled and
that was the assumption this gate was built on. When this plugin is installed
via the KiCad PCM (Plugin and Content Manager) no pip step runs, so we must
detect missing dependencies on first invocation and offer to install them into
KiCad's bundled Python.

The list of packages is read from `requirements.txt` at the plugin root, so
requirements.txt is the single source of truth for both the CLI install path
(install_plugin.py) and this runtime check.

TOO OLD counts as missing (2026-09-18). The probe used to be `except
ImportError` and nothing else, and the specifiers were deliberately stripped
(`numpy>=1.21.0` -> `numpy`), so an ancient numpy passed the gate, raised no
dialog, and could not be repaired by the one-click install either -- `pip
install --upgrade` runs only for packages that are MISSING, and that numpy was
not missing. What the user saw instead was scipy's "A NumPy version >=1.22.4
... is required" and `TypeError: 'numpy._DTypeMeta' object is not
subscriptable`, neither of which names the plugin, and both of which are
checked by running `pip show numpy` against a completely different interpreter
-- where it is up to date. Hence `startup_checks.dependency_problems`, which
reports the version and the FILE the running interpreter actually imports.

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
import subprocess
import sys
import sysconfig
import threading

import wx

# `action_plugin` puts these on sys.path at import time, but this module is also
# imported directly (tests, install tooling), so repair the path here rather
# than depend on the order. The flat PCM layout has startup_checks beside the
# plugin directory; the repo layout has it in py_router/ (#522).
_PLUGIN_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in (_PLUGIN_ROOT, os.path.join(_PLUGIN_ROOT, 'py_router')):
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.insert(0, _d)


# The dependency tables and the probes live in `startup_checks` -- wx-free, and
# shared with the CLI gate (`check_python_dependencies`) so the two fronts
# cannot disagree about what is required or how old is too old. IMPORT_TESTS is
# re-exported here because that is the name this module's callers and tests
# already use, and it is the SAME dict object, not a copy.
from startup_checks import (                                   # noqa: E402
    IMPORT_TESTS,
    Problem,
    dependency_problems,
    requirement_floors,
)


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


def _requirements_path():
    """Path to the requirements.txt at the plugin root.

    deps_check.py lives at <plugin_dir>/kicad_routing_plugin/, so the file is
    one directory up -- which is also where `startup_checks.requirements_path`
    looks from py_router/. Both spellings resolve to the same file in the repo
    layout and in a flat PCM install.
    """
    return os.path.join(_PLUGIN_ROOT, "requirements.txt")


def _parse_requirements(path):
    """Return the pip package names listed in a requirements file.

    Names only. The SPECIFIERS are not discarded any more -- they are read by
    `startup_checks.requirement_floors`, which is what turns `numpy>=1.22.0`
    into a gate instead of a comment -- they are simply not this function's
    answer. See `_problems()` for the version-aware probe.
    """
    from startup_checks import parse_requirements
    return [name for name, _spec in parse_requirements(path)]


def _required_packages():
    """Return [(pip_name, import_statement)] derived from requirements.txt."""
    return [(name, IMPORT_TESTS.get(name, f"import {name}"))
            for name in _parse_requirements(_requirements_path())]


def _problems():
    """Every requirements.txt package that is absent OR too old, as Problems.

    One call, both fronts: `startup_checks.dependency_problems` is the same
    function the CLI gate runs, over the same IMPORT_TESTS and the same floors.
    """
    names = _parse_requirements(_requirements_path())
    return dependency_problems(names, floors=requirement_floors())


def _split_problems(problems):
    """(blocking, optional) Problem lists, split by OPTIONAL_PACKAGES."""
    blocking = [p for p in problems if p.name not in OPTIONAL_PACKAGES]
    optional = [p for p in problems if p.name in OPTIONAL_PACKAGES]
    return blocking, optional


def _missing_packages():
    """Return (blocking, optional): the pip NAMES that are absent or too old,
    split by whether the plugin can run without them (`OPTIONAL_PACKAGES`).

    Kept name-based because that is what the callers and #943's test read; the
    detail behind each name is in `_problems()`.
    """
    blocking, optional = _split_problems(_problems())
    return [p.name for p in blocking], [p.name for p in optional]


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
    """The problem packages as display lines, optional ones marked.

    Takes Problem objects, so an outdated package says its version and the FILE
    it was imported from. That path is the whole point: a user told "your numpy
    is too old" checks `pip show numpy` in a terminal, sees a current version,
    and concludes the plugin is wrong -- because KiCad's Python is not the
    Python they checked, and nothing in the old message said so.
    """
    lines = [p.describe() for p in blocking]
    lines += [f"{p.describe()}  (optional -- "
              f"{OPTIONAL_PACKAGES.get(p.name, 'a feature')})"
              for p in optional]
    return "\n".join(lines)


def _report_externally_managed(parent, blocking, optional, marker):
    """Say what to install, instead of offering an install that cannot run.

    Direction (1) of #944, and deliberately not (2): retrying with
    `--break-system-packages` writes into a prefix the distro's own package
    manager owns, which is the user's call to make and not a plugin's to make
    silently. The command is spelled out so making it is one paste.
    """
    python_exe = _find_python_executable()
    names = [p.name for p in blocking + optional]
    requirements = [p.requirement for p in blocking + optional]
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
        f"{' '.join(requirements)}",
        "Install with your package manager", wx.OK | wx.ICON_INFORMATION,
        parent=parent,
    )


def ensure_dependencies(parent=None):
    """Verify the packages the plugin cannot run without are importable AND new
    enough. If any is missing or stale, prompt the user to install them via pip
    into KiCad's Python. Returns True if all BLOCKING deps are satisfied (after
    any install), False if the user cancelled or the install failed.

    A missing `OPTIONAL_PACKAGES` entry never returns False and never raises a
    dialog of its own (#943) -- the feature it serves turns itself off, which is
    the behaviour the plugin already had at those call sites. It IS added to the
    pip command when the prompt fires for a blocking package anyway, so a fresh
    PCM install (which has none of them) still ends up with everything.

    "Stale" is handled by the same path as "absent" (2026-09-18) and the pip
    command carries the FLOOR (`numpy>=1.22`, not `numpy`), because pip
    considers a bare requirement already satisfied by whatever is installed --
    so the one-click install for a too-old package used to be a no-op that
    reported success.
    """
    blocking, optional = _split_problems(_problems())
    if not blocking:
        return True

    marker = _externally_managed()
    if marker is not None:
        _report_externally_managed(parent, blocking, optional, marker)
        return False

    to_install = [p.requirement for p in blocking + optional]
    pkg_list = ", ".join(to_install)
    python_exe = _find_python_executable()
    msg = (
        f"KiCad Routing Tools needs these Python packages, which are missing "
        f"or too old in the Python that KiCad runs:\n\n"
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

    still_missing, still_optional = _split_problems(_problems())
    if not ok and still_missing:
        wx.MessageBox(
            f"pip install failed.\n\n"
            f"You may need to install manually with:\n"
            f"  \"{python_exe}\" -m pip install --upgrade "
            f"{' '.join(chr(34) + r + chr(34) for r in to_install)}\n\n"
            f"Output:\n{log[-2000:]}",
            "Install failed", wx.OK | wx.ICON_ERROR, parent=parent,
        )
        return False
    if not ok:
        # Only the optional half failed. Say so and carry on -- refusing here
        # would block the plugin on a package it does not need (#943).
        names = [p.name for p in still_optional]
        wx.MessageBox(
            f"Installed what the plugin needs, but pip could not install: "
            f"{', '.join(names)}.\n\n"
            f"{_optional_effect(names)}\n\n"
            f"To install later:\n"
            f"  \"{python_exe}\" -m pip install --upgrade "
            f"{' '.join(p.requirement for p in still_optional)}",
            "Some optional packages missing", wx.OK | wx.ICON_INFORMATION,
            parent=parent,
        )

    if still_missing:
        # Two different situations wear this face, so say which. A package that
        # is still ABSENT is almost always loaded-already/needs-a-restart. A
        # package that is still TOO OLD after a successful upgrade is a
        # SHADOWING problem -- pip wrote a new one somewhere that is not first
        # on sys.path -- and restarting KiCad will not fix that, so the path in
        # each line is the thing to act on.
        stale = [p for p in still_missing if p.state == 'outdated']
        detail = _describe_missing(still_missing, [])
        if stale:
            wx.MessageBox(
                "pip finished, but this Python still imports an older copy:\n\n"
                f"{detail}\n\n"
                "Another copy earlier on sys.path is shadowing the one pip "
                "just installed, so restarting KiCad will not help. Remove or "
                "update the copy at the path above.",
                "Older copy still being imported", wx.OK | wx.ICON_ERROR,
                parent=parent,
            )
        else:
            wx.MessageBox(
                "Dependencies were installed but are still not importable in "
                f"this KiCad session:\n\n{detail}\n\n"
                "Please restart KiCad and try again.",
                "Restart required", wx.OK | wx.ICON_INFORMATION, parent=parent,
            )
        return False

    return True
