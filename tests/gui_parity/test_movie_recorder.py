#!/usr/bin/env python3
"""Regression gate for the GUI's routing-movie recorder (#506).

The Advanced tab's **Make routing movie** debug checkbox (default OFF) makes the
GUI record what it routes and write the same movie `make_movie.py` makes. The
GUI-only half is what can silently rot, so this drives the REAL
`swig_gui.RoutingDialog` headless (parent None, never shown) and asserts:

  1. the checkbox exists in the Advanced tab's debug section and defaults OFF,
     and nothing is recorded while it is off;
  2. ticking it arms a baseline, and ONE routing step then renders ONE movie
     covering that step (the four tabs all report through
     `movie_recorder.record_movie_step`);
  3. a plan run (`begin_group` ... `end_group`, what the AI tab's Run
     Selected Steps buttons bracket) renders ONE movie for ALL its steps,
     not one per step;
  4. the finished path is logged in GREEN (ANSI 92, which the Log tab renders);
  5. `reset_params_to_defaults` does NOT untick it -- the plan executor calls
     that before every step, so resetting it would abandon a plan's movie
     halfway -- while an explicit "reset all settings" does.

Needs KiCad's pcbnew + wx; skips if absent.
Run: python3 tests/gui_parity/test_movie_recorder.py
"""

# ---------------------------------------------------------------------------
# macOS: if this HANGS at ~0% CPU, it is NOT wx, machine load, or a deadlock.
#
# After any wx process here is killed (a pkill, a timeout, a crash), macOS
# decides the app "quit unexpectedly", and the NEXT headless launch stops inside
# NSApplication bootstrap showing the restore-windows alert you cannot see:
#     -[NSPersistentUIRestorer promptToIgnorePersistentState]
#         -> -[NSAlert runModal]
# Headless, nobody can click it, so it waits forever: process state SN accruing
# ~0.3s of CPU over many minutes, which reads exactly like a hang. This cost a
# full session of ".gui-parity-checked" markers recording "wx blocked, gate NOT
# RUN" -- the gates were fine the whole time.
#
#   diagnose:  sample <pid> 3 -mayDie | grep -E "NSAlert|PersistentUI"
#   fix:       defaults write -g ApplePersistenceIgnoreState -bool YES
#
# A sandboxed HOME does NOT help -- cfprefsd serves that pref per-user
# regardless of HOME. With the default set, test_gui_engine_parity.py runs ~90s.
# ---------------------------------------------------------------------------
import os
import subprocess
import sys
import time

# wx debug builds assert on about_tab's sizer flags when the dialog is built
# headless; harmless in the real plugin. Must precede the wx import.
os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

#: Hand-written candidates, kept only as the fallback. The REAL list comes
#: from `kicad_exact_fill.kicad_python_candidates()` -- CALL the finder, do not
#: mirror it.
#:
#: This list had the same defect #647 fixed in the engine and never propagated
#: here: no KiCad >= 6 installs at the UNVERSIONED
#: `C:\Program Files\KiCad\bin\python.exe` (it is `...\KiCad\10.0\bin\...`),
#: so on Windows this gate printed `SKIP: no python with pcbnew + wx found` and
#: exited 0 on a machine with KiCad 10.0.0 and wx 4.2.2 installed and working.
#: A gate that self-skips on a capable machine is worse than one that fails:
#: it reports every claim it guards as checked.
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
]

try:                                       # the engine's own finder, verified
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    from kicad_exact_fill import kicad_python_candidates as _kpc
    KICAD_PYTHONS = list(_kpc()) + [c for c in KICAD_PYTHONS
                                    if c not in set(_kpc())]
except Exception:                                              # noqa: BLE001
    pass

#: `fanout_output.kicad_pcb` has no such file in the repo -- the boards
#: are `fanout_output1` / `fanout_output2` -- so this gate printed
#: `SKIP: fixture board missing` and exited 0, on top of the
#: interpreter defect above. TWO self-skips in a row, both silent.
BOARD = os.path.join(REPO, 'kicad_files', 'fanout_output1.kicad_pcb')
GREEN = '\033[92m'

failures = []


def check(name, ok, detail=''):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f": {detail}" if detail else ''))
    if not ok:
        failures.append(name)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                              capture_output=True).returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                # SUBPROCESS ON WINDOWS, not execv. The CRT re-splits execv's
                # argv on spaces, and the real install path is
                # `C:\Program Files\KiCad\10.0\bin\python.exe` -- so the
                # re-exec ran `C:\Program` with `Files\KiCad\...` as an
                # argument and died before the gate started.
                if os.name == 'nt':
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("SKIP: no python with pcbnew + wx found")
    sys.exit(0)


def movie_lines(log_lines):
    """The GREEN "Routing movie: <path>" lines -- a FINISHED movie. Matching on
    '.mp4' alone would also match the "rendering ... -> x.mp4 ..." announcement,
    which is not a result."""
    return [l for l in log_lines if l.startswith(GREEN + 'Routing movie: ')]


def _wait_for_render(recorder, log_lines, timeout=120):
    """Renders run on a worker thread and report via wx.CallAfter, so pump the
    event loop until the finished-movie line lands."""
    import wx
    deadline = time.time() + timeout
    while time.time() < deadline:
        wx.YieldIfNeeded()
        if movie_lines(log_lines) or not recorder.pending():
            wx.YieldIfNeeded()
            if movie_lines(log_lines):
                return True
        time.sleep(0.1)
    return bool(movie_lines(log_lines))


def main():
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import pcbnew
    import wx
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui
    from kicad_routing_plugin.movie_recorder import record_movie_step

    if not os.path.exists(BOARD):
        print(f"SKIP: fixture board missing ({BOARD})")
        return 0

    app = wx.App(False)
    board = pcbnew.LoadBoard(BOARD)
    pcbnew.GetBoard = lambda: board       # how the plugin reads the live board
    dialog = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), BOARD)
    recorder = dialog.movie_recorder

    # Capture the Log tab's text instead of writing movies next to the fixture
    # board: the recorder logs through the dialog's ANSI-aware _append_log.
    log_lines = []
    dialog._append_log = lambda text: log_lines.append(text)
    dialog.board_filename = None          # -> movies land in the temp dir

    print("1. the checkbox is in the Advanced tab's debug section, default off")
    check("Make routing movie checkbox exists", hasattr(dialog, 'make_movie_check'))
    if not hasattr(dialog, 'make_movie_check'):
        return 1
    check("default is unchecked", dialog.make_movie_check.GetValue() is False)
    check("recorder reports disabled", recorder.enabled() is False)
    record_movie_step(dialog, 'route')
    check("nothing is recorded while unchecked", recorder._frames == []
          and recorder._dir is None)

    print("2. one routing step -> one movie")
    dialog.make_movie_check.SetValue(True)
    recorder.on_toggle()
    check("ticking it arms a baseline",
          bool(recorder._baseline) and os.path.exists(recorder._baseline))
    log_lines.clear()
    record_movie_step(dialog, 'route')
    check("the step was captured", len(recorder._frames) == 0,   # rendered+reset
          f"frames left: {recorder._frames}")
    rendered = _wait_for_render(recorder, log_lines)
    check("a movie was rendered", rendered,
          "".join(log_lines).strip() or "(no movie line logged)")
    movie_line = (movie_lines(log_lines) or [''])[0]
    check("its path is logged in GREEN", movie_line.startswith(GREEN),
          repr(movie_line[:60]))
    path = movie_line.split('Routing movie: ')[-1].split('\033')[0].strip()
    check("the movie file exists", bool(path) and os.path.exists(path), path)
    check("the end state becomes the next baseline",
          recorder._baseline and recorder._baseline.endswith('.kicad_pcb'))

    print("3. a plan run -> ONE movie for all its steps")
    log_lines.clear()
    # The AI tab is what brackets a real plan run (Run Selected Steps ->
    # begin_group, plan finished -> end_group); assert it resolves THIS recorder.
    check("the AI tab reaches the dialog's recorder",
          dialog.ai_tab._movie_recorder() is recorder)
    recorder.begin_group()
    record_movie_step(dialog, 'fanout')
    record_movie_step(dialog, 'route_diff')
    record_movie_step(dialog, 'planes')
    check("steps accumulate instead of rendering", len(recorder._frames) == 3,
          str(len(recorder._frames)))
    check("no movie logged mid-plan", not movie_lines(log_lines))
    recorder.end_group()
    rendered = _wait_for_render(recorder, log_lines)
    check("one movie after the plan", rendered)
    done = movie_lines(log_lines)
    check("exactly one movie for the whole plan", len(done) == 1, str(len(done)))
    check("covering all 3 steps + the baseline",
          "rendering 4 board(s)" in "".join(log_lines),
          next((l.strip() for l in log_lines if 'rendering' in l), ''))

    print("4. the per-step parameter reset must NOT untick it")
    dialog.reset_params_to_defaults()
    check("still ticked after reset_params_to_defaults",
          dialog.make_movie_check.GetValue() is True)
    dialog._reset_all_settings()
    check("an explicit reset-all DOES untick it",
          dialog.make_movie_check.GetValue() is False)

    try:
        app.ProcessPendingEvents()
        dialog.Destroy()
        app.ProcessPendingEvents()
    except Exception:
        pass

    print()
    if failures:
        print(f"FAILED ({len(failures)}): {failures}")
        return 1
    print("PASS: the routing-movie recorder records, groups, and logs in green")
    return 0


if __name__ == '__main__':
    sys.exit(main())
