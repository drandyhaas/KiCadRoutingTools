#!/usr/bin/env python3
"""#1698: a ceiling typed into the dialog reads as a plan's and the CLI's does, and a
floor pin reaches the log of the run it concerns (#1691, #1699).

A value above the Default class used to be pinned to Default as it was typed, with a popup
saying it had no effect. It has one: the base stays min(Default, ceiling), and every other
class is capped at the value. So typing, a plan and `--clearance-ceiling` now read alike.

The pin warning was printed while a run's parameters were read, before its tab starts
capturing stdout, so it reached the terminal and never the log a GUI user reads.

Run with plain python3; it re-execs into KiCad's. Seconds, no routing.
"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from unittest.mock import patch

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
REPO = Path(__file__).resolve().parents[2]
for sub in ('', 'py_router', 'py_tools', 'py_placer'):
    sys.path.insert(0, str(REPO / sub))

BOARD = REPO / 'kicad_files' / 'flat_hierarchy.kicad_pcb'     # Default 0.2, Wide 0.4
FAILS = []


def check(name, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f': {detail}' if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def reexec():
    from kicad_locate import kicad_python_candidates
    for py in kicad_python_candidates():
        if py != sys.executable and os.path.exists(py) and subprocess.run(
                [py, '-c', 'import pcbnew, wx'], capture_output=True).returncode == 0:
            sys.exit(subprocess.run([py, '-X', 'utf8', os.path.abspath(__file__)]).returncode)
    print('NOT RUN: no python with pcbnew + wx found')
    sys.exit(2)


class Event:
    def Skip(self):
        pass


def main():
    try:
        import pcbnew
        import wx
    except ImportError:
        reexec()
    from copy_board import copy_board
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import gui_utils, swig_gui

    app = wx.App(False)
    wx.Log.SetActiveTarget(wx.LogStderr())
    assert app is not None
    with tempfile.TemporaryDirectory(prefix='krt1698t-') as tmp:
        path = os.path.join(tmp, 'board.kicad_pcb')
        copy_board(str(BOARD), path)
        board = pcbnew.LoadBoard(path)
        with patch.object(pcbnew, 'GetBoard', return_value=board):
            dlg = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board), path)
            try:
                dflt = swig_gui._get_netclass_parameters('Default')['clearance']
                if abs(dflt - 0.2) > 1e-9:
                    raise AssertionError(f'BROKEN TEST: Default reads {dflt}, not 0.2')

                def type_value(value, ceiling=True):
                    dlg.reset_params_to_defaults()
                    dlg.clearance_check.SetValue(True)
                    dlg.clearance_ceiling_check.SetValue(ceiling)
                    dlg.clearance.SetValue(value)
                    popups = []
                    with patch.object(wx, 'CallAfter', lambda f, *a, **k: popups.append(a)):
                        dlg._on_drc_param_changed(Event(), 'clearance')
                    return popups

                print('A. typed above the Default class, ceiling box on')
                popups = type_value(0.3)
                check('the value stays as typed', abs(dlg.clearance.GetValue() - 0.3) < 1e-9,
                      str(dlg.clearance.GetValue()))
                check('no popup says it has no effect',
                      not any('no effect' in str(a) for a in popups), str(popups))
                check('the classes are capped at it, as --clearance-ceiling',
                      abs(dlg._class_ceiling() - 0.3) < 1e-9, str(dlg._class_ceiling()))
                check('the base stays min(Default, ceiling), as the CLI',
                      abs(dlg._effective_clearance() - 0.2) < 1e-9, str(dlg._effective_clearance()))

                print('B. typed below the physical floor: the fab-floor pin still stands')
                popups = type_value(0.05)
                check('it is pinned up to the floor', abs(dlg.clearance.GetValue() - 0.1) < 1e-9,
                      str(dlg.clearance.GetValue()))
                check('and says so', any('fab floor' in str(a) for a in popups), str(popups))

                print('C. typed between the floor and Default')
                popups = type_value(0.15)
                check('it stands, and caps every class',
                      abs(dlg.clearance.GetValue() - 0.15) < 1e-9
                      and abs(dlg._class_ceiling() - 0.15) < 1e-9, str(dlg._class_ceiling()))

                print('D. without the ceiling box a value is the Default class, as before')
                type_value(0.3, ceiling=False)
                check('no class ceiling', dlg._class_ceiling() is None, str(dlg._class_ceiling()))
                check('the base is the value', abs(dlg._effective_clearance() - 0.3) < 1e-9,
                      str(dlg._effective_clearance()))

                print("E. a pin read before a capture starts reaches that capture's log")
                del gui_utils._log_notes[:]
                dlg.reset_params_to_defaults()
                dlg.clearance_check.SetValue(True)
                dlg.clearance_ceiling_check.SetValue(True)
                dlg._drc_validating = True      # a plan sets the control with no event
                dlg.clearance.SetValue(0.0889)
                dlg._drc_validating = False
                dlg._ceiling_pin_said = None
                check('the reading is pinned', abs(dlg._class_ceiling() - 0.1) < 1e-9,
                      str(dlg._class_ceiling()))
                log = []
                gui_utils.StdoutRedirector(log.append, None)
                text = ''.join(log)
                check('the log has the warning',
                      'is below the physical fab floor' in text and '--clearance-ceiling' in text,
                      repr(text))
                log2 = []
                gui_utils.StdoutRedirector(log2.append, None)
                check('and only the once', not log2, repr(log2))

                print('F. inside a capture it is said at once, with nothing held over')
                log3 = []
                saved = sys.stdout
                sys.stdout = gui_utils.StdoutRedirector(log3.append, None)
                try:
                    gui_utils.note_for_the_log('said now')
                finally:
                    sys.stdout = saved
                check('it is in that log', 'said now' in ''.join(log3), repr(log3))
                check('nothing is held', not gui_utils._log_notes, repr(gui_utils._log_notes))
            finally:
                dlg.Destroy()
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('PASS: #1698 a typed ceiling reads as the CLI, and a floor pin reaches its log')
    return 0


if __name__ == '__main__':
    sys.exit(main())
