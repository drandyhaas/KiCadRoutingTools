"""The Differential tab resolves a 0 class diff_pair_width / diff_pair_gap the
way route_diff.py does (found reviewing #1276).

routed_output's Default class stores diff_pair_width 0 and diff_pair_gap 0,
and KiCad 10 reads both as set to 0. With the overrides unchecked the tab
took its own control values (0.3 / 0.101: it tested the 0 for truth) while
route_diff.py took the 0 itself -- the two fronts routed the same board at
different pair geometry. Both now resolve through
list_nets.class_diff_pair_geometry: a 0 is unset and falls back to the class
track width / clearance, 0.09 / 0.09 here.

Rows, on the REAL headless dialog with the overrides unchecked:
  - Default class 0 / 0: the tab's width and gap are the CLI's resolution of
    the same project (0.09 / 0.09), not the control values;
  - Default class declaring 0.2 / 0.15 (control): the tab takes them, as it
    always did.

Run with KiCad Python (wx + pcbnew required):
    python3 tests/gui_parity/test_zero_diff_pair_width_gui.py
Exits 2 when wx/pcbnew are missing.
"""
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
REPO = Path(__file__).resolve().parents[2]
for subdir in ('', 'py_router', 'py_tools'):
    sys.path.insert(0, str(REPO / subdir))

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def main():
    try:
        import pcbnew
        import wx
    except ImportError as exc:
        print(f'ERROR: KiCad Python with wx + pcbnew is required: {exc}')
        return 2
    from copy_board import copy_board
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import swig_gui

    app = wx.App(False)
    assert app is not None
    with tempfile.TemporaryDirectory(prefix='zero_dpw_gui_') as tmp:
        board_path = Path(tmp) / 'board.kicad_pcb'
        copy_board(str(REPO / 'kicad_files/routed_output.kicad_pcb'), str(board_path))
        board = pcbnew.LoadBoard(str(board_path))
        nc = board.GetDesignSettings().m_NetSettings.GetDefaultNetclass()
        # route_diff.py's resolution of this project (tests/
        # test_diff_pair_zero_class_width.py holds the CLI to it): the class
        # track width and clearance
        cli = (pcbnew.ToMM(nc.GetTrackWidth()), pcbnew.ToMM(nc.GetClearance()))
        with patch.object(pcbnew, 'GetBoard', return_value=board):
            dlg = swig_gui.RoutingDialog(None, build_pcb_data_from_board(board),
                                         str(board_path))
            try:
                tab = dlg.differential_tab
                tab.diff_pair_width_check.SetValue(False)
                tab.diff_pair_gap_check.SetValue(False)
                tab.diff_pair_width.SetValue(0.3)
                tab.diff_pair_gap.SetValue(0.101)
                live = (pcbnew.ToMM(nc.GetDiffPairWidth()), pcbnew.ToMM(nc.GetDiffPairGap()))
                check('precondition: KiCad reads the project Default pair width / gap as 0',
                      live == (0.0, 0.0), str(live))
                got = (tab._effective_diff_pair_width(), tab._effective_diff_pair_gap())
                check('Default 0 / 0: the tab resolves what route_diff.py resolves',
                      got == cli == (0.09, 0.09), f'tab {got}, CLI {cli}')
                nc.SetDiffPairWidth(pcbnew.FromMM(0.2))
                nc.SetDiffPairGap(pcbnew.FromMM(0.15))
                got = (tab._effective_diff_pair_width(), tab._effective_diff_pair_gap())
                check('Default 0.2 / 0.15 (control): the tab takes the class values',
                      got == (0.2, 0.15), str(got))
            finally:
                dlg.Destroy()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
