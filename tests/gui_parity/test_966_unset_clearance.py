"""#966: real dialog clearance resolution, including every shared tab config.

Run with KiCad Python (wx + pcbnew required):
    python3 tests/gui_parity/test_966_unset_clearance.py

This gate measures parameter delivery, not copper quality. Public-board routing,
written geometry, DRC and connectivity evidence accompanies the issue report.
Only a temporary copy of splitflap_driver is loaded/mutated.
"""
import json
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
REPO = Path(__file__).resolve().parents[2]
for subdir in ('', 'py_router', 'py_tools'):
    sys.path.insert(0, str(REPO / subdir))


def main():
    try:
        import pcbnew
        import wx
    except ImportError as exc:
        print(f'ERROR: KiCad Python with wx + pcbnew is required: {exc}')
        return 2
    from copy_board import copy_board
    from kicad_parser import build_pcb_data_from_board
    from kicad_routing_plugin import ai_plan, swig_gui

    app = wx.App(False)
    assert app is not None
    rows = []
    with tempfile.TemporaryDirectory(prefix='krt966-') as tmp:
        board_path = Path(tmp) / 'board.kicad_pcb'
        copy_board(str(REPO / 'kicad_files/splitflap_driver.kicad_pcb'),
                   str(board_path))
        board = pcbnew.LoadBoard(str(board_path))
        with patch.object(pcbnew, 'GetBoard', return_value=board):
            dlg = swig_gui.RoutingDialog(
                None, build_pcb_data_from_board(board), str(board_path))
            nc = board.GetDesignSettings().m_NetSettings.GetDefaultNetclass()
            try:
                assert board.GetCopperLayerCount() == 2
                fine = Path(tmp) / 'fine.fab'
                fine.write_text('clearance = 0.08\n', encoding='utf-8')
                # Declared zero, below/at/above the floor, positive control,
                # a different tier, and the user's supported fab capability.
                for tier, overrides, declared, expected in (
                    ('auto', '', 0.0, 0.10),
                    ('auto', '', 0.05, 0.10),
                    ('auto', '', 0.10, 0.10),
                    ('auto', '', 0.100001, 0.100001),
                    ('auto', '', 0.20, 0.20),
                    # Initial parameter pinning uses the physical floor;
                    # the selected tier bounds later automatic descents.
                    ('standard', '', 0.0, 0.10),
                    ('advanced', '', 0.0, 0.10),
                    ('auto', str(fine), 0.0, 0.08),
                ):
                    dlg.reset_params_to_defaults()
                    nc.SetClearance(pcbnew.FromMM(declared))
                    dlg.fab_tier.SetStringSelection(tier)
                    dlg.fab_overrides_path.SetValue(overrides)
                    # An unchecked spin may retain a previous user value.
                    dlg.clearance.SetValue(0.42)
                    assert not dlg.clearance_check.GetValue()
                    got = assert_configs(dlg, expected)
                    rows.append(dict(declared_mm=declared, tier=tier,
                                     fab_override_mm=0.08 if overrides else None,
                                     requested_mm=None, effective_mm=got))

                dlg.reset_params_to_defaults()
                nc.SetClearance(0)
                for params, expected in (
                    ({'clearance': 0.30}, 0.30),
                    ({'clearance': 0.05}, 0.10),
                    ({'clearance_ceiling': 0.30}, 0.10),
                ):
                    dlg.reset_params_to_defaults()
                    ai_plan.apply_step_params({'action': 'route', 'params': params}, dlg)
                    got = assert_configs(dlg, expected)
                    rows.append(dict(declared_mm=0, requested=params, effective_mm=got))
                # A next plan step omitting clearance must return to the board.
                dlg.reset_params_to_defaults()
                ai_plan.apply_step_params({'action': 'route', 'params': {}}, dlg)
                assert_configs(dlg, 0.10)

                # A live KiCad board always has a Default class; simulate only
                # an unavailable read for the explicitly supported fallback.
                # Other netclass fields stay real so all config paths execute.
                real = swig_gui._get_netclass_parameters
                def missing_clearance(name):
                    result = dict(real(name) or {})
                    result.pop('clearance', None)
                    return result
                dlg.clearance.SetValue(0.42)
                with patch.object(swig_gui, '_get_netclass_parameters', missing_clearance):
                    assert_configs(dlg, 0.25)
            finally:
                dlg.Destroy()
    print(json.dumps(rows, indent=2))
    print('PASS: #966 declared/default/override/ceiling/reset and shared-tab parameters')
    return 0


def assert_configs(dlg, expected):
    actual = {
        'route': dlg._build_routing_config([], ['F.Cu', 'B.Cu'])['clearance'],
        'diff': dlg.differential_tab.get_shared_params()['clearance'],
        'planes': dlg.planes_tab.get_shared_params()['clearance'],
        'fanout': dlg.fanout_tab.get_shared_params()['clearance'],
    }
    for path, got in actual.items():
        assert abs(got - expected) < 1e-9, (path, got, expected)
    return actual['route']


if __name__ == '__main__':
    sys.exit(main())
