#!/usr/bin/env python3
"""#966: real dialog clearance resolution, including every shared tab config.

    python3 tests/gui_parity/test_966_unset_clearance.py

(re-execs into KiCad's bundled python automatically, like its siblings)

PORTED TO IPC. Main's copy drives `swig_gui.RoutingDialog` over a live
`pcbnew.LoadBoard`, and declares the Default class by writing the native
`GetDesignSettings().m_NetSettings` netclass. Neither exists on this branch:
the dialog is `routing_dialog.RoutingDialog`, and **net classes are not carried
over IPC at all** -- `build_pcb_data_from_board` reads them out of the sibling
.kicad_pro into `pcb_data.netclass_params`, which `_get_netclass_parameters`
then reads and nothing else does. So the declared class is set HERE by writing
that map, which is this front's whole mechanism rather than a stand-in for it.

The rule under test is unchanged, and it is the one this branch merged from
main: a declared Default clearance of ZERO is UNSET, not a floor of zero. The
routing CLIs keep the declared value as the base and let `enforce_fab_floors`
pin it to the fab capability; an unchecked Min Clearance spin must not supply a
default in its place, and an ABSENT class value falls back to the same constant
the CLIs use.

This gate measures parameter delivery, not copper quality.
"""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[2]

KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    r'C:\Program Files\KiCad\10.0\bin\python.exe',
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import wx, pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with wx found")
    sys.exit(0)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, str(REPO))
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, str(REPO / sub))
    sys.path.insert(0, os.path.dirname(str(REPO)))
    sys.path.insert(0, str(REPO / 'tests' / 'gui_parity'))

    from copy_board import copy_board
    from fake_ipc_board import build_pcb_data_like_ipc
    from kicad_routing_plugin import ai_plan
    from kicad_routing_plugin import routing_dialog

    app = wx.App(False)
    assert app is not None
    rows = []
    with tempfile.TemporaryDirectory(prefix='krt966-') as tmp:
        board_path = Path(tmp) / 'board.kicad_pcb'
        src = REPO / 'kicad_files/splitflap_driver.kicad_pcb'
        # Stage the fixture WITH a sibling .kicad_pro. Main's copy needs none:
        # a live pcbnew board always carries a Default net class whether or not
        # a project exists, and that is what it writes. Over IPC the classes
        # come from the project FILE and nowhere else, so a project-less board
        # gives the dialog an empty netclass map -- every row below would then
        # grade the fallback constant instead of the declared class, and the
        # gate would pass while measuring nothing. pcbnew authors the project,
        # exactly as test_gui_livechain_rp2350 stages its own fixture.
        if (src.with_suffix('.kicad_pro')).is_file():
            copy_board(str(src), str(board_path))
        else:
            import pcbnew
            pcbnew.SaveBoard(str(board_path), pcbnew.LoadBoard(str(src)))
            print("staged the input WITH a KiCad-authored .kicad_pro "
                  "(the fixture has none)")
        pcb_data = build_pcb_data_like_ipc(str(board_path))
        dlg = routing_dialog.RoutingDialog(None, pcb_data, str(board_path))

        # The IPC twin of main's `m_NetSettings.GetDefaultNetclass()`: the map
        # the builder filled from the .kicad_pro IS the declared class here.
        nc = (pcb_data.netclass_params or {}).get('Default')
        if nc is None:
            dlg.Destroy()
            print("ERROR: pcb_data carries no Default net class; every row "
                  "below would grade the fallback instead of the declaration")
            return 1
        try:
            assert len(pcb_data.board_info.copper_layers) == 2, \
                pcb_data.board_info.copper_layers
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
                nc['clearance'] = declared
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
            nc['clearance'] = 0.0
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
            # NOTE the signature: on this branch the resolver takes
            # (class_name, pcb_data) -- the classes live in PCBData, not on a
            # live board -- so a one-argument stand-in would raise instead of
            # exercising the fallback.
            real = routing_dialog._get_netclass_parameters

            def missing_clearance(name, pcb_data=None):
                result = dict(real(name, pcb_data) or {})
                result.pop('clearance', None)
                return result

            dlg.clearance.SetValue(0.42)
            with patch.object(routing_dialog, '_get_netclass_parameters',
                              missing_clearance):
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
