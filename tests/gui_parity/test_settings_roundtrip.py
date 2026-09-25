#!/usr/bin/env python3
"""Settings save/restore round-trip against the REAL headless dialog.

Why this gate exists: closing the routing dialog calls get_dialog_settings()
over every tab's controls, and reopening calls restore_dialog_settings().
Both walk control attributes by NAME, so deleting or renaming a control
without updating persistence raises AttributeError at close time -- with the
user's whole session's settings lost and nothing but a traceback to show for
it. That is exactly what #562's planes-tab retirement did: the repair panel
and the create panel's rip_blocker_check were deleted while
'planes_rip_blocker_check' kept reading create_options.rip_blocker_check.

No routing runs here -- it constructs the dialog headless, round-trips the
settings dict, and asserts nothing raises. Seconds, not minutes.

Run: python3 tests/gui_parity/test_settings_roundtrip.py
(re-execs into KiCad's bundled python automatically, like its siblings)
"""
import glob
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # this file orders its own sys.path further down
KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew+wx found")
    sys.exit(0)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522
    sys.path.insert(0, os.path.dirname(REPO))

    app = wx.App(False)  # noqa: F841  (before any wx object)
    board = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin.settings_persistence import (
        get_dialog_settings, restore_dialog_settings)

    dlg = RoutingDialog(None, parse_kicad_pcb(board), board)
    failures = []

    # 1. SAVE -- the close path.
    try:
        settings = get_dialog_settings(dlg)
        print(f"  get_dialog_settings: OK ({len(settings)} keys)")
    except Exception as e:
        failures.append(f"get_dialog_settings raised {type(e).__name__}: {e}")
        settings = None

    # 2. RESTORE -- the reopen path, with the dict save just produced.
    if settings is not None:
        try:
            restore_dialog_settings(dlg, settings)
            print("  restore_dialog_settings (fresh dict): OK")
        except Exception as e:
            failures.append(
                f"restore_dialog_settings raised {type(e).__name__}: {e}")

        # 3. RESTORE from a LEGACY dict carrying keys this version dropped:
        #    a user upgrading must not crash on their old settings file.
        legacy = dict(settings)
        legacy.update({
            'planes_mode': 1,
            'planes_rip_blocker_check': True,
            'planes_repair_max_track_width': 2.0,
            'planes_repair_min_track_width': 0.2,
            'planes_repair_analysis_grid': 0.5,
            'planes_repair_pads': True,
            'planes_repair_rip_blocker_check': True,
        })
        try:
            restore_dialog_settings(dlg, legacy)
            print("  restore_dialog_settings (legacy keys): OK")
        except Exception as e:
            failures.append(f"restore with legacy keys raised "
                            f"{type(e).__name__}: {e}")

    # 4. Every key the SAVE emits must round-trip without the control
    #    vanishing: re-save and compare key sets.
    if settings is not None:
        try:
            again = get_dialog_settings(dlg)
            missing = set(settings) - set(again)
            if missing:
                failures.append(f"keys lost on re-save: {sorted(missing)}")
            else:
                print(f"  re-save key parity: OK ({len(again)} keys)")
        except Exception as e:
            failures.append(f"second get_dialog_settings raised "
                            f"{type(e).__name__}: {e}")

    dlg.Destroy()

    print()
    if failures:
        for f in failures:
            print(f"FAIL: {f}")
        sys.exit(1)
    print("VERDICT: settings round-trip clean (save, restore, legacy, re-save)")


if __name__ == '__main__':
    main()
