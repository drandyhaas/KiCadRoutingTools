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
    wx.Log.SetActiveTarget(wx.LogStderr())
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

        # 3b. RESTORE from a dict MISSING a key: a settings file saved before
        #     that control existed. Each key is dropped alone (an empty dict
        #     proves nothing -- restore returns before reading any key), so a
        #     restore branch that reads a key it never tested for raises here.
        raised = []
        for key in sorted(settings):
            older = dict(settings)
            older.pop(key)
            try:
                restore_dialog_settings(dlg, older)
            except Exception as e:
                raised.append(f"{key}: {type(e).__name__}: {e}")
        if raised:
            failures.append(f"restore without a key raised for {len(raised)} "
                            f"key(s): {raised}")
        else:
            print(f"  restore_dialog_settings (each of {len(settings)} keys "
                  f"absent in turn): OK")
        restore_dialog_settings(dlg, settings)

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

    # 5. A VALUE round-trip, which key parity cannot see: the Escalation
    #    choice (#857) is restored through FindString, and that branch read
    #    wx.NOT_FOUND in a module that never imports wx -- a NameError its own
    #    try/except swallowed, so the saved choice never came back.
    if settings is not None:
        ch = dlg.escalation
        cur = ch.GetSelection()
        alt = next((i for i in range(ch.GetCount()) if i != cur), None)
        if alt is None:
            failures.append("escalation offers one choice: the value "
                            "round-trip cannot be graded")
        else:
            ch.SetSelection(alt)
            want = ch.GetString(alt)
            saved = get_dialog_settings(dlg)
            dlg2 = RoutingDialog(None, parse_kicad_pcb(board), board)
            try:
                before = dlg2.escalation.GetString(dlg2.escalation.GetSelection())
                restore_dialog_settings(dlg2, saved)
                got = dlg2.escalation.GetString(dlg2.escalation.GetSelection())
            finally:
                dlg2.Destroy()
            if before == want:
                failures.append(f"fixture: a fresh dialog already shows "
                                f"escalation {want!r}, so restoring it proves "
                                f"nothing")
            elif got != want:
                failures.append(f"escalation not restored: saved {want!r}, "
                                f"a fresh dialog still shows {got!r}")
            else:
                print(f"  escalation value round-trip: OK ({before!r} -> "
                      f"{got!r})")

    # 6. The centerline setback's override box (route_diff.py's explicit
    #    --diff-pair-centerline-setback vs the omitted flag). A settings file
    #    saved before the box existed has only the value, where 0 meant auto
    #    and a positive value an explicit setback; a new file can say an
    #    explicit 0. Each must come back as the setback the engine gets.
    if settings is not None:
        dt = dlg.differential_tab
        rows = (('old file, 0 (meant auto)', {'centerline_setback': 0.0}, None),
                ('old file, 0.4', {'centerline_setback': 0.4}, 0.4),
                ('new file, ticked 0 (explicit zero)',
                 {'centerline_setback': 0.0, 'centerline_setback_check': True},
                 0.0),
                ('new file, unticked 0.4 (auto)',
                 {'centerline_setback': 0.4, 'centerline_setback_check': False},
                 None))
        bad_rows = []
        for label, keys, want in rows:
            s = dict(settings)
            s.pop('centerline_setback_check', None)
            s.update(keys)
            restore_dialog_settings(dlg, s)
            got = dt.get_config().get('diff_pair_centerline_setback')
            if got != want or dt.centerline_setback.IsEnabled() != (want is not None):
                bad_rows.append(f"{label}: engine setback {got!r} (want {want!r}), "
                                f"spin enabled {dt.centerline_setback.IsEnabled()}")
        if bad_rows:
            failures.append("centerline setback restore: " + "; ".join(bad_rows))
        else:
            print(f"  centerline setback override restore: OK ({len(rows)} "
                  f"forms, old 0 -> auto, new ticked 0 -> explicit 0)")
        restore_dialog_settings(dlg, settings)

    dlg.Destroy()

    print()
    if failures:
        for f in failures:
            print(f"FAIL: {f}")
        sys.exit(1)
    print("VERDICT: settings round-trip clean (save, restore, legacy, re-save)")


if __name__ == '__main__':
    main()
