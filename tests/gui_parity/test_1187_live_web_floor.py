#!/usr/bin/env python3
"""The GUI's narrow-pad-joint floor is its LIVE board's declared rules (#1187).

fix_kicad_drc_settings.connection_width_floor reads the board's declared rules
(the author's min_connection, else min_track_width). The CLI reads them from
the sibling .kicad_pro, which every chain step rewrites. The GUI used to read
the same file -- but mid-plan that file is still the ORIGINAL project: a step's
writeback lowers the floors in pcbnew's memory (apply_targets_to_board, then
gui_utils.update_live_drc_floors). And a PCBData the GUI built before a step's
writeback is still in use after it. Measured on the engine-parity board
(splitflap_driver): with the repair floor priced at the declared rules alone,
the GUI read 0.2 where the CLI read 0.127 and its copper lost one /SENSOR_F
segment.

So build_pcb_data_from_board gives PCBData a live_rules_provider, read when
the floor is asked for. Real pcbnew, a real routed board:

  * before the step, the floor is the declared 0.3;
  * after one GUI step's writeback, the SAME PCBData answers the lowered live
    floor, while the file on disk still says 0.3;
  * an author's min_connection outranks min_track_width, live as in the file.

Needs KiCad python (pcbnew); re-execs into it like its siblings.

    python3 tests/gui_parity/test_1187_live_web_floor.py
"""
import contextlib
import glob
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Every versioned install, newest first by NUMERIC version (a string sort
# puts KiCad\9.0 above KiCad\10.0).
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]    # main() orders its own sys.path below
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]

SRC = os.path.join(REPO, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')
DECLARED = {'min_track_width': 0.3}
STEP = {'track_width': 0.15, 'via_size': 0.4, 'via_drill': 0.2}
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail else ''))
    if not ok:
        FAILS.append(name)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            # os.execv re-splits argv on spaces on Windows ("Program Files").
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew found; this gate does not self-skip, "
          "because a gate that exits 0 without running reports everything it "
          "guards as checked.")
    sys.exit(2)


def _stage(d, rules):
    dst = os.path.join(d, 'b.kicad_pcb')
    shutil.copyfile(SRC, dst)
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({"board": {"design_settings": {"rules": dict(rules)}},
                   "meta": {"version": 1}}, f, indent=2)
    return dst


def _file_rule(board_path, key):
    with open(os.path.splitext(board_path)[0] + '.kicad_pro', encoding='utf-8') as f:
        return json.load(f)['board']['design_settings']['rules'].get(key)


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import pcbnew
    sys.path.insert(0, os.path.dirname(REPO))
    for sub in ('', 'py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    # ipc-migration: there is no live rule set to read. kipy cannot write live
    # design settings, so this front's writeback (apply_drc_settings_fix ->
    # fix_project_for_output) lowers the floors in the sibling .kicad_pro --
    # the file connection_width_floor already reads off PCBData.source_path
    # when no live_rules_provider is set, which the kipy builder does not set.
    # The stale-file case this gate measures cannot arise here. Refused by
    # name rather than left to die on the import below; kept whole so it
    # re-arms by itself if a live writer is ever ported.
    try:
        from kicad_routing_plugin.gui_utils import update_live_drc_floors
    except ImportError:
        print("REFUSED (exit 77) on ipc-migration: gui_utils has no "
              "update_live_drc_floors -- kipy cannot write live design "
              "settings, so the floors this gate reads live are written to the "
              "sibling .kicad_pro, which connection_width_floor reads through "
              "PCBData.source_path. That file arm is covered by "
              "tests/test_check_weird.py. NOT covered: nothing -- the live arm "
              "has no IPC counterpart.")
        return 77
    from fix_kicad_drc_settings import (apply_targets_to_board,
                                        connection_width_floor)
    from kicad_parser import build_pcb_data_from_board

    td = tempfile.mkdtemp(prefix='t1187_live_')
    try:
        path = _stage(td, DECLARED)
        board = pcbnew.LoadBoard(path)
        pcbnew.GetBoard = lambda: board
        bds = board.GetDesignSettings()
        pcb = build_pcb_data_from_board(board)
        check("the live board carries a rules provider",
              callable(getattr(pcb, 'live_rules_provider', None)))
        f0 = connection_width_floor(pcb)
        check("before the step: the declared floor", abs(f0 - 0.3) < 1e-9, f"{f0}")

        # One GUI routing step's writeback, as swig_gui calls it.
        with contextlib.redirect_stdout(io.StringIO()):
            apply_targets_to_board(board, {'min_track_width': STEP['track_width']}, {})
            update_live_drc_floors(board, **STEP)
        live = bds.m_TrackMinWidth / 1e6
        check("the step lowered the live floor", live < 0.3 - 1e-9, f"{live}")
        check("the file beside the board still declares the original",
              _file_rule(path, 'min_track_width') == 0.3,
              f"{_file_rule(path, 'min_track_width')}")
        f1 = connection_width_floor(pcb)
        check("after it, the SAME PCBData answers the live floor, not the file",
              abs(f1 - live) < 1e-9, f"floor {f1}, live {live}")
        check("the repair's shipped floor reads the live floor too",
              connection_width_floor(pcb, shipped=True) <= live + 1e-9,
              f"{connection_width_floor(pcb, shipped=True)}")

        if hasattr(bds, 'm_MinConn'):
            bds.m_MinConn = 250000                     # 0.25 mm, in nm
            f2 = connection_width_floor(pcb)
            check("an author's live min_connection outranks min_track_width",
                  abs(f2 - 0.25) < 1e-9, f"{f2}")
        else:
            check("this pcbnew exposes m_MinConn", False,
                  "BOARD_DESIGN_SETTINGS has no m_MinConn")
    finally:
        shutil.rmtree(td, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}): " + '; '.join(FAILS))
        return 1
    print("PASS: the GUI's connection-width floor is the live board's declared "
          "rules, read when asked")
    return 0


if __name__ == '__main__':
    sys.exit(main())
