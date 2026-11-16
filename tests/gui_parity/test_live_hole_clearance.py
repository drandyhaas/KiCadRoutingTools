#!/usr/bin/env python3
"""The GUI prices NPTH holes at its LIVE board's min hole clearance (#1038,
#1238), and a KiCad session re-reads a project it saved.

`obstacle_map.resolve_hole_clearance` is the copper-to-hole floor the router
holds tracks off NPTH walls at, and since #1038 any value above 0 also turns on
the via-copper keep-out around them. The CLI reads it from the board's sibling
.kicad_pro, in a fresh process every step. The GUI plugin is one process for a
whole KiCad session, and it used to read the same file through a cache keyed
on the board path: a Board Setup edit was invisible until saved, a save was
invisible for the rest of the session, and an unsaved board (#1238) had no file
to read at all.

Real pcbnew, real boards:

  * flat_hierarchy (its project declares 0.25): both fronts read 0.25 off a
    fresh load; an UNSAVED Board Setup edit to 0.30 is priced by the GUI -- by
    the PCBData built before the edit -- while the file, and so the CLI, still
    says 0.25, and compare_pcb_data reports the difference; after pcbnew saves
    the board, a file parse in the SAME process reads 0.30;
  * splitflap_driver with no project: pcbnew loads it at its factory 0.25, the
    file declares nothing, and both fronts read 0.0 (no divergence on a fresh
    load); after one GUI step's writeback (apply_targets_to_board at 0.2) the
    GUI reads 0.2 -- the floor the CLI's next step reads from the project its
    own writeback (fix_project_for_output at 0.2) leaves; the via descent
    ladder's hole clearance (fab_tiers.board_hole_clearance) is not that
    floor: both fronts still read the factory 0.25 there, since the rule is
    the writeback's, while an unsaved Board Setup edit to 0.15 is honoured,
    and stays so after the next GUI writeback lowers the rule to 0.1;
  * a board that was never saved prices a live edit.

Needs KiCad python (pcbnew); re-execs into it like its siblings, and does NOT
self-skip without it (exit 2).

    python3 tests/gui_parity/test_live_hole_clearance.py
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration. It prices NPTH holes at a live pcbnew board's in-memory min hole
# clearance (`live_rules_provider`, `stage_live_project_rules`). On this branch
# `build_pcb_data_from_board` is the KIPY builder: it reaches a RUNNING KiCad
# over a socket and cannot be handed a pcbnew BOARD, so this gate would grade
# a builder that does not exist here (and `fake_ipc_board` serves reads by
# re-parsing the file, which would grade the text parser against itself).
#
# WHERE THE CLAIM IS GRADED HERE: the FILE half -- the project re-read whenever it
# changes, keyed on its stamp -- by tests/test_1038_hole_floor_fresh.py, which
# the IPC dialog's reads go through.
#
# NOT COVERED: #1038/#1238's LIVE half. kipy cannot read Board Setup design
# settings, so the kipy builder sets no live_rules_provider: an unsaved Board
# Setup edit of the hole clearance is invisible to the IPC front until the
# board is saved, and an unsaved board prices no hole floor at all.
#
# main's body is kept below, unreached, so a later sync merges into it.
if __name__ == '__main__':
    import sys as _sys
    print('SKIP: live hole-clearance pricing reads pcbnew design settings, which kipy cannot; the file half is graded by tests/test_1038_hole_floor_fresh.py; an unsaved Board Setup edit is NOT seen on ipc-migration.')
    _sys.exit(77)
# ---------------------------------------------------------------------------
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

#: rule -> BOARD_DESIGN_SETTINGS attribute, this gate's own table (the
#: independent check on fix_kicad_drc_settings.LIVE_RULE_ATTR's constants).
RULE_ATTR = {
    'min_clearance': 'm_MinClearance', 'min_track_width': 'm_TrackMinWidth',
    'min_connection': 'm_MinConn', 'min_via_diameter': 'm_ViasMinSize',
    'min_through_hole_diameter': 'm_MinThroughDrill',
    'min_via_annular_width': 'm_ViasMinAnnularWidth',
    'min_hole_clearance': 'm_HoleClearance', 'min_hole_to_hole': 'm_HoleToHoleMin',
    'min_copper_edge_clearance': 'm_CopperEdgeClearance',
}
DECLARING = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
PROJECTLESS = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
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


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import pcbnew
    sys.path.insert(0, os.path.dirname(REPO))
    for sub in ('', 'py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    from copy_board import copy_board
    import fix_kicad_drc_settings as F
    from fix_kicad_drc_settings import (apply_targets_to_board, compute_targets,
                                        effective_board_edge_clearance,
                                        fix_project_for_output)
    from kicad_parser import (build_pcb_data_from_board, compare_pcb_data,
                              parse_kicad_pcb, stage_live_project_rules)
    from obstacle_map import resolve_hole_clearance

    def floor(pcb):
        return _quiet(resolve_hole_clearance, pcb, None)

    def hole_diffs(a, b):
        return [d for d in compare_pcb_data(a, b) if 'Copper-to-hole' in d]

    # What pcbnew really initialises each rule to -- the independent check on
    # the constants the declared view reads. The board is held while it is
    # read: a temporary's design settings would be freed under the attribute
    # read. (A standalone script; the GUI process never builds one.)
    fresh = pcbnew.BOARD()
    fresh_bds = fresh.GetDesignSettings()
    factory = fresh_bds.m_HoleClearance / 1e6
    td = tempfile.mkdtemp(prefix='live_hc_')
    try:
        print("--- pcbnew's factory values")
        consts = getattr(F, 'pcbnew_rule_defaults', dict)()
        wrong = {k: (consts.get(k), getattr(fresh_bds, a) / 1e6)
                 for k, a in RULE_ATTR.items()
                 if consts.get(k) is None
                 or abs(consts[k] - getattr(fresh_bds, a) / 1e6) > 1e-9}
        check("the factory values the declared view reads are what a fresh "
              "board holds, every rule", not wrong, f"{wrong}")

        print("--- a board whose project declares 0.25 (flat_hierarchy)")
        path = os.path.join(td, 'decl', 'b.kicad_pcb')
        os.makedirs(os.path.dirname(path))
        _quiet(copy_board, DECLARING, path)
        board = pcbnew.LoadBoard(path)
        pcbnew.GetBoard = lambda: board
        bds = board.GetDesignSettings()
        live = build_pcb_data_from_board(board)
        f0, l0 = floor(parse_kicad_pcb(path)), floor(live)
        check("fresh load: both fronts read the declared 0.25",
              abs(f0 - 0.25) < 1e-9 and abs(l0 - 0.25) < 1e-9, f"file {f0}, live {l0}")
        check("and compare_pcb_data agrees",
              not hole_diffs(live, parse_kicad_pcb(path)),
              f"{hole_diffs(live, parse_kicad_pcb(path))}")

        bds.m_HoleClearance = 300000                  # Board Setup, NOT saved
        l1, f1 = floor(live), floor(parse_kicad_pcb(path))
        check("an unsaved Board Setup edit to 0.30 is priced by the GUI -- "
              "by the PCBData built BEFORE the edit", abs(l1 - 0.30) < 1e-9, f"{l1}")
        check("while the file, which the CLI reads, still says 0.25",
              abs(f1 - 0.25) < 1e-9, f"{f1}")
        d = hole_diffs(live, parse_kicad_pcb(path))
        check("and compare_pcb_data names the difference", bool(d), f"{d}")

        check("pcbnew saves the board and its project",
              bool(pcbnew.SaveBoard(path, board)))
        with open(os.path.splitext(path)[0] + '.kicad_pro', encoding='utf-8') as f:
            saved = json.load(f)['board']['design_settings']['rules'].get(
                'min_hole_clearance')
        check("the saved project declares 0.30", saved == 0.3, f"{saved}")
        f2 = floor(parse_kicad_pcb(path))
        check("after the save, a file parse in the SAME process reads 0.30 "
              "(a path-keyed cache kept 0.25 for the whole session)",
              abs(f2 - 0.30) < 1e-9, f"{f2}")

        print("--- a project-less board (splitflap_driver)")
        bare = os.path.join(td, 'bare', 'b.kicad_pcb')
        os.makedirs(os.path.dirname(bare))
        shutil.copyfile(PROJECTLESS, bare)               # deliberately no .kicad_pro
        board2 = pcbnew.LoadBoard(bare)
        pcbnew.GetBoard = lambda: board2
        bds2 = board2.GetDesignSettings()
        check("pcbnew loads it at the factory value",
              abs(bds2.m_HoleClearance / 1e6 - factory) < 1e-9,
              f"{bds2.m_HoleClearance / 1e6}")
        live2 = build_pcb_data_from_board(board2)
        f3, l3 = floor(parse_kicad_pcb(bare)), floor(live2)
        check("both fronts read it as declaring nothing (0.0): no divergence "
              "on a fresh load", f3 == 0.0 and l3 == 0.0, f"file {f3}, live {l3}")
        check("and compare_pcb_data agrees",
              not hole_diffs(live2, parse_kicad_pcb(bare)),
              f"{hole_diffs(live2, parse_kicad_pcb(bare))}")

        _quiet(apply_targets_to_board, board2, compute_targets(clearance=0.2), {})
        l4 = floor(live2)
        check("one GUI step's writeback lowers the live rule to 0.2, and the "
              "GUI's next step reads 0.2", abs(l4 - 0.2) < 1e-9,
              f"live rule {bds2.m_HoleClearance / 1e6}, floor {l4}")
        cli = os.path.join(td, 'cli', 'b.kicad_pcb')
        os.makedirs(os.path.dirname(cli))
        shutil.copyfile(PROJECTLESS, cli)
        _quiet(fix_project_for_output, cli, input_pcb=PROJECTLESS, clearance=0.2)
        f4 = floor(parse_kicad_pcb(cli))
        check("the floor the CLI's next step reads from its own writeback's "
              "project", abs(f4 - l4) < 1e-9, f"CLI {f4}, GUI {l4}")
        # The via ladder's hole clearance is not that floor: the rule both
        # writebacks wrote is theirs, and the board declared nothing, so the
        # next step on either front prefers the ring for KiCad's factory 0.25
        # (the writeback's record, fab_tiers.HOLE_RING_RECORD_KEY).
        from fab_tiers import board_hole_clearance
        r_gui, r_cli = board_hole_clearance(live2), board_hole_clearance(
            parse_kicad_pcb(cli))
        check("after one step's writeback both fronts' via ladders still read "
              "the factory 0.25", abs(r_gui - factory) < 1e-9
              and abs(r_cli - factory) < 1e-9, f"GUI {r_gui}, CLI {r_cli}")
        bds2.m_HoleClearance = 150000                 # Board Setup, NOT saved
        r_edit = board_hole_clearance(live2)
        check("a Board Setup edit between steps (0.15) is honoured by the "
              "GUI's ladder", abs(r_edit - 0.15) < 1e-9, f"{r_edit}")
        _quiet(apply_targets_to_board, board2, compute_targets(clearance=0.1), {})
        r_next = board_hole_clearance(live2)
        check("and stays honoured after the next GUI writeback lowers the rule "
              "to 0.1", abs(bds2.m_HoleClearance / 1e6 - 0.1) < 1e-9
              and abs(r_next - 0.15) < 1e-9,
              f"rule {bds2.m_HoleClearance / 1e6}, ladder {r_next}")

        print("--- a writeback that targets exactly the factory 0.25")
        at = os.path.join(td, 'at25', 'b.kicad_pcb')
        os.makedirs(os.path.dirname(at))
        shutil.copyfile(PROJECTLESS, at)
        board4 = pcbnew.LoadBoard(at)
        pcbnew.GetBoard = lambda: board4
        live4 = build_pcb_data_from_board(board4)
        _quiet(apply_targets_to_board, board4, compute_targets(clearance=0.25), {})
        l7 = floor(live4)
        cli25 = os.path.join(td, 'cli25', 'b.kicad_pcb')
        os.makedirs(os.path.dirname(cli25))
        shutil.copyfile(PROJECTLESS, cli25)
        _quiet(fix_project_for_output, cli25, input_pcb=PROJECTLESS,
               clearance=0.25)
        f7 = floor(parse_kicad_pcb(cli25))
        check("pcbnew's value does not move, yet the GUI's next step reads "
              "0.25 -- the floor the CLI's writeback writes into its project",
              abs(board4.GetDesignSettings().m_HoleClearance / 1e6 - 0.25) < 1e-9
              and abs(l7 - 0.25) < 1e-9 and abs(f7 - 0.25) < 1e-9,
              f"GUI {l7}, CLI {f7}")

        print("--- the snapshot the GUI's oracle legs parse (stage_live_project_rules)")
        fresh_bare = os.path.join(td, 'snapsrc', 'b.kicad_pcb')
        os.makedirs(os.path.dirname(fresh_bare))
        shutil.copyfile(PROJECTLESS, fresh_bare)
        board5 = pcbnew.LoadBoard(fresh_bare)
        snap = os.path.join(td, 'snap', 'staged.kicad_pcb')
        os.makedirs(os.path.dirname(snap))
        shutil.copyfile(PROJECTLESS, snap)
        stage_live_project_rules(snap, board5)
        with open(os.path.splitext(snap)[0] + '.kicad_pro', encoding='utf-8') as f:
            staged_rules = json.load(f)['board']['design_settings']['rules']
        check("a project-less board's snapshot declares none of pcbnew's "
              "factory stand-ins", not staged_rules, f"{staged_rules}")
        f8, c8 = floor(parse_kicad_pcb(snap)), floor(parse_kicad_pcb(fresh_bare))
        check("so the oracle leg's hole floor is the CLI's on the same file",
              f8 == c8 == 0.0, f"snapshot {f8}, CLI {c8}")
        e8 = effective_board_edge_clearance(snap, 0.0)
        ce8 = effective_board_edge_clearance(fresh_bare, 0.0)
        check("and so is its copper-to-edge band (pcbnew's factory 0.5 used to "
              "reach it)", abs(e8 - ce8) < 1e-9, f"snapshot {e8}, CLI {ce8}")
        _quiet(apply_targets_to_board, board5, compute_targets(clearance=0.2), {})
        stage_live_project_rules(snap, board5)
        f9 = floor(parse_kicad_pcb(snap))
        check("after a GUI writeback the snapshot carries what it lowered",
              abs(f9 - 0.2) < 1e-9, f"{f9}")

        print("--- a board that was never saved (#1238)")
        board3 = pcbnew.BOARD()
        live3 = build_pcb_data_from_board(board3)
        check("no file", not getattr(live3, 'source_path', ''),
              f"{getattr(live3, 'source_path', '')!r}")
        l5 = floor(live3)
        check("at the factory value it declares nothing", l5 == 0.0, f"{l5}")
        board3.GetDesignSettings().m_HoleClearance = 300000
        l6 = floor(live3)
        check("a live edit to 0.30 is priced with no file to read",
              abs(l6 - 0.30) < 1e-9, f"{l6}")
    finally:
        shutil.rmtree(td, ignore_errors=True)

    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}): " + '; '.join(FAILS))
        return 1
    print("PASS: the GUI prices its live board's hole clearance, and a session "
          "re-reads a project it saved")
    return 0


if __name__ == '__main__':
    sys.exit(main())
