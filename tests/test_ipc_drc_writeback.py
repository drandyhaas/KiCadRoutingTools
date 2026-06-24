#!/usr/bin/env python3
"""IPC DRC-settings write-back edits the sibling .kicad_pro (issue #160).

kipy has no setter for design settings / net classes / severities, so the IPC
plugin loosens the open project's DRC floors by editing the .kicad_pro on disk
(the CLI's file path) rather than the live in-memory project. This exercises the
adapter helper end-to-end with the IPC board-path lookup stubbed, so no running
KiCad is needed.

    python3 tests/test_ipc_drc_writeback.py
"""
import os
import sys
import json
import shutil
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import kicad_ipc_adapter as adapter

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(REPO, "kicad_files", "routed_output.kicad_pcb")


def test_writeback_loosens_project():
    tmp = tempfile.mkdtemp()
    try:
        pcb = os.path.join(tmp, "b.kicad_pcb")
        shutil.copyfile(BOARD, pcb)
        # Stub the IPC path lookup so the helper runs without KiCad.
        adapter.get_board_full_path = lambda *a, **k: pcb

        out = adapter.write_drc_settings_to_project(
            board=None, clearance=0.1, hole_to_hole=0.2, edge_clearance=0.2,
            track_width=0.1, via_diameter=0.4, via_drill=0.2,
            keep_thermal=False, diff_pair_gap=0.1, diff_pair_width=0.1)
        assert out and os.path.isfile(out), out
        proj = json.load(open(out))
        rules = proj["board"]["design_settings"]["rules"]
        assert rules["min_clearance"] == 0.1, rules
        assert rules["min_track_width"] == 0.1, rules
        dflt = next(c for c in proj["net_settings"]["classes"]
                    if c.get("name") == "Default")
        assert dflt["clearance"] == 0.1 and dflt["diff_pair_gap"] == 0.1, dflt
        print("PASS writeback_loosens_project")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_writeback_no_project_path():
    adapter.get_board_full_path = lambda *a, **k: None
    out = adapter.write_drc_settings_to_project(board=None, clearance=0.1)
    assert out is None  # unsaved board -> nothing to edit, no crash
    print("PASS writeback_no_project_path")


if __name__ == "__main__":
    test_writeback_loosens_project()
    test_writeback_no_project_path()
    print("\nAll IPC DRC write-back tests passed.")
