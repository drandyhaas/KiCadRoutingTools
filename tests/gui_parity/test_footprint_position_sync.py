#!/usr/bin/env python3
"""Regression gate for the stale-cap-position bug (#362).

The GUI parses pcb_data ONCE when the dialog opens and reuses it across plan
steps. optimize_caps (fanout tab) relocates decoupling caps on the live board
via SetPosition, but the cached pcb_data kept their load-time positions -- so a
LATER signal/diff route step routed around where a cap USED to be, and the moved
cap's pad shorted the fresh +1V1 copper. Fix: _sync_pcb_data_from_board (called
before every route step) now refreshes footprint/pad positions from the board
via gui_utils.sync_footprint_positions_from_board.

This test proves the helper: build pcb_data from a board, move a footprint on
the board, sync, and assert pcb_data (and the shared pads_by_net view the router
consults) now reports the NEW position. Needs KiCad's pcbnew; skips if absent.
Run: python3 tests/gui_parity/test_footprint_position_sync.py
"""
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import pcbnew'],
                              capture_output=True).returncode == 0:
                os.execv(cand, [cand, os.path.abspath(__file__)] + sys.argv[1:])
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import pcbnew
    sys.path.insert(0, REPO)
    sys.path.insert(0, os.path.join(REPO, 'kicad_routing_plugin'))
    from kicad_parser import build_pcb_data_from_board
    from gui_utils import sync_footprint_positions_from_board

    board_path = os.path.join(REPO, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
    if not os.path.exists(board_path):
        print(f"SKIP: {board_path} not found")
        sys.exit(0)

    board = pcbnew.LoadBoard(board_path)
    pcb = build_pcb_data_from_board(board)

    # Pick any footprint that has a pad, to relocate.
    ref = None
    for r, fp in pcb.footprints.items():
        if fp.pads:
            ref = r
            break
    assert ref is not None, "no footprint with pads on the board"
    fp = pcb.footprints[ref]
    pad0 = fp.pads[0]
    before = (pad0.global_x, pad0.global_y)

    # Move the footprint +0.2mm x on the board (as optimize_caps would).
    bfp = board.FindFootprintByReference(ref)
    op = bfp.GetPosition()
    dx_mm = 0.2
    bfp.SetPosition(pcbnew.VECTOR2I(op.x + pcbnew.FromMM(dx_mm), op.y))

    # Before sync, pcb_data is STALE (this is the bug).
    stale = (pad0.global_x, pad0.global_y)
    assert abs(stale[0] - before[0]) < 1e-9, \
        "pcb_data changed without a sync -- test assumption broken"

    # Sync from board -- the fix.
    n = sync_footprint_positions_from_board(board, pcb)
    assert n > 0, "sync reported 0 footprints"

    after = (pad0.global_x, pad0.global_y)
    moved = after[0] - before[0]
    assert abs(moved - dx_mm) < 1e-3, \
        f"{ref} pad x expected +{dx_mm}mm after sync, got +{moved:.4f}mm"

    # The router consults pads_by_net; the SAME pad object must be there.
    same = None
    for nid, pads in pcb.pads_by_net.items():
        for p in pads:
            if p is pad0:
                same = p
                break
        if same:
            break
    assert same is not None, f"{ref} pad0 not found in pads_by_net"
    assert abs(same.global_x - after[0]) < 1e-9, \
        "pads_by_net view did not reflect the sync (separate object!)"

    print(f"PASS: sync moved {ref} pad by +{moved:.4f}mm and pads_by_net "
          f"reflects it (shared object); {n} footprints synced.")


if __name__ == '__main__':
    main()
