#!/usr/bin/env python3
"""#726 parity gate: both parse paths must name a duplicated block the same.

`parse_kicad_pcb` walks the file; `build_pcb_data_from_board` walks a live
pcbnew BOARD. When two footprint blocks claim one reference, the name that
separates them is a FILE-ORDER ORDINAL, which is a property of the ordered
list rather than of any one footprint -- so the two paths agree only if they
enumerate footprints in the same order. This gate is where that is checked.

WHY A KEY-SET COMPARISON IS NOT ENOUGH, and why the old harness was blind.
`compare_pcb_data` is the existing two-path diff, and before #726 it could not
see this defect at all: both paths dropped the SAME block, so their key sets
matched and it reported perfect parity on a board that had lost six
footprints. The residual hazard now is the mirror image -- two paths that
disambiguate in OPPOSITE order still agree on the key SET (`{TP4, TP4~2}`
either way) while `TP4` means a different physical part on each side. So the
load-bearing arm here is per-key identity: same position, same rotation, same
layer, same uuid.

THE SELF-EXPIRING ARM. `board.GetFootprints()` returning KiCad's footprints in
file order is an empirical fact about one release, not a documented contract.
Measured on KiCad 10.0.0 it holds on all 22 tracked corpus boards, for both the
uuid sequence and the reference sequence. The arm named "GetFootprints() still
iterates in FILE ORDER" pins it, and if a future KiCad breaks it the ordinal
scheme is unsound and must move to a different key -- DO NOT relax that arm to
make it pass.

Needs pcbnew; re-execs into KiCad's python automatically. Lives in
`tests/gui_parity/` because `run_all.py`'s glob only collects `tests/test_*.py`
and must never collect a gate that re-execs.

    python3 -X utf8 tests/gui_parity/test_726_parse_path_parity.py
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration. Left as it arrived it RAN and reported FAILED
# on every board, which is worse than not running: the failures were the gate
# feeding a `pcbnew.BOARD` to a builder that no longer takes one.
#
# `build_pcb_data_from_board` is the kipy builder here, not the pcbnew walk the
# docstring above describes. It reaches a RUNNING KiCad over a socket, so the
# two-path comparison cannot be made in process at all -- and it cannot be made
# through `fake_ipc_board` either, which deliberately serves the read path by
# re-parsing the file with `parse_kicad_pcb`: that would grade the text parser
# against itself and pass whatever the kipy builder did.
#
# WHAT STILL HOLDS, AND WHERE. The load-bearing claim -- that the IPC path names
# a duplicated block the way the file does, so the same key means the same
# physical part -- is graded by `tests/test_726_kipy_reference_keys.py`, in
# process, against `iter_footprint_blocks` as the oracle. What it does NOT
# cover is this file's self-expiring arm: that KiCad enumerates footprints in
# FILE ORDER. The ordinal scheme is unsound without it, and on this branch the
# enumeration is `board.get_footprints()` over IPC rather than
# `board.GetFootprints()`, so it is a DIFFERENT empirical fact and one nothing
# here has measured. Measure it against a live KiCad before trusting an ordinal
# on this front; the note in `build_pcb_data_from_board` says the same.
import sys

if __name__ == '__main__':
    print("SKIP: build_pcb_data_from_board is the kipy builder on "
          "ipc-migration and cannot be handed a pcbnew BOARD. The keying claim "
          "is graded by tests/test_726_kipy_reference_keys.py; the file-order "
          "arm needs a live KiCad and is NOT covered.")
    sys.exit(77)
