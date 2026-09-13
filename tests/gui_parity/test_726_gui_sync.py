#!/usr/bin/env python3
"""#726 GUI gate: the live-board sync must reach BOTH blocks, not one twice.

`gui_utils.sync_footprint_positions_from_board` refreshes a cached `PCBData`
from the live pcbnew board between plan steps (#362: a cap moved by
`optimize_caps` had to stop being routed around where it USED to be). It looked
each live footprint up by `GetReference()`. With two blocks named `TP4` that is
the footprint-level twin of the pad bug the function's own docstring already
describes: both live footprints resolve to the ONE `TP4` entry, the second
overwrites the first's pose, and the cached model has two parts on top of each
other with no error anywhere.

WHY THIS FILE EXISTS AT ALL -- it was written because a mutation battery said
so. `tests/mutate_726.py`'s `gui-sync-matches-by-bare-reference` row reverts
that lookup to `GetReference()`, and on its first run the row SURVIVED: nothing
in the suite covered the function on a board with duplicates.
`tests/gui_parity/test_footprint_position_sync.py` does cover it, and stays
GREEN through the mutation, because it runs on `rp2350_fpga_eensy_prePlane`
(61 blocks, 61 references). A passing gate on a board that cannot express the
defect proves nothing about it, and that is the whole reason this one names a
board that can.

THREE THINGS THIS PINS:

1. **A no-op sync is a true no-op**, on a duplicate-carrying board. Every pose
   and every pad position must come back bit-identical. Under the bare-reference
   lookup, `TP4` acquires `TP4~2`'s pose without anything moving on the board.

2. **A real move reaches the block it was made on.** Move ONE twin on the live
   board, sync, and the cached model must show that twin moved and the other
   one still where it was.

3. **The pads follow the footprint they belong to.** The function updates pad
   positions by ITERATION ORDER within a footprint; if the footprint itself is
   the wrong one, the pads land on the wrong part's coordinates, which is what
   the router then treats as copper.

Needs pcbnew; re-execs into KiCad's python. Lives in `tests/gui_parity/`
because `run_all.py`'s glob only collects `tests/test_*.py`.

    python3 -X utf8 tests/gui_parity/test_726_gui_sync.py
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration, and it says so rather than dying on an import.
#
# `gui_utils.sync_footprint_positions_from_board` and `live_footprints_by_key`
# are SWIG live-board helpers: they walk a `pcbnew.BOARD` and refresh a cached
# PCBData from it. The IPC port deleted both -- the plan executor re-reads the
# board through `kicad_ipc_adapter` between steps instead, so there is no
# cached model to refresh and no live pcbnew footprint to match.
#
# Left in place, this file imported those names and died with an ImportError.
# That is the failure mode CLAUDE.md names explicitly: a test that dies before
# it tests anything exits the same way a satisfied guard does, so a run of the
# gui_parity directory would report a #726 gate that ran and could not have.
#
# WHERE THE COVERAGE WENT. Nothing on this branch matches a live footprint to a
# cached one by reference, so there is no mutation of the defect to make. What
# does exist is the KEYING that made the defect possible, and that is graded by
# `tests/test_726_kipy_reference_keys.py` on `kipy_raw_references` --
# in-process, no KiCad. `tests/mutate_726.py` records the same thing at the row
# level: its `gui-sync-matches-by-bare-reference` row is deliberately absent,
# with the reason.
#
# Restore this file WITH the function if a live-board position sync ever comes
# back; the three properties its docstring above enumerates are what it must
# pin.
import sys

if __name__ == '__main__':
    print("SKIP: gui_utils.sync_footprint_positions_from_board does not exist "
          "on ipc-migration (the IPC port removed the live-board position "
          "sync). The keying half of #726 is graded by "
          "tests/test_726_kipy_reference_keys.py.")
    sys.exit(77)
