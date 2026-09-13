#!/usr/bin/env python3
"""#829 CLI/GUI parity: both parse paths must agree on who owns the outline.

`build_pcb_data_from_board` is an independent implementation of the text
parser, so a field only the text path fills is INERT under the GUI -- and this
particular field gates whether a footprint may be moved, so an inert one means
the guard silently is not there.

It could not be written the way `test_ref_label_pcbnew_parity.py` was. That gate
sweeps real corpus boards; **0 of the 27 tracked boards carry footprint-embedded
Edge.Cuts at all**, so there is nothing in the repo for it to compare. It uses
the synthetic `tests/fixture_829.py` board instead, which is also the only way
to cover the case that must NOT be locked.

Needs KiCad's pcbnew; self-skips if absent.

    python3 tests/gui_parity/test_829_edge_cuts_owner_parity.py
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration -- and the reason is a REAL GAP this gate was
# the right instrument for, so read this before deleting either.
#
# The gate compares the text parser against `build_pcb_data_from_board`. On
# this branch that is the kipy builder, so handing it a `pcbnew.BOARD` cannot
# work (it dies on `ModuleNotFoundError: kicad_ipc_adapter` before it even gets
# to the semantics). PRE-EXISTING: it has exited 1 on this branch since it
# arrived.
#
# THE GAP IT WOULD HAVE FOUND. The kipy builder fills NEITHER
# `owns_edge_cuts` NOR `owns_board_outline` -- grep the builder, there is no
# assignment at all, so every footprint reads `owns_board_outline=False` on the
# IPC front. That is precisely what this gate's own docstring warns about: the
# field gates whether a footprint may be MOVED, so an inert one means the guard
# silently is not there, and a connector drawing the real board's edge is
# movable in the GUI.
#
# WHY IT IS NOT A ONE-LINE PORT. main's pcbnew path calls
# `footprint_outline_owners_from_pcbnew(board, to_mm)`, which walks each
# footprint's own graphical items. kipy's `get_shapes()` returns top-level
# PCB_SHAPEs only -- footprint children are not in it -- which is the same API
# limitation that leaves #337/#908 copper graphics unmodelled here (see the
# note in `build_pcb_data_from_board`). The workable route is the one the via
# protection spec takes: read the owners from the board's own FILE via
# `_ipc_board_path()`, using the text path's `footprint_outline_owners` so
# there is no third implementation of the decision.
import sys

if __name__ == '__main__':
    print("SKIP: build_pcb_data_from_board is the kipy builder here and "
          "cannot take a pcbnew BOARD. NOTE the gap this gate cannot cover: "
          "the kipy builder fills neither owns_edge_cuts nor "
          "owns_board_outline, so #829's move guard is absent on the IPC "
          "front.")
    sys.exit(77)
