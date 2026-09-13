#!/usr/bin/env python3
"""#900 on a REAL pcbnew board: does `apply_targets_to_board` write the net
classes at the ROUTED clearance, or at the capped rule floor?

    python3 tests/gui_parity/test_900_live_class_clearance.py

(re-execs into KiCad's bundled python automatically, like its siblings)

THE DEFECT, and why it is a GUI defect and not only a CLI one. `compute_targets`
caps `rules.min_clearance` at the smallest copper-pad clearance override on the
board (#530), because KiCad floors an override there. `apply_targets_to_board`
then read that same capped key for the Default net class, and
`clamp_nondefault_netclasses_on_board` for every other class. The signal, planes
and differential tabs all call `apply_targets_to_board` with
`minima=board_minima_from_live(board)` -- which carries the override -- and only
afterwards call `gui_utils.update_live_drc_floors`, whose class write is correct
but ONLY-LOWER, so it could never undo the capped value written moments before.
One part with a 2 mil library override therefore shipped a 0.0508 mm board out
of the GUI as well.

WHY THIS FILE AND NOT THE WX-FREE ONE. `tests/test_900_class_clearance_not_capped.py`
drives the FILE writeback end to end, the non-Default clamp against fake net
classes, and `apply_targets_to_board` by SOURCE TEXT only -- that function opens
with `import pcbnew`. A source guard is the weaker instrument (#780: the arm it
replaced there had been green throughout the period the defect existed), and it
is spelling-sensitive: a revert written `(targets or {}).get("min_clearance")`
would not trip its absence half. This file grades the behaviour.

THE FIXTURE. `flat_hierarchy` is the repo's only tracked board declaring a
NON-Default class (Default 0.2, Wide 0.4) -- the same reason the #768 and #782
gates use it -- and it carries NO pad clearance override, which is exactly why
neither of those gates could see this bug. The override is set here, on the
loaded board, through pcbnew's own setter: the quantity under test is what
`board_minima_from_live` reads back, so setting it any other way would be
testing the text parser instead.

ORDERING: 0.0508 (the override) < 0.15 (the routed clearance) < 0.2 (Default)
< 0.4 (Wide). Every class is above the routed value and the routed value is
above the override, so a class landing on 0.0508 and a class landing on 0.15 are
distinguishable, and so is a class that was not written at all.
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration, and the reason is that the defect has no front
# here -- not that the gate is inconvenient.
#
# The GUI half of #900 is `apply_targets_to_board` being called with
# `minima=board_minima_from_live(board)`, from the signal, planes and
# differential tabs. This branch calls NEITHER: the port removed
# `gui_utils.board_minima_from_live` (it existed only to avoid re-parsing the
# board inside a wx TIMER dispatch, over a stale pcbnew pointer that no longer
# exists), and `apply_targets_to_board` is not referenced anywhere under
# `kicad_routing_plugin/` at all. Grep both names before changing this.
#
# What this front does instead: `kicad_ipc_adapter.write_drc_settings_to_project`
# delegates to `fix_kicad_drc_settings.fix_project_for_output` -- the CLI's own
# writeback, which scans the minima itself -- so #900's fix arrives here inside
# the shared function rather than needing a mirror. The routed-vs-capped claim
# is graded by `tests/test_900_class_clearance_not_capped.py`, which drives that
# file writeback end to end and needs no wx.
#
# Left as it arrived, this file died on `ImportError: cannot import name
# 'board_minima_from_live'` -- which exits the same way a satisfied guard does,
# and would have read as a #900 gate that ran.
import sys

if __name__ == '__main__':
    print("SKIP: the GUI half of #900 (apply_targets_to_board +/ "
          "board_minima_from_live) does not exist on ipc-migration; the IPC "
          "writeback goes through fix_project_for_output, and the claim is "
          "graded by tests/test_900_class_clearance_not_capped.py.")
    sys.exit(77)
