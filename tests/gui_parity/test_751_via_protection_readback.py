#!/usr/bin/env python3
"""#751: the GUI read half of the via-protection round trip, on a pcbnew whose
SWIG wrapper does NOT export the protection enums.

`_pcbnew_via_protection_attrs` compares each mode against `pcbnew.TENTING_MODE_*`
and friends. The shipping KiCad 10.0.0 wrapper exports the SETTERS but not those
values, and hands back an opaque SwigPyObject from the getters -- so building the
comparison table raised inside the function's own `try` and it returned {} for
EVERY via. `build_pcb_data_from_board` therefore reported no spec at all, and a
user who set via-in-pad protection in the GUI lost it to any pass that re-places
a via (the #313 cap nudge, a rip-up, a tap relocation).

It is BUILD-dependent, not version-dependent -- 10.0.0-103-gacbf1898e0 on macOS
exports all ten constants and returns plain ints -- so this gate does not assume
either environment. It runs BOTH:

  * the wrapper as installed, whatever it is; and
  * a SIMULATED blind wrapper (the ten constants deleted from the module), which
    is the reporter's environment reproduced on any machine.

The second arm carries its own NEGATIVE CONTROL: it first asserts that the raw
live-object reader really does go blind (returns {}), so the arm cannot pass on
`{} == {}` -- the exact vacuity #751 warns about -- and only then asserts that
the resolver still delivers the full spec from the board file.

Needs pcbnew; re-execs into KiCad's python automatically.

    python3 tests/gui_parity/test_751_via_protection_readback.py
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration. PRE-EXISTING -- this file has been red on this
# branch since it arrived, and it exited 1 on an ImportError, which is the
# shape a real failure has.
#
# Every half it grades is SWIG:
#   * `gui_utils.apply_via_protection` (the write half) does not exist here --
#     there is no kipy via-protection writer at all, which is the standing gap
#     the port has carried since the 0830 sync.
#   * `build_pcb_data_from_board` is the kipy builder on this branch, so handing
#     it a `pcbnew.BOARD` -- which this file does, twice -- cannot work.
#   * `pcbnew_via_protection_attrs` / `via_protection_attrs_from_board_file` /
#     `pcbnew_protection_accessors_usable` still exist in `kicad_parser`, but
#     NOTHING on this branch calls them (grepped): they are the SWIG GUI's
#     reader, and the SWIG GUI is gone.
#
# WHAT IS ACTUALLY TRUE HERE, and is worse than an unrunnable gate: the kipy via
# read fills NO `tenting_attrs` and NO `locked` at all (see the `get_vias()`
# loop in `build_pcb_data_from_board`). So on this front
#   * a via the plugin RE-PLACES loses its protection spec, the #489/#741
#     defect this gate exists to catch, with no reader to blame; and
#   * `via.locked` / `segment.locked` are always False, so #521's "KiCad-LOCKED
#     copper is never rippable, with NO override" does not hold on IPC.
# Neither is caused by this file being skipped, and neither is fixed by
# un-skipping it: they need a kipy reader, and then a gate that drives the kipy
# path. Both are recorded here because this is the file someone reaches for.
import sys

if __name__ == '__main__':
    print("SKIP: #751 is a SWIG round trip -- gui_utils.apply_via_protection "
          "does not exist on ipc-migration and build_pcb_data_from_board is "
          "the kipy builder. NOTE the real gap this gate cannot cover: the "
          "kipy via read fills neither tenting_attrs nor locked.")
    sys.exit(77)
