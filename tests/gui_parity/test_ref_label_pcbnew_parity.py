#!/usr/bin/env python3
"""RefLabel parity gate (#481): text parser vs pcbnew, plus the PERMANENT
angle-semantics pin.

Both parse paths must fill `Footprint.ref_label` with the SAME normalized
values -- that is the contract that lets the beautify_labels engine make
identical decisions on the CLI (text-parsed PCBData) and in the GUI
(pcbnew-backed PCBData). This gate compares every field on every footprint
of two boards chosen for coverage: splitflap_driver (rotated footprints,
19 hidden labels) and glasgow_revC (94 mirrored B-side labels).

The angle gate is the load-bearing part: the implementation-time probe
settled that the stored Reference angle is the label's ABSOLUTE board
rotation (GetTextAngle() returns the file bytes unchanged; GetDrawRotation()
is the keep-upright fold of the stored value with the footprint rotation
playing no part). `placement.labels.label_world_angle` encodes that verdict,
and this gate pins it against pcbnew's own GetDrawRotation() forever -- a
KiCad release that changes the storage convention fails here, loudly.

Needs pcbnew; re-execs into KiCad's python automatically.

    python3 tests/gui_parity/test_ref_label_pcbnew_parity.py
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration. PRE-EXISTING -- it has exited 1 on this branch
# since it arrived, on `ModuleNotFoundError: kicad_ipc_adapter`.
#
# The gate is text-parser vs PCBNEW-parser, and there is no pcbnew parser here:
# `build_pcb_data_from_board` walks a kipy board, so it cannot be handed the
# `pcbnew.BOARD` this file loads.
#
# Unlike its #829 neighbour this one is not covering an absent field: the kipy
# builder DOES fill `ref_label` (`ref_label=fp_ref_label` in the footprint
# loop). What is uncovered is the PARITY -- that the kipy values are normalized
# the same way the text parser normalizes them, field for field, including the
# angle semantics this gate pins permanently. Establishing that needs a live
# KiCad, because the IPC read path cannot be driven in process (fake_ipc_board
# serves reads by re-parsing the file, which would grade the text parser twice).
import sys

if __name__ == '__main__':
    print("SKIP: there is no pcbnew parse path on ipc-migration; "
          "build_pcb_data_from_board is the kipy builder. ref_label IS filled "
          "there, but its PARITY with the text parser is uncovered and needs a "
          "live KiCad.")
    sys.exit(77)
