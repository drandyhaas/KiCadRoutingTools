#!/usr/bin/env python3
"""#733 follow-up: the cap edge margin has its OWN control, not the signal one.

    python3 tests/test_733_cap_edge_control_decoupled.py

PR743 gave the decoupling-cap repair an edge margin (it previously took the
0.55 signature default because the plugin passed nothing) -- but sourced it
from the dialog's SHARED "Min Edge Clearance (mm)" control, which is the SIGNAL
copper-to-edge keep-out. The two only share a CLI flag SPELLING, across two
independent tools:

    py_router/route.py             --board-edge-clearance   (signal)
    py_placer/place_fanout_clearance.py --board-edge-clearance   (cap placement)

On the CLI they can differ. Through one control they could not, and because
`resolve_cap_edge_clearance` honours an explicit positive value "in both
directions", ticking the shared override at a normal signal value silently
LOOSENED cap placement:

    unticked            -> 0.55      (the default the plugin had always used)
    ticked, 0.20 typed  -> 0.20      <-- 0.35mm looser, unasked for
    ticked, 0.00 typed  -> 0.55      (this case WAS guarded)

That is the direction #733 exists to close, re-opened through a different door.
The margin now lives on the BGA panel's Cap Placement box next to the other
cap knobs, where 0 means "engine resolves it" exactly as an omitted CLI flag
does.

wx-free: this drives the resolver and the source text, not a live dialog.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from placement.fanout_clearance import resolve_cap_edge_clearance

BOARD = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # --- the resolver contract the control depends on ----------------------
    check("unset (None) resolves to the 0.55 default",
          abs(resolve_cap_edge_clearance(BOARD, None)[0] - 0.55) < 1e-9,
          str(resolve_cap_edge_clearance(BOARD, None)))
    check("a non-positive value is UNSET, not a margin of zero",
          abs(resolve_cap_edge_clearance(BOARD, 0.0)[0] - 0.55) < 1e-9,
          str(resolve_cap_edge_clearance(BOARD, 0.0)))
    # This is the arm that makes the coupling harmful: an explicit positive
    # value is honoured DOWNWARD, so a signal-shaped 0.20 really would land.
    check("an explicit positive value is honoured downward (0.20 -> 0.20)",
          abs(resolve_cap_edge_clearance(BOARD, 0.20)[0] - 0.20) < 1e-9,
          str(resolve_cap_edge_clearance(BOARD, 0.20)))

    fg = open(os.path.join(ROOT, 'kicad_routing_plugin', 'fanout_gui.py')).read()
    sg = open(os.path.join(ROOT, 'kicad_routing_plugin', 'swig_gui.py')).read()
    sp = open(os.path.join(ROOT, 'kicad_routing_plugin',
                           'settings_persistence.py')).read()

    # --- the control exists, on the cap panel ------------------------------
    check("the cap panel owns a cap_board_edge_clearance control",
          'self.cap_board_edge_clearance = _cap_spin(' in fg)
    check("the panel emits it in its cap_* config",
          "'cap_board_edge_clearance': (" in fg)

    # --- and the SIGNAL control no longer reaches it -----------------------
    # The change detector: re-point the value at the shared dialog control and
    # this goes red.
    check("swig_gui no longer reads the shared control for the cap margin",
          '_effective_placement_edge_clearance' not in sg,
          "the shared reader is still present")
    check("no call site sources the cap margin from `shared`",
          "'cap_board_edge_clearance': shared.get(" not in fg,
          "a shared.get override is still overriding the panel value")

    # --- persistence, or the setting is lost on close ----------------------
    check("the control is saved",
          "'fanout_bga_cap_board_edge_clearance':" in sp)
    check("the control is restored",
          "if 'fanout_bga_cap_board_edge_clearance' in settings:" in sp)

    # --- the engine still receives it --------------------------------------
    check("the cap engine call still gets the margin",
          re.search(r"board_edge_clearance=fanout_config\.get\(\s*'cap_board_edge_clearance'\)", fg)
          is not None)

    print()
    if fails:
        print(f"FAILED ({len(fails)}): " + ", ".join(fails))
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())
