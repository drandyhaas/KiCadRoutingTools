#!/usr/bin/env python3
"""Regression test for issue #505 -- the DRC-floor writeback must clamp the min
copper WEB (`min_connection`) down to the routed floor, like every other floor.

    python3 tests/test_505_min_connection_clamp.py

KiCad's connection_width rule grades the narrowest copper joining two areas. A
track IS that web wherever it is the sole connection, so a web floor left ABOVE
the track floor condemns copper the router deliberately laid. The writeback
clamped `min_track_width` down to what was routed but never touched
`min_connection`, leaving a self-inconsistent rule set:

    unrouted baseline : min_connection 0.127, min_track_width 0.127  (consistent)
    every routed step : min_connection 0.127, min_track_width 0.0889 (NOT)

This was the single largest DRC number in the corpus and essentially none of it
was real. ulx5m_gatemate: 13495 of its 14515 tracks sit under the stock 0.127
web rule, and grading through kicad_drc_compare's staged copy reported 4749
connection_width items; with the web rule clamped to the routed floor the SAME
board reports 2, every other DRC count unchanged. Same principle as the #439
clearance ceiling: grade what was built, not the stock aspiration.

Two behaviours are asserted, because the clamp must loosen WITHOUT ever
switching on a check the author had off (KiCad's connection_width checker is
disabled at 0, and an absent key means 0):
  1. a web rule ABOVE the routed track floor is lowered to it;
  2. a rule already below it, an explicit 0, and an ABSENT key are all left
     exactly as they are.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

from fix_kicad_drc_settings import (  # noqa: E402
    compute_targets, apply_targets_to_project)

fails = []


def check(name, cond):
    print(("  ok  " if cond else " FAIL ") + name)
    if not cond:
        fails.append(name)


ROUTED_TW = 0.0889


def writeback(cur_web):
    """Run the floors writeback over a project whose web rule is `cur_web`
    (None = key absent) and return the resulting min_connection."""
    rules = {"min_track_width": 0.127, "min_clearance": 0.1}
    if cur_web is not None:
        rules["min_connection"] = cur_web
    pro = {"board": {"design_settings": {"rules": rules}}}
    targets = compute_targets(clearance=0.1, track_width=ROUTED_TW,
                              minima={"min_track_width": ROUTED_TW})
    apply_targets_to_project(pro, targets, {})
    return pro["board"]["design_settings"]["rules"].get("min_connection")


# 1. The bug: a stock web rule above the routed floor must come DOWN to it.
got = writeback(0.127)
check(f"stale web rule 0.127 clamped to the routed floor {ROUTED_TW} (got {got})",
      got == ROUTED_TW)

# 2. Only-lower: a board already tighter than the routed floor is untouched.
got = writeback(0.05)
check(f"web rule already below the routed floor is left alone (got {got})",
      got == 0.05)

# 3. Never switch the checker ON. KiCad disables connection_width at 0, and an
#    absent key means 0 -- writing a value there would invent a constraint the
#    author never enabled, even though the number itself is a loosening.
got = writeback(0.0)
check(f"explicit 0 (checker OFF) stays off (got {got})", got == 0.0)
got = writeback(None)
check(f"absent key (checker OFF) is not introduced (got {got})", got is None)

# 4. The clamp must not disturb the track floor it is derived from.
rules = {"min_track_width": 0.127, "min_connection": 0.127, "min_clearance": 0.1}
pro = {"board": {"design_settings": {"rules": rules}}}
targets = compute_targets(clearance=0.1, track_width=ROUTED_TW,
                          minima={"min_track_width": ROUTED_TW})
apply_targets_to_project(pro, targets, {})
out = pro["board"]["design_settings"]["rules"]
check(f"min_track_width still clamped to {ROUTED_TW} (got {out.get('min_track_width')})",
      out.get("min_track_width") == ROUTED_TW)
check("web rule never ends up ABOVE the track floor",
      out.get("min_connection") <= out.get("min_track_width") + 1e-9)
# #900: `compute_targets` also emits `class_clearance`, which is NOT a KiCad
# rule. This file feeds its result straight into both writebacks, so it is the
# cheapest place in the suite to notice the day one leaks into the project --
# every other assertion here reads named keys and would stay green.
check("no non-rule key reached the project's rules",
      "class_clearance" not in out)

# --------------------------------------------------------------------------
# GUI parity. The pcbnew path writes BOARD_DESIGN_SETTINGS fields whose names
# vary by KiCad version, so it is guarded by hasattr -- which means a WRONG
# attribute name fails SILENTLY and the GUI would never get the clamp. Pin the
# real name (m_MinConn, confirmed present in BOARD_DESIGN_SETTINGS) and the same
# three semantics. Skips cleanly under a python without pcbnew; run it with
# KiCad's interpreter to exercise it.
# --------------------------------------------------------------------------
try:
    import pcbnew  # noqa: F401
except ImportError:
    print("  [skip] pcbnew not importable -- GUI path not exercised "
          "(run under KiCad's python to cover it)")
else:
    from fix_kicad_drc_settings import apply_targets_to_board  # noqa: E402
    MM = 1e6
    _t = compute_targets(clearance=0.1, track_width=ROUTED_TW,
                         minima={"min_track_width": ROUTED_TW})
    check("pcbnew exposes m_MinConn (else the clamp silently no-ops)",
          hasattr(pcbnew.BOARD().GetDesignSettings(), "m_MinConn"))

    def gui(start_mm):
        b = pcbnew.BOARD()
        bds = b.GetDesignSettings()
        bds.m_TrackMinWidth = int(0.127 * MM)
        bds.m_MinConn = int(start_mm * MM)
        apply_targets_to_board(b, _t, {})
        return round(bds.m_MinConn / MM, 6)

    check(f"GUI: stale 0.127 clamped to {ROUTED_TW} (got {gui(0.127)})",
          gui(0.127) == ROUTED_TW)
    check(f"GUI: already-tighter 0.05 left alone (got {gui(0.05)})",
          gui(0.05) == 0.05)
    check(f"GUI: checker OFF stays off (got {gui(0.0)})", gui(0.0) == 0.0)

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")
