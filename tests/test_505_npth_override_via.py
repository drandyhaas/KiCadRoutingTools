#!/usr/bin/env python3
"""Regression test for issue #505 -- an NPTH pad's clearance OVERRIDE was
enforced against nothing when the offending copper was a VIA.

KiCad's hole_clearance rule holds a via's COPPER (not merely its drill)
``pad.local_clearance`` off the hole wall. Two independent gaps let that fall
through completely:

  * ROUTER: the only via keep-out near a drill is the hole-to-hole DRILL
    minimum (``block_via_cells_near_drills`` at ``hole_to_hole_clearance``).
    An NPTH pad has NO copper, so the pad-copper blocker -- which does honor
    local_clearance (#326) -- never stamps it either.
  * CHECK_DRC: the copper-to-hole pass honors the override but walks TRACKS
    only; vias were covered solely by a drill-to-drill check.

pinci shipped 5 vias 0.76-1.13mm from 0.9/1.3mm-override mounting holes -- every
one satisfying hole-to-hole (1.427mm) while KiCad wanted 2.225/2.625mm, and
check_drc reported the board clean.

The false-positive direction matters just as much: an NPTH pad with NO override
must NOT be graded at the project's NPTH fab floor (NPTH_TO_TRACK_CLEARANCE,
0.20), which is a routing policy rather than a KiCad rule. Grading vias against
it invented 7 phantoms on crkbd at 0.016-0.023mm.

    python3 tests/test_505_npth_override_via.py
"""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from check_drc import run_drc  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from obstacle_map import GridObstacleMap, add_drill_hole_obstacles  # noqa: E402
from routing_config import GridRouteConfig, GridCoord  # noqa: E402

fails = []


def check(name, cond):
    print(("  ok  " if cond else " FAIL ") + name)
    if not cond:
        fails.append(name)


def _rect_edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'(layer "Edge.Cuts") (width 0.1))' for i in range(4))


# NPTH mounting hole at (15,15), 2.1mm drill. VIA_NEAR sits 1.5mm away: outside
# the hole-to-hole drill minimum, but inside a 0.9mm copper override
# (1.05 + 0.275 + 0.9 = 2.225 required). VIA_FAR at 3.0mm clears both.
def board(clearance_tok, via_at):
    # via_at=None builds the board with NO via -- required for the router
    # probes: a via sitting AT the probe point blocks that cell via its own
    # hole-to-hole keep-out, which would mask what is being measured.
    via = (f' (via (at {via_at[0]} {via_at[1]}) (size 0.55) (drill 0.254)\n'
           f'      (layers "F.Cu" "B.Cu") (net 1) (uuid "v"))' if via_at else '')
    return f'''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "SIG")
{_rect_edge(0, 0, 30, 30)}
 (footprint "t:mh" (layer "F.Cu") (at 15 15)
  (property "Reference" "H1" (at 0 -2) (layer "F.SilkS"))
  (pad "" np_thru_hole circle (at 0 0) (size 2.1 2.1) (drill 2.1)
       (layers "F&B.Cu" "*.Mask"){clearance_tok})
 )
 (segment (start 4 4) (end 8 4) (width 0.2) (layer "F.Cu") (net 1) (uuid "s"))
{via}
)'''


def write(txt):
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(txt)
        return f.name


NEAR = (15 + 1.5, 15.0)   # 1.5mm from the hole centre
FAR = (15 + 3.0, 15.0)    # 3.0mm -- clears even the 0.9 override


def via_hole_items(path, clearance=0.2):
    return [v for v in run_drc(path, clearance=clearance, quiet=True,
                               print_summary=False, hole_to_hole_clearance=0.25)
            if v['type'] == 'via-hole']


# --------------------------------------------------------------------------
# 1. check_drc SEES the override violation, and reports KiCad's requirement
# --------------------------------------------------------------------------
p = write(board(' (clearance 0.9)', NEAR))
try:
    items = via_hole_items(p)
    check("override + near via -> check_drc reports via-hole", len(items) == 1)
    if items:
        check("...at the pad's override, not the fab floor",
              abs(items[0].get('required_mm', 0) - 0.9) < 1e-9)
finally:
    os.unlink(p)

# --------------------------------------------------------------------------
# 2. NO false positive: same geometry, NO override, graded at a clearance
#    BELOW the NPTH fab floor so the two differ. Required center distance is
#    1.05 + 0.275 + 0.127 = 1.452 at KiCad's clearance but 1.525 at the 0.20
#    floor; a via at 1.49mm sits between them. KiCad reports nothing there, so
#    neither may we (this is exactly the crkbd phantom: 7 items, 0.016-0.023mm).
# --------------------------------------------------------------------------
p = write(board('', (15 + 1.49, 15.0)))
try:
    check("no override -> no via-hole item at the NPTH fab floor",
          not via_hole_items(p, clearance=0.127))
finally:
    os.unlink(p)

# --------------------------------------------------------------------------
# 3. A via that genuinely clears the override is clean
# --------------------------------------------------------------------------
p = write(board(' (clearance 0.9)', FAR))
try:
    check("override + far via -> clean", not via_hole_items(p))
finally:
    os.unlink(p)


# --------------------------------------------------------------------------
# 4. ROUTER: the via keep-out must block cells inside the override, and must
#    NOT block cells that only the fab floor would have covered.
# --------------------------------------------------------------------------
def blocked_at(path, xy):
    pcb = parse_kicad_pcb(path)
    cfg = GridRouteConfig(layers=["F.Cu", "B.Cu"], grid_step=0.1, clearance=0.2,
                          track_width=0.2, via_size=0.55, via_drill=0.254,
                          hole_to_hole_clearance=0.25)
    obs = GridObstacleMap(len(cfg.layers))
    add_drill_hole_obstacles(obs, pcb, cfg, set())
    gx, gy = GridCoord(cfg.grid_step).to_grid(*xy)
    return obs.is_via_blocked(gx, gy)


p = write(board(' (clearance 0.9)', None))
try:
    check("router blocks a via cell inside the override", blocked_at(p, NEAR))
    check("router leaves a via cell outside the override free",
          not blocked_at(p, FAR))
finally:
    os.unlink(p)

# Without an override the same near cell must stay routable -- the fix must not
# quietly tighten every NPTH hole into a via keep-out. NEAR (1.5mm) clears the
# hole-to-hole minimum of 1.05 + 0.127 + 0.25 = 1.427mm.
p = write(board('', None))
try:
    hole_r, via_r, h2h = 1.05, 0.254 / 2.0, 0.25
    assert math.hypot(NEAR[0] - 15, NEAR[1] - 15) > hole_r + via_r + h2h
    check("no override -> near cell stays routable (h2h still satisfied)",
          not blocked_at(p, NEAR))
finally:
    os.unlink(p)


# --------------------------------------------------------------------------
# 5. #1038: a DECLARED copper-to-hole floor (min_hole_clearance / its
#    fab_floor_origin / an explicit config.hole_clearance) holds via COPPER
#    off the hole on BOTH sides. NEAR (1.5mm) clears h2h (1.427) but its
#    copper edge sits 1.5 - 1.05 - 0.275 = 0.175mm from the wall, inside a
#    declared 0.25 (needs 1.575). The no-override case above pins that a board
#    whose project declares NOTHING is untouched. In a real chain that is only
#    step 1: route.py's DRC writeback writes rules.min_hole_clearance into
#    each route step's output project, so every later step reads as
#    declaring a floor.
# --------------------------------------------------------------------------
def blocked_at_declared(path, xy, hole_clr):
    pcb = parse_kicad_pcb(path)
    cfg = GridRouteConfig(layers=["F.Cu", "B.Cu"], grid_step=0.1, clearance=0.2,
                          track_width=0.2, via_size=0.55, via_drill=0.254,
                          hole_to_hole_clearance=0.25)
    cfg.hole_clearance = hole_clr
    obs = GridObstacleMap(len(cfg.layers))
    add_drill_hole_obstacles(obs, pcb, cfg, set())
    gx, gy = GridCoord(cfg.grid_step).to_grid(*xy)
    return obs.is_via_blocked(gx, gy)


p = write(board('', None))
try:
    check("#1038 declared 0.25 -> router blocks the via cell whose copper "
          "would sit 0.175mm off the hole", blocked_at_declared(p, NEAR, 0.25))
    check("#1038 declared 0.25 -> a cell clearing it (1.6mm) stays free",
          not blocked_at_declared(p, (15 + 1.6, 15.0), 0.25))
finally:
    os.unlink(p)

p = write(board('', NEAR))
try:
    items = [v for v in run_drc(p, clearance=0.2, quiet=True,
                                print_summary=False,
                                hole_to_hole_clearance=0.25,
                                hole_clearance=0.25)
             if v['type'] == 'via-hole']
    check("#1038 check_drc grades the via at the declared hole floor",
          len(items) == 1
          and abs(items[0].get('required_mm', 0) - 0.25) < 1e-9)
    check("#1038 ...and at clearance 0.2 alone (nothing declared) it is "
          "still a via-hole item at the routing clearance, unchanged",
          len(via_hole_items(p)) == 1)
finally:
    os.unlink(p)

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")
