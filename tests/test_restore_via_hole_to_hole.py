#!/usr/bin/env python3
"""A restored via clears a foreign via's DRILL at the hole-to-hole floor.

Both restore gates -- `rip_restore._conflict_sweep` (the terminal restore of a
rip victim, #468) and `rip_up_reroute._saved_route_colliders` (#134's restore
test and its piece-level settles) -- priced a via pair on COPPER only. Between
two fab-tier vias the drill floor is the binding measure: a ripped net's
0.3/0.15 via was put back 0.494 mm centre to centre from a foreign 0.25/0.15
via-in-pad laid while it was ripped. The copper gap (0.219 mm) cleared the
0.09 clearance, the drills sat 0.156 mm inside the board's 0.5 hole-to-hole
floor, and the board shipped a real DRC.

Checks (the exact geometry, clearance 0.09, hole-to-hole 0.5):
  1. both gates refuse the restore; the terminal restore is no longer 'full';
  2. copper-only control: at the 0.2 default floor the same pair is clear,
     so the refusal is the drill floor and nothing else;
  3. past the floor (0.66 mm apart) the restore is admitted;
  4. a pair of vias the step was HANDED (mark_input_copper) stood on the input
     board, so it does not refuse the restore.

    python3 tests/test_restore_via_hole_to_hole.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from routing_config import GridRouteConfig                  # noqa: E402
from synth import make_via, make_pcb                        # noqa: E402
import rip_restore                                          # noqa: E402
import rip_up_reroute as rr                                 # noqa: E402

OWN, FOREIGN = 39, 38
failures = []


def check(label, got, want):
    ok = got == want
    print(f"  [{'ok' if ok else 'FAIL'}] {label}: got {got!r}, want {want!r}")
    if not ok:
        failures.append(label)


def cfg(h2h=0.5):
    return GridRouteConfig(clearance=0.09, track_width=0.1, via_size=0.45,
                           via_drill=0.3, hole_to_hole_clearance=h2h,
                           layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                           grid_step=0.05)


def own_via(x=232.70, y=112.80):
    return make_via(x, y, net_id=OWN, size=0.3, drill=0.15)


def foreign_via():
    return make_via(232.385, 113.18, net_id=FOREIGN, size=0.25, drill=0.15)


def verdicts(saved, board, config, mark=False):
    """(terminal-restore conflict, #134 restore collision) for one pair."""
    pcb = make_pcb(vias=[board])
    if mark:
        pcb.vias.append(saved)       # both vias came in with the step
        rr.mark_input_copper(pcb)
        pcb.vias.remove(saved)       # ...and the own one was then ripped
    a = rip_restore._copper_conflicts(pcb, config, {OWN}, [], [saved])
    b = rr._saved_route_collides({'new_segments': [], 'new_vias': [saved]},
                                 pcb, [OWN], config.clearance, config=config)
    return a, b


print("1. the exact pair at a 0.5 hole-to-hole floor")
check("both gates refuse", verdicts(own_via(), foreign_via(), cfg()),
      (True, True))
pcb = make_pcb(vias=[foreign_via()])
pcb._rip_saved = {OWN: ({'new_segments': [], 'new_vias': [own_via()]},
                        [OWN], True)}
check("terminal restore is not 'full'",
      rip_restore.try_terminal_restore(pcb, cfg(), OWN), None)

print("2. copper-only control: the 0.2 default floor admits the same pair")
check("both gates admit", verdicts(own_via(), foreign_via(), cfg(0.2)),
      (False, False))

print("3. past the floor")
# 0.66 mm centre to centre: drills 0.51 apart, over the 0.5 floor
check("both gates admit", verdicts(own_via(232.385, 113.18 - 0.66),
                                   foreign_via(), cfg()), (False, False))

print("4. a pair the step was handed")
check("both gates admit", verdicts(own_via(), foreign_via(), cfg(), mark=True),
      (False, False))

print()
if failures:
    print(f"FAIL: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("PASS: test_restore_via_hole_to_hole")
