#!/usr/bin/env python3
"""#1609: a pair of unmoved INPUT copper never refuses a restore.

A step rips a net an earlier step routed, cannot reroute it, and puts the
saved copper back only if no foreign copper is within clearance of it. That
test priced every pair at THIS step's clearance, so copper an earlier step
laid at a smaller one "collided" with the neighbours it sat beside on the
input board, and the refused restore cascaded into rips of more input nets.

Both restore tests -- `rip_up_reroute._saved_route_colliders` (the #134
restore) and `rip_restore._conflict_sweep` (the #468 stub restore) -- now skip
a pair whose two items were both handed to the step (mark_input_copper) and
both still sit where the input had them (`input_pair_unmoved`).

Checks, for each test:
  1. Both input, unmoved, 0.15 apart at a 0.2 step: not a collision
     (seg/seg, seg/via, via/via, via/seg).
  2. The foreign item MOVED since the mark: a collision.
  3. The restored item moved since the mark: a collision.
  4. The foreign item is copper laid after the mark: a collision.
  5. No mark at all: a collision, as before.

    python3 tests/test_1609_restore_input_pair.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from routing_config import GridRouteConfig                  # noqa: E402
from synth import make_seg, make_via, make_pcb              # noqa: E402
import rip_up_reroute as rr                                 # noqa: E402
import rip_restore                                          # noqa: E402

OWN, FOREIGN = 1, 9
failures = []


def check(label, got, want):
    ok = got == want
    print(f"  [{'ok' if ok else 'FAIL'}] {label}: got {got!r}, want {want!r}")
    if not ok:
        failures.append(label)


def cfg():
    return GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                           via_drill=0.3, layers=['F.Cu', 'B.Cu'], grid_step=0.05)


def ripped_board(own_items, foreign_items, mark=True):
    """The input board with both nets, marked, then OWN's copper ripped."""
    segs = [o for o in own_items + foreign_items if hasattr(o, 'start_x')]
    vias = [o for o in own_items + foreign_items if not hasattr(o, 'start_x')]
    pcb = make_pcb(segments=segs, vias=vias)
    if mark:
        rr.mark_input_copper(pcb)
    pcb.segments = [s for s in pcb.segments if s.net_id != OWN]
    pcb.vias = [v for v in pcb.vias if v.net_id != OWN]
    return pcb


def both_tests(pcb, own_items):
    segs = [o for o in own_items if hasattr(o, 'start_x')]
    vias = [o for o in own_items if not hasattr(o, 'start_x')]
    a = rr._saved_route_collides({'new_segments': segs, 'new_vias': vias},
                                 pcb, [OWN], 0.2, config=cfg())
    b = rip_restore._copper_conflicts(pcb, cfg(), {OWN}, segs, vias)
    return (a, b)


# 0.2 mm tracks / 0.6 mm vias with a 0.15 mm edge gap: a graze at the 0.2 step.
def own_seg():
    return make_seg(0, 0, 10, 0, net_id=OWN, width=0.2)


def foreign_seg():
    return make_seg(0, 0.35, 10, 0.35, net_id=FOREIGN, width=0.2)


def own_via():
    return make_via(0, 0, net_id=OWN, size=0.6, drill=0.3)


CASES = {
    'seg/seg': (own_seg, foreign_seg),
    'seg/via': (own_seg, lambda: make_via(5, 0.55, net_id=FOREIGN, size=0.6, drill=0.3)),
    'via/via': (own_via, lambda: make_via(0.75, 0, net_id=FOREIGN, size=0.6, drill=0.3)),
    'via/seg': (own_via, lambda: make_seg(-5, 0.55, 5, 0.55, net_id=FOREIGN, width=0.2)),
}
COLLIDE = (True, True)
CLEAR = (False, False)

for name, (mk_own, mk_foreign) in CASES.items():
    print(f"{name}")
    o, f = mk_own(), mk_foreign()
    check("1. both input, unmoved", both_tests(ripped_board([o], [f]), [o]), CLEAR)

    o, f = mk_own(), mk_foreign()
    pcb = ripped_board([o], [f])
    if hasattr(f, 'start_x'):
        f.start_x -= 0.01
    else:
        f.x -= 0.01
    check("2. foreign moved since the mark", both_tests(pcb, [o]), COLLIDE)

    o, f = mk_own(), mk_foreign()
    pcb = ripped_board([o], [f])
    if hasattr(o, 'start_x'):
        o.end_x += 0.01
    else:
        o.y += 0.001
    check("3. restored item moved since the mark", both_tests(pcb, [o]), COLLIDE)

    o, f = mk_own(), mk_foreign()
    pcb = ripped_board([o], [f])
    late = mk_foreign()
    if hasattr(late, 'start_x'):
        pcb.segments = [s for s in pcb.segments if s is not f] + [late]
    else:
        pcb.vias = [v for v in pcb.vias if v is not f] + [late]
    check("4. foreign laid after the mark", both_tests(pcb, [o]), COLLIDE)

    o, f = mk_own(), mk_foreign()
    check("5. no mark", both_tests(ripped_board([o], [f], mark=False), [o]), COLLIDE)

print()
if failures:
    print(f"FAIL: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("PASS: test_1609_restore_input_pair")
