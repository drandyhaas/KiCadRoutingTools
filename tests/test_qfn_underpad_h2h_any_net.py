#!/usr/bin/env python3
"""The QFN under-pad escape spaces a new via's DRILL from every via this run
placed, whatever its net.

`run_output_conflict` priced a DIFFERENT-net pair at the copper rule alone,
`via_size + clearance`, and applied `via_drill + hole_to_hole` only to a
same-net pair. KiCad's hole-to-hole rule holds for every pair of holes, and on
fine vias it is the larger one:

    0.25 / 0.15 via, clearance 0.09, min_hole_to_hole 0.3:
        copper 0.34, drill 0.45

so a board declaring min_hole_to_hole 0.3 got its 0.4 mm-pitch connector
escaped with a via in every pad, each pair 0.25 hole to hole -- the fanout
printed "Hole-to-hole 0.3mm (from the board's own min_hole_to_hole)", counted
0 grazes, and KiCad graded every pair. The same blindness sat in `via_clears`
against a foreign via, and in the default stagger, which spaced the legs at
the copper rule.

The check is module-level and pure, so it is asserted directly here; the
geometry is the measured one.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))

from qfn_fanout import run_output_conflict

VIA, DRILL, CLR, TW = 0.25, 0.15, 0.09, 0.1
PITCH = 0.4


def conflict(dx, dy, other_net, h2h):
    # A via already placed at the origin on net 1; the candidate sits on its
    # own pad, so neither carries a stub.
    placed = [(0.0, 0.0, 1, 0.0, 0.0)]
    return run_output_conflict(dx, dy, other_net, placed, dx, dy,
                               via_size=VIA, via_drill=DRILL, clearance=CLR,
                               track_width=TW, hole_to_hole=h2h)


def main():
    fails = []

    def check(name, got, want):
        print(f"  {'ok  ' if got == want else 'FAIL'} {name}: {got}")
        if got != want:
            fails.append(name)

    # The measured case: different nets, one pitch apart, h2h 0.3. The
    # holes are 0.25 apart and must be refused.
    check("different net, 0.4 pitch, h2h 0.3 -> rejected",
          conflict(PITCH, 0.0, 2, 0.3), True)
    # The same pair under h2h 0.2 needs 0.35: legal at 0.4.
    check("different net, 0.4 pitch, h2h 0.2 -> accepted",
          conflict(PITCH, 0.0, 2, 0.2), False)
    # Staggered one pitch across and 0.256 along the pad (the stagger the
    # escape now computes): 0.475 centre to centre clears the 0.45 drill rule.
    check("different net, staggered 0.256, h2h 0.3 -> accepted",
          conflict(PITCH, 0.256, 2, 0.3), False)
    # Same net keeps the drill floor it always had.
    check("same net, 0.4 pitch, h2h 0.3 -> rejected",
          conflict(PITCH, 0.0, 1, 0.3), True)
    # Where the copper rule is the larger one it still decides: 0.45/0.2 vias
    # at clearance 0.12 need 0.57 copper vs 0.5 drill.
    placed = [(0.0, 0.0, 1, 0.0, 0.0)]
    check("copper rule still binds standard vias at 0.55",
          run_output_conflict(0.55, 0.0, 2, placed, 0.55, 0.0,
                              via_size=0.45, via_drill=0.2, clearance=0.12,
                              track_width=TW, hole_to_hole=0.3), True)

    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
