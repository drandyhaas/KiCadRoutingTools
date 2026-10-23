#!/usr/bin/env python3
"""Overlapping plane zones must fill DETERMINISTICALLY, and the oracle must
survive a net whose only copper is vias.

Both defects were found together on usp_obc_v7 (set19), where an A/B reported
"1 net unconnected, completion 100% -> 97.8%" over copper that was
BIT-IDENTICAL between the two runs -- a phantom regression:

  1. route_planes' Voronoi cells are NOT guaranteed disjoint: In1.Cu came out
     with the whole Net-(U5-GND) pour nested inside the GND cell, both at the
     default priority 0. KiCad fills higher-priority zones first and only
     lower-priority zones pull back, so equal-priority overlapping zones have
     no defined winner -- it tie-breaks on the zones' KIIDs, i.e. their UUIDs.
     We mint a fresh uuid4 per zone per run, so the FILL moved between runs:
     the same board graded "1 net unconnected" on 4 of 6 UUID rolls. Emitting
     DISTINCT priorities removes the ambiguity at the source (verified: link
     count went from {32,33} across 8 UUID rolls to a constant 32).

  2. kicad_oracle._net_track_components returned bare [] label lists on its
     no-component paths while still returning the populated `vias` list.
     Callers index them by enumerate() position (`comp_of_via[j]`), so any net
     with vias but NO segments -- Net-(U5-GND): 2 vias, 2 pads, fed entirely
     by zone fill -- raised IndexError. That took down exact_clusters(), so
     the DETERMINISTIC pcbnew link source (#490) fell back to kicad-cli, and
     it took down oracle_reconnect() outright, killing route_disconnected_
     planes before its .kicad_pro writeback -- after which the board graded
     against the stock netclass and sprouted 48 phantom violations (#441).

    python3 tests/test_zone_priority_determinism.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_tools'))  # #522

from kicad_writer import (  # noqa: E402
    zone_overlap_priorities, generate_zone_sexpr, _polygons_overlap)
from kicad_oracle import _net_track_components, _cluster_points  # noqa: E402

BIG = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
SMALL = [(40.0, 40.0), (60.0, 40.0), (60.0, 60.0), (40.0, 60.0)]
TINY = [(45.0, 45.0), (50.0, 45.0), (50.0, 50.0), (45.0, 50.0)]
FAR = [(200.0, 200.0), (210.0, 200.0), (210.0, 210.0), (200.0, 210.0)]

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name
          + (f"  [{detail}]" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


def main():
    # ---- 1. zone priorities -------------------------------------------------
    p = zone_overlap_priorities([("In1.Cu", 1, BIG), ("In1.Cu", 2, SMALL)])
    check("nested pours on one layer get DISTINCT priorities",
          len(set(p)) == 2, f"{p}")
    check("the SMALLER (nested) pour wins the overlap",
          p[1] > p[0], f"{p}")

    # Same net: the copper merges either way, but KiCad's DRC calls two
    # overlapping zones at equal priority `zones_intersect` (argus_mainboard's
    # split In2.Cu GND), so the tie is split, the smaller a level above --
    # and only that tie: every other pair keeps its order and its ties.
    p = zone_overlap_priorities([("In1.Cu", 1, BIG), ("In1.Cu", 1, SMALL)])
    check("same-net overlap gets distinct priorities, the smaller winning",
          p == [0, 1], f"{p}")

    p = zone_overlap_priorities([("In1.Cu", 1, BIG), ("In2.Cu", 2, SMALL)])
    check("zones on DIFFERENT layers never contend", p == [0, 0], f"{p}")

    p = zone_overlap_priorities([("In1.Cu", 1, BIG), ("In1.Cu", 2, FAR)])
    check("disjoint zones stay at priority 0", p == [0, 0], f"{p}")

    # ADJACENCY IS NOT OVERLAP. Voronoi cells tile the board, so neighbours
    # share a boundary and their vertices sit exactly on each other's edges --
    # where the even-odd rule is undefined. Counting that as overlap re-fills
    # zone pairs that were never ambiguous (scalenode_cm4: 13 real overlaps
    # read as 31).
    LEFT = [(0.0, 0.0), (50.0, 0.0), (50.0, 100.0), (0.0, 100.0)]
    RIGHT = [(50.0, 0.0), (100.0, 0.0), (100.0, 100.0), (50.0, 100.0)]
    check("zones sharing only an EDGE do not count as overlapping",
          not _polygons_overlap(LEFT, RIGHT))
    p = zone_overlap_priorities([("In1.Cu", 1, LEFT), ("In1.Cu", 2, RIGHT)])
    check("adjacent tiling cells stay at priority 0", p == [0, 0], f"{p}")
    check("a genuinely CROSSING pair is still caught",
          _polygons_overlap(BIG, [(50.0, 50.0), (150.0, 50.0),
                                  (150.0, 150.0), (50.0, 150.0)]))

    p = zone_overlap_priorities([("In1.Cu", 1, BIG), ("In1.Cu", 2, SMALL),
                                 ("In1.Cu", 3, TINY)])
    check("three nested pours get three distinct priorities",
          sorted(p) == [0, 1, 2], f"{p}")

    # The whole point is order-independence: a dict-iteration reshuffle must
    # not change which zone wins, or the fill moves run to run again.
    a = zone_overlap_priorities([("In1.Cu", 1, BIG), ("In1.Cu", 2, SMALL)])
    b = zone_overlap_priorities([("In1.Cu", 2, SMALL), ("In1.Cu", 1, BIG)])
    check("priority is independent of input order",
          a == [0, 1] and b == [1, 0], f"{a} vs {b}")

    # Equal-area overlapping zones still must be separated (deterministically).
    SHIFT = [(50.0, 0.0), (150.0, 0.0), (150.0, 100.0), (50.0, 100.0)]
    p = zone_overlap_priorities([("In1.Cu", 1, BIG), ("In1.Cu", 2, SHIFT)])
    check("partially-overlapping equal-area zones are still separated",
          len(set(p)) == 2, f"{p}")

    # ---- 2. the emitted s-expression ---------------------------------------
    s = generate_zone_sexpr(2, "NET_A", "In1.Cu", SMALL, priority=1)
    check("priority is emitted into the zone s-expr", "(priority 1)" in s)
    s0 = generate_zone_sexpr(1, "GND", "In1.Cu", BIG, priority=0)
    check("priority 0 emits NO line (unambiguous boards unchanged)",
          "(priority" not in s0)

    # ---- 3. via-only net must not crash the oracle --------------------------
    via = SimpleNamespace(net_id=3, x=5.0, y=5.0, size=0.6, drill=0.3,
                          layers=['F.Cu', 'B.Cu'])
    via2 = SimpleNamespace(net_id=3, x=9.0, y=9.0, size=0.6, drill=0.3,
                           layers=['F.Cu', 'B.Cu'])
    pcb = SimpleNamespace(segments=[], vias=[via, via2], pads_by_net={},
                          nets={}, zones=[])
    comp_of_seg, comp_of_via, segs, vias, _ = _net_track_components(pcb, 3)
    check("via-only net: labels are parallel to the via list",
          len(comp_of_via) == len(vias) == 2,
          f"{len(comp_of_via)} labels vs {len(vias)} vias")
    check("via-only net: no segments, no segment labels",
          len(comp_of_seg) == len(segs) == 0)

    # This is the call that raised IndexError in exact_clusters() and
    # oracle_reconnect(): scanning the vias for the reported point's component.
    try:
        pts, root = _cluster_points(pcb, 3, 5.0, 5.0, 'F.Cu',
                                    (comp_of_seg, comp_of_via, segs, vias, {}))
        ok, detail = True, ""
    except Exception as e:  # noqa: BLE001 - the regression is ANY exception
        ok, detail = False, f"{type(e).__name__}: {e}"
        pts, root = None, None
    check("_cluster_points survives a via-only net", ok, detail)
    if ok:
        check("unknown component degrades to the single reported point",
              root is None and pts == [(5.0, 5.0, 'F.Cu')], f"{pts}, {root}")

    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s): {FAILS}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
