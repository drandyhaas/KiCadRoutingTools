#!/usr/bin/env python3
"""#2080: a #530 via-rung sub-build leaves out the via cells the parent build
already holds in the STATIC via bitmap, and every query answers as before.

`is_via_blocked_rung` ORs the static bitmap into every rung, and nothing
clears that bitmap, so a via cell the parent blocks statically is blocked at
every rung whether or not the rung map holds it. The sub-build therefore
skips such cells (`_StaticViaRecord` / `_RungPassStampProxy`), and stamps no
track cells at all (the parent reads only the rung's via cells). A rung the
filter empties keeps ONE statically blocked cell, because an empty rung map
reads as "no rung" and would fall back to rung 0's rules.

Checks, on an in-repo board with per-net via sizes, the filtered build
against the same build with the record disabled (the unfiltered path),
cell by cell over the board box at every rung:
  1. a larger rung, a smaller rung (the filter empties it: the sentinel path)
     and a near-equal rung, each with and without a pour barrier, answer
     identically, and a rung is populated in one build iff in the other;
  2. the filter actually removed rows (else check 1 tests nothing);
  3. a board with no per-net via rung builds no record (it costs up to
     400 MB and nothing would read it).

    python3 tests/test_2080_rung_static_filter.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import numpy as np                                            # noqa: E402

import obstacle_map as om                                     # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from obstacle_cache import via_rungs                          # noqa: E402
from routing_config import GridRouteConfig, GridCoord         # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
FAILED = []


def check(ok, what):
    print(f"  [{'ok' if ok else 'FAIL'}] {what}")
    if not ok:
        FAILED.append(what)


def config(pcb, rung=None, barrier=False):
    vnets = ({nid: rung for nid, n in pcb.nets.items()
              if (n.name or '').lstrip('/').startswith('V')} if rung else {})
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.25,
                          clearance=0.2, grid_step=0.1, via_size=0.6,
                          via_drill=0.4, net_via_sizes=vnets)
    if barrier:
        bb = pcb.board_info.board_bounds
        cx, cy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
        poly = [(cx - 8, cy - 6), (cx + 8, cy - 6), (cx + 8, cy + 6),
                (cx - 8, cy + 6)]
        cfg.pour_barriers = [{'net': 'GND', 'layer': 'B.Cu',
                              'clearance': 0.2, 'polygons': [poly]}]
    return cfg


def build(pcb, cfg, route_ids, filtered):
    orig = om._StaticViaRecord.for_board
    if not filtered:
        om._StaticViaRecord.for_board = classmethod(lambda cls, *a, **k: None)
    try:
        return om.build_base_obstacle_map(pcb, cfg, route_ids, static_base=True)
    finally:
        om._StaticViaRecord.for_board = orig


def main():
    if not os.path.isfile(BOARD):
        print(f"BROKEN TEST: fixture {BOARD} missing")
        return 2
    pcb = parse_kicad_pcb(BOARD)
    route_ids = [nid for nid in pcb.nets if nid != 0][::2]
    coord = GridCoord(0.1)
    bb = pcb.board_info.board_bounds
    x0, y0 = coord.to_grid(bb[0] - 2, bb[1] - 2)
    x1, y1 = coord.to_grid(bb[2] + 2, bb[3] + 2)
    gxs = range(min(x0, x1), max(x0, x1) + 1)
    gys = range(min(y0, y1), max(y0, y1) + 1)

    print("1/2. filtered rung maps answer as the unfiltered ones")
    removed_any = sentinel_seen = False
    for rung, label in (((1.0, 0.7), 'larger'), ((0.5, 0.3), 'smaller'),
                        ((0.62, 0.4), 'near-equal')):
        for barrier in (False, True):
            cfg = config(pcb, rung, barrier)
            rungs = via_rungs(cfg, pcb)
            if not rungs:
                print(f"BROKEN TEST: no via rung for {rung}")
                return 2
            new = build(pcb, cfg, route_ids, True)
            old = build(pcb, cfg, route_ids, False)
            for r in range(len(rungs) + 1):
                lo, ln = old.rung_len(r), new.rung_len(r)
                removed_any |= ln < lo
                sentinel_seen |= (r > 0 and ln == 1 and lo > 1)
                diff = sum(old.is_via_blocked_rung(gx, gy, r)
                           != new.is_via_blocked_rung(gx, gy, r)
                           for gx in gxs for gy in gys)
                tag = f"{label} rung {rung}{' + barrier' if barrier else ''}, r={r}"
                check(diff == 0, f"{tag}: 0 differing cells (got {diff}; "
                                 f"rows {lo} -> {ln})")
                check((lo > 0) == (ln > 0),
                      f"{tag}: populated in both builds or in neither")
    check(removed_any, "the filter removed rows somewhere (check 1 is live)")
    check(sentinel_seen, "a rung the filter emptied kept its one sentinel cell")

    print("3. no per-net via rung -> no record is built")
    calls = []
    orig = om._StaticViaRecord.for_board.__func__

    def counting(cls, *a, **k):
        calls.append(1)
        return orig(cls, *a, **k)
    om._StaticViaRecord.for_board = classmethod(counting)
    try:
        om.build_base_obstacle_map(pcb, config(pcb), route_ids, static_base=True)
        check(not calls, f"record built {len(calls)} time(s) with no rung")
        calls.clear()
        om.build_base_obstacle_map(pcb, config(pcb, (1.0, 0.7)), route_ids,
                                   static_base=True)
        check(bool(calls), "and it IS built when a rung will read it")
    finally:
        om._StaticViaRecord.for_board = classmethod(orig)

    print(f"\n{'FAILED: ' + str(len(FAILED)) if FAILED else 'ALL PASS'}")
    return 1 if FAILED else 0


if __name__ == '__main__':
    sys.exit(main())
