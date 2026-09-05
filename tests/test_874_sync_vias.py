#!/usr/bin/env python3
"""#874: sync_pcb_data_segments reconciles VIAS too, not only segments.

WHAT WAS WRONG. `routing_common.sync_pcb_data_segments` reconciled
`pcb_data.segments` against the routed results and referenced vias zero times,
while the OUTPUT is written from "input-file text + the RESULTS" and never from
`pcb_data.vias`. So the two lists could disagree about the same board, in both
directions:

  * a result via absent from `pcb_data` is SHIPPED copper no obstacle map can
    see -- and route.py calls this function immediately before Phase 3 tap
    routing, whose whole stated contract is "taps see the post-length-matching
    copper as obstacles";
  * a superseded via still in `pcb_data` BLOCKS the map where nothing will be
    written.

`apply_meanders_to_diff_pair` produces both at once: it replaces a coupled
pair result's `new_vias` wholesale with fresh objects (`_float_path_to_geometry`
plus `_create_gnd_vias`) AFTER `add_route_to_pcb_data` committed the old ones.

MEASURED, before the fix (census printed at the sync point):
  * ddr5_testbed, route.py step with --length-match-group 'CA*' 'DQ*':
    56 unique result vias, **1 MISSING** from pcb_data -- VDDQ at
    (139.300, 89.200), which IS present in the written output board. 0 phantoms.
  * ddr5_testbed route_diff and orangecrab route_diff: 0 missing, 0 phantom
    (their pairs took the hybrid-escape path, so no meander rebuild landed).
The phantom population is 0 on boards where no meander rebuild happens, which
is why the removal half keeps every via that is an input-file original OR is
referenced by any current result: it can only drop genuinely superseded
barrels.

THREE CHECKS:

  1. UNIT, the meander-rebuild shape: commit a result, then swap its
     `new_vias` for fresh objects the way apply_meanders_to_diff_pair does,
     sync, and assert the new via is ON the board, the superseded one is GONE,
     and the input-file original SURVIVED.
  2. UNIT, the keep-alives: a via belonging to a routed net that is neither an
     original nor in any result is NOT dropped when it is carried by another
     result (the pair-partner / GND case), and omitting `original_via_ids`
     leaves `pcb_data.vias` untouched (callers with no keep-alive cannot tell
     an original from a superseded via, so the via half must not run at all).
  3. WIRING: both callers actually pass a via keep-alive snapshot taken at the
     same point as the segment one. A fix in the engine that no caller reaches
     is inert, and only a negative control finds that.

NEGATIVE CONTROL (measured, the via half stubbed to a no-op): check 1 fails on
"meandered via on board" and "superseded via dropped" with
pcb_data.vias=[(1.0,1.0), (5.0,5.0)] -- the shipped barrel absent and the stale
one still blocking -- while "input-file original kept" and the segment contract
still PASS, which is the point: the segment half was never broken.

Run:  python3 tests/test_874_sync_vias.py [-v]
"""

import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from kicad_parser import BoardInfo, PCBData, Segment, Via  # noqa: E402
from routing_common import sync_pcb_data_segments  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name
          + (f"  [{detail}]" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


def _via(x, y, net_id):
    return Via(x=x, y=y, size=0.45, drill=0.2,
               layers=['F.Cu', 'B.Cu'], net_id=net_id)


def _seg(x1, y1, x2, y2, net_id):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=0.1, layer='F.Cu', net_id=net_id)


def _board(segments, vias):
    return PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                             copper_layers=['F.Cu', 'B.Cu']),
        nets={}, footprints={},
        vias=list(vias), segments=list(segments), pads_by_net={})


# ---------------------------------------------------------------------------
# 1. The meander-rebuild shape, end to end through the real function.
# ---------------------------------------------------------------------------
def test_meander_rebuild(verbose):
    print("\n[1] a rebuilt result via list reaches pcb_data, and the stale one leaves")
    P = 7
    orig_via = _via(1.0, 1.0, P)          # input-file barrel: must survive
    orig_seg = _seg(0.0, 0.0, 1.0, 1.0, P)
    committed_via = _via(5.0, 5.0, P)     # what add_route_to_pcb_data committed
    committed_seg = _seg(1.0, 1.0, 5.0, 5.0, P)

    pcb = _board([orig_seg, committed_seg], [orig_via, committed_via])
    original_segment_ids = {id(orig_seg)}
    original_via_ids = {id(orig_via)}

    # apply_meanders_to_diff_pair rebuilds BOTH lists with fresh objects.
    meandered_seg = _seg(1.0, 1.0, 5.2, 5.2, P)
    meandered_via = _via(5.2, 5.2, P)
    results = {P: {'new_segments': [meandered_seg], 'new_vias': [meandered_via],
                   'is_diff_pair': True}}

    sync_pcb_data_segments(pcb, results, original_segment_ids,
                           original_via_ids=original_via_ids)

    ids = {id(v) for v in pcb.vias}
    check("meandered via on board", id(meandered_via) in ids,
          f"pcb_data.vias={[(v.x, v.y) for v in pcb.vias]}")
    check("superseded via dropped", id(committed_via) not in ids,
          f"stale barrel at (5.0,5.0) still blocking; "
          f"pcb_data.vias={[(v.x, v.y) for v in pcb.vias]}")
    check("input-file original kept", id(orig_via) in ids)
    # the segment contract must be unchanged by the via work
    sids = {id(s) for s in pcb.segments}
    check("segment contract intact",
          id(meandered_seg) in sids and id(orig_seg) in sids
          and id(committed_seg) not in sids)
    if verbose:
        print(f"      vias now: {[(v.x, v.y) for v in pcb.vias]}")


# ---------------------------------------------------------------------------
# 2. The keep-alives: what the removal half must NOT drop.
# ---------------------------------------------------------------------------
def test_keepalives(verbose):
    print("\n[2] removal keeps originals, keeps any result's vias, and is opt-in")
    P, N = 7, 8
    # A pair result carries BOTH members' barrels; N's own result key is a
    # different net, so a per-net removal would strand this one.
    partner_via = _via(9.0, 9.0, N)
    orig_via = _via(1.0, 1.0, P)
    pcb = _board([], [orig_via, partner_via])
    results = {P: {'new_segments': [], 'new_vias': [partner_via]},
               N: {'new_segments': [], 'new_vias': []}}
    sync_pcb_data_segments(pcb, results, set(),
                           original_via_ids={id(orig_via)})
    ids = {id(v) for v in pcb.vias}
    check("partner via carried by another result is kept", id(partner_via) in ids)
    check("original kept alongside it", id(orig_via) in ids)

    # Without a keep-alive the via half must not run at all.
    stale = _via(5.0, 5.0, P)
    pcb2 = _board([], [stale])
    before = list(pcb2.vias)
    sync_pcb_data_segments(pcb2, {P: {'new_segments': [], 'new_vias': []}}, set())
    check("no keep-alive given -> vias untouched",
          [id(v) for v in pcb2.vias] == [id(v) for v in before],
          f"{len(before)} -> {len(pcb2.vias)}")


# ---------------------------------------------------------------------------
# 3. Wiring: an engine fix no caller reaches is inert.
# ---------------------------------------------------------------------------
def test_callers_wired(verbose):
    print("\n[3] route.py and route_diff.py both pass a via keep-alive")
    for name in ('route.py', 'route_diff.py'):
        path = os.path.join(ROOT, 'py_router', name)
        src = open(path, encoding='utf-8').read()
        snap = re.search(r'_original_vias_keepalive\s*=\s*list\(pcb_data\.vias\)', src)
        ids_ = re.search(r'original_via_ids\s*=\s*set\(id\(v\) for v in '
                         r'_original_vias_keepalive\)', src)
        call = re.search(r'sync_pcb_data_segments\([^)]*original_via_ids\s*=',
                         src, re.S)
        check(f"{name}: snapshots pcb_data.vias", bool(snap))
        check(f"{name}: builds original_via_ids from the keep-alive", bool(ids_))
        check(f"{name}: passes it to sync_pcb_data_segments", bool(call))
        if snap and ids_:
            # the snapshot must be taken with the segment one, i.e. BEFORE
            # routing commits anything -- a later snapshot would call routed
            # copper "original" and never drop a superseded barrel.
            seg_snap = re.search(r'_original_segments_keepalive\s*=\s*'
                                 r'list\(pcb_data\.segments\)', src)
            check(f"{name}: taken next to the segment snapshot",
                  bool(seg_snap) and abs(snap.start() - seg_snap.start()) < 600,
                  f"segments@{seg_snap.start() if seg_snap else '?'} "
                  f"vias@{snap.start()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()
    test_meander_rebuild(args.verbose)
    test_keepalives(args.verbose)
    test_callers_wired(args.verbose)
    print()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s): " + "; ".join(FAILS))
        return 1
    print("PASS: #874 sync reconciles vias (meander rebuild, keep-alives, wiring)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
