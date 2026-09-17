#!/usr/bin/env python3
"""#958: the octolinear smoother prefers FEWER SEGMENTS at equal length.

`smooth_octolinear_chains` accepted an octolinear connector only when it saved
at least `min_gain` of copper. A grid jog that is already a shortest octolinear
path (diag / axis / diag, or axis / diag / axis) therefore survived: moving its
corner removes a segment without changing the length, so the length-only gate
refused the simpler path `_octolinear_bends` had already proposed.

What must hold (every "collapses" case below FAILS at c38b3f5f, before the fix):
  * an equal-length diagonal/axis jog, an equal-length horizontal step and a
    longer equal-length staircase collapse to two legs at the SAME length
    (1e-9 mm), octolinear, endpoints preserved, saved_mm reported as 0;
  * a detour around a round pad whose prefix is a shortest path collapses to
    two corners that still clear the pad, in all four reflections and both
    chain directions -- AND keeps the copper the strict rule saves on its
    tail. The tie-break runs as a SECOND greedy phase over the strict
    phase's result: taken in the same farthest-first pass it would commit
    the equal-length prefix and pre-empt the strictly shorter tail span
    (3 legs at 10.24 mm instead of 3 legs at 9.66 mm);
  * a two-leg elbow is left alone (equal length, equal count), and the pass is
    idempotent on its own output;
  * a connector that is longer by a MEASURABLE amount is refused even with
    fewer legs and even when the growth is under min_gain: min_gain is a
    saving floor, never a growth allowance;
  * foreign copper on BOTH alternatives blocks the collapse; on ONE of them
    the pass takes the other, and the copper it emits keeps clearance;
  * a same-net via or pad touching the chain mid-span pins its span (only the
    touch-free remainder collapses) and keeps its copper contact;
  * skip_net_ids (the protected-net channel) is absolute; dry_run measures
    without mutating; write-list custody holds for routed segments.

    python3 tests/test_958_smoother_equal_length.py
"""
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

from kicad_parser import Pad, Net, PCBData, BoardInfo, Segment, Via
from pcb_modification import smooth_octolinear_chains

fails = []
W = 0.15
CLR = 0.1
SQ2 = math.sqrt(2.0)


def check(name, cond, detail=""):
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


def _pad(ref, x, y, net_id, net_name, sx=0.5, sy=0.5, shape='rect', layer='F.Cu'):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape=shape,
               layers=[layer], net_id=net_id, net_name=net_name, drill=0)


def _board(pads_by_net, segments, vias=None):
    return PCBData(footprints={},
                   nets={1: Net(1, '/SIG'), 2: Net(2, '/OTHER')},
                   segments=segments, vias=vias or [],
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(0.0, 0.0, 200.0, 200.0)),
                   pads_by_net=pads_by_net)


def polyline(points, w=W, layer='F.Cu', net_id=1):
    return [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                    width=w, layer=layer, net_id=net_id)
            for a, b in zip(points, points[1:])]


def total_len(segs):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


def octolinear(s):
    dx, dy = abs(s.end_x - s.start_x), abs(s.end_y - s.start_y)
    return dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6


def vertices(segs):
    pts = set()
    for s in segs:
        pts.add((round(s.start_x, 4), round(s.start_y, 4)))
        pts.add((round(s.end_x, 4), round(s.end_y, 4)))
    return pts


def end_pads(points):
    (ax, ay), (bx, by) = points[0], points[-1]
    return {1: [_pad('U1', ax, ay, 1, '/SIG'), _pad('U2', bx, by, 1, '/SIG')]}


def foreign_pad_ok(pcb):
    """Every net-1 segment keeps clearance from every foreign pad."""
    from single_ended_routing import _seg_foreign_pad_dist
    for s in pcb.segments:
        if s.net_id != 1:
            continue
        d = _seg_foreign_pad_dist(pcb, 1, s.start_x, s.start_y, s.end_x, s.end_y,
                                  s.layer, base_clearance=CLR)
        if d < CLR + s.width / 2 - 1e-4:
            return False
    return True


# Shortest octolinear paths carrying a needless corner. Each is exactly
# |dx - dy| + min(|dx|, |dy|) * sqrt(2) long, so no connector can be SHORTER.
JOG_DIAG_AXIS = [(10.0, 10.0), (11.0, 11.0), (11.0, 12.0), (12.0, 13.0)]    # D V D
JOG_HORIZONTAL = [(10.0, 10.0), (11.0, 10.0), (12.0, 11.0), (13.0, 11.0)]   # H D H
STAIR6 = [(10.0, 10.0), (11.0, 11.0), (11.0, 12.0), (12.0, 13.0),
          (12.0, 14.0), (13.0, 15.0), (13.0, 16.0)]                       # D V D V D V


def run_equal_length(name, points, expect_segs=2):
    pcb = _board(end_pads(points), polyline(points))
    before = total_len(pcb.segments)
    n_in = len(pcb.segments)
    _n, nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
    after = total_len(pcb.segments)
    check(f"{name}: collapses to {expect_segs} legs",
          len(pcb.segments) == expect_segs and nets == 1 and st['spans'] == 1,
          f"{n_in} -> {len(pcb.segments)} segs, {st}")
    check(f"{name}: length unchanged", abs(after - before) <= 1e-9,
          f"{before:.12f} -> {after:.12f}")
    check(f"{name}: saved_mm is 0, never negative", st['saved_mm'] == 0.0, st)
    check(f"{name}: output octolinear", all(octolinear(s) for s in pcb.segments))
    v = vertices(pcb.segments)
    check(f"{name}: endpoints preserved", points[0] in v and points[-1] in v, v)
    check(f"{name}: strip carries the input removals and the count matches",
          len(strip) == n_in and st['segs_removed'] == n_in
          and len(added) == expect_segs and st['segs_added'] == expect_segs,
          (len(strip), st))
    return pcb


def test_equal_length_jogs():
    run_equal_length("diag/axis jog", JOG_DIAG_AXIS)
    run_equal_length("horizontal step", JOG_HORIZONTAL)
    run_equal_length("six-step staircase", STAIR6)


def test_elbow_stable_and_idempotent():
    elbow = [(10.0, 10.0), (12.0, 12.0), (13.0, 12.0)]
    pcb = _board(end_pads(elbow), polyline(elbow))
    ids = [id(s) for s in pcb.segments]
    _n, nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
    check("two-leg elbow is left alone (equal length, equal count)",
          nets == 0 and st['spans'] == 0 and not strip and not added
          and [id(s) for s in pcb.segments] == ids, st)
    # A second pass over a collapsed jog finds nothing more to do.
    pcb2 = run_equal_length("idempotence setup", JOG_DIAG_AXIS)
    ids2 = [id(s) for s in pcb2.segments]
    _n, nets2, strip2, added2, st2 = smooth_octolinear_chains([], pcb2, clearance=CLR)
    check("pass is idempotent on its own output",
          nets2 == 0 and st2['spans'] == 0 and not strip2 and not added2
          and [id(s) for s in pcb2.segments] == ids2, st2)


def test_refuses_measurable_growth():
    # Three near-octolinear legs from (10,10) to (13,12) that are SHORTER than
    # the octolinear connector (1 + 2*sqrt2) by less than min_gain: the
    # connector has fewer legs, but taking it would ADD copper.
    d = 0.02
    pts = [(10.0, 10.0), (12.0, 12.0 - d), (12.0 + d, 12.0), (13.0, 12.0)]
    pcb = _board(end_pads(pts), polyline(pts))
    growth = (1.0 + 2 * SQ2) - total_len(pcb.segments)
    check("premise: connector grows the copper by 1e-6 < g < min_gain",
          1e-6 < growth < 0.01, f"growth={growth:.6f}")
    ids = [id(s) for s in pcb.segments]
    _n, nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
    check("longer connector refused despite fewer legs",
          nets == 0 and st['spans'] == 0 and not strip and not added
          and [id(s) for s in pcb.segments] == ids, st)


def test_blocked_alternatives():
    # The jog's two connectors bend at (12,12) [diag-then-vertical, tried
    # first] and (10,11) [vertical-then-diag]. Neither point is near the
    # ORIGINAL copper (0.7 mm off the diagonals, 1 mm off the riser).
    pads = end_pads(JOG_DIAG_AXIS)
    pads[2] = [_pad('C1', 12.0, 12.0, 2, '/OTHER', sx=0.3, sy=0.3),
               _pad('C2', 10.0, 11.0, 2, '/OTHER', sx=0.3, sy=0.3)]
    pcb = _board(pads, polyline(JOG_DIAG_AXIS))
    check("premise: the original jog clears both blockers", foreign_pad_ok(pcb))
    ids = [id(s) for s in pcb.segments]
    _n, nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
    check("both alternatives blocked: jog kept",
          nets == 0 and st['spans'] == 0 and [id(s) for s in pcb.segments] == ids, st)
    # Block only the first-tried bend: the pass falls through to the other.
    pads = end_pads(JOG_DIAG_AXIS)
    pads[2] = [_pad('C1', 12.0, 12.0, 2, '/OTHER', sx=0.3, sy=0.3)]
    pcb = _board(pads, polyline(JOG_DIAG_AXIS))
    _n, nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
    check("one alternative blocked: the other is taken",
          nets == 1 and len(pcb.segments) == 2 and (10.0, 11.0) in vertices(pcb.segments),
          (st, sorted(vertices(pcb.segments))))
    check("taken alternative keeps clearance", foreign_pad_ok(pcb))
    check("taken alternative is octolinear", all(octolinear(s) for s in pcb.segments))


def _pinned_mid_span(name, pcb, touch_xy):
    """STAIR6 with same-net copper touching its second leg (the (11,11)-(11,12)
    riser) mid-span: legs 0-1 hold, legs 2-5 collapse to two."""
    before = total_len(pcb.segments)
    _n, nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
    check(f"{name}: pinned span held, free remainder collapsed",
          nets == 1 and st['spans'] == 1 and len(pcb.segments) == 4,
          f"{len(pcb.segments)} segs, {st}")
    check(f"{name}: length unchanged", abs(total_len(pcb.segments) - before) <= 1e-9)
    tx, ty = touch_xy
    riser = [s for s in pcb.segments
             if (round(s.start_x, 4), round(s.start_y, 4)) == (11.0, 11.0)
             and (round(s.end_x, 4), round(s.end_y, 4)) == (11.0, 12.0)]
    check(f"{name}: the touched riser still exists, contact kept", len(riser) == 1
          and min(abs(tx - 11.0), 1.0) < W / 2 and 11.0 <= ty <= 12.0)
    check(f"{name}: output octolinear", all(octolinear(s) for s in pcb.segments))


def test_via_mid_span_pins():
    via = Via(x=11.0, y=11.5, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    pcb = _board(end_pads(STAIR6), polyline(STAIR6), vias=[via])
    _pinned_mid_span("via tap", pcb, (11.0, 11.5))


def test_pad_mid_span_pins():
    pads = end_pads(STAIR6)
    pads[1].append(_pad('R7', 11.0, 11.5, 1, '/SIG', sx=0.3, sy=0.3))
    pcb = _board(pads, polyline(STAIR6))
    _pinned_mid_span("pad touch", pcb, (11.0, 11.5))


def test_round_pad_detour_all_poses():
    # A shortest-path detour (V D V D V D H) around a round foreign pad that
    # sits on the direct diagonal. The old rule left all 7 legs; the tie-break
    # gives two corners, and the diag-then-vertical bend at the pad centre is
    # refused so the vertical-then-diag one is taken.
    base = [(10.0, 10.0), (10.0, 11.0), (11.0, 12.0), (11.0, 13.0), (12.0, 14.0),
            (12.0, 15.0), (13.0, 16.0), (16.0, 16.0)]
    blocker = (13.0, 13.0)

    def pose(mx, my, rev):
        f = lambda p: ((30.0 - p[0]) if mx else p[0], (30.0 - p[1]) if my else p[1])
        pts = [f(p) for p in base]
        if rev:
            pts = pts[::-1]
        return pts, f(blocker)

    for mx in (False, True):
        for my in (False, True):
            for rev in (False, True):
                pts, (bx, by) = pose(mx, my, rev)
                name = f"detour mx={int(mx)} my={int(my)} rev={int(rev)}"
                pads = end_pads(pts)
                pads[2] = [_pad('C5', bx, by, 2, '/OTHER', sx=2.0, sy=2.0, shape='circle')]
                pcb = _board(pads, polyline(pts))
                check(f"{name}: premise, original clears the pad", foreign_pad_ok(pcb))
                before = total_len(pcb.segments)
                _n, nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
                check(f"{name}: 7 legs -> 3",
                      nets == 1 and len(pcb.segments) == 3,
                      f"{len(pcb.segments)} segs, {st}")
                # The strict phase alone gives 5 legs at 4 + 4*sqrt2 (it
                # shortens the tail); the tie-break then folds the prefix at
                # that SAME length. A tie-break riding in the strict pass
                # would commit the equal-length prefix first and forfeit the
                # tail's saving: 3 legs at 6 + 3*sqrt2 (= the input length).
                check(f"{name}: keeps the strict phase's saving",
                      abs(total_len(pcb.segments) - (4.0 + 4.0 * SQ2)) <= 1e-9
                      and abs(before - (6.0 + 3.0 * SQ2)) <= 1e-9,
                      f"{before:.4f} -> {total_len(pcb.segments):.4f}")
                check(f"{name}: clears the pad", foreign_pad_ok(pcb))
                check(f"{name}: octolinear", all(octolinear(s) for s in pcb.segments))
                v = vertices(pcb.segments)
                check(f"{name}: endpoints preserved", pts[0] in v and pts[-1] in v)


def test_skip_net_ids_absolute():
    pcb = _board(end_pads(JOG_DIAG_AXIS), polyline(JOG_DIAG_AXIS))
    _n, nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=CLR,
                                                          skip_net_ids={1})
    check("skip_net_ids wins over the tie-break",
          nets == 0 and not strip and not added and st['spans'] == 0
          and len(pcb.segments) == 3, st)


def test_dry_run_measures_without_mutating():
    pcb = _board(end_pads(JOG_DIAG_AXIS), polyline(JOG_DIAG_AXIS))
    ids = [id(s) for s in pcb.segments]
    _n, _nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=CLR,
                                                           dry_run=True)
    check("dry run leaves the board alone",
          [id(s) for s in pcb.segments] == ids and not strip and not added)
    check("dry run still measures the tie-break",
          st['spans'] == 1 and st['segs_removed'] == 3 and st['segs_added'] == 2
          and st['saved_mm'] == 0.0, st)


def test_writelist_custody():
    segs = polyline(JOG_DIAG_AXIS)
    res = {'new_segments': list(segs), 'new_vias': []}
    pcb = _board(end_pads(JOG_DIAG_AXIS), list(segs))
    _n, nets, strip, added, st = smooth_octolinear_chains([res], pcb, clearance=CLR)
    check("routed removals leave the write-list, not the strip",
          nets == 1 and not strip and st['segs_removed'] == 3, (len(strip), st))
    check("added legs join the owning result",
          len(added) == 2 and all(s in res['new_segments'] for s in added))
    check("removed routed legs dropped from new_segments",
          not [s for s in segs if s in res['new_segments']]
          and len(res['new_segments']) == 2)


if __name__ == '__main__':
    for fn in (test_equal_length_jogs, test_elbow_stable_and_idempotent,
               test_refuses_measurable_growth, test_blocked_alternatives,
               test_via_mid_span_pins, test_pad_mid_span_pins,
               test_round_pad_detour_all_poses, test_skip_net_ids_absolute,
               test_dry_run_measures_without_mutating, test_writelist_custody):
        print(fn.__name__)
        fn()
    if fails:
        print(f"\nFAILED ({len(fails)}): {fails}")
        sys.exit(1)
    print("\nALL PASS")
