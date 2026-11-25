#!/usr/bin/env python3
"""45-degree corners: chamfer_corners cuts 90-degree track corners.

With chamfer_corners (the cleanup pipeline's last copper step; route.py and
repair_planes.py run it with --chamfer-corners, off by default), each 90-degree
corner of a routed net gets a
45-degree leg, under the shortcut clearance test and the keep-away band. A
corner on a pad, or with other same-net copper on the cut-off pieces, is left.
Without it, nothing changes. Acute corners are left alone.

    python3 tests/test_chamfer_corners.py
"""
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

from kicad_parser import Pad, Net, PCBData, BoardInfo, Segment, Via  # noqa: E402
from pcb_modification import smooth_octolinear_chains  # noqa: E402

fails = []
W = 0.15
CLR = 0.1


def check(name, cond, detail=""):
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)
        if 'pytest' in sys.modules:       # pytest: fail the test case here
            raise AssertionError(f"{name}  {detail}")


def _pad(ref, x, y, net_id, name, sx=0.5, sy=0.5):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape='rect',
               layers=['F.Cu'], net_id=net_id, net_name=name, drill=0)


def board(points, extra_pads=(), vias=()):
    pads = {1: [_pad('A', *points[0], 1, '/SIG'), _pad('B', *points[-1], 1, '/SIG')]}
    for p in extra_pads:
        pads.setdefault(p.net_id, []).append(p)
    segs = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                    width=W, layer='F.Cu', net_id=1) for a, b in zip(points, points[1:])]
    return PCBData(footprints={}, nets={1: Net(1, '/SIG'), 2: Net(2, '/OTHER')},
                   segments=segs, vias=list(vias),
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(0.0, 0.0, 200.0, 200.0)),
                   pads_by_net=pads)


def octolinear(s):
    dx, dy = abs(s.end_x - s.start_x), abs(s.end_y - s.start_y)
    return dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6


def has_corner(segs):
    for a in segs:
        for b in segs:
            if a is b:
                continue
            for pa, qa in (((a.start_x, a.start_y), (a.end_x, a.end_y)),
                           ((a.end_x, a.end_y), (a.start_x, a.start_y))):
                for pb, qb in (((b.start_x, b.start_y), (b.end_x, b.end_y)),
                               ((b.end_x, b.end_y), (b.start_x, b.start_y))):
                    if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < 1e-6:
                        ua = (qa[0] - pa[0], qa[1] - pa[1])
                        ub = (qb[0] - pb[0], qb[1] - pb[1])
                        if abs(ua[0] * ub[0] + ua[1] * ub[1]) < 1e-6:
                            return True
    return False


def length(segs):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


ELBOW = [(10.0, 10.0), (14.0, 10.0), (14.0, 14.0)]
# foreign copper in the middle of the elbow: the smoothing shortcut (the 45-degree
# diagonal from end to end) cannot clear it, so the 90-degree corner survives
# smoothing -- which is where a routed board keeps its corners
BLOCK = _pad('Y', 12.0, 12.0, 2, '/OTHER', 0.5, 0.5)


def test_elbow_chamfered():
    pcb = board(ELBOW, extra_pads=[BLOCK])
    before = length(pcb.segments)
    _n, nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR,
                                                             chamfer_corners=True)
    check("90-degree elbow: chamfered", st.get('corners_chamfered') == 1 and nets == 1, st)
    check("... three legs, none at 90 degrees",
          len(pcb.segments) == 3 and not has_corner(pcb.segments),
          [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments])
    check("... octolinear and shorter", all(octolinear(s) for s in pcb.segments)
          and length(pcb.segments) < before - 0.1)
    ends = {(round(s.start_x, 6), round(s.start_y, 6)) for s in pcb.segments} | \
           {(round(s.end_x, 6), round(s.end_y, 6)) for s in pcb.segments}
    check("... endpoints kept", ELBOW[0] in ends and ELBOW[-1] in ends)


def test_off_by_default_in_the_pass():
    pcb = board(ELBOW, extra_pads=[BLOCK])
    ids = [id(s) for s in pcb.segments]
    _n, nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR)
    check("without chamfer_corners the elbow stays",
          not st.get('corners_chamfered') and [id(s) for s in pcb.segments] == ids, st)


def test_foreign_copper_blocks():
    # a foreign pad sitting inside the corner, across every cut the pass may try
    pcb = board(ELBOW, extra_pads=[BLOCK, _pad('X', 13.8, 10.2, 2, '/OTHER', 0.2, 0.2)])
    ids = [id(s) for s in pcb.segments]
    _n, _nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR,
                                                              chamfer_corners=True)
    check("foreign pad inside the corner: left as it is",
          not st.get('corners_chamfered') and [id(s) for s in pcb.segments] == ids, st)


def test_same_net_via_at_corner_pins_it():
    pcb = board(ELBOW, extra_pads=[BLOCK], vias=[Via(x=14.0, y=10.0, size=0.6, drill=0.3,
                                 layers=['F.Cu', 'B.Cu'], net_id=1)])
    ids = [id(s) for s in pcb.segments]
    _n, _nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR,
                                                              chamfer_corners=True)
    check("same-net via on the corner: left as it is",
          not st.get('corners_chamfered') and [id(s) for s in pcb.segments] == ids, st)


def test_acute_corner_left_alone():
    pts = [(10.0, 10.0), (14.0, 10.0), (12.0, 12.0)]
    pcb = board(pts, extra_pads=[_pad('Y', 11.0, 11.0, 2, '/OTHER', 0.5, 0.5)])
    _n, _nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR,
                                                              chamfer_corners=True)
    check("acute corner: not chamfered", not st.get('corners_chamfered'), st)


def corners(pcb, config=None):
    return smooth_octolinear_chains([], pcb, clearance=CLR, config=config,
                                    chamfer_corners=True)[4].get('corners_chamfered', 0)


def test_corner_on_a_pad_left():
    # the corner vertex lies on a same-net pad: the pad copper fills the corner,
    # and a cut would leave the track joined to the pad by a sliver
    pad = _pad('C', 14.0, 10.0, 1, '/SIG', 0.4, 0.4)
    pcb = board(ELBOW, extra_pads=[BLOCK, pad])
    ids = [id(s) for s in pcb.segments]
    n = corners(pcb)
    check("corner on a same-net pad: left as it is",
          n == 0 and [id(s) for s in pcb.segments] == ids, n)


def test_stem_on_a_cut_piece_blocks():
    # a same-net stem ends on leg 1, 0.2 mm before the corner; its pad is also
    # reached another way (a loop back to A), so connectivity alone would allow
    # the cut. The loop's own corners may be cut; the elbow's may not.
    pcb = board(ELBOW, extra_pads=[BLOCK])
    elbow = list(pcb.segments)
    pcb.segments += [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=W,
                             layer='F.Cu', net_id=1)
                     for a, b in (((13.8, 10.0), (13.8, 8.0)), ((13.8, 8.0), (10.0, 8.0)),
                                  ((10.0, 8.0), (10.0, 10.0)))]
    corners(pcb)
    check("same-net stem on a cut-off piece: the elbow is left",
          all(s in pcb.segments for s in elbow))


def test_chain_continuing_past_a_short_leg():
    # leg 2 is 0.25 mm and the chain goes on from its far end: that end stays
    # on the new path, so it does not block the cut (a cut piece ends within a
    # track width of it)
    pts = [(10.0, 10.0), (14.0, 10.0), (14.0, 10.25), (18.0, 10.25)]
    pcb = board(pts, extra_pads=[BLOCK])
    corners(pcb)
    check("chain going on past a short leg: the corner is cut",
          not any(abs(x - 14.0) < 1e-9 and abs(y - 10.0) < 1e-9 for s in pcb.segments
                  for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))),
          [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments])


def test_via_on_a_cut_piece_blocks():
    via = Via(x=14.0, y=10.2, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    pcb = board(ELBOW, extra_pads=[BLOCK], vias=[via])
    n = corners(pcb)
    check("same-net via on a cut-off piece: left as it is", n == 0, n)


def test_keep_away_band():
    # a /ANA pad inside the corner with rule /SIG:/ANA:0.3: the full 0.5 mm cut
    # would move the leg into its band, the half cut stays out of it
    import keep_away as ka
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=W, clearance=CLR,
                          keep_away=ka.normalize_keep_away_specs(['/SIG:/ANA:0.3']),
                          keep_away_free=1.0, keep_away_cost=0.5)

    def in_band(pcb):
        return ka.keep_away_report(pcb, cfg)['nets'].get('/SIG', {}).get('in_band_mm', 0.0)

    def diagonal(pcb):
        return [math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in pcb.segments
                if s.net_id == 1 and abs(s.end_x - s.start_x) > 1e-6
                and abs(s.end_y - s.start_y) > 1e-6]

    def make():
        pcb = board(ELBOW, extra_pads=[BLOCK, _pad('Z', 13.4, 10.6, 3, '/ANA', 0.3, 0.3)])
        pcb.nets[3] = Net(3, '/ANA')
        return pcb

    pcb = make()
    before = in_band(pcb)
    n = corners(pcb, cfg)
    leg = diagonal(pcb)
    check("keep-away band: cut with the half leg",
          n == 1 and len(leg) == 1 and abs(leg[0] - 0.25 * math.sqrt(2)) < 1e-6, (n, leg))
    check("... no track added inside the band", in_band(pcb) <= before + 1e-9,
          (before, in_band(pcb)))
    pcb = make()
    corners(pcb)
    check("... without the rule the full leg", [round(x, 6) for x in diagonal(pcb)]
          == [round(0.5 * math.sqrt(2), 6)], diagonal(pcb))


def test_corner_pass_smooths_nothing():
    # a staircase the smoother would shorten: the corner pass leaves it
    pts = [(10.0, 10.0), (11.0, 10.0), (11.0, 11.0), (12.0, 11.0), (12.0, 12.0), (20.0, 12.0)]
    pcb = board(pts)
    _n, _nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=CLR,
                                                              chamfer_corners=True)
    check("corner pass: no smoothing span", st.get('spans', 0) == 0, st)


def test_plane_finalize_cleanup():
    # the plane finalize runs the cleanup pipeline without smoothing and hands
    # chamfer_corners on from its route knobs: its taps get the corner pass too
    from types import SimpleNamespace
    from cleanup_pipeline import run_post_route_cleanup
    from route import _FINALIZE_ROUTE_KNOBS
    check("chamfer_corners is a route knob of the plane finalize",
          'chamfer_corners' in _FINALIZE_ROUTE_KNOBS)
    from repair_planes import plane_chamfer_corners
    check("repair_planes: off by default, standalone and in-run",
          not plane_chamfer_corners(None) and not plane_chamfer_corners({'smoothing': True}))
    check("repair_planes: on with --chamfer-corners, off again with --no-smoothing",
          plane_chamfer_corners({'chamfer_corners': True})
          and not plane_chamfer_corners({'chamfer_corners': True, 'smoothing': False}))
    for flag in (True, False):
        pcb = board(ELBOW, extra_pads=[BLOCK])
        out = run_post_route_cleanup(
            [], pcb, {1}, SimpleNamespace(clearance=CLR, grid_step=0.05, board_edge_clearance=0.0),
            label='Plane ', phantom=False, via_nudge=False, neck=False,
            microshift_max_shift=0.05, chamfer_corners=flag)
        n = out.counts.get('corners_chamfered', 0)
        check(f"plane cleanup, chamfer_corners={flag}: {'cut' if flag else 'kept'}",
              n == (1 if flag else 0) and has_corner(pcb.segments) != flag, n)


def _stats(pcb):
    return smooth_octolinear_chains([], pcb, clearance=CLR, chamfer_corners=True)[4]


def test_width_change_cut_at_the_narrower_width():
    # a 0.6 mm track necks down to 0.15 mm at a 90-degree corner: the new leg
    # takes the narrower width, each kept piece keeps its own
    pcb = board(ELBOW, extra_pads=[BLOCK])
    pcb.segments[0].width = 0.6
    st = _stats(pcb)
    check("neck-down elbow: chamfered", st.get('corners_chamfered') == 1, st)
    diag = [s for s in pcb.segments if abs(s.end_x - s.start_x) > 1e-6
            and abs(s.end_y - s.start_y) > 1e-6]
    check("... the 45-degree leg at the narrower width",
          len(diag) == 1 and abs(diag[0].width - 0.15) < 1e-9, [(s.width) for s in diag])
    check("... the kept pieces keep their widths",
          sorted(round(s.width, 3) for s in pcb.segments if s not in diag) == [0.15, 0.6],
          [(s.start_x, s.start_y, s.end_x, s.end_y, s.width) for s in pcb.segments])


def test_corners_left_counted_by_reason():
    via = Via(x=14.0, y=10.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    st = _stats(board(ELBOW, extra_pads=[BLOCK], vias=[via]))
    check("a via at the corner: left, counted as 'via'",
          st.get('corners_left') == {'via': 1}, st)
    # a 0.12 mm leg too short to cut, ending on a small same-net pad (not a
    # plain joint), so it cannot be taken whole either
    pad_b = _pad('C', 14.0, 10.14, 1, '/SIG', 0.06, 0.06)
    st = _stats(board([(10.0, 10.0), (14.0, 10.0), (14.0, 10.14), (18.0, 10.14)],
                      extra_pads=[BLOCK, pad_b]))
    check("a short leg ending on a pad: left, counted as 'short leg'",
          st.get('corners_left', {}).get('short leg', 0) >= 1, st)
    st = _stats(board(ELBOW, extra_pads=[BLOCK, _pad('X', 13.8, 10.2, 2, '/OTHER', 0.2, 0.2)]))
    check("foreign copper inside the corner: left, counted as 'blocked'",
          st.get('corners_left') == {'blocked': 1}, st)
    st = _stats(board(ELBOW, extra_pads=[BLOCK]))
    check("a corner that is cut is not counted as left",
          not st.get('corners_left'), st)


def _no_right_angle(segs):
    return not has_corner(segs)


def test_short_leg_taken_whole():
    # a 0.12 mm jog between two long runs at 0.15 mm: too short to cut from
    # both ends, but its far end is a plain joint whose turn stays at 45
    # degrees, so the cut takes the leg whole: both 90-degree corners go
    jog = [(10.0, 10.0), (14.0, 10.0), (14.0, 10.12), (18.0, 10.12)]
    pcb = board(jog, extra_pads=[BLOCK])
    st = _stats(pcb)
    check("a jog: the short leg taken whole, no 90-degree corner left",
          st.get('corners_chamfered') == 1 and _no_right_angle(pcb.segments)
          and not st.get('corners_left'),
          (st, [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments]))
    check("... the new leg ends exactly at the joint (14, 10.12)",
          any(abs(s.end_x - 14.0) < 1e-9 and abs(s.end_y - 10.12) < 1e-9
              or abs(s.start_x - 14.0) < 1e-9 and abs(s.start_y - 10.12) < 1e-9
              for s in pcb.segments))


def test_short_leg_left_when_the_turn_at_its_end_would_not_stay():
    # the run after the short leg turns so that the diagonal would meet it at
    # 90 degrees (down-right) or double back (acute): left
    for name, nxt in (("90 degrees", (16.0, 8.12)), ("acute", (10.0, 10.12))):
        pcb = board([(10.0, 10.0), (14.0, 10.0), (14.0, 10.12), nxt], extra_pads=[BLOCK])
        st = _stats(pcb)
        check(f"turn at the short leg's end would be {name}: not taken whole",
              not st.get('corners_chamfered') and st.get('corners_left', {}).get('short leg'),
              st)


def test_degenerate_short_leg_left():
    # a leg below the degenerate length would be filtered out of the new
    # copper and leave a gap: not taken, counted as 'short leg'
    pcb = board([(10.0, 10.0), (14.0, 10.0), (14.0, 10.000005), (18.0, 10.000005)],
                extra_pads=[BLOCK])
    st = _stats(pcb)
    check("a degenerate short leg: not taken",
          not st.get('corners_chamfered'), st)


def test_corners_on_a_net_over_the_size_cap_counted():
    # a net with more tracks than max_net_segs is not looked at; its corners
    # are counted as left, not silently missing
    pcb = board(ELBOW, extra_pads=[BLOCK])
    st = smooth_octolinear_chains([], pcb, clearance=CLR, chamfer_corners=True,
                                  max_net_segs=1)[4]
    check("a net over the size cap: its corner counted as 'net over size cap'",
          st.get('corners_left') == {'net over size cap': 1}
          and st.get('nets_skipped_large') == 1, st)


def test_cli_plane_cleanup():
    # the CLI plane finalize's cleanup leg (clean_plane_copper ->
    # compute_plane_copper_cleanup) cuts the corner with the flag, keeps it without
    from pcb_modification import compute_plane_copper_cleanup
    for flag in (True, False):
        pcb = board(ELBOW, extra_pads=[BLOCK])
        delta = compute_plane_copper_cleanup(pcb, ['/SIG'], CLR, 0.05, 0.0,
                                             chamfer_corners=flag)
        diag = [c for c in delta.connectors if abs(c.end_x - c.start_x) > 1e-6
                and abs(c.end_y - c.start_y) > 1e-6]
        check(f"CLI plane cleanup, chamfer_corners={flag}: {'cut' if flag else 'kept'}",
              bool(diag) == flag and bool(delta.segments_to_remove) == flag,
              ([(c.start_x, c.start_y, c.end_x, c.end_y) for c in delta.connectors],
               len(delta.segments_to_remove)))


def test_default_is_off():
    # opt-in: route.py's batch_route and the cleanup pipeline leave corners
    # alone unless asked, so a default run is upstream's
    import inspect
    from route import batch_route
    from cleanup_pipeline import run_post_route_cleanup
    check("batch_route: chamfer_corners defaults to False",
          inspect.signature(batch_route).parameters['chamfer_corners'].default is False)
    check("run_post_route_cleanup: chamfer_corners defaults to False",
          inspect.signature(run_post_route_cleanup).parameters['chamfer_corners'].default is False)


TESTS = [test_elbow_chamfered, test_off_by_default_in_the_pass, test_foreign_copper_blocks,
         test_same_net_via_at_corner_pins_it, test_acute_corner_left_alone,
         test_corner_on_a_pad_left, test_stem_on_a_cut_piece_blocks,
         test_chain_continuing_past_a_short_leg,
         test_via_on_a_cut_piece_blocks, test_keep_away_band, test_corner_pass_smooths_nothing,
         test_plane_finalize_cleanup, test_width_change_cut_at_the_narrower_width,
         test_corners_left_counted_by_reason, test_short_leg_taken_whole,
         test_short_leg_left_when_the_turn_at_its_end_would_not_stay,
         test_degenerate_short_leg_left, test_corners_on_a_net_over_the_size_cap_counted,
         test_cli_plane_cleanup, test_default_is_off]

if __name__ == '__main__':
    for t in TESTS:
        print(t.__name__)
        t()
    print("ALL CHECKS PASSED" if not fails else f"FAILED: {len(fails)} check(s): {fails}")
    sys.exit(1 if fails else 0)
