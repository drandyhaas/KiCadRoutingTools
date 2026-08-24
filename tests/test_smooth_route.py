#!/usr/bin/env python3
"""#536 octolinear route smoothing: smooth_octolinear_chains + pipeline wiring.

What must hold:
  * a staircase of grid micro-jogs collapses to a diagonal+axis shortcut,
    STRICTLY octolinear output, endpoints and pad contacts preserved;
  * a foreign pad on the shortcut path blocks it (partial smoothing only, and
    every surviving segment keeps clearance);
  * dry_run measures without mutating;
  * a mid-span same-net via tap splits the chain and keeps its copper contact;
  * write-list custody: routed segments are dropped from their result's
    new_segments, input-file segments land on the strip list;
  * skip_net_ids is absolute; _smooth_skip_net_ids picks up protected_nets
    registrations (the diff-pair / length-match / impedance skip);
  * the pipeline runs the pass ONLY under KICAD_SMOOTH_ROUTE=1.

    python3 tests/test_smooth_route.py
"""
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

import env_knobs
from kicad_parser import Pad, Net, PCBData, BoardInfo, Segment, Via
from pcb_modification import smooth_octolinear_chains

fails = []


def check(name, cond, detail=""):
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


def _pad(ref, x, y, net_id, net_name, sx=0.5, sy=0.5, layer='F.Cu'):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape='rect',
               layers=[layer], net_id=net_id, net_name=net_name, drill=0)


def _board(pads_by_net, segments, vias=None):
    return PCBData(footprints={},
                   nets={1: Net(1, '/SIG'), 2: Net(2, '/OTHER')},
                   segments=segments, vias=vias or [],
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(0.0, 0.0, 200.0, 200.0)),
                   pads_by_net=pads_by_net)


def staircase(x0, y0, steps, pitch=0.2, w=0.15):
    """Alternating H/V micro-jogs from (x0, y0), heading +x+y."""
    segs = []
    x, y = x0, y0
    for i in range(steps):
        uid = f"seg-{i}"
        if i % 2 == 0:
            segs.append(Segment(start_x=x, start_y=y, end_x=x + pitch, end_y=y,
                                width=w, layer='F.Cu', net_id=1, uuid=uid))
            x += pitch
        else:
            segs.append(Segment(start_x=x, start_y=y, end_x=x, end_y=y + pitch,
                                width=w, layer='F.Cu', net_id=1, uuid=uid))
            y += pitch
    return segs, (x, y)


def total_len(segs):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


def octolinear(s):
    dx, dy = abs(s.end_x - s.start_x), abs(s.end_y - s.start_y)
    return dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6


def end_pads(ex, ey):
    return {1: [_pad('U1', 10.0, 10.0, 1, '/SIG'), _pad('U2', ex, ey, 1, '/SIG')]}


def test_staircase_collapses():
    segs, (ex, ey) = staircase(10.0, 10.0, 20)
    pcb = _board(end_pads(ex, ey), list(segs))
    before = total_len(pcb.segments)
    _n, nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=0.1)
    after = total_len(pcb.segments)
    check("staircase collapsed", nets == 1 and st['spans'] >= 1, st)
    check("copper got shorter", after < before - 0.5, f"{before:.3f}->{after:.3f}")
    check("output is octolinear", all(octolinear(s) for s in pcb.segments))
    check("input removals ride the strip list",
          len(strip) == st['segs_removed'] and st['segs_removed'] > 0,
          (len(strip), st['segs_removed']))
    pts = set()
    for s in pcb.segments:
        pts.add((round(s.start_x, 4), round(s.start_y, 4)))
        pts.add((round(s.end_x, 4), round(s.end_y, 4)))
    check("chain endpoints preserved",
          (10.0, 10.0) in pts and (round(ex, 4), round(ey, 4)) in pts)


def test_blocked_shortcut_keeps_clearance():
    # Coarse staircase so the input copper clears the blocker; only the direct
    # (10,10)->(14,14) diagonal passes through the foreign pad at (11.5, 11.5).
    segs, (ex, ey) = staircase(10.0, 10.0, 8, pitch=1.0)
    pads = end_pads(ex, ey)
    pads[2] = [_pad('C9', 11.5, 11.5, 2, '/OTHER', sx=0.3, sy=0.3)]
    pcb = _board(pads, list(segs))
    smooth_octolinear_chains([], pcb, clearance=0.1)
    from single_ended_routing import _seg_foreign_pad_dist
    ok = True
    for s in pcb.segments:
        if s.net_id != 1:
            continue
        d = _seg_foreign_pad_dist(pcb, 1, s.start_x, s.start_y, s.end_x, s.end_y,
                                  'F.Cu', base_clearance=0.1)
        ok = ok and d >= 0.1 + s.width / 2 - 1e-4
    check("smoothed copper clears the blocking pad", ok)
    check("blocked case stays octolinear", all(octolinear(s) for s in pcb.segments))


def test_dry_run_mutates_nothing():
    segs, (ex, ey) = staircase(10.0, 10.0, 20)
    pcb = _board(end_pads(ex, ey), list(segs))
    ids = [id(s) for s in pcb.segments]
    _n, _nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=0.1,
                                                           dry_run=True)
    check("dry run leaves the board alone",
          [id(s) for s in pcb.segments] == ids and not strip and not added)
    check("dry run still measures", st['spans'] >= 1 and st['saved_mm'] > 0.5, st)


def test_via_tap_splits_chain():
    segs, (ex, ey) = staircase(10.0, 10.0, 20)
    tapx, tapy = segs[10].start_x, segs[10].start_y
    via = Via(x=tapx, y=tapy, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    pcb = _board(end_pads(ex, ey), list(segs), vias=[via])
    _n, _nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=0.1)
    check("via tap splits the chain", st['chains'] == 2, st)
    touch = any(s.net_id == 1 and
                (math.hypot(s.start_x - tapx, s.start_y - tapy) < 0.3 + s.width / 2 or
                 math.hypot(s.end_x - tapx, s.end_y - tapy) < 0.3 + s.width / 2)
                for s in pcb.segments)
    check("via tap keeps copper contact", touch)


def test_writelist_custody():
    segs, (ex, ey) = staircase(10.0, 10.0, 20)
    res = {'new_segments': list(segs), 'new_vias': []}
    pcb = _board(end_pads(ex, ey), list(segs))
    results = [res]
    _n, _nets, strip, added, st = smooth_octolinear_chains(results, pcb, clearance=0.1)
    check("routed removals leave the write-list, not the strip",
          not strip and st['segs_removed'] > 0, (len(strip), st))
    check("added copper joins the owning result",
          added and all(s in res['new_segments'] for s in added))
    removed_still_listed = [s for s in segs
                            if s not in pcb.segments and s in res['new_segments']]
    check("removed routed segs dropped from new_segments", not removed_still_listed)


def test_skip_net_ids_absolute():
    segs, (ex, ey) = staircase(10.0, 10.0, 20)
    pcb = _board(end_pads(ex, ey), list(segs))
    _n, nets, strip, added, st = smooth_octolinear_chains([], pcb, clearance=0.1,
                                                          skip_net_ids={1})
    check("skip_net_ids wins", nets == 0 and not strip and not added and
          st['spans'] == 0, st)


def test_selection_scope_is_strict():
    segs, (ex, ey) = staircase(10.0, 10.0, 20)
    untouched = list(segs[10:])
    pcb = _board(end_pads(ex, ey), list(segs))
    smooth_octolinear_chains([], pcb, clearance=0.1,
                             eligible_segment_uuids={s.uuid for s in segs[:10]})
    check("unselected segments remain the same objects",
          all(s in pcb.segments for s in untouched))
    check("selection boundary remains fixed", segs[10] in pcb.segments)


def test_locked_selected_segment_is_immutable():
    segs, (ex, ey) = staircase(10.0, 10.0, 20)
    segs[10].locked = True
    pcb = _board(end_pads(ex, ey), list(segs))
    smooth_octolinear_chains([], pcb, clearance=0.1,
                             eligible_segment_uuids={s.uuid for s in segs})
    check("locked selected segment remains", segs[10] in pcb.segments)


def test_smooth_skip_reads_protection_notes():
    import protected_nets as pn
    from cleanup_pipeline import _smooth_skip_net_ids
    segs, (ex, ey) = staircase(10.0, 10.0, 4)
    pcb = _board(end_pads(ex, ey), list(segs))
    saved = dict(pn._notes)
    try:
        pn._notes.clear()
        check("no notes -> no skips", _smooth_skip_net_ids(pcb) == set())
        pn.note_protection_candidates({'/SIG': 'diff-pair'})
        check("protection note skips the net", _smooth_skip_net_ids(pcb) == {1})
        check("peek does not consume",
              pn._notes.get(pn.PRO_KEY, {}).get('/SIG') == 'diff-pair')
        pn._notes.clear()
        pn.note_impedance_specs({'/SIG': {'ohms': 50}})
        check("impedance note skips the net", _smooth_skip_net_ids(pcb) == {1})
    finally:
        pn._notes.clear()
        pn._notes.update(saved)


def test_keepout_blocks_shortcut():
    """A rule area with (tracks not_allowed) is a hard connector keep-out:
    the route detoured around it BY DESIGN, and no later stage repairs a
    shortcut through it (the run_all rule-area gate caught this class)."""
    segs, (ex, ey) = staircase(10.0, 10.0, 8, pitch=1.0)
    pcb = _board(end_pads(ex, ey), list(segs))
    # Small box in the pocket between staircase corners: the ORIGINAL route
    # clears it, only the shortcut diagonal (10,10)->(14,14) crosses it.
    pcb.board_info.keepouts = [{
        'polygon': [(11.3, 11.3), (11.7, 11.3), (11.7, 11.7), (11.3, 11.7)],
        'layers': {'F.Cu'}, 'tracks_allowed': False, 'vias_allowed': False,
    }]
    _n, _nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=0.1)
    from obstacle_map import point_in_polygon
    poly = pcb.board_info.keepouts[0]['polygon']
    intruders = []
    for s in pcb.segments:
        n = max(2, int(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) / 0.05))
        for q in range(n + 1):
            t = q / n
            px = s.start_x + t * (s.end_x - s.start_x)
            py = s.start_y + t * (s.end_y - s.start_y)
            if point_in_polygon(px, py, poly):
                intruders.append((s.start_x, s.start_y, s.end_x, s.end_y))
                break
    check("no smoothed copper inside the rule area", not intruders, intruders[:2])
    # same board, keepout permissive -> the shortcut IS taken (gating check)
    segs2, _ = staircase(10.0, 10.0, 8, pitch=1.0)
    pcb2 = _board(end_pads(ex, ey), list(segs2))
    pcb2.board_info.keepouts = [{
        'polygon': [(11.3, 11.3), (11.7, 11.3), (11.7, 11.7), (11.3, 11.7)],
        'layers': {'F.Cu'}, 'tracks_allowed': True, 'vias_allowed': True,
    }]
    _n2, _nets2, _s2, _a2, st2 = smooth_octolinear_chains([], pcb2, clearance=0.1)
    check("permissive rule area does not block", st2['saved_mm'] > st['saved_mm'],
          (st['saved_mm'], st2['saved_mm']))


def test_pipeline_gate():
    """Tri-state gating: the ``smooth`` kwarg is the FRONT default (route
    step passes True, diff/plane fronts False); KICAD_SMOOTH_ROUTE overrides
    both ways ('1' on, '0' off, '' = front default)."""
    from cleanup_pipeline import run_post_route_cleanup
    from routing_config import GridRouteConfig

    def _run(smooth):
        segs, (ex, ey) = staircase(10.0, 10.0, 20)
        pcb = _board(end_pads(ex, ey), list(segs))
        cfg = GridRouteConfig(clearance=0.1)
        out = run_post_route_cleanup([], pcb, {1}, cfg,
                                     snap=False, phantom=False, graze=False,
                                     octolinear=False, via_nudge=False,
                                     cycles=False, neck=False, smooth=smooth)
        return pcb, out

    def ran(pcb, out):
        return out.counts.get('smoothed_spans', 0) >= 1 and len(pcb.segments) < 20

    old = os.environ.pop('KICAD_SMOOTH_ROUTE', None)
    try:
        env_knobs.refresh()
        check("env unset + route-front default ON: runs", ran(*_run(True)))
        check("env unset + diff/plane-front default OFF: does not run",
              not ran(*_run(False)))
        os.environ['KICAD_SMOOTH_ROUTE'] = '0'
        env_knobs.refresh()
        check("env '0' overrides front ON: does not run", not ran(*_run(True)))
        os.environ['KICAD_SMOOTH_ROUTE'] = '1'
        env_knobs.refresh()
        check("env '1' overrides front OFF: runs", ran(*_run(False)))
        os.environ.pop('KICAD_SMOOTH_ROUTE', None)
        env_knobs.refresh()
        # Guide-corridor steps (#7): drawn geometry is the spec -- the front
        # default must not flatten the arch the route deliberately followed.
        segs_g, (exg, eyg) = staircase(10.0, 10.0, 20)
        pcb_g = _board(end_pads(exg, eyg), list(segs_g))
        from routing_config import GridRouteConfig
        from cleanup_pipeline import run_post_route_cleanup
        cfg_g = GridRouteConfig(clearance=0.1, guide_corridor_enabled=True)
        out_g = run_post_route_cleanup([], pcb_g, {1}, cfg_g,
                                       snap=False, phantom=False, graze=False,
                                       octolinear=False, via_nudge=False,
                                       cycles=False, neck=False, smooth=True)
        check("guide-corridor step: front default does not smooth",
              not ran(pcb_g, out_g))
        pcb_on, out_on = _run(True)
        check("strip carries input removals",
              len(out_on.input_strip_segments) > 0)
        check("output octolinear", all(octolinear(s) for s in pcb_on.segments))
    finally:
        if old is None:
            os.environ.pop('KICAD_SMOOTH_ROUTE', None)
        else:
            os.environ['KICAD_SMOOTH_ROUTE'] = old
        env_knobs.refresh()


if __name__ == '__main__':
    for fn in (test_staircase_collapses, test_blocked_shortcut_keeps_clearance,
               test_dry_run_mutates_nothing, test_via_tap_splits_chain,
               test_writelist_custody, test_skip_net_ids_absolute,
               test_selection_scope_is_strict, test_locked_selected_segment_is_immutable,
               test_smooth_skip_reads_protection_notes,
               test_keepout_blocks_shortcut, test_pipeline_gate):
        print(fn.__name__)
        fn()
    if fails:
        print(f"\nFAILED: {fails}")
        sys.exit(1)
    print("\nALL PASS")
