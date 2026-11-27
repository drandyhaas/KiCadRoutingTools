#!/usr/bin/env python3
"""smooth_octolinear_chains on a net above max_net_segs (--smooth-large-nets).

By default such a net is skipped whole (`nets_skipped_large`), as before: a
power net that taps many pads crosses 400 segments mid-run and ships its grid
staircases. With smooth_large_nets (route.py --smooth-large-nets) it is
smoothed with ONE connectivity check for the whole net instead of one per
chain:

  * default: the net is skipped and counted, its copper untouched;
  * flag: a staircase on a net of 450+ segments collapses (octolinear,
    endpoints kept), counted in `nets_large`;
  * flag: if the whole net's result grades worse, none of it is committed: the
    copper is unchanged and the net is counted in `nets_reverted_large`;
  * the cleanup pipeline: default skips it, the flag smooths it and reports a
    revert, and with smoothing off the flag does nothing.

    python3 tests/test_smooth_large_net.py
"""
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

import check_connected  # noqa: E402
from kicad_parser import Pad, Net, PCBData, BoardInfo, Segment  # noqa: E402
from pcb_modification import smooth_octolinear_chains  # noqa: E402

fails = []


def check(name, cond, detail=""):
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)
        if 'pytest' in sys.modules:       # pytest: fail the test case here
            raise AssertionError(f"{name}  {detail}")


def _pad(ref, x, y):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5, shape='rect',
               layers=['F.Cu'], net_id=1, net_name='/GND', drill=0)


def board():
    """A 20-step staircase U1 -> U2, plus a 450-piece run U3 -> U4 on the same
    net: 470 segments, above the default max_net_segs (400)."""
    segs, x, y = [], 10.0, 10.0
    for i in range(20):
        if i % 2 == 0:
            segs.append(Segment(start_x=x, start_y=y, end_x=x + 0.2, end_y=y,
                                width=0.15, layer='F.Cu', net_id=1))
            x += 0.2
        else:
            segs.append(Segment(start_x=x, start_y=y, end_x=x, end_y=y + 0.2,
                                width=0.15, layer='F.Cu', net_id=1))
            y += 0.2
    stair_end = (x, y)
    for i in range(450):
        segs.append(Segment(start_x=20.0 + 0.1 * i, start_y=40.0, end_x=20.1 + 0.1 * i,
                            end_y=40.0, width=0.15, layer='F.Cu', net_id=1))
    pads = {1: [_pad('U1', 10.0, 10.0), _pad('U2', *stair_end),
                _pad('U3', 20.0, 40.0), _pad('U4', 65.0, 40.0)]}
    pcb = PCBData(footprints={}, nets={1: Net(1, '/GND')}, segments=segs, vias=[],
                  board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=(0.0, 0.0, 200.0, 200.0)),
                  pads_by_net=pads)
    return pcb, stair_end


def stair_segs(pcb):
    return [s for s in pcb.segments if max(s.start_y, s.end_y) < 30.0]


def _copper(pcb):
    return [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments]


def test_default_skips_large_net():
    pcb, _ = board()
    before = _copper(pcb)
    _n, nets, _strip, added, st = smooth_octolinear_chains([], pcb, clearance=0.1)
    check("default: a 470-segment net is skipped and counted",
          st['nets_skipped_large'] == 1 and st.get('nets_large') == 0 and st['spans'] == 0, str(st))
    check("default: its copper is untouched", _copper(pcb) == before and not added)


def test_large_net_is_smoothed():
    pcb, (ex, ey) = board()
    n_before = len(pcb.segments)
    _n, nets, _strip, _added, st = smooth_octolinear_chains([], pcb, clearance=0.1,
                                                             smooth_large_nets=True)
    stair = stair_segs(pcb)
    check("flag: a 470-segment net is smoothed, not skipped",
          st.get('nets_large') == 1 and st['nets_skipped_large'] == 0 and st['spans'] > 0, str(st))
    check("its staircase collapsed", len(stair) < 20 and len(pcb.segments) < n_before,
          f"{len(stair)} staircase segments left")
    check("output is octolinear",
          all(abs(s.end_x - s.start_x) < 1e-6 or abs(s.end_y - s.start_y) < 1e-6
              or abs(abs(s.end_x - s.start_x) - abs(s.end_y - s.start_y)) < 1e-6 for s in stair))
    ends = {(round(p[0], 4), round(p[1], 4)) for s in stair
            for p in ((s.start_x, s.start_y), (s.end_x, s.end_y))}
    check("both staircase endpoints kept", (10.0, 10.0) in ends and (round(ex, 4), round(ey, 4)) in ends)


def test_large_net_worse_result_reverts_whole_net():
    pcb, _ = board()
    before = [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments]
    real = check_connected.check_net_connectivity
    calls = []

    def fake(net_id, segs, *a, **k):
        # the smoothed whole net grades one component worse than it came
        calls.append(len(segs))
        res = real(net_id, segs, *a, **k)
        if len(calls) > 1:
            res = dict(res, num_components=(res.get('num_components') or 1) + 1)
        return res

    check_connected.check_net_connectivity = fake
    try:
        _n, nets, _strip, added, st = smooth_octolinear_chains([], pcb, clearance=0.1,
                                                               smooth_large_nets=True)
    finally:
        check_connected.check_net_connectivity = real
    after = [(s.start_x, s.start_y, s.end_x, s.end_y) for s in pcb.segments]
    check("the whole net is reverted", st.get('nets_reverted_large') == 1 and st['spans'] == 0, str(st))
    check("its copper is unchanged", after == before and not added)
    check("one check for the net as it came, one for the result", len(calls) == 2, str(calls))


def _pipeline(pcb, **kw):
    import contextlib
    import io
    from cleanup_pipeline import run_post_route_cleanup
    from routing_config import GridRouteConfig
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = run_post_route_cleanup([], pcb, {1}, GridRouteConfig(clearance=0.1),
                                     snap=False, phantom=False, graze=False,
                                     octolinear=False, via_nudge=False,
                                     cycles=False, neck=False, **kw)
    return out, buf.getvalue()


def test_pipeline_arms():
    """The route step's cleanup: default and --no-smoothing leave the large
    net's staircase; only smoothing with the flag collapses it."""
    os.environ.pop('KICAD_SMOOTH_ROUTE', None)
    for label, kw, collapsed in (("default", dict(smooth=True), False),
                                 ("--smooth-large-nets", dict(smooth=True, smooth_large_nets=True), True),
                                 ("--no-smoothing --smooth-large-nets",
                                  dict(smooth=False, smooth_large_nets=True), False)):
        pcb, _ = board()
        _pipeline(pcb, **kw)
        n = len(stair_segs(pcb))
        check(f"pipeline, {label}: staircase {'collapsed' if collapsed else 'left as it came'}",
              (n < 20) if collapsed else (n == 20), f"{n} staircase segments")


def test_pipeline_reports_a_reverted_large_net():
    """The cleanup pipeline counts and prints a large net that kept its copper."""
    pcb, _ = board()
    real = check_connected.check_net_connectivity

    def fake(net_id, segs, *a, **k):
        # other cleanup passes check the net too: grade worse only the
        # smoothed result (fewer segments than the 470 it came with)
        res = real(net_id, segs, *a, **k)
        if len(segs) < 470:
            res = dict(res, num_components=(res.get('num_components') or 1) + 1)
        return res

    check_connected.check_net_connectivity = fake
    try:
        out, text = _pipeline(pcb, smooth=True, smooth_large_nets=True)
    finally:
        check_connected.check_net_connectivity = real
    check("counts['smoothed_nets_reverted_large'] == 1",
          out.counts.get('smoothed_nets_reverted_large') == 1, str(out.counts.get('smoothed_nets_reverted_large')))
    check("the smoothing line says so", "1 large net(s) reverted" in text,
          [ln for ln in text.splitlines() if 'smoothing' in ln.lower()])


if __name__ == '__main__':
    for t in (test_default_skips_large_net, test_large_net_is_smoothed,
              test_large_net_worse_result_reverts_whole_net, test_pipeline_arms,
              test_pipeline_reports_a_reverted_large_net):
        print(t.__name__)
        t()
    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'OK'}")
    sys.exit(1 if fails else 0)
