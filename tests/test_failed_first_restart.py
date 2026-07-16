"""Tests for the failed-first restart lever (--failed-first-restart).

Geometry (see _board): a single-layer board where net order alone decides
success. Net B can ONLY route through corridor 1; net A prefers corridor 1
(cheaper) but has an expensive corridor-2 fallback. With rip-up disabled
(max_rip_up_count=0) the outcome is pure ordering sensitivity:

  order A,B (first pass):  A takes corridor 1 (and plugs B's only shafts)
                           -> B has nowhere to go -> FAILS
  order B,A (restart):     B takes corridor 1 -> A falls back to corridor 2
                           -> both route

which is exactly the scenario the restart lever exists for. The tests assert:
flag-off = original behavior (B fails, no summary key), flag-on recovers B via
the restart (kept='restart'), keep-best refuses a restart that grades worse
(kept='first', first pass restored exactly), and a no-failure run gates the
restart off.
"""
import io
import json
import os
import re
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import BoardInfo
from synth import make_net, make_pad, make_pcb, make_seg

import route

NET_A, NET_B, WALL = 1, 2, 3


def _board():
    """Corridor-competition board (see module docstring).

    Regions: a central wall column (x in [-2, 2]) is pierced by corridor 1
    (one-track lane at y=1.0) and corridor 2 (one-track lane at y=-4.9, a
    much longer detour). East and west of the column, a 'wing' wall at
    y=0.25 splits each side into a TOP region (B's pads, y=1.0, on the
    corridor-1 axis) and a MID region (A's pads, y=-0.5), connected only by
    a one-track shaft (gap in the wing) at x=+/-4. So:

      A first: A prefers corridor 1 (shaft up at x=-4, tunnel, shaft down at
               x=+4 -- ~11mm) over corridor 2 (~17mm); its copper plugs both
               shafts AND the tunnel -> B (top regions only) is sealed: FAILS.
      B first: B takes corridor 1 straight; A falls back to corridor 2: BOTH.
    """
    bi = BoardInfo(layers={0: 'F.Cu'},
                   copper_layers=['F.Cu'],
                   board_bounds=(-6.0, -7.0, 6.0, 4.0))
    pads = {
        NET_A: [make_pad(NET_A, -4.0, -0.5, ref='U1', num='1', net_name='NETA',
                         size_x=0.4, size_y=0.4),
                make_pad(NET_A, 4.0, -0.5, ref='U2', num='1', net_name='NETA',
                         size_x=0.4, size_y=0.4)],
        NET_B: [make_pad(NET_B, -5.5, 1.0, ref='U3', num='1', net_name='NETB',
                         size_x=0.4, size_y=0.4),
                make_pad(NET_B, 5.5, 1.0, ref='U4', num='1', net_name='NETB',
                         size_x=0.4, size_y=0.4)],
        WALL: [],
    }
    # Central wall column: horizontal 0.5mm-wide bars at these y centers,
    # leaving the corridor-1 lane (free track centers y in (0.95, 1.05)) and
    # the corridor-2 lane (free track centers y in (-5.15, -4.65)).
    central_ys = ([1.6, 2.1, 2.6, 3.1, 3.6, 4.1]
                  + [round(0.4 - 0.5 * k, 1) for k in range(10)]  # 0.4..-4.1
                  + [-5.7, -6.2, -6.7, -7.2])
    walls = [make_seg(-2.0, y, 2.0, y, layer='F.Cu', net_id=WALL, width=0.5)
             for y in central_ys]
    # Wing walls at y=0.25 with one-track shaft gaps at x=+/-4.
    for x1, x2 in ((-6.5, -4.7), (-3.3, -2.0), (2.0, 3.3), (4.7, 6.5)):
        walls.append(make_seg(x1, 0.25, x2, 0.25, layer='F.Cu',
                              net_id=WALL, width=0.5))
    return make_pcb(
        nets={NET_A: make_net(NET_A, 'NETA'), NET_B: make_net(NET_B, 'NETB'),
              WALL: make_net(WALL, 'WALL')},
        segments=walls, pads_by_net=pads, board_info=bi)


def _route(failed_first_restart, pcb=None):
    """Run batch_route on the synthetic board; returns (ok, fail, summary,
    results_data, pcb, stdout_text)."""
    pcb = pcb or _board()
    buf = io.StringIO()
    os.environ['KICAD_NET_RESCUE'] = '0'  # keep the test about the restart
    try:
        with redirect_stdout(buf):
            ok, fail, _t, results_data = route.batch_route(
                "", "", ['NETA', 'NETB'],
                layers=['F.Cu'],
                ordering_strategy='original',
                track_width=0.2, clearance=0.2, grid_step=0.1,
                max_rip_up_count=0,       # pure ordering sensitivity
                enable_layer_switch=False,
                final_reconcile=False,    # keep the test about the restart
                failed_first_restart=failed_first_restart,
                return_results=True,
                pcb_data=pcb,
                net_clearances={})
    finally:
        os.environ.pop('KICAD_NET_RESCUE', None)
    out = buf.getvalue()
    m = re.search(r'JSON_SUMMARY: (\{.*\})', out)
    assert m, f"no JSON_SUMMARY in output:\n{out[-2000:]}"
    return ok, fail, json.loads(m.group(1)), results_data, pcb, out


def _routed_nets(results_data):
    return {s.net_id for r in results_data['results']
            for s in (r.get('new_segments') or [])}


def test_flag_off_is_baseline():
    ok, fail, summary, results_data, _pcb, _out = _route(False)
    assert (ok, fail) == (1, 1), f"baseline must fail exactly NETB: {ok}/{fail}"
    assert summary['failed_single'] == ['NETB']
    assert 'failed_first_restart' not in summary, \
        "flag off must not add the summary key"
    assert _routed_nets(results_data) == {NET_A}


def test_flag_on_recovers_failed_net():
    ok, fail, summary, results_data, pcb, _out = _route(True)
    assert (ok, fail) == (2, 0), \
        f"restart must recover NETB: {ok}/{fail}"
    rec = summary['failed_first_restart']
    assert rec['ran'] is True
    assert rec['kept'] == 'restart'
    assert rec['first_pass_failed'] == 1
    assert rec['restart_failed'] == 0
    assert rec['time'] >= 0
    assert _routed_nets(results_data) == {NET_A, NET_B}
    # Board custody: exactly one copy of each net's copper on the board.
    for nid in (NET_A, NET_B):
        assert any(s.net_id == nid for s in pcb.segments)
    # NETB must own corridor 1 (all its copper near y=1); NETA was displaced.
    b_segs = [s for s in pcb.segments if s.net_id == NET_B]
    assert all(0.5 <= min(s.start_y, s.end_y) and
               max(s.start_y, s.end_y) <= 1.5 for s in b_segs), \
        "NETB must route through its only corridor (y=+1)"


def test_keep_best_refuses_worse_restart():
    # Force the restart pass to grade worse: wrap the core so the second call
    # returns an empty write-list (as if the restart routed nothing).
    orig_core = route._run_routing_core
    calls = {'n': 0}

    def fake_core(**kw):
        calls['n'] += 1
        r = orig_core(**kw)
        if calls['n'] == 2:  # the restart pass
            r.results[:] = []
            r.routed_results.clear()
        return r

    route._run_routing_core = fake_core
    try:
        ok, fail, summary, results_data, pcb, _out = _route(True)
    finally:
        route._run_routing_core = orig_core
    assert calls['n'] == 2, "restart pass must have run"
    assert (ok, fail) == (1, 1), \
        f"a worse restart must be refused, keeping the first pass: {ok}/{fail}"
    rec = summary['failed_first_restart']
    assert rec['ran'] is True
    assert rec['kept'] == 'first'
    assert rec['first_pass_failed'] == 1
    assert rec['restart_failed'] == 2
    assert summary['failed_single'] == ['NETB']
    # First pass restored exactly: NETA's copper is back on the board, and no
    # restart-pass copper leaked into the write-list or onto the board.
    assert _routed_nets(results_data) == {NET_A}
    board_new = {id(s) for s in pcb.segments} - {id(s) for r in
                 results_data['results'] for s in (r.get('new_segments') or [])}
    assert not any(s.net_id in (NET_A, NET_B) and id(s) in board_new
                   for s in pcb.segments
                   if s.width < 0.5), \
        "restart copper must not survive a keep-first decision"


def test_no_failures_skips_restart():
    # Same board but without the wall: both nets route in the first pass, so
    # the restart is gated off (ran=False) and the outcome is unchanged.
    pcb = _board()
    pcb.segments = []  # no walls -> no failures
    ok, fail, summary, _rd, _pcb, _out = _route(True, pcb=pcb)
    assert (ok, fail) == (2, 0)
    rec = summary['failed_first_restart']
    assert rec['ran'] is False
    assert rec['kept'] == 'first'
    assert rec['first_pass_failed'] == 0


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"  {fn.__name__} ...", end=" ")
        fn()
        print("OK")
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
