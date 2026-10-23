#!/usr/bin/env python3
"""A route the terminal short gate refuses takes its via-in-pad unblock with it.

`_route_with_via_unblock` drops a fab-floor via inside a boxed pad (#189) and
COMMITS it to pcb_data at once (#803), before its caller has converted the
path. Two callers can still refuse that path at the terminal short gate:
`route_net_with_obstacles` (the #157 gate) and the Phase 3 tap loop (the tap
gate, which also refuses a path the unconditional TAP PAD-VIA RESCUE routed
through a via it committed the same way). The refused route's result is
never written, so neither is the via -- yet it stayed in pcb_data, and the
net's next route read it as the pad's own via: it ended on B.Cu at the pad and
emitted no via. Measured on the fixtures below at bb5943a0: the orphan
fallback tap and the second 2-pad route both reported the pad connected, and
the copper their results carry (what is written) left it open. Found reviewing
#1273, whose Phase 1 gate got the same fix (`_uncommit_unblock_copper`).

Fixture: P1, a 0.3 mm SMD pad boxed in by another net's F.Cu tracks, so any
route into it needs a via-in-pad. The gate is stubbed to fire once (no small
board makes a terminal physically overlap foreign copper under the default
policy); everything else is the real engine.

Rows:
  1. tap gate, via from _route_with_via_unblock
  2. tap gate, via from the TAP PAD-VIA RESCUE (the wrapper is stubbed to
     fail once as congestion, so the rescue rung runs for real)
  3. #157 gate in route_net_with_obstacles, then the net routed again
  4. controls: with no refusal the unblock via is committed AND carried (tap
     and 2-pad); a refusal with no unblock (P1 unboxed) removes nothing, and
     the board's own same-net and foreign vias survive every refusal.

    python3 tests/test_terminal_gate_uncommits_unblock_via.py
"""
import contextlib
import io
import os
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

import constraint_agreement as ca                       # noqa: E402
import fab_tiers                                         # noqa: E402
import single_ended_routing as ser                       # noqa: E402
from check_connected import check_net_connectivity       # noqa: E402
from connectivity import get_multipoint_net_pads         # noqa: E402
from kicad_parser import parse_kicad_pcb                 # noqa: E402
from obstacle_map import build_base_obstacle_map         # noqa: E402
from routing_config import GridRouteConfig               # noqa: E402

FAILS = []
SECTION = ['']
OVERLAP = [(SimpleNamespace(layer='F.Cu'), 0.0, None)]
P1 = (26, 20)
# A same-net via far from every route and a foreign via: the board's own
# copper, which no refusal may take.
OWN_VIA = (12, 30, 0.6, 0.3, 2)
FOREIGN_VIA = (12, 10, 0.6, 0.3, 1)


def section(title):
    SECTION[0] = title.split('.')[0]
    print(title)


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(f'{SECTION[0]}: {name}')


def fixture(pads, boxed=True):
    """Net B pads [(ref, x, y, size)]; P1 boxed by net A's F.Cu tracks."""
    h = 0.5        # no 0.6 via fits inside the box, a fab-floor one does
    x, y = P1
    box = [(x - h, y - h, x + h, y - h), (x + h, y - h, x + h, y + h),
           (x + h, y + h, x - h, y + h), (x - h, y + h, x - h, y - h)]
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'boxed.kicad_pcb')
        ca.write_board(board, vias=[OWN_VIA, FOREIGN_VIA],
                       segments=[(a, b, c, d, 0.2, 'F.Cu', 1) for a, b, c, d in box]
                       if boxed else [],
                       footprints=[dict(ref=r, x=px, y=py, net_id=2, net_name='B', size=s)
                                   for r, px, py, s in pads])
        pcb = parse_kicad_pcb(board)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.4, clearance=0.15,
                          grid_step=0.05, via_size=0.6, via_drill=0.3)
    return pcb, cfg, build_base_obstacle_map(pcb, cfg, [2], static_base=True)


MULTI = [('P2', 5, 20, 0.6), ('P3', 20, 20, 0.6), ('P1', P1[0], P1[1], 0.3)]
TWO = [('P3', 20, 20, 0.6), ('P1', P1[0], P1[1], 0.3)]


@contextlib.contextmanager
def patched(name, make):
    real = getattr(ser, name)
    calls = []
    setattr(ser, name, make(real, calls))
    try:
        yield calls
    finally:
        setattr(ser, name, real)


def gate_fires_on(n):
    """_neck_route_terminal_grazes reports an overlap on its n-th call only."""
    def make(real, calls):
        def stub(segments, *a, **k):
            calls.append(1)
            got = real(segments, *a, **k)
            return OVERLAP if len(calls) == n else got
        return stub
    return make


def neck_overlaps_once(real, calls):
    """_neck_terminal_grazes (the #157 gate) reports an overlap on its 1st call."""
    def stub(segments, *a, **k):
        calls.append(1)
        necked, hard = real(segments, *a, **k)
        return (necked, OVERLAP) if len(calls) == 1 else (necked, hard)
    return stub


def wrapper_fails_on(n):
    """_route_with_via_unblock fails as CONGESTION (full budget) on its n-th
    call, so the tap loop's unconditional pad-via rescue runs."""
    def make(real, calls):
        def stub(*a, **k):
            calls.append(1)
            if len(calls) == n:
                return (None, 5000, [], [], False, 5000, 5000, False, None, [], [])
            return real(*a, **k)
        return stub
    return make


def quiet(fn, *a, **k):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ret = fn(*a, **k)
    return ret, out.getvalue()


def added(pcb, before):
    return [v for v in pcb.vias if id(v) not in before]


def board_copper_kept(pcb):
    have = {(round(v.x, 3), round(v.y, 3), v.net_id) for v in pcb.vias}
    return ((OWN_VIA[0], OWN_VIA[1], 2) in have and (FOREIGN_VIA[0], FOREIGN_VIA[1], 1) in have)


def written_connects(pcb, *results):
    segs, vias = {}, {}
    for r in results:
        for s in r.get('new_segments') or []:
            segs[id(s)] = s
        for v in r.get('new_vias') or []:
            vias[id(v)] = v
    got = check_net_connectivity(2, list(segs.values()), list(vias.values()),
                                 pcb.pads_by_net.get(2, []), [], tolerance=0.02)
    return bool(got.get('connected')), len(got.get('disconnected_pads') or [])


def run_taps(pads, boxed=True, gate_call=None, wrapper_call=None):
    pcb, cfg, obs = fixture(pads, boxed)
    before = {id(v) for v in pcb.vias}
    pad_info = get_multipoint_net_pads(pcb, 2, cfg)
    log = ''
    with contextlib.ExitStack() as st:
        if gate_call:
            st.enter_context(patched('_neck_route_terminal_grazes', gate_fires_on(gate_call)))
        if wrapper_call:
            st.enter_context(patched('_route_with_via_unblock', wrapper_fails_on(wrapper_call)))
        main, l1 = quiet(ser.route_multipoint_main, pcb, 2, cfg, obs, pad_info)
        log += l1
        taps = {}
        if main and not main.get('failed'):
            taps, l2 = quiet(ser.route_multipoint_taps, pcb, 2, cfg, obs, main)
            log += l2
    return pcb, before, main, taps or {}, log


def tap_rows(label, wrapper_call, fixture_marker):
    section(label)
    # Phase 1 routes P2-P3 (conversion 1); the tap into boxed P1 is conversion 2.
    pcb, before, main, taps, log = run_taps(MULTI, gate_call=2, wrapper_call=wrapper_call)
    check('fixture: the tap into P1 dropped its via, then the tap gate refused it',
          fixture_marker in log and 'failing the MST edge rather than' in log
          and log.index(fixture_marker) < log.index('failing the MST edge rather than'))
    check('fixture: the orphan fallback retried P1', 'attempting orphaned pad' in log)
    carried = {id(v) for v in taps.get('new_vias') or []}
    ghosts = [(round(v.x, 3), round(v.y, 3)) for v in added(pcb, before) if id(v) not in carried]
    check('pcb_data holds no via the results do not carry', not ghosts, str(ghosts))
    ok, n_open = written_connects(pcb, taps)
    reported = len(taps.get('failed_pads_info') or [])
    check('the written copper connects every pad the taps report connected',
          ok and reported == 0, f'{n_open} pad(s) open in the written copper; '
                                f'taps report {reported} failed')
    check("the board's own vias survive", board_copper_kept(pcb))


def main():
    fab_tiers.set_escalation_policy('fab')

    tap_rows('1. tap gate: via from the boxed-pad unblock', None,
             'Via-in-pad unblock: dropped')
    # Wrapper call 1 is Phase 1's main edge, call 2 the tap into P1.
    tap_rows('2. tap gate: via from the TAP PAD-VIA RESCUE', 2, 'TAP PAD-VIA RESCUE')

    section('3. #157 gate in route_net_with_obstacles, then the net routed again')
    pcb, cfg, obs = fixture(TWO)
    before = {id(v) for v in pcb.vias}
    with patched('_neck_terminal_grazes', neck_overlaps_once):
        first, log1 = quiet(ser.route_net_with_obstacles, pcb, 2, cfg, obs)
    check('fixture: the route dropped its via, then the #157 gate refused it',
          'Via-in-pad unblock: dropped' in log1 and 'rejecting the route' in log1
          and bool(first) and first.get('failed'))
    check('pcb_data holds no via of the refused route',
          not added(pcb, before), str([(v.x, v.y) for v in added(pcb, before)]))
    second, _ = quiet(ser.route_net_with_obstacles, pcb, 2, cfg, obs)
    check('the net routes again', bool(second) and not second.get('failed'))
    if second and not second.get('failed'):
        ok, n_open = written_connects(pcb, second)
        check("the second route's written copper connects both pads", ok,
              f'{n_open} pad(s) open')
    check("the board's own vias survive", board_copper_kept(pcb))

    section('4. controls')
    pcb, before, main_r, taps, log = run_taps(MULTI)
    carried = {id(v) for v in taps.get('new_vias') or []}
    new = added(pcb, before)
    check('no refusal: the tap unblock via is committed and carried',
          'Via-in-pad unblock: dropped' in log and new and all(id(v) in carried for v in new),
          str([(round(v.x, 3), round(v.y, 3), id(v) in carried) for v in new]))
    check('no refusal: the written copper connects every pad', written_connects(pcb, taps)[0])
    pcb, cfg, obs = fixture(TWO)
    before = {id(v) for v in pcb.vias}
    r, log = quiet(ser.route_net_with_obstacles, pcb, 2, cfg, obs)
    new = added(pcb, before)
    check('no refusal: the 2-pad unblock via is committed and carried',
          'Via-in-pad unblock: dropped' in log and bool(r) and not r.get('failed')
          and new and all(id(v) in {id(x) for x in r['new_vias']} for v in new))
    pcb, cfg, obs = fixture(TWO, boxed=False)
    n0 = (len(pcb.vias), len(pcb.segments))
    with patched('_neck_terminal_grazes', neck_overlaps_once):
        r, log = quiet(ser.route_net_with_obstacles, pcb, 2, cfg, obs)
    check('a refusal with no unblock removes nothing',
          'Via-in-pad unblock' not in log and r.get('failed')
          and (len(pcb.vias), len(pcb.segments)) == n0 and board_copper_kept(pcb),
          f'{n0} -> {(len(pcb.vias), len(pcb.segments))}')
    pcb, before, main_r, taps, log = run_taps(MULTI, boxed=False, gate_call=2)
    check('a tap refusal with no unblock removes nothing',
          'Via-in-pad unblock' not in log and 'failing the MST edge rather than' in log
          and board_copper_kept(pcb)
          and not [v for v in added(pcb, before)
                   if id(v) not in {id(x) for x in taps.get('new_vias') or []}])

    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
