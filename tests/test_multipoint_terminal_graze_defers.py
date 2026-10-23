#!/usr/bin/env python3
"""A multipoint main edge whose terminal copper overlaps foreign copper defers
that edge; it does not fail the whole net (#101's contract).

route_multipoint_main tries MST edges longest-first so one boxed pad cannot
zero a power net (#101). The terminal-bridge short gate (ux pf9) that runs
after Phase 1 FOUND a path returned failed for the whole net instead,
bypassing that loop: cm5-io's +3V3_CM5 (34 pads) routed nothing because its
longest edge's terminal grazed a foreign via-in-pad by 0.085 mm.

Fixture: net B with three pads in a row, so the MST has two edges and the
longest (P1-P2) is Phase 1's first pick. Four sections:

1. The terminal-overlap measurement is stubbed to report an overlap on the
   FIRST Phase 1 conversion only (no small board reproduces a physical
   terminal overlap under the default policy); this pins the control flow:
   Phase 1 must ship the other edge, keep the deferred one for Phase 3, and
   the taps must then connect every pad.
2. The retry is a FRESH call with that edge deferred: the same copper, MST
   order and obstacle map as calling route_multipoint_main with
   `_deferred_main_edges` directly, and the same answer on a second run.
3. The discarded attempt leaves no copper behind. P1 is boxed in on F.Cu, so
   the first attempt drops a #189 via-in-pad unblock, which
   `_route_with_via_unblock` COMMITS to pcb_data (#803) before the gate
   discards the attempt. Left there it is in no result, so it is never
   written, while the Phase 3 tap reads it as the net's own: it reached P1 on
   B.Cu through that via and reported every pad connected, and the written
   copper left P1 open.
4. Unstubbed: under `--escalation off` a full-width terminal into a small pad
   beside a foreign track is a clearance graze the gate refuses (#842/#1159).
   Phase 1 now routes the other edge, and the tap gate still refuses P1's
   terminal, so P1 is reported rather than shipped grazing.

    python3 tests/test_multipoint_terminal_graze_defers.py
"""
import contextlib
import io
import os
import re
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


def section(title):
    SECTION[0] = title.split('.')[0]
    print(title)


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(f'{SECTION[0]}: {name}')


def fixture(p1_size=0.6, segments=(), track_width=0.15):
    """Net B on P1 (5,20), P2 (20,20), P3 (32,20); `segments` are other nets'."""
    fp = lambda ref, x, y, s: dict(ref=ref, x=x, y=y, net_id=2, net_name='B', size=s)
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'graze.kicad_pcb')
        ca.write_board(board, segments=segments,
                       footprints=[fp('P1', 5, 20, p1_size), fp('P2', 20, 20, 0.6),
                                   fp('P3', 32, 20, 0.6)])
        pcb = parse_kicad_pcb(board)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=track_width, clearance=0.15,
                          grid_step=0.05, via_size=0.6, via_drill=0.3)
    obs = build_base_obstacle_map(pcb, cfg, [2], static_base=True)
    return pcb, cfg, obs, get_multipoint_net_pads(pcb, 2, cfg)


@contextlib.contextmanager
def first_conversion_overlaps():
    real = ser._neck_route_terminal_grazes
    calls = []

    def stub(segments, *a, **k):
        calls.append(len(segments))
        real(segments, *a, **k)
        if len(calls) == 1:
            return [(SimpleNamespace(layer='F.Cu'), 0.0, None)]
        return []

    ser._neck_route_terminal_grazes = stub
    try:
        yield calls
    finally:
        ser._neck_route_terminal_grazes = real


@contextlib.contextmanager
def escalation(policy):
    prev = fab_tiers.get_escalation_policy()
    fab_tiers.set_escalation_policy(policy)
    try:
        yield
    finally:
        fab_tiers.set_escalation_policy(*prev)


def quiet(fn, *a, **k):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ret = fn(*a, **k)
    return ret, out.getvalue()


def copper(res):
    return (sorted((round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
                    round(s.end_y, 4), s.layer, round(s.width, 4))
                   for s in res.get('new_segments') or []),
            sorted((round(v.x, 4), round(v.y, 4), v.size) for v in res.get('new_vias') or []))


def map_print(obs, pad_info):
    """The obstacle map's counters plus every cell's blocked/via-blocked state
    in a 2 mm window around each terminal, on both layers."""
    cells = []
    for gx0, gy0 in {(p[0], p[1]) for p in pad_info}:
        for gx in range(gx0 - 20, gx0 + 21):
            for gy in range(gy0 - 20, gy0 + 21):
                cells.append((obs.is_blocked(gx, gy, 0), obs.is_blocked(gx, gy, 1),
                              obs.is_via_blocked(gx, gy)))
    return obs.get_stats(), obs.get_static_stats(), tuple(cells)


def routed(res):
    return bool(res) and not res.get('failed')


def stubbed_run():
    pcb, cfg, obs, pad_info = fixture()
    with escalation('fab'), first_conversion_overlaps():
        res, log = quiet(ser.route_multipoint_main, pcb, 2, cfg, obs, pad_info)
    return pcb, cfg, obs, res, log


def section_control_flow():
    section('1. a grazing main edge is deferred, not the net failed')
    pcb, cfg, obs, res, log = stubbed_run()
    stats = map_print(obs, res['multipoint_pad_info']) if routed(res) else None
    check('fixture: the overlap gate fired on Phase 1', 'would OVERLAP' in log)
    check('Phase 1 still routes a main edge', routed(res),
          str(res and {k: res[k] for k in ('failed', 'iterations') if k in res}))
    if not routed(res):
        return None
    edges = [tuple(sorted(e[:2])) for e in res['mst_edges']]
    m = re.search(r'routing longest MST edge \(pads (\d+) and (\d+)', log)
    deferred = tuple(sorted((int(m.group(1)), int(m.group(2))))) if m else None
    check('the shipped main edge is not the deferred longest edge',
          deferred is not None and edges[0] != deferred, f'{edges}, deferred {deferred}')
    check('the deferred edge is kept for Phase 3', deferred in edges[1:], str(edges))
    note = res.get('edge_selection_note') or {}
    check('the net story names the terminal gate, not a selector, as the reason',
          note.get('mode') == 'terminal-gate-deferred'
          and note.get('deferred_by_terminal_gate') == [list(res['mst_edges'][-1][:2])]
          and 'Deferring MST edge' in log, str(note))
    with escalation('fab'):
        taps, _ = quiet(ser.route_multipoint_taps, pcb, 2, cfg, obs, res)
    taps = taps or {}
    check('the taps connect every pad', not (taps.get('failed_pads_info') or []),
          str(taps.get('failed_pads_info')))
    return res, stats


def section_fresh_call(first):
    section('2. the retry is a fresh call with the edge deferred')
    if first is None:
        check('section 1 produced a result to compare', False)
        return
    res, stats = first
    key = tuple(res['mst_edges'][-1][:2])
    pcb, cfg, obs, pad_info = fixture()
    with escalation('fab'):
        fresh, _ = quiet(ser.route_multipoint_main, pcb, 2, cfg, obs, pad_info,
                         _deferred_main_edges=frozenset({key}))
    check('a direct call with the edge deferred routes', routed(fresh))
    if not routed(fresh):
        return
    check('same copper as the direct call', copper(res) == copper(fresh),
          f'{copper(res)} vs {copper(fresh)}')
    check('same MST order as the direct call', res['mst_edges'] == fresh['mst_edges'],
          f"{res['mst_edges']} vs {fresh['mst_edges']}")
    now = map_print(obs, pad_info)
    check('same obstacle map as the direct call', stats == now,
          f'counters {stats[:2]} vs {now[:2]}, '
          f'{sum(a != b for a, b in zip(stats[2], now[2]))} cell(s) differ')
    _, _, _, again, _ = stubbed_run()
    check('deterministic: a second stubbed run ships the same copper',
          routed(again) and copper(again) == copper(res) and again['mst_edges'] == res['mst_edges'])


def section_no_ghost_copper():
    section('3. the discarded attempt leaves no unblock copper behind')
    h = 0.5        # foreign F.Cu box around P1: no 0.6 via fits inside, a fab-floor one does
    box = [(5 - h, 20 - h, 5 + h, 20 - h), (5 + h, 20 - h, 5 + h, 20 + h),
           (5 + h, 20 + h, 5 - h, 20 + h), (5 - h, 20 + h, 5 - h, 20 - h)]
    pcb, cfg, obs, pad_info = fixture(p1_size=0.3, track_width=0.4,
                                      segments=[(a, b, c, d, 0.2, 'F.Cu', 1) for a, b, c, d in box])
    vias0 = {id(v) for v in pcb.vias}
    with escalation('fab'), first_conversion_overlaps():
        res, log = quiet(ser.route_multipoint_main, pcb, 2, cfg, obs, pad_info)
    check('fixture: the discarded attempt dropped a via-in-pad unblock',
          'Via-in-pad unblock' in log.split('would OVERLAP')[0] and 'would OVERLAP' in log)
    check('Phase 1 still routes a main edge', routed(res))
    if not routed(res):
        return
    carried = {id(v) for v in res.get('new_vias') or []}
    ghosts = [(round(v.x, 3), round(v.y, 3)) for v in pcb.vias
              if id(v) not in vias0 and id(v) not in carried]
    check('pcb_data holds no via the result does not carry', not ghosts, str(ghosts))
    with escalation('fab'):
        taps, _ = quiet(ser.route_multipoint_taps, pcb, 2, cfg, obs, res)
    taps = taps or {}
    segs = {id(s): s for s in (res.get('new_segments') or []) + (taps.get('new_segments') or [])}
    vias = {id(v): v for v in (res.get('new_vias') or []) + (taps.get('new_vias') or [])}
    written = check_net_connectivity(2, list(segs.values()), list(vias.values()),
                                     pcb.pads_by_net.get(2, []), [], tolerance=0.02)
    check('the copper the results carry (what is written) connects every pad',
          bool(written.get('connected')),
          f"{len(written.get('disconnected_pads') or [])} pad(s) open; taps report "
          f"{len(taps.get('failed_pads_info') or [])} failed")


def section_unstubbed_escalation_off():
    section('4. unstubbed: an --escalation off terminal graze defers the edge')
    pcb, cfg, obs, pad_info = fixture(p1_size=0.3, track_width=0.4,
                                      segments=[(2, 20.4, 9, 20.4, 0.2, 'F.Cu', 1)])
    with escalation('off'):
        res, log = quiet(ser.route_multipoint_main, pcb, 2, cfg, obs, pad_info)
    check('fixture: the gate refused a terminal graze on Phase 1',
          'Phase 1 terminal copper' in log and 'forbids necking it' in log)
    check('Phase 1 still routes a main edge', routed(res))
    if not routed(res):
        return
    edges = [tuple(sorted(e[:2])) for e in res['mst_edges']]
    check('the main edge avoids P1 and the P1 edge waits for Phase 3',
          edges[0] == (1, 2) and (0, 1) in edges[1:], str(edges))
    with escalation('off'):
        taps, tlog = quiet(ser.route_multipoint_taps, pcb, 2, cfg, obs, res)
    failed = sorted(p.get('component_ref') for p in (taps or {}).get('failed_pads_info') or [])
    check('the tap gate still refuses P1: reported, not shipped grazing',
          failed == ['P1'] and 'forbids necking it' in tlog, str(failed))


def main():
    section_fresh_call(section_control_flow())
    section_no_ghost_copper()
    section_unstubbed_escalation_off()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
