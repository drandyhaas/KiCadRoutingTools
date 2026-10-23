#!/usr/bin/env python3
"""A caller that DROPS a routing result takes back its via-in-pad unblock.

`_route_with_via_unblock` commits a #189 via-in-pad unblock to pcb_data the
moment its route is found (#803), long before the caller commits the result.
The writer reads results, never pcb_data, so a caller that drops a result --
a probe that only reads the path, a forced-link batch that fails part way, a
tap retry that loses to the pass it was retrying, or the pass a winning retry
replaces -- left that via in pcb_data: never written, yet read by the net's
later routes as its own copper, until cleanup's orphan sweep. Found reviewing
#1273; the in-callee half (the terminal gates) is
tests/test_terminal_gate_uncommits_unblock_via.py.

`uncommit_dropped_result(pcb_data, dropped, kept=None)` is the one call every
such caller makes. A tap result carries its main route as a prefix, which IS
committed, so a dropped tap result names the result kept in its place and the
copper they share is spared.

Rows:
  1. the helper: takes back only the dropped result's copper that is in
     pcb_data and not carried by `kept`; the board's own copper, the kept
     result's and anything not on the board are untouched
  2. route_oracle_links: link 1 routes into a boxed pad (its unblock via is
     committed), link 2 cannot route -- the batch fails and drops link 1;
     its via must not stay behind
  3. control: the same link alone succeeds and keeps its via (committed and
     carried)
  4. Phase 3's stub-switch retry (control flow: the switch and the tap pass
     are stubbed; the tap pass commits a via the way #803 does): a retry
     that reconnects nothing is dropped and takes its via back, the pass it
     retried keeps all of its copper; a retry that reconnects a pad is kept
     with its via

    python3 tests/test_dropped_route_uncommits_unblock_via.py
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
from kicad_parser import Segment, Via, parse_kicad_pcb   # noqa: E402
from obstacle_map import build_base_obstacle_map         # noqa: E402
from routing_config import GridRouteConfig               # noqa: E402

FAILS = []
SECTION = ['']
P1 = (26, 20)


def section(title):
    SECTION[0] = title.split('.')[0]
    print(title)


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(f'{SECTION[0]}: {name}')


def quiet(fn, *a, **k):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ret = fn(*a, **k)
    return ret, out.getvalue()


def helper_rows():
    section('1. uncommit_dropped_result takes back only what the drop owned')
    helper = getattr(ser, 'uncommit_dropped_result', None)
    check('single_ended_routing.uncommit_dropped_result exists', helper is not None)
    if helper is None:
        return
    seg = lambda x: Segment(start_x=x, start_y=0, end_x=x + 1, end_y=0, width=0.2,
                            layer='F.Cu', net_id=2)
    via = lambda x: Via(x=x, y=0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=2)
    board_s, board_v = seg(0), via(0)                  # the board's own copper
    main_s, main_v = seg(10), via(10)                  # committed main route
    pre_v, pre_s = via(20), seg(20)                    # dropped result's unblock
    loose_v = via(30)                                  # dropped, never committed
    kept_v = via(40)                                   # kept result's unblock
    pcb = SimpleNamespace(segments=[board_s, main_s, pre_s],
                          vias=[board_v, main_v, pre_v, kept_v], _copper_epoch=7)
    dropped = {'new_segments': [main_s, pre_s], 'new_vias': [main_v, pre_v, loose_v]}
    kept = {'new_segments': [main_s], 'new_vias': [main_v, kept_v]}
    helper(pcb, dropped, kept=kept)
    check('the dropped result\'s unblock copper is taken back',
          pre_v not in pcb.vias and pre_s not in pcb.segments)
    check('the copper the kept result shares (the main prefix) stays',
          main_v in pcb.vias and main_s in pcb.segments)
    check("the board's own copper and the kept result's unblock stay",
          board_v in pcb.vias and board_s in pcb.segments and kept_v in pcb.vias)
    check('the copper epoch moved', pcb._copper_epoch == 8, str(pcb._copper_epoch))
    lists = (list(pcb.vias), list(pcb.segments))
    helper(pcb, None)
    helper(pcb, {'new_segments': [], 'new_vias': [loose_v]})
    check('nothing on the board to take back: no change, epoch unmoved',
          (pcb.vias, pcb.segments) == lists and pcb._copper_epoch == 8)


def boxed_board():
    h = 0.5        # no 0.6 via fits inside the box, a fab-floor one does
    x, y = P1
    box = [(x - h, y - h, x + h, y - h), (x + h, y - h, x + h, y + h),
           (x + h, y + h, x - h, y + h), (x - h, y + h, x - h, y - h)]
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'boxed.kicad_pcb')
        ca.write_board(board, segments=[(a, b, c, d, 0.2, 'F.Cu', 1) for a, b, c, d in box],
                       footprints=[dict(ref='P3', x=20, y=20, net_id=2, net_name='B', size=0.6),
                                   dict(ref='P1', x=x, y=y, net_id=2, net_name='B', size=0.3)])
        pcb = parse_kicad_pcb(board)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.4, clearance=0.15,
                          grid_step=0.05, via_size=0.6, via_drill=0.3)
    return pcb, cfg, build_base_obstacle_map(pcb, cfg, [2], static_base=True)


LINK = ((20.0, 20.0, 'F.Cu', 'pad'), (float(P1[0]), float(P1[1]), 'F.Cu', 'pad'))
# A link whose endpoint layer is not in the routing stack: refused after the
# first link already routed.
BAD = ((20.0, 20.0, 'In1.Cu', 'pad'), (5.0, 5.0, 'In1.Cu', 'pad'))


def oracle_rows():
    section('2. route_oracle_links drops the earlier links of a failed batch')
    pcb, cfg, obs = boxed_board()
    before = {id(v) for v in pcb.vias}
    res, log = quiet(ser.route_oracle_links, pcb, 2, cfg, obs, [LINK, BAD])
    check('fixture: link 1 dropped its via, then the batch failed',
          'Via-in-pad unblock: dropped' in log and bool(res) and res.get('failed'))
    left = [(round(v.x, 3), round(v.y, 3)) for v in pcb.vias if id(v) not in before]
    check('pcb_data holds no via of the dropped links', not left, str(left))

    section('3. control: the link alone succeeds and keeps its via')
    pcb, cfg, obs = boxed_board()
    before = {id(v) for v in pcb.vias}
    res, log = quiet(ser.route_oracle_links, pcb, 2, cfg, obs, [LINK])
    new = [v for v in pcb.vias if id(v) not in before]
    carried = {id(v) for v in (res or {}).get('new_vias') or []}
    check('the link routes through its unblock via, committed and carried',
          'Via-in-pad unblock: dropped' in log and bool(res) and not res.get('failed')
          and new and all(id(v) in carried for v in new),
          str([(round(v.x, 3), round(v.y, 3), id(v) in carried) for v in new]))


@contextlib.contextmanager
def patched(mod, **attrs):
    real = {k: getattr(mod, k) for k in attrs}
    for k, v in attrs.items():
        setattr(mod, k, v)
    try:
        yield
    finally:
        for k, v in real.items():
            setattr(mod, k, v)


def stub_switch_rows():
    section('4. Phase 3 stub-switch retry: a dropped retry takes its via back')
    import phase3_routing
    import stub_layer_switching as sls

    def run(failed_after):
        main_v = Via(x=1, y=1, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=2)
        tap_v = Via(x=2, y=1, size=0.3, drill=0.15, layers=['F.Cu', 'B.Cu'], net_id=2)
        pcb = SimpleNamespace(vias=[main_v, tap_v], segments=[], _copper_epoch=0)
        failed = [{'x': 5.0, 'y': 5.0, 'component_ref': 'P1', 'pad_number': '1'}]
        completed = {'new_segments': [], 'new_vias': [main_v, tap_v],
                     'failed_pads_info': failed}
        retry_v = Via(x=5, y=5, size=0.3, drill=0.15, layers=['F.Cu', 'B.Cu'], net_id=2)

        def taps(pcb_data, net_id, config, obstacles, main_result, **k):
            pcb_data.vias.append(retry_v)                 # the #803 commit
            return {'new_segments': list(main_result['new_segments']),
                    'new_vias': list(main_result['new_vias']) + [retry_v],
                    'failed_pads_info': failed if failed_after else []}

        state = SimpleNamespace(all_segment_modifications=[], all_swap_vias=[])
        no_switch = dict(foreign_stub_registry=lambda *a, **k: {},
                         switch_boxed_stub_near=lambda *a, **k: ([], [], 'F.Cu', 'B.Cu'),
                         revert_stub_layer_switch=lambda *a, **k: None)
        with patched(sls, **no_switch), patched(ser, route_multipoint_taps=taps):
            got, _ = quiet(phase3_routing._phase3_stub_switch_retry, 2, completed, pcb,
                           None, state, lambda: None, 0, 0, 0)
        return pcb, got, (main_v, tap_v, retry_v)

    pcb, got, (main_v, tap_v, retry_v) = run(failed_after=True)
    on = [id(v) for v in pcb.vias]
    check('fixture: the retry reconnected nothing and was dropped', got is None)
    check("the dropped retry's via is taken back", id(retry_v) not in on)
    check('the retried pass keeps all of its copper',
          id(main_v) in on and id(tap_v) in on)
    pcb, got, (main_v, tap_v, retry_v) = run(failed_after=False)
    on = [id(v) for v in pcb.vias]
    check('control: a retry that reconnects the pad is kept with its via',
          got is not None and id(retry_v) in on and id(main_v) in on and id(tap_v) in on)


def main():
    fab_tiers.set_escalation_policy('fab')
    helper_rows()
    oracle_rows()
    stub_switch_rows()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
