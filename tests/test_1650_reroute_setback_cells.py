#!/usr/bin/env python3
"""#1650: the reroute loop's diff-pair setback fallback reads cells of ITS
OWN pass, or none.

`run_reroute_loop`'s diff-pair branch filled `fwd_cells` / `bwd_cells` only
inside `if routed_net_paths and result:`, and the setback fallback after the
rip ladder read them whenever `result` was set. `routed_net_paths` can be
empty there: a routed pair whose result carries no p_path/n_path (a direct
hybrid) never enters it, and neither does a pair restored without its paths.
So:

  1. The FIRST failed pair of the loop with an empty routed_net_paths raised
     UnboundLocalError and the run wrote no board. Now it is counted failed
     and the setback fallback does not run -- as in the main diff-pair loop,
     whose setback fallback also runs only inside its routed_net_paths
     branch.
  2. A LATER such pair read the cells an EARLIER pair's pass left in the
     variables, and on that evidence ran the fallback layer swap with the
     earlier pair's blocked cells. Now it reads none.

The router and the obstacle builders are stubbed: the loop's own control
flow is what is under test.

    python3 tests/test_1650_reroute_setback_cells.py
"""
import os
import sys
import traceback

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from routing_config import GridRouteConfig, DiffPairNet     # noqa: E402
from routing_state import RoutingState                     # noqa: E402
from synth import make_pcb, make_net                       # noqa: E402
import reroute_loop                                        # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{': ' + detail if detail else ''}")
    if not ok:
        failures.append(label)


def pair(base, p_id, n_id):
    return DiffPairNet(base_name=base, p_net_id=p_id, n_net_id=n_id,
                       p_net_name=f"{base}_P", n_net_name=f"{base}_N")


def make_state(pairs):
    nets = {}
    for p in pairs:
        nets[p.p_net_id] = make_net(p.p_net_id, p.p_net_name)
        nets[p.n_net_id] = make_net(p.n_net_id, p.n_net_name)
    nets[99] = make_net(99, 'OTHER')
    cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                          via_drill=0.3, layers=['F.Cu', 'B.Cu'],
                          grid_step=0.05)
    cfg.diff_pair_hybrid_escape = False   # the stage after the swap; not under test
    st = RoutingState(pcb_data=make_pcb(nets=nets), config=cfg)
    st.layer_map = {'F.Cu': 0, 'B.Cu': 1}
    st.enable_layer_switch = True
    st.reroute_queue = [('diff_pair', p.base_name, p) for p in pairs]
    return st


def run(state, results_by_pair, on_route=None):
    """Drive the loop with the router answering `results_by_pair[name]`;
    returns (loop return or the exception, the cells each swap was given)."""
    swaps = []
    saved = {k: getattr(reroute_loop, k) for k in (
        'route_diff_pair_with_obstacles', 'build_diff_pair_obstacles',
        'get_diff_pair_endpoints', 'try_fallback_layer_swap')}

    def route(pcb, p, *a, **k):
        if on_route:
            on_route(p)
        return dict(results_by_pair[p.base_name])

    def swap(pcb, p, name, cfg, fwd, bwd, *a, **k):
        swaps.append((name, list(fwd), list(bwd)))
        return False, None, None, None

    reroute_loop.route_diff_pair_with_obstacles = route
    reroute_loop.build_diff_pair_obstacles = lambda *a, **k: (None, [])
    reroute_loop.get_diff_pair_endpoints = lambda *a, **k: ([], [], None)
    reroute_loop.try_fallback_layer_swap = swap
    try:
        try:
            out = reroute_loop.run_reroute_loop(state)
        except Exception as e:                     # noqa: BLE001
            out = e
            traceback.print_exc()
    finally:
        for k, v in saved.items():
            setattr(reroute_loop, k, v)
    return out, swaps


SETBACK = {'failed': True, 'iterations': 0, 'iterations_forward': 0,
           'iterations_backward': 0, 'blocked_cells_forward': [(5, 5, 0)],
           'blocked_cells_backward': []}


def t_first_pair_with_no_paths():
    print("1. first failed pair, routed_net_paths empty")
    a = pair('A', 1, 2)
    st = make_state([a])
    out, swaps = run(st, {'A': SETBACK})
    if isinstance(out, UnboundLocalError):
        check('loop completes', False, f'UnboundLocalError: {out}')
        return
    if isinstance(out, BaseException):
        # Anything else is THIS TEST breaking, not the defect it tests.
        raise AssertionError(f"BROKEN TEST: the loop died on {out!r}, which "
                             f"is not the defect under test") from out
    check('loop completes', True)
    successful, failed = out[0], out[1]
    check('the pair is counted failed', (successful, failed) == (0, 1),
          f'successful={successful} failed={failed}')
    check('no setback fallback without routed paths (main-loop parity)',
          swaps == [], f'swaps={swaps}')


def t_later_pair_does_not_read_earlier_cells():
    print("2. a later pair with no paths does not inherit an earlier pair's cells")
    a, b = pair('A', 1, 2), pair('B', 3, 4)
    st = make_state([a, b])
    st.routed_net_paths[99] = [(0, 0, 0), (1, 0, 0)]   # A's pass enters the branch
    # A: a non-setback failure whose BACKWARD search left blocked cells; its
    # forward search was the faster, so blocker analysis gets none.
    a_res = {'failed': True, 'iterations': 10, 'iterations_forward': 5,
             'iterations_backward': 5, 'blocked_cells_forward': [],
             'blocked_cells_backward': [(7, 7, 1)]}
    # B: a setback-shaped failure with NO blocked cells of its own.
    b_res = {'failed': True, 'iterations': 0, 'iterations_forward': 0,
             'iterations_backward': 0}

    def on_route(p):
        if p.base_name == 'B':
            st.routed_net_paths.clear()        # nothing B can analyse

    out, swaps = run(st, {'A': a_res, 'B': b_res}, on_route)
    if isinstance(out, BaseException):
        raise AssertionError(f"BROKEN TEST: the loop died on {out!r}") from out
    check('both pairs counted failed', out[1] == 2, f'failed={out[1]}')
    check("B's fallback is not run on A's blocked cells", swaps == [],
          f'swaps={swaps}')


if __name__ == '__main__':
    t_first_pair_with_no_paths()
    t_later_pair_does_not_read_earlier_cells()
    if failures:
        print(f"FAILED: {failures}")
        sys.exit(1)
    print("PASS")
