#!/usr/bin/env python3
"""bga_fanout's layer rebalance with --layer-costs: a pair the greedy pass put
on a costlier layer is left where it is.

With --layer-costs, generate_bga_fanout balances only across the cheapest
tier (`balance_layers`), so an escape that overflowed onto a costlier layer
is on a layer rebalance_layers was not given. Phase 1 skipped such routes;
phase 2 (the edge/half-edge pairs) indexed its per-layer count by
`route.layer` and raised KeyError: 'In1.Cu' -- the whole fanout of the zynq's
U1 died on `--layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 6 2.5 1`.

Checks: phase 2 runs over an edge pair on a balanced layer and one on an
unbalanced layer without raising, and the unbalanced pair stays on its layer.

    python3 tests/test_bga_layer_balance_overflow.py
"""
import os
import sys
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from bga_fanout.layer_balance import rebalance_layers   # noqa: E402
from bga_fanout.types import FanoutRoute                # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f' -- {detail}' if detail else ''))
    if not ok:
        failures.append(label)


def route(net_id, x, y, layer, pair, is_p):
    pad = SimpleNamespace(net_id=net_id, global_x=x, global_y=y)
    return FanoutRoute(pad=pad, pad_pos=(x, y), stub_end=(x, y - 0.4), exit_pos=(x, y - 1.0),
                       escape_dir='up', is_edge=True, layer=layer, pair_id=pair, is_p=is_p)


# an edge pair on F.Cu (balanced) and one the greedy pass put on In1.Cu (overflow onto a costlier layer)
routes = [route(1, 0.0, 0.0, 'F.Cu', 'A', True), route(2, 0.8, 0.0, 'F.Cu', 'A', False),
          route(3, 4.0, 0.0, 'In1.Cu', 'B', True), route(4, 4.8, 0.0, 'In1.Cu', 'B', False)]
try:
    moved = rebalance_layers(routes, [], [], ['F.Cu', 'B.Cu'], 0.1)
    check('rebalance over a pair on a layer outside the balance set runs', True, f'{moved} moved')
except KeyError as e:
    check('rebalance over a pair on a layer outside the balance set runs', False, f'KeyError {e}')
check('the overflow pair stays on its layer', [r.layer for r in routes[2:]] == ['In1.Cu', 'In1.Cu'],
      str([r.layer for r in routes[2:]]))

print(f'\n{len(failures)} failure(s)')
sys.exit(1 if failures else 0)
