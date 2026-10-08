"""The plane finalize's reroute sub-runs route by the step's own knobs.

The in-run finalize (#562) reconnects rip casualties and joins regions through
nested batch_route calls in repair_planes. Those calls named their geometry
and nothing else, so they fell back to batch_route's own defaults: a via cost
of 50 where the CLI and GUI route at 75, `inside_out` ordering where they use
`mps`, every BGA zone re-armed on a step run with --no-bga-zones, keepouts
and guide corridors off, and the step's soft costs and keep-away rules gone.
The region-join sub-run also routed at a board-edge clearance of 0.

Rows:
  - every forwarded knob is a real batch_route parameter, and geometry and
    scope are not among them;
  - the parent's values win, including BGA zones and the iteration budget,
    and the pending-casualty ghost kwargs win over the parent's avoidance
    cost; without a parent the engine's own settings stand in;
  - both finalize call sites hand the knobs down, and all three sub-runs
    take them (the region join with the board-edge floor too);
  - batch_route's and batch_route_diff_pairs' own defaults are the CLI's.

    python3 tests/test_finalize_route_knobs.py
"""

import inspect
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import repair_planes  # noqa: E402
import route  # noqa: E402
import route_diff  # noqa: E402
import routing_defaults as defaults  # noqa: E402


def test_knobs_are_batch_route_parameters():
    params = inspect.signature(route.batch_route).parameters
    unknown = [k for k in route._FINALIZE_ROUTE_KNOBS if k not in params]
    assert not unknown, unknown
    for geometry in ('track_width', 'clearance', 'via_size', 'via_drill',
                     'layers', 'grid_step', 'net_names', 'pcb_data'):
        assert geometry not in route._FINALIZE_ROUTE_KNOBS, geometry
    call = {k: object() for k in params}
    got = route._finalize_route_knobs(call)
    assert set(got) == set(route._FINALIZE_ROUTE_KNOBS), set(got) ^ set(
        route._FINALIZE_ROUTE_KNOBS)
    assert all(got[k] is call[k] for k in got)


def test_sub_run_kwargs_precedence():
    parent = {'via_cost': 75, 'disable_bga_zones': ['U1'], 'max_iterations': 9,
              'ripped_route_avoidance_cost': 0.1, 'keep_away': ('A:B:0.5',)}
    ghosts = {'external_ripped_ghosts': {}, 'ripped_route_avoidance_cost': 3.0}
    kw = repair_planes._sub_run_kwargs(parent, False, 200000, ghosts)
    assert kw['disable_bga_zones'] == ['U1'] and kw['max_iterations'] == 9, kw
    assert kw['via_cost'] == 75 and kw['keep_away'] == ('A:B:0.5',), kw
    assert kw['ripped_route_avoidance_cost'] == 3.0, "the ghosts' own cost lost"
    alone = repair_planes._sub_run_kwargs(None, True, 1234, {})
    assert alone == {'disable_bga_zones': [], 'max_iterations': 1234}, alone


def _source(mod):
    return inspect.getsource(mod)


def test_every_sub_run_takes_the_knobs():
    src = _source(route)
    calls = [m.start() for m in re.finditer(r'_rdp_engine\(', src)]
    calls = [c for c in calls if 'import' not in src[c - 40:c]]
    assert len(calls) == 2, calls
    for c in calls:
        body = src[c:c + 4000]          # the call and its argument comments
        assert 'route_knobs=_finalize_route_knobs(_reconcile_kwargs)' in body, \
            src[c:c + 200]
    rsrc = _source(repair_planes.repair_planes)
    subs = [m.start() for m in re.finditer(r'= batch_route\(', rsrc)]
    assert len(subs) == 3, len(subs)
    for c in subs:
        end = rsrc.index('_ghost_kwargs(', c)
        call = rsrc[c:end + 200]
        assert '_sub_run_kwargs(route_knobs, no_bga_zone, max_iterations' in call
        assert 'board_edge_clearance=' in call, call[:300]
        assert 'disable_bga_zones=' not in call and 'max_iterations=' not in call


def test_engine_defaults_are_the_cli_defaults():
    for fn in (route.batch_route, route_diff.batch_route_diff_pairs):
        p = inspect.signature(fn).parameters
        assert p['ordering_strategy'].default == defaults.DEFAULT_ORDERING_STRATEGY, fn
    assert inspect.signature(route.batch_route).parameters['via_cost'].default \
        == defaults.VIA_COST


TESTS = [test_knobs_are_batch_route_parameters, test_sub_run_kwargs_precedence,
         test_every_sub_run_takes_the_knobs, test_engine_defaults_are_the_cli_defaults]


if __name__ == '__main__':
    fails = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
