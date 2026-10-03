#!/usr/bin/env python3
"""#980: the KiCad oracle receives the run's net-class map, by NAME, on both
fronts.

The oracle's #649 stitching-via admission and its sliver weld price foreign
copper through `config.pair_clearance`, but no config the oracle received
carried a class map, so the pricing could not see a class. Every caller now
passes `net_clearances_by_name` -- keyed by NAME, because the oracle re-parses
its board every round and, on the GUI path, that board is a pcbnew save whose
nets are numbered afresh. What each case pins:

* `_oracle_class_map` re-keys a name map onto the ids of the board it is
  given, whatever ids the caller had;
* every `oracle_reconnect(` call in py_router/ and kicad_routing_plugin/
  passes the keyword (an AST walk, not a grep);
* route.py's GUI payload carries the map, `run_kicad_oracle_on_live_board`
  takes it and forwards it, and swig_gui hands it over from the payload;
* repair_planes' main() passes the map its engine published.

    python3 tests/test_980_oracle_class_map.py
"""
import ast
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

SOURCES = [os.path.join('py_router', f) for f in
           sorted(os.listdir(os.path.join(ROOT, 'py_router')))
           if f.endswith('.py')] + \
          [os.path.join('kicad_routing_plugin', f) for f in
           sorted(os.listdir(os.path.join(ROOT, 'kicad_routing_plugin')))
           if f.endswith('.py')]


def _tree(rel):
    with open(os.path.join(ROOT, rel), encoding='utf-8') as fh:
        return ast.parse(fh.read(), rel)


def _calls(tree, name):
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            if (isinstance(f, ast.Name) and f.id == name) or \
                    (isinstance(f, ast.Attribute) and f.attr == name):
                yield n


def _kw(call, name):
    return next((k for k in call.keywords if k.arg == name), None)


def test_the_map_is_rekeyed_by_name():
    from kicad_oracle import _oracle_class_map
    from routing_config import GridRouteConfig
    run = GridRouteConfig(clearance=0.2)
    run.set_net_clearances({4: 0.4, 9: 0.3}, routed_net_ids=[4])
    caller_nets = {4: NS(name='/HV'), 9: NS(name='/MID'), 5: NS(name='/B')}
    by_name = run.net_clearances_by_name(caller_nets)
    assert by_name == {'/HV': 0.4, '/MID': 0.3}, by_name
    # the staged save numbers the same nets differently
    staged = NS(nets={1: NS(name='/B'), 2: NS(name='/MID'),
                      3: NS(name='/HV'), 0: NS(name='')})
    assert _oracle_class_map(staged, by_name) == {3: 0.4, 2: 0.3}
    assert _oracle_class_map(staged, {}) == {}
    print("  PASS: /HV (id 4 in the run) is id 3 on the staged board and "
          "keeps its 0.4")


def test_every_oracle_call_passes_the_map():
    seen = []
    for rel in SOURCES:
        for c in _calls(_tree(rel), 'oracle_reconnect'):
            seen.append((rel, c.lineno, _kw(c, 'net_clearances_by_name')))
    assert len(seen) >= 4, seen
    missing = [(r, ln) for r, ln, k in seen if k is None]
    assert not missing, missing
    print(f"  PASS: {len(seen)} oracle_reconnect call(s), every one passes "
          f"net_clearances_by_name")


def test_the_gui_payload_carries_it_and_both_fronts_forward_it():
    route = _tree(os.path.join('py_router', 'route.py'))
    payloads = [n for n in ast.walk(route) if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Subscript)
                        and isinstance(t.slice, ast.Constant)
                        and t.slice.value == 'plane_finalize_oracle'
                        for t in n.targets)]
    assert len(payloads) == 1, len(payloads)
    keys = {k.value for k in payloads[0].value.keys
            if isinstance(k, ast.Constant)}
    assert 'net_clearances_by_name' in keys, sorted(keys)

    gui = _tree(os.path.join('kicad_routing_plugin', 'gui_utils.py'))
    fn = next(n for n in ast.walk(gui) if isinstance(n, ast.FunctionDef)
              and n.name == 'run_kicad_oracle_on_live_board')
    params = {a.arg for a in fn.args.args + fn.args.kwonlyargs}
    assert 'net_clearances_by_name' in params, params
    inner = [c for c in _calls(fn, 'oracle_reconnect')]
    assert len(inner) == 1
    fwd = _kw(inner[0], 'net_clearances_by_name')
    assert isinstance(fwd.value, ast.Name) and \
        fwd.value.id == 'net_clearances_by_name', ast.dump(fwd.value)

    swig = _tree(os.path.join('kicad_routing_plugin', 'swig_gui.py'))
    calls = list(_calls(swig, 'run_kicad_oracle_on_live_board'))
    assert calls, 'swig_gui no longer calls the live oracle'
    for c in calls:
        k = _kw(c, 'net_clearances_by_name')
        assert k is not None and 'net_clearances_by_name' in ast.unparse(
            k.value), (c.lineno, ast.unparse(c))
    print("  PASS: the payload carries the map; gui_utils takes and forwards "
          "it; swig_gui hands it over")


def test_repair_planes_main_passes_the_published_map():
    rp = _tree(os.path.join('py_router', 'repair_planes.py'))
    main = next(n for n in rp.body if isinstance(n, ast.FunctionDef)
                and n.name == 'main')
    calls = list(_calls(main, 'oracle_reconnect'))
    assert len(calls) == 1, len(calls)
    k = _kw(calls[0], 'net_clearances_by_name')
    assert isinstance(k.value, ast.Name) and \
        k.value.id == 'LAST_NET_CLEARANCES_BY_NAME', ast.unparse(k.value)
    import repair_planes
    assert repair_planes.LAST_NET_CLEARANCES_BY_NAME == {}
    print("  PASS: repair_planes main() passes LAST_NET_CLEARANCES_BY_NAME")


TESTS = [test_the_map_is_rekeyed_by_name,
         test_every_oracle_call_passes_the_map,
         test_the_gui_payload_carries_it_and_both_fronts_forward_it,
         test_repair_planes_main_passes_the_published_map]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
