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
* every `oracle_reconnect` call in py_router/ and kicad_routing_plugin/,
  aliased imports included, passes the keyword with a real value (an AST
  walk, not a grep);
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


def _oracle_aliases(tree):
    """Every local name `oracle_reconnect` is imported as in `tree`."""
    names = {'oracle_reconnect'}
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom):
            for a in n.names:
                if a.name == 'oracle_reconnect' and a.asname:
                    names.add(a.asname)
    return names


def test_every_oracle_call_passes_the_map():
    """Every call -- including the ones imported under another name (route.py's
    #678 pour-promise weld and #589 re-audit) -- passes the map, and passes a
    real one: a constant `{}` / `None` there is the map dropped."""
    seen = []
    for rel in SOURCES:
        tree = _tree(rel)
        for name in sorted(_oracle_aliases(tree)):
            for c in _calls(tree, name):
                seen.append((rel, c.lineno, name,
                             _kw(c, 'net_clearances_by_name')))
    assert len(seen) >= 6, [(r, ln, nm) for r, ln, nm, _k in seen]
    missing = [(r, ln, nm) for r, ln, nm, k in seen if k is None]
    assert not missing, missing
    const = [(r, ln, nm) for r, ln, nm, k in seen
             if isinstance(k.value, (ast.Constant, ast.Dict))]
    assert not const, const
    print(f"  PASS: {len(seen)} oracle_reconnect call(s) (aliases "
          f"included), every one passes a real net_clearances_by_name")


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


def test_the_escalation_and_the_cap_config_carry_the_rules():
    """The weld escalation builds its own obstacle map (`_attempt_edge`'s
    fifth argument is the class map it prices foreign nets with), and
    route.py's #678 cap config installs the layer rules WITH the board, so
    they are read for the board's own copper layers, not the default two."""
    orc = _tree(os.path.join('py_router', 'kicad_oracle.py'))
    esc = list(_calls(orc, '_attempt_edge'))
    assert len(esc) >= 2, len(esc)
    for c in esc:
        assert len(c.args) >= 5, (c.lineno, ast.unparse(c))
        arg = c.args[4]
        assert not isinstance(arg, ast.Constant), (c.lineno, ast.unparse(c))
        assert 'net_clearances' in ast.unparse(arg), (c.lineno,
                                                      ast.unparse(arg))
    route = _tree(os.path.join('py_router', 'route.py'))
    cap = [c for c in _calls(route, 'install_layer_clearances')
           if c.args and isinstance(c.args[0], ast.Name)
           and c.args[0].id == '_cap_cfg']
    assert len(cap) == 1, len(cap)
    board = (cap[0].args[3] if len(cap[0].args) > 3
             else getattr(_kw(cap[0], 'pcb_data'), 'value', None))
    assert board is not None and not (isinstance(board, ast.Constant)
                                      and board.value is None), \
        ast.unparse(cap[0])
    print(f"  PASS: {len(esc)} escalation call(s) pass the class map; the "
          f"cap config installs its layer rules with the board")


TESTS = [test_the_map_is_rekeyed_by_name,
         test_every_oracle_call_passes_the_map,
         test_the_gui_payload_carries_it_and_both_fronts_forward_it,
         test_repair_planes_main_passes_the_published_map,
         test_the_escalation_and_the_cap_config_carry_the_rules]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # a misspelt witness filter would otherwise pass vacuously
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
