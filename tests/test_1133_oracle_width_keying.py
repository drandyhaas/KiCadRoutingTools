#!/usr/bin/env python3
"""#1133: the oracle's per-net WIDTH maps travel by net NAME, and the copper
it hands back carries the CALLER's net ids -- with no KiCad at all.

On the GUI path the oracle parses a pcbnew save of the live board, which
numbers its nets afresh (flat_hierarchy: 94 of 111 nets move), while the
engine run's maps -- power_net_widths, net_track_widths, net_layer_widths --
were keyed by the run's ids (pcbnew netcodes). They landed on other nets.
And the copper the oracle returned carried the PARSE's ids, which the GUI
then applied with SetNetCode. What each case pins:

* `oracle_net_widths_by_name` / `rekey_by_name` carry all three maps across
  a renumbering, and the re-keyed config answers `get_net_track_width` for
  the right net;
* route.py's oracle legs and its GUI payload carry the maps BY NAME (no
  id-keyed width map is put on an oracle config or in the payload), the
  GUI applier takes them and swig_gui forwards them, and the two GUI-facing
  consumers hand the oracle the caller's {name: id} for the way back;
* a stubbed oracle round on a board whose parse renumbers the nets: the
  width lands on the named net by name (and an id-keyed map, the old
  channel, demonstrably lands on another -- the negative control);
* the copper the oracle lays, and the stranded fragment it deletes, come
  back on the caller's ids when `net_ids_by_name` is given, on the parse's
  ids when it is not, and an object whose net the caller lacks is dropped.

tests/gui_parity/test_1133_staged_save_widths.py runs the GUI consumer on a
real pcbnew save of flat_hierarchy (needs KiCad's python).

    python3 tests/test_1133_oracle_width_keying.py [case-substring ...]
"""
import ast
import contextlib
import io
import os
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))


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


WIDTH_MAPS = ('power_net_widths', 'net_track_widths', 'net_layer_widths')


# ---------------------------------------------------------------- the maps
def test_widths_cross_a_renumbering_by_name():
    from kicad_oracle import (ORACLE_WIDTH_MAPS, oracle_net_widths_by_name,
                              oracle_net_ids_by_name, rekey_by_name)
    from routing_config import GridRouteConfig
    assert set(WIDTH_MAPS) <= set(ORACLE_WIDTH_MAPS), ORACLE_WIDTH_MAPS
    run = GridRouteConfig(clearance=0.2, track_width=0.15)
    run.power_net_widths = {4: 0.5}
    run.net_track_widths = {9: 0.3}
    run.net_layer_widths = {7: {'F.Cu': 0.25, 'B.Cu': 0.35}}
    run_nets = {4: NS(name='/P'), 9: NS(name='/S'), 7: NS(name='/Z'),
                5: NS(name='/X')}
    by_name = oracle_net_widths_by_name(run, run_nets)
    assert by_name == {'power_net_widths': {'/P': 0.5},
                       'net_track_widths': {'/S': 0.3},
                       'net_layer_widths': {'/Z': {'F.Cu': 0.25,
                                                   'B.Cu': 0.35}}}, by_name
    only = oracle_net_widths_by_name(run, run_nets,
                                     fields=('power_net_widths',))
    assert only == {'power_net_widths': {'/P': 0.5}}, only
    assert oracle_net_widths_by_name(GridRouteConfig(), run_nets) == {}
    # the staged save numbers the same nets differently
    staged = {1: NS(name='/S'), 2: NS(name='/X'), 3: NS(name='/P'),
              4: NS(name='/Z')}
    cfg = GridRouteConfig(clearance=0.2, track_width=0.15)
    for f, m in by_name.items():
        setattr(cfg, f, rekey_by_name(m, staged))
    got = {nid: cfg.get_net_track_width(nid, 'B.Cu') for nid in staged}
    assert got == {1: 0.3, 2: 0.15, 3: 0.5, 4: 0.35}, got
    ids = oracle_net_ids_by_name(run_nets)
    assert ids == {'': 0, '/P': 4, '/S': 9, '/Z': 7, '/X': 5}, ids
    print("  PASS: /P's power width, /S's class width and /Z's per-layer "
          "width follow their NAMES onto the staged ids")


# ---------------------------------------------------------------- the code
def test_oracle_legs_and_payload_carry_names():
    route = _tree(os.path.join('py_router', 'route.py'))
    # no oracle config is built with an id-keyed width map
    oracle_cfgs = [n for n in ast.walk(route) if isinstance(n, ast.Assign)
                   and isinstance(n.value, ast.Call)
                   and getattr(n.value.func, 'id', '') == 'GridRouteConfig'
                   and any(isinstance(t, ast.Name)
                           and t.id in ('_ocfg', '_cap_cfg')
                           for t in n.targets)]
    assert len(oracle_cfgs) == 2, len(oracle_cfgs)
    for a in oracle_cfgs:
        bad = [k.arg for k in a.value.keywords if k.arg in WIDTH_MAPS]
        assert not bad, (a.lineno, bad)
    # every route.py oracle call that had widths passes them by name
    aliases = {'oracle_reconnect'} | {
        a.asname for n in ast.walk(route) if isinstance(n, ast.ImportFrom)
        for a in n.names if a.name == 'oracle_reconnect' and a.asname}
    calls = [c for nm in aliases for c in _calls(route, nm)]
    assert len(calls) == 4, len(calls)
    for c in calls:
        k = _kw(c, 'net_widths_by_name')
        assert k is not None and not isinstance(
            k.value, (ast.Constant, ast.Dict)), (c.lineno, ast.unparse(c))
    # the finalize leg (the one the GUI runs in-run) hands the way back
    fin = [c for c in calls if len(c.args) > 2
           and ast.unparse(c.args[2]) == '_ocfg']
    assert len(fin) == 1 and _kw(fin[0], 'net_ids_by_name') is not None, \
        [ast.unparse(c) for c in fin]
    # the GUI payload: by name, and no id-keyed width map left in it
    payloads = [n for n in ast.walk(route) if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Subscript)
                        and isinstance(t.slice, ast.Constant)
                        and t.slice.value == 'plane_finalize_oracle'
                        for t in n.targets)]
    assert len(payloads) == 1
    keys = {k.value for k in payloads[0].value.keys
            if isinstance(k, ast.Constant)}
    assert 'net_widths_by_name' in keys and not (set(WIDTH_MAPS) & keys), \
        sorted(keys)
    # the applier
    # ipc-migration: the applier is kicad_ipc_adapter.apply_oracle_reconnect
    # and routing_dialog calls it (there is no gui_utils.
    # run_kicad_oracle_on_live_board and no swig_gui). It computes the way
    # back from its pcb_data inline, so that one is a call, not a name.
    gui = _tree('kicad_ipc_adapter.py')
    fn = next(n for n in ast.walk(gui) if isinstance(n, ast.FunctionDef)
              and n.name == 'apply_oracle_reconnect')
    params = {a.arg for a in fn.args.args + fn.args.kwonlyargs}
    assert 'net_widths_by_name' in params and not (set(WIDTH_MAPS) & params), \
        sorted(params)
    inner = list(_calls(fn, 'oracle_reconnect'))
    assert len(inner) == 1
    k = _kw(inner[0], 'net_widths_by_name')
    assert k is not None and isinstance(k.value, ast.Name), \
        ('net_widths_by_name', ast.unparse(inner[0]))
    k = _kw(inner[0], 'net_ids_by_name')
    assert k is not None and 'oracle_net_ids_by_name' in ast.unparse(k.value), \
        ('net_ids_by_name', ast.unparse(inner[0]))
    swig = _tree(os.path.join('kicad_routing_plugin', 'routing_dialog.py'))
    sc = list(_calls(swig, 'apply_oracle_reconnect'))
    assert sc
    for c in sc:
        k = _kw(c, 'net_widths_by_name')
        assert k is not None and "'net_widths_by_name'" in ast.unparse(
            k.value), ast.unparse(c)
        assert not any(_kw(c, f) for f in WIDTH_MAPS), ast.unparse(c)
    print(f"  PASS: {len(calls)} route.py oracle call(s) pass the widths by "
          f"name; the payload, the applier and swig_gui carry only the "
          f"by-name map; the in-run leg and the applier pass the way back")


# ------------------------------------------------- the oracle, end to end
def _v10_board(path, segments=(), pads=()):
    """KiCad-10 style (nets by NAME): the parse numbers nets by first
    appearance, so `pads` order decides the ids."""
    fps = ''.join(f'''
  (footprint "T:P" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "{n}")))'''
                  for ref, x, y, n in pads)
    segs = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width {w}) '
                   f'(layer "{l}") (net "{n}"))'
                   for a, b, c, d, w, l, n in segments)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts")){fps}{segs}
)
''')


#: The CALLER's numbering (A=1, B=2, C=3, as a pcbnew netcode table would
#: have it); the board below parses as B=1, A=2, C=3.
CALLER_IDS = {'': 0, 'A': 1, 'B': 2, 'C': 3}


def _run_oracle(widths_by_name=None, ids_by_name=None, caller_widths=None):
    """One stubbed round. Links: A's 0.6 mm same-layer gap (the routers are
    stubbed to fail, so it reaches the sliver weld, which lays it) and C's
    pad-to-fragment link (the fragment is pad-less, so it is deleted).
    Returns (spied configs, oracle result, parsed ids)."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from routing_config import GridRouteConfig
    from kicad_parser import parse_kicad_pcb
    seen = []
    calls = {'n': 0}

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        calls['n'] += 1
        if calls['n'] > 1:
            return []
        return [('A', (10.0, 10.0, 'F.Cu', 'track'),
                 (10.6, 10.0, 'F.Cu', 'track')),
                ('C', (32.0, 32.0, 'F.Cu', 'pad'),
                 (26.0, 25.0, 'F.Cu', 'track'))]

    real_bbo = plane_region_connector.build_base_obstacles

    def spy_bbo(*a, **k):
        seen.append(k.get('config'))
        return real_bbo(*a, **k)

    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected,
             kicad_exact_fill.refill_islands,
             plane_region_connector.build_base_obstacles,
             plane_region_connector.route_plane_connection_wide,
             net_rescue._attempt_edge)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    kicad_exact_fill.refill_islands = lambda *a, **k: None
    plane_region_connector.build_base_obstacles = spy_bbo
    plane_region_connector.route_plane_connection_wide = \
        lambda *a, **k: (None, 0)
    net_rescue._attempt_edge = lambda *a, **k: (None, None)
    try:
        with tempfile.TemporaryDirectory() as td:
            board = os.path.join(td, 'b.kicad_pcb')
            _v10_board(board,
                       segments=[(5, 10, 10, 10, 0.2, 'F.Cu', 'A'),
                                 (10.6, 10, 15, 10, 0.2, 'F.Cu', 'A'),
                                 (5, 14, 15, 14, 0.2, 'F.Cu', 'B'),
                                 (25, 25, 27, 25, 0.2, 'F.Cu', 'C')],
                       pads=[('R2', 30, 30, 'B'), ('R1', 5, 10, 'A'),
                             ('R3', 15, 10, 'A'), ('R4', 32, 32, 'C')])
            ids = {n.name: nid for nid, n in
                   parse_kicad_pcb(board).nets.items() if n.name}
            caller = GridRouteConfig(clearance=0.2, track_width=0.2,
                                     via_size=0.6, via_drill=0.3,
                                     layers=['F.Cu', 'B.Cu'],
                                     board_edge_clearance=0.2)
            if caller_widths:
                caller.power_net_widths = dict(caller_widths)
            with contextlib.redirect_stdout(io.StringIO()):
                res = kicad_oracle.oracle_reconnect(
                    board, ['A', 'C'], caller, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.2, max_rounds=1,
                    net_widths_by_name=widths_by_name,
                    net_ids_by_name=ids_by_name)
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.build_base_obstacles,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge) = saved
    return seen, res, ids


def test_the_width_lands_on_the_named_net():
    seen, _res, ids = _run_oracle(
        widths_by_name={'power_net_widths': {'A': 0.5}})
    assert ids == {'B': 1, 'A': 2, 'C': 3}, \
        f"fixture: the parse must renumber the nets, got {ids}"
    assert seen, 'the oracle never built an obstacle map'
    cfg = seen[0]
    got = {n: cfg.get_net_track_width(ids[n], 'F.Cu') for n in ids}
    assert got == {'A': 0.5, 'B': 0.2, 'C': 0.2}, got
    # NEGATIVE CONTROL: the old channel -- the caller's id-keyed map put on
    # the config -- lands on the parse's id 1, which is B, not A
    seen2, _r, _i = _run_oracle(caller_widths={CALLER_IDS['A']: 0.5})
    got2 = {n: seen2[0].get_net_track_width(ids[n], 'F.Cu') for n in ids}
    assert got2['B'] == 0.5 and got2['A'] == 0.2, \
        f"negative control: the id-keyed map should land on B: {got2}"
    print(f"  PASS: by name, A's 0.5 lands on A ({got}); the id-keyed map "
          f"lands on B ({got2})")


def test_returned_copper_comes_back_on_the_callers_ids():
    _s, res, ids = _run_oracle(ids_by_name=dict(CALLER_IDS))
    new = list(res.get('new_segments') or [])
    rem = list(res.get('removed_segments') or [])
    assert new and rem, f"fixture: expected a weld and a deletion: {res}"
    assert {s.net_id for s in new} == {CALLER_IDS['A']}, \
        [(s.net_id, s.start_x) for s in new]
    assert {s.net_id for s in rem} == {CALLER_IDS['C']}, \
        [(s.net_id, s.start_x) for s in rem]
    # without the way back: the parse's ids, as before
    _s, res0, _i = _run_oracle()
    assert {s.net_id for s in res0['new_segments']} == {ids['A']}
    assert {s.net_id for s in res0['removed_segments']} == {ids['C']}
    # a net the caller does not carry: its object is dropped, not guessed
    _s, res1, _i = _run_oracle(ids_by_name={'': 0, 'A': 1, 'B': 2})
    assert {s.net_id for s in res1['new_segments']} == {1}
    assert not res1['removed_segments'], res1['removed_segments']
    print(f"  PASS: the weld comes back on A={CALLER_IDS['A']} (parse "
          f"{ids['A']}), the deleted fragment on C={CALLER_IDS['C']}; an "
          f"unknown net's object is dropped")


TESTS = [test_widths_cross_a_renumbering_by_name,
         test_oracle_legs_and_payload_carry_names,
         test_the_width_lands_on_the_named_net,
         test_returned_copper_comes_back_on_the_callers_ids]


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
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
