#!/usr/bin/env python3
"""#1137: the KiCad oracle receives the run's resolved net-class map BY NAME,
re-keys it onto the board it parses, and its admission checks use it.

The oracle re-parses its board every round, and on the GUI path that board is
a pcbnew save whose nets are numbered afresh (#1133: 94 of flat_hierarchy's
111 nets move), so the map crosses as {net name: mm}
(`net_clearances_by_name`) and `kicad_oracle.rekey_by_name` puts it back on
the parsed ids. What each case pins:

* every `oracle_reconnect` call in py_router/ and kicad_routing_plugin/ --
  the aliased imports in route.py (#678 pour-promise weld, #589 re-audit)
  included -- passes the keyword with a real value; the GUI payload carries
  it, gui_utils takes and forwards it, swig_gui hands it over, repair_planes'
  main() passes the map its engine resolved; the escalation's `_attempt_edge`
  calls pass the round's map, and route.py's `_cap_cfg` installs the
  .kicad_dru layer rules as `_ocfg` does (an AST walk, not a grep);
* repair_planes() publishes the map it resolved AFTER the ceiling clamp;
* on a written board whose parse numbers the nets differently from the
  caller, the map lands on the right nets BY NAME, the link's routing floor
  is its own class, and the caller's config is left alone;
* the #649b stitching-via admission and the sliver weld price a foreign
  object at the pair value (class, .kicad_dru layer rule, stack for
  via-via), and with no map they are exactly the flat check;
* end to end: the same sliver weld the oracle lays with no map is refused
  when the foreign net's class (handed in by name) forbids it.

The KiCad link source, the island refill and the routers are stubbed, so no
KiCad is needed.

    python3 tests/test_1137_oracle_class_map.py [case-substring ...]
"""
import ast
import os
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', os.path.join('tests', 'oracle'), 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

SOURCES = [os.path.join('py_router', f) for f in
           sorted(os.listdir(os.path.join(ROOT, 'py_router')))
           if f.endswith('.py')] + \
          [os.path.join('kicad_routing_plugin', f) for f in
           sorted(os.listdir(os.path.join(ROOT, 'kicad_routing_plugin')))
           if f.endswith('.py')] + \
          ['kicad_ipc_adapter.py']   # ipc-migration: the GUI's oracle applier


# --------------------------------------------------------------------- AST
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


def _oracle_aliases(tree):
    names = {'oracle_reconnect'}
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom):
            for a in n.names:
                if a.name == 'oracle_reconnect' and a.asname:
                    names.add(a.asname)
    return names


def test_every_oracle_call_hands_over_the_map():
    seen = []
    for rel in SOURCES:
        tree = _tree(rel)
        for name in sorted(_oracle_aliases(tree)):
            for c in _calls(tree, name):
                seen.append((rel, c.lineno, name,
                             _kw(c, 'net_clearances_by_name')))
    aliased = [s for s in seen if s[2] != 'oracle_reconnect']
    # route.py's #678 weld and #589 re-audit import it under other names
    assert len(aliased) >= 2, [(r, ln, nm) for r, ln, nm, _k in seen]
    assert len(seen) >= 6, [(r, ln, nm) for r, ln, nm, _k in seen]
    missing = [(r, ln, nm) for r, ln, nm, k in seen if k is None]
    assert not missing, f"oracle call(s) without the class map: {missing}"
    const = [(r, ln, nm) for r, ln, nm, k in seen
             if isinstance(k.value, (ast.Constant, ast.Dict))]
    assert not const, f"a constant map is the map dropped: {const}"
    print(f"  PASS: {len(seen)} oracle_reconnect call(s) ({len(aliased)} "
          f"aliased) pass a real net_clearances_by_name")


def test_both_fronts_forward_it():
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

    # ipc-migration: the applier is kicad_ipc_adapter.apply_oracle_reconnect
    # and routing_dialog calls it (there is no gui_utils.
    # run_kicad_oracle_on_live_board and no swig_gui).
    gui = _tree('kicad_ipc_adapter.py')
    fn = next(n for n in ast.walk(gui) if isinstance(n, ast.FunctionDef)
              and n.name == 'apply_oracle_reconnect')
    params = {a.arg for a in fn.args.args + fn.args.kwonlyargs}
    assert 'net_clearances_by_name' in params, params
    inner = list(_calls(fn, 'oracle_reconnect'))
    assert len(inner) == 1, len(inner)
    fwd = _kw(inner[0], 'net_clearances_by_name')
    assert fwd is not None and isinstance(fwd.value, ast.Name) and \
        fwd.value.id == 'net_clearances_by_name', ast.unparse(inner[0])

    swig = _tree(os.path.join('kicad_routing_plugin', 'routing_dialog.py'))
    calls = list(_calls(swig, 'apply_oracle_reconnect'))
    assert calls, 'routing_dialog no longer calls the live oracle'
    for c in calls:
        k = _kw(c, 'net_clearances_by_name')
        assert k is not None and "'net_clearances_by_name'" in ast.unparse(
            k.value), (c.lineno, ast.unparse(c))

    rp = _tree(os.path.join('py_router', 'repair_planes.py'))
    main = next(n for n in rp.body if isinstance(n, ast.FunctionDef)
                and n.name == 'main')
    rc = list(_calls(main, 'oracle_reconnect'))
    assert len(rc) == 1, len(rc)
    k = _kw(rc[0], 'net_clearances_by_name')
    assert k is not None and isinstance(k.value, ast.Name) and \
        k.value.id == 'LAST_NET_CLEARANCES_BY_NAME', ast.unparse(rc[0])
    print("  PASS: the payload carries the map; gui_utils takes and forwards "
          "it; swig_gui hands it over; repair_planes main() passes its "
          "engine's map")


def test_escalation_and_cap_config_carry_the_rules():
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
    # installed the way the finalize leg's _ocfg installs them, so the two
    # oracle configs cannot drift apart
    ocfg = [c for c in _calls(route, 'install_layer_clearances')
            if c.args and isinstance(c.args[0], ast.Name)
            and c.args[0].id == '_ocfg']
    assert len(ocfg) == 1, len(ocfg)
    assert ast.unparse(cap[0]).replace('_cap_cfg', '_ocfg') == \
        ast.unparse(ocfg[0]), (ast.unparse(cap[0]), ast.unparse(ocfg[0]))
    # ...and both are handed the run's RESOLVED map (expanded over the
    # board's copper), not re-read over their own routed subset of layers
    assert 'config.layer_clearances' in ast.unparse(ocfg[0].args[1]), \
        ast.unparse(ocfg[0])
    print(f"  PASS: {len(esc)} escalation call(s) pass the round's class map; "
          f"_cap_cfg installs its layer rules as _ocfg does")


# ---------------------------------------------------------------- re-keying
def test_rekey_by_name():
    from kicad_oracle import rekey_by_name
    from routing_config import GridRouteConfig
    run = GridRouteConfig(clearance=0.2)
    run.set_net_clearances({4: 0.4, 9: 0.3}, routed_net_ids=[4])
    by_name = run.net_clearances_by_name(
        {4: NS(name='/HV'), 9: NS(name='/MID'), 5: NS(name='/B')})
    assert by_name == {'/HV': 0.4, '/MID': 0.3}, by_name
    staged = {1: NS(name='/B'), 2: NS(name='/MID'), 3: NS(name='/HV'),
              0: NS(name='')}
    assert rekey_by_name(by_name, staged) == {3: 0.4, 2: 0.3}
    assert rekey_by_name({}, staged) == {}
    assert rekey_by_name(None, staged) == {}
    print("  PASS: /HV (id 4 in the run) is id 3 on the staged board and "
          "keeps its 0.4")


def test_repair_planes_publishes_the_clamped_map():
    """The map repair_planes() resolved -- after the ceiling clamp -- is what
    its main() hands the oracle."""
    import contextlib
    import io
    from constraint_agreement import write_board
    import repair_planes
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        write_board(b, segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1),
                                 (5, 12, 15, 12, 0.2, 'F.Cu', 2)],
                    classes=[{'name': 'WIDE', 'clearance': 0.5,
                              'priority': 0},
                             {'name': 'MID', 'clearance': 0.25,
                              'priority': 1}],
                    patterns=[('B', 'WIDE'), ('A', 'MID')])
        repair_planes.LAST_NET_CLEARANCES_BY_NAME = {'stale': 9.9}
        with contextlib.redirect_stdout(io.StringIO()):
            repair_planes.repair_planes(
                b, os.path.join(td, 'o.kicad_pcb'), ['A'], ['B.Cu'],
                clearance=0.2, dry_run=True, repair_pads=False,
                clamp_netclasses=True, clearance_ceiling=0.3)
        got = dict(repair_planes.LAST_NET_CLEARANCES_BY_NAME)
    assert got.get('B') == 0.3 and got.get('A') == 0.25 \
        and 'stale' not in got, got
    print(f"  PASS: repair_planes published {got} (B's 0.5 class clamped to "
          f"the 0.3 ceiling)")


# --------------------------------------------------------------- admission
def _v10_board(path, segments=(), vias=(), pads=()):
    """A KiCad-10 style board (nets by NAME, no numbered table): the parser
    numbers nets by first appearance, so `pads` order decides the ids.
    segments: (x1, y1, x2, y2, w, layer, net); vias: (x, y, size, drill, net);
    pads: (ref, x, y, net)."""
    fps = ''.join(f'''
  (footprint "T:P" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "{n}")))'''
                  for ref, x, y, n in pads)
    segs = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width {w}) '
                   f'(layer "{l}") (net "{n}"))'
                   for a, b, c, d, w, l, n in segments)
    vs = ''.join(f'\n  (via (at {x} {y}) (size {s}) (drill {d}) '
                 f'(layers "F.Cu" "B.Cu") (net "{n}"))'
                 for x, y, s, d, n in vias)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts")){fps}{segs}{vs}
)
''')


def _parse(path):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    return pcb, {n.name: nid for nid, n in pcb.nets.items() if n.name}


def test_stitch_via_admission_prices_the_pair():
    """A via of A at (10, 10); B's 0.2 track 0.62 from it (F.Cu), C's via
    1.2 from it. Flat 0.2: track need 0.3+0.1+0.2 = 0.6 -> clear; via need
    0.3+0.3+0.2 = 0.8 -> clear. B in a 0.35 class: track need 0.75 -> NOT
    clear. A .kicad_dru-style B.Cu rule of 0.65 prices via-via over the
    STACK (need 1.25 -> not clear) but not the F.Cu track."""
    from kicad_oracle import _stitch_via_clear
    from routing_config import GridRouteConfig
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        _v10_board(b, segments=[(5, 10.62, 15, 10.62, 0.2, 'F.Cu', 'B')],
                   vias=[(11.2, 10, 0.6, 0.3, 'C')],
                   pads=[('R1', 30, 30, 'A'), ('R2', 31, 31, 'B'),
                         ('R3', 32, 32, 'C')])
        pcb, ids = _parse(b)

        def cfg(**kw):
            return GridRouteConfig(clearance=0.2, via_size=0.6,
                                   via_drill=0.3, layers=['F.Cu', 'B.Cu'],
                                   **kw)
        flat = cfg()
        assert _stitch_via_clear(pcb, ids['A'], 10, 10, flat, 0.2)
        cls = cfg()
        cls.net_clearances = {ids['B']: 0.35}
        assert not _stitch_via_clear(pcb, ids['A'], 10, 10, cls, 0.2), \
            "B's class did not reach the via-vs-track test"
        # the class on the via's OWN net counts too (max of the pair)
        own = cfg()
        own.net_clearances = {ids['A']: 0.35}
        assert not _stitch_via_clear(pcb, ids['A'], 10, 10, own, 0.2)
        stack = cfg()
        stack.layer_clearances = {'B.Cu': 0.65}
        assert not _stitch_via_clear(pcb, ids['A'], 10, 10, stack, 0.2), \
            "via-via must be priced over the stack"
        # the same B.Cu rule leaves the F.Cu track alone: move C's via away
        pcb.vias[:] = []
        assert _stitch_via_clear(pcb, ids['A'], 10, 10, stack, 0.2)
    print("  PASS: via-vs-track at the pair's class, via-via over the "
          "stack; flat config unchanged")


def test_sliver_weld_admission_prices_the_pair():
    """A's weld from (10, 10) to (10.6, 10) on F.Cu, width 0.2 (reach
    0.105). B's 0.2 track at y=10.48: flat need 0.105+0.1+0.2 = 0.405 <=
    0.48 -> laid; B in a 0.35 class needs 0.555 -> refused. A .kicad_dru
    F.Cu rule REPLACES the class on its layer (0.25: laid again), and a
    #735 track rule raises a track pair (0.4 on B: refused)."""
    from kicad_oracle import _direct_sliver_weld
    from routing_config import GridRouteConfig
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        _v10_board(b, segments=[(5, 10.48, 15, 10.48, 0.2, 'F.Cu', 'B')],
                   pads=[('R1', 30, 30, 'A'), ('R2', 31, 31, 'B')])
        pcb, ids = _parse(b)

        def weld(**kw):
            c = GridRouteConfig(clearance=0.2, track_width=0.2,
                                layers=['F.Cu', 'B.Cu'])
            for k, v in kw.items():
                setattr(c, k, v)
            return _direct_sliver_weld(pcb, ids['A'], 10, 10, 10.6, 10,
                                       'F.Cu', c)
        assert weld() is not None, "the flat weld should be laid"
        assert weld(net_clearances={ids['B']: 0.35}) is None, \
            "B's class did not reach the weld's track test"
        assert weld(net_clearances={ids['B']: 0.35},
                    layer_clearances={'F.Cu': 0.25}) is not None, \
            "a layer rule replaces the class on its layer"
        assert weld(track_clearances={ids['B']: 0.4}) is None, \
            "the weld is a track: the track rule must raise the pair"
    print("  PASS: the sliver weld prices B's track at its class, the layer "
          "rule and the track rule; flat unchanged")


def test_pad_override_handling_is_unchanged():
    """#1137 changes the class and rule term only. A pad's own clearance
    override keeps the handling each site always had, so a board that
    declares no class and no rule is unchanged whatever its pads carry: the
    weld's override only RAISES its value, and the stitching via never read
    one. (KiCad lets an override replace the class, floored at the board
    minimum; the oracle's configs carry no board minimum, so replacing here
    would admit copper below it.)"""
    from kicad_oracle import _direct_sliver_weld, _stitch_via_clear
    from routing_config import GridRouteConfig
    with tempfile.TemporaryDirectory() as td:
        # the weld: A from (10, 10) to (10.6, 10), reach 0.105; B's 0.6 pad
        # edge 0.2 off it. Flat need 0.305 -> refused.
        b = os.path.join(td, 'w.kicad_pcb')
        _v10_board(b, pads=[('R1', 30, 30, 'A'), ('R2', 10.3, 10.5, 'B')])
        pcb, ids = _parse(b)
        pad = pcb.footprints['R2'].pads[0]

        def weld():
            c = GridRouteConfig(clearance=0.2, track_width=0.2,
                                layers=['F.Cu', 'B.Cu'])
            return _direct_sliver_weld(pcb, ids['A'], 10, 10, 10.6, 10,
                                       'F.Cu', c)
        assert weld() is None, 'fixture: the flat weld grazes the pad'
        pad.local_clearance = 0.05
        assert weld() is None, 'a LOW override must not relax the weld'
        # stitching via of A at (10, 10), 0.6: B's 0.6 pad 0.7 away, need
        # 0.3 + 0.3 + 0.2 = 0.8 -> refused, override or not
        s = os.path.join(td, 's.kicad_pcb')
        _v10_board(s, pads=[('R1', 30, 30, 'A'), ('R2', 10.7, 10, 'B')])
        pcb, ids = _parse(s)
        cfg = GridRouteConfig(clearance=0.2, via_size=0.6, via_drill=0.3,
                              layers=['F.Cu', 'B.Cu'])
        assert not _stitch_via_clear(pcb, ids['A'], 10, 10, cfg, 0.2)
        pcb.footprints['R2'].pads[0].local_clearance = 0.05
        assert not _stitch_via_clear(pcb, ids['A'], 10, 10, cfg, 0.2), \
            'the stitching via never read a pad override'
    print("  PASS: a low pad override relaxes neither the weld nor the "
          "stitching via")


# ----------------------------------------------------- the oracle, end to end
def _run_oracle(by_name, caller_ids_map=None):
    """One stubbed oracle round on a v10 board where the PARSE numbers
    B=1, A=2, C=3 (B's pad comes first) while the caller's run numbered
    A=1, B=2, C=3. Link: A's 0.6 mm same-layer gap at y=10, B's track 0.48
    away. The routers are stubbed to fail, so the link reaches the sliver
    weld. Returns (spy records, weld results, caller config, parsed ids)."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from routing_config import GridRouteConfig
    seen, welds = [], []
    calls = {'n': 0}

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        calls['n'] += 1
        if calls['n'] > 1:
            return []
        return [('A', (10.0, 10.0, 'F.Cu', 'track'),
                 (10.6, 10.0, 'F.Cu', 'track'))]

    real_bbo = plane_region_connector.build_base_obstacles
    real_weld = kicad_oracle._direct_sliver_weld

    def spy_bbo(*a, **k):
        cfg = k.get('config')
        seen.append({'cfg': cfg, 'nc': dict(cfg.net_clearances or {}),
                     'floor': cfg.net_clearance_floor})
        return real_bbo(*a, **k)

    def spy_weld(*a, **k):
        r = real_weld(*a, **k)
        welds.append(r)
        return r

    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected,
             kicad_exact_fill.refill_islands,
             plane_region_connector.build_base_obstacles,
             plane_region_connector.route_plane_connection_wide,
             net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    kicad_exact_fill.refill_islands = lambda *a, **k: None
    plane_region_connector.build_base_obstacles = spy_bbo
    plane_region_connector.route_plane_connection_wide = \
        lambda *a, **k: (None, 0)
    net_rescue._attempt_edge = lambda *a, **k: (None, None)
    kicad_oracle._direct_sliver_weld = spy_weld
    try:
        with tempfile.TemporaryDirectory() as td:
            board = os.path.join(td, 'b.kicad_pcb')
            _v10_board(board,
                       segments=[(5, 10, 10, 10, 0.2, 'F.Cu', 'A'),
                                 (10.6, 10, 15, 10, 0.2, 'F.Cu', 'A'),
                                 (5, 10.48, 15, 10.48, 0.2, 'F.Cu', 'B')],
                       pads=[('R2', 30, 30, 'B'), ('R1', 5, 10, 'A'),
                             ('R3', 15, 10, 'A'), ('R4', 32, 32, 'C')])
            _pcb, ids = _parse(board)
            caller = GridRouteConfig(clearance=0.2, track_width=0.2,
                                     via_size=0.6, via_drill=0.3,
                                     layers=['F.Cu', 'B.Cu'],
                                     board_edge_clearance=0.2)
            if caller_ids_map is not None:
                caller.net_clearances = dict(caller_ids_map)
            import contextlib
            import io
            with contextlib.redirect_stdout(io.StringIO()):
                kicad_oracle.oracle_reconnect(
                    board, ['A'], caller, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.2, max_rounds=1,
                    net_clearances_by_name=by_name)
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.build_base_obstacles,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld) = saved
    return seen, welds, caller, ids


def test_the_map_lands_by_name_and_the_floor_is_the_links_class():
    # the caller's own ids would put B's 0.35 on the parse's A (id 2)
    seen, welds, caller, ids = _run_oracle({'A': 0.3, 'B': 0.35},
                                           caller_ids_map={1: 0.3, 2: 0.35})
    assert ids == {'B': 1, 'A': 2, 'C': 3}, \
        f"fixture: the parse must renumber the nets, got {ids}"
    assert seen, 'the oracle never built an obstacle map for the link'
    first = seen[0]
    assert first['nc'] == {ids['A']: 0.3, ids['B']: 0.35}, (first['nc'], ids)
    assert abs(first['floor'] - 0.3) < 1e-12, first['floor']
    cfg = first['cfg']
    assert abs(cfg.obstacle_clearance(ids['B']) - 0.35) < 1e-12
    assert abs(cfg.obstacle_clearance(ids['C']) - 0.3) < 1e-12
    assert cfg is not caller
    assert caller.net_clearances == {1: 0.3, 2: 0.35}
    assert caller.net_clearance_floor is None
    print(f"  PASS: the link's map is {first['nc']} (by name, on the parse's "
          f"ids {ids}), its floor A's 0.3; the caller's config is unchanged")


def test_admission_uses_the_handed_in_class_end_to_end():
    _s, welds_flat, _c, _i = _run_oracle(None)
    assert welds_flat and welds_flat[0] is not None, \
        f"fixture: with no class map the sliver weld must be laid {welds_flat}"
    seen, welds_cls, caller, _i = _run_oracle({'B': 0.35})
    assert welds_cls and welds_cls[0] is None, \
        f"B's 0.35 class (by name) did not refuse the weld: {welds_cls}"
    # no map -> the oracle's config is a field-for-field copy of the caller's
    s0, _w, c0, _i = _run_oracle(None)
    assert s0[0]['cfg'] is not c0 and s0[0]['nc'] == {} \
        and s0[0]['floor'] is None
    print("  PASS: flat -> weld laid; B's class handed in by name -> the "
          "same weld refused")


TESTS = [test_every_oracle_call_hands_over_the_map,
         test_both_fronts_forward_it,
         test_escalation_and_cap_config_carry_the_rules,
         test_rekey_by_name,
         test_repair_planes_publishes_the_clamped_map,
         test_stitch_via_admission_prices_the_pair,
         test_sliver_weld_admission_prices_the_pair,
         test_pad_override_handling_is_unchanged,
         test_the_map_lands_by_name_and_the_floor_is_the_links_class,
         test_admission_uses_the_handed_in_class_end_to_end]


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
