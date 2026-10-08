#!/usr/bin/env python3
"""#1135: every config that lays copper carries the board's .kicad_dru
track-to-track rules (#735), not only route.py's and route_diff.py's.

The fixture is the issue's: a board whose .kicad_dru raises track-to-track
clearance to 0.5 mm between class CRIT (net A) and every other class. What
each case pins:

* `route_planes.create_plane` and `repair_planes.repair_planes` -- the
  configs they actually build, captured where they install their rules --
  carry the rule; an explicit map handed to repair_planes wins (route.py's
  finalize forwards its run's, as it does the layer map, because the
  output's .kicad_dru does not exist yet mid-run), and both finalize legs
  forward it;
* the KiCad oracle installs the rule on every parse, read beside
  `project_from` and keyed by THAT parse's net ids: on a board whose parse
  renumbers the nets (the GUI's pcbnew save) the raise lands on B and C, not
  on whatever their ids were in the caller's run;
* end to end, the sliver weld the oracle lays with no rules file is refused
  once the board's track rule forbids it;
* with no .kicad_dru every one of these configs carries an empty map, as
  before.

The KiCad link source, the island refill and the routers are stubbed; no
KiCad is needed.

    python3 tests/test_1135_track_rules_everywhere.py [case-substring ...]
"""
import ast
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', os.path.join('tests', 'oracle'), 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from constraint_agreement import write_board  # noqa: E402

#: The issue's rule: CRIT (net A) vs any other class, tracks only.
RULE = ('(rule "crit space" (condition "A.Type==\'track\' && '
        'B.Type==\'track\' && A.NetClass==\'CRIT\' && B.NetClass!=\'CRIT\'") '
        '(constraint clearance (min 0.5mm)))')
CLASSES = [{'name': 'CRIT', 'clearance': 0.2, 'priority': 0}]


def _board(td, dru=True):
    b = os.path.join(td, 'b.kicad_pcb')
    write_board(b, segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1),
                             (5, 10.5, 15, 10.5, 0.2, 'F.Cu', 2)],
                classes=CLASSES, patterns=[('A', 'CRIT')],
                dru=RULE if dru else None)
    return b


class _Stop(Exception):
    pass


@contextlib.contextmanager
def _captured_track_installs():
    """Record every config `install_track_clearances` fills, then stop the
    engine at its next step (`set_board_net_clearances`, which both plane
    engines call right after their rules): the config is what is graded."""
    import kicad_dru
    import plane_fill_model
    seen = []
    real = kicad_dru.install_track_clearances

    def spy(config, *a, **k):
        real(config, *a, **k)
        seen.append(config)

    def stop(*a, **k):
        raise _Stop()
    saved = (kicad_dru.install_track_clearances,
             plane_fill_model.set_board_net_clearances)
    kicad_dru.install_track_clearances = spy
    plane_fill_model.set_board_net_clearances = stop
    try:
        yield seen
    finally:
        (kicad_dru.install_track_clearances,
         plane_fill_model.set_board_net_clearances) = saved


def _engine_config(engine, dru=True, **kw):
    import route_planes
    import repair_planes
    with tempfile.TemporaryDirectory() as td:
        b = _board(td, dru=dru)
        with _captured_track_installs() as seen, \
                contextlib.redirect_stdout(io.StringIO()):
            try:
                if engine == 'create_plane':
                    route_planes.create_plane(
                        b, os.path.join(td, 'o.kicad_pcb'), ['A'], ['B.Cu'],
                        dry_run=True, all_layers=['F.Cu', 'B.Cu'], **kw)
                else:
                    repair_planes.repair_planes(
                        b, os.path.join(td, 'o.kicad_pcb'), ['A'], ['B.Cu'],
                        dry_run=True, repair_pads=False, **kw)
            except _Stop:
                pass
    assert len(seen) == 1, f"{engine}: installed track rules {len(seen)}x"
    return seen[0]


def test_route_style_config_sees_the_rule():
    """The fixture itself: route.py's install reads the rule (the control
    the plane configs are held to)."""
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    import kicad_dru
    with tempfile.TemporaryDirectory() as td:
        b = _board(td)
        pcb = parse_kicad_pcb(b)
        cfg = GridRouteConfig(clearance=0.2, layers=['F.Cu', 'B.Cu'])
        with contextlib.redirect_stdout(io.StringIO()):
            kicad_dru.install_track_clearances(cfg, None, b, pcb,
                                               routed_net_ids=list(pcb.nets))
    assert cfg.track_clearances == {1: 0.5, 2: 0.5, 3: 0.5}, \
        cfg.track_clearances
    print(f"  PASS: route.py-style config {cfg.track_clearances}")


def test_plane_engines_install_the_rule():
    cp = _engine_config('create_plane')
    rp = _engine_config('repair_planes')
    # routed set = A (CRIT): the rule binds A against B and C
    assert cp.track_clearances == {2: 0.5, 3: 0.5}, cp.track_clearances
    assert rp.track_clearances == {2: 0.5, 3: 0.5}, rp.track_clearances
    explicit = _engine_config('repair_planes', track_clearances={2: 0.9})
    assert explicit.track_clearances == {2: 0.9}, explicit.track_clearances
    none_cp = _engine_config('create_plane', dru=False)
    none_rp = _engine_config('repair_planes', dru=False)
    assert none_cp.track_clearances == {} and none_rp.track_clearances == {}
    print(f"  PASS: create_plane {cp.track_clearances}, repair_planes "
          f"{rp.track_clearances}; an explicit map wins; no rules file -> {{}}")


def test_the_finalize_forwards_the_runs_rules():
    with open(os.path.join(ROOT, 'py_router', 'route.py'),
              encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    legs = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and getattr(n.func, 'id', '') == '_rdp_engine']
    assert len(legs) == 2, len(legs)
    for c in legs:
        kws = {k.arg: ast.unparse(k.value) for k in c.keywords}
        assert 'config.track_clearances' in kws.get('track_clearances', ''), \
            (c.lineno, sorted(kws))
        assert 'config.layer_clearances' in kws.get('layer_clearances', '')
    print("  PASS: both plane-finalize legs (CLI, GUI) forward the run's "
          "track rules beside its layer map")


# ------------------------------------------------------------ the oracle
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


def _run_oracle(dru):
    """One stubbed round on a board whose parse numbers B=1, A=2, C=3, with
    the project (.kicad_pro, .kicad_dru when `dru`) beside it. Link: A's
    0.6 mm same-layer gap at y=10; B's track 0.48 away (flat need 0.405,
    under the 0.5 track rule 0.705). Returns (spied configs, sliver weld
    results, parsed ids)."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from routing_config import GridRouteConfig
    from kicad_parser import parse_kicad_pcb
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
        seen.append(k.get('config'))
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
            # the project (classes, patterns, rules) is the issue's; the
            # board is rewritten in the by-name dialect so its parse
            # renumbers the nets
            proj = os.path.join(td, 'proj.kicad_pcb')
            write_board(proj, classes=CLASSES, patterns=[('A', 'CRIT')],
                        dru=RULE if dru else None)
            board = os.path.join(td, 'staged.kicad_pcb')
            _v10_board(board,
                       segments=[(5, 10, 10, 10, 0.2, 'F.Cu', 'A'),
                                 (10.6, 10, 15, 10, 0.2, 'F.Cu', 'A'),
                                 (5, 10.48, 15, 10.48, 0.2, 'F.Cu', 'B')],
                       pads=[('R2', 30, 30, 'B'), ('R1', 5, 10, 'A'),
                             ('R3', 15, 10, 'A'), ('R4', 32, 32, 'C')])
            ids = {n.name: nid for nid, n in
                   parse_kicad_pcb(board).nets.items() if n.name}
            caller = GridRouteConfig(clearance=0.2, track_width=0.2,
                                     via_size=0.6, via_drill=0.3,
                                     layers=['F.Cu', 'B.Cu'],
                                     board_edge_clearance=0.2)
            with contextlib.redirect_stdout(io.StringIO()):
                kicad_oracle.oracle_reconnect(
                    board, ['A'], caller, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.2, max_rounds=1,
                    project_from=proj)
            assert caller.track_clearances == {}, caller.track_clearances
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.build_base_obstacles,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld) = saved
    return seen, welds, ids


def test_the_oracle_installs_the_rule_on_its_parse():
    seen, welds, ids = _run_oracle(dru=True)
    assert ids == {'B': 1, 'A': 2, 'C': 3}, \
        f"fixture: the parse must renumber the nets, got {ids}"
    assert seen, 'the oracle never built an obstacle map'
    tc = seen[0].track_clearances
    assert tc == {ids['B']: 0.5, ids['C']: 0.5}, (tc, ids)
    assert welds and welds[0] is None, \
        f"the 0.5 track rule did not refuse the sliver weld: {welds}"
    seen0, welds0, _ids = _run_oracle(dru=False)
    assert seen0 and seen0[0].track_clearances == {}
    assert welds0 and welds0[0] is not None, \
        f"fixture: with no rules file the weld must be laid: {welds0}"
    print(f"  PASS: the oracle's map {tc} is keyed by its parse (B={ids['B']}, "
          f"C={ids['C']}); the weld laid with no rules file is refused under "
          f"the rule")


def test_the_plane_builders_read_the_rule():
    """Installing the rule is only half of it: the builders and the gate the
    plane steps and the oracle's main weld lay copper through must READ it,
    on a track-vs-track pair only. Net A (the plane / weld net) and B's 0.2
    track at y=10.48: flat, A's 0.2 track at y=10.0 needs 0.4 centre to
    centre (clears 0.48); under B's 0.5 rule it needs 0.7 (refused)."""
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig, GridCoord
    from plane_region_connector import build_base_obstacles, wide_route_clear
    from plane_obstacle_builder import build_routing_obstacle_map
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        write_board(b, segments=[(5, 10, 10, 10, 0.2, 'F.Cu', 1),
                                 (5, 10.48, 15, 10.48, 0.2, 'F.Cu', 2)])
        pcb = parse_kicad_pcb(b)

    def cfg(tc):
        c = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                            via_drill=0.3, layers=['F.Cu', 'B.Cu'],
                            grid_step=0.05)
        c.track_clearances = dict(tc)
        return c
    flat, ruled = cfg({}), cfg({2: 0.5})
    g = GridCoord(0.05)
    probe = g.to_grid(12.0, 10.05)      # 0.43 from B's centreline
    with contextlib.redirect_stdout(io.StringIO()):
        maps = {name: build_base_obstacles(
            exclude_net_ids={1}, routing_layers=['F.Cu', 'B.Cu'],
            pcb_data=pcb, config=c, track_width=0.2,
            track_via_clearance=0.2, hole_to_hole_clearance=0.2)[0]
            for name, c in (('flat', flat), ('ruled', ruled))}
        rmaps = {name: build_routing_obstacle_map(pcb, c, 1, 'F.Cu',
                                                  verbose=False)
                 for name, c in (('flat', flat), ('ruled', ruled))}
    assert not maps['flat'].is_blocked(*probe, 0), \
        'fixture: flat, 0.43 c-c clears the 0.4 keep-out'
    assert maps['ruled'].is_blocked(*probe, 0), \
        'plane_region_connector.build_base_obstacles ignores the track rule'
    assert not rmaps['flat'].is_blocked(*probe, 0)
    assert rmaps['ruled'].is_blocked(*probe, 0), \
        'plane_obstacle_builder.build_routing_obstacle_map ignores the rule'
    weld = [(10.0, 10.0, 'F.Cu'), (10.6, 10.0, 'F.Cu')]
    assert wide_route_clear(weld, 0.2, pcb, 1, flat, board_edge_clearance=0.0)
    assert not wide_route_clear(weld, 0.2, pcb, 1, ruled,
                                board_edge_clearance=0.0), \
        "wide_route_clear (the oracle's emitted-copper gate) ignores the rule"
    # the rule binds tracks: a via stamp is not raised by it
    vprobe = g.to_grid(12.0, 10.48 - 0.1 - 0.3 - 0.2 - 0.05)
    assert maps['flat'].is_via_blocked(*vprobe) == \
        maps['ruled'].is_via_blocked(*vprobe), 'the track rule raised a via stamp'
    print("  PASS: build_base_obstacles, build_routing_obstacle_map and "
          "wide_route_clear price B's track at its 0.5 rule; vias untouched")


TESTS = [test_route_style_config_sees_the_rule,
         test_plane_engines_install_the_rule,
         test_the_finalize_forwards_the_runs_rules,
         test_the_oracle_installs_the_rule_on_its_parse,
         test_the_plane_builders_read_the_rule]


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
