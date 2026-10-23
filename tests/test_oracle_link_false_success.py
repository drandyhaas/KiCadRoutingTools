#!/usr/bin/env python3
"""A plane link counts as made only when its copper can join what KiCad
reported apart.

Four ways a stranded plane pad used to be reported fixed (wave
cloud_g14integ-kc, 12 nets on 9 boards shipped open):

* the stub-swap and tap-relocation rescues routed a net carrying exact-fill
  oracle links (#572) by endpoint DERIVATION -- the fill model's zone credit
  merges the clusters KiCad grades apart, so the route "succeeded" on a pour
  cell of an island (a GND connector pin: "STUB-SWAP RESCUE ... routed (1
  segments)", shipped open). They now route the links, and never re-layer a
  stub a link end sits on;
* route_oracle_links refused every link with a via or plated-barrel end,
  which KiCad anchors with no layer (an FPGA pin's link: "forced link
  endpoint layer not in the routing stack (F.Cu / None)"). Such an end is
  copper on every layer it spans;
* the oracle printed "OK 0 seg(s), 0 via(s)" and counted the link routed
  when the path laid no copper (a GND link);
* the weld escalation's reach test was layer-blind, so a 0.28 mm In1.Cu
  piece on the net's existing In1 copper was credited for a B.Cu pad's link
  two rounds running (a GND capacitor pad).

Every row runs with the routers stubbed: no KiCad, no grid search.

    python3 tests/test_oracle_link_false_success.py [case-substring ...]
"""
import contextlib
import io
import os
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def _via(x, y, net=1, size=0.4, layers=('F.Cu', 'B.Cu'), via_type=None):
    from kicad_parser import Via
    v = Via(x=x, y=y, size=size, drill=0.2, layers=list(layers), net_id=net)
    if via_type:
        v.via_type = via_type
    return v


def _th_pad(x, y, net=1):
    return NS(global_x=x, global_y=y, size_x=1.0, size_y=1.0, drill=0.6,
              pad_type='thru_hole', layers=['*.Cu', '*.Mask'], net_id=net,
              hole_x=None, hole_y=None)


def test_a_via_or_barrel_end_resolves_to_its_span():
    from single_ended_routing import oracle_link_end_layers as ends
    pcb = NS(vias=[_via(10, 10), _via(20, 20, via_type='blind',
                                      layers=('F.Cu', 'In1.Cu'))],
             pads_by_net={1: [_th_pad(30, 30)]})
    assert ends(pcb, 1, 5, 5, 'In2.Cu', LAYERS) == ['In2.Cu']
    assert ends(pcb, 1, 10, 10, None, LAYERS) == LAYERS, \
        'a through via is copper on every layer'
    assert ends(pcb, 1, 20, 20, None, LAYERS) == ['F.Cu', 'In1.Cu'], \
        "a blind via is copper on its span only"
    assert ends(pcb, 1, 30.3, 30, '*.Cu', LAYERS) == LAYERS, \
        'a plated barrel is copper on every layer'
    assert ends(pcb, 1, 30, 30, None, LAYERS) == LAYERS
    assert ends(pcb, 2, 10, 10, None, LAYERS) == [], \
        "another net's via is not this end's copper"
    assert ends(pcb, 1, 50, 50, None, LAYERS) == [], 'nothing there'
    assert ends(pcb, 1, 5, 5, 'In5.Cu', LAYERS) == [], \
        'a named layer outside the stack stays unroutable'
    print('  PASS: named layer kept; via/barrel ends resolve to their span')


def test_a_forced_link_to_a_via_routes_on_its_span():
    import single_ended_routing as ser
    from routing_config import GridRouteConfig
    seen = []

    def fake_route(pcb, nid, cfg, obst, **kw):
        seen.append((kw['sources_override'], kw['targets_override']))
        return {'failed': False, 'new_segments': [object()], 'new_vias': [],
                'iterations': 1}
    pcb = NS(vias=[_via(10, 11)], pads_by_net={})
    cfg = GridRouteConfig(layers=list(LAYERS), grid_step=0.05)
    link = ((10.0, 10.0, 'F.Cu', 'pad'), (10.0, 11.0, None, 'zone'))
    saved = ser.route_net_with_obstacles
    ser.route_net_with_obstacles = fake_route
    try:
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            r = ser.route_oracle_links(pcb, 1, cfg, None, [link])
    finally:
        ser.route_net_with_obstacles = saved
    assert 'not in the routing stack' not in log.getvalue(), log.getvalue()
    assert r and not r.get('failed'), r
    (src, tgt), = seen
    assert [s[2] for s in src] == [0], src
    assert sorted(t[2] for t in tgt) == [0, 1, 2, 3], \
        f'the via end must seed every layer it spans: {tgt}'
    print('  PASS: F.Cu pad <-> through via routes, target on all 4 layers')


def test_the_rescue_routes_the_forced_links():
    import single_ended_loop as sel
    calls = []
    saved = (sel.prepare_obstacles_inplace, sel.restore_obstacles_inplace,
             sel.route_oracle_links, sel.route_net_with_obstacles)
    sel.prepare_obstacles_inplace = lambda *a, **k: ([], None)
    sel.restore_obstacles_inplace = lambda *a, **k: None
    sel.route_oracle_links = \
        lambda pcb, nid, cfg, obst, links, **k: calls.append(('links', links))
    sel.route_net_with_obstacles = \
        lambda pcb, nid, cfg, obst, **k: calls.append(('derive', None))
    link = ((1.0, 1.0, 'F.Cu', 'pad'), (2.0, 2.0, 'F.Cu', 'zone'))
    try:
        for olinks in ({7: [link]}, {}):
            state = NS(working_obstacles=object(), all_unrouted_net_ids=[],
                       net_obstacles_cache={}, ripped_route_layer_costs={},
                       ripped_route_via_positions={},
                       oracle_links_by_net=olinks)
            sel._route_on_prepared_map(None, 7, None, state, [], {}, {})
    finally:
        (sel.prepare_obstacles_inplace, sel.restore_obstacles_inplace,
         sel.route_oracle_links, sel.route_net_with_obstacles) = saved
    assert calls == [('links', [link]), ('derive', None)], calls
    print('  PASS: a net with oracle links routes them; without, it derives')


def test_the_stub_swap_leaves_a_stub_carrying_a_link_end():
    import connectivity
    import stub_layer_switching as sls
    import single_ended_loop as sel
    from kicad_parser import Segment
    from routing_config import GridRouteConfig
    stub_seg = Segment(start_x=0, start_y=0, end_x=2, end_y=0, width=0.2,
                       layer='In1.Cu', net_id=7)
    validated = []
    names = ('get_stub_endpoints',)
    saved_c = {n: getattr(connectivity, n) for n in names}
    keep = ('get_stub_info', 'validate_single_swap', 'foreign_stub_registry',
            'stub_switch_segments')
    saved_s = {n: getattr(sls, n) for n in keep}
    connectivity.get_stub_endpoints = lambda pcb, nids: [(2, 0, 'In1.Cu')]
    sls.get_stub_info = lambda *a, **k: NS(net_id=7, layer='In1.Cu',
                                           segments=[stub_seg])
    sls.foreign_stub_registry = lambda *a, **k: {}
    sls.stub_switch_segments = lambda pcb, stub: list(stub.segments)
    sls.validate_single_swap = \
        lambda stub, dest, *a, **k: validated.append(dest) or (False, 'x')
    cfg = GridRouteConfig(layers=['F.Cu', 'In1.Cu', 'B.Cu'])
    try:
        for anchor, expect_tried in (((1.0, 0.0, 'In1.Cu', 'track'), False),
                                     ((1.0, 0.0, None, 'zone'), False),
                                     ((1.0, 0.0, 'F.Cu', 'pad'), True),
                                     ((9.0, 9.0, 'In1.Cu', 'zone'), True)):
            validated.clear()
            link = (anchor, (5.0, 5.0, 'In1.Cu', 'zone'))
            state = NS(oracle_links_by_net={7: [link]})
            sel._stub_swap_rescue(None, 7, cfg, state, [], [], [], {}, {},
                                  {}, {}, {})
            assert bool(validated) == expect_tried, (anchor, validated)
    finally:
        for n, f in saved_c.items():
            setattr(connectivity, n, f)
        for n, f in saved_s.items():
            setattr(sls, n, f)
    print('  PASS: a stub under a link end (on its layer, or a via end) is '
          'never re-layered; other stubs still are')


def _v10_board(path, segments=(), pads=()):
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


def test_a_path_that_lays_no_copper_is_not_a_weld():
    """The link routes, but its path only changes layer at one point: no
    leg, no via. That used to print OK 0 seg(s), 0 via(s) and count."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from routing_config import GridRouteConfig

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        return [('A', (10.0, 10.0, 'F.Cu', 'track'),
                 (10.6, 10.0, 'B.Cu', 'track'))]
    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected, kicad_exact_fill.refill_islands,
             plane_region_connector.route_plane_connection_wide,
             net_rescue._attempt_edge)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    kicad_exact_fill.refill_islands = lambda *a, **k: None
    plane_region_connector.route_plane_connection_wide = \
        lambda *a, **k: (([(10.3, 10.0, 'F.Cu'), (10.3, 10.0, 'B.Cu')], []), 1)
    net_rescue._attempt_edge = lambda *a, **k: (None, None)
    try:
        with tempfile.TemporaryDirectory() as td:
            board = os.path.join(td, 'b.kicad_pcb')
            _v10_board(board,
                       segments=[(5, 10, 10, 10, 0.2, 'F.Cu', 'A'),
                                 (10.6, 10, 15, 10, 0.2, 'B.Cu', 'A')],
                       pads=[('R1', 5, 10, 'A'), ('R3', 30, 30, 'A')])
            cfg = GridRouteConfig(clearance=0.2, track_width=0.2,
                                  via_size=0.6, via_drill=0.3,
                                  layers=['F.Cu', 'B.Cu'])
            log = io.StringIO()
            with contextlib.redirect_stdout(log):
                r = kicad_oracle.oracle_reconnect(
                    board, ['A'], cfg, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.2, max_rounds=1)
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge) = saved
    out = log.getvalue()
    assert 'OK 0 seg(s), 0 via(s)' not in out, out
    assert 'lays no copper' in out, out
    assert r['links_routed'] == 0 and r['links_failed'] == 1, r
    print('  PASS: a zero-copper path is FAILED, not OK')


def test_the_weld_reach_test_is_layer_aware():
    from kicad_oracle import _weld_touches_endpoints as touches
    from kicad_parser import Segment
    # A GND capacitor pad: the pad end is on B.Cu, the via end has no layer
    piece = Segment(start_x=139.0, start_y=121.7, end_x=138.8, end_y=121.5,
                    width=0.0762, layer='In1.Cu', net_id=1)
    ends = (139.1, 121.98, 138.4, 121.2)
    assert touches([piece], [], *ends), \
        'control: xy-only, the piece is within tol of both ends'
    assert not touches([piece], [], *ends, layers=('B.Cu', None)), \
        'an In1.Cu piece cannot reach a B.Cu pad'
    on_b = Segment(start_x=139.1, start_y=121.98, end_x=138.4, end_y=121.2,
                   width=0.0762, layer='B.Cu', net_id=1)
    assert touches([on_b], [], *ends, layers=('B.Cu', None))
    assert touches([piece], [_via(139.1, 121.9)], *ends,
                   layers=('B.Cu', None)), 'a through via reaches B.Cu'
    assert not touches([piece], [_via(139.1, 121.9, via_type='blind',
                                      layers=('F.Cu', 'In1.Cu'))],
                       *ends, layers=('B.Cu', None)), \
        'a blind F.Cu-In1.Cu via does not'
    print('  PASS: an end on a named layer is reached only on that layer')


def test_the_exact_tier_reads_an_anchor_on_its_own_layer():
    """A GND net: a carved-off ball's F.Cu pad anchor lies in no F.Cu
    island but inside the In1 plane; the tier took the In1 plane as that
    side and strapped it to the F.Cu pour where the two outlines coincide
    (the board edge, 30 mm away). Fixture: an F.Cu pour y 0..10 and an In1
    plane y 0..20 share three outline edges; the pad sits at (10, 12)."""
    from kicad_oracle import _exact_fill_endpoints
    f_main = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
    in1 = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
    islands = {('GND', 'F.Cu'): [f_main], ('GND', 'In1.Cu'): [in1]}
    pcb = NS(board_info=NS(board_outlines=[], board_outline=[],
                           board_cutouts=[], board_bounds=(0, 0, 20, 20)))
    r = _exact_fill_endpoints(pcb, 1, 'GND', (10.0, 12.0, 'F.Cu', 'pad'),
                              (10.0, 9.9, 'F.Cu', 'zone'), islands,
                              track_half=0.05)
    assert isinstance(r, tuple), r
    _src, _tgt, pa, pb, layer = r
    assert layer == 'F.Cu' and pa[2] == 'F.Cu' and pb[2] == 'F.Cu', r[2:]
    assert abs(pa[0] - 10.0) < 1e-6 and abs(pa[1] - 12.0) < 1e-6, \
        f'the pad end must seed from the pad, got {pa}'
    assert abs(pb[1] - 10.0) < 0.3 and abs(pb[0] - 10.0) < 0.5, \
        f'the pour end must be the F.Cu edge facing the pad, got {pb}'
    print(f'  PASS: strap {pa} -> {pb} on F.Cu (was the shared outline)')


def test_the_strap_escalation_routes_the_exact_seeds():
    """The scoped rescue the oracle escalates to derived each side of the gap
    from the net's tracks, pads and vias near it -- layer-blind, and blind to
    fill, which is what a plane link must reach. Given the exact clusters'
    seeds it routes them to each other on their own layers. Fixture: the
    seeds are fill points (no copper of the net within 20 mm), one on F.Cu
    and one on B.Cu: the derivation has nothing to route, the seeds route
    with a via and the weld reaches both."""
    import ast
    import net_rescue
    from kicad_parser import parse_kicad_pcb
    from kicad_oracle import _weld_joins_seeds
    from routing_config import GridRouteConfig
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        _v10_board(b, pads=[('R1', 30, 30, 'A'), ('R2', 31, 31, 'A'),
                            ('R3', 33, 33, 'B')])
        pcb = parse_kicad_pcb(b)
        nid = {n.name: i for i, n in pcb.nets.items()}['A']
        cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                              via_drill=0.3, layers=['F.Cu', 'B.Cu'],
                              grid_step=0.1)
        src, tgt = [(5.0, 5.0, 'F.Cu')], [(8.0, 5.0, 'B.Cu')]
        gap = (3.0, 5.0, 5.0, 8.0, 5.0)
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            derived, _c = net_rescue._attempt_edge(
                pcb, nid, gap, cfg, None, strict_endpoints=True)
            seeded, _c = net_rescue._attempt_edge(
                pcb, nid, gap, cfg, None, strict_endpoints=True,
                endpoints=(src, tgt))
    assert derived is None, 'control: the derivation cannot see fill seeds'
    assert seeded and not seeded.get('failed'), log.getvalue()[-2000:]
    segs, vias = seeded['new_segments'], seeded['new_vias']
    assert vias, 'F.Cu to B.Cu must take a via'
    assert _weld_joins_seeds(segs, vias, src, tgt)
    assert not _weld_joins_seeds(segs, vias, src, [(8.0, 5.0, 'In1.Cu')]), \
        'a seed on another layer is not reached'
    # Every escalation in the oracle that HAS the exact seeds hands them over
    with open(os.path.join(ROOT, 'py_router', 'kicad_oracle.py'),
              encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == '_attempt_edge']
    assert len(calls) == 2, len(calls)
    assert all(any(k.arg == 'endpoints' for k in c.keywords) for c in calls)
    print(f"  PASS: seeded escalation laid {len(segs)} seg(s) + {len(vias)} "
          f"via(s) between fill seeds the derivation could not see")


def test_a_long_pad_joins_only_copper_that_touches_it():
    """ddr5 GND J1.177, a 0.35 x 2.5 mm SODIMM finger: exact_clusters looked
    its copper up with a tolerance inflated by the pad's half-LENGTH, and the
    pad rung measures from the pad's copper, so a via 1.40 mm off the finger
    joined it. The exact source then reported no link for a pad KiCad grades
    open. Fixture: the finger at (10, 10), a GND via 1.43 mm off its copper,
    tracked to a far GND pad; no fill."""
    from kicad_parser import parse_kicad_pcb
    from kicad_exact_fill import exact_clusters
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        with open(b, 'w', encoding='utf-8') as f:
            f.write('''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (footprint "T:F" (layer "F.Cu") (at 10 10)
    (property "Reference" "J1" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.35 2.5) (layers "F.Cu") (net "GND")))
  (footprint "T:P" (layer "F.Cu") (at 20 8.6)
    (property "Reference" "R2" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "GND")))
  (via (at 11.6 8.6) (size 0.35) (drill 0.2) (layers "F.Cu" "B.Cu") (net "GND"))
  (segment (start 11.6 8.6) (end 20 8.6) (width 0.2) (layer "F.Cu") (net "GND"))
)
''')
        pcb = parse_kicad_pcb(b)
        gid = {n.name: i for i, n in pcb.nets.items()}['GND']
        cl = exact_clusters(pcb, gid, [])
    groups = sorted(sorted(f"{p.component_ref}.{p.pad_number}"
                           for p in c['pads']) for c in cl)
    assert groups == [['J1.1'], ['R2.1']], \
        f'the finger must stand alone (the via is 1.43 mm off it): {groups}'
    print(f"  PASS: clusters {groups}")


TESTS = [test_a_via_or_barrel_end_resolves_to_its_span,
         test_a_forced_link_to_a_via_routes_on_its_span,
         test_the_rescue_routes_the_forced_links,
         test_the_stub_swap_leaves_a_stub_carrying_a_link_end,
         test_a_path_that_lays_no_copper_is_not_a_weld,
         test_the_weld_reach_test_is_layer_aware,
         test_the_exact_tier_reads_an_anchor_on_its_own_layer,
         test_the_strap_escalation_routes_the_exact_seeds,
         test_a_long_pad_joins_only_copper_that_touches_it]


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
