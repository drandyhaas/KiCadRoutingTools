#!/usr/bin/env python3
"""#1544: the KiCad oracle never welds a link that ends outside the outline.

route.py drops a pad clearly outside the Edge.Cuts outline from every routing
endpoint set (#291, `connectivity.drop_off_board_pads`), but KiCad's own
connectivity knows nothing of the outline: both link sources the oracle reads
(the exact-fill refill and kicad-cli) report links to and between such pads.
The plane finalize's oracle leg routed them -- copper between two off-board
pads and from an off-board pad back onto the board -- and handed the ones it
could not weld to the final reconciliation as #572 forced edges, which routed
them again. What each case pins:

* `make_off_board_link_end_test` reads an end as off the board when the
  point is outside the outline, or when it lies on the copper of a pad its
  net loses to #291 (a nearest-approach anchor on such a pad can sit nearer
  the edge than the pad centre the #291 test reads); an on-board end, and
  a pad #291 keeps (one straddling the edge), is not;
* one stubbed oracle round: the off-board link is never routed, still counts
  in `remaining` (the net IS open), is reported in `off_board`, and is kept
  out of `remaining_links`, so no custody or forced edge routes it; the
  on-board link beside it is routed as before;
* a round whose only links end off the board routes nothing and says so.

The link source and the routers are stubbed, so no KiCad is needed.

    python3 tests/test_1544_oracle_off_board_links.py [case-substring ...]
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))


def _board(path, pads, segments=()):
    """A KiCad-10 board, outline 0..40 x 0..40. pads: (ref, x, y, net, size);
    segments: (x1, y1, x2, y2, w, layer, net)."""
    fps = ''.join(f'''
  (footprint "T:P" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size {s} {s}) (layers "F.Cu") (net "{n}")))'''
                  for ref, x, y, n, s in pads)
    segs = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width {w}) '
                   f'(layer "{l}") (net "{n}"))'
                   for a, b, c, d, w, l, n in segments)
    edge = ('\n  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) '
            '(type default)) (fill none) (layer "Edge.Cuts"))')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0)){edge}{fps}{segs}
)
''')


def _parse(path):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    return pcb, {n.name: nid for nid, n in pcb.nets.items() if n.name}


def test_link_end_test():
    from connectivity import make_off_board_link_end_test
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        # R5's centre is 1.0 mm off the board (the #291 test drops it); its
        # 1.2 mm copper reaches to 0.4 mm off, inside the test's 0.5 mm
        # tolerance. R6 straddles the edge (centre 0.2 off): #291 keeps it.
        _board(b, [('R1', 5, 10, 'A', 0.6), ('R5', 10, -1.0, 'A', 1.2),
                   ('R6', 20, -0.2, 'A', 0.6), ('R7', 30, -1.0, 'B', 1.2)])
        pcb, ids = _parse(b)
        off = make_off_board_link_end_test(pcb)
        assert off is not None
        A, B = ids['A'], ids['B']
        assert not off(A, 5, 10), 'an on-board pad is not off the board'
        assert off(A, 10, -1.0), 'the off-board pad centre'
        assert off(A, 10, -0.45), \
            "an anchor on the dropped pad's copper, 0.45 off the edge"
        assert not off(A, 20, -0.2), 'a pad #291 keeps is a routing end'
        assert off(A, 25, -5.0), 'a bare point clearly off the board'
        # the pad test is per net: B's anchor on A's dropped pad is not on
        # a pad of B's, and at 0.45 off it is inside the point tolerance
        assert not off(B, 10, -0.45)
        assert off(B, 30, -0.45)
    print("  PASS: off-board point / dropped pad's copper are off; a kept "
          "edge pad and on-board copper are not")


def _run_oracle(links_round1):
    """One stubbed oracle round over net A. The routers are stubbed to fail,
    so every link it takes is left flagged. Returns (oracle result, points
    the link loop clustered, stdout)."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from routing_config import GridRouteConfig
    clustered = []
    calls = {'n': 0}

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        calls['n'] += 1
        return list(links_round1)

    real_cluster = kicad_oracle._cluster_points

    def spy_cluster(pcb_data, net_id, x, y, layer, comps, *a, **k):
        clustered.append((round(x, 3), round(y, 3)))
        return real_cluster(pcb_data, net_id, x, y, layer, comps, *a, **k)

    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected,
             kicad_exact_fill.refill_islands,
             plane_region_connector.route_plane_connection_wide,
             net_rescue._attempt_edge, kicad_oracle._cluster_points,
             kicad_oracle._direct_sliver_weld)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    kicad_exact_fill.refill_islands = lambda *a, **k: None
    plane_region_connector.route_plane_connection_wide = \
        lambda *a, **k: (None, 0)
    net_rescue._attempt_edge = lambda *a, **k: (None, None)
    kicad_oracle._cluster_points = spy_cluster
    kicad_oracle._direct_sliver_weld = lambda *a, **k: None
    out = io.StringIO()
    try:
        with tempfile.TemporaryDirectory() as td:
            board = os.path.join(td, 'b.kicad_pcb')
            _board(board,
                   [('R1', 5, 10, 'A', 0.6), ('R3', 15, 10, 'A', 0.6),
                    ('R5', 10, -10, 'A', 0.6), ('R8', 12, -10, 'A', 0.6)],
                   segments=[(5, 10, 9, 10, 0.2, 'F.Cu', 'A'),
                             (11, 10, 15, 10, 0.2, 'F.Cu', 'A')])
            cfg = GridRouteConfig(clearance=0.2, track_width=0.2,
                                  via_size=0.6, via_drill=0.3,
                                  layers=['F.Cu', 'B.Cu'],
                                  board_edge_clearance=0.2)
            with contextlib.redirect_stdout(out):
                orc = kicad_oracle.oracle_reconnect(
                    board, ['A'], cfg, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.2, max_rounds=1)
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge, kicad_oracle._cluster_points,
         kicad_oracle._direct_sliver_weld) = saved
    return orc, clustered, out.getvalue()


ON = ('A', (9.0, 10.0, 'F.Cu', 'track'), (11.0, 10.0, 'F.Cu', 'track'))
# from the off-board pad R5 back onto the board, and between R5 and R8
BACK = ('A', (10.0, -10.0, 'F.Cu', 'pad'), (9.0, 10.0, 'F.Cu', 'track'))
BETWEEN = ('A', (10.0, -10.0, 'F.Cu', 'pad'), (12.0, -10.0, 'F.Cu', 'pad'))


def test_oracle_never_routes_an_off_board_link():
    orc, clustered, log = _run_oracle([ON, BACK, BETWEEN])
    assert (9.0, 10.0) in clustered and (11.0, 10.0) in clustered, \
        f"fixture: the on-board link must reach the link loop {clustered}"
    off = [p for p in clustered if p[1] < 0]
    assert not off, f"the oracle routed toward an off-board pad: {off}"
    assert orc.get('remaining') == 3, orc.get('remaining')
    assert orc.get('off_board') == 2, orc.get('off_board')
    rl = orc.get('remaining_links')
    assert rl is not None and all(l[1][1] > 0 and l[2][1] > 0 for l in rl), \
        f"an off-board link reached remaining_links (custody, #572): {rl}"
    assert 'OUTSIDE the board outline' in log, log
    print(f"  PASS: on-board link routed; 2 off-board links exempt "
          f"(remaining {orc['remaining']}, remaining_links {len(rl)})")


def test_a_round_of_only_off_board_links_routes_nothing():
    orc, clustered, log = _run_oracle([BACK, BETWEEN])
    assert not clustered, f"links routed: {clustered}"
    assert orc.get('remaining') == 2 and orc.get('off_board') == 2, orc
    assert orc.get('remaining_links') == [], orc.get('remaining_links')
    assert orc.get('links_routed') == 0 and orc.get('links_failed') == 0
    assert 'all processed nets complete' not in log, log
    print("  PASS: nothing routed, the net reported open, no custody link")


TESTS = [test_link_end_test,
         test_oracle_never_routes_an_off_board_link,
         test_a_round_of_only_off_board_links_routes_nothing]


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
