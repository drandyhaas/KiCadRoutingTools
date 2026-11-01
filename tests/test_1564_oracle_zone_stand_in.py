#!/usr/bin/env python3
"""#1564: a zone end off the outline is a stand-in; the oracle straps the
real end's cluster at its true gap.

kicad-cli anchors a zone end of an unconnected link at the zone outline's
first corner. For a coincident zone|zone link the oracle already reads that
anchor as a stand-in and derives the gap from KiCad's exact islands and the
copper union-find (cluster mode, #648/#666). A zone|track link has two
different anchors, so it never took that path: on a non-rectangular board the
corner lies outside the outline, no island holds it, and the strap had no
side to start from -- the weld failed and the net shipped open. What each
case pins:

* `_exact_fill_endpoints` resolves such a link to the cluster the TRACK end
  lies on and the main cluster, at their nearest approach -- not to a nearer
  unrelated orphan, which is what the smallest-gap rule of the coincident
  case would pick;
* the link loop takes cluster mode for it (a zone end off the board) and
  rebinds the link to that gap, and never makes a raw retry toward the
  off-board corner;
* such links go last in a round, and one whose real split a link of the
  round already took (an exact-source link naming the same gap) is not
  welded a second time.

The exact islands, the link source and the routers are stubbed, so no KiCad
is needed.

    python3 tests/test_1564_oracle_zone_stand_in.py [case-substring ...]
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))


def _rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


# Net A's B.Cu fill in three islands: the main one (x 1..20), a small orphan
# 1 mm from it (the decoy), and the orphan the track sits on, 5 mm away.
MAIN = _rect(1.0, 1.0, 20.0, 39.0)
DECOY = _rect(21.0, 30.0, 22.0, 31.0)
ORPHAN = _rect(25.0, 1.0, 39.0, 39.0)
ISLANDS = {('A', 'B.Cu'): [MAIN, DECOY, ORPHAN]}

# kicad-cli's link: the zone outline's first corner, off the 0..40 board,
# and a track end on the orphan
CORNER = (-3.0, -3.0, 'B.Cu', 'zone')
TRACK = (30.0, 20.0, 'B.Cu', 'track')


def _board(path):
    segs = [(5, 20, 10, 20), (30, 20, 35, 20)]
    s = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width 0.2) '
                f'(layer "B.Cu") (net "A"))' for a, b, c, d in segs)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts")){s}
)
''')


def test_endpoints_strap_the_track_cluster():
    from kicad_parser import parse_kicad_pcb
    from kicad_oracle import _exact_fill_endpoints
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        _board(b)
        pcb = parse_kicad_pcb(b)
    nid = next(i for i, n in pcb.nets.items() if n.name == 'A')
    for A, B in ((CORNER, TRACK), (TRACK, CORNER)):
        r = _exact_fill_endpoints(pcb, nid, 'A', A, B, ISLANDS,
                                  track_half=0.1, edge_clearance=0.2)
        assert r is not None and r != 'cross-board', \
            f"no strap for a zone|track link with an off-board zone end: {r}"
        src, tgt, pa, pb, layer = r
        xs = sorted((pa[0], pb[0]))
        assert abs(xs[0] - 20.0) < 0.3 and abs(xs[1] - 25.0) < 0.3, \
            (f"strap ({pa[0]:.2f},{pa[1]:.2f})<->({pb[0]:.2f},{pb[1]:.2f}) "
             f"is not the track's orphan <-> main gap (x 25 <-> 20)")
        assert src and tgt and layer == 'B.Cu'
    print("  PASS: the strap joins the track's orphan to the main island at "
          "x 25 <-> 20, not the nearer decoy")


def _run_oracle(exact_links, cli_links=None):
    """One stubbed oracle run over net A (routers fail); returns its log."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from routing_config import GridRouteConfig
    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected,
             kicad_exact_fill.refill_islands,
             plane_region_connector.route_plane_connection_wide,
             net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld)
    kicad_exact_fill.exact_unconnected = lambda *a, **k: list(exact_links)
    kicad_oracle.kicad_unconnected = \
        lambda *a, **k: None if cli_links is None else list(cli_links)
    kicad_exact_fill.refill_islands = \
        lambda *a, **k: {k_: [list(p) for p in v] for k_, v in ISLANDS.items()}
    plane_region_connector.route_plane_connection_wide = \
        lambda *a, **k: (None, 0)
    net_rescue._attempt_edge = lambda *a, **k: (None, None)
    kicad_oracle._direct_sliver_weld = lambda *a, **k: None
    out = io.StringIO()
    try:
        with tempfile.TemporaryDirectory() as td:
            board = os.path.join(td, 'b.kicad_pcb')
            _board(board)
            cfg = GridRouteConfig(clearance=0.2, track_width=0.2,
                                  via_size=0.6, via_drill=0.3,
                                  layers=['F.Cu', 'B.Cu'],
                                  board_edge_clearance=0.2)
            with contextlib.redirect_stdout(out):
                kicad_oracle.oracle_reconnect(
                    board, ['A'], cfg, track_via_clearance=0.2,
                    hole_to_hole_clearance=0.2, max_rounds=2)
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld) = saved
    return out.getvalue()


def test_link_loop_takes_cluster_mode():
    log = _run_oracle([('A', CORNER, TRACK)])
    line = next((ln for ln in log.splitlines()
                 if 'cluster mode: true gap' in ln), None)
    assert line is not None, \
        f"the off-board zone|track link never took cluster mode:\n{log}"
    assert '25.00' in line and '20.00' in line, line
    assert 'retry with raw endpoints' not in log, \
        f"a raw retry toward the off-board corner:\n{log}"
    print(f"  PASS: {line.strip()}")


# the exact source's own link for the track's orphan, at the gap cluster
# mode derives for it (a track|zone link: a zone|zone one is split into two
# links to the main cluster); it sorts AFTER the stand-in
EXACT = ('A', (25.0, 1.0, 'B.Cu', 'track'), (20.0, 1.0, 'B.Cu', 'zone'))


def test_a_split_already_taken_is_not_welded_twice():
    log = _run_oracle([EXACT], cli_links=[('A', CORNER, TRACK)])
    lines = log.splitlines()
    first = next((i for i, ln in enumerate(lines)
                  if '(25.00,1.00)<->(20.00,1.00)' in ln), None)
    stand = next((i for i, ln in enumerate(lines)
                  if 'cluster mode: true gap (25.00,1.00)' in ln), None)
    assert first is not None and stand is not None and first < stand, \
        f"the exact link must be taken first, the stand-in after:\n{log}"
    assert 'not welded twice' in log, log
    print("  PASS: the exact link takes the split first; the stand-in that "
          "names it is not welded again")


TESTS = [test_endpoints_strap_the_track_cluster,
         test_link_loop_takes_cluster_mode,
         test_a_split_already_taken_is_not_welded_twice]


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
