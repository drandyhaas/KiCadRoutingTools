#!/usr/bin/env python3
"""An oracle link's NEW vias keep the drill hole-to-hole floor from EACH OTHER.

oracle_reconnect routes each link with route_plane_connection_wide, whose
search never spaces the path's own drills (via_exclusion_radius 0). Its
same-net drill guard then held every new via off the net's EXISTING holes
(vias and plated pad barrels, #282/#479) and bridged a too-close one to the
barrel (#340) -- but never compared the new vias with one another. A 0.8 mm
width-upgrade route on a corpus board shipped two GND vias one diagonal grid
cell apart (centres 0.283 mm, floor 0.25 drill + 0.25 hole-to-hole), joined
by one inner-layer hop: a KiCad hole-to-hole violation.

The guard now checks each new via against the link's KEPT new vias too, at
the run's hole-to-hole value (the oracle's argument, not the config's 0.20
default). When the two layer changes are consecutive along the path, the hop
between them moves onto the layer the path leaves the second one on and that
hole is never drilled (a dip, in -> mid -> in, needs neither hole); otherwise
the later via is bridged to the kept one like an existing barrel.

The link source, the refill and the router are stubbed (as in
test_oracle_hole_floor_from_project), so no KiCad is needed. Each case hands
the oracle one path with known new vias and reads the board it writes back:

  * fold   -- F->In1 at K, In1 hop 0.47 mm, In1->B: one hole, the hop on B.Cu
              (0.47 clears the 0.20 default floor, so this also pins that the
              run's 0.25 is the floor used)
  * dip    -- F->In1 at K, In1 diagonal hop, In1->F, then a far F->B via:
              only the far hole
  * bridge -- K, a far via M, then back to V near K: V dropped, bridged to K
  * apart  -- two new vias 0.6 mm apart: both kept (the guard does not fire)

Each case asserts the oracle reported the link OK with the expected via count,
that no two vias of the net sit closer than the floor, and that the net's
copper still joins the link's two ends.

    python3 tests/test_oracle_link_own_via_h2h.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from run_utils import evidence                          # noqa: E402

fails = []

DRILL = 0.25
H2H = 0.25                      # the run's floor; the config default is 0.20
FLOOR = DRILL + H2H             # centre-to-centre for two new 0.25 drills
A_END = (10.0, 10.0, 'F.Cu')
B_END = (16.0, 10.0, 'B.Cu')


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


def _board(path):
    """A 4-layer KiCad-10 board: net A's F.Cu piece ending at A_END and its
    B.Cu piece starting at B_END (the link's two ends), no vias."""
    with open(path, 'w', encoding='utf-8') as f:
        f.write('''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal)
    (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (footprint "T:P" (layer "F.Cu") (at 5 10)
    (property "Reference" "R1" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "A")))
  (segment (start 5 10) (end 10 10) (width 0.2) (layer "F.Cu") (net "A"))
  (segment (start 16 10) (end 20 10) (width 0.2) (layer "B.Cu") (net "A"))
)
''')


# Each case: the path the stubbed router returns (x, y, layer), the new vias
# it reports (every layer change), and the via count the link should ship.
F, I1, I2, B = 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'
CASES = {
    'fold': ([A_END, (12, 10, F), (12, 10, I1), (12.47, 10, I1), (12.47, 10, B),
              (16, 10, B)],
             [(12, 10), (12.47, 10)], 1),
    'dip': ([A_END, (12, 10, F), (12, 10, I1), (12.2, 10.2, I1), (12.2, 10.2, F),
             (14, 10.2, F), (14, 10.2, B), (16, 10.2, B), (16, 10, B)],
            [(12, 10), (12.2, 10.2), (14, 10.2)], 1),
    'bridge': ([A_END, (12, 10, F), (12, 10, I1), (13, 10, I1), (13, 10, I2),
                (12.2, 10.3, I2), (12.2, 10.3, B), (16, 10.3, B), (16, 10, B)],
               [(12, 10), (13, 10), (12.2, 10.3)], 2),
    'apart': ([A_END, (12, 10, F), (12, 10, I1), (12.6, 10, I1), (12.6, 10, B),
               (16, 10, B)],
              [(12, 10), (12.6, 10)], 2),
}


def _run(path_pts, new_vias):
    """One stubbed oracle round over a fresh board. Returns (pcb of the
    written board, oracle stdout, number of router calls)."""
    import kicad_oracle
    import kicad_exact_fill
    import plane_region_connector
    import net_rescue
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    calls = {'exact': 0, 'route': 0}

    def fake_exact(board_file, net_names=None, pcb_data=None,
                   verbose=False, project_from=None):
        calls['exact'] += 1
        if calls['exact'] > 1:
            return []
        return [('A', A_END + ('track',), B_END + ('track',))]

    def fake_route(*a, **k):
        calls['route'] += 1
        return (list(path_pts), list(new_vias)), 1

    saved = (kicad_exact_fill.exact_unconnected,
             kicad_oracle.kicad_unconnected,
             kicad_exact_fill.refill_islands,
             plane_region_connector.route_plane_connection_wide,
             net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld)
    kicad_exact_fill.exact_unconnected = fake_exact
    kicad_oracle.kicad_unconnected = lambda *a, **k: None
    kicad_exact_fill.refill_islands = lambda *a, **k: None
    plane_region_connector.route_plane_connection_wide = fake_route
    net_rescue._attempt_edge = lambda *a, **k: (None, None)
    kicad_oracle._direct_sliver_weld = lambda *a, **k: None
    out = io.StringIO()
    try:
        with tempfile.TemporaryDirectory() as td:
            board = os.path.join(td, 'b.kicad_pcb')
            _board(board)
            cfg = GridRouteConfig(clearance=0.1, track_width=0.2,
                                  via_size=0.45, via_drill=DRILL,
                                  layers=[F, I1, I2, B])
            with contextlib.redirect_stdout(out):
                kicad_oracle.oracle_reconnect(
                    board, ['A'], cfg, track_via_clearance=0.2,
                    hole_to_hole_clearance=H2H, max_rounds=1)
            pcb = parse_kicad_pcb(evidence(board, 'the board the oracle wrote'))
    finally:
        (kicad_exact_fill.exact_unconnected, kicad_oracle.kicad_unconnected,
         kicad_exact_fill.refill_islands,
         plane_region_connector.route_plane_connection_wide,
         net_rescue._attempt_edge, kicad_oracle._direct_sliver_weld) = saved
    return pcb, out.getvalue(), calls['route']


def _joined(pcb, nid, a, b):
    """True when net nid's copper joins point a to point b ((x, y, layer)):
    segment ends on one layer, vias on every layer, ends matched to 1 um."""
    parent = {}

    def key(x, y, layer):
        return (round(x, 3), round(y, 3), layer)

    def find(n):
        parent.setdefault(n, n)
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    def union(p, q):
        parent[find(p)] = find(q)
    for s in pcb.segments:
        if s.net_id == nid:
            union(key(s.start_x, s.start_y, s.layer), key(s.end_x, s.end_y, s.layer))
    for v in pcb.vias:
        if v.net_id == nid:
            for layer in (F, I1, I2, B):
                union(key(v.x, v.y, F), key(v.x, v.y, layer))
    return find(key(*a)) == find(key(*b))


for name, (pts, vias, want) in CASES.items():
    print(f"{name}:")
    pcb, log, n_route = _run(pts, vias)
    check("the stubbed router was asked for the link", n_route > 0)
    ok_line = next((ln for ln in log.splitlines() if ' OK ' in ln and 'A:' in ln), '')
    check("the oracle reported the link OK (not failed or declined)", bool(ok_line),
          ok_line.strip() or log[-400:])
    check(f"...with {want} via(s)", f" {want} via(s)" in ok_line, ok_line.strip())
    nid = next(i for i, n in pcb.nets.items() if n.name == 'A')
    nv = [v for v in pcb.vias if v.net_id == nid]
    check(f"the board carries {want} net-A via(s)", len(nv) == want,
          f"got {[(v.x, v.y) for v in nv]}")
    close = [((p.x, p.y), (q.x, q.y), round(math.hypot(p.x - q.x, p.y - q.y), 4))
             for i, p in enumerate(nv) for q in nv[i + 1:]
             if math.hypot(p.x - q.x, p.y - q.y) < FLOOR - 1e-6]
    check(f"no two vias closer than drill + h2h = {FLOOR}", not close, f"{close}")
    check("the net's copper still joins the link's two ends",
          _joined(pcb, nid, A_END, B_END))

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")
