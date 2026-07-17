#!/usr/bin/env python3
"""Issue #416: a track that terminates so its round end cap overlaps only the
CORNER of its target SMD pad joins through a copper web thinner than the board's
minimum track width (KiCad's connection_width class) -- DRC-clean and connected,
but a fab hazard. close_soft_joints firms such a joint by ADDING a short
connector into the pad interior (a parallel wide copper path); it must NOT touch
a legitimate landing (full overlap, or a perpendicular edge crossing whose web
is full-width), and must REFUSE a connector that would graze foreign copper.

Plain python3-runnable, no pytest. Boards are built as .kicad_pcb text and
parsed (synthetic segments carry a (uuid ...) as the parser expects). A thin
0.25 mm reference segment sets the board's connection_width floor below the
0.3 mm terminals, so joints have margin and the cases are unambiguous (a
terminal exactly at the floor width is inherently borderline)."""
import os
import sys
import tempfile
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb
from pcb_modification import close_soft_joints, terminal_web_neck_exact

FLOOR = 0.25   # min track width on the board (set by the reference segment).
TERM_W = 0.3   # terminal track width (> floor, so a good joint has margin).
# Pad net1 centred at (10,10), 1.0 x 1.0 rect on F.Cu -> spans [9.5,10.5]^2.
# A thin reference segment far from the pad pins the board floor at 0.25 mm.
REF_SEG = ((2.0, 2.0), (2.5, 2.0), FLOOR, 1)

_HEAD = """(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(general (thickness 1.6))
	(paper "A4")
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(setup (pad_to_mask_clearance 0))
	(net 0 "")
	(net 1 "N1")
	(net 2 "N2")
	(footprint "test:R"
		(layer "F.Cu")
		(uuid "aaaaaaaa-0000-0000-0000-000000000001")
		(at 10 10)
		(pad "1" smd rect (at 0 0) (size 1.0 1.0)
			(layers "F.Cu" "F.Mask" "F.Paste")
			(net 1 "N1")
			(uuid "aaaaaaaa-0000-0000-0000-0000000000a1"))
	)
"""

_SEG = """	(segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) (layer "F.Cu")
		(net {net}) (uuid "bbbbbbbb-0000-0000-0000-0000000{i:05d}"))
"""


def _write(segments):
    body = "".join(_SEG.format(x1=a[0], y1=a[1], x2=b[0], y2=b[1], w=w, net=n, i=i)
                   for i, (a, b, w, n) in enumerate(segments))
    d = tempfile.mkdtemp(prefix="web416_")
    p = os.path.join(d, "b.kicad_pcb")
    with open(p, "w") as f:
        f.write(_HEAD + body + ")\n")
    return p


def _cfg():
    return SimpleNamespace(clearance=0.15, track_width=TERM_W,
                           hole_to_hole_clearance=0.2, grid_step=0.1)


def _new_net1_web_at(pcb, x, y, tol=0.05):
    return [s for s in pcb.segments if s.net_id == 1 and s.layer == "F.Cu"
            and ((abs(s.start_x - x) < tol and abs(s.start_y - y) < tol)
                 or (abs(s.end_x - x) < tol and abs(s.end_y - y) < tol))]


def test_corner_graze_gets_connector():
    """E just outside the top-left corner (inside x by 0.03, outside y by 0.02):
    a real sub-floor web. A connector must be added and the neck resolved."""
    E = (9.53, 10.52)
    pcb = parse_kicad_pcb(_write([REF_SEG, ((9.53, 11.2), E, TERM_W, 1)]))
    assert terminal_web_neck_exact(pcb, 1, "F.Cu", E[0], E[1], FLOOR) is True, \
        "fixture is not a real neck -- adjust geometry"
    n = close_soft_joints([], pcb, {1}, _cfg())
    assert n >= 1, f"expected a connector, got {n}"
    assert _new_net1_web_at(pcb, E[0], E[1]), "no connector touches the endpoint"
    assert terminal_web_neck_exact(pcb, 1, "F.Cu", E[0], E[1], FLOOR) is False, \
        "neck not resolved after the connector"
    print("PASS corner graze -> connector added, web resolved")


def test_full_overlap_untouched():
    """A terminal that lands squarely in the pad interior is not a neck and must
    be left completely alone."""
    E = (10.0, 10.0)
    pcb = parse_kicad_pcb(_write([REF_SEG, ((10.0, 11.2), E, TERM_W, 1)]))
    assert terminal_web_neck_exact(pcb, 1, "F.Cu", E[0], E[1], FLOOR) is False
    before = len(pcb.segments)
    n = close_soft_joints([], pcb, {1}, _cfg())
    assert n == 0 and len(pcb.segments) == before, \
        f"a clean joint was modified ({n} connector(s))"
    print("PASS full overlap -> untouched")


def test_perpendicular_edge_entry_untouched():
    """A PERPENDICULAR crossing through an edge centre (cap far from any corner)
    has a full-width web even though the cap centre sits just outside the edge.
    The cheap pre-filter trips on it, but the exact erosion clears it, so NO
    connector is added -- the false-positive churn is suppressed."""
    E = (10.0, 10.52)  # above the top edge, centred in x -> wide overlap band
    pcb = parse_kicad_pcb(_write([REF_SEG, ((10.0, 11.2), E, TERM_W, 1)]))
    assert terminal_web_neck_exact(pcb, 1, "F.Cu", E[0], E[1], FLOOR) is False
    n = close_soft_joints([], pcb, {1}, _cfg())
    assert n == 0, f"perpendicular edge entry must not be touched, got {n}"
    print("PASS perpendicular edge entry -> untouched (no false positive)")


def test_connector_refused_when_it_would_graze_foreign():
    """Same corner graze, but foreign net-2 copper sits on the connector's path:
    the clearance gate must refuse it (the neck is then left for the DRC report
    rather than papered over with a shorting connector)."""
    E = (9.53, 10.52)
    pcb = parse_kicad_pcb(_write([
        REF_SEG,
        ((9.53, 11.2), E, TERM_W, 1),
        ((9.55, 10.30), (9.75, 10.30), 0.2, 2)]))  # foreign, on the route
    assert terminal_web_neck_exact(pcb, 1, "F.Cu", E[0], E[1], FLOOR) is True
    n = close_soft_joints([], pcb, {1}, _cfg())
    assert n == 0, f"connector must be refused near foreign copper, got {n}"
    print("PASS connector refused when it would graze foreign copper")


if __name__ == "__main__":
    test_corner_graze_gets_connector()
    test_full_overlap_untouched()
    test_perpendicular_edge_entry_untouched()
    test_connector_refused_when_it_would_graze_foreign()
    print("\nALL PASS")
