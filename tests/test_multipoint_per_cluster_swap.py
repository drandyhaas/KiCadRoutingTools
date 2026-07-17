#!/usr/bin/env python3
"""Per-cluster layer assignment fallback for multi-point nets (issue #265 follow-up).

`apply_single_ended_layer_swaps` PHASE 3 first tries to collapse every endpoint
of a multi-point net (3+ unconnected terminals) onto ONE common layer. When no
single layer serves every endpoint (e.g. two bare SMD pads pinned to different
inner layers), it used to give up (-1) and route the whole net with as many
layer-change vias as the geometry forced.

It now falls back to a per-cluster assignment: a greedy set cover picks the
FEWEST distinct layers that cover all endpoints, and only the stubs whose
assigned layer differs from their current one are relayered (each through the
existing apply_stub_layer_switch + validators). This test covers:

  1. PER-CLUSTER WIN: no common layer exists, but two layers cover all three
     endpoints. Assert exactly 2 layers are used and only the one off-cluster
     stub is moved (via the fallback, not the common-layer path).
  2. COMMON-LAYER STILL WINS: a single common layer is feasible -> the original
     collapse path handles it and the fallback is never reached (unchanged).
  3. VALIDATION REFUSAL: the assigned target layer is blocked by foreign copper
     -> the move is refused, the endpoint stays on its current layer, the board
     is left unchanged, and nothing crashes.

Run:  python3 tests/test_multipoint_per_cluster_swap.py
"""
import io
import os
import sys
import tempfile
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from routing_config import GridRouteConfig
from layer_swap_optimization import apply_single_ended_layer_swaps

LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]


def _config():
    return GridRouteConfig(layers=LAYERS, grid_step=0.1, track_width=0.2,
                           clearance=0.1, via_size=0.4, via_drill=0.2)


def _write(text):
    f = tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False)
    f.write(text)
    f.close()
    return f.name


def _edge():
    return ('\t(gr_line (start 0 0) (end 30 0) (layer "Edge.Cuts") (width 0.1))\n'
            '\t(gr_line (start 30 0) (end 30 12) (layer "Edge.Cuts") (width 0.1))\n'
            '\t(gr_line (start 30 12) (end 0 12) (layer "Edge.Cuts") (width 0.1))\n'
            '\t(gr_line (start 0 12) (end 0 0) (layer "Edge.Cuts") (width 0.1))\n')


def _net_id(pcb, name):
    for nid, n in pcb.nets.items():
        if n.name == name:
            return nid
    return None


def _stub_layer(pcb, net_id):
    """Layer of the /MULTI dangling stub segment (there is exactly one)."""
    segs = [s for s in pcb.segments if s.net_id == net_id]
    assert len(segs) == 1, f"expected 1 stub segment, got {len(segs)}"
    return segs[0].layer


# ---------------------------------------------------------------------------
# Case 1: per-cluster win. P1 pinned to In1.Cu, P2 pinned to In2.Cu (no common
# layer possible), plus a genuinely movable fanout stub: an F.Cu SMD pad with a
# through via-in-pad escaping to a dangling B.Cu stub. Two layers {In1.Cu,
# In2.Cu} cover everything; the escaped stub is the only off-cluster endpoint.
# ---------------------------------------------------------------------------
def _board_per_cluster(block_target=False):
    # Optional foreign-net track laid on In1.Cu exactly where the stub would be
    # relayered, to force validation refusal (Case 3).
    blocker = ''
    if block_target:
        blocker = ('\t(net 2 "/OTHER")\n'
                   '\t(segment (start 18 3) (end 22 3) (width 0.2) '
                   '(layer "In1.Cu") (net 2) (uuid "blk"))\n')
    return f'''(kicad_pcb
\t(version 20221018)
\t(net 0 "")
\t(net 1 "/MULTI")
{blocker}{_edge()}\t(footprint "L:A" (layer "F.Cu") (at 3 3)
\t\t(property "Reference" "U1")
\t\t(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "In1.Cu") (net 1 "/MULTI")))
\t(footprint "L:B" (layer "F.Cu") (at 27 3)
\t\t(property "Reference" "U2")
\t\t(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "In2.Cu") (net 1 "/MULTI")))
\t(footprint "L:C" (layer "F.Cu") (at 18 3)
\t\t(property "Reference" "U3")
\t\t(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/MULTI")))
\t(via (at 18 3) (size 0.4) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1) (uuid "v3"))
\t(segment (start 18 3) (end 22 3) (width 0.2) (layer "B.Cu") (net 1) (uuid "s3"))
)'''


def _board_common_wins():
    """P1 pinned to In1.Cu, P2 a through-hole pad (all layers), plus a movable
    B.Cu fanout stub (F.Cu SMD pad + through via-in-pad). In1.Cu is feasible for
    EVERY endpoint -> the common-layer collapse applies (stub B.Cu -> In1.Cu) and
    the per-cluster fallback is never reached."""
    return f'''(kicad_pcb
\t(version 20221018)
\t(net 0 "")
\t(net 1 "/MULTI")
{_edge()}\t(footprint "L:A" (layer "F.Cu") (at 3 3)
\t\t(property "Reference" "U1")
\t\t(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "In1.Cu") (net 1 "/MULTI")))
\t(footprint "L:B" (layer "F.Cu") (at 27 3)
\t\t(property "Reference" "U2")
\t\t(pad "1" thru_hole circle (at 0 0) (size 0.8 0.8) (drill 0.4) (layers "*.Cu") (net 1 "/MULTI")))
\t(footprint "L:C" (layer "F.Cu") (at 18 3)
\t\t(property "Reference" "U3")
\t\t(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/MULTI")))
\t(via (at 18 3) (size 0.4) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1) (uuid "v3"))
\t(segment (start 18 3) (end 22 3) (width 0.2) (layer "B.Cu") (net 1) (uuid "s3"))
)'''


def _run_swaps(pcb, config):
    """Invoke the real PHASE-3 path; capture stdout to tell which branch ran."""
    mods, vias, segs = [], [], []
    net_id = _net_id(pcb, "/MULTI")
    buf = io.StringIO()
    with redirect_stdout(buf):
        total = apply_single_ended_layer_swaps(
            pcb, config, [("/MULTI", net_id)],
            can_swap_to_top_layer=True,
            all_segment_modifications=mods,
            all_swap_vias=vias,
            all_swap_segments=segs,
            verbose=True,
        )
    return total, buf.getvalue(), net_id


def test_per_cluster_win():
    pcb = parse_kicad_pcb(_write(_board_per_cluster()))
    config = _config()
    net_id = _net_id(pcb, "/MULTI")
    assert _stub_layer(pcb, net_id) == "B.Cu"

    total, out, net_id = _run_swaps(pcb, config)

    assert "per-cluster" in out, f"fallback not used:\n{out}"
    assert "common-layer collapse" not in out, f"common path fired unexpectedly:\n{out}"
    # Exactly one stub moved, onto one of the two chosen inner layers.
    assert total == 1, f"expected 1 stub moved, got {total}\n{out}"
    assert "2 layer(s)" in out, f"expected a 2-layer assignment:\n{out}"
    new_layer = _stub_layer(pcb, net_id)
    assert new_layer in ("In1.Cu", "In2.Cu"), f"stub moved to {new_layer}, not a cluster layer"
    assert new_layer != "B.Cu"
    print(f"  [1] per-cluster win: stub B.Cu -> {new_layer}, 2 layers, 1 move  OK")


def test_common_layer_still_wins():
    pcb = parse_kicad_pcb(_write(_board_common_wins()))
    config = _config()
    net_id = _net_id(pcb, "/MULTI")

    total, out, net_id = _run_swaps(pcb, config)

    assert "Multi-point collapse" in out, f"common-layer path did not fire:\n{out}"
    assert "per-cluster" not in out, f"fallback wrongly used when a common layer exists:\n{out}"
    assert total == 1, f"expected 1 stub moved by the common collapse, got {total}\n{out}"
    assert _stub_layer(pcb, net_id) == "In1.Cu", "stub should collapse onto the common In1.Cu"
    print("  [2] common-layer path unchanged: stub B.Cu -> In1.Cu via collapse  OK")


def test_validation_refusal_degrades():
    pcb = parse_kicad_pcb(_write(_board_per_cluster(block_target=True)))
    config = _config()
    net_id = _net_id(pcb, "/MULTI")
    assert _stub_layer(pcb, net_id) == "B.Cu"
    n_segs_before = len(pcb.segments)
    n_vias_before = len(pcb.vias)

    total, out, net_id = _run_swaps(pcb, config)

    # The assigned target layer is blocked -> no move applied, board unchanged.
    assert total == 0, f"expected no swaps applied, got {total}\n{out}"
    assert _stub_layer(pcb, net_id) == "B.Cu", "stub must stay on its current layer when refused"
    assert len(pcb.segments) == n_segs_before, "no segments should be added on refusal"
    assert len(pcb.vias) == n_vias_before, "no vias should be added on refusal"
    print("  [3] validation refusal degrades gracefully: stub stays on B.Cu  OK")


if __name__ == "__main__":
    test_per_cluster_win()
    test_common_layer_still_wins()
    test_validation_refusal_degrades()
    print("All per-cluster multi-point swap tests passed.")
