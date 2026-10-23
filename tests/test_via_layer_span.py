#!/usr/bin/env python3
"""An existing via blocks tracks only on the copper it spans.

A through via (F.Cu..B.Cu) occupies every layer. A blind or micro via occupies only
the layers between its two end layers: an F.Cu->In1.Cu GND microvia in a pad leaves
In2..B.Cu free for tracks. Every via used to be rasterized on all layers, so
via-in-pad microvias freed no routing space at all. Add and remove must stay
mirror images, or rip-up leaks ref-counts.

What makes a via blind is its TYPE word (`(via micro ...`), never its layer
pair: KiCad loads an untyped `(via ... (layers "F.Cu" "In1.Cu"))` as a THROUGH
via, and this repo's writer emits exactly that shape (zynq7020_som's back-side
BGA fanout writes `(layers "B.Cu" "In6.Cu")` GND vias). Reading the span off
the layer pair alone would let later routing cross those barrels.

The span is matched to the map by layer NAME: a full-stack run's config.layers
is not in stack order (`--layers F.Cu In2.Cu B.Cu` on a 6-layer board gives
[F.Cu, In2.Cu, B.Cu, In1.Cu, In3.Cu, In4.Cu]).

    python3 tests/test_via_layer_span.py
"""
import os
import sys
import tempfile

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from geometry_utils import via_span_layer_indices  # noqa: E402
from kicad_parser import Via, parse_kicad_pcb  # noqa: E402
from obstacle_cache import (add_net_obstacles_from_cache,  # noqa: E402
                            precompute_net_obstacles,
                            remove_net_obstacles_from_cache)
from obstacle_map import (GridObstacleMap, add_vias_list_as_obstacles,  # noqa: E402
                          remove_vias_list_from_obstacles)
from routing_config import GridCoord, GridRouteConfig  # noqa: E402

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']
# What `--layers F.Cu In2.Cu B.Cu` produces on this stack: the unrequested
# layers appended, FORBIDDEN, after B.Cu (route.py's full-stack normalization).
SHUFFLED = ['F.Cu', 'In2.Cu', 'B.Cu', 'In1.Cu', 'In3.Cu', 'In4.Cu']
FAILS = []

# A 6-layer board as KiCad 10 writes it (B.Cu is layer id 2), one net A with
# a typed microvia, an UNTYPED via whose layer pair is F.Cu/In1.Cu (a through
# via to KiCad), and a plain through via.
BOARD = '''(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(4 "In1.Cu" signal)
\t\t(6 "In2.Cu" signal)
\t\t(8 "In3.Cu" signal)
\t\t(10 "In4.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(25 "Edge.Cuts" user)
\t)
\t(net 0 "")
\t(net 1 "A")
\t(gr_line (start 0 0) (end 20 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e1"))
\t(gr_line (start 20 0) (end 20 10) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e2"))
\t(gr_line (start 20 10) (end 0 10) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e3"))
\t(gr_line (start 0 10) (end 0 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e4"))
\t(via micro (at 5 5) (size 0.25) (drill 0.1) (layers "F.Cu" "In1.Cu") (net 1) (uuid "v-micro"))
\t(via (at 10 5) (size 0.25) (drill 0.1) (layers "F.Cu" "In1.Cu") (net 1) (uuid "v-untyped"))
\t(via (at 15 5) (size 0.25) (drill 0.15) (layers "F.Cu" "B.Cu") (net 1) (uuid "v-through"))
)
'''


def check(label, ok, detail=None):
    print(("PASS " if ok else "FAIL ") + label
          + (f"  [{detail}]" if detail is not None and not ok else ""))
    if not ok:
        FAILS.append(label)


def _config(layers=LAYERS):
    cfg = GridRouteConfig()
    cfg.layers = list(layers)
    cfg.grid_step = 0.05
    cfg.clearance = 0.1
    cfg.track_width = 0.1
    cfg.via_size = 0.25
    cfg.via_drill = 0.15
    return cfg


def _blocked_layers(obstacles, gx, gy, n=len(LAYERS)):
    return [i for i in range(n) if obstacles.is_blocked(gx, gy, i)]


def _v(layers, via_type):
    return Via(x=0, y=0, size=0.25, drill=0.1, layers=layers, net_id=1,
               via_type=via_type)


def test_span_indices():
    span = via_span_layer_indices
    check("F.Cu->In1.Cu microvia spans layers 0..1",
          list(span(_v(['F.Cu', 'In1.Cu'], 'micro'), LAYERS, 6)) == [0, 1])
    check("B.Cu->In4.Cu microvia spans layers 4..5 (end order irrelevant)",
          list(span(_v(['B.Cu', 'In4.Cu'], 'micro'), LAYERS, 6)) == [4, 5])
    check("through via spans every layer",
          list(span(_v(['F.Cu', 'B.Cu'], 'through'), LAYERS, 6)) == list(range(6)))
    check("buried via In1..In2 on a 4-layer routing set maps by name",
          list(span(_v(['In1.Cu', 'In2.Cu'], 'buried'),
                    ['F.Cu', 'In2.Cu', 'In3.Cu', 'B.Cu'], 4)) == [1])
    check("unknown layers fall back to every layer",
          list(span(_v(None, 'micro'), LAYERS, 6)) == list(range(6)))
    # The type decides, not the layer pair: an untyped via is a through via
    # to KiCad, wherever its (layers ...) points.
    check("an UNTYPED via whose pair is F.Cu/In1.Cu spans every layer",
          list(span(_v(['F.Cu', 'In1.Cu'], 'through'), LAYERS, 6)) == list(range(6)))
    check("...and so does the zynq7020_som shape (B.Cu/In6.Cu, untyped)",
          list(span(_v(['B.Cu', 'In6.Cu'], 'through'),
                    LAYERS[:-1] + ['In5.Cu', 'In6.Cu', 'B.Cu'], 8)) == list(range(8)))
    check("a Via built without a type is a through via",
          list(span(Via(x=0, y=0, size=0.3, drill=0.15, layers=['F.Cu', 'In2.Cu'],
                        net_id=1), LAYERS, 6)) == list(range(6)))
    # Full-stack order: a `--layers F.Cu In2.Cu B.Cu` run.
    check("shuffled routing order: F.Cu->In1.Cu microvia is indices 0 and 3 "
          "(F.Cu, In1.Cu), not 0..1 (F.Cu, In2.Cu)",
          list(span(_v(['F.Cu', 'In1.Cu'], 'micro'), SHUFFLED, 6)) == [0, 3])
    check("shuffled routing order: In4.Cu->B.Cu microvia is indices 2 and 5",
          list(span(_v(['In4.Cu', 'B.Cu'], 'micro'), SHUFFLED, 6)) == [2, 5])
    check("shuffled routing order: buried In1..In3 is In2, In1, In3",
          list(span(_v(['In1.Cu', 'In3.Cu'], 'buried'), SHUFFLED, 6)) == [1, 3, 4])
    # Conservative edges: never fewer layers than the via may have copper on.
    check("a routing layer without a stack name counts as spanned",
          list(span(_v(['F.Cu', 'In1.Cu'], 'micro'), ['F.Cu', 'Mid', 'B.Cu'], 3)) == [0, 1])
    check("a map with more layers than names gets every layer",
          list(span(_v(['F.Cu', 'In1.Cu'], 'micro'), LAYERS[:3], 6)) == list(range(6)))


def test_parser_reads_the_type():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'span.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(BOARD)
        pcb = parse_kicad_pcb(path)
    by_uuid = {v.uuid: v for v in pcb.vias}
    check("text parser: `(via micro` is via_type micro",
          by_uuid['v-micro'].via_type == 'micro')
    check("text parser: an untyped via is via_type through even with an "
          "F.Cu/In1.Cu layer pair", by_uuid['v-untyped'].via_type == 'through')
    # ...and its pair is snapped to F.Cu/B.Cu, as KiCad loads it and as
    # build_pcb_data_from_board reports it: every consumer that still reads
    # the pair sees a through via, and the two parse paths agree.
    check("text parser: an untyped F.Cu/In1.Cu via reads layers F.Cu/B.Cu",
          by_uuid['v-untyped'].layers == ['F.Cu', 'B.Cu'],
          by_uuid['v-untyped'].layers)
    check("text parser: a microvia keeps its own pair (control)",
          by_uuid['v-micro'].layers == ['F.Cu', 'In1.Cu'], by_uuid['v-micro'].layers)
    check("text parser: plain through via", by_uuid['v-through'].via_type == 'through')
    return pcb


def test_parser_reads_the_pair_top_first():
    """A pair written bottom first reads top layer first, as pcbnew's
    TopLayer()/BottomLayer() read it back on the GUI path: a through via
    `(layers "B.Cu" "F.Cu")` (this repo's writer emits them: zynq7020_som's
    Type VII vias) and typed pairs either way round. Placement Apply keys a
    via on its ORDERED pair, so the paths disagreeing re-created every such
    via, with a new uuid, on every apply."""
    vias = ('\t(via (at 10 5) (size 0.25) (drill 0.1) (layers "F.Cu" "In1.Cu") '
            '(net 1) (uuid "v-untyped"))\n')
    reversed_vias = (
        '\t(via (at 10 5) (size 0.3) (drill 0.15) (layers "B.Cu" "F.Cu") (net 1) (uuid "r-through"))\n'
        '\t(via blind (at 11 5) (size 0.3) (drill 0.15) (layers "In2.Cu" "F.Cu") (net 1) (uuid "r-blind"))\n'
        '\t(via micro (at 12 5) (size 0.25) (drill 0.1) (layers "B.Cu" "In4.Cu") (net 1) (uuid "r-micro"))\n')
    assert BOARD.count(vias) == 1
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'reversed.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(BOARD.replace(vias, reversed_vias))
        pcb = parse_kicad_pcb(path)
    by_uuid = {v.uuid: v for v in pcb.vias}
    for uuid, want in (('r-through', ['F.Cu', 'B.Cu']), ('r-blind', ['F.Cu', 'In2.Cu']),
                       ('r-micro', ['In4.Cu', 'B.Cu'])):
        got = by_uuid[uuid].layers if uuid in by_uuid else None
        check(f"text parser: {uuid} written bottom first reads {want}", got == want, got)


def test_microvia_frees_inner_layers_and_balances():
    cfg = _config()
    coord = GridCoord(cfg.grid_step)
    micro = Via(x=5.0, y=5.0, size=0.25, drill=0.1, layers=['F.Cu', 'In1.Cu'],
                net_id=1, via_type='micro')
    untyped = Via(x=11.0, y=5.0, size=0.25, drill=0.1, layers=['F.Cu', 'In1.Cu'],
                  net_id=1)
    through = Via(x=8.0, y=5.0, size=0.25, drill=0.15, layers=['F.Cu', 'B.Cu'], net_id=1)
    vias = [micro, untyped, through]
    obstacles = GridObstacleMap(len(LAYERS))
    add_vias_list_as_obstacles(obstacles, vias, cfg)
    gx, gy = coord.to_grid(micro.x, micro.y)
    check("microvia blocks tracks on F.Cu and In1.Cu only", _blocked_layers(obstacles, gx, gy) == [0, 1])
    tx, ty = coord.to_grid(through.x, through.y)
    check("through via still blocks every layer", _blocked_layers(obstacles, tx, ty) == list(range(6)))
    ux, uy = coord.to_grid(untyped.x, untyped.y)
    check("an untyped F.Cu/In1.Cu via blocks every layer (KiCad: through)",
          _blocked_layers(obstacles, ux, uy) == list(range(6)))
    check("microvia still blocks a new via at its spot", obstacles.is_via_blocked(gx, gy))
    remove_vias_list_from_obstacles(obstacles, vias, cfg)
    pts = ((gx, gy), (tx, ty), (ux, uy))
    clean = all(not obstacles.is_blocked(x, y, layer)
                for x, y in pts for layer in range(len(LAYERS)))
    check("remove mirrors add: no cell left blocked", clean)
    check("remove mirrors add: via map clear",
          not any(obstacles.is_via_blocked(x, y) for x, y in pts))


def test_shuffled_map():
    cfg = _config(SHUFFLED)
    coord = GridCoord(cfg.grid_step)
    micro = Via(x=5.0, y=5.0, size=0.25, drill=0.1, layers=['F.Cu', 'In1.Cu'],
                net_id=1, via_type='micro')
    obstacles = GridObstacleMap(len(SHUFFLED))
    add_vias_list_as_obstacles(obstacles, [micro], cfg)
    gx, gy = coord.to_grid(micro.x, micro.y)
    got = [SHUFFLED[i] for i in _blocked_layers(obstacles, gx, gy)]
    check("shuffled map: the microvia blocks the F.Cu and In1.Cu PLANES of the map",
          got == ['F.Cu', 'In1.Cu'])
    remove_vias_list_from_obstacles(obstacles, [micro], cfg)
    check("shuffled map: remove mirrors add", not _blocked_layers(obstacles, gx, gy))


def test_per_net_cache(pcb):
    """The per-net cache (_collect_via_obstacles) is the other rasterizer: it
    must span exactly as the list path does, and add/remove through it must
    balance."""
    cfg = _config()
    coord = GridCoord(cfg.grid_step)
    cache = precompute_net_obstacles(pcb, 1, cfg)
    via_map = GridObstacleMap(len(LAYERS))
    add_vias_list_as_obstacles(via_map, [v for v in pcb.vias if v.net_id == 1], cfg)
    cache_map = GridObstacleMap(len(LAYERS))
    add_net_obstacles_from_cache(cache_map, cache)
    for v in pcb.vias:
        gx, gy = coord.to_grid(v.x, v.y)
        want = [0, 1] if v.via_type == 'micro' else list(range(6))
        check(f"per-net cache: {v.uuid} blocks {want}",
              _blocked_layers(cache_map, gx, gy) == want)
        ring = [(gx + d, gy) for d in range(-8, 9)]
        check(f"per-net cache: {v.uuid} matches the list path cell by cell",
              all(_blocked_layers(cache_map, x, y) == _blocked_layers(via_map, x, y)
                  for x, y in ring))
    remove_net_obstacles_from_cache(cache_map, cache)
    check("per-net cache: remove mirrors add",
          all(not _blocked_layers(cache_map, *coord.to_grid(v.x, v.y))
              and not cache_map.is_via_blocked(*coord.to_grid(v.x, v.y))
              for v in pcb.vias))


def main():
    test_span_indices()
    pcb = test_parser_reads_the_type()
    test_parser_reads_the_pair_top_first()
    test_microvia_frees_inner_layers_and_balances()
    test_shuffled_map()
    test_per_net_cache(pcb)
    if FAILS:
        print(f"\n{len(FAILS)} FAILED")
        return 1
    print("\nall passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
