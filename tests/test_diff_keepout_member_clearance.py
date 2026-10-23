#!/usr/bin/env python3
"""A diff pair's MEMBERS stay out of native and user-layer keep-outs.

route_diff routes a pair's centreline on a map whose every obstacle is grown
by the member offset, (track_width + diff_pair_gap) / 2, so the members, not
the centreline, keep their clearance. The two keep-out passes in
build_base_obstacle_map took no such extra clearance: the centreline kept the
keep-out's band and a member, offset to its side, reached into the area.
Measured on lvds_converter_dualclk with a rule area across /CLK's path:
/CLK+'s copper 0.5 um inside a native keep-out (KiCad 10: items_not_allowed),
and /CLK-'s centreline on a User.2 keep-out's edge (found reviewing #1278,
whose net-filtered regions are stamped with the member offset).

Invariants:
1. Both passes block exactly the band their geometry defines, checked against
   an independent per-cell oracle: a rule area at clearance + half the item +
   extra, a user-layer polygon at extra (its interior alone at 0, as before).
2. build_base_obstacle_map threads its extra clearance into both: a cell the
   diff map blocks is open on the single-ended map built from the same board.
3. End to end (route_diff): the members' copper stays out of a native
   keep-out (KiCad's DRC agrees when kicad-cli is installed) and their
   centrelines stay out of a user-layer one.

Run:
    python3 tests/test_diff_keepout_member_clearance.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb, pcb_uses_name_nets  # noqa: E402
from routing_config import GridCoord, GridRouteConfig  # noqa: E402

LVDS = os.path.join(ROOT, "kicad_files", "lvds_converter_dualclk.kicad_pcb")
BOX = [(86.5, 58.0), (88.5, 58.0), (88.5, 60.2), (86.5, 60.2)]
ENV = dict(os.environ, MSYS2_ARG_CONV_EXCL='*')


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


class _Rec:
    """Records the cells a keep-out pass stamps."""

    def __init__(self):
        self.cells, self.vias = set(), set()

    def add_blocked_cells_batch(self, arr):
        self.cells |= {tuple(r) for r in arr.tolist()}

    def add_blocked_vias_batch(self, arr):
        self.vias |= {tuple(r) for r in arr.tolist()}


def _oracle(poly, coord, band, eps=1e-6):
    """(gx, gy) cells whose centre is inside `poly` or within `band` mm of its
    outline, by plain point-in-polygon and edge distance; ties (within eps of
    the band) are left out, as the rasteriser resolves them open."""
    from obstacle_map import point_in_polygon, point_to_polygon_edge_distance
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    lo = coord.to_grid(min(xs) - band - 1, min(ys) - band - 1)
    hi = coord.to_grid(max(xs) + band + 1, max(ys) + band + 1)
    out, ties = set(), set()
    for gx in range(lo[0], hi[0] + 1):
        for gy in range(lo[1], hi[1] + 1):
            x, y = gx * coord.grid_step, gy * coord.grid_step
            d = point_to_polygon_edge_distance(x, y, poly)
            if point_in_polygon(x, y, poly) or d < band - eps:
                out.add((gx, gy))
            elif abs(d - band) <= eps:
                ties.add((gx, gy))
    return out, ties


def test_both_passes_block_their_band():
    from obstacle_map import add_rule_area_keepout_obstacles, add_user_keepout_obstacles
    cfg = GridRouteConfig(layers=["F.Cu"], track_width=0.3, clearance=0.25,
                          via_size=0.6, grid_step=0.1, keepout_enabled=True)
    coord = GridCoord(cfg.grid_step)
    extra = (0.3 + 0.25) / 2
    ko = {'polygon': BOX, 'holes': [], 'layers': {'F.Cu'},
          'tracks_allowed': False, 'vias_allowed': False}
    board = NS(board_info=NS(keepouts=[ko], board_bounds=None),
               keepout_zones=[NS(points=BOX)])
    for e in (0.0, extra):
        rec = _Rec()
        add_rule_area_keepout_obstacles(rec, board, cfg, extra_clearance=e)
        want, ties = _oracle(BOX, coord, cfg.clearance + cfg.track_width / 2 + e)
        got = {(x, y) for x, y, _l in rec.cells}
        assert got - ties == want - ties, (e, len(got ^ want))
        want_v, ties_v = _oracle(BOX, coord, cfg.clearance + cfg.via_size / 2 + e)
        assert rec.vias - ties_v == want_v - ties_v, (e, len(rec.vias ^ want_v))

        rec = _Rec()
        add_user_keepout_obstacles(rec, board, cfg, coord, 1, e)
        want, ties = _oracle(BOX, coord, e)
        got = {(x, y) for x, y, _l in rec.cells}
        assert got - ties == want - ties, ('user', e, len(got ^ want))
        assert rec.vias == got, "a user keep-out blocks vias on the same cells"


def _fixture(kind):
    """lvds_converter_dualclk with BOX as a native rule area ('native', tracks
    and vias not allowed) or a User.2 polygon ('user')."""
    from copy_board import copy_board
    from kicad_writer import generate_rule_area_sexpr
    out = os.path.join(tempfile.mkdtemp(prefix="dkmc_"), f"{kind}.kicad_pcb")
    _quiet(copy_board, LVDS, out)
    with open(out, encoding="utf-8", newline="") as f:
        text = f.read()
    if kind == "native":
        named = pcb_uses_name_nets(_quiet(parse_kicad_pcb, LVDS))
        add = generate_rule_area_sexpr(["F.Cu", "B.Cu"], BOX, "ko",
                                       not_allowed=("tracks", "vias"), use_net_name=named)
    else:
        pts = " ".join(f"(xy {x} {y})" for x, y in BOX)
        add = (f'  (gr_poly (pts {pts}) (stroke (width 0.1) (type solid)) (fill none) '
               f'(layer "User.2") (uuid 00009999-0000-0000-0000-000000000000))')
    close = text.rstrip().rfind(")")
    text = text[:close].rstrip("\n") + "\n" + add + "\n" + text[close:]
    with open(out, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    return out


def test_base_map_threads_the_extra_clearance():
    """A cell 0.55 mm off the keep-out's edge is outside a single track's band
    (0.25 + 0.15 = 0.40) and inside a member's (0.40 + 0.275 = 0.675): open on
    the single-ended map, blocked on the diff-pair map of the same board."""
    from obstacle_map import build_base_obstacle_map
    pcb = _quiet(parse_kicad_pcb, _fixture("native"))
    cfg = GridRouteConfig(layers=["F.Cu", "B.Cu"], track_width=0.3, clearance=0.25,
                          diff_pair_gap=0.25, grid_step=0.1)
    clk = [i for i, n in pcb.nets.items() if n.name in ("/CLK+", "/CLK-")]
    extra = (cfg.track_width + cfg.diff_pair_gap) / 2
    single = _quiet(build_base_obstacle_map, pcb, cfg, clk)
    pair = _quiet(build_base_obstacle_map, pcb, cfg, clk, extra)
    gx, gy = GridCoord(cfg.grid_step).to_grid(87.5, 58.0 - 0.55)
    assert not single.is_blocked(gx, gy, 0), "control: a single track may run here"
    assert pair.is_blocked(gx, gy, 0), "a member would reach into the keep-out from here"


def _route_diff(board, *extra):
    out = os.path.join(os.path.dirname(board), "routed.kicad_pcb")
    r = subprocess.run([sys.executable, "-X", "utf8", "py_router/route_diff.py", board, out,
                        "--nets", "/CLK+", "/CLK-"] + list(extra),
                       cwd=ROOT, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", env=ENV, timeout=900)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    summary = json.loads(r.stdout.split("JSON_SUMMARY: ", 1)[1].splitlines()[0])
    assert summary["routed_diff_pairs"] == ["/CLK"], summary["routed_diff_pairs"]
    return out


def _gaps(board, net):
    """(copper gap, centreline gap) in mm from `net`'s tracks to BOX; 0 for
    the centreline (and minus half the width for the copper) when a track
    end lies inside it. Plain segment geometry, no router code."""
    from geometry_utils import segment_to_segment_distance
    from obstacle_map import point_in_polygon
    pcb = _quiet(parse_kicad_pcb, board)
    nid = next(i for i, n in pcb.nets.items() if n.name == net)
    edges = list(zip(BOX, BOX[1:] + BOX[:1]))
    copper = centre = float("inf")
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        if point_in_polygon(s.start_x, s.start_y, BOX) or point_in_polygon(s.end_x, s.end_y, BOX):
            d = 0.0
        else:
            d = min(segment_to_segment_distance(s.start_x, s.start_y, s.end_x, s.end_y,
                                                a[0], a[1], b[0], b[1]) for a, b in edges)
        centre, copper = min(centre, d), min(copper, d - s.width / 2)
    return copper, centre


def _kicad_items_not_allowed(board):
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        return None
    out = board + ".drc.json"
    subprocess.run([cli, "pcb", "drc", "--format", "json", "--severity-all",
                    "--output", out, board], capture_output=True, timeout=600)
    with open(out, encoding="utf-8") as f:
        return sum(v["type"] == "items_not_allowed" for v in json.load(f)["violations"])


def test_members_stay_out_of_a_native_keepout():
    out = _route_diff(_fixture("native"))
    gaps = {n: _gaps(out, n)[0] for n in ("/CLK+", "/CLK-")}
    assert min(gaps.values()) > 0.2, f"a member's copper reached the keep-out: {gaps}"
    k = _kicad_items_not_allowed(out)
    assert k in (None, 0), f"KiCad reports {k} items_not_allowed"


def test_members_stay_out_of_a_user_keepout():
    out = _route_diff(_fixture("user"), "--keepout")
    gaps = {n: _gaps(out, n)[1] for n in ("/CLK+", "/CLK-")}
    assert min(gaps.values()) > 0, f"a member's centreline entered the keep-out: {gaps}"


TESTS = [
    test_both_passes_block_their_band,
    test_base_map_threads_the_extra_clearance,
    test_members_stay_out_of_a_native_keepout,
    test_members_stay_out_of_a_user_keepout,
]


def main():
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as e:
            failed += 1
            print(f"  FAIL: {t.__name__}: {e}")
            continue
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"  ERROR: {t.__name__}: {type(e).__name__}: {e}")
            continue
        print(f"  PASS: {t.__name__}")
    print(f"{len(TESTS) - failed}/{len(TESTS)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
