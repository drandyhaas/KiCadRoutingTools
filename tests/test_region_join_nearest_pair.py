#!/usr/bin/env python3
"""Region-join pair selection and seeding.

1. Vectorized closest-approach is bit-identical to the brute force (#351).
   `find_region_connection_points` measured closest approach between every region
   pair with a pure-Python O(N_i x N_j) double loop over (anchors + subsampled fill
   cells) -- the dominant cost of the region-join MST on big multi-region plane nets.
   It now uses one numpy argmin over the row-major dist^2 matrix per pair.
   The selection MUST be unchanged (the chosen points feed which copper the router
   bridges): np.argmin returns the FIRST global minimum in (i-outer, j-inner) order,
   exactly the brute force's strict-`<` first-minimum tie-break. Pinned against a
   verbatim copy of the old loop over randomized inputs (including ties, empty
   regions, and a daisho-class perf case).

2. The open-space fallback seed must be ON the region (neo6502 GND).
   `find_open_space_point` answers "where is the emptiest cell within 5mm of the
   region's centroid" -- a question about the OBSTACLE MAP, not about the
   region's copper -- and `_try_route_between_regions` fed that point to the A*
   as an extra source/target seed. Being the cheapest cell around, the search
   lands on it, so the strap's end is bare board: neo6502's GND repair produced
   15 such straps (including a 0.1mm one between two regions' open points), all
   correctly rejected by the #479 endpoint verification, which then burned each
   pair's whole try allowance so the real join was never retried. The seed must
   now satisfy the SAME material predicate the join is verified against.

3. A zero-length route means the pair is the same copper, not a failed join.
   Overlapping same-net zones on one layer are labelled per zone, so a patch pour
   inside a board-wide pour becomes a second "region" over identical fill; the A*
   answers with a one-point path. That is merged without copper, not reported as
   an UNVERIFIED failure.

Run:  python3 tests/test_region_join_nearest_pair.py [-v]
"""
import argparse
import contextlib
import io
import math
import os
import random
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

from routing_config import GridCoord, GridRouteConfig
import plane_region_connector as prc
from plane_region_connector import find_region_connection_points, UnionFind
from grid_router import GridObstacleMap


def _reference(region_anchors, region_cells, coord):
    """Verbatim pre-#351 brute force, as the correctness oracle."""
    n = len(region_anchors)
    if n < 2:
        return []
    region_pts = []
    for i in range(n):
        pts = list(region_anchors[i])
        pts.extend(prc._subsample_cell_points(
            region_cells[i] if i < len(region_cells) else (), coord))
        region_pts.append(pts)
    edges = []
    for i in range(n):
        for j in range(i + 1, n):
            bd, bpi, bpj = float('inf'), None, None
            for pi in region_pts[i]:
                for pj in region_pts[j]:
                    d = math.sqrt((pi[0] - pj[0]) ** 2 + (pi[1] - pj[1]) ** 2)
                    if d < bd:
                        bd, bpi, bpj = d, pi, pj
            if bpi and bpj:
                edges.append((bd, i, j, bpi, bpj))
    edges.sort(key=lambda e: e[0])
    uf, mst = UnionFind(), []
    for d, i, j, pi, pj in edges:
        if not uf.connected(i, j):
            uf.union(i, j)
            mst.append((i, j, pi, pj, d))
            if len(mst) == n - 1:
                break
    return mst


def _config():
    return GridRouteConfig(track_width=0.2, clearance=0.2, via_size=0.7,
                           via_drill=0.4, grid_step=0.1,
                           layers=['F.Cu', 'B.Cu'])


def check_open_space_seed_validity(verbose=False):
    """`find_open_space_point` must never hand back a point its caller's
    material predicate rejects, and must be unchanged when nothing is rejected."""
    coord = GridCoord(0.1)
    obs = GridObstacleMap(1)
    anchors = [(8.0, 8.0)]
    bounds = (0.0, 0.0, 20.0, 20.0)

    # Nothing is blocked, so EVERY cell in the search square ties at the
    # clearance cap. The ungated scan keeps the first tie in raw grid order --
    # the far bottom-left corner of the +-5mm box, ~7mm off the region. This is
    # the defect shape measured on neo6502 (open points 3-5mm off the region
    # they seed), pinned here because it is what the predicate must stop.
    unguarded = prc.find_open_space_point(anchors, obs, 0, coord, bounds=bounds)
    assert unguarded is not None, "no open point found at all"
    d_far = math.hypot(unguarded[0] - 8.0, unguarded[1] - 8.0)
    assert d_far > 3.0, \
        f"expected the unguarded scan to pick a far corner, got {unguarded}"

    # The gated scan must rank the tied cells NEAREST THE REGION, not by grid
    # order: the probe budget is finite, and grid order spends all of it on the
    # far edge of the box (2.5% of a +-5mm box at this 0.1mm grid), which
    # reports "no valid point" while thousands sit on the region. A predicate
    # that accepts everything therefore lands on the centroid, NOT on the
    # ungated far corner.
    permissive = prc.find_open_space_point(anchors, obs, 0, coord, bounds=bounds,
                                           validity=lambda x, y: True)
    assert permissive is not None and \
        math.hypot(permissive[0] - 8.0, permissive[1] - 8.0) <= 0.5, \
        f"gated scan is not ranked by distance to the region: {permissive}"

    # ...so a predicate that only credits the region's own 1mm neighbourhood
    # still FINDS a point. (Returning None here would mean the guard silently
    # disabled the fallback instead of correcting it.)
    guarded = prc.find_open_space_point(
        anchors, obs, 0, coord, bounds=bounds,
        validity=lambda x, y: math.hypot(x - 8.0, y - 8.0) <= 1.0)
    assert guarded is not None, \
        "the probe budget threw away every valid candidate near the region"
    assert math.hypot(guarded[0] - 8.0, guarded[1] - 8.0) <= 1.0, \
        f"returned a point its own validity rejects: {guarded}"

    # A predicate nothing in range satisfies yields None -- an absent seed
    # beats a bogus one; it must never fall back to the far corner.
    starved = prc.find_open_space_point(anchors, obs, 0, coord, bounds=bounds,
                                        validity=lambda x, y: False)
    assert starved is None, f"ignored its own validity and returned {starved}"
    if verbose:
        print(f"  unguarded={unguarded} permissive={permissive} guarded={guarded}")
    print("PASS: find_open_space_point honors the material predicate")


def check_open_space_seed_is_gated(verbose=False):
    """`_try_route_between_regions` must gate its open-space seed on the
    region's material, so an off-region point can no longer become a strap end."""
    FAR = (50.0, 50.0)
    seen_validity = []

    def fake_open(anchors, base_obstacles, plane_layer_idx, coord,
                  search_radius=5.0, bounds=None, validity=None):
        seen_validity.append(validity)
        if validity is not None and not validity(FAR[0], FAR[1]):
            return None
        return FAR

    def fake_route(source_points, target_points, **kw):
        # The corridor is walled: a path exists ONLY from the open-space point.
        if FAR in source_points and FAR in target_points:
            return ([(FAR[0], FAR[1], 'F.Cu'),
                     (FAR[0] + 0.1, FAR[1], 'F.Cu')], []), 10
        return None, kw.get('max_iterations', 1)

    anchors_i = [(1.0, 1.0)]
    anchors_j = [(30.0, 30.0)]
    kw = dict(base_obstacles=GridObstacleMap(2), plane_layer_idx=0,
              routing_layers=['F.Cu', 'B.Cu'], config=_config(),
              net_vias=[], max_track_width=0.4, min_track_width=0.2,
              max_iterations=1000, coord=GridCoord(0.1),
              bounds=(0.0, 0.0, 60.0, 60.0))

    real_open, real_route = prc.find_open_space_point, prc.route_plane_connection_wide
    prc.find_open_space_point = fake_open
    prc.route_plane_connection_wide = fake_route
    try:
        # Ungated (legacy behaviour): the bogus open-space strap IS produced --
        # both its ends sit 40mm+ from either region.
        legacy, _w, _v, _ex = prc._try_route_between_regions(
            anchors_i, anchors_j, **kw)
        assert legacy is not None, "fixture no longer reproduces the defect"
        assert legacy[0][0][:2] == FAR, \
            f"expected the strap to start on the open point, got {legacy[0][0]}"

        # Gated on material: neither region credits the open point, so no seed
        # is offered and no strap is invented.
        seen_validity.clear()
        off_region = lambda x, y: False           # noqa: E731
        gated, _w, _v, _ex = prc._try_route_between_regions(
            anchors_i, anchors_j, open_ok_i=off_region, open_ok_j=off_region,
            **kw)
        assert gated is None, \
            f"an off-material open seed still produced a strap: {gated}"
        assert seen_validity and all(v is not None for v in seen_validity), \
            "the material predicate never reached find_open_space_point"

        # A region that DOES credit the point keeps its rescue.
        on_region = lambda x, y: True             # noqa: E731
        kept, _w, _v, _ex = prc._try_route_between_regions(
            anchors_i, anchors_j, open_ok_i=on_region, open_ok_j=on_region,
            **kw)
        assert kept is not None, "a legitimate open-space rescue was lost"
    finally:
        prc.find_open_space_point = real_open
        prc.route_plane_connection_wide = real_route
    print("PASS: the open-space fallback seed is gated on region material")


def _stub_board():
    """Smallest PCBData the region-join loop will accept: two GND pads, no
    copper, no zones (so the fill models are inert and the regions come from
    the stubbed detector below)."""
    from kicad_parser import PCBData, BoardInfo, Net, Pad
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 20.0, 20.0))
    pads = []
    for i, (x, y) in enumerate(((1.0, 1.0), (15.0, 15.0))):
        p = Pad(pad_number=str(i + 1), net_id=1, net_name='GND',
                global_x=x, global_y=y, local_x=0.0, local_y=0.0,
                size_x=1.0, size_y=1.0, shape='circle',
                layers=['F.Cu'], drill=0.0, component_ref=f'R{i}')
        pads.append(p)
    return PCBData(board_info=bi,
                   nets={1: Net(net_id=1, name='GND', pads=pads)},
                   footprints={}, vias=[], segments=[],
                   pads_by_net={1: pads}, zones=[])


def _run_join_loop(meet_pt, cells_a, cells_b):
    """Drive route_disconnected_regions over two stubbed regions whose join
    attempt returns a ONE-POINT route landing at `meet_pt`. Returns
    (stdout, segments, vias, routes_added)."""
    def fake_regions(*a, **kw):
        # region 1 carries no anchors -> the #513 "already bridged" shortcut is
        # skipped and the join loop actually runs.
        return ([[(0.5, 0.5)], []], [cells_a, cells_b], [], [set(), set()])

    def fake_try(anchors_i, anchors_j, **kw):
        return (([meet_pt], []), 0.2, None, False)

    real_regions, real_try = (prc.find_disconnected_zone_regions,
                              prc._try_route_between_regions)
    prc.find_disconnected_zone_regions = fake_regions
    prc._try_route_between_regions = fake_try
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            segs, vias, added, _paths, _cp = prc.route_disconnected_regions(
                net_id=1, net_name='GND', plane_layer='F.Cu',
                zone_bounds=(0.0, 0.0, 20.0, 20.0), pcb_data=_stub_board(),
                config=_config(), base_obstacles=GridObstacleMap(2),
                layer_map={'F.Cu': 0, 'B.Cu': 1}, analysis_grid_step=0.5)
    finally:
        prc.find_disconnected_zone_regions = real_regions
        prc._try_route_between_regions = real_try
    return buf.getvalue(), segs, vias, added


def check_coincident_pair_merges_without_copper(verbose=False):
    """A one-point route means a source cell IS a target cell. When that cell
    is STRICT material for both components they are the same copper: merge
    them silently instead of reporting `UNVERIFIED ends=None/None` (which
    burned the pair's try allowance forever). When it is not, the merge is a
    false "plane joined" and must still be refused."""
    # Analysis grid is 0.5mm, so cell (gx,gy) covers mm (gx*0.5, gy*0.5).
    # The two regions' cell sets ABUT at gx=10 (mm 5.0): a shared cell there is
    # real evidence of shared copper.
    cells_a = {(gx, gy) for gx in range(6, 11) for gy in range(6, 11)}
    cells_b = {(gx, gy) for gx in range(10, 15) for gy in range(6, 11)}
    out, segs, vias, added = _run_join_loop((5.0, 4.0, 'F.Cu'), cells_a, cells_b)
    if verbose:
        print('  ' + out.replace('\n', '\n  ').strip())
    assert 'UNVERIFIED' not in out, \
        "a zero-length route on shared copper is still a failed join:\n" + out
    assert 'COINCIDENT' in out, \
        "the coincident pair was not recognised:\n" + out
    assert not segs and not vias, \
        f"a zero-length strap emitted copper: {len(segs)} seg(s), {len(vias)} via(s)"
    assert added == 0, f"a zero-length strap was counted as a join ({added})"

    # NEGATIVE: two regions 5mm apart whose seed lists merely quantized onto
    # one routing cell. A zero-length "join" there fuses genuinely disconnected
    # copper while emitting nothing -- the silent false merge #479 exists to
    # stop -- so it must be refused, not merged.
    far_a = {(gx, gy) for gx in range(0, 3) for gy in range(0, 3)}
    far_b = {(gx, gy) for gx in range(20, 23) for gy in range(20, 23)}
    out2, segs2, vias2, added2 = _run_join_loop((30.0, 30.0, 'F.Cu'),
                                                far_a, far_b)
    assert 'COINCIDENT' not in out2, \
        "fused two disconnected components on a shared seed cell:\n" + out2
    assert 'UNVERIFIED' in out2, \
        "an off-material zero-length route was not refused:\n" + out2
    assert not segs2 and not vias2 and added2 == 0, \
        "the refused pair still shipped copper"
    print("PASS: a coincident region pair merges without copper; a false one is refused")


def check_open_space_gate_reaches_every_rung(verbose=False):
    """Every `_try_route_between_regions` call in the join loop must carry the
    material predicates. The escalation rungs (retry-full, budget-x5,
    narrow-width) are separate call sites, and dropping the kwargs from one of
    them re-opens the bug on exactly the hard pairs that reach it."""
    import ast
    import inspect
    import textwrap
    src = textwrap.dedent(inspect.getsource(prc.route_disconnected_regions))
    tree = ast.parse(src)
    sites = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == '_try_route_between_regions']
    assert len(sites) >= 3, \
        f"expected the seed + budget-x5 + narrow-width rungs, found {len(sites)}"
    for n in sites:
        kws = {k.arg for k in n.keywords}
        assert {'open_ok_i', 'open_ok_j'} <= kws, \
            (f"_try_route_between_regions call at line {n.lineno} of "
             f"route_disconnected_regions does not gate its open-space seed "
             f"(kwargs: {sorted(kws)})")
    if verbose:
        print(f"  {len(sites)} gated call site(s)")
    print("PASS: every join rung gates its open-space seed")


def check_colocated_islands_are_one_region(verbose=False):
    """Two same-net zones on ONE layer whose fills overlap are ONE conductor:
    KiCad merges same-net fills, but a ZoneFillModel is built per ZONE and
    labels its fill in isolation, so a patch pour inside a board-wide pour used
    to become a second region over identical copper -- a phantom split the join
    loop then tried (and failed) to bridge."""
    from kicad_parser import PCBData, BoardInfo, Net, Zone
    from plane_fill_model import get_fill_models

    def rect(x0, y0, x1, y1):
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def board(zones):
        bi = BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
                       board_bounds=(0.0, 0.0, 20.0, 20.0))
        return PCBData(board_info=bi, nets={1: Net(1, 'GND')}, footprints={},
                       vias=[], segments=[], pads_by_net={}, zones=zones)

    def regions(pcb, anchors):
        models = get_fill_models(pcb, 1)
        return prc._regions_from_fill_models(
            1, pcb, GridCoord(0.5), 'F.Cu', {'F.Cu'}, models, anchors,
            (0.0, 0.0, 20.0, 20.0), 0.5)

    # The SMALL zone is listed first, so `_comp_key_at` (first model with fill
    # wins) credits an anchor inside it to the patch island and an anchor
    # outside it to the board-wide island: two keys, one piece of copper.
    over = board([Zone(net_id=1, net_name='GND', layer='F.Cu',
                       polygon=rect(5.0, 5.0, 9.0, 9.0), priority=1),
                  Zone(net_id=1, net_name='GND', layer='F.Cu',
                       polygon=rect(0.5, 0.5, 19.5, 19.5))])
    ra, _rc, _ri = regions(over, [(7.0, 7.0), (15.0, 15.0)])
    assert len(ra) == 1, \
        f"overlapping same-net pours still split into {len(ra)} regions"

    # NEGATIVE: two same-net pours that never share a point stay two regions.
    apart = board([Zone(net_id=1, net_name='GND', layer='F.Cu',
                        polygon=rect(0.5, 0.5, 6.0, 6.0)),
                   Zone(net_id=1, net_name='GND', layer='F.Cu',
                        polygon=rect(13.0, 13.0, 19.5, 19.5))])
    ra2, _rc2, _ri2 = regions(apart, [(3.0, 3.0), (16.0, 16.0)])
    assert len(ra2) == 2, \
        f"disjoint same-net pours were fused into {len(ra2)} region(s)"
    if verbose:
        print(f"  overlapping -> {len(ra)} region(s); disjoint -> {len(ra2)}")
    print("PASS: co-located fill islands are one region, disjoint ones are not")


def run(verbose=False):
    coord = GridCoord(0.05)
    random.seed(1234)
    for trial in range(200):
        n = random.randint(2, 6)
        ra, rc = [], []
        for _ in range(n):
            na = random.randint(0, 5)
            ra.append([(round(random.uniform(0, 50), 2), round(random.uniform(0, 50), 2))
                       for _ in range(na)])
            nc = random.randint(0, 60)
            rc.append(set((random.randint(0, 1000), random.randint(0, 1000))
                          for _ in range(nc)))
        assert find_region_connection_points(ra, rc, coord) == _reference(ra, rc, coord), \
            f"mismatch on random trial {trial}"

    # Deliberate ties: two regions with identical point sets.
    ra = [[(0.0, 0.0), (1.0, 1.0)], [(1.0, 1.0), (0.0, 0.0)]]
    assert find_region_connection_points(ra, [set(), set()], coord) == \
        _reference(ra, [set(), set()], coord), "tie case mismatch"

    # Empty regions contribute no edge (and must not crash).
    ra = [[], [(2.0, 3.0)], [(2.0, 4.0)]]
    assert find_region_connection_points(ra, [set(), set(), set()], coord) == \
        _reference(ra, [set(), set(), set()], coord), "empty-region mismatch"

    # < 2 regions -> no edges.
    assert find_region_connection_points([[(1.0, 1.0)]], [set()], coord) == []

    # The per-region interior-point memo (#351) must be transparent: cached
    # results identical to uncached, and _nearest_cell_points' sort must NOT
    # corrupt the cached list (it returns a fresh sorted copy).
    big = set((random.randint(0, 300), random.randint(0, 300)) for _ in range(8000))
    cache = {}
    base = prc._subsample_cell_points(big, coord, max_pts=400)
    assert prc._subsample_cell_points(big, coord, max_pts=400, interior_cache=cache) == base
    near = (5.0, 5.0)
    assert prc._nearest_cell_points(big, coord, near) == \
        prc._nearest_cell_points(big, coord, near, interior_cache=cache)
    assert prc._subsample_cell_points(big, coord, max_pts=400, interior_cache=cache) == base, \
        "cached interior list corrupted by _nearest_cell_points sort"

    # Perf/parity on a daisho-class input (many regions, dense anchors + cells).
    random.seed(7)
    n = 20
    ra = [[(round(random.uniform(0, 120), 3), round(random.uniform(0, 120), 3))
           for _ in range(120)] for _ in range(n)]
    rc = [set((random.randint(0, 2400), random.randint(0, 2400)) for _ in range(1500))
          for _ in range(n)]
    t = time.perf_counter()
    got = find_region_connection_points(ra, rc, coord)
    dt = time.perf_counter() - t
    assert got == _reference(ra, rc, coord), "daisho-class result differs from brute force"
    if verbose:
        print(f"  daisho-class n={n}: {dt * 1000:.1f} ms, {len(got)} MST edges")

    print("PASS: region-join nearest-pair is bit-identical to the brute force (#351)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    run(args.verbose)
    check_open_space_seed_validity(args.verbose)
    check_open_space_seed_is_gated(args.verbose)
    check_open_space_gate_reaches_every_rung(args.verbose)
    check_coincident_pair_merges_without_copper(args.verbose)
    check_colocated_islands_are_one_region(args.verbose)
