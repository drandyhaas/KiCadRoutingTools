"""Foreign-pour guard (#1260): a pour this run cannot repair is not cut.

Routing a track through another net's copper pour is normal in KiCad: the
refill clears the pour around the track. What is NOT normal is a track that
cuts a pour into pieces. KiCad then deletes every piece that no longer
touches a pad of the pour's net (island removal), and if the pour was the
only copper joining two of that net's pads, the net is broken after refill.
The routed board reads clean before that refill, because the fill written
in the file is the old one.

The plane finalize (#562) repairs such cuts, but only for the zone nets in
the run's own scope. A zone net outside it is excluded BY PLAN, so a cut in
its pour ships. The #424 plane-fragility cost does not stop the cut either:
it prices a pour's NARROW parts, which is the right proxy for a plane that
signals must cross, but a wide pour spanning the whole route is cut by any
crossing, and the cost is zero in the middle of it. #1260's board is that
case: a 12 mm pour of ZONE_NET across the full height between SIG's two
pads, and `route.py --nets SIG` drew SIG straight through it.

The guard is topological rather than a cost. A routed net that enters a
fill island must leave it again unless one of its terminals (a pad, via or
stub end) lies inside the island or one of its holes. A path that enters an
island and leaves it again joins two points of the island's outer boundary,
so it separates the island into two pieces. An island no routed net has a
terminal inside can therefore only be CUT by this run, never used.

Such an island is a BARRIER when the cut would cost something: it touches
two or more of its own net's pads, vias or tracks (the cut can separate
them), or one, and is at least the minimum island area (the cut loses the
far side; KiCad's default floor, 10 mm^2, or the zone's own in area mode).
`build_base_obstacle_map` stamps a barrier as a hard track and via keep-out,
at the larger of the pair clearance and the zone's own clearance, like a pad
or an existing track of another net. Routing goes around it, or the net
fails.

Everything else stays crossable, which keeps ordinary routing over a pour
working. A full-board GND pour holds every pad on the board, and a track
leaving a pad inside it opens a slot from a hole to the outer edge, which
divides nothing. A small island on one pad loses only a sliver of copper to
a cut. Another net CAN still cut an island that holds a routed terminal, so
the outermost run grades those zone nets again on the copper it ships
(`audit_pour_cuts`) and names any it left with more disconnected pads than
the input had. The audit reports; it does not refuse.

Scope: the zone nets outside the run, which are the ones the finalize
excludes (`matches_net_filter` against the caller's net filter, the finalize's
own test) and which the run does not route or rip. Nested sub-runs (the
reconciliation laps, the finalize's repair sub-runs) inherit the top-level
run's barriers through the `pour_barriers` kwarg instead of recomputing them
from their own, narrower scope.

Geometry: the exact KiCad fill when it can be had (the GUI's live-board
provider, else a refill of the source file, both memoised and shared with
the plane-fragility field), else the zone outlines, which over-cover.

`KICAD_POUR_GUARD=0` turns the guard off, for A/B.
"""
from __future__ import annotations

import math
import os
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from kicad_parser import pour_zones


def _zone_net_name(pcb_data, zone) -> Optional[str]:
    net = (pcb_data.nets or {}).get(zone.net_id)
    name = getattr(net, 'name', None) or getattr(zone, 'net_name', None)
    return name or None


def out_of_scope_zone_nets(pcb_data, net_names, routed_ids,
                           layers: Iterable[str]) -> Dict[int, str]:
    """{net_id: name} of the zone nets on `layers` that this run neither
    routes nor repairs.

    Out of scope = not matched by the caller's net filter (the test the plane
    finalize uses to exclude a zone net BY PLAN) and not among `routed_ids`
    (the nets the run routes or may rip). No filter means every net is in
    scope, as it is for the finalize."""
    if not net_names:
        return {}
    from net_queries import matches_net_filter
    layers = set(layers or ())
    routed = set(routed_ids or ())
    out: Dict[int, str] = {}
    for z in pour_zones(getattr(pcb_data, 'zones', None) or []):
        if not z.net_id or z.net_id in routed or z.layer not in layers:
            continue
        name = _zone_net_name(pcb_data, z)
        if not name or matches_net_filter(name, net_names):
            continue
        out[z.net_id] = name
    return out


def fill_islands(pcb_data, wanted: Dict[Tuple[str, str], list]):
    """({(net name, layer): [polygon, ...]}, source, why) for the wanted
    (net, layer) pairs.

    The exact fill first: the live-board provider (GUI), then a refill of
    `pcb_data.source_path` (CLI). Both are memoised on the board state, so
    the plane-fragility field that asks next pays nothing. When neither is
    available, the zone outlines in `wanted`'s values are returned instead:
    an outline covers at least the fill, so a barrier drawn from it is never
    smaller than the copper it protects."""
    why = ''
    provider = getattr(pcb_data, 'exact_fill_provider', None)
    if provider is not None:
        try:
            fills = provider()
            if fills:
                return ({k: fills.get(k, []) for k in wanted},
                        'live_board', '')
            why = 'the live-board fill poured nothing'
        except Exception as e:                       # noqa: BLE001
            why = f'live-board fill failed ({type(e).__name__}: {e})'
    src = getattr(pcb_data, 'source_path', None)
    if src and os.path.isfile(src):
        try:
            from kicad_exact_fill import refill_islands_ex
            fills, st = refill_islands_ex(src)
            if st.ok and fills is not None:
                return ({k: fills.get(k, []) for k in wanted},
                        'kicad_refill', '')
            why = st.why()
        except Exception as e:                       # noqa: BLE001
            why = f'{type(e).__name__}: {e}'
    elif not why:
        why = 'no board file to refill'
    return {k: list(v) for k, v in wanted.items()}, 'zone_outlines', why


def _terminal_points(pcb_data, routed_ids) -> np.ndarray:
    """(N, 2) points where a routed net's copper starts: its pads, vias and
    existing track ends. A route may begin at any of them."""
    routed = set(routed_ids or ())
    pts: List[Tuple[float, float]] = []
    for nid in routed:
        for p in (pcb_data.pads_by_net or {}).get(nid, []) or []:
            pts.append((p.global_x, p.global_y))
    for v in pcb_data.vias:
        if v.net_id in routed:
            pts.append((v.x, v.y))
    for s in pcb_data.segments:
        if s.net_id in routed:
            pts.append((s.start_x, s.start_y))
            pts.append((s.end_x, s.end_y))
    if not pts:
        return np.empty((0, 2), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


def _anchor_points(pcb_data, net_id, layer):
    """((N, 2) points, (N,) item index) of the copper a pour of `net_id` on
    `layer` hangs from: the net's pads on that layer (or through it), its
    vias, and its tracks on that layer (both ends of one track are ONE item).
    KiCad keeps a fill island only while it touches one of them."""
    from net_queries import expand_pad_layers
    pts: List[Tuple[float, float]] = []
    item: List[int] = []
    k = 0
    for p in (pcb_data.pads_by_net or {}).get(net_id, []) or []:
        if p.pad_type == 'np_thru_hole':
            continue
        if (p.drill or 0) > 0 or layer in expand_pad_layers(p.layers or [],
                                                            [layer]):
            pts.append((p.global_x, p.global_y))
            item.append(k)
            k += 1
    for v in pcb_data.vias:
        if v.net_id == net_id:
            pts.append((v.x, v.y))
            item.append(k)
            k += 1
    for s in pcb_data.segments:
        if s.net_id == net_id and s.layer == layer:
            pts += [(s.start_x, s.start_y), (s.end_x, s.end_y)]
            item += [k, k]
            k += 1
    if not pts:
        return np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=np.int64)
    return (np.asarray(pts, dtype=np.float64),
            np.asarray(item, dtype=np.int64))


def _label(mask: np.ndarray):
    """4-connected component labels of a boolean grid (scipy when present,
    the plane fill model's numpy labeller otherwise -- KiCad's own python
    has no scipy)."""
    try:
        from scipy import ndimage
        return ndimage.label(mask)
    except ImportError:
        from plane_fill_model import _label_components
        return _label_components(mask)


#: Hull rasters above this many cells are taken at a coarser step.
_HULL_CELL_BUDGET = 4_000_000


class IslandHull:
    """One fill island and its holes, for "is this point inside it?".

    The island is rasterised at `step` only when a query point falls inside
    its bounding box; every non-fill cell connected to the raster's border is
    outside, every other non-fill cell is a hole. A point in a hole is one a
    route can start from without entering the island from outside, and the
    hole a thermal relief or a clearance opens is where the pour's own pads
    sit."""

    def __init__(self, poly, step: float):
        self.poly = poly
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        self.box = (min(xs), min(ys), max(xs), max(ys))
        self.step = step
        self._raster = None

    def _build(self):
        x0, y0, x1, y1 = self.box
        step = self.step
        cells = ((x1 - x0) / step + 5) * ((y1 - y0) / step + 5)
        if cells > _HULL_CELL_BUDGET:
            step = step * float(np.sqrt(cells / _HULL_CELL_BUDGET))
        from obstacle_map import _rasterize_polygon_box
        from routing_config import GridCoord
        coord = GridCoord(step)
        gx_lo, gy_lo, nx, ny, inside, _ = _rasterize_polygon_box(
            self.poly, coord, 2 * step)
        if inside is None:
            self._raster = (coord, 0, 0, 0, 0, None)
            return
        fill = inside.reshape(ny, nx)
        labels, _n = _label(~fill)
        border = np.unique(np.concatenate([labels[0, :], labels[-1, :],
                                           labels[:, 0], labels[:, -1]]))
        hull = fill | ((labels > 0) & ~np.isin(labels, border))
        self._raster = (coord, gx_lo, gy_lo, nx, ny, hull)

    def contains(self, pts: np.ndarray) -> np.ndarray:
        """Bool mask over `pts` (N, 2): inside the island or one of its holes."""
        out = np.zeros(len(pts), dtype=bool)
        if not len(pts):
            return out
        x0, y0, x1, y1 = self.box
        sel = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1)
               & (pts[:, 1] >= y0) & (pts[:, 1] <= y1))
        if not sel.any():
            return out
        if self._raster is None:
            self._build()
        coord, gx_lo, gy_lo, nx, ny, hull = self._raster
        if hull is None:
            return out
        k = np.nonzero(sel)[0]
        # GridCoord.to_grid, vectorised (np.rint rounds half to even, as
        # Python's round does).
        i = np.rint(pts[k, 0] * coord.inv_step).astype(np.int64) - gx_lo
        j = np.rint(pts[k, 1] * coord.inv_step).astype(np.int64) - gy_lo
        ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        out[k[ok]] = hull[j[ok], i[ok]]
        return out


def _poly_area(poly) -> float:
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


#: KiCad's default minimum island area (ZONE_SETTINGS, probed on 10.0:
#: 10 mm^2): the size below which KiCad's own "remove islands below area"
#: mode does not think a piece of pour worth keeping.
KICAD_MIN_ISLAND_AREA_MM2 = 10.0


def _min_island_area(zones) -> float:
    """The area a single-anchor island must reach to be guarded: the zone's
    own `island_area_min` when it removes islands by area (mode 2), else
    KiCad's default."""
    floors = [float(z.island_area_min) for z in zones
              if getattr(z, 'island_removal_mode', 0) == 2
              and (getattr(z, 'island_area_min', 0) or 0) > 0]
    return min(floors) if floors else KICAD_MIN_ISLAND_AREA_MM2


def compute_pour_barriers(pcb_data, layers, routed_ids, net_names,
                          grid_step: float):
    """(barriers, report) for this run.

    An island of an out-of-scope pour is a BARRIER when no routed net has a
    terminal inside it (so this run could only cut it) and a cut would cost
    something: it touches two or more of its net's pads, vias or tracks (a
    cut can separate them), or one and is at least the minimum island area
    (a cut loses the far side). An island below that area with one anchor
    stays crossable -- a cut there shaves copper off one pad -- and so does
    one with no anchor, which KiCad does not keep.

    `barriers` is what `GridRouteConfig.pour_barriers` holds, one entry per
    (net, layer) with a barrier island: {'net', 'layer', 'polygons',
    'clearance'}. `report` is the JSON-ready account: the geometry source
    and, per (net, layer), the islands blocked and left crossable."""
    oos = out_of_scope_zone_nets(pcb_data, net_names, routed_ids, layers)
    if not oos:
        return [], None
    layers = set(layers)
    wanted: Dict[Tuple[str, str], list] = {}
    zones_of: Dict[Tuple[str, str], list] = {}
    net_of: Dict[Tuple[str, str], int] = {}
    clearance: Dict[Tuple[str, str], float] = {}
    for z in pour_zones(pcb_data.zones):
        if z.net_id not in oos or z.layer not in layers:
            continue
        key = (oos[z.net_id], z.layer)
        net_of[key] = z.net_id
        zones_of.setdefault(key, []).append(z)
        if len(z.polygon or ()) >= 3:
            wanted.setdefault(key, []).append(list(z.polygon))
        if z.clearance is not None:
            clearance[key] = max(clearance.get(key, 0.0), float(z.clearance))
    if not wanted:
        return [], None
    islands, source, why = fill_islands(pcb_data, wanted)
    terms = _terminal_points(pcb_data, routed_ids)
    barriers: List[dict] = []
    rows: List[dict] = []
    for key in sorted(wanted):
        net, layer = key
        a_pts, a_item = _anchor_points(pcb_data, net_of[key], layer)
        floor = _min_island_area(zones_of[key])
        blocked: List[list] = []
        crossable = minor = 0
        for poly in islands.get(key) or []:
            if len(poly) < 3:
                continue
            hull = IslandHull(poly, grid_step)
            if hull.contains(terms).any():
                crossable += 1
                continue
            anchors = len(set(a_item[hull.contains(a_pts)].tolist()))
            if anchors >= 2 or (anchors == 1 and _poly_area(poly) >= floor):
                blocked.append([(float(x), float(y)) for x, y in poly])
            else:
                minor += 1
        if blocked:
            barriers.append({'net': net, 'layer': layer, 'polygons': blocked,
                             'clearance': clearance.get(key)})
        if blocked or crossable or minor:
            rows.append({'net': net, 'layer': layer,
                         'blocked_islands': len(blocked),
                         'blocked_mm2': round(sum(_poly_area(p)
                                                  for p in blocked), 1),
                         'crossable_islands': crossable,
                         'minor_islands': minor})
    report = {'source': source, 'why': why, 'pours': rows}
    return barriers, report


def install_pour_barriers(config, pcb_data, routed_ids, net_names,
                          given=None):
    """Set `config.pour_barriers` for this run and return the report (None
    when there is nothing to say).

    `given` is a parent run's list (a nested sub-run inherits it rather than
    recomputing from its own narrower scope); an empty list is a real answer,
    "the parent blocked nothing"."""
    import env_knobs
    if given is not None:
        config.pour_barriers = list(given)
        return None
    if not env_knobs.POUR_GUARD:
        config.pour_barriers = []
        return None
    barriers, report = compute_pour_barriers(
        pcb_data, config.layers, routed_ids, net_names, config.grid_step)
    config.pour_barriers = barriers
    if report is None:
        return None
    for r in report['pours']:
        if r['blocked_islands']:
            print(f"Pour guard (#1260): {r['net']} pour on {r['layer']} is "
                  f"outside this run's nets, and no routed net has a pad "
                  f"inside {r['blocked_islands']} of its island(s) "
                  f"({r['blocked_mm2']} mm^2) -- blocked, so no track cuts "
                  f"them (KICAD_POUR_GUARD=0 reverts)")
        left = []
        if r['crossable_islands']:
            left.append(f"{r['crossable_islands']} hold a routed net's "
                        f"terminal")
        if r['minor_islands']:
            left.append(f"{r['minor_islands']} are below the minimum island "
                        f"area on one anchor")
        if left:
            print(f"Pour guard (#1260): {r['net']} pour on {r['layer']}: "
                  f"islands left crossable -- {'; '.join(left)}")
    if report['source'] == 'zone_outlines' and barriers:
        print(f"Pour guard (#1260): no exact fill ({report['why']}); "
              f"blocking the zone OUTLINES, which cover more than the fill")
    return report


def _barrier_keepouts(config, pcb_data):
    """[(net, layer, polygons, bbox, clearance)] of the `config.pour_barriers`
    islands, at the clearance `add_pour_barrier_obstacles` stamps."""
    name_to_id = {n.name: nid for nid, n in (pcb_data.nets or {}).items()}
    rows = []
    for b in getattr(config, 'pour_barriers', None) or ():
        polys = [p for p in b.get('polygons') or () if p]
        if not polys:
            continue
        nid = name_to_id.get(b.get('net'))
        pair = (config.obstacle_clearance(nid) if nid is not None
                else config.clearance)
        bbox = (min(x for p in polys for x, _ in p), min(y for p in polys for _, y in p),
                max(x for p in polys for x, _ in p), max(y for p in polys for _, y in p))
        rows.append((b.get('net'), b.get('layer'), polys, bbox,
                     max(pair, b.get('clearance') or 0.0)))
    return rows


def _via_enters(row, x, y, size) -> bool:
    from check_drc import _point_to_polys_distance
    _net, _layer, polys, (x0, y0, x1, y1), clear = row
    reach = clear + size / 2
    if not (x0 - reach <= x <= x1 + reach and y0 - reach <= y <= y1 + reach):
        return False
    return _point_to_polys_distance(x, y, polys) < reach - 1e-6


def barrier_via_site_filter(config, pcb_data, via_size: float):
    """An (x, y) -> bool predicate: may a through via sit at (x, y) outside
    every pour barrier's keep-out? None when the run has no barrier. For the
    pad taps' via search, which `add_pour_barrier_obstacles` never reaches."""
    rows = _barrier_keepouts(config, pcb_data)
    if not rows:
        return None
    return lambda x, y: not any(_via_enters(r, x, y, via_size) for r in rows)


#: sample pitch (mm) along a trace when its part on its own pad is set aside
OWN_PAD_SAMPLE_MM = 0.01


def _pad_on_layer(pad, layer) -> bool:
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return False
    layers = getattr(pad, 'layers', None) or ()
    return layer in layers or '*.Cu' in layers


def _disk_on_pad(x, y, h, pad) -> bool:
    """True when the disk of radius `h` at (x, y) lies wholly on `pad`'s
    copper: that much of a trace adds nothing a pour has to clear, since the
    pour already keeps its clearance from the pad there."""
    from check_drc import (pad_corner_radius, point_to_rect_distance,
                           _into_pad_frame, _point_in_poly,
                           point_to_segment_distance)
    polys = getattr(pad, 'polygons', None)
    if polys:
        for poly in polys:
            n = len(poly)
            if n >= 3 and _point_in_poly(x, y, poly) and min(
                    point_to_segment_distance(x, y, *poly[i], *poly[(i + 1) % n])
                    for i in range(n)) >= h - 1e-9:
                return True
        return False
    # the pad eroded by h: a rounded rect h smaller each side, its corner
    # radius h smaller too
    hx, hy = pad.size_x / 2 - h, pad.size_y / 2 - h
    if hx < 0 or hy < 0:
        return False
    r = max(min(pad_corner_radius(pad), pad.size_x / 2, pad.size_y / 2) - h, 0.0)
    rot = getattr(pad, 'rect_rotation', 0.0) or 0.0
    if rot:
        rad = math.radians(rot)
        x, y = _into_pad_frame(x, y, pad, math.cos(rad), math.sin(rad))
    return point_to_rect_distance(x, y, pad.global_x, pad.global_y, hx, hy, r) <= 1e-9


def _off_pad_clear(s, polys, need, pads) -> bool:
    """True when every part of trace `s` that is not on one of `pads` keeps
    `need` (clearance + half width) from `polys`, sampled along the trace."""
    from check_drc import _point_to_polys_distance
    h = s.width / 2
    n = max(1, int(math.ceil(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                             / OWN_PAD_SAMPLE_MM)))
    for i in range(n + 1):
        t = i / n
        x = s.start_x + t * (s.end_x - s.start_x)
        y = s.start_y + t * (s.end_y - s.start_y)
        if any(_disk_on_pad(x, y, h, p) for p in pads):
            continue
        if _point_to_polys_distance(x, y, polys) < need - 1e-6:
            return False
    return True


def barrier_entered(config, pcb_data, segments, vias, own_pads=()) -> Optional[str]:
    """The net of the first pour barrier whose keep-out one of `segments` or
    `vias` (through vias) enters, else None.

    `own_pads`: the pad(s) the copper taps. The part of a trace lying on its
    own pad's copper is not measured -- the pour already keeps its clearance
    from the pad there, so that copper cannot cut it further. Only the trace
    copper off the pad is held to the keep-out."""
    from check_drc import _segment_to_polys_distance
    for row in _barrier_keepouts(config, pcb_data):
        net, layer, polys, _bbox, clear = row
        if any(_via_enters(row, v.x, v.y, v.size) for v in vias):
            return net
        pads = [p for p in own_pads or () if _pad_on_layer(p, layer)]
        for s in segments:
            if s.layer != layer:
                continue
            need = clear + s.width / 2
            if _segment_to_polys_distance(s.start_x, s.start_y, s.end_x, s.end_y,
                                          polys)[0] >= need - 1e-6:
                continue
            if pads and _off_pad_clear(s, polys, need, pads):
                continue
            return net
    return None


# ------------------------------------------------------------------ the audit
def snapshot_copper(pcb_data):
    """Value copies of the board's tracks and vias, taken before the run
    moves any of them in place, for `audit_pour_cuts`' "before"."""
    import copy
    return ([copy.copy(s) for s in pcb_data.segments],
            [copy.copy(v) for v in pcb_data.vias])


def _open_pads(board, net_id) -> int:
    """Disconnected pads of `net_id` on `board` (a CopperView), by the
    fill-aware grade check_connected uses: every zone of the net is modelled
    against the view's own copper, in the view's own cache."""
    from check_connected import (check_net_connectivity,
                                 net_break_within_outlines)
    zones = [z for z in (board.zones or []) if z.net_id == net_id and z.polygon]
    r = check_net_connectivity(
        net_id, [s for s in board.segments if s.net_id == net_id],
        [v for v in board.vias if v.net_id == net_id],
        (board.pads_by_net or {}).get(net_id, []), zones, pcb_data=board)
    broken, dis = net_break_within_outlines(board, r)
    return len(dis or []) if broken else 0


def audit_nets(report) -> List[str]:
    """The out-of-scope zone nets a run's copper could still cut apart: those
    with an island the guard left crossable because a routed net has a
    terminal inside it. A barrier island is not entered, and a minor one has
    at most one anchor, so neither can come apart."""
    if not report:
        return []
    return sorted({r['net'] for r in report.get('pours') or []
                   if r.get('crossable_islands')})


def audit_pour_cuts(before_board, before_copper, after_board, after_copper,
                    net_names) -> List[dict]:
    """[{'net', 'open_pads_before', 'open_pads_after'}] for each of
    `net_names` (two or more pads) that the run left with MORE disconnected
    pads than the input had: its pour was cut where the guard let tracks
    cross. Both states are graded the same way, so the raster model's own
    error cancels."""
    from plane_fill_model import CopperView
    before = CopperView(before_board, *before_copper)
    after = CopperView(after_board, *after_copper)
    b_ids = {n.name: i for i, n in before.nets.items()}
    a_ids = {n.name: i for i, n in after.nets.items()}
    rows = []
    for name in net_names:
        bi, ai = b_ids.get(name), a_ids.get(name)
        if bi is None or ai is None:
            continue
        if len((before.pads_by_net or {}).get(bi, []) or []) < 2:
            continue
        ob, oa = _open_pads(before, bi), _open_pads(after, ai)
        if oa > ob:
            rows.append({'net': name, 'open_pads_before': ob,
                         'open_pads_after': oa})
    return rows


def format_pour_cuts(rows) -> str:
    """One console line naming the cut pours, or '' when there are none."""
    if not rows:
        return ''
    what = ', '.join(f"{r['net']} ({r['open_pads_before']} -> "
                     f"{r['open_pads_after']} open pads)" for r in rows)
    return (f"Pour cut (#1260): {what} -- this run's copper cut the pour of "
            f"a zone net outside its nets, where a routed net's pad inside "
            f"the pour kept it crossable; nothing in this run repairs it, so "
            f"route that net in a later step or reroute the crossing")
