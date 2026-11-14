"""#595: a soft routing cost on signal-layer cells over a void in the
reference plane.

A track's return current runs in the plane copper next to it. Where the
adjacent plane layer has no fill under the track (a split between two pours,
a void, a slot) the return path detours. `reference_plane.py` MEASURES that on
a routed board; this module PRICES it while routing, so a controlled-impedance
net or a differential pair is steered off a void where it has a choice. It is
a cost, never a ban: where the only route crosses, the route is still laid.

OFF unless `KICAD_REFERENCE_VOID_COST` is set above 0 (mm-equivalent per void
cell, the unit of the plane-fragility and congestion fields). It moves routed
copper, so its default waits on a corpus A/B.

The geometry is the plane-fragility field's own raster of each pour's filled
copper at the routing grid (`plane_fragility._PourState.mask`, the exact
KiCad fill where the board file is reachable, the drawn outlines otherwise).
For a routing layer L the reference layers are the copper layers next to it
in the stack that carry a pour. A cell of L is a void cell when no reference
layer has fill there, over the whole board: a track that leaves the pours'
outline has left its reference as surely as one over a split. Holes
narrower than `KICAD_REFERENCE_VOID_MIN` mm (default 0.5: a via's or a pad's
clearance hole) are closed first, so a field of antipads does not price every
track that passes one.

The post-route smoothing pass does not read soft costs, and would straighten
a detour back across the gap; for a payer it refuses a connector that lies
over more void than the copper it replaces (`VoidMap.worse`).

The field is computed once when the step starts. It is delivered through the
track-proximity cache under one key, like the congestion field, and
`without_reference_void` leaves it out for a net that does not pay: the
payers are the nets with an impedance declaration (the board's
`net_impedance` record and this step's own widths), the pairs of a diff-pair
step, and the nets `KICAD_REFERENCE_VOID_NETS` names (comma-separated globs).
"""

import fnmatch
import os
from typing import Dict, Iterable, List, Optional, Set

import numpy as np

REFVOID_CACHE_KEY = ('refvoid',)

# id(the run's track_proximity_cache) -> (that cache, the nets that pay).
# The entry holds the cache, so its id cannot be reused while it is live.
_PAYERS: Dict[int, tuple] = {}
_PAYERS_MAX = 8


def void_cost_mm() -> float:
    try:
        return float(os.environ.get('KICAD_REFERENCE_VOID_COST', '0') or 0)
    except ValueError:
        return 0.0


def void_min_mm() -> float:
    try:
        return max(0.0, float(os.environ.get('KICAD_REFERENCE_VOID_MIN', '0.5') or 0.5))
    except ValueError:
        return 0.5


def _close(mask: np.ndarray, k: int) -> np.ndarray:
    """`mask` with every hole narrower than 2k cells filled (k 4-neighbour
    dilations, then k erosions; the border repeats its edge)."""
    if k <= 0:
        return mask
    m = np.pad(mask, k, mode='edge')
    for _ in range(k):
        g = m.copy()
        g[1:, :] |= m[:-1, :]
        g[:-1, :] |= m[1:, :]
        g[:, 1:] |= m[:, :-1]
        g[:, :-1] |= m[:, 1:]
        m = g
    for _ in range(k):
        g = m.copy()
        g[1:, :] &= m[:-1, :]
        g[:-1, :] &= m[1:, :]
        g[:, 1:] &= m[:, :-1]
        g[:, :-1] &= m[:, 1:]
        m = g
    return m[k:-k, k:-k]


class VoidMap:
    """The void cells per routing layer, for a pass that moves a payer's
    copper after routing: `worse` says whether a replacement path lies over
    more void than the path it replaces. The router's cost is a routing-time
    preference; without this the smoothing pass straightens a detour back
    across the gap it was routed round."""

    def __init__(self, masks: Dict[str, tuple], grid_step: float, payers):
        self.masks = masks              # layer -> (gx0, gy0, bool (H, W) void)
        self.step = grid_step
        self.payers = frozenset(payers)

    def void_mm(self, layer: str, pts) -> float:
        ent = self.masks.get(layer)
        if ent is None:
            return 0.0
        gx0, gy0, void = ent
        H, W = void.shape
        total = 0.0
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            length = float(np.hypot(x2 - x1, y2 - y1))
            n = max(1, int(np.ceil(length / (self.step / 2.0))))
            t = (np.arange(n) + 0.5) / n
            gi = np.rint((x1 + (x2 - x1) * t) / self.step).astype(np.int64) - gx0
            gj = np.rint((y1 + (y2 - y1) * t) / self.step).astype(np.int64) - gy0
            inside = (gi >= 0) & (gi < W) & (gj >= 0) & (gj < H)
            hit = np.zeros(n, dtype=bool)
            hit[inside] = void[gj[inside], gi[inside]]
            total += length * float(hit.sum()) / n
        return total

    def worse(self, net_id, layer: str, old_pts, new_pts) -> bool:
        if int(net_id or 0) not in self.payers:
            return False
        return self.void_mm(layer, new_pts) > self.void_mm(layer, old_pts) + self.step / 2.0


def void_masks(states, copper_layers: List[str], routing_layers: List[str],
               grid_step: float, min_mm: float, bounds=None) -> Dict[str, tuple]:
    """{routing layer: (gx0, gy0, bool (H, W) void mask)}: the cells of each
    routing layer with no fill on any adjacent poured layer, over the board
    (`bounds`: min_x, min_y, max_x, max_y in mm) and the pours' own windows.
    Holes narrower than `min_mm` are closed first."""
    by_layer: Dict[str, list] = {}
    for st in states or ():
        by_layer.setdefault(st.layer, []).append(st)
    out: Dict[str, tuple] = {}
    k = int(round(min_mm / 2.0 / grid_step)) if grid_step > 0 else 0
    for layer in routing_layers:
        if layer not in copper_layers:
            continue
        i = copper_layers.index(layer)
        sts = [s for j in (i - 1, i + 1) if 0 <= j < len(copper_layers)
               for s in by_layer.get(copper_layers[j], ())]
        if not sts:
            continue
        gx0, gy0 = min(s.gx0 for s in sts), min(s.gy0 for s in sts)
        gx1 = max(s.gx0 + s.W for s in sts)
        gy1 = max(s.gy0 + s.H for s in sts)
        if bounds is not None and grid_step > 0:
            gx0 = min(gx0, int(np.floor(bounds[0] / grid_step)))
            gy0 = min(gy0, int(np.floor(bounds[1] / grid_step)))
            gx1 = max(gx1, int(np.ceil(bounds[2] / grid_step)) + 1)
            gy1 = max(gy1, int(np.ceil(bounds[3] / grid_step)) + 1)
        cov = np.zeros((gy1 - gy0, gx1 - gx0), dtype=bool)
        for s in sts:
            cov[s.gy0 - gy0:s.gy0 - gy0 + s.H, s.gx0 - gx0:s.gx0 - gx0 + s.W] |= s.mask
        out[layer] = (gx0, gy0, ~_close(cov, k))
    return out


def void_rows(masks: Dict[str, tuple], routing_layers: List[str],
              cell_cost: int) -> np.ndarray:
    """(N, 4) [layer index, gx, gy, cost] rows of `void_masks`' cells, each
    at `cell_cost`."""
    out = []
    for li, layer in enumerate(routing_layers):
        ent = masks.get(layer)
        if ent is None:
            continue
        gx0, gy0, void = ent
        jj, ii = np.nonzero(void)
        if len(jj):
            out.append(np.column_stack([
                np.full(len(jj), li, dtype=np.int32), (ii + gx0).astype(np.int32),
                (jj + gy0).astype(np.int32),
                np.full(len(jj), int(cell_cost), dtype=np.int32)]))
    return np.vstack(out) if out else np.empty((0, 4), dtype=np.int32)


def payer_net_ids(pcb_data, config, extra: Iterable[int] = ()) -> Set[int]:
    """The nets the field is charged to (see the module docstring)."""
    out = {int(n) for n in extra or () if n}
    out |= {int(n) for n in (getattr(config, 'net_layer_widths', None) or {})}
    by_name = {n.name: nid for nid, n in (getattr(pcb_data, 'nets', None) or {}).items()
               if getattr(n, 'name', None)}
    try:
        from protected_nets import read_impedance_for_pcb_data
        for name in read_impedance_for_pcb_data(pcb_data) or {}:
            if name in by_name:
                out.add(int(by_name[name]))
    except Exception:                                            # noqa: BLE001
        pass
    pats = [p.strip() for p in os.environ.get('KICAD_REFERENCE_VOID_NETS', '').split(',')
            if p.strip()]
    for name, nid in by_name.items():
        if any(fnmatch.fnmatchcase(name, p) for p in pats):
            out.add(int(nid))
    return out


def register_reference_void(pcb_data, config, track_proximity_cache,
                            extra_payers: Iterable[int] = ()) -> None:
    """Compute the field and register it under `REFVOID_CACHE_KEY` (no-op
    when the cost is 0). Leaves `config._reference_void` set either way: the
    record a run's summary discloses."""
    cost = void_cost_mm()
    rec = {'cost_mm': cost, 'cells': 0, 'payers': 0, 'why': ''}
    config._reference_void = rec
    if cost <= 0:
        rec['why'] = 'KICAD_REFERENCE_VOID_COST is 0 (off)'
        return
    if getattr(config, 'quick', 0) >= 2:
        rec['why'] = '--quick 2'
        return
    field = getattr(config, '_fragility_field', None)
    states = getattr(field, 'states', None)
    if states is None:
        try:
            import contextlib
            import io
            from plane_fragility import _compute_cells_and_states
            with contextlib.redirect_stdout(io.StringIO()):
                states = _compute_cells_and_states(pcb_data, config, want_states=True)[1]
        except Exception as e:                                   # noqa: BLE001
            rec['why'] = f'the pours could not be rasterized ({e})'
            print(f"Reference-void field: not built ({rec['why']})")
            return
    if not states:
        rec['why'] = ('no pour raster (the board has no zones, or the plane-fragility '
                      'field that rasterizes them is off)')
        return
    masks = void_masks(states, list(pcb_data.board_info.copper_layers),
                       list(config.layers), config.grid_step, void_min_mm(),
                       bounds=getattr(pcb_data.board_info, 'board_bounds', None))
    rows = void_rows(masks, list(config.layers), config.cell_cost(cost))
    payers = payer_net_ids(pcb_data, config, extra_payers)
    config._reference_void_map = VoidMap(masks, config.grid_step, payers)
    rec['cells'], rec['payers'] = int(len(rows)), len(payers)
    rec['min_mm'] = void_min_mm()
    if not len(rows):
        rec['why'] = 'no void cell under a routing layer'
        return
    track_proximity_cache[REFVOID_CACHE_KEY] = rows
    _PAYERS[id(track_proximity_cache)] = (track_proximity_cache, frozenset(payers))
    while len(_PAYERS) > _PAYERS_MAX:
        _PAYERS.pop(next(iter(_PAYERS)))
    print(f"Reference-void field: {len(rows)} cell(s) over a gap in the adjacent "
          f"plane (KICAD_REFERENCE_VOID_COST={cost:g}mm-equiv, holes under "
          f"{void_min_mm():g} mm closed); {len(payers)} net(s) pay it"
          + ("" if payers else " -- none: no net has an impedance declaration, is a "
             "pair of this step, or matches KICAD_REFERENCE_VOID_NETS"))


def without_reference_void(cache, run_cache, net_ids):
    """`cache` (the run's `run_cache`, or a view of it) without the field
    when none of `net_ids` pays it -- the same dict when the field is absent
    or a payer is being routed."""
    if REFVOID_CACHE_KEY not in cache:
        return cache
    ent = _PAYERS.get(id(run_cache))
    payers = ent[1] if ent is not None and ent[0] is run_cache else frozenset()
    if any(int(n) in payers for n in net_ids or ()):
        return cache
    return {k: v for k, v in cache.items() if k != REFVOID_CACHE_KEY}
