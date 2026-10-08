"""History-based negotiated congestion (#590) -- PathFinder-style.

Every soft cost this router already has measures the board's state NOW:
track/stub/BGA proximity price copper that exists, congestion v1/v2 price
density and demand, the ripped-route ghosts price a corridor that was just
vacated -- and vanish again the moment their owner reroutes (the C1 filter in
``routing_context.filter_ripped_ghosts``).

Nothing prices conflict over TIME. That is what PathFinder's history cost is:
a small PERMANENT bump on every cell that participates in a conflict, so
repeatedly-contested ground gets progressively more expensive and later
attempts route around it without anyone arbitrating whom to rip. The 0802
rip-arbitration study concluded exactly this by elimination -- every
frame-local rip gate was refuted, "soft rra pricing beats refusal" -- and
history is the principled version of that finding: per-CELL and cumulative
where the rip ghosts are per-NET and transient.

Conflict events (all in mm-equivalent, composed like every other soft
source). PathFinder charges only the OVERUSED nodes; the engine's analog is
the contest, and the shipped default (below) charges all three events:

  * a CONTEST (full increment) -- the intersection of a FAILED
    search's blocked frontier with a routed net's copper: ground one net
    holds and another just stalled against. ``analyze_frontier_blocking``
    already computes exactly these cells per blocker (its rip-candidate
    ranking); they are charged there, BEFORE any rip, so a ripped blocker's
    reroute already sees its contested ground priced and relocates instead
    of re-taking it.
  * a RIP's whole footprint (``KICAD_HISTORY_RIP_WEIGHT``, shipped 1.0).
  * a FAILED search's raw blocked frontier
    (``KICAD_HISTORY_BLOCKED_WEIGHT``, shipped 0.25).

History of that choice (see the #590 issue thread): v2 made the contest the
primary event and demoted the other two to weight 0, on a local diagnosis
(a few boards) that the rip stamp priced mostly uncontested, vacated
corridor and the frontier charge mostly static copper. The corpus A/B
reversed that for the rip stamp: the contest alone, and contests plus
frontier without rip stamps, both measured WORSE than baseline, while all
three together was the best arm.

Repeat contests can ESCALATE (``KICAD_HISTORY_ESCALATE``): with a value
above 0, the second charge of a cell adds max(inc, escalate x accumulated),
so at 1.0 the cell's price DOUBLES per repeat (0.1, 0.2, 0.4, ...). That was
v2's default, on the reasoning that a routing call has only a handful of rip
rounds to build a gradient in. On the corpus it walled off fine-pitch BGA
escape fields, where approaches are mandatory and contests repeat, so the
shipped value is 0: a repeat adds the flat increment again.

Scope is ONE routing call: the field is created/reset at batch start and
attached to the config, like congestion v2's bins. No decay (v1) -- decay is
an FPGA-ism for hundred-iteration convergence; a PCB run has a handful of rip
rounds.

PROMOTED TO A SHIPPED DEFAULT (2026-08-13) at the "v1flat_01" settings: cost
0.1, cap 0.5, rip_weight 1.0, blocked_weight 0.25, escalate 0 -- the flat
diffuse field. It was the best arm of every one tested, on the two corpora
that compared arms (sets 1-10 and 11-20; sets 21-27 ran it against off
only), and on sets 1-10 it recovers ~40 of the ~41-net gap that opened
between v0.20.2 and HEAD while keeping HEAD's DRC advantage (66 vs the
release's 144).

Read the caveat with the result. The win is CONCENTRATED ON CONGESTED BOARDS
and vanishes where there are none: sets 1-10 -47 nets (20W/11L, p=0.075), sets
11-20 +1 (flat, a repeated null), sets 21-27 -15 but with two boards supplying
113% of it (7W/6L, p=0.50). A pre-registered sets 21-27 confirmation did NOT
clear its own thresholds (>=3% AND p<0.05); it was shipped as a default anyway,
on the strength of the congested-corpus recovery. Nothing here reaches p<0.05,
so treat a future corpus result that contradicts it as informative rather than
surprising -- and note two defaults have been reverted on this repo before for
exactly that reason.

`KICAD_HISTORY_COST=0` restores the pre-promotion OFF state for A/B. There is
still no CLI flag and no GUI control: both fronts read these knobs through the
shared engine, so the default reaches them identically and needs no parity
wiring.

Knobs via environment:

  KICAD_HISTORY_COST            mm-equivalent added to each contested cell per
                                conflict event (0 = disabled; SHIPPED 0.1)
  KICAD_HISTORY_CAP             ceiling on the accumulated per-cell history in
                                mm-equivalent (0 = uncapped; SHIPPED 0.5; a
                                cap keeps PRODUCTIVE churn -- the fine-pitch
                                BGA escape field, where the 0802 study found
                                15 rips converge -- from walling itself off)
  KICAD_HISTORY_ESCALATE        repeat-contest multiplier: when > 0,
                                re-charging a cell adds max(inc, escalate x
                                accumulated), so 1.0 doubles per repeat.
                                SHIPPED 0 = flat accumulation
  KICAD_HISTORY_RIP_WEIGHT      fraction of the increment for the
                                whole-footprint rip stamp (SHIPPED 1.0;
                                0 = off)
  KICAD_HISTORY_RADIUS          mm added to the copper half-width when
                                stamping a rip (default 0.25 ~ one
                                clearance)
  KICAD_HISTORY_BLOCKED_WEIGHT  fraction of the increment charged to a failed
                                search's RAW blocked frontier (SHIPPED 0.25;
                                0 = off)
  KICAD_HISTORY_MAX_CELLS       growth guard: past this many cells, each new
                                event EVICTS the lowest-weight cells to make
                                room -- chronological refusal would unprice
                                the endgame conflicts, which are the ones
                                completion is measured on
"""
from __future__ import annotations

from time import perf_counter as _perf
from typing import Dict, Optional

import numpy as np

import env_knobs
from routing_config import GridCoord, GridRouteConfig, REFERENCE_GRID_STEP

# Cell packing, identical in shape to merge_track_proximity_costs' sum-mode
# key: grid coords are well inside +/-2^23.
_OFF = 1 << 23


def history_knobs() -> dict:
    # Copy so a caller mutating its dict cannot poison the read-once values.
    return dict(env_knobs.HISTORY)


def history_enabled() -> bool:
    return env_knobs.HISTORY['cost'] > 0


def _pack(layer, gx, gy) -> np.ndarray:
    return ((np.asarray(layer, dtype=np.int64) << 48)
            | ((np.asarray(gx, dtype=np.int64) + _OFF) << 24)
            | (np.asarray(gy, dtype=np.int64) + _OFF))


class HistoryField:
    """Per-cell accumulated conflict history for one routing call.

    Stored as parallel sorted arrays (packed key -> mm-equivalent weight)
    rather than a dict: an event bumps thousands of cells at once, and the
    per-prepare consumer wants an (N, 4) rows array anyway.
    """

    __slots__ = ('keys', 'weights', 'version', '_rows', '_rows_version',
                 'rips', 'frontiers', 'contests', 'evicted',
                 'last_frontier', 'last_contest',
                 'record_s', 'rows_s')

    def __init__(self):
        self.keys = np.empty(0, dtype=np.int64)
        self.weights = np.empty(0, dtype=np.float64)
        self.version = 0
        self._rows = None
        self._rows_version = -1
        self.rips = 0
        self.frontiers = 0
        self.contests = 0
        self.evicted = 0            # cells evicted by the growth guard
        self.last_frontier = None   # signature of the last frontier charged
        self.last_contest = None    # signature of the last contest charged
        self.record_s = 0.0         # time in the event path (disclosed)
        self.rows_s = 0.0           # time building composition rows

    def accumulate(self, cells: np.ndarray, inc: float) -> None:
        """Charge every packed cell key in ``cells``.

        Fresh cells take ``inc`` mm-equivalent; cells already in the field
        take another flat ``inc`` at the shipped escalate 0, or
        ``max(inc, escalate x accumulated)`` when escalate > 0 (at 1.0 a
        repeat contest DOUBLES the cell's price).

        Kept O(field) per event, not O(field log field): ``keys`` is sorted,
        ``np.unique`` returns the event's cells sorted, so the new cells go in
        by merge-insert instead of re-sorting the whole field.
        """
        if inc <= 0 or cells is None or len(cells) == 0:
            return
        t0 = _perf()
        esc = env_knobs.HISTORY['escalate']
        cells = np.unique(np.asarray(cells, dtype=np.int64))
        if self.keys.size == 0:
            self.keys = cells
            self.weights = np.full(cells.size, float(inc), dtype=np.float64)
        else:
            idx = np.searchsorted(self.keys, cells)
            hit = np.zeros(cells.size, dtype=bool)
            inside = idx < self.keys.size
            hit[inside] = self.keys[idx[inside]] == cells[inside]
            if hit.any():
                # `cells` is unique, so the hit targets are distinct and this
                # fancy-index += cannot lose an update to aliasing.
                w = self.weights[idx[hit]]
                charge = np.maximum(float(inc), esc * w) if esc > 0 \
                    else float(inc)
                self.weights[idx[hit]] = w + charge
            fresh_mask = ~hit
            if fresh_mask.any():
                self.keys = np.insert(self.keys, idx[fresh_mask],
                                      cells[fresh_mask])
                self.weights = np.insert(self.weights, idx[fresh_mask],
                                         float(inc))
        over = self.keys.size - env_knobs.HISTORY['max_cells']
        if over > 0:
            # Growth guard: EVICT the lowest-weight cells (deterministic
            # tie-break on the key) instead of refusing new ones -- the
            # conflicts that arrive after the field fills are the endgame
            # ones completion is measured on, so chronological refusal
            # would unprice exactly the cells that matter most.
            order = np.lexsort((self.keys, self.weights))
            keep = np.ones(self.keys.size, dtype=bool)
            keep[order[:over]] = False
            self.keys = self.keys[keep]        # mask keeps the sort order
            self.weights = self.weights[keep]
            self.evicted += over
        self.version += 1
        self.record_s += _perf() - t0

    def rows(self) -> Optional[np.ndarray]:
        """(N, 4) [layer, gx, gy, cost] rows, or None when empty.

        Memoized on the event version: the field only changes at a conflict,
        while this is read on every per-net prepare.
        """
        if self._rows_version == self.version:
            return self._rows
        t0 = _perf()
        rows = None
        if self.keys.size:
            cap = env_knobs.HISTORY['cap']
            w = self.weights if cap <= 0 else np.minimum(self.weights, cap)
            # cell_cost is linear in mm (int(cost_mm * 1000 / REFERENCE)); do
            # it vectorized, with the same truncation.
            cost = (w * (1000.0 / REFERENCE_GRID_STEP)).astype(np.int64)
            keep = cost > 0
            if keep.any():
                k = self.keys[keep]
                out = np.empty((k.size, 4), dtype=np.int32)
                out[:, 0] = (k >> 48).astype(np.int32)
                out[:, 1] = ((k >> 24) & ((1 << 24) - 1)).astype(np.int32) - _OFF
                out[:, 2] = (k & ((1 << 24) - 1)).astype(np.int32) - _OFF
                out[:, 3] = np.minimum(cost[keep],
                                       np.iinfo(np.int32).max).astype(np.int32)
                rows = out
        self._rows = rows
        self._rows_version = self.version
        self.rows_s += _perf() - t0
        return rows

    def summary(self) -> str:
        k = env_knobs.HISTORY
        peak = float(self.weights.max()) if self.weights.size else 0.0
        evicted = f", {self.evicted} cell(s) evicted by the growth guard" \
            if self.evicted else ""
        return (f"History congestion (#590): {self.contests} contest + "
                f"{self.rips} rip + {self.frontiers} blocked-frontier "
                f"event(s) over {self.keys.size} cell(s), peak "
                f"{min(peak, k['cap']) if k['cap'] > 0 else peak:.2f}mm-equiv"
                f" (inc {k['cost']}mm, cap "
                f"{k['cap'] if k['cap'] > 0 else 'none'}, escalate "
                f"{k['escalate']}, rip weight {k['rip_weight']}, frontier "
                f"weight {k['blocked_weight']}), "
                f"{self.record_s + self.rows_s:.2f}s of field upkeep{evicted}")


def reset_history(config: GridRouteConfig) -> Optional[HistoryField]:
    """Fresh field for one routing call (no-op when disabled).

    Called at batch start. Scoping to the call mirrors the ripped-route
    ghosts; cross-step persistence is a v2 question (#590).
    """
    if not history_enabled():
        config._history_cong = None
        return None
    field = HistoryField()
    config._history_cong = field
    k = history_knobs()
    print(f"History congestion (#590) armed: +{k['cost']}mm-equiv per contest"
          f", cap {k['cap'] if k['cap'] > 0 else 'none'}, escalate "
          f"{k['escalate']}, rip weight {k['rip_weight']} (radius "
          f"+{k['radius']}mm), frontier weight {k['blocked_weight']}")
    return field


def _field(config) -> Optional[HistoryField]:
    """The live field, or None when the feature is off for this call.

    Lazily created so a config that never went through reset_history (a
    cloned config for an oracle sub-route, a direct engine call) still
    records instead of silently dropping its conflicts.
    """
    if not history_enabled():
        return None
    field = getattr(config, '_history_cong', None)
    if field is None:
        field = HistoryField()
        config._history_cong = field
    return field


def _route_cell_keys(saved_result: dict, config: GridRouteConfig,
                     layer_map: Dict[str, int]) -> np.ndarray:
    """Packed cell keys covering a route's copper footprint.

    Segments stamp a disk of (half width + KICAD_HISTORY_RADIUS) along their
    centerline, sampled densely enough that consecutive disks overlap; vias
    stamp (half size + radius) on EVERY layer (a barrel contests all of them).
    """
    coord = GridCoord(config.grid_step)
    extra = env_knobs.HISTORY['radius']
    chunks = []

    for seg in saved_result.get('new_segments') or []:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        r = coord.to_grid_dist((seg.width or 0.0) / 2.0 + extra)
        off = _disk_offsets(r)
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        # Sample the centerline every r/2 cells, not every cell: disks that
        # overlap by half still cover the full-radius corridor (worst-case
        # scallop 0.13 r, inside the same cell at any realistic r), and the
        # emitted row count is otherwise QUADRATIC in 1/grid_step -- a 20 mm
        # segment at the 0.025 fine-pitch grid would emit ~10^5 rows on its
        # own. Same trick as the ripped-route ghost's 1 mm sampling.
        #
        # Sampled by linspace, NOT walk_line: at this spacing the exact
        # Bresenham cells are irrelevant (each sample stamps a disk of radius
        # r around it), and a per-cell Python generator is the single most
        # expensive thing this module could do on a long power trunk.
        span = max(abs(gx2 - gx1), abs(gy2 - gy1))
        n = max(1, -(-span // max(1, r // 2)))       # ceil div, >= 1 interval
        t = np.linspace(0.0, 1.0, n + 1)
        px = np.rint(gx1 + (gx2 - gx1) * t).astype(np.int64)
        py = np.rint(gy1 + (gy2 - gy1) * t).astype(np.int64)
        gx = (px[:, None] + off[:, 0]).ravel()
        gy = (py[:, None] + off[:, 1]).ravel()
        chunks.append(_pack(layer_idx, gx, gy))

    vias = saved_result.get('new_vias') or []
    if vias:
        n_layers = max(1, len(config.layers))
        for via in vias:
            r = coord.to_grid_dist((via.size or 0.0) / 2.0 + extra)
            off = _disk_offsets(r)
            gx0, gy0 = coord.to_grid(via.x, via.y)
            gx = gx0 + off[:, 0]
            gy = gy0 + off[:, 1]
            for li in range(n_layers):
                chunks.append(_pack(li, gx, gy))

    if not chunks:
        return np.empty(0, dtype=np.int64)
    return np.concatenate(chunks)


_DISK_CACHE: Dict[int, np.ndarray] = {}


def _disk_offsets(radius_grid: int) -> np.ndarray:
    """(O, 2) integer offsets within ``radius_grid`` cells (always incl. 0,0)."""
    r = max(0, int(radius_grid))
    off = _DISK_CACHE.get(r)
    if off is None:
        if r == 0:
            off = np.zeros((1, 2), dtype=np.int64)
        else:
            ax = np.arange(-r, r + 1, dtype=np.int64)
            ex, ey = np.meshgrid(ax, ax, indexing='ij')
            m = (ex * ex + ey * ey) <= r * r
            off = np.stack((ex[m], ey[m]), axis=1)
        _DISK_CACHE[r] = off
    return off


def record_rip(config: GridRouteConfig, saved_result: dict,
               layer_map: Optional[Dict[str, int]]) -> None:
    """Rip event. The contested-cell charge already happened at blocking
    analysis (record_contested, which identified this rip's victim); this
    adds the whole-footprint stamp at KICAD_HISTORY_RIP_WEIGHT x the
    increment (shipped 1.0; the corpus A/B found it the ingredient doing the
    work, after v2 had demoted it to 0)."""
    field = _field(config)
    if field is None or not saved_result:
        return
    field.rips += 1
    # The board just changed: a frontier/contest seen after this is a NEW
    # conflict, not the re-analysis the duplicate guard suppresses.
    field.last_frontier = None
    field.last_contest = None
    w = env_knobs.HISTORY['rip_weight']
    if w <= 0 or not layer_map:
        return
    keys = _route_cell_keys(saved_result, config, layer_map)
    if keys.size == 0:
        return
    field.accumulate(keys, env_knobs.HISTORY['cost'] * w)


def record_contested(config: GridRouteConfig, cells) -> None:
    """PRIMARY conflict event (#590 v2): frontier∩blocker intersection.

    ``cells`` are (gx, gy, layer) triples -- the cells of ROUTED nets a
    failed search stalled against, straight from analyze_frontier_blocking's
    per-blocker intersection. This is the engine's analog of PathFinder's
    overused nodes: ground one net holds and another wants. Charged at the
    FULL increment, before any rip, so a ripped blocker's reroute already
    sees its contested ground priced and relocates instead of re-taking it.
    """
    field = _field(config)
    if field is None or cells is None or len(cells) == 0:
        return
    arr = np.asarray(cells, dtype=np.int64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        return
    keys = np.unique(_pack(arr[:, 2], arr[:, 0], arr[:, 1]))
    # Same consecutive-duplicate guard as the frontier event: one failure is
    # often analyzed twice back to back with a different exclude set -- one
    # conflict, one charge. A contest recurring LATER (other events in
    # between) is a genuine repeat and escalates.
    sig = (keys.size, int(keys[0]), int(keys[-1]), int(keys.sum()))
    if sig == field.last_contest:
        return
    field.last_contest = sig
    field.contests += 1
    field.accumulate(keys, env_knobs.HISTORY['cost'])


def record_blocked_frontier(config: GridRouteConfig, blocked_cells) -> None:
    """Weaker conflict event: cells a FAILED search stalled against.

    ``blocked_cells`` are the router's (gx, gy, layer) frontier triples.
    """
    field = _field(config)
    if field is None or blocked_cells is None or len(blocked_cells) == 0:
        return
    w = env_knobs.HISTORY['blocked_weight']
    if w <= 0:
        return
    arr = np.asarray(blocked_cells, dtype=np.int64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        return
    keys = np.unique(_pack(arr[:, 2], arr[:, 0], arr[:, 1]))
    # ONE failure is often analyzed twice back to back (a re-analysis with a
    # different exclude set, or `fresh_blockers` after a rip). That is one
    # conflict, not two -- charge consecutive identical frontiers once. A
    # frontier that recurs LATER, with other events in between, is a genuine
    # repeat and does accrue.
    sig = (keys.size, int(keys[0]), int(keys[-1]), int(keys.sum()))
    if sig == field.last_frontier:
        return
    field.last_frontier = sig
    field.frontiers += 1
    field.accumulate(keys, env_knobs.HISTORY['cost'] * w)


# Composition-pass key for the history field. A tuple, like congestion v2's,
# so it can never collide with the int net ids the same dict carries.
HISTORY_SOURCE_KEY = ('history',)


def add_history_source(ghost_costs, config):
    """Fold the history field into a ``merge_track_proximity_costs`` call's
    ghost_costs (returns it unchanged when the feature is off).

    Every obstacle builder in the engine goes through this one helper, so a
    search path cannot silently drop the field: single-ended (fresh /
    incremental / in-place prepare), diff pair, and the diff-pair layer-swap
    fallback's retry / rip / reroute maps.
    """
    rows = history_rows(config)
    if rows is None:
        return ghost_costs
    merged = dict(ghost_costs or {})
    merged[HISTORY_SOURCE_KEY] = rows
    return merged


def history_rows(config) -> Optional[np.ndarray]:
    """Composition source for merge_track_proximity_costs (None when off).

    Rows are unique per (layer, cell) by construction = within-source dedupe,
    so sum mode counts history exactly once like every other source.
    """
    field = getattr(config, '_history_cong', None)
    if field is None or not history_enabled():
        return None
    return field.rows()


def print_history_summary(config) -> None:
    field = getattr(config, '_history_cong', None)
    if field is not None and (field.rips or field.frontiers or field.contests):
        print(f"  {field.summary()}")
