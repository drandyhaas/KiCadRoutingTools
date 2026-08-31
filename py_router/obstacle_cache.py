"""
Net obstacle caching for PCB routing.

Provides pre-computation and incremental updates for per-net obstacles,
dramatically speeding up routing by avoiding redundant obstacle calculations.
"""
from __future__ import annotations

from typing import List, Tuple, Dict, Set, Optional
from dataclasses import dataclass, field
import math
import numpy as np

from kicad_parser import PCBData
from routing_config import GridRouteConfig, GridCoord
import routing_defaults as defaults
from routing_utils import build_layer_map, iter_pad_blocked_cells, \
    pad_blocked_cells_array, segment_blocked_cells_array, segment_blocked_spans, \
    circle_offsets, GRID_TIE_EPS
from net_queries import expand_pad_layers


# Import Rust router
import sys
import env_knobs
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads

# Over-blocking audit F3 A/B kill-switch: 1 restores the pre-fix cache stamps
# (obstacle net's CONFIGURED width for narrow existing copper + via->track
# rings). Temporary -- remove once the actual-width fix is validated corpus-wide.
_LEGACY_CACHE_WIDTH = os.environ.get('KICAD_OBSCACHE_LEGACY_WIDTH', '') in ('1', 'true', 'on')

try:
    from grid_router import GridObstacleMap
except ImportError:
    GridObstacleMap = None


_PACK_OFFSET = 1 << 20  # grid coords stay well within +/-2^20 at any allowed grid step

# --- Obstacle-map ref-count ledger (issue #309) --------------------------------
# KICAD_OBSTACLE_LEDGER=1 arms a cheap per-object ledger over the persistent
# working map: every NetObstacleData gets a creation serial, and every
# add/remove of a cache object against the WORKING map (identified by id(); set
# by build_working_obstacle_map) is recorded with its call site. The invariant
# `working == base + sum(current caches)` holds iff, at end of run, every
# serial's adds - removes == 1 for objects still resident in the cache dict and
# == 0 for replaced/dropped ones. obstacle_ledger_report() prints the serials
# (and sites) that violate it - naming the exact leaking interleaving that the
# whole-map KICAD_OBSTACLE_AUDIT diff can only total. Raw (non-cache) list ops
# on the working map are accounted separately by get_stats() deltas per site.
import itertools as _it
_UID = _it.count(1)
_LEDGER_ENV = os.environ.get("KICAD_OBSTACLE_LEDGER") == "1"
_LEDGER = None
if _LEDGER_ENV:
    import itertools as _itertools
    _LEDGER = {
        "wid": None,             # id() of the persistent working map
        "serial": _itertools.count(1),
        "meta": {},              # serial -> (net_id, creation site)
        "adds": {},              # serial -> {site: count}
        "removes": {},           # serial -> {site: count}
        "raw": {},               # site -> [d_cells, d_vias] cumulative (raw list ops)
        "events": 0,
    }


# ---------------------------------------------------------------------------
# CELL WATCH (#803). The ledger above accounts per cache-OBJECT and per call
# SITE; neither can answer "who unblocked THIS cell". That is the question a
# contact DRC violation asks: a track ended on a foreign via's centre, so that
# cell must have been free when the track routed, and something freed it.
#
#   KICAD_CELL_WATCH="77.80,91.20;75.70,90.50"   (board mm, ';'-separated)
#   KICAD_CELL_WATCH_GRID=0.1                    (grid step; must match the run)
#
# Every cache add/remove and every raw list op re-queries the watched cells and
# prints only TRANSITIONS, with the call site -- so the output is the blocked/
# free history of exactly those cells, not a flood.
_CELL_WATCH = None
_CELL_WATCH_STATE = {}


def _cell_watch_spec():
    global _CELL_WATCH
    if _CELL_WATCH is not None:
        return _CELL_WATCH
    spec = os.environ.get("KICAD_CELL_WATCH", "").strip()
    if not spec:
        _CELL_WATCH = []
        return _CELL_WATCH
    step = float(os.environ.get("KICAD_CELL_WATCH_GRID", "0.1"))
    out = []
    for part in spec.split(";"):
        part = part.strip()
        if not part:
            continue
        try:
            xs, ys = part.split(",")
            x, y = float(xs), float(ys)
        except ValueError:
            print(f"[CELLWATCH] bad spec {part!r} -- want 'x,y' in mm")
            continue
        out.append((int(round(x / step)), int(round(y / step)), x, y))
    print(f"[CELLWATCH] watching {len(out)} cell(s) at grid {step}: "
          + ", ".join(f"({x},{y})->g({gx},{gy})" for gx, gy, x, y in out))
    _CELL_WATCH = out
    return _CELL_WATCH


def ledger_cell_watch(obstacles, site: str) -> None:
    """Print blocked/free TRANSITIONS of the watched cells, with the site."""
    cells = _cell_watch_spec()
    if not cells:
        return
    try:
        n = obstacles.num_layers
        n = n() if callable(n) else n
    except Exception:
        n = 4
    # VIA-block state first: it is layer-independent, has NO source/target
    # override, and it -- not is_blocked -- is what gates via PLACEMENT. A cell
    # can be correctly cell-blocked while its via-block is missing, which is
    # exactly how a via lands on a foreign track.
    for gx, gy, mx, my in cells:
        try:
            vb = bool(obstacles.is_via_blocked(gx, gy))
        except Exception:
            vb = None
        if vb is not None:
            key = (id(obstacles), gx, gy, 'via')
            prev = _CELL_WATCH_STATE.get(key)
            if prev is None:
                _CELL_WATCH_STATE[key] = vb
            elif prev != vb:
                _CELL_WATCH_STATE[key] = vb
                print(f"[CELLWATCH] ({mx},{my}) VIA-BLOCK: "
                      f"{'BLOCKED' if prev else 'free'} -> "
                      f"{'BLOCKED' if vb else 'FREE'}   @ {site}")
    for gx, gy, mx, my in cells:
        for li in range(int(n)):
            try:
                b = bool(obstacles.is_blocked(gx, gy, li))
            except Exception:
                continue
            key = (id(obstacles), gx, gy, li)
            prev = _CELL_WATCH_STATE.get(key)
            if prev is None:
                _CELL_WATCH_STATE[key] = b
                continue
            if prev != b:
                _CELL_WATCH_STATE[key] = b
                print(f"[CELLWATCH] ({mx},{my}) layer{li}: "
                      f"{'BLOCKED' if prev else 'free'} -> "
                      f"{'BLOCKED' if b else 'FREE'}   @ {site}")


def _ledger_site(depth: int = 2, frames: int = 3) -> str:
    import sys as _sys
    parts = []
    try:
        f = _sys._getframe(depth)
    except ValueError:
        return "?"
    for _ in range(frames):
        if f is None:
            break
        parts.append(f"{os.path.basename(f.f_code.co_filename)}:{f.f_code.co_name}:{f.f_lineno}")
        f = f.f_back
    return " <- ".join(parts)


def ledger_set_working_map(obstacles) -> None:
    """Mark `obstacles` as THE persistent working map the ledger audits.

    Also resets the per-run books: a process can host MORE THAN ONE routing
    run (route.py's end-of-run reconciliation re-invokes batch_route on the
    written output, #348), and each run has its own working map and cache
    objects. Without the reset, the second run's ledger report grades the
    FIRST run's records against the second run's final cache and prints
    phantom UNBALANCED serials."""
    if _LEDGER is not None:
        _LEDGER["wid"] = id(obstacles)
        # KEEP meta across the reset. Serials are unique for the life of the
        # process, so an old entry can never collide with a new one -- and this
        # run's caches are precomputed BEFORE its working map is built
        # (route.py:1928 then :1952), so wiping meta here erased the net-id of
        # every initially-built cache. The report then printed "net ?" for
        # exactly the objects most likely to be unbalanced, which is the whole
        # question it exists to answer.
        _LEDGER["adds"] = {}
        _LEDGER["removes"] = {}
        _LEDGER["raw"] = {}
        _LEDGER["events"] = 0


def _ledger_cache_op(op: str, obstacles, cache_data) -> None:
    if _LEDGER is None or _LEDGER["wid"] != id(obstacles):
        return
    serial = getattr(cache_data, "_audit_serial", 0)
    site = _ledger_site(depth=3)
    book = _LEDGER["adds" if op == "add" else "removes"]
    per = book.setdefault(serial, {})
    per[site] = per.get(site, 0) + 1
    _LEDGER["events"] += 1


def ledger_raw_delta(obstacles, site_tag: str, d_cells: int, d_vias: int) -> None:
    """Record a raw (non-cache) mutation of the working map, as get_stats deltas."""
    if _LEDGER is None or _LEDGER["wid"] != id(obstacles):
        return
    ent = _LEDGER["raw"].setdefault(site_tag, [0, 0])
    ent[0] += d_cells
    ent[1] += d_vias
    _LEDGER["events"] += 1


def run_obstacle_audit(base_obstacles, working_obstacles,
                       net_obstacles_cache: Dict[int, "NetObstacleData"],
                       label: str = "") -> None:
    """End-of-run ref-count integrity audit (issue #309), shared by every
    front-end that maintains a persistent working map + per-net cache dict.

    Invariant: working == base + sum(current net caches). Clones the working
    map, removes every net's CURRENT cache, and compares entry counts to the
    base. Any residual in the ref-counted sets is a leak (an add not mirrored
    by a remove) or an over-decrement. Fully defensive; env-gated by callers.
    """
    try:
        if base_obstacles is None or working_obstacles is None:
            return
        probe = working_obstacles.clone_fresh()
        cache = net_obstacles_cache or {}
        for _nid, _cd in list(cache.items()):
            remove_net_obstacles_from_cache(probe, _cd)
        labels = ("blocked_cells", "blocked_vias", "stub_prox",
                  "layer_prox", "cross_layer", "source_target", "free_vias",
                  "blocked_vias_small")  # #568: 8th get_stats element
        # HARD = ref-counted obstacle sets that cause DRC/routing errors when
        # leaked; SOFT = proximity/cost maps whose residual is expected.
        # blocked_vias_small is HARD with a sharper failure mode than a leak:
        # an unbalanced small map makes rung-1 vias LEGAL where they aren't.
        hard = {"blocked_cells", "blocked_vias", "source_target", "free_vias",
                "blocked_vias_small"}
        bs, ps = base_obstacles.get_stats(), probe.get_stats()
        resid = [(labels[i], ps[i] - bs[i])
                 for i in range(min(len(labels), len(bs), len(ps)))]
        leak = [(n, d) for n, d in resid if d != 0 and n in hard]
        soft = [(n, d) for n, d in resid if d != 0 and n not in hard]
        print("\n" + "=" * 60)
        print(f"[OBSTACLE AUDIT{' ' + label if label else ''}] "
              f"working - sum(caches) vs base ({len(cache)} net caches removed)")
        if leak:
            print("  LEAK/DESYNC in ref-counted obstacle maps:")
            for n, d in leak:
                print(f"    {n}: {d:+d} cells unaccounted for by any net cache")
        else:
            print("  BALANCED: ref-counted maps (blocked_cells/blocked_vias/"
                  "source_target/free_vias) return exactly to base.")
        if soft:
            print("  (soft cost maps, residual expected: "
                  + ", ".join(f"{n} {d:+d}" for n, d in soft) + ")")
        print("=" * 60)
        if _LEDGER_ENV:
            obstacle_ledger_report(net_obstacles_cache)
    except Exception as e:
        print(f"[OBSTACLE AUDIT] skipped ({e})")


def obstacle_ledger_report(final_cache: Dict[int, "NetObstacleData"]) -> None:
    """Print every cache object whose working-map adds/removes don't balance."""
    if _LEDGER is None:
        print("[OBSTACLE LEDGER] not armed (KICAD_OBSTACLE_LEDGER != 1)")
        return
    resident = {getattr(cd, "_audit_serial", 0) for cd in (final_cache or {}).values()}
    serials = set(_LEDGER["adds"]) | set(_LEDGER["removes"])
    bad = []
    for s in sorted(serials):
        adds = _LEDGER["adds"].get(s, {})
        removes = _LEDGER["removes"].get(s, {})
        balance = sum(adds.values()) - sum(removes.values())
        expected = 1 if s in resident else 0
        if balance != expected:
            bad.append((s, balance, expected, adds, removes))
    print("\n" + "=" * 60)
    print(f"[OBSTACLE LEDGER] {_LEDGER['events']} events, "
          f"{len(serials)} cache objects touched the working map, "
          f"{len(resident)} resident at end")
    if _LEDGER["events"] == 0:
        print("  WARNING: zero events recorded - instrument did not engage "
              "(working map id never matched?)")
    for s, balance, expected, adds, removes in bad:
        net_id, created = _LEDGER["meta"].get(s, ("?", "?"))
        print(f"  UNBALANCED serial {s} (net {net_id}): adds-removes={balance}, "
              f"expected {expected} (resident={s in resident})")
        print(f"    created at: {created}")
        for site, n in adds.items():
            print(f"    +{n} add    {site}")
        for site, n in removes.items():
            print(f"    -{n} remove {site}")
    if not bad:
        print("  cache-object ledger BALANCED (leak, if any, is in raw ops below)")
    raw = {k: v for k, v in _LEDGER["raw"].items() if v[0] or v[1]}
    if raw:
        print("  raw-op cumulative stats deltas (should cancel per add/remove pair):")
        for site, (dc, dv) in sorted(raw.items()):
            print(f"    cells {dc:+d} vias {dv:+d}  {site}")
    print("=" * 60)

# Bitmap dedupe allocates one bool per cell in the rows' bounding box; cap the
# allocation (200M bools = 200MB) and fall back to sorting for outliers.
_BITMAP_DEDUPE_MAX_CELLS = 200_000_000


def expand_via_spans(spans: "np.ndarray") -> "np.ndarray":
    """(K,3) [gx, lo, hi] spans -> (N,2) [gx, gy] cells.

    Only for the few consumers that genuinely need cells (the #568 small-via
    array, diagnostics). The routing path never calls this: expanding in Python
    costs 7.4 us against a 0.17 us memo hit, which is why the stamping APIs
    expand in Rust instead.
    """
    if spans is None or len(spans) == 0:
        return np.empty((0, 2), dtype=np.int32)
    lo = spans[:, 1].astype(np.int64)
    counts = (spans[:, 2].astype(np.int64) - lo + 1)
    counts[counts < 0] = 0
    total = int(counts.sum())
    if total == 0:
        return np.empty((0, 2), dtype=np.int32)
    out = np.empty((total, 2), dtype=np.int32)
    out[:, 0] = np.repeat(spans[:, 0], counts)
    starts = np.zeros(len(counts), dtype=np.int64)
    np.cumsum(counts[:-1], out=starts[1:])
    out[:, 1] = (np.repeat(lo, counts)
                 + (np.arange(total, dtype=np.int64) - np.repeat(starts, counts)))
    return out


def _unique_rows(arr: np.ndarray) -> np.ndarray:
    """Row-deduplicate an int32 (N,2) or (N,3) cell array.

    Equivalent to np.unique(arr, axis=0) except for row order, which callers
    must not rely on (these are unordered cell sets). np.unique(axis=0) sorts
    rows as void records, which dominated routing time at fine grid steps;
    marking cells in a bounding-box bitmap dedupes without sorting at all.
    """
    mins = arr.min(axis=0)
    rel = arr - mins  # stays int32; ravel_multi_index widens internally
    dims = (rel.max(axis=0) + 1).astype(np.int64)
    total = int(np.prod(dims))
    if total > _BITMAP_DEDUPE_MAX_CELLS:
        # Sparse outlier (huge extent): pack rows into scalar int64 keys so
        # np.unique sorts scalars instead of void records.
        a = arr.astype(np.int64)
        key = ((a[:, 0] + _PACK_OFFSET) << 21) | (a[:, 1] + _PACK_OFFSET)
        if a.shape[1] == 3:
            key = (key << 8) | a[:, 2]
        _, idx = np.unique(key, return_index=True)
        return arr[idx]
    seen = np.zeros(total, dtype=bool)
    seen[np.ravel_multi_index(rel.T, dims)] = True
    uniq = np.flatnonzero(seen)
    out = np.empty((len(uniq), arr.shape[1]), dtype=arr.dtype)
    for i, col in enumerate(np.unravel_index(uniq, dims)):
        out[:, i] = col + mins[i]
    return out


@dataclass
class NetObstacleData:
    """Cached obstacle data for a single net.

    Pre-computed blocked cells and vias for fast batch adding.
    Uses numpy arrays for memory efficiency (12 bytes per cell vs 72+ for tuples).
    """
    # Blocked cells as numpy array of shape (N, 3) with columns [gx, gy, layer_idx]
    # dtype=int32 for efficient storage
    blocked_cells: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.int32))
    # Blocked via positions as numpy array of shape (M, 2) with columns [gx, gy]
    blocked_vias: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.int32))
    # #568: via-block cells at the SMALL fab-rung reserve (KICAD_VIA_RUNG=2
    # only; empty otherwise). Stamped/removed in lockstep with blocked_vias --
    # once the map's small set is non-empty, rung-1 queries trust it
    # EXCLUSIVELY for dynamic copper, so an unbalanced stamp doesn't just leak
    # cells, it makes small vias legal where they aren't (audit-enforced).
    blocked_vias_small: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.int32))
    # #815: SEGMENT keep-outs, as capsule SPANS instead of cells --
    # (K, 4) [gx, gy_lo, gy_hi, layer] and (K, 3) [gx, gy_lo, gy_hi].
    #
    # Why segments and not everything: a capsule is convex, so a column is one
    # contiguous run and a span costs 12 bytes where its ~8.3 cells cost 66 --
    # 5.2x denser. That density is the point: the capsule memo was thrashing
    # (on glasgow, 2,270,477 of 2,317,497 misses were EVICTIONS, ~54 s/route
    # re-rasterising capsules it had already computed), and the memo can only
    # hold a working set the CONSUMER is willing to take in span form.
    # Pad and via keep-outs stay cells: they come from the integer offset
    # tables (circle_offsets / pad_blocked_cells_array), not the capsule memo,
    # so spans would buy nothing there.
    #
    # These are stamped and UNSTAMPED in lockstep with the cell arrays by
    # add_/remove_net_obstacles_from_cache. Rust expands them, so the add and
    # the remove walk the identical cell multiset and the refcounts balance --
    # the invariant route.py asserts and test_obstacle_addremove_parity.py /
    # test_obstacle_map_balance.py defend. Unlike the cell arrays these are NOT
    # row-deduplicated: two overlapping same-net capsules refcount a shared
    # cell twice, which is harmless precisely because the remove is symmetric.
    blocked_cell_spans: np.ndarray = field(default_factory=lambda: np.empty((0, 4), dtype=np.int32))
    blocked_via_spans: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.int32))


# #568 run-scoped interlock. `_via_rung_unsafe` lives on pcb_data, but the
# RAW map mirrors in obstacle_map get only (obstacles, config) -- they cannot
# see it, so they used to arm the small map on the env var alone, defeating
# the interlock exactly where it matters (an unfrozen base + raw-only small
# stamps = rung-1 searches blind to base copper). Arming is a property of the
# RUN, so both sides consult this one flag.
_RUNG_UNSAFE = False


def set_rung_unsafe(flag: bool) -> None:
    """Disarm (or re-arm) #568 rung-1 legality for this run. Called by the
    engine when it builds a base the rung-1 map cannot be trusted against."""
    global _RUNG_UNSAFE
    _RUNG_UNSAFE = bool(flag)


def rung_small_armed() -> bool:
    """True when rung-1 small-via legality may be stamped/trusted this run."""
    return env_knobs.VIA_RUNG_2 and not _RUNG_UNSAFE


def _small_via_pair(config, pcb_data):
    """#568: the fab-ladder (diameter, drill) the rung-1 legality map is
    stamped at -- the FIRST ladder entry smaller than the configured via,
    the SAME selection rule as the boxed-endpoint rung search
    (single_ended_routing._rung_search_pair; the mid-retry _via_rung_retry
    was removed 2026-08-05), so map and search agree by construction. None
    when rust mode is off
    (KICAD_VIA_RUNG != 2) or no smaller rung exists."""
    if not rung_small_armed():
        return None
    # Safety interlock: rung-1 legality is only sound when BASE copper blocks
    # every rung (the frozen static bitmap). A flow that builds an UNFROZEN
    # base and then adds small-stamped caches would leave base copper missing
    # from the small map -- under-blocking, not leaking. Such flows (the vis
    # branch) mark the board; stamping then never arms, small stays empty,
    # and rung-1 queries fall back to full-size legality (safe everywhere).
    if getattr(pcb_data, '_via_rung_unsafe', False):
        set_rung_unsafe(True)   # make the raw mirrors honor it too
        return None
    try:
        from fab_tiers import fab_floor_ladder
        ncu = len([l for l in (pcb_data.board_info.copper_layers or [])
                   if l.endswith('.Cu')]) or 2
        for f in fab_floor_ladder(ncu):
            pair = (round(f['via_diameter'], 3), round(f['via_drill'], 3))
            if pair[0] < config.via_size - 1e-9:
                return pair
    except Exception:
        return None
    return None


def precompute_net_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              extra_clearance: float = 0.0,
                              diagonal_margin: float = defaults.DIAGONAL_MARGIN,
                              _small_pass: bool = False) -> NetObstacleData:
    """Pre-compute obstacle cells for a single net.

    Returns cached data that can be quickly added to obstacle maps via batch operations.

    Args:
        pcb_data: PCB data structure
        net_id: Net ID to compute obstacles for
        config: Routing configuration
        extra_clearance: Additional clearance (for diff pair routing)
        diagonal_margin: Extra margin for via blocking near diagonal segments

    Returns:
        NetObstacleData with blocked_cells and blocked_vias lists
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)

    # Cross-class clearance (PR392): this per-net cache is the keep-out that OTHER
    # routed nets must respect around net_id's copper, so it is priced at net_id's
    # own KiCad pairwise clearance = max(routing-side floor, net_id's class). A
    # single value per net keeps the per-layer precompute cheap (one lookup, no
    # per-obstacle branching). Inert (== config.clearance) when no map is set, so
    # the whole function is byte-identical to pre-PR392 then.
    obs_clearance = config.obstacle_clearance(net_id)

    # Collect cell arrays; deduplicated with np.unique at the end (same
    # unique-cell semantics as the per-cell sets this replaces)
    blocked_cells_set: List["np.ndarray"] = []
    blocked_vias_set: List["np.ndarray"] = []
    # #815: segment keep-outs accumulate here in SPAN form; pads and vias stay
    # in the cell lists above (they come from integer offset tables, not the
    # capsule memo, so spans buy nothing there).
    blocked_cell_spans_set: List["np.ndarray"] = []
    blocked_via_spans_set: List["np.ndarray"] = []

    # Precompute per-layer expansion values for impedance-controlled and power net routing
    # Use to_grid_dist_safe for via-related clearances to avoid grid quantization DRC errors
    expansion_mm_by_layer = {}
    via_block_mm_by_layer = {}
    layer_widths = []  # per-layer future-routing-track width (impedance / power)
    for layer_name in config.layers:
        # Use per-net width for power nets, otherwise layer width (impedance) or default
        layer_width = config.get_net_track_width(net_id, layer_name)
        # Routing-side width for the via->track ring: the FUTURE track that must
        # clear this net's vias belongs to whatever net routes next, so its
        # half-width is the routing-side RESERVE width (#156: nominal for the
        # single-ended engine, the impedance layer width for the diff engine) --
        # never this net's own configured (power) width, which the routed net's
        # track_margin already covers. Matches _via_track_expansion_per_layer
        # (obstacle_map), whose docstring calls the wider variant out as
        # double-counting.
        # KICAD_OBSCACHE_LEGACY_WIDTH=1 restores the pre-fix stamps (A/B only).
        layer_widths.append(layer_width if _LEGACY_CACHE_WIDTH
                            else config.route_reserve_width(layer_name))
        # #498: a .kicad_dru layer rule REPLACES the net/class value on its layer.
        clr_l = config.layer_clearance(layer_name, obs_clearance)
        # A track-scoped DRU rule RAISES the seg-vs-seg capsule only (#735);
        # the via ring keeps clr_l. Add/remove parity is automatic (the same
        # NetObstacleData arrays are batch-added and batch-removed).
        trk_l = config.track_obstacle_clearance(net_id, clr_l)
        expansion_mm = layer_width / 2 + trk_l + config.route_reserve_width(layer_name) / 2 + extra_clearance
        # Float keep-out half-width for the capsule segment stamp (no floor): the
        # true perpendicular clearance, so off-grid / diagonal tracks are covered.
        expansion_mm_by_layer[layer_name] = max(coord.grid_step, expansion_mm)
        # Segment via-block: future ROUTE via (config.via_size) near this net's copper.
        via_block_mm = config.via_size / 2 + layer_width / 2 + clr_l + extra_clearance
        via_block_mm_by_layer[layer_name] = via_block_mm

    # Process segments. Keep-out from the segment's ACTUAL width in BOTH
    # directions, not the net's configured width: a pre-existing wide/diff-pair
    # trace (e.g. a 0.2mm trunk placed by route_diff) under-reserved when
    # stamped at the default 0.127 track width (#172); and a wide-CONFIGURED
    # net's narrow pre-existing copper (a 0.1mm fanout stub on a net configured
    # 0.3 via --power-nets) over-blocked by (configured-actual)/2 per side when
    # the old max(layer_w, seg_w) let the configured width win -- sealing real
    # escape corridors past power stubs (over-blocking audit F3). The base map
    # (build_base_obstacle_map) already stamps foreign copper at seg.width;
    # this keeps the cache at parity. Normal tracks (seg.width == configured)
    # are unaffected - no blanket margin.
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            # Copper on a layer OUTSIDE config.layers: tracks cannot go there,
            # but a VIA spans the whole stack and must respect it (butterstick
            # DQ11 class -- see build_base_obstacle_map). Via keep-out only.
            if seg.layer.endswith('.Cu'):
                seg_w = seg.width if (getattr(seg, 'width', 0) and seg.width > 0) \
                    else config.track_width
                via_block_mm = (config.via_size / 2 + seg_w / 2
                                + config.layer_clearance(seg.layer, obs_clearance)  # #498
                                + extra_clearance)
                _vs = segment_blocked_spans(          # #815
                    seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                    via_block_mm, coord.grid_step)
                if len(_vs):
                    blocked_via_spans_set.append(_vs)
            continue
        layer_w = config.get_net_track_width(net_id, seg.layer)
        seg_w = seg.width if (getattr(seg, 'width', 0) and seg.width > 0) else layer_w
        own_half = (max(layer_w, seg_w) if _LEGACY_CACHE_WIDTH else seg_w) / 2
        if (seg_w <= layer_w) if _LEGACY_CACHE_WIDTH else (seg_w == layer_w):
            expansion_mm = expansion_mm_by_layer.get(seg.layer, coord.grid_step)
            via_block_mm = via_block_mm_by_layer.get(seg.layer)
        else:
            _clr_l = config.layer_clearance(seg.layer, obs_clearance)  # #498
            _trk_l = config.track_obstacle_clearance(net_id, _clr_l)  # dru track rule
            expansion_mm = max(coord.grid_step,
                               own_half + _trk_l
                               + config.route_reserve_width(seg.layer) / 2 + extra_clearance)
            via_block_mm = config.via_size / 2 + own_half + _clr_l + extra_clearance
        _collect_segment_obstacles(seg, coord, layer_idx, expansion_mm,
                                   blocked_cell_spans_set, blocked_via_spans_set,
                                   via_block_mm)

    # Process vias. Keep-out from the via's ACTUAL size, not config.via_size: a
    # fanout via-in-pad is larger (e.g. 0.45 vs 0.3), and using config under-
    # expanded it by ~half the size difference. At grid 0.1 that was masked by the
    # off-grid offset (off_cells) of those vias; at grid 0.05 the same vias land
    # on-grid (off_cells=0), exposing 74 track-via grazes. Matches the non-cache
    # add_net_vias_as_obstacles, which already uses via.size.
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        vs = via.size if (getattr(via, 'size', 0) and via.size > 0) else config.via_size
        # #498: the via meets each layer's copper ON that layer -> per-layer rule.
        via_track_list = [max(1, coord.to_grid_dist_safe(
            vs / 2 + lw / 2 + config.layer_clearance(ln, obs_clearance) + extra_clearance))
            for ln, lw in zip(config.layers, layer_widths)]
        # Via-via: this via (actual size) vs a future ROUTE via (config.via_size).
        # Float radius (no floor) so the disc threshold blocks the true clearance.
        # #498: barrels meet on every stack layer -> stack max.
        via_via_radius = max(1.0, (vs / 2 + config.via_size / 2
                                   + config.stack_clearance(obs_clearance)) * coord.inv_step)
        _collect_via_obstacles(via, coord, num_layers, via_track_list,
                                via_via_radius, diagonal_margin,
                                blocked_cells_set, blocked_vias_set)
        # #441: net-INDEPENDENT drill hole-to-hole keepout, stamped into the via
        # map IN ADDITION to the copper via-via disc above (which is copper-only:
        # vs/2 + via_size/2 + clearance). A future ROUTE via must also clear this
        # via's DRILL by the fab hole-to-hole floor -- without it two different-net
        # vias drill within it (vis_nir_spec / cryologger_aws: diff-net via drills
        # ~0.26mm overlap on boards whose min_hole_to_hole > the copper spacing).
        # Mirrors precompute_via_placement_obstacles's plane-path disc exactly
        # (float-centre, strict <); added to blocked_vias_set so it is added AND
        # removed with the whole NetObstacleData -- ref-count balanced (#208).
        _h2h = getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0
        if _h2h > 0 and getattr(via, 'drill', 0) > 0:
            _gx, _gy = coord.to_grid(via.x, via.y)
            _req = via.drill / 2.0 + config.via_drill / 2.0 + _h2h
            _de = coord.to_grid_dist_safe(_req) + 1  # ceil + 1-cell bbox margin
            _off = np.arange(-_de, _de + 1)
            _dxg, _dyg = np.meshgrid(_off, _off, indexing="ij")
            _cx = (_gx + _dxg) * config.grid_step
            _cy = (_gy + _dyg) * config.grid_step
            # Boundary cells resolve OPEN (GRID_TIE_EPS), exactly as
            # block_via_cells_near_drills does -- this disc is a documented
            # MIRROR of it, and giving one side the epsilon and not the other
            # desynced the incremental via map from a fresh rebuild by 2 cells
            # (test_shared_via_maps). drill/2 + via_drill/2 + h2h lands on an
            # exact grid multiple readily: 0.15 + 0.15 + 0.5 = 0.8 = 16 x 0.05.
            _dm = ((_cx - via.x) ** 2 + (_cy - via.y) ** 2) < (_req - GRID_TIE_EPS) ** 2
            if _dm.any():
                blocked_vias_set.append(
                    np.column_stack([(_gx + _dxg)[_dm], (_gy + _dyg)[_dm]]).astype(np.int32))

    # Process pads
    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _collect_pad_obstacles(pad, coord, layer_map, config, extra_clearance,
                                blocked_cells_set, blocked_vias_set,
                                obs_clearance=obs_clearance)

    # Concatenate and deduplicate (the Rust map refcounts batch adds, so
    # each cell must appear once per net - same as the old set semantics)
    if blocked_cells_set:
        blocked_cells_arr = _unique_rows(np.concatenate(blocked_cells_set))
    else:
        blocked_cells_arr = np.empty((0, 3), dtype=np.int32)

    if blocked_vias_set:
        blocked_vias_arr = _unique_rows(np.concatenate(blocked_vias_set))
    else:
        blocked_vias_arr = np.empty((0, 2), dtype=np.int32)

    # Spans are NOT row-deduplicated: overlapping same-net capsules refcount a
    # shared cell more than once, which is harmless because the unstamp walks
    # the identical array (see NetObstacleData.blocked_cell_spans).
    blocked_cell_spans_arr = (np.concatenate(blocked_cell_spans_set)
                              if blocked_cell_spans_set
                              else np.empty((0, 4), dtype=np.int32))
    blocked_via_spans_arr = (np.concatenate(blocked_via_spans_set)
                             if blocked_via_spans_set
                             else np.empty((0, 3), dtype=np.int32))

    data = NetObstacleData(
        blocked_cells=blocked_cells_arr,
        blocked_vias=blocked_vias_arr,
        blocked_cell_spans=blocked_cell_spans_arr,
        blocked_via_spans=blocked_via_spans_arr,
    )
    # #568 dual stamping: a second pass with the via size/drill swapped for the
    # small fab rung; only its blocked_vias survive (blocked_cells are
    # via-size-independent and discarded). Two-pass is the correct-first cut --
    # single-pass dual radii is the optimization if the cost shows up.
    if not _small_pass:
        _sp = _small_via_pair(config, pcb_data)
        if _sp is not None:
            from dataclasses import replace as _dc_replace
            _sd = precompute_net_obstacles(
                pcb_data, net_id,
                _dc_replace(config, via_size=_sp[0], via_drill=_sp[1]),
                extra_clearance, diagonal_margin, _small_pass=True)
            # #568 small rung still stamps CELLS (add_blocked_vias_small_batch
            # has no span twin, and this path is off unless KICAD_VIA_RUNG=2),
            # so expand the segment spans back and merge with the cell part.
            _small_cells = [_sd.blocked_vias]
            if len(_sd.blocked_via_spans):
                _small_cells.append(expand_via_spans(_sd.blocked_via_spans))
            _small_cat = [a for a in _small_cells if len(a)]
            data.blocked_vias_small = (_unique_rows(np.concatenate(_small_cat))
                                       if _small_cat
                                       else np.empty((0, 2), dtype=np.int32))
    if not _small_pass:
        # A STABLE identity for every cache object, armed or not. id() cannot
        # serve: CPython recycles ids, and make_local_window mints a
        # short-lived PCBData (and its caches) per tap/rescue, so a freed
        # object's id is readily handed straight to its successor. Anything
        # keying bookkeeping on id() therefore attributes one object's state to
        # another -- net_cost._cache_for documents the same hazard for boards.
        # Measured: an id-keyed residency tracker misattributed cache objects
        # and made a repair fire on the wrong ones.
        data._cache_uid = next(_UID)
    if _LEDGER is not None and not _small_pass:
        data._audit_serial = data._cache_uid
        _LEDGER["meta"][data._audit_serial] = (net_id, _ledger_site(depth=2))
    return data


def _collect_segment_obstacles(seg, coord: GridCoord, layer_idx: int,
                                track_margin_mm: float,
                                blocked_cells: List["np.ndarray"],
                                blocked_vias: List["np.ndarray"],
                                via_block_mm: float):
    """Collect segment obstacle SPANS into lists (no obstacle map modification).

    Both keep-outs are an exact capsule measured from the REAL float segment
    (handles off-grid endpoints + diagonals): the track at `track_margin_mm`, the
    via at `via_block_mm`. Matches build_base's _add_segment_obstacle.

    #815: appends SPANS, not cells -- `blocked_cells` receives (K, 4)
    [gx, gy_lo, gy_hi, layer] and `blocked_vias` (K, 3) [gx, gy_lo, gy_hi].
    The lists are therefore the span accumulators, not the cell ones.
    """
    # #815: SPAN form. Same membership as segment_blocked_cells_array (both
    # derive from routing_utils._capsule_mask, so they cannot disagree), at
    # ~5.2x less memory -- which is what lets the capsule memo hold a working
    # set the cell form evicted. Rust expands these when stamping.
    spans = segment_blocked_spans(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                                  track_margin_mm, coord.grid_step)
    if len(spans):
        rows = np.empty((len(spans), 4), dtype=np.int32)
        rows[:, :3] = spans
        rows[:, 3] = layer_idx
        blocked_cells.append(rows)

    vspans = segment_blocked_spans(
        seg.start_x, seg.start_y, seg.end_x, seg.end_y, via_block_mm, coord.grid_step)
    if len(vspans):
        blocked_vias.append(vspans)


def _collect_via_obstacles(via, coord: GridCoord, num_layers: int,
                            via_track_expansion_grid, via_via_expansion_grid: int,
                            diagonal_margin: float,
                            blocked_cells: List["np.ndarray"],
                            blocked_vias: List["np.ndarray"]):
    """Collect via obstacle cells into sets (no obstacle map modification).

    Args:
        via_track_expansion_grid: Either a single int or list of ints (per-layer) for impedance control
    """
    gx, gy = coord.to_grid(via.x, via.y)
    center = np.array([gx, gy], dtype=np.int32)

    # Sub-grid offset of the real via centre from its quantized cell. An off-grid
    # via (e.g. a BGA fanout via-in-pad) has its blocked disc centred on the
    # ROUNDED cell, so foreign copper that clears the rounded centre still grazes
    # the TRUE centre by up to the offset (issue #70). Grow each blocking radius by
    # this offset so the disc covers the real via. On-grid (router-placed) vias
    # have offset ~0 and are unchanged. Mirror of obstacle_map._add_via_obstacle.
    off_cells = math.hypot(via.x - gx * coord.grid_step,
                           via.y - gy * coord.grid_step) / coord.grid_step

    def add_layer_cells(offs, layer_idx):
        cells = center + offs
        rows = np.empty((len(cells), 3), dtype=np.int32)
        rows[:, :2] = cells
        rows[:, 2] = layer_idx
        blocked_cells.append(rows)

    # Support per-layer expansion for impedance-controlled routing
    if isinstance(via_track_expansion_grid, list):
        for layer_idx in range(num_layers):
            layer_expansion = via_track_expansion_grid[layer_idx]
            radius = layer_expansion + diagonal_margin + off_cells
            add_layer_cells(circle_offsets(int(math.ceil(radius)), radius ** 2), layer_idx)
    else:
        radius = via_track_expansion_grid + diagonal_margin + off_cells
        offs = circle_offsets(int(math.ceil(radius)), radius ** 2)
        for layer_idx in range(num_layers):
            add_layer_cells(offs, layer_idx)

    via_radius = via_via_expansion_grid + off_cells
    via_offs = circle_offsets(int(math.ceil(via_radius)), via_radius * via_radius)
    blocked_vias.append(center + via_offs)


def _collect_pad_obstacles(pad, coord: GridCoord, layer_map: Dict[str, int],
                            config: GridRouteConfig, extra_clearance: float,
                            blocked_cells: List["np.ndarray"],
                            blocked_vias: List["np.ndarray"],
                            obs_clearance: float = None):
    """Collect pad obstacle cells into sets (no obstacle map modification).

    Uses rectangular-with-rounded-corners pattern matching other pad blocking functions.

    obs_clearance: the pad net's KiCad cross-class clearance (== config.clearance
    when no per-net map is active); priced by precompute_net_obstacles.
    """
    if obs_clearance is None:
        obs_clearance = config.clearance
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    # Sub-cell offset of the real pad center from its grid cell (issue #70).
    off_x = pad.global_x - gx * coord.grid_step
    off_y = pad.global_y - gy * coord.grid_step
    half_width = pad.size_x / 2
    half_height = pad.size_y / 2
    # Cross-class clearance (PR392): obs_clearance already carries this pad net's
    # KiCad pairwise clearance (max(routing-side floor, its class)). Per-pad
    # local/footprint clearance override (#326 B6): this per-net CACHE is what the
    # routing loop actually consults for same-run nets' pads, so it must honor
    # pad.local_clearance exactly like _add_pad_obstacle does -- otherwise a
    # to-be-routed net's fiducial/keep-clear pad is stamped at the bare clearance
    # and every sibling net may route inside its ring.
    clearance = obs_clearance
    lc = getattr(pad, 'local_clearance', 0.0) or 0.0

    # #498 per-layer .kicad_dru rules, mirroring _add_pad_obstacle: a layer
    # rule replaces the net/class fallback; the pad's local keep-clear stays a
    # hard floor on top. Layers sharing a resolved value share one
    # rasterization -- a board without rules takes the old single-margin path.
    def _layer_clr(layer_name):
        return max(config.layer_clearance(layer_name, clearance), lc)

    def _clr_groups(expanded):
        groups = {}
        for l in expanded:
            li = layer_map.get(l)
            if li is not None:
                groups.setdefault(_layer_clr(l), []).append(li)
        return groups

    def _via_clr(expanded):
        return max((_layer_clr(l) for l in expanded if l.endswith('.Cu')),
                   default=max(clearance, lc))

    if lc > clearance:
        clearance = lc

    # Custom comb/finger pads: rasterize the real copper polygon(s), leaving the
    # finger channels open, instead of the bounding box (issue #188). This is the
    # per-net obstacle CACHE the routing loop actually uses, so the fix must live
    # here as well as in _add_pad_obstacle.
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        from obstacle_map import _rasterize_polygon_box, _box_masked_cells
        expanded_layers = expand_pad_layers(pad.layers, config.layers)
        clr_groups = _clr_groups(expanded_layers)
        on_copper = any(l.endswith('.Cu') for l in expanded_layers)
        via_margin = config.via_size / 2 + _via_clr(expanded_layers) + config.grid_step / 2
        for poly in pad_polys:
            for g_clr, g_idxs in clr_groups.items():
                margin = config.track_width / 2 + g_clr + extra_clearance
                gx_lo, gy_lo, nx, ny, inside, edist = _rasterize_polygon_box(
                    poly, coord, margin)
                if inside is not None:
                    m = inside | (edist <= margin - GRID_TIE_EPS)
                    if m.any():
                        gxs, gys = _box_masked_cells(gx_lo, gy_lo, nx, m)
                        cells = np.column_stack([gxs, gys])
                        for li in g_idxs:
                            rows = np.empty((len(cells), 3), dtype=np.int32)
                            rows[:, :2] = cells
                            rows[:, 2] = li
                            blocked_cells.append(rows)
            if on_copper:
                vgx_lo, vgy_lo, vnx, vny, vin, ved = _rasterize_polygon_box(
                    poly, coord, via_margin)
                if vin is not None:
                    vm = vin | (ved <= via_margin - GRID_TIE_EPS)
                    if vm.any():
                        vgxs, vgys = _box_masked_cells(vgx_lo, vgy_lo, vnx, vm)
                        blocked_vias.append(np.column_stack([vgxs, vgys]))
        return

    # Corner radius based on pad shape (circle/oval use min dimension, roundrect uses rratio)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(half_width, half_height)
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0

    expanded_layers = expand_pad_layers(pad.layers, config.layers)

    # Batched rasterization (issue #35): same cell sets as the generator
    # (pad_blocked_cells_array is bit-identical to iter_pad_blocked_cells)
    for g_clr, g_idxs in _clr_groups(expanded_layers).items():
        cells = pad_blocked_cells_array(gx, gy, half_width, half_height,
                                        config.track_width / 2 + g_clr + extra_clearance,
                                        config.grid_step, corner_radius,
                                        off_x=off_x, off_y=off_y,
                                        rotation_deg=pad.rect_rotation)
        for layer_idx in g_idxs:
            rows = np.empty((len(cells), 3), dtype=np.int32)
            rows[:, :2] = cells
            rows[:, 2] = layer_idx
            blocked_cells.append(rows)

    # Via blocking around pads
    if any(layer.endswith('.Cu') for layer in expanded_layers):
        # Add half grid step buffer to account for grid quantization errors
        via_margin = config.via_size / 2 + _via_clr(expanded_layers) + config.grid_step / 2
        blocked_vias.append(pad_blocked_cells_array(gx, gy, half_width, half_height,
                                                    via_margin, config.grid_step, corner_radius,
                                                    off_x=off_x, off_y=off_y,
                                                    rotation_deg=pad.rect_rotation))


def precompute_all_net_obstacles(pcb_data: PCBData, net_ids: List[int], config: GridRouteConfig,
                                   extra_clearance: float = 0.0,
                                   diagonal_margin: float = defaults.DIAGONAL_MARGIN) -> Dict[int, NetObstacleData]:
    """Pre-compute obstacles for all nets to route.

    This is called once at the start of routing to build a cache that
    dramatically speeds up per-route obstacle building.

    Args:
        pcb_data: PCB data structure
        net_ids: List of net IDs to precompute obstacles for
        config: Routing configuration
        extra_clearance: Additional clearance (for diff pair routing)
        diagonal_margin: Extra margin for via blocking near diagonal segments

    Returns:
        Dict mapping net_id to NetObstacleData
    """
    cache: Dict[int, NetObstacleData] = {}
    for net_id in net_ids:
        cache[net_id] = precompute_net_obstacles(pcb_data, net_id, config,
                                                   extra_clearance, diagonal_margin)
    return cache


def add_net_obstacles_from_cache(obstacles: GridObstacleMap, cache_data: NetObstacleData):
    """Add a net's obstacles from cache using batch operations.

    This is much faster than the traditional per-segment/via/pad approach
    because it uses batch FFI calls instead of individual cell additions.

    Args:
        obstacles: The obstacle map to add to
        cache_data: Pre-computed NetObstacleData for the net
    """
    if _LEDGER is not None:
        _ledger_cache_op("add", obstacles, cache_data)
    if len(cache_data.blocked_cells) > 0:
        # Pass numpy array directly to Rust (no conversion needed)
        obstacles.add_blocked_cells_batch(cache_data.blocked_cells)
    if len(cache_data.blocked_vias) > 0:
        obstacles.add_blocked_vias_batch(cache_data.blocked_vias)
    # #815: segment keep-outs, expanded in Rust. Must be mirrored EXACTLY by
    # remove_net_obstacles_from_cache or cells leak blocked.
    _cs = getattr(cache_data, 'blocked_cell_spans', None)
    if _cs is not None and len(_cs) > 0:
        obstacles.add_blocked_cell_spans_batch(_cs)
    _vs = getattr(cache_data, 'blocked_via_spans', None)
    if _vs is not None and len(_vs) > 0:
        obstacles.add_blocked_via_spans_batch(_vs)
    _small = getattr(cache_data, 'blocked_vias_small', None)
    if _small is not None and len(_small) > 0:
        obstacles.add_blocked_vias_small_batch(_small)
    if _CELL_WATCH != []:
        ledger_cell_watch(obstacles, "cache-add " + _ledger_site(depth=2, frames=3))


def remove_net_obstacles_from_cache(obstacles: GridObstacleMap, cache_data: NetObstacleData):
    """Remove a net's obstacles from the map using batch operations.

    Used to temporarily remove a net's obstacles before routing it,
    so the router can use the net's own stubs/pads as endpoints.

    Args:
        obstacles: The obstacle map to remove from
        cache_data: Pre-computed NetObstacleData for the net
    """
    if _LEDGER is not None:
        _ledger_cache_op("remove", obstacles, cache_data)
    if len(cache_data.blocked_cells) > 0:
        obstacles.remove_blocked_cells_batch(cache_data.blocked_cells)
    if len(cache_data.blocked_vias) > 0:
        obstacles.remove_blocked_vias_batch(cache_data.blocked_vias)
    # #815: exact mirror of the add above -- same arrays, same order.
    _cs = getattr(cache_data, 'blocked_cell_spans', None)
    if _cs is not None and len(_cs) > 0:
        obstacles.remove_blocked_cell_spans_batch(_cs)
    _vs = getattr(cache_data, 'blocked_via_spans', None)
    if _vs is not None and len(_vs) > 0:
        obstacles.remove_blocked_via_spans_batch(_vs)
    _small = getattr(cache_data, 'blocked_vias_small', None)
    if _small is not None and len(_small) > 0:
        obstacles.remove_blocked_vias_small_batch(_small)
    if _CELL_WATCH != []:
        ledger_cell_watch(obstacles, "cache-remove " + _ledger_site(depth=2, frames=3))


def build_working_obstacle_map(base_obstacles: GridObstacleMap,
                                 net_obstacles_cache: Dict[int, NetObstacleData]) -> GridObstacleMap:
    """Build a working obstacle map with base + all cached net obstacles.

    This creates a complete obstacle map at startup that can be incrementally
    updated during routing (remove current net, add new route segments).

    Args:
        base_obstacles: Base obstacle map with static obstacles
        net_obstacles_cache: Pre-computed obstacles for all nets to route

    Returns:
        Working obstacle map ready for incremental updates
    """
    working = base_obstacles.clone_fresh()
    ledger_set_working_map(working)  # no-op unless KICAD_OBSTACLE_LEDGER=1 (#309)
    for net_id, cache_data in net_obstacles_cache.items():
        add_net_obstacles_from_cache(working, cache_data)
    return working


def update_net_obstacles_after_routing(pcb_data, net_id: int, result: Dict,
                                         config, net_obstacles_cache: Dict[int, NetObstacleData]):
    """Update the net obstacles cache after a successful route.

    Recomputes the net's obstacles to include the newly routed segments/vias.
    This should be called after add_route_to_pcb_data().

    Args:
        pcb_data: PCB data (already updated with new route)
        net_id: The net that was just routed
        result: Routing result dict with new_segments and new_vias
        config: Routing configuration
        net_obstacles_cache: Cache to update
    """
    # Recompute this net's obstacles (now includes the new route)
    net_obstacles_cache[net_id] = precompute_net_obstacles(
        pcb_data, net_id, config,
        extra_clearance=0.0, diagonal_margin=defaults.DIAGONAL_MARGIN
    )


# =============================================================================
# Via Placement Obstacle Cache
# =============================================================================
# Separate cache for via placement in route_planes.py. Uses different clearance
# calculations than routing (actual segment widths vs uniform track width).

@dataclass
class ViaPlacementObstacleData:
    """Cached obstacle data for via placement (used by route_planes.py).

    Stores blocked via positions and routing cells per layer for incremental
    updates when ripping up nets during plane via placement.
    """
    # Blocked via positions as numpy array of shape (N, 2) with columns [gx, gy]
    # May contain duplicates to match reference counting in GridObstacleMap
    blocked_vias: np.ndarray
    # Blocked routing cells per layer as dict: layer_name -> numpy array of shape (M, 2) [gx, gy]
    blocked_cells_by_layer: Dict[str, np.ndarray]


def precompute_via_placement_obstacles(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    all_copper_layers: List[str],
    exclude_net_id: Optional[int] = None,
) -> ViaPlacementObstacleData:
    """
    Pre-compute via placement obstacle positions for a single net.

    Used for incremental updates when ripping up nets - we can quickly remove
    this net's blocked positions instead of rebuilding the entire obstacle map.

    Unlike precompute_net_obstacles (for routing), this uses actual segment widths
    for clearance calculations rather than a uniform track width.

    Args:
        pcb_data: PCB data structure
        net_id: Net ID to compute obstacles for
        config: Routing configuration
        all_copper_layers: All copper layers for routing obstacle cells
        exclude_net_id: The plane net the target maps were built around
            (build_via_obstacle_map's exclude_net_id). #543: the builders price
            net_id's copper at max(run clearance, ITS netclass clearance)
            (config.obstacle_clearance, #434) under any .kicad_dru layer rules
            (#498) -- EXCEPT the exclude net's own vias, which the via-map
            builder prices at the plain run clearance. Pass it whenever
            net_id may BE the exclude net (SharedViaMaps stamping the plane
            net's own tap vias); rip/restore callers price foreign blocker
            nets and may omit it. All-Default boards with no dru rules are
            byte-identical either way.

    Returns:
        ViaPlacementObstacleData with blocked_vias and blocked_cells_by_layer
    """
    import math
    coord = GridCoord(config.grid_step)
    # build_via_obstacle_map / build_routing_obstacle_map add a half-cell
    # discretization cushion to the via-block margin; removal must match (#208).
    grid_cushion = config.grid_step / 2
    # #543: mirror the builders' cross-class pricing (see exclude_net_id above).
    _own = exclude_net_id is not None and net_id == exclude_net_id
    obs_clr = config.clearance if _own else config.obstacle_clearance(net_id)

    # Collect blocked positions as numpy chunks (one per segment/via stamp), then
    # concatenate. Duplicate counts are preserved - the obstacle maps are REFERENCE
    # COUNTED, so a net's removal must remove EXACTLY the cell multiset its add
    # stamped, or ripping it drives shared cells' counts to zero and wrongly
    # unblocks copper a present net still needs (issue #208). So these stamps mirror
    # the add side cell-for-cell: build_via_obstacle_map and build_routing_obstacle_map
    # both use the exact capsule (segment_blocked_cells_array) for segments -- NOT the
    # bresenham-line + circle this used to use, which stamped a disc at every line
    # cell and over-counted a near cell ~7x, so removal over-decremented by ~7x.
    via_chunks: List["np.ndarray"] = []
    cell_chunks: Dict[str, List["np.ndarray"]] = {layer: [] for layer in all_copper_layers}
    track_w_by_layer = {layer: config.get_track_width(layer) for layer in all_copper_layers}

    # Process this net's segments - they block via placement (all layers) and
    # routing (own layer).
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        # Via map: capsule at via_size/2 + seg/2 + clearance + cushion (== _add_segment_via_obstacle).
        # #543: the net's class clearance under the seg's layer rule (#434/#498),
        # exactly as build_via_obstacle_map prices this segment.
        via_margin = (config.via_size / 2 + seg.width / 2
                      + config.layer_clearance(seg.layer, obs_clr) + grid_cushion)
        via_chunks.append(segment_blocked_cells_array(
            seg.start_x, seg.start_y, seg.end_x, seg.end_y, via_margin, config.grid_step))
        # Routing map: capsule at the per-layer track width, NO cushion (== _add_segment_routing_obstacle).
        if seg.layer in cell_chunks:
            route_margin = (track_w_by_layer[seg.layer] / 2 + seg.width / 2
                            + config.layer_clearance(seg.layer, obs_clr))
            cell_chunks[seg.layer].append(segment_blocked_cells_array(
                seg.start_x, seg.start_y, seg.end_x, seg.end_y, route_margin, config.grid_step))

    # Process this net's vias - they block via placement and routing on all layers.
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        center = np.array([[gx, gy]], dtype=np.int32)  # (1, 2)
        # Via map: per-size via-via disc + cushion at the grid-snapped centre
        # (== build_via_obstacle_map's per-size circle, keyed on this via's size).
        # #543: stack-max class clearance for foreign vias, run clearance for the
        # exclude net's own -- exactly the builder's (size, clearance) grouping.
        vv_mm = (via.size / 2 + config.via_size / 2
                 + config.stack_clearance(obs_clr) + grid_cushion)
        vv_sq = (vv_mm / config.grid_step) ** 2
        via_chunks.append(center + circle_offsets(max(1, int(math.ceil(math.sqrt(vv_sq)))), vv_sq))
        # Via map: drill hole-to-hole keepout, stamped IN ADDITION to the via-via
        # disc by _add_drill_hole_via_obstacles (== block_via_cells_near_drills, a
        # float-centre disc, strict <). Removing it too keeps the via map exactly in
        # sync -- without it the ripped via leaves a stale drill keepout (#208).
        h2h = getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0
        if h2h > 0 and via.drill > 0:
            req = via.drill / 2.0 + config.via_drill / 2.0 + h2h
            # tie -> OPEN, mirroring block_via_cells_near_drills (see above)
            req_sq = (req - GRID_TIE_EPS) ** 2
            de = coord.to_grid_dist_safe(req) + 1  # ceil + 1-cell bbox margin (matches source)
            doff = np.arange(-de, de + 1)
            dxg, dyg = np.meshgrid(doff, doff, indexing="ij")
            cx = (gx + dxg) * config.grid_step
            cy = (gy + dyg) * config.grid_step
            dmask = ((cx - via.x) ** 2 + (cy - via.y) ** 2) < req_sq
            via_chunks.append(
                np.column_stack([(gx + dxg)[dmask], (gy + dyg)[dmask]]).astype(np.int32))
        # Routing map: float-centre real-distance disc + half-cell, per-layer track
        # width (== build_routing_obstacle_map's via loop). The via spans all layers.
        for layer in all_copper_layers:
            r_mm = (via.size / 2 + track_w_by_layer[layer] / 2
                    + config.layer_clearance(layer, obs_clr)  # #543 (#434/#498)
                    + config.grid_step / 2)
            rg = coord.to_grid_dist_safe(r_mm)
            r_sq = r_mm * r_mm
            off = np.arange(-rg, rg + 1)
            exg, eyg = np.meshgrid(off, off, indexing="ij")
            ddx = (gx + exg) * config.grid_step - via.x
            ddy = (gy + eyg) * config.grid_step - via.y
            mask = (ddx * ddx + ddy * ddy) <= r_sq
            cell_chunks[layer].append(
                np.column_stack([(gx + exg)[mask], (gy + eyg)[mask]]).astype(np.int32))

    # Concatenate chunks into the final arrays (keep duplicates for reference counting)
    if via_chunks:
        blocked_vias_arr = np.concatenate(via_chunks).astype(np.int32, copy=False)
    else:
        blocked_vias_arr = np.empty((0, 2), dtype=np.int32)

    blocked_cells_arrays = {}
    for layer, chunks in cell_chunks.items():
        if chunks:
            blocked_cells_arrays[layer] = np.concatenate(chunks).astype(np.int32, copy=False)
        else:
            blocked_cells_arrays[layer] = np.empty((0, 2), dtype=np.int32)

    return ViaPlacementObstacleData(
        blocked_vias=blocked_vias_arr,
        blocked_cells_by_layer=blocked_cells_arrays
    )

