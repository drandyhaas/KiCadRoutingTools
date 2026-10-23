# PCB Modification & Geometry API (`pcb_modification`, `geometry_utils`)

- **`pcb_modification.py`** — keeps the in-memory `PCBData` model in sync
  with routing: add a routed result (with cleanup), remove a net (for rip-up),
  swap pad nets. If you add copper here, every subsequent routing/obstacle
  call sees it.
- **`geometry_utils.py`** — the shared 2-D geometry primitives: distances,
  intersections, closest points, path simplification, a `UnionFind`, and
  which layers a via's copper is on.

All coordinates are mm.

Examples run from the repository root with the engine directory on the
import path: `PYTHONPATH=py_router python3 <your script>` (see the
[overview](python-api.md)).

## `pcb_modification.py`

### `add_route_to_pcb_data`

```python
add_route_to_pcb_data(pcb_data: PCBData, result: dict,
                      debug_lines: bool = False,
                      trace_event: str = 'route') -> None
```

Appends a routed result's copper to `pcb_data.segments` / `pcb_data.vias`,
**after cleaning it**. `result` is a dict with `'new_segments'` (list of
`Segment`) and `'new_vias'` (list of `Via`). Mutates `pcb_data` in place and
also replaces `result['new_segments']` with the cleaned list, so the same
result dict can go straight to the writer. `trace_event='restore'` marks a
re-commit of ripped copper, which must land exactly as it was.

Cleanup (per net, on this result's new segments only):

1. **Degenerate segment removal** — segments under 0.01 mm are dropped;
   micro-bridges between two chains are welded shut (neighbors moved to the
   midpoint) so no gap opens.
2. **Dead-end prune** (#84) — this route's own dead-end spurs are unwound, so
   dead copper does not block the routes that follow. The net's copper
   already on the board anchors junctions but is never removed here, and the
   prune is gated: it never leaves a pad less connected or splits a copper
   component (#1227, below).
3. **Exactly once** — an object already on the board (a restore handing back
   a saved result) is not appended again.

Call this after each successful route and before routing the next net, so
incremental obstacle maps stay truthful.

### `remove_route_from_pcb_data`

```python
remove_route_from_pcb_data(pcb_data: PCBData, result: dict) -> None
```

Inverse of `add_route_to_pcb_data` for rip-up: removes exactly the segment
and via OBJECTS recorded in `result`, by identity, so an input-file original
that a reroute retraced exactly survives (#389). An object not found by
identity (a caller holding a copy) falls back to a geometry match —
normalized endpoints at `POSITION_DECIMALS`, layer and net — removing at most
one item per object, newest first, and never a KiCad-locked one.

Both functions bump the board's copper epoch, which memos derived from the
copper key on. A pass that edits `pcb_data.segments` / `.vias` any other way
must call `bump_copper_epoch(pcb_data)` itself.

### `remove_net_from_pcb_data` / `restore_net_to_pcb_data`

```python
remove_net_from_pcb_data(pcb_data, net_id: int)
    -> Tuple[List[Segment], List[Via]]
restore_net_to_pcb_data(pcb_data, segments, vias) -> None
```

Blunter tools: strip **all** copper of a net (returning it so you can
restore on failure), and put it back. Graphic copper (`Segment.graphic`) is
left in place: the file cannot lose it, so neither does the model.

```python
from kicad_parser import parse_kicad_pcb
from pcb_modification import remove_net_from_pcb_data, restore_net_to_pcb_data

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
net = next(n for n in pcb.nets.values() if 'lvds' in n.name and len(n.pads) >= 2)

before = len(pcb.segments)
segs, vias = remove_net_from_pcb_data(pcb, net.net_id)
print(f"removed {len(segs)} segments, {len(vias)} vias of {net.name}")

restore_net_to_pcb_data(pcb, segs, vias)
assert len(pcb.segments) == before
```

### `swap_pad_nets_in_pcb_data`

```python
swap_pad_nets_in_pcb_data(pcb_data: PCBData, pad_a: Pad, pad_b: Pad) -> None
```

Swaps two pads' net assignments in the in-memory model (`net_id`,
`net_name`, `pads_by_net`, `Net.pads`). The file-level counterpart is
[`kicad_writer.swap_pad_nets_in_content`](api-kicad-writer.md#swap_pad_nets_in_content);
the routing pipeline applies both so the model and the output file agree.

### Cleanup helpers (advanced)

> **Removed:** `collapse_appendices` and `fix_self_intersections` were deleted
> (issue #159). `fix_self_intersections` resolved a same-net self-crossing by
> *extending* a segment to a far off-grid endpoint, which created long
> non-orthonormal diagonals that crossed foreign copper. `collapse_appendices`
> was already a vestigial wrapper (its short-appendix trim was removed in #148,
> redundant with `sweep_dead_ends`). `add_route_to_pcb_data` now commits routed
> segments with no self-intersection pass; connectivity cleanup is handled by `prune_redundant_cycles`
> and the whole-net dead-end trim `sweep_dead_ends` (#84). The residual same-net
> self-crossings are tracked in #162.

> **#1063 -- one removability predicate.** `collapse_strict_redundant(results,
> pcb_data, scope_net_ids, keep_input_copper=False, protect_segment_ids=None,
> stats=None)` deletes the copper `StrictRemovalModel` accepts, and
> `check_weird`'s `removable-segment` reports exactly what that model accepts,
> so the checker never calls copper removable that the cleanup keeps. The model
> grades a removal of segments (a single one, or an unbranched run) plus the
> vias it leaves joining nothing: every pad stays connected on the strict
> graph (track widths clamped; a via joins a pad only by its centre,
> `check_net_connectivity(..., via_in_pad_margin=...)`; on a layer a pad's
> unconnected-layer mode removes, only its hole joins,
> `check_net_connectivity(..., unflashed_hole_only=True)`) and on the physical
> graph, no copper island, dangling end or soft joint appears, and no via the
> pass may not drop -- anything but this run's own, unlocked vias -- is left
> dangling. In-pad / in-via wiggles (both ends in same-net pad or via copper),
> which #217 kept, are removed. A redundant loop through vias loses its via
> branch first. `run_post_route_cleanup(strict_collapse=True)` runs the
> collapse twice: after the cycle prune, and again after `close_soft_joints` on
> the nets smoothing or bridging changed, never removing the soft-joint bridges
> and web connectors close lays. **route.py passes `strict_collapse=False`**
> and collapses ONCE at the end of the outermost run
> (`route._late_strict_collapse1063`), after the plane finalize, the oracle
> legs and the reconciliation, on the board it ships (the written file on the
> CLI, the write model on the GUI); the finalize's in-process plane cleanups
> skip it too (`cleanup_pipeline.defer_strict_collapse`). Collapsing before
> those passes steered their rip/reroute (cparti_fpga: 6 open nets -> 15 from
> one input). Nets over `STRICT_REMOVAL_MAX_SEGS` (500)
> segments are skipped by both. `check_connected.analyze_conn_excluding` takes
> `excluded_via_indices` for this and now returns the `num_copper_components`
> it always computed.

> **#1166 -- dangling vias.** `sweep_dangling_via_branches(board, net_ids,
> protected_ids=frozenset(), max_rounds=32, stats=None)` removes every via
> `check_weird` calls `dangling-via` (reached on one layer, KiCad's
> `via_dangling`) or `unsupported-via` on `net_ids`, graded by the checker's own
> `via_support_parts`, together with the chain its one supported layer carries
> away: through every plain vertex (two segment ends, no via, pad or zone
> there) up to the first junction, tee, pad, via, zone or free end. A via with
> several segments on its one layer goes alone. Each removal is gated on the
> net: no pad less connected, no dangling end where there was none, no more
> dangling vias, no new soft joint. A via a removal exposes is the next
> round's candidate, so a via-to-via dead branch goes whole. A net with a pad
> still disconnected is skipped (its copper is the next step's landing site,
> #473). Pads, locked
> copper and `protected_ids` are never removed. The dead-end passes cannot do
> this: they count any same-net via as an anchor. route.py runs it at the end
> of the outermost run, after the strict collapse
> (`route._late_dangling_via_sweep1166`, both fronts, input copper included
> unless `--keep-input-copper`), and on the "nothing to route" return too.

> **#1261 -- KiCad-locked copper is read-only to every cleanup pass.** A
> `(locked yes)` segment or via, or copper a `sense:` group holds (#1279) --
> `pcb_modification._locked`, which is `kicad_parser.is_immutable_copper` --
> is treated by `sweep_dead_ends`, `remove_orphan_islands`,
> `prune_redundant_cycles`, `trim_dangles_past_body_anchor`,
> `prune_grazing_segments`, `nudge_grazing_octolinear`,
> `nudge_grazing_microshift`, `merge_collinear_segments`,
> `trim_net_stub_debris`, the strict collapse, the dangling-via sweep and the
> castellated retract exactly as `keep_input_copper` treats input copper -- it
> anchors their models but is never removed, split, re-bent or moved --
> whether or not that flag is set.
> An island holding locked copper is kept whole, a micro-shift never moves a
> vertex a locked segment touches, and the #284 stale-input strip and
> `remove_route_from_pcb_data`'s signature fallback never take a locked item.

> **The graze prune keeps a fill island's only link.** `prune_grazing_segments`
> grades each removal on a zoned net twice: under the real-fill validator
> (`check_connected.make_real_fill_validator`, no pour credit within 0.25 mm
> of foreign copper) and under the grading model (the same
> `check_net_connectivity` without it, which `check_connected` and route.py's
> #209 gate read); the removal must leave both no worse. The validator's
> denial is conservative for the pad it would over-credit and the opposite
> for the copper being removed: a tap ending in a narrow fill island (an F.Cu
> pocket between a BGA's fanout tracks) loses its end's credit, reads as a
> dead end, the island's pad reads as already open, and the removal looks
> free. A 0.5 mm csBGA's GND ball lost its 0.33 mm tail that way in
> every route step. `tests/test_graze_prune_fill_island.py` pins it.

> **#1227 -- a dead-end prune never splits copper.** Every gated dead-end pass
> (`add_route_to_pcb_data`'s per-commit prune, `sweep_dead_ends`,
> `trim_net_stub_debris`, `cleanup_plane_taps_grazing`) removes through
> `_safe_prune_net`, which now also refuses a removal
> that breaks one of the net's strict copper components into pieces. A dead end
> hangs off the copper at one end, so a real one never splits anything. The
> pad-count gates could not see this once the strict graph was already split: a
> route committed with a 0.022 mm hole of its own unwound from the hole back to
> its pad, and the fine-track steps behind the rejected bridge each passed on
> their neighbours' cap overlap (18 segments committed, 7 kept,
> `segment-endpoint-gap` on every stump). The test compares each trial against
> the state it is tried on, not the base: removing an island lowers the
> component count, and that must not pay for a split.
> `analyze_conn_excluding(..., return_uf=True)` returns the union-find it uses.

> **#672 -- the two ends of ONE segment are not a soft joint.** Every
> soft-joint detector (`_soft_joint_pairs`, `close_soft_joints`, `check_drc`,
> `check_weird`) skips the pair: a lone sub-cap sliver used to pair with
> itself, which made `_restore_soft_joint_bridges` put a just-removed dead
> neighbour back. `sweep_dead_ends` keeps every piece of a net in
> `protect_net_ids` (nets with unfinished pads keep every landing site, #473).

### File-level post-passes

```python
compute_plane_copper_cleanup(pcb, plane_net_names, clearance=0.1,
                             grid_step=0.05, progress_callback=None)
    -> PlaneCleanupDelta      # connectors, segments_to_remove, vias_to_remove, snapped
clean_plane_copper(output_file, plane_net_names, clearance=0.1,
                   grid_step=0.05) -> Tuple[int, int]   # (snapped, removed)
compute_castellated_landing_retract(pcb, board_edge_clearance, margin=0.02)
    -> CastellationRetractDelta   # moves: [(segment, 'start'|'end', new_x, new_y)]
retract_castellated_landings(output_file, board_edge_clearance) -> int
```

Each pass is a board-level core that returns a DELTA, plus a file front that
parses a written board, applies the delta and rewrites the file. The GUI
applies the same delta to the live pcbnew board, so the two fronts cannot
drift. The plane cleanup runs the shared post-route cleanup pipeline (gap
snap, graze prune, re-bend, micro-shift, cycle prune, dead-end sweep,
soft-joint bridge) on the plane nets' copper. The castellated retract pulls a
track end that lands on a castellated pad (`Pad.castellated`) back out of the
board-edge clearance zone to the pad's inner reach.

## `geometry_utils.py`

Pure functions; no PCB context needed.

### Distances and closest points

```python
point_to_segment_distance(px, py, x1, y1, x2, y2) -> float
point_to_segment_distance_seg(px, py, seg: Segment) -> float
closest_point_on_segment(px, py, x1, y1, x2, y2) -> Tuple[float, float]
segment_to_segment_distance(x1, y1, x2, y2, x3, y3, x4, y4) -> float
segment_to_segment_distance_seg(seg1, seg2) -> float
segment_to_segment_closest_points(seg1, seg2)
    -> Tuple[float, Tuple[float, float], Tuple[float, float]]
    # (distance, point_on_seg1, point_on_seg2)
```

Degenerate (zero-length) segments are handled; intersecting segments have
distance 0.

### Intersection tests

```python
segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4) -> bool   # strict crossing
segments_intersect_tuple(p1, p2, p3, p4) -> bool             # tuple endpoints
segments_intersect_2d(s1_start, s1_end, s2_start, s2_end,
                      tolerance=0.001) -> bool   # also catches collinear overlap
```

`segments_intersect` detects proper interior crossings only (shared
endpoints don't count); `segments_intersect_2d` additionally reports
endpoint touches and collinear overlaps within `tolerance`.

### `UnionFind`

```python
from geometry_utils import UnionFind

uf = UnionFind()
uf.union('a', 'b')        # merge the sets containing 'a' and 'b'
uf.find('a')              # canonical representative (auto-creates singletons)
uf.connected('a', 'b')    # same set? -> True
```

Disjoint-set with path compression and union by rank; keys are any hashable
values. This is what connectivity grouping is built on.

### Other helpers

```python
simplify_path(path: List[Tuple[int, int, str]]) -> List[...]
```

Removes collinear intermediate points from an A* path. Note: operates on
**grid** coordinates `(gx, gy, layer)`, not mm.

### Which layers a via has copper on

```python
via_on_layer(via, layer_name) -> bool
via_copper_layers(via, copper_layers) -> List[str]
via_end_layers(via) -> Tuple[str, str]
via_span_layer_indices(via, routing_layers, num_layers)
PARTIAL_VIA_TYPES = frozenset({'blind', 'buried', 'micro'})
```

A via's stored `layers` pair is not its span. KiCad loads a via with no type
word as a THROUGH via whatever pair it carries, so only a via whose
`Via.via_type` is blind, buried or micro covers part of the stack: the layers
between its two end layers, in stack order. Ask these (they take a `Via` or a
via dict) instead of reading the pair; `via_end_layers` is what a
barrel-length measurement spans (`('F.Cu', 'B.Cu')` for a through via).

```python
from kicad_parser import Via
from geometry_utils import via_copper_layers

layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
micro = Via(0, 0, 0.3, 0.1, ['F.Cu', 'In1.Cu'], net_id=1, via_type='micro')
untyped = Via(0, 0, 0.3, 0.1, ['F.Cu', 'In1.Cu'], net_id=1)
print(via_copper_layers(micro, layers))     # ['F.Cu', 'In1.Cu']
print(via_copper_layers(untyped, layers))   # all four: a through via
```

### Example

```python
from geometry_utils import (point_to_segment_distance, closest_point_on_segment,
                            segments_intersect, UnionFind)

# How far is the point (5, 5) from the track (0,0)-(10,0)?
print(point_to_segment_distance(5, 5, 0, 0, 10, 0))     # 5.0
print(closest_point_on_segment(5, 5, 0, 0, 10, 0))      # (5.0, 0.0)

# Do two tracks cross?
print(segments_intersect(0, 0, 10, 10, 0, 10, 10, 0))   # True
print(segments_intersect(0, 0, 10, 0, 0, 1, 10, 1))     # False (parallel)

# Group endpoints into connected nets
uf = UnionFind()
uf.union('A', 'B'); uf.union('B', 'C'); uf.union('X', 'Y')
print(uf.connected('A', 'C'), uf.connected('A', 'X'))   # True False
```

## Gotchas

- **`add_route_to_pcb_data` is mandatory in routing loops** — skipping it
  means later nets route through copper they can't see, and the writer gets
  uncleaned geometry.
- **Per-net cleanup**: the dead-end / cycle cleanup helpers (`prune_redundant_cycles`,
  `sweep_dead_ends`) operate on **one net at a time** — never feed segments from
  multiple nets into one call.
- **`remove_route_from_pcb_data` removes the objects it is given** — pass the
  result whose objects `add_route_to_pcb_data` committed. Copies only match
  by geometry, so if you transform or copy geometry in between, use
  `remove_net_from_pcb_data` instead.
- **`simplify_path` is grid-space**; everything else in `geometry_utils` is
  mm.
