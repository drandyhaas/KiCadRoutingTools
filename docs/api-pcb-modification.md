# PCB Modification & Geometry API (`pcb_modification`, `geometry_utils`)

- **`pcb_modification.py`** — keeps the in-memory `PCBData` model in sync
  with routing: add a routed result (with cleanup), remove a net (for rip-up),
  swap pad nets. If you add copper here, every subsequent routing/obstacle
  call sees it.
- **`geometry_utils.py`** — the shared 2-D geometry primitives: distances,
  intersections, closest points, path simplification, and a `UnionFind`.

All coordinates are mm.

## `pcb_modification.py`

### `add_route_to_pcb_data`

```python
add_route_to_pcb_data(pcb_data: PCBData, result: dict,
                      debug_lines: bool = False) -> None
```

Appends a routed result's copper to `pcb_data.segments` / `pcb_data.vias`,
**after cleaning it**. `result` is a dict with `'new_segments'` (list of
`Segment`) and `'new_vias'` (list of `Via`). Mutates `pcb_data` in place and
also replaces `result['new_segments']` with the cleaned list, so the same
result dict can go straight to the writer.

Cleanup pipeline (per net):

1. **Self-intersection fixing** — short connector segments that cross the
   net's existing copper are trimmed to the crossing point; orphaned
   downstream pieces are removed.
2. **Appendix collapsing** — short dead-end segments left by grid snapping
   are collapsed to micro-stubs at their junction.
3. **Degenerate segment removal** — segments under 0.01 mm are dropped;
   micro-bridges between two chains are welded shut (neighbors moved to the
   midpoint) so no gap opens.

Call this after each successful route and before routing the next net, so
incremental obstacle maps stay truthful.

### `remove_route_from_pcb_data`

```python
remove_route_from_pcb_data(pcb_data: PCBData, result: dict) -> None
```

Inverse of `add_route_to_pcb_data` for rip-up: removes exactly the segments
and vias recorded in `result` (matched by normalized endpoints rounded to
`POSITION_DECIMALS`, direction-agnostic).

### `remove_net_from_pcb_data` / `restore_net_to_pcb_data`

```python
remove_net_from_pcb_data(pcb_data, net_id: int)
    -> Tuple[List[Segment], List[Via]]
restore_net_to_pcb_data(pcb_data, segments, vias) -> None
```

Blunter tools: strip **all** copper of a net (returning it so you can
restore on failure), and put it back.

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

The cleanup stages are callable directly when you need them outside
`add_route_to_pcb_data`:

```python
get_copper_layers_from_segments(segments, existing_segments=None) -> List[str]
```

Returns a new list (input is not mutated).

> **Removed:** `collapse_appendices` and `fix_self_intersections` were deleted
> (issue #159). `fix_self_intersections` resolved a same-net self-crossing by
> *extending* a segment to a far off-grid endpoint, which created long
> non-orthonormal diagonals that crossed foreign copper. `collapse_appendices`
> was already a vestigial wrapper (its short-appendix trim was removed in #148,
> redundant with `sweep_dead_ends`). `add_route_to_pcb_data` now commits routed
> segments directly; connectivity cleanup is handled by `prune_redundant_cycles`
> and the whole-net dead-end trim `sweep_dead_ends` (#84). The residual same-net
> self-crossings are tracked in #162.

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
point_key(x, y, layer, tolerance=0.02) -> Tuple[int, int, str]
```

Quantizes a point to a hashable key (default 20 µm buckets) — for building
position dictionaries that tolerate float noise.

```python
simplify_path(path: List[Tuple[int, int, str]]) -> List[...]
```

Removes collinear intermediate points from an A* path. Note: operates on
**grid** coordinates `(gx, gy, layer)`, not mm.

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
- **`remove_route_from_pcb_data` must see the same coordinates it added** —
  if you transform geometry in between, use `remove_net_from_pcb_data`
  instead.
- **`simplify_path` is grid-space**; everything else in `geometry_utils` is
  mm.
