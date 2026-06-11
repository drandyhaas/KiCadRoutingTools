# KiCad Writer API (`kicad_writer`, `output_writer`)

Writing copper back to `.kicad_pcb` files. Two layers:

- **`kicad_writer.py`** — the building blocks: add tracks/vias from plain
  dicts, generate S-expressions, swap nets/layers of existing copper, zones,
  teardrops. Use this from your own scripts.
- **`output_writer.py`** — the routing pipeline's orchestrator
  (`write_routed_output`): applies target swaps, layer modifications, and
  polarity fixes in the right order, then inserts all routed copper. Use this
  when you have full routing results.

Both preserve the original file content and append/edit in place — nothing
else in the file is regenerated, so user edits, footprints, and zones
survive untouched. Every new element gets a fresh UUID.

## Contents

- [Adding tracks and vias](#adding-tracks-and-vias)
- [KiCad 9 vs KiCad 10](#kicad-9-vs-kicad-10)
- [S-expression generators](#s-expression-generators)
- [Modifying existing copper](#modifying-existing-copper) (net swaps, layer changes)
- [Text and teardrops](#text-and-teardrops)
- [`output_writer.py`](#output_writerpy)

## Adding tracks and vias

### `add_tracks_and_vias_to_pcb`

```python
add_tracks_and_vias_to_pcb(input_path: str, output_path: str,
                           tracks: List[Dict], vias: List[Dict] = None,
                           remove_vias: List[Dict] = None,
                           net_id_to_name: Dict[int, str] = None) -> bool
```

The main entry point for scripting. Reads `input_path`, appends the given
copper, writes `output_path`. Returns `True` on success.

Track dicts:

```python
{'start': (x, y), 'end': (x, y),   # mm
 'width': 0.25,                    # mm
 'layer': 'F.Cu',
 'net_id': 42}
```

Via dicts:

```python
{'x': 105.0, 'y': 100.0,           # mm
 'size': 0.6, 'drill': 0.3,        # mm
 'layers': ['F.Cu', 'B.Cu'],
 'net_id': 42,
 'free': False}                    # optional; True adds (free yes)
```

`remove_vias` takes dicts with `x`/`y` keys; matching vias are deleted from
the file before the new copper is added (position-based regex match — make
sure coordinates match the file's values).

There is also `add_tracks_to_pcb(input_path, output_path, tracks,
net_id_to_name=None)` for segments only.

### Example: drawing a box of tracks

```python
from kicad_parser import parse_kicad_pcb, KICAD_10_MIN_VERSION
from kicad_writer import add_tracks_and_vias_to_pcb

board = 'kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb'
pcb = parse_kicad_pcb(board)
gnd = next(n for n in pcb.nets.values() if n.name == 'GND')

corners = [(100, 100), (110, 100), (110, 110), (100, 110), (100, 100)]
tracks = [{'start': corners[i], 'end': corners[i + 1],
           'width': 0.3, 'layer': 'F.Cu', 'net_id': gnd.net_id}
          for i in range(4)]
vias = [{'x': 100, 'y': 100, 'size': 0.6, 'drill': 0.3,
         'layers': ['F.Cu', 'B.Cu'], 'net_id': gnd.net_id}]

names = pcb.net_id_to_name if pcb.kicad_version >= KICAD_10_MIN_VERSION else None
ok = add_tracks_and_vias_to_pcb(board, 'box_output.kicad_pcb',
                                tracks, vias, net_id_to_name=names)
print('wrote box_output.kicad_pcb' if ok else 'failed')
```

## KiCad 9 vs KiCad 10

KiCad 10 files reference nets by **name**, KiCad 9 by **numeric ID**. The
writer emits the right format based on whether you pass `net_id_to_name`:

| You pass | Output format |
|----------|---------------|
| `net_id_to_name=None` | KiCad 9: `(net 42)` |
| `net_id_to_name={42: 'GND', ...}` | KiCad 10: `(net "GND")`, plus tenting fields on vias |

The canonical pattern (used by the CLIs):

```python
from kicad_parser import KICAD_10_MIN_VERSION
names = pcb.net_id_to_name if pcb.kicad_version >= KICAD_10_MIN_VERSION else None
```

Passing the mapping for a KiCad 9 file (or `None` for a KiCad 10 file)
produces copper KiCad won't connect to the right nets — this is the most
common scripting mistake.

## S-expression generators

Lower-level functions that return formatted S-expression strings (with fresh
UUIDs, coordinates to 6 decimals). Use them if you're assembling file content
yourself:

```python
generate_segment_sexpr(start, end, width, layer, net_id, net_name=None) -> str
generate_via_sexpr(x, y, size, drill, layers, net_id,
                   free=False, net_name=None) -> str
generate_gr_line_sexpr(start, end, width, layer) -> str      # debug graphics
generate_gr_text_sexpr(text, x, y, layer, size=0.5, angle=0) -> str
generate_zone_sexpr(net_id, net_name, layer, polygon_points,
                    clearance=0.2, min_thickness=0.1,
                    thermal_gap=0.2, thermal_bridge_width=0.2,
                    direct_connect=True, use_net_name=False) -> str
```

If `net_name` is given (or `use_net_name=True` for zones), KiCad 10 format is
emitted; otherwise KiCad 9. `generate_zone_sexpr` is what
`route_planes.py` uses to create power/ground planes; `direct_connect=True`
makes solid pad connections instead of thermal reliefs.

```python
from kicad_writer import generate_segment_sexpr
print(generate_segment_sexpr((100.0, 100.0), (105.0, 100.0),
                             0.25, 'F.Cu', 42))
```

## Modifying existing copper

These operate on file **content strings** (read the file yourself, call,
write back). They're the machinery behind target swaps, polarity fixes, and
stub layer switching.

### `modify_segment_layers`

```python
modify_segment_layers(content: str, segment_mods: List[Dict])
    -> Tuple[str, int]   # (new_content, modified_count)
```

Each mod: `{'start': (x, y), 'end': (x, y), 'net_id': int,
'old_layer': str, 'new_layer': str}`. Matches segments by endpoints (either
order) and net, then rewrites the layer. Falls back to coordinate+old_layer
matching when net IDs have already been swapped.

### `swap_segment_nets_at_positions`

```python
swap_segment_nets_at_positions(content, positions: set,
                               old_net_id: int, new_net_id: int,
                               layer: str = None,
                               old_net_name=None, new_net_name=None)
    -> Tuple[str, int]
```

Reassigns every segment whose start **or** end is in `positions` (build the
set with `routing_utils.pos_key`) from one net to another. Always pass
`layer` when you can — without it, stubs on other layers sharing the same
XY get swapped too. Pass the `*_net_name` arguments for KiCad 10 files.

### `swap_via_nets_at_positions`

```python
swap_via_nets_at_positions(content, positions: set,
                           old_net_id: int, new_net_id: int,
                           tolerance: float = 0.02,
                           old_net_name=None, new_net_name=None)
    -> Tuple[str, int]
```

Same idea for vias, with a distance tolerance (mm) since via centers can be
slightly off segment endpoints.

### `swap_pad_nets_in_content`

```python
swap_pad_nets_in_content(content: str, pad1: Pad, pad2: Pad) -> str
```

Swaps the `(net ...)` declarations of two pads (located by
`component_ref` + `pad_number`). Used for diff-pair polarity fixes. The
in-memory counterpart is
[`pcb_modification.swap_pad_nets_in_pcb_data`](api-pcb-modification.md#swap_pad_nets_in_pcb_data) —
apply both if you keep routing afterwards.

## Text and teardrops

```python
move_copper_text_to_silkscreen(content: str) -> str
```

Moves `gr_text` from `F.Cu`/`B.Cu` to the matching silkscreen layer so text
doesn't collide with routed copper. `write_routed_output` and
`add_tracks_*` call this automatically.

```python
add_teardrops_to_pads(content: str, best_length_ratio=0.5, max_length=1.0,
                      best_width_ratio=1.0, max_width=2.0,
                      curved_edges=False, filter_ratio=0.9,
                      allow_two_segments=True,
                      prefer_zone_connections=True) -> Tuple[str, int]
```

Adds a `(teardrops ...)` settings block to every pad that lacks one; returns
the modified content and the count. Exposed on the CLIs as `--teardrops`.

## `output_writer.py`

### `write_routed_output`

```python
write_routed_output(input_file: str, output_file: str,
                    results: List[Dict],
                    all_segment_modifications: List,
                    all_swap_vias: List,
                    target_swap_info: List[Dict],
                    single_ended_target_swap_info: List[Dict],
                    pad_swaps: List[Dict],
                    pcb_data,
                    debug_lines: bool = False,
                    exclusion_zone_lines: List = None,
                    boundary_debug_labels: List = None,
                    skip_routing: bool = False,
                    add_teardrops: bool = False) -> bool
```

The single exit point of `route.py`/`route_diff.py`. Each entry in `results`
is one routed net's result dict; the keys that matter here are
`new_segments` (list of `Segment`-like objects) and `new_vias` (list of
`Via`-like objects). The remaining arguments carry layer modifications and
swap records accumulated during routing; pass empty lists when you have
none.

Steps, in order (the order matters — layer modifications are recorded with
post-swap net IDs, so swaps must hit the file first):

1. Move copper text to silkscreen
2. Add teardrops (optional)
3. Apply diff-pair target swaps (segments, vias, pads)
4. Apply single-ended target swaps
5. Apply stub layer modifications
6. Apply polarity pad/stub swaps
7. Insert all new segments and vias (plus debug geometry on User layers
   when `debug_lines=True`)

Minimal scripted use, with routed results in hand:

```python
from output_writer import write_routed_output

write_routed_output(
    'input.kicad_pcb', 'routed.kicad_pcb',
    results=results,                  # from the routing loops
    all_segment_modifications=[], all_swap_vias=[],
    target_swap_info=[], single_ended_target_swap_info=[],
    pad_swaps=[], pcb_data=pcb)
```

If you only need to add copper and have no swaps, prefer
[`add_tracks_and_vias_to_pcb`](#add_tracks_and_vias_to_pcb) — it takes plain
dicts.

Debug geometry layers (with `debug_lines=True`): raw A* path on `User.9`,
simplified path on `User.8`, connector segments on `User.3`, stub arrows on
`User.4`, exclusion zones on `User.5`, boundary labels on `User.6`.

## Gotchas

- **Pass `net_id_to_name` only for KiCad 10 files** (see
  [above](#kicad-9-vs-kicad-10)).
- **Position-based matching is exact-ish**: net/via swap helpers round
  through `pos_key` / tolerances, but coordinates must come from the same
  parse of the same file. Re-parse after each content rewrite if you chain
  operations that depend on positions.
- **These functions edit file text with regex/paren matching.** Hand-mangled
  or exotic files may not match; the functions then skip silently (counts in
  the return values tell you how many elements were actually touched —
  check them).
- **Writers don't validate clearances.** Run `check_drc.py` (see
  [Utilities](utilities.md)) on anything you generate.
