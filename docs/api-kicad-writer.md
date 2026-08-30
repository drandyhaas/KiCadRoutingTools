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

`net_id_to_name` maps the parser's synthetic net IDs to names so net
references are written name-style on KiCad 10 boards. If omitted on a
KiCad 10 board, the mapping is derived from the file automatically (the
synthetic IDs match any caller that got its `net_id`s from parsing that
file), so numeric refs are never mixed into a name-style board.

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
                   free=False, net_name=None, tenting_attrs=None,
                   inherit_when_unspecified=False) -> str
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

### Via protection (`tenting_attrs`)

`tenting_attrs` is a via's own protection spec -- `Via.tenting_attrs`, the
`(tenting ...)` / `(covering ...)` / `(plugging ...)` / `(capping ...)` /
`(filling ...)` family, see `docs/api-kicad-parser.md`. Which value you pass
is read together with `inherit_when_unspecified`, and the four combinations
are not interchangeable (#489 s8, #741):

| `tenting_attrs` | Emitted |
|---|---|
| a non-empty parsed spec | that spec, in canonical token order |
| `None` or `{}` | **nothing** -- the via inherits the board's `(setup ...)`, in either net dialect |

`inherit_when_unspecified` no longer changes the output: emitting nothing for an
empty spec is now the behaviour in every case. It is kept because callers pass
it and because it still records, at the call site, that a via ALREADY EXISTED.
The old KiCad-10 fallback to front+back tenting is gone -- see below.

### Why an unspecified via emits nothing

Probed against pcbnew 10.0.0: a via left at `TENTING_MODE_FROM_BOARD`
serialises with **no token**, and a token appears **only** when the via
explicitly overrides the board -- KiCad writes one even when the overriding
value equals the board default. So a token is a statement that this via is
special. Stamping one on a via the tool added converts an inheriting via into
an override, which is not a decision this tool was ever asked to make.

Two rules were retired to get here:

* **The hardcoded front+back tenting** on KiCad-10 output. Measured over 886
  corpus boards, three (`nanovoltmeter_marge`, `hexberry_fpga`, `pedal_404`)
  declare `(tenting (front no) (back no))` board-wide, and every via the tool
  added to them came back tented -- tenting a via the designer said to leave
  exposed, which is a fab error rather than a cosmetic one. It hid because
  KiCad's FACTORY policy is tented front+back, so on an ordinary board the
  stamp and the inheritance agree.
* **`prevailing_via_protection` as the added-via default** (the #489 s8 rule).
  Across those same 886 boards a prevailing spec never once disagreed with the
  board's own `(setup ...)`, so it only ever wrote a redundant token -- and the
  tool then read its OWN stamps back as "the board's convention" on the next
  run. orangecrab_ext_pll's 136 `(tenting …)` tokens are byte-identical to
  `generate_via_sexpr` output; hackrf_one's pristine source has **0/498** vias
  declaring anything, its 498 tokens being KiCad 10's upgrade migration writing
  out the factory defaults. The function remains correct and available; it is
  simply not a default.

`prevailing_via_protection(vias)` / `prevailing_via_protection_in_text(content)`
still answer "what do this board's vias actually say", which is the right
question if you ever need a new via to match its neighbours rather than the
board policy. No writer calls them today.

**Pass the spec back for any via that already existed.** A via you re-place
without it -- rip-up, sub-grid nudge, tap relocation -- is re-stamped with
front+back tenting, which is wrong for via-in-pad (it needs IPC-4761 Type VII:
filled + capped + plated).

**Pass `inherit_when_unspecified=True` for any via that already existed**, so
the re-placement does not *gain* an attribute the board never gave it. This is
what the GUI side has always done -- `gui_utils.apply_via_protection` returns
early on an empty spec, because pcbnew's `*_MODE_FROM_BOARD` already means
inherit.

**And keep the board's net dialect**, with `via_net_name(net_id,
net_id_to_name)` -- the ONE resolver, used by every emit site (#749 D).
`net_id_to_name` has no key `0` on any board (`extract_nets` records
`name_to_id[""] = 0` but never builds a `Net` for id 0), so a plain `.get`
sends every no-net via down the numeric dialect. On a name-net board that is
the mixed-dialect state `tests/stress/fix_mixed_net_refs.py` exists to undo.

It used to be worse than a dialect slip: `generate_via_sexpr` picks the dialect
from `net_name` and the protection tokens from `tenting_attrs` independently,
and `extract_vias`' numeric pattern had no gap for those tokens -- so a numeric
ref emitted alongside a spec produced a via the parser could not read back at
all, and the barrel VANISHED from the model. That is #748, fixed in the parser:
both dialects now tolerate the protection family in any position. The resolver
still matters, because emitting the board's own dialect is right regardless.

### Which sites pass what

| Site | Vias it emits | What it passes |
|---|---|---|
| `output_writer` | routed + swap vias | the via's own spec, else nothing |
| `add_tracks_and_vias_to_pcb` | depends on caller | the via dict's `tenting_attrs` / `inherit_when_unspecified`, else nothing (#749 A) |
| `route.py` #666 re-emit | PRE-EXISTING copper the write lost | own spec + `inherit_when_unspecified=True` |
| `bga_fanout`, `qfn_fanout`, `route_planes` | new | nothing -> inherits `(setup ...)` |
| `kicad_oracle` (3 weld/link sites) | new | nothing, spelled out as `None` (#749 B) |
| `plane_io.create_plane` / `repair_planes` | new | the via's own spec, else nothing |
| `plane_io.restore_failed_reroute_nets` | RESTORED | own spec + `inherit_when_unspecified=True` (#749 C) |
| `py_placer/placement/writer.py` | re-placed (the #313 nudge) | own spec + `inherit_when_unspecified=True` (#741) |

`tests/test_749_via_protection_emit_sites.py` walks the AST of every module in
that table and fails on a `generate_via_sexpr` call with no `tenting_attrs=`,
so a NEW emit site that forgets is caught rather than discovered on a board.

**GUI side**, `kicad_parser.pcbnew_via_protection_attrs(via, text_specs)` is the
reader, not `_pcbnew_via_protection_attrs`: the shipping KiCad 10.0.0 SWIG
wrapper does not export the `TENTING_MODE_*` family, so the live-object reader
answers `{}` for every via and the resolver falls back to the board file (#751).

**CLI and GUI now agree on new vias.** The GUI builds a `PCB_VIA` and calls
`apply_via_protection`, which returns early on an empty spec and leaves every
mode at `*_MODE_FROM_BOARD` -- inheriting the board's `(setup ...)`. The CLI
emits no token, which is the same thing in the file. There is no longer a
CLI-only default to drift.

For vias you **add**, pass nothing: no token means the via inherits the board's
`(setup ...)`, which is what KiCad does for a via a user places.

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
`add_tracks_*` call this automatically. A companion
`move_copper_graphics_to_silkscreen(content)` does the same for net-less copper
logos/artwork (graphic polys, lines, arcs). The GUI plugin mirrors both moves on
apply (via `kicad_routing_plugin/gui_utils.move_copper_graphics_to_silkscreen_board`)
so its output matches the CLI writer (issue #146).

```python
add_teardrops_to_pads(content: str, best_length_ratio=0.5, max_length=1.0,
                      best_width_ratio=1.0, max_width=2.0,
                      curved_edges=False, filter_ratio=0.9,
                      allow_two_segments=True,
                      prefer_zone_connections=True) -> Tuple[str, int]
```

Adds a `(teardrops ...)` settings block to every pad that lacks one; returns
the modified content and the count. Exposed on the CLIs as `--add-teardrops`.

```python
add_teardrops_to_vias(content: str, ...same parameters...) -> Tuple[str, int]
```

The same for **vias**, which got teardrops on no path at all before #489 §9 —
track-to-via teardrops matter most exactly where a 0.1 mm trace meets a 0.25 mm
via pad in a fine-pitch escape. Two details are load-bearing:

- The block is inserted **after** the via's `(uuid ...)`, as its last child.
  KiCad does not care about child order (verified by round-trip through pcbnew:
  0 vias reported `GetTeardropsEnabled()` before, all 63 after), while this
  repo's own via regexes require `layers → (free)? → net → (uuid)?` to be
  contiguous (uuid itself is optional since PR #534, but an earlier insertion
  would still split the required run) — inserting earlier would make the
  boards we write unparseable by us. A uuid-less via has no anchor and is
  skipped by this pass (benign: the tool's own output always writes uuids).
- Both writers run this pass **last**, after the run's new copper is inserted,
  so vias *this run placed* get teardrops too. Running it on the input text
  covered only pre-existing vias (63 of 81 on megadesk).

Idempotent: a via that already has a block is skipped.

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
`User.4`, BGA exclusion-zone rectangles plus stub/pad proximity circles on
`User.5`, boundary labels on `User.6`.

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
