# KiCad Parser API (`kicad_parser`)

Parses `.kicad_pcb` files (KiCad 9 and 10) into plain Python dataclasses:
pads, nets, tracks, vias, zones, footprints, board outline, stackup, and
keepouts. No KiCad installation is required — the parser reads the
S-expression file format directly.

```python
from kicad_parser import parse_kicad_pcb

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')
```

All coordinates and sizes are in millimeters. See
[Python API Overview](python-api.md#conventions) for shared conventions.

## Contents

- [Main entry points](#main-entry-points)
- [PCBData](#pcbdata) and its [dataclasses](#dataclasses)
- [Net filtering](#net-filtering-get_nets_to_route)
- [Package detection and BGA helpers](#package-detection-and-bga-helpers)
- [Version handling (KiCad 9 vs 10)](#kicad-version-handling)
- [Lower-level extraction functions](#lower-level-extraction-functions)
- [Gotchas](#gotchas)

## Main entry points

### `parse_kicad_pcb`

```python
parse_kicad_pcb(filepath: str,
                guide_layer: str = "User.1",
                keepout_layer: str = "User.2") -> PCBData
```

Parse a `.kicad_pcb` file and return a fully populated [`PCBData`](#pcbdata).

- `guide_layer` — user layer scanned for drawn guide-corridor polylines
  (`gr_line` chains / `gr_poly`), exposed as `pcb.guide_paths`
- `keepout_layer` — user layer scanned for drawn keepout polygons, exposed as
  `pcb.keepout_zones`
- Raises `FileNotFoundError` if the file doesn't exist
- Detects KiCad 9 vs 10 automatically (see
  [version handling](#kicad-version-handling))

### `build_pcb_data_from_board`

```python
build_pcb_data_from_board(board,
                          guide_layer: str = "User.1",
                          keepout_layer: str = "User.2") -> PCBData
```

Build the same `PCBData` structure directly from a live `pcbnew.BOARD`
object (inside KiCad's Python console or an action plugin). Much faster than
saving and re-parsing, and reflects unsaved edits.

```python
# Inside KiCad's scripting console:
import pcbnew
from kicad_parser import build_pcb_data_from_board
pcb = build_pcb_data_from_board(pcbnew.GetBoard())
```

## PCBData

The top-level container returned by both entry points.

| Field | Type | Meaning |
|-------|------|---------|
| `board_info` | `BoardInfo` | Layers, bounds, outline, stackup, keepouts |
| `nets` | `Dict[int, Net]` | Nets keyed by net ID (ID 0 = unconnected) |
| `footprints` | `Dict[str, Footprint]` | Footprints keyed by reference (`'U9'`, `'R1'`) |
| `segments` | `List[Segment]` | All track segments |
| `vias` | `List[Via]` | All vias |
| `pads_by_net` | `Dict[int, List[Pad]]` | Pads grouped by net ID (fast lookup) |
| `zones` | `List[Zone]` | Filled copper zones (planes) |
| `guide_paths` | `List[GuidePath]` | User-drawn guide polylines from `guide_layer` |
| `keepout_zones` | `List[GuidePath]` | User-drawn keepout polygons from `keepout_layer` |
| `kicad_version` | `int` | File format version (e.g. `20241229` = KiCad 9) |
| `net_id_to_name` | `Dict[int, str]` | Net ID → name map (needed when writing KiCad 10 files) |

### `PCBData.get_via_barrel_length`

```python
pcb.get_via_barrel_length(layer1: str, layer2: str) -> float
```

Distance in mm through the stackup between two copper layers (including both
copper layers' own thickness). Returns `0.0` if the board has no stackup
section. Used for accurate length/time matching.

## Dataclasses

### `Pad`

| Field | Type | Meaning |
|-------|------|---------|
| `component_ref` | str | Parent footprint reference (`'U9'`) |
| `pad_number` | str | Pad identifier (`'1'`, `'H2'`; may be empty) |
| `global_x`, `global_y` | float | Absolute board position (footprint rotation applied) |
| `local_x`, `local_y` | float | Position relative to footprint origin (before rotation) |
| `size_x`, `size_y` | float | Pad dimensions in board space |
| `shape` | str | `'circle'`, `'rect'`, `'roundrect'`, `'oval'`, `'trapezoid'`, `'custom'` |
| `layers` | List[str] | Layers the pad is on (`['*.Cu']` = all copper, for through-hole) |
| `net_id` | int | Net ID (0 = unconnected) |
| `net_name` | str | Net name (`''` if unconnected) |
| `rotation` | float | Total rotation in degrees (pad + footprint), normalized to [0, 360) |
| `drill` | float | Drill diameter (0 for SMD, > 0 for through-hole). Oval/slot drills (`(drill oval w h)`) report `max(w, h)` so they keep `> 0` / through-hole semantics. |
| `pinfunction` | str | Pin function from the schematic (`'TX'`, `'~RESET'`) |
| `pintype` | str | Pin electrical type (`'passive'`, `'input'`, `'power_in'`) |
| `pad_type` | str | Pad kind: `'smd'`, `'thru_hole'`, `'np_thru_hole'`, `'connect'`. NPTH pads carry **no copper** (their size is just the mask opening, even when `layers` lists `*.Cu`), so copper-clearance checks skip them; only their drill hole matters. |
| `roundrect_rratio` | float | Corner radius ratio for roundrect pads (0–0.5) |
| `rect_rotation` | float | Residual rectangle tilt in the global frame, in (-90, 90]. `0` for axis-aligned pads (the common case); non-zero only for pads on non-orthogonal angles. |

Pad dimensions are resolved into board space using the pad's **absolute** angle
(the KiCad `(at … angle)` already includes the footprint rotation; applying the
footprint rotation a second time would make adjacent pad copper overlap). For
near-orthogonal pads (0/90/180/270°) `size_x`/`size_y` are swapped as needed so
they are board-axis-aligned and `rect_rotation` is `0`. A genuinely diagonal pad
keeps its given dimensions and carries the tilt in `rect_rotation`; the obstacle
map and DRC rotate the pad rectangle by that angle. `check_pads.py` flags pads
whose resolved copper overlaps a different-net neighbour (a modelling error).

### `Segment`

| Field | Type | Meaning |
|-------|------|---------|
| `start_x`, `start_y` | float | Start point |
| `end_x`, `end_y` | float | End point |
| `width` | float | Track width |
| `layer` | str | Layer name |
| `net_id` | int | Net ID |
| `uuid` | str | UUID from the file (`''` for newly created segments) |
| `start_x_str`, … | str | Original coordinate strings, kept for exact file matching |

### `Via`

| Field | Type | Meaning |
|-------|------|---------|
| `x`, `y` | float | Position |
| `size` | float | Outer (pad) diameter |
| `drill` | float | Hole diameter |
| `layers` | List[str] | Layer span, e.g. `['F.Cu', 'B.Cu']` |
| `net_id` | int | Net ID |
| `uuid` | str | UUID from the file |
| `free` | bool | `(free yes)` flag — KiCad won't reassign the net from overlapping copper |

### `Net`

| Field | Type | Meaning |
|-------|------|---------|
| `net_id` | int | Net ID (0 = unconnected) |
| `name` | str | Net name (may be `''`) |
| `pads` | List[Pad] | All pads on this net |

### `Footprint`

| Field | Type | Meaning |
|-------|------|---------|
| `reference` | str | Reference designator — also the key in `pcb.footprints` |
| `footprint_name` | str | Library ID (`'interf_u:PGA120'`) |
| `x`, `y` | float | Footprint origin |
| `rotation` | float | Rotation in degrees |
| `layer` | str | `'F.Cu'` or `'B.Cu'` |
| `pads` | List[Pad] | The footprint's pads |
| `value` | str | Component value (`'100nF'`, `'MCF5213'`) |

### `Zone`

| Field | Type | Meaning |
|-------|------|---------|
| `net_id`, `net_name` | int, str | Zone's net |
| `layer` | str | Layer name |
| `polygon` | List[Tuple[float, float]] | Outline vertices |
| `uuid` | str | UUID from the file |

### `BoardInfo`

| Field | Type | Meaning |
|-------|------|---------|
| `layers` | Dict[int, str] | Layer ID → name for all layers |
| `copper_layers` | List[str] | Copper layer names, top to bottom. Includes every `.Cu` layer regardless of declared use-type (`signal`, `power`, `mixed`, `jumper`) — KiCad often types plane layers `power`. |
| `board_bounds` | Optional[Tuple] | `(min_x, min_y, max_x, max_y)` from Edge.Cuts (board-level graphics AND footprint-embedded outline shapes), or `None` |
| `board_outline` | List[Tuple[float, float]] | Outline polygon for non-rectangular boards (empty if rectangular) |
| `board_cutouts` | List[List[Tuple]] | Interior cutout polygons |
| `stackup` | List[StackupLayer] | Physical stackup, top to bottom (empty if the board has none) |
| `keepouts` | List[dict] | KiCad keepout rule areas: `{'polygon': [...], 'layers': set, 'tracks_allowed': bool, 'vias_allowed': bool}` |

### `StackupLayer`

| Field | Type | Meaning |
|-------|------|---------|
| `name` | str | `'F.Cu'`, `'Core'`, `'Prepreg'`, … |
| `layer_type` | str | `'copper'`, `'core'`, `'prepreg'`, `'dielectric'` |
| `thickness` | float | Thickness in mm |
| `epsilon_r` | float | Relative permittivity (dielectric layers) |
| `loss_tangent` | float | Dissipation factor (dielectric layers) |
| `material` | str | Material name (`'FR4'`) |

### `GuidePath`

| Field | Type | Meaning |
|-------|------|---------|
| `layer` | str | User layer the path was drawn on |
| `points` | List[Tuple[float, float]] | Ordered vertices (≥ 2) |
| `is_closed` | bool | `True` for polygons (`gr_poly`), `False` for polylines |

### Example: inspecting a board

```python
from kicad_parser import parse_kicad_pcb

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')

# Largest nets by pad count
biggest = sorted(pcb.nets.values(), key=lambda n: len(n.pads), reverse=True)[:5]
for net in biggest:
    print(f"net {net.net_id:4d}  {net.name:20s} {len(net.pads)} pads")

# Every pad of one footprint
ref, fp = next(iter(pcb.footprints.items()))
print(f"\n{ref} ({fp.footprint_name}) at ({fp.x:.2f}, {fp.y:.2f}):")
for pad in fp.pads[:5]:
    th = 'TH' if pad.drill > 0 else 'SMD'
    print(f"  pad {pad.pad_number:3s} {th}  ({pad.global_x:.2f}, {pad.global_y:.2f})"
          f"  net={pad.net_name or '(none)'}")

# Stackup (empty list if the board has no stackup section)
for sl in pcb.board_info.stackup:
    print(f"{sl.name:12s} {sl.layer_type:8s} {sl.thickness:.3f}mm"
          + (f"  Er={sl.epsilon_r}" if sl.epsilon_r else ""))
```

## Net filtering: `get_nets_to_route`

```python
get_nets_to_route(pcb_data: PCBData,
                  net_patterns: Optional[List[str]] = None,
                  exclude_patterns: Optional[List[str]] = None,
                  component_ref: Optional[str] = None) -> List[Net]
```

Returns routable nets (≥ 2 pads), filtered by case-insensitive wildcard
patterns. `net_patterns=None` means all nets; the default exclusions are
`['*GND*', '*VCC*', '*VDD*', '*unconnected*', '*NC*', '']`. With
`component_ref`, only nets touching that component are returned.

```python
from kicad_parser import parse_kicad_pcb, get_nets_to_route

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')
nets = get_nets_to_route(pcb, net_patterns=['*D[0-9]*'], component_ref='U301')
print(f"{len(nets)} routable data nets on U301")
```

For richer pattern handling (explicit `!` exclusions, expansion to names) see
[`net_queries.expand_net_patterns`](api-net-analysis.md#expand_net_patterns).

## Package detection and BGA helpers

```python
detect_package_type(footprint: Footprint) -> str
    # 'BGA' | 'QFN' | 'QFP' | 'SOIC' | 'DIP' | 'OTHER'

find_components_by_type(pcb_data: PCBData, package_type: str) -> List[Footprint]

get_footprint_bounds(footprint: Footprint, margin: float = 0.0)
    -> Tuple[float, float, float, float]   # (min_x, min_y, max_x, max_y)

detect_bga_pitch(footprint: Footprint) -> float   # mm; 1.0 if undetectable

auto_detect_bga_exclusion_zones(pcb_data: PCBData, margin: float = 0.0)
    -> List[Tuple[float, float, float, float, float]]
    # (min_x, min_y, max_x, max_y, edge_tolerance) per BGA
```

Classification uses the footprint name first, then pad arrangement (grid vs
perimeter) and pad shapes. Land-grid and chip-scale families are classified as
`BGA` by name so they get fanout + BGA exclusion zones: `LGA` (land grid array,
e.g. LGA-12), `CSP` / `WLCSP` / `WLP` (wafer-level / chip-scale = micro-BGA),
and `CGA` (column grid array), alongside `BGA` / `FBGA` / `LFBGA` (issue #144).
The geometry path only declares `BGA` for a genuine
ball/pin matrix: >=16 pads, near-uniform pad size, and >=3 rows *and* >=3
columns each holding several pads. That last test rejects wide-pitch
through-hole headers (e.g. a 2-row `RPi_Pico_SMD_TH`) and connector arrays,
which are otherwise uniform-size but only one or two columns wide (issue #82).
The exclusion zones feed `GridRouteConfig.bga_exclusion_zones` to keep vias out
from under BGAs.

```python
from kicad_parser import (parse_kicad_pcb, find_components_by_type,
                          detect_bga_pitch, auto_detect_bga_exclusion_zones)

pcb = parse_kicad_pcb('kicad_files/fanout_starting_point.kicad_pcb')
for fp in find_components_by_type(pcb, 'BGA'):
    print(f"{fp.reference}: pitch {detect_bga_pitch(fp)}mm, {len(fp.pads)} pads")
for zone in auto_detect_bga_exclusion_zones(pcb):
    print(zone)
```

## KiCad version handling

```python
detect_kicad_version(content: str) -> int   # e.g. 20241229; 0 if missing
is_kicad_10(content: str) -> bool           # version >= KICAD_10_MIN_VERSION
KICAD_10_MIN_VERSION = 20250000
```

KiCad 10 removed numeric net IDs from the file format (`(net "name")` instead
of `(net 29 "name")`). The parser hides this: it assigns synthetic integer
IDs so `net_id`-based code works on both formats, and records the mapping in
`pcb.net_id_to_name`. **When writing copper back to a KiCad 10 file you must
pass that mapping** so the writer emits name-based nets — see
[KiCad Writer](api-kicad-writer.md#kicad-9-vs-kicad-10).

```python
from kicad_parser import parse_kicad_pcb, KICAD_10_MIN_VERSION

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')
v10 = pcb.kicad_version >= KICAD_10_MIN_VERSION
print(f"File version {pcb.kicad_version} -> {'KiCad 10+' if v10 else 'KiCad 9'} format")
```

## Coordinate transformation

```python
local_to_global(fp_x, fp_y, fp_rotation_deg,
                pad_local_x, pad_local_y) -> Tuple[float, float]
```

Converts footprint-local pad coordinates to absolute board coordinates.
KiCad's rotation convention means the angle is **negated** inside the
standard rotation matrix — use this helper rather than rolling your own.

## Utility functions

```python
compare_pcb_data(from_board: PCBData, from_file: PCBData,
                 tolerance: float = 0.01) -> List[str]
```

Compares two parses (e.g. live-board vs file) and returns human-readable
difference strings; empty list means they match. Useful to validate
`build_pcb_data_from_board` against `parse_kicad_pcb` (this is what the GUI's
**Validate PCB Data** button runs). Coverage spans the fields the router
actually consumes: board info (layers, bounds, stackup), nets, footprint and
per-pad geometry (position, size, `rect_rotation`, `roundrect_rratio`,
`local_clearance`, drill, layers), track and via geometry (matched as a
multiset, not just counts), zones, board outline/cutouts, keepout zones, and
guide paths.

```python
save_extracted_data(pcb_data: PCBData, output_path: str)
```

Serializes the parsed structure to JSON (for inspection or external tools).

## Lower-level extraction functions

`parse_kicad_pcb` is a thin orchestrator over per-section extractors that
each take the raw file text. Use these when you only need one section of a
large file:

| Function | Returns |
|----------|---------|
| `extract_layers(content)` | `BoardInfo` (layers, bounds, stackup, outline, cutouts) |
| `extract_stackup(content)` | `List[StackupLayer]` |
| `extract_nets(content, kicad_version=0)` | `(Dict[int, Net], Dict[str, int])` — nets and name→id |
| `extract_footprints_and_pads(content, nets, name_to_id=None)` | `(Dict[str, Footprint], Dict[int, List[Pad]])` |
| `extract_segments(content, name_to_id=None)` | `List[Segment]` |
| `extract_vias(content, name_to_id=None)` | `List[Via]` |
| `extract_zones(content, name_to_id=None)` | `List[Zone]` |
| `extract_keepouts(content)` | `List[dict]` (keepout rule areas) |
| `extract_board_outline(content)` | Outline polygon vertices |
| `extract_board_contours(content)` | `(outline, cutouts)` |
| `parse_guide_paths(content, layer)` | `List[GuidePath]` from a user layer |
| `parse_keepout_zones(content, layer)` | `List[GuidePath]` (closed only) |

There is also `parse_s_expression(text)`, a small recursive S-expression
parser returning nested lists, for ad-hoc digging.

## Gotchas

- **`global_x/y` vs `local_x/y`**: global is after footprint rotation, local
  is before. Use globals for anything board-related.
- **Net ID 0** is "no net". Skip it when iterating nets to route, but
  remember its through-hole pads still block routing.
- **`pad.layers` may contain `'*.Cu'`** (through-hole pads). Expand with
  [`net_queries.expand_pad_layers`](api-net-analysis.md#expand_pad_layers)
  before comparing to routing layers.
- **`board_info.stackup` is on `board_info`**, not on `PCBData` directly,
  and is an empty list when the board file has no stackup section (impedance
  functions then fall back to FR4 defaults).
- **Rotated pad sizes are resolved for you** — `size_x`/`size_y` are already
  board-axis-aligned for orthogonal pads; for diagonal pads apply
  `rect_rotation` to the rectangle (don't re-derive a swap from `rotation`).
