# KiCad Parser API (`kicad_parser`)

Parses `.kicad_pcb` files (KiCad 6 through 10; a KiCad 5 file is
[refused](#kicad-5-and-older-refused)) into plain Python dataclasses:
pads, nets, tracks, vias, zones, footprints, board outline, stackup, and
keepouts. No KiCad installation is required — the parser reads the
S-expression file format directly.

```python
from kicad_parser import parse_kicad_pcb

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')
```

All coordinates and sizes are in millimeters. See
[Python API Overview](python-api.md#conventions) for shared conventions.

Examples run from the repository root with the engine directory on the
import path: `PYTHONPATH=py_router python3 <your script>` (see the
[overview](python-api.md)).

## Contents

- [Main entry points](#main-entry-points)
- [PCBData](#pcbdata) and its [dataclasses](#dataclasses)
- [Net filtering](#net-filtering-get_nets_to_route)
- [Package detection and BGA helpers](#package-detection-and-bga-helpers)
- [Version handling (KiCad 9 vs 10)](#kicad-version-handling)
- [Pad and copper helpers](#pad-and-copper-helpers)
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
- Raises `FileNotFoundError` if the file doesn't exist, and
  `UnsupportedBoardFormat` (a `ValueError`) for a KiCad 5 or older file
- Detects KiCad 9 vs 10 automatically (see
  [version handling](#kicad-version-handling))
- Reads the text the way KiCad does (`read_board_text`): a `(teardrops ...)`
  token missing its opening paren is repaired (#1149) and a board written on
  one line is reflowed, each with a `NOTE` on stderr. A reference claimed by
  two footprint blocks prints a `WARNING` on stderr too (see
  [Gotchas](#gotchas)); stdout stays clean for tools that print JSON
- Keeps nothing of the board's text once it returns. Three whole-file
  questions (pad primitives masked, footprint blocks, footprint edge points)
  are memoised so one parse asks each once, and the memos are keyed on the
  text, so `parse_kicad_pcb` empties them as it returns
  (`release_text_caches()`, which a caller scanning board text itself can
  call too). Kept, they held the text and a masked copy: 216 MB after
  parsing a 54 MB board.
- Finds each footprint block, and each footprint's shapes, once. Several
  extractors walk the same blocks (the board bounds, the outline owners,
  the footprints, the graphic copper, the paste openings), some over the text
  with pad primitives masked out -- which has the same blocks, since a mask
  is a balanced token of the same length. While a parse runs it keeps each
  block's end, and each footprint's shape offsets, in `_PARSE_MEMO`, for its
  own texts only; the memo is gone when the parse returns. A 54 MB board
  parsed in 8.1 s and parses in 5.3 s, to the same `PCBData`.

### `build_pcb_data_from_board`

```python
build_pcb_data_from_board(board,
                          guide_layer: str = "User.1",
                          keepout_layer: str = "User.2") -> PCBData
```

Build the same `PCBData` structure directly from a live `pcbnew.BOARD`
object (inside KiCad's Python console or an action plugin). Much faster than
saving and re-parsing, and reflects unsaved edits.

Layer names are the file's canonical tokens (`In1.Cu`, `User.1`), as
`parse_kicad_pcb` reads them, even when Board Setup gives a layer a display
name (In1.Cu shown as "GND"). Map a live board's layers the same way:
`pcbnew_copper_layer_names()` returns `{pcbnew layer id: 'F.Cu' | 'In1.Cu' |
... | 'B.Cu'}`. `board.GetLayerName()` returns the display name, which matches
nothing the engine emits (#1056).

```python
# Inside KiCad's scripting console:
import pcbnew
from kicad_parser import build_pcb_data_from_board
pcb = build_pcb_data_from_board(pcbnew.GetBoard())
```

The live board's tracks, arc pieces and vias are read by ONE extractor,
`pcbnew_track_copper(board)` -> `([Segment], [Via])` (uuid, KiCad lock, via
type and protection spec included), and its copper GRAPHICS (#337 board
drawings, #908 footprint copper) by ONE emitter,
`pcbnew_graphic_copper(board, live_fps, live_keys, get_layer_name, to_mm)`,
with the footprints and their keys from `pcbnew_live_footprints(board)`. A
caller that keeps one `PCBData` across edits of the live board (the GUI dialog
does, step to step) refreshes it with `resync_live_copper(pcb, board)` -- the
board's tracks, vias and copper graphics through those two, so a footprint
moved in the session carries its copper art, every memo derived from the old
copper dropped -- and then, once pads and zones are current,
`resolve_live_sense_branches(pcb, board)`, which re-reads the board's
`sense:<REF>.<PAD>` groups (#1279). Building copper objects by hand loses
what the extractor carries: the GUI's re-sync once did, and every step after
the first ran with no KiCad lock, no uuid and no arc copper.

## PCBData

The top-level container returned by both entry points.

| Field | Type | Meaning |
|-------|------|---------|
| `board_info` | `BoardInfo` | Layers, bounds, outline, stackup, keepouts |
| `nets` | `Dict[int, Net]` | Nets keyed by net ID (ID 0 = unconnected) |
| `footprints` | `Dict[str, Footprint]` | Every footprint BLOCK, keyed by reference (`'U9'`, `'R1'`). Duplicated references get a file-order ordinal (`TP4`, `TP4~2`); a reference-less block is keyed `#<uuid>` (#726) |
| `duplicate_references` | `Dict[str, int]` | `{reference as the FILE spells it: how many blocks claim it}`, for the ones claimed more than once. Advisory -- a duplicate is legal in KiCad |
| `segments` | `List[Segment]` | All track segments |
| `vias` | `List[Via]` | All vias |
| `pads_by_net` | `Dict[int, List[Pad]]` | Pads grouped by net ID (fast lookup) |
| `zones` | `List[Zone]` | Filled copper zones (planes) |
| `guide_paths` | `List[GuidePath]` | User-drawn guide polylines from `guide_layer` |
| `keepout_zones` | `List[GuidePath]` | User-drawn keepout polygons from `keepout_layer` |
| `kicad_version` | `int` | File format version (e.g. `20241229` = KiCad 9) |
| `uses_name_nets` | `Optional[bool]` | The file's net-token dialect, decided from its content (`board_uses_name_nets`, see [version handling](#kicad-version-handling)). `None` on the pcbnew path, which writes through pcbnew rather than net tokens. Read it through `pcb_uses_name_nets(pcb)` |
| `net_id_to_name` | `Dict[int, str]` | Net ID → name map, filled for every board (needed when writing name-net files). A name-net board has no key `0` (no `Net` is built for no-net), so resolve a via's name with `kicad_writer.via_net_name` |
| `groups` | `Dict[str, List[str]]` | KiCad `(group ...)` blocks (#459) as `{group name: [footprint reference, ...]}`, members resolved by `Footprint.uuid`; non-footprint members are dropped. The designer's own grouping, which placement ranks above inferred ones |
| `source_path` | `str` | Absolute path this data came from (`""` = in-memory). Lets engines with no `input_file` discover sibling project files, e.g. the `.kicad_dru` per-layer clearance rules (#498) |
| `exact_fill_provider` | `Optional[Callable]` | Zero-arg callable returning `{(net_name, layer): [island_polygon, ...]}` — KiCad-truth fill for exact-fill consumers (#424). `None` (file-parsed boards) = refill `source_path`; `build_pcb_data_from_board` sets it to a staged-save refill of the live board (live copper AND live clearances) |
| `live_rules_provider` | `Optional[Callable]` | Zero-arg callable returning the LIVE board's design-settings rules `{'min_track_width': mm, 'min_connection': mm, 'min_hole_clearance': mm}` (#1187, #1038), read when called. `None` (file-parsed boards) = the sibling `.kicad_pro`; `build_pcb_data_from_board` sets it, because mid-plan the file beside the live board is the original project while the floors the plan lowered live in pcbnew's memory -- and a Board Setup edit lives there before it is saved, or on a board with no file at all (#1238). Read by `fix_kicad_drc_settings.connection_width_floor` and `obstacle_map.declared_hole_clearance`. It answers only the rules the live board DECLARES (`fix_kicad_drc_settings.live_declared_rules`): pcbnew holds every project-less board at its factory values (`pcbnew.DEFAULT_*`: hole clearance 0.25, track 0.2, ...), which the file does not declare, so a live value counts when the project declares the key, a GUI writer of this session targeted it, or it differs from the factory value. `None` when it declares none of them, which sends both readers to the file |
| `live_records_provider` | `Optional[Callable]` | Zero-arg callable returning the LIVE board's `kicad_routing_tools` records (fab-floor origin, protected nets, impedance specs, pour promises, ...): the newer of its project's copy and the copy on the board itself (`tool_records.live_namespace`), read when called. The GUI keeps the records on the board because KiCad's own project save drops what was written into the `.kicad_pro` after load. `None` (file-parsed boards) = the files: the newer of the project's copy and the `.kicad_pcb`'s (`tool_records.effective_namespace`). Read by the `protected_nets` readers and the hole floor's origin |
| `paste_apertures` | `List[PasteAperture]` | Every solder-paste OPENING (#962), from `paste_apertures.build_paste_apertures`. Sources: `pad` (a pad on a paste layer, grown by its resolved paste margin -- pad, then footprint, then board setup, per axis, ratio and margin independently, clamped at -size/2 except for a custom pad, which KiCad does not clamp), `paste_only_pad` (a pad on a paste layer with no copper, e.g. windowpanes) and `graphic` (a paste-layer shape, e.g. esp_prog U2's F.Paste tab). Which nets an opening concerns is `paste_apertures.aperture_nets` (the per-net lists `apertures_by_net` / `apertures_for_net` are memoised): a pad opening its pad's net, a graphic or paste-only one the nets of the owner's copper it overlaps. Both parse paths feed the one builder, and the GUI's pre-step re-sync rebuilds it from the live board (`resync_live_paste_apertures`) once the parts are current, so a part moved in the session moves its openings |
| `graphic_copper_unmeasured` | `List[dict]` | Footprint copper the parser does NOT model, per owner (`{owner_ref, kind, reason}`, kind `logo` / `curve` / `text`), so the off-outline grade can say what it did not measure (#962) |
| `sense_branches` | `List[SenseBranch]` | One entry per KiCad group named `sense:<REF>.<PAD>` (#1279, `sense_branch.py`): a protected Kelvin sense branch whose join pad is `REF.PAD`, its member copper resolved by uuid, its sense pads, and the `reason` it is refused when it cannot be protected. Both parse paths resolve it the same way (`board.Groups()` on the live board) and `compare_pcb_data` diffs it. Separate from `groups` |

### `PCBData.get_via_barrel_length`

```python
pcb.get_via_barrel_length(layer1: str, layer2: str) -> float
```

Distance in mm through the stackup between two copper layers (including both
copper layers' own thickness). Returns `0.0` if the board has no stackup
section or either layer is missing from it. Used for accurate length/time
matching.

`pcb.net_tie_exempt_pad_ids(net_id)` is the other method: the `id()`s of the
pads that `net_id`'s copper may ignore because a footprint's
`net_tie_groups` ties them to one of its pads (see `Footprint` below).

## Dataclasses

### `Pad`

| Field | Type | Meaning |
|-------|------|---------|
| `component_ref` | str | Parent footprint reference (`'U9'`) |
| `pad_number` | str | Pad identifier (`'1'`, `'H2'`; may be empty). `''` for a pad that cannot carry a number -- an NPTH hole, or an SMD/connector pad with no copper layer (an aperture) -- whatever the file wrote, as pcbnew's reader blanks it (`PAD::CanHaveNumber`) |
| `global_x`, `global_y` | float | Absolute board position (footprint rotation applied) |
| `local_x`, `local_y` | float | Position relative to footprint origin (before rotation) |
| `size_x`, `size_y` | float | Pad dimensions in board space. A KiCad 9 `(padstack ...)` pad is the per-axis MAX over the copper layers it is ON (within the board's copper): an entry for a layer the pad has no copper on does not count (both parse paths) |
| `shape` | str | `'circle'`, `'rect'`, `'roundrect'`, `'oval'`, `'trapezoid'`, `'custom'` |
| `layers` | List[str] | Layers the pad is on, as the SET pcbnew loads (`canonical_pad_layers`): `'*.Cu'` = every copper layer; `F&B.Cu` reads as `'F.Cu', 'B.Cu'` and `*.Adhes` / `*.Fab` / `*.CrtYd` / `*In.Cu` as their members; `*.Mask` / `*.Paste` / `*.SilkS` stay wildcards (two explicit members collapse to one); a **plated** through-hole pad is on every copper layer of the board whatever the file lists. File order is kept otherwise. Both parse paths give the same set |
| `net_id` | int | Net ID (0 = unconnected) |
| `net_name` | str | Net name (`''` if unconnected) |
| `rotation` | float | Total rotation in degrees (pad + footprint), normalized to [0, 360) |
| `drill` | float | Drill diameter (0 for SMD, > 0 for through-hole). Oval/slot drills (`(drill oval w h)`) report `max(w, h)` so they keep `> 0` / through-hole semantics. Read in exponent form too: `(drill 1e-06)` is 1 nm, not 1 mm. |
| `drill_w`, `drill_h` | float | A slot drill's size in the PAD frame (`0` = round). Use `pad_drill_capsule` / `pad_drill_circles` ([helpers](#pad-and-copper-helpers)) for the real slot geometry, not `drill` |
| `hole_x`, `hole_y` | Optional[float] | The drill position when the pad's copper is OFFSET from it (`(drill (offset x y))`, castellated-module paddles); `None` = the hole is at `global_x`/`global_y`. `global_x`/`global_y` is always the COPPER centre |
| `polygons` | Optional[List[List[Tuple]]] | A custom pad's real copper outline(s), or a chamfered-rect pad's outline, GLOBAL mm (see below); `None` for ordinary pads, which `size_x`/`size_y` model exactly |
| `pinfunction` | str | Pin function from the schematic (`'TX'`, `'~RESET'`) |
| `pintype` | str | Pin electrical type (`'passive'`, `'input'`, `'power_in'`) |
| `pad_type` | str | Pad kind: `'smd'`, `'thru_hole'`, `'np_thru_hole'`, `'connect'`. NPTH pads carry **no copper** (their size is just the mask opening, even when `layers` lists `*.Cu`), so copper-clearance checks skip them; only their drill hole matters. |
| `roundrect_rratio` | float | Corner radius ratio for roundrect pads (0–0.5); on a chamfered-rect pad, the rounding of its corners that are not chamfered |
| `geometry_approximations` | Tuple[str, ...] | Shape variants simplified by the parser, currently chamfered pads (whose outline is in `polygons`, but whose box is still what placement's edge grade and a grown paste opening read) and per-layer padstacks. Both text and live-board paths disclose them. An empty tuple does not certify arbitrary custom geometry; exactness still depends on the consumer's supported shapes. Placement edge grading retains approximate findings but marks these variants unmeasured. |
| `rect_rotation` | float | Residual rectangle tilt in the global frame, in (-90, 90]. `0` for axis-aligned pads (the common case); non-zero only for pads on non-orthogonal angles. |
| `local_clearance` | float | The pad's **resolved** clearance override in mm (issue #326): its own `(clearance …)` token, else the footprint-level override, else `0` (= use the global/netclass clearance). KiCad enforces `max(the two items' clearances)` per pair; the router's obstacle stamps and `check_drc` honor this the same way. Negative (shrinking) overrides clamp to `0`. |
| `castellated` | bool | KiCad's `(property pad_prop_castellated)`: a deliberate half-hole pad **on** the board outline. Set by both parse paths. The routing mains run a castellated-landing retract post-pass that pulls track ends landing inside such a pad's edge-clearance zone back to the pad's inner reach (`pcb_modification.retract_castellated_landings`). |
| `unconnected_layer_mode` | str | KiCad's unconnected-layer mode, from `(remove_unused_layers …)` + `(keep_end_layers …)`: `'keep_all'`, `'remove_all'` or `'remove_except_start_end'`. On a layer the mode removes, KiCad flashes a through-hole pad only when copper reaches its **hole**, so a track ending in the annulus short of the drill is not connected there. Set by both parse paths; `connectivity.pad_unflashed_layers(pad, layers)` names those layers. |
| `die_length` | float | KiCad's **pad-to-die** length in mm, `(die_length …)` / `PAD.GetPadToDieLength()`: the bond wire or package trace inside the part. KiCad adds it to the net's length in its tuners (start and end pads) and its length/skew DRC (every pad). `0.0` = none declared. Set by both parse paths (#1248); `net_queries.net_match_length` and every length-matching pass add it. |
| `die_delay` | float | The time twin in **ps**, `(die_delay …)` / `PAD.GetPadToDieDelay()` (KiCad 10; `0.0` on KiCad 9). Time matching adds it; a `die_length` is never converted into a delay, as in KiCad 10. Set by both parse paths (#1248). |
| `sense` | str | The `sense:<REF>.<PAD>` group this pad is a SENSE pad of (#1279): a pad of the branch's net that its member copper touches, the join pad excepted. Set only on a protected branch; the router never attaches new copper to it. `''` otherwise |
| `zone_connection` | str or None | The pad's zone-connection **override**, spelled like `Zone.pad_connection` (`'none'`, `'thermal'`, `'solid'`, `'thru_hole_only'`): its own `(zone_connect N)`, else its footprint's, **resolved** at parse time like `local_clearance` (pcbnew: `PAD.GetZoneConnectionOverrides()`). `None` = no override, so each zone's `connect_pads` decides. Never read it raw to ask whether a pour reaches the pad: `pad_zone_connection(pad, zone)` applies the zone and the thermal-for-PTH rule, and `pad_joins_zone(pad, zone)` is `False` exactly when KiCad keeps that zone's fill off the pad (`'none'`). Then the fill model knocks the pad out at the zone's clearance and `check_net_connectivity` gives it no pour credit, so the pad needs a track, a via or another pad (a GND solder-jumper half with `(zone_connect 0)` shipped open while every grade credited the B.Cu pour). A `.kicad_dru` `zone_connection` rule, which KiCad ranks between the pad's and the footprint's override, is not read. Set by both parse paths; `compare_pcb_data` diffs it |

Pad dimensions are resolved into board space using the pad's **absolute** angle
(the KiCad `(at … angle)` already includes the footprint rotation; applying the
footprint rotation a second time would make adjacent pad copper overlap). For
near-orthogonal pads (0/90/180/270°) `size_x`/`size_y` are swapped as needed so
they are board-axis-aligned and `rect_rotation` is `0`. A genuinely diagonal pad
keeps its given dimensions and carries the tilt in `rect_rotation`; the obstacle
map and DRC rotate the pad rectangle by that angle. `check_pads.py` flags pads
whose resolved copper overlaps a different-net neighbour (a modelling error).

A **custom** pad carries its real copper in `polygons` (GLOBAL mm; their union
is the copper), and its `size_x`/`size_y`/`rect_rotation` follow one rule on
both parse paths: a pad whose copper IS its anchor -- no primitive, or every
primitive inside the anchor shape -- is the anchor rect with its residual
tilt, like an ordinary pad; otherwise it is the board-axis box of the copper
(anchor shape plus primitives) about the copper centre, `rect_rotation` 0. The
box never includes the drill (an NPTH custom pad is its anchor, not its hole)
and is centred on the copper, not the hole, when the pad has a copper offset.
`anchor_size` is the anchor in the pad frame. A box about the pad holds copper
drawn to one side of it at twice its size, so when the copper's centre is more
than `COPPER_BOX_OFFSET_MM` (0.01) off the pad's, `copper_box` carries the
copper's own box `(x0, y0, x1, y1)` about `global_x`/`global_y`, board frame
(bitaxe's CSD17311Q5 drain pad: `size_x` x `size_y` 10.0 x 9.1, `copper_box`
5.0 x 4.6 to one side); None otherwise. Both parse paths set it, from the same
points as the box (text) and from pcbnew's effective shape. Placement reads it
through `legality.pad_copper_box`; routing reads `polygons`.

A **chamfered-rect** pad -- KiCad's `roundrect` (or `rect`) with
`(chamfer <corners>)` and `(chamfer_ratio r)`, which KiCad's Eagle importer
writes for octagon pads -- carries its outline in `polygons` too (#1450), so
the cut corners are not copper to the obstacle map, `check_drc`,
`check_connected` and `check_pads`. Each named corner (`top_left`,
`top_right`, `bottom_left`, `bottom_right`, in the pad's own frame, which KiCad
mirrors when it flips a footprint) is cut by `r * min(w, h)` along each side;
every other corner is rounded by `roundrect_rratio * min(w, h)` (0.25 when the
file names none, as pcbnew loads it), and the outline is turned by the pad's
absolute angle about the copper centre. The text parser builds it with
`chamfered_rect_polygon`; `build_pcb_data_from_board` takes pcbnew's effective
polygon, and the two agree to within pcbnew's arc error. `shape` reads
`'roundrect'` and `size_x`/`size_y` stay the pad's box. A pad with a per-layer
padstack keeps its box. A paste opening with a positive margin is the grown
box (a superset); at zero or a negative margin it is the copper outline.

### `Segment`

| Field | Type | Meaning |
|-------|------|---------|
| `start_x`, `start_y` | float | Start point |
| `end_x`, `end_y` | float | End point |
| `width` | float | Track width |
| `layer` | str | Layer name |
| `net_id` | int | Net ID |
| `uuid` | str | UUID from the file, or the live item's `m_Uuid` on the pcbnew path (#1279) (`''` for newly created segments and for uuid-less file items — KiCad treats the token as optional, PR #534) |
| `start_x_str`, … | str | Original coordinate strings, kept for exact file matching |
| `graphic` | bool | This copper came from a **graphic**, not a track (issue #337, extended to footprint shapes by #908). It is real copper for obstacles and DRC, and it is immutable: cleanup passes must never prune it and writers cannot strip it, because there is no `(segment …)` block to match. A **board-level** graphic (`gr_*`, `owner_ref` empty) conducts as it does in KiCad 10: its stroke joins the same-net copper it touches and a filled shape (`graphic_ring`) its whole interior (`check_connected.conducting_stroke`). Footprint copper (`owner_ref` set) conducts nothing, as in KiCad. The router never targets either. |
| `locked` | bool | KiCad `(locked yes)`: the user pinned this copper. Its net is never rip-eligible (#521, no override) -- before the run and in every in-run rip ladder, rescue and blame (`protected_nets.rip_refusal`, #1261); locked copper was already an obstacle (#150). No layer switch moves it (`stub_layer_switching.get_stub_info` answers None for a stub whose switch would), no cleanup pass removes, trims or re-bends it, and a net whose copper is all locked and whose pads grade connected is not routed (#1261). Both parse paths set it. The text parser reads the token anywhere in the block, as KiCad does -- `(locked yes)`, `(locked)` or the bare `(segment locked ...` word (#1158) -- and reads every track block by its own tokens, so a hand-written block in any field order is modelled, and one it still cannot model is reported on stderr rather than dropped. |
| `owner_ref` | str | For copper drawn **inside a footprint**, the disambiguated footprint key that owns it (`'U2'`, `'TP4~2'`); `''` for board-level graphics and every routed track (#908). It is what lets a DRC report name the object the way KiCad does — `net_0 [Polygon(U2)]` beside KiCad's *"Polygon [\<no net\>] of U2 on F.Cu"* — and what scopes the own-pad obstacle lift to the owning part. |
| `drawn_width` | Optional[float] | Graphic copper only: the stroke AS DRAWN (#962). `width` models a filled shape drawn at stroke 0 at the fab track width, which is right for an obstacle and wrong for a measurement; the off-outline grade reads this. An UNFILLED shape whose file stroke is 0 is stroked at `UNSTROKED_SHAPE_WIDTH` (0.1 mm), as pcbnew's reader strokes it (paste graphics too). A polygon's `(arc ...)` point entries are linearized in order (`poly_points_with_arcs`; the pcbnew builder linearizes the live chain's arcs with the same function, so both paths emit the same segments). `None` for tracks |
| `graphic_kind` | str | Graphic copper only: the primitive (`'line'`, `'arc'`, `'poly'`, `'rect'`, `'circle'`; a footprint rect at a non-cardinal angle reads `'poly'`, as pcbnew loads it). `''` for tracks |
| `graphic_circle` | Optional[Tuple] | For a circle, its TRUE `(cx, cy, r)`: the outline is a 16-gon whose chord midpoints sit 1.9% of r inside the curve, so a reach measured on the chords under-reads |
| `graphic_filled` | bool | A closed graphic (poly/rect/circle) whose interior is copper, by KiCad's loader rules (a `(fill ...)` token; without one a poly is filled and a rect or circle only at stroke 0). The segments model only the outline; the off-outline grade reads this to look inside |
| `graphic_ring` | Optional[Tuple] | For a FILLED closed graphic, its outline vertices as one tuple shared by every segment of the shape; `None` otherwise (#1181). The obstacle map stamps the interior it encloses, check_drc grades copper inside it (`check_drc.filled_graphic_shapes`), and for board-level copper the connectivity graph joins whatever lies inside it. Both parse paths set it |
| `sense` | str | The protected `sense:<REF>.<PAD>` group this track is a member of (#1279), `''` otherwise. Drives the routing rule: for its own net the copper is foreign except on the join pad, and never a place to attach |
| `held_by` | str | The `sense:` group holding this track immutable (#1279), `''` otherwise: set whenever the group's members resolve on the join pad's net, protected or refused. `kicad_parser.is_immutable_copper(item)` (`locked or held_by`) is what the move/rewrite/remove guards read. Not a KiCad lock: never written, and the net's routes of the run stay rippable; only a whole-net strip of the net's pre-existing copper is refused (`protected_nets.protection_map` reason `sense:<REF>.<PAD>`) |

### `Via`

| Field | Type | Meaning |
|-------|------|---------|
| `x`, `y` | float | Position |
| `size` | float | Outer (pad) diameter |
| `drill` | float | Hole diameter |
| `layers` | List[str] | The two end layers, e.g. `['F.Cu', 'B.Cu']`. A through via reads `['F.Cu', 'B.Cu']` on both paths whatever its `(layers ...)` says: KiCad loads a via with no type word as THROUGH and snaps its pair, pcbnew's `TopLayer()`/`BottomLayer()` report that, and the text parser does the same (a through via written `(layers "B.Cu" "F.Cu")` included). A typed via keeps its file pair, top layer first as pcbnew reads it back, so both paths hold one ordered pair. Whether the via covers only part of the stack is `via_type`, never this pair |
| `via_type` | str | `'through'`, `'blind'`, `'buried'` or `'micro'`: the block's type word (text) or `GetViaType()` (pcbnew). KiCad loads a via with no type word as THROUGH whatever its `(layers ...)` says, and this repo's writer emits no type word, so only a typed via has copper on part of the stack, the layers between its end layers (`geometry_utils.via_on_layer` / `via_span_layer_indices`, which the obstacle maps and check_drc read, #1268). A `Via` constructed without one is `'through'` |
| `net_id` | int | Net ID, as pcbnew's connectivity pass leaves it on load: a non-free via with no same-net track or pad on it that sits in the zone FILL of exactly one net (on a layer it spans), with no track or pad of another net on it, takes that net whatever the file wrote (`propagate_via_cluster_nets`; antmicro lpddr4_test_board's stitching vias). A via joined to netted copper only by a track is not re-netted here |
| `uuid` | str | UUID from the file, or the live item's `m_Uuid` on the pcbnew path (#1279) |
| `sense`, `held_by` | str | As on `Segment` (#1279) |
| `free` | bool | `(free yes)` flag — KiCad won't reassign the net from overlapping copper |
| `locked` | bool | KiCad `(locked yes)`, as on `Segment`: its net is never rip-eligible (#521) and no pass moves or removes the via |
| `tenting_attrs` | Dict[str, str] | Protection spec as `{token: raw inner s-expr}` for `tenting`/`covering`/`plugging`/`capping`/`filling`, e.g. `{'covering': '(front no) (back no)', 'capping': 'no'}`. `{}` = the board specified nothing (KiCad inherits its board default). Read by **both** parse paths (text and `build_pcb_data_from_board`) in the same normalized form. What a via is actually FABRICATED with is resolved token by token against `BoardInfo.via_protection_setup` by `fab_notes.effective_via_protection` (#962) |

Pass `tenting_attrs` (and `via_type`) back to `generate_via_sexpr` for any via
that already existed, so a ripped-and-re-placed via keeps its real spec instead
of losing it (#489 §8) — that matters most for via-in-pad, which needs IPC-4761
Type VII (filled + capped + plated) — and a blind or micro via stays one. A via
you ADD gets no token and inherits the board's `(setup ...)`, except one under
solder, which the ship-time stamp declares Type VII (#962, see the writer
docs).

### `Net`

| Field | Type | Meaning |
|-------|------|---------|
| `net_id` | int | Net ID (0 = unconnected) |
| `name` | str | Net name (may be `''`) |
| `pads` | List[Pad] | All pads on this net |

### `Footprint`

| Field | Type | Meaning |
|-------|------|---------|
| `reference` | str | The key in `pcb.footprints`, which is the reference designator for all but a duplicated one (`TP4~2`) or a reference-less block (`#<uuid>`). `Pad.component_ref` carries the same value |
| `footprint_name` | str | Library ID (`'interf_u:PGA120'`) |
| `x`, `y` | float | Footprint origin |
| `rotation` | float | Rotation in degrees |
| `layer` | str | `'F.Cu'` or `'B.Cu'` |
| `pads` | List[Pad] | The footprint's pads |
| `value` | str | Component value (`'100nF'`, `'MCF5213'`) |
| `dnp` | bool | Do-not-populate (the `dnp` word in `(attr ...)`; pcbnew `IsDNP()`). A no-pop series part is an open circuit, so its pads do not join two nets |
| `attrs` | Tuple[str, ...] | The footprint's `(attr ...)` flags, sorted, limited to `FOOTPRINT_ATTR_TOKENS` (`smd`, `through_hole`, `board_only`, `exclude_from_pos_files`, `exclude_from_bom`, `dnp`, `allow_missing_courtyard`, `allow_soldermask_bridges`), the set both parse paths can spell (#1098) |
| `has_model` | bool | The footprint declares a 3D model. With `attrs` it says what the assembly house places: a `board_only` / `exclude_from_pos_files` part with no model is board copper, not a component (#1098) |
| `ref_label` | Optional[RefLabel] | The Reference text's geometry (#481); `None` for a reference-less footprint. Fields: [Python API overview](python-api.md#reference-labels-reflabel) |
| `locked` | bool | KiCad `(locked yes)` on the footprint (also `(locked)` and KiCad 6's bare `locked` word): placement never moves it. Both parse paths set it (pcbnew: `FOOTPRINT.IsLocked()`) and `compare_pcb_data` diffs it, because the GUI's placement Apply sets every live lock from the text parse of the result board (#1054) |
| `clearance` | float | Footprint-level `(clearance …)` override in mm (0 = none). Already **resolved into** each pad's `local_clearance` at parse time — read that field for clearance decisions; this records the raw footprint value (issue #326). |
| `zone_connection` | str or None | Footprint-level `(zone_connect N)` override (pcbnew `FOOTPRINT.GetLocalZoneConnection()`), raw; `None` = not set. Already resolved into each pad's `zone_connection` without one of its own |
| `uuid` | str | The footprint's own `(uuid …)` (KiCad 6: `(tstamp …)`); KiCad's `(group …)` blocks list members by it (#459) |
| `sheet_path` | str | The schematic `(path "/<sheet uuid>/…/<symbol uuid>")`. The LAST uuid is the symbol, the rest the sheet instance; two instances of one hierarchical sheet carry the SAME symbol uuid under different prefixes, which `place_pose replicate` maps parts by (#1211). A single-uuid path is top level |
| `sheet_name`, `sheet_file` | str | The schematic sheet's NAME and FILE as the file spells them (#1211): KiCad 8+ header tokens `(sheetname "/CH1/")` / `(sheetfile "strip.kicad_sch")`, or KiCad 6/7's `Sheetname` / `Sheetfile` properties (KiCad 6 dev files spell them `Sheet name` / `Sheet file`; either property counts only up to file version `SHEET_PROPERTIES_MAX_VERSION` (20230619), where pcbnew stops reading it -- in a later file it is an ordinary user field) (the value is a path `"/CH1/"` on some boards, a bare name `"IO_Buffer_B"` on others; not normalised). Read from the block header only, escapes undone. `build_pcb_data_from_board` reads `FOOTPRINT.GetSheetname()` / `GetSheetfile()`, which return the same spelling for both forms (KiCad 10.0.0); `compare_pcb_data` diffs all three sheet fields, and `tests/gui_parity/test_1211_sheet_name_parity.py` pins the parity. `''` = the file names none |
| `net_tie_groups` | List[List[str]] | Pad-number groups the footprint deliberately shorts (`(net_tie_pad_groups "1, 2")` — Kelvin shunts, net-ties). KiCad's clearance exemption between the grouped pads is **local**: a tied net's copper may contact the partner pad only where the contact lies on its own pad. Query per-net via `pcb.net_tie_exempt_pad_ids(net_id)`. |
| `owns_edge_cuts` | bool | The footprint draws Edge.Cuts geometry of its own (`fp_line`/`fp_rect`/`fp_arc`/`fp_circle`/`fp_poly`/`fp_curve` on that layer, issue #829). Its `(at x y rot)` therefore transforms part of the board outline. |
| `owns_board_outline` | bool | ...and that geometry is **the board's own boundary** rather than a relief the part carries, so moving the footprint would resize the board (issue #829). A footprint is *carried* (this stays `False`, and it remains movable) only when its Edge.Cuts segments **close on themselves** — a window, slot or milled relief — **and** either the footprint has pads and the shape **encloses none of them** (judged in the footprint's own frame, so the answer is the same wherever the part sits, on the board or off it; issue #1409), or the shape lies inside the outline the board draws without it. An open path cannot be a cut-out, so it is always the boundary. The first test reads which side of the ring the part's copper is on: a hole beside or under a part leaves its pads outside the ring, while a ring that IS the board's edge has the part's pads inside it (a castellated pad ON the ring counts as inside). A ring around one of its own pads (a plated slot) and a pad-less footprint's ring are judged only by the second test, at the part's current pose. crkbd draws 184 per-LED windows as carried geometry, and #628 measured that freezing such a part costs it every legal pose it has. **Movers gate on this field, never on `owns_edge_cuts`.** Computed by `kicad_parser.footprint_outline_owners` (text) and `footprint_outline_owners_from_pcbnew` (live board), which share the one decision function `classify_outline_owners`. |


**Footprint copper** (issue #908). A footprint may draw copper of its own on
`F.Cu`/`B.Cu` — the tab of a SOT89/DPAK, a PCB antenna, a solder-jumper
bridge. Those `fp_poly`/`fp_line`/`fp_arc`/`fp_rect`/`fp_circle` shapes are
modelled exactly like board-level graphics: net-0 `Segment`s with
`graphic=True` and an `owner_ref`, so they are obstacles and DRC copper for
free. A footprint shape **cannot carry a `(net …)` in KiCad**, so #337's
"net-tied copper is functional, net-less copper is a logo" rule cannot tell
the two apart; `footprint_copper_is_functional(pad_count)` does, and the
writer's silkscreen mover reads the same predicate — a footprint with copper
pads owns a land pattern (modelled, kept on copper), a pad-less one is a logo
(relocated to silk by the writer, as #146 has always done, and therefore not
modelled). The segments are the **perimeter**; a filled shape's interior is
its `graphic_ring` (#1181), which the obstacle map stamps for every foreign net
and check_drc grades containment in -- the same for board-level graphics. The
off-outline grade (`check_drc.footprint_graphic_outline_census`, #962) reads
`Segment.graphic_filled` to look inside a filled shape, and another part's
PADS on the copper (`check_drc.footprint_graphic_pad_census`, #1266) are
measured the same way -- strokes, then the interior -- for check_drc, the
placement gate and check_assembly. A footprint's OWN pads against each other
are `check_drc.footprint_own_pad_census` (#1306), keyed by the index into
`footprint.pads`, so a pair can be found again on a `--baseline` board.

### `Zone`

| Field | Type | Meaning |
|-------|------|---------|
| `net_id`, `net_name` | int, str | Zone's net |
| `layer` | str | Layer name |
| `polygon` | List[Tuple[float, float]] | Outline vertices, as pcbnew loads them: an `(arc (start) (mid) (end))` vertex becomes KiCad's own polyline (`kicad_arc_polyline`, below), a vertex equal on the nm grid to the one before it is dropped, and so is a last vertex repeating the first (pcbnew closes the outline as it parses it, in every file format; a KiCad 5-converted board writes one on every zone) |
| `uuid` | str | UUID from the file |
| `priority` | int | `(priority N)`; higher-priority zones win where outlines overlap (0 when absent) |
| `island_removal_mode` | int | 0 = always remove isolated islands (KiCad default), 1 = never, 2 = below `island_area_min` |
| `island_area_min` | float | mm² floor for mode 2 (0.0 when absent) |
| `clearance`, `min_thickness` | float | The pour's own fill clearance (`(connect_pads ... (clearance X))`) and minimum width; `None` when the file has no token |
| `pad_connection` | str | How the pour joins its net's pads: `thermal` (KiCad's default, `(connect_pads (clearance X))`), `solid` (`connect_pads yes`), `none` (`no`) or `thru_hole_only`. Read by both parse paths (`zone_connection_settings` / pcbnew `GetPadConnection()`); route_planes uses it to say what replacing the pour changes (#1329). A pad's own or footprint's override outranks it: ask `pad_zone_connection(pad, zone)` (see `Pad.zone_connection`) |
| `thermal_gap`, `thermal_bridge_width` | float | The `(fill ...)` block's thermal-relief gap and spoke width; `None` when the file has no token (the live board reports pcbnew's default instead) |
| `in_footprint` | bool | Zone is owned by a footprint (nested in its `(footprint ...)` block); points are still board coordinates. Plane creation neither replaces nor aborts on these |
| `teardrop` | bool | KiCad's `(attr (teardrop ...))` (pcbnew `IsTeardropArea()`): a generated teardrop area where a track meets a pad or via. Routing output, so `assess_placement`'s `has_copper` counts it (#1164) |
| `holes` | List[List[Tuple[float, float]]] | The outline's cut-outs: KiCad's file holds ONE outline per zone and every later `(polygon ...)` block of it is a hole in that outline (pcbnew loads it so, `Outline().HoleCount(0)`). The fill never enters one. Read the same way as `polygon`, by both parse paths; `[]` without cut-outs (#1279) |

A teardrop is still COPPER: the obstacle map, DRC and connectivity read it
through `pcb.zones` like any zone. It is never a POUR: a consumer deciding
"is this net a plane / poured net" asks `pour_zones(pcb.zones)`, every zone
but the teardrops. Counted as a pour, a teardropped signal net read as a
plane net (never offered to `--rip-existing-nets`, handed to the plane
finalize), and a GND teardrop on F.Cu read as "GND already pours on F.Cu",
which route_planes replaced, or kept instead of the plane it was asked for.

A zone on no copper layer -- a solder-mask or paste opening drawn as a zone
(`(layers "*.Mask")`, `(layers "F.Mask" "F.Paste")`), a silkscreen fill -- is
not copper, and neither front puts it in `pcb.zones` (pcbnew strips its net on
load). A zone listing copper among other layers is modelled on its copper
layers only.

### `BoardInfo`

| Field | Type | Meaning |
|-------|------|---------|
| `layers` | Dict[int, str] | Layer ID → name for all layers |
| `copper_layers` | List[str] | Copper layer names, top to bottom. Includes every `.Cu` layer regardless of declared use-type (`signal`, `power`, `mixed`, `jumper`) — KiCad often types plane layers `power`. |
| `board_bounds` | Optional[Tuple] | `(min_x, min_y, max_x, max_y)` from Edge.Cuts (board-level graphics AND footprint-embedded outline shapes), or `None` |
| `board_outline` | List[Tuple[float, float]] | Outline polygon for non-rectangular boards (empty if rectangular); the largest outer ring when there are several |
| `board_outlines` | List[List[Tuple]] | EVERY outer ring (#304): split keyboards drawn as two halves, panels. Empty on a single-outline or rectangular board, which means "use `board_outline`" |
| `board_cutouts` | List[List[Tuple]] | Interior cutout polygons |
| `board_edge_contours` | List[List[Tuple]] | Inner Edge.Cuts contours that enclose pads (#505): not cutouts (that would put the pads off-board) but still milled edges, so copper keeps its edge clearance from them |
| `min_clearance_used` | Optional[float] | The smallest copper clearance a routing step actually used this run (`None` until one records it); the routers fold it into the `.kicad_pro` DRC floor |
| `stackup` | List[StackupLayer] | Physical stackup, top to bottom (empty if the board has none) |
| `keepouts` | List[dict] | KiCad keepout rule areas (board-level AND footprint-owned): `{'polygon': [...], 'holes': [...], 'layers': set, 'tracks_allowed': bool, 'vias_allowed': bool, 'copper_pour_allowed': bool, 'pads_allowed': bool, 'footprints_allowed': bool, 'in_footprint': bool, 'owner': str, 'name': str}` (`name` is the area's `(name ...)`, `''` without one: a `.kicad_dru` area predicate names it; `owner` is the key of the footprint a footprint-owned area sits in, `''` for a board-level one; a pad or footprint inside an area that forbids it is KiCad's `items_not_allowed`, graded by `legality.rule_area_item_findings`, #1451) |
| `pad_to_paste_clearance`, `pad_to_paste_clearance_ratio` | float | The board-level paste margin and ratio from `(setup ...)`, the last term of a pad's paste-margin resolution (#962) |
| `via_protection_setup` | Dict[str, str] | The board's via protection policy, `{token: inner}` for all five tokens (`tenting`, `covering`, `plugging`, `capping`, `filling`), canonicalised from KiCad 10's per-token form and the legacy `(tenting front back)` form; a token the board does not declare takes KiCad's factory value (#962) |

**Paste overrides** (#962). `Pad.paste_margin` / `paste_margin_ratio` and
`Footprint.paste_margin` / `paste_margin_ratio` hold the raw
`(solder_paste_margin ...)` / `(solder_paste_margin_ratio ...)` overrides
(`None` = unset; an explicit 0 counts as unset only up to file version
20240201, as KiCad reads it). `Pad.anchor_size` is a custom pad's anchor size,
which is what KiCad sizes a paste RATIO from. `Footprint.parsed_pose` is the
`(x, y, rotation, layer)` the footprint had at parse, so a consumer that moves
parts in memory can re-pose their graphic copper.

### `StackupLayer`

| Field | Type | Meaning |
|-------|------|---------|
| `name` | str | The stackup's own layer name: `'F.Cu'`, `'In1.Cu'`, `'dielectric 1'`, … |
| `layer_type` | str | `'copper'`, `'core'` or `'prepreg'`; mask, paste and silk layers are left out (`'dielectric'` can appear only from a pcbnew build whose stackup bindings answer, see `compare_pcb_data` below) |
| `thickness` | float | Thickness in mm (a locked `(thickness X locked)` reads as X) |
| `epsilon_r` | float | Relative permittivity (dielectric layers) |
| `loss_tangent` | float | Dissipation factor (dielectric layers) |
| `material` | str | Material name (`'FR4'`) |

A dielectric built from several sheets -- KiCad's `addsublayer`, e.g. a
3 x 2116 prepreg -- is ONE `StackupLayer` holding the whole layer (#1222):
`thickness` is the sum of the sheets, `epsilon_r` the series value
`sum(t) / sum(t / er)` (the field of a trace crosses the sheets in turn),
`loss_tangent` weighted by `t / er`, and `material` the sheets' common name
or each sheet's joined by `' + '`. A sheet KiCad left at its sublayer
defaults (`epsilon_r` 1, `loss_tangent` 0) is a field nobody filled in, so it
sits out of those averages. A single-sheet layer is exactly as written.

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

Returns routable nets (≥ 2 pads), filtered by fnmatch wildcards matched
against the full net name: `exclude_patterns` case-insensitively,
`net_patterns` case-sensitively. `net_patterns=None` means all nets; the
default exclusions are `['*GND*', '*VCC*', '*VDD*', '*unconnected*', '*NC*',
'']`. With `component_ref`, only nets touching that component are returned.

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

`detect_bga_pitch` is the MEDIAN adjacent pad gap per axis, then the smaller
axis -- on a regular array that is the pitch, and it is not moved by a few odd
pads. It is deliberately not a `min()` over adjacent gaps: that let one
anomalous pad pair speak for the package (cparti_fpga's 256-ball 1.0mm U1 read
1e-6mm), which collapsed `auto_detect_bga_exclusion_zones`' `edge_tolerance` to
nothing -- and `connectivity.is_edge_stub` compares a pad CENTRE against a
pad-EDGE box, so no pad could then be an edge stub. `1.0` still means "could not
detect"; never 0.0, which callers would read as falsy AND as infinitely fine.

Classification uses the footprint name first, then pad arrangement (grid vs
perimeter) and pad shapes, read on the pins only: a paste or mask window
(`pad_is_aperture_only`) is not a pad here or in `detect_bga_pitch` (#1148).
`DFN` / `MLF` names read as `QFN`; an SMD `PLCC` is `QFN` (perimeter) and a
through-hole PLCC socket `OTHER`. Land-grid and chip-scale families are
classified as `BGA` by name so they get fanout + BGA exclusion zones: `LGA` (land grid array,
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

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
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
has_numeric_net_table(content: str) -> bool # (net <id> "name") declarations present
board_uses_name_nets(content: str) -> bool  # the file's net dialect, from its content
pcb_uses_name_nets(pcb) -> bool             # the same for a parsed PCBData
```

KiCad 10 removed numeric net IDs from the file format (`(net "name")` instead
of `(net 29 "name")`). The parser hides this: with no numeric net table it
assigns synthetic integer IDs, so `net_id`-based code works on both formats,
and it records every board's mapping in `pcb.net_id_to_name`.

Which dialect a file uses is decided from its **content**, not its version
stamp: `board_uses_name_nets(content)` is true when the file has name refs
`(net "name")`, false when it has a numeric `(net N "name")` table and no name
refs (third-party generators write a KiCad-10 stamp over a numeric table,
#1028), and only a board with no nets at all falls back to the stamp. The
parse records the answer as `pcb.uses_name_nets`; read it with
`pcb_uses_name_nets(pcb)`, which falls back to the stamp on the pcbnew path.
**Copper written into a name-net file needs that mapping** so the writer emits
name-based nets — see
[KiCad Writer](api-kicad-writer.md#kicad-9-vs-kicad-10).

```python
from kicad_parser import parse_kicad_pcb, pcb_uses_name_nets

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')
names = pcb_uses_name_nets(pcb)
print(f"File version {pcb.kicad_version}: nets by "
      f"{'name (KiCad 10)' if names else 'numeric id (KiCad 9)'}")
```

### KiCad 5 and older: refused

A board older than `FIRST_SUPPORTED_BOARD_VERSION` (20201115, KiCad's
"module -> footprint" change) writes its parts as `(module ...)` blocks with
bare net names, which this parser does not read. Rather than return a board
with no footprints and no nets, `parse_kicad_pcb` raises
`UnsupportedBoardFormat` (a `ValueError`) saying what the file is and what to
do: open it in KiCad 6 or newer and save it, or use the KiCad plugin, which
reads the board through KiCad itself. The CLIs print it as one `ERROR:` line
and exit 1.

### KiCad 6-era files

pcbnew converts several KiCad 6-era conventions when it loads a file, and the
GUI (which builds from pcbnew) has always seen the converted board.
`parse_kicad_pcb` applies the same conversions, at KiCad's own version cutoffs
(`pcb_io_kicad_sexpr_parser.cpp` / `pcb_io_kicad_sexpr.h`, KiCad 10.0.0), so a
KiCad 6 board parses on the CLI the way it loads in KiCad:

| Convention in the file | Read as | Applies to |
|---|---|---|
| `(layers *.Cu *.Mask)` — names unquoted | the same names | every layer list (KiCad 6 writes pad lists bare) |
| `(pad 1 smd rect ...)` — the pad number unquoted | the same number | all versions (a footprint pasted in from an older library spelling; pcbnew reads it) |
| a pad's `(net 3 GND)` / `(net GND)`, `(pinfunction GND)` — names unquoted | the same name, resolved through the board's nets by name | all versions |
| a zone on `(layers F&B.Cu)` / `(layers *.Cu)` | one `Zone` per layer: F.Cu + B.Cu / every copper layer | all versions |
| a pad on `(layers "F&B.Cu" ...)`, `"*.Adhes"`, ..., or a plated pad declared on one side | `Pad.layers` as the set pcbnew loads (`canonical_pad_layers`) | all versions |
| an NPTH or copper-less pad with a number | `pad_number` `''` | all versions |
| `(property "Sheet name" ...)` / `(property "Sheetname" ...)` | `Footprint.sheet_name` (and the file pair) | version ≤ `SHEET_PROPERTIES_MAX_VERSION` (20230619) |
| `(gr_arc (start CENTER) (end ARC-START) (angle SWEEP))` | the three-point arc pcbnew writes | version ≤ `LEGACY_ARC_FORMATTING` (20210925) |
| a footprint whose `(tags ...)` start with `"net tie"` | one `net_tie_groups` entry of every pad | version ≤ `LEGACY_NET_TIES` (20220815) |
| `(tstamp ...)` | the item's `uuid` (an 8-hex-digit stamp expanded as KiCad's KIID does) | footprints, tracks, vias, zones |
| `~X~` overbars in net names | `~{X}` | version < `NEW_OVERBAR_NOTATION` (20210606) |
| a bare `locked`: `(footprint "X" locked (layer ...`, `(gr_line locked (start ...`; or `(locked)` (20210108-20210423) | `Footprint.locked`; the shape is read as unlocked geometry | all versions (KiCad 6 spells it so) |

The legacy arc rewrite and the shape-lock strip happen on the parser's
**analysis copy** only: never write `upgrade_legacy_arcs` output back into a
file of that version, which KiCad would then fail to load.

```python
layer_list_tokens(body: str) -> List[str]          # names in a (layers ...) body, quoted or bare
canonical_pad_layers(tokens, board_copper=(), plated_through=False) -> List[str]  # a pad's layers as pcbnew loads them
map_layer_list_tokens(body: str, fn) -> str        # rewrite each name, keeping its spelling
upgrade_legacy_arcs(content: str, kicad_version=None) -> str
kiid_from_tstamp(raw: str) -> str
uuid_or_tstamp(text: str) -> str                   # (uuid "...") else (tstamp ...), else ''
convert_to_new_overbar_notation(old: str) -> str   # KiCad's ConvertToNewOverbarNotation
footprint_head_flags(fp_text: str) -> set          # bare words after the name: {'locked', 'placed'}
strip_bare_shape_locks(content: str) -> str        # (gr_line locked (start -> (gr_line (start
```

```python
from kicad_parser import (convert_to_new_overbar_notation, kiid_from_tstamp,
                          layer_list_tokens)

print(convert_to_new_overbar_notation('/~RST~'))   # /~{RST}
print(kiid_from_tstamp('5E3F1A2B'))                # 00000000-0000-0000-0000-00005e3f1a2b
print(layer_list_tokens(' *.Cu *.Mask)'))          # ['*.Cu', '*.Mask']
```

## Coordinate transformation

```python
local_to_global(fp_x, fp_y, fp_rotation_deg,
                pad_local_x, pad_local_y) -> Tuple[float, float]
```

Converts footprint-local pad coordinates to absolute board coordinates.
KiCad's rotation convention means the angle is **negated** inside the
standard rotation matrix — use this helper rather than rolling your own.

There is **no mirror term**, and that is correct rather than an omission: KiCad
stores a B-side footprint's children *pre-mirrored*, so a plain rotate and
translate resolves them. A consumer that adds a mirror of its own for B-side
parts double-applies it.

## Board side: `flip_layer_token`

```python
flip_layer_token(name: str) -> str
```

The other face's spelling of a layer token (#714): `F.SilkS` → `B.SilkS`,
`B.Cu` → `F.Cu`, and **anything else unchanged**.

A prefix rule, deliberately not a table — the knowledge is that KiCad spells a
sided layer `F.<x>` / `B.<x>`, not the list of which ones exist. Measured over
the 22 tracked boards, the layer tokens appearing inside a `(footprint ...)`
block are `F.Cu F.Mask F.Paste F.SilkS F.CrtYd F.Fab F.Adhes B.SilkS B.Fab B.Cu
B.CrtYd B.Mask B.Paste *.Cu *.Mask Dwgs.User Cmts.User Eco1.User Eco2.User
User.1`; this flips the thirteen sided ones and returns the other seven as they
are.

Two things it deliberately does **not** do, because both would be guesses:

- `*.Cu` and `*.Mask` pass through. They are KiCad's ALL-copper / all-mask sets
  and are already their own mirror; narrowing `*.Cu` to `B.Cu` would silently
  drop 66 pads on `rp2350_fpga_eensy_prePlane` U8 alone.
- Inner copper passes through unchanged, and that matches KiCad: probed
  against pcbnew 10.0.0 on a six-layer board, `FOOTPRINT::Flip` leaves a pad on
  `In1.Cu` and an `fp_line` on `In2.Cu` where they are. So the identity answer
  is correct here, not merely conservative. `placement.writer` nonetheless
  **refuses** a pad naming an explicit `In<n>.Cu`, for a different reason: no
  tracked board carries one, so nothing here would notice if a future KiCad
  began remapping them.

## Pad and copper helpers

```python
pad_is_plated_through(pad) -> bool      # drill > 0 and not NPTH: the barrel joins every copper layer
pad_drill_capsule(pad) -> ((x1, y1), (x2, y2), radius)   # the hole; a slot is a capsule
pad_drill_circles(pad, step=0.0) -> [(x, y, diameter)]  # the hole as circles along the slot
pad_is_aperture_only(pad) -> bool       # a paste/mask window: no copper, no drill, not NPTH
non_aperture_pads(fp) -> List[Pad]      # fp.pads without those windows
is_immutable_copper(item) -> bool       # KiCad-locked, or held by a sense: group
pour_zones(zones) -> List[Zone]         # every zone but the teardrops
```

Ask "does this pad connect layers" with `pad_is_plated_through`, never bare
`pad.drill > 0`: an NPTH mounting hole has a hole and no copper, even when it
carries a net (#328). The drill helpers work from the hole position
(`hole_x`/`hole_y`, else the pad centre) and follow a slot along its axis, so a
30 mm milled slot is not modelled as a 30 mm round hole. A pass that moves,
rewrites or removes existing copper tests `is_immutable_copper`, not
`locked` alone (#1279).

## Utility functions

```python
compare_pcb_data(from_board: PCBData, from_file: PCBData,
                 tolerance: float = 0.01) -> List[str]
```

Compares two parses (e.g. live-board vs file) and returns human-readable
difference strings; empty list means they match. Useful to validate
`build_pcb_data_from_board` against `parse_kicad_pcb` (this is what the GUI's
**Validate PCB Data** button runs). Coverage spans the fields the router
actually consumes: board info (layers, bounds, and the stackup layer by layer:
name, type, thickness, epsilon_r, loss tangent), nets, duplicate references,
footprint locks, sheet fields and net-tie groups, footprint and per-pad
geometry (position, size, `rect_rotation`, `roundrect_rratio`,
`local_clearance`, `zone_connection` (pad and footprint), drill, layers, and a custom or chamfered pad's `polygons`, compared as
the UNION each path describes -- pcbnew merges the primitives into one ring,
the text parser keeps one per primitive -- by `_polygon_union_excess`, both
ways), track and via geometry (matched as a multiset, not just counts; net 0
is the same net on both paths), zones, board outline/cutouts, keepout zones,
guide paths, the board's paste setup and via-protection policy, the paste
openings (paired within one signature by nearest bounds,
`_pair_nearest_bounds`, never by a sort of float coordinates), the unmeasured
graphic copper, and the sense branches.
Zones are compared per (net, layer) as a multiset of the teardrop
flag, the outline's and its cut-outs' vertices on the nm grid, the priority
and the island-removal semantics, so a pour and the teardrops that share its net and
layer cannot be paired with the wrong partner; a disagreement on the flag is
named as a `teardrop count` line (#1164, pinned by
`tests/gui_parity/test_1164_teardrop_parity.py`). Both fronts hold the
vertices pcbnew loads (arcs as KiCad's polyline, the closing vertex dropped),
so they agree to the nanometre or not at all.
On KiCad 10.0.0 the SWIG bindings do not wrap `BOARD_STACKUP`
(`GetStackupDescriptor()` returns an opaque pointer), so
`build_pcb_data_from_board` reads the stackup from the board FILE through
`extract_stackup`, the text parser's own reader, and the stackup comparison
cannot disagree there; it guards a build whose SWIG branch answers.

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
| `read_board_text(path, quiet=False)` | the file's text as KiCad reads it (teardrop-token repair, one-line reflow) — what `parse_kicad_pcb` hands the extractors |
| `extract_layers(content)` | `BoardInfo` (layers, bounds, stackup, outline, cutouts) |
| `extract_board_bounds(content)` | `(min_x, min_y, max_x, max_y)` from Edge.Cuts, or `None` |
| `extract_stackup(content)` | `List[StackupLayer]` |
| `extract_nets(content, kicad_version=0)` | `(Dict[int, Net], Dict[str, int])` — nets and name→id; the dialect is read from the content, `kicad_version` is kept for callers |
| `extract_footprints_and_pads(content, nets, name_to_id=None, duplicates=None, board_copper=None)` | `(Dict[str, Footprint], Dict[int, List[Pad]])` — `duplicates`, if given, is FILLED with the duplicate-reference counts; `board_copper` (the board's copper layers) puts a plated pad on all of them, as pcbnew does |
| `extract_groups(content, footprints)` | `Dict[str, List[str]]` — `PCBData.groups` |
| `iter_footprint_blocks(content)` | yields `(start, end, fp_text, raw_reference, key)` per block in FILE ORDER — the one place that decides what a block is CALLED |
| `disambiguate_references(raw_refs)` | ordered raw references -> ordered unique keys, by file-order ordinal |
| `footprint_raw_reference(fp_text)` | the name a block claims, before disambiguation |
| `extract_segments(content, name_to_id=None)` | `List[Segment]` |
| `extract_vias(content, name_to_id=None)` | `List[Via]` |
| `extract_zones(content, name_to_id=None)` | `List[Zone]` |
| `extract_keepouts(content)` | `List[dict]` (keepout rule areas) |
| `zone_outline_points(pts_text)` | the vertices of one zone / rule-area `(pts ...)` as pcbnew loads it (mm) |
| `kicad_poly_shape_points(pts_text)` | the vertices of a `gr_poly` / `fp_poly` `(pts ...)` as pcbnew holds them (mm; a polygon shape's chain is closed before its points arrive) |
| `kicad_arc_polyline(start, mid, end, max_error=ZONE_ARC_MAX_ERROR_NM)` | the polyline pcbnew holds for a three-point arc, integer nm |
| `extract_board_contours(content)` | `(outline, cutouts)` |
| `parse_guide_paths(content, layer)` | `List[GuidePath]` from a user layer |
| `parse_keepout_zones(content, layer)` | `List[GuidePath]` (closed only) |

## Gotchas

- **A reference is not unique, and the dict key is not always the reference**
  (#726). Five of the 22 boards this repo tracks carry two or more footprint
  blocks claiming one reference — on `watchy` they are real test points, on
  `glasgow_revC` there are seven `REF**`. Every block is an entry; the first
  keeps the bare name and later ones get a file-order ordinal (`TP4~2`), and a
  reference-LESS block is keyed `#<uuid>`. `pcb.duplicate_references` reports
  the board's own spelling. Consequences worth knowing:
  - `len(pcb.footprints)` is the BLOCK count, so it can exceed the number of
    distinct designators on the schematic.
  - `Pad.component_ref` carries the same key, so comparing two pads'
    `component_ref` is a genuine same-part test.
  - **A writer must locate blocks with `iter_footprint_blocks`, never by
    matching the Reference string.** One placement used to rewrite every block
    carrying the name; measured, that teleported a fiducial 23.44 mm onto its
    twin, and a write whose contract is that nothing moves relocated 3 blocks.
  - A `--lock`-style glob still behaves: `~` is not an fnmatch metacharacter,
    so `TP4*` covers both twins and `TP4` names the first.
- **`global_x/y` vs `local_x/y`**: global is after footprint rotation, local
  is before. Use globals for anything board-related.
- **Net ID 0** is "no net". Skip it when iterating nets to route, but
  remember its through-hole pads still block routing.
- **`pad.layers` may contain `'*.Cu'`** (through-hole pads). Expand with
  [`net_queries.expand_pad_layers`](api-net-analysis.md#expand_pad_layers)
  before comparing to routing layers.
- **A zone or rule-area outline can be drawn with arcs.** KiCad writes a
  rounded outline as `(arc (start ..) (mid ..) (end ..))` vertices among its
  `(xy ..)` points, keeps the arc, and hands every consumer its polyline.
  `extract_zones` and `extract_keepouts` build that same polyline
  (`kicad_arc_polyline`, a port of `SHAPE_ARC::ConvertToPolyline` as
  `SHAPE_LINE_CHAIN::Append` calls it in KiCad 10.0.0, max error
  `ZONE_ARC_MAX_ERROR_NM` = 1000 nm), so the CLI holds the GUI's vertices to
  the nanometre (`tests/gui_parity/test_zone_arc_outline_parity.py` holds
  it to pcbnew on random outlines -- radii 1 µm to 500 mm, both sweep
  directions, chained, mixed with `xy` points, as written and as pcbnew saves
  them -- and on degenerate arcs; `tests/test_zone_outline_points.py` pins
  vertices pcbnew produced). Before, only `(xy ..)` was read: an all-arc
  outline gave no points and its zone was dropped (key_ripper's `F&B.Cu` GND
  pour), a mixed one lost its arcs.
- **Outline-arc tessellation is KiCad-VERSION dependent, and the parser
  follows KiCad 10.** pcbnew polygonises an outline arc at
  `getArcPolygonizationMaxError()` = 1000 nm in KiCad 10, and at
  `ARC_HIGH_DEF` = 5000 nm in KiCad 9. The text parser cannot know which
  KiCad the GUI runs, so it uses `ZONE_ARC_MAX_ERROR_NM` = 1000 nm. On a
  KiCad 9 GUI, `build_pcb_data_from_board` therefore holds fewer vertices on
  every arc in a zone, rule-area or Edge.Cuts polygon outline, and
  `compare_pcb_data` reports a vertex-count difference on each of them: that
  is this known difference, not a parse bug, and the copper differs by under
  5 µm. (`gr_arc` / `gr_circle` / `gr_curve` board edges are not affected:
  both fronts tessellate those with the same function.)
- **An Edge.Cuts `gr_poly` / `fp_poly` is read with pcbnew's points**
  (`kicad_poly_shape_points`): its `(arc ..)` vertices as KiCad's polyline,
  and -- because KiCad closes a polygon shape's chain BEFORE its points
  arrive -- a last `xy` repeating the first point is kept (only an arc ending
  on the first point merges it), while a zero-length edge is skipped, as both
  fronts skip one from a `gr_line`. So a rectangle drawn as a polygon with its
  closing point repeated takes the plain-rectangle path (no ring; the bounds),
  as on the GUI and as in KiCad's own board outline (Castor_and_Pollux's
  faceplate used to chain into a 5-vertex ring on the CLI), and a polygon
  with an arc vertex is not lost (`tests/gui_parity/test_edge_poly_outline_parity.py`).
- **`board_info.stackup` is on `board_info`**, not on `PCBData` directly,
  and is an empty list when the board file has no stackup section. The
  impedance width solver then solves nothing (every layer gets the fallback
  width) and the delay functions assume Er 4.0 — see
  [Impedance](api-impedance.md#gotchas).
- **Rotated pad sizes are resolved for you** — `size_x`/`size_y` are already
  board-axis-aligned for orthogonal pads; for diagonal pads apply
  `rect_rotation` to the rectangle (don't re-derive a swap from `rotation`).
