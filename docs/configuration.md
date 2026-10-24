# Configuration Reference

This document describes all configuration options for the KiCad Grid Router.

## Command-Line Options

### Basic Usage

```bash
# Single-ended routing (all nets by default)
python3 py_router/route.py input.kicad_pcb [output.kicad_pcb] [OPTIONS]  # Default output: input_routed.kicad_pcb
python3 py_router/route.py input.kicad_pcb --overwrite [OPTIONS]         # Overwrite input file

# Differential pair routing (all nets by default)
python3 py_router/route_diff.py input.kicad_pcb [output.kicad_pcb] [OPTIONS]  # Default output: input_routed.kicad_pcb
python3 py_router/route_diff.py input.kicad_pcb --overwrite [OPTIONS]         # Overwrite input file
```

Use `route.py` for single-ended nets and `route_diff.py` for differential pairs. By default, all nets are routed. Use `--nets` to filter specific patterns. The output may also be named with `--output FILE`.

This page covers the options of `route.py` and `route_diff.py`, and the ones
every routing CLI shares (fab tier, escalation, DRC settings). The plane
CLIs' own options (`route_planes.py`, `repair_planes.py`) are in
[Power/Ground Planes](route-plane.md#command-line-options); the fanout CLIs' in
[`bga_fanout`](../py_router/bga_fanout/README.md) and
[`qfn_fanout`](../py_router/qfn_fanout/README.md).

`route.py` always writes the output file, even when nothing routes (no valid nets, or all already connected) — it writes an unchanged copy of the input in that case, so output→input pipelines don't break on a missing file. The exception is a step whose `--nets` holds a poured (zone) net: there "all already connected" is only the router's fill model, so the run carries on to the in-run plane finalize, which checks the pours against KiCad's exact fill and repairs them, and the end-of-run cleanup runs with it (#1112).

Pads with no pad number (paste/thermal-via artifacts KiCad doesn't netlist individually) are not used as routing targets; they remain copper obstacles.

### Net Selection

Net names support glob wildcards. Use `--nets` (or `-n`) to specify patterns:

```bash
# Exact net names
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"

# Wildcard patterns
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net-(U2A-DATA_*)"

# Multiple patterns
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net-(*CLK*)" "Net-(*DATA*)"

# Route all nets on a component (auto-excludes GND/VCC/VDD/unconnected)
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --component U1

# Several components at once, with fnmatch globs. A BARE reference is exact,
# so U1 does not also select U10; write U1* if that is what you want.
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --component U3 U4 "J1*"
```

`--component` takes one or more references and errors out (naming the offender,
with a did-you-mean) if any of them matches no footprint — routing the remaining
subset after a typo would quietly do the wrong thing. Because it accepts several
values it consumes the tokens after it, so pass the output via `--output` or put
`--component` last.

A pattern prefixed with `!` excludes the nets it matches (`--nets "*" "!GND"`);
a list of exclusions only implies `*`. A pattern without a sheet path (`GND`,
`VCC*`) also matches the last path component of a hierarchical net (`/GND`),
so the plain spelling works on a sheet-qualified board. A `!`-name that is
itself a net on the board is that net (an active-low signal), and `\!` escapes
the `!`. `unconnected-*` nets are left out of a wildcard; name one to route
it.

**Placement blocks** (`route.py` only) route the nets of one block -- a
schematic sheet, a KiCad group, an IC and its decaps:

| Option | Default | Description |
|--------|---------|-------------|
| `--group BLOCK` | - | Route only the nets of this placement block (e.g. `decap:U3`, `sheet:558c3023`). Composes with net patterns (both given: the intersection) |
| `--group-by SOURCES` | `auto` | How blocks are derived: a comma list of `kicad`, `sheet`, `netprefix`, `decap`; `auto` = `kicad,sheet` |
| `--group-scope` | `touching` (`internal` with `--undo`) | `touching` = every net with a pad in the block, its interface included (what `--component` means); `internal` = nets whose every pad is inside the block |
| `--list-groups` | off | List the blocks `--group-by` derives, with their net counts, and exit |

**Preview and undo** (`route.py` only):

| Option | Default | Description |
|--------|---------|-------------|
| `--preview` | off | Route, report what WOULD change, and write no output file |
| `--preview-png FILE` | - | With `--preview`, also render the board with the proposed copper highlighted (needs Pillow) |
| `--undo` | off | Remove the scoped nets' routed copper (tracks and vias, never a pour) instead of routing them. Needs an explicit scope (`--group`, `--component` or net patterns; pass `'*'` to mean every net). KiCad-locked copper and `sense:` group copper are refused; pad/target swaps are not inverted |

### Ripping Pre-Existing Routes

| Option | Default | Description |
|--------|---------|-------------|
| `--rip-existing-nets` | off (small nets only) | Net name patterns of **pre-existing** routed nets that may be ripped up and re-routed when they block a net being routed |

Without the flag the router may still rip **small** pre-existing nets: an
unprotected, unlocked, non-zone net with at most 30 segments and 6 vias that is
not `!`-negated in `--nets`. Such a rip is custody-backed -- the victim is
rerouted in the same run, restored, or at least keeps its escape stub -- and the
in-run plane finalize's pad repair may rip a signal net blocking a tap
(`KICAD_RIP_PREEXISTING=0` / `KICAD_FINALIZE_RIP=0` switch these off; see
[Rip-Up and Reroute](rip-up-reroute.md)). `--rip-existing-nets PATTERN` extends
the authority to the matching pre-existing routed nets whatever their size, so
the router may tear them up and re-route them when they block a net it is trying
to route (for example on a board already routed by a previous run). Use `'*'` to
allow any non-plane net.

```bash
# Let the router rip and re-route any pre-existing DATA net that gets in the way
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "*CLK*" --rip-existing-nets "*DATA*"

# Allow ripping any pre-existing (non-plane) net
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "*" --rip-existing-nets "*"
```

### Forcing a Re-Route

| Option | Default | Description |
|--------|---------|-------------|
| `--force-reroute` | off | Rip and re-route **from scratch** every net selected by `--nets`, even if it is already fully connected |

`--rip-existing-nets` only rips nets that *block* another route, so an
already-connected net named in `--nets` is normally skipped ("already fully
connected") — there is no way to make the router replace a
connected-but-unwanted route (e.g. one that squeaked through a corridor you
want cleared). `--force-reroute` is that way: every selected net has its
copper stripped and is replanned from scratch. In the GUI it is the
**Force re-route selected nets** checkbox on the Route tab.

Safety rails:

- Requires an explicit net scope (`--nets`, positional patterns, or
  `--component`) — without one the run is rejected rather than silently
  ripping the whole board.
- Protected nets (length-matched groups, routed diff pairs — see
  [Length Matching](length-matching.md)) are skipped unless named **exactly**
  (a glob is not an override); KiCad-**locked** copper is never ripped.
- Plane (zone-owning) nets are skipped — use `route_planes.py` for those.
- If the re-route produces no copper at all, the net's original copper is
  restored, so a failed replan never trades a working route for nothing.
- The net re-routes at its original dominant track width unless this run
  specifies one (same width-preservation as ripped pre-existing nets).

```bash
# Replace the existing /CLK route with a fresh from-scratch plan
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "/CLK" --force-reroute
```

### Geometry Options

| Option | Default | Description |
|--------|---------|-------------|
| `--track-width` | Default net-class track width (else 0.3) | Track width in mm. `route.py` ignores it when `--impedance` is given. `route_diff.py`: the pair's leg width, defaulting to the Default class's `diff_pair_width` (else 0.3); with `--impedance` it is the minimum width and the fallback when the board has no stackup |
| `--impedance` | - | Target single-ended impedance in ohms (calculates width per layer from stackup) |
| `--coplanar-gap` | 0.0 (off) | Declare that the impedance-controlled traces run through a ground pour on their own layer, this far (mm, trace edge to pour edge) from it: outer layers then use the coplanar-waveguide-over-ground model, which gives a narrower trace for the same ohms. Requires `--impedance`. The pour does not exist yet, so pour with a matching `route_planes.py --zone-clearance` and verify with `check_impedance.py --coplanar-gap` (see [Impedance API](api-impedance.md)). `route.py` and `route_diff.py` (where it applies to every pair in the call) |
| `--coplanar-nets` | (every net) | `route.py` only: limit `--coplanar-gap` to the nets these patterns match; the rest stay microstrip |
| `--clearance` | board's Default net-class clearance (else 0.25) | Copper clearance of the **Default net class** for this run, in mm; nets in other classes route at their own class clearance (pairwise `max`, as KiCad's DRC does). **Omitted** → the board's own Default class from the sibling `.kicad_pro`. Given below the board's Default class, the DRC writeback lowers the output project's Default class to it (it never raises a class). Since #530 this no longer caps the other classes; use `--net-clearances <json>` for explicit per-net values |
| `--net-clearances JSON` | (auto-read) | A JSON object mapping net name → that net's class clearance (mm), used as given instead of the map auto-read from the sibling `.kicad_pro`'s non-Default classes. `route.py`, `route_diff.py`, `route_planes.py`. See [cross-class clearance](api-routing-config.md#cross-class-clearance-kicad-maxclassa-classb) |
| `--clearance-ceiling` | - | Cap **every** net class (Default included) at this clearance for the run and clamp the output `.kicad_pro`'s classes down to it — the "stock net classes are aspirational" workflow that `--clearance` used to switch on implicitly (#439). **This is the flag for a chained run**: on a project an earlier step already lowered, the run stays at that lower floor instead of routing wider, which is what the recorded corpus chains and the routing skills use. GUI: the **Clearance ceiling** checkbox next to Min Clearance |
| `--via-size` | Default net-class via (else 0.5) | Via outer diameter in mm. **Given** → every net's vias are this size. **Omitted** → each net's vias are drawn at its **own** net class / `.kicad_dru` `via_diameter` (#530 decision 4): the router carries one via-legality map per distinct via geometry on the board (`grid_router` 0.22.0+; an older binary routes every net at the Default class size and says so) |
| `--via-drill` | Default net-class drill (else 0.3) | Via drill diameter in mm; per-net exactly like `--via-size` |
| `--grid-step` | 0.1 | Grid resolution in mm |
| `--same-net-pad-clearance` | the project's recorded value, else -1 | Edge-to-edge clearance (mm) between every via the run places and same-net pads (#581). `> 0` keeps vias off same-net SMD pads and off the net's solder-paste openings (#962), i.e. no via-in-pad, and is recorded in the sibling `.kicad_pro` so later chain steps inherit it; `-1` allows via-in-pad. On every routing CLI (`route.py`, `route_diff.py`, `route_planes.py`, `repair_planes.py`, `bga_fanout.py`, `qfn_fanout.py`) |

**Impedance-controlled routing:** When `--impedance` is specified, track widths are automatically calculated per layer using IPC-2141 formulas based on the board stackup. Outer layers use microstrip formulas (typically wider tracks) and inner layers use stripline formulas (typically narrower tracks). Via clearance calculations account for the varying track widths per layer.

### Fab Tier Options

The **fab tier** is the JLCPCB manufacturing floor every routing step shrinks tracks,
vias and clearances *down toward* when it needs to. It is shared by every CLI
(`route.py`, `route_diff.py`, `route_planes.py`, `repair_planes.py`,
`bga_fanout.py`, `qfn_fanout.py`, `check_drc.py`, `fix_kicad_drc_settings.py`,
`list_nets.py`, `py_tools/check_reachability.py`) and the GUI (one selector on
the Route tab). The defaults (`auto`, `fab`) live in `routing_defaults.py`
(`FAB_TIER`, `ESCALATION`). Values are sourced from
[jlcpcb.com/capabilities](https://jlcpcb.com/capabilities).

| Option | Default | Description |
|--------|---------|-------------|
| `--fab-tier` | `auto` | `auto` (the standard floor, escalating to advanced when a fine-pitch fan-out, plane tap or last-resort via cannot fit; warned and counted), `standard` (no extra fab cost, **hard**) or `advanced` (tighter, "more costly", **hard**) |
| `--fab-overrides` | - | Path to a file overlaying the tier's floors (only the keys it lists) |
| `--escalation` | `fab` | How far below a **requested** size a failing net may be retried: `fab` (down to the fab tier floor, below the board's own minimums; completion first, every narrowing disclosed), `board` (down to the board's own Board Setup minimums; an unset key falls back to the fab tier floor, except clearance, which falls back to the Default net class's clearance, #1160; with `--no-fix-drc-settings` descents stop at what KiCad grades the unchanged project at: a rule it leaves out at KiCad's factory value, #1287, and clearance at the Default class floored at `min_clearance`, #1262), `off` (never; the net fails and is reported -- except a wide power/impedance net's pad-entry neck of at most 2 mm of path, `routing_defaults.PAD_ENTRY_NECK_MM`) |
| `--strict-sizes` | off | Exit 3 when any feature was delivered below its requested size or a fab-tier escalation fired -- on every routing CLI that takes it (`route.py`, `route_diff.py`, `route_planes.py`, `repair_planes.py`, `bga_fanout.py`, `qfn_fanout.py`); an exit code the run already had for its own reason wins. For a net with its own width -- `--power-nets`, an impedance width, a wider net class -- this means copper that SHIPS under width (one `design_rules` row per net), not every narrower routing attempt: a power net on `route.py` is measured over the whole net (#1033); an impedance- or class-width net, and `repair_planes.py`'s `--power-nets` taps, over the copper the run laid, so a fanout stub an earlier step drew at its own width is not counted |

The tier is a **floor ladder**:

- **`standard`** — the cheap floor (per layer count: 2-layer track/clearance 0.127/0.127,
  4+ layer 0.0889/0.10; via 0.45 / drill 0.20; via hole-to-hole 0.20). A **hard**
  floor since #857: the router never descends below it on its own (an explicit
  request below it is honoured, see *Floor enforcement* below).
- **`advanced`** — the JLC "more costly" floor (track/clearance 0.10/0.10 on 2-layer,
  0.0762/0.09 on 4+; via 0.25 / drill 0.15). A **hard** floor.
- **`auto`** — the default: `standard` first, **escalating to `advanced`** (one
  warning line per context, and a count in the run summary) when a fine-pitch fan-out,
  a plane tap or a last-resort via genuinely cannot fit at the standard floor. On a
  4+ layer board a 0.30 / 0.15 via is tried between the two. This
  is what maximises completion; what #857 changed is the disclosure — the escalation
  used to change what the board costs without saying so anywhere a harness could see
  it (149 silent escalations on one board), and now every one is counted and named.
  Pass `standard` or `advanced` for a hard floor that never escalates.

**Escalation policy** (`--escalation`, the same choice on the GUI's Route tab) is
orthogonal to the tier and bounds every place the engine narrows a track, shrinks a
via or tightens a clearance to complete a net: the per-net rescue, the terminal
escalation, the terminal graze neck, fine-pitch plane taps, via-in-pad clamps and
last-resort vias. Under the default `fab`, a descent may go down to the fab tier
floor, below the board's own declared minimums (the writeback then lowers the
project's `rules.min_*` to match, as before); this is the completion-first setting.
Under `board`, a descent stops at the board's own `rules.min_*` (Board Setup >
Constraints); a minimum the board leaves unset (KiCad writes 0) falls back to the
fab tier floor for that key, except `min_clearance`, which falls back to the
Default class (#1160). Non-Default nets never descend below their own class.
Whether KiCad then accepts the copper depends on the DRC-settings writeback:

- **With it** (the default), a clearance descent below the Default class lowers
  the OUTPUT project's Default class to the clearance used and says so, so the
  board is DRC-clean against the project it ships with. It is not clean against
  the INPUT project when `min_clearance` sits below the Default class: the input
  still grades Default nets at the class.
- **Without it** (`--no-fix-drc-settings`, or the GUI's "Fix DRC settings after
  routing" unticked) the project is left as it was, so every descent stops at
  what KiCad grades the input project at. A rule the project leaves out is
  graded at KiCad's factory value, which is what KiCad loads for it (KiCad 10:
  track 0.2, via 0.5 / drill 0.3, annular 0.1, hole-to-hole 0.25,
  copper-to-edge 0.5; `fab_tiers.KICAD_FACTORY_RULES`, #1287). A declared 0 is
  KiCad's "no check" and still falls back to the fab tier floor. Clearance
  descents stop at the clearance KiCad grades a Default net at: the Default
  class, floored at `min_clearance` (#1262). A run whose own request is
  already below one of these floors (an explicit `--clearance`,
  `--clearance-ceiling` or `--track-width`) keeps the request as that key's
  floor and descends nothing of that key.

Either way a `.kicad_dru` layer rule or a pad clearance override replaces the
base clearance where it applies, as KiCad does. A board minimum bounds descents only
when the run's own request respects it: with the writeback, a request already
below the declared minimum (the stock 0.5 mm via on a project nobody edited,
routed with `--via-size 0.3`) marks that minimum as stale for the run, is said
so on the console, and descents for that key bound at the fab floor instead.
Without the writeback the request is that key's floor, as above. An explicit
request is never pinned up to a board minimum, only to the fab floor. Under
`off` nothing is narrowed and a net that cannot complete at the requested
geometry fails and is reported, with one bounded exception: a wide (power or
impedance) net may neck below its width for at most 2 mm of routed path
(`routing_defaults.PAD_ENTRY_NECK_MM`) on one edge or one via-to-pad tap --
the pad-entry stretch a wide rail cannot keep into a narrow pin. A longer neck
fails the edge. A power net's under-width copper is recorded like any other
narrowing (below), so `--strict-sizes` still sees it.

**Net-filtered keep-outs.** A KiCad rule area has no net dimension, so a
region only some nets may cross is a NAMED rule area plus a `.kicad_dru` rule
whose condition names it, e.g.
`(rule "corridor" (constraint disallow track via) (condition "A.intersectsArea('corridor') && A.NetName != 'GND'"))`.
Every routing step reads such rules from the board's own `.kicad_dru` (no flag,
no GUI control: as for the per-layer rules, the rules file is the single source
of truth) and keeps each net the rule binds out of the area while that net
routes; the nets it exempts route through. The bound nets are the rule's own
condition evaluated per net (net name, net class, item type, layer), separately
for tracks (`track`) and vias (`via`, `through_via`); a diff pair is bound when
either member is, and its members' copper, not just its centreline, is kept out.
`insideArea`, `intersectsArea` and `enclosedByArea` are read (the last as
`intersectsArea`, which keeps out more than KiCad requires). The console says
`.kicad_dru rule 'corridor': honoured as a net-filtered keep-out: N of M net(s)
kept out of 'corridor' (tracks, vias)`. A rule the tool cannot model completely
-- one with another constraint beside `disallow`, one that forbids an item
outside every named area (`!A.intersectsArea(...)`), one that combines areas
other than as a union (`a && b`), or one naming no area the board has -- stays
`NOT honoured` and is listed in `design_rules.unsupported_rules`; nothing of it
is applied. The run ends with an audit of the board it ships: bound-net copper
inside a region (from a path that does not stamp the region, such as the plane
finalize's taps and joins, or already in the input) is printed and reported as
`net_keepouts` in `JSON_SUMMARY` / `--json-out` and the GUI's results (`[]` =
clean; absent when the board has no such rule).

**Disclosure.** Every descent is recorded and reported three ways: the
`JSON_SUMMARY` block `design_rules` (policy, tier, the board floors read, every
narrowing with net / kind / requested / delivered / site, the fab-tier escalation
count, and the `.kicad_dru` rules the tool could not honour), an `escalations`
count in `JSON_SUMMARY_MIN`, and one end-of-run line (`Design rules [--escalation
fab, --fab-tier auto]: 7 feature(s) on 3 net(s) delivered below the requested
size (...)`). `--strict-sizes` makes that line a non-zero exit. Every routing CLI
prints the line and honours the flag; the non-route CLIs add `design_rules` to their
`JSON_SUMMARY` only when the run narrowed something, and the GUI's diff, planes and
fanout tabs print the same line in their log.

| Floor (per layer count) | standard | advanced |
|---|---|---|
| via diameter / drill | 0.45 / 0.20 | 0.25 / 0.15 |
| track / clearance (2-layer) | 0.127 / 0.127 | 0.10 / 0.10 |
| track / clearance (4+ layer) | 0.0889 / 0.10 | 0.0762 / 0.09 |
| via hole-to-hole / pad hole-to-hole | 0.20 / 0.45 | 0.20 / 0.45 |
| copper to board edge | 0.20 | 0.20 |

**Override file** (`--fab-overrides`) — a plain, human-editable file overlaying the
selected tier. Only the floor values it lists change; the rest come from the base tier
(`standard` under `auto`). Supplying one **disables escalation** (the floor becomes
exactly *base tier + file*), since the file states your exact fab limits. One
`key = value` (or `key: value`, or `key value`) per line; `#` starts a comment; keys
are `track_width`, `clearance`, `via_diameter`, `via_drill`, `hole_to_hole`,
`pad_hole_to_hole`, `annular`, `board_edge`:

```
# my_fab.txt — JLC + a tighter drill I've confirmed
via_drill = 0.15
clearance = 0.09
```

A ready-to-copy, fully-commented template listing every key and the built-in tier
values lives at **[`fab_overrides.example.txt`](../fab_overrides.example.txt)** in the
repo root. (`track_width` / `clearance` are layer-dependent in the tier tables, but an
override sets one fixed value for every board — override them only if you want that.)

```bash
# The defaults (--fab-tier auto --escalation fab): the standard floor, escalating to
# the advanced via where a fan-out, plane tap or last-resort via cannot fit; descents
# may go below the board's own minimums, and every one is disclosed
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net*"

# A hard cheap floor; descents stop at the board's own Board Setup minimums
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --fab-tier standard --escalation board

# Opt the whole board into the tighter, more-costly floor
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --fab-tier advanced

# Never narrow anything; fail the net instead, and make that a non-zero exit
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --escalation off --strict-sizes

# Declare your own fab capability
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --fab-overrides my_fab.txt
```

**Floor enforcement.** An explicit `--track-width`, `--clearance`, `--via-size`,
`--via-drill` or `--hole-to-hole-clearance` is checked against the **physical** fab
floor (the standard tier with the `--fab-overrides` file overlaid when one is given,
else the advanced rung: 0.25/0.15 via, track/clearance 0.10/0.10 on 2-layer and
0.0762/0.09 on 4+) and **pinned** up to it with a warning when set below.
The selected tier and the board's own minimums bound what the router may descend to
on its own, never an explicit request: `--via-size 0.3` under the hard `standard` tier
is routed at 0.3 as asked (no automatic descent below it), exactly as a request below a
stock Board Setup minimum is. The GUI pins the corresponding Basic-tab spin control the
same way. Grade verification (`check_drc.py`) grades sizes at the board's own minimums
and rules (what KiCad grades) raised to the same physical floor.

### Post-Route DRC Settings

As their final step, the routing CLIs (`route.py`, `route_diff.py`,
`route_planes.py`, `repair_planes.py`, `bga_fanout.py`, `qfn_fanout.py`) rewrite the output's sibling
`.kicad_pro` so KiCad's Board Setup floors match the clearances/sizes just routed
— a manual DRC in KiCad then flags only genuine problems instead of stock-default
noise (issue #160). The [DRC Settings Fixer](utilities.md#drc-settings-fixer-fix_kicad_drc_settingspy)
does the work; these flags tune it. The GUI plugin applies the equivalent on the
live board via the pcbnew API.

| Option | Default | Description |
|--------|---------|-------------|
| `--no-fix-drc-settings` | off (fix is on) | Do **not** adjust the output's `.kicad_pro` DRC constraints; it ships with the input project's floors as authored. route.py's in-run floor sync (#650) stands down too, so its in-run audits grade the pours at those floors (#1285) |
| `--relax-drc-severities` | off | ALSO lower the project's DRC severities for the non-routing categories (courtyard shapes, solder-mask bridges, footprint/library issues incl. `annular_width` → ignore; `starved_thermal`, `courtyards_overlap` → warning). Off by default (#856): a routing step never changes what the project counts as a violation unless asked; the previous values are kept under `kicad_routing_tools.saved_severities` |
| `--keep-thermal` | off | Deprecated no-op (with `--relax-drc-severities`, leaves `starved_thermal` untouched) |
| `--enable-used-layers` | off | Add any layer the board uses but is missing from its `(layers)` table back into the `.kicad_pcb`, so KiCad stops flagging `item_on_disabled_layer`. Off by default because it edits the board, not just DRC settings |

### Power Net Options

| Option | Default | Description |
|--------|---------|-------------|
| `--power-nets` | - | Glob patterns for power nets (e.g., `"*GND*" "*VCC*"`) |
| `--power-nets-widths` | - | Track widths in mm for each power-net pattern (must match `--power-nets` length) |
| `--neckdown-length` | 2.5 | Length in mm of the pad-entry zone at each pad end of a neck-down route (a failed wide power route retried at the default width): the power width capped at the pad's short side where it fits, the default width only where it does not (#1318) |
| `--neckdown-taper-length` | 0.5 | Narrow-to-wide width taper length in mm (0 = abrupt) |
| `--no-power-tap-neckdown` | off | Disable the neck-down retry of failed wide power routes |

```bash
# Route with wider tracks for power nets
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "Net*" \
  --power-nets "*GND*" "*VCC*" "+3.3V" \
  --power-nets-widths 0.4 0.5 0.3
  # other nets route at --track-width (default: the board's Default class, else 0.3)
```

Patterns are matched in order - first match determines width; a power width below `--track-width` is raised to it. Obstacle clearances automatically adjust for wider power traces. `route_planes.py` and `repair_planes.py` take the same two options for the nets they re-route.

See [Power Net Analysis](power-nets.md) for automatic detection, AI-powered analysis, and IPC-2221 track width guidelines.

### Algorithm Options

| Option | Default | Description |
|--------|---------|-------------|
| `--via-cost` | 75 | Via penalty in 0.1mm grid steps, i.e. 75 = 7.5mm of path; mm-equivalent at any `--grid-step` (effectively doubled for diff pairs since two vias are placed) |
| `--max-iterations` | 200000 | A* iteration limit per route |
| `--max-probe-iterations` | 5000 | Quick probe per direction to detect stuck routes |
| `--heuristic-weight` | 2.3 | A* greediness (>1 = faster, <1 = more optimal). 2.3 is the corpus dose-response peak (#586); 1.9 was the default before it |
| `--turn-cost` | 1000 | Penalty for direction changes (encourages straighter paths) |
| `--direction-preference-cost` | 250 | Penalty for a move off the layer's preferred direction (layers alternate horizontal / vertical, horizontal on top; 0 = off). `route.py`, `route_diff.py` |
| `--max-ripup` | 3 | Max blockers to rip up at once during rip-up and retry |
| `--quick [LEVEL]` | 0 (off) | Route to judge, not to ship. `--quick` (level 1), a placement review's first round: the main pass only, without the net rescue, the terminal escalation, the plane finalize and the reconciliation laps; what stays open and what costs most names what to move (`py_tools/route_review.py`). `--quick 2`, the seed judge: also skips every pass that only polishes copper for shipping (the cleanup's graze, cycle and neck passes, smoothing, the seam re-asks, the plane-fragility field, the late collapse and sweeps, the final re-grade, the improvement gate, the via-protection stamp), about half the time of level 1. `route_diff.py --quick 2` skips the cleanup polish, the seam re-asks and the plane-fragility field; level 1 changes nothing there. The board is a diagnosis, not one to ship, and `route.py`'s JSON_SUMMARY says so (`quick`: level and skipped stages) |
| `--ripup-abandon-metric` | `stranded` | `route.py`: keep-retry vs abandon rule for multipoint tap rip-ups (see [rip-up-reroute.md](rip-up-reroute.md#abandon-metrics)) |
| `--ripup-blocker-select` | `count` | Which blocker the rip-up ladder rips first: `count`, `near-target`, `bidir`, `mincut` or `cost` (see [rip-up-reroute.md](rip-up-reroute.md#blocker-selection-algorithms)). `route.py`, `route_diff.py`, `route_planes.py`, `repair_planes.py` |
| `--routing-clearance-margin` | 1.0 | Diff pairs only (a pair the run routes or restores): multiplier on the track-to-via distance that sets the P/N via offset and the centerline's via keep-out (1.0 = minimum DRC). Single-ended tracks and vias do not read it |
| `--hole-to-hole-clearance` | board's `min_hole_to_hole` (else 0.20) | Minimum drill hole edge-to-edge clearance (mm). Omitted → the board's own constraint minimum (#439) |
| `--board-edge-clearance` | board's `min_copper_edge_clearance` (else 0.0) | Clearance from board edge in mm. Omitted → the board's own constraint minimum (#439) |
| `--proximity-heuristic-factor` | 0.02 | Factor for proximity-aware A* heuristic (higher = faster but may find suboptimal paths, 0 = disabled) |
| `--ripped-route-avoidance-radius` | 1.0 | Radius around ripped route corridors to apply soft penalty (mm) |
| `--ripped-route-avoidance-cost` | 0.1 | Cost for other nets routing through a ripped net's former corridor, reserving it for that net's reroute (0 = disabled) |

See [Rip-Up and Reroute](rip-up-reroute.md) for how failed routes trigger rip-up, how blockers are identified and ranked, and how ripped nets are rerouted.

### Routing Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering` / `-o` | mps | Net ordering: `mps`, `inside_out`, or `original`; `route.py` also takes `bus` (detected bus groups first, members middle-out, the rest by mps; ordering only, corridor attraction still needs `--bus`). See [Net Ordering](net-ordering.md) |
| `--direction` / `-d` | forward | Direction: `forward` or `backward` |
| `--layers` / `-l` | all copper layers | Routing layers. For `route.py` the default is all of the board's copper layers; `route_diff.py` and `bga_fanout.py` default to `F.Cu B.Cu` |
| `--layer-costs` | (see below) | Per-layer cost multipliers (1.0-1000; a negative value forbids track on that layer, which stays an obstacle vias may span), one per `--layers` entry (any other count is refused, see [Layer Costs](#layer-costs)). `route.py` default: all 1.0 when routing 4+ layers, else F.Cu=1.0 and the others 3.0; `route_diff.py` default: all 1.0 |
| `--no-bga-zones [REFS...]` | (auto-detect) | Disable BGA exclusion zones. No args = all. With refs (U1 U3) = only those |
| `--no-crossing-layer-check` | off | Count crossings between routes whatever their layers (by default only same-layer crossings count) |
| `--mps-reverse-rounds` | off | Route the most-conflicting MPS groups first (instead of least) |
| `--mps-layer-swap` | off | MPS-aware layer swaps to reduce crossing conflicts |
| `--mps-segment-intersection` | auto | Force segment intersection for MPS crossing detection (auto-enabled when no nets are on BGAs) |
| `--swappable-nets` | - | Glob patterns for nets whose targets may be swapped (e.g. `'*DATA_*'`, `'rx1_*'`) |
| `--crossing-penalty` | 1000.0 | Penalty for crossing assignments in target swap optimization |
| `--schematic-dir` | - | Directory of `.kicad_sch` files to update with the pad swaps made (default: no schematic update) |
| `--smoothing` / `--no-smoothing` | on | `route.py`: collapse grid-A* staircase micro-jogs into octolinear shortcuts (#536). `KICAD_SMOOTH_ROUTE=0/1` overrides either way |
| `--keep-input-copper` | off | Treat the input file's own copper as read-only in the post-route cleanup: no pass removes or rewrites it, only this run's copper, including stubs of nets this run failed to route. For chains whose earlier steps author copper (fanout stubs, hand routes) a later step must see verbatim. Off: dead input stubs on in-scope nets are swept (#84) |

The options above are on both `route.py` and `route_diff.py` unless marked.

### Proximity Penalty Options

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | 2.0 | Radius around stubs to penalize (mm) |
| `--stub-proximity-cost` | 0.2 | Cost penalty at stub center (mm equivalent) |
| `--via-proximity-cost` | 10.0 | Via cost multiplier in stub/BGA proximity zones (0 = no extra cost) |
| `--bga-proximity-radius` | 7.0 | Radius around BGA edges to penalize (mm) |
| `--bga-proximity-cost` | 0.2 | Cost penalty at BGA edge (mm equivalent) |
| `--track-proximity-distance` | 2.0 | Radius around routed tracks to penalize on same layer (mm) |
| `--track-proximity-cost` | 0.0 | Cost penalty near routed tracks (0 = disabled) |
| `--keep-away AGG:VICTIM:GAP` | (off) | Soft keep-away between two net groups (#1146), repeatable. While a net of one side routes, cells where its track would sit closer than GAP mm (edge to edge, same layer) to the other side's copper cost `--keep-away-cost`. Each side is comma-separated net patterns as in `--nets` and/or net classes as `class=NAME` (`!class=NAME` takes one out), e.g. `class=Digital:class=Audio:0.5`. `route.py` and `route_diff.py`. The board's design brief may declare rules too (its `keep_away` list, #1253); they are added to these with no flag. See [Keep-away](api-routing-config.md#pairwise-keep-away-keep_away--keep_away_free--keep_away_cost) |
| `--keep-away-free` | 1.5 | Within this distance of the routed net's own pads the keep-away band is not priced (mm) |
| `--keep-away-cost` | 0.5 | Cost per cell inside a keep-away band (mm equivalent; 0 = measure and report only) |
| `--vertical-attraction-radius` | 1.0 | Radius for cross-layer track attraction (mm) |
| `--vertical-attraction-cost` | 0.0 | Cost bonus for aligning with tracks on other layers (0 = disabled) |

### Bus Routing Options

Detects groups of nets with clustered endpoints (buses), plans a corridor with room for each group, and routes the group first and middle-out as parallel tracks, each net attracted to the planned corridor (or, where no corridor was planned, to its already-routed neighbor). See [Bus Routing](bus-routing.md) for the detection algorithm, the strict geometric filter, corridor planning, and tuning guidance.

| Option | Default | Description |
|--------|---------|-------------|
| `--bus` | off | Enable bus detection and routing |
| `--bus-detection-radius` | 5.0 | Max endpoint distance for nets to form a bus (mm) |
| `--bus-min-nets` | 2 | Minimum number of nets for a raw clique (the strict geometric filter then keeps a group only with 3 or more) |
| `--bus-attraction-radius` | 5.0 | Reach of the attraction around the corridor or neighbor path (mm) |
| `--bus-attraction-bonus` | 5000 | Attraction strength: `bonus/50` percent off a step's cost, capped at 90% (the default is at the cap) |

### Guide Corridor Options (preferred route)

Draw a polyline on a User layer in KiCad, then enable `--guide-corridor` to route the
selected nets so they **follow** that line. The router takes the polyline's vertices as
waypoints (`--guide-corridor-spacing > 0` subdivides long segments for tighter curves) and
routes source → waypoints → target. It keeps the route on a single layer where it can — a
waypoint it can't reach cleanly on the current layer is skipped rather than forcing a layer
change, so a corridor doesn't add vias the direct route wouldn't need. It applies to all nets
routed in the run and works in either draw direction; multiple nets following the same
corridor pack alongside each other without overlapping. Supported by `route.py` and the
plugin ("Follow User-layer guide path" checkbox); not currently supported by `route_diff.py`
or `route_planes.py`.

| Option | Default | Description |
|--------|---------|-------------|
| `--guide-corridor` | off | Route selected nets to follow a user-drawn guide polyline |
| `--guide-corridor-layer` | User.1 | User layer the guide polyline is drawn on |
| `--guide-corridor-spacing` | 0.0 | Max mm between waypoints. `0` uses only the drawn segment endpoints (vertices); `>0` subdivides long segments to follow curves more tightly |

```bash
# Route two nets so they follow a corridor drawn on User.1
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net-(A)" "Net-(B)" --guide-corridor
```

Notes:
- **Endpoints/topology are untouched.** The waypoints only steer the path *between* the
  pads the router already chose to connect. For a multi-point net the existing MST is kept,
  and each waypoint steers the MST segment it is nearest to (so the net follows the whole
  drawn line across its topology); a waypoint used by one segment is not reused by others.
- **A corridor never makes a route fail.** It is strictly best-effort: a waypoint that
  can't be reached (or that would strand the route) is dropped, and if no waypoints can be
  followed the segment routes directly — identical to routing with no corridor. With the
  flag off, routing is byte-for-byte unchanged.
- Routes are placed one at a time; each becomes an obstacle for the next, which keeps
  multiple corridor nets from overlapping. Draw the corridor wide enough (or use separate
  lines) if you want several nets to follow it comfortably.
- The guide applies to every net routed in the run, so select only the nets you want it to
  steer. Members of a detected bus (`--bus`) whose group clustered at its targets route in one
  direction under the bus attraction and skip the guide.
- The route is built by routing A* legs between the waypoints and concatenating them. A
  multi-point tap or a detour can cross an earlier leg of the **same** net. KiCad allows
  same-net copper to overlap, so the net stays connected and DRC-clean against other nets,
  but the trace may be less tidy than a hand-route.
- Tested by `tests/test_guide_corridor.py` — follow, blocked-waypoint snap, shared-corridor
  non-overlap, flag-off regression, never-blocks-routing, spacing subdivision, and a real
  bundled board (`kicad_files/lvds_converter_dualclk.kicad_pcb`) with a `User.1` guide.

### Keepout Zone Options

Draw one or more closed polygons (`gr_poly`) or rectangles (`gr_rect`) on a User layer in KiCad,
then enable `--keepout` to keep routed tracks **out** of those areas. This is a **hard** keepout:
routes (and vias) cannot enter any of the polygons on any copper layer. It applies to every net
routed in the run.

| Option | Default | Description |
|--------|---------|-------------|
| `--keepout` | off | Keep routed tracks out of polygons drawn on a User layer |
| `--keepout-layer` | User.2 | User layer the keepout polygons are drawn on |

```bash
# Route a net so it avoids keepout polygons drawn on User.2
python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets "Net-(A)" --keepout
```

Notes:
- All closed polygons (`gr_poly`) and rectangles (`gr_rect`) found on the keepout layer are
  blocked — draw as many as you need. Use the polygon or rectangle tool, not the line tool: open
  lines (`gr_line`) don't bound an area and are ignored.
- The default keepout layer is `User.2`, distinct from the guide-corridor layer (`User.1`), so
  you can use both features at once.
- **Coverage:** keepouts are respected across single-ended, multipoint, and **differential-pair**
  routing (`route_diff.py --keepout`), and across plane via-stitching and plane repair. KiCad
  native keep-out rule areas (`(zone … (keepout …))`) are honoured automatically everywhere too,
  with no flag. (Guide corridors, by contrast, steer single-ended/multipoint routing only.)
- It's a non-copper User layer, so the polygon never affects manufacturing — it only steers the
  router.
- **Don't draw a zone over a pad you need to route.** Cells inside the polygon are blocked
  unconditionally, so a zone covering a routed net's pad can make that net unroutable. Zones are
  meant for open board area.
- Unlike a guide corridor (which is best-effort), a keepout is absolute — if a zone walls off the
  only path to a pad, that net will fail to route. With the flag off, routing is unchanged.
- **Plugin only:** the Route tab has optional "Clear guide layer after routing" / "Clear keepout
  layer after routing" checkboxes (unchecked by default). When ticked, a *successful* route deletes
  the drawn guide/keepout graphics from that User layer so you can draw fresh ones — it only acts
  for the feature it pairs with, and never runs if nothing routed.
- Tested by `tests/test_keepout.py` — flag-off regression, single-net detour, multi-net avoidance,
  and a real bundled board (`kicad_files/lvds_converter_dualclk.kicad_pcb`).

### Kelvin Sense Branches (board-declared, no option)

A Kelvin sense trace (a regulator's VOS/feedback divider, a shunt's sense
pair) is on the same net as the load current it measures, so the router would
otherwise connect the net's loads to it anywhere: connected, DRC-clean and
electrically wrong. Declare it in the board with a KiCad **group** named
`sense:<REF>.<PAD>`, where `REF.PAD` is the join (star) pad, holding the sense
tracks, arcs and vias:

```
(group "sense:C108.1" (uuid ...) (members "<track uuid>" "<via uuid>" ...))
```

In KiCad: select the sense copper, Group it, and name the group. The sense pads
are the pads of the same net that member copper touches, the join pad excepted
-- touching the way KiCad's connectivity does: a track across a pad off-centre
touches it, so do two tracks crossing in an X, and a blind or buried via touches
only on the layers it spans. Pads are their real copper (a round pad is a
circle). A member uuid that matches no item is ignored, as KiCad drops it on
load, and named in the entry's `warnings`.
There is no CLI flag and no GUI control: the board file is the declaration, so
every step of a chain, both front-ends and every leg that parses the board read
it (`sense_branch.py`). For the branch's own net the member copper and the
sense pads are foreign copper, at the net's clearance, everywhere except on the
join pad: only new copper sticking out of the join pad is held to the
clearance, so a join pad no wider than the track (an 0402 cap under a power
track) is landed at its centre without narrowing anything; the endpoint derivations never offer them as a place to attach, plane
pad taps skip the sense pads, and the member copper is held immutable
(`held_by`): no pass moves, rewrites or removes it, even when the branch is
refused. The hold is not a KiCad lock and is never written to the board. The
net's routes of this run stay rippable like any other net's; only a whole-net
strip of the net's PRE-EXISTING copper (`--rip-existing-nets`,
`--force-reroute`, plane-repair blocker rips, undo) is refused, with the
protection reason `sense:<REF>.<PAD>` and no exact-name override -- ungroup or
rename the group to release it. Net ids are never changed.

Each branch is printed and reported in `JSON_SUMMARY.sense_branches`
(`route.py` and `route_diff.py`), as the input resolved it; a branch the input
protected and the routed copper no longer does also carries `tapped_by_run`,
with a printed WARNING. A branch is reported and left unprotected,
with the reason, when the group name is not `sense:REF.PAD`, the join pad is
missing or has no net, a member is not a track or via on the board, member
copper is on another net, member copper does not touch the join pad, a sense
pad or member is not joined to the join pad by member copper yet, other copper
of the net already touches the branch off the join pad, or a pour of the same
net can fill over it off the join pad (the fill already taps it). A pour's
cut-outs and copper-pour keep-out rule areas are not fill: a pour cut around
the sense trace leaves the branch protected. KiCad's DRC cannot
see topology, so check the written board.

`route_planes.py` and the Planes tab print a WARNING naming each protected
branch a pour they create covers, and `route_planes.py` records the layers as
`poured_over` in its `JSON_SUMMARY.sense_branches`; `repair_planes.py` reports
the branches judged on the board it wrote.

### Differential Pair Options (route_diff.py only)

These options are only available in `route_diff.py`. All nets passed to route_diff.py are treated as differential pairs.

| Option | Default | Description |
|--------|---------|-------------|
| `--impedance` | - | Target differential impedance in ohms (calculates width per layer from stackup) |
| `--diff-pair-gap` | Default net-class `diff_pair_gap` (else 0.101) | Gap between P and N traces (mm), also used as spacing for impedance calculation |
| `--diff-pair-centerline-setback` | 2x P-N dist | Distance in front of stubs to start centerline (mm) |
| `--min-turning-radius` | 0.2 | Minimum turning radius for pose-based routing (mm) |
| `--max-turn-angle` | 180 | Max cumulative turn angle (degrees) to prevent U-turns |
| `--max-setback-angle` | 45.0 | Maximum angle for setback position search (degrees) |
| `--polarity-swap-nets` | (none = deny all) | Glob patterns naming pairs allowed to polarity-swap; `'*'` = all (#279) |
| `--no-gnd-vias` | false | Disable GND via placement near signal vias |
| `--diff-chamfer-extra` | 1.5 | Chamfer multiplier for diff pair meanders (>1 avoids P/N crossings) |
| `--diff-pair-intra-match` | false | Match P/N lengths within each diff pair |
| `--ac-couple-match` | false | End-to-end length-match AC-coupled pairs split by series DC-blocking caps (#196): the cap chain is found, the concatenated P path is matched against the N path, and the meanders go on whichever segment has room |

`--track-width` is the pair's leg width here (see [Geometry Options](#geometry-options)).
The ordering, swap and MPS options in [Routing Strategy Options](#routing-strategy-options)
apply to `route_diff.py` too.

### Layer Optimization Options

These options control stub layer switching, which moves stubs to different layers before routing to avoid vias. Works for differential pairs, single-ended nets, and multi-point nets (3+ endpoints).

| Option | Default | Description |
|--------|---------|-------------|
| `--no-stub-layer-swap` | false | Disable stub layer switching (enabled by default): the upfront pass and every rescue that moves a stub (route.py's stub-swap rescue and boxed-stub switches, route_diff.py's multipoint terminal switch). The GUI's "Stub layer swaps" checkbox is the same setting |
| `--can-swap-to-top-layer` | false | Allow swapping stubs to F.Cu (off by default for diff pairs due to clearance) |

**How it works:**
- When source and target stubs are on different layers, a via is normally required
- Stub layer switching moves one stub to match the other's layer, eliminating the via
- For **swap pairs**, two nets exchange layers to help each other (e.g., Net1 src:A→B and Net2 src:B→A)
- For **solo switches**, a single stub moves when it doesn't conflict with other stubs
- For **bare-pad target swaps**, a diff-pair target that is a bare outer-layer pad
  (F.Cu/B.Cu) with no stub - e.g. a connector pin boxed in by neighbouring pads
  such as the front row of a 2-row header - is fanned out onto the pair's source
  layer: a through-via is dropped on each pad and a short stub grown on the source
  layer, so an inner-layer pair can land on the pad via the via instead of fighting
  through the congested surface channel
- Multiple swap options are tried: src/src, tgt/tgt, src/tgt, tgt/src
- No switch, relocation or fan-out targets a **forbidden** layer: the copper
  layers a `--layers` subset run did not request (appended as obstacles at
  cost -1) or a negative `--layer-costs` entry. A stub whose other end sits on
  a forbidden layer is not moved there; the net keeps its via
- For **multi-point nets** (3+ endpoints, #265), all movable stubs are collapsed onto one
  common layer when every endpoint allows it (through-hole and via-in-pad pads reach any
  layer; bare SMD pads fix their copper layers; already-connected pad-to-pad copper never
  moves), picking the destination layer that needs the fewest new pad vias. Nets with no
  feasible common layer are left unchanged
- The **pad via sizes itself to fit**: a switch leaving the pad's layer drops a
  through-via at the pad, at the run's nominal via size when it clears the
  surrounding copper, else the first smaller via from the fab-tier ladder that
  does (with the standard fab-escalation warning). Every switch that keeps the
  via also checks its drill against every existing hole by the hole-to-hole
  floor, the net's own vias included, shrinking it further when that helps and
  reverting the switch when nothing fits. A nominal 0.5mm via never
  fits a 0.65mm-pitch ball field boxed by neighboring fanout vias; the 0.3/0.25
  rungs do — so a stub walled in on its own layer can still switch to an open
  layer and escape. Applies everywhere the switch machinery is used: the
  routing-failure rescue rungs (single-ended and diff-pair fallback swaps), and
  the diff-pair multipoint chain, which moves blocked terminals' stubs to the
  most-open layer as its last resort before deferring the pair to single-ended
  (moving a stub also removes the copper that walls its coupled twin)

### Length Matching Options

Length matching adds trombone-style meanders to match route lengths within groups. Useful for DRAM DQ/DQS signals.

| Option | Default | Description |
|--------|---------|-------------|
| `--length-match-group` | - | Group of nets to length match. Use `auto` for DRAM auto-grouping (DDR2-DDR5, LPDDR4/5), or specify patterns |
| `--length-match-tolerance` | 0.1 | Acceptable length variance within group (mm) |
| `--meander-amplitude` | 1.0 | Height of meanders perpendicular to trace (mm) |
| `--meander-spacing` | 2.0 | Centre-to-centre pitch of adjacent single-ended meander arms, in multiples of the net's routed track width (2.0 = 2W, a 1W gap between arms; #501). On `route_diff.py` it sets the intra-pair and AC-coupled meanders; pair-centerline meanders scale with the pair gap (`--diff-chamfer-extra`) |
| `--diff-chamfer-extra` | 1.5 | `route_diff.py`: chamfer multiplier for diff pair meanders (>1 avoids P/N crossings) |
| `--diff-pair-intra-match` | false | `route_diff.py`: match P/N lengths within each diff pair (meander shorter track) |

**Auto-grouping for DRAM:**
```bash
--length-match-group auto
```
Groups nets by byte lane: DQ0-7 + DQS0, DQ8-15 + DQS1, etc., plus one
command/address group, separately for each DRAM interface and channel
(see [Length Matching](length-matching.md#ddr-auto-grouping)).

**Manual grouping:**
```bash
--length-match-group "Net-(*DQ[0-7]*)" "Net-(*DQS0*)"
--length-match-group "Net-(*DQ[8-9]*)" "Net-(*DQ1[0-5]*)" "Net-(*DQS1*)"
```

**How it works:**
- Routes all nets first, then adds meanders to shorter routes
- Calculates pad-to-pad length including existing stub segments and via barrel lengths (parsed from board stackup)
- Stub via barrel lengths use actual stub-layer-to-pad-layer distance (not full via span for through-hole BGA pad vias)
- Via barrel length matches KiCad's length calculation for accurate matching
- Adds each pad's pad-to-die length (`(die_length ...)`, #1248) before picking the target, and with `--time-matching` each pad's `(die_delay ...)` in ps, as KiCad's tuners and length/skew DRC do
- Finds longest straight segment for meander insertion
- Iteratively adds meander bumps until length exceeds target, then scales down amplitude to hit exact target
- Per-bump clearance checking reduces amplitude to avoid conflicts with other traces
- Uses 45° chamfered corners for smooth trombone patterns
- Supports multi-layer routes: meanders are applied to same-layer sections, preserving via positions

See [Length Matching](length-matching.md) for the full algorithm: meander geometry, per-bump clearance checking, via barrel lengths, and DRAM auto-grouping details.

### Time Matching Options

Time matching is an alternative to length matching that matches propagation delay instead of physical length. This accounts for different signal speeds on different layers:
- **Outer layers (microstrip)**: Faster propagation, effective dielectric = (Er + 1) / 2
- **Inner layers (stripline)**: Slower propagation, effective dielectric = Er

| Option | Default | Description |
|--------|---------|-------------|
| `--time-matching` | false | Match propagation time instead of length |
| `--time-match-tolerance` | 1.0 | Acceptable time variance within group (ps) |

**When to use time matching:**
- When routes in a group use different layers with different dielectric properties
- For high-speed signals where propagation delay matters more than physical length
- Typical FR4: outer layers ~5.4 ps/mm, inner layers ~6.9 ps/mm (~28% difference)

**Example:**
```bash
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "*DQ*" \
    --length-match-group auto \
    --time-matching \
    --time-match-tolerance 1.0
```

See [Length Matching](length-matching.md#time-matching) for how propagation delay is computed from the stackup, including via barrel delays.

### Debug Options

| Option | Default | Description |
|--------|---------|-------------|
| `--debug-lines` | false | Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*) |
| `--verbose` / `-v` | false | Print detailed diagnostic output (setback checks, bus routing order, etc.) |
| `--skip-routing` | false | Skip actual routing, only do swaps and write debug info |
| `--debug-memory` | false | Print memory usage statistics at key points during routing |
| `--stats` | false | `route.py`: print A* search statistics (cells expanded, heuristic efficiency) |
| `--add-teardrops` | false | Add teardrop settings to all pads **and vias** in the output file. Available on every step that writes pad/via copper: `route.py`, `route_diff.py`, `route_planes.py`, `repair_planes.py`, `bga_fanout.py`, `qfn_fanout.py` (#489 §9). In the GUI it is the one shared "Add teardrops" checkbox, which now reaches the fanout and planes tabs too. |

### Output Options (route.py)

| Option | Default | Description |
|--------|---------|-------------|
| `--json-out FILE` | - | Also write the run's `JSON_SUMMARY` to FILE. This is the merged tally: when the end-of-run reconciliation fires, stdout carries a second, subset-scoped summary, and scraping either one alone is wrong. Prefer this over parsing stdout. `route_diff.py` takes it too, with the same meaning (#1421). A run with nothing to route still writes the file, with a `status` saying why |
| `--write-fill` | off | After writing the board, fill its zones with KiCad's own zone filler so the file carries `(filled_polygon ...)` blocks (#910); without it the board ships zone outlines only. Needs KiCad's bundled python |
| `--capabilities` | off | Print this clone's module and flag inventory as JSON and exit (see `krt_capabilities.py --require`) |

## GridRouteConfig Class

The `GridRouteConfig` dataclass holds the routing parameters; the main ones
are below. Every field, with the engine-set ones and the methods, is in the
[Routing Configuration API](api-routing-config.md).

**These are the DATACLASS's own field defaults, which is what a bare
`GridRouteConfig()` gives you — not what a routed board gets.** The four track
geometry fields below read lower than the `--track-width` / `--clearance` /
`--via-size` / `--via-drill` rows in the flag table above, and both are right:
every routing entry point resolves geometry from the board's own net class,
else from `routing_defaults.py` (`TRACK_WIDTH` 0.3, `CLEARANCE` 0.25,
`VIA_SIZE` 0.5, `VIA_DRILL` 0.3), and passes that in — so these four are
overridden before a single track is laid. `tests/run_doc_examples.py` holds
the scalar values in this block against `py_router/routing_config.py` (and
holds the field NAMES against the API page's tables); the flag tables above
are gated by neither.

```python
@dataclass
class GridRouteConfig:
    # Track geometry -- the dataclass floor; the CLIs pass the board's net
    # class, else routing_defaults.py's (looser) values, over the top
    track_width: float = 0.1      # mm (default for non-power nets)
    clearance: float = 0.1       # mm between tracks
    via_size: float = 0.3         # mm via outer diameter
    via_drill: float = 0.2        # mm via drill
    power_net_widths: Dict[int, float] = {}  # net_id -> width for power nets
    power_tap_neckdown: bool = True   # retry failed wide power routes at default width
    neckdown_length: float = 2.5      # mm pad-entry zone on necked routes (pad-capped width where it fits)
    neckdown_taper_length: float = 0.5  # mm narrow->wide width taper (0 = abrupt)

    # Grid
    grid_step: float = 0.1        # mm grid resolution

    # A* algorithm
    via_cost: int = 75            # via penalty in 0.1mm grid steps = 7.5mm of path (diff pairs place 2 vias)
    max_iterations: int = 200000
    max_probe_iterations: int = 5000  # quick probe per direction to detect stuck routes
    heuristic_weight: float = 2.3
    turn_cost: int = 1000         # penalty for direction changes (straighter paths)
    direction_preference_cost: int = 250  # off-direction move penalty (layers alternate H/V; 0 = off)
    max_rip_up_count: int = 3     # max blockers to rip up at once (progressive N+1)
    ripup_abandon_metric: str = 'stranded'  # tap rip-up abandon rule (docs/rip-up-reroute.md)
    ripup_blocker_select: str = 'count'     # which blocker the rip-up ladder rips first
    max_setback_angle: float = 45.0  # degrees
    routing_clearance_margin: float = 1.0  # diff-pair via spacing only: track-to-via distance multiplier (1.0 = min DRC)
    hole_to_hole_clearance: float = 0.20  # mm - drill-to-drill fab floor
    board_edge_clearance: float = 0.0    # mm - clearance from board edge (0 = use clearance)
    same_net_pad_clearance: float = -1.0  # mm via to same-net pad (> 0 = no via-in-pad; -1 = allowed)
    proximity_heuristic_factor: float = 0.0  # proximity-aware heuristic (0 = off; the CLIs pass 0.02)
    ripped_route_avoidance_radius: float = 1.0  # mm - radius around ripped route corridors
    ripped_route_avoidance_cost: float = 0.1    # cost for routing through ripped corridors

    # Layers
    layers: List[str] = ['F.Cu', 'B.Cu']
    layer_costs: List[float] = []    # per-layer cost multipliers (route.py fills 1.0 for 4+ layers, else F.Cu=1.0/others 3.0)

    # BGA zones
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = []

    # Stub proximity
    stub_proximity_radius: float = 2.0   # mm
    stub_proximity_cost: float = 0.2     # mm equivalent
    via_proximity_cost: float = 10.0     # multiplier for vias near stubs

    # BGA proximity
    bga_proximity_radius: float = 7.0   # mm from BGA edges
    bga_proximity_cost: float = 0.2      # mm equivalent

    # Track proximity (same layer)
    track_proximity_distance: float = 2.0  # mm
    track_proximity_cost: float = 0.0      # mm equivalent (0 = disabled)

    # Pairwise keep-away between net groups (#1146)
    keep_away: Tuple[str, ...] = ()        # 'AGG:VICTIM:GAP' rules
    keep_away_free: float = 1.5            # mm around the routed net's own pads
    keep_away_cost: float = 0.5            # mm equivalent per cell (0 = report only)

    # Vertical track alignment (cross-layer attraction)
    vertical_attraction_radius: float = 1.0  # mm
    vertical_attraction_cost: float = 0.0    # mm equivalent bonus

    # Direction
    direction_order: str = "forward"     # forward or backward

    # Differential pairs
    diff_pair_gap: float = 0.101         # mm between P and N
    diff_pair_centerline_setback: float = None  # mm in front of stubs (None = 2x P-N spacing)
    min_turning_radius: float = 0.2      # mm for pose-based routing
    max_turn_angle: float = 180.0        # degrees, prevents U-turns
    stub_layer_swap: bool = True         # enable stub layer switching optimization
    gnd_via_enabled: bool = True         # place GND vias near signal vias
    target_swap_crossing_penalty: float = 1000.0  # penalty for crossing assignments
    crossing_layer_check: bool = True    # only count crossings on same layer

    # Length matching
    length_match_groups: List[List[str]] = []  # groups of net patterns to match
    length_match_tolerance: float = 0.1        # mm - acceptable length variance
    meander_amplitude: float = 1.0             # mm - height of meander perpendicular to trace
    meander_spacing: float = 2.0               # arm pitch, in multiples of the net's track width
    diff_chamfer_extra: float = 1.5            # chamfer multiplier for diff pair meanders
    diff_pair_intra_match: bool = False        # match P/N lengths within each diff pair

    # Time matching (alternative to length matching)
    time_matching: bool = False                # match propagation time instead of length
    time_match_tolerance: float = 1.0          # ps - acceptable time variance

    # Debug
    debug_lines: bool = False
    verbose: bool = False
    debug_memory: bool = False

    # --quick LEVEL: 0 = route to ship, 1 = review, 2 = seed judge
    quick: int = 0
```

## Parameter Guidelines

### Via Cost

The `via_cost` parameter controls how much the router penalizes layer changes:

| Value | Effect |
|-------|--------|
| 0-25 | Many vias, shorter paths |
| 75 (default) | Balanced, discourages unnecessary vias |
| 100+ | Few vias, longer paths |

75 is the corpus-measured default (#586: against the old 50 it took the
disconnection verdict down 8 and DRC down 13, and it composes with heuristic
weight 2.3). 25 and 100 both measured worse. Do not lower it for BGA escape
routing: 25 lost on a corpus that includes BGA escape boards.

All cost knobs (via cost, proximity costs, attraction bonuses) are calibrated at a 0.1mm
reference grid and scale internally so the cost per mm of path is the same at any
`--grid-step` - changing the grid resolution does not change what the router optimizes for.

### Heuristic Weight

Controls A* behavior:

| Value | Effect |
|-------|--------|
| 1.0 | Optimal paths (slower) |
| 2.3 (default) | The corpus dose-response peak (#586) -- measured, not intuitive |
| 2.5+ | Faster but may miss tight routes |

### Grid Step

Smaller grid steps allow finer routing but increase computation:

| Value | Use Case |
|-------|----------|
| 0.05 | Fine-pitch BGAs, tight clearances |
| 0.1 (default) | General purpose |
| 0.2 | Fast routing, less detail |

### Layer Costs

The `--layer-costs` parameter controls per-layer routing preferences. Higher costs make the router avoid that layer:

| Value | Effect |
|-------|--------|
| 1.0 | No penalty, layer used freely |
| 3.0 | Moderately avoided (3x cost per grid step) |
| 10.0+ | Strongly avoided, used only when necessary |

**Defaults** (`route.py`, by the number of routing layers; `route_diff.py`
uses 1.0 on every layer):
- 4+ layers: all 1.0 (inner layers available for balanced routing)
- 2 layers: F.Cu=1.0, B.Cu=3.0 (prefer top layer for cleaner assembly)

A negative value (e.g. `-1`) forbids the layer: it stays an obstacle and
through-vias may span it, but no track is placed on it.

**Example:** To keep signals on F.Cu and only use B.Cu when necessary:
```bash
--layers F.Cu B.Cu --layer-costs 1.0 5.0
```

**One value per routing layer, in `--layers` order** (for `route.py` with
`--layers` omitted, one per board copper layer; for `route_planes.py`, one per
entry of its `--layers`, default `F.Cu` + plane layers + `B.Cu`). The copper
layers a run does not route on are appended after them as FORBIDDEN
obstacles, so a list of any other length is refused (`ERROR: --layer-costs
needs one value per routing layer ...`) rather than shifted onto the wrong
layers: `--layers F.Cu B.Cu --layer-costs 1.0` on a 4-layer board used to
forbid the requested `B.Cu` and leave the unrequested `In2.Cu` routable.
`route.py`, `route_diff.py`, `route_planes.py`, `repair_planes.py` and
`bga_fanout.py` all refuse it. In the GUI the Layer Costs field takes one
value per copper layer (its default) or one per selected layer, and each tab
places them on its own layers by name.

### Stub Proximity

Penalizes routes that pass near unrouted stubs:

```
Cost = stub_proximity_cost * (1 - distance/stub_proximity_radius)
```

This encourages routes to avoid blocking future routing paths.

## Layer Configuration

### Standard 4-Layer Stack

```bash
--layers F.Cu In1.Cu In2.Cu B.Cu
```

### 2-Layer Board

```bash
--layers F.Cu B.Cu
```

### 6-Layer with Ground Planes

```bash
# Skip In1.Cu and In4.Cu (ground planes)
--layers F.Cu In2.Cu In3.Cu B.Cu
```

## Example Configurations

### BGA Fanout (Dense, Many Vias)

```bash
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --ordering inside_out
```

Leave the via cost, heuristic weight and stub proximity at their defaults
(75, 2.3, 0.2): a lower via cost or heuristic weight measured worse across
the corpus (#586), and the stub penalty is near-optimal as shipped (#584).

### Long Routes (Few Vias)

```bash
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --ordering mps
```

`--via-cost` is the knob that trades path length for vias: raising it above
the default 75 buys fewer vias with longer routes, lowering it the reverse.
The default is the corpus-measured setting, so compare a changed value
against a default run on the same board before keeping it.

### Differential Pairs (LVDS)

```bash
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*" \
    --diff-pair-gap 0.1 \
    --track-width 0.1 \
    --clearance 0.1 \
    --no-bga-zones
```

### Fast Routing (Large Boards)

```bash
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --grid-step 0.2
```

A coarser grid is the speed lever. Keep `--heuristic-weight` at 2.3 (lower
is slower, not faster) and `--max-iterations` at its default: a smaller
budget fails long routes instead of finishing them sooner.

### Fine-Pitch BGA

```bash
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --grid-step 0.05 \
    --track-width 0.1 \
    --clearance 0.1 \
    --via-size 0.25 \
    --via-drill 0.15
```

These are the advanced fab rung. A value below the physical fab floor is
pinned up to it with a warning (see *Floor enforcement* under
[Fab Tier Options](#fab-tier-options)); pass `--fab-overrides` to declare a
fab that can go smaller.

### Power and Signal Mixed (Wider Power Tracks)

```bash
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "*" \
    --power-nets "*GND*" "*VCC*" "*VDD*" "+*V" \
    --power-nets-widths 0.5 0.4 0.4 0.3 \
    --track-width 0.2 \
    --clearance 0.2
```
