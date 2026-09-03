# Proposal: one KiCad-standard design-rule model for sizes, clearances and escalation

**Status:** proposal, not implemented. Audited at `e239e067` (main, 2026-09-03) on branch
`worktree-constraints-dru`.
**Issues:** #530 (constraints must equal KiCad's DRC), #842 (tracks always 0.127),
#857 (`--fab-tier standard` escalates silently), and the related #856 (severities
rewritten to ignore), #770 (dru subset never fires on a real board), #502 (drill aspect
ratio).

---

## 1. Summary

Today the router decides track width, via size, clearance and the hole/edge floors
through **four disconnected mechanisms** that were each added to fix one issue:

1. a **fab-tier ladder** (`fab_tiers.py`) that is documented as a floor but acts as a
   *routing preference* which 18 code sites descend on their own;
2. a **netclass reader** (`list_nets.py`) that reads six fields of the Default class and
   treats the class width as the exact routed width (KiCad treats it as a *preferred*
   width, `opt`);
3. a **`.kicad_dru` reader** (`kicad_dru.py`) that honours exactly one constraint kind
   (`clearance (min)`) in two scope shapes, and drops every other rule silently. It has
   never bound on any real board we ingest (#770): the five real `.kicad_dru` files on
   this machine use netclass-pair, `hasNetclass`, `NetName`, `Type`, `intersectsArea`,
   `inDiffPair` and `memberOfGroup` scopes, and the kinds `track_width`, `via_diameter`,
   `hole_size`, `edge_clearance`, `hole_to_hole`, `diff_pair_gap`, `disallow`, `length`;
4. a **project writeback** (`fix_kicad_drc_settings.py`) that lowers the board's
   minimums *and its netclass draw sizes* to the smallest object the run produced, and
   sets six DRC severities to `ignore`.

The composition of these is a caller convention (79 call sites hand-assemble the
clearance chain), so the router, `check_drc`, the placer and the GUI each resolve a
slightly different number.

**#842 is the four mechanisms composing:** one terminal segment is necked to the 2-layer
fab floor (0.127 mm), the writeback lowers the *Default netclass* `track_width` to 0.127,
and the next run reads the Default class back as "the board's own width". Nothing ever
raises it again. The GUI's "Obey design rule constraints" checkbox never reaches the
engine at all: it only clamps spin controls, and the Board Setup minimum the reporter
set is never read as a width.

**The proposal** replaces the four mechanisms with one **`DesignRules` resolver** whose
inputs are exactly KiCad's (Board Setup minimums, net classes with priority,
`.kicad_dru` custom rules, per-item overrides), whose evaluation order is KiCad's
(verified against `drc_engine.cpp`), and whose output is consumed by the router, the
graders, the placer and both front-ends. Around it:

- **Draw sizes are `opt`, floors are `min`, exactly as in KiCad.** A net routes at its
  requested width and is never narrowed unless the user's **escalation policy** allows
  it, and never below what KiCad's DRC would accept unless the policy says `fab`.
- **Escalation is one explicit knob**, identical on CLI, GUI, plan JSON and manifests:
  `--escalation off | board | fab` (recommended default `board`), plus
  `--fab-tier standard | advanced | auto` where `auto` is today's silent ladder made
  explicit. Every escalation is counted in `JSON_SUMMARY`, printed once at the end of the
  run, and shown in the GUI results panel.
- **The writeback stops ratcheting.** Netclass draw sizes are never lowered, severities
  are never touched by a routing step, and `rules.min_*` are lowered only when
  `--escalation fab` actually crossed a board minimum, with the original recorded.
- **Fab capability lives where KiCad puts it**: the tier tables become a way to *fill
  in* unset Board Setup minimums (and to bound `fab` escalation), not a parallel rule
  system. A fab profile can be expressed as a `.kicad_dru` KiCad itself enforces.

All of it is Python except one piece. The Rust router takes no width, via or clearance
parameter; those are obstacle-map expansion in Python. The exception is per-net via
sizes, which go into Rust as N via-legality bitmaps (§6.5, decision 4), with their own
crate bump and binary release.

---

## 2. What KiCad actually does (verified)

Read from `pcbnew/drc/drc_engine.cpp` (9.0 and master), the in-app rule syntax help,
`pns_kicad_iface.cpp`, and a probe of the installed KiCad 10.0 SWIG bindings.

**Rule evaluation.** For each constraint kind the engine builds one vector:
Board-Setup implicit rules, then one implicit rule per net class (condition
`A.hasExactNetclass('X')`), then keepout areas, then the `.kicad_dru` rules in file
order. `EvalRules` walks the vector **forward over every rule** and applies `min`,
`opt`, `max` **per field**; the last matching rule wins each field. There is no rule
`priority`; priority is a property of *net classes* (a net in several classes gets an
aggregate class taking each property from the highest-priority class that sets it).

**Board Setup minimums are only a hard floor for copper clearance.** `min_clearance`
(and, using the same value, the diff-pair gap) is applied *after* the rule walk as a
post-loop `max`. `min_track_width`, `min_via_diameter`, `min_through_hole_diameter`,
`min_via_annular_width`, `min_hole_to_hole`, `min_hole_clearance` and
`min_copper_edge_clearance` are merely the *first* rule in their vector; a later
`.kicad_dru` rule replaces them outright, in either direction.

**Net classes.** `clearance` is `SetMin` (DRC-enforced, pairwise `max(A, B)`).
`track_width` is `SetMin(board min) + SetOpt(class width)`. `via_diameter`,
`via_drill`, `diff_pair_width`, `diff_pair_gap` are `SetOpt` only. The class sizes are
what the interactive router *draws*; DRC accepts anything at or above the board minimum.

**Local overrides.** A pad or footprint `(clearance ...)` is a *clearance override*:
the engine takes `max(override_a, override_b)`, floors it at `min_clearance`, and
**returns before evaluating net classes or custom rules**. A zone's local clearance is
applied after the rule walk as `max(rules, local)`. The repo currently models both as
`max(rules, local)` (#326/#697), which is right for zones and wrong for pads that
declare a clearance *below* their class.

**The interactive router (PNS).** `ImportSizes` starts each size at the board minimum
and takes `max(board min, resolved opt)` under "use netclass values", or
`max(board min, toolbar value)` under "custom size". It never narrows a trace
mid-route. So a KiCad user who sets netclass width 0.25 and board minimum 0.127
expects 0.25 mm copper, and expects a 0.15 mm track to pass DRC.

**Custom-rule language.** Constraint tokens the 9.0 parser accepts: `clearance`,
`hole_clearance`, `edge_clearance`, `hole_to_hole`, `physical_clearance`,
`physical_hole_clearance`, `courtyard_clearance`, `silk_clearance`, `track_width`,
`track_angle`, `track_segment_length`, `connection_width`, `annular_width`,
`via_diameter`, `hole_size`, `via_count`, `disallow`, `length`, `skew`,
`diff_pair_gap`, `diff_pair_uncoupled`, `zone_connection`, `thermal_relief_gap`,
`thermal_spoke_width`, `min_resolved_spokes`, `assertion`, `creepage`, `text_height`,
`text_thickness`. Clauses: `(constraint ...)`, `(condition "...")`,
`(layer name|outer|inner)`, `(severity ...)`. A `clearance (min)` may be negative
(allow overlap).

**API.** SWIG exposes every Board Setup minimum, `m_NetSettings` (default class,
by-name lookup, effective class, pattern and label assignments, class priority) and per
item `GetLocalClearance` / `GetClearanceOverrides`. It does **not** expose
`DRC_ENGINE`, `EvalRules` or the custom rules; the `.kicad_dru` must be read from the
file beside `board.GetFileName()`. The IPC API exposes less. So the resolver must be
ours, and its correctness must be *measured* against `kicad-cli pcb drc` (§7.1).

---

## 3. What the code does today (audit)

Full details with `file:line` citations are in the audit notes this proposal was
written from; the load-bearing findings are summarised here.

### 3.1 Track width

- Origins, in order of application: `routing_defaults.TRACK_WIDTH` 0.3 → the board's
  Default netclass `track_width` → `--track-width` → per-net ladder in
  `GridRouteConfig.get_net_track_width` (power width, stored impedance width, own
  class width, coplanar width, layer width) → `enforce_fab_floors` raise-only.
- The class width is used **exactly** (`net_track_widths`, #435) and floored at the
  *advanced* fab tier, not the board minimum.
- Narrowing after the choice, all engine-side so both fronts inherit it:
  1. `_neck_terminal_grazes` (`single_ended_routing.py:691-765`) runs on **every**
     converted route and necks a grazing terminal segment down to
     `_fab_track_floor` = `fab_floors(n)['track_width']` = **0.127 on 2 layers**.
  2. `net_rescue._rescue_rungs` (`net_rescue.py:366-427`) re-routes every failed net at
     `min(nominal, fab_track, class floor)` and **pops the user's power width** for
     that net.
  3. `net_rescue._escalation_ladder` (`:1336-1400`) marches width *and* via to the fab
     floor for any still-open net, stripping every per-net override first; entered
     unconditionally from `route.py:2374-2400`.
  4. Fine-pitch taps (`plane_pad_tap.fine_tap_configs`), impedance/power neck-down
     (bounded at `config.track_width`).
- The requested→delivered histogram already exists (`net_rescue.py:1258-1320`) and is
  not surfaced; `tests/test_rescue_width_provenance.py` pins that narrowing is
  legitimate as long as it is *reported*.

### 3.2 Via size and the fab-tier ladder

- `via_size` / `via_drill` are **per-call scalars**; there is no per-net map, although
  183 of 445 corpus boards declare a non-Default class whose via differs from Default.
- `fab_floor_ladder(standard)` = `[0.45/0.20, (0.30/0.15 on 4+ layers), 0.25/0.15]`.
  **18 sites** walk it downward on their own initiative (plane taps, last-resort
  via-in-pad, unblock refit, layer-switch vias, net rescue, terminal escalation, BGA and
  QFN via-in-pad clamps, drill thinning, under-pad rescues, plane last-resort vias, the
  diff-pair layer-swap shrink, the #568 small-via bitmap). Three of them warn nowhere
  (`_unblock_via_refit`, `layer_swap_optimization._swap_vias_fit_or_shrink`, and the
  under-pad warning that sits inside `if verbose:`).
- The only disclosure is `warn_fab_escalation`: a deduped `print`. Nothing reaches the
  results dict, `JSON_SUMMARY`, or the GUI results panel. `--fab-overrides` "pins" only
  because it collapses the ladder to one rung.
- `enforce_fab_floors` and the GUI's `_fab_floored` pin user input at `fab_floor_min`
  (the *deepest* rung), so `--via-size 0.25` is accepted under `standard` without a word.
- `check_drc` grades sizes against `fab_floor_min` unconditionally, never against the
  board's own `min_*`; the writeback then lowers `min_via_diameter` etc. to the smallest
  object on the board. Grader and project relax in lockstep with the escalation, which is
  why #857 reads "0 violations, exit 0".

### 3.3 Clearance

- The chain `max(class_a, class_b)` → `.kicad_dru` layer rule REPLACES → `max(...,
  pad.local_clearance)` → `max(..., dru track rule)` is assembled by hand at **79 call
  sites** across ten files; `get_net_clearance` is dead code.
- The fab floor is injected *inside* `read_board_layer_clearances`; `check_drc` and the
  placer call it without the floor, so a sub-fab rule is routed at the fab floor and
  graded at the rule.
- Two netclass resolvers disagree: the router's `net_clearance_map_by_id` takes the
  max over all memberships and drops Default; the grader's `net_clearance_map` takes
  the single resolved class and keeps Default.
- The `--clearance` ceiling (#439) is applied by pre-capping the map in `main()`; the
  engine's own auto-read (`route.py:777-786`) caps **unconditionally**, so any direct
  `batch_route` caller behaves as if `--clearance` were given.
- `min_hole_clearance` is read as `rules` alone by the grader and as
  `max(rules, fab_floor_origin)` by the router.

### 3.4 Writeback

- Lowers `rules.min_clearance / min_hole_clearance / min_hole_to_hole / min_track_width
  / min_connection / min_via_diameter / min_through_hole_diameter / min_via_drill /
  min_via_annular_width` to the routed floor or the smallest object scanned on the board.
- **Lowers the Default netclass `track_width`, `via_diameter`, `via_drill`,
  `clearance`** (and non-Default `clearance` when the ceiling was given). This is the
  #842 ratchet; nothing raises them back. The GUI's `update_live_drc_floors` does the
  same on the live board.
- Sets `malformed_courtyard`, `npth/pth_inside_courtyard`, `solder_mask_bridge`,
  `annular_width`, `lib_footprint_*` to `ignore` and `starved_thermal` to `warning`
  (`courtyards_overlap` is `warning`, not `ignore`; #856 is stale on that one line). The
  routing CLIs expose only `--keep-thermal`; the other keep flags exist only on the
  standalone script. So every `route.py` run silences `annular_width`, a fab floor.
- Round trip: step N writes the Default class down; step N+1 reads it as the board's
  own width. Only `resolve_hole_clearance` consults `fab_floor_origin`.

### 3.5 GUI

- "Obey design rule constraints" **never enters the config**. It clamps the spin
  controls to `m_TrackMinWidth` etc. on open, toggle and spin events. Values set by
  `SetValue` (settings restore, plan executor) bypass it.
- With an override box unchecked (the default) `_effective_geometry_floor` reads the
  **Default netclass** value, never the Board Setup minimum; with it checked, the typed
  value; in both cases `_fab_floored` to the deepest tier rung.
- Fab tier and overrides file are exposed and persisted; escalation is identical to the
  CLI and equally undisclosed (the `print` lands in the log pane).
- Other parity gaps found: the hole-to-hole control is labelled "Min Hole Clearance";
  `planes_zone_clearance` override checkbox is not persisted; `board_edge_clearance`
  is missing from `_revalidate_fab_floors`; the single-ended impedance control is an
  integer spin against a float flag; five `route_planes` constraint flags have no
  control; `--min/--max-track-width` are dead plan params; QFN fanout writes back almost
  no floors; the planes tab omits the edge rule from its writeback.

### 3.6 Corpus facts (445 stress boards, measured)

| Fact | Count |
|---|---|
| boards with a non-Default net class | 216 |
| ...whose track or via size differs from Default | 183 |
| boards using `netclass_patterns` | 196 |
| `min_track_width` > 0 | 349 |
| `min_via_diameter` > 0 | 444 |
| `min_hole_clearance` > 0 | 426 |
| `min_copper_edge_clearance` > 0 | 356 |
| `min_clearance` > 0 | 185 |

Per-net via sizing is not an edge case; it is 41% of the corpus.

---

## 4. Design

### 4.1 One resolver, one vocabulary

New module `py_router/design_rules.py`:

```python
@dataclass(frozen=True)
class Constraint:
    kind: str                 # 'clearance', 'track_width', 'via_diameter', ...
    min: Optional[float]      # mm (or count for via_count, degrees for track_angle)
    opt: Optional[float]
    max: Optional[float]
    source: str               # 'board minimum' | 'netclass X' | 'rule "name"' |
                              # 'pad override' | 'fab floor standard' | 'cli' | 'default'

class RuleItem:               # the A / B of a KiCad condition
    type: str                 # 'track' | 'via' | 'pad' | 'zone' | 'hole' | 'footprint'
    net_id: int
    netclasses: frozenset     # every class the net is a member of
    effective_class: str      # the aggregate (priority-resolved) class name
    net_name: str
    layers: frozenset
    footprint_ref: Optional[str]
    pad_type: Optional[str]   # 'smd' | 'thru_hole' | ...
    via_type: Optional[str]   # 'through' | 'micro' | 'blind_buried'
    groups: frozenset
    diff_pair: Optional[str]
    xy: Optional[Tuple[float, float]]   # for area predicates
    local_clearance: Optional[float]    # pad/footprint override (#326)
    clearance_override: Optional[float] # zone local clearance

class DesignRules:
    @classmethod
    def from_project(cls, pcb_data, board_path, *, fab_profile, cli) -> "DesignRules"
    @classmethod
    def from_pcbnew(cls, board, pcb_data, *, fab_profile, cli) -> "DesignRules"

    def resolve(self, kind, a: RuleItem, b: Optional[RuleItem] = None,
                layer: Optional[str] = None) -> Constraint
    def resolve_stack(self, kind, a, b, layers) -> Constraint     # max over layers
    def draw_size(self, kind, net_id, layer) -> float             # the opt path (§4.3)
    def floor(self, kind, net_id=None, layer=None) -> float       # the min path
    def unsupported(self) -> List[str]                            # rules we skipped, by name
    def table(self) -> dict                                       # for JSON/GUI disclosure
```

Evaluation inside `resolve` **is** KiCad's order, and the code reads as the
documentation:

```
1  pad/footprint clearance override on a or b (clearance kinds only):
       max(overrides) floored at board min_clearance -> return          [KiCad: early return]
2  vector[kind] = [board minimum] + [netclass implicit rules, priority-ordered]
                + [keepouts] + [.kicad_dru rules, file order]
   walk forward; a rule whose scope matches (a, b, layer) overwrites min/opt/max PER FIELD
3  zone local clearance (clearance kinds): max(result, local)
4  kind in {clearance, diff_pair_gap}: max(result, board min_clearance)  [KiCad post-loop floor]
5  fab profile floor (this tool only, §4.4): max(result.min, fab.min)  -- reported when it binds
```

Step 5 is the one deliberate departure from KiCad, and it is *raise-only* and
*disclosed*, which is the only kind of departure this design permits.

Both loaders produce the same rule table for the same board; a parity gate
(`tests/gui_parity/test_design_rules_loader_parity.py`) diffs `table()` from
`from_project` against `from_pcbnew` on the live board, the way the parser-parity
tests already do for `PCBData`.

`GridRouteConfig` gains one field, `rules: DesignRules`, and its
`obstacle_clearance` / `layer_clearance` / `stack_clearance` /
`track_obstacle_clearance` / `get_net_track_width` become **delegating shims** so the
79 call sites keep working during migration. A `KICAD_ROUTING_STRICT_RULES=1` mode
makes a shim that is called *without* the layer argument raise, so the "caller forgot
a tier" hole (#530) is caught in CI on a ruled fixture rather than shipped.

### 4.2 `.kicad_dru`: parse everything, evaluate a declared subset, disclose the rest

The parser reads **every** rule and every `min` / `opt` / `max`. Evaluation supports
the predicate subset that the real files on this machine actually use, which is also
the subset our item model can answer without guessing:

| Predicate | Needs | Phase |
|---|---|---|
| `(layer name\|outer\|inner)`, `A.onLayer`, `existsOnLayer`, `A.Layer` | layer set | 1 |
| `A.NetClass ==/!= 'X'`, `A.hasNetclass('X')`, `A.hasExactNetclass('X')`, same for `B` | memberships (already resolved by `net_class_memberships`) | 1 |
| `A.NetName ==/!= 'X'`, `A.Net != B.Net`, `A.Net == B.Net` | net | 1 |
| `A.Type == 'Track'\|'Via'\|'Pad'\|'Zone'`, `A.Pad_Type`, `A.Via_Type`, `isMicroVia`, `isBlindBuriedVia`, `isPlated` | item type | 1 |
| `&&`, `\|\|`, `!`, parentheses, quoted strings, unit arithmetic in values | expression parser | 1 |
| `A.memberOfFootprint('glob')`, `A.intersectsCourtyard('ref')` | footprint ref / courtyard polygon | 2 |
| `A.memberOfGroup('G')` | group membership (parseable from the file; GUI via `GetParentGroup`) | 2 |
| `A.intersectsArea('zone')`, `enclosedByArea` | rule-area zone polygons (we parse zones) | 2 |
| `A.inDiffPair('glob')`, `AB.isCoupledDiffPair()` | diff-pair detection (exists) | 2 |
| `fromTo`, `memberOfSheet`, `hasComponentClass`, `getField`, `A.Length`, assertions | out of scope | never |

A rule using an unsupported predicate or kind is kept in the table marked
**UNSUPPORTED** and reported at run start (once per board), in `JSON_SUMMARY`
(`design_rules.unsupported: [names]`) and in the GUI results panel, replacing today's
silent drop. That is #770's "make the skip loud" and #530's "warn about what we cannot
honour" in one channel.

Kinds the router *consumes* and how:

| Kind | Consumer | Semantics |
|---|---|---|
| `clearance`, `hole_clearance`, `edge_clearance`, `hole_to_hole`, `physical_clearance`, `physical_hole_clearance` | obstacle builders, plane builders, `check_drc`, placer | `min`, per (a, b, layer); replaces the scalar `hole_to_hole_clearance` / `board_edge_clearance` / `hole_clearance` config fields |
| `track_width`, `via_diameter`, `hole_size`, `diff_pair_gap`, `diff_pair_width` | `draw_size` (§4.3), escalation floor (§4.4), `check_drc` (`min` and the new `max` checks) | `opt` is what we draw when free to choose, `min` is how far escalation may go, `max` is a hard cap (a wide power track may not exceed a rule's `max`) |
| `annular_width` | via clamp, `check_drc` | `min` |
| `disallow track\|via` on a scope | keepout machinery (`--keepout` already exists) / drop layer from the net's layer set | phase 2 |
| `via_count (max)` | the per-net via budget the rescue ladders already reason about | phase 2 |
| `length`, `skew`, `diff_pair_uncoupled` | length matching (`--length-match` groups), reported | phase 3 |
| `track_angle (min 135)` | octolinear vs orthogonal smoothing selector | phase 3 |
| everything else | table only, reported | — |

### 4.3 Sizes: KiCad's `opt` / `min` split

The routed size of a net on a layer is:

```
draw_size(kind, net, layer) =
    CLI explicit flag           (PNS "custom size": --track-width / --via-size / ...)
 or .kicad_dru opt              (the last matching rule that sets opt for this net/layer)
 or impedance solver width      (per layer; #521 stored widths behave the same)
 or aggregate netclass value    (priority-resolved; Default when unassigned)
 or routing_defaults
then max(., floor(kind, net, layer))     -- never below what DRC accepts
then min(., rule max) if a max exists
```

This replaces `track_width` / `net_track_widths` / `netclass_width_floors` /
`via_size` / `via_drill` with one accessor, adds the per-net **via size** path that does
not exist today, and makes `--track-width` mean what a KiCad user expects: the width
you typed, for every net you routed with it (a `--power-nets-widths` still outranks it
per net, as today).

The **obstacle reserve** (`route_reserve_width`) keeps reserving the *nominal* width and
carrying extra half-width as a margin, as today; nothing about the A* cost model
changes.

### 4.4 Escalation: one knob, explicit, disclosed

```
--escalation off    sizes and clearances are exact. A net that cannot complete at them
                    FAILS and is reported with the geometry that would have completed it.
--escalation board  (recommended default) a failing net may be re-tried narrower /
                    with a smaller via / tighter clearance, down to the board's own
                    floors: rules.min_* and .kicad_dru mins, i.e. what KiCad's DRC
                    accepts. No project writeback is needed for the result to be clean.
--escalation fab    additionally may go below the board's floors down to the fab
                    profile's floor (today's behaviour). This is the ONLY mode in which
                    rules.min_* are lowered in the project, and it records the original
                    in kicad_routing_tools.fab_floor_origin (exists) and logs each key.
```

Where a board leaves a minimum unset (KiCad writes 0; 96 of 445 corpus boards for
`min_track_width`), `board` falls back to the fab profile's floor **for that key**, and
says so in the disclosure. This keeps today's completion rate on undeclared boards while
never crossing a floor a user did declare.

**A board minimum bounds descents only when the run's own request respects it**
(decided during implementation, 2026-09-03). A request already below the declared
minimum, e.g. `--via-size 0.3` on a project still carrying KiCad's stock
`min_via_diameter` 0.5, marks that minimum as stale for the run: it is announced on the
console, descents for that key bound at the fab floor instead, and the request is never
pinned up to the board minimum (only to the fab floor, as today). The alternative,
KiCad's interactive-router rule of `max(board minimum, requested)`, would have rerouted
most corpus commands at stock minimums nobody had edited, which is not what those
commands asked for.

Escalation is **per failing net and per attempt** (the rescue ladders already are);
it never changes the run's nominal `config.track_width`, and the terminal-graze neck
(`_neck_terminal_grazes`) becomes an escalation like the others: under `off` a graze
that cannot be cleared at full width fails the route through its existing `hard`
channel instead of shipping narrowed copper.

`--fab-tier` keeps its two tiers and gains `auto`:

```
--fab-tier standard   hard: nothing on the board goes below the standard floor
--fab-tier advanced   hard
--fab-tier auto       today's standard->advanced ladder, made opt-in (#857)
--fab-overrides FILE  overlays the selected tier (as today); also accepts a .kicad_dru
                      of (constraint ... (min ...)) rules, the KiCad-native way to write
                      a fab profile
```

`fab_floor_ladder` becomes the single chokepoint: it returns one rung unless the tier
is `auto`, so all 18 descent sites become inert by construction, exactly the mechanism
`--fab-overrides` uses today. Each site's "no rung left" path must become a **reported
refusal**, which `thin_drill_to_clear` and the BGA "escape dropped" disclosure already
model.

**Explicit requests are checked against the physical floor, not the tier** (decided
during implementation, 2026-09-03). `enforce_fab_floors`, the GUI's `_fab_floored` and
`check_drc`'s size floors use `physical_fab_floor` (the override file, else the
advanced rung). The tier bounds only automatic descents. Pinning `--via-size 0.3` up to
the standard tier's 0.45 would have rerouted every recorded command that asks for a
0.3 via and then had the grader flag the vias it asked for; the request is the operator
declaring their fab can make it, the same reading the stale-board-minimum rule gives.

**Disclosure**, one shape for every front:

```json
"design_rules": {
  "escalation_policy": "board",
  "fab_tier": "standard",
  "escalations": {
    "count": 7,
    "nets": [{"net": "/SDQ3", "kind": "track_width", "requested": 0.25, "delivered": 0.15,
              "floor_source": "board minimum", "site": "net rescue"}],
    "fab_tier_escalations": 0,
    "min_track_width_used": 0.15, "min_via_diameter_used": 0.45, "min_clearance_used": 0.2
  },
  "unsupported_rules": ["SI: No acute or right-angle signal corners"],
  "project_writes": [{"key": "rules.min_track_width", "from": 0.2, "to": 0.15}]
}
```

plus one end-of-run line ("7 nets delivered below their requested size; smallest
track 0.15 mm (board minimum); 0 fab-tier escalations") and a results-panel row in the
GUI. `--strict-sizes` (CLI) / the same checkbox (GUI) makes any escalation a non-zero
exit so a CI harness needs no grep. `check_complete` folds the count into its existing
`fab_floors` verdict.

### 4.5 Writeback: never lower a draw default, never touch severities

`fix_kicad_drc_settings` keeps its "only loosen" `rules.min_*` role but with these
changes:

1. **Netclass `track_width`, `via_diameter`, `via_drill`, `diff_pair_width` are never
   written** by a routing step. They are `opt` values, the user's intent, and lowering
   them is the #842 ratchet. (`_NONDEFAULT_CLAMP_FIELDS` already encodes this reasoning
   for non-Default classes; it simply never applied it to Default.)
2. **`rule_severities` are not touched** by `route.py`, `route_diff.py`,
   `route_planes.py`, the fanouts, or the GUI (#856). The standalone
   `fix_kicad_drc_settings.py --relax-severities` keeps the behaviour for whoever wants
   it, logs one line per category changed, and stores the previous values under
   `kicad_routing_tools.saved_severities`.
3. `rules.min_*` are lowered only under `--escalation fab` and only for keys the run
   actually crossed, each logged and listed in `project_writes`. Under `board` the
   output is DRC-clean against the input project by construction, so the writeback is
   a no-op and the "route.py silently changes the routing on re-run" trap in CLAUDE.md
   disappears with it.
4. `min_clearance` is written as today (the clearance ledger's effective value, capped
   at the smallest honoured rule), because KiCad enforces it as an absolute floor and
   the fine-pitch tap ladder legitimately routes below the class.
5. The netclass **clearance** clamp stays tied to the explicit ceiling (§4.6).
6. One board-level core (`write_project_floors(rules_result, ...)`) backs both
   `apply_targets_to_project` and `apply_targets_to_board` / `update_live_drc_floors`,
   so the GUI cannot drift from the CLI again (the planes-tab and QFN-tab omissions in
   §3.5 close for free).

### 4.6 `--clearance`: two meanings, one of them explicit

Today the *presence* of `--clearance` is a ceiling switch over every class (#439).
KiCad has no ceiling concept; a user with a `Default` 0.2 / `HV` 1.0 project who passes
`--clearance 0.15` because their fab allows it gets `HV` capped to 0.15, which no KiCad
user expects. Proposed:

- `--clearance X` (and the GUI "Min Clearance" override) = **the Default class clearance
  is X for this run**. Non-Default classes are honoured pairwise as KiCad does. The
  Default class is written back to X, as today.
- `--clearance-ceiling X` (new; GUI: a second checkbox) = today's behaviour, capping
  every class at X and clamping the classes in the writeback. This is the "stock classes
  are aspirational" workflow CLAUDE.md documents, kept, but named.
- The engine's unconditional cap in `batch_route` (`route.py:777-786`) goes; the engine
  receives the resolver, and the ceiling is a rule-table transform applied once by
  whoever built it.

This is a **behaviour change on the corpus** (the RUNBOOK passes `--clearance`
everywhere) and is gated by the corpus A/B in §7.3. If the A/B shows a completion
regression on boards with aspirational classes, the fallback is to keep the ceiling
semantics under `--clearance` and ship only the explicit flag; the resolver does not
care which spelling wins.

### 4.7 Graders share the resolver

`check_drc` replaces its private closures (`_pair_cl`, `_layer_cl`, `_stack_cl`,
`_pads_cl`, `_track_pair_cl`, the `npth_clr` chain) and its `fab_floor_min` size floors
with `rules.resolve(...)` on the same table the router used. It grades sizes against
the board's own `min_*` and rules (what KiCad grades) and reports the **fab capability**
check separately ("KiCad DRC: 0; fab profile standard: 3 vias below 0.45"), so the two
verdicts can no longer be conflated. The placer's `PadClearanceModel` and
`fanout_clearance` (a flat-scalar channel today) resolve through the same object. The
`max` checks for the draw-size family are new.

### 4.8 GUI

- **"Obey design rule constraints" is retired** and replaced by the Escalation choice
  (`Off / Board minimums / Fab floor`) next to the Fab Tier choice. The old box did
  nothing the engine could see; its spin-control clamping is subsumed because
  `draw_size` floors at the board minimum in the engine, for both fronts.
- The per-size override checkboxes keep their meaning ("custom size" vs "use netclass
  values"), which is the PNS toolbar's own distinction and what users already know.
  Unchecked now resolves through `draw_size` (aggregate class per net, floored at the
  board minimum), so the reporter's Board Setup 0.25 finally binds.
- `Fab Tier` gains `auto`; `--strict-sizes` gets a checkbox; results panel and log get
  the disclosure block; hole-to-hole control relabelled.
- New params (`escalation`, `strict_sizes`, `clearance_ceiling`) are named after the
  controls, added to `reset_params_to_defaults`, `settings_persistence`, and
  `manifest_to_plan.FLAG_PARAMS`, per the CLAUDE.md three-step rule.
- `DesignRules.from_pcbnew` reads minimums and classes from `BOARD_DESIGN_SETTINGS`
  (with the #782 `GetNetclasses`-omits-Default workaround in one place), per-item
  overrides from `GetLocalClearance` / `GetClearanceOverrides`, and the `.kicad_dru`
  from the file beside `board.GetFileName()`. An unsaved board gets a visible "no
  custom rules: board has no file" line instead of today's silence.

### 4.9 Drill aspect ratio (#502)

The fab profile gains `max_aspect_ratio` (8 standard / 10 advanced) and the resolver's
`hole_size` floor becomes `max(rule min, thickness / max_aspect_ratio)` using the
stackup thickness `kicad_parser` already sums. It is a floor like any other, so the
via clamp, the escalation ladder and `check_drc` all honour it through the same path
with no special case.

---

## 5. What each issue gets

| Issue | Resolution |
|---|---|
| #842 | Board Setup minimum and netclass width both bind through `draw_size`; the terminal neck / rescue / escalation are gated by `--escalation`; the writeback never lowers the netclass width, so the ratchet is gone; the GUI checkbox that did nothing is replaced by one that does; deliveries below request are listed. |
| #857 | `--fab-tier standard` is hard; `auto` is the opt-in; `fab_tier_escalations` and the smallest sizes used are in `JSON_SUMMARY`, printed once at the end, shown in the GUI; `--strict-sizes` fails the run. The grader no longer relaxes with the escalation. |
| #530 | The resolver *is* the precedence chain, encoded from the engine source (§2) and pinned by the agreement harness (§7.1); every constraint kind is parsed; `opt`/`max` are honoured; per-net via sizing exists; unsupported rules are named. |
| #856 | Routing steps never write severities; the standalone relaxer is opt-in, logged and reversible. |
| #770 | The predicate subset covers every rule in the five real files on this machine; whatever remains unsupported is announced per board. |
| #502 | Aspect ratio is a hole-size floor in the same resolver. |

---

## 6. Migration and the parts that need care

### 6.1 Zero-behaviour-change first

The resolver lands with `resolve('clearance', ...)` reproducing today's numbers
byte-for-byte for the clearance chain (shims delegate; the corpus A/B must be
identical). Only then are semantics changed, one knob at a time, each behind its own
A/B.

### 6.2 Tests that pin today's behaviour and will move

- `tests/test_fab_tiers.py:33-70` (ladder lengths; `standard` returns one rung unless
  `auto`), `:55-62` (2-layer standard 0.127 stays).
- `tests/test_620_pending_via_pairs.py:514-522` (asserts the literal escalation warning
  string) and `:895-906` (override file = one rung; still true).
- `tests/test_rescue_width_provenance.py` (narrowing must be reported; now also gated).
- `tests/gui_parity/test_geometry_floor_leak.py`, `probe_effective_floors.py`,
  `test_gui_engine_parity.py:207-213` (unchecked box → Default netclass; becomes →
  `draw_size`).
- `tests/test_run14_fab_floor_origin.py`, `test_run8_fab_floor_disclosure.py`,
  `test_fix_drc_settings.py`, `test_505_min_connection_clamp.py` (writeback shape).
- `tests/test_kicad_dru.py` (0.05 rule pinned to the fab floor 0.127 → pinned to the
  board `min_clearance`, then the fab floor only under `fab`).
- `tests/test_local_clearance_overrides_326.py`, `test_697_placement_pad_clearance.py`,
  `test_725_fanout_clearance_pad_floors.py` (pad override = replace, not max; the
  fiducial-keep-clear case is unchanged because its override is *larger*).

### 6.3 Deliberate non-unifications to preserve

- `rip_restore.py:69-75` prices restored copper at what the router stamped
  (`config.track_clearances`), not pair-exact. The shim keeps that.
- `legality.py:1560-1568` excludes track rules from the pad model. Correct in KiCad's
  terms too (`A.Type=='track'` never binds a pad), so the resolver answers the same.

### 6.4 The clearance `max` vs `replace` change for pad overrides

Pads that declare a clearance **below** their class are the only case where routed
copper changes. Measured on the corpus (parser-resolved `pad.local_clearance` vs the
Default class clearance):

| | |
|---|---|
| pads with a positive local clearance | 4568 |
| ...below their board's Default class | **2932 pads on 48 boards** |
| typical shape | fine-pitch BGA/QFN footprints at 0.05 to 0.15 mm on a 0.2 mm class (spartan6_6layer 324 pads, crazyflie_fpga_deck 324, fpga_sdram 260, hackrf_one 259, hackgdl_badge 176) |

So it is not rare, and the direction matters: today's `max(rules, local)` prices these
pads at the *class* clearance, wider than KiCad requires, so the router is conservative
around exactly the packages whose escapes are hardest. Adopting KiCad's override
semantics can only *gain* routability there and cannot manufacture a KiCad violation;
`check_drc` relaxes the same way and must, or it would keep flagging copper KiCad
accepts. Ship it with the agreement harness row for this boundary (§7.1) as the gate,
since this is precisely the boundary #530 says it could not verify.

### 6.5 Per-net via sizes: N via bitmaps in Rust (decision 4)

The Rust router sees vias only as two pre-stamped bitmaps (`blocked_vias`,
`blocked_vias_small`, `rust_router/src/obstacle_map.rs`) plus a per-search `via_rung`
(`router.rs`). Per-net via sizing generalises that pair to **N via-legality bitmaps**,
one per distinct resolved via geometry on the board (corpus boards have 1 to 3), with
each search told which rung its net uses. Python stamps the N maps (the obstacle cache
already keys via stamps by size), passes them once per run, and routes every net in one
pass with its own via size. The escalation ladder becomes "try rung k, then k+1" inside
the same map set, which retires the #568 two-rung special case rather than extending it.

Costs, per CLAUDE.md: bump `rust_router/Cargo.toml`, note it in
`rust_router/README.md`, keep the release triple (`Cargo.toml` + `/VERSION` +
`metadata.json`) aligned, rebuild with `build_router.py --from-source`, and publish
per-platform binaries in a release before the Python side depends on the new
signature. Python must keep working against the previous binary until that release is
out: a version gate on `grid_router.__version__` selects the two-bitmap path (today's)
or the N-bitmap path, so a stale prebuilt degrades to today's behaviour with a printed
note rather than an import error. The Python-only alternative (group nets by via
geometry, rebuild the map per group, route groups in sequence) stays documented here as
the fallback if the Rust release slips.

### 6.6 Docs that state the old model

`CLAUDE.md` (net-class, `--clearance`, `.kicad_dru`, `local_clearance` paragraphs),
`docs/api-routing-config.md`, `docs/api-kicad-parser.md` (`local_clearance` "max"),
`docs/configuration.md` (Fab Tier Options claims the CLI *errors* below the floor; it
pins), `docs/utilities.md` (check_drc checks 11/12; "min_clearance is unreliable and
unenforced", which KiCad contradicts), `docs/route-plane.md`, `docs/python-api.md`,
`README.md`, the `kicad_dru.py` docstring ("last-to-first, first match"; the engine is
forward, per-field), `tests/README.md`, `tests/stress/README.md` ("netclass
`track_width` is a minimum": wrong) and `RUNBOOK.md` ("the router does NOT read any of
this": stale since #439), `py_placer/placement/README.md`, and the two skills that
describe fab sizing (`plan-pcb-routing`, `find-high-speed-nets`).

---

## 7. Verification

### 7.1 Constraint-agreement harness (ships first, #530 §0)

`tests/oracle/constraint_agreement.py`: for a fixture board and a constraint kind,
stamp a probe pair at `KRT_resolved − ε` and `+ ε`, run `kicad-cli pcb drc
--refill-zones` (reusing `kicad_oracle.py`'s detection, timeout and staging), and
require the `−ε` probe flagged and the `+ε` probe clean. Fixture matrix, one board per
boundary: netclass alone; netclass + tightening rule; netclass + relaxing rule; pad
override below class (the boundary §2 verified in source and this harness measures);
board `min_*` above a relaxing rule for a size kind (KiCad lets the rule win; we must
too); netclass-scoped and netname-scoped conditions; multi-class net with priorities;
`(layer inner)` scoping. KiCad is installed on this machine, so the harness runs
locally today. It becomes a permanent gate wherever `kicad-cli` exists and skips with a
note elsewhere, mirroring the existing oracle pattern.

### 7.2 Unit and parity gates

- `tests/test_design_rules.py`: one case per tier boundary in §4.1, each citing the
  harness row that established it; parse matrix per kind × scope; the unsupported list.
- `tests/gui_parity/test_design_rules_loader_parity.py`: `from_project` vs
  `from_pcbnew` table identity on the in-repo boards and on a board with a
  `.kicad_dru`.
- Existing gates, re-run: `test_manifest_plan_parity.py` (new flags),
  `test_cli_postpass_coverage.py` (the writeback core), `test_settings_roundtrip.py`
  (new controls, one removed), `test_gui_engine_parity.py` (the `KICAD_DUMP_BATCH_KWARGS`
  diff will show the config key change; read it, do not trust the exit code).
- `.gui-parity-checked` updated at the end of each phase.

### 7.3 Corpus A/B, per phase

`tests/stress/ab_replay_grade.py` over the recorded sets, grading DRC at the routed
clearance and connectivity with `check_connected`, comparing only boards that replayed
an identical chain. Decision rules pre-registered per phase:

- Phase 1 (resolver, shims): copper identical on every board.
- Phase 2 (writeback and severities): copper identical; `kicad-cli` error counts rise
  only in the categories un-silenced (expected, that is the fix).
- Phase 3 (`--escalation board` default, fab tier hard): completion may drop on boards
  that relied on tier escalation; the report must attribute every lost net to a named
  floor. Ship if the loss is confined to boards whose declared floors the old code was
  violating; otherwise keep `auto` as the default for one release with the disclosure
  on, and revisit.
- Phase 4 (`.kicad_dru` kinds and predicates): the set28 `mez_rx` and `storm_tracker`
  boards, plus `vme-wren` and the Haasoscope Pro board from `~/Documents`, are the
  fixtures; every rule in them must either bind (harness-verified) or be named.
- Phase 5 (per-net via): 183 boards change copper by construction; grade on
  `blocking` first, then vias/copper_mm, per the placement-grading rules in CLAUDE.md.
- Phase 6 (`--clearance` semantics): the A/B that decides §4.6.

A two-board result is not a default change.

---

## 8. Shipping order

| Phase | Content | Size | Behaviour change |
|---|---|---|---|
| 0 | Agreement harness + fixture matrix | S | none |
| 1 | `design_rules.py` resolver; `GridRouteConfig.rules`; shims; strict mode; loader parity gate | M | none (byte-identical A/B) |
| 2 | Writeback: no netclass draw-size lowering; severities out of routing steps (`--relax-severities` standalone); `project_writes` disclosure | S | #856, half of #842 |
| 3 | `--escalation` + `--fab-tier auto`; ladder chokepoint; reported refusals at the 18 sites (incl. the three silent ones); `JSON_SUMMARY` block; end-of-run line; GUI Escalation choice replaces Obey-DRC; `--strict-sizes` | M | #857, rest of #842 |
| 4 | Parser: all kinds, min/opt/max, predicate subset (phase-1 rows of §4.2), unsupported disclosure; `draw_size` consumes `opt`; `check_drc` on the resolver incl. `max` checks; `hole_to_hole` / edge / hole_clearance through the resolver | L | #530 coverage, #770 |
| 5 | Per-net via sizes: N via bitmaps in Rust (crate bump + binary release, version-gated Python path); `annular_width`; aspect ratio (#502) | M + release | 183 corpus boards |
| 6 | Predicate phase-2 rows (areas, groups, footprints, diff pairs); `disallow`, `via_count` | M | ruled boards only |
| 7 | `--clearance` / `--clearance-ceiling` split (gated by A/B); docs sweep; `.kicad_dru` fab-profile export | M | corpus-wide, gated |

Phases 2 and 3 are the ones users are waiting on and do not depend on phase 4; they can
ship on the resolver's shims.

**Status (branch `worktree-constraints-dru`, 2026-09-03):** phases 0, 1, 2, 3, 4, 5
and 7 are implemented and committed on the branch (phase 5 without the aspect-ratio
check, #502, and without the `annular_width` resolver kind, which the fab ladder's
`annular` floor still covers). Phase 6 is not started. The GUI engine parity gate,
the rp2350 live-chain gate, the settings round-trip and the 21-row agreement harness
are green on the branch head. The crate is 0.22.0 with `/VERSION` 0.22.0; `metadata.json`
and the per-platform binaries are the release step, not done on the branch.

---

## 9. Decisions taken (Andy, 2026-09-03)

1. **Default escalation policy: `board`.** The board's declared floors are the user's
   stated acceptance; keys the board leaves unset fall back to the fab tier floor for
   that key, disclosed. `off` and `fab` remain available on every front.
   **Amended 2026-09-03 after the sets 1-5 A/B (Andy): the defaults are `--fab-tier
   auto` and `--escalation fab`** -- the pre-#857 ladder, so default completion is
   maximal -- with everything the branch adds kept: the ledger, the `design_rules`
   summary block, the end-of-run line and `--strict-sizes`. The hard tiers and the
   `board` / `off` policies are the opt-in for a run that must not narrow. Measured
   reason: with the hard tier and `board` policy as defaults the 74-board corpus read
   +73 real DRC / +313 incomplete nets against 0.21.4, all of it policy (the engine-only
   arm under the old policy read -3 / -11).
2. **`--clearance` means the Default class clearance.** Non-Default classes are honoured
   pairwise as KiCad does. The #439 cap-every-class behaviour becomes the explicit
   `--clearance-ceiling` (and a second GUI checkbox). The switch ships gated by the
   phase 7 corpus A/B; if that A/B shows a completion regression on boards with
   aspirational classes, the fallback is to keep the ceiling under `--clearance` and ship
   only the explicit flag.
3. **Pad clearance overrides adopt KiCad's replace semantics now**, in the resolver's
   first phase, with the agreement-harness row for that boundary as the gate (§6.4).
4. **Per-net via sizes go into Rust as N bitmaps now** (§6.5), not the Python-only
   grouping. That is a crate bump, a `build_router.py --from-source` rebuild and
   re-published per-platform binaries, scheduled as its own release step in phase 5.
