# Floorplan intent, graded (#549)

Placement in this toolchain is judged by `crossings` and `hpwl`. Both are
indifferent between a sensible layout and a scattered one with the same
wirelength — and a render only moves the judgement from a number to a vibe.
Nothing declares where parts *belong*, so nothing can check whether they went
there.

`check_floorplan.py` is the check. You declare what the floorplan is meant to
be; it measures the board and exits non-zero with the number that broke.

```bash
# start from what the board already is, then edit it down
python3 py_tools/check_floorplan.py board.kicad_pcb --emit-intent floorplan.json

# grade
python3 py_tools/check_floorplan.py board.kicad_pcb --intent floorplan.json
python3 py_tools/check_floorplan.py board.kicad_pcb --intent floorplan.json --json findings.json
```

The intent is also a GENERATOR input, not only a grader's, and it now has
**three** distinct jobs:

1. **A construction source.** `place_seed.py` turns a declared intent into an
   initial placement for an unplaced board — zones, edge bands, keep-outs,
   locks and decap rules become placement constraints, and the emitted seed is
   graded against the same intent it was built from.
2. **A hard per-move gate inside the optimizer (#702).** Every CLI that
   quenches — `place_optimize.py`, `place_route_loop.py`, `place_seed.py`'s
   polish and `place_portfolio.py`'s per-candidate quench — takes `--intent`
   and refuses any move that breaks a declared zone, keep-out or exclusive
   zone. Before this, the intent reached `place_seed` and stopped, so the two
   tools that run the most quench iterations in a real chain could walk a part
   straight out of a zone the file declared. Measured on ulx3s: the seed grades
   clean, an ungated quench manufactures 4 `zone_containment` errors, a gated
   one manufactures none.
3. **A rank gate and a health source.** `place_portfolio.py` ranks K perturbed
   candidates only if they grade error-free, using the `health` signals in the
   rank key.

See `placement/README.md` for all of them.

**The gate is MONOTONE: it prevents a walk-out, it does not repair one.** A
part that arrives already outside its zone may improve or hold, never worsen —
so on a board whose seed already violates, the correct outcome is *the seed's
count, held*, not zero.

> **Careful with a self-emitted intent.** `--emit-intent` run on the board you
> are repairing records that board's damage as the requirement. Before #702
> that only mis-graded; now it also **steers the optimizer toward the damage**.

## The board outline is not editable by this toolchain

`envelope` is **read from the board**, never authored. A part outside it is a
finding about the **part**.

Board size, cutouts, slots and mounting-hole geometry are mechanical decisions —
enclosure fit, panel rails, connector apertures — and they belong to whoever owns
the mechanical design. If a board is genuinely too small for its parts, the
honest response is to say so with the measured number and stop. Nothing in this
module writes `Edge.Cuts`, and `--emit-intent` reports the cutouts as read-only
`context` so an editor can see what the parts must avoid without mistaking it for
something to change.

## Exit codes

| code | meaning |
|---|---|
| `0` | graded, no error-severity violations (or `--exit-zero`) |
| `1` | crash |
| `2` | argparse, **including an unreadable or malformed intent** |
| `3` | the board is not in a state this tool can work on — unplaced, or no trustworthy outline |
| `4` | graded successfully, **violations found** |

`4` rather than `1` because `3` is already taken by the placement-family
board-state gate and `1` is ambiguous with a crash. A caller has to be able to
tell *"the floorplan is wrong"* from *"the grader is broken"* — that distinction
is the entire product. (`check_drc.py` uses `1` for its violations; this diverges
knowingly.)

## The intent file

```jsonc
{
  "schema": 1, "kind": "floorplan-intent", "units": "mm",

  // READ from the board. A mismatch is a finding about the intent.
  "envelope": { "rect": [94.1, 61.42, 188.08, 112.22], "tolerance_mm": 0.5 },
  "defaults": { "zone_tolerance_mm": 0.5 },

  "blocks": [
    { "name": "power",
      "refs": ["U3", "C1?", "L1"],      // globs, the PRIMARY form
      "group": "sheet:58d913ec",        // optional; raw key or short_name
      "zone": [2, 2, 40, 30],
      "side": "F",
      "exclusive": false,
      "note": "buck; keep the switch node tight" }
  ],

  "keepouts": [
    { "name": "mount-NW", "rect": [0, 0, 6, 6], "sides": ["F","B"], "allow": ["MH1"] },
    { "name": "antenna",  "circle": [50, 5, 8], "sides": ["F"] }
  ],

  "edge_connectors": [
    { "ref": "J1", "edge": "north", "overhang_mm": { "min": 0.0, "max": 1.5 },
      "center_on_edge": { "tolerance_mm": 1.0 } },     // WHERE ALONG the edge...
    { "ref": "J2", "edge": "east",
      "along_edge_band": { "from": 0.10, "to": 0.35 } } // ...either form, never both
  ],

  "decaps": { "max_distance_mm": 2.5, "exempt": ["C99"] },
  "must_lock": ["MH*", "J1"],
  "legality_budget": { "overlap_area": 0.0, "oob_count": 0 },
  "severity": { "decap_distance": "warn" }
}
```

### Every key, and what happens to one you misspell

An unknown key is **refused at load time, at every level** — not dropped. A
typo'd key that is silently ignored is a constraint the author believes they
set and the grader never checks, which is the same failure `block_unresolved`
exists to prevent, one level down. The message names the key that was wrong and
lists the ones that were accepted:

```
edge_connectors[0]: unknown key(s) max_setback. Known: class, context, edge,
max_setback_mm, note, observed_overhang_mm, overhang_capped, overhang_mm, ref,
source, suspect, suspect_reason
```

| object | keys |
|---|---|
| top level | `schema`, `kind`, `board`, `units`, `min_reader`, `envelope`, `defaults`, `blocks`, `keepouts`, `edge_connectors`, `decaps`, `must_lock`, `legality_budget`, `health`, `severity`, `overlap_waivers`, `context` |
| `envelope` | `rect`, `tolerance_mm` |
| `defaults` | `zone_tolerance_mm` |
| `blocks[]` | `name`, `group`, `refs`, `zone`, `side`, `exclusive`, `tolerance_mm`, `note`, `context` |
| `keepouts[]` | `name`, `rect`, `circle`, `sides`, `allow`, `note`, `context` |
| `edge_connectors[]` | `ref`, `edge`, `overhang_mm`, `max_setback_mm`, `center_on_edge`, `along_edge_band`, `class`, `note`, `context`, and the emitter-written `source`, `suspect`, `suspect_reason`, `overhang_capped`, `observed_overhang_mm` |
| `edge_connectors[].overhang_mm` | `min`, `max` |
| `edge_connectors[].center_on_edge` | `tolerance_mm` (required — see below) |
| `edge_connectors[].along_edge_band` | `from`, `to` |
| `decaps` | `max_distance_mm`, `exempt`, `search_radius_mm`, `max_pin_distance_mm`, `pin_functions`, `same_side` |
| `legality_budget` | `overlap_area`, `oob_count`, `oob_amount` (`oob_area` refused — see below) |
| `health` | `bus_corridors`, `classes`, `block_displacement_mm`, `ignore_net_ids`, `max_fanout`, `zoned_blocks`, `affinity_exempt_nets`, `affinity_exempt_net_ids`, `plane_layers` |
| `health.bus_corridors[]` | `name`, `nets`, `width_mm` |
| `severity` | any of the 18 rule names below |
| `overlap_waivers[]` | `pair`, `reason`, `context` |
| `must_lock` | a list of reference globs (no nested keys) |

`severity` keys are checked too. The settable names are the eleven rules —
`envelope`, `zone_containment`, `zone_side`, `zone_exclusive`, `keepout`,
`edge_connector`, `decap_distance`, `decap_ungraded`, `decap_pin_distance`,
`must_lock`, `legality` — plus the five findings raised outside the rule
loop: `intent_zone_outside_envelope`, `intent_zone_overlap`,
`block_unresolved`, `intent_zone_in_keepout`, `keepout_allow_unresolved`,
plus two more that `rule_decap_pin_distance` raises BESIDE its own name —
`decap_pin_distance_inferred` and `decap_pin_uncovered` (#705). One
measurement can support several claims, and an author must be able to set
their severities apart: a pin inferred from a net name and a pin the pad
declares are not the same evidence. `decap_ungraded`,
`decap_pin_distance_inferred` and `decap_pin_uncovered` default to
**warn**; all but those and the last of the five default to **error**;
`keepout_allow_unresolved` defaults to **warn** and is upgraded by writing
`{"keepout_allow_unresolved": "error"}`.
`{"decap_distanc": "warn"}` is a
demotion that never happens, so it is refused rather than accepted.

### `context` is the slot for everything that is not a claim

`context` is **deliberately open** — free-form keys, at the top level and on
every `blocks[]`, `keepouts[]`, `edge_connectors[]` and `overlap_waivers[]`
entry. Nothing reads it and no rule grades it; it is where a run records why a
claim is what it is:

```jsonc
{ "ref": "J1", "edge": "east",
  "context": { "why": "the mating face is on the east wall of the enclosure",
               "rejected_alternative": "north, blocked by the display window" } }
```

It is open for the same reason everything else is strict. With nowhere to put
reasoning, it drifts into the graded keys — a recorded run had
`edge_connectors[]` entries carrying `band_basis`, `why`, `why_not_repaired`
and `rejected_alternative`, which every consumer ignored. `note` is not the
place either: it is grepped for the substring `SUSPECT`, so appending prose to
it can change behaviour. A `context` value must still be an object; the keys
inside it are yours.

### Versioning: `schema` is the format, `min_reader` is the vocabulary

`schema` is matched **exactly**, so bumping it invalidates every existing
intent file at once — far too blunt for "this build learned a new field".

So field-level compatibility is a second number. `READER_VERSION` (currently
`2`) is what this build can act on, and an intent sets `min_reader` when a
claim must not be silently ignored:

```jsonc
{ "schema": 1, "kind": "floorplan-intent", "min_reader": 2, ... }
```

A build whose `READER_VERSION` is lower **refuses the file** rather than
grading it without the claim — checked before anything else is read, because
grading it halfway is the same wrong answer as grading it fully. A build older
than the field itself refuses `min_reader` as an unknown top-level key, which
is the same answer.

The policy, and what each half is for. Refusing unknown keys already covers
**new** fields: an older build does not know the key, so it refuses the file
outright and says which key it did not understand. That is loud, automatic,
and needs no `min_reader`.

What refusal cannot see is a key an older build *does* know:

- its accepted **values** widened (a new `edge` direction, a new `class`);
- its **meaning** changed, so an old build acts on it differently;
- a **default** changed, so the same file grades differently than intended.

Those are what `min_reader` is for, and the file's author is the only one who
can know it applies. `READER_VERSION` bumps in the commit that makes such a
change, and files depending on it declare `min_reader`.

**Reader 2 arrived with [#712](https://github.com/drandyhaas/KiCadRoutingTools/issues/712):**
`edge_connectors[].center_on_edge` and `.along_edge_band`. By the paragraph
above the bump was not needed for *safety* — a reader-1 build refuses the file
outright with `unknown key(s) center_on_edge`, which is loud and automatic. It
was needed because `READER_VERSION` is the number an author copies into
`min_reader`, and at `1` a document could have claimed a reader-1 build acts on
a claim it has never heard of. That would be a false statement in the one field
whose only job is to be true.

### WHERE ALONG the edge: `center_on_edge` and `along_edge_band`

`edge_connector` grades the overhang band, the nearest-edge identity and a
setback. All three are satisfied *anywhere along* the edge — so a receptacle
well off the centre of its edge grades exactly as well as a centred one. On a
stick-shaped board where the PCB is the plug body, that is the difference
between a product and something a human rejects on sight. Measured on this
repo's own `esp_prog`: USB1 sits **1.75 mm off the centre of its 14.50 mm east
edge**, 12.07 %, and no conjunct could say so.

```jsonc
{ "ref": "USB1", "edge": "east", "center_on_edge": { "tolerance_mm": 0.5 } }
{ "ref": "CON2", "edge": "south", "along_edge_band": { "from": 0.30, "to": 0.45 } }
```

`along_edge_band` is the general form — `from` and `to` are fractions of that
edge's own span, so a declaration survives an outline resize. `center_on_edge`
is sugar for a symmetric band around the midpoint. **Both on one entry is
refused at load**, because it is two bands on one claim with no rule for which
wins. (The issue proposed refusing this in `validate_intent`; it is refused in
`intent_from_dict` instead, because `validate_intent` has exactly one caller —
`grade()` — while `place_seed` and `place_reconstruct` load an intent and never
call it, and those are the tools that *act* on this field.)

**`tolerance_mm` is required and has no default.** Measured on this repo's own
tracked boards, tigard's three connectors sit at **+16.1 %, −25.4 % and
−28.7 %** off their edge centres, and ulx3s's `J1`/`J2` sit at ±0.0 %. A
threshold this tool picked would fail a good human board three times out of
three. The author supplies the number or there is no claim.

**The offset is measured on every entry that names an `edge`, declared or not**,
and reported as `edge_seating` in `--json` and in the text output — mm and
percent of the span. Only a *declared* `center_on_edge` / `along_edge_band`
produces a violation. An `--emit-intent` document never carries either field,
so every intent written before this change grades identically.

**The span comes from the outline ring, and abstains rather than guess.** The
edge's span is resolved from the board's own Edge.Cuts ring when one parsed —
the contiguous run lying on that side. Only when there are no rings *and* the
outline is a simple rectangle (where the bounding box **is** the outline
exactly) is the bbox used. Anything else abstains with the reason, in the same
"declared value(s) NOT DERIVABLE — not graded, not passed" channel as a
withheld budget. The measurement that forced this: on `interf_u_unrouted_placed`
the bbox south edge spans 115.57 mm while the board's real south edge spans
81.28 mm, and `BUS1` — which sits *exactly* on the real centre — reads
**5.715 mm off** against the bounding box. A silently wrong centring number is
worse than none. Note the converse, also measured: `watchy` is not a simple
rectangle, yet its east and west ring spans are identical to the bbox on all 13
of its entries, so abstaining per *board* rather than per *edge* would throw
away 13 correct measurements.

### `refs` is the primitive, not `group`

Sheet group keys are opaque uuid paths — KiCad's `Sheetname` property is absent
from every corpus board — so nothing can author `"group": "sheet:1a2b3c4d"`
without listing the board first. `group` is accepted and matched against **both**
the raw `derive_groups` key and its `short_name` form (what `--list-groups`
prints, and therefore what anyone would copy), but reference globs are what a
human or a model can actually write.

### A block that resolves to nothing is an error

Not an empty block. A typo'd `refs` matches nothing, every rule over it iterates
an empty set, and the board grades clean while nothing was checked. That is the
exact failure this tool exists to prevent, so it is reported as
`block_unresolved` at error severity.

### An `allow` that exempts nothing is a warning

`allow` is a glob tuple, matched with `fnmatch` against every reference. Nothing
used to check that a pattern matched anything, so this

```jsonc
{ "name": "mount-NW", "rect": [0, 0, 6, 6], "allow": ["MH01"] }
```

on a board whose mounting hole is `MH1` exempted **nothing**. Since
[#701](https://github.com/drandyhaas/KiCadRoutingTools/issues/701) the seat
search consults the same resolver, so the typo does not merely fail to excuse
the part — it **strands the part the keep-out was drawn around**.

Measured on `splitflap_driver`, before the finding existed, in the two states an
intent is read in:

| the keep-out covers | what the author saw |
|---|---|
| the part it names | `[ERROR] keepout: H1 (F) is inside keep-out 'mount-NW'` — about the part, never the exemption. The failing pattern reached the JSON in `expected.allow` and the printed text never |
| empty space (the state a board is in *before* placement, which is when an intent is authored) | `violations 0, errors 0, pass true`, exit 0 — nothing at all |

Reported at **warn**, not error: a spec written before a refdes renumber is a
real case, and it should be loud without being fatal. Per **pattern**, not per
entry, so `allow: ["MH1", "MH01"]` still reports the typo. "Resolved" means the
pattern matches *some* reference — deliberately not "the exemption changes an
outcome", because a pattern naming a real part the keep-out would not have bound
anyway (wrong side) is not a typo.

### `rules_run` and `rules_skipped`

Both are in the `JSON_SUMMARY`. **"0 violations" and "0 rules ran" must not look
the same to a machine** — a caller reading only `pass` would treat a fully
skipped grade as a clean board. A rule is skipped when nothing in the intent asks
for it, and the reason is printed:

```
  4 rule(s) did not run:
    - decap_distance: the intent declares no decaps.max_distance_mm
    - keepout: the intent declares no keepouts
```

## The rules

| rule | fires when | measured with |
|---|---|---|
| `envelope` | the declared envelope is not the board's outline | `board_bounds` |
| `zone_containment` | a member's courtyard leaves its block's zone | `GradedPart.rect`. **Enforced, not only graded, since [#702](https://github.com/drandyhaas/KiCadRoutingTools/issues/702)** — the quench refuses such a MOVE, through the same `zone_escape` this rule calls |
| `zone_side` | a member is on the other face | `legality.footprint_side` |
| `zone_exclusive` | a non-member intrudes on a reserved zone | `rect_overlap_area`, **courtyard only** — a through-hole stranger's leads may cross a reserved zone, unlike a keep-out's. **Enforced since [#702](https://github.com/drandyhaas/KiCadRoutingTools/issues/702)**, same way — and since [#797](https://github.com/drandyhaas/KiCadRoutingTools/issues/797) the seat search refuses such a pose too, with the verdict `zone_exclusive_blocks` |
| `keepout` | any part enters a keep-out, unless in `allow` | courtyard **and** through-hole rect. **Enforced, not only graded, since [#701](https://github.com/drandyhaas/KiCadRoutingTools/issues/701)** — the seat search refuses such a pose through the same `keepout_hit` this rule calls — and since [#702](https://github.com/drandyhaas/KiCadRoutingTools/issues/702) the quench refuses such a MOVE through it too |
| `edge_connector` | overhang outside `[min,max]`, or the wrong edge; a `connector_affinity` entry seated more than 3 mm from every edge fires at **warn** whatever the configured severity | `BoardOutlineGate.rect_outside_amount`, `edge_clearance` |
| `decap_distance` | a decoupling cap is too far from its own IC | `groups.decap_populations` (`near`) |
| `decap_ungraded` | a cap in scope lies BEYOND the tether search radius, so `decap_distance` never measured it against the declared limit — a claim about COVERAGE, not compliance. **warn** by default ([#794](https://github.com/drandyhaas/KiCadRoutingTools/issues/794)) | `groups.decap_populations` (`beyond`) |
| `decap_pin_distance` | a DECLARED supply pin is further than `max_pin_distance_mm` from the nearest decoupling cap on its own rail, pad edge to pad edge ([#705](https://github.com/drandyhaas/KiCadRoutingTools/issues/705)) | `floorplan.supply_pins`, `legality.pad_rect` + `rect_gap` |
| `decap_pin_distance_inferred` | the same measurement for a pin inferred from a net NAME rather than from a `pintype` or `pinfunction`. **warn** by default, because the pin set is the inference | same |
| `decap_pin_uncovered` | a declared supply pin's rail carries no decoupling cap at all, anywhere. A design fact, not a placement failure, so **warn** and per (IC, rail) rather than per pin | same |
| `must_lock` | a declared-critical part is not locked in the file | `parser.extract_locked_refs` |
| `legality` | overlap or off-board parts exceed a budget | `QuenchState.legality_metrics` |
| `block_unresolved` | a block matched no footprint | — |
| `intent_zone_in_keepout` | a declared zone is contradicted by a keep-out that binds its members: covered entirely (reported per block), or left with no pose for a member at any rotation (per member) | `zone_covered_by_keepout`, then `zone_pose_feasibility` |
| `keepout_allow_unresolved` | a keep-out's `allow` pattern matches no footprint (**warn** by default) | `allow_pattern_matches`, the resolver's own matcher |
| `intent_zone_overlap`, `intent_zone_outside_envelope` | the intent contradicts itself (no board needed) | — |

Every one of them measures with the geometry the **optimizer itself gates on**.
A grader with its own idea of what "legal" means grades the reimplementation
rather than the board — so where re-deriving was plausible, a test asserts the
two agree exactly (all 39 of ulx3s's NON-ZERO cap distances identical to
the grouper's — of 53 tethers; the other 14 clamp to 0 inside their IC's
bounding box and a 0 mm limit does not flag them — and all five legality
numbers identical to the quench's). The count read **34** here until #705
re-derived it; the test never asserted a number, so nothing caught it.

### Which rules the SEARCH can see

A constraint the search cannot see can only ever produce a failing grade. This
is the whole table, and the "no" column is the part worth reading — each of
them is a decision, not an omission.

There is a **third** consumer beside the two columns below, added by
[#698](https://github.com/drandyhaas/KiCadRoutingTools/issues/698):
`place_seed --reseat REF` measures the same three enforced rules *before and
after* the pass, through the same `zone_escape` / `keepout_hit` /
`rect_overlap_area` the grade calls, and uses them two ways — the per-term
**vector** as a licence (no declared claim may get worse, termwise) and the
breach **count** as one of the terms an explicit re-seat may be accepted *on*.
It is a measurement, not a per-pose gate: arming the monotone zone gate on that
path would make the re-seat refuse its own target, which is why
`pose_score.make_state` hands it keep-outs and withholds zones. `reseat_scope`
reports the whole picture in `accept_basis`.

| rule | graded | seat search (#701) | quench gate (#702) | if not, why not |
|---|---|---|---|---|
| `zone_containment` | yes | via `zone_gate` | **yes** | — |
| `zone_exclusive` | yes | **yes** | **yes** | — |
| `keepout` | yes | **yes** | **yes** | — |
| `must_lock` | yes | — | **by freezing** | it is a claim about the FILE; no pose satisfies or violates it |
| `edge_connector` | yes | anchor tier | **by freezing** | two of its three sub-claims are bounds on being *off* the board, so a per-pose term would fight the containment gate rather than complement it |
| `zone_side` | yes | — | no | **vacuous, not conservative**: the quench never flips a side, so the term is invariant under every move it can make. Reported once at load instead |
| `envelope` | yes | — | no | a claim about the intent FILE against the board, not about any pose |
| `decap_ungraded`, `decap_pin_*` | yes | — | no | `decap_ungraded` is a claim about what the GRADE covers rather than about any pose, so there is nothing for a search to refuse. The pin rules are a THIRD currency — pad edge to pad edge on one net — and the objection below applies to them more strongly, not less |
| `decap_distance` | yes | scope stage | no | graded in a currency the optimizer does not carry — pad centroid to an IC's pad bbox inflated 0.5 mm, not courtyard to courtyard. A gate in the wrong currency can *admit what the grade flags*, which is worse than no gate. And the cap→IC tether is re-elected from live poses, so a per-move form would have the `corridor_weight` non-stationarity problem too |
| `legality` | yes | — | no | a whole-board aggregate against a BUDGET, so a per-pose form is non-local: whether A's move is admissible would depend on B's violation |

The two zone rows reach the seat search by **different channels**, and the
difference is the reason one of them could be gated and the other could not.
`zone_containment` is a *must-be-inside* claim, so it arrives as the per-call
`constraint` of `seeder.zone_gate` — anchor-aware, per-part, and never as a
state-wide gate, because a monotone one would make a repair refuse its own
target (`pose_score.make_state` withholds `intent_zones` from every seat state,
and `tests/test_698_reseat_acceptance.py` arm H parses `seeder.py` to keep it
that way). `zone_exclusive` is a *must-be-outside* claim whose target is clean
by definition, so it can be — and is — gated **absolutely**, through
`QuenchState.exclusive_clear` over the `zone_exclusive` slice of the same
`build_zone_spec` the quench gate reads — one resolution and one measurement,
so neither can invent its own idea of who is a member.

They are not identical, and the one difference is deliberate. The seat slice
drops a zone whose members did **not resolve**; the quench's per-move channel
does not. "Stranger" means "not a member", so an unresolved block makes every
part one, and a gate that can STRAND a part must not act on a claim it cannot
attribute — while the quench only ever declines a move, so it cannot strand
anything. `rule_zone_exclusive` carries the same filter, which is what keeps
the pair that matters in step: a seat the search accepts and the grade then
flags is an exit 4 on a correct board, and that is the round trip this table's
`keepout` row records closing for #701.

Enforcing is not free, and the price is recorded rather than described:
`tests/test_placement_ab.py` carries four `intent-*` rows and
`tests/placement_ab_baseline.json` the numbers. A hard constraint removes poses
from the search, so `crossings` and `hpwl` can get worse — on ulx3s they do,
and that row is pinned `regress` with the containment errors going 4 → 0.

### A through-hole part is in a keep-out from either side

Its leads pass through. `keepout` tests the courtyard **and** the drilled-pad
rect against every face the part occupies, so a mounting-hole keep-out cannot be
walked through from the back.

### A zone a keep-out leaves no room in is refused

[#702](https://github.com/drandyhaas/KiCadRoutingTools/issues/702) refuses an
intent whose keep-out covers a declared zone **entirely**. The question
underneath is *does the zone minus the binding keep-outs still hold this part at
some rotation*, and the two coincide only at total coverage. Measured before
[#799](https://github.com/drandyhaas/KiCadRoutingTools/issues/799): zone
`[10,10,20,20]` at `tolerance_mm: 0` against a keep-out `[10,10,19.9,20]` — 99%
of it — raised nothing while the member had **zero** satisfying poses, and two
keep-outs covering half each were missed the same way.

It has to be refused where it is authored because the #702 quench gate is
termwise-monotone: `keepout` falls only by leaving the zone, leaving raises
`zone_containment`, and no candidate lowers both. Such a member is **confined to
its zone** for the whole run — confined, not frozen: every pose inside the zone
scores identically, so the rule admits all of them. What it can never do is get
out.

Reported **per member**, since the answer differs per member: a 2×2 part and an
8×8 part in the same zone under the same keep-out disagree. Total coverage is
still reported once per block, with its original message.

What the check does **not** claim: it models the zone and the intent's own
keep-outs, and deliberately not the board outline, clearance or neighbours.
A feasible verdict says there is room, never that the part can be seated —
`place_seed`'s `keepout_blocks` verdict answers that, with the seat predicate
and a pose count. Two consequences are disclosed rather than hidden:

- **circle keep-outs never refuse.** Nothing in this tree computes disc/rect
  free area (`keepout_hit` returns a marker for a disc and says so), so a
  refusal resting on one would be unsound. When the rects alone still refuse,
  the refusal stands; only when dropping the discs would have found a pose is
  the answer undecided. Total disc coverage is still caught exactly, as before.
- **a part with no courtyard is modelled by its pad bbox**, which is smaller, so
  the check under-reports rather than over-reports on such a part.

### `oob_area` cannot be budgeted, and says so

`legality_budget.oob_area` is **refused at load time**:

```
legality_budget.oob_area: not gateable. out_of_board_area is measured against
the bounding-box inset, so a part sitting inside a CUTOUT scores 0.0 area and
would grade clean. Use oob_count or oob_amount, which both see the real
Edge.Cuts rings.
```

`out_of_board_area` measures against the rectangular usable inset — its own
docstring calls itself *"a lower bound on a notched one"*. A part sitting
entirely inside a milled slot scores `oob_count=1, oob_amount>0, oob_area=0.0`.
Refused loudly rather than ignored, so the reason reaches whoever wrote it.

## A board whose outline did not parse is refused, not graded

A broken outline degrades **silently**: unclosable segment groups are dropped,
`extract_board_contours` returns `([], [])`, `BoardOutlineGate.active` goes
`False`, and every containment check quietly falls back to the bounding box. No
exception, no warning. A grader that inherits that reports a clean board because
it stopped checking.

So `outline_state` validates the envelope before anything is graded against it,
and the run exits **3** rather than producing a verdict. It reproduces the
parser's own simple-rectangle short-circuit to tell the three "no rings" cases
apart, because a plain axis-aligned rectangular board **is** its bounding box
exactly — refusing that would refuse most of the corpus.

The case that motivated it is [#550](https://github.com/drandyhaas/KiCadRoutingTools/issues/550):
`extract_board_bounds` reads neither board-level `gr_circle` nor `gr_curve`, so a
round board reports `board_bounds: None` while its 64-point ring parses fine.

## `--health`: what tells you the floorplan is wrong

Separate from the rules, and advisory. An intent violation says *"this is not the
floorplan you declared"*; a health signal says *"this floorplan will fight the
router whatever you declared"*. That is
[discussion #407](https://github.com/drandyhaas/KiCadRoutingTools/discussions/407)'s
question — *knowing when to stop routing and go move something* — whose two scars
were a magnetics block 80 mm from both its own endpoints, and ~22 nets no knob
could fix because the answer was re-floorplanning a quadrant.

```jsonc
"health": {
  "block_displacement_mm": 15.0,
  "bus_corridors": [ { "name": "sdram", "nets": ["SDRAM_*"], "width_mm": 8.0 } ],
  "classes":      { "SDRAM": ["SDRAM_*"], "USB": ["USB_*"] }
}
```

| signal | computable | what it means |
|---|---|---|
| **block displacement** | from geometry alone | the block's own pad centroid vs the centroid of everything it connects to. This is #459's "connectivity-centroid displacement" |
| **bus crossings** | pre-route, but the corridor is a **model** | a straight rectangle between the bus's two pad clusters; its long sides are fed to the quench's own crossing kernel. A screening signal, not a verdict — real routes bend |
| **convergence** | only with declared `classes` | which critical classes crowd one corridor. Placement has no net-class notion and "critical" is design intent, not a fact in the file, so **it is skipped rather than guessed** |
| **net affinity** | from geometry alone | the per-PART inverse of block displacement: which single part carries a net its own block sits away from. See below |
| **escape lanes** | from geometry alone | per fine-pitch part, per face: lanes that fit against nets that must leave. Needs no declaration. See below |
| **blocked-cell share** | **not pre-route at all** | needs #409's blocker JSON, which only exists after a routing attempt. Reported as skipped, with that reason |

### `net_affinity`: the member a block metric averages away

Block displacement is an average over a block's members, so it is quiet when
*one* member is the problem. Measured on a real board: a series resistor zoned
into a far-edge block carried **85.7% of a critical bus net's routed length**,
forcing ten drop-vias and eight reference-plane voids. The block it sat in was
flagged as displaced; the resistor was not, and four runs went by before a
human found it.

Reported per (part, net), ranked, advisory. Two numbers reach `JSON_SUMMARY`:
`health_net_affinity_offenders` (rows that dominate a net *and* pierce a
declared corridor) and `health_net_affinity_worst_norm` (the largest
recoverable length as a fraction of the board diagonal — mm never compares
across boards).

Four entry conditions, none of them a tuned constant, because a diagnostic
that cries wolf is worse than none:

- the same fanout / `ignore_net_ids` cut as block displacement, so a rail never
  appears;
- the part must sit in a block **that has a zone** — without a declared seat
  there is nothing to blame for where it ended up;
- **three or more owning parts.** A two-owner net has one MST edge, incident on
  both parts, so `share` is 1.0 for each and dominance would mean nothing;
- moving the part onto its own net's centroid must free at least 10% of what it
  carries. A part in the MIDDLE of a net's span is incident on the edges either
  side of it and also scores 1.0, while being exactly where it belongs — the
  recoverable test is what separates a misplacement from a topological hub.

Locked parts are reported **with a flag**, not suppressed: "this cannot move"
is triage, not absence. `health.affinity_exempt_nets` (globs) silences a
deliberately long net.

`recoverable_mm` is a mechanical counterfactual, not a guess — the net's MST is
rebuilt with the part translated onto the centroid of the pads it talks to,
using the same override primitive the quench scores real moves with. It is an
upper bound: nothing checks that the part may legally sit there.

### Power and ground are excluded, and that is what makes these signals work

GND owns **96** of ulx3s's parts and `+3V3` owns **45**, out of 329 nets whose
*median* is 2. Left in:

- 8 of 10 blocks report a foreign-pad count within 10% of the same median — they
  are all seeing the board's power nets, so the "net centroid" is really the
  board centroid and displacement degenerates into *distance from the middle of
  the board*. Filtered, the median drops from 332 to 40 and the ranking changes.
- The same rails cross **every** corridor, because a 96-part MST sprays airwires
  board-wide. On ulx3s's SDRAM corridor the unfiltered top three offenders were
  `GND`, `+5V`, `+3V3` — a fiction, since those route on a plane rather than as
  traces through the channel. Filtered: 24 crossings → 18, and the offenders
  become real signal nets.

Pass `health.ignore_net_ids` to name the plane nets explicitly (as `--ignore-nets`
does elsewhere); `health.max_fanout` is the backstop, default 20.

### `escape_lanes`: the difference between "the router failed" and "this was never routable"

For each fine-pitch part, per face: lanes **supplied** (the face's usable span
divided by one track plus one clearance, at the board's own floor) against lanes
**demanded** (nets that must leave through it). A face in deficit is a *binding
constraint* — net ordering only chooses which nets strand there, never how many.
Runs have been spent on ordering experiments against a face this settles in
seconds.

Reported without any declaration, on every board: a face that cannot pass its
own nets is a fact about geometry, and requiring an opt-in would mean it is only
measured where someone already suspected it. `health_escape_deficit_parts` and
`health_escape_worst_deficit` reach `JSON_SUMMARY`.

Three things keep it honest:

- **The lane pitch is read from the board**, not assumed. At 0.20 mm a face
  passes and at 0.35 mm the same face is short, so a constant would manufacture
  or hide a structural finding depending on the board it met.
- **`blockers` names who ate the lanes.** A count says a face is short; the
  blocker list says which neighbour to move. Read that field first — it is the
  difference between a signal and an action. A neighbour is charged only if it
  shares a board face with the part (a drilled part occupies both) and is not a
  container — a module outline covering half the board, which the part sits
  inside rather than beside (#835). Before that, ulx3s reported six faces in
  deficit and every one of them was charged to copper on the other side. What
  a charged neighbour CONTRIBUTES is its pad copper (#841) — not its
  courtyard, which is an assembly keep-out a track may legally run under, and
  not the bbox of its pad centres, which gives a two-terminal passive a
  zero-width body.
- **Interior pads count toward no face** and are reported separately. A boxed-in
  pad does not escape sideways at any pitch; it needs a via. Rolling it into a
  face's demand would blame the face for a fanout problem.

#### The layer term (#700)

`supply` and `deficit` are **own-layer**, and stay that way. Beside them the
ledger now reports what the other signal layers could take:

    supply_other_max = min(via_slots, (signal_layers - 1) x lanes along the face)
    deficit_floor    = max(0, demand - supply - supply_other_max)

- **`deficit_floor` is a LOWER bound**, the dual of an upper bound on supply.
  `deficit_floor > 0` says the face is short *even using every other layer* — a
  strictly stronger verdict than `deficit > 0`. **`deficit_floor == 0` proves
  nothing**, which is why nothing gates on it and why the floorplan report
  prints its line only when it is positive.
- **`supply_bound` names which term binds**, and that is the actionable half.
  `via_slots` does not depend on the layer count, so the `min` saturates almost
  at once — on every in-repo board the answer at 2 signal layers equals the
  answer at 6. Reported as `via_slots`, that is a finding with an action (via
  geometry, underpad fanout, freeing span); hidden inside the `min()` it would
  be a number that mysteriously ignores the stackup, which is the complaint
  #700 was filed about one level up.
- **`via_slots` is measured on the face's full span, unobstructed**, while
  `supply` on the same row is not. Deliberate, and the reason has changed
  twice: it was that `blocked_mm` was side-blind and container-blind (#835
  fixed both), then that it modelled a neighbour by the bbox of its pad
  CENTRES (#841 fixed that). What remains is the asymmetry itself — `supply`
  is a floor and `via_slots` a ceiling, and feeding a `blocked_mm` that is too
  large into an upper bound would silently turn it into a lower one, which is
  outside this term's contract in a way over-stating is not.
- **`signal_layers` is observed or declared, never guessed.** A copper layer
  counts as a plane when a named-net, board-level zone covers most of the board.
  Placement runs *before* the pours exist, so on a board being placed the answer
  is usually "every copper layer" — which is **optimistic**, and
  `signal_layers_source` says so on every row. Declare `plane_layers` under
  `health` (e.g. `"plane_layers": ["In1.Cu", "In2.Cu"]`) for the real answer;
  it is spelled the way `health.ignore_net_ids` names the plane *nets*, and it
  is the channel that actually carries an answer on a board being placed.

Detection is by **pad pitch, not footprint name** (a house library carries no
pitch in its name) and is deliberately wider than the fanout test: fanout asks
"is this pad boxed in", which needs interior pads; escape asks "do this face's
pads fit through the channel beside it", which a fine-pitch *perimeter* part
fails with no interior pad at all. Through-hole parts are excluded — a THT pin
is reachable on every copper layer, so there is no escape to be short of.

A worked pairing wants two signals computed from different quantities naming
the same part: an escape face in deficit whose `blockers` names a neighbour,
and `net_affinity` reporting that same neighbour carrying most of the nets on
that face. That is the case worth acting on.

The example this section used to give was ulx3s `U9 west: supply 6 < demand 14`,
`15.35mm of that face is taken by SD1`. **It was an artifact and is kept here
as one:** U9 is on B.Cu and SD1 on F.Cu, so each was charged the other's whole
body across the board, and #835 removed the charge entirely —
`tests/test_835_escape_side_aware.py` now asserts that the pair charges each
other nowhere. A blocker list is only an action list if the names are real.

## What `--emit-intent` does and does not claim

It writes an intent that **grades clean by construction** — a baseline to
tighten, and the round trip is what proves the rules are wired to real geometry
rather than silently skipping.

It claims a `zone` only where it can defend one. A schematic sheet is a
*functional* grouping, not a spatial one: its members scatter across the board,
so per-sheet bounding boxes mutually overlap — all ten of ulx3s's do, by up to
4508 mm². Emitting those as zones produces an intent no placement could satisfy,
and it would be the emitter that was wrong. Zones are emitted only where
**disjoint**, tightest first (ulx3s: 4 of 10), clamped to the envelope; the rest
carry membership and say why they have no zone.

This is the same spatial incoherence that makes sheet blocks useless for
*movement* — see `placement/README.md`.

Parts already overhanging the outline are recorded as `edge_connectors` by
observation, which is what stops `oob_count` reporting a card edge or USB shell
as a defect forever.

With `--declare-classes`, connector-family parts that claim no edge (headers,
JST, terminal blocks; `part_class` calls them `connector_affinity`) are also
recorded, with `"class": "connector_affinity"` and **no `edge`** (naming one
would be an invention). The grade then flags such a part seated more than
3 mm from every edge at `warn` only, because legitimately interior connectors
exist; write `max_setback_mm` or `edge` on the entry to make it a real claim
at the configured severity.

These entries are **declarations, not seat claims**, and the placement engines
do not act on them. `edge_connectors` therefore holds two populations, and
`Intent.edge_claims()` is the split: it drops `connector_affinity` and is what
`place_seed` (which LOCKS edge refs for its polish quench), `place_reconstruct`
(banded off-outline allowance, exchange-stage exclusion) and
`reconstruct.classify` (the anchor tier) read. The `edge_connector` **rule**
reads the whole key, because flagging an interior pose is the entry's only
purpose. Writing `max_setback_mm` or `edge` on an entry changes its class-based
severity, not its membership -- to hand a part the edge-part treatment in the
engines, declare it with an edge class.

The `overlap_area` budget is **withheld** when the emitting board carries a
blocking body pair, or an unwaived courtyard interpenetration past the
blocking floors: baking the number would bless the board it was measured on.
`context.budget_withheld` names each withheld key and why, so a reader can tell
"withheld" from "forgot"; declare the budget by hand if the overlap is by
design.

A withheld key is **abstained at grade time, not passed**. `check_floorplan
--intent` reads `context.budget_withheld`, reports every withheld key that the
intent does not also declare under `N declared value(s) NOT DERIVABLE --
not graded, not passed`, and carries them as `budget_abstained` in `--json` and
as `budget_abstained` / `budget_abstained_keys` in `JSON_SUMMARY`. When the
whole budget is empty the `legality` rule does not run at all, and its skip
reason names the withholding rather than saying only "the intent declares no
legality_budget". Declaring the key by hand overrides the note: a declared
budget is graded.

**And it is not a pass in the machine channel either, since #713.** The human
line used to read `PASS: N rule(s) ran, no violations` twenty-six lines above
`NOT DERIVABLE -- not graded, not passed`, with `pass: true` and exit 0 --
measured on 5 of the 22 tracked boards in the default emit-then-grade round
trip. A grade that left a declared channel unmeasured now reports
`complete: false`, `not_graded: {channel: count}`, `pass: false`, prints
`INCOMPLETE` instead of `PASS`, and exits 4. `--exit-zero` suppresses the code
without changing the verdict, exactly as it does for violations.

`not_graded` names WHICH channel and never aggregates: `budget_abstained` (the
emitter could not derive a declared key), `rules_skipped_armed` (the intent
asked and this BOARD cannot answer) and `edge_seating_abstained` (a
measurement with no basis) are different claims. **`rules_skipped` is NOT
wholly in it**: its `_SKIP_REASON` entries all mean "the intent declares no X"
-- nobody asked, which is honest and fires on every board -- and counting
those would make any board with a minimal intent permanently incomplete.

The withholding channel is **not legality-only**. `_WITHHELD_RULE` maps each
withholdable key to the rule it disarms and to the test for "declared by hand
anyway", so a withheld `decaps.max_distance_mm` reaches `decap_distance`'s skip
reason exactly as `overlap_area` reaches `legality`'s. A key that is in
`budget_withheld` but in no such mapping still abstains — reported as not
derivable, blamed on no rule — rather than being dropped, so a typo'd
withholding note is visible instead of silent.

## `--declare-decaps`: a decap limit read off the board (#704)

`emit_intent` wrote `decaps: {}` as a constant, so `rule_decap_distance` could
never fire on an auto-emitted intent. With `--declare-decaps` it derives
`max_distance_mm` from the board's own tethers.

**The statistic is `ceil(max)`, and the argument is a fixed point, not a
statistic.** An emitted intent is a baseline to tighten: emit, grade clean,
re-emit, get the same limit. Only the max has that property. The median is
refuted by measurement — `glasgow_revC`'s tether median is **0.0000**, because
58 of its 87 caps sit inside their IC's bounding box and clamp to zero, so a
median limit flags 29 caps on a healthy human-routed board at the first
emission. A high percentile has no fixed point at all: it flags ~5% by
construction, forever, and every violation is then manufactured by the emitter
rather than found on the board.

`_ceil4`, not `round`, and derived from the **unrounded** max: `round(v, 4)`
can land below the measured value, so the document would fail against the very
board it was written from. That bites on 3 of the 9 tracked boards
(splitflap_driver 3.4617228369700497 → 3.4617, ulx3s 4.714904558949195 →
4.7149, glasgow_revC 4.786912496589008 → 4.7869). `context.decap_census`
therefore carries `max_mm` unrounded and `max_mm_display` for the reader.

### When it is withheld, and why the issue's own guard could not be used

The obvious guard — withhold when the emitting board already violates the
number — is **unreachable**: `ceil(max(observed))` is ≥ every observed value by
construction, so it would be a branch that never executes, which is worse than
absent because it reads like a guard.

The reachable analogue is **censoring**. `groups.decap_tethers` drops any cap
further than `DECAP_RADIUS_MM` (5 mm) from a chip carrying its rail, so the
observed distribution is censored and a limit read off the survivors can bless
a board whose caps have left decoupling range. Measured over the tracked
boards:

| board | tethers ≤5 mm | beyond | censored | max (≤5 mm) | worst beyond |
|---|---|---|---|---|---|
| tigard | 25 | 1 | 0.04 | 3.0650 | 6.17 |
| glasgow_revC | 87 | 5 | 0.05 | 4.7869 | 10.34 |
| splitflap_driver | 11 | 1 | 0.08 | 3.4617 | **19.30** |
| watchy | 24 | 2 | 0.08 | 3.7525 | 11.16 |
| ulx3s | 53 | 7 | 0.12 | 4.7149 | 12.24 |
| kit-dev-coldfire | 41 | 12 | 0.23 | 4.8990 | — |
| interf_u_unrouted | 1 | 4 | **0.80** | 4.5800 | 19.82 |
| sonde_u | **0** | 5 | **1.00** | — | 15.58 |

Healthy 0.04–0.23, degenerate 0.80–1.00. So the limit is withheld when there
are **no** tethers, **fewer than `DECAP_MIN_SAMPLE` (3)** — a max over one
sample is a coordinate, not a limit — or **more than `DECAP_MAX_CENSORED`
(0.25)** of the rail-sharing caps lie beyond the radius.

A rejected alternative, recorded so it is not re-proposed: withhold when
`max > K × p75` (the max is a tail outlier over its own body). Built and
measured, and it does **not** separate — healthy splitflap_driver scores 2.46
and mid-repair `tigard_placed` 2.22.

### What the census discloses that the rule cannot see

`context.decap_census` is written on **every** emission, with or without the
flag, so a reader of the document can tell "no cap is far from its IC" from
"nobody measured". It carries the metric, the search radius, the tether and IC
counts, the max and median, and — the point of it — `beyond_radius`,
`beyond_radius_refs` and `worst_beyond_mm`.

That last group was a **known, unfixed hole** until
[#794](https://github.com/drandyhaas/KiCadRoutingTools/issues/794): the emitter
and the rule both call `decap_tethers` at the same 5 mm truncation, so
`splitflap_driver` emitted a limit of 3.4618 while a rail-sharing cap sat
**19.30 mm** from its IC, invisible to both. `rules_run` went up; that cap was
still ungraded.

`decap_ungraded` closes it, and **not by widening the radius** — that path is
closed by measurement. Widen the rule alone and the emitter's `ceil(max)` no
longer covers the newly visible caps, so the round trip stops grading clean on
its own board; widen both and splitflap's limit goes 3.4618 → 19.30 and ulx3s's
4.7149 → 12.24, which is a limit that says nothing. So the caps beyond the
horizon get their own finding, at **warn**, whose claim is COVERAGE rather than
compliance: *your limit was derived from the caps inside the radius, so it says
nothing about this one.* Ten tracked boards emit 32 such findings; none becomes
an error, and `board_score`'s `blocking` is unmoved.

It names the IC with no hedge. The comment that used to stand in
`decap_census` claimed the unbounded pass was "a second measurement rather
than a superset" because the election could pick a different chip without the
prune — that was never true of the code (the radius only ever applied *after*
the argmin), and since #794 the radius does not reach the election at all.
Measured: 0 of 357 tethers re-elect.

A board whose limit is WITHHELD cannot arm either decap rule, so the horizon
reaches it through the abstention channel instead: `_WITHHELD_RULE` maps the
key to **both** rules, and the withholding reason names the beyond count and
the worst distance. That reason now carries the horizon on the
`DECAP_MIN_SAMPLE` arm too, not only the zero-tether one — without it
`interf_u_unrouted`, withheld for having one sample while carrying four caps
beyond the radius and a 19.82 mm worst, abstained with a note that said nothing
about them.

**`worst_beyond_mm` used to understate.** It read the last element of a list
sorted by cap REFERENCE, so it reported the alphabetically last beyond-cap
rather than the farthest — on 7 of the 14 boards that have any: kit-dev 10.80
where the truth is 22.89, interf_u 8.39 where it is 19.82, flat_hierarchy 6.44
where it is 18.42. The number appears in `--declare-decaps` stdout, in the
withholding reason and in the censoring table below, so all three understated
the thing the census exists to disclose. It survived because splitflap_driver —
the board every other census arm is written against — has exactly ONE
beyond-cap, where last and farthest are the same cap.

### Declaring this key CHANGES PLACEMENT

It is not a grading-only knob, which is why it is opt-in, default off, and on
its own flag rather than folded into `--declare-classes`. `place_seed` reads
`decaps.max_distance_mm` and, when it is set, pulls every two-net-bearing-pad
`C*` out of radial zone packing into its per-supply-pin stage. Measured:

| board | in scope | graded | beyond the radius | no rail-carrying chip | predicate |
|---|---|---|---|---|---|
| ulx3s | 70 | 53 | 7 | **10** | **0** |
| glasgow_revC | 92 | 87 | 5 | 0 | **0** |
| watchy | 28 | 24 | 2 | 2 | **0** |
| splitflap_driver | 12 | 11 | 1 | 0 | **0** |

**This table used to blame the gap on the predicates, and that was wrong.** The
text said the two populations differ "because the two 'is this a decap'
predicates differ — the grouper tests *exactly two distinct net ids*, the
seeder *exactly two net-bearing pads*". There were in fact THREE spellings (a
hybrid also lived in `emit_intent`), and measured over every tracked board all
three name **identical** sets: the residue attributable to them is **zero**, on
every board, and no tracked board even has a lowercase `c*` reference for the
case difference to bite on. `tests/test_792_decap_predicate.py` is the standing
measurement.

The gap is two other things, and the census now reports them separately
(`beyond_radius` and `no_rail_chip`, with `unaccounted` staying 0 so a reader
can check that the three arms are the whole scope):

* **beyond the 5 mm radius** — a real grading hole, and `decap_ungraded`'s
  business since #794;
* **no chip carries the cap's rail at all.** Ten of ulx3s's seventeen: `C3 C4
  C22` on `/power/P1V1`, `C7 C8 C24` on `/power/P3V3`, `C11 C12 C23` on
  `/power/P2V5`, `C14` on `/power/SHUT`. Those rails are owned only by two-pad
  passives — the caps themselves plus `L1`–`L3` and `RA*`/`RP*` — so they are
  bulk and filter caps upstream of an LC network, not decouplers. **The grader
  is right to ignore all ten**; there is no IC for them to be far from.

**The seeder was not right about them**, and that is what #792 actually fixes.
`place_seed`'s pin stage seats a cap AT A CHIP'S PIN, so a cap with no such pin
can never be claimed — yet the old scope evicted all ten from radial zone
packing anyway, and they fell through to the centroid stage and landed near the
board middle. The scope is now the caps that **elect a tether at any distance**
(`seeder_pin_scope`, 60 of ulx3s's 70), and anything the pin stage declines at
run time is put back into its declared zone with a note rather than dropped.

So the two predicates were never the story. One predicate was being asked two
different questions — *is this cap graded against an IC?* and *is there a pin
to seat it at?* — and those have different right answers for exactly this
population.

## Grading the pin, not the package (#705)

`decap_distance` measures the cap's **centroid** to the IC's pad bounding box
inflated 0.5 mm, clamped to 0 inside. The requirement it stands in for is about
the **pin**: a 100 nF sitting 1.7 mm from a QFN and 9 mm from the IOVDD pin it
decouples satisfies that rule and fails the spec. A downstream project using
this toolchain hit exactly that and wrote its own checker.

`decaps.max_pin_distance_mm` grades the pin. It is **opt-in with no default** —
at 3 mm the tracked corpus produces **127** findings — 44 declared, 77 inferred,
6 uncovered — with the worst gaps at 37.47 mm (`interf_u_unrouted_placed`),
34.08 mm (`interf_u_unrouted`) and 30.77 mm (`flat_hierarchy`), so a shipped
default would flood every board on day one.

**The invariant.** Compute a SUPERSET of the caps that could serve a pin, and
violate only when the MINIMUM over that superset exceeds the limit. Every cap
added can only lower the number, so the only way to manufacture a violation is
to have MISSED a cap — which turns "is this finding real?" into one auditable
question. The answer is forced: every two-net `C*` carrying the pin's exact net,
either side, any distance, whoever else it also serves.

That is also why the cap set is **derived rather than taken from
`decap_tethers`**. The tether map is a cap→one-IC assignment truncated at 5 mm
that discards the net that matched, so it cannot answer a per-rail question.
Measured over 271 typed supply pins on 8 boards, a tether-derived set reports a
larger distance on 11 and a spurious "uncovered" on 56 — it manufactures
findings in both currencies. What stays shared is the *cap* predicate, so the
two rules disagree about assignment (deliberately) and agree about what a decap
is.

### Which pads are supply pins, and on what evidence

Three channels, tried **per chip**, and keyed on whether the channel YIELDS
pins rather than on whether its field is populated:

| # | channel | test |
|---|---|---|
| 1 | `pintype` | token-split on `+`; `power_in`/`power_out` present, `no_connect` absent |
| 2 | `pinfunction` | exact or prefix match against `decaps.pin_functions`, else the built-in table |
| 3 | rail net | the net NAME looks like a rail **and** already carries a decoupling cap |

**Per chip**, because on the 85 corpus chips where channels 1 and 2 both yield
they disagree about the pin SET on 31 (36%) — the order materially decides the
answer, so it must be decided somewhere a reader can see. Per *pin* would mix
channels inside one chip and produce a set no channel asserts; per *board* would
grade half of watchy on the wrong evidence (glasgow_revC wins 33 chips on
`pintype` and 1 on the fallback; watchy 5 and 5).

**Yield-keyed**, because `lvds_converter_dualclk` carries `pintype` on 76 of 76
pads and **zero** are `power_in`/`power_out` — IC2/IC3/IC4's supply pins are
typed `passive` and named `VCC_14` / `GND_7` / `VCC_16`. A ladder that asked
"does this board carry `pintype`?" would stop at channel 1 with an empty set,
grade nothing, and record that channel 1 fired: a vacuous pass that no
naturally-written test catches, because the field IS there.

**"Channel N fired" is not "channel N is right."** A board whose supply pins are
all typed `bidirectional` yields nothing at channel 1 and falls through, and no
tool can tell that from a correct board. So every channel's count is recorded
for every chip, fired or not, in `decap_pin_evidence` (and in `JSON_SUMMARY` as
`decap_pins_by_channel`); the chips where a lower channel would have named a
different set are listed as `order_decided`; and channel 3's findings carry
their own rule name at warn.

**Ground is not graded.** On `lvds_converter_dualclk` the three VCC pins sit
2.100 / 2.100 / 1.925 mm from their rail's nearest cap while the three GND pins
on the *same* ICs sit 8.043 / 7.425 / 6.941 mm — what a ground arm measures is a
SOIC-14's opposite corners, not a placement. Half of every `power_in` population
is ground (580 of 1163 corpus pins), so grading it would make the pass rate a
statement about pin numbering.

### Three findings, because they are three claims

| finding | default | granularity |
|---|---|---|
| `decap_pin_distance` | error | per pin, for a pin channel 1 or 2 declared |
| `decap_pin_distance_inferred` | warn | per pin, for a pin inferred from a net NAME |
| `decap_pin_uncovered` | warn | per (IC, rail) — no decoupling cap on that rail at all |

At a 3 mm limit corpus-wide the split is 44 declared, 77 inferred, 6 uncovered.
One name would make `ulx3s` — which carries no `pintype` at all — **fail** on 39
inferences. `decap_pin_uncovered` is per (IC, rail) rather than per pin because
`haasoscope_pro_max_test` is an 8-footprint fixture with 98 typed supply pins
and zero capacitors, which per-pin would be 98 findings saying one thing.

### When the board cannot answer, the rule abstains

`_wants` reads only the intent; nothing read the board. A rule the intent asked
for and the board cannot answer used to land in `rules_run` and print "N rule(s)
ran, no violations" — which makes `--require-rules` *easier* to satisfy, the
vacuous pass that flag exists to catch, arriving through the mechanism that
implements it. A per-rule `_ARM` table now runs after `_wants` and feeds the same
`rules_skipped` dict, with a reason that names the pad census:
`orangecrab_ext_pll` has 28 candidate ICs and zero supply pins on any channel,
and says so.

This is deliberately **not** the `budget_abstained` channel, which means "the
emitter could not derive this key" — a property of the intent, computed with no
board. This is the opposite, and overloading one key with both would make
`budget_abstained_keys` mean two things.

### `same_side` is a manufacturing claim, and it is expensive

Default **false**, and it must be: 43% of corpus decap tethers (152 of 357) put
the cap on the opposite side from its IC — glasgow_revC 61 of 87, orangecrab 44
of 74 — because a decap directly under its pin on the back side, connected by a
via, is standard practice. On the very board #705 offers as its proof that the
rule is safe, `glasgow_revC` U30 is on F.Cu with **all 15** of its tethered
decaps on B.Cu.

`same_side: true` is therefore not an electrical assertion but a **fab** one:
the author is saying the back side is not available — single-sided assembly, a
shielding can or heatsink over it, an enclosure wall or thermal pad. It is
authored, never inferred, and no emitter path can set it.

### What the rule does NOT claim

* It does not claim the pin **needs** a local cap. A `power_in` pad may be an
  LDO's input, a sense pin, or a rail a plane already handles. It grades the pin
  the file declares; it cannot read a datasheet.
* It does not claim each pin has its **own** cap. The metric is a minimum, so
  one cap may satisfy every pin on its rail. Over 353 corpus (chip, rail) pairs
  only 2 have fewer caps than pins; `caps_on_rail` and `pins_on_rail` ride in
  every finding so a reader can see it.
* It does not measure the **return path**, loop inductance, via count or plane
  stitching. It measures a gap in millimetres.
* `decap_pin_distance_inferred` does not claim the pad **is** a supply pin.
* `decap_pin_uncovered` does not claim the rail is undecoupled — only that no
  two-net `C*` on this board carries that net.
* A **clean** result does not mean the board is decoupled. It means every pin
  the rule could identify had a cap on its own net within the limit. When it can
  identify none, it says so in `rules_skipped` and does not report a pass.
* It does not claim its number equals `decap_distance`'s. They answer different
  questions, in different currencies, over different populations.
* `is_ground_net_name` does not match `VSS`, so a board that spells its ground
  `VSS` will have ground pins graded. No tracked board does, so the case is
  structurally untestable here — disclosed rather than fixed with a tenth
  power/ground predicate.

### The emitter derives no pin limit, and `min_reader` does not move

`ceil(max)` over pin gaps is censored by construction — an uncovered pin
contributes no distance, so the max is taken over survivors, which is the exact
censoring failure `--declare-decaps` spends a table on — and a limit derived
from a board and then graded against that board is vacuous. The distribution
goes to `context.decap_census` instead, which has no key set to grow.

`READER_VERSION` stays 1. `_reject_unknown` already refuses an unknown `decaps`
key loudly and automatically, and `min_reader` exists for what refusal *cannot*
see: a widened value set, a changed meaning, a changed default. Adding keys is
none of those.

### `keepouts` stays empty, and says so

A keep-out is a mechanical fact — an enclosure rib, a standoff, a battery, a
display window, an antenna clearance — and none of those can be read off a
board, so the emitter keeps writing `[]`. What it should not do is leave the
reader unable to tell *"none declared"* from *"not considered"*, so
`context.keepouts_note` states which one it is. At grade time the same
distinction is already carried by `rules_skipped` and by `--require-rules`.
