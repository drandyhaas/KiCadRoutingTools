# Floorplan intent, graded

A placement in this toolchain is judged by routing it. Before it is routed,
its own numbers (`crossings`, `hpwl`, the placer's cost) are indifferent
between a sensible layout and a scattered one with the same wirelength — and
a render only moves the judgement from a number to a vibe.
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

The intent is also a GENERATOR input, not only a grader's. These tools read
it:

1. **`place_simple.py --intent` places from it** (the flag is required). It
   reads the `proximity[]` claims and their `max_mm` limits, and the decap
   limits it grades at error (`decaps.max_distance_mm`,
   `max_pin_distance_mm`), all cost terms of its objective; the keep-outs
   and zones (charged in its anneal's overlap term, or held by its final
   legality check; a part left breaking one is named and the run exits 4);
   every declared face (`blocks[].side`, `edge_connectors[].side`,
   `fixed_poses[].side`, applied before it places, by the writer's flip);
   `fixed_poses[]` (seated at the declared pose and held there); and each
   `edge_connectors[]` entry with an `edge` (seated on that edge). The rest
   of the intent is graded only; "Which rules the placer can see" below has
   the table.
2. **`place_pose --intent` refuses a pose that walks a part out of its
   zone.** A pose that leaves a moved part further outside its block's zone
   than it was is refused at exit 4, and `place_pose replicate --intent` (no
   `--from` / `--to`) applies the intent's `replicate[]` rows.
3. **`place_fanout_clearance --intent` holds its decap limits** and the
   rotations it declares for caps.

See [py_placer/placement/README.md](../py_placer/placement/README.md) for all
of them.

**`place_pose`'s zone check is RELATIVE: it prevents a walk-out, it does not
demand an arrival.** A part that is already outside its zone may improve or
hold, never worsen, so a move that brings a part from the pile toward its
zone is never refused for not arriving.

> **Careful with a self-emitted intent.** `--emit-intent` run on the board you
> are repairing records that board's damage as the requirement. That
> mis-grades, and because `place_simple` reads the zones and claims, it also
> **steers the placement toward the damage**.

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
edge_connectors[0]: unknown key(s) max_setback. Known: along_edge_band,
center_on_edge, class, context, edge, mating_face, max_setback_mm, note,
observed_overhang_mm, overhang_capped, overhang_mm, reason, ref, side, source,
suspect, suspect_reason, why
```

| object | keys |
|---|---|
| top level | `schema`, `kind`, `board`, `units`, `min_reader`, `envelope`, `defaults`, `blocks`, `keepouts`, `edge_connectors`, `decaps`, `must_lock`, `legality_budget`, `health`, `severity`, `overlap_waivers`, `assembly`, `proximity`, `keep_away`, `replicate`, `templates`, `dispositions`, `arrays`, `fixed_poses`, `context` |
| `envelope` | `rect`, `tolerance_mm` |
| `defaults` | `zone_tolerance_mm` |
| `blocks[]` | `name`, `group`, `refs`, `zone`, `side`, `exclusive`, `tolerance_mm`, `rotation`, `rotation_candidates`, `why` (or `note` / `reason`), `context`. `rigid` is refused by name: its only reader, the retired quench engine, is gone, and `place_simple` moves each group of the groups file together |
| `keepouts[]` | `name`, `rect`, `circle`, `sides`, `allow`, `why` (or `note` / `reason`), `context` |
| `edge_connectors[]` | `ref`, `edge`, `overhang_mm`, `max_setback_mm`, `center_on_edge`, `along_edge_band`, `side` (`F`/`B`, the connector's face, graded as the advisory `edge_connector_side`; needs `min_reader` 6), `mating_face` (`+x`/`-x`/`+y`/`-y`, see "Which way it opens" below), `class`, `why` (or `note` / `reason`), `context`, and the emitter-written `source`, `suspect`, `suspect_reason`, `overhang_capped`, `observed_overhang_mm` |
| `edge_connectors[].overhang_mm` | `min`, `max` |
| `edge_connectors[].center_on_edge` | `tolerance_mm` (required — see below) |
| `edge_connectors[].along_edge_band` | `from`, `to` |
| `decaps` | `max_distance_mm`, `exempt`, `search_radius_mm`, `max_pin_distance_mm`, `pin_functions`, `same_side`, and the `--decaps-from`-written `within_radius_refs` / `within_radius_mm` (the caps the reference keeps within that radius of their chip, each held to it per cap; the two come together, need `max_distance_mm`, must match `search_radius_mm`, and need `min_reader` 8) |
| `assembly` | `sides` (`"F"`, `"B"` or `"both"`), `body_edge_mm` (the least distance in mm a component body may sit from the board edge, which arms check_assembly's `body_edge_band` gate; needs `min_reader` 9), `why` (or `note` / `reason`), `context` |
| `proximity[]` | `ref`, `near`, `max_mm`, `basis` (`"pad_edge"` or `"body"`), `pads`, `why` (or `note` / `reason`), `context`, and the compiler-written `source` |
| `keep_away[]` | `rule` (ONE rule in route.py's `--keep-away` syntax, `AGGRESSOR:VICTIM:GAP`), `why` (or `note` / `reason`), `context`, and the compiler-written `source` (see "`keep_away[]`" below; needs `min_reader` 10) |
| `replicate[]` | `from` (a sheet instance: its name or path, as `place_pose replicate --from` takes it), `to` (a list of them), `anchor`, `allow_net_mismatch` (`REF.PAD` entries), `only` or `skip` (refs; not both), `search_mm`, `tolerance_mm`, `why` (or `note` / `reason`), `context`, `source` (see "`replicate[]`" below; needs `min_reader` 11) |
| `templates[]` | `anchor` (the chip), `members` (each `ref`, `at` [dx, dy] mm and `rot`, a quarter turn, in the chip's own frame; `pads` and `note` as the fit wrote them), `tolerance_mm`, `kind` (`switching stage` or `rf front end`), `ground_return_mm`, `source`, `why` (or `note` / `reason`), `context`, and the fit-written `fitted` (see "`templates[]`" below; needs `min_reader` 13, and 14 with `kind` or `ground_return_mm`) |
| `arrays[]` | `name`, `members` (an ORDERED list of literal refs, at least two), `serves` (a ref, or `"unknown"`), `order` (`"pin"`, `"declared"` or `"unknown"`), `rotation` (degrees, `"shared"` or `"unknown"`), `pitch_mm` (`"auto"` or mm), `axis` (`"auto"`, `"x"` or `"y"`), `allow_mixed`, `why` (or `note` / `reason`), `context`, and the compiler-written `source` (see "Arrays" below; needs `min_reader` 7) |
| `fixed_poses[]` | `ref`, `x`, `y`, `rot` (degrees or `"unknown"`), `side` (`"F"`, `"B"` or `"unknown"`), `basis` (`"declared"`, `"mechanical"` or `"derived_default"`), `why` (or `note` / `reason`), `context`, `accept_courtyard_overlap` (a list of literal refs) (see "Fixed poses" below; needs `min_reader` 7) |
| `legality_budget` | `overlap_area`, `oob_count`, `oob_amount` (`oob_area` refused — see below) |
| `health` | `bus_corridors`, `classes`, `block_displacement_mm`, `ignore_net_ids`, `max_fanout`, `zoned_blocks`, `affinity_exempt_nets`, `affinity_exempt_net_ids`, `plane_layers` |
| `health.bus_corridors[]` | `name`, `nets`, `width_mm` |
| `severity` | any of the 45 rule names below |
| `overlap_waivers[]` | `pair`, `why` (or `note` / `reason`), `context` |
| `dispositions` | `rules`, `withheld`, `refs`, `contradictions` -- each `{key: why}`, a non-empty written reason (see "The rule roster" below) |
| `must_lock` | a list of reference globs (no nested keys) |

**A row's free text is `why`, on every row type (#1357).** `note` and
`reason` are aliases with the same meaning, accepted wherever `why` is: the
author's reason for the row, which no rule grades. Readers take whichever
spelling the row carries (all of them, joined with `; `, when it carries more
than one), so `fixed_poses[].accept_courtyard_overlap`'s required reason may
be spelled any of the three ways. Each must be a string. The emitter writes
`note` for its own remarks, which is the same slot. A truly unknown key is
still refused. Before #1357 the spelling depended on the row --
`proximity[]` refused `why` and `fixed_poses[]` refused `note` -- so a reason
written the way the neighbouring row spelled it cost the run its intent.

`blocks[].rotation` and `rotation_candidates` are read by
`place_fanout_clearance --intent`, which never turns a cap away from the angle
its block declares and turns one with candidates only within them, and
by the grade's `array_conflict`. `place_simple` does not read them.

`severity` keys are checked too. The settable names are the eighteen rules —
`envelope`, `zone_containment`, `zone_side`, `assembly_side`, `zone_exclusive`, `keepout`,
`edge_connector`, `decap_distance`, `decap_ungraded`, `decap_pin_distance`,
`proximity`, `keep_away`, `must_lock`, `legality`, `pins_to_edge`,
`array_formation`, `replicate`, `template` — plus the ten
findings raised outside the rule loop: `intent_zone_outside_envelope`, `intent_zone_overlap`,
`block_unresolved`, `intent_zone_in_keepout`, `keepout_allow_unresolved`,
`array_unresolved`, `array_conflict`, `fixed_pose_unresolved`,
`fixed_pose_overlap_waived` (**warn** by default),
`mechanical_drift` (raised only when a `mechanical.json` is read, and
settable to `error` only -- `warn` is refused, since the pose is a recorded
fact),
the twelve `plan_check` findings (`plan_zone_exclusive_unsatisfiable`,
`block_glob_literal`, `plan_fixed_outside_zone`, `plan_fixed_pose_conflict`,
`plan_zone_overfull`,
`plan_zone_crowded`, `plan_edge_overfull`, `plan_edge_crowded`,
`plan_board_overfull`, `plan_board_crowded`, `plan_fixed_overlap`,
`plan_fixed_overlap_budget`), `edge_connector_side` (settable to `warn`
only -- `error` is refused, because the rule is advisory by design),
`edge_oversubscribed` (raised by check_floorplan's grade and by
`--plan-only`), plus seven more raised BESIDE a rule's own name —
`decap_pin_distance_inferred` and `decap_pin_uncovered`,
`proximity_unresolved`, `keep_away_unresolved`, `replicate_unresolved`,
`template_unresolved` and `template_ground_return`.
One measurement can support several claims, and
an author must be able to set their severities apart: a pin inferred from a net
name and a pin the pad declares are not the same evidence, and a proximity
claim naming a part the board does not have is a different finding from one
whose parts are simply too far apart — a DNP-variant board can demote the first
without demoting the second.

Every name defaults to **error** except these, which default to **warn**:
`assembly_side`, `decap_ungraded`, `decap_pin_distance_inferred`,
`decap_pin_uncovered`, `keep_away` and `keep_away_unresolved`, `replicate`
and `replicate_unresolved`, `template`, `template_unresolved` and
`template_ground_return`,
`keepout_allow_unresolved`,
`intent_zone_overlap`, `fixed_pose_overlap_waived`, `plan_fixed_overlap` and
the three `plan_*_crowded` findings and `edge_oversubscribed`;
`pins_to_edge` and
`edge_connector_side` are always warn; `mechanical_drift` is warn for a
move the anchor already reports and error for a turn. Writing
`{"keepout_allow_unresolved": "error"}` upgrades one.
`{"decap_distanc": "warn"}` is a
demotion that never happens, so it is refused rather than accepted.

### `context` is the slot for everything that is not a claim

`context` is **deliberately open** — free-form keys, at the top level, on
`assembly`, and on every entry of `blocks[]`, `keepouts[]`,
`edge_connectors[]`, `overlap_waivers[]`, `proximity[]`, `keep_away[]`,
`replicate[]`, `templates[]`, `arrays[]` and `fixed_poses[]`. No rule grades it; it is where
a run records why a claim is what it is:

```jsonc
{ "ref": "J1", "edge": "east",
  "context": { "why": "the mating face is on the east wall of the enclosure",
               "rejected_alternative": "north, blocked by the display window" } }
```

It is open for the same reason everything else is strict: with nowhere to
put reasoning, it drifts into the graded keys, where every consumer ignores
it. `note` is not the place either: it is grepped for the substring `SUSPECT`, so appending prose to
it can change behaviour. A `context` value must still be an object; the keys
inside it are yours.

A few keys inside it are read, all on `edge_connectors[]` entries:
- `context.mount_mode: edge_mount` makes the seat read the drawn body, and
  `edge_mount` or `through_edge` puts the entry under the facing conjunct.
- `context.mount_mode: top_mount` / `bottom_mount` exempts the part from the
  edge-receptacle seat. With a design brief, a plan and brief that disagree
  about a vertical mount drift.
- `context.compiled_from` is read by drift attribution.

At the top level, `context.basis` labels numbers and changes no verdict, and
`context.budget_withheld` is read by the grade: a key the emitter withheld
is abstained on, not passed (see "What `--emit-intent` does and does not
claim").

### Versioning: `schema` is the format, `min_reader` is the vocabulary

`schema` is matched **exactly**, so bumping it invalidates every existing
intent file at once — far too blunt for "this build learned a new field".

So field-level compatibility is a second number. `READER_VERSION` (currently
`11`) is what this build can act on, and an intent sets `min_reader` when a
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

Each `min_reader` level and the keys it covers:

| `min_reader` | keys |
|---|---|
| 2 | `edge_connectors[].center_on_edge`, `edge_connectors[].along_edge_band` |
| 3 | `assembly.sides` |
| 4 | `proximity[]` |
| 5 | `blocks[].rotation`, `blocks[].rotation_candidates` |
| 6 | `edge_connectors[].side` |
| 7 | `arrays[]`, `fixed_poses[]`, `blocks[].rigid` (refused by name since its reader was retired) |
| 8 | `decaps.within_radius_refs`, `decaps.within_radius_mm` |
| 9 | `assembly.body_edge_mm` |
| 10 | `keep_away[]` |
| 11 | `replicate[]` |
| 12 | a `proximity[]` row whose `context.source` is `placement.crystals` |
| 13 | `templates[]` |
| 14 | `templates[].kind`, `templates[].ground_return_mm` |

The emitter and the design-brief compiler stamp the level their output needs.

### Arrays: parts that form one row

```jsonc
"arrays": [{"name": "rn_bus_a", "members": ["RN1", "RN2", "RN3", "RN4"],
            "serves": "U30", "order": "pin", "rotation": "shared",
            "pitch_mm": "auto", "axis": "auto",
            "why": "series terminations on U30's data bus"}]
```

An array says these parts are ONE row: on a common axis, in an order, at one
rotation, at an even pitch. `order: "pin"` is the pin order of the part the
row `serves` -- each member's lowest pad of `serves` on a net that member alone
reaches, preferring a net that lands on one pad (a pull-down's ground lands on
every ground pin), read off pads and nets and never off poses
(`arrays.pin_order`). `"declared"` is `members` in the order written;
`"unknown"` leaves the order unchecked. A row may run either way along its
axis. `rotation` is a number (every member at it), `"shared"` (every member at
one angle, whichever) or `"unknown"`. `"unknown"` is a first-class answer
everywhere and is reported apart from an absent key.

Which parts form a row is a design decision the board file does not record,
so `emit_intent` writes none by default. `check_floorplan BOARD
--suggest-arrays` prints what the pose-blind detector (`arrays.suggest_arrays`)
would suggest, as JSON with its evidence: rows of identical parts each on its
own pin of one part (`pin_run`), caps on one chip's rail pair (`decap_row`), or
one sheet's repeated channel (`sheet_bank`). Its `declined` list holds what it
refused, each with a `why`: a bank that bridges its host and a second big part
(a driver bank between an FPGA and its connectors, laid by hand as a
multi-row block, which one `arrays[]` row cannot express), a host under the
pad-ratio floor, parts excluded from membership. The reader accepts or
declines each suggestion into `arrays[]`; the Python API's
`emit_intent(derive_arrays='auto')` writes them all, and has no CLI flag.
`array_formation` is a POLICY rule, dark when nothing is declared, and it
grades through `arrays.formation` (tolerances in `arrays.DEFAULT_TOLERANCES`,
measured on courtyard centres).

Refused at load, naming the member: a member in two arrays, fewer than two
members, a member listed twice, a member also named by `must_lock`, also an
`edge_connectors` entry or also in `fixed_poses`, and `order: "pin"` with no
known `serves`. Reported against the board by the grade, each naming its
member: `array_unresolved` (a member or `serves` the board does not have) and
`array_conflict` (a member locked in the board file; mixed footprints without `allow_mixed`; a member whose block
declares a `rotation` or `rotation_candidates` the row's numeric or shared
rotation contradicts; members split across zoned blocks, or only partly in
one -- a row is one piece).

No placement tool reads `arrays[]`: `place_simple` neither seats a row whole
nor holds one together, so a declared row is graded only. To keep a row's
parts together while placing, put them in one group of the groups file.

### Fixed poses: a pose to seat, not only to grade

A `fixed_poses[]` entry is an exact pose a part is to be placed at, with
`basis` saying whose fact it is: `declared` (the design brief's
`fixed[].pose`) or `mechanical` (`mechanical.json`, compiled by
`check_floorplan --emit-intent` for each ref the grade anchors, unless another
claim of the same intent already places it -- that skip is recorded in
`context.mechanical.fixed_skipped`), or `derived_default` -- nobody's fact: a
pose a planner chose because nothing declared one. Every entry is GRADED,
whatever its source: the grade compiles an anchor block `fixed:<ref>` from it with the same
`reconcile.anchor_blocks` geometry a `mechanical.json` pose uses, and reports a
part outside it as `zone_containment`. A ref the mechanical file itself
declares is left to the file's own anchor, so one moved part is one finding. A
ref the board does not have is `fixed_pose_unresolved` (**error**); one the
anchor cannot grade (a pad-less part) is the same name at **warn**, with the
reason.

A ref in `fixed_poses` and also in `edge_connectors` or `must_lock` is refused
by name -- two placements for one part. `fixed:` is a reserved block-name prefix,
like `mech:`.

The placement skill declares a plug-in module's pin rows here
(`derived_default`, a whole number of 2.54 mm apart): an edge seat puts each
row flush with its own edge, so their spacing would follow the outline --
13.21 mm on ch32v003_usb -- instead of the socket's 0.1 in grid (12.70 mm).

A `mechanical` entry sitting AT its declared pose, on its declared face, whose
courtyard the census counts off the outline while every pad and its footprint
graphic copper are on the board at zero margin, leaves
`legality_budget`'s `oob_count` and `oob_amount` the way a declared edge
connector inside its overhang band does. `oob_count_exempt` counts
both kinds; a `legality` finding's `measured` carries the reduced amount
beside `oob_amount_raw`. Off its pose, on the other face, or with pad or
graphic copper past the outline, it stays counted. A `declared` entry is never
exempt.

**`place_simple` seats every entry before it places anything** and holds it
there: each part is written at EXACTLY its declared pose -- `rot` / `side`
absent or `"unknown"` keep the part's current angle / face, and a declared
side is applied by the writer's flip -- and is then a fixed part for the
whole anneal, like one the file locks. It is a write, never a search. A part
the file locks on another face or at another pose is refused (exit 2,
nothing written), since a lock is a decision. An unlocked `mechanical.json`
ref is accepted, instead of needing a hand lock, when the plan names it here
at the declared pose.

`place_simple` never moves a declared pose to make it legal. It checks each
against the parts that stay where they are (`seeder._fixed_pose_check`),
names one that is not legal, writes it as declared and exits 4. Whether two
declared poses can both stand is the plan check's question
(`plan_fixed_pose_conflict`,
below, through `seeder.fixed_pose_pair_conflicts`). That check judges
courtyards the way KiCad's DRC judges them: an overlap is illegal,
courtyards that abut (gap 0) are not, so a human's edge-to-edge rows can be
declared. Pad and hole clearance keep their normal rules, ABSOLUTE, and two
parts' pads may not stack on each other whatever their nets. **A board whose own project sets
`courtyards_overlap` to `ignore`** is not refused a courtyard overlap
at all: KiCad checks none, and check_assembly grades it that way.
Holes stay refused hole to hole, and the emitter writes no `overlap_area`
budget. A pair that clashes is named with its measurement ("courtyard
overlaps FID8 by 1.15x1.15mm"). An entry whose declared side is not the
part's face on the board is not judged there.

**A named courtyard waiver.** A human pose can overlap another part's
courtyard and still be the design -- glasgow's U30 overlaps FID8 by
1.15 x 1.15 mm, which kicad-cli's DRC reports as `courtyards_overlap`.
`accept_courtyard_overlap: ["FID8"]` on the entry, with a
`why` (required), waives exactly that: the COURTYARD overlap with the named
refs, as an unordered pair, so FID8 declared as well does not refuse both
halves. Pad clearance, pad shorts and hole clearance -- hole to copper, and
hole to hole at the board's `min_hole_to_hole` -- keep their absolute rules.
The refs must be literal (a pattern is
refused), must not name the entry's own ref, and must be on the board (else
the grade raises `fixed_pose_unresolved`). The measured
overlap is disclosed in the grade as `fixed_pose_overlap_waived` (**warn**; a
waiver that measures no overlap says so too). The plan check's pairwise test reads the waiver (`Intent.courtyard_waiver_pairs`),
and since #1298 `check_assembly --intent` and `render_placement --intent` read
it too (`Intent.fixed_pose_courtyard_waivers`, handed to
`grade_body_overlap(courtyard_waivers=...)`), COURTYARD ONLY: the pair's
courtyard overlap, and a frame pin under the courtyard, are waived and listed
as waived -- label `fixed_pose_accepted`, the section `COURTYARD ACCEPTED BY A
DECLARED POSE`, JSON `courtyard_fixed_pose_waived` -- whatever the parts' class
or KiCad lock. It is NOT an `overlap_waivers[]` pair, whose consumers also
exempt the drawn-body containment gate: one part sitting inside the other's
drawn body still gates, as do pads and holes. A named ref the board lacks is a
WARNING there, and a pair whose courtyards do not overlap a note
(`courtyard_waivers_unresolved` / `_unused`). The pairwise test does honour an
`overlap_waivers[]` pair's courtyard the same way. Nor does it spend the
overlap: the pair's area still counts in `legality.overlap_area` and in
`plan_check`'s `plan_fixed_overlap` (a warn) and its budget, since the board
carries exactly the finding KiCad's DRC reports -- declare the budget to fit
it.

**Declared poses that overlap each other (#1298).** A human layout's poses,
compiled from `mechanical.json` or the brief, can overlap each other -- on
glasgow_revC J5's courtyard overlaps MK2's by 4.07 mm2 and each fiducial sits
in a mounting hole's; on CM5_MINIMA_3 the two module halves overlap by
14.58 mm2 -- and compiled silently they made every placement that honours the
intent NOT BUILDABLE on a pair no move can clear. `check_floorplan
--emit-intent` therefore measures the emitted `fixed_poses[]` against each
other with the plan check's own predicate (`floorplan.declared_pose_overlaps`,
the seeder's `_fixed_pose_check`) and, for every pair whose COURTYARDS overlap
there, writes the other ref into one entry's `accept_courtyard_overlap`, with a
`note` saying who wrote it and why, and prints a WARNING naming each pair. The
waiver is explicit, in the file, where the author can delete a ref to refuse
the overlap instead; check_assembly never infers one. A pair that also
conflicts by pad clearance or by holes (glasgow's fiducials against their
mounting holes' copper ring, CM5's module halves' shared holes) is named in a
second WARNING: no waiver covers pads or holes, so place_simple's seat and the
plan check still refuse it until a pose moves. Both lists are in the emitted intent's
`context.declared_pose_overlaps` (`accepted`, `hard`).

**A shared ORIGIN is a stack only when something physical is shared.**
check_assembly's COINCIDENT ORIGINS conjunct, which catches parts stacked at
one point whose rotated pads interleave so that no pad intersects, groups
parts whose origins agree to 1 um. An origin is a library's datum, so within
such a group a pair counts only when, on a face both occupy, their COPPER pad
boxes overlap (a plated barrel is on both faces), an NPTH hole of one lies on
the other's copper, or the body channel reports pad copper or two drawn
(non-silk) bodies shared there (`placement_state.coincident_touch`). The two
halves of a CM5 IO module footprint share one origin and the module's two
mounting holes with 34 mm between their pads, and rp2350_fpga_eensy_prePlane's
Tag-Connect J2 (B.Cu) sits over U3 (F.Cu) between its alignment holes:
neither is a stack. An `overlap_waivers[]` pair is accepted here as its
courtyard pair is. Both kinds are listed, never silently, in
`coincident_origins_exempt` (`why` `apart` or `waived`). board_score reads the
same JSON.

**Courtyards are graded as drawn, at the project's own severity.** The
courtyard and fab channels keep the part rects as their broad phase and
measure an overlapping pair on the DRAWN outlines: the courtyard united pad by
pad with the copper, a custom pad by its parsed primitives rather than its
box. A pad on no copper layer, such as a paste-only aperture, is not copper:
it is not united, and the placement measures do not count it as a pad either
(`kicad_parser.pad_is_aperture_only`, which keeps NPTH and drilled pads). The
movers -- `QuenchState`'s zero-pad branch, `reconcile.anchor_blocks`, the
`mechanical_drift` default and `stale_dispositions` -- still treat a part whose
only pads are apertures as movable, because such a logo is seated by its
courtyard. The fixed-pose check above calls the same measure. A board whose
own `.kicad_pro` sets `courtyards_overlap` to `ignore` gets every courtyard
pair its intent does not already waive labelled `project_severity_ignore`,
never gating (KiCad runs no courtyard check then). `warning` is graded as
error, since KiCad still reports it and `fix_kicad_drc_settings
--relax-severities` writes exactly that demotion. An `ignore` carrying every
category older route steps of this repo wrote is taken as the tool's, not the
author's, and graded at error; a `kicad_routing_tools.saved_severities` record
is the author's value. None of this touches the fab containment channel,
which is not KiCad's courtyard rule. `check_assembly
--ignore-project-severity` is the OFF arm.

### WHICH WAY it opens: the facing conjunct

The nearest-edge identity reads where a part SITS, and a connector seated
backwards sits nearest its declared edge whenever the back of its shell
grazes it -- a DSUB-9 declared east, seated with its 18 mm shell over the
board and its opening pointing inward, passes the nearest-edge identity. The
facing conjunct reads which way the body leaves its pad row -- a horizontal
connector solders at the back and extends forward to its opening -- through
`floorplan.edge_facing` (`placement/mating.py` has the measurement):

```
[ERROR] edge_connector: J2's body leaves its pad row toward the west
(courtyard+fab, 7.39mm), away from its declared east edge -- its mating face
points west, not out of the east edge
```

- **Who is graded:** an entry that declares a `mating_face` (whatever its
  mount: the seat turns it by that face); else one whose `context.mount_mode`
  is `edge_mount` or `through_edge`; else a declared `class: edge_receptacle`;
  else, with no class declared, a part `part_class` infers as a receptacle. A
  vertical mount, `connector_affinity` and the actuators are exempt -- a connector the board merely keeps near an edge need not
  open out of it (StickHub J5 opens east from a corner of the north edge),
  and a switch's plastic says nothing about which way it is pressed.
- **Strong is an ERROR, weak a WARN.** A face whose offset reaches 2 mm
  fails at the rule's severity; a weaker one at `warn`. KiCad's stock
  library has three receptacles whose pad layout sets a confidently wrong
  face between 0.9 and 1.4 mm, among 60+ correct USB faces in the same range.
  Such a wrong weak face is not caught: a jack whose wrong weak face points
  at the edge opens into the board, and this conjunct reads it as a pass --
  unless the footprint's marker says otherwise (below). Those three stock
  jacks draw no marker, so for them it stands.
- **The footprint's "PCB Edge" marker confirms or vetoes the face.**
  Many footprints draw a "PCB Edge" / "Board edge" text beside a line where
  the board edge goes. `mating.marker_face_local` reads that LINE (parallel
  to the text, nearest it, not on a courtyard or copper layer; the face is
  the side away from the pads) -- not the text's position, which sits at an
  end of the line. A marker that agrees with the body's face makes it STRONG
  (`basis` gains `+marker`, so esp_prog's USB1 and orangecrab's J3, at 1.05
  and 0.63 mm, fail at ERROR when turned away); one that contradicts it makes
  the face abstain, never fail; with no marker, the body's reading stands. It
  never names a face the body does not. Calibrated on KiCad 10's 154 marked
  stock footprints, it agrees with all 73 strong body faces, at every
  quarter turn.
- **It abstains** on a symmetric part, votes that disagree, a board-copper
  plug, a `Vertical` footprint name and a pose near a diagonal; the
  evidence row says so (`facing.disposition: abstained`, with the reason).
- The evidence row carries `facing` (`{edge, facing, local_face, basis,
  offset_mm, strong, policy, reason, disposition}`) and the text line ends
  `; facing west [courtyard+fab+marker, strong] FAIL`. The violation's `measured`
  has no `edge` key: the nearest-edge violation owns that one.
- The facing conjunct leaves the nearest-edge identity, `edge_seat_rect` and
  the default bands as they are. `check_assembly` evaluates no intent clause,
  so a
  reversed connector reaches `board_score` through this ERROR's floorplan
  count, not through the assembly verdict.

### Which way it opens: `mating_face`

The facing conjunct reads a connector's mating face off its drawn body
(`placement/mating.py`), and abstains when the body sits centred on its pads:
a side-entry JST drawn "symmetric about its pads" names no face, so the grade
exempts it and `place_simple` may turn it any way, its opening into the board
included. `mating_face` declares the face for such a part: the side, in the
footprint's own unrotated frame, its mating face opens on.

```jsonc
{ "ref": "J7", "edge": "south", "mating_face": "+y" }
```

It is used only where the body names no face. There it is the face for
`place_simple`'s edge seat (only the turns that point it at the edge) and for
the grade, which then grades the entry's facing whatever its class or mount,
at ERROR.
A `Vertical` header names no face either, and for one `mating_face` is the
side its cable leaves -- a 2xN ribbon off a long side -- so the seat lays it
with its long side along the edge (tigard's J5 `+x`).
The emitter never writes it. No footprint cue predicts it reliably enough to
infer: over the corpus's human boards, the best one (the face is on the side of
the mounting pads) agrees with the designer on 47 of 50 named side-entry
connectors, 94%, and fails on two that use tigard J7's own footprint. So it is
the placement skill's step 3.4 that writes it, from the part's datasheet.

### WHERE ALONG the edge: `center_on_edge` and `along_edge_band`

`edge_connector` grades the overhang band, the nearest-edge identity, the
facing and a setback. All four are satisfied *anywhere along* the edge — so a receptacle
well off the centre of its edge grades exactly as well as a centred one. On a
stick-shaped board where the PCB is the plug body, that is the difference
between a product and something a human rejects on sight. On this repo's
`esp_prog`, USB1 sits **1.75 mm off the centre of its 14.50 mm east edge**
(12.07 %); these two keys are how an intent says whether that is acceptable.

```jsonc
{ "ref": "USB1", "edge": "east", "center_on_edge": { "tolerance_mm": 0.5 } }
{ "ref": "CON2", "edge": "south", "along_edge_band": { "from": 0.30, "to": 0.45 } }
```

`along_edge_band` is the general form — `from` and `to` are fractions of that
edge's own span, so a declaration survives an outline resize. `center_on_edge`
is sugar for a symmetric band around the midpoint. **Both on one entry is
refused at load**, because it is two bands on one claim with no rule for which
wins. (It is refused in `intent_from_dict` rather than in `validate_intent`,
because `validate_intent` has exactly one caller — `grade()` — and every tool
that loads an intent must refuse the file, not only the grade.)

**`tolerance_mm` is required and has no default.** Measured on this repo's own
tracked boards, tigard's three connectors sit at **+16.1 %, −25.4 % and
−28.7 %** off their edge centres, and ulx3s's `J1`/`J2` sit at ±0.0 %. A
threshold this tool picked would fail a good human board three times out of
three. The author supplies the number or there is no claim.

**The offset is measured on every entry that names an `edge`, declared or not**,
and reported as `edge_seating` in `--json` and in the text output — mm and
percent of the span. Only a *declared* `center_on_edge` / `along_edge_band`
produces a violation. An `--emit-intent` document never carries either field.

**The span comes from the outline ring, and abstains rather than guess.** The
edge's span is resolved from the board's own Edge.Cuts ring when one parsed —
the contiguous run lying on that side. Only when there are no rings *and* the
outline is a simple rectangle (where the bounding box **is** the outline
exactly) is the bbox used. Anything else abstains with the reason, in the same
"declared value(s) NOT DERIVABLE — not graded, not passed" channel as a
withheld budget. On `interf_u_unrouted_placed`, for example, the bounding
box's south edge spans 115.57 mm while the board's real south edge spans
81.28 mm, so `BUS1`, centred on the real edge, would read 5.715 mm off against
the box. It abstains per edge, not per board.

### The overhang band is graded on the drawn body

`overhang_mm` is graded on the **drawn body's overhang past the outline, at
zero margin**, whenever that body can be measured: a declared `edge`, a closed
convex Fab outline (SilkS when the part draws nothing on Fab, or when its Fab
drawing is a marking such as a pin-1 dot -- the same
`body.fab_cannot_be_body` decision the facing grade reads) and a rectangular
Edge.Cuts outline. A zero `min` is therefore nonbinding and a positive `min`
is a real conjunct. The graded number is summed over every side the body
crosses, so a corner part counts both sides; for a body that crosses only its
declared edge, it is exactly that edge's overhang.

When no body can be measured, the band is graded on the courtyard (or, with
none, the pad box) instead, also at zero margin (basis
`legacy_occupancy@margin=0`, #1301 below). Unmeasurable cases include a
courtyard-only part, an arc in the Fab drawing, a non-rectangular outline, an
entry with no `edge`, and a
drawn envelope that does not enclose the centroid of its own pads (a pin-1
marker is not a body). On that path nothing here grades pad copper against the
outline, so a part whose courtyard does not enclose its pads can carry copper
past the edge unnamed; `check_drc --check-pad-edge` is the channel that names
it. The basis says which reading was used, and `_Ctx.oob_exempt` and the
edge seat (`edge_seat._Probe`) read the band in the same currency as the rule.

**The band takes no margin (#1301).** The courtyard reading used to be the
occupancy reading at the gate's margin, `max(clearance, board edge
clearance)`: `overhang + margin` outside the board. Every routing step's DRC
writeback rewrites both inputs, so one pose read differently placed and
routed. On daplink, with nothing moved, J2/J3 read 7.0062 placed (margin
0.55) and 6.6562 routed (margin 0.2), and P1 read 1.0500 and 0.7000; three
band errors appeared after routing. An overhang of x now reads x at any
margin, so write a band as the distance the part reaches past the edge, and
never add the board's edge margin to it. A band written in the old currency
reads the margin too high: daplink's hand-written J2/J3 `{7.0, 7.4}` and P1
`{1.05, 1.35}` added 0.55, and now read 6.4562 and 0.5000 placed and routed
alike. The setback conjunct's "no overhang" test reads the courtyard at zero
margin too. It used to let a receptacle 0.52 mm inside its edge count as
overhanging at a 0.55 margin, skipping its seat tolerance, and then fail it at
0.2. The pad-copper edge clearance and the `oob_count` census still read the
board's edge floor. So does the nearest-edge identity: the sides a part
reaches, and sticks out past, are read within the board's edge clearance
(`floorplan.edge_reach_mm`, which the edge seat, reconcile and the emitter
read too), not within the gate's `max(clearance, edge clearance)`. The
writeback lowers the clearance half of that max (sonde_xilinx's class 0.254 is
written back as 0.2), so a part 0.23 mm from a second side used to reach it
placed and not routed. When `min_copper_edge_clearance` is unset, that floor is
the fab edge floor every routing step writes back (0.20 mm, source `fab edge
floor (what routing writes)`), not a fixed 0.55 mm. `--emit-intent` reads the
board the same way: at the floors its grade reads, a part counted AT the edge
when it comes within the grade's receptacle seat tolerance (0.5 mm) of the
outline or past it, its band `max` its zero-margin overhang plus 0.5, and its
edge at the same reach. It used to read all three at a fixed 0.55 edge
clearance, so an intent emitted before this carries bands 0.55 mm wider than
the part's overhang (more on a corner part, whose reading summed both sides).

Every declared connector on the board gets a row in `edge_connector_evidence`
(`--json`), including passing ones. Each row carries:
- `overhang_mm` and `overhang_basis` (`body:<layer>`, e.g. `body:F.Fab`,
  `body:B.Fab`, `body:F.SilkS`, or `legacy_occupancy@margin=0`), plus the
  limit and a disposition;
- the body's signed position, setback and other-edge overhang, or
  `body_unmeasured_reason`;
- `pad_copper_edge`: the part's pad-copper edge clearance from
  `grade_pad_edge_clearance`, at the floor `grade_pad_legality` resolves. This
  is the channel `check_drc --check-pad-edge` grades, and it is reported
  alongside the body because a legal body overhang does not make the copper
  legal. The *clearance* half is evidence only: a pad inside the edge floor
  is reported here and graded by `check_drc`, never by this rule. Its
  `outside_mm` half is not. On the body path, a part whose pad copper leaves
  the **outline** (zero margin, castellated pads excepted) is a violation,
  and `oob_exempt` will not exempt it, because a band licenses the body and
  never copper.

The same keys are added to the entry's `edge_seating` row when one exists.

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

`allow` is a glob tuple, matched with `fnmatch` against every reference. A
pattern that matches nothing exempts nothing:

```jsonc
{ "name": "mount-NW", "rect": [0, 0, 6, 6], "allow": ["MH01"] }
```

on a board whose mounting hole is `MH1` exempts **nothing**. `place_simple`
consults the same resolver (`keepouts_for_ref`, through `QuenchState`), so the
typo does not merely fail to excuse the part -- unless that part is fixed (the
file locks it, or `fixed_poses[]` seats it), `place_simple` **pushes the part
the keep-out was drawn around out of it**. Before placement the keep-out
covers empty space and grades clean, so this warning is the only sign of the
typo.

Reported at **warn**, not error: a spec written before a refdes renumber is a
real case, and it should be loud without being fatal. Per **pattern**, not per
entry, so `allow: ["MH1", "MH01"]` still reports the typo. "Resolved" means the
pattern matches *some* reference — deliberately not "the exemption changes an
outcome", because a pattern naming a real part the keep-out would not have bound
anyway (wrong side) is not a typo.

### `pad stacks` (printed, not a rule)

`check_floorplan` prints `pad stacks: N` -- two parts' pad copper overlapping
on a shared side, ANY net, which check_assembly grades NOT BUILDABLE -- with
up to five pairs, and carries the count as `pad_stack_count` in its
`JSON_SUMMARY` (`--json` has the rows under `pad_stacks`). Each stack is
confirmed on the pads' outlines, by check_assembly's own `legality.pad_intersection_pairs`,
and its area is that channel's pad-rectangle overlap,
and it is never a violation: a floorplan PASS still says nothing about it, and
the line is there so a reader of the grade cannot miss it. The
`pad_intersection_pairs` key beside it is a different number -- a
bounding-box census of pad overlaps.

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

### The rule roster

A skip reason is easy to read past. The roster turns each dark rule into a
question the plan answers. It is printed by the grade, by `--emit-intent`
and by `--plan-only`. `--json` carries it as `rule_roster`, and `JSON_SUMMARY`
carries `rules_dark_undispositioned`.

`check_floorplan --plan-only`'s rule roster names a rule as OWING an answer when
it meets all five of these conditions (nothing refuses a run on it, so read the
roster):

1. The rule is dark, meaning the plan does not arm it.
2. It is not a **policy** rule. `proximity`, `zone_exclusive`,
   `array_formation`, `keep_away`, `replicate` and `template` are policy
   rules: nothing on the board says which parts belong together, which area
   is reserved, which parts form one row, which nets are noisy, which sheet
   instance the others copy, or where a chip's datasheet puts its parts.
3. A **board fact** says the rule applies. The plan's own claims never count
   here.
4. It is **gating**: its table-default severity is ERROR. An intent can promote
   a rule to ERROR, but never demote one out of the refusal.
5. Nothing answers for it in `dispositions.rules`.

Budget keys work the same way. `legality_budget` without `overlap_area` or
`oob_count` owes that key a `dispositions.withheld.<key>`. The debt is read off
the budget the plan actually declares, not off the plan's
`context.budget_withheld` note.

A rule the **design brief** declares is answered in one place only, the
brief-clause check (`check_floorplan`'s DRIFT report). It names a plan that drops
or contradicts a brief clause, by clause id. The roster
reports such a rule as `uncovered` and asks nothing more.

`check_floorplan --json` also writes a `declaration_ledger`. It has one row per
requirement: each rule, each brief clause, and each reconciled channel value.
Every row carries its source, its authority, the rule that grades it, and one
status from this list:

| status | meaning |
|---|---|
| `pending` | no grade yet |
| `graded_pass` / `graded_fail` | the grade has run and passed or failed |
| `graded_warn` | the grade has run and its advisory finding fired (the connector face, `edge_connector_side`) |
| `dark` | owed, nothing answers it |
| `dispositioned` | answered in writing |
| `uncovered` | declared, but not carried by the plan |
| `policy` | a policy rule |
| `advisory` | the rule's findings are WARN only |
| `inapplicable` | the rule does not apply to this board |
| `abstained` | the rule is armed but cannot measure this board |
| `carried` / `unmeasured` / `unknown` | the brief fact is reported, but not physically checked |

## The rules

| rule | fires when | measured with |
|---|---|---|
| `envelope` | the declared envelope is not the board's outline | `board_bounds` |
| `zone_containment` | a member's courtyard leaves its block's zone. A zone side is OPEN for an edge-claim member when that side lies on the outline (within 0.01 mm) and the member's claim names that edge with an `overhang_mm.max`: the edge rule bounds how far past the edge it may go, so the zone does not eject it for going there. Its PADS stay graded there, against the board's own side and at zero whatever the zone's tolerance: pad copper past it is the error (`measured.pads_graded`), since a soldered pad cannot overhang; castellated pads are left out, as the edge rule leaves them out | `GradedPart.rect`. **Priced, not only graded** — `place_simple`'s anneal charges how far a member is outside its block's zone, exclusive or not, through the same `zone_escape` this rule calls with the same open sides (its moves keep every courtyard on the board, so an open side changes nothing there); the feasibility check and the plan check call it with the same open sides (`zone_open_sides`); `place_pose --intent` refuses a pose that leaves a part further outside |
| `zone_side` | a member is on the other face | `legality.footprint_side` |
| `assembly_side` | a part sits on a face the board's declared assembly policy does not populate. **warn** by default: no placement tool reads the policy, and a part changes face only by a face declaration (`place_simple` applies `blocks[].side` and the rest before it places), so an error would be a red mark until someone declares the faces | `legality.assembly_census`, body face — the pad-bearing population, so a zero-pad graphic on the back is not a part (a part whose only pads are paste/mask apertures is zero-pad too) |
| `zone_exclusive` | a non-member intrudes on a reserved zone | `rect_overlap_area`, **courtyard only** — a through-hole stranger's leads may cross a reserved zone, unlike a keep-out's. **Priced and enforced** — `place_simple`'s anneal charges a stranger's courtyard area inside the zone, and its final legality check (`seeder.pose_ok`, through `QuenchState.exclusive_clear`) refuses a stranger's pose inside it and moves the part to the nearest legal pose |
| `keepout` | any part enters a keep-out, unless in `allow` | courtyard **and** through-hole rect. **Priced and enforced, not only graded** — `place_simple`'s anneal charges the area inside a keep-out, and its final legality check (`seeder.pose_ok`) refuses such a pose, both through the same `keepout_hit` this rule calls |
| `edge_connector` | overhang outside `[min,max]`; the wrong edge; a graded receptacle whose mating face points away from its declared edge (**error** on a strong face, **warn** on a weak one); on the body path, pad copper past the OUTLINE (castellated pads excepted); a `connector_affinity` entry seated more than 3 mm from every edge fires at **warn** whatever the configured severity | the band: the drawn body's overhang past the outline, summed over the sides it crosses (`body_outside_mm`, `connector_geometry`), else `BoardOutlineGate.rect_outside_amount`; the seat: `edge_clearance` |
| `decap_distance` | a decoupling cap is too far from its own IC | `groups.decap_populations` (`near`) |
| `decap_ungraded` | a cap in scope lies BEYOND the tether search radius, so `decap_distance` never measured it against the declared limit — a claim about COVERAGE, not compliance. **warn** by default. **ERROR, per cap, for a cap in `decaps.within_radius_refs`**: one the `--decaps-from` reference keeps inside the radius, so leaving it beyond strands a decoupler. An explicit `severity.decap_ungraded` wins either way | `groups.decap_populations` (`beyond`) |
| `decap_pin_distance` | a DECLARED supply pin is further than `max_pin_distance_mm` from the nearest decoupling cap on its own rail, pad edge to pad edge | `floorplan.supply_pins`, `legality.pad_rect` + `rect_gap` |
| `decap_pin_distance_inferred` | the same measurement for a pin inferred from a net NAME rather than from a `pintype` or `pinfunction`. **warn** by default, because the pin set is the inference | same |
| `decap_pin_uncovered` | a declared supply pin's rail carries no decoupling cap at all, anywhere. A design fact, not a placement failure, so **warn** and per (IC, rail) rather than per pin | same |
| `proximity` | two DECLARED parts are further apart than `max_mm`. Per SUBJECT pad when the claim names `pads`, once per pair when it does not | `legality.pad_rect` + `rect_gap`, or `placement.body`'s DRAWN body for `basis: "body"` |
| `proximity_unresolved` | a proximity claim names a ref, or a pad number, this board does not have. A separate NAME so a DNP-variant board can demote it without demoting the distance claim | same |
| `keep_away` | a pad of one DECLARED net group sits closer than GAP to a pad of the other, on two different parts. One finding per part pair, at its closest pads, naming each pad's side. **warn** by default; `severity` may promote it | `legality.pad_rect` + `rect_gap` (plan view), sides resolved by the router's `keep_away.resolve_rule_sides` |
| `keep_away_unresolved` | a keep-away rule has a side that matches no net on this board (**warn** by default) | same |
| `replicate` | a declared copy target is not laid out as a translated copy of its source instance: a part further than `tolerance_mm` (default 0.05) from the best translation, or rotated, or on the other side. One finding per target, naming the worst parts. **warn** by default; `severity` may promote it | `replicate.measure_declared`: parts matched by schematic symbol as `place_pose replicate` matches them, offsets fitted by their median |
| `replicate_unresolved` | a declaration the board cannot resolve -- an unknown or ambiguous sheet, instances not wired alike, a waiver naming no pad -- with `place_pose replicate`'s own reason (**warn** by default) | same |
| `must_lock` | a declared-critical part is not locked in the file | `parser.extract_locked_refs` |
| `legality` | overlap or off-board parts exceed a budget. An overlap finding's `measured` carries `overlap_area` (the outline reading the budget grades), `overlap_area_rect`, the worst 50 `pairs` (`[a, b, rect, exact]`) with `pairs_total`, and `excluded` -- the containers, pad-less logos and silk-only parts the budget does not see (a silk body never gates); `legality.overlap_area_excluded` counts them on every grade | overlap: `legality.courtyard_overlap_pairs` (check_assembly's drawn outlines); off-board: `QuenchState.legality_metrics` |
| `pins_to_edge` | a part's pad row faces the outline (within `edge_facing.EDGE_MM`, 2 mm) with no partner beyond it, so its nets can leave only along the edge strip or under the part. Always **warn**, whatever `severity` says; armed only by declared `edge_connectors`, whose edge claims it excludes | `placement.edge_facing` |
| `edge_connector_side` | a connector's declared `side` is not the face it is on (`place_simple` applies that face before placing, so it fires on a board placed some other way). Always **warn** (`error` is refused at load) | the graded part's face |
| `edge_oversubscribed` | an edge (on one face) whose declared parts cannot all seat on it together: the record `place_simple` makes and reports once a run (`oversubscribed`), measured on the board it seats on -- declared faces and poses applied -- so check_floorplan says it of the pile BEFORE placing, and of every placed seed alike. Raised by check_floorplan's grade (`grade(edge_seats=True)`) and by `--plan-only`. **warn** by default; `severity` may promote it | `edge_seat.oversubscribed` (`seats`, its rows packed by `_crowding`, worded by `crowding_text`) |
| `mechanical_drift` | a `mechanical.json` ref moved or turned away from its recorded pose: **warn** for a move its anchor already reports, **error** for a turn and for a pad-less ref; the map may promote the warn, never demote the error | the recorded pose vs the board |
| `block_unresolved` | a block matched no footprint | — |
| `intent_zone_in_keepout` | a declared zone is contradicted by a keep-out that binds its members: covered entirely (reported per block), or left with no pose for a member at any rotation (per member) | `zone_covered_by_keepout`, then `zone_pose_feasibility` |
| `keepout_allow_unresolved` | a keep-out's `allow` pattern matches no footprint (**warn** by default) | `allow_pattern_matches`, the resolver's own matcher |
| `intent_zone_outside_envelope` | the intent contradicts itself (no board needed) | — |
| `intent_zone_overlap` | two zones overlap on one face -- a **warn**: a member of either may sit in the shared area | — |
| `plan_zone_exclusive_unsatisfiable` | a member has no pose in its own zone that stays out of a stranger's EXCLUSIVE zone -- the one overlap no placement can satisfy | `zone_pose_feasibility` with the exclusive zone as a keep-out |
| `array_formation` | a declared array is not ONE formed row: off a common axis, out of the expected order (either direction), at more than one rotation or not the declared one, or at uneven (or not the declared) pitch. A policy rule, dark when nothing is declared | `arrays.formation` over the courtyard centres (`GradedPart.rect`) and board rotations; the order from `arrays.pin_order` |
| `array_unresolved` | an array names a member, or a `serves` part, this board does not have | — |
| `array_conflict` | an array cannot be formed as declared: a member locked in the file, mixed footprints without `allow_mixed`, a block rotation contradicting the row's, members split across zoned blocks | `rotations_for_ref`, `resolve_blocks` |
| `fixed_pose_unresolved` | a `fixed_poses[]` ref is not on this board (**error**), or the anchor cannot grade it (**warn**, with the reason); a part off its fixed pose is reported as `zone_containment` on the anchor block `fixed:<ref>`. Also an `accept_courtyard_overlap` ref the board does not have (**error**) | `reconcile.anchor_blocks` |
| `fixed_pose_overlap_waived` | a `fixed_poses[].accept_courtyard_overlap` pair, with its measured courtyard overlap on this board, or with none (a stale waiver) (**warn**) | `legality.pair_overlap_area` |

Where a rule and a placement tool measure the same quantity (zones,
keep-outs, the legality numbers), they call the same function, so a pose the
tool accepts is one the grade accepts. A grader with its own idea of what
"legal" means would grade the reimplementation rather than the board.

### Which rules the placer can see

A constraint the placer cannot see can only ever produce a failing grade. This
is the whole table for `place_simple`, and the "no" column is the part worth
reading.

| rule | graded | `place_simple` | if not, why not |
|---|---|---|---|
| `zone_containment` | yes | **priced**: the anneal's overlap term charges a member for how far it is outside its block's zone, past the zone's tolerance; a member still outside at the end is named, and the run exits 4 | — |
| `zone_exclusive` | yes | **priced and enforced**: a stranger's courtyard area inside the zone is in the anneal's overlap term, and the exact legality check (`simple.legal`, which is `seeder.pose_ok`) refuses such a pose at the end | — |
| `keepout` | yes | **priced and enforced**: the area inside a keep-out is in the anneal's overlap term, and the legality check refuses such a pose | — |
| `zone_side` | yes | **applied before placing**: a member on the other face is flipped at its pose (the writer's flip) | — |
| `fixed_poses[]` (anchor `fixed:<ref>`) | yes, as `zone_containment` on the anchor | **seated and held**: written at the declared pose before placing, and fixed for the whole anneal | — |
| `proximity` | yes | **priced**: each claimed pad's distance to its partner's nearest pad on its net (pad centres) is a term of the objective, and each mm a claimed pad is past `max_mm`, pad edge to pad edge as the grade measures it, costs `--claim-limit-weight` (default 10). A claimed part's `jump` move aims at its partner's pads, a move of the partner carries the parts holding their claim on it, and a claimed part joins its partner's group. The crystal claims -- each passive crystal at its IC's oscillator pins (through a series resistor where a leg has one) and its load caps at the crystal (`placement/crystals.py`, `max_mm` 8.1 and 2.2, derived defaults) -- are written into `proximity[]` by `--emit-intent` with `context.source` `placement.crystals`; `place_simple` derives the same rows itself for an intent that lacks them, and a pair the intent already claims keeps the intent's row | the term reads pads on shared nets whatever the `basis`, so a claim whose parts share no net is not priced. A crystal row is graded at **warn** whatever `severity.proximity` says, and `place_fanout_clearance --intent` does not hold it: its limit is the human corpus's 75th percentile, which one human board in four exceeds. Copy the row without that `source` to make it an error-graded claim of your own (reader 12) |
| `edge_connector` | yes | **seated**: an unlocked edge part (connector, receptacle, switch or test pad, by the entry's declared `class`, else the part's inferred one, as the grade reads it) declared on an edge is held there, its mating face (`mating_face` where the body names none) pointing out and flush with the low end of its `overhang_mm` band, and only slides along it, within its `along_edge_band` / `center_on_edge` when it declares one; its `side` is applied before placing | a part that is not an edge part by class (a cap or a jumper by the edge) is emitted with its observed overhang as a band and no edge, so it is placed like any other part. Lock a connector at its edge pose in the file, or declare that pose in `fixed_poses[]`, when the position along the edge is fixed |
| `must_lock` | yes | — | it is a claim about the FILE; no pose satisfies or violates it. `place_simple` holds the parts the file locks |
| `assembly_side` | yes | — | no move carries a side; a part changes face only by a declaration (the `zone_side` row). **warn** by default so it cannot become a permanent red mark |
| `envelope` | yes | — | a claim about the intent FILE against the board, not about any pose |
| `decap_distance`, `decap_pin_distance` | yes | **priced** when graded at error: each mm past `max_distance_mm` or `max_pin_distance_mm`, measured as the grade measures it (the tether gate's terms, `floorplan.tether_gate_spec`), costs `--decap-limit-weight` (default 10), and a move of a chip carries the caps holding their limits at it. Beside it, the decap term pulls each cap toward a supply pin of its rail on an IC of its own group (any IC's when the group has none), the rail's caps matched one to a pin site, whatever the limit | a limit demoted to warn is not priced. `place_fanout_clearance --intent` (and the GUI fanout tab, which shares its engine) holds both limits on the caps it moves, measured by CALLING the grader's own distances (`groups.elect_live`, `floorplan.nearest_rail_cap`) |
| `decap_ungraded`, `decap_pin_uncovered`, `decap_pin_distance_inferred` | yes | — | `decap_ungraded` and `decap_pin_uncovered` are claims about what the GRADE covers rather than about any pose, and `decap_pin_distance_inferred` is a WARN about a pin inferred from a net name. A cap a `--decaps-from` reference holds leaving the radius is counted by `place_fanout_clearance --intent` as a decap claim made worse when it picks the arm to keep |
| `legality` | yes | — | a whole-board aggregate against a BUDGET. The anneal keeps only an overlap-free state as its best, and the legality check holds every part clear of its neighbours |
| `replicate` | yes | — | `place_pose replicate --intent` applies the declaration; an anneal after it may move a copied part, and the grade then reports it |
| `template` | yes | **seated, carried and priced**: each member starts at its template pose round its chip, moves rigidly with every move of the chip, and its mean pad distance from that pose costs `--template-weight` (default 100) per mm; a member's `jump` goes back to its pose, and a member joins the chip's group and is not priced as a decap | the final legality step can still nudge a member to its nearest legal pose; the grade then reports the drift |
| `keep_away` | yes | — | grade only; the routers price it once there are tracks |
| `pins_to_edge` | yes | — | always-WARN advice for a reviewer |
| `array_formation` | yes | — | graded only. A group in the groups file keeps its parts together, not in a row |

The two zone rows reach `place_simple` by **different channels**.
`zone_containment` is a *must-be-inside* claim, so it is a distance to lower:
its weight in the overlap term rises as the run cools, so a part may be
outside its zone early and is pulled in later. `zone_exclusive` is a
*must-be-outside* claim whose target is clean by definition, so it is checked
**absolutely**, through `QuenchState.exclusive_clear` over the
`zone_exclusive` slice (`exclusive_spec`) of the same `build_zone_spec`
construction the containment terms come from — one resolution and one
measurement, so neither can invent its own idea of who is a member.

The slice drops a zone whose members did **not resolve**. "Stranger" means
"not a member", so an unresolved block makes every part one, and a check that
moves a part must not act on a claim it cannot attribute. `rule_zone_exclusive`
carries the same filter, so a pose the check accepts is a pose the grade
accepts.

### `assembly.sides`: which faces the fab will populate

A board-level fab claim, not a per-block one. `blocks[].side` says where one
subsystem belongs; this says how many reflow passes the board gets.

```jsonc
"assembly": { "sides": "F" }        // "F" | "B" | "both"
```

**The face is named, and `single` is refused.** Single-sided does not mean
front-sided: `ulx3s` is back-dominant, 163 of its 226 pad-bearing parts, so a
reader that took `single` to mean F.Cu would be wrong about most of a shipping
board. The refusal says so rather than just listing the enum.

Two things read it:

* **`rule_assembly_side`** flags a part on a face the policy does not populate.
  It **warns** by default, deliberately: no placement tool reads the policy —
  the writer's flip moves a part between faces only where a face declaration
  names it, and no move in the anneal carries a side — so an error would be a red mark until someone declares the faces. An author who
  wants the hard gate writes `"severity": {"assembly_side": "error"}` and
  means it.
* **`options.grow_board`** charges the parts against one face instead of the
  busier one. That is the arithmetic the key exists for: without it, a board's
  back-side area is credited to every board with nothing charged against it.
  The plan check reads it (`--plan-only`): a declared `F` or `B` on which the
  parts do not fit by area is a `plan_board_crowded` WARN.

It grades the **body face**, never `sides_occupied`. A through-hole part on the
front needs wave, selective or hand soldering — not a second reflow pass — and
`sides_occupied` would call `splitflap_driver` two-sided on 22 drilled parts
when its back face is empty.

`--emit-intent` writes the **observed** policy, so an existing board emits its
own faces and grades clean by construction. `single` appears only where a human
typed it, which is the case where the violations are the point. The count is
the pad-bearing population: `glasgow_revC` declared `F` reports **92**, not the
94 blocks on its back, because two of those blocks carry no pads. The census in
`check_assembly.py` prints the same numbers and declares its basis.

### `assembly.body_edge_mm`: how close a body may sit to the edge

This is an assembler's rule, and the board file cannot say it. JLCPCB's assembly
terms ask for at least 2.5 mm between a component body and the board edge.
JLC also adds edge rails when the file has none, and after that the board edge
is no longer the conveyor edge.

```jsonc
"assembly": { "body_edge_mm": 2.5 }   // mm, > 0; `sides` may sit beside it
```

`check_assembly` measures every pad-bearing part's body in its
`body_edge_band` census. The body is the drawn one (.Fab, else silk united with
the pads), else the courtyard, else the pad box, and the rung is named. It is
measured against the real outline at margin 0, cutouts included. A body that
reaches past the outline reads 0. Some parts belong at the edge by what they
are: edge-class parts, markers (mounting holes, fiducials, test points) and
parts whose `edge_connectors[]` entry names an edge. They are listed apart and
never counted. A connector with no edge claim stays in, because connectors are
most of the hits on human boards.

* **Undeclared**, the band is 2.5 mm, labelled `derived_default` with the JLC
  citation. The census is **report-only**. Every human board measured breaks
  it: splitflap 10 of 58 parts, esp_prog 7 of 15, glasgow 4 of 232, watchy 10
  of 70 and ulx3s 9 of 210.
* **Declared**, every part in the band is a `not_buildable` conjunct
  (`body_edge_band_gating`). render_placement's `a_body_edge_band` fails
  `--gate` the same way.
* With `--baseline` (render: `--before`), `new_in_band` counts the parts in the
  band that moved.

No floorplan rule grades this key. `check_floorplan` carries it and names
check_assembly as its grader. The design brief declares it as
`assembly.body_edge_mm` too, and that compiles here. The key is spelled
`body_edge_mm` rather than "...band", so it cannot be confused with
`edge_connectors[].along_edge_band`, which is a position along an edge.

### `keep_away[]`: two net groups apart, before any track exists

```jsonc
"keep_away": [{ "rule": "Net-(D5-A):/32K_P,/32K_N:1" }]
```

A rule is route.py's own `--keep-away` rule, `AGGRESSOR:VICTIM:GAP`, with net
patterns and `class=` terms, parsed by the router's `parse_keep_away_rules`
and resolved by its own resolver. One rule per row. The routing keep-away
measures tracks, so on a placed, unrouted board it checks nothing;
this rule measures the pads the placement put down. Per rule, it measures
every aggressor pad against every victim pad, pad edge to pad edge
(`legality.pad_rect` + `rect_gap`, the currency of `proximity`'s `pad_edge`
basis). It skips two kinds of pair that placement cannot change: two pads of
one net, and two pads of one part, whose footprint fixes their gap. A part
pair closer than GAP is one **warn** finding at its closest pads, naming each
pad's side:

```
[warn ] keep_away: Y1 pad 1 (F.Cu) on /32K_P is 0.91mm from L2 pad 1 (F.Cu) on Net-(D5-A), inside the 1.00mm keep-away (keep_away[0] 'Net-(D5-A):/32K_P,/32K_N:1')
```

`severity: {"keep_away": "error"}` promotes it. Every rule also prints its
closest pair when it passes (`keep-away <rule>: closest 15.34mm, ...`), and
`--json` carries those readings as `keep_away[]`. A side that matches no net
is `keep_away_unresolved` (**warn**). A rule with nothing between two parts to
measure abstains.

What it cannot see is printed beside the readings. It does not see tracks,
which route.py prices and reports once routed. It does not see coupling
through planes or between faces, because the distance is plan view; each
pad's side is named, and a pair on opposite faces says so (ulx3s: `Y1 pad 3
(F.Cu) on CLK_25MHz is 1.05mm from U4 pad 3 (B.Cu) on /power/L2`). It does not
know which nets are noisy or sensitive; the rule declares that.

It is **not** a `min_mm` on `proximity[]`, which the design brief refuses by
name. That would be a copper clearance between two parts; this is a
functional distance between two net groups. Declare it in the design brief
(`keep_away`), not here by hand. The brief compiles it into this key, and
route.py, route_diff.py and the GUI read the same brief key and price it. A
row written straight into an intent is graded, but no router reads it, and
check_floorplan says so. It asks the router's own reader
(`keep_away.unpriced_rules`) which rules the brief beside the board
declares, and warns about every intent row that is not among them:

```
WARNING keep_away: 1 keep_away rule(s) in this intent are graded on the placement but will NOT be priced by route.py, route_diff.py or the GUI: the routers read keep-away rules from the design brief beside the board, never from an intent, and no design brief sits beside the board (w.design-brief.json):
  - keep_away[0] 'Net-(D5-A):/32K_P,/32K_N:1'
  fix: declare it in the `keep_away` list of .../w.design-brief.json, the brief the route steps read beside this board, then re-run check_floorplan.py --emit-intent so the intent carries it from the brief
```

`--json` carries the same as `keep_away_unpriced` (the rows, the brief the
router reads and what it declares), and JSON_SUMMARY counts them as
`keep_away_unpriced`. A rule compiled from a `--brief` that is not the file
beside the board is caught the same way, at `--emit-intent` too. No
placement tool reads the key either (see "Which rules the placer can see").

### `replicate[]`: sibling sheet instances laid out alike

```jsonc
"replicate": [{ "from": "CH1", "to": ["CH2", "CH3", "CH4"], "anchor": "J1",
                "why": "four identical channel strips at the front-panel pitch" }]
```

A board drawn from one hierarchical sheet instantiated several times (a
mixer's channel strips, one I/O buffer per bank) already says which parts
correspond: every footprint's `(path ...)` ends in the same symbol uuid in
every instance. That one instance is the TEMPLATE the others copy is a design
decision, so this key is never emitted; the AI declares the structure and the
tools do the geometry, as `place_pose replicate` does by hand.

A row names a SOURCE instance and its TARGETS, by sheet name or path as
`place_pose replicate --from/--to` takes them, and carries that verb's own
settings: `anchor` (the part that keeps its place in each target), one
`allow_net_mismatch` entry per pad wired differently BY DESIGN (an address
strap), `only` or `skip` (a partial copy), and `search_mm` (fit a copy that
does not fit as asked). `tolerance_mm` is the grade's. The loader refuses a
target named twice (two copies onto one instance would each undo the other),
`from` among its own `to`, `only` with `skip`, and a waiver that is not
`REF.PAD`. Whether the sheets exist and are wired alike is the board's
question, answered when the row is used.

Two things read it:

- **`place_pose BOARD OUT replicate --intent PATH`** (no `--from`/`--to`)
  applies every row, in file order, as one call, with the verb's usual
  refusals: the copy is written only if it grades no worse than the board
  and moves no locked part.
- **The grade** (`replicate`, **warn** by default) measures, per target, how
  far the layout is from the source's: the parts are matched as the verb
  matches them, their offsets are fitted by ONE translation (the median, so
  a copy laid around any anchor reads 0 and one stray part reads as itself),
  and each part's distance from it is its residual. A target with a part
  past `tolerance_mm`, rotated or on the other side is one finding naming the
  worst parts; every target prints a line, passing ones too, and `--json`
  carries the readings as `replicate[]`. A row the board cannot resolve is
  `replicate_unresolved`, with the verb's own reason.

`place_simple` does not read it: an anneal run after the copy may move a
copied part, and the grade then reports it.

### `templates[]`: a chip's datasheet layout

```jsonc
"templates": [{ "anchor": "U2",
                "source": "TI LMR36006 SNVSB48C, 11.1 and Figure 57",
                "members": [{ "ref": "L1", "at": [-5.808, 0.0], "rot": 180.0,
                              "pads": { "L1": ["1"], "U2": ["12"] },
                              "note": "inductor on SW 12" }, ...],
                "tolerance_mm": 0.3 }]
```

A switching regulator's datasheet comes with a recommended layout: where
each support part sits round the chip and which way it faces.
A template records that arrangement for one chip on one board, as each
member's pose (`at`, `rot`) in the chip's own frame: the chip at the origin,
unturned. Where a datasheet puts the parts is read from the datasheet, not
the board, so this key is never emitted. `py_tools/fit_template.py` writes
it from a sketch, the members in the drawing's order with the pins each
serves, fitting each member to this board's footprints as close to those
pins as the courtyards allow (`placement.templates.fit`). The skill's §3.5
says how an agent gets from the datasheet to the sketch.

The loader refuses a member named twice (in one template or across two), a
member that is a template's anchor, an `at` that is not two numbers and a
`rot` that is not a quarter turn. Whether the refs exist is the board's
question (`template_unresolved`).

Two things read it:

- **`place_simple`** seats each member at its template pose when every
  start is laid out, carries the members rigidly with every move of the
  chip, and prices each member's drift (see "Which rules the placer can
  see").
- **The grade** (`template`, **warn** by default) measures each member's
  worst pad's distance from where the template puts it for the chip's pose
  on the board. A template with a member past `tolerance_mm` (default 0.3)
  or on the other face is one finding naming the worst members; every
  template prints a line, and `--json` carries the readings as
  `templates[]`.
- **The ground return** (`template_ground_return`, **warn** by default), on a
  row whose `kind` is `switching stage`. A member can sit exactly at its
  template pose and the template still be wrong: an output cap fitted with
  only its VOUT pad named is seated past the inductor, its ground 10 mm from
  the chip's on fusb302_pd_dev, where the datasheet's drawing joins it to the
  input caps' ground beside the PGND pins. So each member capacitor with a
  pad on one of the chip's grounds has its ground return measured: the least
  sum of pad-edge gaps from that pad to the chip's nearest pad on that ground,
  or a member transistor's (a controller's low-side FET, where its switching
  current returns), through other members' pads on it -- the copper a surface
  pour lays to join them (`templates.ground_returns`). A template with a cap
  past `ground_return_mm` (default 3) is one finding naming the worst, and
  the readings ride in its `templates[]` entry as `ground_return`. Templates
  fitted with every cap's ground beside the chip's or a like cap's read
  0.2-1.9 mm; output caps fitted with their VOUT partner alone read 8.2-10.5
  mm. `fit_template.py` prints the same measure on the fitted poses, with a
  NOTE for a cap past the limit, so the sketch can be fixed before anything
  is placed. A row with no `kind`, or an `rf front end` (whose shunt parts
  ground at their own vias), is not measured.

### A through-hole part is in a keep-out from either side

Its leads pass through. `keepout` tests the courtyard **and** the drilled-pad
rect against every face the part occupies, so a mounting-hole keep-out cannot be
walked through from the back.

### Three instruments charge a through-hole part per face, and they differ

A drilled part's body is on one face and its leads come out on the other, so
every per-face instrument has to decide what the far face costs. They do not all
answer the same way, and that is deliberate — they are not all asking the same
question.

| instrument | its question | near face | far face |
|---|---|---|---|
| `legality.rect_on` | may these two parts overlap? | courtyard | **drilled-pad rect** |
| `simple_anneal` overlap, `simple.Model` halo | how far apart must the anneal push them? | courtyard | **drilled-pad rect** |
| `options.grow_board`, undeclared | does the area fit on the busier face? | courtyard | **drilled-pad rect** |
| `options.grow_board`, declared `F`/`B` | does every part fit on the one populated face? | courtyard | **nothing** — see below |
| `floorplan.rule_assembly_side` | how many reflow passes? | body face only | **nothing** |

`grow_board` charges the far face **only when no face is declared**. Under a
declared `F`/`B` the number is `sum` over the populated dict — each part exactly
once, on the one face the fab builds — and a part's leads come out on the face
nobody populates, where they compete with nothing. Charging them there would
count the same area twice. The `NOT MODELLED` line on such a run says so explicitly.

`rule_assembly_side` charges nothing because a through-hole part on the front
demands wave or hand soldering, not a second reflow pass.

**`floorplan.rule_keepout` is not a fourth model.** It uses `part.sides` only to decide *which* keep-outs bind, then hands
`keepout_hit` both rects and takes a `max` over them, consulting no face at all.

`splitflap_driver` is the board where the answers are visibly different: 65
parts, every one on `F.Cu`, 24 of them drilled. Its populated back area is
`0.00 mm²`, its obstructed back area is `558.34 mm²`, `sides_occupied` calls it
two-sided, and `rule_assembly_side` calls it `F`.

### A zone a keep-out leaves no room in is refused

An intent is refused where its keep-outs leave a member of a zoned block no
room: the question is *does the zone minus the binding keep-outs still hold
this part at some rotation*. A zone `[10,10,20,20]` at `tolerance_mm: 0`
under a keep-out `[10,10,19.9,20]` is refused, and so are two keep-outs that
cover half of it each.

It has to be refused where it is authored because no placement can satisfy
it: in `place_simple`'s anneal such a member pays the keep-out term inside its
zone and the containment term outside it, and no pose clears both.

Reported **per member**, since the answer differs per member: a 2×2 part and an
8×8 part in the same zone under the same keep-out disagree. Total coverage is
reported once per block.

What the check does **not** claim: it models the zone and the intent's own
keep-outs, and deliberately not the board outline, clearance or neighbours.
A feasible verdict says there is room, never that the part can be seated. Two
consequences are disclosed rather than hidden:

- **circle keep-outs never refuse.** Nothing in this tree computes disc/rect
  free area (`keepout_hit` returns a marker for a disc and says so), so a
  refusal resting on one would be unsound. When the rects alone still refuse,
  the refusal stands; only when dropping the discs would have found a pose is
  the answer undecided. Total disc coverage is caught exactly.
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

## The plan, checked before any pose (`--plan-only`)

`floorplan.plan_check` checks the plan against itself and the board without a
pose for any movable part, so a plan's errors show before anything is placed:

```bash
python3 py_tools/check_floorplan.py board.kicad_pcb --intent plan.json --plan-only
```

It prints each finding and the rule roster; `JSON_SUMMARY` counts the
declaration ledger in its before-placement view (armed rows are `pending`), and
`--json` writes its rows. It needs no placed board and exits 4 on an ERROR.

**Every ERROR is sound**: no arrangement of the movable parts satisfies the
plan. The WARN is the same quantity with a margin.

| finding | ERROR when | otherwise |
|---|---|---|
| `plan_zone_exclusive_unsatisfiable` / `intent_zone_overlap` | a member has no pose in its own zone that stays out of a stranger's exclusive zone | any overlap is a WARN |
| `intent_zone_in_keepout` | as `grade` raises it | |
| `block_glob_literal` | a real reference used as a glob over-matches a block into a second, disjoint zone | a stray over-match with no such conflict is a WARN; one the same list also names (an intended over-match) is no finding |
| `plan_fixed_outside_zone` | a FILE-locked member is already outside its zone; or (WARN) a `fixed_poses[]` entry's declared pose is outside its own block's zone | |
| `plan_zone_overfull` / `_crowded` | per face, the members' areas exceed the zone by more than the declared `legality_budget.overlap_area`. Fitting area A into zone Z forces at least A - Z of courtyard overlap; the ERROR sums the members' graded OUTLINES and far-side clusters (the geometry the budget is measured on), the WARN their rects. `measured.zones[]` carries both (`members_outline_area_mm2`, `members_area_mm2`) | the WARN applies without a budget, on a project that waives courtyard overlap, or past a crowding margin |
| `plan_edge_overfull` / `_crowded` | one edge-claimed part's pad extent, at its best 90-degree turn, is longer than its edge | summed extents are a WARN, because flanges overhang corners |
| `plan_board_overfull` / `_crowded` | the parts' graded outlines, per face (the face the intent declares for a part, below), force more overlap against `options.grow_board`'s usable area (clearance 0) than the budget allows, with `oob_count` declared 0; `measured.outline_area_per_face_mm2`, and the rect reading as `forced_overlap_rect_mm2` | |
| `plan_fixed_overlap_budget` / `plan_fixed_overlap` | the FILE-locked pairs alone exceed the overlap budget | each such pair is a WARN |
| `plan_fixed_pose_conflict` | two `fixed_poses[]` entries whose declared poses conflict under the fixed-pose courtyard, pad and hole check ("Fixed poses" above; `seeder.fixed_pose_pair_conflicts`; an `accept_courtyard_overlap` waiver is honoured; an entry whose declared side differs from the board is not judged) | |

**The face a part is priced on** by the zone and board rows is the one the
intent DECLARES for it -- `blocks[].side`, `edge_connectors[].side`,
`fixed_poses[].side`, when they agree (`floorplan.declared_side_claims`) --
else the face it is on in the board. A pile has every part on F.Cu; the plan
is priced as it will be placed once those faces are applied, which is what
`place_simple` does first: it mirrors every part whose declared face differs
(these three, and the groups file's `side`), at its pose (the writer's flip). It
refuses, writing nothing, a ref two declarations put on different faces and a
KiCad-locked part the intent puts on the other face (exit 2).

A plan ERROR is only as strong as the rules it stands for. With
`zone_containment`, `zone_exclusive`, `legality` or `edge_connector` demoted to
warn, the matching plan finding is a WARN. An explicit severity for the finding
itself still wins. `block_glob_literal` is about the plan's own spelling, not a
graded rule, and keeps its own severity.
A plan that declares no `legality_budget` gets WARNs only from the area rows.
Nothing bounds its overlap, so no area bound is sound.

**Callers.**
- `check_floorplan --plan-only` reports every plan ERROR.
- `place_pose --intent` refuses, at exit 4, a single pose that leaves a moved
  part further outside its block's zone than it was, when that
  `zone_containment` finding is an ERROR.

## `mechanical.json`: recorded facts, reconciled and anchored

A `mechanical.json` beside the board records the parts whose pose the
enclosure fixes, with the board's floors.

`check_floorplan` and `board_brief` (and so `/plan-pcb-placement`, through
`--emit-intent`) discover it beside the board. `--mechanical PATH` names
another file, and `--no-mechanical` is the OFF arm. The file is read in the
declaration form `{interfaces, fixed, project}`; anything else exits 2.

**Every value keeps its authority**, and the authority says who stands behind
it, never what the value calls itself:

| authority | the value |
|---|---|
| `declared` | matches the compiled design brief |
| `recorded_fact` | existed before the run: the outline, an existing `.kicad_pro`, and a `mechanical.json` |
| `hypothesis` | anything the run wrote: zone plans, `--intent` files, locks added during the run |
| `inferred` | re-derived from the board |
| `assumption` | a staging default (a floor fixed at staging) or a code default dimension |

The winner is declared, then recorded_fact, then hypothesis, then inferred.
Every disagreement is reported with both values and their sources: in the
emitted intent's `context.reconciliation`, and in the grade's `--json` as a
top-level `reconciliation`.

- **Two declared or recorded values that disagree** are a CONTRADICTION, and
  `check_floorplan` lists it as undispositioned until it is answered in
  `dispositions.contradictions`, because the
  loser may be the right one. That includes a mechanical POSE for a part the
  brief declares an ARRAY member (`<ref>:array`): the file pins it, the row
  moves it, and no plan can hold both (the loader refuses a fixed pose on a
  member, and a file-locked member is an `array_conflict`). The declared brief
  wins, so the mechanical value loses and is not anchored; to keep the pose
  instead, take the part out of the array.
- **A plan that disagrees with the brief** is DRIFT, and the declared value wins.
  No `dispositions` entry clears it. `check_floorplan` reports it as DRIFT
  with the brief-clause wording.

**Anchors.** Each mechanical ref that wins is compiled at GRADE time, from the
file itself, into a grade-only anchor block `mech:<ref>`. The anchor is the
grader's own rect at the declared pose, rounded outward, and is graded by
`zone_containment` at a fixed ERROR. The `mech:` prefix is reserved in plans.

- The anchor itself does not check a lock. Each such ref that carries pads
  must be FILE-locked at its pose, or named -- unlocked, at that
  pose -- by the plan's `fixed_poses[]`, which `place_simple` seats there and
  holds. `--emit-intent` compiles such an entry for every anchored
  ref no edge connector, `must_lock` pattern, brief pose or array row
  already claims
  (`context.mechanical.fixed_poses` / `fixed_skipped`). A file-locked part is
  placed already, and its anchor is only graded.
- `mechanical_drift` reports any mechanical ref, locked or not, that moved or
  turned away from its declaration. It is an ERROR for a turn, which no anchor sees, and for any
  drift of a pad-less ref, which has no anchor -- and a plan's `severity` map
  cannot demote that ERROR, only promote the WARN it gives a move the anchor
  already reports.

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

## `--health`: what tells you the floorplan is wrong

Separate from the rules, and advisory. An intent violation says *"this is not the
floorplan you declared"*; a health signal says *"this floorplan will fight the
router whatever you declared"*: it is for *knowing when to stop routing and go
move something*.

```jsonc
"health": {
  "block_displacement_mm": 15.0,
  "bus_corridors": [ { "name": "sdram", "nets": ["SDRAM_*"], "width_mm": 8.0 } ],
  "classes":      { "SDRAM": ["SDRAM_*"], "USB": ["USB_*"] }
}
```

| signal | computable | what it means |
|---|---|---|
| **block displacement** | from geometry alone | the block's own pad centroid vs the centroid of everything it connects to |
| **bus crossings** | pre-route, but the corridor is a **model** | a straight rectangle between the bus's two pad clusters; its long sides are fed to the crossing kernel `quench._count_crossings_np`. A screening signal, not a verdict — real routes bend |
| **convergence** | only with declared `classes` | which critical classes crowd one corridor. Placement has no net-class notion and "critical" is design intent, not a fact in the file, so **it is skipped rather than guessed** |
| **net affinity** | from geometry alone | the per-PART inverse of block displacement: which single part carries a net its own block sits away from. See below |
| **escape lanes** | from geometry alone | per fine-pitch part, per face: lanes that fit against nets that must leave. Needs no declaration. See below |
| **blocked-cell share** | **not pre-route at all** | needs the router's blocker JSON, which only exists after a routing attempt. Reported as skipped, with that reason |

### `net_affinity`: the member a block metric averages away

Block displacement is an average over a block's members, so it is quiet when
*one* member is the problem: a series resistor zoned into a far-edge block can
carry most of a bus net's routed length while its block's average looks
merely displaced. This signal names the part.

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
using `QuenchState`'s airwire override (`_build_net_airwires`). It is an upper
bound: nothing checks that the part may legally sit there.

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
"Fine-pitch" is the absolute pitch test **or** the board's own rules: no track of the board's width passes between two adjacent pads at its
clearance, or two vias cannot sit side by side at the pad pitch — so a SOIC on
a board asking for track 0.3 / clearance 0.3 is measured, where the absolute
0.6 / 0.8 mm pitches never looked at it.

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
  inside rather than beside. What a charged neighbour CONTRIBUTES is its pad
  copper — not its courtyard, which is an assembly keep-out a track may legally run under, and
  not the bbox of its pad centres, which gives a two-terminal passive a
  zero-width body.
- **Interior pads count toward no face** and are reported separately. A boxed-in
  pad does not escape sideways at any pitch; it needs a via. Rolling it into a
  face's demand would blame the face for a fanout problem.

#### The layer term

`supply` and `deficit` are **own-layer**. Beside them the ledger reports what
the other signal layers could take:

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
  be a number that mysteriously ignores the stackup.
- **`via_slots` is measured on the face's full span, unobstructed**, while
  `supply` on the same row is not. Deliberate: `supply` is a floor and
  `via_slots` a ceiling, and feeding a `blocked_mm` that is too large into an
  upper bound would silently turn it into a lower one.
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

This is the same spatial incoherence that makes a sheet a poor unit to place
by: the groups file `place_simple` reads groups parts by function, not by
sheet ([py_placer/placement/README.md](../py_placer/placement/README.md)).

Parts already overhanging the outline are recorded as `edge_connectors` by
observation, which is what stops `oob_count` reporting a card edge or USB shell
as a defect forever.

**On a pile, no unlocked part's pose is read** (#1103). When
`PlacementState.pile` calls the board a pile (a heap, or a staging ring),
its unlocked parts' poses are staging, not decisions, and every staged part
is nearest SOME edge. So the emitter writes no edge read off such a pose
(with `--declare-classes`, an edge receptacle keeps its class and band, with
no `edge`), no zone, and no `overlap_area` or `oob_count` budget read off
the staging (both go to `context.budget_withheld`).
`context.pose_claims_withheld` says what was withheld and why, and lists the
staged parts off the board. A KiCad-locked part's pose is a decision and
keeps its claims, and a design brief or `mechanical.json` still merges after
this.

An edge part whose STRONG mating face points at another edge than the one it
sits nearest -- the opposite edge (seated reversed) or one at right angles
(opening along its edge) -- is not given that edge by the emitter: the entry
keeps its ref and band, drops `edge`, is flagged `suspect`, and its `note`
says which of the two it measured.
Declaring it would bless the very pose the facing conjunct fails. On the 22
tracked boards and the fa10 designer boards it never fires. Declaring the
edge such a part belongs on puts it back under the facing conjunct.

Emitted zones are rounded to 3 decimals and clamped inside the envelope,
which is written at 4, so a zone never rounds past its own envelope. An
overhanging edge member is not clamped out of its block's zone: the zone side
on its edge is open (above).

With `--declare-classes`, connector-family parts that claim no edge (headers,
JST, terminal blocks; `part_class` calls them `connector_affinity`) are also
recorded, with `"class": "connector_affinity"` and **no `edge`** (naming one
would be an invention). The grade then flags such a part seated more than
3 mm from every edge at `warn` only, because legitimately interior connectors
exist; write `max_setback_mm` on the entry to hold that setback at the
configured severity. Writing `edge` does not upgrade the interior flag --
it stays a `warn` -- but it adds the nearest-edge identity and the overhang
band at the configured severity. The facing conjunct never grades a
`connector_affinity` entry unless the entry declares `context.mount_mode`
`edge_mount` or `through_edge`, which outranks its class.

These entries are **declarations, not seat claims**. `edge_connectors`
therefore holds two populations, and `Intent.edge_claims()` is the split: it
drops `connector_affinity` and is what the grade's edge readers use -- the
overhang exemption (`_Ctx.oob_exempt`), a zone's open sides
(`claimed_edges`), `pins_to_edge`'s exclusion and the body-to-edge census's
edge parts (`edge_claim_refs`). The `edge_connector` **rule** reads the whole
key, because flagging an interior pose is the entry's only purpose. Writing
`max_setback_mm` on an entry changes its class-based severity, not its
membership -- to hand a part the edge-part treatment, declare it with an edge
class.

The `overlap_area` budget is **withheld** when the emitting board carries a
blocking body pair, or an unwaived courtyard interpenetration past the
blocking floors: baking the number would bless the board it was measured on.
It is also withheld on a pile, and when the board's project sets
`courtyards_overlap` to `ignore` (a courtyard overlap budget would grade
nothing it asks for). `oob_count` is withheld on a pile that has parts off
the board, and when an edge connector sits in a SUSPECT pose.
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

**And it is not a pass in the machine channel either.** A grade that left a
declared channel unmeasured reports
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

## `--declare-decaps`: a decap limit read off the board

`--declare-decaps` derives `decaps.max_distance_mm` from the board's own
tethers, which arms `decap_distance` on an emitted intent.

**`--decaps-from <placed board>`** derives the same number from a
PLACED REFERENCE of the same design instead -- a human layout, an earlier
placement -- which is the only way to arm the rule on a pile, where there is
nothing to read. Same derivation and withholding; the basis is
`reference:<file>`, in the census and in `context.basis`. The intent NAMES the
reference and never points at it: `context.decap_census.reference_board`
is the file's basename, `reference_board_sha256` identifies its bytes, and a
withholding reason names the basename too. The reference is usually a human
layout, and a run that grades against an intent carrying its path is one Read
away from the placement it must not match. It withholds when
the two boards share, under the same reference and footprint, under 90% of
EITHER board's pad-bearing parts (another design: a small board of generic
passives shares most of ITS parts with almost any large one), and when the
reference is itself a pile -- `PlacementState.pile`, the one pile test. A
placed reference with a few parts sharing an origin is NOT a pile: rp2350_fpga_eensy_prePlane's J2, a Tag-Connect on B.Cu, sits over
U3 on F.Cu by design, and a CM5 module footprint's two halves share one
origin. It is read as placed, and the shared ones are named in
`context.decap_census.reference_stacked_refs`. The limit is still read off
every part, because the reference must grade clean against its own number.
StickHub's human board, for example, gives 2.18 mm from 38 tethers. It also
derives the PIN limit, `max_pin_distance_mm` (see "The emitter derives no pin
limit" below for why only from a reference), and lists the caps
the reference keeps within the search radius (`within_radius_refs`, with the
radius in `within_radius_mm`): each of them is held to that radius, PER CAP.
Leaving a listed cap beyond it is a `decap_ungraded` ERROR, while a cap the
reference itself keeps beyond (likelier a bulk or filter cap) stays a WARN --
and so does a cap the reference lacks or carries under another footprint. "Held" means only
that the reference put the cap within 5 mm of its chip: against a machine
placement used as the reference, that can include a bulk cap.
A board graded on its own `--decaps-from` intent has 0 such
errors -- true by construction: a held cap is in the same `decap_populations`
near set the grade reads.

**Three states**, selected by `--no-declare-decaps`,
`--declare-decaps` (strict) and `--auto-declare-decaps`. The default,
`check_floorplan.DECLARE_DECAPS_DEFAULT`, is `off`.

- **`auto`** derives only off a PLACED board. It keys on
  `PlacementState.pile`, so a heap of parts is never read as a placement,
  while a placed board with a pair of parts that share an origin by design
  still counts as placed. When auto withholds, it records why in
  `context.decap_census.auto_withheld`, never in `budget_withheld`, so an
  auto emit moves no exit code.
- **The number is labelled** `observed_baseline` in `context.basis`, alongside
  every other number the emitter read off the board (the legality budget, the
  observed edges and overhang bands, block sides and zones, `assembly.sides`);
  the module's own constants are `derived_default` (`envelope.tolerance_mm`,
  `defaults.zone_tolerance_mm`, and an overhang band that is a class default,
  the sanity cap or the connector-affinity floor rather than a reading). The `decap_distance`
  message says "an observed regression baseline read off a board, not an
  electrical requirement" -- only while the limit is still the one the census
  recorded (`emitted_max_distance_mm`): a hand-edited limit loses the label
  when the intent is read. A brief merged over the intent re-labels whatever it
  declares.
- **A declared relation supersedes the inferred tether.** A `proximity` claim
  the BRIEF makes that names the cap's pads on the cap's rail, and that the
  graded intent carries unchanged (same partner, limit, basis and pads), takes
  that cap out of the decap rules at grade time. A row only a plan carries
  supersedes nothing, and neither does a brief row the plan dropped or
  changed. `decaps.exempt` is never written, because exempting the cap would
  turn a declared `max_pin_distance_mm` on its rail into `decap_pin_uncovered`.
  When every cap is superseded, an armed decap rule runs and skips each one
  (the grade stays complete), and a dark one is not applicable. The census
  lists the superseded caps under `superseded`; the emitted limit is still
  read off every cap, because a tool handed only the intent grades without
  the brief.

**The default is `off`.** `check_floorplan --emit-intent` adds no decap
flag, and `/plan-pcb-placement` adds `--decaps-from` only when a placed
reference of the design exists. The rule roster (`check_floorplan --plan-only`) asks the question instead: an applicable dark
`decap_distance` is answered by a limit from a requirement, or by a written
disposition.

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
board it was written from (splitflap_driver's 3.4617228 rounds to 3.4617).
`context.decap_census`
therefore carries `max_mm` unrounded and `max_mm_display` for the reader.

### When it is withheld

A limit read off the board can never be violated by that board, so that is not
the test. The test is **censoring**. `groups.decap_tethers` drops any decap
further than `DECAP_RADIUS_MM` (5 mm) from its nearest owner, so the
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

### What the census discloses that the rule cannot see

`context.decap_census` is written on **every** emission, with or without the
flag, so a reader of the document can tell "no cap is far from its IC" from
"nobody measured". It carries the metric, the search radius, the tether and IC
counts, the max and median, and — the point of it — `beyond_radius`,
`beyond_radius_refs` and `worst_beyond_mm`.

The emitter and the rule both measure tethers at the same 5 mm truncation, so
a cap farther than that from its IC is invisible to `decap_distance`.
`decap_ungraded` reports those caps at **warn**, as a COVERAGE claim rather than
compliance: *your limit was derived from the caps inside the radius, so it says
nothing about this one.* Widening the radius would not help: the emitter's
`ceil(max)` would no longer cover the newly visible caps, so the round trip
would stop grading clean on its own board, and widening both would turn
splitflap's limit from 3.4618 into 19.30 mm, a limit that says nothing.

The radius does not reach the election: the IC a cap is graded against is the
same with or without it.

A board whose limit is WITHHELD cannot arm either decap rule, so the horizon
reaches it through the abstention channel instead: `_WITHHELD_RULE` maps the
key to **both** rules, and the withholding reason names the beyond count and
the worst distance, on every withholding arm.

### What reads the declared limit

`place_simple` prices it: at error severity, each mm a cap or a supply pin is
past `max_distance_mm` or `max_pin_distance_mm`, measured as the grade
measures it, costs `--decap-limit-weight` (default 10; 0 leaves the limits
unpriced), on top of its decap term, which pulls each cap toward a supply pin
of its rail on an IC of its own group (any IC's when the group has none)
whatever the limit. `place_fanout_clearance --intent` (and the GUI
fanout tab, which shares its engine) holds it: at error severity,
`max_distance_mm` and `max_pin_distance_mm` are held on every cap it moves --
no move may take a claim past its limit and further than the board as it
stands, unless no clear pose keeps it, and then the claim it broke is named.
The key is opt-in, default off, and on its own flag rather than
folded into `--declare-classes`.

The census (`context.decap_census`) splits the caps it scopes. Measured:

| board | in scope | graded | beyond the radius | no rail-carrying chip | predicate |
|---|---|---|---|---|---|
| ulx3s | 70 | 53 | 7 | **10** | **0** |
| glasgow_revC | 92 | 87 | 5 | 0 | **0** |
| watchy | 28 | 24 | 2 | 2 | **0** |
| splitflap_driver | 12 | 11 | 1 | 0 | **0** |

**The gap is not the predicate.** The last column is the residue attributable
to the "is this a decap" predicate: measured over every tracked board, its
spellings name **identical** sets, and no tracked board even has a lowercase
`c*` reference for a case difference to bite on.

The gap is two other things, and the census reports them separately
(`beyond_radius` and `no_rail_chip`, with `unaccounted` staying 0 so a reader
can check that the three arms are the whole scope):

* **beyond the 5 mm radius** — a real grading hole, and `decap_ungraded`'s
  business;
* **no chip carries the cap's rail at all.** Ten of ulx3s's seventeen: `C3 C4
  C22` on `/power/P1V1`, `C7 C8 C24` on `/power/P3V3`, `C11 C12 C23` on
  `/power/P2V5`, `C14` on `/power/SHUT`. Those rails are owned only by two-pad
  passives — the caps themselves plus `L1`–`L3` and `RA*`/`RP*` — so they are
  bulk and filter caps upstream of an LC network, not decouplers. **The grader
  is right to ignore all ten**; there is no IC for them to be far from.

## Grading the pin, not the package

`decap_distance` measures the cap's **centroid** to the IC's pad bounding box
inflated 0.5 mm, clamped to 0 inside. The requirement it stands in for is about
the **pin**: a 100 nF sitting 1.7 mm from a QFN and 9 mm from the IOVDD pin it
decouples satisfies that rule and fails the spec.

`decaps.max_pin_distance_mm` grades the pin. It is **opt-in with no default**:
at 3 mm the tracked corpus produces 127 findings, the worst 37 mm, so any
shipped default would flood every board.

**The invariant.** Compute a SUPERSET of the caps that could serve a pin, and
violate only when the MINIMUM over that superset exceeds the limit. Every cap
added can only lower the number, so the only way to manufacture a violation is
to have MISSED a cap — which turns "is this finding real?" into one auditable
question. The answer: every decap (below) on the pin's exact net, either side,
any distance, whoever else it also serves.

**What a decap is -- one rule for the grade, the gate and the placer.** A cap
(`groups.is_decoupling_cap`: a `C*` bridging two nets) is a decap
(`grouping.BoardView.decaps`, read through `groups.decap_population`) when it
runs

* from ground (`grouping.is_ground`) to a rail some chip other than a
  connector has a supply pin on; its owners are those chips. A part whose
  pads span half the board's bounds or more is the board's own contacts
  (`grouping.is_carrier`: a module's castellated or LGA pads, a Teensy or
  DIP-40 form factor's pin rows) and owns none, like a connector: every part
  on such a board lies inside its pad span, so it was the nearest owner of
  every cap;
* or between two rails that each carry a supply pin of ONE chip -- a stacked
  domain, where one die's VSS is the next one's ground (bitaxe's BM1366, VDD1
  to VDD2 to VDD3), or a chip's own analog ground (an ECP5's VCCPLL to
  GNDPLL). Its owners are the chips with a supply pin on both, a switch-node,
  bootstrap or charge-pump pin aside, so a buck's BST-to-SW cap is no decap.

A chip's supply pins here are the pin rule's (below: one channel per chip)
joined with its pins on a REAL rail (`BoardView.rail_pins`), which that one
channel leaves out when the chip wins on another: tigard's FT2232H wins on
its typed pins and lost VPHY, VPLL and VREG. A pin of a chip with a ground
pad is on a real rail when its name does not read as a signal
(`grouping.reads_as_rail`) and either its net is NAMED as a rail (`/VPHY`,
`V1P8`, `/DCDC/16V_OUT`) or its own name reads as one (`REGOUT`, `CPOUT`,
`AVIN`) and the net is local to it: nothing else on it but caps to ground,
an inductor, ferrite or test point, or a resistor from a net named as a
rail (an RC supply filter). A pin or net named as a reference or bias
(`VREF`, `VCOM`), a sense or detect input, a compensation, filter or tuning
node, a soft-start or slew pin, a switch node or bootstrap, a reset or
enable, or a clock is a signal, so its cap stays out.

`decap_distance` elects the nearest owner and `place_simple` prices the same
caps. An RC cap on a reset pin, a crystal's load cap, a cap on a connector's
power pin, a compensation or bootstrap cap is no decap, so its distance neither
sets an emitted limit nor is graded by one. The pin rule grades its own
ladder's pins only (the rail pins are not graded), against the decaps on each
pin's net: a stacked cap counts on both of its rails.

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
pins rather than on whether its field is populated -- plus channel 0 for a
supply owner:

| # | channel | test |
|---|---|---|
| 0 | `role` | a supply owner a `power_out` pad or the role table calls a regulator (below): every non-ground pin |
| 1 | `pintype` | token-split on `+`; `power_in`/`power_out` present, `no_connect` absent |
| 2 | `pinfunction` | exact or prefix match against `decaps.pin_functions`, else the built-in table |
| 3 | rail net | the net NAME looks like a rail **and** already carries a decoupling cap |

**Who owns caps.** The parts asked about are the cap owners
`groups.chip_refs` names -- the same set the tether election and
`QuenchState`'s tether terms read. An owner is an IC by shape (four
or more copper pads, not a single row, and not a passive by role: #1351 drops a
part the role table calls a resistor, diode or LED, or a resistor network
(`RN`, `R_Array_*`), unless a pin of it is typed or named as a supply pin on a
rail or its name says it is an addressable LED -- so rp2350's pull-up arrays
RN1/RN2 and its RGB LED D1 no longer own the +3V3 caps, while an SK6812 still
owns its VDD cap), or a **supply owner**: a part with at
least three copper pads and no `part_class` that has a pad typed `power_out`,
or that `board_context`'s role table calls a `regulator`
(`placement/roles.py`), or that has exactly three copper pads on three nets --
one ground, and two that each carry a decoupling cap. So a 3-pad SOT-89 or
TO-92 regulator owns its caps: esp_prog's C3 and C1 (the ME6210's VOUT and VIN
caps) elect U2, and complex_hierarchy's C102 / C104 elect U101. No one signal
covers both boards -- U2 carries no pintype, U101's
footprint name says only `TO-92` -- hence three. The three-pad shape also
admits kit-dev LV101, a DS1818 reset supervisor whose VCC and RST nets are
both capped. Channel 0 exists
because a regulator's VIN and VOUT are supply pins by ROLE: U2 has no pintype
and its VIN net is `Net-(C1-Pad1)`, which channel 3 never reads as a rail, and
U101's VI is typed `input`. It yields only for an owner a `power_out` pad or
the role table calls a regulator (`groups.regulator_owners`) -- the shape alone
says a part owns caps, not which pins are supply pins, so LV101 keeps the
ladder (its +3.3V pin, not RST) -- and its findings are `decap_pin_distance` (error), like channels 1-2.

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

A rule the intent asks for but the board cannot answer is not counted as run.
A per-rule `_ARM` table, run after `_wants` (which reads only the intent),
puts it in `rules_skipped` with a reason that names the pad census. Counting
it as run would make `--require-rules` *easier* to satisfy -- the vacuous pass
that flag exists to catch. `orangecrab_ext_pll`, for example, has 28 candidate
ICs and zero supply pins on any channel, and says so.

This is deliberately **not** the `budget_abstained` channel, which means "the
emitter could not derive this key" — a property of the intent, computed with no
board. This is the opposite, and overloading one key with both would make
`budget_abstained_keys` mean two things.

### `same_side` is a manufacturing claim, and it is expensive

Default **false**, and it must be: 43% of corpus decap tethers (152 of 357) put
the cap on the opposite side from its IC — glasgow_revC 61 of 87, orangecrab 44
of 74 — because a decap directly under its pin on the back side, connected by a
via, is standard practice. On `glasgow_revC`, U30 is on F.Cu with **all 15**
of its tethered decaps on B.Cu.

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
  decap on this board (from ground to it, or a stacked one) carries that net.
* A **clean** result does not mean the board is decoupled. It means every pin
  the rule could identify had a cap on its own net within the limit. When it can
  identify none, it says so in `rules_skipped` and does not report a pass.
* It does not claim its number equals `decap_distance`'s. They answer different
  questions, in different currencies, over different populations.
* `is_ground_net_name` does not match `VSS`, so a board that spells its ground
  `VSS` will have ground pins graded. No tracked board does, so the case is
  structurally untestable here — disclosed rather than fixed with a tenth
  power/ground predicate.

### The emitter derives no pin limit from the board itself

`ceil(max)` over pin gaps is censored by construction — an uncovered pin
contributes no distance, so the max is taken over survivors, which is the exact
censoring failure `--declare-decaps` spends a table on — and a limit derived
from a board and then graded against that board is vacuous. The distribution
goes to `context.decap_census` instead, which has no key set to grow.

**Except from a reference.** `--decaps-from <placed board>` derives
`max_pin_distance_mm` from the REFERENCE, where neither objection holds: it is
not the board being graded, and the derivation is withheld when more than 25%
of the reference's supply pins have no cap on their net (the censoring case) or
fewer than 3 are covered. `decap_ungraded` is promoted to error per cap,
through `decaps.within_radius_refs` (above), so a cap stranded beyond the
radius is named even when the reference has bulk caps of its own out there.
The reference grades clean against its own limits.

### `keepouts` stays empty, and says so

A keep-out is a mechanical fact — an enclosure rib, a standoff, a battery, a
display window, an antenna clearance — and none of those can be read off a
board, so the emitter keeps writing `[]`. What it should not do is leave the
reader unable to tell *"none declared"* from *"not considered"*, so
`context.keepouts_note` states which one it is. At grade time the same
distinction is already carried by `rules_skipped` and by `--require-rules`.

### A PCB-edge plug's mating region is a keep-out nobody has to declare

One keep-out CAN be read off a board, because the footprint states it: the
tongue of a plug made of board copper (a PCB-trace USB plug, a card edge),
which has to enter a socket with nothing on either face. Such a plug is SMD
fingers with a courtyard on F.CrtYd only, and courtyards are per side, so no
courtyard check sees a part on the back of the tongue.

`floorplan.derived_mating_keepouts` derives one keep-out per plug, applied at
the consumers rather than written into the intent:

- **a plug is** a footprint KiCad does not assemble (`board_only` or
  `exclude_from_pos_files`, and no 3D model), not a net-tie, with >= 4 netted
  pads (a USB-A PCB plug has four; KiCad's 2- and 3-pad solder jumpers carry
  the same attrs) and none drilled;
- **it is seated** while none of its pad copper is past the outline, >= 2 of
  its netted pads are within 1 mm of the outline, and its courtyard is on the
  board and reaches the outline. Only a seated plug derives a region, and only
  a seated plug is locked -- by `QuenchState`, so `place_simple` never moves
  it, and by `place_pose`, which refuses to move it unless it is named in
  `unlock`, since the region moves with it.
  A plug in the staging pile or hanging across an edge is free to move, and
  the off-board copper gate reports its copper;
- **the region is** that courtyard's board rect inset by 0.25 mm, on BOTH
  faces, allowing the plug itself and every part with no copper pad (a slot
  such as StickHub's H1). The inset keeps a neighbour's courtyard margin that
  grazes the tongue's root (StickHub J2, J6: 0.15 mm) from reading as a part
  on the plug;
- **it binds** `QuenchState` (which takes the union, so `place_simple`'s
  anneal and its legality check see it), every grade (`_Ctx` grades the
  intent plus it), `grade_pad_legality`'s `mating_keepout_*` (which
  `place_pose` gates on) and `check_assembly`, where it is a NOT BUILDABLE
  conjunct. All of them measure `QuenchState`'s rect (courtyard, else pad
  bbox), so the placer and the checker cannot disagree;
- **a declared keep-out named `mating:<ref>`** (intent or brief) replaces
  the derived one, for a plug whose real insertion depth differs -- in every
  consumer handed the intent, `check_assembly --intent` and `place_pose
  --intent` included (`floorplan.mating_keepouts`). The plug it names is
  locked by the same seated test.
