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
    { "ref": "J1", "edge": "north", "overhang_mm": { "min": 0.0, "max": 1.5 } }
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
| `edge_connectors[]` | `ref`, `edge`, `overhang_mm`, `max_setback_mm`, `class`, `note`, `context`, and the emitter-written `source`, `suspect`, `suspect_reason`, `overhang_capped`, `observed_overhang_mm` |
| `edge_connectors[].overhang_mm` | `min`, `max` |
| `decaps` | `max_distance_mm`, `exempt`, `search_radius_mm` |
| `legality_budget` | `overlap_area`, `oob_count`, `oob_amount` (`oob_area` refused — see below) |
| `health` | `bus_corridors`, `classes`, `block_displacement_mm`, `ignore_net_ids`, `max_fanout`, `zoned_blocks`, `affinity_exempt_nets`, `affinity_exempt_net_ids` |
| `health.bus_corridors[]` | `name`, `nets`, `width_mm` |
| `severity` | any of the 13 rule names below |
| `overlap_waivers[]` | `pair`, `reason`, `context` |
| `must_lock` | a list of reference globs (no nested keys) |

`severity` keys are checked too. The settable names are the nine rules —
`envelope`, `zone_containment`, `zone_side`, `zone_exclusive`, `keepout`,
`edge_connector`, `decap_distance`, `must_lock`, `legality` — plus the four
findings raised outside the rule loop: `intent_zone_outside_envelope`,
`intent_zone_overlap`, `block_unresolved`, `intent_zone_in_keepout`.
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
`1`) is what this build can act on, and an intent sets `min_reader` when a
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
| `zone_exclusive` | a non-member intrudes on a reserved zone | `rect_overlap_area`. **Enforced since [#702](https://github.com/drandyhaas/KiCadRoutingTools/issues/702)**, same way |
| `keepout` | any part enters a keep-out, unless in `allow` | courtyard **and** through-hole rect. **Enforced, not only graded, since [#701](https://github.com/drandyhaas/KiCadRoutingTools/issues/701)** — the seat search refuses such a pose through the same `keepout_hit` this rule calls — and since [#702](https://github.com/drandyhaas/KiCadRoutingTools/issues/702) the quench refuses such a MOVE through it too |
| `edge_connector` | overhang outside `[min,max]`, or the wrong edge; a `connector_affinity` entry seated more than 3 mm from every edge fires at **warn** whatever the configured severity | `BoardOutlineGate.rect_outside_amount`, `edge_clearance` |
| `decap_distance` | a decoupling cap is too far from its own IC | `groups.decap_tethers` |
| `must_lock` | a declared-critical part is not locked in the file | `parser.extract_locked_refs` |
| `legality` | overlap or off-board parts exceed a budget | `QuenchState.legality_metrics` |
| `block_unresolved` | a block matched no footprint | — |
| `intent_zone_overlap`, `intent_zone_outside_envelope` | the intent contradicts itself (no board needed) | — |

Every one of them measures with the geometry the **optimizer itself gates on**.
A grader with its own idea of what "legal" means grades the reimplementation
rather than the board — so where re-deriving was plausible, a test asserts the
two agree exactly (all 34 of ulx3s's cap distances identical to the grouper's,
all five legality numbers identical to the quench's).

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
| `zone_exclusive` | yes | no | **yes** | the seeder has no verdict string for this refusal yet |
| `keepout` | yes | **yes** | **yes** | — |
| `must_lock` | yes | — | **by freezing** | it is a claim about the FILE; no pose satisfies or violates it |
| `edge_connector` | yes | anchor tier | **by freezing** | two of its three sub-claims are bounds on being *off* the board, so a per-pose term would fight the containment gate rather than complement it |
| `zone_side` | yes | — | no | **vacuous, not conservative**: the quench never flips a side, so the term is invariant under every move it can make. Reported once at load instead |
| `envelope` | yes | — | no | a claim about the intent FILE against the board, not about any pose |
| `decap_distance` | yes | scope stage | no | graded in a currency the optimizer does not carry — pad centroid to an IC's pad bbox inflated 0.5 mm, not courtyard to courtyard. A gate in the wrong currency can *admit what the grade flags*, which is worse than no gate. And the cap→IC tether is re-elected from live poses, so a per-move form would have the `corridor_weight` non-stationarity problem too |
| `legality` | yes | — | no | a whole-board aggregate against a BUDGET, so a per-pose form is non-local: whether A's move is admissible would depend on B's violation |

Enforcing is not free, and the price is recorded rather than described:
`tests/test_placement_ab.py` carries four `intent-*` rows and
`tests/placement_ab_baseline.json` the numbers. A hard constraint removes poses
from the search, so `crossings` and `hpwl` can get worse — on ulx3s they do,
and that row is pinned `regress` with the containment errors going 4 → 0.

### A through-hole part is in a keep-out from either side

Its leads pass through. `keepout` tests the courtyard **and** the drilled-pad
rect against every face the part occupies, so a mounting-hole keep-out cannot be
walked through from the back.

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
  difference between a signal and an action.
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
  `supply` on the same row is not. Deliberate: `blocked_mm` is side-blind and
  container-blind (on rp2350, U8's pad rect *contains* U6's and is charged the
  whole 6.90 mm face), and feeding that into an upper bound would silently turn
  it into a lower one.
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

A worked pairing from ulx3s: the ledger reports `U9 west: supply 6 < demand 14`
with `15.35mm of that face is taken by SD1`, and `net_affinity` independently
reports SD1 carrying 57–63% of `SD_D0`, `SD_D1` and `SD_CMD`. Two signals
computed from different quantities naming the same part is the case worth
acting on.

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

That last group is a **known, unfixed hole**, disclosed rather than closed: the
emitter and the rule both call `decap_tethers` at the same 5 mm truncation, so
`splitflap_driver` emits a limit of 3.4618 while a rail-sharing cap sits
**19.30 mm** from its IC, invisible to both. `rules_run` goes up; that cap is
still ungraded.

### Declaring this key CHANGES PLACEMENT

It is not a grading-only knob, which is why it is opt-in, default off, and on
its own flag rather than folded into `--declare-classes`. `place_seed` reads
`decaps.max_distance_mm` and, when it is set, pulls every two-net-bearing-pad
`C*` out of radial zone packing into its per-supply-pin stage. Measured:

| board | seeder scope | graded tethers | in scope, never graded |
|---|---|---|---|
| ulx3s | 70 | 53 | 17 |
| glasgow_revC | 92 | 87 | 5 |
| watchy | 28 | 24 | 4 |
| splitflap_driver | 12 | 11 | 1 |

The two populations differ because the two "is this a decap" predicates
differ — the grouper tests *exactly two distinct net ids*, the seeder *exactly
two net-bearing pads* — and the metrics differ too. `context.decap_census`
carries `seeder_scope` and `seeder_scope_ungraded`, and `--emit-intent` prints
the same warning. Reconciling the predicates is a separate change.

### `keepouts` stays empty, and says so

A keep-out is a mechanical fact — an enclosure rib, a standoff, a battery, a
display window, an antenna clearance — and none of those can be read off a
board, so the emitter keeps writing `[]`. What it should not do is leave the
reader unable to tell *"none declared"* from *"not considered"*, so
`context.keepouts_note` states which one it is. At grade time the same
distinction is already carried by `rules_skipped` and by `--require-rules`.
