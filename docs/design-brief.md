# The design brief: what the board cannot know (#711)

Placement in this toolchain infers everything about a board from the board.
`emit_intent` says so in its own docstring — *"a starter intent READ OFF the
board … the emitted intent grades CLEAN by construction"* — and every
connector's edge is **guessed** from its current pose by `_nearest_edge`, which
is the only source of an edge anywhere in the toolchain.

A board file cannot say which end of a stick the USB plug is, which header a
ribbon mates with, or where the enclosure rib runs. Those are facts about the
product, and until now there was nowhere to put them: `board_brief.py
--requirements` takes prose, carries it verbatim, and is read by nothing.

The design brief is that channel.

```bash
# it is a sibling of the board, discovered automatically
ls board.kicad_pcb board.design-brief.json

# authoring: the brief is COMPILED into the intent, which is what gets graded
python3 py_tools/check_floorplan.py board.kicad_pcb \
    --emit-intent floorplan.json --declare-classes

# grading: the intent is graded; the brief REPORTS DRIFT, it does not merge
python3 py_tools/check_floorplan.py board.kicad_pcb --intent floorplan.json
```

`--brief PATH` overrides the sibling. `--no-brief` suppresses discovery — the
OFF arm, without which there is no way to measure what declaring changed.
`--require-brief` exits 4 when none was found, the parallel of
`--require-rules` one level up.

## It is a compiler, not a second constraint system

Everything a brief declares becomes an intent entry that the existing rules
already grade and the existing seat search already honours — ordinarily an
`edge_connectors[]` or a `keepouts[]` one. Provenance rides in
`source: "brief"` and `context`, both of which
[the intent schema](floorplan-intent.md) already accepts.

| brief | becomes | provenance |
|---|---|---|
| `interfaces[]` with an `edge` | an `edge_connectors[]` entry — `edge`, `overhang_mm`, `center_on_edge` / `along_edge_band`, and `class: "edge_receptacle"` when `user_facing` is true | `source: "brief"` |
| `interfaces[]` with `edge: "unknown"` | an entry with **no** `edge` key | `source: "brief"`, a `note` naming the unknown |
| `keepouts[]` | `keepouts[]`, verbatim; `kind` and `why` move into `context` | `context.source` |
| `proximity[]` | `proximity[]`, one row per member of a list `ref`; `why` and `requirement` move into `context` | `source: "brief"`, `context.proximity_note` |
| `fixed[]` | `context.brief.fixed` — carried, never asserted (see below) | — |
| `product`, `unknown[]`, `mount_mode`, `cable_entry` | `context` | — |

### The one key the compiler adds, and why that is the rule rather than an exception

Until [#902](https://github.com/drandyhaas/KiCadRoutingTools/issues/902) this
section said the compiler adds **no** intent key. It adds exactly one now,
`proximity[]`, and the principle is unchanged — because the principle was never
"never add a key". It is **never declare what nothing grades**, and this key
is only half of the change: the other half is the rule in `floorplan`
that grades the compiled rows, and a build carrying this key without
that rule declares something nothing measures.

It needed a key of its own because no existing entry means *these two named
parts, this far apart*:

- `decaps.max_distance_mm` is **one board-wide budget over a derived
  population**, not a claim about a named pair — and its partner election is
  exactly what #902 is about. A decap is syntactic (`ref` starts with `C`,
  bridges two nets) and the IC it is tethered to must carry ≥ 4 copper pads, so
  a 3-pad SOT-89 regulator can never be a tether target *at any radius*.
  Measured on the board that motivated the issue, the election gives
  `C1 → USB1 2.03 mm` and `C3 → U1 1.83 mm`: the regulator's own bulk caps,
  graded against a USB socket and a bridge IC.
- A **keep-out is an exclusion, not an attraction.** Compiling "beside" into
  "not inside" grades the opposite claim.

So the honest choice was a key that arrives with a rule, rather than a spelling
that quietly graded something else.

A board that carries no brief costs nothing: discovery returns the empty value
and every code path is the one that existed before. `--no-brief` on a board that
*does* carry one reproduces the pre-brief document exactly, and a test asserts
it.

## The smallest useful brief is three fields and one row per connector

```jsonc
{
  "schema": 1, "kind": "design-brief", "units": "mm",

  "product": {
    "form_factor": "usb_stick",              // free text, or "unknown"
    "primary_axis": "east-west",             // east-west | north-south | unknown
    "held_by": "hand; plugs into a host USB-A port",
    "user_top_side": "F"                     // F | B | unknown
  },

  "interfaces": [
    { "ref": "USB1", "role": "host_uplink",
      "user_facing": true,                   // true | false | "unknown"
      "edge": "east",                        // north|south|east|west|"unknown"
      "along_edge": "center",                // "center" | {"from":f, "to":f} | "unknown"
      "along_edge_tolerance_mm": 0.5,        // REQUIRED with "center"
      "overhang_mm": { "min": 0.0, "max": 0.65 },
      "mount_mode": "edge_mount",            // carried, not graded
      "cable_entry": "in_plane",             // carried, not graded
      "requirement": "PROG-CONN01",
      "why": "the board IS the plug body" }
  ],

  "keepouts": [
    { "name": "battery", "rect": [4, 20, 26, 44], "sides": ["B"],
      "allow": ["BT1"], "kind": "enclosure_rib", "why": "CR2032 holder" }
  ],

  "proximity": [
    { "ref": "Y1", "near": "U1",              // ref may be a LIST: ["C1","C3"]
      "max_mm": 2.0,                          // or "unknown" -- see below
      "basis": "pad_edge",                    // "pad_edge" | "body"
      "pads": { "Y1": ["1","2"], "U1": ["9","10"] },   // optional; STRINGS
      "requirement": "PROG-CLK01",
      "why": "the crystal loop sets the oscillator's stability" }
  ],

  "fixed":   [ { "ref": "MH1", "why": "M2.5 enclosure boss" } ],
  "unknown": ["mounting_datum", "panel"]
}
```

`user_top_side` is which face the **user looks at** — an enclosure fact. It is
not the intent's `assembly.sides` (#837), which is which faces the **fab
populates**. The two are independent and cannot contradict each other: a
back-populated board whose front carries the label is `assembly.sides: "B"`
with `user_top_side: "F"`, and that is coherent. `user_top_side` is carried and
graded by nothing, and the brief report says so under `not_graded`;
`assembly.sides` is graded by `rule_assembly_side` and charged by
`options.grow_board` — which since
[#878](https://github.com/drandyhaas/KiCadRoutingTools/issues/878) also charges
a through-hole part's leads against the face it is *not* mounted on, so the
busier face is the busier **obstructed** one.

Every key is optional. Strictness is the same as the intent's, at every level
and for the same reason (#710): a typo'd key that loads clean is a constraint
the author believes they set and nothing ever checks.

`schema` is matched exactly and `min_reader` is the field vocabulary — the same
two-number policy the intent uses, copied rather than reinvented.

## "I do not know" is a value, and it is not the same as saying nothing

The failure this is designed against is **a brief nobody writes**, so a
half-written brief must not read like a complete one. Three states, reported
apart everywhere:

| state | written as | reported as |
|---|---|---|
| **declared** | the value | compiled into the intent, graded |
| **declared unknown** | the literal `"unknown"`, or the field named in `unknown[]` | `brief_unknown` — never compiled, never guessed |
| **not declared** | the key is absent | `brief_absent` — nobody looked |

`edge: "unknown"` compiles to an entry with **no** `edge` key, and that path is
already correct everywhere: the seeder's stage 1 skips edge-less entries, the
repair path refuses honestly, and `emit_intent` refuses to name an edge for an
implausible pose. "I do not know" reaches machinery that knows what to do
with it.

When no brief is found the tool says so, and says what is filling the gap:

```
design brief: none beside this board (looked for board.design-brief.json).
Every `edge` below is INFERRED from the part's current pose by _nearest_edge;
nothing here declares design intent.
```

`JSON_SUMMARY` carries `brief`, `brief_declared`, `brief_unknown`,
`brief_unknown_keys`, `brief_absent` and `brief_drift`, so all of this is
machine-visible rather than prose.

## `proximity[]`: the constraint the netlist implies and nothing measures (#902)

A 3 mm crystal loop and a 30 mm one have **identical connectivity**. Every
instrument in this toolchain reads the board, and the board cannot tell them
apart, so "Y1 beside U1, as short as possible" was ungradable until this key
existed. One row is one claim is one limit:

| key | | |
|---|---|---|
| `ref` | required | a reference, or a **list** of them |
| `near` | required | the part the subject(s) must stay close to |
| `max_mm` | required | a positive distance, **or `"unknown"`** |
| `basis` | optional, default `pad_edge` | `pad_edge` or `body` |
| `pads` | optional, or `"unknown"` | `{"REF": ["1", "2"]}` — pad numbers as **strings** |
| `requirement`, `why`, `note`, `context` | optional | carried into the compiled entry's `context` |

**A list `ref` is sugar and is expanded at compile time.**
`{"ref": ["C1","C3"], "near": "U2", "max_mm": 2.0}` becomes two intent rows.
It does *not* mean "every pair within the list": a three-part list would then
be three claims sharing one number, and a violation could not say which pair
the limit was about — `Violation.ref` holds exactly one reference. "Q1 and Q2
together" is already `{"ref": "Q1", "near": "Q2"}`, so nothing is lost.

**Identity is the ordered `(ref, near)` pair.** A duplicate is refused for the
reason a duplicate `interfaces[].ref` is: two limits on one relation, with no
rule for which wins, and the rule charges both. The *reversed* pair is refused
too — but only when neither row names a `pads` list **for its own subject**,
because only then are the two provably the same symmetric number charged twice.
A subject pad list is what turns the existential minimum into *for each of my
pads, some pad of yours is close enough*, so with one on each side the two rows
are different claims and both are kept.

**A claim is reported as `proximity[<row>:<ref>~<near>]`.** The row index is
there because a reference may legally contain `~` — `disambiguate_references`
produces `TP4~2` for a duplicated refdes, and `esp_prog` parses `Ref*~2` — so
without it a row `A~B` near `C` and a row `A` near `B~C` share one id, and one
of two declared "I do not know"s vanishes into a set union. Use
`design_brief.proximity_claim_id()` rather than spelling it by hand: the same
string is meant to name a claim for every later reader of one, and hand-written
f-strings at each site would drift.

### Two "unknown"s that go opposite ways, and why

This key is where the three-state contract above does its most visible work.

- **`max_mm: "unknown"` is accepted**, and compiles to **no intent row**. "As
  short as possible" is a real thing for a spec to say: the author has declared
  the *relation* and not the *number*. It is reported as
  `proximity[0:Y1~U1].max_mm` in `brief_unknown_keys` (`brief_unknown` is the
  count) and never appears in `declared` — a limit nobody stated cannot be
  graded, and inventing one is the guess this channel exists to refuse.
- **`basis: "unknown"` is refused**, alone among the enums here, because
  `basis` **has a default**. Declaring it unknown would compile to that default
  while the report says nobody knows — two different documents. Omit the key to
  take the default, or name the one you mean.
- **`pads: "unknown"`** is accepted and sits between the two: the row still
  grades, part to part, and the reader is told the pin-level claim was not
  stated rather than left to infer it from an absent key.

### `basis` is `pad_edge` or `body` — and `"courtyard"` is refused by name

`pad_edge` measures pad copper to pad copper (`legality.pad_rect` +
`rect_gap`), which is the currency `decap_pin_distance` already uses. `body`
measures the drawn bodies through
[#896](https://github.com/drandyhaas/KiCadRoutingTools/issues/896)'s one body
model, and it exists because two parts a brief wants "together" may share no
net at all — an auto-reset transistor pair has no pad pair to measure.

`basis: "courtyard"` is refused *with the measurement that decided it*: since
#896, `placement.body` is a **ladder** — courtyard, then fab, then silk ∪ pads,
then the pad bbox — and it answers with whichever rung the library drew. On the
board this rule was written for, **0 of 21 footprints draw a courtyard**: 10
answer from fab, 4 from silk, 4 from the pad bbox and 3 draw nothing at all. So
a claim spelled `courtyard` would grade nothing there. The rung that actually
answered is reported beside each number, so it is never read without knowing
what it rests on.

### Pad numbers are strings, and that refusal is load-bearing

`Pad.pad_number` is a string on both parse paths. A JSON `1` would match no
pad, resolve an empty pad set, measure nothing and grade **clean** — a refusal
that reads as a pass, in the one direction nobody checks. So an integer pad
number is refused at load, naming exactly that consequence.

## Declared outranks inferred — and the evidence survives

On `--emit-intent`, a brief entry overwrites the emitter's guess **per key**,
while the emitter's *evidence* is kept: `observed_overhang_mm`, `suspect` and
`suspect_reason` all survive. A brief naming an edge for a part the emitter
marked SUSPECT keeps both — the suspect bit is a fact about the **board**, the
brief is a claim about the **spec**, and both are true at once. Dropping the
first is how a damaged pose gets laundered into the spec meant to gate its
repair.

A contradiction is **printed and resolved in the brief's favour**:

```
CONTRADICTION J1: the brief declares the south edge, the board observes north
-- the brief wins, and the grade will flag the part
```

That is the point of the channel. An intent derived from a board can only ever
re-state that board, including its damage.

A brief ref the board does not have is **kept**, not dropped, and named:
dropping it would make a typo grade clean, which is `block_unresolved`'s failure
one level over. `rule_edge_connector` then reports it at error severity with a
rule name attached.

## On `--intent`, the brief reports drift; it does not merge

Merging there would make the graded document differ from the file on disk, so
every violation would cite a claim its reader cannot find.

```
design brief DRIFT: 6 claim(s) the brief declares that this intent does not
carry. The intent is what is graded -- re-run --emit-intent to fold them in:
  - USB1.edge: brief says 'east', the intent does not declare it
  - USB1.overhang_mm: brief says {'min': 0.0, 'max': 0.65}, the intent says
    {'max': 2.0, 'min': 0.0}
  - keepout 'usb-shell-shadow': declared by the brief, absent from the intent
```

## What it deliberately cannot say

Four top-level keys are refused **by name, with the reason**, rather than as
merely unknown — an author who wrote one believes it is being honoured:

- **`envelope` / `outline`** — the board outline is READ from the board, never
  authored. A part outside it is a finding about the *part*. Nothing in this
  toolchain writes `Edge.Cuts`.
- **`board_size`** — the same fact one step earlier: board size is a mechanical
  decision this toolchain does not make.
- **`height`** — nothing in the placement stack measures z. There is no height
  in `legality.GradedPart` and none in the parser, so a declared limit would
  grade **nothing**, which is worse than not declaring it: it is exactly the
  "constraint the author believes they set and the grader never checks" failure
  the strict key sets exist to prevent.

Two more are refused inside a `proximity[]` row, for the same reason — each is
a spelling an author reaches for first, so the message carries the correction
rather than only the refusal:

- **`min_mm`** — a *minimum* separation is the clearance channel's claim
  (`legality`, and `check_drc` pad-to-pad), in a different currency and behind
  a different gate. A second minimum here would let one board pass one and fail
  the other with no rule for which wins. The key is `max_mm`: how far apart
  these parts may be, not how close.
- **`max_distance_mm`** — that is the `decaps` spelling, and it means cap
  *centroid* to the IC pad-bbox inflated 0.5 mm, clamped to 0 inside. Two
  spellings for two currencies, so a reader cannot mistake one number for the
  other.

A **duplicate `ref`** is refused too. Two rows for one part is two claims with
no rule for which wins, and the three consumers disagree about it in three
different ways: the grade charges *both*, the repair dispatch keeps the *last*,
and the seeder's even distribution shifts *every other connector on that edge*.
`blocks` already refuses a duplicate name for the same reason.

## `fixed[]` is carried, never asserted, and never `must_lock`

#711 asks for fixed poses to become `place_fixed` ops plus `must_lock`. Neither
is available here, and both refusals are deliberate:

- **`place_fixed`** and the rest of that plan-op vocabulary are named only in
  comments in `py_tools/board_brief.py`. There is no plan-op implementation in
  this repository; those ops live in a downstream consumer.
- **`must_lock`** is refused on measured evidence. `emit_intent` writes it empty
  and its own comment records why: filling it made `place_seed --repair` treat
  those refs as seeder-owned and **lift the user's locks** — "unlock exactly the
  parts the user locked, and move them", measured on two run-7 boards. It also
  buys nothing, because `resolve_intent_gate` already freezes every edge claim
  inside the quench, so a brief-declared connector is already pinned where it
  matters.

So `fixed[]` reaches `context.brief.fixed`, where a reader and a later tool can
see it, and no engine acts on it.

## Deferred, and named rather than silently absent

Height limits as geometry, thermal, panel/depanel, deriving a cable-shadow
keep-out from `cable_entry`, asserting a fixed coordinate, subsystem
decomposition, a side policy, a mounting datum as a coordinate frame, and
HARD/SOFT as a severity axis. Each is expressible today only as an `unknown[]`
entry or a `context` note. HARD/SOFT in particular needs a precedence decision
against the intent's existing per-rule `severity` and `connector_affinity`'s
forced WARN, and that is a decision to take deliberately rather than in passing.

`proximity[]` came off this list in #902, but only its part-to-part form. Its
unresolvable relatives stay deferred and are named here rather than discovered
later: proximity to a **net's** centroid rather than to a part, proximity to a
board FEATURE (an edge, a mounting hole, a keep-out) rather than to a part, and
a per-side or per-layer qualifier. Each needs an anchor the current rule has no
way to resolve.

## The sibling travels with the board

`.design-brief.json` is in `copy_board.SIBLING_EXTS`, and **every** other site
that copies a board's siblings imports that one list. There were nine
independent hand-written copies of it before this change — **four** of them the
narrower `('.kicad_pro', '.kicad_dru')` form (`ai_gui.py`, `plane_score.py`,
`check_join.py`, `placement_run.py`) — and the one the placement CLIs actually
go through, `placement/portfolio.copy_siblings`, was not the one anybody would
think to edit. `tests/test_711_sibling_lists.py` refuses a tenth.

Stranding it does not fail: it silently reverts the next step to inferring what
the sibling declared, which is the whole class of bug `.kicad_pro` and
`.kicad_dru` already taught this repo about.
