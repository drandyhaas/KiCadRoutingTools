# The design brief: what the board cannot know

Placement in this toolchain infers everything about a board from the board.
`emit_intent` says so in its own docstring — *"a starter intent READ OFF the
board … the emitted intent grades CLEAN by construction"* — and every
connector's edge is **guessed** from its current pose by `_nearest_edge`, which
is the only source of an edge anywhere in the toolchain. On a pile not even
that: no unlocked part's pose is read, so the emitted intent claims no edge
for it at all (`context.pose_claims_withheld`, #1103).

A board file cannot say which end of a stick the USB plug is, which header a
ribbon mates with, or where the enclosure rib runs. Those are facts about the
product, and the board file has nowhere to put them (`board_brief.py
--requirements` takes prose and carries it verbatim; no tool reads it).

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
`--require-brief` exits 4 when none was found, or when the one found declares
nothing gradable — the parallel of `--require-rules` one level up.
`--require-brief-coverage` exits 4 unless every clause the brief declares
reached a verdict: `--require-rules` counts RULES, and a grade can run six of
them and measure no clause its brief declared. A clause declared `"unknown"`,
and one this toolchain carries by design, never blocks. `board_brief.py`
takes the same `--brief` / `--no-brief`.

## It is a compiler, not a second constraint system

Everything a brief declares becomes an intent entry that the existing rules
already grade, and that `place_simple` reads wherever it reads the intent
("Which rules the placer can see" in [floorplan-intent.md](floorplan-intent.md))
— ordinarily an `edge_connectors[]` or a `keepouts[]` one. Provenance rides in
`source: "brief"` and `context`, both of which
[the intent schema](floorplan-intent.md) already accepts.

| brief | becomes | provenance |
|---|---|---|
| `interfaces[]` with an `edge` | an `edge_connectors[]` entry — `edge`, `overhang_mm`, `center_on_edge` / `along_edge_band`, `mating_face`, and `class: "edge_receptacle"` when `user_facing` is true | `source: "brief"` |
| `interfaces[]` with `edge: "unknown"` | an entry with **no** `edge` key | `source: "brief"`, a `note` naming the unknown |
| `keepouts[]` | `keepouts[]`, verbatim; `kind` and `why` move into `context` | `context.source` |
| `proximity[]` | `proximity[]`, one row per member of a list `ref`; `why` and `requirement` move into `context` | `source: "brief"`, `context.proximity_note` |
| `fixed[]` | `context.brief.fixed` — carried; a row with a `pose` also compiles to `fixed_poses[]` (see below) | `basis: "declared"` |
| `arrays[]` | `arrays[]`, one intent row per brief row, validated by the intent's own loader; `requirement` moves into `context` | `source: "brief"`, `context.brief_row` |
| `mount_mode`, `cable_entry`, `cable_envelope_mm`, `product.user_top_side` | compiled by `connector_consequences` into `max_setback_mm`, `side` and cable `keepouts[]` -- see [below](#connector-declarations-compile-to-clauses) | `context.basis`, `context.compiled_from` |
| `assembly.body_edge_mm` | `assembly.body_edge_mm`, merged into the emitted `assembly` block; `why` and `requirement` move into `assembly.context.body_edge_mm`. Graded by check_assembly's `body_edge_band`, not by a floorplan rule (see [below](#assemblybody_edge_mm-the-assemblers-edge-rule)) | `context.basis`: `declared`, `min_reader` 9 |
| `keep_away` | `keep_away[]`, one intent row per rule, in its canonical spelling; `why` and `requirement` move into `context`. Graded by `keep_away`, and read by the routing engines too (see [below](#keep_away-two-net-groups-apart)) | `source: "brief"`, `context.keep_away_note`, `min_reader` 10 |
| `product`, `unknown[]` | `context` | — |

### The compiler declares nothing that nothing grades

Every key the brief compiles is graded: by a floorplan rule, or, for
`assembly.body_edge_mm`, by check_assembly. Where no existing intent entry
meant what the brief says, the key arrived with its own rule: `proximity[]`
(`proximity`), `keep_away[]` (`keep_away`), `arrays[]` (`array_formation`)
and `fixed_poses[]` (an anchor graded as `zone_containment`). `proximity[]`
is the plainest case, because no other entry means *these two named parts,
this far apart*:

- `decaps.max_distance_mm` is **one board-wide budget over a derived
  population**, not a claim about a named pair. A decap is a `C*` part
  bridging ground (or a stacked domain) and a chip's supply rail, so an
  inductor, a crystal or an ESD array is never its subject, and its partner
  is elected by the grader and measured to a pad box, not to a named pin.
- A **keep-out is an exclusion, not an attraction.** Compiling "beside" into
  "not inside" grades the opposite claim.

A board that carries no brief costs nothing: discovery returns the empty value.
`--no-brief` on a board that *does* carry one gives exactly the document the
board alone gives.

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
      "mating_face": "+y",                   // +x|-x|+y|-y in the footprint's own frame, or "unknown":
                                             // the side it opens on, for a body that names none
      "mount_mode": "edge_mount",            // compiled -- see below
      "cable_entry": "in_plane",             // compiled -- see below
      "cable_envelope_mm": { "depth": 2.0 }, // or "unknown"; NO default
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
not the intent's `assembly.sides`, which is which faces the **fab
populates**. The two are independent and cannot contradict each other: a
back-populated board whose front carries the label is `assembly.sides: "B"`
with `user_top_side: "F"`, and that is coherent. With a perpendicular-cable
connector, `user_top_side` compiles to the face that cable plugs into, graded as `edge_connector_side` at a fixed WARN, and
applied by `place_simple` before it places (see below); on its own it is
carried and graded by nothing, and the brief report says so under
`not_graded`. `assembly.sides` is graded by `rule_assembly_side`
and charged by
`options.grow_board`, which also charges a through-hole part's leads against the face it is *not* mounted on, so the
busier face is the busier **obstructed** one.

Every key is optional. Strictness is the same as the intent's, at every level
and for the same reason: a typo'd key that loads clean is a constraint
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
already correct everywhere: the edge rule skips its wrong-edge conjunct for
an edge-less entry, the placer reads no edge (only an entry's `side`), and
`emit_intent` refuses to name an edge for an implausible pose. "I do not
know" reaches machinery that knows what to do with it.

When no brief is found the tool says so, and says what is filling the gap:

```
design brief: none beside this board (looked for board.design-brief.json).
Every `edge` below is INFERRED from the part's current pose by _nearest_edge;
nothing here declares design intent.
```

`JSON_SUMMARY` carries `brief`, `brief_declared`, `brief_unknown`,
`brief_unknown_keys`, `brief_absent` and `brief_drift`, so all of this is
machine-visible rather than prose.

## `proximity[]`: the constraint the netlist implies and nothing measures

A 3 mm crystal loop and a 30 mm one have **identical connectivity**. Every
instrument in this toolchain reads the board, and the board cannot tell them
apart, so "Y1 beside U1, as short as possible" is ungradable without this key.
One row is one claim is one limit:

| key | | |
|---|---|---|
| `ref` | required | a reference, or a **list** of them |
| `near` | required | the part the subject(s) must stay close to |
| `max_mm` | required | a positive distance, **or `"unknown"`** |
| `basis` | optional, default `pad_edge` | `pad_edge` or `body` |
| `pads` | optional, or `"unknown"` | `{"REF": ["1", "2"]}` — pad numbers as **strings** |
| `requirement`, `why`, `note`, `context` | optional | carried into the compiled entry's `context` |

Each compiled row is graded by `proximity` and priced by `place_simple`: the
subject is pulled toward its partner's pads on shared nets, and each mm past
`max_mm` costs ("Which rules the placer can see" in
[floorplan-intent.md](floorplan-intent.md)). `--emit-intent` also claims each
passive crystal at its IC's oscillator pins, and its load caps at the
crystal, at derived default limits graded as warnings; declare the row here
when the loop must be held to a tighter number or graded as an error, and the
brief's row replaces the derived one.

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
measures the drawn bodies through the one body model (`placement.body`), and
it exists because two parts a brief wants "together" may share no
net at all — an auto-reset transistor pair has no pad pair to measure.

`basis: "courtyard"` is refused: `placement.body` is a **ladder** — courtyard,
then fab, then silk ∪ pads, then the pad bbox — and it answers with whichever
rung the library drew. Many boards' footprints draw no courtyard at all, and a
claim spelled `courtyard` would grade nothing there. The rung that actually
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
no rule for which wins, and the grade would charge *both*. `blocks` already
refuses a duplicate name for the same reason.

## `fixed[]` is carried, a `pose` compiles, and it is never `must_lock`

A `fixed[]` row may carry the exact pose the part is to be placed at:

```jsonc
"fixed": [ { "ref": "H1", "why": "M3 enclosure standoff",
             "pose": { "x": 103.5, "y": 61.0, "rot": 0, "side": "F" } } ]
```

`x` and `y` are required (a pose with no position is not a pose -- drop `pose`
to carry the part as fixed without one); `rot` and `side` may be `"unknown"`,
which is reported as a declared unknown. The row compiles to an intent
`fixed_poses[]` entry with `basis: "declared"` and `min_reader` 7, which the
grade anchors exactly as it anchors a `mechanical.json` pose (the
`fixed_poses[]` section of [floorplan-intent.md](floorplan-intent.md)) and
`place_simple` SEATS: written exactly at that pose before anything is placed,
never searched or nudged, and held there for the whole anneal. Its
clause id is `fixed[REF].pose`, graded by that anchor. A ref with a pose that
is also a declared `interfaces[]` row is refused by name: the interface
compiles to an edge connector graded at its band, and a pose fixes it
exactly. A row with a pose may also carry `accept_courtyard_overlap: [refs]`,
passed through to the intent entry as written -- a named waiver of
COURTYARD overlap only, with a required `why`, for a human pose that overlaps a
part it is designed to (see the fixed-poses section of
[floorplan-intent.md](floorplan-intent.md)); on a row with no pose it is
refused. A pose never becomes `must_lock`; merging over an emitted intent drops
an edge entry the EMITTER inferred for the same ref, and says so under
`context.brief.contradictions`.

**`must_lock`** is refused here. A lock is the board's own `(locked yes)`,
which every placement tool keeps, and `emit_intent` writes the key empty: a
lock list in the intent would be a second, disagreeing record of it. A
connector whose pose is a fact is declared with a `pose`, which compiles to a
`fixed_poses[]` entry that `place_simple` seats and holds.
`board_brief` used to recommend `place_fixed`, a plan op that does not exist
in this repository; it now names the tools that do (#1353): an intent
`fixed_poses[]` row (or a brief `fixed[]` row's `pose`), or `place_pose set`
and `lock`.

So every `fixed[]` row reaches `context.brief.fixed`, where a reader and a
later tool can see it; only a `pose` reaches an engine, through `fixed_poses[]`.

## `arrays[]`: parts that form one row

```jsonc
"arrays": [ { "name": "rn_bus_a", "members": ["RN1", "RN2", "RN3", "RN4"],
              "serves": "U30", "order": "pin", "rotation": "shared",
              "why": "series terminations on U30's data bus" } ]
```

The same row the intent's `arrays[]` carries, plus the prose slot
`requirement` -- validated by the INTENT's own loader, so a row the brief
accepts is a row the compiled intent loads. It compiles 1:1, with `min_reader`
7, and is graded by `array_formation`. No placement tool reads it: to keep a
row's parts together while placing, put them in one group of the groups
file. Which rows to declare is the
author's decision: `check_floorplan --suggest-arrays` lists candidates with
their evidence, and the ones it `declined` with why. `serves` and `order` may be
`"unknown"` (the author looked and does not know), `rotation` too; each is
reported as a declared unknown, apart from a key nobody wrote. Clause ids are
`arrays[NAME].members` / `.serves` / `.order` / `.rotation` / `.pitch_mm` /
`.axis`, and `--intent` drift compares every field at its effective value. A
member that is also a declared interface or a fixed pose is refused by name.
Merged over an emitted intent, a row drops any edge entry the emitter inferred
for a member, and says so.

## Connector declarations compile to clauses

`mount_mode`, `cable_entry` and `user_top_side` compile, in
`design_brief.connector_consequences`. Every tool that reads a brief against a
board calls it through `design_brief.compile_with_consequences` --
`check_floorplan` on both its emit and grade paths, `--plan-only`, and
`board_brief` -- so the grade and drift see exactly what the emit wrote.

The mapping is calibrated on five as-built boards (esp_prog, tigard,
splitflap_driver, glasgow_revC, ulx3s -- their briefs are
`tests/fixtures/959/asbuilt/`), none of which gains an ERROR from it:

| declaration | compiles to | basis |
|---|---|---|
| `mount_mode: edge_mount` | `max_setback_mm` 0.75, on the drawn body (the courtyard when the library drew no fab or silk outline) | `derived_default` -- two shipping edge-mount bodies sit past the receptacle seat's 0.5 mm: tigard J7 at 0.60, rp2350 J3 at 0.614. As built the seat binds on neither, because each courtyard reaches the edge and the seat is asked only of a part whose courtyard sits a margin inside; 0.75 admits those bodies wherever it does bind |
| `mount_mode: through_edge` | the same `max_setback_mm` 0.75: the part reaches the edge, read on the drawn body when the entry is an edge receptacle (the class `user_facing`, the emitter's classifier or a declared edge assigns) and on the courtyard otherwise. How far PAST it is `overhang_mm`, declared or emitted, never derived -- an overhang floor of 0 is vacuous (a body inside the board reads 0 overhang) | `derived_default` |
| `mount_mode: top_mount` / `bottom_mount` | an exemption, reported `carried`: NOT held to the edge-receptacle seat; its declared edge and overhang still grade it, and nothing measures the mount itself | declared -- without it, `user_facing` would make vertical headers edge receptacles and fail them |
| `cable_entry: perpendicular_top` / `_bottom`, with `product.user_top_side` | `side`: the face the cable plugs into, graded as `edge_connector_side` | declared; a fixed **WARN** an intent cannot raise; `place_simple` applies the face before it places |
| `cable_entry: in_plane` | reported `carried`: the declared `edge` is already a clause and in-plane adds none; with `edge: "unknown"` it is reported unmeasured | declared |
| `cable_entry: none` | nothing: carried, and listed under `not_graded` | declared |
| `cable_envelope_mm: {depth}` (in-plane) or `{clear}` (perpendicular) | a `cable:<REF>` keep-out -- a band `depth` in from the edge across the body, or the body plus `clear` on the cable's face -- ONLY for a FILE-locked part (off an unlocked one it would move with every seed), and for an in-plane band only when the body reaches within `depth` of its declared edge. A `cable:<REF>` keep-out the brief declares itself wins | declared |

`user_facing` compiles no face. That the user reaches a part says nothing
about which face it sits on, and reading it as the viewing face would put
B-side connectors on shipping boards (a DSUB-9, a JST-SH, a microSD) on the
wrong one.

**There is no default cable envelope.** No clearance passes every as-built
control, and a fixed in-plane band lies inside some connectors' own bodies. An envelope that is `"unknown"` or absent is reported
`unmeasured`, naming the dimension. An envelope on an interface whose
`cable_entry` is absent, `none` or `"unknown"` is refused at load: it
describes a cable nothing says the part has. z-height and insertion travel are never
measured: a keep-out is a 2D projection, and the row says so.

Each consequence is a row in the check's `consequences` list and in the
declaration ledger (`derived:<clause id>`), with its basis. `JSON_SUMMARY`
lists `derived_default_clauses` apart, so a default is never read as a
validated claim. A value the brief declares itself always wins over a derived
one, and drift is attributed to the declaration a key came from: the setback
to `mount_mode`, the face to `cable_entry`. A compiled key the brief no
longer produces also drifts, and so does a derived keep-out whose envelope the
brief no longer states. A `side` needs `min_reader` 6.

In the ledger, a clause about a keep-out is judged by the keep-out's NAME (a
keep-out finding names the part that intrudes, never the connector), a fired
`edge_connector_side` WARN reads `graded_warn`, a withheld envelope reads
`abstained` and keeps coverage incomplete, and a `derived_default` number
carries authority `assumption`: the declaration is the author's, the number is
this code's.

## `assembly.body_edge_mm`: the assembler's edge rule

```jsonc
"assembly": {"body_edge_mm": 2.5,
             "why": "JLCPCB assembly, no edge rails"}
```

This is the least distance a component body may sit from the board edge. The
rule belongs to the assembler, and the board file has nowhere to put it. The
value is mm (> 0) or `"unknown"`. An `assembly` block with no `body_edge_mm`
is refused, since it would constrain nothing. `sides` is not declarable here,
because the emitter observes it off the board.

It compiles to the intent's `assembly.body_edge_mm`, beside the observed
`sides`, with `min_reader` 9. That arms check_assembly's `body_edge_band`
gate, and render_placement's `a_body_edge_band` under `--gate`. With no
declaration, both report against JLCPCB's 2.5 mm, labelled `derived_default`,
and never gate. No floorplan rule grades it, so `check_floorplan` lists the
clause as carried and names check_assembly as the grader. On the `--intent`
path, an intent without the value drifts. A connector whose interface
declares an `edge` is listed apart from the band rather than counted, like an
edge-class part.

## `keep_away`: two net groups apart

```jsonc
"keep_away": [
  {"rule": "Net-(D5-A):/32K_P,/32K_N:1",
   "why": "32 kHz crystal vs the boost switch node"},
  "class=Digital:class=Audio:0.5"
]
```

Each entry is ONE rule in route.py's `--keep-away` syntax: `AGGRESSOR:VICTIM:
GAP`, with net patterns and `class=` terms, as a string or as an object with
`rule` and the usual prose slots. `"keep_away": "unknown"` says the author
looked and cannot yet say which nets are noisy. A malformed rule is refused
with the rule parser's own message. A string holding two rules, or one rule
twice, is refused as well.

This is the one brief key with **two readers**, and it is one declaration, not
two copies. Both read it through `keep_away.brief_rule_rows`:

- `compile_brief` turns each rule into an intent `keep_away[]` row, and
  check_floorplan grades it on the PLACEMENT. Every aggressor pad is measured
  against every victim pad on another part, pad edge to pad edge. A part pair
  inside the band is a **warn** naming both pads and each pad's side, unless
  the intent's `severity` promotes it. The rule is described in [the floorplan
  intent](floorplan-intent.md).
- The routing engines (`route.py`, `route_diff.py`, the GUI's Route and
  Differential tabs) read the same key off the brief beside the board
  (`keep_away.resolve_keep_away`, in the engine, so there is no flag or GUI
  control) and price it on the tracks, added to any `--keep-away` they are
  given; the run prints `Keep-away: N rule(s) read from the design brief`.
  `route.py` and `route_diff.py` refuse a malformed `keep_away` there before
  routing (exit 2, the parser's message); it is never skipped.
  The brief travels with every routed output, so later steps keep it.

On the `--intent` path a rule the brief declares and the intent lacks drifts.
So does a rule the intent carries from the brief that the brief no longer
declares, because the router has stopped pricing it. Rules are compared in
their canonical spelling, so `1.0` and `1` are the same rule. The reverse
case is a rule the intent carries and the brief beside the board does not:
a row written into the intent by hand, or compiled from a `--brief` naming
another file. check_floorplan grades it, but no router reads it, so it
prints a `WARNING keep_away` naming each such row and the brief to declare
it in. `--json` carries the same as `keep_away_unpriced`.

It is **not** `min_mm` on a `proximity[]` row, and that refusal stands. A
proximity row is about two named parts and how close they must be. A keep-away
is about two net groups and a functional distance between them. The grade
cannot see coupling through planes or between faces (it is plan view, and each
pad's side is named), tracks (the router measures those), or which nets are
noisy (the rule says that).

`board_context --json` **proposes** rules for this key, in `emc_roles.keep_away`.
For each crystal or oscillator it writes one rule that keeps the clock nets
away from every switch node the pins name: a converter's SW or LX pin, or a
discrete switch's drain on an inductor net. Where a part's pins carry no
function, that side is a `candidate` read off the nets: a pin-less buck's
inductor end, or the one undecoupled output of a pin-less oscillator. The
gap is 3 mm, a derived default and not a datasheet value. It is twice
route.py's `--keep-away-free`, so the band reaches past the radius the router
leaves unpriced around each net's own pins. Each proposal carries
check_floorplan's reading of the rule on the board (`measured`, `inside_now`)
and a `brief_row` to paste here. On watchy the rule is
`Net-(D5-A):/32K_P,/32K_N:3`, and the human layout clears it at 15.34 mm. On
ulx3s, `/power/L1,/power/L2,/power/L3:CLK_25MHz:3` is already inside at
1.05 mm, with the oscillator Y1 and the buck U4 on opposite faces. Nothing
reads a proposal until it is pasted.

## Deferred, and named rather than silently absent

Height limits as geometry, thermal, panel/depanel, a cable keep-out WITHOUT a
declared envelope (a declared `cable_envelope_mm` derives one),
subsystem decomposition,
a side POLICY beyond the advisory connector face, a mounting datum as a
coordinate frame, and HARD/SOFT as a severity axis. Each is expressible today only as an `unknown[]`
entry or a `context` note. HARD/SOFT in particular needs a precedence decision
against the intent's existing per-rule `severity` and `connector_affinity`'s
forced WARN, and that is a decision to take deliberately rather than in passing.

Of proximity, only the part-to-part form is declarable. Its relatives stay
deferred and are named here rather than discovered later: proximity to a **net's** centroid rather than to a part, proximity to a
board FEATURE (an edge, a mounting hole, a keep-out) rather than to a part, and
a per-side or per-layer qualifier. Each needs an anchor the current rule has no
way to resolve.

## The sibling travels with the board

`.design-brief.json` is in `copy_board.SIBLING_EXTS`, and **every** other site
that copies a board's siblings imports that one list.

Stranding it does not fail: it silently reverts the next step to inferring what
the sibling declared, as a stranded `.kicad_pro` or `.kicad_dru` would.
