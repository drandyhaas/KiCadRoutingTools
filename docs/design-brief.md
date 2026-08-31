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

Everything a brief declares becomes an ordinary `edge_connectors[]` or
`keepouts[]` entry that the existing rules already grade and the existing seat
search already honours. The compiler adds **no** intent key: provenance rides
in `source: "brief"` and `context`, both of which
[the intent schema](floorplan-intent.md) already accepts.

| brief | becomes | provenance |
|---|---|---|
| `interfaces[]` with an `edge` | an `edge_connectors[]` entry — `edge`, `overhang_mm`, `center_on_edge` / `along_edge_band`, and `class: "edge_receptacle"` when `user_facing` is true | `source: "brief"` |
| `interfaces[]` with `edge: "unknown"` | an entry with **no** `edge` key | `source: "brief"`, a `note` naming the unknown |
| `keepouts[]` | `keepouts[]`, verbatim; `kind` and `why` move into `context` | `context.source` |
| `fixed[]` | `context.brief.fixed` — carried, never asserted (see below) | — |
| `product`, `unknown[]`, `mount_mode`, `cable_entry` | `context` | — |

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

  "fixed":   [ { "ref": "MH1", "why": "M2.5 enclosure boss" } ],
  "unknown": ["mounting_datum", "panel"]
}
```

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

Three keys are refused **by name, with the reason**, rather than as merely
unknown — an author who wrote one believes it is being honoured:

- **`envelope` / `outline`** — the board outline is READ from the board, never
  authored. A part outside it is a finding about the *part*. Nothing in this
  toolchain writes `Edge.Cuts`.
- **`height`** — nothing in the placement stack measures z. There is no height
  in `legality.GradedPart` and none in the parser, so a declared limit would
  grade **nothing**, which is worse than not declaring it: it is exactly the
  "constraint the author believes they set and the grader never checks" failure
  the strict key sets exist to prevent.

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
