---
name: plan-pcb-placement
description: Places a KiCad board by function, and in full mode routes and verifies it. The agent sorts the parts into functional groups, names each switching stage and RF front end, copies its datasheet layout into a template its support parts are held to, place_simple.py anneals one objective over 12 seeds (6 started scattered, 6 with each group gathered), a quick judging route of the 6 its objective rates best keeps the best, and in full mode that board is then routed by /plan-pcb-routing. One independent verifier grades the result, and the board comes back with REPORT.md. Modes are place and full (place + route). Use it to place, arrange, re-place or optimise components, handle an unplaced board or parts piled off the outline, place and route from scratch, or run the GUI's Place / Place + Route. It never changes the outline. To route a placed board without moving parts, use plan-pcb-routing; for sign-off QA, review-routed-board.
---

# Plan PCB placement

A human lays a board out by function: the USB connector with its resistors
right behind it, the regulator and its caps between the power entry and the
loads, the MCU with its crystal on its oscillator pins, the LEDs and the
button along an edge. This skill places that way. You decide what belongs
together; `py_placer/place_simple.py` arranges it over several seeds; a fast
route of each keeps the best.

Invocation: `/plan-pcb-placement <mode> <board.kicad_pcb> [intent.json]
[--scattered N] [--clustered N] [--anchored N] [--judge N]`,
where mode is `place` or `full`. With no mode, use `full`. A given
`intent.json` is the intent: use it in place of emitting one (§3.2).
`--scattered`, `--clustered`, `--anchored` and `--judge` are the seeds of
each start and how many are judged (§3.6), passed to `place_simple` as
given; its defaults (6, 6, 0, 6) apply when they are not. Anchored seeds
(each group started round the point its wiring prefers) are opt-in.

## 1. The goal, per mode

| mode | input | DONE means | better, once DONE |
|---|---|---|---|
| `place` | an unplaced or placed board | `check_assembly` buildable, `check_floorplan --intent` 0 errors, no pad, graphic or keep-out copper off the outline, mechanical facts met, and a fast route of it with no blocking net | fewer vias, then less copper_mm, then fewer segments, on that fast route |
| `full` | an unplaced or placed board | `board_score` blocking 0, zero unrouted, zero broken, `check_complete` DONE | fewer vias, then less copper_mm, then fewer segments |

- **"Better" is lexicographic, never a weighted sum.** A lower via count never
  buys back an open net.
- **Below DONE, open connections outrank vias and copper.** Between two
  boards of equal `blocking`, the one whose `joins_needed` (in `board_score`
  and `quick_score` JSON: the joins its open nets still need) is lower is
  better, whatever its vias or copper: `unrouted` counts a net as 1 whatever
  its size, and a board missing a 17-pad rail has fewer vias because the
  rail is missing. Vias, copper_mm and segments break a tie only after that.
- **In `place` mode the fast route is your own check.** `scripts/grade.py
  --mode place` and the verifier grade the placement, not a route; keep the
  fast-routed board and its score JSON (`quick_score`) as the evidence for
  the "no blocking net" condition.
- **The routed board judges a placement**, not the objective's cost, crossings
  or hpwl. Those only rank candidates you have not routed yet.
- **DONE is measured with every spec the board declares.** Pass
  `--net-min-widths`, `--impedance-nets`, `--length-groups` (a FILE) and
  `--min-track-width` / `--min-via-diameter` / `--min-via-drill` to
  `board_score` and `check_complete`. An `ungraded` component is UNEXAMINED,
  not passed. `--impedance-nets` grades reference-plane continuity: a net
  that crosses a gap in its reference plane (a track on that plane cuts
  one) is blocking.
- **Matching the original layout does not count.** A human layout is a
  benchmark to approach, not a pose to match.

## 2. Before the first command

- **Work in `wk/<run>/`.** Copy every board with
  `python3 -X utf8 py_router/copy_board.py <src> <dst>`, never a bare `cp`:
  the `.kicad_pro` beside a board carries its DRC floor.
- **Keep the seeds in `wk/<run>/`**, with these names, which the GUI's
  candidates table reads: `s<S>.kicad_pcb` and `s<S>.json` (seed S's
  placement and its `place_simple` report), `s<S>_route/` (the fast route's
  working files), `s<S>_fast.kicad_pcb` (the fast-routed board, copied out of
  that folder with `copy_board.py`) and `s<S>_fast.score.json` (its
  `quick_score`). The kept board goes to `wk/<run>/final.kicad_pcb`.
- **Read the board.**
  `python3 -X utf8 py_tools/board_brief.py <board> --json wk/<run>/brief.json`
  (read `pile` and `has_copper` on its `JSON_SUMMARY` line; `poured_nets`
  names the nets a pour already serves) and
  `python3 -X utf8 py_tools/board_context.py <board> --md` (every part's value,
  pin functions, nets by face, what each cap serves, `CROSSED` pin-order pairs,
  and, from the project's schematic, each sheet's drawn blocks with the notes
  beside them and the designer's layout notes). It finds the schematic beside
  the board (`<board>.kicad_sch`); if its `## Schematic` section says none was
  found and the project has one elsewhere, run it again with
  `--schematic <root .kicad_sch>`.
- **Read `references/part-roles.md` before you judge what a part is.** A
  prefix, a footprint name or pins named IN/OUT mislead: an RF amplifier
  reads as a regulator, a crystal's load cap as a decap.
- **Read `references/placement-lessons.md` before you group and hint.** It is
  the one file of placement lessons: what 437 human boards do (where each
  functional group sits, spacing, decaps, regulator stages, connectors,
  clocks, rows, repeated channels), how a mistake shows on a routed board,
  and what to change.
- **Declared facts win.** If the board has a sibling
  `<board>.design-brief.json` or `mechanical.json`, the intent is compiled
  from it (next section). If the user states a mechanical fact (a
  connector's edge, a mounting-hole position), it is a requirement, and the
  verifier checks it.
- **Ask for the EMC rows the board cannot state.** Run
  `python3 -X utf8 py_tools/board_context.py <board> --json -o <ctx.json>`
  and read `emc_roles.roles`: each converter, crystal, oscillator and ESD
  array its pin functions name, each discrete ESD/TVS diode at the one
  connector its line reaches (`esd diode`), every leg measured pad edge to
  pad edge, and a ready `proximity` row with `max_mm` left null. Ask the
  user for the limits (crystal to its load pins, input cap to the
  regulator's VIN and PGND, inductor to SW, ESD array or diode to its
  connector, and each row of the schematic's layout notes, §3.3), put the
  rows in the brief's or the intent's `proximity`, and report which you
  were given. Ask
  in the same breath about `emc_roles.keep_away`: per crystal or oscillator,
  a rule keeping its clock nets away from every switch node, with its
  measured gap and a `brief_row` ready to paste into the brief's
  `keep_away` list. Which nets are noisy is the user's to say; do not infer
  it from net names. A converter you give a datasheet template (§3.5) needs
  only the `keep_away` rows: the template places its support parts, so do
  not ask for their limits.
- **Nothing is sacred except a KiCad lock.** `place_simple` re-places every
  part the board does not lock (`(locked yes)`) or the intent does not seat
  (`fixed_poses[]`). For a from-scratch experiment with everything off the
  board, build the input with
  `python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/make_unplaced.py <src> <dst>`.
  It refuses a routed input: strip that first with
  `python3 -X utf8 tests/stress/strip_copper_only.py <src> <dst>`.
- **Git Bash:** `export MSYS2_ARG_CONV_EXCL='*'` before any command that
  carries a net name. Quote `--nets '*'`.

## 3. The workflow

### 3.1 Place and lock the fixed parts

Connectors at the edge, module headers, mounting holes and fiducials are
mechanical decisions, not placement results. Place each with
`python3 -X utf8 py_placer/place_pose.py <in> <out> set REF X Y --rot DEG lock REF`.
Take the pose from the design brief or `mechanical.json` when the board has
one, from the user when they state it, and from the input board only when it
is a believable layout (not a pile). A pose the intent declares in
`fixed_poses[]` needs no lock: `place_simple` seats it there and holds it.
Leave declared mounting holes and fiducials unlocked too: a KiCad lock voids
the marker waiver on their courtyard pairs, so two locked markers that
overlap gate `check_assembly`. Declared poses that overlap each other by
courtyard (a fiducial in a mounting hole's courtyard on the human board) are
written by `--emit-intent` as `fixed_poses[].accept_courtyard_overlap`, with
a WARNING naming each pair; `check_assembly` and the plan check waive exactly
those pairs, courtyard only, whatever the lock. A pad or hole conflict
between two declared poses is named in a WARNING of its own: no waiver
covers it, so move one pose. Any other deliberate overlap is accepted by
naming the pair in the intent's `overlap_waivers[]` (with a `reason`), which
excuses its drawn bodies too.
A connector whose EDGE is decided but not its place along it needs no lock
either: leave it unlocked with its `edge` in the intent's
`edge_connectors[]` entry (and an `along_edge_band` when only part of the
edge will do), and `place_simple` holds it to that edge, mating face out,
and slides it along. On a pile the emitter writes no `edge` -- the staging
pose says nothing; `--emit-intent --declare-classes` writes each edge part's
entry without one, plain `--emit-intent` no entry -- so write it from the
brief, `mechanical.json` or the user.

### 3.2 Emit the intent

```bash
python3 -X utf8 py_tools/check_floorplan.py wk/<run>/fixed.kicad_pcb --emit-intent wk/<run>/intent.json
```

- When a placed reference of the design exists (a human layout, an earlier
  placement), add `--decaps-from <reference.kicad_pcb>`: it arms the decap
  limits, and say in your report that you did.
- **Never emit the intent from a damaged board.** It records the damage as
  the spec. Emit it from the brief, or edit it down.
- **On a pile** (`pile` in board_brief's summary), give every
  `check_floorplan` call before the placement `--allow-unplaced`: without it
  the tool refuses a board that does not look placed (exit 3, "does not look
  PLACED"). That gate is for tools that refine a placement, not a refusal of
  this run.
- `place_simple` reads the intent's `proximity` claims, keep-outs, zones,
  declared faces, `fixed_poses`, the edges of `edge_connectors[]` and the
  decap limits it grades at error.
- **Waive the locked parts' own overlaps**, right after the intent exists:

  ```bash
  python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/waive_locked_overlaps.py \
      wk/<run>/fixed.kicad_pcb wk/<run>/intent.json
  ```

  Two parts locked at the input's poses whose courtyards or drawn bodies
  overlap are the designer's (two footprints offered for one site, a part
  drawn over another), and neither moves, yet `check_assembly` gates them:
  every seed then reads NOT BUILDABLE on a pair no move can change. The
  script names each such pair in `overlap_waivers[]`, with its reason. A pad
  or hole conflict between two locked parts is printed as a WARNING and not
  waived, because no waiver covers pads or holes: ask the user which part
  to move.

### 3.3 Group the parts (your judgement)

Write `wk/<run>/groups.json` (the format is in `py_placer/placement/grouping.py`):

```json
{"schema": 1,
 "groups": [
   {"name": "usb_in", "role": "interface", "refs": ["J1", "R2", "R3", "R4", "R5"],
    "anchor": "J1", "near": ["power", "core"],
    "why": "USB-C with its CC pull-downs and D+/D- series resistors"},
   {"name": "power", "role": "power", "refs": ["U1", "C1", "C2"], "anchor": "U1",
    "near": ["usb_in"], "why": "the LDO from VBUS with its input and output caps"},
   {"name": "core", "role": "core", "refs": ["U3", "C3", "R1"], "anchor": "U3",
    "why": "the MCU, its VDD decap, the D- pull-up its GPIO switches"}]}
```

- **Every part the board does not lock (and the intent does not seat in
  `fixed_poses[]`) is in exactly one group.** Fixed parts may be listed too:
  a connector in its interface's group anchors it.
- **Group by function:** the power entry and its conditioning, the core and
  its decaps, its clock, an interface with its protection and series parts,
  signal conditioning, the user interface (LEDs with their resistors,
  buttons). Five to eight groups suit a small board. Never group by net count.
- **Start from the schematic when `board_context` read one** (its
  `## Schematic` section). The designer drew the functions there:
  - **Sheets are the first cut.** Take each sheet as a group, put a small
    sheet in the group it serves (a connector sheet with its interface), and
    split a sheet that holds several functions (an MCU sheet with USB and
    LEDs) along its blocks.
  - **Keep each drawn block in one group.** A block is the parts drawn
    together (a regulator with its caps, a crystal with its load caps, an LED
    with its resistor). Split one only for a reason you write in the group's
    `why`, such as a cap drawn beside the wrong chip.
  - **Read the notes.** A note beside a block ("ESP32 3V3 LDO", "Buck
    Converter (12v -> 5v)") says what the block is: use it for the group's
    name and `why`, and for the template candidates (a buck or boost note
    names a switching stage).
  - **A layout note is the designer's own proximity claim** ("C11 close to
    pin45"). Put each row the `### Layout notes` subsection prints into the
    intent's `proximity[]`, with a `why` quoting the note, and ask the user
    for each limit along with the EMC rows (§2). With nobody to ask, use
    2.0 mm for a part named at a pin (a row with `pads`) and 5.0 mm for a
    part named near a part. A note that does not name its subject lists the
    parts drawn nearest it: write its row only when the note, those parts
    and the nets leave no doubt which two parts, and which pin, it means.
    Never write a row marked STALE: the note names a part that is not on
    that pin's net. Name the rows you added, and their limits, in REPORT.md.
- **A cap belongs with the IC whose supply pin it decouples.** A load or RC
  cap belongs with its crystal or its button.
- **Crystals are claimed for you.** `place_simple` claims each passive
  crystal at its IC's oscillator pins (link by link through a series
  resistor on a leg) and its load caps at the crystal, and moves them all
  into the IC's group. The rest of their clock group follows
  only when the crystal is that group's `anchor`; any other member stays in
  the clock group. Every intent `proximity` claim works the same way: the
  claimed part joins its partner's group. The emitted intent carries these
  crystal claims as `proximity[]` rows (`context.source`
  `placement.crystals`, `max_mm` 8.1 and 2.2), graded as warnings; edit a
  row's `max_mm` to loosen or tighten that loop, or drop the `source` to
  make it an error-graded claim. Each limit is priced, not guaranteed: a
  seed can leave a crystal past it, so read the grade's `proximity`
  warnings (the judge ranks only errors).
- **Hints, each a soft term of the objective:** `near` (a list of groups to
  sit against), `apart` (groups to keep 3 mm from), `edge` (`N`/`S`/`E`/`W`,
  a group that belongs on a board side; a group holding an edge connector
  needs none). **Do not send the power group to an edge**: on 437 human boards
  the regulator sits by its power entry -- say `near` its entry's group.
- **Faces:** give a group a `side` (`F`/`B`) only to put all its parts on that
  face; without one each part keeps its face.
- **Name the template candidates** in the group that holds each one, as
  `"templates": [{"chip": "U2", "kind": "switching stage", "why": "buck
  converter; its datasheet's layout figure"}]`. §3.5 copies each one's
  datasheet layout. Judge from what the part IS (its value, and its
  datasheet when the value does not settle it), never from pin names alone.
  Pin names miss many stages, and some chips have none. Two kinds:
  - **`switching stage`**: anything that switches current through an
    inductor, a transformer, a flying or bootstrap cap, or a motor:
    - a buck, boost, inverting or buck-boost converter or controller, with
      its FETs;
    - a flyback with its transformer;
    - a charge pump;
    - a class-D amplifier's output filter;
    - a motor or gate driver's bridge with its bootstrap caps;
    - the built-in DC-DC of an MCU or a radio (an RP2350's VREG_LX, an nRF52's
      DCC, an SX126x's DCC_SW).
  - **`rf front end`**: the parts between a radio or RF chip's RF pins and
    its antenna or RF connector: the matching network, a balun, a filter, an
    RF switch, an LNA or PA. A module that holds its own (a radio module, a
    power module) needs none.
  - **Never** an LDO, a crystal or a plain decap: the decap and crystal rules
    hold those, with no datasheet to read.

  On a big board, find them by their parts. List every inductor,
  transformer, power diode and flying cap, and run `fit_template.py --pins`
  on each to see which chip it reaches. Then list every chip's value for
  radios and RF parts.
  - **One chip, one template.** A chip that is both kinds (an nRF52 or an
    SX126x: a radio with a DC-DC) takes one template holding both stages.
    Name it once, with its main kind, and say both in `why`. A dual converter
    is one template too.
  - **Members go in the chip's group.** `place_simple` moves a template's
    members into its chip's group whatever the file says, and `check_groups`
    warns of any you put elsewhere.
  - **Record the exceptions:** `"skip": "<why>"` on a candidate that will get
    no template (§3.5 step 1), and `{"chip": "U8", "kind": "none", "why":
    "..."}` for a chip a hint points at that is no stage (an ADC whose supply
    has an LC filter).

  `check_groups` lists the candidates and, with `--intent`, which have a
  template row yet. It also prints a HINT for each chip the board's pins and
  nets suggest that no group names:
  - `emc_roles` reads it as a switching converter;
  - a pin's net carries an inductor and at most four other parts;
  - a cap sits across two of its pins, one named as a bootstrap or flying-cap
    pin;
  - a pin is named as an RF port.

  Run it once before you name anything, to see every hint. A hint is
  evidence to weigh, not a verdict, and no hint does not mean no candidate:
  an RF chain without named pins, or a stage of a chip with no pin names,
  shows none.
- **Look parts up.** A part number the board and `part-roles.md` do not
  settle is a datasheet question: WebSearch it and WebFetch the datasheet.
  If you must save a file, save it only under `$KRT_DATASHEET_DIR` when that
  is set (GUI runs), else under `wk/<run>/datasheets/`, and delete that
  folder before you hand back.

```bash
python3 -X utf8 py_tools/check_groups.py wk/<run>/fixed.kicad_pcb --groups wk/<run>/groups.json \
    --intent wk/<run>/intent.json
```

It reads the intent the way `place_simple` will: seated parts are fixed,
and it names each claimed part that will follow its partner into the
partner's group. The groups it lists and checks are the file as written, so
a follower still shows in the group you gave it. Fix every ERROR (a part
in no group or in two, a ref not on the board) and
read every WARN: a part with more signal links to another group may belong
there; a decap "exempt" because no IC of its group carries its rail is right
for a load or RC cap and wrong for a real decap. Read every template HINT
(above).

### 3.4 Orient and check the connectors and fixed parts (your judgement)

```bash
python3 -X utf8 py_tools/check_floorplan.py wk/<run>/fixed.kicad_pcb --intent wk/<run>/intent.json
```

For each edge connector, read its edge, its facing (`facing ... pass`, or
`facing abstained (...)` when its body names no face) and the group you gave
it. Then decide, from the part's function and its datasheet:

- **The edge, and roughly where along it** when that matters (a USB or
  barrel jack where the enclosure opens, a debug header out of the user's
  way): `edge`, and `along_edge_band` as fractions of the edge.
- **The mating face of a connector whose facing abstained** (a side-entry
  JST drawn symmetric about its pads): find the side its opening is on in the
  datasheet's PCB-layout drawing and write `mating_face` (`+x`/`-x`/`+y`/`-y`,
  the footprint's own frame). The side of its solder tabs, away from the
  contact row, is a hint to check against the drawing, not a rule: it is
  wrong on 3 of 50 side-entry connectors in the corpus.
- **The side a vertical header's cable leaves.** A `Vertical` header mates
  off the board and names no face, but its cable leaves on one side: a
  2xN ribbon off a long side. Write that side as its `mating_face`, so the
  seat lays the header with its long side along the edge and the cable runs
  off the board. Without it, a 2x05 header lay across its edge on most
  seeds, its cable along the edge.
- **The pin rows of a plug-in module** (a DIP or breadboard module, rows
  that go into a socket): the socket sets their spacing, so declare each row
  in `fixed_poses[]` (`basis: derived_default`), not on its edge: parallel,
  along the long edges, pins on one 0.1 in grid, centred on the board, a
  whole number of 2.54 mm apart (the socket's, else the most that keeps
  every pad the board's edge clearance inside the outline). Seated flush on
  their edges instead, ch32v003_usb's two 1x10 rows ended 13.21 mm apart,
  not 12.70, and the module no longer fits a breadboard.
- **How far an edge receptacle reaches past the edge.** The seat puts a
  receptacle flush with the low end of its `overhang_mm` band, which the
  emitter writes as 0. Where the datasheet's PCB-layout drawing puts the
  board edge inside the body (a USB-C or barrel jack's shell past the edge),
  write that distance as `overhang_mm.min`, `max` at or above it. Seated
  flush instead, ch32v003_usb's USB-C J1 sat 0.88 mm deeper than its drawing
  and the board routed with about 15 mm more copper a seed. The band is
  read at zero margin, on the drawn body or else the courtyard (#1301): an
  overhang of x reads x, so write the distance the part reaches past the
  edge and never add the board's edge margin to it. check_floorplan's
  `overhang_basis` says which of the two it read.

Write these into the intent's `edge_connectors[]` entries, the rows into
`fixed_poses[]` instead (a ref in both is refused), or into the brief,
then emit again (§3.2). Check the fixed parts -- mounting holes, switches, LEDs
a panel shows -- against the brief or `mechanical.json`, and move any that is
wrong with `place_pose ... set ... lock` as in §3.1. Run check_floorplan again:
no `edge_connector` error, and no `edge_oversubscribed` warning (an edge
declared more connectors than fit on it together: move one to another edge,
drop a declaration, or widen a band now, before placing).

### 3.5 Datasheet layouts (your judgement)

A switching stage's datasheet draws where each of its support parts goes:
the input caps across VIN and ground, the inductor on the switch pin, the boot
cap across BOOT and SW, the feedback divider at FB, away from the switch node.
An RF chip's draws its matching network as a short line from the RF pin
toward the antenna, each shunt part beside it with its own ground. The parts'
values are tuned on that geometry. The router cannot see whether the board
follows the drawing, so the board can route clean and still work badly.
Copy the drawing into the intent as a **template**. `place_simple` seats each
part at its template pose, carries it with every move of the chip, and makes
leaving that pose expensive.

1. **Which chips.** The template candidates your groups file names (§3.3),
   one template each. A chip whose pads carry no pin functions (`--pins`
   shows them blank) has its pins read off the datasheet's pinout.
   - **A figure is best, but guidelines can do.** A datasheet with no layout
     figure whose text names each part's pins still makes a template, with
     no `side`s: "a 10 nF bootstrap cap between pins 1 and 2", "1 uF as
     close as possible to VS (pin 6) and GROUND (pin 7)".
   - **Skip** a candidate when no datasheet or reference design can be found
     for it, when neither a figure nor such text places its parts, or when
     its support parts are locked (`check_groups` lists each group's fixed
     parts). Put the reason in the candidate's `skip` and in REPORT.md. Then
     write a `proximity[]` row (§2) for each part its datasheet says to keep
     close: `emc_roles` writes rows only for chips whose pins it can read as
     a converter's, so a bridge or an unnamed chip would be held by nothing.
2. **Get the drawing.** WebSearch the part number with "datasheet" and take
   the manufacturer's PDF. TI's is `https://www.ti.com/lit/ds/symlink/<part>.pdf`
   and Analog Devices' (Linear too) is
   `https://www.analog.com/media/en/technical-documentation/data-sheets/<part>.pdf`,
   the base part number in lower case with the package and reel suffix
   dropped: LMR36006CRNX is `lmr36006`, LT3471 rev B is `3471fb`. A search
   often returns an HTML viewer or distributors' pages instead.
   - **When the maker's site blocks a download** (a browser check, a
     JavaScript link), take a distributor's or a vendor CDN's copy
     (SparkFun, Adafruit, Mouser), and check it is the maker's file: its
     document number and revision on every page.
   - **Save it** under `$KRT_DATASHEET_DIR` when that is set, else under
     `wk/<run>/datasheets/<part>/`, one folder per part: a download is data,
     not code. Delete the folder before you hand back (§3.3). WebFetch
     mangles a PDF, so download it (`curl -sSL -o <folder>/<part>.pdf <url>`)
     and read it with `py_tools/pdf_text.py` (below), not the Read tool's
     `pages`: that needs poppler, which many machines lack (#1417).

   You need three things from it:
   - the **layout guidelines**. They say what matters most and sometimes
     give distances, but may say neither;
   - the **layout figure** ("Layout Example" or "Recommended Layout", near
     the end);
   - the **application schematic** whose designators that figure uses
     ("Typical Application" or "Design Example"). Without it the figure's
     C1 and Q2 mean nothing. Where no schematic uses the figure's names,
     take one of the same topology and match by the nets the figure draws.

   - **Several readings of one figure.** Where the figure and the schematic
     disagree on how many of a part there are, the figure is the layout to
     copy. A part the schematic calls 2x and the figure draws once: seat
     the second beside the first. A family datasheet may draw a sibling's
     extra parts (an adjustable version's divider), and the board's own
     version decides which exist.
   - **An RF chip or a built-in DC-DC** often has its layout in a separate
     **reference design** or application note (Nordic's reference layouts,
     Semtech's reference design for the SX126x). Take the configuration that
     matches the board: its supply, and which DC-DCs it uses. An application
     note's evaluation-board figure will do when the design files cannot be
     fetched. A reference design's pick-and-place file gives each part's
     centre and turn exactly (Altium's in mils, +y up). Use it to read the
     arrangement, then fit as below; never copy its coordinates.
   - **Finding the pages.** `py_tools/pdf_text.py` uses whatever the
     machine has -- poppler, pypdf, PyMuPDF, or on macOS the system's own
     PDFKit, so it needs no install there -- and names the backend it used:
     - `python3 py_tools/pdf_text.py <pdf> --find "layout example" "typical application"`
       prints `page N: <line>` for each heading (the table of contents
       lists them too);
     - `python3 py_tools/pdf_text.py <pdf> --pages N` prints page N's text;
     - `python3 py_tools/pdf_text.py <pdf> --render N --out <folder>/pN.png --dpi 200`
       renders the figure; Read the PNG, and render at a higher `--dpi`
       (300) and crop when it is too small to read.
     Exit 3 means no backend is present: install poppler or `pip install
     pypdf`. Where poppler is installed, `pdfimages -j` extracts a figure
     the PDF holds as an embedded image, which rendering cannot sharpen.
3. **Match the drawing to the board, by net and pin function, never by
   designator.** The datasheet's C1 is not the board's C1: a bitaxe agent
   found Q1, Q2, R4, R10 and C1 naming different parts in the two, so
   matching by name would have swapped the FETs. Nets first; values only
   break ties, since a board often fits other values than the datasheet's.
   - **`--pins`.**
     `python3 -X utf8 py_tools/fit_template.py wk/<run>/fixed.kicad_pcb --pins <REF>`
     lists a part's pads: number, pin function, position and side in its
     own frame (`centre` for an exposed pad, `inner -x` and so on for an
     inner pad off the middle), net, and the other parts on that net with
     their values and pads (`@B` or `@F` marks a part on the other face). It takes any
     part, so walk a net from part to part. A net of more than 20 parts
     prints as a count; with `--groups wk/<run>/groups.json` it still lists
     that net's parts in the part's own group.
   - **The stage's parts.** CIN is on VIN and ground, CBOOT from BOOT to SW,
     the inductor on SW, the divider on FB. COUT is on the output rail,
     wherever the topology puts it: past a buck's inductor, on a boost's
     diode cathode, past an inverter's second inductor. Where several parts
     qualify (four caps on VIN), take the ones your groups file puts in the
     chip's group; the others serve something else. A chip with two VIN/PGND
     pin pairs takes a cap set at each pair, where the board has them.
   - **Faces.** A member keeps the face the author gave it; a template
     never moves a part between faces. A member on the other face from a
     through-hole chip (a bypass cap on the back between its pins) is
     fitted clear of the chip's leads, under its body where there is room,
     as the seat gate allows. If the whole group belongs on one face
     (`side`, §3.3), fit the template after the faces are settled.
   - **What is not a member.** The chip's other decaps and its crystal: the
     decap and crystal rules hold them. A heatsink or thermal pad under the
     chip: it stays with the group. Bulk capacitance the text places by
     role, not by pin ("at least 100 uF per amp"): it stays with the group
     unless it fits beside the loop.
   - **"The drawing" is the whole layout section**: the figure and the
     guidelines, which may name a part the figure omits (a CFF). Leave out
     what it has and the board does not, and what the board has and it does
     not. Where the board has fewer like parts than the figure draws (two
     COUT for four), take the figure's positions nearest the chip. A drawn
     part that only fits far out and does not matter to the loop (a
     soft-start RC) may be left to the group.

   **A controller with external FETs** (no VIN pin of its own; `emc_roles`
   notes it) takes its FETs as members too. Their input caps go across the
   high-side FET's drain and the low-side FET's source, and the inductor
   on the switch node between the FETs. Those members' legs name the FETs'
   pads (an earlier member), not the controller's pins, and the FETs' legs
   name the gate-drive pins. **A motor bridge**'s bootstrap caps go across
   each BOOTSTRAP and OUTPUT pin pair, and its supply bypass across VS and
   ground; its hot loop is that bypass.

   **An RF front end** is a chain: walk it from the chip's RF pin with
   `--pins` on each part in turn, along the reference schematic.
   - **Its parts.** A series part carries the chain on both pads, a series
     0R or jumper included. A shunt part has one pad on the chain and one on
     ground. A balun or RF switch takes the chain in and passes it on. Where
     the reference uses another part than the board (a different RF switch),
     match by function: which port is which.
   - **Its ends.** The antenna or the RF connector ends the chain. It is
     placed and locked with the edge parts, so it is no member.
   - **A locked part mid-chain** (a test connector) splits it: template each
     side on its own chip (the radio, the switch), and leave the parts that
     land on the locked part to their group.
   - **A locked chip** (the author locked the radio) takes the board's
     locked parts round it into the fit: obstacles where they are, and
     partners a member may name, so the chain can head for the antenna's
     feed pad.
4. **Turn the drawing into the footprint's frame.** The template is in the
   footprint's own frame: rotation 0, +x right, +y down. From `--pins`, see
   where each pin group lies, then map the drawing's directions onto it. For
   example, the LMR36006's Figure 57 draws SW (pin 12) at the top, and
   fusb302_pd_dev's footprint has pad 12 at `inner -x`, so the drawing's up
   is -x there.

   **Check whether it is a turn or a mirror** on pin 1 and the way the
   numbers run from it.
   - Read the numbers off the layout figure itself, since some datasheets
     turn the figure from the pinout page.
   - Plot `--pins`' positions with +y DOWN, as the board draws them.
   - If the numbers run the same way round in the figure and in `--pins`,
     it is a turn. If they run the other way, the drawing is a bottom view
     or the chip is on the back. Mirror it about the axis that keeps pin 1's
     side where `--pins` puts it.
   - A ball grid runs by rows and columns instead: its letters and numbers
     must run the same way in both.
   - A back-side chip's frame is mirrored, so read its sides from `--pins`
     and never from the drawing.
5. **Write the sketch:** the chip as `anchor`, then the members. Seating is
   greedy: each member takes the best spot left, so order decides.
   - **The hot loop first.** A buck's input caps; a boost's diode and output
     cap; an inverter's coupling cap and diode; a controller's FETs; a
     bridge's supply bypass. Then the inductor, then the boot cap and the rest
     at the chip, then the small signal parts.
   - **Every part that sits beyond another member** (a second rank of input
     caps, COUT beside the inductor) after all the parts at the chip.
     Listed earlier, it takes a chip-side part's spot.
   - **An RF chain** goes in its order from the RF pin: each series part's
     partner is the part before it (the chip's pin for the first), each
     shunt part's the series pads on its node. After a shunt the next series
     part names both.
   - **If the fit puts a part where the figure has another**, list the
     figure's occupant first.

   Give each member:
   - `pads`: its own pads, and for each partner (the chip, or a member
     listed earlier) the partner's pads it belongs beside.
     - Pads are matched by net, so their order does not matter.
     - Each of the member's named pads goes to the NEAREST same-net pad
       among the partners named, and the fit minimises the sum over its
       pads. So partner pads on one net are alternatives. Only a partner for
       ANOTHER of the member's pads holds it in place: COUT names the
       inductor's output pad for its VOUT pad and CIN's ground for its
       ground pad, the copper the figure shares.
     - A second part like one already seated (the second input cap) names
       the first.
     - Name a cap's ground pad where the drawing joins it to the chip's
       ground copper: at a PGND pin, or at a member's ground pad there, as
       COUT's joins CIN's. A decap off the power loop usually grounds at
       its own via;
   - `side`: where the drawing puts it, in the footprint frame. `-x`, `+x`,
     `-y` or `+y` means its centre is beyond that edge of the chip's
     courtyard, anywhere along it; two of different axes (`+x-y`) mean a
     corner, beyond both edges. It is relative to the chip only. On a small
     chip a corner costs a long leg; a big inductor beside a small chip
     often needs one;
   - `rot` wherever the drawing fixes the part's axis or which pad faces
     which way:
     - a FET, a diode or a balun;
     - an inductor;
     - a series RF part;
     - any two-pad part whose other pad leaves the template (to a locked
       part, or a pad not named), which the fit would otherwise lay along
       the chip's pins.

     Leave it out for a cap whose two pads both go to the chip. It is the
     part's turn relative to the chip, in degrees (0, 90, 180 or 270),
     counter-clockwise in the footprint frame as KiCad turns parts. On the
     board the part sits at the chip's rotation plus `rot`;
   - `reach` (mm, default 4, `--reach` for all) for a member whose spot
     lies further from its partners;
   - a `note` naming what the drawing calls it.

   Give the sketch the `kind` your groups file names the candidate with
   (`switching stage` or `rf front end`): a switching stage has its caps'
   ground returns checked, by the fit and by the grade. Cite the datasheet
   (title, revision, section, figure) in `source`:
   ```json
   {"anchor": "U2", "kind": "switching stage", "source": "TI LMR36006 SNVSB48C, 11.1 and Figure 57",
    "members": [
     {"ref": "C2", "pads": {"C2": ["1", "2"], "U2": ["2", "1"]}, "side": "+y", "note": "CHF across VIN 2 / PGND 1"},
     {"ref": "C5", "pads": {"C5": ["1", "2"], "U2": ["4", "3"]}, "side": "+y", "note": "CBOOT across BOOT 4 / NC-SW 3"},
     {"ref": "C1", "pads": {"C1": ["1", "2"], "C2": ["1", "2"]}, "side": "+y", "note": "CIN beyond CHF"},
     {"ref": "L1", "pads": {"L1": ["1"], "U2": ["12"]}, "side": "-x", "rot": 180, "note": "inductor on SW 12"},
     {"ref": "R2", "pads": {"R2": ["1"], "U2": ["7"]}, "side": "+x", "note": "RFBB at FB 7"},
     {"ref": "C7", "pads": {"C7": ["1", "2"], "L1": ["2"], "C1": ["2"]}, "side": "-x+y",
      "note": "COUT beside the inductor: VOUT on L1's output, ground by CIN's"}]}
   ```
6. **Fit, look, adjust.**
   ```bash
   python3 -X utf8 py_tools/fit_template.py wk/<run>/fixed.kicad_pcb --sketch <sketch> \
       --render <png> --intent wk/<run>/intent.json --write-intent wk/<run>/intent.json
   ```
   - **What it does.** It seats each member in order, within its reach of
     its partner pads, as close to them as the courtyards already seated
     allow, kept the board's clearance apart (the placer's own rule). It
     tries each quarter turn and keeps the one that brings the pads closest.
   - **What it prints.** Each member's pose, the gap of each of its legs
     (`legs 1.07 + 0.74 mm`), and its gap to the chip. A NOTE flags a member
     that ended beside another edge than its `side`.
   - **For a switching stage, each cap's ground return**: the pad-edge gaps
     from its ground pad to the chip's ground pins (or an external low-side
     FET's), through other members' ground pads, as a surface pour would
     join them. A NOTE flags one over 3 mm (`ground_return_mm` in the sketch
     sets another). It is the check legs cannot make: a cap whose ground
     pad names no partner is seated by its other pad alone, and an output
     cap seated past the inductor read 10 mm here while every leg it was
     given was short. Give that pad a partner and re-fit.
   - **What it writes.** The template goes into the intent, replacing that
     chip's row, so writing over the input and re-fitting as often as you
     like is safe.
   - **If a member cannot be seated**, the fit stops. It says why: its
     `side`, or which members already seated take its spot. `--render`
     then draws the members seated so far. Move the member earlier in the
     order, change its side, make a like part its partner, or raise its
     `reach`.
   - **Look at the PNG** beside the datasheet's figure, turned to the
     footprint's frame.
     - Check that every part sits on its side and faces the pin it serves.
     - Each red leg runs across a pad-edge gap to the nearest partner pad,
       so it should be short. A short leg can still hide a part far from
       the chip, when its partner is a member that strayed, so read the gap
       to the chip too.
     - A leg of several mm usually means the member lost its spot to one
       listed before it. A part much bigger than the drawn one (an IHLP
       inductor beside a 3 mm chip) leaves a long leg that no order fixes.
   - **A part with no courtyard** is fitted, and placed, on an estimated
     courtyard: its .Fab body (else silk, else pads) with its pads, and no
     margin round it (#1300). The run names it, and the render draws
     it dashed. Check that box against the part's real body. A custom pad (a power FET's drain, a
     printed antenna) is boxed round its own copper, even when that copper
     lies to one side of the pad.

   **Give up on the template**, keep the §2 rows and say why in REPORT.md,
   when after adjusting a member's legs still run several mm past what the
   drawing itself shows. Judge against the distances the guidelines give,
   else against the drawing's own legs for the parts it puts first (the hot
   loop, the switch-node inductor, the RF pin's first part). A template
   from text alone has no drawing: judge each leg against the shortest it
   gets when its member is listed first. A member you leave out of a
   template gets a `proximity[]` row instead.
7. **The rest is automatic.** `place_simple` seats and carries the template
   (`--template-weight`, default 100 per mm of drift), and `check_floorplan`
   grades each member's drift from its template pose (`template`, a WARN,
   past 0.3 mm by default, the row's `tolerance_mm`). On the unplaced board
   every member reads off its pose; that is expected. Report each template
   in REPORT.md: the datasheet and figure, the parts matched, and the drift
   graded on the final board.

- **Never copy coordinates from the drawing.** A datasheet packs its parts
  tighter than this board's courtyards allow (on fusb302_pd_dev the 0402s
  cannot come closer than 0.6 mm to the chip's pins); the fit sizes the
  template to the board's own footprints.
- One part belongs to at most one template. A template's members need no
  EMC `proximity[]` rows (§2): the template holds them.

### 3.6 Place

```bash
python3 -X utf8 py_placer/place_simple.py wk/<run>/fixed.kicad_pcb 'wk/<run>/s{seed}.kicad_pcb' \
    --groups wk/<run>/groups.json --intent wk/<run>/intent.json --jobs 4 \
    --json 'wk/<run>/s{seed}.json'
```

- **The seeds run in parallel, then the fast routes.** An OUT carrying
  `{seed}` places the default seeds, one process per seed (`--jobs N` caps how many at once; default one per seed, at most 4),
  each giving exactly the placement a `--seed` run would, its log beside its
  board as `s<S>.log`; one placement process takes about 120 MB. A judging
  route takes up to about 1.3 GB on a small board and 5 GB on a 350-part one:
  run as many at once as memory allows, and on a machine with 2 GB to spare
  one at a time, never beside a placement.

- Every part not locked or seated by the intent starts at a random place and
  is annealed; the same board, inputs and seed give the same placement.
  It places `--scattered` seeds (default 6) that way, then
  `--clustered` seeds (default 6, numbered after them) that start with each
  group's parts gathered round one random point instead: on boards dominated
  by one big IC those route clearly better, on others the scattered ones do,
  and the judge takes whichever routes best.
  The run is 500 moves per movable part by default (`--moves N` sets the
  total), roughly 2 s for 25 parts.
- **Place 12 seeds (6 scattered, 6 clustered); judge the 6 the objective
  rates best.** place_simple ends by naming them: printed, and written to
  `wk/<run>/judge.json` (`judge`, in judging order; a seed that left a part
  illegal is never named). Each seed grades its own floorplan
  (`check_floorplan --intent`, the `floorplan` line of its log and of
  `judge.json`), and the seeds it grades without an error are named first,
  cheapest first, then the fewest errors: a route cannot clear a floorplan
  error (a decap past its pin limit), so such a seed cannot be DONE as
  placed, and the limit it breaks often makes it the cheapest. A WARNING
  names each judged seed that carries one. Seeds differ more than most input changes, and the anneal's own
  cost (`final.cost` in each `s<S>.json`) says which are worth a judging
  route: the seed the judge would pick out of 16 is in the cheaper half
  85-100% of the time. Over 11 boards from 25 to 350 parts (each seed's
  full route, 2000 resampled pools per board), this ships 50 open nets in
  all against 57 for 24 scattered seeds with the 12 cheapest judged, 71
  blocking against 101 and 6426 vias against 6495, in half the placing and
  half the judging; one board of the 11 does worse (3.8 open nets against
  0.6). Judging all 12 instead of the 6 cheapest ships worse
  (60 open nets): the judge is wrong more often with more to choose from.
  Exit 4 if any seed left a part illegal (its `illegal` list is not empty):
  never judge or keep that seed. Exit 4 too when a declared pose is refused:
  it is the intent's, the same in every seed, so it is listed once under
  `refused` (in `judge.json` and each seed's summary) and the seeds are still
  judged; fix the intent (the pose, or an `accept_courtyard_overlap` the
  human board carries -- `--emit-intent` writes one for each pair of
  declared poses whose courtyards overlap) rather than placing again. Exit 4
  too when an edge
  cannot seat all the connectors the intent declares on it together (two
  connectors declared on the south edge where only one fits): listed once
  under `oversubscribed` (the edge, each part's length along it and where
  it can stand, the free span, and which it seated), the most that fit
  seated (the earlier declared on a tie) and the rest placed freely, so the
  seeds are still judged. That is the INPUT's to fix, not a seed's: change
  the intent -- move a connector to another edge, drop a declaration, or
  widen a band -- and place again. Exit 2 if any failed outright.
- A run that leaves a part with no legal pose anneals again from a fresh
  start, up to two more times (`runs` in the `--json` report). Exit 4 names
  parts still left illegal; exit 2 a face or pose the inputs contradict.
  Read the `cost` line: `hints` and `groups` say which group or hint the
  result could not satisfy.
- **Check and LOOK** at each judged seed:
  `python3 -X utf8 py_tools/check_assembly.py <placed> --baseline wk/<run>/fixed.kicad_pcb`
  (read `buildable`),
  `python3 -X utf8 py_tools/check_floorplan.py <placed> --intent wk/<run>/intent.json`
  (0 errors; the seed's `floorplan` line already counts them) and
  `python3 -X utf8 py_tools/render_placement.py <placed> -o <placed>.png`.
  Check the render against `references/placement-lessons.md`: the power
  stage between its entry and its loads, a regulator's caps tight around it,
  parts on the side of the chip where their pins are, connectors' partners
  right behind them. A judged seed that is NOT BUILDABLE, or that carries a
  floorplan error, cannot be DONE as placed: fast-route only the ones that
  are clean (name them to `judge_seeds.py`, e.g. `--seeds 3,4`, §3.7), and
  go on to the next block when none is.
- **When no seed can be judged.** Three cases; in each, what to do depends
  on whether the cause is the same in every seed (below):
  - **`judge.json` names no seed** (`judge` empty; `judge_seeds.py` refuses
    it, exit 2): every seed left a part illegal (`illegal` in judge.json and
    each `s<S>.json`) or failed outright (exit 2, its `s<S>.log`). A
    declared pose that is not legal no longer empties it, nor does an edge
    declared more connectors than it holds: they are listed under `refused`
    and `oversubscribed`, and the seeds are still judged (fix the intent, as
    above).
  - **Every judged seed is NOT BUILDABLE** (`check_assembly`'s `buildable`
    false): read the pairs it names.
  - **Every judged seed fails the floorplan** (place_simple WARNs that it
    judged a seed with floorplan errors; `check_floorplan --intent` names
    each finding, its measured distance and its limit).

  Then:
  1. **The same part or finding in every seed is the input's**, and no seed
     will fix it: the part has no room (a WARNING that it fits the usable
     area at neither quarter turn, or that an edge part has no seat there,
     which names what stopped it: a band its overhang never comes down to,
     the part in the way, or a part seated on the adjacent edge that covers
     its whole line at the corner), an edge declared more connectors than fit on
     it together (`oversubscribed`: the same parts and free span in every
     seed; move a connector to another edge, drop a declaration, or widen a
     band),
     a fixed part or a declared pose walls it in, a keep-out, zone,
     `along_edge_band` or group `side` leaves it nowhere, or a limit no
     placement can meet (a decap pin limit on a rail with one cap far from
     the pin). Change that input -- move or unlock the fixed part
     (`place_pose`), correct the declared pose or the intent row -- and
     place again (§3.6, the default seeds).
  2. **A different part or pair in each seed** is the anneal's miss on a
     crowded board. Take the seed with the fewest illegal parts (in the
     other two cases, the first seed `judge.json` names: fewest floorplan
     errors, then cheapest) and repair it by hand: `place_pose.py <seed> <seed> set REF
     --near X Y` (it snaps to the nearest legal pose; `rotate`, `face` for
     the rest), until `check_assembly` says buildable and `check_floorplan
     --intent` reports 0 errors. Then judge it on its own:
     `judge_seeds.py wk/<run>/judge.json --seeds S --chain ...`.
  - **Whether it is a new pass.** Placing again after changing an input is a
    new pass of §3.6, with every default seed: record the unusable pass in
    the ledger as a `--kind placement --rejected` row (its least-bad seed as
    `--board`) whose `--lever` names the input you changed, and count it as
    one approach toward the stop rule's three (§5, rule 3). A hand repair of a seed is not a new pass: record
    the repaired board as a `--kind placement` row whose `--lever` lists the
    moves, and list them in REPORT.md. A hand-moved part stays where you put
    it only on that board: placing again moves every part the board does not
    lock and the intent does not seat (§2).

### 3.7 Fast-route the judged seeds and keep the best

1. **Write the judging chain** to `wk/<run>/judge_chain.txt`: the board's
   routing chain (the recipe is `plan-pcb-routing`'s: planes, fanout, the
   decap clearance step, `route_diff.py`, `route.py`), one command per line
   (a trailing `\` continues it), each naming its boards by placeholder:
   `{in}` the board the step reads, `{out}` the one it writes, `{dir}` the
   seed's route folder (`--json-out {dir}/route.json`), `{run}` this folder
   (`--intent {run}/intent.json`).
   ```text
   python3 -X utf8 py_router/route_planes.py {in} {out} --nets GND --plane-layers B.Cu
   python3 -X utf8 py_router/route.py {in} {out} --nets '*' --json-out {dir}/route.json
   ```
2. **Fast-route and score the seeds `wk/<run>/judge.json` names** (the
   `--judge` legal seeds, buildable and floorplan-clean first, then lowest
   `final.cost`; place_simple grades each seed with `check_assembly
   --baseline` against the board it was placed from, and WARNS about a
   judged seed that is NOT BUILDABLE):
   ```bash
   python3 -X utf8 py_placer/judge_seeds.py wk/<run>/judge.json \
       --chain wk/<run>/judge_chain.txt --intent wk/<run>/intent.json
   ```
   It runs the SAME chain on every seed, so they compare, with `route.py`
   and every `route_diff.py` step at `--quick 2` (added when a line has
   none). Level 2 skips the recovery tail (net rescue, terminal escalation,
   plane finalize, reconciliation laps), where a poor placement's routing
   time goes, and every pass that only polishes copper for shipping: about
   half the time of `--quick` alone, and it picks seeds as well (measured
   over 16 seeds of five corpus boards against their full routes). Each
   seed runs in `s<S>_route/` (its `chain.log` holds every step), its last
   board is copied to `s<S>_fast.kicad_pcb` and scored by
   `py_tools/quick_score.py --intent` into `s<S>_fast.score.json`: the
   board's open nets, graded as `board_score` grades a full route but over
   the nets without a zone (level 2 skips the plane finalize, which joins a
   poured net's pads; the score lists those apart as `poured`), the
   floorplan's errors as `board_score --intent` grades them
   (`blocking_by.floorplan`, summed into `blocking`), the assembly as
   `board_score --baseline` grades it against the board the seeds were
   placed from (`blocking_by.assembly`, summed the same way; judge_seeds
   reads that board from judge.json's `baseline`), `joins_needed`, and
   `board_score`'s via, copper and segment counts. (`board_score` itself
   would count DRC grazes that level 2 leaves for the full route's cleanup,
   which say nothing about the placement.) It ends by ranking the seeds
   and naming the best. `--jobs N` routes N seeds at once: a judging route
   takes up to about 1.3 GB on a small board and 5 GB on a 350-part one.
   Never ship a `--quick` board.
3. **Keep the best** by the placement's own defects first (floorplan errors
   plus the assembly count), then blocking, then `joins_needed`, then vias,
   copper_mm, segments (`quick_score.judge_key`): every seed whose floorplan
   grades clean and whose assembly is buildable ranks above every seed that
   is not, since a route can clear neither. Copy its placement (`s<S>.kicad_pcb`, never the
   `--quick` board) to `wk/<run>/final.kicad_pcb` with `copy_board.py`. A
   kept seed is not finished while its `floorplan` or `assembly` is above 0,
   whatever its route.

In `place` mode that placement is the result. Run
`python3 -X utf8 py_tools/route_review.py wk/<run>/s<S>_fast.kicad_pcb --json wk/<run>/review.json`
on its fast route: turn each two-pad part it lists as turned against its
wiring with the `place_pose BOARD OUT set REF X Y --rot R` it prints for that
part, then check the board again (`check_assembly`, `check_floorplan
--intent`, `render_placement`). That pose is a half turn about the midpoint of
the two pads, so they trade places; `place_pose ... rotate` turns about the
footprint origin instead (pad 1 on a through-hole axial, which swings the body
by the pitch), and without `--relative` its angle is absolute (#1322).

### 3.8 Route (full mode)

Route the kept placement by running `/plan-pcb-routing` on
`wk/<run>/final.kicad_pcb`: the routing is that skill's, chain and checks
included, never a `--quick` run (the judging chain is a starting point it may
use). Then `board_score` and `check_complete` with every spec flag. If DONE
does not come, classify the failure (§4).

### 3.9 Finish the board (both modes)

Once, on the board you will hand back (`wk/<run>/final.kicad_pcb` in `place`
mode, the routed board in `full` mode), before the verifier (§5, rule 5).
In `full` mode `/plan-pcb-routing` ends with the same two steps (After
Routing Completes): run them once, here, not again inside the routing.

1. **Tidy the reference labels, and keep the tidy only if KiCad agrees.**
   Placing moved the parts and left their labels where the library put
   them, and no DONE grader reads silk (`kicad_drc_compare` drops the silk
   classes by design). Tidy them into a new file and grade both with
   KiCad's own silk classes:
   ```bash
   python3 -X utf8 py_placer/beautify_labels.py <board> wk/<run>/labelled.kicad_pcb
   KCLI=$(python3 py_tools/kicad_cli_path.py)
   "$KCLI" pcb drc <board> --severity-all --format json -o wk/<run>/silk_before.json
   "$KCLI" pcb drc wk/<run>/labelled.kicad_pcb --severity-all --format json -o wk/<run>/silk_after.json
   python3 -c "import json,sys,collections;[print(f,dict(collections.Counter(x['type'] for x in json.load(open(f))['violations'] if x['type'].startswith('silk')))) for f in sys.argv[1:]]" wk/<run>/silk_before.json wk/<run>/silk_after.json
   ```
   `beautify_labels` moves reference labels clear of pads, holes, the edge
   and each other and touches nothing else (copper byte for byte, siblings
   copied). It does not see drawn silk, a part's own outline included, so
   it can trade one overlap for another: one board went from 4 KiCad silk
   errors to 0, another from 54 to 76. Keep `labelled.kicad_pcb` as the
   board only when its silk count is lower, and put both counts in
   REPORT.md. `kicad-cli` is often not on PATH (macOS keeps it in the app
   bundle), which is why the block resolves it with `kicad_cli_path.py`, the
   tools' own lookup (#1418). Only when THAT exits 1 is there no
   `kicad-cli`: then keep the board as it was and say the silk is ungraded.
2. **`full` mode: deliver a filled copy, last (#910).** A routed board
   carries its zones as outlines with no fill: KiCad shows the planes empty
   until they are refilled (press B), and a DRC that does not refill reads
   plane-net opens that are not real. `check_connected`, `check_complete`
   and `board_score` refill on their own and never see it. From the board
   step 1 kept:
   ```bash
   python3 -X utf8 py_tools/fill_for_delivery.py <board> -o wk/<run>/delivered.kicad_pcb
   ```
   It fills with KiCad's own zone filler, copies the siblings and keeps the
   net classes. `delivered.kicad_pcb` is the board you hand back and the
   verifier grades; route nothing further from it, since a fill stored
   before more copper is laid is stale. On exit 1 (no KiCad python, or a
   net class went missing), hand back the unfilled board and say in
   REPORT.md that it must be refilled before it is plotted.

## 4. Traps measured in real runs

**Placement**
- **Rotation and pin order.** A `CROSSED` pin-order pair in `board_context`
  costs a via per net at every rotation.
- **Repeated sheet instances (channel strips, one buffer per bank): arrange
  ONE, then copy it**:
  `python3 -X utf8 py_placer/place_pose.py <in> <out> replicate --from CH1 --to CH2 --to CH3 --anchor J1`
  (#1211). It matches parts by schematic symbol and refuses unless the
  instances are wired alike, naming the pads (`--allow-net-mismatch REF.PAD`
  waives one by design). Lock the copies before placing the rest.
- **`place_pose` refuses a pad stack, but "legal" is still not
  "buildable".** Courtyards, bodies and coincident origins are
  `check_assembly`'s alone: after every pose change, run it and read
  `buildable`, not `blocking`.
- **`render_placement`'s pad-clearance list is the grader's** (#1065); its
  pad-stack list and JSON `metrics` are bounding-box counts. Read stacks
  from `check_assembly`.
- **Keep-out bands.** A pad in a `(keepout (tracks not_allowed))` band cannot
  be routed even on an empty board (#1031). Treat
  `checklist.a_off_outline.keepout_copper` like off-outline pad copper.
- **A module antenna needs its copper keep-out WRITTEN, after placement.** A
  brief's `keepouts[]` grades and places only; no routing step reads it:
  `python3 -X utf8 py_router/add_rule_area.py <in> <out> --name ANT_KEEPOUT --ref U1 --rect X0 Y0 X1 Y1`
  (#1200).
- **Re-placing strands routed copper.** Strip the copper and re-route.

**Routing**
- **Classify a routing failure before retrying it.**
  - Failed net's `blockers` empty, and `boxed_in[].geometry` above the
    board/fab floor: parameters (grid, ripup budget, width).
  - Geometry already at the floor, failures clustered at one part's escape
    face, or `py_tools/check_reachability.py --pad REF.PAD` says CAGED:
    placement. Itemise it in the report with that measurement (§6). A finer
    grid alone never widens a gap.
- **`route.py`'s failure tally covers only the nets that step owns** (#1069).
  Count the board's open nets with `check_connected` or `board_score`.
- **`route.py` finishes the planes; a repair step does not (#562, #1112).**
  Pour first (`route_planes.py`), then route with the plane nets inside
  `--nets` (`'*'` covers them). Do not end on `repair_planes.py`.
- **Widths are requests.** After each route, read
  `power_widths.<net>.under_mm`; grade power widths with
  `board_score --net-min-widths`.
- **Around a 0.8 mm BGA, the router can run out of lanes:** finer tracks and
  vias, a cheaper ground-plane layer cost (`--layer-costs`). Both go below
  the board's authored floors, so disclose that (§6). On later steps of a
  chain, pass `--clearance-ceiling`, not `--clearance`.
- **Hand-written copper:** check each join with
  `python3 -X utf8 py_tools/check_join.py <board> <NET> x,y,layer x,y,layer [via:x,y] --keep-staged <out>`.
  On `CLEAN` (exit 0), `<out>` is the board plus that join, siblings
  copied: continue from it. No tool here locks a track or via, so the join
  stays rippable, and a later route step may rip its net and route it
  again. Write joins after the last route step, re-run `check_connected`
  and `check_drc` on the result (new copper changes how the pours fill),
  and list each join in REPORT.md.

## 5. Rules

1. **Follow the workflow; choose within it.** The groups, hints and claims
   are your judgement; the stop rules and the verifier are the process.
2. **Never change the board outline** (Edge.Cuts), and never move a part that
   draws it.
3. **Stop rules:**
   - **First reach DONE**; nothing else before it.
   - **Watch long jobs.** Never leave a background job unwatched for more than
     20 minutes.
   - **If DONE will not come:** after three consecutive DIFFERENT approaches
     that each fail to lower `unrouted` + `broken` (then the rest of
     `blocking`), measured, stop and ship the best board (by `blocking`,
     then `joins_needed`, §1) with every remaining blocker itemised with its
     measurement.
   - **Hard cap:** 10 hours of wall clock, unless the user sets another.
4. **An "impossible" claim needs its measurement.** Quote
   `check_reachability` (PASSABLE is a router problem, CAGED is geometry) or
   the log block, not only its first refusal line.
5. **One verifier, at most 3 calls.** When you believe you have your best
   board, spawn ONE fresh subagent (not a fork) with
   `references/verifier.md`, filled in for your mode, board, sha256, baseline,
   intent and spec flags. On FAIL, fix what it names and continue.
6. **Record milestones in the ledger:** the first legal placement, the kept
   placement, the first routed board, the first DONE, the
   final board, and tried-and-worse boards with `--rejected`.
   ```bash
   python3 -X utf8 py_tools/board_score.py <board> --intent <i> --baseline <input> --json <board>.score.json --quiet
   python3 -X utf8 py_placer/converge.py record --ledger wk/<run>/ledger.jsonl \
       --board <board> --kind placement --parent <the board it was made from> \
       --lever "<what you changed, one line>" --score-file <board>.score.json
   ```
   Use `--kind completion` for a routed board, and close the ledger with one
   `record --final --stop-condition <1|2|3|4>` row, passing the verifier's
   per-lens files as `--lens-file`. The stop conditions: 1 done, 2 budget
   spent, 3 plateau, 4 measured-unfixable. `record` refuses 4 unless the
   ledger's last `--kind classification` row (its `--shape`: parameter,
   placement or floorplan) is not `--rejected` and no lap was recorded after
   it: the measurement behind the claim must be on the record.

## 6. Hand-back

1. **The board** §3.9 finished (in `full` mode the filled
   `delivered.kicad_pcb`), with its `.kicad_pro`, its sha256, `groups.json`
   and the intent.
2. **`wk/<run>/REPORT.md`**, containing:
   - **the result first:** a table of the mode's DONE conditions with measured
     values, the `blocking_by` breakdown, and vias / copper_mm / segments;
   - **the groups and every judgement call** in them, with its reason;
   - **which stop rule fired**, and every remaining blocker BY NAME with its
     measurement;
   - **floors:** where the run went below the board's authored or fab floors
     (route.py's `Design rules [...]` line, `check_complete --authored-from <input>`);
   - **silkscreen:** KiCad's silk counts before and after the label tidy
     (§3.9), and which board was kept;
   - **fill** (`full` mode): whether `fill_for_delivery` ran, and if not,
     that the board must be refilled before plotting;
   - **a timeline** and **where the time went**;
   - **tool gaps**, including any tool that said "fine" about something that
     was not;
   - **each verifier verdict.**
3. **Independent grade and timing**, for the report's own table:
   ```bash
   python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/grade.py <board> --baseline <input> --intent <i> --mode <mode>
   python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/measure.py --self --evidence wk/<run>
   ```
   Add `--spec <name>=<value>` to `grade.py` for each spec flag the board
   declares. `measure.py` must run from your own shell; if it refuses (exit
   3), quote the refusal.
4. The datasheet folder deleted (`rm -rf wk/<run>/datasheets`).
