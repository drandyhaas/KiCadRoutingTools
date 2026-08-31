---
name: plan-pcb-placement
description: Analyses a KiCad PCB's PLACEMENT and produces a placement plan. Measures whether the board should be placed or re-placed at all and decides from that measurement, detects an unplaced board, separates the parts whose position is a mechanical fact from the ones a search may move, reconstructs a damaged placement (pattern fit, rigid vectors, exact assignment, minimal-move legalize), repairs local violations, offers a slate of arrangements, and grades the result against a declared floorplan intent. Never changes the board outline, and never routes.
---

# Plan PCB Placement

Placement decides what routing can achieve, and a wrong placement cannot be
rescued by any router. This skill owns that half: deciding whether to touch the
placement at all, fixing it when it is wrong, and proving the fix.

**It never routes.** When the board is placed and you want copper, use
`/plan-pcb-routing`. When one run must do both, use
`/plan-pcb-placement-and-routing`, which sequences the two and owns the rules
that only exist when they meet.

## What you are optimising FOR

**A board that ROUTES: parts arranged so they work together, ending at zero
`unrouted` and zero `broken`.** Not parts returned to the poses they used to
have. On a repair or reconstruction job this distinction decides which result
you call a success.

- **Lead with the routed outcome.** `board_score`'s `blocking` —
  `unrouted + broken` first — then `quality` as the tie-break once it is 0.
- On the perturbed corpus, **`recovery` and `home /N` are DIAGNOSTICS, not the
  score.** They measure distance to the original pose. Measured (run 10):
  `recovery` −0.0014, `home` 0/30, on a run that took copper-free DRC 9 → 0,
  assembly blocking 4 → 0, and the board from NOT BUILDABLE to **buildable**.
  The one recovery figure that IS a defect signal is `collateral_pad_rms`
  rising — that means parts nothing had damaged were moved.
- A human original is a **benchmark to approach** (its vias / copper / segment
  counts), not a pose to match.

### The top-priority defect: pad copper off the outline

Ahead of every clearance graze. Those nets **cannot be routed at all**, so the
defect converts one-for-one into `unrouted` and `broken`. Measured, run 10: 11
such parts caused ALL 13 unrouted nets and most of 37 broken ones.

Read it from `render_placement --json-out`'s
`checklist.a_off_outline.pad_copper`. A whole-board pass/fail verdict is the
wrong channel to learn it from — check the per-part list.

### Scope the search to the refs the gate names

When a gate names specific parts, free exactly those and lock everything else.
A whole-board sweep orders its violators by its own priority — usually
worst-off-board first — and may never reach the ones blocking you. Measured:
2 parts freed and 105 locked cleared both blocking pairs in **63 seconds**,
where whole-board sweeps ran 10+ minutes without touching them.

Repair searches start from a part's CURRENT pose, which carries no information
once it is tens of millimetres from where it belongs; cost grows sharply with
the displacement cap. **Prefer RE-SEATING such a part over nudging it** — lift
it and search from its net centroid instead, holding everything else fixed:

    python3 -X utf8 py_placer/place_seed.py <board> <out> --intent fp.json --reseat \
        --clearance <floor>                     # bare --reseat = auto scope

The auto scope is the off-outline pad-CENTRE census, which is zero on all 33
corpus boards, so this is a no-op with exit 0 on a healthy board. It composes
with `--repair` and runs before it, and `place_reconstruct --stages
...,reseat,legalize` is the same engine as a ladder rung. **Read
`witnesses_after`, not `reseated`** — the first predicts routability, the
second counts effort. Measured on the same board: `--repair` spent 4 m 55 s
and attempted none of the 11 off-board parts (its cap ladder tops out at 5 mm
from the wrong centre); `--reseat` seated 11 of 11 in 13 s, taking the
off-outline count to 0.

Expect `recovery` to get no better and `collateral_pad_rms` to grow: this puts
parts where the netlist wants them, not where they were. That is the intended
trade.

**On a `[GATE REFUSED]`, read `accept_basis`, not the gate tuple (#698).** A
named scope and the bare auto scope are judged by different rules, and the
summary says which: `policy` is `auto:oob-strict` or
`explicit:one-term-strict`, `fired` is the basis that carried the pass, and
`terms` lists *every* basis with its before/after — so a refusal tells you what
would have to change. The auto rule is unchanged (the off-board amount must
strictly improve). A named scope is accepted when the off-outline count did not
grow, no hard gate term got worse, no declared claim got worse, and one
scope-relevant basis strictly improved; `hpwl` is the one term it is licensed to
pay, because a seat made for a declared reason is hpwl-worse by construction.
`--reseat-min-gain MM` raises the bar on the wirelength basis only (the count
bases threshold at one whole defect). The default is 0: measured over 16
explicit re-seats on four corpus boards, every **non-zero** gain was ≥1 mm, so a
floor would buy nothing there — see `tests/measure_698_min_gain.py`, and read its
`pre_prune` and `moved` columns rather than the headline, because 9 of the 10
zero rows are seats the prune sweep reverted rather than shuffles.

#### When a part will not seat, read the verdict before reaching for a hammer

Every part the rung leaves unseated carries a **`no_pose_verdict`** in the
JSON_SUMMARY and as a NOTE, and it tells you whether eviction can help at all
— `no_pose_blockers == {}` used to mean two opposite things, *nothing is near
this part* and *everything near it is locked*.

| verdict | what it means | what to do |
| --- | --- | --- |
| `keepout_blocks` | a **declared keep-out** refuses it — measured, by recounting the poses with that keep-out lifted (#701) | move the keep-out, or add the part to its `allow` list if it owns it |
| `no_movable_neighbour` | nothing seated is near it | eviction cannot help — the pocket does not exist; re-check the outline or the intent |
| `immovable_given_frozen` | everything near it is frozen (it names each neighbour **and the decision that froze it**) | unlock one of them, or accept it |
| `blocker_available` | a useful blocker exists, the rung is disarmed | raise `--evict-depth` |
| `no_single_lift_frees` | lifting any ONE neighbour frees no pose | try `--evict-depth 2` |
| `no_pair_lift_frees` | pairs do not free one either | stop; this is a floorplan problem |
| `trade_reverted` | the trade was made and scored worse | the pocket is real but costs more than it buys |
| `seated_after_eviction` | it worked | — |
| `no_target_recorded` | the part reached `unseated` with no ledger entry | a bug; report it |

`--evict-depth 1` trades out the single neighbour that frees the most poses;
`2` also sweeps **pairs**, and only for a part no single lift helped — in the
case pairs exist for, every single-lift count is zero, so the sweep cannot be
pruned by them. Bounded by counts and never a clock (#621): at most 8
candidates per part, 16 pairs, and **one trade per part**; what a cap drops is
reported. Depth 2 costs the most, and depth 3 is refused rather than defined.

Two things to know before arming it. Locked parts and declared edge connectors
are never evicted, and `--lock` is honoured by the rung — so lock what must not
move. And **an evicted blocker is WRITTEN**: it is named in `evicted` and in a
NOTE, but it is by construction outside the scope you asked for, so the board
that comes back has moved a part you did not name. `reseated` deliberately does
not count it — an eviction is not a re-seat.

### An observed violation is evidence, never permission

**Never derive an intent from a damaged board and then use it to gate that
damage's repair.** `check_floorplan --emit-intent --declare-classes` records
what the board DOES; run it on a damaged board and every overhang becomes a
declared `edge_connectors` band, and every consumer that exempts declared edge
parts then goes blind to exactly the damage you are trying to fix. Measured,
run 10: an 81 mm board emitted bands up to **160 mm** (each part's own damage
displacement); the repair census skipped all 11 off-board parts and reported
5 violators, none of them the ones whose pads were in the air; and the
reconstruct gate read `oob = 4.348` on a board with parts 158 mm out.

The emitter now caps an observed band at `max(5 mm, the part's own size)` and
emits the entry **without an `edge`** above that, saying it is treating the
overhang as damage — so a fresh intent is safe. An intent emitted BEFORE this
is not: check `overhang_mm.max` against the part before trusting one, and
re-emit if in doubt.

<non_negotiable>
1. NEVER skip the assessment. It is two commands on the copper-free board and
   it decides everything below. Skipping it is how a board with parts stacked
   on each other reaches routing.
   What the assessment decides:
     no placement at all              -> place it (P1)
     placed, and the measurement is DIRTY  -> fix it (P2/P3/P4)
     placed, and the measurement is CLEAN  -> STOP. Do not run an optimizer
       over a placement that already passes: a careful hand placement is made
       WORSE by a polish pass (measured), and the default weights caused two
       new routing failures.
   "Clean" is a measured verdict, never an impression of the board.
2. THE BOARD OUTLINE IS NOT YOURS TO CHANGE. Size, cutouts and slots are
   mechanical decisions the user owns. NEVER RESIZE A BOARD to make parts fit
   -- if they do not fit, say so and stop. This holds today only by
   construction (no writer here emits an Edge.Cuts primitive), so it is
   written down rather than left implied. Three tools in the repo DO rewrite
   Edge.Cuts and are not part of placement: `stress/fix_outline_gaps.py`,
   `stress/strip_routing.py` and `stress/prep_set2.py`. They exist to prepare
   corpus boards; never run them on a user's board.
3. A part the file marks `(locked yes)` is never yours to move, whatever an
   intent says.
4. Gate on hpwl, PAD-PAD conflicts and the assembly channel's blocking pairs.
   REPORT `crossings` and aggregate courtyard overlap; never gate on them --
   both correlate POSITIVELY with **distance-to-truth**, which is the dependent
   variable that evidence was measured against. Against routed `blocking`,
   `crossings` HAS now been measured (#703) -- it FAILS its sign rule on the
   full sample (5 boards right, 1 wrong) and PASSES it (6/0) once
   optimizer-made placements are excluded, so neither arm is the answer. The
   prohibition rests on the distance measurement, now with the knowledge that
   the routed-blocking evidence is arm-dependent
   (`docs/placement-predictors.md`).
   WHICH pad-pad channel: the pad-INTERSECTION count, which is
   `check_assembly`'s `blocking` and the routing loop's L2 refusal. The
   clearance-GRAZE count (`pad_conflicts`) is reported and not gated -- #788
   measured it as redundant with that refusal, not as unimportant, and the
   reason is written beside the gate in `loop_driver.py`.
5. Every proposal is decided by a MEASUREMENT on the board in front of you, not
   by what a pattern suggests. If no instrument confirms it, do not apply it.
6. Placement invalidates every downstream routed board. Never run it mid-chain.
</non_negotiable>

## How to run this skill

Do not read this file end to end and improvise the order. Ask the driver for
one stage at a time; it prints that stage's instructions and nothing else, and
it REFUSES to print a later stage until you hand it the evidence the earlier
one was supposed to produce.

```bash
D=.claude/skills/plan-pcb-placement/scripts/placement_driver.py
python3 -X utf8 $D --list                                  # the eight stages
python3 -X utf8 $D --stage P0 --board board.kicad_pcb      # always start here
```

Each emission carries exactly one of three tags:

| tag | what to do with it |
|---|---|
| `<stage_instructions>` | act on these yourself |
| `<subagent_prompt>` | copy VERBATIM into a subagent. Do NOT read it as your own instructions |
| `<error>` | you skipped evidence. Go produce it; do not improvise around the guard |

An `<error>` is not a malfunction. It means a gate is holding, and the gate is
there because prose gates get skimmed. Produce what it asks for and re-run the
same stage.

The rest of this file is the reference the stages point into: the doctrine
behind each rung, with the measurements that decided it. Read the part a stage
names, when it names it.

The instruments this skill decides with, all report-only until you accept:

```bash
python3 -X utf8 py_router/check_drc.py board.kicad_pcb --clearance <floor>   # on the COPPER-FREE board
python3 -X utf8 py_tools/check_assembly.py board.kicad_pcb [--baseline before.kicad_pcb]
python3 -X utf8 py_tools/check_channels.py board.kicad_pcb [--baseline before.kicad_pcb --gate]
python3 -X utf8 check_rigid_consistency.py before.kicad_pcb after.kicad_pcb
python3 -X utf8 py_tools/check_floorplan.py board.kicad_pcb --intent fp.json [--health]
```

Shared doctrine lives with the routing skill and applies here unchanged --
read it when the step below points at it:

| file | read when |
|---|---|
| `.claude/skills/plan-pcb-placement-and-routing/references/evidence-map.md` | you are about to quote a number from any instrument |
| `.claude/skills/plan-pcb-placement-and-routing/references/verifier-prompts.md` | you are dispatching a verification subagent |
| `.claude/skills/plan-pcb-placement-and-routing/references/convergence.md` | you are running a fix loop and need its stop conditions |

## Step 0: Placement gate — measure first, then decide

Before planning any routing, MEASURE whether the board should be **placed** or
**re-placed** at all, then decide from that. Do not decide from the board's
appearance, and do not decide in advance.

The measurement below is two commands on the copper-free board. Both of its
outcomes are real answers: a board that measures clean is routed as it stands
(running an optimizer over a good placement makes it worse -- measured), and a
board that measures dirty is repaired at the rung its damage calls for. What
is never an answer is not looking.

```bash
# Is the board even placed? (report-only, writes nothing, exits 3 if not)
python3 -X utf8 py_placer/place_optimize.py board.kicad_pcb --suggest-locks
```

### Decision table — when to run placement

| board state | run placement? | tool |
|---|---|---|
| **unplaced** (test it — see below) | follow the **unplaced ladder** below — seeder, then intent-driven `place_seed.py`, and report-and-stop only when NEITHER applies | see below |
| careful hand placement, routing not yet attempted | **NO** | — |
| routing already completed clean | **NO** — *unless* a spec clause placement could fix is still violated; see the last row of 9.3d in `/plan-pcb-placement-and-routing` | — |
| board already carries copper (the tools exit 3) | **NO** — placement moves footprints, not tracks | — |
| **placed, but placed WRONG** — `check_drc` on the **copper-free** board returns violations, or a mechanically-fixed part sits where mechanics forbid (an edge connector clear of every outline edge, a hole away from the pattern its siblings define) | yes, and **the quench is the wrong tool** | **Step 0a-0 first: RECONSTRUCT** — `place_seed --repair` for local violations, `place_reconstruct.py` for structural damage. Only then `place_optimize` on the residue |
| rough / imported / auto-generated placement | yes | `place_optimize.py --max-displacement 3` |
| placed, and the user wants placement OPTIONS — or a converged run's remaining failures were classified **floorplan-shaped** | yes — generate a SLATE, not a nudge | `place_portfolio.py` (Step 0c-bis) |
| routing FAILED and `/diagnose-routing-failures` blames **congestion / blockers** | yes | `place_route_loop.py` |
| routing FAILED and the diagnosis is **parameters** (grid, ripup budget, layer costs) | **NO** — fix the parameters | — |

`docs/placement-optimization.md`'s own measured verdict: ship the quench as a
**repair** tool for rough/generated placements, **not** as a polish pass on
careful hand placements — on a good hand placement the result was neutral at
best, and the default weights *caused 2 new routing failures*.

**Placement invalidates every downstream routed board.** Never run it mid-chain;
re-run the whole chain from the placed board.

### Step 0a-0: RECONSTRUCT before you optimise — the mechanical parts are arithmetic

**Run this before Step 0a, and it takes seconds.** The quench is a local search over a
continuous lattice; it is the right tool for a *rough* placement and the wrong one for a
*wrong* placement. Anything whose position is a mechanical fact should be **computed and
placed**, not handed to an optimizer — and computing it usually also collapses the search
space for everything else.

**First, TWO commands Step 0 runs on the board with ZERO copper — both conjuncts,
always (the R2 rule, applied to the gate itself):**

```bash
python3 -X utf8 py_router/check_drc.py board.kicad_pcb --clearance <floor>   # NOT piped
echo "EXIT=$?"
python3 -X utf8 py_tools/check_assembly.py board.kicad_pcb   # reads the board's own floor
echo "EXIT=$?"
```

`check_assembly` resolves its clearance from the board (Default net-class, else
`routing_defaults`) and prints it with its source, so **omit `--clearance`** —
that is what grades at the board's floor. Pass it only to override
deliberately. It used to default to a flat 0.25 and so graded stricter than the
board it was grading.

A copper-free board has no routing, so **every violation these return is a placement
defect that no router can ever remove**. `check_drc` measures copper clearances (68
PAD-PAD on one damaged board vs 0 placed correctly); `check_assembly` measures
BUILDABILITY — its blocking channel is cross-footprint pad INTERSECTION, which the
clearance channel is structurally blind to when the pads share a net (run 5 SHIPPED
two 0402s stacked, C14 on R14, both pads +3V3: `check_drc` 0, every gate green, KiCad's
own courtyard check gagged by the project writeback). Both land in `board_score`'s
`blocking` (`drc` and `assembly`). The channel calibration that makes `check_assembly`
gateable: its blocking count reads **0 on all 33 healthy in-repo boards** in both exact
and AABB currencies.

**Do not gate on the AGGREGATE `overlap_area`** — it has a legitimate nonzero floor on
human boards (one human-routed 2-layer board in the corpus carries 5.375 mm2 of
mount-hole-under-shell courtyard kisses in its own shipped layout) and run 2 measured it positively correlated with **distance-to-truth** — a
different question from routed `blocking`, which nothing here has measured it
against (`docs/placement-predictors.md`). The
per-pair blocking COUNT is the gateable quantity; courtyard/fab pairs are ADVISORY
(`check_assembly` labels each with its waiver class), fix targets for the placement
loop below, never a gate alone.

**Then reconstruct, in this order. Each rung has an applicability test — run the test,
and when it fails, say so and fall through to the next rung.** None of these invents
geometry: every one either reads a determinant off the board or reads it from the spec.

**R1 — Separate the parts whose position is NOT a netlist question.** For each, name what
determines it. This is Step 0a's table, used as a *placement* list rather than a lock
list:

| class | what determines the position | detection test on ANY board |
|---|---|---|
| mounting holes, NPTH, drill-outs | the enclosure's standoff pattern | `pad.pad_type == 'np_thru_hole'`, or 0 connected pins. **The quench provably cannot place these** — the advisor says so itself: *"0 connected pin(s) → invisible to the airwire cost, so only the halo term decides where it goes"* |
| edge connectors, castellations, card edges | the mating standard / the outline | courtyard must intersect (castellated: be centred on) the outline. `check_floorplan`'s `edge_connectors` block already computes the edge and overhang |
| enclosure-referenced parts (USB/barrel/RF jacks, buttons, LEDs, displays) | an aperture in the spec | the EXACT position is spec-only, but the CLASS is board-derivable (run-4 `placement/part_class.py`): an `edge_receptacle`-class part (USB/HDMI/RJ45/card/jack by footprint name or CC1/CC2-style pinfunctions) with **no overhang AND edge clearance past the seat tolerance is implausibly posed** — the plug cannot reach it. NOT a bare distance threshold: run 3's displaced USB-C sat only 2.45 mm from a *different* edge, so "far from every edge" misses a swapped part. `check_floorplan --emit-intent --declare-classes` declares these automatically; the advisor DEMOTES an implausibly-posed receptacle out of the lock list (a lock is not a placement) |
| fiducials, test points | a fab or test-fixture rule | usually spec; sometimes a symmetric pattern (see R2) |

**R2 — Ask whether the board itself determines the position. Often it does, and then it
is arithmetic rather than search. But PROPOSE, never trigger.**

The idea: a part may belong to a family whose pattern is over-determined by its surviving
members, so you can fit the pattern to the survivors and predict the rest. Worked, on one
board: a rectangular outline `30–76` with two mounting holes still at `(73,33)` and
`(73,73)` over-determines a **3.00 mm corner inset**, which predicts the two displaced
holes at `(33,33)` and `(33,73)` — **2 µm from where the human had put them**, with no
spec and no reference board.

**Do not turn that into a detector.** Written as one it is unsafe, and both failure modes
were measured over this repo's 33 in-repo boards:

- **False positive.** One corpus board has six holes: four at a `(3.81, 3.81)`
  corner inset and two elsewhere. A "hole off its siblings' pattern" rule flags those two, and
  they are a perfectly ordinary **mid-edge mounting pattern**. Reconstructing that board
  would damage a correct one.
- **False negative, on the very board the rule was derived from.** Its four holes sit
  at one 3 mm inset, but its outline runs `30.0 - 75.99764`, so nearest-corner insets come out
  as `3.000` and `2.998` and a naive comparison reports *four distinct patterns* and
  stays silent. Sub-µm outline asymmetry is normal, not exceptional.

So the safe shape is: **the pattern fit PROPOSES a position; the bare-board `check_drc`
DECIDES.** Apply a proposal only when

- the fit is over-determined (≥ 2 survivors for a translation, ≥ 3 for anything with a
  rotation or a scale), **and**
- the residual on the survivors collapses to a grid step when compared with a tolerance,
  **not** to exact equality, **and**
- **applying it measurably improves the copper-free `check_drc` count AND does not
  increase the out-of-board amount.** The count alone is the one Step 0 instrument with
  a measured false-positive rate of **0 on 31 of 31** in-repo boards — and it is also
  **gameable by evacuation**: a conflict removed by pushing the part off the board
  reads as an improvement (measured, run 2: a wrong corner assignment reached PAD-PAD
  **0** while `oob_amount` went 9.6 → **146.6 mm**, and the count-only gate would have
  chosen it; the oob conjunct rejected it on board-only numbers). Both conjuncts, always.
- One insensitivity to know: NPTH mounting holes have **no copper**, so a proposal that
  moves only them cannot change the PAD-PAD count at all — gate the hole moves together
  with the R4 descent they enable (R2+R4 as a unit), not alone.

If the count does not improve, **revert the proposal**: the determinant was not on the
board. Then go to the spec, and **if there is no spec, say so and stop reconstructing.**
A predicted position that no instrument confirms is an invention, and inventing
mechanical geometry is what this skill forbids everywhere else.

**R3 — Apply the positions you derived, with the REPAIR tools, not the seeder.**
Two measured facts govern this rung (run 2, both lock configurations tried):
`place_seed --force` on a placed board **re-seats everything not file-locked at
connectivity centroids** (85 of 92 parts) while leaving the two zone-targeted
parts **exactly where they were** — `must_lock` locks are stamped on the OUTPUT,
so on a re-run they freeze the part at the wrong position the zone exists to
fix, and a zone smaller than the part's courtyard used to be unsatisfiable by
construction. The tools that apply a derived position on a placed board are:

```bash
# violation-driven minimal-move repair: ONLY violators move, worst first,
# escalating displacement cap; everything clean freezes
python3 -X utf8 py_placer/place_seed.py board.kicad_pcb repaired.kicad_pcb \
    --intent floorplan.json --repair [--dry-run]

# structural reconstruction (swapped regions, dragged selections): pattern
# fit -> rigid ±v vectors -> ONE simultaneous candidate assignment (exact
# ILP) -> minimal-move legalize. Propose-only stages, each gated.
python3 -X utf8 py_placer/place_reconstruct.py board.kicad_pcb repaired.kicad_pcb \
    [--intent floorplan.json] [--dry-run]
```

Zone semantics that make the spec-coordinate pattern work: a zone that cannot
contain the part's courtyard at any rotation is graded (and seated) on the
part's **anchor point** instead — declared automatically, noted in the output.
`place_seed --force` remains the **unplaced-board** tool; reaching for it on a
placed board is the measured mistake, not the procedure. Lock the refs after
they are placed (`--repair` keeps existing stamps; the seeder stamps
`must_lock` on its output), so the lock is applied to a position you verified
rather than to one you inherited.

**R4 — Then test for a rigid displacement, because R2/R3 just gave you the vector.**
When parts moved as a block (a bad merge, a re-imported netlist, a dragged selection),
every member moved by the **same** vector, and the offset between where a known part *is*
and where R2/R3 says it *must be* is a candidate for it. The test, on any board:

- compute that offset for **every** part whose correct position you established;
- **if two or more agree to within a grid step — UP TO SIGN — it is a real group
  vector.** Offsets of `+v` and `−v` are agreement, not disagreement: an EXCHANGE
  (two regions swapped) displaces its two groups by opposite vectors, and that ± pair
  is exactly the swap's signature (run 2's holes read `−v` and `+v` and the rung's
  old letter — "disagree → stop" — would have refused the very case its own measured
  example succeeded on). Offsets that disagree in *magnitude* still mean there is no
  single rigid displacement — then **this rung does not apply, stop here** and hand
  the board to Step 0c;
- when it does apply, the remaining parts no longer live on a continuous plane. Each has
  a small discrete candidate set — `{0, +v, −v, …}` — and you can choose per part by
  coordinate descent, with **pad-clearance conflict as the hard term and `hpwl` to break
  its ties**. Neither alone works: conflict alone has many zero-cost arrangements, and
  `hpwl` alone is what the quench already minimises.

Measured on a 92-part board: two mounting holes collapsed the problem to a **three-way
choice per part**; that descent took PAD-PAD DRC to **0**, `overlap_area` 142 → 13, and
put **25 %** of the displaced parts within 2 mm of home. Every quench and loop arm on the
same board, across seven caps, left PAD-PAD at 25–51 and **0.00 %** of parts home.

**R5 — Anchor the LARGE parts next, and size the gaps between them from what has to
live there.** After the mechanically-fixed parts, the big parts are what set the
floorplan: everything else is assigned to one of them. **Nothing in the placement code
orders by size** — the quench sweeps its parts in reference order and moves whatever the
cost function likes — so this ordering is yours to impose, by placing/locking the anchors
before you run it.

**The gap between two adjacent anchors has to fit two things, and only one of them is
what you would guess:**

```
gap  >=  (small parts that belong in the corridor: sum of their extents, plus clearance)
       +  cut_nets x (track_width + clearance) / copper_layers        # the routing
```

**Measured, and the second term is almost never binding.** Across the in-repo boards the
routing demand between adjacent large parts comes out at 0.4–1.6 mm while the gaps humans
actually left are 2.9–9.8 mm — 2 to 6× more. **The corridor is for the PARTS.** Scored
against the gap a human chose, per board (gaps scale with board size, so the comparison
is within a board, never pooled across them):

| predictor | what it is | correlation with the human's gap (NOT with routed `blocking`) |
|---|---|---|
| **small parts in the corridor** | count, or summed extent | **positive on 8 of 8 distinct boards, r = +0.41 … +0.90** |
| routing cut | nets crossing ÷ layers | ~0, and often negative (−0.48 … +0.08) |
| **the quench's own whitespace term** | `halo_base + halo_coef·sqrt(pin_count)` | **no consistent sign — 5 boards negative, 3 positive** |

The dependent variable in that table is **the gap a human left**, not routed
`blocking`. The corridor law has never been correlated with a routed outcome,
and the A/B recorded below found that widening the two deficit corridors bought
no completion benefit at all — see `docs/placement-predictors.md`.

So **do not expect `--halo-coef` to reserve the corridor**: it is a function of pin count,
which is not the quantity that decides. The assignment you need already exists — `route.py
--list-groups --group-by decap` names which small parts belong to which IC — so the
corridor between two anchors is *their* small parts, and its width is a number you can
compute before placing anything.

**Two run-2 measurements bound what this law is FOR.** (1) The corridor law is
descriptive, not yet prescriptive: an A/B that widened the two deficit
corridors against an identical no-widening control produced **no completion
benefit** (both arms routed to `blocking 0`) and only a marginal quality edge
(−10 vias on one board/one seed — not causal evidence). What converted that
board was the reconstruction and this rung's ORDERING principle, not the
corridor arithmetic. (2) **Moving an anchor to satisfy a corridor can enter
another anchor's ESCAPE FACE**: the widened J6 landed in the QFN's AD-bus face
and cost 6 of 30 fanout escapes. After ANY anchor move, re-run the per-face
lane ledger (the escape-channel section above) for every neighbouring
fine-pitch part before accepting it.

**When this rung does not apply:** a board with no clear size hierarchy (all parts
similar), or one where the large parts are already mechanically fixed by R1–R3 and there
is no freedom left. Say which, and go to Step 0c.

**Hand the residue to Step 0c, not the original mess.** The rungs are complementary and
the measured split is stark: reconstruction fixes the hard, blocking-relevant metrics and
is indifferent to the soft ones (it left `crossings` at 392); the quench fixes the soft
ones (`crossings` 412 → 227) and never touches the hard ones. Running only the quench on a
wrong placement optimises a board that cannot pass DRC however well it routes.

**The whole ladder is now tooling, in escalation order** (each stage is
report-only with `--dry-run`, and each does nothing on a healthy board —
measured: `place_reconstruct` on the correct control board proposed nothing
and moved 0 parts). **The ladder ESCALATES, it never COMPOSES**: each rung
consumes the PREVIOUS rung's gated output or the unmodified input, never a
partial repair — `--repair` legalizes in place and smears the ±v
structure the reconstruct's pattern fit needs (run 3 ran repair as a
dry-run test, judged it insufficient, and correctly handed reconstruct the
UNSMEARED board). Declare the edge classes FIRST (`--declare-classes`) so
the reconstruct sees the bands:

```bash
python3 -X utf8 py_router/check_drc.py board.kicad_pcb --clearance <floor>   # measure (R0)
python3 -X utf8 py_tools/check_floorplan.py board.kicad_pcb \
    --emit-intent auto.json --declare-classes   # part-class auto-declaration:
                                       # edge parts get bands; an implausibly-
                                       # posed receptacle gets NO edge (derive)
python3 -X utf8 py_placer/place_seed.py board.kicad_pcb r.kicad_pcb \
    --intent auto.json --repair        # local violations, minimal-move; seats
                                       # DECLARED-edge parts on their bands
python3 -X utf8 py_placer/place_reconstruct.py board.kicad_pcb r.kicad_pcb \
    --intent auto.json [--assign-rounds 2]  # structural damage (R1-R5
                                       # productized: tiers, pattern fit, ±v,
                                       # exact ILP, prune sweep, legalize).
                                       # Round 2 peels a displaced ISLAND's
                                       # boundary once round 1 made the anchor
                                       # centroids truer (measured: +7 members)
python3 -X utf8 py_placer/place_optimize.py r.kicad_pcb out.kicad_pcb ...    # 0c residue
```

### Step 0a-1: the placement FIX LOOP — measure, fix, VERIFY, repeat until clean

**Routing may not start while a placement mistake is on the record.** The ladder
above is one pass; this loop is what makes the placement phase END only when the
instruments and an independent verifier agree it is clean. Run 5 shipped a stacked
part because the phase ended when the operator believed it was done; the user's
run-6 directive is the loop below, and it is BLOCKING.

Each lap:

1. **Measure** — three instruments, JSONs kept as evidence:

   ```bash
   python3 -X utf8 py_router/check_drc.py board.kicad_pcb --clearance <floor>
   python3 -X utf8 py_tools/check_assembly.py board.kicad_pcb \
       --baseline <the ORIGINAL input board> --json wk/assembly_lapN.json
   python3 -X utf8 py_tools/check_channels.py board.kicad_pcb \
       --grid-step <g> --json wk/channels_lapN.json
   ```

   **`check_assembly` and `check_channels` read the board's own floor** (and
   `check_channels` its track width too) when the flags are omitted, and print
   each value with its source — so omitting them is what grades at the board.
   They used to default to a flat 0.25 / 0.3: on a 0.2 board that track width
   invented a "U2 N short 1 lane" deficit that did not exist (supply 14, demand
   12), handed forward as floorplan-shaped residue. `check_drc` still wants
   `--clearance` spelled out. Read the printed tag: `[fixed default]` means the
   board declared nothing, not that it agreed.

   `--baseline` is load-bearing: dense healthy boards ship hundreds of by-design
   courtyard kisses (corpus: 235), so the loop's advisory fix-list is the pairs
   NEW relative to the input, never a shipped design's own geometry.

2. **Fix iteration** — one ladder invocation targeting the NAMED findings: blocking
   body pairs and pad/hole conflicts go to `place_reconstruct` (full stages if
   structure moved, `--stages legalize` for local residue — the repair census
   charges body pairs since run 6); NEW advisory pairs and channel intruders go to
   the quench with the ledger's `eaten_by`/pair refs as targets and everything else
   locked. One lap = one ledger entry, recorded before the next begins.

3. **VERIFY, blocking** — a 9.4b boundary verification (`/plan-pcb-placement-and-routing`) with **check 5,
   assembly-clean** (below). The verifier receives the fresh `check_assembly` and
   render JSONs; FAIL blocks the next step. The phase may not proceed to routing —
   and a routing failure later classified placement-shaped RE-ENTERS this loop —
   until the verifier passes.

4. **Repeat** until: `check_assembly` blocking == 0 AND no undispositioned
   NEW-vs-baseline advisory pair AND bare `check_drc` == 0 — or a lap reports a
   finding **measured-unfixable** (locked pair, no legal slot), which stops the loop
   with the named pairs and reasons, never with "done". Cap: 5 laps (each is
   verifier-gated, so a lying lap cannot advance; a 6th lap means the fix tool is
   the problem — file it, do not iterate past it).

A face in **deficit at the finest legal grid** in the channels JSON is a
floorplan/placement fact no routing parameter can fix — it classifies later
routing failures as placement-shaped BY MEASUREMENT (9.3d of `/plan-pcb-placement-and-routing`, first mechanical
input) and its `eaten_by` refs are the lap's quench targets. A deficit only at
the routed grid means: try the finer grid before moving anything.

### If the board is UNPLACED

**Do not rely on an exit code to tell you.** `--suggest-locks` exited **0** and
happily gave advice on a board with 42 footprints at their generator's default
positions and no outline at all. Test positively instead — any of these means
unplaced, regardless of what the tools return:

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('board.kicad_pcb')
pcb.board_info.board_bounds is None        # no Edge.Cuts outline to place INTO
len({(round(f.x, 3), round(f.y, 3)) for f in pcb.footprints.values()}) < len(pcb.footprints) / 2
```

`check_floorplan.py --emit-intent` is the other honest probe: it exits **3** with
*"the board has no Edge.Cuts outline"*, and its `JSON_SUMMARY` carries
`state_unplaced` / `state_partially_unplaced` / `state_spread_ratio` for the
cases where an outline does exist.

This toolchain does not place a board from scratch **unaided** — but "report
and stop" is the LAST rung of a ladder now, not the first answer. Walk it in
order:

1. **The repo has its own seeder** (a script that writes a starting floorplan
   and the outline from the spec): that is the placement step — run it, then
   treat its output as the "rough / generated placement" row of the table
   above. If the seeder takes a `--seed`/`--variant` axis, that plus
   Step 0c-bis is how you offer the user OPTIONS instead of one take-it-or-
   leave-it arrangement — but rank the SEEDS first (`compare_seeds.py`, next
   rung): the portfolio explores around ONE seed and cannot rank across
   them.
2. **No seeder, but an intent exists — or the spec states placement facts**
   (connector edges, a fixed regulator cluster, a decap rule): author/verify
   the intent with the Step 0e machinery, then generate the seed from it:

   ```bash
   python3 -X utf8 py_placer/place_seed.py board.kicad_pcb seed.kicad_pcb \
       --intent floorplan.json [--seed N]
   ```

   The seeder turns the intent's constructs into placement (edge bands →
   edge poses, single-ref zones → the spec coordinate, multi-ref zones →
   a packed block, everything else → its connectivity centroid), stamps
   `must_lock` refs `(locked yes)`, polishes, and **grades its own output
   against the same intent** — exit 4 means the seed does not satisfy the
   intent it was built from, and says which rule broke. Rotations: the input
   rotation is kept when it fits, with a noted 90° lattice fallback when it
   does not; a part whose rotation is a DECISION (pin order) must be locked —
   the intent schema cannot express one, and an unlocked load-bearing
   rotation was never protected from the quench either. Explore a LOCKED
   part's rotation with Step 0a-bis arithmetic (below) — **the portfolio's
   `poses` strategy never perturbs locked refs, so it cannot explore exactly
   the rotations the lock protects** (measured, run 7: five poses iterations
   across two slates, zero pose changes; the winning rot-180 came only from
   the manual 0a-bis pass).

   **When the seeder takes `--seed`, rank the seeds with ONE identical
   full-board probe each BEFORE sinking 0a-bis/portfolio effort into any of
   them** — crossings/hpwl cannot rank seeds, and the portfolio's shared-net
   window never crosses seeds (measured, run 7: the crossings-best seed's
   family probed 53 full-board failures while a crossings-middle seed probed
   39, at a third of the search effort — the whole first portfolio line was
   spent on the seed the wider instrument then displaced). That is one
   command now:

   ```bash
   python3 -X utf8 py_placer/compare_seeds.py board.kicad_pcb --intent floorplan.json \
       --seeds 0 1 2 --out-dir wk/seedcmp --ignore-nets <the Step 5 plane nets>
   ```

   It runs place_seed per seed (a seed failing its own intent gate is
   recorded, never ranked), probes every survivor with the SAME net patterns
   and route args, and emits the ranked table + `seeds.json` +
   `best_seed`.

   **Its printed note TRUNCATES the failed-net list and `seeds.json` does not
   carry the rest** — so ledgering the ranking from either one records a
   count in disguise, against 9.4's names-not-counts rule (`/plan-pcb-placement-and-routing`). Recover the full
   list from the per-seed probe output before ledgering. The same gap opens
   at ADOPTION: the board you adopt is a quench of the seed you ranked, i.e.
   a different board, so probe THAT one for its own failed names — otherwise
   the routing phase's first whack-a-mole comparison has no baseline to diff
   against.

   **Forwarded flags need the `=` form**: `--seed-args` / `--route-args` are
   `nargs="+"`, which argparse refuses to fill with dash-prefixed values, so
   `--seed-args --clearance 0.15` exits 2. Pass one quoted string —
   `--seed-args='--clearance 0.15 --max-displacement 2'`.

   **For every must_lock part under a HARD geometric clause, run Step 0a-bis
   ON THE SEEDED OUTPUT before the portfolio** — the seeder keeps the pile's
   input rotation, which is a generator default, not a decision. If a
   rotation wins on inversions, apply it with `write_placed_output` and
   RE-SEAT the decaps that served its supply pins (`seeder._try_place` at
   the relocated pin, zone-constrained), then re-run the pin-exact gate:
   rotating a part moves its pins, and a decap seated to the old pin
   silently breaks the proximity clause (measured, run 7: 7.74 mm vs the
   3 mm limit until the re-seat).
3. **Neither** — no seeder, no intent, and the spec pins nothing (or there
   is no outline; the outline is spec-owned and is never invented): report
   that plainly, tell the user to place the parts in KiCad, and offer to
   show them the current state.

   Before concluding this, check whether the board carries a **declared
   design brief** — a `<board>.design-brief.json` sibling, discovered
   automatically, holding the facts the board file cannot contain (which
   connectors are user-facing, which edge each belongs on and where along
   it, what the enclosure forbids). `check_floorplan --emit-intent` compiles
   it into the intent, so branch 2 applies after all. If there is none, ASK
   for one — three fields and one row per connector is a useful brief, and
   `"unknown"` is a legal answer that is reported rather than guessed. See
   `docs/design-brief.md` and the driver's `P-brief` stage.

**Then copy the `.kicad_pro` and `.kicad_dru` onto its output yourself.** A
seeder writes a `.kicad_pcb` and, like `place_optimize.py`, usually nothing else
— and it is the FIRST thing that touches the board, so a missing sibling there
propagates through the entire chain: every later step reads no project, resolves
its floor from the stock netclass instead of the spec, and stamps that looser
floor over tighter copper. `copy_board.py` copies a board *with* its siblings,
but a seeder is not a copy, so this one is on you:

```bash
cp board.kicad_pro seed.kicad_pro && cp board.kicad_dru seed.kicad_dru
```

```bash
python3 -X utf8 py_tools/render_placement.py board.kicad_pcb -o /tmp/state.png
```

Do not pass `--allow-unplaced` to "make it work". On a pile of parts every
candidate pose is illegal, so the run prints "0 parts moved" plus a legality
block that *looks like a result*.

### Step 0a: what the SPEC fixes in place — read this before the lock advisor

**Every board mates with something.** A Pico-footprint carrier drops into a
2.54 mm header; a USB receptacle has to line up with an enclosure aperture; a
mounting hole has to hit a standoff. Those positions are **mechanical facts, not
optimizer variables** — and a 3 mm nudge that improves `crossings` by 20% can
make the board physically not fit, which no routing metric will ever tell you.

**The lock advisor does not know this.** `--suggest-locks` (Step 0b) infers from
footprint names and reference prefixes; it cannot read a requirements document,
and its lexical rules "miss house libraries entirely". It is the **second** pass.
The spec is the first.

**Where the spec goes so a tool can read it.** A `<board>.design-brief.json`
sibling is the declared channel (#711). `check_floorplan.py` auto-discovers it
and `--emit-intent` COMPILES it into `edge_connectors` and `keepouts` — the
constructs the grade and the seat search already act on — so the placement
CLIs receive it through the `--intent` they already take, not through a flag
of their own. It travels with the board's other siblings down a chain. Read
what a board already declares with

```bash
python3 -X utf8 py_tools/board_brief.py board.kicad_pcb --json wk/brief0.json
```

whose `design_brief` section is the only DECLARED one in that document; every
other section, `mechanical` included, is inference. `docs/design-brief.md` is
the reference.

**1. List what the spec fixes, and cite the requirement next to each ref.**
Read the board's requirements/spec before touching placement. Anything with a
coordinate, a pitch, a mating standard or an enclosure feature is fixed:

| what | typical source | example |
|---|---|---|
| edge/board-to-board connectors | the mating standard | Pico castellated rows: 2.54 mm pitch, 17.78 mm apart |
| USB / barrel / RF connectors | enclosure aperture | a spec clause fixing the receptacle to a named edge |
| mounting holes | standoff pattern | keep-out + exact XY |
| castellated edges | the carrier's pad field | pad centred **on** the outline |
| test points, antennas, sensors | mechanical or RF | an antenna keepout is not negotiable |

The table is the mechanical-facts slice of a broader taxonomy (#118): **part
classes obey different placement logic** — a decap is governed by pin
proximity, a connector by the mating standard, a crystal by field distance
and leg symmetry, a series termination by being the chain's free terminal, a
bulk cap by almost nothing. When a part fits no row above, ask which CLASS
governs it — and which rule and grading signal that class implies — before
treating it as free for the optimizer to trade.

**2. CHECK EACH ONE IS ALREADY THERE, and move it if it is not — a lock is not a
placement.** This step has exactly one verb in most runs, `--lock`, and `--lock` means
*freeze it where it currently sits*. On a correctly-placed board that is the same as
correct. **On a misplaced board both branches are wrong**: lock the list and you pin the
error at exactly the parts whose position is a mechanical fact; leave it unlocked and you
hand a connector to an optimizer that has no idea where a connector belongs. Measured on
one board: of the 13 refs the advisor printed, **8 were displaced**, including a USB-C
receptacle, two headers, a JST connector and two mounting holes — each 15.8 mm from its
mating position, and nothing in the advisor's output hinted at it. Its two HIGH
*geometric* reasons (`courtyard leaves the board outline`) were **products of the
misplacement being reported as evidence for preserving it**, and on the correct board the
same advisor rated two other connectors HIGH that it demoted to name-only MEDIUM on the
damaged one — it loses true signals as well as manufacturing false ones.

So the order is **place it, then lock it** (Step 0a-0 above computes the position;
`place_seed.py --intent … --force` applies it), and the lock is only meaningful once you
have confirmed the part is where the mechanics say.

**Then pass the locks to EVERY placement invocation.**
Not just the first — `place_optimize`, `place_route_loop` and every retry:

```bash
python3 -X utf8 py_placer/place_optimize.py board.kicad_pcb placed.kicad_pcb \
    --lock 'J*' 'H*' 'U1' --max-displacement 2
```

**And pass the INTENT to every placement invocation, for the same reason.**
Since #702 `--intent` is not a grading flag: its declared zones, keep-outs and
exclusive zones are HARD per-move gates inside the quench, and its `must_lock`
globs and edge claims are locked. A step you forget to pass it to is a step
that optimises against no constraint at all — which is exactly the defect #702
fixed, one level up. `place_optimize`, `place_route_loop`, `place_seed` and
`place_portfolio` all take it:

```bash
python3 -X utf8 py_placer/place_optimize.py board.kicad_pcb placed.kicad_pcb     --intent wk/intent.json --lock 'J*' 'H*' 'U1' --max-displacement 2
```

It is MONOTONE — it prevents a part being walked out of its zone, it does not
walk one back in. A part that is already out stays out; re-seating it is
`place_seed --repair`'s job, not the gate's.

`(locked yes)` stamped in the board file (a seeder's `must_lock` output)
satisfies this for every place_* tool — the file lock is honored everywhere
`--lock` is. When you rely on it instead of `--lock`, say so once in the
ledger, so an auditor can tell a carried lock from a dropped one (run 7's
portfolio argvs carried no `--lock` and the audit had to reverse-engineer
that the in-file stamps made it harmless).

**3. Record them in the intent, so they are GRADED and not merely hoped for.**
A lock you forgot to re-pass is silent. A `must_lock` the grader checks is not:

```jsonc
{
  "schema": 1,                          // REQUIRED -- the loader refuses without it
  "kind": "floorplan-intent",           // REQUIRED
  "must_lock": ["J1", "J2", "H1", "H2", "H3", "H4"],   // cite the requirement id here
  "blocks": [
    { "name": "pico-header-north",                      // and here
      "refs": ["J1"],
      "zone": [100.0, 59.9, 100.4, 60.3] }   // [x0, y0, x1, y1] -- NOT {x,y,w,h}
  ]
}
```

A zone tighter than the part's courtyard is the spec-COORDINATE pattern: the
grader and the seeder both fall back to anchor-point-in-zone for it (noted in
their output), so a 0.4 mm zone around a mounting hole is satisfiable. Zones
meant to CONTAIN a group must still be at least courtyard-sized.

`edge_connectors` constrains **which edge** and `overhang_mm` — it cannot pin an
XY. Anything with an *exact* position needs a `blocks` zone a few hundred microns
wide around the spec coordinate. Then `check_floorplan --intent` **fails** the
moment an iteration walks one, which is the mechanism a spec-conformant board
needs and prose does not provide.

**4. Scope `decaps` to the caps the requirement names, and lock those too.**
The quench has no decap-proximity term, so it walks a *different* cap past the
limit every run — lock one and the next moves. Locking them one at a time is
whack-a-mole; lock the named set at once. Measured, that cost `crossings`
52 → 60, and **that is the correct trade, not a regression**: a spec-conformant
placement that routes slightly worse beats a spec-violating one that routes well.

**Do not** lock a part the spec does not fix, "to be safe". A wrong lock freezes
a part that needed to move and the failure is invisible — which is the same
reason nothing is auto-locked.

**Then RE-SEAT the locked cluster instead of leaving it frozen.** Locking is the
right answer to "the quench walks a different cap out every run"; it is the
wrong answer to "these parts are in the wrong places". `placement/reseat.py`
gives the second one without giving up the first: it re-assigns a cluster's
members among slots generated *around the anchor's own pins*, so every candidate
satisfies the proximity rule by construction, and it accepts only when the
cluster's exact objective improves.

```python
from placement import reseat
cl = reseat.clusters_from_tethers(pcb, state, radius_mm=3.0)
for row in reseat.reseat(state, cl):
    print(row.to_dict())     # moved / before / after / repairs / accepted
```

Use it when a rotation moved the pins a decap was seated to (Step 0a-bis:
rotating a part silently breaks the proximity clause — measured 7.74 mm against
a 3 mm limit until the re-seat), or when the caps are locked and the rest of the
board has since moved around them. It cannot make the cluster worse — acceptance
is on the exact objective, not on the assignment surrogate — but re-run the
proximity gate afterwards anyway, because `radius_mm` is what *you* told it the
rule was.

**Expect most decap clusters to report "every member net is an ignored or
high-fanout rail" and be left alone, and do not read that as a failure.** A
decoupling cap's nets *are* rails, and scoring a pose against a 96-part GND MST
would measure distance from the middle of the board. Where such a cap sits is
governed by its pin, which the slot pool already enforces. The rows that matter
are clusters carrying **signal** nets — a series termination, a filter network,
a part tethered into a zone.

### Step 0a-bis: enumerate the POSES of every part under a HARD geometric clause

For any part a HARD clause constrains geometrically (a matched pair's skew, a
bus's pin order, a diff pair's P/N geometry), compute the metric for **all four
rotations** from pad coordinates BEFORE accepting the placement — arithmetic,
not routing. Run 5 measured both directions of this:

- PCB23 (pair skew): pose enumeration on the connector pair took the skew from
  3.69 mm to **0.29 mm against a 1 mm limit** — a clause that fifteen routing
  experiments could not fix, closed by placement arithmetic.
- QSPI pin order: U3 at rot 0 met U1's stack in reversed order (9 field
  inversions counted by hand; 4/7 nets routed). Rot 180 made the same nets
  route **7/7 on the first plain call**. The optimizer's score was indifferent
  between the two poses.

The mechanical tools: `converge.py poses --ref U3` ranks legal poses and its
`components.inversions` carries the pin-order count (`placement/pair_order.py`
— a LOWER bound on crossings no router can remove, so trust it over the cost
tie). A **series part in a matched chain** (source-termination resistor, AC
cap) is a *free terminal*: its pose is the knob that sets where the chain's
segment lands, so enumerate its poses too, not just the ICs'.

**Read `poses` output with two caveats** (run-7 S4). The JSON now discloses
its `knobs` (unset clearance/edge resolve from the BOARD's own floor) and a
`dropped_in_place` census — a nonempty `dropped_in_place` on a thin or empty
ranking means the knobs (or the lattice) vetoed rotations at the part's own
spot, not that the part is stuck; check the resolved knobs against the
board's floor before believing "no legal pose". And the candidate lattice
does not necessarily contain the part's CURRENT coordinate, so check the
four rotations AT THE PART'S OWN (x, y) with `candidate_valid` besides
reading the ranked list — the cheapest and most common 0a-bis candidate,
flip in place, can be absent from an otherwise-legal ranking (measured:
U3 rot-180 legal in place on two seeds where the enumeration listed no
rot-180 pose at all).

Run this **second**, after Step 0a, and read the reasons. Nothing is locked
automatically, deliberately: a wrong auto-lock silently freezes a part that
needed to move, and that failure is invisible.

```bash
python3 -X utf8 py_placer/place_optimize.py board.kicad_pcb --suggest-locks \
    --suggest-locks-json /tmp/lock_advice.json
```

It reports mounting holes (structurally invisible to the airwire cost, so the
optimizer will happily slide them), parts whose body overhangs the board outline
(card edges, USB shells — the "HAT port" case), and connectors. Each finding
carries its reason and a confidence; the lexical rules (footprint name,
reference prefix) miss house libraries entirely, so treat a *quiet* result as
"nothing detected", not "nothing to lock".

### Step 0c: repair the placement, with those locks

```bash
python3 -X utf8 py_placer/place_optimize.py board.kicad_pcb board_placed.kicad_pcb \
    --max-displacement 3 --length-weight 0.3 --crossing-penalty 30 \
    --halo-coef 0.15 --halo-weight 2 --edge-halo 2 \
    --ignore-nets GND VCC \
    --lock <the exact refs printed by 0b> \
    2>&1 | tee /tmp/step0_place.txt
```

`--max-displacement 3` is the measured sweet spot on both test boards; 10 mm with
strong halos destroyed a data-bus corridor (15 new failures). **A repo-measured
value outranks this cross-board default**: if the repo's README, journal or a
prior run's ledger records a knob measured ON THIS BOARD (run 5's repo said
2 mm where the skill said 3 mm), use the repo's number and cite it — the
skill's defaults are priors, not clauses. `--ignore-nets`
must equal the Step 5 plane-net set — a plane-routed rail's airwire is a fiction
the optimizer would otherwise chase across the board.

**The quench now enforces pad+drill legality by default.** Historically it was
courtyard-rects-only and DRC-blind — measured twice: it walked a diode into a
LOCKED connector's mounting pad at exactly the clearance floor, and dropped
another part onto the first one's *exact vacated coordinates* — and runs had to
revert those moves by hand. The current engine gates every move (swaps
included) on a pad/hole layer that never worsens any pair beyond the input
board and never admits a new different-net pad intersection; NPTH mounting
holes are modelled and **frozen by default** (no connected pins → the airwire
cost cannot see them; `--move-unconnected` frees them deliberately); a
displacement-scaled acceptance (`--min-gain-per-mm`, default 0.1) suppresses
moves whose gain does not pay for their motion. `--courtyard-only` restores
the old model for A/B. The JSON_SUMMARY carries `pad_conflicts_before/after` —
if `after > before`, that is a bug report, not a result.

**The legality gates protect LEGALITY, not STRUCTURE.** Measured, run 3:
both rejected quenches held every hard invariant green (pad conflicts 0→0,
holes 0, oob unchanged, NPTH frozen) while eroding a recovered placement
from 26 parts home to 15 — and the ONLY board-only signal that caught it
was the hpwl acceptance rule, twice. A quench result whose hpwl worsened is
discarded no matter how green its legality block reads.

**Acceptance rule — apply it, do not skip it.** It is a CONJUNCTION, and all
three parts are required:

1. Read the `JSON_SUMMARY:` line from 0c. If `crossings_after > crossings_before`
   or `hpwl_after > hpwl_before`, **discard the result.** **And add a third term the
   quench has no objective for: `check_drc` PAD-PAD must not rise.** Rule 1 as written is
   built entirely out of the quench's own cost function, so it can only measure whether
   the quench succeeded at being a quench. Measured across 29 candidates on one board,
   against distance-to-the-correct-placement: **r(crossings) = +0.780** and
   **r(overlap_area) = +0.723** — *lower crossings goes with a WORSE placement*.
   **Both are measured against distance-to-the-correct-placement, NOT against
   routed `blocking`** (29 candidates, ONE board, run 6). See
   `docs/placement-predictors.md` for what has and has not been correlated with a
   routed outcome. The decision this supports — gate `hpwl`, never `crossings`
   — is unchanged by that correction: it never rested on a routability claim. One
   candidate reached **233 crossings, better than the human original's 276**, while
   sitting 18.7 mm out of position. `hpwl` behaves (its minimum is at the truth) and is
   what actually does the work in this rule. **Gate on `hpwl`, on PAD-PAD DRC, and on
   `check_assembly`'s blocking pairs; report `crossings` and the aggregate
   `overlap_area` and never gate on those two.** (Run 6 measured why the blocking-pair
   COUNT belongs with the gates while the aggregate area never can: hpwl is gameable by
   body-STACKING exactly as PAD-PAD was gameable by evacuation — two parts moved into
   the same space lower hpwl — and the run-5 board shipped that way. The per-pair count
   is 0-calibrated on every healthy corpus board; the aggregate has a nonzero floor.)
2. `check_floorplan --intent` must still **pass**. It did not, once: the quench
   walked a crystal 1.40 mm out of its declared zone while both metrics improved.
   **But an intent emitted from the board under repair INVERTS this rule.**
   `--emit-intent` records the board as it is, so on a misplaced board it writes the
   damage down as the requirement: a displaced connector's position becomes its declared
   edge, and the measured `overlap_area` becomes the `legality_budget`. Measured, one
   emitted intent: it **failed the correct board** (`H1 sits nearest the north edge but
   is declared on the west edge`; `oob 7 exceeds the declared budget 5`) and **passed the
   142 mm² pile-up** with zero violations — so rule 2 would have vetoed the repair and
   blessed the defect. Before trusting rule 2, check the intent was authored or verified
   against something other than the board you are repairing; if it was not, say so and
   fall back to rules 1 and 3.
3. Any repo-local requirement gate must still pass.

**Two numbers produced by the optimizer cannot adjudicate a requirement the
optimizer has no term for.** Measured, one accepted-by-rule-1 placement:
`crossings` 85→60 and `hpwl` 602→596, both "better" — while a decap requirement
went from 2.04 mm to **9.57 mm** because the quench **rotated** the part it served
by 180°. The footprint origin never moved, so a position diff showed nothing and
the delta render drew no arrow.

**Rotation is the trap.** Lock anything whose **pin positions** a requirement
depends on, not merely anything the spec gives a coordinate to. On one board that
added six parts the spec pins nowhere: the flash (a decap-per-pin rule), the
crystal and its load caps (leg length and symmetry), and two series resistors (a
pair's coupled geometry). Expect to pay: `crossings` 85→74 instead of 85→60. That
is the same trade the decap case records — a spec-conformant placement that routes
slightly worse beats a spec-violating one that routes well.

When routing has already failed on congestion, use the loop instead — it consumes
exactly the failed and blocker nets the router reported:

```bash
python3 -X utf8 py_placer/place_route_loop.py board.kicad_pcb board_repaired.kicad_pcb \
    --route-args '--nets "*" "!GND" "!VCC" --clearance <floor> --max-ripup 10' \
    --max-displacement 3 --max-target-pins 40 --ratsnest-screen 20 \
    --lock <refs from 0b> --ignore-nets GND VCC
```

Costly: it re-routes the whole board every round. `--ratsnest-screen 20` buys
some of that back by skipping candidates whose ratsnest clearly regressed.

### Step 0c-bis: when one placement is not enough — the portfolio

The quench is deterministic by design, so re-running Step 0c can never
produce a different arrangement — every run walks into the same local
minimum, and each converged run's measurements tend to get folded back into
the seed as constants, ratcheting the search space smaller. When the right
question is "what are the placement OPTIONS", generate a slate:

```bash
python3 -X utf8 py_placer/place_portfolio.py board.kicad_pcb --out-dir pf --seed 0 \
    --intent floorplan.json --ignore-nets <the Step 5 plane nets> \
    --lock <the 0a/0b locks>
```

**WHEN:** (a) run N+1 of a converged board whose remaining failures were
classified **floorplan-shaped** (9.3d of `/plan-pcb-placement-and-routing`) — the portfolio explores exactly the
axis those findings blame; (b) right after a seeder produced the first
placement, before sinking a full chain into the only arrangement anyone has
ever tried; (c) the user asks for options. It runs on the placed PRE-ROUTE
board (copper → exit 3), and candidate 0 is always the plain quench of the
input — "keep what I have" stays a first-class outcome.

**The acceptance rule generalizes K-way, and stays a conjunction.** The
Step 0c rule (metrics no worse + intent passes + repo gates pass) applies
**per candidate against the baseline row**. What the portfolio enforces as
HARD gates is legality + intent (rule 2); **rule 1 — metrics no worse than
the baseline — is ANNOTATED, not gated**: violators carry `gates.rule1` /
`rule1_violators` in portfolio.json and are excluded from the
JSON_SUMMARY `best` pick (which falls back to the baseline), but they stay
in the rankings.
**Rule 1 is now the hpwl clause only.** Its crossings half was WITHDRAWN by
#789 on the exit criterion `docs/placement-predictors.md` pre-registered for
it: a six-board slate found candidates barred on crossings alone that routed
to strictly lower `blocking` than the baseline, and removing the bar changed
no board's pick on the static order. Read the limit with the result: under a
probe that mis-ranks, the withdrawal CAN pick worse, and the arm that showed
it picking better ranks by the true routed outcome and so cannot show harm by
construction. The direction is still
measured and printed as `RULE1-ADVISORY` / `gates.rule1_advisory`; it simply
no longer bars. `rank_key` still ORDERS on crossings — that is a separate
question, and the same run left it unresolved.
Verify rule 1 YOURSELF against the baseline row before
adopting ANY index (measured, run 7: a slate's routed-best carried
crossings 74 vs baseline 67 with hpwl also worse; only the manual
conjunction check at adoption caught it). The repo-local gates are still
yours to run on the candidate you pick. Among survivors:

1. Prefer `ranking_routed` over `ranking_static` — the probe tier
   (`--route-top`, default: baseline + top 2) routes one SHARED affected-net
   set, so its `failures`/`iterations` compare like with like. Never adopt
   on static rank alone when the budget allows one probe route: crossings
   and hpwl are proxies, and the router is the judge that counts.
   **`ranking_routed: []` with `--route-top >= 1` means the probe tier
   FAILED, not that it was skipped** — read the `[probe]` log lines and fix
   before adopting; empty-routed is never a license to adopt on static rank
   (measured: caller-relative paths broke every probe rc=1 and the slate
   silently degraded to static). And **the shared window compares like with
   like WITHIN the slate; it is not whole-board routability** — it routes
   only the affected nets (run 7: 13–16 of 45). Pruning on it is fine;
   before ADOPTING on it, pass `--full-probe`: it routes the WHOLE board on
   the baseline + the window winner and that verdict outranks the window
   (measured: window-best 14 failures, same board full-probed 52 vs
   baseline 41 — verdict reversed). `probe_kind` in the summary says which
   instrument produced the ranking you are reading.
2. Ties go to the LEAST displacement (the slate is diverse by construction —
   kept candidates sit ≥ `--diversity-mm` apart in pose distance — so a tie
   is a real tie, not two clones).
3. A `backfilled` entry in portfolio.json means the diversity bar was not
   met and that candidate is a near-clone kept only to fill the count —
   weigh it accordingly.

**Reading the slate is ONE image-budget item, not K.** The per-candidate
renders (or the `--montage` grid) answer a single question — which
arrangement — so read them together as one budgeted read, then quote
`portfolio.json`'s numbers, not the pictures, as evidence.

**Adopting a candidate:** copy it out WITH its siblings (`copy_board.py` —
the portfolio writes `.kicad_pro`/`.kicad_dru` next to every candidate), and
it becomes the input to the normal chain. With `--ledger`, every kept
candidate is stored content-addressed with an exact `--only N` replay
command (`converge.py replay` runs it); record your adoption as an
`accepted: true` entry so the chain's provenance survives. Same `--seed` +
same input reproduces the whole portfolio byte for byte.

### Step 0d: see it before trusting it — and this one is ENFORCED

```bash
python3 -X utf8 py_tools/render_placement.py board_placed.kicad_pcb \
    --before board.kicad_pcb --pair \
    --clearance <the board's own floor> --ignore-nets <the poured nets> \
    --expect-moved <what the step reported> \
    --json-out wk/render.json -o wk/render.png
```

**`placement_driver` refuses to open P4, P6 and P-close without
`--render-json`.** It checks the document is of THIS board (`instrument.board`
vs `--board`), carries a `checklist`, and agrees with `--expect-moved`. It
cannot check that you looked; that is still yours, and `converge record
--render-json` is the only trace it leaves.

The reason it is a gate rather than a sentence: a whole placement campaign once
ran with **zero** reads and nothing noticed — not the driver, not the ledger,
not afterwards, because nothing recorded reads either way.

**Use `--pair` after a move, not bare `--before`.** `--before` overlays ghosts
and arrows on one panel and shows what MOVED. `--pair` renders both boards at
identical settings and diffs the findings **by name**, which is what says
whether the move helped:

```
body stacks: 55 -> 46   [9 fixed, 0 NEW, 46 kept]
VERDICT: 29 resolved, none introduced.
overlap mm2: 237.50 -> 239.02   (worse)
```

`46 -> 46` can be nine fixed and nine new somewhere else; only names show that.
And note the last line — every discrete finding improved while the aggregate got
worse. A lap that introduces findings it did not resolve is a lap to revert.

The tool also prints **WHAT THIS PANEL SHOWS** (every finding in words, with its
consequence), **THE WORST N** (one ready `--view` crop command each — run one),
and **DECLUTTER** (`--no-ratsnest` first). `--gate` makes the checklist decide
the exit code; `--focus` now clusters legality findings even with no route
summary, which is the only form of the question available on a copper-free
board.

Ghost rects mark seed positions, arrows show what moved, and the caption strip
carries the real metrics — it wraps rather than clipping, so the trailing fields
(`hole-conflict`, `oob`) are actually present. On a `--view` crop those metrics
are still WHOLE-BOARD, and the caption says so.

**The render is triage, not a verdict.** The verdict is the numbers —
`crossings`/`hpwl` from the `JSON_SUMMARY`, and for the loop `failures` and
`iterations`. Do **not** judge a placement by how much moved: "lots moved, looks
broken" and "barely moved, looks safe" are both wrong.

### Step 0e: declare the floorplan, so it can be checked

A placement judged only by `crossings` and `hpwl` is judged by two numbers that
are **indifferent between a sensible layout and a scattered one with the same
wirelength**. Declaring where parts belong is what makes the rest checkable.

```bash
# read a starter intent OFF the board, then edit it down
python3 -X utf8 py_tools/check_floorplan.py board.kicad_pcb --emit-intent /tmp/intent.json
python3 -X utf8 py_tools/check_floorplan.py board.kicad_pcb --intent /tmp/intent.json --health
```

Exit **0** clean, **4** violations, **3** the board is not in a state it can
grade. Each violation carries the measured number beside the limit it broke, so
`--json` output is quotable evidence rather than an opinion.

**The grading knobs resolve from the BOARD when unset** — the legality rules
inflate part rects by the grading clearance, so a fixed default looser than
the board's floor manufactures phantom oob/overlap findings (run-7 S1: a
phantom 5th oob part, budget 4, exit 4, on a board that grades 0 errors at
its real 0.15/0.3 floor). The JSON_SUMMARY discloses `clearance_used` /
`edge_clearance_used` with their source — quote them with any violation you
report, and pass explicit `--clearance`/`--board-edge-clearance` only when
you mean to grade at a different floor than the board's own.

Worth declaring, in rough order of value: `must_lock` for the parts the lock
advisor flagged; `edge_connectors` for anything that is *meant* to overhang
(this is what stops `oob_count` reporting a card edge as a defect forever);
`keepouts` for mounting holes and antenna clearances; `decaps.max_distance_mm`;
and `blocks` with a `zone` **only where the parts really are one contiguous
area**. Schematic sheets usually are not: on one 4-layer corpus board all ten sheet
bounding boxes overlap each other, so `--emit-intent` claims a zone for only 4 of 10 and
says why for the rest.

A misspelled key is **refused**, at every level of the file and including
`severity` keys — you will be told the key that was wrong and the ones that
were accepted, so fix the spelling rather than assuming the claim landed.
Reasoning about *why* a claim is what it is goes in `context`, which every
entry accepts and nothing grades; do not put it in `note`, which is read for
the substring `SUSPECT`.

`--health` adds the routability signals: how far each block sits from the parts
it connects to, and what crosses each declared bus corridor. Advisory — they say
the floorplan will fight the router, not that it breaks your intent.

### Scope `decaps` to the caps the requirement names, then LOCK them

A spec clause like *"100nF within 3mm of every VDD pin"* names **one BOM line**,
not every capacitor. Read the MPNs off the board and exempt the rest —
bulk electrolytics, crystal loads, a regulator network — in `decaps.exempt`,
citing the line item. That is scoping the rule to what it says, not relaxing it.

Then **lock the caps it does govern**. The quench has **no decap-proximity
term**, so any decap it may move can drift past the limit for a fraction of a
millimetre of wirelength, and it is a different cap every run — measured, one
cap on the first run and two different ones on the next. Locking them one at a time is
whack-a-mole. Their proximity *is* the requirement; their exact position is not
the optimizer's to trade. Expect to pay for it: locking ten decaps there took
crossings from 52 to 60. That is the correct trade, not a regression.

### Board features that live ON the outline

Castellated edge rows, card edges and a USB shell are *meant* to cross the
boundary. Declare them in `must_lock` **and** `edge_connectors` — the second is
what stops `oob_count` reporting them as defects forever.

Four things follow that nothing will tell you:

- **`check_drc` has no castellation exemption.** A track landing on a half-hole
  is flagged `SEGMENT-BOARD-EDGE` and the tool exits non-zero. Do not call that
  board clean — say what the tool reports, then why it is benign, and check the
  flagged coordinates really are on exempt pads before claiming they are.
- **Set `pad_prop_castellated` on the pads — after COUNTING what is already
  there.** KiCad has the property; without it the fab has nothing
  machine-readable saying these half-holes are deliberate, and KiCad's own DRC
  reports pad-outside-outline on every one. Setting it is also what arms the
  retract below. First `grep -c pad_prop_castellated` the board against the
  spec's half-hole count: a seeder may pre-encode the marks, making a repo
  marking script a required no-op — trust the board over the repo README's
  recipe, and a fixup whose recorded usage demands an artifact that no longer
  exists is a stale recipe to flag, not a step to force (run 6's
  `fix_castellated --verify` case).
- **Expect landings at the pad center — which IS the board edge — and let the
  retract fix them.** The router lands on a castellated pad's center, on the
  outline by construction. With the property set, the routing mains' post-pass
  (`pcb_modification.retract_castellated_landings`) pulls each such track end
  back to the pad's inner reach automatically (run 5 hand-trimmed the same two
  landings every lap before this existed). VERIFY before classifying any
  remaining edge flag as benign: check the flagged coordinate is on a
  castellated pad AND the retract ran (its log line names the count) — a flag
  on a non-castellated pad is a real defect, not folklore.
- **A collinear row is not an IC.** `decap_tethers` filters those out now, but
  the reason is worth carrying: a 1×N row spanning a board edge and carrying a
  rail sits nearer to half the decaps than their real IC, and it used to capture
  them — which made a distant decap grade **clean** against the wrong part.

### No impossibility claim without a numeric field

**Never write "no placement can fix this", "this pad is sealed", or "this was
never routable" without a measured number.** Across four recorded runs, **9 of
14** such claims were later refuted — and *every* claim that a pad was
unroutable in principle was wrong. They came from renders, from the router
failing, and from arithmetic on the wrong two edges. A router failing is
evidence about the **router**; only a measurement is evidence about the board.

```bash
python3 -X utf8 py_tools/check_reachability.py board.kicad_pcb --pad U3.23
python3 -X utf8 py_tools/check_reachability.py board.kicad_pcb --net GND --at 142.5,88.1 --json
```

It measures the widest track that can actually reach the rest of the net —
Euclidean distance transform per clearance class, then an exact Kruskal
widest-path — and returns one of two verdicts that mean different things:

| verdict | what it licenses |
|---|---|
| **PASSABLE** at track *t* | a route EXISTS at *t*. If the router failed, the finding is about the ROUTER: grid, ordering, ripup budget, layer pin. Do not touch placement on this evidence. |
| **CAGED** at track *t* | no route exists at ANY grid. This one IS geometry, and placement or the spec is the fix. |

Exit code 0 PASSABLE, 1 CAGED, 2 usage/"nothing to reach". Clearances come from
the board's netclasses, so **quote the clearance with the verdict** — the same
copper is passable at 0.10 mm and caged at 0.20 mm, and a verdict without its
clearance is not a verdict.

Two ways to misread it, both seen:

- **`BOTTLENECK >= …mm` with "bounded by the VIEW"** is not a measurement of
  the board. Nothing was in the way inside the window; the number is the size
  of the question you asked.
- **A margin inside one raster step is not resolved.** The default step is
  0.01 mm. Run 8's decisive case was a **6.4 µm** throat; halve `--step` until
  the verdict stops moving before believing a margin under ~20 µm.

### Before routing a dense escape: is the channel even wide enough?

If a board fans a bus out to an edge, measure the corridor before blaming the
router: `escapes x trace pitch / channel width`. It is the difference between
*"the router failed"* and *"this was never routable"*, and a router you cannot
tell those apart on measures nothing. A spec may set its own gate — one asks for
≤75%, and the as-built channel measured 5.60mm against 2.40mm of escape, 42.9%.

Go one level finer on a dense part: the **per-face lane ledger**. This is a
tool now — do not compute it by hand:

```bash
python3 -X utf8 py_tools/check_floorplan.py board.kicad_pcb --intent fp.json --health
```

Every `--health` run reports it, with no declaration needed, for every
fine-pitch part on the board (`placement/escape.py`; `health_escape_deficit_parts`
and `health_escape_worst_deficit` reach `JSON_SUMMARY`). It counts lanes
SUPPLIED (face span ÷ (track+clearance) **read from the board's own netclass**,
minus span eaten by neighbours) against lanes DEMANDED (nets that must escape
through that face). A face in deficit is the **binding constraint**: reordering
nets only chooses WHICH nets strand there, never how many (run 5 spent multiple
ordering experiments proving this on a face whose ledger would have said it in
seconds).

Read **`blockers` first** — it names the neighbouring parts whose bodies ate the
lanes, which is the move to make. `U9 west: supply 6 < demand 14 … 15.35mm of
that face is taken by SD1` is an instruction; "west face is short" is not.
Interior pads are reported separately and are a **fanout** question, not a lane
one — they need a via, not a channel.

Two caveats before calling a deficit structural: recompute the supply at the
finest legal grid (run 5's v2 bulk ran at 0.05 mm and a lane count at that
pitch understated supply a 0.025 mm pass could reach), and remember the ledger
does not model **supply taps** — a via field feeding the part eats lanes exactly
like signals do, so subtract those by hand before trusting a marginal pass.



---

<agent_identity>
You plan and repair PCB PLACEMENT. You decide with measurements, not with
patterns; you never move a locked part; you never edit the board outline; and
you never route. When the placement is good enough, you hand the board on and
stop.
</agent_identity>
