---
name: plan-pcb-routing
description: Analyzes a KiCad PCB file and creates a comprehensive routing plan. Examines components for fanout needs (BGA/QFN/QFP/PGA), identifies differential pairs, categorizes power/ground nets, and presents a step-by-step routing workflow with explanations.
---

# Plan PCB Routing

When this skill is invoked with a KiCad PCB file, perform a comprehensive analysis and present a routing plan to the user.

## How to run this skill

Do not read this file end to end and improvise the order. Work ONE STAGE AT A
TIME, and let the board decide which stages exist at all: a board with no
fine-pitch parts never runs the fanout stages, a board with no plane nets never
runs the pours. Derive that from the board before you start --

```bash
# Is the board even placed? (report-only, writes nothing, exits 3 if not)
python3 -X utf8 py_placer/place_optimize.py board.kicad_pcb --suggest-locks
```

-- then take the applicable stages in order. **Decide each stage in or out
before you begin, and say which**: "skip if not applicable" read mid-chain is
exactly what gets misread, and a stage silently skipped is indistinguishable
from one that ran clean.

| board state | run placement? | tool |
|---|---|---|
| **unplaced** (test it — see below) | follow the **unplaced ladder** below — seeder, then intent-driven `place_seed.py`, and report-and-stop only when NEITHER applies | see below |
| careful hand placement, routing not yet attempted | **NO** | — |
| routing already completed clean | **NO** — *unless* a spec clause placement could fix is still violated; see the last row of 9.3d | — |
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
python3 -X utf8 py_tools/check_assembly.py board.kicad_pcb --clearance <floor>
echo "EXIT=$?"
```

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
human boards (one corpus layout carries 5.375 mm² of mount-hole-under-shell courtyard
kisses) and run 2 measured it positively correlated with distance-to-truth. The
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

- **False positive.** One corpus board has six holes: four at a `(3.81, 3.81)` corner
  inset and two elsewhere. A "hole off its siblings' pattern" rule flags those two, and
  they are a perfectly ordinary **mid-edge mounting pattern**. Reconstructing that board
  would damage a correct one.
- **False negative, on the board the rule was derived from.** Its four holes sit at
  one 3 mm inset, but its outline is `30.0 – 75.99764`, so nearest-corner insets come out
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

| predictor | what it is | correlation with the human's gap |
|---|---|---|
| **small parts in the corridor** | count, or summed extent | **positive on 8 of 8 distinct boards, r = +0.41 … +0.90** |
| routing cut | nets crossing ÷ layers | ~0, and often negative (−0.48 … +0.08) |
| **the quench's own whitespace term** | `halo_base + halo_coef·sqrt(pin_count)` | **no consistent sign — 5 boards negative, 3 positive** |

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
   python3 -X utf8 py_tools/check_assembly.py board.kicad_pcb --clearance <floor> \
       --baseline <the ORIGINAL input board> --json wk/assembly_lapN.json
   python3 -X utf8 py_tools/check_channels.py board.kicad_pcb --clearance <floor> \
       --track-width <w> --grid-step <g> --json wk/channels_lapN.json
   ```

   `--baseline` is load-bearing: dense healthy boards ship hundreds of by-design
   courtyard kisses (corpus: 235), so the loop's advisory fix-list is the pairs
   NEW relative to the input, never a shipped design's own geometry.

2. **Fix iteration** — one ladder invocation targeting the NAMED findings: blocking
   body pairs and pad/hole conflicts go to `place_reconstruct` (full stages if
   structure moved, `--stages legalize` for local residue — the repair census
   charges body pairs since run 6); NEW advisory pairs and channel intruders go to
   the quench with the ledger's `eaten_by`/pair refs as targets and everything else
   locked. One lap = one ledger entry, recorded before the next begins.

3. **VERIFY, blocking** — a 9.4b boundary verification with **check 5,
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
routing failures as placement-shaped BY MEASUREMENT (9.3d's first mechanical
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
   count in disguise, against 9.4's names-not-counts rule. Recover the full
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
   **r(overlap_area) = +0.723** — *lower crossings goes with a WORSE placement*. One
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
classified **floorplan-shaped** (9.3d) — the portfolio explores exactly the
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
in the rankings. Verify rule 1 YOURSELF against the baseline row before
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

### Step 0d: see it before trusting it

```bash
python3 -X utf8 py_tools/render_placement.py board_placed.kicad_pcb \
    --before board.kicad_pcb -o /tmp/placement_delta.png
```

Ghost rects mark seed positions, arrows show what moved, and the caption strip
carries the real metrics.

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
area**. Schematic sheets usually are not — on one corpus board all ten sheet bounding
boxes overlap each other, so `--emit-intent` claims a zone for only 4 of 10 and
says why for the rest.

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
| `<stage_instructions>` | act on these yourself |
| `<subagent_prompt>` | copy VERBATIM into a subagent. Do NOT read it as your own instructions |
| `<error>` | you skipped evidence. Go produce it; do not improvise around the guard |

An `<error>` means a gate is holding, not that something broke. Every routing
stage refuses without the net-coverage partition, because a net routed by two
stages -- or by none -- is the failure hardest to see afterwards.

The references, and when to open them. Read the one a stage names, when it
names it -- not all of them, and not up front:

| file | read when |
|---|---|
| `../plan-pcb-placement-and-routing/references/evidence-map.md` | you are about to quote a number from any instrument, and need to know which key answers which question |
| `../plan-pcb-placement-and-routing/references/convergence.md` | you are in the convergence loop and need its budget, ledger schema or stop conditions |
| `../plan-pcb-placement-and-routing/references/verifier-prompts.md` | you are dispatching a verification subagent. Quote it whole; it is written FOR the subagent, not for you |

The rest of this file is doctrine: the measurement behind each rule, for when
a stage's instruction is not enough or you need to know why a rule exists
before overriding it.

The rest of this file is the reference the stages point into. Read the part a
stage names, when it names it.

## Step 0: Placement gate — measure first, then decide

Before planning copper, answer one question, and answer it by MEASURING:
**is this board's placement fit to route?**

Routing-only is this skill's normal path, but "normal" is the outcome of the
check, not a reason to skip it. A board whose placement is wrong cannot be
rescued by any router, and the check costs two commands.

| board state | do this |
|---|---|
| placed, and BOTH checks below come back clean | go to Step 1. This is the common outcome -- and it is an outcome, not an assumption |
| board already carries copper | **nothing** -- placement moves footprints, not tracks; the placement tools exit 3 |
| unplaced, or placed WRONG (a copper-free `check_drc` returns violations, or a mechanically-fixed part sits where mechanics forbid) | **stop and invoke `/plan-pcb-placement`**, then start this skill again from its output |
| routing FAILED and the diagnosis is congestion/blockers | invoke `/plan-pcb-placement-and-routing`, which owns the loop |
| routing FAILED and the diagnosis is parameters (grid, ripup budget, layer costs) | fix the parameters here; placement is not the lever |

Test the "placed WRONG" row rather than eyeballing it -- it takes seconds and
it is the one instrument in this step that measures what reaches `blocking`:

```bash
python3 -X utf8 py_tools/check_floorplan.py board.kicad_pcb --intent fp.json --health
```

Every violation a copper-free board returns is a placement defect no router can
remove. Both conjuncts, always: `check_drc` cannot see two parts stacked on the
same net, and `check_assembly` is the channel that can.

**Placement invalidates every downstream routed board.** If placement runs, this
skill starts again from the placed board -- never mid-chain.

### THE BOARD OUTLINE IS NOT YOURS TO CHANGE

Size, shape, cutouts, slots and mounting-hole geometry are mechanical decisions
the user owns: enclosure fit, panel rails, connector apertures.

- **Never resize a board**, and never "just widen it a little". If a board is
  genuinely too small for its parts, **say so in words with the measured number
  and stop.** That is a design decision, not a routing one.
- **Never run** `tests/stress/fix_outline_gaps.py`, `strip_routing.py` or
  `prep_set2.py`. They are corpus-normalization tools and they *do* rewrite
  `Edge.Cuts` — the only things in this repo that do.
- The intent's `envelope` is **read from the board**, never authored. A part
  outside it is a finding about the **part**.
- A part sitting inside a **cutout** is caught by `oob_count` and `oob_amount`,
  never by `oob_area` — that one is measured against the bounding-box inset and
  scores a part in a slot as `0.0`. `check_floorplan` refuses it as a budget
  key, with that reason.

This mirrors the rules you already follow for user-owned geometry: guide
corridors and keepout polygons are described in words and drawn by the user, and
the stackup is never edited directly.

### Which artifact to produce, and what to check it against

**Never read a picture on its own.** Every render is paired with a number that
either confirms or contradicts it, and the number wins. A render that looks
tidier while `crossings` went up is a worse placement that photographs well.

**And at a BOUNDARY, never read the numbers on their own either** (run-23).
The key-pairing discipline has a measured failure mode: keys become the ONLY
channel, and a reviewer with "overlap 26.30mm2" printed in the image banner
read past it because the keys had already said clean — the board shipped with
15 courtyard interpenetrations and four mid-board connectors a human rejected
at a glance. At each boundary (post-placement close, the L2 handoff, after
the FIRST route lap, close-out): produce `render_placement --review-sheet`,
VIEW it **blind-first** — write your observations (connectors vs edges with
distances, density pockets vs empty regions, orientation coherence, anything
wrong that no key names) BEFORE reading any checklist key — then write a
reconciliation paragraph dispositioning each observation against a named
number, or refuse the boundary. The L2 and close-out gates refuse without the
written review (`review_handoff.md` / `review_close.md`); the post-first-route
review is yours to keep honest, and it is the cheapest of the four — the
first route lap is where a placement-shaped failure is still one cheap
re-entry away.

Full key-by-key map in
[`../plan-pcb-placement-and-routing/references/evidence-map.md`](../plan-pcb-placement-and-routing/references/evidence-map.md) — read it before
quoting any number. The headline pairings:

| after this step | produce | and CHECK it against |
|---|---|---|
| 0b lock advice | *(none)* | `JSON_SUMMARY` `unlocked_high` — re-run with your `--lock` list until it is **0**, or say which findings you are deliberately leaving free and why |
| 0c `place_optimize` | `render_placement.py placed --before seed -o delta.png` | `JSON_SUMMARY` `crossings_after` vs `crossings_before`, `hpwl_after` vs `hpwl_before`. **Both must improve or you discard the result.** The arrows show what moved; only these say whether it helped |
| 0c on a two-sided board | add `--per-side` | `overlap_area` — a per-side panel is the only place a back-side collision is visible, and `overlap_area > 0` tells you one exists before you go looking |
| chasing one bus / clock | add `--ratsnest-nets '/CLK*'` | the same crossings/hpwl pair — and on a POSE/ORDER decision this is a READ, not a show: two renders, one per candidate pose, side by side (image case 5) |
| a `place_route_loop` round | `make_movie.py WORKDIR --camera auto` | per-round `failures` and `iterations` from the loop's own output. A round that moved a lot and changed neither is noise |
| a run that TRIED more than it kept | `make_film.py --from-loop-dir WORKDIR` | the same per-round numbers, for the **rejected** rounds too — the badged beats are the ones whose `failures` did not improve, and seeing where the search went is the point |
| routing failed after placement | `--summary-json <route log>` on the render | the `failed_nets` and `blockers` in that same summary — the render colours exactly those, so the picture and the diagnosis are the same data |
| board looks wrong / empty | `render_placement.py board -o state.png --json` | the `unplaced` key in its JSON. The renderer deliberately WARNS where the placement CLIs refuse (seeing an unplaced board is the point of a renderer), so **it always exits 0** — the old "exit 3" advice here pointed at a code that never fires |
| **any board you are about to call done** | `../plan-pcb-placement-and-routing/scripts/board_score.py board --intent I --json wk/score.json` | `blocking` — it must be **0**. `ungraded` lists what nothing examined; that is *unexamined*, not clean. This is the one number not produced by the thing being graded |

### Which flag, at which step — the trigger table

Producing a render at the wrong moment, or without the flag that answers the
question you actually have, is the same as not producing it. Each row is a
**trigger**: when the left column happens, run that command *then*.

| when this happens | run | because it answers |
|---|---|---|
| before authoring an intent, board has back-side parts | `render_placement --per-side` | you cannot declare zones for a side you have not seen. Pairs with `overlap_area` |
| any accepted placement change | `render_placement after --before <the ledger's parent board — step-back its parent_sha>` | did the macro structure survive? **`--before` is the last ACCEPTED board**, not iteration N−1 — N−1 renders a delta that never existed |
| any route step failed | `render_placement board --summary-json <route log> --focus` | do the failures share one pocket (→ placement) or scatter (→ parameters)? **`--focus` emits nothing without `--summary-json`** |
| a `--group-by` decision is live | `render_placement --zoom-group <name> --group-by sheet` | which parts does this block actually pull in? |
| chasing one bus, pair or clock | `render_placement --ratsnest-nets '*USB*'` | route.py `--nets` glob syntax, exclusions included. On a pose/order decision: TWO renders, one per pose, and READ them (image case 5) |
| a claim about ONE spot (an intrusion, an edge row, a wedge, a stop claim) | `route_render.py BOARD --view x0,y0,x1,y1` | the question-scoped crop, self-describing: rect label, ref designators cross-marked at their JSON origins, mm ruler on the edges (`--refs`/`--ruler` default ON for crops — **route_render.py flags; `render_placement --view` crops too but has NEITHER**, so use route_render for self-describing crops). READ it (image case 6). Numbers still decide magnitudes |
| `check_drc` failed | re-run with `--render wk/drcN/` | one panel per violation cluster: red rings, ref labels + mm ruler for JSON matching, count/types/rect caption. READ them (image case 7): one cluster = local fix, board-wide = grading floor (9.1b) |
| every placement render | add `--ignore-nets <same as place_optimize>` | **must match** or `crossings`/`hpwl` will not reproduce the optimizer's `JSON_SUMMARY`, and you will chase a phantom disagreement |
| every placement render | add `--clearance <the board's real floor>` | halo and overlap are otherwise graded at the wrong gap |
| every render, always | add `--json-out wk/renderN.json` | the re-measurement channel, to a FILE (run-4 G1; the bare `--json` flag still prints the `JSON_SUMMARY:` stdout line). The document carries an `instrument` block (board/before/clearance/ignore_nets/size) so a before/after series is PROVABLY same-instrument — run 3's watcher could not verify 632-vs-412 crossings on one board because neither JSON said which `--ignore-nets` produced it — and a `checklist` block naming refs per mandate-8 question, channel-labelled (pad_copper vs courtyard: they legitimately differ and run 3 lost time to exactly that unlabelled disagreement) |
| with `--focus` | `-o` names a **DIRECTORY** | `render_placement --focus -o wk/x.png` writes `wk/x.png/<board>.png` and `wk/x.png/<board>_focus1.png`. Give it a directory name, and read the panel paths back out of the `panels` array |
| once, before choosing a budget | `route.py --list-groups --group-by auto` | whether the board decomposes at all. The budget is **100 per board** either way (9.2) |
| after each accepted placement | `check_floorplan --intent I --health` | will this floorplan *fight* the router? Block displacement and bus-corridor crossings |
| every Step 9 iteration | `check_floorplan --intent I --json` | the per-rule measurements the ledger records |
| every Step 9 iteration | `board_score.py --json` | the only number that decides better/worse |

**Not evidence:** `--size` and `--supersample` change how the picture looks, not
what is true — measure instead. `--ratsnest-all` is the deliberate hairball, for
showing a human, never for reading.

**Worked render recipes (run-4 G2 — copy these, so the same-instrument habit
is copied rather than remembered).** The `--clearance`/`--ignore-nets` values
are the board's routed floor and the plane-net set (run 3's pair was
`--clearance 0.09 --ignore-nets GND +3V3`); a series whose flags differ is
not a series, and the JSON's `instrument` block is what proves it:

```bash
# per-side (any board with back-side parts; sibling FILES x_F.png/x_B.png)
python3 -X utf8 py_tools/render_placement.py board.kicad_pcb --per-side \
    --clearance 0.09 --ignore-nets GND +3V3 \
    --json-out wk/r_state.json -o wk/r_state.png

# mandate-5 POSE PAIR: two renders, one per candidate pose, read side by side
python3 -X utf8 py_tools/render_placement.py cand_rot0.kicad_pcb \
    --ratsnest-nets 'QSPI_*' --clearance 0.09 --ignore-nets GND +3V3 \
    --json-out wk/pose0.json -o wk/pose0.png
python3 -X utf8 py_tools/render_placement.py cand_rot180.kicad_pcb \
    --ratsnest-nets 'QSPI_*' --clearance 0.09 --ignore-nets GND +3V3 \
    --json-out wk/pose180.json -o wk/pose180.png

# one block, framed (block names exactly as route.py --group takes them)
python3 -X utf8 py_tools/render_placement.py board.kicad_pcb \
    --zoom-group sheet:58d913ec --clearance 0.09 --ignore-nets GND +3V3 \
    --json-out wk/blk.json -o wk/blk.png
```

### LOOK at the render — you, not just the user

Renders are for **intent**; numbers are for **legality**. A render answers *"is
this the structure I meant?"* — bus corridors, block cohesion, connector
orientation, which pocket the failures sit in. It never answers *"is this
legal?"*: clearance, overlap, off-board, connectivity and DRC all come from
numbers. **Do not adjudicate clearance from pixels.**

**`Read` the PNG yourself, and say what you saw, in exactly these eight cases.
These are MANDATES tied to triggers, not permissions** — run 5 had this list as
four permissions and read **zero** images across the whole run: the placement
delta was produced and never opened, the R10-in-the-corridor fact that a crop
shows instantly was found by a subagent rebuilding geometry from coordinates,
and the U3 pin-order flip cost fifteen ordering experiments that two ratsnest
renders would have replaced. An unread mandated image is a skipped step, and
the ledger entry must name the panels read (see 9.4).

1. **Before writing an intent** — `--per-side` on any board with back-side
   parts. You cannot declare zones for a board you have not looked at.
2. **After any accepted placement change** — the delta against the board it
   actually came from. One question only: *did the macro structure survive?*
   Two cases the bare rule leaves undefined, and both get skipped because of
   it: for a seeder's FIRST output the parent is a PILE at one coordinate,
   so the delta is all-arrows noise and the question has no referent — read
   a plain `render_placement` of the seed instead. And a RUN of micro-moves
   (a sub-millimetre re-seat, then another, then another) still owes one
   read: render the CUMULATIVE delta against the last board you read, rather
   than skipping each move on the reasoning that one part cannot change the
   macro structure — that reasoning is exactly what the read exists to test.
   Record the read, or record the non-trigger, every time.
3. **Every Step 9 iteration whose score has `unrouted` or `broken` > 0** —
   `render_placement --summary-json wk/routeN.json --focus -o wk/focusN/`.
   One question: *do the failures share one pocket* (→ placement) *or are
   they scattered* (→ parameters)? This is 9.1a's classification made
   visual; do it BEFORE picking the lever, not after three levers failed.
4. **When a block decision is live** — `--zoom-group`.
5. **Any pose or ordering decision on a bus** (a `converge poses` candidate,
   a rot-0-vs-rot-180 tie, an escape-order experiment) — TWO
   `--ratsnest-nets '<bus>*'` renders, one per candidate pose, read side by
   side. The pin-order fan is directly visible; count the crossings you see
   and check them against `components.inversions`. Run 5's measured
   exchange rate: fifteen routing experiments for what two renders show.
6. **Before any stop-3/stop-4 claim** — a `--view x0,y0,x1,y1` crop of the
   claimed-blocked region (route_render or render_placement, both take it).
   A crop self-describes: its rect in the corner label, **reference
   designators cross-marked at their exact origins, and a mm ruler on the
   edges** — so everything seen matches the JSON that cites refs and
   coordinates by name. Read it YOURSELF, then hand it to the watcher with
   the pad coordinates. A
   "boxed in" claim whose crop shows open copper is refuted before it
   costs a report.
7. **When `check_drc` fails** — re-run with `--render wk/drcN/` and read the
   cluster panels (red rings at each violation, count/types/rect in the
   caption). One question: *is this one cluster* (a local fix — a rip, a
   nudge, a retract) *or board-wide* (a grading-floor or class problem,
   9.1b)?
8. **After EVERY placement tool step — accepted, rejected, or probed** —
   `render_placement` the output with `--json-out` (the legality overlay is
   default-ON: red rings per pad/hole conflict, orange NPTH keepout
   circles, dashed-red extents on off-board copper) and read it against a
   fixed checklist, all four answered in writing: *(a) any part off the
   outline? (b) any part-on-part overlap? (c) any part on a hole or a
   locked part? (d) did more parts move than the step claimed?* **QUOTE
   the JSON's `checklist.a`..`d` blocks as the four answers** (run-4 G5:
   they name the refs, channel-labelled, and `d` compares `--expect-moved
   N` against the measured move count) AND say what you saw in the pixels
   — the numbers stay the verdict; the eyes catch what no metric
   models. A run once read ONE image across
   an entire placement campaign and missed off-board parts a render plainly
   showed; the checklist, not the glance, is what catches them. The caption's
   `pad-conflicts` / `hole-conflict` numbers pair with what you see — the
   numbers stay the verdict.

### Which of these a tool ENFORCES, and which are on you

The eight above are doctrine. Only some are gated, and knowing which is the
difference between a rule and a hope:

| mandate | enforced by | what happens if you skip it |
|---|---|---|
| 2, 8 — after a placement move | `placement_driver` **P4** refuses without `--render-json` | `<error>`, exit 4, no instructions |
| 1 — before writing an intent | `placement_driver` **P6** refuses | `<error>`, exit 4 |
| the close-out | `placement_driver` **P-close** refuses | `<error>`, exit 4 |
| the ROUTING close-out | `loop_driver` **L5** refuses without `--score`, and its close-out dispatches the three lens verifiers (../plan-pcb-placement-and-routing/references/verifier-prompts.md) until every `--lens` verdict is in hand | `<error>`, exit 4 |
| shipping | `loop_driver` **L5** refuses without a `check_complete` close-out, and refuses again when it CONTRADICTS `converge` | `<error>`, exit 4 |
| recording the verdicts | `converge record --final` refuses without the three routed-board lenses, and forbids stop-condition 1 when any FAILED | exit 2, nothing written |
| recording the read | `converge record --kind placement` NOTEs when `--render-json` is absent | a warning; the row is still written |
| 3 (partially) | `loop_driver` **L4** refuses a grid/parameter lever when `--route-summary` reports the failing nets boxed in by STATIC obstacles at the board's declared floor | `<error>`, exit 4, unless `--accept-boxin "<reason>"` |
| 4, 5, 6, 7 | **nothing** | silence |

The gate checks that a render EXISTS, is of **this** board (`instrument.board`
vs `--board`), carries a `checklist`, and agrees with `--expect-moved`. It
**cannot** check that you looked at the pixels. That part is still yours, and
the only trace it leaves is `--render-json` in the ledger — which is why the
`[read: …]` convention moved from free-text into a field.

### The render tells you what it sees — read that too

`render_placement` prints three blocks unless `--no-describe`:

- **WHAT THIS PANEL SHOWS** — every finding in words, with the consequence
  attached (off-board pad copper: *"their nets cannot be routed at all"*; a body
  stack: *"not a clearance graze; no router can fix one"*; a locked part:
  *"the OTHER part must move"*).
- **THE WORST N, framed one per crop** — ranked by severity, each with a ready
  `--view` command. Run one. Clustering everything implicated does not work on a
  dense board — single-linkage merges 54 parts into one board-sized "pocket" —
  so these frame ONE finding each.
- **DECLUTTER** — `--no-ratsnest` first, it is the biggest source of noise.

**`--pair` is the one to reach for after a move.** `--before` alone overlays
ghosts and arrows on one panel and tells you what MOVED; `--pair` renders both
boards at identical instrument settings and diffs the findings **by name**:

```
body stacks: 55 -> 46   [9 fixed, 0 NEW, 46 kept]
VERDICT: 29 resolved, none introduced.
overlap mm2: 237.50 -> 239.02   (worse)
```

Names, not counts, for the same reason the ledger records failing nets by name:
`46 -> 46` can be nine fixed and nine new somewhere else. And note the last row —
every discrete finding improved while the aggregate got worse; a single scalar
would have picked one and hidden the other.

**`--gate` makes the checklist decide the exit code** (4 when anything is off the
outline, overlapping, hole-conflicting, or disagreeing with `--expect-moved`).
Default stays 0: seeing a broken board is the renderer's job.

**Two things the caption cannot do for you.** On a `--view` crop the metrics are
still WHOLE-BOARD — the strip says so, but read it as the board's totals, not the
crop's. And there is no colour key: red rings = pad/hole conflict, solid red =
body stack, orange = NPTH keepout, dashed red = off-board pad extent, yellow =
move vectors, hatched = KiCad-locked.

**Show without reading:** the movie, `--ratsnest-all` hairballs, full panel
dumps. Those are for the human. Budget **≤3 images read per turn**, crops
count as cheap — pick by the question you have, not by what is available. The
budget bounds curiosity, never a mandate: a turn whose triggers demand four
reads takes four.

### Always produce the movie — it is the only artifact that shows *how*

`place_route_loop` renders it **by default** (`--no-movie` opts out). Every other
artifact is a snapshot: the movie is the only one that shows which round moved
what, and what the router did with the room it was given. Do not make the user
ask for it.

When the chain was **not** a `place_route_loop` run — a hand chain of
`place_optimize` → `route` → `route_planes` → repair, which has no
`loop_round*.json` sidecars and so no camera — build it from the step boards you
already wrote, in order. They are cumulative, which is exactly what the animator
wants:

```bash
python3 -X utf8 py_router/make_movie.py placed.kicad_pcb r3.kicad_pcb r4.kicad_pcb r5.kicad_pcb \
    -o wk/routing.gif --size 1600 --fps 12 --chunks 30 --end-hold 12
```

`.mp4` needs `imageio` + `imageio-ffmpeg` and falls back to a sibling `.gif`
without them — ask for `.gif` directly if you know they are missing, rather than
letting the fallback surprise you. Hand it to the user with `SendUserFile`; do
not `Read` it — it is a show-without-reading artifact, and its frames would blow
the ≤3 budget for nothing.

A part move is animated over **at least 10 frames** (`--tween`, default 10,
floored at `movie_camera.MIN_MOVE_FRAMES`) — below that it reads as a jump cut
and you cannot see *which* part went *where*, which is the only reason the beat
is in the film. `--camera-budget` may squeeze a pan to nothing but never a move.
`--tween 0` is still an explicit "cut straight there, no glide".

**Pass `--camera auto` on a chain that placed anything.** A placement step
changes no copper, and the movie animates copper deltas, so without it the step
that decides everything downstream renders as a **single frame** — measured:
seed → placed, 14 parts moved, one frame. The camera used to need the loop's
sidecars; it now recovers the moves from the boards themselves when there are
none, so a hand chain gets the same animation the loop does.

**A Step 9 convergence produces one film, not N disconnected ones.** Each
iteration may render its own movie; the artifact the user wants is the whole
convergence, and the ledger already holds its frame list — **the accepted boards,
in order**:

```bash
python3 -X utf8 py_router/make_movie.py \
    wk/iter00.kicad_pcb wk/iter01.kicad_pcb wk/iter04.kicad_pcb wk/iter07.kicad_pcb \
    -o wk/convergence.gif --size 1600 --fps 12 --chunks 30 --end-hold 12
```

Feed it the **accepted** boards only. A reverted iteration spliced into *this*
sequence animates a change that was undone, which reads as the router thrashing
when it was doing the opposite.

**The attempts are a second film, not a looser cut of this one** — and they are
usually the more interesting artifact, because the accepted spine is a small
fraction of what a run actually tried (one run: 10 boards of the 45 on disk).
`make_film.py` composes the whole search in one pass: the placements animate,
every attempt is shown and then explicitly reverted with a red **TRIED** badge
so it reads as a search rather than as churn, and the diagnostic renders that
were the *input* to each decision are spliced in as cards where they were made.

```bash
# a place_route_loop run: every round it wrote, kept AND dropped
python3 -X utf8 py_tools/make_film.py --from-loop-dir wk/ -o wk/film.gif --size 1200

# a converge.py run: beats captioned with the lever_argv that produced them
python3 -X utf8 py_tools/make_film.py --from-ledger wk/ledger.jsonl -o wk/film.gif

# a hand chain: name the dead ends, point it at the renders
python3 -X utf8 py_tools/make_film.py wk/seed.kicad_pcb wk/placed.kicad_pcb \
    'wk/r*.kicad_pcb' --reject 'r4[bcd]*' --cards-from wk/ \
    -o wk/film.gif --size 1200 --fps 8
```

`--accepted-only` gives back the convergence cut. Produce both when a run had
attempts worth seeing; produce the convergence one always.

A render can never establish: that routing will now succeed (only a re-route
shows that); that the placement improved (`crossings`/`hpwl` decide); that a
part is or is not in violation (`overlap_area`/`oob_count` decide); or anything
at a coordinate outside that panel's `view` rect.

Two things the picture cannot show you at all: `board_edge_contours` (milled
inner contours the router keeps clearance from) are **not drawn**, and a board
whose outline failed to chain renders as a clean **rectangle**. Both are visible
only in `check_floorplan`'s `outline` block.

### Verify, do not assume

- **`failed_single` is HALF the answer — read `failed_multipoint` too, and read
  EVERY `JSON_SUMMARY` in the log, not the last one.** A net can fail as
  multipoint while the single-ended bucket is empty, and route.py's in-run
  reconciliation prints a SECOND summary whose buckets differ from the first.
  Measured: a call printed `routed_single: ["QSPI_SD1"], failed_single: []` and
  wrote a board with **0 segments** on that net — which reads exactly like the
  "routers report false success" hazard and was **not** one. The reconciliation
  had re-routed `QSPI_SD2` and reported breaking `QSPI_SD1` in
  `failed_multipoint`, the field the chain was not grepping. A grep of one bucket
  turns an honestly-reported failure into a silent one, and then into a wrong bug
  report against the engine. Grep both, from every summary line:

  ```bash
  grep -oE '"routed_single": \[[^]]*\]|"failed_single": \[[^]]*\]|"failed_multipoint": \[[^]]*' run.log
  ```

- **YOUR OWN CHECKS ARE INSTRUMENTS TOO, and they fail the same way.** Every
  rule above is about a tool that can fail two ways and reports one. The checks
  you write to test those tools have exactly that shape, and they are *easier*
  to get wrong, because a check's failure path is the path nobody looks at.

  Measured in a single session, all five reporting "the guard held" or "the
  feature is absent" when neither was true:

  | what I ran | what it actually did | what I concluded |
  |---|---|---|
  | a negative control copied to a temp dir | died on `ModuleNotFoundError`, exit 1 | "the gate refused" |
  | a probe calling `check_connectivity(...)` | that function does not exist | "the branch never fires" |
  | evidence passed as `<(echo '{...}')` | fd gone before a Windows child opened it | "the stage never mentions it" |
  | an `awk` section splitter | matched the first of two `===== L5 =====` | "zero render mentions" |
  | `--deadline 0` must yield a partial | a board with no work correctly completes | "the deadline is broken" |

  Four rules, and the first one is most of it:

  1. **A non-zero exit is not evidence.** Assert the REASON. `tests/run_utils.py`
     has `check(argv, refuse='<the reason>', code=N)`, which reports an
     `ImportError`/traceback/argparse accident as a **BROKEN TEST** rather than
     as a satisfied guard. Use it instead of `assert r.returncode == 2`.
  2. **Verify the input before trusting the output.** `run_utils.evidence(path)`
     refuses a path that is not a real non-empty file. A check whose input is
     missing tests nothing — and process substitution is not a file on Windows.
  3. **Test both directions.** "It refuses when X" is half a test; "it accepts
     when not-X" is the half that catches a gate wedged shut. The deadline case
     above was a spec error, and only the accepting direction exposed it.
  4. **When a check reports something surprising, suspect the check first.**
     Every one of the five looked like a real finding. The tell is always the
     same: a result that would require the code to be broken in a way you have
     no other evidence for.

- **THE SILENT-TIMEOUT FAMILY. Learn its signature, because five different
  instruments share it and none of them says the word "timeout" where you look.**
  A long quiet phase after a complete-looking report; an exit code belonging to
  the **shell** rather than the tool; and staged output that leaves nothing at
  the output path on a kill. Measured members:

  | where | limit | on expiry | what you see |
  |---|---|---|---|
  | `place_seed` (plain seed and `--repair/--reseat`) | `--deadline` | stops between parts/violators, keeps the seats | `JSON_SUMMARY` `status: deadline`, exit `7` |
  | `route.py` | `--deadline` | stops between nets, writes the copper it has | the same |
  | `route_diff.py` / `route_planes.py` / `repair_planes.py` / `place_reconstruct` | **none** (upstream #621 removed their `--deadline`; passing it is an argparse error) | runs to completion or an external kill | shell `124`/`143`, no output, no `JSON_SUMMARY` |
  | *any budget-capable step WITHOUT `--deadline`* | none | runs forever | shell `124`, no output, no `JSON_SUMMARY` |
  | `EXACT_FILL_TIMEOUT` (`kicad_exact_fill.py`) | 300 s | returns `None` | ONE misattributed line |
  | `ORACLE_DRC_TIMEOUT` (`kicad_oracle.py`) | 240 s | `None`, and **memoises the board so every later step skips the oracle too** | nothing at all |
  | `converge record` argv | ~32 kB | never execs | shell `126`, **no ledger row** |

  The last three degrade to a fallback with no failure signal and no effect on
  the exit code. Two consequences you must build into how you run:

  1. **Pass `--deadline` on any budget-capable step with an external timeout**
     (`route.py`, `place_seed` — see the table above for the
     tools that lost it), set well below
     it (~0.8×). It is the only mechanism that works: on Windows a harness kill
     is `TerminateProcess`, which no handler, `atexit` or flush can catch, so
     the tool must stop ITSELF. **`route.py` has it too now** — run 11
     lost a full lap because `route.py` did not, and only a re-parse of the
     output board established that the engine had in fact finished. A log with
     no `DEADLINE:` line is proof no budget was set. `KRT_DEADLINE_S` budgets a
     whole chain with one export. Note the cancel is cooperative — measured
     109 s against a 45 s budget — so it guarantees TERMINATION, not a
     wall-clock cap.

     **Read `complete` before you read any tally.** A partial run's
     `JSON_SUMMARY` parses perfectly and its numbers are not a whole board's;
     `place_route_loop` already refuses one, and a hand-read must too
     (`.get('complete', True)`, so pre-deadline logs are unaffected).
  2. **Check the row count after every `converge.py record`.** The 126 is the
     shell's, so a caller that does not re-count sees no error and the lap is
     gone. Prefer `--score-file` over `--score "$(cat …)"`.

- **A DELEGATED HALF CANNOT BLOCK ON A DETACHED PROCESS. Hand back, do not
  wait.** Both routing teammates in run 20 returned mid-route with "still
  working, I'll wait" — because they could not: a subagent has no way to sit on
  a backgrounded step and be woken when it exits. The orchestrator had to arm
  the wait itself and resume the teammate twice, and each of those round trips
  cost a re-read of the whole context.

  So a half that launches a detached step **returns**, with three things and
  nothing else:

  | | |
  |---|---|
  | the LOG PATH | where the step is writing, so the parent can watch it |
  | the COMPLETION MARKER | the file whose appearance means "done" (`--done`), or the exact string to wait for in the log |
  | what to do with the result | which stage or command consumes it when it lands |

  The parent arms the wait (a background `run_watch.py timer --done <marker>`,
  or its own blocking read) and RESUMES the half when it fires. That is a
  handback, not a failure, and it must not be reported as one: a half that says
  "I have launched X, here is where it lands, wake me" has done its job
  correctly.

  Two things that make this work and are easy to drop:

  * **The marker must be written by the STEP, not by the half.** A half that
    writes its own "done" file before returning has told the parent the work
    finished when it has not started.
  * **Pair it with `--deadline` where the tool takes one** (`route.py`,
    `place_seed`; the other mains lost the flag to #621 — bound
    those by scope). A handback whose step never terminates turns
    one blocked agent into two, and the parent's wait has no more information
    than the half's did.

  A `'NoneType' object has no attribute 'items'` from the plane fragility field
  was, for a long time, the *caller's* own `AttributeError` printed where the
  diagnosis should be — the real reason (a 300 s fill timeout) was computed and
  discarded. If a message names a Python type error, suspect that the instrument
  is reporting its own bug rather than the board's.

- **The routable denominator is ON-BOARD pads.** `board_score` counts a net
  routable at ≥2 pads; the router's own `net_queries.filter_routable_nets`
  requires ≥2 pads **on the board**. Measured on one board: 147 vs 149, and the
  two nets in the gap (2 pads, 1 on-board each) appear in `unrouted` forever
  while no router could ever route them. Before reporting an unrouted net as a
  routing failure, check it has two pads the router can reach.

  **This is a family, not a route.py quirk**: ANY truncated or piped read of
  ANY instrument — an exit code through a pipe, a `grep | tail -N` of a long
  log, a score payload copied forward between iterations — will eventually
  read success where the instrument printed failure. Read the structured tail
  (JSON_SUMMARY / the EXIT line / the final tally block) or parse it; never
  conclude from a truncated grep. Run 6 logged three incidents on three
  different instruments read three different lossy ways; the instrument was
  right every time. **An oracle LISTING gets the same rule**: never `head`/
  `tail` it — consume the whole list and assert the lines you consumed equal
  the count the instrument itself printed. `check_drc` now prints a
  machine-checkable `LISTING: M of T violation(s) shown` line (run-4 B2):
  quote a specific item only when `M == T`, else re-run `--max-print 0`
  first — run 3's orphan incident read 1 of 3 off a tail and shipped the
  wrong count into a ledger entry. Run 7's close-out shipped "5
  opens" off a `tail -5` of a list whose sixth line (SWDIO) sat above the
  window; the oracle had printed 6, and the wrong number reached the final
  report and the promotion decision before the watcher caught it.
- **Before concluding a clause is geometrically unsatisfiable, check your own
  flags.** One net was reported "walled in by its neighbours" when the actual
  blocker was a `--track-width 0.4` the analyst had passed: a 0.4 mm trace cannot
  leave a 0.200 mm QFN pad, and that net carried **no HARD width clause at all**
  — the 0.4 came from the netclass nominal, not the spec. It routed at 0.16 with
  no other change. Confirm the requirement is real (`--net-min-widths` is the
  list of what the SPEC demands) and that you are not asking for something the
  spec never did, before writing it up as stop condition 4.
- **Re-read the `JSON_SUMMARY` line you just produced.** Do not carry a number
  forward from an earlier step or from memory of what you expected.
- **VERIFY THE WIDTH LANDED.** `--track-width` and `--power-nets-widths` are
  *requests*. A wide route that will not fit is necked down, and the per-net
  rescue re-routes a failed net at the **fab floor** — both leave `failed_single`
  empty and print one easily-missed line in a long log. Measure the copper
  instead, after every width-bearing step:

  ```python
  from collections import Counter
  from kicad_parser import parse_kicad_pcb
  pcb = parse_kicad_pcb('out.kicad_pcb')
  print(Counter(round(s.width, 4) for s in pcb.segments))    # widths ACTUALLY emitted
  ```

  There is **no flag that makes the net fail instead of going under** --
  `--track-width-floor` was REMOVED in 53a5a16e and nothing replaced it. So
  score with `board_score.py --net-min-widths`, which is now the only way a
  per-net width requirement is graded rather than hoped for, and pin the fab
  floor with `--fab-overrides` so the tier cannot step down beneath it.
- **Carry the `.kicad_dru` with the `.kicad_pro`.** Rule lookup is
  `splitext(board)[0] + ".kicad_dru"`, strictly per board stem, so every
  intermediate board needs its own. `copy_board.py` takes every sibling; a hand
  `cp` of two files does not.
- **`place_optimize.py` writes NO project sibling at all** — only the
  `.kicad_pcb`. The next step then reads no project and resolves its floor from
  the stock netclass, which is the exact failure the `copy_board.py` warning
  exists to prevent, arriving through a door nothing guards. Copy the
  `.kicad_pro` (and `.kicad_dru`) onto the placed board yourself.
- **A `.kicad_dru` rule outside the two honored grammars is enforced by nothing
  in this chain.** Layer-scoped rules are auto-read (#498), and (#549) so is the
  track-channel grammar `A.NetClass=='X' && B.NetClass!='X' && A.Type=='track'
  && B.Type=='track'` — the router enforces it as a hard, per-layer, seg-vs-seg
  obstacle expansion (pass order decides who pays it), and `check_drc` grades it
  pair-exact, tagging rule-governed pairs as `segment-segment-track-rule`
  (board_score reports them as `advisory`, gated by a repo's registered floors,
  not by `blocking`). Anything else — an area scope, a mixed condition — is
  skipped with a note saying KiCad will still enforce it: true in KiCad, false
  for `check_drc.py` and the router. Without `kicad-cli` installed, such a rule
  is graded by **nobody**, and the score cannot even list it as `ungraded`
  because it never knew about it. Say so explicitly rather than letting the rule
  read as covered.
- **A tool's own report does not satisfy its own gate.** `place_optimize` says
  the placement improved; confirm it on a *different channel* by running
  `render_placement.py <the written output> --json` and checking `metrics.
  crossings` and `metrics.hpwl` reproduce it. That is what catches a writer that
  dropped something between the objective and the file.
- **A render proves nothing about connectivity or DRC.** Those come from
  `check_connected.py` and `check_drc.py`, graded at the clearance the board was
  actually routed to.
- **After ANY placement change, every downstream routed board is stale.** Re-run
  the chain from the placed board. Do not reuse a routed artifact from before it.
- **If two sources disagree, believe the JSON.** The picture is a summary of it.
- **`0 violations` and `0 rules ran` are different.** `check_floorplan` reports
  `rules_run` and `rules_skipped` precisely so you can tell them apart; quote
  both.

### Verify with independent subagents, when you can

For anything beyond a single obvious call, **fan out verifiers in ONE response**
and hand each only its slice of the round's evidence — never the raw
`.kicad_pcb`. **Every verifier WRITES its verdict to
`wk/turn<N>/verdict_<lens>.txt` before returning, and you read the FILES,
never the replies** (run-23: two lens verdicts delivered as completion
notifications were lost outright; the parent waited on them, was nudged, and
re-derived both inline. A verdict file that never appears is itself the
finding — the verifier died — not a reason to assume PASS). The prompts are in
[`../plan-pcb-placement-and-routing/references/verifier-prompts.md`](../plan-pcb-placement-and-routing/references/verifier-prompts.md). Nine lenses —
six grade the **placement** (`intent`, `legality`, `delta`, `blocks`,
`routing-feedback`, `coverage`) and three grade the **routed board**
(`connectivity`, `drc`, `spec`).

**Run the `spec` lens ONCE AT THE START, before the chain is built.** It is the
only step in this procedure that walks the requirements document and asks *"what
will measure this?"* of every clause — and a clause nothing models is **invisible**,
not `ungraded`. `score.json#/ungraded` lists components that know they did not run;
a requirement no component represents never appears there at all. Two HARD clauses
were shipped unmeasured that way: a plane-continuity rule that no checker
implements, and three fiducials that were never in the netlist, so no routing step
could have noticed they were absent. Running `spec` early tells you which clauses
you must measure by hand, in time to build that into the chain instead of
discovering it after delivery.

**Then run `connectivity`, `drc` and `spec` again on every board you are about to
call done.** They are the gate, not the write-up: a `VERDICT=FAIL` means `blocking`
was not really zero, so it **re-enters the Step 9 loop** at the step named in
`route=` while budget remains (100 per board). Do not re-word a FAIL into a caveat — "complete,
with some DRC warnings" describes a board that did not pass.

Each returns exactly one machine-readable line:

```
VERDICT=PASS:lens=<lens>
VERDICT=FAIL:lens=<lens>;finding=<one line>;evidence=<path#json-pointer|path@x,y>;route=<step>
```

**A verifier that cannot fill `evidence=` has not verified anything.** The gate
is met when every lens passes or every finding is dispositioned in writing.

`VERDICT=`, **not** `RESULT=`. The GUI takes the **last** `RESULT=` line in a
reply and parses it as the plan JSON, so a verdict spelled that way would be
read as a malformed plan.

**If the Agent tool is unavailable** — the GUI's headless runs allow only
`Read,Glob,Grep,Bash,WebSearch` — run the identical lenses yourself, in the same
order, on the same inputs, tag each `mode=inline`, and **say in the report that
verification was single-agent**. A run must never look like a fan-out happened
when it did not.

**When two instruments disagree, suspect the PASSING one first.** Before
accepting "checker A flags it, checker B is clean, so it's A's noise", open the
passing checker and confirm it implements the clause **in code** — run 5 found
a gate whose docstring promised a check (PCB15/24 pour continuity) that nothing
in the body implemented, so its pass was vacuous while the other instrument's
284 flags were real. A disagreement you reconcile counts as a **systemic**
iteration in the ledger: the budget went to the instrument, not the board, and
the status warning that watches the systemic share must see it.

### The watcher pattern: refute the stop before you write it

On ANY stop-3/stop-4 claim — "irreducible crossing", "unsatisfiable",
"capacity-bound" — fan out one independent agent BEFORE the claim enters the
report. Hand it only the pad coordinates and the claim, and instruct it to
rebuild the geometry from those coordinates and try to REFUTE the claim, not
to review your reasoning. Run 5's watcher did exactly this twice in both
directions: it refuted an "irreducible SD1×SS crossing" by finding the R10.y
variable the claim had frozen, and it CONFIRMED the homotopy floors on the
QSPI lengths by recomputing them (16.91/18.89 routed vs 16.81/18.48 floors).
A stop condition that has survived one hostile rebuild is a finding; one that
has not is a hypothesis.

Pair the coordinates with a picture — image read-case 6: render a
`--view x0,y0,x1,y1` crop of the claimed-blocked region (route_render.py or
render_placement.py, both take the flag; the crop labels itself with its
rect), READ it yourself first, and hand it to the watcher beside the numbers.
Run 5's refuted claim — R10 astride SD1's corridor — is fully visible in a
4×6 mm crop; the watcher found it only by re-deriving the geometry from pad
coordinates. The crop shows WHERE; the watcher's arithmetic still decides
HOW MUCH (never clearance from pixels).

**Prefer the field over the rebuild where one exists.** For "sealed",
"unreachable", "no path at any grid", `check_reachability.py --pad REF.NUM`
is the hostile rebuild, done exactly and in seconds. Run 8's watcher refuted a
sealed-pad claim with a hand-derived 6.4 µm throat; the field agreed to within
a micron and also confirmed the one genuinely caged pad. Give the watcher the
tool's output *and* the coordinates — a claim that survives a numeric field is
much stronger than one that survives an argument.

**Watch the argv, not only the ledger.** A watcher brief that covers ledger
discipline will not catch a *dropped* flag, because nothing in the ledger is
missing — the recorded command is simply the command that ran. Run 8 lost 12
plane-void crossings to a `--layers F.Cu` pin omitted from one scoped pass, and
no audit caught it: the entry was complete, consistent and wrong. When a pass
is scoped (a layer pin, a width, a net subset), diff its argv against the pass
it was derived from and say in the ledger which flags were **deliberately**
dropped.

### Good and bad, concretely

**Do**

- Default to **not** running placement. The measured verdict is that the quench
  is a repair tool for rough/generated placements, not a polish pass — on a good
  hand placement it was neutral at best and its default weights caused 2 new
  routing failures. (Exploring OPTIONS is different from polishing: when the
  question is "which arrangement", `place_portfolio.py` generates a diverse
  slate without touching the default path — Step 0c-bis.)
- Run the lock advisor **before** the first placement run, and read the reasons
  rather than pasting the list blind.
- Keep `--max-displacement` at ~3 mm. It is the dominant safety knob; 10 mm with
  strong halos destroyed a data-bus corridor (15 new failures). A value the
  repo measured on this board (README/journal/ledger) outranks the ~3 mm.
- Pass `--ignore-nets` equal to the plane-net set, so the optimizer does not
  chase a plane-routed rail's airwire across the board.
- Show the render **and** quote the caption metrics when reporting to the user.

**Do not**

- Judge a placement by how much moved. "Lots moved, looks broken" and "barely
  moved, looks safe" are both wrong — this is the single most common misreading.
- Run placement mid-chain, between routing steps.
- Pass `--allow-unplaced` or `--allow-routed` to make an error go away. They exist
  for a human who has read the message and decided; not to unblock a script.
- Add `--group*` to a command unless the user asked to scope to a block. A
  routing run that silently acquires a scope routes a fraction of the board and
  reports success on that fraction.
- Auto-lock anything the advisor merely suggested at `low` confidence.
- Present a render as evidence that routing will now succeed. Only a re-route
  shows that.

### The inner loop — and why its verdict is not the verdict

`place_route_loop` is the router-in-the-loop form: it routes, reads the failure
diagnostics, moves only the parts implicated, re-routes, and keeps the result
**only if `(failures, iterations)` improved**. Rejected rounds revert and widen
the search. Each round costs a full re-route (minutes); `--ratsnest-screen 20`
skips the route for candidates whose ratsnest clearly regressed, buying some back.

**Its `ACCEPTED` / `REJECTED` is the ROUTER'S OWN OPINION, not a quality
verdict.** `better()` (`place_route_loop.py:358-362`) compares `failures` and
`iterations`, and `metrics_from_summary` (`:224`) reads both out of route.py's
`JSON_SUMMARY`. **No checker runs.** CLAUDE.md states the hazard directly —
*"Routers can report false success… re-verify with the authoritative,
zone/fill-aware check"* — so a round can be ACCEPTED with pads disconnected and
DRC dirty. Two consequences:

- **It is a cheap pre-filter, not a gate.** Re-score every board it hands you
  with `../plan-pcb-placement-and-routing/scripts/board_score.py` before believing it improved anything.
- **Its move candidates come ONLY from failed nets, so on a board where nothing
fails it does nothing at all.** `metrics_from_summary` builds the target set from
`failed_single` + `failed_multipoint` and from the `blockers` diagnostics — all of
which are empty when every net routed. A board whose only blocker is a spec
clause therefore hands the loop an empty target list: it is not that it moves the
wrong parts, it never moves any, and the run looks like placement was "tried".
Two flags together fix it, and neither is sufficient alone:

```bash
# --target-nets = WHAT to move around (the clause's nets, routed or not)
# --accept-cmd  = HOW to tell better from worse (the comparator cannot see a
#                 length or a width; your judge can)
place_route_loop.py in.kicad_pcb out.kicad_pcb \
    --target-nets QSPI_SD0 QSPI_SD3 \
    --accept-cmd 'python3 judge.py' \
    --route-args '...' --max-displacement 3
```

`--target-nets` names nets to treat as targets even though the router routed
them; `--accept-cmd` replaces the failures-then-iterations comparator, which
cannot see a length or a width. One supplies the targets, the other the
gradient.

**It is also only ONE `route.py` call** (`_ROUTE_PY`, `:52`), not the chain.
  Planes, plane-repair, reconnect and diff pairs never participate in its
  feedback, so a board that needs a plane repair to connect cannot converge
  inside it. That is what the **Step 9** outer loop is for.

`--rounds` (default **5**) bounds this inner loop. The Step 9 budget (100 ledger
entries per board) bounds the outer one. They are different budgets; do not confuse
raising `--rounds` with taking another outer iteration.

**Full convergence procedure: [`../plan-pcb-placement-and-routing/references/convergence.md`](../plan-pcb-placement-and-routing/references/convergence.md).**

### Placement is CLI-only

There is no placement tab and no `place` plan action, so a placement step
**cannot** ride in a plan's `steps`. Run it on the command line *before* the
plan and hand the plan the placed board. `make_plan.py` / `manifest_to_plan.py`
**refuse** a recorded `place_optimize.py` / `place_route_loop.py` command loudly
rather than convert it.

### Do we need blocks at all, and which ones? — a procedure

**G0. The default is no blocks.** `--group-by` defaults to `none` on the
placement CLIs and `route.py --group` is unset. **Do not add `--group*` to a
plan or a command unless the user asked for it** — a routing run that silently
acquires a scope routes a fraction of the board and reports success on that
fraction.

**G1. Name the job first.** Three different jobs want three different sources:

| the job | the tool | the source that works |
|---|---|---|
| move parts together | `place_optimize` / `place_route_loop --group-by` | `decap`, and realistically only `decap` |
| scope a route or an undo | `route.py --group` + `--group-scope` | `sheet` (or `kicad` if it exists) |
| frame a picture, or a zone in an intent | `render_placement --zoom-group`, `check_floorplan` | `sheet` |

**G2. List before deciding.** Both are report-only, exit 0, write nothing:

```bash
python3 -X utf8 py_router/route.py board.kicad_pcb --list-groups --group-by auto
python3 -X utf8 py_tools/render_placement.py board.kicad_pcb --list-groups --group-by sheet
```

**G3. Choose on the measured evidence, in this order.**

1. `kicad` — the designer's own `(group ...)`. Exact when present, but on **0 of
   27** in-repo boards. If it fires, trust it and stop looking.
2. `sheet` — **12 of 22** boards with sheet paths have more than one. The
   workhorse **for scoping and framing**. **Not for movement**: sheet blocks of
   16–83 parts moved on no board tried. Also not for zones — a sheet is a
   *functional* grouping, so its members scatter and its bounding box overlaps
   its neighbours' (on one corpus board all 10 do).
3. `decap` — the only source that measurably *moves* anything. **0% internal by
   construction** (a cap bridges VCC and GND, both board-spanning), so it is
   meaningless as a routing scope.
4. `netprefix` — weakest. Enable expecting little.

**G4. Pick the scope deliberately.** `--group-scope` defaults **depend on the
operation**: `touching` when routing (routing a block's interface is the point),
`internal` with `--undo`, because a block's touching set contains GND/VCC and
undoing those strips copper board-wide (rp2350: 170 segments vs 75).

**G5. Confirm it did something.** Read `blocks` and `block_parts` from the
`JSON_SUMMARY` and the `describe()` banner. **`blocks: 0` means drop the flag**,
not "it helped". And if a round's `groups` pulled a large block in, the run moved
far more than you targeted.

- Always list first: `python3 py_router/route.py board.kicad_pcb --list-groups --group-by auto`
  (prints parts and touching/internal net counts, exits 0, routes nothing).
- Which source: `kicad` groups exist on **0 of 27** in-repo boards; `sheet` is the
  workhorse (**12 of 22**); `netprefix` is weakest; `decap` is strong but **0%
  internal by construction**.
- `--group-scope`: routing defaults to `touching` (routing a block's interface is
  the point); `--undo` defaults to `internal`, because a block's touching set
  contains GND/VCC and undoing those strips copper **board-wide** (measured on
  rp2350: 170 segments vs 75).
- For placement, `--group-by decap` is the one that measurably moves anything.
  **Sheet blocks of 16-83 parts moved on no board tried** — don't burn a run
  discovering that.
- **Hard rule:** `route.py --group` is a *scope*. A routing run that silently
  acquires one routes a fraction of the board and reports success on that
  fraction — the same class of defect as a Step 5b coverage gap.

## Step 1: Load and Analyze PCB Structure

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')

# Basic stats
print(f'Total nets: {len(pcb.nets)}')
print(f'Total footprints: {len(pcb.footprints)}')
print(f'Existing segments: {len(pcb.segments)}')
print(f'Existing vias: {len(pcb.vias)}')
```

Report to user:
- Number of nets, components, existing routing
- Whether this is a fresh board or partially routed

## Step 2: Identify Copper Layers

Check the KiCad file directly for layer definitions:

```bash
grep -E "^\s+\([0-9]+ \".*\.Cu\"" path/to/file.kicad_pcb
```

Report to user:
- Available copper layers (F.Cu, B.Cu, In1.Cu, In2.Cu, etc.)
- Whether it's a 2-layer, 4-layer, or multi-layer board

### Stackup Check (always run this early)

Inspect the stackup now, before planning, and report the verdict **at the top of the
plan report** so problems surface before any routing work:

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')
for layer in pcb.board_info.stackup:  # List[StackupLayer], ordered top to bottom
    print(layer.name, layer.layer_type, layer.thickness, layer.epsilon_r)
```

- No stackup section, or all dielectrics with identical thickness and ε_r ≈ 4.5, means
  KiCad's untouched default. If the board also has impedance-relevant signals (see the
  speed detection in Step 4), lead the report with a clear warning: impedance and
  time-matching calculations will not match the user's fab, and `/recommend-stackup`
  should be run before impedance-controlled routing. Take plane-layer assignments from
  its output when available.
- A 2-layer board with multiple differential pairs or planes-worth of power nets is
  itself worth flagging (no inner layers for reference planes).
- If the stackup looks deliberate, say so in one line and move on.

Report problems prominently but still produce the full plan - the user decides whether
to fix the stackup first.

## Step 3: Check for Components Needing Fanout

Identify BGA, QFN, QFP, PGA, LGA, and other array packages that benefit from escape routing:

```python
for ref, fp in pcb.footprints.items():
    name_upper = fp.footprint_name.upper()
    pad_count = len(fp.pads)

    # Check for array / fine-pitch land/no-lead packages by name. Note 'QFP'
    # already matches LQFP/TQFP/VQFP, 'QFN' matches VQFN/WQFN/HVQFN, and 'BGA'
    # matches FBGA/UFBGA/TFBGA, so only distinct families need listing.
    needs_fanout = any(k in name_upper for k in (
        'BGA',          # ball grid array
        'PGA',          # pin grid array (through-hole)
        'LGA',          # land grid array (interior lands, e.g. LGA-12) - issue #144
        'CSP', 'WLCSP', 'WLP',  # wafer-level / chip-scale = micro-BGA, sub-0.5mm
        'CGA',          # column grid array
        'QFN', 'DFN',   # quad / dual no-lead (exposed-pad)
        'QFP',          # quad flat pack
    ))

    # SMD vs through-hole FIRST -- it gates everything below (#513 item 16).
    smd_count = sum(1 for p in fp.pads if p.drill == 0)
    th_count = sum(1 for p in fp.pads if p.drill > 0)
    mostly_tht = th_count > smd_count

    # A THT part's pins are reachable on EVERY copper layer -- there is no
    # "escape" problem to solve, so fanout buys nothing regardless of pad
    # count. PLCC/DIP/ZIF SOCKETS are the trap: a PLCC-44 THT socket's
    # staggered double-ring reads as a sparse uniform grid and used to be
    # misdetected as a BGA (rc2014_82c55_ide U1 -- nets near it burned >1M
    # A* iterations each behind a phantom exclusion zone, #513 item 16).
    # Wide-pitch (>=2mm) PGAs route fine without fanout too; only reach for
    # bga_fanout on a PGA when its channels are genuinely contested.
    if mostly_tht and 'PGA' not in name_upper:
        needs_fanout = False

    # Fine-pitch arrays strand even at low pad count: trigger by PITCH + interior
    # pads, not just pad_count > 40 (issue #144: LGA-12 at 0.5mm has only 12 pads
    # but its center lands box in). Compute the min pad-to-pad spacing and whether
    # any pad is interior (not on the bounding-box edge).
    if not needs_fanout and not mostly_tht and pad_count >= 6:
        xs = sorted({round(p.local_x, 3) for p in fp.pads})
        ys = sorted({round(p.local_y, 3) for p in fp.pads})
        def _min_step(v):
            return min((b - a for a, b in zip(v, v[1:])), default=999)
        pitch = min(_min_step(xs), _min_step(ys))
        minx, maxx, miny, maxy = xs[0], xs[-1], ys[0], ys[-1]
        has_interior = any(minx < round(p.local_x, 3) < maxx and
                           miny < round(p.local_y, 3) < maxy for p in fp.pads)
        # Fine pitch (<=0.6mm) with interior pads, OR a large multi-row part
        # AT FINE PITCH. Raw pad_count > 40 alone is NOT a fanout signal: a
        # 44-pin THT socket, a 2x20 header, or a 1.27mm connector trips it
        # while gaining nothing from escape routing.
        if (pitch <= 0.6 and has_interior) or (pad_count > 40 and pitch <= 0.8):
            needs_fanout = True

    if needs_fanout:
        # Analyze pad arrangement
        xs = sorted(set(round(p.local_x, 2) for p in fp.pads))
        ys = sorted(set(round(p.local_y, 2) for p in fp.pads))
        grid_cols, grid_rows = len(xs), len(ys)
```

### Does this part actually BENEFIT from fanout? (check before planning it)

A name/pad-count match is a candidate, not a decision. Fanout (escape routing)
exists to solve ONE problem: pads that cannot be reached by ordinary routing
because neighboring pads at fine pitch box them in. Before adding a fanout
step, confirm the geometry actually has that problem:

1. **Through-hole part (most pads drilled)?** → **No fanout.** Every pin is
   reachable on every layer; there is nothing to escape. This includes
   PLCC/DIP/ZIF **sockets** (a PLCC-44 THT socket's staggered pin field looks
   like a sparse grid but is just a socket, #513 item 16), headers, and DIN /
   backplane connectors. Wide-pitch (>=2mm) PGAs also normally route fine
   without fanout.
2. **Wide-pitch SMD (>=1.27mm) perimeter part?** → No fanout; plain routing
   handles it.
3. **Interior pads at fine pitch (<=0.6mm), or a perimeter at <=0.65mm with
   many pads?** → Yes, fanout genuinely helps (this is the boxed-in case).
   Dense 2-row mezzanine/card-edge connectors at 0.4mm (CM4/CM5, 200+ pads)
   DO benefit -- use `qfn_fanout.py --escape-method underpad --allow-via-in-pad`.
4. **Unsure?** The fanout tools now refuse or warn on wrong shapes
   (staggered arrays, non-arrays). Trust a refusal: if the tool says the part
   is not an array and the geometry checks above say the pins are reachable,
   plan ordinary routing instead of forcing a workaround.

### Fanout Tool Selection

| Package Type | Tool | Notes |
|--------------|------|-------|
| BGA (SMD grid) | `bga_fanout.py` | Escape routing for ball grid arrays |
| PGA (through-hole grid) | `bga_fanout.py` | Same tool works for PGA |
| LGA / WLCSP / CGA (land/chip-scale grid) | `bga_fanout.py` | Grid escape; interior lands strand without it (issue #144) |
| QFN/QFP/DFN (perimeter SMD) | `qfn_fanout.py` | Stub routing for quad/dual no-lead and flat packages |
| **AQFN / staggered multi-row no-lead** | `qfn_fanout.py` **`--escape-method underpad --allow-via-in-pad`** | Inner rows the surface fan cannot reach - see below. **Never `bga_fanout.py`** |
| DIP/SOIC (through-hole/SMD rows) | None needed | Standard routing handles these |
| PLCC (SMD J-lead or THT socket) | None needed | Perimeter part; the THT socket's pins reach every layer. Never a BGA (#513 item 16) |
| Sockets / headers / backplane connectors (THT) | None needed | All-layer reachable; pad count alone is not a fanout signal |

### When to Use Fanout for BGA/PGA/LGA

**Rule: Use fanout for any grid array (BGA/PGA/LGA/WLCSP/CGA) with more than 2 pins
depth from outside to center, OR any fine-pitch (<=0.5mm) array with interior pads
regardless of pin count** — a small LGA-12/WLCSP at 0.5mm pitch boxes its center
lands in even though it has well under 40 pads (issue #144).

**Important:** Calculate ACTUAL depth by counting pads from the edge toward center, not grid size.
Many PGA/BGA packages (especially FPGAs/CPLDs) have hollow centers with only perimeter pins populated.

To calculate actual depth:
```python
# Check middle column from top edge toward center
mid_col = xs[len(xs)//2]
depth = 0
for y in ys:  # ys sorted from edge
    if (mid_col, y) in pad_positions:
        depth += 1
    else:
        break  # Stop at first empty position
```

Examples:
- 13×13 grid, fully populated → depth = 7 → **USE FANOUT**
- 13×13 grid, hollow center (3 rows populated) → depth = 3 → **USE FANOUT**
- 10×10 grid, hollow center (2 rows populated) → depth = 2 → fanout optional
- 4×4 grid, fully populated → depth = 2 → fanout optional

Inner pins beyond depth 2 cannot escape without fanout routing through channels between outer pins.

**Escape layers (multi-layer boards):** `bga_fanout.py` defaults to `--layers F.Cu B.Cu`
only. On a 4+ layer board, pass ALL the board's copper layers, e.g.
`--layers F.Cu In1.Cu In2.Cu B.Cu` — otherwise deep balls have nowhere to escape to
and those nets are dropped from the fanout. `qfn_fanout.py` is perimeter-only and
doesn't take escape layers.

**Staggered multi-row no-lead packages (AQFN) - use via-in-pad (#500).** An
AQFN (e.g. `Nordic_AQFN-73-1EP_7x7mm_P0.5mm`, on osprey_kb / hex_gateway /
mikoto_nrf52840) puts its pads in TWO OR MORE staggered rows per side. The
surface 45-degree stub fan reaches only the outermost row, so the default
silently drops the rest. Measured on osprey_kb U1 (78 pads, 39 nets):

| command | escaped | time |
|---|---|---|
| `qfn_fanout.py` (default stub) | 26/40 | 2.4s |
| `qfn_fanout.py --escape-method underpad` | 35/40 | 2.6s |
| **`qfn_fanout.py --escape-method underpad --allow-via-in-pad`** | **39/39, DRC-clean** | **2.4s** |
| `bga_fanout.py` | 39/39 | **2967s** |

So: **for any AQFN or staggered multi-row no-lead part, plan
`qfn_fanout.py --escape-method underpad --allow-via-in-pad`.** Via-in-pad is
what reaches the innermost row; without it 5 pads drop.

Do NOT send these to `bga_fanout.py`. It models a ball grid, and a staggered
package's two offset rows project onto each axis at HALF the real pad spacing -
so its detected pitch is half the truth, its escape budget evaluates to a
NEGATIVE via size, and it grinds for ~50 minutes to reach the same answer.
`bga_fanout.py` now refuses these outright with the qfn_fanout command to use
(override: `KICAD_ALLOW_STAGGERED_BGA=1`).

Spotting one: the footprint name contains `AQFN`, or the part has far more pads
than a single peripheral ring of its size would hold (73-90 pads on a 7x7mm
body), or `bga_fanout.py` reports a pitch that is half the name's `P<pitch>mm`.

**Crowded fine-pitch QFN edge (surface fan has no room):** if a `qfn_fanout`
stub (especially a diff pair) is boxed in by a neighbour pair and a foreign
track and the surface 45° fan drops it, use `qfn_fanout.py --escape-method
underpad --via-size 0.45 --via-drill 0.25` (#164). It drops a through-via just
past each pad and escapes on an inner/back layer — straight out past the lateral
congestion instead of fanning into it (adjacent vias are staggered to clear).
Match `--via-size`/`--via-drill` to the board's fine-pitch via rule. If the
underpad run still **drops** a leg ("N dropped") because the via has no clear
room *outward* (a neighbour pad/track exactly one pitch away), add
`--allow-via-in-pad` (#161): the escape via may then sit on its own pad and
stagger *inward toward the chip*, away from the neighbour, instead of being
dropped. It still clears every other-net pad/via/track — it only gains
permission to overlap its own pad — so reach for it specifically when underpad
reports drops on a boxed-in fine-pitch pair.

**Size the escape via/track to the pitch BEFORE running fanout (issue #158).**
`bga_fanout.py` escapes one track down the channel between adjacent via columns —
at the **half-pitch**. So the via, track, and clearance must fit that half-pitch
or *every* escape grazes the neighbouring column's via by a few µm, and the fanout
still reports `failed: 0` (its success metric ignores sub-clearance grazes). The
budget, per array (measure each component's own pitch — they differ):

```
via_size + track_width + 2·clearance + margin ≤ pitch     (one escape track per channel)
via_size ≥ via_drill + 2·min_annular_ring,  track_width ≥ fab min track   (fab floors)
```

Don't just shrink the via against a *fixed* track — **solve for via AND track
together**, taking each down toward the fab floor as the pitch demands, and leave
a little margin so the result clears DRC instead of merely touching it. Read each
array's own ball pitch `P` (the min ball spacing — arrays on one board differ) and
the requested clearance `C` (Default net-class clearance from
`list_nets.py --design-rules`), plus the board's fab floors (`min_track_width`,
`min_via_diameter`/`min_via_drill`, annular ring), then:

```python
margin = 0.05                                  # slack: clear DRC, don't graze it
budget = P - 2*C - margin                       # room for one via + one track
track  = max(min(nominal_track, 0.15), min_track_width)   # keep a routable track
via    = min(nominal_via, budget - track)       # largest via that still fits
if via < via_floor:                # via fell below the floor -> thin the track to free room
    via   = via_floor
    track = max(min_track_width, budget - via)
infeasible = track < min_track_width or via < via_floor   # even fab floors won't fit
via_drill  = max(min_via_drill, via - 2*min_annular_ring)  # hold the annular ring at floor
# via_floor = max(min_via_diameter, min_via_drill + 2*min_annular_ring)
```

Pass the computed `--via-size via --via-drill via_drill --track-width track
--clearance C` to the fanout step. If `infeasible`, the pitch can't take a channel
escape even at the fab floor → for a POPULATED array prefer `--escape-method dogbone` (it never escapes fewer balls than underpad and matches the human idiom; this supersedes older underpad advice), else `--escape-method underpad`, and/or add
escape layers; don't ship the graze.

**Plan params can set ANY GUI option:** in the GUI's RESULT schema, each
step's `params` may include any option shown on that step's tab or the shared
options panel, keyed by its snake_case field name (`max_iterations`,
`max_ripup`, `grid_step`, `board_edge_clearance`, `hole_to_hole_clearance`,
`via_cost`, `heuristic_weight`, `turn_cost`, `ordering_strategy`, ...).
Unknown names are ignored with a note in the plan log. Use this to carry the
same values the equivalent CLI chain would pass (e.g. `--max-ripup 5
--grid-step 0.05`), so a GUI plan run matches a stress run step for step.
(Leave `max_iterations` at its default — the engine self-budgets, #529.)

**Why this heuristic matters for the GUI:** the plugin runs `/plan-pcb-routing` in
*plan-only* mode — it never executes the fanout and never runs the DRC↔smaller-via
retry loop, so it cannot discover a too-big via after the fact and shrink it. The
plan must therefore carry via/track that are **already** DRC-safe for the pitch.
Computing them here — both dimensions, with margin, clamped to the fab floor — is
what lets the single fanout the GUI runs come out clean the first time.

Worked example (a 256-ball 0.8 mm-pitch BGA, clearance 0.1, fab floor track 0.1 / via 0.45):
`budget = 0.8 − 0.2 − 0.05 = 0.55`; track 0.127 → via = min(working, 0.55−0.127) =
**0.42** (≥ floor) → DRC-clean, vs the Ø0.5 the net-class default would have used
(163 grazes). At 0.4 mm pitch the budget forces both to the floor (track 0.10, via
~0.30/0.20 advanced); if even those don't fit, go `--escape-method dogbone`
(populated array; `underpad` only when no inter-ball gap site exists at all).
`bga_fanout.py` also warns `WARNING: escape via ... busts the half-pitch budget`
when handed infeasible params, but choose feasible ones here so it never fires.

**Always check the fanout escaped all requested balls.** `bga_fanout.py` ends with
`JSON_SUMMARY: {"component", "requested", "escaped", "failed", "unescaped_nets", ...}`.
A dropped ball is **removed from the output** and later fails signal routing as "no
rippable blockers", so it must be caught here. If `failed > 0`, retry the fanout with
more layers and/or a smaller `--clearance` (see "Escape clearance" below) before
moving on — do not start signal routing while balls are still dropped.

**If balls still drop on a dense, fully-populated array, switch to the dog-bone
escape:** add `--escape-method dogbone` with a small via/track for the pitch
(e.g. `--via-size 0.35 --track-width 0.12 --clearance 0.1` at 0.8 mm pitch). The
`channel` engine confines every layer to the gaps *between* ball rows, so a few
channels over-subscribe and the deepest balls can't escape; `dogbone` stubs each
ball to a via in the diagonal inter-ball gap (falling back per-ball to
via-in-pad), so it never escapes fewer balls than `underpad` at roughly half the
via-in-pad / IPC-4761 fab burden (#669: orangecrab U3 108/108 vs underpad's
104/108, with the 3 stranded balls unrecoverable by ANY later routing).
`underpad` (every via in its pad) is for arrays with **no legal inter-ball gap
at all** — WLCSP-class pitches where even a floor via busts the half-pitch lane
budget. Caveats (both grid escapes): diff pairs route **single-ended**, and
power/plane nets are skipped as escapes — but every skipped plane ball still
gets a **plane-drop via** (below), so nothing is left stranded. `auto` (the
default) retries channel's drops with `underpad` only — a dogbone-first retry
was measured and REJECTED as the default (#669 sets1-5 corpus A/B: +10
incomplete nets, +59 kicad DRC — dogbone gap vias claim inter-ball streets
that chains not authored for dogbone then collide with). So on a populated
array, `dogbone` must be passed EXPLICITLY, with via/track/clearance chosen
for it — which is exactly what this plan does.

**How humans escape big BGAs — and which of OUR tool options that maps to**
(survey of 54 human corpus boards with a real ≥100-ball array; the fanout
places vias itself, so this is about choosing its options, not via positions):

- **Dog-bone is the dominant human method at every pitch** (median 30–43% of
  balls; via-in-pad is ~0% on most boards, appearing only on a handful of very
  dense 6/8-layer designs). Roughly HALF of all balls get no via at all — the
  outer rings escape on the surface, rails connect into pours. Mapping: for a
  populated array prefer **`--escape-method dogbone`** — each ball vias in a
  free inter-ball gap and falls back per-ball to via-in-pad, so it never
  escapes fewer balls than `underpad` while keeping the inner-layer streets
  open. `channel` (`auto`'s first pass) already leaves the outer rings
  via-free; keep it for sparse/perimeter-heavy arrays and diff pairs. Note
  `auto`'s retry for channel's drops is `underpad`, NOT dogbone (#669 measured
  a dogbone-first retry worse as a default) — so a populated array gets
  dogbone only by passing `--escape-method dogbone` explicitly, with params
  chosen for it.
- **Rail balls under a pour need NO via when the pour is on their own layer**
  — the plane-drop pass (#424) detects this automatically when the pours
  already exist (the Step 1 pour runs before fanout): it prints `N pour-covered (no via
  needed)` and skips those vias (measured: 104 of 127 GND balls on a 285-ball
  BGA, ~100 via barrels kept out of the escape field;
  `KICAD_FANOUT_POUR_DIRECT=0` reverts). This is why Step 1 pours before fanout;
  put rail pours on the layers that carry the rail balls (the outer layer for
  a surface flood). This generalizes beyond BGAs: **choose each outer-layer
  flood net by same-layer SMD pad count** — every SMD pad of the flood net on
  that layer connects by fill contact with no via at all. `list_nets.py
  --power` prints per-net `(F.Cu n SMD, B.Cu n SMD, TH n)` for this choice;
  ignore the TH counts (barrels connect on every layer regardless).
- **Escape via, by pitch:** at 0.8–1.0 mm the median minimum via in the
  courtyard is 0.45/0.20; at ≤0.5 mm humans go to 0.28/0.15 and even
  0.25/0.10. Escape-track minimum: median 0.125 mm at coarse pitch, 0.089–0.10
  (the fab floor) at fine pitch. The computed budget-per-pitch above lands in
  the same range — trust it, and treat 0.25/0.15 as the floor for ≤0.5 mm.
- **Deep balls leave through the inners, not the surface.** Inner-layer share
  of courtyard copper: ~0–15% on 4-layer boards, 30–67% on 6/8-layer. Mapping:
  give the fanout the FULL `--layers` list, and keep the escape-depth inner
  layers ROUTABLE — on a 6-layer board that means at most ONE solid inner
  plane next to each outer (fine-pitch-BGA humans keep a median of ONE solid
  plane total; **pouring 2–3 solid inner planes on a 6-layer BGA board is the
  classic self-inflicted failure** — it leaves signals a 2-layer board).
  Rails beyond GND go as SPLIT region pours or late route+pour, not extra
  solid planes and not wide tracks.
- **Buses concentrate.** A RAM/DDR bus runs on ONE inner "highway" layer with
  a solid GND plane adjacent (plus the outers). Mapping: pick the highway
  layer at plan time, keep its `--layer-costs` at 1.0–1.5, and put the solid
  GND plane on the layer NEXT to it.

**Plane-net balls are dropped to vias automatically (#424).** With any escape
method, after the signal escape each SMD ball on a plane net — a net excluded
from the fanout with ≥ 6 balls on the part, or an excluded net that already
owns a copper zone — gets a via immediately: a dog-bone via in a free
inter-ball gap, else a via-in-pad tap. The Step 1 pour — run BEFORE fanout and
before any routing — picks these vias up at fill while the pour is
still intact, which kills the tap-behind-the-ball-wall failure class (#360)
and, with the default-on plane-fragility field, keeps the plane whole through
signal routing (measured: pour-first + fragility served 63/70 balls by fill
alone; pour-last served 0/70 — every ball needed repair welds). Consequences
for the plan:
- Keep excluding plane nets from the FANOUT's `--nets` — the exclusion is
  exactly what marks them for drops. (The ROUTE step later includes them,
  #562 — its pour-launch anchors and in-run finalize complete them.)
- Pour the planes in Step 1, BEFORE fanout (see the Routing Order Rationale),
  so the drop pass sees real fill.
- The route step's plane finalize rarely needs to do more than verify under
  a dropped BGA (its oracle exits at round 0 on a healthy board).
- `--plane-drop off` disables the pass; `KICAD_FANOUT_PLANE_DROP=0/1`
  overrides either way (the recorded-manifest A/B switch). The per-net drop
  counts are in `JSON_SUMMARY.plane_drop`.

**After every BGA/PGA fanout, run the decoupling-cap placement optimizer
(#130).** A fanout drops vias near the ball field; where a foreign-net via
lands under a decoupling cap placed at a ball, the via copper overlaps the
cap pad → a real `PAD-VIA` DRC violation at the clearance floor. The fix is
placement, so run `place_fanout_clearance.py` on the **fanned** board to
nudge those caps clear (and pull each pad toward its nearest same-net ball so
a power/GND via dropped there later shares the via). See "Step 1b" below for
the command. It's cheap, only touches caps near a BGA, and is a no-op when
nothing collides — so run it ONCE after ALL fanouts are done, before signal
routing (see Step 1c for why once, not per-BGA).

Report to user:
- List of components that may need fanout
- Package type, pad count, and grid depth for each
- Recommended fanout tool

## Step 4: Check for Differential Pairs and Power Nets

Use `list_nets.py` to detect differential pairs and power/ground nets:

```bash
python3 py_router/list_nets.py path/to/file.kicad_pcb --diff-pairs --power
```

### Read the board's design rules and pass them to the CLI

The router does NOT read the board's design rules — it falls back to a generic
`--clearance 0.25` / `--track-width` default, which is often WIDER than the
board's own rule and can box pads in so nets fail with "no rippable blockers".
Read the board's real rules and pass them explicitly:

```bash
python3 py_router/list_nets.py path/to/file.kicad_pcb --design-rules
```

**KiCad has TWO tiers of rules, and DRC only enforces one of them — this matters
for fine-pitch boards (#111/#115):**

- **Net-class values** (`clearance`, `track_width`, `via_diameter`, `via_drill`):
  these are the size new objects are *drawn at*. Of these, only **clearance** is
  a DRC-enforced minimum. `track_width` and `via_diameter`/`drill` are **not** DRC
  floors — they are just defaults, so a board can (and the human originals do) use
  a **smaller** via/track than the net-class nominal and still pass DRC.
- **Board Constraints** (`min_clearance`, `min_track_width`, `min_via_diameter`,
  `min_hole_to_hole`, `min_through_hole_diameter`): **these are the actual DRC
  floors.** `--design-rules` reads them from `design_settings.rules` and combines
  them with the JLCPCB fab minimum (backstop when a Constraint is 0/unset — e.g.
  `min_clearance` is frequently 0) into a single **manufacturing floor**.

### FIRST: does this board have a requirements document?

**If it does, the SPEC outranks everything below.** `--design-rules` reports what
the *fab* can make and what the *board file* currently declares. Neither is
permission. On a board with real requirements its suggestions actively
contradicted three HARD rules at once:

| `--design-rules` printed | the spec said |
|---|---|
| `--via-drill 0.25` | 0.3 mm, HARD |
| *"drop `--track-width` to the fab floor 0.1 … BELOW the board's own rule"* | 0.15 mm HARD, and *"shall not be routed to"* 0.10 |
| `check_drc.py --clearance 0.1` | classes at 0.16 / 0.45 — grading at 0.1 hides real violations |

So: read the requirements first, write the numbers down with their requirement
IDs, and treat the printed flags as a *starting point to be overridden*. The
"route at the fab floor" advice further down is correct **absent a requirements
document** and wrong with one — it is exactly how a previous run shipped 267
segments at 0.127 mm against a 0.15 mm HARD floor.

ONE flag now holds a spec floor, and it is not discoverable from the
suggestions (a second, `--track-width-floor`, was removed -- see below):

- **`--track-width` is a REQUEST, and nothing enforces it.** A wide route that
  will not fit is necked down, and the per-net rescue re-routes a failed net at
  the **fab** floor -- both reporting the net routed. `--track-width-floor`,
  which used to make such a net fail honestly, was REMOVED in 53a5a16e and has
  no replacement: measure the copper afterwards with
  `board_score.py --net-min-widths`. (Not to be confused
  with `repair_planes --min-track-width`, its region-join width band,
  nor `check_drc --min-track-width`, which grades.)
- **`--fab-overrides <file>`** — `key = value` lines over the `--fab-tier` floor
  (`clearance`, `track_width`, `via_diameter`, `via_drill`, `annular`,
  `board_edge`, `hole_to_hole`, `pad_hole_to_hole`). Supplying it also **forbids
  the silent `standard`→`advanced` escalation**, which is what puts 0.25/0.15
  vias on a board that asked for 0.6/0.3. Measured, one such file took the score's
  `undersized` from **169 to 0**.

  **Its `clearance` key REPLACES the per-class clearance map, it does not floor
  it.** Set it to the board's *Default class*, not the spec minimum: pinning it
  to the tighter figure is what silently dropped an `XTAL_12M` class from 0.45 mm
  to 0.15 mm, and restoring 0.45 afterwards produced 126 violations.

Use the printed flags as-is **only when the board has no spec of its own**:

- **Routing** (`route.py`, `qfn_fanout.py`, `bga_fanout.py`, `route_planes.py`):
  `--clearance` from the **Default class**, but **`--via-size`/`--via-drill`
  from the working floor**, NOT the net-class `via_diameter`. Emitting the net-class
  via everywhere is #115 — it's a max-like default, far too big for fine-pitch
  escape (e.g. a 0.4 mm QFN/BGA needs the small working via the original used).
  For `--track-width`, the net-class value is only a starting point and is *not* a
  hard minimum: on dense/congested boards route ordinary signals at the **fab
  physical floor** instead (thinner is both more complete and faster — see "Route
  signals at the FAB floor by default" in Diagnose and Retry). Keep the net-class
  width only for **current-carrying nets** (`--power-nets`).
  **Do NOT keep the net-class gap/width for impedance-controlled (diff-pair) nets** —
  the stock net class is usually wide (`diff_pair_gap` 0.25 / width 0.2 mm), and a
  fat pair is a wider bundle that gets dropped on congested boards (measured:
  a 4-layer FPGA corpus board routes all 13 of its pairs at `--diff-pair-gap 0.1` but loses 2 at
  0.25). Per `/find-high-speed-nets`, route those at the **fab floor for gap and
  clearance (~0.1 mm)** while keeping `--impedance` for the width (the router
  computes it from the stackup and clamps it to the floor). `route_diff.py` then
  auto-updates the Default net class to those tight values (only-loosen, via
  `fix_kicad_drc_settings.py`), so the `.kicad_pro` stops advertising the wide gap.
- **Diff-pair sizing default + shrink-to-succeed.** Default `route_diff.py` to
  **`--track-width 0.1` and `--diff-pair-gap 0.1`** (the fab floor) — a thin, tight
  bundle routes on congested boards where a fat pair is dropped. If the interface
  is impedance-controlled, ALSO pass `--impedance <ohms>`: the router derives the
  width from the stackup and clamps it to the floor, so the target impedance is
  **maintained** while the geometry stays as small as it can. **When a pair fails
  or falls back** — `route_diff.py`'s `JSON_SUMMARY` lists it in `failed_diff_pairs`
  or `single_ended_diff_pairs`, or DRC shows an intra-pair / via-via graze — re-run
  the failing pairs with **smaller track width, smaller gap, AND smaller vias**
  (`--via-size`/`--via-drill` toward the fab via floor). A tighter track+gap fits a
  narrower channel, and smaller vias fit a tight pad pitch (measured: lumenpnp
  USB_D's two 0.5 mm vias collide by 0.1 mm at the connector pitch — a smaller via
  clears it). Step **toward, never below**, the fab floors, and keep `--impedance`
  so the ohms target is held as the geometry shrinks.

  **No width FLOOR exists on any tool** (`--track-width-floor` was removed in
  53a5a16e). So a pair whose width is a HARD requirement — a spec'd 0.8 mm USB
  geometry, say — has **no floor protection at all** in the diff-pair step: if
  the engine necks it down to fit, it does so silently and the summary still
  reports the pair routed. Measure the emitted copper after the call, and carry
  the requirement in `board_score.py --net-min-widths` so it reaches `blocking`:

  ```python
  from collections import Counter
  from kicad_parser import parse_kicad_pcb
  pcb = parse_kicad_pcb('out.kicad_pcb')
  print(Counter(round(s.width, 4) for s in pcb.segments
                if pcb.nets[s.net_id].name.startswith('USB_')))
  ```

  (The same asymmetry note applies as for `--coplanar-gap`/`--coplanar-nets`
  below: the diff engine takes some of route.py's flags and not others, and the
  ones it drops fail quietly.)
- **Escape clearance — trigger on dropped balls, not pitch (issue #122):** the
  inter-ball channel is too narrow to fit a track at the net-class clearance on
  more BGAs than just "fine-pitch" ones. Even an **0.8 mm-pitch** BGA drops balls
  at `--clearance 0.2` (the ~0.45 mm gap between 0.35 mm balls can't fit a 0.2 mm
  track at 0.2 mm clearance) — the same board escapes **all** balls at the 0.1 mm
  floor. So don't gate on pitch: gate on whether balls actually dropped.
  `bga_fanout.py` and `qfn_fanout.py` both end with a `JSON_SUMMARY: {...}` line
  giving `requested`/`escaped`/`failed`/`unescaped_nets`. **After every fanout, parse it;
  if `failed > 0` (escaped < requested), re-run the fanout with `--clearance` at
  the manufacturing floor** (never below it — the floor is the rule the human
  board passes DRC against, so tightening board-wide is manufacturable and needs
  no rule-area settings). If still short, also try the smaller **fine-pitch escape
  via** (below) and/or a narrower `--track-width` toward the floor. Do not proceed
  to signal routing with `failed > 0` unexpected — those balls are dropped from the
  output and will fail later as "no rippable blockers".
- **Also check `drc_grazes` (even when `failed == 0`).** The summary's
  `drc_grazes` (graded at the fanout `--clearance`) reports sub-clearance grazes the
  escape left in the output: `via_segment` / `pad_via` are the #130 classes (an
  escape via too close to a foreign track or pad), `segment_segment` is the #179
  class (two escape **stubs** grazing — typically the 45° fans of two adjacent pads
  of a tight-pitch diff pair, e.g. 0.4 mm-pitch QFN, clipping at the wrist),
  `total` is all DRC violations. A *successful* fanout (every ball/pad escaped) can
  still leave many of these — they're not caught by `failed`. **If any
  `drc_grazes` class > 0 and there is headroom above the fab floor, re-run the
  fanout stepping toward — never below — the floor:**
  - `via_segment` / `pad_via` (#130): smaller **`--via-size` / `--via-drill`**
    (and/or a thinner `--track-width`).
  - `segment_segment` (#179): thinner **`--width`** — the escape stubs carry the
    track width, so narrowing them widens the gap between the two converging
    diagonals. Step down toward the fab-floor track (e.g. 0.15 → 0.13 → 0.10 mm)
    until `segment_segment == 0`; all pads still escape (`failed` stays 0).
    (Measured on a dense QFN corpus board: 3 grazes at `--width 0.15`/`0.13`, 0 at `0.10`.)

  These grazes are typically a uniform ~1-grid-cell shortfall, so even one size step
  down usually clears them all; shrinking the via also relieves escape congestion.
  (For *via-over-pad* grazes where a decoupling cap/resistor sits on a via,
  `place_fanout_clearance.py` (Step 1b) is the better fix — it moves the part;
  smaller vias/thinner tracks help the via-over-track and stub-over-stub classes.)
- **Fine-pitch escape VIA (4+ layer):** the 0.45 mm standard via can't dog-bone /
  via-in-pad sub-~0.5 mm-pitch BGA/QFN balls. For *those parts only*, pass the
  smaller **fine-pitch escape via** that `--design-rules` prints (`fine-pitch
  escape via <d>/<drill>`, e.g. `0.30/0.15` — JLC "advanced", small extra cost)
  as `--via-size`/`--via-drill` to that part's `bga_fanout.py` / `qfn_fanout.py`
  and to `route_diff.py` when it launches from that part's escaped stubs. (The
  in-run plane finalize taps at the ROUTE step's via — fine-pitch plane balls
  under such parts should already carry fanout-time plane-drop vias, #424; if
  the finalize still reports them unconnected, re-run the route step with this
  smaller via.) Keep the **standard** working via for
  general `route.py` routing and the bulk `route_planes.py` pour — the advanced
  via is escape-only, not a board-wide default (issues #99/#122).
- **Non-Default classes:** route those nets separately with that class's
  `--clearance`/`--track-width` (clearance is the one per-class DRC value, so keep
  each class's nets at their own clearance rather than forcing one global value).
- **Diff pairs:** default `--track-width 0.1 --diff-pair-gap 0.1` for `route_diff.py`
  (NOT the wide net-class values), plus `--impedance` when the interface is
  impedance-controlled; shrink track/gap/via further toward the fab floor for any
  pair that fails or grazes (see "Diff-pair sizing default + shrink-to-succeed").
  **Never set `--diff-pair-gap` below the same command's `--clearance`** — KiCad
  grades the pair's P↔N coupling as a plain clearance violation, so `route_diff`
  floors the gap up to clearance (#441). Set the two equal (both at the fab floor).

**Verification (DRC/connectivity) grades at the manufacturing floor**, not the
inflated net-class clearance — that is the same rule the human original passes, so
it's the honest delta. The routing/plane/fanout steps now **record the smallest
clearance any step actually used** (route_planes, route.py's plane finalize, and the
single-ended multipoint taps auto-step the fine-pitch tap clearance DOWN toward the
fab floor as the geometry demands) into the output `.kicad_pro` DRC floor and into
`JSON_SUMMARY` (`min_clearance_used`). `check_drc.py` **auto-grades at that
`.kicad_pro` clearance when `-c` is omitted**, so a bare `check_drc.py board.kicad_pcb`
already grades at the true routed floor. Passing `--clearance <floor>` still works
to TIGHTEN the grade — it is a FLOOR, `max(-c, classA, classB)`, not an
override, so a value at or below the board's netclasses changes nothing.
See Step 6.

**`check_drc.py -c` is NOT `route.py --clearance`.** On route.py the flag is a
**ceiling over every class** (`--clearance` caps each net at `min(its class,
--clearance)`). On `check_drc` it is only the **global fallback**, and a netclass
override still wins — the tool prints `Required clearance: 0.1600mm
(local/netclass override; global 0.1500mm)` and grades at 0.16 no matter what
`-c` says. Measured on one board: 7 violations at `-c 0.16`, the same 7 at
`-c 0.15`, the same 7 at `-c 0.149`. If you expected a looser `-c` to clear
class-driven violations, it will not; change the class, or use
`--clearance-margin` (default 0.05) to filter grid-quantisation noise — and when
you use it, quote the unfiltered count beside the filtered one.

Only fall back to tool defaults when neither net classes nor Constraints are found
(`--design-rules` then prints the JLCPCB fab floor for the board's layer count).

This will output:
- Differential pairs detected (P/N naming conventions)
- Ground nets with pad counts
- Power nets with pad counts

If differential pairs are found:
- List each P/N pair
- Note that `route_diff.py` should be used for these
- Explain that diff pairs maintain consistent spacing and length matching
- **If a pair's pads are on a BGA/PGA being fanned out, escape it with
  `bga_fanout.py` too** — pass `--diff-pairs "<patterns>" --diff-pair-gap <gap>`
  so P and N escape the array together on one layer. Don't just exclude the
  pair from fanout and hand it to `route_diff.py`: it can't launch from the
  deep balls ("no valid position at any setback"). `route_diff.py` then
  connects the escaped stubs — **but on a 4+ layer board you must pass those
  inner layers to `route_diff.py` via `--layers` too** (it defaults to F.Cu
  B.Cu, so an inner-layer escaped stub is otherwise unreachable and the pair is
  silently dropped — issue #116). Pairs not on an array package don't need fanout.

> **Tip:** Name-based detection misses pairs with unconventional names. For boards with
> high-speed ICs (PHYs, SerDes, USB, FPGA transceivers), or when detection finds suspiciously
> few pairs, run `/identify-diff-pairs` for datasheet-based detection by pin function and
> per-interface gap/impedance recommendations.

**Polarity-swap policy (#279).** `route_diff.py` can resolve a P/N polarity mismatch by
swapping the target pads' net assignments — but a swap physically cross-connects one
device's P pin to the other's N pin, and is only harmless when an endpoint can compensate.
Swaps are **denied by default**; grant them per pair with `--polarity-swap-nets <patterns>`.
Before emitting the route_diff command, classify each pair's electrical endpoints (walk
through series AC caps/resistors to the real device):

- **Allow** pairs with an FPGA/CPLD generic-I/O endpoint (pin functions are reassigned in
  gateware — look for paired `IO_LxxP/N`-style pinfunctions on Xilinx/Lattice/Altera/Gowin
  parts), and protocol-tolerant links (PCIe lanes, SerDes with polarity-invert, 1000BASE-T).
- **Deny** USB `D+/D-`, MIPI, TMDS/HDMI/DP, CAN, RS-485/422, DDR `CK/DQS`, clock/analog
  inputs to fixed-function parts, anything reaching a connector or unknown part, and any
  pair whose nets carry an asymmetric attachment (e.g. a single-sided pull-up) — it stays
  on its net and would land on the wrong physical wire. MCUs/SoCs do NOT count as
  programmable (their diff functions are fixed silicon). **When in doubt, deny** — a
  skipped pair beats a dead interface. `/identify-diff-pairs` reports a per-pair
  `polarity_swappable` verdict from datasheet pin functions for the ambiguous cases.

Pass the resulting allowlist, e.g. `--polarity-swap-nets '/fpga/IO_*'` (use `'*'` only when
every pair classifies swappable). Applied swaps are listed in `polarity_swapped_pairs` —
when they happen, the schematic sync step below applies (see "Schematic Synchronization
After Swaps"). Pairs that *wanted* a swap but were denied are listed in
`polarity_swap_denied_pairs` — surface these to the user (they either routed via the
opposite-side flip or failed honestly and may need a manual pin swap in the schematic).

**Far-apart terminal pads → single-ended follow-up (issue #121).** A "diff pair"
sometimes has pads that aren't a coupled connection — e.g. a P and an N test point
several mm apart, or a logical pair daisy-chained through spread-out parts. If the
coupled chain can't be routed, `route_diff.py` peels those far-apart pads off the
chain (routing the genuinely-coupled terminals as a pair) and lists the affected
nets under `single_ended_followup_nets` in its `JSON_SUMMARY` (and a "route them
single-ended next" block on stdout). Those pads are **not** dropped — the **Signal
Routing** step (`route.py --nets "*"`) connects them P→P / N→N along with
every other unrouted net, since they remain unrouted after the diff-pair step. So:
**do not exclude the diff-pair nets from the signal-routing step's net selection** —
that step is what finishes the peeled pads. If you scope the signal step to specific
nets instead of `"*"`, add any `single_ended_followup_nets` to it explicitly.

**EXCEPT when the pair carries a LAYER or VIA clause — then the signal step is the
wrong finisher and you give the peel its own pass.** The bulk step routes **both
layers** at the **signal width**; a pair spec'd L1-only, 0 vias, 0.8 mm comes back
from it with a via, at 0.16 mm, and every summary says routed. Measured on one
board: `route_diff` returned `partial` with the two post-resistor halves carrying
**zero** copper, and following this rule would have finished them illegally. Give
the peel a Step-2c-shaped pass instead — same nets, `--layers F.Cu`, the pair's own
width. Note that the trick this used to rely on is GONE: a call scoped to only
those nets once let the global `--track-width-floor` scalar act as a per-net
floor, and that flag was removed in 53a5a16e. The scoped call is still worth
doing -- it is the only way to route the pair at its own width -- but the width
is now a request with nothing enforcing it, so MEASURE the result.

Two traps in that pass, both measured on the same board:

- **The router's own hint names the blocker, and it is the pair's other half.**
  `ROUTE FAILED - no rippable blockers found … the blocking copper belongs to
  pre-existing net(s) 'USB_DP' … Retry with --rip-existing-nets` — the coupled
  route committed copper the peel pass then cannot get past. Rip by exact name;
  the call already pins layer and width, so the rip is safe (9.3c rule 2).
- **A floor that makes the net FAIL is doing its job — read the failure before
  removing it.** With the floor at 0.8 the pass routed **0 of 4**; with the taper
  allowed, **4 of 4**. That is not a reason to drop the floor quietly. Measure
  *why*: on that board a 0.8 mm trace centred on a 0.200 mm QFN pad at 0.4 mm
  pitch overhangs **0.300 mm per side into a 0.200 mm gap** — it lands on the
  neighbouring pads. Unsatisfiable by construction, max width at that pad 0.30 mm.
  Allow the taper because 9.1a ranks connectivity above width, and report the
  shortfall as **stop condition 4, a finding about the requirement**, with the
  arithmetic. Never as "routed, with some width caveats".

**CARRY THE PAIR'S WIDTH INTO EVERY STEP THAT MAY TOUCH IT.** This handoff is a
silent width leak, and on a board with a HARD pair geometry it destroys the
requirement. The peeled pads are finished by a step whose `--track-width` is the
*signal* width, so they come back thin — and every later `"*"` pass (the all-nets Step 3 route and route.py's own in-run
finalize reconnect especially) can do the same to any pair segment it decides to redo.
Measured on one board: `route_diff` emitted the pair correctly at 0.8 mm, and by
the end of the chain **14 segments of it were 0.15 mm**, with `failed_diff_pairs`
empty and every step reporting success.

The fix is the same one 9.3c rule 2 gives for rips — a net returns at the
**calling** command's parameters — applied to the peel path:

```bash
# on EVERY pass that can redo the pair -- the all-nets Step 3 route included
route.py ... --nets "*" "!GND" \
    --power-nets USB_DP USB_DM USB_DP_R USB_DM_R --power-nets-widths 0.8 0.8 0.8 0.8
```

`--power-nets` is not only for power: it is the per-net width channel, and it is
the only way a `"*"` pass can honour a geometry an earlier step established. Then
verify with the width counter below — `board_score --net-min-widths` will show it
as `net_widths` if you miss it, but only if you passed the file.

**ONE `--power-nets` per command, never two.** It is `nargs="*"` with no `append`
action (`route.py:2981`), so a second occurrence **replaces** the first rather
than adding to it — and the widths are positional against the net list, so the
whole first group loses its width silently. Building the flag from two shell
variables (`$PWR $USBW`) is the natural way to write this and it is wrong:
measured, the rails' 0.4 mm vanished and **248 of 248** power segments came back
at the signal width, with nothing reporting a problem. Merge the nets and the
widths into a single flag:

```bash
--power-nets VCC3V3 VBUS USB_DP USB_DM --power-nets-widths 0.4 0.4 0.8 0.8
```

**And `--power-nets-widths` is itself only a REQUEST.** Getting the flag right is
necessary and still not sufficient: a wide route that will not fit is necked down
by the same ladder as `--track-width`, and the log says so in one line among
thousands — `Wide power route blocked - routed short edge at 0.2000mm (down from
0.8000)`. Measured: the flag landed correctly (`0.8mm: USB_DP_R, USB_DP, USB_DM,
USB_DM_R` in the log) and the board still carried that pair at 0.2 and 0.15.

Two things the rest of this skill does not tell you, and both matter here:

- **`--no-power-tap-neckdown` is the actual off-switch.** It forbids the taper
  rather than asking for a width. Reach for it when a width is a HARD
  requirement and you would rather the net FAIL than come back thin — which is
  the whole point of a floor.
- **There is no width floor flag at all any more.** `--track-width-floor` was a
  single GLOBAL scalar and was removed in 53a5a16e; even while it existed it
  could not hold a pair at 0.8 while signals ran at 0.15. There is no per-net
  floor flag and no global one. So the only honest gate on a per-net width is
  to **measure the
  emitted copper** and carry the requirement in `board_score --net-min-widths`.

### Check for DDR/High-Speed Memory Signals

Look for DDR signal patterns in the net list that may need length matching:
- Data signals: DQ0-DQ63
- Strobes: DQS, DQM, DM
- Clocks: CLK, CK

If DDR signals detected:
- Note that `--length-match-group auto` should be used
- DQ0-7 + DQS0 form byte lane 0, DQ8-15 + DQS1 form byte lane 1, etc.

Report to user:
- List of detected differential pairs (or "none found")
- Whether `route_diff.py` is needed
- Whether DDR/length-matching is needed

### High-Speed Signal Check (delegate to /find-high-speed-nets)

Whether the plan includes GND return vias - and the `--gnd-via-distance` to use -
is the `/find-high-speed-nets` skill's job: it classifies nets into speed tiers
(datasheet lookup, rise-time estimates) and maps tiers to recommended distances.
Follow that skill's methodology here (its quick net-name/footprint scan decides
whether the deeper datasheet pass is worth it) and put the recommended distance
into the plan's GND-via step. Remember its physical floor: never set
`--gnd-via-distance` below 3 x (via_size + clearance), ~2.5 mm for standard vias.

Report to user when presenting the plan:
- If high-speed nets found: "**GND Return Vias:** This board has [tier] signals ([examples]).
  GND return vias are included in Step N with `--gnd-via-distance [X]mm`. Let me know if
  you'd like to skip this step."
- If no high-speed nets found: "**GND Return Vias:** The high-speed scan found
  no nets that need them (only low-frequency I2C/UART/GPIO). The step is
  included; it is cheap and harmless here. Want me to remove it?"
  Say what the SCAN found, not that the vias are "optional" -- optional invites
  dropping them on a board where the scan simply was not run, and a missing
  return path is not visible in any DRC.

`/find-high-speed-nets` ALSO reports **controlled-impedance nets** (its Step 4.5):
RF/antenna feeds (radio/PA/LNA -> SMA/U.FL/chip-antenna = **50 ohm single-ended**,
or 100 ohm if balanced), DDR SSTL, and the impedance-controlled diff interfaces.
Thread these into the plan:

- **Differential** impedance nets stay in the diff-pair step (Step 2) — just add
  `route_diff.py --impedance <ohms>`.
- **Single-ended** impedance nets (RF 50, DDR SSTL 40) get a **dedicated
  `route.py --impedance` pass placed AFTER diff pairs and BEFORE the general
  signal route** (Step 2b below). They must then be **excluded from the general
  signal route** (`"*" "!RF"` — the plane nets stay IN that route, #562) and
  counted in the Step 5b ledger as
  claimed by the impedance step — otherwise a later rip-up re-routes them at the
  wrong width.
- Impedance width is computed from the **stackup**: if the board has only KiCad's
  default stackup, lead the report with that warning and run `/recommend-stackup`
  first (an RF feed routed at a wrong width is electrically useless).
- For an RF/antenna feed also recommend (in words) a `User.2` keepout around the
  antenna region and `--keepout`, and route it short/direct on an outer layer.

If no controlled-impedance nets are found, omit Step 2b.

### Step 2b-i: Coplanar (CPW-over-ground) — decide this WITH the plane step (#486)

An impedance trace on an **outer layer that will also carry a GND pour** is not a
microstrip: the side ground pulls Z0 down hard, so hitting the target needs a
**narrower** trace (e.g. 0.277 mm instead of 0.376 mm for 50 Ω on 0.2 mm FR4).
Routing the microstrip width through a pour lands the trace well below target.

The router cannot detect this — the trace width comes from your declaration,
not from sensing copper. So this is **your decision to make in the plan**, and
it must be coordinated across two steps. (With the pour-first order, an
outer-layer GND pour normally comes from Step 1c — give THAT call the matching
`--zone-clearance G`; a pour-first pour makes the declaration safer, since the
copper the trace is sized against actually exists when it routes.)

**Declare coplanar when ALL of these hold:**
1. The impedance net routes on an **outer layer** (`F.Cu` / `B.Cu`). Inner layers
   are stripline; the flag is ignored there.
2. A `route_planes` step in this plan pours **GND on that same layer** — or the
   board already has an outer-layer GND pour that will survive.
3. You can name the gap: it is the pour's zone clearance.

**If you are not pouring on the signal's own layer, do NOT pass `--coplanar-gap`.**
A coplanar declaration whose pour never arrives leaves the trace too narrow, i.e.
impedance too HIGH — the opposite error, equally wrong.

**Coordination — one number, three places:**

```bash
# choose ONE gap G (the pour's clearance; near the fab floor, e.g. 0.2)
# 1. route the impedance nets, declaring G
python3 py_router/route.py in.kicad_pcb s2b.kicad_pcb --nets "RF*" \
    --impedance 50 --coplanar-gap 0.2 --clearance 0.2

# 2. pour GND on the SAME layer with a MATCHING zone clearance
python3 py_router/route_planes.py s2b.kicad_pcb s5.kicad_pcb \
    --nets GND GND --plane-layers F.Cu B.Cu --zone-clearance 0.2

# 3. verify the declaration actually held
python3 py_tools/check_impedance.py s5.kicad_pcb --coplanar-gap 0.2 --nets "RF*"
```

- `--coplanar-nets "<patterns>"` narrows the declaration to some nets in a call;
  omit it and every net in that call is treated as coplanar. Since Step 2b is
  already a dedicated impedance pass over exactly those nets, omitting it is
  usually right.
- `route_diff.py` takes `--coplanar-gap` but has **no** `--coplanar-nets` (the
  diff engine bakes one width per layer). Split interfaces into separate calls.
- The gap must be **achievable**: it is a pour clearance, so it cannot be below
  the fab floor, and near via antipads / pads the real gap will be wider. The
  Step-3 audit reports how much of each net actually achieved it.

**Report to the user** which nets you declared coplanar, the gap, and the plane
step it is tied to — this is a coupled choice they may want to override. If the
board has no outer-layer pour planned, say so explicitly and note that the
impedance nets are being routed as plain microstrip.

## Step 5: Review Power and Ground Net Strategy (delegate to /recommend-plane-mappings)

Which nets deserve planes and on which copper layers is the
`/recommend-plane-mappings` skill's job: it weighs pad counts and datasheet
current estimates, and assigns layers with SI rationale (GND adjacent to signal
layers for return paths, power planes paired against GND, split layers for
multiple rails). Follow its methodology here, seeded by the `list_nets.py --power`
output, and put the resulting net -> layer assignments into the plan's
`route_planes` steps. Nets it leaves to wide traces become `--power-nets` /
`--power-nets-widths` on the route step instead.

Report to user:
- Identified GND nets and pad counts
- Identified power nets and pad counts
- Recommended strategy (plane vs wide traces) with layer assignments

### Step 5a-tuned: Plane-map derivation rules (measured-optimal; refine the delegate's output with these)

**THE DENSITY GATE COMES FIRST — plane-map aggression must scale with the
board (15-board wave + controlled A/B, 2026-08-17).** The aggressive map
below (outer floods, rail co-pours, many-rail splits) is what wins on dense
BGA boards (orangecrab 18 KiCad-unconnected, the best from-scratch result of
five arms; daisho 8-layer: 1 open, 0 new DRC). The SAME map applied to
small boards was the wave's dominant failure source AND its dominant time
sink: outer floods got carved into pad-anchored islands and hairline
(<60 µm) gaps, and every route pass re-oracled the big outer fills.
Controlled A/B on the four regressed boards, changing ONLY the plane map
(floods+fragility=0 → inner-only) with every other step identical:

| board | aggressive map | inner-only map |
|---|---|---|
| a 4-layer board | 7 open, 60s | **0 open, 41s** |
| upduino | 5 open + 2 DRC, 286s | **0 open, 1 DRC, 61s** |
| eis | 3 open, 504s | **0 open, 2 DRC, 115s** |
| a small 2-layer board | 1 open, 428s | **0 open, 0 DRC, 88s** |

And the reverse control on the dense board cuts the other way just as
hard — the same skill chain with the conservative (recorded-style) map
on orangecrab: **26 open at 7687 s vs 18 open at 2028 s** with the
aggressive map (plus ~2000 self-crossing weld-debris warnings on the
conservative arm). The aggressive map on a dense board is BOTH more
complete and ~4× faster; the conservative map on a small board is both
more complete and 3–5× faster. Neither map is "the safe one" — the GATE
is the safety.

**Compute the tier, don't vibe it. DENSE** = the board has **6+ copper
layers AND** (a populated fine-pitch grid array of ≥100 balls at ≤0.8 mm
pitch, **or** >150 nets). Everything else is STANDARD. Layer count is the
load-bearing half of the gate: on ≤4 layers the outer layers ARE the
routing surface, so floods there lose even next to a big BGA (measured:
eis, 4-layer with a fully-populated BGA-121 @0.8 mm, went 3 opens/504 s
with the aggressive map → 0 opens/115 s inner-only; orangecrab and daisho,
6/8-layer, are where the aggressive map wins).

**STANDARD boards (the measured-optimal default — most boards):**

- **Inner-only pours**: GND solid on the first inner layer (price 6.0);
  the ONE dominant rail (most pads) solid or split on the second inner
  (price 2.5). **NO outer-layer floods** — on a small board the outer
  layers ARE the routing surface, and a flood there becomes island debt,
  sub-60 µm gap debt, and board-edge DRC (all three measured).
- **Every other rail rides `--power-nets` as a wide trace.** Do not
  Voronoi many rails onto one layer: **never more than 2–3 rails share a
  split layer** (measured: six rails Voronoi'd onto one 4-layer board's In2
  fragmented +3V3 into 8 pad-anchored islands → 7 opens; the 2-rail map
  → 0).
- 2-layer boards: GND flood(s) per Step 8's 2-layer flow (pour LAST on
  dense 2-layer); rails as traces.
- No per-zone fragility overrides — the default fragility field protects
  inner pours correctly.

**DENSE boards (the aggressive map — every rule keys on a MEASURED board
property; derive, don't copy):**

1. **Outer-layer GND floods (pour-direct service).** Count GND SMD pads
   per outer layer. An outer layer with a substantial GND SMD population
   (≳20 pads, or a fine-pitch BGA's GND balls on it) gets a GND flood:
   pads and balls are then served by FILL CONTACT with zero vias
   (measured: 124 balls pour-served, the first 100% fanout escapes).
   Floods must be carve-free — per-zone fragility `GND@<layer>=0` — or
   later routing fragments them into weld debt. (This knob is DENSE-only:
   it is exactly what turned small-board floods into island debt.)
2. **One solid inner GND plane, adjacent to the highway.** The unsplit
   reference layer. Price it 6.0 in `--layer-costs`.
3. **Bus-highway layer: derive it, keep it EMPTY and FREE.** Find the
   board's widest bus (largest same-endpoint-footprint-pair net group, or
   the dominant netclass family — DDR data/address, ≥8 nets). The highway
   is the inner layer adjacent to the GND reference spanning the bus's
   endpoints. NO pours on it, cost 1.0 — pricing or pouring the highway
   cost completions every time it was measured.
4. **A rail whose pads live overwhelmingly on one outer layer shares
   that layer's flood.** ≥~80% of the rail's pads SMD on outer layer L
   (termination arrays, e.g. VTT on B.Cu) → co-pour on L by Voronoi/
   grammar partition; the pads connect by fill contact and the rail needs
   no inner-layer real estate at all.
5. **Remaining rails: split across the remaining inner layers, grouped
   by pad geography — capped at 2–3 rails per layer.** Cluster rails
   geographically (the grammar-pour clustering) and assign clusters per
   layer so each partition stays compact (#662 shape targets: sheet
   compactness ≥0.6, islands ≥0.5). Price rail layers 2.5. Rails that
   don't fit under the cap (or whose region would be a sliver) ride
   `--power-nets` as wide traces instead — a fragmented pour is worse
   than no pour.
6. **Carry the SAME `--layer-costs` vector into `route_diff`** — the
   diff step is otherwise plane-blind and its pairs squat the priced
   layers.

### Step 5a-tuned-ii: Escape completeness sweep (all interior-pad parts)

COMPLETE FANOUT IS THE INVARIANT — never trade an escaped ball for a
routing score. Beyond the big BGAs, enumerate EVERY component with
interior pads the surface router cannot reach (WLCSP/CSP at ≤0.5mm pitch,
staggered no-lead arrays — the issue-#144 class) and give each its own
fanout step (`underpad` + via-in-pad at the pitch-derived fab-floor via)
BEFORE the route step. On the benchmark, three 0.4mm WLCSP regulators
nobody had ever fanned were part of the winning plan. A board-wide sweep:
for every footprint, compute min pad pitch and whether any pad is
enclosed by other pads on all four sides; plan an escape for every hit.

### Step 5a-tuned-iii: Length matching from the board's own classes

If netclass names carry length-match hints (`*_LM<tol>`, `*length*`,
DDR-class groupings), wire them into the route step:
`--length-match-group auto --length-match-tolerance <tol>` and
`--time-matching` when the bus spans layers — but `--time-matching` ONLY
on a board with a real stackup (it converts length to delay through the
dielectrics; on KiCad's default stackup it computes garbage — the Step 10
rule-1 no-stackup precedence applies to it exactly as to `--impedance`).
The board's classes are the author's spec — honor them even when the
recorded chains never did.

## Step 5b: Net-Coverage Reconciliation (mandatory — do not skip)

The stages partition every routable net by glob pattern, and the patterns are
**not** reconciled automatically. The failure mode this step prevents: a net is
*excluded* from one stage (`!X`) but never *claimed* by a later one, so it
silently gets zero copper and the run "completes" with it fully unrouted. This
is exactly how `GNDA` (an analog ground tied to `GND` through a single 0Ω/
ferrite) was dropped — excluded from the signal route as a "power net", yet never
added to the plane step's `--nets`, ending with 0/23 pads connected while the run
reported success.

**The invariant: every routable net (≥2 pads, not no-connect) must be claimed by
exactly one stage. A net excluded from any stage MUST be claimed by a later one.**

Before running any command, write the net-handling ledger and reconcile it
mechanically — do not eyeball it:

1. **Assign every routable net to one handler:**
   - `route step` — ordinary signals AND the plane nets (#562: the route step
     takes `"*"`; plane pads weld into their pour via pour-launch and the
     in-run finalize taps whatever fill can't reach)
   - `diff-pair route` — detected pairs
   - `impedance SE route (Step 2b)` — single-ended controlled-impedance nets (RF/antenna
     50 ohm, DDR SSTL 40 ohm); the ONLY nets excluded from the route step
   - `pour` — nets the plane step pours; they are ALSO in the route step (see above)
   - `wide trace` — power carried via `--power-nets` widths (never excluded)

2. **Diff the pattern lists (#562 rules).** Two checks, and note that the old
   "every poured net must be excluded" rule is now exactly backwards — a
   poured net that is missing from the route step is the bug, because nothing
   then welds its pads to the pour:
   - the route step's exclusions MUST equal the Step-2b impedance set;
   - every poured net MUST appear in the route step's `--power-nets` (that is
     where the finalize's taps and welds get their width).
   ```python
   route_exclusions = {"RF"}                    # the !X you will pass route.py
   plane_nets       = {"GND", "+3V3"}           # the --nets you pass route_planes.py
   impedance_se     = {"RF"}                    # nets routed in Step 2b (route.py --impedance)
   power_nets       = {"GND", "+3V3"}           # the --power-nets on the route step
   orphans = route_exclusions ^ impedance_se
   assert not orphans, f"Net-coverage gap: {sorted(orphans)} handled by no stage"
   unsized = plane_nets - power_nets
   assert not unsized, f"Poured but no route-step width: {sorted(unsized)}"
   ```
   Do not proceed until both are empty.

3. **Secondary grounds / split rails** (`AGND`, `GNDA`, `DGND`, `VREF`, or any rail
   tied to its parent through a single 0Ω resistor or ferrite bead — find the tie
   with `list_nets.py`: the part with one pad on each net). These are real,
   separate nets. Pour each as **its own local region** (Voronoi-sharing an inner
   layer with the main ground is fine) and let the single tie component join it to
   the parent. **Never** merge it into the parent plane (that shorts the split and
   defeats its purpose — a green connectivity check then hides an electrical error)
   and **never** leave it out (that leaves it unrouted). Give each its own `--nets`
   entry in the plane step, so it appears in BOTH lists in step 2 above.

## Step 6: Generate Routing Plan

Based on the analysis, generate a step-by-step plan. The general order is:

### Routing Order Rationale

1. **Pour the planes FIRST** — before fanout, before any routing. A bare
   `route_planes`
   call: nets + layers only. NO `--add-gnd-vias`, NO `--stitch-vias` — those
   adapt to signals that don't exist yet (the old #56 hazard) and belong in
   Step 3. (The pour cannot rip at all any more: `--rip-blocker-nets` and the
   other tap knobs were REMOVED from `route_planes` with the tap machinery.) Why the pour comes first (#424, measured):
   the fanout's plane-drop vias connect to a still-intact pour immediately, and
   the **plane-fragility field** (default on: `KICAD_PLANE_FRAGILITY_COST`,
   2.0 mm-equiv, `=0` reverts) then makes every later routing step pay to cut
   the real fill where it is narrow — so signals cross planes mid-pour, not at
   necks. Measured on a 4-layer corpus board, this order + the field: power nets fully connected,
   +3V3 pour ONE intact island, GND weld copper cut to a third, connectivity
   net-better, DRC clean. With planes poured signals-first style instead, the
   pour under a BGA arrives pre-shredded and every drop via needs repair welds.
1b. **Fanout** (if needed) - Escape routing on the poured board. Exclude the
   plane nets (`"*" "!GND" "!VCC"`) — that exclusion marks them for automatic
   **plane-drop vias** (#424), and because the pour already exists the drop
   pass can skip a via entirely where the fill already covers the ball
   (pour-direct) and land the rest on intact copper.
1c. **After ALL fanouts are done — once, not per-BGA — run
   `place_fanout_clearance.py`** to clear decoupling-cap / fanout-via
   collisions (#130) before routing. The pass is board-global (it reads every
   via and every BGA), so one late run sees everything; running it per-BGA
   compounds cap displacement and changes what later fanouts route around.
   See Step 1c.
2. **Differential Pairs** - The most constrained routes claim their channels before
   anything else can block them (if present). Add `--impedance <ohms>` for the
   controlled ones (USB/Ethernet/LVDS/balanced-RF; from `/find-high-speed-nets`).
   May peel far-apart "terminal" pads (e.g. spread-out test points) off the coupled
   chain and leave them for the signal-routing step (reported as
   `single_ended_followup_nets`, issue #121). (The pour is not an obstacle to
   them — pours never block the router; the fragility field only prices
   plane-severing paths.)
2b. **Impedance-controlled single-ended nets** (only if `/find-high-speed-nets`
   found any - RF/antenna feeds = 50 ohm, DDR SSTL = 40 ohm). A dedicated
   `route.py --impedance <ohms>` pass, routed here - after diff pairs, before the
   bulk signal route - because they need a stackup-derived width and a short,
   direct path over a clean ground reference, so (like diff pairs) they must claim
   their channel before the bulk signals fill the area. Route an RF feed on an
   outer layer (`--layers F.Cu`); requires a real stackup (see Step 2 stackup
   check). These nets are then EXCLUDED from step 3.
2c. **ANY net with a per-net geometric requirement** — not just impedance ones.
   The step-2b shape (own pass, before the bulk route, then excluded from it) is
   the general answer whenever the spec constrains ONE net's geometry: a required
   width, a layer restriction, a **via ban**, a maximum length. Routed in the bulk
   pass those nets get whatever the router finds convenient, and no later step can
   put it back without ripping everything around them.
   Worked example — a crystal spec'd at *0.15 mm, max 0 vias per leg*: routed in
   the bulk pass it came out at 0.16 mm with **6 vias**; given its own single-layer
   pass first (`--nets XIN XOUT XTAL_XOUT --layers F.Cu --track-width 0.15`) it met
   both clauses, because a one-layer route cannot place a via at all. Constrain by
   **construction** where you can — a `--layers` with one entry is a via ban the
   router cannot violate — rather than by hoping the bulk pass agrees.

   **A MAXIMUM LENGTH CANNOT BE CONSTRUCTED, AND THE DEFAULT SEARCH ACTIVELY
   FIGHTS IT. Set `--heuristic-weight 1.0` on that pass.** Every other clause in
   this list has a flag that makes violating it impossible; a length clause has
   none, so it is held by two things instead: routing the net FIRST (which 2c
   already mandates), and asking the router for a *short* path rather than a
   *fast* one. `route.py`/`route_diff.py` default to **1.9**, which is
   **inadmissible** — a weight `w > 1` licenses a returned path up to ~`w`× the
   optimal cost. On a net whose HARD requirement IS its length, that default is
   not a speed knob, it is the requirement set wrong. Measured on a bus spec'd at
   ≤15 mm, same input board, same everything, `--grid-step` held at 0.05:

   | | `--heuristic-weight 1.9` | `--heuristic-weight 1.0` |
   |---|---|---|
   | routed length | **44.50 mm** | **7.73 mm** |
   | straight-line pad distance | 7.71 mm | 7.71 mm |

   Two things go with it, and skipping either turns the fix into a new failure:

   - **Pick a grid fine enough — the iteration budget takes care of itself
     (#529).** An admissible search expands vastly more nodes; the dynamic
     budget self-extends to a 1e7 ceiling while the search progresses, so do
     NOT pass `--max-iterations` (in the historical static-budget experiment,
     weight 1.0 at the default `--grid-step 0.1` exhausted and returned NO
     PATH where 1.9 had returned the 54 mm one — converting a length
     violation into an `unrouted`, which 9.1a ranks strictly worse). Pair 1.0
     with a finer grid, then check the net actually routed.
   - **Measure routed length against the straight-line pad distance and quote the
     ratio.** That number, not the router's success line, is what says whether the
     clause is met.

   Leave the bulk signal pass at the fast default — its nets carry no length
   clause and 1.0 there costs a great deal for nothing.

   **The general rule: before pulling a knob for a clause, ask what that knob's
   default optimises.** A price is not a ban (`--via-cost 50` *buys* a via
   whenever the detour exceeds ~5 mm; only a one-entry `--layers` forbids one), a
   crossing count is not a length (`--ordering mps` minimises crossings), and
   speed is not shortness. Where the clause can be met by construction, construct
   it; where it cannot, set the objective and measure the result.

   **ORDER THE 2c PASSES BY WHICH ESCAPE CHANNEL THEY CONTEND FOR, NOT BY WHICH
   CLAUSE IS TIGHTEST.** The obvious ordering — most-constrained first — is the
   right instinct for *claiming board area* and the wrong one when two passes
   leave the **same fine-pitch part** on the **same layer**. There, the first
   pass's copper is a wall for the second, and the wall's width is what matters,
   not the clause's tightness. Measured on a QFN-60 at 0.4 mm pitch: a 0.8 mm USB
   pair routed before the QSPI bus landed on pins 51/52, immediately adjacent to
   the bus's pins 55–60, and two bus nets could then find **no path on any
   parameter** — admissible heuristic, grid down to 0.025, rip sets from single
   named nets up to the whole bus, 10⁶ iterations, and even with a second layer
   allowed. The router's own history named the blocker plainly: `Blocked by:
   QSPI_SD2, QSPI_SCLK` — its own neighbours, three mid-row pins deep.

   So before fixing the order, list for each 2c group: which component it escapes
   from, on which layer, at what pitch, and how wide its copper is. Groups that
   share a part *and* a layer are in contention; among those, **route the widest
   copper LAST**, because a wide trace boxes in far more of a fine-pitch field
   than it needs and a thin one can thread past copper already down. Groups that
   escape from different parts do not interact and their order is free.

   **When contending groups on one part want different widths, the answer is a
   fanout SCOPED to one group.** A whole-part fanout commits every stub at a
   single width, which cannot serve a 0.8 mm pair and a 0.15 mm crystal and a
   0.16 mm bus at once — a real reason to skip it. But `--nets` scopes a fanout
   the same way it scopes a route, so `qfn_fanout.py --component U1 --nets
   'QSPI_*' --width 0.16` stages exactly the escapes that are contended, at
   exactly their clause's width, and leaves the pair and the crystal alone.
   Skipping the fanout wholesale because *one* group's width is incompatible is
   the mistake — it was correct for the pair and the crystal and wrong for the
   bus, and the bus is what failed.

   **READ EVERY DOCUMENT THAT DERIVES FROM THE CLAUSE, not just the spec row.**
   A requirements table is rarely the whole requirement. This cost a real
   mistake, in the direction that matters: a bus whose spec row read *"Max
   direct-run length, single layer | ≤15 mm"* was pinned to one layer; the pin
   made the board hard to route (**29 broken pieces**), so the row was re-read as
   "it only bounds a length" and the pin removed. That reading was **wrong**. The
   repo's design brief said, deriving from the same clause:

   ```
   | Layer preference | L1 (top) only, one layer, direct run — HARD | HW-TB-PCB19 |
   | Via transitions  | Max 0 vias — the ≤15 mm run length assumes a direct
                        single-layer path with no layer changes  | derived from HW-TB-PCB19 |
   ```

   and the repo's own checker gated on vias for that clause. Unpinning healed
   connectivity and made the clause fail **worse** (6 violated lines instead of
   5) — a board that scores better and conforms less.

   The rule: before you decide a clause does *not* impose a constraint, check the
   **design brief, the checker, the netclass and the `.kicad_dru`** as well as the
   spec table. A constraint that is expensive to hold is not evidence that it
   isn't there — and "the score improved when I dropped it" is exactly the
   reasoning that ships a non-conformant board. If a HARD constraint really is
   unsatisfiable, that is stop condition 4: report it with the measurement, do
   not quietly relax it.

   (The genuine caution stands: a one-entry `--layers` halves the routing space,
   which on a 2-layer board is most of the board. Expect to pay for it, and say
   what it cost — but pay it when the requirement says so.)

   **A Step 2c pass is not durable. Two later steps silently undo it, and both
   report success.** Excluding the net from the bulk signal route is necessary
   and NOT sufficient — measured on one board, a crystal and a QSPI bus that
   left their own passes on F.Cu with **0 vias** ended the chain on both layers
   with **8 vias**, failing four HARD clauses, while every step printed a clean
   summary. The two doors:

   1. **`repair_planes --rip-blocker-nets` reconnects what it rips,
      IN-STEP, at ITS OWN parameters.** It does not know your `--layers`. Pass
      Both `--net-layers <json>` and `--track-width-floor` were REMOVED in
      53a5a16e, so a layer pin and a width clause can no longer be restated on
      the retry: a rip IS a silent constraint reset, and there is no flag that
      prevents it. Re-measure the ripped nets afterwards
      (`board_score.py --net-min-widths`, and check the layers landed).
   2. **The all-nets Step 3 route's `--nets "*"` re-routes them again (#562: its
      in-run finalize also reconnects rip casualties).** The template
      below excludes only the *plane* nets; on a board with per-net geometry that
      flattens every Step 2c pass in one command. **Mirror the geometry passes in
      the reconnect, in the same order, and sweep the remainder last:**

      ```bash
      route.py r7.kicad_pcb r8a.kicad_pcb --nets XIN XOUT XTAL_XOUT --layers F.Cu --track-width 0.15
      route.py r8a.kicad_pcb r8b.kicad_pcb --nets "QSPI_*" FLASH_CS --layers F.Cu --track-width 0.16
      route.py r8b.kicad_pcb r8.kicad_pcb  --nets "*" "!GND" "!XIN" "!XOUT" "!XTAL_XOUT" "!QSPI_*" "!FLASH_CS"
      ```

   **The rule, and its exact scope: a constraint with no persistence channel in
   the `.kicad_pro` must be re-stated at EVERY step that can touch the net.**
   That is **layer and width pins specifically** — `--layers` and
   `--power-nets-widths`. (The per-net pair, `--net-layers` and
   `--track-width-floor`, was REMOVED in 53a5a16e; nothing replaced them.) A
   step that
   re-routes without them resets the net to that step's defaults. Same failure as
   9.3c rule 2 (a ripped net returns at the *calling* command's parameters),
   reaching the plane repair and the reconnect as much as an explicit
   `--rip-existing-nets`.

   Do **not** over-generalise it — several constraints ARE durable and re-stating
   them is wasted effort:

   - **protected nets** (#521): matched groups and routed diff pairs are recorded
     in the sibling `.kicad_pro` and no rip glob or `--rip-blocker-nets` touches them;
   - **KiCad-`locked` copper**: never rippable, with no override at all;
   - **`net_impedance` declarations**: persisted, and recomputed identically by a
     later step from the stackup;
   - **`.kicad_dru` per-LAYER clearance**: auto-read by every routing step. Note
     the qualifier — layer-scoped rules, **and (#549) the track-channel
     grammar**: a rule of the form `A.NetClass=='X' && B.NetClass!='X' &&
     A.Type=='track' && B.Type=='track'` IS honored natively by the router as
     a per-layer, seg-vs-seg hard obstacle expansion around committed foreign
     copper (pads/vias exempt, raise-only over the fab floor). Consequence:
     pass ORDER decides which side pays the channel — route the protected
     class first so the stamp protects its copper from later passes. Only
     rules outside BOTH grammars (an area scope, a mixed condition) are
     *skipped with a printed note* ("KiCad will still enforce it"), which is
     true in KiCad and false for `check_drc.py` and for the router — without
     `kicad-cli`, such a rule is graded by **nobody**, and it cannot even
     appear in `ungraded` because no component ever knew about it. If your
     spec's separation rules live in the dru, say in the report exactly which
     steps enforced them and which did not;
   - **an explicit exclusion**: a bulk pass with `"!QSPI_*"` leaves that copper
     byte-identical — the exclusion works, it just isn't sufficient on its own.

   The asymmetry is the point: those four have a home in the project file, and
   layers and widths do not.
3. **Route ALL nets (#562)** - `--nets "*"`, plane nets INCLUDED: their pads
   weld into the pours via pour-launch anchors (default on) instead of routing
   as track webs, and the run FINISHES with the in-run plane finalize (taps +
   region joins + cleanup + KiCad-oracle verify, stubborn links joining the
   final reconciliation). Pass the plane nets in `--power-nets` with widths.
   Exclude only the single-ended impedance nets from step 2b (`"!RF"`) -
   re-routing those would drop their controlled width. This step also finishes
   any diff-pair pads peeled off in step 2, so keep the diff-pair nets in its
   selection (the `"*"` covers them). The fragility field steers it away from
   severing the Step 1 pours.
4. **Finalize planes (only when GND return vias / stitching are wanted)** -
   Re-run `route_planes` with the same nets/layers plus `--add-gnd-vias` (and
   any `--stitch-*` flags): an existing same-net zone is REPLACED in place, and
   the return/stitching vias now adapt around the finished signals — the #56
   ordering concern lives here, not at the pour. This step cannot rip either --
   the pour never taps, so there is no blocker to clear.
   **Stitching is normal human practice, not an exotic add-on**: 58% of ~400
   human corpus boards carry a free-standing GND stitch lattice. **But gate
   it on the SPEED TIER, not on pour count**: recommend `--stitch-vias` only
   when `/find-high-speed-nets` puts the board at high tier or above (RF,
   TMDS/SerDes, DDR) — "GND pours on 2+ layers" alone is NOT sufficient.
   Measured cost of stitching low-speed boards (15-board wave): the
   re-pour + mandatory trailing route.py roughly double a small board's
   wall time, and the stitch pass is the source of the recurring
   8–21 µm via-segment micro-graze DRC class and occasional orphan stitch
   vias — pure cost when nothing on the board needs the lattice. Leave
   the PITCH at the tool default (20 mm); only `/find-high-speed-nets` output
   tightens it (via `--stitch-max-freq`, which derives λ/20 and overrides the
   pitch). Do not hand-pick a pitch from corpus statistics.
   **Do not recommend `--add-teardrops`** (7% of human boards use teardrops)
   and **do not set `--thermal-relief`** — leave the tool's default connection
   style alone.
5. **Verification** - DRC and connectivity checks (plane repair is inside
   step 3's finalize — there is no separate repair step, #562)

### Example Plan Output Format

Present the plan to the user as a numbered list with explanations:

```
## Routing Plan for board.kicad_pcb

### Board Summary
- 2-layer board (F.Cu, B.Cu)
- 174 nets, 25 components
- Unrouted (0 existing traces)

### Components Requiring Special Handling
- **U9 (PGA120)**: 120-pin grid array - use bga_fanout.py for signals only

### Differential Pairs
- None detected

### Power/Ground Nets
- **GND**: 42 pads - use plane on B.Cu
- **VCC**: 23 pads - use plane on F.Cu (or wide traces if planes not desired)

---

## Step-by-Step Routing Commands

### Step 1: Pour the Power Planes (FIRST — before fanout and routing, #424/#562)
A bare pour: nets and layers ONLY. No `--add-gnd-vias`, no `--stitch-*` —
those adapt to signals that don't exist yet and run in Step 3 instead. **The pour runs FIRST, before fanout**: the fanout's
plane-drop pass then sees real fill, so a ball the pour already covers needs
no via at all (pour-direct) and the ones that do get a via land on intact
copper. The pour step itself does no routing at all (#562: it places no taps
— the route step's pour-launch and in-run finalize own every plane pad).
The default-on plane-fragility field
(`KICAD_PLANE_FRAGILITY_COST`, 2.0 mm-equiv; `=0` reverts) then charges every
later routing step for cutting the fill where it is narrow — signals cross the
planes mid-pour instead of severing them at necks.

python3 -X utf8 py_router/route_planes.py board.kicad_pcb board_step1.kicad_pcb \
    --nets GND VCC \
    --plane-layers B.Cu F.Cu \
    2>&1 | tee /tmp/step1_pour.txt

**Zone clearance is a MINIMUM-ALLOWED, not a target — never pass a
`--zone-clearance` larger than the routed clearance.** The default already
follows `--clearance` and auto-steps down to the fab floor when the pour
can't thread the densest BGA via lattice; a larger value only stops pours
from penetrating between balls/vias (human boards pour at ~0.1 for exactly
this reason). Watch the pour output for
`pour cannot thread the densest BGA lattice even at the fab floor`: when it
fires, no clearance setting can get an INNER-layer pour through that field —
the fix is a pour on the balls' OWN (outer) layer, which connects the pads by
direct contact (the plane-drop pass then skips those vias: `N pour-covered`).
`--min-thickness` (default 0.1) matches human under-BGA pours (0.089–0.1);
leave it unless a fab demands wider minimum copper.

Expect `check_connected` to show the plane nets fully connected from here on
(the drops + pour serve every BGA plane ball with no tap search).

### Step 1b: Fanout U9 (PGA120) - All Non-Plane Nets
Generates escape routing for ALL nets on the component EXCEPT those that the
planes step will handle. This ensures every signal net gets fanned out,
avoiding `--no-bga-zone` workarounds during routing.

**Important:** Use `"*" "!GND" "!VCC"` to fan out all nets except the power
plane nets. Do NOT use `"/*"` alone, as it misses nets with non-hierarchical
names like `Net-(U9-Pad1)` which would then require `--no-bga-zone` to route.

On a 4+ layer board also pass every copper layer with `--layers` (default is
F.Cu B.Cu only) so inner balls can escape — drop `--layers` only for true
2-layer boards.

python3 -X utf8 py_router/bga_fanout.py board_step1.kicad_pcb \
    --component U9 \
    --nets "*" "!GND" "!VCC" \
    --layers F.Cu In1.Cu In2.Cu B.Cu \
    --output board_step1b.kicad_pcb \
    2>&1 | tee /tmp/step1_fanout.txt

**Then check the `JSON_SUMMARY` line: if `failed > 0`, balls were dropped — retry
before continuing.** First confirm all copper layers are passed; then re-run with
`--clearance` at the manufacturing floor (e.g. `--clearance 0.1`), which fixes the
common case (an 0.8 mm-pitch BGA can't fit a track between balls at 0.2 mm). If still
short, add the fine-pitch escape via and/or a smaller `--track-width`. Only proceed
to Step 2 once `failed == 0` (or the remaining `unescaped_nets` are understood and
accepted).

### Step 1c: Optimize Decoupling-Cap Placement (run ONCE after ALL fanouts — issue #130)
Nudges decoupling caps near the BGA off the foreign-net fanout vias (the
`PAD-VIA` violations #130) and pulls each pad toward its nearest same-net
ball. Run it on the fully-fanned board — after the LAST fanout, **before**
signal routing. Use the
**same `--clearance`** you gave the fanout / your DRC floor — that's the only
setting that matters (it reads each via's real size from the board).

python3 py_placer/place_fanout_clearance.py board_step1b.kicad_pcb board_step1c.kicad_pcb \
    --clearance 0.1

It prints `Moved N cap(s); resolved R/M ... K unresolved`. Any **unresolved**
caps had no clear spot within the displacement budget — note them for a manual
nudge; they are not auto-fixed. By default (`--cap-prefix C,R`) it moves 2-pad
**caps and resistors** near a BGA (RN-style arrays auto-excluded since only
2-copper-pad parts move); it never overlaps parts, and is a no-op when nothing
collides. Feed `board_step1c.kicad_pcb`
into the next step. **With multiple BGAs, run it ONCE after the LAST fanout,
not after each.** The pass is board-global — it reads every via on the board
(`for v in pcb_data.vias`) and every BGA footprint for the same-net ball
attraction — so a single late run already sees every constraint at once. Per-BGA
runs are not equivalent, in two ways:
- **Displacement compounds.** Each cap's seed is wherever it sits on the board
  it is handed (`seed_x, seed_y = fp.x, fp.y`), and the budget
  (`--max-displacement` 2.0, growing ×1.5 to `--max-displacement-cap` 3.0) is
  measured from THAT seed. A second run re-seeds at the already-moved position,
  so a cap can drift ~2× the cap budget from where it started, and "move as
  little as possible" becomes minimal-from-the-moved-spot rather than from its
  real seed.
- **Moving caps changes later fanouts.** Cap pads are in the escape router's
  obstacle map (foreign pads + existing copper + vias), so tidying after BGA1
  hands BGA2's fanout a different obstacle field — different escapes, different
  vias, and then different cap decisions.
The two orders usually converge anyway (decoupling caps cluster around their own
BGA, so BGA1's caps are rarely in BGA2's escape field), which is why per-BGA was
long treated as interchangeable. Once-after-all is the default because it cannot
compound and costs one step instead of N.
Verify with `check_drc.py board_step1c.kicad_pcb -c 0.1` (PAD-VIA count drops).

### Step 2a: Differential Pairs (only if any were detected)
The most constrained routes claim their channels first. Add `--impedance <ohms>`
for controlled interfaces (USB/Ethernet/LVDS/balanced RF, from
`/find-high-speed-nets` or `/identify-diff-pairs`). The pours from Step 1 do not
block these — pours are never obstacles; the fragility field only prices paths
that would sever them. Pairs may peel far-apart terminal pads off the coupled
chain and report them in `single_ended_followup_nets`; the Step 2 route finishes
those, so do NOT exclude the pair nets there.

python3 -X utf8 py_router/route_diff.py board_step1c.kicad_pcb board_diff.kicad_pcb \
    --nets <pair globs, e.g. '/usb/*'> \
    --track-width 0.1 --diff-pair-gap 0.1 --clearance <floor> \
    [--impedance 90] \
    2>&1 | tee /tmp/step2a_diffpairs.txt

(No diff pairs on the board? Skip this step and feed `board_step1c.kicad_pcb`
straight into Step 2b / Step 2.)

### Step 2b: Impedance-Controlled Single-Ended Nets (only if any were found; runs before the Step 2 signal route)
ONLY when `/find-high-speed-nets` reported single-ended controlled-impedance nets
(RF/antenna feed = 50 ohm, DDR SSTL = 40 ohm). Route them in their own
`--impedance` pass, after diff pairs and BEFORE the general signal route, so they
claim a clean, short, direct channel at the stackup-derived width. Requires a real
stackup (run `/recommend-stackup` first if the board has KiCad's default). Route an
RF feed on an outer layer over the GND plane; recommend a `User.2` keepout +
`--keepout` around any antenna region (user draws it).

python3 -X utf8 py_router/route.py board_diff.kicad_pcb board_step2b.kicad_pcb \
    --nets RF --impedance 50 --layers F.Cu \
    --clearance <floor> --no-bga-zones \
    2>&1 | tee /tmp/step2b_impedance.txt

### Step 2: Route ALL Nets — plane nets included (#562)
Routes every unrouted net, **including the plane nets poured in Step 1** —
`--nets "*"` with no plane exclusions. Plane-net pads connect by welding
into the pour (pour-launch anchors, on by default), not by re-routing the
net as a track web, and the run **finishes with the plane finalize**: the
plane-repair engine (pad taps + region joins), the plane-copper cleanup,
and the KiCad-oracle exact-fill verify/reconnect all run IN this step, with
any stubborn oracle links joining the run's own final reconciliation. There
is **no separate plane-repair step anymore** — `repair_planes.py`
remains only for repairing a board outside this chain. Exclude only the
single-ended impedance nets already routed in Step 2b (`"!RF"`), so the
bulk pass cannot re-route them off their controlled width. The pours don't
block the router, and the fragility field makes plane-severing paths
expensive, which is what keeps them intact through this step.

**Pass the plane nets in `--power-nets` with widths** (e.g. `GND 0.3`): the
finalize's taps and welds size their copper from the power-width channel.

For boards with BGA/PGA components, use `--no-bga-zone` to allow the router
to find alternative paths through the dense pin area (even when fanout was
done, some paths may require this). Use `--max-ripup 5` for difficult
2-layer boards.

**If the finalize reports `Pads still unconnected` on fine-pitch (BGA/QFN
≤0.5 mm-pitch) pads, re-run this step in this order — cheapest first:**
1. **Smaller via** — drop `--via-size`/`--via-drill` toward the fab's
   fine-pitch escape via (e.g. `0.30/0.15`), never below the fab via floor.
   A boxed ball usually fails because the tap via can't fit beside it.
2. **Then finer grid** — drop `--grid-step` (e.g. `0.05 → 0.025`), not
   below the board's minimum feature: a 0.65 mm-pitch escape can be a
   grid-resolution limit, not a width one.
(BGA plane balls under a dropped part should already carry fanout-time
plane-drop vias (#424), so this retry is rare.)

> **Do NOT pass `--max-iterations` (#529 dynamic iterations, default on).**
> The router self-budgets: full searches automatically earn +1×base
> extensions while the search's heuristic keeps approaching the target, up
> to a 1e7-iteration ceiling — a genuinely hard net gets far MORE than the
> old `--max-iterations 1000000` advice ever gave it, while hopeless
> searches stop early. A net that still fails after an
> `"dynamic iterations (#529): search extended to N"` log line is a
> capacity problem (rip-up, clearance, layers), not a budget problem.
> (`KICAD_DYNAMIC_ITERATIONS=0` restores the legacy static caps for A/B.)

python3 -X utf8 py_router/route.py board_step1c.kicad_pcb board_step2.kicad_pcb \
    --nets "*" \
    --no-bga-zone \
    --max-ripup 5 \
    --power-nets GND VCC <other PWR...> --power-nets-widths 0.3 0.4 <W...> \
    --layers <ALL copper layers> --layer-costs <1.0 signals / 3.0 solid planes / 1.5 split-or-highway> \
    2>&1 | tee /tmp/step2_routing.txt

The `--layer-costs` line is NOT optional when Step 1 poured any solid plane:
without it signals cross the pours at cost 1.0 and shred them (measured: split
power pours at 0–2% connected under a BGA on a chain that omitted it). Order
matches `--layers`; 3.0 on solid-plane layers, 1.0–1.5 on split/route+pour and
highway layers, 1.0 on F/B. On dense boards use the measured-optimal pricing
from Step 2c instead (GND plane 6.0, rail pours 2.5, bus highway FREE).

(When Step 2b ran, exclude its impedance nets, e.g. `--nets "*" "!RF"`, and
route from `board_step2b.kicad_pcb`.)

This produces the **canonical final board** — the finalize's `JSON_ORACLE`
line reports the KiCad-verified plane-completion verdict for the run.

### Step 2c: Tuned route parameters (the measured-optimal set)

A 15-board screen (2026-08-17) measured the following parameter set as
STRICTLY DOMINANT over each board's naive parameters — total KiCad
post-refill unconnected 62 → 23 across the corpus at **equal total wall
time** (better first-pass arrangement repays the extra search in saved
rip/retry churn). Apply it whenever the board is dense enough that any
fanout or escape is contested; on trivially-open boards the defaults are
fine.

1. **Strict small features** (the single biggest lever on packed boards —
   lane pitch is quantized by track+clearance, and a 0.1 grid cannot
   express a 0.28 pitch at fat features):
   `--track-width 0.0762 --clearance 0.0889 --via-size 0.25
   --via-drill 0.15`
   The fab-floor clamps pin track/clearance/via UP automatically on
   boards whose layer count or fab tier can't take them — passing those
   four is always safe. Grading stays honest via the `.kicad_pro` floor
   writeback.
   **EXCEPTION — `--hole-to-hole-clearance` does NOT clamp**: route.py
   board-derives h2h only when the flag is OMITTED and honors an explicit
   value verbatim, while `check_drc` pins its grade UP to the board's
   `min_hole_to_hole` — so an explicit 0.2 on a 0.25-constraint board
   routes real, graded drill-pair violations (verified in code by two
   independent plan audits). Pass the BOARD's own `min_hole_to_hole`
   (from `--design-rules`), or omit the flag and let route.py derive it.
2. **Direction preference**: this is now the DEFAULT (5), so passing
   `--direction-preference-cost 5` is optional — keep it if you want the
   manifest self-documenting. #663's corpus screen took the old 250 default
   to 5 on the strength of sets 1-5, 75 boards per arm at one commit: −22
   incomplete nets (−19.6%), W15/L6, real DRC flat. A weak nudge organizes
   layers; 250 priced every off-axis move above 3 vias and forced detours,
   while 0 loses the organization entirely.
3. **Layer pricing** (order matches `--layers`): GND solid-plane layer
   **6.0**; rail/split pour layers **2.5**; F/B and free routing layers
   **1.0**; and leave the board's **bus-highway layer at 1.0 even if it
   carries pours** — the inner layer adjacent to the largest BGA that its
   widest bus needs (orangecrab: In2, the RAM highway; pricing it cost
   completions every time it was tried).
4. **The plan/attraction environment** (route step only, as env-var
   prefixes on the command line so the manifest replays them):
   `KICAD_GLOBAL_PLAN=1 KICAD_GLOBAL_PLAN_SEQ=1
   KICAD_GLOBAL_PLAN_SEQ_COST=1.5 KICAD_GLOBAL_PLAN_VIA_COST=20
   KICAD_GLOBAL_PLAN_ITERS=50000 KICAD_GLOBAL_PLAN_ATTRACT=1
   KICAD_ATTRACT_POTENTIAL=65 KICAD_GLOBAL_PLAN_RIVER=1
   KICAD_FINALIZE_REAUDIT=1 KICAD_PACK_INLINE=1`
   (SEQ-negotiated global plan + potential attraction + river packing +
   finalize re-audit. These are env knobs today, so they ride the
   redo-manifest form of the plan but NOT the GUI plan JSON — see the
   promotion note in Step 9.)
   **On DENSE-tier boards (Step 5a-tuned gate), ALSO prepend**
   `KICAD_GLOBAL_PLAN_LAYER_MODE=clique KICAD_GLOBAL_PLAN_LAYER=pref` —
   clique-negotiated layer assignment with preference-directed layers.
   These were the knobs that unlocked the orangecrab hand-ladder's 22→15
   descent; on the skill's own plan they measured 17 vs 18 unconnected at
   equal wall time (within single-board wobble by itself, but
   directionally consistent and free). Leave them OFF for STANDARD
   boards — unmeasured there, and standard boards already hit 0.

### Step 2d: Guided iteration + endgame (dense boards)

When Step 2 leaves failures on a dense board, do NOT hand-tune — iterate:

```bash
# each pass re-attempts only the failed/open tail (connected nets
# gate-skip), with rip authority against the settled board; the plan
# guidance persists through rips, which is what makes iteration
# CONVERGE instead of plateauing
python3 -X utf8 py_router/route.py board_step2.kicad_pcb board_iter1.kicad_pcb --nets "*" <same flags+env>
python3 -X utf8 py_router/route.py board_iter1.kicad_pcb board_iter2.kicad_pcb --nets "*" <same flags+env>
```

Two iterations are near-free (measured: the tuned corpus run with 2
iterations baked in cost +1.5% total time) and historically descend
frontier boards 25→15 over 3–5 passes — so bake two passes into every
plan, dense or not. The near-free property DEPENDS on the Step 5a-tuned
density gate: connected nets gate-skip, but each pass still re-oracles
every pour, so big carve-free outer floods on a small board turn "free"
iterations into 100 s+ passes (one measured 428 s chain; 88 s with
the same iterations after the flood was removed). Then two endgame
signatures:

- **A net walled by its own protected diff partner** (`no rippable
  blockers` naming the partner): the #521 override — name BOTH members
  EXACTLY with `--nets P N --force-reroute`, then one more all-nets
  iteration (override passes may pay off one pass late).
- **Bare-ball / island signatures** are now handled automatically
  in-engine (bare-ball fanout rescue incl. the cap-move, per-island
  stitcher) — if a net still ships bare, check the rescue log lines
  before reaching for manual surgery.

#### Octolinear smoothing is ON by default -- leave it alone

`route.py` collapses grid-A* staircase micro-jogs into octolinear shortcuts
(#536) at the end of every route step, by default. Do not disable it, and do not
try to schedule it onto one step.

It was briefly defaulted OFF, on the theory that smoothing mid-chain removes the
staircase slack a later step's rip-up/rescue needs. Two boards supported that
(cubesat_backplane 10 -> 2 incomplete nets, spartan6_6layer 12 -> 7). A corpus
A/B refuted it -- one commit, 147 boards, only the knob differing:

| smoothing | incomplete nets | per board |
|---|---|---|
| ON (default) | 129 | 0.878 |
| OFF | 149 | 1.014 |

Turning it off costs ~20 nets: 34 boards worse, 10 better. Even spartan6, one of
the two boards that motivated the change, measures ON=6 OFF=9 there. So
smoothing is not starving the later passes -- on balance it helps them,
presumably by freeing corridor space.

`--no-smoothing` exists for A/B only (`KICAD_SMOOTH_ROUTE=0/1` overrides either
way). The lesson worth carrying: a two-board result is not a default change.

### Step 3: Finalize Planes — GND Return Vias + Stitching (only if wanted)
Skip this step entirely on low-speed boards. When the speed analysis calls
for GND return vias or area stitching, re-run `route_planes` with the SAME
nets/layers as Step 1 plus the via flags: an existing same-net zone on the
target layer is REPLACED in place (CLI default), and `--add-gnd-vias` places
return-current vias that adapt around the now-finished signals — the old
"stitching vias placed early block a diff pair's only channel" concern (#56)
is why these vias run HERE and not at the Step 1 pour. BGA plane balls
already carry their fanout-time plane-drop vias (#424), so this step needs
no tapping under a dropped BGA. It cannot rip anything either: the pour
places no taps, so `--rip-blocker-nets` is gone from `route_planes`. (For
the record, the measured failure mode of ripping here was routed signals
lost for tap pads the drops already serve. Reconnect anything a chain does
leave open with a follow-up `route.py` pass naming them,
using the same parameters as Step 2.)

> **Note to user:** GND return vias improve signal integrity for high-speed
> signals. Based on the speed analysis, this board has [speed_tier] signals,
> so `--gnd-via-distance` is set to [X] mm. If this is a purely low-frequency
> board (I2C/UART/GPIO only), drop `--add-gnd-vias`. Let me know if you'd
> like that.

python3 -X utf8 py_router/route_planes.py board_step2.kicad_pcb board_step4.kicad_pcb \
    --nets GND VCC \
    --plane-layers B.Cu F.Cu \
    --add-gnd-vias --gnd-via-distance 2.0 \
    2>&1 | tee /tmp/step3_planes.txt

Adjust `--gnd-via-distance` based on the board's highest signal speed:
- Ultra-high (>1 GHz): 2.0 mm
- High (100 MHz - 1 GHz): 3.0 mm
- Medium (10 - 100 MHz): 5.0 mm
- Minimum physical limit: 3 x (via_size + clearance)

### (No separate repair step — absorbed into Step 2, #562)
The old Step 5 (`repair_planes.py`) and its Step 5c reconnect
are **gone from the chain**: `route.py` finishes every run with the same
plane-repair engine (pad taps + region joins), the plane-copper cleanup,
and the KiCad-oracle exact-fill verify — and any oracle links its own
router can't route join the run's final reconciliation WITH rip authority,
so the old rip-then-reconnect two-step happens inside one invocation.
(Step 3 cannot rip — see its own note — so there is nothing to reconnect
after it.) `repair_planes.py` still exists
for repairing a board OUTSIDE this chain (e.g. a hand-edited board).

**The pour-ordering rule this step used to carry still holds.** Any pour or
tap pass over a PARTIALLY-ROUTED board is a one-way door — for routed
corridors AND for unrouted pads. Once taps land next to committed copper the
corridors the constrained nets used are sealed, and every still-open pad's
escape channel is consumed by the tap-via carpet, which is not rippable
copper: `--rip-existing-nets` can move a NET, never a tap field (measured: 5
bare fine-pitch rail pads at a late pour oscillated 6-9 oracle joins across
five post-pour repair attempts and never closed). **Nothing ENFORCES this** —
`route_planes` once refused to pour (exit 3) over a partially-routed board
carrying bare pads, but that gate and its `--allow-bare-pads` opt-out were
removed in 5832e4eb so the branch matches main, which never had one. The
ordering is yours to keep: connect every pad before you pour, or accept
losing it. **The Step 1 empty-board pour is exempt by design** — taps placed
before any signal copper exists are ordinary obstacles the router routes
around from the start, which is half of why pour-first wins.

**If you DO invoke the standalone `repair_planes.py`** on an out-of-chain
board, it re-routes what it rips at ITS OWN parameters, so pass the per-net
pins in the same call: `--power-nets`/`--power-nets-widths` covering every
width-bearing net that could be ripped. (`--net-layers` and
`--track-width-floor` were REMOVED in 53a5a16e, so a single-layer net can come
back where it can take a via, and a width clause cannot be restated at all --
verify both afterwards rather than assuming the pin held.) There is no `--heuristic-weight` on that script, so keep
max-length nets out of the rip set by name. It auto-reads the sibling
`.kicad_dru` (#498/#549, no flag) and prices the FULL rules, so a net that
only routed under a staged/lifted rule cannot be re-joined once ripped.
`protected_nets` in the output `.kicad_pro` is what keeps a routed diff pair
out of the rip set — the log line `N PROTECTED net(s) excluded from blocker
rip-up` is the tell that it survived.

**Check `protected_nets` is still there before relying on it.** `route_diff`
records the pair under `kicad_routing_tools.protected_nets` in the output
`.kicad_pro`, and any helper that *replaces* that project file rather than
merging into it — a "restore the canonical netclasses" script, for instance —
silently deletes the record and re-exposes the pair. The tell is that log
line: if it is absent or its count dropped, the protection is gone and only
your `--power-nets` width carries it.

**This tool no longer takes `--deadline`** (upstream #621 removed it from the
plane-repair CLI; passing it is an argparse error), so bound it by SCOPE and
run it detached with an external cap you can attribute. The hazard the budget
used to close is real and measured: 40 min with `--rip-blocker-nets` and 25
min plain on a 217-part board, no board written either time (run 9), and it
fired again on a **113**-part board (run 15) — so do not assume a small board
is safe. Without a bound you get a
shell code and nothing to read.

> **Never `cp` a board without its `.kicad_pro`.** A bare `cp a.kicad_pcb
> b.kicad_pcb` copies only the board and strands the sibling `.kicad_pro`, which
> holds the DRC floor (the Default-netclass clearance/track/via the chain routed to).
> The next routing step then reads no project, resolves its floor from the STOCK
> (looser) netclass, and its writeback stamps that looser floor over tighter copper —
> so KiCad grades correct sub-floor copper as phantom clearance violations (measured:
> a dropped 0.09 floor became 0.10 → 160 phantom grazes on one corpus board). Use
> **`python3 py_router/copy_board.py src.kicad_pcb dst.kicad_pcb`** — it copies the board plus every
> sibling (`.kicad_pro`/`.kicad_prl`) and self-records into the redo manifest — or, if you
> must use `cp`, copy the `.kicad_pro` too. The routing scripts also WARN when an input
> board has no sibling `.kicad_pro` (#441).

### Step 6: Verify Results
The final board is `board_step2.kicad_pcb` (or `board_step4.kicad_pcb` when
the optional Step 3 GND-via pass ran) — call it `board_final` below.
Invoke `/review-routed-board board_final.kicad_pcb` for the full review (DRC,
connectivity, orphan stubs, length-match tolerances, GND return via coverage,
diff pair checks). If that skill is unavailable, run the raw checks — `check_drc.py`
**auto-grades at the `.kicad_pro` clearance the routing steps wrote** (the smallest
clearance any step used, including auto-stepped fine-pitch taps), NOT a hardcoded
0.25, so legitimately-tight fine-pitch escapes that are still fabbable don't read as
violations (#111/#226). A bare invocation is correct; pass `--clearance <floor>`
(from Step 4's `--design-rules` output) only to override:

python3 -X utf8 py_router/check_drc.py board_final.kicad_pcb 2>&1 | tee /tmp/step6_drc.txt
python3 -X utf8 py_router/check_connected.py board_final.kicad_pcb 2>&1 | tee /tmp/step6_connectivity.txt
python3 -X utf8 py_tools/check_orphan_stubs.py board_final.kicad_pcb 2>&1 | tee /tmp/step6_orphans.txt
```

**Coverage gate (mandatory — close the loop on Step 5b).** `check_connected.py`
already lists every net with ≥2 pads but no copper and no covering zone as
"Unrouted net with N pads" (it accounts for plane zones and ignores genuine
single-pad / no-connect nets). After Step 2's in-run plane finalize, **this unrouted list must
be empty** except for entries you can individually justify in writing (true
single-pad nets, deliberate no-connects). A fully-unrouted multi-pad net is a
coverage defect, NOT a shortfall to report-and-accept: it means a net fell
through the stage partition (Step 5b). For each one, go back and handle it —
route it, or add it to the plane step (a secondary ground gets its own pour
region per Step 5b) — then re-verify. Do not declare the board done while the
list has unjustified entries.

### Alternative: VCC as Wide Traces (No Plane)

If you prefer not to use a VCC plane, route VCC with wide traces instead:

```
### Step 1 (Alternative): Pour GND only
python3 -X utf8 py_router/route_planes.py board.kicad_pcb board_step1.kicad_pcb \
    --nets GND --plane-layers B.Cu

### Step 1b (Alternative): Fanout U9 Including VCC
python3 -X utf8 py_router/bga_fanout.py board_step1.kicad_pcb \
    --component U9 \
    --nets "*" "!GND" \
    --output board_step1b.kicad_pcb

### Step 2 (Alternative): Route ALL Nets, VCC as Wide Traces
python3 -X utf8 py_router/route.py board_step1b.kicad_pcb board_step2.kicad_pcb \
    --nets "*" \
    --power-nets GND VCC --power-nets-widths 0.3 0.5
```

VCC simply stays out of the pour assignments and rides the route step at
its wide power width; GND still pours in Step 1c and completes through the
route step's finalize like the main flow. If VCC wasn't fanned out, add
`--no-bga-zone U9` to allow router access.

## Step 7: Check for High-Speed Signal Requirements

### Length Matching (DDR, high-speed buses)

For DDR memory or other length-matched buses, detect signals that need matching:

```python
# Common DDR signal patterns
ddr_patterns = ['DQ', 'DQS', 'DQM', 'DM', 'CLK', 'CK', 'CAS', 'RAS', 'WE', 'CS', 'ODT', 'CKE']
ddr_nets = [n.name for n in pcb.nets.values()
            if n.name and any(p in n.name.upper() for p in ddr_patterns)]
```

If DDR or length-matched signals detected, add to the plan:
- `--length-match-group auto` for automatic DDR byte lane grouping
- `--length-match-tolerance 0.1` for acceptable variance (mm)
- `--time-matching` if routes span different layers (accounts for dielectric)

### Impedance-Controlled Routing

For high-speed signals with impedance requirements:
- `--impedance 50` for 50Ω single-ended (calculates width per layer from stackup)
- `--impedance 100` with `route_diff.py` for 100Ω differential

### Bus Detection

For parallel data/address buses with clustered endpoints:
- `--bus` enables automatic bus detection and parallel routing
- Routes are attracted to neighbors, creating clean parallel traces

## Step 8: Handle Special Cases

### 2-Layer Board with Dense Components

On 2-layer boards, BGA/PGA fanout may fail for some inner pins due to
insufficient routing channels. Options:
- Accept partial fanout; router will complete remaining connections
- Skip fanout entirely; direct routing often works for through-hole PGA

**Dense 2-layer boards: treat B.Cu as a real routing layer, not a plane.**
Reserving B.Cu for a GND plane (and/or pricing it 3×) turns a congested
2-layer board into single-layer routing — a dense 2-layer corpus board's human
original carries 47% of its routed length on B.Cu and pours GND *around* the
routes on both sides afterwards; a plane-first chain on the same board left 25
nets open. On a dense 2-layer board: route signals on BOTH layers at cost 1.0
(long-haul nets cross on the back), then pour GND last (`route_planes.py`
after the signal steps — the pour flows around existing copper; 80% of human
2-layer boards pour BOTH sides this way). Power rails as pours are a minority
practice on 2-layer (≈38% of human boards) — pour them too when there's room,
but GND-both-sides is the priority. Only plane-first on 2-layer boards with
light signal content.

**Important:** If you skip fanout for a BGA/PGA component but still need to connect its
internal pads, use `--no-bga-zone <component>` to disable the automatic exclusion zone
and allow the router to enter the dense pin area:

```bash
python3 py_router/route.py board.kicad_pcb \
    --nets "*" \
    --no-bga-zone U9 \
    --output board_routed.kicad_pcb
```

Without this flag, the router auto-detects BGA/PGA zones and avoids them, which would
leave internal pads unconnected if they weren't fanned out.

### Multi-Layer Boards (4+ layers)

**Precedence: the Step 5a-tuned DENSITY GATE outranks this section.** The
survey below describes HUMAN practice, and our engine measurably matches it
only on DENSE boards. On STANDARD boards (no ≥100-ball fine-pitch array,
≤150 nets) the measured-optimal map is inner-only pours + rails as wide
traces (see the Step 5a-tuned A/B: four small boards, floods → 16 opens
total; inner-only → 0). Read this section as the Tier-DENSE playbook and
as background on why pours matter at all.

**Pour philosophy (from a survey of the human-routed corpus): pour EVERY GND
and power net that has more than a few pads, and treat pours as cheap.** Human
boards deliver power as copper pours, not tracks — a rail's ball/pad drops a
via straight into a pour instead of consuming a routed track through the
congested escape field. A dense 6-layer BGA board in the corpus that a
plane-light plan (GND-only, rails as wide tracks) left ~26% incomplete spends
~20% of its track copper on rails the human never routes at all. Concretely:

- **Which nets:** GND always, plus every power rail with more than a few pads
  — and pouring scales with layer count. Across ~400 human corpus boards,
  86% of 4-layer, 97% of 6-layer, and 100% of 8-layer boards pour power rails
  (2-layer boards: 38% — GND-only flood is the 2-layer norm). Humans pour
  even small rails (corpus median poured net ≈ 3 pads); a board with many
  rails gets many pours — human 4-layer boards commonly pour 5–15 distinct
  nets, dense 6-layer boards 10–20.
- **Which layers — any, including routing layers.** Pours are not confined to
  dedicated plane layers: 80–100% of human boards at every layer count pour
  copper ON their outer routing layers, flooding GND/rails around the
  finished tracks. The layer typology that recurs:
  - **4-layer:** signals+pours on F/B; one inner = solid GND; the second
    inner is a minority-solid choice (corpus: ~23% solid power, the rest
    SPLIT multi-rail (`/recommend-plane-mappings` Step 3b), route+pour, or
    plain routing when the board is dense and signals need to cross inner).
  - **6/8-layer:** solid GND planes nearest the outer signal layers (In1 and
    the last inner — 2/3 of human 6/8-layer boards have a solid plane
    adjacent to an outer layer), split power planes and/or route+pour in the
    middle, signals concentrated on F/B plus one inner "highway" layer.
- **High-speed nets need an UNSPLIT reference plane on the adjacent layer.**
  Whatever else moves, keep one solid (not split, not track-fragmented) GND
  plane directly under each layer that carries high-speed routes (DDR/RAM
  buses, USB HS, SerDes, RF — from `/find-high-speed-nets`). Split planes and
  route+pour layers are fine anywhere that isn't a high-speed reference.
- **Dense boards (BGA ≥ ~100 balls, DDR/SDRAM buses): keep escape-depth layers
  ROUTABLE, but still poured.** Don't let plane assignments turn the region
  around a big BGA into 2-layer routing — long-haul nets need to cross
  *through* inner layers (1–2 vias each). The resolution is order, not
  abstinence: solid planes pour FIRST (Step 1c); a layer signals must cross
  keeps its cost low (≤1.5) and gets its rail pours LATE (after the signal
  steps, like the 2-layer flow below — the pour flows around existing copper).
  Never leave a many-pad rail as pure tracks because its natural layer is
  shared with routing.
- **Check where the BGA fanout escapes landed before finalizing the plane
  layers** — a plane on a layer full of escape stubs leaves the route step
  threading its plane taps through a crowded field. Pick solid-plane layers
  the escapes avoid.

**Adapt the pour plan to the BOARD TYPE (measured across ~400 human corpus
boards, grouped by dominant component/function):**

| board type | human pour strategy |
|---|---|
| **Fine-pitch big BGA** (≥100 balls, ≤0.5 mm) | The most pour-heavy class: median poured nets 5 (4L) → 17 (6L) → 21 (8L). But median only ONE solid plane per board — a third have NONE. Keep nearly every layer routable (split + route+pour), pour many rails around the routes, and deliver every rail near the BGA by pour+via, never by tracks through the escape field. |
| **Big BGA, coarser pitch** (≥100 balls, >0.5 mm) | Same direction, milder: rails poured on 84% of boards, median ~6 nets; solid GND + split power inners are common. |
| **RF / radio** | GND-dominated: typically ONE GND net poured as many islands on every layer (coplanar grounding around RF traces); few rail pours. Keep GND pours tight to the RF path; rails as tracks are fine. |
| **Power / motor** | Heavy-current rails (V+, GNDPWR, phase outputs) are ALWAYS pours — 40–90-pad rails delivered as multi-layer copper regions, never wide tracks. Pour every supply rail on every layer it visits. |
| **Keyboard / LED matrix** | Humans pour nearly every net (rows, columns, LED chains) as small local zones on both sides of a 2-layer board. Our chain approximates this with: route both sides thin, then GND+rail pours; don't fight for a dedicated plane side. |
| **MCU / QFN, light 2-layer** | Modest: GND flood both sides (80%), rails poured on ~40–60%; a couple of pours is normal, don't force more. |

- More fanout options available.

**MANDATORY whenever any layer carries a solid plane: derive `--layer-costs`
from the plane plan and pass it to EVERY signal-routing step** (`route.py`,
the finalize's reconciliation, and retries). A measured failure mode: a 6-layer BGA
chain poured three solid inner planes and then passed NO `--layer-costs`
anywhere — signals crossed all three pours at cost 1.0, shredded them into
islands, and the board graded worse than a plane-light plan. Pour-first order
means the plane layers are already known when the signal steps run; there is
no excuse to omit this.

**Derive `--layer-costs` from the plane plan — penalize the plane-reserved
layers (issue #185).** The 4-layer default is **all 1.0**, so the router has no
idea which inner layers are about to become planes and freely routes signals
across them. Once you've decided the plane→layer map (via
`/recommend-plane-mappings` or the `route_planes` call you're about to make),
pass `--layer-costs` to the **signal** `route.py` step (and the later reconnect
passes) that makes each plane-reserved layer expensive, so signals prefer the
signal layers and leave the inner layers clean for the pour:
```bash
# GND plane on In1.Cu, power plane on In2.Cu -> penalize In1/In2 for signals:
py_router/route.py ... --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
```
- **~3× is the sweet spot on boards where F/B alone can carry the signals.**
  Any value ≥2× keeps signals off the planes and doesn't hurt completion; ≥5×
  just adds vias/copper for negligible further gain. Order matches `--layers`;
  keep the real signal layers (F.Cu/B.Cu) at 1.0. **On dense boards (BGA ≥
  ~100 balls / DDR buses) where an inner layer was deliberately left
  signal-routable (see the dense-board exception above), keep that layer at
  1.0–1.5** — 3× on the only spare layer starves the long-haul nets that need
  it (measured: a 4-layer FPGA corpus board failed 72 nets at 3×; its retry at
  1.5 was the correct call).
- **Why it matters — it's a cascade, not just tidiness.** Signals crossing a
  plane layer fragment the pour into islands; the route step's plane finalize then
  carpets the layer with island-stitching tracks. Keep signals off the plane
  layers and the planes stay whole, so the repair has almost nothing to stitch.
- **Measured on a 4-layer corpus board** (In1=GND, In2=+3.3V/+3.3VA), full chain,
  default `1.0 1.0 1.0 1.0` vs smart `1.0 3.0 3.0 1.0`, both fully connected and
  DRC-clean:

  | | default | smart 3× |
  |---|---|---|
  | total segments | 4857 | **2966 (−39%)** |
  | signal copper on plane layers | 307 mm | **44 mm (−86%)** |
  | vias | 309 | 318 (+9) |

  The 39% segment drop is the carpet disappearing because the planes stayed whole.

This is the 4-layer analogue of the 2-layer rebalance in best-practice #8 / #178:
in both cases derive the costs from how the layers will actually be used, rather
than taking the blunt default.

### Differential Pairs Present

Insert diff pair routing after fanout but before single-ended signals:

```bash
python3 py_router/route_diff.py board.kicad_pcb \
    --nets "*LVDS*" "*USB*" \
    --diff-pair-gap 0.15 \
    --layers F.Cu In1.Cu In2.Cu B.Cu \
    --output board_diff.kicad_pcb
```

**Escape layers (multi-layer boards):** like `bga_fanout.py`, `route_diff.py`
defaults to `--layers F.Cu B.Cu` only. On a 4+ layer board you MUST pass every
copper layer — when a pair was escaped by `bga_fanout.py` onto an INNER layer,
`route_diff.py` can only launch from those escaped stubs if that inner layer is
in `--layers`. Omitting it strands the inner-layer stubs and silently drops
those pairs (you'll see a low routed-pair count, e.g. 8/40 instead of 22/40 —
issue #116). Use the same copper-layer list you passed to `bga_fanout.py`; drop
`--layers` only for true 2-layer boards.

Key options:
- `--diff-pair-gap 0.1` - Gap between P and N traces (mm)
- `--no-gnd-vias` - Disable automatic GND via placement near signal vias
- `--diff-pair-intra-match` - Match P/N lengths within each pair
- `--swappable-nets "*rx*"` - Allow target swap optimization for memory lanes

### QFN/QFP Components (Perimeter Pads)

Use `qfn_fanout.py` instead of `bga_fanout.py`:

```bash
python3 py_router/qfn_fanout.py board.kicad_pcb \
    --component U1 \
    --output board_qfn.kicad_pcb
```

Creates two-segment stubs (straight + 45° fan) for each pad. On a crowded
fine-pitch edge where the surface fan has no room, add `--escape-method underpad`
(drop a through-via past each pad) and, if a boxed-in leg still drops,
`--allow-via-in-pad` so the via can sit on its own pad and stagger inward — see
"Crowded fine-pitch QFN edge" above.

Like `bga_fanout.py`, `qfn_fanout.py` ends with a `JSON_SUMMARY` carrying
`drc_grazes` (graded at `--clearance`). **Parse it after the fanout:** if
`drc_grazes.segment_segment > 0` the 45° escape stubs of two adjacent tight-pitch
pads (often a diff pair) are grazing at the wrist — re-run with a thinner
`--width` toward the fab floor until it's 0 (issue #179; see the `drc_grazes`
bullet under Step 1). All pads keep escaping (`failed` stays 0).

### Power Net Width Options

Instead of routing power separately, use `--power-nets` with signal routing:

```bash
python3 py_router/route.py board.kicad_pcb \
    --nets "*" \
    --power-nets "GND" "VCC" "+3.3V" \
    --power-nets-widths 0.5 0.4 0.4 \
    --output board_routed.kicad_pcb
```

First matching pattern determines width. Useful when not using planes.

**Size power widths for the destination pitch, not just the current.** A
0.3–0.5 mm trunk physically cannot reach interior balls of a ≤0.8 mm-pitch
BGA (at 0.5 mm pitch only one ~0.09 mm track fits between balls; at 0.8 mm a
0.25 mm trace + 0.09 clearance is a knife-edge). The power step's automatic
tap neck-down helps at the pad, but if a rail feeds MANY interior balls
(core rails like +1V1/P1.35V/VCC_1V8), a fat-track tree through the ball
field fails outright — the human originals feed such rails with zones on
every layer plus 0.09–0.2 mm necks. For those rails prefer a plane/region
(`/recommend-plane-mappings`), or set the rail's width to what the ball
field admits (e.g. 0.15–0.2) rather than the open-field ideal.

### Target Swap Optimization (Memory Routing)

For swappable signals (e.g., memory data lanes where any DQ can connect to any):

```bash
python3 py_router/route.py board.kicad_pcb \
    --nets "*DQ*" \
    --swappable-nets "*DQ*" \
    --output board_routed.kicad_pcb
```

Uses Hungarian algorithm to find optimal assignments minimizing crossings.

### Schematic Synchronization After Swaps

When routing performs polarity swaps (P↔N) or target swaps, the schematic can get
out of sync with the PCB. Use `--schematic-dir` to automatically update:

```bash
python3 py_router/route_diff.py board.kicad_pcb \
    --nets "*LVDS*" \
    --swappable-nets "*LVDS*" \
    --schematic-dir /path/to/kicad/project \
    --output board_routed.kicad_pcb
```

This updates the `.kicad_sch` files with any pad swaps made during routing.

**Shared symbols are refused, not rewritten (#489 §3).** Pin numbers live in the
file's `lib_symbols` definition, which every instance of that `lib_id` shares. When
a second component uses the same symbol — the common case for connectors, identical
channels, and multi-channel analog — the swap is **refused** for that file with a
message naming the sharers, because applying it would silently re-pin those other
components too. The units of one multi-unit part (U2A/U2B) share the definition
legitimately and are still updated. A refused swap means board and schematic
disagree on those pins: report it and tell the user to fix it by hand (or give the
component its own uniquely-named symbol) before fabricating.

**Important:** After routing with swaps, ask the user:
> "The router performed X polarity swaps and Y target swaps. Would you like to
> update the schematic to match? If so, provide the path to your KiCad project
> directory and I'll re-run with `--schematic-dir`."

Schematic sync is **disabled by default** to avoid unexpected changes. Only enable
when the user confirms they want schematic updates.

### Guide Corridors (user-drawn preferred routes)

When specific nets keep taking bad paths (or the user wants control over where a bundle
runs), the user can draw a polyline on `User.1` in KiCad and re-route those nets with:

```bash
python3 py_router/route.py board.kicad_pcb --nets "SPI*" --guide-corridor --output board_routed.kicad_pcb
```

The route follows the line as waypoints, strictly best-effort — a guide never makes a route
fail or adds vias. See `docs/configuration.md` "Guide Corridor Options" for details.

**Scope rule: do NOT draw guide corridor geometry yourself.** Suggest *in words* where a
corridor would help ("a line on User.1 south of J3, between the mounting hole and C14") and
let the user draw it; then incorporate `--guide-corridor` into the plan.

### Keepout Zones (RF / analog exclusions)

Check the board for components that warrant routing exclusions: antennas (footprint/value
keywords ANT, ANTENNA, chip antenna parts), RF modules, and sensitive analog front-ends. If
found, recommend the user draw closed polygon(s) on `User.2` around those regions and add
`--keepout` to every routing step (`route.py`, `route_diff.py`) so tracks and vias stay out
on all copper layers. Same scope rule as guide corridors: describe where the keepout should
go; the user draws it.

### MPS Layer Swap (crossing conflicts)

When MPS ordering reports crossing conflicts (nets in Round 2+), or failures show pairs of
nets repeatedly ripping each other up, add `--mps-layer-swap` to attempt layer swaps that
eliminate same-layer crossings before routing begins.

### Vertical Track Alignment

On 4+ layer boards where through-hole components need via space, `--vertical-attraction-radius`
/ `--vertical-attraction-cost` attract tracks on different layers to stack vertically,
consolidating routing corridors.

### Plane Via Placement Options (route_planes.py)

- Multiple nets can share one plane layer (Voronoi partitioning): `--nets GND VCC --plane-layers In2.Cu In2.Cu`
- `--same-net-pad-clearance <mm>` forces plane vias outside same-net pads with that edge-to-edge clearance (default places at pad center when possible)
- The pour places NO tap vias and draws NO traces (#562), so it has no via-search or blocker-rip knobs: `--max-search-radius`, `--max-via-reuse-radius`, `--close-via-radius`, `--rip-blocker-nets`, `--max-rip-nets` and `--reroute-ripped-nets` are REMOVED. Do not emit them. Plane pads are welded by the route step's pour-launch and its in-run plane finalize.

### Net Ordering Strategies

| Strategy | Flag | Best For |
|----------|------|----------|
| MPS (default) | `--ordering mps` | General routing, minimizes crossings |
| Inside-Out | `--ordering inside_out` | BGA escape routing |
| Original | `--ordering original` | Manual control |

### Useful Utility Scripts

| Script | Purpose |
|--------|---------|
| `list_nets.py U1` | List all nets connected to a component |
| `list_nets.py U1 --pads` | Show pad-to-net assignments |
| `check_orphan_stubs.py` | Find traces ending without connection |

### Debug and Visualization Options

When routing fails or behaves unexpectedly:

```bash
# Verbose output with diagnostic info
python3 py_router/route.py board.kicad_pcb --nets "*" --verbose --output board_debug.kicad_pcb

# Debug geometry on User layers (visible in KiCad)
python3 py_router/route.py board.kicad_pcb --nets "*" --debug-lines --output board_debug.kicad_pcb


# A* search statistics
python3 py_router/route.py board.kicad_pcb --nets "*" --stats --output board_debug.kicad_pcb
```

### Post-Routing Enhancements

```bash
# Add teardrop settings to all pads (improves manufacturability)
python3 py_router/route.py board.kicad_pcb --nets "*" --add-teardrops --output board_routed.kicad_pcb
```

### Advanced Routing Parameters

For difficult boards, consider tuning these parameters:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `--max-ripup 3` | 3 | Max blocking nets to rip up and retry |
| `--no-smoothing` | (smoothing is ON) | Disables #536 octolinear smoothing. A/B only — OFF measured ~20 nets worse across 147 boards |
| `--max-iterations 200000` | 200000 | A* base budget per route (self-extends to 1e7 while progressing — #529; don't tune) |
| `--heuristic-weight 2.3` | 2.3 | >1 = faster but may miss tight routes, 1.0 = optimal. 2.3 = the corpus dose-response peak (#586: 1.7 and 3.0 both worse; do not "tune it down for quality" -- measured, not intuitive) |
| `--via-cost 75` | 75 | Higher = fewer vias, longer paths; lower (25) for BGA escape. 75 = corpus-measured default (#586); 25 measured WORSE overall |
| `--grid-step 0.1` | 0.1 | Smaller = finer routing but slower; 0.05 for fine-pitch |

Manufacturing constraints (set to match your fab's requirements):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--clearance 0.25` | 0.25 | Track-to-track clearance (mm) |
| `--board-edge-clearance 0.5` | 0 | Min distance from board edge (mm) |
| `--hole-to-hole-clearance 0.2` | 0.2 | Min drill-to-drill spacing (mm) |

### Proximity Penalties

For dense boards, use proximity penalties to spread out routes:

```bash
python3 py_router/route.py board.kicad_pcb --nets "*" \
    --stub-proximity-radius 2.0 --stub-proximity-cost 0.2 \
    --bga-proximity-radius 7.0 --bga-proximity-cost 0.2 \
    --track-proximity-distance 2.0 --track-proximity-cost 0.1 \
    --output board_routed.kicad_pcb
```

## Important Notes

0. **Net-coverage invariant (Step 5b)** - Every routable net must be claimed by a stage. Since #562 the route step takes `"*"` INCLUDING the plane nets, so the only legitimate exclusions are the Step-2b impedance nets; reconcile the exclusion set against that set (symmetric difference empty), check every poured net also appears in the route step's `--power-nets`, and confirm `check_connected.py`'s unrouted list is empty at the end. This is the guard against a net (e.g. a secondary ground like GNDA) being silently dropped by every stage.
1. **Always check for GND connections** - If a component has GND pads but GND isn't being fanned out, the plane vias will handle it
2. **Fanout ALL non-plane nets** - Use `--nets "*" "!GND" "!VCC"` to fan out all nets except those handled by planes. Do NOT use `"/*"` alone as it misses nets with non-hierarchical names like `Net-(U9-Pad1)`. Unconnected nets are automatically filtered out.
3. **Order matters** - Fanout (with plane-ball drops) comes AFTER the Step 1 bare pour (#424: planes FIRST, so the fill picks up the drop vias while intact and the fragility field steers every later route), then diff pairs, then the all-nets route with the plane nets INCLUDED (#562 — the run ends with the in-run plane finalize, so there is no separate repair step), then optional GND return vias/stitching. Signals route before stitching because stitching vias can relocate around tracks, but a diff pair cannot relocate around a badly placed via
4. **Verify at the end** - Always run DRC, connectivity, and orphan stub checks
5. **Consider the analyze-power-nets skill** - For complex boards where power net identification isn't obvious, use that skill first to analyze component datasheets
6. **Consider the find-high-speed-nets skill** - For accurate GND return via distance recommendations based on actual component datasheet speeds and rise times, run `/find-high-speed-nets` before planning. The lightweight inline analysis (Step 4) uses net name patterns only.
7. **Stub layer switching is on by default** - The router automatically moves stubs to eliminate vias when beneficial; disable with `--no-stub-layer-swap`
8. **Default layer costs** - 2-layer boards default to F.Cu=1.0, B.Cu=3.0 to prefer top layer; 4+ layer boards use 1.0 for all. On **dense** 2-layer boards this 3× back-side penalty can over-bias routing onto F.Cu (top channel exhausted, B.Cu empty, excess vias, stranded pads); if completion is low or the layer balance is badly skewed, **retry with more balanced `--layer-costs` (e.g. `1.0 1.5`, down toward `1.0 1.0`)** — see "Dense 2-layer boards: rebalance layer costs" under Diagnose and Retry (issue #178). On **4+ layer** boards the all-1.0 default is plane-blind: **derive `--layer-costs` from the plane→layer map and penalize the plane-reserved inner layers (~3×)** so signals stay on F.Cu/B.Cu and the planes stay whole — see "Multi-Layer Boards (4+ layers)" (issue #185).
9. **Schematic sync is disabled by default** - After routing with swaps, offer to re-run with `--schematic-dir` if the user wants to update their schematic
10. **Rip-up and reroute is automatic** - When a route fails, the router automatically rips up blocking nets and retries (up to `--max-ripup` blockers)
11. **Component shortcut** - Use `--component U1` to route all signal nets on a component (auto-excludes GND/VCC/unconnected)
12. **Use --no-bga-zone for difficult boards** - Even when fanout is complete, use `--no-bga-zone` during routing to allow the router to find alternative paths through the dense pin area. This is especially important for 2-layer boards where routing channels are limited.
13. **Windows UTF-8 encoding** - On Windows, use `python3 -X utf8` to avoid Unicode encoding errors when scripts print special characters (like Ω for resistance). Example: `python3 -X utf8 py_router/route_planes.py ...`
14. **BGA/PGA power pins and planes** - When using power planes, BGA/PGA power pins (GND, VCC) connect most efficiently via direct vias to the plane rather than fanout routing. Create planes first, then fanout only signal nets (this is the Step 1 -> 1b order). Through-hole PGA pads automatically connect to planes on that layer; SMD BGA pads need vias placed by `route_planes.py`. This approach:
    - Reduces routing congestion (power pins don't consume escape channels)
    - Provides lower impedance power connections
15. **Rip-up depth: MORE IS NOT BETTER (measured).** On a 6-board chain A/B, `--max-ripup 5` beat 10 (+0.78 pts completion, 13 fewer connectivity items, 3 boards better / 0 worse) and 20 was worse than 10 — each extra rip level risks a permanent casualty (a ripped victim whose corridor gets taken cannot be restored), and the gains from deep ripping don't materialize because victims can almost always reroute anyway. The optimum sits in 3-5 and wobbles by board (measured: one board monotone-better all the way down to 3, another best at 5) -- the SHIPPED default is 3 (the sets-11-15 holdout showed 5 hurting ordinary boards while helping knob-sensitive ones -- #586); try 5 as a free retry variant on difficult boards (deterministic: keep whichever grades better), and escalate above 5 only as a last resort on a specific failing net, never as the opening move. Do NOT add `--max-iterations` — the router self-budgets (#529 dynamic iterations, default on, up to a 1e7 ceiling while a search progresses); see the note in the routing-step section.
16. **Guide corridors and keepouts are user-drawn** - Never draw `User.1` guide polylines or `User.2` keepout polygons yourself; suggest in words where they should go and let the user draw them, then add `--guide-corridor` / `--keepout` to the plan.
17. **Companion skills** - Defer to `/identify-diff-pairs` (datasheet-based pair detection), `/recommend-stackup` (before impedance/time-matching work), `/diagnose-routing-failures` (after failures), and `/review-routed-board` (final verification) rather than duplicating their logic inline.

## Presenting the Plan

After generating the plan:
1. Show the board summary
2. Explain any special components found
3. List differential pairs if detected
4. Highlight any length-matching or impedance requirements
5. Present each step with the command AND a brief explanation of why
6. Ask the user if they want to proceed or modify the plan
7. Offer to run the commands if approved

## After Routing Completes

### Capture Logs for Analysis

Always capture command output to `/tmp` files for later analysis:

```bash
python3 -X utf8 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "*" 2>&1 | tee /tmp/route_output.txt
python3 -X utf8 py_router/route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu 2>&1 | tee /tmp/planes_output.txt
python3 -X utf8 py_router/check_connected.py output.kicad_pcb 2>&1 | tee /tmp/connectivity.txt
python3 -X utf8 py_router/check_drc.py output.kicad_pcb --clearance <floor> --hole-to-hole-clearance <floor> 2>&1 | tee /tmp/drc.txt
```

(`<floor>` = the manufacturing floor from `list_nets.py --design-rules`, not the
0.2 default — grade DRC at the rule the board's own Constraints + fab capability
define, per #111/#115.)

### Parse Logs for Failure Analysis

After routing, parse the log files to understand failures:

```bash
# Check routing summary (last 20 lines usually have the summary)
tail -20 /tmp/route_output.txt

# Look for failed nets
grep -i "failed\|FAILED" /tmp/route_output.txt

# Check JSON summary for detailed failure info
grep "JSON_SUMMARY" /tmp/route_output.txt | sed 's/JSON_SUMMARY: //' | python -m json.tool

# Find specific failure reasons
grep -A5 "FAILED NET HISTORIES" /tmp/route_output.txt
```

The JSON_SUMMARY line contains structured data including:
- `failed_single`: List of single-ended nets with NO result at all
- `open_single`: Nets that KEPT a result whose copper still leaves pads disconnected (non-multipoint only). A run is clean only when `failed_single` AND `open_single` are both empty — a board can ship open copper with `failed_single: []`
- `terminal_restores`: `{net: outcome}` for rip victims restored at terminal failure — `full` is the only success; `full_open`/`stub` ship broken
- `stacked_copper`: Same-net duplicate copper KiCad's DRC never flags (disclosure only, not a routing failure)
- `failed_multipoint`: List of nets with unconnected pads (includes pad coordinates)
- `blockers`: Per still-failed net, which routed nets wall it off (`blocked_by` with cell counts; #409)
- `pad_pairs_connected`/`pad_pairs_total` + `pad_pairs_open`: Pad-pair routability tallies (PRR = connected/total) and per-open-net outcome — route-time failures are opens; shorts are DRC's domain (#409 follow-up)
- `multipoint_pads_connected` vs `multipoint_pads_total`: Connection success rate

### Tune mode (issue #153) — opt-in per-board feedback loop

When the user asks for **tune** (e.g. "plan routing with tune", "tune mode"),
don't just run the standard pipeline once with defaults: close the loop.
After EACH step, read the step's own diagnostics and adjust that board's
options before moving on. Off unless requested — the standard plan stays
deterministic and fast.

Rules of the loop:
- **Bounded, guided adjustment — not a grid sweep.** At most 2–3 targeted
  re-runs per step, each driven by a diagnosed failure mode (the symptom→knob
  table below and the failure-pattern table in Diagnose and Retry). Never
  loosen below the fab/board-constraint floor.
- **Signals to read after each step:** the `JSON_SUMMARY` line (failed nets,
  `rescue` block, `single_ended_diff_pairs`/`failed_diff_pairs`,
  `drc_grazes`), the FAILED NET HISTORIES block (`preexisting_blockers`
  hints, `no rippable blockers`, iteration exhaustion), fanout escape
  tallies (unescaped balls), and plane-step tap/`ripped`/`STILL FLOATING`
  reports. `/diagnose-routing-failures` automates most of this.
- **Symptom → knob map** (beyond the Diagnose and Retry table):
  - Fanout drops balls in one quadrant → re-run that fanout with
    `--escape-method dogbone` (`underpad` if no gap sites exist), a smaller
    via from the fab ladder (0.30/0.15 → 0.25/0.15), or different
    `--primary-escape` direction.
  - Signal step fails a cluster of long cross-board nets while an inner
    layer is plane-reserved → revisit the plane→layer map (dense-board
    exception above): free one inner layer, drop its `--layer-costs` entry
    to 1.0–1.5, re-run the failed nets.
  - `preexisting_blockers` hints repeat for the same nets → re-run those
    nets with the hinted `--rip-existing-nets` set (the engine now
    self-escalates once in reconciliation; a manual retry may widen the set).
  - Power multipoint pads fail inside a BGA courtyard → shrink that rail's
    `--power-nets-widths` entry toward the ball-field width (0.15–0.2) or
    promote the rail to a plane/region and re-run.
  - Diff pairs deferred single-ended → re-run the pairs with smaller
    `--diff-pair-gap`/width/vias toward the fab floor (keep `--impedance`).
  - The route step's plane finalize ships tap failures with fill nearby →
    re-run the route step at the advanced fab tier so smaller tap vias fit
    (or, outside the chain, `repair_planes` with a larger
    `--max-search-radius`).
  - A handful of nets fail on a NOT-saturated board (few failed nets, short
    detours available, failures share a corridor with early-routed nets) →
    try a **failed-first split**: re-run the step as two invocations, first
    `--nets <the failed nets>` on the clean input, then everything else to a
    fresh output. This is the one retry where naming nets is the POINT (you are
    changing the order), so accept it knowingly: a manifest that names nets
    cannot be A/B'd afterwards, because a replay hands the baseline a rescue
    fitted to its own failures and penalises every engine change that fails a
    different net. An ordinary retry should use `--nets '*'` instead -- route.py
    skips already-connected nets, so a wildcard retries exactly what is still
    broken. Ordering is the cheapest knob but rarely decisive:
    measured on four corpus boards of varying density, an automatic
    failed-first restart NEVER beat the normal order (twice it graded
    worse), so an in-engine restart was tried and removed — only reach for
    this manually when the failure histories actually show corridor
    competition, and expect it to matter on few boards.
- **Explainability:** keep a short tuning log per board — which knob changed,
  the before/after metric (completion / DRC / coupled pairs), and whether it
  helped. Revert a change that didn't help before trying the next.
- **Honest gates:** grade every accepted retry with `check_connected` AND
  `check_drc` at the routed clearance (plus the kicad oracle for final
  boards) — never accept a retry that trades new DRC for completion.

### End every chain on route.py

**A chain's LAST board is what ships, and only `route.py` finalizes planes.**
Since #562 a pour alone connects nothing and `route_diff.py` runs no plane
finalize: a chain that ends on a bare `route_planes.py` re-pour or on a diff
step writes a final board no weld/oracle pass ever verified. That is a PLAN
ERROR, not a tuning choice (set5-0805 evidence: dilemma and ghoul ended on a
bare re-pour and both shipped disconnected; core1106 simply stopped after a
failed retry). If you re-pour or re-run diffs late, ALWAYS follow with a
`route.py` step whose `--nets` covers the plane nets (even `--nets GND`
suffices — its in-run finalize welds and verifies against KiCad's fill).

### Step 9: converge — score the board, pick a lever, repeat

**Full procedure, worked example and ledger schema:
[`../plan-pcb-placement-and-routing/references/convergence.md`](../plan-pcb-placement-and-routing/references/convergence.md).** The summary below is
the part you must not get wrong.

**A chain that ran is not a board that is done.** The failure this step exists to
prevent is concrete: a board went out at **39 of 44 nets connected, 762 DRC
errors, and 141 of 141 vias below its own spec**, and every tool in the chain
reported success. Nothing looped back, because nothing had measured the board.

#### 9.1 — Score it. The router's opinion is not evidence.

```bash
python3 -X utf8 .claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py \
    board.kicad_pcb --intent floorplan.json \
    --min-track-width 0.15 --min-via-diameter 0.6 --min-via-drill 0.3 \
    --net-min-widths wk/net_min_widths.json \
    --impedance-nets '<every net with a reference-plane clause>' \
    --length-groups '<every length-matched group>' \
    --json wk/score_iter3.json
```

**Every one of those flags is what makes its clause reach `blocking`. A component
with no flag reports `ungraded`, which is not a pass.** The pattern is identical
each time, and it is how a HARD clause ships unmeasured:

| flag | without it | measured worth on one board |
|---|---|---|
| `--net-min-widths` | `undersized` sees only BOARD-WIDE floors, so a clause naming ONE net — a 0.8 mm pair, a 0.4 mm rail — is invisible | `net_widths` 5, while `undersized` read 0 |
| `--impedance-nets` | the component returns *"no --impedance-nets given"* and a plane-continuity clause is never checked at all | `impedance` 10 — 68 reference crossings, 63 segments over void |
| `--length-groups` | length matching is ungraded | — |

Same board, same copper: **`blocking` 12 without those flags, 27 with them.** A
run that reports 12 has not found a better board; it has looked at less of it.

**Glob-list flags take SPACE-separated patterns** (`--impedance-nets 'USB_*'
'QSPI_*'`); comma-joined tokens are also split, and a token matching NOTHING
routes the impedance component to `unknown` → **exit 4**, never a silent pass
(run 5 scored several iterations with a comma no-op before this was fixed —
the score was vacuously clean). **Assert non-vacuity on the FIRST scoring run**
of every chain: check `impedance.nets_analyzed` equals the number of nets you
named, exactly as you assert `ran == true`. A vacuity discovered at iteration
9 invalidates every earlier score.

Also **read `net_widths.patterns_matching_no_routed_net`.** A width clause on a
net with NO copper never appears in `net_widths` — the component only walks nets
that HAVE segments — so an unrouted net's width requirement lands in that list
and nowhere else.

**And `blocking == 0` is not the whole gate when the repo ships its own spec
checker.** Some clauses are not expressible to `board_score` at all: an absolute
maximum length, a symmetry match between two *series chains* through a resistor,
a via ban per leg. If the repo has a `check_spec.py` (or equivalent), run it
**beside** `board_score.py` every iteration, treat a HARD failure as blocking even
when `blocking` reads 0, and wire it into `place_route_loop --accept-cmd` so the
inner loop stops accepting rounds that break it.

**Produce:** the command above, every iteration, on the board you just wrote.
**Read:** `blocking`, `blocking_by`, `ungraded`, `unknown`, `quality`.
**Decide:** `blocking == 0` → go to 9.4. Otherwise pick the lever by **9.1a**,
NOT by the largest `blocking_by` entry.

##### 9.1a — CONNECTIVITY FIRST. The largest number is not the lever.

The obvious rule — *"the biggest `blocking_by` entry names the lever"* — is wrong,
and it wrecked a run. On a board with 5 nets carrying **no copper at all**, the
biggest entry was `drc: 18`, of which **16 were grading artifacts**. The loop spent
eleven iterations on clearances while five nets sat dead, and the board could not
have booted.

**Work the components in this fixed order, regardless of size:**

| order | component | why it outranks the rest |
|---|---|---|
| 1 | `unrouted` | a net with no copper is a dead wire. Nothing else matters while one exists. **Run `converge.py where BOARD --nets <names>` before touching a parameter** — it names the gap endpoints and the foreign copper walling them in, per layer, nearest-first (9.1b-ii). **And READ the focus panels** — image read-case 3: `render_placement --summary-json wk/routeN.json --focus` classifies pocket-vs-scattered in one look, BEFORE the first lever. Guessing from the score is how eleven iterations went to clearances while five nets sat dead |
| 2 | `broken` | a net in N pieces is N−1 dead wires. **Read `components.broken.nets`, not the count** — see below; the count alone is not a work list and a loop driven on it does not move |
| 3 | `net_widths`, `undersized` | real copper, wrong size — fixable by re-routing what is already there |
| 4 | `floorplan` | placement or intent |
| 5 | `drc` | **last, and only after auditing it — see below** |

`unrouted` and `broken` are the **ratsnest**: they count connections the board is
supposed to have and does not. They are never artifacts, never a grading choice,
and they map one-to-one onto whether the thing works. Drive the loop on them.

##### 9.1a-ii — `broken` needs a WORK LIST, and one tool per class

`unrouted` is actionable from its names alone: the net has no copper, so route
it. **`broken` is not.** Before you can act you need three more things — which
net, how many pieces (a 5-way split and a 2-way split are different jobs), and
*where* the stranded pads are. Measured failure mode: a run drove `unrouted` to
0 using `net_widths`' per-net detail as its model, then left `broken` at **14
across two iterations**, because `blocking_by.broken: 14` is a number with
nothing behind it.

`board_score.py` emits the list under `components.broken.nets`:

```jsonc
"GND":      {"components": 5, "joins_needed": 4, "handler": "repair_planes",
             "stranded_pads": [{"x":137.72,"y":66.05,"layer":"F.Cu","ref":"SW1"}, ...]},
"VCC1V1":   {"components": 4, "joins_needed": 3, "handler": "route",
             "stranded_pads": [{"ref":"U1"}, ...]},
"FLASH_CS": {"components": 2, "joins_needed": 1, "handler": "route",
             "stranded_pads": [{"ref":"R1"}]}
```

`joins_needed` sums exactly to `blocking_by.broken`, so the list is complete and
you can see what each entry is worth. **Sort by it** — above, GND alone is 4 of
the 14, and seven single-join nets are worth 1 each.

**`handler` names the step, and it is a FACT off the board, not a guess:** it is
`repair_planes` when the net has a zone (see `broken.poured_nets`,
read from the board's own `(zone (net "…"))` blocks) and `route` otherwise.
`route.py` cannot tap a pour, so a stranded plane pad handed to it is work that
cannot succeed. Measured: `broken` sat at **14 across two iterations** of
`route.py` calls and fell to **11 in one** `repair_planes` call once
the plane net was separated out.

**`poured_nets` is that handler decision and NOTHING else — it is not a list of
plane nets, and it is not a safe `--ignore-nets` population.** It means "this net
has at least one zone", which on a board that pours signal nets includes them:
measured on neo6502, its 61 nets covered **332 of 545 pads (72%)**, all of
`/A0`–`/A15` among them. A run read it as "the planes, ignore those" and removed
most of the board from its own render. The field publishes this sentence itself,
as `broken.poured_nets_meaning` — read it there rather than inferring from the
name. Every net name `board_score` publishes is now checked against the board
(`net_name_audit`); a non-zero `unknown_count` is a bug in the instrument, not a
finding about the board.

**One tool per class — they are not interchangeable, and using `route.py` on all
of them is why the count does not move:**

| the break | the tool |
|---|---|
| a **plane net** (GND, any poured rail): stranded pads that cannot reach the pour | `repair_planes --rip-blocker-nets`. `route.py` will not tap a pour |
| a **multipoint** signal/power net: some MST edges landed, one did not | `route.py --nets <that net>` — and read **`failed_multipoint`**, which is where its failure is reported |
| a break whose stranded pad sits on a **DNF / do-not-fit** part | **not a defect.** Chasing it never converges. Say so once, with the ref, and exclude it from the target set |
| a break at a fine-pitch pad with no room for a tap via | smaller `--via-size`/`--via-drill`, then finer `--grid-step` — the Step 5 ladder |

The `ref` on each stranded pad is what tells these apart, which is why it is in
the list. A break on `[R1]` where R1 is unpopulated and a break on `[U1]` are the
same number and completely different work.

##### 9.1b — Audit `drc` before you believe it

(And when two DRC instruments disagree, audit the PASSING one first — confirm
it implements the clause in code, not just in a docstring; see the
cross-instrument rule in the verifier section. Reconciling them is a
`--kind systemic` ledger entry.)

`check_drc` grades the whole board at **one clearance**. A board with more than one
net class therefore reports violations that are purely a grading choice. The
signature is unmistakable — **many violations, all the same net pair, all the same
overlap**:

```
SEGMENT-SEGMENT  USB_DM <-> USB_DP   Overlap: 0.010mm     x15
PAD-SEGMENT      USB_DP  <-> USB_DM   Overlap: 0.010mm     x1
```

0.010 mm is exactly `0.16 − 0.15`: the Default class grading a pair whose own class
permits 0.15. Re-grade at the tighter class and they vanish:

```bash
check_drc.py board.kicad_pcb            # 18 violations
check_drc.py board.kicad_pcb -c 0.15    #  2 violations
```

**Before letting `drc` drive anything:** group the violations by (type, net pair,
overlap). Any group that is large, uniform, and sits within a µm of a
class-clearance difference is an artifact of the grading scalar, not a defect.
Quote both numbers in the report and say which classes each applies to. Never pick
the flattering one silently.

##### 9.1b-ii — Tools that already answer this, which nothing in a chain calls

A whole convergence went by hand-rolling worse versions of four of these. Before
writing a script to answer a question, check whether one of them already does.

| you want | run | why it beats the obvious thing |
|---|---|---|
| where is the gap, and what is walling it in | `net_forensics.py --nets N --radius 1.0` | per net: the connected ISLANDS, the exact unclosed gap endpoints, and an inventory of the foreign copper around each gap — **named, per layer, nearest-first**. Better than a ratsnest, which tells you two pads are unjoined and nothing about why |
| the honest unconnected count | `kicad_unconnected.py board --items` | KiCad's own DRC, and it **refills the zones itself** — which is 9.1c's whole problem, already solved. Exit 4 = items remain, 3 = no oracle (NOT clean) |
| WHERE the DRC violations sit, as a picture | `check_drc.py board --render wk/drc/` | one cropped panel per spatial cluster, red rings at each violation, count/types/rect in the caption — image read-case 7. The panel shows WHERE; the violation records say how much |
| the endgame work list, join by join | `kicad_unconnected.py board --pairs-json wk/pairs.json`, or `converge.py where BOARD --oracle` | each remaining join as an exact net + pad↔copper endpoint pair (x/y/layer/kind) — the JOIN SPEC for a scoped route, no re-deriving from prose. `where --oracle` prints the pairs then runs forensics on exactly those nets |
| what kind of failure is this | `converge.py where` / the router's own hint | the hint names the flag and the nets (9.3b); it diagnoses better than the score does |
| where should this part go, facing which way | `converge.py poses BOARD --ref R` | ranks legal (x, y, rotation) poses by placement cost in **milliseconds**, with a per-component breakdown, and `--route` pays for tier 3 on only the top few |
| will this hand join fit, BEFORE committing it | `check_join.py BOARD NET x,y,layer ... via:x,y` | stages the candidate polyline+vias onto a copy of the board and diffs the REAL check_drc engine (netclasses, `.kicad_dru`, rotated pads, edge, hole-to-hole), plus missing-via and same-net-stack checks DRC omits. Exit 0 clean / 1 violations. Rung 8's condition 3 |
| is this even the engine I pinned | `route.py --capabilities` / `krt_capabilities.py --require` | a chain can otherwise run green against a clone missing the module it depends on. **Spelling is `module:--flag`, WITH the dashes.** And ground-truth a PLANE-step flag with `--help`: `--require` scans imports one level to catch shared registrars, and both plane scripts import `route.py` — so they used to inherit its whole vocabulary and answer OK for flags argparse rejects with exit 2 (fixed, but the lesson stands: a capability gate is evidence, not proof) |
| step back to iteration N | `converge.py step-back --iteration N` | byte-exact, because the board is addressed by content instead of by a path three iterations overwrote |
| re-run what iteration N did | `converge.py replay --iteration N` | replays the recorded argv. If it refuses, the ledger recorded prose instead of a command — fix the ledger, not the memory |

**Trust order when instruments disagree on connectivity: the KiCad oracle
(`kicad_unconnected`) > `net_forensics` islands > `board_score` components >
route.py's own JSON_SUMMARY tallies.** The router's multipoint model reported
25/27 pads connected on a net the oracle showed in **7 islands** (run 6,
VCC3V3). Router tallies pick the next lever; **only the oracle accepts an
iteration.** When a tally and the oracle diverge, that divergence is itself a
`--kind systemic` finding — file it, don't average it. (route.py now runs the
oracle itself at end of run — `oracle_check`/`oracle_open` in JSON_SUMMARY —
and a `fragmented_nets` key names pad-connected nets whose copper is several
KiCad islands. `oracle_open` feeds its own reconciliation; `fragmented_nets` is
DISCLOSURE ONLY -- it names the splits, nothing re-queues them, so a fragmented
net there is YOUR next action (route it by name). `oracle_check: unavailable`
means the run had no kicad-cli and you are on in-process grading alone.)

**The divergence is a MODE, not an event.** After the model's success channel
is caught lying ONCE on a board (`failed_single`/routed tallies vs oracle
opens), demote it for the remainder of that board's endgame: the oracle's
`--pairs-json` work list is the only open-set, every accept runs the oracle,
and after every FAILED call diff the board itself (per-net segment/via
counts, duplicate vias at identical coords) before trusting "no change" — a
failed rip-restore can write fragments while reporting success. Run 7 held
the trust order per event but kept consuming the model's tallies lap after
lap, and paid one oracle run of latency for each of three defect classes.
(The engine now counts routed-but-OPEN nets in `open_single`, grades
terminal restores before claiming them — `terminal_restores` — and dedups
stacked vias with a `stacked_copper` disclosure; a summary carrying those
keys has already had this class of lie audited once, which narrows the gap
but does not repeal the trust order.)

**Ratchet floors measured by a refill-jittery instrument (kicad-cli DRC) flap
at exact equality.** (1) A single exit-4 at floor==count is a RE-MEASURE
event, not a regression — re-run the checker once before reverting anything.
(2) Register or lower a floor only to a count observed twice. (3) Name
at-equality rules in the promote note as jitter-exposed, and keep the flap
log.

##### 9.1c — The authoritative ratsnest needs the zones FILLED

`route_planes` writes a zone **outline** with no `filled_polygon`. Until something
fills it, every KiCad-side check reads the pour as empty. Measured on one board,
same file, fill the only difference:

```
unfilled -> kicad-cli pcb drc:  48 unconnected items
filled   -> kicad-cli pcb drc:  15 unconnected items   == what check_connected says
```

So: **fill before you grade, and then the two agree.** If `check_connected` and
`kicad-cli` disagree by a lot on a board with a pour, the fill is the first
suspect — not the checker.

**Use `kicad_unconnected.py`, which refills for you** — it exists precisely for
this, and a hand-rolled fill has a trap the tool does not:

```bash
python3 -X utf8 py_tools/kicad_unconnected.py board.kicad_pcb --items
```

For the ENDGAME — the last few opens after the all-nets route — add `--pairs-json` (or run
`converge.py where BOARD --oracle`): it writes each remaining join as an exact
net + endpoint pair, which is the work list a scoped single-net call consumes.
Run 5 spent its endgame re-deriving those endpoints from the `--items` prose,
one join at a time.

If you must fill in place (to hand a filled board to something else), note that
`pcbnew.LoadBoard(...).Save()` **rewrites the sibling `.kicad_pro` and deletes
every non-Default net class**, leaving the netclass patterns orphaned. A board has
shipped that way. Restore the project afterwards and assert the classes are back
rather than trusting a success message.

**And re-assert the net classes afterwards.** `pcbnew.LoadBoard(...).Save()`
rewrites the sibling `.kicad_pro` and **deletes every non-Default net class**,
leaving the `netclass_patterns` orphaned. A board has shipped that way. Restore the
project after the fill, and assert the classes exist rather than trusting a
success message.

Three rules about that number:

- **`blocking` must reach 0 before a board is deliverable.** It is
  `unrouted + broken + drc + undersized + floorplan + impedance + length`.
  `quality` (vias, copper length) is a **tie-break only**, compared once
  `blocking` is 0 — otherwise a router buys off a disconnected net with a lower
  via count.
- **Pass the spec's size floors when the spec is tighter than the fab.**
  `check_drc` defaults to the fab minimum for the layer count. That is why 141
  vias at 0.25 mm graded clean against a 0.6 mm spec. If the spec gives numbers,
  pass them.
- **`ungraded` is not `passed`.** A component with no `--intent`, no
  `--impedance-nets`, no `--length-groups` is *unexamined*. Say so in the report;
  never let it read as clean.

**`place_route_loop`'s own `ACCEPTED` / `REJECTED` is NOT a quality verdict.**
`better()` (`place_route_loop.py:358`) compares `failures` and `iterations`, both
from route.py's own `JSON_SUMMARY`; it never runs a checker. Treat it as a cheap
pre-filter and **re-score with `board_score.py` before believing it.**

**It is also spec-blind, and `--accept-cmd` is the fix.** `better()` compares
failures then iterations; nothing in a route summary tells it a net exceeded a
maximum length, took a via where none is allowed, came out under a required
width, or drifted a decap past a proximity limit. On a board with a real spec
those are what decide whether a placement improved, so the loop will accept a
round that broke one and print ACCEPTED. Pass
`--accept-cmd 'CMD'` and the loop asks your judge instead:
`CMD <placed> <routed> <route.json>` printing one line `SCORE=<float>`, lower
better; a non-zero exit or a missing SCORE rejects the round.

#### 9.2 — Budget: 100 iterations per board, and they are cheap if you spend them right

**The budget is 100 per board.** Not 20 — 20 was set when every iteration meant a
full chain re-run, and that assumption is wrong (9.3a). A scoped retry takes
seconds, so a hundred of them is an afternoon, not a week.

**Count three kinds separately, and say which you are spending:**

| kind | what it does | example |
|---|---|---|
| **completion** | changes the copper: routes a net, heals a separation, fixes a width | `route.py --nets QSPI_SD1 ... --rip-existing-nets ...` |
| **placement** | moves footprints: a quench, a repair, a reconstruction — connects nothing, tunes no instrument | `place_seed --repair`, `place_reconstruct`, a 0c quench, a loop round |
| **systemic** | changes how the chain routes, measures or grades — no net gets connected by it | pinning the fab floor, restoring net classes, filling zones, fixing a checker |

(`placement` exists because two runs had to file placement repairs as
`systemic` for want of a kind, and `status`'s systemic-share warning cried
wolf about runs that were spending their budget on the board.)

Systemic iterations are necessary and they are not progress. A run once spent
**nine of eleven** on them, moved `blocking` every time, and finished with five
nets carrying no copper. **If three consecutive iterations are systemic, stop and
ask what is actually unconnected** — you are tuning the instrument, not the board.

Record `"kind": "completion" | "placement" | "systemic"` in every ledger entry.
The final report states all three counts.

```bash
python3 -X utf8 py_router/route.py board.kicad_pcb --list-groups --group-by auto
```

**The per-group budget needs groups that are separately convergeable — not just
groups that exist.** Test each candidate against all three:

1. its parts occupy a **distinct region** (not interleaved with other blocks),
2. its nets are mostly **internal** (`--list-groups` prints touching/internal),
3. routing it can **succeed or fail on its own**, without the others' copper.

Fail any of them and it is a *label*, not a convergence unit: take the per-board
budget and say so. A board of functional modules sharing one congested centre is
the common case — iterating per module there routes a fraction and reports
success on that fraction, which is the same defect the `route.py --group` rule
warns about.

**`kicad` groups exist on 0 of 27 boards *in this repo's corpus*** — that figure
is about KRT's own test boards, not about boards in general. A generated board
(e.g. Zener `.zen`) carries one `kicad:` group **per module**, so the naive
reading of "groups exist → per-group" authorised **8 × 20 = 160 iterations** on a
42-part board whose modules all fight over the same 21 mm of width. Take the
per-board budget there.

**Do not invent groups to iterate over** — a `sheet` block of 16–83 parts moved
on no board tried, so iterating per sheet-block burns the budget on a lever that
does not move.

#### 9.3 — Cheapest lever first, and revert what did not help

##### 9.3a — RE-ENTER AT THE FAILING STEP. Do not re-run the chain.

The single most expensive mistake available here. A full chain run is 3–5 minutes;
re-routing three nets from the board that failed them is **seconds**. The ledger
already records `parent_sha` per iteration precisely so you can go back to it
(`converge.py step-back` checks it out byte-exact).

```bash
# NOT: bash chain.sh          (re-seeds, re-places, re-routes everything)
# THIS:
route.py wk/r4.kicad_pcb wk/i15.kicad_pcb --nets QSPI_SD1 ...
```

Re-run the chain only when a **placement** changed (which invalidates every routed
board downstream) or when you are producing the final artifact. Everything else is
a scoped retry on the board that already failed.

**A scoped `--nets` retry on a net that is ALREADY CONNECTED is a no-op.** The
router has nothing to improve there: the escalation ladder never fires, and the
copper comes back byte-identical. To change the geometry of copper the router is
happy with you must **rip** it (`--rip-existing-nets <exact names>`) or route the
whole board. Naming it in `--nets` alone changes nothing. Run 20 spent a lap
discovering this — a targeted via fix produced vias at identical coordinates in
both boards.

##### 9.3b — READ THE ROUTER'S HINT. It names the flag and the nets.

When `route.py` fails a net it prints the fix, and it is usually right:

```
ROUTE FAILED - no rippable blockers found
  Hint: the blocking copper belongs to pre-existing net(s) 'QSPI_SD2' 'QSPI_SS'
  'VCC3V3' (committed by an earlier run/step), which this run is not allowed to
  rip. Retry with --rip-existing-nets 'QSPI_SD2' 'QSPI_SS' 'VCC3V3' ...
```
```
  Hint: ... boxed in by static obstacles ..., not by congestion. the geometry
  is ALREADY at this board's declared floor (clearance 0.15, track 0.15), so
  there is nothing left to pair a finer grid with. ... THIS IS A
  PLACEMENT/FLOORPLAN FINDING, NOT A PARAMETER ONE
  (current: grid 0.05, clearance 0.15, track 0.15 mm)
```

**READ THE `(current: ...)` TAIL BEFORE ACTING ON THIS HINT.** It used to
advise `--clearance 0.15 --track-width 0.15` unconditionally, so on a board
routing at exactly 0.15/0.15 it recommended the values already in force and the
only novel token in the sentence was the grid. It now derives the advice from
the board's own declared floor and says outright when the parameter family is
spent — but a *saved* log from before that change still carries the old wording,
and the tail is how you tell.

On one board these two hints, applied, took `unrouted` from 5 to 0. The router
diagnoses better than the score does — the score said `drc`, the router said
"rip these four nets", and the router was right.

##### 9.3b-ii — Carry `--fab-overrides` on EVERY retry when the spec floor is tighter

A scoped retry is a fresh `route.py` call, and it resolves its floor from the fab
tier unless told otherwise. Two things then happen quietly: the **per-net rescue
re-routes a failed net AT the tier floor**, and the `standard`→`advanced` tier
escalation is allowed, which is what puts sub-spec vias on a board that asked for
big ones. Both report the net routed.

So every route call in the loop — not only the first one — carries
`--fab-overrides <the spec file>` when the spec is tighter than the tier.
(There is no longer a width-clause flag to add beside it: `--track-width-floor`
was REMOVED in 53a5a16e.) Measured, one such file took a board's
`undersized` from **169 to 0**. Check `min_clearance_used` in the `JSON_SUMMARY`
afterwards: it is the only place a floor that was silently loosened shows up.

##### 9.3c — Ripping blocking nets IS a sanctioned lever

`--rip-existing-nets` rips named nets, re-routes them in the same run, and reports
honestly if one cannot be. It is often the **only** way past copper an earlier step
committed. Use it — with four rules, each of which cost a wasted iteration to
learn:

1. **Scope the rip.** Start with the set the hint names, then bisect if you want a
   minimal one. Do not reach for `'*'` — and subtract **large multipoint rails**
   from the hint set before using it: rip a rail as collateral and every one of
   its pads opens at once (run 5: one collateral rail rip opened **19 pads** and
   cost the iteration). Rip leaf/2-pad nets by exact name; a rail that truly
   blocks gets its own deliberate, single-net call.
2. **A ripped net returns at the CALLING command's parameters, not the ones it was
   originally routed with.** Ripping a 0.8 mm USB net from a plain signal call
   brings it back at 0.16 mm and silently destroys the spec geometry. **Whenever
   the rip set contains a width-bearing net, pass its `--power-nets` /
   `--power-nets-widths` (or `--impedance`) in the same call.** And the rule
   does not extend to dru rules — a net routed under a staged/lifted dru
   cannot be re-made by any call that reads the full sibling dru (see Step 5's
   dru-has-no-pin bullet).
3. **One net per call — with ONE scoped exception, the JOINT CO-ROUTE.** Routing
   two unrelated nets together let the second rip the first — reported as `1/2
   routed` twice running, a different net each time; sequential single-net calls
   connected both. The exception (run-23): when forensics NAME the committed
   copper caging a net (`converge.py where --oracle`, `net_forensics --json`,
   a check_join refutation naming the wall), route the caged net **and exactly
   its named blockers in one call**, blockers also in the rip set:

       route.py ... --nets <net> <blocker...> --rip-existing-nets <blocker...>

   Sequential calls CANNOT solve this shape — each re-commits a blocker the
   next call must rip (run 23 measured it: 12 single-net laps, 6 of them
   zero-delta, blocking flat for 18 consecutive laps, while the one joint
   co-route lap negotiated the pocket). The scope is the discipline: the
   co-route set is the FORENSICS' list, never "the nets that keep failing".
   Rule 2 still applies to every member (width-bearing blockers keep their
   widths), and take this rung EARLY — right after the first failed
   single-net retry with a named wall — not as a last resort.
4. **A glob does not override a lock.** `--rip-existing-nets 'QSPI_*'` silently
   skips a locked or protected net (#521) while the router keeps asking for that
   exact rip. Name it EXACTLY (the exact-name override now reaches the in-run
   ladders too, not just the pre-run filters), and if it is KiCad-locked,
   nothing overrides that — unlock it or route around it. To protect a
   SINGLE net YOU verified (not just matcher-produced ones), pass
   `--protect-nets <name>` on the step that routes it — the protection
   persists in the `.kicad_pro` and every later step's rip machinery honors
   it. **A GROUP routed together must NOT be protected on its own pass**: the
   protection binds that same call's in-run ladder, so the bus can no longer
   rip/reorder itself (measured, run 6: 6/7 with `protected_skipped` on the
   group's own QSPI pass; 7/7 without). Protect the group on the NEXT
   committing step instead; the `.kicad_pro` record carries it from there.
5. **Tap passes over routed copper are a one-way door.** With the pour-first
   order (#424) the Step 1c pour ambushes nobody — its taps land on an empty
   board and every later route sees them from the start. The door is the
   LATE tap passes: the plane FINALIZE (`--add-gnd-vias`/`--stitch-*`), the
   repair, and any re-pour over a routed board. Never name a
   geometry-constrained net in a rip set after those have run: the corridors
   it used are gone (run 5 measured the static frontier at **10,903/10,939
   cells** around the wrapped nets — the rip could only put the copper back
   where it was, minus luck); their rips are for freeing a blocked pad,
   never for improving a route. **The door also closes on UNROUTED pads**:
   a tap carpet consumes every open pad's escape channel and is not
   rippable copper (run 6: 5 bare pads at a late pour, never recovered) —
   and no gate stops you: the exit-3 refusal and its `--allow-bare-pads`
   override were removed in 5832e4eb (the empty-board 1c pour was the
   exempt case). Connect every pad first, or accept losing it. (Cross-ref:
   the Step 5 ordering block says the same from the other side.)

For plane-net pads that cannot reach their pour, the equivalent is
`repair_planes --rip-blocker-nets` (out-of-chain only; it leaves the ripped
nets unrouted for a following `route.py` pass — never re-route them in-step,
#141. In-chain, route.py's own finalize does the whole rip-and-reconnect). Budget for it: on a
dense board it can run **20× longer** than the plain repair, so start it early
rather than discovering the cost at the end.

##### 9.3d — Classify the blocker, then pick

Never spend a full-chain iteration on something a parameter fixes. Classify the
top blocker on the exact keys, not on impressions:

| evidence | verdict | where to go |
|---|---|---|
| failures cluster into ≤2 pockets (`--focus` panels), their refs share one block, `blockers` non-empty | **floorplan** | back to **Step 0e** — re-zone. A 3 mm nudge cannot move a block 80 mm |
| failures scattered, `blockers` non-empty, every failing ref is a ≤40-pin passive | **placement detail** | back to **Step 0c**, `place_route_loop` with the caps above |
| `blockers` empty; `boxed_in` names the net; **geometry has travel** above the board's declared floor | **parameters — geometry first** | shrink track/clearance/via toward that floor, and pair a finer grid **with** it. The grid is the COMPLEMENT, never the lever alone: a finer grid resolves the same obstacles more precisely, it does not make a gap wider |
| `blockers` empty; `boxed_in` names the net; **geometry at the declared floor** | **placement / floorplan** | the parameter family is spent. Measure it with `check_reachability` on the **copper-free** board and re-enter placement. Measured (run 20): `0.05 -> 0.025 -> 0.0125` left `unrouted` at exactly BUSY / Net-(U4-XTAL_P) / SCK at every resolution, ~40 min. **L4 refuses a grid lever here** unless you pass `--accept-boxin "<reason>"` |
| 2-layer board, heavy F.Cu skew, via count far above a hand layout | **parameters** | layer-cost rebalance, below |
| `oob_count` or `overlap_area` rose after the last placement | **the placement is illegal** | discard it; do not route it |
| `check_floorplan` exits 4 with `zone_containment` | **intent violated** | fix the placement to match, or say why the intent changed. Do not quietly rewrite the intent to match the board |
| a whole net has no copper while `pad_pairs_connected` looks healthy | **coverage bug** | the Step 5b ledger — not a placement problem at all |
| `undersized` non-zero | **parameters** | re-route at the spec's width/via. Placement is not the lever |
| a **maximum-length clause fails** and the net's own geometry pass ran at the default `--heuristic-weight` | **parameters — rung 1, seconds** | 1.9 is inadmissible; it returns a path up to ~1.9× optimal. Re-run **that pass**, on **its own input board**, at `--heuristic-weight 1.0` with a finer `--grid-step` (the #529 dynamic budget self-extends; do not pass `--max-iterations`), then re-measure routed:straight-line. Measured: 44.50 mm → 7.73 mm against a 7.71 mm direct. **Do not go to placement before this.** See Step 2c |
| `--heuristic-weight 1.0` **on the net's own FIRST pass**, on a board carrying only what must precede it, did not change the length | **placement** | now the router genuinely had no shorter path. Signature: routed length far above the straight-line pad distance *and stable under an admissible search*. Go to `place_route_loop` — see the warning below, it needs BOTH `--target-nets` and `--accept-cmd` to see this at all. **A null measured on a SATURATED board proves nothing** — one run tested 1.0 at iteration 4, after fanout, USB and every signal were committed, got a byte-identical board, and recorded "no shorter path exists at this placement". Re-tested on the first pass that lays the net's copper, the same flag was worth 5.8× |
| `unrouted` names a plane net | **the pour step** | it was excluded and never poured — Step 1c (or the Step 3 finalize / Step 5 repair), not placement |
| the log names **pre-existing nets** it is "not allowed to rip" | **rip lever** | 9.3c — `--rip-existing-nets` with the set it named |
| a net fails on ONE layer at every grid and rip set, and routes instantly with a second layer | **the single-layer constraint is the blocker** | not a router failure. Report it against the requirement that imposed the layer restriction, with both measurements |
| `drc` is large, uniform, one net pair, one overlap value | **grading artifact** | 9.1b — re-grade at the right class. Not a lever at all |
| `broken` is mostly plane-net pads | **the pour could not reach them** | `repair_planes --rip-blocker-nets` (out-of-chain utility; in-chain, route.py's own finalize does this) |
| `check_connected` and `kicad-cli` disagree badly | **the zones are unfilled** | 9.1c — fill, then re-read. Do not "average" them |
| a **symmetry/match clause fails SHORT** (one leg under-length, not over) | **routing lever first** | `--length-match-group` on the pair's own pass meanders the short leg up; only if the group cannot meander (no room) is it placement — then the lever is the **free terminal's position** (the series R/C in the chain), not the ICs |
| a **multi-net rip-return rotates its victim** (each order strands a different net) | **stop rotating orders** | run each candidate order as a FULL chain lineage and compare their `blocking` scores in the ledger — order A's board vs order B's board, not order A's tail vs order B's head. Fifteen ordering experiments in run 5 re-learned this. **Quote each lineage's `min_clearance_used` in the compare**: branch boards inherit their branch-point's `.kicad_pro` floor (run 6 had three floors live at once — 0.1508/0.1532/0.1556), and a floor delta is a confound unless the LOSING side held the looser floor |
| the **same victim set recurs under every order** at every grid | **capacity, not order** | the lane ledger (`check_floorplan --health`) will show the deficit; that is stop condition 3 with the ledger as the measurement, not another ordering lap |
| you are about to write **"this pad cannot be routed"** | **unproven until measured** | `check_reachability.py --pad REF.NUM`. PASSABLE means it is a ROUTER finding and placement is the wrong lever; CAGED means geometry. 9 of 14 such claims across four runs were later refuted — see the impossibility-claim rule |
| one part carries most of a critical net while its BLOCK sits elsewhere | **floorplan, at PART granularity** | `health_net_affinity_offenders` names it and prints the `converge.py poses --ref` line. Block displacement averages this away, so a quiet block metric is not evidence of absence |
| the **bulk pass keeps stranding fine-pitch RAIL pads** under mps | **ordering, before placement** | re-run the bulk with `--ordering original`, rails FIRST (netlist order puts power nets before GPIOs). Order cannot change how many strand — but it chooses WHICH, and a stranded leaf GPIO can still be re-routed before the plane FINALIZE/repair taps land, while a stranded trace-fed rail pad tends to stay lost once they do (rule 5's one-way door; poured rails are already connected from Step 1c and out of this fight). Spend the strandings on the recoverable class. Measured (run 6, signals-first era): mps stranded 5 QFN rail pads; rails-first closed them and moved the fails to leaf nets |

**Accept an iteration only if `blocking` strictly decreased**, or `blocking` is
unchanged and `quality` improved. Otherwise **revert to the parent board** and
take the next lever. An iteration that made it worse is not a starting point.

**One exception, and state it when you use it:** an iteration that reduces
`unrouted` or `broken` while raising a lower-ranked component may be accepted even
if `blocking` is level, because 9.1a ranks connectivity above the rest. Say so in
the ledger with both numbers. A dead net is worse than a wide trace, and the scalar
does not know that.

**When the two boards were graded over DIFFERENT COMPONENTS, the comparison is
VOID — say so, do not pick the smaller number.** `blocking` is a scalar summed
over things that are not the same kind of thing, so the set it was summed over is
part of the number's meaning. `board_score` now prints that set on the identity
line (`[over 9 components; ungraded: floorplan,impedance,length,net_widths]`) —
read it before comparing two runs.

The mechanism already exists:

```bash
python3 -X utf8 py_placer/converge.py record --ledger wk/ledger.jsonl \
    --board <board> --kind completion --score-file wk/score.json \
    --accept-incommensurable "cycle 2 removed 3 vias with NO annular ring \
(unmanufacturable), which no grader counts; blocking 10 -> 13 is broken +1 and \
drc +2 set against a -3 the accept rule cannot see"
```

Run 20 is the worked example. A cycle removed three vias with **zero annular
ring** — holes with no barrel land, which no fab makes — and could not be
accepted: as RECORDED in that run's `score.json` the lap read `blocking`
10 → 13, and the rise was invisible-improvement-plus-visible-cost. Nothing
counted the vias it had removed, because `check_drc` grades via diameter and
drill separately and a 0.3/0.3 via passes both — no grader sees a ring-0 via,
so the removal earns no credit in `undersized` while the `broken` +1 and
`drc` +2 it cost are fully visible.

That is exactly the case this flag is for: the improvement is real, the
graders cannot see it, and no scalar ordering of three components that moved
in three directions is the right answer. State the trade in the ledger text,
with the numbers, so the next reader can re-derive it.

**A THIRD EXCEPTION, for a MANDATORY CHAIN STEP that manufactures `broken` by
construction.** A fanout converts nets that had NO copper into nets whose copper
is in fragments — that is what an escape stub *is* — so it moves work from
`unrouted` into `broken` and `blocking` rises. The step is not a candidate
iteration to be accepted or reverted; it is a step the chain requires, and the
pass that closes those fragments comes later. Measured on one board: the U1
fanout took `blocking` 297 → 371 while `unrouted` fell 144 → 83 (61 nets gained
copper) and `broken` rose 11 → 140; the bulk signal route then took `blocking`
to 222 and `broken` to 65.

Neither of the two exceptions above covers it, and reaching for them is the
mistake: exception 1 is scoped to "`blocking` is **level**" and here it rose 74,
and exception 2's commensurability probes (`ungraded`,
`patterns_matching_no_routed_net`, `nets_analyzed`) are **identical** across the
two rows, because nothing new became measurable — the same nets are graded, they
simply moved between components. By the letter of the accept rule that lap
should have been reverted, which would have deleted the fanout.

So: **name the step as a mandatory chain step, record the component-level
movement (not the scalar), and say which later step closes the fragments.** Do
not dress it up as 9.1a rank — 9.1a is a LEVER-SELECTION rule, not an accept
rule, and citing it here is a category error that reads as compliance.

**A SECOND EXCEPTION, and it is the one you will get backwards: `blocking` can
RISE because the iteration made more of the board MEASURABLE.** 9.1's rule that
"a run reporting 12 has not found a better board, it has looked at less of it"
does not stop applying once you are inside the loop — it governs comparing two
*iterations* exactly as it governs comparing two flag sets. A net with **no
copper** is invisible to `net_widths` (its clause lands in
`patterns_matching_no_routed_net`), invisible to `check_impedance` (it has no
segments to walk, so it is not in `nets_analyzed`), and invisible to any
per-segment geometry check. Route it and every one of those violations *appears*,
having been there all along.

Measured across one pair of iterations:

| | i1 | i2 |
|---|---|---|
| `blocking` | **34** | 41 |
| width clauses with no copper to measure | 2 nets | **0** |
| `impedance.nets_analyzed` | 7 | **9** |
| HARD clauses failing (repo `check_spec`) | 8 | **7** |

i2 shipped. **Before comparing two `blocking` values, compare what each one
looked at** — `patterns_matching_no_routed_net`, `nets_analyzed`, and `ungraded`.
If they differ, the scores are not commensurable and the scalar is the wrong
arbiter; fall back to the components that ran in both, and to the repo's own spec
checker if it has one. Assert `nets_analyzed` equals the number of nets you named
in `--impedance-nets` every iteration, exactly as you assert `ran == true`.

**Watch for whack-a-mole.** Ripping to route net A can leave net B unrouted, and
the tally still reads "1 failed" — a *different* net. Compare the failing net
**names** between iterations, never just the counts. If they alternate, route them
one per call with the other explicitly out of the rip set (9.3c rule 3). If the
victim keeps rotating across MORE than two nets as you permute the order, stop
permuting: score each order as its own full-chain lineage and let the ledger
pick (9.3d's rip-return row) — and if the same victim SET survives every order,
it is capacity (the lane ledger has the number), not ordering.

**And CAP the single-net laps (run-23).** After **2 laps with no
oracle-verified blocking delta on the WRITTEN board**, single-net laps are
forbidden: escalate to the joint co-route (9.3c rule 3's exception) with
forensics-named blockers, or to analysis — never a third single-net lap. Two
clauses that make the cap honest, both measured: (a) the delta is the
ORACLE'S, on the written file — run 23's R15 closed ZERO of its 4 targets
while the in-run buckets said only one failed, because terminal restores
reverted after the tally printed; (b) a lap that produced **zero board
delta does not count as an attempt but DOES count against the cap** — run 23
recorded six measured no-op laps (~330s of router time) that its loop
counted as tries.

**The tooling-vs-placement discriminator (#118), for a converged board with
few failures left:** ask whether a competent human could hand-route the
remaining nets on THIS placement. If yes, the gap is tooling — file it as a
systemic finding and take rung 8, which exists for exactly this. If no human
could either, it is placement or capacity — the lane ledger has the number,
and the finding goes to the next run's Step 0, not to more router laps.
Run 7's west fan answered "no human could" (~25 exact-clearance candidates,
every one within 0.1 mm of committed constrained copper), which is what made
it a capacity finding rather than an engine complaint.

**After ANY placement change every downstream routed board is stale** — re-run
the chain from the placed board. Never keep a routed artifact from before it.

#### 9.4 — Write the ledger, every iteration, before the next one starts

`wk/ledger.jsonl` in the work dir. It is what makes the run resumable, lets the
final report name which stop condition fired, and gives the film its frames.

**Write it with `converge.py record`, and read `converge.py status` back every
iteration.** The verbs that make a ledger worth keeping — `step-back` (byte-exact,
because the board is stored by content hash), `replay` (re-runs the recorded argv),
`status` (the systemic/completion split) and `make_film.py --from-ledger` — all
read append-only **JSONL** through `board_store.Ledger`. A hand-written single JSON
document is readable by a person and by nothing else, so every one of them is
unreachable from it.

```bash
python3 -X utf8 py_placer/converge.py record --ledger wk/ledger.jsonl \
    --board wk/iter03.kicad_pcb --kind completion \
    --lever 'rip lever: --rip-existing-nets QSPI_SD2 + --grid-step 0.025' \
    --score-file wk/score_iter03.json \
    --argv python3 -X utf8 py_router/route.py wk/iter02.kicad_pcb wk/iter03.kicad_pcb --nets QSPI_SD1 ...

python3 -X utf8 py_placer/converge.py status --ledger wk/ledger.jsonl      # EVERY iteration
```

`status` is the alarm for 9.2's failure mode: it splits the budget into completion
vs systemic and warns when at least half went to the instrument. Nothing else in
the loop says that out loud, and the run that needed to hear it did not.

**Name the panels you READ in the `--lever` text** (`... [read: focus3/panel1,
drc3/cluster1]`). The image mandates are auditable only through the ledger: run
5's breach — a produced-but-never-opened delta render — was invisible precisely
because nothing recorded reads. An iteration whose score had `unrouted`/`broken`
> 0 or a failed `check_drc`, with no `[read: ...]` in its entry, skipped
read-case 3 or 7. **Record NON-triggers the same way**: when a mandate's
trigger is checked and absent, say so in the entry (`[checked: 0 B.Cu parts ->
no --per-side]`) — an unrecorded non-trigger is indistinguishable from a
skipped mandate to any later audit (run 6's watcher had to grep the board to
tell them apart). **A pose decision is read-case 5 even when the arithmetic
is decisive**: a rot-0-vs-rot-180 call made on `components.inversions` alone,
with no side-by-side ratsnest reads in the ledger, is a mandate skipped —
run 7 decided the U3 pose twice that way; the number was right, and the
breach is still a breach the audit had to flag.

**What `record` actually writes is this, and only this:**

```jsonc
{"iteration": 3, "kind": "completion",       // or "systemic" -- see 9.2
 "parent_sha": "...", "result_sha": "...",   // content hashes, not paths
 "lever": "rip lever: --rip-existing-nets QSPI_SD2 QSPI_SS + --grid-step 0.025",
 "lever_argv": ["python3", "-X", "utf8", "route.py", "..."],
 "score": {"blocking": 12, "blocking_by": {"unrouted": 1, "drc": 11}},
 "accepted": true}
```

**There is no `--unrouted-nets`, no `--parent`, no `--verdicts` flag** — those
fields do not exist, and a reader who assumes they are captured will keep no
record of the one thing 9.3d says decides an iteration. Until they do exist,
**put the failing net NAMES in `--lever`**, which is free text and is what
`status` and the film both display:

```bash
--lever 'rip lever: --rip-existing-nets QSPI_SD2 + --grid-step 0.025.
         unrouted BY NAME: QSPI_SD0, QSPI_SD1. Fixed USB_DP_R/USB_DM_R;
         newly broke QSPI_SS, GPIO2 -- whack-a-mole, net a traded for net b.'
```

**Record the failing nets by NAME, not by count.** Counts hide whack-a-mole
completely: measured on one run, `unrouted` read **4 → 4** across an iteration
that had in fact fixed four nets and broken five different ones. The scalar said
"no progress"; the names said the iteration was churning. **This applies to
PROBE records too, and to the BASELINE side of every comparison**: run 7's
adoption entry recorded `full_probe_failures: 41` — a count — and the 41
names the ADOPTED board failed appeared nowhere, so the routing phase's first
whack-a-mole comparison had nothing to diff against. A probe verdict enters
the ledger as names (`score.failed_nets`, or the names in the lever text),
for the candidate AND the baseline row it beat. `record` now nags exactly
this: a score carrying `failures` without `failed_nets` draws a NOTE.

**`parent_sha` is the board this iteration actually came from** — `record`
derives it from the last accepted entry, not from iteration N−1. When you need a
path for `render_placement --before`, resolve it out of the store by that sha
rather than guessing; using N−1 renders a delta that never existed. **Never
reuse an output path across iterations**: a ledger that says
`wk/placed.kicad_pcb` when three iterations wrote that name is unauditable, and
one that named a *rejected* board as the parent of everything downstream got
shipped.

**Take the argv from the tool's own `CMD:` line, not from memory.** Every
routing tool now self-echoes `CMD: <the exact invocation>` as its first stdout
line and `EXIT=<rc>` as its last (`route.py`, `route_diff.py`,
`route_planes.py`, `route_disconnected_planes.py`, and the checkers). That line
comes from `sys.orig_argv`, so it carries interpreter flags like `-X utf8`
verbatim and is REPLAYABLE truth rather than a reconstruction. Until run 12 the
three signal/diff/plane routers did not have it, which made "paste the tool's
own `CMD:` line into `converge record --argv`" unsatisfiable for a routing lap
— run 11 hand-wrote replay scripts instead, which is exactly the placeholder-
argv failure the next paragraph exists to stop.

**The argv must be real, and the close-out must name its stop condition —
`record` now enforces both.** An `--argv` whose first token is neither an
existing file nor on PATH is refused (exit 2, nothing written): run 7's
endgame recorded `["python3","-X","utf8","dummy"]`, which `replay` can never
run — a placeholder argv turns the ledger back into prose. The run-closing
entry takes `--final --stop-condition '<which of 9.5 fired>'`; `--final`
without a stop condition is refused the same way. And **before quoting a
headline in the lever text, diff it against the SAME entry's score payload**:
run 7's final entry said "SWD closed, 5 opens" while its own score listed
SWDIO among 6 unrouted — the prose shipped into the report and the correction
cost a commit. The score is the record; the lever text is a caption of it.

**Log the systemic/completion split in the final report**: *"41 iterations: 9
systemic, 32 completion"* is a fact about how the budget was spent, and a run that
cannot state it was not keeping a ledger.

#### 9.4b — Boundary verification: BLOCKING, at every accepted iteration and at close

The ledger records what the operator SAYS happened; nothing above checks it
until the close-out audit, and by then the errors have compounded. Run 4's
audit found exactly this: 6 of 8 entries batch-written after the fact (the
per-step timestamps said so), and one `[read:]` tag claiming a pixel read
that never happened. Both were honest-looking entries a contemporaneous
check would have bounced in seconds.

**The rule: after every ACCEPTED iteration's ledger entry, and at close-out,
an independent subagent verifies the entry against its artifacts BEFORE the
next step may start. A FAIL blocks; remediate (fix the entry, re-read the
artifact, or re-run the step) and re-verify.**

What the boundary verifier receives — and it must be ONLY this, never the
raw board (it verifies the RECORD, not the routing):

- the ledger entry (the JSONL line just written),
- the score JSON it attaches (`--score` payload),
- the render JSON(s) the entry's `[read: ...]` tags name,
- the operator's one-paragraph claim of what the iteration did.

What it checks, each with the artifact that decides:

1. **board_sha binding** — the score payload's `board_sha` matches the
   entry's `result_sha`; a stale attachment (run 4 had two, deliberate but
   warned) must be lever-explained in the entry itself.
2. **`[read:]` truthfulness** — every panel the entry claims was read exists
   in the named render JSON, and every checklist value the claim quotes
   matches that JSON. An entry whose claim quotes numbers appearing in no
   attached artifact FAILS (run 4's e4: the tag implied a read that lived
   only in the journal).
3. **Contemporaneity** — the entry's timestamp is AFTER its artifacts'
   mtimes and BEFORE any later step's artifacts. Batch-written history
   shows up as a cluster of entries stamped within seconds; that is a FAIL
   on every entry in the cluster except the last (run-2's defect, recurred
   run 4; "record between steps or disclose in the entry, no third option").
4. **Claims-vs-artifacts** — every number in the operator's claim traces to
   a field in the entry, the score, or a named render JSON. "Fixed 4 nets"
   with no names in the lever text is a FAIL (the whack-a-mole rule above).
5. **Assembly-clean** (run 6; placement-phase and fix-loop boundaries) —
   the verifier additionally receives the fresh `check_assembly` JSON and
   the render JSON, and FAILS unless `blocking == 0`, the checklist's
   `b_body_overlap_pairs` is empty, and every NEW-vs-baseline advisory
   pair is either fixed or dispositioned in the entry. A claim of
   "placement done" with no attached `check_assembly` JSON is itself a
   FAIL — the run-5 lesson is that the phase ended on belief.

Reply format is the watcher's: one line,
`VERDICT=PASS` or `VERDICT=FAIL:check=<1-5>;finding=<one line>;evidence=<path#pointer>`.

Cadence discipline: REJECTED iterations do not get a boundary verification
(their entries record a road not taken; the close-out audit samples them),
and the verifier is bounded to the slices above — handing it the whole work
dir invites it to re-litigate routing decisions, which is the convergence
loop's job, not the record-keeper's. The close-out boundary verification
additionally walks the WHOLE ledger for checks 3 and 4 (monotone t-stamps
end to end; the final entry's stop condition quoted against its score).

#### 9.5 — Stop conditions. Only these four. Say which one fired, every time.

1. **`blocking == 0`, the repo's own spec checker passes, and every verifier lens
   passes** → done. All three are required. `board_score` exits **0** at
   `blocking == 0` even on a board with ten HARD clauses violated, because the
   clauses a repo checker measures are not `board_score` components — so
   exit-code-driven automation stops there unless you gate on the checker too.
   See "Verify with independent subagents".
2. **Budget exhausted** — you have actually written **100** ledger entries for this
   board. Report the best-scoring board **and the remaining blockers itemised with
   measurements**. Do not present it as finished.
3. **Five consecutive iterations with `unrouted` and `broken` both unchanged,
   after trying the rip lever, a finer grid, and a layer change on the failing
   nets** → floorplan-limited or spec-limited. Say which, with the number. (Five,
   and on the connectivity components — three iterations of `drc` not moving means
   nothing when the real blocker is a dead net.)
4. **A blocker is geometrically unsatisfiable** → stop and report it as a
   **finding about the requirement**, with the measurements that prove it. Worked
   example: a 2.4 mm clearance requirement written as a netclass also applies
   pad-to-pad, and on that connector the measured pad gaps were 0.500 mm (VBUS)
   and 1.300 mm (GND) — 23/44 nets with it, 38/44 without. That is unsatisfiable
   *as written*, and it took one measurement to prove. **Do not silently relax
   it, and do not grind iterations against it.**

   **When the unsatisfiable clause is a dru rule the router hard-enforces**
   (#549 track channel or a layer rule), stop-4 scopes to the REGION, not the
   run: (1) route the pass WITH the rule first and measure the failure —
   decide on the measurement, not in advance; (2) then stage a sibling dru
   for that pass with ONLY the unsatisfiable rule lifted — never a bare
   no-dru copy, which silently drops every OTHER rule's protection for that
   pass (run 6's watcher caught exactly this before it shipped); (3) grade
   the residue against the registered floor and report the clause as a
   requirement finding. The rest of the chain keeps the full dru.

##### These are NOT stop conditions

Stopping for any of these is a process failure, not an outcome. If one of them is
pulling at you, write the next ledger entry instead:

- **"This is taking a long time."** Wall-clock is not a stop condition. A run once
  stopped at 11 of 20, labelled it "budget exhausted", and recorded in its own
  ledger that the levers were *not* exhausted. Eleven is not twenty.
- **"The score stopped moving."** Check *which* component. `blocking` level while
  `unrouted` falls is progress (9.3d's exception).
- **"The remaining work is hard."** Hard is what the budget is for. A net that
  needs a scoped rip, a finer grid and a layer change is three cheap iterations,
  not a wall.
- **"I have written up the findings."** The report is not the deliverable while
  nets are unrouted. Finish the board, then write.
- **"The last lever failed."** Revert and take the next one. The ladder has more
  rungs than you have tried: rip set → **joint co-route (forensics-named
  blockers, 9.3c rule 3's exception)** → grid → layer → via cost → width →
  order → placement → hand-authored micro-copper. The joint co-route sits
  SECOND deliberately (run-23): it was tried last, after 12 single-net laps
  and 18 flat laps of blocking, and it was the lever that finally negotiated
  the pocket.

**Rung 8, hand-authored micro-copper, exists — with FIVE hard conditions.**

1. **Only after every mechanical rung is exhausted** (dru lifts, nc-map fixes,
   scoped grids, rip sets — run 7 entered rung 8 only after all four were
   demonstrably spent, and that part it got right).
2. **Round-cap arithmetic off the ACTUAL copper widths** (a track's reach is
   `endpoint + width/2`; run 5 authored 0.427 and 0.442 mm candidates that
   both failed the arithmetic before 0.465 mm passed).
3. **A join verifier BEFORE the first segment, graded against ALL copper.**
   Reach is the easy half; the shorts live in the other half — third-party
   segments, vias, pads, pour. Run every candidate polyline+via through
   `check_join.py BOARD NET x,y,layer ... via:x,y` — it stages the candidate
   onto a copy of the board and diffs the REAL check_drc engine (netclasses,
   `.kicad_dru` layers, rotated pads, board edge, hole-to-hole), plus the
   join-specific checks DRC omits: a layer change with no `via:` and same-net
   via stacks. (No check_join in the pinned engine? A scratch-board
   `check_drc.py` run per candidate is the zero-build version.) Measured,
   run 7: pad-edge-only arithmetic at 0.1575+ margins authored **42 shorts**;
   the verifier was built mid-recovery instead of first.
4. **Re-gate EVERY edit** — drc + connectivity + score on the edited board,
   recorded in the ledger like any other lever, revert on regression. Per
   EDIT, not per phase: run 7 gated a whole hand-copper session on
   connectivity alone with DRC deferred, and the deferred DRC was where the
   42 shorts surfaced. A hand segment that skips the gate is not a fix, it is
   an unmeasured edit.
5. **Lock it, and commit it FIRST.** Stamp every hand-authored segment and via
   `(locked yes)` at authoring time — locked copper's net is never
   rip-eligible (#521), which is exactly the semantics a hand join needs: no
   later `--rip-existing-nets` glob, in-run ladder, or plane-repair
   `--rip-blocker-nets` picker may treat the one corridor you proved as free
   space (measured, run 7: the plane repair picked three hand-routed GPIOs as
   tap blockers and ripped them; unlock deliberately if a rebuild must move
   them). And hand joins are the most constrained copper on the board — one
   proven corridor, zero router flexibility — so constrained-first applies
   across the hand/router boundary: commit them BEFORE the flexible nets'
   final routing and make the router route around them (strip and re-route
   the flexible set last if needed). Never author a hand join into a fabric
   of committed copper the router could have placed elsewhere (measured,
   run 7: fixed-joins-first closed at 0 shorts where the reverse order had
   authored 42).

**Rung 8 has TWO exits: placed-and-gated copper, or an exhaustive NO-JOIN —
and the second is a result, not a failure, but ONLY when it ships all four
parts:** (1) the sweep envelope (width, clearance, path families, step)
recorded in the ledger like any lever; (2) ONE scoped route at the router's
own hinted finest grid (0.025 at ≤0.4 mm pitch) to cover paths outside the
enumerated families; (3) a `--view` crop of the region, read and ledgered;
(4) the verification-mode disclosure — a fanned-out hostile rebuild, or the
single-agent statement that the sweep itself is the rebuild. A NO-JOIN
missing any part is a HYPOTHESIS, and every report that quotes it must say
so (run 7's west-fan capacity claim shipped with the envelope, the
finest-grid route and the crop all missing — it is a hypothesis pending the
next run, by this rule). A complete sweep IS the hostile rebuild the watcher
pattern demands; do not follow it with more router laps at the grid that
already failed (run 6 ran two rotation laps after its sweep proved the
fabric sealed).

**A worked case of stopping wrongly, because it is the most expensive mistake in
this document.** One run reported four unrouted nets, wrote up "stop condition 3"
and named the next lever *in the same write-up without trying it*. Budget spent:
**4 of 100**. Resuming cost four scoped single-net calls of a few seconds each
and took `unrouted` **4 → 0**. Every blocker it had reported dissolved:

| reported as | actually was |
|---|---|
| "boxed in by its QFN neighbours" | needed the rip set the **router** named, which was wider than the one `net_forensics`' 1 mm radius showed |
| "walled by VREG_AVDD/VCC3V3" | the analyst's own `--track-width 0.4`, on a net with no width clause |
| "needs a fanout; that is what run 5 should try" | routed on one layer at `--grid-step 0.025`, no fanout |

Two rules fall out. **The working grid at a 0.4 mm-pitch part is 0.025, not
0.05** — "0.05, or 0.025 for sub-0.4 mm pitch" reads as excluding a part that is
*at* 0.4 mm, and it should not. And **the router's hint beats the forensics
wall**: forensics reports what is inside a radius, the router reports what its
whole obstacle map says is decisive. When they disagree, take the router's set.

**Before invoking condition 2 or 3, answer in writing:** how many nets are
unrouted, what is the router's own hint for each, and which of the 9.3c rip rules
has not been tried on them? If any of those is unanswered, the loop is not done.
**The answers are already printed:** each failing net's `Hint:` line is an
untried rung until you run it or refute it — quote it; and any
`protected_skipped: {net: user}` line means YOUR OWN protection is the named
blocker — exercise the exact-name override or write why not (run 6's final
watcher found both classes sitting unread in the logs while the stop was
claimed). And an endgame burst is still iterations: **one ledger entry per
board-state, journal per phase** — a stop-3 claim rising out of a one-entry
endgame cannot exhibit its five unchanged iterations.

Ending on 2, 3 or 4 is a legitimate outcome. Ending on any of them **while
calling the board finished is not**, and ending on none of them is not an ending.

### Diagnose and Retry

**Soft-cost retry levers (measured on 12-board challenging-chain A/B; these
are RETRY settings — the defaults stay mild on purpose):**

- **First-choice retry on any struggling board:** re-run the failing signal
  step (or chain) with `--ripped-route-avoidance-cost 3
  --track-proximity-cost 2` and **KEEP WHICHEVER RESULT GRADES BETTER** —
  routing is deterministic and the comparison is cheap. Across 12 hard
  boards this improved 8 (top gains +6.1 and +4.1 pts, connectivity down on
  nearly every win), regressed 2, and timed out 2 (expect up to 2× runtime).
  Board-type prediction is IMPERFECT — a 6-layer RAM board regressed −5.0 —
  so never blind-apply: always retry-and-compare.
- **Thrash-class variant:** on boards whose logs show heavy rip-up churn,
  ALSO try `--via-proximity-cost 100` on top (rescued one thrash board
  +2.4 pts where the base combo timed out) — but it fails more often than
  it helps elsewhere (3 wins / 5 losses / 3 timeouts); strictly a
  second-attempt lever, same keep-better rule.
- **Do not stack these with `--bga-proximity-cost` or a lower `--max-ripup`**
  — both combinations measured WORSE than either alone (they remove exactly
  the freedom the corridor pricing needs).
- **Boards routing fine at defaults:** leave everything alone.
- `--via-proximity-cost 0` now simply means "no extra via cost from
  proximity" (Rust 0.20.1 removed the old 0 = hard-via-ban mode, which was a
  measured ~200x CPU explosion) — safe, but rarely useful: the default 10 is
  what keeps vias out of escape fields. Leave
  `--ripped-route-avoidance-radius` at its default (widening it measured
  worse).

After running routing commands:
1. Report how many nets were routed successfully
2. **If routes failed**, invoke `/diagnose-routing-failures <board> <log files>` — it parses
   the JSON summary, failed-net histories, and blocking reports, correlates failures
   spatially, and outputs a targeted retry command. Apply its recommendation. If that skill
   is unavailable, fall back to this table:

| Failure Pattern | Likely Cause | Solution |
|-----------------|--------------|----------|
| "no rippable blockers found" | Route blocked by non-rippable obstacle | Use `--no-bga-zone`. If pads are "boxed in by static obstacles", read the hint's `(current: ...)` tail and the summary's `boxed_in[].at_floor`: **geometry has travel** → shrink it and pair a finer grid with it; **already at the declared floor** → the parameter family is spent, this is placement (see 9.3d). A finer grid alone resolves the same obstacles more precisely; it does not make a gap wider |
| "Re-route FAILED: no path found" | Ripped net couldn't find new path | Capacity problem (`--max-iterations` self-extends, #529): `--max-ripup`, clearance, or layers |
| Many multipoint pads failed on same component | Congested area | Shrink geometry toward the fab floor (see below); keep `--max-ripup` at ~5 (deeper measured worse) |
| Many failures cluster in one channel/region | Tracks too fat for the channel | **Congestion escalation**: re-route the failed nets at smaller track/via/clearance down to the fab floor (see below) |
| 2-layer board: low completion, via count far above a hand layout, or copper badly skewed to F.Cu while B.Cu sits empty | Default B.Cu cost (3.0×) over-penalizes the back layer | Retry with balanced `--layer-costs 1.0 1.5` (down toward `1.0 1.0`) — see "Dense 2-layer boards: rebalance layer costs" below |
| Routes near BGA boundary failing | BGA exclusion zone too aggressive | Use `--no-bga-zone` |

```bash
python3 -X utf8 py_router/route.py board_prev.kicad_pcb board_routed.kicad_pcb \
    --nets "*" \
    --no-bga-zone \
    --max-ripup 5 \
    2>&1 | tee /tmp/route_retry.txt
```

   Key parameters for difficult boards (especially 2-layer with BGA/PGA):
   - `--no-bga-zone` - **Critical**: Allows router to enter BGA area for alternative paths
   - `--max-ripup 5` (default 3) - More rip-up attempts to resolve conflicts (measured optimum 3-5; deeper loses, see note 15)
   - Do NOT pass `--max-iterations` — self-budgeting (#529) extends hard searches to a 1e7 ceiling automatically; a post-extension failure is a capacity problem, not a budget one
   - `--stub-proximity-radius 10 --stub-proximity-cost 3.0` - Spread out fanout stubs (optional, for aesthetics)

#### Dense 2-layer boards: rebalance layer costs (issue #178)

On 2-layer boards the router defaults to per-layer costs **F.Cu=1.0, B.Cu=3.0**
(best practice #8) to keep most signal copper on top. But with a GND/power plane
already filling B.Cu, that 3× back-side penalty can over-bias routing onto F.Cu:
the top channel fills up while B.Cu sits nearly empty, the router takes long F.Cu
detours that then need a via to reach a B.Cu pad, and on congested boards the
exhausted F.Cu channel strands pads that B.Cu could have carried. This is the
dominant route-quality gap on tight 2-layer keyboard/peripheral boards.

**When to suspect it** (check the route `JSON_SUMMARY` / `comparison` block, or
measure per-layer copper length and via count against a reference):
- Strong F.Cu skew — e.g. >80% of signal copper on F.Cu while B.Cu is sparse.
- Via count far above a hand layout (the F.Cu-detour-then-via pattern).
- Low completion with failed pads clustered where F.Cu is full but B.Cu is free.

**Retry with more balanced layer costs** so the router crosses to B.Cu for short
diagonal runs instead of detouring on F.Cu (order matches `--layers`: F.Cu first,
B.Cu second):
```bash
python3 -X utf8 py_router/route.py board_fanout.kicad_pcb board_signal.kicad_pcb \
    --nets "*" \
    --track-width 0.127 --clearance 0.1 \
    --layer-costs 1.0 1.5 \
    --no-bga-zone --max-ripup 5 \
    2>&1 | tee /tmp/route_balanced.txt
```
(Mirror your Step 2 net selection and `--power-nets`/widths exactly in these
retries — the plane nets ride the route step in the pours-first chain, #562.)
Start around **`1.0 1.5`** (down from the `1.0 3.0` default); if F.Cu is still
saturated, step to **`1.0 1.2`** or fully balanced **`1.0 1.0`** (fine when a
plane fills B.Cu — signals carve the pour and it reflows around them). This is
**complementary to**, not a replacement for, routing at the fab floor (below): a
balanced layer that's still too fat won't fit the channel either, so keep
`--track-width` thin. Re-route the **whole** signal step, not just the failures (a
victim is blocked by the successful F.Cu tracks already in its channel). Then
compare completion, via count, and F.Cu:B.Cu balance, and keep whichever connects
more pads with fewer vias.

Measured at `--track-width 0.127` (B/F = B.Cu:F.Cu copper-length ratio; both
boards stay 100% connected at every setting — the win is via count and balance):

| board | default `1.0 3.0` | `1.0 1.5` |
|-------|-------------------|-----------|
| 2-layer corpus board A | B/F 0.17, 177 vias | **B/F 1.01, 98 vias** |
| 2-layer corpus board B | B/F 0.19, 102 vias | **B/F 1.85, 59 vias** |

`1.0 1.5` roughly **halves the via count** and pulls the layer balance from a
~6:1 F.Cu skew to near parity (board A's human layout sits around B/F 0.89).
`1.0 1.0` lands in the same neighbourhood — pick the one with fewer vias.

#### Route signals at the FAB floor by default (thin is faster AND more complete)

**`track_width` and `via_diameter` are NOT DRC floors** (Step 4), and — this is
the subtlety — **the fab floor is NOT the board's `min_track_width` constraint
either.** Three different numbers get confused here; keep them straight:

- **Board `min_track_width`** (from `.kicad_pro`, often 0.2 mm) — the
  author's self-imposed DRC rule. Often conservative. Note `list_nets
  --design-rules` reports its "manufacturing floor" track as `max(this, JLC min)`,
  so it currently **clamps the track floor to this constraint** (0.2) and does NOT
  surface the finer fab capability — do not treat that printed track number as the
  real floor (it's right for clearance/via, just not for track).
- **Fab physical track minimum** (JLC ≈ **0.0889 mm / 3.5 mil** standard; **0.127
  mm / 5 mil** is the safe no-extra-cost width) — the actual floor. **This is the
  target.** It can be *below* the board's `min_track_width`: human corpus
  boards routinely route most signals at 0.127 mm, under their own 0.2 mm constraint,
  which is exactly why they fit channels our 0.2 mm net-class tracks can't.

For ordinary signals there is **no benefit to routing fat** and a real cost.
Measured on a 4-layer corpus board (signal pass, same clearance/grid, width only):

| Signal track width | Multipoint nets routed | Pads connected | Time |
|--------------------|------------------------|----------------|------|
| **0.127 (5 mil)**  | **122**                | **360/376**    | **2.69 s** |
| 0.15               | 118                    | 354/376        | 2.93 s |
| 0.20 (net-class)   | 103                    | 323/376        | 6.52 s |

Thinner is **monotonically better on both axes** — more nets complete *and* it
finishes faster (fat tracks cause ripup churn). So don't route fat and escalate;
**route the signal step at the fab floor from the start, and if still congested
go DOWN toward the fab physical minimum** (0.2 → 0.127 → 0.0889), not toward the
board's conservative `min_track_width`. There is no "knee" above the fab floor to
hunt for.

1. **Take the fab floor**, not the board constraint: the fab's physical track
   minimum (JLC 0.0889 mm / 3.5 mil; use 0.127 mm / 5 mil for a zero-cost,
   high-yield default). Going below the board's `min_track_width` is intended here
   — it's what the human did. (Keep DRC honest separately: grade at the clearance
   floor from `--design-rules`; a thinner track only *increases* clearance to
   neighbours, so it never creates a clearance violation.)
2. **Route the whole signal step at that width** (re-route everything, not just the
   failed nets — a victim is blocked by the *successful* wide tracks already in its
   channel, so thinning only the failures leaves the channel full):
   ```bash
   python3 -X utf8 py_router/route.py board_fanout.kicad_pcb board_signal.kicad_pcb \
       --nets "*" \
       --track-width <fab floor, e.g. 0.127 or 0.0889> --clearance <floor, e.g. 0.1> \
       --via-size <floor via, e.g. 0.30> --via-drill <floor drill, e.g. 0.15> \
       --no-bga-zone --max-ripup 5 \
       2>&1 | tee /tmp/route_signal.txt
   ```
   A finer `--grid-step` (0.05, or 0.025 AT ≤0.4 mm pitch — a part *at* 0.4 mm
   needs 0.025: "sub-0.4" reads as excluding it, and measurement says otherwise)
   is the complementary
   lever — a corridor that exists geometrically still needs a grid line on it to be
   found; pair it with the thin width at fine-pitch escapes ("boxed in by static
   obstacles"). If still congested, step the width down further toward the fab
   physical minimum and re-route.
3. **Keep only the nets that NEED width wide — by rule, not by sweep.**
   Power/high-current nets stay wide via `--power-nets`/`--power-nets-widths`, and
   impedance-controlled nets keep their calculated width (`--impedance`, or
   `route_diff.py` for pairs). Everything else routes at the fab floor. You do
   **not** need to find which signals are "genuinely congested": there's no reason
   to widen an ordinary signal at all, so the question never arises (and a net that
   passes wide can itself be the blocker of another, so a per-net width guess is
   unsound regardless).

3. **If swaps occurred** (polarity or target swaps):
   - Tell the user how many swaps were made
   - Ask if they want to sync the schematic
   - If yes, ask for the KiCad project directory path
   - Re-run the routing command with `--schematic-dir` added
4. Run verification: invoke `/review-routed-board` (falls back to the raw DRC and connectivity checks)
4b. **Apply the coverage gate (Step 6):** if `check_connected.py` lists any
   fully-unrouted multi-pad net, the board is NOT done — handle each (route or
   pour it) and re-verify before summarizing. Do not present an unrouted net as
   an accepted shortfall.
5. Summarize the final state of the board
6. **Offer to clean up intermediate files**:
   - List the intermediate `.kicad_pcb` files created (e.g., `board_step1.kicad_pcb`, `board_step2.kicad_pcb`, etc.)
   - Ask if the user wants to delete them, keeping only the final output
   - If yes, delete the intermediate files

Example cleanup prompt:
> "Routing complete. The following intermediate files were created:
> - board_step1.kicad_pcb (after fanout)
> - board_step1c.kicad_pcb (after GND/VCC pours)
> - board_step2.kicad_pcb (after the all-nets route + in-run plane finalize)
> - board_step4.kicad_pcb (after GND return vias, if run)
>
> The final routed board is: board_step2.kicad_pcb (or board_step4 if GND vias ran)
>
> Would you like me to delete the intermediate files?"

## Step 9: Emit the plan as an executable artifact (always)

The plan is not finished as prose — it ships as TWO machine-loadable files
next to the board, so the exact tuned choices replay with no LLM:

1. **`<board>_plan.sh`** — every board-mutating command from the plan, one
   per line, fully quoted, in execution order, env-knob prefixes included
   (the redo_commands.sh format; `tests/stress/redo_stress_test.py` replays
   it verbatim). This is the authoritative form: it carries EVERYTHING,
   including the Step 2c environment block and iteration passes.
   **Begin the file with an explicit `cd <repo>` line** (not just a
   `# cwd=` comment) so a bare `bash <board>_plan.sh` works — relative
   `py_router/` tool paths fail from any other directory.
   **Verification commands go in as COMMENTS, never executable lines** —
   the file's exit code is read by executors as the chain verdict, and a
   trailing read-only check that exits non-zero (violations found, or a
   wrong relative path) makes a chain whose `final.kicad_pcb` is perfectly
   good report as failed (measured: half the 15-board wave reported
   rc≠0 from exactly this). The last executable line must be the final
   `route.py`.
2. **`<board>_plan.json`** — the GUI AI-tab form:

   ```bash
   python3 tests/stress/manifest_to_plan.py <board>_plan.sh -o <board>_plan.json
   ```

   The AI tab's Load button accepts this file and the plan executor runs
   the same chain through the GUI engine path. KNOWN LIMITATION: env-only
   knobs (the Step 2c plan/attraction stack) do not survive into the JSON —
   only `--flag` params map to plan params. Until those knobs are promoted
   to first-class engine params + dialog controls (the #513 discipline),
   the `.sh` form is the one that reproduces the tuned result exactly; say
   so in the summary when the two forms differ.

A per-board plan file that ever gets improved (a better recipe found on a
later pass) should be REGENERATED through this skill, not hand-patched —
the skill is the tuner; the plan file is its output.

## Step 10: Plan-authoring precedence rules (resolve these conflicts explicitly)

Lessons from a dry-run audit (an agent following this skill end-to-end):

1. **No stackup ⇒ no impedance passes, period.** When the board has KiCad's
   default stackup, SKIP every `--impedance` step (including the DDR SSTL
   40Ω pass) and lead the plan with the /recommend-stackup warning. The
   no-stackup rule OUTRANKS every interface-specific impedance
   recommendation.
2. **Populated-array escape:** `dogbone` supersedes the older
   "channel-infeasible → underpad" advice for populated BGAs; underpad is
   for WLCSP/inner-row cases where no inter-pad gap exists at all.
3. **Fanout `--layers` must EXCLUDE any layer carrying a solid plane**
   (e.g. the In1 GND plane) — escapes on the solid plane shred it, and
   the fanout does not avoid poured layers on its own.
4. **The route flag is `--no-bga-zones` (plural).** The singular spelling
   fails argparse.
5. **No pipes in the emitted plan.sh.** `2>&1 | tee ...` is for
   interactive runs only — `manifest_to_plan.py` tokenizes pipe segments
   into net globs and corrupts the JSON. Plain redirects or nothing.
6. **`--max-ripup`: 5 on dense boards** (any fine-pitch BGA present or
   >150 nets), else leave the default 3.
7. **A "pair" whose far end is bare test points** may stay in the diff
   step (the peel machinery degrades gracefully) — note it in the plan
   rather than agonizing.
8. **The Step 5a-tuned density gate outranks the pour-philosophy survey.**
   Outer-layer floods, per-zone fragility `=0`, and rail co-pours are
   DENSE-tier moves only; a STANDARD board gets inner-only pours and rails
   as wide traces, no matter how many pads a rail has (measured: the four
   flood-regressed wave boards all went to 0 opens and 3–5× faster on the
   inner-only map).
9. **Never split more than 2–3 rails onto one Voronoi layer** — prefer
   wide traces for the overflow (measured: six rails on In2 → +3V3 in 8
   islands, 7 opens).
10. **Stitch/GND-via tail only at high speed tier or above** (from
   `/find-high-speed-nets`) — and when it runs, the chain still ends on
   `route.py`. Low-speed boards end at the Step 2 route (plus Step 2d
   iterations) with no re-pour tail.
