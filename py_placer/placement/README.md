# Placement

Perturbative placement optimization for KiCad PCB files. Starts from an
existing (hand- or AI-made) placement and improves it for routability —
it does not place boards from scratch *unaided* (`place_seed.py` below is
the aided path: a declared floorplan intent carries the constraints a
from-scratch run lacks). Background research and experiment results:
[docs/placement-optimization.md](../../docs/placement-optimization.md).

Four command-line tools sit on top of this module:

## place_optimize.py — greedy quench

Small nudges (capped by `--max-displacement`), 90° rotations (on each part's
own angular lattice, so a part placed at 45° rotates to 135/225/315 rather
than snapping to the axes), and same-footprint swaps (capped by
`--swap-max-displacement`, default: the same cap) that reduce airwire length
+ crossings + a whitespace (halo) penalty scaled by pin count + a soft
board-edge margin. Locked footprints never move.

```bash
# Conservative polish (recommended starting point)
python py_placer/place_optimize.py input.kicad_pcb optimized.kicad_pcb \
    --max-displacement 3 --length-weight 0.3 --crossing-penalty 30 \
    --halo-coef 0.15 --halo-weight 2 --edge-halo 2 \
    --ignore-nets GND "+3.3V" \
    --lock "J*" "P30*" "*PORT*"
```

Key options:

| Option | Default | Description |
|--------|---------|-------------|
| `--max-displacement` | 10 mm | Max distance a part may move from its seed position; applies to nudges and swaps alike (3 mm recommended; large values can destroy the placement's macro structure) |
| `--swap-max-displacement` | = max-displacement | Displacement cap for swap moves; must be ≤ `--max-displacement` |
| `--ignore-nets` | – | Net patterns excluded from airwire scoring (plane-routed power nets) |
| `--lock` | – | Reference patterns to pin in place (connectors, mounting-critical parts) |
| `--halo-coef` | 0.25 | Extra whitespace per √(pin count); keep modest (~0.15) on dense boards |
| `--brief` / `--no-brief` | the sibling `<board>.design-brief.json`, auto-discovered | The DECLARED design intent (#711): which connectors are user-facing, which edge each belongs on and where along it, what the enclosure forbids. Compiled into `edge_connectors` / `keepouts` by `check_floorplan --emit-intent`, so nothing downstream needs a second channel. A declared claim OUTRANKS the edge `_nearest_edge` would infer from a part's current pose. `--no-brief` is the OFF arm. See [docs/design-brief.md](../../docs/design-brief.md) |
| `--intent` | – | Floorplan intent JSON. Its declared zones, keep-outs and exclusive zones become HARD per-move gates; its `must_lock` globs and `edge_connectors` edge claims are locked. MONOTONE: it prevents a part being walked out of a zone, it does not walk one back in. Omitted, the run is bit-identical to one built before the flag existed (#702) |
| `--no-rotate` / `--no-swap` | off | Disable rotation / swap moves. `--no-rotate` freezes every part's angle: nudges keep the current rotation, and same-footprint swaps are restricted to pairs that already share one, since a swap exchanges full poses and a mixed-angle pair would rotate both parts |

## place_route_loop.py — router-in-the-loop repair

Routes the board with the real router, reads the failure diagnostics
(failed nets + the blocker nets named in the frontier analysis), and
micro-quenches only the small parts that could help those routes succeed,
weighting the failed nets so both their airwire length and any crossing they
take part in cost more. Re-routes and keeps the new placement only if
(failures, router effort) actually improves; otherwise reverts and widens the
search.

The tally counts the router's end-of-run reconciliation pass, so a net that
pass recovers is not treated as a failure in the next round. A rejected round
widens the **nudge** search 1.5×; the swap cap does not move. It stays at
`--swap-max-displacement` (default: the initial `--max-displacement`), so
widening the search can never turn into a long-range swap.
`--swap-max-displacement`, `--no-rotate` and `--no-swap` work here exactly as
they do in `place_optimize.py`, and `--verbose` surfaces each accepted quench
move plus the per-pass `swap-capped=N` count.

```bash
python py_placer/place_route_loop.py input.kicad_pcb repaired.kicad_pcb \
    --route-args '--nets "/*" "Net-*" --track-width 0.2 --clearance 0.2 ...' \
    --ignore-nets GND "+3.3V" --lock "J*" --swap-max-displacement 2
```

On the kit-dev-coldfire demo board this repaired the hand placement from
3 failed nets to 0 with 4.8× less router effort, moving only
resistors/caps/jumpers.

### Choosing the movers: `--target-select` (#553)

`--target-select pins` (the DEFAULT, unchanged) is the rule above: pad owners
of the failed and blocker nets, minus anything over `--max-target-pins`.

`--target-select diagnosis` ranks blocks and loose parts on three signals —
connectivity-centroid displacement, blocked cells owned, and legality defect
pairs — and takes the round-robin union of their top-k. It requires
`--group-by` (only the displacement signal needs blocks; the other two rank
loose parts, which is why the flag still does something on a board that
derives none). Each round is budgeted at the number of parts the pin filter
would have offered — so the two selectors are *asked* for the same size, but a
**block is added whole**, which makes the budget a floor rather than a cap.
Measured at budget 8, glasgow_revC selects 67 parts under `--group-by auto`,
because its one derivable block has 68 members. The overshoot is reported in
the run's own output; do not read "budgeted" as "bounded".

The budget is also how #553's own grading rule is answered. The issue asked for
a check that the diagnosis-selected set "differs from the pin-count set" — but
that is satisfied by *any* different function, including a random one, so on
its own it establishes nothing. Matching the two selectors' requested size
turns it into a comparison of *which* parts, not *how many*, and the per-round
`target_select_overlap` record (`{pins, diagnosis, overlap, fallback}`) is where
a run reports it. Equal numbers with `fallback: false` mean the diagnosis
independently agreed; equal numbers with `fallback: true` mean it was
discarded. Those are different facts and the flag distinguishes them.

```bash
python py_placer/place_route_loop.py input.kicad_pcb repaired.kicad_pcb \
    --route-args '--nets "/*" --track-width 0.2' \
    --target-select diagnosis --group-by auto,decap --diagnosis-top-k 3
```

**No measurement shows this routes better than `pins`, and the study built to
support it came back NULL.** `tests/stress/diagnosis_recall.py` displaces a
known block and asks whether the ranking concentrates on it, paired against the
same ranking on the UNDAMAGED board: evidence-arm median deltas **+0.135**
(4 cells up, 2 down) and **+0.000** (2 up, 3 down). The study, its rows and its
baseline are committed as a recorded negative rather than deleted; its
docstring explains the three ways an earlier reading of the same data looked
like a finding. The flag ships default-off, the run verdict carries the
disclaimer in `target_select_efficacy`, and
`target_select_rounds_diagnosis` beside `target_select_rounds_fallback` says
how often it steered anything at all. See `diagnosis.py`'s own docstring for
what backs each signal — which is not the same for the three — and for why
`foreign_crossings` is absent.

A run in this mode prints a group-source census before round 0, because
`--group-by auto` derives NO BLOCK on five of the six boards this repo grades
placement on.

### Moving the block: `--relocate` (#554)

The quench can translate a block rigidly (#538), but only while every member
stays within `--max-displacement` of its own seed and everybody else is frozen.
Measured with the neighbours frozen, a block travels a **median 0.00 mm toward
its connectivity target against a median want of 10.36 mm**, over the 24
measurable blocks on the 9 boards that have one — a shipped board has no vacancy to translate into. `--relocate`
(**default off**) lets the neighbours yield instead, with their relative order
as a hard constraint and their total displacement minimised, and writes the
result as its own board so the quench can refine the new pose. The round's
re-route accepts or reverts it, byte for byte.

```bash
python py_placer/place_route_loop.py in.kicad_pcb out.kicad_pcb \
    --route-args '--nets "*"' \
    --relocate --group-by auto,netprefix,decap --relocate-max-corridor 20
```

Every proposal reports the corridor (*who* yielded, and how far) and a
**binding chain** — the named parts and gaps that stopped the block, straight
off the constraint graph's own shortest path. Every refusal is a named reason,
never a count: `no_room_at_any_dose`, `block_member_locked:<ref>`,
`geometry_worsened:<ref>`, `declared_keepout_refused_a_shift:<ref>`,
`corridor_over_budget`, and the rest are enumerated in `relocate.py`.

**The mechanism holds; the routed result does not support the feature.**
Letting neighbours yield bought ≥ 1 mm more travel than freezing them on 11 of
the 24 measurable blocks over 6 of the 9 boards that have one, max 16.66 mm
(`tests/stress/relocation_reach.py`; its frozen arm is the same solve with
everything else pinned, so the arms differ in exactly one thing — and
`reach >= frozen` is a theorem, so the evidence is the magnitude, never a win
rate).

The routed A/B (`tests/stress/block_relocation_study.py`) has been run, and it
is **UNDERPOWERED and lost to the incumbent**: the relocation recovered damage
on 3 of 3 evidence cells, median delta **+0.57** — but over only **2 boards**,
where the acceptance rule counts ≥ 3, and `place_route_loop` with the pin gate
lifted reached a **strictly better** routed result on **2 of those 3** cells
(esp_prog/swap 0.83 vs 0.33; splitflap/swap 0.71 vs 0.57; they tie at 1.0 on
esp_prog/wrong_side). So the flag stays default-off, and the run verdict carries
that whole sentence as `relocate_efficacy`. Under pre-registered rule 12 this is
not yet a withdrawal — the study has not met its own N — but it is the
opposite of support, and the next run of it decides.

Three limits worth knowing before reaching for it: it never fires on a board
that already routes (the loop stops at `failures == 0`, and `--target-nets` does
not lift that stop); `--group-by auto` derives no block on most tracked boards;
and the one board where every precondition holds, kit-dev-coldfire, is already
taken from 3 failed nets to 0 by the shipped loop (at a cap widened to 6.75 mm
by round 4). See `relocate.py`'s docstring for
why the constraint graph is *not* a conservative model of legality, and why the
exact re-check rather than the graph is what makes the pass safe.

## place_portfolio.py — K diverse candidates from one placement

The quench is deterministic by design (#457), so re-running it never produces
a different placement: every run walks into the same local minimum. When the
question is "what are my placement OPTIONS", this tool generates them: legal
seeded perturbations of the input placement (`jitter` disc offsets, `poses`
rotation variants pruned by `pair_order` inversions, `swap` block-interior
position exchanges), each quenched with the ordinary engine, scored **without
routing**, pruned to a diverse slate, probe-routed at the top, and presented
as per-candidate renders plus `portfolio.json`.

```bash
python py_placer/place_portfolio.py board.kicad_pcb --out-dir pf --seed 0 \
    --intent floorplan.json --ignore-nets GND VCC
```

What the contract guarantees:

- **Candidate 0 is the plain quench of the input** — "keep what I have" is a
  first-class outcome, and its numbers equal a `place_optimize.py` run with
  the same knobs (asserted by `tests/test_portfolio.py`).
- **Same `--seed` + same input ⇒ byte-identical portfolio**, across
  `PYTHONHASHSEED` values. Each candidate draws from its own
  `random.Random(f"{seed}:{i}:{strategy}")` stream, so `--only N`
  regenerates candidate N alone, byte-identically — that is the replay
  primitive the `--ledger` records (`converge.py replay` runs it).
- **Hard gates, then an ungameable rank.** A candidate is ranked only if it
  adds no courtyard overlap and no out-of-board parts beyond the baseline's,
  and (with `--intent`) grades error-free. Rank is lexicographic over
  numbers the repo already trusts: `(crossings, inversions, hpwl,
  health_penalty, displacement, index)` — no new magic weights.
- **Diversity is pose distance, not hpwl distance** (hpwl reads per-net
  extremes, so a mirrored arrangement can tie a clone). Kept candidates sit
  ≥ `--diversity-mm` apart, the baseline included; when fewer survive, the
  slate is backfilled by rank and SAYS so.
- **Probe tier (`--route-top 2` by default)**: baseline + top kept candidates
  are actually routed (one shared affected-net set, so verdicts compare like
  with like; the `_ratsnest_screen` veto skips a probe on a candidate whose
  ratsnest clearly regressed). `portfolio.json` carries TWO rankings —
  `ranking_static` and `ranking_routed` — because measured copper and a
  proxy must not interleave in one list.

A board that already carries copper is refused (exit 3): placement moves
footprints, not tracks. Run the portfolio on the placed pre-route board and
re-route the chosen candidate.

## place_seed.py — intent-driven initial placement

The aided from-scratch path. Given an UNPLACED board (netlist import, a
generator's pile) plus a floorplan intent, it emits a legal starting
placement: edge connectors on their declared edge inside their overhang band,
single-ref zones at the spec coordinate, multi-ref zones packed radially,
everything else at the nearest legal pose to its connectivity centroid (which
is also what lands a decap next to its IC). The intent's `must_lock` refs are
stamped `(locked yes)` into the output, a quench polish tidies the free
parts, and the result is **graded against the same intent it was built
from** — a seed that fails its own intent exits 4, deliberately.

```bash
python py_placer/place_seed.py unplaced.kicad_pcb seed.kicad_pcb --intent floorplan.json
python py_placer/place_seed.py unplaced.kicad_pcb seed3.kicad_pcb --intent floorplan.json --seed 3
```

Rotations: the input rotation is tried in full first and kept when it fits; a
part with no contained legal pose at it falls back to its 90° lattice (noted
in the output — measured: an LDO with 0 legal poses at rot 0 and 3 at rot 90
on a packed board). A part whose rotation is a *decision* (pin order, the U3
rot-180 case) must be **locked** — the intent schema cannot express a
rotation, and an unlocked load-bearing rotation was never protected from the
quench either. Explore rotations deliberately with
`place_portfolio.py --strategy poses`.

### The eviction rung (`--evict-depth`, #630, #699)

A part with no legal pose is not necessarily a part with no **room**. Run 19
measured the difference: three sweeps returned a bare *"no legal pose anywhere
on the board"* for two switches, and when the question was finally asked in
scoped form the engine answered precisely: with D14 in place **0** poses,
with D14 lifted **46**; with D31, **0** then **32**. One eviction each and both
seated. That verdict was reachable the whole time and nothing asked for it.

So when a part cannot be seated, the seeder counts its poses with each nearby
seated incumbent lifted in turn. That census runs at every depth and is what
the JSON_SUMMARY's `no_pose_blockers` (`{ref: {blocker: poses_freed}}`)
reports, next to `unseated_refs` (names, not just a count). `--evict-depth 0`,
the default, stops there: *tell me what is in the way, move nothing*.

`--evict-depth 1` also trades: evict the incumbent that frees the most, seat
the blocked part **first** against the lifted board, then put the blocker back
(inside its own zone, searching out from its old pose) with the part in place.
That ordering is the point: `reseat_scope` re-seats its scope at their net
centroids, which is back into the pockets they block, and returned a null
three times on exactly this case. The trade is kept only if, in this order,
both seats were found, both are legal against **every** seated part (re-checked
with the seat predicate, not read off the search's return value; the two parts
are obstacles to each other), and the seated board's violation count and
overlap area did not rise. HPWL is recorded and never consulted: a part in the
pile has an artificially short HPWL, so any gate that ranks it refuses every
legal seat. A trade that fails puts both parts back and is recorded as
reverted (`evictions` counts kept trades, `evictions_reverted` the others).

**`--evict-depth 2` lifts a PAIR (#699).** A rung that only ever lifts one
neighbour records *"immovable"* for a part two neighbours jointly block, and
that verdict is true only of the basin the board happens to be in — the
reporting case censused 8 neighbours, none of which frees a pose alone, on a
board whose truth arrangement seats the part by moving two of them together.
Depth 2 asks the same question of pairs, and **only for a part no single lift
helped**: the pair sweep cannot be pruned by the single-lift counts, because
in the case it exists for every one of them is zero. The trade, the ordering
and the acceptance rule are the same code, not a second copy — the blockers
simply go back **hardest first** (descending courtyard extent), each with the
not-yet-returned ones still excluded, since a lifted part has not moved and
would otherwise veto from a pocket it is about to vacate. Nothing deeper is
defined: depth 3 is refused rather than silently meaning 2.

Bounded on every axis: **no recursion at either depth** (a blocker's own
blocker is not chased), at most 8 candidates per part (a geometric superset,
so a part outside the box frees zero poses by construction), at most
`EVICT_MAX_PAIRS = 16` of the C(8,2) pairs — ordered by `(i + j)` over the
nearest-first candidate list, so truncation drops the far-far pairs instead of
starving one candidate of partners — **one trade per part** (a single lift
that was useful but whose trade reverted does *not* fall back to a pair), and
the census counts to a cap. Every one of those is a **count**, never a clock:
a wall-clock budget would place the same board differently on a slow machine
and a fast one (#621). What a cap drops is reported, not silently omitted.
Locked parts and declared edge connectors are never candidates. The rung only
fires on a part that was going to be reported unseated, so a run that seats
everything is unaffected at any depth. It is opt-in until an A/B row on three
boards exists (`tests/test_placement_ab.py`).

**The verdict says WHY, not just who (#699).** `no_pose_blockers[ref] == {}`
used to mean two different things with opposite answers for the reader —
"nothing seated is near this part" and "everything near it is locked" — and at
depth 0 a census that freed nothing printed nothing at all. Every unseated
part the rung reaches now carries a `no_pose_verdict` and a `no_pose_census`,
in the JSON_SUMMARY and as a NOTE:

| verdict | what it means | what to do about it |
| --- | --- | --- |
| `keepout_blocks` | a **declared keep-out** is what refuses it — measured, not inferred: the poses are recounted with that keep-out lifted (#701) | move the keep-out, or add the part to its `allow` list if it owns it |
| `no_movable_neighbour` | nothing seated is near enough to be in the way | the outline, the zone or the part's own size refuses it |
| `immovable_given_frozen` | the only neighbours in the way are locked or declared edge connectors, **named with the decision that froze each** | relax that lock, or accept the pose |
| `no_single_lift_frees` | movable neighbours censused; no single lift frees a pose | try `--evict-depth 2` |
| `no_pair_lift_frees` | ...and no pair of them frees one either | the room is not there |
| `blocker_available` | a lift *would* free a pose; the depth declined to move | raise `--evict-depth` |
| `trade_reverted` | a trade was attempted and put back | read the recorded conjunct |
| `seated_after_eviction` | not unseated after all | — |
| `no_target_recorded` | the rung never got to ask (no recorded seat context) | — |

`no_pose_census[ref]` carries the counts those verdicts came from — `boxed`,
`movable`, `censused`, `frozen`, `truncated`, `baseline`, `pairs_total`,
`pairs_censused`, `pairs_truncated`, `best_pair`, `keepouts_freeing`,
`keepouts_joint` — so a capped sweep can never
read as a complete one. `keepouts_freeing` is `{keep-out name: poses freed by
lifting it}`, filled only for a part with no pose at all and only over the
keep-outs that bind it; it is the count `keepout_blocks` is derived from, so
the verdict cannot drift from a differently-computed claim.
`keepouts_joint` is the poses freed by lifting **every** bound keep-out at
once, and it exists because two that overlap over the part's feasible region
each free *nothing alone* — so `keepouts_freeing` is `{}` and, without this,
the verdict would fall back to `no_movable_neighbour` and blame the outline.
`movable` and `censused` are deliberately separate:
the first is how many neighbours *could* have been censused, the second how
many were, and quoting the first as the second is the inversion the whole
disclosure exists to prevent.

**On the `--reseat` path** the same flag applies and the same keys are
carried. Depth ≥ 1 is the one exception to that pass's *"every other part is
held fixed"* contract: it may trade out a seated **non-scope** neighbour, and
those refs are named in `evicted`, in a NOTE, and in `moves`. They have to be
in `moves` — that list is the whole of what gets written, so an evicted part
left out of it is written at its **old** pose while the scope ref takes the
pocket it vacated, which is overlapping copper reported as success. `evicted`
names what reached the **written** board: a pass the gate refused wrote
nothing, so it reports none even though `evictions` records the attempt.

An evicted part is **exempt from the per-part prune sweep**, and that is what
makes the trade atomic rather than a nicety. `prune_assignment` reverts a
moved part whenever restoring its input pose strictly improves the gate tuple
— `evidenced` gates only the *equal* case — and `GATE_TERMS` ranks `hpwl`
above `overlap`, so putting an evicted blocker back into the pocket the trade
just gave away scores as an improvement. Measured: prune restored a blocker
inside the seated part's courtyard on hpwl 24.4 → 14.4, the licence then
correctly refused the board prune had damaged, and a legal pair trade that
would have taken `oob` 9.65 → 0 was thrown away whole. `exempt`'s own
rationale — *"an edge-class seat is hpwl-worse BY DESIGN; pruning it back
would undo the seat one stage later"* — is exactly this case.

The pass additionally refuses any eviction that raised the board's stack count
or overlap area (`eviction_licence_ok`): its ordinary gate compares `oob`
lexicographically first, and `oob` moves hugely in this pass's own favour, so
a new stack would sit below it unread. At depth 0 — the default, and what
`place_reconstruct`'s reseat rung uses — nothing outside the scope is touched
and the counts are 0.

**An EXPLICIT scope is accepted on a different rule from the auto one (#698),
and the difference is structural rather than a preference.** On
`auto:damage_witnesses` the pass's win *is* `oob`, at index 3 of the gate tuple
— **above** `hpwl` — so the lexicographic compare already sees it, `prune`
cannot revert a genuine homecoming, and `after[oob] < before[oob]` is a complete
rule. On an explicit scope the win is a declared claim or the scope's own
wirelength, which the tuple cannot see **at all**; `after[oob] < before[oob]` is
then unsatisfiable for any part that is on the board, so an explicitly named
part could never be re-seated whatever the search found. So:

- **Safety is TERM-WISE, not lexicographic.** Every gate term must not worsen
  except `hpwl`, the one licensed term — a seat made for a declared reason is
  hpwl-worse *by construction*, for the same reason `exempt` gives above.
  Measured on #698's fixture, swept over 20 seeds: escaping the declared
  keep-out costs hpwl on **20 of 20** — 6.82 to 12.39 mm, a different value per
  seed, since the seat search is seeded — so a lexicographic `after <= before`
  refuses exactly the case the change exists for. `oob` stays hard but is no
  longer required to *improve*, and that one asymmetry is the whole bug.
- **A separate trigger.** At least one basis in `RESEAT_BASES` must strictly
  improve: the six hard gate terms, the scope's own HPWL, and the count of
  breached declared claims. All are reported in `accept_basis` whether they
  fired or not — a basis that measured nothing and a basis that measured no
  change must not look alike.
- **The intent VECTOR is the guard; the intent COUNT is only the trigger.** A
  bare count carries the trap `quench._IntentTerm` names — a part hopping from
  keep-out A into keep-out B reads `1 -> 1`, which a monotone rule admits — so
  `IntentProbe.licence` separately refuses any term that *rose*, termwise and
  never summed.
- **`prune_assignment` had to be taught the same thing**, because it reverts
  *first*. Its tuple has no intent term either, so a seat that cleared a
  keep-out reads as a pure hpwl loss and was undone before the gate ran. It now
  takes an `intent_probe` and refuses a revert that would re-break a
  declaration — a conjunct rather than an `exempt` entry, so the sweep still
  catches every mis-move it caught before and stays monotone, now on
  `(tuple, intent vector)` jointly. Kept moves are named in a `prune: KEPT …`
  note.
- **`--reseat-min-gain` (mm) gates the `scope_hpwl` basis only.** Count bases
  threshold at one whole defect; one number compared against both currencies
  would assert an exchange rate between half a millimetre of wire and half a
  keep-out violation. Every continuous basis is *also* floored at
  `MEASURE_QUANTUM`, the last digit `reconstruct.measure` keeps for the 4-dp
  legality terms: without it the old `gain > 1e-9` let an `overlap` improvement
  of 1e-4 mm² — a change in the last representable digit — accept a pass, and
  since `hpwl` is the licensed term such a pass may be arbitrarily worse on
  wirelength. The floor is also what makes the rule **monotone in
  `--reseat-min-gain`**: a threshold below the quantum falls through to it, so
  a stricter flag can never produce a looser gate.

  The default is **0.0**, from `tests/measure_698_min_gain.py` (16 explicit
  re-seats on four corpus boards, parts chosen by pad count so the sample cannot
  be fitted to the conclusion). **Read that script's `pre_prune` column before
  quoting it**: the gains look bimodal — 10 exactly 0.000, and 6 running 1.004
  to 19.875 mm — but 9 of the 10 zeros are seats `prune_assignment` REVERTED
  (the search relocated by tens of millimetres to an hpwl-worse pose, so `after`
  is the restored input pose), and only one is a genuine no-op. The bimodality
  is therefore partly structural: anything surviving prune has improved hpwl by
  construction. What the sample does support is narrower and still sufficient —
  every re-seat whose gain was **non-zero** gained at least ~1 mm, so a non-zero
  default would buy nothing here while risking a genuine small win.

Requires an Edge.Cuts outline (exit 3 without one — the outline is spec-owned
and will not be invented) and refuses a board that already looks placed
(use `place_portfolio.py` to explore around an existing placement, or
`--force` to discard it). Different `--seed` values give genuinely different
legal seeds; the same seed reproduces byte for byte. The two compose:
`place_seed --seed N` → `place_portfolio` diversifies and ranks.

## place_fanout_clearance.py — decoupling-cap clearance repair (issue #130)

Run **after** `bga_fanout.py`. Nudges decoupling caps near a BGA so their
pads clear every foreign-net fanout via, every foreign track on the cap's
own copper side, and every foreign component pad (#130/#278/#275 — a graze
already present at the seed placement is a violation to fix, not a baseline
to preserve), and pulls each pad toward the nearest **same-net** ball — so
a power/GND via dropped at that ball later also lands on the cap pad (one
shared via connects ball + cap + plane). Caps move as little as possible
(90° rotations allowed), never overlap each other or a locked part, and a cap
that can't clear within the (auto-grown) displacement budget is reported
unresolved for a manual nudge.

The summary line reads `Moved N cap(s); resolved R/V initial violations;
K unresolved`. Since #746 both counts are graded from the **same** board state,
at the end of the pass: `resolved` means "was grazing at the seed and is clean
now" and therefore credits the #313 via-nudge as well as the cap move, with
`(F freed by via-nudge)` naming that share. `unresolved` means "grazing now",
which is not a subset of the seed violators — copper the pass itself drew can
break a cap that started clean, and when it does, a `Re-grazed by this pass's
own connector copper:` line names those caps. Before #746 `resolved` was
computed before the nudge and never refreshed, so a cap the nudge freed reached
neither list, and a cap the sweep had cleaned before the pass re-grazed it
reached both.

It reads each via's actual size from the board, so the only setting that
matters is `--clearance`, which must match the fanout / DRC floor:

```bash
# after: bga_fanout.py board.kicad_pcb -o fanned.kicad_pcb --clearance 0.1 ...
python py_placer/place_fanout_clearance.py fanned.kicad_pcb capclean.kicad_pcb --clearance 0.1
```

| Option | Default | Description |
|--------|---------|-------------|
| `--clearance` | the board's own Default net-class clearance, else 0.25 mm | Copper clearance **CEILING**, and the PRESENCE of the flag is the clamp switch -- the same two branches CLAUDE.md documents for `route.py` (#768). GIVEN: every class is priced at `min(class, this)` and the output `.kicad_pro` is clamped down to it. OMITTED: each pair is priced at its own net-class clearance and the classes are PRESERVED. Before #768 this step ran one branch for pricing and the other for the writeback in the same invocation, so `--clearance 0.1` on a board declaring 0.2 moved caps to satisfy 0.2 and then shipped a project saying 0.1 (measured on `glasgow_revC`, 19 pairs, and `ottercast_audio`, 79). Only the NETCLASS tier is capped: a `.kicad_dru` layer rule and a pad `local_clearance` still outrank the ceiling, because the writeback clamps neither and KiCad goes on enforcing both. `--clearance` is a copper-clearance ceiling only -- it no longer redefines `min_hole_clearance`, which the writeback now passes from the board's own declaration. A value below the layer-bucketed fab clearance floor (0.10 mm at <=2 copper layers, 0.09 mm at >=3) is DISCLOSED and honoured, not raised, because `check_drc` does not raise it either. |
| `--cap-prefix` | `C,R,FB` | Comma-separated reference prefix(es) for movable passives near a BGA (caps, resistors **and ferrite beads** by default, #252). Only 2-copper-pad parts move, so RN-style arrays are auto-excluded; paste-only apertures are ignored when counting pads. |
| `--capture-radius` | 2 mm | Max distance over which a same-net ball attracts a pad |
| `--max-displacement` / `--max-displacement-cap` | 2 / 3 mm | Initial and grown move budget per cap |
| `--default-via-size` | 0.3 mm | Fallback only, for vias with no readable size. Honoured by the grader **and** the via-nudge since #732; before that the nudge priced such a via at a hard-coded 0.5 and the two disagreed. |
| `--board-edge-clearance` | the board's own `min_copper_edge_clearance` when it asks for MORE, else 0.55 mm | Copper-to-Edge.Cuts margin for a moved cap **and** for a via the #313 nudge relocates -- one number since #733, where the nudger gated its own emitted copper at the bare `--clearance` and parked it 0.30 mm inside the band the cap mover reserves. Resolved by the shared engine, so the GUI plugin and `animate_fanout_clearance.py` get the same answer; TIGHTEN-only on an omitted flag, because `fix_project_for_output` pins this field up to the 0.20 fab floor on every board the chain writes. A given value is honoured as typed. |
| `--lock` | – | Extra reference patterns to pin in place |
| *(no flag)* track-scoped `.kicad_dru` rules | the board's own custom rules | What the #313 via-nudge charges between the connector copper it draws and a foreign **track**. KiCad stores a track-to-track requirement as a custom rule scoped to a net class (`A.Type=='track' && B.Type=='track' && A.NetClass=='X'`); netclasses cannot express it, and before #735 this pass could not read it, so on a declaring board it drew connectors closer to foreign copper than `check_drc` accepts -- an under-block, which ships the violation rather than refusing the landing. **RAISE-only** over the pair's already-resolved value, and **tracks only**: the cap-pad, board-pad and via arms are exempt by KiCad's own condition, not by omission. The board's value is a **PREFERENCE, not a gate**, the same shape as the drill floors above: the sweep runs every drill rung honouring the rule first and only falls back to the base requirement if nothing clears, saying so on stdout -- because a hard gate would abandon the via and leave the pad-via graze this pass exists to remove, which `check_drc` counts too. Resolved by the shared engine through the same `kicad_dru.track_pair_clearance` `check_drc` grades with, so the GUI plugin and `animate_fanout_clearance.py` get the same answer; an unsaved GUI board has no path to read a `.kicad_dru` from and keeps the netclass value. No board in this repo ships a `.kicad_dru`, so the channel is inert on the whole tracked corpus. |
| *(no flag)* drill-to-drill floors | the board's own `min_hole_to_hole` when it asks for MORE, else the fab tier's 0.20 (via-hole to via-hole) and 0.45 (via-hole to pad-hole) | What the #313 via-nudge charges when it relocates a barrel. Board-first and RAISE-only since #756; before that both were flat literals, so on a board declaring above 0.20 the pass parked a via at 0.20 while `check_drc` graded the same drill pair at the declared value and flagged it. Resolved by the shared engine off the board's sibling `.kicad_pro`, so the GUI plugin and `animate_fanout_clearance.py` get the same answer; an unsaved GUI board has no project to read and keeps the fab floors. The board's value is a PREFERENCE, not a gate: the nudge sweeps for a landing that clears it and falls back to the fab floor rather than abandoning the repair, so it can never place a via worse than it would have before. `--fab-tier` cannot move these floors (both tiers declare 0.20/0.45); a `--fab-overrides` file can, but note `place_fanout_clearance.py` accepts neither flag — the fab tier reaches this pass only as the process-wide value some other step set, which is how the GUI's Fanout tab supplies it. The pad-hole floor stays stricter than `check_drc`'s pad-drill arm (which grades at the single hole-to-hole value) by `max(d, 0.45) - max(0.20, d)` — 0.25 mm on a board declaring nothing, decaying to 0 at 0.45. Deliberate: 0.45 is the JLC fab minimum and nothing else in the repo enforces it. |

On ulx3s U1 (22×22, 0.8 mm) this took the fanned board from 4 PAD-VIA to
fully DRC-clean, tidying 19 caps toward same-net balls (all ≤1.9 mm). In the
GUI, the **"Optimize decoupling cap placement"** checkbox on the BGA fanout
tab runs the same engine automatically right after fanout (off by default).
The advanced knobs above (capture radius, near margin, search step, max
displacement / cap / growth, max passes, cap-ref prefix, allow-rotation) are
exposed in that tab's **"Cap Placement (advanced)"** box; `--clearance`,
`--grid-step`, and the via size come from the Basic tab.

### animate_fanout_clearance.py — visualize the repair as a GIF

`animate_fanout_clearance.py` (repo root) runs the **same** repair engine and
records every accepted cap move via the engine's optional `on_move` hook, then
renders an animated GIF of the caps gliding from their seed placement to their
final, via-clearing positions. The view is framed to the BGA ball field (not
the whole board); fanout vias appear as net-coloured disks with their keep-out
ring, cap pads are coloured by net, and a faint ghost rectangle marks each
cap's seed position. It accepts all of `place_fanout_clearance.py`'s repair
options plus `--size`, `--fps`, and `--sub-frames` (motion smoothness).

![Decoupling caps gliding off foreign-net fanout vias on the glasgow revC BGA](../../docs/fanout-cap-placement.gif)

```bash
# after: bga_fanout.py board.kicad_pcb -o fanned.kicad_pcb --escape-method underpad \
#            --via-size 0.3 --via-drill 0.2 --track-width 0.1 --clearance 0.1
python py_tools/animate_fanout_clearance.py fanned.kicad_pcb capmove.gif --clearance 0.1
```

This is a read-only visualization tool: the `on_move` hook defaults to `None`,
so `place_fanout_clearance.py`, the GUI, and the engine itself behave exactly
as before when it is unused. Requires only **Pillow** -- as of #431 it renders
through `route_render.BoardRenderer` and encodes through
`animate_route.save_movie`, so it no longer carries its own world->pixel
transform, GIF writer or font handling, and `pygame` is no longer needed. Two
things came free with the port: the **real board** beneath the BGA field
(outline, cutouts, zones), which the pygame version never drew, and `.mp4`
output (extension picks the format, falling back to a sibling `.gif` when
imageio-ffmpeg is absent).

## Testing

The placement tests are standalone scripts (no pytest needed), all in
`tests/run_all.py`'s `--fast` lane:

```bash
python3 tests/test_quench_swap_cap.py        # swap displacement cap (#430)
python3 tests/test_quench_neighbor_lists.py  # pruned-scan bit-exactness (#430)
python3 tests/test_458_loop_steering.py      # loop caps, tally, summary merge
python3 tests/test_458_quench_net_weights.py # weighted crossings
python3 tests/test_458_quench_rotations.py   # rotation lattice, --no-rotate
python3 tests/test_fanout_clearance.py       # cap clearance repair (#130)
python3 tests/test_456_courtyard_parser.py   # courtyard shapes + silk bleed (#456)
python3 tests/test_456_side_and_outline.py   # board side, real outline, graders (#456)
python3 tests/test_459_groups.py             # block sources + parsing (#459)
python3 tests/test_459_group_moves.py        # rigid block translation (#459)
python3 tests/test_554_order_graph.py        # the relocation constraint graph (#554)
python3 tests/test_554_relocate_solve.py     # min-perturbation + the exact re-check (#554)
python3 tests/test_554_loop_relocate.py      # --relocate wiring, mocked router (#554)
python3 tests/test_554_reach_regen.py        # the reach measurement, re-derived in full
python3 tests/mutate_554.py                  # does the #554 suite bite? (rewrites relocate.py)
python3 tests/test_portfolio_strategies.py   # perturbation strategy invariants
python3 tests/test_portfolio_determinism.py  # portfolio seed/replay contract (slow)
python3 tests/test_portfolio.py              # portfolio smoke + identity anchor (slow)
python3 tests/test_place_seed.py             # intent-driven seeding (slow)
```

Quench output is reproducible across processes — the same board and arguments
give the same placement, with no `PYTHONHASHSEED` pinning (#457). It used to
depend on the hash seed: `net_refs` was a set of reference strings, its iteration
order became the MST's point order, and Prim's tie-break is first-index-wins, so
equidistant pads (uniform-pitch GND arrays, decaps on a grid, symmetric
connectors) built a different tree per process. `interf_u_unrouted` scored 447 /
457 / 450 crossings under three seeds before a single move was made. `net_refs`
now holds sorted lists, and `tests/test_457_determinism.py` pins it.

When comparing two placements, `hpwl` is the metric to reach for first: it reads
only each net's pad-position extremes, so unlike the MST length and the crossing
count it is invariant to airwire order. If HPWL agrees and crossings do not, the
two runs differ in tie-breaks rather than in placement quality.

## Ratsnest metrics, and the pre-route screen (#504)

The quench cost function computes airwire length and crossings on every pass, and
`hpwl()` is pure pad geometry. Those numbers used to be printed and discarded.
They are now exported:

- `quench(..., metrics_out=d)` fills `d` with `{'before', 'after', 'legality'}` —
  an out-param rather than a changed return, since the return is consumed
  positionally by both CLIs and four test files.
- `place_optimize.py` emits them as a `JSON_SUMMARY:` line, so a chain or grader
  can gate on what a run achieved instead of scraping stdout.
- `place_route_loop.py` records `ratsnest_crossings` / `ratsnest_hpwl` /
  `ratsnest_length` in each round's metrics dict, report-only beside the
  `pad_pairs_*` keys. `better()` is deliberately untouched — reworking the
  comparator is #458.

**Which numbers compare.** `crossings` (a raw count by contract) and `hpwl` are
unweighted, so they are comparable across runs. `length` and `total` are scaled
by `net_weights`, so they only compare between the `before` and `after` of the
*same* call — which is why the screen thresholds on the first two.

**`--ratsnest-screen N`** (percent, `0` = disabled, the default) skips the routing
run when a candidate's crossings or HPWL regress by more than N% against the board
it came from. Routing is the honest judge but an expensive one, often minutes per
round; a candidate whose ratsnest clearly got worse is very unlikely to route
better. The baseline is free — quench is handed the current best board, so its own
`before` *is* that board's ratsnest. Every decision is logged with its numbers, so
it is auditable whether the screen ever skipped a placement that would have won.

## Alignment and orientation (`--align-weight`, `--orient-weight`, #548)

Two cost terms for the things a human does constantly and the quench does not:
put parts that belong together on a shared axis, and turn a part toward the net
it exists to serve. **Both default to 0 (off).**

```bash
python3 py_placer/place_optimize.py board.kicad_pcb out.kicad_pcb --max-displacement 3 \
    --align-weight 5 --orient-weight 1
```

### The premise in #548 is wrong in a way worth recording

The issue proposes to *"score airwires from the actual pad the net lands on"*.
The cost path **already does that** — `_net_points` emits one MST node per
connected pad from `pad_globals()`, full rotation applied, and there is no
centroid anywhere in the objective.

The gap is **numeric**, not geometric. Measured on the test fixture: the four
rotations of a part whose one connected pad faces away from its anchor differ by
**0.500 mm across a 19.8 mm net**. The directional signal is present and drowned.
So what was missing is a term that is directional at *part* scale with its own
weight, which is what `--orient-weight` is.

### Alignment

A pairwise penalty between **peers** — same `footprint_name`, the pairing the
swap phase already indexes — on the nearer shared axis:
`w * min(d, radius)²`.

- **Continuous** at the radius. The obvious charge-inside/zero-outside shape has
  a cliff there that pays a part to *flee* the row rather than join it.
- **Saturating** beyond it, so a distant peer contributes a constant that
  cancels between one part's candidate poses instead of dragging it across the
  board.
- **Zero** on a shared axis, so a tidy row is free.

Four caps seeded at y ∈ {99.8, 100.0, 100.2, 100.3} come out at four distinct y
today, and on one shared y with the term on.

The peer index is built in `__init__` and deliberately does **not** ride on the
pruned neighbour lists: that prune is a 2-D *box* overlap test, so a pair must be
near in both axes — but alignment is inherently long-range *along* the shared
axis (two caps 50 mm apart in x and 0.1 mm apart in y **are** aligned). It is a
*lossy* prune, unlike `_neighbors`' exact one: peers are fixed from seed
positions, so a pair that drifts within `--align-span` later is not picked up.

### Orientation

Sums `|r| − r·û` per connected pad, where `û` points from the pose origin toward
the centroid of that net's pads owned by *other* parts. Zero facing the anchor,
`2|r|` facing away — **bounded at part scale on purpose**: it breaks a rotation
tie, it does not outrank a real length win.

Two things it is not, worth knowing: it is not purely rotational (moving the part
changes `û` too, so a large weight also pulls the part toward its nets), and
through `part_geometry_cost` it reaches the group phase, so a rigid block
translate is steered by it as well.

### Measured, so the recommendation is not a guess

Sweep at `--max-displacement 3`, 3 passes. *tidy* is the mean share of distinct
axis positions within each same-footprint group — **lower is tidier**.

| board | align | orient | crossings | hpwl | tidy | overlap / oob |
|---|---|---|---|---|---|---|
| splitflap_driver | 0 | 0 | 194 | 2407.1 | 0.652 | 0.0 / 6 |
| | 5 | 0 | **187** | 2386.2 | 0.581 | 0.0 / 6 |
| | 15 | 0 | 187 | 2404.7 | **0.560** | 0.0 / 6 |
| | 0 | 3 | 185 | 2415.5 | 0.712 | 0.0 / 6 |
| | **5** | **1** | **185** | 2391.2 | 0.605 | 0.0 / 6 |
| interf_u_unrouted | 0 | 0 | 404 | 4175.0 | 1.000 | 0.0 / 2 |
| | 5 | 0 | 404 | 4174.5 | 1.000 | 0.0 / 2 |
| | **5** | **1** | **402** | **4162.3** | **0.889** | 0.0 / 2 |

**`--align-weight 5 --orient-weight 1`** is the recommended pair: best or
joint-best crossings on both boards, tidier, and **no legality regression
anywhere** — overlap stayed 0.0 and `oob_count` never moved.

Two honest caveats. **n = 2 boards, and no re-route** — crossings and HPWL are
proxies, and the router is the only judge that counts, so do not read the
crossing improvements as a routability claim. And **over-weighting trades
wirelength for tidiness**: `--align-weight 15` is the tidiest row in the table
and its HPWL is *worse* than at 5. `interf_u_unrouted` shows the other limit —
alignment does nothing at all on a board with few repeated footprints, because
there are no peers to align.

### `5` is not a transferable number

The two sweeps above make `--align-weight 5` look like a default. It is not. On a
third board — 42 footprints, 16 identical 0402 decouplers in two exactly-aligned
rows, so peers were not the limit — `5` destroyed both rows just as thoroughly as
switching the term off, at **either** crossing penalty. The mechanism was fine:
at `50` both rows returned to a `0.0000` y spread. It was the *weight* that did
not transfer.

The reason is in the shape of the term. The penalty saturates at `w · radius²`, so
the recommended pair is worth `5 × 0.5² = 1.25` per peer pair — against a halo term
of order 100s and tens per crossing. It is simply outvoted.

**Sweep `--align-weight` on the board in front of you and check a row's y spread;
do not carry `5` over.** And note what a weight large enough to win can cost: on
that board `50` bought aligned rows at +3 crossings and +3.2 mm HPWL, which the
Step 0c acceptance rule scores as a regression.

### Why off by default

The router, not a tidiness score, is the judge of a placement, and neither term
has been measured against routing *outcomes* — only against proxies, above.
On-by-default would silently change every user's board to buy legibility.

It would also corrupt the isolated fixtures in `test_458_*`, which zero every
geometry knob they know about so the objective is clean enough to assert
`total == 30.0`. A term with a nonzero default is invisible to those and breaks
the isolation — that is degrading a correctness test, not re-baselining a golden.

At `0.0` both hooks return before touching any geometry and the peer index is
empty, so a default run is **bit-identical**, verified on `interf_u_unrouted` and
`splitflap_driver` rather than argued.

## Placement blocks (`groups.py`, #459)

The per-part nudge cannot express "these parts need to travel together": an IC
and its decoupling caps fight each other one at a time, because moving either
alone worsens the pair. `--group-by` gives a block a **rigid translate** move —
the whole body shifts by one offset.

Sources, in precedence order, first match wins per part (a part is in at most one
block):

| source | what it is | corpus evidence |
|---|---|---|
| `kicad` | KiCad `(group ...)` blocks | **0 of 27** in-repo boards have one. Exact when present; verified against a synthetic fixture. |
| `sheet` | schematic sheet path | **12 of 22** boards with `(path ...)` have >1 sheet. `ulx3s`: 10 blocks sized 83/34/23/20/20/12. The workhorse. |
| `netprefix` | net-name prefix (`AUDIO_*`) | The **weakest** source. Raw prefixes are dominated by KiCad's auto-generated `Net-(U1-Pad)` names — one bogus bucket of up to 92 refs spanning 75mm — so those and power nets are excluded and a 20mm coherence gate applied. What survives is small but real: ulx3s 9 blocks, tigard 0. |
| `decap` | 2-pad caps tethered to their IC | Strong. A cap sits 0.0–2.6mm (median) from the nearest IC and **shares a net with it 93–100%** of the time; the net check rejects the rest (13 of ulx3s' 70). |

`--group-by auto` means `kicad,sheet`. Default is `none`: grouping is opt-in, and
with it off the group phase never runs and output is byte-identical to the
ungrouped engine — which is what lets every existing bit-identity test stand.

**What actually moves.** Small blocks move; large ones do not. On
`splitflap_driver` with `--group-by decap`, 6 blocks of 2–3 parts translate and
the objective improves (total 4781.6 → 4689.7, crossings 211 → 208, HPWL
2468.0 → 2436.3 against the ungrouped run). Sheet blocks of 16–83 parts moved on
none of the boards tried — at a few mm of displacement a rigid shift of that many
parts rarely finds a legal, improving offset. That is the expected shape: #459's
80mm block *relocation* is separate, unimplemented work.

**Why the cap is per-member.** Every member must land within `--max-displacement`
of **its own seed**. That single rule is what keeps three existing guarantees
true: `build_neighbor_lists`' pruning stays exact rather than lossy, the outline
gate's per-ref cached reach cannot be outrun, and the no-stranding invariant
holds unchanged. Lifting it is precisely what makes the 80mm case hard.

### Blocks are also a ROUTING scope

The same blocks drive `route.py`: `--group BLOCK` scopes a routing run to a
block's nets, `--preview` reports what a run would add without writing a board,
and `--undo` strips a block's copper back to unrouted. `--list-groups` prints
what is inferred, with both net counts, before you trust any of it.

Which nets a block "owns" has two honest answers, so `--group-scope` picks:

| scope | means | when it is the right one |
|---|---|---|
| `internal` | every pad of the net is inside the block | a schematic sheet — measured **60–70% internal** (glasgow's two 68-part sheets: 52 internal vs 23 boundary), so "route this sheet" is a real self-contained job |
| `touching` | any pad inside the block, interface nets included | matches what `--component` already means, and it is the **only** useful reading for a `decap` block: those are **0% internal** by construction, since a decoupling cap bridges VCC to GND and both span the board |

**The default depends on the operation**, because the same set of nets is right
for one and dangerous for the other. Routing defaults to `touching` — routing a
block's interface is the point. `--undo` defaults to `internal`, because a
block's touching set contains GND/VCC, and undoing *those* strips their copper
across the **whole board**: on the rp2350 fixture, `touching` removes 170
segments, 54 of them nowhere near the block, versus 75 for `internal`. Asking
for `--group-scope touching --undo` explicitly still works, and warns.

`--undo` refuses KiCad-**locked** copper — `locked` gets no override anywhere
else in the toolchain (#521) and an undo is not the place to invent one, in the
segment path or the arc path. Nets protected for a *reason* (length-matched,
diff-pair) are removed, because naming a net exactly is the deliberate targeting
those reasons exist to allow. It carries the sibling `.kicad_pro` across (#441):
an undo only removes copper, so the input's DRC floor is still the correct one
and must travel.

It removes `(segment ...)` **and** `(arc ...)` tracks. Arcs need their own pass:
the parser *linearizes* them into `pcb_data.segments`, so an undo counts them,
but the text writer only matches `(segment ...)` blocks — without the arc pass an
arc-routed (i.e. hand-routed) net kept its copper while the run reported success.

What `--undo` does **not** do, and says so at runtime:

- **Zone pours.** A filled plane is copper, so a net with a surviving pour is not
  back to "no copper". Deleting a pour is a plane decision, not an undo, so it
  reports and leaves it — re-routing such a net gives a track web beside the
  pour, not the original plane.
- **Pad/target swaps.** No inverse exists, and they can touch nets outside the
  scope. It returns the named nets to "no copper", not the board to a prior state.
- **Copper graphics.** Net-tagged artwork has no `(segment)` block to delete; it
  is counted and named rather than silently left behind.

It also **refuses to run unscoped** — for a routing run "no scope" sensibly means
the whole board, but for an undo that silently means erasing every track on it.

## Board-state gates (#431)

Both CLIs refuse, with **exit code 3**, two board states they cannot do anything
useful with. `0` ok, `1` crash, `2` argparse, `3` "the board is not in a state
this tool can work on" -- a distinct code so a caller branches on the number
rather than scraping text.

| state | why refusing beats trying | override |
|---|---|---|
| **unplaced** (parts stacked at one coordinate) | the quench REFINES a placement. On a pile every candidate pose is illegal, so the run prints "0 parts moved" plus a legality block that looks like a result, and a large `--max-displacement` yields a tiny scatter around the origin that looks like progress | `--allow-unplaced` |
| **already routed** | the quench models no copper at all: legality is courtyard + outline, cost is pad-to-pad airwires, and `writer.write_placed_output` rewrites footprint positions only. Every track would be left behind, detached from its pad | `--allow-routed` |

`place_route_loop` gates BEFORE round 0, which routes the whole board -- refusing
there saves minutes-to-hours of A* that would fail everything and then quench a
pile. `render_placement.py` WARNS and renders instead: being able to SEE an
unplaced board is the point of having a renderer for one.

Detection is board-relative. Coincident positions is the only signal strong
enough to fire alone -- two parts at the *same* coordinate is physically
impossible in a real placement, however dense. Low spread never fires alone. The
outside-the-outline share fires at 0.9, not 0.5, because castellated boards
legitimately overhang; and with no usable outline it is **unavailable**, never
true, so a misparsed outline cannot condemn a placed board. Measured: none of
the 27 tracked boards trips any of it, `watchy` included -- and `watchy` is the
worst case, with 81 of 82 parts in courtyard violation. Density is not
unplacedness, which is why this does not live in `legality.py`.

## Floorplan intent (`floorplan.py`, #549)

Everything above judges a placement by `crossings` and `hpwl`, and both are
indifferent between a sensible layout and a scattered one with the same
wirelength. Nothing declares where parts *belong*, so nothing can check whether
they went there.

`check_floorplan.py` closes that: declare the floorplan, grade the board against
it, exit non-zero with the number that broke.

```bash
python3 py_tools/check_floorplan.py board.kicad_pcb --emit-intent floorplan.json   # start here
python3 py_tools/check_floorplan.py board.kicad_pcb --intent floorplan.json        # 0 clean, 4 violations
```

Full reference: [docs/floorplan-intent.md](../../docs/floorplan-intent.md). The
parts worth knowing from here:

- **The board outline is not editable by this toolchain.** `envelope` is READ
  from the board; a part outside it is a finding about the **part**. Board size,
  cutouts and slots are mechanical decisions the user owns.
- **Every rule reuses the geometry the optimizer gates on** — `QuenchState`'s
  `legality_metrics` and `edge_gate`, `GradedPart` rects and sides, and
  `groups.decap_tethers` for the cap→IC rule. A grader with its own idea of
  "legal" grades the reimplementation, so tests assert the two agree exactly.
- **A block that resolves to nothing is an error**, not an empty block. A typo'd
  `refs` grades clean while nothing was checked.
- **`rules_run` / `rules_skipped` are reported**, because "0 violations" and
  "0 rules ran" must not look the same to a machine.
- **A board with no trustworthy outline is refused (exit 3), not graded.** The
  fallback is invisible: no rings ⇒ `BoardOutlineGate.active` False ⇒ every
  containment test silently degrades to the bounding box.
- **`oob_area` cannot be budgeted.** It is measured against the bbox inset, so a
  part sitting entirely inside a cutout scores `0.0`. Refused at load time with
  that reason; use `oob_count` / `oob_amount`.
- **An unknown key is refused at every level of the intent** (#710), and so is a
  `severity` key that is not a rule name. Same reason as `block_unresolved`, one
  level down: a typo'd key that is silently dropped is a constraint the author
  believes they set and the grader never checks. `context` is the one exception,
  open by design at the top level and on every entry, because provenance with
  nowhere to go ends up in a key that IS graded.
- **`schema` is the format number; `min_reader` is the field vocabulary.** An
  intent sets `min_reader` when a claim must not be silently ignored, and a
  build whose `READER_VERSION` is lower refuses the file instead of grading it
  without the claim.

One thing `--emit-intent` will not do: claim a `zone` for a sheet block. A
schematic sheet is a *functional* grouping, so its members scatter and all ten of
ulx3s's sheet bounding boxes mutually overlap (up to 4508 mm²). Zones are emitted
only where disjoint — the same spatial incoherence that makes sheet blocks
useless for movement, above.

## Lock advisor (#431)

`--suggest-locks` on either CLI reports which parts look position-critical, with
a reason each, and prints a paste-ready `--lock` list. It **never locks
anything**: a wrong auto-lock silently freezes a part that needed to move, and
the run just quietly does less.

Its strongest rule is a code fact rather than folklore. `quench.py:207` keeps
only `net_id > 0` pads, so a net-less NPTH mounting hole has `pin_count == 0`
and no airwires -- while the only skip in the part loop is `if not fp.pads`, and
a mounting hole HAS a pad. It is movable with **nothing opposing it but the halo
term**. `tigard` ships four unlocked M3 holes in exactly that state.

Geometric rules (NPTH, outline overhang, edge proximity) measure the board.
Lexical ones (footprint name, reference prefix, pin function) guess from names
and are asymmetric: near-exact on stock KiCad libraries, silent on a house
library like `interf_u:PGA120`. False negatives are common, false positives
rare, and the output says so -- treat a quiet result as "nothing detected", not
"nothing to lock". A ref matched by rules from two different evidence channels
promotes to high.

High-pin parts are an **advisory**, not a suggestion: `place_route_loop` guards
them with `--max-target-pins` but `place_optimize` does not, and "large" is not
"position-critical". Exact refs by default, never invented globs -- a `J*` you
did not inspect freezes parts you never looked at, which is the auto-lock
failure merely deferred.

```bash
python3 py_placer/place_optimize.py board.kicad_pcb --suggest-locks     --suggest-locks-json /tmp/locks.json     # writes NO board
# review the reasons, then:
python3 py_placer/place_optimize.py board.kicad_pcb out.kicad_pcb --lock H1 J1 J2 ...
# confirm you covered them -- unlocked_high must be 0:
python3 py_placer/place_optimize.py board.kicad_pcb --suggest-locks --lock H1 J1 J2 ...
```

## Reviewing a placement: render it (#431)

```bash
python3 py_tools/render_placement.py placed.kicad_pcb --before seed.kicad_pcb -o delta.png
python3 py_tools/render_placement.py board.kicad_pcb --list-groups --group-by sheet
python3 py_tools/render_placement.py board.kicad_pcb --zoom-group sheet:58d913ec --per-side -o out/
```

**The render is triage, not a verdict.** The verdict is the caption's numbers.
Do not judge a placement by how much moved -- "lots moved, looks broken" and
"barely moved, looks safe" are both wrong.

### What it renders

Every panel below is `render_placement.py` output, and every one carries a
metrics caption — the render is triage, the caption is the verdict.

**The placement diff, drawn.** `--before` turns on the dashed seed rects and the
displacement arrows, so "what moved and how far" is the picture rather than a
diff of two files. 22 parts here; `C4` travelled 95.65 mm.

```bash
python3 py_tools/render_placement.py placed.kicad_pcb --before seed.kicad_pcb -o delta.png
```

![placement delta](../docs/placement-delta.png)

**Per-side panels.** Placement lives on two sides, so `--per-side` emits one
panel per side instead of one flattened projection. Parts on the side you asked
for are bright; the far side's SMD pads are dimmed (still context — a back-side
part is why a front trace has to detour); **through-hole pads stay full
brightness on both**, because they are physically on both and a hole cannot move
on one side only.

```bash
python3 py_tools/render_placement.py board.kicad_pcb --before seed.kicad_pcb --per-side -o out/
```

| front | back |
|---|---|
| ![front](../docs/placement-side-F.png) | ![back](../docs/placement-side-B.png) |

**Zoom to one placement block.** `--zoom-group` takes the same block names as
`route.py --group` — `--list-groups` prints them with their part and net counts.

```bash
python3 py_tools/render_placement.py board.kicad_pcb --list-groups --group-by sheet
python3 py_tools/render_placement.py board.kicad_pcb --zoom-group 58d913ec --group-by sheet -o blk.png
```

![group zoom](../docs/placement-group-zoom.png)

**An unplaced board.** The placement CLIs refuse this one (exit 3); the renderer
warns and draws it anyway, framed to the PARTS rather than the outline — a pile
at the origin would otherwise render as a dot in the corner of an empty board.
13 footprints at 1 distinct position, and `overlap 792 mm²` gives it away.

![unplaced](../docs/placement-unplaced.png)

**Reference designators.** On by default, and sized to the PART rather than to
the image — scaling by image height alone gives an 8px label at any zoom:
drawn, technically present, and unreadable. A part too small to letter is
skipped, and its arrow carries the meaning instead.

![labels](../docs/placement-labels.png)

**Every toggle, on one board.** The same 13-part delta, one flag at a time:

![toggles](../docs/placement-toggles.png)

`--no-delta-first` looks unchanged there for a real reason: 12 of the 13 parts
moved, so there is no context left to dim. It matters on a board where the delta
is a genuine subset — which is also where `--ratsnest-all` earns its keep:

![ratsnest](../docs/placement-ratsnest.png)

By default only the moved and attributed nets are drawn. `--ratsnest-all` is the
deliberate hairball switch: on a dense board it reproduces exactly the
unreadable mess KiCad already shows, which is limitation #1 in the issue and the
reason delta-first is a default rather than polish.

**Just the nets you are chasing.** `--ratsnest-nets` takes the same globs as
`route.py --nets`, exclusions included, and draws only those airwires — in their
own colour, with the parts they run between labelled and un-dimmed. Between "the
nets that moved" and "every net on the board" this is the case that actually
comes up.

```bash
python3 py_tools/render_placement.py board.kicad_pcb --before seed.kicad_pcb     --ratsnest-nets '/CLK*' '/DATA*' '!*_N' -o clk.png
```

![named nets](../docs/placement-ratsnest-nets.png)

The matching goes through `net_queries.matches_net_filter` — the same helper
`route.py --nets` and the plan executor use — so a pattern means here exactly
what it means there. A pattern that matches nothing says so on stderr rather
than rendering an empty ratsnest, which would read as "this net has no airwires".

`--focus` adds one cropped panel per failed-net cluster.

### The movie

`make_movie.py WORKDIR --camera auto` animates a `place_route_loop` work dir:
an establishing overview, a zoom to the parts each round moved, and — when the
work moves to the other face — the board **turns over**, 180° about the vertical
axis, with every frame after that mirrored, because you are now looking at the
back.

13 components, 4 on the back, routed. `--tween 0` cuts to each new placement
instead of gliding.

![placement movie](../docs/placement-movie.gif)

One rule makes the sequencing legible: **a frame either moves the camera or
changes the board, never both.** Every transition happens over a frozen board and
is followed by a settle beat, so the moves only play once the camera has arrived.

## Module layout

| File | Purpose |
|------|---------|
| `quench.py` | The optimizer: cost terms, move generation, greedy quench |
| `portfolio.py` | K diverse candidates: perturbation strategies, gates, ranking, diversity selection |
| `seeder.py` | Intent-driven initial placement for unplaced boards, and `(locked yes)` stamping |
| `../group_routing.py` | Block → net scoping, and the undo, for `route.py` (#459) |
| `fanout_clearance.py` | Post-fanout decoupling-cap clearance repair (#130) |
| `groups.py` | Placement blocks: which parts move as one rigid body (#459) |
| `diagnosis.py` | Which parts to offer the quench, from routing evidence rather than pin count (#553). Ranks; never combines |
| `relocate.py` | Bounded block relocation as a constraint solve (#554): the relative-order graph, and how far a block travels when its neighbours yield |
| `routability.py` | Block displacement, corridors, escape lanes, net affinity — the geometry behind `floorplan.grade`'s health block |
| `reconstruct.py` | The structural ("puzzle") solver: tier classification, pattern fit, rigid vectors, the assignment ILP, and the minimal-move legalize sweep |
| `reseat.py` | Hungarian re-assignment of a proximity-tethered cluster onto rings around its anchor. NOT `seeder.reseat_scope`, which is a different mechanism for a different problem |
| `reachability.py` | Whether a pose is reachable at all, for the repair ladders |
| `labels.py` | Reference-designator silkscreen geometry (`beautify_labels`, #481) |
| `perturb.py` | Manufactures known-bad seeds by displacing a block (#411). Damage, never repair |
| `recovery.py` | Grades a repaired placement against the original it was damaged from (#411) |
| `floorplan.py` | Floorplan intent: declare where things are supposed to go, then grade it (#549). A checker; it authors no geometry |
| `escape.py` | Per-face escape-lane ledger: can the nets on this part's face get OUT? |
| `pair_order.py` | Pin-order inversions between connected part pairs. A LOWER BOUND, not a proxy |
| `part_class.py` | What determines a part's position — a pose-INDEPENDENT class |
| `options.py` | When a board cannot hold its parts, say so WITH THE NUMBER and the options |
| `provenance.py` | Was every pose in this board produced by a registered engine lever? |
| `lock_advisor.py` | Which parts should not be moved, and why (#431). Advice only |
| `placement_state.py` | Board-state gates: unplaced, and already-routed (#431) |
| `cli_gates.py` | argparse shared by both placement CLIs so they cannot drift |
| `../render_placement.py` | Headless PNG stills of placement status (#431) |
| `legality.py` | Hard constraints shared by both engines: board side, real Edge.Cuts containment, and the OO/OoB graders (#456) |
| `parser.py` | Courtyard boundary and locked-footprint extraction |
| `writer.py` | Writes new positions/rotations (rotates pad angles with the footprint, as KiCad stores pad angle = footprint + pad rotation) |
| `utility.py` | Shared utilities (bbox from pads, grid snapping) |

## Legality model (`legality.py`, #456)

What counts as a *legal* placement is decided in one place, so the optimizer and
any grader cannot disagree:

- **Board side.** A part occupies its own side with its full courtyard, and the
  opposite side only with the bounding box of its **drilled** pads. So a
  back-side decoupling cap may sit under a front-side BGA (they overlap in XY,
  not in copper), but not inside a front-side connector's pin field. Cross-side
  pairs also pay no halo penalty — spreading them apart buys no routing room.
  On a single-sided board every test reduces to plain courtyard-vs-courtyard.
- **Board containment** measures against the real Edge.Cuts rings, not an inset
  of the axis-aligned `board_bounds`, so parts are not nudged into an L-shaped
  board's notch or an interior cutout. Three levels of short-circuit keep it
  cheap: the gate self-disables when the outline *is* its bounding box (or when
  the parser found no usable ring, where the bbox is all there is), then a cached
  per-part reachable-disk prune, then the exact ring test.
- **Off-board seeds are not frozen.** Only candidate poses are validated, never
  the incumbent, so a part sitting outside the board had every alternative
  rejected and could never move — not even toward the board. A part whose only
  violation is board containment may now take a pose that moves it strictly back
  toward the board without overlapping anything.

  This is deliberately limited to the board term. An *overlapping* part keeps the
  original rule (it may move only to a fully legal pose), because the violation
  measure is a distance while the thing at stake is an area: trading one deep
  narrow overlap for a shallow wide one lowers the distance and raises the area.
  Measured on `watchy`, where 81 of 82 parts start in violation — its hand
  placement is tighter than the 0.25 mm courtyard clearance quench asks for — a
  permissive rule took total courtyard overlap from 9.1 mm² to 37.9 mm²
  (strict-decrease: 16.8) where the board-only rule gets 0.23 while also walking
  10 of the 13 off-board parts back on.
- **Graders.** `placement_overlap_area` (OO, mm²) and `placement_out_of_board`
  (OoB) report the same geometry the optimizer gates on;
  `QuenchState.legality_metrics()` returns both for a live placement. All zero
  means fully legal. Intended for the placement scorecard in #411/#110.

**Cost on two-sided boards.** Side-awareness removes a large number of false
collisions, so many candidate poses that used to be rejected outright now reach
the cost function. On `glasgow_revC` (172 front / 92 back parts) a bounded 40-part
pass makes **3.4× more airwire cost evaluations** than before (9.3k → 31.4k
`_count_crossings_np` calls) and takes correspondingly longer. Nothing per-call
got slower — the optimizer is searching the space it was previously, wrongly,
skipping. Small boards are unaffected or faster (`watchy`: 69 s against 109 s
before).

Courtyard extraction (`parser.py`) reads `fp_line`/`fp_rect`/`fp_arc`/
`fp_circle`/`fp_poly` per side. A footprint with no courtyard on any layer falls
back to its pad bounding box — which carries no courtyard margin at all, so the
part is modelled smaller than it is; that fallback now warns and names the refs.

Note: an earlier from-scratch constructive placer (`place.py` +
`rust_placer/`) was removed after experiments showed hand placements beat it
by ~500× in router effort; see git history and
docs/placement-optimization.md for details.

## Pad+drill legality layer, repair mode, and the reconstruct solver

Added after two evaluation runs on the #411 swap corpus measured the failure
modes directly (parts walked onto a locked connector's pad, parts left
off-board, parts on NPTH holes, 48/92-part churn, `place_seed --force`
re-seating 85/92 while leaving its zone targets unmoved):

- **`legality.PartPads` / `LegalityContext` / `grade_pad_legality`** — the
  pad+drill layer. Gate currency: rotation-inflated AABB pad rects (the
  `_Cap.pad_rects` pattern; conservative — can falsely reject, never falsely
  accept), NPTH drills held off foreign copper at a STANDOFF FROM THE HOLE
  WALL of `max(--clearance, NPTH_TO_TRACK_CLEARANCE, the board's declared
  min_hole_clearance, the hole pad's own local_clearance)` — `check_drc`'s own
  requirement (#730, #761). Resolved in ONE place, `PartPads.hole_keepouts`:
  the stored `holes_local` radius is the growth ABOVE `--clearance` and the
  consumer adds the clearance back, exactly as `fanout_clearance`'s cap gate
  and `labels.py`'s silk test each do — before #761 legality added nothing, so
  its modelled standoff collapsed to zero at and above the requirement.
  `copper_holes=False` opts a SILK caller out of both copper terms (the pad
  override and the board floor); the board floor arrives as a resolved FLOAT
  (`resolve_npth_floor`), never a board pointer, and only at the two of six
  `build_part_pads` call sites that read hole keep-outs. `holes_extent`
  carries the same holes WITHOUT either copper term, so an author's keep-clear
  can never push a part off the outline. Per-PAIR
  baselines from SEED poses ("never worse than the board you were handed"; a
  NEW different-net pad intersection is never admitted). Exact `check_drc`
  geometry runs once per CLI for reports, so summaries carry no AABB phantoms.
- **Quench integration (default ON)** — `candidate_valid`, the off-board
  unfreeze branch and the swap phase all carry the pad gate; the halo term no
  longer saturates on overlap (existing overlaps now have a repair gradient);
  zero-net parts (mounting holes) are frozen by the quench unless
  `--move-unconnected`; zero-pad footprints with a courtyard are static
  obstacles; `--min-gain-per-mm` (default 0.1) is a displacement-scaled
  acceptance threshold. `--courtyard-only` restores the old model bit-for-bit.
- **`place_seed --repair`** — violation-driven minimal-move repair: only
  violators move, worst first, escalating caps (0.5/1/2/5 mm), file-locked
  non-must_lock violators are reported, never moved. Zones smaller than a
  part's courtyard grade (and seat) on the anchor point — the spec-coordinate
  pattern is satisfiable by construction now.
- **`place_reconstruct.py`** (`placement/reconstruct.py`) — the structural
  ("puzzle") solver: tier classification (frame -> anchors -> smalls),
  corner-inset pattern fit (propose-only), rigid ±v vector detection, ONE
  simultaneous candidate assignment as an Assignment-Problem-with-Conflicts
  ILP (`scipy.optimize.milp`/HiGHS; breakout-weighted descent fallback), and
  a minimal-move legalize sweep. Every stage is gated on the lexicographic
  tuple (pad conflicts, hole shortfall, pad off-board, overlap, hpwl), so the
  count gate cannot be satisfied by pushing parts off the board. Acceptance
  measured on the swap corpus: bare-board PAD-PAD 68 -> 0 in one solve with
  zero evacuation; on the correct control board it proposes nothing and moves
  0 parts. Zero-net pattern parts (two M3 holes) carry no net-anchor cost, so
  slot assignments used to be exactly degenerate and the solver picked
  arbitrarily — run 3 shipped the two repaired holes CROSSED, ~40 mm from
  home each, "mechanically equivalent" and recovery-visible (worth ~0.16 of
  recovery on that board). Run-4 F1 deliberately REVERSES the earlier
  position that this was acceptable: a scale-free nearest-slot tiebreak
  (`DIST_TIEBREAK_PER_MM`) now makes each pattern part take the slot nearest
  its current pose — equivalent to the board is not equivalent to recovery,
  and nearest is the minimal-perturbation choice. The assign stage also
  requires ≥2 distinct supporting refs per rigid vector (R4's own "two or
  more agree" letter) and runs a per-part revert sweep (`prune_assignment`)
  after acceptance, because the board-wide gate tuple cannot see an
  individual mis-move smuggled inside a hugely-improving set (run 3's J7:
  31.6 mm from home, worse than its 15.8 mm input).
- **`render_placement --legality`** (default ON) draws the defects the caption
  used to only count: conflict rings/links, NPTH keepout circles (the real
  keep-out since #761 — it drew the bare drill at `--clearance >= 0.20`,
  agreeing with the model's own blind spot), dashed-red off-board pad
  extents; caption gains `pad-conflicts` / `hole-conflict`.

Design lineage (see the session literature survey): Abacus/minimum-
perturbation legalization (Spindler 2008; Brenner 2012; Kahng-Markov-Reda
2004), conflict-directed repair scoping (FLOORIST, Moffitt 2006), assignment
with conflicts (Oncan 2019), breakout weighting (Morris 1993), frame-first +
largest-margin-first ordering (Wolfson 1988; Paikin & Tal 2015; regret-k,
Ropke & Pisinger 2006).
