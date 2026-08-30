# What predicts routed `blocking` — the first measurement (#703)

**Status: COMPLETE. The declared table is 6 boards x K=20 = 120 placements
planned, of which 119 produced a routed result** — which is what this file's
own body says at "119 of 120 variants produced a routed result", and which this
line used to contradict. No predictor here is "validated" in a strong sense; the
acceptance rule's own false-positive rate at N=6 is **1.5%-5.5%** (median 2.5%)
over the 21 predictors defined on all six boards, measured, and that number
belongs beside every PASSES below.

Every correlation number this repo quoted before #703 was measured against
*distance-to-the-correct-placement* or against *the gap a human left*.
`r(crossings) = +0.780` is 29 candidates on ONE board against distance-to-truth;
the corridor law's `r = +0.41 … +0.90` is against the human's gap. CLAUDE.md's
*"What a placement run is FOR"* says the headline is routed `blocking`, and no
predictor had ever been correlated with it.

This is that measurement. Nothing in it is a proxy for a proxy: 120 placements
were generated, each routed once with an argv frozen before any variant existed,
and each graded by `board_score`.

> **THE ROWS ARE NOT COMMITTED, and that is a deliberate trade.** They were,
> at 276 KB, so this table could not drift from them. drandyhaas asked for the
> file to go on review: the results are regenerable, and he verified that rather
> than trusting the contract — `--task esp_prog:authored` rebuilt a row in 4.4 s
> with every predictor, `truth` and `poses_sha256` matching.
>
> **What that costs, said plainly: no number in this document has an automated
> change detector any more** — not the rho table, not the per-board tables, not
> the shuffle-control rates — **and re-deriving them is a 60-hour job** (the sum of `provenance.total_seconds`
> over the 119 rows that carry one; median 343 s, max 12058 s on
> `kit-dev-coldfire-xilinx_5213:portfolio-5` — 3.3 h in one row, a floor no
> amount of parallelism beats). The `-j 4` wall clock has not been measured; a
> naive divide puts it near 15 h. It is the recorded finding.
>
> An earlier revision of this box said 8.8 h over 87 rows, median 217 s, max
> 2012 s on `tigard:perturb-wrong_side`. Those are the numbers of the rows file
> this document used to carry, `tests/stress/predictor_rows.json`, dropped in
> `acad9c0c` and still recoverable:
>
> ```bash
> git show acad9c0c^:tests/stress/predictor_rows.json
> ```
>
> 87 rows over **eight** `board_key`s — four swept at K≈20 (esp_prog 21,
> tigard 21, splitflap_driver 20, watchy 20) plus four singletons (smartknob 2,
> neo6502, piantor, urchin) — summing to 8.77 h over the **77** rows that carry
> a `total_seconds`, with a median of 217 s when the 10 that do not are counted
> as zero, and a max of 2012.2 s on `tigard:perturb-wrong_side`. Every figure
> was correct for that file, and all four were carried forward unchanged when
> the study was re-run at six boards x K=20.
>
> ```bash
> # rebuild the rows (60 h serial; ~15 h at -j 4, and one row alone is 3.3 h)
> python3 -X utf8 tests/stress/predictor_study.py --out wk/703 -j 4
> # then every statistic, free, from the rows it wrote
> python3 -X utf8 tests/stress/predictor_study.py --from-rows wk/703/rows.jsonl
> python3 -X utf8 tests/stress/predictor_study.py --from-rows wk/703/rows.jsonl --shuffle-control 200
> ```
>
> What IS guarded is the **rig**: `tests/test_703_predictor_regen.py`
> regenerates four cheap declared variants (~51 s) and diffs their predictors
> and routed truth against literals, so a change to the placement engine, the
> predictor extraction or the route argv cannot silently move what this study
> measured. `predictor_study.py --verify-row esp_prog:authored` is the same
> check for one row.

*(This file is deliberately NOT in `test_431_skill_commands.py`'s `SOURCES`. Its
commands invoke `tests/stress/*.py`, which `krt_capabilities._tool_path` cannot
resolve — it searches only `''`, `py_router`, `py_tools`, `py_placer` — so
adding it would turn a green gate red for a reason unrelated to skills.)*

---

## The headline

**The predictors that rank routed `blocking` are the LEGALITY counts. The
classical placement proxies — `crossings`, `hpwl`, `halo`, `overlap_area` — do
not.**

| predictor | boards right / wrong | median rho | verdict |
|---|---|---|---|
| `pad_shortfall` | 6 / 0 | +0.786 | **passes** |
| `pad_intersection_pairs` | 6 / 0 | +0.785 | **passes** |
| `pad_overlap_pairs` | 6 / 0 | +0.785 | **passes** |
| `body_overlap_pairs` | 6 / 0 | +0.785 | **passes** |
| `pad_conflict_pairs` | 6 / 0 | +0.785 | **passes** |
| `pad_clearance_pairs` | 6 / 0 | +0.785 | **passes** |
| `courtyard_blocking_pairs` | 6 / 0 | +0.684 | **passes** |
| `pad_copper` (off-outline pad copper) | 6 / 0 | +0.524 | **passes** |
| `hole_shortfall` | 4 / 0 | +0.505 | **passes** |
| `hole_conflicts` | 4 / 0 | +0.501 | **passes** |
| `overlap_area` | 5 / 1 | +0.652 | fails |
| `total` | 5 / 1 | +0.599 | fails |
| `halo` | 5 / 1 | +0.585 | fails |
| `courtyard_overlap_mm2` | 5 / 1 | +0.548 | fails |
| `courtyard_advisory_pairs` | 5 / 1 | +0.526 | fails |
| **`crossings`** | **5 / 1** | **+0.515** | **fails** |
| `oob_count` | 5 / 1 | +0.294 | fails |
| `courtyard_off_outline` | 5 / 1 | +0.294 | fails |
| `oob_amount` | 5 / 1 | +0.268 | fails |
| `oob_area` | 4 / 2 | +0.259 | fails |
| `length` | 4 / 2 | +0.131 | fails |
| **`hpwl`** | **4 / 2** | **+0.059** | **fails** |
| `edge` | 3 / 3 | +0.038 | fails |
| `cross_side_stacks` | 2 / 1 | +0.030 | fails |
| `align`, `corridor_cut`, `orient`, `locked_contact_pairs` | 0 / 0 | - | **no verdict** (constant everywhere) |

The ten that pass do so at a two-sided p of **0.031**, the floor for a 6-board
sign test.

The rule is `test_placement_ab.gate()`'s, transposed from marks to signs: right
direction on ≥ N−1 boards, wrong direction on none, over the boards that
produced a *defined* ρ. Spearman is computed **within each board** and never
pooled.

### Three things this says that the repo did not know

**1. The off-outline channel ranks `blocking` on 6 of 6 boards, and the number
that REFUSES is a different census of it.** `pad_copper` ranks positively on
every board: +0.481 esp_prog, +0.392 kit-dev-coldfire, +0.649 sonde_u, +0.568
splitflap, +0.473 tigard, +0.622 watchy. Two sentences this document printed
beside that finding were wrong, and #788 corrects them:

- **The L2 gate does not read this key.** `loop_driver.py`'s refusal reads
  `check_assembly.py`'s `oob_pad_count` — a part-level pad AABB against an
  outline inflated by the grading clearance, so it moves when `--clearance`
  moves — while `checklist.a_off_outline.pad_copper` is `render_placement.py`'s
  per-PAD, margin-0 census, which reads no `--intent`. `legality.py`'s own
  `oob_pad_basis` string has recorded the distinction all along. The skill
  guidance had the same conflation and is corrected with it.
- **It is not the only pre-route number here that refuses.** L2 also refuses on
  `buildable`/`verdict`, on `locked_contacts`
  and on `blocking` — and `blocking` belongs to the +0.785 family, because
  `check_assembly`'s `blocking` counts pad-INTERSECTION pairs only, which is
  this table's `pad_intersection_pairs`. So the strongest member of the family
  was already gating before anyone measured it; what the +0.524 member adds is
  a different failure, not a weaker version of the same one.

How close the two censuses actually are, and what the answer depends on, is
measured below.

**2. `hpwl` -- which the drivers DO gate on -- barely relates to the routed
outcome.** 4 boards right, 2 wrong, median **+0.059**, two-sided p = 0.69:

| board | rho(hpwl, blocking) |
|---|---|
| esp_prog | **-0.525** [LOO -0.648..-0.443, K=20] |
| tigard | **-0.185** [LOO -0.353..-0.045, K=20] |
| sonde_u | +0.040 [LOO -0.168..+0.345, K=17] |
| watchy | +0.079 [LOO -0.080..+0.360, K=14] |
| splitflap_driver | +0.429 [LOO +0.352..+0.739, K=20] |
| kit-dev-coldfire | +0.608 [LOO +0.538..+0.723, K=19] |

Its median moved +0.158 -> +0.040 -> +0.059 as the 4th, 5th and 6th boards
landed, and it is NEGATIVE on two of six. The reasoning behind gating on it
(*"hpwl's minimum is at the truth, so it is the one that can carry a gate"*) is
about distance-to-truth and remains untouched. What is new is that its
relationship to the routed outcome is close to a coin flip across boards.
**This does not license removing that gate**; it means the gate rests on the
distance argument alone, and nobody should describe it as a routability gate.

**3. `crossings` fails, and one board's sign is opposite.** rho = **-0.179** on
esp_prog against +0.716 / +0.017 / +0.636 / +0.705 / +0.394 elsewhere. More
crossings went with *less* blocking on one of six boards, and sonde_u is
indistinguishable from zero. Its median of +0.515 is the highest of any failing
predictor -- it is not useless, it is inconsistent, and the sign rule is about
consistency.

## The circularity control changed the answer for `crossings`

The realistic-end sampler is `portfolio.generate`, whose quench **minimises
crossings and hpwl**. Those rows carry `generator: portfolio_quench`, and every
statistic is computed twice:

| predictor | quench rows INCLUDED | quench rows EXCLUDED |
|---|---|---|
| `pad_copper` | 6/0, +0.524, **passes** | 6/0, +0.534, **passes** |
| `pad_clearance_pairs` | 6/0, +0.785, **passes** | 6/0, +0.918, **passes** |
| `courtyard_blocking_pairs` | 6/0, +0.684, **passes** | 6/0, +0.913, **passes** |
| **`crossings`** | **5/1, +0.515, fails** | **6/0, +0.525, passes** |
| `halo` | 5/1, +0.585, fails | 6/0, +0.906, passes |
| `overlap_area` | 5/1, +0.652, fails | 6/0, +0.975, passes |
| `hpwl` | 4/2, +0.059, fails | 4/2, +0.090, fails |

**Whether `crossings` passes depends on whether the sample includes placements
made by an optimizer that minimises crossings.** That is the circle #703 exists
to break, and it is why neither arm is reported as *the* answer for it. The
legality predictors pass in **both** arms, which is what makes them the finding.

---

## How much to believe a PASSES

Permuting the truth **within each board** and re-running the whole aggregation
200 times (`--shuffle-control 200`) measures the acceptance rule's own
false-positive rate:

```
cross_side_stacks             30.0%   <- defined on 3 boards, the minimum
hole_conflicts                 8.5%   <- defined on 4
hole_shortfall                 8.0%   <- defined on 4
edge                           5.5%
courtyard_advisory_pairs       4.5%
courtyard_overlap_mm2          4.5%
halo                           4.0%
```

Full sweep over the 21 predictors defined on all six boards: **min 1.5%, max
5.5%, median 2.5%**. The rows above that band are the predictors defined on
fewer boards, which are judged by an easier rule -- and `cross_side_stacks` at
**30%** on three boards is the clearest possible statement of why the number of
boards is the whole game.

Two consequences, both load-bearing:

- **A predictor defined on too few boards clears the rule by luck.** Before the
  fix below, `cross_side_stacks` -- constant on three of six boards, so defined
  on three -- passed **100%** of these shuffles when it was defined on ONE,
  because `consistent >= max(1, N-1)` is `1 >= 1` at N=1.
  `rank_stats.MIN_SIGN_BOARDS` is now 3, the same value and the same reason as
  `test_placement_ab.MIN_TRIAL_BOARDS`. It is defined on three boards here and
  still shuffles at **30%**, which is the honest reading: a three-board verdict
  is barely a verdict, and the doc reports it as failing rather than as a
  finding.
- **At N=6 the rule's empirical false-positive rate is 1.5-5.5%** (median 2.5%),
  against 4.5-8.0% at N=5 and 9-18% at N=4. Each added board roughly halved it,
  which is the concrete value of running the declared table rather than stopping
  early -- and it is why the sixth board was worth its fourteen hours. It is
  still not the 0.031 the two-sided p-value suggests. Ten predictors passing is
  therefore *evidence*, not proof; the six that share a median of +0.785 are
  plainly one quantity seen through six counters, so they are not ten
  independent findings.

The control permutes truth over the same deduplicated sample the ρ values use.
It did not at first — watchy entered the null with its six duplicate placements
still in, at K=19 against the K=13 its ρ was computed on — and correcting that
moved individual rates by up to 5.5 points in both directions.

---

## The boards and what was actually run

| board | K ranked | classification | `blocking` values observed | notes |
|---|---|---|---|---|
| esp_prog | 20 | measurable | 0, 2, 3, 6, 14, 32, 466 | - |
| splitflap_driver | 20 | measurable | 0, 12, 29, 47, 55, 65, 5424 | - |
| tigard | 20 | measurable | 0, 1, 2, 64, 71, 73, 114, 158, 13078 | - |
| kit-dev-coldfire-xilinx_5213 | 19 | measurable | 1, 2, 3, 6, 7, 8, 9, 114, 671 | `perturb-pile` timed out at 14400 s |
| sonde_u | 17 | measurable | 0, 2, 21, 23, 877 | 3 duplicate placements collapsed |
| watchy | 14 | measurable | 1, 3, 5, 44, 94, 8521 | 6 duplicate placements collapsed |

**119 of 120 variants produced a routed result.** The single exclusion is
kit-dev-coldfire's `perturb-pile`, which did not finish inside a four-hour
route budget. That is the hardest route in the study by construction -- 160
parts collapsed onto one coordinate on a four-layer board -- and the pile route
time across the boards that did finish scales steeply with board size: 183 s
(esp_prog), 739 s (sonde_u), 2288 s (splitflap), 3106 s (watchy), 9877 s
(tigard). kit-dev's effective K is 19, far above the floor.

An earlier revision of this document reported `perturb-pile` timing out on 3 of
4 boards at a 2400 s budget. Those were near-misses rather than impossibilities
(splitflap needed 2798 s), and raising a *timeout* is not a protocol change for
rows that already finished -- a route that completed in 300 s completes
identically at any larger budget -- so it strictly adds samples. Only the killed
rows were re-run.

Every board is git-tracked, so every row is regenerable from this repo.
`portfolio.generate(..., only=i)` is byte-identical by contract, the route argv
is frozen per board in `ARGV.json`, and each row carries its
`poses_sha256`, `input_board_sha` and `argv_sha`.

**What is NOT in this run, said plainly:**

- **One variant of 120 is missing** (kit-dev-coldfire `perturb-pile`, above).
  Every p-value and every N is over the boards that produced a *defined* rho,
  never over a planned count; `predictor_study.py` prints that denominator on
  the same line as the p-value for exactly this reason.
- **The duplicate guard cost sonde_u 3 slots and watchy 6.** Their `translate`
  and `scatter` blocks have little feasible travel on those outlines, so some
  variants reproduced the authored board exactly. Every drop is named in the
  report.
- **Six boards is six boards.** The sign test's floor at N=6 is p = 0.031, and
  the shuffle control says the rule's real false-positive rate here is 1.5-5.5%.
  Ten predictors clearing that is evidence, not proof, and the six sharing a
  median of +0.785 are one quantity seen through six counters.

## What #788 asked of these numbers

#788 read the table above and asked why the family that ranks `blocking` best
is advisory while the weakest passing member is the only one that refuses. Two
measurements answer it. Both regenerate with

```bash
python3 -X utf8 tests/measure_788_censuses.py -j 8
```

and `tests/test_788_marginal_literals.py` is the clean-clone change detector
for a declared subset of the same rows.

### The two censuses agree, but not unconditionally — the floor decides

The study measured its predictors on the PRE-ROUTE variant board;
`check_assembly` runs on a placed board. Copper is inert to
`grade_pad_legality`, which reads footprints, courtyards and the outline, so
anything that moves between the two arms is the clearance floor and nothing
else. That is measured, not assumed — `measure_788_censuses.py --inertness`
grades both boards at a PINNED clearance and requires all five keys to agree:
**60 variants across esp_prog, watchy and tigard, 0 disagreements**, including
`perturb-pile` at 109 grazes / 112 courtyard pairs / 88 blocking pairs. (An
earlier version of this claim rested on three esp_prog variants, two of which
are all-zero on every key — a sample that cannot tell inertness from
cleanliness.) And the floor does move, because the study's variant writer
produced
no sibling `.kicad_pro` (the #441 hazard, live in this dataset): the pre-route
boards all grade at the `fixed default` 0.25, the routed boards at each board's
own netclass, 0.1 to 0.2 mm.

| study predictor vs `check_assembly` key | at each board's own floor | at the 0.25 fallback |
|---|---|---|
| `pad_copper` vs `oob_pad_count` | **119/119** sign, 118/119 exact | **112/120** sign, 104/120 exact |
| `pad_clearance_pairs` vs `pad_conflicts` | 118/119 sign, 109/119 exact | 119/120 sign, 114/120 exact |
| `hole_conflicts` vs `hole_conflicts` | 118/119 sign, 116/119 exact | 120/120 exact |
| `courtyard_blocking_pairs` vs `courtyard_blocking` | 119/119 exact | 120/120 exact |
| `pad_intersection_pairs` vs `blocking` | 119/119 sign, 113/119 exact | 120/120 sign, 113/120 exact |

The off-outline row is the one that moves, and it moves in one direction:
every one of the 8 sign disagreements is `check_assembly` reporting a breach
the per-pad measure does not.

**Those 8 rows are 2 distinct placements**, and the difference matters. Seven
are watchy — `authored` plus six perturbations that did not move it, all
sharing one `poses_sha256` — and the eighth is `sonde_u:portfolio-2`. The study
drops such duplicates before computing anything (`9 DUPLICATE placement(s)
dropped`); the table above counts rows, so it does not. On the 111 distinct
placements the same measurement reads **108/110**, not 112/120. Neither number
is wrong and they are not interchangeable, so both are stated.

The watchy placement is worth naming because it is the human shipping board: at
0.25 the gate's census says four parts carry off-board pad copper (SW1..SW4,
0.1696 mm each) and the per-pad census says none. That is the
AABB-vs-inflated-outline basis behaving exactly as `legality.py` documents it.

**So "the two are the same quantity" is not safe as an unconditional claim.**
The #703 validation of the off-outline channel transfers to the shipped gate at
the floor a routed board carries, and degrades at a looser one. Neither arm is
the deployment condition, because a real placed board does carry a project;
what the study never pinned is the floor, and this table is the size of that
gap rather than a reason to prefer one number.

### Conditioned on the refusals that already run, the ungated half earns nothing

The question #788 poses for any gate proposal — how many placements would it
refuse, and how many of those routed worse than the ones it passed — answered
on the same rows, at each board's own floor:

```
L2 as it ships (blocking > 0 or oob_pad_count > 0)   refuses 35 of 119
    the 84 it passes routed to blocking   min 0 / median 1 / max 9
+ (pad_conflicts > 0 or hole_conflicts > 0)          refuses 37
    the 2 it adds routed to blocking      0 and 2
```

23 of the 25 rows carrying a clearance graze and 7 of the 7 carrying a hole
conflict are refused already. There is no tail to catch either: the worst row
that gets past the four checks routed to 9.

**Be precise about the two it would add, because the first draft of this
section was not.** They routed to 0 and 2 against a passed distribution of
`{0: 41, 1: 14, 2: 8, 3: 10, 6: 3, 7: 2, 8: 1, 9: 5}`. One is as good as
anything the gate passes; the other is worse than 55 of the 84 rows it passes
and better than 21. So they are not "both false refusals" — that is a claim
this data does not make. What they are is *inside* the range the gate already
accepts, and nowhere near what it exists for: every one of the 23 rows
`blocking` refuses on its own routed to 12 or worse.

**The channel is not weak.** `pad_conflicts` is one of five pair counts tied at
the top of the table above (+0.785; only `pad_shortfall`, at +0.786, is
higher). **It is REDUNDANT with what already refuses.** `hole_conflicts` is a
weaker case again — +0.501 on four boards, and the study's second-worst
shuffle-control rate — and it contributes no marginal refusal at all.

The same measurement rules out `courtyard_blocking_pairs` as an absolute census
far more sharply: it would add 21 rows whose routed `blocking` is
`{0: x6, 1: x12, 2: x3}`, 18 of them at or below the median of the rows the
gate passes. That is the reason `check_assembly` narrows it to the
moved-vs-baseline subset.

**Row counts, not board counts.** The study is six boards; 119 is its graded
placements, of which 110 are distinct. On distinct placements the picture is
the same where it matters — the same two marginal rows, routing to the same 0
and 2 — while the passed median falls from 1 to 0 and courtyard's marginal 21
becomes 15. On the 0.25 arm the absolute counts change again (L2 refuses 43,
passed median 0) and the marginal result still does not. That invariance across
all four arms is the finding; the absolute counts are not.

### Two things this measurement says about the gate that already ships

- **The two conjuncts are complementary, not a strong one and a weak one.**
  `blocking > 0` alone refuses 23, and every one of those routed to `blocking`
  12 or worse. `oob_pad_count > 0` alone refuses 24; 12 of those are refusals
  the first conjunct does not make. The worst board in the study
  (`tigard:perturb-pile`, routed `blocking` 13078) has `oob_pad_count` **0** and
  `blocking` 2783 — the catastrophe is caught by the pad-intersection conjunct,
  not the off-outline one, and each conjunct catches boards the other passes.
- **The off-outline conjunct's own marginal refusals sit where a graze gate's
  would.** Those 12 routed to `blocking`
  `[0, 1, 1, 2, 3, 3, 3, 3, 3, 5, 5, 5]`, against a passed median of 1.
  `check_assembly.py` predicted this in source — the count "moves with
  `--clearance`, so a clearance-band graze reads as copper in the air". This is
  an argument against adding a second marginal channel, **not** an argument for
  removing this one: its rationale is a mechanism (a part whose pads lie off the
  board carries nets no router can reach, and it produces no blocking pair
  because there is nothing out there to collide with) and it ranks `blocking`
  on 6 of 6 boards. Removing a gate is its own decision and would need its own
  measurement.

### What would reverse the answer

Not "the corpus never exercises per-pair clearance" — it does.
`pad_clearance_required`, the #697 disclosure of *conflicting* pairs whose
requirement exceeds the board-wide floor (`legality.py` appends to it only
inside `if pair_hit:`, so a raised requirement with adequate spacing never
appears there), is non-empty on 11 of 119 rows at the routed floor and 4 of
120 at 0.25, from two distinct causes: a 1.016 mm **pad override** on four
esp_prog rows and an **NPTH hole** requirement at 0.2 on seven watchy / tigard
/ splitflap rows. And the two marginal refusals a graze gate would add are
themselves pad-override pairs, not netclass grazes — so the case #697 built
`PadClearanceModel` for is represented, and the grazes it produced routed to
`blocking` 0 and 2.

What is NOT represented is a board where such a raised requirement is violated
on a placement that the four shipped checks pass and that then routes badly.
One of those, routed, re-opens this on evidence.

## Pre-registered decision rules

*Recorded 2026-08-28, before the sign test over the declared board set is
complete. Any later change to a rank key or a gate should cite the rule it is
discharging, not a number chosen afterwards.*

1. **`portfolio.rank_key` leads with `crossings`, and `rule1_check` bars a
   candidate on it, while `loop_driver.py` and the placement skill say in
   capitals never to gate on crossings.** Both cannot be right. **Neither is
   changed by this study**: `crossings` passes in one arm and fails in the
   other, so the evidence does not license a reorder in either direction. The
   contradiction is disclosed at both code sites and stays disclosed.
2. If a future run finds ≥ 1 board on which a rule-1 violator routed to strictly
   lower `blocking` than the baseline, the `crossings` clause of `rule1_check`
   is withdrawn — and the withdrawal keeps its row, with its measured direction,
   in the `rejected` style `test_placement_ab.py` uses.
   **DISCHARGED 2026-08-30** by the slate run below, on esp_prog and
   kit-dev-coldfire. The clause is withdrawn; the direction survives in
   `portfolio.rule1_advisory`; the row is
   `tests/placement_rule1_withdrawal.json` and its detector is
   `tests/test_789_rule1_withdrawal.py`. `rank_key` slot 1 is untouched — rule
   2 was about the bar, and the order is rule 5.
3. A predictor is reported as ranking `blocking` only on ≥ N−1 boards right and
   none wrong, over boards with a *defined* ρ, with N ≥ 3 and the shuffle-control
   rate printed beside it.
4. Saturated and starved boards are reported with their constant value and
   excluded from the denominator. They are never dropped.
5. *Recorded 2026-08-30, before any slate run has produced a tau. This is NEW
   pre-registration, not a discharge of rule 2: rule 2 is about the BAR and
   says nothing about rank order, and no tau discharges it.*

   `portfolio.rank_key` puts `crossings` in slot 1, which decides which
   candidates a probe budget is spent on. **A slate run measures Kendall tau-b
   between the static order and the routed order over the SAME candidate set**,
   the static side being `sorted(..., key=rank_key(c, 0))` with candidate 0's
   key COMPUTED rather than pinned first by fiat, and the routed side being
   `board_score`'s `blocking` VALUE with its ties left as ties. It is never
   `ranking_routed` as printed, whose final tiebreak is the static position and
   which would therefore agree with the static order by construction. tau is
   computed on `(static_position, blocking)`, so a POSITIVE tau means the key
   AGREES: three candidates at static positions 0, 1, 2 routing to `blocking`
   1, 2, 3 give three concordant pairs and tau-b +1.0.

   **The rule.** `rank_key`'s order is reported as agreeing only if tau is
   positive on >= N-1 boards and negative on none, over boards with a *defined*
   tau, with N >= 3 and the shuffle-control rate printed beside it. That is
   rule 3's shape, unchanged. **No magnitude threshold is introduced**, because
   any tau cut-off would be a number chosen after seeing tau's scale; the
   threshold IS the sign rule and its null is exactly binomial. Clearing it
   licenses *reporting agreement over the population the rank key actually
   sees*, and nothing else — not a reorder, and not rule 2. Failing it is
   reported as NOT SHOWN TO AGREE and licenses nothing. The one asymmetric
   outcome pre-registered here: tau NEGATIVE on >= N-1 boards and positive on
   none licenses opening a reorder issue — the issue, not the reorder,
   following rule 1's precedent that neither arm was licensed.

   **Declared before the run, so that a later denominator cannot be chosen:**
   the board set is #703's six; K is 11 per board (candidate 0 plus the ten
   #703 recorded); kit-dev-coldfire-xilinx_5213 runs with REDUCED guards (one
   regeneration check, no re-route control) because its quench and route cost
   hours. `splitflap_driver` is expected to come back `baseline_clean` and
   `sonde_u` probably — their #703 candidates route to `blocking` 0 and
   {0 x9, 2} — so N is expected to be 3-5 and `MIN_SIGN_BOARDS` is 3. A NO
   VERDICT at N < 3 is a first-class outcome, committed like any other.

   **Rule 2 is an EXISTENCE claim: it can be discharged, never refuted.** The
   clause standing is a default, not a finding. And the null rate of rule 2's
   own criterion is measured and printed beside its verdict — permuting
   `blocking` within each board and re-evaluating the predicate. It is expected
   to be HIGH, because roughly half a slate is crossings-barred and `blocking`
   varies. Saying that after seeing the rate would read as excuse-making;
   saying it here is the honest form. A discharge at a high null rate is still
   a discharge, and still evidence of very little.

## The slate run (#789) — what the rank key and the rule-1 bar are worth

Six boards, K=11 each: candidate 0 (the plain quench, which `portfolio.generate`
refuses to produce under `only=`, so #703 never made one) generated, routed and
graded here, plus #703's ten recorded portfolio candidates grafted in. **One new
route per board instead of eleven.** Regenerate with

```bash
python3 -X utf8 tests/stress/slate_study.py --out wk/789 -j 4
python3 -X utf8 tests/stress/slate_study.py --from-rows wk/789/slate_rows.jsonl
```

The graft is guarded rather than asserted, and every guard ran: the board's
#703 `argv_sha`, the input-board sha, each candidate's `variant_board_sha`, a
regeneration control (two candidates per board re-derived and required to match
their recorded `poses_sha256`) and a re-route control (one grafted candidate
re-routed to a fresh path and required to return the `blocking` its `score.json`
recorded). Zero boards voided. kit-dev-coldfire ran with reduced guards — one
regeneration check, no re-route control — declared before the run, because its
quench and route cost hours each.

### Rule 2: DISCHARGED, and the criterion that discharged it is weak

| board | verdict | baseline `blocking` | barred on crossings | fired | null rate |
|---|---|---|---|---|---|
| esp_prog | **fires** | 4 | 4 | 3 | 100% |
| kit-dev-coldfire | **fires** | 5 | 4 | 1 | 99% |
| tigard | does not fire | 2 | 1 | 0 | 0% |
| watchy | does not fire | 3 | 2 | 0 | 21% |
| sonde_u | cannot fire (baseline clean) | 0 | 0 | — | — |
| splitflap_driver | cannot fire (baseline clean) | 0 | 5 | — | — |

esp_prog's three firing candidates are barred on crossings **alone** — their
hpwl is below the baseline's — and routed to `blocking` 3, 3 and 2 against the
baseline's 4. The one at 2 carries the **highest crossings on the slate**, and
the best candidate the bar allows through routes to 3.

**The null rate is the number that says how much that is worth**, and it was
pre-registered as expected-high before the run: permuting `blocking` within a
board, the criterion still fires 100% of the time on esp_prog and 99% on
kit-dev, because almost every candidate on those slates routed below their
baseline. On tigard it is 0%. So the criterion is easy to satisfy exactly where
it fired, and a discharge on it is evidence of very little.

### What the withdrawal actually changes — the consequence, not the criterion

`select_best` takes the first index in `ranking_primary + ranking_static` that
is 0 or not a violator, so the answer depends on whether a probe ranking exists.
`--route-top` defaults to 2, so in a shipped run one does. Both arms:

| arm | result |
|---|---|
| static order only (`--route-top 0`) | **6 of 6 unchanged** |
| with a probe ranked by the candidates' actual routed `blocking` | esp_prog picks `blocking` **2** where the bar forced a fall-through to **3**; the other five unchanged |
| **worse on** | **0 boards, in either arm** |

The static-only result is structural rather than lucky: `rank_key`'s slot 1 *is*
crossings, so a candidate barred for having more crossings than the baseline
already sorts below every candidate with fewer, and `select_best` reaches a
non-violator first. The bar could only ever bite when the head of the static
order is itself crossings-barred. **That is what makes the withdrawal safe to
ship** — it rests on a measured consequence, not on the weak criterion alone.

### Rule 5: NOT SHOWN TO AGREE

| board | tau-b (static position vs routed `blocking`) |
|---|---|
| watchy | +0.816 [LOO +0.816..+0.816, K=4] (7 gate-rejected) |
| kit-dev-coldfire | +0.250 [LOO +0.094..+0.454, K=11] |
| esp_prog | −0.070 [LOO −0.218..+0.183, K=11] |
| sonde_u | −0.341 [LOO −0.447..−0.348, K=11] |
| tigard | −0.408 [LOO −0.816..+0.000, K=4] (7 gate-rejected) |
| splitflap_driver | n/a — saturated, every candidate routes to `blocking` 0 |

2 positive, 3 negative, N=5, two-sided p = 1.000. Rule 5 requires positive on
≥ N−1 and negative on none (or the mirror image to license a reorder *issue*).
Neither holds. **So this licenses nothing**: `rank_key` slot 1 is unchanged and
the contradiction between it and the drivers' prohibition stays disclosed at
both code sites, exactly as rule 1 left it.

Two disclosures travel with that table. splitflap_driver is saturated and out
of the denominator per rule 4. tigard and watchy report K=4 because 7 of their
11 candidates fail `score_candidate`'s baseline-relative legality gates — which
is the shipped selector's own behaviour, since `rank_static` ranks only viable
candidates, but a tau over 4 and a tau over 11 are not the same evidence.

### What this run does not claim

- **It measures `portfolio.generate`'s library-default slate, not the CLI's.**
  #703 generated its candidates with `quench_kw=None` (max_displacement 10.0,
  crossing_penalty 10.0, length_weight 1.0, halo_coef 0.25) while
  `place_portfolio.main` ships 3.0 / 30.0 / 0.3 / 0.15. Candidate 0 is generated
  the same way, so the comparison inside a board is on equal terms and rule 2 is
  discharged on its own terms — but the 4× lower crossing penalty is on exactly
  the axis the question is about, and a replication on the CLI's slate would be
  a different measurement.
- The probe arm uses each candidate's **actual routed `blocking`** as the probe
  order. A real probe ranks on windowed route failures, so that arm bounds the
  effect rather than predicting a particular run.
- Six boards is six boards, and the tau side is N=5 with two boards at K=4.

## What this does not claim

- Not that the passing predictors *cause* anything. They rank an outcome on six
  boards.
- Not that `crossings` or `hpwl` are useless. Both are measured against
  distance-to-truth, both retain the roles that measurement supports, and the
  drivers' existing prohibition on gating `crossings` is untouched.
- Not that the legality family is ten findings. Six of them share a median to
  three decimals; they are one quantity seen through six counters.
- Not a corpus-wide result. Six boards, one machine, one router build. Each
  row records `provenance.measured_git` (`v0.21.3-199-g3b8edc19`), from which
  the router version follows; there is no explicit router-version or machine
  field on a row, and this document previously claimed there was.

## What this study does NOT license for #553

`--target-select diagnosis` (`py_placer/placement/diagnosis.py`) ranks movers on
three signals, one of which is the legality family measured above. Three
boundaries, recorded before anyone reads the flag's output as a result:

1. **The extrapolation is untested.** This study measured the legality counts as
   BOARD-LEVEL scalars ranking a BOARD-LEVEL outcome across many placements.
   #553 uses them as PER-CANDIDATE counts ranking candidates WITHIN one board.
   Nobody has measured that, and the module docstring says so where a reader of
   the code will see it.
2. **`foreign_crossings` was NOT rejected by this study.** It is absent from
   #553 for three reasons: no caller has the declared corridor it needs;
   auto-deriving one manufactures the number (`CORRIDOR_MIN_COVER` exists
   because a cluster fit returns a confident rectangle for a bus that is not
   there); and its nearest measured relative, `crossings`, fails the sign test
   here. That third reason is family-level doubt. `foreign_crossings` is a
   corridor-pierce count — a different quantity — and it has never been measured
   against anything. Recording the omission as "#703 measured it and it failed"
   would be false in a way that survives.
3. **Recall is not efficacy — and the recall study came back NULL.**
   `tests/stress/diagnosis_recall.py` displaces a known block and asks whether
   the ranking concentrates on it. Nothing routes. Its evidence arms' median
   DELTAS — lift on the damaged board minus the lift the SAME ranking scores
   on the UNDAMAGED one — are `swap` **+0.135** (4 cells up, 2 down) and
   `wrong_side` **+0.000** (2 up, 3 down). That is not a finding. The study is
   kept, and kept committed, as a recorded negative rather than deleted.

   An earlier reading of the same data said "above chance on 4 of 4 boards,
   median lift 2.32, negative control at chance". It was wrong in three ways,
   all found by re-measuring rather than by re-reading, and all three are worth
   carrying because each is a trap the next study of this shape will meet:

   - **No undamaged control.** `perturb.pick_block` ranks units by source and
     size, and so, largely, does this ranking — so on tigard the damaged block
     is already ranked #1 by displacement on the PRISTINE board. The raw lift
     could not tell "found the damage" from "always ranks that block first".
   - **Lift saturates.** Its ceiling is `movable / n`, reached whenever the
     selection contains the whole truth, and most cells sat exactly ON it
     (7 of 8 `wrong_side` cells) — where the number reports how big the
     selection was and nothing else.
   - **The negative control mostly did not perturb.**
     `portfolio.perturb_jitter` skips a part whose incumbent pose is not fully
     legal, which on a dense board is every part, so three `scatter` cells
     produced a board byte-identical to the control. The published 0.83 was an
     interpolated median of `{0, 0, 1.66, 5.06}` — no cell was near 1.0.

   Two limits survive even in the corrected form. `base_rate` is the exact null
   for a uniform random pick of n PARTS, but this selector picks whole BLOCKS
   and the truth is a block from the same derivation; the block-structured null
   runs 0.69–3.12 depending on the board, and under it no cell reaches the 95th
   percentile. And the study runs the ranking **unbudgeted** while the loop
   budgets every round — which the module's own `SIGNAL_ORDER` note says can
   change the selected set outright.

### Pre-registered rule 5 — what would settle it, and why it has not been run

The claim `--target-select diagnosis` does NOT make is that it routes better.
Settling that needs a paired routed A/B, `pins` against `diagnosis`, on ≥ 3
boards with the same round budget, graded on `failures` then routed `blocking`,
by the same accept rule this document uses elsewhere: right direction on ≥ N−1
boards, wrong on none.

It has not been run, and the cost is the reason: this study's own provenance
records a median 217 s per route and a maximum of 2012 s over its 87 rows (on
`tigard:perturb-wrong_side`), plus a single `tigard:perturb-pile` row at
9877 s — so three boards × five rounds × two arms is hours to overnight
serially. There is a harder problem than cost, and it is recorded here so the
next person does not rediscover it: **the tracked corpus has almost no board
that fails to route at its authored placement.** The one that does is
kit-dev-coldfire — `py_placer/placement/README.md` records the loop taking its
hand placement from 3 failed nets to 0, and this document's own table shows no
zero among its observed `blocking` values — so an A/B would rest on ONE board
where N ≥ 3 is the acceptance rule. Manufacturing failures elsewhere means
perturbing, which re-introduces the damage-family caveat the recall study
already carries.

### Pre-registered rule 6 — the withdrawal

If that A/B runs and `diagnosis` does not win by rule 5's criterion, the flag is
withdrawn from the default-available set — and the row keeps its measured
direction, in the `rejected` style `test_placement_ab.py` uses. A rejected
finding that is deleted becomes folklore; a rejected finding that keeps its row
stays a change detector.

## What #554 measured, and the rules recorded before it ran

`--relocate` (`py_placer/placement/relocate.py`) moves one diagnosed block toward
its connectivity target by letting the neighbours yield in preserved relative
order. Two questions, and only the first is answered.

**The mechanism question — ANSWERED.** Does yielding buy travel a frozen board
cannot? `tests/stress/relocation_reach.py`, paired, one variable changed: both
arms run the same solve over the same graph, and the frozen arm simply pins every
other unit. Over the 24 measurable blocks on the 9 boards that have one (a
tenth, glasgow_revC, contributes only refusals), yielding bought ≥ 1 mm on **11 cells
spanning 6 boards**, median gain 0.31 mm, max 16.66 mm; 11 cells move in neither
arm and are counted, not dropped; 3 are refused by name (glasgow_revC's blocks
contain KiCad-locked parts).

**The routed question — NOT ANSWERED by that study, and it must not be read as
if it were.** Reach is not routability.

### Pre-registered rule 7 — the headline is routed, and `recovery` is not it

A relocation is reported as working only on
`route_recovery = (blocking(D) − blocking(R)) / (blocking(D) − blocking(C))`
over failed + open + pad-deficit, where D is the damaged board, C the undamaged
control and R the repair. A cell counts only if `blocking(D) > blocking(C)` on
identical router argv; one that fails that is `not_placement_limited` and is
reported with both numbers, never dropped.

Pose `recovery` is a DIAGNOSTIC here for a reason sharper than CLAUDE.md's
general one: the solve aims at `net_centroid`, and across the 24 live cells of
`tests/stress/relocation_reach.py` a block sits **3.39–69.20 mm from its own net
centroid at the human's placement** (`want_mm` in the committed baseline; median
10.36 mm).
The best achievable pose recovery is therefore bounded above by
`1 − d_ctrl/dose`, which is at or below zero on most cells before anything runs.
Every pose number is reported with that ceiling beside it, and a pose null on a
ceiling-bound cell is not evidence about the solver. `compensated` — a routed win
at recovery ≈ 0 — is a first-class PASS, exactly as #411 concluded for the loop.

### Pre-registered rule 8 — the delta is the evidence, not the raw recovery

The same relocation is applied to the UNDAMAGED control (`R0`) on every counted
cell, and `route_recovery − route_recovery(R0)` is what is reported. Without it a
repair arm cannot tell "it fixed the damage" from "it moves that block on every
board" — which is exactly how #553's recall study produced a finding that
evaporated. Acceptance: the delta positive on ≥ N−1 evidence cells and negative
on none, with N ≥ 3, with the `loop@allon` column printed beside it. #554 is only
interesting if it beats a tool that already exists.

### Pre-registered rule 9 — the contract can fail a cell that routed better

`collateral_pad_rms` must not rise and the block must not tear, whatever
`blocking` did. "Move only diagnosed blocks; keep the relative order of
everything else as a hard constraint" is #554's definition, not its bonus: a
solve that buys routability by walking the neighbours is `place_route_loop`, and
that already exists.

### Pre-registered rule 10 — `translate` is an instrument, never evidence

`perturb.block_direction` computes the damage direction from
`routability.block_displacements`, the same quantity this solve consumes, so a
`translate` recovery is partly arithmetic. It is printed and never aggregated —
the same convention `diagnosis_recall.py` applies for the same reason. `swap` and
`wrong_side` are the evidence arms; `scatter` is excluded because
`portfolio.perturb_jitter` skips illegal-pose parts and it often damages nothing,
and `pile` because every free part moves so there is no "the block".

### Pre-registered rule 11 — the dose ladder is amended, not ignored

#411's 5/10/20/40/80 mm ladder is **geometrically impossible on most of this
corpus**: of the 10 boards the reach sweep covers, **5** have a board diagonal
under 80 mm and **2** under 40 mm, and a diagonal is a generous upper bound on
how far a block inside the outline can travel. (An earlier revision said "9 of
the 11", against a set of 11 boards that does not exist in the tree; the numbers
above are re-derivable from `board_bounds`.) It is replaced by a probed dose per
(board, block, direction), with the APPLIED damage measured by diffing the
control against the damaged board — never from `clipped` or
`max_feasible_dose_mm`, which describe a rigid-translate probe that only two of
the arms clip against and which was wrong by 79× once. A cell whose applied dose
is under `2 × recovery.HOME_TOLERANCE_MM` is `unmeasurable_dose` and is reported
with its numbers.

### Pre-registered rule 12 — the withdrawal, both ways

If the routed study runs and the relocation does not clear rule 8, `--relocate`
is not offered in the default path, and the row keeps its measured direction in
the `rejected` style. And symmetrically: if it clears rule 8 while failing
rule 9, what is recorded is "it routes better by moving the neighbours" — that is
the loop — and **#554's constraint framing is withdrawn rather than the tool**.

### What #554 does NOT license

- Not that a relocated board routes better. Nothing has measured that.
- Not that `reach_mm` is closure on the target: it is board-frame travel, and the
  target moves with the corridor. Bounded by LP, closure is smaller on most cells
  and larger on a few. The aggregate mechanism verdict survives the substitution;
  no individual cell's number is the closure.
- Not that the constraint graph is a conservative model of legality. It is not —
  a Euclidean gap against a per-axis constraint leaves 73 of 54,841 corpus pairs
  exposed. The exact re-check, not the graph, is what makes the pass safe.

### What the routed study actually returned (#554)

`tests/stress/block_relocation_study.py` ran on 2026-08-30. Rows and summary are
committed at `tests/554_block_relocation_baseline.json`;
`tests/test_554_relocation_regen.py` re-derives the whole summary from those rows
through the study's own `summarise()` (it does **not** re-run the routes — hours
— and says so rather than sampling and implying otherwise).

| board / kind | blocking C → D | R | R0 | L (`loop@allon`) | delta |
|---|---|---|---|---|---|
| esp_prog / swap | 0 → 6 | 4 (**0.33**) | 0 | 1 (**0.83**) | +0.333 |
| esp_prog / wrong_side | 0 → 3 | 0 (**1.00**) | 0 | 0 (**1.00**) | +1.000 |
| splitflap_driver / swap | 0 → 7 | 3 (**0.57**) | 0 | 2 (**0.71**) | +0.571 |
| esp_prog / translate | 0 → 3 | 0 | — | 0 | instrument only |
| splitflap / wrong_side, translate | 0 → 0 | — | — | — | `not_placement_limited` |

**Verdict: UNDERPOWERED, and the incumbent won.** The direction is right — 3 of 3
evidence cells positive against their own undamaged pairing, median **+0.571**,
none negative — but on **2 boards**, and rule 8 counts boards, because per-board
spread here is ±2–3 nets. And `place_route_loop` with the pin gate lifted reached
a **strictly better** routed result on **2 of the 3** cells, tying on the third.

Three things this establishes, none of which is "it works":

1. **The pipeline fires end to end and the arithmetic is sound.** The paired
   `R0` arm reads 0 on every cell — the relocation does not disturb an undamaged
   board — so the deltas are not the artefact that voided #553's first reading.
2. **Two of six cells were `not_placement_limited`**: on splitflap the damage did
   not make the board route worse at all, so there was nothing to restore. Those
   are reported with both routed numbers, because the RUNBOOK's definition of a
   subject is exactly that the dose must threaten routability.
3. **The feature does not currently beat the tool that exists.** That is the
   comparison rule 8 pre-registered, and it came back against #554.

What would settle it: the same study over ≥ 3 boards that fail at their authored
placement or under a dose that threatens routing. On this corpus that means
adding tigard and kit-dev-coldfire, at roughly 15–40 minutes per cell.
