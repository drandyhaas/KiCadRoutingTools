# What predicts routed `blocking` — the first measurement (#703)

**Status: COMPLETE. The declared table is 6 boards x K=20 = 120 placements, and
all 120 were routed.** No predictor here is "validated" in a strong sense; the
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
> the shuffle-control rates — **and re-deriving them is an 8.8-hour job** (the sum of `provenance.total_seconds`
> over the 87 rows; median 217 s, max 2012 s on `tigard:perturb-wrong_side`;
> about 2.5 h wall clock at `-j 4`). It is the recorded finding.
>
> ```bash
> # rebuild the rows (8.8 h serial, ~2.5 h at -j 4)
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

**1. `pad_copper` -- the one pre-route number that already refuses -- is
validated.** `loop_driver.py`'s L2 gate blocks the first route on
`checklist.a_off_outline.pad_copper`, and it is the only pre-route quantity in
this repo that refuses anything. It ranks `blocking` positively on 6 of 6
boards: +0.481 esp_prog, +0.392 kit-dev-coldfire, +0.649 sonde_u, +0.568
splitflap, +0.473 tigard, +0.622 watchy. The gate was right, and now it is
measured rather than assumed.

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
3. A predictor is reported as ranking `blocking` only on ≥ N−1 boards right and
   none wrong, over boards with a *defined* ρ, with N ≥ 3 and the shuffle-control
   rate printed beside it.
4. Saturated and starved boards are reported with their constant value and
   excluded from the denominator. They are never dropped.

## What this does not claim

- Not that the passing predictors *cause* anything. They rank an outcome on four
  boards.
- Not that `crossings` or `hpwl` are useless. Both are measured against
  distance-to-truth, both retain the roles that measurement supports, and the
  drivers' existing prohibition on gating `crossings` is untouched.
- Not that the legality family is ten findings. Six of them share a median to
  three decimals; they are one quantity seen through six counters.
- Not a corpus-wide result. Four boards, one machine, one router build. Each
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
3. **Recall is not efficacy.** `tests/stress/diagnosis_recall.py` measures
   whether the ranking concentrates on a block the perturber displaced, with no
   routing at all. Its evidence arm is above chance on 4 of 4 boards (median
   lift 2.32) while its negative control sits at chance (0.83), and its
   `translate` arm is arithmetic and labelled so. None of that is a routing
   outcome.

### Pre-registered rule 5 — what would settle it, and why it has not been run

The claim `--target-select diagnosis` does NOT make is that it routes better.
Settling that needs a paired routed A/B, `pins` against `diagnosis`, on ≥ 3
boards with the same round budget, graded on `failures` then routed `blocking`,
by the same accept rule this document uses elsewhere: right direction on ≥ N−1
boards, wrong on none.

It has not been run, and the cost is the reason: this study's own provenance
records a median 217 s per route and a maximum of 2012 s, with tigard's hard
rows at 9877 s, so three boards × five rounds × two arms is hours to overnight
serially. There is a harder problem than cost, and it is recorded here so the
next person does not rediscover it: **the tracked corpus has no board that
fails to route at its authored placement.** Manufacturing failures means
perturbing, which re-introduces the damage-family caveat the recall study
already carries.

### Pre-registered rule 6 — the withdrawal

If that A/B runs and `diagnosis` does not win by rule 5's criterion, the flag is
withdrawn from the default-available set — and the row keeps its measured
direction, in the `rejected` style `test_placement_ab.py` uses. A rejected
finding that is deleted becomes folklore; a rejected finding that keeps its row
stays a change detector.
