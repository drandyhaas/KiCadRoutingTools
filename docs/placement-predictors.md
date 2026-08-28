# What predicts routed `blocking` — the first measurement (#703)

**Status: MEASURED, on 5 boards of a declared 6, with NO excluded variants.
The sixth (`kit-dev-coldfire-xilinx_5213`) is routing as of this revision.**
No predictor here is "validated" in a strong sense; the acceptance rule's own
false-positive rate at N=5 is **4.5%-8.0%** (median 6.5%) over the 21 predictors
defined on all five boards, measured, and that number belongs beside every
PASSES below.

Every correlation number this repo quoted before #703 was measured against
*distance-to-the-correct-placement* or against *the gap a human left*.
`r(crossings) = +0.780` is 29 candidates on ONE board against distance-to-truth;
the corridor law's `r = +0.41 … +0.90` is against the human's gap. CLAUDE.md's
*"What a placement run is FOR"* says the headline is routed `blocking`, and no
predictor had ever been correlated with it.

This is that measurement. Nothing in it is a proxy for a proxy: 100 placements
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
| `pad_shortfall` | 5 / 0 | +0.830 | **passes** |
| `pad_intersection_pairs` | 5 / 0 | +0.828 | **passes** |
| `pad_overlap_pairs` | 5 / 0 | +0.828 | **passes** |
| `body_overlap_pairs` | 5 / 0 | +0.828 | **passes** |
| `pad_conflict_pairs` | 5 / 0 | +0.827 | **passes** |
| `pad_clearance_pairs` | 5 / 0 | +0.827 | **passes** |
| `courtyard_blocking_pairs` | 5 / 0 | +0.826 | **passes** |
| `pad_copper` (off-outline pad copper) | 5 / 0 | +0.568 | **passes** |
| `hole_shortfall` | 4 / 0 | +0.505 | **passes** |
| `hole_conflicts` | 4 / 0 | +0.501 | **passes** |
| `overlap_area` | 4 / 1 | +0.764 | fails |
| `halo` | 4 / 1 | +0.603 | fails |
| `courtyard_overlap_mm2` | 4 / 1 | +0.557 | fails |
| `courtyard_advisory_pairs` | 4 / 1 | +0.516 | fails |
| `total` | 4 / 1 | +0.487 | fails |
| **`crossings`** | **4 / 1** | **+0.394** | **fails** |
| `oob_count` | 4 / 1 | +0.220 | fails |
| `courtyard_off_outline` | 4 / 1 | +0.220 | fails |
| `oob_amount` | 4 / 1 | +0.168 | fails |
| `oob_area` | 3 / 2 | +0.150 | fails |
| `length` | 3 / 2 | +0.061 | fails |
| **`hpwl`** | **3 / 2** | **+0.040** | **fails** |
| `edge` | 2 / 3 | -0.003 | fails |
| `cross_side_stacks` | 1 / 1 | +0.076 | **no verdict** (defined on 2 boards) |
| `align`, `corridor_cut`, `orient`, `locked_contact_pairs` | 0 / 0 | - | **no verdict** (constant everywhere) |

The rule is `test_placement_ab.gate()`'s, transposed from marks to signs: right
direction on ≥ N−1 boards, wrong direction on none, over the boards that
produced a *defined* ρ. Spearman is computed **within each board** and never
pooled.

### Three things this says that the repo did not know

**1. `pad_copper` -- the one pre-route number that already refuses -- is
validated.** `loop_driver.py`'s L2 gate blocks the first route on
`checklist.a_off_outline.pad_copper`, and it is the only pre-route quantity in
this repo that refuses anything. It ranks `blocking` positively on 5 of 5
boards: +0.481 esp_prog, +0.649 sonde_u, +0.568 splitflap, +0.473 tigard,
+0.622 watchy. The gate was right, and now it is measured.

**2. `hpwl` -- which the drivers DO gate on -- barely relates to the routed
outcome at all.** 3 boards right, 2 wrong, median **+0.040**, two-sided p = 1.0:

| board | rho(hpwl, blocking) |
|---|---|
| esp_prog | **-0.525** [LOO -0.648..-0.443, K=20] |
| sonde_u | +0.040 [LOO -0.168..+0.345, K=17] |
| splitflap_driver | +0.429 [LOO +0.352..+0.739, K=20] |
| tigard | **-0.185** [LOO -0.353..-0.045, K=20] |
| watchy | +0.079 [LOO -0.080..+0.360, K=14] |

Adding the fifth board moved it from +0.158 to +0.040 -- the more boards, the
less there is. The reasoning behind gating on it (*"hpwl's minimum is at the
truth, so it is the one that can carry a gate"*) is about distance-to-truth and
remains untouched. What is new is that its relationship to the routed outcome is
a coin flip across boards. **This does not license removing that gate**; it means
the gate is justified by the distance argument alone, and nobody should describe
it as a routability gate.

**3. `crossings` fails, and its sign flips.** rho = -0.179 on esp_prog against
+0.017 / +0.636 / +0.705 / +0.394 elsewhere. More crossings went with *less*
blocking on one of five boards, and sonde_u is indistinguishable from zero.

## The circularity control changed the answer for `crossings`

The realistic-end sampler is `portfolio.generate`, whose quench **minimises
crossings and hpwl**. Those rows carry `generator: portfolio_quench`, and every
statistic is computed twice:

| predictor | quench rows INCLUDED | quench rows EXCLUDED |
|---|---|---|
| `pad_copper` | 5/0, +0.568, **passes** | 5/0, +0.527, **passes** |
| `pad_clearance_pairs` | 5/0, +0.827, **passes** | 5/0, +0.959, **passes** |
| `courtyard_blocking_pairs` | 5/0, +0.826, **passes** | 5/0, +0.950, **passes** |
| **`crossings`** | **4/1, +0.394, fails** | **5/0, +0.475, passes** |
| `halo` | 4/1, +0.603, fails | 5/0, +0.945, passes |
| `overlap_area` | 4/1, +0.764, fails | 5/0, +1.000, passes |
| `hpwl` | 3/2, +0.040, fails | 3/2, +0.039, fails |

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
hole_conflicts                11.5%
hole_shortfall                11.5%
hpwl                           8.0%
length                         8.0%
courtyard_advisory_pairs       7.5%
body_overlap_pairs             7.0%
...
cross_side_stacks              0.0%   <- defined on 2 boards; see below
```

Full sweep over the 21 predictors defined on all five boards: **min 4.5%, max
8.0%, median 6.5%**. (`hole_conflicts` and `hole_shortfall` sit above that band
because they are defined on only four boards, so they are judged by the easier
N=4 rule.)

Two consequences, both load-bearing:

- **A predictor defined on one board used to pass the rule every single time.**
  Before the fix below, `cross_side_stacks` — constant on three of four boards,
  so defined on one — passed **100%** of these shuffles, because
  `consistent >= max(1, N-1)` is `1 >= 1` at N=1. `rank_stats.MIN_SIGN_BOARDS`
  is now 3, the same value and the same reason as
  `test_placement_ab.MIN_TRIAL_BOARDS`, and such a predictor reports **NO
  VERDICT** instead. Its rate in the block above is 0.0% *because that guard is
  in place* — the 100% is what the control measured before it existed, and it
  is why it exists.
- **At N=5 the rule's empirical false-positive rate is 4.5–8.0%** (median
  6.5%), against 9–18% at N=4 -- the fifth board roughly halved it, which is the
  concrete value of running the declared table rather than stopping early. Still
  not the 0.125
  the two-sided p-value suggests. Ten predictors passing is therefore *evidence*
  and not proof; the seven that share a median near +0.83 are also plainly
  measuring one underlying quantity, so they are not ten independent findings.

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
| tigard | 20 | measurable | 0, 1, 2, 64, 71, 73, 114, 158 | - |
| sonde_u | 17 | measurable | 0, 2, 21, 23, 877 | 3 duplicate placements collapsed |
| watchy | 14 | measurable | 1, 3, 5, 44, 94, 8521 | 6 duplicate placements collapsed |

**Every variant produced a routed result. There are no excluded rows.** An
earlier revision reported `perturb-pile` timing out on 3 of 4 boards at a 2400 s
budget; those were near-misses rather than impossibilities (splitflap needed
2798 s), and all five boards now carry a pile row: blocking 466 / 877 / 5424 /
8521 / 13078. Raising a *timeout* is not a protocol change for rows that already
finished -- a route that completed in 300 s completes identically at any larger
budget -- so it strictly adds samples.

Every board is git-tracked, so every row is regenerable from this repo.
`portfolio.generate(..., only=i)` is byte-identical by contract, the route argv
is frozen per board in `ARGV.json`, and each row carries its
`poses_sha256`, `input_board_sha` and `argv_sha`.

**What is NOT in this run, said plainly:**

- **The declared table is 6 boards; 5 are complete.**
  `kit-dev-coldfire-xilinx_5213` is routing as of this revision. It is the most
  expensive board in the set by a wide margin -- its *authored* placement takes
  **5013 s** to route against tigard's 603 s -- and its first four variants score
  `blocking` 7, 9, 9, 6, so it is measurable, just slow. Every p-value and every
  N above is over the 5 that are done, never over the planned count;
  `predictor_study.py` prints that denominator on the same line as the p-value
  for exactly this reason.
- **The duplicate guard cost sonde_u 3 slots and watchy 6.** Their `translate`
  and `scatter` blocks have little feasible travel on those outlines, so some
  variants reproduced the authored board exactly. Every drop is named in the
  report.

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
