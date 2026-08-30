# Calibrating P-close's routability ratio — refuse on the DECISION, never on the numbers (#803)

**Status: COMPLETE, and the conclusion is a withdrawal.** The threshold is
withdrawn; the refusal is not. A scoring gate was measured refusing correct
answers, so the refusal moved from *"your hpwl is too low"* to *"nobody said
which this is"*, and any written disposition clears it.

This page exists because eleven sites in the two placement drivers cited a file
that was never committed. `wk/` is gitignored, `wk/calibration/RESULT.md` was
authored in a throwaway worktree, and three of those eleven citations are
**printed to the operator at runtime** — so a run emitted a refusal naming a file
its reader could not open. The reasoning below is the only record of *why*
`placement_driver.py`'s disposition gate has the shape it has, which makes it
load-bearing rather than a stale comment. #803 is that finding; this file is the
repair.

> **The rows ARE committed here, and they are small.** 4.4 KB of blobs:
> [`tests/placement_calibration_rows.json`](../tests/placement_calibration_rows.json)
> (the three per-board metric blocks) and
> [`tests/placement_calibration_recovered.json`](../tests/placement_calibration_recovered.json)
> (the partial-board gains).
> [`tests/test_803_calibration_claims.py`](../tests/test_803_calibration_claims.py)
> parses this markdown and re-derives it from those two files, so the tables
> cannot drift from their evidence the way `docs/placement-predictors.md`
> deliberately accepted that its rows could.
>
> **What that gate does NOT cover, stated because the alternative is a false
> sense of coverage.** It checks the three tables cell by cell, the
> perfect-repair arithmetic, the separation, the blocking result and the
> per-board timings. It does **not** check the prose — a sentence contradicting
> a table would pass, and one did (see reason 3). Numbers quoted only in prose
> are marked where they appear.

Reproduce the rows (hours, not minutes — see *Side findings*):

```bash
python3 -X utf8 tests/stress/calibrate_congestion_ratio.py \
    --boards neo6502 urchin piantor --out wk/calibration
```

---

## What was being measured

P-close refused a close-out when `hpwl_gain < ratio * halo_gain`, each gain being
`(before - after)/before` against the board the placement run started from. The
ratio shipped at **0.25 — a number that was chosen, not measured.**

**The rule has a second clause, and the `verdict` column below cannot be read
without it:** the comparison only happens when `halo_gain >= 0.25`. Below that
the arm is treated as having barely moved and passes untested — what the tables
call **"pass (early-out)"**. It is `calibrate_congestion_ratio.py:202-203`, and
it is live today as `if halo_gain >= 0.25 and hpwl_gain < 0.25 * halo_gain` in
`placement_driver.py`. Without it urchin's partial repair (halo_gain 0.0026,
hpwl_gain −0.1583) would read as a REFUSE rather than the early-out it is.

Three populations per board were built to find where, or whether, a ratio
separates a healthy repair from a legality-only pass:

| population | what it is |
|---|---|
| `pristine` | the corpus board, undamaged |
| `damaged` | `pristine` after `--kind swap` |
| `repaired` | `damaged` after a full `place_reconstruct` |
| `legality_only` | `damaged` after a legality-only pass |

Metrics come from `PlacementModel(...).metrics` directly — byte-identical to
`render_placement --json-out` by construction, and verified so on neo6502 (halo
869.1450, hpwl 4193.6257, crossings 1871, exactly the run-15 render JSON).

## The measurements

`neo6502` is the only board that completed all four populations.

| board | population | halo | hpwl | crossings | blocking |
|---|---|---|---|---|---|
| neo6502 | pristine | 208.014 | 3622.595 | 1324 | 0 |
| neo6502 | damaged | 869.145 | 4193.626 | 1871 | **18** |
| neo6502 | repaired | 536.071 | 4044.555 | 1828 | **0** |
| neo6502 | legality_only | 450.567 | 4174.505 | 1878 | **0** |
| urchin | pristine | 6721.157 | 2581.550 | 123 | 0 |
| urchin | damaged | 7112.586 | 3766.213 | 234 | 1 |
| piantor | pristine | 986.914 | 2263.609 | 186 | 0 |
| piantor | damaged | 1390.545 | 1965.075 | 128 | 1 |

And the gains the gate actually reads:

| board | population | halo_gain | hpwl_gain | ratio | verdict @ 0.25 |
|---|---|---|---|---|---|
| neo6502 | undamaged → itself | 0.0000 | 0.0000 | — | pass (early-out) |
| neo6502 | damaged → **full repair** | 0.3832 | 0.0355 | **0.0928** | **REFUSE** |
| neo6502 | damaged → legality only | 0.4816 | 0.0046 | 0.0095 | **REFUSE** |
| urchin | damaged → repair *(partial)* | 0.0026 | **−0.1583** | −60.6 | pass (early-out) |
| urchin | damaged → legality *(partial)* | 0.0000 | 0.0000 | — | pass (early-out) |
| piantor | damaged → repair *(partial)* | 0.0000 | 0.0000 | — | pass (early-out) |
| piantor | damaged → legality *(partial)* | 0.0000 | 0.0000 | — | pass (early-out) |

> **Read the partial rows correctly.** `rows.json` carries **no `g_repair` or
> `g_legality` key at all** for urchin and piantor, because neither board's
> repair terminated and `calibrate_congestion_ratio.py` only writes them on
> success. (An earlier draft of this page said "null"; the keys are absent,
> which is a different thing and the one the gate actually tests.) The four
> *(partial)* rows above come from
> `placement_calibration_recovered.json`, which was assembled by hand from the
> `<output>.staging.kicad_pcb` boards those runs left behind. **No committed
> script writes that file** — `calibrate_congestion_ratio.py` writes only
> `rows.json`. Treat the partial rows as recovered evidence, not as a rerunnable
> measurement.

## Four reasons not to refuse on this

### 1. At 0.25 the gate refuses a repair that worked

neo6502's full `place_reconstruct` took blocking **18 → 0** and is REFUSED (ratio
0.0928). So is the legality-only arm (0.0095). **0.25 refuses everything this
corpus can produce**, which means "the gate catches run 15" was true and
uninformative — it catches every outcome, correct ones included.

### 2. The premise inverts on 1 of 3 boards

The gate assumed damage raises hpwl, so a repair should lower it. Measured:

| board | hpwl pristine | hpwl damaged | delta | premise |
|---|---|---|---|---|
| neo6502 | 3622.6 | 4193.6 | +571.0 | holds |
| urchin | 2581.5 | 3766.2 | +1184.7 | holds |
| **piantor** | 2263.6 | **1965.1** | **−298.5** | **inverted** |

`swap` exchanges parts; on a regular structure — piantor is a keyboard matrix —
that can *shorten* nets. So on piantor a **perfect repair, restoring the pristine
board exactly, scores `halo_gain` +0.2903, `hpwl_gain` −0.1519, ratio −0.5234,
and the gate REFUSES IT.** A gate that refuses the correct answer must not
refuse.

Crossings inverts on the same board and harder: pristine 186 against damaged 128,
a gain of −0.4531.

### 3. Only ONE board yields a calibration pair at all

`--kind swap` produced **18** blocking pairs on neo6502 and **1** on both urchin
and piantor. With one pair to clear there is nothing for a legality repair to
do, which is why three of the four partial gains are exactly 0.0000. The corpus
therefore supports **n = 1**, and a threshold fitted to one board is the same
kind of number as the one it would replace.

The fourth is not 0.0000, and the lost write-up's "every urchin/piantor gain is
exactly 0.0000" was wrong about it: urchin's partial repair scored halo_gain
+0.0026 and **hpwl_gain −0.1583**, ratio −60.6. It moved almost no halo and
lengthened nets, on a board whose one blocking pair left nothing to repair — so
it does not supply a calibration pair either, but the reason is "the arm did
almost nothing", not "the arm scored zero".

(`qualify.json` rated all three GOOD, which is not contradicted: it counts a gate
firing on *either* DRC or assembly across 8 draws, while this needs a material
**assembly** gap from one particular kind of damage.)

### 4. Even the best-fitting ratio would not do the job it was built for

On neo6502 the two populations separate by **9.80×** (0.09276 against
0.00947), so a ratio near 0.02–0.05 would split them. But run 15's own arm scored
**0.064** — *above* that band — so the calibrated gate would **pass** the very
board it was built to catch. The threshold cannot simultaneously admit a healthy
repair and reject run 15's.

(0.064 is run 15's figure, from that run's own render JSON. It is the one number
on this page that is not in the committed rows, and it is quoted, not re-derived.)

## What is kept

The **measurement** is worth keeping even though the refusal is not:

- neo6502's 9.80× separation is real signal — a full repair moves hpwl an order
  of magnitude more, relative to halo, than a legality-only pass.
- The run-15 lesson does not depend on a threshold: *before claiming a failure is
  `parameter`-shaped, look at global congestion*, because every per-net test can
  pass on a board no router can finish.

So both guards keep their **evidence requirement** — you must supply the
before/after render — and replace the **threshold refusal** with a **disposition
refusal**:

* P-close refuses a poor read until `--waive congestion:<reason>` or a lever is
  pulled.
* L4 refuses `--shape parameter` on a poor read until `--accept-congestion
  "<reason>"` — a dedicated flag, deliberately NOT part of `--accept-residue`,
  whose vocabulary belongs to the L2 placement gate; a waiver spanning two gates
  waives the one that was working.

The distinction is the whole result. A gate that scores the board refused a
perfect repair on piantor. A gate that asks *"did anybody decide?"* cannot make
that mistake: the perfect repair records why the read looks poor, and proceeds.

**Merely REPORTING was tried first and was wrong.** It left run 15's exact arm
closing out clean (`exit 0`) with the evidence on screen and nothing asked of it.
Advisory is what the close-out already had too much of. An earlier draft of the
lost write-up concluded "report, don't refuse"; that draft is the one this
section overturns.

## What this qualifies elsewhere

**"hpwl behaves — its minimum is at the truth"** is not universally true, and
`loop_driver.py` asserted it as the reason hpwl is the metric that can carry a
gate. On piantor the *damaged* board scores lower hpwl than the truth (1965.07
against 2263.61), so hpwl's minimum is not at the truth there. That comment now
carries the caveat and points here. This does not make hpwl useless — it makes it
a metric whose direction must be checked per board rather than assumed.

## What this does not claim

- **Not that 0.25 is merely mis-tuned.** Reason 4 is the load-bearing one: no
  single value satisfies both constraints the gate was asked to satisfy.
- **Not that a routability ratio is unmeasurable in principle**, only that this
  corpus cannot measure one — reason 3 puts n at 1.
- **Not that halo and hpwl are interchangeable across configurations.** The ratio
  is **knob-dependent**: `halo` is a penalty whose value depends on
  `halo_coef`/`halo_weight`/`clearance`. `render_placement` uses 0.25/2.0;
  `pose_score.make_state` uses `halo_coef` 0.15 with `halo_weight` 2.0, and it
  is its caller `placement/recovery.py`'s `static_metrics` that drops halo from
  the reported set — the lost write-up said `make_state` "discards halo", which
  the harness it was derived from got right and the write-up did not. Any ratio
  measured here would have been valid only for `render_placement`'s weights —
  one more reason a single fitted number was never going to travel.
- **Not a statement about routed `blocking` as a dependent variable.** The
  `blocking` column above is the *assembly* blocking-pair count these
  populations were built around, not a routed outcome. What predicts routed
  `blocking` is measured separately, in
  [`docs/placement-predictors.md`](placement-predictors.md).

## Side findings

- **`place_reconstruct` does not complete on the larger boards.** In the
  committed run, urchin exited 7 after **1061.8 s** and piantor hit
  **`TIMEOUT after 1200s`** (exit 124); the legality-only arms exited 7 after
  904.4 s and 901.4 s. neo6502, by contrast, repaired in **29.7 s**. This is the
  run-9 non-termination shape on 87–103-part boards, not 217.
  *(An earlier run of the same experiment, at a 3000 s deadline, reached the same
  qualitative verdict with different seconds — urchin 8/13 reseats at t=2213 s,
  piantor 735 s to seat one part of five. Those figures are quoted from the lost
  write-up and are **not** the committed rows; the rows here are the run that
  ships.)*
- **Its deadline behaviour is correct and easy to misread**: on a deadline the
  output path is NOT written, and the partial board is left at
  `<output>.staging.kicad_pcb`, said only on stderr. The first harness read that
  as "no board produced". It is not — and it is what made the partial rows
  recoverable at all.
