# Calibrating P-close's routability ratio — refuse on the DECISION, never on the numbers

**Verdict: the threshold is withdrawn. The refusal is not.**

The gate must not judge the numbers — measurement says a scoring gate refuses
correct answers. It must still refuse a run that leaves the question
unanswered. So the refusal moves from *"your hpwl is too low"* to *"nobody said
which this is"*, and any written disposition clears it.

(An earlier draft of this file concluded "report, don't refuse". That was tried
and was wrong — see **What is kept**. It left run 15's exact arm closing out
clean with the evidence on screen.)

Reproduce:

```bash
python3 -X utf8 tests/stress/calibrate_congestion_ratio.py \
    --boards neo6502 urchin piantor --out wk/calibration
```

Raw rows: `rows.json`; partial-board recovery: `recovered.json`.

---

## What was being measured

P-close refuses a close-out when `hpwl_gain < ratio * halo_gain`, each gain
being `(before - after)/before` against the board the placement run started
from. The ratio shipped at **0.25 — a number I chose, not measured.** Three
populations per board were built to find where, or whether, a ratio separates a
healthy repair from a legality-only one.

Metrics come from `PlacementModel(...).metrics` directly — byte-identical to
`render_placement --json-out` by construction, and verified so on neo6502
(halo 869.1450, hpwl 4193.6257, crossings 1871, exactly the run-15 render JSON).

## The measurements

| board | population | halo_gain | hpwl_gain | ratio | blocking | verdict @ 0.25 |
|---|---|---|---|---|---|---|
| neo6502 | undamaged → itself | 0.0000 | 0.0000 | — | 0 | pass (early-out) |
| neo6502 | damaged → **full repair** | 0.3832 | 0.0355 | **0.0928** | 18 → **0** | **REFUSE** |
| neo6502 | damaged → legality only | 0.4816 | 0.0046 | 0.0095 | 18 → **0** | REFUSE |
| urchin | damaged → repair *(partial)* | 0.0026 | **−0.1583** | −60.6 | 1 | pass (early-out) |
| urchin | damaged → legality *(partial)* | 0.0000 | 0.0000 | — | 1 | pass (early-out) |
| piantor | damaged → repair *(partial)* | 0.0000 | 0.0000 | — | 1 | pass (early-out) |
| piantor | damaged → legality *(partial)* | 0.0000 | 0.0000 | — | 1 | pass (early-out) |

## Four reasons not to refuse on this

### 1. At 0.25 the gate refuses a repair that worked

neo6502's full `place_reconstruct` took blocking **18 → 0** and is REFUSED
(ratio 0.0928). So is the legality-only arm (0.0095). **0.25 refuses everything
this corpus can produce**, which means "the gate catches run 15" was true and
uninformative — it catches every outcome, correct ones included.

### 2. The premise inverts on 1 of 3 boards

The gate assumes damage raises hpwl, so a repair should lower it. Measured:

| board | hpwl pristine | hpwl damaged | delta | premise |
|---|---|---|---|---|
| neo6502 | 3622.6 | 4193.6 | +571.0 | holds |
| urchin | 2581.5 | 3766.2 | +1184.7 | holds |
| **piantor** | 2263.6 | **1965.1** | **−298.5** | **inverted** |

`swap` exchanges parts; on a regular structure (piantor is a keyboard matrix)
that can *shorten* nets. On piantor a **perfect repair — restoring the pristine
board exactly — scores `halo_gain +0.2903`, `hpwl_gain −0.1519`, ratio −0.5234,
and the gate REFUSES IT.** A gate that refuses the correct answer must not
refuse.

This also qualifies the placement skill's own claim that *"hpwl behaves — its
minimum is at the truth"*. On piantor the damaged board has **lower** hpwl than
the truth, so hpwl's minimum is not at the truth there either. That claim needs
a caveat, or a measurement showing piantor is unrepresentative.

### 3. Only ONE board yields a calibration pair at all

`--kind swap` produced **18** blocking pairs on neo6502 and **1** on both urchin
and piantor. With one pair to clear there is nothing for a legality repair to do,
which is why every urchin/piantor gain is exactly 0.0000. So the corpus supports
**n = 1**, and a threshold fitted to one board is the same kind of number as the
one it would replace.

(`qualify.json` rated all three GOOD, which is not contradicted: it counts a gate
firing on *either* DRC or assembly across 8 draws, while this needs a material
**assembly** gap from one particular kind.)

### 4. Even the best-fitting ratio would not do the job it was built for

On neo6502 the two populations separate by **9.8×** (0.0928 vs 0.0095), so a
ratio near 0.02–0.05 would split them. But run 15's own arm scored **0.064** —
*above* that band — so the calibrated gate would **pass** the very board it was
built to catch. The threshold cannot simultaneously admit a healthy repair and
reject run 15's.

## What is kept

The **measurement** is worth keeping even though the refusal is not:

- neo6502's 9.8× separation is real signal — a full repair moves hpwl an order of
  magnitude more than a legality-only pass.
- The run-15 lesson does not depend on a threshold: *before claiming a failure is
  `parameter`-shaped, look at global congestion*, because every per-net test can
  pass on a board no router can finish.

So both guards keep their **evidence requirement** — you must supply the
before/after render — and replace the **threshold refusal** with a
**disposition refusal**.

The distinction is the whole result. A gate that scores the board refused a
perfect repair on piantor. A gate that asks *"did anybody decide?"* cannot make
that mistake: the perfect repair records why the read looks poor and proceeds.
So the driver never judges the numbers, and never lets a run in run 15's shape
close out unacknowledged either.

Merely REPORTING was tried first and was wrong — it left run 15's exact arm
closing out clean (`exit 0`) with the evidence on screen and nothing asked of
it. Advisory is what the close-out already had too much of.

  * P-close refuses a poor read until `--waive congestion:<reason>` or a lever
    is pulled.
  * L4 refuses `--shape parameter` on a poor read until `--accept-congestion
    "<reason>"` — a dedicated flag, deliberately NOT part of
    `--accept-residue`, whose vocabulary belongs to the L2 placement gate; a
    waiver spanning two gates waives the one that was working.

## Side findings

- **`place_reconstruct` does not complete on the larger boards.** urchin reached
  8/13 reseats at t=2213 s and hit a 3000 s deadline; piantor took **735 s to
  seat one part** of five. Both exit 7. This is the run-9 non-termination shape
  on 87–103-part boards, not 217.
- **Its deadline behaviour is correct and easy to miss**: the output path is NOT
  written, and the partial board is left at `<output>.staging.kicad_pcb`, said
  only on stderr. My harness first read this as "no board produced" — it is not.
- **The ratio is knob-dependent.** `halo` is a penalty whose value depends on
  `halo_coef`/`halo_weight`/`clearance`. `render_placement` uses 0.25/2.0;
  `pose_score.make_state` uses 0.15/… and discards halo. Any ratio measured here
  would have been valid only for `render_placement`'s weights — one more reason
  a single fitted number was never going to travel.
