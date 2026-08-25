# Pre-registration — #590 confirmation, sets 21–27

**Written and committed BEFORE the sweep ran.** Its purpose is to remove the
degree of freedom that produced the sets 11–20 headline: there, the corpus-wide
result was p=0.15, and it became p=0.046 only after I chose (defensibly, but
*after seeing the data*) to drop 22 replay-biased boards. That sequence is how a
non-reproducible finding is manufactured. Everything below is fixed in advance;
if the run comes back short of these thresholds, the answer is no.

## Question

Should `KICAD_HISTORY_*` be armed by DEFAULT at the settings that won sets 1–10
and sets 11–20?

```
KICAD_HISTORY_COST=0.1          KICAD_HISTORY_CAP=0.5
KICAD_HISTORY_RIP_WEIGHT=1.0    KICAD_HISTORY_BLOCKED_WEIGHT=0.25
KICAD_HISTORY_ESCALATE=0
```

## Design

- **Corpus**: sets 21–27, 99 boards. Never used in this campaign — sets 1–10 and
  11–20 are both spent.
- **Arms**: exactly TWO. `base2127` (no overrides) and `hist2127` (the settings
  above). No dose ladder, no ingredient arms: the multiplicity that makes a
  0.046 unremarkable is itself a thing to eliminate. Those questions were
  answered on the earlier waves and are not re-opened here.
- **Excluded boards, fixed now**: `core64_logic` (#625, never terminates). No
  monster-board exclusions are expected to be needed; if the plan shows a
  wall-clock floor above ~6 h, the boards dropped will be recorded HERE by
  amending this file BEFORE launch, never after.

## Primary endpoint

`rank_arms.py <sweep.json> --drop-rescue-clean`, i.e.

- verdict = `nets_incomplete` (unrouted + connectivity-issue nets), summed over
  boards paired on step count + final board + gradeable net census;
- boards excluded where a recorded rescue step names nets AND the control leaves
  <2 nets incomplete (RUNBOOK rule 5). 24 of the 99 boards carry a rescue step;
  which of those clear the <2 bar is not knowable until the run, but the RULE is
  fixed and the board list is in `retry_shape_2127.json`.

## Decision rule

Arming by default is justified **only if BOTH hold** on the primary endpoint:

1. verdict improves by **≥3%** relative to the control, AND
2. one-sided sign test over decisive boards **p < 0.05**.

Plus neither of these safety conditions may fail:

3. DRC must not worsen by more than 2% of the control's total;
4. no single board may lose more than 5 nets.

If (1) or (2) fails → **no default**. The fallback is the congestion-gated
arming (contest-rate detector), for which the evidence is already stronger and
which this run does not test.

## Committed in advance

- Secondary metrics (pads, speed, memory, diff pairs) are REPORTED but cannot
  rescue a failed primary. They are not decision inputs.
- No other subset, cut, or threshold will be introduced after seeing results. If
  something surprising turns up it is a hypothesis for a further run, stated as
  such, not folded into this verdict.
- The result is reported whichever way it lands, including if it refutes the
  two prior waves.
