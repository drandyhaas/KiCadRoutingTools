# Verifier prompts

Fan these out **in one response**, each handed only its slice. **Never hand a
verifier the raw `.kicad_pcb`** — it is 100k+ lines of s-expressions and the
verifier will skim it and guess.

Every verifier ends with exactly one line:

```
VERDICT=PASS:lens=<lens>
VERDICT=FAIL:lens=<lens>;finding=<one line>;evidence=<path#json-pointer|path@x,y>;route=<step>
```

**And ALSO writes that line to disk, as a durable record** (run-23). The reply
is the delivery channel and the driver asks for it that way, but a reply is a
notification and notifications get lost: run 23's connectivity and drc lens
verdicts never reached the parent — it waited on them, was nudged, and had to
re-derive both lenses inline, which the copy on disk would have made
unnecessary. Put it beside the round's other artifacts, named for the lens. A
verdict you cannot produce afterwards is a finding about the verifier, not a
reason to assume PASS.

`evidence=` must point into the round's own files — `wk/place.log#JSON_SUMMARY.
crossings_after`, `wk/intent.json#/violations/3`, `wk/view/board_F.png@112.4,63.1`.
**A verifier that cannot fill it has not verified anything.**

`VERDICT=`, not `RESULT=`: the GUI parses the last `RESULT=` line in a reply as
the plan JSON.

## The round bundle

Build once per turn, then slice it:

```
wk/turn<N>/
  locks.json  locks.log          # A -- lock advisor
  place.log                      # B -- place_optimize JSON_SUMMARY (tee'd)
  view/*.png  view.log           # C -- renders + their --json
  loop_round*.json               # D -- loop sidecars
  intent.json  intent_result.json  # E -- check_floorplan
  route.log  drc.txt  conn.txt   # F -- routing + the authoritative checkers
  list_groups.txt                # G
  score.json  ledger.jsonl       # H -- board_score.py + the Step 9 ledger
```

## The nine lenses

Lenses 1–6 grade the **placement**. Lenses 7–9 grade the **routed board** — the
thing actually being handed over. Run 7–9 on every board you are about to call
done; skipping them is how a board went out at 39/44 nets with 762 DRC errors and
a clean-looking report.

**1. `intent`** — given `intent.json`, `intent_result.json`, the front panel.
> Does this board honour the declared floorplan? Report every `violations[]`
> entry with its `measured` vs `expected`. Check `rules_run` and
> `rules_skipped`: if `rules_run` is 0 this is a vacuous pass and you must FAIL
> it. You may not conclude anything about DRC or connectivity.

**2. `legality`** — given `view.log`'s JSON, both side panels.
> Did legality regress? Compare `overlap_area` and `oob_count`/`oob_amount`
> against the seed's. Absolute zero is NOT the test -- a shipped corpus board legitimately
> ships 81 of 82 parts in courtyard violation. **Never use `oob_area`**: it is
> measured against a bounding-box inset, so a part inside a cutout scores 0.0.
> You may not conclude that a specific pair collides — that is `check_drc`'s job.

**3. `delta`** — given `place.log`'s JSON, `locks.json`, the loop sidecars'
`moved[]`, and the delta render.
> Did the numbers improve, and did anything move that should not have?
> `hpwl_after <= hpwl_before` AND PAD-PAD conflicts did not rise AND the
> assembly channel's blocking pairs did not rise, or the result is
> discarded. `crossings` is REPORTED, never gated: it correlates POSITIVELY
> with distance-to-truth (r = +0.78 -- that is DISTANCE, not routed
> `blocking`; no predictor here has been correlated with `blocking`, see
> `docs/placement-predictors.md`), so a verifier failing a placement on
> it rejects exactly the correct homecomings. Then intersect `moved[].reference` with the advisor's
> high-confidence findings and with `locked_refs`: any overlap is a FAIL.
> **Do not judge by how much moved** — "lots moved, looks broken" and "barely
> moved, looks safe" are both wrong.

**4. `blocks`** — given `list_groups.txt`, the `blocks`/`block_parts` keys, any
zoom panels.
> Is the block set the right scope, and did grouping do anything at all?
> `blocks: 0` with a `--group-by` passed means the source found nothing. Check
> that no routing command silently acquired a `--group` the user did not ask for.

**5. `routing-feedback`** — given `route.log`'s JSON, the loop sidecars'
metrics, the `--focus` panels.
> Are these failures floorplan-shaped, placement-detail-shaped, or
> parameter-shaped? Use `blockers` (empty ⇒ nothing to move ⇒ not placement),
> the spatial clustering of the focus panels, and the pin counts of the failing
> refs. **The default answer is parameters.** You may not conclude that a
> specific move will fix it.

**6. `coverage`** — given the Step 5b net ledger, the route `--nets` list, the
plane nets, and any placement `--ignore-nets`.
> Is every routable net claimed exactly once? Does `--ignore-nets` equal the
> plane-net set? A placement step claims no nets and must not appear in the
> partition.

**7. `connectivity`** — given `conn.txt`, `score.json`, the route `--nets` lists.
> Is every net actually joined? Read `score.json#/components/unrouted/count` and
> `/components/broken/count` and reconcile them against `conn.txt`. A net that is
> unrouted because it was *excluded* from every route step and never poured is a
> **FAIL**, not an accepted shortfall — name it and say which step should have
> claimed it. The router's own `failures` tally is not evidence here: it comes
> from the thing being graded. You may not conclude anything about clearance.

**8. `drc`** — given `drc.txt`, `score.json`, the routed clearance.
> Is this board manufacturable? Read `score.json#/components/drc/count` and
> `/components/undersized/count`, and check `/components/drc/graded_at` is the
> floor the board was actually routed to — grading stricter than the route
> manufactures phantom violations, grading looser hides real ones. **`undersized`
> is the one that hides:** `check_drc` defaults its size floors to the FAB
> minimum, so a via meeting the fab and violating the board's own tighter spec
> grades clean unless the spec numbers were passed. If the spec states sizes and
> `board_score.py` was called without them, FAIL on that alone.

**9. `spec`** — given `score.json`, the requirements document, `intent.json`.
> Does the board meet what was *asked for*, not just what is manufacturable?
> Walk every requirement with a number in it — track widths, pair widths and
> gaps, length rules, impedance, clearances, connector positions — and find the
> measurement that confirms or refutes it. Report the ones with **no measurement
> at all** as findings: `score.json#/ungraded` lists components nothing examined,
> and *ungraded is not passed*. If a requirement is **geometrically
> unsatisfiable** as written, say so with the measurement that proves it rather
> than reporting it as a routing failure.

## Adversarial completeness

On anything high-stakes, add one more whose only job is to disprove the result:

> Actively try to PROVE this placement is not ready to route. Find a part that
> moved and should not have, a metric that regressed, an intent rule that did
> not run, a render being used as evidence for something only a number can
> establish, or a claim in the report with no `evidence=` behind it. **Report
> the single most damning finding.**

## Resolving disagreement

- `route=` names where the finding goes back to, so routing is mechanical.
- The gate is met when every lens PASSES **or every finding is dispositioned in
  writing** — a disposition lives in the report, not in your head.
- A machine check failing outranks any verifier's judgement.
- If two sources disagree, believe the JSON.

## A FAIL re-enters the loop — it is not a footnote on a finished board

This is the half that makes the fan-out a **gate** rather than a report. When a
lens fails, `blocking` was not really zero:

```
VERDICT=FAIL:lens=drc;finding=8 vias below the 0.6 mm spec on B.Cu;
  evidence=wk/score.json#/components/undersized/by_type/via-size;route=Step 2
```

1. **Record the verdict with `converge.py record --lens`**, passing the
   `VERDICT=` line verbatim — as the verifier returned it, or from its copy on
   disk, never as one you remember — (repeatable; stored raw as
   `entry["lenses"]`). It
   refuses at write time anything that is not a `VERDICT=(PASS|FAIL):lens=…`
   line, so a malformed verdict stays visible instead of being normalised into
   something that reads like a pass — and `--final` refuses without all three
   routed-board lenses, because `blocking == 0` and "every lens passes" are two
   different claims and only the first had a number.

   *(This used to say the record schema had no verdict field and to put it in
   free-text `--lever`. That was documenting a gap, not a design constraint —
   the same gap `--render-json` was added to close, in the same words.)*
2. **Spend another iteration at the step named in `route=`**, if budget remains.
3. If the budget is exhausted, stop on **condition 2** and report the finding as
   an outstanding blocker with its measurement — never as a passing board with a
   note attached.

Two failure modes to refuse by name:

- **Do not re-word a FAIL into a caveat.** "Routing is complete, with some DRC
  warnings" describes a board that did not pass. Either fix it, or report it as
  not done.
- **Do not accept a lens that passed vacuously.** A lens whose inputs were empty
  has not verified anything; that is why lens 1 must FAIL on `rules_run == 0` and
  lens 9 must report `ungraded` as a finding.

**Stop condition 4 is the exception, and it is the only one.** A requirement that
is geometrically unsatisfiable does not get more iterations — it gets a
measurement and a written finding about the requirement.

**When the `Agent` tool is unavailable** — the GUI headless path allows only
`Read,Glob,Grep,Bash,WebSearch` — run the same lenses yourself, in the same
order, on the same inputs, tag each `mode=inline`, and **say in the report that
verification was single-agent**. A run must never look like a fan-out happened
when it did not.


## Pre-registration rule: arm falsifiers in BOTH directions (run-4 B12)

A watcher pre-registering predictions must give, per prediction, the
number/band that would CONFIRM it and the one that would REFUTE it — in
both directions. Run 3's prereg armed a placement-erosion falsifier only
upward ("home rises >= 3"), so the actual 26->15 DOWNWARD erosion on the
rejected quenches never tripped it and was caught only by the close-out
grading. This applies to regressions and to improvement claims alike: an
improvement band without a floor is a prediction that cannot lose.

## The boundary verifier (run-5, SKILL 9.4b): blocking, per accepted iteration

Distinct from the close-out lenses above: it runs DURING the loop, after
every accepted iteration's ledger entry, and a FAIL blocks the next step.
It verifies the RECORD, never the routing — hand it the slices only.

Prompt skeleton (fill the <>):

> You are a blocking boundary verifier. You receive: (1) one ledger JSONL
> entry, (2) the score JSON it attaches, (3) the render JSON(s) its
> `[read:]` tags name, (4) the operator's one-paragraph claim. You never
> see the board. Check, in order: [1] the score's `board_sha` equals the
> entry's `result_sha` (a mismatch must be lever-explained in the entry);
> [2] every claimed `[read: panel]` exists in the named render JSON and
> every quoted checklist value matches it; [3] the entry's timestamp is
> after its artifacts' mtimes and before any later artifact you were
> given (a cluster of near-identical stamps is batch-written history —
> FAIL); [4] every number in the claim traces to a field in an artifact
> you hold; failing nets must be NAMED, not counted. Reply with exactly
> one line: VERDICT=PASS or
> VERDICT=FAIL:check=<1-4>;finding=<one line>;evidence=<path#pointer>.
> Report the single most damning finding.

Rules of engagement, mirrored from 9.4b:

- Accepted iterations and close-out only; rejected entries wait for the
  close-out audit.
- The close-out instance additionally walks the whole ledger: monotone
  timestamps end to end, and the `--final` entry's stop condition quoted
  against its own score payload.
- A FAIL is remediated (fix the entry, re-read the artifact, or re-run
  the step) and re-verified before the loop continues — it is never
  re-worded into a caveat.
- When the `Agent` tool is unavailable, run the same checks yourself,
  tag `mode=inline`, and say so in the report — same as the lens rule.

## Check 5 addendum (run-6): assembly-clean at placement boundaries

At any placement-phase or fix-loop boundary the verifier's input set grows
by the fresh `check_assembly --json` output (and the render JSON's
`checklist.b_body_overlap_pairs`). FAIL unless: `blocking == 0`;
`b_body_overlap_pairs` is `[]`; every `new_advisory_pairs` entry (the
--baseline delta -- the loop currency) is fixed or dispositioned in the
ledger entry. An operator claim of "placement done" with no check_assembly
JSON attached is a FAIL by itself. The verdict line's check id is 5.
