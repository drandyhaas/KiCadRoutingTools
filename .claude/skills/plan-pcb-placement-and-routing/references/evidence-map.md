# Evidence map: produce X → read key Y → do Z

The artifacts the placement tools emit, the exact JSON key to read from each, and
the decision that key drives. Read this before quoting any number.

"**Every** artifact" is what this line used to claim, and it was not true: there
was no row for `py_tools/board_context.py` (E3) and none for `brief_coverage`
(E2), both of which a verifier is REQUIRED to read. If you need a key that is
not here, run the tool and read its output — then add the row.

**The governing rule: never read a picture on its own.** Every render is paired
with a number that either confirms or contradicts it, and the number wins.

Keys are literal. If a key below is not in the output you are looking at, you are
looking at the wrong artifact — do not substitute a similar-looking one.

---

## A. Lock advisor — `place_optimize.py BOARD --suggest-locks --suggest-locks-json wk/locks.json`

Writes no board. Run it **before** the first placement run, and read the reasons.

| key | decision |
|---|---|
| `JSON_SUMMARY.unlocked_high` | **The gate.** Must be `0` before placement runs. Re-run with your `--lock` list until it is, or name each finding you are leaving free and why |
| `findings[].evidence.npth_pads > 0` and `plated_pads == 0` | A structural mounting hole. It has **no net, so no airwire at all** — nothing but the halo term decides where the optimizer slides it. Always lock |
| `findings[].evidence.outside_amount_mm > 0` | The body legitimately leaves the outline (card edge, USB shell, HAT header). Lock it **and** add it to the intent's `edge_connectors` |
| `findings[].evidence.edge_clearance_mm` | Measured, exact. Use it as the intent's own limit rather than inventing one |
| `findings[].evidence.connected_pins` | `0` → invisible to the cost. `>= 40` → see `advisories` |
| `findings[].evidence.side` | Which face — decides whether the verify render needs `--per-side` |
| `advisories[].kind == "high_pin_count"` | `place_optimize` has **no** `--max-target-pins` guard (`place_route_loop` does). Either lock these or use the loop. Never let a ≥40-pin part move in a bare `place_optimize` run |
| `lock_argv` | Paste verbatim — it is already argv-shaped. Do not retype `lock_patterns` by hand |
| `locked_refs` | Refs the FILE already pins. Advice telling the user to move one of these is wrong |
| `findings_covered` vs `findings_high` | Round-trip proof your `--lock` list actually matched, rather than matching nothing |

A **quiet result is "nothing detected", not "nothing to lock"** — the lexical
rules (footprint name, reference prefix) miss house libraries entirely.

---

## B. `place_optimize.py IN OUT ... 2>&1 | tee wk/place.log`

The `JSON_SUMMARY` goes to **stdout only**; `tee` is what makes it citable.

| key | decision |
|---|---|
| `parts_moved` | `0` → nothing happened. Do not render a delta or claim improvement; widen `--max-displacement` or narrow `--lock` |
| `crossings_before` / `crossings_after` | **Report only -- NEVER a gate** (run-6: crossings correlates POSITIVELY with distance-to-truth, r = +0.78 -- that is distance, NOT routed `blocking`. #703 has since measured it against routed `blocking` too, where it FAILS its sign rule on the full sample (5 boards right, 1 wrong) and PASSES it (6/0) once optimizer-made placements are excluded -- so neither arm is the answer and the prohibition still rests on the distance measurement; see `docs/placement-predictors.md` -- so gating on it rejects correct homecomings). Gate on hpwl + PAD-PAD + the assembly channel. Any increase → route from the original board |
| `hpwl_before` / `hpwl_after` | **Discard gate.** Same |
| `overlap_area` | Courtyard overlap of the output. Must not increase. **Absolute zero is the wrong test** -- one shipped, legitimately placed corpus board carries 81 of its 82 parts in courtyard violation |
| `oob_count` | Parts leaving the outline. Must not increase, and every one should be in the intent's `edge_connectors` |
| `oob_amount` | Severity. Count flat but amount up = an already-overhanging part got pushed further out |
| `oob_area` | **Do not gate on this.** Measured against the bounding-box inset, so a part sitting entirely inside a **cutout** scores `0.0` |
| `cost_before` / `cost_after`, `airwire_length_*` | **Weighted.** Comparable only within one run. A big `cost` drop with flat crossings/hpwl means halo and edge moved: cosmetic spreading, not routability. Do not report it as an improvement |
| `blocks` / `block_parts` | Proof `--group-by` did anything. `blocks: 0` → drop the flag |

---

## C. `render_placement.py ... --json-out wk/view.json --ignore-nets <same set as B> -o wk/view/`

Always pass `-o`. Without it the tool writes `<board>_placement.png` **next to
the board**.

`--json-out`, not a bare `--json`: the keys below are a document to read back,
and a document is a file. This section is a KEY TABLE, so it does **not** ask
for `--quiet` — the tables elsewhere that ask you to LOOK first do, because
that is where reading the keys before the picture costs something.

**Pass the same `--ignore-nets` you gave `place_optimize`, or the re-measurement
below compares two different net sets and always "fails".** B's numbers exclude
the plane nets; this tool's do not unless told to. On one run, GND
alone moved the same board from 53 crossings to 116 — which reads exactly like a
corrupted write and is not one.

| key | decision |
|---|---|
| `metrics.crossings`, `metrics.hpwl` | **The re-measurement channel.** Run this on the *written output board*; with the same `--ignore-nets` it must reproduce B's `*_after` **exactly** (53 / 587.3150 on both sides, not "about the same"). A mismatch means something was lost between the objective and the file — this is the self-assertion ban with teeth |
| `metrics.overlap_area`, `metrics.oob_*` | Independent read of B's legality numbers off the file |
| `metrics.halo`, `metrics.edge` | Cost decomposition. `total` improved while `length` and `crossings` did not ⇒ the win was halo/edge only ⇒ not a routability win |
| `no_outline: true` | The oob metrics are **unavailable**, not zero. Say "unavailable" — never "0 parts off board" |
| `unplaced: true` | The placement CLIs would exit 3. Report and stop |
| `moved` | Cross-check against B's `parts_moved`. A mismatch means you rendered the wrong `--before`/board pair |
| `failed_nets`, `blocker_nets` | Which nets the picture colours. Non-empty only when `--summary-json` was given — this is what ties a render to a specific route log |
| `panels[].path` | The exact PNG to `Read`, and what goes in a verifier's `evidence=` |
| `panels[].view` | The world rect that panel covers. **A finding at a coordinate outside it was not visible in that panel** — reject the claim |

---

## D. `wk/loop_round{N}.json` — the `place_route_loop` sidecars

Nobody reads these today, and they are where the loop's actual history lives.
`place_route_loop` prints **no** `JSON_SUMMARY` for a normal run.

| key | decision |
|---|---|
| `accepted` | Only accepted rounds are progress. Never quote a rejected round's board as the result |
| `parent` | **The board this round derived from — the last ACCEPTED board, not N−1.** Use it as `--before`. Using N−1 renders a delta that never existed |
| `screened` | `true` → the ratsnest screen skipped the routing run and `metrics` is empty. Never report failures for a screened round |
| `metrics.failures` | **The ratchet.** Unchanged across two consecutive non-accepted rounds → floorplan-limited → stop, do not raise `--rounds` |
| `metrics.failed_nets` | Feed to `--ratsnest-nets` and to `/diagnose-routing-failures` |
| `metrics.blockers` | The other half of the move-candidate set. **Empty blockers with non-empty failures means there is nothing to move** — not a placement problem |
| `metrics.iterations` | `better()` requires `< best × 0.95`, so a round improving effort by under 5% at equal failures is **rejected by construction**. That is the tool working, not evidence the placement is wrong |
| `metrics.vias` | The cost side of the trade. Report it; do not hide a failures win paid for in vias |
| `metrics.pad_pairs_connected` / `_total` | A round that lowers `failures` while lowering the connected ratio is not progress |
| `metrics.ratsnest_crossings` / `_hpwl` | The proxy beside the router's verdict. Proxy improving while `failures` is flat ⇒ proxy exhausted, stop |
| `metrics.ratsnest_length` | **Weighted, report-only.** Never compare across rounds |
| `targets` | Which refs the round was allowed to move. Any `must_lock` ref here → tighten `--lock` and re-run |
| `groups` | What blocks pulled extra parts in. A 40-part block here means the round moved far more than you targeted |
| `moved[].reference` / `.from` / `.to` | Exact per-part deltas. Intersect with the lock advisor's high-confidence findings; a non-empty intersection is a blocker, not a note |

---

## E1. `check_floorplan.py BOARD --intent ... [--health]` — the stdout `JSON_SUMMARY`

| key | decision |
|---|---|
| exit code | `0` clean · `2` argparse or malformed intent · `3` board state / untrustworthy outline · `4` **violations found** |
| `pass` | The gate |
| `violations` | A **COUNT** here. In the E2 document the same name is the LIST. Quote the one you are looking at |
| `cutouts` / `edge_contours` | **Counts** here; the geometry is `outline.*` in E2 |
| `violations_by_rule` | Which rule fired, and how often — names the thing to fix |
| `rules_run` / `rules_skipped` | **Anti-vacuity.** `0 violations` with `rules_run: 0` means nothing was checked. Quote both |
| `rules_dark_undispositioned` | #959. The rules P1 refuses on: dark (or armed with a withheld budget key nobody answered), applicable by a board fact, gating, and answered by nothing in `dispositions`. `null` means no roster was built, which is different from an empty list |
| `ledger_status` / `carried_facts` / `unmeasured_facts` / `derived_default_clauses` | #959. The declaration ledger, counted by status. `carried_facts` and `unmeasured_facts` are declared facts that were **NOT physically checked**, so never read `complete` without them. `derived_default_clauses` are numbers this code chose, not the author |
| `contradictions` / `contradiction_ids` / `contradictions_undispositioned` | #959. Two declared or recorded channels (the brief, `mechanical.json`, the outline) that disagree about one ref. P1 refuses each undispositioned id |
| `blocks_resolved` vs `blocks` | A block resolving to nothing is reported as an error, but check this too |
| `state_*` | `duplicate_fraction`, `spread_ratio`, `outside_fraction`, `partially_unplaced` — the signals behind the exit-3 verdict, and the only place they are emitted at all. `partially_unplaced` is the case exit 3 **hides**: a netlist re-import dropped a few new parts at the origin on an otherwise-placed board, and "place these specific refs" is the right instruction |
| `health_block_displacement_max_mm`, `health_blocks_displaced` | How far a block sits from what it connects to. The 80 mm-magnetics failure mode a nudge cannot fix |
| `health_bus_foreign_crossings` | What crosses a declared bus corridor |
| `health_signals_skipped` | A **COUNT** of signals that did not run (`len(health.skipped)`) — the per-signal REASONS are `health.skipped` in the E2 document, not here. `blocked_cell_share` always needs a route first |

---

## E2. `check_floorplan.py BOARD --intent ... --json wk/floorplan.json` — the document

The SAME run writes two artifacts and they are not the same shape. `state` is a nested object here and a `state_*` prefix in E1; `violations` is a list here and a count there. Per the rule at the top of this page: read the keys of the artifact in front of you, and do not substitute the other spelling.

| key | decision |
|---|---|
| `violations[].measured` / `.expected` | The falsifiable number. This is what goes in a verifier's `evidence=`. E1 has only the count |
| `state.stacked_refs` / `state.stacked_suspect_refs` | Co-located refs, and the subset `partially_unplaced` is actually decided on. They differ **by design** — parts the far side of the board cannot reach, and marker classes (fiducial / mount_hole / testpoint) that share a coordinate deliberately, are excluded. Quote the SUSPECT list; a non-empty `stacked_refs` beside `partially_unplaced: false` is normal, not a broken check. Two traps: a **drilled** part counts on both sides, so it is never excluded by side; and being `(locked yes)` is **not** an excuse, because this toolchain stamps its own locks |
| `outline.cutouts` / `.edge_contours` / `.simple_rectangle` | What the parts must avoid. `edge_contours` are clearance-bearing and **invisible in every render**. E1 carries the first two as bare counts and `simple_rectangle` not at all |
| `health.skipped` | The signals that did not run, each with its REASON. E1's `health_signals_skipped` is the length of this |
| `brief_coverage.uncovered` / `.abstained` / `.drifted` / `.complete` | What the declared design brief (#711) did and did not reach. `verifier-prompts.md` makes lens 1 FAIL on these, and `--require-brief-coverage` refuses a grade with nothing declared behind it — so a verifier handed only this page could not satisfy the lens it was asked to run until this row existed |
| `legality`, `edge_seating`, `decap_pin_evidence` | The per-rule evidence behind `violations`. Present only when the intent declares the rule |

---

## E3. `py_tools/board_context.py BOARD --json -o wk/context.json` — the per-part sheet (#891)

Boundary criteria 1, 2 and 4 are decided on these keys and this page had no row for the tool at all, so a verifier handed only this file could not answer the questions it was given.

| key | decision |
|---|---|
| `pin_order.rows[].verdict` / `.span_mm` | Per interface: does the connector's pin order agree with what it mates to, and over what span. `.inversions` counts the crossings; `.scope` and `.nets` say what was compared. `pin_order.error` is set when nothing could be compared — read it before quoting a clean verdict |
| `parts[].pads_by_face` | Which face of the part its pads escape toward. An empty `{}` means the body model could not decide, NOT that there are no pads |
| `parts[].partners` | What this part actually connects to, ranked. An empty list is a part with no signal partners — normal for a decap, a finding for a bridge IC |
| `parts[].body_mm` / `.body_source` | The body extent and WHERE IT CAME FROM. `body_mm: null` means unmodelled, so every clearance conclusion drawn from it is a guess; `body_source` is how you tell |
| `parts[].part_class` / `.part_class_confidence` / `.role` / `.serves` | What the part is for. Low confidence is a reason to look, not to overrule |
| `floors.clearance` / `.track_width` | The board's OWN floors — what every other instrument must be graded at, never a round number you picked |
| `sources` | Which derivation produced each of the above. The anti-vacuity row: a key computed from nothing still has a value |

## F. Routing summary — `route.py`'s `JSON_SUMMARY`, in `wk/route.log`

The feedback edge back into placement.

| key | decision |
|---|---|
| `failed_single`, `failed_multipoint[].net_name` | The failure set → Step 9's classifier, and `--summary-json` for the render |
| `blockers[].blocked_by[].net` | Which routed nets wall off each failure → the move-anchor set |
| `pad_pairs_connected` / `pad_pairs_total` | The honest completion number for comparing two placements |
| `total_iterations` | Effort. A placement that halves iterations at equal completion is a real win |
| `total_vias` | The cost side of the same trade |
| `power_trace_ampacity[].bottleneck_width_mm` | **Did the width you asked for actually happen?** `--power-nets-widths` degrades quietly: a wide tap that will not fit is re-routed at the layer default (#72's neckdown retry), and on a dense board that fallback can take the *whole* run, not just the pad. Measured, `--power-nets-widths 0.8` on a pair produced **1.30 mm of 0.8 mm copper out of 41 mm** — three of the four nets got none at all. Compare this key against what you asked for, every time; the routed board is DRC-clean either way |
| `power_widths.<net>.under_mm` / `under_share` | **How MUCH of it did not happen** (#1033). The bottleneck above is one number; this is the length. Measured on the board the run wrote, per `--power-nets-widths` net: `requested_mm`, `length_mm`, `under_mm`, `min_mm`, `by_width_mm`. Run 32 asked +3V3 for 0.3 and shipped 34% of its length at the 0.127 signal width with the bottleneck reading the same as a 1 mm neck would. Each narrowing SITE is also in `design_rules.narrowed` (`power neck-down (long trunk)`, `power short edge`, `fine-pitch tap`, `oracle reconnect`, `net rescue`). `power_widths_measured_on` says which copper was measured (the written board on the CLI; the change-set on the GUI, before any post-apply oracle leg). The grader twin is `board_score --net-min-widths` `components.net_widths.nets.<net>.length_under_mm` |
| `min_clearance_used` | The floor the run actually reached, which is **not** what you asked for. Below your netclass means the gap-rescue stepped down toward the fab floor. Measured: nominal 0.16, `min_clearance_used` 0.127, and **25% of all copper (180 of 710 mm) ended up at 0.127** — under the board's own 0.15 minimum. The `.kicad_pro` writeback then clamps the DRC floor to the routed value, so **KiCad grades it clean**. This key is the only place the step-down is visible |
| `rescue.unchanged` | Nets the rescue pass could not improve. Distinguishes "nearly made it" from "never had a path" |

### Routed length: half of it is already built, and the half that is not will bite you

**Matching IS supported — use it.** `route.py` / `route_diff.py` take
`--length-match-group NETS --length-match-tolerance MM`, `length_matching.py`
measures each net with `net_queries.calculate_route_length` and prints
`WARNING: <net> is X mm SHORT of the group target`, and
`/review-routed-board` Step 2 grades the spread. A spec clause like *"XTAL legs
symmetric to within 1 mm"* or *"QSPI intra-group skew <= 5 mm"* is a
`--length-match-group`, not something to hand-roll.

**An ABSOLUTE cap is not expressible anywhere.** No `check_*.py` says "this net
must be under N mm" or "this net must have 0 vias". (`check_orthonormal
--max-len` is per-segment non-orthonormality; `check_impedance
--min-void-length` is a void run.) So `QSPI <= 15 mm pad-to-pad, 0 vias` and
`XTAL <= 10 mm/leg` have to be measured from `pcb.segments` by hand — do it, and
say plainly that you did.

One routed board graded **`check_floorplan` PASS** and 86%
connected while breaking **19** such limits, including a QSPI net at 32.6 mm
against a 15 mm HARD budget and crystal legs 6.03 mm apart against 1 mm. Green
on the KRT gates is not green on the spec.

### The DRC writeback RATCHETS — check it between chain steps

`route.py` clamps the sibling `.kicad_pro` down to what the run actually
achieved, and **that includes `track_width`, which the next step reads back as
its nominal**. Measured across one chain:

| project | `Default.clearance` | `Default.track_width` |
|---|---|---|
| `placed.kicad_pro` (authored) | 0.16 | 0.16 |
| after the signal route | **0.127** | **0.127** |
| after the plane pour | 0.127 | 0.127 |
| after the repair route | 0.127 | 0.127 |

One rescue at 0.127 became the default width for every later step, and 25% of
the final copper (180 of 710 mm) sits at 0.127 — under the board's own 0.15 HARD
minimum. Because the project now *says* 0.127, `check_drc` and KiCad both grade
it clean; grading the same board at the pre-clamp 0.16 gives **34** violations.
`min_clearance_used` and `Default.track_width` are the two places this is
visible. Diff the `.kicad_pro` between steps whenever the spec has a width floor.

---

## G. Blocks

| output | decision |
|---|---|
| `route.py --list-groups`: `parts=`, `touching=`, `internal=` | The routing-scope decision. `internal ≈ 0` ⇒ only `touching` is meaningful (always true of `decap`) |
| `render_placement.py --list-groups`: `parts=`, `front=`, `back=` | A block with parts on both faces cannot be reviewed in one panel |

---

## H. The board score — `scripts/board_score.py BOARD --json wk/score.json`

The only number that decides better-from-worse in the Step 9 loop, and the only
one produced by something **other than the tool being graded**. Everything else
on this page describes what a step *claims*; this describes what the board *is*.

| key | decision |
|---|---|
| `blocking` | **must reach 0 before the board is deliverable.** `unrouted + broken + drc + undersized + floorplan + assembly + impedance + length + net_widths` -- NINE, re-derived from `board_score.py`'s own `parts` dict by `tests/test_918_gate_wording.py` |
| `blocking_by.<component>` | names WHERE the blocking sits. **The largest entry is NOT automatically the lever** -- that rule wrecked a run. Choose by the connectivity-first ladder (unrouted -> broken -> widths -> floorplan -> drc); an entry's size ranks within a rung, never across rungs |
| `quality` = `{vias, copper_mm, segments}` | tie-break **only** at `blocking == 0`. Comparing it earlier lets a router trade a disconnected net for a lower via count |
| `ungraded` | components nothing examined (no `--intent`, no `--impedance-nets`, no `--length-groups`). **Report as unexamined, never as clean** |
| `unknown` | a component that was asked for and could not run. `blocking` is `null`, not 0 — the loop must not stop here |
| `components.assembly.buildable` / `.verdict` | **`check_assembly`'s OWN verdict, over all FIVE `not_buildable` conjuncts.** Gate on this, NOT `blocking == 0` — that scalar is one of the five, so a board unbuildable through a coincident-origin stack or a containment reads 0 and is still NOT BUILDABLE (#918). `buildable: false` is not deliverable at any `blocking` |
| `components.assembly.conjuncts` / `.count_basis` | the five conjunct counts as published, and which of them `count` was built from. `courtyard_blocking_gating` is **`null` here by construction** — board_score passes no `--baseline`, so the fifth conjunct is structurally unarmed; `conjuncts_unmeasured` names it. Not measured is **not** measured clean |
| `placement` (copper-free laps, `--placement-terms`) | the five terms a placement lap can be RANKED by -- worst diff-pair span, crossed pin orders, cluster distance, plane-cut proxy, pad-area balance. **Report-only**: never in `blocking`, never in the exit code. Without it `quality` is `(0, 0.0, 0)` for every placement of every board and two laps cannot be ordered at all |
| `placement.terms.<t>.basis` | the POPULATION a term was measured over. When it moves between laps the term is **not judged** -- a total over a different population is not a larger or smaller version of the first (the argument `commensurability` makes about `blocking`). Measured over four laps of one board: a plane term's blocker set read 3 / 11 / 3 / 3 as parts were frozen, and its value 0.8 / 22.3 / 0.0 / 0.0 with it |
| `placement.vs_parent.verdict` | PARETO against the parent lap: `better` only when no measured term regressed. `mixed` names both sides and is **not** an improvement. There is no aggregate and no weight -- #694 |
| `components.drc.graded_at` | the clearance actually graded at. Confirm it is the routed floor; stricter invents violations, looser hides them |
| `components.drc.by_type` | `segment-segment`, `pad-segment`, … — clearance conflicts |
| `components.undersized.by_type` | `track-width`, `via-size`, `via-drill-size` — **sub-spec copper**, graded separately per floor. Note what this cannot see: diameter and drill are tested independently, so a via whose ring is zero (a 0.3/0.3 via) passes both — removing such vias earns no credit here; state it in the ledger |
| `components.floorplan.rules_run` / `.rules_skipped` | `0 violations` with `0 rules run` is a vacuous pass. Quote both |
| `connectivity_nets` | *which* nets failed. Same nets every iteration ⇒ parameters; different nets ⇒ congestion |
| exit code | `0` blocking is zero · `4` graded with blockers · `3` board state · `2` args · `1` crash |

**The size floors default to the FAB minimum, not the spec.** `check_drc` derives
them from the copper-layer count, so a via that clears the fab and violates the
board's own tighter spec **grades clean**. That is not hypothetical: 141 of 141
vias at 0.25 mm ⌀ passed against a 0.6 mm spec requirement. If the spec states
sizes, pass them:

```bash
python3 -X utf8 .claude/skills/plan-pcb-placement-and-routing/scripts/board_score.py board.kicad_pcb \
    --intent wk/floorplan.json \
    --min-track-width 0.15 --min-via-diameter 0.6 --min-via-drill 0.3
```

**Omit `--clearance`.** `check_drc` then reads the sibling `.kicad_pro`, which is
the floor the board was actually routed to (see F's writeback ratchet). Pass one
only when you know better than the board.

### `ledger.jsonl` — the Step 9 ledger (one `converge.py record` line per iteration)

| key | decision |
|---|---|
| `parent_sha` | the last **accepted** board (content hash; `step-back --to` checks it out). Resolve it for `render_placement --before`; using N−1 renders a delta that never existed |
| `lever` + `lever_argv` | `lever` is the one-line intent, `lever_argv` the reproducible command — `replay` refuses prose-only entries, and `record` refuses an argv carrying an MSYS2-rewritten net name (`C:/Program Files/Git/…`), which would replay as a vacuous pass. "tuned parameters" is not a lever. A verdict list has no field of its own: name it **in the `--lever` text** |
| `stop_condition` + `stop_reason` | #901. The condition is a TOKEN — `1 \| 2 \| 3 \| 4 \| DONE-EXHAUSTED \| STUCK \| BUDGET` — checked on EVERY record, not only beside a failing lens. The prose goes in `stop_reason`, written either as `--stop-reason` or after the token (`"3: five laps, no new copper"`) |
| `accepted` (`--rejected` at record time) | a rejected iteration is data — keeping it is what makes "five unchanged iterations" (stop-3) detectable |
| `score.blocking` | flat across FIVE RECORDED laps of that half — **accepted or rejected** — after the rip lever / finer grid / layer change ⇒ stop condition 3. This is what `converge verdict --flat` actually measures: `(blocking, quality)` lexicographic, per half, over the last N recorded laps, with `blocking: null` laps dropped as unjudged. It is NOT `unrouted` and `broken` read separately — quality counts too, so a lap that only moved vias is not a plateau. Run the tool rather than counting by eye |
| `kind` | `systemic` = budget went to the instrument; `status` warns when that share hits half |
| accepted `result_sha`s, in order | the frame list for `make_movie.py`. Reverted boards animate a change that was undone |

Full procedure: [`convergence.md`](convergence.md).

---

## I. `place_seed.py IN OUT --intent fp.json [--repair] [--reseat [REF ...]] [--dry-run]` — the stdout `JSON_SUMMARY`

Whatever the grade says, the board is written (a `--dry-run` writes nothing, and its report is the short form under `reason`), and the report below never changes the exit code: it says what was and was not measured about the declared edge connectors (#974), it does not gate. A `--repair` / `--reseat` summary also carries a top-level `complete` that is always `true` — that is not this one.

| key | decision |
|---|---|
| exit code | `0` passed its gate · `2` bad arguments or an intent that will not load · `3` refused before writing (no outline, copper on the board, already placed without `--force`, a pile that is all `(locked yes)`, `--repair` / `--reseat` on an unplaced board) — or, on a fresh seed, an outline that cannot be trusted for grading, AFTER the board was written and with no summary · `4` on a fresh seed: parts unseated, own grade errors, pad conflicts among the parts it placed (`pad_conflicts_seeded` — not the ones against a part it could not seat, #982), or hole conflicts it added; on `--repair` / `--reseat`: parts unrepairable, a re-seat that leaves parts unseated or is refused or not accepted, or own grade errors (pad and hole conflicts do not gate there) · `5` (#959) the zone PLAN is refused before anything is written: `floorplan.plan_check` found an area bound no arrangement can meet within a declared overlap budget, a part longer than its edge, or a real reference used as a glob that lands a part in two disjoint zones (`floorplan.PLAN_SEED_REFUSES`) -- fix the plan; `check_floorplan --intent PLAN --plan-only` checks it without seeding, and `--repair` / `--reseat` only report these · `1` on `--repair` / `--reseat`, an outline that cannot be trusted for grading is NOT caught: a traceback and no summary, with the board written by `--reseat` alone but not by `--repair` (which grades inside the repair, before any write). A dry run writes nothing and exits `0` or `4` on the same repair / re-seat causes, or `1` as above |
| `plan_findings` / `plan_measured` | Exit 5 only (#959), where `refused` is `plan_check` and `written` is false (on `--reseat`, `refused` is instead the list of refused re-seats), and the findings carry their measured areas, lengths and budgets. Every other `PLAN [...]` line is printed and the seed proceeds |
| `grade_errors` / `grade_errors_pinned` | Errors the run answers for, and errors on `(locked yes)` / `must_lock` parts, which are named and do NOT fail it. Absent on a dry run |
| `pad_conflicts_seeded` / `pad_conflicts_seeded_pairs` | Fresh seed only. **The gate's pad channel.** A pair with a part the seed MOVED on at least one side, and neither side a part it could not seat. The pairs are named, so a finding is actionable by ref |
| `pad_conflicts_unseated` / `pad_conflicts_unseated_pairs` | Fresh seed only. #982. Real copper, named, and NOT charged to the seed: a pair with a part the seed MOVED on one side and a part it could not seat on the other, the latter written at the pose it came in with. Whether anything lands there is incidental to this seed: measured, moving one unrelated connector by 0.386 mm took the count from 0 to 1. **Do not read a non-zero value as a placement regression, and do not read zero as a clean board**: the exit code is already 4 for the unseated part. Grade the severity with `check_assembly`, which reads these anywhere from a sub-0.2 mm graze on a board it still calls buildable up to a BLOCKING `pad_intersection` of over 1 mm². Seat the part and they go with it. A pair between TWO unseated parts is in `pad_conflicts_inherited` instead, since the seed moved neither |
| `pad_conflicts_inherited` | Fresh seed only. The board's own: neither side is a part this seed moved. Reported, never charged |
| `pad_conflicts_after` | The written board's total, which those three buckets PARTITION on a fresh seed. Check the arithmetic rather than trusting one bucket. A `--repair` / `--reseat` summary carries this key WITHOUT the three buckets, and a `--dry-run` carries neither -- its only pad count is `pad_conflicts_before`, the board as it arrived |
| `connector_requirements.complete` | `true` only when every declared edge-connector requirement was MEASURED and no band was dropped. About measurement, not passing: read the errors for that |
| `connector_requirements.reason` | Present only on the short form, which is exactly `complete: false` plus this: `dry-run` (the post-write grade the report reads did not run), or `connector_requirements failed: …` (the grade ran and set the exit code; only the report broke) |
| `connector_requirements.declared_refs` | The refs of the intent the grade was given, after any `--reseat` drop. Empty means nothing was declared or every declaration was dropped (`bands_dropped` then names them) — never that everything passed |
| `connector_requirements.errors_own` / `.errors_pinned` | The `edge_connector` errors on each side of the split the exit code used: `errors_own` non-empty means exit 4. Full violations, with `measured` / `expected` where the finding has them. A `legality` `oob_count` error a connector caused carries no ref: it is in `grade_errors`, not here |
| `connector_requirements.warnings` | `edge_connector` findings below error: a `connector_affinity` setback, or ANY connector finding once the intent's `severity` demotes the rule to `warn` — band, copper, wrong edge, along-edge, not on the board. A demoted finding reaches neither `errors_own` nor the exit code |
| `connector_requirements.unmeasured[].requirement` / `.reason` / `.graded_on` | What was NOT measured, per ref: `presence` (no graded part for the declaration), `overhang_body` (the band was graded on the old occupancy reading named in `graded_on`, because no drawn body could be measured or the entry names no edge), `pad_copper_outside` (copper past the outline went ungraded: a pad shape the edge grader cannot model, or an entry claiming an edge whose body could not be read — whatever its band says; certification is #961's, so a pad the grader approximates but models, such as a parsed custom polygon, counts as measured even where the row's copper `disposition` reads `unmeasured`), `center_on_edge` / `along_edge_band` (abstained, or never measured). A ref can be here AND in the errors |
| `connector_requirements.overhang_evidence[].overhang_basis` / `.overhang_disposition` / `.pad_copper_edge` | A projection of the #961 evidence row for each declared entry whose part is on the board: the band's `overhang_mm`, basis, limit and disposition, `body_measured`, and the copper's `disposition`, `outside_mm`, `certified`, `minimum_gap_mm` and `required_mm`, with COUNTS where the grade has lists: `n_findings` and `n_unmeasured` (this part's pads) and `n_rules_unmeasured` (the board-wide custom edge rules, the same on every row). The whole rows — body position and reasons, the per-pad lists — are E2's `edge_connector_evidence`. The copper clearance is evidence only and never makes the report incomplete |
| `connector_requirements.bands_dropped[].ref` / `.band_max_mm` | One row per ref for the declarations `--reseat` set aside for every edge-claiming ref in its scope — whether or not that ref then moved, or the pass was accepted: nothing about them was graded, at any pose. `band_max_mm` `0.0` can also mean the entry declared no max |
| `edge_floor_fallback` | #975. Per ref, a declared edge connector this run seated with pad copper inside the board-edge floor because no pose the seat ladder tried at its rotation clears it — that seat moved inward, and later rungs (where the ladder has any) only when the shortfall faces a side moving inward cannot fix or the outline is sampled: `pose`, `required_mm`, the worst `shortfall_mm` and `min_gap_mm`, `n_pads_short` with up to four `pads` (`pad_ref`, `pad_index`, `side`, `gap_mm`, `shortfall_mm`; `gap_mm` is null on a sampled outline), the pads that could not be measured, `kept` (`conflict_free`, or `crowding` when no seat cleared the placed parts), and `why` it stayed: `band_min` / `band_max` (clearing it leaves the declared overhang band), `setback`, `along_edge`, `outline_sampled`, `refused`, `crowds`, `still_short`, `nearest_edge` (the clearing pose reads nearest another edge), `along_edge_window` (it sits outside the declared along-edge window), `grade_delta` (beside this seat the clearing pose does not pass the intent grade: an error the seat does not have, a budget it is already over that the move grows, or a grade that could not be asked -- each row is listed in `grade_delta`, one cause being a pose that would change whether an interior Edge.Cuts contour reads as a hole), `crowding`. Filled on a fresh seed and on `--repair`; `{}` on `--reseat`, which drops its scope's edge declarations. Empty means every edge seat this run made clears the floor on the pads it could measure — NOT that `pad_edge_after` is clean, which also grades parts no seat moved. Join to `pad_edge_after.findings` by `pad_ref` + `pad_index`. Never changes the exit code |
