---
name: diagnose-routing-failures
description: Root-causes failed routes from a routing run's logs and the board file. Parses the JSON summary, failed-net histories, and blocking reports, correlates failures spatially with board regions and components, classifies the failure mode, and outputs a ready-to-run retry command. Use after route.py/route_diff.py runs report failed nets.
---

# Diagnose Routing Failures

When this skill is invoked with a board file and routing log file(s), root-cause the failures and produce a targeted retry command. Routing runs should always be captured with `2>&1 | tee /tmp/<step>.txt` so the logs are available.

## Step 1: Parse the Structured Outputs

### JSON summary

```bash
grep "JSON_SUMMARY" /tmp/route_output.txt | sed 's/.*JSON_SUMMARY: //' | python3 -m json.tool
```

Key fields:
- `failed_single`: failed single-ended net names — nets with NO result at all
- `open_single`: nets that KEPT a result whose copper still leaves pads
  disconnected (non-multipoint only; a multipoint shortfall is already the pad
  deficit). **Always read this alongside `failed_single`** — a board can ship
  open copper with `failed_single: []`, so a run is only clean when both are
  empty. These nets also appear by name in `failed_multipoint`.
- `terminal_restores`: `{net: outcome}` for rip victims restored at terminal
  failure. `full` is the only success; `full_open` restored copper that never
  covered every pad, and `stub` kept only the escape stubs — both ship broken
  and are worth reporting as failures with a distinct cause (the reroute failed
  and the pre-rip copper could not be fully put back).
- `stacked_copper`: same-net duplicate copper KiCad's DRC will never flag. Not a
  routing failure, but if present, say so — it is invisible to every other check.
  It is censused before the plane finalize; route.py's end-of-run dedup (#162)
  then removes exact same-span copies, so read `check_weird` on the output for
  what shipped.
- `close_same_net_vias` (#1414): same-net via pairs this run laid under the drill
  hole-to-hole floor (check_drc `VIA-DRILL-HOLE`); `different_net_crossings` (#1375):
  different-net tracks crossing on one layer among the run's copper (a short;
  check_drc `SEGMENT-CROSSING`). Both ship despite `failed 0`: say so.
- `failed_multipoint`: nets with unconnected pads, including pad coordinates
- `blockers`: per still-failed net, the run's LAST frontier blocking analysis
  (#409): `{net, stage, blocked_by: [{net, blocked_count, unique_cells,
  track_cells, via_cells, near_target_cells, near_source_cells}], more?}` —
  "net X failed BECAUSE nets Y,Z wall it off" as data. A net boxed in by
  PRE-EXISTING copper gets an entry with `stage: "preexisting"` whose
  `blocked_by` items carry only `{net, preexisting: true}`. Key absent when no
  failed net has an attributable blocker.
- `boxed_in`: per failed net whose search died almost at once with nothing
  rippable — `{net, verdict: "boxed_in_static", iterations, geometry:
  {grid_step, clearance, track_width, via_diameter}}`. Static obstacles
  (neighbouring pads + clearance), not congestion: the lever is a finer grid
  or smaller clearance/track, unless `geometry` is already at the floor.
- `fanout_dropped`: per failed net with a ball its fanout never escaped —
  `{net, pad, component, plane_like, pads}` (#652). The fix is upstream of
  this step: re-run the fanout for that net, or, when `plane_like`, the plane.
- `sealed_by_snpc`: per failed net whose last legal via site was closed by
  this run's own `--same-net-pad-clearance` (#907) — `{net, pad,
  same_net_pad_clearance, required_surround_mm, escape_reach_mm, aperture}`.
  Re-run with `--same-net-pad-clearance 0` (a via then placed in a pad is
  stamped filled+capped, #962), or widen the channel in placement.
- `multipoint_pads_connected` vs `multipoint_pads_total`: connection success rate.
  Derived from the final-board union-find (the same check `check_connected.py`
  uses), so it credits pads reached via planes/zones, fanout stubs, and
  rip-up/retry reroutes and agrees with `check_connected.py` (issue #184).
- `pad_pairs_connected` / `pad_pairs_total`: pad-pair routability tallies
  (#409 follow-up; PRR = connected/total downstream). Per graded multi-pad
  net: connected = |pads| − pad components from the same final-board
  union-find, total = |pads| − 1 (clamped so a net whose pads are invisible
  to the union-find grades as connected). Population: the routing scope
  after already-routed filtering, pre-existing rippable nets the run graded,
  and disturbed coverage-gate nets. NOT reconcilable with
  `multipoint_edges_*` (those count component-MST edges — pre-existing
  copper joins terminals first, so there are fewer edges than pad pairs).
  Each summary's are computed before the final reconciliation, like
  `blockers`; the merged document (`--json-out`, `JSON_SUMMARY_MIN`) takes a
  still-broken net's tally from the end-of-run re-grade of the shipped board,
  so a net broken after the last summary is listed too (#1309). Counted per
  outer board outline (#479): a gap between two outlines is neither a routable
  pair nor an open one.
- `pad_pairs_open`: per net shipping a pair deficit: `{net, pairs_connected,
  pairs_total, outcome, open_subtype}`, sorted by net; key absent when none.
  `outcome` is always `"open"` today — route.py runs no DRC, so a route-time
  failure is definitionally an open; shorts are `check_drc.py`'s domain and
  the field exists so a DRC-integrated emitter can add `"short"` without a
  schema break. (Distinct namespace from route_diff `pair_reports.outcome`.)
  `open_subtype`: `collision_refused` (left open because restoring the
  ripped track would collide — a short was averted, #134), `coverage_gate`
  (disturbed out-of-scope net shipping broken), `unrouted` (scope net, no
  result at all), `partial` (routed a result but left pads disconnected).
  Join with `blockers` on `net` for cause + outcome per failed net.

### Failed net histories

```bash
grep -A 30 "FAILED NET HISTORIES" /tmp/route_output.txt
```

Each failed net's history records its route attempts, which nets ripped it up (`ripped_by`, with escalation level N), and reroute outcomes. A net that was ripped and never successfully rerouted points at the net that ripped it as much as at the failed net itself.

### Blocking reports

Look for blocking analysis lines:

```bash
grep -B2 -A8 -i "blocked by\|no rippable blockers\|stuck (\|Hint:" /tmp/route_output.txt
```

These name the previously-routed nets occupying the failed route's frontier
("Frontier blocked by N nets ..."), which search direction got stuck
("forward stuck (...)"), and the `Hint:` line the router prints for
pre-existing blocking copper, a static box-in, a fanout-dropped ball or a
`--same-net-pad-clearance` seal.

Note: the final `JSON_SUMMARY` carries the same attribution structured — `blockers` (#409), `boxed_in`, `fanout_dropped` and `sealed_by_snpc` hold each still-failed net's last verdict. Prefer them for final-state diagnosis; the greps above remain useful for transient mid-run analyses (blockers of nets that later routed).

### route_diff runs: the pair keys

A `route_diff.py` step's `JSON_SUMMARY` (parse it as above, or read its
`--json-out FILE`) has none of route.py's `failed_single` / `blockers`. It puts
each pair it was given in one bucket:

- `routed_diff_pairs`: coupled, and credited.
- `failed_diff_pairs`: no coupled route, and not deferred: the pair is not routed.
- `single_ended_diff_pairs`: the whole pair was left for single-ended routing.
  **A 2-terminal pair whose coupled route fails lands here, not in
  `failed_diff_pairs`** (#289), so an empty `failed_diff_pairs` does not make
  the step clean. Only `failure_reason` `electrically-short` is by design (every leg is below
  the coupled floor, so coupling buys nothing); any other reason is a pair that
  should be coupled shipping single-ended. `single_ended_fallback`, and per pair
  `se_fallback` (`p`, `n`: `routed` / `partial` / `failed` / `already-connected`),
  say how its members came out when route_diff routed them single-ended itself.
- `partial_diff_pairs`: the member audit took the credit back: a claimed coupled
  route left a member pad open (`outcome` `incomplete`), or a `partial` pair kept
  no coupled trunk (`no_coupled_trunk`).
- `uncoupled_diff_pairs` (#1303): both members routed, but the copper is not
  coupled; `pair_reports[].copper` (`layers_p`, `layers_n`, `coupled_mm`,
  `coupled_share`, `gap_mm`) says what ships.
- `skipped_bad_fanout`: never attempted, because the fanout's stubs overlap
  themselves (#242).

And across the buckets:

- `member_incomplete_pairs`: every pair with a member pad still open at the end
  of the step. Expected only when the pair's members are in
  `single_ended_followup_nets`, left for the next `route.py` by design (a
  `partial` pair's peeled far terminals, or an `electrically-short` pair's
  members); open copper for any other.
- `polarity_swap_denied_pairs`: a P/N mismatch the run was not allowed to
  resolve by a swap (#279); `polarity_swapped_pairs` lists the swaps it made.
- `casualty_reconcile`: nets this step ripped whose reroute failed
  (`restored`, `rerouted`, `partial`, `unrecovered`).

`pair_reports[]` has one record per pair: `pair`, `p_net`, `n_net`, `outcome`
(`coupled`, `partial`, `deferred`, `failed`, `skipped`, `uncoupled`,
`incomplete`), `failure_reason` and `failure_stage` (set only for `failed` and
`deferred`; the stage is `pre-route`, `initial-route`, `multipoint`, `reroute`
or `casualty-reconcile`), `incomplete_members`, and when they apply
`blocking_nets`, `casualty`, `se_fallback` and `copper`. Classify each
`failure_reason` with the pair table in Step 3.

## Step 2: Correlate Failures Spatially

Load the board and map failures to regions and components:

```python
import sys
sys.path.insert(0, "py_router")
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('board.kicad_pcb')

# For each failed net: where are its pads, and what is nearby?
for net in pcb.nets.values():
    if net.name in failed_names:
        for pad in net.pads:
            print(net.name, pad.component_ref, pad.global_x, pad.global_y, pad.layers)
```

Check whether failures cluster:
- Near one component (especially a BGA/PGA — its auto-detected exclusion zone may be walling off paths)
- In one board region (a congested corridor every failed net needs to cross)
- On one layer (layer costs or direction preference starving that layer)

Report the spatial pattern explicitly, e.g. "all 6 failures have an endpoint within 3mm of U9's southeast corner."

## Step 3: Classify and Recommend

Work through the failure modes most-targeted-first. Do not jump straight to global parameter increases — they are the last resort, not the first.

| Evidence | Failure mode | Fix |
|----------|--------------|-----|
| `unroutable_as_designed` names the net (JSON_SUMMARY rows, JSON_SUMMARY_MIN names; the console prints "Unroutable as designed") | A pad is already shorted to another net as drawn: another net's pad on it, or another part's copper (stacked or swapped net-tie footprints) | Stop retrying: no routing reaches that pad without the short, and KiCad grades it `shorting_items` before any routing. Report the pad and the foreign copper named in the row; the fix is the board's (re-pose the part), not a router flag |
| "no rippable blockers found" near a BGA/PGA | Blocked by the BGA exclusion zone or pre-existing copper | `--no-bga-zone <ref>` for that component |
| Stuck positions cluster in one corridor; blockers are routed nets that themselves had few alternatives | Corridor congestion | Suggest the user **draw a guide corridor**: a polyline on `User.1` through a less congested region, then re-route the failed nets with `--guide-corridor`. Describe in words where the corridor should go (e.g. "south of J3, between the mounting hole and C14"). **Do not draw the geometry yourself** — guide drawing is the user's decision |
| Failed nets have source and target stubs on conflicting layers; rip-up histories show repeated mutual ripping | Same-layer crossing conflicts | `--mps-layer-swap`, or revisit `--layer-costs` |
| "Re-route FAILED: no path found" for ripped nets | Search space exhausted | NOT a budget problem: the router self-budgets (#529, default on — auto-extends to 1e7 iterations while progressing), so do not suggest raising `--max-iterations`. A "dynamic iterations (#529): search extended to N" line followed by failure means genuine progress ran out — escalate capacity: `--max-ripup`, clearance/track toward fab floor, or layers. (`KICAD_DYNAMIC_ITERATIONS=0` restores legacy static caps for A/B only) |
| Many multipoint pads failed on the same fine-pitch component; `boxed_in` entries for them | Grid too coarse for the pad geometry | `--grid-step 0.05`, and check `--track-width`/`--clearance` against the pad pitch |
| `fanout_dropped` names the net's ball | The fanout never escaped that ball — no rip or retry can reach it | Re-run the fanout for that net (`bga_fanout.py`/`qfn_fanout.py`, `--escape-method underpad`, or a smaller `--via-size`); when `plane_like`, re-create the plane instead |
| `sealed_by_snpc` names the pad | This run's own `--same-net-pad-clearance` closed the last via site | Retry with `--same-net-pad-clearance 0`, or widen the channel in placement |
| Failures cluster on one part's pads or escape face; `py_tools/check_reachability.py <board> --pad REF.PAD` says CAGED (its verdict grades the placement, with the board's routed copper set aside); or `boxed_in[].geometry` is already at the board/fab floor | Placement-shaped: geometry, not the router | Stop retrying: no router flag fixes it. Hand the board back to `/plan-pcb-placement full` (or `place`) to re-place, then re-route |
| `check_reachability` says PASSABLE, and its `routed` line says ROUTED COPPER cages the pad (#1347) | Congestion: the copper already laid walls the pad in | A routing finding, NOT a placement one: rip up and reroute the nets named at that throat (`--rip-existing-nets`), or reorder; do not re-place |
| Failures spread across the board, blockers vary | Genuine capacity problem | Escalate in order: reduce `--clearance`/`--track-width` toward fab minimums → add routing layers. Do NOT raise `--max-ripup` past ~5 (measured worse: deep rip chains strand their victims) |

**Pair failures** (a `route_diff.py` step), by `pair_reports[].failure_reason`.
The shrink below means `--track-width`, `--diff-pair-gap` and
`--via-size`/`--via-drill` stepped toward, never below, the fab floors, keeping
`--impedance`; the gap is floored at the clearance (#441), so it shrinks only
as far as the clearance does. It is the fix for a pair the geometry crowds out,
and for none of the polarity or graze codes.

| `failure_reason` | What it means | Fix |
|------------------|---------------|-----|
| `electrically-short` | Every leg is shorter than the coupled floor | None: single-ended by design. Check that its members closed (`se_fallback`, or the next `route.py`) |
| `congestion` | Copper of nets this run routed walls the coupled route (`blocking_nets` names them), and the step's own rip ladder did not clear it | The shrink; or route the pair in an earlier step, before the nets that wall it |
| `no-escape-path` | An endpoint's escape is blocked: the search never launched | The shrink, or a shorter `--diff-pair-centerline-setback` / wider `--max-setback-angle`; on a BGA or QFN the escape is the fanout's |
| `needs-fanout` (#764) | A single track launches at the terminal, the coupled pair does not; the step skips its rip ladder, since no freed copper helps | A coupled fanout before the pair step (`bga_fanout.py ... --diff-pairs P N`); for a terminal on a pad array the step's log prints that command with the run's own numbers |
| `pose-router-failure` (#346), `multipoint-leg-failure` | The coupled search ran and was exhausted (for a multipoint pair, on one of its legs) | Capacity: the shrink, more `--layers`, a finer `--grid-step` |
| `coupled-route-failed` | A multipoint pair no coupled chain could route, left whole for single-ended | It ships single-ended: a failure if the interface needs coupling; retry it as `pose-router-failure` |
| `connector-graze` (#165/#246) | A route was found, then rejected: a connector, setback or offset segment grazes foreign copper by check_drc's measure. The step rips a grazed net it routed itself and retries; an earlier step's copper and every pad stay | The step's log names the grazed pad, via or track. Route the pair before that copper (an earlier step), or move the terminal join: a narrower track or smaller via there, or a fanout that lands the pair clear |
| `polarity-swap-denied` (the pair is also in `polarity_swap_denied_pairs`) | P and N land on the wrong sides, and the swap that resolves it is not allowed for this pair | Only when the pair's endpoints tolerate a swap (plan-pcb-routing's polarity-swap policy; `/identify-diff-pairs` gives `polarity_swappable`): add it to `--polarity-swap-nets`. Otherwise no router flag fixes it: it is a pin swap in the schematic, or the pair routes single-ended |
| `polarity-mismatch`, `pn-crossing` | A swap was tried and could not resolve the mismatch / P and N must cross and untangling failed. A `pn-crossing` pair can be in `polarity_swap_denied_pairs` too | The same policy question first; a `pn-crossing` pair may also clear on another layer pair (`--layers`) |
| `no-route` | Failed with no iterations and no blocked cells. An **intra-pair clearance** rejection reports this code too (no code of its own): P and N would sit closer than the clearance between them, after necking; the step's log says `intra-pair clearance` | For intra-pair: the terminal's geometry (its pads or fanout stubs), not a smaller gap, which is floored at the clearance |
| `endpoint-resolution-failure` | The router found no usable endpoints for the pair | Check that the pair's pads (or fanout stubs) exist on the routed `--layers` |
| `ripped-not-restored` (stage `casualty-reconcile`) | Another pair's route ripped it, and neither its reroute nor its restore landed (`casualty`) | Re-run route_diff on this pair alone (`--nets P N`) |
| `fanout-self-overlap` (`outcome` `skipped`) | The fanout's stubs overlap themselves (#242) | Fix the fanout, then re-run the pair step |

## Step 4: Output the Retry Command

Produce one command that retries **only the failed nets** with the targeted fixes, and explain each parameter change in one line:

```bash
python3 -X utf8 py_router/route.py board_routed.kicad_pcb board_retry.kicad_pcb \
    --nets "SDA" "Net-(U1-Pad8)" \
    --no-bga-zone U9 \
    --max-ripup 5 \
    2>&1 | tee /tmp/route_retry.txt
```

**Keep the run's rule decision.** "Toward the fab floor" in the tables above
holds where the project's rules are aspirational stock. When they are the
user's own product rules (`/plan-pcb-routing` Step 4, "Whose rules does the
project hold?"), the floor is the board's own Board Setup minimums: the retry
carries the failing step's `--escalation board` (or `off`) and
`--no-fix-drc-settings`, shrinks no size below those minimums, and names any
shrink below them as a decision for the user rather than proposing it as the
fix.

Routing only the failed nets preserves the successful routes (already in the input file) and is much faster than a full re-run. **On a board with pours, add the poured nets to `--nets` too** (#1032): the retry's copper can cut a pour, and its in-run plane finalize repairs only the poured nets inside its scope. A broken poured net is retried the same way, with `route.py`, never a standalone `repair_planes.py` (#1112). After the retry, re-check with `check_connected.py`, and if nets still fail, repeat the diagnosis on the new log — the failure mode often changes after the first fix.

**For a pair step, the retry is a `route_diff.py` call** on the failing pairs'
members only, from that step's output, with the step's own flags verbatim
except the change the pair table names:

```bash
python3 -X utf8 py_router/route_diff.py board_diff.kicad_pcb board_diff_retry.kicad_pcb \
    --nets "/USB_D+" "/USB_D-" \
    --track-width 0.1 --diff-pair-gap 0.1 --via-size 0.3 --via-drill 0.2 \
    --force-reroute --json-out /tmp/route_diff_retry.json \
    2>&1 | tee /tmp/route_diff_retry.txt
```

`--force-reroute` rips the members and routes them again AS A PAIR; it is
needed when they already carry copper (routed single-ended, or a `partial` /
`uncoupled` pair). A pair that should be coupled is retried with `route_diff.py`,
never `route.py`, which routes single-ended and would de-pair it (#1399); the
members in `single_ended_followup_nets` are the exception, closed by the next
`route.py` by design. Read the retry's `pair_reports` the same way before the
next step.

If the retry log prints `IMPROVEMENT GATE: ... -- REJECTED` (#600, on by default), the retry broke more than it connected (or raised the disconnected-pad count), so route.py wrote a copy of the INPUT board to the output path (the verdict is also on the `JSON_IMPROVEMENT_GATE:` line). More rip authority cannot help: change the approach (thinner track, finer grid, other layers) or report the nets as open. When the gate says its verdict rests on zone nets outside `--nets`, add those nets to `--nets` and retry.
