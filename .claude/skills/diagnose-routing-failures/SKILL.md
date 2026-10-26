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

## Step 4: Output the Retry Command

Produce one command that retries **only the failed nets** with the targeted fixes, and explain each parameter change in one line:

```bash
python3 -X utf8 py_router/route.py board_routed.kicad_pcb board_retry.kicad_pcb \
    --nets "SDA" "Net-(U1-Pad8)" \
    --no-bga-zone U9 \
    --max-ripup 5 \
    2>&1 | tee /tmp/route_retry.txt
```

Routing only the failed nets preserves the successful routes (already in the input file) and is much faster than a full re-run. **On a board with pours, add the poured nets to `--nets` too** (#1032): the retry's copper can cut a pour, and its in-run plane finalize repairs only the poured nets inside its scope. A broken poured net is retried the same way, with `route.py`, never a standalone `repair_planes.py` (#1112). After the retry, re-check with `check_connected.py`, and if nets still fail, repeat the diagnosis on the new log — the failure mode often changes after the first fix.

If the retry log prints `IMPROVEMENT GATE: ... -- REJECTED` (#600, on by default), the retry broke more than it connected (or raised the disconnected-pad count), so route.py wrote a copy of the INPUT board to the output path (the verdict is also on the `JSON_IMPROVEMENT_GATE:` line). More rip authority cannot help: change the approach (thinner track, finer grid, other layers) or report the nets as open. When the gate says its verdict rests on zone nets outside `--nets`, add those nets to `--nets` and retry.
