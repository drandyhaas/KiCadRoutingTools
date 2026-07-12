# Rip-Up and Reroute

This document describes what happens when a route fails: how the router figures out which previously-routed nets are in the way, rips them up, retries, and then re-routes the ripped nets.

Implementation: `rip_up_reroute.py` (rip/restore), `blocking_analysis.py` (who is blocking), `reroute_loop.py` (the escalation and reroute queue), `obstacle_costs.py` (corridor avoidance).

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--max-ripup` | 3 | Maximum number of blockers ripped up at once for one failing net |
| `--ripup-abandon-metric` | `stranded` | How a multipoint tap rip-up decides keep-retry vs abandon (see [Abandon metrics](#abandon-metrics)) |
| `--ripped-route-avoidance-radius` | 1.0 | Soft-penalty radius around a ripped net's former corridor (mm) |
| `--ripped-route-avoidance-cost` | 0.1 | Soft-penalty cost in the former corridor (0 disables) |

## When Rip-Up Triggers

Two mechanisms start a rip-up:

1. **Full A\* failure** — the search exhausts `--max-iterations` without reaching the target. The router returns the *frontier*: the set of cells the search tried to expand into but found blocked, for both the forward and reverse direction.
2. **Early probe detection** — before attempting a full route, quick probes (default 5000 iterations, `MAX_PROBE_ITERATIONS`) run in both directions. If a probe gets stuck against obstacles rather than merely running out of budget, rip-up starts immediately — without burning a full A* budget first.

## Blocking Analysis

`analyze_frontier_blocking()` attributes the blocked frontier cells to previously-routed nets. For each routed net it computes the obstacle cells its tracks and vias occupy (path expanded by track width + clearance) and intersects them with the frontier. Blockers are then prioritized:

- Nets that are the **sole blocker** of every cell they block come first — ripping them is guaranteed to open the frontier.
- Otherwise nets are scored by `unique_cells + near_endpoint_unique + 0.5 × shared_cells`, where `near_endpoint_unique` counts uniquely-blocked cells within 3mm of the failing net's source or target (blockages near endpoints are usually the decisive ones).

`filter_rippable_blockers()` removes candidates that can't be ripped: nets not actually routed in this run (by default, pre-existing tracks are left untouched — see [Ripping Pre-Existing Routes](#ripping-pre-existing-routes) for the `--rip-existing-nets` exception), and diff pair members whose partner isn't routed. Differential pairs are treated as one unit — P and N are always ripped and restored together.

The failure report printed to the console comes from this analysis ("Route stuck at (x, y) on F.Cu, blocked by: …").

## Ripping Pre-Existing Routes

By default only nets routed **in the current run** are rip-up candidates; tracks and vias already committed on the input board are left untouched, so re-running the router never disturbs existing routing. `route.py --rip-existing-nets PATTERN [PATTERN …]` opts specific pre-existing routed nets into the rip-up machinery: when one of them blocks a net the router is trying to route, it may be ripped up and re-routed like an in-run net. Use it on a board that a previous run (or another tool) already routed and that now needs a new net threaded through congested copper. Pass `'*'` to allow any non-plane net. Without the flag the default holds — pre-existing committed tracks are never ripped.

## Progressive N+1 Escalation

For a failing net, the router escalates through rip-up rounds (`reroute_loop.py`):

1. **N=1**: rip the top-ranked blocker, rebuild obstacles, retry the route.
2. If the retry fails, the *new* frontier is re-analyzed — the next blocker is chosen from fresh data, not the original ranking, since ripping one net changes what's in the way.
3. **N=2**: rip the next blocker as well (now two are ripped), retry. And so on up to `--max-ripup`.

A history set of `(net, frozenset(ripped blockers))` combinations prevents retrying a combination that already failed, which (together with the N cap) guarantees termination. If all rounds fail, every ripped net is restored unchanged and the net is reported as failed.

On success, the ripped nets are appended to the **reroute queue**.

## Ripped-Corridor Avoidance

When a net is ripped, its former corridor gets a *soft* cost penalty (`compute_ripped_route_costs()`): cells within `--ripped-route-avoidance-radius` of its old segments and vias cost slightly more (`--ripped-route-avoidance-cost`) for subsequent routing. The net that triggered the rip-up therefore tends to route *near* but not *through* the freed corridor, leaving room for the ripped net to re-route along something close to its original path. This is a penalty, not a block — if the corridor is the only way through, it is still used.

## The Reroute Loop

After the main routing pass, `run_reroute_loop()` processes the queue of ripped nets:

- Each ripped net (or diff pair, as a unit) is re-routed with the current obstacle state.
- If a reroute fails, the same blocking analysis and N+1 escalation applies — a reroute can itself rip further nets, which join the back of the queue (cascading).
- Termination is guaranteed by the same combination-history and `--max-ripup` cap; the queue is processed linearly and only grows by successful rip-ups.

## Abandon Metrics

When a multipoint net's tap route fails, Phase 3 rips blockers and retries
(`try_phase3_ripup` in `phase3_routing.py`). The retry can succeed *locally*
while the ripped victims — and nets they ripped in turn (the **rip tree**) —
end up worse off. The *abandon decision* arbitrates: keep the retry, or
abandon it, restore the net's original tap, and re-route the whole rip tree
around it (issues #85, #354).

`--ripup-abandon-metric` (GUI: Advanced tab → "Rip-up Abandon Metric"; env
override `KICAD_RIPUP_ABANDON_METRIC` for replay A/Bs) selects how the two
worlds are compared. All metrics except `stranded` compare the **retry
world** (retry kept, rip-tree victims re-routed around it) against the
**before world** (original tap, every rip-tree net as it was when first
ripped):

| Metric | Compares | Rationale / trade-off |
|--------|----------|-----------------------|
| `stranded` *(default)* | Root pads gained vs. pads on victims left with **no route at all** | The original #85 rule. Partial victim regressions count zero — they are usually greedy churn that later passes recover. Blind to partial losses; a victim stranded by the *retry* world vetoes the retry even if it would also strand in the abandon world. |
| `total-pads` | Total connected pads across root + rip tree | Symmetric; catches partial regressions. May veto retries whose partial losses would have recovered. |
| `complete-nets` | Count of fully-connected nets across root + rip tree (ties broken on total pads) | Aligns with how boards are graded (disconnected nets). Coarse: a net dropping one pad counts the same as one dropping all. |
| `congestion` | `total-pads` with each pad weighted `1 + min(foreign pads/vias within 1 mm, 24)/8` (range 1–4) | Boxed-in pads (few escape corridors) are unlikely to be reconnectable later, so losing one costs more. Reuses the #347 boxed-in-risk spatial hash. |
| `history` | `total-pads` with each pad weighted `1 + min(times its net was ripped or failed a re-route this run, 3)` (range 1–4) | Empirically-hard nets cost more to lose. Catches difficulty a static congestion proxy can't see (corridor shape, layer starvation). |
| `weighted` | Both weights multiplied | Congestion and history are complementary signals. |
| `probe` | `total-pads`, but a stranded rip-tree net is first probed (capped-iteration main-route attempt) in the **abandon world** (original tap guarded into the map); if it cannot route there either, it is lost either way and does not vote | Fixes the pessimism of counting a pad against the retry when abandoning would not save it. Probes the 2-terminal main route only; nothing is committed. |
| `weighted-probe` | `probe` filtering + `weighted` weights | The full combination. |

The rip tree itself (which nets participated, and each net's pre-rip
connectivity snapshot) is tracked by the #354 `rip_sinks` accumulator, which
also drives the abandon path's re-rip of every cascade victim so the kept
original tap can never be committed on top of cascade-era copper.

## Diagnostics

Every rip-up event is recorded in the per-net history (`record_net_event()` in `routing_state.py`): which net ripped it, at which escalation level N, and whether its reroute succeeded. For failed nets, this history is printed at the end of the run, and `routing_diagnostics.py` suggests parameter changes (e.g. raising `--max-ripup`, lowering clearance or track width) based on the failure pattern. Use `--verbose` for per-attempt detail.

## Tuning

- `--max-ripup 3` (default) resolves most single-blocker and double-blocker situations. Raise it on dense boards where failures persist — the cost is time, not correctness, since failed combinations are always rolled back.
- Setting `--ripped-route-avoidance-cost 0` disables corridor avoidance; the triggering net will then happily occupy the freed corridor, making the ripped net's reroute more likely to fail and cascade.
