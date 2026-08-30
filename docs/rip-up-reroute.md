# Rip-Up and Reroute

This document describes what happens when a route fails: how the router figures out which previously-routed nets are in the way, rips them up, retries, and then re-routes the ripped nets.

Implementation: `rip_up_reroute.py` (rip/restore), `blocking_analysis.py` (who is blocking), `reroute_loop.py` (the escalation and reroute queue), `obstacle_costs.py` (corridor avoidance).

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--max-ripup` | 3 | Maximum number of blockers ripped up at once for one failing net |
| `--ripup-abandon-metric` | `stranded` | How a multipoint tap rip-up decides keep-retry vs abandon (see [Abandon metrics](#abandon-metrics)) |
| `--ripup-blocker-select` | `count` | Which blocker the rip-up ladder targets first (see [Blocker selection algorithms](#blocker-selection-algorithms)). Also on `route_diff.py`, `route_planes.py`, `repair_planes.py`. Env override (route.py): `KICAD_RIPUP_BLOCKER_SELECT` |
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

The failure report printed to the console comes from this analysis ("Route stuck at (x, y) on F.Cu, blocked by: …"). A coverage line reports how many frontier cells were attributed to routed nets; the remainder is static, unrippable copper (pads, planes, pre-existing tracks, the board edge) — when that share dominates, ripping cannot open the frontier at all.

Two refinements sharpen the signal before ranking:

- **Fastest-failing direction.** The search runs from both ends; the side that died in fewer iterations is the constrained one, and its frontier is the drained pocket's perimeter (hugging track included). Attribution uses that side's cells rather than pooling both directions, so the broad flood cannot swamp the tight pocket. This applies in the single-ended, reroute, victim-retry, and multipoint tap paths.
- **Per-edge attribution (multipoint).** When several tap edges of one net fail, each edge is attributed separately against its *own* target, and the per-net results merge by taking each net's strongest edge — a net decisive at one edge is not diluted by edges it is irrelevant to, and a pooled "cell soup" can no longer promote a net relevant to none of them.

### Blocker selection algorithms

`--ripup-blocker-select` chooses how the candidates from the blocking analysis are ordered before the ladder starts ripping. The frontier is the *perimeter* of the reachable pocket, not the wall that actually separates source from target, so per-net cell counts can be misleading — the alternatives re-rank the same candidate set using better evidence:

- **`count`** (default) — the historical weighted-cell-count ranking described above.
- **`near-target`** — endpoint proximity first. The decisive last-mile blocker often hugs the failing pad but contributes only a handful of frontier cells, so the count ranking buries it under large far walls. Sort key: near-endpoint-unique cells, then unique, then count.
- **`bidir`** — both-directions boost. The search runs from both ends; a net attributed in BOTH directions' frontiers lies on a genuine separating wall, while a bystander boxed in behind one endpoint appears in only one. Both-direction nets get their weighted score doubled.
- **`mincut`** — soft-cost probe. The failing net is re-routed once on a clone of the obstacle map with every rippable candidate's copper converted from hard-blocked to a high soft cost; the copper the resulting path crosses is the true joint cut, and those nets are ripped first. If even the all-soft probe finds no path, the separating wall is static (unrippable) copper; the ladder still runs in legacy order, since ripping can free space for the via-placement and swap rescues even when no simple path exists. Costs roughly one extra A* search, on the failure path only.

Validator-named blockers — identities proved by geometry validators such as via placement (the exact copper that vetoes a pad via) — sort ahead of every frontier-inferred tier under all four algorithms.

## Ripping Pre-Existing Routes

By default only nets routed **in the current run** are rip-up candidates; tracks and vias already committed on the input board are left untouched, so re-running the router never disturbs existing routing. `route.py --rip-existing-nets PATTERN [PATTERN …]` opts specific pre-existing routed nets into the rip-up machinery: when one of them blocks a net the router is trying to route, it may be ripped up and re-routed like an in-run net. Use it on a board that a previous run (or another tool) already routed and that now needs a new net threaded through congested copper. Pass `'*'` to allow any non-plane net. Without the flag the default holds — pre-existing committed tracks are never ripped.

One exception: the end-of-run oracle-reconnect pass may auto-grant `--rip-existing-nets` authority over pre-existing blockers that earlier failure hints named (capped at 12). That escalation always respects the run's own net filter — a net the caller excluded by pattern (`'!GND'` while planes pour in a later step) is excluded *by plan* and is never auto-ripped; only an explicit operator `--rip-existing-nets` can override that.

### When to grant rip authority — and when not to

`--rip-existing-nets` and `--force-reroute` are **permissions to destroy
already-routed copper**. The router will use them, and a rip whose restore is
refused — its corridor was taken by copper routed while it was out — leaves
that net broken. In the sets-21-27 corpus wave this was the single largest
source of lost connectivity, ahead of routing failure itself (#600): 7 of 99
boards, one turning a 3-pad problem into a 20-pad one while trying to fix it,
another losing 20 nets all of their copper.

Two things that are not obvious:

- **Scoping `--nets` does not protect you.** One board regressed from a retry
  naming three nets. It is the rip *permission* that does the damage, not the
  route *scope*, so "just retry fewer nets" is not a safety measure.
- **You usually do not need to pass anything.** The #103 escalation above
  already grants the reconciliation rip authority over the exact blockers the
  failures named, under guards (never protected, negated, plane-backed, or
  large nets). A plain retry gets targeted rip authority for free; `'*'` adds
  only the *untargeted* part.

In rough order of preference for a congestion retry:

1. **Re-run the whole signal step thinner** (fab-floor `--track-width`, finer
   `--grid-step`). Thinner is monotonically better on dense boards and destroys
   nothing.
2. **Plain retry of the failed nets** — the in-run escalation supplies its own
   targeted authority.
3. **Named authority**: `--rip-existing-nets <the blockers the log named>`. The
   router prints exactly which pre-existing nets boxed each failure; ripping
   those is a decision, whereas `'*'` is a hope.
4. **`--rip-existing-nets '*'`** — last resort. Especially avoid combining it
   with `--force-reroute` over a large net list: that is the shape that cost
   `spartan6_4layer` 20 nets' copper.

Whatever you pass, the [improvement gate](#improvement-gate-600) is the
backstop: a run that ends net-worse is reverted rather than shipped.

## Improvement gate (#600)

At the end of a run `route.py` compares the board it is about to ship against
the board it was given, per multi-pad net, with the same authoritative
union-find `check_connected` uses:

- **lost** — connected before, broken after
- **gained** — broken (or bare) before, connected after

The run is **rejected** when it ends worse on **either** axis — more nets broken
than it connected, *or* more disconnected pads than it started with. Both must
be non-worse to ship. A rejected run's output file is replaced by the input
board, because in every recorded case the pre-rip board was the better
artifact, and the chain keeps a board to continue from.

**Neither axis may outvote the other**, and `spartan6_4layer` is why. Re-running
its wave command gives `lost 36, gained 43` — a net count that looks *better* by
7 — while the board's disconnected pads go `83 → 154`. The nets it broke were
far larger than the ones it closed. An earlier version of this gate compared
the two lexicographically with the net count first and shipped that board. Pad
count is the honest measure of how much of the board is unreachable, so a run
that raises it is worse regardless of the net tally.

This is still not an "any regression" test: a pass that closes five nets and
breaks one ships, provided the pad count did not rise — the recovered pads
outnumber the lost ones, which is exactly when discarding the run would throw
away more than it saves.

An equal trade on both axes ships, reported with both net lists named:
`bms_sensor`'s retry reproduced today closes the three nets it was asked to
close, breaks three others, and leaves the pad count unchanged. The operator who
passed `--rip-existing-nets` authorised that; what they could not do before was
see it.

```
IMPROVEMENT GATE: this run broke 3 previously-connected net(s) and connected 3 -- ACCEPTED
  broken by this run: /BMS.Can_L, /BMS.Enable_Out, V_+5V
  connected by this run: /CAN.Interrupt, /SPI.Clock, /SPI.Miso
  disconnected pads: 3 -> 3 over 43 multi-pad net(s)
```

The verdict is also emitted as a machine-readable `JSON_IMPROVEMENT_GATE:` line
(`lost`, `gained`, `disconnected_pads_before/after`, `nets_compared`,
`verdict`), so a chain can assert on it instead of grepping prose.

**If you see `REVERTED`, the retry did not fail to run — it ran and was
rejected.** Re-running it with *more* rip authority is the one response
guaranteed not to help; change the approach instead (thinner, finer grid,
different layer budget, or accept the open net and report it).

`KICAD_IMPROVEMENT_GATE=0` disables the gate — for A/B measurement, or when you
deliberately want the regressed board on disk to inspect it.

**Both fronts measure and both revert**, by the same verdict from the same
engine code. Only the spelling of the revert differs, because the artifact
does: the file front rewrites the output as the input board; the GUI front
returns an empty change-set, which is a true rollback rather than an un-apply
because the plugin's applier runs *after* `batch_route` returns and therefore
never touches the live board. The GUI also receives the verdict as
`results_data['improvement_gate']`; diagnostics (`blockers`, `pad_pairs_open`,
`boxed_in`) survive a rejection, since they are what the caller needs in order
to try something different.

The gate is skipped when it cannot mean anything: a run whose input board has
no copper at all (nothing to regress), an in-place run (`output == input`,
where no pre-run board survives to revert to), and every nested `batch_route` —
the reconciliation sub-run and the plane finalize's reconnects all carry
`final_reconcile=False`, and only the run that owns the artifact gates.

## Progressive N+1 Escalation

For a failing net, the router escalates through rip-up rounds (`reroute_loop.py`):

1. **N=1**: rip the top-ranked blocker, rebuild obstacles, retry the route.
2. If the retry fails, the *new* frontier is re-analyzed — the next blocker is chosen from fresh data, not the original ranking, since ripping one net changes what's in the way.
3. **Slot-0 guard**: if the re-analysis names a *different* top pick than the ripped singleton, the refuted rip is restored and the fresh pick is tried **alone** once before extending to pairs. Without this, the ladder only ever grows a prefix around its first guess, so a wrong first pick contaminated every combination it would ever try.
4. **N=2**: rip the next blocker as well (now two are ripped), retry. And so on up to `--max-ripup`.

A history set of `(net, frozenset(ripped blockers))` combinations (recorded when a combination *succeeds*) prevents pointlessly re-ripping a combo that already worked once for this net; together with the N cap this guarantees termination. If all rounds fail, every ripped net is restored unchanged and the net is reported as failed.

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

`--ripup-abandon-metric` (GUI: Advanced options tab → "Rip-up Abandon Metric"; env
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

- `--max-ripup 3` (default) resolves most single-blocker and double-blocker situations. Raise it on dense boards where failures persist — the cost is NOT purely time: measured on a 6-board chain A/B, `--max-ripup 5` beat 10 and 20 was worse than 10, because each extra rip level risks a permanent casualty -- a ripped victim whose corridor is taken while it is out cannot be restored (see `terminal_restores` in JSON_SUMMARY: `full_open` and `stub` ship broken). Treat 3-5 as the working range and escalate only on a specific failing net.
- Setting `--ripped-route-avoidance-cost 0` disables corridor avoidance; the triggering net will then happily occupy the freed corridor, making the ripped net's reroute more likely to fail and cascade.
