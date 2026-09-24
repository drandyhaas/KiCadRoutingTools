"""Board-level improvement gate (#600): never ship a board a run made WORSE.

Why this exists
---------------
`route.py` may rip already-routed copper -- the in-run finalize/reconciliation
rips a routed net to clear a blocker or place a plane tap, and
`--rip-existing-nets` retry rounds rip nets as blockers -- and a rip whose
restore is refused (its corridor was taken by copper routed while it was
ripped) leaves that net broken. The run then reports the damage honestly and
writes the board anyway. In the sets-21-27 wave that was the single largest
source of lost connectivity, larger than routing failure itself: 7 of 99
boards, `bms_sensor` turning a 3-pad problem into a 20-pad one while trying to
fix it, `spartan6_4layer` losing 20 nets all of their copper.

The missing half was never detection -- the coverage gate and the #293 warning
both fire correctly. It was that **a rip which cannot be restored was not
treated as a reason to roll back**. In every recorded case the pre-rip board
was the better artifact.

Scoping the retry does NOT protect you: `ftdi_debug_toolkit` regressed from a
retry naming three nets. It is the rip PERMISSION that does the damage, not
the route scope -- so this gate is about permission and outcome, and lives at
the write boundary where every rip entry point converges.

The measurement
---------------
Per multi-pad net, "is it fully connected", by the same authoritative
union-find (`check_net_connectivity`) and the same within-outline rule
(`net_break_within_outlines`) route.py's own coverage gate and check_connected
use -- so a verdict here cannot disagree with the engine's own sweep or with
the grader.

- **lost**    -- connected BEFORE, broken AFTER. The regression this exists for.
- **gained**  -- broken (or bare) BEFORE, connected AFTER. The run's actual work.

A run is REJECTED when it ends worse on EITHER axis: more nets broken than it
connected, **or** more disconnected pads than it started with. Both must be
non-worse to ship.

Neither axis may outvote the other, and `spartan6_4layer` is why. Re-running
its wave command produced:

    lost 36 nets, gained 43   ->  net count says BETTER by 7
    disconnected pads 83 -> 154  ->  the board's open pads nearly DOUBLED

An earlier version of this gate compared the two lexicographically with the net
count first, and shipped that board: the 36 lost nets were far larger than the
43 gained ones, so a favourable net count hid a doubling of open pads. Pad count
is the honest measure of how much of the board is unreachable -- it is what the
graders report and what a human sees as damage -- so a run that increases it is
worse no matter how the net tally looks.

This is still not an "any regression" test. A pass that closes five nets and
breaks one ships, provided the pad count did not rise: the recovered pads
outnumber the lost ones, which is exactly the case where discarding the run
would throw away more than it saves.

An exact draw on both axes ships, and is reported loudly with both net lists.
`bms_sensor`'s retry reproduced today closes the three nets it was asked to
close, breaks three others, and leaves the pad count unchanged at 3 -- a
genuine lateral trade. The operator who passed `--rip-existing-nets` authorised
that; what they could not do before is see it.

The prior art is unanimous on why the gate has to exist at all: five separate
measurements (2026-08-05/06) found that the SAME rip authority WITHOUT an
accept gate closes opens and trades them for casualties, while the identical
authority WITH a grade-and-revert gate closes them at zero casualties.
Authority is only safe when someone checks the result and can say no.
"""
from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Tuple


def net_connectivity_map(pcb_data, tolerance: float = 0.02,
                         segs_by_net: Optional[Dict[int, list]] = None,
                         vias_by_net: Optional[Dict[int, list]] = None,
                         zones_by_net: Optional[Dict[int, list]] = None
                         ) -> Dict[int, Tuple[bool, int]]:
    """{net_id: (fully_connected, disconnected_pad_count)} per multi-pad net.

    Copper defaults to the board's own segments/vias/zones; pass the
    *_by_net overrides to measure a WRITE MODEL (the copper a run is about
    to emit) instead of what is currently in pcb_data.
    """
    from check_connected import check_net_connectivity, net_break_within_outlines

    if segs_by_net is None:
        segs_by_net = {}
        for s in pcb_data.segments:
            segs_by_net.setdefault(s.net_id, []).append(s)
    if vias_by_net is None:
        vias_by_net = {}
        for v in pcb_data.vias:
            vias_by_net.setdefault(v.net_id, []).append(v)
    if zones_by_net is None:
        zones_by_net = {}
        for z in (getattr(pcb_data, 'zones', None) or []):
            zones_by_net.setdefault(z.net_id, []).append(z)

    out: Dict[int, Tuple[bool, int]] = {}
    for net_id, pads in (pcb_data.pads_by_net or {}).items():
        if not net_id or len(pads or []) < 2:
            continue          # net 0 pseudo-net / trivially connected
        r = check_net_connectivity(
            net_id, segs_by_net.get(net_id, []), vias_by_net.get(net_id, []),
            pads, zones_by_net.get(net_id, []), tolerance=tolerance,
            pcb_data=pcb_data)
        # #479 multi-board: only a break WITHIN one outline is a real break.
        broken, dis_pads = net_break_within_outlines(pcb_data, r)
        out[net_id] = (not broken, len(dis_pads or []) if broken else 0)
    return out


def compare_connectivity(before: Dict[int, Tuple[bool, int]],
                         after: Dict[int, Tuple[bool, int]],
                         net_name: callable,
                         excluded_ids: Optional[Iterable[int]] = None) -> Dict:
    """Per-net connectivity delta between two states of the same board.

    Only nets present in BOTH maps are compared: a net that exists in one
    reading and not the other is a parse/scope difference, not a routing
    outcome, and must not be able to trip the gate.

    `excluded_ids` (#1032) are nets the run was told, BY PLAN, not to repair:
    the poured plane nets outside a scoped call's `--nets`, which the in-run
    finalize skips (`finalize_excluded_nets`). A signal lap crossing such a
    pour still cuts it, and judging the lap on pads its own finalize was
    forbidden to heal compares unlike with unlike. They are dropped from BOTH
    maps and reported apart in `excluded_by_plan` as (name, before, after)
    disconnected-pad counts, so the damage stays visible -- it just cannot
    vote.

    `worsened` lists every compared net whose disconnected-pad count ROSE
    WITHOUT being newly broken (it was already open before the run), as
    (name, before, after) -- disjoint from `lost`, so a net is named once. A
    pad-count rejection used to name no net at all in that case, because the
    net is then not `lost`.
    """
    excluded = set(excluded_ids or ())
    lost: List[str] = []
    gained: List[str] = []
    worsened: List[Tuple[str, int, int]] = []
    excluded_by_plan: List[Tuple[str, int, int]] = []
    pads_before = pads_after = 0
    compared = 0
    for net_id, (conn_b, dis_b) in before.items():
        if net_id not in after:
            continue
        conn_a, dis_a = after[net_id]
        if net_id in excluded:
            excluded_by_plan.append((net_name(net_id), dis_b, dis_a))
            continue
        compared += 1
        pads_before += dis_b
        pads_after += dis_a
        if conn_b and not conn_a:
            lost.append(net_name(net_id))
        elif conn_a and not conn_b:
            gained.append(net_name(net_id))
        elif dis_a > dis_b:
            worsened.append((net_name(net_id), dis_b, dis_a))
    return {
        'lost': sorted(lost),
        'gained': sorted(gained),
        'worsened': sorted(worsened),
        'excluded_by_plan': sorted(excluded_by_plan),
        'disconnected_pads_before': pads_before,
        'disconnected_pads_after': pads_after,
        'nets_compared': compared,
    }


def gate_verdict(cmp: Dict) -> str:
    """'reject' when the run left the board worse on EITHER axis.

    Worse = more nets broken than connected, OR more disconnected pads than it
    started with. Neither axis may outvote the other: spartan6_4layer gained 7
    on the net count while DOUBLING its open pads (83 -> 154), because the nets
    it broke were much larger than the ones it closed. See the module
    docstring.
    """
    net_delta = len(cmp['lost']) - len(cmp['gained'])
    pad_delta = cmp['disconnected_pads_after'] - cmp['disconnected_pads_before']
    return 'reject' if (net_delta > 0 or pad_delta > 0) else 'accept'


def plan_excluded_net_names(zone_net_names: Iterable[str],
                            net_names: Optional[List[str]]) -> List[str]:
    """The poured nets a scoped call leaves to a LATER step (#1032).

    The same rule route.py's in-run finalize applies (`finalize_excluded_nets`):
    a zone net that the call's `--nets` filter does not match. An unscoped call
    (no filter) excludes nothing. Used by the gate when the finalize did not
    run (KICAD_PLANE_FINALIZE=0, a checkpoint stop) so the gate still compares
    like with like.
    """
    if not net_names:
        return []
    from net_queries import matches_net_filter
    return sorted({n for n in zone_net_names
                   if n and not matches_net_filter(n, net_names)})


def format_report(cmp: Dict, verdict: str, action: str) -> str:
    """The human line(s). Names the nets -- a count alone is not actionable,
    and the whole point of the gate is that the operator can see WHICH
    already-routed copper a rip took out."""
    lines = []
    # #1032: the head line NAMES what it judged on, each list at its OWN
    # clause. `broke 1 ... REJECTED` hid that the one net was GND; a
    # pad-count-only rejection (the net was already broken before the run,
    # so it is not `lost`) named nothing; and one bracket after "connected"
    # read as if a net that got WORSE had been connected.
    worsened = cmp.get('worsened') or []
    lost = list(cmp['lost'])
    # `worsened` is disjoint from `lost` (compare_connectivity): pad count
    # rose on a net that was already open. The filter only guards a caller
    # that built the dict by hand.
    wors = [(n, b, a) for n, b, a in worsened if n not in lost]

    def _capped(items, cap=6):
        shown = ', '.join(items[:cap])
        if len(items) > cap:
            shown += f", +{len(items) - cap} more"
        return f" [{shown}]" if items else ""

    head = ("IMPROVEMENT GATE: this run broke "
            f"{len(lost)} previously-connected net(s){_capped(lost)}, "
            f"worsened {len(wors)}"
            f"{_capped([f'{n} {b}->{a}' for n, b, a in wors])}, "
            f"connected {len(cmp['gained'])}")
    lines.append(head + f" -- {verdict.upper()}ED")
    if cmp['lost']:
        lines.append(f"  broken by this run: {', '.join(cmp['lost'])}")
    if worsened:
        lines.append("  more disconnected pads: " + ', '.join(
            f"{n} {b}->{a}" for n, b, a in worsened))
    excl = cmp.get('excluded_by_plan') or []
    if excl:
        lines.append(
            "  excluded (plane nets outside --nets, not repaired BY PLAN, "
            "so not judged): " + ', '.join(
                f"{n} {b}->{a}" for n, b, a in excl))
    if cmp['gained']:
        lines.append(f"  connected by this run: {', '.join(cmp['gained'])}")
    lines.append(f"  disconnected pads: {cmp['disconnected_pads_before']} "
                 f"-> {cmp['disconnected_pads_after']} "
                 f"over {cmp['nets_compared']} multi-pad net(s)")
    lines.append(f"  {action}")
    return "\n".join(lines)
