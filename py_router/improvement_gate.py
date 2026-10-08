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

from typing import Dict, List, Optional, Tuple


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
    segs_by_net, vias_by_net, zones_by_net = _copper_by_net(
        pcb_data, segs_by_net, vias_by_net, zones_by_net)
    out: Dict[int, Tuple[bool, int]] = {}
    for net_id, pads in (pcb_data.pads_by_net or {}).items():
        if not net_id or len(pads or []) < 2:
            continue          # net 0 pseudo-net / trivially connected
        broken, dis_pads = _grade_net(
            pcb_data, net_id, pads, segs_by_net.get(net_id, []),
            vias_by_net.get(net_id, []), zones_by_net.get(net_id, []),
            tolerance)
        out[net_id] = (not broken, len(dis_pads) if broken else 0)
    return out


def copper_signature(segments, vias, net_name) -> Dict[str, object]:
    """{net name: multiset of its copper} for "did this run change the net's
    copper?" -- the one question a per-net diff has to answer (#1069).

    Values, not object identity (a nudge moves a via object in place), keyed
    by NAME (two parses of one board may number nets differently), rounded to
    0.1 um so the writer's nm quantisation does not read as a change. A
    segment is direction-free. Net 0 is skipped.
    """
    from collections import Counter

    out: Dict[str, Counter] = {}
    for item in list(segments) + list(vias):
        name = net_name(item.net_id) if item.net_id else None
        if not name:
            continue
        out.setdefault(name, Counter())[copper_item_key(item)] += 1
    return out


def copper_item_key(item) -> tuple:
    """One segment's or via's key in copper_signature: its values, rounded to
    0.1 um, a segment direction-free."""
    def r(v):
        return round(float(v), 4)
    if hasattr(item, 'start_x'):
        a = (r(item.start_x), r(item.start_y))
        b = (r(item.end_x), r(item.end_y))
        return ('s', item.layer, min(a, b), max(a, b), r(item.width))
    return ('v', r(item.x), r(item.y), r(item.size), r(item.drill),
            tuple(item.layers or ()))


def _copper_by_net(pcb_data, segs_by_net, vias_by_net, zones_by_net):
    """Fill in whichever per-net copper maps the caller did not pass."""
    def _by_net(items):
        d: Dict[int, list] = {}
        for it in items:
            d.setdefault(it.net_id, []).append(it)
        return d

    if segs_by_net is None:
        segs_by_net = _by_net(pcb_data.segments)
    if vias_by_net is None:
        vias_by_net = _by_net(pcb_data.vias)
    if zones_by_net is None:
        zones_by_net = _by_net(getattr(pcb_data, 'zones', None) or [])
    return segs_by_net, vias_by_net, zones_by_net


def _grade_net(pcb_data, net_id, pads, segs, vias, zones, tolerance):
    """(broken, disconnected_pad_locations) for one net: the zone/fill-aware
    union-find route.py's own sweeps grade with, multi-board aware."""
    from check_connected import check_net_connectivity, net_break_within_outlines
    r = check_net_connectivity(net_id, segs, vias, pads, zones,
                               tolerance=tolerance, pcb_data=pcb_data)
    # #479 multi-board: only a break WITHIN one outline is a real break.
    broken, dis_pads = net_break_within_outlines(pcb_data, r)
    return bool(broken), (list(dis_pads or []) if broken else [])


def grade_nets(pcb_data, net_ids, tolerance: float = 0.02,
               segs_by_net: Optional[Dict[int, list]] = None,
               vias_by_net: Optional[Dict[int, list]] = None,
               zones_by_net: Optional[Dict[int, list]] = None
               ) -> Dict[int, Dict]:
    """Per-net detail for `net_ids` ONLY, on `net_connectivity_map`'s grade:
    {net_id: {'pads', 'broken', 'copper', 'failed_pads'}}, `failed_pads`
    shaped like route.py's failed_multipoint entries. Nets with fewer than two
    pads are skipped (trivially connected). route.py's final re-grade (#1069)
    uses it because it must grade the nets the run OWNS, never the whole
    board: a scoped step would otherwise report other steps' nets."""
    segs_by_net, vias_by_net, zones_by_net = _copper_by_net(
        pcb_data, segs_by_net, vias_by_net, zones_by_net)
    out: Dict[int, Dict] = {}
    for net_id in net_ids:
        pads = (pcb_data.pads_by_net or {}).get(net_id) or []
        if not net_id or len(pads) < 2:
            continue
        segs = segs_by_net.get(net_id, [])
        vias = vias_by_net.get(net_id, [])
        broken, dis = _grade_net(pcb_data, net_id, pads, segs, vias,
                                 zones_by_net.get(net_id, []), tolerance)
        out[net_id] = {
            'pads': len(pads),
            'broken': broken,
            'copper': bool(segs or vias),
            'failed_pads': [
                {'x': round(float(p[0]), 4), 'y': round(float(p[1]), 4),
                 'component_ref': p[3] if len(p) > 3 else '?',
                 'pad_number': '?'} for p in dis],
        }
    return out


def compare_connectivity(before: Dict[int, Tuple[bool, int]],
                         after: Dict[int, Tuple[bool, int]],
                         net_name: callable) -> Dict:
    """Per-net connectivity delta between two states of the same board.

    Only nets present in BOTH maps are compared: a net that exists in one
    reading and not the other is a parse/scope difference, not a routing
    outcome, and must not be able to trip the gate.

    `worsened` lists every compared net whose disconnected-pad count ROSE
    WITHOUT being newly broken (it was already open before the run), as
    (name, before, after) -- disjoint from `lost`, so a net is named once. A
    pad-count rejection used to name no net at all in that case, because the
    net is then not `lost`.
    """
    lost: List[str] = []
    gained: List[str] = []
    worsened: List[Tuple[str, int, int]] = []
    pads_before = pads_after = 0
    compared = 0
    for net_id, (conn_b, dis_b) in before.items():
        if net_id not in after:
            continue
        conn_a, dis_a = after[net_id]
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


def excluded_plane_attribution(before: Dict[int, Tuple[bool, int]],
                               after: Dict[int, Tuple[bool, int]],
                               net_name: callable,
                               excluded_names) -> Dict:
    """Which of a rejection's nets are zone nets the in-run finalize excluded
    BY PLAN, and whether the run is rejected on them ALONE (#1114).

    Since #562 the plane repair is the route step's own finalize, which runs
    before this gate -- but only over the zone nets in the step's --nets. A
    scoped round whose copper cuts a pour outside that scope ships the cut
    unrepaired, and the gate rejects the round on the plane net. The verdict
    is right; what the agent needs is to be TOLD that the plane net is the
    whole reason, and that carrying it in --nets lets the finalize repair the
    pour before the gate grades it.

    Returns {'nets': names among `excluded_names` that the run broke or
    worsened, 'alone': True when the verdict without them would accept}."""
    excluded = set(excluded_names or ())
    if not excluded:
        return {'nets': [], 'alone': False}
    ids = {nid for nid in before if net_name(nid) in excluded}
    hit = sorted(net_name(nid) for nid in ids if nid in after
                 and after[nid][1] > before[nid][1])
    if not hit:
        return {'nets': [], 'alone': False}
    rest = compare_connectivity(
        {k: v for k, v in before.items() if k not in ids},
        {k: v for k, v in after.items() if k not in ids}, net_name)
    return {'nets': hit, 'alone': gate_verdict(rest) == 'accept'}


def format_report(cmp: Dict, verdict: str, action: str) -> str:
    """The human line(s). Names the nets -- a count alone is not actionable,
    and the whole point of the gate is that the operator can see WHICH
    already-routed copper a rip took out."""
    lines = []
    # The head line NAMES what it judged on, each list at its OWN
    # clause (#1032). `broke 1 ... REJECTED` hid that the one net was GND; a
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
    if cmp['gained']:
        lines.append(f"  connected by this run: {', '.join(cmp['gained'])}")
    lines.append(f"  disconnected pads: {cmp['disconnected_pads_before']} "
                 f"-> {cmp['disconnected_pads_after']} "
                 f"over {cmp['nets_compared']} multi-pad net(s)")
    excl = cmp.get('excluded_plane_nets') or []
    if excl:
        # #1114: the plane net is outside --nets, so the finalize that would
        # have repaired its pour before this grade skipped it BY PLAN.
        lines.append(
            f"  zone net(s) outside this run's --nets, so NOT repaired by the "
            f"in-run finalize: {', '.join(excl)}"
            + (" -- the verdict rests on them ALONE"
               if cmp.get('rejected_on_excluded_plane_nets_alone') else ""))
    lines.append(f"  {action}")
    return "\n".join(lines)
