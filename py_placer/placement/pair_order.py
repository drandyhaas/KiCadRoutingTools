"""Pin-order inversions between connected part pairs: a LOWER BOUND, not a proxy.

The placement objective (wirelength, crossings, halos) is indifferent between a
part whose pins face its partner in matching order and the same part rotated
180 degrees -- the airwire lengths barely move -- yet the second one FORCES
crossings no router can remove. Test-board run 5 measured it: U3 (flash) at
rotation 0 put its QSPI pins in reversed order against the MCU's, the router
shipped 4/7, and rotating U3 180 degrees made the same nets route 7/7 on the
first attempt. The score never saw the difference; a human counting pin order
did (wk5 journal [3]).

The number computed here is that count, made mechanical. For a pair of parts
sharing m nets, project each part's shared-net escape pads onto the axis
PERPENDICULAR to the line joining the parts (the channel's cross-section), read
off the two net orders, and count order inversions between them:

  * ``inversions`` -- pairs of nets whose relative order differs between the
    two sides. In a two-sided channel each such pair must cross at least once,
    so this is a lower bound on forced crossings (Supowit 1987, "channel
    routing is NP-complete"; the crossing-minimal order argument goes back to
    Leiserson & Pinter 1983 on river routing). 0 = the pair can in principle
    route as a planar ribbon.
  * ``lis`` -- the longest increasing subsequence of one order against the
    other: the largest subset of nets routable with NO crossings at all
    (Supowit's theorem: max planar subset == LIS). ``m - lis`` nets must each
    cross something.

These are lower bounds on what ANY router must pay, which is what makes them
safe to rank poses by -- unlike a correlational proxy, improving them cannot be
gamed by the optimizer.

They are REPORTED, not optimised: being a lower bound rather than a
correlational proxy is an argument for trying them in an objective, not a
measurement that it helps. The consumer is the operator reading a report
(`board_context`'s pair rows).
"""
from __future__ import annotations

import bisect
import math
from typing import Dict, List, Optional, Tuple


def _merge_count(seq: List[int]) -> int:
    """Inversion count of an integer sequence, by merge sort (O(m log m))."""
    if len(seq) < 2:
        return 0
    mid = len(seq) // 2
    left, right = seq[:mid], seq[mid:]
    inv = _merge_count(left) + _merge_count(right)
    merged, i, j = [], 0, 0
    while i < len(left) and j < len(right):
        if left[i] <= right[j]:
            merged.append(left[i]); i += 1
        else:
            inv += len(left) - i
            merged.append(right[j]); j += 1
    seq[:] = merged + left[i:] + right[j:]
    return inv


def _lis_length(seq: List[int]) -> int:
    """Longest strictly increasing subsequence length (patience, O(m log m))."""
    tails: List[int] = []
    for v in seq:
        k = bisect.bisect_left(tails, v)
        if k == len(tails):
            tails.append(v)
        else:
            tails[k] = v
    return len(tails)


def _escape_pads(part, nets, toward_xy) -> Dict[int, Tuple[float, float]]:
    """One representative pad per shared net: the pad nearest the partner
    (the one that would actually escape into the channel). Deterministic
    tie-break by (x, y)."""
    tx, ty = toward_xy
    best: Dict[int, Tuple[float, float, float]] = {}
    for gx, gy, nid in part.pad_globals(part.x, part.y, part.rot):
        if nid not in nets:
            continue
        d = (gx - tx) ** 2 + (gy - ty) ** 2
        cur = best.get(nid)
        if cur is None or (d, gx, gy) < cur:
            best[nid] = (d, gx, gy)
    return {nid: (gx, gy) for nid, (_, gx, gy) in best.items()}


def _scoring_net_ids(state):
    """`set(state.net_refs)`, cached on the state.

    `pair_metrics` intersected against a freshly built `set(state.net_refs)` on
    EVERY call -- the whole board's net-id set, rebuilt once per partner per
    pose. It is pose-invariant: `net_refs` is assigned once in
    `QuenchState.__init__` and never mutated afterwards, and `part.nets` is
    filtered earlier in that same constructor, so both are frozen by the time
    anything here runs.
    """
    ids = getattr(state, '_pair_order_net_ids', None)
    if ids is None:
        ids = set(state.net_refs)
        try:
            state._pair_order_net_ids = ids
        except AttributeError:        # a caller's stand-in state with __slots__
            pass
    return ids


def _part_net_set(state, ref, part):
    """`set(part.nets)` intersected with the scoring nets, cached per ref."""
    cache = getattr(state, '_pair_order_part_nets', None)
    if cache is None:
        cache = {}
        try:
            state._pair_order_part_nets = cache
        except AttributeError:
            return set(part.nets) & _scoring_net_ids(state)
    got = cache.get(ref)
    if got is None:
        got = set(part.nets) & _scoring_net_ids(state)
        cache[ref] = got
    return got


def pair_metrics(state, ref_a: str, ref_b: str,
                 only_nets=None) -> Optional[Dict]:
    """{'inversions', 'lis', 'nets'} for one part pair at their live poses,
    or None when they share fewer than 2 scoring nets (one net cannot be out
    of order).

    ``only_nets`` (net ids) narrows the question to a SUBSET of the shared
    nets; None, the default, keeps the whole-interface question every existing
    caller asks. It exists because the two questions have different answers,
    and #891 is about the narrow one: on esp_prog U1 and USB1 share three nets
    and their overall order AGREES, while the two forming the USB differential
    pair are CROSSED -- USB1 pad 2 = /D_N sits north of pad 3 = /D_P, and U1
    pad 6 = /D_P sits north of pad 7 = /D_N. That parity cannot be fixed by
    rotating either part; only a mirror or a via hop flips it. Reporting the
    interface number alone hides the fact behind a third net, which is exactly
    how run 25 shipped without seeing it.
    """
    pa, pb = state.parts.get(ref_a), state.parts.get(ref_b)
    if pa is None or pb is None:
        return None
    shared = sorted(_part_net_set(state, ref_a, pa)
                    & _part_net_set(state, ref_b, pb))
    if only_nets is not None:
        keep = set(only_nets)
        shared = [n for n in shared if n in keep]
    if len(shared) < 2:
        return None
    ax, ay, bx, by = pa.x, pa.y, pb.x, pb.y
    ux, uy = bx - ax, by - ay
    n = math.hypot(ux, uy)
    if n < 1e-9:
        return None                      # coincident parts: no channel axis
    # Cross-section axis of the channel between the parts.
    vx, vy = -uy / n, ux / n
    ea = _escape_pads(pa, set(shared), (bx, by))
    eb = _escape_pads(pb, set(shared), (ax, ay))
    common = [nid for nid in shared if nid in ea and nid in eb]
    if len(common) < 2:
        return None
    return _order_metrics(common, ea, eb, vx, vy)


def _order_metrics(common, ea, eb, vx, vy) -> Dict:
    """The order comparison itself, given each side's escape pad per net.

    `common` is the net ids both sides carry, in ascending id order (the sort
    tie-break), `ea` / `eb` map each to its escape pad, and (vx, vy) is the
    channel's cross-section axis. Split out of `pair_metrics` so a pair whose
    two ends carry DIFFERENT net ids -- one conductor renamed by a series part,
    `composed_pair_metrics` -- is judged by this code rather than a copy.
    """
    order_a = sorted(common, key=lambda nid: (ea[nid][0] * vx + ea[nid][1] * vy,
                                              nid))
    rank_b = {nid: i for i, nid in enumerate(
        sorted(common, key=lambda nid: (eb[nid][0] * vx + eb[nid][1] * vy,
                                        nid)))}
    seq = [rank_b[nid] for nid in order_a]
    lis = _lis_length(seq)
    # #891. How many of the projections TIE, on either side. Both sorts above
    # fall back to the net id when two pads land on the same coordinate along
    # the channel axis, which is deterministic but arbitrary: it invents an
    # order the geometry does not have. Measured on the tracked esp_prog, U1's
    # /D_P and /D_N sit at the SAME y while the channel to USB1 runs along x,
    # so both project to one point and the pair reported `inversions 0` --
    # "AGREES" -- for a question that has no answer at this pose.
    #
    # Reported rather than resolved: the count is additive, every existing
    # consumer of `inversions` / `lis` is unaffected, and a reader that cares
    # about a two-net pair (where one tie makes the whole verdict meaningless)
    # can say UNDETERMINED instead of inheriting the tie-break's opinion.
    def _ties(esc):
        proj = sorted(round(esc[nid][0] * vx + esc[nid][1] * vy, 6)
                      for nid in common)
        return sum(1 for i in range(1, len(proj)) if proj[i] == proj[i - 1])
    return {'inversions': _merge_count(seq), 'lis': lis, 'nets': len(common),
            'ties': _ties(ea) + _ties(eb)}


def composed_pair_metrics(state, ref_a: str, ref_b: str,
                          nets_a, nets_b) -> Optional[Dict]:
    """`pair_metrics` for two parts that share NO net: the ends of a
    differential pair that runs through per-leg series parts (#1251).

    U1 -> C42/C43 -> GPDI1 carries one signal on four net ids: U1 holds
    FPDI_D0+/-, GPDI1 holds /gpdi/GPDI_D0+/-, and each cap holds one of
    each. `pair_metrics` asks about SHARED nets, so the two end parts have
    nothing to compare. Here ``nets_a[i]`` on `ref_a` and ``nets_b[i]`` on
    `ref_b` are named as one conductor; B's escape pads are re-keyed to A's
    net ids and judged by the same `_order_metrics`, on the axis between the
    two end parts. The series parts are not consulted: this is the end-to-end
    parity, the question a crossover (#1055) would have to answer.

    None when a side lacks an escape pad for two of the conductors, the
    parts coincide, or either ref is not a part of `state`.
    """
    pa, pb = state.parts.get(ref_a), state.parts.get(ref_b)
    if pa is None or pb is None or len(nets_a) != len(nets_b):
        return None
    own_a = _part_net_set(state, ref_a, pa)
    own_b = _part_net_set(state, ref_b, pb)
    alias = {nb: na for na, nb in zip(nets_a, nets_b)
             if na in own_a and nb in own_b}
    if len(alias) < 2 or len(set(alias.values())) != len(alias):
        return None
    ux, uy = pb.x - pa.x, pb.y - pa.y
    n = math.hypot(ux, uy)
    if n < 1e-9:
        return None                      # coincident parts: no channel axis
    vx, vy = -uy / n, ux / n
    ea = _escape_pads(pa, set(alias.values()), (pb.x, pb.y))
    eb = {alias[nid]: xy for nid, xy in
          _escape_pads(pb, set(alias), (pa.x, pa.y)).items()}
    common = [nid for nid in sorted(alias.values()) if nid in ea and nid in eb]
    if len(common) < 2:
        return None
    return _order_metrics(common, ea, eb, vx, vy)


def pair_inversions(state) -> Dict[Tuple[str, str], Dict]:
    """Metrics for EVERY connected part pair sharing >= 2 scoring nets.

    Keys are (ref_a, ref_b) with ref_a < ref_b. Pairs are discovered through
    ``state.net_refs`` (already filtered of ignored nets), so a plane-routed
    rail never manufactures a pair.
    """
    pairs = set()
    for refs in state.net_refs.values():
        for i, a in enumerate(refs):
            for b in refs[i + 1:]:
                pairs.add((a, b))
    out: Dict[Tuple[str, str], Dict] = {}
    for a, b in sorted(pairs):
        m = pair_metrics(state, a, b)
        if m is not None:
            out[(a, b)] = m
    return out
