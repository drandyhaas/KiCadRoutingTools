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
gamed by the optimizer, which is also why they deliberately do NOT join
``quench.total_cost`` (docs/placement-optimization.md fact (a): proxies mislead
optimizers). The consumers are pose ranking (pose_score components) and the
operator reading a report.
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


def _escape_pads(part, nets, toward_xy, pose=None) -> Dict[int, Tuple[float, float]]:
    """One representative pad per shared net: the pad nearest the partner
    (the one that would actually escape into the channel). Deterministic
    tie-break by (x, y). `pose` overrides the part's live (x, y, rot)."""
    x, y, rot = pose if pose is not None else (part.x, part.y, part.rot)
    tx, ty = toward_xy
    best: Dict[int, Tuple[float, float, float]] = {}
    for gx, gy, nid in part.pad_globals(x, y, rot):
        if nid not in nets:
            continue
        d = (gx - tx) ** 2 + (gy - ty) ** 2
        cur = best.get(nid)
        if cur is None or (d, gx, gy) < cur:
            best[nid] = (d, gx, gy)
    return {nid: (gx, gy) for nid, (_, gx, gy) in best.items()}


def pair_metrics(state, ref_a: str, ref_b: str,
                 pose_a: Optional[Tuple[float, float, float]] = None,
                 pose_b: Optional[Tuple[float, float, float]] = None,
                 only_nets=None) -> Optional[Dict]:
    """{'inversions', 'lis', 'nets'} for one part pair, or None when they share
    fewer than 2 scoring nets (one net cannot be out of order).

    ``pose_a`` / ``pose_b`` override the live poses -- this is what makes the
    delta cheap: nothing on the state is mutated or rebuilt.

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
    shared = sorted(set(pa.nets) & set(pb.nets) & set(state.net_refs))
    if only_nets is not None:
        keep = set(only_nets)
        shared = [n for n in shared if n in keep]
    if len(shared) < 2:
        return None
    ax, ay = (pose_a[0], pose_a[1]) if pose_a is not None else (pa.x, pa.y)
    bx, by = (pose_b[0], pose_b[1]) if pose_b is not None else (pb.x, pb.y)
    ux, uy = bx - ax, by - ay
    n = math.hypot(ux, uy)
    if n < 1e-9:
        return None                      # coincident parts: no channel axis
    # Cross-section axis of the channel between the parts.
    vx, vy = -uy / n, ux / n
    ea = _escape_pads(pa, set(shared), (bx, by), pose_a)
    eb = _escape_pads(pb, set(shared), (ax, ay), pose_b)
    common = [nid for nid in shared if nid in ea and nid in eb]
    if len(common) < 2:
        return None
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


def ref_inversions(state, ref: str,
                   x: Optional[float] = None, y: Optional[float] = None,
                   rot: Optional[float] = None) -> int:
    """Total inversions over every pair involving `ref`, at an optional
    hypothetical pose. This is the pose-ranking number: cheap (only `ref`'s
    pairs are recomputed) and side-effect free."""
    part = state.parts.get(ref)
    if part is None:
        return 0
    pose = None
    if x is not None or y is not None or rot is not None:
        pose = (part.x if x is None else x,
                part.y if y is None else y,
                part.rot if rot is None else rot)
    partners = set()
    for nid in part.nets:
        partners.update(state.net_refs.get(nid, ()))
    partners.discard(ref)
    total = 0
    for other in sorted(partners):
        m = (pair_metrics(state, ref, other, pose_a=pose) if ref < other
             else pair_metrics(state, other, ref, pose_b=pose))
        if m is not None:
            total += m['inversions']
    return total


def inversions_delta(state, ref: str, x: float, y: float, rot: float) -> Dict:
    """Before/after inversion totals for moving `ref` to (x, y, rot), with the
    per-pair breakdown at the CANDIDATE pose. Recomputes only pairs involving
    `ref`; the state is never mutated."""
    part = state.parts.get(ref)
    if part is None:
        return {'before': 0, 'after': 0, 'delta': 0, 'pairs': {}}
    before = ref_inversions(state, ref)
    after = ref_inversions(state, ref, x, y, rot)
    pose = (x, y, rot)
    partners = set()
    for nid in part.nets:
        partners.update(state.net_refs.get(nid, ()))
    partners.discard(ref)
    pairs = {}
    for other in sorted(partners):
        m = (pair_metrics(state, ref, other, pose_a=pose) if ref < other
             else pair_metrics(state, other, ref, pose_b=pose))
        if m is not None:
            pairs[tuple(sorted((ref, other)))] = m
    return {'before': before, 'after': after, 'delta': after - before,
            'pairs': pairs}
