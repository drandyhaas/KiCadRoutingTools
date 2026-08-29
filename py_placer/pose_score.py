#!/usr/bin/env python3
"""Score candidate (x, y, rotation) poses for a part, cheapest evidence first.

The pieces to answer "where should this part go, and facing which way" all
exist and none of them is joined up:

    legality of a pose      QuenchState.candidate_valid
    cost of a placement     QuenchState.total_cost  (wirelength, crossings,
                                                     halo, edge, alignment)
    the rotations to try    quench.ROTATIONS = [0, 90, 180, 270]
    does it actually route  route.py --nets <affected> --json-out

What did not exist is a function that takes a part and returns its candidate
poses RANKED. The one multi-pose scorer in the tree holds position fixed and
varies only rotation, so a part that needs to move 1.5 mm and turn 90 degrees
has never been scored as a single decision.

This module is deliberately only tiers 1 and 2 -- milliseconds, no router. Tier
3 (a scoped route of just the affected nets) and tier 4 (the full chain) belong
to the caller, because only the caller knows what "affected" means for its
board. The point of ranking here is that tier 3 is seconds and tier 4 is
minutes: you want to spend them on two candidates, not forty.

    poses = rank_poses(pcb_data, board_path, 'U3', radius=2.0)
    for p in poses[:2]:
        ...route the affected nets from p and compare...
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
from typing import Dict, List, Optional, Sequence

ROTATIONS: Sequence[float] = (0.0, 90.0, 180.0, 270.0)


def _offsets(radius: float, step: float):
    """Concentric ring offsets out to `radius`, nearest first.

    Nearest-first matters: a tie between two equally-costed poses should go to
    the one that disturbs the board least, and the caller usually stops early.
    """
    out = [(0.0, 0.0)]
    n = max(1, int(round(radius / step)))
    for ring in range(1, n + 1):
        r = ring * step
        ring_pts = []
        k = max(1, int(round(r / step)))
        for i in range(-k, k + 1):
            for j in range(-k, k + 1):
                if max(abs(i), abs(j)) != k:
                    continue                      # perimeter of this ring only
                ring_pts.append((i * step, j * step))
        ring_pts.sort(key=lambda d: (d[0] * d[0] + d[1] * d[1]))
        out.extend(ring_pts)
    return out


def make_state(pcb_data, board_path: str, *, clearance: float = 0.25,
               board_edge_clearance: float = 0.55, grid_step: float = 0.1,
               crossing_penalty: float = 30.0, length_weight: float = 0.3,
               halo_base: float = 0.5, halo_coef: float = 0.15,
               halo_weight: float = 2.0, edge_halo: float = 2.0,
               edge_weight: float = 2.0,
               ignore_net_ids=None, net_weights=None, move_refs=None,
               extra_locked_refs=None, keepouts=None):
    """A QuenchState used purely as an oracle -- built, queried, thrown away.

    Defaults mirror the placement guidance rather than quench's own library
    defaults, so a pose ranked here is ranked by the same objective a real
    place_optimize run would optimise.
    """
    from placement.quench import QuenchState
    return QuenchState(
        pcb_data=pcb_data, pcb_file=board_path,
        clearance=clearance, board_edge_clearance=board_edge_clearance,
        crossing_penalty=crossing_penalty, halo_base=halo_base,
        halo_coef=halo_coef, halo_weight=halo_weight, edge_halo=edge_halo,
        edge_weight=edge_weight, grid_step=grid_step,
        length_weight=length_weight, ignore_net_ids=ignore_net_ids,
        net_weights=net_weights, move_refs=move_refs,
        extra_locked_refs=extra_locked_refs,
        # #701: the intent's keep-outs, for the SEAT predicate. Passed through
        # rather than read here, because this factory has no intent -- every
        # caller that has one hands it over, and a caller that has none gets
        # the inert default.
        keepouts=keepouts)


def rank_poses(pcb_data, board_path: str, ref: str, *, radius: float = 2.0,
               step: float = 0.5, rotations: Sequence[float] = ROTATIONS,
               limit: int = 12, allow_rotations: bool = True,
               state=None, diagnostics: Optional[Dict] = None,
               cancel_check=None,
               **state_kw) -> List[Dict]:
    """Legal poses for `ref`, cheapest first.

    Returns `[{x, y, rot, cost, delta, dist_mm, legal}]`, sorted by cost. `delta`
    is against the part's CURRENT pose, so a negative delta is an improvement
    and a list whose best delta is >= 0 says "leave it alone" -- which is a real
    answer and one a caller should be able to get without paying for a route.

    Illegal poses are dropped, not scored: an illegal pose has no meaningful
    cost, and returning one ranked would invite a caller to try it. They are
    no longer dropped SILENTLY, though (run-7 S4): pass `diagnostics={}` and
    it comes back with `dropped_total` and `dropped_in_place` -- the rotations
    at the part's OWN (x, y) that candidate_valid vetoed. An empty ranked list
    whose dropped_in_place includes the part's current rotation says "the
    knobs veto even staying put", which is a knob problem, not a pose answer.
    """
    st = state if state is not None else make_state(pcb_data, board_path, **state_kw)
    part = st.parts.get(ref) if hasattr(st, 'parts') else None
    if part is None:
        raise KeyError(f"{ref} is not a movable part on this board")

    x0, y0, rot0 = part.x, part.y, part.rot
    # total_cost() returns the objective AND its components (length, crossings,
    # halo, edge, hpwl, align, orient). Rank on 'total', but carry the breakdown:
    # "better overall, worse on crossings" is exactly the kind of trade a caller
    # needs to see before spending a routing run on it.
    base = st.total_cost()
    rots = list(rotations) if allow_rotations else [rot0]

    # Pin-order inversions (run-6, placement/pair_order.py): a LOWER BOUND on
    # crossings a router cannot remove, carried as a component but NOT in the
    # ranked total -- the objective must not chase a bound it can't trade off
    # (fact (a): proxies mislead optimizers). rot-0 vs rot-180 ties in `cost`
    # are exactly where this number decides (test-board U3: 9 -> 0).
    from placement.pair_order import ref_inversions
    base_inv = ref_inversions(st, ref)

    scored = []
    dropped_total = 0
    dropped_in_place = []
    for dx, dy in _offsets(radius, step):
        # THE SWEEP, which `--limit` never bounded: limit truncates the sorted
        # RESULT on the last line of this function, while every candidate here
        # has already paid a full total_cost() plus ref_inversions(). A 30mm /
        # 0.25mm ring set is 231,392 candidates and --limit 12 evaluates all of
        # them. `_offsets` yields flat POINTS, not rings, so this runs once
        # per candidate position (58,081 for r=30/s=0.25) -- finer than a
        # ring and still negligible against a full cost evaluation.
        if cancel_check is not None and cancel_check():
            if diagnostics is not None:
                diagnostics['stopped_early'] = True
            break
        for rot in rots:
            x, y = x0 + dx, y0 + dy
            if not st.candidate_valid(ref, x, y, rot):
                dropped_total += 1
                if dx == 0.0 and dy == 0.0:
                    dropped_in_place.append(rot)
                continue
            st.apply_move(ref, x, y, rot)
            cost = st.total_cost()
            st.apply_move(ref, x0, y0, rot0)          # always restore
            inv = ref_inversions(st, ref, x, y, rot)
            scored.append({
                'x': round(x, 4), 'y': round(y, 4), 'rot': rot,
                'cost': round(cost['total'], 4),
                'delta': round(cost['total'] - base['total'], 4),
                'dist_mm': round((dx * dx + dy * dy) ** 0.5, 4),
                'legal': True,
                'components': dict(
                    {k: round(v, 4) for k, v in cost.items() if k != 'total'},
                    inversions=inv),
                'component_delta': dict(
                    {k: round(v - base.get(k, 0), 4)
                     for k, v in cost.items() if k != 'total'},
                    inversions=inv - base_inv),
            })
    if diagnostics is not None:
        diagnostics['dropped_total'] = dropped_total
        diagnostics['dropped_in_place'] = dropped_in_place
    # cost, then least disturbance, then a stable rotation order
    scored.sort(key=lambda p: (p['cost'], p['dist_mm'], p['rot']))
    return scored[:limit]


def best_pose(pcb_data, board_path: str, ref: str, *, min_gain: float = 1e-6,
              **kw) -> Optional[Dict]:
    """The single best pose, or None when nothing beats staying put.

    `min_gain` is hysteresis: a pose that is better by a rounding error is not
    better, and accepting one turns a convergence loop into a random walk.
    """
    poses = rank_poses(pcb_data, board_path, ref, **kw)
    if not poses:
        return None
    top = poses[0]
    return top if top['delta'] < -abs(min_gain) else None
