#!/usr/bin/env python3
"""Rank candidate (x, y, rotation) poses for a part, nearest legal first.

    legality of a pose      QuenchState.candidate_valid
    the rotations to try    ROTATIONS = (0, 90, 180, 270)
    does it actually route  route.py --nets <affected> --json-out

`rank_poses` sweeps a lattice around the part's pose, drops every pose the
seat gate refuses, and returns the rest nearest first -- the least
disturbance that is legal. It scores no objective: only the routed result
judges a placement, and the caller (`place_pose --snap` / `--near`) re-grades
every candidate before it writes one.

    poses = rank_poses(pcb_data, board_path, 'U3', radius=2.0)
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
from typing import Dict, List, Optional, Sequence

ROTATIONS: Sequence[float] = (0.0, 90.0, 180.0, 270.0)


def _offsets(radius: float, step: float):
    """Square-ring offsets out to `radius`, ring by ring, each ring nearest
    first. A ring's corner lies further out than the next ring's edge, so a
    caller that needs true distance order sorts (`rank_poses` does)."""
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


def make_state(pcb_data, board_path: str, *, clearance: Optional[float] = None,
               board_edge_clearance: Optional[float] = None, extra_locked_refs=None,
               keepouts=None, intent_zones=None, exclusive_zones=None,
               estimate_courtyards=False, estimate_skip=()):
    """A QuenchState for `pcb_data`: its parts, their poses and the seat
    gate. `place_simple` keeps one for the whole run; the pose tools build
    one per call. A floor left None is the board's own
    (`floorplan.resolved_floors`, #1301), never the fixed (0.25, 0.55) it
    used to default to."""
    from placement.floorplan import resolved_floors
    from placement.quench import QuenchState
    clearance, board_edge_clearance = resolved_floors(
        board_path, clearance, board_edge_clearance)
    return QuenchState(
        pcb_data=pcb_data, pcb_file=board_path,
        clearance=clearance, board_edge_clearance=board_edge_clearance,
        extra_locked_refs=extra_locked_refs,
        # The intent's keep-outs, for the seat gate. Passed through rather
        # than read here: this factory has no intent, so every caller that
        # has one hands it over, and a caller that has none gets the inert
        # default.
        keepouts=keepouts,
        # Declared zones: containment, gated MONOTONE by `candidate_valid`
        # (`QuenchState.intent_ok`) -- a pose inside the zone, or no further
        # outside it than the part's current pose.
        intent_zones=intent_zones,
        # The EXCLUSIVE slice, a must-be-OUTSIDE claim gated absolutely
        # (see `quench.exclusive_spec`).
        exclusive_zones=exclusive_zones,
        # #1300: a courtyard-less part seated on an estimated courtyard;
        # `place_simple` asks, a grading caller does not.
        estimate_courtyards=estimate_courtyards, estimate_skip=estimate_skip)


class PoseUnrankable(KeyError):
    """A ref `rank_poses` cannot rank, with the reason and the exit code a CLI
    should use (#959, #999).

    It was a bare `KeyError`, so a pad-less logo block exited 1 through a
    traceback -- the SAME code as the verdict "no legal pose, including
    staying put". A crash and a verdict must not share a code.

    `code` follows `pose_ops.PoseRefusal`: 2 when the ref names nothing on this
    board (a typo the caller fixes by rewriting the command), 4 when it names a
    real block the placement state cannot move (a measurement the caller acts
    on). Still a `KeyError` subclass, so a caller that caught the old error
    keeps working; `__str__` is the plain reason rather than KeyError's quoted
    repr.
    """

    def __init__(self, reason: str, code: int, why: str = ''):
        super().__init__(reason)
        self.reason = reason
        self.code = code
        #: The bare cause, without the advice `reason` appends -- a caller
        #: that is itself `place_pose set` must not be told to run it.
        self.why = why or reason

    def __str__(self):
        return self.reason


def _turn(a, b) -> float:
    """The smaller angle (degrees) between rotations `a` and `b`."""
    d = abs(float(a) - float(b)) % 360.0
    return min(d, 360.0 - d)


def _same_rot(a, b) -> bool:
    return _turn(a, b) <= 1e-6


def veto_phrase(dropped_by: Dict, top: int = 3) -> str:
    """`rank_poses`' `dropped_by` as one line, most frequent check first:
    `pads 290 (C12 88, C13 61, C9 40); courtyard 33 (U2 20)` (#1113)."""
    out = []
    for chk, d in list((dropped_by or {}).items())[:top]:
        blk = ', '.join(f"{r} {n}" for r, n in d.get('blockers') or ())
        out.append(f"{chk} {d['count']}" + (f" ({blk})" if blk else ''))
    return '; '.join(out) or 'nothing'


def rank_poses(pcb_data, board_path: str, ref: str, *, radius: float = 2.0,
               step: float = 0.5, rotations: Sequence[float] = ROTATIONS,
               limit: int = 12, state=None,
               diagnostics: Optional[Dict] = None,
               **state_kw) -> List[Dict]:
    """Legal poses for `ref`, nearest its current pose first.

    Returns `[{x, y, rot, dist_mm, legal}]` (plus `escape: True` on a pose
    only the escape branch admits, below). `dist_mm` is the straight-line
    distance from the part's CURRENT position (the sweep's centre); the order
    is `dist_mm`, then the smallest turn from the current rotation, then
    rotation, y and x, so the same board gives the same list.

    Illegal poses are dropped: returning one ranked would invite a caller to
    try it. They are not dropped SILENTLY (run-7 S4): pass `diagnostics={}` and
    it comes back with `dropped_total` and `dropped_in_place` -- the rotations
    at the part's OWN (x, y) that candidate_valid vetoed. An empty ranked list
    whose dropped_in_place includes the part's current rotation says "the
    knobs veto even staying put", which is a knob problem, not a pose answer.

    #1113: and it says WHICH check vetoed each one -- `dropped_by` ({check:
    {count, blockers: [[ref, n], ...] (top 3), blockers_distinct}}, from
    `QuenchState.candidate_veto`, the same predicate candidate_valid is),
    `dropped_in_place_by` ([{rot, check, blocker}]), `evaluated_total`, and
    `all_moves_vetoed`: nothing survived but the part's own pose. A part
    whose rotation its neighbours were packed around reads exactly that way,
    and a one-part move cannot judge such a rotation (`veto_phrase` says it
    in one line).

    `legal` is TRUE only for a pose the ordinary gate admits. For a part
    whose current pose (or the incumbent a caller pinned, `incumbent_at`) is
    off the board, `candidate_valid`'s escape branch also admits poses merely
    LESS off the board, with no overlap, so the part can come home; those
    are ranked too, but labelled `legal: False, escape: True`, and counted
    in `diagnostics['escape_admitted']`. They were labelled `legal: True`:
    with esp_prog's R3 staged at (146.0, 101.5), a radius-3 ranking returned
    (146.0, 104.5) at 270 degrees -- both pads past the east edge -- as
    legal (found reviewing #1277).
    """
    st = state if state is not None else make_state(pcb_data, board_path, **state_kw)
    part = st.parts.get(ref) if hasattr(st, 'parts') else None
    if part is None:
        fp = (getattr(pcb_data, 'footprints', None) or {}).get(ref)
        if fp is None:
            raise PoseUnrankable(
                f"{ref} is not a footprint block on this board", code=2,
                why='it is not a footprint block on this board')
        from kicad_parser import non_aperture_pads
        why = ('it has no pads and no courtyard, so the placement state '
               'carries no geometry for it' if not non_aperture_pads(fp) else
               'the placement state does not carry it as a movable part')
        raise PoseUnrankable(
            f"{ref} cannot be ranked: {why}. Place it with `place_pose set` "
            f"and lock it", code=4, why=why)

    x0, y0, rot0 = part.x, part.y, part.rot
    rots = list(rotations)

    scored = []
    dropped_total = 0
    dropped_in_place = []
    dropped_by: Dict[str, Dict] = {}
    dropped_in_place_by = []
    evaluated = 0
    # ONE labelled predicate call per candidate; the label below asks again,
    # of an ADMITTED pose only, with the escape branch closed.
    _veto = getattr(st, 'candidate_veto', None)
    if _veto is None:       # a duck-typed state without the #1113 labels
        def _veto(r, x, y, rot):
            return None if st.candidate_valid(r, x, y, rot) else (
                'unattributed', None)
    # Is the escape branch open for `ref`? Its incumbent does not change
    # during the sweep (no candidate is applied), so ask once. A
    # duck-typed state without the branch's test admits only what it calls
    # legal, as before.
    _esc = getattr(st, 'escape_board_term', None)
    escape_open = _esc is not None and _esc(ref) is not None
    escape_admitted = 0
    for dx, dy in _offsets(radius, step):
        # THE SWEEP, which `limit` does not bound: it truncates the sorted
        # RESULT on the last line of this function. `_offsets` yields flat
        # POINTS, not rings, so this runs once per candidate position.
        for rot in rots:
            x, y = x0 + dx, y0 + dy
            evaluated += 1
            veto = _veto(ref, x, y, rot)
            if veto is not None:
                dropped_total += 1
                _chk, _blk = veto
                _d = dropped_by.setdefault(_chk, {'count': 0, '_b': {}})
                _d['count'] += 1
                if _blk is not None:
                    _d['_b'][_blk] = _d['_b'].get(_blk, 0) + 1
                if dx == 0.0 and dy == 0.0:
                    dropped_in_place.append(rot)
                    dropped_in_place_by.append(
                        {'rot': rot, 'check': _chk, 'blocker': _blk})
                continue
            legal = True
            if escape_open:
                st.escape_closed.add(ref)
                try:
                    legal = bool(st.candidate_valid(ref, x, y, rot))
                finally:
                    st.escape_closed.discard(ref)
            row = {
                'x': round(x, 4), 'y': round(y, 4), 'rot': rot,
                'dist_mm': round((dx * dx + dy * dy) ** 0.5, 4),
                'legal': legal,
            }
            if not legal:
                # Admitted by the escape branch alone: less off the board.
                row['escape'] = True
                escape_admitted += 1
            scored.append(row)
    if diagnostics is not None:
        diagnostics['dropped_total'] = dropped_total
        diagnostics['escape_admitted'] = escape_admitted
        diagnostics['dropped_in_place'] = dropped_in_place
        diagnostics['dropped_in_place_by'] = dropped_in_place_by
        diagnostics['evaluated_total'] = evaluated
        diagnostics['dropped_by'] = {
            chk: {'count': d['count'],
                  'blockers': [[r, n] for r, n in sorted(
                      d['_b'].items(), key=lambda kv: (-kv[1], kv[0]))[:3]],
                  'blockers_distinct': len(d['_b'])}
            for chk, d in sorted(dropped_by.items(),
                                 key=lambda kv: (-kv[1]['count'], kv[0]))}
        # Nothing survived but staying put (or nothing at all): the reading
        # a one-part move gives a part its neighbours were packed around.
        diagnostics['all_moves_vetoed'] = bool(dropped_total) and all(
            p['dist_mm'] == 0 and _same_rot(p['rot'], rot0) for p in scored)
        diagnostics['in_place_evaluated'] = any(
            _same_rot(r, rot0) for r in rots)
        diagnostics['input_rotation'] = rot0
    # least disturbance: distance, then the smallest turn, then a stable order
    scored.sort(key=lambda p: (p['dist_mm'], _turn(p['rot'], rot0), p['rot'],
                               p['y'], p['x']))
    return scored[:limit]


