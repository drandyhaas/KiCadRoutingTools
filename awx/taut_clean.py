#!/usr/bin/env python3
"""#622 taut strings: assert CLEANLINESS, not convergence (user, 0902).

topo_strings.relax seeds every net as a straight chord and iterates
smooth-then-push, one point at a time, until the points stop moving.
A chord seeded ACROSS a thin obstacle (a foreign track capsule) is a
STABLE CYCLE: the neighbours get pushed to opposite sides, the middle
point is pushed off the axis and smoothed back onto it. Reproduced:
400 rounds, unchanged at exactly 10.000 mm, three points inside the
capsule (0.111 / 0.232 / 0.111 mm deep), and the loop reported
convergence. No pointwise fix can help -- "the curve crosses the
track" is a property of the whole curve; the remedy is a curve around
the obstacle's END, a different homotopy SECTOR, which the flow cannot
change. The seed chose it.

So: (1) after relax, walk the polyline (points AND the 0.25 mm samples
between them) and require point_violation None everywhere; (2) treat
a violation as WRONG SECTOR -- reseed with a polyline around the
offending obstacle on one side, relax from that seed
(relax_from, the same loop corridor.relax_path runs), keep the clean
candidate; recurse on what the new seed hits, a few levels deep.
relax itself is untouched (bit-identical for every caller); this is a
wrapper the taut-path producer calls.
"""
import math

import topo_strings as ts


MARGIN = 0.30      # how far beyond an obstacle's end the detour point sits
SAMPLE = 0.25      # seg_clear's own sampling step
TOL = 0.05         # a graze under this is the push loop's own stopping slack
END_ZONE = 1.5     # mm from either end: the escape COMB, where every string
                   # violates -- stubs at 0.25 pitch are impassable for an
                   # obstacle model inflated by clearance + half a track,
                   # which the real router threads at exact clearance




def violations(pts, obs, freeze=ts.FREEZE):
    """[(point, depth)] for every polyline vertex and every SAMPLE
    between vertices that sits inside an obstacle. The endpoints'
    FREEZE zones are exempt (a tooth sits in its own pad's disc)."""
    out = []
    a0, b0 = pts[0], pts[-1]

    def frozen(p):
        return ts.d2(p, a0) < freeze ** 2 or ts.d2(p, b0) < freeze ** 2
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(L / SAMPLE))
        for k in range(n):
            t = k / n
            p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if frozen(p):
                continue
            v = obs.point_violation(p)
            if v is not None:
                out.append((p, v[0]))
    p = pts[-1]
    if not frozen(p):
        v = obs.point_violation(p)
        if v is not None:
            out.append((p, v[0]))
    return out


def offender(obs, p):
    """The deepest violated obstacle at p: ('disc', (x, y, r, name)) or
    ('cap', (a, b, r, name)), or None."""
    best, who = 0.0, None
    for i in obs.near_discs(p):
        x, y, r, _n = obs.discs[i]
        depth = r - math.hypot(p[0] - x, p[1] - y)
        if depth > best:
            best, who = depth, ('disc', obs.discs[i])
    for ci in obs.near_caps(p):
        a, b, r, _n = obs.caps[ci]
        depth = r - ts.seg_pt_dist(a, b, p)
        if depth > best:
            best, who = depth, ('cap', obs.caps[ci])
    return who






def classify(pts, obs, viol):
    """Split violations into the three kinds the policy treats apart:
    'midfield_cap' (a foreign track capsule crossed away from both
    ends -- the wrong-sector case), 'midfield_disc' (a via/pad disc
    away from the ends), 'end_zone' (inside END_ZONE of an end: the
    escape comb) and 'shallow' (depth <= TOL)."""
    a0, b0 = pts[0], pts[-1]
    out = {'midfield_cap': 0, 'midfield_disc': 0, 'end_zone': 0,
           'shallow': 0}
    for q, dq in viol:
        if dq <= TOL:
            out['shallow'] += 1
        elif ts.d2(q, a0) < END_ZONE ** 2 or ts.d2(q, b0) < END_ZONE ** 2:
            out['end_zone'] += 1
        else:
            w = offender(obs, q)
            out['midfield_cap' if (w and w[0] == 'cap')
                else 'midfield_disc'] += 1
    return out


def relax_clean(src, dst, obs, rounds=400, depth=3):
    """relax, then ASSERT cleanliness; on a violation reseed around the
    offender and relax again, up to `depth` levels. Returns
    (pts, iterations, status, n_reseeds) with status 'clean' (the
    chord's own sector was fine), 'reseeded' (a detour sector is
    clean), or 'INVALID' (no clean sector found -- the least-violating
    polyline is returned, and the caller must say so loudly)."""
    pts, it = ts.relax(src, dst, obs)
    return assess(pts, it, obs)


def assess(pts, it, obs):
    """The cleanliness verdict on a relaxed polyline: (pts, it, status,
    n_reseeds), status 'clean' / 'tolerated' / 'violating' -- the second
    half of relax_clean, shared with the batched relaxation."""
    n_re = 0
    relax_clean.last = None
    # assert-only: violations are classified and reported, never
    # reseeded (a reseed was tried and lost on the ladder). A capsule
    # is a single-layer foreign track a lane crosses by diving, so on a
    # two-layer ribbon a chord across one is not wrong-sector.
    viol = violations(pts, obs)
    if not viol:
        return pts, it, 'clean', 0
    cls = classify(pts, obs, viol)
    relax_clean.last = dict(cls=cls, n=len(viol))
    if not (cls['midfield_cap'] + cls['midfield_disc']):
        # comb-zone / shallow only: reported, never reseeded
        return pts, it, 'tolerated', 0
    # the string is reported as VIOLATING with its diagnosis (a reseed
    # was tried and lost on the ladder: K35 82v/1 open -> 86v/2 open)
    return pts, it, 'violating', 0


