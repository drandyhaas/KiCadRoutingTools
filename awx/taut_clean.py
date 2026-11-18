#!/usr/bin/env python3
"""#622 taut strings: assert CLEANLINESS, not convergence.

A relaxation iterates until its points stop moving, and a string that has
stopped can still stand inside copper. A chord seeded ACROSS a thin obstacle
(a foreign track's capsule) is a stable cycle: the neighbours are pushed to
opposite sides, the middle point is pushed off the axis and smoothed back
onto it, and the loop reports convergence. "The curve crosses the track" is
a property of the whole curve, which no pointwise step sees.

So after a relaxation the polyline is walked (its points AND the 0.25 mm
samples between them) and every violation is classified (`assess`): clean,
tolerated (in the escape comb, or shallow) or violating. Reported, never
repaired.
"""
import math

import topo_strings as ts


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
        return ts.d2(p, a0) < freeze * freeze or ts.d2(p, b0) < freeze * freeze
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
        elif ts.d2(q, a0) < END_ZONE * END_ZONE or ts.d2(q, b0) < END_ZONE * END_ZONE:
            out['end_zone'] += 1
        else:
            w = offender(obs, q)
            out['midfield_cap' if (w and w[0] == 'cap')
                else 'midfield_disc'] += 1
    return out


# the finding on the last polyline assessed: {'cls': its violations counted by kind (classify), 'n': how many they are},
# None when it is clean. assess sets it; the taut paths' producer (detect_buses.taut_paths) prints it
last = None


def assess(pts, it, obs):
    """The cleanliness verdict on a relaxed polyline: (pts, it, status,
    n_reseeds), status 'clean' / 'tolerated' / 'violating'."""
    global last
    n_re = 0
    last = None
    # assert-only: violations are classified and reported, never
    # reseeded (a reseed was tried and lost on the ladder). A capsule
    # is a single-layer foreign track a lane crosses by diving, so on a
    # two-layer ribbon a chord across one is not wrong-sector -- or a
    # side of a pad drawn as capsules, which the strings see as its disc
    # (taut_fast.pad_discs) and count here as a capsule.
    viol = violations(pts, obs)
    if not viol:
        return pts, it, 'clean', 0
    cls = classify(pts, obs, viol)
    last = dict(cls=cls, n=len(viol))
    if not (cls['midfield_cap'] + cls['midfield_disc']):
        # comb-zone / shallow only: reported, never reseeded
        return pts, it, 'tolerated', 0
    # the string is reported as VIOLATING with its diagnosis (a reseed
    # was tried and lost on the ladder: K35 82v/1 open -> 86v/2 open)
    return pts, it, 'violating', 0


