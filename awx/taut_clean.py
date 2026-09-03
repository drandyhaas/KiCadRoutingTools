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

import os

MARGIN = 0.30      # how far beyond an obstacle's end the detour point sits
SAMPLE = 0.25      # seg_clear's own sampling step
TOL = 0.05         # a graze under this is the push loop's own stopping slack
END_ZONE = 1.5     # mm from either end: the escape COMB, where every string
                   # violates -- stubs at 0.25 pitch are impassable for an
                   # obstacle model inflated by clearance + half a track,
                   # which the real router threads at exact clearance


def relax_from(init, obs, rounds=400):
    """topo_strings.relax's loop from a given initial polyline (the
    elastic band settles in the homotopy class of `init`). Returns
    (pts, iterations). relax(src, dst) == relax_from([src, dst])."""
    pts = ts.densify(list(init))
    ends = (pts[0], pts[-1])
    it = 0
    for it in range(rounds):
        moved = 0.0
        for i in range(1, len(pts) - 1):
            p = pts[i]
            if ts.d2(p, ends[0]) < ts.FREEZE ** 2 or \
                    ts.d2(p, ends[1]) < ts.FREEZE ** 2:
                continue
            q = (0.5 * p[0] + 0.25 * pts[i - 1][0] + 0.25 * pts[i + 1][0],
                 0.5 * p[1] + 0.25 * pts[i - 1][1] + 0.25 * pts[i + 1][1])
            for _k in range(6):
                v = obs.point_violation(q)
                if v is None:
                    break
                depth, (ux, uy) = v
                q = (q[0] + ux * (depth + 0.01), q[1] + uy * (depth + 0.01))
            moved += math.hypot(q[0] - p[0], q[1] - p[1])
            pts[i] = q
        if it % 25 == 24:
            pts = ts.densify(ts.shortcut(pts, obs))
        if moved < 1e-4 * len(pts):
            break
    return ts.densify(ts.shortcut(pts, obs)), it + 1


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


def _unit(v):
    h = math.hypot(v[0], v[1])
    return (v[0] / h, v[1] / h) if h > 1e-12 else (1.0, 0.0)


def reseeds(src, dst, who, at):
    """Initial polylines that go AROUND the offender on one side: past
    each END of a capsule (the two sectors a thin track separates), or
    past either side of a disc."""
    kind, geo = who
    outs = []
    if kind == 'cap':
        a, b, r, _n = geo
        ab = _unit((b[0] - a[0], b[1] - a[1]))
        for end, away in ((a, (-ab[0], -ab[1])), (b, ab)):
            e = (end[0] + away[0] * (r + MARGIN),
                 end[1] + away[1] * (r + MARGIN))
            outs.append([src, e, dst])
    else:
        x, y, r, _n = geo
        n = _unit((dst[1] - src[1], -(dst[0] - src[0])))
        for sgn in (1.0, -1.0):
            e = (x + sgn * n[0] * (r + MARGIN), y + sgn * n[1] * (r + MARGIN))
            outs.append([src, e, dst])
    return outs


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
    n_re = 0
    relax_clean.last = None
    # TAUT_RESEED: '0' assert-only (default); '1' reseed on any midfield
    # violation; 'disc' reseed only when a midfield DISC (a via -- an
    # obstacle on BOTH layers, which no lane can cross) is violated. A
    # capsule is a single-layer foreign track a lane crosses by diving,
    # so on a two-layer ribbon a chord across one is not wrong-sector.
    mode = os.environ.get('TAUT_RESEED', '0')
    reseed_on = mode in ('1', 'disc')
    for _level in range(depth):
        viol = violations(pts, obs)
        if not viol:
            return pts, it, ('clean' if n_re == 0 else 'reseeded'), n_re
        cls = classify(pts, obs, viol)
        actionable = cls['midfield_cap'] + cls['midfield_disc']
        if mode == 'disc':
            actionable = cls['midfield_disc']
        if not actionable:
            # comb-zone / shallow only: reported, never reseeded
            relax_clean.last = dict(cls=cls, n=len(viol))
            return pts, it, ('tolerated' if n_re == 0 else 'reseeded'), n_re
        if not reseed_on:
            # ASSERT-ONLY mode (default): the string is reported as
            # VIOLATING with its diagnosis; the reseed policy is gated
            # (TAUT_RESEED=1) until the ladder judges it -- first draw
            # at K35 moved 82v/1 open -> 86v/2 open with 22/35 reseeds
            relax_clean.last = dict(cls=cls, n=len(viol))
            return pts, it, 'violating', 0
        if mode == 'disc':
            # the deepest DISC violation drives the reseed
            a0, b0 = pts[0], pts[-1]
            dv = [(q, dq) for q, dq in viol
                  if dq > TOL and not (ts.d2(q, a0) < END_ZONE ** 2
                                       or ts.d2(q, b0) < END_ZONE ** 2)
                  and (offender(obs, q) or ('?',))[0] == 'disc']
            p, _d = max(dv, key=lambda v: v[1])
        else:
            p, _d = max(viol, key=lambda v: v[1])
        who = offender(obs, p)
        # diagnosis for the caller: what the string violates, how many
        # samples, how deep, and where along the string (fraction)
        kinds = {}
        for q, dq in viol:
            w = offender(obs, q)
            k = (w[0] + ':' + str(w[1][3])) if w else '?'
            kinds[k] = kinds.get(k, 0) + 1
        relax_clean.last = dict(
            cls=cls, n=len(viol), deepest=round(_d, 3), at=round(p[0], 2),
            at_y=round(p[1], 2), offenders=sorted(
                kinds.items(), key=lambda kv: -kv[1])[:4])
        if who is None:
            break
        cands = []
        for init in reseeds(src, dst, who, p):
            q, itq = relax_from(init, obs, rounds)
            vq = violations(q, obs)
            cands.append((len(vq), ts.polyline_len(q), q, itq))
        n_re += 1
        if not cands:
            break
        cands.sort(key=lambda c: (c[0], c[1]))
        nv, _L, pts, it = cands[0]
        if nv == 0:
            return pts, it, 'reseeded', n_re
    return pts, it, 'INVALID', n_re
