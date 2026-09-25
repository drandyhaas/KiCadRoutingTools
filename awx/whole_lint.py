"""whole_lint.py PLAN.json -- a snapped plan's own invariants, before any audit: what whole_snap PROMISES.
  grid     every vertex a router grid point (the exact tooth and berth excepted)
  angle    every piece 0/45/90 degrees (the joins to the exact tooth and berth excepted)
  join     a join to the tooth / berth no longer than two diagonal grid steps
  gap      consecutive pieces meet
  zero     no zero-length piece
  reverse  no turn sharper than a right angle between consecutive pieces
  stub     the first (last) track's width of the lane runs within 90 degrees of its stub (no fold at the end)
  via      every layer change at one of the lane's vias, every via at a layer change"""
import sys, json, math, collections
geo = json.load(open(sys.argv[1]))
RU = geo['rules']; g = RU['grid']; TW = RU['track']
EPS = 1e-6
bad = collections.defaultdict(list)
on = lambda v: abs(v / g - round(v / g)) < 1e-4
ang = lambda p: math.atan2(p[3] - p[1], p[2] - p[0])
ln = lambda p: math.hypot(p[2] - p[0], p[3] - p[1])
vias = collections.defaultdict(list)
for v in geo['vias']:
    vias[v[0]].append((v[1], v[2]))


def chord_dir(pcs, fwd, L):
    """the direction from the lane's end to the point L along it (fwd: from the tooth; else from the berth)"""
    seq = pcs if fwd else [(p[2], p[3], p[0], p[1], p[4]) for p in reversed(pcs)]
    left, (x0, y0) = L, (seq[0][0], seq[0][1])
    for p in seq:
        l_ = ln(p)
        if l_ >= left:
            f = left / l_ if l_ else 0
            x1, y1 = p[0] + (p[2] - p[0]) * f, p[1] + (p[3] - p[1]) * f
            return (x1 - x0, y1 - y0)
        left -= l_
    return (seq[-1][2] - x0, seq[-1][3] - y0)


for n, L in geo['lanes'].items():
    pcs = L['pieces']
    if not pcs:
        continue
    for i, p in enumerate(pcs):
        first, last = i == 0, i == len(pcs) - 1
        if ln(p) < EPS:
            bad['zero'].append((n, p[:2]))
            continue
        a = math.degrees(ang(p)) % 45
        if min(a, 45 - a) > 1e-3 and not (first or last):
            bad['angle'].append((n, p[:2], round(math.degrees(ang(p)), 1)))
        if (first or last) and ln(p) > 2 * g * math.sqrt(2) + EPS and min(a, 45 - a) > 1e-3:
            bad['join'].append((n, p[:2], round(ln(p), 3)))
        for k, (x, y) in enumerate(((p[0], p[1]), (p[2], p[3]))):
            if (first and k == 0) or (last and k == 1):
                continue
            if not (on(x) and on(y)):
                bad['grid'].append((n, (round(x, 4), round(y, 4))))
        if i:
            q = pcs[i - 1]
            if math.hypot(q[2] - p[0], q[3] - p[1]) > EPS:
                bad['gap'].append((n, p[:2]))
            if ln(q) > EPS:
                t = math.degrees((ang(p) - ang(q) + math.pi) % (2 * math.pi) - math.pi)
                if abs(t) > 90 + 1e-3:
                    bad['reverse'].append((n, (round(p[0], 3), round(p[1], 3)), round(t)))
            if q[4] != p[4] and not any(math.hypot(vx - p[0], vy - p[1]) < 1e-4 for vx, vy in vias[n]):
                bad['via'].append((n, 'layer change with no via', (round(p[0], 3), round(p[1], 3))))
    chg = [(p[0], p[1]) for q, p in zip(pcs, pcs[1:]) if q[4] != p[4]]
    for vx, vy in vias[n]:
        if not any(math.hypot(vx - x, vy - y) < 1e-4 for x, y in chg):
            bad['via'].append((n, 'via with no layer change', (round(vx, 3), round(vy, 3))))
    td = geo.get('tdir', {}).get(n)
    if td:
        for fwd, e in ((True, td[0]), (False, [-td[1][0], -td[1][1]])):
            d = chord_dir(pcs, fwd, TW)
            dd = math.hypot(*d)
            if not fwd:
                d = (-d[0], -d[1])
            if dd > EPS and (d[0] * e[0] + d[1] * e[1]) / dd < -1e-6:     # more than 90 degrees off: a fold
                bad['stub'].append((n, 'tooth' if fwd else 'berth',
                                    round(math.degrees(math.acos(max(-1, min(1, (d[0] * e[0] + d[1] * e[1]) / dd))))), 'deg off'))
for k, v in bad.items():
    for row in v[:12]:
        print('LINT', k, *row)
    if len(v) > 12:
        print('LINT', k, f'... {len(v) - 12} more')
print('LINT ' + (', '.join(f'{k} {len(v)}' for k, v in sorted(bad.items())) or 'clean'))
