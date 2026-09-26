"""whole_lint.py PLAN.json -- a snapped plan's own invariants, before any audit: what whole_snap PROMISES.
  grid     every vertex a router grid point (the exact tooth and berth excepted)
  angle    every piece 0/45/90 degrees (the joins to the exact tooth and berth excepted)
  join     a join to the tooth / berth no longer than two diagonal grid steps
  gap      consecutive pieces meet
  zero     no zero-length piece
  reverse  no turn sharper than a right angle between consecutive pieces
  stub     the first (last) track's width of the lane runs within 90 degrees of its stub (no fold at the end)
  via      every layer change at one of the lane's vias, every via at a layer change
  pair     a pair moves as the pair router does (pose_router.rs): its turns 45 degrees, each followed by
           pair_turn_steps straight steps before the next; pair_via_steps straight steps with one heading on
           each side of a via (the joins to its tips excepted)
  ends     a pair's END CONNECTORS: each pose a grid point on a router heading where its body starts or ends, each
           leg ending on the pose's own leg (half its pitch across the heading), turning 45 degrees at most, the two
           a track and the clearance apart (a pair with them has no copper join: the join and stub rules skip it)
  cross    an opposite-hands pair's CROSSOVER: its two poses grid points on a router heading, the body straight
           through them and its centre; each leg from its entry point to its exit point, on the one layer then the
           other, changing layer at its own barrel, every piece on a router direction; the entry and exit points on
           their poses' two legs, P and N on opposite sides and SWAPPED between entry and exit"""
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
    has_ends = bool(L.get('ends'))
    for i, p in enumerate(pcs):
        first, last = i == 0, i == len(pcs) - 1
        if ln(p) < EPS:
            bad['zero'].append((n, p[:2]))
            continue
        a = math.degrees(ang(p)) % 45
        if min(a, 45 - a) > 1e-3 and not (first or last):
            bad['angle'].append((n, p[:2], round(math.degrees(ang(p)), 1)))
        if (first or last) and not has_ends and ln(p) > 2 * g * math.sqrt(2) + EPS and min(a, 45 - a) > 1e-3:
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
    if td and not has_ends:
        for fwd, e in ((True, td[0]), (False, [-td[1][0], -td[1][1]])):
            d = chord_dir(pcs, fwd, TW)
            dd = math.hypot(*d)
            if not fwd:
                d = (-d[0], -d[1])
            if dd > EPS and (d[0] * e[0] + d[1] * e[1]) / dd < -1e-6:     # more than 90 degrees off: a fold
                bad['stub'].append((n, 'tooth' if fwd else 'berth',
                                    round(math.degrees(math.acos(max(-1, min(1, (d[0] * e[0] + d[1] * e[1]) / dd))))), 'deg off'))
PAIRS = set(geo.get('pairs', []))
RT, ST = RU.get('pair_turn_steps', 0), RU.get('pair_via_steps', 0)
for n in [n for n in geo['lanes'] if n in PAIRS]:
    pcs = [p for p in geo['lanes'][n]['pieces'][1:-1] if ln(p) > EPS]        # the joins to the tips excepted
    runs = []                                  # [heading, length in steps, [via positions in steps from the run's start]]
    for i, p in enumerate(pcs):
        u = (round((p[2] - p[0]) / ln(p), 6), round((p[3] - p[1]) / ln(p), 6))
        steps = ln(p) / (g / max(abs(u[0]), abs(u[1])))
        if runs and runs[-1][0] == u:
            if pcs[i - 1][4] != p[4]:
                runs[-1][2].append(runs[-1][1])
            runs[-1][1] += steps
        else:
            if runs and pcs[i - 1][4] != p[4]:
                bad['pair'].append((n, 'turns at a via', p[:2]))
            runs.append([u, steps, []])
    for k_, (u, L, vs) in enumerate(runs):
        if 0 < k_ < len(runs) - 1 and L < RT - 1e-6:
            bad['pair'].append((n, f'{L:.1f} straight steps after a turn (need {RT})', ))
        if k_:
            a0 = runs[k_ - 1][0]
            if a0[0] * u[0] + a0[1] * u[1] < math.cos(math.radians(45)) - 1e-6:
                bad['pair'].append((n, 'turns more than 45 degrees'))
        for s_ in vs:
            if (k_ and s_ < ST - 1e-6) or (k_ < len(runs) - 1 and L - s_ < ST - 1e-6):
                bad['pair'].append((n, f'via {s_:.1f} / {L - s_:.1f} straight steps either side (need {ST})'))
# a pair's END CONNECTORS: what the snap promises of each (the pair step lays them as drawn)
def _ps(p_, a_, b_):
    dx, dy = b_[0] - a_[0], b_[1] - a_[1]
    l2 = dx * dx + dy * dy
    t = 0 if l2 < 1e-18 else max(0, min(1, ((p_[0] - a_[0]) * dx + (p_[1] - a_[1]) * dy) / l2))
    return math.hypot(p_[0] - a_[0] - t * dx, p_[1] - a_[1] - t * dy)


for n in [n for n in geo['lanes'] if geo['lanes'][n].get('ends')]:
    pcs = geo['lanes'][n]['pieces']
    for k_, e_ in enumerate(geo['lanes'][n]['ends']):
        (px, py), (hx, hy) = e_['pose'], e_['heading']
        if not (on(px) and on(py)):
            bad['ends'].append((n, k_, 'pose off the grid', (round(px, 4), round(py, 4))))
        if min(abs(math.degrees(math.atan2(hy, hx)) % 45), 45 - abs(math.degrees(math.atan2(hy, hx)) % 45)) > 1e-3:
            bad['ends'].append((n, k_, 'heading off the router directions'))
        body_end = (pcs[0][2], pcs[0][3]) if k_ == 0 else (pcs[-1][0], pcs[-1][1])
        if math.hypot(body_end[0] - px, body_end[1] - py) > 1e-6:
            bad['ends'].append((n, k_, 'the body does not start at its pose'))
        legs = e_['legs']
        offs = [((q[-1][0] - px) * -hy + (q[-1][1] - py) * hx) for q in legs]      # across the heading
        if abs(abs(offs[0]) - abs(offs[1])) > 1e-6 or offs[0] * offs[1] > 0:
            bad['ends'].append((n, k_, 'legs not on the pose\'s two legs'))
        for q in legs:
            for a_, b_, c_ in zip(q, q[1:], q[2:]):
                h1 = math.atan2(b_[1] - a_[1], b_[0] - a_[0]); h2 = math.atan2(c_[1] - b_[1], c_[0] - b_[0])
                t = abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi))
                if t > 45 + 1e-3 and math.hypot(c_[0] - b_[0], c_[1] - b_[1]) > EPS and math.hypot(b_[0] - a_[0], b_[1] - a_[1]) > EPS:
                    bad['ends'].append((n, k_, f'a leg turns {t:.0f} degrees'))
        P_, N_ = legs                               # two lines apart: nearest at a vertex of one or the other
        dmin = min([_ps(p_, a_, b_) for p_ in P_ for a_, b_ in zip(N_, N_[1:])]
                   + [_ps(p_, a_, b_) for p_ in N_ for a_, b_ in zip(P_, P_[1:])])
        if dmin < TW + RU['clear'] - 1e-6:
            bad['ends'].append((n, k_, f'legs {dmin:.3f} apart (need {TW + RU["clear"]:.3f})'))
# an OPPOSITE-HANDS pair (whole_snap records them) must swap its legs: laid without a crossover it arrives crossed
for n in geo.get('opposite', []):
    if n in geo['lanes'] and not geo['lanes'][n].get('cross'):
        bad['cross'].append((n, 'an opposite-hands pair laid without its crossover'))
# a crossed pair's CROSSOVER: what the snap promises of it (the pair step lays it as drawn)
octi_ok = lambda dx, dy: min(math.degrees(math.atan2(dy, dx)) % 45, 45 - math.degrees(math.atan2(dy, dx)) % 45) < 1e-3
for n in [n for n in geo['lanes'] if geo['lanes'][n].get('cross')]:
    xo, pcs = geo['lanes'][n]['cross'], geo['lanes'][n]['pieces']
    hx, hy = xo['heading']
    hl = math.hypot(hx, hy)
    ux, uy = hx / hl, hy / hl
    if not octi_ok(hx, hy):
        bad['cross'].append((n, 'heading off the router directions'))
    for q in xo['poses']:
        if not (on(q[0]) and on(q[1])):
            bad['cross'].append((n, 'pose off the grid', (round(q[0], 4), round(q[1], 4))))
    for q in list(xo['poses']) + [xo['at']]:
        if not any(_ps(q, (p[0], p[1]), (p[2], p[3])) < 1e-6 for p in pcs):
            bad['cross'].append((n, 'the body does not pass its pose or centre', (round(q[0], 3), round(q[1], 3))))
    for k, runs in xo['legs'].items():
        (pts0, _La), (pts1, _Lb) = runs[0], runs[-1]
        barrel = [(x, y) for x, y, kk in xo['vias'] if kk == k]
        if math.hypot(pts0[0][0] - xo['entry'][k][0], pts0[0][1] - xo['entry'][k][1]) > 1e-6 \
                or math.hypot(pts1[-1][0] - xo['exit'][k][0], pts1[-1][1] - xo['exit'][k][1]) > 1e-6:
            bad['cross'].append((n, k, 'leg not from its entry point to its exit point'))
        if [L_ for _q, L_ in runs] != list(xo['layers']):
            bad['cross'].append((n, k, 'leg layers', [L_ for _q, L_ in runs]))
        if len(barrel) != 1 or math.hypot(pts0[-1][0] - barrel[0][0], pts0[-1][1] - barrel[0][1]) > 1e-6 \
                or math.hypot(pts1[0][0] - barrel[0][0], pts1[0][1] - barrel[0][1]) > 1e-6:
            bad['cross'].append((n, k, 'leg does not change layer at its own barrel'))
        for pts, _L in runs:
            for a_, b_ in zip(pts, pts[1:]):
                if math.hypot(b_[0] - a_[0], b_[1] - a_[1]) > EPS and not octi_ok(b_[0] - a_[0], b_[1] - a_[1]):
                    bad['cross'].append((n, k, 'a leg piece off the router directions', (round(a_[0], 3), round(a_[1], 3))))
    side = {}
    for which, pose in (('entry', xo['poses'][0]), ('exit', xo['poses'][1])):
        offs = {k: (pt[0] - pose[0]) * -uy + (pt[1] - pose[1]) * ux for k, pt in xo[which].items()}
        if abs(abs(offs['P']) - abs(offs['N'])) > 1e-6 or offs['P'] * offs['N'] > 0:
            bad['cross'].append((n, f'{which} points not on the pose\'s two legs'))
        side[which] = offs['P'] > 0
    if side.get('entry') == side.get('exit'):
        bad['cross'].append((n, 'the legs do not swap sides'))
for k, v in bad.items():
    for row in v[:12]:
        print('LINT', k, *row)
    if len(v) > 12:
        print('LINT', k, f'... {len(v) - 12} more')
print('LINT ' + (', '.join(f'{k} {len(v)}' for k, v in sorted(bad.items())) or 'clean'))
