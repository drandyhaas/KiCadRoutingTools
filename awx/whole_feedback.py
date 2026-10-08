"""whole_feedback.py SIDECAR.plan.json OUT.json HOT.json [HOT.json ...] -- the ENDS the whole route's audits found
crowded, for the fanout to choose again (fanout_from_plan: FEEDBACK=OUT.json; whole_ends prices them).
whole_feedback.py --repeat SIDECAR.plan.json HOT_BEFORE.json HOT_NOW.json -- exit 0 (and name them) when a finding at
the ends -- its kind, its lanes, its end -- stands in both rounds' audits: the solve had its round and did not move it,
and only the fanout can (whole_loop stops there).
whole_feedback.py --refused SIDECAR.plan.json OUT.json REFUSED.json -- the FANOUT AUDIT's findings on a board the fanout
refused (fanout_from_plan: a pair split by other lanes' ends between its tips): each split pair's end and each such
lane's end there, a PAIR of ends not to be chosen together again; a tooth on the source's far face, an end to AVOID --
the next round, incremental, frees them.
whole_feedback.py --name SIDECAR.plan.json OUT.json [NET ...] -- a round the whole route left nets OPEN on, or laid
NOTHING on (no plan from its solve, none its loop held), with nothing new from its audits: lanes to free, their ends
to AVOID -- the lanes of the NETs given (the round's open nets), each at the end where its trouble is: where the
round's audit findings name it (FB_HOTS, the round's hot files), else the array nearer a ball of it cut off from its
copper (FB_CONN, the round's check_connected log), else both; with no NET, the lanes the ends model names on those
ends (the sidecar's 'ends_model', whole_ends), both their ends: its nets over two vias, else its lanes loaded past
LOAD_OK on the trunk, else its NAME_TOP most crossed -- the first of these naming a lane not named before. The lanes
are kept in OUT's 'named', in the order named: the lanes whole_route's last resort leaves out of a partial plan.
whole_feedback.py --raise SIDECAR.plan.json OUT.json [--widen] -- a round whose fanout laid the round before's ends
again (priced its feedback and did not follow it): every end to avoid last named in that round (FB_ROUND) raised once
more; with --widen, both ends of every lane named (--name) to avoid as well -- an open lane named where it fails, whose
failing end has no other place to go (a via-in-pad berth), freed and priced at its other end too. whole_route then
runs that round's fanout again, not its route.
whole_feedback.py --now SIDECAR.plan.json HOT.json -- exit 0 (and name them) when a finding at the ends is one no solve
moves: a pitch or a static clearance there comes of where the ends stand (a dive there is the solve's: its via cut
moves it; a fold, the crossings the solve put round it) -- whole_loop stops at once, before paying a solve.

A finding (whole_gate --hot: its place, its kind, the lanes it names) is AT an array's ends -- the teeth or the berths --
where their own copper is: beside a side face, within the width that face's lanes stack to (its ends' count, a lane
pitch each, and two more); in front of a face, within its face band (a change's room, as whole_solve keeps clear) and a
lane pitch. One out between the arrays is the solve's, not the fanout's (K35: a fold a lane's re-solved crossings made
1.9 mm out from the source's face was read as its tooth's, and the fanout moved a tooth that was not the trouble). Two lanes it names
together are a PAIR of ends not to be chosen together again; one lane alone (a fold, a dive, a lane against static
copper) an end to AVOID. Each end is named by its lane and its legs' points and layer as laid (the sidecar's), which
whole_ends prices against its options, by place (whole_ends.fb_weights). OUT.json is merged into when it exists: the
feedback of every round stands, and an end to avoid named again in a LATER round (FB_ROUND, the whole route's round;
without one, by an earlier call) is not added twice -- its 'times' rises, and whole_ends doubles its price each time.

The findings are in the PLAN's frame -- a board of pair chirality -1 turned over (braid.setup) -- and the sidecar's
ends in the board's: a finding is turned back before it is placed (y -> 2 CY - y about the board's mirror axis, read
from the board beside the sidecar). A lane is named as whole_ends names it: a pair by its base (pairs.pair_names),
never by a leg."""
import itertools
import json
import math
import os
import awx_settings
import sys

import braid as bd
import pairs as _pairs


REPEAT = sys.argv[1] == '--repeat'
NOW = sys.argv[1] == '--now'
REFUSED = sys.argv[1] == '--refused'
NAME = sys.argv[1] == '--name'
RAISE = sys.argv[1] == '--raise'
if REPEAT or NOW or REFUSED or NAME or RAISE:
    sys.argv.pop(1)
NAME_TOP = 3                                         # --name's last tier: the lanes most crossed, this many a round
if NOW:
    sys.argv.insert(2, '')                            # (no earlier round)
sidecar, out, hots = sys.argv[1], sys.argv[2], sys.argv[3:]
S = json.load(open(sidecar))
ends = dict(S['ends'])
lay = {0: dict(S['tooth_layer']), 1: dict(S['dest_layer'])}
# the lanes as whole_ends makes them: a pair one lane, under the same switch
PAIRS = (_pairs.pair_names(list(ends)) if int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0)
         else {})
LANE = {leg: base for base, pr in PAIRS.items() for leg in pr}
CY = None
if int(S.get('chi', 1)) < 0:
    from kicad_parser import parse_kicad_pcb
    from bga_fanout.flip_frame import mirror_axis
    CY = mirror_axis(parse_kicad_pcb(sidecar[:-len('.plan.json')] + '.kicad_pcb'))


def board_xy(x, y):
    """a finding's point (the plan's frame) in the board's"""
    return (x, 2.0 * CY - y) if CY is not None else (x, y)


def legs_of(lane):
    """the sidecar's nets of a lane: itself, or a pair's two legs"""
    if lane in PAIRS:
        return [n for n in PAIRS[lane] if n in ends]
    return [lane] if lane in ends else []


def lanes_named(names):
    """the lanes a finding names (a leg read as its pair), in order, each once"""
    return list(dict.fromkeys(ln for ln in (LANE.get(n, n) for n in names) if legs_of(ln)))


def box(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


boxes = {0: box([e[0] for e in ends.values()]), 1: box([e[1] for e in ends.values()])}
FRONT = 2 * bd.VIA_NEED + bd.LANE_MIN           # in front of a face: its band (whole_solve FACE_ROOM) and a lane pitch


def side_reach(k, face):
    """beside a side face ('N' or 'S') of array k's box: the width its ends' lanes stack to, and two pitches"""
    b = boxes[k]
    y0 = b[1] if face == 'N' else b[3]
    n = sum(1 for e in ends.values() if abs(e[k][1] - y0) < 1e-3)
    return (n + 2) * bd.LANE_MIN


def near(k, x, y):
    """is (x, y) at array k's ends: beside a side face within its stack, else in front within the band"""
    b = boxes[k]
    dx = max(b[0] - x, 0.0, x - b[2])
    dy = max(b[1] - y, 0.0, y - b[3])
    if dx == 0.0 and dy > 0.0:
        return dy <= side_reach(k, 'N' if y < b[1] else 'S')
    return math.sqrt(dx * dx + dy * dy) <= FRONT


def end_of(lane, k):
    lg = legs_of(lane)
    if not lg:
        return None
    return {'lane': lane, 'end': k, 'points': [ends[n][k] for n in lg], 'layer': lay[k].get(lg[0])}


def at_ends(fn):
    """the findings of a hot file at the ends: {(kind, lanes, end)}"""
    got = set()
    for x, y, kind, *rest in json.load(open(fn)).get('hot', []):
        x, y = board_xy(x, y)
        lanes = tuple(sorted(lanes_named(rest[0] if rest else [])))
        for k in (0, 1):
            if lanes and near(k, x, y):
                got.add((kind, lanes, k))
    return got


if NOW:
    fanouts = sorted(f for f in at_ends(hots[0]) if f[0] in ('PITCH', 'STATIC'))
    for kind, lanes, k in fanouts:
        print(f'  {kind} {"/".join(lanes)} at the {"teeth" if k == 0 else "berths"}')
    sys.exit(0 if fanouts else 1)
if REPEAT:
    both = at_ends(out) & at_ends(hots[0])            # (here `out` is the earlier round's hot file)
    for kind, lanes, k in sorted(both):
        print(f'  {kind} {"/".join(lanes)} at the {"teeth" if k == 0 else "berths"}, both rounds')
    sys.exit(0 if both else 1)

fb = json.load(open(out)) if os.path.exists(out) else {'pairs': [], 'avoid': []}
ROUND = awx_settings.get('FB_ROUND')                 # the whole route's round naming ends now


def _akey(it):
    """an end to avoid by what it names (its lane, end, points and layer), not by how often or when"""
    return json.dumps({k: v for k, v in it.items() if k not in ('times', 'round')}, sort_keys=True)


_avoid = {_akey(x): x for x in fb['avoid']}
_raised = set()


def put_avoid(it):
    """an end to AVOID: added; or, named in an EARLIER round (without ROUND, by an earlier call), its 'times' raised
    once -- 1 when either, else 0"""
    k = _akey(it)
    x = _avoid.get(k)
    if x is None:
        x = _avoid[k] = dict(it, round=ROUND)
        fb['avoid'].append(x)
    elif k in _raised or (ROUND is not None and str(x.get('round')) == str(ROUND)):
        return 0
    else:
        x['times'] = int(x.get('times', 1)) + 1
        x['round'] = ROUND
    _raised.add(k)
    return 1


def disconnected(conn_log):
    """{lane: [(x, y)]}: the balls check_connected found cut off from their net's copper"""
    import re
    got, cur = {}, None
    for ln in (open(conn_log, errors='replace') if conn_log and os.path.isfile(conn_log) else ()):
        m = re.match(r'^\s*(.+?) \(net \d+\):\s*$', ln)
        if m:
            n = m.group(1).split('/')[-1]
            cur = LANE.get(n, n)
            continue
        m = re.match(r'^\s*\(([-\d.]+), ([-\d.]+)\) on \S+ \[\S+\]', ln)
        if m and cur:
            got.setdefault(cur, []).append((float(m.group(1)), float(m.group(2))))
    return got


def failing_ends(ln):
    """the ends of open lane `ln` where its trouble is: those the round's audit findings name it at (FB_HOTS), else
    the array nearer a ball of it cut off from its copper (FB_CONN), else both"""
    ks = set()
    for fn in [f for f in (awx_settings.get('FB_HOTS') or '').split(',') if f and os.path.isfile(f)]:
        for x, y, _kind, *rest in json.load(open(fn)).get('hot', []):
            if ln in lanes_named(rest[0] if rest else []):
                x, y = board_xy(x, y)
                ks |= {k for k in (0, 1) if near(k, x, y)}
    if not ks:
        for x, y in disconnected(awx_settings.get('FB_CONN')).get(ln, ()):
            ks.add(min((0, 1), key=lambda k: math.hypot(max(boxes[k][0] - x, 0.0, x - boxes[k][2]),
                                                        max(boxes[k][1] - y, 0.0, y - boxes[k][3]))))
    return sorted(ks) or [0, 1]


if RAISE:
    raised = added = 0
    for x in fb['avoid']:
        if ROUND is not None and str(x.get('round')) == str(ROUND):
            x['times'] = int(x.get('times', 1)) + 1
            raised += 1
    if '--widen' in hots:
        for ln in fb.get('named') or []:
            for k in (0, 1):
                e = end_of(ln, k)
                if e and _akey(e) not in _avoid:
                    _avoid[_akey(e)] = x = dict(e, round=ROUND)
                    fb['avoid'].append(x)
                    added += 1
    json.dump(fb, open(out, 'w'), indent=1)
    print(f'whole_feedback: {raised + added} raised: {raised} ends named in round {ROUND} once more, {added} other '
          f'ends of the lanes named added, {len(fb["pairs"])} pairs and {len(fb["avoid"])} ends to avoid in {out}')
    sys.exit(0)
if REFUSED:
    seen = {json.dumps(x, sort_keys=True) for x in fb['pairs']}
    added = 0
    for pr_, end_, between_ in json.load(open(hots[0])).get('splits', []):
        k = 0 if end_ == 'tooth' else 1
        a = end_of(LANE.get(pr_, pr_), k)
        for ln in between_:
            b = end_of(LANE.get(ln, ln), k)
            if a and b:
                it = sorted([a, b], key=lambda e: e['lane'])
                key = json.dumps(it, sort_keys=True)
                if key not in seen:
                    seen.add(key)
                    fb['pairs'].append(it)
                    added += 1
    for ln in json.load(open(hots[0])).get('far', []):
        a = end_of(LANE.get(ln, ln), 0)
        if a:
            added += put_avoid(a)
    json.dump(fb, open(out, 'w'), indent=1)
    print(f'whole_feedback: {added} new from the fanout audit (split pairs, far-face teeth), {len(fb["pairs"])} pairs '
          f'and {len(fb["avoid"])} ends to avoid in {out}')
    sys.exit(0)
if NAME:
    from whole_ends import LOAD_OK
    em = S.get('ends_model') or {}
    ov, ld, xs = em.get('over') or {}, em.get('load') or {}, em.get('x') or {}
    def fresh(ln):
        """the ends of lane `ln` not yet to avoid"""
        return [e for e in (end_of(ln, 0), end_of(ln, 1)) if e and _akey(e) not in _avoid]
    named, why = lanes_named(hots), 'open'
    if named:
        # the open lanes, each at the end where its trouble is -- an end named in an earlier round raised
        added = sum(put_avoid(e) for ln in named for e in (end_of(ln, k) for k in failing_ends(ln)) if e)
    else:
        tiers = [('over two vias', sorted((ln for ln in ov if ov[ln] > 0), key=lambda ln: (-ov[ln], -ld.get(ln, 0), ln))),
                 ('loaded on the trunk', sorted((ln for ln in ld if ld[ln] > LOAD_OK), key=lambda ln: (-ld[ln], ln))),
                 ('most crossed', [ln for ln in sorted(xs, key=lambda ln: (-xs[ln], ln)) if fresh(ln)][:NAME_TOP])]
        for why, lanes in tiers:
            named = [ln for ln in lanes if fresh(ln)]
            if named:
                break
        added = sum(put_avoid(e) for ln in named for e in fresh(ln))
    fb['named'] = list(dict.fromkeys(list(fb.get('named') or []) + named))
    json.dump(fb, open(out, 'w'), indent=1)
    print(f'whole_feedback: {added} new, the lanes named: ' +
          (f'{", ".join(named)} ({why})' if named else 'no lane left to name') +
          f', {len(fb["pairs"])} pairs and {len(fb["avoid"])} ends to avoid in {out}')
    sys.exit(0)
seen = {json.dumps(x, sort_keys=True) for x in fb['pairs']}
added = 0
for fn in hots:
    for x, y, kind, *rest in json.load(open(fn)).get('hot', []):
        x, y = board_xy(x, y)
        lanes = lanes_named(rest[0] if rest else [])
        for k in (0, 1):
            if not near(k, x, y):
                continue
            es = [e for e in (end_of(ln, k) for ln in lanes) if e]
            items = ([('pairs', sorted([a, b], key=lambda e: e['lane'])) for a, b in itertools.combinations(es, 2)]
                     if len(es) >= 2 else [('avoid', es[0])] if es else [])
            for kind_, it in items:
                if kind_ == 'avoid':
                    added += put_avoid(it)
                    continue
                key = json.dumps(it, sort_keys=True)
                if key not in seen:
                    seen.add(key)
                    fb[kind_].append(it)
                    added += 1
json.dump(fb, open(out, 'w'), indent=1)
print(f'whole_feedback: {added} new, {len(fb["pairs"])} pairs and {len(fb["avoid"])} ends to avoid in {out}')
