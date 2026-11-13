"""whole_feedback.py SIDECAR.plan.json OUT.json HOT.json [HOT.json ...] -- the ENDS the whole route's audits found
crowded, for the fanout to choose again (fanout_from_plan: FEEDBACK=OUT.json; whole_ends prices them).
whole_feedback.py --repeat SIDECAR.plan.json HOT_BEFORE.json HOT_NOW.json -- exit 0 (and name them) when a finding at
the ends -- its kind, its lanes, its end -- stands in both rounds' audits: the solve had its round and did not move it,
and only the fanout can (whole_loop stops there) -- but not while the round has a finding of the solve's own that the
round before had not: the solve answers that first.
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
whole_feedback.py --edits SIDECAR.plan.json OUT.json EDITS.json -- the ends a round's EDITS name (whole_route's ROUND
EDITS: its `avoid` and `ban`, each {"lane", "end"} -- 0 the tooth, 1 the berth --, its `pairs`, two such each, and its
`pin`, each {"lane", "end", "at", "layer"}) added to OUT as ends to avoid (priced), ends banned (whole_ends: none of the
lane's options there), pairs of ends not to be chosen together, each end's points and layer read off the sidecar as
laid, and ends pinned (the lane's option nearest `at`, in the sidecar's frame, on `layer` where one is).
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
import ends_near
import pairs as _pairs


REPEAT = sys.argv[1] == '--repeat'
NOW = sys.argv[1] == '--now'
REFUSED = sys.argv[1] == '--refused'
NAME = sys.argv[1] == '--name'
RAISE = sys.argv[1] == '--raise'
EDITS = sys.argv[1] == '--edits'
if REPEAT or NOW or REFUSED or NAME or RAISE or EDITS:
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


# (at the ends: ends_near's one test, the ends' repair's too -- whole_route.repair_ends)
# (a run to several parts, one source and several ends -- the sidecar's dest_of, fanout_from_plan: the berths' box of
# each part, which the ends near one of them are at; one box round them all would span the gap between the parts)
BOXES = ends_near.boxes(ends, S.get('dest_of') or {})
FRONT = 2 * bd.VIA_NEED + bd.LANE_MIN           # in front of a face: its band (whole_solve FACE_ROOM) and a lane pitch


def near(k, x, y):
    """is (x, y) at array k's ends: beside a side face within its stack, else in front within the band (of the box
    of its ends nearest it)"""
    return ends_near.near(ends, BOXES, k, x, y, FRONT, bd.LANE_MIN)


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
    # ...and NOT while the round has a finding of the SOLVE's that the round before had not (a dive anywhere, anything
    # out between the arrays): the solve is still answering what its rounds turn up, and the finding at the ends
    # stands beside that, not in place of it. Stopped there, a loop whose first round's three findings were down to a
    # berth 0.07 mm from a capacitor's pad and one new dive beside another bus's lane handed on a plan the snap laid a
    # lane short at that dive -- the dive one more round's via cut away, the berth laid round the pad with no violation
    def solves(fn):
        """the findings of a hot file that are the solve's to move, {(kind, lanes)}: all but a pitch or a static
        clearance at the ends"""
        got = set()
        for x, y, kind, *rest in json.load(open(fn)).get('hot', []):
            x, y = board_xy(x, y)
            lanes = tuple(sorted(lanes_named(rest[0] if rest else [])))
            if not (kind in ('PITCH', 'STATIC') and any(near(k, x, y) for k in (0, 1))):
                got.add((kind, lanes))
        return got
    fresh = sorted(solves(hots[0]) - solves(out)) if both else []
    if fresh:
        print(f'  {len(both)} finding(s) at the ends in both rounds, and {len(fresh)} of the solve\'s this round the '
              f'round before had not -- the solve\'s to answer first: '
              + ', '.join(f'{kind} {"/".join(lanes)}' for kind, lanes in fresh))
        sys.exit(1)
    for kind, lanes, k in sorted(both):
        print(f'  {kind} {"/".join(lanes)} at the {"teeth" if k == 0 else "berths"}, both rounds')
    sys.exit(0 if both else 1)

fb = json.load(open(out)) if os.path.exists(out) else {'pairs': [], 'avoid': []}
ROUND = awx_settings.get('FB_ROUND')                 # the whole route's round naming ends now
import run_trace                                     # noqa: E402
TRF = run_trace.Trace(out[:-len('.json')] + '.trace.jsonl' if out.endswith('.json') else out + '.trace.jsonl',
                      fresh=False)                   # (each call adds to OUT; its trace too)


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


_KEYS = []                  # (the board beside the sidecar's nets by the whole route's names, read once: disconnected)


def disconnected(conn_log):
    """{lane: [(x, y)]}: the balls check_connected found cut off from their net's copper"""
    import re
    from coherent_nets import key_of
    got, cur = {}, None
    for ln in (open(conn_log, errors='replace') if conn_log and os.path.isfile(conn_log) else ()):
        m = re.match(r'^\s*(.+?) \(net \d+\):\s*$', ln)
        if m:
            if not _KEYS:
                import contextlib
                import io
                from kicad_parser import parse_kicad_pcb
                from coherent_nets import net_keys
                with contextlib.redirect_stdout(io.StringIO()):
                    _KEYS.append(net_keys(parse_kicad_pcb(sidecar[:-len('.plan.json')] + '.kicad_pcb')))
            n = key_of(m.group(1), _KEYS[0])
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
            ks.add(min((0, 1), key=lambda k: min(ends_near.gap(b_, x, y) for b_ in BOXES[k])))
    return sorted(ks) or [0, 1]


if EDITS:
    ed = json.load(open(hots[0]))
    added = 0
    for it_ in ed.get('avoid') or ():
        e_ = end_of(LANE.get(it_['lane'], it_['lane']), int(it_['end']))
        if e_ is None:
            print(f'  whole_feedback: no lane {it_["lane"]} on the sidecar -- an end to avoid not added')
            continue
        added += put_avoid(dict(e_, times=int(it_['times'])) if it_.get('times') else e_)
    # (an end BANNED: none of the lane's options at that end where it stands now, its laid end not held -- whole_ends)
    fb.setdefault('ban', [])
    seen_b = {json.dumps(x, sort_keys=True) for x in fb['ban']}
    for it_ in ed.get('ban') or ():
        e_ = end_of(LANE.get(it_['lane'], it_['lane']), int(it_['end']))
        if e_ is None:
            print(f'  whole_feedback: no lane {it_["lane"]} on the sidecar -- an end to ban not added')
            continue
        # (the place on every layer -- a tooth moved to another layer at the same exit is the same tooth -- or on the
        # layer it names: 'laid' its laid one)
        e_ = dict(e_, layer=e_['layer'] if it_.get('layer') == 'laid' else it_.get('layer'))
        if json.dumps(e_, sort_keys=True) not in seen_b:
            seen_b.add(json.dumps(e_, sort_keys=True))
            fb['ban'].append(e_)
            added += 1
    # (an end PINNED: the lane's option nearest the point asked, in this sidecar's frame -- whole_route turned the
    # agent's board point into it -- on the layer asked where one is; a later pin of the same end replaces it)
    fb.setdefault('pin', [])
    for it_ in ed.get('pin') or ():
        ln_ = LANE.get(it_['lane'], it_['lane'])
        if not legs_of(ln_):
            print(f'  whole_feedback: no lane {it_["lane"]} on the sidecar -- a pin not added')
            continue
        p_ = {'lane': ln_, 'end': int(it_['end']), 'points': [[float(v_) for v_ in it_['at']]],
              'layer': it_.get('layer')}
        fb['pin'] = [x for x in fb['pin'] if (x['lane'], x['end']) != (ln_, p_['end'])] + [p_]
        added += 1
    seen = {json.dumps(x, sort_keys=True) for x in fb['pairs']}
    for pr_ in ed.get('pairs') or ():
        es_ = [end_of(LANE.get(x_['lane'], x_['lane']), int(x_['end'])) for x_ in pr_]
        if len(es_) != 2 or None in es_:
            print(f'  whole_feedback: a pair of ends not both on the sidecar -- not added: {pr_}')
            continue
        it = sorted(es_, key=lambda e: e['lane'])
        key = json.dumps(it, sort_keys=True)
        if key not in seen:
            seen.add(key)
            fb['pairs'].append(it)
            added += 1
    json.dump(fb, open(out, 'w'), indent=1)
    print(f'whole_feedback: {added} new from the round\'s edits, {len(fb["pairs"])} pairs, {len(fb["avoid"])} ends to '
          f'avoid, {len(fb["ban"])} banned and {len(fb["pin"])} pinned in {out}')
    sys.exit(0)
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
        fe_ = {ln: failing_ends(ln) for ln in named}
        added = sum(put_avoid(e) for ln in named for e in (end_of(ln, k) for k in fe_[ln]) if e)
        TRF.rec('name', round=ROUND, why=why, lanes={ln: [('tooth', 'berth')[k] for k in ks] for ln, ks in fe_.items()})
    else:
        tiers = [('over two vias', sorted((ln for ln in ov if ov[ln] > 0), key=lambda ln: (-ov[ln], -ld.get(ln, 0), ln))),
                 ('loaded on the trunk', sorted((ln for ln in ld if ld[ln] > LOAD_OK), key=lambda ln: (-ld[ln], ln))),
                 ('most crossed', [ln for ln in sorted(xs, key=lambda ln: (-xs[ln], ln)) if fresh(ln)][:NAME_TOP])]
        for why, lanes in tiers:
            named = [ln for ln in lanes if fresh(ln)]
            if named:
                break
        added = sum(put_avoid(e) for ln in named for e in fresh(ln))
        TRF.rec('name', round=ROUND, why=why, lanes={ln: ['tooth', 'berth'] for ln in named})
    fb['named'] = list(dict.fromkeys(list(fb.get('named') or []) + named))
    json.dump(fb, open(out, 'w'), indent=1)
    print(f'whole_feedback: {added} new, the lanes named: ' +
          (f'{", ".join(named)} ({why})' if named else 'no lane left to name') +
          f', {len(fb["pairs"])} pairs and {len(fb["avoid"])} ends to avoid in {out}')
    sys.exit(0)
seen = {json.dumps(x, sort_keys=True) for x in fb['pairs']}
added = 0
DIRK = {0: dict(S.get('tooth_dir') or {}), 1: dict(S.get('stub_dir') or {})}   # each net's end's escape direction:
#                                                                                 its FACE (none given: every end one
#                                                                                 face, near_own then near's own test)
_FACE_BOXES = {}


def near_own(lane, k, x, y):
    """is (x, y) at lane's OWN end k: at the ends of its face -- the ends k leaving the array the way its own does --
    as `near` tests the array's (ends_near). Near the array's ends alone, a lane passing in front of one face on its
    way to a berth on another was named at that berth, 12 mm from the finding"""
    lg = legs_of(lane)
    f = tuple(DIRK[k].get(lg[0]) or ()) if lg else ()
    if (k, f) not in _FACE_BOXES:
        sub = {n: e for n, e in ends.items() if tuple(DIRK[k].get(n) or ()) == f}
        _FACE_BOXES[(k, f)] = (sub, ends_near.boxes(sub, S.get('dest_of') or {}) if sub else None)
    sub, bxs = _FACE_BOXES[(k, f)]
    return bool(sub) and ends_near.near(sub, bxs, k, x, y, FRONT, bd.LANE_MIN)


for fn in hots:
    for x, y, kind, *rest in json.load(open(fn)).get('hot', []):
        x, y = board_xy(x, y)
        lanes = lanes_named(rest[0] if rest else [])
        items = []
        for k in (0, 1):
            if not near(k, x, y):
                continue
            # (the lanes it names at their OWN end there -- near_own; a lane only passing that face is not named there)
            es = [e for e in (end_of(ln, k) for ln in lanes if near_own(ln, k, x, y)) if e]
            items += ([('pairs', sorted([a, b], key=lambda e: e['lane'])) for a, b in itertools.combinations(es, 2)]
                      if len(es) >= 2 else [('avoid', es[0])] if es else [])
        acts = []
        for kind_, it in items:
            if kind_ == 'avoid':
                a1_ = put_avoid(it)
                added += a1_
                acts.append('added or raised' if a1_ else 'named already this round')
                continue
            key = json.dumps(it, sort_keys=True)
            if key not in seen:
                seen.add(key)
                fb[kind_].append(it)
                added += 1
                acts.append('added')
            else:
                acts.append('named already')
        # (THE CREDIT, a trace's: the finding, which rule named which ends, how far each end is from it, and what was
        # done -- OUT.trace.jsonl beside OUT)
        TRF.rec('finding', src=os.path.basename(fn), round=ROUND, kind=kind, lanes=lanes, xy=[x, y],
                near=[k for k in (0, 1) if near(k, x, y)],
                credited=[{'as': kind_, 'act': act_, 'ends': [
                    {'lane': e['lane'], 'end': ('tooth', 'berth')[e['end']], 'layer': e['layer'],
                     'dist_mm': round(min(math.hypot(p[0] - x, p[1] - y) for p in e['points']), 2)}
                    for e in (it if kind_ == 'pairs' else [it])]} for (kind_, it), act_ in zip(items, acts)])
json.dump(fb, open(out, 'w'), indent=1)
print(f'whole_feedback: {added} new, {len(fb["pairs"])} pairs and {len(fb["avoid"])} ends to avoid in {out}')
