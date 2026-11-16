#!/usr/bin/env python3
"""#1939: the restore collision test finds its pairs with numpy and returns
what the pair-by-pair loop returned.

`rip_up_reroute._saved_route_colliders` decides whether a ripped net's saved
copper can be put back. It tested every restored item against every foreign
item in the saved route's box in a Python double loop, which was a tenth of
the wall time of a rip-up-heavy route step. It now takes its pairs from
`_restore_pair_candidates`, which measures them in numpy and hands on only
those within their threshold plus a band; each of those is still decided by
the scalar test, unchanged.

`reference_colliders` below is the pair-by-pair loop as it was before the
change, kept here as the thing to compare against. Checks:

  1. Edge cases, each by name: a segment touching, crossing, parallel at the
     exact threshold and one step either side of it, collinear, zero-length,
     on another layer, of the own net, of net 0, on the edge of the saved
     route's box; a via at the threshold; two vias at the hole-to-hole floor.
  2. Randomised boards (flat clearance; net classes with a layer rule; input
     copper marked, some of it moved; hole-to-hole on and off), `first_only`
     on and off: the same culprits, the same objects, in the same order.
  3. Each case is run with the array path forced on and with it forced off.
  4. Non-vacuity: the array path ran, it left most pairs out, the cases hold
     both verdicts, and a broad phase that drops a colliding pair is caught.

    python3 tests/test_1939_restore_colliders_vectorised.py
"""
import os
import random
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from routing_config import GridRouteConfig                  # noqa: E402
from synth import make_seg, make_via, make_pcb              # noqa: E402
import rip_up_reroute as rr                                 # noqa: E402

OWN, OWN2 = 1, 2
LAYERS = ['F.Cu', 'In1.Cu', 'B.Cu']
failures = []
stats = {'cases': 0, 'collide': 0, 'clear': 0, 'array_calls': 0,
         'candidates': 0, 'pairs': 0}


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def reference_colliders(saved_result, pcb_data, own_net_ids, clearance,
                        first_only=False, config=None):
    """`_saved_route_colliders` as it stood before #1939: every restored item
    against every foreign item in the box, one pair at a time."""
    own = set(own_net_ids)
    segs = saved_result.get('new_segments', []) or []
    vias = saved_result.get('new_vias', []) or []
    if not segs and not vias:
        return []

    pairwise = config is not None and not config.pair_clearance_inert()
    _inp = getattr(pcb_data, '_input_copper', None) if pairwise else None
    inp_ids = _inp[0] if _inp else frozenset()

    def _clr(own_net, o, layer, kind):
        if not pairwise:
            return clearance
        c = config.pair_clearance(own_net, o.net_id, layer, kind=kind,
                                  base=clearance)
        if id(o) in inp_ids and c > clearance:
            return clearance
        return c

    xs, ys = [], []
    own_half = 0.0
    for s in segs:
        xs.extend((s.start_x, s.end_x))
        ys.extend((s.start_y, s.end_y))
        own_half = max(own_half, s.width / 2.0)
    for v in vias:
        xs.append(v.x)
        ys.append(v.y)
        own_half = max(own_half, v.size / 2.0)
    margin = own_half + (config.max_pair_clearance(base=clearance)
                         if pairwise else clearance)
    h2h = (getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0) if config else 0.0
    _inp_all = getattr(pcb_data, '_input_copper', None)
    inp_all = _inp_all[0] if _inp_all else frozenset()
    if h2h > 0 and vias:
        margin = max(margin, max(v.drill for v in vias) / 2.0 + h2h)
    if config is not None and getattr(config, 'hole_rules', None) is not None:
        from check_drc import _hole_rule_reach
        margin = max(margin, own_half + _hole_rule_reach(getattr(config, 'rules', None)))
    minx, maxx = min(xs) - margin, max(xs) + margin
    miny, maxy = min(ys) - margin, max(ys) + margin

    def _in_box(lo_x, hi_x, lo_y, hi_y):
        return not (hi_x < minx or lo_x > maxx or hi_y < miny or lo_y > maxy)

    o_segs = []
    for s in pcb_data.segments:
        if s.net_id in own or s.net_id == 0:
            continue
        h = s.width / 2.0
        if _in_box(min(s.start_x, s.end_x) - h, max(s.start_x, s.end_x) + h,
                   min(s.start_y, s.end_y) - h, max(s.start_y, s.end_y) + h):
            o_segs.append(s)
    o_vias = []
    for v in pcb_data.vias:
        if v.net_id in own or v.net_id == 0:
            continue
        h = v.size / 2.0
        if _in_box(v.x - h, v.x + h, v.y - h, v.y + h):
            o_vias.append(v)

    hits = []
    seen = set()
    held = rr.input_pair_unmoved(pcb_data)

    def _hit(kind, obj, mine):
        if held is not None and held(mine, obj):
            return False
        if id(obj) not in seen:
            seen.add(id(obj))
            hits.append((kind, obj))
        return True

    for s in segs:
        hw = s.width / 2.0
        for o in o_segs:
            if o.layer != s.layer:
                continue
            thr = hw + o.width / 2.0 + _clr(s.net_id, o, s.layer, 'track')
            if rr._seg_seg_dist_sq(s.start_x, s.start_y, s.end_x, s.end_y,
                                   o.start_x, o.start_y, o.end_x, o.end_y) < thr * thr:
                if _hit('segment', o, s) and first_only:
                    return hits
        for v in o_vias:
            thr = hw + v.size / 2.0 + _clr(s.net_id, v, s.layer, 'layer')
            if rr._pt_seg_dist_sq(v.x, v.y, s.start_x, s.start_y,
                                  s.end_x, s.end_y) < thr * thr:
                if _hit('via', v, s) and first_only:
                    return hits

    for vv in vias:
        vr = vv.size / 2.0
        for v in o_vias:
            thr = vr + v.size / 2.0 + _clr(vv.net_id, v, None, 'stack')
            if h2h > 0 and vv.drill > 0 and v.drill > 0 \
                    and not (id(vv) in inp_all and id(v) in inp_all):
                thr = max(thr, vv.drill / 2.0 + v.drill / 2.0 + h2h)
            if (vv.x - v.x) ** 2 + (vv.y - v.y) ** 2 < thr * thr:
                if _hit('via', v, vv) and first_only:
                    return hits
        for o in o_segs:
            thr = vr + o.width / 2.0 + _clr(vv.net_id, o, o.layer, 'layer')
            if rr._pt_seg_dist_sq(vv.x, vv.y, o.start_x, o.start_y,
                                  o.end_x, o.end_y) < thr * thr:
                if _hit('segment', o, vv) and first_only:
                    return hits

    if not hits and config is not None \
            and getattr(config, 'hole_rules', None) is not None:
        hits.extend(rr._hole_rule_colliders(segs, vias, pcb_data, config, held,
                                            o_segs, o_vias))
    return hits


def cfg(classes=None, layer_clearances=None, h2h=0.0):
    c = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                        via_drill=0.3, layers=list(LAYERS), grid_step=0.05,
                        hole_to_hole_clearance=h2h)
    if classes:
        c.set_net_clearances(dict(classes), routed_net_ids=[OWN, OWN2])
    if layer_clearances:
        c.layer_clearances = dict(layer_clearances)
    return c


def same(a, b):
    return [(k, id(o)) for k, o in a] == [(k, id(o)) for k, o in b]


def compare(label, saved, pcb, config, clearance=0.2, quiet=False):
    """Reference against the new collider: array path forced on, then off, and
    `first_only` on and off. Returns the full reference culprit list."""
    bad = []
    full = None
    for first_only in (False, True):
        want = reference_colliders(saved, pcb, [OWN, OWN2], clearance,
                                   first_only=first_only, config=config)
        if not first_only:
            full = want
        for min_pairs, path in ((0, 'arrays'), (10 ** 12, 'pair loop')):
            rr._RESTORE_BROAD_MIN_PAIRS = min_pairs
            try:
                got = rr._saved_route_colliders(saved, pcb, [OWN, OWN2], clearance,
                                                first_only=first_only, config=config)
            finally:
                rr._RESTORE_BROAD_MIN_PAIRS = DEFAULT_MIN_PAIRS
            if not same(got, want):
                bad.append(f"{path}, first_only={first_only}: "
                           f"{len(got)} culprits, reference {len(want)}")
    stats['cases'] += 1
    stats['collide' if full else 'clear'] += 1
    if bad or not quiet:
        ok(label, not bad, '; '.join(bad) if bad
           else f"{len(full)} culprit(s) on every path")
    return full


DEFAULT_MIN_PAIRS = rr._RESTORE_BROAD_MIN_PAIRS

# Count what the array path is handed and what it hands on.
_real_candidates = rr._restore_pair_candidates


def _counting(segs, vias, o_segs, o_vias, clr, flat, h2h):
    arrays = rr._RESTORE_BROAD_MIN_PAIRS == 0
    n = 0
    for item in _real_candidates(segs, vias, o_segs, o_vias, clr, flat, h2h):
        n += 1
        yield item
    if arrays:
        stats['array_calls'] += 1
        stats['candidates'] += n
        stats['pairs'] += (len(segs) * (len(o_segs) + len(o_vias))
                           + len(vias) * (len(o_vias) + len(o_segs)))


rr._restore_pair_candidates = _counting


# ---------------------------------------------------------------------------
print("1. Edge cases")
# The saved route: one 0.2 mm track along y=0 from x=0 to x=10 on F.Cu. At the
# flat 0.2 clearance a 0.2 mm foreign track collides when its centreline is
# closer than 0.4 mm.
def saved_track():
    return {'new_segments': [make_seg(0, 0, 10, 0, net_id=OWN, width=0.2)],
            'new_vias': []}


def edge(label, foreign_segs=(), foreign_vias=(), saved=None, config=None,
         want=None):
    pcb = make_pcb(segments=list(foreign_segs), vias=list(foreign_vias))
    full = compare(label, saved or saved_track(), pcb, config)
    if want is not None:
        ok(f"   ... and that is {'a collision' if want else 'clear'}",
           bool(full) == want)


STEP = 2.0 ** -40          # a few hundred last-digit steps at 0.4
edge("parallel, exactly at the threshold",
     [make_seg(0, 0.4, 10, 0.4, net_id=9)])
edge("parallel, one step inside the threshold",
     [make_seg(0, 0.4 - STEP, 10, 0.4 - STEP, net_id=9)], want=True)
edge("parallel, one step outside the threshold",
     [make_seg(0, 0.4 + STEP, 10, 0.4 + STEP, net_id=9)], want=False)
edge("touching end to end", [make_seg(10, 0, 20, 0, net_id=9)], want=True)
edge("crossing in the middle", [make_seg(5, -5, 5, 5, net_id=9)], want=True)
edge("crossing, both far longer than the threshold",
     [make_seg(-40, -30, 60, 30, net_id=9)], want=True)
edge("collinear and overlapping", [make_seg(3, 0, 7, 0, net_id=9)], want=True)
edge("collinear with a gap beyond the threshold",
     [make_seg(10.5, 0, 12, 0, net_id=9)], want=False)
edge("zero-length foreign segment on the track",
     [make_seg(5, 0.1, 5, 0.1, net_id=9)], want=True)
edge("zero-length foreign segment at the threshold",
     [make_seg(5, 0.4, 5, 0.4, net_id=9)])
edge("near-zero-length foreign segment (below the kernel's guard)",
     [make_seg(5, 0.39, 5 + 1e-6, 0.39, net_id=9)], want=True)
edge("zero-length saved segment beside a track",
     [make_seg(0, 0.3, 10, 0.3, net_id=9)],
     saved={'new_segments': [make_seg(5, 0, 5, 0, net_id=OWN)], 'new_vias': []},
     want=True)
edge("same place, another layer",
     [make_seg(0, 0, 10, 0, net_id=9, layer='B.Cu')], want=False)
edge("same place, the own net", [make_seg(0, 0.1, 10, 0.1, net_id=OWN2)],
     want=False)
edge("same place, net 0", [make_seg(0, 0.1, 10, 0.1, net_id=0)], want=False)
edge("on the edge of the saved route's box",
     [make_seg(10.4, -3, 10.4, 3, net_id=9)])
edge("just past the edge of the box",
     [make_seg(10.4 + STEP, -3, 10.4 + STEP, 3, net_id=9)], want=False)
edge("a wide foreign track reaching in from outside",
     [make_seg(0, 1.45, 10, 1.45, net_id=9, width=2.4)], want=True)
edge("foreign via exactly at the threshold",
     foreign_vias=[make_via(5, 0.55, net_id=9, size=0.5)])
edge("foreign via inside the threshold",
     foreign_vias=[make_via(5, 0.5, net_id=9, size=0.5)], want=True)
edge("foreign via off the end of the track",
     foreign_vias=[make_via(10.6, 0, net_id=9, size=0.5)], want=False)
via_saved = {'new_segments': [], 'new_vias': [make_via(0, 0, net_id=OWN, size=0.5)]}
edge("saved via against a foreign via, copper clear, drills inside the floor",
     foreign_vias=[make_via(0.75, 0, net_id=9, size=0.5)], saved=via_saved,
     config=cfg(h2h=0.5), want=True)
edge("the same pair with no hole-to-hole floor",
     foreign_vias=[make_via(0.75, 0, net_id=9, size=0.5)], saved=via_saved,
     config=cfg(), want=False)
edge("saved via against a foreign track on another layer than its pair",
     [make_seg(-5, 0.4, 5, 0.4, net_id=9, layer='In1.Cu')], saved=via_saved,
     want=True)
edge("a class clearance that makes a clear pair collide",
     [make_seg(0, 0.5, 10, 0.5, net_id=9)], config=cfg(classes={9: 0.35}),
     want=True)
edge("a layer rule that relaxes it again",
     [make_seg(0, 0.5, 10, 0.5, net_id=9)],
     config=cfg(classes={9: 0.35}, layer_clearances={'F.Cu': 0.1}), want=False)
edge("nothing on the board", want=False)

# The scalar test squares its threshold, so a negative one (a clearance below
# minus the two half widths) still refuses a pair closer than its magnitude.
# The saved route's box is empty at such a value, so this is asked of the
# broad phase directly: the pair must be among its candidates.
_s = make_seg(0, 0, 10, 0, net_id=OWN, width=0.2)
_o = make_seg(0, 0.3 - 1e-7, 10, 0.3 - 1e-7, net_id=9, width=0.2)
_thr = 0.1 + 0.1 - 0.5
_refused = rr._seg_seg_dist_sq(0, 0, 10, 0, 0, 0.3 - 1e-7, 10, 0.3 - 1e-7) < _thr * _thr
rr._RESTORE_BROAD_MIN_PAIRS = 0
try:
    _cands = list(rr._restore_pair_candidates([_s], [], [_o], [], None, -0.5, 0.0))
finally:
    rr._RESTORE_BROAD_MIN_PAIRS = DEFAULT_MIN_PAIRS
ok("a pair inside a negative threshold's magnitude is a candidate",
   _refused and [(c, id(o)) for _m, c, o in _cands] == [('ss', id(_o))])

# ---------------------------------------------------------------------------
print("2. Randomised boards")


def snap(rng, lo, hi):
    return round(rng.uniform(lo, hi) / 0.05) * 0.05


def random_seg(rng, net, span, short=False):
    x, y = snap(rng, 0, span), snap(rng, 0, span)
    kind = rng.random()
    reach = rng.choice([0.05, 0.1, 0.5]) if short else rng.choice([0.5, 1, 2, 4])
    if kind < 0.08:
        x2, y2 = x, y                                   # zero length
    elif kind < 0.4:
        x2, y2 = x + reach, y
    elif kind < 0.7:
        x2, y2 = x, y + reach
    else:
        d = reach * rng.choice([1, -1])
        x2, y2 = x + reach, y + d
    return make_seg(x, y, x2, y2, net_id=net, layer=rng.choice(LAYERS),
                    width=rng.choice([0.1, 0.15, 0.2, 0.3, 1.0]))


def random_via(rng, net, span):
    size = rng.choice([0.35, 0.5, 0.6, 0.8])
    return make_via(snap(rng, 0, span), snap(rng, 0, span), net_id=net,
                    size=size, drill=rng.choice([0.0, 0.2, 0.3]),
                    layers=('F.Cu', 'B.Cu'))


def random_case(seed):
    rng = random.Random(seed)
    span = rng.choice([3.0, 8.0, 20.0, 40.0])
    foreign_nets = [0, OWN2, 5, 6, 7, 8, 9]
    board_segs = [random_seg(rng, rng.choice(foreign_nets), span)
                  for _ in range(rng.randrange(0, 60))]
    board_vias = [random_via(rng, rng.choice(foreign_nets), span)
                  for _ in range(rng.randrange(0, 12))]
    mode = seed % 4
    if mode == 0:
        config = None
    elif mode == 1:
        config = cfg(h2h=rng.choice([0.0, 0.25, 0.5]))
    elif mode == 2:
        config = cfg(classes={5: 0.35, 7: 0.1, OWN2: 0.3},
                     h2h=rng.choice([0.0, 0.25]))
    else:
        config = cfg(classes={6: 0.4, 9: 0.25},
                     layer_clearances={'In1.Cu': 0.1, 'B.Cu': 0.3}, h2h=0.25)
    own_nets = [OWN, OWN] if seed % 3 else [OWN, OWN2]
    saved_segs = [random_seg(rng, rng.choice(own_nets), span, short=seed % 2 == 0)
                  for _ in range(rng.randrange(0, 25))]
    saved_vias = [random_via(rng, rng.choice(own_nets), span)
                  for _ in range(rng.randrange(0, 4))]
    pcb = make_pcb(segments=board_segs + saved_segs, vias=board_vias + saved_vias)
    if seed % 5 < 2:
        # the step was handed all of it; then some of it moved, and more arrived
        rr.mark_input_copper(pcb)
        for o in rng.sample(board_segs, len(board_segs) // 4):
            o.end_x += 0.05
        for o in rng.sample(board_vias, len(board_vias) // 4):
            o.y += 0.05
        pcb.segments.extend(random_seg(rng, rng.choice([5, 9]), span)
                            for _ in range(8))
        pcb.vias.extend(random_via(rng, 9, span) for _ in range(2))
    pcb.segments = [s for s in pcb.segments if s not in saved_segs]
    pcb.vias = [v for v in pcb.vias if v not in saved_vias]
    if seed % 7 == 0:
        rng.shuffle(pcb.segments)
    return {'new_segments': saved_segs, 'new_vias': saved_vias}, pcb, config


N_RANDOM = 600
before = len(failures)
for seed in range(N_RANDOM):
    saved, pcb, config = random_case(seed)
    compare(f"random board {seed}", saved, pcb, config, quiet=True)
ok(f"{N_RANDOM} randomised boards: the same culprits, objects and order",
   len(failures) == before, f"{len(failures) - before} differ")

# A crowded box in one piece, so several chunks of the array pass are taken.
rr._RESTORE_BROAD_CHUNK = 256
try:
    before = len(failures)
    for seed in range(1000, 1040):
        saved, pcb, config = random_case(seed)
        compare(f"random board {seed}, small chunks", saved, pcb, config, quiet=True)
    ok("40 more with the array pass cut into small chunks",
       len(failures) == before, f"{len(failures) - before} differ")
finally:
    rr._RESTORE_BROAD_CHUNK = 1 << 16

# ---------------------------------------------------------------------------
print("3. Non-vacuity")
ok("the array path ran", stats['array_calls'] > 500, f"{stats['array_calls']} calls")
ok("it left most pairs out",
   0 < stats['candidates'] < stats['pairs'] / 4,
   f"{stats['candidates']} candidates of {stats['pairs']} pairs")
ok("the cases hold both verdicts",
   stats['collide'] > 100 and stats['clear'] > 100,
   f"{stats['collide']} colliding, {stats['clear']} clear")
ok("the default sends a small call to the pair loop and a large one to arrays",
   0 < DEFAULT_MIN_PAIRS < 10 ** 6, f"{DEFAULT_MIN_PAIRS} pairs")


# A broad phase that drops the colliding pair must be caught by `compare`.
def _dropping(segs, vias, o_segs, o_vias, clr, flat, h2h):
    for mine, code, o in _real_candidates(segs, vias, o_segs, o_vias, clr, flat, h2h):
        if code != 'ss':
            yield mine, code, o


rr._restore_pair_candidates = _dropping
try:
    pcb = make_pcb(segments=[make_seg(0, 0.3, 10, 0.3, net_id=9)])
    want = reference_colliders(saved_track(), pcb, [OWN], 0.2)
    got = rr._saved_route_colliders(saved_track(), pcb, [OWN], 0.2)
    ok("a broad phase that drops a colliding pair differs from the reference",
       bool(want) and not same(got, want))
finally:
    rr._restore_pair_candidates = _real_candidates

print()
if failures:
    print(f"FAIL: test_1939_restore_colliders_vectorised ({len(failures)}): "
          + "; ".join(failures[:8]))
    sys.exit(1)
print("PASS: test_1939_restore_colliders_vectorised")
