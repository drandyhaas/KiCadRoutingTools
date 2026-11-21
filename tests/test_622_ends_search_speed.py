#!/usr/bin/env python3
"""The ends' search does the same work in less time (awx/whole_ends.py: dive_cuts, route_floor; awx/group_ends.py:
tier_search.run, array_rows).

  python3 tests/test_622_ends_search_speed.py

On two routing layers a round of the group's search spent a fifth of its time in its solve. The rest was Python
and solves that told nothing new:

1. dive_cuts wrote a row for every three lanes whatever their layers -- some ninety thousand a round, of which the
   solver's presolve kept a few hundred. A HELD lane stands on the layers its two held options allow (J.dom): where
   that is one layer, the other's literal is a constant, and a three one of whose lanes cannot stand as a dive
   needs it is no row. The rows left are the same rows: a dive among three held lanes whose layers are all fixed is
   the three lanes' dive literals alone; a lane both of whose layers are open keeps its layer's literal in the row;
   a free lane is asked as before;
2. route_floor's order puzzle asked again of the same state -- the lanes, their places and layers at the two ends,
   their legs, the seed -- is answered from what it was answered, with no solve; another state is solved;
3. the order puzzle is solved under two full strategies (FLOOR_SEARCH); a round on two routing layers without the
   solver's own neighbourhood workers (tier_search.run `lns`), a round on more layers, a bus's ends in turn and the
   whole model's start with them; and each via's reach in an array's rules is worked out once (the calls are
   there).
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)
try:
    from ortools.sat.python import cp_model
except ImportError:
    print('SKIP: ortools is not installed (the ends\' search plans with CP-SAT)')
    sys.exit(77)

import whole_ends as we  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


def model(lanes, places, layers, free=()):
    """a stand-in for the joint model of `lanes`: each lane one option at each end (index 0), at the places `places`
    {lane: (at the teeth, at the berths)}, its ends' layers `layers` {lane: (the teeth's allowed, the berths')} --
    a set of layers each; the lanes `free` asked by the model's own order literals"""
    m = cp_model.CpModel()
    xs = {k: {l: {0: m.NewBoolVar(f'x{k}{l}')} for l in lanes} for k in (0, 1)}
    y = {k: {l: [m.NewBoolVar(f'y{k}{l}{q}') for q in (0, 1)] for l in lanes} for k in (0, 1)}
    pos = {(k, l, 0): places[l][k] for k in (0, 1) for l in lanes}
    dom = {(k, l, 0): (None, None, set(layers[l][k])) for k in (0, 1) for l in lanes}
    lits = {}

    def before(k, a, b):
        if a in free or b in free:
            return lits.setdefault((k, a, b), m.NewBoolVar(f'b{k}{a}{b}'))
        return int(places[a][k] + 5 < places[b][k])
    J = types.SimpleNamespace(m=m, NL=2, lanes=list(lanes), xs=xs, y=y, pos=pos, dom=dom, before=before,
                              before1=lambda a, b: before(1, a, b), npair={l: 1 for l in lanes})
    for a in lanes:                 # (every order literal made before the model is copied, as the builder has them)
        for b in lanes:
            if a != b:
                before(0, a, b), before(1, a, b)
    return J


def state(J, lay):
    """a value per variable: every option taken, each end on the layer `lay` {lane: (teeth, berths)} gives it"""
    vals = [0] * len(J.m.Proto().variables)
    for k in (0, 1):
        for l in J.lanes:
            vals[J.xs[k][l][0].Index()] = 1
            vals[J.y[k][l][lay[l][k]].Index()] = 1
    return vals


def rows(J, free, lay):
    """dive_cuts' rows on a copy of J's model: [each row's literals], the dive literals named 'dive'"""
    mc = J.m.Clone()
    n0, v0 = len(mc.Proto().constraints), len(mc.Proto().variables)
    n = we.dive_cuts(J, mc, set(free), state(J, lay), 2000)
    P = mc.Proto()
    name = lambda i: 'dive' if (i if i >= 0 else -i - 1) >= v0 else ('-' if i < 0 else '') + P.variables[i if i >= 0 else -i - 1].name   # noqa: E731
    out = [sorted(name(i) for i in c.bool_or.literals) for c in list(P.constraints)[n0:]]
    return n, out


# e changes layer (front at the teeth, back at the berths), p stays on the front, q on the back; e before p before q
# at the teeth, p before q before e at the berths: one of the three goes down and back
PLACES = {'e': (0, 300), 'p': (100, 100), 'q': (200, 200)}
LAY = {'e': (0, 1), 'p': (0, 0), 'q': (1, 1)}
ONE = {l: ({LAY[l][0]}, {LAY[l][1]}) for l in LAY}

# 1. the dives' rows
J = model('epq', PLACES, ONE)
n, got = rows(J, (), LAY)
check('1. three held lanes, each end on the one layer its option allows: the dive\'s row is their three dive '
      'literals alone', n == 1 and got == [['dive', 'dive', 'dive']], (n, got))
J = model('epq', PLACES, dict(ONE, p=({1}, {1})))
n, got = rows(J, (), dict(LAY, p=(1, 1)))
check('1. ...the same three with p on the back by its options: it cannot stand as that dive needs it -- no row',
      n == 0 and got == [], (n, got))
BOTH = dict(ONE, p=({0, 1}, {0, 1}))
J = model('epq', PLACES, BOTH)
n, got = rows(J, (), LAY)
check('1. ...a held lane both of whose layers are open keeps its layers\' literals in the row',
      n == 1 and got == [sorted(['dive', 'dive', 'dive', '-y0p0', '-y1p0'])], (n, got))
J = model('epq', PLACES, ONE, free=('p',))
n, got = rows(J, ('p',), LAY)
check('1. ...a FREE lane is asked as before: by its own layer literals and the model\'s order literals',
      n >= 1 and any('-y0p0' in r and '-y1p0' in r and any(x.startswith('-b') for x in r) for r in got), (n, got))
J = model('epq', {'e': (0, 0), 'p': (100, 100), 'q': (200, 200)}, ONE)
n, got = rows(J, (), LAY)
check('1. no crossing, no dive: no row', n == 0, (n, got))

# 2. the order puzzle asked again
we._FLOOR_MEMO.clear()
J = model('epq', PLACES, ONE)
vals = state(J, LAY)
val = lambda v: v if isinstance(v, int) else vals[v.Index()]       # noqa: E731
first = we.route_floor(J, val)
check('2. the order puzzle of a dive: two changes of layer past the model\'s own', first is not None and first[0] >= 1,
      first)
import cp_seed  # noqa: E402
real, asked = cp_seed.solver, [0]


def counted():
    asked[0] += 1
    return real()


cp_seed.solver = counted
try:
    again = we.route_floor(J, val)
    check('2. asked again of the same state: the same answer, and no solve', again == first and asked[0] == 0,
          (again, asked[0]))
    again[2]['e'] = 99
    check('2. ...the answer kept is its own (a caller changing what it was handed changes nothing kept)',
          we.route_floor(J, val) == first)
    J2 = model('epq', {'e': (0, 0), 'p': (100, 100), 'q': (200, 200)}, ONE)
    v2 = state(J2, LAY)
    other = we.route_floor(J2, lambda v: v if isinstance(v, int) else v2[v.Index()])
    check('2. another state is solved', asked[0] == 1 and other is not None, (asked[0], other))
finally:
    cp_seed.solver = real

# 3. the calls are there
WE = open(os.path.join(ROOT, 'awx', 'whole_ends.py')).read()
GE = open(os.path.join(ROOT, 'awx', 'group_ends.py')).read()
check('3. the order puzzle under its two full strategies', we.FLOOR_SEARCH == ['core', 'no_lp']
      and 's.parameters.subsolvers.extend(FLOOR_SEARCH)' in WE)
check('3. a round on two routing layers without the solver\'s own neighbourhood workers; on more, a bus\'s ends in '
      'turn and the start with them',
      'once=False, lns=True):' in GE and 's.parameters.use_lns = False' in GE
      and 'once=hold_y, lns=not two, extra=(' in GE and "label=f\"{bus_}'s {side_}, the rest held\")" in GE)
check('3. each via\'s reach in an array\'s rules worked out once', 'ring_[(site_[0], site_[1], r_)] = je._ring(site_, r_)'
      in GE and 'rg_ = ring_[(site[0], site[1], r_)]' in GE)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
