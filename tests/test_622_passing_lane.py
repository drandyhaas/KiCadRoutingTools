#!/usr/bin/env python3
"""A finding at an array's ends is the FANOUT's only where every lane it names ENDS there; a lane that only passes is
the solve's to move (awx/whole_feedback.py --now / --own, awx/whole_route.py audit_layer_cuts).

  python3 tests/test_622_passing_lane.py

The loop stops a round at once -- ENDS CROWDED, the fanout's to change -- when its audit finds two lanes short of
their pitch at the teeth or the berths: no solve moves an end. It read "at the berths" off the place alone. A lane to
a part's far side that crossed the approach of the near side's last berth, on that berth's layer, was so read: the
loop stopped with nothing asked of the solve, the berth's lane shipped open, and the group's model was told to move
a berth that had one place. The passing lane can leave that layer there -- the loop's own layer cut -- and the solve
is owed the round that asks it.

On a part with berths on two faces (A and B end on its west face, C on its south):
1. --own names, for each finding at the ends, the lanes at their OWN end there: A of [A, C], both of [A, B];
2. --now stops at once (exit 0) on [A, B] alone -- both end there: the fanout's;
3. ...not on [A, C] alone (exit 1): C only passes;
4. ...nor on the two together: the solve's own finding first, said;
5. the layer cut: with that reading the lane PASSING is held off the layer there (C under A); by the cut's box alone,
   A's end 0.7 mm off, no cut was made; and two lanes that both end there get none either way;
6. a TWIST -- two lanes found on top of each other, far from both ends, that the round's solve crossed nowhere: the
   geometry took one across the other's approach where its handoff had no room, and no crossing of the solve's
   stands there to price, so the same plan came back with the same twist. The one whose own end is the farther is
   held off the layer there; none where the solve crossed the two (its own crossing to price), where they are only
   short of their pitch, where both ends are as far, or with no solve's crossings given.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


ENV = {k: v for k, v in os.environ.items() if not k.startswith(('AWX_', 'FB_', 'ROUTE_'))}
ENV['ROUTE_LAYERS'] = 'F.Cu,B.Cu,In1.Cu,In2.Cu'
# (the teeth 30 mm west; the part's berths: A, B on its west face x = 30, C on its south face y = 3)
SIDECAR = {'ends': {'A': [[0.0, 0.0], [30.0, 0.0]], 'B': [[0.0, 0.8], [30.0, 0.8]], 'C': [[0.0, 1.6], [33.0, 3.0]]},
           'tooth_layer': {n: 'F.Cu' for n in 'ABC'}, 'dest_layer': {n: 'In1.Cu' for n in 'ABC'},
           'tooth_dir': {n: 'right' for n in 'ABC'}, 'stub_dir': {'A': 'left', 'B': 'left', 'C': 'down'}, 'chi': 1}
PASS, BOTH = [29.3, 0.2, 'PITCH', ['A', 'C']], [29.6, 0.4, 'PITCH', ['A', 'B']]


def feedback(td, mode, hot):
    hp = os.path.join(td, 'hot.json')
    json.dump({'hot': hot}, open(hp, 'w'))
    r = subprocess.run([sys.executable, 'whole_feedback.py', mode, os.path.join(td, 'b.plan.json'), hp], cwd=AWX,
                       env=ENV, capture_output=True, text=True)
    if r.returncode not in (0, 1):
        raise SystemExit(f'BROKEN TEST: whole_feedback {mode}: rc {r.returncode}: {r.stderr[-300:]}')
    return r.returncode, r.stdout


with tempfile.TemporaryDirectory() as td:
    json.dump(SIDECAR, open(os.path.join(td, 'b.plan.json'), 'w'))
    rc, so = feedback(td, '--own', [PASS, BOTH])
    ln = next((x for x in so.splitlines() if x.startswith('OWN ')), None)
    rows = {tuple(r['lanes']): r['own'] for r in json.loads(ln[4:])} if ln else {}
    check('1. --own: the lane passing is not at its own end', rows.get(('A', 'C')) == ['A'], rows)
    check('1. ...two lanes that end there both are', rows.get(('A', 'B')) == ['A', 'B'], rows)
    rc, so = feedback(td, '--now', [BOTH])
    check('2. --now: both end there -- the fanout\'s, stopped at once', rc == 0 and 'PITCH A/B at the berths' in so, so)
    rc, so = feedback(td, '--now', [PASS])
    check('3. ...one only passes: not the fanout\'s', rc == 1, so)
    rc, so = feedback(td, '--now', [PASS, BOTH])
    check('4. ...and with it in the round, the solve\'s first', rc == 1 and 'the solve\'s first' in so and 'A/C' in so, so)

    # 5. the layer cut
    os.environ.update(ENV)
    sys.path.insert(0, AWX)
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    import whole_route as wr  # noqa: E402
    audit = os.path.join(td, 'p.audit')
    open(audit, 'w').write(
        'PITCH A       C       In1.Cu min 0.008  over 0.48 mm  at (29.30,0.20)  corridor 0 s  29.30 trunk  (bar 0.2454)\n'
        'PITCH A       B       In1.Cu min 0.011  over 0.45 mm  at (29.60,0.40)  corridor 0 s  29.60 trunk  (bar 0.2454)\n')
    geo = {'lanes': {'A': {'xy': [[0.0, 0.0], [30.0, 0.0]]}, 'B': {'xy': [[0.0, 0.8], [30.0, 0.8]]},
                     'C': {'xy': [[0.0, 1.6], [33.0, 3.0]]}}}
    own = {(29.3, 0.2): {'A'}, (29.6, 0.4): {'A', 'B'}}
    cuts = wr.audit_layer_cuts(audit, geo, own=own)
    check('5. the passing lane held off the layer there', [(c['lane'], c['island']) for c in cuts] == [('C', 'A')]
          and cuts[0].get('firm') and len(cuts[0]['blocked']) == 1, cuts)
    check('5. ...by the cut\'s box alone, none', wr.audit_layer_cuts(audit, geo) == [], wr.audit_layer_cuts(audit, geo))

    # 6. a twist: A's end 3 mm from the place, C's 6.6 mm
    def twist(min_, at='(27.00,0.30)', cross=frozenset(), a='A', b='C'):
        open(audit, 'w').write(f'PITCH {a}       {b}       In1.Cu min {min_}  over 0.48 mm  at {at}  corridor 0 s  27.00 '
                               f'trunk  (bar 0.2454)\n')
        return [(c['lane'], c['island'], c.get('firm')) for c in wr.audit_layer_cuts(audit, geo, own={}, cross=cross)]
    check('6. a twist the solve never crossed: the lane ending farther held off the layer', twist('0.003')
          == [('C', 'A', True)], twist('0.003'))
    check('6. ...whichever the finding names first', twist('0.003', a='C', b='A') == [('C', 'A', True)])
    check('6. ...none where the solve crossed the two', twist('0.003', cross={frozenset(('A', 'C'))}) == [])
    check('6. ...none where they are only short of their pitch', twist('0.200') == [])
    check('6. ...none where both ends are as far', twist('0.003', at='(15.00,0.50)') == [])
    open(audit, 'w').write('PITCH A       C       In1.Cu min 0.003  over 0.48 mm  at (27.00,0.30)  corridor 0 s  27.00 '
                           'trunk  (bar 0.2454)\n')
    check('6. ...none with no solve\'s crossings given', wr.audit_layer_cuts(audit, geo, own={}) == [])

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
