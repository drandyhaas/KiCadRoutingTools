#!/usr/bin/env python3
"""The whole route's ends: the rules the fanout on our own ends chooses and bans by.

  python3 tests/test_622_ends_fixes.py

whole_ends chooses every net's tooth and berth (fanout_from_plan under PLAN_JUDGE=ends), and the rest of the whole
route is only as good as those ends. This pins the rules it prices and bans by, without a bench to route:

1. stacking (whole_ends._stacks): two lanes' ends at one point on different layers count only where either lane
   changes layer, a pair's twice; two ends on one layer, or apart, are no stack;
2. a move's class (source_realize.move_class): every leg variant of one exit is one class, and another exit another;
3. bans by class (fanout_from_plan.ban_moves): under the ends judge a refused berth bans its class as well as its
   signature, and a pair's two legs are banned jointly -- and under the braid's judges by signature alone;
4. the crossing rule (select_moves.SEL_XING) is the judge's: 2 under PLAN_JUDGE=ends, 1 otherwise;
5. feedback by place (whole_ends.fb_weights): a pair is priced in full where EITHER leg's exit stands on the item's
   layer, so it cannot leave the price by moving one leg; FB_OTHER_LAYER of that on the other layer there; off the place
   less than the other layer there, the nearest other exit included, falling to nothing FB_RADIUS beyond; FB_ESCALATE
   times more for an end named again;
6. the exact ranking (Ends.best_exact): a state whose exact route found no plan ranks after every state whose route
   did, even with the lower objective.
"""
import os
import subprocess
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
os.environ['PLAN_JUDGE'] = 'ends'          # (fanout_from_plan and select_moves read it at import)
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import whole_ends as we  # noqa: E402
import source_realize as sr  # noqa: E402
import fanout_from_plan as ffp  # noqa: E402


def mv(x, y, layer, kind='dogbone', direction='down', site=None, legs=()):
    return NS(exit_pt=(x, y), layer=layer, kind=kind, direction=direction, site=site, legs=list(legs), vias=0)


def opt(*moves, layer=None):
    """a lane's option at one end: (moves per leg, point, layer, vias)"""
    pt = (sum(m.exit_pt[0] for m in moves) / len(moves), sum(m.exit_pt[1] for m in moves) / len(moves))
    return (tuple(moves), pt, layer or moves[0].layer, 0)


def py(code, **env):
    """`code` run in a fresh process under `env` (the settings read at import), its stdout"""
    e = dict(os.environ)
    e.pop('PLAN_JUDGE', None)
    e.pop('SEL_XING', None)
    e.update(env)
    r = subprocess.run([sys.executable, '-c', code], cwd=AWX, env=e, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f'BROKEN TEST: the probe raised -- {r.stderr.strip()[-300:]}')
    return r.stdout.strip()


def main():
    print('=' * 60)
    print('the whole route\'s ends: stacking, classes, bans, the crossing rule, feedback, the exact ranking')
    print('=' * 60)
    fails = []

    # 1. stacking
    far = {'A': opt(mv(0, 0, 'F.Cu')), 'B': opt(mv(5, 0, 'F.Cu')), 'P': opt(mv(10, 0, 'F.Cu'), mv(10.5, 0, 'F.Cu'))}
    lanes = [('A', ('A',)), ('B', ('B',))]
    stacked = {'A': opt(mv(1, 1, 'F.Cu')), 'B': opt(mv(1, 1, 'B.Cu'))}
    cases = [
        ('a stack where A changes layer', lanes, stacked, {'A': 1, 'B': 0}, 1),
        ('no stack where neither changes layer', lanes, stacked, {'A': 0, 'B': 0}, 0),
        ('no stack on ONE layer', lanes, {'A': opt(mv(1, 1, 'F.Cu')), 'B': opt(mv(1, 1, 'F.Cu'))}, {'A': 1}, 0),
        ('no stack where the ends stand apart', lanes,
         {'A': opt(mv(1, 1, 'F.Cu')), 'B': opt(mv(1 + 4 * we.DUP_TOL, 1, 'B.Cu'))}, {'A': 1}, 0),
        ('a PAIR over a single counts twice', [('A', ('A',)), ('P', ('Pp', 'Pn'))],
         {'A': opt(mv(1, 1, 'F.Cu')), 'P': opt(mv(1, 1, 'B.Cu'), mv(1.5, 1, 'B.Cu'))}, {'A': 1}, 2),
    ]
    for what, ln, bo, chg, want in cases:
        to = {l_: far[l_] if l_ in far else far['P'] for l_, _lg in ln}
        got = we._stacks(ln, to, bo, chg)
        if got != want:
            fails.append(f'stacking, {what}: {got}, want {want}')

    # 2. a move's class
    a = mv(1, 2, 'F.Cu', site=(1, 1), legs=[((1, 1), (1, 2), 'F.Cu')])
    b = mv(1, 2, 'F.Cu', site=(1.2, 1), legs=[((1.2, 1), (1, 2), 'F.Cu')])
    c = mv(1.4, 2, 'F.Cu', site=(1, 1))
    if sr.move_sig(a) == sr.move_sig(b):
        fails.append('move_sig: two leg variants read as one move (the class test below would test nothing)')
    if sr.move_class(a) != sr.move_class(b):
        fails.append('move_class: two leg variants of one exit are different classes')
    if sr.move_class(a) == sr.move_class(c):
        fails.append('move_class: two different exits are one class')

    # 3. bans by class, and the braid's judges unchanged
    banned = set()
    ffp.ban_moves(banned, {'SA1': a}, ['SA1'], ['SA1'])
    if ('SA1', sr.move_sig(a)) not in banned or ('SA1', 'class', sr.move_class(a)) not in banned:
        fails.append(f'ban_moves (ends): {banned}, want SA1\'s signature and its class')
    out = py("import fanout_from_plan as f, source_realize as sr\n"
             "from types import SimpleNamespace as NS\n"
             "m = NS(exit_pt=(1, 2), layer='F.Cu', kind='dogbone', direction='down', site=(1, 1), legs=[])\n"
             "b = set(); f.ban_moves(b, {'SA1': m}, ['SA1'], ['SA1']); print(sorted(str(x[1]) for x in b))",
             PLAN_JUDGE='count')
    if 'class' in out:
        fails.append(f'ban_moves (count judge): bans a class ({out}) -- the braid chain must ban by signature alone')

    # 4. the crossing rule's default is the judge's
    for judge, want in (('ends', '2'), ('count', '1')):
        got = py('import select_moves as sm; print(sm.SEL_XING)', PLAN_JUDGE=judge)
        if got != want:
            fails.append(f'SEL_XING under PLAN_JUDGE={judge}: {got}, want {want}')

    # 5. feedback by place
    pair = [opt(mv(10, 0, 'F.Cu'), mv(10.5, 0, 'F.Cu')), opt(mv(20, 0, 'F.Cu'), mv(20.5, 0, 'F.Cu'))]
    half = 10.5 + we.DUP_TOL + we.FB_RADIUS / 2           # half way down the fall
    for what, item, want in (
            ('a pair named by ONE leg\'s place, the option keeping that leg there', {'layer': 'F.Cu', 'points': [(10.5, 0)]},
             {0: 1.0}),
            ('the other layer at the place', {'layer': 'B.Cu', 'points': [(10.5, 0)]}, {0: we.FB_OTHER_LAYER}),
            ('half way down the fall', {'layer': 'F.Cu', 'points': [(half, 0)]}, {0: we.FB_OTHER_LAYER / 2}),
            ('where no leg stands near', {'layer': 'F.Cu', 'points': [(15, 0)]}, {}),
            ('named again', {'layer': 'F.Cu', 'points': [(10.5, 0)], 'times': 2}, {0: we.FB_ESCALATE})):
        got = we.fb_weights(pair, item)
        if set(got) != set(want) or any(abs(got[i] - want[i]) > 1e-9 for i in want):
            fails.append(f'fb_weights, {what}: {got}, want {want}')
    # (moving away a better answer than a layer change: the nearest other exit, 0.4 mm on zynq's teeth, pays less)
    near = we.fb_weights(pair, {'layer': 'F.Cu', 'points': [(10.9, 0)]}).get(0, 0.0)
    other = we.fb_weights(pair, {'layer': 'B.Cu', 'points': [(10.5, 0)]}).get(0, 0.0)
    if not 0.0 < near < other:
        fails.append(f'fb_weights: one exit over (0.4 mm) pays {near:.3f}, the other layer at the place {other:.3f} -- '
                     f'want less, and more than nothing')

    # 6. the exact ranking
    s_fail, s_ok = {'A': (0, 0)}, {'A': (1, 0)}
    parts = {tuple(sorted(s_fail.items())): (1.0, {'exact_failed': True}),
             tuple(sorted(s_ok.items())): (3.0, {'exact_failed': False})}
    fake = NS(pool={k: v[0] for k, v in parts.items()},
              score=lambda s, exact=False: parts[tuple(sorted(s.items()))])
    we._fmt = lambda p: str(p)         # (its log line formats a whole parts record; these carry one key)
    got = we.Ends.best_exact(fake, dict(s_fail), log=lambda *a: None)[0]
    if got != s_ok:
        fails.append(f'best_exact: chose {got}, the state whose exact route found no plan (its objective, 1.0, the '
                     f'estimate\'s), over one routed at 3.0')

    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: stacks counted where a lane changes layer (a pair twice), classes and bans by class under the ends '
          'judge only, the crossing rule the judge\'s, feedback by place and escalated, an exact failure ranked last')
    return 0


if __name__ == '__main__':
    sys.exit(main())
