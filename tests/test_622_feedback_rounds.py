#!/usr/bin/env python3
"""The whole route's feedback across rounds: an end named again is raised, and an open net is named where it fails.

  python3 tests/test_622_feedback_rounds.py

whole_feedback writes the ends the next round's fanout must avoid (whole_ends prices them, by place). This pins, on a
two-lane sidecar of its own (no bench to route):

1. an end found crowded again in a LATER round (FB_ROUND) is not added twice: its 'times' rises (whole_ends doubles its
   price), once a round -- the same round's second finding (a snapped plan's hot file) raises nothing;
2. --name names an open net at the end where it fails: the array nearer its ball cut off from its copper (FB_CONN,
   check_connected's log), or where the round's findings name it (FB_HOTS) -- never both ends when one is known;
3. --refused: a far-face tooth named again is the same item (no duplicate, which would add to its price);
4. --name with no net: the ends model's tiers skip a lane whose ends are all avoided already -- an item's identity is
   its lane, end, points and layer, not its 'times' or 'round';
5. --raise (a round whose fanout laid the round before's ends again): every end last named in that round raised once
   more, no other; with --widen, the other ends of the lanes named added at the base price.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')

SIDECAR = {'chi': 1,
           'ends': {'N1': [[0.0, 0.0], [20.0, 0.0]], 'N2': [[0.0, 1.0], [20.0, 1.0]]},
           'tooth_layer': {'N1': 'F.Cu', 'N2': 'F.Cu'}, 'dest_layer': {'N1': 'F.Cu', 'N2': 'B.Cu'},
           'ends_model': {'over': {'N2': 1}, 'load': {}, 'x': {'N1': 3, 'N2': 1}}}
HOT = {'hot': [[0.2, 0.5, 'STATIC', ['N1']]]}              # in front of the teeth, naming N1
CONN = """  Connectivity issues (1):

  N2 (net 5):
    Segments: 3, Vias: 1, Pads: 2
    Disconnected components: 2
    Disconnected pads:
      (20.10, 1.00) on B.Cu [U2]
"""


def fb_run(td, args, **env):
    """whole_feedback ARGS under ENV, in its own process: its line, and the feedback file after"""
    e = {k: v for k, v in os.environ.items() if not k.startswith('FB_') and k not in ('PLAN_PAIRS', 'BRAID_PAIRS')}
    e.update(env)
    r = subprocess.run([sys.executable, 'whole_feedback.py'] + args, cwd=AWX, env=e, capture_output=True, text=True)
    if r.returncode != 0 or not r.stdout.startswith('whole_feedback:'):
        raise SystemExit(f'BROKEN TEST: whole_feedback {args[0]} -- rc {r.returncode}: {(r.stderr or r.stdout)[-300:]}')
    return r.stdout.strip(), json.load(open(os.path.join(td, 'fb.json')))


def item(fb, lane, end):
    got = [x for x in fb['avoid'] if x['lane'] == lane and x['end'] == end]
    return got


def main():
    print('=' * 60)
    print('the whole route\'s feedback across rounds: raised, not repeated; an open net named where it fails')
    print('=' * 60)
    fails = []
    with tempfile.TemporaryDirectory() as td:
        sc, hot, conn, out = (os.path.join(td, n) for n in ('fo.plan.json', 'hot.json', 'conn.log', 'fb.json'))
        json.dump(SIDECAR, open(sc, 'w'))
        json.dump(HOT, open(hot, 'w'))
        open(conn, 'w').write(CONN)

        # 1. raised a later round, once
        line, fb = fb_run(td, [sc, out, hot], FB_ROUND='1')
        if not line.startswith('whole_feedback: 1 new') or len(item(fb, 'N1', 0)) != 1:
            fails.append(f'round 1: {line}, N1\'s tooth {item(fb, "N1", 0)} -- want it added')
        line, fb = fb_run(td, [sc, out, hot], FB_ROUND='1')
        if not line.startswith('whole_feedback: 0 new') or int(item(fb, 'N1', 0)[0].get('times', 1)) != 1:
            fails.append(f'round 1 again: {line}, {item(fb, "N1", 0)} -- the same round raises nothing')
        line, fb = fb_run(td, [sc, out, hot], FB_ROUND='2')
        if (not line.startswith('whole_feedback: 1 new') or len(item(fb, 'N1', 0)) != 1
                or int(item(fb, 'N1', 0)[0].get('times', 1)) != 2):
            fails.append(f'round 2: {line}, {item(fb, "N1", 0)} -- want one item, times 2')

        # 2. an open net named where it fails
        line, fb = fb_run(td, ['--name', sc, out, 'N2'], FB_ROUND='2', FB_CONN=conn)
        if len(item(fb, 'N2', 1)) != 1 or item(fb, 'N2', 0):
            fails.append(f'--name N2 by its cut-off ball at the berths: {line} -- N2 ends {item(fb, "N2", 0)} / '
                         f'{item(fb, "N2", 1)}, want the berth alone')
        line, fb = fb_run(td, ['--name', sc, out, 'N1'], FB_ROUND='3', FB_HOTS=hot)
        if int(item(fb, 'N1', 0)[0].get('times', 1)) != 3 or item(fb, 'N1', 1):
            fails.append(f'--name N1 by the round\'s finding at the teeth: {line} -- N1 ends {item(fb, "N1", 0)} / '
                         f'{item(fb, "N1", 1)}, want the tooth raised to 3 and the berth untouched')

        # 3. --refused: the same item, no duplicate
        ref = os.path.join(td, 'refused.json')
        json.dump({'splits': [], 'far': ['N2']}, open(ref, 'w'))
        fb_run(td, ['--refused', sc, out, ref], FB_ROUND='3')
        line, fb = fb_run(td, ['--refused', sc, out, ref], FB_ROUND='3')
        if len(item(fb, 'N2', 0)) != 1 or 'whole_feedback: 0 new' not in line:
            fails.append(f'--refused twice in a round: {line}, N2\'s tooth {item(fb, "N2", 0)} -- want one item')

        # 4. --name with no net: N2 (over two) has both ends avoided; the next tier names N1 (most crossed)
        line, fb = fb_run(td, ['--name', sc, out], FB_ROUND='4')
        if '(most crossed)' not in line or 'N1' not in line or len(item(fb, 'N1', 1)) != 1:
            fails.append(f'--name, no net: {line} -- want N1 named most crossed (N2\'s ends all avoided already)')

        # 5. --raise: round 4 named N1's berth; raised once more, nothing else
        before = {(x['lane'], x['end']): int(x.get('times', 1)) for x in fb['avoid']}
        line, fb = fb_run(td, ['--raise', sc, out], FB_ROUND='4')
        after = {(x['lane'], x['end']): int(x.get('times', 1)) for x in fb['avoid']}
        want = dict(before)
        want[('N1', 1)] = want.get(('N1', 1), 0) + 1        # (absent: step 4 failed, and this fails with it)
        if after != want or not line.startswith('whole_feedback: 1 raised'):
            fails.append(f'--raise: {line} -- {after}, want {want} (round 4\'s end alone raised)')
        # --widen: N2 named, its tooth not avoided (taken out here) -- added at the base price
        fb['named'] = ['N2']
        fb['avoid'] = [x for x in fb['avoid'] if not (x['lane'] == 'N2' and x['end'] == 0)]
        json.dump(fb, open(out, 'w'))
        line, fb = fb_run(td, ['--raise', sc, out, '--widen'], FB_ROUND='4')
        if len(item(fb, 'N2', 0)) != 1 or int(item(fb, 'N2', 0)[0].get('times', 1)) != 1:
            fails.append(f'--raise --widen: {line} -- N2\'s tooth {item(fb, "N2", 0)}, want it added at the base price')

    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: an end raised once a later round, not twice in one; an open net named at its failing end by its '
          'connectivity and by the round\'s findings; no duplicate from a refusal; the tiers read an item by identity; '
          '--raise raises the round\'s ends alone, --widen adds the named lanes\' other ends')
    return 0


if __name__ == '__main__':
    sys.exit(main())
