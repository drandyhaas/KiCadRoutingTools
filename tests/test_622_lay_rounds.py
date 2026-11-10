#!/usr/bin/env python3
"""A loop that did not pass lays its other rounds' plans until one leaves no net open.

  python3 tests/test_622_lay_rounds.py

whole_route.lay_rounds: the held round's plan laid with nets open, the loop's other rounds are snapped and laid,
fewest findings first, and the best lay kept. One other round was the limit, and it was not enough: the round of
fewest findings may hold a plan that leaves nets open too (a pair its snap could not lay), while a round after it
lays every lane. This drives lay_rounds with the snap, the gate and the lay stood in for and pins:

1. no limit set: the rounds are tried in the order of their findings until one lays with no net open, and that
   one is kept -- its board and logs copied over the held one's names -- with no round tried after it;
2. LAY_TRY=2: the held plan's and ONE other round's, the better of the two kept, as before;
3. LAY_TRY=1: the held plan's lay alone, nothing else laid;
4. a held plan with no net open: nothing else laid.
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import awx_settings  # noqa: E402
import whole_route as wr  # noqa: E402

FINDINGS = {1: 5, 2: 12, 4: 9}          # (round 3 is the held one)
OPEN = {1: 2, 2: 0, 4: 0}               # each round's lay: nets left open


def drive(lay_try, held_open=2):
    """(the result kept, the rounds laid in order, the lines said) of lay_rounds on a loop of four rounds, round 3 held"""
    with tempfile.TemporaryDirectory() as d:
        loopd = os.path.join(d, 'loop')
        os.makedirs(loopd)
        json.dump({'i': 3}, open(os.path.join(loopd, 'best.json'), 'w'))
        for i in (1, 2, 3, 4):
            for ext in ('.json', '.audit'):
                open(os.path.join(loopd, f'p{i}{ext}'), 'w').write('{}')
        laid, said = [], []
        run0, held0, get0 = wr.run, wr.held_plan, awx_settings.get

        def fake_run(argv, env, *a, **k):
            i = int(os.path.basename(argv[1])[1:-len('.json')])
            return 0, f'smooth: audit: dive {FINDINGS[i]}, static 0, shape 0, swim 0, band 0.000 mm, pitch 0 in the plan\n'

        def fake_lay(hp, tag):
            i = int(tag[2:])
            laid.append(i)
            for nm in ('seq.kicad_pcb', 'route_seq.log'):
                b, e = os.path.splitext(nm)
                open(os.path.join(d, f'{b}{tag}{e}'), 'w').write(f'round {i}')
            return {'open': OPEN[i], 'drc': 0, 'key': (OPEN[i], 0, 30, 800), 'tag': tag,
                    'board': os.path.join(d, f'seq{tag}.kicad_pcb')}
        wr.run = fake_run
        wr.held_plan = lambda loopd_, env, i=None: os.path.join(loopd_, f'held{i}_plan.json')
        awx_settings.get = lambda k, *a: (lay_try if k == 'LAY_TRY' else get0(k, *a))
        try:
            res = {'open': held_open, 'drc': 0, 'key': (held_open, 0, 26, 838), 'tag': '',
                   'board': os.path.join(d, 'seq.kicad_pcb')}
            open(res['board'], 'w').write('held')
            if lay_try == '1':
                # (the chain asks lay_rounds only where more than the held plan may be laid)
                out = res if wr.lay_tries() == 1 else wr.lay_rounds(loopd, d, {}, res, fake_lay, said.append)
            else:
                out = wr.lay_rounds(loopd, d, {}, res, fake_lay, said.append)
            board = open(os.path.join(d, 'seq.kicad_pcb')).read()
        finally:
            wr.run, wr.held_plan, awx_settings.get = run0, held0, get0
        return out, laid, said, board


def main():
    fails = []

    def check(ok, what):
        print(('PASS: ' if ok else 'FAIL: ') + what)
        if not ok:
            fails.append(what)
    out, laid, said, board = drive(None)
    check(laid == [1, 4] and out['open'] == 0 and board == 'round 4',
          f'no limit: rounds laid by their findings until one leaves no net open -- laid {laid}, kept open '
          f'{out["open"]}, the board kept "{board}"')
    check(any('until one lays with no net open' in ln for ln in said), 'its line says the rounds go on until none is open')
    out, laid, said, board = drive('2')
    check(laid == [1] and out['open'] == 2, f'LAY_TRY=2: one other round laid -- laid {laid}, kept open {out["open"]}')
    out, laid, said, board = drive('1')
    check(laid == [] and board == 'held', f'LAY_TRY=1: the held plan alone -- laid {laid}')
    out, laid, said, board = drive(None, held_open=0)
    check(laid == [] and board == 'held', f'a held plan with no net open: nothing else laid -- laid {laid}')
    print(f'\n{5 - len(fails)}/5 checks passed')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
