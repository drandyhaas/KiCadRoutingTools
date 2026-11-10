#!/usr/bin/env python3
"""A bus step keeps the array's OTHER balls an escape (awx/fanout_from_plan.py: trial_wins; awx/route_bus.py:
ball_way's next sites).

  python3 tests/test_622_other_balls.py

Under the whole-array fanout the bus's teeth are planned with every other ball of the array. The residue rounds laid
the teeth that plan chose, judged them a few counts worse than the teeth before and put those back -- on which a
supply ball in the bus's own rows had no option at any size, was left bare, and was walled in by the lanes laid past
it. trial_wins: fewer pairs split, then fewer other balls left without an escape, then the count.

And where the lay still walls a ball in, its lanes are lifted and routed again round the via site the ball reaches with
them gone (route_lanes.lift_walls). The nearest site alone was tried: a clearance off the lifted lane's own tooth,
which stands, it left the lane no room to leave past the via. ball_way's `sites` gives the next ones too.

1. trial_wins: the pairs split decide first; equal, the balls stranded; equal, the judge's count;
2. ball_way with `sites` (on kicad_files/ulx3s.kicad_pcb U1's innermost ball): the first way is the one it gives
   alone, each site a via's width or more from every one before it, and each way starts at the ball and ends at its
   site.
"""
import contextlib
import io
import math
import os
import sys

try:
    import ortools  # noqa: F401  (fanout_from_plan's planner imports the solver)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.chdir(os.path.join(ROOT, 'awx'))
BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')

import fanout_from_plan as fp  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BAD = []
N = [0]


def check(ok, what):
    N[0] += 1
    print(('PASS: ' if ok else 'FAIL: ') + what)
    if not ok:
        BAD.append(what)


# ---- 1. trial_wins
check(fp.trial_wins(1, 0, 0, 3, False) and not fp.trial_wins(0, 1, 3, 0, True),
      'the pairs split decide first, whatever the balls stranded and the count say')
check(fp.trial_wins(0, 0, 1, 0, False) and not fp.trial_wins(0, 0, 0, 1, True),
      'the pairs equal, fewer other balls left without an escape wins -- against the count either way')
check(fp.trial_wins(0, 0, 1, 1, True) and not fp.trial_wins(0, 0, 1, 1, False),
      'both equal, the judge\'s count decides')

# ---- 2. a walled ball's next via sites
if not os.path.exists(BOARD):
    print(f'SKIP (via sites): board not present: {BOARD}')
    print(f'\n{N[0] - len(BAD)}/{N[0]} checks passed')
    sys.exit(1 if BAD else 0)
os.environ.pop('FANOUT_JOINT', None)
import route_bus as rb  # noqa: E402

with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    pcb = parse_kicad_pcb(BOARD)
foot = pcb.footprints['U1']
rung = rb._cage_rung(pcb)
ball = min((p for p in foot.pads if p.net_id), key=lambda p: math.hypot(p.global_x - foot.x, p.global_y - foot.y))
with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    one = rb.ball_way(pcb, ball, set(), rung, with_path=True)
    ways = rb.ball_way(pcb, ball, set(), rung, with_path=True, sites=5)
check(one is not None and len(ways) >= 2 and ways[0] == one,
      f'{ball.pad_number}: {len(ways or ())} via site(s) asked for five, the first the one given alone')
far = all(math.hypot(a_[0][0] - b_[0][0], a_[0][1] - b_[0][1]) >= rung[1] - 1e-9
          for i, a_ in enumerate(ways) for b_ in ways[:i])
ends = all(w[2][0] == (ball.global_x, ball.global_y) and w[2][-1] == w[0] for w in ways)
check(far and ends, f'each site a via ({rung[1]}) or more from those before it, each way from the ball to its site')

print(f'\n{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
