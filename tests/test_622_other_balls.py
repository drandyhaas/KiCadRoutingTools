#!/usr/bin/env python3
"""A bus step keeps the array's OTHER balls an escape (awx/route_bus.py: ball_way's next sites).

  python3 tests/test_622_other_balls.py

Where the lay still walls a ball in, its lanes are lifted and routed again round the via site the ball reaches with
them gone (route_lanes.lift_walls). The nearest site alone was tried: a clearance off the lifted lane's own tooth,
which stands, it left the lane no room to leave past the via. ball_way's `sites` gives the next ones too.

ball_way with `sites` (on kicad_files/ulx3s.kicad_pcb U1's innermost ball): the first way is the one it gives
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

from kicad_parser import parse_kicad_pcb  # noqa: E402

BAD = []
N = [0]


def check(ok, what):
    N[0] += 1
    print(('PASS: ' if ok else 'FAIL: ') + what)
    if not ok:
        BAD.append(what)


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
