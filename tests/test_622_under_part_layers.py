#!/usr/bin/env python3
"""The layers a lane may take under a group's part: what stands on its own line, not every pad in the part's box
(awx/whole_frame.py: open_on_line, own_layers, Group.open_line, under_open; their readers in whole_solve, whole_geo
and whole_passages).

  python3 tests/test_622_under_part_layers.py

A two-row connector on the back layer with decoupling caps on the front layer between its rows: the hand-routed board
runs the far row's lanes over the near row on the front layer, between the caps. Read off the part's whole box, each
cap shut the front layer under all of it -- a lane the solve sent there straight to its via was held off it, and the
census the 2-D geometry reads stood the box as copper on that layer, so the lanes were read round the connector's end.

1. open_on_line: a line clear of the caps leaves the front layer open, a line through a cap (or within a lane's reach
   of it) shuts it; the part's own pads' layer is shut under it on every line (the census stands its box there); the
   inner layers stay open;
2. ...where open_layers, reading the whole box, shuts the front layer for every line;
3. ...another part's drilled pad within reach of the line shuts every layer; out of reach it shuts none;
4. ...a part with drilled pins of its own is shut whole, as open_layers has it;
5. own_layers: the layers of the part's OWN pads alone (every layer for a drilled part) -- where its box is copper;
6. the readers: Group.options' ways under parts carry the layers along their own line; the solve's under-part cuts and
   the geometry's bound read the lane's way (under_open); the census and the solve's destination boxes stand a group
   part's box on its own pads' layers.
"""
import ast
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.environ['ROUTE_LAYERS'] = 'F.Cu,B.Cu,In1.Cu,In2.Cu'
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    import whole_frame as wf  # noqa: E402
except ImportError as exc:
    print('SKIP: %s' % exc)
    sys.exit(77)

BAD = []
N = [0]


def check(what, got, want):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


def pad(x, y, sx, sy, layers, drill=0.0, kind='smd'):
    return NS(global_x=x, global_y=y, size_x=sx, size_y=sy, layers=list(layers), drill=drill, pad_type=kind,
              rect_rotation=0.0)


# a two-row connector on the back layer: rows at x 0 and 3, pins every 0.4 mm from y 0 to 8
CONN = [pad(x, 0.4 * i, 0.7, 0.2, ['B.Cu']) for x in (0.0, 3.0) for i in range(21)]
# a cap on the front layer between the rows at y 4.6, its pads 0.56 x 0.62
CAP = [pad(1.0, 4.6, 0.56, 0.62, ['F.Cu']), pad(2.0, 4.6, 0.56, 0.62, ['F.Cu'])]
pcb = NS(footprints={'J': NS(pads=CONN), 'C': NS(pads=CAP)})
BOX = [(-0.35, -0.1, 3.35, 8.1)]
line = lambda y: wf.open_on_line(pcb, ['J'], BOX, (3.6, y), (-0.6, y))      # noqa: E731  (over both rows)
mid = lambda y: wf.open_on_line(pcb, ['J'], BOX, (2.3, y), (0.7, y))        # noqa: E731  (between the rows only)

check('1. a line between the pins, clear of the cap: front and inner layers open', line(2.2),
      frozenset({'F.Cu', 'In1.Cu', 'In2.Cu'}))
check('1. a line through the cap: the front layer shut', line(4.6), frozenset({'In1.Cu', 'In2.Cu'}))
check('1. a line a hair past the cap\'s edge: shut (a lane\'s reach of it)', line(5.0), frozenset({'In1.Cu', 'In2.Cu'}))
check('1. a line well clear of the cap: open', line(5.6), frozenset({'F.Cu', 'In1.Cu', 'In2.Cu'}))
check('1. a line between the rows that meets no pin: the part\'s own pads\' layer still shut under it', mid(2.2),
      frozenset({'F.Cu', 'In1.Cu', 'In2.Cu'}))
check('2. the whole box read, as open_layers does: the front layer shut for every line',
      wf.open_layers(pcb, 'J'), frozenset({'In1.Cu', 'In2.Cu'}))

pcb_h = NS(footprints={'J': NS(pads=CONN), 'H': NS(pads=[pad(1.5, 6.0, 0.6, 0.6, ['*.Cu'], drill=0.3,
                                                              kind='thru_hole')])})
check('3. another part\'s drilled pad on the line: every layer shut',
      wf.open_on_line(pcb_h, ['J'], BOX, (3.6, 6.0), (-0.6, 6.0)), frozenset())
check('3. ...off the line: it shuts nothing there',
      wf.open_on_line(pcb_h, ['J'], BOX, (3.6, 2.2), (-0.6, 2.2)), frozenset({'F.Cu', 'In1.Cu', 'In2.Cu'}))
pcb_t = NS(footprints={'T': NS(pads=[pad(0.0, 2.54 * i, 1.6, 1.6, ['*.Cu'], drill=1.0, kind='thru_hole')
                                     for i in range(4)])})
check('4. a part with drilled pins of its own is shut whole',
      wf.open_on_line(pcb_t, ['T'], [(-0.8, -0.8, 0.8, 8.4)], (1.0, 1.27), (-1.0, 1.27)), frozenset())

check('5. own_layers: the connector\'s own pads\' layer alone', wf.own_layers(pcb, 'J'), frozenset({'B.Cu'}))
check('5. ...every layer for a drilled part', wf.own_layers(pcb_t, 'T'),
      frozenset({'F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu'}))

AW = os.path.join(ROOT, 'awx')
src = {f: open(os.path.join(AW, f)).read() for f in ('whole_frame.py', 'whole_solve.py', 'whole_geo.py',
                                                      'whole_passages.py')}
opt = next(n for n in ast.walk(ast.parse(src['whole_frame.py'])) if isinstance(n, ast.FunctionDef)
           and n.name == 'options')
opt_src = ast.get_source_segment(src['whole_frame.py'], opt)
check('6. the group\'s ways under parts carry the layers along their own line',
      'self.open_line(parts, a, b)' in opt_src and 'open_on(crossed, p, q)' in opt_src
      and 'open_on(crossed, q0, q)' in opt_src and 'self.open[' not in opt_src, True)
check('6. the solve\'s under-part cuts read the lane\'s way',
      'whole_frame.under_open(Fr, ctx, n_)' in src['whole_solve.py'], True)
check('6. the geometry\'s bound reads the lane\'s way',
      'OPEN_N.get((n, r_), OPEN_I[r_])' in src['whole_geo.py'] and 'whole_frame.under_open(Fr, ctx, n_)'
      in src['whole_geo.py'], True)
check('6. the census and the solve\'s destination boxes stand on the part\'s own pads\' layers',
      'self.LAYN[i] in grp.own[r_]' in src['whole_passages.py']
      and 'LAYN[L_] in Fr.group.own[r_]' in src['whole_solve.py'], True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
