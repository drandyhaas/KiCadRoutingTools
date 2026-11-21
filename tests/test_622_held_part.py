#!/usr/bin/env python3
"""A part the nudge could not clear, held where it came, is copper to an option's tracks and to a lane's way out as
it is to its vias (awx/group_ends.py: rect_seg_gap, pad_on, held_hits, way_of; array_rows' HELD rows, teach_laid,
write_field's --lock).

  python3 tests/test_622_held_part.py

The menus of an array's fanout are measured as if the movable passives beneath it were not there, and the nudge moves
those off the copper afterwards -- and off each bus lane's way out of its exit, laid for the nudge alone. A passive it
cannot clear is HELD where it came: from there on it is copper to the model. Held against the vias alone, a two-pad
passive just below an array's rim, in two lanes' ways out, was moved off them at every laying into a nook where it
grazed a third lane's track; that lane's end was ruled out, another of its ends with the same track took its place,
and the model ended on a field that broke the board's rules, with nothing routed. No via stood near it: the hold
ruled nothing out.

Synthetic geometry: a two-pad passive on the back layer 0.36 mm below a rim, three exits on the rim above it.

1. rect_seg_gap: the gap between a pad's rectangle and a run's centre line -- 0 where the run crosses it, a corner's,
   an end's; against a fine sampling of the run on random cases;
2. pad_on: a pad's layers, `*.Cu`, and a record with no layers (every layer);
3. held_hits, the vias: within the clearance of a pad of another net the option cannot stand; a pad of its own net
   rules nothing out;
4. ...the legs: on a layer the pad has copper on, the option cannot stand; on a layer it has none, nothing; on its
   lane's own layer (the run's), the pad's layers come back for the model to keep the lane's end off;
5. ...the way out, as the nudge is given it (way_of is wayout_runs' own run): the exit over a pad is ruled out on the
   pad's layer and no other; the exit between the pads, and one away from them, are not;
6. the case: the option's own copper ends at the rim, clear of the part -- by the vias alone nothing is ruled out --
   and with its way out the exit is;
7. a held part stays where it came in the nudge (write_field gives it the nudge's --lock), and its record carries
   its pads' layers;
8. standing: the lanes that stand on the part are read off the copper as laid and the ways out -- a lane whose way
   out is over a pad, one whose laid run crosses a pad on the pad's layer -- and not a lane clear of it, one on the
   other layer, or the pad's own net; no menu is read (a ball's menu is narrowed once the search ends, and a
   state's option is then no index of it).
"""
import inspect
import math
import os
import random
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)

import rules as _rules  # noqa: E402
_rules.install_defaults()
import group_ends as ge  # noqa: E402
import joint_escape as je  # noqa: E402
from escape_moves import DIRS  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


R = _rules.active()
CL, TW = R.clearance, R.fan_track
BACK, FRONT = 'B.Cu', 'F.Cu'
# (the passive: two pads on the back layer, their near edges 0.36 mm past the rim y = 0, 0.4 mm between them)
HX, HY, PY = 0.28, 0.31, 0.67
PADS = [(0.48, PY, HX, HY, 'P', (BACK, 'B.Mask', 'B.Paste')), (-0.48, PY, HX, HY, 'G', (BACK, 'B.Mask', 'B.Paste'))]
PAD_LY = PADS[1][5]
piece = lambda legs=(), vias=(): je.Piece('S', list(legs), list(vias), [])      # noqa: E731
exit_at = lambda x: types.SimpleNamespace(exit_pt=(x, 0.0), direction=next(d for d, v in DIRS.items() if v == (0, 1)))  # noqa: E731

# 1. rect_seg_gap
g = ge.rect_seg_gap
check('1. a run across the rectangle: 0', g(0, 0, 1, 1, (-3, 0), (3, 0)) == 0.0)
check('1. a run inside the rectangle: 0', g(0, 0, 1, 1, (0.2, 0.2), (0.5, 0.5)) == 0.0)
check('1. a run along a side, 1 mm off', abs(g(0, 0, 1, 1, (2, -5), (2, 5)) - 1.0) < 1e-12)
check('1. a run ending off a corner', abs(g(0, 0, 1, 1, (2, 2), (3, 3)) - math.sqrt(2)) < 1e-12)
check('1. a diagonal run past a corner: the corner to the run',
      abs(g(0, 0, 1, 1, (3, 0), (0, 3)) - (3 / math.sqrt(2) - math.sqrt(2))) < 1e-9)
check('1. a run of no length is a point', abs(g(0, 0, 1, 1, (1.5, -0.5), (1.5, -0.5)) - 0.5) < 1e-12)
random.seed(7)
worst = 0.0
for _ in range(1500):
    a = (random.uniform(-4, 4), random.uniform(-4, 4))
    b = (random.uniform(-4, 4), random.uniform(-4, 4))
    hx, hy = random.uniform(0.1, 1.5), random.uniform(0.1, 1.5)
    ref = min(math.hypot(max(abs(a[0] + (b[0] - a[0]) * t / 4000) - hx, 0.0),
                         max(abs(a[1] + (b[1] - a[1]) * t / 4000) - hy, 0.0)) for t in range(4001))
    worst = max(worst, abs(ref - g(0, 0, hx, hy, a, b)))
check('1. 1500 random runs: the gap a fine sampling of the run gives', worst < 2e-3, worst)

# 2. pad_on
check('2. a pad has copper on its own layer and not on the other',
      ge.pad_on(PAD_LY, BACK) and not ge.pad_on(PAD_LY, FRONT))
check('2. `*.Cu` is every copper layer', ge.pad_on(('*.Cu', '*.Mask'), FRONT))
check('2. a record with no layers: every layer', ge.pad_on((), FRONT) and ge.pad_on((), BACK))

# 3. the vias
near_g = [((-0.48, 0.2), 0.125, 0.075)]                # (0.035 mm off the pad of net G)
check('3. a via within the clearance of a pad of another net: the option cannot stand',
      ge.held_hits(PADS, piece(vias=near_g), 'S', TW, CL, je.RUN) == (True, []))
check('3. ...of its own net: nothing ruled out',
      ge.held_hits(PADS, je.Piece('G', [], near_g, []), 'G', TW, CL, je.RUN) == (False, []))
check('3. a via a clearance and more away: nothing',
      ge.held_hits(PADS, piece(vias=[((-0.48, -0.2), 0.125, 0.075)]), 'S', TW, CL, je.RUN) == (False, []))

# 4. the legs
leg = lambda L: [((-0.66, -1.0), (-0.66, 0.3), L)]      # noqa: E731  (ends 0.06 mm short of the pad of net G)
check('4. a leg on the pad\'s layer within the clearance: the option cannot stand',
      ge.held_hits(PADS, piece(leg(BACK)), 'S', TW, CL, je.RUN) == (True, []))
check('4. the same leg on the other layer: nothing',
      ge.held_hits(PADS, piece(leg(FRONT)), 'S', TW, CL, je.RUN) == (False, []))
dead, on = ge.held_hits(PADS, piece(leg(je.RUN)), 'S', TW, CL, je.RUN)
check('4. the same leg on its lane\'s own layer: the pad\'s layers, for the model to keep the lane\'s end off',
      not dead and on == [PAD_LY] and ge.pad_on(on[0], BACK) and not ge.pad_on(on[0], FRONT), (dead, on))
check('4. a pad recorded without its layers: every layer',
      ge.held_hits([p[:5] for p in PADS], piece(leg(je.RUN)), 'S', TW, CL, je.RUN) == (False, [()]))

# 5. the way out
import contextlib  # noqa: E402
import io  # noqa: E402
with contextlib.redirect_stdout(io.StringIO()):
    w = ge.wayout_runs([(0, (-0.66, 0.0), exit_at(0).direction, BACK)], [BACK])
way = ge.way_of(exit_at(-0.66), BACK)
check('5. way_of is the run the nudge is given: wayout_runs\' start, end and width',
      len(w) == 1 and way == ((w[0]['start'], w[0]['end']), w[0]['width']), way)
check('5. ...straight out of the exit, a lane\'s bar and more wide',
      way[0][0] == (-0.66, 0.0) and way[0][1][0] == -0.66 and way[0][1][1] > 0.2 and way[1] > R.track, way)
check('5. no way out for an exit with no direction',
      ge.way_of(types.SimpleNamespace(exit_pt=(0.0, 0.0), direction=None), BACK) is None)
hit = lambda x: ge.held_hits(PADS, piece(), 'S', TW, CL, je.RUN, ge.way_of(exit_at(x), BACK))  # noqa: E731
check('5. the exit over a pad: ruled out on the pad\'s layer, and on no other',
      hit(-0.66) == (False, [PAD_LY]) and not ge.pad_on(hit(-0.66)[1][0], FRONT), hit(-0.66))
check('5. the exit between the two pads, its way out a clearance off each: not ruled out',
      hit(0.0) == (False, []), hit(0.0))
check('5. an exit away from the part: not ruled out', hit(1.5) == (False, []), hit(1.5))
check('5. the pad\'s own net leaves by the exit over it',
      ge.held_hits(PADS, je.Piece('G', [], [], []), 'G', TW, CL, je.RUN, ge.way_of(exit_at(-0.66), BACK)) == (False, []))

# 6. the case
own = piece([((-0.66, -1.2), (-0.66, 0.0), BACK)], [((-0.66, -1.2), 0.125, 0.075)])
check('6. an option whose copper ends at the rim: by its vias and legs alone nothing is ruled out',
      ge.held_hits(PADS, own, 'S', TW, CL, je.RUN) == (False, []))
check('6. ...with its lane\'s way out, the exit is ruled out on the part\'s layer',
      ge.held_hits(PADS, own, 'S', TW, CL, je.RUN, ge.way_of(exit_at(-0.66), BACK)) == (False, [PAD_LY]))

# 7. the nudge and the record
src = inspect.getsource(ge.write_field)
check('7. write_field takes the held parts and gives them to the nudge as --lock',
      'locked' in inspect.signature(ge.write_field).parameters and "'--lock'" in src)
main_src = inspect.getsource(ge.main)
check('7. the field is laid with the held parts locked', 'locked=sorted(held)' in main_src)
check('7. a held part\'s record carries its pads\' layers', 'tuple(p_.layers)' in main_src)

# 8. the lanes that stand on the part
DOWN = exit_at(0).direction
NAMES = {1: 'S1', 2: 'S2', 3: 'S3', 4: 'S4', 5: 'G', 6: 'S6'}
laid = [(1, piece([((-0.66, -1.2), (-0.66, 0.0), BACK)]), BACK, TW, 'A'),      # (its way out is over the pad of G)
        (2, piece([((1.5, -1.2), (1.5, 0.0), BACK)]), BACK, TW, 'A'),          # (clear of the part)
        (3, piece([((0.48, -0.2), (0.48, 0.5), je.RUN)]), BACK, TW, 'A'),      # (its run crosses the pad of P)
        (4, piece([((0.48, -0.2), (0.48, 0.5), je.RUN)]), FRONT, TW, 'A'),     # (the same run on the other layer)
        (5, piece([((-0.66, -1.2), (-0.66, 0.0), BACK)]), BACK, TW, 'A'),      # (the pad's own net)
        (6, piece([((-0.66, -1.2), (-0.66, 0.0), FRONT)]), FRONT, TW, 'A')]    # (its way out on the other layer)
ways = [(1, (-0.66, 0.0), DOWN, BACK), (2, (1.5, 0.0), DOWN, BACK), (5, (-0.66, 0.0), DOWN, BACK),
        (6, (-0.66, 0.0), DOWN, FRONT)]
buses = [types.SimpleNamespace(name='b1', leg_lane={'S1': 'l1', 'S2': 'l2', 'S6': 'l6'}),
         types.SimpleNamespace(name='b2', leg_lane={'S3': 'l3', 'S4': 'l4', 'G': 'lg'})]
got = ge.standing(PADS, laid, ways, NAMES.get, buses, CL)
check('8. the lane whose way out is over a pad, and the one whose laid run crosses a pad on its layer',
      got == {'b1': {'l1'}, 'b2': {'l3'}}, got)
check('8. nothing laid on the part: no lane', ge.standing(PADS, laid[1:2], ways[1:2], NAMES.get, buses, CL)
      == {'b1': set(), 'b2': set()})
check('8. no menu is read: the lanes come from the copper as laid',
      not any(w_ in inspect.getsource(ge.standing) for w_ in ('.menu', '.opts', '.state')))
check('8. the lesson is given the field\'s copper and the ways out', 'M_[\'moved\'], laid, wayouts)' in main_src)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
