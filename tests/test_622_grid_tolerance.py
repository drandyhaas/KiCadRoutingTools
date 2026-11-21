#!/usr/bin/env python3
"""A ball a few nanometres off its grid has the menu a ball on the grid has (awx/escape_plan.py: escape_menu,
escape_out; awx/joint_escape.py: escape_field_moves; awx/escape_moves.py: enumerate_moves, ON_LINE).

  python3 tests/test_622_grid_tolerance.py

An array's box and its lattice lines are its balls' coordinates, and a board's balls stand a few nanometres off their
grid. A ball's own coordinate was tested against those lines to the nanometre: 2 nm outward, the line along the box's
edge was "outside" it and gone from that ball's menu -- one exit the fewer than its neighbours on the grid had -- and
2 nm inward, a point ON the box's edge was "inside" it. A ball on a line of its array is on it within the chain's
point tolerance (escape_moves.ON_LINE, the relayer's TOL).

A square array on synthetic geometry, every track and via clear; one ball moved off its grid by 0, 1, 2, 5, 11 and
200 nm either way.

1. escape_menu: a ball in the column next to the box's edge, its via at the gap point toward that edge -- by a face
   along that edge its exits are its own line's, the street's and the line's along the box's edge, at every offset;
2. ...its own line alone (`own_line`), the via where a board has it, 2 nm off the gap point: still the ball's line;
3. escape_out: a ball on the box's edge leaves on its own layer by its own line and the gap lines beside it, out to
   half a pitch past the box: the same exits at every offset;
4. escape_field_moves, the pattern: where the gap point toward the nearest corner lies ON the box's edge (a row set
   half a pitch in), it is that point at every offset -- not the one the other way;
5. enumerate_moves: a ball on the ball line beside a band of the array with no balls is offered its side-steps
   into the band at every offset; a pad's menu on a full array is one menu at every offset;
6. one tolerance, and no test of the box at a nanometre left in the three modules.
"""
import os
import re
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)

import rules as _rules  # noqa: E402
_rules.install_defaults()
import escape_moves as em  # noqa: E402
import escape_plan as ep  # noqa: E402
import joint_escape as je  # noqa: E402
import relayer  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


class Cu:
    """every track clear: the menus' own geometry alone"""
    track, clear = 0.1, 0.1

    def net_key(self, n):
        return n

    def path_clear(self, pts, layer, nid):
        return True


P = 0.8
XS = [10 + P * i for i in range(6)]
YS = [20 + P * i for i in range(6)]
GRID = em.Grid(xs=XS, ys=YS, pitch_x=P, pitch_y=P, bbox=(XS[0], YS[0], XS[-1], YS[-1]))
NM = (0, 1, -1, 2, -2, 5, -5, 11, -11, 200, -200)


def pad(x, y):
    return types.SimpleNamespace(global_x=x, global_y=y, net_name='N1', net_id=1, layers=['F.Cu'], pad_number='A1',
                                 size_x=0.4, size_y=0.4, drill=0, pad_type='smd', shape='circle', component_ref='U1')


def sig(moves, x0, y0):
    """the moves by kind, face, layer and exit, the exit from the ball's place on the grid to ten microns (the
    lines here stand a tenth of a millimetre and more apart, the offsets tried a fifth of a micron at most)"""
    return sorted((m.kind, m.direction, m.layer, round(m.exit_pt[0] - x0, 2) + 0.0, round(m.exit_pt[1] - y0, 2) + 0.0)
                  for m in moves)


def same(build):
    """whether `build(offset in mm)` gives one menu at every offset, and that menu"""
    got = {nm: build(nm * 1e-6) for nm in NM}
    return all(g == got[0] for g in got.values()), got[0], {nm: len(g) for nm, g in got.items()}


# 1. escape_menu, the line along the box's edge
def menu1(off):
    bx = XS[-2] + off
    return sig(ep.escape_menu(pad(bx, YS[2]), (bx + P / 2, YS[2] + P / 2), GRID, ['down'], ['B.Cu'], Cu(), side=0),
               XS[-2], YS[2])


ok, m0, n = same(menu1)
check('1. the ball\'s exits by a face along the box\'s edge: the same at every offset', ok, n)
check('1. ...its own line, the street a pitch over and the line along the box\'s edge among them',
      {round(XS[-1] - XS[-2], 2), 0.0} <= {s[3] for s in m0}, sorted({s[3] for s in m0}))


# 2. the own line, the via a board's
def menu2(off):
    return sig(ep.escape_menu(pad(XS[2], YS[2]), (XS[2] + P / 2 + off, YS[2] + P / 2), GRID, ['down'], ['B.Cu'], Cu(),
                              side=0, own_line=True), XS[2], YS[2])


ok, m0, n = same(menu2)
check('2. the ball\'s own line alone, its via a few nanometres off the gap point: one exit at every offset',
      ok and len(m0) == 1 and abs(m0[0][3]) < 1e-3, n)


# 3. escape_out, half a pitch past the box
def menu3(off):
    return sig(ep.escape_out(pad(XS[-1] + off, YS[-1]), GRID, ['down'], Cu()), XS[-1], YS[-1])


ok, m0, n = same(menu3)
check('3. a surface escape of a ball on the box\'s edge: the same exits at every offset', ok, n)
check('3. ...out to the gap line half a pitch past the box', round(P / 2, 2) in {s[3] for s in m0},
      sorted({s[3] for s in m0}))

# 4. the pattern's gap point, ON the box's edge
XS4 = XS + [XS[-1] - P / 2]                      # (a row set half a pitch in from the box's edge)
GRID4 = em.Grid(xs=sorted(XS4), ys=YS, pitch_x=P, pitch_y=P, bbox=(XS[0], YS[0], XS[-1], YS[-1]))


def menu4(off):
    p_ = pad(XS[-1] - P / 2 + off, YS[3])
    mv = je.escape_field_moves(p_, GRID4, 'F.Cu', ['F.Cu', 'B.Cu'], ['down'], Cu(), lambda q, r: True,
                               lambda a, b: True, 0.1, False, pattern=True)
    return sorted({(round(m.site[0] - XS[-1], 2) + 0.0, round(m.site[1] - YS[3], 2) + 0.0) for m in mv
                   if m.site is not None})


ok, m0, n = same(menu4)
check('4. the pattern\'s gap point toward the corner, ON the box\'s edge: that point at every offset',
      ok and m0 and m0[0][0] == 0.0, (m0, n))


# 5. enumerate_moves: the side-steps into a band of the array with no balls
COLS = [0, 1, 2, 3, 7, 8, 9, 10]            # (three empty columns between the fourth and the fifth)
XS5 = [10 + P * i for i in COLS]
YS5 = [20 + P * i for i in range(8)]
GRID5 = em.Grid(xs=XS5, ys=YS5, pitch_x=P, pitch_y=P, bbox=(XS5[0], YS5[0], XS5[-1], YS5[-1]))


def menu5(grid, x, y, **kw):
    def build(off):
        return sig(em.enumerate_moves(pad(x + off, y), grid, ['F.Cu', 'B.Cu'], lambda a, b, L: True,
                                      lambda q, L: True, **kw), x, y)
    return build


plain = len(menu5(GRID5, XS5[3], YS5[3])(0.0))
for col, name in ((3, 'before'), (4, 'after')):
    ok, m0, n = same(menu5(GRID5, XS5[col], YS5[3], street=2, street_pitch=0.35))
    check(f'5. a ball on the ball line {name} an empty band: its side-steps into the band at every offset',
          ok and len(m0) > plain, (plain, n))
for col, name in ((-2, 'next to the box\'s edge'), (-1, 'on the box\'s edge'), (2, 'inside the array')):
    ok, m0, n = same(menu5(GRID, XS[col], YS[2]))
    check(f'5. ...and a pad {name} of a full array: one menu at every offset', ok and len(m0) > 0, n)

# 6. one tolerance
check('6. one tolerance: the relayer\'s, in each module', em.ON_LINE == relayer.TOL == ep._END_TOL)
left = []
for f in ('escape_plan.py', 'joint_escape.py', 'escape_moves.py'):
    for i, ln in enumerate(open(os.path.join(ROOT, 'awx', f)), 1):
        if re.search(r'\b(x0|x1|y0|y1|lo|hi|lo_c|hi_c)\b *[-+] *(h *[-+] *)?1e-[6-9]', ln) or \
                re.search(r'\((x0|y0|lo_c) < [^<]+ < (x1|y1|hi_c)\b', ln):
            left.append(f'{f}:{i}')
check('6. no test of the array\'s box at a nanometre left in the escapes\' modules', not left, left)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
