#!/usr/bin/env python3
"""#622 `awx/conflict_groups.py`: the whole route's move conflicts stated as groups, EXACTLY.

The joint escape plan (awx/joint_escape.py) gives CP-SAT the conflicts between every ball's escape moves as cliques
and bicliques rather than pair by pair (zynq U1's bus alone: 3.1 million pairs, 129 s). The groups must state the same
relation the whole route uses, pair for pair -- a missing pair lets the plan choose two moves the engine cannot both
lay, an extra one forbids a plan that exists.

What this asserts:

1. `_windows` (the lane test `|c1 - c2| < tol` as maximal windows of coordinates): every pair within the tolerance
   shares a window and no window is `tol` wide -- on a lattice's few wandering values and on STREET lanes a stacking
   pitch apart, which chain wider than the tolerance (zynq U2 in the flow frame; the tight-cluster grouping this
   replaced asserted there).
2. On a real array's menus (the H3 bench's DDR3 DU1, a slice of its nets, every move kind: surface,
   straight, dog-bones, vias-in-pad, climbs, streets): the groups expanded == pages_first._conflicts(strict, stack),
   pair for pair -- nothing missing, nothing extra -- at each crossing rule, the rule passed on both sides.
3. Two plain escapes built to cross: no conflict at xing=1 (select_moves' own default, a crossing counted only
   where a move climbs), a conflict at xing=2 (every same-layer crossing, what a plan of a whole array needs) --
   the check is live, where a slice of a real array need not hold two plain escapes that cross.
4. The rule is not the process's: importing pages_first leaves select_moves.SEL_XING as the environment set it (it
   used to raise it to 2 for the whole process, so the fanout's first greedy choice ran at 1 and every later one, and
   the whole route's ends model, at 2), and the ends model's test (pages_first._conflicts, no rule given) and the
   greedy seed's (select_moves._conflict, no rule given) are one rule.
"""
import contextlib
import io
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.chdir(AWX)
FAIL = []

import conflict_groups as cg  # noqa: E402
import joint_escape as je  # noqa: E402
import pages_first as pf  # noqa: E402
import select_moves as sm  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = os.path.join(AWX, 'fb_t2q_pairs.kicad_pcb')
REF = 'DU1'
N_NETS = 20


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        FAIL.append(what)


def windows_cover(vals, tol):
    ws = cg._windows(vals, tol)
    vs = sorted(set(vals))
    wide = [w for w in ws if w[-1] - w[0] >= tol]
    within = [(a, b) for i, a in enumerate(vs) for b in vs[i + 1:] if b - a < tol]
    missed = [(a, b) for a, b in within if not any(a in w and b in w for w in ws)]
    return ws, wide, missed


# 1. the windows
for name, vals in (('a lattice: two gaps, each a few wandering values', [10.0, 10.0004, 10.0011, 10.8, 10.8007]),
                   ('street lanes a stacking pitch apart, chained wider than the tolerance',
                    [103.499, 103.608, 103.615, 103.714, 103.929, 104.007, 104.008, 104.014, 104.144]),
                   ('one value', [5.0])):
    ws, wide, missed = windows_cover(vals, cg.TOL)
    check(not wide and not missed, f'_windows, {name}: {len(ws)} window(s), none {cg.TOL} wide, every pair within '
          f'{cg.TOL} in one (wide {wide}, missed {missed})')

# 2. and 3. a real array's menus
with contextlib.redirect_stdout(io.StringIO()):
    pcb = parse_kicad_pcb(BOARD)
foot = pcb.footprints[REF]
zones = {z.net_id for z in pcb.zones if z.net_id}
nets = sorted({p.net_name for p in foot.pads if p.net_id and p.net_id not in zones and p.net_name})[:N_NETS]
with contextlib.redirect_stdout(io.StringIO()):
    bm = je.build_menus(pcb, REF, [], nets, ["F.Cu", "B.Cu"], climb=2, street=2)
menu = {k: v for k, v in bm.menu.items() if v}
n_moves = sum(len(v) for v in menu.values())
kinds = {(getattr(m, 'kind', ''), bool(getattr(m, 'climb', 0)), bool(getattr(m, 'street', 0)))
         for v in menu.values() for m in v}
print(f'{REF} of the H3 bench: {len(menu)} balls of {len(nets)} nets, {n_moves} moves, kinds {sorted(kinds)}')
check(len(menu) >= 15 and any(c for _k, c, _s in kinds) and any(st for _k, _c, st in kinds), "the menus are a real test: 15+ balls, climbs and streets among them")


def reference(xing):
    with contextlib.redirect_stdout(io.StringIO()):
        ref = pf._conflicts(menu, strict=True, stack=True, xing=xing)
    out = set()
    for a, i, b, j in ref:
        x, y = (a, i), (b, j)
        out.add((x, y) if x < y else (y, x))
    return out


for xing in (1, 2):
    t = time.time()
    ref_x = reference(xing)
    t_ref = time.time() - t
    t = time.time()
    mine_x = cg.expand(*cg.conflict_groups(menu, stack=True, xing=xing))
    t_mine = time.time() - t
    check(ref_x == mine_x, f'xing={xing}: the groups == pages_first._conflicts at SEL_XING {xing}, pair for pair: '
          f'{len(ref_x)} pairs (missing {len(ref_x - mine_x)}, extra {len(mine_x - ref_x)}; {t_ref:.1f} s pair by '
          f'pair, {t_mine:.1f} s grouped)')
# live: two PLAIN escapes whose lanes cross on one layer -- a ball escaping left along the row gap y = 2.4 and
# another escaping up the column gap x = 1.6, both on F.Cu, neither climbing, their exits on two faces and no via:
# the crossing is their only conflict. select_moves' own default (1: a crossing counts only where a move climbs) does
# not state it, xing=2 does, and each agrees with select_moves._conflict at its own SEL_XING.
from escape_moves import Move  # noqa: E402
F = 'F.Cu'
A = Move('A', 'surface', 'left', F, (0.0, 2.4), 0, legs=[((2.0, 2.0), (2.0, 2.4), F), ((2.0, 2.4), (0.0, 2.4), F)])
B = Move('B', 'surface', 'up', F, (1.6, 0.0), 0, legs=[((1.2, 4.0), (1.6, 4.0), F), ((1.6, 4.0), (1.6, 0.0), F)])
cross = {'A#1': [A], 'B#2': [B]}
pair = (('A#1', 0), ('B#2', 0))
d1 = cg.expand(*cg.conflict_groups(cross, stack=True, xing=1))
d2 = cg.expand(*cg.conflict_groups(cross, stack=True, xing=2))
r1 = sm._conflict(A, B, strict=True, stack=True, xing=1)
r2 = sm._conflict(A, B, strict=True, stack=True, xing=2)
check(pair not in d1 and not r1, f'two plain escapes crossing: not a conflict at xing=1 (groups {pair in d1}, '
      f'select_moves {r1})')
check(pair in d2 and r2, f'two plain escapes crossing: a conflict with xing=2 (groups {pair in d2}, select_moves at 2 '
      f'{r2})')

# 4. the rule is the caller's, never the import's: pages_first was imported above, and the module value is still the
# environment's; with no rule given, the ends model's test (pages_first._conflicts) and the greedy seed's
# (select_moves._conflict) agree on the crossing pair -- one rule for the seed and the judge
want = int(os.environ.get('SEL_XING', '2' if os.environ.get('PLAN_JUDGE') == 'ends' else '1'))
check(sm.SEL_XING == want, f'importing pages_first leaves select_moves.SEL_XING at {want} (it is {sm.SEL_XING})')
judge = pair in {((a, i), (b, j)) if (a, i) < (b, j) else ((b, j), (a, i))
                 for a, i, b, j in pf._conflicts(cross, strict=True, stack=True)}
seed = sm._conflict(A, B, strict=True, stack=True)
check(judge == seed == (want >= 2), f'with no rule given, the ends model\'s test ({judge}) and the greedy seed\'s '
      f'({seed}) agree, at SEL_XING {want}')

print(f'\n{len(FAIL)} failure(s)')
sys.exit(1 if FAIL else 0)
