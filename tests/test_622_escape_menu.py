#!/usr/bin/env python3
"""The escapes of a dog-boned ball (escape_plan.escape_menu, own_via, Copper) -- the menu the arrays' plan and the
group's ends offer a ball from its dog-bone via -- and the move conflicts the plans price (select_moves._conflict,
select_moves._conflicts).

  python3 tests/test_622_escape_menu.py

On a board of its own -- a 5 x 5 ball array at 0.8 mm pitch (0.4 mm round balls on F.Cu), four copper layers, every
ball netted, its nine inner balls dog-boned (a stub on F.Cu to a via on a diagonal gap point, the rows dog-boned out of
the array's middle; the outer ring bare) -- at the zynq LVDS run's sizes (clearance 0.09, escape track 0.12, via
0.45/0.3), this pins:

1. escape_menu, from a ball's via: every move a 'dogbone' whose first leg is that stub on F.Cu and whose runs are on
   another layer, axis-aligned (the selector prices them as lanes: `climb` set), ending a pitch past the outer balls
   (PAST) on a BALL line -- the free lines of a dog-boned field -- or, a STREET escape, on a gap line;
2. the moves keep the escape track and clearance off every other net's via and ball on their layer;
3. SIDESTEPS: lines a pitch and more past the two beside the via, reached along the ball row half a pitch out;
4. a foreign via standing on a ball line takes that line's moves out of the menu;
5. select_moves._conflict: two exits stacked at one point on two layers are no conflict (the whole route stacks them),
   two at one point on one layer are;
6. STREETS: the gap line between rows 1 and 2 free of vias -- a ball beside it escapes along it both ways, its exit on
   that gap line; a ball whose two neighbouring gap lines carry vias has no street escape;
7. a via keeps another lane's run off by the rule (rules.fan_via_clear), not the router's grid ring (fan_via_reach):
   at 0.45 mm vias, 0.12 escapes and 0.1 clearance the ring is past the 0.4 a dog-bone stands off the next ball line,
   and every dog-bone met every neighbouring line's run; select_moves._conflicts passes the reach through.
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)

import rules as _rules  # noqa: E402
_rules.install(_rules.Rules(clearance=0.09, track=0.15, via_size=0.45, via_drill=0.3, fan_track=0.12,
                            hole_to_hole=0.25, source='test'))
import escape_moves as em  # noqa: E402
import escape_plan as ep  # noqa: E402
import select_moves as sm  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

N, PITCH = 5, 0.8


def board(extra='', nc=()):
    nets = '\n'.join(f'  (net {i + 1} "N{i}")' for i in range(N * N))
    pads = '\n'.join(f'    (pad "{i}" smd circle (at {(i % N) * PITCH:.3f} {(i // N) * PITCH:.3f}) (size 0.4 0.4) '
                     f'(layers "F.Cu" "F.Mask") (net {i + 1} "N{i}"))' for i in range(N * N))
    # (each ball's net a second pad on J1, 30 mm off -- a ball whose net has none is a no-connect; N0 meets U2 too)
    jpads = '\n'.join(f'    (pad "{i}" smd rect (at {i * 1.0:.3f} 0) (size 0.5 0.5) (layers "F.Cu" "F.Mask") '
                      f'(net {i + 1} "N{i}"))' for i in range(1, N * N) if i not in nc)
    return f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
{nets}
  (net {N * N + 1} "OTHER")
  (footprint "BGA" (layer "F.Cu") (at 0 0)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
{pads}
  )
  (footprint "BGA2" (layer "F.Cu") (at 20 0)
    (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd circle (at 0 0) (size 0.4 0.4) (layers "F.Cu" "F.Mask") (net 1 "N0"))
  )
  (footprint "CONN" (layer "F.Cu") (at -30 30)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
{jpads}
  )
{extra}
)
"""


def parse(text):
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'b.kicad_pcb')
    with open(p, 'w') as f:
        f.write(text)
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(p), p


fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


# the dog-boned field: each inner ball's via (x, y, net) and its stub (x0, y0, x1, y1, net) on F.Cu
VIAS = [[1.2, 0.4, 7], [2.0, 0.4, 8], [2.8, 0.4, 9], [1.2, 2.0, 12], [1.2, 2.8, 17], [2.8, 2.0, 14], [2.0, 2.8, 18], [2.8, 2.8, 19], [2.0, 2.0, 13]]
STUBS = [[0.8, 0.8, 1.2, 0.4, 7], [1.6, 0.8, 2.0, 0.4, 8], [2.4, 0.8, 2.8, 0.4, 9], [0.8, 1.6, 1.2, 2.0, 12], [0.8, 2.4, 1.2, 2.8, 17], [2.4, 1.6, 2.8, 2.0, 14], [1.6, 2.4, 2.0, 2.8, 18], [2.4, 2.4, 2.8, 2.8, 19], [1.6, 1.6, 2.0, 2.0, 13]]

vtxt = '\n'.join(f'  (via (at {x:.4f} {y:.4f}) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net {n}))'
                 for x, y, n in VIAS)
stxt = '\n'.join(f'  (segment (start {x0:.4f} {y0:.4f}) (end {x1:.4f} {y1:.4f}) (width 0.12) (layer "F.Cu") '
                 f'(net {n}))' for x0, y0, x1, y1, n in STUBS)
pcb, path = parse(board(vtxt + '\n' + stxt))
r = _rules.active()
copper = ep.Copper(pcb, r.fan_track, r.fan_clear)
grid = em.grid_of(pcb.footprints['U1'])
pad = next(p for p in pcb.footprints['U1'].pads if p.pad_number == str(1 * N + 2))     # row 1, column 2
site = ep.own_via(pcb, pad, pad.net_id, grid)
check(site is not None, '1: the ball has its dog-bone via')
layers = ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']
moves = ep.escape_menu(pad, site, grid, ['right', 'down', 'up', 'left'], layers, copper)
check(len(moves) > 0, '1: the ball has escapes')
x1 = grid.bbox[2]
for m in moves:
    check(m.kind == 'dogbone' and m.climb == 1, f'1: {m.direction} {m.layer}: a dogbone with climb set')
    check(m.legs[0][2] == 'F.Cu' and m.legs[0][1] == site, '1: the first leg the stub on F.Cu to the via')
    check(m.layer != 'F.Cu' and all(L == m.layer for _p, _q, L in m.legs[1:]), '1: the runs off F.Cu, on its layer')
    for p, q, _L in m.legs[1:]:
        check(abs(p[0] - q[0]) < 1e-9 or abs(p[1] - q[1]) < 1e-9, f'1: run {p}->{q} axis-aligned')
    along = m.exit_pt[1] if m.direction in ('left', 'right') else m.exit_pt[0]
    on_gap = abs(along / PITCH - round(along / PITCH - 0.5) - 0.5) < 1e-6
    check(abs(along / PITCH - round(along / PITCH)) < 1e-6 or (getattr(m, 'street', False) and on_gap),
          f'1: exit {m.exit_pt} on a ball line, or a street escape\'s on a gap line')
    if m.direction == 'right':
        check(abs(m.exit_pt[0] - (x1 + PITCH)) < 1e-6, f'1: a right exit a pitch past the outer balls: {m.exit_pt}')
# 2. clear of other nets' vias and balls on its layer
for m in moves:
    for p, q, L in m.legs[1:]:
        for v in pcb.vias:
            if v.net_id != pad.net_id:
                check(ep._pt_seg((v.x, v.y), p, q) >= v.size / 2 + r.fan_track / 2 + r.fan_clear - 1e-6,
                      f'2: {m.direction} {L} run past via of net {v.net_id}')
# 3. sidesteps
check(any(getattr(m, 'esc_side', 0) >= 1 for m in moves), '3: sidestep moves offered')
# 4. a foreign via on a ball line takes that line out
lines_right = {round(m.exit_pt[1], 3) for m in moves if m.direction == 'right'}
target = min(lines_right, key=lambda y: abs(y - site[1]))
blk = f'  (via (at {x1 + PITCH / 2 + 0.05:.4f} {target:.4f}) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net {N * N + 1}))'
pcb2, _ = parse(board(vtxt + '\n' + stxt + '\n' + blk))
moves2 = ep.escape_menu(pad, site, grid, ['right'], layers, ep.Copper(pcb2, r.fan_track, r.fan_clear))
check(not any(abs(m.exit_pt[1] - target) < 1e-6 for m in moves2),
      f'4: no right escape on the blocked line {target}')
check(len(moves2) > 0, '4: the other lines still offered')
# 5. stacked exits and no_stack
a = em.Move('A', 'dogbone', 'right', 'B.Cu', (5.0, 1.6), 1, [((0, 0), (0.4, 0.4), 'F.Cu'), ((0.4, 1.6), (5.0, 1.6), 'B.Cu')],
            site=(0.4, 0.4), climb=1)
b = em.Move('B', 'dogbone', 'right', 'In1.Cu', (5.0, 1.6), 1,
            [((0.8, 0), (1.2, 0.4), 'F.Cu'), ((1.2, 1.6), (5.0, 1.6), 'In1.Cu')], site=(1.2, 0.4), climb=1)
check(not sm._conflict(a, b, strict=True, stack=True), '5: stacked exits on two layers: no conflict')
b.layer = 'B.Cu'
b.legs = [((0.8, 0), (1.2, 0.4), 'F.Cu'), ((1.2, 1.6), (5.0, 1.6), 'B.Cu')]
b.__dict__.pop('_spans_c', None)
check(sm._conflict(a, b, strict=True, stack=True), '5: two exits at one point on ONE layer: a conflict')
# 6. streets: the gap line between rows 1 and 2 (y 1.2) free of vias, the test ball (row 1) beside it
gy = 1.5 * PITCH
check(not any(abs(v.y - gy) < 1e-6 for v in pcb.vias if v.net_id <= N * N),
      '6: no U1 via on the gap line between rows 1 and 2 (the field dog-boned out of its middle)')
st_ = [m for m in moves if getattr(m, 'street', False)]
check({m.direction for m in st_} >= {'left', 'right'} and all(abs(m.exit_pt[1] - gy) < 1e-6 for m in st_),
      f'6: the ball beside the street escapes along it both ways: {[(m.direction, m.exit_pt) for m in st_]}')
dpad = next(p for p in pcb.footprints['U1'].pads if p.pad_number == str(3 * N + 2))     # row 3, column 2
dmoves = ep.escape_menu(dpad, ep.own_via(pcb, dpad, dpad.net_id, grid), grid, ['right', 'left'], layers, copper)
check(dmoves and not any(getattr(m, 'street', False) for m in dmoves),
      f'6: a ball with vias on both its gap lines has no street escape: {len(dmoves)} moves')
r10 = _rules.Rules(clearance=0.1, track=0.15, via_size=0.45, via_drill=0.2, fan_track=0.12, source='test')
ring, clear = _rules.fan_via_reach(r10), _rules.fan_via_clear(r10)
check(clear < 0.4 < ring, f'7: at 0.45/0.12/0.1 the grid ring {ring:.5f} is past 0.4 and the rule {clear:.3f} short of it')
# (A along the ball line y 0 on B.Cu past B's dog-bone via, 0.4 off it; B a STREET move on In1.Cu, which buckets every
# move by a via's reach -- select_moves._conflicts)
ra = em.Move('A', 'dogbone', 'left', 'B.Cu', (-1.2, 0.0), 1,
             [((2.4, 0.0), (2.8, 0.4), 'F.Cu'), ((2.8, 0.4), (2.8, 0.0), 'B.Cu'), ((2.8, 0.0), (-1.2, 0.0), 'B.Cu')],
             site=(2.8, 0.4), climb=1)
rb = em.Move('B', 'dogbone', 'up', 'In1.Cu', (1.2, -1.2), 1,
             [((0.8, 0.8), (1.2, 0.4), 'F.Cu'), ((1.2, 0.4), (1.2, -1.2), 'In1.Cu')], site=(1.2, 0.4), climb=1)
rb.street = 1
check(sm._conflict(ra, rb, strict=True, stack=True, xing=2, reach=ring),
      '7: under the grid ring, A\'s run beside B\'s via is a conflict')
check(not sm._conflict(ra, rb, strict=True, stack=True, xing=2, reach=clear),
      '7: under the rule it is none')
check(not sm._conflicts({'A': [ra], 'B': [rb]}, True, stack=True, xing=2, via_reach=clear),
      '7: select_moves._conflicts passes the reach through (a street move among them)')
check(sm._conflicts({'A': [ra], 'B': [rb]}, True, stack=True, xing=2, via_reach=ring) != [],
      '7: ...and the ring\'s conflict with it')


print('escape_menu: ' + ('FAIL' if fails else 'PASS: a dog-boned ball\'s escapes, their clearances, sidesteps, '
                         'blocked lines, streets; stacked exits; the via reach by the rule'))
sys.exit(1 if fails else 0)
