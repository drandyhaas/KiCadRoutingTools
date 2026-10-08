#!/usr/bin/env python3
"""A lane's layers on its trunk and ring pieces read where its changes are drawn (awx/corridor.py: piece_u, read by
whole_geo.lay_u).

  python3 tests/test_622_piece_u.py

whole_geo draws a ring lane in two pieces, its trunk to the ring's handoff (HK) and its ring after it, and a layer
change's via in the piece its place falls in: the trunk's up to HK, the ring's after. Each piece's columns read the
lane's layer at their place along it, u; but a ring piece starts where its trunk ENDS, which may be short of the
ring's origin, and its first columns' u short of HK. On the zynq DDR's six-rail bench, the joint fanout on all four
layers, DQ1's changes stood at u 29.097 and 33.284, its ring's handoff at 34.509, and its ring piece started at about
u 33.1: read there, the ring took the trunk's last change again -- drawn F, B for 0.18 mm, F, two vias 0.15 mm apart,
their drills overlapping.

On those numbers, the lane starting on F.Cu, its trunk's columns every 0.4 mm to u 34.6 (a column past the handoff,
as the trunk's last rounds to), its ring's from u 33.1:

1. LIVENESS: read at u itself, the ring's columns change layer where the ring draws no via (the defect); else the
   checks below test nothing (BROKEN TEST);
2. read at piece_u, every layer change on either piece falls at a change the piece draws a via for -- the trunk's two,
   the ring's none -- and the ring starts on the layer the trunk ends on;
3. a ring change just past the handoff is the ring's alone: the trunk's last column, past it, does not take it.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
import corridor as cor  # noqa: E402

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


HK = 34.50875094084331
TRUNK = [19.27 + 0.4 * i for i in range(int((34.6 - 19.27) / 0.4) + 2)]
RING = [33.1 + 0.4 * i for i in range(14)]


def layer(u, chg, start=0):
    """0 F.Cu, 1 B.Cu: the start layer flipped at each change before u"""
    return start ^ (sum(1 for cu in chg if cu < u) & 1)


def flips(cols, read, chg):
    """[(u_a, u_b)]: the column pairs between which the piece's layer, read at read(u), changes"""
    ls = [layer(read(u), chg) for u in cols]
    return [(a, b) for a, b, la, lb in zip(cols, cols[1:], ls, ls[1:]) if la != lb]


def drawn(chg, trunk):
    """the changes a piece draws a via for: the trunk's up to the handoff, the ring's after"""
    return [cu for cu in chg if (cu <= HK) == trunk]


def covers(fl, vias):
    """every layer flip between two columns has a drawn via between them, and every drawn via in the piece's span a flip"""
    return all(any(a < cu <= b for cu in vias) for a, b in fl) and len(fl) == len(vias)


CHG = [29.097, 33.284]
raw = lambda u: u
on_trunk = lambda u: cor.piece_u(u, True, HK)
on_ring = lambda u: cor.piece_u(u, False, HK)

# 1. liveness: the defect
fr = flips(RING, raw, CHG)
if not fr:
    print(f'BROKEN TEST: read at u itself the ring takes no change ({fr}) -- the case reproduces nothing')
    sys.exit(2)
check(not covers(fr, drawn(CHG, False)), f'liveness: read at u itself, the ring changes layer at {fr}, where it draws '
                                         f'no via')

# 2. read at piece_u
ft, fr = flips(TRUNK, on_trunk, CHG), flips(RING, on_ring, CHG)
check(covers(ft, drawn(CHG, True)), f'the trunk changes layer at its two drawn changes ({ft})')
check(covers(fr, drawn(CHG, False)), f'the ring changes layer nowhere, as it draws none ({fr})')
check(layer(on_ring(RING[0]), CHG) == layer(on_trunk(TRUNK[-1]), CHG),
      'the ring starts on the layer the trunk ends on')

# 3. a ring change just past the handoff
CHG3 = [29.097, HK + 0.05]
ft, fr = flips(TRUNK, on_trunk, CHG3), flips(RING, on_ring, CHG3)
check(covers(ft, drawn(CHG3, True)) and covers(fr, drawn(CHG3, False)),
      f'a change just past the handoff is the ring\'s alone: trunk {ft}, ring {fr}')
ft_raw = flips(TRUNK, raw, CHG3)
check(not covers(ft_raw, drawn(CHG3, True)), f'(read at u itself, the trunk\'s last column took it: {ft_raw})')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
