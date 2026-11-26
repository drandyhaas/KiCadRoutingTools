#!/usr/bin/env python3
"""The field's ties take the supply pads BESIDE a group's parts as well as those beneath them (awx/field_ties.py:
beside, TIE_BESIDE, lay).

  python3 tests/test_622_field_ties_beside.py

The bus step ties the pads on a poured net that stand beneath its parts before a lane is planned (beneath). A
decoupling capacitor a millimetre outside an array's last row stands where the lanes leave that face; left to the
route step it had a lane over it on every layer a via needs, and its net shipped open. So the pads within TIE_BESIDE
of a part's pad box are tied before the lanes too, each by copper kept within TIE_NEAR of its own pad.

On a board of its own -- an array's pads on a 4 x 4 grid at 0.5 mm, a ground net poured, a signal net not:

1. beneath: a ground pad inside the array's pad box, and no other;
2. beside: a ground pad 1 mm outside the box, and one TIE_BESIDE out; not one beyond TIE_BESIDE, not the pad inside
   the box (beneath's), not a pad of a net with no pour, not a plated through-hole pad, not the array's own pads;
3. beside measures from the NEAREST box when there are two parts, and to a box's corner diagonally;
4. a board whose parts have no pads: no pad is beside them.
"""
import math
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import field_ties as ft  # noqa: E402
from synth import make_net, make_pad, make_pcb  # noqa: E402

fails = []
GND, SIG = 1, 2


def check(cond, msg):
    print(('ok   ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


def fp(ref, pads):
    return types.SimpleNamespace(reference=ref, pads=pads)


def names(pads):
    return sorted(f'{p.component_ref}.{p.pad_number}' for p in pads)


# the array U1: 16 pads at 0.5 mm, its pad box (0, 0)-(1.5, 1.5)
arr = [make_pad(GND if (i + j) % 2 else SIG, 0.5 * i, 0.5 * j, ref='U1', num=f'{i}{j}', size_x=0.25, size_y=0.25)
       for i in range(4) for j in range(4)]
TIE = ft.TIE_BESIDE
if not (1.0 < TIE <= 3.0):
    raise SystemExit(f'BROKEN TEST: TIE_BESIDE {TIE} mm is not the reach these pads are placed for')
parts = {
    'U1': fp('U1', arr),
    'C1': fp('C1', [make_pad(GND, 0.75, 0.75, ref='C1', num='1', layers=('B.Cu',))]),          # beneath
    'C2': fp('C2', [make_pad(GND, 2.5, 0.75, ref='C2', num='1', layers=('B.Cu',))]),           # 1 mm out
    'C3': fp('C3', [make_pad(GND, 0.75, -TIE, ref='C3', num='1', layers=('B.Cu',))]),          # TIE_BESIDE out
    'C4': fp('C4', [make_pad(GND, 1.5 + TIE + 0.3, 0.75, ref='C4', num='1')]),                 # beyond the reach
    'C5': fp('C5', [make_pad(SIG, 2.5, 1.0, ref='C5', num='1')]),                              # no pour
    'J1': fp('J1', [make_pad(GND, 0.75, 2.5, ref='J1', num='1', drill=0.8, pad_type='thru_hole',
                             layers=('*.Cu',))]),                                              # plated through
}
pcb = make_pcb(nets={GND: make_net(GND, 'GND'), SIG: make_net(SIG, 'SIG')}, footprints=parts)
poured = {GND}

# ---- 1. beneath
check(names(ft.beneath(pcb, {'U1'}, poured)) == ['C1.1'], '1. beneath: the ground pad inside the pad box alone -- '
      f'{names(ft.beneath(pcb, {"U1"}, poured))}')

# ---- 2. beside
got = names(ft.beside(pcb, {'U1'}, poured, TIE))
check(got == ['C2.1', 'C3.1'], f'2. beside: the ground pads 1 mm and TIE_BESIDE out; not one beyond, not the pad '
      f'beneath, not a net with no pour, not a plated through-hole pad, not the array\'s own -- {got}')

# ---- 3. two parts, and a corner
d = 1.0 / math.sqrt(2.0)                                        # a pad 1 mm off the box's corner, diagonally
far = make_pad(GND, 1.5 + d, 1.5 + d, ref='C6', num='1')
near_b = make_pad(GND, 10.0 + 1.5 + TIE + 0.3, 0.75, ref='C7', num='1')     # beyond U1's reach and U2's
u2 = [make_pad(SIG, 10.0 + 0.5 * i, 0.5 * j, ref='U2', num=f'{i}{j}', size_x=0.25, size_y=0.25)
      for i in range(4) for j in range(4)]
c8 = make_pad(GND, 10.0 - 1.0, 0.75, ref='C8', num='1')                     # 1 mm outside U2 alone
pcb2 = make_pcb(nets={GND: make_net(GND, 'GND'), SIG: make_net(SIG, 'SIG')},
                footprints={'U1': fp('U1', arr), 'U2': fp('U2', u2), 'C6': fp('C6', [far]),
                            'C7': fp('C7', [near_b]), 'C8': fp('C8', [c8])})
got = names(ft.beside(pcb2, {'U1', 'U2'}, poured, TIE))
check(got == ['C6.1', 'C8.1'], f'3. two parts: a pad 1 mm off U1\'s corner and one 1 mm beside U2 alone; not one '
      f'beyond both -- {got}')

# ---- 4. parts with no pads
pcb3 = make_pcb(nets={GND: make_net(GND, 'GND')}, footprints={'U1': fp('U1', []), 'C2': parts['C2']})
check(ft.beside(pcb3, {'U1'}, poured, TIE) == [], '4. a part with no pads: no pad beside it')

print(f'{"FAILED" if fails else "passed"}: {len(fails)} failure(s)')
sys.exit(1 if fails else 0)
