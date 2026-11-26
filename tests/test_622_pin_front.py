#!/usr/bin/env python3
"""A plated through-hole pin's stub is entered from the side as readily as straight on (whole_ends._pin_stub, the
front's reading of such an end in whole_ends._front).

  python3 tests/test_622_pin_front.py

The ends model prices a single lane's end by the straight run out of it (tests/test_622_exit_front.py). A header's pin
is no such end: its barrel is on every layer and a lane comes into it from either side or at 45 degrees -- a hand
enters a row of pins along the gap between the row and whatever part stands in front of it. Read by the straight run
alone, a pin with a resistor pack two millimetres in front of it on the back layer had no clear end on that layer,
its lane took an inner one and went the long way round.

On a board of its own -- an end at the origin heading down on B.Cu, a pad of another net straight in front of it:

1. _pin_stub: a surface move of one leg that begins on a plated through-hole pad of its net is one; the same move on
   a surface pad, a move of another kind on the pin, one of two legs, and one of a net the pin is not on are none;
2. the surface pad's end with the pad in front: blocked, as any end's;
3. the through pin's end with the same pad in front: clear -- a side stands open;
4. the through pin's end with a pad in front, on both diagonals and on both sides: blocked -- no way in is left;
5. ...and with one of the five taken away again: clear.
"""
import math
import os
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)        # (the default rules: the test's distances are drawn from them)

import whole_ends as we  # noqa: E402
import braid as bd  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "LANE")
  (net 2 "VCC")
  (net 3 "OTHER")
  {parts}
  (gr_line (start -10 -10) (end 10 -10) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 10 -10) (end 10 10) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 10 10) (end -10 10) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start -10 10) (end -10 -10) (layer "Edge.Cuts") (width 0.1))
)
"""
PIN = """(footprint "PIN" (layer "F.Cu") (at 0 0)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
    (pad "1" thru_hole circle (at 0 0) (size 0.5 0.5) (drill 0.3) (layers "*.Cu" "*.Mask") (net 1 "LANE")))"""
FLAT = """(footprint "FLAT" (layer "B.Cu") (at 0 0)
    (property "Reference" "J1" (at 0 0) (layer "B.SilkS"))
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "B.Cu") (net 1 "LANE")))"""
BLOCK = """(footprint "PAD" (layer "B.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 0) (layer "B.SilkS"))
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "B.Cu") (net 2 "VCC")))"""

N, BAD = [0], []


def check(what, got, want):
    N[0] += 1
    ok = got == want
    if not ok:
        BAD.append(what)
    print(f'{"ok  " if ok else "FAIL"} {what}' + ('' if ok else f': got {got!r}, want {want!r}'))


def board(td, name, parts):
    p = os.path.join(td, f'{name}.kicad_pcb')
    with open(p, 'w') as f:
        f.write(BOARD.format(parts='\n  '.join(parts)))
    pcb = parse_kicad_pcb(p)
    if len([q for f_ in pcb.footprints.values() for q in f_.pads]) != len(parts):
        raise SystemExit(f'BROKEN TEST: the board {name} parsed without one of its {len(parts)} pads')
    return pcb


END = dict(exit_pt=(0.0, 0.0), direction='down', layer='B.Cu')
stub = NS(kind='surface', legs=[((0.0, 0.0), (0.0, 0.0))], **END)

run = bd.CLEAR + bd.TRACK / 2                          # a track's centre from a pad's edge
far = 1.0                                              # the blocks' centres from the end: 0.5 mm pads on a ring
r2 = math.sqrt(0.5)
if far - 0.25 - run >= we.FRONT_REACH - 0.1:
    raise SystemExit(f'BROKEN TEST: a block ({far - 0.25:.2f}) stands past the front\'s reach ({we.FRONT_REACH})')
if far * r2 - 0.25 * math.sqrt(2) < run + we._pairs.corner_buffer(bd.GRID) + 0.05:
    raise SystemExit('BROKEN TEST: a block stands in the run beside it -- one taken away would open no way')
ahead = BLOCK.format(x=0, y=round(far, 4), ref='C1')
round_ = [BLOCK.format(x=round(x, 4), y=round(y, 4), ref=f'C{i + 2}')
          for i, (x, y) in enumerate(((-far * r2, far * r2), (far * r2, far * r2), (-far, 0.0), (far, 0.0)))]

with tempfile.TemporaryDirectory() as td:
    pin = board(td, 'pin', [PIN, ahead])
    flat = board(td, 'flat', [FLAT, ahead])
    # ---- 1. which ends are a through pin's stub
    check('1. a surface move of one leg from a through pin of its net: a pin\'s stub', we._pin_stub(pin, 1, stub), True)
    check('1. the same move from a surface pad: none', we._pin_stub(flat, 1, stub), False)
    check('1. a move of another kind from the pin: none',
          we._pin_stub(pin, 1, NS(kind='via', legs=stub.legs, **END)), False)
    check('1. a move of two legs: none',
          we._pin_stub(pin, 1, NS(kind='surface', legs=stub.legs * 2, **END)), False)
    check('1. a move of a net the pin is not on: none', we._pin_stub(pin, 3, stub), False)
    # ---- 2, 3. a pad straight in front: a wall to a surface pad's end, none to a pin's
    check('2. a surface pad\'s end, a pad in front of it: blocked', we.exit_front(flat, 1, stub, set()) > 0, True)
    check('3. a through pin\'s end, the same pad in front: clear', we.exit_front(pin, 1, stub, set()), 0)
    # ---- 4, 5. every way in closed, and one opened again
    check('4. a through pin\'s end, a pad in front, on both diagonals and both sides: blocked',
          we.exit_front(board(td, 'ring', [PIN, ahead] + round_), 1, stub, set()) > 0, True)
    for i, left in enumerate(round_):
        rest = [b for b in round_ if b is not left]
        check(f'5. ...one of the four taken away ({("left diagonal", "right diagonal", "left side", "right side")[i]}): '
              f'clear', we.exit_front(board(td, f'open{i}', [PIN, ahead] + rest), 1, stub, set()), 0)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
