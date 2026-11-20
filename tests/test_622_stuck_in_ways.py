#!/usr/bin/env python3
"""A part the nudge could not move clear of the lanes' WAYS OUT is a finding of the laid field (awx/group_ends.py:
stuck_in_ways).

  python3 tests/test_622_stuck_in_ways.py

The group's field is laid, and the passives beneath the arrays are nudged off it -- off the fanout's copper, and off
each lane's WAY OUT, its exit's straight run on to where the lane can turn, laid for the nudge alone and taken up
again. A part the nudge cannot move clear says so ("Stuck", "Unresolved") and is left where it best stands. Left in
a lane's way out it breaks no rule of the board's -- the way out is no copper of the field's -- so the field's check
is clean, nothing was taught, the lane was planned through the part, and the loop stopped with its ends crowded. Such
a part is a finding now: each lane whose way out one of its pads stands in, a row, and the model is taught that
lane's end as it is by a violation.

1. a part the nudge names as stuck, a pad of it in a lane's way out on that pad's layer: one finding, the lane's net;
2. ...none for a way out it stands clear of, nor for one on another layer than its pads';
3. ...none for the pad's own net's way out;
4. a part the nudge does not name gives none, whatever it stands in; nor does any with no way out given.
"""
import contextlib
import io
import os
import sys
import tempfile

try:
    from ortools.sat.python import cp_model  # noqa: F401  (group_ends imports its models' modules)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)
import group_ends as ge  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


# (a capacitor on the back, its pads at (10.0, 10) and (11.0, 10), 0.5 x 0.6: nets VCC and GND; three lanes' nets)
BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "/A")
  (net 2 "/B")
  (net 3 "VCC")
  (net 4 "GND")
  (footprint "t:C" (layer "B.Cu") (at 10.5 10)
    (property "Reference" "C1" (at 0 -2) (layer "B.SilkS"))
    (pad "1" smd rect (at -0.5 0) (size 0.5 0.6) (layers "B.Cu" "B.Mask") (net 3 "VCC"))
    (pad "2" smd rect (at 0.5 0) (size 0.5 0.6) (layers "B.Cu" "B.Mask") (net 4 "GND"))
  )
  (footprint "t:C" (layer "B.Cu") (at 20.5 10)
    (property "Reference" "C2" (at 0 -2) (layer "B.SilkS"))
    (pad "1" smd rect (at -0.5 0) (size 0.5 0.6) (layers "B.Cu" "B.Mask") (net 3 "VCC"))
    (pad "2" smd rect (at 0.5 0) (size 0.5 0.6) (layers "B.Cu" "B.Mask") (net 4 "GND"))
  )
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts"))
)
"""
STUCK = ['Stuck with 1 cap(s) still grazing foreign copper (at the displacement cap): C1',
         'Moved 3 cap(s); resolved 2/3 initial violations; 1 unresolved.', 'Unresolved (need manual attention): C1']

with tempfile.TemporaryDirectory() as td:
    field = os.path.join(td, 'f.kicad_pcb')
    open(field, 'w').write(BOARD)

    def found(caps, ways):
        with contextlib.redirect_stdout(io.StringIO()):
            return [(i['net1'], i['layer'], i['pad_ref'], i['type']) for i in ge.stuck_in_ways(field, caps, ways)]
    # (a way out: (the net's id, its exit, its direction, its layer) -- down from y = 9.4 runs through y = 10's pads)
    through = (1, (10.0, 9.4), 'down', 'B.Cu')          # /A, straight through C1.1
    clear = (2, (12.5, 9.4), 'down', 'B.Cu')            # /B, a millimetre past C1.2
    front = (2, (10.0, 9.4), 'down', 'F.Cu')            # /B over C1.1, on the other layer
    own = (3, (10.0, 9.4), 'down', 'B.Cu')              # VCC's own way out over its own pad
    at_c2 = (1, (20.0, 9.4), 'down', 'B.Cu')            # /A through C2.1, a part the nudge did not name
    check('1. a stuck part\'s pad in a lane\'s way out: that lane\'s finding',
          found(STUCK, [through]) == [('/A', 'B.Cu', 'C1.1', 'way-out')], found(STUCK, [through]))
    check('2. ...none for a way out it stands clear of', found(STUCK, [clear]) == [], found(STUCK, [clear]))
    check('2. ...nor on another layer than its pads\'', found(STUCK, [front]) == [], found(STUCK, [front]))
    check('3. ...nor its own net\'s', found(STUCK, [own]) == [], found(STUCK, [own]))
    check('4. a part the nudge does not name: none', found(STUCK, [at_c2]) == [], found(STUCK, [at_c2]))
    check('4. ...no part named: none', found(['Moved 3 cap(s); resolved 3/3 initial violations; 0 unresolved.'],
                                             [through]) == [])
    check('4. ...no way out given: none', found(STUCK, []) == [])

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
