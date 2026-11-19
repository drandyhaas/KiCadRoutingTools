#!/usr/bin/env python3
"""A crowded end's re-escape measures its new tooth against EVERY part of a group's destination (awx/ends_reescape.py,
its three source_realize.measure_tooth calls; braid.endpoints underneath).

  python3 tests/test_622_reescape_group_dest.py

A bus whose destination is a group of parts ('D1+D2') has lanes ending at its second part. Once a re-escape strips
such a lane and lays its source escape again, the lane's copper is two stubs, each with a free end: the tooth at the
source and the berth at D2. Told the destination is D1 alone, the end attribution reads neither free end as the
destination's, and takes the one farther from its own pad or via for the tooth -- the berth's stub where that one is
the longer. The re-escape then wrote the berth into the plan sidecar as the lane's tooth, the whole route's geometry
drew the lane with no length, and the loop on the repaired ends died on it.

1. braid.endpoints, the destination given whole: the tooth is the source stub's free end, the berth the D2 stub's;
2. ...given its first part alone: the D2 stub's end (the longer stub) is taken for the tooth -- the defect, kept as the
   reason the destination must be given whole;
3. ends_reescape gives every measure_tooth call the group's whole destination, never plan_state's first part.
"""
import ast
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    import braid  # noqa: E402
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


def pad(ref, x, y):
    return NS(component_ref=ref, global_x=x, global_y=y, size_x=0.3, size_y=0.3, rect_rotation=0.0, shape='rect',
              layers=['F.Cu'], drill=0.0, pad_type='smd', net_id=7)


def seg(ax, ay, bx, by, layer='F.Cu'):
    return NS(start_x=ax, start_y=ay, end_x=bx, end_y=by, layer=layer, width=0.09, net_id=7)


# the source ball S at (0, 0): its new escape a via-free run of 1.6 mm; the berth at D2's pad (10, 0): a stub of
# 1.7 mm back toward the source. D1, the group's first part, stands elsewhere and has no pad of this net.
PADS = [pad('S', 0.0, 0.0), pad('D2', 10.0, 0.0)]
net = NS(pads=PADS)
pcb = NS(segments=[seg(0.0, 0.0, 1.6, 0.0), seg(10.0, 0.0, 8.3, 0.0)], vias=[])
byname = {'LANE': (7, net)}

whole = braid.endpoints(pcb, ['LANE'], byname, dest_ref='D1+D2')['LANE']
check('1. the group given whole: the tooth the source stub\'s free end', tuple(round(v, 3) for v in whole[0]),
      (1.6, 0.0))
check('1. ...and the berth the D2 stub\'s, at D2', (tuple(round(v, 3) for v in whole[1]), whole[2]),
      ((8.3, 0.0), 'D2'))
first = braid.endpoints(pcb, ['LANE'], byname, dest_ref='D1')['LANE']
check('2. its first part alone: the berth\'s stub end taken for the tooth', tuple(round(v, 3) for v in first[0]),
      (8.3, 0.0))

src = open(os.path.join(ROOT, 'awx', 'ends_reescape.py')).read()
calls = [n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)
         and getattr(n.func, 'attr', getattr(n.func, 'id', None)) == 'measure_tooth']
dest_args = [ast.get_source_segment(src, k.value) for c in calls for k in c.keywords if k.arg == 'dest_ref']
check('3. ends_reescape: every measure_tooth call is given the destination', len(dest_args) == len(calls) > 0, True)
check('3. ...the group\'s whole destination, never plan_state\'s first part',
      sorted(set(dest_args)), ['dest_all'])
check('3. ...read off plan_state\'s parts of the group',
      "dest_all = '+'.join(st['dests']) if len(st.get('dests') or ()) > 1 else st['dref']" in src, True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
