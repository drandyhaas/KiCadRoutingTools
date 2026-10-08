#!/usr/bin/env python3
"""The exact web test sees an oval pad, and the collapse guard re-tests the
terminals a removal narrows (#1161).

`_pad_web_polygon` built an oval as ``box.buffer(-r).buffer(r)`` with r the
half short axis: the negative buffer collapsed the short axis, so EVERY oval
-- KiCad's 1.7x1.7 pin-header pad included -- was an empty polygon and
`terminal_web_neck_exact` judged oval-pad joints with the pad left out. On
ecc83 that reported a phantom narrow-pad-joint at P1.2 and missed real necks
at an oval rim. Separately, the strict collapse guard re-tested only the
removed unit's own ends, so removing an in-pad wiggle that widened a
NEIGHBOURING end's joint left that end narrow.

    python3 tests/test_1161_oval_pad_web.py
"""
import math
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

from kicad_parser import Pad, Segment                          # noqa: E402
from pcb_modification import (_pad_web_polygon, StrictRemovalModel,  # noqa: E402
                              terminal_web_neck_exact)

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def pad(shape, sx, sy, x=0.0, y=0.0, net=1, ref='P1', rratio=0.0, drill=1.0,
        layers=('*.Cu',)):
    return Pad(component_ref=ref, pad_number='2', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape=shape,
               layers=list(layers), net_id=net, net_name='/N', drill=drill,
               pad_type='thru_hole' if drill else 'smd', roundrect_rratio=rratio)


def seg(x1, y1, x2, y2, w, net=1, layer='F.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def near(a, b, rel=2e-3):
    return abs(a - b) <= rel * max(abs(a), abs(b))


print('A. the polygon is the pad')
p = _pad_web_polygon(pad('oval', 1.7, 1.7))
check('a 1.7x1.7 oval is its circle, not empty',
      p is not None and near(p.area, math.pi * 0.85 ** 2), f'area {p.area if p else None}')
p = _pad_web_polygon(pad('oval', 2.03, 3.05))
want = math.pi * 1.015 ** 2 + 2.03 * (3.05 - 2.03)
check('a 2.03x3.05 oval is its stadium', p is not None and near(p.area, want),
      f'area {p.area:.4f}, want {want:.4f}')
check('...long axis along y', near(p.bounds[3] - p.bounds[1], 3.05)
      and near(p.bounds[2] - p.bounds[0], 2.03), f'bounds {p.bounds}')
p = _pad_web_polygon(pad('roundrect', 1.0, 2.0, rratio=0.25))
want = 1.0 * 2.0 - (4.0 - math.pi) * 0.25 ** 2
check("a roundrect's radius is rratio x the FULL short side (KiCad's)",
      near(p.area, want), f'area {p.area:.4f}, want {want:.4f}')
p = _pad_web_polygon(pad('roundrect', 1.0, 2.0, rratio=0.5))
want = math.pi * 0.5 ** 2 + 1.0 * 1.0
check('a 0.5-ratio roundrect is a stadium, not empty',
      p is not None and near(p.area, want), f'area {p.area if p else None}')
check('a rect is still its box', near(_pad_web_polygon(pad('rect', 1.0, 2.0)).area, 2.0))


print('B. the exact test sees the oval')
# Missed real neck: a 0.25 track whose cap grazes a 1.7 pad's rim, the same
# geometry as a circle pad (which the test always handled).
for shape in ('circle', 'oval'):
    view = NS(segments=[seg(0.0, 0.95, 0.0, 3.0, 0.25)],
              pads_by_net={1: [pad(shape, 1.7, 1.7)]})
    got = terminal_web_neck_exact(view, 1, 'F.Cu', 0.0, 0.95, 0.25)
    check(f'a cap grazing a 1.7 {shape} pad rim is a neck', got is True, f'got {got}')
# Phantom finding (ecc83 P1.2): a 0.8636 track ending 0.66 mm from an oval
# pad's centre, another same-net track leaving from the centre. With the pad
# left out the "web" is the lens between the two caps.
for shape in ('circle', 'oval'):
    view = NS(segments=[seg(0.66, 0.0, 3.0, 0.0, 0.8636),
                        seg(0.0, 0.0, 0.0, -3.0, 0.8636)],
              pads_by_net={1: [pad(shape, 1.7, 1.7)]})
    got = terminal_web_neck_exact(view, 1, 'F.Cu', 0.66, 0.0, 0.8636)
    check(f'an end deep inside a 1.7 {shape} pad is not a neck', got is False, f'got {got}')
view = NS(segments=[seg(0.66, 0.0, 3.0, 0.0, 0.8636), seg(0.0, 0.0, 0.0, -3.0, 0.8636)],
          pads_by_net={1: []})
check('...control: without the pad the same pair IS a neck',
      terminal_web_neck_exact(view, 1, 'F.Cu', 0.66, 0.0, 0.8636) is True)

print('C. the classifier uses the same polygon')
from classify_connection_width import _pad_shape   # noqa: E402
q = pad('oval', 2.03, 3.05)
check('classify_connection_width models the oval as the router does',
      _pad_shape(q).equals(_pad_web_polygon(q)))

print('D. the collapse guard re-tests a terminal the removal narrows')
# Pad [9.5,10.5]^2. A 0.3 terminal ends at T=(9.53,10.52), a corner graze
# whose web is under the 0.25 floor on its own. An in-pad wiggle W beside it
# widens that web; W's own ends are nowhere near T's node.
P1 = pad('rect', 1.0, 1.0, x=10.0, y=10.0, drill=0.0, layers=('F.Cu',))
P2 = pad('rect', 1.0, 1.0, x=9.53, y=13.0, ref='P2', drill=0.0, layers=('F.Cu',))
term = seg(9.53, 13.0, 9.53, 10.52, 0.3)
wig = seg(9.62, 10.40, 9.9, 10.40, 0.3)
floor = 0.25
v_with = NS(segments=[term, wig], pads_by_net={1: [P1, P2]})
v_without = NS(segments=[term], pads_by_net={1: [P1, P2]})
pre_with = terminal_web_neck_exact(v_with, 1, 'F.Cu', 9.53, 10.52, floor)
pre_without = terminal_web_neck_exact(v_without, 1, 'F.Cu', 9.53, 10.52, floor)
check('fixture: the wiggle widens T past the floor', pre_with is False, f'{pre_with}')
check('fixture: without it T is a neck', pre_without is True, f'{pre_without}')
m = StrictRemovalModel(1, [term, wig], [], [P1, P2], ['F.Cu', 'B.Cu'],
                       web_floor=floor)
check('fixture: the net is gradable and the wiggle a candidate',
      m.valid and 1 in m._cand_set)
ok, E, D = m.trial(frozenset(), frozenset(), (1,))
check('removing the wiggle that keeps T wide is refused', ok is False, f'ok={ok}')
far = seg(9.62, 10.0, 9.9, 10.0, 0.3)   # in-pad, clear of T's web
m = StrictRemovalModel(1, [term, far], [], [P1, P2], ['F.Cu', 'B.Cu'],
                       web_floor=floor)
pre = terminal_web_neck_exact(NS(segments=[term, far], pads_by_net={1: [P1, P2]}),
                              1, 'F.Cu', 9.53, 10.52, floor)
ok, E, D = m.trial(frozenset(), frozenset(), (1,))
check('control: a wiggle that never widened T still goes', pre is True and ok is True,
      f'pre={pre} ok={ok}')

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
