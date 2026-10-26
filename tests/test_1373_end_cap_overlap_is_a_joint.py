#!/usr/bin/env python3
"""#1373: an end whose cap overlaps another same-net track's copper is a joint.

check_weird credited same-net track copper only by an exact vertex, an interior
T, or an end-to-end gap of at most 0.1 mm, so a 0.25 mm track ending 0.33 mm
from the joint of two 0.5 mm tracks (caps overlapping 0.045 mm) read as a
dangling end -- while KiCad, check_connected and check_orphan_stubs all call
it connected, and removing the "dangling" track splits the net.

    python3 tests/test_1373_end_cap_overlap_is_a_joint.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in (TESTS_DIR, ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    sys.path.insert(0, _d)

from test_check_weird import _seg, _pad, _pcb, _cats     # noqa: E402
from check_weird import check_weird                      # noqa: E402
from check_connected import check_net_connectivity       # noqa: E402

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def dangling(segs, pads):
    f, _ = check_weird(_pcb(segs, pads=pads))
    return [x for x in f if x['category'] == 'dangling-end'], _cats(f)


# A 0.5 mm trunk bending at (17, 8); a pad at each far end; and a 0.25 mm
# track from a third pad whose end stops short of the bend, its cap overlapping
# the trunk's end cap (0.33 mm apart; half-widths sum 0.375).
PADS = [_pad(17, 5, num='1'), _pad(15, 8, num='2', ref='U2'),
        _pad(20, 8, num='3', ref='U3')]
TRUNK = [_seg(17, 5, 17, 8, width=0.5), _seg(17, 8, 15, 8, width=0.5)]

print('1: overlapping caps are a joint')
d, cats = dangling(TRUNK + [_seg(20, 8, 17.2333, 8.2333, width=0.25)], PADS)
ok('no dangling-end', not d, str(d))
ok('no soft-joint either (the partner is joined elsewhere)',
   cats['soft-joint'] == 0, str(dict(cats)))

print('2: an end that lies on a trunk body is credited too')
d, _ = dangling(TRUNK + [_seg(20, 8, 17.2, 7.4, width=0.25)], PADS)
ok('end 0.2 mm off the trunk centre line: connected', not d, str(d))

print('3: the overlap must be real')
d, _ = dangling(TRUNK + [_seg(20, 8, 17.5, 8.3, width=0.25)], PADS)
ok('end 0.39 mm from the bend, caps clear: still a dangling end',
   len(d) == 1, str(d))

print('4: a sharp fold onto the stub\'s own chain is not a joint')
pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
d, _ = dangling([_seg(0, 0, 10, 0), _seg(5, 0, 5, 3, width=0.2),
                 _seg(5, 3, 5.1, 2.5, width=0.2)], pads)
ok('fold-back stub end is still reported', len(d) >= 1, str(d))

print('5: the "dangling" track is load-bearing (the net is connected)')
pcb = _pcb(TRUNK + [_seg(20, 8, 17.2333, 8.2333, width=0.25)], pads=PADS)
try:
    res = check_net_connectivity(1, pcb.segments, pcb.vias, PADS, [])
    ok('check_net_connectivity says connected',
       not res.get('disconnected_pads') and not res.get('components', 1) > 1,
       str(res)[:200])
except Exception as e:                                   # noqa: BLE001
    ok('check_net_connectivity ran', False, repr(e))

if failures:
    print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
    sys.exit(1)
print('\nAll checks passed')
