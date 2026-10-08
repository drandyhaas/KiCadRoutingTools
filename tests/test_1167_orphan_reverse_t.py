#!/usr/bin/env python3
"""check_orphan_stubs credits copper overlap against tracks, and a reverse T
(#1167).

sonde_xilinx shipped a VCC tap on B.Cu (all 0.635 mm): trunk
(138.2,76.4)-(138.2,91.9)-(139.5,93.2), branch (137.654545,91.854545)-
(141.5,95.7). The branch's free end overhangs the trunk, and its body overlaps
the trunk diagonal for 1.84 mm. KiCad, check_connected and check_weird call it
connected; check_orphan_stubs called it an orphan (its track test dropped the
stub's own half-width, and it had no reverse-T test), and check_complete,
which gates on it, said INCOMPLETE.

Rows: that shape is not an orphan and check_weird agrees; #601's real 0.316 mm
gap between two 0.127 mm stubs is still an orphan; a reverse-T anchor with a
LONG tail past it is still an orphan, as check_weird reports it.

    python3 tests/test_1167_orphan_reverse_t.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from check_orphan_stubs import find_orphan_stubs    # noqa: E402

FAILS = []

HEAD = """(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(25 "Edge.Cuts" user)
\t)
\t(setup (pad_to_mask_clearance 0))
\t(net 0 "")
\t(net 1 "VCC")
"""
PAD = """\t(footprint "t:P" (layer "F.Cu") (uuid "aaaaaaaa-0000-0000-0000-0000000{i:05d}")
\t\t(at {x} {y})
\t\t(pad "1" thru_hole circle (at 0 0) (size 1.2 1.2) (drill 0.6)
\t\t\t(layers "*.Cu" "*.Mask") (net 1 "VCC")
\t\t\t(uuid "cccccccc-0000-0000-0000-0000000{i:05d}"))
\t)
"""
SEG = """\t(segment (start {a} {b}) (end {c} {d}) (width {w}) (layer "B.Cu")
\t\t(net 1) (uuid "bbbbbbbb-0000-0000-0000-0000000{i:05d}"))
"""


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def board(segs, pads):
    d = tempfile.mkdtemp(prefix='t1167_')
    p = os.path.join(d, 'b.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as f:
        f.write(HEAD + ''.join(PAD.format(i=i, x=x, y=y) for i, (x, y) in enumerate(pads))
                + ''.join(SEG.format(i=i, a=a, b=b, c=c, d=dd, w=w)
                          for i, (a, b, c, dd, w) in enumerate(segs)) + ')\n')
    return p


def orphans(p):
    return sorted(pt for layers in find_orphan_stubs(p).values()
                  for pts in layers.values() for pt in pts)


def weird_dangles(p):
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_weird.py'), p],
                       capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if 'dangling' in ln.lower()
            and '(' in ln], r.stdout


W = 0.635
SONDE = [(138.2, 76.4, 138.2, 91.9, W), (138.2, 91.9, 139.5, 93.2, W),
         (137.654545, 91.854545, 141.5, 95.7, W)]
p = board(SONDE, [(138.2, 76.4), (139.5, 93.2), (141.5, 95.7)])
o = orphans(p)
check('sonde_xilinx VCC tap: no orphan', o == [], f'{o}')
d, out = weird_dangles(p)
check('...and check_weird agrees (control on the fixture)', d == [] and 'NO WEIRD' in out, f'{d}')

# #601 bancouver40 col9: a 0.127 fanout stub tip and the route start 0.316 mm
# apart, no via or pad at either end -- a real open.
G = [(113.5, 41.0, 113.5, 38.7, 0.127), (113.4, 38.4, 110.0, 38.4, 0.127)]
p = board(G, [(113.5, 41.0), (110.0, 38.4)])
o = orphans(p)
check('#601 real gap: both ends still orphans', len(o) == 2, f'{o}')

# A reverse-T anchor with a long tail past it: the trunk vertex lands on the
# branch body 4 mm from the branch's free end -- a dead tail, as check_weird
# reports it.
LONG = [(10.0, 0.0, 10.0, 10.0, W), (6.0, 10.0, 20.0, 10.0, W)]
p = board(LONG, [(10.0, 0.0), (20.0, 10.0)])
o = orphans(p)
check('a long tail past a reverse-T anchor is still an orphan', o == [(6.0, 10.0)], f'{o}')
d, _ = weird_dangles(p)
check('...and check_weird reports it too', len(d) == 1, f'{d}')
# The same shape with a sub-visible nib (0.4 mm) past the anchor is not.
NIB = [(10.0, 0.0, 10.0, 10.0, W), (9.6, 10.0, 20.0, 10.0, W)]
p = board(NIB, [(10.0, 0.0), (20.0, 10.0)])
check('a nib past a reverse-T anchor is not an orphan', orphans(p) == [], f'{orphans(p)}')

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
