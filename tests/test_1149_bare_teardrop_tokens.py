#!/usr/bin/env python3
"""#1149: a teardrop child written without its ``(`` is read the way KiCad
reads it, so it no longer ends the pad, the footprint or the via early.

KiCad 10.0.0's RoyalBlue54L-Feather demo carries 349 blocks like
``(curved_edges no)filter_ratio 0.9)``. KiCad's teardrop reader takes each
child's opening paren as optional, so it loads that as ``(filter_ratio 0.9)``
and the ``)`` that looks surplus is the child's own close. Every
paren-counting reader here took it as the pad's close: the text parser ended
U2 (QFN-32), U4 and U6 after their first pad (pcbnew: 59/10/21), and a via
whose (net ...) follows its teardrops lost the net and was dropped.

Checks:
  1. repair_bare_teardrop_tokens inserts exactly KiCad's paren, for a bare
     key after `)` and after whitespace, and leaves a non-key bare word, a
     clean file (the same string object) and a quoted lookalike alone.
  2. parse_kicad_pcb on a board with the defect in every pad and a via: all
     pads on the footprint, the via kept with its net.
  3. write_placed_output rotating that footprint rotates every pad, not just
     the first.

    python3 tests/test_1149_bare_teardrop_tokens.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb, repair_bare_teardrop_tokens  # noqa: E402
from placement.writer import write_placed_output                       # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


TD_BAD = ('(teardrops (best_length_ratio 0.5) (max_length 1) (best_width_ratio 1) '
          '(max_width 2) (curved_edges no)filter_ratio 0.9) (enabled yes) '
          '(allow_two_segments yes) (prefer_zone_connections yes))')
TD_GOOD = TD_BAD.replace(')filter_ratio', ')(filter_ratio')


def pad(num, x, net):
    return (f'(pad "{num}" smd rect (at {x} 0 0) (size 0.5 0.8) (layers "F.Cu" "F.Mask") '
            f'(net "{net}") {TD_BAD} (uuid "p{num}"))')


BOARD = f'''(kicad_pcb (version 20241229) (generator "pcbnew")
 (general (thickness 1.6))
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
 (net 0 "") (net 1 "A") (net 2 "B") (net 3 "C")
 (footprint "lib:QFN" (layer "F.Cu") (uuid "fp1") (at 10 10 0)
  (property "Reference" "U2" (at 0 -2 0) (layer "F.SilkS") (uuid "r1"))
  {pad(1, -1, "A")}
  {pad(2, 0, "B")}
  {pad(3, 1, "C")}
 )
 (via (at 20 20) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") {TD_BAD} (net "B") (uuid "v1"))
 (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "e1"))
)
'''

print("1. the repair")
fixed, n = repair_bare_teardrop_tokens(TD_BAD)
check("bare key after ')' gets KiCad's paren", (fixed, n) == (TD_GOOD, 1), fixed)
ws = TD_BAD.replace(')filter_ratio', ') filter_ratio')
fixed, n = repair_bare_teardrop_tokens(ws)
check("bare key after whitespace too", n == 1 and fixed.count('(') == fixed.count(')'))
other = TD_GOOD.replace('(enabled yes)', 'mystery 3)')
fixed, n = repair_bare_teardrop_tokens(other)
check("a bare word KiCad does not know is left alone", (fixed, n) == (other, 0))
fixed, n = repair_bare_teardrop_tokens(TD_GOOD)
check("a clean block is the same string", n == 0 and fixed is TD_GOOD)
quoted = '(property "x" "(teardrops )filter_ratio 1)")'
check("no teardrop block, nothing to repair", repair_bare_teardrop_tokens(quoted)[1] == 0)

tmp = tempfile.mkdtemp(prefix='t1149_')
try:
    src = os.path.join(tmp, 'in.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(BOARD)

    print("2. the parse")
    err = io.StringIO()
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(err):
        pcb = parse_kicad_pcb(src)
    pads = pcb.footprints['U2'].pads
    check("U2 has all three pads", len(pads) == 3, f"{len(pads)} pads")
    check("the via is kept with its net",
          len(pcb.vias) == 1 and pcb.net_id_to_name.get(pcb.vias[0].net_id) == 'B',
          f"{[(v.net_id) for v in pcb.vias]}")
    check("the read says what it repaired", '4 teardrop token(s)' in err.getvalue(),
          err.getvalue().strip()[:100])

    print("3. the placement writer")
    out = os.path.join(tmp, 'out.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        write_placed_output(src, out, [{'reference': 'U2', 'new_x': 10, 'new_y': 10,
                                        'new_rotation': 90}], pcb_data=pcb)
    text = open(out, encoding='utf-8').read()
    angles = re.findall(r'\(pad "\d" smd rect \(at [\d.-]+ [\d.-]+ ([\d.-]+)\)', text)
    check("every pad carries the new angle", angles == ['90', '90', '90'], f"{angles}")
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        again = parse_kicad_pcb(out)
    check("the output re-parses whole", len(again.footprints['U2'].pads) == 3
          and len(again.vias) == 1)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")
