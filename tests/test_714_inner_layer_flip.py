#!/usr/bin/env python3
"""A flipped footprint's inner-layer items mirror through the stack (#714).

pcbnew's own flip, since 10.0.6, sends an item on `In<k>.Cu` of a board with n
inner copper layers to `In<n+1-k>.Cu` (10.0.0 left it in place). The KiCad
gate `tests/gui_parity/test_714_mirror_pcbnew_parity.py` compares the writer
against pcbnew itself; this is its wx-free half, which `run_all.py` collects:

  1. `flip_layer_token`: faces swap; with the count, inner copper mirrors;
     without it, an inner layer passes through;
  2. the writer's block flip: a footprint text and a footprint zone on inner
     copper land where pcbnew puts them, the zone's layer list in layer-id
     order, and an inner-layer item is REFUSED when no count was given;
  3. `board_inner_copper_count` reads the board's `(layers ...)` table.

    python3 tests/test_714_inner_layer_flip.py
"""
import os
import re
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path[:0] = [os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_placer')]

from kicad_parser import flip_layer_token  # noqa: E402
from placement import writer  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


BLOCK = '''(footprint "t:X"
\t\t(layer "F.Cu")
\t\t(at 10 20)
\t\t(fp_text user "IN" (at 0 1 0) (layer "In1.Cu") (effects (font (size 1 1) (thickness 0.15))))
\t\t(zone (net 0) (net_name "") (layers "F.Cu" "B.Cu" "In1.Cu" "In2.Cu") (hatch edge 0.5)
\t\t\t(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
\t\t\t(polygon (pts (xy 9 19) (xy 11 19) (xy 11 21) (xy 9 21))))
\t)'''


def main():
    print('1. the token')
    check('faces swap', flip_layer_token('F.Cu') == 'B.Cu'
          and flip_layer_token('B.SilkS') == 'F.SilkS')
    try:
        four = [flip_layer_token(f'In{k}.Cu', 4) for k in (1, 2, 3, 4)]
        two = flip_layer_token('In1.Cu', 2)
    except TypeError as ex:
        four, two = str(ex), None
    check('inner copper mirrors through four inner layers',
          four == ['In4.Cu', 'In3.Cu', 'In2.Cu', 'In1.Cu'], str(four))
    check('...and through two', two == 'In2.Cu', str(two))
    check('no count: an inner layer passes through',
          flip_layer_token('In1.Cu') == 'In1.Cu')

    print('2. the block flip')
    flip = getattr(writer, '_flip_footprint_block')
    try:
        out = flip(BLOCK, 'X', 0.0, 0.0, old_y='20', inner_count=4)
    except TypeError as ex:
        check('the block flip takes the inner-layer count', False, str(ex))
        out = ''
    text = re.search(r'\(fp_text user "IN".*?\(layer "([^"]+)"\)', out)
    check('the text on In1.Cu lands on In4.Cu', bool(text) and text.group(1) == 'In4.Cu',
          text.group(1) if text else out[:200])
    zl = re.search(r'\(zone.*?\(layers ([^)]*)\)', out, re.S)
    check('the zone on F/B/In1/In2 is on F/B/In3/In4, in layer-id order',
          bool(zl) and zl.group(1) == '"F.Cu" "B.Cu" "In3.Cu" "In4.Cu"',
          zl.group(1) if zl else '')
    try:
        flip(BLOCK, 'X', 0.0, 0.0, old_y='20')
        refused = False
    except writer.SideFlipUnsupported:
        refused = True
    check('no count: an inner-layer item is refused, not passed through', refused)

    print('3. the count off the layer table')
    count = getattr(writer, 'board_inner_copper_count', None)
    six = ('(kicad_pcb\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(4 "In1.Cu" signal)\n'
           '\t\t(6 "In2.Cu" power "GND")\n\t\t(8 "In3.Cu" signal)\n\t\t(10 "In4.Cu" signal)\n'
           '\t\t(2 "B.Cu" signal)\n\t\t(5 "F.SilkS" user)\n\t)\n)\n')
    check('a six-layer table: 4 inner', count is not None and count(six) == 4,
          str(count(six) if count else None))

    print('FAILED: ' + ', '.join(failures) if failures else 'ALL PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
