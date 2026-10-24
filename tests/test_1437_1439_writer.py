#!/usr/bin/env python3
"""
Regression tests for two kicad_writer text-handling bugs:

  #1437: move_copper_text_to_silkscreen counted parentheses INSIDE gr_text
         strings, so "note :(" swallowed the items after it and moved their
         copper (layer ...) to silkscreen.
  #1439: swap_segment_nets_at_positions did not match a locked segment
         (`(locked yes)` / `(locked)` child, or the bare `(segment locked`
         head), so locked copper kept its old net.

Run:
    python3 tests/test_1437_1439_writer.py
"""

import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from kicad_writer import (move_copper_text_to_silkscreen,
                          swap_segment_nets_at_positions, pos_key)


def main():
    fails = []

    def check(name, ok):
        print(("  PASS " if ok else "  FAIL ") + name)
        if not ok:
            fails.append(name)

    # --- #1437 -------------------------------------------------------------
    out = move_copper_text_to_silkscreen('''(kicad_pcb
  (gr_text "note :(" (at 1 1) (layer "F.Cu") (uuid "a"))
  (segment (start 0 0) (end 1 1) (width 0.2) (layer "F.Cu") (net 1) (uuid "b"))
  (gr_text "ok :)" (at 1 1) (layer "B.Cu") (uuid "c"))
  (gr_text "esc \\"(" (at 1 1) (layer "F.Cu") (uuid "d"))
  (via (at 3 3) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "e"))
)''')
    lines = {m.group(1): m.group(0) for m in
             re.finditer(r'.*\(uuid "(\w)"\).*', out)}
    check('#1437 gr_text "note :(" moved to F.SilkS', 'F.SilkS' in lines['a'])
    check('#1437 segment after it stays on F.Cu',
          '(layer "F.Cu")' in lines['b'])
    check('#1437 gr_text "ok :)" moved to B.SilkS', 'B.SilkS' in lines['c'])
    check('#1437 escaped-quote gr_text moved to F.SilkS',
          'F.SilkS' in lines['d'])
    check('#1437 via after it keeps its copper layers',
          '(layers "F.Cu" "B.Cu")' in lines['e'])

    # --- #1439 -------------------------------------------------------------
    pos = {pos_key(1, 1)}
    child = ('(kicad_pcb\n\t(segment{head}\n\t\t(start 1 1)\n\t\t(end 2 2)\n'
             '\t\t(width 0.2){lock}\n\t\t(layer "F.Cu")\n\t\t{net}\n'
             '\t\t(uuid "u")\n\t)\n)\n')
    for head, lock in (('', ''), ('', '\n\t\t(locked yes)'),
                       ('', '\n\t\t(locked)'), (' locked', '')):
        tag = f"head={head!r} lock={lock.strip()!r}"
        new, n = swap_segment_nets_at_positions(
            child.format(head=head, lock=lock, net='(net 5)'), pos, 5, 6)
        check(f"#1439 numeric swap {tag}", n == 1 and '(net 6)' in new)
        new, n = swap_segment_nets_at_positions(
            child.format(head=head, lock=lock, net='(net "A")'), pos, 5, 6,
            old_net_name='A', new_net_name='B')
        check(f"#1439 name swap {tag}", n == 1 and '(net "B")' in new)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
