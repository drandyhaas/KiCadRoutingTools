#!/usr/bin/env python3
"""A strip never takes a locked block for an unlocked target.

The file strips (remove_segments_from_content / remove_vias_from_content)
matched a block by span, layer and net (vias: position and net) and ignored
`(locked yes)`. A locked track with an unlocked copy stacked on it shares that
key, so when an end-of-run pass chose the unlocked COPY for removal (the
strict collapse #1063 spares locked copper and picks the copy), the strip
deleted the first block of the key in the file: the locked original. The
board shipped with the user's locked track gone and the unlocked copy in its
place. The lock is now part of the strip key.

Checks:
  A. remove_segments_from_content: an unlocked target over a locked block
     followed by its unlocked twin removes the twin and keeps the locked
     block; a locked target still removes the locked block
  B. remove_vias_from_content: the same for a stacked via pair
  C. route.py on a board whose net is locked and carries one unlocked twin
     keeps every locked item

    python3 tests/test_strip_locked_twin.py
"""
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, tool                    # noqa: E402
from synth import RoutingBoard                          # noqa: E402
from kicad_parser import Segment, Via, parse_kicad_pcb  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def _seg_text():
    return ('(kicad_pcb (version 20241229)\n  (net 0 "")\n  (net 1 "N")\n'
            '  (segment (start 5 5) (end 10 5) (width 0.2) '
            '(layer "F.Cu") (net 1) (locked yes) (uuid "pinned"))\n'
            '  (segment (start 5 5) (end 10 5) (width 0.2) '
            '(layer "F.Cu") (net 1) (uuid "copy"))\n)\n')


def _via_text():
    return ('(kicad_pcb (version 20241229)\n  (net 0 "")\n  (net 1 "N")\n'
            '  (via (at 10 5) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
            '(locked yes) (net 1) (uuid "pinned"))\n'
            '  (via (at 10 5) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
            '(net 1) (uuid "copy"))\n)\n')


def check_segments():
    print('A. a segment strip keeps the locked member of a stack')
    from kicad_writer import remove_segments_from_content
    for mw in (False, True):
        copy = Segment(5, 5, 10, 5, 0.2, 'F.Cu', 1)
        out, n = remove_segments_from_content(_seg_text(), [copy],
                                              match_width=mw)
        check(f'unlocked target removes the copy (match_width={mw})',
              n == 1 and '"pinned"' in out and '"copy"' not in out)
    pinned = Segment(5, 5, 10, 5, 0.2, 'F.Cu', 1)
    pinned.locked = True
    out, n = remove_segments_from_content(_seg_text(), [pinned])
    check('locked target removes the locked block',
          n == 1 and '"pinned"' not in out and '"copy"' in out)
    alone = _seg_text().replace(
        '  (segment (start 5 5) (end 10 5) (width 0.2) '
        '(layer "F.Cu") (net 1) (uuid "copy"))\n', '')
    left = []
    out, n = remove_segments_from_content(alone, [Segment(5, 5, 10, 5, 0.2,
                                                          'F.Cu', 1)],
                                          unmatched_out=left)
    check('unlocked target never falls back to a lone locked block',
          n == 0 and '"pinned"' in out and len(left) == 1)


def check_vias():
    print('B. a via strip keeps the locked member of a stack')
    from kicad_writer import remove_vias_from_content
    copy = Via(10, 5, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1)
    out, n = remove_vias_from_content(_via_text(), [copy])
    check('unlocked target removes the copy',
          n == 1 and '"pinned"' in out and '"copy"' not in out)
    pinned = Via(10, 5, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1)
    pinned.locked = True
    out, n = remove_vias_from_content(_via_text(), [pinned])
    check('locked target removes the locked block',
          n == 1 and '"pinned"' not in out and '"copy"' in out)


def _count(path):
    pcb = parse_kicad_pcb(path)
    items = [s for s in pcb.segments if not getattr(s, 'graphic', False)]
    items += list(pcb.vias)
    return len(items), sum(1 for i in items if i.locked)


def check_cli(tmp):
    print('C. route.py keeps every locked item over an unlocked twin')
    b = RoutingBoard(w=30, h=10)
    b.pad('J1', 5, 5, 'N')
    b.pad('J2', 20, 5, 'N')
    b.seg(5, 5, 12, 5, 'N', w=0.2, locked=True)
    b.seg(12, 5, 20, 5, 'N', w=0.2, locked=True)
    b.seg(5, 5, 12, 5, 'N', w=0.2, uid='unlocked-twin')
    src = os.path.join(tmp, 'locked.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(b.text())
    n_in, l_in = _count(src)
    out = os.path.join(tmp, 'locked_out.kicad_pcb')
    r = subprocess.run([sys.executable, tool('route.py'), src, out, '--nets', 'N',
                        '--grid-step', '0.1', '--track-width', '0.2',
                        '--clearance', '0.1', '--layers', 'F.Cu', 'B.Cu'],
                       capture_output=True, text=True, timeout=600)
    check('route.py exits 0', r.returncode == 0, r.stderr[-400:])
    n_out, l_out = _count(evidence(out))
    check('every locked item kept', l_out == l_in, f'{l_in} -> {l_out}')
    check('the unlocked twin is the one removed', n_out == l_in,
          f'{n_in} -> {n_out}')
    with open(out, encoding='utf-8') as f:
        check('the twin uuid is gone', not re.search(r'unlocked-twin', f.read()))


def main():
    with tempfile.TemporaryDirectory() as tmp:
        check_segments()
        check_vias()
        check_cli(tmp)
    if FAILS:
        print(f'FAIL: {len(FAILS)} check(s): {FAILS}')
        sys.exit(1)
    print('PASS: strip keeps locked twin')


if __name__ == '__main__':
    main()
