#!/usr/bin/env python3
"""A human board in KiCad 10's own layout read by the human-ends bench (awx/human_ends_bench.py).

  python3 tests/test_622_human_ends_kicad10.py

KiCad 10 writes a top-level block on a line of its own indented by a TAB, each field on a line below, and names a
track's net (`(net "name")`); the bench's readers took only the older layout (two spaces, one line, `(net N)`). On
the zynq_ad9364 board they stripped none of its 8174 copper items and read none of its 110 track arcs, and the bench
kept the human's whole route under the stubs it clipped.

1. strip_blocks removes every top-level segment, via and arc in either layout, and nothing nested (a footprint's
   fp_arc, a pad), and leaves the rest of the text as it was;
2. read_arcs reads an arc in either layout, its net by number or by name, as two chords through its middle point;
3. an arc whose net name the board does not know is skipped, not given another net.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(1, os.path.join(ROOT, 'py_router'))
import human_ends_bench as heb  # noqa: E402

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


NEW = '''(kicad_pcb
\t(version 20250114)
\t(footprint "R_0402"
\t\t(at 10 10)
\t\t(fp_arc
\t\t\t(start 0 0)
\t\t\t(mid 1 1)
\t\t\t(end 2 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at 0 0)
\t\t)
\t)
\t(segment
\t\t(start 1 2)
\t\t(end 3 4)
\t\t(width 0.2)
\t\t(layer "F.Cu")
\t\t(net "SIG_A")
\t\t(uuid "a")
\t)
\t(via
\t\t(at 5 6)
\t\t(size 0.45)
\t\t(drill 0.2)
\t\t(layers "F.Cu" "B.Cu")
\t\t(net "SIG_A")
\t)
\t(arc
\t\t(start 0 0)
\t\t(mid 1 1)
\t\t(end 2 0)
\t\t(width 0.15)
\t\t(layer "B.Cu")
\t\t(net "SIG_B")
\t\t(uuid "b")
\t)
\t(arc
\t\t(start 7 7)
\t\t(mid 8 8)
\t\t(end 9 7)
\t\t(width 0.15)
\t\t(layer "B.Cu")
\t\t(net "NOT_ON_BOARD")
\t)
\t(gr_line
\t\t(start 0 0)
\t\t(end 1 1)
\t)
)
'''

OLD = '''(kicad_pcb (version 20221018)
  (footprint "R_0402" (at 10 10)
    (fp_arc (start 0 0) (mid 1 1) (end 2 0))
  )
  (segment (start 1 2) (end 3 4) (width 0.2) (layer "F.Cu") (net 1) (tstamp a))
  (via (at 5 6) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1))
  (arc (start 0 0) (mid 1 1) (end 2 0) (width 0.15) (layer "B.Cu") (net 2) (tstamp b))
  (gr_line (start 0 0) (end 1 1))
)
'''


class Net:
    def __init__(self, name):
        self.name = name


class Pcb:
    def __init__(self):
        self.nets = {1: Net('SIG_A'), 2: Net('SIG_B')}
        self.segments = []


def main():
    import tempfile
    for tag, txt, want in (('KiCad 10', NEW, 4), ('older', OLD, 3)):
        out, n = heb.strip_blocks(txt)
        check(n == want, f'{tag}: {want} top-level copper blocks stripped (got {n})')
        check('(segment' not in out and '(via' not in out and '\n\t(arc' not in out and '\n  (arc' not in out,
              f'{tag}: no segment, via or top-level arc left')
        check('fp_arc' in out and '(footprint' in out and '(gr_line' in out and out.rstrip().endswith(')'),
              f'{tag}: the footprint (its fp_arc too), the graphic and the closing paren kept')
        check(out.count('(') == out.count(')'), f'{tag}: the text left balanced')
        with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
            f.write(txt)
        p = Pcb()
        try:
            na = heb.read_arcs(p, f.name)
        finally:
            os.unlink(f.name)
        check(na == 1, f'{tag}: one arc read (got {na}){" -- the unknown net skipped" if tag == "KiCad 10" else ""}')
        segs = [(s.start_x, s.start_y, s.end_x, s.end_y, s.layer, s.net_id, s.width) for s in p.segments]
        check(segs == [(0.0, 0.0, 1.0, 1.0, 'B.Cu', 2, 0.15), (1.0, 1.0, 2.0, 0.0, 'B.Cu', 2, 0.15)],
              f'{tag}: two chords through its middle, on its layer and net (got {segs})')
    print('FAILED: ' + '; '.join(BAD) if BAD else 'PASSED')
    sys.exit(1 if BAD else 0)


if __name__ == '__main__':
    main()
