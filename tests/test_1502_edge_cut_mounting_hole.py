"""#1502: a mounting hole drawn as an Edge.Cuts circle is a mounting hole.

`mounting_holes()` walked footprints only, so a 3.2 mm screw hole drawn as a
board cut-out (an Edge.Cuts `gr_circle`) read "No mounting holes on this
board." in check_mounting_holes and wrote no ring in add_rule_area
--mounting-holes, though the same hole as a footprint failed with 5 items.
Now a closed Edge.Cuts circle of 2.0 mm or more that is not the board's own
outline is a hole (`edge_cut_holes`, named CUT1, CUT2 ...).

    python3 tests/test_1502_edge_cut_mounting_hole.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def circle(x, y, d):
    r = d / 2.0
    return (f'  (gr_circle (center {x} {y}) (end {x + r} {y}) (stroke (width 0.05) '
            f'(type default)) (fill no) (layer "Edge.Cuts"))\n')


def add(board, text):
    s = open(board, encoding='utf-8', newline='').read()
    close = s.rstrip().rfind(')')
    open(board, 'w', encoding='utf-8', newline='').write(s[:close] + text + s[close:])


def holes(path):
    from kicad_parser import parse_kicad_pcb
    from mounting_holes import mounting_holes
    with contextlib.redirect_stdout(io.StringIO()):
        return mounting_holes(parse_kicad_pcb(path))


def main():
    td = tempfile.mkdtemp(prefix='krt1502_')
    try:
        print("1. the issue's hole: a 3.2 mm Edge.Cuts circle beside a track")
        b = os.path.join(td, 'circle.kicad_pcb')
        subprocess.run(PY + [os.path.join('py_router', 'copy_board.py'),
                             os.path.join('kicad_files', 'routed_output.kicad_pcb'), b],
                       capture_output=True, cwd=REPO)
        run_utils.evidence(b)
        add(b, circle(213.55, 92.1, 3.2))
        r = subprocess.run(PY + [os.path.join('py_tools', 'check_mounting_holes.py'), b],
                           capture_output=True, text=True, cwd=REPO)
        check('check_mounting_holes grades it: FAIL, 5 items under the head',
              r.returncode == 1 and 'FAIL  CUT1' in r.stdout and '5 item(s)' in r.stdout,
              r.stdout[-400:])
        out = os.path.join(td, 'out.kicad_pcb')
        r = subprocess.run(PY + [os.path.join('py_router', 'add_rule_area.py'), b, out,
                                 '--name', 'MOUNT', '--mounting-holes', '--forbid', 'tracks', 'vias'],
                           capture_output=True, text=True, cwd=REPO)
        check("add_rule_area --mounting-holes rings it (MOUNT_CUT1)",
              r.returncode == 0 and os.path.isfile(out)
              and 'MOUNT_CUT1' in open(out, encoding='utf-8').read(), r.stdout[-300:])

        print('2. what is not a hole')
        rb = os.path.join(td, 'round.kicad_pcb')
        with open(rb, 'w') as fh:
            fh.write('(kicad_pcb (version 20240108) (generator test)\n'
                     '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                     '  (net 0 "")\n'
                     + circle(50, 50, 40)        # the round board's own outline
                     + circle(40, 50, 1.5)       # under 2.0 mm: a via-size cut-out
                     + circle(60, 50, 3.2)       # a real hole
                     + ')\n')
        got = holes(rb)
        check('a round board: its outline is no hole, a 1.5 mm cut-out is none, the 3.2 mm one is',
              [(h.ref, round(h.x, 2), h.drill) for h in got] == [('CUT1', 60.0, 3.2)], f'{got}')
        add(rb, '  (footprint "MountingHole:M3" (layer "F.Cu") (at 60 50)\n'
                '    (property "Reference" "H1" (at 0 0) (layer "F.SilkS"))\n'
                '    (pad "" np_thru_hole circle (at 0 0) (size 3.2 3.2) (drill 3.2) '
                '(layers "*.Cu" "*.Mask")))\n')
        got = holes(rb)
        check('a footprint hole drawn over the circle is one hole, the footprint',
              [h.ref for h in got] == ['H1'], f'{[h.ref for h in got]}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
