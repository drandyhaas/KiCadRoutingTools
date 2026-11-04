#!/usr/bin/env python3
"""#1648: a wide Edge.Cuts circle is a window, not a mounting hole; --skip leaves one hole out.

plan-pcb-routing Step 0 defined a mounting hole as a footprint, while
`add_rule_area --mounting-holes` also rang every closed Edge.Cuts circle of
2 mm or more (#1502), at 0.4 x its diameter whatever its size: a 20 mm round
window took an 8 mm keep-out. The skill's only way out, "leave the step out",
dropped every ring, since `--ref` is refused with `--mounting-holes`.

Now an Edge.Cuts circle wider than `EDGE_CUT_MAX_SCREW_DIAMETER` (7.0 mm,
ISO 273's coarse clearance hole for M6) is a window and no mounting hole, and
`--skip HOLE ...` leaves a named hole without its ring.

Checks:
  1. `mounting_holes()` on a board with footprint holes plus 3.2, 7.0 and
     20 mm Edge.Cuts circles reports CUT1 (3.2) and CUT2 (7.0, the limit
     itself) and not the 20 mm window; add_rule_area rings exactly those,
     and check_mounting_holes grades no window;
  2. `--skip CUT2` writes every ring but CUT2's and says so;
  3. a `--skip` name that is not one of the board's holes is refused (exit
     2, the board's holes printed), nothing written; `--skip` without
     `--mounting-holes` is a usage error;
  4. the skill's Step 0 names the Edge.Cuts circle, its limit, the CUT names
     and --skip.

    python3 tests/test_1648_mounting_hole_skip.py
"""
import contextlib
import io
import os
import re
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
TOOL = os.path.join(REPO, 'py_router', 'add_rule_area.py')
BOARD = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
SKILL = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')


def circle(x, y, d):
    return (f'  (gr_circle (center {x} {y}) (end {x + d / 2} {y}) (stroke (width 0.05) '
            f'(type default)) (fill no) (layer "Edge.Cuts"))\n')


def rings(path):
    return sorted(set(re.findall(r'\(name "(MOUNT_[^"]+)"\)', open(path, encoding='utf-8').read())))


def main():
    evidence(BOARD, 'fixture board')
    from kicad_parser import parse_kicad_pcb
    import mounting_holes as mh
    assert mh.EDGE_CUT_MAX_SCREW_DIAMETER == 7.0, mh.EDGE_CUT_MAX_SCREW_DIAMETER
    with tempfile.TemporaryDirectory(prefix='krt1648_') as td:
        b = os.path.join(td, 'in.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, b], accept=True)
        with contextlib.redirect_stdout(io.StringIO()):
            before = [h.ref for h in mh.mounting_holes(parse_kicad_pcb(b))]
        s = open(b, encoding='utf-8', newline='').read()
        close = s.rstrip().rfind(')')
        open(b, 'w', encoding='utf-8', newline='').write(
            s[:close] + circle(60, 40, 3.2) + circle(80, 40, 7.0) + circle(100, 40, 20.0)
            + s[close:])

        print('1. the 20 mm window is no mounting hole; 3.2 and 7.0 mm are')
        with contextlib.redirect_stdout(io.StringIO()):
            got = [(h.ref, h.drill) for h in mh.mounting_holes(parse_kicad_pcb(b))]
        cuts = [g for g in got if g[0].startswith('CUT')]
        assert cuts == [('CUT1', 3.2), ('CUT2', 7.0)], got
        assert [g[0] for g in got if not g[0].startswith('CUT')] == before, (got, before)
        full = os.path.join(td, 'full.kicad_pcb')
        check(PY + [TOOL, b, full, '--name', 'MOUNT', '--mounting-holes'], accept=True)
        every = rings(evidence(full))
        assert every == sorted(f'MOUNT_{r}' for r in before + ['CUT1', 'CUT2']), every
        r = check(PY + [os.path.join(REPO, 'py_tools', 'check_mounting_holes.py'), b],
                  accept=True)
        assert 'CUT1' in r.stdout and 'CUT2' in r.stdout, r.stdout[-600:]
        assert 'CUT3' not in r.stdout, r.stdout[-600:]
        print(f'  PASS: {len(every)} rings, none for the window; check_mounting_holes '
              f'grades no window')

        print('2. --skip leaves one hole out')
        out = os.path.join(td, 'out.kicad_pcb')
        r = check(PY + [TOOL, b, out, '--name', 'MOUNT', '--mounting-holes', '--skip', 'CUT2'],
                  accept=True)
        assert 'left without a ring, as --skip asks: CUT2' in r.stdout, r.stdout
        got = rings(evidence(out))
        assert got == [n for n in every if n != 'MOUNT_CUT2'], (got, every)
        print(f'  PASS: {len(got)} rings written, CUT2 left out and named')

        print('3. refusals')
        bad = os.path.join(td, 'bad.kicad_pcb')
        r = check(PY + [TOOL, b, bad, '--name', 'MOUNT', '--mounting-holes', '--skip', 'CUT3'],
                  refuse='not a mounting hole of this board', code=2)
        assert 'CUT2' in r.stderr and not os.path.exists(bad), r.stderr
        check(PY + [TOOL, b, bad, '--name', 'K', '--rect', '0', '0', '1', '1', '--skip', 'H1'],
              refuse='it needs --mounting-holes', code=2)
        assert not os.path.exists(bad)
        print('  PASS: the window name (no hole), and --skip without --mounting-holes, '
              'refused; nothing written')

    print('4. the skill')
    text = open(SKILL, encoding='utf-8').read()
    step0 = text[text.index('### Step 0 of the example chain'):]
    step0 = step0[:step0.index('\n### ', 5)]
    for want in ('Edge.Cuts circle of 2 to 7 mm', 'is a window', '`<name>_CUT1`', '--skip'):
        assert want in step0, f'Step 0 does not say {want!r}'
    assert 'leave the step out then' not in step0, 'Step 0 still drops every ring for one hole'
    print('  PASS: Step 0 names the Edge.Cuts hole and its limit, its CUT name and --skip')
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
