#!/usr/bin/env python3
"""#1648: add_rule_area --mounting-holes can leave one hole out, and the skill says what a hole is.

plan-pcb-routing Step 0 defined a mounting hole as a footprint, while
`add_rule_area --mounting-holes` also rings every closed Edge.Cuts circle of
2 mm or more (#1502), at 0.4 x its diameter whatever its size: a 20 mm round
window took an 8 mm keep-out. The skill's only way out, "leave the step out",
dropped every ring, since `--ref` is refused with `--mounting-holes`.

Checks:
  1. on a board with footprint holes plus a 3.2 mm and a 20 mm Edge.Cuts
     circle, `--skip CUT2` writes every ring but the window's and says so;
  2. a `--skip` name that is not one of the board's holes is refused (exit
     2, the name and the board's holes printed), nothing written; `--skip`
     without `--mounting-holes` is a usage error;
  3. the skill's Step 0 names the Edge.Cuts circle, the CUT names and --skip.

    python3 tests/test_1648_mounting_hole_skip.py
"""
import os
import re
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
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
    with tempfile.TemporaryDirectory(prefix='krt1648_') as td:
        b = os.path.join(td, 'in.kicad_pcb')
        check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, b], accept=True)
        s = open(b, encoding='utf-8', newline='').read()
        close = s.rstrip().rfind(')')
        open(b, 'w', encoding='utf-8', newline='').write(
            s[:close] + circle(60, 40, 3.2) + circle(100, 40, 20.0) + s[close:])

        print('1. every ring but the skipped window')
        full = os.path.join(td, 'full.kicad_pcb')
        check(PY + [TOOL, b, full, '--name', 'MOUNT', '--mounting-holes'], accept=True)
        every = rings(evidence(full))
        assert 'MOUNT_CUT1' in every and 'MOUNT_CUT2' in every, every
        out = os.path.join(td, 'out.kicad_pcb')
        r = check(PY + [TOOL, b, out, '--name', 'MOUNT', '--mounting-holes', '--skip', 'CUT2'],
                  accept=True)
        assert 'left without a ring, as --skip asks: CUT2' in r.stdout, r.stdout
        got = rings(evidence(out))
        assert got == [n for n in every if n != 'MOUNT_CUT2'], (got, every)
        print(f'  PASS: {len(got)} rings written, CUT2 left out and named')

        print('2. refusals')
        bad = os.path.join(td, 'bad.kicad_pcb')
        r = check(PY + [TOOL, b, bad, '--name', 'MOUNT', '--mounting-holes', '--skip', 'CUT9'],
                  refuse='not a mounting hole of this board', code=2)
        assert 'CUT2' in r.stderr and not os.path.exists(bad), r.stderr
        check(PY + [TOOL, b, bad, '--name', 'K', '--rect', '0', '0', '1', '1', '--skip', 'H1'],
              refuse='it needs --mounting-holes', code=2)
        assert not os.path.exists(bad)
        print('  PASS: an unknown name and --skip without --mounting-holes refused, '
              'nothing written')

    print('3. the skill')
    text = open(SKILL, encoding='utf-8').read()
    step0 = text[text.index('### Step 0 of the example chain'):]
    step0 = step0[:step0.index('\n### ', 5)]
    for want in ('Edge.Cuts circle', '`<name>_CUT1`', '--skip'):
        assert want in step0, f'Step 0 does not say {want!r}'
    assert 'leave the step out then' not in step0, 'Step 0 still drops every ring for one hole'
    print('  PASS: Step 0 names the Edge.Cuts hole, its CUT name and --skip')
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
