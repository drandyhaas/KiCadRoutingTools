#!/usr/bin/env python3
"""#1811: on a poured net the copper grade already lists, check_connected and
board_score report the pieces KiCad's refill counts, not the pads-only count.

KiCad links every copper cluster of a net, a fill island holding only a via
included (tests/test_island_link_semantics.py). The copper grade counts pad
components, and the KiCad cross-check used to take KiCad's count only for a
net the copper grade had PASSED. So a GND pour cut into 4 clusters (2 with a
pad, 2 with only a via) read 1 join against KiCad's 3 links, under a status
line saying KiCad "agrees"; and joining the open pad (3 clusters, now read
through the KiCad-only path) RAISED the count to 2 -- a repair reading as a
regression.

Three boards from tests/fixtures/island_semantics/two_islands_smd_stub_via:
  two    = the fixture: 2 GND clusters (control)
  four   = two more SIG tracks cut the right half in three, one GND via in
           each new island: 4 GND clusters
  joined = four, plus an F.Cu GND track joining the two pads: 3 clusters

Needs kicad-cli; exits 77 without it.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import check, evidence  # noqa: E402

FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', 'island_semantics',
                       'two_islands_smd_stub_via.kicad_pcb')
CHECK = os.path.join(ROOT, 'py_router', 'check_connected.py')
SCORE = os.path.join(ROOT, 'py_tools', 'board_score.py')


def _seg(x1, y1, x2, y2, layer, net):
    return (f'(segment\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
            f'\t\t(width 0.3)\n\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t)\n\n')


def _via(x, y):
    return (f'(via\n\t\t(at {x} {y})\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "GND")\n\t)\n\n')


def _boards(d):
    with open(evidence(FIXTURE, 'the island fixture'), encoding='utf-8',
              newline='') as f:
        text = f.read()
    zone = '(zone\n'
    assert text.count(zone) == 1 and '(island_removal_mode 0)' in text, \
        'the fixture changed shape: one zone, islands kept'
    cuts = (_seg(10, 5, 19.5, 5, 'B.Cu', 'SIG') + _seg(10, 15, 19.5, 15, 'B.Cu', 'SIG')
            + _via(15, 3) + _via(15, 17))
    out = {}
    for name, extra in (('two', ''), ('four', cuts),
                        ('joined', cuts + _seg(5, 10, 15, 10, 'F.Cu', 'GND'))):
        t = text.replace(zone, extra + zone)
        if extra:
            t = t.replace('(island_removal_mode 0)', '(island_removal_mode 1)')
        p = os.path.join(d, f'{name}.kicad_pcb')
        with open(p, 'w', encoding='utf-8', newline='') as f:
            f.write(t)
        out[name] = p
    return out


def _connected(board, pieces, env=None):
    """check_connected exits 1 (the net is open) naming `pieces` pieces."""
    return check([sys.executable, '-X', 'utf8', CHECK, board],
                 refuse=f'Disconnected components: {pieces}\n', code=1,
                 env=env).stdout


def _joins(board, d):
    js = os.path.join(d, os.path.basename(board) + '.score.json')
    subprocess.run([sys.executable, '-X', 'utf8', SCORE, board, '--json', js,
                    '--quiet'], capture_output=True, cwd=ROOT, timeout=900)
    with open(evidence(js, 'board_score json'), encoding='utf-8') as f:
        return json.load(f)['joins_needed']


def main():
    from kicad_oracle import find_kicad_cli
    if not find_kicad_cli():
        print('SKIP: kicad-cli not found')
        sys.exit(77)
    with tempfile.TemporaryDirectory() as d:
        b = _boards(d)

        out = _connected(b['four'], 4)
        assert 'agrees with copper grading' not in out, out
        assert 'copper grading counted 2 by pads alone' in out, out
        print('  PASS: four clusters read 4 pieces (KiCad: 3 links), and the '
              'status line does not say "agrees"')

        _connected(b['joined'], 3)
        out = _connected(b['two'], 2)
        assert 'agrees with copper grading' in out, out
        print('  PASS: controls unchanged: joined reads 3, the fixture reads 2 '
              'and "agrees"')

        env = dict(os.environ, KICAD_NO_GRADE_RECONCILE='1')
        _connected(b['four'], 2, env=env)
        print('  PASS: without the cross-check, four keeps the pads-only 2')

        j4, jj = _joins(b['four'], d), _joins(b['joined'], d)
        assert (j4, jj) == (3, 2), (
            f'board_score joins_needed four={j4} joined={jj}; KiCad needs 3 '
            f'then 2 -- joining the open pad must not raise the count')
        print('  PASS: board_score joins_needed 3 -> 2 when the open pad is joined')
    print('ALL PASS')


if __name__ == '__main__':
    main()
