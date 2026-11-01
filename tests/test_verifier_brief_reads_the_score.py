#!/usr/bin/env python3
"""The placement verifier's brief must grade what its own score grades.

`.claude/skills/plan-pcb-placement/references/verifier.md` is handed to an
independent verifier, which follows it literally. Where the brief and
`board_score` (the score the same brief runs, and `converge record --final`
reads) disagree, the verifier writes a verdict the run's own tools refuse.

#1601: step 5 turned a declared class with an `ungraded` component into FAIL,
and listed "power widths" among the classes. Every full-mode route records
power widths (`--power-nets-widths`, which plan-pcb-routing prescribes on every
board), and board_score lists `net_widths` as ungraded unless
`--net-min-widths` is given -- the requested widths are measured in
`net_widths.requested_under`, not graded -- so a verifier following step 5
failed every board the routing skill produced, while the same brief's next
sentences say those widths are only reported. A width counts as a declared
class here only when it is given in `--net-min-widths`.

Static: reads the brief, spawns nothing.

    python3 -X utf8 tests/test_verifier_brief_reads_the_score.py
"""
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
VERIFIER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                        'references', 'verifier.md')


def _read(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def _fail_clause_classes(text):
    """The classes in step 5's "declares a class (...) and its component is
    ungraded, the verdict is FAIL", or None when the clause is not there."""
    flat = ' '.join(text.split())
    m = re.search(r'declares a class \(([^)]*)\) and its component is '
                  r'ungraded, the verdict is FAIL', flat)
    if not m:
        return None
    return [c.strip() for c in re.split(r',\s*(?:or\s+)?', m.group(1))
            if c.strip()]


def _width_problems(classes):
    """Why this class list FAILs a board for widths board_score leaves
    ungraded by design: [] when it does not."""
    out = []
    for c in classes:
        if 'power' in c.lower():
            out.append(f'{c!r}: requested power widths are measured, not '
                       f'graded (net_widths stays ungraded without '
                       f'--net-min-widths)')
        elif 'width' in c.lower() and '--net-min-widths' not in c:
            out.append(f'{c!r}: a width is a declared class only when given '
                       f'in --net-min-widths')
    return out


def test_requested_power_widths_are_not_an_ungraded_declaration():
    classes = _fail_clause_classes(_read(VERIFIER))
    assert classes, ('verifier.md step 5 no longer states its FAIL clause in '
                     'the form this test reads; re-read the brief and update '
                     'the pattern rather than letting the check pass on '
                     'nothing')
    problems = _width_problems(classes)
    assert not problems, 'verifier.md step 5: ' + '; '.join(problems)
    print(f'  PASS: step 5 FAILs only the classes board_score grades '
          f'({len(classes)}: {", ".join(classes)})')


def test_the_width_check_trips_on_the_text_it_replaced():
    """Negative control: the #1601 wording must trip `_width_problems`, or
    the check above passes whatever the brief says."""
    old = ('5. **Declared classes.** Name every `ungraded` component as '
           'UNEXAMINED. If the\n   board declares a class (diff pairs, '
           'impedance, length groups, power widths,\n   high-speed nets) and '
           'its component is ungraded, the verdict is FAIL.')
    classes = _fail_clause_classes(old)
    assert classes and 'power widths' in classes, classes
    assert _width_problems(classes), 'the old wording passed the width check'
    assert _width_problems(['per-net widths']), \
        'a width class that names no --net-min-widths passed'
    assert not _width_problems(['widths given in `--net-min-widths`'])
    print('  PASS: the #1601 wording trips the width check')


if __name__ == '__main__':
    for _name, _fn in sorted(globals().items()):
        if _name.startswith('test_') and callable(_fn):
            print(f'--- {_name}')
            _fn()
    print('ALL PASS')
