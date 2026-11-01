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

#1595: the brief's check_drc line -- the one its `drc` lens is defined as --
passed `--clearance-margin 0.1`, and so did the skill's `scripts/grade.py`,
while board_score runs check_drc at its default margin (0.05). On esp_prog
with a track 7% inside the clearance the line read NO DRC VIOLATIONS beside
board_score's `drc 1`, grade.py recorded `check_drc_rc 0` beside `blocking 1`,
and `converge record --final` refused the drc-lens PASS the brief produced. The
check below holds the brief's command and grade.py's to the distance-tolerance
flags board_score's own check_drc call (`score_drc`) passes. What it cannot
see: a tolerance a CALLER of `score_drc` hands it through an argument.

Static: reads the brief, grade.py and two tools' sources; spawns nothing.

    python3 -X utf8 tests/test_verifier_brief_reads_the_score.py
"""
import ast
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement')
VERIFIER = os.path.join(SKILL, 'references', 'verifier.md')
GRADE = os.path.join(SKILL, 'scripts', 'grade.py')
CHECK_DRC = os.path.join(ROOT, 'py_router', 'check_drc.py')
BOARD_SCORE = os.path.join(ROOT, 'py_tools', 'board_score.py')


def _read(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def _tolerance_flags(check_drc_src):
    """check_drc's options that move, by distance, the line between a
    violation and none: every option whose name says margin or tolerance."""
    return set(re.findall(
        r"""add_argument\(\s*['"](--[\w-]*(?:margin|toleran)[\w-]*)['"]""",
        check_drc_src))


def _dashed(nodes):
    return {n.value for n in nodes
            if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and n.value.startswith('--')}


def _score_drc_flags(board_score_src):
    """The --flags board_score's `score_drc` puts on its check_drc command
    line, or None when it no longer runs check_drc there."""
    for node in ast.walk(ast.parse(board_score_src)):
        if isinstance(node, ast.FunctionDef) and node.name == 'score_drc':
            consts = list(ast.walk(node))
            if not any(isinstance(n, ast.Constant) and n.value == 'check_drc.py'
                       for n in consts):
                return None
            return _dashed(consts)
    return None


def _grade_check_drc_flags(grade_src):
    """The --flags of every argv list in grade.py that runs check_drc, or
    None when there is none."""
    found = None
    for node in ast.walk(ast.parse(grade_src)):
        if isinstance(node, ast.List) and any(
                isinstance(e, ast.Constant) and e.value == 'py_router/check_drc.py'
                for e in node.elts):
            found = (found or set()) | _dashed(node.elts)
    return found


def _brief_check_drc_flags(text):
    """(commands, their --flags): every backticked command in the brief that
    runs check_drc.py."""
    cmds = [c for c in re.findall(r'`([^`\n]*)`', text)
            if c.lstrip().startswith('python') and 'check_drc.py' in c]
    return cmds, {f for c in cmds for f in re.findall(r'(--[\w-]+)', c)}


def _tolerance_mismatches(tol, score, brief, grade):
    out = []
    for where, flags in (('verifier.md', brief), ('grade.py', grade)):
        if flags & tol != score & tol:
            out.append(f'{where} runs check_drc with tolerance flag(s) '
                       f'{sorted(flags & tol)}, board_score.score_drc with '
                       f'{sorted(score & tol)}')
    return out


def test_the_drc_lens_reads_the_tolerance_the_score_grades_at():
    tol = _tolerance_flags(_read(CHECK_DRC))
    assert '--clearance-margin' in tol, \
        f'check_drc tolerance flags not found where expected: {sorted(tol)}'
    score = _score_drc_flags(_read(BOARD_SCORE))
    assert score is not None, 'board_score.score_drc no longer runs check_drc'
    cmds, brief = _brief_check_drc_flags(_read(VERIFIER))
    assert cmds, 'verifier.md runs no check_drc command this test can read'
    grade = _grade_check_drc_flags(_read(GRADE))
    assert grade is not None, 'grade.py runs no check_drc argv this test can read'
    bad = _tolerance_mismatches(tol, score, brief, grade)
    assert not bad, '; '.join(bad)
    print(f'  PASS: verifier.md ({len(cmds)} command(s)), grade.py and '
          f'board_score run check_drc at one tolerance '
          f'({sorted(score & tol) or "check_drc defaults"})')


def test_the_tolerance_check_trips_on_the_text_it_replaced():
    """Negative control: the #1595 command line and argv must trip it."""
    tol = _tolerance_flags(_read(CHECK_DRC))
    score = _score_drc_flags(_read(BOARD_SCORE))
    _, brief = _brief_check_drc_flags(
        '   - `python3 -X utf8 py_router/check_drc.py <BOARD> --baseline '
        '<INPUT BOARD> --clearance-margin 0.1`: no real violations.')
    grade = _grade_check_drc_flags(
        "rc, log = _run(['py_router/check_drc.py', board, '--baseline', "
        "baseline,\n                '--clearance-margin', '0.1'])\n")
    bad = _tolerance_mismatches(tol, score, brief, grade)
    assert len(bad) == 2, bad
    print('  PASS: the #1595 command line and argv trip the tolerance check')


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
