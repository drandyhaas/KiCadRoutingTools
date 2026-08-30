#!/usr/bin/env python3
"""#788: the record of WHICH pre-route census the routing loop refuses on.

    python3 -X utf8 tests/test_788_instrument_record.py

Three files used to say the L2 gate blocks a first route on
`render_placement`'s `checklist.a_off_outline.pad_copper`. It does not -- it
refuses on `check_assembly`'s `oob_pad_count`, a different measurement of the
same idea (a part-level pad AABB against an outline inflated by the grading
clearance, versus a per-PAD margin-0 outline test). The same paragraph also
called that number "the only pre-route quantity in this repo that refuses
anything", while L2 refuses on `buildable`, `locked_contacts` and `blocking`
too.

WHY THIS IS A TEST AND NOT A CAREFUL COMMIT MESSAGE. Nothing reads a document,
so nothing would have caught the claim drifting back. (It had not been wrong
for long -- the doc sentence is two days old and shipped in no release; the
older site is `placement_driver.py`'s guidance, which this branch amends rather
than corrects. An earlier draft of this file said "wrong for two releases",
which a `git log -S` refutes in one command.) The gate
here is BIDIRECTIONAL, in the shape `test_703_predictor_claims.py` established:
the corrected sentence must be PRESENT and the wrong one ABSENT. An
absence-only check is the trap this repo keeps re-finding -- it passes when the
file is renamed, emptied, or its wording drifts, so every absence assert below
is paired with a presence assert on the same file, and every file is proved
non-empty before either runs.

It costs milliseconds, spawns nothing, and needs no board: the numbers behind
the decision live in `tests/measure_788_censuses.py` (which needs the
gitignored study tree) and `tests/test_788_marginal_literals.py` (which does
not).
"""
import os
import sys

RUN_ALL_TIMEOUT = 60
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DOC = 'docs/placement-predictors.md'
HARVEST = 'tests/stress/harvest_predictor_rows.py'
LOOP = ('.claude/skills/plan-pcb-placement-and-routing/scripts/'
        'loop_driver.py')
DRIVER = '.claude/skills/plan-pcb-placement/scripts/placement_driver.py'
LEGALITY = 'py_placer/placement/legality.py'
ASSEMBLY = 'py_tools/check_assembly.py'
RENDER = 'py_tools/render_placement.py'

#: The sentence that was wrong, compared WHITESPACE-NORMALISED. Review found
#: the first version line-break-exact: re-inserting the identical claim wrapped
#: differently ("only pre-route quantity\nin this repo...") passed both
#: spellings, which is the same shape of hole this file exists to close. A
#: paraphrase still evades it -- that is the honest limit and the docstring
#: says so -- but a revert of the words themselves cannot.
WRONG_ONLY = 'only pre-route quantity in this repo that refuses anything'


def _flat(text):
    """Collapse every run of whitespace, so a re-wrap is not a new sentence."""
    return ' '.join(text.split())

FAILURES = []


def check(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILURES.append(what)


def read(rel):
    """The file's text, having proved it is a real non-empty file first.

    `run_utils.evidence`'s rule, inlined because importing run_utils would put
    this file in run_all's integration bucket for a check that spawns nothing.
    A check whose input is missing tests nothing, and an absence assert over an
    empty string passes every time.
    """
    p = os.path.join(ROOT, rel)
    if not os.path.isfile(p) or os.path.getsize(p) == 0:
        FAILURES.append(f'{rel} is missing or empty -- nothing below it is '
                        f'evidence')
        return ''
    with open(p, encoding='utf-8') as fh:
        return fh.read()


def t_the_doc_names_the_census_that_refuses():
    doc = read(DOC)
    if not doc:
        return
    check(WRONG_ONLY not in _flat(doc),
          f'{DOC} no longer calls it the only pre-route quantity that refuses')
    check('oob_pad_count' in doc,
          f'{DOC} names oob_pad_count, which is what the gate reads')
    check('checklist.a_off_outline.pad_copper' in doc
          or 'a_off_outline.pad_copper' in doc,
          f'{DOC} still names the render key it is distinguishing FROM')
    # The correction is only a correction if the doc says which is which. A
    # file that mentions both names and asserts nothing about them would pass
    # the two checks above while saying nothing.
    check('per-PAD' in doc or 'per-pad' in doc,
          f'{DOC} says how the two censuses differ, not just that they do')
    check('locked_contacts' in doc and 'buildable' in doc,
          f'{DOC} names the other quantities L2 refuses on')


def t_the_harvest_comment_agrees_with_the_doc():
    src = read(HARVEST)
    if not src:
        return
    check(WRONG_ONLY not in _flat(src.replace('#:', ' ')),
          f'{HARVEST} no longer carries the same wrong claim')
    check('oob_pad_count' in src,
          f'{HARVEST} names the key the gate actually refuses on')


def t_the_gate_really_does_read_oob_pad_count():
    """Behavioural, not documentary: the claim is about code, so read the code.

    If a later change moved the L2 gate onto the render channel, the doc
    correction would become wrong again and every check above would still
    pass -- they only assert what the DOC says.
    """
    src = read(LOOP)
    if not src:
        return
    check("_count('oob_pad_count')" in src,
          'loop_driver reads oob_pad_count through the vetted _count rule')
    check('a_off_outline' not in src,
          'loop_driver does not read render_placement\'s off-outline channel')
    for key in ("_count('blocking')", "_count('locked_contacts')"):
        check(key in src, f'loop_driver still refuses via {key}')
    check("rep.get('buildable')" in src,
          'loop_driver still reads buildable')


def t_both_censuses_still_exist_and_are_distinct():
    """The correction claims two producers. Prove both are still there.

    Otherwise this whole file could go green on a tree where one census was
    deleted, and the doc would be describing a distinction that no longer
    exists.
    """
    asm, ren, leg = read(ASSEMBLY), read(RENDER), read(LEGALITY)
    if not (asm and ren and leg):
        return
    check("'oob_pad_count':" in asm,
          'check_assembly still emits oob_pad_count')
    check("'pad_conflicts':" in asm and "'hole_conflicts':" in asm,
          'check_assembly still emits the graze and hole counts L2 could read')
    check("'pad_copper':" in ren,
          'render_placement still emits checklist.a_off_outline.pad_copper')
    check('oob_pad_basis' in leg,
          'legality still ships the basis string that names the difference')
    check('render_placement' in leg,
          'the basis string still points at the other census by name')


def t_the_driver_guidance_points_at_both():
    src = read(DRIVER)
    if not src:
        return
    check('oob_pad_count' in src,
          'placement_driver names the number that refuses downstream')
    check('a_off_outline.pad_copper' in src,
          'placement_driver still names the render channel it also uses')


def main():
    for fn in (t_the_doc_names_the_census_that_refuses,
               t_the_harvest_comment_agrees_with_the_doc,
               t_the_gate_really_does_read_oob_pad_count,
               t_both_censuses_still_exist_and_are_distinct,
               t_the_driver_guidance_points_at_both):
        print(fn.__name__ + ':')
        fn()
    if FAILURES:
        print(f'\ntest_788_instrument_record: {len(FAILURES)} failure(s)')
        for f in FAILURES:
            print('  - ' + f)
        return 1
    print('\ntest_788_instrument_record: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
