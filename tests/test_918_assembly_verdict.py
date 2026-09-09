#!/usr/bin/env python3
"""`board_score`'s assembly component must read the VERDICT, not one conjunct (#918).

`check_assembly` decides NOT BUILDABLE on five conjuncts
(`py_tools/check_assembly.py`)::

    not_buildable = bool(g['blocking'] or locked_contact or stack_groups
                         or g['containment_blocking']
                         or courtyard_gating)

`blocking` is the FIRST of them and means "pad intersections". `board_score`'s
`score_assembly` read that scalar alone and accepted `rc in (0, 4)`, so a board
that is unbuildable through any of the other four contributed **0** to the
number the whole placement/routing loop ranks and stops on.

The producer already publishes the answer -- `buildable` and `verdict` were
added for exactly this, and the comment above them names `board_score` as the
consumer that still derived it. So the fix is to READ the key, never to
re-derive the disjunction here.

The fixture is the run-19 shape, built the same way
`tests/test_assembly_coincident_stack.py` builds it: C1 (an 0603) moved onto
C3's origin (an 8x10 electrolytic whose pads sit ~3mm out). No pad intersects,
so `blocking` stays 0 and ONLY the coincident-origin conjunct fires. That is
the sharpest possible input for this claim: every other conjunct is zero, so a
non-zero assembly count can only have come from the verdict.

Both arms run the REAL `score_assembly` against the REAL `check_assembly`. A
unit test over a hand-built dict would re-implement the partition it is meant
to check and would pass either way.

Run: python3 -X utf8 tests/test_918_assembly_verdict.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(
    ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts'))
import run_utils                                              # noqa: E402
import board_score                                            # noqa: E402

FIXTURE = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
SCORE = os.path.join(ROOT, '.claude', 'skills',
                     'plan-pcb-placement-and-routing', 'scripts',
                     'board_score.py')
CHECK_ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')

#: C1's pose in the tracked board, and C3's -- the same two literals
#: `test_assembly_coincident_stack.py` uses. A drifted fixture must fail
#: LOUDLY here, not downstream as a mystery-clean grade.
C1_AT = '(at 200.66 34.29 90)'
C3_AT = '(at 144.78 31.75 90)'

passed = 0
failed = 0


def check(name, ok, detail=''):
    """`detail` reads as a MEASUREMENT: it prints on OK as well as on FAIL."""
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}' + (f' -- {detail}' if detail else ''))
    else:
        failed += 1
        print(f'  FAIL {name}' + (f' -- {detail}' if detail else ''))


def _stage(tmp, stacked):
    """A copy of the fixture WITH its siblings, optionally with C1 stacked.

    Staged with `copy_board`, which carries every sibling, because a bare
    `cp` of a board strands the `.kicad_pro` holding its DRC floor and the
    next step then grades a different spec (#441). This particular fixture
    ships no `.kicad_pro` -- it prints the tool's own warning saying so --
    but both arms are staged the same way, so the comparison stands.
    """
    dst = os.path.join(tmp, 'subject.kicad_pcb')
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    from copy_board import copy_board                        # noqa: E402
    copy_board(FIXTURE, dst)
    if stacked:
        text = open(dst, encoding='utf-8').read()
        moved = text.replace(C1_AT, C3_AT, 1)
        if moved == text:
            raise AssertionError(
                f'fixture drift: {C1_AT!r} is not in {FIXTURE}. This test '
                f'stacks C1 onto C3 to fire the coincident-origin conjunct '
                f'with `blocking` at 0; without the move it grades a clean '
                f'board and asserts nothing.')
        open(dst, 'w', encoding='utf-8').write(moved)
    return dst


def _check_assembly(board, out):
    r = subprocess.run([sys.executable, '-X', 'utf8', CHECK_ASSEMBLY, board,
                        '--json', out], capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=600,
                       cwd=ROOT, env=run_utils.tool_env())
    doc = json.load(open(out, encoding='utf-8')) if os.path.isfile(out) else None
    return r.returncode, r.stdout + r.stderr, doc


def main():
    run_utils.evidence(FIXTURE, 'the tracked fixture board')
    tmp = tempfile.mkdtemp(prefix='t918_')
    try:
        # ------------------------------------------------ the INPUT, first.
        # A check whose input is not what it believes tests nothing.
        print('the staged board is NOT BUILDABLE with blocking 0')
        board = _stage(tmp, stacked=True)
        rc, out, doc = _check_assembly(board, os.path.join(tmp, 'a.json'))
        check('check_assembly exits 4', rc == 4, f'exit {rc}')
        check('...through a conjunct that is NOT `blocking`',
              doc is not None and doc.get('blocking') == 0,
              f"blocking={doc and doc.get('blocking')}")
        check('...and says so in the key it publishes for consumers',
              doc is not None and doc.get('buildable') is False,
              f"buildable={doc and doc.get('buildable')}")
        check('the firing conjunct is the coincident-origin stack',
              doc is not None and (doc.get('coincident_origins') or 0) >= 1,
              f"coincident_origins={doc and doc.get('coincident_origins')}")

        # ---------------------------------------------- the CLAIM under test.
        print('board_score.score_assembly does not score that board clean')
        with tempfile.TemporaryDirectory(prefix='t918_score_') as sd:
            comp = board_score.score_assembly(ROOT, board, '', sd,
                                              clearance=None)
        check('the component ran', comp.get('ran') is True, str(comp.get('reason')))
        check('it reports the verdict it read', comp.get('buildable') is False,
              f"buildable={comp.get('buildable')!r} "
              f"(absent means the component never read the key)")
        check('its count is NOT 0 on an unbuildable board',
              isinstance(comp.get('count'), int) and comp['count'] > 0,
              f"count={comp.get('count')!r} -- #918: the old code returned "
              f"int(doc['blocking'] or 0), which is 0 here")

        # ------------------------------------------------- and end to end.
        print('...so the published score does not report assembly 0 either')
        sj = os.path.join(tmp, 'score.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', SCORE, board,
                            '--json', sj, '-q'], capture_output=True,
                           text=True, encoding='utf-8', errors='replace',
                           timeout=900, cwd=ROOT, env=run_utils.tool_env())
        sdoc = json.load(open(sj, encoding='utf-8')) if os.path.isfile(sj) else None
        # The EXIT CODE is not the assertion: splitflap_driver is copper-free,
        # so `unrouted` alone already makes this 4. The claim is about the
        # assembly entry.
        check('blocking_by.assembly is non-zero',
              sdoc is not None
              and (sdoc.get('blocking_by') or {}).get('assembly'),
              f"assembly={sdoc and (sdoc.get('blocking_by') or {}).get('assembly')!r}"
              f" rc={r.returncode}")

        # ------------------------------------------------------ the CONTROL.
        # Without it a component that returned a constant would pass every
        # assertion above.
        print('the unmodified board still scores assembly 0, buildable')
        clean = _stage(tmp, stacked=False)
        rc, out, doc = _check_assembly(clean, os.path.join(tmp, 'c.json'))
        check('check_assembly exits 0 on the control', rc == 0, f'exit {rc}')
        with tempfile.TemporaryDirectory(prefix='t918_ctl_') as sd:
            comp = board_score.score_assembly(ROOT, clean, '', sd,
                                              clearance=None)
        check('control: buildable True', comp.get('buildable') is True,
              str(comp.get('buildable')))
        check('control: count 0', comp.get('count') == 0, str(comp.get('count')))

        # ------------------------------------------------- the REFUSAL arms.
        # `assembly_component` is pure, so the shapes a real board cannot
        # easily produce are asserted directly -- and they are asserted on the
        # REASON, because a component that returns `ran: False` for the wrong
        # reason is a guard that did not hold.
        print('the scorer refuses a document it cannot read honestly')
        r = board_score.assembly_component({'blocking': 0}, 0)
        check('no `buildable` key -> skipped, not 0',
              r.get('ran') is False and r.get('count') is None
              and 'buildable' in (r.get('reason') or ''),
              repr(r.get('reason')))
        r = board_score.assembly_component(
            {'blocking': 0, 'buildable': True, 'verdict': 'buildable'}, 4)
        check('exit 4 with buildable True -> skipped, naming the contradiction',
              r.get('ran') is False
              and 'contradicts itself' in (r.get('reason') or ''),
              repr(r.get('reason')))
        r = board_score.assembly_component(
            {'blocking': 0, 'buildable': False, 'verdict': 'NOT BUILDABLE'}, 0)
        check('exit 0 with buildable False -> skipped too (the other way)',
              r.get('ran') is False
              and 'contradicts itself' in (r.get('reason') or ''),
              repr(r.get('reason')))

        print('a verdict whose every published magnitude is 0 still counts')
        r = board_score.assembly_component(
            {'blocking': 0, 'buildable': False, 'verdict': 'NOT BUILDABLE',
             'locked_contacts': 0, 'coincident_origins': 0,
             'containment_blocking': 0, 'courtyard_blocking_gating': None}, 4)
        check('count is floored at 1, not 0', r.get('count') == 1,
              f"count={r.get('count')!r} basis={r.get('count_basis')!r}")
        check('...and the basis says the floor was used',
              'floored at 1' in (r.get('count_basis') or ''),
              repr(r.get('count_basis')))
        check('the unarmed fifth conjunct is named, never counted as 0',
              r.get('conjuncts_unmeasured') == ['courtyard_blocking_gating']
              and r.get('courtyard_gating_armed') is False
              and '--baseline' in (r.get('courtyard_gating_reason') or ''),
              f"unmeasured={r.get('conjuncts_unmeasured')!r}")

        print('the containment conjunct is the BLOCKING subset, not `contained`')
        r = board_score.assembly_component(
            {'blocking': 0, 'buildable': True, 'verdict': 'buildable',
             'locked_contacts': 0, 'coincident_origins': 0,
             'contained': 4, 'containment_blocking': 0,
             'courtyard_blocking_gating': None}, 0)
        check('4 by-design containments do not make a buildable board score',
              r.get('count') == 0 and r['conjuncts']['containment_blocking'] == 0,
              f"count={r.get('count')!r} conjuncts={r.get('conjuncts')!r}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print(f'\n{passed} passed, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
