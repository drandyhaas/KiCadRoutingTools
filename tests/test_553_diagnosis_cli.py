"""#553 at the command line: the refusals, and the board census behind them.

Every check here goes through `run_utils.check(..., refuse=...)`, so a
traceback or an argparse accident is reported as a BROKEN TEST rather than
read as a satisfied guard. A non-zero exit is not evidence; the REASON is.

All four refusals happen BEFORE round 0's route -- that is the point of where
they sit in `main()`, and it is why this file costs seconds rather than the
minutes a routing run would. `run_utils.evidence` checks the board first,
because a check whose input is missing tests nothing.

The last test is the measurement that shaped the whole feature: on the boards
this repo grades placement on, `--group-by auto` derives NO BLOCK AT ALL. That
is why the diagnosis ranks loose parts as well as blocks, and why an
all-blocks design would have been inert on almost the entire tracked corpus.

    python3 -X utf8 tests/test_553_diagnosis_cli.py
"""

import os
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 300

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # placement split

import run_utils  # noqa: E402
from run_utils import check, evidence, tool  # noqa: E402

LOOP = tool('place_route_loop.py')
BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'),
                 'the board every refusal below is fed')
# A temp dir, NOT kicad_files/. If any of the four gates ever regresses, the
# loop routes esp_prog and scatters loop_round0*.{kicad_pcb,json,log} beside the
# board it read -- `--work-dir` defaults to the output's directory. The tree
# must not be the blast radius of a failing test.
_TMP = tempfile.mkdtemp(prefix='t553_cli_')
OUT = os.path.join(_TMP, 'should_never_be_written.kicad_pcb')

FAILURES = []


def report(what, fn):
    try:
        fn()
    except AssertionError as e:
        print(f'  FAIL {what}\n       {e}')
        FAILURES.append(what)
    except Exception as e:                            # noqa: BLE001
        print(f'  FAIL {what}\n       raised {type(e).__name__}: {e}')
        FAILURES.append(what)
    else:
        print(f'  ok   {what}')


def _argv(*extra):
    return [sys.executable, '-X', 'utf8', LOOP, BOARD, OUT,
            '--route-args', '--nets /*'] + list(extra)


def t_an_unknown_selector_names_the_ones_that_exist():
    check(_argv('--target-select', 'bogus'), code=2,
          refuse="invalid choice: 'bogus'",
          allow=('error: argument',))


def t_diagnosis_without_blocks_names_the_flag_that_fixes_it():
    r = check(_argv('--target-select', 'diagnosis'), code=2,
              refuse='--group-by none derives none')
    out = (r.stdout or '') + (r.stderr or '')
    assert '--group-by auto' in out, 'the refusal must name the fix, not just the fault'
    assert 'blocked cells and legality pairs' in out, (
        'and must say the other two signals do rank loose parts, or an '
        'operator reads it as "diagnosis needs blocks", which is false')


def t_the_refusal_happens_before_anything_is_routed():
    check(_argv('--target-select', 'diagnosis'), code=2,
          refuse='--group-by none derives none')
    # The output board is the weak half of this check -- a run that routed for
    # twenty minutes and then died would also leave it absent. The load-bearing
    # half is that the WORK DIR is untouched: round 0 writes loop_round0.kicad_pcb
    # into it before it routes anything.
    assert not os.path.exists(OUT), 'the refusal wrote a board'
    left = sorted(os.listdir(_TMP))
    assert left == [], (
        f'the refusal reached round 0 -- it left {left} in the work dir, and '
        f'it must land before the whole-board route these gates exist to save')


def t_a_zero_report_size_is_refused():
    check(_argv('--target-select', 'diagnosis', '--group-by', 'auto',
                '--diagnosis-top-k', '0'), code=2,
          refuse='--diagnosis-top-k must be >= 1')


def t_the_help_carries_the_no_efficacy_sentence():
    r = check([sys.executable, '-X', 'utf8', LOOP, '--help'], accept=True)
    # argparse REFLOWS help text to the terminal width, so the sentence is
    # split across lines at a width nobody controls. Collapse whitespace
    # before looking for it, or this assertion passes and fails by console.
    out = ' '.join((r.stdout or '').split())
    assert "NO MEASUREMENT SHOWS 'diagnosis' ROUTES BETTER THAN 'pins'" in out, (
        "the --help text must state that nothing measured 'diagnosis' as "
        'better than pins; an operator choosing the flag reads this first')
    assert "'pins'" in out and 'default' in out, 'and that pins is the default'


def t_auto_derives_no_block_on_the_boards_this_repo_grades_on():
    """The measurement the feature is shaped around, re-run rather than quoted.

    esp_prog, splitflap_driver, watchy, tigard and sonde_u are five of the six
    boards in docs/placement-predictors.md's declared table, and `auto`
    (kicad,sheet) derives NOTHING on ANY of the five -- they are flat schematics
    whose `(path ...)` entries are all distinct top-level uuids, and no corpus
    board carries a KiCad `(group ...)` at all. A blocks-only mover ranking
    would have been inert here.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.groups import derive_groups, parse_sources
    auto = parse_sources('auto')
    flat = []
    for name in ('esp_prog', 'splitflap_driver', 'watchy', 'tigard', 'sonde_u'):
        path = os.path.join(ROOT, 'kicad_files', f'{name}.kicad_pcb')
        if not os.path.isfile(path):
            continue
        pcb = parse_kicad_pcb(path)
        got = derive_groups(pcb, auto)
        if not got:
            flat.append(name)
        # decap is the source that DOES fire on these boards; if that ever
        # stops being true the census line in the loop becomes useless.
        # sonde_u is the sharp case and is named rather than waived: NO
        # source derives a block on it, so the displacement signal can never
        # run there under any --group-by. That is what the census before
        # round 0 exists to tell the operator.
        if name == 'sonde_u':
            assert not derive_groups(pcb, ('decap',)), (
                'sonde_u: decap derived a block; the census claim changed')
            assert len(derive_groups(pcb, ('netprefix',))) == 1, (
                'sonde_u was the board with exactly ONE derivable block, so a '
                'ranking over its candidates makes no choice; if that changed, '
                'the census claim changed with it')
        else:
            assert derive_groups(pcb, ('decap',)), (
                f'{name}: decap was the source that fires here; if it stopped, '
                f'the census line in the loop is telling the operator nothing')
    assert len(flat) == 5, (
        f'auto was expected to derive nothing on ALL FIVE of these boards; it '
        f'derived nothing on {flat}. If that changed, the reason the diagnosis '
        f'ranks loose parts needs re-examining, not deleting')


TESTS = [
    ('an unknown selector names the ones that exist',
     t_an_unknown_selector_names_the_ones_that_exist),
    ('diagnosis + --group-by none is refused, naming the fix',
     t_diagnosis_without_blocks_names_the_flag_that_fixes_it),
    ('and refused before round 0 routes anything',
     t_the_refusal_happens_before_anything_is_routed),
    ('--diagnosis-top-k 0 is refused',
     t_a_zero_report_size_is_refused),
    ('--help states that no measurement backs diagnosis',
     t_the_help_carries_the_no_efficacy_sentence),
    ('auto derives no block on the boards this repo grades on',
     t_auto_derives_no_block_on_the_boards_this_repo_grades_on),
]


def main():
    print(f'run_utils: {os.path.basename(run_utils.ROOT_DIR)}')
    for what, fn in TESTS:
        report(what, fn)
    shutil.rmtree(_TMP, ignore_errors=True)
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_553_diagnosis_cli: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
