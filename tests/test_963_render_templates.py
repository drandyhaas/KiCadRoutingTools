"""A render that writes its keys to a file must not also print them (#963).

Run 29 built 13 review sheets and 10 of them printed every checklist key before
the picture could be looked at. The blind-first ordering is the whole mechanism
of the boundary review -- `SKILL.md`: "VIEW it, write your observations ...
**before reading any checklist key**" -- and the commands the review gates
prescribe were the ones defeating it, because ten of the thirteen render
templates the two drivers emit omitted `--quiet`.

TWO SEPARATE DEFECTS, and the second is the one the issue does not name:

  * `--quiet` missing. Measured below, on a real board: the flag takes stdout
    from tens of lines carrying `checklist` to three that carry none, and both
    forms still write the PNG and the JSON. Nothing is silenced into nowhere.
  * `--review-sheet` missing, which made a live guard unable to fire.
    `placement_driver._guard_render`'s fifth check reads
    `doc.get('review_sheet') if 'review_sheet' in doc else ''` and treats an
    ABSENT key as "never asked for" -- correctly, for a render produced
    elsewhere. But `render_placement` writes that key only under
    `if args.review_sheet:`, and no template passed the flag, so the guard
    could not fire on a single render the driver's own text produced. Its
    self-test reached it by hand-injecting the key. That is a gate whose
    population was empty, which looks exactly like a gate with nothing to
    report.

The rule itself -- `--json-out` implies `--quiet` unless `--pair` or `--focus`
makes the narrative the deliverable -- is held over every emitted command by
`tests/test_431_skill_commands.py::
test_a_render_that_writes_its_keys_to_a_file_does_not_print_them`, which shares
that file's `--dump-all`/`--dump-refusals` cache. This file is the part that
has to RUN something.
"""
import io
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import run_utils                                               # noqa: E402

RENDER = os.path.join(ROOT, 'py_tools', 'render_placement.py')
PLACE_DRIVER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                            'scripts', 'placement_driver.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _run(argv, timeout=600):
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, timeout=timeout)


def test_quiet_takes_the_keys_off_stdout_and_keeps_both_files():
    """The measurement the issue reports, re-derived rather than quoted.

    #963 measured 36 lines / 8,513 characters without the flag and 3 / 536
    with it, on `tests/fixtures/run23/tigard_placed.kicad_pcb` -- the fixture
    its addendum names, NOT run 29's board. And the CHARACTER counts are not
    reproducible even there: the stdout carries the temp paths it was given,
    so the same command in a different directory prints a different number.
    The LINE counts survive that, and what has to hold on any board is the
    SHAPE -- the checklist leaves stdout, stdout shrinks, and neither file
    goes missing.
    """
    run_utils.evidence(BOARD, 'the fixture board')
    with tempfile.TemporaryDirectory() as td:
        outs = {}
        for name, extra in (('loud', []), ('quiet', ['--quiet'])):
            png = os.path.join(td, name + '.png')
            doc = os.path.join(td, name + '.json')
            r = _run([RENDER, BOARD, '--json-out', doc, '-o', png] + extra)
            assert r.returncode == 0, r.stderr[-400:]
            run_utils.evidence(png, name + ' png')
            run_utils.evidence(doc, name + ' json')
            outs[name] = (r.stdout, json.load(io.open(doc, encoding='utf-8')))

        loud, quiet = outs['loud'][0], outs['quiet'][0]
        assert 'checklist' in loud, (
            'the fixture no longer prints the keys, so this measures nothing')
        assert 'checklist' not in quiet, quiet[:400]
        assert len(quiet.splitlines()) < len(loud.splitlines()), (
            len(quiet.splitlines()), len(loud.splitlines()))
        assert len(quiet) < len(loud) / 2, (len(quiet), len(loud))
        # The DOCUMENT is identical in the part that matters: --quiet moves the
        # keys, it does not withhold them. (`instrument` carries the paths,
        # which differ by construction.)
        assert outs['loud'][1]['checklist'] == outs['quiet'][1]['checklist']
        print(f"  PASS: stdout {len(loud)} -> {len(quiet)} chars, "
              f"checklist off stdout, both files written")


def test_a_bare_quiet_would_be_the_worst_of_both():
    """Why the gate also refuses `--quiet` without `--json-out`.

    `render_placement` gates the JSON echo on `args.quiet and args.json_out`
    and the prose on `args.quiet` alone, so a bare `--quiet` silences the
    narrative while the keys still print -- the exact inversion of what the
    flag is for. Pinned here because the rule in test_431 would otherwise read
    as an arbitrary second clause.
    """
    with tempfile.TemporaryDirectory() as td:
        png = os.path.join(td, 'b.png')
        r = _run([RENDER, BOARD, '--json', '-o', png, '--quiet'])
        assert r.returncode == 0, r.stderr[-300:]
        assert 'JSON_SUMMARY' in r.stdout, (
            'a bare --quiet no longer prints the keys -- if that changed on '
            'purpose, this test and the test_431 clause are both stale')
    print("  PASS: --quiet without --json-out still prints the keys")


def test_the_guard_can_fire_on_the_drivers_own_render():
    """The recipe the driver hands a reader must produce the key it checks.

    Reachability is the thing nobody asserted, and it is why the guard died:
    every test of it injected the key by hand, so the guard passed its own
    tests while being unable to see a single real render.
    """
    r = _run([PLACE_DRIVER, '--dump-refusals'], timeout=900)
    assert r.returncode == 0, r.stderr[-400:]
    blocks = [b for b in r.stdout.split('python3')
              if 'render_placement.py' in b and '--json-out wk/render.json' in b]
    assert blocks, 'the no-render recipe is no longer emitted at all'
    assert any('--review-sheet' in b for b in blocks), (
        'the recipe _guard_render hands a reader with no render yet does not '
        'ask for a review sheet, so the document it produces carries no '
        '`review_sheet` key and the fifth check cannot fire on it')

    # ...and a render of that SHAPE really does carry the key.
    with tempfile.TemporaryDirectory() as td:
        doc = os.path.join(td, 'render.json')
        out = _run([RENDER, BOARD, '--review-sheet',
                    os.path.join(td, 'sheet.png'), '--json-out', doc,
                    '-o', os.path.join(td, 'r.png'), '--quiet'])
        assert out.returncode == 0, out.stderr[-400:]
        run_utils.evidence(doc, 'the render document')
        got = json.load(io.open(doc, encoding='utf-8'))
        assert 'review_sheet' in got, sorted(got)[:20]
        assert got['review_sheet'], (
            'the sheet was requested and the key is falsy, which is the '
            'REFUSING shape -- the guard would reject its own recipe')
    print("  PASS: the driver's own recipe produces the key its guard reads")


def test_the_self_test_no_longer_claims_the_arm_is_unreachable():
    """A comment that outlives its fact is how the next reader re-derives it.

    The placement driver's self-test said the fifth check "can only be seen by
    feeding the two shapes that mean something, or it is an arm nothing
    exercises". After the templates pass `--review-sheet` that is false, and a
    reader who believes it will not look for the real population.
    """
    src = io.open(PLACE_DRIVER, encoding='utf-8').read()
    assert 'or it is an arm nothing exercises' not in src, (
        'the stale claim is back in placement_driver.py')
    assert 'could not fire on its own population' in src, (
        'the correction that replaced it is gone; say what changed, or the '
        'next reader re-derives the wrong conclusion')
    print("  PASS: the self-test says what is true of the guard now")



def test_an_unwritable_sheet_still_leaves_the_document_and_still_exits_2():
    """The headline of this item's render half, on the path with no --gate.

    Deferring the sheet failure until after `json.dump` is the whole change:
    one unwritable sheet path used to cost the caller the render DOCUMENT --
    `--json-out` never reached its dump, so a run had a PNG, an exit 2 and
    nothing for any downstream gate to read. A verifier mutated
    `return _sheet_exit or 0` to `return 0` and nothing failed, because every
    test here read the document and none read the EXIT on that branch: the
    #898 pin covers the `--gate` branch (`_sheet_exit or 4`) and this one was
    uncovered. Both halves are asserted here -- the document exists AND the
    exit is still 2 -- because either alone passes for the wrong reason.
    """
    with tempfile.TemporaryDirectory() as td:
        doc = os.path.join(td, 'view.json')
        png = os.path.join(td, 'view.png')
        # A directory that does not exist: render_placement cannot write the
        # sheet there and says so.
        sheet = os.path.join(td, 'no', 'such', 'dir', 'sheet.png')
        r = _run([RENDER, BOARD, '--review-sheet', sheet, '--json-out', doc,
                  '-o', png, '--quiet'])
        assert r.returncode == 2, (
            f'an unwritable --review-sheet exited {r.returncode}, not 2 -- '
            f'the caller asked for an artifact and did not get one\n'
            + (r.stdout + r.stderr)[-500:])
        assert 'review-sheet' in r.stderr, r.stderr[-400:]
        assert os.path.isfile(doc), (
            'the render document was not written: the sheet failure is back '
            'in front of the json.dump, which is the defect this fixed')
        d = json.load(io.open(doc, encoding='utf-8'))
        assert 'checklist' in d, sorted(d)
    print("  PASS: the document survives an unwritable sheet, and 2 still "
          "reaches the caller")


if __name__ == '__main__':
    run_utils.evidence(BOARD)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print("--- " + k)
            v()
    print("ALL PASS")
