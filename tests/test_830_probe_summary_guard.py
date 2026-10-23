#!/usr/bin/env python3
"""#830: a route summary is published whole or not at all, and read as such.

A `--json-out` file that EXISTS and does not parse used to be the normal
result of a failed write: batch_route streamed json.dump into the handle, so
a write that stopped partway left a valid PREFIX of the document, which
`os.path.isfile` accepts and `json.load` does not.

Pinned here:

  1. route.py publishes --json-out all-or-nothing (serialise, temp, replace),
     so a failed write leaves the destination untouched rather than truncated.
  2. `converge record` refuses a --score-file it cannot parse before anything
     is written, rather than crashing after the board is stored.

No board is routed. Hence the FAST_OK opt-out from run_all's "mentions
subprocess => integration" classification.
"""
import ast
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import converge                                              # noqa: E402
from route_summary import write_summary_file                 # noqa: E402

RUN_ALL_FAST_OK = True

# ------------------------------------------------------------- 1: the publisher

def test_a_failed_write_leaves_the_destination_untouched():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'route.json')

        # a payload that cannot serialise must create NOTHING -- not the
        # destination (a reader checks os.path.isfile) and no temp litter
        try:
            write_summary_file(path, {'failed_single': ['A'],
                                      'poison': {1, 2}, 'tail': 'x'})
        except OSError as exc:
            assert 'not JSON serializable' in str(exc), exc
            assert isinstance(exc.__cause__, TypeError), (
                'the original cause must be chained, not swallowed')
        else:
            raise AssertionError('an unserialisable summary must raise')
        assert not os.path.exists(path), (
            'a failed write must not create the destination: the reader '
            'checks os.path.isfile and would then crash on json.load')
        assert os.listdir(td) == [], f'temp litter survived: {os.listdir(td)}'

        # a good payload publishes, byte-identical to the streaming writer
        good = {'failed_single': [], 'open_single': [], 'total_iterations': 3}
        write_summary_file(path, good)
        with open(path, encoding='utf-8') as f:
            published = f.read()
        buf = io.StringIO()
        json.dump(good, buf, indent=1)
        assert published == buf.getvalue(), (
            'the on-disk format changed; --json-out is a published contract')

        # a LATER failed write must REMOVE what is already published, not
        # preserve it. A previous run's summary standing at this run's path is
        # a complete, parseable document with no signal that it is stale --
        # every reader accepts it as this run's. Preserving it was the first
        # version of this function's real defect, and this assertion is the
        # inversion of what that version asserted.
        try:
            write_summary_file(path, {'poison': {1, 2}})
        except OSError:
            pass
        assert not os.path.exists(path), (
            'a failed write left the previous run\'s summary standing in for '
            'this one')
        assert os.listdir(td) == [], os.listdir(td)
        assert published  # the first write really did publish
    print('  PASS: --json-out is whole or absent, never half')


def test_a_write_that_fails_AFTER_the_temp_file_still_leaves_it_untouched():
    """The ATOMICITY half, which the test above cannot see.

    A payload that will not serialise never opens anything, so
    serialise-first alone passes that check -- a writer that wrote straight
    into the destination would too. What separates them is a failure BETWEEN
    the write and the publish, which is exactly the reachable case here:
    ENOSPC, a flush that fails at close, or -- measured in this repo, at
    predictor_study.py:271 -- os.replace raising PermissionError [WinError 5]
    on Windows because another process holds the destination open.

    So: force os.replace to fail and require the destination to keep the bytes
    it already had. A non-atomic writer has overwritten them by this point.
    """
    real_replace = os.replace
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'route.json')
        first = {'failed_single': [], 'open_single': [], 'total_iterations': 1}
        write_summary_file(path, first)
        with open(path, encoding='utf-8') as f:
            before = f.read()

        def _boom(src, dst):
            raise OSError(5, 'Access is denied')

        os.replace = _boom
        err = None
        try:
            write_summary_file(path, {'failed_single': ['LATER'],
                                      'total_iterations': 999})
        except OSError as exc:
            err = exc
        else:
            raise AssertionError('a failed publish must not report success')
        finally:
            os.replace = real_replace
        assert before, 'the stale document was never published; repro is void'

        # FAIL CLOSED, not fail stale. The destination must not still hold the
        # PREVIOUS run's summary: that is a complete, parseable document every
        # reader accepts as this run's, with no signal anywhere -- strictly
        # worse than the truncated file this replaced, which announces itself.
        assert not os.path.exists(path), (
            'a failed publish left the previous run\'s summary in place; a '
            'reader would score this run against the last one\'s tally')
        assert os.listdir(td) == [], (
            f'the temp file survived a failed publish: {os.listdir(td)}')
        assert 'no file was left behind' in str(err), (
            f'the error must say what state the destination is in: {err}')
    print('  PASS: a publish that fails mid-flight leaves NO file, and says so')


def test_the_publish_is_a_rename_from_a_different_file():
    """The atomicity property itself, which neither test above can see.

    Once a failed publish removes the destination, `tmp = path` reaches the
    same END STATE as the atomic version by every observable this file has --
    both leave no file. What actually differs is what a CONCURRENT READER can
    observe mid-write, which no single-process test can catch by racing it.
    So assert the mechanism instead: the publish must be a rename INTO the
    destination from somewhere else. A writer that streams into the
    destination renames it onto itself, and that is exactly the mutant.
    """
    real_replace, seen = os.replace, []

    def _record(src, dst):
        seen.append((src, dst))
        return real_replace(src, dst)

    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'route.json')
        os.replace = _record
        try:
            write_summary_file(path, {'failed_single': []})
        finally:
            os.replace = real_replace
    assert seen, 'the publish never went through os.replace at all'
    src, dst = seen[-1]
    assert dst == path, f'published to {dst}, not {path}'
    assert src != path, (
        'the summary was renamed onto itself -- it was written straight into '
        'the destination, so a reader can see it half-written')
    print('  PASS: the summary is published by renaming a sibling into place')


def test_none_publishes_an_empty_document_as_the_old_writer_did():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'route.json')
        write_summary_file(path, None)
        with open(path, encoding='utf-8') as f:
            assert json.load(f) == {}, 'merge_summaries(None) must write {}'
    print('  PASS: a None merge still writes an empty document')


def test_batch_route_no_longer_streams_into_json_out():
    """A SOURCE guard: the write block only runs at the end of a real route, so
    no behavioural test of batch_route is cheap enough to catch a revert. This
    walks the AST rather than grepping text, because a comment quoting the old
    call would satisfy a grep."""
    src = ast.parse(open(os.path.join(ROOT, 'py_router', 'route.py'),
                         encoding='utf-8').read())
    fn = next(n for n in ast.walk(src)
              if isinstance(n, ast.FunctionDef) and n.name == 'batch_route')
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call)]
    names = {c.func.id for c in calls if isinstance(c.func, ast.Name)}
    assert 'write_summary_file' in names, (
        'batch_route must publish --json-out through '
        'route_summary.write_summary_file (serialise, temp file, os.replace)')
    # and no json.dump(...) whose second argument is a file handle opened on
    # json_out -- i.e. no streaming dump left anywhere in the function
    dumps = [c for c in calls if isinstance(c.func, ast.Attribute)
             and c.func.attr == 'dump']
    for c in dumps:
        seg = ast.unparse(c)
        assert '_jf' not in seg, (
            f'batch_route still streams json.dump into the --json-out handle: '
            f'{seg}')
    print('  PASS: batch_route publishes --json-out atomically')


# ---------------------------------------------------------------- 2: the reader

def test_record_refuses_a_score_payload_it_cannot_parse():
    """The SECOND unguarded json.loads in this file, and a live one.

    `cmd_record` parsed the score three times: twice inside `except Exception:
    ... = None` swallowers, then once unguarded while building the ledger
    entry. Neither swallow proves the string parses, so a truncated
    --score-file reached the third parse and crashed with a traceback at exit
    1 -- AFTER store.put() had written the board into the content store. It
    now refuses at the top, which is the posture the surrounding comment
    already claims ("Refuse before anything is written").
    """
    import shutil
    import subprocess
    board_src = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    if not os.path.isfile(board_src):
        print('  SKIP: fixture missing')
        return
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'b.kicad_pcb')
        shutil.copy(board_src, board)
        score = os.path.join(td, 'score.json')
        with open(score, 'w', encoding='utf-8') as f:
            f.write('{\n "blocking": 3,\n "board_sha": ')   # a valid PREFIX
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', converge.__file__, 'record',
             '--ledger', os.path.join(td, 'ledger'), '--board', board,
             '--kind', 'completion', '--score-file', score],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT)
        both = (r.stdout or '') + (r.stderr or '')
        assert 'Traceback' not in both, f'record still crashes:\n{both[-800:]}'
        assert r.returncode == 2, f'expected a refusal (2), got {r.returncode}'
        assert 'not readable JSON' in both, both[-400:]
        assert 'Nothing was written' in both, both[-400:]
    print('  PASS: record refuses an unparseable score before writing anything')


def main():
    bad = []
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_'):
            continue
        print(f'--- {name}')
        try:
            fn()
        except Exception as exc:                             # noqa: BLE001
            bad.append(f'{name}: {type(exc).__name__}: {exc}')
            print(f'  FAIL: {bad[-1]}')
    if bad:
        print(f'\n{len(bad)} FAILED')
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
