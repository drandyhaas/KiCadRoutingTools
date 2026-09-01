#!/usr/bin/env python3
"""#830: an unreadable route summary is a NON-VERDICT, not a crash.

`converge.scoped_route` json.load'ed route.py's `--json-out` with no guard, so
a file that EXISTS and does not parse raised JSONDecodeError straight out of
the function. That did not spoil one candidate -- it emptied the CALLER's loop:
`cmd_poses` has no try at all, and `place_portfolio._probe` and `compare_seeds`
catch only `subprocess.TimeoutExpired`. compare_seeds writes seeds.json only
AFTER its loop, so a throw on seed 3 of 8 shipped no document at all.

Pinned here:

  1. scoped_route RETURNS on an unreadable summary, with `summary` {} -- the
     no-verdict channel every caller already handles.
  2. It DISCLOSES which absence it was (`summary_error`, always present, None
     when the read was fine). The return code is 0 either way, so without this
     "wrote nothing" and "wrote garbage" are the same row.
  3. The complaint goes to STDERR: converge's stdout is a JSON data channel.
  4. route_verdict refuses a truthy-but-KEYLESS summary instead of scoring it
     0 failures / 'clean'. Hardening, not a live bug -- and the two boundaries
     it must not move are re-pinned here so the predicate cannot drift into
     them.
  5. route.py publishes --json-out all-or-nothing (serialise, temp, replace),
     so a failed write leaves the destination untouched rather than truncated.

No board is routed: route.py is replaced by a stub that writes a chosen payload
to whatever --json-out path scoped_route picked. Hence the FAST_OK opt-out from
run_all's "mentions subprocess => integration" classification.
"""
import ast
import contextlib
import io
import json
import os
import shutil
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

#: Exactly what a streaming json.dump leaves when it stops partway: a valid
#: PREFIX of the document, which `os.path.isfile` accepts and `json.load` does
#: not.
TRUNCATED = '{\n "failed_single": [\n  "A"\n ],\n "poison": '

#: The stub router. It reads its own argv to find whatever --json-out path
#: scoped_route chose, writes `PAYLOAD`, and exits 0 -- which is what route.py
#: does on this path: its `except` only prints a WARNING, and a route.py run
#: with failed nets exits 0 too.
_STUB = '''\
import sys
p = sys.argv[sys.argv.index('--json-out') + 1]
with open(p, 'w', encoding='utf-8') as f:
    f.write(PAYLOAD)
sys.exit(RC)
'''


@contextlib.contextmanager
def _router_writes(payload, rc=0):
    """Point converge at a stub route.py that writes `payload`."""
    td = tempfile.mkdtemp(prefix='t830_stub_')
    stub = os.path.join(td, 'route_stub.py')
    with open(stub, 'w', encoding='utf-8') as f:
        f.write(_STUB.replace('PAYLOAD', repr(payload)).replace('RC', str(rc)))
    real = converge._ROUTE_PY
    converge._ROUTE_PY = stub
    try:
        yield
    finally:
        converge._ROUTE_PY = real
        shutil.rmtree(td, ignore_errors=True)


def _scoped(payload, rc=0):
    with _router_writes(payload, rc):
        res = converge.scoped_route('no-such.kicad_pcb', ['GND'])
    # Verify the INPUT before trusting the output: a stub that wrote nothing
    # would exercise the missing-file branch and prove nothing at all.
    assert os.path.isfile(res['json']), (
        f"the stub wrote no file; this test would prove nothing: {res}")
    with open(res['json'], encoding='utf-8') as f:
        raw = f.read()
    shutil.rmtree(os.path.dirname(res['json']), ignore_errors=True)
    return res, raw


# ------------------------------------------------------- 1-3: the reader guard

def test_a_truncated_summary_returns_instead_of_raising():
    res, raw = _scoped(TRUNCATED)
    try:
        json.loads(raw)
    except ValueError:
        pass
    else:
        raise AssertionError('the stub wrote parseable JSON; repro is void')
    assert res['summary'] == {}, res['summary']
    assert 'JSONDecodeError' in (res['summary_error'] or ''), res
    assert converge.route_verdict(res['summary']) == (None, 'no summary')
    print('  PASS: a truncated route.json degrades to no-summary, not a crash')


def test_an_empty_file_lands_in_the_same_channel():
    """A router killed before its first byte, not one that wrote nothing."""
    res, _ = _scoped('')
    assert res['summary'] == {} and res['summary_error'], res
    print('  PASS: an empty summary file is disclosed, not silently empty')


def test_a_readable_summary_carries_the_key_as_None():
    """The key is UNCONDITIONAL: a row's shape must not depend on its outcome,
    or a consumer has to guess whether the absence of the key means success."""
    res, _ = _scoped(json.dumps({'failed_single': [], 'open_single': [],
                                 'multipoint_pads_total': 0,
                                 'multipoint_pads_connected': 0}))
    assert 'summary_error' in res, f'key missing on the happy path: {res}'
    assert res['summary_error'] is None, res['summary_error']
    assert converge.route_verdict(res['summary']) == (0, 'clean')
    print('  PASS: summary_error is present and None on a good read')


def test_the_complaint_is_on_stderr_because_stdout_is_a_data_channel():
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        res, _ = _scoped(TRUNCATED)
    assert out.getvalue() == '', (
        f'stdout carries the `poses` JSON document: {out.getvalue()!r}')
    assert 'JSONDecodeError' in err.getvalue(), err.getvalue()
    assert res['json'] in err.getvalue(), 'the warning must name the file'
    print('  PASS: the unreadable-summary warning goes to stderr')


def test_the_guard_catches_oserror_too_not_just_a_parse_failure():
    """json.load decodes the handle FIRST, so a tail cut mid-UTF-8-sequence
    raises UnicodeDecodeError -- a ValueError that is NOT a JSONDecodeError.
    The narrower `except json.JSONDecodeError` the issue suggested misses it."""
    src = ast.parse(open(converge.__file__, encoding='utf-8').read())
    fn = next(n for n in ast.walk(src)
              if isinstance(n, ast.FunctionDef) and n.name == 'scoped_route')
    caught = {i.id for h in ast.walk(fn) if isinstance(h, ast.ExceptHandler)
              for i in ast.walk(h.type) if isinstance(i, ast.Name)}
    assert {'OSError', 'ValueError'} <= caught, (
        f'scoped_route must catch (OSError, ValueError); catches {caught}')
    # and prove the wider clause is not decoration
    payload = b'{"failed_single": ["\xe2\x82'          # cut mid-UTF-8
    td = tempfile.mkdtemp(prefix='t830_utf8_')
    p = os.path.join(td, 'route.json')
    with open(p, 'wb') as f:
        f.write(payload)
    try:
        with open(p, encoding='utf-8') as f:
            json.load(f)
    except ValueError as exc:
        assert not isinstance(exc, json.JSONDecodeError), (
            'pick a payload that really decodes badly')
    else:
        raise AssertionError('the cut sequence parsed; repro is void')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print('  PASS: the guard is wide enough for a bad decode, not just bad JSON')


# ------------------------------------------------ 4: the route_verdict tripwire

def test_route_verdict_refuses_a_summary_with_none_of_its_keys():
    assert converge.route_verdict({'x': 1}) == (None, 'unreadable summary')
    # protected_skipped only decorates the note -- alone it is not a verdict,
    # so it must NOT be enough to unlock the arithmetic.
    assert converge.route_verdict(
        {'protected_skipped': {'--rip-existing-nets': {'GND': 'locked'}}}) == (
        None, 'unreadable summary')
    print('  PASS: a truthy but keyless summary no longer scores as clean')


def test_the_guard_moves_neither_boundary_it_must_not():
    """Both are pinned by existing tests; re-pinned here because the predicate
    that refuses a keyless dict is one edit away from refusing these."""
    # tests/test_converge.py: an empty summary keeps its own note
    assert converge.route_verdict({}) == (None, 'no summary')
    # tests/test_open_single_verdict.py: ONE key present is enough to judge --
    # a log predating open_single must degrade to the old arithmetic, not
    # be refused.
    assert converge.route_verdict({'failed_single': ['A']})[0] == 1
    # a clean route that happens to carry only the multipoint keys
    assert converge.route_verdict({'multipoint_pads_total': 4,
                                   'multipoint_pads_connected': 4}) == (
        0, 'clean')
    print('  PASS: the presence test refuses only documents with no verdict')


# ------------------------------------------------------------ 5: the publisher

def test_a_failed_write_leaves_the_destination_untouched():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'route.json')

        # a payload that cannot serialise must create NOTHING -- not the
        # destination (a reader checks os.path.isfile) and no temp litter
        try:
            write_summary_file(path, {'failed_single': ['A'],
                                      'poison': {1, 2}, 'tail': 'x'})
        except TypeError:
            pass
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

        # a LATER failed write must not disturb what is already published
        try:
            write_summary_file(path, {'poison': {1, 2}})
        except TypeError:
            pass
        with open(path, encoding='utf-8') as f:
            assert f.read() == published, (
                'a failed write disturbed the previously published file')
        assert os.listdir(td) == ['route.json'], os.listdir(td)
    print('  PASS: --json-out is whole or absent, never half')


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


def test_cmd_poses_route_row_says_what_happened():
    """The row is hand-built (nothing in the repo reads it, so this is the only
    thing holding its shape). AST, not grep: prose could satisfy a text match."""
    src = ast.parse(open(converge.__file__, encoding='utf-8').read())
    fn = next(n for n in ast.walk(src)
              if isinstance(n, ast.FunctionDef) and n.name == 'cmd_poses')
    rows = [n.value for n in ast.walk(fn) if isinstance(n, ast.Assign)
            and isinstance(n.value, ast.Dict)
            and any(isinstance(k, ast.Constant) and k.value == 'failures'
                    for k in n.value.keys)]
    assert len(rows) == 1, f'expected one route row literal, found {len(rows)}'
    keys = {k.value for k in rows[0].keys if isinstance(k, ast.Constant)}
    assert {'failures', 'note', 'iterations', 'vias', 'nets', 'returncode',
            'summary_error'} <= keys, (
        f'the poses route row cannot say what happened; keys={sorted(keys)}')
    print('  PASS: cmd_poses reports nets/returncode/summary_error')


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
