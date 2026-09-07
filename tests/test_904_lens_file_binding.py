#!/usr/bin/env python3
"""A close-out verdict is bound to the file the verifier wrote (#904).

`references/verifier-prompts.md` has required every verifier to ALSO write its
`VERDICT=` line to disk since run 23 -- "a reply is a notification and
notifications get lost". Nothing ever read those files: the line was retyped
into `--lens` from a reply, so the ledger recorded a claim about the RUN where
it could have recorded a claim about a FILE. Measured (run 25): a close-out
inherited `VERDICT=PASS:lens=spec` from an earlier step, the end-to-end verifier
returned FAIL on the same clause two hours later, and the terminal row had to be
re-recorded -- an append-only ledger then holds two answers to one question,
which is not a record of a verdict but a record of a disagreement.

`--lens-file` is the reader, `entry["lens_source"]` is the provenance, and a
`--final --kind completion` row refuses a bare `--lens` for connectivity, drc or
spec.

The adversarial half is the selection rule. It takes the FIRST line beginning
`VERDICT=`, whatever that line says, so it cannot step past a malformed verdict
to a well-formed one further down -- the normalisation this toolchain forbids
everywhere else. Those cases are the ones worth reading below.

`run_utils.check` is used for the refusals rather than a bare
`assert returncode == 2`, so an ImportError or an argparse accident reports a
BROKEN TEST instead of reading as a guard that held (CLAUDE.md).
"""
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
import run_utils                                              # noqa: E402

CV = os.path.join(ROOT, 'py_placer', 'converge.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _write(td, name, text, encoding='utf-8'):
    p = os.path.join(td, name)
    with open(p, 'w', encoding=encoding) as fh:
        fh.write(text)
    return p


def _score(td, name='s.json', **over):
    doc = {'schema': 1, 'kind': 'board-score', 'blocking': 0, 'quality': {}}
    doc.update(over)
    return _write(td, name, json.dumps(doc))


def _final(td, led, *extra):
    return [sys.executable, '-X', 'utf8', CV, 'record', '--ledger', led,
            '--board', BOARD, '--kind', 'completion', '--final',
            '--stop-condition', 'DONE-EXHAUSTED', '--lever', 'close-out',
            '--score-file', _score(td)] + list(extra) + [
            '--argv', sys.executable, '-c', 'pass']


def _files(td, **verdicts):
    out = []
    for lens in ('connectivity', 'drc', 'spec'):
        out += ['--lens-file',
                _write(td, f'verdict_{lens}.txt',
                       verdicts.get(lens, f'VERDICT=PASS:lens={lens}') + '\n')]
    return out


def test_a_close_out_verdict_carries_its_file_and_hash():
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        args = _files(td)
        r = subprocess.run(_final(td, led, *args), capture_output=True,
                           text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr
        e = json.loads(r.stdout)
        assert e['lenses'] == ['VERDICT=PASS:lens=connectivity',
                               'VERDICT=PASS:lens=drc',
                               'VERDICT=PASS:lens=spec'], e['lenses']
        src = e['lens_source']
        assert len(src) == 3 and all(s for s in src), src
        from board_store import sha256_file
        for s, flag in zip(src, args[1::2]):
            assert s['path'] == flag, (s, flag)
            # BOTH paths. `path` is as the caller spelled it -- and the command
            # L5 prints is relative to the work dir, so on its own it is
            # unresolvable from anywhere else, which would make the hash
            # unverifiable in practice.
            assert s['abspath'] == os.path.abspath(flag), s
            # the WHOLE FILE, not the selected line: a verdict is its finding
            # and its evidence pointer as much as it is PASS or FAIL.
            assert s['sha256'] == sha256_file(flag), s
            assert s['line'] == 1, s

        # ...and the hash is LOAD-BEARING: rewriting only the finding text
        # around the verdict must break it.
        with open(args[1], 'a', encoding='utf-8') as fh:
            fh.write('the verifier adds a note\n')
        assert src[0]['sha256'] != sha256_file(args[1])
    print("  PASS: the row records path, abspath, sha256 and line per lens")


def test_a_bare_lens_is_refused_on_a_close_out_and_kept_on_a_lap():
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        bare = ['--lens', 'VERDICT=PASS:lens=connectivity',
                '--lens', 'VERDICT=PASS:lens=drc',
                '--lens', 'VERDICT=PASS:lens=spec']
        run_utils.check(_final(td, led, *bare),
                        refuse='no file behind them', code=2)
        assert not os.path.exists(led), 'nothing may be written on refusal'

        # ONE bare line among two files is still a refusal, and the message
        # names only the unsourced one.
        mixed = _files(td)[:4] + ['--lens', 'VERDICT=PASS:lens=spec']
        r = subprocess.run(_final(td, led, *mixed), capture_output=True,
                           text=True, cwd=ROOT)
        assert r.returncode == 2, r.stdout[:200]
        assert 'lens=spec' in r.stderr and 'lens=drc' not in r.stderr, r.stderr

        # A FAIL is refused for the same reason a PASS is: a STUCK close-out
        # normally carries one, so a PASS-only rule would leave the mechanism
        # unexercised on half the runs that print this command.
        failing = ['--lens', 'VERDICT=FAIL:lens=spec;finding=x;evidence=y',
                   '--lens', 'VERDICT=PASS:lens=connectivity',
                   '--lens', 'VERDICT=PASS:lens=drc']
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger', led,
             '--board', BOARD, '--kind', 'completion', '--final',
             '--stop-condition', 'STUCK', '--lever', 'x',
             '--score-file', _score(td)] + failing
            + ['--argv', sys.executable, '-c', 'pass'],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 2 and 'no file behind' in r.stderr, r.stderr

        # A LAP is untouched: its lenses are working notes, not a record.
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger', led,
             '--board', BOARD, '--kind', 'completion', '--lever', 'a lap',
             '--lens', 'VERDICT=PASS:lens=drc',
             '--argv', sys.executable, '-c', 'pass'],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr
        e = json.loads(r.stdout)
        assert e['lens_source'] == [None], e['lens_source']
    print("  PASS: a close-out needs files; a lap does not")


def test_the_first_verdict_line_wins_even_when_it_is_the_bad_news():
    """The selection rule, hostile cases first."""
    with tempfile.TemporaryDirectory() as td:
        # A FAIL above a PASS selects the FAIL. Anything else is a mechanism
        # for burying bad news below good news.
        p = _write(td, 'two.txt',
                   'the verifier re-derived every number.\n'
                   'VERDICT=FAIL:lens=spec;finding=impedance 1;evidence=s.json\n'
                   'VERDICT=PASS:lens=spec\n')
        led = os.path.join(td, 'l.jsonl')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger', led,
             '--board', BOARD, '--kind', 'completion', '--lever', 'x',
             '--lens-file', p, '--argv', sys.executable, '-c', 'pass'],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr
        e = json.loads(r.stdout)
        assert e['lenses'][0].startswith('VERDICT=FAIL'), e['lenses']
        assert e['lens_source'][0]['line'] == 2, e['lens_source']

        # A MALFORMED first verdict refuses; it must NOT be skipped in favour
        # of the well-formed line below it. Selecting by _LENS_RE instead of by
        # the VERDICT= prefix would silently normalise a broken verdict into
        # one that reads like a pass.
        bad = _write(td, 'bad.txt',
                     'VERDICT=MAYBE:lens=spec\nVERDICT=PASS:lens=spec\n')
        run_utils.check(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger',
             os.path.join(td, 'l2.jsonl'), '--board', BOARD,
             '--kind', 'completion', '--lever', 'x', '--lens-file', bad,
             '--argv', sys.executable, '-c', 'pass'],
            refuse='verbatim', code=2)

        # A BOM must not make a file that visibly holds a verdict report that
        # it holds none.
        bom = _write(td, 'bom.txt', 'VERDICT=PASS:lens=drc\n',
                     encoding='utf-8-sig')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger',
             os.path.join(td, 'l3.jsonl'), '--board', BOARD,
             '--kind', 'completion', '--lever', 'x', '--lens-file', bom,
             '--argv', sys.executable, '-c', 'pass'],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr

        # No VERDICT= line at all, and an unreadable path: two different
        # refusals, both writing nothing.
        none = _write(td, 'none.txt', 'the verifier could not be reached.\n')
        run_utils.check(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger',
             os.path.join(td, 'l4.jsonl'), '--board', BOARD,
             '--kind', 'completion', '--lever', 'x', '--lens-file', none,
             '--argv', sys.executable, '-c', 'pass'],
            refuse="no line beginning 'VERDICT='", code=2)
        assert not os.path.exists(os.path.join(td, 'l4.jsonl'))
        run_utils.check(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger',
             os.path.join(td, 'l5.jsonl'), '--board', BOARD,
             '--kind', 'completion', '--lever', 'x',
             '--lens-file', os.path.join(td, 'nope.txt'),
             '--argv', sys.executable, '-c', 'pass'],
            # `allow`: the refusal QUOTES the OSError, so its own text contains
            # the phrase run_utils treats as an accident. Naming it here is the
            # point of that parameter -- it says "this substring is expected in
            # this refusal" rather than switching the accident detector off.
            refuse='--lens-file unreadable', code=2,
            allow=('No such file or directory',))

        # The boundary verifier's grammar is NOT a lens: check=<1-5> carries no
        # lens name, so the row would record a verdict about nothing.
        chk = _write(td, 'record.txt', 'VERDICT=FAIL:check=3;finding=x\n')
        run_utils.check(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger',
             os.path.join(td, 'l6.jsonl'), '--board', BOARD,
             '--kind', 'completion', '--lever', 'x', '--lens-file', chk,
             '--argv', sys.executable, '-c', 'pass'],
            refuse='verbatim', code=2)
    print("  PASS: first VERDICT= line wins, malformed refuses, BOM is fine")


def test_the_film_captions_a_row_that_has_no_lever():
    """The film is "the ledger read out loud" -- so a row whose whole content
    is a human's reason may not be read out as silence."""
    sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
    import make_film
    from board_store import BoardStore
    with tempfile.TemporaryDirectory() as td:
        store = BoardStore(os.path.join(td, 'boards'))
        sha = store.put(BOARD)
        led = os.path.join(td, 'l.jsonl')
        rows = [
            {'iteration': 0, 'kind': 'placement', 'accepted': True,
             'result_sha': sha, 'lever': 'place_optimize --refs U1'},
            {'iteration': 1, 'kind': 'systemic', 'accepted': True,
             'result_sha': sha, 'lever': None,
             'exhausted': {'half': 'placement', 'reason': 'every lever spent'}},
            {'iteration': 2, 'kind': 'completion', 'accepted': True,
             'result_sha': sha, 'final': True, 'stop_condition': 'STUCK'},
        ]
        with open(led, 'w', encoding='utf-8') as fh:
            for row in rows:
                fh.write(json.dumps(row) + '\n')
        labels = [s.get('label') or '' for s in
                  make_film.shots_from_ledger(led, work=os.path.join(td, 'f'))]
        assert len(labels) == 3, labels
        assert 'place_optimize --refs U1' in labels[0], labels
        assert 'declared exhausted: every lever spent' in labels[1], labels
        assert 'close-out: STUCK' in labels[2], labels
    print("  PASS: the film captions a lever-less row with what it does say")


def test_a_final_row_of_any_kind_needs_its_files():
    """The gate is about the RECORD, so `--kind` cannot get you round it.

    `_cross_check`'s per-lens supersession takes the LATEST `--final` row that
    speaks to a lens. A one-lens `--kind systemic --final` row is trivially
    writable, so scoping the file requirement to `completion` would leave a
    bare `VERDICT=PASS:lens=spec` able to override a sourced FAIL -- the
    "one record, two rules depending on something orthogonal" shape #901 was
    filed about, rebuilt inside its own fix.
    """
    with tempfile.TemporaryDirectory() as td:
        for kind in ('systemic', 'placement', 'classification'):
            led = os.path.join(td, f'l_{kind}.jsonl')
            run_utils.check(
                [sys.executable, '-X', 'utf8', CV, 'record', '--ledger', led,
                 '--board', BOARD, '--kind', kind, '--final',
                 '--stop-condition', 'STUCK', '--lever', 'x',
                 '--lens', 'VERDICT=PASS:lens=spec',
                 '--argv', sys.executable, '-c', 'pass'],
                refuse='no file behind them', code=2)
            assert not os.path.exists(led), kind
        # ...and the sourced form of the same row is accepted, so the gate
        # refuses the missing FILE and not the kind.
        led = os.path.join(td, 'ok.jsonl')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger', led,
             '--board', BOARD, '--kind', 'systemic', '--final',
             '--stop-condition', 'STUCK', '--lever', 'x',
             '--lens-file', _write(td, 'verdict_spec.txt',
                                   'VERDICT=PASS:lens=spec\n'),
             '--argv', sys.executable, '-c', 'pass'],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr
        # A lens outside the three is not covered by the rule.
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger',
             os.path.join(td, 'other.jsonl'), '--board', BOARD,
             '--kind', 'systemic', '--final', '--stop-condition', 'STUCK',
             '--lever', 'x', '--lens', 'VERDICT=PASS:lens=assembly',
             '--argv', sys.executable, '-c', 'pass'],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr
    print("  PASS: a --final row's routed-board lens needs a file, any kind")


def test_a_file_that_is_not_text_says_so():
    """UnicodeDecodeError SUBCLASSES ValueError, so without its own branch a
    binary file was reported as 'no line beginning VERDICT=' -- an answer about
    the content of a file nothing could read."""
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'binary.txt')
        with open(p, 'wb') as fh:
            fh.write(b'VERDICT=PASS:lens=spec \xe9\xff\xfe not utf-8\n')
        run_utils.check(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger',
             os.path.join(td, 'l.jsonl'), '--board', BOARD,
             '--kind', 'completion', '--lever', 'x', '--lens-file', p,
             '--argv', sys.executable, '-c', 'pass'],
            refuse='is not UTF-8 text', code=2)
        assert not os.path.exists(os.path.join(td, 'l.jsonl'))
    print("  PASS: a non-text verdict file is named, not misdiagnosed")


def test_status_names_every_unlevered_row_on_an_ordinary_ledger():
    """#904's inherited item, on the ledger shape it actually happens on."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        rows = [{'iteration': i, 'kind': 'placement', 'accepted': True,
                 'lever': f'lap {i}'} for i in range(4)]
        rows.append({'iteration': 4, 'kind': 'systemic', 'accepted': True,
                     'lever': None,
                     'exhausted': {'half': 'placement',
                                   'reason': 'every lever spent'}})
        rows.append({'iteration': 5, 'kind': 'completion', 'accepted': True,
                     'final': True, 'stop_condition': 'STUCK'})
        with open(led, 'w', encoding='utf-8') as fh:
            for row in rows:
                fh.write(json.dumps(row) + '\n')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'status', '--ledger', led],
            capture_output=True, text=True, cwd=ROOT)
        assert r.returncode == 0, r.stderr
        doc = json.loads(r.stdout)          # STILL exactly one JSON document
        assert doc['unlevered'] == 2, doc
        assert doc['total'] == 6, doc
        # the systemic share is 1/6, so the old nesting printed NOTHING here
        assert 'SYSTEMIC' not in r.stderr, r.stderr
        assert 'i4  declared exhausted: every lever spent' in r.stderr, r.stderr
        assert 'i5  close-out: STUCK' in r.stderr, r.stderr
        assert 'lap 0' not in r.stderr, 'a levered row needs no itemisation'
    print("  PASS: status names the unlevered rows and stays one JSON doc")


class _NoConverge:
    """Make `import converge` fail, the way KiCad's plugin loader does.

    Without this the test is a TAUTOLOGY. `placement_run._row_label` tries the
    real import first and only falls back on ImportError -- and this file puts
    `py_placer` on sys.path, so the import SUCCEEDS and the assertion reduces to
    `converge.row_label(row) == converge.row_label(row)`. Measured: the copy
    could be replaced by `return "TOTALLY WRONG COPY"` with the test still
    green. A meta_path finder that refuses the module is the only way to reach
    the branch that exists for an environment this test is not running in.
    """

    def find_module(self, name, path=None):              # py2-era hook, unused
        return None

    def find_spec(self, name, path=None, target=None):
        if name == 'converge':
            raise ImportError('blocked: standing in for the plugin loader')
        return None


def test_the_gui_label_ladder_matches_converges():
    """`placement_run` cannot import converge inside KiCad's plugin loader, so
    it carries a copy. A copy drifts; this is what makes it not."""
    sys.path.insert(0, os.path.join(ROOT, 'kicad_routing_plugin'))
    import placement_run
    from converge import row_label
    rows = [
        ({'lever': 'place_optimize --refs U1'}, 'place_optimize --refs U1'),
        ({'lever': None,
          'exhausted': {'half': 'placement', 'reason': 'every lever spent'}},
         'declared exhausted: every lever spent'),
        ({'lever': '', 'final': True, 'stop_condition': 'STUCK'},
         'close-out: STUCK'),
        ({}, '(no lever recorded)'),
        # whitespace is not content, in either rung
        ({'lever': '  ', 'exhausted': {'half': 'routing', 'reason': '  '},
          'stop_condition': 'BUDGET'}, 'close-out: BUDGET'),
        # `exhausted` present but not a dict, and a non-string lever
        ({'lever': 0, 'exhausted': 'spent'}, '(no lever recorded)'),
        ({'lever': 12, 'stop_condition': None}, '12'),
    ]
    blocker = _NoConverge()
    sys.meta_path.insert(0, blocker)
    saved = sys.modules.pop('converge', None)
    try:
        # THE EXPECTED STRING, not just agreement: two implementations that are
        # both wrong agree perfectly, and this test's whole job is to be the
        # thing that notices.
        for row, want in rows:
            assert placement_run._row_label(row) == want, (row, want)
    finally:
        sys.meta_path.remove(blocker)
        if saved is not None:
            sys.modules['converge'] = saved
    # ...and with converge importable, the AUTHORITY answers the same.
    for row, want in rows:
        assert row_label(row) == want, (row, want)
        assert placement_run._row_label(row) == want, row
    assert placement_run.derive_stage(
        None, {'iteration': 31, 'kind': 'systemic', 'lever': None,
               'exhausted': {'half': 'placement', 'reason': 'spent'}},
        None) == 'lap 31: systemic/declared exhausted: spent'
    print("  PASS: one label ladder, checked against expected text, with the "
          "import blocked and allowed")


if __name__ == '__main__':
    run_utils.evidence(BOARD)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
