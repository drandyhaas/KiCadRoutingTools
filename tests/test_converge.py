#!/usr/bin/env python3
"""converge.py: the run ledger (`record`), its stop verdict (`verdict`) and
`where`.

A run used to keep its history as prose: a step back meant reading it and
reconstructing a board by hand, and a stop was whatever the last message said.
`record` stores every board by content beside a row saying what produced it,
and `verdict` reads those rows to say continue, or which kind of stop.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
# #522 reorg: the engine moved to py_router/, the placer to py_placer/, and
# board_score.py lives in py_tools/. Tests that shell out to or import them
# need those roots on sys.path.
for _p in ('py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split

import converge  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _cv(args, **kw):
    return subprocess.run([sys.executable, '-X', 'utf8',
                           os.path.join(ROOT, 'py_placer', 'converge.py')] + args,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, **kw)


def _lens_files(td, **verdicts):
    """--lens-file flags for the routed-board lenses, written to `td` (#904).

    A `--final --kind completion` row may not carry a BARE `--lens` for
    connectivity, drc or spec: a close-out is the terminal record, nothing
    reopens a ledger, and a line retyped from a reply is a claim about the run
    where the row could carry a claim about a file. So every close-out arm in
    this file writes the verdict where the verifier is already required to
    write it (the plan-pcb-placement verifier brief) and passes the path.

    Defaults to a clean PASS for all three; name a lens to override, e.g.
    `_lens_files(td, drc='VERDICT=FAIL:lens=drc;finding=short;evidence=x')`.
    """
    out = []
    for lens in ('connectivity', 'drc', 'spec'):
        line = verdicts.get(lens, f'VERDICT=PASS:lens={lens}')
        p = os.path.join(td, f'verdict_{lens}.txt')
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(line + '\n')
        out += ['--lens-file', p]
    return out


def _lens_file(td, line, name='verdict_lens.txt'):
    """--lens-file flags for ONE verdict line, written to `td`."""
    p = os.path.join(td, name)
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(line + '\n')
    return ['--lens-file', p]


def _score_file(td, doc, name='score.json'):
    """--score-file flags for one score document, written to `td`."""
    p = os.path.join(td, name)
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(doc if isinstance(doc, str) else json.dumps(doc))
    return ['--score-file', p]


def test_record_stores_the_board_by_content():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', BOARD,
                 '--lever', 'seed'])
        assert r.returncode == 0, r.stderr
        sha = json.loads(r.stdout)['result_sha']
        from board_store import sha256_file
        stored = os.path.join(td, 'boards', sha, 'board.kicad_pcb')
        assert sha256_file(stored) == sha, "the stored board must be byte-exact"
    print("  PASS: record stores the board, byte-exact, under its sha")


def test_record_final_requires_stop_condition():
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final'])
        assert r.returncode == 2 and 'stop-condition' in r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"
        # A completion --final now also needs the routed-board lens verdicts:
        # `blocking == 0` and "every lens passes" are two different claims and
        # only the first ever had a number, so a close-out could be written
        # with no lens dispatched at all.
        # #901: the stop condition is a TOKEN and is checked on EVERY record,
        # not only when a lens failed. This arm used to pass the bare prose
        # 'plateau: 3 iterations, no new copper' and assert it round-tripped
        # into the row -- i.e. it pinned the bug. The plateau IS stop condition
        # 3 (convergence.md S3), and the prose is now its reason.
        STOP = '3: plateau, 3 iterations, no new copper'
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                 '--stop-condition', STOP])
        assert r.returncode == 2 and 'routed-board lenses' in r.stderr, r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                 '--stop-condition', 'plateau: 3 iterations, no new copper'])
        assert r.returncode == 2 and 'stop condition' in r.stderr, r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"
        lenses = _lens_files(td)
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                 '--stop-condition', STOP] + lenses)
        assert r.returncode == 0, r.stderr
        e = json.loads(r.stdout)
        assert e.get('final') is True
        assert e.get('stop_condition') == '3', e.get('stop_condition')
        assert e.get('stop_reason') == 'plateau, 3 iterations, no new copper', \
            e.get('stop_reason')
        assert len(e.get('lenses') or []) == 3, e.get('lenses')
    print("  PASS: --final without --stop-condition is refused; with it, recorded")


def test_record_final_wants_the_lens_verdicts():
    """A FAILED lens means `blocking` was not really zero, so the run did not
    finish clean -- stop condition 1 is then not available to it."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--lever', 'x']
                + _lens_file(td, 'VERDICT=all three passed'))
        assert r.returncode == 2 and 'per-lens verdict' in r.stderr, r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"

        base = (['record', '--ledger', led, '--board', BOARD, '--final']
                + _lens_files(
                    td, drc='VERDICT=FAIL:lens=drc;finding=short;evidence=x'))
        r = _cv(base + ['--stop-condition', '1'])
        assert r.returncode == 2 and 'lens FAILED' in r.stderr, r.stderr
        # Stop condition 4 is "measured-unfixable", and since #963 the claim
        # needs the measurement on the record. This test uses 4 as a
        # FAIL-compatible TOKEN, so it supplies the decision and carries on.
        assert _cv(['record', '--ledger', led, '--board', BOARD,
                    '--kind', 'classification', '--shape', 'placement',
                    '--lever', 'no lane exists at the escape faces']
                   ).returncode == 0
        r = _cv(base + ['--stop-condition', '4'])
        assert r.returncode == 0, r.stderr
        assert json.loads(r.stdout).get('lenses')[1].startswith('VERDICT=FAIL')
    print("  PASS: lens grammar is enforced, and a FAIL forbids condition 1")


def test_record_refuses_a_lens_verdict_its_own_score_contradicts():
    """D1. `--lens` was validated for FORMAT and never against anything.

    Measured (run 17, ledger iteration 21): a --final row carrying
    VERDICT=PASS:lens=connectivity on a score reporting 32 unrouted nets and 47
    broken joins, while route.log -- 2.65 MB, 19 JSON_SUMMARY blocks -- held
    zero VERDICT= lines, because no verifier had ever run. That row passed L3,
    L4 and L5 untested and was corrected only when a human challenged it.
    """
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')

        def sc(doc, name):
            p = os.path.join(td, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        # The measured row, to the number.
        run17 = sc({'blocking': 79, 'quality': {},
                    'blocking_by': {'unrouted': 32, 'broken': 47, 'drc': 0,
                                    'undersized': 0}}, 's.json')
        final = (['record', '--ledger', led, '--board', BOARD, '--final',
                  '--kind', 'completion', '--stop-condition', '1',
                  '--score-file', run17] + _lens_files(td))
        r = _cv(final)
        assert r.returncode == 2, (r.returncode, r.stdout[:400])
        assert 'CONTRADICTS' in r.stderr, r.stderr
        assert 'unrouted = 32' in r.stderr and 'broken = 47' in r.stderr, r.stderr
        assert 'VERDICT=FAIL:lens=connectivity' in r.stderr, \
            'the refusal must name the honest verdict it will accept'
        assert not os.path.exists(led), 'nothing may be written on refusal'

        # ...and the honest verdict IS accepted (stop condition 4: measured
        # unfixable and said so) -- once the measurement is on the record,
        # which since #963 is what 4 means.
        assert _cv(['record', '--ledger', led, '--board', BOARD,
                    '--kind', 'classification', '--shape', 'placement',
                    '--lever', '32 nets need a lane no parameter creates']
                   ).returncode == 0
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                 '--kind', 'completion', '--stop-condition', '4',
                 '--score-file', run17]
                + _lens_files(td, connectivity=(
                    'VERDICT=FAIL:lens=connectivity;finding=32 nets carry no '
                    'copper;evidence=score.json#/blocking_by')))
        assert r.returncode == 0, r.stderr

        # CONSERVATIVE: an UNGRADED component is not a contradiction. This is
        # the half that keeps the check from refusing correct rows -- most
        # boards ship with impedance/length/net_widths null.
        # (fresh ledgers below: a second row in the SAME half would also be
        # tested for commensurability, which is a different gate)
        r = _cv(['record', '--ledger', os.path.join(td, 'l2.jsonl'),
                 '--board', BOARD,
                 '--kind', 'completion', '--lever', 'x',
                 '--score-file', sc({'blocking': 0, 'quality': {},
                                     'blocking_by': {'unrouted': 0, 'broken': 0,
                                                     'drc': None,
                                                     'undersized': None}},
                                    'null.json')]
                + _lens_file(td, 'VERDICT=PASS:lens=drc'))
        assert r.returncode == 0, r.stderr
        assert 'CONTRADICTS' not in r.stderr, r.stderr

        # ...and a score that demonstrably grades ANOTHER board does not fire
        # it either: that would be judging a verdict about board A with board
        # B's numbers, the same mistake in the other direction.
        r = _cv(['record', '--ledger', os.path.join(td, 'l3.jsonl'),
                 '--board', BOARD, '--kind',
                 'completion', '--lever', 'baseline row',
                 '--score-file', sc({'blocking': 9, 'board_sha': 'f' * 64,
                                     'quality': {},
                                     'blocking_by': {'unrouted': 9}},
                                    'other.json')]
                + _lens_file(td, 'VERDICT=PASS:lens=connectivity'))
        assert r.returncode == 0, r.stderr
        assert 'CONTRADICTS' not in r.stderr, r.stderr
    print("  PASS: a PASS lens beside a contradicting count is refused, named")


def test_plateau_counts_rejected_laps_and_states_the_true_threshold():
    """D2. The gate was unsatisfiable by construction.

    The guard was `<=`, so --flat 5 needed SIX accepted laps while the refusal
    said five; it counted accepted rows only, though L5 instructs "Record the
    lap you are about to run, accepted or rejected"; and routing accepts only
    on strict improvement, so an exhausted half has no honest accepted lap left
    to produce. Two consecutive refusals came out byte-identical.
    """
    acc = {'kind': 'completion', 'accepted': True,
           'score': {'blocking': 0, 'quality': {}, 'ungraded': []}}
    rej = {'kind': 'completion', 'accepted': False, 'score': None}

    # EXACTLY --flat accepted laps is answerable, and answers `plateau`.
    st = converge._half_state([acc] * 5, 'routing', 5)
    assert st['why'] == 'plateau' and st['flat'] is True, st

    # A REJECTED lap is plateau evidence: it is precisely the record of a half
    # that tried and did not improve.
    st = converge._half_state([acc] * 2 + [rej] * 3, 'routing', 5)
    assert st['laps'] == 5 and st['accepted'] == 2 and st['rejected'] == 3, st
    assert st['flat'] is True and st['why'] == 'plateau', st

    # ...and a half that really is still moving is still not flat.
    moving = [dict(acc, score={'blocking': b, 'quality': {}, 'ungraded': []})
              for b in (9, 8, 7, 6, 5)]
    st = converge._half_state(moving, 'routing', 5)
    assert st['flat'] is False and st['why'] == 'improving', st

    # The refusal text must state the threshold it actually applies.
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        with open(led, 'w', encoding='utf-8') as fh:
            for i, r in enumerate([acc] * 2):
                fh.write(json.dumps(dict(r, iteration=i)) + '\n')
        p = os.path.join(td, 's.json')
        json.dump({'blocking': 0, 'quality': {}, 'ungraded': []},
                  open(p, 'w', encoding='utf-8'))
        r = _cv(['verdict', '--ledger', led, '--score', p, '--flat', '5'])
        doc = json.loads(r.stdout)
        assert doc['verdict'] == 'CONTINUE', doc
        assert '2 recorded lap(s)' in doc['reason'], doc['reason']
        assert 'fewer than the 5 this test needs' in doc['reason'], doc['reason']
        assert '--exhausted routing' in doc['reason'], \
            'the text names a remedy; the remedy must be a real flag'
    print("  PASS: the plateau counter counts rejections and states its true "
          "threshold")


def test_a_half_can_declare_itself_exhausted_on_the_record():
    """D2, the remedy the verdict text has always promised and never had.

    "the lever is to give it something further to optimise (or to say on the
    record that there is nothing)" -- and no flag implemented the parenthetical.
    --accept-residue / --accept-unclosed / --accept-congestion are documented as
    deliberately non-overlapping and none touches the counter, while --flat is
    forbidden by the same sentence.
    """
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        p = os.path.join(td, 's.json')
        json.dump({'blocking': 0, 'quality': {}, 'ungraded': []},
                  open(p, 'w', encoding='utf-8'))

        def rec(*extra):
            return _cv(['record', '--ledger', led, '--board', BOARD,
                        '--kind', 'systemic'] + list(extra))

        # A declaration with no reason is just a lower --flat with extra steps.
        r = rec('--exhausted', 'routing', '--lever', 'nothing left')
        assert r.returncode == 2 and 'exhausted-reason' in r.stderr, r.stderr
        assert not os.path.exists(led), 'nothing may be written on refusal'

        r = rec('--exhausted', 'routing', '--lever', 'declaration',
                '--exhausted-reason',
                'every rip depth and grid tried; the 32 open nets need a lane '
                'no parameter creates')
        assert r.returncode == 0, r.stderr
        assert json.loads(r.stdout)['exhausted']['half'] == 'routing'

        for i in range(5):                       # placement plateaus normally
            assert _cv(['record', '--ledger', led, '--board', BOARD, '--kind',
                        'placement', '--lever', f'lap {i}']
                       + _score_file(td, {'blocking': 0, 'quality': {},
                                          'ungraded': []}, 'lap.json')
                       ).returncode == 0
        doc = json.loads(_cv(['verdict', '--ledger', led, '--score', p]).stdout)
        assert doc['verdict'] == 'DONE-EXHAUSTED', doc['verdict']
        assert doc['routing']['why'] == 'declared-exhausted', doc['routing']
        assert 'DECLARED exhausted' in doc['reason'], doc['reason']
        assert 'no parameter creates' in doc['reason'], \
            'the reason a human gave is the evidence; it must reach the artifact'

        # Running that half again RETRACTS the declaration -- no flag, no edit.
        assert _cv(['record', '--ledger', led, '--board', BOARD, '--kind',
                    'completion', '--lever', 'back to work']).returncode == 0
        doc = json.loads(_cv(['verdict', '--ledger', led, '--score', p]).stdout)
        assert doc['verdict'] == 'CONTINUE', doc['verdict']
        assert doc['routing'].get('declared_superseded'), doc['routing']
    print("  PASS: a half can be declared exhausted, with a reason, and a "
          "later lap retracts it")


def test_a_score_that_measured_nothing_is_reported_not_raised():
    """#936 D1. `verdict --score` on a document whose `blocking` is null.

    `_score_key` returns None for `blocking is None` deliberately -- its own
    comment says an unmeasured lap must not rank as `inf`, because `inf >= inf`
    made the plateau test TRUE and an unmeasured lap then read as a plateaued
    one. It also says "the callers already filter None". The ledger-row caller
    does; the `--score` caller did not, so `blocking = key[0]` raised
    `TypeError: 'NoneType' object is not subscriptable` -- on exactly the input
    the docstring is written for.

    `blocking: null` is what board_score emits when a component that was ASKED
    for could not answer. The verdict must name the missing measurement and
    must NOT be a verdict about the board.

    THE LEDGER HERE IS PLATEAUED ON PURPOSE -- five recorded laps in each half
    at an identical score. A bare `key[0] if key else None` guard is the weaker
    fix this test exists to reject, and on a ONE-LAP ledger it is
    indistinguishable from the real one: both halves report `too-few-laps`, so
    execution reaches CONTINUE either way and only the exit code differs. Only
    on a plateaued ledger does the bare guard fall past `elif blocking == 0`
    into the terminal branch and print
    `STUCK: blocking == None and neither half improved in its last 5 recorded
    laps`, which reads as a measurement of a board nothing measured. Measured
    with the bare guard substituted: verdict STUCK, exit 5.

    `'DONE' not in stdout` would be a tautology (a null `blocking` can never
    satisfy `elif blocking == 0`), so the assertion below is on the verdict
    NAME, which is the thing a caller reads (the retired loop_driver's L5
    branched on it and discarded the exit code entirely).

    _score_key returns None for three distinct documents. Each gets its own
    sentence, and the "a component could not answer" cause is asserted only
    when the score itself names one in `unknown` -- publishing an unmeasured
    cause is the defect this whole issue is about.
    """
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        flat = json.dumps({'blocking': 3, 'quality': {}})
        for half, n in (('placement', 5), ('completion', 5)):
            for i in range(n):
                assert _cv(['record', '--ledger', led, '--board', BOARD,
                            '--kind', half, '--lever', f'{half} lap {i}']
                           + _score_file(td, flat, 'flat.json')).returncode == 0

        p = os.path.join(td, 'null.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': None, 'quality': {},
                       'ungraded': ['length'], 'unknown': ['impedance']}, fh)
        r = _cv(['verdict', '--ledger', led, '--score', p])
        assert 'Traceback' not in r.stderr, r.stderr
        doc = json.loads(r.stdout)
        # The half that matters: NOT a verdict about the board. This is what
        # the bare guard fails -- it says STUCK here.
        assert doc['verdict'] == 'NO-SCORE', doc
        assert r.returncode == 2, (r.returncode, r.stdout, r.stderr)
        assert 'could not answer (impedance)' in doc['reason'], doc['reason']
        assert doc['unknown'] == ['impedance'], doc
        assert doc['ungraded'] == ['length'], doc

        # ...and with nothing named in `unknown`, the cause is NOT asserted.
        p2 = os.path.join(td, 'null2.json')
        with open(p2, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': None, 'quality': {}}, fh)
        why = json.loads(_cv(['verdict', '--ledger', led,
                              '--score', p2]).stdout)['reason']
        assert 'nothing measured it' in why and 'could not answer' not in why, why

        # The other two None-shaped documents get their OWN sentences, because
        # a missing key and a null value are not the same fact.
        for doc_in, phrase in (
                ([1, 2], 'is a list, not an object'),
                ({'quality': {}}, 'no `blocking` key at all'),
                (None, 'holds the JSON document `null`')):
            q = os.path.join(td, 'shape.json')
            with open(q, 'w', encoding='utf-8') as fh:
                json.dump(doc_in, fh)
            r = _cv(['verdict', '--ledger', led, '--score', q])
            assert r.returncode == 2 and 'Traceback' not in r.stderr, r.stderr
            got = json.loads(r.stdout)
            assert got['verdict'] == 'NO-SCORE', got
            assert phrase in got['reason'], (phrase, got['reason'])
            # ONE shape for every NO-SCORE document: a consumer reading
            # doc['ungraded'] must not KeyError on half of them.
            assert 'ungraded' in got and 'unknown' in got, got

        # Reading a MALFORMED score must not be the thing that raises: this is
        # the crash class the guard exists to remove, and `sorted(x or [])`
        # reintroduces it one line inside the fix.
        bad = os.path.join(td, 'badungraded.json')
        with open(bad, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': None, 'ungraded': 5}, fh)
        r = _cv(['verdict', '--ledger', led, '--score', bad])
        assert r.returncode == 2 and 'Traceback' not in r.stderr, r.stderr
        assert json.loads(r.stdout)['ungraded'] == ['<not a list: 5>'], r.stdout

        # ...and the cause is claimed ONLY when the score names one as a LIST.
        # `{"unknown": "impedance"}` is not a component that answered; saying
        # so would be publishing an unmeasured cause.
        us = os.path.join(td, 'unknown_str.json')
        with open(us, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': None, 'unknown': 'impedance'}, fh)
        why = json.loads(_cv(['verdict', '--ledger', led,
                              '--score', us]).stdout)['reason']
        assert 'could not answer' not in why, why

        # `quality` IS NOT NECESSARILY A DICT. This raised AttributeError in
        # _score_key -- the same crash class, in the same function, one line
        # above the guard added for it -- and `record` accepts such a score, so
        # the raising value lands in the LEDGER and every later verdict on that
        # ledger tracebacks in the row comprehension whatever `--score` says.
        ql = os.path.join(td, 'quality_list.json')
        with open(ql, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': 1, 'quality': [1, 2]}, fh)
        r = _cv(['verdict', '--ledger', led, '--score', ql])
        assert 'Traceback' not in r.stderr, r.stderr
        assert json.loads(r.stdout)['verdict'] != 'NO-SCORE', r.stdout

        poison = os.path.join(td, 'poison.jsonl')
        assert _cv(['record', '--ledger', poison, '--board', BOARD, '--kind',
                    'placement', '--lever', 'malformed quality']
                   + _score_file(td, {'blocking': 1, 'quality': [1, 2]},
                                 'poison.json')
                   ).returncode == 0, 'record accepts it, which is the problem'
        good = os.path.join(td, 'good.json')
        with open(good, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': 0, 'quality': {'vias': 1, 'copper_mm': 2,
                                                  'segments': 3}}, fh)
        r = _cv(['verdict', '--ledger', poison, '--score', good])
        assert 'Traceback' not in r.stderr, \
            'one malformed ledger row must not break every later verdict'

        # ...and a score that DID measure still gets a real verdict, so the
        # guard is not a blanket refusal.
        ok = os.path.join(td, 'ok.json')
        with open(ok, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': 0, 'quality': {'vias': 1, 'copper_mm': 2,
                                                  'segments': 3}}, fh)
        r = _cv(['verdict', '--ledger', led, '--score', ok])
        assert r.returncode in (converge.CONTINUE, converge.DONE,
                                converge.STUCK, converge.BUDGET), r.returncode
        assert json.loads(r.stdout)['verdict'] != 'NO-SCORE', r.stdout
        # 2 is not one of the four verdict codes, so a caller switching on the
        # exit cannot mistake "nothing measured" for a verdict about a board.
        assert 2 not in (converge.CONTINUE, converge.DONE, converge.STUCK,
                         converge.BUDGET)
    print("  PASS: a null `blocking` is reported as NO-SCORE, not raised, "
          "and not graded as STUCK on a plateaued ledger")


def test_two_scores_that_graded_different_components_do_not_compare():
    """D12. Two `blocking` totals over different component sets are not larger
    and smaller versions of each other.

    Measured (run 17, cycles 2 and 3 of one board): compared as 78 vs 79 and a
    decision taken on the difference. Graded commensurably -- with
    --impedance-nets, which neither run passed and which the board warrants --
    both score 92 (32 unrouted / 46 broken, tied net for net), and the board
    called WORSE was one impedance crossing BETTER.
    """
    full = {'blocking': 79, 'quality': {}, 'ungraded': [],
            'blocking_by': {'unrouted': 32, 'broken': 46, 'drc': 1,
                            'impedance': 0}}
    part = {'blocking': 78, 'quality': {}, 'ungraded': ['impedance'],
            'blocking_by': {'unrouted': 32, 'broken': 46, 'drc': 0,
                            'impedance': None},
            'components': {'impedance': {
                'ran': False,
                'reason': 'no --impedance-nets given; impedance is ungraded'}}}
    differ, hint, false_improvement = converge.commensurability(full, part)
    assert differ == ['impedance'] and false_improvement is True, (differ, hint)
    assert '--impedance-nets' in hint, hint
    assert converge.commensurability(full, dict(full, blocking=70)) is None, \
        'two scores over the same component set compare normally'

    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')

        def sc(doc, name):
            p = os.path.join(td, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        def rec(score, *extra):
            return _cv(['record', '--ledger', led, '--board', BOARD, '--kind',
                        'completion', '--lever', 'lap', '--score-file', score]
                       + list(extra))

        assert rec(sc(full, 'c2.json')).returncode == 0
        # 79 -> 78 while grading one component LESS. Routing accepts on strict
        # improvement, so this is how a ledger acquires a false monotone trend.
        r = rec(sc(part, 'c3.json'))
        assert r.returncode == 2, (r.returncode, r.stdout[:300])
        assert 'graded FEWER components' in r.stderr, r.stderr
        assert 'impedance' in r.stderr and '--impedance-nets' in r.stderr
        # Recorded --rejected, it is not a claimed improvement, and is kept.
        r = rec(sc(part, 'c3b.json'), '--rejected')
        assert r.returncode == 0, r.stderr
        assert json.loads(r.stdout)['accepted'] is False

    # ...and the verdict never CREDITS an improvement across such a pair.
    def acc(s):
        return {'kind': 'completion', 'accepted': True, 'score': s}

    st = converge._half_state([acc(full)] * 4 + [acc(part)], 'routing', 5)
    assert st['flat'] is True, \
        '78 after four 79s is not an improvement when it graded one less'
    assert st['incommensurable'] and st['compared'] == 4, st

    # ...but guarding a false improvement must not MANUFACTURE a false plateau.
    # Judging only the LEADING comparable run collapsed a window to one lap
    # whenever its first pair did not compare, and `min(keys[:1]) >= keys[0]`
    # is a tautology: 78 -> 70 -> 60 -> 50 -> 40, four comparable improvements
    # of 10 each, reported `plateau` and ended the run at STUCK.
    def _s(b, ung=()):
        return {'blocking': b, 'quality': {}, 'ungraded': list(ung),
                'blocking_by': {'unrouted': b,
                                'impedance': (None if ung else 0)}}

    st = converge._half_state(
        [acc(_s(78, ['impedance']))] + [acc(_s(b)) for b in (70, 60, 50, 40)],
        'routing', 5)
    assert st['why'] == 'improving', \
        'an incommensurable FIRST pair must not discard four real improvements'
    print("  PASS: incommensurable scores are refused at record and never "
          "credited as improvement")


def test_a_plateau_is_never_claimed_over_laps_nothing_measured():
    """A plateau asserted over laps that measured nothing is the
    "reported clean because unexamined" error, in the stop rule.

    Measured on the real run-17 ledger: the placement half read `plateau` from
    a window of two cycle-1 accepted laps plus three cycle-2 REJECTIONS, while
    five accepted cycle-2 laps were invisible for carrying `score: null`.
    """
    def acc(b):
        return {'kind': 'completion', 'accepted': True,
                'score': {'blocking': b, 'quality': {}, 'ungraded': []}}

    rej = {'kind': 'completion', 'accepted': False, 'score': None}
    unscored = {'kind': 'completion', 'accepted': True, 'score': None}

    st = converge._half_state([acc(0), acc(0)] + [unscored] * 3, 'routing', 5)
    assert st['why'] == 'no-comparison' and st['unjudged'] == 3, st
    assert st['blocked'] == 'unjudged', st
    assert st['flat'] is False, 'an unjudged lap is not evidence of a plateau'
    # An unscored ACCEPTED lap is still a lap: it must not vanish from the
    # count while rejections are kept.
    assert st['laps'] == 5 and st['accepted'] == 5, st
    # A window whose only accepted lap cannot be compared with anything is
    # likewise not answerable -- it used to read `plateau` off a single key,
    # where `min(keys[:1]) >= keys[0]` is a tautology.
    st = converge._half_state([rej] * 4 + [acc(40)], 'routing', 5)
    assert st['why'] == 'no-comparison' and st['flat'] is False, st
    assert st['blocked'] == 'single-lap', st
    # ...while an ALL-rejected window is a definite plateau: a rejection is
    # itself the measurement.
    assert converge._half_state([rej] * 5, 'routing', 5)['why'] == 'plateau'

    # A REJECTION DOES NOT SEPARATE TWO SCORES. Comparability is a property of
    # two scores' graded component sets and of nothing else, so treating a
    # rejection as a break in the chain was a second false plateau -- measured
    # over all 7776 accept/reject windows of 5, 441 flipped `improving` ->
    # `plateau` and 2313 flipped `plateau` -> unanswerable, on the alternating
    # accept/reject pattern that IS normal routing.
    assert converge._half_state(
        [acc(78), acc(78), acc(78), rej, acc(40)], 'routing', 5
    )['why'] == 'improving', 'a rejection must not hide the lap after it'
    assert converge._half_state(
        [acc(40), rej, acc(40), rej, acc(40)], 'routing', 5
    )['why'] == 'plateau', 'alternating accept/reject must still be judgeable'

    # THE MESSAGE MUST NAME THE RIGHT CAUSE. The three blocked states call for
    # different actions, and a text that guessed told a still-improving half to
    # declare itself exhausted.
    def _incomm(b, ung):
        return {'kind': 'completion', 'accepted': True,
                'score': {'blocking': b, 'quality': {}, 'ungraded': list(ung),
                          'blocking_by': {'unrouted': b,
                                          'impedance': None if ung else 0}}}

    st = converge._half_state(
        [_incomm(9, []), _incomm(9, ['impedance']), rej, rej, rej],
        'routing', 5)
    assert st['blocked'] == 'incommensurable', st
    with tempfile.TemporaryDirectory() as td:
        def _v(rows, name):
            p = os.path.join(td, name)
            with open(p, 'w', encoding='utf-8') as fh:
                for i, r in enumerate(rows):
                    fh.write(json.dumps(dict(r, iteration=i)) + '\n')
            s = os.path.join(td, name + '.score')
            json.dump({'blocking': 0, 'quality': {}, 'ungraded': []},
                      open(s, 'w', encoding='utf-8'))
            return json.loads(_cv(['verdict', '--ledger', p,
                                   '--score', s]).stdout)['reason']

        flat_p = [{'kind': 'placement', 'accepted': True,
                   'score': {'blocking': 0, 'quality': {}, 'ungraded': []}}] * 5
        r = _v(flat_p + [acc(0), acc(0)] + [unscored] * 3, 'u.jsonl')
        assert 'recorded no `blocking`' in r, r
        assert 'not graded over the same components' not in r, r
        r = _v(flat_p + [rej] * 4 + [acc(40)], 's.jsonl')
        assert 'only one of them carries a score' in r, r
    print("  PASS: a plateau needs every lap in the window to have been judged")


def test_a_lens_verdict_is_never_skipped_silently():
    """The lens-vs-score check is skipped when the score grades another board
    -- correctly, but an UNANNOUNCED skip is a door: attach any other board's
    score and the whole gate switches off. Replaying all 56 real run-17 rows
    with their original board_sha against a dummy board gave 0 refusals."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        p = os.path.join(td, 's.json')
        json.dump({'blocking': 9, 'board_sha': 'f' * 64,
                   'blocking_by': {'unrouted': 9, 'broken': 0}},
                  open(p, 'w', encoding='utf-8'))
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--kind',
                 'completion', '--lever', 'x', '--score-file', p]
                + _lens_file(td, 'VERDICT=PASS:lens=connectivity'))
        assert r.returncode == 0, r.stderr
        assert 'did NOT run' in r.stderr and 'UNCHECKED' in r.stderr, r.stderr

    # A capitalised lens name used to pass the format check, miss the
    # lower-case table and never be compared at all.
    bad = {'blocking': 79, 'blocking_by': {'unrouted': 32, 'broken': 47}}
    for name in ('connectivity', 'Connectivity', 'CONNECTIVITY'):
        assert converge.lens_contradictions(
            [f'VERDICT=PASS:lens={name}'], bad), name
    print("  PASS: a skipped lens check says so; a capitalised lens still "
          "compares")


def test_the_exhausted_declaration_does_not_leak_between_halves():
    """One half declaring itself finished must not finish the other, and a
    `systemic` row -- which changes no copper -- must not retract it."""
    dec = {'kind': 'systemic', 'accepted': True,
           'exhausted': {'half': 'placement', 'reason': 'nothing left'}}
    lap = {'kind': 'completion', 'accepted': True,
           'score': {'blocking': 0, 'quality': {}, 'ungraded': []}}
    # ON THE DECLARATION ALONE. Asserting on [dec, lap] proved nothing: the
    # routing lap in that list retracts a leaked declaration, so the assertion
    # passed even with the half check deleted -- verified by mutation.
    assert converge._half_state([dec], 'placement', 5)['why'] \
        == 'declared-exhausted'
    assert converge._half_state([dec], 'routing', 5)['why'] != \
        'declared-exhausted', 'a declaration must not cross halves'
    rows = [dec, lap]
    assert converge._half_state(rows, 'placement', 5)['why'] \
        == 'declared-exhausted', 'the other half\'s lap does not retract it'
    # A systemic row after the declaration is not a lap of that half.
    assert converge._half_state(
        [dec, {'kind': 'systemic', 'accepted': True}], 'placement', 5
    )['why'] == 'declared-exhausted'
    # ...but a PLACEMENT lap after it is, and it retracts.
    assert converge._half_state(
        [dec, {'kind': 'placement', 'accepted': True}], 'placement', 5
    )['why'] != 'declared-exhausted'
    print("  PASS: --exhausted binds to its own half and only its own laps "
          "retract it")


def test_every_incommensurable_pair_is_reported_even_when_it_changes_nothing():
    """The WARNING must fire on ANY incommensurable pair, not only the shape
    that gets refused -- and the verdict must say so on EVERY verdict, not
    only the one it happened to change."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')

        def sc(doc, name):
            p = os.path.join(td, name)
            json.dump(doc, open(p, 'w', encoding='utf-8'))
            return p

        full = {'blocking': 70, 'quality': {}, 'ungraded': [],
                'blocking_by': {'unrouted': 70, 'impedance': 0}}
        # blocking ROSE, so this is not a false improvement and is not refused
        # -- but the two rows still did not measure the same thing.
        worse = {'blocking': 90, 'quality': {}, 'ungraded': ['impedance'],
                 'blocking_by': {'unrouted': 90, 'impedance': None}}
        assert _cv(['record', '--ledger', led, '--board', BOARD, '--kind',
                    'completion', '--lever', 'a',
                    '--score-file', sc(full, 'a.json')]).returncode == 0
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--kind',
                 'completion', '--lever', 'b',
                 '--score-file', sc(worse, 'b.json')])
        assert r.returncode == 0, r.stderr
        assert 'INCOMMENSURABLE' in r.stderr, r.stderr

        # And on the verdict, whatever the verdict is.
        rows = ([{'kind': 'placement', 'accepted': True, 'score': full}] * 5
                + [{'kind': 'completion', 'accepted': True, 'score': full}] * 4
                + [{'kind': 'completion', 'accepted': True, 'score': worse}])
        lp = os.path.join(td, 'v.jsonl')
        with open(lp, 'w', encoding='utf-8') as fh:
            for i, r2 in enumerate(rows):
                fh.write(json.dumps(dict(r2, iteration=i)) + '\n')
        doc = json.loads(_cv(['verdict', '--ledger', lp,
                              '--score', sc(full, 'v.json')]).stdout)
        assert 'NOT COMPARABLE' in doc['reason'], doc['reason']
        assert '--impedance-nets' in doc['reason'], doc['reason']
        # The window is judged over maximal comparable RUNS, not "the first N",
        # and N could be 0 -- the sentence has to describe what happened.
        assert 'within the 4 lap(s) that do compare' in doc['reason'], \
            doc['reason']
    print("  PASS: every incommensurable pair is reported, at record and at "
          "verdict")


def test_record_score_failures_want_names():
    """Run-7 S10/F5: a score carrying only a COUNT forces every later read to
    re-derive which nets -- and a truncated re-derivation shipped a wrong
    close-out. Names print when given; a count without names draws a NOTE."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', BOARD]
                + _score_file(td, '{"failures": 3}', 'f3.json'))
        assert r.returncode == 0
        assert 'names no nets' in r.stderr, r.stderr
        r = _cv(['record', '--ledger', led, '--board', BOARD]
                + _score_file(td, '{"failures": 2, "failed_nets": '
                                  '["SWDIO", "GPIO1"]}', 'f2.json'))
        assert r.returncode == 0
        assert 'SWDIO' in r.stderr and 'GPIO1' in r.stderr, r.stderr
    print("  PASS: failing-net names surface; a bare count draws a NOTE")


# ------------------------------------------------------------ where --oracle

def test_parse_pairs_yields_join_specs():
    """parse_pairs turns raw DRC unconnected items into {net, a, b} join
    specs with x/y/layer/kind -- no kicad-cli needed to verify the shape."""
    from kicad_unconnected import parse_pairs
    items = [
        {'items': [
            {'description': 'Pad 1 [GND] of R1 on F.Cu',
             'pos': {'x': 10.0, 'y': 20.0}},
            {'description': 'Track [GND] on F.Cu',
             'pos': {'x': 11.5, 'y': 20.0}}]},
        # straddles nets -> dropped, exactly as the oracle drops it
        {'items': [
            {'description': 'Pad 2 [VCC] of R1 on F.Cu',
             'pos': {'x': 1.0, 'y': 1.0}},
            {'description': 'Track [GND] on F.Cu',
             'pos': {'x': 2.0, 'y': 1.0}}]},
    ]
    pairs = parse_pairs(items)
    assert len(pairs) == 1, pairs
    p = pairs[0]
    assert p['net'] == 'GND'
    assert p['a'] == {'x': 10.0, 'y': 20.0, 'layer': 'F.Cu', 'kind': 'pad'}
    assert p['b']['kind'] == 'track'
    print("  PASS: raw DRC items become exact join specs; cross-net pairs drop")


def test_where_without_nets_or_oracle_refuses():
    r = _cv(['where', BOARD])
    assert r.returncode == 2, (r.returncode, r.stdout)
    assert '--oracle' in r.stdout
    print("  PASS: where with neither --nets nor --oracle refuses with advice")


def test_where_oracle_prints_the_join_work_list():
    """On the unrouted fixture, KiCad's own DRC (zones refilled) must surface
    joins, each as an exact endpoint-pair spec, before the forensics."""
    from kicad_unconnected import find_kicad_cli
    if find_kicad_cli() is None:
        print("  SKIP: kicad-cli not installed")
        return
    r = _cv(['where', BOARD, '--oracle', '--nets', 'GND'], timeout=600)
    assert 'ORACLE:' in r.stdout, r.stdout[:1500]
    assert 'join(s) remain' in r.stdout
    assert ' <-> ' in r.stdout, "no pair spec printed:\n" + r.stdout[:1500]
    print("  PASS: where --oracle prints KiCad's join list as pair specs")


def test_record_warns_on_unbound_or_mismatched_score():
    """Run-3 B4: three ledger entries shipped embedding a PRIOR board's
    quality because a score was free JSON with no binding to --board.
    board_score now embeds board_sha; record warns loudly on absence or
    mismatch (a warning, not a refusal: baseline rows legitimately attach a
    parent score to a rejected candidate -- but never silently)."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from board_store import sha256_file
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')

        def rec(score):
            return _cv(['record', '--ledger', led, '--board', BOARD,
                        '--lever', 'x'] + _score_file(td, score))

        r = rec({'blocking': 0, 'board_sha': sha256_file(BOARD)})
        assert r.returncode == 0, r.stderr
        assert 'record WARNING' not in r.stderr, r.stderr

        r = rec({'blocking': 0})  # no board_sha (pre-B4 payload)
        assert r.returncode == 0
        assert 'no board_sha' in r.stderr, r.stderr

        r = rec({'blocking': 0, 'board_sha': 'f' * 64})  # a different board
        assert r.returncode == 0
        assert 'DIFFERENT board' in r.stderr, r.stderr
    print("  PASS: record binds score payloads to the recorded board")


def test_board_score_emits_board_sha():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from board_store import sha256_file
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'board_score.py'), BOARD, '-q'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT)
    line = [l for l in r.stdout.splitlines() if l.startswith('SCORE_JSON=')]
    assert line, r.stdout[:800] + r.stderr[:800]
    doc = json.loads(line[0].split('=', 1)[1])
    assert doc.get('board_sha') == sha256_file(BOARD), \
        "board_score must bind its payload to the graded file"
    print("  PASS: board_score embeds board_sha")


def test_a_final_close_out_is_accepted_for_every_terminal_verdict():
    """D1: a run-closing `record --final` with three --lens-file slots is
    ACCEPTED for every terminal verdict. The STUCK/BUDGET paths carry a FAIL
    lens as the normal case, and DONE-EXHAUSTED must never sit beside one.
    (This used to execute the retired loop_driver's printed L5 command; the
    command is now built here in the same shape, since converge's contract
    is what remains.)"""
    _PASS = {'connectivity': 'VERDICT=PASS:lens=connectivity',
             'drc': 'VERDICT=PASS:lens=drc',
             'spec': 'VERDICT=PASS:lens=spec'}
    _FAIL_DRC = ('VERDICT=FAIL:lens=drc;finding=3 clearance violations;'
                 'evidence=score.json#/blocking_by/drc')
    _CLEAN = {'blocking': 0, 'blocking_by':
              {'unrouted': 0, 'broken': 0, 'drc': 0, 'undersized': 0}}
    _DIRTY = {'blocking': 3, 'blocking_by':
              {'unrouted': 0, 'broken': 0, 'drc': 3, 'undersized': 0}}

    def close_out(name, lens_by_name, score_doc, led):
        work = os.path.dirname(led)
        score = os.path.join(work, 'score.json')
        with open(score, 'w', encoding='utf-8') as f:
            json.dump(score_doc, f)
        slots = []
        for lens in ('connectivity', 'drc', 'spec'):
            path = os.path.join(work, f'verdict_{lens}.txt')
            with open(path, 'w', encoding='utf-8') as f:
                f.write(lens_by_name[lens] + '\n')
            slots += ['--lens-file', path]
        return _cv(['record', '--ledger', led, '--board', BOARD,
                    '--kind', 'completion', '--final',
                    '--stop-condition', name] + slots
                   + ['--lever', f'close-out: {name}', '--score-file', score])

    with tempfile.TemporaryDirectory() as td:
        # DONE-EXHAUSTED: every lens passes, clean score.
        r = close_out('DONE-EXHAUSTED', _PASS, _CLEAN,
                      os.path.join(td, 'done.jsonl'))
        assert r.returncode == 0, r.stderr
        e = json.loads(r.stdout)
        assert e.get('final') is True and \
            e.get('stop_condition') == 'DONE-EXHAUSTED'
        # STUCK and BUDGET: a FAIL lens is the NORMAL case, not a refusal.
        for name in ('STUCK', 'BUDGET'):
            lenses = dict(_PASS, drc=_FAIL_DRC)
            r = close_out(name, lenses, _DIRTY,
                          os.path.join(td, name + '.jsonl'))
            assert r.returncode == 0, f'{name}: {r.stderr}'
            e = json.loads(r.stdout)
            assert e.get('stop_condition') == name
            assert any(v.startswith('VERDICT=FAIL') for v in e['lenses'])
        # DONE-EXHAUSTED beside a FAIL lens is the one contradiction.
        led = os.path.join(td, 'contra.jsonl')
        r = close_out('DONE-EXHAUSTED', dict(_PASS, drc=_FAIL_DRC),
                      _DIRTY, led)
        assert r.returncode == 2 and 'contradiction' in r.stderr, r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"
    print("  PASS: a --final close-out is accepted for all three verdicts, "
          "and DONE-EXHAUSTED beside a FAIL lens is refused")


def _row_1071(i, kind, blocking, accepted=True, **extra):
    """A ledger row in the shape `record` writes, with a hand-picked score --
    `record` now refuses the values these tests need, so they are written."""
    score = {'blocking': blocking}
    score.update(extra)
    return {'iteration': i, 'kind': kind, 'accepted': accepted,
            'lever': f'{kind} lap {i}',
            'parent_sha': None, 'result_sha': 'a' * 64, 'score': score,
            't': 1000.0 + i}


def _write_ledger_1071(path, rows):
    with open(path, 'w', encoding='utf-8') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')
    return path


def _verdict_1071(td, led, score_text):
    p = os.path.join(td, 's.json')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(score_text)
    r = _cv(['verdict', '--ledger', led, '--score', p])
    assert 'Traceback' not in r.stderr, r.stderr
    return r


def _no_bare_constants(text):
    """json.loads that refuses NaN/Infinity -- strict JSON, as a reader other
    than Python would parse the verdict document."""
    def _refuse(tok):
        raise ValueError(f'bare {tok} in the verdict document')
    return json.loads(text, parse_constant=_refuse)


def _run_utils():
    _t = os.path.join(ROOT, 'tests')
    if _t not in sys.path:
        sys.path.insert(0, _t)
    import run_utils
    return run_utils


#: `blocking` values that are not a count, as raw JSON, with the phrase the
#: refusal must name for each (#1071, #1075).
_NOT_A_COUNT_1071 = (
    ('{"assembly_blocking_pairs": 8, "drc_violations": 30}', 'a JSON object'),
    ('[1]', 'a JSON array'),
    ('"10"', 'a string'),
    ('true', 'boolean'),
    ('false', 'boolean'),
    ('-1', 'negative'),
    ('NaN', 'nan'),
    ('Infinity', 'inf'),
    ('-Infinity', 'inf'),
    ('1' + '0' * 400, '401 digits'),
)


def test_1071_a_ledger_holding_a_non_count_blocking_is_read_not_raised():
    """#1071, #1075. A recorded `blocking` that is not a count is UNJUDGED.

    The #1071 shape is wk/run21/tigard/ledger.jsonl: 7 placement rows whose
    `blocking` is a per-term dict and no `quality`. `_score_key` returned
    `(dict, quality)`, and once the flat window filled `min(r) < r[0]`
    compared two different dicts: every verdict raised, however good its
    --score. The scalar cases did not raise -- they gave a WRONG verdict:
    `false`, `NaN`, `"10"`, `-1`, `true` and `Infinity` all plateaued, so a
    board scored `{"blocking": 0}` read DONE-EXHAUSTED on laps that ranked
    nothing.
    """
    with tempfile.TemporaryDirectory() as td:
        # The tigard shape: accepted T,F,T,T,F,F,T, dict blocking, no quality.
        acc = (True, False, True, True, False, False, True)
        rows = [{'iteration': i, 'kind': 'placement', 'accepted': a,
                 'lever': f'lap {i}', 'parent_sha': None,
                 'result_sha': 'a' * 64, 't': 1000.0 + i,
                 'score': {'blocking': {'assembly_blocking_pairs': 8 - i,
                                        'drc_violations': 30 + i,
                                        'off_outline_pad_copper': 0}}}
                for i, a in enumerate(acc)]
        led = _write_ledger_1071(os.path.join(td, 'tigard.jsonl'), rows)
        r = _verdict_1071(td, led, '{"blocking": 0, "quality": {}}')
        assert r.returncode == converge.CONTINUE, (r.returncode, r.stdout)
        doc = _no_bare_constants(r.stdout)
        assert doc['verdict'] == 'CONTINUE', doc
        pl = doc['placement']
        assert pl['why'] == 'no-comparison', pl
        assert pl['blocked'] == 'unjudged', pl
        assert pl['unjudged_iterations'] == [2, 3, 6], pl
        assert 'recorded no `blocking` a verdict can rank (null, absent, or ' \
            'not a non-negative number)' in doc['reason'], doc['reason']

        # The scalar cases: every lap of BOTH halves carries the same bad
        # value. Before the fix each of these read DONE-EXHAUSTED, exit 0.
        for raw, _phrase in _NOT_A_COUNT_1071:
            rows = ([_row_1071(i, 'placement', None) for i in range(5)]
                    + [_row_1071(5 + i, 'completion', None) for i in range(5)])
            text = ''.join(json.dumps(r).replace(
                '"blocking": null', '"blocking": ' + raw) + '\n' for r in rows)
            led = os.path.join(td, 'scalar.jsonl')
            with open(led, 'w', encoding='utf-8') as fh:
                fh.write(text)
            r = _verdict_1071(td, led, '{"blocking": 0}')
            assert r.returncode == converge.CONTINUE, (raw, r.returncode,
                                                       r.stdout)
            doc = _no_bare_constants(r.stdout)
            for h in ('placement', 'routing'):
                assert doc[h]['why'] == 'no-comparison', (raw, h, doc[h])

        # A string beside an int raised (`'<' not supported between 'str' and
        # 'int'`). The string laps drop out, and the two counts either side of
        # them are compared on their own: 5 -> 4 is a real improvement, which
        # stands whatever else is in the window.
        rows = ([_row_1071(0, 'placement', 5), _row_1071(1, 'placement', '9'),
                 _row_1071(2, 'placement', '10'), _row_1071(3, 'placement', 4),
                 _row_1071(4, 'placement', 4)])
        led = _write_ledger_1071(os.path.join(td, 'mixed.jsonl'), rows)
        r = _verdict_1071(td, led, '{"blocking": 0}')
        assert r.returncode == converge.CONTINUE, (r.returncode, r.stdout)
        assert json.loads(r.stdout)['placement']['why'] == 'improving', r.stdout
        # ...and with no count to anchor them, "9" -> "10" is NOT an
        # improvement ("10" < "9" as strings): the half is unjudged.
        rows = [_row_1071(i, 'placement', s)
                for i, s in enumerate(('9', '10', '10', '10', '10'))]
        led = _write_ledger_1071(os.path.join(td, 'strings.jsonl'), rows)
        r = _verdict_1071(td, led, '{"blocking": 0}')
        doc = json.loads(r.stdout)
        assert doc['placement']['why'] == 'no-comparison', doc['placement']
        assert doc['placement']['unjudged_iterations'] == [0, 1, 2, 3, 4], \
            doc['placement']
    print("  PASS: a recorded blocking that is not a count is unjudged, never "
          "ranked, never a plateau, never a traceback")


def test_1075_a_score_whose_blocking_is_not_a_count_is_NO_SCORE():
    """#1075. The --score itself: `false == 0` read DONE-EXHAUSTED, and an
    `Infinity` printed "STUCK: blocking == None" -- the sentence #936 D1
    removed for the null case, back through the inf -> None echo."""
    with tempfile.TemporaryDirectory() as td:
        rows = ([_row_1071(i, 'placement', 3) for i in range(5)]
                + [_row_1071(5 + i, 'completion', 3) for i in range(5)])
        led = _write_ledger_1071(os.path.join(td, 'flat.jsonl'), rows)
        # The control: this ledger IS plateaued, so a real count stops it.
        r = _verdict_1071(td, led, '{"blocking": 3}')
        assert json.loads(r.stdout)['verdict'] == 'STUCK', r.stdout
        for raw, phrase in _NOT_A_COUNT_1071 + (('"0"', 'a string'),):
            r = _verdict_1071(td, led, '{"blocking": %s}' % raw)
            assert r.returncode == 2, (raw, r.returncode, r.stdout)
            doc = _no_bare_constants(r.stdout)
            assert doc['verdict'] == 'NO-SCORE', (raw, doc)
            assert 'must be a non-negative number' in doc['reason'], (raw, doc)
            assert phrase in doc['reason'], (raw, phrase, doc['reason'])
            assert 'is null' not in doc['reason'], (raw, doc['reason'])
            assert 'blocking == None' not in r.stdout, raw
            assert 'ungraded' in doc and 'unknown' in doc, doc
    # `quality` items: a bool or a NaN is not a measurement either. NaN never
    # compares, so it scrambled every tie it touched; `true` ranked as 1 via.
    inf = float('inf')
    for v in (True, float('nan'), inf, '3'):
        k = converge._score_key({'blocking': 0, 'quality': {'vias': v}})
        assert k == (0, (0, inf, inf, inf)), (v, k)
    assert converge._score_key({'blocking': 0, 'quality': {'vias': 4}}) \
        == (0, (0, 4, inf, inf))
    # A 400-digit JSON integer: `isfinite` converts an int to float and raised
    # OverflowError. As a `blocking` it is not a count (past any float, and the
    # film plots `float(b)`); as a quality item it ranks, and a ledger row
    # carrying one must not break every later verdict (`record` does not
    # validate `quality`).
    huge = 10 ** 400
    assert 'digits' in (converge.blocking_defect(huge) or ''), \
        converge.blocking_defect(huge)
    assert converge._score_key({'blocking': 3, 'quality': {'vias': huge}}) \
        == (3, (inf, huge, inf, inf))
    with tempfile.TemporaryDirectory() as td:
        rows = [_row_1071(i, 'completion', 3, quality={'vias': huge})
                for i in range(5)]
        led = _write_ledger_1071(os.path.join(td, 'huge.jsonl'), rows)
        r = _verdict_1071(td, led, '{"blocking": 3}')
        assert r.returncode == converge.CONTINUE, \
            (r.returncode, r.stderr[-300:])
    print("  PASS: a --score whose blocking is not a count is NO-SCORE, named "
          "for what it is")
    # #1340: at equal blocking the joins the open nets need rank before vias:
    # the lap that stranded a 9-pad rail (8 joins, 143 vias) is not better
    # than the one with a 2-pad net open (1 join, 151 vias)
    rail = converge._score_key({'blocking': 1, 'joins_needed': 8,
                                'quality': {'vias': 143, 'copper_mm': 962.8,
                                            'segments': 900}})
    one = converge._score_key({'blocking': 1, 'joins_needed': 1,
                               'quality': {'vias': 151, 'copper_mm': 980.3,
                                           'segments': 950}})
    assert one < rail, (one, rail)
    # a score without joins_needed: 0 at blocking 0, unmeasured above it
    assert converge._score_key({'blocking': 0, 'quality': {}})[1][0] == 0
    assert converge._score_key({'blocking': 2, 'quality': {}})[1][0] == inf
    print("  PASS: joins_needed ranks ahead of vias at equal blocking (#1340)")


def test_1076_verdict_reads_a_malformed_ungraded_beside_a_real_blocking():
    """#1076. `sorted(score.get('ungraded') or [])` ran next to a REAL
    blocking, where the NO-SCORE branch's `_names` guard never did: an int
    raised `not iterable` and a mixed list raised on `<`."""
    with tempfile.TemporaryDirectory() as td:
        led = _write_ledger_1071(os.path.join(td, 'l.jsonl'),
                                 [_row_1071(0, 'completion', 3)])
        r = _verdict_1071(td, led, '{"blocking": 1, "ungraded": 5}')
        assert r.returncode == converge.CONTINUE, (r.returncode, r.stderr)
        doc = json.loads(r.stdout)
        assert doc['ungraded'] == ['<not a list: 5>'], doc['ungraded']
        r = _verdict_1071(td, led, '{"blocking": 0, "unknown": "impedance"}')
        doc = json.loads(r.stdout)
        assert doc['unknown'] == ["<not a list: 'impedance'>"], doc['unknown']
        r = _verdict_1071(td, led, '{"blocking": 0, "ungraded": ["length", 3]}')
        doc = json.loads(r.stdout)
        assert doc['ungraded'] == ['3', 'length'], doc['ungraded']
        # Silent before, not raised: `sorted("abc")` named three components
        # a, b and c as UNEXAMINED.
        r = _verdict_1071(td, led, '{"blocking": 0, "ungraded": "abc"}')
        doc = json.loads(r.stdout)
        assert doc['ungraded'] == ["<not a list: 'abc'>"], doc['ungraded']
        assert 'UNEXAMINED' not in doc['reason'], doc['reason']
        assert '`ungraded` is not a list' in doc['reason'], doc['reason']
        r = _verdict_1071(td, led, '{"blocking": 0, "ungraded": ["length"]}')
        assert 'UNEXAMINED, and not passed: length' in \
            json.loads(r.stdout)['reason']
        # ...and a non-list `unknown` names no component, so the verdict must
        # not say one RAN and could not answer (the NO-SCORE branch's rule).
        r = _verdict_1071(td, led, '{"blocking": 1, "unknown": false}')
        doc = json.loads(r.stdout)
        assert 'RAN and could not answer' not in doc['reason'], doc['reason']
        assert '`unknown` is not a list' in doc['reason'], doc['reason']
        r = _verdict_1071(td, led, '{"blocking": 1, "unknown": ["impedance"]}')
        assert 'RAN and could not answer: impedance' in \
            json.loads(r.stdout)['reason']
    print("  PASS: a malformed ungraded/unknown beside a real blocking is "
          "echoed, not raised")


def test_1071_record_refuses_a_score_that_is_not_a_count_object():
    """#1071, #1075, #1078. `record` is the one place to stop a bad score: the
    ledger is append-only. It accepted any JSON, so a per-term dict landed and
    broke every later verdict, and `[1]` was APPENDED and then crashed record
    itself. Refused now, before the ledger or the board store is touched."""
    run_utils = _run_utils()
    base = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'converge.py'), 'record']
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')

        def _argv(score_text, kind='placement'):
            sf = os.path.join(td, 'arg_score.json')
            with open(sf, 'w', encoding='utf-8') as fh:
                fh.write(score_text)
            return base + ['--ledger', led, '--board', BOARD, '--kind', kind,
                           '--lever', 'x', '--score-file', sf]

        # A score that is not an object at all (#1078) -- FIRST, so a
        # regression here reads as this refusal failing, not as a traceback
        # in some later case.
        for raw in ('[1, 2]', '"x"', '5'):
            # `_cv`, not run_utils.check: the #1078 failure IS a traceback
            # after the append, which check() would report as a broken test
            # before the witness that names the defect is reached.
            r = _cv(_argv(raw)[len(base) - 1:])      # from 'record' on
            assert not os.path.exists(led), \
                f'{raw}: the row was APPENDED before record failed (#1078)'
            assert r.returncode == 2 and 'must hold a JSON object' in r.stderr \
                and 'Traceback' not in r.stderr, (raw, r.returncode, r.stderr)
        for raw, phrase in _NOT_A_COUNT_1071:
            r = run_utils.check(_argv('{"blocking": %s}' % raw),
                                refuse='Nothing was written.', code=2)
            assert "--score-file's `blocking` is" in r.stderr, (raw, r.stderr)
            assert phrase in r.stderr, (raw, phrase, r.stderr)
            assert not os.path.exists(led), raw
            assert not os.path.exists(os.path.join(td, 'boards')), raw
        # Not only placement rows: the refusal does not look at --kind, and
        # the film reads every row.
        run_utils.check(_argv('{"blocking": false}', kind='completion'),
                        refuse="`blocking` is the boolean false", code=2)
        # A per-term dict, the #1071 shape, refused with its remedy.
        sf = os.path.join(td, 'score.json')
        with open(sf, 'w', encoding='utf-8') as fh:
            fh.write('{"blocking": {"drc_violations": 3}}')
        run_utils.check(base + ['--ledger', led, '--board', BOARD, '--kind',
                                'placement', '--lever', 'x', '--score-file',
                                run_utils.evidence(sf)],
                        refuse='blocking_by', code=2)
        assert not os.path.exists(led)
        assert not os.path.exists(os.path.join(td, 'boards'))

        # What IS a count, or honestly unmeasured, is still recorded.
        for raw in ('{"blocking": 0}', '{"blocking": 3}', '{"blocking": 2.0}',
                    '{"blocking": null}', '{"quality": {}}'):
            run_utils.check(_argv(raw), accept=True)
        with open(led, encoding='utf-8') as fh:
            assert sum(1 for line in fh if line.strip()) == 5
    print("  PASS: record refuses a blocking that is not a count, and a score "
          "that is not an object, before anything is written")


def test_1078_a_stray_ledger_line_is_skipped_not_raised():
    """#1078. `Ledger.entries()` returned a line that parses to a non-object,
    so one stray `42` made every verdict exit 1 on `.get`."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        with open(led, 'w', encoding='utf-8') as fh:
            fh.write(json.dumps(_row_1071(0, 'completion', 3)) + '\n42\n[1]\n'
                     '"x"\n')
        from board_store import Ledger
        assert len(Ledger(led).entries()) == 1
        r = _verdict_1071(td, led, '{"blocking": 1}')
        assert r.returncode == converge.CONTINUE, (r.returncode, r.stderr)
        assert json.loads(r.stdout)['ledger_rows'] == 1, r.stdout
    print("  PASS: a ledger line that is not an object is skipped, like a torn "
          "one")


def test_1078_a_new_row_is_numbered_past_every_used_iteration():
    """#1078's leftover. `record` numbered a row `len(entries())`; once the
    stray `42` is skipped, a ledger whose later rows were numbered by the old
    count (0, 42, 2) hands out 2 again, and two rows then answer to
    iteration 2."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        with open(led, 'w', encoding='utf-8') as fh:
            fh.write(json.dumps(_row_1071(0, 'placement', 3)) + '\n42\n'
                     + json.dumps(_row_1071(2, 'placement', 2)) + '\n')
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--kind',
                 'placement', '--lever', 'after the stray line'])
        assert r.returncode == 0, (r.returncode, r.stderr)
        from board_store import Ledger
        its = [e.get('iteration') for e in Ledger(led).entries()]
        assert its == [0, 2, 3], its
        # ...and a ledger numbered 0..n-1 keeps the count it always had.
        assert Ledger(os.path.join(td, 'none.jsonl')).next_iteration() == 0
        assert Ledger(led).next_iteration() == 4
    print("  PASS: a new row is numbered past every iteration already used")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
