"""An exhaustion is a claim about the board its own row names (#963).

Run 29 recorded `--exhausted placement` against a board that existed for eight
minutes: `os-promote-placed` overwrote `frozen.kicad_pcb` nine seconds after
the row was written, and the claim then outlived its board and survived three
L5 calls into the run's terminal record. Row 30 retracted rows 25-27 and not
row 29.

The contributor's follow-up reproduced it on two tracked fixtures -- declare on
`tigard_placed`, re-record `tigard_damaged` as a systemic replacement, and the
half stays `declared-exhausted` with the verdict STUCK. `_declaration` read row
ORDER only, and `_is_lap` (correctly) says a `systemic` row is a lap of neither
half, so nothing retracted it and nothing noticed the board had changed.

WHAT IS NOT DONE HERE, and why, because the obvious fix is the wrong one:

  * `flat` and `why` DO NOT CHANGE. "Any digest change invalidates" -- the
    follow-up's own "safe initial implementation" -- is inert-making. Every
    accepted lap of EITHER half writes a new sha, so a placement exhaustion
    would die on the next routing lap although routing copper says nothing
    about placement's remaining levers. Worse, the L2 freeze row changes the
    sha BY CONSTRUCTION ("new file, new content hash") while
    `test_904_not_a_lap::test_a_declaration_survives_a_freeze_and_a_close_out`
    pins that a freeze must not retract. A sha cannot tell "rewrote the file,
    same poses" from "replaced the placement", and `verdict` opens no board.
  * So converge REPORTS (`declared_stale_board`, named on every verdict) and
    the driver's terminal branch REFUSES -- the place where the claim is made
    to a reader. That half is pinned in `loop_driver --dump-refusals` and
    `--self-test`.
  * No new `exhausted.board_sha` field. Every row already carries `result_sha`
    from `store.put(a.board)` and `--board` is required, so the binding is on
    disk on every ledger ever written. A second number for one fact would be
    absent on all of them.

`tests/test_converge.py` keeps the exhaustion ROUND-TRIP (declare, plateau,
retract by running the half again). This file is only about the board binding.
The arithmetic over hand-built rows is in `tests/test_904_not_a_lap.py`, which
owns that idiom; everything here goes through the real CLI.
"""
import io
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
PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')
DAMAGED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                       'tigard_damaged.kicad_pcb')


def _cv(args):
    return subprocess.run([sys.executable, '-X', 'utf8', CV] + args,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT)


def _argv(args):
    return [sys.executable, '-X', 'utf8', CV] + args


def _sha_of(path):
    from board_store import sha256_file
    return sha256_file(path)


def _score(td, name, **kw):
    p = os.path.join(td, name)
    doc = {'blocking': 0, 'quality': {}, 'ungraded': []}
    doc.update(kw)
    with io.open(p, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return p


def _rows(led):
    return [json.loads(x) for x
            in io.open(led, encoding='utf-8').read().splitlines() if x.strip()]


def test_a_declaration_does_not_survive_the_board_it_was_made_about():
    """#963's acceptance line 2, on the contributor's own two fixtures."""
    run_utils.evidence(PLACED)
    run_utils.evidence(DAMAGED)
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', PLACED,
                 '--kind', 'systemic', '--exhausted', 'placement',
                 '--exhausted-reason',
                 'AUDIT TEST: every lever spent on the placed fixture'])
        assert r.returncode == 0, r.stderr
        assert json.loads(r.stdout)['result_sha'] == _sha_of(PLACED), \
            'the binding is result_sha, and it has always been on the row'

        # The board is replaced, exactly as the follow-up did it.
        assert _cv(['record', '--ledger', led, '--board', DAMAGED,
                    '--kind', 'systemic',
                    '--lever', 'AUDIT TEST: systemic board replacement']
                   ).returncode == 0

        sp = _score(td, 's.json', board_sha=_sha_of(DAMAGED))
        doc = json.loads(_cv(['verdict', '--ledger', led,
                              '--score', sp]).stdout)
        st = doc['placement']
        assert st['declared_board'] == _sha_of(PLACED), st
        assert st['declared_stale_board'] == _sha_of(PLACED), (
            'the declaration outlived its board and nothing said so: ' + str(st))
        assert doc['board_sha'] == _sha_of(DAMAGED), doc['board_sha']
        assert doc['board_sha_source'] == 'score', doc['board_sha_source']
        assert 'DECLARED ABOUT ANOTHER BOARD' in doc['reason'], doc['reason']
        assert st['why'] == 'declared-exhausted', (
            'reporting must not retract: a sha cannot tell a rewritten file '
            'from a replaced placement -- ' + str(st))
    print("  PASS: a declaration about another board is named on the verdict")


def test_board_comes_from_the_score_or_from_board_and_says_which():
    """`--board` is the second channel, for a score with no board_sha.

    #694: an aggregate verdict cannot say which of its inputs moved, so the
    input is published beside the answer.
    """
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        assert _cv(['record', '--ledger', led, '--board', PLACED,
                    '--kind', 'systemic', '--exhausted', 'placement',
                    '--exhausted-reason', 'AUDIT TEST']).returncode == 0
        sp = _score(td, 's.json')                       # no board_sha at all

        doc = json.loads(_cv(['verdict', '--ledger', led,
                              '--score', sp]).stdout)
        assert doc['board_sha'] is None, doc['board_sha']
        assert doc['board_sha_source'] is None, doc['board_sha_source']
        assert 'declared_stale_board' not in doc['placement'], (
            'unanswerable must invalidate nothing -- every hand-built score '
            'in the suite lacks board_sha')

        doc = json.loads(_cv(['verdict', '--ledger', led, '--score', sp,
                              '--board', DAMAGED]).stdout)
        assert doc['board_sha'] == _sha_of(DAMAGED), doc['board_sha']
        assert doc['board_sha_source'] == '--board', doc['board_sha_source']
        assert doc['placement']['declared_stale_board'] == _sha_of(PLACED)
    print("  PASS: the binding names the input it came from, or says null")


def test_verdict_refuses_a_board_its_score_does_not_grade():
    """Both channels given and disagreeing is not a verdict to hand anyone."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        assert _cv(['record', '--ledger', led, '--board', PLACED,
                    '--kind', 'placement', '--lever', 'a lap']).returncode == 0
        sp = _score(td, 's.json', board_sha=_sha_of(PLACED))
        run_utils.check(_argv(['verdict', '--ledger', led, '--score', sp,
                               '--board', DAMAGED]),
                        refuse='disagree about which board', code=2)
    print("  PASS: --board and --score disagreeing is refused, not judged")


def test_record_refuses_a_stale_score_on_a_declaration_and_warns_otherwise():
    """The two strengths side by side, so neither drifts onto the other.

    An ordinary row keeps the WARNING -- a baseline row legitimately attaches a
    parent score to a rejected candidate, which `test_converge.py::
    test_record_warns_on_unbound_or_mismatched_score` pins. A declaration has
    no such case: its whole content is "this board's half has nothing left", so
    numbers taken on another board are not weak evidence for it, they are
    evidence about something else.
    """
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        stale = json.dumps({'blocking': 0, 'quality': {}, 'ungraded': [],
                            'board_sha': _sha_of(DAMAGED)})

        r = _cv(['record', '--ledger', led, '--board', PLACED,
                 '--kind', 'completion', '--lever', 'a baseline attachment',
                 '--rejected', '--score', stale])
        assert r.returncode == 0, r.stderr
        assert 'DIFFERENT board' in r.stderr, r.stderr
        row = json.loads(r.stdout)
        assert row['score_stale']['binding'] == 'other', row.get('score_stale')
        assert row['score_stale']['payload_sha'] == _sha_of(DAMAGED)

        run_utils.check(
            _argv(['record', '--ledger', led, '--board', PLACED,
                   '--kind', 'systemic', '--exhausted', 'placement',
                   '--exhausted-reason', 'AUDIT TEST', '--score', stale]),
            refuse='grades a DIFFERENT board', code=2)
        assert len(_rows(led)) == 1, (
            'the refused declaration was written anyway: ' + repr(_rows(led)))
    print("  PASS: a declaration refuses a foreign score; a baseline row warns")


def test_the_stale_attachment_is_on_the_row_not_only_on_stderr():
    """Nothing reads `score_stale` yet, and that is deliberate.

    Making `_score_key` or `_half_state` skip such a row would move plateau
    windows on every ledger already written -- a large behaviour change to hide
    inside a "just record it" line. What it fixes now is that a later reader
    could not tell a warned row from a clean one at all.
    """
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', PLACED,
                 '--kind', 'placement', '--lever', 'no sha at all',
                 '--score', json.dumps({'blocking': 0, 'quality': {}})])
        assert r.returncode == 0, r.stderr
        assert json.loads(r.stdout)['score_stale']['binding'] == 'unbound'

        r = _cv(['record', '--ledger', led, '--board', PLACED,
                 '--kind', 'placement', '--lever', 'bound correctly',
                 '--score', json.dumps({'blocking': 0, 'quality': {},
                                        'board_sha': _sha_of(PLACED)})])
        assert r.returncode == 0, r.stderr
        assert 'score_stale' not in json.loads(r.stdout), \
            'a correctly bound row must not be labelled'
    print("  PASS: an unbound or foreign score is labelled on its own row")


def test_a_declaration_is_not_retracted_by_the_board_moving_on():
    """The anti-"any digest change" case, through the real CLI.

    Declare placement exhausted, then record five ROUTING laps -- each of which
    writes a new board and a new sha, which is what a routing lap IS. Under
    "any digest change invalidates", the placement declaration dies here.
    """
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        assert _cv(['record', '--ledger', led, '--board', PLACED,
                    '--kind', 'systemic', '--exhausted', 'placement',
                    '--exhausted-reason', 'AUDIT TEST: levers spent']
                   ).returncode == 0
        for i in range(5):
            b = DAMAGED if i % 2 else PLACED       # the board keeps changing
            assert _cv(['record', '--ledger', led, '--board', b,
                        '--kind', 'completion', '--lever', 'routing lap ' + str(i),
                        '--score', json.dumps({'blocking': 0, 'quality': {},
                                               'ungraded': [],
                                               'board_sha': _sha_of(b)})]
                       ).returncode == 0, 'routing lap ' + str(i)
        sp = _score(td, 's.json', board_sha=_sha_of(DAMAGED))
        doc = json.loads(_cv(['verdict', '--ledger', led,
                              '--score', sp]).stdout)
        st = doc['placement']
        assert st['why'] == 'declared-exhausted', (
            'five routing laps retracted a PLACEMENT declaration -- routing '
            'copper says nothing about placement levers: ' + str(st))
        assert 'declared_superseded' not in st, st
    print("  PASS: the board moving on does not retract the other half")


if __name__ == '__main__':
    run_utils.evidence(PLACED)
    run_utils.evidence(DAMAGED)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print("--- " + k)
            v()
    print("ALL PASS")
