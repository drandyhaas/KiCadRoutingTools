"""A retry needs a DECISION on the record, not a process that made one (#963).

Run 29's combined loop never invoked `L3` (classify) or `L4` (re-enter) -- zero
times in 439 commands -- while `L5` PRINTED the `--stage L3` command in three of
its six emissions. The mechanical root is that CONTINUE returns
`<stage_instructions>`, and `main` derives the exit code from whether the text
starts with `<error>`, so the advice sat inside a success. Fifty-seven inline
routing calls then ran against an unclassified blocker, and the run recorded a
"geometrically unsatisfiable" stop that an outside reader refuted in one pass
(BLOCKING 2 -> 0) using a board the routing half had already written and the
ledger named zero times.

WHY THE GATE IS ON A ROW AND NOT ON A STAGE. #963's follow-up objects, rightly,
that requiring an `L3` invocation or a fresh prompt file would refuse a model
that diagnosed the failure inline or through another harness. So what is
required is the EVIDENCE: a `--kind classification --shape <...>` row, which any
topology can write, which changes no board, and which enters neither half's
plateau window -- so producing it cannot move a verdict, only make one
available. `test_the_refusal_asks_for_a_row_and_names_no_process` is the pin
that keeps it that way; a future editor restoring the stage command to the
refusal text is the likeliest way this becomes topology-bound again.

THREE CONJUNCTS, and every one of them is a way the gate would otherwise be
wrong rather than a belt-and-braces guard:

  * `blocking != 0`. L3 at `blocking == 0` returns "nothing to classify" and
    tells the reader not to bounce back, so a quality-polishing run CANNOT
    produce the row -- gating it would refuse such a run forever.
  * ROUTING laps only. A placement lap after `shape=placement` is the
    classification being ACTED ON.
  * "No classification row anywhere" is its own arm. "Laps since the last
    classification" is vacuously false when there has never been one -- which
    is precisely run 29 -- so a predicate without this arm cannot catch the
    case it was written for.

The threshold is ONE lap per decision, derived rather than tuned: L4's parameter
arm says "Re-enter the FAILING ROUTING STEP ... Record it, then go back to L3
with the new score". The first lap after a decision is the lap that decision
bought; the second is the first lap no decision authorised.
"""
import io
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'scripts')
for _p in (os.path.join(ROOT, 'tests'), os.path.join(ROOT, 'py_placer'),
           SCRIPTS, ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                               # noqa: E402
import converge as C                                           # noqa: E402
import loop_driver as L                                        # noqa: E402

CV = os.path.join(ROOT, 'py_placer', 'converge.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _rows(*rows):
    return [dict(r, iteration=i) for i, r in enumerate(rows)]


def lap(half='routing', **over):
    r = {'kind': 'placement' if half == 'placement' else 'completion',
         'accepted': True, 'score': {'blocking': 2, 'quality': {}}}
    r.update(over)
    return r


def classification(shape='parameter', **over):
    r = {'kind': 'classification', 'accepted': True, 'shape': shape,
         'lever': 'the escape faces are saturated'}
    r.update(over)
    return r


def _cv(args):
    return subprocess.run([sys.executable, '-X', 'utf8', CV] + args,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT)


def _argv(args):
    return [sys.executable, '-X', 'utf8', CV] + args


def _ledger(td, name, rows):
    p = os.path.join(td, name)
    with io.open(p, 'w', encoding='utf-8') as fh:
        for i, r in enumerate(rows):
            fh.write(json.dumps(dict(r, iteration=i)) + '\n')
    return p


def _score(td, name, blocking=2):
    p = os.path.join(td, name)
    with io.open(p, 'w', encoding='utf-8') as fh:
        json.dump({'blocking': blocking, 'quality': {}, 'ungraded': []}, fh)
    return p


def _l5(td, rows, blocking=2, extra=()):
    return L.STAGES['L5'](L._args(
        ['--board', BOARD,
         '--ledger', _ledger(td, 'l.jsonl', rows),
         '--score', _score(td, 's.json', blocking)] + list(extra)))


def test_classification_state_counts_laps_since_the_decision():
    """And counts them with `_is_lap`, so the rows that turn no loop are out.

    #904 made that ONE predicate for a measured reason: three readers
    disagreed, and a freeze row silently retracted three recorded declarations.
    Counting here with a fresh kind-match would have been a fourth reader.
    """
    assert C._classification_state(_rows(lap(), lap())) is None, \
        'no classification row at all must be None, not an empty state'
    st = C._classification_state(_rows(lap(), classification(), lap(),
                                       lap('placement')))
    assert st['shape'] == 'parameter', st
    assert st['iteration'] == 1, st
    assert st['laps_since'] == {'routing': 1, 'placement': 1}, st

    # The three shapes that are NOT laps, in the window after the decision.
    st = C._classification_state(_rows(
        classification(),
        lap(final=True, stop_condition='STUCK'),
        lap(exhausted={'half': 'routing', 'reason': 'spent'}),
        {'kind': 'systemic', 'accepted': True, 'lever': 'a freeze'}))
    assert st['laps_since']['routing'] == 0, (
        'a close-out, a declaration and a freeze turn no loop: ' + str(st))

    # The LAST one wins, and its own shape is the one reported.
    st = C._classification_state(_rows(classification('parameter'), lap(),
                                       classification('floorplan')))
    assert (st['shape'], st['laps_since']['routing']) == ('floorplan', 0), st
    print("  PASS: the last decision, and the laps recorded after it")


def test_the_second_unclassified_routing_lap_is_refused():
    """One lap per decision, and the second is the one nothing authorised."""
    with tempfile.TemporaryDirectory() as td:
        out = _l5(td, [lap()])
        assert not out.startswith('<error>'), (
            'the first lap is the lap the last decision bought:\n' + out[:400])
        out = _l5(td, [lap(), lap()])
        assert out.startswith('<error>'), out[:400]
        assert 'kind classification' in out and '--shape' in out, out[:600]
        assert 'no classification row has ever been written' in out, out[:600]
    print("  PASS: the second unclassified routing lap refuses, naming the row")


def test_the_refusal_asks_for_a_row_and_names_no_process():
    """The pin that keeps this evidence-bound rather than topology-bound.

    #963's follow-up: requiring an L3 invocation, or a fresh prompt file, would
    refuse a model that diagnosed correctly by another route. Restoring the
    stage command to this text is the likeliest way that comes back, because it
    reads like helpfulness.
    """
    with tempfile.TemporaryDirectory() as td:
        out = _l5(td, [lap(), lap()])
    assert out.startswith('<error>'), out[:200]
    for banned in ('--stage L3', 'route_prompt', 'place_prompt', '--delegate'):
        assert banned not in out, (
            f'the refusal names {banned!r}, which demands a PROCESS. It may '
            f'ask only for the row: ' + out[:400])
    assert 'not a demand that a particular stage ran' in out, out[:400]
    print("  PASS: the refusal demands a row, not a process")


def test_a_recorded_classification_clears_it_and_moves_no_verdict():
    """The escape is one command, which is why there is no waiver for it.

    It is only an escape if writing the row is genuinely free: `_HALF` has no
    `classification` key, so the row enters neither plateau window and the
    commensurability lookback skips it. If that stopped being true, the gate
    would be charging a run for satisfying it.
    """
    with tempfile.TemporaryDirectory() as td:
        rows = [lap(), lap()]
        assert _l5(td, rows).startswith('<error>')
        out = _l5(td, rows + [classification()])
        assert not out.startswith('<error>'), out[:400]

        # ...and the halves read exactly the same before and after the row.
        led_before = _ledger(td, 'b.jsonl', rows)
        led_after = _ledger(td, 'a.jsonl', rows + [classification()])
        sp = _score(td, 'v.json')
        before = json.loads(_cv(['verdict', '--ledger', led_before,
                                 '--score', sp]).stdout)
        after = json.loads(_cv(['verdict', '--ledger', led_after,
                                '--score', sp]).stdout)
        for half in ('placement', 'routing'):
            assert before[half] == after[half], (
                f'recording a classification moved the {half} half: '
                f'{before[half]} -> {after[half]}')
        assert before['verdict'] == after['verdict'], (before['verdict'],
                                                       after['verdict'])
    print("  PASS: the row clears the gate and moves neither half")


def test_the_three_ways_it_must_not_fire():
    """A gate measured only by what it refuses has an unmeasured cost."""
    with tempfile.TemporaryDirectory() as td:
        out = _l5(td, [lap(), lap()], blocking=0)
        assert not out.startswith('<error>'), (
            'blocking == 0 must never be refused: L3 returns "nothing to '
            'classify" there, so the row the gate wants cannot be produced:\n'
            + out[:400])

        out = _l5(td, [lap('placement')] * 4)
        assert not out.startswith('<error>'), (
            'placement laps are not routing laps:\n' + out[:400])

        out = _l5(td, [classification('placement'), lap('placement'),
                       lap('placement')])
        assert not out.startswith('<error>'), (
            'placement laps after shape=placement are the classification '
            'being ACTED ON, not evidence that it went unread:\n' + out[:400])
    print("  PASS: blocking 0, placement laps, and acting on the decision")


def test_the_waiver_is_reason_bearing_and_separate():
    with tempfile.TemporaryDirectory() as td:
        out = _l5(td, [lap(), lap()],
                  extra=['--accept-unclassified', 'the board is gone'])
        assert not out.startswith('<error>'), out[:400]
        # It is NOT --accept-unclosed's vocabulary: that one is the terminal
        # branch's, and sharing it would let a close-out waiver clear a retry.
        out = _l5(td, [lap(), lap()],
                  extra=['--accept-unclosed', 'verifier'])
        assert out.startswith('<error>'), (
            '--accept-unclosed cleared a gate that is not its own:\n'
            + out[:400])
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(SCRIPTS, 'loop_driver.py'), '--stage', 'L5',
         '--board', BOARD, '--accept-unclassified'],
        capture_output=True, text=True, cwd=ROOT)
    assert r.returncode == 2, (
        'a bare --accept-unclassified must not waive anything: '
        f'exit {r.returncode}\n{r.stderr[-300:]}')
    print("  PASS: the waiver needs a reason, and is its own vocabulary")


def test_a_classification_row_must_name_a_shape():
    """Otherwise the gate is a formality one flagless command clears."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        run_utils.check(
            _argv(['record', '--ledger', led, '--board', BOARD,
                   '--kind', 'classification', '--lever', 'a decision']),
            refuse='needs --shape', code=2)
        assert not os.path.exists(led), 'nothing may be written on refusal'
        assert _cv(['record', '--ledger', led, '--board', BOARD,
                    '--kind', 'classification', '--shape', 'floorplan',
                    '--lever', 'a decision']).returncode == 0
    print("  PASS: a classification names the shape it decided")


def test_a_measured_unfixable_close_out_needs_a_live_classification():
    """Stop condition 4 is the claim run 29 recorded falsely.

    Bound to the row that MAKES the claim rather than to the stage that prints
    it: run 29's close-out was written without L5's advice carrying at all, so
    a gate in the driver would have been one more thing to walk past.
    """
    lenses = []
    with tempfile.TemporaryDirectory() as td:
        for name in ('connectivity', 'drc', 'spec'):
            p = os.path.join(td, f'verdict_{name}.txt')
            io.open(p, 'w', encoding='utf-8').write(
                f'VERDICT=PASS:lens={name}\n')
            lenses += ['--lens-file', p]
        led = os.path.join(td, 'l.jsonl')
        base = ['record', '--ledger', led, '--board', BOARD, '--final',
                '--kind', 'completion', '--stop-condition', '4'] + lenses
        run_utils.check(_argv(base),
                        refuse='no classification row was ever recorded',
                        code=2)

        # A PLATEAU is a different claim and needs none of this.
        assert _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                    '--kind', 'completion', '--stop-condition', 'STUCK']
                   + lenses).returncode == 0, 'STUCK is not an unfixability claim'

        led2 = os.path.join(td, 'l2.jsonl')
        assert _cv(['record', '--ledger', led2, '--board', BOARD,
                    '--kind', 'classification', '--shape', 'placement',
                    '--lever', 'no lane exists at the escape faces']
                   ).returncode == 0
        assert _cv(['record', '--ledger', led2, '--board', BOARD, '--final',
                    '--kind', 'completion', '--stop-condition', '4'] + lenses
                   ).returncode == 0, 'a classified 4 must be accepted'

        # ...and a routing lap AFTER the decision makes the claim stale again.
        assert _cv(['record', '--ledger', led2, '--board', BOARD,
                    '--kind', 'completion', '--lever', 'one more arm']
                   ).returncode == 0
        led3 = os.path.join(td, 'l3.jsonl')
        io.open(led3, 'w', encoding='utf-8').write(
            io.open(led2, encoding='utf-8').read())
        run_utils.check(
            _argv(['record', '--ledger', led3, '--board', BOARD, '--final',
                   '--kind', 'completion', '--stop-condition', '4'] + lenses),
            refuse='routing lap(s) were recorded after the last '
                   'classification', code=2)
    print("  PASS: a measured-unfixable claim needs the measurement recorded")


def test_l3_and_l4_print_the_command_that_satisfies_the_gate():
    """A refusal whose remedy nothing emits is a remedy nobody runs.

    L4's step 1 has said "Record the classification in the ledger" in prose
    with NO command since it was written, and run 29 recorded none at all.
    """
    dump = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(SCRIPTS, 'loop_driver.py'), '--dump-all'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT)
    assert dump.returncode == 0, dump.stderr[-400:]
    for arm in ('L3', 'L4'):
        body = dump.stdout.split(f'===== {arm} =====', 1)[1].split('=====', 1)[0]
        assert '--kind classification' in body, (
            f'{arm} does not emit the record command its own text asks for')
        assert '--shape' in body, arm
    print("  PASS: L3 and L4 emit the row the gate asks for")


if __name__ == '__main__':
    run_utils.evidence(BOARD)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print("--- " + k)
            v()
    print("ALL PASS")
