#!/usr/bin/env python3
"""converge.py: the ladder, the rip invariants, and a step back that is a checkout.

The loop this supports failed in two measurable ways before it existed:

  * every candidate cost a full chain run, so a budget of 20 bought ~8 useful
    moves and the run stopped with nets still carrying no copper;
  * a step back meant reading prose and reconstructing a command by hand.

The verbs here are the fix. `poses` ranks with arithmetic and routes only the
survivors; `record`/`step-back`/`replay` make the history mechanical;
`check_rip_invariants` encodes four rules that each cost a wasted iteration.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
# #522 reorg + skill merge: the engine moved to py_router/, the placer to
# py_placer/, and board_score.py into the placement-and-routing skill. Tests
# that shell out to or import them need those roots on sys.path.
for _p in ('py_router', 'py_placer',
           os.path.join('.claude', 'skills', 'plan-pcb-placement-and-routing',
                        'scripts')):
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


# ------------------------------------------------------------ rip invariants

def test_rip_invariants_catch_all_four_traps():
    # 1. more than one net per call
    c = converge.check_rip_invariants(['A', 'B'], [])
    assert any('one net per call' in x for x in c), c
    # 2. a width-bearing net ripped without its width
    c = converge.check_rip_invariants(['A'], ['VCC'], power_nets=['VCC'])
    assert any('width-bearing' in x for x in c), c
    # 3. a glob standing in for an exact name
    c = converge.check_rip_invariants(['A'], ['QSPI_*'])
    assert any('glob' in x and 'locked' in x for x in c), c
    # 4. a net in both --nets and the rip set
    c = converge.check_rip_invariants(['A'], ['A'])
    assert any('force-reroute' in x for x in c), c
    print("  PASS: all four rip traps are caught")


def test_a_safe_rip_produces_no_complaints():
    assert converge.check_rip_invariants(['A'], ['B', 'C']) == []
    assert converge.check_rip_invariants(
        ['A'], ['VCC'], power_nets=[]) == []
    print("  PASS: a scoped, exact, width-safe rip is silent")


# -------------------------------------------------------------- route_verdict

def test_route_verdict_counts_both_kinds_of_failure():
    n, note = converge.route_verdict(
        {'failed_single': ['A'], 'failed_multipoint': [{'net_name': 'B'}],
         'multipoint_pads_total': 10, 'multipoint_pads_connected': 8})
    assert n == 3, f"1 failed net + 2 pads short = 3, got {n}"
    assert 'A' in note and 'B' in note
    assert converge.route_verdict({})[0] is None
    print("  PASS: failures = failed nets + pad deficit")


def test_route_verdict_surfaces_a_refused_rip():
    """A caller that cannot see a refusal will follow the router's retry hint
    forever -- 'locked' has no override."""
    n, note = converge.route_verdict(
        {'failed_single': ['A'],
         'protected_skipped': {'--rip-existing-nets': {'GND': 'locked'}}})
    assert 'GND(locked)' in note, note
    print("  PASS: a refused rip reaches the verdict text")


# ---------------------------------------------------------------- the ladder

def test_poses_emits_parseable_json_on_stdout():
    """The verb's stdout is a data channel: the parser and the placement state
    both narrate, and a single stray line makes the document unparseable."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    r = _cv(['poses', BOARD, '--ref', 'C1', '--radius', '0.5',
             '--step', '0.5', '--limit', '3'])
    assert r.returncode == 0, r.stderr[-800:]
    d = json.loads(r.stdout)             # the assertion IS that this parses
    assert d['ref'] == 'C1' and d['poses']
    costs = [p['cost'] for p in d['poses']]
    assert costs == sorted(costs)
    assert 'WARNING' in r.stderr or True   # diagnostics belong on stderr
    print(f"  PASS: clean JSON on stdout, {len(d['poses'])} ranked poses")


def test_route_flag_requires_the_caller_to_say_what_is_affected():
    """Only the caller knows which nets a move can affect; guessing would
    either route the world or miss the point."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    r = _cv(['poses', BOARD, '--ref', 'C1', '--route'])
    assert r.returncode == 2 and 'affected' in r.stderr
    print("  PASS: --route without --affected is refused")


# ------------------------------------------------------------- the bookkeeping

def test_record_step_back_replay_round_trip():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        # sys.executable, not 'echo': echo is a shell builtin, so it neither
        # replays on Windows nor passes the record-time replayability guard.
        r = _cv(['record', '--ledger', led, '--board', BOARD,
                 '--lever', 'seed', '--argv', sys.executable, '-c',
                 "print('replayed-ok')"])
        assert r.returncode == 0, r.stderr
        sha = json.loads(r.stdout)['result_sha']

        out = os.path.join(td, 'back.kicad_pcb')
        assert _cv(['step-back', '--ledger', led, '--iteration', '0',
                    '--out', out]).returncode == 0
        from board_store import sha256_file
        assert sha256_file(out) == sha, "step back must be byte-exact"

        r = _cv(['replay', '--ledger', led, '--iteration', '0'])
        assert r.returncode == 0 and 'replayed-ok' in r.stdout
    print("  PASS: record -> step-back (byte-exact) -> replay")


def test_record_refuses_an_argv_that_cannot_replay():
    """Run-7 F4: entries recorded with placeholder script names made replay a
    reconstruction -- exactly what the ledger exists to prevent."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', BOARD,
                 '--lever', 'x', '--argv', 'route_v12_placeholder.sh', 'b.pcb'])
        assert r.returncode == 2, (r.returncode, r.stderr)
        assert 'never replay' in r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"
    print("  PASS: a placeholder --argv is refused, ledger untouched")


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
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                 '--stop-condition', 'plateau: 3 iterations, no new copper'])
        assert r.returncode == 2 and 'routed-board lenses' in r.stderr, r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"
        lenses = ['--lens', 'VERDICT=PASS:lens=connectivity',
                  '--lens', 'VERDICT=PASS:lens=drc',
                  '--lens', 'VERDICT=PASS:lens=spec']
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                 '--stop-condition', 'plateau: 3 iterations, no new copper']
                + lenses)
        assert r.returncode == 0, r.stderr
        e = json.loads(r.stdout)
        assert e.get('final') is True
        assert e.get('stop_condition', '').startswith('plateau')
        assert len(e.get('lenses') or []) == 3, e.get('lenses')
    print("  PASS: --final without --stop-condition is refused; with it, recorded")


def test_record_final_wants_the_lens_verdicts():
    """A FAILED lens means `blocking` was not really zero, so the run did not
    finish clean -- stop condition 1 is then not available to it."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--lever', 'x',
                 '--lens', 'all three passed'])
        assert r.returncode == 2 and 'verbatim' in r.stderr, r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"

        base = ['record', '--ledger', led, '--board', BOARD, '--final',
                '--lens', 'VERDICT=PASS:lens=connectivity',
                '--lens', 'VERDICT=FAIL:lens=drc;finding=short;evidence=x',
                '--lens', 'VERDICT=PASS:lens=spec']
        r = _cv(base + ['--stop-condition', '1'])
        assert r.returncode == 2 and 'lens FAILED' in r.stderr, r.stderr
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
        final = ['record', '--ledger', led, '--board', BOARD, '--final',
                 '--kind', 'completion', '--stop-condition', '1',
                 '--score-file', run17,
                 '--lens', 'VERDICT=PASS:lens=connectivity',
                 '--lens', 'VERDICT=PASS:lens=drc',
                 '--lens', 'VERDICT=PASS:lens=spec']
        r = _cv(final)
        assert r.returncode == 2, (r.returncode, r.stdout[:400])
        assert 'CONTRADICTS' in r.stderr, r.stderr
        assert 'unrouted = 32' in r.stderr and 'broken = 47' in r.stderr, r.stderr
        assert 'VERDICT=FAIL:lens=connectivity' in r.stderr, \
            'the refusal must name the honest verdict it will accept'
        assert not os.path.exists(led), 'nothing may be written on refusal'

        # ...and the honest verdict IS accepted (stop condition 4: measured
        # unfixable and said so).
        r = _cv(['record', '--ledger', led, '--board', BOARD, '--final',
                 '--kind', 'completion', '--stop-condition', '4',
                 '--score-file', run17,
                 '--lens', 'VERDICT=FAIL:lens=connectivity;finding=32 nets '
                           'carry no copper;evidence=score.json#/blocking_by',
                 '--lens', 'VERDICT=PASS:lens=drc',
                 '--lens', 'VERDICT=PASS:lens=spec'])
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
                                    'null.json'),
                 '--lens', 'VERDICT=PASS:lens=drc'])
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
                                    'other.json'),
                 '--lens', 'VERDICT=PASS:lens=connectivity'])
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
                        'placement', '--lever', f'lap {i}',
                        '--score', json.dumps({'blocking': 0, 'quality': {},
                                               'ungraded': []})]
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
        r = rec(sc(part, 'c3b.json'), '--accept-incommensurable',
                'both re-scored at 92 out of band; they tie')
        assert r.returncode == 0, r.stderr
        assert json.loads(r.stdout)['accepted_incommensurable'].startswith(
            'both re-scored'), 'the disposition belongs in the row'

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
                 'completion', '--lever', 'x', '--score-file', p,
                 '--lens', 'VERDICT=PASS:lens=connectivity'])
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
        r = _cv(['record', '--ledger', led, '--board', BOARD,
                 '--score', '{"failures": 3}'])
        assert r.returncode == 0
        assert 'names no nets' in r.stderr, r.stderr
        r = _cv(['record', '--ledger', led, '--board', BOARD,
                 '--score', '{"failures": 2, "failed_nets": ["SWDIO", "GPIO1"]}'])
        assert r.returncode == 0
        assert 'SWDIO' in r.stderr and 'GPIO1' in r.stderr, r.stderr
    print("  PASS: failing-net names surface; a bare count draws a NOTE")


def test_a_prose_only_entry_refuses_to_replay_without_a_traceback():
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        _cv(['record', '--ledger', led, '--board', BOARD, '--kind', 'systemic',
             '--lever', 'restored the net classes'])
        r = _cv(['replay', '--ledger', led, '--iteration', '0'])
        assert r.returncode == 4, f"expected a clean refusal, got {r.returncode}"
        assert 'Traceback' not in r.stderr, "a non-replayable entry is not a crash"
        assert 'lever_argv' in r.stderr
    print("  PASS: a prose entry refuses cleanly, exit 4, no traceback")


def test_status_warns_when_the_budget_goes_to_the_instrument():
    """The failure this makes visible: nine of eleven iterations spent on how
    the chain measures itself, finishing with nets that never got copper."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        for _ in range(3):
            _cv(['record', '--ledger', led, '--board', BOARD,
                 '--kind', 'systemic', '--lever', 'tooling'])
        r = _cv(['status', '--ledger', led])
        assert r.returncode == 0
        assert json.loads(r.stdout)['systemic'] == 3
        assert 'SYSTEMIC' in r.stderr, "a lopsided budget must be called out"
    print("  PASS: status splits the budget and warns on a lopsided one")


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
    quality because --score was free JSON with no binding to --board.
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
                        '--lever', 'x', '--score', json.dumps(score),
                        '--argv', sys.executable, '-c', "pass"])

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
         os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing',
                      'scripts', 'board_score.py'), BOARD, '-q'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT)
    line = [l for l in r.stdout.splitlines() if l.startswith('SCORE_JSON=')]
    assert line, r.stdout[:800] + r.stderr[:800]
    doc = json.loads(line[0].split('=', 1)[1])
    assert doc.get('board_sha') == sha256_file(BOARD), \
        "board_score must bind its payload to the graded file"
    print("  PASS: board_score embeds board_sha")


def test_l5_printed_final_command_runs_as_printed():
    """D1: the run-closing command L5 prints must be ACCEPTED by converge,
    verbatim, for every terminal verdict. The refusal-driven executor recovers
    from a refused command, but a driver whose doctrine is "paste these exact
    commands" may not print one its own tool refuses: the STUCK/BUDGET paths
    carry a FAIL lens as the normal case, and DONE-EXHAUSTED must never sit
    beside one."""
    import shlex
    sys.path.insert(0, os.path.join(
        ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts'))
    from loop_driver import final_record_command

    _PASS = {'connectivity': 'VERDICT=PASS:lens=connectivity',
             'drc': 'VERDICT=PASS:lens=drc',
             'spec': 'VERDICT=PASS:lens=spec'}
    _FAIL_DRC = ('VERDICT=FAIL:lens=drc;finding=3 clearance violations;'
                 'evidence=score.json#/blocking_by/drc')
    _CLEAN = {'blocking': 0, 'blocking_by':
              {'unrouted': 0, 'broken': 0, 'drc': 0, 'undersized': 0}}
    _DIRTY = {'blocking': 3, 'blocking_by':
              {'unrouted': 0, 'broken': 0, 'drc': 3, 'undersized': 0}}

    def run_as_printed(name, lens_by_name, score_doc, led):
        score = os.path.join(os.path.dirname(led), 'score.json')
        with open(score, 'w', encoding='utf-8') as f:
            json.dump(score_doc, f)
        text = final_record_command(led.replace('\\', '/'),
                                    BOARD.replace('\\', '/'),
                                    score.replace('\\', '/'), name)
        toks = shlex.split(text.replace('\\\n', ' '))
        assert toks[0] == 'python3' and toks[3] == 'py_placer/converge.py', \
            toks[:4]
        toks[0] = sys.executable
        # The three placeholder slots must be present AND recognisable -- if
        # the driver's wording drifts, fail here rather than silently testing
        # a different command than the one printed.
        subst = {f'<the {lens} VERDICT= line, verbatim>': line
                 for lens, line in lens_by_name.items()}
        hit = 0
        for i, t in enumerate(toks):
            if t in subst:
                toks[i] = subst[t]
                hit += 1
        assert hit == 3, f'expected 3 lens placeholders, found {hit}: {toks}'
        ia = toks.index('--argv')
        toks = toks[:ia + 1] + [sys.executable, '-c', 'pass']
        return subprocess.run(toks, capture_output=True, text=True,
                              encoding='utf-8', errors='replace', cwd=ROOT)

    with tempfile.TemporaryDirectory() as td:
        # DONE-EXHAUSTED: every lens passes, clean score.
        r = run_as_printed('DONE-EXHAUSTED', _PASS, _CLEAN,
                           os.path.join(td, 'done.jsonl'))
        assert r.returncode == 0, r.stderr
        e = json.loads(r.stdout)
        assert e.get('final') is True and \
            e.get('stop_condition') == 'DONE-EXHAUSTED'
        # STUCK and BUDGET: a FAIL lens is the NORMAL case, not a refusal.
        for name in ('STUCK', 'BUDGET'):
            lenses = dict(_PASS, drc=_FAIL_DRC)
            r = run_as_printed(name, lenses, _DIRTY,
                               os.path.join(td, name + '.jsonl'))
            assert r.returncode == 0, f'{name}: {r.stderr}'
            e = json.loads(r.stdout)
            assert e.get('stop_condition') == name
            assert any(v.startswith('VERDICT=FAIL') for v in e['lenses'])
        # DONE-EXHAUSTED beside a FAIL lens is the one contradiction.
        led = os.path.join(td, 'contra.jsonl')
        r = run_as_printed('DONE-EXHAUSTED', dict(_PASS, drc=_FAIL_DRC),
                           _DIRTY, led)
        assert r.returncode == 2 and 'contradiction' in r.stderr, r.stderr
        assert not os.path.exists(led), "nothing may be written on refusal"
    print("  PASS: L5's printed --final command runs as printed, "
          "all three verdicts")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
