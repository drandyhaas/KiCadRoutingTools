#!/usr/bin/env python3
"""A row that turned no loop is not a lap, in all three readers (#904, #899).

Three ledger rows are recorded by the doctrine itself and none of them is a turn
of the loop: the L2 FREEZE row (same poses, new file), the `--final` CLOSE-OUT
row, and an `--exhausted` DECLARATION. Every one of them was being counted as a
lap of its half, and each moved a verdict it had no business moving.

Measured on run 25 (esp_prog), reproduced here as fixtures:

  * The freeze row RETRACTED three recorded `--exhausted placement`
    declarations. `_declaration` marks a declaration stale as soon as a later
    row of that half exists -- "the half went back to work" -- and a freeze is
    not going back to work. The only way out was a fourth declaration, written
    to satisfy the instrument rather than to say anything new.
  * The `--final` row MOVED THE VERDICT IT RECORDS: re-running the same L5
    command after its own close-out read "routing improved within its last 5
    laps" (that row carries a score graded over a different component set) and
    answered "not done yet" about a run that had already shipped STUCK. A
    verdict of record reproducible only from the ledger state BEFORE the row
    recording it is the wrong way round.
  * The same row is also the previous "accepted lap" that `record` measures
    commensurability against, so the first ordinary lap after a close-out can
    be refused as a false improvement for grading a different component set --
    which is exactly what a close-out score does by construction.

So the predicate is ONE function, `_is_lap`, and this file pins it at all three
call sites. The third (record's lookback) is the one no issue named and the one
a reader is most likely to "simplify" back to a bare `_HALF` match.
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
import converge as C                                          # noqa: E402

CV = os.path.join(ROOT, 'py_placer', 'converge.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _rows(*rows):
    return [dict(r, iteration=i) for i, r in enumerate(rows)]


def lap(half='placement', **over):
    kind = 'placement' if half == 'placement' else 'completion'
    r = {'kind': kind, 'accepted': True,
         'score': {'blocking': 1, 'quality': {}}}
    r.update(over)
    return r


def test_is_lap_names_the_three_shapes_that_are_not_laps():
    assert C._is_lap(lap(), 'placement')
    assert not C._is_lap(lap(), 'routing'), 'a lap belongs to ONE half'
    assert not C._is_lap({'kind': 'systemic', 'accepted': True}, 'placement')
    assert not C._is_lap({'kind': 'classification', 'accepted': True},
                         'routing')
    assert not C._is_lap(lap(final=True), 'placement'), \
        'a --final row is the RECORD of a verdict, not a turn of the loop'
    assert not C._is_lap(
        lap(exhausted={'half': 'placement', 'reason': 'spent'}), 'placement'), \
        'a declaration changes no board and says so'
    # The shape that could retract its own declaration: the row is a lap of the
    # half it declares finished.
    assert not C._is_lap(
        lap(exhausted={'half': 'placement', 'reason': 'spent'}), 'placement')
    assert not C._is_lap({'accepted': True}, 'placement'), \
        'a row with no kind is in neither half'
    print("  PASS: final rows, declarations and non-half kinds are not laps")


def test_a_declaration_survives_a_freeze_and_a_close_out():
    """The supersession branch -- the half of #904 the issue does not name."""
    dec = {'kind': 'systemic', 'accepted': True,
           'exhausted': {'half': 'placement', 'reason': 'every lever spent'}}

    # A REAL lap after the declaration retracts it. That must keep working:
    # running the half again IS the retraction, and it needs no flag.
    st = C._half_state(_rows(lap(), lap(), dec, lap()), 'placement', 5)
    assert st['why'] == 'too-few-laps' and 'declared_superseded' in st, st

    # The L2 freeze row, recorded as the doctrine now prescribes.
    freeze = {'kind': 'systemic', 'accepted': True,
              'lever': 'L2 freeze: 6 refs the placement half named as decisions'}
    st = C._half_state(_rows(lap(), lap(), dec, freeze), 'placement', 5)
    assert st['flat'] and st['why'] == 'declared-exhausted', st
    assert st['declared'] == 'every lever spent', st

    # And a close-out row of the OTHER half cannot retract it either.
    fin = lap('routing', final=True, stop_condition='STUCK')
    st = C._half_state(_rows(lap(), lap(), dec, fin), 'placement', 5)
    assert st['flat'] and st['why'] == 'declared-exhausted', st

    # A routing close-out does not retract a ROUTING declaration.
    rdec = {'kind': 'systemic', 'accepted': True,
            'exhausted': {'half': 'routing', 'reason': 'pair is parity-fixed'}}
    st = C._half_state(_rows(lap('routing'), rdec, fin), 'routing', 5)
    assert st['flat'] and st['why'] == 'declared-exhausted', st
    print("  PASS: a freeze and a close-out no longer retract a declaration")


def test_a_cross_half_declaration_does_not_retract_the_other_halfs():
    """What the `exhausted` clause of `_is_lap` actually saves.

    A `--kind placement --exhausted routing` row is a row of the PLACEMENT half
    by kind and says nothing about placement at all -- yet it used to count as
    placement going back to work, and retract a live placement declaration.

    A SELF-declaring row (`--kind placement --exhausted placement`) never
    reaches the clause: `_declaration`'s first branch matches on
    `exhausted.half` and re-arms the declaration before the `elif` runs. Both
    arms are asserted here so the clause's reason cannot be misremembered.
    """
    dec = {'kind': 'systemic', 'accepted': True,
           'exhausted': {'half': 'placement', 'reason': 'every lever spent'}}
    cross = lap('placement',
                exhausted={'half': 'routing', 'reason': 'pair parity-fixed'})
    st = C._half_state(_rows(lap(), lap(), dec, cross), 'placement', 5)
    assert st['flat'] and st['why'] == 'declared-exhausted', st

    again = lap('placement',
                exhausted={'half': 'placement', 'reason': 'and again'})
    assert C._declaration(_rows(dec, again), 'placement') == \
        ('and again', True), 'a self-declaring row RE-ARMS, it does not retract'
    print("  PASS: a cross-half declaration is not this half going back to work")


def test_the_window_names_its_rows_on_every_branch():
    """Including the one that returns early."""
    rej = [{'kind': 'placement', 'accepted': False, 'score': None}] * 5
    st = C._half_state(_rows(*rej), 'placement', 5)
    assert st['why'] == 'plateau' and st['flat'], st
    assert st['window_iterations'] == [0, 1, 2, 3, 4], (
        'the all-rejected plateau returned before the evidence was attached -- '
        'the one verdict where a reader most needs to know what was counted')

    # A row with no `iteration` is reported as null, not dropped: `unjudged: 2`
    # beside an empty list is a count contradicting its own detail.
    rows = ([{'kind': 'placement', 'accepted': True,
              'score': {'blocking': 1, 'quality': {}}}] * 3
            + [{'kind': 'placement', 'accepted': True, 'score': None}] * 2)
    st = C._half_state(rows, 'placement', 5)          # NOT _rows: no numbers
    assert st['unjudged'] == 2 and st['unjudged_iterations'] == [None, None], st
    assert len(st['window_iterations']) == 5, st

    # ...and the key is ABSENT, not [], when nothing is unjudged.
    st = C._half_state(_rows(lap(), lap()), 'placement', 2)
    assert 'unjudged_iterations' not in st, st
    print("  PASS: every branch names its window; an unnumbered row says so")


def test_the_final_row_does_not_move_the_verdict_it_records():
    flat = [lap('routing', score={'blocking': 0,
                                  'quality': {'vias': 22, 'copper_mm': 230}})
            for _ in range(5)]
    before = C._half_state(_rows(*flat), 'routing', 5)
    assert before['flat'] and before['why'] == 'plateau', before

    # score_final.json grades different components and a better quality tuple:
    # counted as a lap, this reads as an improvement and flips the verdict.
    fin = lap('routing', final=True, stop_condition='STUCK',
              score={'blocking': 0, 'quality': {'vias': 20, 'copper_mm': 210}})
    after = C._half_state(_rows(*(flat + [fin])), 'routing', 5)
    assert after['why'] == before['why'] == 'plateau', (before, after)
    assert after['laps'] == before['laps'] == 5, (before, after)
    assert after['window_iterations'] == before['window_iterations'], \
        'the close-out must not even enter the window it is decided from'
    print("  PASS: L5 re-run after its own --final row keeps its verdict")


def test_no_comparison_names_the_rows_it_could_not_judge():
    unjudged = lap(score=None)
    rows = _rows(lap(), lap(), unjudged, unjudged, lap())
    st = C._half_state(rows, 'placement', 5)
    assert st['why'] == 'no-comparison' and st['blocked'] == 'unjudged', st
    assert st['unjudged_iterations'] == [2, 3], st
    assert st['window_iterations'] == [0, 1, 2, 3, 4], st

    # ...and the number reaches the operator, who is the one who has to act on
    # it. `verdict` is the only surface that prints this, so assert the text.
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        with open(led, 'w', encoding='utf-8') as fh:
            for row in rows + _rows(*[lap('routing') for _ in range(5)]):
                fh.write(json.dumps(row) + '\n')
        sf = os.path.join(td, 's.json')
        with open(sf, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'board-score', 'blocking': 1,
                       'quality': {}}, fh)
        run_utils.evidence(led, 'the fixture ledger')
        run_utils.evidence(sf, 'the fixture score')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'verdict', '--ledger', led,
             '--score', sf, '--flat', '5'],
            capture_output=True, text=True, cwd=ROOT)
        doc = json.loads(r.stdout)
        assert doc['verdict'] == 'CONTINUE', doc
        assert 'iteration(s) 2, 3' in doc['reason'], doc['reason']
        # The half is NOT improving, and the key must not say it is: that is
        # what the driver writes its headline from.
        assert doc.get('improving') == [], doc
        assert 'placement' in (doc.get('unanswerable') or []), doc
    print("  PASS: an unjudged window names its iterations")


def test_a_lap_after_a_close_out_is_not_measured_against_it():
    """record's commensurability lookback -- the third call site."""
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        # The ordinary laps of this run graded `unrouted` only. The CLOSE-OUT
        # was graded with the full final flag set, so it also carries an
        # impedance count -- which is what score_final.json is FOR, and the
        # reason a close-out score is never commensurable with a lap's.
        thin = {'schema': 1, 'kind': 'board-score', 'blocking': 5,
                'blocking_by': {'unrouted': 5, 'impedance': None},
                'ungraded': ['impedance'],
                'components': {'unrouted': {'ran': True, 'count': 5},
                               'impedance': {'ran': False, 'count': None}},
                'quality': {}}
        rich = {'schema': 1, 'kind': 'board-score', 'blocking': 9,
                'blocking_by': {'unrouted': 5, 'impedance': 4},
                'ungraded': [],
                'components': {'unrouted': {'ran': True, 'count': 5},
                               'impedance': {'ran': True, 'count': 4}},
                'quality': {}}
        with open(led, 'w', encoding='utf-8') as fh:
            fh.write(json.dumps({'iteration': 0, 'kind': 'completion',
                                 'accepted': True, 'score': thin}) + '\n')
            fh.write(json.dumps({'iteration': 1, 'kind': 'completion',
                                 'accepted': True, 'final': True,
                                 'stop_condition': 'STUCK',
                                 'score': rich}) + '\n')
        sf = os.path.join(td, 's.json')
        with open(sf, 'w', encoding='utf-8') as fh:
            json.dump(thin, fh)          # graded exactly like iteration 0
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', CV, 'record', '--ledger', led,
             '--board', BOARD, '--kind', 'completion',
             '--lever', 'an ordinary lap after a close-out',
             '--score-file', sf, '--argv', sys.executable, '-c', 'pass'],
            capture_output=True, text=True, cwd=ROOT)
        # Measured against iteration 0 this lap is IDENTICAL -- same component
        # set, same total. Measured against the close-out it grades one
        # component fewer and reads 5 against 9, which is the exact shape the
        # refusal exists for: a false improvement bought by measuring less.
        assert r.returncode == 0, (
            'a lap graded exactly like the last REAL lap was refused, because '
            'it was compared against the close-out row instead:\n' + r.stderr)
        assert 'FEWER components' not in r.stderr, r.stderr
    print("  PASS: the previous lap is a lap, not the close-out")


if __name__ == '__main__':
    run_utils.evidence(BOARD)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
