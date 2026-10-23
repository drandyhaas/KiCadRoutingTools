#!/usr/bin/env python3
"""quick_score declares what it leaves out, so converge compares it honestly (#1323).

quick_score grades the connectivity of a `route.py --quick 2` board's
unpoured nets and nothing else, but its payload carried no `ungraded` list and
no null `blocking_by` key. `converge.ungraded_set` therefore read it as
grading EVERYTHING, and the first board_score row after it -- which honestly
lists impedance, length and net_widths as ungraded -- was refused as an
improvement that "graded FEWER components" (exit 2, nothing written). The
skill records quick and full rows in one ledger, so four teammates hit it.

Checks:
  1. the payload lists, by board_score's component names, every component
     quick_score does not grade, each with a reason converge can quote;
  2. converge reads a quick row followed by a board_score row as
     incommensurable but NOT a false improvement (the full row graded more);
  3. the genuinely dangerous order still refuses: a board_score row followed
     by a quick row whose `blocking` is lower;
  4. end to end, `converge.py record` accepts the board_score row after the
     quick row (exit 0, two ledger rows), and refuses the reverse for the
     stated reason.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _d) if _d else ROOT
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import board_score                                            # noqa: E402
import converge                                               # noqa: E402
import quick_score                                            # noqa: E402
from run_utils import check, evidence                         # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

# A board_score payload of the shape the gp14 repro recorded: blocking 0 with
# --intent and --baseline, so only the three spec components are ungraded.
FULL = {'schema': 1, 'kind': 'board-score', 'blocking': 0,
        'blocking_by': {'unrouted': 0, 'broken': 0, 'drc': 0, 'undersized': 0,
                        'floorplan': 0, 'assembly': 0, 'impedance': None,
                        'length': None, 'net_widths': None},
        'ungraded': ['impedance', 'length', 'net_widths'],
        'quality': {'vias': 150, 'copper_mm': 990.0, 'segments': 1700}}


def quick_payload(board='fast.kicad_pcb'):
    """quick_score.quick_score() over a canned check_connected result: two
    broken joins on a signal net, three on the poured GND."""
    real_conn, real_q = board_score.score_connectivity, board_score.quality
    board_score.score_connectivity = lambda root, b: {
        'ran': True, 'count': 5, 'unrouted': 0, 'broken': 5,
        'unrouted_net_names': [], 'poured_nets': ['GND'],
        'broken_detail': {'/SIG': {'joins_needed': 2}, 'GND': {'joins_needed': 3}}}
    board_score.quality = lambda b, pcb=None: {'vias': 143, 'copper_mm': 962.6,
                                               'segments': 1683}
    try:
        return quick_score.quick_score(board)
    finally:
        board_score.score_connectivity, board_score.quality = real_conn, real_q


def test_payload_declares_what_it_leaves_out():
    sc = quick_payload()
    assert sc['blocking'] == 2, sc
    want = {'drc', 'undersized', 'floorplan', 'assembly', 'assembly.courtyard_gating',
            'impedance', 'length', 'net_widths', 'connectivity.poured_nets'}
    assert set(sc['ungraded']) == want, sorted(sc['ungraded'])
    got = converge.ungraded_set(sc)
    assert got == want, f'converge reads the quick row as grading {sorted(got)} away'
    for k in want - {'assembly.courtyard_gating'}:
        comp = sc['components'][k]
        assert comp['ran'] is False and comp['reason'], (k, comp)
    print('  PASS: the quick payload lists its 9 ungraded components, each with a reason')


def test_quick_then_full_is_not_a_false_improvement():
    cm = converge.commensurability(quick_payload(), FULL)
    assert cm is not None, 'the two rows graded different sets; converge must say so'
    differ, hint, false_improvement = cm
    assert not false_improvement, ('the full row graded MORE components than '
                                   f'the quick row, so 2 -> 0 is no false gain: {hint}')
    assert 'drc' in differ and 'impedance' not in differ, differ
    print('  PASS: quick -> board_score is incommensurable, not a false improvement')


def test_full_then_quick_still_refuses():
    worse = dict(FULL, blocking=3, blocking_by=dict(FULL['blocking_by'], broken=3))
    cm = converge.commensurability(worse, quick_payload())
    assert cm and cm[2], ('a quick row (blocking 2) after a full row (blocking 3) '
                          'graded strictly fewer components: that IS the false '
                          'improvement converge exists to refuse')
    print('  PASS: board_score -> lower quick row is still a false improvement')


def test_record_end_to_end():
    evidence(BOARD, 'record fixture board')
    with tempfile.TemporaryDirectory() as td:
        qp = os.path.join(td, 'quick.score.json')
        fp = os.path.join(td, 'board.score.json')
        with open(qp, 'w', encoding='utf-8') as fh:
            json.dump(quick_payload(), fh)
        with open(fp, 'w', encoding='utf-8') as fh:
            json.dump(FULL, fh)
        cv = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_placer', 'converge.py'),
              'record', '--board', BOARD, '--kind', 'completion']
        led = os.path.join(td, 'ledger.jsonl')
        check(cv + ['--ledger', led, '--lever', 'fast route', '--score-file', qp], accept=True)
        r = check(cv + ['--ledger', led, '--lever', 'full route', '--score-file', fp],
                  accept=True)
        assert 'graded FEWER' not in r.stdout + r.stderr, r.stderr
        with open(led, encoding='utf-8') as fh:
            rows = [x for x in fh if x.strip()]
        assert len(rows) == 2, f'the full row was not written: {len(rows)} row(s)'
        # the reverse order is the real false improvement, refused for that reason
        led2 = os.path.join(td, 'ledger2.jsonl')
        worse = os.path.join(td, 'worse.score.json')
        with open(worse, 'w', encoding='utf-8') as fh:
            json.dump(dict(FULL, blocking=3), fh)
        check(cv + ['--ledger', led2, '--lever', 'full route', '--score-file', worse],
              accept=True)
        check(cv + ['--ledger', led2, '--lever', 'fast route', '--score-file', qp],
              refuse='graded FEWER components', code=2)
    print('  PASS: record keeps quick -> full (2 rows) and refuses full -> lower quick')


def main():
    test_payload_declares_what_it_leaves_out()
    test_quick_then_full_is_not_a_false_improvement()
    test_full_then_quick_still_refuses()
    test_record_end_to_end()
    print('test_1323_quick_score_ungraded: all passed')


if __name__ == '__main__':
    main()
