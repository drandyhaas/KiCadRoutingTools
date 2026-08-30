#!/usr/bin/env python3
"""The #703 harvester, against runs this file MANUFACTURES.

Every fixture here is synthesized in a temp directory. That is not a
convenience: the real recorded runs live under the gitignored `wk/` tree, so a
test that read them would pass on one machine and `skipTest` into a green
nothing everywhere else -- the exact hole `test_718_static_test_hygiene.py`'s
`WK_DEPENDENT` map exists to expose. Synthesizing the shapes means this file
carries its own evidence and needs no registration there.

The shapes it manufactures are the ones the REAL corpus taught, each named
after the run that taught it:

  * the run23/tigard shape -- `RUN_STATE.json` names a terminal board that is
    NOT the one `score.json` grades, because the run took one more lap after
    `routed.kicad_pcb` was written. A harvester that picks by filename reads a
    non-terminal board and reports the wrong truth.
  * the run14/castor shape -- two score files, ONE board_sha, blocking 0 and
    blocking 62, because the same board was graded at two different floors.
    There is no right answer to pick, so the run must be refused.
  * the run17/18/19 shape -- no `RUN_STATE.json` at all (it postdates them), so
    the terminal board is `routed.kicad_pcb` by filename convention. That is a
    WEAKER authority and the row has to say so.
  * the pre-run-23 checklist -- ten of twelve recorded handoffs carry no
    courtyard keys whatsoever. Coercing absent to 0 would record "zero
    courtyard overlap" as a measurement on every one of them.

    python3 -X utf8 tests/test_703_predictor_harvest.py
"""
import json
import os
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 300
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

import harvest_predictor_rows as H          # noqa: E402

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


# ---------------------------------------------------------------------------
# fixture builders
# ---------------------------------------------------------------------------

LEGACY_CHECKLIST = {
    # The six keys every recorded handoff carries. NO courtyard keys: they
    # postdate ten of the twelve documents on disk.
    'a_off_outline': {'courtyard': [['U1', 0.1]], 'pad_copper': []},
    'b_body_overlap_pairs': [],
    'b_pad_clearance_pairs': [['A', 'B']],
    'c_hole_conflicts': [],
    'c_locked_refs': ['U1'],
    'd_moved': {'moved': 0, 'expected': None, 'match': None},
}

METRICS = {k: (100.0 if k == 'hpwl' else 7) for k in H.METRIC_KEYS}


def board(path, text='(kicad_pcb fixture)\n'):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return H.sha256_file(path)


def write(path, obj):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f)


def make_run(tmp, name, *, pre_name='frozen.kicad_pcb', checklist=None,
             metrics=None, scores=(), run_state=None, routed=None,
             ledger_chain=None, ledger_rows=None, pre_text=None):
    """One synthesized run directory. Returns its path.

    `scores` is a list of (filename, board_text_or_None, blocking, label). A
    None board_text means the score grades the `routed` board.
    """
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    pre_sha = board(os.path.join(d, pre_name),
                    pre_text or f'(kicad_pcb pre {name})\n')
    write(os.path.join(d, 'handoff.json'), {
        'metrics': dict(metrics or METRICS),
        'checklist': json.loads(json.dumps(checklist
                                           if checklist is not None
                                           else LEGACY_CHECKLIST)),
        'instrument': {'board': os.path.join('/elsewhere', pre_name),
                       'clearance': 0.15},
    })
    routed_sha = None
    if routed is not None:
        routed_sha = board(os.path.join(d, 'routed.kicad_pcb'), routed)
    for fname, text, blocking, label in scores:
        sha = routed_sha if text is None else H.sha256_file(
            os.path.join(d, board(os.path.join(d, '_tmp.kicad_pcb'), text)
                         and '_tmp.kicad_pcb'))
        write(os.path.join(d, fname), {
            'schema': 1, 'kind': 'board-score', 'board': 'x.kicad_pcb',
            'board_sha': sha, 'label': label, 'blocking': blocking,
            'blocking_by': {'unrouted': 0, 'broken': blocking or 0},
            'quality': {'vias': 12, 'copper_mm': 34.5, 'segments': 56},
            'ungraded': ['impedance'],
        })
    if os.path.isfile(os.path.join(d, '_tmp.kicad_pcb')):
        os.remove(os.path.join(d, '_tmp.kicad_pcb'))
    if run_state is not None:
        write(os.path.join(d, 'RUN_STATE.json'), run_state)
    rows = list(ledger_rows or ())
    if ledger_chain:
        prev = pre_sha
        for n, sha in enumerate(ledger_chain):
            row = {'parent_sha': prev, 'result_sha': sha}
            # The LAST row of a chain is the run's final, accepted board. The
            # ledger is the terminal authority, so a fixture without a
            # `final: true` row is a run that never closed -- which is its own
            # refusal, not a shortcut for the shapes under test.
            if n == len(ledger_chain) - 1:
                row['final'] = True
                row['accepted'] = True
            rows.append(row)
            prev = sha
    if rows:
        with open(os.path.join(d, 'ledger.jsonl'), 'w', encoding='utf-8') as f:
            for r in rows:
                f.write(json.dumps(r) + '\n')
    return d, pre_sha, routed_sha


def sha_of(text):
    """The sha a board holding `text` would have, without keeping the file."""
    tmp = tempfile.mkdtemp()
    try:
        return board(os.path.join(tmp, 'b.kicad_pcb'), text)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# cases
# ---------------------------------------------------------------------------

def t_ledger_final_beats_run_state_and_the_filename(tmp):
    """The run23/tigard shape, which cost the first version of this file.

    `RUN_STATE.board_sha` is set from the LAST ledger row's result_sha whether
    or not that lap was ACCEPTED. On run23 the last lap was rejected -- its own
    lever text ends "Step back to routed.kicad_pcb" -- so RUN_STATE names a
    board the run explicitly walked away from, while its sibling `quality`
    block still reports the ACCEPTED board's vias. Reading `board_sha` there
    picks the output of a rejected lap.

    The ledger has no such ambiguity, so the ledger is the authority and
    RUN_STATE is a disclosed cross-check.
    """
    good = '(kicad_pcb accepted final)' + chr(10)
    rejected = '(kicad_pcb rejected lap)' + chr(10)
    d, pre, _r = make_run(
        tmp, 'tigard_like',
        scores=[('score.json', good, 3, ''),
                ('score_r18.json', rejected, 3, 'the rejected lap')],
        routed=good,
        run_state={'final_recorded': True, 'board_sha': sha_of(rejected)},
        ledger_rows=None,
        ledger_chain=None)
    # a ledger whose LAST row is the rejected lap, and whose last FINAL row is
    # the accepted one -- exactly run23's shape.
    mid = sha_of('(kicad_pcb mid)' + chr(10))
    with open(os.path.join(d, 'ledger.jsonl'), 'w', encoding='utf-8') as f:
        for row in [{'parent_sha': pre, 'result_sha': mid},
                    {'parent_sha': mid, 'result_sha': sha_of(good),
                     'final': True, 'accepted': True},
                    {'parent_sha': sha_of(good), 'result_sha': sha_of(rejected),
                     'accepted': False}]:
            f.write(json.dumps(row) + chr(10))
    row = H.build_row(d, tmp)
    check(row['provenance']['terminal_score_file'] == 'score.json',
          f'the ACCEPTED final board wins, not the last row RUN_STATE names '
          f'(got {row["provenance"]["terminal_score_file"]})')
    check(row['provenance']['terminal_rule'] == 'ledger_final',
          f'and the rule is named on the row '
          f'(got {row["provenance"]["terminal_rule"]})')
    check(any('RUN_STATE.board_sha' in n for n in row['notes']),
          f'the RUN_STATE disagreement is DISCLOSED, not silently won: '
          f'{row["notes"]}')
    check(row['truth']['headline'] == 3, 'the truth comes from that score')


def t_a_final_row_that_was_not_accepted_is_disclosed(tmp):
    good = '(kicad_pcb g)' + chr(10)
    d, pre, _r = make_run(tmp, 'unaccepted_final',
                          scores=[('score.json', good, 5, '')], routed=good)
    with open(os.path.join(d, 'ledger.jsonl'), 'w', encoding='utf-8') as f:
        f.write(json.dumps({'parent_sha': pre, 'result_sha': sha_of(good),
                            'final': True, 'accepted': False}) + chr(10))
    row = H.build_row(d, tmp)
    check(any('accepted=False' in n for n in row['notes']),
          f'a final row the run did not accept is NAMED on the row: '
          f'{row["notes"]}')


def t_no_ledger_refuses(tmp):
    t = '(kicad_pcb t)' + chr(10)
    d, _p, _r = make_run(tmp, 'no_ledger', scores=[('score.json', t, 1, '')],
                         routed=t,
                         run_state={'final_recorded': True, 'board_sha': sha_of(t)})
    try:
        H.build_row(d, tmp)
        check(False, 'a run with no ledger has no terminal authority, even '
                     'with a RUN_STATE that looks decisive')
    except H.Refusal as e:
        check(e.code == 'no_ledger', f'refused as no_ledger (got {e.code})')


def t_run_never_closed_refuses(tmp):
    t = '(kicad_pcb t)' + chr(10)
    d, pre, _r = make_run(tmp, 'never_closed',
                          scores=[('score.json', t, 4, '')], routed=t)
    with open(os.path.join(d, 'ledger.jsonl'), 'w', encoding='utf-8') as f:
        f.write(json.dumps({'parent_sha': pre,
                            'result_sha': sha_of(t)}) + chr(10))
    try:
        H.build_row(d, tmp)
        check(False, 'a ledger with no final row means the run never closed')
    except H.Refusal as e:
        check(e.code == 'run_never_closed',
              f'refused as run_never_closed (got {e.code})')


def t_ambiguous_terminal_score_refuses(tmp):
    """The run14/castor shape: one board, two verdicts, 0 vs 62."""
    same = '(kicad_pcb final)\n'
    d, _p, _r = make_run(
        tmp, 'castor_like',
        scores=[('score.json', same, 0, ''),
                ('score_declared.json', same, 62, 'declared-floor grade')],
        routed=same,
        ledger_chain=[sha_of(same)])
    try:
        H.build_row(d, tmp)
        check(False, 'two scores of ONE board with different blocking must '
                     'refuse, not pick')
    except H.Refusal as e:
        check(e.code == 'ambiguous_terminal_score',
              f'refused as ambiguous_terminal_score (got {e.code})')
        check('blocking=0' in e.detail and 'blocking=62' in e.detail,
              f'and the refusal QUOTES both verdicts: {e.detail}')


def t_vacuous_blocking_refuses(tmp):
    d, _p, _r = make_run(
        tmp, 'vacuous', scores=[('score.json', None, None, '')],
        routed='(kicad_pcb v)\n',
        ledger_chain=[sha_of('(kicad_pcb v)\n')])
    try:
        H.build_row(d, tmp)
        check(False, 'blocking=None must refuse, never become 0')
    except H.Refusal as e:
        check(e.code == 'vacuous_blocking',
              f'refused as vacuous_blocking (got {e.code})')


def t_no_score_for_terminal_sha_refuses(tmp):
    d, _p, _r = make_run(
        tmp, 'orphan_score',
        scores=[('score.json', '(kicad_pcb other)' + chr(10), 5, '')],
        routed='(kicad_pcb routed)' + chr(10),
        ledger_chain=[sha_of('(kicad_pcb routed)' + chr(10))])
    try:
        H.build_row(d, tmp)
        check(False, 'a score grading a different board must refuse')
    except H.Refusal as e:
        check(e.code == 'no_score_for_terminal_board',
              f'refused as no_score_for_terminal_board (got {e.code})')
        check('score.json' in e.detail,
              'and the refusal names every score file it scanned')


def t_no_authority_refuses(tmp):
    d, _p, _r = make_run(tmp, 'no_authority',
                         scores=[('score.json', '(kicad_pcb a)\n', 1, '')])
    try:
        H.build_row(d, tmp)
        check(False, 'no RUN_STATE and no routed.kicad_pcb must refuse')
    except H.Refusal as e:
        check(e.code == 'no_terminal_authority',
              f'refused as no_terminal_authority (got {e.code})')


def t_run_state_not_final_refuses(tmp):
    t = '(kicad_pcb t)\n'
    d, _p, _r = make_run(
        tmp, 'unfinished', scores=[('score.json', t, 4, '')], routed=t,
        run_state={'final_recorded': False, 'board_sha': sha_of(t)},
        ledger_chain=[sha_of(t)])
    try:
        H.build_row(d, tmp)
        check(False, 'a run that did not close out has no terminal score')
    except H.Refusal as e:
        check(e.code == 'run_state_not_final',
              f'refused as run_state_not_final (got {e.code})')


def t_lineage_needs_a_bfs_not_a_walk(tmp):
    """A greedy 'follow the last row' walk returns to the start; a BFS does not.

    This is the run23 shape: the ledger branches, and the terminal board is
    reachable only by exploring rather than by following one edge per node.
    """
    pre_text = '(kicad_pcb pre bfs)\n'
    pre = sha_of(pre_text)
    a, b, term = sha_of('a'), sha_of('b'), sha_of('(kicad_pcb term)\n')
    rows = [
        {'parent_sha': pre, 'result_sha': a},
        {'parent_sha': a, 'result_sha': b},
        # the DEAD END a greedy walk would take, recorded last so a
        # "follow the newest row" rule picks it and terminates at `pre`
        {'parent_sha': b, 'result_sha': term, 'final': True,
         'accepted': True},
        {'parent_sha': b, 'result_sha': pre},
    ]
    d, _p, _r = make_run(
        tmp, 'bfs_lineage',
        scores=[('score.json', '(kicad_pcb term)\n', 2, '')],
        routed='(kicad_pcb term)\n', ledger_rows=rows, pre_text=pre_text)
    row = H.build_row(d, tmp)
    check(row['provenance']['lineage'] == 'ledger_reachable',
          'a branching ledger still proves lineage')
    check(row['provenance']['lineage_hops'] == 3,
          f'over 3 hops (got {row["provenance"]["lineage_hops"]})')
    # And the search must TERMINATE on a cycle rather than spin.
    edges = {pre: {a}, a: {b}, b: {pre}}
    found, hops, seen = H.reachable(edges, pre, sha_of('nowhere'))
    check(found is False and seen == 3,
          f'a cyclic ledger terminates and reports unreachable '
          f'(found={found}, seen={seen})')


def t_lineage_unproven_refuses(tmp):
    d, _p, _r = make_run(
        tmp, 'broken_lineage',
        scores=[('score.json', '(kicad_pcb t)\n', 1, '')],
        routed='(kicad_pcb t)\n',
        ledger_rows=[{'parent_sha': sha_of('x'),
                      'result_sha': sha_of('(kicad_pcb t)' + chr(10)),
                      'final': True, 'accepted': True}])
    try:
        H.build_row(d, tmp)
        check(False, 'an unreachable terminal board must refuse')
    except H.Refusal as e:
        check(e.code == 'lineage_unproven',
              f'refused as lineage_unproven (got {e.code})')


def t_pre_board_gone_refuses(tmp):
    t = '(kicad_pcb t)\n'
    d, _p, _r = make_run(tmp, 'no_pre', scores=[('score.json', t, 1, '')],
                         routed=t, ledger_chain=[sha_of(t)])
    os.remove(os.path.join(d, 'frozen.kicad_pcb'))
    try:
        H.build_row(d, tmp)
        check(False, 'a missing pre-route board must refuse')
    except H.Refusal as e:
        check(e.code == 'pre_board_gone',
              f'refused as pre_board_gone (got {e.code})')


def t_absent_checklist_key_is_null_never_zero(tmp):
    """Ten of twelve recorded handoffs carry no courtyard keys at all."""
    t = '(kicad_pcb t)\n'
    d, _p, _r = make_run(tmp, 'legacy_checklist',
                         scores=[('score.json', t, 1, '')], routed=t,
                         ledger_chain=[sha_of(t)])
    row = H.build_row(d, tmp)
    p = row['predictors']
    check(p['courtyard_blocking_pairs'] is None,
          f'an absent key is null, NOT 0 (got '
          f'{p["courtyard_blocking_pairs"]!r})')
    check(p['cross_side_stacks'] is None, 'and so is every other absent one')
    check('checklist.b_courtyard_blocking_pairs' in row['schema_gaps'],
          f'and the absence is RECORDED: {row["schema_gaps"]}')
    check(p['pad_copper'] == 0,
          'a present-and-empty list is a real measurement of 0')
    check(p['courtyard_off_outline'] == 1,
          'a present list is counted, not truncated')
    check(p['pad_clearance_pairs'] == 1, 'counts come from len()')


def t_legacy_alias_is_normalised_and_recorded(tmp):
    t = '(kicad_pcb t)\n'
    cl = dict(LEGACY_CHECKLIST)
    cl['b_courtyard_overlap_pairs'] = [['A', 'B', 1.0], ['C', 'D', 2.0]]
    d, _p, _r = make_run(tmp, 'aliased', checklist=cl,
                         scores=[('score.json', t, 1, '')], routed=t,
                         ledger_chain=[sha_of(t)])
    row = H.build_row(d, tmp)
    check(row['predictors']['courtyard_advisory_pairs'] == 2,
          'the run-23 rename is read under its old name')
    check(row['schema_aliases_used'].get('courtyard_advisory_pairs')
          == 'b_courtyard_overlap_pairs',
          f'and the alias is RECORDED: {row["schema_aliases_used"]}')
    check('checklist.b_courtyard_advisory_pairs' not in row['schema_gaps'],
          'an aliased key is not also reported as a gap')


def t_unknown_metric_is_reported_not_dropped(tmp):
    t = '(kicad_pcb t)\n'
    m = dict(METRICS)
    m['a_brand_new_metric'] = 99
    d, _p, _r = make_run(tmp, 'new_metric', metrics=m,
                         scores=[('score.json', t, 1, '')], routed=t,
                         ledger_chain=[sha_of(t)])
    row = H.build_row(d, tmp)
    check(any('a_brand_new_metric' in n for n in row['notes']),
          f'a metric this tool does not record is NAMED: {row["notes"]}')
    check('a_brand_new_metric' not in row['predictors'],
          'but not silently folded into the predictor set')


def t_missing_metric_is_a_gap_not_a_zero(tmp):
    t = '(kicad_pcb t)\n'
    m = {k: v for k, v in METRICS.items() if k != 'crossings'}
    d, _p, _r = make_run(tmp, 'lost_metric', metrics=m,
                         scores=[('score.json', t, 1, '')], routed=t,
                         ledger_chain=[sha_of(t)])
    row = H.build_row(d, tmp)
    check(row['predictors']['crossings'] is None,
          'a dropped metric is null, not 0')
    check('metrics.crossings' in row['schema_gaps'],
          'and it is recorded as a gap')


def t_discovery_finds_both_depths(tmp):
    """`wk/run20/handoff.json` is ONE level down; the old glob wanted two."""
    t = '(kicad_pcb t)\n'
    deep = os.path.join(tmp, 'deep')
    os.makedirs(deep, exist_ok=True)
    make_run(tmp, 'flat', scores=[('score.json', t, 1, '')], routed=t,
             ledger_chain=[sha_of(t)])
    make_run(deep, 'nested', scores=[('score.json', t, 1, '')], routed=t,
             ledger_chain=[sha_of(t)])
    found = {os.path.relpath(d, tmp).replace('\\', '/')
             for d in H.discover_runs(tmp)}
    check('flat' in found and 'deep/nested' in found,
          f'discovery reaches depth 1 AND depth 2: {sorted(found)}')


def t_row_covers_every_declared_predictor_key(tmp):
    t = '(kicad_pcb t)\n'
    d, _p, _r = make_run(tmp, 'keys', scores=[('score.json', t, 1, '')],
                         routed=t, ledger_chain=[sha_of(t)])
    row = H.build_row(d, tmp)
    missing = set(H.PREDICTOR_KEYS) - set(row['predictors'])
    extra = set(row['predictors']) - set(H.PREDICTOR_KEYS)
    check(not missing and not extra,
          f'a row carries EXACTLY the declared predictor keys '
          f'(missing {sorted(missing)}, extra {sorted(extra)})')
    check(row['reproducible'] is False,
          'a harvest row is never reproducible -- it reads the gitignored tree')
    check(row['predictor_source'] == 'handoff.json',
          'and it says its predictors were READ, not re-derived')
    check(row['source'] == 'harvest', 'source names the producer')


def t_harvest_reports_refusals_alongside_rows(tmp):
    t = '(kicad_pcb t)\n'
    make_run(tmp, 'good', scores=[('score.json', t, 1, '')], routed=t,
             ledger_chain=[sha_of(t)])
    make_run(tmp, 'bad', scores=[('score.json', '(kicad_pcb z)\n', 1, '')],
             routed=t)
    rows, refusals = H.harvest(tmp, tmp)
    check(len(rows) == 1 and len(refusals) == 1,
          f'one row and one refusal ({len(rows)}, {len(refusals)})')
    check(len(rows) + len(refusals) == len(H.discover_runs(tmp)),
          'every discovered run is accounted for -- no silent drops')
    check(refusals[0]['reason_code'] and refusals[0]['detail'],
          'a refusal carries both a code and a human detail')


CASES = [
    t_ledger_final_beats_run_state_and_the_filename,
    t_a_final_row_that_was_not_accepted_is_disclosed,
    t_no_ledger_refuses,
    t_run_never_closed_refuses,
    t_ambiguous_terminal_score_refuses,
    t_vacuous_blocking_refuses,
    t_no_score_for_terminal_sha_refuses,
    t_lineage_needs_a_bfs_not_a_walk,
    t_lineage_unproven_refuses,
    t_pre_board_gone_refuses,
    t_absent_checklist_key_is_null_never_zero,
    t_legacy_alias_is_normalised_and_recorded,
    t_unknown_metric_is_reported_not_dropped,
    t_missing_metric_is_a_gap_not_a_zero,
    t_discovery_finds_both_depths,
    t_row_covers_every_declared_predictor_key,
    t_harvest_reports_refusals_alongside_rows,
]


def main():
    for fn in CASES:
        print(f'{fn.__name__}:')
        tmp = tempfile.mkdtemp(prefix='h703_')
        try:
            fn(tmp)
        except Exception as e:                                  # noqa: BLE001
            check(False, f'{fn.__name__} raised {type(e).__name__}: {e}')
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_703_predictor_harvest: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
