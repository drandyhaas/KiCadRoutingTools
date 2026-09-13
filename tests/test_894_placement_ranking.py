#!/usr/bin/env python3
"""A placement window whose laps carry the terms is judged, not plateaued (#894).

The defect: on a copper-free board every lap's `_score_key` is
`(blocking, (0, 0.0, 0))` -- identical -- so `min(r) < r[0]` is False,
`unjudged` is 0, and `_half_state` reports **plateau**. Seven laps of one board
produced one number and the half read as finished while it was still moving.

(The issue's own acceptance says L5 "no longer reports no comparison". Traced
through `_half_state`, that is not the shape: `no-comparison` comes only from
`blocked` in {unjudged, incommensurable, single-lap}, none of which a scoring
component can fix. The issue's BODY states the real defect correctly -- "the
plateau test had nothing to compare" -- and that is what is asserted here.)

The fix is a tier that is reachable ONLY after `blocking` and `quality` have
already tied, so it can turn `plateau` into `improving` and nothing else. That
containment is the load-bearing claim of this file, and it is tested in both
directions: a routing window and an improving window must be untouched.

Comparison is PARETO. A lap that trades pair length for balance is NOT an
improvement -- it reports `plateau` with `placement_traded` naming both sides.
There is no weight anywhere: #694's corridor term reversed its measured sign
while an aggregate verdict kept printing PASS.

Run: python3 -X utf8 tests/test_894_placement_ranking.py
"""
import os
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 300

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
                os.path.join(ROOT, 'py_tools')]

import converge as C                                         # noqa: E402
import placement_score as ps                                 # noqa: E402

passed = 0
failed = 0


def check(name, ok, detail=''):
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}' + (f' -- {detail}' if detail else ''))
    else:
        failed += 1
        print(f'  FAIL {name}' + (f' -- {detail}' if detail else ''))


def _score(blocking=0, pair=None, balance=None, segments=0, placement=True):
    """A board_score payload shaped like the real one."""
    doc = {'blocking': blocking,
           'blocking_by': {'unrouted': blocking},
           'ungraded': [],
           'quality': {'vias': 0, 'copper_mm': 0.0, 'segments': segments}}
    if placement:
        terms = {}
        if pair is not None:
            terms['pair_length'] = {'ran': True, 'value': pair, 'unit': 'mm',
                                    'basis': None,
                                    'direction': 'lower-is-better'}
        if balance is not None:
            terms['balance'] = {'ran': True, 'value': balance,
                                'unit': 'fraction of span', 'basis': None,
                                'direction': 'lower-is-better'}
        doc['placement'] = {'term_order': list(ps.TERM_ORDER), 'terms': terms}
    return doc


def _rows(scores, kind='placement', accepted=True):
    out = []
    for i, s in enumerate(scores):
        out.append({'iteration': i, 'kind': kind, 'accepted': accepted,
                    'result_sha': f'sha{i}',
                    'parent_sha': (f'sha{i - 1}' if i else None),
                    'lever': f'lap {i}', 'score': s})
    return out


def test_a_tied_window_that_improves_on_the_terms_is_not_a_plateau():
    tied = [_score(pair=8.5 - 0.1 * i, balance=0.09 - 0.001 * i)
            for i in range(5)]
    st = C._half_state(_rows(tied), 'placement', 5)
    check('the window is judged `improving`, not `plateau`',
          st['why'] == 'improving' and st['flat'] is False, str(st['why']))
    check('...and it names the term that moved',
          'pair_length' in (st.get('placement_improved') or ''),
          str(st.get('placement_improved'))[:90])
    # The control: without the terms, the very same laps plateau. This is what
    # the placement half saw before, and it is why the fix exists.
    bare = [_score(placement=False) for _ in range(5)]
    st0 = C._half_state(_rows(bare), 'placement', 5)
    check('the same window WITHOUT the terms still plateaus',
          st0['why'] == 'plateau', str(st0['why']))


def test_terms_that_TRADE_are_not_an_improvement():
    traded = [_score(pair=8.5, balance=0.05), _score(pair=8.5, balance=0.05),
              _score(pair=8.0, balance=0.09)]
    st = C._half_state(_rows(traded), 'placement', 3)
    check('a traded window is a plateau, not an improvement',
          st['why'] == 'plateau', str(st['why']))
    check('...and both sides are named',
          'pair_length' in (st.get('placement_traded') or '')
          and 'balance' in (st.get('placement_traded') or ''),
          str(st.get('placement_traded'))[:120])


def test_the_tier_cannot_reach_anything_but_a_tied_placement_window():
    # 1. The ROUTING half is untouched. These laps tie on `blocking` while
    #    their placement terms improve monotonically -- exactly the input that
    #    flips the placement half -- and routing must still call it a plateau.
    rows = _rows([_score(blocking=10, pair=8.5 - 0.1 * i) for i in range(5)],
                 kind='completion')
    st = C._half_state(rows, 'routing', 5)
    check('a routing window tied on blocking plateaus DESPITE improving terms',
          st['why'] == 'plateau' and 'placement_improved' not in st,
          f"{st['why']} {sorted(st)}")
    #    ...and the identical laps, read as the placement half, do flip. Same
    #    scores, different half: the guard is the only thing between them.
    st_p = C._half_state(_rows([_score(blocking=10, pair=8.5 - 0.1 * i)
                                for i in range(5)]), 'placement', 5)
    check('...while the SAME laps flip as the placement half',
          st_p['why'] == 'improving', str(st_p['why']))
    # 2. Routing judged on blocking still improves, untouched by any of this.
    st = C._half_state(_rows([_score(blocking=b, pair=9.0)
                              for b in (5, 4, 3, 2, 1)], kind='completion'),
                       'routing', 5)
    check('a routing window that really improves still does',
          st['why'] == 'improving', str(st['why']))
    # 2. A window ALREADY improving on blocking is unchanged.
    improving = _rows([_score(blocking=b, pair=9.0) for b in (5, 4, 3, 2, 1)])
    st = C._half_state(improving, 'placement', 5)
    check('an improving window stays improving', st['why'] == 'improving'
          and 'placement_improved' not in st, str(st['why']))
    # 3. A no-comparison window is NOT rescued into a verdict.
    unjudged = _rows([_score(pair=9.0 - i) for i in range(5)])
    for r in unjudged:
        r['score'].pop('blocking')
    st = C._half_state(unjudged, 'placement', 5)
    check('an unjudged window still says no-comparison',
          st['why'] == 'no-comparison' and st.get('blocked') == 'unjudged',
          f"{st['why']}/{st.get('blocked')}")
    # 4. THE MIXED WINDOW -- some laps compare, one accepted lap carries no
    #    `blocking`. This is the shape the first version of the tier stole
    #    from `no-comparison`, dropping the very keys that name which rows
    #    could not be judged. The control above cannot reach it: popping
    #    `blocking` from ALL rows leaves nothing to compare, so the tier is
    #    never consulted.
    mixed = _rows([_score(pair=8.5 - 0.1 * i) for i in range(5)])
    mixed[4]['score'].pop('blocking')
    st = C._half_state(mixed, 'placement', 5)
    check('a window with ONE unjudged lap stays no-comparison',
          st['why'] == 'no-comparison', str(st['why']))
    check('...and still names which rows it could not judge',
          st.get('blocked') == 'unjudged' and st.get('unjudged') == 1
          and st.get('unjudged_iterations') == [4],
          f"blocked={st.get('blocked')} unjudged={st.get('unjudged')} "
          f"its={st.get('unjudged_iterations')}")
    check('...and does not claim a placement improvement',
          'placement_improved' not in st, str(sorted(st)))


def test_parent_score_reads_the_sha_nothing_ever_read():
    rows = _rows([_score(pair=8.5), _score(pair=8.0)])
    check('a row resolves its parent by content hash',
          C.parent_score(rows, rows[1]) is rows[0]['score'], '')
    check('the first row has no parent',
          C.parent_score(rows, rows[0]) is None, '')
    check('an unknown sha is None, not a guess',
          C.parent_score(rows, {'parent_sha': 'nope'}) is None, '')
    # A re-recorded board makes the parent ambiguous. "I could not tell" must
    # not become an answer.
    dup = rows + [dict(rows[0], iteration=9)]
    check('two rows with one result_sha resolve to None',
          C.parent_score(dup, rows[1]) is None, '')


def test_status_prints_the_terms_and_the_accept_rule():
    import io
    import json
    import tempfile
    import contextlib
    rows = _rows([_score(pair=8.5, balance=0.05),
                  _score(pair=8.0, balance=0.05),
                  _score(pair=8.2, balance=0.05)])
    with tempfile.TemporaryDirectory(prefix='t894st_') as tmp:
        p = os.path.join(tmp, 'ledger.jsonl')
        with open(p, 'w', encoding='utf-8') as fh:
            for r in rows:
                fh.write(json.dumps(r) + '\n')
        buf, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(err):
            C.cmd_status(type('A', (), {'ledger': p})())
    doc = json.loads(buf.getvalue())
    check('stdout is still ONE json document a caller can load whole',
          isinstance(doc, dict) and 'placement_terms' in doc,
          sorted(doc)[:6])
    pt = doc['placement_terms']
    check('every lap carrying the terms is listed', len(pt) == 3, str(len(pt)))
    check('the improving lap reports no term regressed',
          pt[1]['no_term_regressed'] is True and pt[1]['vs_parent'] == 'better',
          str(pt[1].get('delta')))
    check('the regressing lap reports one did',
          pt[2]['no_term_regressed'] is False and pt[2]['vs_parent'] == 'worse',
          str(pt[2].get('delta')))
    check('...and it is itemised on stderr for a reader',
          'placement worse' in err.getvalue(), err.getvalue().strip()[:90])


def main():
    for name in sorted(k for k in globals() if k.startswith('test_')):
        print(f'--- {name}')
        globals()[name]()
    print(f'\n{passed} passed, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
