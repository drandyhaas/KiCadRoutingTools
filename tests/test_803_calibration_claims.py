#!/usr/bin/env python3
"""Every number in docs/placement-calibration.md is re-derived here (#803).

The page it guards is the recovered write-up of the P-close ratio calibration
-- the authority eleven driver sites cite for withdrawing a scoring threshold.
It was lost precisely because it lived only in a gitignored working directory,
so the repair commits its rows and this gate re-computes the page from them.

The rule this file follows, and the reason it is written the way it is: a regen
test must RE-DERIVE the aggregate, not read a constant. Every expected value
below comes out of `tests/placement_calibration_rows.json` or
`tests/placement_calibration_recovered.json` by arithmetic performed here; the
markdown is the thing under test. Change a metric in the JSON without editing
the page (or edit the page without the JSON) and this fails, naming the cell.

The one exception is stated in the page and re-stated here: run 15's own ratio
(0.064) comes from that run's render JSON, which is not in this repo. It is
quoted, not derived, and `t_the_undrivable_number_is_disclosed` pins the
disclosure rather than the number.

    python3 -X utf8 tests/test_803_calibration_claims.py
"""
import io
import json
import os
import re
import sys

RUN_ALL_TIMEOUT = 120
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOC = os.path.join(ROOT, 'docs', 'placement-calibration.md')
ROWS = os.path.join(ROOT, 'tests', 'placement_calibration_rows.json')
RECOVERED = os.path.join(ROOT, 'tests', 'placement_calibration_recovered.json')

FAILURES = []
CHECKED = []


def check(cond, what, detail=''):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}{detail}')
        FAILURES.append(what)


def load():
    rows = {r['board']: r
            for r in json.load(io.open(ROWS, encoding='utf-8'))}
    rec = {(r['board'], r['pop']): r
           for r in json.load(io.open(RECOVERED, encoding='utf-8'))}
    text = io.open(DOC, encoding='utf-8').read().replace('\r\n', '\n')
    return rows, rec, text


def cells(text, header_contains, ncols):
    """Every data row of the first markdown table whose header matches.

    Returns a list of cell-lists. Header and `|---|` separator dropped.
    """
    out = []
    seen_header = False
    for line in text.split('\n'):
        s = line.strip()
        if not s.startswith('|'):
            if seen_header and out:
                break
            continue
        parts = [c.strip() for c in s.strip('|').split('|')]
        if not seen_header:
            if all(h in ' '.join(parts) for h in header_contains):
                seen_header = True
            continue
        if set(''.join(parts)) <= set('-: '):
            continue
        if len(parts) == ncols:
            out.append(parts)
        else:
            break
    return out


def num(s):
    """A numeric markdown cell: strips bold, unicode minus, thousands."""
    s = s.replace('**', '').replace('−', '-').replace(',', '').strip()
    if s in ('', '--', '—'):
        return None
    m = re.match(r'^[+-]?\d+(?:\.\d+)?$', s)
    return float(s) if m else None


def close(got, want, places):
    return got is not None and abs(got - round(want, places)) < 10 ** -places / 2


# ------------------------------------------------------------- the checks

def t_metric_table_matches_the_rows(rows, text):
    """board | population | halo | hpwl | crossings | blocking, at 3 dp."""
    table = cells(text, ('board', 'population', 'halo', 'hpwl', 'crossings',
                         'blocking'), 6)
    bad = []
    for board, pop, halo, hpwl, cross, blk in table:
        r = rows.get(board)
        if r is None or r.get(pop) is None:
            bad.append(f'{board}/{pop}: no such row in the JSON')
            continue
        m = r[pop]
        for label, got, want, places in (
                ('halo', num(halo), m['halo'], 3),
                ('hpwl', num(hpwl), m['hpwl'], 3),
                ('crossings', num(cross), m['crossings'], 0)):
            if not close(got, want, places):
                bad.append(f'{board}/{pop} {label}: page {got}, '
                           f'rows {round(want, places)}')
        want_blk = r.get(pop + '_blocking')
        if num(blk) != want_blk:
            bad.append(f'{board}/{pop} blocking: page {num(blk)}, '
                       f'rows {want_blk}')
        CHECKED.append(f'{board}/{pop}')
    check(len(table) >= 8, 'the measurements table still has >= 8 rows',
          f'\n       found {len(table)}')
    check(not bad, 'every measurements cell is re-derived from the rows',
          ('' if not bad else '\n       ' + '\n       '.join(bad)))


def t_gain_table_matches_rows_and_recovered(rows, rec, text):
    """board | population | halo_gain | hpwl_gain | ratio | verdict.

    The complete arms come from `g_repair`/`g_legality`; the *(partial)* arms
    from recovered.json, which is exactly the split the page discloses.
    """
    table = cells(text, ('halo_gain', 'hpwl_gain', 'ratio', 'verdict'), 6)
    bad = []
    checked = 0
    for board, pop, hg, wg, ratio, verdict in table:
        low = pop.lower()
        if 'undamaged' in low:
            src = {'halo': 0.0, 'hpwl': 0.0}
        elif 'legality' in low:
            src = (rows[board].get('g_legality')
                   or _from_rec(rec, board, 'legality'))
        else:
            src = (rows[board].get('g_repair')
                   or _from_rec(rec, board, 'repair'))
        if src is None:
            bad.append(f'{board}/{pop}: no gain in either JSON')
            continue
        if not close(num(hg), src['halo'], 4):
            bad.append(f'{board}/{pop} halo_gain: page {num(hg)}, '
                       f'json {round(src["halo"], 4)}')
        if not close(num(wg), src['hpwl'], 4):
            bad.append(f'{board}/{pop} hpwl_gain: page {num(wg)}, '
                       f'json {round(src["hpwl"], 4)}')
        want_ratio = (src['hpwl'] / src['halo']) if src['halo'] else None
        got_ratio = num(ratio)
        if want_ratio is None:
            if got_ratio is not None:
                bad.append(f'{board}/{pop} ratio: page {got_ratio}, '
                           'json has no denominator (halo_gain 0)')
        elif not close(got_ratio, want_ratio, 4 if abs(want_ratio) < 1 else 1):
            bad.append(f'{board}/{pop} ratio: page {got_ratio}, '
                       f'json {want_ratio:.4f}')
        # The verdict column, re-derived rather than discarded. It used to be
        # unpacked as `_verdict` and dropped, so a REFUSE could be flipped to
        # pass (or back) with the gate still printing OK -- and it is the
        # column the whole "0.25 refuses a repair that worked" argument rests
        # on. The rule is the one the page now states in full: the ratio test
        # only applies when halo_gain >= 0.25, else the arm early-outs.
        want_verdict = ('REFUSE'
                        if src['halo'] >= 0.25 and src['hpwl'] < 0.25 * src['halo']
                        else 'pass')
        got_verdict = 'REFUSE' if 'REFUSE' in verdict.upper() else 'pass'
        if got_verdict != want_verdict:
            bad.append(f'{board}/{pop} verdict @ 0.25: page {verdict.strip()!r}, '
                       f'the rule gives {want_verdict} '
                       f'(halo_gain {src["halo"]:.4f}, hpwl_gain {src["hpwl"]:.4f})')
        checked += 1
    check(checked >= 7, 'the gains table still has >= 7 rows',
          f'\n       found {checked}')
    check(not bad, 'every gain and ratio is re-derived from the JSONs',
          ('' if not bad else '\n       ' + '\n       '.join(bad)))


def _from_rec(rec, board, pop):
    r = rec.get((board, pop))
    return None if r is None else {'halo': r['halo_gain'],
                                   'hpwl': r['hpwl_gain']}


def t_premise_table_matches_the_rows(rows, text):
    """board | hpwl pristine | hpwl damaged | delta | premise, at 1 dp."""
    table = cells(text, ('hpwl pristine', 'hpwl damaged', 'delta', 'premise'),
                  5)
    bad = []
    for board, pri, dmg, delta, premise in table:
        board = board.replace('**', '').strip()
        r = rows.get(board)
        if r is None:
            bad.append(f'{board}: no such row')
            continue
        want_p, want_d = r['pristine']['hpwl'], r['damaged']['hpwl']
        want_delta = want_d - want_p
        if not close(num(pri), want_p, 1):
            bad.append(f'{board} pristine: page {num(pri)}, '
                       f'rows {round(want_p, 1)}')
        if not close(num(dmg), want_d, 1):
            bad.append(f'{board} damaged: page {num(dmg)}, '
                       f'rows {round(want_d, 1)}')
        if not close(num(delta), want_delta, 1):
            bad.append(f'{board} delta: page {num(delta)}, '
                       f'rows {round(want_delta, 1)}')
        want_word = 'inverted' if want_delta < 0 else 'holds'
        if want_word not in premise.replace('**', ''):
            bad.append(f'{board} premise: page says {premise!r}, '
                       f'the delta says {want_word!r}')
    check(len(table) == 3, 'the premise table still covers all three boards',
          f'\n       found {len(table)}')
    check(not bad, 'every premise cell is re-derived from the rows',
          ('' if not bad else '\n       ' + '\n       '.join(bad)))


def t_the_piantor_perfect_repair_figures(rows, text):
    """The load-bearing claim: a PERFECT repair scores negative and is refused.

    Perfect means `repaired == pristine`, so the gains are computable from the
    two rows the JSON does carry -- the reason this claim survives even though
    piantor's repair arm timed out.
    """
    p = rows['piantor']
    pr, dm = p['pristine'], p['damaged']
    hg = (dm['halo'] - pr['halo']) / dm['halo']
    wg = (dm['hpwl'] - pr['hpwl']) / dm['hpwl']
    cg = (dm['crossings'] - pr['crossings']) / dm['crossings']
    ratio = wg / hg
    bad = []
    for label, want, places in (('halo_gain', hg, 4), ('hpwl_gain', wg, 4),
                                ('ratio', ratio, 4),
                                ('crossings gain', cg, 4)):
        lit = f'{round(want, places):+.{places}f}'
        alt = lit.replace('-', '−').replace('+', '')
        if lit not in text and alt not in text and lit.lstrip('+') not in text:
            bad.append(f'{label} {lit} is not stated on the page')
    check(not bad, 'the piantor perfect-repair figures are on the page',
          ('' if not bad else '\n       ' + '\n       '.join(bad)))
    check(wg < 0 < hg,
          'a perfect piantor repair really does score a negative hpwl gain '
          'against a positive halo gain')


def t_the_separation_and_the_blocking_claims(rows, text):
    n = rows['neo6502']
    sep = ((n['g_repair']['hpwl'] / n['g_repair']['halo'])
           / (n['g_legality']['hpwl'] / n['g_legality']['halo']))
    # EVERY occurrence, not "somewhere in the file". The page states the
    # separation twice; an anywhere-substring check stayed green when one of
    # the two was changed.
    sites = text.count(f'{sep:.2f}')
    wrong = re.findall(r'separate by \*\*([0-9.]+)x?\*\*|separation is real',
                       text)
    check(sites >= 2,
          f'the neo6502 separation ({sep:.2f}x) appears at both of its sites',
          f'\n       found {sites} occurrence(s) of {sep:.2f}; the page states'
          '\n       it in reason 4 and again in "What is kept"')
    check(all(close(num(w), sep, 2) for w in wrong if w),
          'no other separation figure contradicts it',
          f'\n       found {wrong!r} against {sep:.4f}')
    check(n['damaged_blocking'] == 18 and n['repaired_blocking'] == 0
          and n['legality_only_blocking'] == 0,
          'the rows really say neo6502 went blocking 18 -> 0 on both arms')
    check('18' in text and '18 → 0' in text.replace('->', '→'),
          'the page states the 18 -> 0 result')


def t_the_timing_side_finding(rows, text):
    """The page must publish the COMMITTED run's seconds, not the lost one's.

    The recovered write-up quoted a 3000 s deadline run; the rows that ship are
    a different, shorter run. Publishing the 3000 s figures beside these rows
    would be a number with no measurement behind it.
    """
    u, p, n = rows['urchin'], rows['piantor'], rows['neo6502']
    bad = []
    # EVERY timing and exit code the page publishes, not the three that were
    # convenient. The legality seconds and the rc values were quoted and
    # unchecked, which is how "every number is re-derived" stopped being true.
    for label, lit in (('urchin repair seconds', f"{u['repair_seconds']}"),
                       ('urchin legality seconds', f"{u['legality_seconds']}"),
                       # Exit codes are ANCHORED to their phrase. A bare
                       # `str(rc) in text` is vacuous -- "7" occurs all over
                       # the page, so changing 7 to 9 passed. Measured.
                       ('urchin repair rc',
                        f"urchin exited {u['repair_rc']} after"),
                       ('legality-arm rc',
                        f"legality-only arms exited {u['legality_rc']} after"),
                       ('piantor repair timeout',
                        str(p['repair_error']).strip()),
                       ('piantor legality seconds', f"{p['legality_seconds']}"),
                       ('piantor repair rc', f"(exit {p['repair_rc']})"),
                       ('neo6502 repair seconds', f"{n['repair_seconds']}")):
        if lit not in text:
            bad.append(f'{label}: {lit!r} is not on the page')
    check(not bad, 'the side finding quotes the committed run',
          ('' if not bad else '\n       ' + '\n       '.join(bad)))
    check('3000' in text and 'not** the committed rows' in text,
          'the earlier 3000 s run is disclosed as a different run')


def t_the_undrivable_number_is_disclosed(text):
    check('0.064' in text and 'not re-derived' in text,
          "run 15's 0.064 is marked as quoted, not derived")


def t_the_partial_rows_are_disclosed(rows, text):
    """The page says the gain keys are ABSENT; check exactly that.

    This was a bare `assert not rows[board].get('g_repair')`. Two faults, both
    the repo's own documented anti-patterns. `not …get()` is satisfied
    identically by absent, None, {} and 0, so it could not tell which the page
    should say -- and the page said "null" while the data says absent. And a
    bare assert ABORTS the module: when it fired, the named check below never
    printed, the FAIL summary never printed, and the vacuity floor never ran.
    CLAUDE.md: "a non-zero exit is not evidence -- assert the REASON."
    """
    missing = []
    for board in ('urchin', 'piantor'):
        for key in ('g_repair', 'g_legality'):
            if key in rows[board]:
                missing.append(f'{board}.{key} is present ({rows[board][key]!r})'
                               ' -- the page says the key is absent')
    check(not missing,
          'the partial boards carry no gain keys, as the page states',
          ('' if not missing else '\n       ' + '\n       '.join(missing)))
    check('null' not in text.lower().split('## side findings')[0]
          or 'said "null"' in text,
          'the page does not call the absent keys "null" without correcting it')
    check('partial' in text and 'No committed' in text
          and 'writes that file' in text,
          'the page discloses that recovered.json has no producing script')


def t_the_gate_is_not_vacuous():
    check(len(CHECKED) >= 8,
          f'>= 8 metric blocks were actually compared (got {len(CHECKED)})')


TESTS = [
    't_metric_table_matches_the_rows',
    't_gain_table_matches_rows_and_recovered',
    't_premise_table_matches_the_rows',
    't_the_piantor_perfect_repair_figures',
    't_the_separation_and_the_blocking_claims',
    't_the_timing_side_finding',
    't_the_undrivable_number_is_disclosed',
    't_the_partial_rows_are_disclosed',
    't_the_gate_is_not_vacuous',
]


def main():
    print('#803: docs/placement-calibration.md re-derived from its rows')
    rows, rec, text = load()
    t_metric_table_matches_the_rows(rows, text)
    t_gain_table_matches_rows_and_recovered(rows, rec, text)
    t_premise_table_matches_the_rows(rows, text)
    t_the_piantor_perfect_repair_figures(rows, text)
    t_the_separation_and_the_blocking_claims(rows, text)
    t_the_timing_side_finding(rows, text)
    t_the_undrivable_number_is_disclosed(text)
    t_the_partial_rows_are_disclosed(rows, text)
    t_the_gate_is_not_vacuous()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s)')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
