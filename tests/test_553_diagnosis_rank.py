"""#553: the mover ranking, and every way it is allowed to say "I cannot rank".

`placement/diagnosis.py` replaces a pin-count filter with three independent
rankings. The failure this file exists to catch is NOT a wrong number -- it is a
CONFIDENT number where there is no evidence:

  * a block omitted for want of foreign pads reported as 0.0 rather than
    omitted ("connects to nothing outside itself" and "sits exactly on its
    partners" are different facts, and a zero reads as the second);
  * a blocker entry with no cell count imputed a count of 1;
  * GND -- which on a real board is owned by half the parts -- attributed to
    every candidate, so the signal is constant and the ranking is noise;
  * three signals folded into one scalar, which asserts an exchange rate
    between millimetres, cells and defect pairs that nobody has measured;
  * a signal whose candidates all scored the same, ranked anyway.

Each of those has a check here, and `tests/mutate_553.py` re-introduces them one
at a time to prove these checks are not vacuous.

No board and no router: the ranking reads `state.parts`, `state.net_refs` and
`Part.pad_globals()`, so a handful of fakes exercises it exactly. That also
keeps this file in `run_all.py --fast`.

    python3 -X utf8 tests/test_553_diagnosis_rank.py
"""

import json
import os
import sys

RUN_ALL_TIMEOUT = 60

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # placement split

from placement import diagnosis as D  # noqa: E402

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


# --------------------------------------------------------------------------
# Fakes: the whole surface `diagnose` touches.
# --------------------------------------------------------------------------

class _Part:
    def __init__(self, pads):
        self._pads = list(pads)          # [(x, y, net_id)]

    def pad_globals(self):
        return list(self._pads)


class _State:
    def __init__(self, parts, net_refs):
        self.parts = parts
        self.net_refs = net_refs


class _Net:
    def __init__(self, name):
        self.name = name


class _PCB:
    def __init__(self, names):
        self.nets = {nid: _Net(n) for nid, n in names.items()}


# Two blocks and two loose parts.
#   net 1 (/D0): A1 A2 C1      -> sheet:mag owns the plurality
#   net 2 (/D1): B1 C2
#   net 9 (GND): everything    -> the rail that must never rank anything
#
# The block NAMES are deliberately not in value order: `sheet:mag` outranks
# `sheet:acq` on every signal while sorting after it. The first version of this
# fixture called them sheet:A and sheet:B, and `mutate_553.py`'s `identity-rank`
# row -- ranking by NAME instead of by value -- SURVIVED, because alphabetical
# order happened to reproduce the right answer. A fixture whose ordering
# coincides with the tie-break cannot test the ordering.
NET_NAMES = {1: '/D0', 2: '/D1', 9: 'GND'}
BLOCKS = {'sheet:mag': ['A1', 'A2'], 'sheet:acq': ['B1', 'B2']}


def _state(**over):
    parts = {
        'A1': _Part([(0.0, 0.0, 1), (0.0, 0.5, 9)]),
        'A2': _Part([(1.0, 0.0, 1), (1.0, 0.5, 9)]),
        'B1': _Part([(10.0, 0.0, 2), (10.0, 0.5, 9)]),
        'B2': _Part([(11.0, 0.0, 2), (11.0, 0.5, 9)]),
        'C1': _Part([(50.0, 0.0, 1), (50.0, 0.5, 9)]),
        'C2': _Part([(12.0, 0.0, 2), (12.0, 0.5, 9)]),
    }
    net_refs = {1: ['A1', 'A2', 'C1'], 2: ['B1', 'B2', 'C2'],
                9: ['A1', 'A2', 'B1', 'B2', 'C1', 'C2']}
    parts.update(over.pop('parts', {}))
    net_refs.update(over.pop('net_refs', {}))
    return _State(parts, net_refs), _PCB(NET_NAMES)


REPORT = [
    {'net': '/X', 'stage': 'phase3', 'more': 2, 'blocked_by': [
        {'net': '/D0', 'blocked_count': 100, 'unique_cells': 20},
        {'net': 'GND', 'blocked_count': 900, 'unique_cells': 400},
    ]},
    {'net': '/Y', 'stage': 'phase3', 'blocked_by': [
        {'net': '/D1', 'blocked_count': 10, 'unique_cells': 2},
    ]},
]

LEGALITY = {'pairs': [('A1', 'C1', 'pad_clearance'),
                      ('C1', 'C2', 'body_overlap'),
                      ('A1', 'A2', 'pad_clearance')],
            'clearance': 0.15, 'notes': []}


def _run(**kw):
    state, pcb = _state()
    args = dict(blocker_report=REPORT, legality=LEGALITY,
                ignore_net_ids=None, max_fanout=4)
    args.update(kw)
    blocks = args.pop('blocks', BLOCKS)
    return D.diagnose(state, pcb, blocks, **args)


# --------------------------------------------------------------------------
# The router's evidence
# --------------------------------------------------------------------------

def t_evidence_tolerates_every_emitted_shape():
    weird = [
        {'net': '/A'},                                   # no blocked_by at all
        {'net': '/B', 'blocked_by': []},                 # empty
        {'net': '/C', 'blocked_by': [{'preexisting': True}]},   # no 'net'
        'not a dict',
        {'net': '/D', 'stage': 'preexisting', 'blocked_by': [
            {'net': '/VBUS', 'preexisting': True}]},
        {'net': '/E', 'more': 5, 'blocked_by': [
            {'net': '/SIG', 'blocked_count': 7},          # no unique_cells
            'not a dict']},
    ]
    ev = D.blocker_evidence(weird)
    check(ev.cells == {'/SIG': 7}, 'only the counted net contributes cells')
    check(ev.countless == ('/VBUS',),
          'the count-less net is NAMED, not silently dropped')
    check(ev.cells_dropped == 5 and ev.truncated_nets == 1,
          "the 'more' tail is carried so a share can disclose its truncation")
    check(ev.unique == {'/SIG': 0},
          'a missing unique_cells is 0, and it is not the ranked quantity')
    check(D.blocker_evidence(None).total_cells == 0,
          'None yields an empty evidence rather than raising')


def t_no_count_is_ever_imputed():
    ev = D.blocker_evidence([{'net': '/A', 'stage': 'preexisting',
                              'blocked_by': [{'net': 'GND',
                                              'preexisting': True}]}])
    check(ev.total_cells == 0, 'a preexisting-only report attributes NO cells')
    check('GND' not in ev.cells, 'the count-less net gets no fabricated count')
    check(ev.countless == ('GND',), 'it is reported instead')


def t_a_net_counted_anywhere_is_not_also_countless():
    ev = D.blocker_evidence([
        {'net': '/A', 'blocked_by': [{'net': 'GND', 'blocked_count': 5}]},
        {'net': '/B', 'stage': 'preexisting',
         'blocked_by': [{'net': 'GND', 'preexisting': True}]}])
    check(ev.cells == {'GND': 5}, 'the real count survives')
    check(ev.countless == (),
          'a net with evidence somewhere is not also reported as uncounted')


# --------------------------------------------------------------------------
# Degeneracy: every collapse must be NAMED
# --------------------------------------------------------------------------

def t_no_blocks_skips_displacement_by_name():
    d = _run(blocks={})
    check(d.skipped.get('block_displacement') == D._NO_BLOCKS,
          'no blocks -> the displacement signal is skipped with the reason')
    check('block_displacement' not in d.signals_defined,
          'and it does not appear as a defined signal')
    check(d.selected, 'the other signals still rank, so the run continues')


def t_a_block_with_no_foreign_pads_is_omitted_not_zeroed():
    state, pcb = _state()
    # sheet:acq keeps only its own net: no pad outside the block is on it.
    state.net_refs[2] = ['B1', 'B2']
    state.parts['C2'] = _Part([(12.0, 0.0, 7)])
    d = D.diagnose(state, pcb, BLOCKS, blocker_report=None, legality=None,
                   max_fanout=4)
    vals = {c.key: [r.value for r in c.rows if r.signal == 'block_displacement']
            for c in d.candidates}
    check(vals.get('sheet:acq') in (None, []),
          'a block with no foreign pads carries no displacement row')
    check(all(v != [0.0] for v in vals.values()),
          'and is never reported as a 0.0 distance')
    check(any('omitted' in n for n in d.disclosures),
          'the omission is disclosed rather than left as a silent absence')


def t_a_high_fanout_rail_ranks_nothing():
    """GND owns 96 of ulx3s's parts. Unfiltered it dominates every candidate."""
    d = _run(max_fanout=4)
    rows = {c.key: [r for r in c.rows if r.signal == 'blocker_cells']
            for c in d.candidates}
    total = sum(r.value for rs in rows.values() for r in rs)
    check(total == 110,
          f'only /D0 (100) and /D1 (10) rank; GND 900 is cut (got {total})')
    check(any('high-fanout' in n for n in d.disclosures),
          'the dropped rail cells are disclosed, not silently discarded')
    # And with the cut OFF, GND is exactly the degeneracy the cut exists for.
    loose = _run(max_fanout=0)
    loose_total = sum(r.value for c in loose.candidates for r in c.rows
                      if r.signal == 'blocker_cells')
    check(loose_total > total,
          'without the cut the rail dominates -- the cut is doing real work')


def t_ignore_net_ids_also_cuts_the_rail():
    d = _run(max_fanout=0, ignore_net_ids=[9])
    total = sum(r.value for c in d.candidates for r in c.rows
                if r.signal == 'blocker_cells')
    check(total == 110, 'the plane-net set cuts GND with the fanout cut off')


def t_no_spread_refuses_to_rank():
    state, pcb = _state()
    legality = {'pairs': [('A1', 'C1', 'pad_clearance'),
                          ('A2', 'C2', 'pad_clearance')],
                'clearance': 0.15, 'notes': []}
    d = D.diagnose(state, pcb, {}, blocker_report=None, legality=legality,
                   max_fanout=4)
    # sheet:mag is gone (no blocks), so A1/A2/C1/C2 each score exactly 1.
    check('no spread' in d.skipped.get('legality_pairs', ''),
          f'all-equal candidates are refused, not ranked '
          f'({d.skipped.get("legality_pairs")!r})')


def t_a_grader_note_is_propagated_not_swallowed():
    d = _run(legality={'pairs': [], 'clearance': 0.15,
                       'notes': ['courtyard census: no courtyards found']})
    check(any('courtyard census' in n for n in d.disclosures),
          'a census that did not run says so instead of reporting "none"')
    check('no legality findings' in d.skipped.get('legality_pairs', ''),
          'and the signal is skipped by name')


def t_everything_undefined_is_a_named_fallback():
    state, pcb = _state()
    d = D.diagnose(state, pcb, {}, blocker_report=None, legality=None,
                   max_fanout=4)
    check(d.degenerate, 'no signal defined -> degenerate')
    check(d.selected == [], 'and nothing is selected')
    reason = d.fallback_reason()
    check(all(s in reason for s in D.SIGNAL_ORDER),
          f'the fallback reason names every signal and why ({reason!r})')


# --------------------------------------------------------------------------
# Attribution and combination
# --------------------------------------------------------------------------

def t_plurality_attribution_and_its_disclosure():
    d = _run()
    cells = {c.key: r.value for c in d.candidates for r in c.rows
             if r.signal == 'blocker_cells'}
    check(cells.get('sheet:mag') == 100,
          f'/D0 goes whole to the block owning 2 of its 3 pad owners ({cells})')
    check('C1' not in cells,
          'the minority owner does not also get the cells')


def t_legality_pairs_attribute_to_both_ends_and_count_internal_once():
    d = _run()
    pairs = {c.key: r.value for c in d.candidates for r in c.rows
             if r.signal == 'legality_pairs'}
    # (A1,C1) -> sheet:mag + C1 ; (C1,C2) -> C1 + C2 ; (A1,A2) -> sheet:mag once
    check(pairs.get('sheet:mag') == 2,
          f'a pair internal to a block counts ONCE for it ({pairs})')
    check(pairs.get('C1') == 2 and pairs.get('C2') == 1,
          f'each loose end carries its own pairs ({pairs})')
    det = [r.detail for c in d.candidates for r in c.rows
           if c.key == 'sheet:mag' and r.signal == 'legality_pairs'][0]
    check(det.get('internal') == 1, 'the internal pair is disclosed as such')
    check(det.get('kinds') == ['pad_clearance'], 'and the kinds are named')


def t_round_robin_not_concatenation():
    """Concatenation lets one signal own the head; round-robin cannot.

    The discriminating fact on this fixture: `block_displacement` ranks
    sheet:mag then sheet:acq, and `legality_pairs` ranks C1 first. Concatenating
    the per-signal lists takes BOTH of displacement's before C1; sweeping them
    takes C1 second. So the position of C1 relative to sheet:acq IS the rule.
    """
    d = _run()
    order = d.selected_keys
    check(order[0] == 'sheet:mag', f'the strongest candidate still leads {order}')
    check(order.index('C1') < order.index('sheet:acq'),
          f"legality's top-1 is reached before displacement's rank-2 -- "
          f'concatenation would reverse these ({order})')
    by = {c.key: c.selected_by for c in d.candidates}
    check(by.get('sheet:mag') == D.SIGNAL_ORDER,
          f'a candidate ranked by all three records all three ({by})')
    tops = {}
    for sig in D.SIGNAL_ORDER:
        rows = sorted(((c.key, r.value) for c in d.candidates for r in c.rows
                       if r.signal == sig), key=lambda kv: (-kv[1], kv[0]))
        if rows:
            tops[sig] = rows[0][0]
    for sig, key in tops.items():
        check(key in d.selected_keys,
              f'{sig} top-1 ({key}) reached the selection')
        check(sig in by.get(key, ()),
              f'and {key} records {sig} as a reason it was selected')


def t_rank_sum_is_advisory_only():
    d = _run()
    src = open(os.path.join(ROOT, 'py_placer', 'placement', 'diagnosis.py'),
               encoding='utf-8').read()
    sel = src.split('# ---- round-robin union')[1].split('d.candidates =')[0]
    check('rank_sum' not in sel,
          'the selection loop does not read rank_sum')
    check(all('rank_sum_advisory' in c.to_dict() for c in d.candidates),
          'but it is reported, so a future measurement has a candidate')


def t_no_combined_score_is_ever_emitted():
    d = _run()
    blob = json.dumps(d.to_dict())
    for banned in ('"score"', '"total_score"', '"combined"', '"weight"'):
        check(banned not in blob,
              f'the report emits no {banned} -- there is no exchange rate')


# --------------------------------------------------------------------------
# Budget, determinism, disclosure
# --------------------------------------------------------------------------

def t_a_block_is_added_whole_and_the_overshoot_is_reported():
    d = _run(budget=1)
    check(d.selected_keys == ['sheet:mag'],
          f'a budget of 1 part stops after the first candidate ({d.selected_keys})')
    check(sorted(d.selected) == ['A1', 'A2'],
          f'and that block came WHOLE -- never half-moved ({d.selected})')
    check(d.overshoot == 1,
          f'the overshoot past the budget is reported ({d.overshoot})')
    check(any('overshot' in n for n in d.disclosures),
          'and disclosed in words rather than left to arithmetic')
    check(_run(budget=None).selected_keys != ['sheet:mag'],
          'without a budget the sweep continues -- the cap is doing the work')


def t_selected_by_is_only_recorded_for_selected_keys():
    """A signal may not claim credit for a candidate it merely ranked."""
    d = _run(budget=1)
    claimed = sorted(c.key for c in d.candidates if c.selected_by)
    check(claimed == ['sheet:mag'],
          f'only the selected candidate carries a selected_by ({claimed})')
    for c in d.candidates:
        if c.selected_by:
            check(c.key in d.selected_keys,
                  f'{c.key} claims {c.selected_by} and IS selected')


def t_the_report_is_json_and_stable():
    a = _run()
    b = _run()
    blob = json.dumps(a.to_dict(), sort_keys=True)
    check(blob == json.dumps(b.to_dict(), sort_keys=True),
          'two runs of the same input produce the same report')
    check(isinstance(json.loads(blob), dict),
          'the report round-trips through JSON (no tuple or dataclass leaks)')


def t_input_order_cannot_change_the_ranking():
    """Dict insertion order stands in for the hash-seed question here."""
    state, pcb = _state()
    rev = {k: BLOCKS[k] for k in reversed(list(BLOCKS))}
    rev_members = {k: list(reversed(v)) for k, v in rev.items()}
    a = D.diagnose(state, pcb, BLOCKS, blocker_report=REPORT,
                   legality=LEGALITY, max_fanout=4)
    b = D.diagnose(state, pcb, rev_members, blocker_report=list(reversed(REPORT)),
                   legality=dict(LEGALITY,
                                 pairs=list(reversed(LEGALITY['pairs']))),
                   max_fanout=4)
    check(a.selected_keys == b.selected_keys,
          f'the selection is order-independent ({a.selected_keys} vs '
          f'{b.selected_keys})')
    check(json.dumps(a.to_dict(), sort_keys=True)
          == json.dumps(b.to_dict(), sort_keys=True),
          'and so is the whole report')


def t_the_no_efficacy_claim_travels_with_the_result():
    d = _run()
    check('NOT MEASURED' in d.efficacy and d.efficacy == D.NO_EFFICACY_CLAIM,
          'the run carries the no-efficacy sentence')
    check(d.to_dict()['efficacy'] == D.NO_EFFICACY_CLAIM,
          'a machine consumer cannot read the verdict without it')
    check('efficacy' in D.format_text(d),
          'and neither can a human reading the table')


def t_format_text_names_the_skipped_signals():
    d = _run(blocks={})
    txt = D.format_text(d)
    check('block_displacement' in txt and 'SKIPPED' in txt,
          'a skipped signal is printed with its reason, not omitted')
    check('round-robin' in txt and 'exchange rate' in txt,
          'the table states there is no combined score')


def t_the_ranking_path_does_no_io():
    import inspect
    src = inspect.getsource(D.diagnose)
    for banned in ('open(', 'parse_kicad_pcb', 'grade_pad_legality',
                   'os.path', 'Popen'):
        check(banned not in src,
              f'diagnose() does no IO: no {banned}')


TESTS = [
    t_evidence_tolerates_every_emitted_shape,
    t_no_count_is_ever_imputed,
    t_a_net_counted_anywhere_is_not_also_countless,
    t_no_blocks_skips_displacement_by_name,
    t_a_block_with_no_foreign_pads_is_omitted_not_zeroed,
    t_a_high_fanout_rail_ranks_nothing,
    t_ignore_net_ids_also_cuts_the_rail,
    t_no_spread_refuses_to_rank,
    t_a_grader_note_is_propagated_not_swallowed,
    t_everything_undefined_is_a_named_fallback,
    t_plurality_attribution_and_its_disclosure,
    t_legality_pairs_attribute_to_both_ends_and_count_internal_once,
    t_round_robin_not_concatenation,
    t_rank_sum_is_advisory_only,
    t_no_combined_score_is_ever_emitted,
    t_a_block_is_added_whole_and_the_overshoot_is_reported,
    t_selected_by_is_only_recorded_for_selected_keys,
    t_the_report_is_json_and_stable,
    t_input_order_cannot_change_the_ranking,
    t_the_no_efficacy_claim_travels_with_the_result,
    t_format_text_names_the_skipped_signals,
    t_the_ranking_path_does_no_io,
]


def main():
    for fn in TESTS:
        print(f'{fn.__name__}:')
        try:
            fn()
        except Exception as e:                       # noqa: BLE001
            check(False, f'{fn.__name__} raised {type(e).__name__}: {e}')
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_553_diagnosis_rank: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
