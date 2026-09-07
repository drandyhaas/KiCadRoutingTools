#!/usr/bin/env python3
"""`proximity[]`: the constraint the netlist implies and nothing measured (#902).

A 3mm crystal loop and a 30mm one have IDENTICAL connectivity. Every instrument
in this toolchain reads the board, so "Y1 beside U1, as short as possible" was
ungradable -- and run 25 shipped a board whose brief said exactly that, with
`rules_run: 6` and every declared clause ungraded.

This file holds the BRIEF half: the vocabulary, its refusals, and the
three-state contract. The rule that grades the compiled rows is
`rule_proximity`, and its measurements are tested beside it.

Most of these rows exist because an adversarial verifier found the defect they
pin. Every one of the shapes under `_REFUSALS` LOADED CLEAN at some point in
this feature's history, so they are change detectors with a scar, not a
catalogue of things that were never going to happen.

    python3 tests/test_902_proximity.py
"""
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)

from placement import design_brief as db          # noqa: E402
from placement import floorplan as fp             # noqa: E402

RUN_ALL_FAST_OK = True

BASE = {'schema': 1, 'kind': 'design-brief', 'units': 'mm'}
REFS = ('Y1', 'U1', 'U2', 'C1', 'C3', 'Q1', 'Q2')


def _brief(rows, **over):
    raw = dict(BASE, **over)
    raw['proximity'] = rows
    return db.brief_from_dict(raw, 'b.design-brief.json')


def _compile(rows, refs=REFS, **over):
    return db.compile_brief(_brief(rows, **over), board_refs=list(refs))


def _rejects(rows, why, **over):
    """Assert the document is refused AND that the message carries the REASON.

    A refusal is not evidence on its own: the first draft of several of these
    cases "passed" against a message about a completely different key.
    """
    try:
        b = _brief(rows, **over)
        db.compile_brief(b, board_refs=list(REFS))
    except db.BriefError as exc:
        msg = str(exc)
        assert why in msg, (why, msg)
        return msg
    raise AssertionError(f"NOT REFUSED, expected {why!r}: {rows!r}")


#: (rows, the substring the refusal must carry). Each was reachable.
_REFUSALS = [
    ([{'ref': 'Y1', 'near': 'U1'}], 'needs `max_mm`'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 0}], 'expected a positive'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': -1}], 'expected'),
    # inf and nan pass BOTH `_number(lo=0.0)` and a `<= 0.0` guard, and
    # json.load accepts the literals, so this was reachable from a file.
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': float('inf')}],
     'not a finite distance'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': float('nan')}],
     'not a finite distance'),
    ([{'ref': 'Y1', 'near': 'Y1', 'max_mm': 2}], '0mm from itself'),
    ([{'ref': ['Y1', 'U1'], 'near': 'U1', 'max_mm': 2}], '0mm from itself'),
    ([{'ref': ['Y1', 'Y1'], 'near': 'U1', 'max_mm': 2}], 'names a part twice'),
    ([{'ref': 'unknown', 'near': 'U1', 'max_mm': 2}], 'is not a row'),
    ([{'ref': 'Y1', 'near': 'unknown', 'max_mm': 2}], 'no partner measures'),
    ([{'ref': 'Y1', 'max_mm': 2}], 'needs `near`'),
    # `basis` is the ONE enum here with a default, so "unknown" would compile
    # to that default while the report says nobody knows.
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'basis': 'unknown'}],
     'HAS a default'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'basis': 'courtyard'}],
     '0 of 21'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'basis': 'silk'}],
     'expected one of'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'min_mm': 0.2}],
     "clearance channel's claim"),
    ([{'ref': 'Y1', 'near': 'U1', 'max_distance_mm': 2}],
     'the `decaps` spelling'),
    ([{'reff': 'Y1', 'near': 'U1', 'max_mm': 2}], 'unknown key(s) reff'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'pads': {'Q9': ['1']}}],
     'says nothing about'),
    # A JSON int matches no pad, resolves an empty set and grades CLEAN.
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'pads': {'Y1': [1]}}],
     'would match no pad'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'pads': {'Y1': []}}],
     'non-empty list'),
    # An empty dict was a THIRD spelling of "unpadded" the reverse guard did
    # not recognise.
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'pads': {}}], 'empty pad spec'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'pads': 7}], 'expected'),
    # Two limits on one relation, in both spellings of "the same relation".
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2},
      {'ref': 'Y1', 'near': 'U1', 'max_mm': 3}], 'duplicate proximity'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 5},
      {'ref': 'U1', 'near': 'Y1', 'max_mm': 9}], 'same symmetric measurement'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 5, 'pads': 'unknown'},
      {'ref': 'U1', 'near': 'Y1', 'max_mm': 9, 'pads': 'unknown'}],
     'same symmetric measurement'),
    # A pad list naming only the PARTNER leaves both rows existential, so they
    # are still the same measurement.
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 5, 'pads': {'U1': ['9']}},
      {'ref': 'U1', 'near': 'Y1', 'max_mm': 9, 'pads': {'Y1': ['1']}}],
     'same symmetric measurement'),
]


def test_every_malformed_shape_is_refused_by_its_reason():
    for rows, why in _REFUSALS:
        _rejects(rows, why)
    print(f"  PASS: {len(_REFUSALS)} malformed shapes, each refused by its "
          f"reason")


def test_a_whole_key_unknown_is_refused_naming_the_key_not_a_character():
    """`"proximity": "unknown"` used to be refused by ITERATING THE STRING.

    The message then named `proximity[0]` -- the character `u` -- and sent the
    author to a row that does not exist.
    """
    try:
        db.brief_from_dict(dict(BASE, proximity='unknown'))
        raise AssertionError('NOT REFUSED')
    except db.BriefError as exc:
        msg = str(exc)
    assert 'expected a list of rows' in msg, msg
    assert 'proximity[0]' not in msg, msg
    print(f"  PASS: {msg.splitlines()[0][:88]}")


def test_a_reversed_pair_with_real_subject_pads_is_KEPT():
    """The negative control for the reverse guard.

    A guard that refuses everything is not a guard. With a SUBJECT pad list on
    each side the two rows are genuinely different claims -- "for each of MY
    pads, some pad of yours is close enough" is not symmetric -- and both must
    survive.
    """
    frag, _ = _compile([
        {'ref': 'Y1', 'near': 'U1', 'max_mm': 5, 'pads': {'Y1': ['1']}},
        {'ref': 'U1', 'near': 'Y1', 'max_mm': 9, 'pads': {'U1': ['9']}}])
    assert len(frag['proximity']) == 2, frag['proximity']
    print("  PASS: a reversed pair with subject pads on both sides is kept -- "
          "the guard refuses the symmetric case only")


def test_the_three_states_stay_apart():
    """declared / declared-unknown / absent, at claim granularity.

    `max_mm: "unknown"` is DECLARED -- "as short as possible" is a real thing
    for a spec to say -- and compiles to NO row, because a limit nobody stated
    cannot be graded and inventing one is the guess this channel refuses.
    """
    frag, rep = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 'unknown'}])
    assert 'proximity' not in frag, frag
    assert rep['unknown'] == ['proximity[0:Y1~U1].max_mm'], rep['unknown']
    assert not [d for d in rep['declared'] if 'proximity' in d], rep['declared']

    # `pads: "unknown"` is the other arity: the row still grades, part to part.
    frag, rep = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0,
                           'pads': 'unknown'}])
    assert len(frag['proximity']) == 1
    assert 'pads' not in frag['proximity'][0], frag['proximity'][0]
    assert 'proximity[0:Y1~U1].pads' in rep['unknown'], rep['unknown']

    # BOTH unknown must report BOTH. The `continue` used to fire first, so an
    # author who wrote two "I do not know"s saw one.
    _frag, rep = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 'unknown',
                            'pads': 'unknown'}])
    assert rep['unknown'] == ['proximity[0:Y1~U1].max_mm',
                              'proximity[0:Y1~U1].pads'], rep['unknown']

    # The two sets are disjoint, or they are not two states.
    frag, rep = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0},
                          {'ref': 'C1', 'near': 'U2', 'max_mm': 'unknown'}])
    assert not (set(rep['declared']) & set(rep['unknown']))
    print("  PASS: an unknown limit compiles to no row and is reported; an "
          "unknown pads keeps the row; both together report both; disjoint")


def test_a_claim_id_survives_a_reference_that_contains_a_tilde():
    """`TP4~2` is a reference this toolchain PRODUCES.

    `disambiguate_references` spells a duplicated refdes that way and esp_prog
    itself parses `Ref*~2`. With a bare `ref~near` id, a row `A~B` near `C` and
    a row `A` near `B~C` collide -- and since `unknown` is a set union, one of
    two DECLARED "I do not know"s silently disappeared.
    """
    _frag, rep = _compile([{'ref': 'A~B', 'near': 'C', 'max_mm': 'unknown'},
                           {'ref': 'A', 'near': 'B~C', 'max_mm': 'unknown'}],
                          refs=('A~B', 'C', 'A', 'B~C'))
    assert len(rep['unknown']) == 2, rep['unknown']
    assert len(set(rep['unknown'])) == 2, rep['unknown']
    # And the id is the one shared function's, not a local f-string.
    assert db.proximity_claim_id(0, 'A~B', 'C') + '.max_mm' in rep['unknown']
    print(f"  PASS: two tilde-bearing claims keep two ids: {rep['unknown']}")


def test_a_list_ref_expands_and_the_intent_never_carries_the_sugar():
    frag, rep = _compile([{'ref': ['C1', 'C3'], 'near': 'U2', 'max_mm': 2.0,
                           'pads': {'C1': ['1'], 'C3': ['1'],
                                    'U2': ['2', '3']}}])
    rows = frag['proximity']
    assert [r['ref'] for r in rows] == ['C1', 'C3'], rows
    assert all(isinstance(r['ref'], str) for r in rows), rows
    # Each member keeps ONLY its own pads plus the partner's.
    assert rows[0]['pads'] == {'C1': ['1'], 'U2': ['2', '3']}, rows[0]
    assert rows[1]['pads'] == {'C3': ['1'], 'U2': ['2', '3']}, rows[1]
    # And each traces back to the line its author wrote.
    assert all(r['context']['brief_row'] == 0 for r in rows), rows
    assert rep['counts']['proximity'] == 1
    assert rep['counts']['proximity_claims'] == 2
    print("  PASS: one row with a list ref compiles to two single-ref claims, "
          "each carrying its own pads and its source row")


def test_min_reader_is_the_running_max_and_is_not_written_for_a_dropped_row():
    """The field whose only job is to be true.

    A brief carrying both an along-edge claim and a proximity row needs the
    HIGHER reader; writing whichever branch ran last understates it.
    """
    frag, _ = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0}])
    assert frag['min_reader'] == 4, frag

    along = [{'ref': 'USB1', 'edge': 'east', 'along_edge': 'center',
              'along_edge_tolerance_mm': 0.5}]
    frag, _ = _compile([], interfaces=along)
    assert frag['min_reader'] == 2, frag

    frag, _ = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0}],
                       interfaces=along)
    assert frag['min_reader'] == 4, frag

    # A row that compiled to NOTHING must not claim a reader for a key the
    # document does not carry.
    frag, _ = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 'unknown'}])
    assert 'min_reader' not in frag, frag

    frag, _ = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 'unknown'}],
                       interfaces=along)
    assert frag['min_reader'] == 2, frag

    # And the MERGE maxes too, one line below where the fragment does.
    rep = {'contradictions': [], 'path': '', 'declared': [], 'unknown': [],
           'absent': [], 'not_graded': [], 'unmatched': [],
           'unmatched_checked': False, 'counts': {}, 'fixed': [], 'product': {}}
    merged = db.merge_into_intent({'min_reader': 3}, {'min_reader': 2}, rep)
    assert merged['min_reader'] == 3, merged['min_reader']
    print("  PASS: min_reader 4 / 2 / 4, absent for a dropped row, and the "
          "merge takes the max rather than the last writer")


def test_the_counts_disclose_an_expansion_and_a_drop_that_cancel():
    """Three rows, three claims -- and one of each event underneath.

    The first version compared the two counts, which cancel exactly here, so
    the line disclosed nothing on the very shape this feature was built for.
    """
    _frag, rep = _compile([
        {'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0},
        {'ref': ['C1', 'C3'], 'near': 'U2', 'max_mm': 2.0},
        {'ref': 'Q1', 'near': 'Q2', 'max_mm': 'unknown'}])
    c = rep['counts']
    assert c['proximity'] == 3 and c['proximity_claims'] == 3, c
    assert c['proximity_expanded'] == 1 and c['proximity_dropped'] == 1, c
    line = db.format_report(rep, path='b.design-brief.json')
    assert '3 proximity row(s) -> 3 claim(s)' in line, line
    print(f"  PASS: {line}")


def test_a_brief_with_no_proximity_reports_exactly_what_it_did_before():
    """`counts` is emitted verbatim as `design_brief.counts` by board_brief.

    An unconditional key would make every board on the corpus start reporting
    zero of a thing nobody mentioned.
    """
    _frag, rep = _compile([])
    assert rep['counts'] == {'interfaces': 0, 'keepouts': 0, 'fixed': 0}, \
        rep['counts']
    assert 'proximity' not in db.format_report(rep, path='b.json')
    print("  PASS: no proximity -> the counts dict and the report line are "
          "the pre-#902 ones")


def test_an_absent_partner_is_named_once_not_once_per_member():
    """`unmatched` was not deduped, and a list `ref` appends `near` per member.

    check_floorplan prints one line per entry, so one missing part was
    reported twice.
    """
    _frag, rep = _compile([{'ref': ['C1', 'C3'], 'near': 'ZZ9',
                            'max_mm': 2.0}], refs=('C1', 'C3', 'U2'))
    assert rep['unmatched'] == ['ZZ9'], rep['unmatched']
    print("  PASS: an absent partner named once, not once per list member")


def test_drift_compares_at_the_effective_value_not_at_key_presence():
    """A brief meaning the DEFAULT versus an intent declaring otherwise.

    The compiler writes `basis` only when non-default, so `if field in p`
    reported no drift while the grade would have measured bodies against a
    clause written about pads.
    """
    frag, _ = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 3.0}])
    lines = db.drift({'proximity': [{'ref': 'Y1', 'near': 'U1', 'max_mm': 3.0,
                                     'basis': 'body',
                                     'pads': {'Y1': ['9']}}]}, frag)
    assert any('basis' in ln and 'pad_edge' in ln and 'body' in ln
               for ln in lines), lines
    assert any('pads' in ln for ln in lines), lines
    # Identical documents drift by nothing -- the vacuity control.
    assert db.drift({'proximity': [{'ref': 'Y1', 'near': 'U1',
                                    'max_mm': 3.0}]}, frag) == []
    # A claim the intent has no row for at all.
    assert db.drift({'proximity': []}, frag)[0].startswith('Y1 near U1:')
    print(f"  PASS: drift sees a default-vs-declared divergence "
          f"({len(lines)} line(s)) and is silent on identical documents")


def test_the_basis_vocabulary_matches_the_intent_loader_or_says_it_cannot_yet():
    """The brief must not accept a spelling the intent loader then refuses.

    An author would see their own compiled document rejected by the same
    binary that wrote it. This assertion goes LIVE the moment `floorplan`
    grows the tuple; until then it prints that it is vacuous rather than
    passing silently, because a guard that cannot fire is not a guard.
    """
    theirs = getattr(fp, '_PROXIMITY_BASES', None)
    if theirs is None:
        print("  PASS (VACUOUS): floorplan has no _PROXIMITY_BASES yet -- the "
              "intent half is a later phase; this row is a change detector "
              "waiting to arm, not a check that ran")
        return
    assert tuple(theirs) == tuple(db._PROXIMITY_BASES), (theirs,
                                                         db._PROXIMITY_BASES)
    print(f"  PASS: the brief and the intent accept the same bases {theirs}")


def test_the_fixture_brief_compiles_to_the_claims_the_issue_names():
    """The acceptance shape, from the tracked fixture rather than from prose.

    Three rows -- a crystal to its load pins, a regulator's two bulk caps, and
    a transistor pair with no shared net -- compile to the four claims #902
    says nothing could grade.
    """
    path = os.path.join(ROOT, 'tests', 'fixtures', '902',
                        'esp_prog_proximity.design-brief.json')
    assert os.path.isfile(path), path
    with open(path, encoding='utf-8') as fh:
        raw = json.load(fh)
    brief = db.brief_from_dict(raw, path)
    frag, rep = db.compile_brief(brief, board_refs=list(REFS) + ['USB1'])
    got = [(r['ref'], r['near'], r['max_mm'], r.get('basis')) for r in
           frag['proximity']]
    assert got == [('Y1', 'U1', 2.0, 'pad_edge'),
                   ('C1', 'U2', 2.0, 'pad_edge'),
                   ('C3', 'U2', 2.0, 'pad_edge'),
                   ('Q1', 'Q2', 3.0, 'body')], got
    assert frag['min_reader'] == 4
    assert rep['unmatched'] == [], rep['unmatched']
    assert rep['counts']['proximity'] == 3
    assert rep['counts']['proximity_claims'] == 4
    print(f"  PASS: the fixture brief compiles to {len(got)} claims, "
          f"3 pad_edge and 1 body")


TESTS = [
    test_every_malformed_shape_is_refused_by_its_reason,
    test_a_whole_key_unknown_is_refused_naming_the_key_not_a_character,
    test_a_reversed_pair_with_real_subject_pads_is_KEPT,
    test_the_three_states_stay_apart,
    test_a_claim_id_survives_a_reference_that_contains_a_tilde,
    test_a_list_ref_expands_and_the_intent_never_carries_the_sugar,
    test_min_reader_is_the_running_max_and_is_not_written_for_a_dropped_row,
    test_the_counts_disclose_an_expansion_and_a_drop_that_cancel,
    test_a_brief_with_no_proximity_reports_exactly_what_it_did_before,
    test_an_absent_partner_is_named_once_not_once_per_member,
    test_drift_compares_at_the_effective_value_not_at_key_presence,
    test_the_basis_vocabulary_matches_the_intent_loader_or_says_it_cannot_yet,
    test_the_fixture_brief_compiles_to_the_claims_the_issue_names,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
