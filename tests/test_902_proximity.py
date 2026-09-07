#!/usr/bin/env python3
"""`proximity[]`: the constraint the netlist implies and nothing measured (#902).

A 3mm crystal loop and a 30mm one have IDENTICAL connectivity. Every instrument
in this toolchain reads the board, so "Y1 beside U1, as short as possible" was
ungradable -- and run 25 shipped a board whose brief said exactly that, with
`rules_run: 6` and every declared clause ungraded.

This file holds the BRIEF half: the vocabulary, its refusals, and the
three-state contract. Grading the compiled rows is the intent half's job, and
`test_the_basis_vocabulary_matches_the_intent_loader_or_says_it_cannot_yet`
reports which of the two states this tree is in rather than passing either
way.

FIVE of the twenty-six shapes under `_REFUSALS` actually LOADED CLEAN in this
feature's first commit -- rows 3, 4 (`inf` and `NaN` limits), 20 (`pads: {}`)
and 24, 25 (the two spellings of a reversed pair). They are marked `# WAS
CLEAN` below. The other twenty-one were refused from the start.

That sentence began life as "every one of them loaded clean", which was a
claim in the feature's own favour that nobody had measured. The number is
measurable -- run each shape against `git show 7c682f89:...design_brief.py` --
and measuring it turned 26 into 5. It is written out here because the marked
rows are the ones with a scar, and a reader deciding which of these to trust
should not have to take the docstring's word for it.

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
from placement import legality                    # noqa: E402


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
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': -1}], 'expected a positive'),
    # inf and nan pass BOTH `_number(lo=0.0)` and a `<= 0.0` guard, and
    # json.load accepts the literals, so this was reachable from a file.
    # WAS CLEAN at 7c682f89.
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': float('inf')}],
     'not a finite distance'),
    # WAS CLEAN at 7c682f89.
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
    # WAS CLEAN at 7c682f89.
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'pads': {}}], 'empty pad spec'),
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2, 'pads': 7}],
     "expected {'REF': ['1', '2']}"),
    # Two limits on one relation, in both spellings of "the same relation".
    ([{'ref': 'Y1', 'near': 'U1', 'max_mm': 2},
      {'ref': 'Y1', 'near': 'U1', 'max_mm': 3}], 'duplicate proximity'),
    # WAS CLEAN at 7c682f89 (both of the next two).
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


def test_an_absent_partner_is_named_even_when_the_limit_is_unknown():
    """A typo does not become invisible because the limit was not stated.

    The first fix for the "declared unknown was swallowed" defect moved the
    PADS report above the `continue` and left the UNMATCHED report below it --
    so a misspelled partner on an "as short as possible" row was reported by
    nothing at all. The same defect, one line further down, is why this row
    exists rather than an extra assert on the one above.
    """
    for limit in (2.0, 'unknown'):
        _frag, rep = _compile([{'ref': 'Y1', 'near': 'ZZ9', 'max_mm': limit}],
                              refs=('Y1', 'U1'))
        assert rep['unmatched'] == ['ZZ9'], (limit, rep['unmatched'])
    # And the control: a ref the board HAS is never reported.
    _frag, rep = _compile([{'ref': 'Y1', 'near': 'U1', 'max_mm': 'unknown'}],
                          refs=('Y1', 'U1'))
    assert rep['unmatched'] == [], rep['unmatched']
    print("  PASS: an absent partner is named with a limit and with an "
          "unknown one; a present partner is never named")


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


# --------------------------------------------------------------------------
# the RULE: what the compiled claims measure, on tracked boards
# --------------------------------------------------------------------------

#: The two boards, and what the SAME brief measures on each. The placed board
#: is run 25's result; the tracked one is where that run started. The rule is
#: what makes the difference between them visible at all -- and it needs no
#: fixture manipulation, because the improvement is real.
PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                      'esp_prog_placed.kicad_pcb')
UNPLACED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')

FIXTURE_BRIEF = os.path.join(ROOT, 'tests', 'fixtures', '902',
                             'esp_prog_proximity.design-brief.json')


def _graded(board, rows=None):
    from kicad_parser import parse_kicad_pcb
    if rows is None:
        with open(FIXTURE_BRIEF, encoding='utf-8') as fh:
            frag, _rep = db.compile_brief(db.brief_from_dict(json.load(fh)))
        rows = frag['proximity']
    intent = fp.intent_from_dict({'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND,
                                  'units': 'mm', 'proximity': rows})
    return fp.grade(intent, parse_kicad_pcb(board), board)


def _by_pad(result):
    return {(v.ref, v.measured.get('pad')): v for v in result.violations
            if v.rule == 'proximity'}


def test_the_acceptance_numbers_the_issue_names():
    """#902's own measurement, off the pad geometry rather than off prose.

    Asserted on `measured`, never on the message: a message assertion passes
    when the sentence is right and the number is wrong.
    """
    r = _graded(PLACED)
    assert 'proximity' in r.rules_run, r.rules_run
    got = _by_pad(r)
    assert len(got) == 1, sorted(got)
    v = got[('Y1', '1')]
    m = v.measured
    assert m['gap_mm'] == 3.1425, m
    assert (m['near'], m['near_pad']) == ('U1', '9'), m
    assert m['paired_by'] == 'net' and m['pads_basis'] == 'declared', m
    assert m['basis'] == 'pad_edge' and v.expected == {'max_mm': 2.0}, v
    assert v.severity == fp.ERROR, v.severity
    print(f"  PASS: one violation on the placed board -- {v.message}")


def test_the_same_brief_measures_the_board_the_run_started_from():
    """The rule is what makes run 25's improvement measurable.

    Three of the claims fail on the board the run started from and one fails
    on what it shipped. A single-board assertion could not tell "the rule
    works" from "the rule always fires".
    """
    placed, unplaced = _by_pad(_graded(PLACED)), _by_pad(_graded(UNPLACED))
    assert len(unplaced) == 3, sorted(unplaced)
    assert unplaced[('Y1', '1')].measured['gap_mm'] == 4.7425
    assert unplaced[('Y1', '2')].measured['gap_mm'] == 2.522
    assert unplaced[('C1', '1')].measured['gap_mm'] == 4.4111
    assert set(placed) < set(unplaced), (sorted(placed), sorted(unplaced))
    # The pad both boards flag got CLOSER, and that direction is the point.
    assert (placed[('Y1', '1')].measured['gap_mm']
            < unplaced[('Y1', '1')].measured['gap_mm'])
    print("  PASS: 3 violations where the run started, 1 where it ended, and "
          "the shared pad moved 4.7425 -> 3.1425mm")


def test_a_claim_the_board_satisfies_is_a_measured_clean_not_a_skip():
    """The negative control, and both halves are load-bearing.

    "No violations" and "the rule never ran" must not look the same -- this
    file's own subject, one level down. So the assertion is that `proximity`
    IS in `rules_run` AND produced nothing.
    """
    r = _graded(PLACED, rows=[{'ref': 'C3', 'near': 'U2', 'max_mm': 2.0,
                               'pads': {'C3': ['1'], 'U2': ['2', '3']}}])
    assert 'proximity' in r.rules_run, r.rules_run
    assert [v for v in r.violations if v.rule == 'proximity'] == []
    assert 'proximity' not in r.rules_skipped, r.rules_skipped
    print("  PASS: C3 at 0.29mm against a 2.0mm limit -- graded, and clean")


def test_an_intent_declaring_none_does_not_run_the_rule():
    """The `_wants` trap.

    That function ends in a bare `return True`, so a rule registered without
    an explicit branch runs on EVERY board and lands in `rules_run` having
    measured nothing -- the vacuous pass `--require-rules` exists to catch,
    arriving through the mechanism that implements it.
    """
    r = _graded(PLACED, rows=[])
    assert 'proximity' not in r.rules_run, r.rules_run
    why = r.rules_skipped['proximity']
    assert why == 'the intent declares no proximity claims', why
    print(f"  PASS: not declared -> not run, and the skip says why: {why!r}")


def test_both_bases_measure_the_same_pair_differently():
    """Which is why `basis` has no silent default.

    The same two parts are 3.14mm apart pad to pad and 0.32mm apart body to
    body. A default would have graded whichever one the tool preferred.
    """
    pad_edge = _graded(PLACED, rows=[
        {'ref': 'Y1', 'near': 'U1', 'max_mm': 0.1,
         'pads': {'Y1': ['1'], 'U1': ['9']}}])
    body = _graded(PLACED, rows=[
        {'ref': 'Y1', 'near': 'U1', 'max_mm': 0.1, 'basis': 'body'}])
    a = pad_edge.violations[0].measured
    b = body.violations[0].measured
    assert a['gap_mm'] == 3.1425 and a['basis'] == 'pad_edge', a
    assert b['gap_mm'] == 0.32 and b['basis'] == 'body', b
    # The rung that answered is on the wire, which is the whole reason the
    # basis is spelled `body` and `courtyard` is refused BY NAME: this board
    # draws no courtyard at all, so a number under that name would rest on fab
    # and silk without saying so.
    assert (b['basis_source'], b['near_basis_source']) == ('fab', 'silk'), b
    print(f"  PASS: same pair, {a['gap_mm']}mm pad-edge vs {b['gap_mm']}mm "
          f"body (from {b['basis_source']}/{b['near_basis_source']})")


def test_a_body_claim_for_parts_that_share_no_net_is_what_pad_edge_cannot_do():
    """Q1 and Q2 share no net, so there is no pad pair to measure at all."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(PLACED)
    nets = [{p.net_id for p in pcb.footprints[r].pads if p.net_id > 0}
            for r in ('Q1', 'Q2')]
    assert not (nets[0] & nets[1]), nets
    r = _graded(PLACED, rows=[{'ref': 'Q1', 'near': 'Q2', 'max_mm': 0.1,
                               'basis': 'body'}])
    assert r.violations[0].measured['gap_mm'] == 0.295, r.violations[0].measured
    print("  PASS: a pair sharing no net is measurable only body to body "
          "(0.295mm) -- the reason the second basis exists")


def test_a_name_the_board_does_not_have_is_a_finding_not_silence():
    """A typo must not grade clean -- `block_unresolved`'s failure one level
    over. Both spellings: a missing REF, and a pad number a real part lacks.
    """
    r = _graded(PLACED, rows=[{'ref': 'U99', 'near': 'U1', 'max_mm': 2.0}])
    v = [x for x in r.violations if x.rule == 'proximity_unresolved']
    assert len(v) == 1 and 'U99' in v[0].message, r.violations
    assert v[0].severity == fp.ERROR, v[0].severity

    # `Y1` has pads 1, 2, 3, 3 -- there is no pad 7.
    r = _graded(PLACED, rows=[{'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0,
                               'pads': {'Y1': ['7']}}])
    v = [x for x in r.violations if x.rule == 'proximity_unresolved']
    assert len(v) == 1 and "'7'" in v[0].message, r.violations
    assert v[0].measured['unresolved_ref'] == 'Y1', v[0].measured
    # ...and it does NOT also emit a distance measured from no pads at all.
    assert not [x for x in r.violations if x.rule == 'proximity']
    print("  PASS: a missing ref and a missing pad number are each one named "
          "finding, and neither produces a distance measured from nothing")


def test_a_duplicated_pad_number_is_minimised_over_not_first_hit():
    """`Y1` carries TWO pads numbered `3` -- a crystal's ground tabs.

    A first-hit lookup would answer from whichever the parser saw first, so
    the number would depend on file order. Taking every match and minimising
    cannot.
    """
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(PLACED)
    threes = [p for p in pcb.footprints['Y1'].pads if p.pad_number == '3']
    assert len(threes) == 2, threes
    u1_8 = [q for q in pcb.footprints['U1'].pads if q.pad_number == '8'][0]
    gaps = sorted(legality.rect_gap(legality.pad_rect(p),
                                    legality.pad_rect(u1_8)) for p in threes)
    assert gaps[0] != gaps[1], gaps
    r = _graded(PLACED, rows=[{'ref': 'Y1', 'near': 'U1', 'max_mm': 0.01,
                               'pads': {'Y1': ['3'], 'U1': ['8']}}])
    got = r.violations[0].measured['gap_mm']
    assert abs(got - gaps[0]) < 1e-6, (got, gaps)
    print(f"  PASS: two pads named '3' at {gaps[0]:.3f} and {gaps[1]:.3f}mm -- "
          f"the rule reports the minimum, not the first")


def test_the_rule_is_total_over_its_claims():
    """Every claim yields a pass, a violation, an unresolved, or an abstention.

    This is the promise `_ARM` was dropped on: a branch falling through
    without one of the four would put `proximity` in `rules_run` having graded
    a claim it never measured.
    """
    rows = [
        {'ref': 'Y1', 'near': 'U1', 'max_mm': 99.0},               # pass
        {'ref': 'C1', 'near': 'U2', 'max_mm': 0.01},               # violation
        {'ref': 'U99', 'near': 'U1', 'max_mm': 2.0},               # unresolved
        # A DIFFERENT pair for the second unresolved case: the loader refuses
        # two claims about one relation, and the first draft of this row
        # reused Y1~U1 and was refused -- the duplicate guard doing its job on
        # its own test.
        {'ref': 'C3', 'near': 'U2', 'max_mm': 2.0,
         'pads': {'C3': ['9']}},                                   # unresolved
    ]
    r = _graded(PLACED, rows=rows)
    kinds = {v.rule for v in r.violations}
    assert kinds == {'proximity', 'proximity_unresolved'}, kinds
    assert len([v for v in r.violations if v.rule == 'proximity']) == 1
    assert len([v for v in r.violations
                if v.rule == 'proximity_unresolved']) == 2
    print("  PASS: 4 claims -> 1 pass, 1 violation, 2 unresolved, "
          "0 unaccounted for")


def test_severity_is_settable_per_name():
    """A DNP-variant board must be able to demote the RESOLUTION finding
    without demoting the distance one -- which is why there are two names.
    """
    from kicad_parser import parse_kicad_pcb
    intent = fp.intent_from_dict({
        'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
        'severity': {'proximity_unresolved': fp.WARN},
        'proximity': [{'ref': 'U99', 'near': 'U1', 'max_mm': 2.0},
                      {'ref': 'C1', 'near': 'U2', 'max_mm': 0.01}]})
    r = fp.grade(intent, parse_kicad_pcb(PLACED), PLACED)
    by = {v.rule: v.severity for v in r.violations}
    assert by['proximity_unresolved'] == fp.WARN, by
    assert by['proximity'] == fp.ERROR, by
    print("  PASS: proximity_unresolved demoted to warn while proximity stays "
          "an error")


# --------------------------------------------------------------------------
# clause coverage: did a rule reach a verdict on every DECLARED clause?
# --------------------------------------------------------------------------


def _cov(rows=None, intent=None, **kw):
    """Compile the fixture brief and cover it against `intent`."""
    with open(FIXTURE_BRIEF, encoding='utf-8') as fh:
        raw = json.load(fh)
    if rows is not None:
        raw = dict(raw, proximity=rows)
    frag, rep = db.compile_brief(db.brief_from_dict(raw, 'e.design-brief.json'))
    if intent is None:
        # `.get`, because a brief whose every limit is "unknown"
        # compiles to NO proximity key at all -- which is the state
        # `test_unknown_and_carried_clauses_never_block` is about.
        intent = {'proximity': frag.get('proximity', [])}
    kw.setdefault('rules_run', ('proximity',))
    kw.setdefault('drifted_ids', db.drifted_clause_ids(intent, frag))
    return frag, rep, db.clause_coverage(rep, intent, **kw)


def test_every_clause_id_the_compiler_emits_parses_back():
    """ONE parser for a format built in seventeen places.

    A second implementation of a string format is how two halves drift, so
    this round-trips every id `compile_brief` produces. An id that stops
    parsing becomes a test failure rather than a clause that silently vanishes
    from the coverage report.
    """
    _frag, rep, _c = _cov()
    ids = list(rep['declared']) + list(rep['unknown']) + list(rep['not_graded'])
    assert ids, ids
    for cid in ids:
        rec = db.parse_clause_id(cid)
        assert rec is not None, cid
        assert rec['kind'] in ('interfaces', 'keepouts', 'proximity',
                               'product'), (cid, rec)
        if rec['kind'] == 'proximity':
            assert rec['near'] and rec['row'] is not None, (cid, rec)
            assert db.proximity_claim_id(rec['row'], rec['ref'],
                                         rec['near']) in cid, (cid, rec)
    # ...and a free-text `unknown[]` entry is NOT a clause and must not be
    # counted as one.
    assert db.parse_clause_id('mounting_datum') is None
    print(f"  PASS: {len(ids)} clause id(s) all parse back to their parts, and "
          f"a free-text unknown is not mistaken for one")


def test_a_declared_clause_the_intent_does_not_carry_is_UNCOVERED():
    """The run-25 shape, and the whole point of the key.

    Six rules ran, the grade passed, and not one clause the brief declared was
    measured -- because `rules_run` counts RULES and the count was satisfied
    by rules nobody had declared anything for.
    """
    _frag, _rep, cov = _cov(intent={})
    assert cov['uncovered'] == 4 and cov['graded'] == 0, cov
    assert cov['complete'] is False
    for row in cov['clauses']:
        if row['kind'] == 'proximity':
            assert row['state'] == 'uncovered', row
            assert 'no proximity claim' in row['why'], row
    print(f"  PASS: 4 declared clauses, 0 graded, complete=False")


def test_a_rule_that_did_not_run_leaves_its_clauses_uncovered():
    _frag, _rep, cov = _cov(rules_run=())
    assert cov['uncovered'] == 4, cov
    assert all('did not run' in c['why'] for c in cov['clauses']
               if c['kind'] == 'proximity'), cov['clauses']
    print("  PASS: the clause names the rule that did not run")


def test_an_abstention_is_its_own_state_not_a_pass_and_not_an_absence():
    """Three outcomes, not two.

    Folding an abstention into `graded` rebuilds the vacuous pass one key
    over; folding it into `uncovered` makes it unclearable when the board
    genuinely cannot answer. It gets its own state and carries the reason.
    """
    frag, _rep, _c = _cov()
    # The key format the RULE writes, derived from the intent row rather than
    # hardcoded: it carries the row INDEX because a reference may contain `~`,
    # so a coverage consumer resolves the index against the intent instead of
    # matching `ref~near` as a string.
    row_i = next(i for i, p in enumerate(frag['proximity'])
                 if p['ref'] == 'Q1')
    akey = f"proximity[{row_i}:Q1~Q2].basis"
    _f, _r, cov = _cov(abstained={akey: 'Q1 draws no body'})
    assert cov['abstained'] == 1 and cov['graded'] == 3, cov
    row = next(c for c in cov['clauses'] if c['state'] == 'abstained')
    assert row['why'] == 'Q1 draws no body', row
    assert row['ref'] == 'Q1', row
    assert cov['complete'] is False
    # A key naming a row that is not this clause must NOT be attributed to it.
    _f, _r, other = _cov(abstained={'proximity[0:Q1~Q2].basis': 'x'})
    assert other['abstained'] == 0, other
    print(f"  PASS: {row['id']} abstained carrying its reason verbatim, and a "
          f"key naming another row is not attributed to it")


def test_a_graded_clause_can_still_be_DRIFTED():
    """Coverage answers "did a rule look at this"; drift answers "was the
    thing graded the thing declared". A brief saying 2mm against an intent
    saying 9mm is fully graded -- against the wrong requirement.
    """
    with open(FIXTURE_BRIEF, encoding='utf-8') as fh:
        frag, rep = db.compile_brief(db.brief_from_dict(json.load(fh)))
    intent = {'proximity': [dict(p, max_mm=9.0) if p['ref'] == 'Y1' else p
                            for p in frag['proximity']]}
    cov = db.clause_coverage(rep, intent, rules_run=('proximity',),
                             drifted_ids=db.drifted_clause_ids(intent, frag))
    row = next(c for c in cov['clauses'] if c['ref'] == 'Y1')
    assert row['state'] == 'graded' and row['drifted'] is True, row
    assert cov['complete'] is False, cov
    print("  PASS: Y1's clause is graded AND drifted -- measured, against the "
          "wrong number")


def test_unknown_and_carried_clauses_never_block():
    """Declaring honestly must not be punished, or the channel teaches people
    to stop declaring. `mount_mode` is ungraded by design; a `"unknown"` limit
    is an author saying so.
    """
    _frag, _rep, cov = _cov(rows=[{'ref': 'Y1', 'near': 'U1',
                                   'max_mm': 'unknown'}])
    assert cov['not_claimed'] >= 1, cov
    assert cov['uncovered'] == 0 and cov['abstained'] == 0, cov
    assert cov['complete'] is True, cov
    assert cov['carried'] >= 1, cov
    print(f"  PASS: {cov['not_claimed']} unknown + {cov['carried']} carried "
          f"clause(s), complete=True -- the gate is clearable")


def test_the_cli_refuses_and_names_every_uncovered_clause():
    """The acceptance #902 asks for, through the real CLI."""
    import subprocess
    board = PLACED
    intent = os.path.join(ROOT, 'wk', 'cov_test_intent.json')
    os.makedirs(os.path.dirname(intent), exist_ok=True)
    tool = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
    emit = subprocess.run([sys.executable, '-X', 'utf8', tool, board,
                           '--no-brief', '--emit-intent', intent, '-q'],
                          capture_output=True, text=True)
    assert emit.returncode == 0, emit.stderr[-800:]
    r = subprocess.run([sys.executable, '-X', 'utf8', tool, board,
                        '--brief', FIXTURE_BRIEF, '--intent', intent,
                        '--require-brief-coverage'],
                       capture_output=True, text=True)
    assert r.returncode == 4, (r.returncode, r.stdout[-500:], r.stderr[-500:])
    assert 'require-brief-coverage' in r.stderr, r.stderr
    for ref, near in (('Y1', 'U1'), ('C1', 'U2'), ('C3', 'U2'), ('Q1', 'Q2')):
        assert f"{ref}~{near}" in r.stdout, (ref, near, r.stdout[-900:])
    # NOT the rule-count refusal: the right gate has to fire, or this passes
    # for a reason that has nothing to do with clauses.
    assert 'require-rules' not in r.stderr, r.stderr
    print("  PASS: the CLI exits 4 naming all four uncovered clauses, on the "
          "coverage gate rather than the rule count")


def test_a_board_with_no_brief_is_untouched():
    """The control. `counts`, the report line and the JSON must be what they
    were before this key existed, or every board on the corpus starts
    reporting zero of a thing nobody mentioned.
    """
    import subprocess
    tool = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
    intent = os.path.join(ROOT, 'wk', 'cov_nobrief_intent.json')
    os.makedirs(os.path.dirname(intent), exist_ok=True)
    board = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', tool, board, '--no-brief',
                    '--emit-intent', intent, '-q'], capture_output=True,
                   text=True, check=True)
    r = subprocess.run([sys.executable, '-X', 'utf8', tool, board,
                        '--no-brief', '--intent', intent],
                       capture_output=True, text=True)
    assert 'clause coverage' not in r.stdout, r.stdout[-500:]
    assert 'brief_clauses' not in r.stdout, r.stdout[-500:]
    print("  PASS: a board with no brief prints no coverage line and carries "
          "no coverage key")

# --------------------------------------------------------------------------
# the holes a second verifier found: branches nothing reached
# --------------------------------------------------------------------------

#: A board that DRAWS courtyards, which the esp_prog fixture does not (0 of
#: 21). Without it nothing here could see the difference between a courtyard
#: and a drawn body, which is a whole basis' worth of meaning.
COURTYARD_BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')


def test_the_body_basis_measures_the_DRAWN_body_not_the_courtyard():
    """#896 stores two ladders and says why: "a courtyard is not a body -- it
    is a body plus an assembly margin plus any shell overhang".

    `body_local` is courtyard-FIRST, so reading it made `basis: "body"`
    silently measure courtyards on every board that draws them, under-stating
    each gap by the assembly margin and PASSING claims that should fail --
    while `basis: "courtyard"` is refused by name on the grounds that boards
    do not draw them. The esp_prog fixture draws none, so no test on it could
    ever have seen this; it took a board that does.
    """
    from kicad_parser import parse_kicad_pcb
    from placement import body as body_mod
    pcb = parse_kicad_pcb(COURTYARD_BOARD)
    bodies = body_mod.board_bodies(pcb, COURTYARD_BOARD)
    pair = ('U1', 'Y1')
    assert all(bodies[r].source == 'courtyard' for r in pair), \
        {r: bodies[r].source for r in pair}
    assert all(bodies[r].drawn_source == 'fab' for r in pair), \
        {r: bodies[r].drawn_source for r in pair}

    def _rect(ref, local):
        fp_obj = pcb.footprints[ref]
        x0, y0, x1, y1 = legality.rotate_local_bounds(
            *local, fp_obj.rotation or 0.0)
        return (fp_obj.x + x0, fp_obj.y + y0, fp_obj.x + x1, fp_obj.y + y1)

    courtyard_gap = legality.rect_gap(
        _rect(pair[0], bodies[pair[0]].body_local),
        _rect(pair[1], bodies[pair[1]].body_local))
    drawn_gap = legality.rect_gap(
        _rect(pair[0], bodies[pair[0]].drawn_local),
        _rect(pair[1], bodies[pair[1]].drawn_local))
    assert drawn_gap > courtyard_gap + 0.5, (courtyard_gap, drawn_gap)

    intent = fp.intent_from_dict({
        'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
        'proximity': [{'ref': pair[0], 'near': pair[1], 'max_mm': 2.0,
                       'basis': 'body'}]})
    r = fp.grade(intent, pcb, COURTYARD_BOARD)
    got = r.violations[0].measured
    assert abs(got['gap_mm'] - drawn_gap) < 1e-6, (got, drawn_gap)
    assert got['basis_source'] == 'fab', got
    # ...and the courtyard reading would have PASSED this very claim.
    assert courtyard_gap <= 2.0 < drawn_gap, (courtyard_gap, drawn_gap)
    print(f"  PASS: courtyard-first reads {courtyard_gap:.3f}mm and would "
          f"PASS a 2.0mm claim; the drawn bodies are {drawn_gap:.3f}mm apart "
          f"and the rule reports that, sourced 'fab'")


def test_a_partially_wrong_pad_list_is_not_graded_on_the_survivors():
    """`pads: {'Y1': ['2', '7']}` used to grade CLEAN on pad 2 while '7'
    vanished.

    The claim then measured a strictly smaller subject set than it declared --
    the exact failure the loader cites when it refuses an integer pad number
    ("would match no pad, measure nothing, and grade clean"), and a
    falsification of this rule's own invariant, which is quantified over the
    pads the claim DECLARES.
    """
    r = _graded(PLACED, rows=[{'ref': 'Y1', 'near': 'U1', 'max_mm': 2.0,
                               'pads': {'Y1': ['2', '7']}}])
    unres = [v for v in r.violations if v.rule == 'proximity_unresolved']
    assert len(unres) == 1, r.violations
    assert unres[0].measured['pads'] == ['7'], unres[0].measured
    assert unres[0].measured['resolved_pads'] == 1, unres[0].measured
    # ...and NO distance was reported from the pad that did resolve.
    assert not [v for v in r.violations if v.rule == 'proximity'], r.violations

    # The partner side too.
    r = _graded(PLACED, rows=[{'ref': 'Y1', 'near': 'U1', 'max_mm': 0.01,
                               'pads': {'Y1': ['1'], 'U1': ['9', '99']}}])
    unres = [v for v in r.violations if v.rule == 'proximity_unresolved']
    assert len(unres) == 1 and unres[0].measured['unresolved_ref'] == 'U1'
    assert not [v for v in r.violations if v.rule == 'proximity'], r.violations
    print("  PASS: one bad name in a pad list stops the claim on either side, "
          "naming the pad that missed and how many resolved")


def test_the_minimum_over_partners_is_a_minimum():
    """The rule's stated INVARIANT, and nothing reached it.

    Every earlier case gave a subject pad exactly ONE candidate partner (net
    matching reduces the declared arity to one), so `min` and `max` were
    indistinguishable -- flipping the comparison survived every test. This
    gives one subject pad SEVERAL partners on the same net and pins the
    smallest.
    """
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(PLACED)
    # USB1 carries six pads numbered '0', all on GND -- six real candidates
    # for one subject pad, at six different distances.
    zeros = [p for p in pcb.footprints['USB1'].pads if p.pad_number == '0']
    assert len(zeros) == 6, len(zeros)
    u1_8 = [q for q in pcb.footprints['U1'].pads if q.pad_number == '8'][0]
    gaps = sorted(legality.rect_gap(legality.pad_rect(u1_8),
                                    legality.pad_rect(p)) for p in zeros)
    assert gaps[0] < gaps[-1] - 1.0, gaps

    r = _graded(PLACED, rows=[{'ref': 'U1', 'near': 'USB1', 'max_mm': 0.01,
                               'pads': {'U1': ['8'], 'USB1': ['0']}}])
    got = [v for v in r.violations if v.rule == 'proximity']
    assert len(got) == 1, got
    # Against the ROUNDED minimum: `measured['gap_mm']` is 4dp on the wire,
    # and comparing it to a raw float at 1e-6 fails for a reason that has
    # nothing to do with which end of the range the rule took.
    assert got[0].measured['gap_mm'] == round(gaps[0], 4), \
        (got[0].measured['gap_mm'], gaps)
    # ...and NOT the maximum, which is what the flipped comparison produces.
    assert got[0].measured['gap_mm'] != round(gaps[-1], 4), gaps
    assert got[0].measured['near_pads'] == 6, got[0].measured
    print(f"  PASS: one subject pad against six partners {gaps[0]:.3f}.."
          f"{gaps[-1]:.3f}mm -- the rule reports {gaps[0]:.3f}, the minimum")


def test_a_padless_part_abstains_rather_than_passing_silently():
    """The abstention branches, which nothing reached.

    Both could be deleted with the whole suite green, and a claim naming a
    padless part then yielded NOTHING while `proximity` stayed in
    `rules_run` -- the vacuous pass `_ARM` was dropped on.
    """
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(PLACED)
    padless = sorted(r for r, f in pcb.footprints.items() if not f.pads)
    assert padless, 'the fixture has no padless footprint to test with'
    ref = padless[0]

    r = _graded(PLACED, rows=[{'ref': ref, 'near': 'U1', 'max_mm': 2.0}])
    assert not r.violations, r.violations
    assert 'proximity' in r.rules_run, r.rules_run
    keys = [k for k in r.budget_abstained if k.startswith('proximity[')]
    assert len(keys) == 1, r.budget_abstained
    assert 'no pads at all' in r.budget_abstained[keys[0]], r.budget_abstained
    # An abstention makes the grade INCOMPLETE, which is what stops it being
    # read as a clean board.
    assert r.complete is False and r.passed is False
    print(f"  PASS: a claim naming padless {ref} abstains -- "
          f"{r.budget_abstained[keys[0]][:60]}...")


def test_a_body_claim_for_a_part_with_no_geometry_abstains():
    """The other abstention branch, on the body basis."""
    from kicad_parser import parse_kicad_pcb
    from placement import body as body_mod
    pcb = parse_kicad_pcb(PLACED)
    bodies = body_mod.board_bodies(pcb, PLACED)
    nogeom = sorted(r for r, g in bodies.items()
                    if g.drawn_local is None and g.body_local is None)
    if not nogeom:
        print("  PASS (VACUOUS): every part on this fixture has geometry; the "
              "source='none' branch has no subject here and is covered by the "
              "padless case above")
        return
    r = _graded(PLACED, rows=[{'ref': nogeom[0], 'near': 'U1', 'max_mm': 2.0,
                               'basis': 'body'}])
    keys = [k for k in r.budget_abstained if k.startswith('proximity[')]
    assert len(keys) == 1, r.budget_abstained
    assert 'draws no body' in r.budget_abstained[keys[0]]
    print(f"  PASS: a body claim naming {nogeom[0]} abstains rather than "
          f"passing")


def test_the_intent_loader_refuses_the_reversed_pair_the_brief_refuses():
    """The hand-written intent is the path the brief compiler never sees.

    Without this the two documents disagreed about the same rows: the brief
    refused them and the loader accepted them, and the grade then charged one
    symmetric measurement twice, reporting an identical number under two
    claims.
    """
    base = {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm'}
    try:
        fp.intent_from_dict(dict(base, proximity=[
            {'ref': 'Y1', 'near': 'U1', 'max_mm': 0.1},
            {'ref': 'U1', 'near': 'Y1', 'max_mm': 0.1}]))
        raise AssertionError('NOT REFUSED')
    except fp.IntentError as exc:
        assert 'same symmetric measurement' in str(exc), str(exc)
    # ...and the asymmetric form, with a subject pad list on each side, is
    # KEPT -- a guard that refuses everything is not a guard.
    i = fp.intent_from_dict(dict(base, proximity=[
        {'ref': 'Y1', 'near': 'U1', 'max_mm': 0.1, 'pads': {'Y1': ['1']}},
        {'ref': 'U1', 'near': 'Y1', 'max_mm': 0.1, 'pads': {'U1': ['9']}}]))
    assert len(i.proximity) == 2, i.proximity
    print("  PASS: the loader refuses the symmetric reversed pair and keeps "
          "the asymmetric one, exactly as the brief does")

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
    test_an_absent_partner_is_named_even_when_the_limit_is_unknown,
    test_drift_compares_at_the_effective_value_not_at_key_presence,
    test_the_basis_vocabulary_matches_the_intent_loader_or_says_it_cannot_yet,
    test_the_fixture_brief_compiles_to_the_claims_the_issue_names,
    test_the_acceptance_numbers_the_issue_names,
    test_the_same_brief_measures_the_board_the_run_started_from,
    test_a_claim_the_board_satisfies_is_a_measured_clean_not_a_skip,
    test_an_intent_declaring_none_does_not_run_the_rule,
    test_both_bases_measure_the_same_pair_differently,
    test_a_body_claim_for_parts_that_share_no_net_is_what_pad_edge_cannot_do,
    test_a_name_the_board_does_not_have_is_a_finding_not_silence,
    test_a_duplicated_pad_number_is_minimised_over_not_first_hit,
    test_the_rule_is_total_over_its_claims,
    test_severity_is_settable_per_name,
    test_every_clause_id_the_compiler_emits_parses_back,
    test_a_declared_clause_the_intent_does_not_carry_is_UNCOVERED,
    test_a_rule_that_did_not_run_leaves_its_clauses_uncovered,
    test_an_abstention_is_its_own_state_not_a_pass_and_not_an_absence,
    test_a_graded_clause_can_still_be_DRIFTED,
    test_unknown_and_carried_clauses_never_block,
    test_the_cli_refuses_and_names_every_uncovered_clause,
    test_a_board_with_no_brief_is_untouched,
    test_the_body_basis_measures_the_DRAWN_body_not_the_courtyard,
    test_a_partially_wrong_pad_list_is_not_graded_on_the_survivors,
    test_the_minimum_over_partners_is_a_minimum,
    test_a_padless_part_abstains_rather_than_passing_silently,
    test_a_body_claim_for_a_part_with_no_geometry_abstains,
    test_the_intent_loader_refuses_the_reversed_pair_the_brief_refuses,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
