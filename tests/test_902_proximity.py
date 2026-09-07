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
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
