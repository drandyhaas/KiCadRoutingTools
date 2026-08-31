"""The declared design brief: schema, compiler, and the three states (#711).

The board-independent half is deliberately the larger one -- `compile_brief` is
pure, so most of what matters here needs no `.kicad_pcb` at all, the same split
`validate_intent` has from `grade`.

What this file is written to hold down:

* THE REJECTIONS. A typo'd key that loads clean is a constraint the author
  believes they set and nothing ever checks. Every refusal is asserted by its
  REASON, never by "it raised something".

* THE THREE STATES. `declared`, `unknown` (the author looked) and `absent`
  (nobody looked) must stay distinguishable, because the failure this whole
  channel is designed against is a brief nobody writes -- and a half-written
  brief that reads like a complete one is that failure wearing a hat.

* DECLARED OUTRANKS INFERRED, and cannot invert. The board is the inference;
  the brief is the claim. A merge that lets the emitter win puts the toolchain
  back where it started.
"""
import copy
import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _d in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), _d))

import run_utils                                     # noqa: E402
from kicad_parser import parse_kicad_pcb             # noqa: E402
from placement import design_brief as db             # noqa: E402
from placement import floorplan as fp                # noqa: E402

RUN_ALL_TIMEOUT = 900

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(REPO, 'kicad_files')
FIXTURE = os.path.join(REPO, 'tests', 'fixtures', '711',
                       'esp_prog.design-brief.json')
BOARD = os.path.join(KF, 'esp_prog.kicad_pcb')


def _base(**over):
    d = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
         'product': {}, 'interfaces': []}
    d.update(over)
    return d


def _rejects(raw, why):
    """Assert a refusal AND return its message, so callers check the REASON."""
    try:
        db.brief_from_dict(raw)
    except db.BriefError as exc:
        return str(exc)
    raise AssertionError(f"NOT rejected: {why}")


def test_a_board_brief_document_is_refused_by_kind():
    """`board_brief.py --json` writes something that is also JSON and also
    calls itself a brief. Read as a design brief it is a pile of unknown keys,
    and the message would send the reader to fix spellings in a file that was
    never meant to be one."""
    msg = _rejects({'kind': 'board-brief', 'schema': 1},
                   "board_brief.py's own output")
    assert "board_brief.py's OUTPUT" in msg, msg
    assert 'design-brief' in msg, msg
    msg = _rejects({'kind': 'floorplan-intent', 'schema': 1}, "an intent")
    assert 'does not look like a design brief' in msg, msg
    print("  PASS: the near-miss gets its own answer, not an unknown-key pile")


def test_the_undeclarable_keys_are_refused_by_name_with_the_reason():
    """Not "unknown key" -- an author who wrote `height` believes it is being
    honoured, and nothing in this tree measures z."""
    msg = _rejects(_base(envelope={'rect': [0, 0, 10, 10]}), 'envelope')
    assert 'not declarable' in msg and 'READ from the board' in msg, msg
    msg = _rejects(_base(height={'max_mm': 3.0}), 'height')
    assert 'nothing in the placement stack measures height' in msg, msg
    assert 'would grade nothing' in msg, msg
    msg = _rejects(_base(outline={}), 'outline')
    assert 'not declarable' in msg, msg
    print("  PASS: envelope / outline / height refused by name, with why")


def test_every_malformed_shape_is_refused_by_its_reason():
    cases = [
        (_base(interfacess=[]), 'unknown key(s) interfacess'),
        (_base(schema=2), 'this build reads schema 1'),
        (_base(units='inch'), "units 'inch'"),
        (_base(min_reader=99), 'this build is reader 1'),
        (_base(product={'primary_axis': 'diagonal'}), 'expected one of'),
        (_base(interfaces=[{'reff': 'J1'}]), 'unknown key(s) reff'),
        (_base(interfaces=[{'ref': 'J1', 'edge': 'up'}]), "edge: 'up'"),
        (_base(interfaces=[{'ref': 'J1', 'user_facing': 'maybe'}]),
         'expected true, false'),
        (_base(interfaces=[{'ref': 'J1', 'along_edge': 'center'}]),
         'needs `along_edge_tolerance_mm`'),
        (_base(interfaces=[{'ref': 'J1',
                            'along_edge': {'from': 0.9, 'to': 0.1}}]),
         'is not less than to'),
        (_base(interfaces=[{'ref': 'J1',
                            'along_edge': {'from': 0.1, 'to': 1.5}}]),
         'expected [0.0, 1.0]'),
        (_base(interfaces=[{'ref': 'J1'}, {'ref': 'J1'}]),
         'duplicate interface'),
        (_base(keepouts=[{'name': 'k'}]), 'needs exactly one of'),
        (_base(keepouts=[{'name': 'k', 'rect': [0, 0, 1, 1],
                          'circle': [1, 1, 1]}]), 'needs exactly one of'),
        (_base(keepouts=[{'name': 'k', 'rect': [0, 0, 1, 1],
                          'sides': ['T']}]), "expected 'F' or 'B'"),
        (_base(fixed=[{'why': 'x'}]), 'expected an object with a `ref`'),
    ]
    for raw, want in cases:
        msg = _rejects(raw, want)
        assert want in msg, (want, msg)
    print(f"  PASS: {len(cases)} malformed shapes, each refused by its reason")


def test_a_duplicate_ref_is_refused_because_the_consumers_disagree():
    """Three consumers, three different answers, none of them an error.

    `rule_edge_connector` grades BOTH entries; `repair_placement`'s
    `{c['ref']: c ...}` keeps the LAST; and stage 1's even distribution shifts
    every OTHER connector on that edge. `blocks` already refuses a duplicate
    name for exactly this reason.
    """
    msg = _rejects(_base(interfaces=[{'ref': 'J1', 'edge': 'north'},
                                     {'ref': 'J1', 'edge': 'south'}]),
                   'two claims about one part')
    assert 'duplicate interface' in msg and 'interfaces[0]' in msg, msg
    print(f"  PASS: {msg[:90]}...")


def test_the_three_states_stay_apart():
    """declared / unknown / absent. The failure designed against is a brief
    nobody writes, so a half-written one must not read as a complete one."""
    brief = db.brief_from_dict(_base(
        product={'form_factor': 'usb_stick', 'primary_axis': db.UNKNOWN},
        interfaces=[{'ref': 'J1', 'edge': 'east', 'along_edge': db.UNKNOWN},
                    {'ref': 'J2', 'edge': db.UNKNOWN}],
        unknown=['mounting_datum']))
    _, rep = db.compile_brief(brief)
    assert 'product.form_factor' in rep['declared']
    assert 'interfaces[J1].edge' in rep['declared']
    assert 'product.primary_axis' in rep['unknown']
    assert 'interfaces[J1].along_edge' in rep['unknown']
    assert 'interfaces[J2].edge' in rep['unknown']
    assert 'mounting_datum' in rep['unknown']
    # `absent` is the third state: a tier-0 field nobody touched at all.
    assert rep['absent'] == [], rep['absent']
    empty = db.compile_brief(db.brief_from_dict(_base()))[1]
    assert empty['absent'] == ['product.form_factor', 'product.primary_axis',
                               'interfaces'], empty['absent']
    assert not empty['declared'] and not empty['unknown']
    # The three sets are disjoint, or they are not three states.
    assert not (set(rep['declared']) & set(rep['unknown']))
    print(f"  PASS: declared {len(rep['declared'])}, unknown "
          f"{len(rep['unknown'])}, absent {len(empty['absent'])} on an empty "
          f"brief -- disjoint")


def test_an_unknown_edge_never_becomes_an_edge_claim():
    """`edge: "unknown"` compiles to an entry with NO `edge` key.

    If it ever gained one, an inferred edge would be graded as a declared one
    -- the exact inversion this channel exists to prevent. And it must not
    reach `Intent.edge_claims()` as a claim the ENGINES act on.
    """
    frag, _ = db.compile_brief(db.brief_from_dict(_base(
        interfaces=[{'ref': 'J1', 'edge': db.UNKNOWN},
                    {'ref': 'J2', 'edge': 'north', 'user_facing': True}])))
    by_ref = {c['ref']: c for c in frag['edge_connectors']}
    assert 'edge' not in by_ref['J1'], by_ref['J1']
    assert by_ref['J1'].get('note') and 'unknown' in by_ref['J1']['note']
    assert by_ref['J2']['edge'] == 'north'
    # And through the real loader, into the real split.
    doc = _base(schema=fp.SCHEMA_VERSION, kind=fp.KIND)
    doc = {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
           'edge_connectors': frag['edge_connectors']}
    intent = fp.intent_from_dict(doc)
    claims = {c['ref'] for c in intent.edge_claims()}
    assert 'J2' in claims, claims
    assert all(c.get('edge') != db.UNKNOWN for c in intent.edge_connectors)
    print("  PASS: an unknown edge compiles to an edge-less entry that the "
          "loader accepts and no engine reads as an edge claim")


def test_the_fragment_loads_through_the_real_intent_loader():
    """Round trip. A compiler that emits something the loader refuses is a
    compiler that has invented a key."""
    brief = db.load_brief(FIXTURE)
    frag, rep = db.compile_brief(brief, board_refs=['USB1', 'CON1', 'CON2'])
    doc = {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
           'edge_connectors': frag['edge_connectors'],
           'keepouts': frag['keepouts'], 'min_reader': frag['min_reader']}
    intent = fp.intent_from_dict(doc)
    assert len(intent.edge_connectors) == 3
    assert frag['min_reader'] == 2, frag
    assert all(c['source'] == 'brief' for c in intent.edge_connectors)
    # No key outside the intent's own vocabulary.
    for c in intent.edge_connectors:
        assert not (set(c) - fp._EDGE_CONNECTOR_KEYS), set(c)
    for k in intent.keepouts:
        assert not (set(k) - fp._KEEPOUT_KEYS), set(k)
    print(f"  PASS: {len(intent.edge_connectors)} connector(s) + "
          f"{len(intent.keepouts)} keep-out(s) load, min_reader "
          f"{frag['min_reader']}, no key outside the intent vocabulary")


def test_min_reader_is_written_only_when_an_along_edge_field_is():
    plain, _ = db.compile_brief(db.brief_from_dict(_base(
        interfaces=[{'ref': 'J1', 'edge': 'north'}])))
    assert 'min_reader' not in plain, plain
    withpos, _ = db.compile_brief(db.brief_from_dict(_base(
        interfaces=[{'ref': 'J1', 'edge': 'north', 'along_edge': 'center',
                     'along_edge_tolerance_mm': 1.0}])))
    assert withpos['min_reader'] == 2, withpos
    print("  PASS: min_reader 2 only when a reader-2 field was written")


def test_declared_outranks_inferred_and_cannot_invert():
    """The board is the inference; the brief is the claim."""
    emitted = {
        'schema': 1, 'kind': fp.KIND, 'units': 'mm',
        'edge_connectors': [
            {'ref': 'J1', 'edge': 'north', 'source': 'auto-class',
             'class': 'connector_affinity',
             'overhang_mm': {'min': 0.0, 'max': 2.0},
             'observed_overhang_mm': 2.35, 'suspect': True,
             'suspect_reason': 'rigid pattern vectors exist'}],
        'context': {},
    }
    frag, rep = db.compile_brief(db.brief_from_dict(_base(
        interfaces=[{'ref': 'J1', 'edge': 'south', 'user_facing': True,
                     'overhang_mm': {'min': 0.0, 'max': 0.65}}])))
    out = db.merge_into_intent(copy.deepcopy(emitted), frag, rep)
    c = out['edge_connectors'][0]
    assert c['edge'] == 'south', c              # the brief wins
    assert c['overhang_mm']['max'] == 0.65, c
    assert c['source'] == 'brief', c
    assert c['context']['was_source'] == 'auto-class', c
    # The emitter's EVIDENCE survives: the suspect bit is a fact about the
    # BOARD, the brief is a claim about the SPEC, and both are true at once.
    assert c['suspect'] is True and c['observed_overhang_mm'] == 2.35, c
    # `user_facing: true` supplied the class outright.
    assert c['class'] == 'edge_receptacle', c
    assert rep['contradictions'] and 'north' in rep['contradictions'][0]

    # The other path: a brief that declares an EDGE but says nothing about
    # user_facing, over an emitter entry classed `connector_affinity`. That
    # class means "no edge claim" and `edge_claims()` DROPS it, so leaving it
    # in place would have the engines ignore the very declaration the author
    # wrote -- graded, unrepairable, and silent.
    frag2, rep2 = db.compile_brief(db.brief_from_dict(_base(
        interfaces=[{'ref': 'J1', 'edge': 'south'}])))
    assert 'class' not in frag2['edge_connectors'][0], frag2
    out2 = db.merge_into_intent(copy.deepcopy(emitted), frag2, rep2)
    c2 = out2['edge_connectors'][0]
    assert c2['class'] == 'edge_receptacle', c2
    assert c2['context']['was_class'] == 'connector_affinity', c2
    intent2 = fp.intent_from_dict(
        {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
         'edge_connectors': out2['edge_connectors']})
    assert {x['ref'] for x in intent2.edge_claims()} == {'J1'},         "a brief-declared edge did not survive edge_claims()"
    print(f"  PASS: brief edge wins, evidence survives, contradiction "
          f"reported, and a declared edge is promoted out of "
          f"connector_affinity so edge_claims() keeps it")


def test_a_ref_the_board_does_not_have_is_kept_and_reported():
    """Dropping it would make a typo'd ref grade clean, which is
    `block_unresolved`'s failure one level over."""
    frag, rep = db.compile_brief(
        db.brief_from_dict(_base(interfaces=[{'ref': 'J99', 'edge': 'north'}])),
        board_refs=['J1', 'U1'])
    assert rep['unmatched'] == ['J99'], rep
    assert rep['unmatched_checked'] is True
    assert [c['ref'] for c in frag['edge_connectors']] == ['J99']
    # With NO board, the check is skipped and SAID to be skipped -- an empty
    # ref list must not read as "every ref is wrong".
    _, rep2 = db.compile_brief(
        db.brief_from_dict(_base(interfaces=[{'ref': 'J99'}])))
    assert rep2['unmatched'] == [] and rep2['unmatched_checked'] is False
    print("  PASS: an absent ref is kept and named; with no board the check "
          "is skipped and says so")


def test_a_fixed_pose_never_becomes_must_lock():
    """Measured regression, recorded in emit_intent's own comment: filling
    `must_lock` made `place_seed --repair` treat those refs as seeder-owned
    and LIFT the user's locks."""
    frag, rep = db.compile_brief(db.brief_from_dict(_base(
        fixed=[{'ref': 'MH1', 'why': 'M2.5 boss'}])))
    assert 'must_lock' not in frag, frag
    assert rep['fixed'] == [{'ref': 'MH1', 'why': 'M2.5 boss'}]
    out = db.merge_into_intent({'context': {}}, frag, rep)
    assert 'must_lock' not in out, out
    assert out['context']['brief']['fixed'][0]['ref'] == 'MH1'
    print("  PASS: fixed[] is carried in context.brief, never must_lock")


def test_discovery_is_a_strict_no_op_when_there_is_no_sibling():
    assert db.discover_brief('') == ''
    assert db.discover_brief(BOARD) == ''
    assert db.brief_path_for('/a/b.kicad_pcb').endswith('.design-brief.json')
    with tempfile.TemporaryDirectory() as td:
        dst = os.path.join(td, 'b.kicad_pcb')
        shutil.copyfile(BOARD, dst)
        assert db.discover_brief(dst) == ''
        shutil.copyfile(FIXTURE, os.path.join(td, 'b.design-brief.json'))
        assert db.discover_brief(dst) == os.path.join(
            td, 'b.design-brief.json')
    print("  PASS: absent -> '', present -> the path, empty path -> ''")


def test_a_corrupt_sibling_is_a_reported_refusal_not_a_crash():
    with tempfile.TemporaryDirectory() as td:
        dst = os.path.join(td, 'b.kicad_pcb')
        shutil.copyfile(BOARD, dst)
        with open(os.path.join(td, 'b.design-brief.json'), 'w',
                  encoding='utf-8') as fh:
            fh.write('{ this is not json')
        run_utils.evidence(os.path.join(td, 'b.design-brief.json'),
                           'the corrupt brief')
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), dst,
                             '--emit-intent', os.path.join(td, 'out.json')],
                            refuse='cannot read design brief', code=2)
        assert 'Traceback' not in (r.stdout + r.stderr)
    print("  PASS: a corrupt sibling exits 2 naming the file, no traceback")


def test_the_cli_says_so_when_there_is_no_brief():
    """A SILENT absence is the failure this channel exists to fix."""
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'i.json')
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), BOARD,
                             '--emit-intent', out], accept=True)
        assert 'none beside this board' in r.stdout, r.stdout[:400]
        assert 'INFERRED' in r.stdout and '_nearest_edge' in r.stdout
        run_utils.evidence(out, 'the emitted intent')
    print("  PASS: the not-found branch names what is filling the gap")


def test_require_brief_refuses_when_none_was_declared():
    with tempfile.TemporaryDirectory() as td:
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('check_floorplan.py'), BOARD,
                         '--emit-intent', os.path.join(td, 'i.json'),
                         '--require-brief'],
                        refuse='--require-brief', code=4)
    print("  PASS: --require-brief exits 4 with its reason")


def test_the_sibling_reaches_the_grade_and_no_brief_reproduces_the_old_doc():
    """End to end, and the OFF arm.

    `--no-brief` must reproduce the document this tool wrote before the brief
    existed; without that there is no way to measure what declaring changed.
    """
    with tempfile.TemporaryDirectory() as td:
        dst = os.path.join(td, 'b.kicad_pcb')
        shutil.copyfile(BOARD, dst)
        shutil.copyfile(FIXTURE, os.path.join(td, 'b.design-brief.json'))
        run_utils.evidence(dst, 'the staged board')

        withb = os.path.join(td, 'with.json')
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), dst,
                             '--emit-intent', withb, '--declare-classes'],
                            accept=True)
        assert 'design brief b.design-brief.json' in r.stdout, r.stdout[:400]
        assert 'OUTRANK' in r.stdout, r.stdout[:600]
        doc = json.load(open(withb, encoding='utf-8'))
        by_ref = {c['ref']: c for c in doc['edge_connectors']}
        assert by_ref['USB1']['edge'] == 'east', by_ref['USB1']
        assert by_ref['USB1']['center_on_edge'] == {'tolerance_mm': 0.5}
        assert by_ref['CON2']['along_edge_band'] == {'from': 0.25, 'to': 0.75}
        assert len(doc['keepouts']) == 1, doc['keepouts']
        assert doc['min_reader'] == 2

        without = os.path.join(td, 'without.json')
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('check_floorplan.py'), dst,
                         '--emit-intent', without, '--declare-classes',
                         '--no-brief'], accept=True)
        ref = os.path.join(td, 'ref.json')
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('check_floorplan.py'), BOARD,
                         '--emit-intent', ref, '--declare-classes'],
                        accept=True)
        a = json.load(open(without, encoding='utf-8'))
        b = json.load(open(ref, encoding='utf-8'))
        a.pop('board'), b.pop('board')
        assert a == b, "--no-brief did not reproduce the pre-brief document"

        # And the declared document GRADES, with a rule that could not fire
        # on an inferred one.
        g = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), dst,
                             '--intent', withb], refuse='keepout', code=4)
        assert 'along-edge seating' in g.stdout, g.stdout[-800:]
    print("  PASS: the sibling reaches emit and grade; --no-brief reproduces "
          "the pre-brief document exactly")


def test_board_brief_reports_the_declaration_and_says_so_when_there_is_none():
    """#711's third ask: the channel reaches an instrument a placed board hits.

    `test_board_brief.py`'s own fixture passes `requirements='x'`, so its
    "every emitted section names its producer" check never sees this section.
    Asserted here instead, in both branches.
    """
    import board_brief as bb
    pcb = parse_kicad_pcb(BOARD)

    none = bb.build_brief(pcb, BOARD)
    assert 'design_brief' in none, sorted(none)
    assert none['design_brief']['path'] is None
    assert 'NONE DECLARED' in none['design_brief']['note']
    assert 'INFERENCE' in none['design_brief']['note']
    txt = bb.format_text(none)
    assert 'NONE DECLARED' in txt, txt[-400:]

    with_b = bb.build_brief(pcb, BOARD, design_brief=db.load_brief(FIXTURE),
                            design_brief_path=FIXTURE)
    d = with_b['design_brief']
    assert d['counts']['interfaces'] == 3 and d['counts']['keepouts'] == 1
    assert d['unknown'] and d['declared']
    txt = bb.format_text(with_b)
    # Unknowns FIRST: an unknown is the thing an author must go resolve.
    assert 'design brief UNKNOWN' in txt, txt[-600:]

    # Every emitted section must name its producer, with a `[requires:`
    # clause -- the rule test_board_brief enforces for the other sections.
    assert 'design_brief' in bb.SOURCES_NOTE
    assert '[requires:' in bb.SOURCES_NOTE['design_brief']
    print("  PASS: board_brief emits `design_brief` in both branches, sourced, "
          "unknowns first, and says NONE DECLARED when nothing is")


TESTS = [
    test_a_board_brief_document_is_refused_by_kind,
    test_the_undeclarable_keys_are_refused_by_name_with_the_reason,
    test_every_malformed_shape_is_refused_by_its_reason,
    test_a_duplicate_ref_is_refused_because_the_consumers_disagree,
    test_the_three_states_stay_apart,
    test_an_unknown_edge_never_becomes_an_edge_claim,
    test_the_fragment_loads_through_the_real_intent_loader,
    test_min_reader_is_written_only_when_an_along_edge_field_is,
    test_declared_outranks_inferred_and_cannot_invert,
    test_a_ref_the_board_does_not_have_is_kept_and_reported,
    test_a_fixed_pose_never_becomes_must_lock,
    test_discovery_is_a_strict_no_op_when_there_is_no_sibling,
    test_a_corrupt_sibling_is_a_reported_refusal_not_a_crash,
    test_the_cli_says_so_when_there_is_no_brief,
    test_require_brief_refuses_when_none_was_declared,
    test_the_sibling_reaches_the_grade_and_no_brief_reproduces_the_old_doc,
    test_board_brief_reports_the_declaration_and_says_so_when_there_is_none,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
