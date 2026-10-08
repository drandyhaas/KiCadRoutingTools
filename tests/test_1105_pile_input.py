#!/usr/bin/env python3
"""#1105: test_placement_ab's pile input mode measures a PILE, not a board.

The pile rows (and the pilot, tests/measure_1105_pile_variants.py) run on the
basis `tests/1105_pile_ab_prereg.json` fixed: the corpus board staged as an
unaided pile, one intent the CLI emits with `--decaps-from` the board itself,
and place_seed's own seed scope. Each case pins one way that could silently
become the corpus basis again:

* the staged board reads as unplaced, the CLI intent carries the decap limit
  and withholds the pile's pose claims, and the mechanical refs reach the
  intent as fixed poses (only the CLI compiles mechanical.json);
* `stage()` arms the unaided provenance regime over pile/ ONLY: the arms,
  written beside it, are outside every regime;
* the seed scope is place_seed's without --force: the stacked suspects;
* a board that is NOT a pile is refused with an AssertionError that is not
  `PileIneligible` -- a placed board measured under a pile's name is a broken
  measurement, not an ineligible board;
* a pile row that also states seed_intents is refused before anything is
  seated.

    python3 tests/test_1105_pile_input.py [name-substring ...]
"""
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

import test_placement_ab as AB                       # noqa: E402

RUN_ALL_TIMEOUT = 600
ESP = os.path.join(AB.BOARDS, 'esp_prog.kicad_pcb')


def test_the_pile_basis_is_a_pile():
    from kicad_parser import parse_kicad_pcb
    from placement import provenance
    from placement.placement_state import assess_placement
    with tempfile.TemporaryDirectory() as td:
        pile, intent, doc, refs = AB._pile_inputs(ESP, td)
        assert os.path.dirname(pile) == os.path.join(td, 'pile'), pile
        st = assess_placement(parse_kicad_pcb(pile), pile)
        assert st.partially_unplaced or st.unplaced, st.reasons
        assert doc['context']['pose_claims_withheld'], doc['context']
        assert doc['decaps']['max_distance_mm'] is not None, doc['decaps']
        assert doc['context']['decap_census']['reference_board'] == ESP
        # mechanical.json (USB1 and the two Ref* markers on esp_prog) reached
        # the intent as fixed poses -- the CLI's job, not emit_intent's
        fixed = {str(f['ref']) for f in intent.fixed_poses}
        assert 'USB1' in fixed, fixed
        # place_seed's scope without --force: the pile, not the mechanical
        assert refs is not None and 'USB1' not in refs and 'U1' in refs, refs
        assert refs == set(st.stacked_suspect_refs), (refs, st)
        # the regime covers pile/ and nothing the arms write
        assert provenance.regime_for(pile) == os.path.join(td, 'pile')
        for arm in ('off', 'on'):
            assert provenance.regime_for(
                os.path.join(td, f'{arm}.kicad_pcb')) is None, arm
        f = AB._pile_forecast(doc)
        assert f.get('scope', 0) >= 1, f
    print(f"  PASS: esp_prog pile -- {len(refs)} seeded, fixed "
          f"{sorted(fixed)}, limit {doc['decaps']['max_distance_mm']}, "
          f"forecast scope {f['scope']}, regime on pile/ only")


def test_a_placed_board_is_refused_not_ineligible():
    """Stage a 'pile' that is the placed board itself: the helper must raise,
    and must not raise the eligibility exception, which a caller treats as a
    legitimate pinned-neutral outcome."""
    real = None
    stress = os.path.join(ROOT, 'tests', 'stress')
    if stress not in sys.path:
        sys.path.insert(0, stress)
    import stage_unaided
    real = stage_unaided.stage

    def fake(src, out_board, *a, **k):
        shutil.copy2(src, out_board)
        return {}
    stage_unaided.stage = fake
    try:
        with tempfile.TemporaryDirectory() as td:
            try:
                AB._pile_inputs(ESP, td)
            except AB.PileIneligible as exc:
                raise AssertionError(f"a placed board read as INELIGIBLE: "
                                     f"{exc}")
            except AssertionError as exc:
                assert 'does not read it as unplaced' in str(exc), exc
            else:
                raise AssertionError("a placed board was accepted as a pile")
    finally:
        stage_unaided.stage = real
    print("  PASS: a placed board is refused as a broken measurement")


def test_the_pile_row_seeds_the_pile_in_both_arms():
    """run_row's pile branch hands BOTH arms the pile (not the corpus
    board), the ONE pile intent for seeding and grading, and place_seed's
    seed scope; the arms differ only in the row's own seed kwargs. The seed
    itself is replaced by a recorder, so this checks the plumbing, not a
    placement."""
    calls = []

    def rec(board_path, out_path, intent, seed_kw, group_sources=None,
            ignore_nets=(), grade_intent=None, engine_flags=None):
        calls.append({'board': board_path, 'out': out_path, 'intent': intent,
                      'kw': dict(seed_kw), 'grade': grade_intent,
                      'ignore': list(ignore_nets), 'flags': engine_flags})
        return {'seconds': 0.0, 'crossings': 0, 'hpwl': 0.0, 'inversions': 0,
                'unseated': 0, 'body_blocking': 0, 'intent_errors': 0}
    row = {'name': 'pile-plumbing', 'board': 'esp_prog.kicad_pcb',
           'corridors': [], 'engine': 'seed', 'input': 'pile',
           'seed_off': {'decap_claim_after_ics': False},
           'seed_on': {'decap_claim_after_ics': True},
           'ignore_nets': ['GND'], 'signal': 'intent_errors',
           'guard': ('crossings',)}
    real = AB._run_seed
    AB._run_seed = rec
    try:
        with tempfile.TemporaryDirectory() as td:
            AB.run_row(row, td)
            pile = os.path.join(td, 'pile-plumbing', 'pile',
                                'esp_prog.kicad_pcb')
    finally:
        AB._run_seed = real
    assert len(calls) == 2, calls
    off, on = calls
    for c in (off, on):
        assert c['board'] == pile, c['board']
        assert c['intent'] is off['intent'] and c['grade'] is off['intent']
        assert c['kw'].get('seed_refs') and 'U1' in c['kw']['seed_refs'], c
        assert c['ignore'] == ['GND'], c
    assert off['kw']['seed_refs'] == on['kw']['seed_refs']
    assert off['kw']['decap_claim_after_ics'] is False
    assert on['kw']['decap_claim_after_ics'] is True
    assert {k for k in off['kw'] if off['kw'][k] != on['kw'].get(k)} == {
        'decap_claim_after_ics'}, (off['kw'], on['kw'])
    print("  PASS: both arms seed the pile from the one pile intent, with "
          "place_seed's scope; they differ only in the row's seed kwarg")


def test_an_intent_without_the_limit_is_ineligible():
    """The emitted intent arms no decaps.max_distance_mm (simulated by
    stripping it from the CLI's output): `PileIneligible`, the
    pre-registration's pinned-neutral outcome, not a skip."""
    import json
    import subprocess
    real = subprocess.run

    def strip(argv, *a, **k):
        r = real(argv, *a, **k)
        if '--emit-intent' in argv:
            path = argv[argv.index('--emit-intent') + 1]
            with open(path, encoding='utf-8') as fh:
                doc = json.load(fh)
            (doc.get('decaps') or {}).pop('max_distance_mm', None)
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh)
        return r
    subprocess.run = strip
    try:
        with tempfile.TemporaryDirectory() as td:
            try:
                AB._pile_inputs(ESP, td)
            except AB.PileIneligible as exc:
                assert 'armed no decaps.max_distance_mm' in str(exc), exc
            else:
                raise AssertionError("an intent with no limit was eligible")
    finally:
        subprocess.run = real
    print("  PASS: an intent with no decap limit is PileIneligible")


def test_a_stage_the_emitter_does_not_call_a_pile_is_refused():
    """A staged board `assess_placement` calls partially unplaced but whose
    emitted intent withholds nothing (one passive moved onto another -- a
    stack, not a heap): a broken measurement, not an ineligible board. This
    is the second refusal in `_pile_inputs`; the first one cannot see it."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    stress = os.path.join(ROOT, 'tests', 'stress')
    if stress not in sys.path:
        sys.path.insert(0, stress)
    import stage_unaided
    real = stage_unaided.stage
    pcb = parse_kicad_pcb(ESP)
    two = sorted(r for r, fp in pcb.footprints.items()
                 if r.startswith('R') and fp.layer == 'F.Cu')[:2]
    a = pcb.footprints[two[0]]

    def fake(src, out_board, *x, **k):
        write_placed_output(src, out_board, [{
            'reference': two[1], 'new_x': a.x, 'new_y': a.y,
            'new_rotation': a.rotation or 0.0}])
        return {}
    stage_unaided.stage = fake
    try:
        with tempfile.TemporaryDirectory() as td:
            try:
                AB._pile_inputs(ESP, td)
            except AB.PileIneligible as exc:
                raise AssertionError(f"a stacked board read as INELIGIBLE: "
                                     f"{exc}")
            except AssertionError as exc:
                assert 'did not treat' in str(exc), exc
            else:
                raise AssertionError("a stacked board was accepted as a pile")
    finally:
        stage_unaided.stage = real
    print(f"  PASS: {two[1]} stacked on {two[0]} is refused as not a pile")


def test_a_pile_row_reads_no_seed_intents():
    row = {'name': 'pile-ctl', 'board': 'esp_prog.kicad_pcb', 'corridors': [],
           'engine': 'seed', 'input': 'pile',
           'seed_intents': {'off': 'off', 'on': 'auto', 'grade': 'auto'},
           'seed_off': {'decap_claim_after_ics': False},
           'seed_on': {'decap_claim_after_ics': True},
           'signal': 'intent_errors', 'guard': ('crossings',)}
    with tempfile.TemporaryDirectory() as td:
        try:
            AB.run_row(row, td)
        except AssertionError as exc:
            assert 'seed_intents is not read' in str(exc), exc
        else:
            raise AssertionError("a pile row with seed_intents ran")
        assert not os.path.isdir(os.path.join(td, 'pile-ctl', 'pile')), (
            "the refusal came after the pile was staged")
    print("  PASS: a pile row stating seed_intents is refused before staging")


TESTS = [
    test_the_pile_basis_is_a_pile,
    test_a_placed_board_is_refused_not_ineligible,
    test_the_pile_row_seeds_the_pile_in_both_arms,
    test_an_intent_without_the_limit_is_ineligible,
    test_a_stage_the_emitter_does_not_call_a_pile_is_refused,
    test_a_pile_row_reads_no_seed_intents,
]


if __name__ == '__main__':
    want = sys.argv[1:]
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
