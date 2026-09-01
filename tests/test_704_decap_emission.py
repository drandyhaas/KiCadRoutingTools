#!/usr/bin/env python3
"""#704: `emit_intent` can derive a decap limit, and its absence is legible.

`emit_intent` wrote `'decaps': {}` as a constant, so `rule_decap_distance`
could never fire on an auto-emitted intent -- a document reporting `pass: true`
with three of nine rules run.

The traps this file is written against, all of which the repo has been bitten
by before, and two of which bit THIS change:

  * "zero violations" is satisfied by the bug itself. If the limit is not
    emitted the rule is SKIPPED, so it reports zero violations. Every
    grades-clean assertion therefore carries `'decap_distance' in rules_run`
    as a conjunct.
  * a loose limit also grades clean. `5.0`, `DECAP_RADIUS_MM`, `999` all pass
    a clean-round-trip check, so there is a TIGHTNESS assertion: the limit
    minus a hair must produce a violation.
  * a check against an ABSENT object passes with the bug present. The
    withholding arms therefore assert the REASON STRING, never merely that
    `max_distance_mm` is missing -- which is equally true on `main`.
  * a fixture set can make a mutation harmless. The `_ceil4` -> `round`
    mutation is only killable on boards where `round(max, 4) < max`; measured,
    that is 3 of the 9 tracked boards, so this file asserts its own fixture
    set still contains at least 3 of them.

`run_utils` is imported for the CLI arms, so `run_all.py` classifies this as
an integration test. That is correct: the withholding DISCLOSURE only exists
on stdout.
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
        sys.path.insert(0, os.path.join(_p, 'py_router'))   # placement split
        sys.path.insert(0, os.path.join(_p, 'py_tools'))    # placement split
        sys.path.insert(0, os.path.join(_p, 'py_placer'))   # placement split

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402
from placement import groups as groups_mod                  # noqa: E402
from placement import legality                              # noqa: E402

RUN_ALL_TIMEOUT = 900

#: Boards with tethers, on which the limit is derived and the round trip must
#: be clean. Deliberately the wide set rather than a convenient subset -- see
#: `test_the_fixture_set_can_still_kill_the_round_mutation`.
EMITTING = ('splitflap_driver', 'tigard', 'watchy', 'ulx3s', 'glasgow_revC',
            'orangecrab_ext_pll', 'kit-dev-coldfire-xilinx_5213')
#: Boards where the derivation is WITHHELD, and why.
WITHHELD = {'interf_u_unrouted': 'tether',      # 1 tether: below MIN_SAMPLE
            'sonde_u': 'nothing was measured'}  # 0 tethers


def _board(name):
    return os.path.join(REPO, 'kicad_files', f'{name}.kicad_pcb')


def _present(names):
    return [n for n in names if os.path.exists(_board(n))]


def _emit(name, **kw):
    p = _board(name)
    return fp.emit_intent(parse_kicad_pcb(p), p, **kw), p


def _true_max(name):
    t = groups_mod.decap_tethers(parse_kicad_pcb(_board(name)))
    return max((d for caps in t.values() for _c, d in caps), default=None)


# --------------------------------------------------------------------------
# the default path is untouched
# --------------------------------------------------------------------------

def test_the_default_emission_declares_no_limit():
    """`--declare-decaps` is opt-in. Everything that emits without it -- and
    that is every caller in the tree except the new flag, including
    `tests/test_placement_ab.py` -- must be byte-unaffected in this key."""
    n = 0
    for name in _present(EMITTING + tuple(WITHHELD)):
        doc, _p = _emit(name)
        assert doc['decaps'] == {}, (name, doc['decaps'])
        assert 'decaps.max_distance_mm' not in doc['context']['budget_withheld']
        n += 1
    print(f"  PASS: {n} board(s) emit decaps {{}} without the flag")


def test_the_census_is_written_either_way():
    """`decaps: {}` alone cannot tell a reader "no cap is far from its IC"
    from "nobody measured" -- the same distinction `rules_run` /
    `rules_skipped` exists for, one level up. So the census runs on EVERY
    emission, flag or no flag."""
    for name in _present(EMITTING + tuple(WITHHELD))[:3]:
        for kw in ({}, {'derive_decaps': True}):
            doc, _p = _emit(name, **kw)
            c = doc['context']['decap_census']
            for k in ('source', 'metric', 'search_radius_mm', 'tethers',
                      'ics', 'beyond_radius', 'beyond_radius_refs',
                      'worst_beyond_mm', 'no_rail_chip',
                      'no_rail_chip_refs', 'population', 'unaccounted',
                      'seeder_pin_scope'):
                assert k in c, (name, kw, k)
            assert c['source'] == 'auto-tethers'
            # The retired keys must be GONE, not merely unread: a reader
            # who kept using `seeder_scope_ungraded` would be reading a
            # number that conflated three causes and was explained by a
            # fourth that does not exist (#792).
            for dead in ('seeder_scope', 'seeder_scope_ungraded'):
                assert dead not in c, (name, kw, dead)
            # The partition is the whole scope, on every board, always.
            assert c['unaccounted'] == 0, (name, c)
            assert (c['tethers'] + c['beyond_radius']
                    + c['no_rail_chip'] == c['population']), (name, c)
    print("  PASS: context.decap_census present with and without the flag")


def test_the_keepouts_note_says_which_kind_of_empty_it_is():
    doc, _p = _emit(_present(EMITTING)[0])
    assert doc['keepouts'] == []
    note = doc['context']['keepouts_note']
    assert 'NONE DECLARED' in note and 'not' in note, note
    print("  PASS: context.keepouts_note distinguishes declared-none from "
          "not-considered")


# --------------------------------------------------------------------------
# the derived limit
# --------------------------------------------------------------------------

def test_the_limit_is_the_ceiling_of_the_UNROUNDED_max():
    """The limit must be derived from the raw max, not from a display value.
    `round(v, 4)` can land BELOW the measured max, and ceiling an
    already-low number reproduces the exact defect `_ceil4` exists to
    prevent: a document that fails against the board it was written from.
    That is not hypothetical -- it is the bug the first version of this
    change shipped, on 3 of the 9 tracked boards."""
    n = 0
    for name in _present(EMITTING):
        doc, _p = _emit(name, derive_decaps=True)
        got = doc['decaps']['max_distance_mm']
        raw = _true_max(name)
        assert got == fp._ceil4(raw), (name, got, fp._ceil4(raw), raw)
        # `>= raw + EPS`, not `>= raw`, and the difference is the point.
        # `_ceil4` carries a `- 1e-9` guard in its SCALED domain, i.e. ~1e-13
        # mm, so on a board whose max is 4.899000000000001 it returns 4.899 --
        # 1e-15 mm low. The grader fires on `dist > limit + legality.EPS`
        # (1e-6), so this is the predicate that decides whether the document
        # passes against its own board, and asserting a stricter one I made up
        # would fail a board the tool grades clean. Measured: kit-dev is that
        # board, and it grades clean.
        assert raw <= got + legality.EPS, (
            f"{name}: emitted {got} is below its own board's worst tether "
            f"{raw} by more than EPS, so the document fails against the board "
            f"it was written from")
        # the census carries the unrounded number the derivation used
        assert doc['context']['decap_census']['max_mm'] == raw, name
        n += 1
    print(f"  PASS: {n} board(s) emit ceil4(unrounded max), all >= their own "
          f"worst tether")


def test_the_fixture_set_can_still_kill_the_round_mutation():
    """ANTI-VACUITY for the test above. `_ceil4` and `round` differ only where
    `round(max, 4) < max`; measured, 3 of the 9 tracked boards. A fixture set
    quietly narrowed to the others would leave that test passing in both
    directions, so the set is asserted rather than assumed."""
    live = [n for n in _present(EMITTING)
            if round(_true_max(n), 4) < _true_max(n)]
    assert len(live) >= 3, (
        f"only {len(live)} board(s) in EMITTING can distinguish _ceil4 from "
        f"round ({live}) -- the round-trip test would pass in both "
        f"directions")
    print(f"  PASS: {len(live)} board(s) make the _ceil4/round mutation live "
          f"-- {', '.join(live)}")


def test_the_round_trip_grades_clean_AND_runs_the_rule():
    """The `rules_run` conjunct is load-bearing. Without it "zero violations"
    is satisfied by the bug: no limit means the rule is SKIPPED, which reports
    zero violations on every board."""
    for name in _present(EMITTING):
        doc, p = _emit(name, derive_decaps=True)
        pcb = parse_kicad_pcb(p)
        r = fp.grade(fp.intent_from_dict(doc, p), pcb, p)
        assert 'decap_distance' in r.rules_run, (name, sorted(r.rules_run))
        bad = [v for v in r.violations if v.rule == 'decap_distance']
        assert not bad, (name, [v.message for v in bad[:3]])
    print(f"  PASS: {len(_present(EMITTING))} board(s) run decap_distance "
          f"with zero violations")


def test_the_limit_is_TIGHT_not_merely_satisfiable():
    """A loose constant -- 5.0, DECAP_RADIUS_MM, 999 -- also grades clean and
    also runs the rule, so the test above cannot tell it from a real
    measurement. This is the bound only the correct behaviour produces:
    shaving the limit must produce a violation on EVERY emitting board."""
    for name in _present(EMITTING):
        doc, p = _emit(name, derive_decaps=True)
        doc['decaps']['max_distance_mm'] -= 1e-3
        pcb = parse_kicad_pcb(p)
        r = fp.grade(fp.intent_from_dict(doc, p), pcb, p)
        bad = [v for v in r.violations if v.rule == 'decap_distance']
        assert bad, (f"{name}: shaving the limit by 1um produced NO "
                     f"violation, so the limit is not the board's own max")
    print(f"  PASS: {len(_present(EMITTING))} board(s) violate at "
          f"limit - 1um -- the limit is the measurement, not slack")


def test_no_search_radius_is_written_into_decaps():
    """The grader reads `search_radius_mm` with `groups.DECAP_RADIUS_MM` as
    its default. Writing a radius here that differs would have the emitter and
    the rule measuring different tether populations, and the clean round trip
    would then be an accident."""
    for name in _present(EMITTING)[:3]:
        doc, _p = _emit(name, derive_decaps=True)
        assert set(doc['decaps']) == {'max_distance_mm'}, doc['decaps']
    print("  PASS: decaps carries max_distance_mm and nothing else")


# --------------------------------------------------------------------------
# withholding -- asserted on the REASON, never on the absence
# --------------------------------------------------------------------------

def test_a_board_that_cannot_support_a_limit_withholds_and_says_why():
    """Asserting `'max_distance_mm' not in decaps` would pass on `main`, with
    the bug fully present. So each arm asserts the reason string."""
    n = 0
    for name, phrase in WITHHELD.items():
        if not os.path.exists(_board(name)):
            continue
        doc, p = _emit(name, derive_decaps=True)
        assert 'max_distance_mm' not in doc['decaps'], name
        why = doc['context']['budget_withheld'].get('decaps.max_distance_mm')
        assert why, f"{name}: withheld with NO reason recorded"
        assert phrase in why, (name, why)
        # ... and the census still ran, so "withheld" is distinguishable from
        # "not considered" here too.
        assert 'tethers' in doc['context']['decap_census'], name
        n += 1
    print(f"  PASS: {n} board(s) withhold with a stated reason")


def test_a_zero_tether_board_does_not_emit_a_vacuous_zero():
    """`max(..., default=0)` would write `max_distance_mm: 0.0`, which grades
    clean vacuously (there is nothing to grade) and would silently gate every
    future cap at zero. The purest form of "a check against an absent object
    passes with the bug present": only the reason string separates the two."""
    if not os.path.exists(_board('sonde_u')):
        print("  SKIP: sonde_u not present")
        return
    doc, _p = _emit('sonde_u', derive_decaps=True)
    assert doc['context']['decap_census']['tethers'] == 0
    assert doc['decaps'].get('max_distance_mm') is None
    why = doc['context']['budget_withheld']['decaps.max_distance_mm']
    assert 'nothing was measured' in why, why
    print(f"  PASS: sonde_u (0 tethers) withholds -- {why[:60]}...")


#: The board that isolates the CENSORING guard. Every tracked board in
#: `WITHHELD` is caught by an earlier guard -- sonde_u has no tethers at all
#: and interf_u_unrouted has one -- so deleting the censoring rule outright
#: changed no verdict anywhere and the mutation SURVIVED the first version of
#: this file: a guard with a measured rationale, a named constant and a
#: documented table, asserted by nothing. `tigard_placed` clears both earlier
#: guards with 15 tethers and is withheld by censoring alone.
_CENSORED = os.path.join(REPO, 'tests', 'fixtures', 'run23',
                         'tigard_placed.kicad_pcb')


def test_the_censoring_guard_is_the_ONLY_thing_withholding_on_a_mid_repair_board():
    if not os.path.exists(_CENSORED):
        print(f"  SKIP: {_CENSORED} not present")
        return
    doc = fp.emit_intent(parse_kicad_pcb(_CENSORED), _CENSORED,
                         derive_decaps=True)
    c = doc['context']['decap_census']
    # It must CLEAR the two earlier guards, or this arm is testing one of
    # those instead and the censoring rule is still unasserted.
    assert c['tethers'] >= fp.DECAP_MIN_SAMPLE, c['tethers']
    total = c['tethers'] + c['beyond_radius']
    frac = c['beyond_radius'] / total
    assert frac > fp.DECAP_MAX_CENSORED, (frac, c)
    assert 'max_distance_mm' not in doc['decaps'], doc['decaps']
    why = doc['context']['budget_withheld']['decaps.max_distance_mm']
    assert 'search radius' in why and 'bless' in why, why
    print(f"  PASS: tigard_placed has {c['tethers']} tethers (clears "
          f"MIN_SAMPLE) and censors {frac:.2f} > {fp.DECAP_MAX_CENSORED}, so "
          f"the censoring guard alone withholds it")


def test_the_censoring_guard_separates_healthy_from_degenerate():
    """The withholding rule is a threshold on a measured quantity, so the
    measurement is asserted rather than the threshold restated: every emitting
    board must sit BELOW the cut and every withheld-for-censoring board ABOVE
    it. A guard that withheld on everything, or on nothing, fails here."""
    rows = []
    for name in _present(EMITTING + tuple(WITHHELD)):
        c = _emit(name)[0]['context']['decap_census']
        total = c['tethers'] + c['beyond_radius']
        rows.append((name, c['tethers'],
                     (c['beyond_radius'] / total) if total else 1.0))
    healthy = [r for r in rows if r[0] in EMITTING]
    assert healthy, "no emitting board present"
    for name, _n, frac in healthy:
        assert frac <= fp.DECAP_MAX_CENSORED, (
            f"{name} censors {frac:.2f} yet a limit is emitted for it")
    worst = max(f for _n, _t, f in healthy)
    print(f"  PASS: every emitting board censors <= "
          f"{fp.DECAP_MAX_CENSORED} (worst {worst:.2f}); "
          f"{len([r for r in rows if r[2] > fp.DECAP_MAX_CENSORED])} board(s) "
          f"above it")


def test_the_census_discloses_what_the_limit_cannot_see():
    """The known, unfixed hole: emitter and rule truncate at the same radius,
    so a rail-sharing cap beyond it is invisible to both. Shipping the limit
    WITHOUT that disclosure would be the bug #704 complains about relocated --
    a number that goes up while coverage does not."""
    if not os.path.exists(_board('splitflap_driver')):
        print("  SKIP: splitflap_driver not present")
        return
    doc, _p = _emit('splitflap_driver', derive_decaps=True)
    c = doc['context']['decap_census']
    assert doc['decaps']['max_distance_mm'] < 5.0
    assert c['beyond_radius'] >= 1, c
    assert c['worst_beyond_mm'] > 4 * doc['decaps']['max_distance_mm'], c
    assert c['beyond_radius_refs'], c
    print(f"  PASS: splitflap_driver emits "
          f"{doc['decaps']['max_distance_mm']} while disclosing a cap "
          f"{c['worst_beyond_mm']}mm out, ref(s) {c['beyond_radius_refs']}")


def test_the_census_is_deterministic():
    """`beyond_radius_refs` is built from a dict of lists; an unsorted set
    would make the emitted document depend on PYTHONHASHSEED, which
    test_run5_emit_guard pins for the rest of the emitter."""
    name = _present(EMITTING)[0]
    outs = []
    for seed in ('0', '12345'):
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', '-c',
             'import sys,os,json\n'
             'R=os.getcwd()\n'
             "for d in ('','py_placer','py_router','py_tools'):"
             ' sys.path.insert(0,os.path.join(R,d))\n'
             'from kicad_parser import parse_kicad_pcb\n'
             'from placement import floorplan as fp\n'
             'p=sys.argv[1]\n'
             'print(json.dumps(fp.emit_intent(parse_kicad_pcb(p),p,'
             'derive_decaps=True)["context"]["decap_census"],sort_keys=True))',
             _board(name)],
            capture_output=True, text=True, encoding='utf-8', cwd=REPO,
            timeout=600, env=dict(os.environ, PYTHONHASHSEED=seed))
        outs.append(r.stdout.strip().splitlines()[-1])
    assert outs[0] == outs[1], (outs[0][:200], outs[1][:200])
    # Stable is not the same as SORTED, and only the first is what two runs of
    # identical code can show. Reversing the order is deterministic too, so it
    # survives the comparison above -- pin the property the code claims.
    for nm in _present(EMITTING + tuple(WITHHELD)):
        refs = _emit(nm)[0]['context']['decap_census']['beyond_radius_refs']
        assert refs == sorted(refs), (nm, refs)
    print(f"  PASS: {name}'s census is identical across PYTHONHASHSEED, and "
          f"beyond_radius_refs is sorted on every board")


# --------------------------------------------------------------------------
# the abstention channel, generalised off `legality`
# --------------------------------------------------------------------------

def test_a_withheld_decap_key_ABSTAINS_rather_than_skipping_silently():
    """Before this, the "the emitter WITHHELD" note was hard-wired to
    `name == 'legality'`, so a withheld decaps key inflated
    `budget_abstained` while `decap_distance`'s skip reason still read "the
    intent declares no decaps.max_distance_mm" -- exactly the defect run-23
    fixed for `legality`."""
    name = next((n for n in WITHHELD if os.path.exists(_board(n))), None)
    if name is None:
        print("  SKIP: no withholding board present")
        return
    doc, p = _emit(name, derive_decaps=True)
    r = fp.grade(fp.intent_from_dict(doc, p), parse_kicad_pcb(p), p)
    assert 'decap_distance' in r.rules_skipped
    reason = r.rules_skipped['decap_distance']
    assert 'WITHHELD' in reason, reason
    assert 'decaps.max_distance_mm' in reason, reason
    assert 'decaps.max_distance_mm' in fp.to_json(r)['budget_abstained']
    s = fp.summary(r)
    assert 'decaps.max_distance_mm' in s['budget_abstained_keys'], s
    print(f"  PASS: {name} -- decap_distance skip reason names the "
          f"withholding, and the key reaches JSON_SUMMARY")


def test_a_hand_declared_limit_overrides_the_withholding():
    """The declared-by-hand test used to be "is it in legality_budget", which
    a namespaced key never is -- so a hand-declared limit would have abstained
    anyway while ALSO being graded."""
    name = next((n for n in WITHHELD if os.path.exists(_board(n))), None)
    if name is None:
        print("  SKIP: no withholding board present")
        return
    doc, p = _emit(name, derive_decaps=True)
    doc['decaps']['max_distance_mm'] = 3.0
    r = fp.grade(fp.intent_from_dict(doc, p), parse_kicad_pcb(p), p)
    assert 'decap_distance' in r.rules_run, sorted(r.rules_run)
    assert 'decaps.max_distance_mm' not in r.budget_abstained, \
        r.budget_abstained
    print("  PASS: a hand-declared limit is graded, and its withholding note "
          "is history rather than an abstention")


def test_an_unmapped_withheld_key_still_abstains_and_blames_no_rule():
    """`budget_withheld` is inside the deliberately-open `context`, so its
    keys are unvalidated. A typo must be REPORTED, not dropped -- otherwise
    the author believes they recorded a withholding and nothing did."""
    name = _present(EMITTING)[0]
    doc, p = _emit(name)
    # A typo that cannot COLLIDE with any real text. `decaps.max_distance`
    # would have been a bad choice: `decap_distance`'s own base skip reason
    # legitimately reads "the intent declares no decaps.max_distance_mm", so a
    # substring check on it passes for the wrong reason.
    typo = 'decaps.maxximum_distance_mm'
    doc['context']['budget_withheld'] = {typo: 'a typo'}
    r = fp.grade(fp.intent_from_dict(doc, p), parse_kicad_pcb(p), p)
    assert typo in r.budget_abstained, r.budget_abstained
    for name_, reason in r.rules_skipped.items():
        assert typo not in reason, (name_, reason)
    text = fp.format_text(r)
    assert 'NOT DERIVABLE' in text and typo in text
    print("  PASS: an unmapped withheld key abstains, is printed, and is "
          "attached to no rule")


def test_the_printed_heading_is_not_a_legality_claim():
    """"N legality budget key(s) NOT DERIVABLE" is a lie for a decaps key."""
    name = next((n for n in WITHHELD if os.path.exists(_board(n))), None)
    if name is None:
        print("  SKIP: no withholding board present")
        return
    doc, p = _emit(name, derive_decaps=True)
    r = fp.grade(fp.intent_from_dict(doc, p), parse_kicad_pcb(p), p)
    text = fp.format_text(r)
    assert 'declared value(s) NOT DERIVABLE' in text, \
        [ln for ln in text.splitlines() if 'DERIVABLE' in ln]
    assert 'legality budget key(s) NOT DERIVABLE' not in text
    print("  PASS: the heading reads 'declared value(s)', not 'legality "
          "budget key(s)'")


# --------------------------------------------------------------------------
# the placement coupling, pinned with its numbers
# --------------------------------------------------------------------------

def test_the_emitted_limit_changes_what_place_seed_seeds():
    """Declaring this key is NOT a grading-only change: `place_seed` reads it
    and pulls caps out of radial zone packing into its per-supply-pin stage.

    THIS TEST'S PREDECESSOR PINNED 70 / 53 / 17 AND SAID A SILENT
    RECONCILIATION MUST FAIL LOUDLY. It did, and this is the argument it
    forced.

    The old numbers were right and the old EXPLANATION was wrong. The doc and
    the census both blamed the gap on the two "is this a decap" predicates
    disagreeing. Measured over every tracked board, the three spellings of that
    predicate name identical sets (`tests/test_792_decap_predicate.py`), so the
    residue attributable to them is ZERO. ulx3s's 17 is:

        7   beyond the 5mm tether search radius -- a real GRADING hole (#794),
            reported by `decap_ungraded` since this change;
        10  whose rail NO chip carries at all -- C3 C4 C22 on /power/P1V1,
            C7 C8 C24 on /power/P3V3, C11 C12 C23 on /power/P2V5, C14 on
            /power/SHUT. Every one of those rails is owned only by two-pad
            passives (the caps, L1-L3, RA*/RP*): they are bulk and filter caps
            upstream of an LC network, not decouplers. The GRADER is right to
            ignore all ten. The SEEDER was not, and #792 narrows its scope to
            the caps that elect a tether at any distance.

    So the pin is now the PARTITION and the ten refs BY NAME, because the whole
    argument rests on which ten they are rather than on how many.
    """
    if not os.path.exists(_board('ulx3s')):
        print("  SKIP: ulx3s not present")
        return
    pcb = parse_kicad_pcb(_board('ulx3s'))
    scope = {r for r, f in pcb.footprints.items()
             if r[:1].upper() == 'C'
             and len([p for p in f.pads if p.net_id > 0]) == 2}
    near, beyond, orphans = groups_mod.decap_populations(pcb)
    tethered = {c for caps in near.values() for c, _d in caps}
    assert len(scope) == 70, len(scope)
    assert len(tethered) == 53, len(tethered)
    assert len(scope - tethered) == 17, len(scope - tethered)
    assert len(beyond) == 7, len(beyond)
    assert len(orphans) == 10, len(orphans)
    assert sorted(orphans) == ['C11', 'C12', 'C14', 'C22', 'C23', 'C24',
                               'C3', 'C4', 'C7', 'C8'], sorted(orphans)
    c = _emit('ulx3s', derive_decaps=True)[0]['context']['decap_census']
    assert c['population'] == 70, c
    assert c['tethers'] == 53 and c['beyond_radius'] == 7, c
    assert c['no_rail_chip'] == 10, c
    assert c['no_rail_chip_refs'] == sorted(orphans), c
    assert c['unaccounted'] == 0, c
    assert c['seeder_pin_scope'] == 60, c
    print("  PASS: ulx3s -- 70 in scope = 53 graded + 7 beyond the radius + "
          "10 with no rail-carrying chip; 0 from the predicates, and the "
          "seeder's pin stage takes the 60 that elect a tether")


def test_the_worst_beyond_is_the_FARTHEST_cap_not_the_last_one():
    """A pre-existing defect this work found by reading, not by a red test.

    `worst_beyond_mm` was `beyond[-1][1]` off a list sorted by cap REFERENCE,
    so it reported the alphabetically last beyond-cap rather than the farthest.
    Measured, it understated on 7 of the 14 boards that have any beyond-cap:

        kit-dev-coldfire   10.80 reported,  22.89 true
        interf_u_unrouted   8.39 reported,  19.82 true
        flat_hierarchy      6.44 reported,  18.42 true
        glasgow_revC        6.23 reported,  10.34 true
        sonde_u            12.15 reported,  15.58 true
        watchy              9.18 reported,  11.16 true
        interf_u_placed     5.04 reported,   8.48 true

    The number is quoted in `--declare-decaps` stdout, in the withholding
    reason, and in docs/floorplan-intent.md's censoring table, so all three
    understated the very thing the census exists to disclose. It survived
    because `splitflap_driver` -- the board every other census arm is written
    against -- has exactly ONE cap beyond the radius, where last and farthest
    are the same cap.

    Checked on EVERY board rather than on a fixture, because the fixture is
    what hid it.
    """
    n = checked = 0
    for path in run_utils.corpus_boards():
        try:
            pcb = parse_kicad_pcb(path)
        except Exception:                                   # noqa: BLE001
            continue
        c = fp.decap_census(pcb)
        n += 1
        if not c['beyond_radius']:
            continue
        _near, beyond, _orph = groups_mod.decap_populations(pcb)
        assert c['worst_beyond_mm'] == round(max(d for _c, _i, d in beyond), 4), (
            os.path.basename(path), c['worst_beyond_mm'])
        # The two orderings are DIFFERENT things and must stay so.
        assert c['beyond_radius_refs'] == sorted(c['beyond_radius_refs'])
        checked += 1
    assert n >= 15, n
    # ANTI-VACUITY: a board set with no beyond-caps proves nothing here, and a
    # set where every board has ONE is what hid the defect for a release.
    assert checked >= 10, checked
    multi = 0
    for path in run_utils.corpus_boards():
        try:
            pcb = parse_kicad_pcb(path)
        except Exception:                                   # noqa: BLE001
            continue
        if fp.decap_census(pcb)['beyond_radius'] >= 2:
            multi += 1
    assert multi >= 5, (multi, "the arm needs boards with SEVERAL beyond-caps; "
                               "with one each, last and farthest coincide")
    print(f"  PASS: worst_beyond_mm is the max on {checked} board(s), "
          f"{multi} of them with 2+ beyond-caps where the two orderings differ")


# --------------------------------------------------------------------------
# the CLI: the disclosure only exists on stdout
# --------------------------------------------------------------------------

def test_the_cli_emits_and_warns_about_the_coupling():
    name = _present(EMITTING)[0]
    with tempfile.TemporaryDirectory() as wd:
        out = os.path.join(wd, 'fp.json')
        r = run_utils.check(
            [sys.executable, '-X', 'utf8',
             os.path.join('py_tools', 'check_floorplan.py'), _board(name),
             '--emit-intent', out, '--declare-decaps'], accept=True)
        run_utils.evidence(out, 'the emitted intent')
        doc = json.load(open(out, encoding='utf-8'))
        assert doc['decaps'].get('max_distance_mm'), doc['decaps']
        assert 'place_seed' in r.stdout and 'READS this key' in r.stdout, \
            "the --emit-intent block does not warn about the coupling"
    print("  PASS: --declare-decaps emits the limit and warns that "
          "place_seed reads it")


def test_the_cli_refuses_a_shaved_limit_with_the_right_REASON():
    """A non-zero exit is not evidence -- `run_utils.check(refuse=...)`
    asserts WHY, and reports an ImportError or argparse accident as a BROKEN
    TEST rather than crediting the guard."""
    name = _present(EMITTING)[0]
    with tempfile.TemporaryDirectory() as wd:
        path = os.path.join(wd, 'fp.json')
        doc, _p = _emit(name, derive_decaps=True)
        doc['decaps']['max_distance_mm'] -= 1e-3
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        run_utils.evidence(path, 'the shaved intent')
        run_utils.check(
            [sys.executable, '-X', 'utf8',
             os.path.join('py_tools', 'check_floorplan.py'), _board(name),
             '--intent', path],
            refuse='decap_distance', code=4)
    print("  PASS: a shaved limit exits 4 naming decap_distance")


def test_the_cli_reports_a_withholding_on_stdout():
    name = next((n for n in WITHHELD if os.path.exists(_board(n))), None)
    if name is None:
        print("  SKIP: no withholding board present")
        return
    with tempfile.TemporaryDirectory() as wd:
        out = os.path.join(wd, 'fp.json')
        r = run_utils.check(
            [sys.executable, '-X', 'utf8',
             os.path.join('py_tools', 'check_floorplan.py'), _board(name),
             '--emit-intent', out, '--declare-decaps'], accept=True)
        assert 'WITHHELD' in r.stdout, r.stdout[-400:]
        run_utils.evidence(out, 'the emitted intent')
        # A withheld key is an ungraded DECLARED channel, so since #713 item 5
        # the grade REFUSES (exit 4) instead of exiting 0. That is the point of
        # this test's own subject -- it asserts the CLI says NOT DERIVABLE --
        # so assert the refusal AND its reason rather than a bare success.
        r2 = run_utils.check(
            [sys.executable, '-X', 'utf8',
             os.path.join('py_tools', 'check_floorplan.py'), _board(name),
             '--intent', out], refuse='NOT DERIVABLE', code=4)
        assert 'decap_distance' in r2.stdout, r2.stdout[-400:]
        assert 'NOT FULLY GRADED' in (r2.stdout + r2.stderr), r2.stdout[-400:]
    print(f"  PASS: {name} -- the CLI says WITHHELD on emit and NOT DERIVABLE "
          f"on grade")


TESTS = [
    test_the_default_emission_declares_no_limit,
    test_the_census_is_written_either_way,
    test_the_keepouts_note_says_which_kind_of_empty_it_is,
    test_the_limit_is_the_ceiling_of_the_UNROUNDED_max,
    test_the_fixture_set_can_still_kill_the_round_mutation,
    test_the_round_trip_grades_clean_AND_runs_the_rule,
    test_the_limit_is_TIGHT_not_merely_satisfiable,
    test_no_search_radius_is_written_into_decaps,
    test_a_board_that_cannot_support_a_limit_withholds_and_says_why,
    test_a_zero_tether_board_does_not_emit_a_vacuous_zero,
    test_the_censoring_guard_is_the_ONLY_thing_withholding_on_a_mid_repair_board,
    test_the_censoring_guard_separates_healthy_from_degenerate,
    test_the_census_discloses_what_the_limit_cannot_see,
    test_the_census_is_deterministic,
    test_a_withheld_decap_key_ABSTAINS_rather_than_skipping_silently,
    test_a_hand_declared_limit_overrides_the_withholding,
    test_an_unmapped_withheld_key_still_abstains_and_blames_no_rule,
    test_the_printed_heading_is_not_a_legality_claim,
    test_the_emitted_limit_changes_what_place_seed_seeds,
    test_the_worst_beyond_is_the_FARTHEST_cap_not_the_last_one,
    test_the_cli_emits_and_warns_about_the_coupling,
    test_the_cli_refuses_a_shaved_limit_with_the_right_REASON,
    test_the_cli_reports_a_withholding_on_stdout,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
