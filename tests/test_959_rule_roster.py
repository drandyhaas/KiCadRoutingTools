#!/usr/bin/env python3
"""#959 / #997: the rule roster, written dispositions, and P1's refusal.

Run 29 graded its zone plan 22 times with 6 of 14 rules never running. Every
run printed each rule's skip reason -- "the intent declares no ..." -- and
nothing required anyone to act on it. The roster makes that a decision: a
rule that applies to this board, fails the grade when it fires, and that the
plan neither arms nor excuses is refused at P1, by name, with the key that
arms it.

Measured before this was built (#959 Phase 0, P4): refusing EVERY dark ERROR
rule refused 22 of 22 corpus boards with 90 dispositions, most of them the
same two boilerplate answers (`proximity` and `zone_exclusive`, dark on every
emitted intent because the emitter never writes them). So the roster refuses
only when a BOARD FACT says the rule applies and the rule is gating, and the
tests below pin both halves: what is refused, and what must not be.

Traps written against, from this repo's own history:

  * a non-zero exit is not evidence -- the CLI arms assert the REASON with
    `run_utils.check(refuse=...)`;
  * a table that mirrors a function drifts from it -- `_ARMING_KEY` is pinned
    against `_wants` rule by rule, and `_RULE_DEFAULT_SEVERITY` against the
    severity each rule actually EMITS;
  * an escape hatch nobody meant to build -- demoting a dark rule's severity
    must not make its refusal go away (a dark rule never runs, so the
    demotion would change nothing but the refusal).
"""
import json
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(REPO, _sub) if _sub else REPO
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402

RUN_ALL_TIMEOUT = 900

ESP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
BRIEF_711 = os.path.join(REPO, 'tests', 'fixtures', '711',
                         'esp_prog.design-brief.json')
FIX = os.path.join(REPO, 'tests', 'fixtures', '959')
PILE = os.path.join(FIX, 'run29_pile.kicad_pcb')
DRIVER = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement',
                      'scripts', 'placement_driver.py')

#: The smallest raw intent fragment that ARMS each rule. The test asserts the
#: fragment arms `_wants` and that `_ARMING_KEY` names its first segment, so a
#: change to what arms a rule fails here rather than leaving the refusal
#: telling authors to declare a key that no longer does anything.
ARM_WITH = {
    'envelope': {'envelope': {'rect': [0, 0, 10, 10]}},
    'zone_containment': {'blocks': [{'name': 'a', 'refs': ['U1'],
                                     'zone': [0, 0, 5, 5]}]},
    'zone_side': {'blocks': [{'name': 'a', 'refs': ['U1'], 'side': 'F'}]},
    'assembly_side': {'assembly': {'sides': 'F'}},
    'zone_exclusive': {'blocks': [{'name': 'a', 'refs': ['U1'],
                                   'zone': [0, 0, 5, 5], 'exclusive': True}]},
    'keepout': {'keepouts': [{'name': 'k', 'rect': [0, 0, 1, 1]}]},
    'edge_connector': {'edge_connectors': [{'ref': 'J1', 'edge': 'west'}]},
    'decap_distance': {'decaps': {'max_distance_mm': 2.0}},
    'decap_ungraded': {'decaps': {'max_distance_mm': 2.0}},
    'decap_pin_distance': {'decaps': {'max_pin_distance_mm': 1.0}},
    'proximity': {'proximity': [{'ref': 'C1', 'near': 'U1',
                                 'max_mm': 2.0}]},
    'must_lock': {'must_lock': ['U1']},
    'legality': {'legality_budget': {'oob_count': 0}},
    'pins_to_edge': {'edge_connectors': [{'ref': 'J1', 'edge': 'west'}]},
}


def _raw(**extra):
    d = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'}
    d.update(extra)
    return d


def _esp_roster(raw, pcb=None, **kw):
    pcb = pcb or parse_kicad_pcb(ESP)
    it = fp.intent_from_dict(raw, '')
    return {r['rule']: r for r in fp.rule_roster(it, pcb, ESP, **kw)}


def test_every_rule_has_its_table_entries():
    names = {n for n, _ in fp.RULES}
    assert set(fp._ARMING_KEY) == names, sorted(set(fp._ARMING_KEY) ^ names)
    assert set(fp._RULE_DEFAULT_SEVERITY) == names, sorted(
        set(fp._RULE_DEFAULT_SEVERITY) ^ names)
    assert set(ARM_WITH) == names, sorted(set(ARM_WITH) ^ names)
    assert set(fp._POLICY_RULES) <= names
    print(f"  PASS: {len(names)} rules, every one has an arming key and a "
          f"default severity")


def test_the_arming_key_is_what_arms_the_rule():
    """Each rule's `_ARMING_KEY` names the key whose presence flips `_wants`
    -- checked by DECLARING it, not by reading the table."""
    base = fp.intent_from_dict(_raw(), '')
    for name, frag in sorted(ARM_WITH.items()):
        assert not fp._wants(base, name), name
        it = fp.intent_from_dict(_raw(**frag), '')
        assert fp._wants(it, name), (name, frag)
        top = next(iter(frag))
        assert fp._ARMING_KEY[name].split('.')[0].split('[')[0] == top, (
            name, fp._ARMING_KEY[name], top)
    print(f"  PASS: {len(ARM_WITH)} arming keys each arm their rule")


def _severities_emitted(result):
    out = {}
    for v in result.violations:
        out.setdefault(v.rule, set()).add(v.severity)
    return out


def test_the_default_severity_table_is_what_the_rules_emit():
    """Fire the rules on esp_prog with an intent built to violate them, and
    compare what each EMITS against `_RULE_DEFAULT_SEVERITY`. The table is
    what the roster decides "gating" from, so it must not drift from the
    rules -- and a rule that silently moved to WARN would make its refusal
    vanish."""
    pcb = parse_kicad_pcb(ESP)
    u1 = pcb.footprints['U1']
    raw = _raw(
        envelope={'rect': [0.0, 0.0, 1.0, 1.0]},
        blocks=[{'name': 'far', 'refs': ['U1'], 'zone': [0, 0, 1, 1],
                 'note': 'U1 nowhere near here'},
                {'name': 'back', 'refs': ['R1'], 'side': 'B',
                 'note': 'R1 is on the front'},
                {'name': 'mine', 'refs': ['R2'],
                 'zone': [u1.x - 20, u1.y - 20, u1.x + 20, u1.y + 20],
                 'exclusive': True, 'note': 'reserved around U1'}],
        assembly={'sides': 'B'},
        keepouts=[{'name': 'k', 'rect': [u1.x - 1, u1.y - 1,
                                         u1.x + 1, u1.y + 1]}],
        edge_connectors=[{'ref': 'USB1', 'edge': 'north'}],
        decaps={'max_distance_mm': 0.01},
        proximity=[{'ref': 'Y1', 'near': 'U1', 'max_mm': 0.01}],
    )
    it = fp.intent_from_dict(raw, '')
    res = fp.grade(it, pcb, ESP)
    got = _severities_emitted(res)
    checked = []
    for name, _fn in fp.RULES:
        if name not in got:
            continue
        want = fp._RULE_DEFAULT_SEVERITY[name]
        # edge_connector files a connector_affinity arm at forced WARN; the
        # rule's own findings are at its default.
        if name == 'edge_connector':
            assert want in got[name], (name, got[name])
        else:
            assert got[name] == {want}, (name, got[name], want)
        checked.append(name)
    for must in ('envelope', 'zone_containment', 'zone_side',
                 'assembly_side', 'zone_exclusive', 'keepout',
                 'edge_connector', 'decap_distance', 'decap_ungraded',
                 'proximity', 'pins_to_edge'):
        assert must in checked, (must, sorted(got))
    print(f"  PASS: {len(checked)} rules fired, each at its table severity: "
          f"{', '.join(sorted(checked))}")


def test_esp_prog_with_its_brief_owes_exactly_two():
    """The emitted esp_prog intent with fixture 711 merged: zone_containment
    (no zones) and decap_distance (4 caps share a rail with an IC) are owed.
    proximity and zone_exclusive are dark but POLICY; decap_pin_distance is
    dark but ADVISORY here, because every esp_prog supply pin is found only
    through the rail-net channel, which files at WARN."""
    from placement import design_brief as db
    pcb = parse_kicad_pcb(ESP)
    doc = fp.emit_intent(pcb, ESP, declare_classes=True)
    frag, rep = db.compile_brief(db.load_brief(BRIEF_711),
                                 board_refs=sorted(pcb.footprints))
    doc = db.merge_into_intent(doc, frag, rep)
    rows = _esp_roster(doc, pcb, brief_fragment=frag)
    owed = sorted(n for n, r in rows.items() if r['needs_disposition'])
    assert owed == ['decap_distance', 'zone_containment'], owed
    for n in ('proximity', 'zone_exclusive'):
        assert rows[n]['state'] == 'dark' and rows[n]['policy'], rows[n]
        assert not rows[n]['needs_disposition'], n
    pin = rows['decap_pin_distance']
    assert pin['state'] == 'dark' and pin['applicable'] and not pin['gating']
    assert rows['zone_side']['applicable'] is False, rows['zone_side']
    assert rows['keepout']['state'] == 'armed', rows['keepout']
    lines = fp.roster_refusal_lines(rows.values())
    assert any('dispositions.rules.decap_distance' in s
               and '`decaps.max_distance_mm`' in s for s in lines), lines
    print(f"  PASS: owed {owed}; policy and advisory rules reported, not owed")


def test_a_disposition_answers_the_refusal():
    pcb = parse_kicad_pcb(ESP)
    rows = _esp_roster(_raw(dispositions={'rules': {
        'decap_distance': 'no decoupling requirement is known for this '
                          'design yet; graded by proximity instead'}}), pcb)
    r = rows['decap_distance']
    assert r['state'] == 'dark' and not r['needs_disposition'], r
    assert r['disposition'].startswith('no decoupling requirement'), r
    print("  PASS: a written disposition answers a dark rule")


def test_disposition_validation_at_load():
    bad = [
        ({'rules': {'decap_distance': ''}}, 'non-empty'),
        ({'rules': {'decap_distanc': 'x'}}, 'not a rule'),
        ({'nonsense': {'a': 'b'}}, 'unknown key'),
        ({'withheld': {'oob_area': 'x'}}, 'withholdable'),
        ({'rules': {'decap_distance': '   '}}, 'non-empty'),
    ]
    for disp, want in bad:
        try:
            fp.intent_from_dict(_raw(dispositions=disp), '')
        except fp.IntentError as exc:
            assert want in str(exc), (disp, str(exc))
        else:
            raise AssertionError(f'accepted {disp!r}')
    # A disposition for a rule the intent ARMS contradicts the intent.
    try:
        fp.intent_from_dict(_raw(decaps={'max_distance_mm': 2.0},
                                 dispositions={'rules': {
                                     'decap_distance': 'not graded'}}), '')
    except fp.IntentError as exc:
        assert 'ARMED' in str(exc), str(exc)
    else:
        raise AssertionError('accepted a disposition for an armed rule')
    ok = fp.intent_from_dict(_raw(dispositions={
        'rules': {'proximity': 'no relation is declared'},
        'withheld': {'overlap_area': 'inherited from the source board'}}), '')
    assert ok.dispositions['rules']['proximity'] == 'no relation is declared'
    print(f"  PASS: {len(bad) + 1} malformed dispositions refused by reason, "
          f"a well-formed one kept")


def test_a_severity_demotion_does_not_excuse_a_dark_rule():
    """A dark rule never runs, so demoting it changes nothing but P1's answer.
    That would be a way to make the refusal go away by editing a severity no
    finding will ever carry."""
    pcb = parse_kicad_pcb(ESP)
    plain = _esp_roster(_raw(), pcb)
    demoted = _esp_roster(_raw(severity={'legality': 'warn',
                                         'decap_distance': 'warn'}), pcb)
    for n in ('legality', 'decap_distance'):
        assert plain[n]['needs_disposition'], plain[n]
        assert demoted[n]['needs_disposition'], (n, demoted[n])
    print("  PASS: demoting a dark rule leaves it owed")


def test_a_severity_promotion_makes_an_advisory_rule_owed():
    pcb = parse_kicad_pcb(ESP)
    plain = _esp_roster(_raw(), pcb)
    raised = _esp_roster(_raw(severity={'assembly_side': 'error'}), pcb)
    assert not plain['assembly_side']['needs_disposition']
    assert not plain['assembly_side']['gating']
    assert raised['assembly_side']['needs_disposition'], raised['assembly_side']
    # pins_to_edge ignores the map, so a promotion cannot make it gating.
    forced = _esp_roster(_raw(severity={'pins_to_edge': 'error'}), pcb)
    assert not forced['pins_to_edge']['gating']
    print("  PASS: promotion gates, the forced-WARN rule stays advisory")


def test_a_withheld_budget_is_owed_until_answered():
    """legality armed on oob_count, overlap_area WITHHELD: the rule runs and
    half of it is never graded. That half is owed like a dark rule (#959
    Phase 0 checkpoint), and a disposition naming the key answers it."""
    pcb = parse_kicad_pcb(ESP)
    held = {'budget_withheld': {'overlap_area': '83 pairs on the source'}}
    rows = _esp_roster(_raw(legality_budget={'oob_count': 0},
                            context=held), pcb)
    lg = rows['legality']
    assert lg['state'] == 'armed' and lg['withheld'] == {
        'overlap_area': '83 pairs on the source'}, lg
    assert lg['needs_disposition'], lg
    line = [s for s in fp.roster_refusal_lines(rows.values())
            if s.startswith('legality')]
    assert line and 'dispositions.withheld.overlap_area' in line[0], line
    ok = _esp_roster(_raw(legality_budget={'oob_count': 0}, context=held,
                          dispositions={'withheld': {
                              'overlap_area': 'graded by check_assembly'}}),
                     pcb)
    assert not ok['legality']['needs_disposition'], ok['legality']
    print("  PASS: a withheld key is owed, and answered by naming it")


def test_a_stale_disposition_is_named():
    pcb = parse_kicad_pcb(ESP)
    it = fp.intent_from_dict(_raw(dispositions={'withheld': {
        'overlap_area': 'nothing is withheld here'}}), '')
    rows = fp.rule_roster(it, pcb, ESP)
    assert fp.stale_dispositions(it, rows) == [
        'dispositions.withheld.overlap_area'], fp.stale_dispositions(it, rows)
    print("  PASS: a disposition answering nothing is named")


def test_the_run29_pile_owes_decaps_and_the_withheld_overlap():
    """Run 29's own zone plan on its own staged pile: the headline case. The
    decap census has 4 caps with an IC on their rail and the plan declares no
    limit; legality is armed with `overlap_area` withheld. Exactly those two
    are owed -- and the census reads the same on the pile as on the placed
    board, because tethered + beyond-radius does not depend on the poses."""
    pcb = parse_kicad_pcb(PILE)
    it = fp.load_intent(os.path.join(FIX, 'zone_plan_r1.json'))
    rows = {r['rule']: r for r in fp.rule_roster(it, pcb, PILE)}
    owed = sorted(n for n, r in rows.items() if r['needs_disposition'])
    assert owed == ['decap_distance', 'legality'], owed
    assert 'overlap_area' in rows['legality']['withheld'], rows['legality']
    assert '4 decoupling cap(s)' in rows['decap_distance'][
        'applicability_reason'], rows['decap_distance']
    print(f"  PASS: run 29's plan owes {owed}")


def test_a_board_with_only_orphan_caps_is_not_applicable():
    it = fp.intent_from_dict(_raw(), '')
    app, why = fp._applicability(
        'decap_distance', it, None, None,
        {'tethers': 0, 'beyond_radius': 0, 'no_rail_chip': 3}, None)
    assert app is False and 'no decoupling cap shares a rail' in why, why
    app, why = fp._applicability(
        'decap_distance', it, None, None,
        {'tethers': 0, 'beyond_radius': 1, 'no_rail_chip': 3}, None)
    assert app is True, why
    print("  PASS: orphan caps make the decap rules inapplicable; one "
          "beyond-radius cap makes them applicable")


def test_the_cli_prints_the_roster_and_the_carried_facts():
    with tempfile.TemporaryDirectory() as tmp:
        intent = os.path.join(tmp, 'i.json')
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('check_floorplan.py'), ESP,
                         '--emit-intent', intent, '--declare-classes',
                         '--brief', BRIEF_711], accept=True)
        out = os.path.join(tmp, 'g.json')
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), ESP,
                             '--intent', intent, '--brief', BRIEF_711,
                             '--json', out], code=4)
        text = r.stdout
        assert 'rule roster:' in text and 'OWED' in text, text[-2000:]
        assert 'CARRIED, NOT physically checked' in text, text[-2000:]
        run_utils.evidence(out, 'the grade JSON')
        with open(out, encoding='utf-8') as fh:
            doc = json.load(fh)
        ledger = doc['declaration_ledger']
        kinds = {row['kind'] for row in ledger}
        assert kinds == {'rule', 'brief_clause'}, kinds
        statuses = {row['status'] for row in ledger}
        for s in ('graded_pass', 'carried', 'unknown', 'dark', 'inapplicable'):
            assert s in statuses, (s, statuses)
        line = [x for x in text.splitlines() if x.startswith('JSON_SUMMARY:')]
        s = json.loads(line[-1].split('JSON_SUMMARY: ', 1)[1])
        # In RULES order, which is the order the roster reports in.
        assert s['rules_dark_undispositioned'] == ['zone_containment',
                                                   'decap_distance'], s
        assert len(s['carried_facts']) == 8, s['carried_facts']
        assert s['ledger_status']['carried'] == 8, s['ledger_status']
    print("  PASS: the CLI prints the roster, names the 8 carried facts, and "
          "writes the ledger")


def test_a_grade_without_the_roster_says_so():
    pcb = parse_kicad_pcb(ESP)
    res = fp.grade(fp.intent_from_dict(_raw(), ''), pcb, ESP)
    assert res.roster is None
    assert fp.summary(res)['rules_dark_undispositioned'] is None
    print("  PASS: no roster reads as None, never as 'nothing owed'")


def test_p1_refuses_by_name_and_passes_once_answered():
    """The driver's own tiny board declares no envelope and no legality
    budget; both apply to any outlined board and both are gating."""
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    drv = importlib.import_module('placement_driver')
    with tempfile.TemporaryDirectory() as tmp:
        board = drv._tiny_board(os.path.join(tmp, 'b.kicad_pcb'),
                                ('U1', 'U2'))
        blocks = [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                   'note': 'both parts, one zone'}]
        bare = os.path.join(tmp, 'bare.json')
        with open(bare, 'w', encoding='utf-8') as fh:
            json.dump(drv._zone_plan_doc(blocks, dispositions={}), fh)
        argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
                '--board', board, '--zone-plan', bare]
        r = run_utils.check(argv, refuse='dispositions.rules.envelope',
                            code=4)
        assert '`legality_budget`' in r.stdout, r.stdout
        answered = os.path.join(tmp, 'ok.json')
        with open(answered, 'w', encoding='utf-8') as fh:
            json.dump(drv._zone_plan_doc(blocks), fh)
        r = run_utils.check(argv[:-1] + [answered], accept=True)
        assert '<stage_instructions' in r.stdout, r.stdout[:400]
    print("  PASS: P1 refuses naming the arming key, and passes once "
          "answered")


TESTS = [
    test_every_rule_has_its_table_entries,
    test_the_arming_key_is_what_arms_the_rule,
    test_the_default_severity_table_is_what_the_rules_emit,
    test_esp_prog_with_its_brief_owes_exactly_two,
    test_a_disposition_answers_the_refusal,
    test_disposition_validation_at_load,
    test_a_severity_demotion_does_not_excuse_a_dark_rule,
    test_a_severity_promotion_makes_an_advisory_rule_owed,
    test_a_withheld_budget_is_owed_until_answered,
    test_a_stale_disposition_is_named,
    test_the_run29_pile_owes_decaps_and_the_withheld_overlap,
    test_a_board_with_only_orphan_caps_is_not_applicable,
    test_the_cli_prints_the_roster_and_the_carried_facts,
    test_a_grade_without_the_roster_says_so,
    test_p1_refuses_by_name_and_passes_once_answered,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
