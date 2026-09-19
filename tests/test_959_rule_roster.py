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
import subprocess
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

RUN_ALL_TIMEOUT = 1800

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
        # The WHOLE path, not only its first segment: every segment the table
        # names must be present in the fragment that arms the rule (the
        # Phase-1 verifier renamed `envelope.rect` to
        # `envelope.tolerance_mm` and this test still passed).
        node = frag
        for seg in fp._ARMING_KEY[name].split(' ')[0].split('.'):
            key = seg.split('[')[0]
            assert isinstance(node, dict) and key in node, (
                name, fp._ARMING_KEY[name], seg, node)
            node = node[key]
            if isinstance(node, list):
                node = node[0]
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
        must_lock=['C1'],
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
                 'proximity', 'pins_to_edge', 'must_lock'):
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
        'dispositions.withheld.overlap_area: nothing is withheld under that '
        'key'], fp.stale_dispositions(it, rows)
    print("  PASS: a disposition answering nothing is named")


def test_a_stale_ref_disposition_says_why():
    """Each of the three ways a `refs` answer can be stale gets its OWN
    reason: the round-2 verifier found check_floorplan telling a locked
    block's author that "nothing is withheld", which is about a different
    key entirely."""
    pcb = parse_kicad_pcb(ESP)
    logo = sorted(k for k, f in pcb.footprints.items() if not f.pads)[0]
    pcb.footprints[logo].locked = True
    it = fp.intent_from_dict(_raw(dispositions={'refs': {
        'NOPE': 'x', 'U1': 'x', logo: 'x'}}), '')
    rows = fp.rule_roster(it, pcb, ESP)
    got = fp.stale_dispositions(it, rows, pcb)
    want = sorted([
        'dispositions.refs.NOPE: no such block on this board (keys are '
        'exact)',
        'dispositions.refs.U1: the block carries pads -- the seeder places '
        'it, and `refs` answers pad-less blocks only',
        f'dispositions.refs.{logo}: the block is already locked; the lock '
        'is the answer'])
    assert got == want, got
    # Without the board the refs cannot be judged, so none are named.
    assert fp.stale_dispositions(it, rows) == [], fp.stale_dispositions(it,
                                                                     rows)
    # And an unlocked pad-less block is exactly what `refs` is for.
    pcb.footprints[logo].locked = False
    assert not [s for s in fp.stale_dispositions(it, rows, pcb)
                if logo in s]
    lines = fp.format_roster(rows, got)
    assert sum('STALE: dispositions.refs.' in ln for ln in lines) == 3, lines
    assert not [ln for ln in lines if 'refs.' in ln and 'withheld' in ln]
    # The CLI carries the same reason, in its text and its summary.
    with tempfile.TemporaryDirectory() as tmp:
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(_raw(dispositions={'refs': {'U1': 'x'}}), fh)
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            run_utils.tool('check_floorplan.py'), ESP,
                            '--intent', plan, '--plan-only',
                            '--no-mechanical'],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=900)
        assert 'Traceback' not in r.stdout + r.stderr, r.stderr[-1500:]
        why = ('dispositions.refs.U1: the block carries pads -- the seeder '
               'places it, and `refs` answers pad-less blocks only')
        assert 'STALE: ' + why in r.stdout, r.stdout[-2000:]
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')]
        s = json.loads(line[-1].split('JSON_SUMMARY: ', 1)[1])
        assert s['stale_dispositions'] == [why], s['stale_dispositions']
    print("  PASS: an unknown, a pad-bearing and a locked `refs` answer each "
          "name their own reason, in the API and the CLI")


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
        assert kinds == {'rule', 'brief_clause', 'reconciliation',
                         'derived_clause'}, kinds
        statuses = {row['status'] for row in ledger}
        for s in ('graded_pass', 'carried', 'unknown', 'dark', 'inapplicable'):
            assert s in statuses, (s, statuses)
        line = [x for x in text.splitlines() if x.startswith('JSON_SUMMARY:')]
        s = json.loads(line[-1].split('JSON_SUMMARY: ', 1)[1])
        # In RULES order, which is the order the roster reports in.
        assert s['rules_dark_undispositioned'] == ['zone_containment',
                                                   'decap_distance'], s
        # 8 before #1000; the four connector clauses (USB1 and CON2's
        # mount_mode and cable_entry) now COMPILE, and the product facts are
        # what is left carried.
        assert s['carried_facts'] == [
            'product.form_factor', 'product.held_by',
            'product.primary_axis', 'product.user_top_side'], s[
                'carried_facts']
        assert s['ledger_status']['carried'] == 4, s['ledger_status']
    print("  PASS: the CLI prints the roster, names the 4 carried facts, and "
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




def _driver():
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    return importlib.import_module('placement_driver')


def _tiny_with_brief(tmp, brief, name='b', **board_kw):
    drv = _driver()
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    board = drv._tiny_board(os.path.join(d, 'board.kicad_pcb'),
                            ('U1', 'U2'), **board_kw)
    if brief is not None:
        with open(os.path.join(d, 'board.design-brief.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump(dict({'schema': 1, 'kind': 'design-brief',
                            'units': 'mm', 'board': 'board.kicad_pcb'},
                           **brief), fh)
    return drv, board


def _p1(board, plan_doc, tmp, *extra, name='p.json'):
    p = os.path.join(tmp, name)
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump(plan_doc, fh)
    return [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
            '--board', board, '--zone-plan', p] + list(extra)


BLOCKS = [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
           'note': 'both parts, one zone'}]


def test_a_plan_that_drops_brief_declarations_is_refused_at_p1():
    """Phase-1 verifier B1: a plan with `proximity: []` passed P1 against a
    brief declaring proximity claims, and one with no edge entries passed
    against a brief claiming edges -- the roster called both "only a
    declaration can arm it" / "nothing has an edge to claim". Now P1's
    brief-clause check refuses each dropped clause by id (the coverage
    P-close grades), and the roster names the brief instead of asking for a
    second answer to the same question."""
    with tempfile.TemporaryDirectory() as tmp:
        drv, board = _tiny_with_brief(tmp, {
            'interfaces': [{'ref': 'U2', 'edge': 'east',
                            'user_facing': True}],
            'proximity': [{'ref': 'U1', 'near': 'U2', 'max_mm': 5.0,
                           'requirement': 'R1', 'why': 'test'}]},
            locked=('U2',))
        r = run_utils.check(_p1(board, drv._zone_plan_doc(BLOCKS), tmp),
                            refuse='drops or contradicts', code=4)
        assert 'interfaces[U2]' in r.stdout, r.stdout
        assert 'proximity[' in r.stdout, r.stdout
        # Carrying both clauses answers it.
        full = drv._zone_plan_doc(
            BLOCKS, edge_connectors=[{'ref': 'U2', 'edge': 'east',
                                      'class': 'edge_receptacle'}],
            proximity=[{'ref': 'U1', 'near': 'U2', 'max_mm': 5.0}])
        r = subprocess.run(_p1(board, full, tmp, name='full.json'),
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=900)
        assert 'drops or contradicts' not in r.stdout, r.stdout
        # The roster: named, applicable, answered at the clause gate.
        from placement import design_brief as db
        pcb = parse_kicad_pcb(board)
        bp = os.path.join(os.path.dirname(board), 'board.design-brief.json')
        frag, _ = db.compile_brief(db.load_brief(bp),
                                   board_refs=sorted(pcb.footprints))
        rows = {r_['rule']: r_ for r_ in fp.rule_roster(
            fp.intent_from_dict(drv._zone_plan_doc(BLOCKS), ''), pcb,
            board, brief_fragment=frag)}
        for name in ('proximity', 'edge_connector'):
            row = rows[name]
            assert row['brief_claims'] and row['applicable'], row
            assert not row['policy'] and not row['needs_disposition'], row
            assert 'design brief declares' in row['applicability_reason']
        led = {x['id']: x for x in fp.declaration_ledger(
            fp.intent_from_dict(drv._zone_plan_doc(BLOCKS), ''),
            list(rows.values()))}
        assert led['rule:proximity']['status'] == 'uncovered', led
    print("  PASS: dropped brief proximity and edge clauses are refused at "
          "P1 by id; carried, they pass; the roster names the brief")


def test_a_malformed_brief_refuses_p1():
    with tempfile.TemporaryDirectory() as tmp:
        drv, board = _tiny_with_brief(tmp, {'bogus_key': 1})
        run_utils.check(_p1(board, drv._zone_plan_doc(BLOCKS), tmp),
                        refuse='cannot be read', code=4)
    print("  PASS: a brief check_floorplan refuses is refused at P1 too")


def test_the_withheld_debt_is_read_off_the_budget():
    """Phase-1 verifier B2: deleting the plan's own `context.budget_withheld`
    note deleted the debt. A legality budget armed without `overlap_area` or
    `oob_count` now owes the missing key, whatever the notes say."""
    pcb = parse_kicad_pcb(PILE)
    with open(os.path.join(FIX, 'zone_plan_r1.json'), encoding='utf-8') as fh:
        r1 = json.load(fh)
    stripped = json.loads(json.dumps(r1))
    stripped.get('context', {}).pop('budget_withheld', None)
    for doc in (r1, stripped,
                _raw(legality_budget={'oob_count': 0})):
        rows = {r['rule']: r for r in fp.rule_roster(
            fp.intent_from_dict(doc, ''), pcb, PILE)}
        leg = rows['legality']
        assert leg['state'] == 'armed' and leg['needs_disposition'], leg
        assert 'overlap_area' in leg['withheld'], leg['withheld']
        led = {x['id']: x for x in fp.declaration_ledger(
            fp.intent_from_dict(doc, ''), list(rows.values()))}
        assert led['rule:legality']['status'] == 'dark', led['rule:legality']
        assert 'overlap_area' in led['rule:legality']['why']
    answered = _raw(legality_budget={'oob_count': 0},
                    dispositions={'withheld': {
                        'overlap_area': 'graded by check_assembly'}})
    rows = {r['rule']: r for r in fp.rule_roster(
        fp.intent_from_dict(answered, ''), pcb, PILE)}
    assert not rows['legality']['needs_disposition'], rows['legality']
    # The other half: a budget with `overlap_area` but no `oob_count`.
    rows = {r['rule']: r for r in fp.rule_roster(
        fp.intent_from_dict(_raw(legality_budget={'overlap_area': 1.0}), ''),
        pcb, PILE)}
    assert 'oob_count' in rows['legality']['withheld'], rows['legality']
    print("  PASS: overlap_area (and oob_count) are owed with the note, "
          "without it, and on a hand-written budget; a disposition answers it")


def test_a_suspect_overhang_records_why_oob_count_is_withheld():
    """The emitter drops `oob_count` when an edge connector sits in a
    SUSPECT pose; since #959 that drop is recorded in `budget_withheld`, so
    the roster can owe it rather than miss it."""
    from kicad_parser import iter_footprint_blocks
    pcb = parse_kicad_pcb(ESP)
    bx = pcb.board_info.board_bounds
    # R1 and C2 stacked on each other, overhanging the east edge: each
    # overhang is observed, and a part in a pad conflict is SUSPECT.
    x, y = bx[2] - 0.4, (bx[1] + bx[3]) / 2
    text = open(ESP, encoding='utf-8').read()
    for ref in ('R1', 'C2'):
        for start, end, _t, _r, key in iter_footprint_blocks(text):
            if key == ref:
                blk = text[start:end]
                i = blk.index('(at ')
                j = blk.index(')', i)
                blk = blk[:i] + f'(at {x} {y}' + blk[j:]
                text = text[:start] + blk + text[end:]
                break
    with tempfile.TemporaryDirectory() as tmp:
        b = os.path.join(tmp, 'susp.kicad_pcb')
        open(b, 'w', encoding='utf-8').write(text)
        doc = fp.emit_intent(parse_kicad_pcb(b), b, declare_classes=True)
        sus = [c for c in doc['edge_connectors'] if c.get('suspect')]
        assert sus, [c.get('note') for c in doc['edge_connectors']]
        assert 'oob_count' not in (doc.get('legality_budget') or {}), doc
        held = (doc.get('context') or {}).get('budget_withheld') or {}
        assert 'SUSPECT' in held.get('oob_count', ''), held
    print("  PASS: a SUSPECT connector withholds oob_count and says why")


def test_ledger_statuses_say_what_the_roster_says():
    """Phase-1 verifier S1: armed-with-an-open-key read `graded_pass`, and
    policy / advisory rows read `inapplicable`. Also pins `pending` before a
    grade and `graded_fail` after one."""
    pcb = parse_kicad_pcb(ESP)
    side = pcb.footprints['R1']
    from placement import legality as _leg
    raw = _raw(blocks=[{'name': 'far', 'refs': ['U1'], 'zone': [0, 0, 1, 1],
                        'note': 'U1 is nowhere near here'},
                       {'name': 'r1', 'refs': ['R1'],
                        'side': _leg.footprint_side(side),
                        'note': 'R1 on its own face'}],
               legality_budget={'oob_count': 0})
    it = fp.intent_from_dict(raw, '')
    rows = fp.rule_roster(it, pcb, ESP)
    before = {x['id']: x for x in fp.declaration_ledger(it, rows)}
    assert before['rule:zone_containment']['status'] == 'pending', before
    assert before['rule:legality']['status'] == 'dark', before
    assert before['rule:proximity']['status'] == 'policy', before
    assert before['rule:zone_exclusive']['status'] == 'policy', before
    assert before['rule:decap_pin_distance']['status'] == 'advisory', before
    assert before['rule:must_lock']['status'] == 'inapplicable', before
    by = {r['rule']: r for r in rows}
    assert by['zone_containment']['applicable'] is True
    assert 'armed by' in by['zone_containment']['applicability_reason']
    res = fp.grade(it, pcb, ESP, with_roster=True)
    after = {x['id']: x for x in fp.declaration_ledger(
        it, res.roster, result=res)}
    assert after['rule:zone_containment']['status'] == 'graded_fail', after
    assert after['rule:zone_side']['status'] == 'graded_pass', after
    print("  PASS: pending / graded_fail / graded_pass / dark / policy / "
          "advisory / inapplicable each where the roster says")


def test_a_stale_only_plan_opens_with_the_true_sentence():
    """Phase-1 verifier S4: a plan whose only debt is a stale disposition
    opened "0 rule(s) this plan leaves dark ... nothing answers for them"."""
    with tempfile.TemporaryDirectory() as tmp:
        drv, board = _tiny_with_brief(tmp, None)
        doc = drv._zone_plan_doc(BLOCKS, dispositions={
            'rules': {'envelope': 'fixture', 'legality': 'fixture'},
            'withheld': {'overlap_area': 'nothing withholds this'}})
        r = run_utils.check(_p1(board, doc, tmp),
                            refuse='answers something that is not asked',
                            code=4)
        assert '0 rule(s)' not in r.stdout, r.stdout
        assert 'STALE dispositions.withheld.overlap_area' in r.stdout
    print("  PASS: a stale-only plan is refused with the stale header")


def test_the_roster_runs_last_at_p1():
    """Phase-1 verifier S6/D02: the older P1 refusals keep precedence. A plan
    with an unzoned movable part AND a dark gating rule is refused for the
    unzoned part."""
    with tempfile.TemporaryDirectory() as tmp:
        drv = _driver()
        board = drv._tiny_board(os.path.join(tmp, 'b.kicad_pcb'),
                                ('U1', 'U2'))
        doc = drv._zone_plan_doc(
            [{'name': 'one', 'refs': ['U1'], 'zone': [0, 0, 10, 10],
              'note': 'U1 only'}], dispositions={})
        r = run_utils.check(_p1(board, doc, tmp),
                            refuse='sit in no zoned block', code=4)
        assert 'dispositions.rules.envelope' not in r.stdout, r.stdout
    print("  PASS: the unzoned-part refusal comes before the roster's")


def test_gating_and_applicability_predicates():
    """Phase-1 verifier S5: the pin-distance gating arms, the edge-claim
    classes, a two-faced board's zone_side, and `_arm_decap_pins`."""
    def roster(path, **raw):
        pcb = parse_kicad_pcb(path)
        return {r['rule']: r for r in fp.rule_roster(
            fp.intent_from_dict(_raw(**raw), ''), pcb, path)}
    sf = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
    fh_ = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    ul = os.path.join(REPO, 'kicad_files', 'ulx3s.kicad_pcb')
    assert roster(sf)['decap_pin_distance']['gating'] is False
    assert roster(sf, severity={'decap_pin_distance_inferred': 'error'})[
        'decap_pin_distance']['gating'] is True
    assert roster(fh_)['decap_pin_distance']['gating'] is True
    assert roster(fh_, severity={'decap_pin_distance': 'warn'})[
        'decap_pin_distance']['gating'] is True
    # ...and on a pintype board the rule's OWN findings carry its table
    # severity (F17: a table entry moved to WARN survived every test).
    res = fp.grade(fp.intent_from_dict(
        _raw(decaps={'max_pin_distance_mm': 0.01}), ''),
        parse_kicad_pcb(fh_), fh_)
    sev = {v.severity for v in res.violations
           if v.rule == 'decap_pin_distance'}
    assert sev == {fp._RULE_DEFAULT_SEVERITY['decap_pin_distance']}, sev
    assert roster(ESP)['edge_connector']['applicable'] is True
    assert roster(ul)['zone_side']['applicable'] is True
    assert roster(ESP)['zone_side']['applicable'] is False
    with tempfile.TemporaryDirectory() as tmp:
        tiny = _driver()._tiny_board(os.path.join(tmp, 't.kicad_pcb'),
                                     ('U1', 'U2'))
        t = roster(tiny)
        assert t['edge_connector']['applicable'] is False, t['edge_connector']
        assert t['decap_pin_distance']['applicable'] is False, t[
            'decap_pin_distance']
        ab = roster(tiny, decaps={'max_pin_distance_mm': 1.0})
        assert ab['decap_pin_distance']['state'] == 'abstained', ab[
            'decap_pin_distance']
    print("  PASS: pin gating by channel and promotion; edge classes both "
          "ways; zone_side on one and two faces; an abstained pin rule")


def test_the_cli_carries_the_roster_everywhere_it_says():
    """C01 / C05 / F77: the emit path prints the roster, JSON_SUMMARY carries
    `stale_dispositions`, and `to_json` carries `rule_roster`."""
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'e.json')
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), ESP,
                             '--emit-intent', out], accept=True)
        assert 'rule roster:' in r.stdout, r.stdout[-1500:]
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(_raw(dispositions={'withheld': {'oob_amount': 'x'}}),
                      fh)
        js = os.path.join(tmp, 'g.json')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            run_utils.tool('check_floorplan.py'), ESP,
                            '--intent', plan, '--json', js],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=900)
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')][-1]
        s = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
        assert s['stale_dispositions'] == [
            'dispositions.withheld.oob_amount: nothing is withheld under '
            'that key'], s['stale_dispositions']
        with open(js, encoding='utf-8') as fh:
            doc = json.load(fh)
        assert isinstance(doc.get('rule_roster'), list) and doc[
            'rule_roster'], doc.get('rule_roster')
    print("  PASS: emit prints the roster; the summary names the stale key; "
          "the JSON carries the roster")


TESTS = [
    test_a_plan_that_drops_brief_declarations_is_refused_at_p1,
    test_a_malformed_brief_refuses_p1,
    test_the_withheld_debt_is_read_off_the_budget,
    test_a_suspect_overhang_records_why_oob_count_is_withheld,
    test_ledger_statuses_say_what_the_roster_says,
    test_a_stale_only_plan_opens_with_the_true_sentence,
    test_the_roster_runs_last_at_p1,
    test_gating_and_applicability_predicates,
    test_the_cli_carries_the_roster_everywhere_it_says,
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
    test_a_stale_ref_disposition_says_why,
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
