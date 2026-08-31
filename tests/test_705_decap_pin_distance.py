"""#705: grade the decoupling cap to the PIN, not to the package.

`decap_distance` measures cap CENTROID to the IC's pad bbox inflated 0.5mm,
clamped to 0 inside. The requirement is about the pin: a 100nF 1.7mm from a QFN
and 9mm from the IOVDD pin it decouples satisfies that rule and fails the spec.

THE FIXTURE IS `lvds_converter_dualclk`, AND IT IS CHOSEN, NOT CONVENIENT.
It is the only board in the corpus that carries `pintype` on 100% of its pads
while ZERO of them are `power_in`/`power_out` -- IC2/IC3/IC4's supply pins are
typed `passive` and named `VCC_14` / `GND_7` / `VCC_16`. So it is the only
naturally-occurring killer of a PRESENCE-keyed channel ladder, which would stop
at channel 1 with an empty set, grade nothing, and record that channel 1 fired.
It is also small (13 footprints), its three VCC pins measure 2.100 / 2.100 /
1.925 mm so a limit can be TIGHT rather than merely satisfied, and its three
GND pins measure 6.94-8.04 mm so a mutation that starts grading ground produces
exactly three new findings.

WHAT MAKES A CLEAN ARM MEAN SOMETHING HERE. "No violations" is what a rule that
never armed reports too, so every arm that expects nothing also asserts that the
rule RAN, and the tight arms assert a count that only the correct measurement
produces. The abstention arms assert the REASON, not the exit code.
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('', 'py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402
from placement import legality                              # noqa: E402

LVDS = 'lvds_converter_dualclk'


def _board(name):
    return os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')


def _graded(name, decaps):
    path = _board(name)
    pcb = parse_kicad_pcb(path)
    doc = fp.emit_intent(pcb, path)
    doc['decaps'] = dict(decaps)
    return fp.grade(fp.intent_from_dict(doc, path), pcb, path), pcb


def _of(result, rule):
    return [v for v in result.violations if v.rule == rule]


def _pin(result):
    return [v for v in result.violations if v.rule.startswith('decap_pin')]


def test_the_ladder_falls_through_on_YIELD_not_on_the_field_being_present():
    """`lvds_converter_dualclk` has pintype on every pad and no power pin.

    This is the arm the whole ladder design turns on. A presence-keyed ladder
    grades NOTHING here while reporting that channel 1 fired -- and no
    naturally-written test catches that, because the field IS populated."""
    if not os.path.exists(_board(LVDS)):
        print(f"  SKIP: {LVDS} absent")
        return
    pcb = parse_kicad_pcb(_board(LVDS))
    typed = sum(1 for f in pcb.footprints.values() for p in f.pads
                if (getattr(p, 'pintype', '') or ''))
    pads = sum(len(f.pads) for f in pcb.footprints.values())
    assert typed == pads and pads > 0, (typed, pads)   # the trap is armed
    recs = fp.supply_pins(pcb)
    for ref in ('IC2', 'IC3', 'IC4'):
        r = recs[ref]
        assert r['counts']['pintype'] == 0, (ref, r['counts'])
        assert r['channel'] == 'pinfunction', (ref, r['channel'])
        assert len(r['pins']) == 1, (ref, len(r['pins']))
    print(f"  PASS: {typed}/{pads} pads carry a pintype and NONE is a supply "
          f"type; the ladder falls through to pinfunction and finds all three")


def test_the_limit_is_TIGHT_not_merely_satisfied():
    """2.100 / 2.100 / 1.925 mm, so three limits give three different answers.

    A single "grades clean at 3mm" arm is satisfied by a rule that measures
    nothing. These three are satisfied only by the real numbers."""
    if not os.path.exists(_board(LVDS)):
        print(f"  SKIP: {LVDS} absent")
        return
    for limit, want in ((3.0, 0), (2.0, 2), (1.9, 3)):
        r, _p = _graded(LVDS, {'max_pin_distance_mm': limit})
        assert 'decap_pin_distance' in r.rules_run, (limit, sorted(r.rules_run))
        got = _of(r, 'decap_pin_distance')
        assert len(got) == want, (limit, len(got), [v.message for v in got])
        if got:
            assert all(v.severity == fp.ERROR for v in got)
    print("  PASS: 3.0mm -> 0 findings, 2.0mm -> 2, 1.9mm -> 3 -- the rule "
          "ran in all three, so the zero is a measurement not an absence")


def test_the_measured_payload_names_the_pin_the_cap_and_the_channel():
    if not os.path.exists(_board(LVDS)):
        print(f"  SKIP: {LVDS} absent")
        return
    r, _p = _graded(LVDS, {'max_pin_distance_mm': 2.0})
    v = sorted(_of(r, 'decap_pin_distance'), key=lambda x: x.ref)[0]
    m = v.measured
    assert v.ref == 'IC2', v.ref
    assert m['pad'] == '14' and m['net'] == '/VCC', m
    assert m['cap'] == 'C2', m
    assert abs(m['gap_mm'] - 2.1) < 1e-3, m
    assert m['channel'] == 'pinfunction', m
    assert m['caps_on_rail'] >= 2 and m['pins_on_rail'] == 1, m
    assert m['ic_side'] == 'F' and m['cap_side'] == 'F', m
    print(f"  PASS: {v.ref} pin {m['pad']} -> {m['cap']} at {m['gap_mm']}mm "
          f"via {m['channel']}; the finding carries its own evidence")


def test_the_gap_is_measured_to_the_caps_RAIL_pad_not_its_nearest_one():
    """A 0402's ground leg can sit nearer than its rail leg and shave the
    number. Asserted by construction: the gap must equal the distance to the
    cap pad ON THE PIN'S NET, and must NOT equal the smaller all-pads value
    wherever those differ."""
    if not os.path.exists(_board(LVDS)):
        print(f"  SKIP: {LVDS} absent")
        return
    pcb = parse_kicad_pcb(_board(LVDS))
    recs = fp.supply_pins(pcb)
    checked = differ = 0
    for ref, rec in recs.items():
        for pad, _net in rec['pins']:
            for cap in pcb.footprints.values():
                if cap.reference == ref:
                    continue
                on_net = fp._pin_gap(pad, cap, pad.net_id)
                if on_net is None:
                    continue
                any_pad = min(
                    legality.rect_gap(legality.pad_rect(pad),
                                      legality.pad_rect(q))
                    for q in cap.pads)
                assert on_net >= any_pad - 1e-9, (ref, cap.reference)
                checked += 1
                differ += (on_net > any_pad + 1e-9)
    assert checked >= 5, checked
    # ANTI-VACUITY: if the two never differ, this arm proves nothing.
    assert differ >= 1, (differ, "no cap has a nearer non-rail pad here, so "
                                 "the arm cannot tell the two apart")
    print(f"  PASS: {checked} pin/cap pair(s), {differ} where the ground leg "
          f"is nearer -- the rule uses the rail leg on every one")


def test_ground_pins_are_not_graded():
    """The three GND pins on the SAME ICs measure 6.94-8.04mm -- a SOIC-14's
    opposite corners, not a placement. Grading them would flag every SOIC on
    every board with a number that is a datasheet fact."""
    if not os.path.exists(_board(LVDS)):
        print(f"  SKIP: {LVDS} absent")
        return
    pcb = parse_kicad_pcb(_board(LVDS))
    recs = fp.supply_pins(pcb)
    for ref, rec in recs.items():
        for pad, net in rec['pins']:
            assert 'GND' not in net.upper(), (ref, net)
    # And the pins DO exist and ARE far, so the exclusion is doing work.
    from net_queries import is_supply_pinfunction
    far = []
    by = fp._decap_caps_by_net(pcb)
    for ref in ('IC2', 'IC3', 'IC4'):
        for pad in pcb.footprints[ref].pads:
            fn = getattr(pad, 'pinfunction', '') or ''
            if not (fn.upper().startswith('GND')
                    and is_supply_pinfunction(fn)):
                continue
            caps = [c for c in by.get(pad.net_id, ()) if c.reference != ref]
            gs = [fp._pin_gap(pad, c, pad.net_id) for c in caps]
            if gs:
                far.append(round(min(g for g in gs if g is not None), 2))
    assert len(far) == 3 and min(far) > 6.0, far
    r = _graded(LVDS, {'max_pin_distance_mm': 3.0})[0]
    assert not _pin(r), [v.message for v in _pin(r)]
    # POSITIVE CONTROL. "0 findings at 3mm" is also what a rule that
    # emits nothing reports -- my own deletion probe passed this arm.
    # At 1.9mm the SAME board must produce exactly the three VCC pins,
    # and still no GND pin, which an inert rule cannot do.
    tight = _graded(LVDS, {'max_pin_distance_mm': 1.9})[0]
    hits = _pin(tight)
    assert len(hits) == 3, [v.message for v in hits]
    nets = [v.measured['net'] for v in hits]
    assert all('GND' not in n.upper() for n in nets), nets
    print(f"  PASS: the 3 GND pins sit {sorted(far)}mm from their nearest "
          f"cap and produce 0 findings at 3mm -- while the 3 VCC pins all "
          f"fire at 1.9mm, so the zero is a measurement")


def test_a_board_with_no_supply_pin_ABSTAINS_with_a_reason():
    """orangecrab_ext_pll: 28 candidate ICs, 0 supply pins on any channel.

    `rules_run` must NOT contain the rule -- otherwise the grade prints "N
    rule(s) ran, no violations" and `--require-rules` gets EASIER to satisfy,
    which is the vacuous pass that flag exists to catch."""
    name = 'orangecrab_ext_pll'
    if not os.path.exists(_board(name)):
        print(f"  SKIP: {name} absent")
        return
    r, pcb = _graded(name, {'max_pin_distance_mm': 3.0})
    assert 'decap_pin_distance' not in r.rules_run, sorted(r.rules_run)
    why = r.rules_skipped.get('decap_pin_distance', '')
    assert 'no supply pin' in why, why
    assert 'pintype yielded 0' in why, why
    assert not r.decap_pin_evidence, r.decap_pin_evidence
    # The abstention is about the BOARD, not the intent, so it must not be
    # confused with the emitter's withholding channel.
    assert 'decaps.max_pin_distance_mm' not in r.budget_abstained
    print(f"  PASS: {name} abstains -- {why[:96]}")


def test_a_board_with_no_capacitor_at_all_abstains_for_THAT_reason():
    """Two abstentions, two reasons. A board with pins and no caps must not be
    told it has no pins."""
    for name in ('haasoscope_pro_max_test', 'routed_output'):
        if not os.path.exists(_board(name)):
            continue
        r, _p = _graded(name, {'max_pin_distance_mm': 3.0})
        why = r.rules_skipped.get('decap_pin_distance', '')
        assert 'no decoupling capacitor' in why, (name, why)
        # It has plenty of supply pins, so the OTHER reason must not
        # fire. Discriminated on 'candidate IC(s)', which only the
        # pin-side reason carries: a bare 'no supply pin' test matches
        # the cap-side reason's own trailing clause, and did.
        assert 'candidate IC' not in why, (name, why)
        print(f"  PASS: {name} abstains on the caps, not on the pins")
        return
    print("  SKIP: no cap-free fixture present")


def test_an_uncovered_rail_is_a_separate_finding_at_WARN():
    """"No cap on this rail at all" is a design fact, not a placement failure,
    and it is per (IC, rail): `haasoscope_pro_max_test` has 98 typed supply
    pins and zero caps, which per-pin would be 98 findings saying one thing."""
    hits = []
    for path in run_utils.corpus_boards():
        name = os.path.basename(path).replace('.kicad_pcb', '')
        r, _p = _graded(name, {'max_pin_distance_mm': 3.0})
        hits += [(name, v) for v in _of(r, 'decap_pin_uncovered')]
    assert hits, "no board reports an uncovered rail -- the channel is inert"
    for name, v in hits:
        assert v.severity == fp.WARN, (name, v.severity)
        assert v.measured['channel'] in ('pintype', 'pinfunction'), v.measured
        assert v.expected == {'caps_on_rail': '>= 1'}, v.expected
    # Per (IC, rail): no IC may report the same rail twice.
    seen = [(n, v.ref, v.measured['net']) for n, v in hits]
    assert len(seen) == len(set(seen)), seen
    print(f"  PASS: {len(hits)} uncovered rail(s) across the corpus, all warn, "
          f"one per (IC, rail): "
          f"{', '.join(f'{n}:{v.ref}' for n, v in hits[:4])}")


def test_an_INFERRED_pin_carries_its_own_rule_name_at_warn():
    """ulx3s has no `pintype` at all, so every one of its pins comes from a net
    NAME. 39 findings at 3mm -- under one rule name they would FAIL the board on
    an inference."""
    if not os.path.exists(_board('ulx3s')):
        print("  SKIP: ulx3s absent")
        return
    r, _p = _graded('ulx3s', {'max_pin_distance_mm': 3.0})
    inf = _of(r, 'decap_pin_distance_inferred')
    assert len(inf) >= 20, len(inf)
    assert not _of(r, 'decap_pin_distance'), "ulx3s has no declared supply pin"
    assert all(v.severity == fp.WARN for v in inf)
    assert all(v.measured['channel'] == 'rail_net' for v in inf)
    assert all('inferred from the net NAME' in v.message for v in inf)
    assert not [v for v in r.errors if v.rule.startswith('decap_pin')]
    print(f"  PASS: ulx3s reports {len(inf)} inferred finding(s) at warn and "
          f"0 errors -- the board is not failed on an inference")


def test_the_human_reference_board_passes_on_its_DECLARED_pins():
    """#705's own verification claim, re-derived.

    The issue says glasgow_revC U30 has "16 power pins by pintype" and a
    "0.00 to 0.69 mm" range. Measured: **9 gradeable pins** (12 `power_in`, less
    3 that are ground) and a range of **-0.095 to 1.762 mm**. Neither figure
    reproduces -- but the conclusion survives, with 1.24mm of margin at 3mm
    rather than 2.31mm. The negative values are real pad overlaps, which a
    clamped metric would have erased."""
    if not os.path.exists(_board('glasgow_revC')):
        print("  SKIP: glasgow_revC absent")
        return
    pcb = parse_kicad_pcb(_board('glasgow_revC'))
    rec = fp.supply_pins(pcb)['U30']
    assert rec['channel'] == 'pintype', rec['channel']
    assert len(rec['pins']) == 9, len(rec['pins'])
    by = fp._decap_caps_by_net(pcb)
    gaps = []
    for pad, _net in rec['pins']:
        caps = [c for c in by.get(pad.net_id, ()) if c.reference != 'U30']
        gs = [fp._pin_gap(pad, c, pad.net_id) for c in caps]
        gs = [g for g in gs if g is not None]
        assert gs, pad.pad_number
        gaps.append(min(gs))
    assert min(gaps) < 0, min(gaps)          # a real overlap, not clamped away
    assert max(gaps) < 3.0, max(gaps)
    r = _graded('glasgow_revC', {'max_pin_distance_mm': 3.0})[0]
    assert not _of(r, 'decap_pin_distance'), [v.message for v in
                                              _of(r, 'decap_pin_distance')]
    # POSITIVE CONTROL, and it caught a real hole: a rule that emits
    # nothing also reports "the human board passes". Shaving the limit
    # below U30's own worst pin must flag that pin BY NAME, which only
    # a rule doing the measurement can do.
    worst = max(gaps)
    tight = _graded('glasgow_revC',
                    {'max_pin_distance_mm': round(worst - 0.01, 4)})[0]
    named = [v for v in _of(tight, 'decap_pin_distance')
             if v.ref == 'U30']
    assert named, [v.message for v in _of(tight, 'decap_pin_distance')]
    assert abs(max(v.measured['gap_mm'] for v in named) - worst) < 1e-3
    print(f"  PASS: U30's 9 declared pins span {min(gaps):.3f}.."
          f"{max(gaps):.3f}mm and report 0 findings at 3mm -- while a "
          f"limit 0.01mm under the worst flags that pin by name, so the "
          f"pass is a measurement")


def test_same_side_is_OFF_by_default_and_costs_the_reference_board_everything():
    """43% of corpus tethers are opposite-side, and glasgow_revC U30 is on F.Cu
    with ALL of its decaps on B.Cu. Turning `same_side` on there takes the board
    from clean to fully uncovered -- which is why it is opt-in, and why it is a
    MANUFACTURING assertion rather than an electrical one."""
    if not os.path.exists(_board('glasgow_revC')):
        print("  SKIP: glasgow_revC absent")
        return
    off, _p = _graded('glasgow_revC', {'max_pin_distance_mm': 3.0})
    on, _p = _graded('glasgow_revC',
                     {'max_pin_distance_mm': 3.0, 'same_side': True})
    n_off = len(_of(off, 'decap_pin_uncovered'))
    n_on = len(_of(on, 'decap_pin_uncovered'))
    assert n_on > n_off, (n_off, n_on)
    # And the default really is off: an intent that does not mention the key
    # must grade exactly like `same_side: false`.
    same, _p = _graded('glasgow_revC',
                       {'max_pin_distance_mm': 3.0, 'same_side': False})
    assert [v.message for v in _pin(off)] == [v.message for v in _pin(same)]
    print(f"  PASS: same_side off -> {n_off} uncovered rail(s), on -> {n_on}; "
          f"omitting the key grades identically to false")


def test_pin_functions_REPLACES_the_default_table():
    """The failure mode is a DEFAULT entry that is wrong for a board, and an
    add-only override cannot remove one. So a narrow list must SHRINK the pin
    set, and an empty list is refused rather than silently disabling the
    channel."""
    if not os.path.exists(_board(LVDS)):
        print(f"  SKIP: {LVDS} absent")
        return
    pcb = parse_kicad_pcb(_board(LVDS))
    wide = fp.supply_pins(pcb)
    narrow = fp.supply_pins(pcb, pin_functions=['VDD'])   # matches nothing here
    assert sum(len(r['pins']) for r in wide.values()) == 3
    assert all(r['counts']['pinfunction'] == 0 for r in narrow.values()), \
        {k: v['counts'] for k, v in narrow.items()}
    run_utils.check
    try:
        fp.intent_from_dict({'schema': 1, 'kind': 'floorplan-intent',
                             'units': 'mm',
                             'decaps': {'pin_functions': []}})
        raise AssertionError("an empty pin_functions was accepted")
    except fp.IntentError as exc:
        assert 'REPLACES' in str(exc), str(exc)
    print("  PASS: pin_functions replaces the table (VDD-only finds 0 of 3 "
          "here) and an empty list is refused with the reason")


def test_the_evidence_says_HOW_a_clean_pass_was_reached():
    """A clean pass over 39 inferred pins and a clean pass over 9 declared ones
    are different results, and `rules_run` cannot tell them apart."""
    if not (os.path.exists(_board('ulx3s'))
            and os.path.exists(_board('glasgow_revC'))):
        print("  SKIP: fixtures absent")
        return
    u, _p = _graded('ulx3s', {'max_pin_distance_mm': 3.0})
    g, _p = _graded('glasgow_revC', {'max_pin_distance_mm': 3.0})
    assert u.decap_pin_evidence['pins_by_channel']['pintype'] == 0
    assert u.decap_pin_evidence['pins_by_channel']['rail_net'] > 0
    assert g.decap_pin_evidence['pins_by_channel']['pintype'] > 0
    # Every channel is counted for every chip, fired or not -- "channel 1 found
    # 12" and "channel 1 was tried and found none" must not look alike.
    for ev in (u.decap_pin_evidence, g.decap_pin_evidence):
        assert set(ev['by_channel']) == {'pintype', 'pinfunction', 'rail_net'}
        assert ev['chips'] >= ev['chips_graded'] > 0
    # And the chips where the ladder's ORDER decided the answer are named.
    assert g.decap_pin_evidence['order_decided'], g.decap_pin_evidence
    text = fp.format_text(g)
    assert 'decap pins:' in text, text[-500:]
    print(f"  PASS: ulx3s {u.decap_pin_evidence['pins_by_channel']}, "
          f"glasgow {g.decap_pin_evidence['pins_by_channel']}, "
          f"{len(g.decap_pin_evidence['order_decided'])} order-decided chip(s)")


def test_the_corpus_reach_is_measured_and_the_arms_are_all_live():
    """What the rule costs at a 3mm limit, board by board. Both bounds matter:
    an inert rule makes every other arm a fixture artefact, and a rule that
    fires everywhere is noise."""
    tot = {'decap_pin_distance': 0, 'decap_pin_distance_inferred': 0,
           'decap_pin_uncovered': 0}
    ran = skipped = 0
    for path in run_utils.corpus_boards():
        name = os.path.basename(path).replace('.kicad_pcb', '')
        r, _p = _graded(name, {'max_pin_distance_mm': 3.0})
        if 'decap_pin_distance' in r.rules_run:
            ran += 1
        else:
            skipped += 1
            assert r.rules_skipped.get('decap_pin_distance'), name
        for v in _pin(r):
            tot[v.rule] += 1
    assert ran >= 8 and skipped >= 3, (ran, skipped)
    # Every one of the three names must be reachable, or a name is dead code.
    for k, n in tot.items():
        assert n > 0, (k, tot)
    print(f"  PASS: {ran} board(s) graded, {skipped} abstained with a reason; "
          f"declared {tot['decap_pin_distance']}, inferred "
          f"{tot['decap_pin_distance_inferred']}, uncovered "
          f"{tot['decap_pin_uncovered']}")


TESTS = [
    test_the_ladder_falls_through_on_YIELD_not_on_the_field_being_present,
    test_the_limit_is_TIGHT_not_merely_satisfied,
    test_the_measured_payload_names_the_pin_the_cap_and_the_channel,
    test_the_gap_is_measured_to_the_caps_RAIL_pad_not_its_nearest_one,
    test_ground_pins_are_not_graded,
    test_a_board_with_no_supply_pin_ABSTAINS_with_a_reason,
    test_a_board_with_no_capacitor_at_all_abstains_for_THAT_reason,
    test_an_uncovered_rail_is_a_separate_finding_at_WARN,
    test_an_INFERRED_pin_carries_its_own_rule_name_at_warn,
    test_the_human_reference_board_passes_on_its_DECLARED_pins,
    test_same_side_is_OFF_by_default_and_costs_the_reference_board_everything,
    test_pin_functions_REPLACES_the_default_table,
    test_the_evidence_says_HOW_a_clean_pass_was_reached,
    test_the_corpus_reach_is_measured_and_the_arms_are_all_live,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
