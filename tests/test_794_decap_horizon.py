"""#794: a cap past the 5mm horizon was invisible to the emitter AND the rule.

`groups.DECAP_RADIUS_MM` prunes the tether list at 5mm, and both the #704
emitter and `decap_distance` call it at that same truncation. So on
splitflap_driver `--declare-decaps` emits `max_distance_mm 3.4618`,
`decap_distance` RUNS, the verdict is clean -- and C3 sits **19.30mm** from the
IC it shares a rail with. `rules_run` went up and that cap was still ungraded.

WHY A NEW RULE AND NOT A WIDER RADIUS. Measured, widening is closed:

  * widen the RULE only -> the emitter's `ceil(max)` no longer covers the newly
    visible caps, so `test_the_round_trip_grades_clean_AND_runs_the_rule` fails
    on the emitter's own board;
  * widen BOTH -> splitflap's emitted limit goes 3.4618 -> 19.30 and ulx3s's
    4.7149 -> 12.24. The limit stops meaning anything.

`test_no_search_radius_is_written_into_decaps` exists to keep the emitter and
the rule measuring the same population, and it is right to.

WHAT THE RULE CLAIMS. Coverage, not compliance -- which is why it is named
`decap_ungraded` and not `decap_out_of_range`. At 19mm a cap is far likelier a
bulk or filter cap than a failed decoupler, and asserting a violation would
manufacture exactly the finding `_decap_derivation` refuses a percentile limit
for. It says: your limit was derived from the caps inside the radius, so it
says nothing about this one.

WHY IT IS SILENT ON sonde_u, AND WHY THAT IS RIGHT. `sonde_u` has 0 tethers
inside the radius and 5 beyond, so the emitter WITHHOLDS the limit and neither
decap rule arms. The finding is not lost: `_WITHHELD_RULE` routes the
withholding into BOTH rules' skip reasons, and that note names the beyond count
and the worst distance -- it says MORE than this rule could, because it says the
board cannot support a limit at all. Armed on a withheld key the rule could not
even write its message, whose point is "...never measured it against the L mm
limit" and which has no L. `test_every_withholding_board_names_the_horizon`
below is what makes that argument true for all four withholding boards rather
than for the one that happened to have it.
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('', 'py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402
from placement import groups as groups_mod                  # noqa: E402

#: Boards whose emitter derives a limit AND which carry a beyond-cap, so the
#: rule can actually fire. Measured, not guessed.
FIRING = ('splitflap_driver', 'ulx3s', 'glasgow_revC', 'watchy', 'tigard',
          'kit-dev-coldfire-xilinx_5213', 'orangecrab_ext_pll', 'esp_prog',
          'lvds_converter_dualclk')
#: Boards that WITHHOLD the limit while carrying a beyond-cap: the rule cannot
#: arm, and the abstention channel is what must carry the horizon instead.
WITHHOLDING = ('sonde_u', 'interf_u_unrouted', 'flat_hierarchy',
               'interf_u_unrouted_placed')


def _board(name):
    return os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')


def _present(names):
    return [n for n in names if os.path.exists(_board(n))]


def _emit(name, **kw):
    path = _board(name)
    return fp.emit_intent(parse_kicad_pcb(path), path, **kw), path


def _graded(name, mutate=None):
    doc, path = _emit(name, derive_decaps=True)
    if mutate:
        mutate(doc)
    pcb = parse_kicad_pcb(path)
    return fp.grade(fp.intent_from_dict(doc, path), pcb, path), doc, pcb


def _hits(result):
    return [v for v in result.violations if v.rule == 'decap_ungraded']


def test_the_rule_fires_where_the_limit_could_not_see():
    """splitflap_driver is the issue's own case, asserted with its number."""
    if not os.path.exists(_board('splitflap_driver')):
        print("  SKIP: splitflap_driver absent")
        return
    r, doc, _pcb = _graded('splitflap_driver')
    hits = _hits(r)
    assert len(hits) == 1, [v.message for v in hits]
    v = hits[0]
    assert v.ref == 'C3', v.ref
    assert v.measured['distance_mm'] > 19.0, v.measured
    assert v.measured['search_radius_mm'] == 5.0, v.measured
    # The limit is in `expected`, so a reader can see the number the cap was
    # NOT compared against.
    assert v.expected['max_distance_mm'] == doc['decaps']['max_distance_mm']
    # ANTI-VACUITY: decap_distance must have RUN and found nothing, or "the
    # grade was clean" would be satisfied by the rule never arming.
    assert 'decap_distance' in r.rules_run, sorted(r.rules_run)
    assert not [x for x in r.violations if x.rule == 'decap_distance']
    print(f"  PASS: splitflap_driver grades clean on decap_distance at "
          f"{v.expected['max_distance_mm']}mm while decap_ungraded names C3 at "
          f"{v.measured['distance_mm']}mm -- the exact case #794 reports")


def test_it_is_a_WARNING_and_the_board_still_passes_on_it():
    """Loud without being fatal. The severity is the whole design: ten tracked
    boards newly emit this, and on kit-dev it is 12 findings against a limit
    the emitter itself derived and blessed."""
    names = _present(('splitflap_driver', 'watchy', 'tigard'))
    # FLOORED. Without this the loop printed a hardcoded "3 board(s)"
    # however many were actually present, so a corpus that lost two of
    # them would have passed while saying otherwise. Review found it.
    assert len(names) >= 2, names
    for name in names:
        r, _doc, _pcb = _graded(name)
        hits = _hits(r)
        assert hits, name
        assert all(v.severity == fp.WARN for v in hits), name
        # None of them may be in `errors`, which is what the exit code reads.
        assert not [v for v in r.errors if v.rule == 'decap_ungraded'], name
        # `not r.errors`: the claim is "decap_ungraded is a WARN and is not
        # counted", which is about the error list. `passed` additionally
        # requires completeness since #713 item 5, and watchy/tigard withhold
        # `overlap_area`.
        assert not r.errors, (name, [v.message for v in r.errors[:2]])
    print(f"  PASS: {len(names)} board(s) carry decap_ungraded findings "
          f"and still pass -- warn is reported, not counted")


def test_the_severity_is_SETTABLE_and_promoting_it_fails_the_board():
    """`severity_of(..., default=WARN)` and not `ctx.sev`, which hard-defaults
    to ERROR. Both halves are asserted: the default is warn, AND an author who
    has looked can promote it in one key."""
    if not os.path.exists(_board('splitflap_driver')):
        print("  SKIP: splitflap_driver absent")
        return

    def promote(doc):
        doc['severity'] = {'decap_ungraded': 'error'}

    r, _doc, _pcb = _graded('splitflap_driver', mutate=promote)
    hits = _hits(r)
    assert hits and all(v.severity == fp.ERROR for v in hits)
    assert not r.passed
    print("  PASS: default warn, and {'decap_ungraded': 'error'} promotes it "
          "-- the name is settable because it is registered in RULES")


def test_it_names_the_IC_decap_distance_WOULD_have_named():
    """No hedge, and this is why.

    The election is radius-free (`groups._elect_tethers`), so a beyond-cap
    carries the same owner and the same distance `decap_distance` would have
    reported at a wider radius. Asserted against the unbounded pass on every
    firing board rather than argued from the code."""
    checked = 0
    for name in _present(FIRING):
        r, _doc, pcb = _graded(name)
        far = {c: (ic, round(d, 9))
               for ic, v in groups_mod.decap_tethers(
                   pcb, radius=float('inf')).items() for c, d in v}
        for v in _hits(r):
            ic, d = far[v.ref]
            assert v.measured['ic'] == ic, (name, v.ref, v.measured['ic'], ic)
            assert abs(v.measured['distance_mm'] - round(d, 4)) < 1e-9, (
                name, v.ref)
            checked += 1
    assert checked >= 20, checked
    print(f"  PASS: {checked} finding(s) name the IC and the distance the "
          f"unbounded pass elects -- identical, so the message needs no hedge")


def test_a_cap_with_NO_rail_carrying_chip_is_not_reported():
    """`orphans` are not `beyond`. A cap whose rail no chip carries has no IC
    to be far from, so naming one would be an invention -- and ulx3s has TEN of
    them (bulk and filter caps upstream of an LC network) against 7 beyond."""
    if not os.path.exists(_board('ulx3s')):
        print("  SKIP: ulx3s absent")
        return
    r, _doc, pcb = _graded('ulx3s')
    _near, beyond, orphans = groups_mod.decap_populations(pcb)
    assert len(orphans) == 10 and len(beyond) == 7, (len(orphans), len(beyond))
    refs = {v.ref for v in _hits(r)}
    assert refs == {c for c, _ic, _d in beyond}, sorted(refs)
    assert not (refs & set(orphans)), sorted(refs & set(orphans))
    print(f"  PASS: ulx3s reports the 7 beyond-radius caps and none of the 10 "
          f"with no rail-carrying chip ({', '.join(sorted(orphans)[:4])}, ...)")


def test_an_exempted_cap_is_not_reported_as_ungraded_either():
    """An author who waived a cap from the distance claim has decided about it;
    telling them it is also ungraded is noise about their own decision."""
    if not os.path.exists(_board('ulx3s')):
        print("  SKIP: ulx3s absent")
        return
    r0, _d, _p = _graded('ulx3s')
    victim = sorted(v.ref for v in _hits(r0))[0]

    def waive(doc):
        doc['decaps']['exempt'] = [victim]

    r1, _d, _p = _graded('ulx3s', mutate=waive)
    after = {v.ref for v in _hits(r1)}
    assert victim not in after, victim
    assert len(after) == len(_hits(r0)) - 1, (len(after), len(_hits(r0)))
    print(f"  PASS: exempting {victim} removes exactly one finding, not zero "
          f"and not all of them")


def test_both_decap_rules_partition_at_the_SAME_radius():
    """`search_radius_mm` must reach both rules or the two halves of one
    population are measured at different horizons -- and the gap between them
    would be reported by neither."""
    if not os.path.exists(_board('ulx3s')):
        print("  SKIP: ulx3s absent")
        return

    def widen(doc):
        doc['decaps']['search_radius_mm'] = 8.0

    r, _doc, pcb = _graded('ulx3s', mutate=widen)
    near, beyond, _o = groups_mod.decap_populations(pcb, radius=8.0)
    assert {v.ref for v in _hits(r)} == {c for c, _i, _d in beyond}
    # And the caps that moved INSIDE the wider radius are now decap_distance's
    # to grade: at the 5mm limit they exceed it, so they must appear there.
    moved = [v for v in r.violations if v.rule == 'decap_distance']
    assert moved, "widening the radius moved caps into decap_distance's scope"
    graded_refs = {c for caps in near.values() for c, _d in caps}
    assert {v.ref for v in moved} <= graded_refs
    print(f"  PASS: at search_radius_mm 8.0 the split moves together -- "
          f"{len(_hits(r))} ungraded, {len(moved)} now graded and flagged")


def test_every_withholding_board_names_the_horizon_in_its_abstention():
    """The argument that makes "the rule is silent on sonde_u" acceptable.

    The zero-tether withholding arm always carried the beyond-population
    clause. The DECAP_MIN_SAMPLE arm did not -- so `interf_u_unrouted`,
    withheld for having one sample while carrying FOUR caps beyond the radius
    and a 19.82mm worst, abstained with a note that said nothing about them.
    A channel that covers one withholding reason and not the others only looks
    complete, and this is the arm that keeps it honest."""
    checked = 0
    for name in _present(WITHHOLDING):
        doc, path = _emit(name, derive_decaps=True)
        cen = doc['context']['decap_census']
        why = (doc['context'].get('budget_withheld') or {}).get(
            'decaps.max_distance_mm')
        assert why, (name, "expected a withholding")
        assert cen['beyond_radius'] >= 1, (name, cen)
        assert 'beyond' in why, (name, why)
        assert str(cen['worst_beyond_mm']) in why, (name, why)
        # And it must reach the GRADE, not only the emitted document.
        r = fp.grade(fp.intent_from_dict(doc, path), parse_kicad_pcb(path),
                     path)
        for rule in ('decap_distance', 'decap_ungraded'):
            assert rule in r.rules_skipped, (name, rule, sorted(r.rules_skipped))
            assert 'WITHHELD' in r.rules_skipped[rule], (name, rule)
            assert 'beyond' in r.rules_skipped[rule], (name, rule)
        checked += 1
    assert checked >= 3, checked
    print(f"  PASS: {checked} withholding board(s) name the beyond-population "
          f"and its worst distance, in the emitted note AND in BOTH rules' "
          f"skip reasons")


def test_the_withholding_disarms_BOTH_rules_not_just_the_first():
    """`_WITHHELD_RULE` mapped a key to ONE rule name. One key now disarms two,
    and a one-rule mapping would have left the second reading a bare skip
    reason that does not say the emitter ever tried."""
    entry = fp._WITHHELD_RULE['decaps.max_distance_mm']
    assert entry[0] == ('decap_distance', 'decap_ungraded'), entry[0]
    for key in ('overlap_area', 'oob_count', 'oob_amount'):
        assert isinstance(fp._WITHHELD_RULE[key][0], tuple), key
    print("  PASS: the withheld key names both rules it disarms, and every "
          "entry carries a tuple so the reader has one shape to handle")


def test_the_corpus_reach_is_measured_and_bounded():
    """What declaring `max_distance_mm` now costs an author, board by board.

    Both bounds matter. If NO board fires, the rule is inert and every other
    arm here is a fixture artefact. If EVERY board fires, it is noise. And no
    board may gain an ERROR from it."""
    fired = []
    total = 0
    for path in run_utils.corpus_boards():
        name = os.path.basename(path).replace('.kicad_pcb', '')
        pcb = parse_kicad_pcb(path)
        doc = fp.emit_intent(pcb, path, derive_decaps=True)
        if (doc.get('decaps') or {}).get('max_distance_mm') is None:
            continue
        r = fp.grade(fp.intent_from_dict(doc, path), pcb, path)
        n = len(_hits(r))
        assert not [v for v in r.errors if v.rule == 'decap_ungraded'], name
        if n:
            fired.append((name, n))
            total += n
    assert fired, "the rule is inert on the whole corpus"
    assert len(fired) < len(run_utils.corpus_boards()), \
        "every board fires -- that is noise, not a finding"
    print(f"  PASS: {len(fired)} board(s), {total} finding(s), 0 errors -- "
          + ', '.join(f'{n}:{c}' for n, c in fired))


def test_the_cli_shows_it_to_an_author_and_still_exits_zero():
    """Reasoning about the output is not the same as reading it."""
    if not os.path.exists(_board('splitflap_driver')):
        print("  SKIP: splitflap_driver absent")
        return
    with tempfile.TemporaryDirectory() as wd:
        out = os.path.join(wd, 'i.json')
        run_utils.check(
            [sys.executable, '-X', 'utf8',
             run_utils.tool('check_floorplan.py'), _board('splitflap_driver'),
             '--emit-intent', out, '--declare-decaps'], accept=True)
        run_utils.evidence(out, 'the emitted intent')
        r = run_utils.check(
            [sys.executable, '-X', 'utf8',
             run_utils.tool('check_floorplan.py'), _board('splitflap_driver'),
             '--intent', out], accept=True)
        blob = r.stdout + r.stderr
        assert 'decap_ungraded' in blob, blob[-1200:]
        assert '19.30mm' in blob, blob[-1200:]
        assert '[warn ]' in blob, blob[-1200:]
    print("  PASS: the CLI prints the finding at warn and exits 0 -- an author "
          "sees the 19.30mm cap without the grade failing")


TESTS = [
    test_the_rule_fires_where_the_limit_could_not_see,
    test_it_is_a_WARNING_and_the_board_still_passes_on_it,
    test_the_severity_is_SETTABLE_and_promoting_it_fails_the_board,
    test_it_names_the_IC_decap_distance_WOULD_have_named,
    test_a_cap_with_NO_rail_carrying_chip_is_not_reported,
    test_an_exempted_cap_is_not_reported_as_ungraded_either,
    test_both_decap_rules_partition_at_the_SAME_radius,
    test_every_withholding_board_names_the_horizon_in_its_abstention,
    test_the_withholding_disarms_BOTH_rules_not_just_the_first,
    test_the_corpus_reach_is_measured_and_bounded,
    test_the_cli_shows_it_to_an_author_and_still_exits_zero,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
