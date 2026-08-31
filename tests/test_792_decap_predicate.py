"""#792's premise, measured rather than asserted: the predicates never disagree.

Issue #792 says two predicates decide "is this a decap" and disagree in both
directions, and it publishes a table -- ulx3s 70 in the seeder's scope, 53
graded, 17 "in scope, never graded" -- as the evidence. There are actually
THREE spellings, and the table is not evidence for any of them:

    groups.decap_tethers      ref[:1].upper() == 'C', exactly two distinct NETS
    seeder.decap_scope        r[0] == 'C' (case-SENSITIVE), two net-bearing PADS
    floorplan.emit_intent     ref[:1].upper() == 'C', two net-bearing PADS

Measured here over every git-tracked board: **all three name the same parts, on
every board, and the corpus has no lowercase `c*` reference at all.** The 17 is
the 5 mm radius prune plus "no chip carries this cap's rail" -- which is why
`docs/floorplan-intent.md` was wrong to explain it with the predicates, and why
this file exists as the standing measurement rather than a sentence.

WHAT MAKES "GREEN" MEAN SOMETHING HERE. A test that compares three predicates
can pass by looking at nothing -- a fixture list that shrinks to boards with no
capacitors satisfies every equality in it. So each arm asserts SET EQUALITY with
the symmetric difference in the message, and every arm is floored: at least 15
boards, at least 300 in-scope caps, and at least one board carrying 20+. A
mutation that narrows the board list fails on the floor rather than passing
quietly.

The retired seeder spelling is kept alive HERE, in the test, precisely because
#792 deletes it from the engine. This is the only place it still exists, and it
is the change detector that would notice if the corpus ever grew a board on
which the spellings diverge.
"""
import os
import sys

RUN_ALL_FAST_OK = True

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('', 'py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import groups as G                           # noqa: E402

#: Floors, well under today's counts, so an arm cannot pass by looking at less.
MIN_BOARDS = 15
MIN_SCOPE = 300
MIN_ON_ONE_BOARD = 20


def _boards():
    paths = run_utils.corpus_boards()
    if not paths:
        raise AssertionError(
            "corpus_boards() returned nothing -- git could not answer. This "
            "test grades a fixed set; it must not grade an unidentifiable one")
    return paths


def _parsed():
    for path in _boards():
        try:
            yield os.path.basename(path), parse_kicad_pcb(path)
        except Exception as exc:                            # noqa: BLE001
            raise AssertionError(f"{path}: {exc}")


def _grouper(pcb):
    """`groups.decap_tethers`' spelling: two distinct NET IDS, case-blind."""
    return {r for r, fp in pcb.footprints.items()
            if r[:1].upper() == 'C'
            and len({p.net_id for p in fp.pads if p.net_id > 0}) == 2}


def _seeder(pcb):
    """The RETIRED `seeder.decap_scope` spelling: two net-bearing PADS, and
    case-SENSITIVE. Deleted from the engine by #792; kept here as the arm."""
    return {r for r, fp in pcb.footprints.items()
            if r[0:1] == 'C'
            and len([p for p in fp.pads if p.net_id > 0]) == 2}


def _emitter(pcb):
    """The RETIRED `emit_intent._scope` hybrid: pads, but case-blind."""
    return {r for r, fp in pcb.footprints.items()
            if r[:1].upper() == 'C'
            and len([p for p in fp.pads if p.net_id > 0]) == 2}


def test_the_three_decap_predicates_named_the_same_parts():
    n_boards = 0
    total = 0
    widest = 0
    for name, pcb in _parsed():
        n_boards += 1
        a, b, c = _grouper(pcb), _seeder(pcb), _emitter(pcb)
        assert a == b, (f"{name}: grouper vs seeder differ -- "
                        f"{sorted(a ^ b)}")
        assert a == c, (f"{name}: grouper vs emitter differ -- "
                        f"{sorted(a ^ c)}")
        # The engine's own predicate must agree with all three, or the
        # extraction changed the answer it was supposed to preserve.
        live = {r for r, fp in pcb.footprints.items()
                if G.is_decoupling_cap(fp, r)}
        assert live == a, f"{name}: is_decoupling_cap differs -- {sorted(live ^ a)}"
        total += len(a)
        widest = max(widest, len(a))
    assert n_boards >= MIN_BOARDS, n_boards
    assert total >= MIN_SCOPE, total
    assert widest >= MIN_ON_ONE_BOARD, widest
    print(f"  PASS: 3 retired spellings + the live predicate name the same "
          f"parts on {n_boards} board(s), {total} caps, widest {widest}")


def test_no_tracked_board_has_a_lowercase_capacitor_reference():
    """The one difference between the three spellings, and it is unreachable.

    Stated as its own measurement rather than folded into the arm above,
    because "the predicates agree" and "the case rule is untested" are
    different facts and only one of them is a reason to relax."""
    lower = []
    for name, pcb in _parsed():
        lower += [f"{name}:{r}" for r in pcb.footprints if r[0:1] == 'c']
    assert not lower, lower
    print("  PASS: 0 lowercase c* refs, so the case difference is argued, "
          "not measured -- said out loud rather than implied by a green arm")


def test_the_populations_partition_the_scope():
    """near + beyond + orphans is exactly the in-scope set, on every board."""
    n = 0
    tot_near = tot_beyond = tot_orph = 0
    for name, pcb in _parsed():
        near, beyond, orphans = G.decap_populations(pcb)
        flat = [(ic, c) for ic, v in near.items() for c, _d in v]
        scope = _grouper(pcb)
        got = ({c for _ic, c in flat} | {c for c, _ic, _d in beyond}
               | set(orphans))
        assert got == scope, f"{name}: partition != scope -- {sorted(got ^ scope)}"
        assert len(flat) + len(beyond) + len(orphans) == len(scope), (
            f"{name}: a cap appears in two arms")
        tot_near += len(flat)
        tot_beyond += len(beyond)
        tot_orph += len(orphans)
        n += 1
    # ANTI-VACUITY on each arm: a partition whose second and third parts are
    # always empty is a partition nobody tested.
    assert tot_beyond > 0 and tot_orph > 0, (tot_beyond, tot_orph)
    print(f"  PASS: {n} board(s) partition exactly -- near {tot_near}, "
          f"beyond {tot_beyond}, orphans {tot_orph}")


def test_decap_tethers_is_exactly_the_near_population():
    """`decap_populations` may not change what six existing callers see."""
    n = 0
    for name, pcb in _parsed():
        near, _b, _o = G.decap_populations(pcb)
        assert G.decap_tethers(pcb) == near, name
        n += 1
    print(f"  PASS: decap_tethers == near on {n} board(s) -- the six callers "
          f"of the old signature, including --group-by decap, cannot move")


def test_the_unbounded_pass_is_a_SUPERSET_with_the_same_ICs():
    """`decap_census` claimed the opposite. It was wrong, and this pins it.

    The comment said the unbounded pass "can pick a DIFFERENT chip, so this is
    a second measurement rather than a superset". `radius` never reaches the
    election -- it is a prune on the winner -- so the wider answer contains the
    narrower one with byte-identical ICs and distances. Asserted on triples,
    not on counts: equal counts with swapped owners would pass a count test.
    """
    n = shared = 0
    for name, pcb in _parsed():
        near = {(ic, c, round(d, 9))
                for ic, v in G.decap_tethers(pcb).items() for c, d in v}
        far = {(ic, c, round(d, 9))
               for ic, v in G.decap_tethers(pcb, radius=float('inf')).items()
               for c, d in v}
        assert near <= far, f"{name}: {sorted(near - far)}"
        shared += len(near)
        n += 1
    assert shared >= MIN_SCOPE - 100, shared
    print(f"  PASS: near is a subset of far on {n} board(s), {shared} tethers, "
          f"0 re-elected -- guaranteed by one election, not by the corpus")


def test_the_lifted_power_pin_predicate_did_not_change_any_pad():
    """The `analyze_power_paths` lift, proved on every pad of every board.

    The old form tested `pintype in ('power_in', 'power_out')`; the new one
    splits on '+' and rejects `no_connect`. They differ only on a compound
    pintype that is not `X+no_connect`, and the corpus has none -- so this is
    the assertion that the lift was a MOVE and not an edit.
    """
    from net_queries import is_power_pin
    kw = ('VCC', 'VDD', 'VSS', 'GND', 'VCCA', 'VSSA', 'VDDA', 'VDDPLL',
          'VCCPLL', 'GNDPLL', 'VRH', 'VRL', 'AVDD', 'AVSS')

    def old(pinfunction, pintype):
        if pintype in ('power_in', 'power_out'):
            return True
        if pinfunction:
            up = pinfunction.upper()
            return any(up == k or up.startswith(k) for k in kw)
        return False

    pads = fired = 0
    diffs = []
    for name, pcb in _parsed():
        for ref, fp in pcb.footprints.items():
            for p in fp.pads:
                pads += 1
                pt = getattr(p, 'pintype', '') or ''
                pf = getattr(p, 'pinfunction', '') or ''
                o, n = old(pf, pt), is_power_pin(pf, pt)
                fired += bool(o)
                if o != n:
                    diffs.append((name, ref, p.pad_number, pt, pf, o, n))
    assert not diffs, diffs[:5]
    assert pads >= 8000, pads
    # ANTI-VACUITY: a predicate that is False everywhere agrees with anything.
    assert fired >= 1000, fired
    print(f"  PASS: {pads} pads, identical on every one; the predicate fires "
          f"on {fired} of them, so the agreement is not vacuous")


def test_a_compound_no_connect_pintype_is_refused_by_name():
    """`glasgow_revC` U30 pad B10 is `power_in+no_connect`, and it is the whole
    difference between "12 of 12 covered" and "1 uncovered" on the cleanest
    board in the corpus. Named, because an aggregate does not notice 3 pads in
    9488 -- and because BOTH readings of it are wrong for different reasons:
    equality drops it as a side effect, `startswith` grades a pin the designer
    marked no-connect."""
    from net_queries import is_supply_pintype
    assert is_supply_pintype('power_in') is True
    assert is_supply_pintype('power_out') is True
    assert is_supply_pintype('power_in+no_connect') is False
    assert is_supply_pintype('passive+no_connect') is False
    assert is_supply_pintype('passive') is False
    assert is_supply_pintype('') is False

    board = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
    if not os.path.exists(board):
        print("  SKIP: glasgow_revC absent")
        return
    pcb = parse_kicad_pcb(board)
    pad = next((p for p in pcb.footprints['U30'].pads
                if p.pad_number == 'B10'), None)
    assert pad is not None, "U30 has no pad B10"
    assert (getattr(pad, 'pintype', '') or '') == 'power_in+no_connect', (
        f"the fixture moved: U30.B10 is now "
        f"{getattr(pad, 'pintype', '')!r}. Re-measure before relaxing this")
    assert is_supply_pintype(pad.pintype) is False

    # THE GENERAL PROPERTY, asserted because the corpus cannot show it.
    # A mutation replacing the token split with `pintype in ('power_in',
    # 'power_out')` SURVIVED the battery: measured, the two forms agree
    # on all 9488 corpus pads, because every compound spelling here is
    # `X+no_connect` and both forms reject those. So the case for token
    # splitting is a claim about spellings KiCad can emit and this
    # corpus does not contain, and it has to be asserted directly or it
    # is not asserted at all.
    assert is_supply_pintype('power_in+standby') is True, (
        'a compound that is not +no_connect must still be a supply pin; '
        'equality-matching would drop it')
    assert is_supply_pintype('power_out+standby') is True
    assert is_supply_pintype('no_connect+power_in') is False, (
        'token order must not matter')
    print("  PASS: U30.B10 power_in+no_connect is refused by name, and "
          "a compound that is NOT +no_connect is still a supply pin -- "
          "the case equality-matching gets wrong and no board shows")


def test_an_unconnected_net_is_refused_even_when_the_pad_says_power_in():
    """The `unconnected-` filter guards 220 REAL pads and nothing tested it.

    A battery row deleting the filter SURVIVED, because the only boards that
    carry such pads -- `haasoscope_pro_max_test` and `routed_output` -- have no
    capacitors at all, so the pin rule abstains on them and the filter's effect
    is invisible to every end-to-end arm. Measured: 220 pads across the two,
    including `U3` pads on `unconnected-(U3H-VDDQ_PHY-...)`, `...-VCC-...` and
    `...-VCCIO*-...`, every one typed `power_in`.

    Asserted at the PREDICATE, not through a grade, so it does not depend on
    the board being gradeable.
    """
    board = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
    if not os.path.exists(board):
        print("  SKIP: haasoscope_pro_max_test absent")
        return
    pcb = parse_kicad_pcb(board)
    from placement import floorplan as fp
    from net_queries import is_supply_pintype
    victims = [(ref, q) for ref, f in pcb.footprints.items() for q in f.pads
               if is_supply_pintype(getattr(q, 'pintype', '') or '')
               and ((pcb.nets[q.net_id].name if q.net_id in pcb.nets else '')
                    or '').startswith('unconnected-')]
    assert len(victims) >= 100, len(victims)
    recs = fp.supply_pins(pcb)
    graded = {(ref, id(q)) for ref, r in recs.items() for q, _n in r['pins']}
    leaked = [(ref, q.pad_number) for ref, q in victims
              if (ref, id(q)) in graded]
    assert not leaked, leaked[:5]
    # ANTI-VACUITY: the refusal must come from the NET, not from the chip being
    # out of scope -- so at least one victim's chip must be a candidate.
    assert any(ref in recs for ref, _q in victims), (
        'no victim sits on a candidate IC, so this arm proves nothing')
    print(f"  PASS: {len(victims)} power_in pad(s) on unconnected- nets, none "
          f"graded, and their chips ARE candidates")


def test_a_two_terminal_passive_is_not_an_IC_however_it_is_rotated():
    """`chip_refs` counted PASTE apertures toward the four-pad gate.

    A KiCad 0201 footprint carries two solder-paste apertures beside its two
    copper pads, so it reaches four and `build_chip_list` calls it a chip. The
    collinearity guard does not save it: at a non-orthogonal angle its two
    copper pads share neither an x nor a y, so the row test says "not a row".
    The old behaviour was therefore ROTATION-DEPENDENT -- `orangecrab C1` is
    the same four-pad shape and escaped only because it sits at 90 degrees.

    Found by review, and not theoretical: the pin rule graded `rp2350 C28`
    (Capacitor_SMD:C_0201_0603Metric at -45) as an IC needing decoupling and
    reported it 6.11mm from C30 -- a capacitor flagged for being far from
    another capacitor.
    """
    board = os.path.join(ROOT, 'kicad_files',
                         'rp2350_fpga_eensy_prePlane.kicad_pcb')
    if not os.path.exists(board):
        print("  SKIP: rp2350 absent")
        return
    pcb = parse_kicad_pcb(board)
    chips = G.chip_refs(pcb)
    for ref in ('C28', 'R9'):
        assert ref in pcb.footprints, ref
        f = pcb.footprints[ref]
        # The trap must still be ARMED: four raw pads, two copper, tilted.
        assert len(f.pads) >= 4, (ref, len(f.pads))
        assert G._copper_pads(f) == 2, (ref, G._copper_pads(f))
        assert round(f.rotation) % 90 != 0, (ref, f.rotation)
        assert not G._pads_are_collinear(f), (
            ref, 'the row test alone would have caught it, so this arm proves '
                 'nothing about the copper-pad gate')
        assert ref not in chips, ref
    # And no capacitor anywhere in the corpus is called an IC.
    bad = []
    for path in run_utils.corpus_boards():
        p2 = parse_kicad_pcb(path)
        bad += [(os.path.basename(path), r) for r in G.chip_refs(p2)
                if G._copper_pads(p2.footprints[r]) < 4]
    assert not bad, bad[:5]
    print("  PASS: rp2350 C28 and R9 are 4-pad/2-copper parts at -45 degrees, "
          "not collinear, and not chips; no corpus part reaches the chip list "
          "on paste apertures")


TESTS = [
    test_the_three_decap_predicates_named_the_same_parts,
    test_no_tracked_board_has_a_lowercase_capacitor_reference,
    test_the_populations_partition_the_scope,
    test_decap_tethers_is_exactly_the_near_population,
    test_the_unbounded_pass_is_a_SUPERSET_with_the_same_ICs,
    test_the_lifted_power_pin_predicate_did_not_change_any_pad,
    test_a_compound_no_connect_pintype_is_refused_by_name,
    test_an_unconnected_net_is_refused_even_when_the_pad_says_power_in,
    test_a_two_terminal_passive_is_not_an_IC_however_it_is_rotated,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
