"""Board-state gates and the lock advisor (#431).

Two gates that stop a placement tool producing a result-shaped non-result, and
an advisor that says which parts should not be moved -- and never moves them.

The gates' CLIs here are the live callers of `placement.placement_state`:
`check_floorplan.py` refuses a pile (exit 3, `--allow-unplaced` overrides) and
`place_pose.py` refuses a routed board (exit 3, `--allow-routed` overrides).
The CLIs this file first drove (`place_optimize`, `place_route_loop`, and their
`--suggest-locks` flag) were retired in ecd673eb; the advisor's remaining
consumer is `board_brief` (`measurements.locks`, tests/test_board_brief.py).

The advisor's strongest rule is a CODE fact, not folklore. `quench.py:207`
keeps only `net_id > 0` pads, so a net-less NPTH mounting hole has
`pin_count == 0` and no airwires; the only skip in the part loop is
`if not fp.pads`, and a mounting hole HAS a pad. It is therefore movable with
nothing opposing it but the halo term. `tigard.kicad_pcb` ships four unlocked
M3 holes in exactly that state.

Assertions name REFS, not counts. A count drifts the moment a threshold is
tuned; a named ref is a contract.
"""

import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

# #522 reorg + skill merge: engine -> py_router/, placer -> py_placer/,
# board_score.py -> py_tools/. Without these roots the
# imports below raise and the test never runs (it reports as a failure while
# asserting nothing).
_R522 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p522 in ('py_router', 'py_placer',
              'py_tools'):
    _d522 = os.path.join(_R522, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)


from kicad_parser import parse_kicad_pcb  # noqa: E402
import run_utils  # noqa: E402
from placement.lock_advisor import advise_locks, to_json  # noqa: E402
from placement.placement_state import (UNPLACED_EXIT, assess_placement)  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(ROOT, 'kicad_files')
TIGARD = os.path.join(KF, 'tigard.kicad_pcb')
SMALL = os.path.join(KF, 'lvds_converter_dualclk.kicad_pcb')
ROUTED = os.path.join(KF, 'routed_output.kicad_pcb')


def _argv(script, *args):
    """`run_utils.tool` RAISES on a missing CLI, so a moved tool fails here,
    naming itself, rather than as an exit code a gate assertion misreads."""
    return [sys.executable, '-X', 'utf8', run_utils.tool(script)] + list(args)


def _pile(dst):
    """A board with every footprint stacked at one coordinate, built from a
    tracked board via the shipped writer -- reproducible, no new fixture."""
    pcb = parse_kicad_pcb(SMALL)
    bb = pcb.board_info.board_bounds
    cx, cy = (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2
    write_placed_output(SMALL, dst, [{'reference': r, 'new_x': cx, 'new_y': cy,
                                      'new_rotation': 0.0}
                                     for r in pcb.footprints])
    return dst


# --- unplaced detector -------------------------------------------------------

def test_no_tracked_board_reads_as_unplaced():
    """The false-positive gate, and the half that matters most. `watchy` is the
    documented worst case -- 81 of its 82 parts are in COURTYARD VIOLATION -- and
    it must still read as placed. Density is not unplacedness."""
    import glob
    bad = []
    n = 0
    for f in sorted(glob.glob(os.path.join(KF, '*.kicad_pcb'))):
        try:
            pcb = parse_kicad_pcb(f)
        except Exception:
            continue
        n += 1
        if assess_placement(pcb, f).unplaced:
            bad.append(os.path.basename(f))
    assert n >= 20, f"only checked {n} boards"
    assert not bad, (f"false positives on {bad} -- watchy.kicad_pcb in particular "
                     f"is dense and courtyard-violating but IS placed")
    print(f"  PASS: {n} tracked boards, 0 false positives")


def test_a_pile_at_one_coordinate_is_unplaced():
    d = tempfile.mkdtemp()
    try:
        pcb = parse_kicad_pcb(_pile(os.path.join(d, 'pile.kicad_pcb')))
        st = assess_placement(pcb, os.path.join(d, 'pile.kicad_pcb'))
        assert st.unplaced
        assert st.distinct_positions == 1
        assert st.duplicate_fraction == 1.0
        assert any('share a position' in r for r in st.reasons)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_missing_outline_cannot_manufacture_an_unplaced_verdict():
    """`outside_fraction` is None = UNAVAILABLE, never 0.0 and never True. A
    misparsed outline must not be able to condemn a perfectly placed board."""
    pcb = parse_kicad_pcb(SMALL)
    pcb.board_info.board_bounds = None
    st = assess_placement(pcb, None)
    assert st.outside_fraction is None
    assert st.spread_ratio is None
    assert st.unplaced is False


def test_partially_unplaced_is_reported_but_not_blocking():
    """A netlist re-import drops NEW parts at one spot on an otherwise-placed
    board. Worth saying; not worth refusing over."""
    d = tempfile.mkdtemp()
    try:
        pcb = parse_kicad_pcb(SMALL)
        refs = sorted(pcb.footprints)[:3]
        bb = pcb.board_info.board_bounds
        out = os.path.join(d, 'partial.kicad_pcb')
        write_placed_output(SMALL, out,
                            [{'reference': r, 'new_x': bb[0] + 1.0,
                              'new_y': bb[1] + 1.0, 'new_rotation': 0.0}
                             for r in refs])
        st = assess_placement(parse_kicad_pcb(out), out)
        assert st.unplaced is False
        assert st.partially_unplaced is True
        assert set(refs) <= set(st.stacked_refs)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_routed_board_is_detected():
    """write_placed_output rewrites footprint positions only, so placement on
    a routed board strands every track."""
    st = assess_placement(parse_kicad_pcb(ROUTED), ROUTED)
    assert st.has_copper and st.segments > 0
    assert st.blocked


# --- the CLI contract --------------------------------------------------------

def test_the_unplaced_gate_refuses_with_exit_3_and_writes_nothing():
    d = tempfile.mkdtemp()
    try:
        pile = run_utils.evidence(_pile(os.path.join(d, 'pile.kicad_pcb')),
                                  'the pile board')
        out = os.path.join(d, 'intent.json')
        run_utils.check(_argv('check_floorplan.py', pile, '--emit-intent', out),
                        refuse='does not look PLACED', code=UNPLACED_EXIT)
        assert not os.path.exists(out), "refused but still wrote an intent"
        # ...and the override is honoured, on the same pile.
        run_utils.check(_argv('check_floorplan.py', pile, '--emit-intent', out,
                              '--allow-unplaced'), accept=True)
        run_utils.evidence(out, 'the intent --allow-unplaced emitted')
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_routed_gate_and_its_override():
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'out.kicad_pcb')
        run_utils.check(_argv('place_pose.py', ROUTED, out, 'lock', 'U1'),
                        refuse='strands every track', code=UNPLACED_EXIT)
        assert not os.path.exists(out), "refused but still wrote a board"
        run_utils.check(_argv('place_pose.py', ROUTED, out, '--allow-routed',
                              'lock', 'U1'), accept=True)
        run_utils.evidence(out, 'the board --allow-routed wrote')
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_normal_board_is_unaffected_by_the_gates():
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'intent.json')
        r = run_utils.check(_argv('check_floorplan.py', SMALL,
                                  '--emit-intent', out), accept=True)
        assert 'does not look PLACED' not in r.stdout + r.stderr
        run_utils.evidence(out, 'the intent of a placed board')
    finally:
        shutil.rmtree(d, ignore_errors=True)


# --- lock advisor ------------------------------------------------------------

def test_mounting_holes_are_found_by_name_not_by_count():
    """tigard's four M3 holes: NPTH, zero connected pins, unlocked. The rule
    that catches them is verifiable in quench.py, not a naming guess."""
    adv = advise_locks(parse_kicad_pcb(TIGARD), TIGARD)
    by_ref = {f.ref: f for f in adv.findings}
    for ref in ('H1', 'H2', 'H3', 'H4'):
        assert ref in by_ref, f"{ref} not flagged"
        f = by_ref[ref]
        assert f.confidence == 'high'
        assert 'mounting_hole_npth' in f.rules
        assert f.evidence['connected_pins'] == 0
        assert f.evidence['npth_pads'] >= 1 and f.evidence['plated_pads'] == 0


def test_an_overhanging_connector_is_high_confidence():
    """J1 is tigard's USB-C receptacle; its body leaves the outline. That is a
    geometric fact, and it is the 'HAT port' case this advisor exists for."""
    adv = advise_locks(parse_kicad_pcb(TIGARD), TIGARD)
    j1 = next(f for f in adv.findings if f.ref == 'J1')
    assert j1.confidence == 'high'
    assert 'off_board_overhang' in j1.rules
    assert j1.evidence['outside_amount_mm'] > 0


def test_a_plated_through_part_is_not_a_mounting_hole():
    """Guards the `pad_is_plated_through` vs bare `drill > 0` confusion
    CLAUDE.md calls out (#328): a plated THT connector is not a mounting hole."""
    adv = advise_locks(parse_kicad_pcb(TIGARD), TIGARD)
    for f in adv.findings:
        if 'mounting_hole_npth' in f.rules:
            assert f.evidence['plated_pads'] == 0, \
                f"{f.ref} has plated pads but was called a mounting hole"


def test_ordinary_passives_are_not_flagged():
    """The false-positive guard. A mid-board 0402 must produce nothing."""
    pcb = parse_kicad_pcb(TIGARD)
    adv = advise_locks(pcb, TIGARD)
    flagged = {f.ref for f in adv.findings}
    passives = [r for r in pcb.footprints
                if r[:1] in ('R', 'C') and r not in flagged]
    assert len(passives) > 10, "expected many unflagged passives"


def test_the_round_trip_closes():
    """Feed the printed --lock back in and every high finding must report as
    covered. This is the design's own acceptance test."""
    adv = advise_locks(parse_kicad_pcb(TIGARD), TIGARD)
    pats = adv.lock_patterns()
    assert pats, "nothing suggested"
    again = advise_locks(parse_kicad_pcb(TIGARD), TIGARD, lock_patterns=pats)
    assert again.tally()['unlocked_high'] == 0, again.tally()
    assert all(f.lock_source == 'cli'
               for f in again.findings if f.ref in pats)


def test_already_locked_copper_is_recognised():
    """tigard ships one `(locked yes)` footprint. It must read as locked, from
    the FILE rather than from a --lock pattern."""
    adv = advise_locks(parse_kicad_pcb(TIGARD), TIGARD)
    assert adv.locked_refs, "no (locked yes) footprint found"
    assert adv.tally()['locked_in_file'] == len(adv.locked_refs)


def test_high_pin_parts_are_advisory_not_locked():
    """Worth reporting -- but 'large' is not 'position-critical', so it must
    not enter the lock list."""
    adv = advise_locks(parse_kicad_pcb(TIGARD), TIGARD)
    assert adv.advisories, "expected a high-pin-count advisory on tigard"
    advised = {a['ref'] for a in adv.advisories}
    assert not (advised & set(adv.lock_patterns()))


def test_advice_is_deterministic():
    a = json.dumps(to_json(advise_locks(parse_kicad_pcb(TIGARD), TIGARD)), sort_keys=True)
    b = json.dumps(to_json(advise_locks(parse_kicad_pcb(TIGARD), TIGARD)), sort_keys=True)
    assert a == b


TESTS = [
    test_no_tracked_board_reads_as_unplaced,
    test_a_pile_at_one_coordinate_is_unplaced,
    test_missing_outline_cannot_manufacture_an_unplaced_verdict,
    test_partially_unplaced_is_reported_but_not_blocking,
    test_routed_board_is_detected,
    test_the_unplaced_gate_refuses_with_exit_3_and_writes_nothing,
    test_routed_gate_and_its_override,
    test_a_normal_board_is_unaffected_by_the_gates,
    test_mounting_holes_are_found_by_name_not_by_count,
    test_an_overhanging_connector_is_high_confidence,
    test_a_plated_through_part_is_not_a_mounting_hole,
    test_ordinary_passives_are_not_flagged,
    test_the_round_trip_closes,
    test_already_locked_copper_is_recognised,
    test_high_pin_parts_are_advisory_not_locked,
    test_advice_is_deterministic,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
