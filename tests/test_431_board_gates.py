"""Board-state gates and the lock advisor (#431).

Two gates that stop a placement tool producing a result-shaped non-result, and
an advisor that says which parts should not be moved -- and never moves them.

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
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split
from run_utils import tool as _tool  # #522: resolve a moved CLI, loudly

# #522 reorg + skill merge: engine -> py_router/, placer -> py_placer/,
# board_score.py -> the placement-and-routing skill. Without these roots the
# imports below raise and the test never runs (it reports as a failure while
# asserting nothing).
_R522 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p522 in ('py_router', 'py_placer',
              os.path.join('.claude', 'skills',
                           'plan-pcb-placement-and-routing', 'scripts')):
    _d522 = os.path.join(_R522, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)


from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.lock_advisor import advise_locks, to_json  # noqa: E402
from placement.placement_state import (UNPLACED_EXIT, assess_placement)  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

#: Measured 516 s ALONE (2026-08-19, exit 0, all checks pass) -- UNDER the
#: 600 s default, yet it times out in-suite. run_all defaults to -j 4, so
#: it competes for CPU with three other processes while that clock runs.
#: A budget for the contended case, not for a hung test: the standalone
#: number is the evidence that it finishes.
RUN_ALL_TIMEOUT = 1200


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(ROOT, 'kicad_files')
TIGARD = os.path.join(KF, 'tigard.kicad_pcb')
SMALL = os.path.join(KF, 'lvds_converter_dualclk.kicad_pcb')
ROUTED = os.path.join(KF, 'routed_output.kicad_pcb')


def _script_path(script):
    """#522: the placer CLIs live in py_placer/ now, the engine in
    py_router/. Resolve wherever the script actually is rather than
    assuming the repo root, which silently made every case here a
    'file not found' exit instead of a gate."""
    for sub in ('', 'py_placer', 'py_router'):
        cand = os.path.join(ROOT, sub, script) if sub else os.path.join(ROOT, script)
        if os.path.isfile(cand):
            return cand
    return os.path.join(ROOT, script)


def _run(script, *args):
    return subprocess.run([sys.executable, '-X', 'utf8',
                           _script_path(script)] + list(args),
                          capture_output=True, text=True, cwd=ROOT, timeout=1800)


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
    """The quench models no copper: place_optimize.py has zero references to
    segments or vias, and write_placed_output rewrites footprint positions only.
    So placement on a routed board strands every track."""
    st = assess_placement(parse_kicad_pcb(ROUTED), ROUTED)
    assert st.has_copper and st.segments > 0
    assert st.blocked


# --- the CLI contract --------------------------------------------------------

def test_both_clis_refuse_with_exit_3_and_write_nothing():
    d = tempfile.mkdtemp()
    try:
        pile = _pile(os.path.join(d, 'pile.kicad_pcb'))
        out = os.path.join(d, 'out.kicad_pcb')
        r = _run('place_optimize.py', pile, out)
        assert r.returncode == UNPLACED_EXIT, (r.returncode, r.stderr[-400:])
        assert not os.path.exists(out), "refused but still wrote a board"

        work = os.path.join(d, 'work')
        r = _run('place_route_loop.py', pile, out, '--route-args', '--nets *',
                 '--work-dir', work)
        assert r.returncode == UNPLACED_EXIT, (r.returncode, r.stderr[-400:])
        # The whole point of gating BEFORE round 0: no routing was attempted.
        assert not os.path.exists(os.path.join(work, 'loop_round0_route.log')), \
            "refused only AFTER paying for round 0's route"
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_routed_gate_and_its_override():
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'out.kicad_pcb')
        assert _run('place_optimize.py', ROUTED, out).returncode == UNPLACED_EXIT
        r = _run('place_optimize.py', ROUTED, out, '--allow-routed',
                 '--max-passes', '1')
        assert r.returncode == 0, r.stderr[-400:]
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_normal_board_is_unaffected_by_the_gates():
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'out.kicad_pcb')
        r = _run('place_optimize.py', SMALL, out, '--max-passes', '1')
        assert r.returncode == 0, r.stderr[-500:]
        assert os.path.isfile(out)
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
    """`place_route_loop` guards these with --max-target-pins; `place_optimize`
    does not. Worth reporting -- but 'large' is not 'position-critical', so it
    must not enter the lock list."""
    adv = advise_locks(parse_kicad_pcb(TIGARD), TIGARD)
    assert adv.advisories, "expected a high-pin-count advisory on tigard"
    advised = {a['ref'] for a in adv.advisories}
    assert not (advised & set(adv.lock_patterns()))


def test_advice_is_deterministic():
    a = json.dumps(to_json(advise_locks(parse_kicad_pcb(TIGARD), TIGARD)), sort_keys=True)
    b = json.dumps(to_json(advise_locks(parse_kicad_pcb(TIGARD), TIGARD)), sort_keys=True)
    assert a == b


def test_suggest_locks_writes_nothing_at_all():
    """The promise the whole design rests on. Assert the input is untouched AND
    that the default *_optimized.kicad_pcb output was never created."""
    before = (os.path.getmtime(TIGARD), os.path.getsize(TIGARD))
    default_out = os.path.splitext(TIGARD)[0] + '_optimized.kicad_pcb'
    existed = os.path.exists(default_out)
    r = _run('place_optimize.py', TIGARD, '--suggest-locks')
    assert r.returncode == 0, r.stderr[-400:]
    assert 'Nothing was locked' in r.stdout
    assert (os.path.getmtime(TIGARD), os.path.getsize(TIGARD)) == before
    assert os.path.exists(default_out) == existed, \
        "--suggest-locks created an output board"


def test_both_clis_expose_the_advisor_identically():
    """A flag on one tool and not the other is exactly the drift CLAUDE.md warns
    about -- here between two CLIs rather than CLI vs GUI."""
    outs = []
    for script in ('place_optimize.py', 'place_route_loop.py'):
        r = _run(script, TIGARD, '--suggest-locks')
        assert r.returncode == 0, (script, r.stderr[-400:])
        line = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY:')]
        assert line, f"{script} printed no JSON_SUMMARY"
        outs.append(json.loads(line[0].split('JSON_SUMMARY:', 1)[1]))
    assert outs[0] == outs[1], f"advisor disagrees between CLIs: {outs}"


TESTS = [
    test_no_tracked_board_reads_as_unplaced,
    test_a_pile_at_one_coordinate_is_unplaced,
    test_missing_outline_cannot_manufacture_an_unplaced_verdict,
    test_partially_unplaced_is_reported_but_not_blocking,
    test_routed_board_is_detected,
    test_both_clis_refuse_with_exit_3_and_write_nothing,
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
    test_suggest_locks_writes_nothing_at_all,
    test_both_clis_expose_the_advisor_identically,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
