#!/usr/bin/env python3
"""#1121: the portfolio's `poses` strategy turns a part only INTO its declaration.

`perturb_poses` turned one of the top multi-pin free parts by a quarter turn,
checking only `candidate_valid` and the forced-crossing floor, and read no
rotation declaration. On splitflap_driver the default 12-candidate run's
candidate 2 (`poses`, round 0) turns U1 270 -> 90 -- and U1 is also the part
`rank_rotations` picks by default, so the free-agent flow (`rank_rotations
--write-intent`, seed, `place_portfolio --intent`) turned the very part whose
angle it had just declared. The quench after it pins declared parts, but it
starts from the pose the variant already turned.

Now `generate()` hands the strategy the claims the quench is gated with, and
`_pose_variants` offers a declared part only the angles its declaration admits
other than the one it has. A part with no angle to turn to does not take a
slot, so the next part by pin count is explored instead.

The splitflap facts this leans on, measured at 16c096b1 (the fixture arm
refuses to run if they drift): the pin-count order is U1, U5, U7, U9 (16
pins, all at 270), U10, U2, then U4 (15 pins, at 0); U1's only legal turn is
90 (inversions 32 -> 29), U5's is 90, U7's and U10's raise the inversion
floor, U9 already sits invalid, and U4's 180 survives.

Run: python3 -X utf8 tests/test_1121_portfolio_declared_rotation.py [case ...]
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 900

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
PORTFOLIO = os.path.join(ROOT, 'py_placer', 'place_portfolio.py')

_STATE = {}


def _oracle():
    """One shared oracle state, restored to the board's poses on every
    call (perturb_poses applies its variant)."""
    from kicad_parser import parse_kicad_pcb
    from placement import portfolio
    if not _STATE:
        pcb = parse_kicad_pcb(run_utils.evidence(BOARD))
        free = portfolio.free_refs(pcb, BOARD, None)
        st = portfolio.make_oracle(pcb, BOARD, free=free, clearance=0.25,
                                   board_edge_clearance=0.55, grid_step=0.1,
                                   ignore_ids=set())
        _STATE.update(st=st, free=free, origin={
            r: (st.parts[r].x, st.parts[r].y, st.parts[r].rot)
            for r in st.parts})
    st = _STATE['st']
    for r in sorted(_STATE['origin']):
        x, y, rot = _STATE['origin'][r]
        p = st.parts[r]
        if (p.x, p.y, p.rot) != (x, y, rot):
            st.apply_move(r, x, y, rot)
    return st, _STATE['free']


def _sequence(declared=None, sentinel=False):
    """Every variant the strategy offers, in round order, as (ref, rot)."""
    from placement import portfolio
    out, k = [], 0
    while True:
        st, free = _oracle()
        res = (portfolio.perturb_poses(st, free, k) if sentinel
               else portfolio.perturb_poses(st, free, k, declared=declared))
        if res is None:
            _oracle()
            return out
        (p,) = res
        out.append((p['reference'], p['new_rotation']))
        k += 1


def test_the_fixture_offers_u1_first():
    """The control every other arm leans on: undeclared, round 0 turns U1."""
    from placement import portfolio
    got = _sequence(sentinel=True)
    assert got == [('U1', 90.0), ('U5', 90.0)], (
        'the splitflap fixture drifted: %r -- re-measure before trusting any '
        'arm here' % (got,))
    st, free = _oracle()
    assert portfolio._poses_ranked(st, free, {}) == [
        'U1', 'U5', 'U7', 'U9', 'U10', 'U2']
    print("  undeclared: %r" % (got,))


def test_a_declared_angle_is_not_turned():
    """U1 declared at the angle it has: no U1 variant, and its slot goes to
    U4, whose 180 survives."""
    got = _sequence({'U1': (270.0, None)})
    assert got == [('U5', 90.0), ('U4', 180.0)], got
    print("  U1 declared 270: %r" % (got,))


def test_a_declared_slot_goes_to_the_next_part():
    from placement import portfolio
    st, free = _oracle()
    assert portfolio._poses_ranked(st, free, {'U1': (270.0, None)}) == [
        'U5', 'U7', 'U9', 'U10', 'U2', 'U4']
    print("  the six slots skip a part that cannot turn")


def test_a_part_off_its_declared_angle_is_offered_that_angle():
    """Declared 90 while sitting at 270: the only turn offered is INTO the
    declaration. Declared 0 (illegal here): U1 still has a variant by its
    declaration, so it keeps its slot and offers nothing legal."""
    got = _sequence({'U1': (90.0, None)})
    assert [r for ref, r in got if ref == 'U1'] == [90.0], got
    got0 = _sequence({'U1': (0.0, None)})
    assert not [1 for ref, _ in got0 if ref in ('U1', 'U4')], got0
    print("  declared 90 -> %r; declared 0 -> %r" % (got, got0))


def test_a_candidate_set_bounds_the_variants():
    got = _sequence({'U1': (None, (270.0, 90.0))})
    assert ('U1', 90.0) in got, got
    assert [r for ref, r in got if ref == 'U1'] == [90.0], got
    got2 = _sequence({'U1': (None, (270.0, 180.0))})
    assert not [1 for ref, _ in got2 if ref in ('U1', 'U4')], got2
    print("  {270, 90} -> %r; {270, 180} -> %r" % (got, got2))


def test_author_order_and_no_turn_out():
    from placement.portfolio import _pose_variants

    class P:
        rot = 270.0
    assert _pose_variants(P, (None, (180.0, 90.0, 270.0))) == [180.0, 90.0]
    assert _pose_variants(P, (270.0, None)) == []
    assert _pose_variants(P, (90.0, None)) == [90.0]
    assert _pose_variants(P, None) == [0.0, 90.0, 180.0]
    print("  author order; the current angle is never a variant")


def test_an_off_lattice_member_is_judged_on_its_own_box():
    """The box must exist WHEN 45 is judged, not merely afterwards (the code
    reviewer: a cache filled after `candidate_valid` would pass a check of
    the cache alone)."""
    from placement import portfolio
    st, free = _oracle()
    assert 45.0 not in st.parts['U1'].bounds_by_rot
    judged = []
    real = st.candidate_valid

    def spy(ref, x, y, rot):
        if ref == 'U1' and abs(rot - 45.0) < 1e-9:
            judged.append(45.0 in st.parts['U1'].bounds_by_rot)
        return real(ref, x, y, rot)
    st.candidate_valid = spy
    try:
        portfolio.perturb_poses(st, free, 0,
                                declared={'U1': (None, (270.0, 45.0))})
    finally:
        del st.candidate_valid
    assert judged and all(judged), (
        'a declared 45 was judged on the unrotated box: %r' % judged)
    _oracle()
    print("  a declared 45 is judged with its own box in place")


def test_no_declaration_is_the_strategy_unchanged():
    """{} and None are the old call exactly, and a declaration on a part
    outside the top six (R1, two pins) changes nothing."""
    base = _sequence(sentinel=True)
    assert _sequence({}) == base and _sequence(None) == base, base
    assert _sequence({'R1': (0.0, None)}) == base
    print("  undeclared and off-top declarations: %r" % (base,))


def test_place_portfolio_does_not_turn_a_declared_part():
    """End to end, with the block `rank_rotations --write-intent` writes:
    candidate 2 (`poses`, round 0) used to be U1 turned 270 -> 90.

    The SEED board is the witness. On this board the quench happens to turn
    U1 back (16c096b1 wrote cand_02 identical to the baseline), so the
    quenched board reads 270 either way; that assertion only guards the
    quench, and the seed's angle is what the strategy chose."""
    from kicad_parser import parse_kicad_pcb
    from rank_rotations import rotation_block

    def run(td, blocks, name):
        ip = os.path.join(td, name + '.json')
        with open(ip, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                       'blocks': blocks}, fh)
        out = os.path.join(td, name)
        r = run_utils.check([sys.executable, '-X', 'utf8', PORTFOLIO, BOARD,
                             '--out-dir', out, '--intent', ip, '--only', '2',
                             '--no-render'], accept=True)
        assert '"strategy": "poses"' in r.stdout, r.stdout[-1500:]
        seed = parse_kicad_pcb(run_utils.evidence(
            os.path.join(out, 'cand_02.seed.kicad_pcb'))).footprints
        final = parse_kicad_pcb(run_utils.evidence(
            os.path.join(out, 'cand_02.kicad_pcb'))).footprints
        return r, seed, final

    with tempfile.TemporaryDirectory() as td:
        r, seed, final = run(td, [rotation_block('U1', 270.0)], 'declared')
        assert seed['U1'].rotation % 360 == 270.0, seed['U1'].rotation
        assert final['U1'].rotation % 360 == 270.0, final['U1'].rotation
        assert seed['U5'].rotation % 360 == 90.0, seed['U5'].rotation
        assert 'turned only to an angle its declaration admits' in r.stdout
        # The control: the same call with a block that declares nothing
        # about U1 still turns it, so "not turned" is the declaration's doing.
        _r, seed0, _f = run(td, [{'name': 'z', 'refs': ['U5']}], 'control')
        assert seed0['U1'].rotation % 360 == 90.0, seed0['U1'].rotation
    print("  declared U1 stays at 270 in the seed and the quenched board; "
          "the control turns it to 90")


TESTS = [
    test_the_fixture_offers_u1_first,
    test_a_declared_angle_is_not_turned,
    test_a_declared_slot_goes_to_the_next_part,
    test_a_part_off_its_declared_angle_is_offered_that_angle,
    test_a_candidate_set_bounds_the_variants,
    test_author_order_and_no_turn_out,
    test_an_off_lattice_member_is_judged_on_its_own_box,
    test_no_declaration_is_the_strategy_unchanged,
    test_place_portfolio_does_not_turn_a_declared_part,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
