#!/usr/bin/env python3
"""#829 -- the movable set was gated on PADS, not on whether a footprint owns
board geometry, so a pad-bearing footprint carrying Edge.Cuts was freely moved
and the board silently resized underneath every consumer of `board_bounds`.

The distinction this file exists to pin is NOT "does it own Edge.Cuts". It is
**which** Edge.Cuts:

* geometry OUTSIDE the board-level outline is the board's own boundary --
  moving it resizes the board, which is a mechanical decision the user owns;
* geometry INSIDE is a relief the designer parented to the part so it travels
  WITH it. crkbd draws 184 per-LED cutout windows that way, and #628 measured
  that freezing such a part costs it every legal pose it has (run 20's SW2:
  0 legal of 14884).

Getting that backwards is not a hypothetical: the first draft of this fix
locked every owner, and would have frozen all 184.

    python3 tests/test_829_edge_cuts_owner.py
"""
import os
import subprocess
import sys
import tempfile

RUN_ALL_FAST_OK = True   # one cheap place_optimize run; everything else is
                         # in-process parsing of a tiny synthetic board.

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

import fixture_829 as FIX                                        # noqa: E402
import run_utils                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                          # noqa: E402

_fails = []


def check(label, cond, detail=''):
    print(f"  {'ok ' if cond else 'FAIL'}  {label}"
          + (f"   {detail}" if detail and not cond else ''))
    if not cond:
        _fails.append(label)


# --------------------------------------------------------------------------
def test_the_parser_separates_the_fact_from_the_decision():
    """`owns_edge_cuts` is the fact, `owns_board_outline` the decision."""
    board = FIX.write()
    run_utils.evidence(board, 'the #829 fixture board')
    pcb = parse_kicad_pcb(board)
    for ref, (want_owns, want_struct) in FIX.EXPECTED.items():
        fp = pcb.footprints.get(ref)
        check(f"{ref}: parsed", fp is not None)
        if fp is None:
            continue
        check(f"{ref}: owns_edge_cuts={want_owns}",
              fp.owns_edge_cuts == want_owns, f"got {fp.owns_edge_cuts}")
        check(f"{ref}: owns_board_outline={want_struct}",
              fp.owns_board_outline == want_struct,
              f"got {fp.owns_board_outline}")


def test_a_carried_relief_is_not_a_board_outline():
    """The crkbd/#628 case, stated as its own check because it is the half a
    'lock every owner' rule gets wrong."""
    pcb = parse_kicad_pcb(FIX.write())
    carry = pcb.footprints['U_CARRY']
    check("a window under the part's own body is owned but NOT structural",
          carry.owns_edge_cuts and not carry.owns_board_outline)


def test_the_movable_set_refuses_the_owner_and_says_why():
    from placement.portfolio import free_refs
    board = FIX.write()
    refused = {}
    free = free_refs(parse_kicad_pcb(board), board, None, refused=refused)
    check("U_STRUCT is not free", 'U_STRUCT' not in free, str(free))
    check("U_CARRY IS still free", 'U_CARRY' in free, str(free))
    check("U_PLAIN IS still free", 'U_PLAIN' in free, str(free))
    check("the refusal names the ref", 'U_STRUCT' in refused, str(refused))
    check("the refusal gives a reason, not just a flag",
          'outline' in refused.get('U_STRUCT', '').lower()
          and '#829' in refused.get('U_STRUCT', ''),
          refused.get('U_STRUCT', ''))
    check("only the outline owner is refused", list(refused) == ['U_STRUCT'],
          str(refused))


def test_the_quench_locks_the_owner_and_discloses_it():
    from placement.quench import QuenchState
    board = FIX.write()
    st = QuenchState(parse_kicad_pcb(board), board,
                     clearance=0.2, board_edge_clearance=0.2,
                     crossing_penalty=0.0, halo_base=0.0, halo_coef=0.0,
                     halo_weight=0.0, edge_halo=0.0, edge_weight=0.0,
                     grid_step=0.1)
    p = st.parts.get('U_STRUCT')
    check("U_STRUCT is a part (an obstacle), not dropped", p is not None)
    check("U_STRUCT is locked", p is not None and p.locked)
    c = st.parts.get('U_CARRY')
    check("U_CARRY is NOT locked", c is not None and not c.locked)
    check("the lock is disclosed by ref",
          getattr(st, 'outline_locked', None) == ['U_STRUCT'],
          str(getattr(st, 'outline_locked', None)))


def test_the_writer_refuses_rather_than_skipping():
    """A skip would be indistinguishable from success -- `write_placed_output`
    returns True unconditionally, and route.py:4074 gates its in-memory mirror
    on that return."""
    from placement.writer import write_placed_output, OutlineOwnerMove
    board = FIX.write()
    pcb = parse_kicad_pcb(board)

    def move(ref):
        fp = pcb.footprints[ref]
        return [{'reference': ref, 'new_x': fp.x + 1.0, 'new_y': fp.y,
                 'new_rotation': fp.rotation or 0}]

    out = os.path.join(os.path.dirname(board), 'w1.kicad_pcb')
    raised = False
    try:
        write_placed_output(board, out, move('U_STRUCT'))
    except OutlineOwnerMove:
        raised = True
    check("moving the outline owner RAISES", raised)
    check("and writes no output file", not os.path.exists(out))

    out2 = os.path.join(os.path.dirname(board), 'w2.kicad_pcb')
    ok = write_placed_output(board, out2, move('U_CARRY'))
    check("moving a carried relief still succeeds", bool(ok))
    check("and the board does NOT resize",
          parse_kicad_pcb(board).board_info.board_bounds
          == parse_kicad_pcb(out2).board_info.board_bounds)


def test_the_outline_tripwire_fires_when_every_per_ref_gate_is_bypassed():
    """The per-ref gates answer "may this footprint move". The tripwire answers
    the question the contract actually makes -- "is the outline the same board
    it was" -- so it has to hold even when the per-ref gate is defeated.

    Defeat it deliberately, which is the only way to reach the tripwire: with
    the gates working, no placement list can contain an outline owner.
    """
    from placement import writer as W
    board = FIX.write()
    pcb = parse_kicad_pcb(board)
    fp = pcb.footprints['U_STRUCT']
    out = os.path.join(os.path.dirname(board), 'bypass.kicad_pcb')
    mv = [{'reference': 'U_STRUCT', 'new_x': fp.x + 2.0, 'new_y': fp.y,
           'new_rotation': fp.rotation or 0}]

    orig = W._draws_board_outline
    W._draws_board_outline = lambda *a, **k: False   # bypass the per-ref gate
    try:
        raised = False
        try:
            W.write_placed_output(board, out, mv)
        except W.OutlineOwnerMove as exc:
            raised = True
            msg = str(exc)
        check("the tripwire catches an outline change the gates missed", raised)
        check("and it says the outline changed, not just 'refused'",
              raised and 'CHANGED THE BOARD OUTLINE' in msg,
              msg if raised else '')
    finally:
        W._draws_board_outline = orig

    # ...and it does NOT fire for a carried relief, whose whole point is that
    # it travels with the part. This is the half a naive whole-outline
    # fingerprint gets wrong -- moving a window changes board_cutouts.
    W._draws_board_outline = lambda *a, **k: False
    try:
        c = pcb.footprints['U_CARRY']
        out2 = os.path.join(os.path.dirname(board), 'bypass_carry.kicad_pcb')
        ok = W.write_placed_output(board, out2, [
            {'reference': 'U_CARRY', 'new_x': c.x + 2.0, 'new_y': c.y,
             'new_rotation': c.rotation or 0}])
        check("a carried relief still writes with the gate bypassed", bool(ok))
        moved = parse_kicad_pcb(out2).footprints['U_CARRY']
        check("and it really did move", abs(moved.x - c.x) > 1.0,
              f"{c.x} -> {moved.x}")
    finally:
        W._draws_board_outline = orig


def test_a_pure_rotation_is_covered_too():
    """`local_to_global` transforms Edge.Cuts by the footprint's ANGLE, so a
    rotation moves the outline with the part standing still."""
    a = parse_kicad_pcb(FIX.write(struct_rot=None)).board_info.board_bounds
    d = tempfile.mkdtemp(prefix='fix829rot_')
    b = parse_kicad_pcb(FIX.write(d, struct_rot=90)).board_info.board_bounds
    check("rotating the owner alone moves the outline (the hazard is real)",
          a != b, f"{a} vs {b}")
    pcb = parse_kicad_pcb(FIX.write(tempfile.mkdtemp(prefix='fix829rot2_'),
                                    struct_rot=90))
    check("and the rotated owner is still classified structural",
          pcb.footprints['U_STRUCT'].owns_board_outline)


def test_a_board_with_no_owners_is_untouched():
    """The whole corpus is this case, so it must cost nothing and change
    nothing."""
    src = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    run_utils.evidence(src, 'splitflap_driver')
    pcb = parse_kicad_pcb(src)
    check("no footprint reports owning Edge.Cuts",
          not any(f.owns_edge_cuts for f in pcb.footprints.values()))
    check("no footprint reports owning the outline",
          not any(f.owns_board_outline for f in pcb.footprints.values()))


def test_no_tracked_board_carries_footprint_edge_cuts():
    """The claim the fixture rests on, re-derived rather than asserted: if a
    tracked board ever grows footprint-embedded Edge.Cuts, this fix starts
    acting on real boards and someone should know."""
    from kicad_parser import footprint_outline_owners
    # EVERY tracked board, not `run_utils.corpus_boards()` -- that globs
    # kicad_files/ only (22), while the claim in the PR is about all 27 tracked
    # .kicad_pcb files, which includes tests/fixtures/. Scan what the claim
    # says.
    r = subprocess.run(['git', 'ls-files', '*.kicad_pcb'],
                       capture_output=True, text=True, cwd=ROOT)
    boards = [os.path.join(ROOT, p) for p in r.stdout.split('\n') if p.strip()]
    hits = {}
    for b in boards:
        if not os.path.exists(b):
            continue
        with open(b, encoding='utf-8') as fh:
            owners = footprint_outline_owners(fh.read())
        if owners:
            hits[os.path.basename(b)] = owners
    check(f"0 of {len(boards)} tracked boards carry footprint Edge.Cuts",
          not hits, str(hits))
    check("the corpus was actually scanned (not an empty list)",
          len(boards) >= 25, f"{len(boards)} boards")


def test_place_optimize_leaves_the_outline_alone():
    """The product-level arm: the real CLI, on a board with an owner."""
    d = tempfile.mkdtemp(prefix='fix829cli_')
    board = FIX.write(d)
    out = os.path.join(d, 'placed.kicad_pcb')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'place_optimize.py'), board, out,
         '--max-displacement', '3'],
        capture_output=True, text=True, cwd=ROOT, timeout=600)
    check("place_optimize succeeded", r.returncode == 0,
          (r.stderr or r.stdout)[-400:])
    if r.returncode != 0 or not os.path.exists(out):
        return
    a = parse_kicad_pcb(board).board_info
    b = parse_kicad_pcb(out).board_info
    check("board_bounds unchanged", a.board_bounds == b.board_bounds,
          f"{a.board_bounds} -> {b.board_bounds}")
    check("board_outlines unchanged", a.board_outlines == b.board_outlines)
    before = parse_kicad_pcb(board).footprints['U_STRUCT']
    after = parse_kicad_pcb(out).footprints['U_STRUCT']
    check("the outline owner did not move",
          (before.x, before.y, before.rotation) ==
          (after.x, after.y, after.rotation))
    check("the run disclosed the lock",
          '#829' in r.stdout, r.stdout[-300:])


TESTS = [
    test_the_parser_separates_the_fact_from_the_decision,
    test_a_carried_relief_is_not_a_board_outline,
    test_the_movable_set_refuses_the_owner_and_says_why,
    test_the_quench_locks_the_owner_and_discloses_it,
    test_the_writer_refuses_rather_than_skipping,
    test_the_outline_tripwire_fires_when_every_per_ref_gate_is_bypassed,
    test_a_pure_rotation_is_covered_too,
    test_a_board_with_no_owners_is_untouched,
    test_no_tracked_board_carries_footprint_edge_cuts,
    test_place_optimize_leaves_the_outline_alone,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    if _fails:
        print(f"\n{len(_fails)} FAILED: {', '.join(_fails)}")
        sys.exit(1)
    print("\nALL PASS")
