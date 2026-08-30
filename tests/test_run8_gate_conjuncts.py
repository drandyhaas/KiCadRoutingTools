#!/usr/bin/env python3
"""The wrong-basin gates, inside the engine's own accept test.

E6 and E7 shipped first as checkers, which catch a bad placement AFTER it is
written. The same two facts belong in the accept test, where a candidate can be
refused at the moment it is proposed:

  E6  A locked pose is a decision made outside this toolchain. A candidate that
      lands copper on one has not found a trade-off, it has broken a premise.
      So `locked_contact_pairs` leads the lexicographic gate tuple: nothing
      below it -- not hpwl, not overlap -- can buy one.

  E7  In the exchange stage the engine KNOWS the vector it applied to each
      group. Members displaced by the same k*v keep their relative geometry, so
      they cannot newly touch each other. A new contact ACROSS groups means the
      assignment is not a rigid restore but a search result that happens to
      fit, and it is rejected with the pair named.

Both terms are 0 on a healthy board, which is why the corpus no-op sweep does
not move (66/66 against baseline after this change).

Run: python3 -X utf8 tests/test_run8_gate_conjuncts.py
"""
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import reconstruct                              # noqa: E402
import pose_score                                              # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

#: A big part with a .Fab body, a small part, and a fiducial. The pads never
#: touch, so NOTHING in the pad_intersection channel fires -- that is the
#: defect shape: it reported blocking 0.
BODY_BOARD = """(kicad_pcb (version 20221018) (generator pcbnew)
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "VCC") (net 2 "GND")
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))
  (footprint "t:BIG" (layer "F.Cu") (at 20 20)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
    (fp_rect (start -4 -4) (end 4 4) (stroke (width 0.05) (type default)) (layer "F.Fab"))
    (fp_rect (start -4.2 -4.2) (end 4.2 4.2) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
    (pad "1" smd rect (at -3.7 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "VCC"))
    (pad "2" smd rect (at 3.7 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "VCC")))
  (footprint "t:SMALL" (layer "F.Cu") (at 32 20)
    (property "Reference" "RN1" (at 0 0) (layer "F.SilkS"))
    (fp_rect (start -0.5 -0.3) (end 0.5 0.3) (stroke (width 0.05) (type default)) (layer "F.Fab"))
    (fp_rect (start -0.7 -0.5) (end 0.7 0.5) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
    (pad "1" smd rect (at 0 0) (size 0.2 0.2) (layers "F.Cu") (net 2 "GND")))
  (footprint "MountingHole:MountingHole_3.2mm_M3" (layer "F.Cu") (at 32 30)
    (property "Reference" "FID1" (at 0 0) (layer "F.SilkS"))
    (fp_rect (start -0.5 -0.5) (end 0.5 0.5) (stroke (width 0.05) (type default)) (layer "F.Fab"))
    (fp_rect (start -0.7 -0.7) (end 0.7 0.7) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
    (pad "1" smd circle (at 0 0) (size 0.3 0.3) (layers "F.Cu") (net 0 "")))
)
"""
FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def state_for(path, clearance=0.1):
    return pose_score.make_state(parse_kicad_pcb(path), path,
                                 clearance=clearance,
                                 board_edge_clearance=0.2, grid_step=0.1)


def main():
    print('the gate tuple leads with locked contacts')
    st = state_for(BOARD)
    t = reconstruct.measure(st)
    check('the tuple gained a leading term', len(t) == 7, str(t))
    check('a healthy board scores 0 locked contacts', t[0] == 0, str(t))
    check('the state metric exists', 'locked_contact_pairs'
          in st.pad_legality_metrics(), str(sorted(st.pad_legality_metrics())))

    print('a locked contact outranks everything a candidate could win')
    # Two tuples identical but for the leading term: the one with a locked
    # contact must compare WORSE however good the rest is.
    good = (0, 5, 1.0, 2.0, 3, 999.0, 9.0)
    bad = (1, 0, 0.0, 0.0, 0, 0.0, 0.0)
    check('any locked contact loses to none, whatever else improves',
          good < bad, f'{good} vs {bad}')

    print('E7: contact across two move groups is refused')
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'b.kicad_pcb')
        shutil.copy(BOARD, path)
        st2 = state_for(path)
        refs = sorted(st2.parts)
        a, b = refs[0], refs[1]
        snap = {r: (st2.parts[r].x, st2.parts[r].y, st2.parts[r].rot)
                for r in (a, b)}

        # Nothing moved yet: no cross-group contact to find.
        check('an untouched pair in two groups is clean',
              reconstruct._cross_group_contact(st2, {1: [a], -1: [b]}, snap)
              is None)

        # Drive them onto each other; they are in different groups, so the
        # contact they create is exactly what E7 forbids.
        pa, pb = st2.parts[a], st2.parts[b]
        mx, my = (pa.x + pb.x) / 2.0, (pa.y + pb.y) / 2.0
        st2.apply_move(a, mx, my, pa.rot)
        st2.apply_move(b, mx, my, pb.rot)
        hit = reconstruct._cross_group_contact(st2, {1: [a], -1: [b]}, snap)
        check('two parts driven together from different groups are caught',
              hit is not None and set(hit) == {a, b}, str(hit))

        # The same contact WITHIN one group is not this rule's business: a
        # group moves rigidly, so an overlap inside it came from the input.
        check('the same pair inside ONE group is not flagged',
              reconstruct._cross_group_contact(st2, {1: [a, b]}, snap) is None)

    # A gate term printed under its neighbour's name is worse than one not
    # printed at all: the reconstruct ladder's apply rule is "improves the
    # violation count AND does not increase the off-board amount", so a human
    # or an executor compares `oob` by eye. When locked_contacts was prepended
    # to measure(), three f-strings kept their old indices and `oob` printed
    # the HOLE SHORTFALL -- 0.0 on a board carrying a part 44 mm off the
    # outline. Labels now come from one definition; this asserts they match it.
    print('the gate labels match the gate tuple')
    live = reconstruct.measure(st)
    check('there is one name per term',
          len(reconstruct.GATE_TERMS) == len(live),
          f'{len(reconstruct.GATE_TERMS)} names, {len(live)} terms')
    line = reconstruct.format_gate(live)
    check('every term is labelled in the printed line',
          all(f'{n}=' in line for n in reconstruct.GATE_TERMS), line)
    check('a tuple of the wrong width says so instead of mislabelling',
          'GATE TUPLE CHANGED' in reconstruct.format_gate((1, 2, 3)))
    probe = reconstruct.format_gate((0, 1, 2.0, 3.0, 4, 5.0, 6.0))
    check('...and the values land on their own names, in order',
          'oob=3' in probe and 'hpwl=5' in probe and 'overlap=6' in probe,
          probe)
    for name in ('place_reconstruct.py',):
        txt = open(os.path.join(ROOT, 'py_placer', name), encoding='utf-8').read()
        check(f'{name} prints the gate through format_gate only',
              'pad_pairs={' not in txt, 'a hand-indexed gate print survives')

    print('the engine wires both in')
    src = open(os.path.join(ROOT, 'py_placer', 'placement', 'reconstruct.py'),
               encoding='utf-8').read()
    check('the exchange accept requires no cross-group contact',
          'and not cross_group_hit' in src)
    check('a refusal is reported, not silent',
          'REJECTED an assignment' in src)

    print("E8: a candidate pose INSIDE another part's body is refused")
    # Run 22's defect, verbatim. assign/exchange moved parts by a derived
    # +/-v with only pad legality and hpwl in the objective, so a teleport
    # landed a part wholly inside another part's body and scored clean --
    # twice, on two different pairs, both reporting `blocking 0` because no
    # pad copper intersects.
    with tempfile.TemporaryDirectory() as td:
        bp = os.path.join(td, 'body.kicad_pcb')
        with open(bp, 'w', encoding='utf-8') as f:
            f.write(BODY_BOARD)
        bst = state_for(bp)
        big = bst.parts['U1']
        inside = (big.x, big.y)          # dead centre of U1's body
        clear = (big.x + 12.0, big.y)    # well away from it

        check("a pose inside another part's body conflicts",
              reconstruct._pair_conflicts(bst, 'RN1', inside, 'U1',
                                          (big.x, big.y)),
              'the teleport that shipped twice in run 22 is still accepted')
        check("a pose clear of it does not",
              not reconstruct._pair_conflicts(bst, 'RN1', clear, 'U1',
                                              (big.x, big.y)))
        # The marker exemption is load-bearing and measured: orangecrab ships
        # FID2 wholly inside J5 (frac 1.000) and FID1 inside J4 (0.867).
        # Without it a displaced fiducial could never come home.
        check('a fiducial inside a body is EXEMPT, not a conflict',
              not reconstruct._pair_conflicts(bst, 'FID1', inside, 'U1',
                                              (big.x, big.y)),
              'the marker exemption was dropped; orangecrab FID1/FID2 break')
        # The run-4 lesson, pinned into the predicate: a kiss is not a
        # containment and must not be charged as one.
        kiss = (big.x + 4.4, big.y)
        check('a body KISS is not charged as containment',
              not reconstruct._pair_conflicts(bst, 'RN1', kiss, 'U1',
                                              (big.x, big.y)),
              'courtyard/body kisses are being charged -- run-4 regression')

    print('the containment fix did NOT re-promote courtyard in the gate')
    # run-4 measured courtyard overlap at r=+0.72 with distance-to-truth (not
    # with routed blocking -- docs/placement-predictors.md) and
    # demoted it below hpwl deliberately. Containment is a hard predicate,
    # not a gate term, so the tuple must be untouched.
    check('the gate tuple is still 7 terms ending hpwl, overlap',
          len(reconstruct.GATE_TERMS) == 7
          and reconstruct.GATE_TERMS[5:] == ('hpwl', 'overlap'),
          str(reconstruct.GATE_TERMS))
    check('the engine wires containment into the pair predicate',
          '_body_contained(state, a, pos_a, b, pos_b)' in src)

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
