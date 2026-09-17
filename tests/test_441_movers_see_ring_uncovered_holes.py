"""A ring-uncovered plated drill is a HOLE to the copper-movers too (#441).

A pad whose copper ring is SMALLER than its drill leaves the hole exposed: a
track crossing it is cut by the drill, net-independently, exactly like an NPTH
mounting hole. #441 taught that to two of the three consumers --
`add_drill_hole_obstacles` stamps it (so the ROUTER keeps clear) and
`check_drc`'s copper-to-hole branch grades it (both name vfo_ctrl's U4 "MH" in
their comments). `_foreign_hole_capsules`, the list every copper-MOVING pass
consults, kept only the no-copper branch.

So the pad fell between two populations: too much copper to count as a hole,
too little to block anything. Measured on vfo_ctrl, whose U4 carries four
2.5mm mounting holes with 0.001mm pads:

    _pad_has_no_copper(U4.MH)                  False   -> not in the hole list
    foreign-hole capsules on the board             0   -> on a board with four
    _seg_foreign_hole_dist through a centre      1e9   -> completely blind

The router kept its distance and then a mover put copper back:

    unrouted input      0 tracks crossing a 2.5mm MH hole
    step1_planes        0
    step2_route         2          (+3V3 and LOCK)

graded as 3 track-hole violations at v0.22.0 (one overlapping by 0.857mm --
straight through the middle) and 4 at HEAD. This is NOT a v0.22.0 -> HEAD
regression: it predates the release, and HEAD merely routed one more net across
the same hole. With the movers taught the same rule, vfo_ctrl grades

    v0.22.0  3 DRC      HEAD  4 DRC      HEAD + this  0 DRC

i.e. better than the baseline, because the fix removes the pre-existing three
as well.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _board(pad_size, drill, pad_type='thru_hole', layers=('*.Cu', '*.Mask')):
    """One footprint carrying one drilled pad, net 0, at (10, 10)."""
    from kicad_parser import PCBData, BoardInfo, Footprint, Pad
    pad = Pad(pad_number='MH', net_id=0, net_name='', global_x=10.0, global_y=10.0,
              local_x=0.0, local_y=0.0, size_x=pad_size, size_y=pad_size,
              shape='circle', layers=list(layers), drill=drill,
              pad_type=pad_type, component_ref='U4')
    fp = Footprint(reference='U4', footprint_name='t:mh', x=10.0, y=10.0,
                   rotation=0.0, layer='F.Cu', pads=[pad])
    pcb = PCBData(board_info=BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=(0.0, 0.0, 30.0, 30.0), stackup=[]),
                  nets={}, footprints={'U4': fp}, vias=[], segments=[],
                  pads_by_net={0: [pad]})
    return pcb


def t_a_ring_uncovered_plated_hole_is_seen():
    """vfo_ctrl's shape: 0.001mm of copper over a 2.5mm drill."""
    from single_ended_routing import _foreign_hole_capsules, _seg_foreign_hole_dist
    pcb = _board(0.001, 2.5)
    nid = _foreign_hole_capsules(pcb)[0]
    check('t_a_ring_uncovered_plated_hole_is_seen',
          nid.size == 1,
          f'{nid.size} capsule(s) for a 2.5mm drill under a 0.001mm pad')
    d = _seg_foreign_hole_dist(pcb, 7, 8.0, 10.0, 12.0, 10.0)
    check('t_a_track_through_it_measures_negative',
          d < 0,
          f'a track through the centre measures {d:.3f}mm (negative = over the '
          f'hole)' if d < 1e8 else 'still 1e9 -- the movers remain blind')


def t_a_properly_ringed_pad_is_not_a_hole():
    """The control. A pad whose copper SPANS its drill is ordinary copper --
    counting it here would make every through-hole pad a keep-out and strand
    the nets that must reach one."""
    from single_ended_routing import _foreign_hole_capsules
    nid = _foreign_hole_capsules(_board(1.6, 0.8))[0]
    check('t_a_properly_ringed_pad_is_not_a_hole',
          nid.size == 0,
          f'{nid.size} capsule(s) for a 1.6mm pad over a 0.8mm drill (must be 0)')


def t_the_no_copper_population_still_qualifies():
    """#233's original population must not have been displaced."""
    from single_ended_routing import _foreign_hole_capsules
    npth = _foreign_hole_capsules(_board(3.2, 3.0, pad_type='np_thru_hole'))[0]
    check('t_an_npth_hole_still_qualifies',
          npth.size == 1, f'{npth.size} capsule(s) for an NPTH mounting hole')
    masked = _foreign_hole_capsules(_board(3.2, 3.0, layers=('*.Mask',)))[0]
    check('t_a_pad_with_no_copper_layer_still_qualifies',
          masked.size == 1, f'{masked.size} capsule(s) for a mask-only pad')


def t_the_three_consumers_use_one_rule():
    """The map, the grader and the movers must agree on 'exposed drill'.

    Each keeps its own loop for its own geometry, so this compares the TEST
    they branch on rather than demanding a shared function -- that test is what
    drifted, and it is what has to stay in step.
    """
    import re
    root = os.path.join(os.path.dirname(__file__), '..', 'py_router')
    want = re.compile(r'max\(pad\.size_x,\s*pad\.size_y\)\s*<\s*pad\.drill')
    for fname, who in (('obstacle_map.py', 'the router\'s map'),
                       ('check_drc.py', 'the grader'),
                       ('single_ended_routing.py', 'the copper-movers')):
        with open(os.path.join(root, fname)) as f:
            src = f.read()
        check(f't_{fname.split(".")[0]}_uses_the_ring_uncovered_test',
              bool(want.search(src)),
              f'{who} branches on copper-smaller-than-drill')


def main():
    t_a_ring_uncovered_plated_hole_is_seen()
    t_a_properly_ringed_pad_is_not_a_hole()
    t_the_no_copper_population_still_qualifies()
    t_the_three_consumers_use_one_rule()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
