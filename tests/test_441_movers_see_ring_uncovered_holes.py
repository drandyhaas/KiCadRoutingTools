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


def _board(pad_size, drill, pad_type='thru_hole', layers=('*.Cu', '*.Mask'),
           net_id=0):
    """One footprint carrying one drilled pad, net 0 unless given, at (10, 10)."""
    from kicad_parser import PCBData, BoardInfo, Footprint, Pad
    pad = Pad(pad_number='MH', net_id=net_id, net_name='/GND' if net_id else '', global_x=10.0, global_y=10.0,
              local_x=0.0, local_y=0.0, size_x=pad_size, size_y=pad_size,
              shape='circle', layers=list(layers), drill=drill,
              pad_type=pad_type, component_ref='U4')
    fp = Footprint(reference='U4', footprint_name='t:mh', x=10.0, y=10.0,
                   rotation=0.0, layer='F.Cu', pads=[pad])
    pcb = PCBData(board_info=BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=(0.0, 0.0, 30.0, 30.0), stackup=[]),
                  nets={}, footprints={'U4': fp}, vias=[], segments=[],
                  pads_by_net={net_id: [pad]})
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


def t_copper_equal_to_its_drill_is_a_hole():
    """#1753: a NET-LESS pad whose copper equals its drill (an unnamed locating
    peg, 0.6mm over 0.6mm) joins the movers' hole list and the plane maps' via
    floor. On a net it stays copper: the router waives hole keep-outs for a
    net it routes, so a grade holding every net off it would ask more than the
    route was held to. An NPTH pad whose size equals its drill is a bare hole,
    never a peg. The #441 speck stays a hole on any net."""
    from kicad_parser import flush_drill_peg
    from single_ended_routing import _foreign_hole_capsules
    from obstacle_map import via_floor_npth_holes
    nid = _foreign_hole_capsules(_board(0.6, 0.6))[0]
    check('t_copper_equal_to_its_drill_is_a_hole',
          nid.size == 1,
          f'{nid.size} capsule(s) for a net-less 0.6mm pad over a 0.6mm drill '
          f'(must be 1)')
    nid = _foreign_hole_capsules(_board(0.6, 0.6, net_id=3))[0]
    check('t_a_netted_pad_equal_to_its_drill_stays_copper',
          nid.size == 0,
          f'{nid.size} capsule(s) for a GND 0.6mm pad over a 0.6mm drill '
          f'(must be 0)')
    npth = _board(0.6, 0.6, pad_type='np_thru_hole').footprints['U4'].pads[0]
    check('t_an_npth_hole_is_never_a_peg', not flush_drill_peg(npth),
          'flush_drill_peg is True for an NPTH 0.6/0.6 pad (must be False)')
    nid = _foreign_hole_capsules(_board(0.001, 2.5, net_id=3))[0]
    check('t_a_netted_speck_is_still_a_hole',
          nid.size == 1,
          f'{nid.size} capsule(s) for a GND 0.001mm pad over a 2.5mm drill '
          f'(must be 1)')
    holes = via_floor_npth_holes(_board(0.6, 0.6), set())
    check('t_the_plane_via_maps_hold_the_peg',
          len(holes) == 1, f'{len(holes)} via-floor hole(s) for the peg (must be 1)')


def t_the_router_holds_a_peg_at_the_board_hole_clearance():
    """#1753, the router: a 0.10mm-clearance track keeps 0.30 (hole) + 0.05
    (half-width) + 0.20 = 0.55mm off a 0.6/0.6 peg's centre on a board
    declaring a 0.20 hole clearance, and the copper obstacle's 0.45 when it
    declares no more than the clearance (that map is unchanged)."""
    from routing_config import GridRouteConfig
    from obstacle_map import build_base_obstacle_map
    for hc, reach in ((0.1, 0.45), (0.2, 0.55)):
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.1,
                              clearance=0.1, grid_step=0.05, hole_clearance=hc)
        obs = build_base_obstacle_map(_board(0.6, 0.6), cfg, [1])
        inside = obs.is_blocked(int(round((10.0 + reach - 0.025) / 0.05)), 200, 0)
        outside = obs.is_blocked(int(round((10.0 + reach + 0.05) / 0.05)), 200, 0)
        check(f't_the_router_holds_a_peg_at_the_board_hole_clearance ({hc})',
              inside and not outside,
              f'blocked {reach - 0.025:.3f}mm off centre: {inside} (must be True); '
              f'{reach + 0.05:.3f}mm: {outside} (must be False)')


def t_the_grader_holds_a_peg_at_the_board_hole_clearance():
    """#1753, the grader: KiCad's copper-to-hole rule, max(clearance, the
    board's hole clearance) -- not the NPTH 0.20 track fab floor, which KiCad
    does not apply. A track whose edge stands 0.15mm off the peg's wall at a
    0.10 clearance is flagged when the board declares 0.20 and is clean when it
    declares 0.10 (the copper check already grades that distance)."""
    import tempfile
    from kicad_parser import Segment, Net
    from check_drc import run_drc
    got = {}
    for hc in (0.1, 0.2):
        pcb = _board(0.6, 0.6)
        pcb.segments = [Segment(start_x=8.0, start_y=10.5, end_x=12.0, end_y=10.5,
                                width=0.1, layer='F.Cu', net_id=1)]
        pcb.nets = {1: Net(net_id=1, name='/SIG', pads=[])}
        out = os.path.join(tempfile.mkdtemp(), 'b.kicad_pcb')
        got[hc] = [v['type'] for v in run_drc(
            out, clearance=0.1, hole_clearance=hc, quiet=True, print_summary=False,
            check_sizes=False, clearance_margin=0.0, pcb_data=pcb)]
    check('t_the_grader_holds_a_peg_at_the_board_hole_clearance',
          got[0.2] == ['track-hole'] and got[0.1] == [],
          f'hole clearance 0.20 -> {got[0.2]} (must be one track-hole); '
          f'0.10 -> {got[0.1]} (must be none)')


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

    They branch on `kicad_parser.plated_drill_exposed` (#441) and
    `flush_drill_peg` (#1753), so the test that drifted is two functions.
    This pins each site to both and refuses an inline copper-vs-drill
    comparison, the shape that drifted before.
    """
    import re
    root = os.path.join(os.path.dirname(__file__), '..', 'py_router')
    calls = [re.compile(r'\bplated_drill_exposed\('), re.compile(r'\bflush_drill_peg\(')]
    inline = re.compile(r'max\((\w+)\.size_x,\s*\1\.size_y\)\s*[<>]=?\s*\1\.drill')
    # obstacle_map: the signal map and the plane via maps (#1514);
    # single_ended_routing: the movers and the neck's exact standoffs.
    for fname, who, sites in (('obstacle_map.py', 'the router\'s maps', 2),
                              ('check_drc.py', 'the grader', 1),
                              ('single_ended_routing.py', 'the copper-movers', 2)):
        with open(os.path.join(root, fname)) as f:
            src = f.read()
        found = [len(c.findall(src)) for c in calls]
        left = len(inline.findall(src))
        check(f't_{fname.split(".")[0]}_uses_the_ring_uncovered_test',
              found == [sites, sites] and left == 0,
              f'{who}: {found[0]}/{sites} plated_drill_exposed and '
              f'{found[1]}/{sites} flush_drill_peg call(s); '
              f'{left} inline copper-vs-drill comparison(s)')


def main():
    t_a_ring_uncovered_plated_hole_is_seen()
    t_copper_equal_to_its_drill_is_a_hole()
    t_the_router_holds_a_peg_at_the_board_hole_clearance()
    t_the_grader_holds_a_peg_at_the_board_hole_clearance()
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
