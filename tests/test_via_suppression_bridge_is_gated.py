"""A via-suppression bridge is capped, floored, and clearance-checked.

When two vias on one region-join strap land closer than
`via_drill + hole_to_hole`, one is suppressed and `_bridge_to_survivor` joins
the orphaned transition point to the survivor on BOTH transition layers, so the
strap is not severed there (#508 finding 14).

It used to draw that joint at the strap's FULL width and never clearance-check
it. Both halves were wrong, and on zynq_ad9364 they combined into nine
segment-segment violations that are SHORTS, not grazes:

    TX_D1_N <-> RFGND   F.Cu   centre distance 0.300mm, copper gap -0.160mm

The RFGND side is one 0.057mm segment at 0.8mm wide -- a disc, for a joint
between two barrels a fraction of a via apart -- laid across TX_D1_N, a
PROTECTED diff-pair member (#521, never rippable) whose copper is already in
the step's own input. `wide_route_clear` answers False for that leg at EVERY
width in the ladder against both the input and the shipped board, so the
corridor was never legal; nothing ever asked, because the bridge is not a
`route_points` leg and lands on the TRANSITION layers, not the layers the strap
was routed and gated on.

Measured with the fix, on the recorded chain: 11 -> 2 DRC, all nine shorts
gone, and ZERO bridges skipped -- the width cap alone was sufficient, so no
plane connectivity was traded for it.

Run with:  python3 tests/test_via_suppression_bridge_is_gated.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from kicad_parser import BoardInfo, Net, PCBData, Segment  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


# The real geometry, to the 0.001mm, from the zynq_ad9364 run.
NODE, VIA = (88.400, -54.500), (88.354, -54.534)
LEG_F = [(NODE[0], NODE[1], 'F.Cu'), (VIA[0], VIA[1], 'F.Cu')]
STRAP_W, VIA_SIZE, MIN_W = 0.8, 0.45, 0.2


def _cfg(clearance=0.2):
    from routing_config import GridRouteConfig
    return GridRouteConfig(clearance=clearance, track_width=0.15,
                           via_size=VIA_SIZE, grid_step=0.1)


def _pcb(victim_segs=()):
    return PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                             copper_layers=['F.Cu', 'B.Cu'],
                             board_bounds=(80.0, -60.0, 95.0, -50.0), stackup=[]),
        nets={19: Net(net_id=19, name='RFGND'),
              36: Net(net_id=36, name='TX_D1_N')},
        footprints={}, vias=[], segments=list(victim_segs), pads_by_net={})


def _victim():
    """TX_D1_N where the run really has it: centrelines ~0.30mm from the leg."""
    return [Segment(start_x=88.70, start_y=-54.58, end_x=88.68, end_y=-54.61,
                    width=0.12, layer='F.Cu', net_id=36),
            Segment(start_x=88.68, start_y=-54.61, end_x=88.68, end_y=-54.68,
                    width=0.12, layer='F.Cu', net_id=36)]


def t_the_strap_width_is_never_used_for_the_joint():
    """The cap. In open space the bridge still must not be drawn at 0.8mm."""
    from plane_region_connector import via_bridge_width
    w = via_bridge_width(LEG_F, STRAP_W, VIA_SIZE, MIN_W, _pcb(), 19, _cfg())
    check('t_the_strap_width_is_never_used_for_the_joint',
          w is not None and w <= VIA_SIZE + 1e-9,
          f'width {w} is capped at the via diameter {VIA_SIZE}, not the '
          f'strap\'s {STRAP_W}')


def t_it_is_floored_at_the_declared_minimum():
    """The other end of the cap: an advanced-tier via smaller than the run's
    own minimum must not silently thin the joint below what was asked for --
    this repo discloses every narrowing (design_rules.narrowed)."""
    from plane_region_connector import via_bridge_width
    w = via_bridge_width(LEG_F, STRAP_W, 0.25, 0.30, _pcb(), 19, _cfg())
    check('t_it_is_floored_at_the_declared_minimum',
          w is not None and w >= 0.30 - 1e-9,
          f'width {w} >= min_track_width 0.30 even though via_size is 0.25')


def t_the_real_short_is_refused():
    """The bug. With TX_D1_N's copper present, NO width in the ladder clears,
    so the bridge is skipped rather than drawn."""
    from plane_region_connector import via_bridge_width
    w = via_bridge_width(LEG_F, STRAP_W, VIA_SIZE, MIN_W,
                         _pcb(_victim()), 19, _cfg())
    check('t_the_real_short_is_refused', w is None,
          'no width clears TX_D1_N, so the bridge is skipped (the strap stays '
          'split and the run reports it) rather than shipping a short')


def t_it_narrows_before_it_gives_up():
    """Between "draw it fat" and "skip it" there is a middle: a leg that the
    cap cannot clear but min_track_width can must be drawn narrow, not
    abandoned. Otherwise the fix trades shorts for split planes."""
    from plane_region_connector import via_bridge_width
    # A victim far enough that 0.2 clears and 0.45 does not.
    v = [Segment(start_x=88.0, start_y=-54.85, end_x=89.0, end_y=-54.85,
                 width=0.12, layer='F.Cu', net_id=36)]
    cfg = _cfg(clearance=0.05)
    w = via_bridge_width(LEG_F, STRAP_W, VIA_SIZE, MIN_W, _pcb(v), 19, cfg)
    wide_ok = via_bridge_width(LEG_F, STRAP_W, VIA_SIZE, VIA_SIZE,
                               _pcb(v), 19, cfg)
    check('t_it_narrows_before_it_gives_up',
          w == MIN_W and wide_ok is None,
          f'narrowed to {w} where the capped width was refused '
          f'(cap-only answer: {wide_ok})')


def t_no_model_means_no_veto():
    """A caller with no pcb_data/net_id still gets a bridge -- the gate may
    tighten the width, never withhold copper it cannot assess."""
    from plane_region_connector import via_bridge_width
    a = via_bridge_width(LEG_F, STRAP_W, VIA_SIZE, MIN_W, None, 19, _cfg())
    b = via_bridge_width(LEG_F, STRAP_W, VIA_SIZE, MIN_W, _pcb(), None, _cfg())
    check('t_no_model_means_no_veto',
          a is not None and b is not None,
          f'unassessable callers still bridge (pcb_data=None -> {a}, '
          f'net_id=None -> {b})')


def t_the_emitter_routes_through_the_core():
    """Wiring: the bridge site must call it, or this gate is inert."""
    import ast
    src = open(os.path.join(ROOT_DIR, 'py_router',
                            'plane_region_connector.py')).read()
    called = any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == 'via_bridge_width'
                 for n in ast.walk(ast.parse(src)))
    check('t_the_emitter_routes_through_the_core', called,
          '_bridge_to_survivor decides its width via via_bridge_width')


def main():
    t_the_strap_width_is_never_used_for_the_joint()
    t_it_is_floored_at_the_declared_minimum()
    t_the_real_short_is_refused()
    t_it_narrows_before_it_gives_up()
    t_no_model_means_no_veto()
    t_the_emitter_routes_through_the_core()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
