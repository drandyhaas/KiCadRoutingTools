"""--force-reroute's restore must refuse stale copper that would short (#134).

`--force-reroute` strips every named net up front, so those nets contend for
one corridor: whichever routes first may legitimately occupy space a later one
used to hold. When a net's replan then lands NO copper, route.py gives it its
ORIGINAL copper back -- and used to do so VERBATIM, with no check against what
had meanwhile been routed there.

Measured on ecp5_mini step 7 (`route.py ... /PE26+ /PE26- --rip-existing-nets
/PE26+ /PE26- --force-reroute`): /PE26+ and /PE26- are both stripped, /PE26+
routes a via into the vacated corridor at (130.20, 85.10), /PE26- fails every
rescue, and its original via returns at (130.30, 85.10). 0.1 mm apart:

    VIA-VIA          /PE26- <-> /PE26+  overlap 0.500mm  (CONTACT)
    VIA-DRILL-HOLE   /PE26- <-> /PE26+  overlap 0.400mm  (CONTACT)
    VIA-SEG x3       /PE26- <-> /PE26+  overlap 0.053..0.213mm

Five DRC on a board that was otherwise clean, and the net was reported FAILED
anyway -- the copper was both shorted and disconnected. The same run's log
shows the rip/reroute path getting this right two lines earlier:

    restore skipped (net 33): saved copper would short other-net copper; left ripped (#134)

so the predicate existed; this site just never called it.

WHAT IS NOT CHANGED, and what the non-vacuity rows here pin: a restore that
does NOT collide still happens. The point of the restore is that a failed
replan must not be paid for by deleting a working route, and narrowing it to
"never restore" would trade 5 DRC for a pile of open nets.

Run with:  python3 tests/test_force_reroute_restore_is_collision_aware.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from kicad_parser import BoardInfo, Net, PCBData, Segment, Via  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _via(x, y, net_id):
    return Via(x=x, y=y, size=0.5, drill=0.25, layers=['F.Cu', 'B.Cu'],
               net_id=net_id, uuid='')


def _seg(x0, y0, x1, y1, net_id, layer='F.Cu'):
    return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=0.127,
                   layer=layer, net_id=net_id)


def _pcb(segments=(), vias=()):
    return PCBData(
        board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                             copper_layers=['F.Cu', 'B.Cu'],
                             board_bounds=(120.0, 80.0, 140.0, 95.0),
                             stackup=[]),
        nets={35: Net(net_id=35, name='/PE26+'),
              36: Net(net_id=36, name='/PE26-')},
        footprints={}, vias=list(vias), segments=list(segments), pads_by_net={})


# The real geometry, to the 0.01 mm, from the ecp5_mini run.
PLUS_VIA = (130.20, 85.10)
MINUS_VIA = (130.30, 85.10)


def t_the_colliding_restore_is_refused():
    """The bug. /PE26+ owns (130.20, 85.10); /PE26-'s saved via at
    (130.30, 85.10) must NOT come back."""
    from rip_up_reroute import partition_force_restores
    pcb = _pcb(vias=[_via(*PLUS_VIA, 35)])
    saved_via = _via(*MINUS_VIA, 36)
    restored, refused = partition_force_restores(
        {36: ([], [saved_via])}, pcb, clearance=0.1)
    check('t_the_colliding_restore_is_refused',
          restored == [] and refused == [36],
          f'restored={restored} refused={refused}')
    check('t_the_colliding_copper_does_not_reach_the_board',
          all(v is not saved_via for v in pcb.vias) and len(pcb.vias) == 1,
          f'pcb_data still holds only /PE26+\'s via ({len(pcb.vias)} via(s))')


def t_a_clear_restore_still_happens():
    """Non-vacuity, and the intent guard. A rule that refused everything would
    pass the row above and be a worse bug than the one it fixes."""
    from rip_up_reroute import partition_force_restores
    pcb = _pcb(vias=[_via(*PLUS_VIA, 35)])
    saved_via = _via(135.0, 85.10, 36)          # 4.8 mm away: no conflict
    saved_seg = _seg(135.0, 85.10, 137.0, 85.10, 36)
    restored, refused = partition_force_restores(
        {36: ([saved_seg], [saved_via])}, pcb, clearance=0.1)
    check('t_a_clear_restore_still_happens',
          restored == [36] and refused == [],
          f'restored={restored} refused={refused}')
    check('t_a_clear_restore_reaches_the_board',
          any(v is saved_via for v in pcb.vias)
          and any(s is saved_seg for s in pcb.segments),
          'the saved objects themselves are back in pcb_data (identity '
          'preserved, so the #220 stale strip keeps them)')


def t_a_net_whose_replan_landed_copper_is_not_a_candidate():
    """skip_net_ids: a partial replan keeps its NEW copper; stacking the
    originals on top is what the original comment refuses."""
    from rip_up_reroute import partition_force_restores
    pcb = _pcb()
    restored, refused = partition_force_restores(
        {36: ([], [_via(135.0, 85.10, 36)])}, pcb, clearance=0.1,
        skip_net_ids={36})
    check('t_a_net_whose_replan_landed_copper_is_not_a_candidate',
          restored == [] and refused == [] and not pcb.vias,
          f'neither restored nor refused (restored={restored} '
          f'refused={refused}, {len(pcb.vias)} via(s) on the board)')


def t_restores_are_tested_against_earlier_restores():
    """Order is load-bearing: two refused nets must not be re-admitted on top
    of each other. The second is tested against a board that already has the
    first."""
    from rip_up_reroute import partition_force_restores
    pcb = _pcb()
    a, b = _via(135.0, 85.10, 35), _via(135.05, 85.10, 36)
    restored, refused = partition_force_restores(
        {35: ([], [a]), 36: ([], [b])}, pcb, clearance=0.1)
    check('t_restores_are_tested_against_earlier_restores',
          restored == [35] and refused == [36],
          f'the first lands, the second is refused against it '
          f'(restored={restored} refused={refused})')


def t_own_net_copper_is_not_a_collision():
    """A net's own copper already on the board must not veto its own restore --
    otherwise a net with any surviving stub could never be restored."""
    from rip_up_reroute import partition_force_restores
    pcb = _pcb(vias=[_via(135.0, 85.10, 36)])
    saved = _via(135.05, 85.10, 36)             # same net, overlapping
    restored, refused = partition_force_restores(
        {36: ([], [saved])}, pcb, clearance=0.1)
    check('t_own_net_copper_is_not_a_collision',
          restored == [36] and refused == [],
          f'restored={restored} refused={refused}')


def t_route_py_calls_the_core():
    """Wiring, asked of the AST: the core is only worth anything if the
    --force-reroute block actually routes through it."""
    import ast
    src = open(os.path.join(ROOT_DIR, 'py_router', 'route.py')).read()
    called = any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                 and n.func.id == 'partition_force_restores'
                 for n in ast.walk(ast.parse(src)))
    check('t_route_py_calls_the_core', called,
          'route.py calls partition_force_restores rather than appending the '
          'saved copper itself')


def main():
    t_the_colliding_restore_is_refused()
    t_a_clear_restore_still_happens()
    t_a_net_whose_replan_landed_copper_is_not_a_candidate()
    t_restores_are_tested_against_earlier_restores()
    t_own_net_copper_is_not_a_collision()
    t_route_py_calls_the_core()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
