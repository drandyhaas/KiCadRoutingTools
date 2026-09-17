"""A copper-moving pass must treat an NPTH SLOT as board edge (#448/#536).

KiCad's edge provider grades copper against an NPTH SLOT's wall at
`copper_edge_clearance`, while a ROUND NPTH drill stays in the copper-to-hole
domain. check_drc has modelled that since #448. The passes that MOVE committed
copper -- the octolinear smoother and the two grazing nudges -- did not: their
`edge_clears` consulted only the Edge.Cuts rings, which contain no slots, so a
slot's only floor was the hole term's `max(clearance, NPTH_TO_TRACK_CLEARANCE)`.
On any board whose edge clearance exceeds that floor, a mover could legally
place copper inside the band the grader enforces.

MEASURED on sofle_pico (`--board-edge-clearance 0.3`, track 0.25, clearance
0.2), net `row3` beside SW25's 2.8x1.5 slot:

    v0.22.0   the run jogs and finishes at x=101.850
    HEAD      the smoother straightens 12.6mm of it onto x=101.750

              mover's floor   0.2 + 0.125 = 0.325mm   -> legal
              grader's floor  0.3 + 0.125 = 0.425mm   -> VIOLATION
              shipped                       0.350mm   -> 0.075mm overlap

The #958 second greedy phase (fewer segments at equal length) is what made the
straightening available; it is not what made it illegal. The mover was always
free to do this -- nothing had asked it about slots.

THE FIX IS ONE SOURCE, NOT THREE COPIES: `npth_slot_capsules` and
`segment_to_npth_slots_distance` live in check_drc next to
`board_edge_geometry`, and every mover calls them. A mover that mirrors the
checker's geometry is a mover that will drift from it.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []


def _helpers():
    """The two exported helpers, or (None, None) if this build lacks them.

    A bare `from check_drc import npth_slot_capsules` raises ImportError on a
    tree without the fix, and an ImportError is a BROKEN TEST, not a detected
    regression -- the two exit the same way and only one of them is evidence.
    Absence IS the finding here, so it is reported as one.
    """
    try:
        from check_drc import npth_slot_capsules, segment_to_npth_slots_distance
        return npth_slot_capsules, segment_to_npth_slots_distance
    except ImportError:
        return None, None


def _require_helpers(name):
    caps, dist = _helpers()
    if caps is None:
        check(name, False,
              'check_drc exports no npth_slot_capsules / '
              'segment_to_npth_slots_distance -- the movers cannot be grading '
              'slots as board edge, because there is nothing to grade them with')
    return caps, dist


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _pcb_with(pads):
    from kicad_parser import PCBData, BoardInfo, Footprint
    fp = Footprint(reference='SW25', footprint_name='t:sw', x=0.0, y=0.0,
                   rotation=0.0, layer='F.Cu', pads=pads)
    return PCBData(board_info=BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
                                        board_bounds=(0.0, 0.0, 50.0, 50.0),
                                        stackup=[]),
                   nets={}, footprints={'SW25': fp}, vias=[], segments=[],
                   pads_by_net={})


def _npth(x, y, w, h):
    """An NPTH pad whose drill is `w` x `h` (a slot when w != h)."""
    from kicad_parser import Pad
    p = Pad(pad_number='1', net_id=0, net_name='', global_x=x, global_y=y,
            local_x=x, local_y=y, size_x=w, size_y=h, shape='oval',
            layers=['*.Cu'], drill=max(w, h), pad_type='np_thru_hole',
            component_ref='SW25')
    p.drill_w, p.drill_h = w, h
    return p


def t_a_slot_is_a_capsule_and_a_round_hole_is_not():
    """Non-vacuity + the discrimination the whole rule rests on."""
    npth_slot_capsules, _ = _require_helpers(
        't_a_slot_is_reported_as_a_capsule')
    if npth_slot_capsules is None:
        return
    slot = _pcb_with([_npth(10.0, 10.0, 2.8, 1.5)])
    round_ = _pcb_with([_npth(10.0, 10.0, 3.0, 3.0)])
    caps_slot = npth_slot_capsules(slot)
    caps_round = npth_slot_capsules(round_)
    check('t_a_slot_is_reported_as_a_capsule',
          len(caps_slot) == 1,
          f'{len(caps_slot)} capsule(s) for a 2.8x1.5 slot')
    check('t_a_round_npth_is_not_milled_edge',
          not caps_round,
          f'{len(caps_round)} capsule(s) for a round 3.0mm NPTH (must be 0)')
    if caps_slot:
        (p1, p2, r, ref) = caps_slot[0]
        axis = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        check('t_the_capsule_has_the_slot_geometry',
              abs(axis - (2.8 - 1.5)) < 1e-6 and abs(r - 0.75) < 1e-6
              and ref == 'SW25.1',
              f'axis {axis:.3f}mm, radius {r:.3f}mm, ref {ref}')


def t_the_distance_is_measured_to_the_wall():
    """The helper returns centreline-to-WALL, which is what the floors are on."""
    npth_slot_capsules, segment_to_npth_slots_distance = _require_helpers(
        't_the_distance_is_measured_to_the_wall')
    if npth_slot_capsules is None:
        return
    caps = npth_slot_capsules(_pcb_with([_npth(10.0, 10.0, 2.8, 1.5)]))
    # A track running parallel, 2.0mm above the slot's axis: wall is 0.75 away.
    d = segment_to_npth_slots_distance(caps, 5.0, 12.0, 15.0, 12.0)
    check('t_the_distance_is_measured_to_the_wall',
          abs(d - (2.0 - 0.75)) < 1e-6,
          f'{d:.4f}mm to the wall (2.0mm to the axis, radius 0.75mm)')
    check('t_no_slots_means_no_constraint',
          segment_to_npth_slots_distance([], 5.0, 12.0, 15.0, 12.0) == float('inf'),
          'a slot-less board returns +inf rather than blocking everything')


def t_the_movers_ask_about_slots_at_the_edge_floor():
    """Wiring: each mover's edge_clears consults the slot distance.

    Read off the AST, and required to sit inside the function that owns the
    `_edge_clr` floor -- a call somewhere else in the file would prove nothing
    about which floor the slot is graded at.
    """
    import ast
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'pcb_modification.py')).read()
    tree = ast.parse(src)
    lines = src.splitlines()
    want = {'smooth_octolinear_chains', 'nudge_grazing_octolinear',
            'nudge_grazing_microshift'}
    seen = {}
    for fn in ast.walk(tree):
        if isinstance(fn, ast.FunctionDef) and fn.name in want:
            body = "\n".join(lines[fn.lineno - 1:getattr(fn, 'end_lineno', fn.lineno)])
            seen[fn.name] = ('segment_to_npth_slots_distance' in body
                             and 'npth_slot_capsules' in body
                             and '_edge_clr' in body)
    for name in sorted(want):
        check(f't_{name}_grades_slots_as_edge',
              seen.get(name) is True,
              'consults the shared slot distance beside its own edge floor'
              if seen.get(name) else
              f'{name}: {"missing the slot check" if name in seen else "FUNCTION NOT FOUND"}')


def t_the_checker_uses_the_same_source():
    """check_drc must not keep a private copy of the geometry it exports."""
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'check_drc.py')).read()
    body = src[src.index('def check_drc_violations'):] if 'def check_drc_violations' in src else src
    check('t_the_checker_uses_the_same_source',
          'npth_slot_capsules(pcb_data)' in body
          and "'np_thru_hole' or _pd.drill" not in body,
          'the slot-vs-edge grade reads the exported helper, not an inline copy')


def t_the_sofle_geometry_is_decided_the_way_the_grader_decides_it():
    """The measured case, as numbers rather than as a story.

    Slot wall at 0.350mm from the centreline of a 0.25mm track: legal under the
    NPTH floor (0.325mm), illegal under the 0.3mm edge floor (0.425mm). The
    mover must take the second answer.
    """
    npth_slot_capsules, segment_to_npth_slots_distance = _require_helpers(
        't_the_edge_floor_refuses_it')
    if npth_slot_capsules is None:
        return
    from routing_defaults import NPTH_TO_TRACK_CLEARANCE
    caps = npth_slot_capsules(_pcb_with([_npth(10.0, 10.0, 2.8, 1.5)]))
    half, clearance, edge = 0.125, 0.2, 0.3
    # Place the track so its centreline is exactly 0.350mm from the wall.
    y = 10.0 + 0.75 + 0.350
    d = segment_to_npth_slots_distance(caps, 5.0, y, 15.0, y)
    npth_floor = max(clearance, NPTH_TO_TRACK_CLEARANCE) + half
    edge_floor = max(clearance, edge) + half
    check('t_the_npth_floor_would_have_allowed_it',
          d >= npth_floor - 1e-9,
          f'{d:.4f}mm >= {npth_floor:.4f}mm (the floor that let it ship)')
    check('t_the_edge_floor_refuses_it',
          d < edge_floor,
          f'{d:.4f}mm < {edge_floor:.4f}mm (the floor the grader enforces)')


def main():
    t_a_slot_is_a_capsule_and_a_round_hole_is_not()
    t_the_distance_is_measured_to_the_wall()
    t_the_movers_ask_about_slots_at_the_edge_floor()
    t_the_checker_uses_the_same_source()
    t_the_sofle_geometry_is_decided_the_way_the_grader_decides_it()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
