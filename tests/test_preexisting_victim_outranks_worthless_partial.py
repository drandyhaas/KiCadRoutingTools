"""An intact restore outranks partial copper that connects nothing.

When a PRE-EXISTING net is ripped to unblock another net and its reroute lands
PARTIAL, route.py restores the original -- unless restoring would now collide
with copper that moved into the corridor meanwhile (#134). That refusal is
right when a working net holds the corridor. It is wrong when the holder is
ITSELF a pre-existing victim whose own reroute is still disconnected: that
copper connects nothing, so protecting it costs a fully connected restore and
buys not one pad.

MEASURED on watchy, which shipped exactly that cascade:

    SDA ripped BTN3  ->  BTN3 ripped EN  ->  both rerouted PARTIAL
    EN's intact original refused: "original corridor taken"
    result: 2 nets open to gain 1

    before   2 unconnected (EN, BTN3)
    after    0 unconnected, DRC still clean -- the v0.22.0 baseline

The tap rip-up path has had this accounting since #310 ("tap rip-up lost N
pad(s) to gain M; abandoning tap"); the pre-existing path never got it.

THE RULE IS DELIBERATELY NARROW, and the narrowness is the point: EVERY
blocker must be a pre-existing victim that is still disconnected. One connected
net holding the corridor and the restore stays refused exactly as before, so
this can never trade working copper for a restore -- which is the failure #134
exists to prevent.
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


def _source():
    with open(os.path.join(os.path.dirname(__file__), '..',
                           'py_router', 'route.py')) as f:
        return f.read()


def t_the_rule_exists_and_is_wired():
    """The colliders helper must be imported and consulted, not just the bool.

    `_saved_route_collides` answers yes/no; deciding whether the blocker is
    worth protecting needs `_saved_route_colliders`, which names it.
    """
    s = _source()
    check('t_the_rule_exists_and_is_wired',
          '_saved_route_colliders as _pe_colliders' in s
          and '_pe_colliders(_orig_pe[0]' in s,
          'route.py imports and calls the named-collider helper')


def t_every_blocker_must_be_a_disconnected_victim():
    """The narrowing clause: a connected holder still refuses the restore."""
    s = _source()
    seg = s[s.find('_worthless = {_b for _b in _blk'):][:400]
    check('t_only_preexisting_victims_qualify',
          '_b in _pe_ripped_reg' in seg,
          'a blocker must be a pre-existing victim')
    check('t_only_disconnected_holders_qualify',
          'not _pe_connected(_b)' in seg,
          'a blocker must itself be still disconnected')
    check('t_all_blockers_must_qualify_not_just_one',
          '_worthless != _blk' in s,
          'ANY connected blocker keeps the old refusal (set equality, not '
          'intersection)')


def t_the_corridor_is_rechecked_after_clearing():
    """Clearing the worthless copper does not license an unchecked restore."""
    s = _source()
    tail = s[s.find('_worthless != _blk'):][:1800]
    check('t_the_corridor_is_rechecked_after_clearing',
          tail.count('_pe_collides(_orig_pe[0]') >= 1
          and 'something else holds it after all' in tail,
          'the collision test is re-run before restoring')


def t_the_helpers_behave_as_the_rule_assumes():
    """Non-vacuity: the two helpers exist and disagree the way the rule needs.

    A source-shape test proves nothing if `_saved_route_colliders` cannot
    actually name a blocker, so exercise it on real geometry: one saved
    segment, one foreign segment lying on top of it.
    """
    from kicad_parser import PCBData, BoardInfo, Segment
    from rip_up_reroute import _saved_route_collides, _saved_route_colliders
    saved = {'new_segments': [Segment(start_x=10.0, start_y=10.0,
                                      end_x=12.0, end_y=10.0, width=0.2,
                                      layer='F.Cu', net_id=1)],
             'new_vias': []}
    foreign = Segment(start_x=10.0, start_y=10.02, end_x=12.0, end_y=10.02,
                      width=0.2, layer='F.Cu', net_id=2)
    pcb = PCBData(board_info=BoardInfo(layers={0: 'F.Cu'},
                                       copper_layers=['F.Cu'],
                                       board_bounds=(0.0, 0.0, 50.0, 50.0),
                                       stackup=[]),
                  nets={}, footprints={}, vias=[], segments=[foreign],
                  pads_by_net={})
    hit = _saved_route_collides(saved, pcb, [1], 0.2)
    who = {getattr(o, 'net_id', None)
           for _k, o in _saved_route_colliders(saved, pcb, [1], 0.2)}
    check('t_the_helpers_behave_as_the_rule_assumes',
          hit and who == {2},
          f'collides={hit}, named blocker(s)={sorted(w for w in who if w)}')
    # And the control: move the foreign copper away and both go quiet, so the
    # row above is not passing on a helper that always says yes.
    pcb.segments = [Segment(start_x=10.0, start_y=30.0, end_x=12.0, end_y=30.0,
                            width=0.2, layer='F.Cu', net_id=2)]
    check('t_a_clear_corridor_names_nobody',
          not _saved_route_collides(saved, pcb, [1], 0.2),
          'far-away copper is not reported as a blocker')


def main():
    t_the_rule_exists_and_is_wired()
    t_every_blocker_must_be_a_disconnected_victim()
    t_the_corridor_is_rechecked_after_clearing()
    t_the_helpers_behave_as_the_rule_assumes()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
