"""The design rules as the ROUTER applies them on one board, read off that board through py_router's own resolvers --
one resolver, never a second (rules.py reads no board: py_router owns that, and two resolvers would be two chances to
disagree about one board).

- hole_track_clear / board_hole_clear: how far copper keeps off a hole with no copper of its own (an NPTH: a mounting
  hole, a peg). The router's NPTH keep-out (obstacle_map) is the largest of the clearance, the fab's NPTH-to-track floor
  and the board's own copper-to-hole rule (obstacle_map.resolve_hole_clearance); the plan, the geometry, the census,
  the audit and the lay held lanes to the clearance alone there, and the router refused what they had passed.
- fan_via_at: the via the escape fanout lays under a part, stepped down the fab ladder where its plane pours could not
  pass between two of them."""
import contextlib
import io
import types

import rules as _rules


def hole_track_clear(pcb, clearance=None):
    """a track's copper off the wall of a hole with no copper on `pcb`: the clearance (the run's, else the active
    rules'), the fab's NPTH-to-track floor and the board's copper-to-hole rule, the largest"""
    import routing_defaults as _rd          # (py_router's: on the path once a board has been read)
    clr = _rules.active().clearance if clearance is None else clearance
    return max(clr, _rd.NPTH_TO_TRACK_CLEARANCE, board_hole_clear(pcb))


def board_hole_clear(pcb):
    """the board's own copper-to-hole rule (its min_hole_clearance, as declared before the chain lowered it), 0 where it
    declares none -- the floor a via's copper keeps off a hole with no copper, above the clearance, as the router keeps
    it. Read once per board"""
    if '_board_hole_clear' not in pcb.__dict__:
        from obstacle_map import resolve_hole_clearance
        # (quietly: the resolver says what it read, and an audit's output is read line by line)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb.__dict__['_board_hole_clear'] = \
                resolve_hole_clearance(pcb, types.SimpleNamespace(hole_clearance=0.0)) or 0.0
    return pcb.__dict__['_board_hole_clear']


def fan_via_at(pcb, fp, rules=None):
    """(via, drill) the escape fanout lays under footprint `fp`: the rules' via, stepped down the fab ladder where the
    array's plane pours could not pass between two of them a pitch apart -- the engine's own choice for every via of
    that part's fanout (bga_fanout: plane_lattice.plane_web_via, at the fanout's clearance, on the run's escalation
    rungs). The plan prices its moves' vias at it: priced at the via asked, a 0.65 mm array's via-in-pad 0.325 off the
    next column's run read as a conflict the engine's 0.3 mm via clears, and a strobe pair's teeth had no state free of
    one -- the teeth as laid kept, two pairs' legs on two layers and no pair body between them"""
    rules = rules or _rules.active()
    memo = pcb.__dict__.setdefault('_fan_via_at', {})
    key = (getattr(fp, 'reference', None), rules.via_size, rules.via_drill, rules.clearance)
    if key not in memo:
        from list_nets import escalation_rungs
        from plane_lattice import plane_web_via
        w = plane_web_via(fp, pcb, rules.via_size, rules.via_drill, rules.clearance,
                          escalation_rungs(len(pcb.board_info.copper_layers or ()) or 4))
        memo[key] = (w['via'], w['drill']) if w is not None and w['threads'] else (rules.via_size, rules.via_drill)
    return memo[key]
