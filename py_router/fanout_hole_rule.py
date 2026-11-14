"""A `.kicad_dru` hole_clearance rule for the fan-out engines (#1853).

KiCad holds copper off the HOLE of another net's via or pad at the last
hole_clearance rule binding the pair (`check_drc.hole_rule_grade`). The
escape engines space their copper at a copper clearance, and a via site is
asked of `check_drc.HoleRuleGate` before it is drilled, but a track the
engine lays later can still pass a via's hole inside the rule: an escape
threading two rows of dogbone vias stands half the gap between their holes
off each.

So the call's copper -- every track and via it lays, the plane drops
included -- is graded here with the grader itself, against the board and
against itself, once the engines are done:

1. `hole_violations` names the copper of this call in a row: the copper side
   of a pair (the track or via standing too near a hole), or the via whose
   hole it is when the copper was on the board before.
2. `hole_reescape` (the BGA escape) takes off the escape of every ball that
   copper belongs to (a chain of touching copper comes off whole), escapes
   those balls again at the copper clearance that holds the rule at a via's
   ring (rule - (via_size - via_drill) / 2), grades that escape the same way
   and takes off what still falls short. Those balls are reported unescaped.
3. `hole_takeoff` (the QFN fan-out, whose escapes are one stub or one drop
   each) takes the escapes off and reports their nets unescaped, for the
   route step, which holds the rule, to connect.

Free on a board with no hole_clearance rule.
"""
from __future__ import annotations

from typing import List, Tuple


def _key_via(x, y, net):
    return ('v', round(x, 6), round(y, 6), int(net or 0))


def _key_seg(ax, ay, bx, by, layer, net):
    return ('s', round(ax, 6), round(ay, 6), round(bx, 6), round(by, 6), layer, int(net or 0))


def hole_violations(pcb_data, dr, tracks, vias, fixed=(0, 0)) -> List[Tuple[str, int]]:
    """The refs (('t', i) / ('v', i) into `tracks` / `vias`, the engines'
    dicts) of this call's copper in a hole_clearance row. `fixed` =
    (n_tracks, n_vias): leading items already kept, never named; a row
    between one of them and newer copper names the newer side."""
    from check_drc import (HoleRuleGate, hole_rule_grade, _HoleGateBoard,
                           _HoleGatePads, _hole_rule_reach)
    if _hole_rule_reach(dr) <= 0.0 or not (tracks or vias):
        return []
    segs = [HoleRuleGate.as_segment(t) for t in tracks]
    vs = [HoleRuleGate.as_via(v) for v in vias]
    seg_at = {_key_seg(s.start_x, s.start_y, s.end_x, s.end_y, s.layer, s.net_id): i
              for i, s in enumerate(segs)}
    via_at = {_key_via(v.x, v.y, v.net_id): i for i, v in enumerate(vs)}
    # the engines append their vias to the board as they go: one copy each
    board_vias = [v for v in pcb_data.vias if _key_via(v.x, v.y, v.net_id) not in via_at]
    board_segs = [s for s in pcb_data.segments
                  if _key_seg(s.start_x, s.start_y, s.end_x, s.end_y, s.layer,
                              s.net_id) not in seg_at]
    board = _HoleGateBoard(pcb_data.nets, board_vias + vs, board_segs + segs,
                           {k: _HoleGatePads(fp.pads) for k, fp in pcb_data.footprints.items()})
    copper = list(getattr(getattr(pcb_data, 'board_info', None), 'copper_layers', None)
                  or dr.copper_layers)
    nets = {s.net_id for s in segs} | {v.net_id for v in vs}
    rows, _unread, _drop = hole_rule_grade(board, dr, copper, 0.0, 0.0, nets)
    name_to_id = {n.name: nid for nid, n in pcb_data.nets.items() if n.name}
    out = []
    for r in rows:
        cnet = name_to_id.get(r.get('net2'), 0)
        hnet = name_to_id.get(r.get('net1'), 0)
        copper_ref = hole_ref = None
        if r.get('seg_loc') is not None:
            i = seg_at.get(_key_seg(*r['seg_loc'], r.get('layer'), cnet))
            copper_ref = ('t', i) if i is not None else None
        elif r.get('via_loc') is not None:
            i = via_at.get(_key_via(*r['via_loc'], cnet))
            copper_ref = ('v', i) if i is not None else None
        if r.get('hole_ref') == 'via':
            i = via_at.get(_key_via(*r['hole_loc'], hnet))
            hole_ref = ('v', i) if i is not None else None

        def _kept(ref):
            return ref is not None and ((ref[0] == 't' and ref[1] < fixed[0])
                                        or (ref[0] == 'v' and ref[1] < fixed[1]))
        for ref in (copper_ref, hole_ref):
            if ref is not None and not _kept(ref):
                out.append(ref)
                break
    return out


def hole_reescape(core, args, result, dr):
    """Grade `result` (tracks, vias, vias_to_remove, failed) against the
    board's hole_clearance rules and re-escape the balls that fall short.
    `core` is the engine entry and `args` its keyword arguments for this
    part; `dr` the board's `design_rules.DesignRules`. See the module
    docstring."""
    from check_drc import _hole_rule_reach
    from bga_fanout.class_clearance import _chains
    from bga_fanout import ball_has_copper, unserved_nets
    from kicad_parser import Segment as _Seg, Via as _Via
    tracks, vias, vias_rm, failed = result
    tracks, vias, failed = list(tracks), list(vias), list(failed)
    reach = _hole_rule_reach(dr)
    viol = hole_violations(args['pcb_data'], dr, tracks, vias)
    if not viol:
        return tracks, vias, vias_rm, failed
    fp, pcb = args['footprint'], args['pcb_data']

    def take_off(tr, vi, refs, keep_from=(0, 0)):
        of, touched = _chains(tr, vi, fp.pads)
        gone = {of[r] for r in refs}
        balls = []
        for c in gone:
            balls += [p for p in touched.get(c, ()) if p not in balls]
        tr = [t for i, t in enumerate(tr) if i < keep_from[0] or of[('t', i)] not in gone]
        vi = [v for i, v in enumerate(vi) if i < keep_from[1] or of[('v', i)] not in gone]
        return tr, vi, balls

    n_rows = len(viol)
    tracks, vias, balls = take_off(tracks, vias, set(viol))
    base = float(args['clearance'])
    ring = (float(args.get('via_size') or 0.0) - float(args.get('via_drill') or 0.0)) / 2.0
    f = round(max(base, reach - max(ring, 0.0)), 6)
    print(f"  hole_clearance (#1853): {n_rows} row(s) of this escape stood inside "
          f"a {reach:g}mm rule; re-escaping {len(balls)} ball(s) at a "
          f"{f:g}mm copper clearance")
    keys = {(p.global_x, p.global_y) for p in balls}
    n_short = 0
    if keys:
        n_seg0, n_via0 = len(pcb.segments), len(pcb.vias)
        try:
            for t in tracks:
                pcb.segments.append(_Seg(
                    start_x=t['start'][0], start_y=t['start'][1],
                    end_x=t['end'][0], end_y=t['end'][1],
                    width=t['width'], layer=t['layer'], net_id=t['net_id']))
            for v in vias:
                pcb.vias.append(_Via(
                    x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                    layers=v.get('layers') or ['F.Cu', 'B.Cu'], net_id=v['net_id']))
            kw = dict(args, clearance=f, _pad_filter=keys, _ignore_prefanned=True,
                      check_for_previous=True, _clearance_floor=f, bus=None,
                      escape_dir_hints=None)
            t2, v2, _vr2, _f2 = core(**kw)
        finally:
            del pcb.segments[n_seg0:]
            del pcb.vias[n_via0:]
        n0 = (len(tracks), len(vias))
        tr_all, vi_all = tracks + list(t2), vias + list(v2)
        viol2 = hole_violations(pcb, dr, tr_all, vi_all, fixed=n0)
        n_short = len(viol2)
        tracks, vias, _ = take_off(tr_all, vi_all, set(viol2), keep_from=n0)
    tw = float(args.get('track_width') or 0.0)
    left = [p for p in balls if not ball_has_copper(p, vias, tracks, tw)]
    for p in left:
        if p.net_name and p.net_name not in failed:
            failed.append(p.net_name)
    failed = unserved_nets(fp, pcb, failed, tracks, vias, tw)
    print(f"  hole_clearance (#1853): re-escaped {len(balls) - len(left)} of "
          f"{len(balls)} ball(s)"
          + (f"; {len(left)} left unescaped: "
             + ', '.join(sorted({p.net_name for p in left if p.net_name}))
             if left else '')
          + (f" (the re-escape's copper of {n_short} row(s) still inside the "
             f"rule was taken off)" if n_short else ''))
    return tracks, vias, vias_rm, failed


def hole_takeoff(footprint, pcb_data, tracks, vias, failed, dr=None):
    """`hole_violations` over a fan-out's (tracks, vias), with the escape of
    each pad a violating chain touches taken off and its net added to
    `failed`. Returns (tracks, vias, failed); the lists as given on a board
    with no hole_clearance rule."""
    from check_drc import board_design_rules, _hole_rule_reach
    from bga_fanout.class_clearance import _chains
    if dr is None:
        dr = board_design_rules(pcb_data)
    if _hole_rule_reach(dr) <= 0.0:
        return tracks, vias, failed
    tracks, vias, failed = list(tracks), list(vias), list(failed)
    viol = hole_violations(pcb_data, dr, tracks, vias)
    if not viol:
        return tracks, vias, failed
    of, touched = _chains(tracks, vias, footprint.pads)
    gone = {of[r] for r in viol}
    pads = []
    for c in gone:
        pads += [p for p in touched.get(c, ()) if p not in pads]
    tracks = [t for i, t in enumerate(tracks) if of[('t', i)] not in gone]
    vias = [v for i, v in enumerate(vias) if of[('v', i)] not in gone]
    names = sorted({p.net_name for p in pads if p.net_name})
    for n in names:
        if n not in failed:
            failed.append(n)
    print(f"  hole_clearance (#1853): {len(viol)} row(s) of this fan-out stood inside "
          f"a {_hole_rule_reach(dr):g}mm rule; took off the escape of {len(pads)} "
          f"pad(s), left to the route step: {', '.join(names)}")
    return tracks, vias, failed
