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
against itself, once the engines are done (`check_drc.hole_rule_step_rows`):

1. `hole_reescape` (the BGA escape) takes off the escape of every ball whose
   copper stands in a row (a chain of touching copper comes off whole) and
   escapes those balls again at the copper clearance that holds the rule at
   a via's ring (rule - (via_size - via_drill) / 2): the preferred rung.
   Re-escape copper still in a row comes off, and a ball the rung leaves
   bare gets its first escape back -- the LAST rung, laid within the fab
   floors, below the rule. Where that escape and the rung's copper of
   another ball clash, the other ball gives way to its own first escape
   when that clashes with no rung copper left; else the bare ball stays
   bare. Completion first, disclosed.
2. `hole_disclose` (the QFN fan-out, whose escapes are one stub or one drop
   each and whose via sites the gate already holds) keeps the escapes.

Either way what the call's copper still breaks is named per rule
(`check_drc.hole_rule_step_grade`, the key `hole_rules` that
`design_rules.disclose_step_rules` gives a route step) and returned for
JSON_SUMMARY.design_rules. Free on a board with no hole_clearance rule.
"""
from __future__ import annotations

import math
from typing import List, Tuple


def hole_violations(pcb_data, dr, tracks, vias, fixed=(0, 0)) -> List[Tuple[str, int]]:
    """The refs (('t', i) / ('v', i) into `tracks` / `vias`, the engines'
    dicts) of this call's copper in a hole_clearance row: the copper side of
    a pair, else the via whose hole it is. `fixed` = (n_tracks, n_vias):
    leading items already kept, never named; a row between one of them and
    newer copper names the newer side."""
    from check_drc import hole_rule_step_rows

    def _kept(ref):
        return ref is not None and ((ref[0] == 't' and ref[1] < fixed[0])
                                    or (ref[0] == 'v' and ref[1] < fixed[1]))
    out = []
    for _row, cref, href in hole_rule_step_rows(pcb_data, dr, tracks, vias):
        for ref in (cref, href):
            if ref is not None and not _kept(ref):
                out.append(ref)
                break
    return out


def _disclose(pcb_data, dr, tracks, vias, what):
    from check_drc import hole_rule_step_grade, hole_rule_grade_lines
    grade = hole_rule_step_grade(pcb_data, dr, tracks, vias)
    for line in hole_rule_grade_lines(grade):
        print(f"  {what}: {line}")
    return grade


def _clashes(chain_items, kept, base, h2h):
    """Does copper of `chain_items` stand within `base` of the copper of
    another net in `kept` (or a hole within `h2h` of a hole)? Both are
    (kind, dict) lists of engine tracks ('t') and vias ('v')."""
    from geometry_utils import point_to_segment_distance, segment_to_segment_distance
    eps = 1e-6
    for ka, a in chain_items:
        for kb, b in kept:
            if a['net_id'] == b['net_id']:
                continue
            if ka == 't' and kb == 't':
                if a['layer'] != b['layer']:
                    continue
                d = segment_to_segment_distance(*a['start'], *a['end'], *b['start'], *b['end'])
                if d < (a['width'] + b['width']) / 2.0 + base - eps:
                    return True
            elif ka == 'v' and kb == 'v':
                d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                if (d < (a['size'] + b['size']) / 2.0 + base - eps
                        or d < ((a.get('drill') or 0) + (b.get('drill') or 0)) / 2.0 + h2h - eps):
                    return True
            else:
                s, v = (a, b) if ka == 't' else (b, a)
                d = point_to_segment_distance(v['x'], v['y'], *s['start'], *s['end'])
                if d < (v['size'] + s['width']) / 2.0 + base - eps:
                    return True
    return False


def hole_reescape(core, args, result, dr):
    """Grade `result` (tracks, vias, vias_to_remove, failed) against the
    board's hole_clearance rules, re-escape the balls that fall short and
    give a ball the re-escape leaves bare its first escape back. `core` is
    the engine entry and `args` its keyword arguments for this part; `dr`
    the board's `design_rules.DesignRules`. Returns (tracks, vias,
    vias_to_remove, failed, grade): `grade` the rules the copper still
    breaks (`check_drc.hole_rule_step_grade`). See the module docstring."""
    from check_drc import _hole_rule_reach
    from bga_fanout.class_clearance import _chains
    from bga_fanout import ball_has_copper, unserved_nets
    from kicad_parser import Segment as _Seg, Via as _Via
    tracks, vias, vias_rm, failed = result
    tracks, vias, failed = list(tracks), list(vias), list(failed)
    reach = _hole_rule_reach(dr)
    pcb, fp = args['pcb_data'], args['footprint']
    viol = hole_violations(pcb, dr, tracks, vias)
    if not viol:
        return tracks, vias, vias_rm, failed, []
    tw = float(args.get('track_width') or 0.0)

    def take_off(tr, vi, refs, keep_from=(0, 0)):
        """Kept lists, and {chain: ([(kind, item)], [balls])} taken off."""
        of, touched = _chains(tr, vi, fp.pads)
        gone = {of[r] for r in refs if of.get(r) is not None}
        off = {c: ([], list(touched.get(c, ()))) for c in gone}
        kt, kv = [], []
        for i, t in enumerate(tr):
            if i >= keep_from[0] and of[('t', i)] in gone:
                off[of[('t', i)]][0].append(('t', t))
            else:
                kt.append(t)
        for i, v in enumerate(vi):
            if i >= keep_from[1] and of[('v', i)] in gone:
                off[of[('v', i)]][0].append(('v', v))
            else:
                kv.append(v)
        return kt, kv, off

    n_rows = len(viol)
    tracks, vias, first = take_off(tracks, vias, set(viol))
    balls = []
    for _items, bs in first.values():
        balls += [p for p in bs if p not in balls]
    base = float(args['clearance'])
    ring = (float(args.get('via_size') or 0.0) - float(args.get('via_drill') or 0.0)) / 2.0
    f = round(max(base, reach - max(ring, 0.0)), 6)
    print(f"  hole_clearance (#1853): {n_rows} row(s) of this escape stood inside "
          f"a {reach:g}mm rule; re-escaping {len(balls)} ball(s) at a "
          f"{f:g}mm copper clearance")
    keys = {(p.global_x, p.global_y) for p in balls}
    n0 = (len(tracks), len(vias))
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
        tr_all, vi_all = tracks + list(t2), vias + list(v2)
        viol2 = hole_violations(pcb, dr, tr_all, vi_all, fixed=n0)
        tracks, vias, _ = take_off(tr_all, vi_all, set(viol2), keep_from=n0)
    # the last rung: a ball the preferred rung left bare gets its first
    # escape back (completion first). The first escapes were laid together,
    # so they never clash with each other, only with the rung's copper.
    h2h = float(args.get('hole_to_hole_clearance') or 0.0) or float(
        (getattr(dr, 'board_min', None) or {}).get('min_hole_to_hole') or 0.0)
    rung_t, rung_v = tracks[n0[0]:], vias[n0[1]:]
    tracks, vias = tracks[:n0[0]], vias[:n0[1]]
    r_of, r_touched = _chains(rung_t, rung_v, fp.pads)
    rung = {}
    for i, t in enumerate(rung_t):
        rung.setdefault(r_of[('t', i)], []).append(('t', t))
    for i, v in enumerate(rung_v):
        rung.setdefault(r_of[('v', i)], []).append(('v', v))
    rung_balls = {c: list(r_touched.get(c, ())) for c in rung}
    first_of = {}                     # id(ball) -> first chains that touch it
    for c, (_items, bs) in first.items():
        for p in bs:
            first_of.setdefault(id(p), []).append(c)
    served = {id(p) for bs in rung_balls.values() for p in bs}
    back = {c for c, (_items, bs) in first.items()
            if bs and not all(id(p) in served for p in bs)}
    # one step only: a rung chain that clashes with a first escape put back
    # gives way, and its ball takes its own first escape back -- where that
    # clashes with no rung copper left; else the bare ball stays bare
    restored = [it for c in back for it in first[c][0]]
    clash = {rc for rc, items in rung.items() if _clashes(items, restored, base, h2h)}
    keep_rung = [it for rc, items in rung.items() if rc not in clash for it in items]
    dropped = set()
    for rc in sorted(clash):
        mine = set()
        for p in rung_balls[rc]:
            mine.update(first_of.get(id(p), ()))
        if all(not _clashes(first[c][0], keep_rung, base, h2h) for c in mine):
            dropped.add(rc)
            back.update(mine)
    # a first escape put back that still clashes with rung copper kept stays off
    keep_rung = [it for rc, items in rung.items() if rc not in dropped for it in items]
    back = {c for c in back if not _clashes(first[c][0], keep_rung, base, h2h)}
    for rc, items in rung.items():
        if rc not in dropped:
            tracks += [it for k, it in items if k == 't']
            vias += [it for k, it in items if k == 'v']
    for c in back:
        tracks += [it for k, it in first[c][0] if k == 't']
        vias += [it for k, it in first[c][0] if k == 'v']
    n_rung = sum(1 for p in balls
                 if any(p in rung_balls[rc] for rc in rung if rc not in dropped))
    n_back = sum(1 for p in balls if any(c in back for c in first_of.get(id(p), ())))
    left = [p for p in balls if not ball_has_copper(p, vias, tracks, tw)]
    for p in left:
        if p.net_name and p.net_name not in failed:
            failed.append(p.net_name)
    failed = unserved_nets(fp, pcb, failed, tracks, vias, tw)
    print(f"  hole_clearance (#1853): {n_rung} of {len(balls)} ball(s) re-escaped "
          f"inside the rule; {n_back} kept their first escape, below it"
          + (f"; {len(left)} left unescaped: "
             + ', '.join(sorted({p.net_name for p in left if p.net_name}))
             if left else ''))
    grade = _disclose(pcb, dr, tracks, vias, 'BGA fanout')
    return tracks, vias, vias_rm, failed, grade


def hole_disclose(pcb_data, tracks, vias, dr=None, what='QFN fanout'):
    """The hole_clearance rules a fan-out's (tracks, vias) break, named per
    rule and returned (`check_drc.hole_rule_step_grade`); the copper is
    kept. [] on a board with no such rule."""
    from check_drc import board_design_rules, _hole_rule_reach
    if dr is None:
        dr = board_design_rules(pcb_data)
    if _hole_rule_reach(dr) <= 0.0:
        return []
    return _disclose(pcb_data, dr, tracks, vias, what)
