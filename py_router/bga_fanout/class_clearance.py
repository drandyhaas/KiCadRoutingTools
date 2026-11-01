"""Net classes for the BGA escape (#1529).

KiCad grades each pair of copper at max(clearance, class A, class B). The
escape engines space a run at ONE clearance, so they cannot price the pair a
classed ball's escape makes. They price the OBSTACLE's class themselves (the
channel engine's obstacle and pad maps, the under-pad engine's stamps and
registries); this module prices the ESCAPING ball's:

1. `class_violations` grades every pair this call's copper makes -- against
   the board and against itself -- at the pair's class, with check_drc's own
   primitives, and reports the pairs that fall short. Only pairs whose class
   value stands above the run clearance are graded: a pair at the run
   clearance is the engines' own business, priced as before.
2. `class_reescape` takes off the escape of every ball whose copper sets
   such a pair (the side whose class the pair takes; a pair against copper
   that was already on the board takes off the new side), then escapes those
   balls -- and any ball of a wider class the escape left bare -- again with
   their class as the run clearance (the under-pad engine), lowest class
   first, each against everything kept so far, grades the re-escape the same
   way, and takes off what still falls short. Those balls are reported
   unescaped.

Copper is one chain when it touches (an engine starts a via's track anywhere
on the via's copper), so a ball's escape comes off whole.

#1565: a .kicad_dru clearance rule scoped by net class or item type (#1491)
or a track-scoped class rule (#735) is priced the same way, through
`kicad_dru.PairRuleVerdicts`. Its pairs are graded at whatever clearance they
were laid at; balls alike to every rule re-escape together at the value a
rule sets between two of them, and a rule between such groups is held on the
obstacle side, folded into the class map the re-escape takes
(`rule_class_map`).

A board with no class above the run clearance and no such rule never reaches
here.
"""
from __future__ import annotations

import contextlib
import io
import math
from collections import defaultdict
from typing import Dict, List

#: check_drc's own grading tolerance (its default `clearance_margin`), so a
#: pair this check passes is one check_drc and the step's drc_grazes pass.
_MARGIN = 0.05
_CELL = 1.0          # mm, the spatial hash's cell


def _pair_config(pcb_data, base, net_clearances, routed_net_ids=None):
    """A GridRouteConfig that prices pairs as check_drc does: `base`, the
    classes, then the board's .kicad_dru layer rules -- and (#1565) the
    verdicts that add its class/type-scoped and track-scoped rules, for a
    routed set of `routed_net_ids` (the part's nets). Returns (cfg, rules)."""
    from routing_config import GridRouteConfig
    from kicad_dru import install_layer_clearances, install_pair_rule_channels
    layers = list(pcb_data.board_info.copper_layers or ['F.Cu', 'B.Cu'])
    cfg = GridRouteConfig(layers=layers, clearance=base)
    with contextlib.redirect_stdout(io.StringIO()):
        install_layer_clearances(cfg, None, None, pcb_data)
    cfg.set_net_clearances(dict(net_clearances), ())
    return cfg, install_pair_rule_channels(cfg, pcb_data, routed_net_ids)


def _part_nets(footprint):
    return sorted({p.net_id for p in footprint.pads if p.net_id})


def _has_dru(pcb_data):
    import os
    path = getattr(pcb_data, 'source_path', '') or ''
    return bool(path) and os.path.isfile(os.path.splitext(path)[0] + '.kicad_dru')


def rules_bind(pcb_data, footprint, base):
    """#1565: does a class/type-scoped or track-scoped .kicad_dru rule bind a
    net of `footprint`? Such a rule holds whatever clearance the escape was
    laid at -- the run clearance, or a rescue rung below it -- so its pairs
    are graded whether or not it stands above `base`. False, at the cost of
    one file test, on a board with no .kicad_dru."""
    if not _has_dru(pcb_data):
        return False
    nets = _part_nets(footprint)
    _cfg, rules = _pair_config(pcb_data, base, {}, nets)
    return rules.live and bool(rules.obstacle_floors(nets, list(pcb_data.nets)))


def rule_class_map(pcb_data, base, net_clearances, routed_net_ids, rules=None):
    """#1565: `net_clearances` with each net raised to the widest scoped or
    track rule binding its copper against the routed nets' -- the obstacle
    side of those rules, for an engine that holds each obstacle net at one
    value, as it holds a class. `net_clearances` itself on a board with no
    such rule (one file test without a .kicad_dru)."""
    if rules is None:
        if not _has_dru(pcb_data):
            return net_clearances
        _cfg, rules = _pair_config(pcb_data, base, net_clearances or {}, routed_net_ids)
    if not rules.live:
        return net_clearances
    out = dict(net_clearances or {})
    for n, v in rules.obstacle_floors(routed_net_ids, list(pcb_data.nets)).items():
        if v > out.get(n, 0.0):
            out[n] = v
    return out


def _ruled(rules):
    """Is a scoped or track rule live (#1565)?"""
    return rules is not None and rules.live


def _seg_obj(t):
    from kicad_parser import Segment
    return Segment(start_x=t['start'][0], start_y=t['start'][1],
                   end_x=t['end'][0], end_y=t['end'][1], width=t['width'],
                   layer=t['layer'], net_id=t['net_id'])


def _via_obj(v):
    from kicad_parser import Via
    return Via(x=v['x'], y=v['y'], size=v['size'], drill=v.get('drill') or 0.0,
               layers=list(v.get('layers') or ['F.Cu', 'B.Cu']), net_id=v['net_id'])


def _bbox(kind, o):
    if kind == 'seg':
        h = o.width / 2.0
        return (min(o.start_x, o.end_x) - h, min(o.start_y, o.end_y) - h,
                max(o.start_x, o.end_x) + h, max(o.start_y, o.end_y) + h)
    if kind == 'via':
        h = o.size / 2.0
        return (o.x - h, o.y - h, o.x + h, o.y + h)
    vs = [v for poly in (getattr(o, 'polygons', None) or ()) for v in poly]
    if vs:
        return (min(v[0] for v in vs), min(v[1] for v in vs),
                max(v[0] for v in vs), max(v[1] for v in vs))
    r = 0.5 * math.hypot(o.size_x, o.size_y)
    return (o.global_x - r, o.global_y - r, o.global_x + r, o.global_y + r)


def class_violations(pcb_data, tracks, vias, base, net_clearances,
                     partners=(), fixed=(0, 0), cfg=None, rules=None):
    """The pairs this call's copper (`tracks`, `vias`: the engines' dicts)
    makes below KiCad's class clearance.

    Each pair is priced at `cfg.pair_clearance` (max(base, class A, class B),
    a .kicad_dru layer rule replacing it on its layer) -- through `rules`
    (#1565), which adds a class/type-scoped rule for the two items' types and
    a track-scoped rule on a track pair -- and graded with check_drc's
    primitives at that value; only pairs whose value stands above `base` are
    graded, and every pair such a rule binds (a rescue rung may lay copper
    below `base`, and the rule holds there too). `partners` ({frozenset({p,
    n})}) are coupled diff pairs, spaced by the caller's --diff-pair-gap and
    not graded here. `fixed` = (n_tracks, n_vias): the leading items already
    kept -- graded as obstacles, never reported against each other.

    Returns [(a, b, need)] where a is ('t', i) or ('v', i) into the lists and
    b is the same for this call's copper or ('board', None) for copper that
    was there before."""
    from check_drc import (check_segment_overlap, check_via_segment_overlap,
                           check_via_via_overlap, check_pad_segment_overlap,
                           check_pad_via_overlap, _pad_has_no_copper)
    from geometry_utils import via_on_layer
    hot = {n for n, c in net_clearances.items() if c > base + 1e-9}
    if not (tracks or vias):
        return []
    if cfg is None:
        cfg, rules = _pair_config(pcb_data, base, net_clearances)
    elif rules is None:
        from kicad_dru import PairRuleVerdicts
        rules = PairRuleVerdicts(cfg, pcb_data)
    ruled = _ruled(rules)
    if not hot and not ruled:
        return []
    board_cu = list(pcb_data.board_info.copper_layers or ['F.Cu', 'B.Cu'])
    reach = max([net_clearances[n] for n in hot]
                + ([rules.max_value(base)] if ruled else [])) + 0.05

    new = ([('t', i, 'seg', _seg_obj(t)) for i, t in enumerate(tracks)]
           + [('v', i, 'via', _via_obj(v)) for i, v in enumerate(vias)])
    xs0 = min(_bbox(k, o)[0] for _, _, k, o in new) - 2 * reach - 1.0
    ys0 = min(_bbox(k, o)[1] for _, _, k, o in new) - 2 * reach - 1.0
    xs1 = max(_bbox(k, o)[2] for _, _, k, o in new) + 2 * reach + 1.0
    ys1 = max(_bbox(k, o)[3] for _, _, k, o in new) + 2 * reach + 1.0

    def inside(bb):
        return bb[2] >= xs0 and bb[0] <= xs1 and bb[3] >= ys0 and bb[1] <= ys1

    items = []                         # (ref, kind, obj, bbox, net)
    for tag, i, kind, o in new:
        items.append(((tag, i), kind, o, _bbox(kind, o), o.net_id))
    for s in pcb_data.segments:
        if not (s.layer or '').endswith('.Cu'):
            continue
        bb = _bbox('seg', s)
        if inside(bb):
            items.append((('board', None), 'seg', s, bb, s.net_id))
    for v in pcb_data.vias:
        bb = _bbox('via', v)
        if inside(bb):
            items.append((('board', None), 'via', v, bb, v.net_id))
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            if _pad_has_no_copper(p):
                continue
            bb = _bbox('pad', p)
            if inside(bb):
                items.append((('board', None), 'pad', p, bb, p.net_id))

    grid = defaultdict(list)
    for k, it in enumerate(items):
        bb = it[3]
        for gx in range(int(math.floor(bb[0] / _CELL)), int(math.floor(bb[2] / _CELL)) + 1):
            for gy in range(int(math.floor(bb[1] / _CELL)), int(math.floor(bb[3] / _CELL)) + 1):
                grid[(gx, gy)].append(k)

    def is_fixed(ref):
        tag, i = ref
        return (tag == 't' and i < fixed[0]) or (tag == 'v' and i < fixed[1])

    def graded(need, na, nb, layer=None, kind='layer', types=None):
        """A pair above `base`, and (#1565) any pair a scoped or track rule
        binds: a rescue rung may have laid it below `base`."""
        return need > base + 1e-9 or (ruled and rules.binds(
            na, nb, layer, kind=kind, types=types))

    out = []
    n_new = len(new)
    for a in range(n_new):
        ref_a, kind_a, oa, bb, na = items[a]
        cand = set()
        for gx in range(int(math.floor((bb[0] - reach) / _CELL)),
                        int(math.floor((bb[2] + reach) / _CELL)) + 1):
            for gy in range(int(math.floor((bb[1] - reach) / _CELL)),
                            int(math.floor((bb[3] + reach) / _CELL)) + 1):
                cand.update(grid.get((gx, gy), ()))
        for b in cand:
            if b == a or (b < n_new and b < a):
                continue                         # each new-new pair once
            ref_b, kind_b, ob, _bb, nb = items[b]
            if nb == na and na:
                continue                         # same net
            if not ruled and na not in hot and nb not in hot:
                continue
            if frozenset((na, nb)) in partners:
                continue
            if is_fixed(ref_a) and (ref_b[0] == 'board' or is_fixed(ref_b)):
                continue
            hit = False
            need = base
            if kind_a == 'seg' and kind_b == 'seg':
                if oa.layer != ob.layer:
                    continue
                need = rules(na, nb, oa.layer, base=base, kind='track')
                if graded(need, na, nb, oa.layer, 'track'):
                    hit = check_segment_overlap(oa, ob, need, _MARGIN)[0]
            elif {kind_a, kind_b} == {'seg', 'via'}:
                sv, vv = (oa, ob) if kind_a == 'seg' else (ob, oa)
                if not via_on_layer(vv, sv.layer):
                    continue
                _t = (('track' if kind_a == 'seg' else 'via',),
                      ('track' if kind_b == 'seg' else 'via',))
                need = rules(na, nb, sv.layer, base=base, types=_t)
                if graded(need, na, nb, sv.layer, types=_t):
                    hit = check_via_segment_overlap(vv, sv, need, _MARGIN)[0]
            elif kind_a == 'via' and kind_b == 'via':
                need = rules(na, nb, base=base, kind='stack')
                if graded(need, na, nb, kind='stack'):
                    hit = check_via_via_overlap(oa, ob, need, _MARGIN)[0]
            elif kind_b == 'pad':
                _pt = (('pad', getattr(ob, 'pad_type', None) or 'smd'),)
                if kind_a == 'seg':
                    need = rules(na, nb, oa.layer, base=base, types=(('track',), _pt))
                    if graded(need, na, nb, oa.layer, types=(('track',), _pt)):
                        hit = check_pad_segment_overlap(ob, oa, need, board_cu, _MARGIN)[0]
                else:
                    need = rules(na, nb, base=base, kind='stack', types=(('via',), _pt))
                    if graded(need, na, nb, kind='stack', types=(('via',), _pt)):
                        hit = check_pad_via_overlap(ob, oa, need, board_cu, _MARGIN)[0]
            if hit:
                out.append((ref_a, ref_b, need))
    return out


def _chains(tracks, vias, pads):
    """Connected copper per net: items of one net whose copper touches (an
    engine starts a via's track anywhere on the via's copper, not at its
    centre) are one chain. Returns ({item ref: chain id}, {chain id: [pads
    of `pads` the chain's copper touches]})."""
    from check_drc import segment_pad_gap
    from geometry_utils import segment_to_segment_distance, point_to_segment_distance
    items = ([(('t', i), t['net_id'], 'seg', t) for i, t in enumerate(tracks)]
             + [(('v', i), v['net_id'], 'via', v) for i, v in enumerate(vias)])
    parent = {it[0]: it[0] for it in items}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    def bbox(kind, o):
        if kind == 'seg':
            h = o['width'] / 2.0
            (x1, y1), (x2, y2) = o['start'], o['end']
            return (min(x1, x2) - h, min(y1, y2) - h, max(x1, x2) + h, max(y1, y2) + h)
        h = o['size'] / 2.0
        return (o['x'] - h, o['y'] - h, o['x'] + h, o['y'] + h)

    def touch(ka, a, kb, b):
        tol = 1e-4
        if ka == 'seg' and kb == 'seg':
            if a['layer'] != b['layer']:
                return False
            d = segment_to_segment_distance(*a['start'], *a['end'], *b['start'], *b['end'])
            return d <= (a['width'] + b['width']) / 2.0 + tol
        if ka == 'via' and kb == 'via':
            return math.hypot(a['x'] - b['x'], a['y'] - b['y']) <= (a['size'] + b['size']) / 2.0 + tol
        sg, vv = (a, b) if ka == 'seg' else (b, a)
        d = point_to_segment_distance(vv['x'], vv['y'], *sg['start'], *sg['end'])
        return d <= vv['size'] / 2.0 + sg['width'] / 2.0 + tol

    cells = defaultdict(list)
    for k, (ref, net, kind, o) in enumerate(items):
        bb = bbox(kind, o)
        for gx in range(int(math.floor(bb[0] / _CELL)), int(math.floor(bb[2] / _CELL)) + 1):
            for gy in range(int(math.floor(bb[1] / _CELL)), int(math.floor(bb[3] / _CELL)) + 1):
                for k2 in cells[(gx, gy)]:
                    ref2, net2, kind2, o2 = items[k2]
                    if net2 == net and find(ref2) != find(ref) and touch(kind, o, kind2, o2):
                        union(ref, ref2)
                cells[(gx, gy)].append(k)
    of = {ref: find(ref) for ref, *_ in items}
    balls = defaultdict(list)
    by_net = defaultdict(list)
    for p in pads:
        if p.net_id:
            by_net[p.net_id].append(p)
    for ref, net, kind, o in items:
        for p in by_net.get(net, ()):
            if kind == 'seg':
                hit = min(segment_pad_gap(*o['start'], *o['start'], p),
                          segment_pad_gap(*o['end'], *o['end'], p)) <= o['width'] / 2.0
            else:
                hit = segment_pad_gap(o['x'], o['y'], o['x'], o['y'], p) <= o['size'] / 2.0
            if hit and p not in balls[of[ref]]:
                balls[of[ref]].append(p)
    return of, balls


def _drop_refs(violations, floor):
    """The copper each short pair takes off: the side(s) whose class the
    pair's value is (that class priced the pair, so that ball re-escapes at
    it), or the new side of a pair against copper already on the board."""
    drop = set()
    for a, b, need in violations:
        if b[0] == 'board':
            drop.add(a)
            continue
        sides = [r for r, f in ((a, floor(a)), (b, floor(b))) if f >= need - 1e-9]
        drop.update(sides or (a, b))
    return drop


def class_reescape(core, args, result):
    """Grade `result` (the core's (tracks, vias, vias_to_remove, failed)) at
    the net classes and re-escape the balls that fall short at their class.

    `core` is _generate_bga_fanout_core and `args` the keyword arguments it
    produced `result` with (the part's own frame). See the module docstring."""
    tracks, vias, vias_rm, failed = result
    tracks, vias, failed = list(tracks), list(vias), list(failed)
    fp, pcb = args['footprint'], args['pcb_data']
    base = float(args['clearance'])
    nc = {int(n): float(c) for n, c in (args.get('net_clearances') or {}).items()}
    cfg, rules = _pair_config(pcb, base, nc, _part_nets(fp))
    # #1565: the scoped and track rules. Balls alike to every rule
    # (rules.group_key) re-escape together, at the value a rule sets between
    # two of their nets (`_rule_floor`); a rule between groups is held on the
    # obstacle side, as a class is (`_group_classes`).
    ruled = _ruled(rules)
    _intra: Dict[tuple, float] = {}
    if ruled:
        _by_key: Dict[tuple, List[int]] = defaultdict(list)
        for n in _part_nets(fp):
            _by_key[rules.group_key(n)].append(n)
        for k, ns in _by_key.items():
            _intra[k] = rules.rule_value(ns[0], ns[1]) if len(ns) > 1 else 0.0
        print(f"  {rules.describe()}")

    def _rule_floor(net_id):
        return _intra.get(rules.group_key(net_id), 0.0) if ruled else 0.0

    def _group_classes(group_nets):
        """The class map a group re-escapes with: each net at max(its class,
        the widest rule binding it against the group's nets)."""
        if not ruled:
            return args.get('net_clearances')
        return rule_class_map(pcb, base, nc, group_nets, rules=rules)
    what = ("Net classes and .kicad_dru rules (#1529, #1565)" if ruled
            else "Net classes (#1529)")
    partners = set()
    if args.get('diff_pair_patterns'):
        from bga_fanout.diff_pair import find_differential_pairs
        for dp in find_differential_pairs(fp, args['diff_pair_patterns']).values():
            if dp.p_pad and dp.n_pad:
                partners.add(frozenset((dp.p_pad.net_id, dp.n_pad.net_id)))

    def floor_of(net_id):
        return max(base, nc.get(net_id, base), _rule_floor(net_id))

    def chain_split(tr, vi, viol, keep_from=None):
        """Take off the chains `viol` names; return the kept lists and the
        balls taken off. With `keep_from` (n_tracks, n_vias) the leading
        items are kept and every short pair takes off its newer side."""
        if not viol:
            return tr, vi, []
        net_of = {('t', i): t['net_id'] for i, t in enumerate(tr)}
        net_of.update({('v', i): v['net_id'] for i, v in enumerate(vi)})
        if keep_from is None:
            refs = _drop_refs(viol, lambda r: floor_of(net_of[r]))
        else:
            refs = {r for a, b, _n in viol for r in (a, b)
                    if (r[0] == 't' and r[1] >= keep_from[0])
                    or (r[0] == 'v' and r[1] >= keep_from[1])}
        of, touched = _chains(tr, vi, fp.pads)
        gone = {of[r] for r in refs}
        balls = []
        for c in gone:
            balls += [p for p in touched.get(c, ()) if p not in balls]
        k0 = keep_from or (0, 0)       # kept copper stays, even on a gone chain
        tr = [t for i, t in enumerate(tr) if i < k0[0] or of[('t', i)] not in gone]
        vi = [v for i, v in enumerate(vi) if i < k0[1] or of[('v', i)] not in gone]
        return tr, vi, balls

    from bga_fanout import ball_has_copper, unserved_nets
    tw = args['track_width']
    viol = class_violations(pcb, tracks, vias, base, nc, partners, cfg=cfg,
                            rules=rules)
    tracks, vias, balls = chain_split(tracks, vias, viol)
    # ...and a ball of a wider class the escape left bare: the scalar engine
    # could not price its own class, so it is escaped at it too.
    _failed = set(failed)
    _taken = {id(p) for p in balls}
    bare = [p for p in fp.pads if p.net_id and p.net_name in _failed
            and id(p) not in _taken and floor_of(p.net_id) > base + 1e-9
            and not ball_has_copper(p, vias, tracks, tw)]
    if not balls and not bare:
        return tracks, vias, vias_rm, failed
    print(f"  {what}: {len(viol)} pair(s) of this escape fell short "
          f"of their class clearance at {base:g}mm; re-escaping {len(balls)} "
          f"ball(s) at their class"
          + (f", and {len(bare)} bare ball(s) of a wider class" if bare else ''))
    balls = balls + bare

    def descent_floor(net_id, f):
        """The clearance a ball's rescue rungs stop at: its class above the
        run clearance, else (#1565) the widest rule binding it."""
        if f > base + 1e-9:
            return f
        return max(args.get('_clearance_floor', 0.0), _rule_floor(net_id))

    groups: Dict[tuple, List] = defaultdict(list)
    for p in balls:
        f = round(floor_of(p.net_id), 6)
        groups[(f, round(descent_floor(p.net_id, f), 6),
                rules.group_key(p.net_id) if ruled else None)].append(p)
    method = 'dogbone' if args.get('escape_method') == 'dogbone' else 'underpad'
    from kicad_parser import Segment as _Seg, Via as _Via
    n_short = 0
    for gk in sorted(groups, key=lambda k: (k[0], k[1], repr(k[2]))):
        f, d, _k = gk
        keys = {(p.global_x, p.global_y) for p in groups[gk]}
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
            # a classed ball's rescue rungs stop at its class, a ruled one's
            # at its rule; a ball at the run clearance keeps the fab ladder
            kw = dict(args, clearance=f, escape_method=method, _pad_filter=keys,
                      _ignore_prefanned=True, check_for_previous=True,
                      _clearance_floor=d, bus=None, escape_dir_hints=None,
                      net_clearances=_group_classes(
                          sorted({p.net_id for p in groups[gk]})))
            t2, v2, _vr2, _f2 = core(**kw)
        finally:
            del pcb.segments[n_seg0:]
            del pcb.vias[n_via0:]
        n0 = (len(tracks), len(vias))
        tr_all, vi_all = tracks + list(t2), vias + list(v2)
        viol2 = class_violations(pcb, tr_all, vi_all, base, nc, partners,
                                 fixed=n0, cfg=cfg, rules=rules)
        n_short += len(viol2)
        tracks, vias, _ = chain_split(tr_all, vi_all, viol2, keep_from=n0)
    left = [p for p in balls if not ball_has_copper(p, vias, tracks, tw)]
    n_ok = len(balls) - len(left)
    for p in left:
        if p.net_name and p.net_name not in failed:
            failed.append(p.net_name)
    failed = unserved_nets(fp, pcb, failed, tracks, vias, tw)
    print(f"  {what}: re-escaped {n_ok} of {len(balls)} ball(s) at "
          f"their class" + (f"; {len(left)} left unescaped: "
                            + ', '.join(sorted({p.net_name for p in left}))
                            if left else '')
          + (f" (the re-escape's copper of {n_short} pair(s) still short of a "
             f"class was taken off)" if n_short else ''))
    return tracks, vias, vias_rm, failed
