"""Polarity crossover for a differential pair whose two ends have opposite hands.

When P sits on one side of the pair at its source and on the other side at its
target, the coupled route cannot reach both ends without its legs crossing. The
polarity ladder in diff_pair_routing resolves that with a pad swap (only where
an endpoint can compensate, #279), an opposite-side connector flip (bare-pad
ends only) or, for a false positive, the natural geometry. A designer instead
crosses the legs AT A LAYER CHANGE: the leg that changes sides dives first, the
other passes over its new-layer copper and dives just beyond it, and each leg
keeps one via (#1055).

This module is that crossover as the ladder's last rung. It reads the centerline
the rejected no-swap route found and offers crossover SITES along it -- the
path's own dives first, then points on its straight runs, nearest the two ends
first (where a human puts it, at a connector's escape) -- and for each site lays
the crossover's copper as given geometry, then routes the two same-hand spans
with the production pair router: pad end to the crossover's entry, and its exit
to the other pad end. The second span is routed with the first one's copper
stamped in, so neither can run through the other or through the crossover.

The geometry is the #622 whole-route prototype's (awx/pairs.py `crossover`,
bus_take6): both barrels on the first diver's side, staggered along the heading
by the least whole grid steps that keep via-to-via, via-to-jog and hole-to-hole
clearance, every leg turning at most 45 degrees.
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from kicad_parser import Segment, Via

Pt = Tuple[float, float]

# Router calls (each a two-span attempt) the crossover may spend on one pair.
MAX_SITE_ATTEMPTS = 8


# --------------------------------------------------------------- geometry


def _seg_seg_dist(a: Pt, b: Pt, c: Pt, d: Pt) -> float:
    def pt_seg(p, s, e):
        dx, dy = e[0] - s[0], e[1] - s[1]
        L2 = dx * dx + dy * dy
        if L2 < 1e-18:
            return math.hypot(p[0] - s[0], p[1] - s[1])
        t = max(0.0, min(1.0, ((p[0] - s[0]) * dx + (p[1] - s[1]) * dy) / L2))
        return math.hypot(p[0] - s[0] - t * dx, p[1] - s[1] - t * dy)

    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0) and min(abs(o1), abs(o2), abs(o3), abs(o4)) > 1e-15:
        return 0.0
    return min(pt_seg(a, c, d), pt_seg(b, c, d), pt_seg(c, a, b), pt_seg(d, a, b))


def _intra_clear(segs: Sequence[Segment], vias: Sequence[Via], p_id: int, n_id: int,
                 track: float, via_size: float, clearance: float, via_pitch: float) -> bool:
    """True when the pair's own pieces keep their intra-pair clearance: P and N
    segments on one layer, each barrel (a through via: every layer) against the
    other leg's segments, and the two legs' barrels against each other."""
    P = [s for s in segs if s.net_id == p_id]
    N = [s for s in segs if s.net_id == n_id]
    for s in P:
        for t in N:
            if s.layer == t.layer and _seg_seg_dist((s.start_x, s.start_y), (s.end_x, s.end_y),
                                                    (t.start_x, t.start_y), (t.end_x, t.end_y)) \
                    < track + clearance - 1e-6:
                return False
    for v in vias:
        other = N if v.net_id == p_id else P
        for t in other:
            if _seg_seg_dist((v.x, v.y), (v.x, v.y), (t.start_x, t.start_y), (t.end_x, t.end_y)) \
                    < via_size / 2 + track / 2 + clearance - 1e-6:
                return False
    pv = [v for v in vias if v.net_id == p_id]
    nv = [v for v in vias if v.net_id == n_id]
    for a in pv:
        for b in nv:
            if math.hypot(a.x - b.x, a.y - b.y) < via_pitch - 1e-6:
                return False
    return True


def crossover_geometry(V: Pt, u: Pt, s_in: int, half: float, via_half: float, via_size: float,
                       drill: float, track: float, clearance: float, h2h: float, grid: float,
                       first: str = 'P') -> dict:
    """A pair's crossover at V on a straight stretch along u: the two legs swap
    sides. `s_in` is the side of u P lies on before the crossover (+1 left,
    -1 right); after it, the other. The FIRST diver runs its old line on L1,
    steps out at 45 degrees to its barrel (away from the other leg), dives, and
    jogs at 45 degrees on L2 onto its new line (the other's old one); the SECOND
    runs its old line on L1, jogs at 45 degrees over the first's new-layer leg
    to its barrel beside its new line, dives and steps back onto it. Both
    barrels stand on one side, staggered along u. Port of awx/pairs.crossover.

    -> dict(entry={'P','N'}, exit={'P','N'}, poses=(in, out) on the centreline,
    legs={'P': [(points, 'L1'|'L2')], 'N': ...}, vias=[(x, y, 'P'|'N')],
    span=(along_in, along_out) from V)."""
    ul = math.hypot(*u)
    ux, uy = u[0] / ul, u[1] / ul
    lx, ly = -uy, ux

    def to_xy(x, y):
        return (V[0] + x * ux + y * lx, V[1] + x * uy + y * ly)

    yP0 = s_in * half                                  # P's line before; after, -yP0
    yF0 = yP0 if first == 'P' else -yP0                # the first diver's old line
    sg = 1.0 if yF0 > 0 else -1.0
    vy = sg * via_half                                 # both barrels' offset
    jog = half + via_half                              # a jog's reach (and its length along u at 45)
    need = max(via_size + clearance,
               (via_size / 2 + track / 2 + clearance) / math.sin(math.pi / 4),
               drill + h2h)
    dx = math.ceil(need / grid - 1e-9) * grid          # the stagger
    xa = -math.floor(dx / 2 / grid) * grid
    xb = xa + dx
    st = abs(vy - yF0)                                 # a line-to-barrel step (45 degrees)
    x_in, x_out = min(xa - st, xb - jog), max(xa + jog, xb + st)
    step = grid / max(abs(ux), abs(uy))                # a grid step along u
    pose_in = math.floor(x_in / step + 1e-9) * step
    pose_out = math.ceil(x_out / step - 1e-9) * step
    F = [([(pose_in, yF0), (xa - st, yF0), (xa, vy)], 'L1'),
         ([(xa, vy), (xa + jog, -yF0), (pose_out, -yF0)], 'L2')]
    S = [([(pose_in, -yF0), (xb - jog, -yF0), (xb, vy)], 'L1'),
         ([(xb, vy), (xb + st, yF0), (pose_out, yF0)], 'L2')]
    legs = {'P': F, 'N': S} if first == 'P' else {'P': S, 'N': F}
    legs = {k: [([to_xy(*q) for q in pts], L) for pts, L in v] for k, v in legs.items()}
    va, vb = to_xy(xa, vy), to_xy(xb, vy)
    second = 'N' if first == 'P' else 'P'
    return dict(entry={'P': to_xy(pose_in, yP0), 'N': to_xy(pose_in, -yP0)},
                exit={'P': to_xy(pose_out, -yP0), 'N': to_xy(pose_out, yP0)},
                poses=(to_xy(pose_in, 0.0), to_xy(pose_out, 0.0)),
                legs=legs, vias=[(va[0], va[1], first), (vb[0], vb[1], second)],
                span=(pose_in, pose_out))


def crossover_copper(geo: dict, L1: str, L2: str, p_id: int, n_id: int, track: float,
                     via_size: float, drill: float, via_layers: List[str], via_type: str = 'through'):
    """(segments, vias) for a crossover's geometry on layers L1 -> L2."""
    ids = {'P': p_id, 'N': n_id}
    lname = {'L1': L1, 'L2': L2}
    segs = []
    for k, runs in geo['legs'].items():
        for pts, L in runs:
            for a, b in zip(pts, pts[1:]):
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-9:
                    segs.append(Segment(a[0], a[1], b[0], b[1], track, lname[L], ids[k]))
    vias = [Via(x=x, y=y, size=via_size, drill=drill, layers=list(via_layers), net_id=ids[k],
                via_type=via_type) for x, y, k in geo['vias']]
    return segs, vias


def _board_clear(segs: Sequence[Segment], vias: Sequence[Via], pcb_data, config) -> str:
    """'' when the crossover's copper clears the board's other nets, else why
    not: check_drc.new_copper_refusal (moved there for the oracle's rescue
    vias, #2010). The pair map and the connector graze gate are not enough
    for given copper: the first arm shipped a barrel on the partner's pad
    (ethercat_ax58100 P0_TXOP, 0.134 mm into C36.2) and a leg 0.036 mm into a
    routed pair's track (berkeley_obsidian AD1_P / MGT_RX2_P)."""
    from check_drc import new_copper_refusal
    return new_copper_refusal(segs, vias, pcb_data, config)


# --------------------------------------------------------------- sites


def _sites(centerline: Sequence[Tuple[float, float, int]], span_in: float, span_out: float,
           margin_start: float, margin_end: float, grid: float):
    """Crossover sites along a centreline [(x, y, layer_idx)] in route order:
    (kind, V, u, L1 idx, L2 idx or None, arc position). Dives first, then
    straight-run sites nearest either end, then the rest of each run."""
    pts = [(float(x), float(y), int(L)) for x, y, L in centerline]
    if len(pts) < 2:
        return []
    arc = [0.0]
    for a, b in zip(pts, pts[1:]):
        arc.append(arc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = arc[-1]
    dives, runs = [], []
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
        if a[2] != b[2]:
            continue
        if seg_len < 1e-9:
            continue
        u = ((b[0] - a[0]) / seg_len, (b[1] - a[1]) / seg_len)
        step = grid / max(abs(u[0]), abs(u[1]))
        lo = max(arc[i] + span_in, margin_start + span_in)
        hi = min(arc[i + 1] - span_out, total - margin_end - span_out)
        if hi < lo - 1e-9:
            continue
        # grid-aligned positions along this run (the run starts on a grid point)
        k_lo = math.ceil((lo - arc[i]) / step - 1e-9)
        k_hi = math.floor((hi - arc[i]) / step + 1e-9)
        if k_hi < k_lo:
            continue
        ks = sorted({k_lo, k_hi, (k_lo + k_hi) // 2})
        for k in ks:
            t = k * step
            V = (a[0] + u[0] * t, a[1] + u[1] * t)
            runs.append((min(arc[i] + t, total - arc[i] - t), (V, u, a[2], arc[i] + t)))
    # dives: a layer change between consecutive points at (nearly) one position
    for i in range(1, len(pts) - 1):
        if pts[i][2] != pts[i + 1][2] and math.hypot(pts[i + 1][0] - pts[i][0],
                                                       pts[i + 1][1] - pts[i][1]) < 1e-6:
            a = pts[i - 1]
            seg_len = math.hypot(pts[i][0] - a[0], pts[i][1] - a[1])
            if seg_len < 1e-9:
                continue
            u = ((pts[i][0] - a[0]) / seg_len, (pts[i][1] - a[1]) / seg_len)
            if arc[i] - span_in < margin_start or arc[i] + span_out > total - margin_end:
                continue
            dives.append(('dive', (pts[i][0], pts[i][1]), u, pts[i][2], pts[i + 1][2], arc[i]))
    runs.sort(key=lambda r: r[0])
    return dives + [('run', V, u, L1, None, s) for _d, (V, u, L1, s) in runs]


# --------------------------------------------------------------- the rung


def _endpoint(p: Pt, n: Pt, layer_idx: int, coord) -> tuple:
    gp, gn = coord.to_grid(*p), coord.to_grid(*n)
    return (gp[0], gp[1], gn[0], gn[1], layer_idx, p[0], p[1], n[0], n[1])


def try_crossover(pcb_data, diff_pair, config, obstacles, base_obstacles, unrouted_stubs,
                  natural: dict, route_fn, verbose: bool = True) -> Optional[dict]:
    """The polarity ladder's crossover rung. `natural` is what the rejected
    no-swap route recorded (diff_pair_routing: 'xo_natural'): its simplified
    centreline in mm with layer indices, in ROUTE order, the endpoint tuples it
    ran between in that order, the physical-end directions forced at those ends
    (or None), and P's side at its start. Returns a success result for the
    whole pair (both spans and the crossover), or None."""
    from routing_utils import build_layer_map
    from routing_config import GridCoord
    from obstacle_map import add_segments_list_as_obstacles, add_vias_list_as_obstacles, check_line_clearance
    from diff_pair_routing import (_pair_via_offset, _min_via_center_distance, centreline_extra_clearance,
                                   _pn_tracks_cross_full, _neck_pair_partner_grazes)
    from via_span import via_layer_pair as _via_layer_pair, via_kind as _via_kind

    centerline = natural.get('centerline') or []
    if len(centerline) < 2:
        return None
    p_id, n_id = diff_pair.p_net_id, diff_pair.n_net_id
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    names = list(config.layers)
    half = (config.track_width + config.diff_pair_gap) / 2
    via_half = max(_pair_via_offset(config, half, p_id, n_id),
                   (config.via_size + config.clearance) / 2.0,
                   (config.via_size / 2 + config.clearance + config.track_width / 2 - half) / 0.7071 + 0.005)
    h2h = getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0
    clr = config.pair_clearance(p_id, n_id, names[0]) if hasattr(config, 'pair_clearance') else config.clearance
    via_pitch = _min_via_center_distance(config, p_id, n_id)
    via_layers = list(_via_layer_pair(config))
    shape = crossover_geometry((0.0, 0.0), (1.0, 0.0), 1, half, via_half, config.via_size, config.via_drill,
                               config.track_width, clr, h2h, config.grid_step)
    span_in, span_out = -shape['span'][0], shape['span'][1]
    # How far along the path from either end a crossover's pose may stand: the
    # span between them needs the router's own setback at the crossover end
    # and a launch run of the same length from the original end's pose. The
    # router's setback is its configured one, else 4x the half-pitch
    # (diff_pair_routing._try_route_direction).
    cfg_sb = config.diff_pair_centerline_setback
    router_setback = cfg_sb if cfg_sb is not None else half * 4
    margin = 2 * router_setback + 2 * config.grid_step
    s_in = int(natural['p_sign'])
    src_ep, tgt_ep = natural['src_ep'], natural['tgt_ep']
    src_dir, tgt_dir = natural.get('src_forced'), natural.get('tgt_forced')
    xclr = centreline_extra_clearance(config)

    attempts = 0
    tried = 0
    for kind, V, u, L1i, L2i_fixed, at in _sites(centerline, span_in, span_out, margin, margin,
                                                 config.grid_step):
        if attempts >= MAX_SITE_ATTEMPTS:
            break
        L2s = [L2i_fixed] if L2i_fixed is not None else [i for i in range(len(names)) if i != L1i]
        for L2i in L2s:
            if attempts >= MAX_SITE_ATTEMPTS:
                break
            for first in ('P', 'N'):
                geo = crossover_geometry(V, u, s_in, half, via_half, config.via_size, config.via_drill,
                                         config.track_width, clr, h2h, config.grid_step, first=first)
                L1, L2 = names[L1i], names[L2i]
                segs, vias = crossover_copper(geo, L1, L2, p_id, n_id, config.track_width,
                                              config.via_size, config.via_drill, via_layers,
                                              via_type=_via_kind(config))
                tried += 1
                if not _intra_clear(segs, vias, p_id, n_id, config.track_width, config.via_size, clr,
                                    via_pitch):
                    continue
                # the crossover's own copper against the board: the single-track
                # map first (cheap; it holds the static obstacles, rule areas
                # and edge), then every other net's copper exactly
                if not all(check_line_clearance(base_obstacles, s.start_x, s.start_y, s.end_x, s.end_y,
                                                layer_map[s.layer], config) for s in segs):
                    continue
                if any(base_obstacles.is_via_blocked(*coord.to_grid(v.x, v.y)) for v in vias):
                    continue
                if _board_clear(segs, vias, pcb_data, config):
                    continue
                attempts += 1
                if verbose:
                    print(f"  Crossover: trying a {kind} site at ({V[0]:.2f},{V[1]:.2f}) {L1}->{L2}, "
                          f"{first} dives first ({at:.1f} mm along the path)")
                res = _route_spans(pcb_data, diff_pair, config, obstacles, base_obstacles, unrouted_stubs, route_fn,
                                   geo, segs, vias, L1i, L2i, u, src_ep, tgt_ep, src_dir, tgt_dir, coord, xclr,
                                   add_segments_list_as_obstacles, add_vias_list_as_obstacles)
                if res is None:
                    continue
                all_segs = res['new_segments']
                all_vias = res['new_vias']
                if _pn_tracks_cross_full(all_segs, pcb_data, p_id, n_id):
                    if verbose:
                        print("    crossover result: P/N tracks cross on one layer -- rejected")
                    continue
                p_segs = [s for s in all_segs if s.net_id == p_id]
                n_segs = [s for s in all_segs if s.net_id == n_id]
                _necked, hard = _neck_pair_partner_grazes(
                    p_segs, n_segs, config, pcb_data,
                    p_vias=[v for v in all_vias if v.net_id == p_id],
                    n_vias=[v for v in all_vias if v.net_id == n_id])
                if hard:
                    if verbose:
                        print("    crossover result: intra-pair clearance short -- rejected")
                    continue
                # the whole result (both spans, their connectors and return
                # vias, and the crossover) against the board's other copper:
                # the spans' own gates allow a tolerance and a connector the
                # router necked can still graze where the crossover joins it
                why = _board_clear(all_segs, all_vias, pcb_data, config)
                if why:
                    if verbose:
                        print(f"    crossover result: {why} -- rejected")
                    continue
                if verbose:
                    print(f"  Crossover: routed at ({V[0]:.2f},{V[1]:.2f}) {L1}->{L2} "
                          f"after {attempts} site(s)")
                res['crossover'] = {'at': (round(V[0], 4), round(V[1], 4)), 'layers': (L1, L2),
                                    'first': first, 'kind': kind}
                return res
    if verbose:
        print(f"  Crossover: no site routed ({tried} geometry option(s), {attempts} routed attempt(s))")
    return None


def _route_spans(pcb_data, diff_pair, config, obstacles, base_obstacles, unrouted_stubs, route_fn, geo,
                 xsegs, xvias,
                 L1i, L2i, u, src_ep, tgt_ep, src_dir, tgt_dir, coord, xclr,
                 add_segments_list_as_obstacles, add_vias_list_as_obstacles) -> Optional[dict]:
    """Route the two same-hand spans around a laid crossover; the combined
    result, or None.

    Obstacle maps: the ladder's own maps are never touched. Each span routes on
    a CLONE of the pair map (the router mutates the map it is given: endpoint
    exemptions, cleared source/target cells) with the crossover's barrels and
    inner jog pieces stamped at the pair map's centreline extra clearance, as
    committed copper is. The single-track map (`base_obstacles`, the connector
    checks) is passed as is: the spans' connectors JOIN the crossover's outer
    legs, and the pair's own copper there must not block them; the intra-pair
    and crossing gates on the combined copper guard P against N."""
    ent, ext = geo['entry'], geo['exit']
    entry_ep = _endpoint(ent['P'], ent['N'], L1i, coord)
    exit_ep = _endpoint(ext['P'], ext['N'], L2i, coord)
    # the crossover's barrels and its inner (jog) pieces block both spans;
    # its outer straight runs end at the spans' own endpoints and stay open
    ends = [ent['P'], ent['N'], ext['P'], ext['N']]

    def touches_end(s):
        return any(math.hypot(s.start_x - e[0], s.start_y - e[1]) < 1e-6 or
                   math.hypot(s.end_x - e[0], s.end_y - e[1]) < 1e-6 for e in ends)
    inner = [s for s in xsegs if not touches_end(s)]

    obs1 = obstacles.clone()
    add_vias_list_as_obstacles(obs1, xvias, config, extra_clearance=xclr)
    add_segments_list_as_obstacles(obs1, inner, config, extra_clearance=xclr)
    first = route_fn(pcb_data, diff_pair, config, obs1, base_obstacles, unrouted_stubs,
                     endpoints=(src_ep, entry_ep), forced_source_dir=src_dir,
                     forced_target_dir=(-u[0], -u[1]), swap_allowed_ends=(), force_no_swap=True)
    if not first or first.get('failed') or first.get('probe_blocked'):
        print("    span 1 (start -> crossover) did not route")
        return None
    obs2 = obstacles.clone()
    add_vias_list_as_obstacles(obs2, list(xvias) + list(first.get('new_vias', [])), config, extra_clearance=xclr)
    add_segments_list_as_obstacles(obs2, inner + list(first.get('new_segments', [])), config, extra_clearance=xclr)
    second = route_fn(pcb_data, diff_pair, config, obs2, base_obstacles, unrouted_stubs,
                      endpoints=(exit_ep, tgt_ep), forced_source_dir=(u[0], u[1]),
                      forced_target_dir=tgt_dir, swap_allowed_ends=(), force_no_swap=True)
    if not second or second.get('failed') or second.get('probe_blocked'):
        print("    span 2 (crossover -> end) did not route")
        return None
    out = dict(first)
    out['new_segments'] = list(first.get('new_segments', [])) + list(xsegs) + list(second.get('new_segments', []))
    out['new_vias'] = list(first.get('new_vias', [])) + list(xvias) + list(second.get('new_vias', []))
    out['iterations'] = first.get('iterations', 0) + second.get('iterations', 0)
    for k in ('p_end', 'n_end', 'tgt_stub_dir', 'end_stub_dir', 'tgt_layer_name', 'target_base_dir',
              'p_tgt_stub_length', 'n_tgt_stub_length'):
        if k in second:
            out[k] = second[k]
    for k in ('p_path', 'n_path', 'simplified_path', 'raw_astar_path', 'centerline_path_grid'):
        if isinstance(first.get(k), list) and isinstance(second.get(k), list):
            out[k] = list(first[k]) + list(second[k])
    out['gnd_vias_not_placed'] = (list(first.get('gnd_vias_not_placed') or [])
                                  + list(second.get('gnd_vias_not_placed') or []))
    out['polarity_swap_needed'] = False
    out.pop('polarity_fixed', None)
    out.pop('swap_target_pads', None)
    return out
