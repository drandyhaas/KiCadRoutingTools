"""plane_packing.py -- a plane plan's supplies packed onto the FEWEST of its layers that still serve every BGA ball.

A plan names each plane net's layer (route_planes --nets / --plane-layers). Its SUPPLIES -- every plane net but the
ground family, whose reference planes stand as the plan names them -- are moved onto as few of the layers the plan gave
them as serve every BGA ball of each, the rest of those layers left free for signals. A layer serves a supply's balls
when, split among the supplies sharing it (route_planes' own split, run on that layer alone), each ball stands in the
piece of its own net's pour that holds the most of that net's pads -- not in another net's share, not on an island cut
off from the rest of its net -- and that pour is nowhere narrower than the net's declared power width (--power-nets /
--power-nets-widths) on the way to it, or to any other pad of the net the pour covers: the share shrunk by half that
width still reaches within half that width of each (a neck or a finger narrower than it vanishes, and a pad behind one
is unserved). First fit, the supplies
with the most balls first: each onto the first layer already in use that still serves every ball of every supply on
it, else onto the next of the plan's layers -- those beside a ground plane in the stackup LAST, so the layers freed for
signals are the ones with a ground plane beside them, as the outer signal layers have theirs (a human's stack: signals
on F.Cu and B.Cu and on an inner layer between ground planes, the supplies between)
"""
import contextlib
import io

from net_queries import is_ground_net_name

BALL_TOL = 0.05         # mm: a ball's centre this near its net's pour is in it (the pour's edge is a polygon's)
# create_plane's arguments a trial split sets itself, or does not take on (the packing's own call)
PACK_OWN_ARGS = frozenset({'input_file', 'output_file', 'net_names', 'plane_layers', 'layer_nets', 'dry_run',
                           'return_results', 'pcb_data', 'progress_callback', 'cancel_check', 'verbose',
                           'debug_lines', 'fewest_plane_layers'})


def ball_positions(pcb_data):
    """{net id: [(x, y)]} -- the balls of every BGA on the board, by net"""
    from kicad_parser import find_components_by_type
    out = {}
    for fp in find_components_by_type(pcb_data, 'BGA'):
        for p in fp.pads:
            if p.net_id:
                out.setdefault(p.net_id, []).append((p.global_x, p.global_y))
    return out


def pad_positions(pcb_data):
    """{net id: [(x, y)]} -- every pad of every net"""
    out = {}
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            if p.net_id:
                out.setdefault(p.net_id, []).append((p.global_x, p.global_y))
    return out


def unserved_balls(zones, net_ids, balls, pads, widths=None):
    """{net id: [the balls of it the layer's pours do not serve]} for the pours `zones` of one layer (route_planes'
    zone dicts: net_id, polygon_points, priority, clearance) shared by `net_ids`: each net's own share is its pours
    less every other net's pour above it (its priority higher), grown by that pour's clearance -- KiCad fills the
    higher first and keeps the clearance off it -- shrunk by half the net's declared width (`widths` {net id: mm}),
    and a ball is served when it stands within half that width of the piece of that shrunk share holding the most of
    its net's pads: reached by copper no narrower than the width. So is every other pad of the net the share's main
    piece covers (a pad outside it the route step reaches by a track): each listed with the balls when it is not"""
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union
    polys = []
    for z in zones:
        pts = z.get('polygon_points') or []
        if len(pts) >= 3 and z.get('net_id') in net_ids:
            g = Polygon(pts)
            if not g.is_valid:
                g = g.buffer(0)
            polys.append((z['net_id'], int(z.get('priority') or 0), float(z.get('clearance') or 0.0), g))
    out = {}
    for nid in net_ids:
        mine = [(pr, g) for n, pr, _c, g in polys if n == nid]
        if not mine:
            out[nid] = list(balls.get(nid, ()))
            continue
        share = unary_union([g.difference(unary_union([g2.buffer(c2) for n2, pr2, c2, g2 in polys
                                                       if n2 != nid and pr2 > pr] or [Polygon()]))
                             for pr, g in mine])
        own = pads.get(nid, ())
        whole = [share] if share.geom_type == 'Polygon' else list(getattr(share, 'geoms', []))
        whole = [p for p in whole if not p.is_empty]
        if not whole:
            out[nid] = list(balls.get(nid, ()))
            continue
        # (the pads the pour serves: in the piece of it holding the most of them)
        main0 = max(whole, key=lambda p: (sum(1 for q in own if p.distance(Point(q)) <= BALL_TOL), p.area))
        ball_set = set(balls.get(nid, ()))
        targets = list(ball_set) + [q for q in own if q not in ball_set and main0.distance(Point(q)) <= BALL_TOL]
        half = float((widths or {}).get(nid) or 0.0) / 2
        if half > 0:
            share = share.buffer(-half)
        pieces = [share] if share.geom_type == 'Polygon' else list(getattr(share, 'geoms', []))
        pieces = [p for p in pieces if not p.is_empty]
        if not pieces:
            out[nid] = targets
            continue
        main = max(pieces, key=lambda p: (sum(1 for q in own if p.distance(Point(q)) <= half + BALL_TOL), p.area))
        bad = [b for b in targets if main.distance(Point(b)) > half + BALL_TOL]
        if bad:
            out[nid] = bad
    return out


def _split(input_file, layer, nets, plane_kw, pcb_data=None):
    """route_planes' pours of `nets` sharing `layer` alone (a dry run, on a copy of `pcb_data` when one is given):
    its zone dicts"""
    import copy
    from route_planes import create_plane
    with contextlib.redirect_stdout(io.StringIO()):
        got = create_plane(input_file=input_file, output_file='', net_names=list(nets),
                           plane_layers=[layer] * len(nets), layer_nets={layer: list(nets)},
                           dry_run=True, return_results=True,
                           pcb_data=copy.deepcopy(pcb_data) if pcb_data is not None else None, **plane_kw)
    return list(got[5]) if isinstance(got, tuple) and len(got) > 5 else []


def pack_plane_layers(input_file, net_names, plane_layers, plane_kw, pcb_data=None, log=print):
    """(net names, plane layers, layer -> nets) -- the plan `net_names` / `plane_layers` (route_planes' expanded pairs)
    with its supplies packed onto the fewest of their layers that serve every BGA ball (see the module), the ground
    family's pairs as given. `plane_kw`: create_plane's other arguments, the trial splits'; `pcb_data` the board when the
    caller holds it (the Planes tab's live board: each trial on a copy), else `input_file` read. A supply no layer in
    use can take and none left to open stays on the layer its trials left the fewest of its balls unserved on, and is
    named"""
    from kicad_parser import parse_kicad_pcb
    pcb = pcb_data
    if pcb is None:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(input_file)
    from plane_io import resolve_net_id
    nid_of = lambda nm: resolve_net_id(pcb, nm)  # noqa: E731  (the planes step's own reading of a net's name)
    balls, pads = ball_positions(pcb), pad_positions(pcb)
    # (each supply's declared power width, the planes step's own --power-nets / --power-nets-widths)
    widths = {}
    for n, w in zip(plane_kw.get('power_nets') or (), plane_kw.get('power_nets_widths') or ()):
        if nid_of(n) is not None:
            widths[nid_of(n)] = float(w)
    fixed = [(n, L) for n, L in zip(net_names, plane_layers) if is_ground_net_name(n)]
    supplies, layers = [], []
    for n, L in zip(net_names, plane_layers):
        if is_ground_net_name(n):
            continue
        if n not in supplies:
            supplies.append(n)
        if L not in layers:
            layers.append(L)
    # (the stackup's order: a supply layer beside a ground plane is the last taken, the first left free for signals)
    stack = list(getattr(pcb.board_info, 'copper_layers', None) or [])
    gnd = {L for _n, L in fixed}
    beside = lambda L: L in stack and any(  # noqa: E731
        0 <= stack.index(L) + d < len(stack) and stack[stack.index(L) + d] in gnd for d in (-1, 1))
    layers = sorted(layers, key=lambda L: (beside(L), layers.index(L)))
    if len(layers) < 2:
        out_n, out_L = list(net_names), list(plane_layers)
        return out_n, out_L, _layer_nets(out_n, out_L), {'packed': False, 'why': 'the supplies have one layer',
                                                          'pairs': [[n, L] for n, L in zip(out_n, out_L)]}
    order = sorted(supplies, key=lambda n: (-len(balls.get(nid_of(n), ())), supplies.index(n)))
    placed = {}                             # layer -> [supplies], in the order they were placed
    trials, near = 0, {}
    for n in order:
        nid = nid_of(n)
        done = False
        for L in [L for L in layers if L in placed]:
            nets = placed[L] + [n]
            trials += 1
            bad = unserved_balls(_split(input_file, L, nets, plane_kw, pcb_data), {nid_of(m) for m in nets}, balls,
                                 pads, widths)
            miss = sum(len(v) for v in bad.values())
            if miss == 0:
                placed[L].append(n)
                done = True
                break
            if n not in near or miss < near[n][0]:
                near[n] = (miss, L)
        if done:
            continue
        free = [L for L in layers if L not in placed]
        if free:
            placed[free[0]] = [n]
        else:
            L = near[n][1]
            placed[L].append(n)
            log(f'  fewest planes: {n} -- no layer in use serves every ball with it, none left to open: on {L}, '
                f'{near[n][0]} ball(s) unserved there')
    out_n = [n for n, _L in fixed] + [n for L in layers if L in placed for n in placed[L]]
    out_L = [L for _n, L in fixed] + [L for L in layers if L in placed for _n in placed[L]]
    freed = [L for L in layers if L not in placed]
    log(f'  fewest planes: {len(supplies)} supplies on {len(placed)} of the plan\'s {len(layers)} supply layers '
        f'({trials} trial split(s); every pour no narrower than its net\'s declared width: {len(widths)} declared) -- ' + '; '.join(f'{L}: {", ".join(placed[L])}' for L in layers if L in placed)
        + (f' -- FREE for signals: {", ".join(freed)}' if freed else '')
        + (f' (NO ground plane beside: {", ".join(L for L in freed if not beside(L))})'
           if any(not beside(L) for L in freed) else ''))
    return out_n, out_L, _layer_nets(out_n, out_L), {'packed': True, 'pairs': [[n, L] for n, L in zip(out_n, out_L)],
                                                      'layers': {L: placed[L] for L in placed},
                                                      'freed': freed, 'freed_beside_ground': [L for L in freed
                                                                                             if beside(L)],
                                                      'trials': trials}


def _layer_nets(net_names, plane_layers):
    out = {}
    for n, L in zip(net_names, plane_layers):
        out.setdefault(L, []).append(n)
    return out
