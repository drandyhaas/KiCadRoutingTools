"""The routing half of the 3D flow movie: copper as lifetimes on a clock.

Every track, via and pour the chain lays gets the TICK it was born at and
the tick it died at (-1: never). A frame names its tick, and the page shows
an item when `born <= tick` and it has not died before it: an item born at
the frame's tick is drawn lit (white; cyan when the router restored it), an
item dying at it is drawn red, and gone from the next. This is the removed
stage3d timeline's copper-lifetime idea (#1081), on a tick per frame rather
than per op, so a frame is one number.

`play_chain` plays a route_placed.py chain the way `route_movie.play_chain`
plays it in 2D, and with its own helpers: each step's router events
(`<board>_routetrace.json`, written under KICAD_ROUTE_TRACE=1), one per
frame -- a rip shown red for RIP_HOLD frames before it goes, then what the
event laid -- and then whatever else the step changed (a pour, a fanout, a
step with no trace) landing net by net until the copper is the board the
step wrote.
"""
import json
import math
import os

from route_movie import CHUNKS, RIP_HOLD, _board_copper, _skey, _trace_rows, _vkey


#: inner-layer track colours (In1, In2, ...): bright yellow, sky, pink, white,
#: orange -- they are seen through the semi-transparent board
INNER_3D = ((255, 226, 80), (110, 220, 255), (255, 140, 235), (240, 240, 240), (255, 165, 70))

class Copper:
    """The film's copper: every item with its birth and death tick."""

    def __init__(self, layers, outline, cutouts):
        self.layers = list(layers)
        self.li = {n: i for i, n in enumerate(self.layers)}
        self.outline, self.cutouts = outline, cutouts
        self.segs, self.vias, self.pours = [], [], []
        self.live_s, self.live_v, self.live_p = {}, {}, {}
        self.tick = 0

    def next(self):
        self.tick += 1
        return self.tick

    def add_s(self, s, restored=False):
        k = _skey(s)
        if k not in self.live_s:
            self.live_s[k] = len(self.segs)
            self.segs.append([round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
                              round(s.end_y, 4), round(s.width, 4), self.li.get(s.layer, 0),
                              self.tick, -1, int(restored), int(s.net_id or 0)])

    def add_v(self, v, restored=False):
        k = _vkey(v)
        if k not in self.live_v:
            ls = v.layers or [self.layers[0], self.layers[-1]]
            self.live_v[k] = len(self.vias)
            self.vias.append([round(v.x, 4), round(v.y, 4), round(v.size, 4), round(v.drill, 4),
                              self.li.get(ls[0], 0), self.li.get(ls[-1], len(self.layers) - 1),
                              self.tick, -1, int(restored), int(v.net_id or 0)])

    def rm_s(self, k):
        i = self.live_s.pop(k, None)
        if i is not None:
            self.segs[i][7] = self.tick

    def rm_v(self, k):
        i = self.live_v.pop(k, None)
        if i is not None:
            self.vias[i][7] = self.tick

    def set_pours(self, pcb):
        """The pours `pcb` has from this tick on (its zone outlines clipped to
        the board, as stage3d drew them, #1205); the ones it lacks go."""
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        board = Polygon(self.outline).buffer(0)
        if self.cutouts:
            board = board.difference(unary_union([Polygon(c).buffer(0) for c in self.cutouts]))
        want = {}
        for z in getattr(pcb, 'zones', None) or ():
            if z.layer not in self.li or len(z.polygon) < 3:
                continue
            key = (int(z.net_id), z.layer, tuple((round(x, 3), round(y, 3)) for x, y in z.polygon))
            g = Polygon(z.polygon).buffer(0).intersection(board)
            pieces = []
            for piece in getattr(g, 'geoms', [g]):
                if piece.geom_type == 'Polygon' and not piece.is_empty:
                    pieces.append({'poly': [[round(x, 4), round(y, 4)] for x, y in piece.exterior.coords[:-1]],
                                   'holes': [[[round(x, 4), round(y, 4)] for x, y in r.coords[:-1]]
                                             for r in piece.interiors]})
            want[key] = pieces
        for key in [k for k in self.live_p if k not in want]:
            for i in self.live_p.pop(key):
                self.pours[i]['died'] = self.tick
        for key, pieces in want.items():
            if key not in self.live_p:
                self.live_p[key] = []
                for pc in pieces:
                    self.live_p[key].append(len(self.pours))
                    self.pours.append({'net': key[0], 'layer': self.li[key[1]], **pc,
                                       'born': self.tick, 'died': -1})

    def clear(self, rec, label, chunks=CHUNKS):
        """Every track and via cleared, net by net over `chunks` frames, each
        shown red as it goes (route_movie's clearing of the fast route)."""
        nets = sorted({self.segs[i][9] for i in self.live_s.values()}
                      | {self.vias[i][9] for i in self.live_v.values()})
        step = max(1, math.ceil(len(nets) / max(1, chunks)))
        for j in range(0, len(nets), step):
            part = set(nets[j:j + step])
            self.next()
            for k in [k for k, i in self.live_s.items() if self.segs[i][9] in part]:
                self.rm_s(k)
            for k in [k for k, i in self.live_v.items() if self.vias[i][9] in part]:
                self.rm_v(k)
            rec.copper_shot(label)

    def json(self):
        return {'layers': self.layers, 'segs': self.segs, 'vias': self.vias, 'pours': self.pours}


def play_chain(rec, cop, boards, first=None, *, prefix='', first_label='input', budget=0):
    """Play `boards` (a chain, in order) from `first`, the board it started
    from, onto `cop`, each frame through `rec.copper_shot(label)`.
    `budget`: past this many router events in the chain, several share a
    frame (0: one each)."""
    from kicad_parser import parse_kicad_pcb
    traces = {b: os.path.splitext(b)[0] + '_routetrace.json' for b in boards}
    traces = {b: json.load(open(t)) for b, t in traces.items() if os.path.isfile(t)}
    n_events = sum(len(t.get('events', ())) for t in traces.values())
    per = max(1, math.ceil(n_events / budget)) if budget else 1
    cop.next()
    for k in list(cop.live_s):          # whatever is left goes: the chain starts from `first`
        cop.rm_s(k)
    for k in list(cop.live_v):
        cop.rm_v(k)
    if first:
        pcb0 = parse_kicad_pcb(first)
        s0, v0 = _board_copper(pcb0)
        for s in s0.values():
            cop.add_s(s)
        for v in v0.values():
            cop.add_v(v)
        cop.set_pours(pcb0)
    rec.copper_shot(prefix + first_label)
    for board in boards:
        step = prefix + os.path.splitext(os.path.basename(board))[0]
        pcb = parse_kicad_pcb(board)
        events = traces.get(board, {}).get('events', [])
        tlayers = traces.get(board, {}).get('layers') or list(pcb.board_info.copper_layers)
        for i in range(0, len(events), per):
            batch = events[i:i + per]
            span = f"{i + 1}" if len(batch) == 1 else f"{i + 1}-{i + len(batch)}"
            ev = batch[-1]
            label = f"{step}: {span}/{len(events)} {ev.get('event', '')} {ev.get('net_name', '')}"
            cop.next()
            shown = False
            for e in batch:
                ds, dv = _trace_rows(e, 'del', tlayers)
                for s in ds:
                    cop.rm_s(_skey(s))
                for v in dv:
                    cop.rm_v(_vkey(v))
                if len(batch) == 1 and (ds or dv):      # the rip shows red, then goes
                    rec.copper_shot(label, times=RIP_HOLD)
                    cop.next()
                restored = e.get('event') == 'restore'
                a_s, a_v = _trace_rows(e, 'add', tlayers)
                for s in a_s:
                    cop.add_s(s, restored)
                for v in a_v:
                    cop.add_v(v, restored)
                shown = shown or bool(a_s or a_v)
            if shown or len(batch) > 1:
                rec.copper_shot(label)
        # the rest of what the step changed, net by net, until the copper is
        # the board the step wrote
        want_s, want_v = _board_copper(pcb)
        cop.next()
        gone = [k for k in cop.live_s if k not in want_s] + [k for k in cop.live_v if k not in want_v]
        for k in [k for k in cop.live_s if k not in want_s]:
            cop.rm_s(k)
        for k in [k for k in cop.live_v if k not in want_v]:
            cop.rm_v(k)
        before = len(cop.pours), sum(1 for p in cop.pours if p['died'] >= 0)
        cop.set_pours(pcb)
        poured = (len(cop.pours), sum(1 for p in cop.pours if p['died'] >= 0)) != before
        by_net = {}
        for k, s in want_s.items():
            if k not in cop.live_s:
                by_net.setdefault(s.net_id, [[], []])[0].append(s)
        for k, v in want_v.items():
            if k not in cop.live_v:
                by_net.setdefault(v.net_id, [[], []])[1].append(v)
        nets = list(by_net)
        if not nets:
            if gone or poured or not events:
                rec.copper_shot(step)
            continue
        chunk = max(1, math.ceil(len(nets) / CHUNKS))
        for j in range(0, len(nets), chunk):
            if j:
                cop.next()
            part = nets[j:j + chunk]
            for n in part:
                for s in by_net[n][0]:
                    cop.add_s(s)
                for v in by_net[n][1]:
                    cop.add_v(v)
            names = [pcb.nets[n].name for n in part if n in pcb.nets and pcb.nets[n].name]
            rec.copper_shot(f"{step}: {j // chunk + 1}/{math.ceil(len(nets) / chunk)} "
                            + ', '.join(names[:3]) + (f" +{len(names) - 3}" if len(names) > 3 else ''))


ZOOM = 14          # frames of the camera pulling back to the seeds, and flying back in


def film_routes(rec, work, judged, scores, best, full, filmed, budget=0):
    """The routing half of the flow movie, after the placement of seed
    `filmed` (one the judge did not pick, when there is one): its fast
    route; the camera pulling back to every judged seed's fast-routed board
    side by side, each labelled with its score; the best one framed and the
    camera flying in to it; its fast route cleared and its full route played
    -- place_movie's `film_routes`, on the 3D board, from the same work dir
    (place_movie's `run_flow`)."""
    import render_theme
    from kicad_parser import parse_kicad_pcb
    from place_movie import _chain, _score_text
    from placement_story import _ease
    th = render_theme.theme('dark', strict=False)
    layers = list(rec.pcb.board_info.copper_layers)
    pal = render_theme.layer_palette(layers, th)
    # an inner layer's tracks are seen through the board: bright, and never
    # the theme's green, which vanished against the green board
    inner = iter(INNER_3D * len(layers))
    track = {n: (c if n in ('F.Cu', 'B.Cu') else next(inner)) for n, c in pal.items()}
    cop = Copper(layers, rec.scene['outline'], rec.scene['cutouts'])
    rec.cop = cop
    hold = rec.hold
    rec.copper_shot(f"seed {filmed} placed: routing it", times=hold)
    first, boards = _chain(work, f'fast_s{filmed}')
    play_chain(rec, cop, boards, first, prefix='fast route | ',
               first_label=f'placed, seed {filmed}', budget=budget)
    cop.next()          # a hold lights nothing: its copper was all laid before it
    rec.copper_shot(f"fast route, seed {filmed}: {_score_text(scores[filmed])}", times=hold)
    # every judged seed's board, at its placement with its fast route
    seeds = sorted(scores)
    cells = []
    for s in seeds:
        placed = parse_kicad_pcb(os.path.join(work, f's{s}.kicad_pcb'))
        _f, bs = _chain(work, f'fast_s{s}')
        routed = parse_kicad_pcb(bs[-1])
        c = Copper(layers, rec.scene['outline'], rec.scene['cutouts'])
        segs, vias = _board_copper(routed)
        for o in segs.values():
            c.add_s(o)
        for o in vias.values():
            c.add_v(o)
        c.set_pours(routed)
        poses = []
        for r in rec.refs:
            fp = placed.footprints.get(r)
            poses.append(None if fp is None else
                         [round(fp.x, 4), round(fp.y, 4), round((fp.rotation or 0.0) % 360.0, 3),
                          round(rec.base(r, fp.x, fp.y), 3)])
        cu = c.json()
        cells.append({'seed': s, 'best': s == best, 'filmed': s == filmed, 'poses': poses,
                      'copper': {'segs': cu['segs'], 'vias': cu['vias'], 'pours': cu['pours']},
                      'label': (f"BEST  seed {s}: " if s == best else f"seed {s}: ") + _score_text(scores[s])})
    fi, bi = seeds.index(filmed), seeds.index(best)
    rec.scene['grid'] = {'cells': cells, 'filmed': fi, 'best': bi}
    title = f"the {len(scores)} cheapest seeds by the objective, fast-routed"
    grid = dict(grid=True, labels=True, ro=fi)
    for i in range(1, ZOOM + 1):
        rec.copper_shot(title, gv=['out', round(_ease(i / ZOOM), 4)], **grid)
    rec.copper_shot(title, times=2 * hold, gv=['out', 1.0], **grid)
    pick = f"the best: seed {best} ({_score_text(scores[best])})"
    for blink in range(3):
        rec.copper_shot(pick, times=max(1, hold // 2), gv=['out', 1.0], best=blink % 2 == 0, **grid)
    rec.copper_shot(pick, times=hold, gv=['out', 1.0], best=True, **grid)
    if best != filmed:
        # the live board becomes the best seed's, out here where its copy and
        # the filmed seed's look the same: its placement and its fast route
        # (laid on a tick no frame shows, so nothing is lit or red)
        placed = parse_kicad_pcb(os.path.join(work, f's{best}.kicad_pcb'))
        rec.route_poses = {r: (fp.x, fp.y, (fp.rotation or 0.0) % 360.0)
                           for r, fp in placed.footprints.items() if r in rec.idx}
        cop.next()
        for k in list(cop.live_s):
            cop.rm_s(k)
        for k in list(cop.live_v):
            cop.rm_v(k)
        _f, bs = _chain(work, f'fast_s{best}')
        routed = parse_kicad_pcb(bs[-1])
        segs, vias = _board_copper(routed)
        for o in segs.values():
            cop.add_s(o)
        for o in vias.values():
            cop.add_v(o)
        cop.set_pours(routed)
        cop.next()
        grid['ro'] = bi
    for i in range(1, ZOOM + 1):
        rec.copper_shot(pick, gv=['in', round(_ease(i / ZOOM), 4)], best=True, **grid)
    # from here on the live board stands in the best seed's cell, alone: the
    # other seeds' boards would hang at the edges of the frame
    rec.extra = {'ro': bi, 'gv': ['in', 1.0]}
    cop.clear(rec, f"clearing the fast route of seed {best}")
    first, boards = _chain(work, f'full_s{best}')
    play_chain(rec, cop, boards, first, prefix='full route | ',
               first_label=f'seed {best}, cleared', budget=budget)
    cop.next()
    rec.copper_shot(f"full route, seed {best}: {_score_text(full)}", times=3 * hold)
    rec.scene['copper'] = cop.json()
    rec.scene['colors'].update(
        layers=[list(track.get(n, (200, 120, 60))) for n in layers],
        pour_layers=[list(pal.get(n, (200, 120, 60))) for n in layers], via=list(th.rgb('via')),
        # a rip is magenta, not the 2D movie's red: F.Cu is red here, and a
        # red rip on it read as nothing happening
        new=list(th.rgb('event_new')), rip=[235, 70, 235],
        restored=list(th.rgb('event_restored')))
