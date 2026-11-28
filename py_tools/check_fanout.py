#!/usr/bin/env python3
"""Which pins of a fine-pitch part cannot fan out to a via with this placement,
and which parts block them?

    python3 check_fanout.py board.kicad_pcb --component U1
    python3 check_fanout.py board.kicad_pcb -c U1 --layers F.Cu B.Cu --json

A fan-out is a track at the pin's net-class width from the pad to a via of
the net-class size, outside the part's pad ring, legal on every copper layer
and outside any rule area that bans vias. Each pin gets one verdict:

    escapes      a via site is reachable, and one is left for it when every
                 pin fans out at once.
    local        the pin reaches copper of its own net outside the part (a
                 passive's pad, a locked track or via) on its own layer.
    exposed pad  its net is on the part's interior pad, which carries it.
    through-hole the pad is drilled, so it is its own via.
    unconnected  its net reaches no other part or pour.
    caged        no via site is reachable even with the board to itself.
                 Named with the parts whose pads wall the pocket.
    crowded      it reaches via sites alone, but they run out when all pins
                 fan out at once. Reported by group: the pins that compete
                 for the same sites, the sites, and the parts that wall them.

`caged` is exact for the raster: each net's own field (`reachability.
slack_field`), every foreign object at the clearance its pair gets.
`crowded` is a max flow on a lane-wide lattice priced at the Default class:
cells of one track pitch carry one track, via sites sit on a grid that keeps a
track between neighbours (best of four offsets), and each site serves one pin.
It is an estimate, and the report says so. `local` pins take no part in it.
`check_reachability --part` asks the track-only cousin of this question (how
many nets leave the window at once); it does not count a via site per pin.

Layers typed `power` (planes) are left out by default, as `route.py` bans
them; the via still passes through them. The board's routed copper is set aside
(`reachability.without_routed_copper`); locked copper stays.

Parts with signal pads inside the pad ring, like BGAs, are refused: use
`bga_fanout.py` for them.

Exit codes: 0 every pin fans out, 1 a pin is caged or crowded, 2 usage error.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)
import argparse
import json
import math
import os
import sys
from collections import Counter

import numpy as np

import list_nets
from kicad_parser import non_aperture_pads, parse_kicad_pcb, pour_zones
from net_queries import pad_on_layer
from placement import escape, reachability as reach
from placement.escape_flow import DEFAULT_MARGIN_MM, MAX_STEP_MM
from placement.legality import PadClearanceModel

WALLS_SHOWN = 8
ORDER = ('escapes', 'local', 'exposed pad', 'through-hole', 'unconnected',
         'caged', 'crowded')


class Unresolvable(Exception):
    """The arguments name nothing usable on this board (exit 2, never 1)."""


class Grid:
    """The window: `k` raster steps per lattice cell of one Default lane."""

    def __init__(self, ring, margin, cell):
        self.cell = cell
        self.k = max(1, int(math.ceil(cell / MAX_STEP_MM - 1e-9)))
        self.step = cell / self.k
        self.tol = 0.75 * self.step
        self.ncx = int(math.ceil((ring[2] - ring[0] + 2 * margin) / cell))
        self.ncy = int(math.ceil((ring[3] - ring[1] + 2 * margin) / cell))
        x0, y0 = ring[0] - margin, ring[1] - margin
        self.view = (x0, y0, x0 + self.ncx * cell, y0 + self.ncy * cell)
        self.H, self.W = self.ncy * self.k, self.ncx * self.k
        self.yy = y0 + (np.arange(self.H) + 0.5)[:, None] * self.step
        self.xx = x0 + (np.arange(self.W) + 0.5)[None, :] * self.step
        dx = np.maximum(np.maximum(ring[0] - self.xx, self.xx - ring[2]), 0)
        dy = np.maximum(np.maximum(ring[1] - self.yy, self.yy - ring[3]), 0)
        self.ring_dist = np.hypot(dx, dy)
        self.cid = ((np.arange(self.H) // self.k)[:, None] * self.ncx
                    + (np.arange(self.W) // self.k)[None, :])

    def blank(self):
        return np.zeros((self.H, self.W), bool)

    def crop(self, a):
        return a[:self.H, :self.W]

    def pad_mask(self, p):
        m = self.blank()
        reach._rect(m, p.global_x, p.global_y, p.size_x, p.size_y, self.view,
                    self.step, getattr(p, 'rect_rotation', 0.0) or 0.0)
        return m

    def pad_slice(self, p):
        """The raster rows and columns of `p`'s bounding box, or None."""
        r = math.hypot(p.size_x, p.size_y) / 2
        c0 = int((p.global_x - r - self.view[0]) / self.step)
        c1 = int((p.global_x + r - self.view[0]) / self.step) + 1
        r0 = int((p.global_y - r - self.view[1]) / self.step)
        r1 = int((p.global_y + r - self.view[1]) / self.step) + 1
        if c1 <= 0 or r1 <= 0 or c0 >= self.W or r0 >= self.H:
            return None
        return slice(max(0, r0), r1), slice(max(0, c0), c1)


def _via_ban(pcb, grid, cu):
    ban = grid.blank()
    for layer in cu:
        for poly, holes, _tracks_ok, vias_ok in reach.rule_area_keepouts(pcb, layer):
            if not vias_ok:
                reach._region(ban, poly, holes, grid.view, grid.step)
    return ban


def _walls(pcb, grid, region, lay, cu, clearance, track, via, ref, net_id,
           marks):
    """Parts whose pads line `region`: on `lay` within a track's reach of it,
    on another layer within a via's. Then the rule area and the edge."""
    from scipy import ndimage
    dist = ndimage.distance_transform_edt(~region) * grid.step
    near_lay = clearance + track / 2 + 2 * grid.step
    near_via = clearance + via / 2 + 2 * grid.step
    refs = {}
    for r, f in pcb.footprints.items():
        for p in f.pads:
            if (p.net_id and p.net_id == net_id) or p.pad_type == 'np_thru_hole':
                continue
            sl = grid.pad_slice(p)
            if sl is None or not dist[sl].size:
                continue
            if pad_on_layer(p, lay):
                near = near_lay
            elif any(pad_on_layer(p, l) for l in cu):
                near = near_via
            else:
                continue
            if dist[sl].min() <= near:
                refs[r] = refs.get(r, 0) + 1
    own = refs.pop(ref, 0)
    out = sorted(refs, key=lambda r: (-refs[r], r))
    if not out and own:
        out = [f'{ref} (its own pads)']
    touch = ndimage.binary_dilation(region, iterations=int(near_lay / grid.step) + 1)
    for name, mask in marks.items():
        if (touch & mask).any():
            out.append(name)
    return out


def _flow(grid, free, sites_ok, track, clearance, via, halos):
    """Max flow from each pin's halo to via sites, one track per lattice cell
    and one pin per site, at the best of four site-grid offsets. Returns the
    flow per pin, which pins are on the source side of the min cut, and that
    side's track space."""
    from scipy import ndimage
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import breadth_first_order, maximum_flow
    g, H, W, cid = grid, grid.H, grid.W, grid.cid
    spacing = via + 2 * clearance + track + 2 * g.step
    foot = via / 2 + clearance + track / 2 - g.tol
    ncell = g.ncx * g.ncy
    best = None
    for ox, oy in ((0.0, 0.0), (0.5, 0.0), (0.0, 0.5), (0.5, 0.5)):
        fr = free.copy()
        sites = []
        for sy in np.arange(g.view[1] + oy * spacing, g.view[3], spacing):
            for sx in np.arange(g.view[0] + ox * spacing, g.view[2], spacing):
                ix, iy = int((sx - g.view[0]) / g.step), int((sy - g.view[1]) / g.step)
                if 0 <= ix < W and 0 <= iy < H and sites_ok[iy, ix]:
                    sites.append((float(sx), float(sy)))
        site_id = np.full((H, W), -1, np.int32)
        for i, (sx, sy) in enumerate(sites):
            d = np.hypot(g.xx - sx, g.yy - sy)
            fr &= d >= foot
            site_id[(d < foot + 2 * g.step) & (d >= foot)] = i
        ns, npin = len(sites), len(halos)
        p0 = 2 * ncell + ns
        src, snk = p0 + npin, p0 + npin + 1
        nv = snk + 1
        rows, cols = [], []
        used = np.unique(cid[fr])
        rows.append(2 * used)
        cols.append(2 * used + 1)
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            xa, xb = slice(max(0, -dx), W - max(0, dx)), slice(max(0, dx), W - max(0, -dx))
            a, b = fr[0:H - dy, xa], fr[dy:H, xb]
            ca, cb = cid[0:H - dy, xa], cid[dy:H, xb]
            m = a & b & (ca != cb)
            pr = np.unique(np.stack([ca[m], cb[m]], 1), axis=0).reshape(-1, 2)
            rows += [2 * pr[:, 0] + 1, 2 * pr[:, 1] + 1]
            cols += [2 * pr[:, 1], 2 * pr[:, 0]]
        m = fr & (site_id >= 0)
        pr = np.unique(np.stack([cid[m], site_id[m]], 1), axis=0).reshape(-1, 2)
        rows.append(2 * pr[:, 0] + 1)
        cols.append(2 * ncell + pr[:, 1])
        rows.append(2 * ncell + np.arange(ns))
        cols.append(np.full(ns, snk))
        for i, halo in enumerate(halos):
            cells = np.unique(cid[ndimage.binary_dilation(halo, iterations=2) & fr])
            touched = np.unique(site_id[halo & (site_id >= 0)])
            rows += [np.array([src]), np.full(len(cells), p0 + i),
                     np.full(len(touched), p0 + i)]
            cols += [np.array([p0 + i]), 2 * cells, 2 * ncell + touched]
        rc = np.unique(np.stack([np.concatenate(rows), np.concatenate(cols)],
                                1).astype(np.int32), axis=0)
        cap = csr_matrix((np.ones(len(rc), np.int32), (rc[:, 0], rc[:, 1])),
                         shape=(nv, nv))
        mf = maximum_flow(cap, src, snk)
        if best is None or mf.flow_value > best[0]:
            best = (mf.flow_value, cap, mf, fr, p0, src, nv)
    _fv, cap, mf, fr, p0, src, nv = best
    flow = mf.flow.tocsr()
    c = cap.tocoo()
    f = np.asarray(flow[c.row, c.col]).ravel()
    fwd = c.data - f > 0
    rr = np.concatenate([c.row[fwd], c.col[f > 0]])
    cc = np.concatenate([c.col[fwd], c.row[f > 0]])
    resid = csr_matrix((np.ones(len(rr), np.int8), (rr, cc)), shape=(nv, nv))
    seen = np.zeros(nv, bool)
    seen[breadth_first_order(resid, src, return_predecessors=False)] = True
    got = np.asarray(flow[src].todense()).ravel()[p0:p0 + len(halos)]
    side = fr & np.isin(cid, np.flatnonzero(seen[0:2 * ncell:2]))
    return got, seen[p0:p0 + len(halos)], side


def check_fanout(pcb_file, ref, layers=None, margin=DEFAULT_MARGIN_MM):
    ndimage = reach._require_scipy()

    if not os.path.isfile(pcb_file):
        raise Unresolvable(f"no board file '{pcb_file}'")
    pcb, n_segs, n_vias = reach.without_routed_copper(parse_kicad_pcb(pcb_file))
    if ref not in pcb.footprints:
        raise Unresolvable(f"no part '{ref}' on this board")
    cu = list(pcb.board_info.copper_layers)
    planes = pcb.board_info.plane_layers
    if layers is None:
        layers = [l for l in cu if l not in planes]
    unknown = [l for l in layers if l not in cu]
    if unknown:
        raise Unresolvable(f"not copper layers of this board: {', '.join(unknown)}")
    if len(layers) < 2:
        raise Unresolvable('a fan-out to a via needs two allowed layers, got '
                           + (', '.join(layers) or 'none'))

    lane = escape.lane_rules(pcb, pcb_file)
    dr = lane.design_rules or {}
    classes = dr.get('classes') or {}

    def net_rules(name):
        c = classes.get(list_nets.resolve_net_class(name, dr)) or {}
        return (float(c.get('track_width') or lane.track_width),
                float(c.get('clearance') or lane.clearance),
                float(c.get('via_diameter') or lane.via_diameter))

    model = PadClearanceModel.for_board(pcb, lane.clearance, pcb_file)
    edge = list_nets.board_floor_knobs(pcb_file, design_rules=dr or None)[1]
    fp = pcb.footprints[ref]
    pads = non_aperture_pads(fp)
    boxes = {id(p): (p.global_x - p.size_x / 2, p.global_y - p.size_y / 2,
                     p.global_x + p.size_x / 2, p.global_y + p.size_y / 2)
             for p in pads}
    ring = (min(b[0] for b in boxes.values()), min(b[1] for b in boxes.values()),
            max(b[2] for b in boxes.values()), max(b[3] for b in boxes.values()))
    grid = Grid(ring, margin, lane.track_width + lane.clearance)
    pitch = escape.pad_pitch(fp)
    interior = [p for p in pads if escape.face_of(p, ring, pitch, boxes[id(p)]) is None]
    ep_nets = {p.net_id for p in interior if p.net_id}
    if len(ep_nets) > 1:
        raise Unresolvable(f"{ref} has signal pads inside its pad ring, like a BGA; "
                           f"use bga_fanout.py")
    elsewhere = {p.net_id for r, f in pcb.footprints.items() if r != ref
                 for p in f.pads if p.net_id}
    elsewhere |= {z.net_id for z in pcb.zones if z.net_id}
    ban = _via_ban(pcb, grid, cu)
    ban_dist = ndimage.distance_transform_edt(~ban) * grid.step
    marks = {'rule area': ban.copy(), 'board edge': grid.blank()}
    for layer in cu:
        for poly, holes, tracks_ok, _v in reach.rule_area_keepouts(pcb, layer):
            if not tracks_ok:
                reach._region(marks['rule area'], poly, holes, grid.view, grid.step)
    for (ax, ay), (bx, by) in reach.outline_edges(pcb):
        reach._capsule(marks['board edge'], ax, ay, bx, by, grid.step, grid.view,
                       grid.step)

    pins, todo = [], {}
    for p in pads:
        if p in interior:
            continue
        name = pcb.nets[p.net_id].name if p.net_id in pcb.nets else ''
        pin = {'pad': p.pad_number, 'net': name, 'verdict': None}
        pins.append(pin)
        lay = next((l for l in layers if pad_on_layer(p, l)), None)
        if not p.net_id or p.net_id not in elsewhere:
            pin['verdict'] = 'unconnected'
        elif p.net_id in ep_nets:
            pin['verdict'] = 'exposed pad'
        elif p.pad_type == 'thru_hole':
            pin['verdict'] = 'through-hole'
        elif lay is None:
            raise Unresolvable(f"{ref}.{p.pad_number} is on no allowed layer "
                               f"({', '.join(layers)})")
        else:
            pin['layer'] = lay
            todo.setdefault(p.net_id, []).append((p, pin))

    ring_pads = {}
    for p in pads:
        ring_pads.setdefault(p.net_id, []).append(p)
    halos = []
    for net_id, members in todo.items():
        name = members[0][1]['net']
        track, clearance, via = net_rules(name)
        pricing = reach.PairPricing(model, net_id, board_copper=cu)
        field = {l: [grid.crop(a) for a in reach.slack_field(
            pcb, net_id, l, grid.view, lane.clearance, step=grid.step,
            edge_clearance=edge, pricing=pricing)] for l in cu}
        via_ok = np.logical_and.reduce([field[l][2] >= via / 2 - grid.tol for l in cu])
        via_ok &= (grid.ring_dist >= via / 2) & (ban_dist >= via / 2)
        own_ring = grid.blank()
        for q in ring_pads[net_id]:
            own_ring |= grid.pad_mask(q)
        labels = {}
        halo_steps = int(math.ceil((clearance + track / 2) / grid.step)) + 2
        for p, pin in members:
            lay = pin['layer']
            pin['rules'] = {'track_width': track, 'clearance': clearance,
                            'via_diameter': via}
            if lay not in labels:
                labels[lay] = ndimage.label(field[lay][0] >= track - 2 * grid.tol,
                                            structure=np.ones((3, 3)))[0]
            lbl = labels[lay]
            pm = grid.pad_mask(p)
            comps = np.unique(lbl[pm & (lbl > 0)])
            region = np.isin(lbl, comps[comps > 0])
            other = region & field[lay][1] & ~own_ring
            if other.any():
                pin['verdict'] = 'local'
                pin['reaches'] = min(
                    ((math.hypot(q.global_x - p.global_x, q.global_y - p.global_y),
                      f'{r}.{q.pad_number}') for r, f in pcb.footprints.items()
                     if r != ref for q in f.pads if q.net_id == net_id
                     and pad_on_layer(q, lay) and (grid.pad_mask(q) & other).any()),
                    default=(0, 'own-net copper'))[1]
            elif not (region & via_ok).any():
                pin['verdict'] = 'caged'
                pin['walls'] = _walls(pcb, grid, region, lay, cu, clearance, track,
                                      via, ref, net_id, marks)
            else:
                pin['verdict'] = 'escapes'
                halos.append((pin, region & ndimage.binary_dilation(pm, iterations=halo_steps)))

    groups = []
    if halos:
        flat = {l: grid.crop(reach.slack_field(pcb, -1, l, grid.view, lane.clearance,
                                               step=grid.step, edge_clearance=edge)[2])
                for l in cu}
        tw, clr, via = lane.track_width, lane.clearance, lane.via_diameter
        lay = halos[0][0]['layer']
        free = flat[lay] >= tw / 2 - grid.tol
        sites_ok = np.logical_and.reduce([flat[l] >= via / 2 - grid.tol for l in cu])
        sites_ok &= (grid.ring_dist >= via / 2) & (ban_dist >= via / 2)
        got, cut_side, side = _flow(
            grid, free, sites_ok, tw, clr, via, [h for _p, h in halos])
        short = [i for i, g in enumerate(got) if g <= 0]
        for i in short:
            halos[i][0]['verdict'] = 'crowded'
        comp = ndimage.label(side | np.logical_or.reduce(
            [halos[i][1] for i in short] + [grid.blank()]), structure=np.ones((3, 3)))[0]
        for c in sorted({int(x) for i in short for x in np.unique(comp[halos[i][1]]) if x}):
            region = comp == c
            members = [i for i, (_p, h) in enumerate(halos)
                       if (cut_side[i] or got[i] <= 0) and (h & region).any()]
            groups.append({
                'pins': [halos[i][0]['pad'] for i in members],
                'exits': int(sum(got[i] for i in members)),
                'short': [halos[i][0]['pad'] for i in members if got[i] <= 0],
                'walls': _walls(pcb, grid, region, lay, cu, clr, tw, via, ref, None,
                                marks),
                'window_edge': bool(region[0].any() or region[-1].any()
                                    or region[:, 0].any() or region[:, -1].any()),
            })

    return {
        'board': pcb_file, 'ref': ref, 'layers': list(layers),
        'planes': {l: sorted({z.net_name for z in pour_zones(pcb.zones) if z.layer == l})
                   for l in planes},
        'rules': {'default': lane.to_dict(), 'edge_clearance': edge},
        'window': {'margin_mm': margin, 'ring': [round(v, 3) for v in ring],
                   'step_mm': round(grid.step, 4), 'lane_mm': round(grid.cell, 4)},
        'routed_copper_set_aside': {'tracks': n_segs, 'vias': n_vias},
        'counts': dict(Counter(p['verdict'] for p in pins)), 'pins': pins,
        'groups': groups,
    }


def _pad_key(pin):
    n = pin['pad']
    return (0, int(n), '') if n.isdigit() else (1, 0, n)


def report(r):
    ref = r['ref']
    out = [f"check_fanout: {ref} on {os.path.basename(r['board'])}"]
    planes = ', '.join(f"{l} [{', '.join(n) or '?'}]" for l, n in r['planes'].items())
    out.append(f"  layers {', '.join(r['layers'])}"
               + (f"; plane layers left out: {planes}" if planes else ''))
    out.append(f"  Default rules {escape.format_rules(r['rules']['default'])};"
               " each pin at its net class")
    rc = r['routed_copper_set_aside']
    out.append(f"  window {r['window']['margin_mm']:g} mm round the pad ring;"
               f" routed copper set aside: {rc['tracks']} tracks, {rc['vias']} vias")
    c = r['counts']
    out.append(f"  {sum(c.values())} pins: "
               + ', '.join(f"{c[k]} {k}" for k in ORDER if c.get(k)))
    caged = sorted((p for p in r['pins'] if p['verdict'] == 'caged'), key=_pad_key)
    if caged:
        out.append('')
        out.append('CAGED: no legal via site is reachable at the net class width')
        for p in caged:
            ru = p['rules']
            out.append(f"  {ref}.{p['pad']:<4} {p['net']:<16} track {ru['track_width']:g}"
                       f" via {ru['via_diameter']:g}  walled by {', '.join(p['walls'])}")
    if r['groups']:
        out.append('')
        out.append(f"CROWDED: via sites run out when all pins fan out at once"
                   f" (estimate on a {r['window']['lane_mm']:g} mm Default lane"
                   f" lattice)")
        nets = {p['pad']: p['net'] for p in r['pins']}
        for i, g in enumerate(r['groups'], 1):
            short = ', '.join(f"{ref}.{n} {nets[n]}" for n in g['short'])
            walls = g['walls'][:WALLS_SHOWN] + (
                [f"{len(g['walls']) - WALLS_SHOWN} more"]
                if len(g['walls']) > WALLS_SHOWN else [])
            out.append(f"  group {i}: {len(g['pins'])} pins, {g['exits']} fit,"
                       f" {len(g['short'])} short: {short}")
            out.append(f"    walled by {', '.join(walls) or 'nothing named'}"
                       + ('; reaches the window edge, try a wider --margin'
                          if g['window_edge'] else ''))
    local = [p for p in r['pins'] if p['verdict'] == 'local']
    if local:
        out.append('')
        out.append('local, left out of the flow: '
                   + ', '.join(f"{ref}.{p['pad']}->{p['reaches']}"
                               for p in sorted(local, key=_pad_key)))
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board')
    ap.add_argument('--component', '-c', required=True, help='reference, e.g. U1')
    ap.add_argument('--layers', nargs='+', default=None,
                    help='allowed layers (default: copper minus inner plane layers)')
    ap.add_argument('--margin', type=float, default=DEFAULT_MARGIN_MM,
                    help='window round the pad ring, mm (default %(default)s)')
    ap.add_argument('--json', action='store_true', help='print JSON')
    a = ap.parse_args(argv)
    try:
        r = check_fanout(a.board, a.component, a.layers, a.margin)
    except Unresolvable as e:
        print(f"check_fanout: {e}", file=sys.stderr)
        return 2
    print(json.dumps(r, indent=1) if a.json else report(r))
    return 1 if r['counts'].get('caged') or r['counts'].get('crowded') else 0


if __name__ == '__main__':
    sys.exit(main())
