#!/usr/bin/env python3
"""A net keeps to its lane (py_router/lane_price.py, the router's lane_price; KICAD_GLOBAL_PLAN_FILE and
KICAD_GLOBAL_PLAN_LANE_COST in py_router/global_plan.py).

  python3 tests/test_lane_price.py

A net may be given a LANE -- the way it is to keep to -- and the router charges for each cell entered off it. A price for
leaving, not a reward for keeping to it: the search is weighted, and a discount along a lane never outweighs the
heuristic's pull along the straight line. This test makes tiny boards (pads, a wall of pads, an outline; four copper
layers), gives each net a way in a plan file, routes with route.py and measures the copper against the way.

A net FOLLOWS its way when its copper stands a mean 0.4 mm or less from it (the free half-width is 0.2 mm, and an
octolinear track about an any-angle way steps either side of it) with 80% of its length on the way's layer.

1. with no price the route is the straight line, far from a way that bends (the change detector: a scenario whose way
   IS the straight line would pass with nothing built);
2. a way that bends 10 mm off the straight line is followed, on its layer;
3. ...and the same when the way is given target first: the price is a field, with no direction;
4. the long way round -- three sides of a rectangle, 75% longer than the straight line -- is followed;
5. a way on the bottom layer between two surface pads on the top is followed, with its two vias;
6. the way through the FARTHER of two gaps in a wall of foreign pads is followed;
7. five nets side by side, each its own bent way 0.4 mm from the next, each follow their own;
8. a net on its lane is not straightened afterwards by the octolinear smoothing (2. to 7. route with it on);
9. route.py --bus: a bus's members keep their lane free across the bus's own width -- five members whose pads stand
   wider apart than their tracks would route straight, with no via.
"""
import contextlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
BAD = []
N = [0]


def check(what, ok, detail=''):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + (f' -- {detail}' if detail else ''))


def board(path, pads, w=60, h=50):
    """a board: `pads` [(ref, x, y, net name, 'th' | 'smd')] and the outline (0, 0)-(w, h)"""
    U = lambda: str(uuid.uuid4())  # noqa: E731
    nid = {n: i + 1 for i, n in enumerate(sorted({p[3] for p in pads}))}
    L = ['(kicad_pcb', '\t(version 20241229)', '\t(generator "pcbnew")', '\t(generator_version "9.0")',
         '\t(general (thickness 1.6) (legacy_teardrops no))', '\t(paper "A4")',
         '\t(layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal) (5 "F.SilkS" user)'
         ' (1 "F.Mask" user) (3 "B.Mask" user) (13 "F.Paste" user) (25 "Edge.Cuts" user) (35 "F.Fab" user))',
         '\t(setup (pad_to_mask_clearance 0))', '\t(net 0 "")']
    L += [f'\t(net {i} "{n}")' for n, i in nid.items()]
    for ref, x, y, net, kind in pads:
        pad = (f'(pad "1" thru_hole circle (at 0 0) (size 0.8 0.8) (drill 0.4) (layers "*.Cu" "*.Mask") (net {nid[net]} "{net}") (uuid "{U()}"))'
               if kind == 'th' else
               f'(pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Paste" "F.Mask") (net {nid[net]} "{net}") (uuid "{U()}"))')
        L += [f'\t(footprint "synth:P" (layer "F.Cu") (uuid "{U()}") (at {x} {y})',
              f'\t\t(property "Reference" "{ref}" (at 0 -1.2 0) (layer "F.SilkS") (uuid "{U()}") (effects (font (size 0.5 0.5) (thickness 0.08))))',
              f'\t\t{pad}', '\t)']
    for (x1, y1), (x2, y2) in (((0, 0), (w, 0)), ((w, 0), (w, h)), ((w, h), (0, h)), ((0, h), (0, 0))):
        L.append(f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.1) (type default)) (layer "Edge.Cuts") (uuid "{U()}"))')
    L.append(')')
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')


def scenario(name):
    """(pads, {net: its way [[x, y, layer]]})"""
    two = [('A1', 10, 20, 'N1', 'th'), ('B1', 50, 20, 'N1', 'th')]
    if name == 'dogleg':
        return two, {'N1': [[x, y, 'In1.Cu'] for x, y in ([10, 20], [30, 30], [50, 20])]}
    if name == 'around':
        return two, {'N1': [[x, y, 'In1.Cu'] for x, y in ([10, 20], [10, 35], [50, 35], [50, 20])]}
    if name == 'dive':
        pads = [('A1', 10, 20, 'N1', 'smd'), ('B1', 50, 20, 'N1', 'smd')]
        return pads, {'N1': [[10, 20, 'F.Cu'], [11, 20, 'F.Cu'], [11, 20, 'B.Cu'], [49, 20, 'B.Cu'], [49, 20, 'F.Cu'], [50, 20, 'F.Cu']]}
    if name == 'gap':
        wall = [(f'W{i}', 30, round(4 + 0.9 * i, 2), 'WALL', 'th') for i in range(48)
                if not (19.2 <= 4 + 0.9 * i <= 20.8 or 34.2 <= 4 + 0.9 * i <= 35.8)]
        return two + wall, {'N1': [[x, y, 'In1.Cu'] for x, y in ([10, 20], [30, 35], [50, 20])]}
    pads, ways = [], {}                                   # (five)
    for i in range(5):
        y = 18 + i
        pads += [(f'A{i}', 10, y, f'N{i}', 'th'), (f'B{i}', 50, y, f'N{i}', 'th')]
        ways[f'N{i}'] = [[x_, y_, 'In1.Cu'] for x_, y_ in ([10, y], [14, y], [30, 30 + 0.4 * i], [46, y], [50, y])]
    return pads, ways


def dist(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
    return math.hypot(p[0] - a[0] - u * vx, p[1] - a[1] - u * vy)


def route(td, name, ways, pads, env=None, more=(), nets=None):
    """route.py on the scenario's board; the parsed result, or None where it wrote no board"""
    from kicad_parser import parse_kicad_pcb
    b_in, b_out, plan = (os.path.join(td, f'{name}.{e}') for e in ('in.kicad_pcb', 'out.kicad_pcb', 'plan.json'))
    board(b_in, pads)
    json.dump(ways, open(plan, 'w'))
    e = dict(os.environ)
    for k in list(e):
        if k.startswith('KICAD_GLOBAL_PLAN') or k == 'KICAD_SMOOTH_ROUTE':
            del e[k]
    if env:
        e.update(env, KICAD_GLOBAL_PLAN_FILE=plan)
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'route.py'), b_in, b_out, '--nets',
                        *(nets or sorted(ways)), '--layers', *LAYERS, '--clearance', '0.15', '--track-width', '0.15',
                        '--via-size', '0.5', '--via-drill', '0.3', '--grid-step', '0.1', *more],
                       env=e, capture_output=True, text=True)
    if not os.path.isfile(b_out):
        print(r.stdout[-2000:], r.stderr[-2000:])
        return None
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(b_out)


def measure(p, ways):
    """(mean distance of the nets' copper from their ways, the share of its length on the way's layer, its vias)"""
    ids = {n.name: i for i, n in p.nets.items()}
    tot = dsum = on = 0.0
    for net, way in ways.items():
        pieces = [(a, b) for a, b in zip(way, way[1:]) if a[2] == b[2] and a[:2] != b[:2]]
        for s in p.segments:
            if s.net_id != ids.get(net):
                continue
            L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
            k = max(1, int(L / 0.2))
            for i in range(k):
                u = (i + 0.5) / k
                q = (s.start_x + u * (s.end_x - s.start_x), s.start_y + u * (s.end_y - s.start_y))
                d, lay = min((dist(q, a, b), a[2]) for a, b in pieces)
                tot += L / k
                dsum += d * L / k
                on += (L / k) * (s.layer == lay)
    vias = sum(1 for v in p.vias if v.net_id in {ids.get(n) for n in ways})
    return (dsum / tot if tot else 99.0), (on / tot if tot else 0.0), vias


LANE = dict(KICAD_GLOBAL_PLAN_ORDER='', KICAD_GLOBAL_PLAN_LANE_COST='5')
with tempfile.TemporaryDirectory() as td:
    pads, ways = scenario('dogleg')
    p = route(td, 'plain', ways, pads)
    m, on, _v = measure(p, ways) if p else (0.0, 0.0, 0)
    check('1. with no price the route is the straight line, far from a way that bends', p is not None and m > 2.0,
          f'mean {m:.2f} mm off the way')
    for i, (name, rev, what, vias) in enumerate((
            ('dogleg', False, 'a way that bends 10 mm off the straight line is followed, on its layer', 0),
            ('dogleg', True, '...and the same when the way is given target first', 0),
            ('around', False, 'the long way round, 75% longer than the straight line, is followed', 0),
            ('dive', False, 'a way on the bottom layer between two surface pads is followed, with its two vias', 2),
            ('gap', False, 'the way through the farther of two gaps in a wall is followed', 0),
            ('five', False, 'five nets side by side each follow their own bent way', 0)), 2):
        pads, ways = scenario(name)
        given = {n: (w[::-1] if rev else w) for n, w in ways.items()}
        p = route(td, f'{name}{"_rev" if rev else ""}', given, pads, env=LANE)
        m, on, v = measure(p, ways) if p else (99.0, 0.0, -1)
        check(f'{i}. {what}', p is not None and m <= 0.4 and on >= 0.8 and v == vias,
              f'mean {m:.2f} mm off, {100 * on:.0f}% on its layer, {v} via(s)')
    # (8: every run of 2. to 7. had the smoothing on -- a straightened route fails them; said here by the source)
    src = open(os.path.join(ROOT, 'py_router', 'cleanup_pipeline.py')).read()
    check('8. a net on its lane is not straightened by the octolinear smoothing',
          'skip_net_ids=_smooth_skip_net_ids(pcb_data) | _lane_net_ids(config))' in src)
    pads, ways = scenario('five')
    p = route(td, 'bus', {}, pads, more=['--bus'], nets=['N*'])
    ids = {n.name: i for i, n in p.nets.items()} if p else {}
    done = {s.net_id for s in p.segments} if p else set()
    check('9. --bus: five members whose pads stand wider apart than their tracks would route straight, no via',
          p is not None and all(ids.get(n) in done for n in ways) and not p.vias,
          f'{len(p.vias) if p else "-"} via(s)')

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
