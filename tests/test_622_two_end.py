#!/usr/bin/env python3
"""One source, several ends (route_bus --bus SRC:U4+U3, the destination GROUP: whole_frame.Group).

  python3 tests/test_622_two_end.py

On boards stood in for, this pins:

1. flow_frame.dest_refs: DEST names one part or several, the trunk's first; quarter_of reads every named part's pads
   as the destination and turns about the first's centre;
2. whole_frame.Group: the parts' copper boxes joined into one outline by the narrowest strips that join them (the
   longest gap of the tree of nearest gaps, and a lane pitch), its front the hull's face toward the source;
3. a lane's ways in: on the outline; out of either mouth of a pocket, or straight under the part in front of it where
   that part leaves the berth's layer open; under its own part from its far face, with a via;
4. Group.arrange: the ways and the cut that cross the fewest -- the best of every valid split of each pocket's lanes
   (in their order along it), Group.score's every-cut count the count cut by cut, the cut off the front;
5. braid.net_dest: a net's end at the part it reaches, the first named when it reaches none;
6. route_buses: --bus U1:U4+U3 parses, and a bus handed on through several parts records each as gone through -- a
   later escape bus on U3 plans it whole (ESCAPE_JOINT).
"""
import os
import shutil
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import flow_frame  # noqa: E402
import whole_frame as wf  # noqa: E402
import whole_ends  # noqa: E402
import braid as bd  # noqa: E402
import route_bus as rb  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


def pad(ref, x, y, nid=0):
    return types.SimpleNamespace(component_ref=ref, global_x=x, global_y=y, net_id=nid)


def part(ref, cx, cy, half=1.0, nid_pads=()):
    """a part's pads: a 3 x 3 grid `half` either side of (cx, cy), and the given (x, y, net id) pads"""
    ps = [pad(ref, cx + dx * half, cy + dy * half) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    ps += [pad(ref, x, y, nid) for x, y, nid in nid_pads]
    return types.SimpleNamespace(pads=ps, x=cx, y=cy)


# 1. dest_refs, quarter_of
check(flow_frame.dest_refs('U2') == ['U2'] and flow_frame.dest_refs('U4+U3') == ['U4', 'U3'],
      f'1: dest_refs: {flow_frame.dest_refs("U2")}, {flow_frame.dest_refs("U4+U3")}')
# U1 at the origin; U4 due east, U3 north-east (north is -y): the run's nets N1 to U4, N2 to U3
fps = {'U1': part('U1', 0.0, 0.0, nid_pads=[(1.0, 0.0, 1), (1.0, -1.0, 2)]),
       'U4': part('U4', 20.0, 0.0, nid_pads=[(19.0, 0.0, 1)]),
       'U3': part('U3', 19.0, -12.0, nid_pads=[(18.0, -11.0, 2)]),
       'U7': part('U7', 19.0, 10.0)}
nets = {1: types.SimpleNamespace(name='/N1', pads=[p for f in fps.values() for p in f.pads if p.net_id == 1]),
        2: types.SimpleNamespace(name='/N2', pads=[p for f in fps.values() for p in f.pads if p.net_id == 2])}
pcb = types.SimpleNamespace(nets=nets, footprints=fps)
k, cx, cy = flow_frame.quarter_of(pcb, 'U4+U3', {'N1', 'N2'})
check((k, cx, cy) == (0, 20.0, 0.0), f'1: quarter_of U4+U3: no turn, about U4 -- {(k, cx, cy)}')
k1, _x, _y = flow_frame.quarter_of(pcb, 'U4', {'N1', 'N2'})
check(k1 == 0, f'1: quarter_of U4 alone: {k1}')

# 2-4. the GROUP (whole_frame.Group): one source at the origin, its destination three parts as one -- a connector J
# standing across the trunk, and two parts P (south) and Q (north) in front of it, P 1 mm from J (a strip that closes:
# a POCKET), Q 2 mm from it, the two 6 mm apart (a NOTCH). J's pads on B.Cu, P's and Q's on F.Cu


def smd(ref, x, y, layer, sx=0.3, sy=0.3):
    return types.SimpleNamespace(component_ref=ref, global_x=x, global_y=y, size_x=sx, size_y=sy, pad_type='smd',
                                 drill=0, layers=[layer], net_id=0)


gfps = {'S': types.SimpleNamespace(pads=[smd('S', x, y, 'F.Cu') for x in (-2.0, 2.0) for y in (-2.0, 2.0)]),
        'J': types.SimpleNamespace(pads=[smd('J', 22.0, y / 2.0, 'B.Cu') for y in range(-12, 13)]),
        'P': types.SimpleNamespace(pads=[smd('P', x, y, 'F.Cu') for x in (17.0, 20.0) for y in (2.0, 5.0)]),
        'Q': types.SimpleNamespace(pads=[smd('Q', x, y, 'F.Cu') for x in (17.0, 20.0) for y in (-5.0, -2.0)])}
gpcb = types.SimpleNamespace(footprints=gfps)
G = wf.Group(gpcb, ['J', 'P', 'Q'], (0.0, 0.0))
# (J's copper from x 21.85; P's and Q's to 20.15: their strips 1.7 mm wide, the gap between P and Q 3.7 mm)
check(abs(G.bridge - (1.7 + bd.LPITCH)) < 1e-6, f'2: the bridge -- the longest of the gaps joining the parts, and a lane '
      f'pitch: {G.bridge}')
check(G.U.geom_type == 'Polygon', '2: one outline')
f0, f1 = G.pt(G.f0), G.pt(G.f1)
check(abs(f0[0] - 16.85) < 1e-6 and abs(f1[0] - 16.85) < 1e-6 and f0[1] < f1[1],
      f'2: the front -- the hull face toward the source, north to south: {f0} .. {f1}')
# 3. the ways in: a berth on J's face in the notch is on the outline; one behind P is in a pocket -- out of either of
# its mouths, or straight under P (P on F.Cu alone: B.Cu, the berth's layer, open under it)
w_notch = G.options((21.6, 0.0), (-1.0, 0.0), 'J', 'B.Cu', False)
check([w[0] for w in w_notch] == ['on'] and G.in_front(w_notch[0][1]), f'3: a notch berth on the outline, on the front: {w_notch}')
w_pk = G.options((21.6, 3.5), (-1.0, 0.0), 'J', 'B.Cu', False)
check(sorted(w[0] for w in w_pk) == ['mouth', 'mouth', 'straight'], f'3: a pocket berth -- two mouths and straight under: {w_pk}')
check(not any(w[0] == 'straight' for w in G.options((21.6, 3.5), (-1.0, 0.0), 'J', 'F.Cu', False)),
      '3: no way straight under P on F.Cu, its own layer, without a via')
w_back = G.options((22.5, 0.0), (1.0, 0.0), 'J', 'B.Cu', True)
check(sorted(w[0] for w in w_back) == ['on', 'under'], f'3: a far-face berth with a via -- on, or under J: {w_back}')
check(not any(w[0] == 'under' for w in G.options((22.5, 0.0), (1.0, 0.0), 'J', 'B.Cu', False)),
      '3: no way under J on B.Cu, its own layer, without a via')
# 4. the arrangement: every lane's ways and the cut, the fewest crossings -- equal to the best of every valid split
# (each pocket's lanes in their order along it), and Group.score's crossings at every cut one by one
import itertools  # noqa: E402
berths = {'a': ((21.6, 2.6), 'J'), 'b': ((21.6, 3.5), 'J'), 'c': ((21.6, 4.4), 'J'), 'd': ((21.6, 0.0), 'J'),
          'e': ((16.6, 3.5), 'P'), 'f': ((16.6, -3.5), 'Q'), 'g': ((22.5, 1.0), 'J')}
opts = {n: G.options(p, (-1.0, 0.0) if p[0] < 22 else (1.0, 0.0), r, 'B.Cu' if r == 'J' else 'F.Cu', False)
        for n, (p, r) in berths.items()}
launch = ['e', 'a', 'c', 'f', 'b', 'g', 'd']
lkey = {n: i for i, n in enumerate(launch)}
ch, c_, x_ = G.arrange(opts, lkey)
lanes = sorted(opts, key=lkey.get)
rk = {n: i for i, n in enumerate(lanes)}


def direct(choice):
    pl = {n: opts[n][choice[n]][1] for n in lanes}
    best = None
    for _i, cut in G.cuts(list(pl.values())):
        un = {n: (G.unroll(pl[n], cut), rk[n]) for n in lanes}
        x = sum(1 for a_, b_ in itertools.combinations(lanes, 2) if (rk[a_] < rk[b_]) != (un[a_] < un[b_]))
        best = x if best is None else min(best, x)
    return best


bf = min(direct({k: v for alt in combo for k, v in alt.items()} | {n: 0 for n in lanes if not any(n in alt for alt in combo)})
         for combo in itertools.product(*G.units(opts, lanes)))
check(x_ == bf, f'4: the arrangement\'s crossings {x_} and the best valid split\'s {bf}')
check(G.score(opts, rk, ch)[0] == direct(ch), f'4: Group.score at every cut at once and one by one: '
      f'{G.score(opts, rk, ch)[0]} vs {direct(ch)}')
check(not G.in_front(c_), '4: the cut off the front')

# 5. net_dest
n3 = types.SimpleNamespace(pads=[pad('U1', 0, 0), pad('U3', 1, 1)])
n0 = types.SimpleNamespace(pads=[pad('U1', 0, 0), pad('R1', 1, 1)])
check(bd.net_dest(n3, 'U4+U3') == 'U3' and bd.net_dest(n0, 'U4+U3') == 'U4' and bd.net_dest(n3, 'U3') == 'U3',
      '5: net_dest')

# 6. route_buses, the step stood in for
check(rb.parse_bus('U1:U4+U3:plain:F.Cu,B.Cu') == ('U1', 'U4+U3', 'plain', 'F.Cu,B.Cu'), '6: parse_bus U4+U3')
calls = []


def step(board, out, src, dest, *a, joint_fanout=False, escape=False, settings=None, **kw):
    calls.append(dict(src=src, dest=dest, escape=escape, settings=dict(settings or {})))
    shutil.copy(board, out)
    shutil.copy(board[:-len('.kicad_pcb')] + '.kicad_pro', out[:-len('.kicad_pcb')] + '.kicad_pro')
    return 0, f'BUS {src}->{dest}'


real = rb.route_bus
rb.route_bus = step
try:
    with tempfile.TemporaryDirectory() as td:
        b0 = os.path.join(td, 'b.kicad_pcb')
        open(b0, 'w').write('(kicad_pcb)\n')
        open(os.path.join(td, 'b.kicad_pro'), 'w').write('{}\n')
        rb.route_buses(b0, os.path.join(td, 'o.kicad_pcb'), [('U1', 'U4+U3', 'plain', None), ('U3', 'U9', 'escape', None)],
                       log=lambda *a: None)
finally:
    rb.route_bus = real
check([(c_['src'], c_['dest']) for c_ in calls] == [('U1', 'U4+U3'), ('U3', 'U9')], f'6: the buses: {calls}')
check(calls[1]['settings'].get('ESCAPE_JOINT') == 'U3' and calls[1]['settings'].get('ESCAPE_JOINT_ENDS') == b0,
      f'6: U3, gone through by U1:U4+U3, planned whole by the escape bus on it: {calls[1]["settings"]}')

print('two_end: ' + ('FAIL' if fails else 'PASS: DEST names several parts, one GROUP: its outline, front, ways in '
                     'and arrangement'))
sys.exit(1 if fails else 0)
