#!/usr/bin/env python3
"""The ends model keeps every plane ball of the arrays a way down (awx/whole_ends.py: plane_ways, walled_balls).

  python3 tests/test_622_walled_balls.py

Under the joint fanout the whole route's ends price a state that leaves a plane ball no way down (WALLED): each ball's
ways are its drops and its straps, and each end option kills the ways its copper, its via, its lane past its exit or
-- a pair's -- its pocket stands on. Unmeasured, the zynq DDR's ends ran the bus over every way down three of U1's
VCC_1V5 balls had.

walled_balls, on hand-made ways and kills:
1. a ball whose every drop a chosen option kills is walled; another option leaves it a drop;
2. a ball whose drops are killed but whose strap is not, to a ball with a drop left, is served; the strap's ball
   walled too, it is not -- one strap, not a chain.

plane_ways, on kicad_files/ulx3s.kicad_pcb U1 with GND its plane net (a joint spec naming it), two of U1's signal nets
as the run:
3. LIVENESS: a GND ball with a via in its pad and a gap drop among its ways; else the checks below test nothing;
4. an option whose leg runs on B.Cu under the ball kills its pad's via; one a millimetre off does not;
5. a pair whose two exits by one face on one layer stand either side of the gap drop's site kills it -- the site is
   in the pair's pocket, where its legs close -- though neither leg's copper, via nor lane past its exit reaches it;
   the same pair a millimetre aside does not.
"""
import contextlib
import io
import json
import os
import sys
import tempfile
import types

try:
    import ortools  # noqa: F401  (whole_ends imports the joint escape's solver)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    import whole_ends as we  # noqa: E402
    from kicad_parser import parse_kicad_pcb  # noqa: E402

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


# ---- walled_balls
A, B_, C = (0, 'GND#A1'), (0, 'GND#A2'), (0, 'GND#A3')
pw = types.SimpleNamespace(balls=[(A, [0, 1], []), (B_, [2], [(3, C)]), (C, [4], [])],
                           kill={('L', 0, 0): frozenset({0, 1}), ('L', 0, 1): frozenset({0}),
                                 ('M', 0, 0): frozenset({2, 4}), ('M', 0, 1): frozenset({2})})
lanes = [('L', ('L',)), ('M', ('M',))]
got = we.walled_balls(pw, lanes, {'L': (0, 0), 'M': (0, 0)})
check(got == [A, B_, C], f'every drop killed (and the strap\'s ball\'s too): all three walled ({got})')
got = we.walled_balls(pw, lanes, {'L': (1, 0), 'M': (0, 0)})
check(A not in got, f'another option leaves A1 a drop ({got})')
got = we.walled_balls(pw, lanes, {'L': (0, 0), 'M': (1, 0)})
check(B_ not in got and C not in got, f'A2\'s drop killed, its strap to A3 with a drop left: both served ({got})')

# ---- plane_ways
if not os.path.exists(BOARD):
    print(f'SKIP (plane_ways): board not present: {BOARD}')
    sys.exit(1 if BAD else 0)
with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    pcb = parse_kicad_pcb(BOARD)
foot = pcb.footprints['U1']
short = lambda n: (n or '').split('/')[-1]
# (GND's plane under U1: the board pours none, and a ball is never dropped to a plane that is not there)
_xs, _ys = [p.global_x for p in foot.pads], [p.global_y for p in foot.pads]
_gnd = next(i for i, n in pcb.nets.items() if short(n.name) == 'GND')
pcb.zones = list(pcb.zones or []) + [types.SimpleNamespace(
    net_id=_gnd, layer='In1.Cu', priority=0, polygon=[(min(_xs) - 5, min(_ys) - 5), (max(_xs) + 5, min(_ys) - 5),
                                                      (max(_xs) + 5, max(_ys) + 5), (min(_xs) - 5, max(_ys) + 5)])]
sigs = sorted({short(p.net_name) for p in foot.pads if p.net_id and short(p.net_name) not in ('GND', '')
               and not short(p.net_name).startswith(('+', 'VCC', 'Net-'))})[:2]
byname = {short(n.name): (i, n) for i, n in pcb.nets.items() if n.name}
pad_of = {short(p.net_name): p for p in foot.pads if p.net_id}
spec = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
json.dump({'layers': ['F.Cu', 'B.Cu'], 'arrays': [{'ref': 'U1', 'others': [], 'drops': ['GND']}]}, spec)
spec.close()
os.environ['FANOUT_JOINT'] = spec.name
st = dict(pcb=pcb, byname=byname, src_pad={n: pad_of[n] for n in sigs}, dst_pad={}, sref='U1', dref='NONE')


def mv(legs, exit_pt, direction, layer):
    return types.SimpleNamespace(legs=legs, site=None, exit_pt=exit_pt, direction=direction, layer=layer,
                                 kind='surface')


def ways_of(T, lanes_):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return we.plane_ways(st, lanes_, T, {ln: [] for ln, _lg in lanes_}, {n: None for n in sigs})


# 3. liveness: a GND ball with an in-pad way and a gap way, on the board with no run copper of ours
probe = ways_of({'S': []}, [('S', (sigs[0],))])
ball = inpad = gap = None
for b, dw, _sw in probe.balls if probe else ():
    ip = [w for w in dw if probe.ways[w].get('stub') is None]
    gp = [w for w in dw if probe.ways[w].get('stub') is not None]
    if ip and gp:
        ball, inpad, gap = b, probe.ways[ip[0]], probe.ways[gp[0]]
        break
if len(sigs) < 2 or ball is None:
    print(f'BROKEN TEST: no two signal nets on U1 ({sigs}) or no GND ball with a pad via and a gap drop')
    sys.exit(2)
c, s = inpad['site'], gap['site']
check(True, f'liveness: {ball[1]} at {tuple(round(q, 2) for q in c)}, its gap drop at {tuple(round(q, 2) for q in s)}')

# 4. a B.Cu leg under the ball, and one a millimetre off
under = mv([((c[0] - 2.0, c[1]), (c[0] + 2.0, c[1]), 'B.Cu')], (c[0] - 2.0, c[1]), 'left', 'B.Cu')
off = mv([((c[0] - 2.0, c[1] + 1.0), (c[0] + 2.0, c[1] + 1.0), 'B.Cu')], (c[0] - 2.0, c[1] + 1.0), 'left', 'B.Cu')
T = {'S': [((under,), under.exit_pt, 'B.Cu', 0, False), ((off,), off.exit_pt, 'B.Cu', 0, False)]}
pwb = ways_of(T, [('S', (sigs[0],))])
w_in = next(i for i, w in enumerate(pwb.ways) if w.get('stub') is None and w['site'] == c)
check(w_in in pwb.kill.get(('S', 0, 0), ()), 'a leg on B.Cu under the ball kills its pad\'s via')
check(w_in not in pwb.kill.get(('S', 0, 1), ()), 'one a millimetre off does not')

# 5. a pair's exits either side of the gap site (0.9 mm apart, leaving by +x on F.Cu, their legs behind the exits)
def pair(dy):
    p_ = mv([((s[0] - 1.2, s[1] - 0.45 + dy), (s[0] - 0.3, s[1] - 0.45 + dy), 'F.Cu')],
            (s[0] - 0.3, s[1] - 0.45 + dy), 'right', 'F.Cu')
    n_ = mv([((s[0] - 1.2, s[1] + 0.45 + dy), (s[0] - 0.3, s[1] + 0.45 + dy), 'F.Cu')],
            (s[0] - 0.3, s[1] + 0.45 + dy), 'right', 'F.Cu')
    return ((p_, n_), s, 'F.Cu', 0, False)


T = {'P': [pair(0.0), pair(1.0)]}
pwp = ways_of(T, [('P', tuple(sigs))])
w_gap = next(i for i, w in enumerate(pwp.ways) if w.get('stub') is not None and 'site' in w and w['site'] == s)
# (neither leg's own copper reaches it: the same two legs as two singles kill nothing there)
Ts = {'A': [((pair(0.0)[0][0],), s, 'F.Cu', 0, False)], 'B': [((pair(0.0)[0][1],), s, 'F.Cu', 0, False)]}
pws = ways_of(Ts, [('A', (sigs[0],)), ('B', (sigs[1],))])
w_gs = next(i for i, w in enumerate(pws.ways) if w.get('stub') is not None and 'site' in w and w['site'] == s)
single = w_gs in pws.kill.get(('A', 0, 0), ()) or w_gs in pws.kill.get(('B', 0, 0), ())
check(not single, 'control: neither leg alone -- its copper, its lane past its exit -- kills the gap drop')
check(w_gap in pwp.kill.get(('P', 0, 0), ()), 'the pair whose exits stand either side of it kills it: its pocket')
check(w_gap not in pwp.kill.get(('P', 0, 1), ()), 'the same pair a millimetre aside does not')
os.unlink(spec.name)

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
