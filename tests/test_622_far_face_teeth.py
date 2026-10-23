#!/usr/bin/env python3
"""A tooth on the SOURCE's far face -- the face turned away from the destination (whole_frame.tooth_face / src_cut /
lead_ins, whole_ends.FAR_TOOTH).

  python3 tests/test_622_far_face_teeth.py

A pinout can put a bus's balls beside the source's far face, where the only way out is that face (sdi_mipi_conv's SDI
bus on U11). The whole route takes such a lane round the source's corner on a LEAD-IN to a start beside the side face,
and the ends model prices the tooth past any via count, so it is chosen only by a lane with no other way out. This
pins, on a 10 x 10 source box (y down: north is y = 0):

1. a tooth's face is the one its escape leaves by, where the escape runs along an axis -- an exit past the north face
   beside the far corner is a north-face tooth, though the far face is nearer -- and the nearest face otherwise;
2. the far face's teeth are unrolled at its CUT (perim_s): a far-face tooth north of the cut goes round the north,
   first in the launch order, outside every lane of the north face; south of it round the south, last;
3. the cut is the gap between the far-face teeth that crosses the fewest lanes, the widest on a tie: two far-face
   teeth whose berths both lie north are both sent round the north, where the middle of the face split them;
4. the lead-ins nest: on each side the tooth nearest the corner innermost, every lane a lane's room apart along the
   far face and along the side face, each ending outside the box beside its side face -- and they go round copper in
   their way (a part's pad beside the far face, on a lead-in's layer: out past it, its bar clear; on another layer it
   is passed over);
5. END TO END (about 15 s): synth_bus's wind_sorted_e2 routed REVERSED -- its two far-face lanes now at the SOURCE --
   on three routing layers lays every lane, connected and DRC-clean, in one round. Its bench must carry far-face teeth
   (a tooth escaping away from the destination), or it exercises nothing (BROKEN TEST);
6. END TO END, the ESCAPE fanout (about 20 s): the same bus through route_bus --escape, as sdi_mipi_conv's SDI bus is
   routed -- both arrays dog-boned whole, every bus ball an outer ball left bare at both (braid.bare_lane: the ladder
   had held none of them) -- lays every lane, connected and DRC-clean, in one round, its far-face teeth among them;
7. END TO END, a cap beside the far face where the lead-ins run (synth_bus --part c0402@de: on the side they go round):
   every lane laid, connected and DRC-clean, in one round (a lead-in laid through it had left a lane open).
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import whole_frame as wf  # noqa: E402
import braid as bd  # noqa: E402

SB = (0.0, 0.0, 10.0, 10.0)
fails = []


def check(ok, what):
    print(('PASS' if ok else 'FAIL') + ': ' + what)
    if not ok:
        fails.append(what)


# 1. the face a tooth leaves by
check(wf.tooth_face((0.4, -0.6), SB, (0.0, -1.0)) == 'N',
      'an exit past the north face beside the far corner, escaping north, is a north-face tooth')
check(wf.face((0.4, -0.6), SB) == 'W', '...where the nearest face is the far one (the reading it replaces)')
check(wf.tooth_face((-0.6, 5.0), SB, (-1.0, 0.0)) == 'W', 'an exit escaping west is a far-face tooth')
check(wf.tooth_face((8.0, 3.0), SB, (0.7, 0.7)) == wf.face((8.0, 3.0), SB),
      'a diagonal escape (a dog-bone stub) reads the nearest face')
check(wf.tooth_face((8.0, 3.0), SB, None) == wf.face((8.0, 3.0), SB), 'no escape: the nearest face')

# 2. the far face unrolled at its cut
T = {'W_n': (-0.6, 3.0), 'W_s': (-0.6, 7.0), 'N': (5.0, -0.6), 'E': (10.6, 5.0), 'S': (5.0, 10.6)}
for c, want in ((5.0, ['W_n', 'N', 'E', 'S', 'W_s']), (8.0, ['W_s', 'W_n', 'N', 'E', 'S']),
                (2.0, ['N', 'E', 'S', 'W_s', 'W_n'])):
    got = sorted(T, key=lambda k: wf.perim_s(T[k], SB, c))
    check(got == want, f'the far face cut at y {c}: launch order {got}')
check(all(abs(wf.perim_s(p, SB, None) - wf.perim_s(p, SB, 5.0)) < 1e-12 for p in T.values()),
      'no cut: the far face\'s middle, as perim_s always unrolled it')

# 3. the cut by crossings
ys = [3.0, 7.0]
order_n = ['W_s', 'W_n', 'N', 'E', 'S']            # both far-face berths north: the far lanes outermost north


def rank(c_):
    k_ = sorted(T, key=lambda k: wf.perim_s(T[k], SB, c_))
    fi = {n: i for i, n in enumerate(order_n)}
    return sum(1 for i, a in enumerate(k_) for b in k_[i + 1:] if fi[a] > fi[b])
c3 = wf.src_cut(ys, SB[1], SB[3], rank=rank)
check(c3 > 7.0, f'two far teeth bound north: cut south of both (y {c3:.2f}), not the middle')
check(rank(c3) == 0, f'...and the launch order meets the berths\' with no crossing ({rank(c3)})')
check(abs(wf.src_cut(ys, SB[1], SB[3]) - 5.0) < 1e-9,
      f'no rank: the widest gap\'s middle, 3..7 ({wf.src_cut(ys, SB[1], SB[3])})')

# 4. the lead-ins
far = ['a', 'b', 'c']
F = SimpleNamespace(tooth={'a': (-0.6, 2.0), 'b': (-0.6, 4.0), 'c': (-0.6, 8.0), 'n1': (3.0, -0.6),
                           's1': (3.0, 10.6), 'e1': (10.6, 5.0)}, M=['a', 'b', 'c', 'n1', 's1', 'e1'])
L = wf.lead_ins(F, far, 6.0, lambda n: bd.LANE_MIN, SB)
check(set(L) == set(far), f'a lead-in for each far-face lane: {sorted(L)}')
xa, xb = L['a'][1][0], L['b'][1][0]
check(abs((xa - xb) - bd.LANE_MIN) < 1e-9 and xa < -0.6 - bd.LANE_MIN + 1e-9,
      f'north side: a (nearest the corner) innermost, b a lane apart outside it, a lane out of the teeth: {xa:.3f} '
      f'{xb:.3f}')
ya, yb = L['a'][-1][1], L['b'][-1][1]
check(yb < ya < -0.6 - bd.LANE_MIN + 1e-9 and abs((ya - yb) - bd.LANE_MIN) < 1e-9,
      f'...each starting beside the north face a lane apart, outside its teeth: y {ya:.3f}, {yb:.3f}')
check(L['c'][-1][1] > 10.6 + bd.LANE_MIN - 1e-9, f'c, south of the cut, starts beside the south face: {L["c"][-1]}')


def chamfer_gap(p, q):
    """the perpendicular distance between two 45-degree chamfers (each its last two points)"""
    a0, a1 = p[-2], p[-1]
    b0 = q[-2]
    k = (a1[0] - a0[0], a1[1] - a0[1])
    L_ = math.hypot(*k)
    return abs((b0[0] - a0[0]) * k[1] - (b0[1] - a0[1]) * k[0]) / L_
check(abs(chamfer_gap(L['a'], L['b']) - bd.LANE_MIN) < 1e-6,
      f'the corner chamfers concentric, a lane apart: {chamfer_gap(L["a"], L["b"]):.4f}')
check(all(L[n][0] == F.tooth[n] for n in far), 'each lead-in starts at its tooth')
# ...round copper in their way: a pad beside the far face where the north lead-ins run along it
bar_ = bd.CLEAR + bd.TRACK / 2 + wf.HALF_G
pad_ = (xb - 0.15, 0.5, xb + 0.15, 1.2)             # (on b's run along the far face, between its tooth and the corner)
Lp = wf.lead_ins(F, far, 6.0, lambda n: bd.LANE_MIN, SB, obst=[(pad_, {'F.Cu'})], layer=lambda n: 'F.Cu')
gap_ = min(wf._seg_box(p, q, pad_) for n in ('a', 'b') for p, q in zip(Lp[n][1:], Lp[n][2:]))
check(gap_ >= bar_ - 1e-9 and Lp['b'][1][0] < pad_[0], f'a pad on the lead-ins\' run: they pass outside it, '
      f'{gap_:.3f} off it (bar {bar_:.3f}), b at x {Lp["b"][1][0]:.3f}')
check(abs((Lp['a'][1][0] - Lp['b'][1][0]) - bd.LANE_MIN) < 1e-9, '...still nested a lane apart')
Lo = wf.lead_ins(F, far, 6.0, lambda n: bd.LANE_MIN, SB, obst=[(pad_, {'B.Cu'})], layer=lambda n: 'F.Cu')
check(Lo == L, 'a pad on another layer: passed over, the lead-ins as without it')


# 5. end to end
def end_to_end():
    try:
        import ortools  # noqa: F401
    except ImportError as exc:
        print(f'SKIP (5): needs ortools ({exc})')
        return
    import synth_bus as sb
    layers = ','.join(sb.layer_names(3))
    e = {k_: v_ for k_, v_ in os.environ.items()}
    e.update(ROUTE_LAYERS=layers)
    with tempfile.TemporaryDirectory() as td:
        raw, bench, turned = (os.path.join(td, f) for f in ('raw.kicad_pcb', 'bench.kicad_pcb', 'turned.kicad_pcb'))
        # (the source SD1, the array whose far column the --ring-e lanes leave by)
        for cmd in (['synth_bus.py', raw, '--k', '12', '--copper', '4', '--dst-cols', '8', '--closed', '--ring-n', '3',
                     '--ring-s', '3', '--ring-e', '2'], ['make_bench.py', raw, 'SD1', 'SU1', bench]):
            r = subprocess.run([sys.executable] + cmd, cwd=AWX, capture_output=True, text=True, timeout=600, env=e)
            if r.returncode != 0:
                raise SystemExit(f'BROKEN TEST: {cmd[0]} failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
        from kicad_parser import parse_kicad_pcb
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(bench)
        names = sorted({p.net_name.split('/')[-1] for p in pcb.footprints['SU1'].pads if p.net_id}
                       & {p.net_name.split('/')[-1] for p in pcb.footprints['SD1'].pads if p.net_id})
        # (the chain runs in the flow frame: source to destination along +x)
        r = subprocess.run([sys.executable, 'flow_frame.py', 'quarter', bench, 'SU1', ','.join(names)], cwd=AWX,
                           capture_output=True, text=True, timeout=300, env=e)
        k_, cx, cy = r.stdout.split()[-3:]
        r = subprocess.run([sys.executable, 'flow_frame.py', 'turn', bench, turned, k_, cx, cy], cwd=AWX,
                           capture_output=True, text=True, timeout=300, env=e)
        if r.returncode != 0:
            raise SystemExit(f'BROKEN TEST: flow_frame turn failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
        import shutil
        for x in ('kicad_pro', 'ladder.txt'):
            if os.path.isfile(bench[:-len('kicad_pcb')] + x):
                shutil.copy(bench[:-len('kicad_pcb')] + x, turned[:-len('kicad_pcb')] + x)
        run = os.path.join(td, 'run')
        r = subprocess.run([sys.executable, 'whole_route.py', '12', run, '1'], cwd=AWX, capture_output=True, text=True,
                           timeout=1800, env=dict(e, BASE=turned, DEST='SU1'))
        side = os.path.join(run, 'r1', 'fo.plan.json')
        if not os.path.isfile(side):
            raise SystemExit(f'BROKEN TEST: one round laid no fanout:\n{r.stdout[-2000:]}{r.stderr[-2000:]}')
        plan = json.load(open(side))
        far_ = sorted(n for n, d in plan.get('tooth_dir', {}).items() if tuple(d) == (-1, 0) or tuple(d) == (-1.0, 0.0))
        if not far_:
            raise SystemExit(f'BROKEN TEST: the reversed bench carries no far-face tooth: {plan.get("tooth_dir")}')
        print(f'  far-face teeth: {far_}; the source cut {plan.get("src_cut")}')
        whole = [ln for ln in r.stdout.splitlines() if ln.startswith('WHOLE')]
        w = dict(re.findall(r'(\w+)=(\S+)', whole[-1])) if whole else {}
        print(f'  {whole[-1] if whole else "no grade line"}')
        check(not any('far face' in ln and 'REFUSED' in ln for ln in r.stdout.splitlines()),
              'the fanout audit refuses no far-face tooth')
        check(w.get('connected') == '1' and w.get('drc') == '1' and w.get('open') == '0'
              and w.get('lanes', '').split('/')[0] == w.get('lanes', '/').split('/')[-1] == '12',
              f'every lane laid, connected and DRC-clean in one round: {w}')
        check(plan.get('src_cut') is not None, 'the ends model\'s source cut rides the sidecar (src_cut)')


def end_to_end_escape():
    try:
        import ortools  # noqa: F401
    except ImportError as exc:
        print(f'SKIP (6): needs ortools ({exc})')
        return
    import synth_bus as sb
    e = dict(os.environ, ROUTE_LAYERS=','.join(sb.layer_names(3)))
    with tempfile.TemporaryDirectory() as td:
        raw, out = os.path.join(td, 'raw.kicad_pcb'), os.path.join(td, 'out.kicad_pcb')
        r = subprocess.run([sys.executable, 'synth_bus.py', raw, '--k', '12', '--copper', '4', '--dst-cols', '8',
                            '--closed', '--ring-n', '3', '--ring-s', '3', '--ring-e', '2'], cwd=AWX, capture_output=True,
                           text=True, timeout=600, env=e)
        if r.returncode != 0:
            raise SystemExit(f'BROKEN TEST: synth_bus failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
        # (the chain's sizes: the stock netclass a project-less synth board reads is no array's)
        r = subprocess.run([sys.executable, 'route_bus.py', raw, out, '--src', 'SD1', '--dest', 'SU1', '--escape',
                            '--rounds', '1', '--inproc', '--clearance', '0.1', '--track-width', '0.127',
                            '--fanout-track-width', '0.1', '--via-size', '0.25', '--via-drill', '0.15'], cwd=AWX,
                           capture_output=True, text=True, timeout=1800, env=e)
        side = os.path.join(td, 'out.bus', 'run', 'r1', 'fo.plan.json')
        run_k = re.findall(r'the run: K=(\d+)', r.stdout)
        check(run_k == ['12'], f'(6) every lane in the run, none left out of the ladder for want of a stub: K={run_k}')
        if run_k != ['12']:
            return
        if not os.path.isfile(side):
            raise SystemExit(f'BROKEN TEST: the escape round laid no fanout:\n{r.stdout[-2500:]}{r.stderr[-1500:]}')
        plan = json.load(open(side))
        far_ = sorted(n for n, d in plan.get('tooth_dir', {}).items() if tuple(d) in ((-1, 0), (-1.0, 0.0)))
        if not far_:
            raise SystemExit(f'BROKEN TEST: the escape bench carries no far-face tooth: {plan.get("tooth_dir")}')
        print(f'  escape: far-face teeth {far_}; the source cut {plan.get("src_cut")}')
        bus = [ln for ln in r.stdout.splitlines() if ln.startswith('BUS ')]
        w = dict(re.findall(r'(\w+)=(\S+)', bus[-1].split('|')[-1])) if bus else {}
        print(f'  {bus[-1] if bus else "no BUS line"}')
        check(bus and 'lanes=12/12' in bus[-1] and w.get('connected') == '1' and w.get('drc') == '1',
              f'(6) the escape fanout lays every lane, connected and DRC-clean on the board: {w}')


def end_to_end_cap():
    try:
        import ortools  # noqa: F401
    except ImportError as exc:
        print(f'SKIP (7): needs ortools ({exc})')
        return
    import synth_bus as sb
    e = dict(os.environ, ROUTE_LAYERS=','.join(sb.layer_names(3)))
    with tempfile.TemporaryDirectory() as td:
        raw, out = os.path.join(td, 'raw.kicad_pcb'), os.path.join(td, 'out.kicad_pcb')
        r = subprocess.run([sys.executable, 'synth_bus.py', raw, '--k', '12', '--copper', '4', '--dst-cols', '8',
                            '--closed', '--ring-n', '3', '--ring-s', '3', '--ring-e', '2', '--part',
                            'c0402@de:1.0:2.4:F:v'], cwd=AWX, capture_output=True, text=True, timeout=600, env=e)
        if r.returncode != 0:
            raise SystemExit(f'BROKEN TEST: synth_bus failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
        r = subprocess.run([sys.executable, 'route_bus.py', raw, out, '--src', 'SD1', '--dest', 'SU1', '--escape',
                            '--rounds', '1', '--inproc', '--clearance', '0.1', '--track-width', '0.127',
                            '--fanout-track-width', '0.1', '--via-size', '0.25', '--via-drill', '0.15'], cwd=AWX,
                           capture_output=True, text=True, timeout=1800, env=e)
        side = os.path.join(td, 'out.bus', 'run', 'r1', 'fo.plan.json')
        if not os.path.isfile(side):
            raise SystemExit(f'BROKEN TEST: the round laid no fanout:\n{r.stdout[-2500:]}{r.stderr[-1500:]}')
        plan = json.load(open(side))
        far_ = [n for n, d in plan.get('tooth_dir', {}).items() if tuple(d) in ((-1, 0), (-1.0, 0.0))]
        cut_ = plan.get('src_cut')
        # (the cap must stand on the side the far-face lanes go round, or it tests nothing: the turned frame puts the
        # board's south at the frame's north, so a far tooth north of the cut in the frame goes round the cap's side)
        if not far_ or cut_ is None:
            raise SystemExit(f'BROKEN TEST: no far-face teeth or no cut: {far_} {cut_}')
        bus = [ln for ln in r.stdout.splitlines() if ln.startswith('BUS ')]
        w = dict(re.findall(r'(\w+)=(\S+)', bus[-1].split('|')[-1])) if bus else {}
        print(f'  cap: far-face teeth {far_}; {bus[-1] if bus else "no BUS line"}')
        check(bus and 'lanes=12/12' in bus[-1] and w.get('connected') == '1' and w.get('drc') == '1',
              f'(7) a cap where the lead-ins run: every lane laid, connected and DRC-clean on the board: {w}')


end_to_end()
end_to_end_escape()
end_to_end_cap()
print('=' * 60)
print(f'{len(fails)} FAILED' if fails else 'ALL PASS')
sys.exit(1 if fails else 0)
