"""ends_passive.py OUT REF X,Y [--dive=OFF,UX,UY] -- a CROWDED END's passive moved clear on the round's fanout bench
(BENCH0, else BENCH): the loop stopped on a lane found too close, at its tooth or its berth, to a pad of REF -- a
movable passive (placement.fanout_clearance.movable_cap_refs: an unlocked two-pad C/R/FB part by an array) -- which no
solve moves. The copper the lane needs there is stamped on a copy -- with --dive a pair's two dive barrels, OFF either
side of X,Y across the lane's way (UX,UY), each a grid step wider than the via (where the snap lands it), else the
lane's track (a via a track wide) at X,Y -- and the cap step (place_fanout_clearance) run on REF alone, every other
movable passive locked, its move held within MOVE_MAX; then on each other movable passive the stamp still grazes,
alone the same way (ALSO_MAX at most); the stamp is taken off again. Only a part the cap step itself would move: one
beneath an array or beside it within NEAR_MARGIN, never one the board locks. Writes OUT and its plan sidecar (the
bench's: no end moves). Exit 0 moved; 2 REF is no movable passive; 3 the cap step moved it nowhere clear"""
import json
import math
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'py_router')))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'py_placer')))
import awx_settings  # noqa: E402
from kicad_parser import find_components_by_type, parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
import braid as te  # noqa: E402
import pairs as _pairs  # noqa: E402
import rules as _rules  # noqa: E402

NEAR_MARGIN = 3.0           # mm: a part beside an array this near its ball field is one the cap step may move
MOVE_MAX = 0.9              # mm: the farthest the part moves -- a small step off the lane, under a millimetre (Andy):
#                             a decap is where it is for the balls it serves
TOL = 1e-3
ALSO_MAX = 2                # the other passives the stamp grazes, moved with the one the finding names, at most

opt = {a.split('=')[0]: (a.split('=', 1)[1] if '=' in a else '1') for a in sys.argv[1:] if a.startswith('--')}
args = [a for a in sys.argv[1:] if not a.startswith('--')]
out, ref = args[0], args[1]
px, py = (float(v) for v in args[2].split(','))
bench = awx_settings.get('BENCH0') or awx_settings.req('BENCH')
pcb = parse_kicad_pcb(bench)
if ref not in pcb.footprints:
    print(f'ends_passive: no {ref} on the bench')
    sys.exit(2)
from placement.fanout_clearance import movable_cap_refs  # noqa: E402
arrays = sorted(fp.reference for fp in find_components_by_type(pcb, 'BGA'))
movable = movable_cap_refs(pcb, bench, near_margin=NEAR_MARGIN, near_refs=arrays)
if ref not in movable:
    print(f'ends_passive: {ref} is no passive the cap step moves (locked, not two-pad C/R/FB, or not by an array)')
    sys.exit(2)
R = _rules.active()
# ---- the stamp: the copper the lane needs at the point. A pair's dive barrels AS THE SNAP LAYS THEM -- a barrel stands
# off the grid, half a grid step off where the plan draws it either way, and the snap and the audit bar it so (a via's
# half, the clearance and half a step): each barrel's stamp a grid step wider than the via. Stamped as drawn, a part
# moved just clear of a dive left its barrel 0.326 mm from the NEXT part's pad where the snap asks 0.3275 -- the audit
# passed the dive as a near-miss, no cell round it held the barrels, and the pair shipped open. (A lane's track as
# drawn: a grid step wider, a resistor beside a memory lane was moved 0.2 mm the other way where it had been moved
# 0.6, and the singles no longer fitted round the pairs)
if '--dive' in opt:
    off, ux, uy = (float(v) for v in opt['--dive'].split(','))
    n_ = math.hypot(ux, uy) or 1.0
    nx, ny = -uy / n_, ux / n_
    pts, size = [(px + nx * off, py + ny * off), (px - nx * off, py - ny * off)], te.VIA_SIZE + te.GRID
else:
    pts, size = [(px, py)], te.TRACK
stamp = [{'x': x_, 'y': y_, 'size': size, 'drill': min(size * 0.6, te.VIA_DRILL), 'layers': ['F.Cu', 'B.Cu'],
          'net_id': 0} for x_, y_ in pts]
n2n = {i: n.name for i, n in pcb.nets.items()}
tmp, moved_b = out + '.stamp.kicad_pcb', out + '.moved.kicad_pcb'
add_tracks_and_vias_to_pcb(bench, tmp, [], stamp, [], net_id_to_name=n2n)
for ext in ('.kicad_pro', '.kicad_dru'):
    s_ = os.path.splitext(bench)[0] + ext
    if os.path.isfile(s_):
        shutil.copy(s_, os.path.splitext(tmp)[0] + ext)
made = [tmp, moved_b]


def cap_step(src, dst, who):
    """the cap step from `src` to `dst` on `who` alone, every other movable passive locked: (exit code, its text)"""
    lock = sorted(movable - {who})
    p_ = subprocess.run([sys.executable, os.path.join(HERE, '..', 'py_router', 'place_fanout_clearance.py'), src, dst,
                         '--clearance', str(R.clearance), '--near'] + arrays
                        + ['--near-margin', str(NEAR_MARGIN), '--max-displacement', str(MOVE_MAX),
                           '--max-displacement-cap', str(MOVE_MAX)] + (['--lock'] + lock if lock else []),
                        capture_output=True, text=True, env=awx_settings.environ())
    t_ = p_.stdout + p_.stderr
    for ln in t_.splitlines():
        if re.match(r'^(Moved|Stuck|  Unresolved|Caps initially)', ln):
            print('ends_passive: ' + ln.strip()[:200])
    return p_.returncode, t_


def grazes(fp):
    """a pad of `fp` nearer the stamp's copper than the clearance"""
    for pd in fp.pads:
        if pd.pad_type == 'np_thru_hole':
            continue
        cr, tl = _pairs.pad_corner_radius(pd), _pairs.pad_tilt(pd)
        for x_, y_ in pts:
            if float(_pairs.pad_distance(*_pairs.into_pad(x_ - pd.global_x, y_ - pd.global_y, tl),
                                         pd.size_x / 2, pd.size_y / 2, cr)) < size / 2 + R.clearance - TOL:
                return True
    return False


before = pcb.footprints[ref]
rc_, txt = cap_step(tmp, moved_b, ref)
if rc_ != 0 or not os.path.isfile(moved_b):
    print(f'ends_passive: the cap step exited {rc_}')
    sys.exit(3)
pm = parse_kicad_pcb(moved_b)
after = pm.footprints[ref]
d = math.hypot(after.x - before.x, after.y - before.y)
if d < TOL and abs((after.rotation - before.rotation) % 360) < TOL:
    print(f'ends_passive: {ref} not moved')
    sys.exit(3)
if re.search(rf'Unresolved[^\n]*\b{re.escape(ref)}\b', txt):
    print(f'ends_passive: {ref} moved {d:.3f} mm but still grazes')
    sys.exit(3)
# ---- ...and every OTHER movable passive the stamp grazes, each moved alone the same way, ALSO_MAX at most: the
# finding names the one pad nearest the lane's copper, and the copper stands between two parts as often as beside one
# -- a pair's dive between two 0402s, the named one moved 1.4 mm off it, the other left where it grazed the far barrel
also = []
for nb in sorted((r_ for r_ in movable - {ref} if r_ in pm.footprints and grazes(pm.footprints[r_])),
                 key=lambda r_: min(math.hypot(q_.global_x - px, q_.global_y - py) for q_ in pm.footprints[r_].pads)
                 )[:ALSO_MAX]:
    nxt = out + f'.also{len(made)}.kicad_pcb'
    made.append(nxt)
    was = pm.footprints[nb]
    rc_, txt_ = cap_step(moved_b, nxt, nb)
    if rc_ != 0 or not os.path.isfile(nxt):
        continue
    pn = parse_kicad_pcb(nxt)
    now = pn.footprints[nb]
    dn = math.hypot(now.x - was.x, now.y - was.y)
    if (dn < TOL and abs((now.rotation - was.rotation) % 360) < TOL) or \
            re.search(rf'Unresolved[^\n]*\b{re.escape(nb)}\b', txt_):
        continue
    moved_b, pm = nxt, pn
    also.append(f'{nb} {dn:.3f} mm to ({now.x:.3f}, {now.y:.3f}) {now.rotation:g}')
# ---- the stamp taken off again: its via blocks, matched by place (a net-less via has no name to match by)
content = open(moved_b, encoding='utf-8').read()
n_v = 0
for m_ in reversed(list(re.finditer(r'\(via\b', content))):
    k, depth = m_.start(), 0
    for j in range(k, len(content)):
        depth += {'(': 1, ')': -1}.get(content[j], 0)
        if depth == 0:
            break
    blk = content[k:j + 1]
    at_ = re.search(r'\(at ([-\d.]+) ([-\d.]+)\)', blk)
    if at_ and any(abs(float(at_.group(1)) - x_) + abs(float(at_.group(2)) - y_) < 2 * TOL for x_, y_ in pts) and \
            not re.search(r'\(net (?:[1-9]\d*|"[^"]+")', blk):
        e = j + 1
        while e < len(content) and content[e] in ' \t':
            e += 1
        content = content[:k] + content[e + (content[e:e + 1] == '\n'):]
        n_v += 1
if n_v != len(pts):
    print(f'ends_passive: WARNING the stamp came off {n_v}/{len(pts)}')
    sys.exit(3)
open(out, 'w', encoding='utf-8').write(content)
for ext in ('.kicad_pro', '.kicad_dru'):
    s_ = os.path.splitext(moved_b)[0] + ext
    if os.path.isfile(s_):
        shutil.copy(s_, os.path.splitext(out)[0] + ext)
shutil.copy(os.path.splitext(bench)[0] + '.plan.json', os.path.splitext(out)[0] + '.plan.json')
for f_ in made:
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        g_ = os.path.splitext(f_)[0] + ext
        if os.path.isfile(g_):
            os.remove(g_)
print(f'ends_passive: {ref} moved {d:.3f} mm to ({after.x:.3f}, {after.y:.3f}) {after.rotation:g}'
      + ''.join(f', {a_}' for a_ in also) + ' -- clear of '
      + ('the dive' if '--dive' in opt else 'the lane') + f' at ({px:.3f}, {py:.3f})')
print(f'ends_passive: wrote {out}')
