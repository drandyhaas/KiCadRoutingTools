#!/usr/bin/env python3
"""bend_bench.py -- the bench with its DESTINATION array moved so the bus
must turn a corner: DU1 (and the parts under it) translated rigidly to a
new centre, the outline's bottom edge pushed down to make the room, the
source fanout untouched (U1 does not move), the bench's own ladder
copied beside it so every K names the same nets.

    python3 bend_bench.py OUT.kicad_pcb CX CY [--rot DEG] [--bottom Y]
"""
import contextlib, os, re, shutil, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
AWX = HERE
sys.path.insert(0, AWX); sys.path.insert(0, os.path.join(AWX, '..', 'py_router'))
sys.path.insert(0, os.path.join(AWX, '..', 'py_placer'))
from kicad_parser import parse_kicad_pcb
from placement.writer import write_placed_output
import math

out = sys.argv[1]; cx, cy = float(sys.argv[2]), float(sys.argv[3])
rot = float(sys.argv[sys.argv.index('--rot') + 1]) if '--rot' in sys.argv else None
bottom = float(sys.argv[sys.argv.index('--bottom') + 1]) if '--bottom' in sys.argv else 96.0
base = os.path.join(AWX, 'fb_t2q_fresh.kicad_pcb')
pcb = parse_kicad_pcb(base)
du = pcb.footprints['DU1']
xs = [p.global_x for p in du.pads]; ys = [p.global_y for p in du.pads]
box = (min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5)
kids = [r for r, f in pcb.footprints.items() if r != 'DU1'
        and all(box[0] <= p.global_x <= box[2] and box[1] <= p.global_y <= box[3] for p in f.pads)]
dth = 0.0 if rot is None else rot - du.rotation
c, s = math.cos(math.radians(dth)), math.sin(math.radians(dth))
def moved(f):
    rx, ry = f.x - du.x, f.y - du.y
    # KiCad y is down: a positive rotation is counter-clockwise on screen
    nx = cx + rx * c + ry * s
    ny = cy - rx * s + ry * c
    return {'reference': f.reference, 'new_x': nx, 'new_y': ny,
            'new_rotation': (f.rotation + dth) % 360,
            'new_side': 'B' if f.layer.startswith('B') else 'F'}
pl = [moved(du)] + [moved(pcb.footprints[r]) for r in kids]
print(f'DU1 ({du.x:.2f},{du.y:.2f}) rot {du.rotation} -> ({cx},{cy}) rot {pl[0]["new_rotation"]}; with {kids}')
with contextlib.redirect_stdout(sys.stderr):
    write_placed_output(base, out, pl)
txt = open(out, encoding='utf-8').read()
txt, n = re.subn(r'\((start|end) (109\.22|152\.4) 78\.74\)', lambda m: f'({m.group(1)} {m.group(2)} {bottom})', txt)
print(f'outline bottom edge 78.74 -> {bottom} ({n} endpoints)')
open(out, 'w', encoding='utf-8').write(txt)
shutil.copy(os.path.splitext(base)[0] + '.kicad_pro', os.path.splitext(out)[0] + '.kicad_pro')
shutil.copy(os.path.join(AWX, 'k_ladder_coherent.txt'), os.path.splitext(out)[0] + '.ladder.txt')
r = subprocess.run([sys.executable, os.path.join(AWX, '..', 'py_router', 'check_drc.py'), out,
                    '--clearance', '0.1', '--clearance-margin', '0.1'], capture_output=True, text=True)
print([l for l in (r.stdout + r.stderr).splitlines() if 'DRC' in l][-1:])
pcb2 = parse_kicad_pcb(out)
d2 = pcb2.footprints['DU1']
print('DU1 now', round(d2.x, 2), round(d2.y, 2), d2.rotation, 'pads x', round(min(p.global_x for p in d2.pads), 2), '..', round(max(p.global_x for p in d2.pads), 2), 'y', round(min(p.global_y for p in d2.pads), 2), '..', round(max(p.global_y for p in d2.pads), 2), 'bounds', pcb2.board_info.board_bounds)
