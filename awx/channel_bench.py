#!/usr/bin/env python3
"""channel_bench.py -- the bench with a LONG channel and a part standing in
it: DU1 (and the parts under it) translated rigidly to a new centre, the
outline extended to make the room, a synthetic 2x4 through-hole header
(no nets, 2.54 mm pitch) dropped where the straight chord between the
arrays runs, the source fanout untouched, the bench's own ladder beside
it. The chord from the teeth to the stubs passes through the header;
a corridor must bend round it.

    python3 channel_bench.py OUT.kicad_pcb DCX DCY HX HY [--right X] [--bottom Y]
"""
import contextlib, os, re, shutil, subprocess, sys, uuid, math
HERE = os.path.dirname(os.path.abspath(__file__))
AWX = HERE
sys.path.insert(0, AWX); sys.path.insert(0, os.path.join(AWX, '..', 'py_router'))
sys.path.insert(0, os.path.join(AWX, '..', 'py_placer'))
from kicad_parser import parse_kicad_pcb
from placement.writer import write_placed_output

out = sys.argv[1]; cx, cy = float(sys.argv[2]), float(sys.argv[3])
hx, hy = float(sys.argv[4]), float(sys.argv[5])
right = float(sys.argv[sys.argv.index('--right') + 1]) if '--right' in sys.argv else 178.0
bottom = float(sys.argv[sys.argv.index('--bottom') + 1]) if '--bottom' in sys.argv else 78.74
base = os.path.join(AWX, 'fb_t2q_fresh.kicad_pcb')
pcb = parse_kicad_pcb(base)
du = pcb.footprints['DU1']
xs = [p.global_x for p in du.pads]; ys = [p.global_y for p in du.pads]
box = (min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5)
kids = [r for r, f in pcb.footprints.items() if r != 'DU1'
        and all(box[0] <= p.global_x <= box[2] and box[1] <= p.global_y <= box[3] for p in f.pads)]
def moved(f):
    return {'reference': f.reference, 'new_x': cx + (f.x - du.x), 'new_y': cy + (f.y - du.y),
            'new_rotation': f.rotation, 'new_side': 'B' if f.layer.startswith('B') else 'F'}
pl = [moved(du)] + [moved(pcb.footprints[r]) for r in kids]
print(f'DU1 ({du.x:.2f},{du.y:.2f}) -> ({cx},{cy}); with {kids}')
with contextlib.redirect_stdout(sys.stderr):
    write_placed_output(base, out, pl)
txt = open(out, encoding='utf-8').read()
txt, n1 = re.subn(r'\((start|end) 152\.4 (53\.34|78\.74)\)', lambda m: f'({m.group(1)} {right} {m.group(2)})', txt)
txt, n2 = re.subn(r'\((start|end) (109\.22|' + str(right) + r') 78\.74\)', lambda m: f'({m.group(1)} {m.group(2)} {bottom})', txt)
print(f'outline right 152.4 -> {right} ({n1}), bottom 78.74 -> {bottom} ({n2})')
# the header: 2 x 4 through-hole pads, 2.54 pitch, vertical (rows along y), no net
ROWS = int(sys.argv[sys.argv.index("--rows") + 1]) if "--rows" in sys.argv else 5
PAD = float(sys.argv[sys.argv.index("--pad") + 1]) if "--pad" in sys.argv else 1.7
DRILL = float(sys.argv[sys.argv.index("--drill") + 1]) if "--drill" in sys.argv else 1.0
def u(): return str(uuid.uuid4())
pads = []
for col in (0, 1):
    for row in range(ROWS):
        px = (col - 0.5) * 2.54
        py = (row - (ROWS - 1) / 2) * 2.54
        pads.append(f'''\t\t(pad "{col * 4 + row + 1}" thru_hole circle
\t\t\t(at {px:.2f} {py:.2f})
\t\t\t(size {PAD} {PAD})
\t\t\t(drill {DRILL})
\t\t\t(layers "*.Cu" "*.Mask")
\t\t\t(uuid "{u()}")
\t\t)''')
fp = f'''\t(footprint "Synthetic:PinHeader_2x05_P2.54mm_Vertical"
\t\t(layer "F.Cu")
\t\t(uuid "{u()}")
\t\t(at {hx} {hy})
\t\t(property "Reference" "J9"
\t\t\t(at 0 -6 0)
\t\t\t(layer "F.SilkS")
\t\t\t(uuid "{u()}")
\t\t\t(effects
\t\t\t\t(font
\t\t\t\t\t(size 1 1)
\t\t\t\t\t(thickness 0.15)
\t\t\t\t)
\t\t\t)
\t\t)
\t\t(property "Value" "HEADER"
\t\t\t(at 0 6 0)
\t\t\t(layer "F.Fab")
\t\t\t(uuid "{u()}")
\t\t\t(effects
\t\t\t\t(font
\t\t\t\t\t(size 1 1)
\t\t\t\t\t(thickness 0.15)
\t\t\t\t)
\t\t\t)
\t\t)
\t\t(attr through_hole)
''' + '\n'.join(pads) + '\n\t)\n'
i = txt.index('\t(gr_line')
txt = txt[:i] + fp + txt[i:]
open(out, 'w', encoding='utf-8').write(txt)
shutil.copy(os.path.splitext(base)[0] + '.kicad_pro', os.path.splitext(out)[0] + '.kicad_pro')
shutil.copy(os.path.join(AWX, 'k_ladder_coherent.txt'), os.path.splitext(out)[0] + '.ladder.txt')
r = subprocess.run([sys.executable, os.path.join(AWX, '..', 'py_router', 'check_drc.py'), out,
                    '--clearance', '0.1', '--clearance-margin', '0.1'], capture_output=True, text=True)
print([l for l in (r.stdout + r.stderr).splitlines() if 'DRC' in l][-1:])
pcb2 = parse_kicad_pcb(out)
j = pcb2.footprints['J9']; d2 = pcb2.footprints['DU1']
print('J9', len(j.pads), 'pads x', round(min(p.global_x for p in j.pads), 2), '..', round(max(p.global_x for p in j.pads), 2),
      'y', round(min(p.global_y for p in j.pads), 2), '..', round(max(p.global_y for p in j.pads), 2), 'drill', j.pads[0].drill)
print('DU1 pads x', round(min(p.global_x for p in d2.pads), 2), '..', round(max(p.global_x for p in d2.pads), 2), 'y', round(min(p.global_y for p in d2.pads), 2), '..', round(max(p.global_y for p in d2.pads), 2), 'bounds', pcb2.board_info.board_bounds)
