"""whole_render.py PLAN.json OUT.png [view x0,y0,x1,y1] [LANES] -- a whole-route plan drawn over its board: F red, B blue,
vias white, LANES thick and labelled; the plan's conflicts (a failed snap's lanes) yellow and numbered, and with
AUDIT=FILE the audit's failures as magenta crosses. The board is BENCH; the pad boxes drawn are DEST's (and SRC's)."""
import sys, os, json, contextlib, io
A = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, A); sys.path.insert(0, os.path.join(A, '..', 'py_router')); os.chdir(A)
from kicad_parser import parse_kicad_pcb, Segment as _S
from route_render import BoardRenderer
from PIL import ImageFont
g = json.load(open(sys.argv[1])); out = sys.argv[2]
view = tuple(map(float, sys.argv[3].split(','))) if len(sys.argv) > 3 and sys.argv[3] else None
hl = set(sys.argv[4].split(',')) if len(sys.argv) > 4 and sys.argv[4] else set()
ONLY = os.environ.get('LAYER')
with contextlib.redirect_stdout(io.StringIO()):
    pcb = parse_kicad_pcb(os.environ['BENCH'])
try:
    font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 18)
except Exception:
    font = None
COL = {'F.Cu': (255, 90, 70), 'B.Cu': (70, 160, 255)}
def ov(dr, rr):
    for L in ('B.Cu', 'F.Cu'):
        if ONLY and L != ONLY: continue
        for n, v in g['lanes'].items():
            w = 0.09 if n in hl else 0.05
            rr._draw_segments(dr, [_S(a, b, c_, d, w, L, 0) for a, b, c_, d, L_ in v['pieces'] if L_ == L], color=COL[L])
    for (n, x, y) in g['vias']:
        cx, cy = rr.tf.pt(x, y); r_ = rr.tf.length(0.25) / 2
        dr.ellipse([cx - r_, cy - r_, cx + r_, cy + r_], outline=(255, 255, 255), width=2)
    for n in hl:
        xy = g['lanes'][n]['xy']; p = xy[len(xy) // 2]
        cx, cy = rr.tf.pt(*p); dr.text((cx + 4, cy + 4), n, fill=(255, 255, 0), font=font)
    # the lanes a failed snap could not lay (yellow, numbered) and the audit's failures (magenta)
    confs = sorted(g.get('conflicts', []), key=lambda c_: -max(r_['short'] for r_ in c_['charged']))
    for i, c_ in enumerate(confs):
        if not c_.get('xy'):
            continue
        cx, cy = rr.tf.pt(*c_['xy']); r_ = rr.tf.length(0.35)
        dr.ellipse([cx - r_, cy - r_, cx + r_, cy + r_], outline=(255, 230, 0), width=3)
        dr.text((cx + r_ + 2, cy - r_), str(i + 1), fill=(255, 230, 0), font=font)
    if os.environ.get('AUDIT'):
        import re
        for line in open(os.environ['AUDIT']):
            m = re.search(r'at \((-?\d+\.\d+),\s*(-?\d+\.\d+)\)', line) or re.search(r'^DIVE \S+\s+\(\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)\)', line)
            if not m or not re.match(r'^(PITCH|STATIC|DIVE|SHAPE) [A-Z]', line):
                continue
            cx, cy = rr.tf.pt(float(m.group(1)), float(m.group(2))); r_ = rr.tf.length(0.2)
            dr.line([cx - r_, cy - r_, cx + r_, cy + r_], fill=(255, 0, 255), width=3)
            dr.line([cx - r_, cy + r_, cx + r_, cy - r_], fill=(255, 0, 255), width=3)
    for fp in [r_ for r_ in (os.environ.get('SRC'), os.environ.get('DEST')) if r_ and r_ in pcb.footprints]:
        f_ = pcb.footprints[fp]; xs = [p.global_x for p in f_.pads]; ys = [p.global_y for p in f_.pads]
        x0, y0 = rr.tf.pt(min(xs) - 1, min(ys) - 1); x1, y1 = rr.tf.pt(max(xs) + 1, max(ys) + 1)
        dr.rectangle([min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)], outline=(255, 255, 0), width=1)
r = BoardRenderer(pcb, size=1800, supersample=2, show_zones=False, view=view, layer_alpha=70)
r.frame(segments=[], vias=[], overlays=[ov], label=f'{os.path.basename(sys.argv[1])}: F red, B blue, vias white').save(out)
print('wrote', out)
