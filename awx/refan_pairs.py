#!/usr/bin/env python3
"""refan_pairs.py IN.kicad_pcb OUT.kicad_pcb SRC -- the bench's source comb made
pair-aware: every pair leg, and every net whose tooth stands BETWEEN a pair's
two teeth (same face, same layer), is stripped and fanned out again by the
production engine WITH the pairs declared (make_bench.fanout_source diff_pairs=).
The rest of the board is untouched; the .kicad_pro and .ladder.txt come along."""
import math, os, shutil, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, HERE + '/../py_router')
import contextlib, io
with contextlib.redirect_stdout(io.StringIO()):
    from kicad_parser import parse_kicad_pcb
    import make_bench as mb, pairs as _pairs, fanout_from_plan as fp
src_in, out, sref = sys.argv[1], sys.argv[2], sys.argv[3]
with contextlib.redirect_stdout(io.StringIO()):
    pcb = parse_kicad_pcb(src_in)
short = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()}
by = {s: i for i, s in short.items()}
prs = _pairs.pair_names(list(short.values()), admit_all=True)
fpt = pcb.footprints[sref]
xs = [p.global_x for p in fpt.pads]; ys = [p.global_y for p in fpt.pads]
b = (min(xs), min(ys), max(xs), max(ys))

def tooth(nid):
    pts = {}
    for s in pcb.segments:
        if s.net_id != nid: continue
        for q in ((round(s.start_x, 3), round(s.start_y, 3), s.layer), (round(s.end_x, 3), round(s.end_y, 3), s.layer)):
            pts[q] = pts.get(q, 0) + 1
    vias = {(round(v.x, 3), round(v.y, 3)) for v in pcb.vias if v.net_id == nid}
    pads = [p for p in pcb.nets[nid].pads if p.component_ref == sref]
    if not pads: return None
    pad = pads[0]
    tips = [q for q, c in pts.items() if c == 1 and (q[0], q[1]) not in vias
            and math.hypot(q[0] - pad.global_x, q[1] - pad.global_y) > 0.3]
    if not tips: return None
    x, y, l = max(tips, key=lambda q: math.hypot(q[0] - pad.global_x, q[1] - pad.global_y))
    d = {'W': abs(x - b[0]), 'E': abs(x - b[2]), 'N': abs(y - b[1]), 'S': abs(y - b[3])}
    face = min(d, key=d.get)
    return face, l[0], (y if face in 'EW' else x)

teeth = {s: tooth(i) for s, i in by.items() if any(p.component_ref == sref for p in pcb.nets[i].pads)}
refan = set()
for base, (pn, nn) in prs.items():
    tp, tn = teeth.get(pn), teeth.get(nn)
    if not tp or not tn: continue
    refan |= {pn, nn}
    if tp[0] != tn[0] or tp[1] != tn[1]: continue
    lo, hi = sorted((tp[2], tn[2]))
    between = [o for o, t in teeth.items() if t and o not in (pn, nn) and t[0] == tp[0] and t[1] == tp[1] and lo + 0.02 < t[2] < hi - 0.02]
    print(f'pair {base}: teeth {tp} / {tn}; between: {between}')
    refan |= set(between)
names_full = [pcb.nets[by[s]].name for s in sorted(refan)]
print('re-fanning', sorted(refan), 'with pairs', sorted(prs))
txt = open(src_in, encoding='utf-8').read()
txt, n_cut = mb.strip_pair_copper(txt, pcb, names_full)
base_b = out[:-len('.kicad_pcb')] + '_base.kicad_pcb'
open(base_b, 'w', encoding='utf-8').write(txt); fp.copy_pro(src_in, base_b)
print(f'stripped {n_cut} chars of copper')
with contextlib.redirect_stdout(io.StringIO()) as buf:
    nt, nv, failed = mb.fanout_source(base_b, out, sref, names_full, diff_pairs=sorted(prs), escape_method=os.environ.get('REFAN_METHOD', 'underpad'))
print(f'fanout: {nt} tracks, {nv} vias, failed {failed}')
for ext in ('.ladder.txt',):
    s0 = src_in[:-len('.kicad_pcb')] + ext
    if os.path.exists(s0): shutil.copy(s0, out[:-len('.kicad_pcb')] + ext)
