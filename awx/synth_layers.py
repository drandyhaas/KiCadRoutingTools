#!/usr/bin/env python3
"""synth_layers.py -- the whole route on MORE ROUTING LAYERS THAN TWO, graded against the known optimum (#622).

  python3 synth_layers.py [--layers 3,4,6] [--only TAG,..] [--jobs N] [--outdir DIR] [--timeout S] [--list]

Each case is a synth_bus.py bus (its planted permutation and channel), built on a board of 4 copper layers for three
or four routing layers and of 6 for five or six, its routing layers F.Cu, B.Cu and the inner ones from the top
(synth_bus.layer_names; ROUTE_LAYERS), prepared by make_bench.py and routed by whole_route.py (BASE the bench, DEST
SD1) -- and graded on what the route laid, its WHOLE line (connected, DRC, open nets, vias, copper), against the
optimum on that many layers (synth_bus.truth_layers, from the case's own permutation):

  lb     the LIS lower bound, 2 * (K - LIS): a lane off F.Cu leaves it and comes back, two vias, on any number of
         layers -- the arrays' pads are on F.Cu
  whole  the whole-lane optimum: each lane on one layer end to end, an NL-colouring of the crossing graph (none where
         more than NL - 1 lanes all cross each other)
  opt    the exact optimum over every braid order, mid-channel changes included (the whole-lane plan's, where it meets
         the lower bound) -- what the route is graded against

More layers do not lower `opt` on a clear channel -- measured on every pattern here, the exact optimum is the lower
bound on two layers as on six -- they give ROOM: the changes a short channel cannot hold on two layers, a part in the
channel passed under rather than round. `opt` is the CHANNEL's optimum: every tooth on the source's facing column and
every berth on the destination's, in the balls' order. The whole route fans the destination itself and may leave an
array by another face, which changes the orders: reversed K=4 on three layers took two berths round the destination's
north face and laid 2 vias against the channel's 6. So a case is OPTIMAL when it is connected, DRC-clean, has no open
net and lays `opt` vias; BETTER when it lays fewer (other faces taken -- the channel model is then no bound, and no
failure either); ROUTED when it is complete above `opt` (the excess printed); FAIL otherwise. A case whose truth is
not the clear channel's (a part in it, a ring round the destination, pairs) is graded on completion, its vias against
`lb` printed. `--layers 2` runs the same cases on two routing layers, the reference. Each case's laid board is
rendered, a panel a layer and one with them all (OUTDIR/TAG_L<n>/render.png; --sheet puts them on one image a layer
count) The table to OUTDIR/layers.tsv (default tmp/synth_layers), each case's files to
OUTDIR/TAG_L<n>.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import concurrent.futures
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
sys.path.insert(0, HERE)

# (tag, K, synth_bus arguments, graded against the optimum): straight channels at depth 1 (every ball on the facing
# column, so the permutation is the problem) with the arrays' interiors CLOSED (--pad-inner 0.6, synth_ladder's b2:
# the unused balls too fat for an F track between them, so no lane threads an array or reaches another face on F --
# the channel's optimum is then the board's); the same OPEN, ungraded -- what the other faces buy (on two layers as on
# more, the route laid fewer vias than the channel's optimum on nearly every open case); then the cases whose truth is
# not the clear channel's
B8 = ['--dst-cols', '8']
CL = ['--pad-inner', '0.6']
WC = B8 + ['--closed', '--ring-n', '3', '--ring-s', '3', '--ring-e', '2']      # (the winding cases' arrays and faces)
WC16 = B8 + ['--closed', '--ring-n', '4', '--ring-s', '4', '--ring-e', '3']    # (...with sixteen lanes)
CASES = [
    ('sorted_k8', 8, CL + ['--pattern', 'sorted'], True),
    ('rev_k4', 4, CL + ['--pattern', 'reversed'], True),
    ('rev_k6', 6, CL + ['--pattern', 'reversed'], True),
    ('rev_k8', 8, CL + ['--pattern', 'reversed'], True),
    ('blocks3_k9', 9, CL + ['--pattern', 'blocks', '--blocks', '3'], True),
    ('interleave_k10', 10, CL + ['--pattern', 'interleave'], True),
    ('riffle_k10', 10, CL + ['--pattern', 'riffle', '--seed', '2'], True),
    ('shuf_k10s4', 10, CL + ['--pattern', 'shuffle', '--seed', '4'], True),
    ('shuf_k12s3', 12, CL + ['--pattern', 'shuffle', '--seed', '3'], True),
    ('shuf_k12s1', 12, CL + ['--pattern', 'shuffle', '--seed', '1'], True),
    # SHORT CHANNELS (3 mm): the changes need room two layers do not have (on two, k10s4 and k12s3 left nets open)
    ('rev_k6_g3', 6, CL + ['--pattern', 'reversed', '--gap', '3'], True),
    ('shuf_k10s4_g3', 10, CL + ['--pattern', 'shuffle', '--seed', '4', '--gap', '3'], True),
    ('shuf_k12s3_g3', 12, CL + ['--pattern', 'shuffle', '--seed', '3', '--gap', '3'], True),
    # OPEN interiors
    ('open_rev_k6', 6, ['--pattern', 'reversed'], False),
    ('open_shuf_k12s3', 12, ['--pattern', 'shuffle', '--seed', '3'], False),
    ('open_shuf_k12s3_g3', 12, ['--pattern', 'shuffle', '--seed', '3', '--gap', '3'], False),
    # a WALL in the channel on one face (inner layers pass under it), and one through it (barrels: round its ends)
    ('wall_F', 12, B8 + ['--pattern', 'shuffle', '--seed', '3', '--part', 'row@ch:6:0.0:F:v:n8:p0.8'], False),
    ('wall_vias', 12, B8 + ['--pattern', 'shuffle', '--seed', '3', '--part', 'vias@ch:6:0.0:v:n8:p0.8'], False),
    # RINGS round the destination, and PAIRS among the lanes
    ('ring_s4', 12, B8 + ['--ring-s', '4'], False),
    ('ring_ns3', 12, B8 + ['--ring-n', '3', '--ring-s', '3'], False),
    ('pairs_k12', 12, ['--pattern', 'shuffle', '--seed', '3', '--pairs', '2'], False),
    # VIA ENDS (ESCAPE_VIAS=both: every escape at both arrays through a via -- a lane takes its layer at its via, the
    # via dropped where it leaves on F.Cu): graded against the same optimum, a lane off F.Cu paying one via an end
    # whether that end is a surface escape (a change) or a via (its own)
    ('vias_rev_k6', 6, CL + ['--pattern', 'reversed'], True, {'ESCAPE_VIAS': 'both'}),
    ('vias_shuf_k12s3', 12, CL + ['--pattern', 'shuffle', '--seed', '3'], True, {'ESCAPE_VIAS': 'both'}),
    ('vias_shuf_k10s4_g3', 10, CL + ['--pattern', 'shuffle', '--seed', '4', '--gap', '3'], True,
     {'ESCAPE_VIAS': 'both'}),
    # WINDING: lanes ending on all four faces of the destination, its far face too (--ring-e), each free to go round
    # its north or its south (synth_bus.truth_wind: 'lb' the vias with every lane's way round free, 'whole' the
    # fewest of a whole-lane plan, 'opt' / 'opt_len' / 'wound' the plan the router's own price ranks first) --
    # measured, not graded, until the truth's box-hugging lengths are checked against laid copper. The arrays CLOSED
    # on every layer (--closed: their unused balls plated through), so a lane goes round the destination, not under
    # it -- the truth's lanes do (WC)
    ('wind_sorted_e2', 12, WC, False),
    ('wind_rot_e2', 12, WC + ['--pattern', 'rotate', '--shift', '4'], False),
    ('wind_shuf_e2', 12, WC + ['--pattern', 'shuffle', '--seed', '3'], False),
    # ...and the destination close beside the source (a 3 mm channel), the planted order a rotation round it: the
    # source's southern lanes end on the destination's south face east of the middle and on its far face, which a cut
    # on its south face reaches round the north with no crossing at all (truth_wind: 0 vias, five lanes wound, where
    # a cut on the far face pays 4) -- and the same with every escape a via, a dog-bone at each end as a human lays
    ('wind_rot16_g3', 16, WC16 + ['--pattern', 'rotate', '--shift', '11', '--gap', '3'], False),
    ('wind_rot16_g3_vias', 16, WC16 + ['--pattern', 'rotate', '--shift', '11', '--gap', '3'], False,
     {'ESCAPE_VIAS': 'both'}),
]


def run(argv, log, env=None, timeout=None):
    with open(log, 'w') as fh:
        try:
            return subprocess.run(argv, stdout=fh, stderr=subprocess.STDOUT, cwd=HERE, env=env, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            fh.write(f'\nTIMEOUT after {timeout} s\n')
            return -1


# the layers' colours (a board of up to six): F red, the inner ones green, purple, orange, teal, B blue
COLOURS = {'F.Cu': (220, 50, 40), 'In1.Cu': (20, 160, 60), 'In2.Cu': (150, 60, 200), 'In3.Cu': (230, 140, 20),
           'In4.Cu': (20, 160, 170), 'B.Cu': (40, 100, 230)}


def render(board, out, names, title, scale=32.0):
    """`board` drawn to `out`: a panel for each copper layer and one with them all -- the bus's copper (`names`, short
    net names) in its layer's colour, every other net's and every pad grey, vias rings (the bus's black) -- each panel
    titled. Returns the all-layer panel (a PIL image), or None when the board will not read"""
    import contextlib
    import io
    from PIL import Image, ImageDraw
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    from kicad_parser import parse_kicad_pcb
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(board)
    except Exception:
        return None
    bus = {i for i, n in pcb.nets.items() if n.name and n.name.split('/')[-1] in set(names)}
    xs = [p_.global_x for fp in pcb.footprints.values() for p_ in fp.pads] + \
         [v for s_ in pcb.segments for v in (s_.start_x, s_.end_x)]
    ys = [p_.global_y for fp in pcb.footprints.values() for p_ in fp.pads] + \
         [v for s_ in pcb.segments for v in (s_.start_y, s_.end_y)]
    x0, y0, x1, y1 = min(xs) - 1, min(ys) - 1, max(xs) + 1, max(ys) + 1
    layers = list(pcb.board_info.copper_layers)
    from PIL import ImageFont
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 16)
    except Exception:
        font = None
    # (a panel at least as wide as its longest title, the board centred in it)
    tw = max((font.getlength(f'{title}: {L}') if font is not None else 7 * len(f'{title}: {L}'))
             for L in layers + ['all layers'])
    Wb = int((x1 - x0) * scale)
    W, H = max(Wb, int(tw) + 12), int((y1 - y0) * scale) + 24
    ox = (W - Wb) / 2
    P = lambda x, y: ((x - x0) * scale + ox, (y - y0) * scale + 24)
    panels = []
    for only in layers + [None]:
        im = Image.new('RGB', (W, H), 'white')
        dr = ImageDraw.Draw(im)
        for fp in pcb.footprints.values():
            for p_ in fp.pads:
                on = only is None or only in p_.layers or '*.Cu' in p_.layers
                a, b = P(p_.global_x - p_.size_x / 2, p_.global_y - p_.size_y / 2), \
                    P(p_.global_x + p_.size_x / 2, p_.global_y + p_.size_y / 2)
                dr.rectangle([a, b], fill=(205, 205, 205) if on else (238, 238, 238))
        for s_ in sorted(pcb.segments, key=lambda q: q.net_id in bus):
            if only is not None and s_.layer != only:
                continue
            c_ = COLOURS.get(s_.layer, (0, 0, 0)) if s_.net_id in bus else (180, 180, 180)
            dr.line([P(s_.start_x, s_.start_y), P(s_.end_x, s_.end_y)], fill=c_,
                    width=max(1, int(s_.width * scale)))
        for v in pcb.vias:
            c, r_ = P(v.x, v.y), v.size / 2 * scale
            dr.ellipse([c[0] - r_, c[1] - r_, c[0] + r_, c[1] + r_],
                       outline=(0, 0, 0) if v.net_id in bus else (150, 150, 150), width=2)
        dr.text((6, 2), f'{title}: {only or "all layers"}', fill=(0, 0, 0), font=font)
        panels.append(im)
    sheet_ = Image.new('RGB', (W * len(panels) + 8 * (len(panels) - 1), H), (90, 90, 90))
    for i, im in enumerate(panels):
        sheet_.paste(im, (i * (W + 8), 0))
    sheet_.save(out)
    return panels[-1]


def sheet(outdir, rows, NL, cols=4):
    """every case's all-layer panel for NL routing layers on one image, OUTDIR/sheet_L<NL>.png"""
    from PIL import Image
    ims = [Image.open(os.path.join(outdir, f"{r['tag']}_L{NL}", 'all.png')) for r in rows
           if os.path.isfile(os.path.join(outdir, f"{r['tag']}_L{NL}", 'all.png'))]
    if not ims:
        return None
    w, h = max(i.width for i in ims), max(i.height for i in ims)
    rows_ = (len(ims) + cols - 1) // cols
    out = Image.new('RGB', (cols * (w + 8), rows_ * (h + 8)), (90, 90, 90))
    for k, im in enumerate(ims):
        out.paste(im, ((k % cols) * (w + 8), (k // cols) * (h + 8)))
    path = os.path.join(outdir, f'sheet_L{NL}.png')
    out.save(path)
    print(f'sheet: {path}', flush=True)
    return path


def truth_of(raw, NL):
    """the case's optimum on NL routing layers (synth_bus.truth_layers), from its own sidecar's permutation: the lanes
    in source order, `perm` each one's place along the destination's face"""
    import synth_bus as sb
    t = json.load(open(raw[:-len('.kicad_pcb')] + '.truth.json'))
    if t.get('wind'):
        # (a winding case: its lanes' ways round the destination free -- the lower bound, the whole-lane plan the
        # router's own price ranks first, its length and how many of its lanes wind past the cut)
        w = sb.truth_wind(t['wind'], NL)
        return {'lb': w['lb'], 'whole': w['vias'], 'opt': w['opt'], 'opt_len': w['opt_len'], 'wound': w['wound'],
                'cut_opt': w['cut_opt'], 'cut_len': w['cut_len'], 'far_opt': w['far_opt'], 'far_len': w['far_len']}
    src = list(range(len(t['perm'])))
    dst = sorted(src, key=lambda i: t['perm'][i])
    return sb.truth_layers(src, dst, NL)


def one(tag, K, args, graded, NL, outdir, timeout, sets=None):
    import synth_bus as sb
    copper = 2 if NL == 2 else (4 if NL <= 4 else 6)
    d = os.path.join(outdir, f'{tag}_L{NL}')
    os.makedirs(d, exist_ok=True)
    raw, bench = os.path.join(d, 'raw.kicad_pcb'), os.path.join(d, 'bench.kicad_pcb')
    env = dict(os.environ, ROUTE_LAYERS=','.join(sb.layer_names(NL)), **(sets or {}))
    row = {'tag': tag, 'layers': NL, 'k': K,
           'args': ' '.join(args + [f'{k_}={v_}' for k_, v_ in sorted((sets or {}).items())])}
    if run([PY, 'synth_bus.py', raw, '--k', str(K)] + (['--copper', str(copper)] if copper != 2 else []) + args,
           os.path.join(d, 'gen.log'),
           env=env):
        return dict(row, verdict='GEN FAILED')
    tr = truth_of(raw, NL)
    row.update(lb=tr['lb'], whole=tr['whole'], opt=tr['opt'] if graded else None,
               opt_vias=tr['opt'] if 'opt_len' in tr else None, opt_len=tr.get('opt_len'), wound=tr.get('wound'),
               **{k_: tr.get(k_) for k_ in ('cut_opt', 'cut_len', 'far_opt', 'far_len')})
    if run([PY, 'make_bench.py', raw, 'SU1', 'SD1', bench], os.path.join(d, 'bench.log'), env=env) \
            or not os.path.isfile(bench):
        return dict(row, verdict='BENCH FAILED')
    t0 = time.time()
    rc = run([PY, 'whole_route.py', str(K), os.path.join(d, 'run')], os.path.join(d, 'run.log'),
             env=dict(env, BASE=bench, DEST='SD1'), timeout=timeout)
    row['secs'] = round(time.time() - t0)
    whole = [ln for ln in open(os.path.join(d, 'run.log')) if ln.startswith('WHOLE')]
    if not whole:
        return dict(row, verdict=f'NO RESULT (rc {rc})')
    w = dict(re.findall(r'(\w+)=(\S+)', whole[-1]))
    row.update({k: w.get(k) for k in ('K', 'round', 'vias', 'copper', 'connected', 'drc', 'open')})
    done = w.get('connected') == '1' and w.get('drc') == '1' and w.get('open') == '0'
    vias = int(w['vias']) if (w.get('vias') or '').isdigit() else None
    if not done:
        row['verdict'] = 'FAIL'
    elif not graded or row['opt'] is None:
        row.update(verdict='ROUTED', excess=None if vias is None or tr['lb'] is None else vias - tr['lb'])
    else:
        row['excess'] = vias - row['opt']
        row['verdict'] = 'OPTIMAL' if row['excess'] == 0 else ('BETTER' if row['excess'] < 0 else 'ROUTED')
    # the laid board, drawn
    best = os.path.join(d, 'run', 'best.kicad_pcb')
    if os.path.isfile(best):
        names = json.load(open(raw[:-len('.kicad_pcb')] + '.truth.json')).get('names') or []
        title = (f"{tag} on {NL} layers: {row['verdict']}, {row.get('vias')} vias"
                 + (f" (optimum {row['opt']})" if row.get('opt') is not None else f" (bound {tr['lb']})")
                 + (f", {row['open']} open" if row.get('open') not in (None, '0', 0) else ''))
        allp = render(best, os.path.join(d, 'render.png'), names, title)
        if allp is not None:
            allp.save(os.path.join(d, 'all.png'))
    return row


COLS = ('tag', 'layers', 'k', 'verdict', 'opt', 'vias', 'excess', 'lb', 'whole', 'round', 'copper', 'connected', 'drc',
        'open', 'secs', 'opt_vias', 'opt_len', 'wound', 'cut_opt', 'cut_len', 'far_opt', 'far_len', 'args')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--layers', default='3,4,6', help='the routing-layer counts (default 3,4,6; 2 the reference)')
    ap.add_argument('--only', help='TAG,.. of the cases')
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--outdir', default=os.path.join(HERE, 'tmp', 'synth_layers'))
    ap.add_argument('--timeout', type=int, default=1800)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--sheet', action='store_true', help='every case\'s all-layer render on one image a layer count')
    a = ap.parse_args(argv)
    nls = [int(x) for x in a.layers.split(',') if x]
    if any(n < 2 or n > 6 for n in nls):
        raise SystemExit(f'synth_layers: --layers {a.layers}: two to six')
    cases = [c for c in CASES if not a.only or c[0] in a.only.split(',')]
    if a.list:
        for tag, K, args, graded, *sets in cases:
            print(f'{tag:20s} K={K:3d} {"graded" if graded else "routed"}  {" ".join(args)}'
                  + ''.join(f' {k_}={v_}' for st_ in sets for k_, v_ in sorted(st_.items())))
        return 0
    os.makedirs(a.outdir, exist_ok=True)
    jobs = [(c, n) for n in nls for c in cases]
    rows = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = {ex.submit(one, c[0], c[1], c[2], c[3], n, os.path.abspath(a.outdir), a.timeout,
                          c[4] if len(c) > 4 else None): (c, n)
                for c, n in jobs}
        for f in concurrent.futures.as_completed(futs):
            r = f.result()
            rows.append(r)
            print('  '.join(f'{c}={r.get(c)}' for c in COLS if c != 'args'), flush=True)
    rows.sort(key=lambda r: (r['tag'], r['layers']))
    with open(os.path.join(a.outdir, 'layers.tsv'), 'w') as f:
        f.write('\t'.join(COLS) + '\n')
        for r in rows:
            f.write('\t'.join(str(r.get(c, '')) for c in COLS) + '\n')
    n_opt = sum(r['verdict'] in ('OPTIMAL', 'BETTER') for r in rows if r.get('opt') is not None)
    n_graded = sum(1 for r in rows if r.get('opt') is not None)
    n_fail = sum(r['verdict'] not in ('OPTIMAL', 'BETTER', 'ROUTED') for r in rows)
    above = [f"{r['tag']}_L{r['layers']} +{r['excess']}" for r in rows
             if r.get('opt') is not None and r['verdict'] == 'ROUTED']
    print(f'\n{len(rows)} runs: {n_opt} of {n_graded} graded at or below the channel optimum'
          + (f' (above it: {", ".join(above)})' if above else '') + f', {n_fail} failed; '
          f'the table in {os.path.join(a.outdir, "layers.tsv")}')
    if a.sheet:
        for n in nls:
            sheet(a.outdir, [r for r in rows if r['layers'] == n], n)
    return 0 if n_fail == 0 and n_opt == n_graded else 1


if __name__ == '__main__':
    sys.exit(main())
