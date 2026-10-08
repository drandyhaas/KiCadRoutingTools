#!/usr/bin/env python3
"""baseline_bench.py -- the whole route against two other routers and the designer, on the same bench, nets, rules and
grade.

  baseline_bench.py run MODE BENCH DEST K OUTDIR [--board B] [--turn K CX CY] [--own-rules] [--src U1] [--no-fanout]
  baseline_bench.py ladder MODE OUTROOT [--boards PATTERN] [--tags h3_k15 ...] [--jobs N]   rungs of both ladders,
                                                                                           each in OUTROOT/MODE_TAG
  baseline_bench.py table OUTROOT [--tags ...]            the four on every rung: vias / mm / open / DRC, and notes
  baseline_bench.py render OUTROOT PNGDIR [--tags ...] [--no-others]   each rung's four boards drawn alike
                                                                       (baseline_render.py), labelled from the grades
  baseline_bench.py common NETS BOARD ...      vias on the nets every board connects (each board's conn.log beside it)

MODE:
  ours     a board the whole route wrote (--board; ladder: --boards PATTERN, with {ladder} {k} {tag} in it)
  human    the designer's board (--board; ladder: LADDERS' own), turned into the bench's frame first when it is not in
           it (--turn K CX CY, flow_frame.py turn); graded under the shared project like every router (--own-rules:
           under its own -- the zynq's, an Altium import, declares classes its copper was never routed to)
  krt      the toolkit's production chain without the bus step: bga_fanout at DEST, route_diff on the pairs,
           route.py on the rest. Like for like: no plane drops (the whole route lays none), and every other net's
           copper protected (#521) -- the router would otherwise rip and reroute the bench's other escape stubs,
           which the whole route and Freerouting hold fixed.
  fr       Freerouting with its defaults (its fanout and optimizer; --no-fanout turns the fanout off), through KiCad's
           Specctra DSN: baseline_freerouting.py, which needs FREEROUTING_JAR and FREEROUTING_JAVA.
  frgrade  the Freerouting session already in OUTDIR (ladder: OUTROOT/fr_TAG), imported and graded again

The same for every router:
  - the bench as it stands: its source array's escape stubs for every bus net, the destination bare; the rung's own
    stubs may be re-laid, every other net's copper is held (checked: `others_unchanged`);
  - every net class at the whole route's rules (rules.py: clearance, lane track, via, pair gap), in one project that
    every board is graded under;
  - the grade: check_connected and check_drc --clearance-margin 0.1 on the rung's nets, the vias and copper of the
    rung's nets. Two of check_drc's findings are reported apart rather than counted as violations: a track below the
    toolkit's fab floor but not below the board's own minimum track width (Freerouting necks down at pads), and a via
    in a pad (every router puts some there; the toolkit's writer declares them filled + capped, Freerouting's session
    does not). Freerouting routes a pair's two legs as two nets, uncoupled; the grade does not check coupling.

`run` writes OUTDIR/result.json and prints it as one line."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import collections
import contextlib
import hashlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
PYR = os.path.join(HERE, '..', 'py_router')
sys.path.insert(0, HERE)
sys.path.insert(0, PYR)

# the two ladders of the paper's Table I (paths from awx/). The zynq bench is the article's first build
# (tmp/zynq/zynqF.kicad_pcb), and its designer's board the original the README's "Building the zynq article" downloads,
# which stands a quarter turn from the bench (the turn whole_compare.py names)
LADDERS = {
    'h3': dict(bench='fb_t2q_pairs.kicad_pcb', dest='DU1', src='U1', rungs=(15, 28, 35, 41, 51),
               human='fb_t2q_human.kicad_pcb', turn=None),
    'zynq': dict(bench='tmp/zynq/zynqF.kicad_pcb', dest='U2', src='U1', rungs=(18, 26, 32, 38, 42, 44),
                 human='tmp/zynq/src/boards_set1/zynq_ad9364.kicad_pcb', turn=(1, 109.3, -103.2)),
}
ROUTERS = (('ours', 'whole route', 'ours.kicad_pcb'), ('krt', 'KRT chain', 'route.kicad_pcb'),
           ('fr', 'Freerouting', 'out.kicad_pcb'), ('human', 'human', 'human.kicad_pcb'))


def short(name):
    return (name or '').split('/')[-1]


def load(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


def ladder_nets(bench, k):
    """the rung's nets (short names), as whole_route takes them"""
    out = subprocess.run([sys.executable, 'coherent_nets.py', str(k), f'--board={bench}'], cwd=HERE,
                         capture_output=True, text=True).stdout.splitlines()
    return [n for n in (out[-1] if out else '').split(',') if n]


def rungs(tags=None):
    """[(tag, ladder name, its spec, k)] of both ladders, or of the TAGS given"""
    out = [(f'{n}_k{k}', n, spec, k) for n, spec in LADDERS.items() for k in spec['rungs']]
    return [x for x in out if not tags or x[0] in tags]


def prep(bench, out, nets, protect):
    """OUTDIR/in.kicad_pcb: the bench, its every net class at the whole route's rules; `protect`: every other net with
    copper recorded as protected (the toolkit's router never rips it)"""
    import rules
    r = rules.active()
    os.makedirs(out, exist_ok=True)
    stem = bench[:-len('.kicad_pcb')]
    shutil.copy(bench, os.path.join(out, 'in.kicad_pcb'))
    if os.path.isfile(stem + '.kicad_dru'):
        shutil.copy(stem + '.kicad_dru', os.path.join(out, 'in.kicad_dru'))
    pj = json.load(open(stem + '.kicad_pro'))
    for c in pj['net_settings']['classes']:
        c.update(clearance=r.clearance, track_width=r.track, via_diameter=r.via_size, via_drill=r.via_drill,
                 diff_pair_gap=round(r.pair_gap, 4), diff_pair_width=r.track, diff_pair_via_gap=round(r.pair_gap, 4))
    if protect:
        p = load(bench)
        names = {i: n.name for i, n in p.nets.items()}
        have = {names[x.net_id] for x in list(p.segments) + list(p.vias) if names.get(x.net_id)}
        pj.setdefault('kicad_routing_tools', {})['protected_nets'] = {
            n: 'bench' for n in sorted(have) if short(n) not in set(nets)}
    json.dump(pj, open(os.path.join(out, 'in.kicad_pro'), 'w'), indent=2)


def fingerprint(p, rung):
    """{net: hash of its tracks and vias, to 1 um} for every net outside the rung"""
    out = collections.defaultdict(list)
    nm = {i: short(n.name) for i, n in p.nets.items()}
    for s in p.segments:
        if nm.get(s.net_id) not in rung:
            ends = sorted([(round(s.start_x, 3), round(s.start_y, 3)), (round(s.end_x, 3), round(s.end_y, 3))])
            out[nm.get(s.net_id)].append(('s', s.layer, *ends, round(s.width, 3)))
    for v in p.vias:
        if nm.get(v.net_id) not in rung:
            out[nm.get(v.net_id)].append(('v', round(v.x, 3), round(v.y, 3), round(v.size, 3)))
    return {n: hashlib.md5(repr(sorted(x)).encode()).hexdigest() for n, x in out.items()}


def grade(out, board, bench, nets, own_pro=False, others=True):
    """the rung's grade on BOARD, under OUTDIR/in.kicad_pro copied beside it (`own_pro`: under the project already
    beside it); `others`: whether the other nets' copper is the bench's"""
    pro = os.path.splitext(board)[0] + '.kicad_pro'
    if not own_pro and os.path.abspath(pro) != os.path.abspath(os.path.join(out, 'in.kicad_pro')):
        shutil.copy(os.path.join(out, 'in.kicad_pro'), pro)
    pats = [f'*{n}' for n in nets]

    def check(tool, extra, log):
        with open(os.path.join(out, log), 'w') as f:
            return subprocess.run([sys.executable, os.path.join(PYR, tool), board, '--nets'] + pats + extra, cwd=HERE,
                                  stdout=f, stderr=subprocess.STDOUT).returncode
    cc = check('check_connected.py', [], 'conn.log')
    check('check_drc.py', ['--clearance-margin', '0.1'], 'drc.log')
    conn = open(os.path.join(out, 'conn.log')).read()
    opened = sorted({short(m) for m in re.findall(r'^\s+(\S[^\n]*?) \(net \d+\):', conn, re.M)} |
                    {short(m) for m in re.findall(r'^\s+(\S[^\n(]*?) \(\d+ pads\)', conn, re.M)})
    drc = open(os.path.join(out, 'drc.log')).read()
    kinds = {m.group(1): int(m.group(2)) for m in re.finditer(r'^([A-Z][A-Z-]+) violations \((\d+)\)', drc, re.M)}
    min_track = json.load(open(pro))['board']['design_settings']['rules'].get('min_track_width') or 0
    thin = [float(w) for w in re.findall(r'track too thin for fab\s*\n\s*Layer: \S+, Width: ([0-9.]+)mm', drc)]
    thin_legal = sum(1 for w in thin if w >= min_track - 1e-6)
    m = re.search(r'Vias in a solder-paste opening: (\d+) unprotected \(violations\), (\d+) filled\+capped', drc)
    in_pads = (int(m.group(1)) + int(m.group(2))) if m else 0
    real = {k: v for k, v in kinds.items() if k != 'VIA-IN-PASTE'}
    if thin_legal:
        real['TRACK-WIDTH'] = real.get('TRACK-WIDTH', 0) - thin_legal
    real = {k: v for k, v in real.items() if v}
    p = load(board)
    nm = {i: short(n.name) for i, n in p.nets.items()}
    rung = set(nets)
    changed = []
    if others:
        ref, fp = fingerprint(load(bench), rung), fingerprint(p, rung)
        changed = sorted(n for n in set(ref) | set(fp) if ref.get(n) != fp.get(n))
    return dict(open=len(opened) if cc else 0, open_nets=opened if cc else [], connected=int(cc == 0),
                drc=sum(real.values()), drc_kinds=real, thin_legal=thin_legal, vias_in_pads=in_pads,
                vias=sum(1 for v in p.vias if nm.get(v.net_id) in rung),
                mm=round(sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in p.segments
                             if nm.get(s.net_id) in rung)),
                arcs=len(re.findall(r'\(arc\s*\(start', open(board).read())),
                others_unchanged=(not changed) if others else None, others_changed=changed[:20])


def run(mode, bench, dest, k, out, board=None, src='U1', fanout=True, turn=None, own_rules=False):
    bench = os.path.abspath(os.path.join(HERE, bench))
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)
    nets = ladder_nets(bench, k)
    import pairs as _pairs
    legs = [l_ for pr in _pairs.pair_names(nets, admit_all=True).values() for l_ in pr]
    singles = [n for n in nets if n not in legs]
    pats = lambda ns: [f'*{n}' for n in ns]
    timing, t0 = {}, time.time()

    def step(name, argv, env=None, timeout=None):
        t = time.time()
        with open(os.path.join(out, name + '.log'), 'w') as f:
            try:
                rc = subprocess.run(argv, cwd=HERE, stdout=f, stderr=subprocess.STDOUT, env=env,
                                    timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                f.write('\nTIMEOUT\n')
                rc = 124
        timing[name] = round(time.time() - t, 1)
        return rc

    I = lambda f: os.path.join(out, f)
    if mode == 'ours':
        prep(bench, out, nets, protect=False)
        graded = I('ours.kicad_pcb')
        shutil.copy(board, graded)
    elif mode == 'human':
        prep(bench, out, nets, protect=False)
        graded = I('human.kicad_pcb')
        board = os.path.abspath(os.path.join(HERE, board))
        if turn:                                       # its .kicad_pro beside it, as flow_frame writes it
            step('turn', [sys.executable, 'flow_frame.py', 'turn', board, graded] + [str(x) for x in turn])
        else:
            shutil.copy(board, graded)
            shutil.copy(os.path.splitext(board)[0] + '.kicad_pro', I('human.kicad_pro'))
    elif mode == 'krt':
        prep(bench, out, nets, protect=True)
        import rules
        r = rules.active()
        sz = ['--via-size', str(r.via_size), '--via-drill', str(r.via_drill)]
        fo = [sys.executable, os.path.join(PYR, 'bga_fanout.py'), I('in.kicad_pcb'), '--output', I('fo.kicad_pcb'),
              '--component', dest, '--layers', 'F.Cu', 'B.Cu', '--nets'] + pats(nets)
        if legs:
            fo += ['--diff-pairs'] + pats(legs) + ['--diff-pair-gap', f'{r.pair_gap:.4f}']
        step('fanout', fo + ['--track-width', str(r.fan_track), '--clearance', str(r.clearance), '--plane-drop', 'off'] + sz)
        cur = I('fo.kicad_pcb')
        sizes = ['--track-width', str(r.track), '--clearance-ceiling', str(r.clearance)] + sz
        if legs:
            step('route_diff', [sys.executable, os.path.join(PYR, 'route_diff.py'), cur, '--output', I('diff.kicad_pcb'),
                                '--nets'] + pats(legs) + ['--diff-pair-gap', f'{r.pair_gap:.4f}', '--no-gnd-vias'] + sizes)
            cur = I('diff.kicad_pcb')
        step('route', [sys.executable, os.path.join(PYR, 'route.py'), cur, '--output', I('route.kicad_pcb'),
                       '--nets'] + pats(singles) + sizes)
        graded = I('route.kicad_pcb')
    elif mode in ('fr', 'frgrade'):
        import baseline_freerouting
        if mode == 'fr':
            prep(bench, out, nets, protect=False)
        timing.update(baseline_freerouting.route(I('in.kicad_pcb'), nets, src, out, fanout=fanout,
                                                 reimport=(mode == 'frgrade')))
        graded = I('out.kicad_pcb')
    else:
        sys.exit(f'baseline_bench: unknown mode {mode}')
    res = dict(mode=mode, bench=os.path.basename(bench), K=k, nets=len(nets), pairs=len(legs) // 2,
               secs=round(time.time() - t0), timing=timing)
    if os.path.isfile(graded):
        res.update(grade(out, graded, bench, nets, own_pro=(mode == 'human' and own_rules), others=(mode != 'human')))
    else:
        res['error'] = 'no board written'
    json.dump(res, open(I('result.json'), 'w'), indent=1)
    print(json.dumps(res), flush=True)
    return res


def result(outroot, mode, tag):
    p = os.path.join(outroot, f'{mode}_{tag}', 'result.json')
    return json.load(open(p)) if os.path.isfile(p) else None


def table(outroot, tags=None):
    """markdown: every rung, each of the four as vias / mm / open / DRC; then what each cell leaves out"""
    print('| rung | ' + ' | '.join(name for _, name, _ in ROUTERS) + ' |')
    print('|---|' + '---|' * len(ROUTERS))
    notes = []
    for tag, *_ in rungs(tags):
        row = []
        for mode, name, _ in ROUTERS:
            r = result(outroot, mode, tag)
            if r is None or 'error' in r:
                row.append('—' if r is None else r['error'])
                continue
            row.append(f"{r['vias']} / {r['mm']} / {r['open']} / {r['drc']}")
            if r.get('open_nets'):
                notes.append(f"{tag} {name}: open {', '.join(r['open_nets'])}")
            if r.get('drc'):
                notes.append(f"{tag} {name}: DRC {r['drc_kinds']}")
            if r.get('thin_legal'):
                notes.append(f"{tag} {name}: {r['thin_legal']} track(s) under the fab floor, not under the board's minimum")
            if r.get('others_unchanged') is False:
                notes.append(f"{tag} {name}: OTHER NETS' COPPER CHANGED {r['others_changed']}")
        print(f'| {tag} | ' + ' | '.join(row) + ' |')
    print('\ncells: vias / mm / open nets / DRC violations, on the rung\'s nets')
    for n in notes:
        print('-', n)


def render(outroot, pngdir, tags=None, others=True):
    """each rung's four boards drawn alike, labelled from their grades: PNGDIR/TAG.png"""
    os.makedirs(pngdir, exist_ok=True)
    for tag, _lad, spec, k in rungs(tags):
        items = []
        for mode, name, board in ROUTERS:
            r = result(outroot, mode, tag)
            if r is None or 'error' in r:
                continue
            lab = f"{name}: {r['vias']} vias, {r['mm']} mm" + (f", {r['open']} open" if r['open'] else '') + \
                (f", {r['drc']} DRC" if r['drc'] else '')
            items.append(f"{lab}={os.path.join(outroot, f'{mode}_{tag}', board)}")
        nets = ','.join(ladder_nets(os.path.join(HERE, spec['bench']), k))
        out = os.path.join(pngdir, f'{tag}.png')
        subprocess.run([sys.executable, os.path.join(HERE, 'baseline_render.py'), out, nets] + items +
                       ([] if others else ['--no-others']), cwd=HERE)


def common(nets_file, boards):
    rung = [n for n in open(nets_file).read().replace('\n', ',').split(',') if n]
    cols = []
    for b in boards:
        p = load(b)
        nm = {i: short(n.name) for i, n in p.nets.items()}
        res = json.load(open(os.path.join(os.path.dirname(b), 'result.json')))
        cols.append((b, collections.Counter(nm.get(v.net_id) for v in p.vias), set(res.get('open_nets', []))))
    both = [n for n in rung if not any(n in c[2] for c in cols)]
    print(f'{len(both)} of {len(rung)} nets connected on every board; vias on them:')
    for b, v, op in cols:
        print(f'  {b}: {sum(v[n] for n in both)}  (open: {sorted(op)})')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('mode', choices=('ours', 'human', 'krt', 'fr', 'frgrade'))
    r.add_argument('bench'), r.add_argument('dest'), r.add_argument('k', type=int), r.add_argument('outdir')
    r.add_argument('--board', help="ours: the board the whole route wrote; human: the designer's board")
    r.add_argument('--turn', nargs=3, type=float, metavar=('K', 'CX', 'CY'),
                   help="human: flow_frame.py's turn into the bench's frame")
    r.add_argument('--own-rules', action='store_true', help='human: graded under its own project')
    r.add_argument('--src', default='U1', help='the source array (whose escape stubs the bench carries)')
    r.add_argument('--no-fanout', action='store_true', help="fr: Freerouting's fanout off")
    lad = sub.add_parser('ladder')
    lad.add_argument('mode', choices=('ours', 'human', 'krt', 'fr', 'frgrade'))
    lad.add_argument('outroot', help='each rung in OUTROOT/MODE_TAG (frgrade: OUTROOT/fr_TAG)')
    lad.add_argument('--boards', help='ours: the whole route\'s board of each rung, a pattern with {ladder} {k} {tag}')
    lad.add_argument('--tags', nargs='*', help='h3_k15 ... (default every rung of both ladders)')
    lad.add_argument('--jobs', type=int, default=1)
    t = sub.add_parser('table')
    t.add_argument('outroot'), t.add_argument('--tags', nargs='*')
    rd = sub.add_parser('render')
    rd.add_argument('outroot'), rd.add_argument('pngdir'), rd.add_argument('--tags', nargs='*')
    rd.add_argument('--no-others', action='store_true', help="leave out the other nets' copper (the paper's figure)")
    c = sub.add_parser('common')
    c.add_argument('nets'), c.add_argument('boards', nargs='+')
    a = ap.parse_args(argv)
    if a.cmd == 'run':
        if a.mode in ('ours', 'human') and not a.board:
            ap.error(f'{a.mode} needs --board')
        run(a.mode, a.bench, a.dest, a.k, a.outdir, board=a.board, src=a.src, fanout=not a.no_fanout,
            turn=(int(a.turn[0]), a.turn[1], a.turn[2]) if a.turn else None, own_rules=a.own_rules)
    elif a.cmd == 'ladder':
        if a.mode == 'ours' and not a.boards:
            ap.error('ladder ours needs --boards')

        def one(x):
            tag, lad_, spec, k = x
            board = a.boards.format(ladder=lad_, k=k, tag=tag) if a.mode == 'ours' else \
                spec['human'] if a.mode == 'human' else None
            if board and a.mode == 'ours' and not os.path.isfile(board):
                return f'{tag}: no board {board}'
            try:
                # (frgrade grades the session a fr run left in OUTROOT/fr_TAG)
                res = run(a.mode, spec['bench'], spec['dest'], k,
                          os.path.join(a.outroot, f"{'fr' if a.mode == 'frgrade' else a.mode}_{tag}"),
                          board=board, src=spec['src'], turn=spec['turn'] if a.mode == 'human' else None)
                return f"{tag}: open {res.get('open')} drc {res.get('drc')} vias {res.get('vias')} mm {res.get('mm')}"
            except SystemExit as e:
                return f'{tag}: {e}'
        with ThreadPoolExecutor(a.jobs) as ex:
            for line in ex.map(one, rungs(a.tags)):
                print(line, flush=True)
    elif a.cmd == 'table':
        table(a.outroot, a.tags)
    elif a.cmd == 'render':
        render(a.outroot, a.pngdir, a.tags, others=not a.no_others)
    else:
        common(a.nets, a.boards)


if __name__ == '__main__':
    main()
