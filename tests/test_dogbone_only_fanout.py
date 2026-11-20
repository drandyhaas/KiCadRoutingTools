#!/usr/bin/env python3
"""`bga_fanout.py --escape-method dogbone-only`: every connected ball of the
BGA gets one via and nothing escapes the ball field.

A user wanting only the dog-bones -- each ball a short stub to a via in the
diagonal gap, as a hand layout starts -- had nothing to ask for:
`--escape-method dogbone` (#128) places the same gap via and then routes the
escape on an inner layer. `dogbone-only` stops after the via.

The fixture is generated (nothing in it is real design data): an 8 x 8 BGA
at 0.8 mm pitch, 0.4 mm round lands, on a 4-layer board with a project at
clearance 0.1 / track 0.1 / via 0.4572 / 0.2032 and a hole-to-hole floor of
0.25. Its balls are GND (a plane net, so the #424 plane drops lay its vias),
two diff pairs (one along x, one along y), one ball with no net and one
whose net has no other pad (neither is a connected ball), and signals, each
with a partner pad on J1 west of the array.

  1. the CLI run: every connected ball has exactly one via of its net, tied
     to it by its own copper (in its pad, or a stub ending at the via); no
     track or via leaves the field (the array's ball box grown by half a
     pitch, the outermost gap sites, still under the package); the two
     unconnected balls get nothing; no ball is reported failed;
  2. each diff pair's two vias are mirrored about the pair's bisector;
  3. check_connected ties each ball to its via (the board's only copper is
     the fan-out, so a net's pads stay apart but no fan-out stub dangles);
  4. kicad-cli DRC on the output adds no clearance, short, hole or
     track-width row to the input's (skipped, and said, without kicad-cli);
  5. non-vacuity: `--escape-method dogbone` on the same board lays escape
     copper past the field, which is what this mode leaves out.
  6. a ball whose gap sites and stub ways are walled off by foreign copper
     takes its via in its pad; a pair with one such ball takes BOTH vias in
     their pads rather than splitting; with via-in-pad forbidden
     (--same-net-pad-clearance) the walled balls get no via and are reported
     by name, and the pair's free ball is laid alone and counted as split.
  7. the choice survives manifest_to_plan into a GUI plan step, and the
     fanout tab's dropdown offers it.

    python3 tests/test_dogbone_only_fanout.py
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from run_utils import evidence                                      # noqa: E402

PITCH = 0.8
N = 8
LAND = 0.4
CX, CY = 30.0, 30.0
VIA, DRILL = 0.4572, 0.2032
PAIRS = {(2, 3): '/DA_P', (3, 3): '/DA_N',      # along x
         (5, 2): '/DB_P', (5, 3): '/DB_N'}      # along y
NO_NET = (0, 7)
LONE = (7, 0)
WALLED = (3, 5)                                  # a signal ball
PAIR_WALLED = (2, 3)                             # /DA_P
_NS = uuid.UUID('0d0b0e00-0000-4000-8000-00000000d0b0')

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f' -- {detail}' if detail and not cond else ''))
    if not cond:
        failures.append(name)


def _u(key):
    return str(uuid.uuid5(_NS, key))


def ball_xy(i, j):
    span = (N - 1) * PITCH
    return CX - span / 2 + i * PITCH, CY - span / 2 + j * PITCH


def ball_net(i, j):
    if (i, j) in PAIRS:
        return PAIRS[(i, j)]
    if (i, j) == NO_NET:
        return None
    if (i, j) == LONE:
        return '/LONE'
    if (i + 2 * j) % 5 == 0:
        return 'GND'
    return f'/S{i}{j}'


def board_text(walled=()):
    """The board; `walled` lists balls (i, j) boxed in by a square of foreign
    F.Cu track that covers each one's four gap sites (and its stub's way to
    any other), leaving its pad clear for a via."""
    nets = ['']
    balls = []
    for i in range(N):
        for j in range(N):
            net = ball_net(i, j)
            if net and net not in nets:
                nets.append(net)
            balls.append((i, j, net))
    partners = sorted({n for _i, _j, n in balls
                       if n and n not in ('GND', '/LONE')})
    head = ('(kicad_pcb (version 20241229) (generator "synth") '
            '(generator_version "9.0")\n'
            '  (general (thickness 1.6) (legacy_teardrops no))\n  (paper "A4")\n'
            '  (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) '
            '(2 "B.Cu" signal) (37 "F.SilkS" user) (39 "F.Mask" user) '
            '(38 "B.Mask" user) (44 "Edge.Cuts" user) (31 "F.CrtYd" user))\n'
            '  (setup (pad_to_mask_clearance 0))\n')
    if walled:
        nets.append('/WALL')
    out = [head] + [f'  (net {k} "{n}")\n' for k, n in enumerate(nets)]

    def net_of(n):
        return f'(net {nets.index(n)} "{n}")' if n else ''
    pads = []
    span = (N - 1) * PITCH
    for i, j, net in balls:
        x, y = ball_xy(i, j)
        num = f'{"ABCDEFGH"[j]}{i + 1}'
        pads.append(f'    (pad "{num}" smd circle (at {x - CX:.4f} {y - CY:.4f}) '
                    f'(size {LAND} {LAND}) (layers "F.Cu" "F.Mask") {net_of(net)} '
                    f'(uuid "{_u("U1." + num)}"))\n')
    h = span / 2 + 0.8
    out.append(f'  (footprint "synth:BGA64" (layer "F.Cu") (uuid "{_u("U1")}") '
               f'(at {CX} {CY})\n'
               f'    (property "Reference" "U1" (at 0 {-h - 1:.2f} 0) (layer "F.SilkS") '
               f'(uuid "{_u("U1.ref")}") (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
               f'    (attr smd)\n'
               f'    (fp_rect (start {-h:.2f} {-h:.2f}) (end {h:.2f} {h:.2f}) '
               f'(stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd") '
               f'(uuid "{_u("U1.crt")}"))\n' + ''.join(pads) + '  )\n')
    jp = []
    for k, n in enumerate(partners):
        jp.append(f'    (pad "{k + 1}" smd rect (at {(k % 2) * 1.2:.2f} {(k // 2) * 0.8:.2f}) '
                  f'(size 0.6 0.4) (layers "F.Cu" "F.Mask") {net_of(n)} '
                  f'(uuid "{_u("J1." + str(k))}"))\n')
    out.append(f'  (footprint "synth:PARTNER" (layer "F.Cu") (uuid "{_u("J1")}") '
               f'(at 8 12)\n'
               f'    (property "Reference" "J1" (at 0 -1.5 0) (layer "F.SilkS") '
               f'(uuid "{_u("J1.ref")}") (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
               f'    (attr smd)\n' + ''.join(jp) + '  )\n')
    for i, j in walled:
        x, y = ball_xy(i, j)
        r = 0.36
        sq = [(x - r, y - r), (x + r, y - r), (x + r, y + r), (x - r, y + r)]
        for k, (a, b) in enumerate(zip(sq, sq[1:] + sq[:1])):
            out.append(f'  (segment (start {a[0]:.4f} {a[1]:.4f}) (end {b[0]:.4f} {b[1]:.4f}) '
                       f'(width 0.1) (layer "F.Cu") (net {nets.index("/WALL")}) '
                       f'(uuid "{_u(f"wall{i}{j}{k}")}"))\n')
    out.append('  (gr_rect (start 0 0) (end 60 60) (layer "Edge.Cuts") '
               '(stroke (width 0.1) (type solid)) (fill none) '
               f'(uuid "{_u("edge")}"))\n)\n')
    return ''.join(out)


PROJECT = {
    'board': {'design_settings': {'rules': {
        'min_clearance': 0.1, 'min_track_width': 0.1, 'min_via_diameter': 0.4,
        'min_via_annular_width': 0.1, 'min_through_hole_diameter': 0.2,
        'min_hole_to_hole': 0.25, 'min_hole_clearance': 0.1,
        'min_copper_edge_clearance': 0.3}}},
    'net_settings': {'classes': [{
        'name': 'Default', 'clearance': 0.1, 'track_width': 0.1,
        'via_diameter': VIA, 'via_drill': DRILL,
        'diff_pair_width': 0.1, 'diff_pair_gap': 0.1}]},
    'meta': {'filename': 'board.kicad_pro', 'version': 3},
}


def build(td, name='board', walled=()):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(board_text(walled))
    with open(os.path.join(td, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PROJECT, fh, indent=2)
    return path


def fan(src, dst, method, *extra):
    cmd = [sys.executable, os.path.join(ROOT, 'py_router', 'bga_fanout.py'), src,
           '--output', dst, '--component', 'U1', '--escape-method', method,
           '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
           '--diff-pairs', '/D*_[PN]', '--clearance', '0.1',
           '--track-width', '0.1', '--via-size', str(VIA), '--via-drill', str(DRILL)]
    cmd += list(extra)
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    return r


def summary(log):
    line = next((ln for ln in log.splitlines() if ln.startswith('JSON_SUMMARY: ')), None)
    return json.loads(line[len('JSON_SUMMARY: '):]) if line else None


def field_box(fp):
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    m = PITCH / 2 + 1e-3
    return min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m


def inside(box, x, y):
    return box[0] <= x <= box[2] and box[1] <= y <= box[3]


def kicad_cli():
    for c in (os.environ.get('KICAD_CLI'), shutil.which('kicad-cli'),
              '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'):
        if c and os.path.isfile(c):
            return c
    return None


def drc_rows(cli, board, out_json):
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-error',
                    '-o', out_json, board], capture_output=True, text=True)
    evidence(out_json)
    with open(out_json, encoding='utf-8') as fh:
        rep = json.load(fh)
    kinds = ('clearance', 'shorting_items', 'hole_clearance', 'hole_to_hole',
             'track_width', 'via_diameter', 'annular_width', 'drill_out_of_range',
             'copper_edge_clearance')
    rows = {}
    for v in rep.get('violations', []):
        if v.get('type') in kinds:
            rows[v['type']] = rows.get(v['type'], 0) + 1
    return rows


def main():
    from kicad_parser import parse_kicad_pcb
    import contextlib
    import io
    td = tempfile.mkdtemp(prefix='dbo_')
    try:
        src = build(td)
        evidence(src)
        out = os.path.join(td, 'out.kicad_pcb')
        r = fan(src, out, 'dogbone-only')
        log = r.stdout + r.stderr
        print("1. dogbone-only on the synthetic BGA")
        ok('the fan-out ran', r.returncode == 0 and 'Traceback' not in log,
           log[-1500:])
        evidence(out)
        with contextlib.redirect_stdout(io.StringIO()):
            pin = parse_kicad_pcb(src)
            pout = parse_kicad_pcb(out)
        fp = pin.footprints['U1']
        box = field_box(fp)
        name_of = {nid: n.name for nid, n in pout.nets.items()}
        id_of = {n: i for i, n in name_of.items()}
        connected = [p for p in fp.pads if p.net_id
                     and len(pin.pads_by_net.get(p.net_id, [])) >= 2]
        ok('the fixture has connected balls of every kind',
           len(connected) == N * N - 2
           and sum(p.net_name == 'GND' for p in connected) >= 6
           and all(any(p.net_name == n for p in connected) for n in PAIRS.values()),
           f'{len(connected)} connected')
        segs, vias = pout.segments, pout.vias
        ok('the fan-out laid copper', bool(vias), 'no vias')

        def via_of(p):
            """The vias tied to ball p by its own copper: one whose ring
            overlaps the land, or one a same-net stub from the land ends at."""
            got = []
            r_land = LAND / 2
            ends = [((s.start_x, s.start_y), (s.end_x, s.end_y)) for s in segs
                    if s.net_id == p.net_id]
            for v in vias:
                if v.net_id != p.net_id:
                    continue
                if math.hypot(v.x - p.global_x, v.y - p.global_y) < r_land + v.size / 2:
                    got.append(v)
                    continue
                # a stub polyline from the land to the via
                frontier = [(p.global_x, p.global_y)]
                seen = set()
                hit = False
                while frontier and not hit:
                    q = frontier.pop()
                    for a, b in ends:
                        for s0, s1 in ((a, b), (b, a)):
                            if math.hypot(s0[0] - q[0], s0[1] - q[1]) < 1e-3 \
                                    and (s0, s1) not in seen:
                                seen.add((s0, s1))
                                if math.hypot(s1[0] - v.x, s1[1] - v.y) < 1e-3:
                                    hit = True
                                frontier.append(s1)
                if hit:
                    got.append(v)
            return got

        per_ball = {p.pad_number: via_of(p) for p in connected}
        bad = {k: len(v) for k, v in per_ball.items() if len(v) != 1}
        ok('every connected ball has exactly one via, tied to it', not bad,
           f'{len(bad)} ball(s) off: {dict(list(bad.items())[:8])}')
        ok('one via per connected ball, no stray ones',
           len(vias) == len(connected), f'{len(vias)} vias, {len(connected)} balls')
        out_of_field = ([('via', v.x, v.y) for v in vias if not inside(box, v.x, v.y)]
                        + [('track', x, y) for s in segs
                           for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))
                           if not inside(box, x, y)])
        ok('no track or via leaves the ball field', not out_of_field,
           f'{out_of_field[:4]}')
        lone = [p for p in fp.pads if p.net_name == '/LONE' or not p.net_id]
        ok('the two unconnected balls get no via',
           len(lone) == 2 and not any(via_of(p) for p in lone if p.net_id)
           and not any(math.hypot(v.x - p.global_x, v.y - p.global_y) < LAND
                       for p in lone for v in vias))
        sm = summary(log)
        ok('no ball reported failed', sm is not None and not sm.get('failed_nets'),
           f'{sm and sm.get("failed_nets")}')
        ok('the console names the mode and its tally',
           'Dog-bone only:' in log and 'with no legal via site' in log)

        print("2. the diff pairs' vias are mirrored")
        for pn, nn in (('/DA_P', '/DA_N'), ('/DB_P', '/DB_N')):
            P = next(p for p in fp.pads if p.net_name == pn)
            Q = next(p for p in fp.pads if p.net_name == nn)
            vp, vq = per_ball[P.pad_number], per_ball[Q.pad_number]
            if len(vp) != 1 or len(vq) != 1:
                ok(f'{pn[1:3]} pair has its vias', False)
                continue
            mx, my = (P.global_x + Q.global_x) / 2, (P.global_y + Q.global_y) / 2
            ax = abs(P.global_x - Q.global_x) > abs(P.global_y - Q.global_y)
            a, b = vp[0], vq[0]
            if ax:
                mirrored = (abs((a.x - mx) + (b.x - mx)) < 1e-3
                            and abs(a.y - b.y) < 1e-3 and abs(a.x - b.x) > 1e-3)
            else:
                mirrored = (abs((a.y - my) + (b.y - my)) < 1e-3
                            and abs(a.x - b.x) < 1e-3 and abs(a.y - b.y) > 1e-3)
            ok(f'{pn[1:3]} pair ({"x" if ax else "y"} axis) mirrored about its bisector',
               mirrored, f'P via ({a.x:.3f},{a.y:.3f}) N via ({b.x:.3f},{b.y:.3f})')
        ok('the console reports the pairs mirrored', '2 mirrored' in log)

        print("3. check_connected: each ball tied to its via")
        # The fan-out is all the copper there is, so a net's balls stay apart
        # (joining them is the router's job): each net is exactly one copper
        # fragment per ball, every fragment holding a pad -- a ball, its stub
        # and its via -- and none padless (a stub or via off its ball).
        from check_connected import net_copper_fragments
        frag_bad = {}
        for nid in sorted({p.net_id for p in connected}):
            name = pin.nets[nid].name
            oid = id_of.get(name)
            balls = sum(1 for p in connected if p.net_id == nid)
            with contextlib.redirect_stdout(io.StringIO()):
                fr = net_copper_fragments(
                    oid, [s for s in segs if s.net_id == oid],
                    [v for v in vias if v.net_id == oid],
                    pout.pads_by_net.get(oid, []), pcb_data=pout)
            if fr['fragments'] != balls or fr['padless_fragments']:
                frag_bad[name] = (fr['fragments'], fr['padless_fragments'], balls)
        ok('one fragment per ball, each with its ball, on every net', not frag_bad,
           f'(fragments, padless, balls) {dict(list(frag_bad.items())[:6])}')

        print("4. kicad-cli DRC: nothing new")
        cli = kicad_cli()
        if cli is None:
            print("  SKIP: no kicad-cli (set KICAD_CLI); the DRC leg did not run")
        else:
            shutil.copy(os.path.join(td, 'board.kicad_pro'),
                        os.path.join(td, 'grade_in.kicad_pro'))
            g_in = os.path.join(td, 'grade_in.kicad_pcb')
            shutil.copy(src, g_in)
            before = drc_rows(cli, g_in, os.path.join(td, 'in.json'))
            after = drc_rows(cli, out, os.path.join(td, 'out.json'))
            new = {k: after[k] - before.get(k, 0) for k in after
                   if after[k] > before.get(k, 0)}
            ok('no new clearance / short / hole / width rows', not new,
               f'before {before}, after {after}')

        print("5. non-vacuity: dogbone escapes past the field")
        out5 = os.path.join(td, 'esc.kicad_pcb')
        r5 = fan(src, out5, 'dogbone')
        evidence(out5)
        with contextlib.redirect_stdout(io.StringIO()):
            p5 = parse_kicad_pcb(out5)
        past = [s for s in p5.segments
                if not inside(box, s.start_x, s.start_y) or not inside(box, s.end_x, s.end_y)]
        ok('the escape method lays copper past the field', r5.returncode == 0 and bool(past),
           f'{len(past)} track(s) past it')

        print("6. a walled-in ball: via in its pad, else reported")
        src6 = build(td, 'walled', walled=[WALLED, PAIR_WALLED])
        out6 = os.path.join(td, 'walled_out.kicad_pcb')
        r6 = fan(src6, out6, 'dogbone-only')
        log6 = r6.stdout + r6.stderr
        evidence(out6)
        with contextlib.redirect_stdout(io.StringIO()):
            p6 = parse_kicad_pcb(out6)
        wb = next(p for p in fp.pads if p.net_name == ball_net(*WALLED))
        wb_net = next(q.net_id for q in p6.footprints['U1'].pads
                      if q.pad_number == wb.pad_number)
        on_pad = [v for v in p6.vias if v.net_id == wb_net
                  and math.hypot(v.x - wb.global_x, v.y - wb.global_y) < 1e-3]
        ok('the walled ball takes its via in its pad', r6.returncode == 0 and len(on_pad) == 1,
           f'{len(on_pad)} via(s) at its centre')
        pa = next(p for p in p6.footprints['U1'].pads if p.net_name == '/DA_P')
        na = next(p for p in p6.footprints['U1'].pads if p.net_name == '/DA_N')
        in_pads = [q for q in (pa, na)
                   if any(v.net_id == q.net_id and math.hypot(v.x - q.global_x, v.y - q.global_y) < 1e-3
                          for v in p6.vias)]
        ok('a pair with one ball walled in takes both vias in their pads, not split',
           len(in_pads) == 2 and '1 in pad' in log6 and 'split' not in log6,
           log6[-600:])
        sm6 = summary(log6)
        ok('nothing reported failed while a pad can take the via',
           sm6 is not None and not sm6.get('failed_nets'), f'{sm6 and sm6.get("failed_nets")}')
        if cli is not None:
            shutil.copy(os.path.join(td, 'walled.kicad_pro'),
                        os.path.join(td, 'grade_w.kicad_pro'))
            g_w = os.path.join(td, 'grade_w.kicad_pcb')
            shutil.copy(src6, g_w)
            before = drc_rows(cli, g_w, os.path.join(td, 'w_in.json'))
            after = drc_rows(cli, out6, os.path.join(td, 'w_out.json'))
            new = {k: after[k] - before.get(k, 0) for k in after
                   if after[k] > before.get(k, 0)}
            ok('kicad-cli: the in-pad vias add no row either', not new,
               f'before {before}, after {after}')
        out7 = os.path.join(td, 'walled_nvip.kicad_pcb')
        r7 = fan(src6, out7, 'dogbone-only', '--same-net-pad-clearance', '0.1')
        log7 = r7.stdout + r7.stderr
        evidence(out7)
        sm7 = summary(log7)
        want = {ball_net(*WALLED), '/DA_P'}
        got = set((sm7 or {}).get('unescaped_nets') or [])
        ok('with via-in-pad forbidden, the walled balls are reported, by name',
           want <= got and 'with no legal via site: ' in log7, f'unescaped {sorted(got)}')
        with contextlib.redirect_stdout(io.StringIO()):
            p7 = parse_kicad_pcb(out7)
        ok('...and get no via at all',
           not any(v.net_id == q.net_id for v in p7.vias for q in p7.footprints['U1'].pads
                   if q.net_name in want))
        ok('...while their partner ball is laid on its own (split, disclosed)',
           '1 split' in log7 and any(v.net_id == na.net_id for v in p7.vias), log7[-600:])

        print("7. the choice reaches the GUI plan and the fanout tab's dropdown")
        sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
        import manifest_to_plan as m2p
        step = m2p.parse_command(
            ['python3', 'py_router/bga_fanout.py', 'b.kicad_pcb', '--output', 'o.kicad_pcb',
             '--component', 'U1', '--escape-method', 'dogbone-only'])
        ok('--escape-method dogbone-only -> plan param escape_method',
           (step or {}).get('params', {}).get('escape_method') == 'dogbone-only', str(step))
        import ast
        gui = os.path.join(ROOT, 'kicad_routing_plugin', 'fanout_gui.py')
        evidence(gui)
        tree = ast.parse(open(gui, encoding='utf-8').read())
        methods = next((ast.literal_eval(n.value) for n in ast.walk(tree)
                        if isinstance(n, ast.Assign) and any(
                            getattr(t, 'id', None) == 'ESCAPE_METHODS' for t in n.targets)),
                       ())
        ok("the BGA panel's ESCAPE_METHODS offers it (the plan executor and the "
           "settings restore both select by this tuple)", 'dogbone-only' in methods,
           str(methods))
    finally:
        shutil.rmtree(td, ignore_errors=True)

    print()
    if failures:
        print(f"FAILED: {len(failures)} check(s): {', '.join(failures)}")
        return 1
    print("All dogbone-only checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
