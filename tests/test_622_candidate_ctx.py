#!/usr/bin/env python3
"""The ends model asks the WHOLE SOLVE of its own escape ends before any is laid (whole_ends.solve_check): the bench
those ends make is built in memory (whole_ctx.candidate) and given to whole_solve.solve under its strict rules. That is
only the solve's verdict if the in-memory bench IS the one the round's solve will read off the laid board.

  python3 tests/test_622_candidate_ctx.py

On a board of its own -- two 5 x 5 ball arrays at 0.8 mm pitch, the source U1 west of the destination U5 (the canonical
frame), four copper layers, every ball netted, five bus nets between them (one a differential pair) -- the ends are
made as a plain bus's fanout makes them: the source fanned out for the bus (make_bench.fanout_source, the bus step's
own), the plan state read off that board (fanout_from_plan.plan_state), and the berths chosen against the teeth as laid
(fanout_from_plan.dest_choice: the greedy selection, then whole_ends.choose -- a destination re-plan's choice, S2's and
S3's berths held on the destination's near face so those two lanes cross). That state is turned into a plan
(whole_ends.state_plan) and, two ways, into a bench: candidate() in memory, built as solve_check builds it, and
braid.setup on the board with the berths LAID by the fanout engine (fanout_from_plan.fanout_once, every berth as asked)
and the state's plan sidecar beside it, as the round's solve reads it. Pinned:

1. every field the whole frame and the solve read is the same: the lanes and pairs, each lane's ends (a pair's at its
   legs' midpoints) and its legs' ends, the layers (a pair's P leg's, its N leg's beside them), the escape directions,
   the source part, the pair frame, the destination's cut, the routing config, the copper (the board's own and the
   escape runs, each net's runs on each layer as drawn: the engine lays a straight run as one segment where the
   state's tracks may carry it in two), the vias at their sizes, and each lane's TAUT PATH -- round the arrays' other
   balls, and a 0402 beside the trunk whose pads the solve's built-in via cuts read the lanes' references against;
2. the whole frame built on each (whole_frame.build) is the same: its lanes, launch and final orders, ring classes,
   starts, landings, handoffs, and each lane's reference (mid);
3. whole_solve.solve(check=True), asked as solve_check asks it (no escape's face zones, its cores on), gives each the
   same verdict, and measures the same rooms, via ends and banned end layers -- and its solve, run as the round runs
   it but on one worker (on the round's four, plans equal in every price race), the same plan;
4. a run of the pair alone, every lane a pair, reads the same sidecar (keyed by the pair's legs) and the same fields --
   an all-pair bus's setup once found its member's name in none of the sidecar's keys and dropped the plan, reading
   the escape directions off copper (sdi_mipi_conv's MIPI bus to J4: a south tooth read west, refused every round).

All four on the fixture as drawn (pair chirality +1) and with every ball's row mirrored (-1: setup turns the board over
in memory and mirrors the plan into it; candidate() must do the same, or the check reads a bus in that frame as
another board and its verdict is not the round's).

A field setup gains and candidate() does not read is a drift this test sees only where the frame or the solve reads it.
"""
import contextlib
import io
import json
import math
import os
import sys
import tempfile

try:
    import ortools  # noqa: F401  (the solve's check is a CP-SAT solve)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)
# (the fanout stage's own plan environment, as the whole route sets it for every stage: whole_route.stage_env)
os.environ.update(ROUTE_LAYERS='F.Cu,B.Cu,In1.Cu,In2.Cu', DEST='U5', BRAID_EXACT_PAGES='0')
for k_ in ('BENCH', 'NETS', 'FEEDBACK', 'INCREMENTAL', 'STAGE_CACHE', 'TAUT_MEMO', 'GROUP_ENDS', 'ENDS_CAPTURE',
           'FANOUT_NETS', 'ENDS_BLOCKED', 'JOINT_FOREIGN'):
    os.environ.pop(k_, None)

import rules as _rules  # noqa: E402
_rules.install(_rules.Rules(clearance=0.09, track=0.15, via_size=0.45, via_drill=0.3, fan_track=0.12,
                            hole_to_hole=0.25, edge_clearance=0.2, source='test'))
import braid as bd  # noqa: E402
import escape_moves as em  # noqa: E402
import fanout_from_plan as ffp  # noqa: E402
import make_bench as mb  # noqa: E402
import source_realize as sr  # noqa: E402
import whole_ctx  # noqa: E402
import whole_ends  # noqa: E402
import whole_frame  # noqa: E402
import whole_solve  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

# (every solve on ONE worker: two solves of one model then give one plan. On the round's four, plans equal in every
# price -- the crossings' places along the lanes -- come out of the two solves either way, run to run)
whole_solve.SOLVE_WORKERS = 1

N, PITCH, GAP = 5, 0.8, 7.0
# net: (U1 ball, U5 ball) -- the balls' moment about the source-to-destination axis positive: pair chirality +1; S2
# leaves the source's top row for the destination's bottom one and S3 the other way, so the lanes cross
BUS = {'D_P': (18, 16), 'D_N': (23, 21), 'S1': (13, 11), 'S2': (3, 22), 'S3': (24, 1)}
# ...and every ball's row mirrored: the balls' moment negative, chirality -1 -- a plan made in the turned-over pair frame,
# which setup reads on the board turned over in memory and candidate() must read the same way
BUS_TURNED = {nm: tuple((N - 1 - b_ // N) * N + b_ % N for b_ in ab) for nm, ab in BUS.items()}


def board(bus):
    """two ball arrays, U1 at x 0 and U5 at x GAP + 4 pitches: the bus's balls on `bus`'s nets, every other ball a net
    of its own"""
    nets, n_of = ['(net 0 "")'], {}
    for nm in bus:
        n_of[nm] = len(nets)
        nets.append(f'(net {len(nets)} "{nm}")')
    used = {(ref, b): nm for nm, (a, z) in bus.items() for ref, b in (('U1', a), ('U5', z))}
    pads = {}
    for ref, x0 in (('U1', 0.0), ('U5', GAP + (N - 1) * PITCH)):
        out = []
        for i in range(N * N):
            nm = used.get((ref, i))
            if nm is None:
                nm = f'{ref}_{i}'
                n_of[nm] = len(nets)
                nets.append(f'(net {len(nets)} "{nm}")')
            out.append(f'    (pad "{i}" smd circle (at {(i % N) * PITCH:.3f} {(i // N) * PITCH:.3f}) (size 0.4 0.4) '
                       f'(layers "F.Cu" "F.Mask") (net {n_of[nm]} "{nm}"))')
        pads[ref] = (x0, '\n'.join(out))
    fps = '\n'.join(f'  (footprint "BGA" (layer "F.Cu") (at {x0:.3f} 0)\n    (property "Reference" "{ref}" (at 0 0) '
                    f'(layer "F.SilkS"))\n{body}\n  )' for ref, (x0, body) in pads.items())
    # (a 0402 beside the trunk, off the arrays' top row: its pads are what the solve's built-in via cuts read a lane's
    # reference against, and the reference is the lane's taut path's)
    cn = []
    for k in (1, 2):
        n_of[f'C1_{k}'] = len(nets)
        nets.append(f'(net {len(nets)} "C1_{k}")')
        cn.append(f'    (pad "{k}" smd rect (at {(k - 1.5):.3f} 0) (size 0.5 0.5) (layers "F.Cu" "F.Mask") '
                  f'(net {n_of[f"C1_{k}"]} "C1_{k}"))')
    fps += (f'\n  (footprint "C0402" (layer "F.Cu") (at {GAP / 2 + (N - 1) * PITCH / 2 + 1:.3f} -0.9)\n'
            f'    (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))\n' + '\n'.join(cn) + '\n  )')
    W = GAP + 2 * N * PITCH + 4
    return f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  {chr(10).join('  ' + n for n in nets)}
{fps}
  (gr_rect (start -3 -4) (end {W:.3f} 7) (layer "Edge.Cuts") (width 0.1))
)
"""


def main():
    fails = []
    for bus, want in ((BUS, 1), (BUS_TURNED, -1)):
        case(bus, want, fails)
    return report(fails)


def case(bus, want, fails_all):
    """the comparisons on the fixture with `bus`'s balls, whose pair reads chirality `want`; each failure into
    `fails_all`, named by the chirality"""
    fails = []
    _case(bus, want, fails)
    fails_all += [f'(chirality {want:+d}) {f_}' for f_ in fails]


@contextlib.contextmanager
def quiet():
    """the engine's and the planner's console, on both streams, out of the test's"""
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        yield


def runs(segs):
    """the copper of `segs` as drawn: each net's maximal straight runs on each layer at each width, collinear pieces
    that touch or overlap joined -- a run the same however it is cut into segments"""
    by = {}
    for s in segs:
        ax, ay, bx, by_ = s.start_x, s.start_y, s.end_x, s.end_y
        ln = math.hypot(bx - ax, by_ - ay)
        if ln < 1e-9:
            continue
        ux, uy = (bx - ax) / ln, (by_ - ay) / ln
        if ux < -1e-9 or (abs(ux) <= 1e-9 and uy < 0):
            ux, uy = -ux, -uy
        key = (s.net_id, s.layer, round(s.width, 4), round(ux, 4), round(uy, 4), round(ux * ay - uy * ax, 4))
        t0, t1 = sorted((ux * ax + uy * ay, ux * bx + uy * by_))
        by.setdefault(key, []).append((t0, t1))
    out = []
    for key, iv in by.items():
        iv.sort()
        a, b = iv[0]
        for c, d in iv[1:]:
            if c <= b + 1e-4:
                b = max(b, d)
            else:
                out.append(key + (round(a, 4), round(b, 4)))
                a, b = c, d
        out.append(key + (round(a, 4), round(b, 4)))
    return sorted(out)


def _case(bus, want, fails):
    with tempfile.TemporaryDirectory() as td:
        raw, src, laid = (os.path.join(td, f) for f in ('raw.kicad_pcb', 'src.kicad_pcb', 'laid.kicad_pcb'))
        open(raw, 'w').write(board(bus))
        names = list(BUS)
        # the source fanned out for the bus, as the bus step fans it (route_bus: make_bench.fanout_source)
        with quiet():
            _nt, _nv, refused = mb.fanout_source(raw, src, 'U1', names)
            pcb = parse_kicad_pcb(src)
        if refused:
            fails.append(f'the fixture: the source fanout refused {refused}')
            return
        # the plan state on that board, and the berths a destination pass chooses against its teeth as laid
        # (fanout_from_plan.dest_choice: the greedy selection, then the ends model's choice -- whole_ends.choose) --
        # with S2's and S3's held, as a re-plan holds the berths laid before (its `fixed`): each a dog-bone on the
        # destination's near face, so the two lanes cross between the arrays (left to the ends model, it takes them
        # round the destination's sides and nothing crosses: no crossing window, no change, no via cut for the solve)
        with quiet():
            st = ffp.plan_state(pcb, names)
            st['base'] = src
        cen = lambda ref: tuple(sum(c) / len(c) for c in zip(*[(p.global_x, p.global_y)
                                                                for p in pcb.footprints[ref].pads]))
        (sx, sy), (dx, dy) = cen('U1'), cen('U5')
        near = max(em.DIRS, key=lambda d: em.DIRS[d][0] * (sx - dx) + em.DIRS[d][1] * (sy - dy))
        hold = {}
        for nm in ('S2', 'S3'):
            ball = st['dst_pad'][nm]
            ms = [m for m in st['dmenu'][nm] if m.direction == near and m.kind == 'dogbone']
            if not ms:
                fails.append(f'the fixture: {nm} has no dog-bone berth on U5\'s {near} face')
                return
            hold[nm] = sr.move_sig(min(ms, key=lambda m: math.hypot(m.site[0] - ball.global_x,
                                                                     m.site[1] - ball.global_y)))
        with quiet():
            choice, unplaced = ffp.dest_choice(st, src, log=lambda *a: None, fixed=hold)
            E = whole_ends.Ends(st, src_free=False)
            state = E.start(choice)
        if unplaced or set(choice) != set(names):
            fails.append(f'the fixture: no berth chosen for {sorted(set(names) - set(choice))}')
            return
        loose = sorted(nm for nm, sig in hold.items() if sr.move_sig(choice[nm]) != sig)
        if loose:
            fails.append(f'the fixture: the berths held for {loose} not chosen')
            return
        # (the state is the choice: each lane at the berth option of the moves chosen, its teeth as laid)
        berths = {leg: m for lane, lg in E.lanes for leg, m in zip(lg, E.B[lane][state[lane][1]][0])}
        off = sorted(l_ for l_ in names if sr.move_sig(berths[l_]) != sr.move_sig(choice[l_]))
        if off:
            fails.append(f'the fixture: the ends model\'s state is not the berths chosen for {off}')
            return
        plan, tracks, nets = whole_ends.state_plan(E, state)
        if plan['chi'] != want:
            fails.append(f'the fixture reads pair chirality {plan["chi"]}, written for {want:+d}')
        # the berths laid by the fanout engine, as the destination pass lays them -- every one as asked, or the board
        # is not the state's ends and nothing below compares what it means to
        with quiet():
            _got, audit, ok = ffp.fanout_once(laid, names, berths, st['dst_pad'], st['dref'], st['byname'], src)
        missed = sorted(l_ for l_ in names if not (audit.get(l_) or {}).get('exact'))
        if missed or not ok:
            fails.append(f'the fixture: the fanout engine laid {missed or "every berth"} '
                         f'{"not as asked" if missed else "with a DRC violation or a failed net"}')
            return
        json.dump(plan, open(laid[:-len('.kicad_pcb')] + '.plan.json', 'w'))
        with quiet():
            real, _g = bd.setup(laid, nets, 'U5', lambda *a: None, pairs=True)
        # (built as solve_check builds it: the board before its berths, their runs and vias in memory, the run's config
        # read off that board)
        cand = whole_ctx.candidate(pcb, nets, 'U5', plan, tracks, cfg=bd.run_config(pcb, src), board=src)
        if cand is None:
            fails.append('candidate() gave no bench')
            return
        # 1. the fields
        r6 = lambda v: tuple(round(float(x), 6) for x in v)
        for f_ in ('pairs', 'tooth_layer', 'dest_layer', 'pair_layers', 'src_ref', 'dest_cut', 'chi'):
            a, b = getattr(cand, f_), getattr(real, f_)
            if f_ == 'pairs':
                a, b = {k: tuple(v) for k, v in a.items()}, {k: tuple(v) for k, v in b.items()}
            if a != b or (f_ in ('tooth_layer', 'dest_layer') and list(a) != list(b)):
                fails.append(f'1. {f_}: candidate {a} != setup {b}')
        for f_ in ('tooth_dir', 'stub_dir'):
            a = {k: r6(v) for k, v in getattr(cand, f_).items()}
            b = {k: r6(v) for k, v in getattr(real, f_).items() if k in a}
            if a != b:
                fails.append(f'1. {f_}: candidate {a} != setup {b}')
        ends_c = {k: (r6(v[0]), r6(v[1]), v[2]) for k, v in cand.ends.items()}
        ends_r = {k: (r6(v[0]), r6(v[1]), v[2]) for k, v in real.ends.items() if k in ends_c}
        if ends_c != ends_r or set(cand.ends) != set(real.ends):
            fails.append(f'1. ends: candidate {ends_c} != setup {ends_r}')
        pe_c = {k: tuple(tuple(r6(p) for p in e) for e in v) for k, v in cand.pair_ends.items()}
        pe_r = {k: tuple(tuple(r6(p) for p in e) for e in v) for k, v in real.pair_ends.items()}
        if pe_c != pe_r:
            fails.append(f'1. pair_ends: candidate {pe_c} != setup {pe_r}')
        cf = lambda c: (c.track_width, c.clearance, c.via_size, c.via_drill, c.grid_step, c.hole_to_hole_clearance,
                        c.board_edge_clearance, tuple(c.layers))
        if cf(cand.cfg) != cf(real.cfg):
            fails.append(f'1. cfg: candidate {cf(cand.cfg)} != setup {cf(real.cfg)}')
        rc_, rr_ = runs(cand.base_segments), runs(real.base_segments)
        if rc_ != rr_:
            name_ = {i: n.name for i, n in pcb.nets.items()}
            fmt_ = lambda r: f'{name_.get(r[0], r[0])} {r[1]} {r[6]:.3f}..{r[7]:.3f} along ({r[3]:+.2f}, {r[4]:+.2f})'
            fails.append(f'1. copper: {len(rc_)} candidate runs vs {len(rr_)} laid; candidate only: '
                         f'{[fmt_(r) for r in sorted(set(rc_) - set(rr_))]}; laid only: '
                         f'{[fmt_(r) for r in sorted(set(rr_) - set(rc_))]}')
        via = lambda v: (round(v.x, 4), round(v.y, 4), round(v.size, 4), v.net_id)
        vc_, vr_ = sorted(map(via, cand.base_vias)), sorted(map(via, real.base_vias))
        if vc_ != vr_:
            fails.append(f'1. vias differ: candidate only {sorted(set(vc_) - set(vr_))}, laid only '
                         f'{sorted(set(vr_) - set(vc_))}' + (' (one stands twice)' if set(vc_) == set(vr_) else ''))
        pth = lambda c: {k: [r6(p) for p in v] for k, v in c.paths.items()}
        pc_, pr_ = pth(cand), pth(real)
        if pc_ != pr_:
            fails.append(f'1. taut paths differ on {sorted(k for k in set(pc_) | set(pr_) if pc_.get(k) != pr_.get(k))}')
        # 2. the frames
        with quiet():
            Fc, Fr = whole_frame.build(cand, 'U5'), whole_frame.build(real, 'U5')
        for f_ in ('M', 'launch', 'final', 'cls', 'Hk', 'H0'):
            a, b = getattr(Fc, f_), getattr(Fr, f_)
            if (a != b) if not isinstance(a, float) else abs(a - b) > 1e-9:
                fails.append(f'2. frame {f_}: candidate {a} != setup {b}')
        for f_ in ('st', 'land'):
            a = {k: r6(v) for k, v in getattr(Fc, f_).items()}
            b = {k: r6(v) for k, v in getattr(Fr, f_).items()}
            if a != b:
                fails.append(f'2. frame {f_}: candidate {a} != setup {b}')
        a = {k: [r6(p) for p in v] for k, v in Fc.mid.items()}
        b = {k: [r6(p) for p in v] for k, v in Fr.mid.items()}
        if a != b:
            fails.append(f'2. frame mid (the references) differ on {sorted(k for k in a if a[k] != b.get(k))}')
        # 3. the solve: no escape's face zones on either (solve_check's candidate, and the round's solve reading a plain
        # fanout's sidecar), the check asked as solve_check asks it
        cand.escape = real.escape = False
        with quiet():
            vc = whole_solve.solve(cand, 'U5', front={}, check=True, core=True)
            vr = whole_solve.solve(real, 'U5', front={}, check=True, core=True)
            Jc = whole_solve.solve(cand, 'U5', front={})
            Jr = whole_solve.solve(real, 'U5', front={})
        # (the verdict, and what the check measured before it searched: each lane's rooms, its via ends, the layers the
        # copper beside an end bans)
        verdict = lambda v: None if v is None else dict(
            {k: v.get(k) for k in ('plan', 'status', 'core', 'nochange', 'vend', 'vban')},
            rooms={n: [round(float(x), 6) for x in r] for n, r in (v.get('rooms') or {}).items()})
        if verdict(vc) != verdict(vr):
            fails.append(f'3. the check: candidate {verdict(vc)} != setup {verdict(vr)}')
        keep = lambda J: None if J is None else {k: J[k] for k in ('cross', 'changes', 'layers', 'entry', 'end', 'tend')}
        if keep(Jc) != keep(Jr):
            fails.append(f'3. the solve on the two benches differs: candidate {keep(Jc)} != setup {keep(Jr)}')
        # 4. a run of the pair alone (every lane a pair): setup reads the sidecar, which keys the pair by its legs
        pr_nets = ['D_P', 'D_N']
        with quiet():
            real1, _g1 = bd.setup(laid, pr_nets, 'U5', lambda *a: None, pairs=True)
        cand1 = whole_ctx.candidate(pcb, pr_nets, 'U5', plan, tracks, cfg=bd.run_config(pcb, src), board=src)
        if real1.plan is None:
            fails.append('4. the pair alone: setup dropped the plan sidecar')
        for f_ in ('tooth_layer', 'dest_layer', 'pair_layers', 'tooth_dir', 'stub_dir'):
            a = {k: (r6(v) if f_.endswith('_dir') else v) for k, v in getattr(cand1, f_).items()}
            b = {k: (r6(v) if f_.endswith('_dir') else v) for k, v in getattr(real1, f_).items() if k in a}
            if a != b:
                fails.append(f'4. the pair alone, {f_}: candidate {a} != setup {b}')
        if {k: (r6(v[0]), r6(v[1])) for k, v in cand1.ends.items()} != \
                {k: (r6(v[0]), r6(v[1])) for k, v in real1.ends.items() if k in cand1.ends}:
            fails.append('4. the pair alone: ends differ')
        if not fails:
            print(f'   chirality {want:+d}: {len(nets)} nets, {len(E.lanes)} lanes, {len(tracks)} escape runs; check '
                  f'{ {k: (vc or {}).get(k) for k in ("plan", "status", "crossings")} }; frame lanes {Fc.M}')


def report(fails):
    print('=' * 60)
    print('the ends model\'s in-memory bench is the round\'s')
    if fails:
        for f in fails:
            print('FAIL:', f)
        return 1
    print('PASS: the fields the frame and the solve read, the frame, the check and the solve agree with setup\'s '
          'reading of the laid board')
    return 0


if __name__ == '__main__':
    sys.exit(main())
