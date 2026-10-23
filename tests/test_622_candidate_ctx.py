#!/usr/bin/env python3
"""The ends model asks the WHOLE SOLVE of its own escape ends before any is laid (whole_ends.solve_check): the bench
those ends make is built in memory (whole_ctx.candidate) and given to whole_solve.solve under its strict rules. That is
only the solve's verdict if the in-memory bench IS the one the round's solve will read off the laid board.

  python3 tests/test_622_candidate_ctx.py

On a board of its own -- two 5 x 5 ball arrays at 0.8 mm pitch, the source U1 west of the destination U5 (the canonical
frame), four copper layers, every ball netted, five bus nets between them (one a differential pair), both arrays
dog-boned (dogbone_arrays) -- the ends model's starting state on its escape menus is turned into a plan
(whole_ends.state_plan) and, two ways, into a bench: candidate() in memory, and braid.setup on the board with the
state's escapes LAID (escape_plan.lay) and its plan sidecar beside it, as the round's solve reads it. Pinned:

1. every field the whole frame and the solve read is the same: the lanes and pairs, each lane's ends (a pair's at its
   legs' midpoints) and its legs' ends, the layers (a pair's P leg's, its N leg's beside them), the escape directions,
   the source part, the pair frame, the destination's cut, the routing config, the copper (the board's own and the
   escape runs, segment for segment), and each lane's TAUT PATH -- round the arrays' dog-bone vias, and a 0402 beside
   the trunk whose pads the solve's built-in via cuts read the lanes' references against;
2. the whole frame built on each (whole_frame.build) is the same: its lanes, launch and final orders, ring classes,
   starts, landings, handoffs, and each lane's reference (mid);
3. whole_solve.solve(check=True) gives each the same verdict -- and its solve, run as the round runs it, the same plan;
4. a run of the pair alone, every lane a pair, reads the same sidecar (keyed by the pair's legs) and the same fields --
   an all-pair bus's setup once found its member's name in none of the sidecar's keys and dropped the plan, reading
   the escape directions off copper (sdi_mipi_conv's MIPI bus to J4: a south tooth read west, refused every round).

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
os.environ.update(BRAID_PAIRS='1', PLAN_PAIRS='1', ROUTE_LAYERS='F.Cu,B.Cu,In1.Cu,In2.Cu', DEST='U5')
for k_ in ('BENCH', 'NETS', 'FEEDBACK', 'INCREMENTAL', 'FANOUT_ESCAPE', 'STAGE_CACHE', 'TAUT_MEMO'):
    os.environ.pop(k_, None)

import rules as _rules  # noqa: E402
_rules.install(_rules.Rules(clearance=0.09, track=0.15, via_size=0.45, via_drill=0.3, fan_track=0.12,
                            hole_to_hole=0.25, edge_clearance=0.2, source='test'))
import braid as bd  # noqa: E402
import dogbone_arrays as dga  # noqa: E402
import escape_plan as ep  # noqa: E402
import whole_ctx  # noqa: E402
import whole_ends  # noqa: E402
import whole_frame  # noqa: E402
import whole_solve  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

N, PITCH, GAP = 5, 0.8, 7.0
# net: (U1 ball, U5 ball) -- the balls' moment about the source-to-destination axis positive: pair chirality +1; S2
# leaves the source's top row for the destination's bottom one and S3 the other way, so the lanes cross
BUS = {'D_P': (18, 16), 'D_N': (23, 21), 'S1': (13, 11), 'S2': (3, 22), 'S3': (24, 1)}


def board():
    """two dog-boneable arrays, U1 at x 0 and U5 at x GAP + 4 pitches: the bus's balls on BUS's nets, every other ball
    a net of its own"""
    nets, n_of = ['(net 0 "")'], {}
    for nm in BUS:
        n_of[nm] = len(nets)
        nets.append(f'(net {len(nets)} "{nm}")')
    used = {(ref, b): nm for nm, (a, z) in BUS.items() for ref, b in (('U1', a), ('U5', z))}
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
    with tempfile.TemporaryDirectory() as td:
        raw, base, laid = (os.path.join(td, f) for f in ('raw.kicad_pcb', 'base.kicad_pcb', 'laid.kicad_pcb'))
        open(raw, 'w').write(board())
        with contextlib.redirect_stdout(io.StringIO()):
            dga.dogbone_file(raw, base, ['U1', 'U5'], log=lambda *a: None)
            pcb = parse_kicad_pcb(base)
        names = list(BUS)
        st = ep.plan_state(pcb, names, 'U1', 'U5', log=lambda *a: None)
        st['base'] = base
        E = whole_ends.Ends(st, src_free=True)
        state = E.start()
        plan, tracks, nets = whole_ends.state_plan(E, state)
        if plan['chi'] != 1:
            fails.append(f'the fixture reads pair chirality {plan["chi"]}: the test is written for +1')
        moves = []
        for lane, lg in E.lanes:
            for k_, opts in ((0, E.T[lane]), (1, E.B[lane])):
                moves += list(zip(lg, opts[state[lane][k_]][0]))
        with contextlib.redirect_stdout(io.StringIO()):
            ok, faults = ep.lay(base, laid, moves, st['byname'], log=lambda *a: None)
        json.dump(plan, open(laid[:-len('.kicad_pcb')] + '.plan.json', 'w'))
        with contextlib.redirect_stdout(io.StringIO()):
            real, _g = bd.setup(laid, nets, 'U5', lambda *a: None, pairs=True)
        cand = whole_ctx.candidate(pcb, nets, 'U5', plan, tracks, board=base)
        if cand is None:
            fails.append('candidate() gave no bench')
            return report(fails)
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
        seg = lambda s: (round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4),
                         round(s.width, 4), s.layer, s.net_id)
        und = lambda s: tuple(sorted([s[:2], s[2:4]])) + s[4:]
        sc = sorted(und(seg(s)) for s in cand.base_segments)
        sr_ = sorted(und(seg(s)) for s in real.base_segments)
        if sc != sr_:
            fails.append(f'1. copper: {len(sc)} candidate segments vs {len(sr_)} laid, '
                         f'{len(set(sc) ^ set(sr_))} differ')
        via = lambda v: (round(v.x, 4), round(v.y, 4), round(v.size, 4), v.net_id)
        if sorted(map(via, cand.base_vias)) != sorted(map(via, real.base_vias)):
            fails.append('1. vias differ')
        pth = lambda c: {k: [r6(p) for p in v] for k, v in c.paths.items()}
        pc_, pr_ = pth(cand), pth(real)
        if pc_ != pr_:
            fails.append(f'1. taut paths differ on {sorted(k for k in set(pc_) | set(pr_) if pc_.get(k) != pr_.get(k))}')
        # 2. the frames
        with contextlib.redirect_stdout(io.StringIO()):
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
        # 3. the solve
        with contextlib.redirect_stdout(io.StringIO()):
            vc = whole_solve.solve(cand, 'U5', front={}, check=True)
            vr = whole_solve.solve(real, 'U5', front={}, check=True)
            Jc = whole_solve.solve(cand, 'U5', front={})
            Jr = whole_solve.solve(real, 'U5', front={})
        if (vc or {}).get('plan') != (vr or {}).get('plan'):
            fails.append(f'3. the check: candidate {vc} != setup {vr}')
        keep = lambda J: None if J is None else {k: J[k] for k in ('cross', 'changes', 'layers', 'entry', 'end', 'tend')}
        if keep(Jc) != keep(Jr):
            fails.append('3. the solve on the two benches differs')
        # 4. a run of the pair alone (every lane a pair): setup reads the sidecar, which keys the pair by its legs
        pr_nets = ['D_P', 'D_N']
        with contextlib.redirect_stdout(io.StringIO()):
            real1, _g1 = bd.setup(laid, pr_nets, 'U5', lambda *a: None, pairs=True)
        cand1 = whole_ctx.candidate(pcb, pr_nets, 'U5', plan, tracks, board=base)
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
            print(f'   {len(nets)} nets, {len(E.lanes)} lanes, {len(tracks)} escape runs; check {vc}; frame lanes '
                  f'{Fc.M}')
    return report(fails)


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
