#!/usr/bin/env python3
"""synth_ladder.py -- run the chain over generated cases and grade it
against their KNOWN ANSWERS (#622).

    python3 synth_ladder.py --batch first          # 3 cases at K=8, the proof
    python3 synth_ladder.py --batch b1             # patterns x K, open arrays
    python3 synth_ladder.py --batch b2             # the same, interiors CLOSED
    python3 synth_ladder.py --batch b3             # the corridor-obstacle ladder
    python3 synth_ladder.py --batch b1 --only sorted_k8,blocks_k15
    python3 synth_ladder.py --batch b1 --regrade   # no build, no chain

Per case: `synth_bus.py` writes the board and its truth, `make_bench.py`
prepares it the way it prepares a corpus pair (source fanned out, DRC
floor stamped, coherent ladder written), `chain_k.sh` runs plan + fanout +
braid, and the result is graded against the KNOWN ANSWER:

    PLANNER gap  = the braid's plan-implied count   - the known optimum
    BRAID   gap  = the routed vias                  - the plan-implied count
    DP      gap  = the routed vias                  - the known optimum
    DETOUR       = routed copper mm / the straight-line lower bound

plus `open` (completion), `drc`, and `inband/offered` -- the braid's own
first-attempt in-band lane count, read from its `lanes: a/b routed` lines,
because "all lanes in band" is the objective the planner is actually
optimising and a case can route perfectly while leaving it.

The optimum is re-derived FROM THE BOARD (`truth_from_board`) rather than
trusted from the generator's sidecar, so a case whose ladder does not hand
the chain the K that was asked for -- `coherent_nets` counts whole RIVERS,
so it can return fewer -- is still graded against its own real problem;
`k_asked` and `k_real` are printed side by side and a disagreement is a
harness bug, not a routing result.

THE THREE COLUMNS THAT SAY WHETHER A NUMBER MEANS ANYTHING
----------------------------------------------------------
* `lb` / `opt` / `dp`: the LIS lower bound, the whole-lane optimum, and
  the exact optimum over pages AND mid-channel changes. `dp` is the one
  to grade against; `opt` is blank when the crossing graph has an odd
  cycle and `dp` is blank past the DP's K cap.
* `thru`: lanes with copper INSIDE an array. The optimum is
  channel-confined, so a case with `thru > 0` had a cheaper topology
  available than the model describes and its negative gap is NOT a win.
  The b1/b2 pair exists exactly to separate this: same cases, b2 with the
  array interiors closed by fat unused balls.
* `slot_cap`: on an obstacle case, how many lanes can physically pass the
  obstruction by counting. Without it `open=4` cannot be read -- it is a
  router defect if the lanes fit and a correct refusal if they do not.

Times are printed as an OBSERVATION. Nothing here budgets on a clock.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import synth_bus as sy                                        # noqa: E402

PY = sys.executable
SRC, DST = 'SU1', 'SD1'

# ---------------------------------------------------------------------------
# The batches. A case is geometry + a pin pattern; nothing here names a
# board, a net or a face, and every field is a generator argument.
# ---------------------------------------------------------------------------


def case(k, pattern, **kw):
    c = dict(k=k, pattern=pattern, seed=0, blocks=2, inversions=None,
             cols=4, rows=None, depth=1, gap=12.0, dst_rot=0.0, caps=0,
             pad=0.4, pad_inner=None, margin_y=6.0, fanout_layers='F.Cu',
             obstacle_w=0.0, obstacle_h=0.0, obstacle_x=0.5, obstacle_y=0.0)
    c.update(kw)
    bits = [pattern, f'k{k}']
    for key, dflt in (('seed', 0), ('blocks', 2), ('gap', 12.0), ('depth', 1),
                      ('caps', 0), ('dst_rot', 0.0), ('cols', 4),
                      ('pad', 0.4), ('margin_y', 6.0)):
        if c[key] != dflt:
            bits.append(f'{key[:3]}{c[key]:g}' if isinstance(c[key], (int, float))
                        else f'{key[:3]}{c[key]}')
    if c['pad_inner']:
        bits.append(f'pi{c["pad_inner"]:g}')
    if c['obstacle_h']:
        bits.append(f'obs{c["obstacle_w"]:g}x{c["obstacle_h"]:g}'
                    + (f'y{c["obstacle_y"]:g}' if c['obstacle_y'] else '')
                    + (f'x{c["obstacle_x"]:g}' if c['obstacle_x'] != 0.5 else ''))
    if c['fanout_layers'] != 'F.Cu':
        bits.append('fb')
    c['tag'] = '_'.join(bits)
    return c


BATCHES = {
    # the proof: three planted optima at one K, every one exact
    'first': [case(8, 'sorted'), case(8, 'blocks'), case(8, 'interleave')],

    # the first real batch: the patterns crossed with K, plus the axes
    # that are geometry rather than pattern (gap, rotation, foreign
    # parts, escape depth, a two-layer source fanout)
    'b1': (
        [case(k, p) for k in (8, 15, 28)
         for p in ('sorted', 'blocks', 'interleave', 'riffle', 'reversed')]
        + [case(15, 'riffle', seed=s) for s in (1, 2)]
        + [case(15, 'blocks', blocks=3)]
        + [case(15, 'interleave', gap=6.0)]
        + [case(15, 'interleave', dst_rot=180.0)]
        + [case(15, 'interleave', caps=6)]
        + [case(15, 'interleave', depth=2)]
        + [case(15, 'interleave', fanout_layers='F.Cu,B.Cu')]
    ),
}

# b2 is b1 with the array interior CLOSED to F traffic, which is what makes
# the channel-confined optimum an optimum (see synth_bus --pad-inner, and
# the README section). Same cases, so the two tables are paired.
BATCHES['b2'] = [case(**{**{k: v for k, v in c.items() if k != 'tag'},
                         'pad_inner': 0.6}) for c in BATCHES['b1']]

# b3: the OBSTACLE ladder. A through-hole part sits mid-channel and gets
# taller, so the bus has to fan in past it through a shrinking slot at
# each end -- parallel bends at the lane pitch, which is the geometry the
# session-11 island-stack finding says goes infeasible. The optimum
# printed is the CLEAR-CHANNEL one, so the excess over it is the price of
# the obstruction, and `open` is whether the chain got through at all.
# Every case is `pad_inner=0.6` (the interior closed) so a lane cannot
# answer the obstacle by going through an array instead.
#
# Room past the obstacle, for K=15 (17 rows, margin 6): the outline is
# +-12.4 mm, so h=12 leaves 6.4 mm each side, h=18 leaves 3.4, h=20
# leaves 2.4 and h=22 leaves 1.4 -- against the 0.2 mm lane pitch, 15
# lanes need 3.0 mm of width split between the two slots.
BATCHES['b3'] = (
    [case(15, p, pad_inner=0.6, obstacle_w=2.0, obstacle_h=h)
     for p in ('sorted', 'interleave')
     for h in (12.0, 18.0, 20.0, 22.0)]
    # K=8 -- and the h=18 row is a NEGATIVE CONTROL, deliberately kept: the
    # K=8 article is shorter (10 rows), so an 18 mm blocker SEALS the
    # channel and `slot_cap` comes out 0. The chain leaves 8 nets open and
    # that is a correct refusal, not a defect. It is the proof that
    # `slot_cap` discriminates -- without a case where the answer is
    # "impossible", an `open` count cannot be read at all.
    + [case(8, 'interleave', pad_inner=0.6, obstacle_w=2.0, obstacle_h=h)
       for h in (12.0, 18.0)]
    + [case(15, 'blocks', pad_inner=0.6, obstacle_w=2.0, obstacle_h=18.0)]
    + [case(15, 'riffle', pad_inner=0.6, obstacle_w=2.0, obstacle_h=18.0)]
    # a WIDE obstacle at the same height: length of the squeeze, not depth
    + [case(15, 'interleave', pad_inner=0.6, obstacle_w=6.0, obstacle_h=18.0)]
    # and one OFF-CENTRE, so the two slots are unequal and the fan-in has
    # to be asymmetric -- an equal split is then the wrong answer
    + [case(15, 'interleave', pad_inner=0.6, obstacle_w=2.0, obstacle_h=18.0,
            obstacle_y=4.0)]
)


# ---------------------------------------------------------------------------
# The truth, re-derived from the built boards
# ---------------------------------------------------------------------------

def _axis(pcb, src, dst):
    """(u, v): the unit vector source -> destination, and its left normal.
    Read off the two arrays, so a rotated or mirrored pose is the same
    computation -- nothing here assumes the channel runs along +x."""
    a, b = pcb.footprints[src], pcb.footprints[dst]
    dx, dy = b.x - a.x, b.y - a.y
    n = math.hypot(dx, dy) or 1.0
    u = (dx / n, dy / n)
    return u, (-u[1], u[0])


# the bench's 0.1 mm process, which is what make_bench stamps as the DRC
# floor and what chain_k.sh grades at. Named here because the capacity
# bound below is arithmetic on them, not a guess.
TRACK, CLEAR, EDGE = 0.1, 0.1, 0.2


def slot_capacity(pcb, v, src, dst):
    """How many lanes can PHYSICALLY pass a corridor obstacle, by counting.

    Without this an obstacle case cannot be read: 4 open nets is a router
    defect if the lanes fit and a correct refusal if they do not, and the
    two look identical in the grade. So: take the obstacle's copper extent
    across the channel, the room left to the outline on each side, subtract
    the edge clearance and one clearance off the obstacle, and fit lanes at
    `TRACK + CLEAR` pitch -- n lanes need `n*TRACK + (n-1)*CLEAR`. Both
    copper layers are available in the slot (the obstacle blocks both, the
    slot beside it blocks neither), so the capacity is doubled.

    An UPPER bound on what any router could do, and deliberately generous:
    it ignores that a lane must also bend into the slot and back out, which
    is the very thing that goes infeasible. A case routed short of this
    number has room the chain did not use."""
    obs = [f for r, f in pcb.footprints.items()
           if r not in (src, dst)
           and any(p.drill and p.drill > 0 and not p.net_id for p in f.pads)]
    if not obs or not pcb.board_info.board_bounds:
        return None
    x0, y0, x1, y1 = pcb.board_info.board_bounds

    def pv(x, y):
        return x * v[0] + y * v[1]
    lo = hi = None
    for f in obs:
        for p in f.pads:
            rad = max(p.size_x, p.size_y) / 2
            q = pv(p.global_x, p.global_y)
            lo = q - rad if lo is None else min(lo, q - rad)
            hi = q + rad if hi is None else max(hi, q + rad)
    corners = [pv(a, b) for a in (x0, x1) for b in (y0, y1)]
    bmin, bmax = min(corners), max(corners)

    def fit(room):
        usable = room - EDGE - CLEAR
        if usable < TRACK:
            return 0
        return int((usable + CLEAR) // (TRACK + CLEAR))
    a, b = fit(lo - bmin), fit(bmax - hi)
    return {'room': (round(lo - bmin, 2), round(bmax - hi, 2)),
            'per_layer': (a, b), 'capacity': 2 * (a + b)}


def truth_from_board(bench, names, src=SRC, dst=DST):
    """The known answer for the nets the chain is ACTUALLY given, read off
    the prepared bench: each lane's source tooth (the free stub end and
    the layer its copper sits on) and its destination pad, ordered along
    the channel's cross-axis. Returns the crossing count, LIS, the lower
    bound, the optimum and the teeth."""
    from kicad_parser import parse_kicad_pcb
    import braid as te
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(bench)
        bn = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
        ends = te.endpoints(pcb, names, bn)
    u, v = _axis(pcb, src, dst)
    tooth, key_s, key_d = {}, {}, {}
    for nm in names:
        s_pt, t_pt, _ref = ends[nm]
        key_s[nm] = s_pt[0] * v[0] + s_pt[1] * v[1]
        key_d[nm] = t_pt[0] * v[0] + t_pt[1] * v[1]
        nid = bn[nm][0]
        # the tooth's layer: the layer of the source stub segment that
        # owns the free end (that is the layer the lane launches on)
        best, bl = None, 'F.Cu'
        for sg in pcb.segments:
            if sg.net_id != nid:
                continue
            for p in ((sg.start_x, sg.start_y), (sg.end_x, sg.end_y)):
                d = math.hypot(p[0] - s_pt[0], p[1] - s_pt[1])
                if best is None or d < best:
                    best, bl = d, sg.layer
        tooth[nm] = bl
    order_src = sorted(names, key=lambda n: key_s[n])
    order_dst = sorted(names, key=lambda n: key_d[n])
    edges = sy.crossing_edges(order_src, order_dst)
    # the berth is the chain's own choice; the OPTIMUM assumes the best
    # one available, which on these articles is a straight escape on the
    # array's own layer at no via
    berth = {n: 'F.Cu' for n in names}
    opt, _comps, how = sy.optimum(names, edges, tooth_layer=tooth, berth_layer=berth)
    lb, lis = sy.lower_bound(order_src, order_dst)
    # the EXACT answer: pages AND mid-channel layer changes, over the
    # crossing order the real end positions give. This is the number the
    # whole-lane `opt` cannot produce for a non-bipartite case, and it is
    # <= `opt` whenever both exist -- the driver asserts that below.
    dp, dp_how = sy.exact_dp(names, key_s, key_d, tooth_layer=tooth,
                             berth_layer=berth)
    if any(l != 'F.Cu' for l in tooth.values()):
        # a tooth already on B makes the LIS bound (which assumes every
        # lane starts and ends on F) inapplicable -- say so rather than
        # print a number that is not a bound
        lb = None
    mmlb = 0.0
    for nm in names:
        nid, net = bn[nm]
        ps = [p for p in net.pads]
        if len(ps) >= 2:
            mmlb += math.hypot(ps[0].global_x - ps[1].global_x,
                               ps[0].global_y - ps[1].global_y)
    # The two lines the channel-confined model assumes a lane never passes:
    # half a pitch beyond the deepest bus ball at each end. A lane with
    # copper past one of them reached its ball THROUGH the array, which is a
    # topology the permutation's crossing graph does not describe -- it can
    # un-cross a pair with no via at all, so the optimum stops being one.
    def proj(p):
        return p[0] * u[0] + p[1] * u[1]
    sp = [proj((p.global_x, p.global_y)) for nm in names
          for p in bn[nm][1].pads if p.component_ref == src]
    dp_ = [proj((p.global_x, p.global_y)) for nm in names
           for p in bn[nm][1].pads if p.component_ref == dst]
    # the line sits at the deepest bus ball's own copper edge plus a hug
    # clearance: an arrival stub that lands ON its ball stops at the ball
    # centre and never reaches it, so only copper that went PAST the ball
    # to approach from behind trips the count. (Half a pitch was too far:
    # the measured escape ran at exactly grid+pitch/2 and read as 0.)
    rad = max((max(p.size_x, p.size_y) / 2 for nm in names
               for p in bn[nm][1].pads), default=0.2)
    half = rad + 0.105
    assert dp is None or opt is None or dp <= opt, \
        f'exact {dp} above the whole-lane optimum {opt}: one of them is wrong'
    return {'xing': len(edges), 'lis': lis, 'lb': lb, 'opt': opt, 'how': how,
            'dp': dp, 'dp_how': dp_how, 'slot': slot_capacity(pcb, v, src, dst),
            'teeth_b': sum(1 for l in tooth.values() if l != 'F.Cu'),
            'mm_lb': mmlb, 'u': u,
            's_in': (min(sp) - half) if sp else None,
            'd_in': (max(dp_) + half) if dp_ else None}


def through_array(board, names, u, s_in, d_in):
    """How many of `names` have copper INSIDE either array -- past the lines
    `truth_from_board` computed. Printed as `thru`: a non-zero count is the
    warning that this case's routed number may legitimately beat its
    channel-confined optimum, and it is the mechanism the b1/b2 pair of
    tables exists to show."""
    from kicad_parser import parse_kicad_pcb
    if s_in is None or d_in is None:
        return ''
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    ids = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()
           if n.name.split('/')[-1] in names}
    hit = set()
    for s in pcb.segments:
        nm = ids.get(s.net_id)
        if nm is None:
            continue
        for p in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            q = p[0] * u[0] + p[1] * u[1]
            if q > d_in or q < s_in:
                hit.add(nm)
    return len(hit)


def vias_mm(board, names):
    """(vias, copper mm) on `names` for a written board."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    ids = {i for i, n in pcb.nets.items() if n.name.split('/')[-1] in names}
    mm = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in pcb.segments if s.net_id in ids)
    return sum(1 for v in pcb.vias if v.net_id in ids), mm


def plan_count(fo_board, names, dest):
    """The braid's PLAN-IMPLIED via count for the fanout board's plan --
    `judge_gate.grade_one`'s `c_sw`: the ends as laid, every page lane's
    `changes`, every swimmer's `swim_changes`, and `cross_vias`."""
    import braid as te
    sidecar = fo_board.replace('.kicad_pcb', '.plan.json')
    if not os.path.isfile(sidecar):
        return None, None
    plan = json.load(open(sidecar))
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        bp = te.plan_braid(fo_board, list(names), dest, plan)
    ends, _ = vias_mm(fo_board, names)
    ch = sw = xv = resid = 0
    for nm in names:
        b = bp.get(nm, {})
        xv += b.get('cross_vias', 0) or 0
        if b.get('page') is None:
            resid += 1
            sw += b.get('swim_changes') or 0
        else:
            ch += b.get('changes') or 0
    return ends + ch + sw + xv, resid


# ---------------------------------------------------------------------------
# The stages
# ---------------------------------------------------------------------------

def gen_and_bench(c, outdir, log=print):
    """synth_bus -> make_bench. Returns (bench path, real net list)."""
    raw = os.path.join(outdir, c['tag'] + '.kicad_pcb')
    bench = os.path.join(outdir, c['tag'] + '_b.kicad_pcb')
    argv = [PY, 'synth_bus.py', raw, '--k', str(c['k']), '--pattern', c['pattern'],
            '--seed', str(c['seed']), '--blocks', str(c['blocks']),
            '--cols', str(c['cols']), '--depth', str(c['depth']),
            '--gap', str(c['gap']), '--dst-rot', str(c['dst_rot']),
            '--caps', str(c['caps']), '--src', SRC, '--dst', DST,
            '--pad', str(c['pad']), '--margin-y', str(c['margin_y'])]
    if c['pad_inner']:
        argv += ['--pad-inner', str(c['pad_inner'])]
    if c['obstacle_h'] and c['obstacle_w']:
        argv += ['--obstacle-w', str(c['obstacle_w']),
                 '--obstacle-h', str(c['obstacle_h']),
                 '--obstacle-x', str(c['obstacle_x']),
                 '--obstacle-y', str(c['obstacle_y'])]
    if c['rows']:
        argv += ['--rows', str(c['rows'])]
    if c['inversions'] is not None:
        argv += ['--inversions', str(c['inversions'])]
    r = subprocess.run(argv, capture_output=True, text=True)
    if r.returncode:
        log(f'  {c["tag"]}: GENERATOR FAILED\n' + (r.stdout + r.stderr)[-600:])
        return None, []
    log('  ' + r.stdout.strip())
    for ext in ('.kicad_pcb', '.kicad_pro', '.ladder.txt'):
        with contextlib.suppress(FileNotFoundError):
            os.remove(bench.replace('.kicad_pcb', ext))
    argv = [PY, 'make_bench.py', raw, SRC, DST, bench,
            '--fanout-layers', c['fanout_layers']]
    r = subprocess.run(argv, capture_output=True, text=True)
    out = r.stdout + r.stderr
    if r.returncode or not os.path.isfile(bench):
        log(f'  {c["tag"]}: MAKE_BENCH FAILED (rc={r.returncode})\n' + out[-800:])
        return None, []
    for line in r.stdout.splitlines():
        if re.search(r'fanned out|DRC|ladder|refused', line):
            log('  ' + line.strip())
    return bench, coherent(bench, c['k'])


def coherent(bench, K):
    r = subprocess.run([PY, 'coherent_nets.py', str(K), f'--board={bench}'],
                       capture_output=True, text=True)
    return [n for n in r.stdout.strip().split(',') if n]


def run_chain(c, bench, tag, env_extra=None, log=print):
    env = dict(os.environ, BASE=bench, DEST=DST, PLAN_PAGES='1')
    env.update(env_extra or {})
    t0 = time.time()
    r = subprocess.run(['bash', 'chain_k.sh', tag, str(c['k'])],
                       capture_output=True, text=True, env=env)
    dt = time.time() - t0
    out = r.stdout + r.stderr
    for line in out.splitlines():
        if re.search(r'GRADE|NO BRAID|NO FANOUT|FLOW FRAME|Traceback', line):
            log('  ' + line.strip())
    return dt, out


# ---------------------------------------------------------------------------

def inband_lanes(log_path):
    """(lanes routed IN BAND on the first attempt, lanes offered), summed
    over the braid's corridors, from its own `lanes: a/b routed` lines.

    This is the braid's OWN objective -- "all lanes in band" -- and it is
    reported apart from the via count because a case can be routed
    perfectly and still have left the band, which is the chain telling us
    the plan it was handed did not fit."""
    if not os.path.isfile(log_path):
        return '', ''
    a = b = 0
    for line in open(log_path, errors='replace'):
        m = re.search(r'lanes: (\d+)/(\d+) routed', line)
        if m:
            a += int(m.group(1))
            b += int(m.group(2))
    return (a, b) if b else ('', '')


FIELDS = ['tag', 'k_asked', 'k_real', 'pattern', 'xing', 'lis',
          'lb', 'opt', 'dp', 'teeth_b', 'bench_vias', 'total_opt',
          'total_dp', 'thru',
          'plan', 'routed', 'open', 'drc', 'segs', 'resid',
          'inband', 'offered', 'slot_cap', 'slot_room',
          'planner_gap', 'braid_gap', 'total_gap', 'dp_gap',
          'mm', 'mm_lb', 'detour', 'sec', 'how', 'dp_how']


def grade(c, bench, tag, dt, outdir, log=print):
    K = c['k']
    names = coherent(bench, K)
    fo = f'{tag}_fo_k{K}.kicad_pcb'
    rt = f'{tag}_k{K}.kicad_pcb'
    row = {f: '' for f in FIELDS}
    row.update(tag=c['tag'], k_asked=K, k_real=len(names), pattern=c['pattern'],
               sec=round(dt, 1))
    if not names:
        row['how'] = 'NO NETS -- the ladder gave the chain nothing'
        return row
    t = truth_from_board(bench, names)
    bench_vias, _ = vias_mm(bench, names)
    row.update(xing=t['xing'], lis=t['lis'], lb=t['lb'], opt=t['opt'],
               dp=t['dp'], teeth_b=t['teeth_b'], bench_vias=bench_vias,
               mm_lb=round(t['mm_lb'], 1), how=t['how'], dp_how=t['dp_how'])
    if t['slot']:
        row['slot_cap'] = t['slot']['capacity']
        row['slot_room'] = '/'.join(str(r) for r in t['slot']['room'])
    if t['opt'] is not None:
        row['total_opt'] = t['opt'] + bench_vias
    if t['dp'] is not None:
        row['total_dp'] = t['dp'] + bench_vias
    if not os.path.isfile(rt):
        row['how'] = (f'NO ROUTED BOARD at {os.path.basename(rt)} -- the chain '
                      f'produced none (its known answer was lb={t["lb"]} '
                      f'exact={t["dp"]})')
        return row
    g = subprocess.run([PY, 'grade_k.py', rt, ','.join(names)],
                       capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+) segs=(\d+)', g)
    if not m:
        row['how'] = 'GRADE BROKEN: ' + g.strip()[-160:]
        return row
    row.update(open=int(m.group(1)), drc=int(m.group(2)),
               routed=int(m.group(3)), segs=int(m.group(4)))
    _, mm = vias_mm(rt, names)
    row['mm'] = round(mm, 1)
    row['thru'] = through_array(rt, names, t['u'], t['s_in'], t['d_in'])
    row['detour'] = round(mm / t['mm_lb'], 2) if t['mm_lb'] else ''
    row['inband'], row['offered'] = inband_lanes(f'{tag}_k{K}.log')
    pc, resid = plan_count(fo, names, DST)
    if pc is not None:
        row['plan'], row['resid'] = pc, resid
        row['braid_gap'] = row['routed'] - pc
        # the PLANNER gap is measured against the EXACT optimum where
        # there is one, and only falls back to the whole-lane answer
        # when K is past the DP's reach
        ref = row['total_dp'] if row['total_dp'] != '' else row['total_opt']
        if ref != '':
            row['planner_gap'] = pc - ref
    if row['total_opt'] != '':
        row['total_gap'] = row['routed'] - row['total_opt']
    if row['total_dp'] != '':
        row['dp_gap'] = row['routed'] - row['total_dp']
    return row


def print_table(rows, log=print):
    cols = ['tag', 'k_real', 'xing', 'lb', 'opt', 'dp', 'total_dp', 'plan',
            'routed', 'open', 'drc', 'thru', 'slot_cap', 'inband', 'offered',
            'planner_gap', 'braid_gap', 'dp_gap', 'detour', 'sec']
    w = {c: max(len(c), *(len(str(r.get(c, ''))) for r in rows)) for c in cols}
    log('  '.join(c.rjust(w[c]) if c != 'tag' else c.ljust(w[c]) for c in cols))
    for r in rows:
        log('  '.join((str(r.get(c, '')).rjust(w[c]) if c != 'tag'
                       else str(r.get(c, '')).ljust(w[c])) for c in cols))
    # a case with no answer AND a case that never routed are both excluded;
    # `.get(k, '')` is deliberate, because a row from a failed build carries
    # only a tag and a reason (that is how this summary once crashed)
    ok = [r for r in rows
          if r.get('total_dp', '') != '' and r.get('routed', '') != '']
    broke = [r for r in rows if r.get('routed', '') == '']
    if ok:
        conf = [r for r in ok if r.get('thru') == 0]
        log(f'\n{len(ok)} case(s) with an EXACT answer: '
            f'routed {sum(r["routed"] for r in ok)} vias against an optimum of '
            f'{sum(r["total_dp"] for r in ok)}; '
            f'{sum(1 for r in ok if r["dp_gap"] == 0)} exactly optimal, '
            f'{sum(1 for r in ok if r.get("open"))} with open nets, '
            f'{sum(1 for r in ok if r.get("drc"))} with DRC.')
        if conf:
            log(f'{len(conf)} of them CHANNEL-CONFINED (thru=0), where the '
                f'optimum really is one: routed {sum(r["routed"] for r in conf)} '
                f'against {sum(r["total_dp"] for r in conf)}, '
                f'{sum(1 for r in conf if r["dp_gap"] == 0)} exact. The rest had '
                'copper inside an array, so a cheaper topology than the model '
                'describes was available to them.')
        ib = [r for r in ok if r.get('offered', '') != '']
        if ib:
            log(f'In band on the first attempt (the braid\'s own objective): '
                f'{sum(r["inband"] for r in ib)}/{sum(r["offered"] for r in ib)} '
                f'lanes over {len(ib)} case(s); '
                f'{sum(1 for r in ib if r["inband"] < r["offered"])} case(s) '
                'left the band.')
    if broke:
        log(f'\n{len(broke)} case(s) produced NO ROUTED BOARD:')
        for r in broke:
            log(f'  {r.get("tag", "?")}: {r.get("how", "(no reason recorded)")}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--batch', default='first', choices=sorted(BATCHES))
    ap.add_argument('--outdir', default=None, help='default tmp/synth_<batch>')
    ap.add_argument('--only', help='comma-separated case tags')
    ap.add_argument('--regrade', action='store_true',
                    help='no generate, no bench, no chain -- grade what is there')
    ap.add_argument('--no-chain', action='store_true')
    ap.add_argument('--env', default='', help='extra chain env, K=V,K=V')
    a = ap.parse_args(argv)
    outdir = a.outdir or os.path.join('tmp', f'synth_{a.batch}')
    os.makedirs(outdir, exist_ok=True)
    cases = BATCHES[a.batch]
    if a.only:
        want = set(a.only.split(','))
        cases = [c for c in cases if c['tag'] in want]
        if not cases:
            raise SystemExit(f'--only {a.only}: no such case in batch {a.batch}')
    env_extra = dict(kv.split('=', 1) for kv in a.env.split(',') if kv)
    rows = []
    for c in cases:
        tag = os.path.join(outdir, c['tag'])
        bench = os.path.join(outdir, c['tag'] + '_b.kicad_pcb')
        print(f'=== {c["tag"]}  ({c["pattern"]}, K={c["k"]}, gap={c["gap"]}, '
              f'depth={c["depth"]}, rot={c["dst_rot"]}, caps={c["caps"]})')
        dt = 0.0
        if not a.regrade:
            bench, _ = gen_and_bench(c, outdir)
            if bench is None:
                r = {f: '' for f in FIELDS}
                r.update(tag=c['tag'], k_asked=c['k'], pattern=c['pattern'],
                         how='BUILD FAILED -- the generator or make_bench '
                             'refused; see the log above')
                rows.append(r)
                continue
            if not a.no_chain:
                for ext in ('.kicad_pcb', '.kicad_pro', '.plan.json', '.log'):
                    for stem in (f'{tag}_fo_k{c["k"]}', f'{tag}_k{c["k"]}'):
                        with contextlib.suppress(FileNotFoundError):
                            os.remove(stem + ext)
                dt, _ = run_chain(c, bench, tag, env_extra)
        rows.append(grade(c, bench, tag, dt, outdir))
    print()
    print_table(rows)
    tsv = os.path.join(outdir, 'ladder.tsv')
    with open(tsv, 'w', encoding='utf-8') as f:
        f.write('\t'.join(FIELDS) + '\n')
        for r in rows:
            f.write('\t'.join(str(r.get(k, '')) for k in FIELDS) + '\n')
    print(f'\nrows -> {tsv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
