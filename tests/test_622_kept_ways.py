#!/usr/bin/env python3
"""The ways out of the balls a plan walls in are laid on a copy of its bench, the loop runs again on it, and the lay
takes them up (awx/kept_ways.py: lanes_copper, bus_parts, find, write, take_up, said; awx/whole_route.py: the loop's
keep_ways, ways_up, chain's lay).

  python3 tests/test_622_kept_ways.py

An array's ball on no bus net leaves by its own stub, and a bundle of lanes run past the end of that stub walls it
in. The lay found that once the lanes were copper and could lift one lane at a time; a way out that ran between a
lane and a fixed pad a few hundredths of a millimetre short of a track's room shipped walled in. The plan is now
asked first: its lanes as copper on the bench, the step's own cage test, and for a ball they wall in its way out
between them written into a copy of the bench as copper of the ball's own net -- on which the loop is run again,
and from which the lay's board has the way taken up again.

The parts that need no routed board, on a small board of the repository and made-up plans:

1. lanes_copper: a single's pieces a track wide on its net, its changes of layer vias, each named for its lane; a
   pair one track along its line as wide as its two legs stand, named for no lane (the pairs are held: walls);
   a lane of no net of the board left out;
2. bus_parts: the destination's parts, and the part with the most pads on the bus's nets beside them;
3. write: the bench with the ways on it, the ways' record beside it, the fanout's plan sidecar carried, the bench
   itself untouched;
4. take_up: every piece of the ways taken up again, the board's copper what the bench's was; a piece not found
   again stops the run, and says so;
5. ways laid on a bench that has ways already are recorded with those: both are taken up together;
6. a plan with no lane walls nothing in: no way, nothing written;
7. the loop asks only where it can run again on the way's bench: not on a solve's relayed bench, not among
   negotiated buses, not past KEEP_WAYS benches one inside another; a plan that walls nothing in changes nothing;
8. ways_up: nothing where the bench has no ways; the ways taken up where it has; a board without them stops the run;
9. the lay and the pack take the ways up from the boards they write.
"""
import contextlib
import inspect
import io
import json
import os
import shutil
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)
BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
if not os.path.isfile(BOARD):
    raise SystemExit(f'BROKEN TEST: no board {BOARD}')

import rules as _rules  # noqa: E402
_rules.install_defaults()
import awx_settings  # noqa: E402
import kept_ways as kw  # noqa: E402
import whole_route as wr  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


def board(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


pcb = board(BOARD)
ids = {n.name: i for i, n in pcb.nets.items()}
TRACK, VIA, DRILL = 0.1, 0.25, 0.15
NETS = ['INT1', 'INT2', 'DP1_P', 'DP1_N']

# 1. the plan's lanes as copper
plan = {'lanes': {
    'INT1': {'pieces': [[1.0, 1.0, 2.0, 1.0, 'F.Cu'], [2.0, 1.0, 2.5, 1.5, 'B.Cu']], 'vias': [[2.0, 1.0]]},
    'INT2': {'pieces': [[1.0, 2.0, 3.0, 2.0, 'F.Cu']], 'vias': []},
    'DP1': {'pieces': [[1.0, 3.0, 3.0, 3.0, 'F.Cu']], 'vias': [[3.0, 3.0]],
            'ends': [{'handover': [[1.0, 2.85], [1.0, 3.15]]}, {'handover': [[3.0, 2.88], [3.0, 3.12]]}]},
    'NOT_A_NET': {'pieces': [[0.0, 0.0, 1.0, 0.0, 'F.Cu']], 'vias': []}}}
segs, vias, soft = kw.lanes_copper(pcb, plan, NETS, TRACK, VIA, DRILL)
by_net = {}
for s in segs:
    by_net.setdefault(s.net_id, []).append(s)
check('1. a single\'s pieces a track wide on its own net, each named for its lane',
      [round(s.width, 6) for s in by_net.get(ids['INT1'], [])] == [TRACK, TRACK]
      and {soft.get(id(s)) for s in by_net.get(ids['INT1'], [])} == {'INT1'}
      and [s.layer for s in by_net[ids['INT1']]] == ['F.Cu', 'B.Cu'], sorted(by_net))
v1 = [v for v in vias if v.net_id == ids['INT1']]
check('1. its change of layer a via of the run\'s size through the board, named for its lane',
      len(v1) == 1 and (v1[0].x, v1[0].y, v1[0].size, v1[0].drill) == (2.0, 1.0, VIA, DRILL)
      and v1[0].layers == ['F.Cu', 'B.Cu'] and soft.get(id(v1[0])) == 'INT1', [(v.x, v.y, v.size) for v in v1])
pr = by_net.get(ids['DP1_P'], [])
check('1. a pair one track along its line, as wide as its two legs stand and a track, named for no lane',
      len(pr) == 1 and abs(pr[0].width - (0.30 + TRACK)) < 1e-9 and id(pr[0]) not in soft
      and not by_net.get(ids['DP1_N']), [(s.width, s.net_id) for s in pr])
vp = [v for v in vias if v.net_id == ids['DP1_P']]
check('1. ...and its change one via as wide, named for none',
      len(vp) == 1 and abs(vp[0].size - (VIA + 0.30)) < 1e-9 and id(vp[0]) not in soft, [(v.size) for v in vp])
check('1. a lane of no net of the board is left out', len(segs) == 4 and len(vias) == 2, (len(segs), len(vias)))

# 2. the bus's parts
only = next(iter(pcb.footprints))
check('2. the part with the most pads on the bus\'s nets, beside the destination\'s',
      kw.bus_parts(pcb, [ids[n] for n in NETS], 'NOT_ON_THE_BOARD') == [only]
      and kw.bus_parts(pcb, [ids[n] for n in NETS], only) == [only], kw.bus_parts(pcb, [ids[n] for n in NETS], only))

# 3..5 the ways written and taken up
F = ids['FOREIGN']
way = lambda y, n=2: {'ball': f'{only}.1', 'net': 'FOREIGN', 'net_id': F, 'lanes': ['INT1'], 'short': 0.4, 'end': 'edge',  # noqa: E731
                      'length': 1.0 * n, 'vias': [],
                      'tracks': [{'start': (5.0 + k, y), 'end': (6.0 + k, y), 'width': 0.15, 'layer': 'F.Cu', 'net_id': F}
                                 for k in range(n)]}
with tempfile.TemporaryDirectory() as td:
    bench = os.path.join(td, 'bench.kicad_pcb')
    shutil.copyfile(BOARD, bench)
    json.dump({'ends': {}}, open(os.path.join(td, 'bench.plan.json'), 'w'))
    before = open(bench).read()
    out = os.path.join(td, 'with_ways.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        kw.write(bench, out, [way(20.0)])
    b0, b1 = board(bench), board(out)
    added = [s for s in b1.segments if abs(s.width - 0.15) < 1e-9 and s.net_id == F and abs(s.start_y - 20.0) < 1e-6]
    check('3. the bench with the ways on it: each piece a track of the ball\'s own net',
          len(b1.segments) == len(b0.segments) + 2 and len(added) == 2, (len(b0.segments), len(b1.segments)))
    rec = json.load(open(kw.ways_file(out)))
    check('3. the ways\' record beside it, and the fanout\'s plan sidecar carried',
          kw.ways_file(out) == os.path.join(td, 'with_ways.ways.json') and len(rec['ways']) == 1
          and rec['bench'] == os.path.abspath(bench) and os.path.isfile(os.path.join(td, 'with_ways.plan.json')), rec)
    check('3. the bench itself untouched, no record beside it',
          open(bench).read() == before and not os.path.isfile(kw.ways_file(bench)))
    # 5. ways on a bench that has ways already
    out2 = os.path.join(td, 'with_more.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        kw.write(out, out2, [way(21.0, 1)])
    rec2 = json.load(open(kw.ways_file(out2)))
    b2 = board(out2)
    check('5. ways laid on a bench with ways already are recorded with those',
          len(rec2['ways']) == 2 and len(b2.segments) == len(b0.segments) + 3, len(rec2['ways']))
    laid2 = os.path.join(td, 'laid2.kicad_pcb')
    shutil.copyfile(out2, laid2)
    check('5. ...and all are taken up together', kw.take_up(laid2, kw.ways_file(out2)) == 3
          and len(board(laid2).segments) == len(b0.segments))
    # 4. taken up again
    laid = os.path.join(td, 'laid.kicad_pcb')
    shutil.copyfile(out, laid)
    n_up = kw.take_up(laid, kw.ways_file(out))
    b3 = board(laid)
    check('4. every piece of the ways taken up again: the board\'s copper what the bench\'s was',
          n_up == 2 and len(b3.segments) == len(b0.segments)
          and not [s for s in b3.segments if abs(s.width - 0.15) < 1e-9 and s.net_id == F], (n_up, len(b3.segments)))
    why = None
    try:
        kw.take_up(laid, kw.ways_file(out))             # (taken up already: none is found again)
    except SystemExit as e:
        why = str(e.code)
    check('4. a piece not found again stops the run, and says so',
          why is not None and '2 of the 2 track(s)' in why and 'not found again' in why, why)
    check('4. ...the board left as it was', len(board(laid).segments) == len(b0.segments))

    # 8. ways_up
    said = []
    env = dict(awx_settings.environ())
    lg = os.path.join(td, 'up.log')
    wr.ways_up(bench, laid, lg, env, said.append)
    check('8. ways_up: nothing where the bench has no ways', not said and not os.path.isfile(lg), said)
    laid4 = os.path.join(td, 'laid4.kicad_pcb')
    shutil.copyfile(out, laid4)
    wr.ways_up(out, laid4, lg, env, said.append)
    check('8. ...the ways taken up where it has, and said',
          len(board(laid4).segments) == len(b0.segments) and len(said) == 1 and 'taken up' in said[0], said)
    err = None
    try:
        wr.ways_up(out, laid4, lg, env, said.append)
    except RuntimeError as e:
        err = str(e)
    check('8. ...a board without them stops the run', err is not None and 'not taken up' in err, err)

    # 6. a plan with no lane
    with awx_settings.given({**awx_settings.environ(), 'BENCH': bench, 'NETS': ','.join(NETS), 'DEST': only}):
        got = kw.find(bench, {'lanes': {}}, NETS, only)
    check('6. a plan with no lane walls nothing in: no way', got == ([], []), got)

# 7. where the loop asks
src = inspect.getsource(wr.loop)
k0 = src.index('def keep_ways(')
body = src[k0:src.index('def smooth_passes(')]
check('7. the loop asks only where it can run again on the way\'s bench: no relayed bench, no negotiated buses, '
      'KEEP_WAYS benches one inside another at most',
      "if env.get('BENCH0') or env.get('JOINT_DIR') or depth >= KEEP_WAYS:\n            return" in body
      and wr.KEEP_WAYS >= 1, wr.KEEP_WAYS)
check('7. a plan that walls nothing in changes nothing: no way said, nothing run',
      "if not said_:\n            return" in body and body.index('if not said_:') < body.index('loop(st['))
check('7. the loop run again on the bench with the ways, from the round\'s solve, its flips, cuts and history',
      "BENCH=wb" in body and "SEED_FLIPS=st['flips']" in body and "SEED_CUTS=st['cuts']" in body
      and "SEED_HIST=st['hist']" in body and "KEPT_WAYS=str(depth + 1)" in body)
check('7. no plan there: this round\'s plan stands as it was, and the log says so',
      'this round\'s plan stands' in body and body.index('if rc_w != 0:') < body.index("shutil.copyfile(os.path.join(ld, 'plan.json'), plan)"))
check('7. the plan that passes is asked before it is said to pass',
      "keep_ways(i, plan)\n            log(f\"=== the plan passes: {plan}\")" in src)

# 9. the lay and the pack
ch = inspect.getsource(wr.chain)
check('9. the lay takes the ways up from the board it writes, before it is graded',
      ch.index("ways_up(bench, seq,") < ch.index("res_ = graded(r, d, seq, lanes, tag)"))
check('9. ...and so does the pack', "ways_up(laid['bench'], out_p," in ch
      and ch.index("ways_up(laid['bench'], out_p,") < ch.index("res_p = graded(r, dp, out_p, res['lanes'])"))

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
