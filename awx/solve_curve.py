#!/usr/bin/env python3
"""solve_curve.py -- does the pages-first CP-SAT converge, and how fast?

Re-solves an instance the CHAIN wrote (`PLAN_PAGES_DUMP=<dir>`, a binary
CpModelProto with its hint) under the chain's own parameters (4 workers,
interleaved search, deterministic time) but with a LONG budget, and prints
every improvement as (deterministic time, wall s, objective, bound,
swimmers) plus the final status -- the curve the chain's DET-40 stop sits on.
With interleaved deterministic search the run at budget B explores the same
sequence as the run at budget b < B for its first b units, so one long run
reads off the answer at every budget.

usage: solve_curve.py INSTANCE.pb [--det 640] [--workers 4] [--no-hint]
       [--lin 0|1|2] [--sym N] [--swim-cap K] [--min-swim]
  --swim-cap K   add sum(swimmers) <= K (the LEX phase B: the rest of the
                 objective with the swimmer count pinned)
  --min-swim     replace the objective by the swimmer count alone (the LEX
                 phase A: a CERTIFICATE of the fewest swimmers the menus admit)
"""
import json
import os
import sys
import time

from ortools.sat.python import cp_model

SCALE = 100          # pages_first.SCALE: cost units -> ints


def arg(name, default):
    if name in sys.argv:
        return sys.argv[sys.argv.index(name) + 1]
    return default


path = sys.argv[1]
det = float(arg('--det', '640'))
workers = int(arg('--workers', '4'))
meta = {}
jp = os.path.splitext(path)[0] + '.json'
if os.path.exists(jp):
    meta = json.load(open(jp, encoding='utf-8'))
from ortools.sat import cp_model_pb2
_pb = cp_model_pb2.CpModelProto()
with open(path, 'rb') as f:
    _pb.ParseFromString(f.read())
if '--no-hint' in sys.argv:
    _pb.ClearField('solution_hint')
m = cp_model.CpModel()
# this ortools build's CpModel.proto is a C++ helper: it takes the text form
m.proto.parse_text_format(str(_pb))
names = meta.get('names') or []
# the swimmer literals: pages_first names them w_<net> (the joined-key
# bools are w_<net>_<j>, two underscores)
swim_idx = {}
for i, v in enumerate(m.proto.variables):
    if v.name.startswith('w_') and v.name.count('_') == 1:
        swim_idx[v.name[2:]] = i
swim_lits = [m.get_bool_var_from_proto_index(i) for i in swim_idx.values()]
if '--swim-cap' in sys.argv:
    m.Add(sum(swim_lits) <= int(arg('--swim-cap', '0')))
if '--min-swim' in sys.argv:
    m.Minimize(sum(swim_lits))          # replaces the objective; the hint stays
    SCALE = 1
print(f'{os.path.basename(path)}: {len(m.proto.variables)} vars, {len(m.proto.constraints)} constraints, '
      f'{len(swim_idx)} swimmer literals, hint {"kept" if _pb.HasField("solution_hint") else "none"}; '
      f'chain det {meta.get("det")} workers {meta.get("workers")}; this run det {det:g} workers {workers}')


# the choice literals by name, for the objective split: d_<net>_<j>, s_<net>_<i>, mt_/md_<net>
var_idx = {v.name: i for i, v in enumerate(m.proto.variables)}
bt, tt = meta.get('berth_terms') or {}, meta.get('tooth_terms') or {}
VIA_W, CHAN_W = float(meta.get('via_w') or 3), float(meta.get('chan_w') or 2)
SWIM_V, MIS = float(meta.get('swim') or 100), float(meta.get('mismatch') or 1)


def split(val):
    """(swim, vias, channel, reach, mismatch) of a solution, in the objective's units."""
    if not bt:
        return None
    sw = vi = ch = re_ = mi = 0.0
    for n in names:
        for j, (v, ln, ar) in enumerate(bt.get(n, [])):
            k = var_idx.get(f'd_{n}_{j}')
            if k is not None and val(k):
                vi += VIA_W * v; ch += CHAN_W * ln; re_ += ar
        for i, (v, ln, ar) in enumerate(tt.get(n, [])):
            k = var_idx.get(f's_{n}_{i}')
            if k is not None and val(k):
                vi += VIA_W * v; ch += CHAN_W * ln
        vi += VIA_W * float((meta.get('tooth0_vias') or {}).get(n, 0))
        for pre in ('mt_', 'md_'):
            k = var_idx.get(pre + n)
            if k is not None and val(k):
                mi += VIA_W * MIS
        k = var_idx.get('w_' + n)
        if k is not None and val(k):
            sw += SWIM_V * VIA_W
    return sw, vi, ch, re_, mi


class Curve(cp_model.CpSolverSolutionCallback):
    def __init__(self):
        super().__init__()
        self.rows = []

    def on_solution_callback(self):
        sw = [nm for nm, i in swim_idx.items() if self.Value(m.get_bool_var_from_proto_index(i))]
        self.rows.append((self.DeterministicTime(), self.WallTime(), self.ObjectiveValue() / SCALE,
                          self.BestObjectiveBound() / SCALE, len(sw)))
        parts = split(lambda k: self.Value(m.get_bool_var_from_proto_index(k)))
        ps = ''
        if parts:
            ps = f'  [swim {parts[0]:.0f} vias {parts[1]:.0f} chan {parts[2]:.0f} reach {parts[3]:.0f} mism {parts[4]:.0f}]'
        print(f'  det {self.DeterministicTime():8.2f}  wall {self.WallTime():7.1f} s  obj {self.ObjectiveValue() / SCALE:8.1f}  '
              f'bound {self.BestObjectiveBound() / SCALE:8.1f}  swimmers {len(sw)} {sw}{ps}', flush=True)


solver = cp_model.CpSolver()
solver.parameters.num_workers = workers
solver.parameters.interleave_search = True
solver.parameters.max_deterministic_time = det
if '--lin' in sys.argv:
    solver.parameters.linearization_level = int(arg('--lin', '1'))
if '--sym' in sys.argv:
    solver.parameters.symmetry_level = int(arg('--sym', '2'))
cb = Curve()
t0 = time.time()
status = solver.Solve(m, cb)
print(f'FINAL {solver.StatusName(status)} obj {solver.ObjectiveValue() / SCALE:.1f} bound {solver.BestObjectiveBound() / SCALE:.1f} '
      f'det {solver.deterministic_time:.1f} wall {time.time() - t0:.1f} s; {len(cb.rows)} improvements')
# the answer AT the chain's budget: the last improvement before it
chain_det = float(meta.get('det') or 40)
at = [r for r in cb.rows if r[0] <= chain_det]
if at:
    r = at[-1]
    print(f'  at det {chain_det:g}: obj {r[2]:.1f} (swimmers {r[4]}); best found obj {cb.rows[-1][2]:.1f} (swimmers {cb.rows[-1][4]}) at det {cb.rows[-1][0]:.1f}')
