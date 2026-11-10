"""cp_seed.solver(): a CP-SAT solver under the bus step's seed.

Every solve of a whole-route round -- the joint ends and their checks, the escape plans, the round's solve and the
loop's re-solves -- searches under ONE seed, BUS_SEED, so a round is one draw: the same board and seed give the same
plan, and another seed another plan. A round that leaves nets open is followed by round 1 again under the next seed
(whole_route.chain). Unset, a solver keeps CP-SAT's own default seed, and round 1 is what it has always been.

CP-SAT is imported by solver() itself: a driver that reads only DEFAULT (whole_route) imports this module before
its dependency gate (startup_checks.check_bus_dependencies) has run, and a missing ortools must reach that gate.

solve_apart(): one solve in a PROCESS OF ITS OWN, its answer read back -- for a solve the library may die in. CP-SAT
9.15 ends the process, on Linux, in two of the whole solve's first-plan searches: a one-worker search stopped at its
first solution, where glibc reports the heap corrupted ("corrupted size vs. prev_size") from the destructor of the
solver's disjunctive precedences as the search is taken down; and a four-worker search with the objective set aside,
a segmentation fault in a worker's clause propagation. In the caller's process that ends the whole bus step, every
stage of it; in its own, the solve has no answer and the caller goes on as it does with no plan."""
import json
import os
import subprocess
import sys
import tempfile

import awx_settings

DEFAULT = 1             # CP-SAT's own random_seed when none is given: round r runs under DEFAULT + r - 1


def solver():
    """a CpSolver, its random_seed BUS_SEED's when the step gives one"""
    from ortools.sat.python import cp_model
    s = cp_model.CpSolver()
    v = awx_settings.get('BUS_SEED')
    if v not in (None, ''):
        s.parameters.random_seed = int(v)
    return s


class Apart:
    """solve_apart's answer, read as a solver's is: `status` (CP-SAT's, UNKNOWN where the process died -- `died` its
    exit code then), Value(a variable or a linear expression) from every variable's value, StatusName()"""
    def __init__(self, status, values, died=None):
        self.status, self.values, self.died = status, values, died

    def Value(self, e):
        from ortools.sat.python import cp_model_helper
        f = cp_model_helper.FlatIntExpr(e)
        return f.offset + sum(c * self.values[v.index] for v, c in zip(f.vars, f.coeffs))

    def StatusName(self, status=None):
        if self.died is not None:
            return f'UNKNOWN: the solver\'s process died, ' + (f'signal {-self.died}' if self.died < 0
                                                                else f'exit {self.died}')
        return (self.status if status is None else status).name


def solve_apart(model, s):
    """(an Apart, its status): `model` solved under the solver `s`'s parameters in a process of its own -- the same
    model and parameters, so the same answer `s.Solve(model)` gives here. A process that dies (the library's crash)
    leaves status UNKNOWN and no values, its last lines on stderr"""
    from ortools.sat.python import cp_model
    with tempfile.TemporaryDirectory() as d:
        mf, pf, of = (os.path.join(d, n) for n in ('model.txt', 'parameters.txt', 'answer.json'))
        if not model.export_to_file(mf):
            raise RuntimeError(f'solve_apart: the model was not written to {mf}')
        with open(pf, 'w', encoding='utf-8') as f:
            f.write(str(s.parameters))
        p = subprocess.run([sys.executable, '-c', 'import sys, cp_seed; cp_seed._apart(*sys.argv[1:])', mf, pf, of],
                           cwd=os.path.dirname(os.path.abspath(__file__)), capture_output=True, text=True,
                           errors='replace')
        if p.returncode != 0 or not os.path.isfile(of):
            print(f'solve_apart: the solver\'s process ended {p.returncode} with no answer; its last lines:\n'
                  + '\n'.join((p.stderr or '').strip().splitlines()[:6]), file=sys.stderr)
            return Apart(cp_model.UNKNOWN, None, died=p.returncode or 1), cp_model.UNKNOWN
        with open(of, encoding='utf-8') as f:
            r = json.load(f)
    st = type(cp_model.UNKNOWN)(r['status'])
    return Apart(st, r['values']), st


def _apart(mf, pf, of):
    """(solve_apart's process) the model and the parameters read, solved, the answer written"""
    from ortools.sat.python import cp_model
    m, s = cp_model.CpModel(), cp_model.CpSolver()
    with open(mf, encoding='utf-8') as f:
        if not m.Proto().parse_text_format(f.read()):
            sys.exit(f'solve_apart: {mf} is no model')
    with open(pf, encoding='utf-8') as f:
        if not s.parameters.parse_text_format(f.read()):
            sys.exit(f'solve_apart: {pf} are no parameters')
    st = s.Solve(m)
    vals = ([s.Value(m.GetIntVarFromProtoIndex(i)) for i in range(len(m.Proto().variables))]
            if st in (cp_model.OPTIMAL, cp_model.FEASIBLE) else None)
    with open(of + '.part', 'w', encoding='utf-8') as f:
        json.dump({'status': int(st), 'values': vals}, f)
    os.replace(of + '.part', of)
