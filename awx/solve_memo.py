#!/usr/bin/env python3
"""solve_memo.py -- a CP-SAT solve read back instead of run (#622, 2026-09-18).

The plan solver runs one worker under a deterministic-time budget, so the
same model with the same parameters gives the same solution every time.
The model's text form plus the parameters is the key; the solution (every
variable's value) is the record. A hit is replayed through the caller's
own solver with every variable fixed to its recorded value, so every
later `solver.Value(...)` -- on a variable or an expression -- answers as
the original solve did. Measured need: the chain's two fanout arms began
with identical solves (8 s each of a 36 s arm), and the population's
crossovers re-solve near-identical models. PROBE_MEMO=0 turns it off.
"""
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import probe_memo as pm  # noqa: E402

_stats = {'hit': 0, 'miss': 0}


def _store():
    return pm.Store('plan', 'solve') if pm.ENABLED else None


def solve(model, solver, log=None, label='pages-first'):
    """`solver.Solve(model)` behind the memo. Returns the status the
    original solve returned; the solver holds the solution either way."""
    from ortools.sat.python import cp_model
    import ortools
    st = _store()
    if st is None:
        return solver.Solve(model)
    key = hashlib.sha1((str(model.Proto()) + '\n' + str(solver.parameters) + '\n'
                        + getattr(ortools, '__version__', '?')).encode()).hexdigest()
    doc = st.get(key)
    n_vars = len(model.Proto().variables)
    if doc is not None and doc.get('n_vars') == n_vars:
        # the replay: a copy of the model with every variable hinted to its
        # recorded value and the hints made hard, solved by the caller's
        # solver so its Value() answers come from this solution
        m2 = cp_model.CpModel()
        m2.Proto().copy_from(model.Proto())
        m2.clear_hints() if hasattr(m2, 'clear_hints') else m2.ClearHints()
        for i, v in enumerate(doc['values']):
            m2.AddHint(m2.GetIntVarFromProtoIndex(i), int(v))
        keep = solver.parameters.fix_variables_to_their_hinted_value
        solver.parameters.fix_variables_to_their_hinted_value = True
        try:
            st2 = solver.Solve(m2)
        finally:
            solver.parameters.fix_variables_to_their_hinted_value = keep
        if st2 in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            _stats['hit'] += 1
            if log:
                log(f'  {label}: solve read back from the memo ({key[:10]}; '
                    f'{doc["status_name"]} then, objective {doc.get("objective")})')
            return getattr(cp_model, doc['status_name'])     # the status is an enum, stored by name
        if log:
            log(f'  {label}: memo replay did not verify ({solver.StatusName(st2)}) -- solving')
    _stats['miss'] += 1
    status = solver.Solve(model)
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        values = [int(solver.Value(model.GetIntVarFromProtoIndex(i))) for i in range(n_vars)]
        st.put(key, {'status_name': solver.StatusName(status), 'values': values, 'n_vars': n_vars,
                     'objective': solver.ObjectiveValue() if model.Proto().has_objective() else None,
                     'conflicts': solver.NumConflicts(), 'label': label})
    return status


def stats():
    return dict(_stats)


def _self_test():
    from ortools.sat.python import cp_model
    import tempfile
    pm.MEMO_DIR = tempfile.mkdtemp()

    def build():
        m = cp_model.CpModel()
        xs = [m.NewIntVar(0, 5, f'x{i}') for i in range(6)]
        b = m.NewBoolVar('b')
        for i in range(5):
            m.Add(xs[i] + xs[i + 1] <= 7)
        m.Add(xs[0] >= 2).OnlyEnforceIf(b)
        m.Maximize(sum(xs) + 3 * b)
        return m, xs, b
    m, xs, b = build()
    s = cp_model.CpSolver()
    s.parameters.num_workers = 1
    st = solve(m, s)
    v1 = [s.Value(x) for x in xs] + [s.Value(b), s.Value(xs[0] + 2 * xs[1]), s.ObjectiveValue()]
    assert _stats == {'hit': 0, 'miss': 1}
    m, xs, b = build()
    s = cp_model.CpSolver()
    s.parameters.num_workers = 1
    st2 = solve(m, s)
    v2 = [s.Value(x) for x in xs] + [s.Value(b), s.Value(xs[0] + 2 * xs[1]), s.ObjectiveValue()]
    assert _stats == {'hit': 1, 'miss': 1}, _stats
    assert st == st2 and v1 == v2, (v1, v2)
    assert s.StatusName(st2) == 'OPTIMAL', 'the replayed status is the solver\'s own enum'
    assert not s.parameters.fix_variables_to_their_hinted_value, 'the caller\'s parameters are restored'
    m.Add(xs[0] <= 1)                       # another model: a miss
    solve(m, s)
    assert _stats == {'hit': 1, 'miss': 2}
    print(f'solve_memo: self-test ok (values and expressions replay identically; {_stats})')


if __name__ == '__main__':
    _self_test()
