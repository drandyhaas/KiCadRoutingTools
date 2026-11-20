#!/usr/bin/env python3
"""A group of buses that leaves nets open is routed again a rung down the fab ladder, each rung from the run before
it (awx/route_bus.py: --escalation, escalation_ladder, _escalated, _group_state).

  python3 tests/test_622_group_escalation.py

route.py walks its fab escalation for a net that would ship open. The bus step walks the same ladder for a GROUP of
buses: the lanes' track, the escapes' track and the via a rung down together, the clearance never, rung after rung
until one leaves the board clean; the fewest nets open, then violations, then balls walled in stands. A rung starts
from the end the run before it kept -- its group's model, the findings its lay left -- so it is that run repaired,
not the search again.

1. the ladder: under `fab` every rung's lanes and escapes are narrower than the one before, no via larger than the
   run's, the clearance the run's on every rung; `off` has no rung, nor has a run already at the floor;
2. a group's result as the rungs compare them: (open, violations, walled) -- no grade, or a check with no verdict,
   the worst;
3. a clean group is not routed again; nor is one with NO grade -- a run with no result is the step's own result,
   not a net left open;
4. a group with nets open: the rungs in turn, each from the last run that kept its group's end, until one is clean;
   the best one's board, summary and status stand, the escalation's record in its summary;
5. ...a rung that kept no end (its model ended with no state) is passed over as a start: the rung after it starts
   from the run before that one;
6. ...and where no rung does better, the sizes asked for stand;
7. a group's kept end is read where the run left it: `group` beside its lay for one round, `groupN` and `roundN.bus`
   for the round that stands of several; none without a `resume.pkl`;
8. the CLI takes --escalation with the policies route.py takes.
"""
import json
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
if not os.path.isfile(BOARD):
    raise SystemExit(f'BROKEN TEST: no board {BOARD}')

import awx_settings  # noqa: E402
import fab_tiers as ft  # noqa: E402
import route_bus as rb  # noqa: E402

BAD, N = [], [0]
SIZES = dict(clearance_ceiling=0.1, track_width=0.15, fanout_track_width=0.12, via_size=0.45, via_drill=0.3)
quiet = lambda *a_, **k_: None       # noqa: E731


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


def under(policy):
    return awx_settings.given({**awx_settings.environ(), rb.ESCALATION_SETTING: policy})


def ladder(policy, **sizes):
    with under(policy):
        rr = rb.resolve_rules(BOARD, log=quiet, **sizes)
        rb.install_escalation(BOARD, rr)
        return rr, rb.escalation_ladder(rr, BOARD)


# 1. the ladder
rr, rungs = ladder('fab', **SIZES)
tr = [rr.track] + [r.track for r in rungs]
fw = [rr.fan_track] + [r.fan_track for r in rungs]
check('1. fab: two or three rungs', len(rungs) in (2, 3), len(rungs))
check('1. ...the lanes narrower rung by rung', all(a > b for a, b in zip(tr, tr[1:])), tr)
check('1. ...the escapes too', all(a > b for a, b in zip(fw, fw[1:])), fw)
check('1. ...no via larger than the run\'s, none larger than the rung before',
      all(a >= b for a, b in zip([rr.via_size] + [r.via_size for r in rungs], [r.via_size for r in rungs])))
check('1. ...the clearance the run\'s on every rung', all(r.clearance == rr.clearance for r in rungs))
check('1. ...the last rung at the ladder\'s floor',
      rungs[-1].track == min(f['track_width'] for f in ft.escalation_rungs(ft.count_copper_layers_in_file(BOARD))))
check('1. off: no rung', ladder('off', **SIZES)[1] == [])
floor = dict(SIZES, track_width=rungs[-1].track, fanout_track_width=rungs[-1].fan_track,
             via_size=rungs[-1].via_size, via_drill=rungs[-1].via_drill)
check('1. a run at the floor: no rung', ladder('fab', **floor)[1] == [])


# 2. the key
def grade(n_open, drc=0, walled=0):
    return {'grade': {'open': [f'N{i}' for i in range(n_open)], 'drc': {'n': drc},
                      'walled': {'walled': [f'B{i}' for i in range(walled)]}}}


check('2. (open, violations, walled)', rb._group_key(grade(2, 3, 1)) == (2, 3, 1))
check('2. no grade the worst', rb._group_key({})[0] == math.inf)
check('2. a check with no verdict the worst', rb._group_key(grade(0, None))[0] == math.inf)


class Runs:
    """negotiated_buses' stand-in: route_grouped answers each rung with the next of `results` -- (open, keeps its end)
    or None for a run with no result -- and remembers the start it was given"""

    def __init__(self, results):
        self.results, self.starts, self.sizes = list(results), [], []

    def route_grouped(self, board, out, buses, k=None, rounds=4, inproc=False, log=print, json_out=None,
                      feedback_rounds=0, t0=None, start=None, **sizes):
        self.starts.append(start)
        self.sizes.append(sizes)
        res = self.results.pop(0)
        if res is None:
            return 1, []
        n_open, keeps = res
        open(out, 'w').close()
        json.dump(grade(n_open), open(json_out, 'w'))
        if keeps:
            kept(out)
        return 0, []


def kept(out):
    g = os.path.join(out[:-len('.kicad_pcb')] + '.bus', 'group')
    os.makedirs(g, exist_ok=True)
    open(os.path.join(g, 'resume.pkl'), 'w').close()
    return g


def escalated(td, base, results, keep_base=True):
    out, js = os.path.join(td, 'g.kicad_pcb'), os.path.join(td, 'g.summary.json')
    open(out, 'w').close()
    json.dump(base, open(js, 'w'))
    if keep_base:
        kept(out)
    nb = Runs(results)
    with under('fab'):
        got = rb._escalated(BOARD, out, js, 0, dict(base), [('U1', 'U2')], None, 1, False, quiet, 0, nb, SIZES)
    return got, nb, out, js


with tempfile.TemporaryDirectory() as td:
    # 3. nothing to answer
    got, nb, out, js = escalated(td, grade(0), [])
    check('3. a clean group is not routed again', nb.starts == [] and got[0] == out)
with tempfile.TemporaryDirectory() as td:
    got, nb, out, js = escalated(td, {}, [(0, True)])
    check('3. nor a group with no grade', nb.starts == [] and got[0] == out)
with tempfile.TemporaryDirectory() as td:
    # 4. rungs until one is clean
    got, nb, out, js = escalated(td, grade(2), [(1, True), (0, True), (0, True)])
    r1, r2 = out[:-len('.kicad_pcb')] + '.rung1.kicad_pcb', out[:-len('.kicad_pcb')] + '.rung2.kicad_pcb'
    check('4. two rungs, and no third after a clean one', len(nb.starts) == 2, len(nb.starts))
    check('4. ...the first from the group\'s own end', nb.starts[0] == (os.path.join(out[:-10] + '.bus', 'group'),
                                                                  out[:-10] + '.bus'), nb.starts[0])
    check('4. ...the second from the first rung\'s', nb.starts[1] == (os.path.join(r1[:-10] + '.bus', 'group'),
                                                                 r1[:-10] + '.bus'), nb.starts[1])
    check('4. ...each at its rung\'s sizes, the clearance as asked',
          [s['track_width'] for s in nb.sizes] == [r.track for r in rungs[:2]]
          and all(s['clearance_ceiling'] == SIZES['clearance_ceiling'] for s in nb.sizes))
    check('4. ...the clean rung\'s board stands', got[0] == r2 and rb._group_key(got[3]) == (0, 0, 0))
    esc = json.load(open(got[1])).get('escalation') or {}
    check('4. ...the escalation in its summary', esc.get('kept') == 2 and [t['rung'] for t in esc.get('rungs', [])]
          == [0, 1, 2] and [t['result'][0] for t in esc['rungs']] == [2, 1, 0], esc)
with tempfile.TemporaryDirectory() as td:
    # 5. a rung with no end kept
    got, nb, out, js = escalated(td, grade(2), [None, (1, True), (1, True)])
    check('5. a rung with no result is not a start: the next from the group\'s own end again',
          len(nb.starts) == 3 and nb.starts[1] == nb.starts[0] and nb.starts[2] != nb.starts[0], nb.starts)
    check('5. ...the best rung stands', got[0].endswith('.rung2.kicad_pcb'), got[0])
with tempfile.TemporaryDirectory() as td:
    # 6. no rung better
    got, nb, out, js = escalated(td, grade(1), [(1, True), (2, True), (3, True)])
    check('6. no rung better: the sizes asked for stand', got[0] == out and len(nb.starts) == len(rungs))
    check('6. ...its summary says so', (json.load(open(got[1])).get('escalation') or {}).get('kept') == 0)
with tempfile.TemporaryDirectory() as td:
    # 7. where a run keeps its end
    out = os.path.join(td, 'x.kicad_pcb')
    check('7. no resume.pkl: no end', rb._group_state(out, {}) is None)
    g = kept(out)
    check('7. one round: `group` and the lay beside it', rb._group_state(out, {}) == (g, out[:-10] + '.bus'))
    g3 = os.path.join(out[:-10] + '.bus', 'group3')
    os.makedirs(g3)
    open(os.path.join(g3, 'resume.pkl'), 'w').close()
    check('7. the round that stands of several: `group3` and `round3.bus`',
          rb._group_state(out, {'round': 3, 'rounds': [1, 2, 3]}) == (g3, os.path.join(out[:-10] + '.bus', 'round3.bus')))

# 8. the CLI
h = subprocess.run([sys.executable, os.path.join(ROOT, 'awx', 'route_bus.py'), '--help'], capture_output=True, text=True)
check('8. --escalation with route.py\'s policies', h.returncode == 0 and '--escalation' in h.stdout
      and all(p in h.stdout for p in ft.ESCALATION_POLICIES), h.stderr[-200:])

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
