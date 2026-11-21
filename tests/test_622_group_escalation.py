#!/usr/bin/env python3
"""A group of buses that leaves nets open is routed again a rung down the fab ladder, each rung from the run before
it -- the buses that failed alone, where the others are clean (awx/route_bus.py: --escalation, escalation_ladder,
_escalated, _group_state, _failed_buses, _kept, _bus_rung, _bare_arrays).

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
8. the CLI takes --escalation with the policies route.py takes;
9. a group whose ends model found a bus NO ROOM at its sizes -- nothing routed, its end kept -- is the rungs' to
   answer all the same: each rung from the end before it, each told the rungs below it, its record's result none;
10. a rung's outputs are its own: an earlier run's rung files in the same place are removed before the rung runs;
11. a group a bus of which wrote no board (its nets are in no grade) has no result;
12. the widths a walled ball is tested at are the policy's finest rung -- under off, with no rung, the run's own;
13. the driver routes each rung under that rung's sizes;
14. THE BUSES THAT FAILED: the buses whose nets a group's grade has open -- none where the result is anything else
    (a violation, a ball walled in, an open net no bus has, a bus with no summary): that is the whole group's;
15. the board they are routed on again: the group's board with their nets as the board came with them -- their
    copper off, the board's own back, their nets no longer protected -- and every other bus's copper as it stands;
16. the rungs by bus: a group with nets open by SOME of its buses routes those buses alone a rung down on that
    board, the others kept; the rung's summary is the whole group's, graded whole against the board the group came
    in with; a rung that did worse does not stand, and the next starts from its end; every bus open, the whole
    group again;
17. the board a failed bus's ends model is captured on: the kept board without the copper inside its ball arrays
    that the group's input has not;
18. ...and that copper is named for each bus's view of the board all the same (group_ends.kept_copper).
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
        self.results, self.starts, self.sizes, self.below, self.active = list(results), [], [], [], []

    def route_grouped(self, board, out, buses, k=None, rounds=4, inproc=False, log=print, json_out=None,
                      feedback_rounds=0, t0=None, start=None, rungs_below=None, **sizes):
        import rules as _rules
        self.starts.append(start)
        self.sizes.append(sizes)
        self.below.append(rungs_below)
        self.active.append((_rules.active().track, _rules.active().via_size))
        res = self.results.pop(0)
        if res is None:
            return 1, []
        if res == 'no room':
            # (the group's ends model found a bus no room at these sizes: nothing routed, its end kept)
            json.dump({'out': out, 'ends': {'exit': 7}}, open(json_out, 'w'))
            kept(out)
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

NO_ROOM = {'ends': {'exit': 7}}
with tempfile.TemporaryDirectory() as td:
    # 9. no room at the sizes asked for
    got, nb, out, js = escalated(td, dict(NO_ROOM), ['no room', (0, True), (0, True)])
    check('9. a group with no room at its sizes: the rungs, the first from its own end',
          len(nb.starts) == 2 and nb.starts[0] == (os.path.join(out[:-10] + '.bus', 'group'), out[:-10] + '.bus'),
          nb.starts)
    r1 = out[:-len('.kicad_pcb')] + '.rung1.kicad_pcb'
    check('9. ...a rung with no room either: the next from THAT rung\'s end',
          nb.starts[1] == (os.path.join(r1[:-10] + '.bus', 'group'), r1[:-10] + '.bus'), nb.starts[1])
    check('9. ...each rung told the rungs below it, the lanes\' track and the via of each',
          nb.below[0] == [(r.track, r.via_size) for r in rungs[1:]]
          and nb.below[1] == [(r.track, r.via_size) for r in rungs[2:]], nb.below)
    check('9. ...the clean rung stands', got[0].endswith('.rung2.kicad_pcb') and rb._group_key(got[3]) == (0, 0, 0))
    esc = json.load(open(got[1])).get('escalation') or {}
    check('9. ...the record: no result at the sizes asked for nor at rung 1, in JSON any reader takes',
          [t['result'] for t in esc.get('rungs', [])][:2] == [None, None] and 'Infinity' not in open(got[1]).read(), esc)
    # 13. the driver under each run's sizes
    check('13. each rung routed with ITS sizes this process\'s (the merged project is stamped at them)',
          nb.active == [(r.track, r.via_size) for r in rungs[:2]], nb.active)
with tempfile.TemporaryDirectory() as td:
    # 10. a rung's outputs its own
    out = os.path.join(td, 'g.kicad_pcb')
    r1 = out[:-len('.kicad_pcb')] + '.rung1.kicad_pcb'
    r1j = os.path.join(td, 'g.rung1.summary.json')
    open(r1, 'w').close()
    json.dump(grade(0), open(r1j, 'w'))
    stale = kept(r1)
    got, nb, out, js = escalated(td, grade(2), [None, (1, True), (1, True)])
    check('10. an earlier run\'s rung files are not this run\'s: a rung that wrote nothing has no result',
          not os.path.isfile(r1) and not os.path.isfile(r1j) and not os.path.isdir(stale), os.listdir(td))
    check('10. ...nor is that run\'s end the next rung\'s start', nb.starts[1] == nb.starts[0], nb.starts)
    check('10. ...and the best rung routed now stands', got[0].endswith('.rung2.kicad_pcb'), got[0])

# 11. a bus that wrote no board
dead = grade(0)
dead['grade']['dead'] = ['U9']
check('11. a group a bus of which wrote no board has no result, whatever the others\' grade',
      rb._group_key(dead)[0] == math.inf)

# 12. the widths a cage is tested at
import types  # noqa: E402
PCB4 = types.SimpleNamespace(board_info=types.SimpleNamespace(copper_layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']))
rr, _r = ladder('fab', **SIZES)
check('12. under fab: the ladder\'s finest rung', rb._cage_rung(PCB4)[0] == min(
    f['track_width'] for f in ft.escalation_rungs(4)), rb._cage_rung(PCB4))
rr, _r = ladder('off', **SIZES)
import rules as _rules  # noqa: E402
prev = _rules.active()
_rules.install(rr)
try:
    check('12. under off (no rung): the run\'s own sizes', rb._cage_rung(PCB4) == (rr.track, rr.via_size, rr.clearance),
          rb._cage_rung(PCB4))
finally:
    _rules.install(prev)
ladder('fab', **SIZES)

# 14. the buses that failed
def group_sj(td, buses, open_, drc=0, walled=0, tag=''):
    """a group's summary: `buses` {dest: its nets}, each bus's own summary beside it, the nets `open_` open"""
    rows = []
    for d_, nets in buses.items():
        p_ = os.path.join(td, f'{tag}{d_}.summary.json')
        json.dump({'routed': [n for n in nets if n not in open_], 'refused': {n: 'open' for n in nets if n in open_}},
                  open(p_, 'w'))
        rows.append({'src': 'U1', 'dest': d_, 'exit': 0, 'routed': len(nets), 'grade': None, 'summary': p_})
    return {'buses': rows, 'grade': {'open': sorted(open_), 'drc': {'n': drc},
                                     'walled': {'walled': [f'B{i}' for i in range(walled)]}}}


GRP = [('U1', 'A'), ('U1', 'B'), ('U1', 'C')]
NETS = {'A': ['DP1_P'], 'B': ['DP1_N', 'DP2_N'], 'C': ['INT1']}
with tempfile.TemporaryDirectory() as td:
    check('14. the bus whose net is open', rb._failed_buses(group_sj(td, NETS, {'DP2_N'}), GRP) == [('U1', 'B')])
    check('14. ...two of three, in the group\'s order',
          rb._failed_buses(group_sj(td, NETS, {'INT1', 'DP1_N'}), GRP) == [('U1', 'B'), ('U1', 'C')])
    check('14. a clean group: none', rb._failed_buses(group_sj(td, NETS, set()), GRP) is None)
    check('14. a violation, or a ball walled in, is the whole group\'s',
          rb._failed_buses(group_sj(td, NETS, {'DP2_N'}, drc=1), GRP) is None
          and rb._failed_buses(group_sj(td, NETS, {'DP2_N'}, walled=1), GRP) is None)
    sj = group_sj(td, NETS, {'DP2_N'})
    sj['grade']['open'].append('ELSEWHERE')
    check('14. ...and an open net no bus has', rb._failed_buses(sj, GRP) is None)
    sj = group_sj(td, NETS, {'DP2_N'})
    sj['buses'][2]['summary'] = None
    check('14. ...and a bus that wrote no summary', rb._failed_buses(sj, GRP) is None)


# 15. the board the failed buses are routed on again
def seg(net, x):
    return (f'\t(segment\n\t\t(start {x} 3.0)\n\t\t(end {x} 4.0)\n\t\t(width 0.127)\n\t\t(layer "F.Cu")\n'
            f'\t\t(net "{net}")\n\t\t(uuid "00000000-0000-0000-0000-0000000000{int(x * 10):02d}")\n\t)\n')


def copper(path):
    """{net: its segments' (start, end)s} on the board at `path`"""
    from kicad_parser import parse_kicad_pcb
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        p_ = parse_kicad_pcb(path)
    got = {}
    for s_ in p_.segments:
        got.setdefault(p_.nets[s_.net_id].name, set()).add(((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)))
    return got


def routed_board(td, name, nets):
    """BOARD with a segment of its own on each of `nets` (and DP2_N's own taken off, laid anew): a group's board"""
    from copy_board import copy_board
    path = os.path.join(td, name)
    copy_board(BOARD, path)
    txt = open(path, encoding='utf-8').read()
    import braid as te
    txt = te.strip_net_items(txt, 'segment', set(), {'DP2_N'})
    end = txt.rstrip().rfind(')')
    txt = txt[:end] + ''.join(seg(n, 3.0 + i) for i, n in enumerate(nets)) + txt[end:]
    open(path, 'w', encoding='utf-8').write(txt)
    return path


with tempfile.TemporaryDirectory() as td:
    import rules as _rules_k  # noqa: E402
    came = copper(BOARD)
    out = routed_board(td, 'g.kicad_pcb', ['DP1_P', 'DP1_N', 'DP2_N', 'INT1'])
    check('15. (the group\'s board: every bus\'s nets laid, one of them not as it came)',
          set(copper(out)) >= {'DP1_P', 'DP1_N', 'INT1'} and copper(out)['DP2_N'] != came['DP2_N'])
    from protected_nets import persist_protected_nets, pro_path_for_board, read_protected_nets  # noqa: E402
    json.dump({}, open(pro_path_for_board(out), 'w'))
    persist_protected_nets(pro_path_for_board(out), {n: 'bus' for n in ('DP1_P', 'DP1_N', 'DP2_N', 'INT1')},
                           verbose=False)
    sj = group_sj(td, NETS, {'DP2_N'})
    kept_ = os.path.join(td, 'kept.kicad_pcb')
    rr0, _r = ladder('fab', **SIZES)
    with rb._installed(rr0):
        back = rb._kept(BOARD, out, sj, [('U1', 'B')], kept_)
    now = copper(kept_)
    check('15. the failed bus\'s nets as the board came with them: its copper off, the board\'s own back',
          'DP1_N' not in now and now.get('DP2_N') == came['DP2_N'] and set(back) == {'DP1_N', 'DP2_N'}, now)
    check('15. ...every other bus\'s copper as it stands', now.get('DP1_P') == copper(out)['DP1_P']
          and now.get('INT1') == copper(out)['INT1'] and now.get('FOREIGN') == came['FOREIGN'])
    check('15. ...and its nets no longer protected, the others\' still',
          set(read_protected_nets(pro_path_for_board(kept_))) == {'DP1_P', 'INT1'},
          read_protected_nets(pro_path_for_board(kept_)))


# 16. the rungs by bus
class BusRuns(Runs):
    """...route_grouped answering for the buses it is given: each result (the nets left open, keeps its end)"""

    def __init__(self, td, results):
        super().__init__(results)
        self.td, self.buses, self.boards, self.caps = td, [], [], []

    def route_grouped(self, board, out, buses, k=None, rounds=4, inproc=False, log=print, json_out=None,
                      feedback_rounds=0, t0=None, start=None, rungs_below=None, capture_board=None, **sizes):
        self.starts.append(start)
        self.sizes.append(sizes)
        self.buses.append([d_ for _s, d_ in buses])
        self.boards.append((board, copper(board)))
        self.caps.append(capture_board)
        res = self.results.pop(0)
        if res == 'no room':
            json.dump({'out': out, 'ends': {'exit': 7}}, open(json_out, 'w'))
            kept(out)
            return 1, []
        open_, keeps = res
        from copy_board import copy_board
        copy_board(board, out)
        txt = open(out, encoding='utf-8').read()
        end = txt.rstrip().rfind(')')
        laid = [n for _s, d_ in buses for n in NETS[d_] if n not in open_]
        open(out, 'w', encoding='utf-8').write(
            txt[:end] + ''.join(seg(n, 6.0 + len(self.buses) + 0.1 * i) for i, n in enumerate(laid)) + txt[end:])
        json.dump(group_sj(self.td, {d_: NETS[d_] for _s, d_ in buses}, set(open_), tag=f'r{len(self.buses)}.'),
                  open(json_out, 'w'))
        if keeps:
            kept(out)
        return 0, []


def by_bus(td, base_open, results):
    """_escalated on a three-bus group whose base run left `base_open` open; the grade of each rung's whole board a
    stand-in's -- the nets with no copper beyond what the board came with"""
    out, js = routed_board(td, 'g.kicad_pcb', [n for ns in NETS.values() for n in ns if n not in base_open]), \
        os.path.join(td, 'g.summary.json')
    sj = group_sj(td, NETS, set(base_open))
    json.dump(sj, open(js, 'w'))
    kept(out)
    nb = BusRuns(td, results)
    graded = []

    def grade_(board, routed, buses, work, log=print, tag='board'):
        have, came_ = copper(routed), copper(board)
        nets = [n for _s, _d, ns in buses for n in ns]
        graded.append((board, sorted(nets), tag))
        return {'open': sorted(n for n in nets if have.get(n) == came_.get(n)), 'nets': len(nets), 'drc': {'n': 0},
                'walled': {'walled': []}, 'ok': True}
    real = rb.board_grade
    rb.board_grade = grade_
    try:
        with under('fab'):
            got = rb._escalated(BOARD, out, js, 0, dict(sj), GRP, None, 1, False, quiet, 0, nb, SIZES)
    finally:
        rb.board_grade = real
    return got, nb, out, graded


with tempfile.TemporaryDirectory() as td:
    got, nb, out, graded = by_bus(td, {'DP2_N'}, [((), True)])
    check('16. one bus of three with a net open: that bus alone routed again, one rung', nb.buses == [['B']], nb.buses)
    brd, cu = nb.boards[0]
    check('16. ...on the group\'s board with its own nets as they came, the others\' copper kept',
          brd.endswith(os.path.join('g.rung1.bus', 'kept.kicad_pcb')) and 'DP1_N' not in cu
          and cu.get('DP2_N') == copper(BOARD)['DP2_N'] and {'DP1_P', 'INT1'} <= set(cu), (brd, sorted(cu)))
    check('16. ...its captures on that board with its arrays bare, beside it',
          nb.caps[0] == os.path.join(os.path.dirname(brd), 'kept.bare.kicad_pcb') and os.path.isfile(nb.caps[0]),
          nb.caps)
    check('16. ...at the rung\'s sizes, searched from the start (no end of these buses kept)',
          nb.sizes[0]['track_width'] == rungs[0].track and nb.starts == [None], nb.starts)
    sj_ = json.load(open(got[1]))
    check('16. ...the rung\'s summary the whole group\'s: every bus, the kept ones as they stood',
          [b['dest'] for b in sj_['buses']] == ['A', 'B', 'C'] and sj_['held']['buses'] == ['A', 'C']
          and sj_['own']['buses'] == ['B'] and rb._group_key(sj_) == (0, 0, 0), sj_.get('held'))
    check('16. ...graded whole, every bus\'s nets against the board the group came in with',
          graded == [(BOARD, sorted(n for ns in NETS.values() for n in ns), 'group')], graded)
    esc = sj_.get('escalation') or {}
    check('16. ...and the record names the buses the rung routed', esc.get('kept') == 1
          and esc['rungs'][1].get('buses') == ['B'] and 'buses' not in esc['rungs'][0], esc)
with tempfile.TemporaryDirectory() as td:
    got, nb, out, graded = by_bus(td, {'DP2_N', 'INT1'}, [(('INT1',), True), ((), True)])
    check('16. two buses open: both alone at rung 1; the one still open alone at rung 2, from the board rung 1 left',
          nb.buses == [['B', 'C'], ['C']] and nb.boards[1][0].endswith(os.path.join('g.rung2.bus', 'kept.kicad_pcb'))
          and 'DP1_N' in nb.boards[1][1] and 'INT1' not in nb.boards[1][1], (nb.buses, sorted(nb.boards[-1][1])))
    check('16. ...the clean board stands', got[0].endswith('.rung2.kicad_pcb') and rb._group_key(got[3]) == (0, 0, 0))
with tempfile.TemporaryDirectory() as td:
    got, nb, out, graded = by_bus(td, {'DP2_N'}, [(('DP2_N', 'DP1_N'), True), ((), True)])
    check('16. a rung that did worse does not stand: the next routes the same bus on the same board, from that '
          'rung\'s end', nb.buses == [['B'], ['B']] and nb.boards[0][1] == nb.boards[1][1]
          and nb.starts[1] == (os.path.join(out[:-10] + '.rung1.bus', 'group'), out[:-10] + '.rung1.bus'), nb.starts)
with tempfile.TemporaryDirectory() as td:
    got, nb, out, graded = by_bus(td, {'DP1_P', 'DP2_N', 'INT1'}, [((), True)])
    check('16. every bus open: the whole group again, on the board it came in with',
          nb.buses == [['A', 'B', 'C']] and nb.boards[0][0] == BOARD and nb.caps == [None], nb.buses)

# 17. the board a failed bus's ends model is captured on
with tempfile.TemporaryDirectory() as td:
    from kicad_parser import parse_kicad_pcb  # noqa: E402
    import contextlib  # noqa: E402
    import io  # noqa: E402
    with contextlib.redirect_stdout(io.StringIO()):
        fp_u1 = parse_kicad_pcb(BOARD).footprints['U1']
    cx = sum(p.global_x for p in fp_u1.pads) / len(fp_u1.pads)
    cy = sum(p.global_y for p in fp_u1.pads) / len(fp_u1.pads)
    from copy_board import copy_board  # noqa: E402
    kb = os.path.join(td, 'kept.kicad_pcb')
    copy_board(BOARD, kb)
    txt = open(kb, encoding='utf-8').read()
    end = txt.rstrip().rfind(')')
    inside = (f'\t(segment\n\t\t(start {cx:.3f} {cy:.3f})\n\t\t(end {cx + 0.3:.3f} {cy:.3f})\n\t\t(width 0.1)\n'
              f'\t\t(layer "B.Cu")\n\t\t(net "INT1")\n\t\t(uuid "00000000-0000-0000-0000-000000000091")\n\t)\n')
    open(kb, 'w', encoding='utf-8').write(txt[:end] + inside + seg('INT2', 3.0) + txt[end:])
    real_arr = rb.is_array
    rb.is_array = lambda fp: fp.reference == 'U1'       # (the part a ball array for this check)
    try:
        bare_ = os.path.join(td, 'kept.bare.kicad_pcb')
        n_off = rb._bare_arrays(BOARD, kb, [('U1', 'B')], bare_)
    finally:
        rb.is_array = real_arr
    have, came_ = copper(bare_), copper(BOARD)
    check('17. copper the kept board holds inside the bus\'s array, and the board came without: off',
          n_off == 1 and 'INT1' not in have, (n_off, sorted(have)))
    check('17. ...the board\'s own copper there, and the kept copper beyond the array: as they stand',
          all(have.get(n) == came_[n] for n in came_) and have.get('INT2') == copper(kb)['INT2'], sorted(have))
    rb.is_array = lambda fp: False
    try:
        check('17. a part that is no ball array keeps what stands at it',
              rb._bare_arrays(BOARD, kb, [('U1', 'B')], bare_) == 0 and 'INT1' in copper(bare_))
    finally:
        rb.is_array = real_arr
    # 18. ...and the copper left out of the capture is in each bus's view of the board all the same
    try:
        import ortools  # noqa: F401
        import group_ends as ge  # noqa: E402
    except ImportError:
        ge = None
        print('     (18 skipped: ortools is not installed -- the group\'s ends model plans with CP-SAT)')
    if ge is not None:
        rb.is_array = lambda fp: fp.reference == 'U1'
        try:
            rb._bare_arrays(BOARD, kb, [('U1', 'B')], bare_)
        finally:
            rb.is_array = real_arr
        with contextlib.redirect_stdout(io.StringIO()):
            bp, cp = parse_kicad_pcb(kb), parse_kicad_pcb(bare_)
        xs, ys = [p.global_x for p in fp_u1.pads], [p.global_y for p in fp_u1.pads]
        A1 = types.SimpleNamespace(ref='U1', box=(min(xs), min(ys), max(xs), max(ys)))
        got = ge.kept_copper(bp, cp, {'U1': A1})['U1']
        check('18. what the board holds inside the array that the captures\' board has not: named for each bus\'s view',
              [bp.nets[s_.net_id].name for s_ in got[0]] == ['INT1'] and got[1] == [], got)
        check('18. ...none where the captures are of the board itself', ge.kept_copper(bp, bp, {'U1': A1})['U1'] == ([], []))

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
