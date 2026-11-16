#!/usr/bin/env python3
"""The arrays' WHOLE-ARRAY plan (joint_escape.plan_array: every ball of an array escaped in one CP-SAT plan, the plan
the group's ends model builds its menus and its fanout from -- group_ends.py) on more routing layers than two.

  python3 tests/test_622_plan_array.py

1. the plan is the same in every process: two of its row loops (a pair's legs' runs in one capacity group, the run
   colouring's clauses) walked SETS of ball and layer names, in the string hash's order, and CP-SAT's answer follows
   the order its rows come in -- zynq U1's plan, the same ask under two hash seeds, laid 26 of its 47 teeth the same.
   Pinned on a generated four-copper bus: the plan's every model, fingerprinted as it is solved, and its answer, under
   two PYTHONHASHSEEDs in processes of their own;
2. the plan reads an avoided place on its layer alone. Priced, a run there was paid for by whichever ball took the
   exit, on whichever layer the colouring gave it, so the plan took the same exit as unpriced and its run was coloured
   onto the blocked layer again. On a generated four-copper bus: a run's exit avoided on its layer, the run leaves
   there on another, every bus ball served; a surface exit avoided on its layer, no ball leaves there on it; avoided
   on the other outer layer alone, the ball keeps its exit.
"""
import contextlib
import hashlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

if os.environ.get('_JOINT_ROUNDS_PLAN') == '1':
    # (check 3's child: the source array's whole-array plan of the board given, every model fingerprinted as it is
    # solved)
    from ortools.sat.python import cp_model
    shas = []
    _solve = cp_model.CpSolver.Solve

    def _fingerprinted(self, model, *a, **k):
        tmp = os.path.join(os.environ['_JOINT_ROUNDS_TMP'], f'model_{os.getpid()}_{len(shas)}.pb')
        model.ExportToFile(tmp)
        with open(tmp, 'rb') as f_:
            shas.append(hashlib.sha256(f_.read()).hexdigest()[:16])
        os.remove(tmp)
        return _solve(self, model, *a, **k)
    cp_model.CpSolver.Solve = _fingerprinted
    with contextlib.redirect_stdout(io.StringIO()):
        from kicad_parser import parse_kicad_pcb as _parse
        import joint_escape as _je
        import route_layers as _rl
        _pcb = _parse(sys.argv[1])
        _bus = sorted(n.name for n in _pcb.nets.values() if any(p.component_ref == 'SU1' for p in n.pads)
                      and any(p.component_ref == 'SD1' for p in n.pads))
        _hints, _rep = _je.plan_array(_pcb, 'SU1', _bus, [], ['F.Cu', 'B.Cu'], far=_je.far_face(_pcb, 'SU1', 'SD1'),
                                      log=lambda *_a: None, vias_only=_rl.escape_vias('src'))
    _plan = sorted([list(k_), sorted((str(a_), str(b_)) for a_, b_ in v_.items()) if isinstance(v_, dict) else str(v_)]
                   for k_, v_ in _hints.items())
    print(json.dumps({'models': shas, 'plan': hashlib.sha256(json.dumps(_plan).encode()).hexdigest()[:16],
                      'runs': sum((_rep.get('run_layers') or {}).values()), 'pairs': _rep.get('pairs_escaped')}))
    sys.exit(0)
with contextlib.redirect_stdout(io.StringIO()):
    import awx_settings  # noqa: E402
    import joint_escape as je  # noqa: E402
    from kicad_parser import parse_kicad_pcb  # noqa: E402

BAD = []


def check(ok, what):
    print(f'  {"ok" if ok else "FAIL"}: {what}')
    if not ok:
        BAD.append(what)


def synth_board(tmp):
    raw = os.path.join(tmp, 'bus.kicad_pcb')
    r = subprocess.run([sys.executable, 'synth_bus.py', raw, '--k', '8', '--pairs', '2', '--pattern', 'shuffle',
                        '--seed', '1'], cwd=AWX, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(raw):
        raise SystemExit(f'BROKEN TEST: synth_bus wrote no board -- {(r.stderr or r.stdout)[-300:]}')
    return raw


def check_plan_reproducible(tmp):
    print('3. the whole-array plan on four routing layers is the same in every process (two string-hash seeds)')
    raw = os.path.join(tmp, 'bus4.kicad_pcb')
    r = subprocess.run([sys.executable, 'synth_bus.py', raw, '--copper', '4', '--k', '24', '--pairs', '10', '--depth',
                        '4', '--pattern', 'shuffle', '--seed', '2'], cwd=AWX, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(raw):
        raise SystemExit(f'BROKEN TEST: synth_bus wrote no four-copper board -- {(r.stderr or r.stdout)[-300:]}')
    got = {}
    for seed in ('0', '1'):
        env = {k: v for k, v in os.environ.items() if k not in ('ESCAPE_VIAS',)}
        env.update(PYTHONHASHSEED=seed, ROUTE_LAYERS='F.Cu,B.Cu,In1.Cu,In2.Cu', _JOINT_ROUNDS_PLAN='1',
                   _JOINT_ROUNDS_TMP=tmp)
        r = subprocess.run([sys.executable, os.path.abspath(__file__), raw], cwd=AWX, env=env, capture_output=True,
                           text=True)
        out = [ln for ln in r.stdout.splitlines() if ln.startswith('{')]
        if r.returncode != 0 or not out:
            raise SystemExit(f'BROKEN TEST: the plan at hash seed {seed} -- rc {r.returncode}: '
                             f'{(r.stderr or r.stdout)[-300:]}')
        got[seed] = json.loads(out[-1])
    a, b = got['0'], got['1']
    # (the case reaches what it pins: runs on no one layer yet, pairs among them -- else it proves nothing)
    if not a['runs'] or not a['pairs'] or len(a['models']) < 2:
        raise SystemExit(f'BROKEN TEST: the generated bus planned {a["runs"]} run(s) and {a["pairs"]} pair(s) in '
                         f'{len(a["models"])} model(s) -- the case no longer reaches the rows it pins')
    check(a['models'] == b['models'], f'every model the plan solves is the same under both seeds '
                                      f'({len(a["models"])} models: {a["models"]} / {b["models"]})')
    check(a['plan'] == b['plan'], f'the plan is the same under both seeds ({a["plan"]} / {b["plan"]})')


def check_avoid_layer(tmp):
    print('6. the whole-array plan keeps every bus net off an avoided place on its layer, and only on its layer')
    raw = os.path.join(tmp, 'bus6.kicad_pcb')
    # (balls 0.5 mm across: no track between two, so the deeper balls leave by a via and a run)
    r = subprocess.run([sys.executable, 'synth_bus.py', raw, '--copper', '4', '--k', '16', '--depth', '4', '--pad',
                        '0.5', '--pattern', 'shuffle', '--seed', '1'], cwd=AWX, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(raw):
        raise SystemExit(f'BROKEN TEST: synth_bus wrote no four-copper board -- {(r.stderr or r.stdout)[-300:]}')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(raw)
    bus = sorted(n.name for n in pcb.nets.values() if any(p.component_ref == 'SU1' for p in n.pads)
                 and any(p.component_ref == 'SD1' for p in n.pads))
    short = [nm.split('/')[-1] for nm in bus]

    def plan(avoid=None):
        with awx_settings.given({**awx_settings.environ(), 'ROUTE_LAYERS': 'F.Cu,B.Cu,In1.Cu,In2.Cu'}), \
                contextlib.redirect_stdout(io.StringIO()):
            import route_layers as rl
            hints, rep = je.plan_array(pcb, 'SU1', bus, [], ['F.Cu', 'B.Cu'], far=je.far_face(pcb, 'SU1', 'SD1'),
                                       log=lambda *_a: None, vias_only=rl.escape_vias('src'), avoid=avoid)
        return hints, rep
    at_ = lambda hints, ex, layer: [k for k, v in hints.items() if v.get('layer') == layer       # noqa: E731
                                    and math.hypot(v['exit'][0] - ex[0], v['exit'][1] - ex[1]) < 1e-6]
    h0, rep0 = plan()
    runs = sorted((k, v) for k, v in h0.items() if v.get('layer') == 'B.Cu' and v.get('kind') != 'surface')
    surf = sorted((k, v) for k, v in h0.items() if v.get('layer') == 'F.Cu' and v.get('kind') == 'surface')
    # (the case reaches what it pins: every bus ball served, one by a via and its run on B.Cu, one on the surface)
    if rep0.get('bus_escaped') != len(bus) or not runs or not surf:
        raise SystemExit(f'BROKEN TEST: the free plan served {rep0.get("bus_escaped")} of {len(bus)} bus balls, '
                         f'{len(runs)} by a run on B.Cu, {len(surf)} on the surface')
    ball, mv = runs[0]
    h1, rep1 = plan({nm: [(mv['exit'][0], mv['exit'][1], 'B.Cu')] for nm in short})
    got = h1.get(ball) or {}
    check(rep1.get('bus_escaped') == len(bus) and not at_(h1, mv['exit'], 'B.Cu') and got.get('exit') == mv['exit']
          and got.get('layer') not in (None, 'B.Cu'),
          f'a run\'s exit avoided on B.Cu: every bus ball served ({rep1.get("bus_escaped")} of {len(bus)}), the run '
          f'leaving there on another layer ({got.get("exit")} on {got.get("layer")})')
    ball, mv = surf[0]
    h2, rep2 = plan({nm: [(mv['exit'][0], mv['exit'][1], 'F.Cu')] for nm in short})
    check(rep2.get('bus_escaped') == len(bus) and not at_(h2, mv['exit'], 'F.Cu'),
          f'a surface exit avoided on F.Cu: every bus ball served ({rep2.get("bus_escaped")} of {len(bus)}), none '
          f'leaving there on F.Cu ({at_(h2, mv["exit"], "F.Cu")})')
    h3, _rep3 = plan({nm: [(mv['exit'][0], mv['exit'][1], 'B.Cu')] for nm in short})
    check((h3.get(ball) or {}).get('exit') == mv['exit'] and (h3.get(ball) or {}).get('layer') == 'F.Cu',
          f'the surface exit avoided on B.Cu alone: the ball keeps it on F.Cu ({h3.get(ball)})')


def _pad_xy(board, names):
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    out = {}
    for n in pcb.nets.values():
        nm = n.name.split('/')[-1]
        if nm in names:
            p = next(q for q in n.pads if q.component_ref == 'SU1')
            out[nm] = (p.global_x, p.global_y)
    return out


def main():
    print('=' * 60)
    print('the whole-array plan: the same in every process, an avoided place read on its layer')
    print('=' * 60)
    try:
        import ortools  # noqa: F401  (the plan is a CP-SAT solve)
    except ImportError as exc:
        print('SKIP: needs ortools (%s)' % exc)
        sys.exit(77)
    with tempfile.TemporaryDirectory() as tmp:
        check_plan_reproducible(tmp)
        check_avoid_layer(tmp)
    if BAD:
        for f in BAD:
            print(f'  FAIL: {f}')
        sys.exit(1)
    print('PASS: the whole-array plan is the same in every process; it keeps off an avoided place on its layer')


if __name__ == '__main__':
    main()
