#!/usr/bin/env python3
"""The joint fanout on more routing layers than two: a round hands the next its source, and a source re-fan keeps every
pair whole.

  python3 tests/test_622_joint_rounds.py

The zynq LVDS bus on four layers (route_bus --joint-fanout) had its fanout REFUSED by the fanout audit -- pairs split
at U1's teeth -- in every round after the first, on the Mac as on Linux, and in round 1 on Linux. Two defects:

1. the joint destination never NAMED its source board, which whole_route.advance reads off the round's fo.log
   ('source board: NAME,', as fanout_destination prints it on two layers): every later round started again from the
   bench's own pair-blind source fanout -- 11 pairs split, the same 11 every round 2. Pinned: joint_destination's
   line, cut off before it fans anything out, read back by whole_route.source_board_named as the board it ran on;
2. a source realize stripped only the nets asked to move: the joint plan holds a pair's legs together with no other
   exit between them, but only among the balls it plans -- RX_D4_N asked alone was re-planned beside a partner and a
   neighbouring pair left on the board as obstacles, and laid between RX_D0's legs. Pinned on a generated bus, the
   joint plan and the engine stubbed out to see what the realize asks of them: with four routing layers every run net
   at the source is planned in the one call, each unasked one preferring its tooth as measured; with two, the realize
   is as it was (only the asked net, nothing kept);
3. the joint plan on four routing layers is the same in every process: two of its row loops (a pair's legs' runs in
   one capacity group, the run colouring's clauses) walked SETS of ball and layer names, in the string hash's order,
   and CP-SAT's answer follows the order its rows come in -- zynq U1's realize, the same ask under two hash seeds,
   laid 26 of its 47 teeth the same. Pinned on a generated four-copper bus: the plan's every model, fingerprinted as it
   is solved, and its answer, under two PYTHONHASHSEEDs in processes of their own.
"""
import contextlib
import hashlib
import io
import json
import os
import subprocess
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

if os.environ.get('_JOINT_ROUNDS_PLAN') == '1':
    # (check 3's child: the source array's joint plan of the board given, every model fingerprinted as it is solved)
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
    import fanout_from_plan as ff  # noqa: E402
    import joint_escape as je  # noqa: E402
    import source_realize as sr  # noqa: E402
    import whole_route as wr  # noqa: E402
    from kicad_parser import parse_kicad_pcb  # noqa: E402

BAD = []


def check(ok, what):
    print(f'  {"ok" if ok else "FAIL"}: {what}')
    if not ok:
        BAD.append(what)


class _Stop(Exception):
    pass


def check_source_board_named(tmp):
    print('1. the joint destination names its source board, as the next round reads it')
    spec = os.path.join(tmp, 'joint.json')
    json.dump({'arrays': [{'ref': 'SD1', 'others': [], 'drops': []}, {'ref': 'SU1', 'others': [], 'drops': []}],
               'layers': ['F.Cu', 'B.Cu']}, open(spec, 'w'))
    rdir = os.path.join(tmp, 'r1')
    os.makedirs(rdir)
    board = os.path.join(rdir, 'fo_srcres0_1.kicad_pcb')       # a realized source board, written in the round's dir
    open(board, 'w').close()
    lines = []
    real = ff.parse_kicad_pcb

    def stop(*_a, **_k):
        raise _Stop()
    ff.parse_kicad_pcb = stop                                   # cut off before anything is fanned out
    try:
        with awx_settings.given({**awx_settings.environ(), 'FANOUT_JOINT': spec}):
            ff.joint_destination(os.path.join(rdir, 'fo.kicad_pcb'), [], {}, 'SD1', {}, board, set(),
                                 log=lines.append)
    except _Stop:
        pass
    finally:
        ff.parse_kicad_pcb = real
    log = os.path.join(rdir, 'fo.log')
    with open(log, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    got = wr.source_board_named(log)
    check(got == os.path.basename(board), f'whole_route reads the source board off the joint destination\'s log: '
                                          f'{got!r} (want {os.path.basename(board)!r})')
    check(os.path.isfile(os.path.join(rdir, got or '')), 'the name it reads is the board in the round\'s directory')


def synth_board(tmp):
    raw = os.path.join(tmp, 'bus.kicad_pcb')
    r = subprocess.run([sys.executable, 'synth_bus.py', raw, '--k', '8', '--pairs', '2', '--pattern', 'shuffle',
                        '--seed', '1'], cwd=AWX, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(raw):
        raise SystemExit(f'BROKEN TEST: synth_bus wrote no board -- {(r.stderr or r.stdout)[-300:]}')
    return raw


def realize_asks(board, tmp, route_layers, tag):
    """realize() of ONE asked net under the joint fanout, the joint plan and the engine stubbed: (the bus the plan was
    asked for, its preferences, the realize's result) -- the plan None when the realize never asked one"""
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    src_pad = {}
    for nm, (_i, n) in byname.items():
        p = next((q for q in n.pads if q.component_ref == 'SU1'), None)
        if p is not None and any(q.component_ref == 'SD1' for q in n.pads):
            src_pad[nm] = p
    names = sorted(src_pad)
    if len(names) < 4:
        raise SystemExit(f'BROKEN TEST: the generated bus has {len(names)} run nets at SU1')
    asked = names[0]
    p = src_pad[asked]
    move = types.SimpleNamespace(exit_pt=(p.global_x + 9.0, p.global_y), direction='right', layer='F.Cu',
                                 kind='surface', site=None, legs=[], vias=0)
    spec = os.path.join(tmp, f'joint_{tag}.json')
    json.dump({'arrays': [{'ref': 'SU1', 'others': [], 'drops': []}, {'ref': 'SD1', 'others': [], 'drops': []}],
               'layers': ['F.Cu', 'B.Cu']}, open(spec, 'w'))
    seen = {}

    def plan_array(pcb_, ref, bus, others, layers, **kw):
        seen['bus'], seen['prefer'] = list(bus), dict(kw.get('prefer') or {})
        return {}, {'status': 'STUB', 'pairs_escaped': 0, 'pairs': 0, 'others_escaped': 0, 'others_strapped': 0,
                    'others_balls': 0, 'dropped': 0, 'plane_balls': 0, 'hands_held': [], 'hands_free': []}

    def measured(pcb_, nm, pad, byname_, **_k):
        # every run net's tooth as if laid: straight out of its ball, 5 mm right, on F
        return {'tooth': (round(pad.global_x + 5.0, 3), round(pad.global_y, 3)), 'layer': 'F.Cu', 'vias': 0,
                'kind': 'surface', 'direction': 'right', 'bearing': 'right', 'site': None}
    saved = (je.plan_array, sr.generate_bga_fanout, sr.measure_tooth, sr.te.endpoints)
    je.plan_array = plan_array
    sr.generate_bga_fanout = lambda *a, **k: ([], [], [], [])
    sr.measure_tooth = measured
    # (the drift guard's reading of the unasked teeth: the generated board has no copper to walk)
    sr.te.endpoints = lambda pcb_, nms, byname_, **_k: {nm: ((0.0, 0.0), (0.0, 0.0)) for nm in nms}
    try:
        with awx_settings.given({**awx_settings.environ(), 'FANOUT_JOINT': spec, 'ROUTE_LAYERS': route_layers}), \
                contextlib.redirect_stdout(io.StringIO()):
            res = sr.realize(board, {asked: move}, src_pad, byname, 'SU1', os.path.join(tmp, f'out_{tag}.kicad_pcb'),
                             log=lambda *_a: None, guard_names=names)
    finally:
        je.plan_array, sr.generate_bga_fanout, sr.measure_tooth, sr.te.endpoints = saved
    return asked, names, seen.get('bus'), seen.get('prefer'), res


def check_bus_replanned(tmp):
    print('2. a source realize under the joint fanout plans the whole bus, each unasked tooth preferred where it is')
    board = synth_board(tmp)
    asked, names, bus, prefer, res = realize_asks(board, tmp, 'F.Cu,B.Cu,In1.Cu,In2.Cu', 'four')
    full = sorted(n.split('/')[-1] for n in (bus or []))
    check(bus is not None, 'four routing layers: the realize asks the joint plan')
    check(full == names, f'every run net at the source is planned in the one call ({len(full)} of {len(names)})')
    check(sorted(res.get('kept') or []) == sorted(n for n in names if n != asked),
          f'the realize reports the {len(names) - 1} unasked nets as kept')
    ok_pref = bool(prefer) and all(
        prefer.get(nm, {}).get('tooth') == (round(sr_pad_x + 5.0, 3), round(sr_pad_y, 3))
        for nm, (sr_pad_x, sr_pad_y) in _pad_xy(board, names).items() if nm != asked)
    check(ok_pref, 'each unasked net prefers its tooth as measured before the strip')
    check(prefer.get(asked, {}).get('direction') == 'right' and prefer.get(asked, {}).get('layer') == 'F.Cu'
          if prefer else False, 'the asked net prefers its ask')
    print('   two routing layers: as before')
    _a, _n, bus2, _p2, res2 = realize_asks(board, tmp, 'F.Cu,B.Cu', 'two')
    check(bus2 is None, 'two routing layers: no joint plan is asked (the asked teeth are laid as the chain lays them)')
    check(not res2.get('kept'), 'two routing layers: nothing kept -- only the asked net is stripped')


def check_plan_reproducible(tmp):
    print('3. the joint plan on four routing layers is the same in every process (two string-hash seeds)')
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
    print('the joint fanout\'s rounds: the source handed on, every pair re-planned whole')
    print('=' * 60)
    with tempfile.TemporaryDirectory() as tmp:
        check_source_board_named(tmp)
        check_bus_replanned(tmp)
        check_plan_reproducible(tmp)
    if BAD:
        for f in BAD:
            print(f'  FAIL: {f}')
        sys.exit(1)
    print('PASS: the next round starts from the source the round realized; a source re-fan plans every pair whole; '
          'the joint plan is the same in every process')


if __name__ == '__main__':
    main()
