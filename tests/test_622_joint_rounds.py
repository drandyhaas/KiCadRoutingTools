#!/usr/bin/env python3
"""The whole-array fanout on more routing layers than two: a round hands the next its source, and a source re-fan keeps
every pair whole.

  python3 tests/test_622_joint_rounds.py

The zynq LVDS bus on four layers (route_bus --whole-array) had its fanout REFUSED by the fanout audit -- pairs split
at U1's teeth -- in every round after the first, on the Mac as on Linux, and in round 1 on Linux. Two defects:

1. the joint destination never NAMED its source board, which whole_route.advance reads off the round's fo.log
   ('source board: NAME,', as fanout_destination prints it on two layers): every later round started again from the
   bench's own pair-blind source fanout -- 11 pairs split, the same 11 every round 2. Pinned: joint_destination's
   line, cut off before it fans anything out, read back by whole_route.source_board_named as the board it ran on;
2. a source realize stripped only the nets asked to move: the whole-array plan holds a pair's legs together with no
other
   exit between them, but only among the balls it plans -- RX_D4_N asked alone was re-planned beside a partner and a
   neighbouring pair left on the board as obstacles, and laid between RX_D0's legs. Pinned on a generated bus, the
   whole-array plan and the engine stubbed out to see what the realize asks of them: with four routing layers every run
   net at the source is planned in the one call, each unasked one preferring its tooth as measured; with two, the
   realize is as it was (only the asked net, nothing kept);
3. the whole-array plan on four routing layers is the same in every process: two of its row loops (a pair's legs' runs
in
   one capacity group, the run colouring's clauses) walked SETS of ball and layer names, in the string hash's order,
   and CP-SAT's answer follows the order its rows come in -- zynq U1's realize, the same ask under two hash seeds,
   laid 26 of its 47 teeth the same. Pinned on a generated four-copper bus: the plan's every model, fingerprinted as it
   is solved, and its answer, under two PYTHONHASHSEEDs in processes of their own;
4. a later round carries the array's other balls' pieces from the round before (joint_escape.carry) onto its board,
   checked against other nets' copper only -- so a piece the board already had read clear and was laid again on itself
   (sdi_mipi_conv's DDR3 bus after the SDI bus on the shared U11: 168 vias stacked on their own copies). Pinned on the
   generated bus with three balls dog-boned: carried onto the bare board each piece is laid once, onto the fanned board
   none again;
5. a round's blocked teeth (whole_route.put_blocked: no escape of their ball's clears) are carried to every later round,
   whose ends model prices them where it chooses the teeth -- but the source realize re-plans the whole bus, and its
   whole-array plan moved the nets the ends model did not ask for wherever it liked: a memory bus's address line was
   put back on its blocked tooth in each of the three rounds after the one that found it, and left open in all four;
   kept off it alone, its neighbour took the place and was blocked there in its stead. Pinned on the generated bus,
   the plan stubbed: the realize asks it to keep every net it places off each blocked tooth at the source, on its
   layer -- a single's, both of a pair's, one a net it does not plan found -- and off no crowded tooth that is not
   blocked, no destination's end;
6. ...and the whole-array plan reads such a place on its layer alone. Priced, a run there was paid for by whichever
   ball took the exit, on whichever layer the colouring gave it, so the plan took the same exit as unpriced and its run
   was coloured onto the blocked layer again. On a generated four-copper bus: a run's exit avoided on its layer, the
   run leaves there on another, every bus ball served; a surface exit avoided on its layer, no ball leaves there on it;
   avoided on the other outer layer alone, the ball keeps its exit.
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
import types

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


def realize_asks(board, tmp, route_layers, tag, env=None, seen=None):
    """realize() of ONE asked net under the whole-array fanout, the whole-array plan and the engine stubbed: (the bus
    the plan was asked for, its preferences, the realize's result) -- the plan None when the realize never asked one.
    `env`: settings over the run's; `seen`: filled with what the plan was asked, its `avoid` among it"""
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
    seen = {} if seen is None else seen

    def plan_array(pcb_, ref, bus, others, layers, **kw):
        seen['bus'], seen['prefer'], seen['avoid'] = list(bus), dict(kw.get('prefer') or {}), kw.get('avoid')
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
        with awx_settings.given({**awx_settings.environ(), 'FANOUT_JOINT': spec, 'ROUTE_LAYERS': route_layers,
                                 **(env or {})}), \
                contextlib.redirect_stdout(io.StringIO()):
            res = sr.realize(board, {asked: move}, src_pad, byname, 'SU1', os.path.join(tmp, f'out_{tag}.kicad_pcb'),
                             log=lambda *_a: None, guard_names=names)
    finally:
        je.plan_array, sr.generate_bga_fanout, sr.measure_tooth, sr.te.endpoints = saved
    return asked, names, seen.get('bus'), seen.get('prefer'), res


def check_bus_replanned(tmp):
    print('2. a source realize under the whole-array fanout plans the whole bus, each unasked tooth preferred where '
          'it is')
    board = synth_board(tmp)
    asked, names, bus, prefer, res = realize_asks(board, tmp, 'F.Cu,B.Cu,In1.Cu,In2.Cu', 'four')
    full = sorted(n.split('/')[-1] for n in (bus or []))
    check(bus is not None, 'four routing layers: the realize asks the whole-array plan')
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
    check(bus2 is None, 'two routing layers: no whole-array plan is asked (the asked teeth are laid as the chain lays '
          'them)')
    check(not res2.get('kept'), 'two routing layers: nothing kept -- only the asked net is stripped')


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


def check_carry_once(tmp):
    print('4. a later round carries the array\'s other balls\' pieces onto its board once, never onto a copy of their own')
    raw = synth_board(tmp)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(raw)
    balls = sorted((p for p in pcb.footprints['SU1'].pads if p.net_id), key=lambda p: p.pad_number)[:3]
    if len(balls) < 3:
        raise SystemExit(f'BROKEN TEST: the generated bus has {len(balls)} netted balls at SU1')
    # each ball's piece: a 0.4 mm dog-bone leg on F to a via, as a whole-array field lays an other net's ball
    tracks = [{'start': (p.global_x, p.global_y), 'end': (p.global_x + 0.4, p.global_y + 0.4), 'width': 0.12,
               'layer': 'F.Cu', 'net_id': p.net_id} for p in balls]
    vias = [{'x': p.global_x + 0.4, 'y': p.global_y + 0.4, 'size': 0.45, 'drill': 0.2, 'layers': ['F.Cu', 'B.Cu'],
             'net_id': p.net_id} for p in balls]
    fanned = os.path.join(tmp, 'fanned.kicad_pcb')
    from kicad_writer import add_tracks_and_vias_to_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        add_tracks_and_vias_to_pcb(raw, fanned, tracks, vias, [],
                                   net_id_to_name={i: n.name for i, n in pcb.nets.items()})
    others = [p.net_name for p in balls]

    def counts(path):
        with contextlib.redirect_stdout(io.StringIO()):
            b = parse_kicad_pcb(path)
        return len(b.segments), len(b.vias)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        _a, kept_bare, _m = je.carry(fanned, raw, os.path.join(tmp, 'onto_bare.kicad_pcb'), 'SU1', others, [])
        _a, kept_self, _m = je.carry(fanned, fanned, os.path.join(tmp, 'onto_self.kicad_pcb'), 'SU1', others, [])
    # (the case reaches what it pins: every piece stands, on both boards)
    if kept_bare != len(balls) or kept_self != len(balls):
        raise SystemExit(f'BROKEN TEST: {kept_bare} / {kept_self} of {len(balls)} pieces stand -- the case no longer '
                         f'reaches the lay')
    check(counts(os.path.join(tmp, 'onto_bare.kicad_pcb')) == (len(balls), len(balls)),
          f'onto the bare board, each piece laid once: {counts(os.path.join(tmp, "onto_bare.kicad_pcb"))}')
    check(counts(os.path.join(tmp, 'onto_self.kicad_pcb')) == counts(fanned),
          f'onto a board that has them, none laid again: {counts(os.path.join(tmp, "onto_self.kicad_pcb"))} '
          f'vs {counts(fanned)}')


def check_blocked_avoided(tmp):
    print('5. a source realize under the whole-array fanout keeps its plan off the run\'s blocked teeth')
    import pairs as _pairs
    board = synth_board(tmp)
    seen0 = {}
    _a, names, bus, _p, _r = realize_asks(board, tmp, 'F.Cu,B.Cu,In1.Cu,In2.Cu', 'free', env={'ENDS_BLOCKED': ''},
                                          seen=seen0)
    prs = _pairs.pair_names(names)
    legs = {leg for pr in prs.values() for leg in pr}
    singles = [nm for nm in names if nm not in legs]
    # (the case reaches what it pins: the plan asked, a pair and three singles among the realize's nets)
    if bus is None or not prs or len(singles) < 3:
        raise SystemExit(f'BROKEN TEST: the generated bus gives {len(prs)} pair(s) and {len(singles)} single(s), the '
                         f'plan {"asked" if bus is not None else "never asked"}')
    check(seen0.get('avoid') is None, f'nothing blocked: the plan avoids nothing ({seen0.get("avoid")})')
    base, (leg_p, leg_n) = sorted(prs.items())[0]
    s1, s2, s3 = singles[-3:]
    path = os.path.join(tmp, 'blocked_ends.json')
    json.dump({'avoid': [
        {'lane': s1, 'end': 0, 'points': [[1.0, 2.0]], 'layer': 'In1.Cu', 'blocked': True},
        {'lane': base, 'end': 0, 'points': [[3.0, 4.0], [3.0, 4.8]], 'layer': 'In1.Cu', 'blocked': True},
        # (a feedback round's crowded tooth, not blocked: the ends model's to price, as before)
        {'lane': s2, 'end': 0, 'points': [[5.0, 6.0]], 'layer': 'F.Cu'},
        # (a blocked end at the DESTINATION; and one a net this realize does not plan found, its place dead all the
        # same)
        {'lane': s3, 'end': 1, 'points': [[7.0, 8.0]], 'layer': 'F.Cu', 'blocked': True},
        {'lane': 'NOT_A_BUS_NET', 'end': 0, 'points': [[9.0, 9.0]], 'layer': 'F.Cu', 'blocked': True}]},
        open(path, 'w'))
    seen = {}
    realize_asks(board, tmp, 'F.Cu,B.Cu,In1.Cu,In2.Cu', 'blocked', env={'ENDS_BLOCKED': path, 'FEEDBACK': ''},
                 seen=seen)
    got = {k: sorted(tuple(p_) for p_ in v) for k, v in (seen.get('avoid') or {}).items()}
    places = sorted([(1.0, 2.0, 'In1.Cu'), (3.0, 4.0, 'In1.Cu'), (3.0, 4.8, 'In1.Cu'), (9.0, 9.0, 'F.Cu')])
    check(sorted(got) == sorted(names), f'every net the plan places avoids them ({len(got)} of {len(names)})')
    check(all(v == places for v in got.values()),
          f'each avoids every blocked tooth at the source on its layer -- a single\'s, both of a pair\'s, one found '
          f'by a net it does not plan -- and no other: {sorted(set(map(tuple, got.values())))}')


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
    print('the whole-array fanout\'s rounds: the source handed on, every pair re-planned whole')
    print('=' * 60)
    with tempfile.TemporaryDirectory() as tmp:
        check_source_board_named(tmp)
        check_bus_replanned(tmp)
        check_carry_once(tmp)
        check_blocked_avoided(tmp)
        try:
            import ortools  # noqa: F401  (check 3's whole-array plan is a CP-SAT solve)
        except ImportError as exc:
            if BAD:
                for f in BAD:
                    print(f'  FAIL: {f}')
                sys.exit(1)
            print('SKIP: checks 3 and 6 need ortools (%s); checks 1, 2, 4 and 5 passed' % exc)
            sys.exit(77)
        check_plan_reproducible(tmp)
        check_avoid_layer(tmp)
    if BAD:
        for f in BAD:
            print(f'  FAIL: {f}')
        sys.exit(1)
    print('PASS: the next round starts from the source the round realized; a source re-fan plans every pair whole; '
          'the whole-array plan is the same in every process; a carried piece is laid once; the plan keeps off the '
          'blocked teeth, on their layers')


if __name__ == '__main__':
    main()
