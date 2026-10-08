#!/usr/bin/env python3
"""route_bus.py -- the bus step: an array pair's bus routed on the board as it is.

    python3 route_bus.py BOARD.kicad_pcb OUT.kicad_pcb [--src U1 --dest U2] [--k K] [--rounds R] [--inproc]
                         [--clearance C] [--track-width W] [--fanout-track-width W] [--via-size D] [--via-drill D]
                         [--json-out FILE]
    python3 route_bus.py BOARD.kicad_pcb --find

Without --src and --dest, every bus on the board (route_buses): find_buses names each pair of parts sharing
MIN_BUS_NETS nets or more the whole route admits, and takes those with a ball array at both ends (is_array), the
larger array the source, the largest bus first; each is routed in turn on the board the one before handed on, and
OUT is the last. A pair with a row part at one end is named and left to the router, and so is each net on both
parts the whole route does not admit, with why. --find names them and stops.

route_bus() takes a board as the chain hands it on and writes it back with the bus laid, in the board's own frame,
at the chain's sizes (resolve_rules: the routing CLIs' flags, an omitted one resolved as route.py resolves it):

1. The bus: every net between SRC and DEST the ladder admits (make_bench.pair_nets). Refused: a pair through a
   termination part between the arrays (the whole route lays no waypoint); a net whose name's last part is another
   net's too (name_clashes: the whole route names a net by it); and a pair leg whose partner is refused or is no
   lane between the arrays (half_pairs: a pair is laid whole or not at all). The bus's copper, if any, is stripped:
   the step lays both ends of every lane.
2. SRC is fanned out for them (make_bench.fanout_source), the project floor stamped, and the ladder written: the
   plan's rivers, largest first.
3. The board is turned into the flow frame (flow_frame.py): the quarter turn that points the pair's source-to-
   destination direction along +x, about a point on the 0.1 mm lattice, so the turn is exact both ways.
4. The whole route (whole_route.chain) on the ladder's first K nets, the whole ladder by default.
5. The routed board -- the whole route's best round, which never ends with nothing while it had a plan to lay -- is
   turned back into the board's own frame. A bus net the run did not route, or left open, leaves as it came: none
   of the step's copper, its own put back.
6. It is graded there: the nets handed on connected (check_connected), no violation on a bus net and the whole
   board no worse than it came (check_drc, each board at its own project's floor).
7. Handed on only clean: OUT is the routed board, its routed nets protected in its project (#521: route.py's rip
   candidacy and the plane finalize leave them alone, as do the nets the board came with protected -- an earlier
   bus's), the open ones left to the chain's A* -- else OUT is the board as it came, and A* routes the whole bus.
   Either way the summary (--json-out, default <OUT>.bus/summary.json) names the nets routed and refused, and why.

A board with inner copper layers is taken as it is: the lanes run on the ROUTING LAYERS (route_layers:
ROUTE_LAYERS, F.Cu and B.Cu by default, and the inner ones it names), and every via is a through via. A routing layer
that carries a POUR (a plane: zynq's In1 GND and In2 VCC_1V8) is crossed by the lanes: the pour's nets are graded on
the board as it came and as routed, each refilled (kicad_exact_fill), and a split the bus made is named. The caches are off. Everything the step writes goes in OUT's work directory,
<OUT stem>.bus/, beside OUT.

Exits 0 when the bus is handed on routed, connected and clean (every bus, without --src and --dest); 1 when OUT is
the board as it came (a bus of them left as it came, or none found); 2 on a usage error.
"""

KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import collections
import contextlib
import io
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

# the caches are a harness's (awx-caches-off-by-default): the step runs every stage without them
STEP_SETTINGS = {'TAUT_MEMO': '0', 'PROBE_MEMO': '0', 'STAGE_CACHE': '0'}

# the sizes route_bus takes, as the routing CLIs spell them (route.py's, and the escapes' own track)
SIZE_ARGS = ('clearance', 'clearance_ceiling', 'track_width', 'fanout_track_width', 'via_size', 'via_drill',
             'hole_to_hole_clearance', 'board_edge_clearance')


def step_policy(run_rules=None):
    """The settings the step's awx modules are imported and run under: the chain's stage policy (whole_route.stage_env
    -- a pair one lane, the ends' judge), the caches off, and the run's rules when given. A module reads its policy
    when it is FIRST imported in a process (awx_settings), and the chain's in-process stages run on the modules the
    step imported before them: imported under the shell's environment, braid read BRAID_PAIRS off, and every stage
    planned and laid a pair as two single lanes (the zynq's DQS0 and DQS1, the AD9364's fourteen LVDS pairs)"""
    import awx_settings
    import whole_route as wr
    env = dict(os.environ)
    env.update(STEP_SETTINGS)
    wr.stage_env(env)
    if run_rules is not None:
        import rules as _rules
        env[_rules.SETTING] = _rules.as_setting(run_rules)
    return awx_settings.given(env)


def resolve_rules(board, clearance=None, clearance_ceiling=None, track_width=None, fanout_track_width=None,
                  via_size=None, via_drill=None, hole_to_hole_clearance=None, board_edge_clearance=None, log=print):
    """The run's rules: the chain's sizes, resolved as route.py resolves its own, through Rules.from_router_config
    (rules.py's one seam; rules.py itself never reads a board).

    A size given is taken. One omitted is the board's Default net class's, else routing_defaults'. The clearance is
    capped at `clearance_ceiling` (min of the two, route.py's reading). The hole-to-hole and edge floors are the
    board's own when not given (list_nets.resolve_cli_floor). Every size is pinned up to the fab floor for the
    board's copper layer count (fab_tiers.enforce_fab_floors). The escapes' track is `fanout_track_width`, else the
    lanes' -- the chain lays its fanout narrower than its routes, and a router config has one track."""
    import dataclasses
    import types
    import routing_defaults as defaults
    import rules as _rules
    from list_nets import board_default_netclass_param, resolve_cli_floor
    from fab_tiers import enforce_fab_floors, count_copper_layers_in_file
    got = {}
    for name, key, given, fallback in (('track_width', 'track_width', track_width, defaults.TRACK_WIDTH),
                                       ('via_size', 'via_diameter', via_size, defaults.VIA_SIZE),
                                       ('via_drill', 'via_drill', via_drill, defaults.VIA_DRILL),
                                       ('clearance', 'clearance', clearance, defaults.CLEARANCE)):
        if given is None:
            v = board_default_netclass_param(board, key)
            given = v if v is not None else fallback
            log(f'  --{name.replace("_", "-")} not given: {given} mm, '
                f'{"the board Default net class" if v is not None else "the fallback"}')
        got[name] = float(given)
    if clearance_ceiling is not None and clearance_ceiling < got['clearance']:
        log(f'  --clearance-ceiling {clearance_ceiling}: the clearance {got["clearance"]} capped at it')
        got['clearance'] = float(clearance_ceiling)
    got['hole_to_hole_clearance'] = resolve_cli_floor(board, 'hole_to_hole', hole_to_hole_clearance,
                                                      defaults.HOLE_TO_HOLE_CLEARANCE, '--hole-to-hole-clearance')
    got['board_edge_clearance'] = resolve_cli_floor(board, 'board_edge_clearance', board_edge_clearance,
                                                    defaults.BOARD_EDGE_CLEARANCE, '--board-edge-clearance')
    n_cu = count_copper_layers_in_file(board)
    got.update(enforce_fab_floors(n_cu, **got))
    fan = float(fanout_track_width) if fanout_track_width is not None else got['track_width']
    fan = enforce_fab_floors(n_cu, track_width=fan).get('track_width', fan)
    r = _rules.Rules.from_router_config(types.SimpleNamespace(**got), fan_track=fan)
    return dataclasses.replace(r, source='the chain\'s sizes, resolved as route.py resolves its own')


def route_bus(board, out, src, dest, k=None, rounds=3, inproc=False, log=print, json_out=None, joint_fanout=False,
              **sizes):
    """Route the bus between `src` and `dest` on `board`, writing `out`, at the chain's sizes (`sizes`: SIZE_ARGS,
    resolve_rules). Returns (exit code, grade line). The run's rules are installed for the step and supplied to
    every stage (rules.SETTING); the process's own are put back after."""
    import rules as _rules
    bad = sorted(set(sizes) - set(SIZE_ARGS))
    if bad:
        raise TypeError(f'route_bus: no size {bad}')
    run_rules = resolve_rules(board, log=log, **sizes)
    prev = _rules.active()
    _rules.install(run_rules)
    try:
        with step_policy(run_rules):
            return _route(board, out, src, dest, k, rounds, inproc, log, run_rules, json_out, joint_fanout)
    finally:
        _rules.install(prev)


def route_buses(board, out, buses=None, k=None, rounds=3, inproc=False, log=print, json_out=None,
                joint_fanout=False, **sizes):
    """Every bus on `board` routed in turn, each on the board the one before handed on, `out` the last: `buses` as
    [(src, dest)], else the ones find_buses takes, the largest first. Returns (exit code, grade lines): 0 when every
    bus was handed on routed, 1 when one or more left as it came (or none was found), 2 on a usage error. The
    summary (--json-out, default <OUT>.bus/summary.json) holds each bus's own, and the buses found and not taken."""
    from kicad_parser import parse_kicad_pcb
    from copy_board import copy_board
    board, out = os.path.abspath(board), os.path.abspath(out)
    if not out.endswith('.kicad_pcb'):
        out += '.kicad_pcb'
    work = out[:-len('.kicad_pcb')] + '.bus'
    os.makedirs(work, exist_ok=True)
    found = None
    if buses is None:
        with contextlib.redirect_stdout(sys.stderr):
            pcb = parse_kicad_pcb(board)
        found = find_buses(pcb)
        for d in found:
            log(f"route_bus: {'bus' if d['taken'] else 'not taken'} {d['src']} -> {d['dest']}, {len(d['nets'])} nets"
                + (f" -- {d['why']}" if d['why'] else ''))
        buses = [(d['src'], d['dest']) for d in found if d['taken']]
    cur, runs = board, []
    for s, d in buses:
        step_out = os.path.join(work, f'{s}-{d}.kicad_pcb')
        step_json = os.path.join(work, f'{s}-{d}.summary.json')
        code, g = route_bus(cur, step_out, s, d, k, rounds, inproc, log, json_out=step_json,
                            joint_fanout=joint_fanout, **sizes)
        runs.append(dict(src=s, dest=d, exit=code, grade=g, summary=step_json))
        if os.path.isfile(step_out):       # (routed, or the board as it came)
            cur = step_out
    with contextlib.redirect_stdout(sys.stderr):
        copy_board(cur, out)
    code = 1 if not runs else max(r['exit'] for r in runs)
    with open(json_out or os.path.join(work, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(board=board, out=out, buses=runs, found=found, exit=code), f, indent=1)
    if not runs:
        log('route_bus: no bus found -- OUT is the board as it came')
    return code, [r['grade'] for r in runs]


def _route(board, out, src, dest, k, rounds, inproc, log, run_rules, json_out=None, joint_fanout=False):
    import make_bench as mb
    import flow_frame as ff
    import whole_route as wr
    from coherent_nets import coherent_nets
    from kicad_parser import parse_kicad_pcb
    from fix_kicad_drc_settings import fix_project_for_output
    import braid as te
    import source_realize as sr
    import rules as _rules

    t0 = time.time()
    board, out = os.path.abspath(board), os.path.abspath(out)
    if not out.endswith('.kicad_pcb'):
        out += '.kicad_pcb'
    work = out[:-len('.kicad_pcb')] + '.bus'
    os.makedirs(work, exist_ok=True)
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    for r in (src, dest):
        if r not in pcb.footprints:
            log(f'route_bus: no footprint {r} on the board')
            return 2, None
    log(f'route_bus: {os.path.basename(board)}, copper layers {pcb.board_info.copper_layers}, {src} -> {dest}')

    # 1. the bus, its copper stripped
    names = mb.pair_nets(pcb, src, dest)
    log(f'  {len(names)} nets between {src} and {dest}')
    wps = waypoint_pairs(pcb, names, src, dest)
    refused = {leg: f'passes through {part}, a termination part between the arrays: the whole route lays no waypoint'
               for leg, part in wps.items()}
    if wps:
        names = [n for n in names if n.split('/')[-1] not in wps]
        log('  refused, left to the rest of the chain with their copper as it is: '
            + ', '.join(f'{leg} (through {part})' for leg, part in sorted(wps.items()))
            + ' -- a pair that passes through a termination part between the arrays; the whole route lays no '
              'waypoint')
    clash = name_clashes(pcb, names)
    if clash:
        refused.update(clash)
        names = [n for n in names if n.split('/')[-1] not in clash]
        log('  refused, left to the rest of the chain: '
            + '; '.join(f'{n}: {why}' for n, why in sorted(clash.items())))
    halves = half_pairs(pcb, names)
    if halves:
        refused.update(halves)
        names = [n for n in names if n.split('/')[-1] not in halves]
        log('  refused, left to the rest of the chain with their pair: '
            + '; '.join(f'{leg}: {why}' for leg, why in sorted(halves.items())))
    txt, n_cut = mb.strip_pair_copper(open(board, encoding='utf-8').read(), pcb, names)
    if n_cut:
        log(f'  their copper stripped ({n_cut} chars)')
    base = os.path.join(work, 'base.kicad_pcb')
    with open(base, 'w', encoding='utf-8') as f:
        f.write(txt)
    import fanout_from_plan as fp
    fp.copy_pro(board, base)

    # 2. the source fanned out, the project floor, the ladder
    log(f'  rules: clearance {te.SPEC_CLEARANCE}, track {te.TRACK}, fanout {sr.FAN_TRACK}/{sr.FAN_CLEAR}, '
        f'via {te.VIA_SIZE}/{te.VIA_DRILL}  [{run_rules.source}]')
    fanned = os.path.join(work, 'fanned.kicad_pcb')
    # the joint fanout (opt-in): the bus is laid by its own engine call, at the source here and at both arrays each
    # whole-route round, round a via site kept in every other ball of the arrays (joint_escape.reserve_ball_vias);
    # right after, each round, every array's other nets and plane balls are planned and laid jointly round the bus
    # copper just laid (fanout_from_plan.joint_others)
    others = {r: array_others(pcb, r, names) for r in (src, dest)} if joint_fanout else {src: [], dest: []}
    joint, spec = {}, None
    if joint_fanout:
        zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
        # (every copper layer: an escape on a plane layer is the plan's to take only where the outer layers have no
        # room, priced -- joint_escape.plane_layers, C_PLANE_LAYER)
        spec = {'layers': list(pcb.board_info.copper_layers),
                'arrays': [{'ref': r, 'others': others[r],
                            'drops': sorted({p.net_name for p in pcb.footprints[r].pads if p.net_id in zone_ids})}
                           for r in (src, dest)]}
        spec_path = os.path.join(work, 'joint.json')
        with open(spec_path, 'w', encoding='utf-8') as f:
            json.dump(spec, f, indent=1)
        joint = dict(FANOUT_JOINT=spec_path)
        for ar in spec['arrays']:
            log(f'  joint fanout of {ar["ref"]} each round: {len(ar["others"])} other nets on '
                + '/'.join(spec['layers']) + f', the plane balls of {len(ar["drops"])} nets dropped')
    # (with the joint fanout too, the bus's source teeth are the chain's -- its own source fanout, the other balls' via
    # sites kept free (reserve) -- and the rounds plan the array's other balls and plane balls round them. A comb from
    # the array's joint plan chose the teeth for every ball's sake and the bus's order toward the destination lost: the
    # zynq DDR's, laid, the ends model's 86 vias with six nets over two and two ends blocked in front against the
    # chain's teeth's 70, its solve 264 crossings to the chain's 202; round the chain's teeth the joint plan still
    # serves every other net of U1 and 131 of its 132 plane balls)
    with contextlib.redirect_stdout(sys.stderr):
        n_t, n_v, failed = mb.fanout_source(base, fanned, src, names, reserve=spec)
    with contextlib.redirect_stdout(sys.stderr):
        fix_project_for_output(fanned, clearance=te.SPEC_CLEARANCE, track_width=sr.FAN_TRACK,
                               via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL, verbose=False)
    log(f'  {src} fanned out: {n_t} tracks, {n_v} vias' + (f', refused {failed}' if failed else ''))
    with contextlib.redirect_stdout(sys.stderr):
        mb.write_ladder(fanned, names, log=log)
        ladder = coherent_nets(10 ** 6, fanned)
    K = min(int(k), len(ladder)) if k else len(ladder)
    nets = ladder[:K]
    log(f'  the run: K={K} of the ladder\'s {len(ladder)} nets')

    # 3. the flow frame
    with contextlib.redirect_stdout(sys.stderr):
        q, cx, cy = ff.quarter_of(parse_kicad_pcb(fanned), dest, set(nets))
    frame = os.path.join(work, 'frame.kicad_pcb')
    bad = ff.turn_file(fanned, frame, q, cx, cy)
    if bad:
        log('route_bus: the turn into the flow frame failed: ' + '; '.join(bad[:6]))
        return 2, None
    log(f'  turned into the flow frame: {q} quarter turn(s) about ({ff._fmt(cx)}, {ff._fmt(cy)})')

    # 4. the whole route
    wr.INPROC = inproc
    rc, grade = wr.chain(K, os.path.join(work, 'run'), rounds, base=frame, dest=dest,
                         settings={**STEP_SETTINGS, _rules.SETTING: _rules.as_setting(run_rules), **joint})
    seq = _routed(os.path.join(work, 'run'))
    short = lambda n: n.split('/')[-1]
    for n in names:
        if short(n) in {short(f) for f in failed}:
            refused.setdefault(short(n), 'no escape at the source fanout')
        elif short(n) not in ladder:
            refused.setdefault(short(n), 'not in the ladder: no free stub end at the source')
        elif short(n) not in nets:
            refused.setdefault(short(n), f'beyond --k {K}')
    summary = dict(board=board, out=out, src=src, dest=dest, k=K, rules=_rules_dict(run_rules),
                   routed=[], refused=dict(refused), grade=None, secs=None, exit=None)

    def finish(code, why, g=None, routed=()):
        # a bus the step does not hand on clean is not handed on at all: OUT is the board as it came
        if code:
            from copy_board import copy_board
            with contextlib.redirect_stdout(sys.stderr):
                copy_board(board, out)
            for n in nets:
                refused.setdefault(n, why)
        summary.update(routed=sorted(routed), refused=dict(sorted(refused.items())), grade=g,
                       secs=int(time.time() - t0), exit=code, why=why if code else None)
        with open(json_out or os.path.join(work, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=1)
        log(g or f'route_bus: {why}')
        if code:
            log(f'route_bus: {why} -- {os.path.basename(out)} is the board as it came')
        return code, g

    if seq is None:
        return finish(1, f'the whole route laid nothing (exit {rc})', grade)

    # 5. back into the board's own frame; a bus net the run did not route leaves as it came -- none of the step's
    # copper (a source stub beyond --k, a refused escape's), its own copper back if it had any
    routed_pcb = os.path.join(work, 'routed.kicad_pcb')
    bad = ff.turn_file(seq, routed_pcb, -q, cx, cy)
    if bad:
        return finish(2, 'the turn back failed: ' + '; '.join(bad[:6]), grade)
    side = routed_pcb[:-len('.kicad_pcb')] + '.ladder.txt'
    if os.path.exists(side):
        os.remove(side)
    checker = lambda name: os.path.join(HERE, '..', 'py_router', name)
    # ...and so does one the whole route left OPEN (its best round short of every lane, exit 1): the step hands on what
    # it laid connected, never nothing for the sake of a few nets, and the rest of the chain routes those
    if rc != 0:
        wr.run([checker('check_connected.py'), routed_pcb, '--nets'] + [f'*{n}' for n in nets], dict(os.environ),
               log_path=os.path.join(work, 'conn_open.log'), own_process=True)
        opened = sorted(open_nets(os.path.join(work, 'conn_open.log')) & set(nets))
        for n in opened:
            refused.setdefault(n, 'left open by the whole route (its best round)')
        nets = [n for n in nets if n not in opened]
        if not nets:
            return finish(1, f'the whole route connected none of the bus (exit {rc})', grade)
    back = [n for n in names if short(n) not in set(nets)]
    if back:
        put_back(routed_pcb, board, pcb, back)
        log(f'  put back as they came: {", ".join(sorted(short(n) for n in back))}')

    # 6. the grade: every bus net handed on connected, no violation on a bus net, and the whole board no worse than it
    # came (a board the chain hands on carries its earlier steps' violations; each is graded at its own project's floor)
    def drc(path, tag, *more):
        lp = os.path.join(work, f'drc_{tag}.log')
        wr.run([checker('check_drc.py'), path, '--clearance-margin', '0.1', '--max-print', '0', *more],
               dict(os.environ), log_path=lp, own_process=True)
        txt = open(lp, encoding='utf-8', errors='replace').read()
        m = re.search(r'FOUND (\d+) DRC VIOLATIONS', txt)
        return 0 if 'NO DRC VIOLATIONS' in txt else (int(m.group(1)) if m else None)
    d_in = drc(board, 'input')
    # ...and NEVER ALL OR NOTHING: a bus net that fails it on the board -- open there, or named in a violation of the
    # bus's (or of the board's, when the board came out worse) -- goes back as it came and the rest are graded again;
    # the step refuses the whole bus only for a failure no bus net is named in
    for _ in range(3):
        pats = [f'*{n}' for n in nets]
        cc = wr.run([checker('check_connected.py'), routed_pcb, '--nets'] + pats, dict(os.environ),
                    log_path=os.path.join(work, 'conn.log'), own_process=True)[0]
        d_bus, d_out = drc(routed_pcb, 'bus', '--nets', *pats), drc(routed_pcb, 'board')
        # (joint: the step laid the arrays' whole fanout and moved the passives off it each round -- the chain's cap
        # nudge, whole_route -- whose own pad-to-via hits the nudge could not clear are the board's to report; the
        # bus's nets are held clean)
        ok_drc = d_bus == 0 and None not in (d_out, d_in) and (joint_fanout or d_out <= d_in)
        if cc == 0 and ok_drc:
            break
        bad = set(nets) & (open_nets(os.path.join(work, 'conn.log'))
                           | drc_nets(os.path.join(work, 'drc_bus.log'))
                           | (drc_nets(os.path.join(work, 'drc_board.log'))
                              if not joint_fanout and None not in (d_out, d_in) and d_out > d_in else set()))
        if not bad or bad == set(nets):
            break
        for n in sorted(bad):
            refused.setdefault(n, 'failed the grade on the board (open there, or in a violation)')
        nets = [n for n in nets if n not in bad]
        put_back(routed_pcb, board, pcb, [n for n in names if short(n) in bad])
        log(f'  failed the grade, put back as they came: {", ".join(sorted(bad))} -- the rest graded again')
    v, c = wr.grade_counts(routed_pcb, set(nets))
    planes = plane_grade(pcb, board, routed_pcb, log)
    g = (f'BUS {src}->{dest} K={K} {grade.split(" ", 2)[2] if grade else ""} | on the board: vias={v} copper={c}mm '
         f'connected={int(cc == 0)} drc={int(ok_drc)} (bus {d_bus}, board {d_out} vs {d_in} in) '
         + (f'planes={planes["out"]} vs {planes["in"]} in ' if planes and planes.get('out') is not None else '')
         + f'secs={int(time.time() - t0)}')
    summary['drc'] = dict(bus=d_bus, board=d_out, input=d_in)
    if planes:
        summary['planes'] = planes
    if joint_fanout:
        summary['joint_fanout'] = {r: sorted(others[r]) for r in others}
    if cc != 0 or not ok_drc:
        return finish(1, 'the routed bus is not connected and clean on the board', g)

    # 7. handed on: OUT, its routed nets protected in its project (#521) -- route.py's rip candidacy and the plane
    # finalize leave them alone
    from copy_board import copy_board
    from protected_nets import persist_protected_nets, pro_path_for_board, read_protected_nets
    with contextlib.redirect_stdout(sys.stderr):
        copy_board(routed_pcb, out)
    full = {short(n): n for n in names}
    # (the nets the board came with protected stay so -- an earlier bus's among them)
    persist_protected_nets(pro_path_for_board(out), {**read_protected_nets(pro_path_for_board(board)),
                                                     **{full[n]: 'bus' for n in nets}}, verbose=False)
    return finish(0, None, g, routed=nets)


def short_name(n):
    return n.split('/')[-1]


def plane_grade(pcb, board, routed, log=print):
    """(more routing layers than two) the POURS the bus crossed: on each routing layer other than F.Cu and B.Cu that
    carries a pour, its nets' unconnected islands (kicad_exact_fill.exact_unconnected: the board refilled by KiCad, an
    island a link) on the board as it came and as routed -- a lane across a plane can cut it in two, and no other check
    of the step's sees a pour. {layers, nets, in, out, split} (in / out None where KiCad's python is not to be had); None
    when the bus crosses no pour"""
    import route_layers
    inner = set(route_layers.layers()) - {'F.Cu', 'B.Cu'}
    pour = {z.layer for z in (pcb.zones or []) if z.net_id and z.layer in inner}
    if not pour:
        return None
    nets_ = sorted({pcb.nets[z.net_id].name for z in pcb.zones if z.net_id and z.layer in pour and z.net_id in pcb.nets})
    from kicad_exact_fill import exact_unconnected

    def links(path):
        with contextlib.redirect_stdout(io.StringIO()):
            r = exact_unconnected(path, nets_)
        return None if r is None else collections.Counter(l_[0] for l_ in r if l_[0] in set(nets_))
    a, b = links(board), links(routed)
    out = dict(layers=sorted(pour), nets=nets_, **{'in': None if a is None else sum(a.values()),
                                                     'out': None if b is None else sum(b.values())})
    if a is None or b is None:
        log(f'  planes crossed on {", ".join(sorted(pour))} ({", ".join(nets_)}): NOT graded -- no KiCad python for '
            f'the refill')
        return out
    out['split'] = sorted(n for n in nets_ if b[n] > a[n])
    log(f'  planes crossed on {", ".join(sorted(pour))}: {out["out"]} island link(s) routed vs {out["in"]} as it came'
        + (f'; SPLIT by the bus: {", ".join(out["split"])}' if out['split'] else ''))
    return out


def array_others(pcb, ref, names):
    """the nets of `ref`'s pads the joint fanout lays beside the bus: every net with a pad on the array that is not
    one of the bus's (`names`) and owns no zone -- a plane net's balls get the engine's plane drops instead"""
    zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
    bus = {short_name(n) for n in names}
    return sorted({p.net_name for p in pcb.footprints[ref].pads
                   if p.net_id and p.net_name and p.net_id not in zone_ids and short_name(p.net_name) not in bus})


def put_back(path, board, pcb, back):
    """`path` with the copper of the nets `back` (full names) as `board` has it: the step's taken out, the board's own
    put back (each block as the board spells it, its uuid with it)"""
    import braid as te
    ids = {i for i, n in pcb.nets.items() if n.name in set(back)}
    txt = open(path, encoding='utf-8').read()
    for tok in ('segment', 'via'):
        txt = te.strip_net_items(txt, tok, ids, back)
    src = open(board, encoding='utf-8').read()
    own = [b for tok in ('segment', 'via') for b in net_blocks(src, tok, ids, back)]
    if own:
        end = txt.rstrip().rfind(')')
        txt = txt[:end].rstrip() + '\n' + ''.join(f'  {b}\n' for b in own) + txt[end:]
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)


def net_blocks(txt, token, net_ids, net_names=()):
    """every top-level `token` block ('segment', 'via') of the given nets, in either net dialect -- the blocks
    braid.strip_net_items would take out"""
    out = []
    i = 0
    while True:
        j = txt.find('(' + token, i)
        if j < 0:
            return out
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = txt[j:k + 1]
        m = re.search(r'\(net (\d+)\)', block)
        m2 = re.search(r'\(net "([^"]+)"\)', block)
        if (m and int(m.group(1)) in net_ids) or (m2 and m2.group(1) in net_names):
            out.append(block)
        i = k + 1


def _rules_dict(r):
    return {k: getattr(r, k) for k in ('clearance', 'track', 'fan_track', 'via_size', 'via_drill', 'hole_to_hole',
                                       'edge_clearance', 'source')}


# a bus: two parts sharing at least this many nets the whole route admits (make_bench.pair_nets)
MIN_BUS_NETS = 8
# a net on both of a bus's parts and on no more parts than this is named as left to the router (more: a rail)
LEFT_PARTS = 6


def is_array(fp):
    """A ball array: its pads on a grid (bga_fanout's analyze_bga_grid), three rows and three columns at least, and
    balls past the outer two rings on two nets or more -- a QFN's interior is its exposed pad, on one net"""
    from bga_fanout.grid import analyze_bga_grid
    g = analyze_bga_grid(fp)
    if g is None or len(g.rows) < 3 or len(g.cols) < 3:
        return False
    deep = {p.net_id for p in fp.pads if p.pad_type != 'np_thru_hole'
            and min(p.global_x - g.min_x, g.max_x - p.global_x) > 2 * g.pitch_x
            and min(p.global_y - g.min_y, g.max_y - p.global_y) > 2 * g.pitch_y}
    return len(deep) >= 2


def find_buses(pcb, min_nets=MIN_BUS_NETS):
    """Every pair of parts sharing `min_nets` nets or more that the whole route admits (make_bench.pair_nets), the
    largest first: [{src, dest, nets, taken, why, left}]. Taken when both parts are ball arrays (is_array), the
    larger the source; otherwise named with why not (a row part at one end is the router's). `left`: the nets on both
    parts the whole route does not admit, each with why -- its other parts (fly-by, multi-drop, a pull-down), or more
    than one pad on a part -- the router's."""
    with step_policy():                 # (make_bench imports the chain's modules: under its policy, step_policy)
        import make_bench as mb
    from collections import Counter
    many = {r for r, f in pcb.footprints.items() if len(f.pads) > 2}
    count = Counter()
    for net in pcb.nets.values():
        ends = {p.component_ref for p in net.pads if p.component_ref in many}
        if len(ends) == 2:
            count[tuple(sorted(ends))] += 1
    zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
    arrays = {}
    out = []
    for (a, b), n in count.items():
        if n < min_nets:
            continue
        for r in (a, b):
            if r not in arrays:
                arrays[r] = is_array(pcb.footprints[r])
        if not (arrays[a] or arrays[b]):
            continue
        names = mb.pair_nets(pcb, a, b)
        if len(names) < min_nets:
            continue
        balls = lambda r: len(pcb.footprints[r].pads)
        src, dest = sorted((a, b), key=lambda r: (-arrays[r], -balls(r), r))
        why = None if arrays[a] and arrays[b] else \
            f'{dest} is no ball array: the whole route takes an array at both ends, a row part is left to the router'
        left = {}
        for net in pcb.nets.values():
            refs = {p.component_ref for p in net.pads}
            # (a plane's net, or one on many parts or pads -- a rail -- is no signal of the bus's)
            if net.name and {a, b} <= refs and net.name not in names and net.net_id not in zone_ids \
                    and len(refs) <= LEFT_PARTS and len(net.pads) <= 2 * LEFT_PARTS:
                on = Counter(p.component_ref for p in net.pads)
                left[net.name[1:] if net.name.startswith('/') else net.name] = '; '.join(
                    ([f'also on {", ".join(sorted(refs - {a, b}))}'] if refs - {a, b} else [])
                    + [f'{on[r]} pads on {r}' for r in (a, b) if on[r] > 1])
        out.append(dict(src=src, dest=dest, nets=sorted(n_.split('/')[-1] for n_ in names), taken=why is None,
                        why=why, left=dict(sorted(left.items()))))
    out.sort(key=lambda d: (-d['taken'], -len(d['nets']), d['src'], d['dest']))
    return out


def name_clashes(pcb, names):
    """{name: why} for the nets among `names` whose last path part is another net's too (the zynq's ENABLE and
    TEST/ENABLE): the whole route names a net by that part, and the router's net patterns match it (net_queries), so
    the other net would be fanned out, graded and laid for it"""
    by = {}
    for n in pcb.nets.values():
        if n.name:
            by.setdefault(n.name.split('/')[-1], []).append(n.name)
    out = {}
    for nm in names:
        others = sorted(o for o in by.get(nm.split('/')[-1], ()) if o != nm)
        if others:
            out[nm.split('/')[-1]] = f'its name is also {", ".join(others)}\'s last part: the whole route names a net by it'
    return out


def half_pairs(pcb, names):
    """{leg: why} for the pair legs among `names` whose partner is not among them (pairs.pair_names over the board's
    nets): a pair is laid whole or not at all, and a partner on a third part as well is no lane (the zynq's FB_CLK_N,
    pulled to RFGND by RX22)"""
    import pairs as _pairs
    short = {n.split('/')[-1] for n in names}
    out = {}
    by = {n.name.split('/')[-1]: n for n in pcb.nets.values() if n.name}
    for b_, (pn, nn) in _pairs.pair_names(sorted(by), admit_all=True).items():
        if b_ in by:                         # (a base that is a net of its own is no pair: pairs.members)
            continue
        for leg, mate in ((pn, nn), (nn, pn)):
            if leg in short and mate not in short:
                parts = sorted({p.component_ref for p in by[mate].pads})
                out[leg] = f'its pair\'s other leg {mate} is no lane between the arrays (on {", ".join(parts)})'
    return out


def waypoint_pairs(pcb, names, src, dest):
    """{leg: part} for the pairs among `names` that pass through a termination part between the arrays
    (pairs.pair_waypoints: a two-pad part with one pad on each leg, neither array, not under a ball)"""
    import pairs as _pairs
    by = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
    short = [n.split('/')[-1] for n in names]
    out = {}
    for _b, (pn, nn) in _pairs.pair_names(short, admit_all=True).items():
        if pn in by and nn in by:
            for pp, _qn in _pairs.pair_waypoints(pcb, by[pn], by[nn], src, dest):
                out[pn] = out[nn] = pp.component_ref
    return out


def open_nets(conn_log):
    """the nets (short names) check_connected's log finds open: unrouted, or in pieces (whole_route.open_nets)"""
    import whole_route as wr
    return wr.open_nets(conn_log)


def drc_nets(drc_log):
    """the nets (short names) check_drc's log names in its violations: both sides of a line between two items, and
    the net of a line naming one (the warnings after them are not read)"""
    from fanout_from_plan import drc_side_net
    out, found = set(), False
    for ln in open(drc_log, encoding='utf-8', errors='replace') if os.path.isfile(drc_log) else ():
        found = found or ln.startswith('FOUND ')
        if ln.startswith('WARNINGS'):
            break
        if not found or not ln.startswith('  ') or ln.startswith('   '):
            continue
        for side in ln.rstrip('\n').split(' <-> '):
            nm = drc_side_net(re.sub(r':\s*dist=.*$', '', side))
            if nm:
                out.add(nm)
    return out


def _routed(run_dir):
    """the chain's best round's routed board (whole_route: OUTDIR/best.kicad_pcb), else the last round's"""
    if os.path.isfile(os.path.join(run_dir, 'best.kicad_pcb')):
        return os.path.join(run_dir, 'best.kicad_pcb')
    rs = sorted((d for d in os.listdir(run_dir) if d.startswith('r') and d[1:].isdigit()), key=lambda d: int(d[1:]))
    for d in reversed(rs):
        p = os.path.join(run_dir, d, 'seq.kicad_pcb')
        if os.path.isfile(p):
            return p
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board', metavar='BOARD.kicad_pcb')
    ap.add_argument('out', metavar='OUT.kicad_pcb', nargs='?', help='(omit with --find)')
    ap.add_argument('--src', help='the source array (fanned out by the step); with --dest, that one bus -- without '
                                  'both, every bus found (find_buses), the largest first')
    ap.add_argument('--dest', help='the destination array')
    ap.add_argument('--find', action='store_true', help='name the buses found, taken or not and why, and stop')
    ap.add_argument('--k', type=int, help="the ladder's first K nets (default: the whole ladder)")
    ap.add_argument('--rounds', type=int, default=3, help='fanout rounds (default 3)')
    ap.add_argument('--inproc', action='store_true', help='every stage in this process rather than each in its own')
    ap.add_argument('--joint-fanout', action='store_true',
                    help='after each round lays the bus at the two arrays, plan and lay every array\'s other nets '
                         'and plane drops jointly round it, so the chain\'s later fanouts leave the arrays alone')
    ap.add_argument('--json-out', help='the summary: the nets routed and refused (and why), the rules, the grade '
                                     '(default: <OUT>.bus/summary.json)')
    g = ap.add_argument_group('sizes', 'the chain\'s sizes, as the routing CLIs take them; one omitted is resolved as '
                                       'route.py resolves it (the board\'s Default net class, else the fallback)')
    g.add_argument('--clearance', type=float, help='the run\'s clearance (mm)')
    g.add_argument('--clearance-ceiling', type=float, help='cap the clearance at this (mm), as route.py does')
    g.add_argument('--track-width', type=float, help='the lanes\' track (mm)')
    g.add_argument('--fanout-track-width', type=float, help='the escapes\' track (mm; default: --track-width)')
    g.add_argument('--via-size', type=float, help='via diameter (mm)')
    g.add_argument('--via-drill', type=float, help='via drill (mm)')
    g.add_argument('--hole-to-hole-clearance', type=float, help='drill to drill (mm; default: the board\'s own)')
    g.add_argument('--board-edge-clearance', type=float, help='copper to edge (mm; default: the board\'s own)')
    a = ap.parse_args()
    if not os.path.isfile(a.board):
        print(f'route_bus: no board {a.board}', file=sys.stderr)
        sys.exit(2)
    if a.find:
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(sys.stderr):
            pcb = parse_kicad_pcb(a.board)
        for d in find_buses(pcb):
            print(f"{'bus' if d['taken'] else 'not taken'} {d['src']} -> {d['dest']}, {len(d['nets'])} nets"
                  + (f" -- {d['why']}" if d['why'] else ''))
        sys.exit(0)
    if not a.out or (a.src is None) != (a.dest is None):
        ap.error('OUT is needed, and --src and --dest go together')
    sizes = {n: getattr(a, n) for n in SIZE_ARGS if getattr(a, n) is not None}
    if a.src is None:
        sys.exit(route_buses(a.board, a.out, None, a.k, a.rounds, a.inproc, json_out=a.json_out,
                             joint_fanout=a.joint_fanout, **sizes)[0])
    sys.exit(route_bus(a.board, a.out, a.src, a.dest, a.k, a.rounds, a.inproc, json_out=a.json_out,
                       joint_fanout=a.joint_fanout, **sizes)[0])


if __name__ == '__main__':
    main()
