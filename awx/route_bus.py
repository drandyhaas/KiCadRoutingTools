#!/usr/bin/env python3
"""route_bus.py -- the bus step: an array pair's bus routed on the board as it is.

    python3 route_bus.py BOARD.kicad_pcb OUT.kicad_pcb [--src U1 --dest U2] [--k K] [--rounds R] [--inproc]
                         [--clearance C] [--track-width W] [--fanout-track-width W] [--via-size D] [--via-drill D]
                         [--json-out FILE]
    python3 route_bus.py BOARD.kicad_pcb OUT.kicad_pcb --bus SRC:DEST[:MODE[:LAYERS]] [--bus ...] [...]
    python3 route_bus.py BOARD.kicad_pcb --find

Without --src and --dest, every bus on the board (route_buses): find_buses names each pair of parts sharing
MIN_BUS_NETS nets or more the whole route admits, and takes those with a ball array at both ends (is_array), the
larger array the source, the largest bus first; each is routed in turn on the board the one before handed on, and
OUT is the last. A pair with a row part at one end (a QFN) is named and left to the router, and so is each net on
both parts the whole route does not admit, with why. --find names them and stops. --bus names the buses and their
order instead, each with its own fanout (MODE: joint, escape or plain) and routing layers (LAYERS, ROUTE_LAYERS) --
a bus to a row part among them (its pads escape straight out of their rows: escape_moves.row_part); an escape bus on
an array an earlier bus was routed through plans that array whole with it, its ends on the board as it was before
(route_buses: ESCAPE_JOINT, ESCAPE_JOINT_ENDS). A plain bus from such an array keeps the escapes that bus's step
laid it there, a joint one when named (SOURCE_SHARED, keeps_source_escapes). DEST may name several parts,
SRC:U4+U3: ONE SOURCE, SEVERAL ENDS, the buses to them one plain bus planned and laid whole -- the whole route's trunk
runs to the first, and each other's lanes leave it on a branch (whole_frame), crossing the others on the trunk alone.

route_bus() takes a board as the chain hands it on and writes it back with the bus laid, in the board's own frame,
at the chain's sizes (resolve_rules: the routing CLIs' flags, an omitted one resolved as route.py resolves it):

1. The bus: every net between SRC and DEST the ladder admits (make_bench.pair_nets) -- and every one a PULL or a
   TERMINATION alone keeps out (pulled_nets: its other pads on parts to a rail, or across a pair), laid as a lane,
   its stub pads off its net for the step and back on it in OUT, the stub the rest of the chain's to connect.
   Refused: a pair through a termination part between the arrays (the whole route lays no waypoint); a net whose
   name's last part is another net's too (name_clashes: the whole route names a net by it); and a pair leg whose
   partner is refused or is no lane between the arrays (half_pairs: a pair is laid whole or not at all). The bus's
   copper, if any, is stripped: the step lays both ends of every lane.
2. SRC is fanned out for them (make_bench.fanout_source) -- on a plain bus by default (keeps_source_escapes), the
   escapes an earlier step's plan laid them there kept instead (escape_blocks) --, the project floor stamped, and the
   ladder written: the plan's rivers, largest first.
3. The board is turned into the flow frame (flow_frame.py): the quarter turn that points the pair's source-to-
   destination direction along +x, about a point on the 0.1 mm lattice, so the turn is exact both ways.
4. The whole route (whole_route.chain) on the ladder's first K nets, the whole ladder by default.
5. The routed board -- the whole route's best round, which never ends with nothing while it had a plan to lay -- is
   turned back into the board's own frame. A bus net the run did not route, or left open, leaves as it came: none
   of the step's copper, its own put back.
6. It is graded there: the nets handed on connected (check_connected), no violation on a bus net and the whole
   board no worse than it came (check_drc, each board at its own project's floor; the rows of the board's .kicad_dru
   length and skew rules are named, not counted -- the bus is laid untuned, #1363). The arrays' other balls the step
   WALLED IN -- reachable on the board as it came, and from nowhere past the locked lanes at the fab ladder's finest
   rung -- are named (walled_grade: the BUS line's walled=, the summary's 'walled'); each such net ships open.
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
import math
import os
import re
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

# the caches are a harness's (awx-caches-off-by-default): the step runs every stage without them
STEP_SETTINGS = {'TAUT_MEMO': '0', 'PROBE_MEMO': '0', 'STAGE_CACHE': '0'}

# the sizes route_bus takes, as the routing CLIs spell them (route.py's, and the escapes' own track)
SIZE_ARGS = ('clearance', 'clearance_ceiling', 'track_width', 'fanout_track_width', 'via_size', 'via_drill',
             'hole_to_hole_clearance', 'board_edge_clearance')


def step_policy(run_rules=None, settings=None):
    """The settings the step's awx modules are imported and run under: the chain's stage policy (whole_route.stage_env
    -- a pair one lane, the ends' judge), the caches off, and the run's rules when given. A module reads its policy
    when it is FIRST imported in a process (awx_settings), and the chain's in-process stages run on the modules the
    step imported before them: imported under the shell's environment, braid read BRAID_PAIRS off, and every stage
    planned and laid a pair as two single lanes (the zynq's DQS0 and DQS1, the AD9364's fourteen LVDS pairs).
    `settings`: the step's own, over the environment (route_buses: a bus's ROUTE_LAYERS, a shared array's)"""
    import awx_settings
    import whole_route as wr
    env = dict(os.environ)
    env.update(settings or {})
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
              escape=False, settings=None, **sizes):
    """Route the bus between `src` and `dest` on `board`, writing `out`, at the chain's sizes (`sizes`: SIZE_ARGS,
    resolve_rules). Returns (exit code, grade line). The run's rules are installed for the step and supplied to
    every stage (rules.SETTING); the process's own are put back after. `settings` {name: value}: awx settings for
    this step alone, over the environment, in the step and in every stage it runs"""
    import rules as _rules
    bad = sorted(set(sizes) - set(SIZE_ARGS))
    if bad:
        raise TypeError(f'route_bus: no size {bad}')
    run_rules = resolve_rules(board, log=log, **sizes)
    prev = _rules.active()
    _rules.install(run_rules)
    try:
        with step_policy(run_rules, settings):
            return _route(board, out, src, dest, k, rounds, inproc, log, run_rules, json_out, joint_fanout, escape,
                          settings)
    finally:
        _rules.install(prev)


BUS_MODES = ('joint', 'escape', 'plain')


def parse_bus(spec):
    """--bus SRC:DEST[:MODE[:LAYERS]] -> (src, dest, mode or None, layers or None): MODE one of BUS_MODES (joint: the
    joint fanout, escape: the escape fanout, plain: neither), LAYERS the bus's routing layers (ROUTE_LAYERS, comma
    separated). A ValueError names what is wrong"""
    parts = spec.split(':')
    if len(parts) < 2 or len(parts) > 4 or not parts[0] or not parts[1]:
        raise ValueError(f'--bus {spec!r}: SRC:DEST[:MODE[:LAYERS]]')
    mode = parts[2] if len(parts) > 2 and parts[2] else None
    if mode is not None and mode not in BUS_MODES:
        raise ValueError(f'--bus {spec!r}: MODE is one of {", ".join(BUS_MODES)}')
    layers = parts[3] if len(parts) > 3 and parts[3] else None
    return parts[0], parts[1], mode, layers


def route_buses(board, out, buses=None, k=None, rounds=3, inproc=False, log=print, json_out=None,
                joint_fanout=False, escape=False, **sizes):
    """Every bus on `board` routed in turn, each on the board the one before handed on, `out` the last: `buses` as
    [(src, dest)] or [(src, dest, mode, layers)] (parse_bus: MODE joint, escape or plain, None for `joint_fanout` /
    `escape`; LAYERS the bus's ROUTE_LAYERS, None for the environment's), else the ones find_buses takes, the largest
    first. Returns (exit code, grade lines): 0 when every bus was handed on routed, 1 when one or more left as it came
    (or none was found), 2 on a usage error. The summary (--json-out, default <OUT>.bus/summary.json) holds each bus's
    own, and the buses found and not taken.

    A SHARED ARRAY: an escape bus on an array an earlier bus was handed on through has that array planned whole with
    it each round (ESCAPE_JOINT), its ends chosen on the board as it was before the first such earlier bus
    (ESCAPE_JOINT_ENDS) -- the zynq's U1: the DDR3 bus (joint), then the AD9364's LVDS (escape) round it. A plain bus
    from such an array keeps the escapes the earlier step's plan laid its balls there, a joint one when the environment
    names its source (SOURCE_SHARED: keeps_source_escapes) -- the zynq's USB bus from U1 to its QFN"""
    from kicad_parser import parse_kicad_pcb
    from copy_board import copy_board
    from flow_frame import dest_refs as _ff_dest_refs
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
    inputs, through = [], {}        # (each step's input board; {array: the first step handed on through it})
    for bus in buses:
        s, d = bus[0], bus[1]
        mode = (bus[2] if len(bus) > 2 else None) or ('escape' if escape else 'joint' if joint_fanout else 'plain')
        settings = {'ROUTE_LAYERS': bus[3]} if len(bus) > 3 and bus[3] else {}
        parts_ = [s] + _ff_dest_refs(d)                 # (DEST may name several parts: one source, several ends)
        shared = [r_ for r_ in parts_ if r_ in through]
        if mode == 'escape' and shared:
            settings.update(ESCAPE_JOINT=','.join(shared), ESCAPE_JOINT_ENDS=inputs[min(through[r_] for r_ in shared)])
            log(f'route_bus: {s} -> {d}: {", ".join(shared)} routed through by an earlier bus -- planned whole with this '
                f'one, its ends on {os.path.basename(settings["ESCAPE_JOINT_ENDS"])}')
        step_out = os.path.join(work, f'{s}-{d}.kicad_pcb')
        step_json = os.path.join(work, f'{s}-{d}.summary.json')
        inputs.append(cur)
        code, g = route_bus(cur, step_out, s, d, k, rounds, inproc, log, json_out=step_json,
                            joint_fanout=mode == 'joint', escape=mode == 'escape', settings=settings or None, **sizes)
        runs.append(dict(src=s, dest=d, mode=mode, settings=settings, exit=code, grade=g, summary=step_json))
        if code == 0:
            for r_ in parts_:
                through.setdefault(r_, len(inputs) - 1)
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


def _route(board, out, src, dest, k, rounds, inproc, log, run_rules, json_out=None, joint_fanout=False, escape=False,
           settings=None):
    import make_bench as mb
    import flow_frame as ff
    import whole_route as wr
    from coherent_nets import coherent_nets, whole_pairs
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
    # (DEST may name several parts, 'U4+U3': one source, several ends -- the whole route's trunk runs to the first, a
    # branch off it to each other, whole_frame)
    dests = ff.dest_refs(dest)
    for r in [src] + dests:
        if r not in pcb.footprints:
            log(f'route_bus: no footprint {r} on the board')
            return 2, None
    if len(dests) > 1 and (joint_fanout or escape):
        log(f'route_bus: {src} -> {dest}: a bus to several parts is a plain bus (its source fanned out, each '
            f'destination\'s berths laid as planned) -- no joint or escape fanout')
        return 2, None
    log(f'route_bus: {os.path.basename(board)}, copper layers {pcb.board_info.copper_layers}, {src} -> {dest}')

    # 1. the bus, its copper stripped
    names = sorted({n for d in dests for n in mb.pair_nets(pcb, src, d)})
    for d in dests:
        log(f'  {len(mb.pair_nets(pcb, src, d))} nets between {src} and {d}')
    import awx_settings
    # the nets a pull or a termination alone keeps out (pulled_nets) laid as lanes, each a place in the bus's order: a
    # fly-by DDR3 bus's address and command nets, terminated to its Vtt past the memory, are as much the bus's as its
    # data -- left to the router after it, sdi_mipi_conv's lost 12 of 26 round the lanes laid before them
    pulled = {k_: v_ for d in dests for k_, v_ in pulled_nets(pcb, src, d, names).items()}
    if pulled:
        names = sorted(set(names) | set(pulled))
        log(f'  {len(pulled)} more with a PULL or a TERMINATION beside their two ends, laid as lanes, the stub left to the '
            f'rest of the chain: ' + ', '.join(f'{n.split("/")[-1]} ({", ".join(p.component_ref for p in v)})'
                                               for n, v in sorted(pulled.items())))
    wps = {leg: part for d in dests for leg, part in waypoint_pairs(pcb, names, src, d).items()}
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
    txt = open(board, encoding='utf-8').read()
    # the escapes an earlier step laid the bus's balls at its source (keeps_source_escapes): that step planned every
    # ball of the array whole, each escape chosen with every other ball's, and they stay as the bus's teeth -- fanned
    # out again for the bus alone, a full array sends a ball out by any face with room: zynq U1's ETH_RXCTL and
    # ETH_RXD2 out of the south face into the LVDS bus's escapes, its OTG_DATA6, NXT and STP into the LVDS lanes
    kept = []
    if keeps_source_escapes(src, escape, joint_fanout, awx_settings.get('SOURCE_SHARED')):
        kept = escape_blocks(txt, pcb, src, names)
    txt, n_cut = mb.strip_pair_copper(txt, pcb, names)
    if kept:
        end = txt.rstrip().rfind(')')
        txt = txt[:end].rstrip() + '\n' + ''.join(f'  {b}\n' for b in kept) + txt[end:]
        log(f'  {src} shared: the bus\'s escapes there as the plan before laid them, kept ({len(kept)} piece(s))')
    if n_cut:
        log(f'  their copper stripped ({n_cut} chars)')
    # (a pulled net's pull pads off its net for the step -- the net two-ended, as the whole route lays a lane and its
    # grade counts it -- and back on it in OUT)
    pull_pads = [p for n, v in sorted(pulled.items()) if n in set(names) for p in v]
    txt, held = unpull(txt, pull_pads)
    attached = []               # (the escape fanout: the pull pads it joined to their lanes, attach_pulls)
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
    joint_fanout = joint_fanout and not escape          # (the escape fanout lays no joint plan: the field is dog-boned)
    others = {r: array_others(pcb, r, names) for r in (src, dest)} if joint_fanout else {src: [], dest: []}
    joint, spec = {}, None
    escape_env = {}             # (ESCAPE_JOINT: the escape step's joint plan of a shared array -- the --escape block)
    fanned_e = None             # (ESCAPE_JOINT_ENDS: the bus's own field, fanned in the --escape block)
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
    if escape:
        # THE ESCAPE FANOUT (--escape): no source fanout -- every netted ball of both arrays DOG-BONED the human's way
        # (dogbone_arrays: each ball's via on a gap point, the ball lines free below the balls' layer; an outer-ring
        # ball with a clear way out on its own layer left without one, as the human leaves it), the passives
        # beneath the arrays moved off the vias (the chain's cap step, once, here), and each round's fanout plans the
        # bus's ends among the escapes that field has and lays them as they are (escape_plan, FANOUT_ESCAPE)
        import dogbone_arrays as dga
        import awx_settings
        only = None             # (dogbone_arrays: every ball of both arrays)
        # ESCAPE_JOINT=REF[,REF]: a SHARED array -- an earlier step's bus routed through it, other nets and plane balls
        # of its own -- fanned each round in ONE joint plan with the bus (escape_plan, joint_escape.escape_field): its
        # unprotected copper (an earlier step's fanout of its other nets and plane drops) off inside its box, and every
        # unprotected ball dog-boned here as the escape fanout dog-bones an array of its own -- the field the step's
        # ends choose the bus's teeth on, as on a board of the bus alone; the protected (routed) nets' copper as it stands
        joint_refs = [r_ for r_ in (awx_settings.get('ESCAPE_JOINT') or '').split(',') if r_ in (src, dest)]
        if joint_refs:
            from protected_nets import read_protected_nets, pro_path_for_board
            prot_ = {n.split('/')[-1] for n in read_protected_nets(pro_path_for_board(board))}
            with contextlib.redirect_stdout(sys.stderr):
                bj_ = parse_kicad_pcb(base)
            byname_ = {n.name.split('/')[-1]: (i, n) for i, n in bj_.nets.items() if n.name}
            zone_ = {z.net_id for z in (bj_.zones or []) if z.net_id}
            bus_s_ = {n.split('/')[-1] for n in names}
            txt_ = open(base, encoding='utf-8').read()
            only = {}
            arrays_ = []
            for r_ in joint_refs:
                fp_ = bj_.footprints[r_]
                nets_ = {p_.net_id: p_.net_name for p_ in fp_.pads
                         if p_.net_id and p_.net_name and p_.net_name.split('/')[-1] not in prot_}
                xs_, ys_ = [p_.global_x for p_ in fp_.pads], [p_.global_y for p_ in fp_.pads]
                box_ = (min(xs_) - 1.5, min(ys_) - 1.5, max(xs_) + 1.5, max(ys_) + 1.5)
                txt_ = mb.strip_window(txt_, sorted({n.split('/')[-1] for n in nets_.values()} & set(byname_)),
                                       byname_, box_)
                only[r_] = {n.split('/')[-1] for n in nets_.values()}
                arrays_.append({'ref': r_, 'bus': list(names),
                                'others': sorted(n for i, n in nets_.items()
                                                 if i not in zone_ and n.split('/')[-1] not in bus_s_),
                                'drops': sorted(n for i, n in nets_.items() if i in zone_)})
                log(f'  {r_}: joint with the bus each round -- {len(arrays_[-1]["others"])} other net(s) and the plane '
                    f'balls of {len(arrays_[-1]["drops"])} net(s) planned with it, their fanout here taken off; '
                    f'{len(prot_ & {n.split("/")[-1] for n in (p_.net_name or "" for p_ in fp_.pads)})} routed net(s) '
                    f'as they stand')
            with open(base, 'w', encoding='utf-8') as f_:
                f_.write(txt_)
            spec_path = os.path.join(work, 'escape_joint.json')
            with open(spec_path, 'w', encoding='utf-8') as f_:
                json.dump({'arrays': arrays_}, f_, indent=1)
            escape_env = {'ESCAPE_JOINT_SPEC': spec_path}
        with contextlib.redirect_stdout(sys.stderr):
            _ok, dstats = dga.dogbone_file(base, fanned, [src, dest], log=log, only=only)
            fix_project_for_output(fanned, clearance=te.SPEC_CLEARANCE, track_width=sr.FAN_TRACK,
                                   via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL, verbose=False)
        # (only the passives beneath the bus's own two arrays: another array's are another step's -- joint_escape.
        # passives_beyond, locked)
        import joint_escape as je
        lock_ = je.passives_beyond(fanned, [src, dest])
        rcn = wr.run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), fanned, fanned,
                      '--clearance', str(te.SPEC_CLEARANCE), '--beneath-only'] + (['--lock'] + lock_ if lock_ else []),
                     dict(os.environ), log_path=os.path.join(work, 'caps.log'), own_process=True)[0]
        if lock_:
            log(f'  caps: beneath another array, left where they are: {", ".join(lock_)}')
        for ln in wr.grep(os.path.join(work, 'caps.log'), r'^(Moved|Stuck|  Unresolved)'):
            log('  caps: ' + ln.strip()[:200])
        if rcn != 0:
            log(f'  caps: the nudge exited {rcn} -- the passives stay where they were')
        left = {p_ for r_ in (src, dest) for p_ in dstats[r_][2]}
        failed = sorted({short for n in names for short in [n.split('/')[-1]]
                         if any(f'{p.component_ref}.{p.pad_number}' in left for p in
                                pcb.nets[next(i for i, nn in pcb.nets.items() if nn.name == n)].pads)})
        log(f'  {src} and {dest} dog-boned' + (f', no dog-bone for {failed}' if failed else ''))
        if held and awx_settings.get('BUS_ATTACH', '1') != '0':
            held, attached = attach_pulls(fanned, held, [src, dest], log)
        # ESCAPE_JOINT_ENDS=BOARD (with ESCAPE_JOINT): the board as it was before the shared array's earlier step --
        # fanned here exactly as this step fans a board of the bus alone (its copper off, both arrays dog-boned whole,
        # the caps nudged), turned below into the same frame: the field each round's ends are chosen on, so the bus
        # leaves the shared array by the teeth, and reaches the destination by the berths, the bus alone would take
        ends_board = awx_settings.get('ESCAPE_JOINT_ENDS') if joint_refs else None
        fanned_e = None
        if ends_board:
            with contextlib.redirect_stdout(sys.stderr):
                pe_ = parse_kicad_pcb(ends_board)
            txt_e, _n = mb.strip_pair_copper(open(ends_board, encoding='utf-8').read(), pe_, names)
            txt_e = unpull(txt_e, pull_pads)[0]
            base_e = os.path.join(work, 'ends_base.kicad_pcb')
            with open(base_e, 'w', encoding='utf-8') as f_:
                f_.write(txt_e)
            fp.copy_pro(ends_board, base_e)
            fanned_e = os.path.join(work, 'ends_fanned.kicad_pcb')
            elog = []
            with contextlib.redirect_stdout(sys.stderr):
                dga.dogbone_file(base_e, fanned_e, [src, dest], log=elog.append)
                fix_project_for_output(fanned_e, clearance=te.SPEC_CLEARANCE, track_width=sr.FAN_TRACK,
                                       via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL, verbose=False)
            lock_e = je.passives_beyond(fanned_e, [src, dest])
            wr.run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), fanned_e, fanned_e,
                    '--clearance', str(te.SPEC_CLEARANCE), '--beneath-only'] + (['--lock'] + lock_e if lock_e else []),
                   dict(os.environ), log_path=os.path.join(work, 'caps_ends.log'), own_process=True)
            att_e = {}
            if attached:
                # (the pulls the fanout joined to their lanes joined on the bus's own field too, so the ends are chosen
                # on the field they are laid on)
                attach_pulls(fanned_e, attached, [src, dest], log=elog.append)
                att_e = {'BUS_ATTACHED': ','.join(f'{r_}.{n_}' for r_, n_, _t in attached)}
            with contextlib.redirect_stdout(sys.stderr), awx_settings.given({**awx_settings.environ(), **att_e}):
                mb.write_ladder(fanned_e, names, log=lambda *a: None, bare_ok=True)
            log(f'  the bus\'s own field, for the ends: {os.path.basename(ends_board)} fanned as the bus alone -- '
                + '; '.join(x.strip() for x in elog if x.strip().startswith((f'{src}:', f'{dest}:')))[:300])
    else:
        with contextlib.redirect_stdout(sys.stderr):
            n_t, n_v, failed = mb.fanout_source(base, fanned, src, names, reserve=spec)
        with contextlib.redirect_stdout(sys.stderr):
            fix_project_for_output(fanned, clearance=te.SPEC_CLEARANCE, track_width=sr.FAN_TRACK,
                                   via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL, verbose=False)
        log(f'  {src} fanned out: {n_t} tracks, {n_v} vias' + (f', refused {failed}' if failed else ''))
    # (a pull pad the fanout joined to its lane, attach_pulls: on its net, none of the net's two ends --
    # coherent_nets.admissible reads BUS_ATTACHED, here and in the whole route's rounds)
    att_s = ','.join(f'{r_}.{n_}' for r_, n_, _t in attached)
    att_given = (lambda: awx_settings.given({**awx_settings.environ(), 'BUS_ATTACHED': att_s})) if attached else \
        contextlib.nullcontext
    with contextlib.redirect_stdout(sys.stderr), att_given():
        # (a net bare at both arrays -- outer balls left to leave on their own layer -- is a lane the escape fanout lays
        # from its pads: braid.bare_lane)
        mb.write_ladder(fanned, names, log=log, bare_ok=escape)
        if fanned_e:
            # (ESCAPE_JOINT_ENDS: the bus alone's ladder -- its rivers read on its own field, so the run's nets, their
            # order and the K cut are the bus alone's: the ends chosen on that field in that order are its ends)
            import shutil
            shutil.copyfile(os.path.splitext(fanned_e)[0] + '.ladder.txt', os.path.splitext(fanned)[0] + '.ladder.txt')
            log(f'  ladder: the bus alone\'s, from {os.path.basename(fanned_e)}')
        ladder = coherent_nets(10 ** 6, fanned, bare_ok=escape)
    K = min(int(k), len(ladder)) if k else len(ladder)
    with contextlib.redirect_stdout(sys.stderr), att_given():
        nets = whole_pairs(ladder, K, fanned)            # (the chain's own cut: coherent_nets(K) cuts no pair)
    cut = [n for n in ladder[:K] if n not in nets]
    log(f'  the run: K={K} of the ladder\'s {len(ladder)} nets'
        + (f' -- {len(nets)} nets: {", ".join(cut)} left out with their pair\'s other leg' if cut else ''))
    joint_left = set()          # (bus nets the run leaves out, planned as the arrays' others: full names)
    if spec is not None:
        added = others_left_out(spec, pcb, names, nets)
        if added:
            joint_left = {n for _r, n in added}
            with open(os.path.join(work, 'joint.json'), 'w', encoding='utf-8') as f:
                json.dump(spec, f, indent=1)
            log(f'  joint fanout: the bus nets the run leaves out planned as the arrays\' others: '
                + ', '.join(f'{r_}:{short_name(n)}' for r_, n in added))

    # 3. the flow frame
    with contextlib.redirect_stdout(sys.stderr):
        q, cx, cy = ff.quarter_of(parse_kicad_pcb(fanned), dest, set(nets))
    frame = os.path.join(work, 'frame.kicad_pcb')
    bad = ff.turn_file(fanned, frame, q, cx, cy)
    if bad:
        log('route_bus: the turn into the flow frame failed: ' + '; '.join(bad[:6]))
        return 2, None
    log(f'  turned into the flow frame: {q} quarter turn(s) about ({ff._fmt(cx)}, {ff._fmt(cy)})')
    if escape and fanned_e:
        frame_e = os.path.join(work, 'ends_frame.kicad_pcb')
        bad = ff.turn_file(fanned_e, frame_e, q, cx, cy)
        if bad:
            log('  the bus\'s own field: the turn failed -- the ends chosen on the shared field: ' + '; '.join(bad[:3]))
        else:
            escape_env['ESCAPE_ENDS_BASE'] = frame_e

    # 4. the whole route
    wr.INPROC = inproc
    rc, grade = wr.chain(K, os.path.join(work, 'run'), rounds, base=frame, dest=dest,
                         settings={**(settings or {}), **STEP_SETTINGS, _rules.SETTING: _rules.as_setting(run_rules),
                                   **joint,
                                   **({'FANOUT_ESCAPE': '1'} if escape else {}), **escape_env,
                                   **({'BUS_ATTACHED': att_s} if attached else {})})
    seq = _routed(os.path.join(work, 'run'))
    short = lambda n: n.split('/')[-1]
    for n in names:
        if short(n) in {short(f) for f in failed}:
            refused.setdefault(short(n), 'no escape at the source fanout')
        elif short(n) not in ladder:
            refused.setdefault(short(n), 'not in the ladder: no free stub end at the source')
        elif short(n) in cut:
            refused.setdefault(short(n), f'its pair cut by --k {K}: left out with its other leg')
        elif short(n) not in nets:
            refused.setdefault(short(n), f'beyond --k {K}')
    summary = dict(board=board, out=out, src=src, dest=dest, k=K, rules=_rules_dict(run_rules),
                   routed=[], refused=dict(refused), grade=None, secs=None, exit=None,
                   pulled={short(n): sorted(f'{p.component_ref}.{p.pad_number}' for p in v)
                           for n, v in pulled.items() if n in set(names)})

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
    kept_ = escapes_kept(pcb, back, joint_left)
    back = [n for n in back if n not in kept_]
    if kept_:
        log(f'  their escapes kept, the joint plan\'s: {", ".join(short(n) for n in kept_)}')
    if back:
        left_ = put_back(routed_pcb, board, pcb, back)
        log(f'  put back as they came: {", ".join(sorted(short(n) for n in back))}'
            + (f' -- {left_} piece(s) of their earlier copper left out, no longer clear' if left_ else ''))

    # 6. the grade: every bus net handed on connected, no violation on a bus net, and the whole board no worse than it
    # came (a board the chain hands on carries its earlier steps' violations; each is graded at its own project's floor).
    # The rows of the board's length rules (whole_route.LENGTH_ROWS) are named, not counted: the bus is laid untuned
    # (#1363), and an unrouted net is never measured, so the board as it came would always read cleaner than any bus
    length_rows = {}

    def drc(path, tag, *more):
        lp, jp = os.path.join(work, f'drc_{tag}.log'), os.path.join(work, f'drc_{tag}.json')
        if os.path.exists(jp):
            os.remove(jp)
        wr.run([checker('check_drc.py'), path, '--clearance-margin', '0.1', '--max-print', '0', '--json', jp, *more],
               dict(os.environ), log_path=lp, own_process=True)
        n, length_rows[tag] = wr.drc_count(jp)
        if n is not None:
            return n
        txt = open(lp, encoding='utf-8', errors='replace').read()
        m = re.search(r'FOUND (\d+) DRC VIOLATIONS', txt)
        return 0 if 'NO DRC VIOLATIONS' in txt else (int(m.group(1)) if m else None)
    d_in = drc(board, 'input')
    # ...and NEVER ALL OR NOTHING: a violation the step made on another net's copper it laid -- a shared array's other
    # nets, fanned with the bus (ESCAPE_JOINT, the joint fanout), a passive the cap nudge could not move off one of its
    # vias -- puts THAT net back as it came; a bus net that fails it on the board -- open there, or named in a violation
    # of the bus's (or of the board's, when the board came out worse) -- goes back as it came; the rest are graded again.
    # The bus is handed on whenever its own nets come out connected and clean: sdi_mipi_conv's SDI bus, laid 14 nets
    # clean, was refused whole for one GND cap pad on a DDR3 address via the joint field had laid under it
    zone_nets = {short(pcb.nets[z.net_id].name) for z in (pcb.zones or []) if z.net_id in pcb.nets}
    put_others = set()
    for _ in range(6):
        pats = [f'*{n}' for n in nets]
        cc = wr.run([checker('check_connected.py'), routed_pcb, '--nets'] + pats, dict(os.environ),
                    log_path=os.path.join(work, 'conn.log'), own_process=True)[0]
        d_bus, d_out = drc(routed_pcb, 'bus', '--nets', *pats), drc(routed_pcb, 'board')
        # (joint: the step laid the arrays' whole fanout and moved the passives off it each round -- the chain's cap
        # nudge, whole_route -- whose own pad-to-via hits the nudge could not clear are the board's to report; the
        # bus's nets are held clean)
        worse = None in (d_out, d_in) or d_out > d_in
        ok_drc = d_bus == 0 and not worse
        if cc == 0 and ok_drc:
            break
        # (another net's copper the step laid, in a violation of the bus's or one the board did not come with: back
        # first -- the bus is what the step is for. Never a plane's net, whose drops the joint field laid by the dozen)
        named = drc_nets(os.path.join(work, 'drc_bus.log')) if d_bus else set()
        if worse and d_out is not None:
            named |= drc_nets(os.path.join(work, 'drc_board.log'))
        others_ = changed_nets(board, routed_pcb, named - set(nets) - zone_nets - put_others)
        if others_:
            put_others |= others_
            left_ = put_back(routed_pcb, board, pcb, sorted(others_full(pcb, others_)))
            log(f'  another net\'s copper the step laid in a violation, put back as it came: {", ".join(sorted(others_))}'
                + (f' ({left_} piece(s) of its earlier copper left out, no longer clear)' if left_ else '')
                + ' -- the bus graded again')
            continue
        bad = set(nets) & (open_nets(os.path.join(work, 'conn.log'))
                           | drc_nets(os.path.join(work, 'drc_bus.log'))
                           | (drc_nets(os.path.join(work, 'drc_board.log'))
                              if not joint_fanout and None not in (d_out, d_in) and d_out > d_in else set()))
        if not bad or bad == set(nets):
            break
        for n in sorted(bad):
            refused.setdefault(n, 'failed the grade on the board (open there, or in a violation)')
        nets = [n for n in nets if n not in bad]
        left_ = put_back(routed_pcb, board, pcb, [n for n in names if short(n) in bad])
        log(f'  failed the grade, put back as they came: {", ".join(sorted(bad))} -- the rest graded again'
            + (f' ({left_} piece(s) of their earlier copper left out, no longer clear)' if left_ else ''))
    if put_others:
        summary['others_put_back'] = sorted(put_others)
    v, c = wr.grade_counts(routed_pcb, set(nets))
    planes = plane_grade(pcb, board, routed_pcb, log)
    walled = walled_grade(board, routed_pcb, [src] + dest.split('+'), names, log)
    g = (f'BUS {src}->{dest} K={K} {grade.split(" ", 2)[2] if grade else ""} | on the board: vias={v} copper={c}mm '
         f'connected={int(cc == 0)} drc={int(ok_drc)} (bus {d_bus}, board {d_out} vs {d_in} in'
         + (f', {length_rows["bus"]} length rows' if length_rows.get('bus') else '') + ') '
         + (f'planes={planes["out"]} vs {planes["in"]} in ' if planes and planes.get('out') is not None else '')
         + f'walled={len(walled["walled"])} secs={int(time.time() - t0)}')
    summary['drc'] = dict(bus=d_bus, board=d_out, input=d_in)
    if length_rows.get('bus'):
        summary['length_rows'] = dict(bus=length_rows['bus'], board=length_rows.get('board'),
                                      input=length_rows.get('input'))
        log(f'  laid untuned: {length_rows["bus"]} row(s) of the board\'s length rules on the bus\'s nets, named, not '
            f'counted (#1363): {os.path.join(work, "drc_bus.log")}')
    if planes:
        summary['planes'] = planes
    summary['walled'] = walled
    if joint_fanout:
        summary['joint_fanout'] = {r: sorted(others[r]) for r in others}
    if cc != 0 or d_bus != 0 or None in (d_out, d_in):
        return finish(1, 'the routed bus is not connected and clean on the board', g)
    if d_out > d_in:
        # (the bus itself connected and clean: handed on, the board's added violations -- none on a bus net, none on
        # copper of the step's it could put back -- named for the rest of the chain, never the bus thrown away for them)
        summary['board_violations_added'] = d_out - d_in
        log(f'  handed on with {d_out - d_in} violation(s) more on the board than it came with, on no bus net: '
            f'{os.path.join(work, "drc_board.log")}')

    # 7. handed on: OUT, its routed nets protected in its project (#521) -- route.py's rip candidacy and the plane
    # finalize leave them alone
    from copy_board import SIBLING_EXTS, copy_board
    from protected_nets import persist_protected_nets, pro_path_for_board, read_protected_nets
    with contextlib.redirect_stdout(sys.stderr):
        copy_board(routed_pcb, out)
    # ...with the board's own siblings: the step changes none of its rules (.kicad_dru), its brief or its local
    # settings, and a work board on the way lost them -- OUT went on with its .kicad_pro alone, and KiCad graded it
    # without the length windows its rules declare (#1316). The project stays the routed board's: its DRC floor is
    # the one the step routed to
    import shutil
    for ext in SIBLING_EXTS:
        if ext == '.kicad_pro':
            continue
        src_ = board[:-len('.kicad_pcb')] + ext
        if os.path.isfile(src_):
            shutil.copyfile(src_, out[:-len('.kicad_pcb')] + ext)
    if held:
        # (the pull pads back on their nets: the rest of the chain connects each to its lane)
        with open(out, encoding='utf-8') as f:
            txt_o = f.read()
        txt_o, missed = repull(txt_o, held)
        with open(out, 'w', encoding='utf-8') as f:
            f.write(txt_o)
        if missed:
            return finish(2, f'the pull pads {missed} not found again to put back on their nets', g)
    full = {short(n): n for n in names}
    # ...and LOCKED: KiCad's (locked yes) on every track and via of the lanes handed on. The project's protection is
    # lifted by a later step naming the net exactly (#521: a route.py --nets finishing a pull's stub names it), a lock
    # by nothing (#1261): the rest of the chain routes round the lanes and connects to them, never strips them
    # (BUS_LOCK=0: protected only)
    if awx_settings.get('BUS_LOCK', '1') != '0':
        with open(out, encoding='utf-8') as f:
            txt_o = f.read()
        ids_ = {i for i, n_ in pcb.nets.items() if n_.name in {full[n] for n in nets}}
        txt_o, n_locked = lock_lanes(txt_o, ids_, {full[n] for n in nets})
        with open(out, 'w', encoding='utf-8') as f:
            f.write(txt_o)
        log(f'  locked: {n_locked} track(s) and via(s) of the {len(nets)} lane(s) handed on')
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


def walled_grade(board, routed, refs, names, log=print):
    """(#1348) the balls of the step's arrays `refs` on no bus net (`names`) that the step WALLED IN: from the ball, a
    track of the fab ladder's finest rung reaches the rest of its net on the board as it came and not as routed. The
    lanes are handed on locked, so no later step reaches such a ball, at any size, and its net ships open -- and no
    other check of the step's sees it: the bus's grade is its own nets', and the board's DRC finds no violation in a
    ball nothing reaches. Measured: imx6ulz's U2.F1 (VMEM), escaped by a stub to the array's face, a DDR lane on In2.Cu
    under the stub's end and two on F.Cu turning across it. A ball whose net has no other pad has nothing to reach and
    is not asked. {balls, rung, walled: ['REF.PAD'], secs}"""
    from kicad_parser import parse_kicad_pcb
    from list_nets import escalation_rungs
    t0 = time.time()
    bus = {short_name(n) for n in names}
    with contextlib.redirect_stdout(io.StringIO()):
        b_out = parse_kicad_pcb(routed)
    finest = escalation_rungs(len(b_out.board_info.copper_layers or ()) or 4)[-1]
    rung = (finest['track_width'], finest['via_diameter'], finest['clearance'])
    asked = {}
    for r in refs:
        foot = b_out.footprints.get(r)
        if foot is None:
            continue
        asked[r] = [p.pad_number for p in foot.pads
                    if p.net_id and p.net_name and short_name(p.net_name) not in bus and p.pad_type != 'np_thru_hole'
                    and len(b_out.nets[p.net_id].pads) > 1]

    def caged_on(pcb, keys):
        out = []
        for r in keys:
            foot = pcb.footprints.get(r)
            if foot is None:
                continue
            near = _cropped(pcb, foot)
            by_num = {p.pad_number: p for p in foot.pads}
            out += [f'{r}.{n}' for n in keys[r] if n in by_num and _caged(near, by_num[n], *rung)]
        return out
    now = caged_on(b_out, asked)
    walled = []
    if now:
        with contextlib.redirect_stdout(io.StringIO()):
            b_in = parse_kicad_pcb(board)
        was = set(caged_on(b_in, {r: [k.split('.', 1)[1] for k in now if k.split('.', 1)[0] == r] for r in asked}))
        walled = [k for k in now if k not in was]
    out = dict(balls=sum(len(v) for v in asked.values()), rung=list(rung), walled=walled,
               secs=round(time.time() - t0, 1))
    if walled:
        nets_ = {f'{r}.{p.pad_number}': p.net_name for r in asked for p in b_out.footprints[r].pads}
        log(f'  WALLED IN by the step, reached from nowhere at {rung[0]}/{rung[1]}/{rung[2]} (the fab ladder\'s finest): '
            + ', '.join(f'{k} ({nets_.get(k)})' for k in walled) + ' -- each net ships open past the locked lanes')
    return out


CAGE_VIEW, CAGE_STEP = 4.0, 0.025       # mm round the ball (check_reachability's --margin), the raster's cell


def _cropped(pcb, foot):
    """PCB with only the copper within CAGE_VIEW (and a millimetre) of `foot`'s pads -- each ball's raster reads no
    more, and a board's every track read per ball per layer was most of the census's time (sdi_mipi_conv: 22 s)"""
    import copy
    g = CAGE_VIEW + 1.0
    xs, ys = [p.global_x for p in foot.pads], [p.global_y for p in foot.pads]
    x0, y0, x1, y1 = min(xs) - g, min(ys) - g, max(xs) + g, max(ys) + g
    c = copy.copy(pcb)
    c.segments = [s for s in pcb.segments if max(s.start_x, s.end_x) >= x0 and min(s.start_x, s.end_x) <= x1
                  and max(s.start_y, s.end_y) >= y0 and min(s.start_y, s.end_y) <= y1]
    c.vias = [v for v in pcb.vias if x0 <= v.x <= x1 and y0 <= v.y <= y1]
    c.footprints = {k: f for k, f in pcb.footprints.items()
                    if any(x0 - 3 <= p.global_x <= x1 + 3 and y0 - 3 <= p.global_y <= y1 + 3 for p in f.pads)}
    return c


def _caged(pcb, pad, track, via, clearance):
    """no track `track` wide gets from `pad` to the rest of its net within CAGE_VIEW of it -- nor, where the view holds
    none of the rest, to the view's edge -- changing layers where a via `via` wide fits on every layer or at a via of
    the net's own. check_reachability's question (placement.reachability: its slack field, its own-copper islands) at
    one width: each layer's room thresholded and labelled, the layers joined where a via goes, not a widest path. The
    net's own vias join layers here as its copper does -- check_reachability joins them only where a new via of its
    --via fits, and read main's imx6ulz U1.N4, escaped through its own 0.3 mm via, CAGED. Pours are not read: a
    foreign one is filled round what is laid, and a ball's own counts as no copper to reach"""
    import numpy as np
    from scipy import ndimage
    if os.path.join(HERE, '..', 'py_placer') not in sys.path:
        sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
    from placement.reachability import slack_field
    nid, step = pad.net_id, CAGE_STEP
    view = (pad.global_x - CAGE_VIEW, pad.global_y - CAGE_VIEW, pad.global_x + CAGE_VIEW, pad.global_y + CAGE_VIEW)
    layers = list(pcb.board_info.copper_layers)
    home = next((L for L in layers if L in pad.layers), layers[0])
    layers = [home] + [L for L in layers if L != home]
    frees, owns, room = [], [], []
    for L in layers:
        s, o, D = slack_field(pcb, nid, L, view, clearance, None, step)
        frees.append(s >= track)
        owns.append(o)
        room.append(D)
    h, w = frees[0].shape
    via_ok = np.logical_and.reduce([D >= via / 2.0 for D in room])
    own_via = []
    for v in pcb.vias:
        cx, cy = int((v.x - view[0]) / step), int((v.y - view[1]) / step)
        if v.net_id == nid and 0 <= cx < w and 0 <= cy < h:
            own_via.append((cy, cx))

    def joined(masks, extra, vias):
        """labels of `masks` per layer, offset to be unique, and their union-find: joined at the `extra` cells and, with
        `vias`, wherever a via fits (via_ok) and two layers' masks both hold the cell"""
        labs, n = [], 0
        for m in masks:
            lab, k = ndimage.label(m)
            labs.append(np.where(lab > 0, lab + n, 0))
            n += k
        par = np.arange(n + 1)

        def find(a):
            while par[a] != a:
                par[a] = par[par[a]]
                a = par[a]
            return a

        def union(ids):
            ids = [int(i) for i in ids if i]
            for a in ids[1:]:
                ra, rb = find(a), find(ids[0])
                if ra != rb:
                    par[ra] = rb
        if vias:
            for i in range(len(labs)):
                for j in range(i + 1, len(labs)):
                    a, b = labs[i][via_ok], labs[j][via_ok]
                    m = (a > 0) & (b > 0)
                    for x, y in set(zip(a[m].tolist(), b[m].tolist())):
                        union((x, y))
        for cy, cx in extra:
            union([lab[cy, cx] for lab in labs])
        return labs, find, n
    # the ball's own island: its net's copper joined through its own vias; the target, the rest of its net in the view
    olabs, ofind, on = joined(owns, own_via, False)
    sy, sx = int((pad.global_y - view[1]) / step), int((pad.global_x - view[0]) / step)
    if not olabs[0][sy, sx]:
        return False                        # (no copper of its own at its centre: not this check's to say)
    sroot = ofind(int(olabs[0][sy, sx]))
    roots = np.array([ofind(i) for i in range(on + 1)])
    tgt = [o & (roots[ol] != sroot) for o, ol in zip(owns, olabs)]
    if not any(t.any() for t in tgt):
        edge = np.zeros((h, w), dtype=bool)
        edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
        tgt = [edge] * len(layers)
    flabs, ffind, _n = joined(frees, own_via, True)
    if not flabs[0][sy, sx]:
        return True
    root = ffind(int(flabs[0][sy, sx]))
    return not any(ffind(int(i)) == root for lab, t in zip(flabs, tgt) for i in np.unique(lab[t & (lab > 0)]))


def array_others(pcb, ref, names):
    """the nets of `ref`'s pads the joint fanout lays beside the bus: every net with a pad on the array that is not
    one of the bus's (`names`) and owns no zone -- a plane net's balls get the engine's plane drops instead"""
    zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
    bus = {short_name(n) for n in names}
    return sorted({p.net_name for p in pcb.footprints[ref].pads
                   if p.net_id and p.net_name and p.net_id not in zone_ids and short_name(p.net_name) not in bus})


def others_left_out(spec, pcb, names, run):
    """the joint fanout's `spec` ({'arrays': [{'ref', 'others', 'drops'}]}) with each bus net of `names` (full) the run
    `run` (short names) leaves out among the others of every array it has a ball of on `pcb` -- a zone net's balls
    are the plane drops'; [(ref, net)] added. A net the run leaves out -- its ball dropped by the source fanout, beyond
    --k, its pair cut -- is no lane, but its balls are still the arrays': each round's joint plan escapes them with the
    other nets', for the rest of the chain (sdi_mipi_conv's DDR3_A2, U11.L11, neither the bus's nor another net's, was
    planned by no round, and route.py found no way out of the field the bus left it)"""
    run_s = {short_name(n) for n in run}
    zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
    left = sorted({n for n in names if short_name(n) not in run_s})
    added = []
    for ar in spec['arrays']:
        have = {p.net_name for p in pcb.footprints[ar['ref']].pads if p.net_id and p.net_id not in zone_ids}
        new = [n for n in left if n in have and n not in ar['others']]
        ar['others'] = sorted(set(ar['others']) | set(new))
        added += [(ar['ref'], n) for n in new]
    return added


def escapes_kept(pcb, back, left):
    """the nets of `back` (full names: the step's copper to take off) in `left` (others_left_out's) with no copper of
    their own on `pcb`, the board as it came -- the joint plan escaped their balls as every other net's, and those
    escapes stay with the board; a net with copper of its own is put back as it came"""
    own = {s.net_id for s in pcb.segments} | {v.net_id for v in pcb.vias}
    ids = {n.name: i for i, n in pcb.nets.items()}
    return sorted(n for n in back if n in left and ids.get(n) not in own)


def changed_nets(board, routed, names):
    """the nets of `names` (short) whose tracks or vias on `routed` are not the ones `board` has: copper the step laid"""
    if not names:
        return set()
    src, now = open(board, encoding='utf-8').read(), open(routed, encoding='utf-8').read()

    def items(txt):
        out = collections.defaultdict(set)
        pcb_ = parse_kicad_pcb_quiet_text(txt)
        nm = {i: short_name(n.name) for i, n in pcb_.nets.items()}
        for s_ in pcb_.segments:
            if not getattr(s_, 'graphic', False) and nm.get(s_.net_id) in names:
                a_, b_ = sorted([(round(s_.start_x, 3), round(s_.start_y, 3)), (round(s_.end_x, 3), round(s_.end_y, 3))])
                out[nm[s_.net_id]].add(('s', a_, b_, s_.layer, round(s_.width, 3)))
        for v_ in pcb_.vias:
            if nm.get(v_.net_id) in names:
                out[nm[v_.net_id]].add(('v', round(v_.x, 3), round(v_.y, 3), round(v_.size, 3)))
        return out
    a, b = items(src), items(now)
    return {n for n in names if a.get(n, set()) != b.get(n, set())}


def others_full(pcb, shorts):
    """the full names of the nets `shorts` (short names)"""
    return {n.name for n in pcb.nets.values() if n.name and short_name(n.name) in set(shorts)}


def parse_kicad_pcb_quiet_text(txt):
    """a board's text parsed (written to a temporary file, read, removed)"""
    fd, tmp = tempfile.mkstemp(suffix='.kicad_pcb')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(txt)
        with contextlib.redirect_stdout(io.StringIO()):
            return parse_kicad_pcb_quiet(tmp)
    finally:
        os.remove(tmp)


def put_back(path, board, pcb, back):
    """`path` with the copper of the nets `back` (full names) as `board` has it: the step's taken out, the board's own
    put back (each block as the board spells it, its uuid with it) -- each only where it stands CLEAR of the copper
    `path` holds now: the step may have laid the board round it anew (a shared array fanned in one joint plan,
    ESCAPE_JOINT: an open bus net's earlier escape put back over the joint field's vias was ten violations on zynq U1, and
    every net the grade then put back met another). A block that would not stand is left out -- the net to the rest of
    the chain. Returns the blocks left out"""
    import braid as te
    ids = {i for i, n in pcb.nets.items() if n.name in set(back)}
    txt = open(path, encoding='utf-8').read()
    for tok in ('segment', 'via'):
        txt = te.strip_net_items(txt, tok, ids, back)
    src = open(board, encoding='utf-8').read()
    own = [(tok, b) for tok in ('segment', 'via') for b in net_blocks(src, tok, ids, back)]
    left = 0
    if own:
        keep = _clear_blocks(path, txt, own)
        left = len(own) - len(keep)
        if keep:
            end = txt.rstrip().rfind(')')
            txt = txt[:end].rstrip() + '\n' + ''.join(f'  {b}\n' for b in keep) + txt[end:]
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)
    return left


def _clear_blocks(path, txt, blocks):
    """the `blocks` [(token, text)] -- a net's segments and vias -- that stand clear of every other net's copper on the
    board `txt` (written beside `path` to be read): a segment on its layer, a via on every copper layer, at the run's
    clearance (escape_plan.Copper: pads, vias, segments)"""
    import escape_plan as ep
    import rules as _rules
    tmp = path[:-len('.kicad_pcb')] + '.putback.kicad_pcb'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(txt)
    with contextlib.redirect_stdout(sys.stderr):
        now = parse_kicad_pcb_quiet(tmp)
    os.remove(tmp)
    clear = _rules.active().clearance
    by_name = {n.name: i for i, n in now.nets.items() if n.name}
    num = lambda s, k: [float(x) for x in re.search(r'\(' + k + r' ([-\d.e]+) ([-\d.e]+)', s).groups()]   # noqa: E731
    cu, keep = {}, []
    for tok, b in blocks:
        m_id = re.search(r'\(net (\d+)\)', b)
        m_nm = re.search(r'\(net "([^"]+)"\)', b)
        nid = int(m_id.group(1)) if m_id else by_name.get(m_nm.group(1) if m_nm else '', 0)
        if tok == 'segment':
            w = float(re.search(r'\(width ([\d.e-]+)\)', b).group(1))
            L = re.search(r'\(layer "?([^")\s]+)"?\)', b).group(1)
            a, z = num(b, 'start'), num(b, 'end')
            checks = [(w, tuple(a), tuple(z), L)]
        else:
            s = float(re.search(r'\(size ([\d.e-]+)\)', b).group(1))
            at = tuple(num(b, 'at'))
            checks = [(s, at, at, L) for L in now.board_info.copper_layers]
        ok = True
        for w, a, z, L in checks:
            if w not in cu:
                cu[w] = ep.Copper(now, w, clear)
            if not cu[w].seg_clear(a, z, L, nid)[0]:
                ok = False
                break
        if ok:
            keep.append(b)
    return keep


def parse_kicad_pcb_quiet(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


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
# a pull's or a termination's part has no more pads than this (pulled_nets: an eight-element resistor array)
STUB_PADS = 16


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
    larger the source; otherwise named with why not (a row part at one end is the router's, unless --bus names the
    bus: escape_moves.row_part). `left`: the nets on both
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
            f'{dest} is no ball array: left to the router -- the whole route takes it where --bus names it ' \
            f'({src}:{dest}:plain)'
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


def keeps_source_escapes(src, escape, joint_fanout, setting):
    """Whether a bus keeps the escapes an earlier step laid its nets at its source `src` (escape_blocks) as its teeth,
    the balls without one fanned out: a PLAIN bus by default (`setting`, SOURCE_SHARED, unset) -- where no earlier step
    laid any, nothing is kept and the source is fanned out as ever; SOURCE_SHARED=0 none; SOURCE_SHARED=REF,.. a plain or
    a joint bus from those sources; an ESCAPE bus never (its field is planned whole each round). On the zynq's six-rail
    board, after its DDR3 and LVDS buses: the Ethernet bus 14 of 14 lanes kept against 12 fanned out again, the two-end
    USB and Ethernet bus 24 of 26 against 22, the USB bus 10 of 12 either way"""
    if escape:
        return False
    if setting is None or not str(setting).strip():
        return not joint_fanout
    if str(setting).strip() == '0':
        return False
    return src in str(setting).split(',')


def escape_blocks(txt, pcb, ref, names, grow=1.5):
    """The segment and via blocks of the nets `names` (full names) inside `ref`'s box of pads grown `grow` mm, in
    either net dialect: the escapes an earlier fanout laid them at that array (SOURCE_SHARED)"""
    fp_ = pcb.footprints[ref]
    xs, ys = [p.global_x for p in fp_.pads], [p.global_y for p in fp_.pads]
    box = (min(xs) - grow, min(ys) - grow, max(xs) + grow, max(ys) + grow)
    inside = lambda x, y: box[0] <= x <= box[2] and box[1] <= y <= box[3]   # noqa: E731
    ids = {i for i, n in pcb.nets.items() if n.name in set(names)}
    num = lambda s, k: [float(v) for v in re.search(r'\(' + k + r' ([-\d.e]+) ([-\d.e]+)', s).groups()]  # noqa: E731
    out = []
    for b in net_blocks(txt, 'segment', ids, names):
        if inside(*num(b, 'start')) and inside(*num(b, 'end')):
            out.append(b)
    for b in net_blocks(txt, 'via', ids, names):
        if inside(*num(b, 'at')):
            out.append(b)
    return out


def pulled_nets(pcb, src, dest, names):
    """{net name: [its stub pads]} for the nets between `src` and `dest` that a PULL or a TERMINATION alone keeps out of
    `names` (make_bench.pair_nets): one pad on each of the two parts, and every other pad on a part (STUB_PADS pads at
    most, neither end) whose every pad that carries a net is a RAIL's or such a net's -- with a rail among them, or two
    such nets (a termination across a pair). A rail: a plane's net (one that owns a zone), or a net on more parts than
    LEFT_PARTS (a termination voltage). So a pull-up or -down to a rail -- the zynq's Ethernet PHY's strap pins ETH_RXD0-3,
    ETH_RXCTL and ETH_MDIO (R34-R45), the USB PHY's OTG_RESETN (R33, C121) -- a resistor array terminating a fly-by bus
    to its Vtt (sdi_mipi_conv's DDR3 address and command nets, R44-R50 under U12) and the termination across a clock
    pair (its CK, R43). A lane between the two parts is the net's run; its pull or termination a stub off it, the
    router's"""
    zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
    rails = zone_ids | {i for i, n in pcb.nets.items() if len({p.component_ref for p in n.pads}) > LEFT_PARTS}
    have = set(names)
    cand = {}
    for net in pcb.nets.values():
        if not net.name or net.name in have or net.net_id in rails:
            continue
        on = collections.Counter(p.component_ref for p in net.pads)
        rest = [p for p in net.pads if p.component_ref not in (src, dest)]
        if on.get(src) == 1 and on.get(dest) == 1 and rest:
            cand[net.net_id] = rest

    import pairs as _pairs
    short = lambda i: pcb.nets[i].name.split('/')[-1]

    def stub_part(ref):
        fp_ = pcb.footprints.get(ref)
        if fp_ is None or ref in (src, dest) or len(fp_.pads) > STUB_PADS:
            return False
        ns = {p.net_id for p in fp_.pads if p.net_id}
        if not ns or not ns <= rails | set(cand):
            return False
        legs = ns & set(cand)
        return bool(ns & rails) or (len(legs) == 2 and bool(_pairs.pair_names(sorted(short(i) for i in legs),
                                                                              admit_all=True)))
    return {pcb.nets[i].name: rest for i, rest in cand.items() if all(stub_part(p.component_ref) for p in rest)}


def _pad_block(fp_text, number, with_net):
    """(start, end) in `fp_text` of the pad block numbered `number`, the first with a (net ...) token if `with_net`, else
    the first without one; None when there is none"""
    from kicad_parser import find_matching_paren
    # a number is quoted, or bare in the older spelling KiCad still reads (#1290)
    pad_head = re.compile(r'\(pad\s+(?:"%s"|%s(?=[\s()]))' % ((re.escape(number),) * 2))
    i = 0
    while True:
        m = pad_head.search(fp_text, i)
        if m is None:
            return None
        j = m.start()
        k = find_matching_paren(fp_text, j)
        if (re.search(r'\(net\s', fp_text[j:k]) is not None) == with_net:
            return j, k
        i = k

ATTACH_PITCHES = 1.5        # (attach_pulls) a pull pad is joined to its lane by a stub this many of its array's ball
#                             pitches long at most: the tie beside the ball's via -- a longer one runs across the ball
#                             lines the bus's lanes need into the array


def attach_pulls(path, held, refs, log=print):
    """THE PULLS IN THE FANOUT (the escape fanout; BUS_ATTACH=0 none): each pull or termination pad `held` took off
    its net (unpull) that a stub of ATTACH_PITCHES of an array of `refs` reaches from its net's dog-bone via there
    (escape_plan.own_via; the ball itself where it has none and the pad is on its layer) put back on its net in the
    board at `path` and joined to it by that stub, on the pad's layer -- as the human ties zynq TX_FRAME_N's dog-bone
    into its pull RX10 under U1, 1.0 mm: straight, else the shortest way along the array's ball and gap lines, clear
    of every other net's copper by the fanout's rule. The pad is then its lane's own copper and every other net's
    obstacle, and the pull is connected when the lane is. (held, the pads not joined -- left off their nets, the stub
    the rest of the chain's; attached, those joined)"""
    import escape_plan as ep
    import escape_moves as em
    import rules as _rules
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    txt = open(path, encoding='utf-8').read()
    with contextlib.redirect_stdout(sys.stderr):
        pcb0 = parse_kicad_pcb(path)
    grids = {r_: em.grid_of(pcb0.footprints[r_]) for r_ in refs if r_ in pcb0.footprints}
    lims = {r_: ATTACH_PITCHES * max(g_.pitch_x, g_.pitch_y) for r_, g_ in grids.items()}
    near = []
    for h in held:
        fp_ = pcb0.footprints.get(h[0])
        p_ = next((q for q in (fp_.pads if fp_ else ()) if q.pad_number == h[1]), None)
        if p_ is None:
            continue
        r_ = next((r_ for r_, g_ in grids.items()
                   if g_.bbox[0] - lims[r_] <= p_.global_x <= g_.bbox[2] + lims[r_]
                   and g_.bbox[1] - lims[r_] <= p_.global_y <= g_.bbox[3] + lims[r_]), None)
        if r_ is not None:
            near.append((h, r_))
    if not near:
        return held, []
    txt2, _miss = repull(txt, [h for h, _r in near])
    tmp = path[:-len('.kicad_pcb')] + '.attach.kicad_pcb'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(txt2)
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(tmp)
    os.remove(tmp)
    r = _rules.active()
    cop = ep.Copper(pcb, r.fan_track, r.fan_clear)

    def stub(s, t_, g_, L, nid, lim):
        # the shortest clear path from s to the pad's centre t_ on the array's half-pitch lattice (its ball lines and
        # gap lines, eight ways), its last step straight onto the pad -- every step clear of other nets' copper
        import heapq
        if cop.path_clear([s, t_], L, nid):
            return [s, t_] if math.hypot(t_[0] - s[0], t_[1] - s[1]) <= lim else None
        hx, hy = g_.pitch_x / 2, g_.pitch_y / 2
        ox, oy = g_.bbox[0], g_.bbox[1]
        node = lambda q: (round((q[0] - ox) / hx), round((q[1] - oy) / hy))
        pt = lambda n: (ox + n[0] * hx, oy + n[1] * hy)
        lo = (min(s[0], t_[0]) - 1.0, min(s[1], t_[1]) - 1.0)
        hi = (max(s[0], t_[0]) + 1.0, max(s[1], t_[1]) + 1.0)
        n0 = node(s)
        if math.hypot(pt(n0)[0] - s[0], pt(n0)[1] - s[1]) > 1e-3:
            return None                     # (a start off the lattice: no field to walk)
        dist, prev, todo = {n0: 0.0}, {}, [(0.0, n0)]
        best = None
        while todo:
            d0, n = heapq.heappop(todo)
            if d0 > dist.get(n, 1e18) + 1e-9 or d0 > lim:
                continue
            q = pt(n)
            if best is not None and d0 >= best[0]:
                break
            if math.hypot(q[0] - t_[0], q[1] - t_[1]) <= 2 * max(hx, hy) and cop.path_clear([q, t_], L, nid):
                tot = d0 + math.hypot(q[0] - t_[0], q[1] - t_[1])
                if best is None or tot < best[0]:
                    best = (tot, n)
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    m = (n[0] + i, n[1] + j)
                    qm = pt(m)
                    if (i, j) == (0, 0) or not (lo[0] <= qm[0] <= hi[0] and lo[1] <= qm[1] <= hi[1]):
                        continue
                    dm = d0 + math.hypot(qm[0] - q[0], qm[1] - q[1])
                    if dm < dist.get(m, 1e18) - 1e-9 and cop.path_clear([q, qm], L, nid):
                        dist[m], prev[m] = dm, n
                        heapq.heappush(todo, (dm, m))
        if best is None or best[0] > lim:
            return None
        chain_ = [best[1]]
        while chain_[-1] in prev:
            chain_.append(prev[chain_[-1]])
        pts = [s] + [pt(n) for n in reversed(chain_[:-1])] + [t_]
        # (collinear points merged)
        out = [pts[0]]
        for k_ in range(1, len(pts) - 1):
            a_, b_, c_ = out[-1], pts[k_], pts[k_ + 1]
            if abs((b_[0] - a_[0]) * (c_[1] - a_[1]) - (b_[1] - a_[1]) * (c_[0] - a_[0])) > 1e-9:
                out.append(b_)
        out.append(pts[-1])
        return ep.dedupe(out)
    tracks, attached, why, lens = [], [], [], []
    for h, r_ in near:
        fp_ = pcb.footprints[h[0]]
        p_ = next(q for q in fp_.pads if q.pad_number == h[1])
        nid = p_.net_id
        lays = [L for L in p_.layers if L.endswith('.Cu') and not L.startswith('*')]
        ball = next((q for q in pcb.footprints[r_].pads if q.net_id == nid), None)
        if not nid or ball is None or len(lays) != 1:
            why.append(f'{h[0]}.{h[1]} (no ball of its net at {r_}, or not on one layer)')
            continue
        L = lays[0]
        site = ep.own_via(pcb, ball, nid, grids[r_])
        starts = [site] if site is not None else ([(ball.global_x, ball.global_y)] if L in ball.layers else [])
        got = next((q for s in starts
                    for q in [stub(s, (p_.global_x, p_.global_y), grids[r_], L, nid, lims[r_])] if q), None)
        if got is None:
            why.append(f'{h[0]}.{h[1]} (no clear stub on {L} within {lims[r_]:.2f} mm of its ball at {r_})')
            continue
        tracks += [{'start': a, 'end': b, 'width': r.fan_track, 'layer': L, 'net_id': nid}
                   for a, b in zip(got, got[1:])]
        attached.append(h)
        lens.append(f'{h[0]}.{h[1]} {sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(got, got[1:])):.2f} mm')
    if attached:
        txt3, _miss = repull(txt, attached)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(txt3)
        with contextlib.redirect_stdout(sys.stderr):
            add_tracks_and_vias_to_pcb(path, path, tracks, [])
    log(f'  pulls in the fanout: {len(attached)} pull pad(s) joined to their lanes'
        + (f' ({", ".join(lens)})' if attached else '')
        + (f'; left to the rest of the chain: {"; ".join(why)}' if why else ''))
    return [h for h in held if h not in attached], attached


def unpull(txt, pads):
    """`txt` (a board's text) with each pad of `pads` taken off its net -- its (net ...) token removed -- and the tokens
    taken: (txt, held), held [(ref, pad number, the token with the white space before it)] for repull"""
    from kicad_parser import iter_footprint_blocks, find_matching_paren
    held = []
    for p in pads:
        blk = next(((a, b, t) for a, b, t, _r, key in iter_footprint_blocks(txt) if key == p.component_ref), None)
        span = _pad_block(blk[2], p.pad_number, True) if blk else None
        if span is None:
            continue
        pad = blk[2][span[0]:span[1]]
        m = re.search(r'\s*\(net\s', pad)
        e = find_matching_paren(pad, m.end() - len('(net') - 1)
        tok = pad[m.start():e]
        at = blk[0] + span[0]
        txt = txt[:at + m.start()] + txt[at + e:]
        held.append((p.component_ref, p.pad_number, tok))
    return txt, held


def repull(txt, held):
    """`txt` with each pad unpull took off its net put back on it: the token after the pad's (layers ...), where KiCad
    writes it. (txt, the pads not found: [REF.PAD])"""
    from kicad_parser import iter_footprint_blocks, find_matching_paren
    missed = []
    for ref, num, tok in held:
        blk = next(((a, b, t) for a, b, t, _r, key in iter_footprint_blocks(txt) if key == ref), None)
        span = _pad_block(blk[2], num, False) if blk else None
        if span is None:
            missed.append(f'{ref}.{num}')
            continue
        pad = blk[2][span[0]:span[1]]
        lay = pad.find('(layers')
        at = blk[0] + span[0] + (find_matching_paren(pad, lay) if lay >= 0 else len(pad) - 1)
        txt = txt[:at] + tok + txt[at:]
    return txt, missed


def lock_lanes(txt, net_ids, net_names=()):
    """`txt` with KiCad's (locked yes) on every track and via of the nets (their ids, or their full names in the
    string dialect), where KiCad writes it: before the (layer ...) of a track, the (layers ...) of a via. (txt, the
    items locked)"""
    from kicad_parser import find_matching_paren
    n = 0
    for tok, before in (('segment', '(layer '), ('arc', '(layer '), ('via', '(layers ')):
        out, i = [], 0
        while True:
            j = txt.find('(' + tok, i)
            while j >= 0 and txt[j + 1 + len(tok)] not in ' \n\t':
                j = txt.find('(' + tok, j + 1)
            if j < 0:
                out.append(txt[i:])
                break
            k = find_matching_paren(txt, j)
            block = txt[j:k]
            m = re.search(r'\(net (\d+)\)', block)
            m2 = re.search(r'\(net "([^"]+)"\)', block)
            at = block.find(before)
            if ((m and int(m.group(1)) in net_ids) or (m2 and m2.group(1) in net_names)) and at > 0 \
                    and not re.search(r'\(locked[\s)]|^\(\w+\s+locked\b', block):
                block = block[:at] + '(locked yes) ' + block[at:]
                n += 1
            out.append(txt[i:j] + block)
            i = k
        txt = ''.join(out)
    return txt, n


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
    the net of a line naming one (the warnings after them are not read, nor the rows of the board's length rules,
    whole_route.LENGTH_ROWS, which the grade does not count)"""
    import whole_route as wr
    from fanout_from_plan import drc_side_net
    out, found, kind = set(), False, None
    for ln in open(drc_log, encoding='utf-8', errors='replace') if os.path.isfile(drc_log) else ():
        found = found or ln.startswith('FOUND ')
        if ln.startswith('WARNINGS'):
            break
        m = re.match(r'([A-Z][A-Z-]*) violations \(', ln)
        if m:
            kind = m.group(1).lower()
        if not found or not ln.startswith('  ') or ln.startswith('   ') or kind in wr.LENGTH_ROWS:
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
    ap.add_argument('--bus', action='append', metavar='SRC:DEST[:MODE[:LAYERS]]',
                    help='a bus to route, in the order given (repeat for each; instead of --src/--dest): MODE joint, '
                         'escape or plain (default: --joint-fanout / --escape), LAYERS its routing layers '
                         '(ROUTE_LAYERS, comma separated; default the environment\'s); DEST may be a row part (a QFN), '
                         'and may name several parts, U4+U3 (one source, several ends: a plain bus, its trunk to the '
                         'first, a branch off it to each other). An escape bus on an array an earlier bus was routed '
                         'through plans that array whole with it, its ends on the board as it was before that bus; a '
                         'plain one keeps the escapes that bus\'s step laid it there (SOURCE_SHARED=0: none; '
                         '=REF,..: a plain or joint bus from those sources)')
    ap.add_argument('--k', type=int, help="the ladder's first K nets (default: the whole ladder)")
    ap.add_argument('--rounds', type=int, default=3, help='fanout rounds (default 3)')
    ap.add_argument('--inproc', action='store_true', help='every stage in this process rather than each in its own')
    ap.add_argument('--joint-fanout', action='store_true',
                    help='after each round lays the bus at the two arrays, plan and lay every array\'s other nets '
                         'and plane drops jointly round it, so the chain\'s later fanouts leave the arrays alone')
    ap.add_argument('--escape', action='store_true',
                    help='the ESCAPE fanout: no source fanout -- every ball of both arrays dog-boned the human\'s way '
                         '(dogbone_arrays), the passives beneath moved off the vias, and each round plans the bus\'s '
                         'ends among the escapes that field has and lays them as they are (escape_plan); '
                         'overrides --joint-fanout')
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
    if a.bus and a.src is not None:
        ap.error('--bus names the buses: no --src/--dest with it')
    # (before the run is spent: without ortools every stage that plans fails in a log of its own, #1355)
    from startup_checks import StartupCheckError, check_bus_dependencies
    try:
        check_bus_dependencies()
    except StartupCheckError as e:
        print(f'route_bus: {e}', file=sys.stderr)
        sys.exit(2)
    try:
        buses = [parse_bus(b) for b in a.bus] if a.bus else None
    except ValueError as e:
        ap.error(str(e))
    sizes = {n: getattr(a, n) for n in SIZE_ARGS if getattr(a, n) is not None}
    if a.src is None:
        sys.exit(route_buses(a.board, a.out, buses, a.k, a.rounds, a.inproc, json_out=a.json_out,
                             joint_fanout=a.joint_fanout, escape=a.escape, **sizes)[0])
    sys.exit(route_bus(a.board, a.out, a.src, a.dest, a.k, a.rounds, a.inproc, json_out=a.json_out,
                       joint_fanout=a.joint_fanout, escape=a.escape, **sizes)[0])


if __name__ == '__main__':
    main()
