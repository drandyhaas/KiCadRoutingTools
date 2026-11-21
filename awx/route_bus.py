#!/usr/bin/env python3
"""route_bus.py -- the bus step: an array pair's bus routed on the board as it is.

    python3 route_bus.py BOARD.kicad_pcb OUT.kicad_pcb [--bus SRC:DEST ...] [--k K] [--rounds R] [--inproc]
                         [--feedback-rounds F] [--round-edits EDITS.json] [--pack]
                         [--clearance C] [--clearance-ceiling C] [--track-width W] [--fanout-track-width W]
                         [--via-size D] [--via-drill D] [--hole-to-hole-clearance H] [--json-out FILE]
                         [--escalation fab|board|off]
    python3 route_bus.py BOARD.kicad_pcb --find [--json-out FOUND.json] [sizes]
    python3 route_bus.py BOARD.kicad_pcb OUT.kicad_pcb --src SRC --dest DEST [...]     (one bus by itself)

THE BUSES (route_buses): those bus_plan finds at the chain's sizes, or those --bus names. A bus is the nets between a
ball array (is_array) and another part that the whole route admits -- another ball array sharing MIN_BUS_NETS nets or
more, or a ROW PART no track passes between the pads of (row_gap), the row parts off one face of the source one GROUP
BUS (SRC:U4+U3: one source and several ends, planned and laid whole in one frame -- the whole route's trunk runs to the
first, and each other's lanes leave it on a branch, whole_frame). --find prints the buses, in order, and the parts left
to the router, each with why, then the --bus line that names them, and stops. EACH SOURCE'S BUSES ARE ONE GROUP
(negotiated_buses.route_grouped): each bus's ends model captured by itself, every bus's ends chosen in one model with
the arrays' fanout (group_ends.py), and every bus routed on that field at the same time, their solves coupled; the
groups in turn, each on the board the one before handed on. The lanes run on every copper layer (all_layers). A group
that leaves the board short of clean is routed again a rung down the fab ladder (--escalation, route.py's flag and
default: _escalated, escalation_ladder) -- the buses that left nets open alone, where the others are clean, their
copper kept -- and OUT's project floor is lowered to the copper it ships (shipped_rules).

route_bus() takes a board as the chain hands it on and writes it back with the bus laid, in the board's own frame,
at the chain's sizes (resolve_rules: the routing CLIs' flags, an omitted one resolved as route.py resolves it):

1. The bus: every net between SRC and DEST the ladder admits (make_bench.pair_nets) -- and every one OTHER PADS
   alone keep out (pulled_nets: a pull, a termination, a test point, another part's pin; no rail), laid as a lane,
   its stub pads off its net for the step and back on it in OUT, the stub the rest of the chain's to connect.
   Refused: a pair through a termination part between the arrays (the whole route lays no waypoint); and a pair leg
   whose partner is refused or is no lane between the arrays (half_pairs: a pair is laid whole or not at all). The
   bus's copper, if any, is stripped: the step lays both ends of every lane. The whole route names a net by its name's
   last path part, or in full where another net's name ends in that part too (coherent_nets.net_keys).
2. SRC is fanned out for them (make_bench.fanout_source), the escapes an earlier step laid them there kept
   (escape_blocks), the project floor stamped, and the ladder written: the plan's rivers, largest first -- or, on a
   group's field (GROUP_ENDS), the bus's copper there is its ends and the ladder the model's order.
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
ROUTE_LAYERS, every copper layer of the board unless the environment names them), and every via is a through via. A routing layer
that carries a POUR (a plane: zynq's In1 GND and In2 VCC_1V8) is crossed by the lanes: the pour's nets are graded on
the board as it came and as routed, each refilled (kicad_exact_fill), and a split the bus made is named. The caches are off. Everything the step writes goes in OUT's work directory,
<OUT stem>.bus/, beside OUT.

PROGRESS (bus_events.py): the step appends one JSON event a line to <OUT stem>.bus/events.jsonl (or to the file the
BUS_EVENTS setting names) as it goes -- the buses found, each capture, each solve of the group's model and its checks,
each bus's fanout, solve, coupled solve, loop and lay, each bus's grade, the step's end -- written by every process of
the step, so a reader (the GUI's Bus tab) follows it while it runs. --find --json-out writes the buses found as JSON.

Exits 0 whenever it writes OUT (#1738) -- the bus handed on routed, or OUT the board as it came (a bus of them left
as it came, or none found): either way the chain carries on from OUT, and the summary and the console name the nets
routed and refused, and why. Exits 2 on a usage error (a part the board does not have, a bus spec the step cannot
run), and non-zero whenever it wrote no OUT. The summary's `exit` keeps the step's own status: 0 routed, 1 handed
on as it came, 2 refused.
"""

KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door
import argparse
import collections
import contextlib
import copy
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
if __name__ == '__main__':
    # (the step's processes never load KiCad's pcbnew, which OR-Tools cannot share a process with:
    # swig_compat.keep_pcbnew_out, before anything here can import it)
    import swig_compat  # noqa: E402
    swig_compat.keep_pcbnew_out()
from coherent_nets import block_net_name  # noqa: E402
import bus_events  # noqa: E402

# the caches are a harness's (awx-caches-off-by-default): the step runs every stage without them
STEP_SETTINGS = {'TAUT_MEMO': '0', 'STAGE_CACHE': '0'}

# the sizes route_bus takes, as the routing CLIs spell them (route.py's, and the escapes' own track)
SIZE_ARGS = ('clearance', 'clearance_ceiling', 'track_width', 'fanout_track_width', 'via_size', 'via_drill',
             'hole_to_hole_clearance', 'board_edge_clearance')

# the step's --escalation, a setting every bus's process reads (route_buses runs a group's buses each in its own)
ESCALATION_SETTING = 'BUS_ESCALATION'


def install_escalation(board, run_rules):
    """THE STEP'S ESCALATION POLICY -- --escalation (ESCALATION_SETTING), else routing_defaults.ESCALATION -- installed
    in this process as route.py installs its own (fab_tiers.set_policy_from_args): the board's own declared floors
    read off its project, capped at the run's sizes as the request. Returns the policy's name"""
    import types
    import awx_settings
    import fab_tiers as ft
    policy = awx_settings.get(ESCALATION_SETTING) or ft.DEFAULT_ESCALATION
    with contextlib.redirect_stdout(io.StringIO()):
        ft.set_policy_from_args(types.SimpleNamespace(
            escalation=policy, clearance=run_rules.clearance, track_width=run_rules.track,
            via_size=run_rules.via_size, via_drill=run_rules.via_drill, hole_to_hole_clearance=run_rules.hole_to_hole,
            board_edge_clearance=run_rules.edge_clearance), board)
    return policy


def escalation_ladder(run_rules, board):
    """THE BUS'S RUNGS BELOW ITS SIZES: the fab escalation route.py walks for a net that would ship open
    (net_rescue._size_escalation_ladder), for a bus's open lanes or the whole bus. Two or three rungs on which the
    lanes' track, the escapes' track and the via step down TOGETHER from the run's sizes to the installed policy's
    floor -- the fab ladder's last rung under --escalation fab, the board's own declared floors under board, no rung
    under off (fab_tiers.escalation_rungs). The clearance is never reduced: it is the DRC floor's. [Rules], the
    narrowest last"""
    import dataclasses
    import fab_tiers as ft
    import rules as _rules
    ladder = ft.escalation_rungs(ft.count_copper_layers_in_file(board))
    if not ladder:
        return []
    w_floor = min(f['track_width'] for f in ladder)
    w0, f0 = run_rules.track, run_rules.fan_track
    vias = [(f['via_diameter'], min(f['via_drill'], run_rules.via_drill)) for f in ladder
            if f['via_diameter'] < run_rules.via_size - 1e-9]
    travel = max(w0, f0) - w_floor > 1e-9
    if not travel and not vias:
        return []
    m = len(vias)
    n = 3 if (travel and m) else (2 if travel else m)
    out, seen = [], set()
    for k in range(1, n + 1):
        w = round(w0 + (w_floor - w0) * k / n, 4) if w0 > w_floor else w0
        fw = round(f0 + (w_floor - f0) * k / n, 4) if f0 > w_floor else f0
        v, dr = vias[min(m - 1, (k * m - 1) // n)] if m else (run_rules.via_size, run_rules.via_drill)
        if (w, fw, v, dr) in seen:
            continue
        seen.add((w, fw, v, dr))
        # (a via past the standard tier's floor -- the advanced tier's, more costly to fab, or under board the board's
        # own declared minimum -- said when the rung is tried, rung_line)
        past = ('past the standard fab floor: ' + ('the board\'s own declared minimums'
                                                   if ft.escalation_target() == 'board_floors'
                                                   else 'the advanced tier\'s, more costly to fab'),) \
            if m and ft.ladder_escalates(ladder, v, dr) else ()
        # (the rung's pitches its own sizes', as the run's are its own: rules.pitches)
        lp, ep_ = _rules.pitches(w, run_rules.clearance)
        out.append(dataclasses.replace(run_rules, track=w, fan_track=fw, via_size=v, via_drill=dr, lane_pitch=lp,
                                       exit_pitch=ep_, source=f'the fab escalation, rung {len(out) + 1}', notes=past))
    return out


def rung_line(r):
    """a rung's sizes, as the log says them"""
    return (f'track {r.track:g}, escapes {r.fan_track:g}, via {r.via_size:g}/{r.via_drill:g}, the lanes\' pitch '
            f'{r.lane_pitch:g} (clearance {r.clearance:g})' + ''.join(f' -- {n}' for n in r.notes))


def shipped_rules(out, board, names, run_rules, log=print):
    """THE FLOOR THE BUS SHIPS, as route.py ships its own: OUT's project stamped at the run's sizes and lowered to the
    copper OUT carries (fix_project_for_output, which reads the board's own minima) -- so KiCad grades an escalated
    lane at the width it was laid -- and the design rules' ledger (fab_tiers.note_narrowing) of what SHIPS, each of
    the bus's nets (`names`, full) whose smallest track or via stands below what the run asked: a track below the
    escapes' width, or one between it and the lanes'; a via, or its drill, below the run's. The ledger's summary,
    its one line said (fab_tiers.escalation_report_line)"""
    import fab_tiers as ft
    from fix_kicad_drc_settings import fix_project_for_output
    with contextlib.redirect_stdout(io.StringIO()):
        fix_project_for_output(out, board, clearance=run_rules.clearance, track_width=run_rules.fan_track,
                               via_diameter=run_rules.via_size, via_drill=run_rules.via_drill, verbose=False)
    from coherent_nets import namer
    pcb = parse_kicad_pcb_quiet(out)
    sn = namer(pcb)
    want = {n.name for n in pcb.nets.values() if n.name and (n.name in set(names) or sn(n.name) in set(names))}
    eps = 1e-6
    ft.reset_ledger()
    for nid, n in sorted(pcb.nets.items()):
        if n.name not in want:
            continue
        ws = [s.width for s in pcb.segments if s.net_id == nid]
        under = [w for w in ws if w < run_rules.fan_track - eps]
        mid = [w for w in ws if run_rules.fan_track + eps < w < run_rules.track - eps]
        if under:
            ft.note_narrowing(nid, 'track_width', run_rules.fan_track, min(under), 'bus step', count=len(under),
                              net_name=n.name)
        if mid:
            ft.note_narrowing(nid, 'track_width', run_rules.track, min(mid), 'bus step', count=len(mid),
                              net_name=n.name)
        vs = [v for v in pcb.vias if v.net_id == nid]
        small = [v for v in vs if v.size < run_rules.via_size - eps]
        if small:
            ft.note_narrowing(nid, 'via_diameter', run_rules.via_size, min(v.size for v in small), 'bus step',
                              count=len(small), net_name=n.name)
        holes = [v for v in vs if v.drill < run_rules.via_drill - eps]
        if holes:
            ft.note_narrowing(nid, 'hole_size', run_rules.via_drill, min(v.drill for v in holes), 'bus step',
                              count=len(holes), net_name=n.name)
    line = ft.escalation_report_line()
    if line:
        log(f'  {line}')
    return ft.escalation_summary()


def unrecorded():
    """The block's stage processes record nothing in the board's redo manifest (#132): a cap pass, a fanout, a bus's
    own process are the step's insides -- a replay runs them by running the step, which records itself (main) -- and a
    self-recording tool among them added its own line beside the step's. Every setting but REDO_MANIFEST given"""
    import awx_settings
    return awx_settings.given({k: v for k, v in awx_settings.environ().items() if k != 'REDO_MANIFEST'})


def step_policy(run_rules=None, settings=None):
    """The settings the step's awx modules are imported and run under: the chain's stage policy (whole_route.stage_env
    -- the braid's paging), the caches off, and the run's rules when given. A module reads its policy when it is FIRST
    imported in a process (awx_settings), and the chain's in-process stages run on the modules the step imported
    before them -- so the step imports them under this policy, never under the shell's environment.
    `settings`: the step's own, over the environment (route_buses: a bus's ROUTE_LAYERS, a shared array's)"""
    import awx_settings
    import whole_route as wr
    env = awx_settings.environ()
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


def route_bus(board, out, src, dest, k=None, rounds=4, inproc=False, log=print, json_out=None, settings=None,
              feedback_rounds=0, pack=False, **sizes):
    """Route the bus between `src` and `dest` on `board`, writing `out`, at the chain's sizes (`sizes`: SIZE_ARGS,
    resolve_rules). Returns (exit code, grade line). The run's rules are installed for the step and supplied to
    every stage (rules.SETTING); the process's own are put back after. `settings` {name: value}: awx settings for
    this step alone, over the environment, in the step and in every stage it runs. `pack` (--pack): the setting
    BUS_PACK among them -- the whole route packs each round's laid lanes (whole_route.chain's pack_lay)"""
    import rules as _rules
    import awx_settings
    bad = sorted(set(sizes) - set(SIZE_ARGS))
    if bad:
        raise TypeError(f'route_bus: no size {bad}')
    # (the lanes on every copper layer, unless the caller or the environment names them: all_layers)
    if not (settings or {}).get('ROUTE_LAYERS') and not awx_settings.get('ROUTE_LAYERS'):
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(sys.stderr):
            settings = dict(settings or {}, ROUTE_LAYERS=all_layers(parse_kicad_pcb(board)))
    run_rules = resolve_rules(board, log=log, **sizes)
    install_escalation(board, run_rules)
    prev = _rules.active()
    _rules.install(run_rules)
    # (this process routes SRC:DEST: its progress events, and its stages', say so -- bus_events)
    settings = dict(settings or {}, **bus_events.for_bus(src, dest), **({'BUS_PACK': '1'} if pack else {}))
    try:
        with unrecorded(), bus_events.run(bus_events.work_dir(out)), step_policy(run_rules, settings):
            got = _route(board, out, src, dest, k, rounds, inproc, log, run_rules, json_out, settings,
                         feedback_rounds)
    finally:
        _rules.install(prev)
    # (OUT as it ships: its project's floor lowered to its copper, and the design rules' ledger of what it carries, as
    # route.py's main writes its own -- shipped_rules -- in its summary)
    sj_ = json_out or os.path.join(bus_events.work_dir(out), 'summary.json')
    try:
        s_ = json.load(open(sj_, encoding='utf-8'))
    except (OSError, ValueError):
        return got
    if not s_.get('ends_only') and os.path.isfile(out) and s_.get('routed'):
        # (...where a net was routed: a bus handed on as it came leaves its board's project as it came)
        try:
            s_['design_rules'] = shipped_rules(out, board, list(s_.get('routed') or [])
                                               + list(s_.get('refused') or {}), run_rules, log)
        except Exception as e:      # noqa: BLE001
            log(f'route_bus: OUT\'s project floor NOT written ({type(e).__name__}: {e})')
            return got
        with open(sj_, 'w', encoding='utf-8') as f:
            json.dump(s_, f, indent=1)
    return got


#: the OUT boards this process wrote (absolute paths): a step that hands on -- routed, or the board as it came --
#: adds its OUT here, and the CLI exits 0 for it (cli_exit, #1738)
HANDED_ON = set()


def cli_exit(status, out):
    """The process exit for a run whose step returned `status` (route_bus / route_buses) with `out` its OUT: 0
    whenever the run WROTE OUT -- the bus routed or the board as it came, the chain carrying on from it either way and
    the summary naming what was refused -- else the step's status (2 a usage error), and 1 for a run that wrote
    nothing usable with no status of its own (#1738). A plan file runs under `set -e` (route_plan_check R01), and
    the old exit 1 for a board handed on as it came stopped it at the bus step"""
    out = os.path.abspath(out)
    if not out.endswith('.kicad_pcb'):
        out += '.kicad_pcb'
    if out in HANDED_ON:
        return 0
    return status or 1


def bus_usage_errors(board, buses):
    """What the CLI refuses before a run (#1738), `buses` as [(src, dest)] (parse_bus): a part the board does not
    have, and a bus that shares an array with another source's bus -- each source's buses are one group, planned
    whole on its arrays (route_buses), so no array may be two groups' -- made for every named bus up front, so a bus
    named wrong is a usage error (exit 2) rather than one skipped while the run goes on to write OUT"""
    from kicad_parser import parse_kicad_pcb
    import flow_frame as ff
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    bad = []
    for s, d in buses:
        bad += [f'{s} -> {d}: no footprint {r} on the board' for r in [s] + ff.dest_refs(d) if r not in pcb.footprints]
    owner = {}
    for s, d in buses:
        for r in [s] + [r_ for r_ in ff.dest_refs(d) if r_ in pcb.footprints and is_array(pcb.footprints[r_])]:
            if owner.setdefault(r, s) != s:
                bad.append(f'{s} -> {d}: the array {r} is in the buses off {owner[r]} too -- one group of buses an '
                           f'array')
    return bad


def parse_bus(spec):
    """--bus SRC:DEST -> (src, dest): DEST a part, or several (U4+U3: a group bus, one source and several ends). A
    ValueError names what is wrong"""
    parts = spec.split(':')
    if len(parts) != 2 or not parts[0] or not parts[1]:
        raise ValueError(f'--bus {spec!r}: SRC:DEST (no fanout mode and no layers: every bus\'s ends come from its '
                         f'group\'s model, on every copper layer)')
    return parts[0], parts[1]


def all_layers(pcb):
    """ROUTE_LAYERS for a bus on `pcb`: every copper layer, F.Cu and B.Cu first (route_layers)"""
    cu = list(pcb.board_info.copper_layers)
    return ','.join(['F.Cu', 'B.Cu'] + [L for L in cu if L not in ('F.Cu', 'B.Cu')])


def row_gap(fp):
    """(pitch, gap) of a ROW PART (escape_moves.row_part): the pitch along its rows, and the copper-free width between
    two neighbouring pads there (the pitch less the median pad's width across the row); None for no row part"""
    import escape_moves as em
    rp = em._row_pads(fp)
    if rp is None:
        return None
    rows, pitch = rp
    w = sorted(min(p.size_x, p.size_y) for p in rows)[len(rows) // 2]
    return pitch, pitch - w


def two_sided(fp):
    """a ROW PART whose row pads stand on both outer faces (an edge connector's fingers, F.Cu and B.Cu): its escapes
    leave each face's row the same way, which the row model (escape_moves.grid_of: one face a row) does not take"""
    import escape_moves as em
    rp = em._row_pads(fp)
    if rp is None:
        return False
    faces = {L for p in rp[0] for L in (getattr(p, 'layers', None) or []) if L in ('F.Cu', 'B.Cu')}
    return len(faces) > 1 and all((p.drill or 0) == 0 for p in rp[0])


def side_of(src_fp, fp):
    """the face of `src_fp`'s pad box that `fp` stands off, 'N' 'E' 'S' or 'W': the side its pads' box centre's
    direction crosses, each axis scaled by the box's half size"""
    def box(f):
        xs = [p.global_x for p in f.pads]
        ys = [p.global_y for p in f.pads]
        return min(xs), min(ys), max(xs), max(ys)
    x0, y0, x1, y1 = box(src_fp)
    a, b, c, e = box(fp)
    dx, dy = (a + c - x0 - x1) / 2, (b + e - y0 - y1) / 2
    hw, hh = max((x1 - x0) / 2, 1e-3), max((y1 - y0) / 2, 1e-3)
    if abs(dx) / hw >= abs(dy) / hh:
        return 'E' if dx > 0 else 'W'
    return 'S' if dy > 0 else 'N'


FACE_NAMES = {'N': 'north', 'E': 'east', 'S': 'south', 'W': 'west'}


def memory_bus(pcb, names):
    """`names` (full net names) read as a DRAM interface: half of them or more in a byte lane or the command/address
    group of length_matching.classify_ddr_nets, the board's whole net list the context that proves a prefix DRAM"""
    from length_matching import classify_ddr_nets
    got = classify_ddr_nets(sorted(names), [n.name for n in pcb.nets.values() if n.name])['groups']
    return 2 * sum(len(v) for v in got.values()) >= len(names)


def bus_plan(pcb, rules):
    """THE BUSES THE STEP ROUTES on `pcb` at the chain's sizes `rules` (resolve_rules), and in what order:
    {'buses': [{src, dest, nets, why}], 'left': [{src, dest, nets, why}]}.

    A bus is the nets between a ball array (is_array) and another part that the whole route admits
    (make_bench.pair_nets, find_buses):
    - to another BALL ARRAY, MIN_BUS_NETS nets or more -- the larger array the source;
    - to a ROW PART no track passes between the pads of at the chain's sizes (row_gap: the gap between two
      neighbouring pads less than a track and a clearance either side), so its lanes must arrive in its pins' order
      -- the row parts off one FACE of the source (side_of) one GROUP bus (U1:U4+U3), the largest first, when one of
      them has MIN_BUS_NETS nets or more, each other MIN_GROUP_NETS or more.
    Each source's buses are ONE GROUP, its ends chosen in one model (route_buses): its array buses first, a memory
    bus (memory_bus) first of those, then the larger; then its row buses, the larger first. The groups go largest bus
    first, and no array is two groups': a bus that would share an array with an earlier group is left, named."""
    import collections as _c
    need = rules.track + 2 * rules.clearance
    found = find_buses(pcb, min_nets=MIN_GROUP_NETS)
    with step_policy():
        import make_bench as mb
    arr = {}

    def isa(r):
        if r not in arr:
            arr[r] = is_array(pcb.footprints[r])
        return arr[r]
    left, arrays, faces = [], [], _c.defaultdict(list)
    for d in found:
        s, t, n = d['src'], d['dest'], len(d['nets'])
        if isa(s) and isa(t):
            if n < MIN_BUS_NETS:
                left.append(dict(src=s, dest=t, nets=n, why=f'fewer than {MIN_BUS_NETS} nets'))
                continue
            names = mb.pair_nets(pcb, s, t)
            arrays.append(dict(src=s, dest=t, nets=n, mem=memory_bus(pcb, names)))
            continue
        rg = row_gap(pcb.footprints[t])
        if rg is None:
            left.append(dict(src=s, dest=t, nets=n, why=f'{t} is neither a ball array nor a row part'))
            continue
        if two_sided(pcb.footprints[t]):
            left.append(dict(src=s, dest=t, nets=n, why=f'{t}\'s rows stand on both faces (an edge connector\'s '
                                                         f'fingers), which the row model does not take: the router\'s'))
            continue
        if rg[1] >= need - 1e-9:
            left.append(dict(src=s, dest=t, nets=n, why=f'a track passes between {t}\'s pads ({rg[1]:.3f} mm, a '
                                                         f'track and its clearances {need:.3f}): the router\'s'))
            continue
        faces[(s, side_of(pcb.footprints[s], pcb.footprints[t]))].append(dict(dest=t, nets=n, gap=rg[1]))
    # (each source's group: its array buses, memory first, then by size; no array two groups')
    groups, owner = _c.OrderedDict(), {}
    for b in sorted(arrays, key=lambda b: (not b['mem'], -b['nets'], b['src'], b['dest'])):
        s, t = b['src'], b['dest']
        if owner.get(s, s) != s or owner.get(t, s) != s:
            other = owner.get(s) if owner.get(s, s) != s else owner[t]
            left.append(dict(src=s, dest=t, nets=b['nets'], why=f'an array of it is in the buses off {other}: one '
                                                                f'group of buses an array'))
            continue
        owner[s] = owner[t] = s
        groups.setdefault(s, []).append(dict(src=s, dest=t, nets=b['nets'], why='between two ball arrays'
                                             + ('; a memory bus (its nets read as DRAM): first' if b['mem'] else '')))
    rows = []
    for (s, f), ms in faces.items():
        ms.sort(key=lambda m: (-m['nets'], m['dest']))
        head = ms[0]['nets'] >= MIN_BUS_NETS
        take = [m for m in ms if head and m['nets'] >= MIN_GROUP_NETS]
        for m in ms:
            if m not in take:
                left.append(dict(src=s, dest=m['dest'], nets=m['nets'],
                                 why=(f'off {s}\'s {FACE_NAMES[f]} face with fewer than {MIN_GROUP_NETS} nets'
                                      if head else f'off {s}\'s {FACE_NAMES[f]} face, where no row part has '
                                                   f'{MIN_BUS_NETS} nets or more')))
        if not take:
            continue
        if owner.get(s, s) != s:
            for m in take:
                left.append(dict(src=s, dest=m['dest'], nets=m['nets'],
                                 why=f'{s} is in the buses off {owner[s]}: one group of buses an array'))
            continue
        gaps = ', '.join(f"{m['dest']} {m['gap']:.3f} mm" for m in take)
        why = (f'the row part{"s" if len(take) > 1 else ""} off {s}\'s {FACE_NAMES[f]} face no track passes between '
               f'the pads of ({gaps} between them, a track and '
               f'its clearances {need:.3f})' + (': one group bus' if len(take) > 1 else ''))
        rows.append(dict(src=s, dest='+'.join(m['dest'] for m in take), nets=sum(m['nets'] for m in take), why=why))
    for b in sorted(rows, key=lambda b: (-b['nets'], b['src'], b['dest'])):
        owner.setdefault(b['src'], b['src'])
        groups.setdefault(b['src'], []).append(b)
    order = sorted(groups, key=lambda s: (-max(b['nets'] for b in groups[s]), s))
    return dict(buses=[b for s in order for b in groups[s]],
                left=sorted(left, key=lambda b: (-b['nets'], b['src'], b['dest'])))


def plan_lines(plan):
    """bus_plan's plan as the lines --find prints and route_buses logs"""
    return ([f"bus {b['src']} -> {b['dest']}, {b['nets']} nets -- {b['why']}" for b in plan['buses']]
            + [f"not taken {b['src']} -> {b['dest']}, {b['nets']} nets -- {b['why']}" for b in plan['left']])


def route_buses(board, out, buses=None, k=None, rounds=4, inproc=False, log=print, json_out=None, feedback_rounds=0,
                pack=False, **sizes):
    """Every bus on `board` routed, `out` the board handed on: `buses` [(src, dest)] (parse_bus), else bus_plan's.
    Returns (status, grade lines): 0 when every bus was handed on routed, 1 when one or more left as it came (or none
    was found), 2 on a usage error -- the step's status, which the summary keeps; the process exits 0 whenever OUT is
    written (cli_exit, #1738). The summary (--json-out, default <OUT>.bus/summary.json) holds each bus's own, and the
    plan: the buses taken and not, and why.

    EACH SOURCE'S BUSES ARE ONE GROUP (negotiated_buses.route_grouped): every bus's ends model captured by itself,
    every bus's ends chosen in ONE model with the arrays' fanout (group_ends.py), every bus routed on that field at the
    same time -- a bus alone on its array a group of one. The groups in turn, each on the board the one before handed
    on; the lanes on every copper layer (all_layers). The step's progress goes to its events file (bus_events:
    BUS_EVENTS, else <OUT stem>.bus/events.jsonl), which every stage's process writes to as well. `pack` (--pack):
    every bus's whole route packs each round's laid lanes -- the setting BUS_PACK, given to every bus's process."""
    board, out = os.path.abspath(board), os.path.abspath(out)
    if not out.endswith('.kicad_pcb'):
        out += '.kicad_pcb'
    work = out[:-len('.kicad_pcb')] + '.bus'
    os.makedirs(work, exist_ok=True)
    with unrecorded(), bus_events.run(work):
        return _route_buses(board, out, work, buses, k, rounds, inproc, log, json_out, feedback_rounds, pack, **sizes)


def _route_buses(board, out, work, buses, k, rounds, inproc, log, json_out, feedback_rounds, pack=False, **sizes):
    from kicad_parser import parse_kicad_pcb
    from copy_board import copy_board
    import awx_settings
    import negotiated_buses
    bus_events.emit('step', 'start', board=board, out=out, rounds=rounds, feedback_rounds=feedback_rounds, k=k,
                    buses=None if buses is None else [f'{s}:{d}' for s, d in buses], sizes=sizes)
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    plan = None
    if buses is None:
        plan = bus_plan(pcb, resolve_rules(board, log=lambda *a_, **k_: None, **sizes))
        for ln in plan_lines(plan):
            log('route_bus: ' + ln)
        bus_events.emit('step', 'plan', buses=plan['buses'], left=plan['left'])
        buses = [(b['src'], b['dest']) for b in plan['buses']]
    groups = collections.OrderedDict()
    for s, d in buses:
        groups.setdefault(s, []).append((s, d))
    cur, runs, codes = board, [], []

    # (every bus's lanes on every copper layer: the setting each bus's process takes, unless the caller named them)
    env_ = awx_settings.environ()
    env_.setdefault('ROUTE_LAYERS', all_layers(pcb))
    if pack:
        # (--pack: BUS_PACK, the setting every bus's whole route reads after its round's lay -- a bus's process takes
        # the run's settings as its environment, negotiated_buses._env0; a harness sets it there with no flag)
        env_['BUS_PACK'] = '1'
    with awx_settings.given(env_):
        for s, grp in groups.items():
            name = f'{s}-{"_".join(g_[1] for g_ in grp)}'
            step_out = os.path.join(work, f'{name}.kicad_pcb')
            step_json = os.path.join(work, f'{name}.summary.json')
            _fresh(step_out, step_json)
            bus_events.emit('group', 'start', group=s, buses=[f'{s}:{d_}' for _s, d_ in grp],
                            work=bus_events.work_dir(step_out))
            rules0_, _pol0, rungs0_ = _ladder(cur, sizes)
            with _installed(rules0_):
                code, _g = negotiated_buses.route_grouped(cur, step_out, grp, k, rounds, inproc, log, step_json,
                                                          feedback_rounds, rungs_below=_below(rungs0_), **sizes)
            try:
                sj_ = json.load(open(step_json))
            except (OSError, ValueError):
                sj_ = {}
            # THE GROUP ESCALATED (--escalation, route.py's fab escalation for a whole group of buses): a group whose
            # buses left nets open (or broke the board's rules, or walled a ball in) routed again WHOLE -- its ends
            # model, the arrays' fanout and every bus's lanes -- a rung down the fab ladder, the lanes' track, the
            # escapes' and the vias together (escalation_ladder), rung after rung to the policy's floor until one
            # leaves the board clean. The fewest open, then violations, then walled balls stands; of two alike the
            # one with fewer buses left to route again, then the earlier. The clearance is never reduced
            step_out, step_json, code, sj_ = _escalated(cur, step_out, step_json, code, sj_, grp, k, rounds, inproc,
                                                        log, feedback_rounds, negotiated_buses, sizes)
            got, neg_g = sj_.get('buses'), sj_.get('grade')
            # (a grade whose check_drc gave no verdict -- a crash -- said so)
            log('route_bus: the group\'s ends left '
                + ('no grade' if not neg_g else
                   f'{len(neg_g["open"])} open, '
                   + (f'drc {neg_g["drc"]["n"]}' if neg_g['drc']['n'] is not None
                      else 'NO DRC VERDICT (check_drc gave none)')
                   + f', {len(neg_g["walled"]["walled"])} walled'
                   + (f', {len((neg_g.get("plane_balls") or {}).get("off") or [])} plane ball(s) on no pour'
                      if (neg_g.get('plane_balls') or {}).get('off') else '')) + ' on the board'
                + (f' -- NOT its whole result: {", ".join(neg_g["dead"])} wrote no board, and its nets are in no '
                   f'grade' if neg_g and neg_g.get('dead') else ''))
            bus_events.emit('group', 'end', group=s, exit=code, open=len(neg_g['open']) if neg_g else None,
                            drc=neg_g['drc']['n'] if neg_g else None,
                            walled=len(neg_g['walled']['walled']) if neg_g else None, summary=step_json)
            codes.append(code)
            for r_ in got or [dict(src=s, dest=g_[1], exit=None, grade=None, summary=None) for g_ in grp]:
                runs.append(dict(src=r_['src'], dest=r_['dest'], exit=1 if r_['exit'] is None else r_['exit'],
                                 grade=r_['grade'], summary=r_['summary'], group=step_json))
            if os.path.isfile(step_out):
                cur = step_out
    with contextlib.redirect_stdout(sys.stderr):
        copy_board(cur, out)
    HANDED_ON.add(out)
    code = 1 if not runs else max(codes)
    # (OUT as it ships, as route.py's main ships its own: its project's floor lowered to the copper it carries -- an
    # escalated group's narrower tracks and vias graded at their sizes -- and the design rules' ledger of what it
    # carries below the sizes asked for, every bus's nets, said in one line: shipped_rules)
    nets_, routed_ = set(), False
    for r_ in runs:
        try:
            s_ = json.load(open(r_['summary'])) if r_.get('summary') else {}
        except (OSError, ValueError):
            s_ = {}
        nets_ |= set(s_.get('routed') or []) | set(s_.get('refused') or {})
        routed_ = routed_ or bool(s_.get('routed'))
    rules_ = resolve_rules(board, log=lambda *a_, **k_: None, **sizes)
    install_escalation(board, rules_)
    # (...where a net was routed: a step that hands the board on as it came leaves its project as it came too --
    # stamped all the same, a board no bus had touched shipped with its floors lowered to the sizes asked for. And a
    # project that cannot be written is said, as route.py's main says it: OUT stands)
    design = None
    if routed_:
        try:
            design = shipped_rules(out, board, sorted(nets_), rules_, log)
        except Exception as e:      # noqa: BLE001
            log(f'route_bus: OUT\'s project floor NOT written ({type(e).__name__}: {e})')
    with open(json_out or os.path.join(work, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(board=board, out=out, buses=runs, plan=plan, exit=code, design_rules=design), f, indent=1)
    if not runs:
        log('route_bus: no bus found -- OUT is the board as it came')
    bus_events.emit('step', 'end', status=code, out=out, summary=json_out or os.path.join(work, 'summary.json'))
    return code, [r['grade'] for r in runs]


def _fresh(*paths):
    """an earlier run's outputs at these paths removed (a board with its siblings): a step that writes none must not
    leave them to be read as its own -- the next step's board, its summary's nets"""
    from copy_board import SIBLING_EXTS
    for p_ in paths:
        for q_ in [p_] + ([p_[:-len('.kicad_pcb')] + e_ for e_ in SIBLING_EXTS] if p_.endswith('.kicad_pcb') else []):
            if os.path.isfile(q_):
                os.remove(q_)


@contextlib.contextmanager
def _installed(run_rules):
    """THE RUN'S SIZES THIS PROCESS'S, while a group is routed from it -- as each bus's own process has them
    (route_bus). The driver lays no lane, but it merges the buses' boards and stamps the merged project's floor at
    the chain's sizes (negotiated_buses.merge, fanout_from_plan.copy_pro) and grades the merged board: under the
    chain's own defaults that stamp was 0.1 clearance, a 0.1 track, a 0.25 / 0.15 via whatever the run's sizes --
    lower-only, so a board routed at 0.2 handed on a project that said 0.1, graded there and read there by the
    next step"""
    import rules as _rules
    prev = _rules.active()
    _rules.install(run_rules)
    try:
        yield
    finally:
        _rules.install(prev)


def _ladder(board, sizes):
    """(the run's rules, its escalation policy's name, the rungs below the sizes asked for) of BOARD at `sizes`"""
    run_rules = resolve_rules(board, log=lambda *a_, **k_: None, **sizes)
    policy = install_escalation(board, run_rules)
    return run_rules, policy, escalation_ladder(run_rules, board)


def _below(rungs):
    """`rungs` as a group's ends model takes the rungs below its own sizes: [(the lanes' track, the via)]"""
    return [(r.track, r.via_size) for r in rungs]


def _no_room(sj):
    """whether a group's summary SJ is of a run whose ends model found a bus no room at its sizes, and room on a rung
    below (negotiated_buses.ENDS_NO_ROOM): nothing routed, and the rungs' to answer"""
    import negotiated_buses as nb_
    return (sj.get('ends') or {}).get('exit') == nb_.ENDS_NO_ROOM


def _group_key(sj):
    """a group's result as its escalation compares them: (nets open, violations, balls walled in), the fewest best --
    a group with no grade, one whose check gave no verdict, or one a bus of which wrote no board (its nets are in no
    grade: `dead`), the worst"""
    g = sj.get('grade')
    if not g or g.get('dead') or g['drc']['n'] is None:
        return (math.inf, math.inf, math.inf)
    return (len(g['open']), g['drc']['n'], len(g['walled']['walled']))


def _group_state(out, sj):
    """(its group's directory, its lay's work directory) of a group routed to OUT (negotiated_buses.route_grouped) whose
    summary is SJ -- the round whose board stands -- or None where that group kept no end (resume.pkl)"""
    work = out[:-len('.kicad_pcb')] + '.bus'
    r = int(sj.get('round') or 1)
    group = os.path.join(work, 'group' if r == 1 else f'group{r}')
    lay = os.path.join(work, f'round{r}.bus') if sj.get('rounds') else work
    return (group, lay) if os.path.isfile(os.path.join(group, 'resume.pkl')) else None


def _bus_nets(r):
    """the nets (the whole route's names) of a bus by its entry `r` in a group's summary -- those it routed and those
    it put back as they came -- or None where its step wrote no summary"""
    try:
        s_ = json.load(open(r['summary']))
    except (OSError, ValueError, TypeError, KeyError):
        return None
    return set(s_.get('routed') or []) | set(s_.get('refused') or {})


def _failed_buses(sj, grp):
    """the buses of group GRP [(src, dest)] whose nets the group's summary SJ has open -- or None where its result
    is not open nets of its buses alone: no grade, a violation, a ball walled in, a bus that wrote no summary, an
    open net no bus has. Those are the whole group's to answer"""
    g = sj.get('grade')
    if not g or g.get('dead') or g['drc']['n'] is None or g['drc']['n'] or g['walled']['walled'] or not g['open']:
        return None
    by_dest = {r_.get('dest'): r_ for r_ in sj.get('buses') or []}
    left, failed = set(g['open']), []
    for b in grp:
        nets = _bus_nets(by_dest.get(b[1]))
        if nets is None:
            return None
        if nets & left:
            failed.append(b)
        left -= nets
    return failed if failed and not left else None


def _kept(board, out, sj, failed, path):
    """PATH: the group's board OUT (its summary SJ) with the buses FAILED's nets as BOARD, the board the group came in
    with, has them -- their lanes and their escapes taken off, the copper they came with put back (put_back) -- and
    no longer protected: the board those buses are routed on again, every other bus's copper as it stands. Returns
    the nets put back (full names)"""
    import negotiated_buses as nb_
    from copy_board import copy_board
    from protected_nets import pro_path_for_board, read_protected_nets
    by_dest = {r_['dest']: r_ for r_ in sj['buses']}
    short = set().union(*(_bus_nets(by_dest[d_]) for _s, d_ in failed))
    with contextlib.redirect_stdout(sys.stderr):
        copy_board(out, path)
        pcb = parse_kicad_pcb_quiet(board)
    back = others_full(pcb, short)
    put_back(path, board, pcb, back)
    pro = pro_path_for_board(path)
    if os.path.isfile(pro):
        nb_._set_protected(pro, {n_: v_ for n_, v_ in read_protected_nets(pro).items() if n_ not in back})
    return back


def _bare_arrays(board, kept, failed, path):
    """PATH: the board KEPT without the copper it holds INSIDE the buses FAILED's ball arrays that BOARD, the board
    the group came in with, has not -- the other buses' escapes there, the field's: the board those buses' ends
    models are CAPTURED on. A bus's own run fans its array out for its capture among whatever copper stands there,
    and a ball it finds no way out for is no lane of the bus: on the kept board seven of one bus's 54 nets were lost
    so, before any model saw them. The group's model leaves that fanout out and measures every ball's menu on the
    kept board itself (group_ends.load_bus, BARE; served_balls), so the capture is made as on the board the group
    came in with, the kept buses' lanes beyond the arrays in its view. Returns the pieces taken off"""
    import group_ends as ge
    import make_bench as mb
    from copy_board import copy_board
    with contextlib.redirect_stdout(sys.stderr):
        copy_board(kept, path)
        pi = parse_kicad_pcb_quiet(board)
    boxes = []
    for r_ in sorted({s_ for s_, _d in failed} | {r_ for _s, d_ in failed for r_ in d_.split('+')}):
        fp = pi.footprints.get(r_)
        if fp is None or not is_array(fp):
            continue
        xs, ys = [p_.global_x for p_ in fp.pads], [p_.global_y for p_ in fp.pads]
        boxes.append((min(xs) - ge.BARE, min(ys) - ge.BARE, max(xs) + ge.BARE, max(ys) + ge.BARE))
    inb = lambda q: any(b_[0] <= q[0] <= b_[2] and b_[1] <= q[1] <= b_[3] for b_ in boxes)     # noqa: E731
    r3 = lambda q: (round(q[0], 3), round(q[1], 3))        # noqa: E731
    came = {frozenset((r3((s_.start_x, s_.start_y)), r3((s_.end_x, s_.end_y)))) | {s_.layer} for s_ in pi.segments}
    came |= {frozenset({r3((v_.x, v_.y)), 'via'}) for v_ in pi.vias}
    gone = [0]

    def off(token):
        def f(block):
            a_, b_, L_ = mb.block_geom(block, token)
            if not inb(a_) or (token == 'segment' and not inb(b_)):
                return False
            if (frozenset((r3(a_), r3(b_))) | {L_} if token == 'segment' else frozenset({r3(a_), 'via'})) in came:
                return False
            gone[0] += 1
            return True
        return f
    txt = open(path, encoding='utf-8').read()
    txt = mb.walk_blocks(txt, 'segment', off('segment'))
    txt = mb.walk_blocks(txt, 'via', off('via'))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)
    return gone[0]


def _bus_rung(board, prev, failed, grp, out_i, json_i, k, rounds, inproc, log, feedback_rounds, nb, start, below,
              sizes_i):
    """ONE RUNG FOR THE BUSES THAT FAILED, the others kept: `prev` (the group's board that stands, its summary) with
    the buses FAILED's nets as they came (_kept: <OUT_I stem>.bus/kept.kicad_pcb), and those buses routed on it as a
    group of their own at `sizes_i` (negotiated_buses.route_grouped: their ends model, their balls fanned out, their
    lanes -- every other bus's copper the board's, as any earlier step's; their captures on that board with their
    arrays bare, _bare_arrays: kept.bare.kicad_pcb). OUT_I is the whole group's board again;
    its summary JSON_I the whole group's: every bus of GRP, the kept ones as `prev` has them, and the grade of all of
    them on OUT_I against BOARD (board_grade). Returns (status, summary) -- the failed buses' own where they routed
    nothing (no room, no state: no board)"""
    prev_out, prev_sj = prev
    work_i = out_i[:-len('.kicad_pcb')] + '.bus'
    os.makedirs(work_i, exist_ok=True)
    kept = os.path.join(work_i, 'kept.kicad_pcb')
    _kept(board, prev_out, prev_sj, failed, kept)
    bare = os.path.join(work_i, 'kept.bare.kicad_pcb')
    _bare_arrays(board, kept, failed, bare)
    code_f, _g = nb.route_grouped(kept, out_i, failed, k, rounds, inproc, log, json_i, feedback_rounds, start=start,
                                  rungs_below=below, capture_board=bare, **sizes_i)
    try:
        sj_f = json.load(open(json_i))
    except (OSError, ValueError):
        sj_f = {}
    if not sj_f.get('grade') or not os.path.isfile(out_i):
        return code_f, sj_f
    new = {r_['dest']: r_ for r_ in sj_f.get('buses') or []}
    old = {r_['dest']: r_ for r_ in prev_sj['buses']}
    gone = {d_ for _s, d_ in failed}
    buses = [new[d_] if d_ in gone and d_ in new else
             dict(src=s_, dest=d_, exit=None, routed=0, grade=None, summary=None) if d_ in gone else old[d_]
             for s_, d_ in grp]
    dead = sorted(r_['dest'] for r_ in buses if not r_.get('summary'))
    grade = board_grade(board, out_i, [(r_['src'], r_['dest'].split('+'), sorted(_bus_nets(r_) or ())) for r_ in buses],
                        work_i, log=log, tag='group')
    if dead:
        grade['dead'], grade['ok'] = dead, False
    code_i = max([code_f] + [1 if r_['exit'] is None else r_['exit'] for r_ in buses])
    sj_i = dict(sj_f, board=board, buses=buses, grade=grade, exit=code_i,
                held=dict(board=prev_out, buses=[d_ for _s, d_ in grp if d_ not in gone]), own=dict(
                    board=kept, buses=sorted(gone), grade=sj_f.get('grade'), exit=code_f))
    with open(json_i, 'w', encoding='utf-8') as f:
        json.dump(sj_i, f, indent=1)
    return code_i, sj_i


def _escalated(board, step_out, step_json, code, sj, grp, k, rounds, inproc, log, feedback_rounds, nb, sizes):
    """(OUT, its summary's path, status, summary) of the group BOARD -> STEP_OUT as it stands, or escalated (route_buses:
    THE GROUP ESCALATED), each rung its own OUT and summary beside the group's, the escalation's record in the one
    that stands"""
    key0 = _group_key(sj)
    if key0 == (0, 0, 0):
        return step_out, step_json, code, sj
    if key0[0] == math.inf and not _no_room(sj):
        # (NO GRADE -- nothing routed (no capture, a model that ended with no state), or a check with no verdict: the
        # step's own result, named where it happened. The rungs answer nets a routed group left open; a run with no
        # result routed again at three more sizes was that run four times. BUT a group whose ends model found a bus
        # NO ROOM at these sizes, and room on a rung below: that is the rungs' to answer, and it ended for them)
        return step_out, step_json, code, sj
    run_rules, policy, rungs = _ladder(board, sizes)
    say = lambda key_, sj_=None: ('no room at its sizes' if _no_room(sj_ or {}) else 'no result') \
        if key_[0] == math.inf else f'{key_[0]} open, {key_[1]} violation(s), {key_[2]} walled'  # noqa: E731
    if not rungs:
        log(f'route_bus: the fab escalation ({policy}): no rung below the sizes asked for -- the group stands '
            f'({say(key0)})')
        return step_out, step_json, code, sj
    best = (key0, 0, step_out, step_json, code, sj)
    res = lambda key_: None if key_[0] == math.inf else list(key_)  # noqa: E731

    def left(sj_):
        """how many of the group's buses a result leaves to route again: those with a net open, or every one where
        it is not open nets of its buses alone (_failed_buses)"""
        f_ = _failed_buses(sj_, grp)
        return len(f_) if f_ is not None else len(grp)
    tried = [dict(rung=0, **_rules_dict(run_rules), result=res(key0))]
    # (the last run that kept its group's end, BY THE BUSES IT ROUTED: a rung starts from the end of a run of the
    # same buses -- the whole group's, or the same failed buses' a rung up)
    whole = frozenset(d_ for _s, d_ in grp)
    last = {whole: ((step_out, sj), key0)}
    for i, rr in enumerate(rungs, 1):
        # (THE BUSES THAT FAILED ALONE, where the board that stands has nets open by some of its buses and nothing
        # else amiss: the other buses' copper kept as it stands -- _bus_rung. Routed again whole, a rung laid every
        # bus anew at the finer sizes, and a bus that was clean a rung up came back with a net open: a three-bus
        # group had each bus clean at some rung and never all three at one. Else THE WHOLE GROUP: nothing stands yet
        # (no room at the sizes asked for), every bus failed, or a violation or a walled ball no one bus answers for)
        failed = _failed_buses(best[5], grp) if os.path.isfile(best[2]) else None
        if failed is not None and len(failed) == len(grp):
            failed = None
        who = frozenset(d_ for _s, d_ in failed) if failed else whole
        # (...each rung FROM THE RUN BEFORE IT: its group's model from that run's end, the findings its lay left
        # answered and every other lane held (group_ends --at-end), then the checks, the fanout and the lay at the
        # rung's sizes. Searched again from nothing, each rung cost what the first run had -- a three-bus group 51 minutes
        # at its first rung, the whole escalation past three hours)
        (last_o, key_l) = last.get(who) or ((None, {}), best[0])
        start = _group_state(*last_o) if last_o[0] else None
        log(f'route_bus: === the fab escalation ({policy}), rung {i} of {len(rungs)}: '
            + (f'{say(best[0])} on the board that stands -- {", ".join(d_ for _s, d_ in failed)} routed again alone, '
               f'the other bus{"" if len(grp) - len(failed) == 1 else "es"} kept, at {rung_line(rr)}' if failed else
               f'{say(key_l, last_o[1])} left by the run it starts from -- routed again whole at {rung_line(rr)}')
            + (f', from its group\'s end ({os.path.relpath(start[0], os.path.dirname(step_out))})' if start
               else ' -- no group\'s end kept by a run of these buses before: searched from the start'))
        sizes_i = dict(sizes, track_width=rr.track, fanout_track_width=rr.fan_track, via_size=rr.via_size,
                       via_drill=rr.via_drill)
        out_i = step_out[:-len('.kicad_pcb')] + f'.rung{i}.kicad_pcb'
        json_i = step_json[:-len('.summary.json')] + f'.rung{i}.summary.json'
        # (the rung's outputs its own: a rung that writes none -- no capture, a model that ended with no answer --
        # left an earlier run's board and summary into the same OUT to be read as this rung's result, and that run's
        # group directory to be the next rung's start)
        import shutil
        _fresh(out_i, json_i)
        shutil.rmtree(out_i[:-len('.kicad_pcb')] + '.bus', ignore_errors=True)
        with _installed(rr):
            if failed:
                code_i, sj_i = _bus_rung(board, (best[2], best[5]), failed, grp, out_i, json_i, k, rounds, inproc, log,
                                         feedback_rounds, nb, start, _below(rungs[i:]), sizes_i)
            else:
                code_i, _g = nb.route_grouped(board, out_i, grp, k, rounds, inproc, log, json_i, feedback_rounds,
                                              start=start, rungs_below=_below(rungs[i:]), **sizes_i)
                try:
                    sj_i = json.load(open(json_i))
                except (OSError, ValueError):
                    sj_i = {}
        key_i = _group_key(sj_i)
        tried.append(dict(rung=i, **_rules_dict(rr), result=res(key_i),
                          **(dict(buses=sorted(who)) if failed else {})))
        log(f'route_bus: rung {i}: {say(key_i, sj_i)}')
        # (...the last run that KEPT its group's end: a rung whose model ended with no state keeps none, and the rung
        # after it starts from the run before that one -- not from nothing. A run of the failed buses alone keeps
        # theirs: its group's directory is that run's own, whatever the whole group's summary says of rounds)
        if _group_state(out_i, sj_i):
            last[who] = ((out_i, sj_i), key_i)
        # (THE BOARD THAT STANDS: the fewest nets open, then violations, then balls walled in -- and of two alike,
        # the one with FEWER BUSES LEFT TO ROUTE AGAIN, then the earlier. The earlier of two alike stood whatever
        # their buses: a group came in with a net open by each of two buses, a rung left two nets open by one of
        # them and the other clean, and the rung after it routed both again from the board the group came in with
        # -- the bus that rung had made clean came back with a net open, and the group shipped it)
        if (key_i, left(sj_i), i) < (best[0], left(best[5]), best[1]) and os.path.isfile(out_i):
            best = (key_i, i, out_i, json_i, code_i, sj_i)
        if key_i == (0, 0, 0):
            break
    key_b, i_b, out_b, json_b, code_b, sj_b = best
    sj_b['escalation'] = dict(policy=policy, kept=i_b, rungs=tried)
    with open(json_b, 'w', encoding='utf-8') as f:
        json.dump(sj_b, f, indent=1)
    log('route_bus: the fab escalation: ' + (f'rung {i_b} stands ({say(key_b)}; {rung_line(rungs[i_b - 1])})' if i_b
                                             else f'no rung did better -- the sizes asked for stand ({say(key0)})'))
    return out_b, json_b, code_b, sj_b


def _route(board, out, src, dest, k, rounds, inproc, log, run_rules, json_out=None, settings=None, feedback_rounds=0):
    import make_bench as mb
    import flow_frame as ff
    import whole_route as wr
    from coherent_nets import coherent_nets, net_patterns, whole_pairs, net_keys
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
    import run_trace
    run_trace.provenance(work, {'stage': 'route_bus', 'board': board, 'out': out, 'src': src, 'dest': dest})
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    keys = net_keys(pcb)
    short = lambda n: keys.get(n, n)        # (a net by the whole route's name for it: its last part, else in full)
    # (DEST may name several parts, 'U4+U3': one source, several ends -- the whole route's trunk runs to the first, a
    # branch off it to each other, whole_frame)
    dests = ff.dest_refs(dest)
    for r in [src] + dests:
        if r not in pcb.footprints:
            log(f'route_bus: no footprint {r} on the board')
            return 2, None
    log(f'route_bus: {os.path.basename(board)}, copper layers {pcb.board_info.copper_layers}, {src} -> {dest}')

    # 1. the bus, its copper stripped
    names = sorted({n for d in dests for n in mb.pair_nets(pcb, src, d)})
    for d in dests:
        log(f'  {len(mb.pair_nets(pcb, src, d))} nets between {src} and {d}')
    import awx_settings
    # the nets other pads alone keep out (pulled_nets: a pull, a termination, a test point, another part's pin) laid as
    # lanes, each a place in the bus's order: a fly-by DDR3 bus's address and command nets, terminated to its Vtt past
    # the memory, are as much the bus's as its data -- left to the router after it, a board lost 12 of 26 round the
    # lanes laid before them
    pulled = {k_: v_ for d in dests for k_, v_ in pulled_nets(pcb, src, d, names).items()}
    if pulled:
        names = sorted(set(names) | set(pulled))
        log(f'  {len(pulled)} more with other pads beside their two ends (a pull, a termination, a test point, another '
            f'part\'s), laid as lanes, each stub left to the rest of the chain: ' + ', '.join(f'{short(n)} ({", ".join(p.component_ref for p in v)})'
                                               for n, v in sorted(pulled.items())))
    # a pair through a termination ACROSS it between the arrays (waypoint_pairs: a two-pad part, a pad on each leg -- a
    # clock pair's termination) laid as a lane as every other termination is: the part's pads off the pair's nets for
    # the step, a stub each left to the rest of the chain (pulled). Refused -- the whole route lays no waypoint --, a
    # DDR3 bus's clock pair was left to the chain whole
    wps = {}
    for d in dests:
        for leg, pads_ in waypoint_pairs(pcb, names, src, d).items():
            wps.setdefault(leg, []).extend(pads_)
    if wps:
        for n in names:
            if short(n) in wps:
                pulled.setdefault(n, []).extend(wps[short(n)])
        log(f'  {len(wps)} more through a termination part between the arrays, across the pair, laid as lanes, each '
            f'stub to the part left to the rest of the chain: '
            + ', '.join(f'{leg} ({", ".join(sorted({p.component_ref for p in v}))})' for leg, v in sorted(wps.items())))
    refused = {}
    halves = half_pairs(pcb, names)
    if halves:
        refused.update(halves)
        names = [n for n in names if short(n) not in halves]
        log('  refused, left to the rest of the chain with their pair: '
            + '; '.join(f'{leg}: {why}' for leg, why in sorted(halves.items())))
    txt = open(board, encoding='utf-8').read()
    # the escapes an earlier step laid the bus's balls at its source (escape_blocks) stay as the bus's teeth: fanned out
    # again for the bus alone, a full array sends a ball out by any face with room
    # THE GROUP'S ENDS (GROUP_ENDS=DIR, group_ends.py's output): the board is the group's field -- every array of the
    # buses off one array fanned out as their one model chose, this bus's teeth and berths among them -- and the bus's
    # copper on it is its ends: nothing stripped, nothing fanned out, no end chosen again (the fanout stage of each
    # round hands the round the model's own plan, group_ends.bus_stage)
    group = awx_settings.get('GROUP_ENDS')
    kept, n_cut = [], 0
    if group:
        log(f'  the group\'s ends ({group}): the bus\'s copper on the board is its teeth and berths, kept as laid')
    else:
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
    attached = []               # (the pull pads joined to their lanes on the routed board, attach_pulls)
    base = os.path.join(work, 'base.kicad_pcb')
    with open(base, 'w', encoding='utf-8') as f:
        f.write(txt)
    import fanout_from_plan as fp
    fp.copy_pro(board, base)
    # (a ROW PART at a destination -- a QFN: each pad has one escape, straight out of its row, and a decoupling cap
    # standing across one leaves its lane no end at all; such passives are moved off the row escapes first, as the cap
    # step moves a BGA's off its fanout: clear_row_escapes)
    by_name_ = {nn_.name: nn_ for nn_ in pcb.nets.values() if nn_.name}
    keep_ = {p_.component_ref for p_ in pull_pads} | {
        q_.component_ref for n_ in names if n_ in by_name_ for q_ in by_name_[n_].pads}
    for d in ([] if group else dests):
        clear_row_escapes(base, d, names, sorted(keep_ - {src, d}), work, log)

    # 2. the source fanned out, the project floor, the ladder
    log(f'  rules: clearance {te.SPEC_CLEARANCE}, track {te.TRACK}, fanout {sr.FAN_TRACK}/{sr.FAN_CLEAR}, '
        f'via {te.VIA_SIZE}/{te.VIA_DRILL}  [{run_rules.source}]')
    fanned = os.path.join(work, 'fanned.kicad_pcb')
    if group:
        import shutil
        import group_ends
        shutil.copyfile(base, fanned)
        fp.copy_pro(base, fanned)
        with contextlib.redirect_stdout(sys.stderr):
            fix_project_for_output(fanned, clearance=te.SPEC_CLEARANCE, track_width=sr.FAN_TRACK,
                                   via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL, verbose=False)
        failed = []
        # (the run's nets and their order the model's: the ladder, one net a line)
        mine_ = {short(n) for n in names}
        gnets = [n for n in group_ends.bus_nets(group, dest) if n in mine_]
        # (the ends' layers the bus's own solve's, where the model left them to it -- its plan's `held_layers` names
        # the lanes it did not; GROUP_LAYERS=free: every end's, a bench's)
        group_free = awx_settings.get('GROUP_LAYERS') == 'free' or group_ends.layers_free(group, dest)
        with open(os.path.splitext(fanned)[0] + '.ladder.txt', 'w', encoding='utf-8') as f:
            f.write(f'# the group\'s ends: the nets its model gave the bus to {dest} ends, in its order\n'
                    + ''.join(n + '\n' for n in gnets))
        log(f'  no fanout: {len(gnets)} of the bus\'s {len(names)} nets with ends in the group\'s model')
    else:
        # (a ball the comb refuses HOLDS the escape the board came with at the source, where every piece of it stands
        # clear of the board before the comb, and the comb fans the bus's other balls again round it, until it refuses
        # none it can hold: on a shared field, which planned every bus's balls together, a whole-array bus fans its own
        # balls again round the other buses' reserved runs, and its comb's stubs took the dog-bone sites of six deep
        # balls of a memory bus the field had escaped, so their nets were never in the ladder)
        held_e, base_c = [], base
        while True:
            with contextlib.redirect_stdout(sys.stderr):
                n_t, n_v, failed = mb.fanout_source(base_c, fanned, src, [n for n in names if n not in held_e])
            more = hold_board_escapes(board, base_c, os.path.join(work, 'base_held.kicad_pcb'), pcb, src, failed,
                                      names, short) if failed else []
            if not more:
                break
            held_e += more
            base_c = os.path.join(work, 'base_held.kicad_pcb')
            mates_ = [short(n) for n in more if n not in set(failed)]
            log(f'  {src}: the comb\'s refused ball(s) holding the escape the board came with: '
                + ', '.join(short(n) for n in more if n in set(failed))
                + (f' (and their pair partners {", ".join(mates_)})' if mates_ else '')
                + ' -- the rest fanned again round them')
        with contextlib.redirect_stdout(sys.stderr):
            fix_project_for_output(fanned, clearance=te.SPEC_CLEARANCE, track_width=sr.FAN_TRACK,
                                   via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL, verbose=False)
        log(f'  {src} fanned out: {n_t} tracks, {n_v} vias' + (f', refused {failed}' if failed else ''))
        if failed:
            # (a bus ball the engine's comb refused -- boxed by the comb's own stubs, and with the whole-array fanout
            # the via sites kept for the array's other balls -- escaped by the whole-array plan round the comb, at the
            # run's size: the plan reaches lines the comb does not (a ball line on a layer below the balls'), so the
            # ball has a tooth and the run takes it). The plan lays its vias as if the passives beneath the array move,
            # and on a plain bus, whose rounds run no cap step, the step's own moves them then (--beneath-only): held
            # where they stand, a refused ball's only way out ran under a decoupling cap's GND pad
            import shutil
            import joint_escape as je
            resc = os.path.join(work, 'fanned_refused.kicad_pcb')
            # (each ball toward its own part: a bus to several parts, DEST 'A+B', its nets going to each)
            to_ = dest
            if len(dests) > 1:
                to_ = {}
                for d_ in dests:
                    for q_ in pcb.footprints[d_].pads:
                        if q_.net_name in set(failed):
                            to_.setdefault(short(q_.net_name), d_)
            far_ = je.far_face(pcb, src, dest) if isinstance(to_, str) else \
                {n_: je.far_face(pcb, src, d_) for n_, d_ in to_.items()}
            with contextlib.redirect_stdout(sys.stderr):
                _rr, reps_ = je.fan_array(fanned, resc, src, list(failed), [],
                                          list(pcb.board_info.copper_layers), drops=(),
                                          far=far_, log=lambda *a: None,
                                          rungs=[run_rules], exit_rays=True, escape_to=to_)
            bare_ = {b_.split('#')[0] for b_ in reps_[-1]['bare']}
            if len(bare_) < len(failed):
                shutil.copyfile(resc, fanned)
                fp.copy_pro(resc, fanned)
                plain_caps(fanned, [src] + dests, names, je.passives_beyond(fanned, [src, dest], names), work,
                           'caps_refused', lambda s_: log(f'  {src}: caps: ' + s_))
                served_ = [short(n) for n in failed if short(n) not in bare_]
                failed = [n for n in failed if short(n) in bare_]
                log(f'  {src}: the comb\'s refused ball(s) escaped by the whole-array plan: {", ".join(served_)}'
                    + (f'; still refused {failed}' if failed else ''))
            else:
                log(f'  {src}: the whole-array plan has no escape for the comb\'s refused ball(s) either')
        # (no new escape between a PAIR's two legs already on the board: the comb and the rescue fan a ball blind to
        # the pairs whose escapes they keep -- a shared field's, or held -- and one laid between an LVDS pair's legs
        # had the whole route's fanout audit refuse every round's board for the split pair, the bus left to its last
        # resort, 28 nets open; taken out, its net is refused alone)
        gated = pair_gate_cuts(base_c, fanned, pcb, src, names, short)
        if gated:
            txt_g, _n = mb.strip_pair_copper(open(fanned, encoding='utf-8').read(), pcb, gated)
            with open(fanned, 'w', encoding='utf-8') as f:
                f.write(txt_g)
            failed = list(failed) + [n for n in gated if n not in failed]
            log(f'  {src}: escape(s) between a pair\'s legs taken out, refused: {", ".join(short(n) for n in gated)}')
        # (...and a pair the comb itself split -- a deeper ball's escape between its legs -- planned again with its
        # legs together: repair_comb_splits)
        repair_comb_splits(fanned, pcb, src, dest, [n for n in names if n not in set(failed)], short, run_rules,
                           list(pcb.board_info.copper_layers), work, False, log)
    with contextlib.redirect_stdout(sys.stderr):
        if not group:
            mb.write_ladder(fanned, names, log=log)
        ladder = coherent_nets(10 ** 6, fanned)
    K = min(int(k), len(ladder)) if k else len(ladder)
    with contextlib.redirect_stdout(sys.stderr):
        nets = whole_pairs(ladder, K, fanned)            # (the chain's own cut: coherent_nets(K) cuts no pair)
    cut = [n for n in ladder[:K] if n not in nets]
    log(f'  the run: K={K} of the ladder\'s {len(ladder)} nets'
        + (f' -- {len(nets)} nets: {", ".join(cut)} left out with their pair\'s other leg' if cut else ''))
    bus_events.emit('bus', 'start', nets=len(nets), bus_nets=len(names), k=K, work=work)

    # 3. the flow frame
    with contextlib.redirect_stdout(sys.stderr):
        q, cx, cy = ff.turn_for_ends(parse_kicad_pcb(fanned), src, dest, set(nets))
    frame = os.path.join(work, 'frame.kicad_pcb')
    bad = ff.turn_file(fanned, frame, q, cx, cy)
    if bad:
        log('route_bus: the turn into the flow frame failed: ' + '; '.join(bad[:6]))
        return 2, None
    log(f'  turned into the flow frame: {q} quarter turn(s) about ({ff._fmt(cx)}, {ff._fmt(cy)})')
    # (the turn ON FILE, for whoever reads the run's frame coordinates on the board -- bus_trace.py: every file under
    # work/run is in the flow frame, a plan of pair chirality -1 also turned over about the board's mirror axis)
    with open(os.path.join(work, 'frame.json'), 'w', encoding='utf-8') as f_:
        json.dump({'q': q, 'cx': cx, 'cy': cy,
                   'to_board': 'board = (cx, cy) + flow_frame._q(fx - cx, fy - cy, (4 - q) % 4)',
                   'plan_mirror': 'a plan of chirality -1 (fo.plan.json chi): its y -> 2 CY - y first, CY = '
                                  'bga_fanout.flip_frame.mirror_axis(the flow-frame board)'}, f_, indent=1)
    # 4. the whole route
    wr.INPROC = inproc
    # (JOINT_DIR: buses off one shared array, each its own route_bus, their rounds' first plans one COUPLED solve --
    # whole_route.joint_sync, whole_joint.py: this bus's name and its flow frame's turn, and its end said)
    jsync = {'JOINT_BUS': dest, 'JOINT_TURN': f'{q},{cx},{cy}'} if awx_settings.get('JOINT_DIR') else {}
    try:
        rc, grade = wr.chain(K, os.path.join(work, 'run'), rounds, base=frame, dest=dest, F=feedback_rounds,
                             settings={**(settings or {}), **STEP_SETTINGS, _rules.SETTING: _rules.as_setting(run_rules),
                                       'ROUND_EDITS_BUS': f'{src}:{dest}', 'ROUND_EDITS_TURN': f'{q},{cx},{cy}',
                                       **jsync,
                                       # (the group's ends: each end's layer the model's, held -- the model chose
                                       # them so that no two runs of any bus share a line on a layer under an array)
                                       # (GROUP_LAYERS=free, a bus alone on its arrays: its via ends' layers the
                                       # solve's own, as the model's check asked them)
                                       **({'VIA_ENDS': '0'} if group and not group_free else {})})
    finally:
        if jsync:
            # (the run's laid result, the one handed on, for the buses still routing: their stages plan round it --
            # joint_lanes -- and only then this bus said ended)
            try:
                seq_ = _routed(os.path.join(work, 'run'))
                if seq_:
                    with awx_settings.given({**awx_settings.environ(), **jsync}):
                        import joint_lanes
                        joint_lanes.publish_board(seq_, list(nets), 'final')
            finally:
                open(os.path.join(awx_settings.get('JOINT_DIR'), f'{dest}.finished'), 'w').close()
    seq = _routed(os.path.join(work, 'run'))
    for n in names:
        if short(n) in {short(f) for f in failed}:
            refused.setdefault(short(n), 'no escape at the source fanout')
        elif short(n) not in ladder:
            refused.setdefault(short(n), 'no ends in the group\'s model' if group else
                               'not in the ladder: no free stub end at the source')
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
        HANDED_ON.add(out)          # (OUT written either way: routed, or the board as it came)
        summary.update(routed=sorted(routed), refused=dict(sorted(refused.items())), grade=g,
                       secs=int(time.time() - t0), exit=code, why=why if code else None)
        with open(json_out or os.path.join(work, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=1)
        bus_events.emit('bus', 'end', exit=code, why=why if code else None, routed=len(routed),
                        refused=len(refused), secs=summary['secs'], grade=g)
        log(g or f'route_bus: {why}')
        if code:
            log(f'route_bus: {why} -- {os.path.basename(out)} is the board as it came')
        return code, g

    if awx_settings.get('ENDS_ONLY'):
        # THE ENDS ONLY (a negotiation's solo run, for the bus's teeth): OUT is the round's ends board turned back --
        # the fanout's escapes on the layers of the first plan, nothing laid past them -- and the summary names the
        # nets with ends there as its 'routed'. Not a board to hand on: read for where the bus leaves its array
        ends_b = os.path.join(work, 'run', 'ends.kicad_pcb')
        if not os.path.isfile(ends_b):
            return finish(1, f'the whole route chose no ends (exit {rc})', grade)
        bad = ff.turn_file(ends_b, out, -q, cx, cy)
        if bad:
            return finish(2, 'the turn back failed: ' + '; '.join(bad[:6]), grade)
        summary['ends_only'] = True
        return finish(0, '', grade, routed=nets)

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
        wr.run([checker('check_connected.py'), routed_pcb, '--nets'] + net_patterns(nets, keys), awx_settings.environ(),
               log_path=os.path.join(work, 'conn_open.log'), own_process=True)
        opened = sorted(open_nets(os.path.join(work, 'conn_open.log'), keys) & set(nets))
        for n in opened:
            refused.setdefault(n, 'left open by the whole route (its best round)')
        nets = [n for n in nets if n not in opened]
        if not nets:
            return finish(1, f'the whole route connected none of the bus (exit {rc})', grade)
    back = [n for n in names if short(n) not in set(nets)]
    if back:
        left_ = put_back(routed_pcb, board, pcb, back)
        log(f'  put back as they came: {", ".join(sorted(short(n) for n in back))}'
            + (f' -- {left_} piece(s) of their earlier copper left out, no longer clear' if left_ else ''))
    # (the stubs the step joins itself: a stub pad of a routed net near either array, that a short tie reaches from its
    # net's copper there -- its tooth's or berth's via, else its ball -- joined on the routed board (attach_pulls). The
    # rest stay the chain's)
    if held:
        pad_net_ = {(p_.component_ref, p_.pad_number): short(p_.net_name) for p_ in pull_pads}
        mine_ = [h for h in held if pad_net_.get((h[0], h[1])) in set(nets)]
        if mine_:
            _rest, att_ = attach_pulls(routed_pcb, mine_, [src] + dests, log)
            held = [h for h in held if h not in att_]
            attached = attached + att_

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
               awx_settings.environ(), log_path=lp, own_process=True)
        n, length_rows[tag] = wr.drc_count(jp)
        if n is not None:
            return n
        txt = open(lp, encoding='utf-8', errors='replace').read()
        m = re.search(r'FOUND (\d+) DRC VIOLATIONS', txt)
        return 0 if 'NO DRC VIOLATIONS' in txt else (int(m.group(1)) if m else None)
    d_in = drc(board, 'input')
    # ...and NEVER ALL OR NOTHING: a violation the step made on another net's copper it laid -- an array's other nets,
    # fanned with the bus on the group's field, a passive the cap nudge could not move off one of its vias -- puts THAT
    # net back as it came; a bus net that fails it on the board -- open there, or named in a
    # violation of the bus's (or of the board's, when the board came out worse) -- goes back as it came; the rest are
    # graded again. The bus is handed on whenever its own nets come out connected and clean: sdi_mipi_conv's SDI bus,
    # laid 14 nets clean, was refused whole for one GND cap pad on a DDR3 address via the whole-array field had laid
    # under it
    zone_nets = {short(pcb.nets[z.net_id].name) for z in (pcb.zones or []) if z.net_id in pcb.nets}
    put_others = set()
    for _ in range(6):
        pats = net_patterns(nets, keys)
        cc = wr.run([checker('check_connected.py'), routed_pcb, '--nets'] + pats, awx_settings.environ(),
                    log_path=os.path.join(work, 'conn.log'), own_process=True)[0]
        d_bus, d_out = drc(routed_pcb, 'bus', '--nets', *pats), drc(routed_pcb, 'board')
        # (whole-array: the step laid the arrays' whole fanout and moved the passives off it each round -- the chain's
        # cap nudge, whole_route -- whose own pad-to-via hits the nudge could not clear are the board's to report; the
        # bus's nets are held clean)
        worse = None in (d_out, d_in) or d_out > d_in
        ok_drc = d_bus == 0 and not worse
        if cc == 0 and ok_drc:
            break
        # (another net's copper the step laid, in a violation of the bus's or one the board did not come with: back
        # first -- the bus is what the step is for. Never a plane's net, whose drops the whole-array field laid by the
        # dozen)
        named = drc_nets(os.path.join(work, 'drc_bus.log'), keys) if d_bus else set()
        if worse and d_out is not None:
            named |= drc_nets(os.path.join(work, 'drc_board.log'), keys)
        others_ = changed_nets(board, routed_pcb, named - set(nets) - zone_nets - put_others)
        if others_:
            put_others |= others_
            left_ = put_back(routed_pcb, board, pcb, sorted(others_full(pcb, others_)))
            log(f'  another net\'s copper the step laid in a violation, put back as it came: {", ".join(sorted(others_))}'
                + (f' ({left_} piece(s) of its earlier copper left out, no longer clear)' if left_ else '')
                + ' -- the bus graded again')
            continue
        bad = set(nets) & (open_nets(os.path.join(work, 'conn.log'), keys)
                           | drc_nets(os.path.join(work, 'drc_bus.log'), keys)
                           | (drc_nets(os.path.join(work, 'drc_board.log'), keys)
                              if None not in (d_out, d_in) and d_out > d_in else set()))
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
    # (drc: the violations the step answers for, 0 clean -- the bus's own and the board's past those it came with)
    n_drc = '?' if None in (d_bus, d_out, d_in) else max(d_bus, d_out - d_in, 0)
    g = (f'BUS {src}->{dest} K={K} {grade.split(" ", 2)[2] if grade else ""} | on the board: vias={v} copper={c}mm '
         f'connected={int(cc == 0)} drc={n_drc} (bus {d_bus}, board {d_out} vs {d_in} in'
         + (f', {length_rows["bus"]} length rows' if length_rows.get('bus') else '') + ') '
         + (f'planes={planes["out"]} vs {planes["in"]} in ' if planes and planes.get('out') is not None else '')
         + f'walled={len(walled["walled"])} secs={int(time.time() - t0)}')
    summary['drc'] = dict(bus=d_bus, board=d_out, input=d_in)
    bus_events.emit('bus', 'graded', vias=int(v), copper_mm=int(c), connected=cc == 0, drc=n_drc, drc_bus=d_bus,
                    drc_board=d_out, drc_in=d_in, walled=len(walled['walled']), nets=len(nets))
    if length_rows.get('bus'):
        summary['length_rows'] = dict(bus=length_rows['bus'], board=length_rows.get('board'),
                                      input=length_rows.get('input'))
        log(f'  laid untuned: {length_rows["bus"]} row(s) of the board\'s length rules on the bus\'s nets, named, not '
            f'counted (#1363): {os.path.join(work, "drc_bus.log")}')
    if planes:
        summary['planes'] = planes
    summary['walled'] = walled
    if cc != 0 or d_bus != 0 or None in (d_out, d_in):
        # (a checker that gave NO VERDICT -- a crash: no JSON, no count, no 'FOUND N ISSUES' or 'ALL NETS FULLY
        # CONNECTED!' -- said so, not read as the bus failing its grade)
        txt_ = open(os.path.join(work, 'conn.log'), encoding='utf-8', errors='replace').read() \
            if os.path.isfile(os.path.join(work, 'conn.log')) else ''
        mute_ = [w_ for w_, bad_ in (('check_connected', cc != 0 and not re.search(
            r'ALL NETS FULLY CONNECTED!|FOUND \d+ ISSUES', txt_)), ('check_drc', None in (d_bus, d_out, d_in))) if bad_]
        if mute_:
            return finish(1, f'{" and ".join(mute_)} gave no verdict (its log in {work}) -- the bus not handed on', g)
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
    is not asked. A WALL is copper no later step moves (_fixed_nets): the bus's own, a net an earlier step protected
    (#521), a pour's, KiCad-locked or held copper, and every pad. Another net's escape is none: the route step takes it
    up and lays it again elsewhere -- QSPI_SCK, U1.A5 of a DDR3 bus's source, read walled with USR_BTN's B.Cu stub under
    the ball; route.py ripped that stub, dropped a via beside the ball and laid USR_BTN again, both connected.
    {balls, rung, walled: ['REF.PAD'], secs}"""
    from kicad_parser import parse_kicad_pcb
    t0 = time.time()
    bus = set(names)
    with contextlib.redirect_stdout(io.StringIO()):
        b_out = parse_kicad_pcb(routed)
    rung = _cage_rung(b_out)
    asked = {r: [p.pad_number for p in pads] for r, pads in _cage_balls(b_out, refs, bus).items()}
    now = _caged_on(b_out, asked, _fixed_nets(b_out, routed, bus), rung)
    walled = []
    if now:
        with contextlib.redirect_stdout(io.StringIO()):
            b_in = parse_kicad_pcb(board)
        was = set(_caged_on(b_in, {r: [k.split('.', 1)[1] for k in now if k.split('.', 1)[0] == r] for r in asked},
                            _fixed_nets(b_in, board, bus), rung))
        walled = [k for k in now if k not in was]
    out = dict(balls=sum(len(v) for v in asked.values()), rung=list(rung), walled=walled,
               secs=round(time.time() - t0, 1))
    if walled:
        nets_ = {f'{r}.{p.pad_number}': p.net_name for r in asked for p in b_out.footprints[r].pads}
        log(f'  WALLED IN by the step, reached from nowhere at {rung[0]}/{rung[1]}/{rung[2]} (the fab ladder\'s finest): '
            + ', '.join(f'{k} ({nets_.get(k)})' for k in walled) + ' -- each net ships open past the locked lanes')
    return out


def plane_balls_grade(routed, refs, log=print):
    """the PLANE BALLS of the step's arrays `refs` on the board `routed`, POURED FOR REAL: its zones filled by KiCad's
    own filler (kicad_exact_fill.refill_islands_ex), each ball on a net with a pour read as joined to an island of it
    (exact_clusters: the fill's own geometry, no model of it) or not -- a ball fanned out as a signal and left a stub,
    a drop whose via stands in a part of a split layer its net does not pour, a pour a bus's run cut the ball off
    from. And each such net's islands, a pour cut to pieces by runs across it. "No ball walled in" is not "connected":
    a board with none walled had a quarter of its plane balls on no pour. {balls, joined, off: ['REF.PAD (net)'],
    islands: {net: count}, secs}; with no KiCad to pour with, {skipped: why}"""
    import kicad_exact_fill as kf
    from kicad_parser import parse_kicad_pcb
    t0 = time.time()
    isl, st = kf.refill_islands_ex(routed)
    if isl is None:
        log(f'  plane balls: not poured ({st.reason}{": " + st.detail if st.detail else ""}) -- not checked')
        return dict(skipped=st.reason)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(routed)
    # (the ball arrays among them: a row part's pins on a plane's net are the chain's later steps' to tap)
    import escape_moves as em
    refs = [r for r in refs if r in pcb.footprints and not em.row_part(pcb.footprints[r])]
    by_net = collections.defaultdict(list)
    for (nm, layer), polys in isl.items():
        by_net[nm] += [(layer, p_) for p_ in polys]
    n, off, islands = 0, [], {}
    for nid, net in sorted(pcb.nets.items(), key=lambda kv: kv[1].name or ''):
        balls = [p_ for r in refs if r in pcb.footprints for p_ in pcb.footprints[r].pads if p_.net_id == nid]
        if not net.name or net.name not in by_net or not balls:
            continue
        with contextlib.redirect_stdout(io.StringIO()):
            cl = kf.exact_clusters(pcb, nid, by_net[net.name])
        joined = {id(p_) for c in cl if c.get('islands') for p_ in c.get('pads') or ()}
        n += len(balls)
        off += [f'{p_.component_ref}.{p_.pad_number} ({net.name})' for p_ in balls if id(p_) not in joined]
        islands[net.name] = len(by_net[net.name])
    out = dict(balls=n, joined=n - len(off), off=sorted(off), islands=islands, secs=round(time.time() - t0, 1))
    log(f'  plane balls, poured for real: {n - len(off)} of {n} joined to a pour of their net'
        + (f' -- NOT: {", ".join(sorted(off)[:12])}{" ..." if len(off) > 12 else ""}' if off else ''))
    return out


def board_grade(board, routed, buses, work, log=print, tag='board'):
    """A bus step's output `routed` graded against the board it came in with, `board`, for the three things a step
    answers for on the BOARD -- `buses` [(source, [destination parts], [its nets, full names])]: every bus net's pads on
    its own bus's parts one piece of copper there (check_connected's test, zones credited; a net's pads elsewhere -- a
    termination's, a test point's -- are the chain's later steps': `open`, full names), no DRC violation the step made
    (check_drc at the project's floor, margin 0.1: the bus nets' own and the board's past those it came with -- `drc`),
    no ball of the buses' arrays walled in (walled_grade -- `walled`), and every plane ball of theirs joined to a pour
    of its net (plane_balls_grade -- `plane_balls`, where KiCad is there to pour). A step's own grade reads its own board
    alone; buses routed together (negotiated_buses' merge, the bus boards check) are graded here on the board they hand
    on. {open, nets, drc: {bus, board, input, n}, walled, ok}; its files in `work`, prefixed `tag`"""
    import awx_settings
    import whole_route as wr
    import check_connected as _cc
    from coherent_nets import net_keys, net_patterns
    from kicad_parser import parse_kicad_pcb
    os.makedirs(work, exist_ok=True)
    checker = lambda name: os.path.join(HERE, '..', 'py_router', name)  # noqa: E731
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(routed)
    keys = net_keys(pcb)
    # (a net by its full name or the whole route's -- a step's summary names them so: by_key)
    by_name = {n.name: (i, n) for i, n in pcb.nets.items() if n.name}
    by_name.update({keys.get(n.name, n.name): (i, n) for i, n in pcb.nets.items() if n.name})
    full = lambda n: by_name[n][1].name if n in by_name else n  # noqa: E731
    buses = [(s, list(ds), [full(n) for n in ns]) for s, ds, ns in buses]
    names = sorted({n for _s, _d, ns in buses for n in ns})
    refs = sorted({r for s, ds, _n in buses for r in [s] + list(ds)})
    segs, vias, zones = (collections.defaultdict(list) for _ in range(3))
    for s_ in pcb.segments:
        segs[s_.net_id].append(s_)
    for v_ in pcb.vias:
        vias[v_.net_id].append(v_)
    for z_ in pcb.zones or ():
        zones[z_.net_id].append(z_)
    open_ = set()
    for s, ds, ns in buses:
        parts = {s} | set(ds)
        for n in ns:
            if n not in by_name:
                open_.add(n)
                continue
            nid, net = by_name[n]
            pads = [p for p in net.pads if p.component_ref in parts]
            if len(pads) < 2:
                continue                                    # (nothing of the bus's to join)
            with contextlib.redirect_stdout(io.StringIO()):
                r_ = _cc.check_net_connectivity(nid, segs[nid], vias[nid], pads, zones[nid], pcb_data=pcb)
            if not r_.get('connected'):
                open_.add(n)
    open_ = sorted(open_)
    pats = net_patterns(names, keys)

    def drc(path, sub, *more):
        jp = os.path.join(work, f'{tag}_drc_{sub}.json')
        if os.path.exists(jp):
            os.remove(jp)
        wr.run([checker('check_drc.py'), path, '--clearance-margin', '0.1', '--max-print', '0', '--json', jp, *more],
               awx_settings.environ(), log_path=jp[:-len('.json')] + '.log', own_process=True)
        return wr.drc_count(jp)[0]
    d_bus, d_out, d_in = drc(routed, 'bus', '--nets', *pats), drc(routed, 'board'), drc(board, 'input')
    # (the violations the step made: the board's past those it came with, of which the bus nets' own are a part -- the
    # larger of the two, never their sum, which counted a bus net's new violation twice)
    n_drc = None if None in (d_bus, d_out, d_in) else max(d_bus, d_out - d_in, 0)
    walled = walled_grade(board, routed, refs, names, log)
    plane_balls = plane_balls_grade(routed, refs, log)
    # (a ball JOINED TO A POUR of its net is served: it needs no way out, and is not walled in -- the walls' grade
    # asks for a track from the ball out of its window, and named a plane ball whose drop stood among a bus's lanes,
    # on its pour all the while)
    if walled.get('walled') and plane_balls.get('islands') is not None:
        nets_ = {f'{r}.{p_.pad_number}': p_.net_name for r in refs if r in pcb.footprints
                 for p_ in pcb.footprints[r].pads}
        off_p = set(plane_balls.get('off') or [])
        on_pour = [k_ for k_ in walled['walled'] if nets_.get(k_) in plane_balls['islands']
                   and f'{k_} ({nets_.get(k_)})' not in off_p]
        if on_pour:
            walled = dict(walled, walled=[k_ for k_ in walled['walled'] if k_ not in on_pour], on_pour=on_pour)
            log(f'  not walled in, joined to a pour of their net: {", ".join(on_pour)}')
    # (...and every plane ball of the arrays on a pour of its net, poured for real: a run on a plane's layer is the
    # step's to lay only where it cuts no ball off its pour -- measured where KiCad is there to pour, part of the
    # verdict there)
    off_ = plane_balls.get('off') or []
    ok = not open_ and n_drc == 0 and not walled['walled'] and not off_
    log(f'  on the board: {len(open_)} bus net(s) open of {len(names)}'
        + (f' ({", ".join(keys.get(n, n) for n in open_[:8])})' if open_ else '')
        + f', drc {n_drc} (bus {d_bus}, board {d_out} vs {d_in} in), walled {len(walled["walled"])}'
        + (f', {len(off_)} plane ball(s) on no pour of their net' if off_ else '')
        + (' -- clean' if ok else ''))
    return dict(open=open_, nets=len(names), drc=dict(bus=d_bus, board=d_out, input=d_in, n=n_drc), walled=walled,
                plane_balls=plane_balls, ok=ok)


def _cage_rung(pcb):
    """(track, via, clearance) the widths a cage is tested at: the finest rung of the fab ladder for `pcb` under the
    step's escalation policy -- the narrowest copper a later step may lay -- and, where the policy has no rung (off:
    nothing is narrowed), the run's own sizes"""
    from list_nets import escalation_rungs
    rungs = escalation_rungs(len(pcb.board_info.copper_layers or ()) or 4)
    if not rungs:
        import rules as _rules
        r = _rules.active()
        return r.track, r.via_size, r.clearance
    finest = rungs[-1]
    return finest['track_width'], finest['via_diameter'], finest['clearance']


def _cage_balls(pcb, refs, bus):
    """{ref: [pad]} the balls of `refs` a cage test asks: on a net of none of `bus` (full names) with another pad"""
    out = {}
    for r in refs:
        foot = pcb.footprints.get(r)
        if foot is not None:
            out[r] = [p for p in foot.pads if p.net_id and p.net_name and p.net_name not in bus
                      and p.pad_type != 'np_thru_hole' and len(pcb.nets[p.net_id].pads) > 1]
    return out


def _fixed_nets(pcb, path, bus):
    """the nets of `pcb` (the board at `path`) whose copper no later step moves: the bus's, the protected, a pour's"""
    from protected_nets import read_protected_nets
    prot = set(read_protected_nets(os.path.splitext(path)[0] + '.kicad_pro'))
    return {i for i, nn in pcb.nets.items() if nn.name in set(bus) | prot} | \
        {z.net_id for z in (pcb.zones or []) if z.net_id}


def _caged_on(pcb, keys, fixed, rung, near=None):
    """['REF.PAD'] of `keys` ({ref: [pad number]}) caged on `pcb` with `fixed`'s copper the walls (`near`: {ref: the
    board _cropped round it}, reused)"""
    from kicad_parser import is_immutable_copper
    out = []
    for r in keys:
        foot = pcb.footprints.get(r)
        if foot is None:
            continue
        crop = near[r] if near and r in near else _cropped(pcb, foot)
        by_num = {p.pad_number: p for p in foot.pads}
        for n in keys[r]:
            if n in by_num and _caged(_walls(crop, fixed, by_num[n].net_id, is_immutable_copper), by_num[n], *rung):
                out.append(f'{r}.{n}')
    return out


def _walls(crop, fixed, own, immutable):
    """`crop` with only the ball's own copper and the walls: the rest of the copper the route step may take up"""
    walls = copy.copy(crop)
    walls.segments = [s for s in crop.segments if s.net_id in fixed or s.net_id == own or immutable(s)]
    walls.vias = [v for v in crop.vias if v.net_id in fixed or v.net_id == own or immutable(v)]
    return walls


WALL_NEAR = 1.6         # mm from a ball: copper this near is a candidate wall of it (two pitches of a 0.8 mm array)


def caps_walling(before, after, refs, bus, moved):
    """(caps, balls): the passives of `moved` (references; their poses all that differs from `before` to `after`) whose
    move walled a ball of `refs` in -- walled_grade's test -- each such ball's nearest moved passive in its view whose
    pose put back frees it, else every moved one in its view; and the balls"""
    from kicad_parser import parse_kicad_pcb
    g = walled_grade(before, after, refs, bus, log=lambda *a: None)
    if not g['walled']:
        return [], []
    with contextlib.redirect_stdout(io.StringIO()):
        b_pre, b_post = parse_kicad_pcb(before), parse_kicad_pcb(after)
    fixed, rung = _fixed_nets(b_post, after, bus), _cage_rung(b_post)
    out = set()
    for k in g['walled']:
        r, n = k.split('.', 1)
        pad = next(p for p in b_post.footprints[r].pads if p.pad_number == n)
        dist = lambda c_: min(math.hypot(q.global_x - pad.global_x, q.global_y - pad.global_y)  # noqa: E731
                              for q in b_post.footprints[c_].pads)
        near = sorted((c_ for c_ in moved if c_ in b_post.footprints and c_ in b_pre.footprints
                       and b_post.footprints[c_].pads and dist(c_) <= CAGE_VIEW), key=dist)
        hit = None
        for c_ in near:
            post_, pre_ = b_post.footprints[c_].pads, b_pre.footprints[c_].pads
            keep_ = [(q.global_x, q.global_y, q.size_x, q.size_y, q.rect_rotation) for q in post_]
            for q, q0 in zip(post_, pre_):
                q.global_x, q.global_y, q.size_x, q.size_y, q.rect_rotation = \
                    q0.global_x, q0.global_y, q0.size_x, q0.size_y, q0.rect_rotation
            free_ = not _caged_on(b_post, {r: [n]}, fixed, rung)
            for q, s_ in zip(post_, keep_):
                q.global_x, q.global_y, q.size_x, q.size_y, q.rect_rotation = s_
            if free_:
                hit = c_
                break
        out |= {hit} if hit else set(near)
    return sorted(out), list(g['walled'])


def plain_caps(board, refs, bus, lock, work, tag, log=print):
    """A PLAIN bus's cap step on `board` (in place): the passives beneath the arrays `refs` moved off the fanout's copper
    (place_fanout_clearance --beneath-only, `lock` held), each drawn toward its own balls -- and a move that WALLS IN a
    ball the board left free undone: no round plans the arrays' other balls again round where the passives land, as
    the whole-array fanout's does (fanout_from_plan --hold), so a cap drawn under the array onto another net's only
    via site walled that ball in for every later step. Each such passive is held where it stood and the step run
    again from the board before it; a passive that stood on copper (one the step must move) is never held. Moved
    nowhere at all, the passives stood in the bus's own way out: its clock pair's tooth refused, the fanout four times
    as long, and nets open"""
    import shutil
    import awx_settings
    import braid as te
    import fanout_from_plan as fp
    import whole_route as wr
    pre = os.path.join(work, f'{tag}_pre.kicad_pcb')
    shutil.copyfile(board, pre)
    fp.copy_pro(board, pre)
    held, logp = list(lock or ()), os.path.join(work, f'{tag}.log')
    for try_ in range(3):
        if try_:
            shutil.copyfile(pre, board)
            fp.copy_pro(pre, board)
        wr.run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), board, board,
                '--clearance', str(te.SPEC_CLEARANCE), '--beneath-only'] + (['--lock'] + held if held else []),
               awx_settings.environ(), log_path=logp, own_process=True)
        for ln in wr.grep(logp, r'^(Moved|Stuck|  Unresolved)'):
            log(ln.strip()[:200])
        on_copper = set()
        for ln in wr.grep(logp, r'initially grazing'):
            m_ = re.search(r'\(([^)]*)\)\s*$', ln.strip())
            on_copper |= {x.strip() for x in m_.group(1).split(',')} if m_ else set()
        before, after = wr.footprint_poses(pre), wr.footprint_poses(board)
        moved = [r_ for r_, p_ in after.items() if before.get(r_) != p_ and r_ not in on_copper and r_ not in held]
        if not moved:
            return
        caps, balls = caps_walling(pre, board, refs, bus, moved)
        if not caps:
            return
        held += caps
        log(f'{", ".join(caps)} moved onto the way out of {", ".join(balls)} -- held where '
            f'{"it" if len(caps) == 1 else "they"} stood, the step again')
    shutil.copyfile(pre, board)
    fp.copy_pro(pre, board)
    wr.run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), board, board,
            '--clearance', str(te.SPEC_CLEARANCE), '--beneath-only'] + (['--lock'] + held if held else []),
           awx_settings.environ(), log_path=logp, own_process=True)


def walls_made(before, after, refs, bus, moved, project=None, only=None):
    """{'REF.PAD': [net]} the balls of `refs` on no bus net (`bus`, full names) that the board `after` walls in and
    `before` did not -- walled_grade's test at its finest rung -- each with the nets of `moved` (full names, whose
    copper is all that differs) that wall it: the nearest whose copper taken away alone frees the ball, else the fewest
    nearest first; [] when taking all of theirs away leaves it walled. Asked of the balls within WALL_NEAR of a moved
    net's copper only. A board is a path, or a PCBData with `project` the board path its protected nets are read
    beside. `only` ({net full name: ids of `after`'s copper}): the copper a remedy can take away -- a lane's, its
    fanout standing (route_lanes.lift_walls) -- and only that taken away: by the whole net, a stub of a walling lane's
    fanout under the ball named that lane, and lifting the lane freed nothing"""
    from kicad_parser import parse_kicad_pcb, is_immutable_copper
    with contextlib.redirect_stdout(io.StringIO()):
        b_out, b_in = (parse_kicad_pcb(b) if isinstance(b, str) else b for b in (after, before))
    after, before = (b if isinstance(b, str) else project for b in (after, before))
    rung = _cage_rung(b_out)
    mv = {i for i, nn in b_out.nets.items() if nn.name in set(moved)}
    segs = [s for s in b_out.segments if s.net_id in mv]
    vias = [v for v in b_out.vias if v.net_id in mv]

    def dist(q, s):
        ax, ay, dx, dy = s.start_x, s.start_y, s.end_x - s.start_x, s.end_y - s.start_y
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((q[0] - ax) * dx + (q[1] - ay) * dy) / L))
        return math.hypot(q[0] - ax - t * dx, q[1] - ay - t * dy)

    own_net = collections.defaultdict(list)
    for s in b_out.segments:
        own_net[s.net_id].append(s)

    def way_out(p):
        """the places ball `p` leaves by: its pad, and the ends of the copper that runs on from it -- its escape's
        stub, piece by piece (its via stands on one of them) -- within the cage's view of it. What walls a ball in
        stands across the END of its way out: a ball in an array's second row leaves by a stub two pitches long,
        lanes across that end were past WALL_NEAR of the pad, the ball was not asked about, and it shipped walled
        in"""
        pts, r0 = [(p.global_x, p.global_y)], max(p.size_x, p.size_y) / 2
        left = [s for s in own_net.get(p.net_id, ()) if p.net_id
                and max(abs(s.start_x - p.global_x), abs(s.start_y - p.global_y),
                        abs(s.end_x - p.global_x), abs(s.end_y - p.global_y)) < CAGE_VIEW]
        grew = True
        while grew and left:
            grew = False
            for s in list(left):
                ends = ((s.start_x, s.start_y), (s.end_x, s.end_y))
                if any(math.hypot(q[0] - e[0], q[1] - e[1]) <= (r0 if i == 0 else 1e-3)
                       for i, q in enumerate(pts) for e in ends):
                    pts += ends
                    left.remove(s)
                    grew = True
        return pts

    def near_nets(p):
        """{net id: distance} of the moved nets' copper within WALL_NEAR of ball `p`'s way out (way_out)"""
        d = {}
        for q in way_out(p):
            for s in segs:
                if abs(s.start_x - q[0]) < WALL_NEAR + abs(s.end_x - s.start_x) and \
                        abs(s.start_y - q[1]) < WALL_NEAR + abs(s.end_y - s.start_y):
                    x = dist(q, s)
                    if x < WALL_NEAR and x < d.get(s.net_id, 9e9):
                        d[s.net_id] = x
            for v in vias:
                x = math.hypot(v.x - q[0], v.y - q[1])
                if x < WALL_NEAR and x < d.get(v.net_id, 9e9):
                    d[v.net_id] = x
        return {k: x for k, x in d.items() if k != p.net_id}
    fixed = _fixed_nets(b_out, after, bus)
    cand = {r: {p.pad_number: near_nets(p) for p in pads} for r, pads in _cage_balls(b_out, refs, bus).items()}
    cand = {r: {n: d for n, d in v.items() if d} for r, v in cand.items()}
    crops = {r: _cropped(b_out, b_out.footprints[r]) for r in cand if cand[r]}
    now = _caged_on(b_out, {r: list(v) for r, v in cand.items() if v}, fixed, rung, crops)
    if not now:
        return {}
    was = set(_caged_on(b_in, {r: [k.split('.', 1)[1] for k in now if k.split('.', 1)[0] == r] for r in crops},
                        _fixed_nets(b_in, before, bus), rung))
    out = {}
    for k in now:
        if k in was:
            continue
        r, n = k.split('.', 1)
        pad = next(p for p in b_out.footprints[r].pads if p.pad_number == n)
        order = sorted(cand[r][n], key=cand[r][n].get)

        def frees(drop):
            if only is None:
                return not _caged(_walls(crops[r], fixed - set(drop), pad.net_id, is_immutable_copper), pad, *rung)
            ids_ = set().union(*(only.get(b_out.nets[i].name, ()) for i in drop))
            c_ = copy.copy(crops[r])
            c_.segments = [s for s in c_.segments if id(s) not in ids_]
            c_.vias = [v for v in c_.vias if id(v) not in ids_]
            return not _caged(_walls(c_, fixed, pad.net_id, is_immutable_copper), pad, *rung)
        who = next(([i] for i in order if frees([i])), None)
        if who is None:
            who = next((order[:j] for j in range(2, len(order) + 1) if frees(order[:j])), [])
        out[k] = [b_out.nets[i].name for i in who]
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
    """no track `track` wide gets from `pad` to the rest of its net within CAGE_VIEW of it, nor to the view's edge --
    changing layers where a via `via` wide fits on every layer or at a via of the net's own. check_reachability's
    question (placement.reachability: its slack field, its own-copper islands) at one width: each layer's room
    thresholded and labelled, the layers joined where a via goes, not a widest path. Two answers differ from its: the
    view's edge is always a way out (it takes the edge only when the view holds none of the net), and the net's own
    vias join layers here as its copper does -- check_reachability joins them only where a new via of its --via fits,
    and read main's imx6ulz U1.N4, escaped through its own 0.3 mm via, CAGED. Pours are not read: a foreign one is
    filled round what is laid, and a ball's own counts as no copper to reach"""
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
    # and the view's EDGE, always: room that leaves the view left any cage round the ball, and the rest of the net
    # past it is out of this check's sight (kintex_pcie U1.B3: NT7's pad, the one piece of MGTAVTT in its view, walled
    # off by the lanes, while its way between the balls to G2's island and that island's stub to the face crossed
    # the view's edge)
    olabs, ofind, on = joined(owns, own_via, False)
    sy, sx = int((pad.global_y - view[1]) / step), int((pad.global_x - view[0]) / step)
    if not olabs[0][sy, sx]:
        return False                        # (no copper of its own at its centre: not this check's to say)
    sroot = ofind(int(olabs[0][sy, sx]))
    roots = np.array([ofind(i) for i in range(on + 1)])

    def joined_pad(q):
        """pad `q` of the net in the ball's own island, on a layer it stands on"""
        qx, qy = int((q.global_x - view[0]) / step), int((q.global_y - view[1]) / step)
        if not (0 <= qx < w and 0 <= qy < h):
            return False
        thru = '*.Cu' in q.layers or (q.drill or 0) > 0
        return any(ol[qy, qx] and roots[ol[qy, qx]] == sroot
                   for ol, L in zip(olabs, layers) if thru or L in q.layers)
    # (a net WHOLE already -- every pad of it on the board in the ball's own island -- has nothing to reach: not caged,
    # whatever seals the view. Its only target the view's edge, a ball wired to its pads by an earlier step read caged
    # once lanes closed round it, and its walls were lifted for nothing)
    net_ = pcb.nets.get(nid)
    if net_ is not None and len(net_.pads) > 1 and all(joined_pad(q) for q in net_.pads):
        return False
    edge = np.zeros((h, w), dtype=bool)
    edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
    tgt = [(o & (roots[ol] != sroot)) | edge for o, ol in zip(owns, olabs)]
    flabs, ffind, _n = joined(frees, own_via, True)
    if not flabs[0][sy, sx]:
        return True
    root = ffind(int(flabs[0][sy, sx]))
    return not any(ffind(int(i)) == root for lab, t in zip(flabs, tgt) for i in np.unique(lab[t & (lab > 0)]))


def ball_way(pcb, pad, fixed, rung, with_path=False, sites=1):
    """((x, y), layer): the via site `pad` escapes by on `pcb`, `fixed`'s copper the walls -- the nearest point its own
    layer reaches with a track of `rung` where a via of `rung` fits on every layer, one off the pad's copper first (a
    site in the pad by a millimetre's handicap); None when there is none in the cage's view. _caged's raster.
    `with_path`: ((x, y), layer, [points]) -- and the way there on its layer, from the ball to the site, its turns.
    `sites` (with `with_path`) over one: a LIST of up to that many such ways, the nearest first, each site a via's
    width or more from every one before it"""
    import numpy as np
    from scipy import ndimage
    from kicad_parser import is_immutable_copper
    if os.path.join(HERE, '..', 'py_placer') not in sys.path:
        sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
    from placement.reachability import slack_field
    track, via, clearance = rung
    walls = _walls(_cropped(pcb, pcb.footprints[pad.component_ref]), fixed, pad.net_id, is_immutable_copper)
    step = CAGE_STEP
    view = (pad.global_x - CAGE_VIEW, pad.global_y - CAGE_VIEW, pad.global_x + CAGE_VIEW, pad.global_y + CAGE_VIEW)
    layers = list(pcb.board_info.copper_layers)
    home = next((L for L in layers if L in pad.layers), layers[0])
    free, via_ok = None, None
    for L in [home] + [L for L in layers if L != home]:
        s, _o, D = slack_field(walls, pad.net_id, L, view, clearance, None, step)
        free = s >= track if free is None else free
        via_ok = (D >= via / 2.0) if via_ok is None else via_ok & (D >= via / 2.0)
    sy, sx = int((pad.global_y - view[1]) / step), int((pad.global_x - view[0]) / step)
    lab, _n = ndimage.label(free)
    if not lab[sy, sx]:
        return None
    ys, xs = np.nonzero((lab == lab[sy, sx]) & via_ok)
    if not len(xs):
        return None
    px, py = view[0] + (xs + 0.5) * step, view[1] + (ys + 0.5) * step
    import pairs as _pairs
    in_pad = _pairs.in_pad(px, py, pad)
    # (the nearest site along the WAY there -- the free cells' walk from the ball, an 8-way breadth-first search --
    # not across: the nearest across lay back inside the array round the end of a fanout's run, 1.2 mm from the ball
    # and a 3 mm walk, where one outside the array's face stood along the way)
    import collections as _col
    reg = lab == lab[sy, sx]
    H_, W_ = reg.shape
    prev = {(sy, sx): None}
    dist = {(sy, sx): 0}
    q = _col.deque([(sy, sx)])
    while q:
        c_ = q.popleft()
        for dy_, dx_ in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            n_ = (c_[0] + dy_, c_[1] + dx_)
            if 0 <= n_[0] < H_ and 0 <= n_[1] < W_ and reg[n_] and n_ not in prev:
                prev[n_] = c_
                dist[n_] = dist[c_] + 1
                q.append(n_)
    walk = np.array([dist.get((int(y_), int(x_)), 10 ** 9) for y_, x_ in zip(ys, xs)], float) * step
    cost = walk + in_pad * 1.0
    i = int(np.argmin(cost))
    if not with_path:
        return (float(px[i]), float(py[i])), home

    def way_to(i):
        # (the way itself, kept at its turns -- the site may lie round the end of a fanout's run the straight line
        # from the ball crosses)
        goal = (int(ys[i]), int(xs[i]))
        cells, c_ = [], goal
        while c_ is not None and c_ in prev:
            cells.append(c_)
            c_ = prev[c_]
        cells.reverse()
        pts = [(pad.global_x, pad.global_y)]
        for k_ in range(1, len(cells) - 1):
            d0 = (cells[k_][0] - cells[k_ - 1][0], cells[k_][1] - cells[k_ - 1][1])
            d1 = (cells[k_ + 1][0] - cells[k_][0], cells[k_ + 1][1] - cells[k_][1])
            if d0 != d1:
                pts.append((float(view[0] + (cells[k_][1] + 0.5) * step),
                            float(view[1] + (cells[k_][0] + 0.5) * step)))
        pts.append((float(px[i]), float(py[i])))
        return (float(px[i]), float(py[i])), home, pts
    if sites <= 1:
        return way_to(i)
    out, taken = [], []
    for j in np.argsort(cost, kind='stable'):
        if cost[j] >= 10 ** 8 or len(out) >= sites:
            break
        if all(math.hypot(px[j] - x_, py[j] - y_) >= via for x_, y_ in taken):
            taken.append((float(px[j]), float(py[j])))
            out.append(way_to(int(j)))
    return out


CHANNEL_SHORT = 100.0   # (ball_channel) a cell of the way with no room for a track as laid, by what it is short
CHANNEL_VIA = 1.0       # (ball_channel) a change of layer on the way, in mm of it


def ball_channel(pcb, pad, fixed, rung, soft):
    """THE WAY OUT OF ball `pad` BETWEEN THE LANES, on `pcb` with `fixed`'s copper the walls: from the ball's own copper
    to what the cage test asks it to reach (_caged: the rest of its net in its view, else the view's edge), over the
    cells a track of `rung` fits with the copper of `soft` gone ({id of a track or via of `pcb`: its lane} -- what a
    remedy can lay again) and never across a piece of that copper: two lanes PART round a way between them and keep
    their shape, a lane crossed has to go round the way's end. Of such ways the one through the least room that has to
    be made -- a cell with no room for the track as laid is dearer by what it is short (CHANNEL_SHORT), the ball's own
    copper next to free -- so it follows its stub to the end and goes on between the two lanes that close past it.
    (Room as laid is a track's and a raster cell to spare: a lane is laid a hair over the clearance, and one that
    stands the bare clearance off the way is routed again all the same.)
    Layers change where a via of `rung` fits on every layer as laid (CHANNEL_VIA) or at a via of the net's own.
    _caged's raster and its targets. A walled ball's via site alone, reserved, kept nothing open where the wall stood
    past it: a ball of an array's second row, its stub out between two lanes that ran on side by side a track's width
    short of a track's room.
    {'lines': [(p, q, layer)] the way from the end of the ball's own copper on, 'vias': [(x, y)] its changes of layer,
    'lanes': the lanes of `soft` a track along it is short of its clearance to, the nearest the ball first, 'short':
    the mm of it with no room as laid, 'end': 'net' or 'edge'}; None where there is no such way"""
    import heapq
    import numpy as np
    from kicad_parser import is_immutable_copper
    if os.path.join(HERE, '..', 'py_placer') not in sys.path:
        sys.path.insert(0, os.path.join(HERE, '..', 'py_placer'))
    from placement.reachability import slack_field
    track, via, clearance = rung
    walls = _walls(_cropped(pcb, pcb.footprints[pad.component_ref]), fixed, pad.net_id, is_immutable_copper)
    hard, lanes = copy.copy(walls), copy.copy(walls)
    hard.segments = [s for s in walls.segments if id(s) not in soft]
    hard.vias = [v for v in walls.vias if id(v) not in soft]
    lanes.segments = [s for s in walls.segments if id(s) in soft]
    lanes.vias = [v for v in walls.vias if id(v) in soft]
    lanes.footprints = {}
    if not lanes.segments and not lanes.vias:
        return None
    nid, step = pad.net_id, CAGE_STEP
    view = (pad.global_x - CAGE_VIEW, pad.global_y - CAGE_VIEW, pad.global_x + CAGE_VIEW, pad.global_y + CAGE_VIEW)
    layers = list(pcb.board_info.copper_layers)
    home = next((L for L in layers if L in pad.layers), layers[0])
    layers = [home] + [L for L in layers if L != home]
    need = track / 2.0 + step
    ok, price, own, room = [], [], [], []
    for L in layers:
        _s, o, D = slack_field(walls, nid, L, view, clearance, None, step)
        Dh = slack_field(hard, nid, L, view, clearance, None, step)[2]
        Dl = slack_field(lanes, nid, L, view, clearance, None, step)[2]
        ok.append(((Dh >= track / 2.0) | o) & (Dl > -0.5))      # (slack_field's own mark of a cell on copper: -1)
        price.append(np.where(o, 0.01, 1.0 + CHANNEL_SHORT * np.clip((need - D) / (need + clearance), 0.0, 1.0)))
        own.append(o)
        room.append(D)
    h, w = ok[0].shape
    via_ok = np.logical_and.reduce([D >= via / 2.0 for D in room])
    own_via = set()
    for v in walls.vias:
        cx, cy = int((v.x - view[0]) / step), int((v.y - view[1]) / step)
        if v.net_id == nid and 0 <= cx < w and 0 <= cy < h:
            own_via.add((cy, cx))
    sy, sx = int((pad.global_y - view[1]) / step), int((pad.global_x - view[0]) / step)
    if not (0 <= sy < h and 0 <= sx < w) or not own[0][sy, sx]:
        return None
    # the ball's own island (its net's copper joined through its own vias), and the targets: the rest of its net, the
    # view's edge
    isl = [np.zeros((h, w), dtype=bool) for _L in layers]
    todo = [(0, sy, sx)]
    isl[0][sy, sx] = True
    while todo:
        li, y, x = todo.pop()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            y1, x1 = y + dy, x + dx
            if 0 <= y1 < h and 0 <= x1 < w and own[li][y1, x1] and not isl[li][y1, x1]:
                isl[li][y1, x1] = True
                todo.append((li, y1, x1))
        if (y, x) in own_via:
            for lj in range(len(layers)):
                if own[lj][y, x] and not isl[lj][y, x]:
                    isl[lj][y, x] = True
                    todo.append((lj, y, x))
    edge = np.zeros((h, w), dtype=bool)
    edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
    tgt = [(o & ~i_) | (edge & k_) for o, i_, k_ in zip(own, isl, ok)]
    dist, prev, heap, end = {(0, sy, sx): 0.0}, {(0, sy, sx): None}, [(0.0, 0, sy, sx)], None
    while heap:
        d, li, y, x = heapq.heappop(heap)
        if d > dist.get((li, y, x), 9e18):
            continue
        if tgt[li][y, x]:
            end = (li, y, x)
            break
        okL, prL = ok[li], price[li]
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            y1, x1 = y + dy, x + dx
            # (no step across a corner two walls meet at: the cage test's regions are joined by their sides)
            if not (0 <= y1 < h and 0 <= x1 < w) or not okL[y1, x1] or (dy and dx and not (okL[y, x1] and okL[y1, x])):
                continue
            d1 = d + step * (1.41421356 if dy and dx else 1.0) * prL[y1, x1]
            if d1 < dist.get((li, y1, x1), 9e18):
                dist[(li, y1, x1)], prev[(li, y1, x1)] = d1, (li, y, x)
                heapq.heappush(heap, (d1, li, y1, x1))
        hop = 0.0 if (y, x) in own_via else CHANNEL_VIA if via_ok[y, x] else None
        if hop is not None:
            for lj in range(len(layers)):
                if lj != li and ok[lj][y, x] and d + hop < dist.get((lj, y, x), 9e18):
                    dist[(lj, y, x)], prev[(lj, y, x)] = d + hop, (li, y, x)
                    heapq.heappush(heap, (d + hop, lj, y, x))
    if end is None:
        return None
    cells = []
    while end is not None:
        cells.append(end)
        end = prev[end]
    cells.reverse()
    # (from the end of the ball's own copper on: what stands on its own stub reserves nothing)
    k0 = max((i for i, (li, y, x) in enumerate(cells) if isl[li][y, x]), default=0)
    way = cells[k0:]
    at = lambda c: (float(view[0] + (c[2] + 0.5) * step), float(view[1] + (c[1] + 0.5) * step))   # noqa: E731

    def turns(run):
        """a run of cells on one layer as its points: the ends and each turn worth one -- a cell's step aside in a
        straight stretch is the raster's, and reserved as drawn it took room from the lane beside it (Douglas-Peucker,
        a cell and a half)"""
        pts = [at(c) for c in run]

        def keep(i, j):
            if j - i < 2:
                return []
            (ax, ay), (bx, by) = pts[i], pts[j]
            L = math.hypot(bx - ax, by - ay) or 1e-12
            d = [abs((bx - ax) * (ay - y) - (ax - x) * (by - ay)) / L for x, y in pts[i + 1:j]]
            k = max(range(len(d)), key=d.__getitem__)
            return [] if d[k] <= 1.5 * step else keep(i, i + 1 + k) + [i + 1 + k] + keep(i + 1 + k, j)
        return [pts[i] for i in [0] + keep(0, len(pts) - 1) + ([len(pts) - 1] if len(pts) > 1 else [])]
    lines, vias_, run = [], [], [way[0]]
    for c in way[1:] + [None]:
        if c is not None and c[0] == run[-1][0]:
            run.append(c)
            continue
        pts = turns(run)
        lines += [(a, b, layers[run[0][0]]) for a, b in zip(pts, pts[1:])
                  if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-9]
        if c is not None:
            if (run[-1][1], run[-1][2]) not in own_via:
                vias_.append(at(run[-1]))
            run = [c]
    # the lanes it is short of its clearance to: their copper within a track's room of a cell of it with none as laid
    short = [c for c in way if not own[c[0]][c[1], c[2]] and room[c[0]][c[1], c[2]] < need]
    first = {}
    for i, c in enumerate(short):
        px, py = at(c)
        for s in lanes.segments:
            if s.layer != layers[c[0]] or soft[id(s)] in first:
                continue
            ax, ay, dx, dy = s.start_x, s.start_y, s.end_x - s.start_x, s.end_y - s.start_y
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            if math.hypot(px - ax - t * dx, py - ay - t * dy) < need + clearance + s.width / 2.0:
                first[soft[id(s)]] = i
        for v in lanes.vias:
            if soft[id(v)] not in first and \
                    math.hypot(px - v.x, py - v.y) < need + clearance + v.size / 2.0:
                first[soft[id(v)]] = i
    last = cells[-1]
    return dict(lines=lines, vias=vias_, lanes=sorted(first, key=lambda m: (first[m], str(m))),
                short=round(len(short) * step, 3), end='net' if own[last[0]][last[1], last[2]] else 'edge')


def changed_nets(board, routed, names):
    """the nets of `names` (the whole route's names, coherent_nets.net_keys) whose tracks or vias on `routed` are not
    the ones `board` has: copper the step laid"""
    from coherent_nets import key_of, net_keys
    if not names:
        return set()
    src, now = open(board, encoding='utf-8').read(), open(routed, encoding='utf-8').read()

    def items(txt):
        out = collections.defaultdict(set)
        pcb_ = parse_kicad_pcb_quiet_text(txt)
        keys = net_keys(pcb_)
        nm = {i: key_of(n.name, keys) for i, n in pcb_.nets.items()}
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
    """the full names of the nets `shorts` (the whole route's names, coherent_nets.net_keys)"""
    from coherent_nets import net_keys
    keys = net_keys(pcb)
    return {n.name for n in pcb.nets.values() if n.name and keys[n.name] in set(shorts)}


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
    `path` holds now: the step may have laid the board round it anew (an array fanned in one plan with its buses: an
    open bus net's earlier escape put back over the field's vias was ten violations on zynq U1, and every net the grade
    then put back met another). A block that would not stand is left out -- the net to the
    rest of the chain. Returns the blocks left out"""
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
        m_nm = block_net_name(b)
        nid = int(m_id.group(1)) if m_id else by_name.get(m_nm if m_nm is not None else '', 0)
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
        m2 = block_net_name(block)
        if (m and int(m.group(1)) in net_ids) or (m2 is not None and m2 in net_names):
            out.append(block)
        i = k + 1


def _rules_dict(r):
    return {k: getattr(r, k) for k in ('clearance', 'track', 'fan_track', 'via_size', 'via_drill', 'hole_to_hole',
                                       'edge_clearance', 'source')}


# a bus: two parts sharing at least this many nets the whole route admits (make_bench.pair_nets)
MIN_BUS_NETS = 8
#: a part joins a face's group bus with this many nets or more, when a part of the group has MIN_BUS_NETS (bus_plan)
MIN_GROUP_NETS = MIN_BUS_NETS // 2
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
    larger the source; otherwise named with why not (a row part at one end is the router's, unless --bus names the
    bus: escape_moves.row_part). `left`: the nets on both
    parts the whole route does not admit, each with why -- its other parts (fly-by, multi-drop, a pull-down), or more
    than one pad on a part -- the router's."""
    with step_policy():                 # (make_bench imports the chain's modules: under its policy, step_policy)
        import make_bench as mb
    from collections import Counter
    from coherent_nets import net_keys
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
            f'({src}:{dest})'
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
        keys = net_keys(pcb)
        out.append(dict(src=src, dest=dest, nets=sorted(keys.get(n_, n_) for n_ in names), taken=why is None,
                        why=why, left=dict(sorted(left.items()))))
    out.sort(key=lambda d: (-d['taken'], -len(d['nets']), d['src'], d['dest']))
    return out


def half_pairs(pcb, names):
    """{leg: why} for the pair legs among `names` whose partner is not among them (pairs.pair_names over the board's
    nets, by the whole route's names: coherent_nets.net_keys): a pair is laid whole or not at all, and a partner on a
    third part as well is no lane (the zynq's FB_CLK_N, pulled to RFGND by RX22)"""
    import pairs as _pairs
    from coherent_nets import net_keys
    keys = net_keys(pcb)
    short = {keys.get(n, n) for n in names}
    out = {}
    by = {keys[n.name]: n for n in pcb.nets.values() if n.name}
    for b_, (pn, nn) in _pairs.pair_names(sorted(by), admit_all=True).items():
        if b_ in by:                         # (a base that is a net of its own is no pair: pairs.members)
            continue
        for leg, mate in ((pn, nn), (nn, pn)):
            if leg in short and mate not in short:
                parts = sorted({p.component_ref for p in by[mate].pads})
                out[leg] = f'its pair\'s other leg {mate} is no lane between the arrays (on {", ".join(parts)})'
    return out


def clear_row_escapes(board, ref, names, keep, work, log=print, zone=None):
    """The movable passives beside a ROW PART `ref` (escape_moves.row_part: a QFN) moved off the bus's escapes there,
    on `board` in place: each of `names`' pads on it has one escape, straight out of its row, and a decoupling cap
    standing across it leaves its lane no end at all. The escapes are laid on a copy as the plan models them -- the
    lane track grown by the braid's clearance past the rule's, as the cap step clears copper at the rule (its
    --clearance a ceiling), and the via the plan may stand at its end (escape_moves: a row pad's stub and a via at its
    end, the lane leaving it on any layer) --, the cap step moves the passives near `ref` off them
    (place_fanout_clearance --near, the parts `keep` locked: a termination's, a pull's) and the poses it leaves are put
    on `board`. `zone` (grid, direction) -> (stagger, out) or None: a fine-pitch part's FAN-OUT ZONE (group_ends.
    row_zone) -- the stub laid `out` mm farther, a via at each of its rows -- cleared as the stub and its via are.
    Returns the moved refs"""
    import braid as te
    import escape_moves as em
    import route_layers
    import whole_route as wr
    import fanout_from_plan as fp
    import awx_settings
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    with contextlib.redirect_stdout(sys.stderr):
        pcb = parse_kicad_pcb(board)
    foot = pcb.footprints.get(ref)
    if foot is None or not em.row_part(foot):
        return []
    grid = em.grid_of(foot)
    want = set(names)
    layers = route_layers.escape_layers(pcb.board_info.copper_layers)
    width = te.TRACK + 2 * max(0.0, te.CLEAR - te.SPEC_CLEARANCE)
    tracks, vias = [], []
    for p in foot.pads:
        if p.net_name in want:
            ms = em.enumerate_moves(p, grid, layers, lambda a, b, L: True, lambda q, L: True)
            z_ = zone(grid, ms[0].direction) if zone is not None and ms else None
            for m in ms[:1]:
                tracks += [{'start': tuple(a), 'end': tuple(b), 'width': width, 'layer': L, 'net_id': p.net_id}
                           for a, b, L in m.legs]
                if z_ is not None:
                    dx_, dy_ = em.DIRS[m.direction]
                    e_ = tuple(m.exit_pt)
                    tracks.append({'start': e_, 'end': (e_[0] + dx_ * z_[1], e_[1] + dy_ * z_[1]), 'width': width,
                                   'layer': m.layer, 'net_id': p.net_id})
                    k_ = 1
                    while k_ * z_[0] < z_[1] - 1e-9:
                        vias.append({'x': e_[0] + dx_ * k_ * z_[0], 'y': e_[1] + dy_ * k_ * z_[0], 'size': te.VIA_SIZE,
                                     'drill': te.VIA_DRILL, 'layers': ['F.Cu', 'B.Cu'], 'net_id': p.net_id})
                        k_ += 1
            # (...and the via at its end: a decap 0.3 mm off a stub's end blocked a via end there -- a group's QFN's
            # caps straddling its edge, in the strip where its neighbour connector's pins escape)
            for m in [m_ for m_ in ms if m_.site is not None][:1]:
                vias.append({'x': m.site[0], 'y': m.site[1], 'size': te.VIA_SIZE, 'drill': te.VIA_DRILL,
                             'layers': ['F.Cu', 'B.Cu'], 'net_id': p.net_id})
    if not tracks:
        return []
    tmp = os.path.join(work, f'row_escapes_{ref}.kicad_pcb')
    with contextlib.redirect_stdout(sys.stderr):
        add_tracks_and_vias_to_pcb(board, tmp, tracks, vias, [], net_id_to_name={i: n.name for i, n in pcb.nets.items()})
    fp.copy_pro(board, tmp)
    moved_board = tmp[:-len('.kicad_pcb')] + '_capped.kicad_pcb'
    caps_log = os.path.join(work, f'caps_{ref}.log')
    rc = wr.run([os.path.join('..', 'py_router', 'place_fanout_clearance.py'), tmp, moved_board,
                 '--clearance', str(te.SPEC_CLEARANCE), '--near', ref] + (['--lock'] + list(keep) if keep else []),
                awx_settings.environ(), log_path=caps_log, own_process=True)[0]
    if rc != 0 or not os.path.isfile(moved_board):
        log(f'  {ref}: the cap step off its row escapes exited {rc} -- the passives stay where they are')
        return []
    before, after = wr.footprint_poses(board), wr.footprint_poses(moved_board)
    moved = {r_: p_ for r_, p_ in sorted(after.items()) if before.get(r_) != p_}
    if moved:
        got = wr.place_held(board, tmp[:-len('.kicad_pcb')] + '_held.kicad_pcb', moved)
        if got != board:
            import shutil
            shutil.copyfile(got, board)
        log(f'  {ref}: a row part -- {len(moved)} passive(s) beside it moved off its pads\' escapes: '
            + ', '.join(moved))
    for ln in wr.grep(caps_log, r'^(Stuck|  Unresolved)'):
        log(f'  {ref}: caps: ' + ln.strip()[:200])
    return sorted(moved)


def pair_gate_cuts(base, fanned, pcb, src, names, short=lambda n: n):
    """The nets of `names` (full) whose copper `fanned` has at `src` and `base` has not -- the escapes this step laid --
    crossing the GATE of a pair of `names` whose two legs' escapes `base` has there: the two legs joined, on their one
    layer, at every depth from the ball out (each leg's points in order along its copper from its ball). Returns the
    nets (full names)"""
    import pairs as _pairs
    import joint_escape as je
    with contextlib.redirect_stdout(io.StringIO()):
        pb, pf = parse_kicad_pcb_quiet(base), parse_kicad_pcb_quiet(fanned)
    fp_ = pb.footprints[src]
    xs, ys = [p.global_x for p in fp_.pads], [p.global_y for p in fp_.pads]
    box = (min(xs) - 1.5, min(ys) - 1.5, max(xs) + 1.5, max(ys) + 1.5)
    inside = lambda x, y: box[0] <= x <= box[2] and box[1] <= y <= box[3]   # noqa: E731
    full = {short(n): n for n in names}
    ids_b = {n.name: i for i, n in pb.nets.items() if n.name}
    ids_f = {n.name: i for i, n in pf.nets.items() if n.name}

    def chain(nid):
        """([points], layer) of net `nid`'s copper on `base` at `src` on its TOOTH's layer -- the layer of its copper
        farthest from its ball -- in order from the ball out"""
        ball = next((p for p in fp_.pads if p.net_id == nid), None)
        segs = [s for s in pb.segments if s.net_id == nid and inside(s.start_x, s.start_y) and inside(s.end_x, s.end_y)]
        if ball is None or not segs:
            return None
        d_ = lambda q: math.hypot(q[0] - ball.global_x, q[1] - ball.global_y)          # noqa: E731
        L = max(segs, key=lambda s: max(d_((s.start_x, s.start_y)), d_((s.end_x, s.end_y)))).layer
        pts = sorted({(round(x, 4), round(y, 4)) for s in segs if s.layer == L
                      for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))}, key=d_)
        return pts, L
    gates = []
    for _b, (pn, nn) in _pairs.pair_names(sorted(full)).items():
        cp, cn = (chain(ids_b.get(full[x], -1)) for x in (pn, nn))
        if cp and cn and cp[1] == cn[1]:
            gates += [(cp[1], a, b, {full[pn], full[nn]}) for a, b in zip(reversed(cp[0]), reversed(cn[0]))]
    if not gates:
        return []
    old = {(s.net_id, round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4), s.layer)
           for s in pb.segments}
    name_f = {i: n for n, i in ids_f.items()}
    out = []
    for s in pf.segments:
        nm = name_f.get(s.net_id)
        if nm not in set(names) or nm in out or \
                (ids_b.get(nm), round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4),
                 s.layer) in old:
            continue
        if any(L == s.layer and nm not in legs and je._cross((s.start_x, s.start_y), (s.end_x, s.end_y), a, b)
               for L, a, b, legs in gates):
            out.append(nm)
    return out


def comb_splits(fanned, pcb, src, dest, names, settings_env=None):
    """The pairs the comb on `fanned` left SPLIT AT THEIR TEETH -- another lane's tooth on the pair's layer between its
    two -- as the whole route's fanout audit reads them (fanout_from_plan.fanout_audit, whole_frame's PairSplit) on
    the board turned into the bus's flow frame: [(pair, [the lanes between])], short names, [] when none. The rounds'
    audit refuses every board on which one stands, and on a plain bus no round re-plans a pair nobody asks to move"""
    import fanout_from_plan as fp
    import flow_frame as ff
    import awx_settings
    with tempfile.TemporaryDirectory() as td:
        with contextlib.redirect_stdout(io.StringIO()):
            q, cx, cy = ff.turn_for_ends(parse_kicad_pcb_quiet(fanned), src, dest, set(names))
        frame = os.path.join(td, 'frame.kicad_pcb')
        if ff.turn_file(fanned, frame, q, cx, cy):
            return []
        fp.copy_pro(fanned, frame)
        with awx_settings.given({**awx_settings.environ(), **(settings_env or {}), 'BASE': frame}):
            splits, _far, _stops = fp.fanout_audit(frame, list(names), dest)
    return [(pr, list(ls)) for pr, end, ls in splits if end == 'tooth']


def repair_comb_splits(fanned, pcb, src, dest, names, short, run_rules, layers, work, whole_array, log=print,
                       settings_env=None):
    """THE COMB'S SPLIT PAIRS RE-PLANNED: the source comb (the chain's fanout engine, mb.fanout_source) fans ball by
    ball and keeps a pair's two escapes together only where its legs are neighbours on one row -- a DIAGONAL pair (one
    leg a row in from the other) came out with a deeper ball's escape between its legs, and the whole route's audit
    refused every round's board for it. Each pair split at its teeth (comb_splits), its two legs and the lanes standing
    between them, are stripped and planned again by the whole-array plan (joint_escape.fan_array: plan_array, a pair's
    legs held together, nothing between them) at the run's sizes, every other ball's copper as it stands. Kept only when
    it leaves fewer pairs split and no ball bare that had an escape; on a plain bus the passives beneath are moved off
    the new vias as for a refused ball. Returns the pairs repaired (short names)"""
    import shutil
    import awx_settings
    import braid as te
    import make_bench as mb
    import pairs as _pairs
    import joint_escape as je
    import fanout_from_plan as fp
    import flow_frame as ff
    # (the audit reads the bus's nets by the whole route's names, as its rounds do: full, a pair whose leg names carry a
    # sheet's path was never paired -- the setup looked its stand-in up by the path, and the step stopped)
    full = {short(n): n for n in names}
    splits = comb_splits(fanned, pcb, src, dest, sorted(full), settings_env)
    if not splits:
        return []
    legs = _pairs.pair_names(sorted(full))
    redo = []
    for pr, between in splits:
        redo += [full[x] for x in list(legs.get(pr, ())) + list(between) if x in full]
    redo = list(dict.fromkeys(redo))
    log(f'  {src}: the comb split {len(splits)} pair(s) at their teeth -- '
        + '; '.join(f'{pr} (round {", ".join(ls)})' for pr, ls in splits)
        + f' -- re-planned whole: {", ".join(short(n) for n in redo)}')
    stripped = os.path.join(work, 'fanned_pairs_in.kicad_pcb')
    txt, _n = mb.strip_pair_copper(open(fanned, encoding='utf-8').read(), pcb, redo)
    with open(stripped, 'w', encoding='utf-8') as f:
        f.write(txt)
    fp.copy_pro(fanned, stripped)
    dests = ff.dest_refs(dest)
    to_ = dest
    if len(dests) > 1:
        to_ = {}
        for d_ in dests:
            for q_ in pcb.footprints[d_].pads:
                if q_.net_name in set(redo):
                    to_.setdefault(short(q_.net_name), d_)
    far_ = je.far_face(pcb, src, dest) if isinstance(to_, str) else \
        {n_: je.far_face(pcb, src, d_) for n_, d_ in to_.items()}
    out = os.path.join(work, 'fanned_pairs.kicad_pcb')
    with contextlib.redirect_stdout(sys.stderr):
        _rr, reps_ = je.fan_array(stripped, out, src, redo, [], layers, drops=(), far=far_, log=lambda *a: None,
                                  rungs=[run_rules], exit_rays=True, escape_to=to_)
    bare = sorted({b_.split('#')[0] for b_ in reps_[-1]['bare']})
    after = comb_splits(out, pcb, src, dest, sorted(full), settings_env) if not bare else splits
    if bare or len(after) >= len(splits):
        log(f'  {src}: the re-plan kept out -- '
            + (f'balls left bare: {", ".join(short(n) for n in bare)}' if bare else
               f'still split: {"; ".join(pr for pr, _ls in after)}') + ' -- the comb as it was')
        return []
    shutil.copyfile(out, fanned)
    fp.copy_pro(out, fanned)
    if not whole_array:
        plain_caps(fanned, [src] + dests, names, je.passives_beyond(fanned, [src] + dests, names), work,
                   'caps_pairs', lambda s_: log(f'  {src}: caps: ' + s_))
    fixed = sorted({pr for pr, _ls in splits} - {pr for pr, _ls in after})
    log(f'  {src}: pairs re-planned together: {", ".join(fixed)}'
        + (f'; still split: {"; ".join(pr for pr, _ls in after)}' if after else ''))
    return fixed


def hold_board_escapes(board, base, out, pcb, src, failed, names=(), short=lambda n: n):
    """The refused balls `failed` (full names) of `src` that can HOLD the escape `board` has them at `src`
    (escape_blocks): a net's whole escape where every piece of it stands clear of the copper `base` holds
    (_clear_blocks), none of it otherwise. A refused leg of a pair of `names` holds its escape only with its
    partner's, the two together or neither: held alone, a memory bus's strobe leg kept the field's escape on an inner
    layer while the comb fanned its partner again on the surface, onto the held leg's tip, and the pair, its two tips
    one above the other, had no end connector in any round. `out`: `base` with those escapes laid (its project
    `base`'s). Returns the nets that hold one"""
    import pairs as _pairs
    full = {short(n): n for n in list(names) + list(failed)}
    mate = {}
    for _b, (pn, nn) in _pairs.pair_names(sorted(full)).items():
        mate[full[pn]], mate[full[nn]] = full[nn], full[pn]
    want = list(dict.fromkeys(list(failed) + [mate[n] for n in failed if n in mate]))
    blocks = escape_blocks(open(board, encoding='utf-8').read(), pcb, src, want)
    if not blocks:
        return []
    name_of = {i: n.name for i, n in pcb.nets.items() if n.name}

    def net_of(b):
        m_nm = block_net_name(b)
        m_id = re.search(r'\(net (\d+)\)', b)
        return m_nm if m_nm is not None else name_of.get(int(m_id.group(1))) if m_id else None
    by_net = collections.defaultdict(list)
    for b in blocks:
        by_net[net_of(b)].append(('via' if b.lstrip().startswith('(via') else 'segment', b))
    txt = open(base, encoding='utf-8').read()
    ok = {n for n in want if by_net.get(n) and len(_clear_blocks(base, txt, by_net[n])) == len(by_net[n])}
    held = [n for n in want if n in ok and (n not in mate or mate[n] in ok)]
    if held:
        end = txt.rstrip().rfind(')')
        with open(out, 'w', encoding='utf-8') as f:
            f.write(txt[:end].rstrip() + '\n' + ''.join(f'  {b}\n' for n in held for _t, b in by_net[n]) + txt[end:])
        if os.path.abspath(out) != os.path.abspath(base):
            import fanout_from_plan as fp
            fp.copy_pro(base, out)
    return held


def escape_blocks(txt, pcb, ref, names, grow=1.5):
    """The segment and via blocks of the nets `names` (full names) inside `ref`'s box of pads grown `grow` mm, in
    either net dialect: the escapes an earlier fanout laid them at that array"""
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
    """{net name: [its stub pads]} for the nets between `src` and `dest` that another pad keeps out of `names`
    (make_bench.pair_nets): one pad on each of the two parts, and any other pads -- a pull or a termination (a PHY's
    strap pins to their rails, a fly-by DDR3 bus's address and command nets terminated to its Vtt past the memory, the
    termination across a clock pair), a test point, another part's pin. A lane between the two parts is the net's run;
    each other pad a stub off it, the router's. Left to the router as they are: a rail (a plane's net, one that owns a
    zone, or a net on more parts than LEFT_PARTS) -- a termination voltage, a supply -- and a net with more than one pad
    on either part. Left to the router, a net with a test point between two arrays ran its way through the bus's
    corridor, and two such were left open there"""
    zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
    rails = zone_ids | {i for i, n in pcb.nets.items() if len({p.component_ref for p in n.pads}) > LEFT_PARTS}
    have = set(names)
    out = {}
    for net in pcb.nets.values():
        if not net.name or net.name in have or net.net_id in rails:
            continue
        on = collections.Counter(p.component_ref for p in net.pads)
        rest = [p for p in net.pads if p.component_ref not in (src, dest)]
        if on.get(src) == 1 and on.get(dest) == 1 and rest:
            out[net.name] = rest
    return out


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
    """THE STUBS THE STEP JOINS (on its routed board, after the run): each stub pad `held` took off its net (unpull) -- a pull's, a termination's, a test point's, another
    part's -- that a stub of ATTACH_PITCHES of an array of `refs` reaches from its net's dog-bone via there
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
        log(f'  stubs joined: none -- no stub pad within {ATTACH_PITCHES} pitches of {", ".join(sorted(grids))}; '
            f'{len(held)} left to the rest of the chain')
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
        # (the tracks' nets by the names of the board they were planned on: a name-net board's ids are the order its
        # nets first appear, and the board written has fewer pads put back than that one)
        with contextlib.redirect_stdout(sys.stderr):
            add_tracks_and_vias_to_pcb(path, path, tracks, [],
                                       net_id_to_name={i_: n_.name for i_, n_ in pcb.nets.items()})
    log(f'  stubs joined: {len(attached)} stub pad(s) joined to their lanes'
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
            m2 = block_net_name(block)
            at = block.find(before)
            if ((m and int(m.group(1)) in net_ids) or (m2 is not None and m2 in net_names)) and at > 0 \
                    and not re.search(r'\(locked[\s)]|^\(\w+\s+locked\b', block):
                block = block[:at] + '(locked yes) ' + block[at:]
                n += 1
            out.append(txt[i:j] + block)
            i = k
        txt = ''.join(out)
    return txt, n


def waypoint_pairs(pcb, names, src, dest):
    """{leg: [its pads on them]} for the pairs among `names` that pass through a termination part between the arrays
    (pairs.pair_waypoints: a two-pad part with one pad on each leg, neither array) -- each pad of it nothing serves
    yet, a pad under its own leg's ball or touched by its net's copper left on its net"""
    import pairs as _pairs
    from coherent_nets import by_key, net_keys
    by = {k: i for k, (i, _n) in by_key(pcb).items()}
    keys = net_keys(pcb)
    short = [keys.get(n, n) for n in names]
    out = {}
    for _b, (pn, nn) in _pairs.pair_names(short, admit_all=True).items():
        if pn in by and nn in by:
            for pp, qn in _pairs.pair_waypoints(pcb, by[pn], by[nn], src, dest, each=True):
                for leg_, pad_ in ((pn, pp), (nn, qn)):
                    if pad_ is not None:
                        out.setdefault(leg_, []).append(pad_)
    return out


def open_nets(conn_log, keys=None):
    """the nets check_connected's log finds open: unrouted, or in pieces (whole_route.open_nets), by the whole route's
    names (`keys`, coherent_nets.net_keys)"""
    import whole_route as wr
    return wr.open_nets(conn_log, keys)


def drc_nets(drc_log, keys=None):
    """the nets check_drc's log names in its violations, by the whole route's names (`keys`, coherent_nets.net_keys):
    both sides of a line between two items, and the net of a line naming one (the warnings after them are not read,
    nor the rows of the board's length rules, whole_route.LENGTH_ROWS, which the grade does not count)"""
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
            nm = drc_side_net(re.sub(r':\s*dist=.*$', '', side), keys)
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
    # (a library's crash -- a signal, no Python exception -- leaves the interpreter's own traceback in the log of the
    # stage it died in: the step's stages run in this process, and a solver's abort ended it with no line at all)
    import faulthandler
    faulthandler.enable()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board', metavar='BOARD.kicad_pcb')
    ap.add_argument('out', metavar='OUT.kicad_pcb', nargs='?', help='(omit with --find)')
    ap.add_argument('--src', help='with --dest: that one bus BY ITSELF, its own ends (a group\'s bus\'s own process; '
                                  'a debugging aid) -- without both, the buses bus_plan finds, each source\'s one group')
    ap.add_argument('--dest', help='the destination part (with --src)')
    ap.add_argument('--find', action='store_true',
                    help='print the buses the step would route, in order, and the ones it would not, each with why, '
                         'then the --bus line that names them -- at the sizes given -- and stop')
    ap.add_argument('--bus', action='append', metavar='SRC:DEST',
                    help='a bus to route, in the order given (repeat for each), instead of the buses bus_plan finds; '
                         'DEST may be a row part (a QFN), and may name several parts, U4+U3 (a group bus, one source '
                         'and several ends: one bus in one frame, its trunk to the first, a branch off it to each '
                         'other). The buses off one source are one group (route_buses)')
    import routing_defaults as defaults
    ap.add_argument('--k', type=int, help="the ladder's first K nets (default: the whole ladder)")
    ap.add_argument('--rounds', type=int, default=defaults.BUS_ROUNDS,
                    help='the whole route\'s rounds at most, each a draw under the next seed (default '
                         f'{defaults.BUS_ROUNDS})')
    ap.add_argument('--feedback-rounds', type=int, default=defaults.BUS_FEEDBACK_ROUNDS, metavar='F',
                    help='after each draw that leaves nets open, up to F of the rounds under its seed, the fanout '
                         'fed what the round before found (its crowded ends, its lanes over two vias, its open nets) '
                         'and choosing again from that round\'s board (default 0: every round a draw, but a round '
                         'whose solve had no plan is followed by a feedback round whatever F)')
    ap.add_argument('--round-edits', metavar='EDITS.json',
                    help='the ROUND EDITS: what the agent driving the step asks of its rounds after looking at the round '
                         'before, per bus and round -- {"SRC:DEST": {"2": {"ban": [{"lane": L, "end": 0}], "pin": '
                         '[{"lane": L, "end": 1, "at": [X, Y], "layer": LAYER}], "avoid": [{"lane": L, "end": 1}], '
                         '"pairs": [[{"lane": A, "end": 0}, {"lane": B, "end": 0}]], "cuts": [{"lane": L, "box": '
                         '[X0, Y0, X1, Y1], "layers": [..]}], "areas": [{"poly": [[X, Y], ..], "layers": [..], '
                         '"lanes": [..]}], "raw_cuts": {CUT FILE}, "flips": [[LANE, ISLAND]], "frame": "board"}}}: its '
                         'ends (0 the tooth, 1 the berth) banned (that place out of the lane\'s options), pinned, '
                         'avoided (priced) or not to be chosen together make it a feedback round from the round before '
                         '(--rounds must reach it); its cuts and areas keep lanes off layers there, its raw cuts and '
                         'flips seed its solve and loop. Points in the board\'s frame ("frame": "bench" the round\'s '
                         'bench). Recorded with the step, so a replay runs the same rounds with no one looking')
    ap.add_argument('--pack', action='store_true',
                    help='THE PACK, off by default (the setting BUS_PACK=1): after a round\'s lay each single lane is '
                         'pulled taut against its neighbour and laid again as packed octilinear copper (pack_lanes.py, '
                         'pack.py); the packed board is graded as the lay is and kept only where it has no more nets '
                         'open and no more violations, else the lay stands. A pair, and a lane the lay necked, stay '
                         'as laid')
    ap.add_argument('--inproc', action='store_true', help='every stage in this process rather than each in its own')
    ap.add_argument('--json-out', help='the summary: the nets routed and refused (and why), the rules, the grade '
                                     '(default: <OUT>.bus/summary.json). With --find: the buses found and not, as '
                                     'JSON -- {buses, left, step, arrays}: each bus {src, dest, nets, why}, the --bus '
                                     'arguments that name them, and the board\'s ball arrays')
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
    import fab_tiers as _ft
    g.add_argument('--escalation', choices=list(_ft.ESCALATION_POLICIES), default=None,
                   help='how far below the sizes a group of buses that leaves nets open (or violations, or a ball '
                        'walled in) is routed again, as route.py\'s --escalation (default '
                        f'{_ft.DEFAULT_ESCALATION}): \'fab\' down the fab ladder to its floor, \'board\' to the '
                        'board\'s own declared floors, \'off\' never. The group is routed again whole -- its ends, '
                        'the arrays\' fanout and every bus\'s lanes -- the lanes\' and the escapes\' tracks and the '
                        'vias a rung down together, the clearance never')
    a = ap.parse_args()
    # (a setting every bus's process reads -- the negotiated buses' too, each its own process: one with no flag of its
    # own runs under its parent's)
    if a.escalation:
        os.environ[ESCALATION_SETTING] = a.escalation
    if not os.path.isfile(a.board):
        print(f'route_bus: no board {a.board}', file=sys.stderr)
        sys.exit(2)
    sizes = {n: getattr(a, n) for n in SIZE_ARGS if getattr(a, n) is not None}
    if a.find:
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(sys.stderr):
            pcb = parse_kicad_pcb(a.board)
        with contextlib.redirect_stdout(sys.stderr):
            rules_ = resolve_rules(a.board, log=lambda *a_, **k_: print(*a_, file=sys.stderr), **sizes)
        plan = bus_plan(pcb, rules_)
        for ln in plan_lines(plan):
            print(ln)
        step = [f"{b['src']}:{b['dest']}" for b in plan['buses']]
        print('the step: ' + (' '.join(f'--bus {s_}' for s_ in step) or 'no bus -- OUT the board as it came'))
        if a.json_out:
            with open(a.json_out, 'w', encoding='utf-8') as f:
                json.dump(dict(plan, step=step, arrays=sorted(r for r, fp in pcb.footprints.items()
                                                              if len(fp.pads) > 8 and is_array(fp))), f, indent=1)
        sys.exit(0)
    if not a.out or (a.src is None) != (a.dest is None):
        ap.error('OUT is needed, and --src and --dest go together')
    if a.bus and a.src is not None:
        ap.error('--bus names the buses: no --src/--dest with it')
    # (the step in the board's redo manifest, #132: after --find, which routes nothing -- it prints the buses, and its
    # --json-out is that listing -- and never a --src/--dest run, which is a group's bus's own process, started by the
    # step it is part of, or a debugging aid)
    if a.src is None:
        from redo_record import record_invocation
        record_invocation()
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
    if a.round_edits:
        if not os.path.isfile(a.round_edits):
            print(f'route_bus: no round edits file {a.round_edits}', file=sys.stderr)
            sys.exit(2)
        # (a setting every bus's whole route reads, its own section by its SRC:DEST -- the negotiated buses' processes too)
        os.environ['ROUND_EDITS'] = os.path.abspath(a.round_edits)
    named = buses if buses is not None else ([(a.src, a.dest)] if a.src is not None else [])
    bad = bus_usage_errors(a.board, named) if named else []
    if bad:
        for why in bad:
            print(f'route_bus: {why}', file=sys.stderr)
        sys.exit(2)
    if a.src is None:
        sys.exit(cli_exit(route_buses(a.board, a.out, buses, a.k, a.rounds, a.inproc, json_out=a.json_out,
                                      feedback_rounds=a.feedback_rounds, pack=a.pack, **sizes)[0], a.out))
    sys.exit(cli_exit(route_bus(a.board, a.out, a.src, a.dest, a.k, a.rounds, a.inproc, json_out=a.json_out,
                                feedback_rounds=a.feedback_rounds, pack=a.pack, **sizes)[0], a.out))


if __name__ == '__main__':
    main()
