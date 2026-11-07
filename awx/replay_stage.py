#!/usr/bin/env python3
"""replay_stage.py WORK ROUND_DIR --sizes CLR,TRK,FTRK,VIA,DRILL SCRIPT [ARG ...] -- ONE stage of a bus's recorded
round run again on its own, under the environment that round's loop gave its stages: the bus step's rules at the run's
sizes (route_bus.resolve_rules on WORK's frame.kicad_pcb), route_bus.STEP_SETTINGS, whole_route.stage_env, the loop's
braid settings, BENCH the round's fanout bench (ROUND_DIR/fo.kicad_pcb) and NETS its nets.lines. `--set K=V` gives the
stage what the loop passed it (BENCH, HINT, CUTS, HIST, SOLVE_HOLD, CUTS_FIRM, SOLVE_UNPROVED, ...: read them off the
loop's files, OUTDIR/rN/loop) or a setting of its own.

What it is for: a stage's trouble reproduced in seconds, no round run -- a pair snap from the loop's recorded plan
(`whole_snap.py loop/p3.json OUT --pairs`), a repair re-solve from its recorded hold and cut files (`whole_solve.py
OUT --set SOLVE_HOLD=.. --set CUTS=..`), a geometry from a recorded solve. WORK is the bus's work dir (route_bus's
BUS.bus: frame.kicad_pcb, joint.json for a whole-array bus); ROUND_DIR a whole_route round in it (run/rN).
Exits as the stage does; its lines to stdout."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def round_env(work, round_dir, sizes, dest=None, layers=None, sets=()):
    """the environment a bus's round gives its loop's stages (as route_bus._route and whole_route.chain build it),
    over this process's: `sizes` the run's (clearance ceiling, track, fanout track, via, drill), `layers` its routing
    layers (default F.Cu, B.Cu and the frame's inner layers in stack order -- the order the run's cut files index),
    `sets` K=V given last"""
    import route_bus as rb
    import rules as _rules
    import whole_route as wr
    from kicad_parser import parse_kicad_pcb
    work, round_dir = os.path.abspath(work), os.path.abspath(round_dir)
    frame = os.path.join(work, 'frame.kicad_pcb')
    clr, trk, ftrk, vs, vd = (float(x) for x in sizes.split(','))
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        rr = rb.resolve_rules(frame, clearance_ceiling=clr, track_width=trk, fanout_track_width=ftrk, via_size=vs,
                              via_drill=vd, log=lambda *a: None)
        cu = parse_kicad_pcb(frame).board_info.copper_layers
    env = dict(os.environ)
    env.update(rb.STEP_SETTINGS)
    env[_rules.SETTING] = _rules.as_setting(rr)
    if os.path.isfile(os.path.join(work, 'joint.json')):
        env['FANOUT_JOINT'] = os.path.join(work, 'joint.json')
    env.update(BASE=frame, DEST=dest or env.get('DEST') or 'U1',
               ROUTE_LAYERS=layers or ','.join(['F.Cu', 'B.Cu'] + [L for L in cu if L not in ('F.Cu', 'B.Cu')]))
    wr.stage_env(env)
    env.update(BRAID_PAIRS='1', BRAID_EXACT_PAGES='0', PLAN_PAGES_SIDERS='2', STAGE_CACHE='0', TAUT_MEMO='1')
    nets = [ln.strip() for ln in open(os.path.join(round_dir, 'nets.lines')) if ln.strip()]
    env.update(BENCH=os.path.join(round_dir, 'fo.kicad_pcb'), NETS=','.join(nets))
    for kv in sets:
        k, _, v = kv.partition('=')
        # (a file named there, or each of a list, taken where this was run: the stages run in awx/)
        env[k] = ','.join(os.path.abspath(x) if x and os.path.exists(x) else x for x in v.split(','))
    return env


def add_round_args(ap):
    """the arguments every replay of a round takes"""
    ap.add_argument('work', help="the bus's work dir (route_bus's BUS.bus: frame.kicad_pcb)")
    ap.add_argument('round_dir', help='a whole_route round in it: run/rN (fo.kicad_pcb, nets.lines)')
    ap.add_argument('--sizes', required=True, metavar='CLR,TRK,FTRK,VIA,DRILL',
                    help="the run's --clearance-ceiling, --track-width, --fanout-track-width, --via-size, --via-drill")
    ap.add_argument('--dest', default=None, help='the destination part(s) (default: DEST, else U1)')
    ap.add_argument('--layers', default=None, help="the bus's routing layers (default F.Cu,B.Cu and the inner ones)")
    ap.add_argument('--set', action='append', default=[], metavar='K=V', help='a setting for the stage(s)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    add_round_args(ap)
    ap.add_argument('script', help='the awx stage: whole_solve.py, whole_snap.py, whole_geo.py, ...')
    ap.add_argument('args', nargs=argparse.REMAINDER, help="the stage's own arguments")
    a = ap.parse_args()
    env = round_env(a.work, a.round_dir, a.sizes, a.dest, a.layers, a.set)
    # (the stage runs in awx/: its arguments other than flags are files, taken where this was run)
    args = [x if x.startswith('-') else os.path.abspath(x) for x in a.args]
    sys.exit(subprocess.run([sys.executable, os.path.join(HERE, a.script)] + args, cwd=HERE, env=env).returncode)


if __name__ == '__main__':
    main()
