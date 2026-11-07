#!/usr/bin/env python3
"""replay_round.py WORK OUT_DIR --sizes CLR,TRK,FTRK,VIA,DRILL --k K [--fanout-from ROUND_DIR] -- ONE round of a bus's
whole route run again from its recorded work dir: whole_route.chain at one round under the settings route_bus gives
it (STEP_SETTINGS, the run's rules at its sizes, FANOUT_JOINT, the bus's ROUTE_LAYERS), in process as --inproc runs
it. With `--fanout-from` the round's FANOUT is the recorded round's (its fo.kicad_pcb with siblings, fo.plan.json and
fo.log copied in where fanout_from_plan.py would write them), so the rest of the round -- solve, loop, lay -- runs on
the very same ends.

What it is for: a change to the solve, the loop or the lay judged on the ends a recorded run laid, the fanout's own
spread taken out; and a round in a fraction of its time where the fanout is the slow part. WORK is the bus's work
dir (route_bus's BUS.bus: frame.kicad_pcb, joint.json for a whole-array bus). Its lines to stdout, its run dir
OUT_DIR/run; the last line the chain's grade."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('work', help="the bus's work dir (route_bus's BUS.bus: frame.kicad_pcb)")
    ap.add_argument('out_dir')
    ap.add_argument('--sizes', required=True, metavar='CLR,TRK,FTRK,VIA,DRILL',
                    help="the run's --clearance-ceiling, --track-width, --fanout-track-width, --via-size, --via-drill")
    ap.add_argument('--k', type=int, required=True, help="the bus's width (route_bus's K)")
    ap.add_argument('--dest', default=None, help='the destination part(s) (default: DEST, else U1)')
    ap.add_argument('--layers', default=None, help="the bus's routing layers (default F.Cu,B.Cu and the inner ones)")
    ap.add_argument('--fanout-from', default=None, metavar='ROUND_DIR', help="a recorded round whose fanout to take")
    ap.add_argument('--set', action='append', default=[], metavar='K=V', help='a setting for the round')
    a = ap.parse_args()
    work, out = os.path.abspath(a.work), os.path.abspath(a.out_dir)
    fo_from = os.path.abspath(a.fanout_from) if a.fanout_from else None
    os.chdir(HERE)
    import route_bus as rb
    import rules as _rules
    import whole_route as wr
    from kicad_parser import parse_kicad_pcb
    frame = os.path.join(work, 'frame.kicad_pcb')
    clr, trk, ftrk, vs, vd = (float(x) for x in a.sizes.split(','))
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        rr = rb.resolve_rules(frame, clearance_ceiling=clr, track_width=trk, fanout_track_width=ftrk, via_size=vs,
                              via_drill=vd, log=lambda *x: None)
        cu = parse_kicad_pcb(frame).board_info.copper_layers
    layers = a.layers or ','.join(['F.Cu', 'B.Cu'] + [L for L in cu if L not in ('F.Cu', 'B.Cu')])
    settings = {'ROUTE_LAYERS': layers, **rb.STEP_SETTINGS, _rules.SETTING: _rules.as_setting(rr)}
    if os.path.isfile(os.path.join(work, 'joint.json')):
        settings['FANOUT_JOINT'] = os.path.join(work, 'joint.json')
    settings.update(kv.partition('=')[::2] for kv in a.set)
    os.makedirs(out, exist_ok=True)
    wr.INPROC = True
    if fo_from:
        run0 = wr.run

        def run_(argv, env, *x, **k):
            # (the round's fanout -- not a held one, which lays the round's own plan -- taken from the recorded round)
            if argv and argv[0] == 'fanout_from_plan.py' and '--hold' not in argv:
                d_ = os.path.dirname(argv[1])
                for f_ in ('fo.kicad_pcb', 'fo.kicad_pro', 'fo.kicad_prl', 'fo.plan.json', 'fo.log'):
                    if os.path.isfile(os.path.join(fo_from, f_)):
                        shutil.copy2(os.path.join(fo_from, f_), os.path.join(d_, f_))
                if k.get('log_path') and os.path.isfile(os.path.join(fo_from, 'fo.log')):
                    shutil.copy2(os.path.join(fo_from, 'fo.log'), k['log_path'])
                print(f'replay_round: the fanout taken from {fo_from}', flush=True)
                return 0, ''
            return run0(argv, env, *x, **k)
        wr.run = run_
    with rb.step_policy(rr, settings):
        rc, grade = wr.chain(a.k, os.path.join(out, 'run'), 1, base=frame, dest=a.dest or os.environ.get('DEST') or 'U1',
                             F=0, settings=settings)
    print(f'replay_round: chain exit {rc}; {grade}')
    sys.exit(rc)


if __name__ == '__main__':
    main()
