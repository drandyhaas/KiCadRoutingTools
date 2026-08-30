#!/usr/bin/env python3
"""Does the plan's order model (plan_order.BraidOrder) predict the order
the braid actually lays? Build the model the way fanout_from_plan does
-- from the BASE board, the destination not yet fanned out -- then feed
it the stub ends of a RECORDED fanout board as moves and compare its
launch and target orders, floor and columns with braid.Corridor's on
that board. usage: plan_model_check.py FANOUT_BOARD K [--base BOARD]"""
import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import detect_buses as db  # noqa: E402
import escape_moves as em  # noqa: E402
import plan_order as po  # noqa: E402
import topo_strings as ts  # noqa: E402
from schedule import Schedule  # noqa: E402

_DIRV = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}


def build_model(base, names, log=None):
    """The model as fanout_from_plan builds it (kept in step with it)."""
    pcb = parse_kicad_pcb(base)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ends = te.endpoints(pcb, names, byname)
    kids = {byname[n][0] for n in names}
    cache = {}

    def obs(nid, layer):
        if (nid, layer) not in cache:
            cache[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
        return cache[(nid, layer)]
    launch = {nm: ends[nm][0] for nm in names}
    paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
    dref = ends[names[0]][2]
    tooth0, srefs = {}, {}
    for nm in names:
        nid, net = byname[nm]
        tp = ends[nm][0]
        tooth0[nm] = next(
            (s.layer for s in pcb.segments if s.net_id == nid
             and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
                  or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1]) < 0.005)),
            'F.Cu')
        others = [p for p in net.pads if p.component_ref != dref]
        if others:
            srefs[others[0].component_ref] = srefs.get(others[0].component_ref, 0) + 1
    sref = max(srefs, key=srefs.get)
    dgrid = em.grid_of(pcb.footprints[dref])
    model = po.build_model(pcb, names, ends, byname, obs, paths, launch, tooth0,
                           dref, sref, dgrid.bbox, log=log)
    return model, pcb, byname, ends


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--base', default=os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
    ap.add_argument('--dest', default='DU1')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    # the braid on the recorded board
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    c = te.Corridor(0, groups[0], ctx, quiet)
    c.build_spine()
    c.classify()
    c.offsets(0.35)
    c.reserve_intervals()
    sched = Schedule(c.launch, c.target, ctx.tooth_layer,
                     dest_layer=ctx.dest_layer)
    cols, gate = c.plan_columns(sched, {d: 1 for d in sched.divers},
                                {d: 0 for d in sched.divers})
    print(f'braid: corridor 0 = {len(groups[0])} of {len(names)} nets, '
          f'{len(sched.divers)} divers (floor {2 * len(sched.divers)}), '
          f'{len(cols)} columns gate {gate}, L_free {c.L_free:.2f}')
    # the model from the base board, fed the recorded stubs as moves
    model, _pcb, _byname, _ends = build_model(a.base, names)
    grp = list(groups[0])
    sel = {}
    for nm in grp:
        stub = ctx.ends[nm][1]
        d = ctx.stub_dir[nm]
        direction = min(_DIRV, key=lambda k: (_DIRV[k][0] - d[0]) ** 2
                        + (_DIRV[k][1] - d[1]) ** 2)
        sel[nm] = em.Move(nm, 'surface', direction, ctx.dest_layer[nm], stub, 0)
    lo = model.order(grp, sel)
    tg = model.target_order(grp, sel)
    vias, fl, ncol, cap, n_gated = model.schedule(sel)
    # every net of the board, for the corridor count the braid made
    sel_all = {}
    for nm in names:
        stub = ctx.ends[nm][1]
        d = ctx.stub_dir[nm]
        direction = min(_DIRV, key=lambda k: (_DIRV[k][0] - d[0]) ** 2
                        + (_DIRV[k][1] - d[1]) ** 2)
        sel_all[nm] = em.Move(nm, 'surface', direction, ctx.dest_layer[nm], stub, 0)
    print(f'model: corridor vias {vias} (floor {fl}, exit-leg crossed '
          f'{len(model.leg_crossed(grp, sel))}), {n_gated} gated columns, capacity {cap}, '
          f'{ncol} as laid; corridors {model.corridors(sel_all)} (braid {len(groups)}) '
          f'(joiners {sorted(model.joiners)})')
    print(f'braid joiners: {sorted(c.joiners)}')

    def diff(tag, ours, theirs):
        if ours == theirs:
            print(f'  {tag}: IDENTICAL')
            return
        pos = {n: i for i, n in enumerate(theirs)}
        off = [(n, i, pos[n]) for i, n in enumerate(ours) if pos.get(n) != i]
        print(f'  {tag}: {len(off)} nets out of place:')
        print(f'    braid {theirs}')
        print(f'    model {ours}')
    diff('launch order', lo, c.launch)
    diff('target order', tg, c.target)
    kinds = {nm: k for nm, (k, _sg, _s, _o) in model.exit_kinds(grp, sel).items()}
    hb = {nm: ('head' if nm in c.heads_e else 'side') for nm in grp}
    bad = [nm for nm in grp if kinds[nm] != hb[nm]]
    print(f'  exit kinds: {len(bad)} differ' + (f': {bad}' if bad else ''))


if __name__ == '__main__':
    main()
