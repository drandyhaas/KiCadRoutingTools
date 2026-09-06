#!/usr/bin/env python3
"""Fan out the destination array in the directions the PLAN chose.

The plan (plan_ends) picks, per ball, an escape direction, and hands
it to the production fanout engine (generate_bga_fanout) as
escape_dir_hints. A hint accepted is not a hint obeyed -- the engine
still has to find a free channel -- so the copper that actually leaves
each ball is measured against what was asked for (`obeyed`).

The source array arrives already fanned out (the bench). The plan's
source refinement is run for its effect on the destination choice
(the launch points later rounds select against) but its source moves
are never applied.

usage: fanout_from_plan.py OUT.kicad_pcb K --board=BASE.kicad_pcb
"""
import math
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
import detect_buses as db  # noqa: E402
import plan_ends as pe  # noqa: E402
from coherent_nets import coherent_nets  # noqa: E402

DIRS = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}
LAYERS = ('F.Cu', 'B.Cu')


def obeyed(tracks, chosen, pads, byname, label):
    """Did the fanout OBEY? Measure the direction of the copper that
    actually leaves each ball, rather than trusting that the hint
    landed: pad centre to the far end of the emitted copper, snapped to
    the axis it mostly runs along."""
    by_net = {}
    for t in tracks:
        by_net.setdefault(t['net_id'], []).append(t)
    agree = disagree = absent = 0
    bad = []
    for nm, m in chosen.items():
        p = pads[nm]
        ts = by_net.get(byname[nm][0], [])
        if not ts:
            absent += 1
            continue
        far = max((pt for t in ts for pt in (t['start'], t['end'])),
                  key=lambda q: (q[0] - p.global_x) ** 2
                  + (q[1] - p.global_y) ** 2)
        dx, dy = far[0] - p.global_x, far[1] - p.global_y
        h = math.hypot(dx, dy) or 1
        got = min(DIRS, key=lambda k: (DIRS[k][0] - dx / h) ** 2
                  + (DIRS[k][1] - dy / h) ** 2)
        if got == m.direction:
            agree += 1
        else:
            disagree += 1
            bad.append(f'{nm}({m.direction}->{got})')
    print(f'{label} plan obeyed: {agree}/{agree + disagree + absent} balls '
          f'escaped in the planned direction, {disagree} took another, '
          f'{absent} no copper')
    if bad:
        print('  differed: ' + ', '.join(bad[:12])
              + (f' (+{len(bad) - 12} more)' if len(bad) > 12 else ''))


def copy_pro(src_board, dst_board):
    pro = os.path.splitext(src_board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst_board)[0] + '.kicad_pro')


def plan(base, names):
    """The plan: one destination move per net (choice), from menus of
    legal escapes at both ends, taut-path buses, and the source
    refinement's launch points."""
    pcb = parse_kicad_pcb(base)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ends = te.endpoints(pcb, names, byname)
    kids = {byname[n][0] for n in names}
    cache = {}

    def obs(nid, layer):
        if (nid, layer) not in cache:
            cache[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
        return cache[(nid, layer)]

    def menu(pad, grid, nid):
        return em.enumerate_moves(
            pad, grid, LAYERS,
            lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
            lambda p, L, _n=nid: not (obs(_n, L).point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
    dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
    for nm in names:
        nid, net = byname[nm]
        fp = pcb.footprints[ends[nm][2]]
        bx, by = ends[nm][1]
        pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
                  + (p.global_y - by) ** 2)
        dst_pad[nm] = pad
        dmenu[nm] = menu(pad, em.grid_of(fp), nid)
        launch[nm] = ends[nm][0]
        others = [p for p in net.pads if p.component_ref != ends[nm][2]]
        src_pad[nm] = others[0] if others else None
    dref = ends[names[0]][2]
    dgrid = em.grid_of(pcb.footprints[dref])
    refs = {}
    for nm in names:
        if src_pad[nm] is not None:
            refs[src_pad[nm].component_ref] = refs.get(
                src_pad[nm].component_ref, 0) + 1
    sref = max(refs, key=refs.get)
    sgrid = em.grid_of(pcb.footprints[sref])
    smenu = {}
    for nm in names:
        p = src_pad[nm]
        if p is None or p.component_ref != sref:
            continue
        smenu[nm] = menu(p, sgrid, byname[nm][0])

    print('taut pre-routes...')
    paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
    buses = db.cluster(names, paths)
    tooth0 = {}
    for nm in names:
        nid = byname[nm][0]
        tp = ends[nm][0]
        tooth0[nm] = next(
            (s.layer for s in pcb.segments if s.net_id == nid
             and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
                  or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1]) < 0.005)),
            'F.Cu')
    # the source stubs already on the board, as MOVES: the plan's lane
    # checks must see the gap and via site of every net it leaves alone
    src_seed = {}
    for nm in smenu:
        p = src_pad[nm]
        nid = byname[nm][0]
        tooth = launch[nm]
        near_vias = [v for v in pcb.vias if v.net_id == nid
                     and math.hypot(v.x - p.global_x, v.y - p.global_y) < 2.5]
        d = (tooth[0] - p.global_x, tooth[1] - p.global_y)
        direction = min(DIRS, key=lambda k: (DIRS[k][0] * math.hypot(*d) - d[0]) ** 2
                        + (DIRS[k][1] * math.hypot(*d) - d[1]) ** 2)
        site = (near_vias[0].x, near_vias[0].y) if near_vias else None
        src_seed[nm] = em.Move(nm, 'dogbone' if site else 'surface', direction,
                               tooth0[nm], tooth, len(near_vias),
                               [((p.global_x, p.global_y), tooth, tooth0[nm])],
                               site=site)

    print('planning...')
    _sc, choice, _lp, report = pe.plan_ends(
        smenu, dmenu, launch, sgrid.bbox, dgrid.bbox, buses=buses,
        tooth_layer0=tooth0, src_seed=src_seed,
        pads={nm: (dst_pad[nm].global_x, dst_pad[nm].global_y) for nm in names})
    for line in report:
        print(line)
    return choice, dst_pad, dref, byname


def main():
    out_path = sys.argv[1]
    rest = [a for a in sys.argv[2:] if not a.startswith('-')]
    K = int(rest[0]) if rest else 21
    base = next((a.split('=', 1)[1] for a in sys.argv
                 if a.startswith('--board=')),
                os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb'))
    names = coherent_nets(K)
    choice, dst_pad, dref, byname = plan(base, names)
    hints = {}
    for nm, m in choice.items():
        p = dst_pad[nm]
        hints[(round(p.global_x, 3), round(p.global_y, 3))] = m.direction
    print(f'\nplan: {len(hints)} berth escape directions '
          + ', '.join(f'{d}:{sum(1 for v in hints.values() if v == d)}'
                      for d in sorted(set(hints.values()))))

    # the production engine, in the planned directions; no plane-drop
    # pass (it collides with the decoupling caps under the array, a
    # defect of that pass, not of anything here)
    pcb = parse_kicad_pcb(base)
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[dref], pcb, net_filter=names, layers=list(LAYERS),
        track_width=0.1, clearance=0.1, via_size=0.45, via_drill=0.25,
        exit_margin=0.5, escape_method='auto', plane_drop='off',
        escape_dir_hints=hints)
    if tracks:
        add_tracks_and_vias_to_pcb(
            base, out_path, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb.nets.items()})
    else:
        shutil.copy(base, out_path)
    copy_pro(base, out_path)
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        out_path, '--clearance', '0.1',
                        '--clearance-margin', '0.1'],
                       capture_output=True, text=True)
    clean = 'NO DRC VIOLATIONS' in (r.stdout + r.stderr)
    obeyed(tracks, choice, dst_pad, byname, 'berth')
    print(f'\nwrote {out_path}: {len(tracks)} tracks, {len(vias_add)} '
          f'vias, {len(set(failed))} failed nets, '
          f'{"DRC clean" if clean else "DRC VIOLATIONS"}')
    if failed or not clean:
        print('fanout is not clean and complete -- the braid would route '
              'against broken berths', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
