#!/usr/bin/env python3
"""Fan out the destination array in the directions the PLAN chose.

The plan (plan_ends) picks, per ball, an escape direction, and hands
it to the production fanout engine (generate_bga_fanout) as
escape_dir_hints as FULL moves (face, exit gap, layer, kind, dog-bone
site). A hint accepted is not a move achieved, so the copper that
actually leaves each ball is measured against what was asked for, per
dimension and as an order along each face (source_realize.audit).

The source array arrives already fanned out (the bench), and the plan
is ONE consistent loop over it: destination chosen against the teeth
as they are on the board, source refined on paper against that
destination, the refinement REALIZED by re-fanning those balls with
the same engine (source_realize: every tooth audited, original vs
asked vs achieved), and the next round's destination chosen against
the teeth the copper actually produced. The best realized round's
board is the one the destination fanout runs on.

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
import source_realize as sr  # noqa: E402
from coherent_nets import coherent_nets  # noqa: E402

DIRS = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}
LAYERS = ('F.Cu', 'B.Cu')


def copy_pro(src_board, dst_board):
    pro = os.path.splitext(src_board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst_board)[0] + '.kicad_pro')


ROUNDS = 8     # realized source rounds (feasibility bans need re-plans)
DST_ITERS = 8  # destination select -> fan out -> audit -> ban -> re-select


def plan_state(pcb, names, banned=frozenset()):
    """Everything the plan reads off ONE board: the menus of legal escapes
    at both ends, the launch points (the source teeth AS THEY ARE on this
    board), the layer each tooth ends on, the taut-path buses. `banned`
    holds (net, move signature) pairs the fanout has REFUSED to lay as
    asked: the plan's model said they were possible, the engine said no,
    and the engine is the authority -- they leave the menus."""
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ends = te.endpoints(pcb, names, byname)
    kids = {byname[n][0] for n in names}
    cache = {}

    def obs(nid, layer, own_only=False):
        # `kids` (every net of the run) is excluded for the taut paths and
        # the DESTINATION menu -- a bare array, nothing to exclude. The
        # SOURCE menu prices its moves against the OTHER nets' real stubs
        # (own_only): a partial re-fan of an occupied array meets them as
        # copper, and a menu that hid them chose gaps SDQM0 and SA11 held.
        key = (nid, layer, own_only)
        if key not in cache:
            cache[key] = te.build_obstacles(pcb, nid, {nid} if own_only else kids,
                                            layer)
        return cache[key]

    def menu(pad, grid, nid, own_only=False):
        return em.enumerate_moves(
            pad, grid, LAYERS,
            lambda p, q, L, _n=nid: obs(_n, L, own_only).seg_clear(p, q),
            lambda p, L, _n=nid: not (obs(_n, L, own_only).point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
    dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
    for nm in names:
        nid, net = byname[nm]
        fp = pcb.footprints[ends[nm][2]]
        bx, by = ends[nm][1]
        pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
                  + (p.global_y - by) ** 2)
        dst_pad[nm] = pad
        dmenu[nm] = [m for m in menu(pad, em.grid_of(fp), nid)
                     if (nm, sr.move_sig(m)) not in banned]
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
        smenu[nm] = [m for m in menu(p, sgrid, byname[nm][0], own_only=True)
                     if (nm, sr.move_sig(m)) not in banned]
    paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
    buses = db.cluster(names, paths)
    tooth0 = {}
    tooth_vias = {}
    for nm in names:
        nid = byname[nm][0]
        tp = ends[nm][0]
        tooth0[nm] = next(
            (s.layer for s in pcb.segments if s.net_id == nid
             and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
                  or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1]) < 0.005)),
            'F.Cu')
        # the source escape's own vias, as laid (the judged objective
        # counts them; a dog-bone tooth is a via the crossing floor never saw)
        p = src_pad[nm]
        tooth_vias[nm] = (sum(1 for v in pcb.vias if v.net_id == nid
                              and math.hypot(v.x - p.global_x, v.y - p.global_y) < 6.0)
                          if p is not None else 0)
    return {'byname': byname, 'dmenu': dmenu, 'smenu': smenu, 'launch': launch,
            'tooth0': tooth0, 'tooth_vias': tooth_vias, 'src_pad': src_pad,
            'dst_pad': dst_pad, 'sref': sref, 'dref': dref, 'sgrid': sgrid,
            'dgrid': dgrid, 'buses': buses, 'obs': obs, 'pcb': pcb,
            'pads_of': {ref: [(p.global_x, p.global_y) for p in fp.pads]
                        for ref, fp in pcb.footprints.items()}}


def planned_buses(st, choice):
    """The corridors the BRAID will form for this plan, by the braid's own
    rule (corridor.cluster_corridors: stubs within D on one array that
    arrive from within 60 degrees, single linkage, then split off what
    one spine cannot reach), on the same inputs -- each net's tooth to
    the exit point its planned berth move ends at. The plan's own
    clusters (tooth to the ball, detect_buses alone) were not those: at
    K15 they gave four corridors where the braid formed one, and a via
    model grouped by them predicted 12 for 18 realized."""
    import corridor as cr
    names = [n for n in choice]
    ends = {nm: (st['launch'][nm], choice[nm].exit_pt, st['dref']) for nm in names}
    paths = db.taut_paths(names, ends,
                          lambda nm: st['obs'](st['byname'][nm][0], 'F.Cu'))
    pcb_pads = {}

    def centre_of(ref):
        if ref not in pcb_pads:
            ps = st['pads_of'][ref]
            pcb_pads[ref] = (sum(p[0] for p in ps) / len(ps),
                             sum(p[1] for p in ps) / len(ps))
        return pcb_pads[ref]

    class _Spine:
        """A straight spine for the reach test: mean tooth to mean stub."""
        def __init__(self, grp):
            t = [st['launch'][n] for n in grp]
            s = [choice[n].exit_pt for n in grp]
            p0 = (sum(x for x, _ in t) / len(t), sum(y for _, y in t) / len(t))
            p1 = (sum(x for x, _ in s) / len(s), sum(y for _, y in s) / len(s))
            h = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) or 1.0
            d = ((p1[0] - p0[0]) / h, (p1[1] - p0[1]) / h)
            self.P = [p0, p1]
            self.d = [d, d]
    nid0 = st['byname'][names[0]][0]
    pad_clear = lambda p, q: st['obs'](nid0, 'F.Cu').seg_clear(p, q)
    return cr.cluster_corridors(
        names, paths, {nm: st['launch'][nm] for nm in names},
        {nm: choice[nm].exit_pt for nm in names}, pad_clear, D=6.0,
        spine_fn=lambda grp: _Spine(grp),
        dest_ref={nm: st['dref'] for nm in names},
        centres={nm: centre_of(st['dref']) for nm in names},
        src_centres={nm: centre_of(st['sref']) for nm in names})


def braid_plan_of(st, choice, board, achieved=None):
    """The plan as the braid's planner takes it: ends (tooth on the
    board, planned exit -- or, once the fanout has laid it, the ACHIEVED
    stub end, which sits an occupancy cell inside the boundary line: a
    lane routed to the planned point instead left a 25 um same-net gap at
    every berth), both layers, escape directions (the tooth's read off
    its copper by the braid's own _end_dir; the berth's = its face)."""
    pcb = st['pcb']
    plan = {'ends': {}, 'tooth_layer': {}, 'dest_layer': {},
            'tooth_dir': {}, 'stub_dir': {}}
    for nm, m in choice.items():
        nid, net = st['byname'][nm]
        exit_pt = (achieved[nm]['tooth'] if achieved and nm in achieved
                   else m.exit_pt)
        plan['ends'][nm] = [list(st['launch'][nm]), list(exit_pt)]
        plan['tooth_layer'][nm] = st['tooth0'][nm]
        plan['dest_layer'][nm] = m.layer
        plan['tooth_dir'][nm] = list(te._end_dir(pcb, nid, st['launch'][nm], net.pads))
        plan['stub_dir'][nm] = list(DIRS[m.direction])
    return plan


def judge_by_braid(st, choice, board, achieved=None):
    """THE judgment of a candidate plan: the braid's own planner
    (braid.plan_braid) on the plan's ends -- corridors as the braid forms
    them, its orders, its pages -- priced per net (plan_ends.vias_from_pages)
    plus the ride round both arrays. Returns (cost, per-net vias, the
    braid's per-net plan, the plan dict)."""
    plan = braid_plan_of(st, choice, board, achieved)
    bp = te.plan_braid(board, list(choice), st['dref'], plan)
    pages = {nm: bp[nm]['page'] for nm in choice}
    legs = {nm: bp[nm].get('exit_leg_layer') for nm in choice}
    ups = {nm: bp[nm].get('underpasses', 0) for nm in choice}
    pred = pe.vias_from_pages(choice, st['tooth0'], st['tooth_vias'], pages, legs, ups)
    ride = pe.sm.ride_mm(choice, st['launch'], st['dgrid'].bbox,
                         st['sgrid'].bbox) / pe.sm.VIA_MM
    return sum(pred.values()) + ride, pred, bp, plan


def total(dst_c, st, cache, buses=None):
    """What the plan is judged on (plan_ends.judged_cost): the vias the
    plan's own model implies, both escapes' vias included, plus the ride
    round both arrays at VIA_MM per via -- keepers judged within the
    corridors the braid will form (planned_buses)."""
    return pe.judged_cost(dst_c, st['launch'], st['dgrid'].bbox, cache,
                          st['sgrid'].bbox, st['tooth0'], st['tooth_vias'],
                          buses if buses is not None else planned_buses(st, dst_c))


def plan(base, names, work):
    """The plan as ONE consistent loop. Each round: choose the destination
    escapes against the source teeth AS THEY ARE on the current board;
    refine the source on paper against that destination; REALIZE the
    refinement -- strip those nets' source copper and re-fan them with the
    production engine in the asked faces (source_realize, which audits
    every tooth: original vs asked vs achieved); the next round chooses
    the destination against the teeth that copper produced. There is no
    paper-only launch point anywhere: every floor printed here is measured
    against teeth that exist on a written board. The best round's board and
    destination choice are what the destination fanout then runs on."""
    board = base
    best = None
    prev_launch = None
    realized = []
    banned = set()          # (net, move signature) the fanout refused
    new_bans = 0
    for r in range(ROUNDS + 1):
        st = plan_state(parse_kicad_pcb(board), names, banned)
        if prev_launch is not None and st['launch'] == prev_launch and not new_bans:
            print(f'  round {r}: the realized teeth are identical to the previous '
                  f'round\'s and nothing new was banned -- converged')
            break
        prev_launch = dict(st['launch'])
        new_bans = 0
        cache = {}
        pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y)
                for nm in names}
        dst_choice, un = pe.sm.select(st['dmenu'], st['launch'],
                                      keep_out=st['dgrid'].bbox, buses=st['buses'],
                                      tooth_layer=st['tooth0'], log=None, pads=pads)
        if not dst_choice:
            print(f'  round {r}: no destination choice'); break
        pb = planned_buses(st, dst_choice)
        f_fast = total(dst_choice, st, cache, pb)
        f, _pred, bp, _plan = judge_by_braid(st, dst_choice, board)
        n_corr = len({v['corridor'] for v in bp.values()})
        line = (f'  round {r}: destination vs the teeth ON {os.path.basename(board)}: '
                f'braid-judged {f:.2f} (fast proxy {f_fast:.2f}), {len(dst_choice)} placed'
                + (f', {len(un)} unplaced' if un else ''))
        if best is None or f < best[0]:
            best = (f, board, dst_choice, st, r)
            line += '   <- best'
        line += f'  ({n_corr} corridor(s) by the braid\'s planner)'
        print(line)
        if r == ROUNDS:
            break
        sub = {n: ms for n, ms in st['smenu'].items() if n in dst_choice and ms}
        if not sub:
            break
        src_choice, _nxt, sf = pe.refine_source({}, sub, dst_choice,
                                                st['dgrid'].bbox, st['launch'],
                                                cache=cache,
                                                src_box=st['sgrid'].bbox,
                                                tooth_layer0=st['tooth0'],
                                                tooth_vias0=st['tooth_vias'],
                                                buses=pb)
        print(f'  round {r}: source refine on PAPER -> floor {sf:.2f} '
              f'({len(src_choice)} teeth to move)')
        if not src_choice:
            print('  no source move to realize'); break
        new_board = f'{work}_src{r + 1}.kicad_pcb'
        res = sr.realize(board, src_choice, st['src_pad'], st['byname'],
                         st['sref'], new_board, guard_names=names)
        realized.append(res)
        # FEEDBACK: every asked move the engine did not lay exactly leaves
        # that net's menu; the next round plans over what is achievable
        misses = [nm for nm, e in res['audit'].items() if not e['exact']]
        for nm in misses:
            banned.add((nm, sr.move_sig(src_choice[nm])))
        new_bans = len(misses)
        if misses:
            print(f'  round {r}: {len(misses)} asked source move(s) not laid as '
                  f'asked -> banned for re-planning: {misses}')
        if res['rejected']:
            print(f'  round {r}: realized board REJECTED ({res["rejected"]}); '
                  f'keeping {os.path.basename(board)}')
            for nm in src_choice:
                banned.add((nm, sr.move_sig(src_choice[nm])))
            new_bans += len(src_choice)
            continue
        board = new_board
    f, board, choice, st, r = best
    print(f'  kept round {r}: floor {f:.2f} on {os.path.basename(board)}')
    return choice, st['dst_pad'], st['dref'], st['byname'], board, realized, banned


def explain_plan(choice, st, names, out_path=None, board=None, achieved=None):
    """The PLANNER's model of the plan that ships, per net -- the braid's
    own (plan_braid on the plan's ends): corridor, launch/target index,
    page, the layers at both ends, the escapes' vias, the vias predicted
    -- so it can be held against via_census. With `out_path` the plan is
    written beside the fanout board as `<board>.plan.json`; the braid
    reads it and builds its corridors from the identical inputs."""
    import json
    cost, pred, bp, plan = judge_by_braid(st, choice, board, achieved)
    corrs = sorted({v['corridor'] for v in bp.values()})
    print('  planner (the braid\'s own, on the plan\'s ends) per net: corridor, '
          'launch/target index, page, tooth layer+vias, berth escape, predicted vias')
    for ci in corrs:
        mem = sorted([nm for nm in choice if bp[nm]['corridor'] == ci],
                     key=lambda n: (bp[n]['launch_idx'] if bp[n]['launch_idx'] is not None else -1))
        print(f'    corridor {ci} ({len(mem)}): launch order {mem}')
        print(f'      page F: {[n for n in mem if bp[n]["page"] == "F.Cu"]}')
        print(f'      page B: {[n for n in mem if bp[n]["page"] == "B.Cu"]}   '
              f'swimmers: {[n for n in mem if bp[n]["page"] is None]}')
        for nm in mem:
            m = choice[nm]
            pg = bp[nm]['page']
            print(f'      {nm:7s} L{bp[nm]["launch_idx"]}->T{bp[nm]["target_idx"]}  '
                  f'page {pg[0] if pg else "swim"}  tooth {st["tooth0"][nm][0]} '
                  f'v={st["tooth_vias"].get(nm, 0)}  berth {m.kind}/{m.direction}/{m.layer[0]} '
                  f'v={m.vias}  predicted {pred[nm]}'
                  + ('  joiner' if bp[nm]['joiner'] else '')
                  + (f'  side-exit leg on {bp[nm]["exit_leg_layer"][0]}'
                     if bp[nm].get('exit_leg_layer') else
                     ('  side-exit' if bp[nm]['side_exit'] else ''))
                  + (f'  under-passes {bp[nm]["underpasses"]}'
                     if bp[nm].get('underpasses') else ''))
    print(f'  plan model total predicted vias: {sum(pred.values())} over {len(pred)} nets '
          f'(braid-judged cost {cost:.2f} incl. ride)')
    if out_path:
        side = os.path.splitext(out_path)[0] + '.plan.json'
        with open(side, 'w', encoding='utf-8') as f:
            json.dump(plan, f, indent=1, sort_keys=True)
        print(f'  plan written to {os.path.basename(side)}')


def main():
    out_path = sys.argv[1]
    rest = [a for a in sys.argv[2:] if not a.startswith('-')]
    K = int(rest[0]) if rest else 21
    base = next((a.split('=', 1)[1] for a in sys.argv
                 if a.startswith('--board=')),
                os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb'))
    names = coherent_nets(K)
    print('planning (source realized every round)...')
    work = out_path[:-len('.kicad_pcb')] if out_path.endswith('.kicad_pcb') else out_path
    choice, dst_pad, dref, byname, board, realized, banned = plan(base, names, work)
    return fanout_destination(out_path, names, choice, dst_pad, dref, byname,
                              board, realized, banned)


def fanout_destination(out_path, names, choice, dst_pad, dref, byname, board,
                       realized, banned):
    """Fan out DU1 to the plan, audit, and FEED BACK: a berth the engine
    could not lay as asked leaves that net's menu, the destination is
    re-selected against the same teeth, and the fanout runs again --
    until every berth is exactly the plan's, or nothing changes. The
    plan's own via model is printed for the plan that ships."""
    st = plan_state(parse_kicad_pcb(board), names, banned)
    for it in range(DST_ITERS):
        faces = [m.direction for m in choice.values()]
        print(f'\nplan (destination pass {it}): {len(choice)} berth escape directions '
              + ', '.join(f'{d}:{faces.count(d)}' for d in sorted(set(faces)))
              + f'  (source board: {os.path.basename(board)}, '
              f'{len(realized)} realized round(s), {len(banned)} banned move(s))')
        laid, audit_d, ok = fanout_once(out_path, names, choice, dst_pad, dref,
                                        byname, board)
        misses = [nm for nm in choice if not audit_d.get(nm, {}).get('exact')]
        if not misses:
            print(f'  destination pass {it}: every berth laid as planned')
            # judged on the WRITTEN fanout board: the same copper the braid
            # will read, so its taut paths are the memo's (detect_buses)
            explain_plan(choice, st, names, out_path, out_path,
                         achieved=getattr(fanout_once, 'achieved', None))
            return 0 if ok else 1
        for nm in misses:
            banned.add((nm, sr.move_sig(choice[nm])))
        print(f'  destination pass {it}: {len(misses)} berth(s) not laid as asked '
              f'-> banned, re-planning the destination: {misses}')
        st = plan_state(parse_kicad_pcb(board), names, banned)
        pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y)
                for nm in names}
        new_choice, un = pe.sm.select(st['dmenu'], st['launch'],
                                      keep_out=st['dgrid'].bbox, buses=st['buses'],
                                      tooth_layer=st['tooth0'], log=None, pads=pads)
        if not new_choice or new_choice == choice:
            print('  destination: the re-plan changed nothing -- stopping')
            explain_plan(choice, st, names, out_path, out_path,
                         achieved=getattr(fanout_once, 'achieved', None))
            return 0 if ok else 1
        f, _p, _bp, _pl = judge_by_braid(st, new_choice, board)
        print(f'  destination re-plan: braid-judged {f:.2f}, {len(new_choice)} placed'
              + (f', {len(un)} unplaced' if un else ''))
        choice, dst_pad = new_choice, st['dst_pad']
    explain_plan(choice, st, names, out_path, out_path,
                 achieved=getattr(fanout_once, 'achieved', None))
    return 0


def fanout_once(out_path, names, choice, dst_pad, dref, byname, board):
    """One destination fanout to `choice`, written to out_path and audited.
    Returns (laid nets, audit dict, clean-and-complete)."""
    hints = {}
    for nm, m in choice.items():
        p = dst_pad[nm]
        hints[(round(p.global_x, 3), round(p.global_y, 3))] = sr.full_move(m)
    # the production engine, following the FULL planned moves (face, exit
    # gap, layer, kind), with the VIA the plan priced its moves with (the
    # braid's 0.25/0.15; a 0.45 via cannot sit in a 0.65 mm pitch gap).
    # The under-pad engine is the one that follows a plan (its plan-follow
    # phase; 'auto' would let the channel engine take the face and choose
    # the rest itself). No plane-drop pass (it collides with the decoupling
    # caps under the array, a defect of that pass, not of anything here).
    # No placement step follows this chain, so every foreign pad is one a
    # via must clear.
    pcb = parse_kicad_pcb(board)
    pcb._fanout_all_foreign_immovable = True
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[dref], pcb, net_filter=names, layers=list(LAYERS),
        track_width=0.1, clearance=0.1, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
        exit_margin=0.5, escape_method='underpad', plane_drop='off',
        escape_dir_hints=hints)
    if tracks:
        add_tracks_and_vias_to_pcb(
            board, out_path, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb.nets.items()})
    else:
        shutil.copy(board, out_path)
    copy_pro(board, out_path)
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        out_path, '--clearance', '0.1',
                        '--clearance-margin', '0.1'],
                       capture_output=True, text=True)
    clean = 'NO DRC VIOLATIONS' in (r.stdout + r.stderr)
    # the per-tooth audit at the destination: face, layer, kind, gap and
    # ORDER, measured off the written board (source_realize.audit)
    pcb_out = parse_kicad_pcb(out_path)
    got = {t['net_id'] for t in tracks}
    laid = [nm for nm in choice if byname[nm][0] in got]
    achieved = {nm: sr.measure_tooth(pcb_out, nm, dst_pad[nm], byname, dest_ref=dref)
                for nm in laid}
    audit_d, _counts = sr.audit(choice, achieved, None, laid, print, 'berth')
    fanout_once.achieved = achieved
    print(f'\nwrote {out_path}: {len(tracks)} tracks, {len(vias_add)} '
          f'vias, {len(set(failed))} failed nets, '
          f'{"DRC clean" if clean else "DRC VIOLATIONS"}')
    if failed or not clean:
        print('fanout is not clean and complete -- the braid would route '
              'against broken berths', file=sys.stderr)
    return laid, audit_d, (not failed and clean)


if __name__ == '__main__':
    sys.exit(main())
