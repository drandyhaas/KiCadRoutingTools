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
import contextlib
import io
import os
import re
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
import rules as _rules  # noqa: E402  ONE source for every design rule

# SRC_CLIMB=k (2026-09-10): the SOURCE menu also offers CLIMBS -- a dog-bone
# or via-in-pad whose run first travels up to k pitches along a gap under
# the array and leaves the face at a chosen row or column (the human's
# north riders at K51; escape_moves.enumerate_moves climb=). 0 = off, the
# menu byte-identical. replan.py runs with 14.
SRC_CLIMB = int(os.environ.get('SRC_CLIMB', '0'))
# PLAN_PAGES=1 (2026-09-13): the PAGES-FIRST planner (pages_first.py) chooses
# BOTH ends and the page of every net in one CP-SAT with hard two-page
# planarity, so no net needs more than two vias by construction. 0 = the
# recorded planner, byte-identical. DST_CLIMB=k enumerates destination
# dog-bones whose run climbs along the array up to k pitches before it
# leaves (escape_moves climb=), the class that makes a B berth's rank free.
PLAN_PAGES = int(os.environ.get('PLAN_PAGES', '0'))
DST_CLIMB = int(os.environ.get('DST_CLIMB', '0'))
# PLAN_JUDGE (2026-09-15, THE PLAN item 1): what a candidate plan is JUDGED
# on. '' (default, byte-identical): the old planner on judge_by_braid's
# ride-priced cost, pages-first on (the braid's residue, the CP-SAT's own
# via count). 'count': the braid's PLAN-IMPLIED COUNT is the cost --
# the ends as they stand (the teeth as laid, the berths as chosen) + every
# page lane's `changes` + every swimmer's `swim_changes` + cross-corridor
# vias, NO ride -- what plan_vias.py / judge_gate.py compute, the one
# number that predicted the routed board (K41 DET 40/80/160/320: 84 / 94
# / 90 / 96 -> routed 79 / 102+1o / 98 / 92 where the residue + model
# judge approved every worse plan); the pages-first key becomes (count,
# residue). 'flat': the same with a flat prices.SWIM per swimmer.
PLAN_JUDGE = os.environ.get('PLAN_JUDGE', '')
PLAN_JUDGE_RIDE = int(os.environ.get('PLAN_JUDGE_RIDE', '1') or 0)
# PLAN_JUDGE_LEN: the length ESTIMATOR the judge prices at VIA_MM -- 'lane' (the
# braid's planned polylines + berth runs; default) or 'ride' (the around-box
# ride from launch to berth exit, ride_mm: the jcr arm, 3x over on the K35 batch)
PLAN_JUDGE_LEN = os.environ.get('PLAN_JUDGE_LEN', 'ride')
if PLAN_JUDGE not in ('', 'count', 'flat'):
    raise SystemExit(f'PLAN_JUDGE={PLAN_JUDGE!r}: expected count | flat | unset')


from escape_moves import DIRS, LAYERS  # noqa: E402,F401  -- ONE source


FAST_PRO = False   # True (a probe's intermediate boards): the sidecar copied, not re-scanned


def copy_pro(src_board, dst_board):
    pro = os.path.splitext(src_board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst_board)[0] + '.kicad_pro')
    if FAST_PRO:
        # a probe's bare / source / destination boards carry the copper of
        # the board they came from at the same widths: the floor the copy
        # already holds is the floor the scan would write (2026-09-18: the
        # scan was 0.25 s of a 6.6 s probe, three boards a probe)
        return
    # ...and stamp the floor this stage fans out at (0.1 / 0.1 / the braid's
    # via, the numbers fanout_once is called with below), lower-only, as the
    # production CLIs do -- see braid.write_out for why a bare copy was not
    # enough (a Default class clearance of 0.0 rode down every chain step).
    if os.environ.get('AWX_STAMP_PRO', '1') == '0':   # the flag-off parity control
        return
    try:
        from fix_kicad_drc_settings import fix_project_for_output
        fix_project_for_output(dst_board, src_board, clearance=te.SPEC_CLEARANCE,
                               track_width=sr.FAN_TRACK,
                               via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL,
                               verbose=False)
    except Exception as e:
        print(f'  project floor NOT stamped: {e}', flush=True)


ROUNDS = int(os.environ.get('SRC_ROUNDS', '8'))   # realized source rounds (feasibility bans need re-plans); 0 = the teeth as they stand
DST_ITERS = 8  # destination select -> fan out -> audit -> ban -> re-select


def _load_force(var):
    """PLAN_FORCE_DST / PLAN_FORCE_SRC (a PROBE, 2026-09-11, ported back
    from handoff_0910b): a JSON file {net: {"direction": .., "layer": ..,
    "kind": ..}} restricts that net's menu at that end to the class named
    (any subset of the three keys). Written by tmp/human_sides.py off the
    human's copper, it measures the plan's headroom -- what the braid does
    on our teeth with the human's destination classes -- not a mechanism."""
    path = os.environ.get(var, '')
    if not path:
        return {}
    import json
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _force(force, nm, moves, end):
    want = force.get(nm)
    if not want:
        return moves
    keep = [m for m in moves
            if all(getattr(m, k, None) == v for k, v in want.items()
                   if k in ('direction', 'layer', 'kind'))]
    if not keep:
        print(f'  force ({end}): {nm} has no {want} move among {len(moves)} -- menu kept')
        return moves
    return keep


FORCE_DST = _load_force('PLAN_FORCE_DST')
FORCE_SRC = _load_force('PLAN_FORCE_SRC')


def dedupe_climbs(moves):
    """One CLIMBED candidate per (kind, face, layer, exit row) -- the cheapest
    by (vias, run length) -- the plain candidates untouched, in the menu's
    own order (the CP-SAT model is built in it). enumerate_moves emits a
    climb per (start site, gap side, half-pitch step), so one exit row
    arrives four to six ways that differ only in the run's first bend;
    measured at K28 with DST_CLIMB=2: 2812 berth candidates and 718k
    pairwise exclusions, the solve stopping worse than without climbs and
    routing 42 against 34; deduped 1361. Inert when no move climbs."""
    best = {}
    for m in moves:
        if not getattr(m, 'climb', 0):
            continue
        ax = 0 if m.direction in ('up', 'down') else 1
        k = (m.kind, m.direction, m.layer, round(m.exit_pt[ax], 2))
        c = (m.vias, sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b, L in m.legs))
        if k not in best or c < best[k][0]:
            best[k] = (c, m)
    keep = {id(v[1]) for v in best.values()}
    return [m for m in moves if not getattr(m, 'climb', 0) or id(m) in keep]


def _lane_of(m, ax):
    """The coordinate of the LANE a climb runs along -- the column line or
    column-gap midline for a run that climbs in y, the row line or row gap
    for one that climbs in x. It is the run leg's constant coordinate, read
    off the move's own legs (the one perpendicular to the face's `ax`)."""
    pax = 1 - ax
    for (a, b, _L) in reversed(m.legs or ()):
        if abs(a[ax] - b[ax]) > 1e-6 and abs(a[pax] - b[pax]) <= 1e-6:
            return a[pax]                      # the climb leg itself
    return (m.site or m.exit_pt)[pax]


def plan_state(pcb, names, banned=frozenset()):
    """Everything the plan reads off ONE board: the menus of legal escapes
    at both ends, the launch points (the source teeth AS THEY ARE on this
    board), the layer each tooth ends on, the taut-path buses. `banned`
    holds (net, move signature) pairs the fanout has REFUSED to lay as
    asked: the plan's model said they were possible, the engine said no,
    and the engine is the authority -- they leave the menus."""
    plan_state._pair_legs = None
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

    def menu(pad, grid, nid, own_only=False, climb=0):
        return em.enumerate_moves(
            pad, grid, LAYERS,
            lambda p, q, L, _n=nid: obs(_n, L, own_only).seg_clear(p, q),
            lambda p, L, _n=nid: not (obs(_n, L, own_only).point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0],
            climb=climb)
    dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
    dref = ends[names[0]][2]
    dgrid = em.grid_of(pcb.footprints[dref])
    dboxes = dgrid.bbox
    for nm in names:
        nid, net = byname[nm]
        fp = pcb.footprints[ends[nm][2]]
        bx, by = ends[nm][1]
        pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
                  + (p.global_y - by) ** 2)
        dst_pad[nm] = pad
        moves = dedupe_climbs(menu(pad, em.grid_of(fp), nid, climb=DST_CLIMB))
        dmenu[nm] = [m for m in moves if (nm, sr.move_sig(m)) not in banned]
        # a pad of this net on the other layer UNDER the ball (a back-side
        # termination resistor under a DDR clock ball) is served by a TIE
        # VIA at the ball (tie_vias_under, after the berths are laid), so
        # the menu stays the plan's. It used to keep via-in-pad moves only:
        # every one of them was "infeasible even alone" under the DDR (the
        # B.Cu escape from the barrel runs into the resistor's other pad
        # and the neighbours' dogbones), three destination passes banned
        # and re-planned the berths, and the plan ended on a surface
        # escape with the pad unreached (K36, 2026-09-20).
        import pairs as _pairs
        if any(_pairs.under_pad(pad, q, te.VIA_SIZE) for q in net.pads):
            # ...and it takes NO via-in-pad escape: the barrel's other-layer
            # run leaves through the pad's own footprint and the partner pad
            # beside it (measured infeasible in every direction at K36), and
            # a banned pair leg with 33 berths held has no joint move left
            keep = [m for m in dmenu[nm] if m.kind != 'via_in_pad']
            print(f'  {nm}: a pad of its own lies under the ball on the other layer -- '
                  f'tied by a via at the ball once its berth is laid; no via-in-pad escape '
                  f'({len(keep)} of {len(dmenu[nm])} moves kept)')
            if keep:
                dmenu[nm] = keep
        # a PAIR leg keeps only the moves with room for the pair at the exit
        _plegs = getattr(plan_state, '_pair_legs', None)
        if _plegs is None:
            _plegs = {}
            if int(os.environ.get('PLAN_PAIRS', os.environ.get('BRAID_PAIRS', '0')) or 0):
                for _b, (_pn, _nn) in _pairs.pair_names(names).items():
                    _plegs[_pn], _plegs[_nn] = _nn, _pn
            plan_state._pair_legs = _plegs
        if nm in _plegs and _plegs[nm] in byname:
            keep = [m for m in dmenu[nm] if pair_exit_clear(pcb, nid, byname[_plegs[nm]][0], m)]
            if len(keep) < len(dmenu[nm]):
                print(f'  {nm}: {len(dmenu[nm]) - len(keep)} of {len(dmenu[nm])} berth moves have no room '
                      f'for the pair at the exit -- dropped')
            if keep:
                dmenu[nm] = keep
        dmenu[nm] = _force(FORCE_DST, nm, dmenu[nm], 'destination')
        launch[nm] = ends[nm][0]
        others = [p for p in net.pads if p.component_ref != ends[nm][2]]
        src_pad[nm] = others[0] if others else None
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
        smenu[nm] = [m for m in dedupe_climbs(menu(p, sgrid, byname[nm][0], own_only=True, climb=SRC_CLIMB))
                     if (nm, sr.move_sig(m)) not in banned]
        _plegs = getattr(plan_state, '_pair_legs', None) or {}
        if nm in _plegs and _plegs[nm] in byname:
            keep = [m for m in smenu[nm] if pair_exit_clear(pcb, byname[nm][0], byname[_plegs[nm]][0], m)]
            if len(keep) < len(smenu[nm]):
                print(f'  {nm}: {len(smenu[nm]) - len(keep)} of {len(smenu[nm])} tooth moves have no room '
                      f'for the pair at the exit -- dropped')
            smenu[nm] = keep
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
    # the layer the taut paths relax against: the one most teeth are
    # born on, not F -- the board turned over (mirror_board.py, every
    # tooth on B) planned 55 vias for 28 nets where the front planned 37,
    # because its taut strings dodged the caps that had come to F and
    # ignored the arrays' own side; the bench (every tooth on F) is
    # unchanged by construction
    bundle_layer = te.bundle_layer_of(tooth0)
    # the pair's canonical frame for the handed selector (select_moves)
    chi = pe.sm.pair_chirality({nm: (src_pad[nm].global_x, src_pad[nm].global_y)
                                for nm in names if src_pad.get(nm) is not None},
                               {nm: (dst_pad[nm].global_x, dst_pad[nm].global_y)
                                for nm in names if dst_pad.get(nm) is not None},
                               sgrid.bbox, dgrid.bbox)
    paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], bundle_layer))
    buses = db.cluster(names, paths)
    return {'banned': banned,          # the feasibility ledger, for a proposal
                                       # that enumerates its own moves
            'byname': byname, 'dmenu': dmenu, 'smenu': smenu, 'launch': launch,
            'tooth0': tooth0, 'tooth_vias': tooth_vias, 'src_pad': src_pad,
            'dst_pad': dst_pad, 'sref': sref, 'dref': dref, 'sgrid': sgrid,
            'bundle_layer': bundle_layer, 'chi': chi,
            'dgrid': dgrid, 'dboxes': dboxes,
            'buses': buses, 'obs': obs, 'pcb': pcb,
            'pads_of': {ref: [(p.global_x, p.global_y) for p in fp.pads]
                        for ref, fp in pcb.footprints.items()}}


def _menu_match(menu, g, tol=0.35):
    """The menu move of the ACHIEVED berth's class (kind, face, layer)
    nearest to its stub end along the face, within `tol`; None when the
    engine laid something the menu does not name."""
    if not g or g.get('direction') not in DIRS:
        return None
    ax = 0 if g['direction'] in ('up', 'down') else 1
    best = None
    for m in menu:
        if m.kind == g['kind'] and m.direction == g['direction'] and m.layer == g['layer']:
            d = abs(m.exit_pt[ax] - g['tooth'][ax])
            if d <= tol and (best is None or d < best[0]):
                best = (d, m)
    return best[1] if best else None


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
                          lambda nm: st['obs'](st['byname'][nm][0], st['bundle_layer']))
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
    pad_clear = lambda p, q: st['obs'](nid0, st['bundle_layer']).seg_clear(p, q)
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
            'tooth_dir': {}, 'stub_dir': {}, 'chi': int(st['chi'])}
    for nm, m in choice.items():
        nid, net = st['byname'][nm]
        # the sidecar describes the BOARD it sits beside: once the fanout
        # has laid a berth, its laid end, layer and face -- not the asked
        # ones. K41's destination passes never converge, and a sidecar
        # written from the asked layers named 22 berths on the wrong
        # layer; the braid then routed to stub ends with no copper there.
        got = achieved[nm] if achieved and nm in achieved else None
        exit_pt = got['tooth'] if got else m.exit_pt
        plan['ends'][nm] = [list(st['launch'][nm]), list(exit_pt)]
        plan['tooth_layer'][nm] = st['tooth0'][nm]
        plan['dest_layer'][nm] = got['layer'] if got else m.layer
        # a CANDIDATE TOOTH (src_over): its copper is not on the
        # board yet, so _end_dir would read the tooth it would REPLACE and
        # hand the braid the old escape direction
        so = (st.get('src_over') or {}).get(nm)
        plan['tooth_dir'][nm] = (list(so['dir']) if so else
                                 list(te._end_dir(pcb, nid, st['launch'][nm], net.pads)))
        plan['stub_dir'][nm] = list(DIRS[(got['direction'] if got and got.get('direction') in DIRS
                                          else m.direction)])
    return plan


# EVERY BUDGET IN THIS FILE IS COUNTED IN JUDGE CALLS, NEVER IN SECONDS.
# A clock budget does not make a slow machine answer later, it makes it
# answer DIFFERENTLY: measured, two identical cloud runs of the K35
# baseline came back 72 vias / 1436 segs and 58 / 1840, because a
# container is ~2x slower than the laptop and budgets that never bind
# locally bound there. The loops below all terminate naturally (finite
# sweeps over finite candidates); these caps are the safety net, in the
# one unit that is the same on every machine -- calls to the braid's
# planner, which is what the search actually spends.
PLAN_CALLS = [0]


def _spent():
    return PLAN_CALLS[0]


PAIR_BAD_W = float(os.environ.get('PLAN_PAIR_BAD_W', '50') or 0)


def pair_penalty(st, choice, log=None):
    """THE JUDGE'S VIEW OF A PAIR (2026-09-20): a differential pair whose
    two ends are not NEIGHBOURS -- one face, one layer, within 1.3 pitches,
    nothing of any other net between them -- cannot be launched or landed
    coupled, so a plan with such an end is not a plan for that pair. The
    price is PAIR_BAD_W per bad end (vias-equivalent, above any count the
    residue rounds trade: a source round that moved the pair's teeth
    together was judged 204 -> 233 and REVERTED, the moves banned, K36
    pf9). Berths from `choice`, teeth as they stand (st['launch'] / tooth0).
    Returns (penalty, [reasons])."""
    import pairs as _pairs
    if not int(os.environ.get('PLAN_PAIRS', os.environ.get('BRAID_PAIRS', '0')) or 0) or PAIR_BAD_W <= 0:
        return 0.0, []
    bad = []
    dg, sg = st['dgrid'], st['sgrid']
    reach_d = 1.3 * max(dg.pitch_x, dg.pitch_y)
    reach_s = 1.3 * max(sg.pitch_x, sg.pitch_y)
    for base, (pn, nn) in _pairs.pair_names(list(choice)).items():
        if pn not in choice or nn not in choice:
            continue
        a, b = choice[pn], choice[nn]
        ea, eb = getattr(a, 'exit_pt', None), getattr(b, 'exit_pt', None)
        if ea is not None and eb is not None:
            d = math.hypot(ea[0] - eb[0], ea[1] - eb[1])
            if (a.direction, a.layer) != (b.direction, b.layer) or not (0.05 < d <= reach_d):
                bad.append(f'{base} berths apart ({a.direction}/{a.layer} vs {b.direction}/{b.layer}, {d:.2f} mm)')
            else:
                ax = 0 if a.direction in ('up', 'down') else 1
                lo_, hi_ = sorted((ea[ax], eb[ax]))
                between = [o for o, m in choice.items() if o not in (pn, nn)
                           and getattr(m, 'exit_pt', None) is not None
                           and (m.direction, m.layer) == (a.direction, a.layer)
                           and lo_ + 0.02 < m.exit_pt[ax] < hi_ - 0.02]
                if between:
                    bad.append(f'{base} berths with {between} between')
        la, lb = st['launch'].get(pn), st['launch'].get(nn)
        if la is not None and lb is not None:
            L_a, L_b = st['tooth0'].get(pn), st['tooth0'].get(nn)
            d = math.hypot(la[0] - lb[0], la[1] - lb[1])
            if L_a != L_b or not (0.05 < d <= reach_s):
                bad.append(f'{base} teeth apart ({L_a} vs {L_b}, {d:.2f} mm)')
            else:
                if abs(la[0] - lb[0]) <= 0.05:
                    fx, ax = 0, 1
                elif abs(la[1] - lb[1]) <= 0.05:
                    fx, ax = 1, 0
                else:
                    fx, ax = None, None
                if ax is not None:
                    lo_, hi_ = sorted((la[ax], lb[ax]))
                    between = [o for o, p in st['launch'].items() if o not in (pn, nn)
                               and st['tooth0'].get(o) == L_a and abs(p[fx] - la[fx]) <= 0.05
                               and lo_ + 0.02 < p[ax] < hi_ - 0.02]
                    if between:
                        bad.append(f'{base} teeth with {between} between')
        # ...and one handedness at both ends (pairs.hand)
        if ea is not None and eb is not None and la is not None and lb is not None:
            try:
                ga = sr.measure_tooth(st['pcb'], pn, st['src_pad'][pn], st['byname'])
                hs = _pairs.hand(ga['direction'], la, lb) if ga and ga.get('direction') else 0
            except Exception:
                hs = 0
            hd = _pairs.hand(a.direction, ea, eb, arriving=True)
            if hs and hd and hs != hd:
                bad.append(f'{base} handedness: teeth {hs:+d}, berths {hd:+d}')
    if bad and log:
        log('  pairs: ' + '; '.join(bad))
    return PAIR_BAD_W * len(bad), bad


def judge_by_braid(st, choice, board, achieved=None, bp=None):
    """THE judgment of a candidate plan: the braid's own planner
    (braid.plan_braid) on the plan's ends -- corridors as the braid forms
    them, its orders, its pages -- priced per net (plan_ends.vias_from_pages)
    plus the ride round both arrays. Returns (cost, per-net vias, the
    braid's per-net plan, the plan dict). `bp` = the planner's answer
    computed elsewhere (a worker process), priced here."""
    plan = braid_plan_of(st, choice, board, achieved)
    if bp is None:
        PLAN_CALLS[0] += 1
        bp = te.plan_braid(board, list(choice), st['dref'], plan)
    pages = {nm: bp[nm]['page'] for nm in choice}
    legs = {nm: bp[nm].get('exit_leg_layer') for nm in choice}
    chg = {nm: bp[nm].get('changes') for nm in choice}
    xv = {nm: bp[nm].get('cross_vias', 0) for nm in choice}
    swc = {nm: bp[nm].get('swim_changes') for nm in choice}
    pred = pe.vias_from_pages(choice, st['tooth0'], st['tooth_vias'], pages, legs,
                              changes=chg, cross=xv, swim_changes=swc,
                              swim_mode={'count': 'changes', 'flat': 'flat'}.get(PLAN_JUDGE))
    ride = pe.sm.ride_mm(choice, st['launch'], st['dboxes'],
                         st['sgrid'].bbox) / pe.sm.VIA_MM
    if PLAN_JUDGE and PLAN_JUDGE_RIDE and PLAN_JUDGE_LEN == 'lane':
        # the LENGTH the braid itself planned: each lane's polyline (launch
        # to stub end, as the plan phase drew it) plus the berth's own run,
        # at VIA_MM; the around-box ride only for a net the plan phase gave
        # no lane. Measured on the K35 batch the ride-judge reverted: the
        # ride model said +60 mm, the copper +19 mm (jcr vs jc), and the
        # batch was worth 5.4 vias net under the rule.
        length = 0.0
        for nm, m in choice.items():
            pts = bp.get(nm, {}).get('lane')
            if pts:
                length += sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))
            else:
                length += pe.sm.ride_mm({nm: m}, st['launch'], st['dboxes'], st['sgrid'].bbox)
            length += pe.sm._length(m)
        ride = length / pe.sm.VIA_MM
    pp, _bad = pair_penalty(st, choice)
    if PLAN_JUDGE:
        # THE PLAN item 1: the braid's plan-implied COUNT is the cost -- plus
        # the ride round both arrays at VIA_MM (Andy, 2026-09-15: length at
        # 7.5 mm per via EVERYWHERE), the one term that sees a far-face tooth
        # (K35 SDQ13: 13 mm out and 13 mm back for a ball on U1's east
        # column). PLAN_JUDGE_RIDE=0 = the count alone (the jc arm).
        return sum(pred.values()) + (ride if PLAN_JUDGE_RIDE else 0.0) + pp, pred, bp, plan
    return sum(pred.values()) + ride + pp, pred, bp, plan


def pf_key(choice, bp, cost, model_vias=None):
    """The KEY a realize-and-confirm site compares plans on: (residue,
    cost) as recorded -- under PLAN_PAGES the cost is the CP-SAT's own
    via count `model_vias` where the caller has one -- or, under
    PLAN_JUDGE, (the braid's count, residue): the count decides, the
    residue is the tie-break and the completion guard."""
    resid = sum(1 for nm in choice if bp.get(nm, {}).get('page') is None)
    if PLAN_JUDGE:
        return (cost, resid)
    return (resid, model_vias if model_vias is not None else cost)


def pf_better(new, best):
    """Is key `new` better than `best`? Lexicographic."""
    return new < best


def pf_fmt(k0, k1):
    """`judged residue A -> B, cost X -> Y` (the recorded line), or under
    PLAN_JUDGE `judged count X -> Y, residue A -> B`."""
    if PLAN_JUDGE:
        return f'judged count {k0[0]:.0f} -> {k1[0]:.0f}, residue {k0[1]} -> {k1[1]}'
    return f'judged residue {k0[0]} -> {k1[0]}, cost {k0[1]:.2f} -> {k1[1]:.2f}'


SRC_RESIDUE_ROUNDS = int(os.environ.get('SRC_RESIDUE_ROUNDS', '8'))   # one tooth realized -> re-chosen, at most this often a round


def total(dst_c, st, cache, buses=None):
    """What the plan is judged on (plan_ends.judged_cost): the vias the
    plan's own model implies, both escapes' vias included, plus the ride
    round both arrays at VIA_MM per via -- keepers judged within the
    corridors the braid will form (planned_buses)."""
    return pe.judged_cost(dst_c, st['launch'], st['dboxes'], cache,
                          st['sgrid'].bbox, st['tooth0'], st['tooth_vias'],
                          buses if buses is not None else planned_buses(st, dst_c),
                          chi=st['chi'])


def dest_choice(st, board, log=print, fixed=None, learned=None, src_out=None):
    """The destination choice on a plan state: the greedy selector, then
    the pages-first planner over it (the greedy's move stays the fallback
    for a net the planner leaves out). ONE function for the first plan and
    for every re-plan after a refused berth. Returns (choice, unplaced)."""
    pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y)
            for nm in st['dst_pad']}
    choice, un = pe.sm.select(st['dmenu'], st['launch'],
                              keep_out=st['dboxes'], buses=st['buses'],
                              tooth_layer=st['tooth0'], log=None, pads=pads, chi=st['chi'])
    if choice and PLAN_PAGES:
        import pages_first
        # the destination re-plan loop realizes no source move: there the
        # tooth as it stands is the only source candidate (src_free False)
        pf_choice, pf_src, rep = pages_first.choose(st, board, log=log or (lambda *a: None),
                                                    fixed=fixed, learned=learned,
                                                    src_free=(src_out is not None), seed=choice)
        for ln in rep:
            (log or (lambda *a: None))(ln)
        if pf_choice:
            for nm, mv in choice.items():
                pf_choice.setdefault(nm, mv)
            un = [nm for nm in un if nm not in pf_choice]
            choice = pf_choice
            if src_out is not None and pf_src:
                src_out.update(pf_src)
    if choice and fixed:
        # the berths a caller holds fixed (a re-plan on a realized source
        # board keeps the previous choice) bind the greedy's answer too
        for nm, sig in fixed.items():
            m = next((mm for mm in st['dmenu'].get(nm, ()) if sr.move_sig(mm) == sig), None)
            if m is not None:
                choice[nm] = m
    return choice, un


# SRC_REFAN_JOINT=1 (2026-09-11): the JOINT SOURCE RE-FAN. A tooth the
# planner wants is usually blocked by NEIGHBOURING escapes that are already
# laid, and a one-net re-fan cannot move them -- to that call they are
# foreign copper, so the engine degrades the ask down its ladder and hands
# back the escape the net already had (K41: SA11 asked a dogbone under U1
# and got `level 3 lost ['face','gap','layer','kind'] = original`; the
# chain then graded no change and banned the move as if the plan were
# infeasible). Inside ONE call `underpad._follow_plan` can rip and re-lay a
# blocker around the ask. So: name the blockers (source_realize.blockers_of
# -- the nets whose copper stands in the room the ask's legs and via site
# need), strip them WITH the chosen tooth, and let the engine re-lay the
# region. Blockers outside the run are reported and cannot be moved (the
# K35 climb was walled by SA14, a net not in the run). 0 = off, the
# one-net re-fan as before.
SRC_REFAN_JOINT = int(os.environ.get('SRC_REFAN_JOINT', '0'))
# cap on how many blockers may be re-fanned with one tooth: the region the
# engine is asked to re-solve, not the whole array
SRC_REFAN_MAX = int(os.environ.get('SRC_REFAN_MAX', '6'))


def _st_with_src(st, nm, m):
    """`st` as it would be if net `nm` launched from move `m` -- the launch
    point, the tooth layer, its via count and its escape direction. Shallow
    copies only: the planner reads these four and nothing writes them."""
    st2 = dict(st)
    st2['launch'] = dict(st['launch']); st2['launch'][nm] = tuple(m.exit_pt)
    st2['tooth0'] = dict(st['tooth0']); st2['tooth0'][nm] = m.layer
    st2['tooth_vias'] = dict(st['tooth_vias']); st2['tooth_vias'][nm] = m.vias
    st2['src_over'] = dict(st.get('src_over') or {})
    st2['src_over'][nm] = {'dir': DIRS[m.direction]}
    return st2


def _blockers_for(board_pcb, moves, st, pool, log=print, label='joint re-fan'):
    """The nets of `pool` whose copper stands in the room `moves` need
    (`source_realize.blockers_of`), capped at SRC_REFAN_MAX and reported."""
    free, pinned = [], set()
    for nm, mv in moves.items():
        mov, pin = sr.blockers_of(board_pcb, mv, st['byname'][nm][0], st['byname'], pool)
        free += [b for b in mov if b not in free]
        pinned |= set(pin)
    if len(free) > SRC_REFAN_MAX:
        log(f'    {label}: {len(free)} blocker(s), capping at {SRC_REFAN_MAX}: '
            f'{free[SRC_REFAN_MAX:]} left in place')
        free = free[:SRC_REFAN_MAX]
    log(f'    {label}: blockers of {sorted(moves)} = {free or "none"}'
        + (f'; PINNED (outside the run, immovable): {sorted(pinned)}' if pinned else ''))
    return free


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
        src_out = {}
        dst_choice, un = dest_choice(st, board, src_out=src_out)
        if not dst_choice:
            print(f'  round {r}: no destination choice'); break
        if src_out and PLAN_PAGES:
            # the choice solve asked for source moves: realize them with the
            # engine, choose the destination again on the new board, and
            # KEEP the new board only if the full judge (residue, then
            # cost) says it is better -- the choice solve's model is the
            # held-lines one, and a realized tooth changes every lane's
            # neighbours (K35: three unconfirmed rounds took the judged
            # cost 141 -> 147). A rejected round's moves are banned.
            def _key(ch):
                f_, _p, bp_, _pl = judge_by_braid(st, ch, board)
                mv = None
                if PLAN_PAGES and not PLAN_JUDGE:
                    # the planner's objective: the braid's residue (exact
                    # pages), then the pages-first model's vias of the plan
                    # just chosen -- not the old judge's ride-priced cost,
                    # which reverted a batch of teeth this plan needed
                    # (PLAN_JUDGE: the braid's count, pf_key)
                    import pages_first
                    mv = getattr(pages_first.choose, 'last', {}).get('vias', f_)
                return pf_key(ch, bp_, f_, mv)
            best_key = _key(dst_choice)
            for _k in range(SRC_RESIDUE_ROUNDS):
                new_board = f'{work}_srcres{r}_{_k}.kicad_pcb'
                # the moves IN THE PLAN'S ORDER: it is the order
                # `_blockers_for` walks (and its `free` list is capped) and
                # the order the engine receives its hints in, so a rebuilt
                # dict gives the engine a different call and different
                # copper (measured at K35: 69 segments different, judged 198
                # against 181)
                rest = dict(src_out)
                _free = []
                if SRC_REFAN_JOINT:
                    _free = _blockers_for(parse_kicad_pcb(board), rest, st,
                                          set(names) - set(rest), log=print)
                res_r = sr.realize(board, rest, st['src_pad'], st['byname'],
                                   st['sref'], new_board, guard_names=names,
                                   free=_free)
                realized.append(res_r)
                misses = [nm for nm, e in res_r['audit'].items() if not e['exact']]
                for nm in misses:
                    banned.add((nm, sr.move_sig(rest[nm])))
                src_out = dict(rest)
                line = (f'  round {r}: source residue move(s) realized: {sorted(src_out)}'
                        + (f'; not laid as asked (banned): {misses}' if misses else '')
                        + (f'; REJECTED ({res_r["rejected"]})' if res_r['rejected'] else ''))
                if res_r['rejected']:
                    print(line)
                    for nm in src_out:
                        banned.add((nm, sr.move_sig(src_out[nm])))
                    break
                st2 = plan_state(parse_kicad_pcb(new_board), names, banned)
                src2 = {}
                # the destination re-chosen FROM THE PREVIOUS CHOICE: every
                # berth but the moved net's is handed to the planner as
                # fixed (a one-move menu), so the choice solve frees only
                # what it decides to. A re-plan from scratch re-decided
                # every berth on the new board and the whole-plan comparison
                # then charged the tooth for the greedy's other 20 changes
                # (K41: SA1's realized tooth, laid exactly and scheduled by
                # the plain judge, graded 14 -> 16 residue against a
                # from-scratch re-plan)
                keep_sig = {nm: sr.move_sig(m) for nm, m in dst_choice.items()
                            if nm not in src_out and nm in st2['dmenu']
                            and any(sr.move_sig(mm) == sr.move_sig(m) for mm in st2['dmenu'][nm])}
                ch2, un2 = dest_choice(st2, new_board, src_out=src2, fixed=keep_sig)
                if not ch2:
                    print(line + '; no destination choice on the new board -- reverted'); break
                f2, _p2, bp2, _pl2 = judge_by_braid(st2, ch2, new_board)
                mv2 = None
                if PLAN_PAGES and not PLAN_JUDGE:
                    import pages_first
                    mv2 = getattr(pages_first.choose, 'last', {}).get('vias', f2)
                key2 = pf_key(ch2, bp2, f2, mv2)
                if pf_better(key2, best_key):
                    print(line + f'; {pf_fmt(best_key, key2)}: KEPT')
                    board, st, dst_choice, un, best_key, src_out = new_board, st2, ch2, un2, key2, src2
                else:
                    print(line + f'; {pf_fmt(best_key, key2)}: '
                          f'not better -- reverted, moves banned')
                    for nm in src_out:
                        banned.add((nm, sr.move_sig(src_out[nm])))
                    break
                if not src_out:
                    break
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
        if PLAN_PAGES:
            # the pages-first planner chose the source itself (realized and
            # confirmed above); the paper refinement is the old planner's
            break
        sub = {n: ms for n, ms in st['smenu'].items() if n in dst_choice and ms}
        if not sub:
            break
        src_choice, _nxt, sf = pe.refine_source({}, sub, dst_choice,
                                                st['dboxes'], st['launch'],
                                                cache=cache,
                                                src_box=st['sgrid'].bbox,
                                                tooth_layer0=st['tooth0'],
                                                tooth_vias0=st['tooth_vias'],
                                                buses=pb, chi=st['chi'])
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
                  + (f'  changes {bp[nm]["changes"]}'
                     if bp[nm].get('changes') is not None else '')
                  + (f'  cross-corridor dives {bp[nm]["cross_vias"] // 2}'
                     if bp[nm].get('cross_vias') else ''))
    print(f'  plan model total predicted vias: {sum(pred.values())} over {len(pred)} nets '
          + (f'(PLAN_JUDGE={PLAN_JUDGE}: the braid\'s count {cost:.0f})' if PLAN_JUDGE else
             f'(braid-judged cost {cost:.2f} incl. ride)'))
    if out_path:
        side = os.path.splitext(out_path)[0] + '.plan.json'
        if PLAN_PAGES:
            plan['pages_first'] = True      # the braid stage pages this plan EXACTLY
        with open(side, 'w', encoding='utf-8') as f:
            json.dump(plan, f, indent=1, sort_keys=True)
        print(f'  plan written to {os.path.basename(side)}')


def main():
    if len(sys.argv) < 2 or sys.argv[1].startswith('-'):
        # The first argument is the OUTPUT tag, not a flag: `--help` once
        # ran a round and wrote --help_src1.kicad_pcb into the caller's
        # directory (the #937 registry probes every script that way).
        print(__doc__.strip().splitlines()[0])
        print('usage: python3 awx/fanout_from_plan.py OUT_TAG [BASE=...] [K=...] '
              '(see the module docstring; a tag beginning with "-" is refused)')
        sys.exit(2)
    out_path = sys.argv[1]
    rest = [a for a in sys.argv[2:] if not a.startswith('-')]
    K = int(rest[0]) if rest else 21
    base = next((a.split('=', 1)[1] for a in sys.argv
                 if a.startswith('--board=')),
                os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb'))
    # THE DESIGN CONSTANTS (rules.py), installed once per stage-process --
    # inert today, and the seam for a supplied geometry (see rules.py).
    _r = _rules.install_defaults()
    # printed from the MODULES THIS STAGE READS, never from the Rules object
    # (see braid.main: a print of the Rules cannot tell you the install
    # reached anything).
    print(f'rules: clearance {te.SPEC_CLEARANCE} (hug {te.CLEAR}), '
          f'track {te.TRACK}, fanout {sr.FAN_TRACK}/{sr.FAN_CLEAR}, '
          f'via {te.VIA_SIZE}/{te.VIA_DRILL}  [{_r.source}]')
    names = coherent_nets(K, base)
    print('planning (source realized every round)...')
    work = out_path[:-len('.kicad_pcb')] if out_path.endswith('.kicad_pcb') else out_path
    choice, dst_pad, dref, byname, board, realized, banned = plan(base, names, work)
    return fanout_destination(out_path, names, choice, dst_pad, dref, byname, board, realized, banned)


PAIR_EXIT_REACH = float(os.environ.get('PLAN_PAIR_EXIT_REACH', '1.2') or 0)


def pair_exit_clear(pcb, nid, other, m, reach=None):
    """A pair leg's move has ROOM FOR THE PAIR at its exit (2026-09-20):
    the ray from its exit point along its escape direction, `reach` mm
    (the pose router's setback ladder reaches 1.09), is clear of static
    foreign copper on the move's layer at the leg's own line and a pair
    pitch either side of it -- where the partner leg may run
    (braid.build_obstacles: every foreign pad, segment and via, inflated
    by clearance and half a track; the partner is not foreign). A single
    lane's exit is checked by the engine only to the tooth's tip, and a
    pair's pose needs a millimetre more: on the zynq bench C105, a
    back-side capacitor 0.6 mm in front of DQS0's tooth, refused every
    pose at every setback."""
    import pairs as _pairs
    if reach is None:
        reach = PAIR_EXIT_REACH
    if reach <= 0 or getattr(m, 'exit_pt', None) is None:
        return True
    mp = te.build_obstacles(pcb, nid, {nid, other}, m.layer)
    d = DIRS.get(m.direction)
    if d is None:
        return True
    n = (-d[1], d[0])
    pitch = _pairs.pitch(te.TRACK)
    # the leg's own line must be clear, and the partner's line on ONE side
    # of it (either): a leg running along the array's edge has the balls
    # on one side and room on the other, which is where the partner goes
    side_ok = {1: True, -1: True}
    k = 1
    while k * 0.1 <= reach + 1e-9:
        x = m.exit_pt[0] + d[0] * 0.1 * k
        y = m.exit_pt[1] + d[1] * 0.1 * k
        if mp.point_violation((x, y)):
            if os.environ.get('PLAN_PAIR_EXIT_DEBUG'):
                print(f'    exit blocked: {m.net} {m.kind}/{m.direction}/{m.layer[0]} at {0.1 * k:.1f} mm on its own line')
            return False
        for sg in (1, -1):
            if side_ok[sg] and mp.point_violation((x + n[0] * pitch * sg, y + n[1] * pitch * sg)):
                side_ok[sg] = False
        if not (side_ok[1] or side_ok[-1]):
            if os.environ.get('PLAN_PAIR_EXIT_DEBUG'):
                print(f'    exit blocked: {m.net} {m.kind}/{m.direction}/{m.layer[0]} at {0.1 * k:.1f} mm on both sides')
            return False
        k += 1
    return True


def tie_vias_under(pcb, nms, byname, dst_pad, vias_add, log=print):
    """A TIE VIA for every destination ball with a pad of its OWN net under
    it on the other layer (a back-side termination under a DDR clock ball,
    pairs.under_pad): the escape is whatever the plan chose; the pad is
    served by a via at the ball -- nudged toward the pad's centre within
    the ball's own slack, so the barrel's far annulus lies in the pad's
    copper -- unless a via of the net already stands in the ball (a
    via-in-pad escape ties it by itself). Checked clean of foreign pads
    on either layer and of every drill (hole to hole) before it is added;
    a graze leaves the pad open and says so. Returns via dicts for
    add_tracks_and_vias_to_pcb."""
    import pairs as _pairs
    from routing_defaults import HOLE_TO_HOLE_CLEARANCE
    out = []
    for nm in nms:
        nid, net = byname[nm]
        ball = dst_pad.get(nm)
        if ball is None:
            continue
        under = [q for q in net.pads if _pairs.under_pad(ball, q, te.VIA_SIZE)]
        if not under:
            continue
        r_ball = min(ball.size_x, ball.size_y) / 2
        bx, by = ball.global_x, ball.global_y
        if any(v['net_id'] == nid and math.hypot(v['x'] - bx, v['y'] - by) <= r_ball for v in (vias_add or [])) \
                or any(v.net_id == nid and math.hypot(v.x - bx, v.y - by) <= r_ball for v in pcb.vias):
            continue
        q = under[0]
        slack = max(r_ball - te.VIA_SIZE / 2, 0.0)
        dx, dy = q.global_x - bx, q.global_y - by
        d = math.hypot(dx, dy)
        x, y = bx, by
        if d > 1e-9:
            m = min(slack, d)
            x, y = bx + dx / d * m, by + dy / d * m
        need = te.VIA_SIZE / 2 + sr.FAN_CLEAR
        bad = None
        for fp in pcb.footprints.values():
            for p in fp.pads:
                if p.net_id == nid:
                    continue
                if p.pad_type == 'np_thru_hole' or p.drill > 0:
                    if math.hypot(p.global_x - x, p.global_y - y) < (p.drill + te.VIA_DRILL) / 2 + HOLE_TO_HOLE_CLEARANCE - 1e-6:
                        bad = f'the hole of {fp.reference}.{p.pad_number}'
                        break
                if p.pad_type == 'np_thru_hole':
                    continue
                ex = max(abs(p.global_x - x) - p.size_x / 2, 0.0)
                ey = max(abs(p.global_y - y) - p.size_y / 2, 0.0)
                if math.hypot(ex, ey) < need - 1e-6:
                    bad = f'pad {fp.reference}.{p.pad_number}'
                    break
            if bad:
                break
        if not bad:
            for v in list(pcb.vias) + list(vias_add or []):
                vx, vy = (v.x, v.y) if hasattr(v, 'x') else (v['x'], v['y'])
                vn = v.net_id if hasattr(v, 'net_id') else v['net_id']
                vs = v.size if hasattr(v, 'size') else v['size']
                vd = v.drill if hasattr(v, 'drill') else v['drill']
                dd = math.hypot(vx - x, vy - y)
                if dd < 1e-6:
                    continue
                if vn != nid and dd < need + vs / 2 - 1e-6:
                    bad = 'a via'
                    break
                if dd < (vd + te.VIA_DRILL) / 2 + HOLE_TO_HOLE_CLEARANCE - 1e-6:
                    bad = 'a drill'
                    break
        if bad:
            log(f'  {nm}: {q.component_ref}.{q.pad_number} lies under the ball, but a tie via there '
                f'grazes {bad} -- left open')
            continue
        out.append({'x': round(x, 4), 'y': round(y, 4), 'size': te.VIA_SIZE, 'drill': te.VIA_DRILL,
                    'layers': list(LAYERS), 'net_id': nid})
        log(f'  {nm}: tie via at ({x:.2f},{y:.2f}) serves {q.component_ref}.{q.pad_number} under the ball')
    return out


def fanout_destination(out_path, names, choice, dst_pad, dref, byname, board,
                       realized, banned):
    """Fan out DU1 to the plan, audit, and FEED BACK: a berth the engine
    could not lay as asked leaves that net's menu, the destination is
    re-selected against the same teeth, and the fanout runs again --
    until every berth is exactly the plan's, or nothing changes. The
    plan's own via model is printed for the plan that ships."""
    _pcb0 = parse_kicad_pcb(board)
    st = plan_state(_pcb0, names, banned)
    laid_pass = None       # the LAST pass fanned out: (choice, st, achieved, ok)
    learned = set()        # move pairs the planner must avoid together (none are learned today)
    # PAIR BERTHS (pairs.harmonise, PLAN_PAIRS): a differential pair's two
    # berths are made one move -- same face, layer and kind, neighbouring
    # exits -- before the engine lays them, every pass. Off unless the
    # braid routes pairs as members (BRAID_PAIRS), so the chain is
    # byte-identical without it; inert on a run with no pairs.
    _harm = int(os.environ.get('PLAN_PAIRS', os.environ.get('BRAID_PAIRS', '0')) or 0)
    _pitch = 0.0
    if _harm:
        import pairs as _pairs
        _g = em.grid_of(_pcb0.footprints[dref])
        _pitch = max(_g.pitch_x, _g.pitch_y)
    for it in range(DST_ITERS):
        if _harm:
            _pairs.harmonise(choice, st['dmenu'], names, _pitch, pe.sm._conflict, print)
        faces = [m.direction for m in choice.values()]
        print(f'\nplan (destination pass {it}): {len(choice)} berth escape directions '
              + ', '.join(f'{d}:{faces.count(d)}' for d in sorted(set(faces)))
              + f'  (source board: {os.path.basename(board)}, '
              f'{len(realized)} realized round(s), {len(banned)} banned move(s))')
        laid, audit_d, ok = fanout_once(out_path, names, choice, dst_pad, dref,
                                        byname, board)
        laid_pass = (dict(choice), st, getattr(fanout_once, 'achieved', None), ok)
        misses = [nm for nm in choice if not audit_d.get(nm, {}).get('exact')]
        drc_nets = [nm for nm in sorted(getattr(fanout_once, 'drc_nets', ()))
                    if nm in choice and nm not in misses]
        if drc_nets:
            print(f'  destination pass {it}: {len(drc_nets)} berth(s) laid as asked but in a '
                  f'DRC violation -> treated as refused: {drc_nets}')
            misses += drc_nets
        if not misses:
            print(f'  destination pass {it}: every berth laid as planned')
            break
        for nm in misses:
            m = choice[nm]
            banned.add((nm, sr.move_sig(m)))
        print(f'  destination pass {it}: {len(misses)} berth(s) not laid as asked '
              f'-> banned, re-planning the destination: {misses}')
        st = plan_state(parse_kicad_pcb(board), names, banned)
        # the berths laid exactly stay as laid
        laid_ok = {nm: sr.move_sig(choice[nm]) for nm in choice
                   if audit_d.get(nm, {}).get('exact')}
        # pages-first: the berths laid exactly stay FIXED, only the missed
        # nets are re-planned -- a re-plan from scratch asked for 5-6 new
        # berths every pass and never converged (K28, 8 passes, 27 bans)
        new_choice, un = dest_choice(st, board,
                                     fixed=laid_ok if PLAN_PAGES else None,
                                     learned=learned)
        if not new_choice or new_choice == choice:
            print('  destination: the re-plan changed nothing -- stopping')
            break
        f, _p, _bp, _pl = judge_by_braid(st, new_choice, board)
        print(f'  destination re-plan: braid-judged {f:.2f}, {len(new_choice)} placed'
              + (f', {len(un)} unplaced' if un else ''))
        choice, dst_pad = new_choice, st['dst_pad']
    # The LAST PASS SHIPS, and its sidecar is its own: the choice that was
    # fanned out and audited, never the re-plan after it (which is a
    # choice no board was laid to). Measured over every K41 pass board
    # (2026-09-07, the braid on each): passes 0..7 graded 6/3/4/5/3/1/1/1
    # open at 86/108/92/74/81/98/102/78 vias -- the last pass best, and
    # neither the audit's exact count (pass 5: 38/40 vs 34/40, 1 open at
    # 98) nor the judge's cost (pass 6 the lowest, 1 open at 102 with 6
    # DRC) picks a better one. Judged on the WRITTEN fanout board: the
    # same copper the braid will read, so its taut paths are the memo's.
    choice_l, st_l, achieved_l, ok_l = laid_pass
    explain_plan(choice_l, st_l, names, out_path, out_path, achieved=achieved_l)
    # TIE VIAS for the pads under balls, on the SHIPPED board and only
    # there: inside the loop a barrel in the ball reads to the audit as a
    # via-in-pad berth that was never asked, and the pass bans and
    # re-plans the berth every time (K36 pf5: eight passes, the SCK pair's
    # berths driven 7 mm apart)
    try:
        pcb_f = parse_kicad_pcb(out_path)
        ties = tie_vias_under(pcb_f, list(names), byname, st_l['dst_pad'] if isinstance(st_l, dict) and 'dst_pad' in st_l else dst_pad, [])
        if ties:
            tmp = out_path[:-len('.kicad_pcb')] + '_tie.kicad_pcb'
            add_tracks_and_vias_to_pcb(out_path, tmp, [], ties, [],
                                       net_id_to_name={i: n.name for i, n in pcb_f.nets.items()})
            os.replace(tmp, out_path)
    except Exception as e:      # a tie must not lose the board
        print(f'  tie vias NOT added: {e}')
    return 0 if ok_l else 1


def fanout_once(out_path, names, choice, dst_pad, dref, byname, board,
                relay=None, already=(), face_asks=None):
    """One destination fanout to `choice`, written to out_path and audited.
    `relay` None: every net of the run is fanned out from `board` (the
    source board, its destination bare). Otherwise an INCREMENTAL pass
    (2026-09-09): the previous pass's board (out_path as it stands) is the
    base, only the `relay` nets' destination copper is stripped and they
    alone are re-fanned against everything else's -- a berth the engine
    laid exactly is copper the next pass cannot displace. Re-fanning the
    whole array from bare each pass laid a neighbour's new ask ahead of a
    frozen berth (the engine claims deepest first) and the loop then
    banned the frozen berth's class: K41 misses 7/6/7/4/3/1/1/2, K51
    9/4/3/1/2/1/1/1, nearly every late miss a berth exact the pass before.
    `already`: nets with destination copper from earlier passes.
    `face_asks` {net: face}: a net asked for a FACE only (the engine's bare
    hint -- its own search picks the gap, layer and kind on that face), for a
    net whose menu of straight escapes is empty on an occupied board
    (reberth.py); measured like an unplanned net, not audited.
    Returns (laid nets, audit dict, clean-and-complete)."""
    if relay is None:
        targets = list(names)
        pcb = parse_kicad_pcb(board)
        src_file = board
    else:
        targets = list(relay)
        prev = out_path[:-len('.kicad_pcb')] + '.prev.kicad_pcb'
        shutil.copy(out_path, prev)
        pcb = parse_kicad_pcb(prev)
        n2n = {i: n.name for i, n in pcb.nets.items()}
        x0, y0, x1, y1 = em.grid_of(pcb.footprints[dref]).bbox
        x0, y0, x1, y1 = x0 - 2.0, y0 - 2.0, x1 + 2.0, y1 + 2.0
        nids = {byname[nm][0] for nm in targets}
        segs = [s for s in pcb.segments if s.net_id in nids
                and x0 <= min(s.start_x, s.end_x) and max(s.start_x, s.end_x) <= x1
                and y0 <= min(s.start_y, s.end_y) and max(s.start_y, s.end_y) <= y1]
        vias = [v for v in pcb.vias if v.net_id in nids
                and x0 <= v.x <= x1 and y0 <= v.y <= y1]
        content = open(prev, encoding='utf-8').read()
        content, n_s = sr.remove_segments_from_content(content, segs, n2n)
        content, n_v = sr.remove_vias_from_content(content, vias, n2n)
        if n_s != len(segs) or n_v != len(vias):
            print(f'  destination re-lay: WARNING strip matched {n_s}/{len(segs)} '
                  f'segments, {n_v}/{len(vias)} vias')
        # the engine's own view: those nets bare at the destination (their
        # source stubs, 7 mm away, are the same net and no obstacle)
        rm_s, rm_v = set(map(id, segs)), set(map(id, vias))
        pcb.segments = [s for s in pcb.segments if id(s) not in rm_s]
        pcb.vias = [v for v in pcb.vias if id(v) not in rm_v]
        src_file = out_path[:-len('.kicad_pcb')] + '.stripped.tmp'
        with open(src_file, 'w', encoding='utf-8') as f:
            f.write(content)
    hints = {}
    for nm in targets:
        if nm in choice:
            p = dst_pad[nm]
            hints[(round(p.global_x, 3), round(p.global_y, 3))] = sr.full_move(choice[nm])
            if PLAN_PAGES:
                # STRICT plan-follow (underpad._follow_plan): a negotiation
                # is kept only when the count of exact berths rises, and a
                # ball with no berth on its asked face is left unescaped
                # for the planner's re-plan, never dumped on another face
                hints[(round(p.global_x, 3), round(p.global_y, 3))]['strict'] = True
        elif face_asks and nm in face_asks:
            p = dst_pad[nm]
            # a face-only ask (replan.py): the engine escapes the ball in
            # its generic Phase A, before the planned balls
            hints[(round(p.global_x, 3), round(p.global_y, 3))] = face_asks[nm]
    # the production engine, following the FULL planned moves (face, exit
    # gap, layer, kind), with the VIA the plan priced its moves with (the
    # braid's 0.25/0.15; a 0.45 via cannot sit in a 0.65 mm pitch gap).
    # The under-pad engine is the one that follows a plan (its plan-follow
    # phase; 'auto' would let the channel engine take the face and choose
    # the rest itself). No plane-drop pass (it collides with the decoupling
    # caps under the array, a defect of that pass, not of anything here).
    # No placement step follows this chain, so every foreign pad is one a
    # via must clear.
    pcb._fanout_all_foreign_immovable = True
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[dref], pcb, net_filter=targets, layers=list(LAYERS),
        track_width=sr.FAN_TRACK, clearance=sr.FAN_CLEAR, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
        exit_margin=0.5, escape_method='underpad', plane_drop='off',
        escape_dir_hints=hints)
    if tracks:
        add_tracks_and_vias_to_pcb(
            src_file, out_path, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb.nets.items()})
    else:
        shutil.copy(src_file, out_path)
    if relay is not None:
        os.remove(src_file)
    copy_pro(board, out_path)
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        out_path, '--clearance', '0.1',
                        '--clearance-margin', '0.1',
                        # or check_drc truncates each category at 20 and the
                        # nets beyond that are never banned, never freed
                        '--max-print', '0'],
                       capture_output=True, text=True)
    _drc_txt = r.stdout + r.stderr
    clean = 'NO DRC VIOLATIONS' in _drc_txt
    if not clean and 'DRC VIOLATION' not in _drc_txt:
        raise RuntimeError(f'check_drc gave no verdict for {out_path} '
                           f'(exit {r.returncode}): '
                           + ((_drc_txt.strip().splitlines() or ['(no output)'])[-1])[:200])
    # the nets of every violation the fanout board ships (check_drc names
    # the pair on a line of its own): the loop treats them as berths not
    # laid as asked, or a pass with every berth "exact" and a crossing
    # between two of them converges on a broken board (K8, K35 2026-09-11)
    import re as _re
    drc_nets = set()
    drc_pairs = set()       # the PAIRS too
    # check_drc prints the two sides as `Kind:/NET` with a SUFFIX on some
    # forms -- `Pad:/NET (REF.PAD)`, `... [SHORT]`, `Via:/NET (drill hole
    # clearance)`. Taking the whole side and splitting on '/' recovered a
    # clean name only for seg-seg and via-via, so EVERY pad violation --
    # including a pad-pad SHORT, the commonest BGA-fanout defect -- fell
    # out of the feedback, and the loop could print "every berth laid as
    # planned" on a shorted board. Strip the kind prefix and everything
    # from the first space or bracket.
    def _net_of(side):
        # strip the kind prefix and the TRAILING annotations only -- never
        # split on whitespace: this board's nets are `/DDR3 16x1/SDQ2`, so
        # a space split yields `DDR3`
        side = side.strip()
        if ':' in side[:6]:
            side = side.split(':', 1)[1]
        for _ in range(3):
            side = _re.sub(r'\s*\[[^\]]*\]\s*$', '', side)
            side = _re.sub(r'\s*\([^)]*\)\s*$', '', side)
        return side.strip().split('/')[-1]
    for a, b in _re.findall(r'^\s+(.+?) <-> (.+?)\s*$', _drc_txt, flags=_re.M):
        a, b = _net_of(a), _net_of(b)
        if not a or not b:
            continue
        drc_nets.add(a); drc_nets.add(b)
        drc_pairs.add(frozenset((a, b)))
    fanout_once.drc_nets = drc_nets
    fanout_once.drc_pairs = drc_pairs
    # the per-tooth audit at the destination: face, layer, kind, gap and
    # ORDER, measured off the written board (source_realize.audit)
    pcb_out = parse_kicad_pcb(out_path)
    got = {t['net_id'] for t in tracks}
    have = (set(already) - set(targets)) | {nm for nm in names if byname[nm][0] in got}
    laid = [nm for nm in choice if nm in have]
    achieved = {nm: sr.measure_tooth(pcb_out, nm, dst_pad[nm], byname, dest_ref=dref)
                for nm in laid}
    audit_d, _counts = sr.audit(choice, achieved, None, laid, print, 'berth')
    # the berth of a net the plan left UNPLACED, laid by the engine's own
    # choice: measured too, so the loop can keep it (the audit above is
    # over the asked berths only)
    for nm in names:
        if nm not in choice and nm in have and nm in dst_pad:
            achieved[nm] = sr.measure_tooth(pcb_out, nm, dst_pad[nm], byname, dest_ref=dref)
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
