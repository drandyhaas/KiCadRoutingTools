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
import collections
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
import schedule as sch  # noqa: E402  -- lis_keep, for plan_lis
import source_realize as sr  # noqa: E402
from coherent_nets import coherent_nets  # noqa: E402

# SRC_CLIMB=k (2026-09-10): the SOURCE menu also offers CLIMBS -- a dog-bone
# or via-in-pad whose run first travels up to k pitches along a gap under
# the array and leaves the face at a chosen row or column (the human's
# north riders at K51; escape_moves.enumerate_moves climb=). 0 = off, the
# menu byte-identical. replan.py runs with 14.
SRC_CLIMB = int(os.environ.get('SRC_CLIMB', '0'))

# SPLIT_BLOCKS=1 (2026-09-10): a destination array whose ball grid has a
# depopulated BAND (a DDR3/DDR4 FBGA: two blocks of three ball columns
# either side of a three-pitch-wide empty street) is planned and fanned out
# as its BLOCKS (escape_moves.blocks_of): each block's menu is enumerated
# against the block's own bbox, so the band is a face of each block and a
# stub may END in it; the ride model goes round the blocks and through the
# band (select_moves.around_boxes_path); the engine is run once per block
# on a view of the board where the block is the array and the other blocks
# are a foreign part beside it. 0 = off, one array as before, byte-identical.
SPLIT_BLOCKS = int(os.environ.get('SPLIT_BLOCKS', '0'))
# the engine's exit margin past a block's boundary line for the per-block
# calls (the engine's own default is 0.5; a band stub 0.5 + half a pitch
# past its ball line eats a third of a 3.2 mm band's width)
BAND_EXIT = float(os.environ.get('BAND_EXIT', '0.5'))

# DST_WALK=k (2026-09-10): the destination menu also offers WALKED dog-bones
# -- the surface stub to a diagonal elbow and then up to k pitches along the
# lane through it (a band's edge line, a row gap) to a via site, the run on
# the other layer leaving from that site in any direction
# (escape_moves.enumerate_moves walk=). The human's DU1 (measured 2026-09-10,
# awx/tmp/human_du1.py): 24 vias inside the array, 14 in the band, the first
# via a median 1.3 mm from its ball; the via field is the destination, the
# faces are where the lanes come from. 0 = off, the menu byte-identical.
DST_WALK = int(os.environ.get('DST_WALK', '0'))

# DST_WALK_OFF=1 (2026-09-11): the walked dog-bone may take ONE step past the
# ball field's boundary, to a via site in the clear margin BESIDE the array
# (escape_moves.enumerate_moves walk_off=). The human's DU1 corner nets
# (SA13 SA14 SA15 SA6 SA11) escape exactly that way and the generator had no
# move of the class -- it broke at the edge. Needs DST_WALK > 0 to do
# anything. 0 = off, the menu byte-identical.
DST_WALK_OFF = int(os.environ.get('DST_WALK_OFF', '0'))

from escape_moves import DIRS, LAYERS  # noqa: E402,F401  -- ONE source


def copy_pro(src_board, dst_board):
    pro = os.path.splitext(src_board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst_board)[0] + '.kicad_pro')


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

    def menu(pad, grid, nid, own_only=False, climb=0, walk=0):
        return em.enumerate_moves(
            pad, grid, LAYERS,
            lambda p, q, L, _n=nid: obs(_n, L, own_only).seg_clear(p, q),
            lambda p, L, _n=nid: not (obs(_n, L, own_only).point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0],
            climb=climb, walk=walk, walk_off=DST_WALK_OFF if walk else 0)
    dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
    dref = ends[names[0]][2]
    dgrid = em.grid_of(pcb.footprints[dref])
    # the destination's BLOCKS (SPLIT_BLOCKS): each ball's menu against its
    # own block, the keep-out the list of block boxes with the band open
    dblocks = em.blocks_of(pcb.footprints[dref]) if SPLIT_BLOCKS else [dgrid]
    dboxes = [b.bbox for b in dblocks] if len(dblocks) > 1 else dgrid.bbox
    if len(dblocks) > 1:
        # where a band stub's tip stands off its ball line: half a pitch
        # (the boundary line) plus one occupancy cell -- the under-pad
        # engine ends its stubs at the boundary cell, not at exit_margin
        # (measured 0.425 at 0.8 mm pitch) -- the selector's band capacity
        # is what fits between the tip lines
        pe.sm.BAND_TIP = max(dgrid.pitch_x, dgrid.pitch_y) / 2 + 0.05
    for nm in names:
        nid, net = byname[nm]
        fp = pcb.footprints[ends[nm][2]]
        bx, by = ends[nm][1]
        pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
                  + (p.global_y - by) ** 2)
        dst_pad[nm] = pad
        moves = menu(pad, em.grid_of(fp), nid, walk=DST_WALK)
        if ends[nm][2] == dref and len(dblocks) > 1:
            # the BLOCK's moves too: its band faces are moves the whole
            # array does not have; its outer faces are the array's own
            # (same exit, deduplicated by signature); the whole array's
            # through-runs (a column gap across the band and the next
            # block to the far face) stay -- the split is a superset
            seen = {sr.move_sig(m) for m in moves}
            moves += [m for m in menu(pad, em.block_of(pad, dblocks), nid)
                      if sr.move_sig(m) not in seen]
        dmenu[nm] = [m for m in moves if (nm, sr.move_sig(m)) not in banned]
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
        smenu[nm] = [m for m in menu(p, sgrid, byname[nm][0], own_only=True, climb=SRC_CLIMB)
                     if (nm, sr.move_sig(m)) not in banned]
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
    return {'byname': byname, 'dmenu': dmenu, 'smenu': smenu, 'launch': launch,
            'tooth0': tooth0, 'tooth_vias': tooth_vias, 'src_pad': src_pad,
            'dst_pad': dst_pad, 'sref': sref, 'dref': dref, 'sgrid': sgrid,
            'bundle_layer': bundle_layer, 'chi': chi,
            'dgrid': dgrid, 'dblocks': dblocks, 'dboxes': dboxes,
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
        # a CANDIDATE TOOTH (src_over, SRC_REPLAN): its copper is not on the
        # board yet, so _end_dir would read the tooth it would REPLACE and
        # hand the braid the old escape direction
        so = (st.get('src_over') or {}).get(nm)
        plan['tooth_dir'][nm] = (list(so['dir']) if so else
                                 list(te._end_dir(pcb, nid, st['launch'][nm], net.pads)))
        plan['stub_dir'][nm] = list(DIRS[(got['direction'] if got and got.get('direction') in DIRS
                                          else m.direction)])
    return plan


def _plan_braid_worker(args):
    """One judge trial in a worker process (residue_search): the braid's
    planner on a plan dict; returns its per-net plan and the level-5
    residue sets it now holds (the parent hands them to the next trial,
    so no worker solves cold)."""
    board, names, dref, plan, seeds = args
    import braid as te_
    # START FROM EXACTLY THE PARENT'S SNAPSHOT, never from what this worker
    # happens to be holding. A pool worker is long-lived and accumulates
    # _L5_SEED across the trials it runs; `setdefault` let a trial inherit
    # whichever predecessor landed in the same worker. The seed is not
    # decoration -- _profiles5 fixes the `w` binaries to it and solves at
    # L5_JUDGE_NODES (30), so the seed IS the answer at that cap. Which
    # worker takes which trial is a wall-clock race and how many workers
    # exist was os.cpu_count()-2, so the judged via count -- and the berth
    # chosen from it -- depended on the machine. That is the defect the
    # no-clock rule exists to forbid, wearing a different hat.
    te_._L5_SEED.clear()
    for k, v in (seeds or {}).items():
        te_._L5_SEED[frozenset(k)] = set(v)
    bp = te_.plan_braid(board, names, dref, plan)
    return bp, {tuple(sorted(k)): sorted(v) for k, v in te_._L5_SEED.items()}


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


# SF_ESC_W (2026-09-12, README TODO 1b): the weight on the ESCAPE half of
# the judge -- tooth vias + berth vias + the ride. 1.0 is the judge exactly
# as it has always been; 0.0 keeps only the CORRIDOR half (the braid
# planner's own layer changes and cross-corridor dives).
# WHY THIS KNOB EXISTS. Pairwise rank agreement with the ROUTED via count,
# over the distinct plans on disk:
#     statistic                     K41    K35
#     pred (the judge as it stands)  59%    51%
#     its CORRIDOR half alone        62%    64%
#     its ESCAPE half alone          53%    41%
# The judge is WORSE THAN ITS OWN CORRIDOR HALF at both K, and the reason
# is a SIGN, not a scale: at K35 pearson(escape, LIS) = +0.57 while
# pearson(escape, routed) = -0.34. A plan that spends more escape vias has
# a longer crossing-free chain and routes BETTER -- the human's trade, a
# dogbone at the ball bought to fix the order -- and the judge charges it
# +1 per via. Sweeping corridor + w*escape at K35 goes 64% (w=0) to 51%
# (w=1). This is the reason a 51-64% comparator makes MORE SEARCH WORSE,
# and it is a level the swimmer-price work could not reach.
# Default 1.0 until the K ladder says otherwise.
SF_ESC_W = float(os.environ.get('SF_ESC_W', '1'))

# SF_ACCEPT_MARGIN (2026-09-12, README TODO 1c): how much a candidate must
# BEAT the incumbent by before the search takes it. 0 = off, the plain
# tuple compare this search has always done.
# WHY. Over the K35/K41 logs the search took 965 moves with a median
# improvement of 2.00 judged vias, 527 of 823 under 3 -- and 142 of them
# with the judged cost RISING, because the key is lexicographic on the
# residue count and a residue drop trumps any cost. A comparator right
# 51-64% of the time (see SF_ESC_W), accepting at 1e-6, is a random walk
# with a drift. That is why deleting the wall clock cost K35 58 -> 66:
# THE CLOCK WAS AN EARLY STOP, and an early stop is a crude regulariser.
# This is the same regulariser made work-free and deterministic, so it
# keeps the no-clocks rule. It also gives a free control: a margin large
# enough to accept nothing reproduces the pre-search plan exactly.
ACCEPT_MARGIN = float(os.environ.get('SF_ACCEPT_MARGIN', '0') or 0)

# SF_LIS_GUARD (2026-09-12, README TODO 1b): refuse a move that SHORTENS the
# longest crossing-free chain. 0 = off, the default.
# WHY A GUARD AND NOT A COST. LIS of launch->target is the best single
# statistic measured against the routed via count (K41 64%, K35 76%, against
# the judge's own 59%/51%) -- but the settled table is full of correlations
# that became objectives and lost, and this one has a mechanism for it: two
# pages hold at most twice the chain, so chain length is a CAPACITY, and
# buying capacity you already have costs vias for nothing (measured at K41,
# where slack is 1: DST_XING bought chain 21 -> 22 and paid 80 -> 98 vias).
# A guard spends nothing when the capacity is not binding and refuses the
# moves that eat it when it is.
# Summed PER CORRIDOR, because lanes only cross inside one.
LIS_GUARD = int(os.environ.get('SF_LIS_GUARD', '0') or 0)


def plan_lis(bp):
    """Total longest crossing-free chain of a braid plan: per corridor, the
    lanes in LAUNCH order, the LIS of their TARGET ranks. `bp` is
    braid.plan_braid's answer, which carries both indices per net."""
    by_c = {}
    for nm, d in bp.items():
        li, ti = d.get('launch_idx'), d.get('target_idx')
        if li is None or ti is None:
            continue
        by_c.setdefault(d.get('corridor'), []).append((li, ti))
    tot = 0
    for pairs in by_c.values():
        pairs.sort()
        tot += len(sch.lis_keep([t for _l, t in pairs]))
    return tot


def accept_key(k1, k0, lis1=None, lis0=None):
    """Is k1 = (residue, judged) worth taking over the incumbent k0?
    At margin 0 this is `k1 < k0` -- unchanged. Above it, a move must win
    by the margin on the judged cost, and a residue drop must no longer
    trump a cost rise of any size: it may cost at most the margin.
    `lis1`/`lis0` are the two plans' crossing-free chains (plan_lis); under
    LIS_GUARD a move that shortens the chain is refused whatever it costs."""
    if LIS_GUARD and lis1 is not None and lis0 is not None and lis1 < lis0:
        return False
    if not ACCEPT_MARGIN:
        return k1 < k0
    if k1[0] != k0[0]:
        return k1[0] < k0[0] and k1[1] <= k0[1] + ACCEPT_MARGIN
    return k1[1] < k0[1] - ACCEPT_MARGIN


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
                              changes=chg, cross=xv, swim_changes=swc)
    ride = pe.sm.ride_mm(choice, st['launch'], st['dboxes'],
                         st['sgrid'].bbox) / pe.sm.VIA_MM
    if SF_ESC_W == 1.0:
        return sum(pred.values()) + ride, pred, bp, plan
    # split pred back into its two halves. Per net vias_from_pages emits
    # tooth_vias + cross + (changes | SWIM) + m.vias, so the escape half is
    # exactly the tooth and berth vias and everything else is corridor --
    # cross-corridor dives included, which is where they belong.
    esc = (sum(st['tooth_vias'].get(n, 0) for n in choice)
           + sum(m.vias for m in choice.values()))
    cor = sum(pred.values()) - esc
    return cor + SF_ESC_W * (esc + ride), pred, bp, plan


# DST_SEARCH=1 (2026-09-10): after the greedy selection, a LOCAL SEARCH over
# the destination choice judged by the braid's own planner -- one net's
# berth at a time, the fast proxy (plan_ends.judged_cost on the seed's
# corridors) screening the candidates and the braid's planner
# (judge_by_braid, ~0.1 s warm) confirming each move. The greedy selector
# optimises its own cost, and given a richer menu (SPLIT_BLOCKS' band
# faces, short and cheap in that cost) it takes moves the braid's planner
# judges worse (K28: 98.5 against 95.6 for the whole-array menu, routed 50
# against 36 vias). 0 = off, the greedy choice ships as before.
DST_SEARCH = int(os.environ.get('DST_SEARCH', '0'))
SCHED_FIRST = int(os.environ.get('SCHED_FIRST', '0'))
# SF_FIXED=1: on a re-plan after a refusal, the berths laid exactly stay as
# laid and only the refused nets move. Converges in 3-4 passes instead of
# 8 (K41 148 s against 603 s) but locks the first pass's structure in:
# K28 46 / K35 75 / K41 113 against 41 / 60 / 84 without it (2026-09-11).
SF_FIXED = int(os.environ.get('SF_FIXED', '0'))
SF_LEARN = int(os.environ.get('SF_LEARN', '0'))   # refusals with a named occupier learned as move PAIRS
# SF_EQUIV=1 (2026-09-11): the destination laid to be EQUIVALENT to the plan
# under the planner's own judge (sched_first.judge) -- fanout_equivalent
# replaces fanout_destination: berths the judge seats are kept as laid,
# only the nets it swims (or that have no copper / a DRC pair) are re-planned
# with the kept ones fixed and re-laid alone. Flag off: the exact-gap loop.
SF_EQUIV = int(os.environ.get('SF_EQUIV', '0'))
# DST_FACE_GROUP=1 (2026-09-11): the berth LAYERS along each destination
# face re-chosen as contiguous BLOCKS (face_group_search), judged by the
# braid's planner. The residue probe showed why nets still need >2 vias
# on a faithful chain: a changer's crossings with front stayers and back
# lanes alternate along the corridor because the two families alternate
# along the faces at the ends, and a straight ribbon crosses them in that
# order. Grouped families give grouped crossings and one change a net.
DST_FACE_GROUP = int(os.environ.get('DST_FACE_GROUP', '0'))
# DST_RESIDUE=1 (2026-09-11): a berth search aimed at the braid's RESIDUE --
# the nets its schedule (BRAID_ONE_DIVE=4) cannot place and would leave to
# weave, which are the lanes refused in band. For each residue net every
# collision-free move of its menu is judged by the braid's planner; the
# move that lowers the residue count, then the judged cost, is kept;
# sweeps until nothing improves. DST_RESIDUE_CALLS caps the judge calls.
# DST_RESIDUE=2 (2026-09-11, latest): the berth choice INSIDE the braid's
# level-5 solve (residue_choice, braid._alts5) -- every collision-free
# move of every residue net is handed to the planner as a candidate
# berth, each a lane of its own in ONE MILP whose rows a choice binary
# gates, the other lanes held; the solve picks the berths jointly and
# the full judge confirms them. A pass is one solve and two or three
# judge calls instead of 40-65 trials.
# DST_RESIDUE=3: the JOINT ORDER -- every net is offered its moves,
# occupied berths included (a candidate may take a neighbour's berth
# only if that neighbour moves: an exclusion against the occupant's
# laid berth), so the solve can reorder the targets; the residue at
# K35 is structural (the launch-vs-target permutation needs eight
# crossing-free pages) and no residue net's own free move fixes it.
DST_RESIDUE = int(os.environ.get('DST_RESIDUE', '0'))
# a cap in JUDGE CALLS (see PLAN_CALLS). Generous: the loops end on their
# own sweeps, so this only stops a runaway -- and it stops it at the same
# place on every machine.
DST_RESIDUE_CALLS = int(os.environ.get('DST_RESIDUE_CALLS', '20000'))
# DST_RESIDUE=3's screen: per net the cheapest move on EVERY face it can
# leave by (the face is what moves a net along the target order, the gap
# only fine-tunes it), then the rest filled to DST_RESIDUE_CANDS with
# the faces NEAR the berth it holds served first -- the move of the
# nearest face, then one of the next face, round again -- since a gap
# or a layer along the face a net already uses is the fine-tuning the
# solve wants most, a far face the one it wants rarely. Every move of
# every net (K35: ~10 a net) built a 100k-row instance that took a
# gigabyte before it solved; and the four cheapest by cost alone dropped
# the one face that reorders (K35's address nets: teeth at U1's south
# end, balls in DU1's north rows, the order-consistent berth a
# through-run to the SOUTH face at a via, undercut by the free surface
# exits north and west).
DST_RESIDUE_CANDS = int(os.environ.get('DST_RESIDUE_CANDS', '8'))
# DST_RESIDUE_POOL: which nets are offered candidates under DST_RESIDUE=3.
# 'displaced' (default): the residue nets, and every net whose laid berth
# a residue net's candidate would take (so the occupant can step aside);
# 'all': every net (K35: 140 candidates, 112k rows, stage A 100 s at its
# node limit -- the joint instance is past what solves at the root).
DST_RESIDUE_POOL = os.environ.get('DST_RESIDUE_POOL', 'displaced')
# DST_RESIDUE_SRC=1 (2026-09-11, latest): the residue nets' SOURCE moves
# are candidates too -- a candidate TOOTH is a lane with a different
# launch end in the same solve (braid._alt_geo 'src'), costed by its own
# vias against the tooth as laid. A chosen tooth is REALIZED with the
# engine (source_realize) and the plan re-read off the new board, since
# the launch order is what the eight persistent K35 residue nets need:
# the human peels them out U1's east face on B (teeth in the north half)
# where ours leave the south end, and no destination berth reorders that.
DST_RESIDUE_SRC = int(os.environ.get('DST_RESIDUE_SRC', '0'))
SRC_RESIDUE_CANDS = int(os.environ.get('SRC_RESIDUE_CANDS', '8'))
SRC_RESIDUE_ROUNDS = int(os.environ.get('SRC_RESIDUE_ROUNDS', '8'))   # one tooth realized -> re-chosen, at most this often a round
# DST_RESIDUE_WORKERS (2026-09-11): one net's candidate moves are judged in
# parallel worker processes -- the trials of a net are independent (the
# greedy accepts one per net, in order, as before), a trial is ~3.3 s of
# plan_braid at K41 and a pass is 40-65 of them, the slowest stage of the
# chain by far. 1 = the sequential loop.
# A FIXED worker count, not os.cpu_count()-2: the pool's size decided how
# the trials were distributed, and with it (before the fix in
# _plan_braid_worker) which seed each trial ran from. Pool.map is ordered,
# so results no longer depend on this -- but a machine-derived default is
# exactly the shape of dependence this chain is not allowed to have.
DST_RESIDUE_WORKERS = int(os.environ.get('DST_RESIDUE_WORKERS', '6'))
DST_SEARCH_CALLS = int(os.environ.get('DST_SEARCH_CALLS', '4000'))   # judge calls per call


# DST_DIVERS=1 (2026-09-10): the HUMAN's berth for every net that dives,
# chosen by the planner, no search. The braid's planner names the
# keepers (predicted 0 vias: tooth, lane and berth all on one layer) and
# the page every other net is delivered on; a keeper keeps its surface
# berth, and every other net takes the berth on the layer its lane
# arrives on -- a walked dog-bone into the via field where the menu has
# one, a dog-bone else, via-in-pad only when nothing else fits (the
# human: 0 via-in-pad on 47 nets) -- whose exit is cheapest by the
# selector's own reach + channel + crossings with the VIA FREE, since a
# diver pays its dive at the berth or in the corridor either way. A net
# delivered on its berth's own layer with a surface berth is left alone.
# Measured by the route at K28 (replan, 2026-09-10): nine such swaps tied
# 36 vias and took 25 mm off; the greedy over the walked menu, which
# offers the via berths to keepers too, routed 39.
DST_DIVERS = int(os.environ.get('DST_DIVERS', '0'))
DST_FACE_ASK = int(os.environ.get('DST_FACE_ASK', '0'))


def divers_berths(st, choice, board, log=print):
    """choice with every diver's berth on its delivery layer at the
    cheapest via site (see DST_DIVERS). Returns (choice, swaps)."""
    sm = pe.sm
    f0, pred, bp, _ = judge_by_braid(st, choice, board)
    box = st['dgrid'].bbox
    geo = sm.Corridor(box, st['launch'])
    bands = sm.bands_of_boxes(st['dboxes']) if isinstance(st['dboxes'], list) else []
    new = dict(choice)
    legs = {n: geo.leg(n, m) for n, m in new.items()}
    swaps = []

    def cost(n, m):
        reach = sm.around_box(st['launch'][n], m.exit_pt, box)
        xs = sum(1 for o, l in legs.items() if o != n and geo.paths_cross(geo.leg(n, m), l))
        return reach + 2.0 * sm._length(m) + 6.0 * xs

    def _layer(v):
        if v is None:
            return None
        v = str(v)
        return 'F.Cu' if v.startswith('F') else 'B.Cu' if v.startswith('B') else None

    f_cur, pred_cur, bp_cur = f0, pred, bp
    for n in sorted(choice, key=lambda k: (-pred.get(k, 0), k)):
        if n not in bp:
            continue
        cur = new[n]
        # a CERTAIN diver only: its tooth on the far layer (it cannot reach
        # the ball without a via), or a seed berth that already carries a
        # via. The planner's prediction is not the test -- SDQM0, SA13
        # and SA9 are route-keepers it predicts at 2, and a via berth on
        # one of those costs the two vias the human never pays
        pad_layer = st['dst_pad'][n].layers[0] if st['dst_pad'][n].layers else 'F.Cu'
        if not (st['tooth0'][n] != pad_layer or cur.vias >= 1):
            continue
        # the layer the braid DELIVERS the net on at its berth: the exit
        # leg's layer where the planner runs one (SA1: page B, leg on F,
        # and a B berth then costs two more vias), else the page
        arrive = _layer(bp_cur[n].get('exit_leg_layer')) or _layer(bp_cur[n].get('page'))
        if arrive is None:
            continue                                  # a swimmer: the planner has no layer for it
        if cur.kind == 'surface' and arrive == cur.layer:
            continue                                  # delivered on its berth's layer already
        if cur.kind == 'surface' and cur.vias == 0 and arrive != cur.layer:
            pass                                      # the MISMATCH: lane on the far layer, F stub
        elif cur.vias >= 1:
            pass                                      # a via berth: the human's site instead of a via-in-pad
        else:
            continue
        want = arrive
        # the berth keeps its FACE: the lane arrives there, and a face
        # change is a corridor change the route judged against (K28 SA13
        # right -> down, a keeper turned diver)
        cands = [m for m in st['dmenu'].get(n, ()) if m.layer == want and m.vias >= 1
                 and m.direction == cur.direction
                 and m is not cur and sm.lanes_free(m, new, n, strict=False, bands=bands)]
        if any(m.kind != 'via_in_pad' for m in cands):
            cands = [m for m in cands if m.kind != 'via_in_pad']
        if not cands:
            continue
        cands.sort(key=lambda m: (cost(n, m), getattr(m, 'walk', 0)))
        if cur.layer == want and cur.vias >= 1:
            cands = [m for m in cands if cost(n, m) < cost(n, cur) - 1e-6]
        # the braid's planner must agree: the net's own prediction and the
        # judged total may not rise (a veto, not a search: the human rule
        # names the candidate, the planner only refuses)
        for m in cands[:3]:
            trial = dict(new)
            trial[n] = m
            f_t, pred_t, bp_t, _ = judge_by_braid(st, trial, board)
            if pred_t.get(n, 0) <= pred_cur.get(n, 0) and f_t <= f_cur + 1e-6:
                new, f_cur, pred_cur, bp_cur = trial, f_t, pred_t, bp_t
                legs[n] = geo.leg(n, m)
                swaps.append((n, cur, m))
                break
    f1, pred1, bp1, _ = judge_by_braid(st, new, board)
    log(f'    divers\' berths (DST_DIVERS): {len(swaps)} swap(s), walked '
        f'{sum(1 for _n, _c, m in swaps if getattr(m, "walk", 0))}; judged {f0:.2f} -> {f1:.2f}, '
        f'predicted {sum(pred.values())} -> {sum(pred1.values())}, keepers '
        f'{sum(1 for v in pred.values() if v == 0)} -> {sum(1 for v in pred1.values() if v == 0)}')
    for n, c, m in swaps:
        log(f'      {n:7s} {sr.fmt_ask(c)} -> {sr.fmt_ask(m)}'
            + (f' walk {m.walk}' if getattr(m, 'walk', 0) else ''))
    return new, swaps


def seed_menu(st):
    """The menu the greedy selector seeds from. With the judged search on
    a banded destination, the WHOLE-ARRAY moves alone: the greedy prices
    a band berth as nearly free and fills the band with the deepest
    balls (K28: 50-57 vias against 36 without the band), while the
    search, judged by the braid's planner with its band comb, adds band
    berths one at a time where they pay. Otherwise the full menu."""
    if DST_SEARCH and isinstance(st['dboxes'], list):
        x0, y0, x1, y1 = st['dgrid'].bbox
        return {nm: [m for m in ms if not (x0 < m.exit_pt[0] < x1 and y0 < m.exit_pt[1] < y1)]
                for nm, ms in st['dmenu'].items()}
    return st['dmenu']


def refine_dest_by_judge(st, choice, board, log=print, budget=None):
    """Improve `choice` one berth at a time, judged by the braid's planner.
    Each net (costliest predicted first) tries every other move of its
    menu that its neighbours leave free (lanes and band room); the fast
    proxy ranks the improvements and the braid's planner confirms the
    best few; the first confirmed improvement is applied. Sweeps until
    nothing improves or the budget is spent. Returns (choice, cost)."""
    import time
    budget = DST_SEARCH_CALLS if budget is None else budget
    t0 = time.time()
    c0 = _spent()
    cache = {}
    pb = planned_buses(st, choice)
    bands = pe.sm.bands_of_boxes(st['dboxes']) if st['dboxes'] else []
    best_f, best_pred, _bp, _pl = judge_by_braid(st, choice, board)
    cur_p = total(choice, st, cache, pb)
    f0, n_moves, n_conf = best_f, 0, 0
    sweeps = 0
    improved = True
    while improved and _spent() - c0 < budget:
        improved = False
        sweeps += 1
        for n in sorted(choice, key=lambda k: -best_pred.get(k, 0)):
            if _spent() - c0 > budget:
                break
            cands = []
            for m in st['dmenu'].get(n, ()):
                if m is choice[n]:
                    continue
                if not pe.sm.lanes_free(m, choice, n, strict=False, bands=bands):
                    continue
                trial = dict(choice)
                trial[n] = m
                p = total(trial, st, cache, pb)
                if p < cur_p - 1e-6:
                    cands.append((p, m))
            cands.sort(key=lambda t: t[0])
            for p, m in cands[:3]:
                trial = dict(choice)
                trial[n] = m
                f, pred, _b, _q = judge_by_braid(st, trial, board)
                n_conf += 1
                if f < best_f - 1e-6:
                    log(f'    search: {n} {sr.fmt_ask(choice[n])} -> {sr.fmt_ask(m)}: '
                        f'judged {best_f:.2f} -> {f:.2f}')
                    choice, best_f, best_pred, cur_p = trial, f, pred, p
                    n_moves += 1
                    improved = True
                    break
    log(f'  destination search: judged {f0:.2f} -> {best_f:.2f}, {n_moves} move(s) in '
        f'{sweeps} sweep(s), {n_conf} confirmed by the planner, {time.time() - t0:.0f} s')
    return choice, best_f


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
    -- SCHED_FIRST -- the schedule-first planner over it (the greedy's
    move stays the fallback for a net the chains leave out), then the
    judged search and the divers' berths when those are on. ONE function
    for the first plan and for every re-plan after a refused berth: until
    2026-09-11 the re-plan in fanout_destination called the greedy alone,
    so a schedule-first plan with one refused berth shipped as the
    greedy's plan (K41: 15 refused on pass 0, passes 1-2 the greedy's).
    Returns (choice, unplaced)."""
    pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y)
            for nm in st['dst_pad']}
    if DST_SEED == 'pattern':
        choice = pattern_seed(st, log=log or (lambda *a: None))
        un = [nm for nm in st['dmenu'] if nm not in choice]
    else:
        choice, un = pe.sm.select(seed_menu(st), st['launch'],
                                  keep_out=st['dboxes'], buses=st['buses'],
                                  tooth_layer=st['tooth0'], log=None, pads=pads, chi=st['chi'])
    if choice and SCHED_FIRST:
        import sched_first
        order = None
        if sched_first.SF_BRAID_ORDER:
            # the braid's launch order on the provisional (greedy) plan
            _f, _p, bp, _pl = judge_by_braid(st, choice, board)
            corr = {v['corridor'] for v in bp.values()}
            if len(corr) == 1:
                order = [nm for nm, _v in sorted(bp.items(), key=lambda kv: kv[1]['launch_idx'])]
            elif log:
                log(f'  schedule-first: {len(corr)} corridors -- launch order by the teeth')
        sf_choice, _rep = sched_first.choose(st, seed_menu(st), log=log, launch_order=order,
                                             fixed=fixed, learned=learned)
        for nm, mv in choice.items():
            sf_choice.setdefault(nm, mv)
        un = [nm for nm in un if nm not in sf_choice]
        choice = sf_choice
    if choice and fixed and not SCHED_FIRST:
        # the berths a caller holds fixed (a re-plan on a realized source
        # board keeps the previous choice) bind the greedy's answer too:
        # `fixed` used to reach sched_first alone, and the chain env does
        # not set SCHED_FIRST, so the greedy re-chose every berth
        for nm, sig in fixed.items():
            m = next((mm for mm in st['dmenu'].get(nm, ()) if sr.move_sig(mm) == sig), None)
            if m is not None:
                choice[nm] = m
    if choice and DST_FACE_GROUP:
        choice = face_group_search(st, choice, board, log=log or (lambda *a: None))
    if choice and DST_RESIDUE:
        if DST_RESIDUE >= 2:
            choice = residue_choice(st, choice, board, log=log or (lambda *a: None), src_out=src_out)
        else:
            choice = residue_search(st, choice, board, log=log or (lambda *a: None))
    if choice and DST_SEARCH:
        choice, _sf = refine_dest_by_judge(st, choice, board)
    if choice and DST_DIVERS:
        choice, _sw = divers_berths(st, choice, board)
    return choice, un


def _nearest_move(menu, face, layer, at, ax):
    """The menu move on `face` and `layer` nearest position `at` along
    the face (fewest vias on a tie), else None."""
    best = None
    for m in menu:
        if m.direction != face or m.layer != layer:
            continue
        key = (abs(m.exit_pt[ax] - at), m.vias)
        if best is None or key < best[0]:
            best = (key, m)
    return best[1] if best else None


def face_group_search(st, choice, board, log=print, sweeps=2):
    """DST_FACE_GROUP: per destination face, the berth layers as up to two
    contiguous blocks along the face -- nets ordered by their berth
    position, a split point, the first block on one layer and the rest on
    the other (both orientations; the split at either end is 'all one
    layer') -- each net taking its menu move of that layer nearest its
    present gap; a candidate colliding with any chosen move (select_moves
    ._conflict, relaxed) is dropped; the braid's planner (judge_by_braid,
    under whatever schedule the env selects) judges every candidate and
    the best is kept. Coordinate descent over the faces, largest first,
    until a sweep improves nothing. Returns the choice."""
    import itertools
    best_f, _pred, _bp, _pl = judge_by_braid(st, choice, board)
    f0 = best_f
    n_moves = n_judged = 0
    for _sw in range(sweeps):
        improved = False
        faces = {}
        for nm, m in choice.items():
            faces.setdefault(m.direction, []).append(nm)
        for face, nets in sorted(faces.items(), key=lambda kv: -len(kv[1])):
            if len(nets) < 2:
                continue
            ax = 0 if face in ('up', 'down') else 1
            order = sorted(nets, key=lambda n: choice[n].exit_pt[ax])
            seen = set()
            cands = []
            for L1, L2 in (('F.Cu', 'B.Cu'), ('B.Cu', 'F.Cu')):
                for i in range(len(order) + 1):
                    trial = dict(choice)
                    ok = True
                    for k, nm in enumerate(order):
                        L = L1 if k < i else L2
                        if choice[nm].layer == L:
                            continue
                        m = _nearest_move(st['dmenu'].get(nm, ()), face, L,
                                          choice[nm].exit_pt[ax], ax)
                        if m is None:
                            ok = False
                            break
                        trial[nm] = m
                    if not ok:
                        continue
                    sig = tuple(sorted((nm, sr.move_sig(m)) for nm, m in trial.items()))
                    if sig in seen:
                        continue
                    seen.add(sig)
                    if sig == tuple(sorted((nm, sr.move_sig(m)) for nm, m in choice.items())):
                        continue
                    changed = [nm for nm in order if trial[nm] is not choice[nm]]
                    if any(pe.sm._conflict(trial[a], trial[b], strict=False)
                           for a in changed for b in trial if b != a):
                        continue
                    cands.append((L1[0], i, trial))
            for L1, i, trial in cands:
                f, _p, _b, _q = judge_by_braid(st, trial, board)
                n_judged += 1
                if f < best_f - 1e-6:
                    log(f'    face group: {face} {L1}x{i}/{len(order)}: judged {best_f:.2f} -> {f:.2f}')
                    best_f, choice = f, trial
                    n_moves += 1
                    improved = True
        if not improved:
            break
    log(f'  face groups: judged {f0:.2f} -> {best_f:.2f}, {n_moves} block(s) taken, '
        f'{n_judged} judged')
    return choice


def residue_search(st, choice, board, log=print, sweeps=4):
    """DST_RESIDUE: see the flag. Returns the choice."""
    import time
    t0 = time.time()
    c0 = _spent()
    bands = pe.sm.bands_of_boxes(st['dboxes']) if st['dboxes'] else []

    def judge(ch):
        f, pred, bp, _pl = judge_by_braid(st, ch, board)
        res = [nm for nm in ch if bp.get(nm, {}).get('page') is None]
        return (len(res), f), res, pred, (plan_lis(bp) if LIS_GUARD else None)
    key0, res, pred, lis0 = judge(choice)
    f0, n0 = key0[1], key0[0]
    n_moves = n_judged = 0
    pool = None
    if DST_RESIDUE_WORKERS > 1:
        import multiprocessing as mp
        try:
            pool = mp.get_context('spawn').Pool(DST_RESIDUE_WORKERS)
        except Exception as e:      # no pool: the sequential loop
            log(f'  residue search: no worker pool ({e}); sequential')
            pool = None
    log(f'  residue search: {len(res)} residue net(s) {res}, judged {f0:.2f}'
        + (f'; {DST_RESIDUE_WORKERS} workers' if pool else ''))
    for _sw in range(sweeps):
        if not res or _spent() - c0 > DST_RESIDUE_CALLS:
            break
        improved = False
        for nm in sorted(res, key=lambda n: -pred.get(n, 0)):
            if _spent() - c0 > DST_RESIDUE_CALLS:
                break
            best = None
            cands = []
            for m in st['dmenu'].get(nm, ()):
                if m is choice[nm]:
                    continue
                if not pe.sm.lanes_free(m, choice, nm, strict=False, bands=bands):
                    continue
                if any(o != nm and pe.sm._conflict(m, choice[o], strict=False) for o in choice):
                    continue
                cands.append(m)
            if pool is not None and len(cands) > 1:
                trials = []
                for m in cands:
                    trial = dict(choice)
                    trial[nm] = m
                    trials.append((m, trial, braid_plan_of(st, trial, board)))
                seeds = {tuple(sorted(k)): sorted(v) for k, v in te._L5_SEED.items()}
                try:
                    outs = pool.map(_plan_braid_worker,
                                    [(board, list(trial), st['dref'], pl, seeds) for _m, trial, pl in trials])
                    PLAN_CALLS[0] += len(trials)   # after the map: the
                    # sequential fallback below counts its own
                except Exception as e:
                    log(f'  residue search: worker pool failed ({e}); sequential from here')
                    pool.terminate(); pool = None
                    outs = None
                if outs is not None:
                    bps = [o[0] for o in outs]
                    for o in outs:
                        for k, v in o[1].items():
                            te._L5_SEED.setdefault(frozenset(k), set(v))
                    for (m, trial, _pl), bp in zip(trials, bps):
                        f, p2, _bp, _plan = judge_by_braid(st, trial, board, bp=bp)
                        r2 = [n for n in trial if bp.get(n, {}).get('page') is None]
                        k = (len(r2), f)
                        n_judged += 1
                        if best is None or k < best[0]:
                            best = (k, m, r2, p2, plan_lis(bp) if LIS_GUARD else None)
                    cands = []
            for m in cands:
                trial = dict(choice)
                trial[nm] = m
                k, r2, p2, l2 = judge(trial)
                n_judged += 1
                if best is None or k < best[0]:
                    best = (k, m, r2, p2, l2)
            if best is not None and accept_key(best[0], key0, best[4], lis0):
                log(f'    residue search: {nm} {sr.fmt_ask(choice[nm])} -> {sr.fmt_ask(best[1])}: '
                    f'residue {key0[0]} -> {best[0][0]}, judged {key0[1]:.2f} -> {best[0][1]:.2f}')
                choice = dict(choice)
                choice[nm] = best[1]
                key0, res, pred, lis0 = best[0], best[2], best[3], best[4]
                n_moves += 1
                improved = True
        if not improved:
            break
    if pool is not None:
        pool.close()
        pool.join()
    log(f'  residue search: {n0} -> {key0[0]} residue, judged {f0:.2f} -> {key0[1]:.2f}, '
        f'{n_moves} move(s), {n_judged} judged, {time.time() - t0:.0f} s')
    return choice


# SRC_REPLAN (2026-09-11, item 3): the SOURCE order judged honestly. The
# choice solve (DST_RESIDUE_SRC=1) prices a candidate tooth against lines it
# HOLDS STILL, and that is the one thing a tooth move cannot do -- a new
# tooth re-derives the launch comb for EVERY lane, so the schedule re-solves
# for all of them. Measured at K41 the held-lines model over-promised every
# time (SA1: 14 -> 13 residue promised, +2.1 delivered; confirmed, reverted
# and banned on each of three arms). This asks the PLANNER instead: for each
# candidate tooth, re-plan the whole corridor with that launch end in place
# (one braid.plan_braid call, no copper laid) and keep the candidate the
# planner itself ranks best. The winner still goes through the existing
# realize-and-confirm loop, so a tooth the engine cannot lay as asked is
# still banned. 0 = off. Cost is len(residue) x SRC_REPLAN_CANDS planner
# calls, so it is budgeted (SRC_REPLAN_CALLS).
SRC_REPLAN = int(os.environ.get('SRC_REPLAN', '0'))
SRC_REPLAN_CANDS = int(os.environ.get('SRC_REPLAN_CANDS', '3'))
SRC_REPLAN_CALLS = int(os.environ.get('SRC_REPLAN_CALLS', '15000'))
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
# SRC_REFAN_STRICT=1: cap the engine's degrade ladder at level 2, so a
# refused ask is REFUSED (and the net keeps its copper, said so) instead of
# arriving as a near-miss the audit has to unpick.
SRC_REFAN_STRICT = int(os.environ.get('SRC_REFAN_STRICT', '0'))
# DST_CONTEND (2026-09-11): the via-site CONTENTION term, in the JOINT
# SOLVE's own currency. A barrel does not just cost a via, it takes an
# inter-ball site, and under a ball field those sites are the scarcest room
# on the board. Measured on DU1's menu at K41: a site INSIDE the field is
# wanted by 6.14 other nets, one OUTSIDE it by 2.37 -- and among via-bearing
# moves the off-array one costs a median of +0.035 vias while denying 4.07
# fewer nets, so 0.016 vias per contender flips the median net. That is what
# the human buys with the corner nets' off-array vias and what our cost
# could not see. Charged as a DIFFERENCE against the berth the net holds,
# like the vias and ride terms beside it. The same term on the GREEDY's
# first selection (select_moves.SEL_CONTEND) was measured inert at K41 and
# 10 vias WORSE at K35, identically at 0.05 and 0.30 -- it flips a discrete
# set of choices at any weight and saturates. Here it is inside the solve
# that actually trades congestion against the schedule.
DST_CONTEND = float(os.environ.get('DST_CONTEND', '0') or 0)
# DST_XING (2026-09-12): the CROSSING term -- the objective the planner was
# missing. Two lanes need DIFFERENT PAGES exactly when their paths cross, so
# one page holds at most the longest crossing-free chain and two layers hold
# twice that. Measured, our chain is ~21 lanes at every K, so two pages hold
# ~42: K35 (35 lanes) has slack 7, K41 (41) has ONE -- the knife edge -- and
# K51 (47) is SEVEN OVER, which is why K51's extra nets cost 48 vias, 38 of
# them on the nets that were already there. The human's berths give a chain
# of 29 (capacity 58) and 68 crossings where ours give 251.
# Nothing priced this: the candidate screen ranked berths by `vias + ride`
# alone, the greedy prices every geometric crossing alike (and SEL_XLAYER=1,
# which prices by page consumption, measured WORSE because it must guess
# layers before pages exist), and the level-5 MILP optimises VIAS, so it
# accepts any crossing it believes it can page.
# The metric is the STRAIGHT tooth->berth segments' crossings, which needs no
# rank reconstruction: measured against the braid's own inversions it reads
# 177/168, 194/250, 351/352 at K35/K41/K51. (A projection of the ends onto
# one axis does NOT track -- 234/295/496 -- because the braid orders its exit
# BLOCK by leg position, not by stub offset; head-on exits alone it gets
# exactly right, block exits 22% wrong.)
DST_XING = float(os.environ.get('DST_XING', '0') or 0)
# the cap on crossing PAIRS handed to the solve. A pair costs one variable
# and one row, and only pairs whose crossing status VARIES across the two
# nets' candidates are emitted at all (one that crosses in every combination
# is a constant), so the count is far below the candidate-pair square. The
# most DISCRIMINATING pairs go first -- the ones nearest an even split, which
# are the ones the choice can actually act on.
DST_XING_PAIRS = int(os.environ.get('DST_XING_PAIRS', '4000'))
# DST_XING_SCREEN=1 (2026-09-12): rank the candidate berths a net offers the
# solve by the CROSSINGS they make, not by `vias + ride`. Measured, each K51
# net has a median of 26 distinct berth geometries and the cap hands the
# solve 4 of them -- 16% -- pre-filtered by a LOCAL cost that ignores
# crossings entirely. So the pairwise crossing term (DST_XING), which is
# correct and cheap, was optimising over a set chosen by the very criterion
# it exists to replace, and it beat the baseline on nothing. This screens
# the whole menu on the objective first, and the cap then keeps the four
# that can actually restructure the order. The one-per-FACE guarantee stays
# (the face is what moves a net along the target order), but faces are now
# served by their best candidate's crossings rather than by nearness.
DST_XING_SCREEN = int(os.environ.get('DST_XING_SCREEN', '0') or 0)
# DST_SEED=pattern (2026-09-12): seed the berths the way a HUMAN lays a bus
# -- not by searching every net's menu independently, but by FOLLOWING A
# PATTERN: take the nets in launch order and give each one the berth that
# CONTINUES the previous net's, same face and same layer and the next gap
# along that face in the direction of travel, breaking the pattern only when
# nothing continues it. A run laid that way is monotone, and a monotone run
# is a crossing-free chain -- which is exactly the quantity that sets the
# two-page capacity (~21 lanes for us, 29 for the human, and K51 needs 24).
# The greedy instead prices each net alone (`via_weight*vias + channel + reach`)
# and reaches a chain of ~21 with 250-352 crossings where the human's berths
# give 68. Off = the greedy's seed, byte-identical.
DST_SEED = os.environ.get('DST_SEED', '')
# DST_ASK_BAN (2026-09-12): a berth the fanout engine answered with one a
# LAYER, a KIND or a FACE away is not asked for again. See the ban site in
# fanout_destination for what it measured.
DST_ASK_BAN = int(os.environ.get('DST_ASK_BAN', '0') or 0)
# ...and WHICH ORDER the pattern is monotone in. The braid's own crossing
# count is the inversion count of launch order against TARGET order, and its
# target order is ONE scalar per net -- `target_o`, the spine's perpendicular
# offset `o` (braid.Corridor, `self.target = sorted(members, key=target_o)`).
# Faces there only group a block WITHIN that one axis. So a seed ordered
# "along its own face" is monotone in a coordinate the objective never uses,
# and two nets on different faces are never compared at all:
#   face  -- per (face, layer), monotone along each face (the first build)
#   perp  -- per LAYER, monotone in the spine's own o (the braid's frame)
#   arc   -- per LAYER, monotone in the angle about the destination as seen
#            from the source: the perimeter walked from one flank round the
#            front to the other, which is what a bus wrapping an array does
DST_SEED_ORDER = os.environ.get('DST_SEED_ORDER', 'face')
# DST_SWIM (2026-09-12): the candidate SCREEN priced by the inversions a
# berth makes WITH LANES ON ITS OWN PAGE. The screen ranks ~26 berths by
# `vias + ride` and hands the solve 4 of them, so the solve can only
# repair an order the screen already chose -- and `vias + ride` knows
# nothing about pages. Raw crossing count was tried twice (DST_XING,
# DST_XING_SCREEN) and beat nothing, and the reason is that MOST
# crossings are free: two lanes on different pages cross at no cost. The
# crossings that are NOT free are the ones inside a page, and they are
# exactly what turns a lane into a swimmer. So count those.
DST_SWIM = float(os.environ.get('DST_SWIM', '0') or 0)


def _seat_repair(out, menu, cost_of, log, launch=None):
    """Seat the nets the walk could not, by MOVING one that can move.

    A net is left out when every berth in its menu conflicts with one
    already seated -- a matching failure, not an ordering one, and the
    measured case has the blocked net down to a single free berth while the
    net holding it has twenty. So: for each unseated net, find a berth
    whose only blocker is ONE seated net that has an alternative berth
    conflicting with nothing else, and swap. One augmenting step, which is
    all the measured cases need."""
    def xing(nm, m, skip=None):
        """The lanes this berth would cross. A seat is only worth taking
        if it keeps the ORDER -- seating by price alone was measured to
        seat every net and triple the crossings (K51 192 -> 281)."""
        if launch is None:
            return 0
        a, b = tuple(launch[nm]), tuple(m.exit_pt)
        return sum(1 for o, om in out.items()
                   if o != nm and o != skip and om.layer == m.layer
                   and lanes_cross(a, b, tuple(launch[o]), tuple(om.exit_pt)))
    n = 0
    for nm in [x for x in menu if menu[x] and x not in out]:
        done = False
        for m in sorted(menu[nm], key=lambda m: (xing(nm, m), cost_of(nm, m))):
            blk = [o for o, om in out.items()
                   if pe.sm._conflict(m, om, strict=False)]
            if len(blk) != 1:
                continue
            o = blk[0]
            keep = xing(nm, m, skip=o) + xing(o, out[o])
            for om2 in sorted(menu[o], key=lambda x: (xing(o, x, skip=o),
                                                      cost_of(o, x))):
                if pe.sm._conflict(om2, m, strict=False):
                    continue
                if any(pe.sm._conflict(om2, om3, strict=False)
                       for o3, om3 in out.items() if o3 != o):
                    continue
                if xing(nm, m, skip=o) + xing(o, om2, skip=o) > keep + 2:
                    continue          # the swap costs more order than it buys
                out[o] = om2
                out[nm] = m
                n += 1
                done = True
                break
            if done:
                break
    if n:
        log(f'    seat repair: {n} net(s) seated by moving one held berth')
    return n


def pattern_seed(st, log=print, order=None):
    """Berths laid the way a HUMAN lays a bus: the nets keep their ORDER.

    Not "each net's own best berth" (the greedy) and not "the next berth
    along the face" (measured worse than the greedy: it marches one ribbon
    through berths whose owners launch from all over). The property that
    actually kills crossings is MONOTONICITY -- on each destination face,
    the berths must run in the same order as the nets' launches, because two
    lanes cross exactly when their launch order and berth order disagree.
    So: give every net a face, sort that face's nets by launch, and walk
    them assigning berths that only ever move FORWARD along the face. The
    pattern breaks -- a berth behind the last one -- only when the net's own
    menu offers nothing ahead, which is the corner a human turns too.
    """
    menu = seed_menu(st)
    launch = st['launch']
    db = st.get('dboxes')
    if db and not isinstance(db[0], (int, float)):
        db = db[0]
    sx = sum(p[0] for p in launch.values()) / max(1, len(launch))
    sy = sum(p[1] for p in launch.values()) / max(1, len(launch))
    dx_, dy_ = (((db[0] + db[2]) / 2 - sx, (db[1] + db[3]) / 2 - sy)
                if db and len(db) == 4 else (1.0, 0.0))
    L = math.hypot(dx_, dy_) or 1.0
    px, py = -dy_ / L, dx_ / L

    def o_of(pt):
        return pt[0] * px + pt[1] * py

    def along(m):
        """Where a berth sits ALONG its own face."""
        return m.exit_pt[1] if m.direction in ('left', 'right') else m.exit_pt[0]
    cx, cy = ((db[0] + db[2]) / 2, (db[1] + db[3]) / 2) if db and len(db) == 4 else (sx, sy)

    def arc(m):
        """Where a berth sits around the destination, as the source sees
        it: the angle off the source->destination axis, in (-pi, pi]. The
        near face runs through 0 and the far face is the branch cut, so the
        coordinate walks the perimeter from one flank round the front to the
        other -- one order over all four faces, which `along` cannot give."""
        vx, vy = m.exit_pt[0] - cx, m.exit_pt[1] - cy
        # ACROSS the source axis over ALONG it, negated so the near side is
        # the zero. `vx*py - vy*px` is v.dhat -- the along component twice,
        # i.e. atan2(u, -u), which takes two values and orders nothing.
        return math.atan2((vy * dx_ - vx * dy_) / L, -(vx * dx_ + vy * dy_) / L)
    mode = (order if order is not None else DST_SEED_ORDER).lower()
    nets = [nm for nm in menu if menu[nm]]
    # 1. a face per net: the one its cheapest berth uses, i.e. where the net
    #    naturally wants to go. Forcing the FACES themselves into one
    #    rotational arc (the bus wrapping the array) was tried and is worse
    #    -- K35 106 -> 134 crossings, K51 244 -> 257 -- because it puts nets
    #    on faces that do not suit them. The order within a face is what
    #    pays; the face itself is the net's own business.
    face = {}
    for nm in nets:
        m = min(menu[nm], key=lambda m: (m.vias, abs(o_of(m.exit_pt) - o_of(launch[nm]))))
        face[nm] = m.direction
    out, used = {}, set()
    n_break = 0
    # 2. the walk is per (FACE, LAYER), not per face. A crossing only costs
    #    when both lanes are on the SAME page -- two stubs on different
    #    layers may cross for free -- so the order that has to be monotone
    #    is the order WITHIN a layer. Each (face, layer) is one page's worth
    #    of that face, and a net's layer is its tooth's: a net born on F
    #    stays on F unless its menu there is empty, which is the layer a
    #    human keeps too.
    tl = st.get('tooth0', {})
    lay = {}
    for nm in nets:
        want = tl.get(nm)
        have = {m.layer for m in menu[nm] if m.direction == face[nm]} or {m.layer for m in menu[nm]}
        lay[nm] = want if want in have else (sorted(have)[0] if have else want)

    def ok(m):
        # the same feasibility the greedy honours: a berth that CONFLICTS
        # with one already seeded cannot be laid beside it (deduping exact
        # exit points is not enough -- K51 seeded 5 nets onto berths that
        # could not coexist and shipped them open, on a plan whose ORDER
        # was the best measured)
        if (round(m.exit_pt[0], 2), round(m.exit_pt[1], 2)) in used:
            return False
        return not any(pe.sm._conflict(m, om, strict=False) for om in out.values())
    def cost_of(nm, m):
        """The berth's own local price, the greedy's: vias plus the ride it
        asks for, at VIA_MM per via."""
        return m.vias + pe.sm.ride_mm({nm: m}, launch, st['dboxes'],
                                      st['sgrid'].bbox) / pe.sm.VIA_MM

    if mode == 'dp':
        # THE PATTERN AS AN ASSIGNMENT rather than as a walk, over BOTH
        # pages at once. Two lanes cross exactly when their launch order
        # and their order around the destination disagree, and a crossing
        # is only PAID for when both lanes sit on the same page -- so the
        # berths that cost nothing are two interleaved runs, each monotone,
        # one per page. That is the two-page capacity itself, and a DP over
        # (last berth on page 0, last berth on page 1) finds the largest
        # one there is. The greedy walk instead takes the first berth that
        # is forward, so one net reaching far ahead pushes every net after
        # it; the nets this cannot seat are the PATTERN BREAKS, filled
        # afterwards at their own best berth (a break is a swimmer, and a
        # swimmer costs ~1 via -- far less than a crossing that consumes a
        # page). No face anywhere: `arc` is one order around the whole
        # array and it reverses at the corners by itself.
        nets_o = sorted(nets, key=lambda nm: o_of(launch[nm]))

        def _sk_of(m):
            return (round(m.exit_pt[0], 2), round(m.exit_pt[1], 2),
                    m.direction, m.layer, m.kind)
        slot, cand = {}, {}
        for nm in nets_o:
            for m in menu[nm]:
                sk = (round(m.exit_pt[0], 2), round(m.exit_pt[1], 2),
                      m.direction, m.layer, m.kind)
                slot.setdefault(sk, m)
                cand.setdefault(nm, {})[sk] = m
        pg_names = sorted({sk[3] for sk in slot})[:2]
        if len(pg_names) == 1:
            pg_names.append(None)
        pick = {nm: min(cand[nm].values(), key=lambda m: m.vias)
                for nm in nets_o if cand.get(nm)}
        seqa = [arc(pick[nm]) for nm in nets_o if nm in pick]

        def bad(sg):
            v = [sg * a_ for a_ in seqa]
            return sum(1 for i in range(len(v)) for j in range(i + 1, len(v))
                       if v[i] > v[j])
        sgn = 1 if bad(1) <= bad(-1) else -1
        order = {pg: sorted((sk for sk in slot if sk[3] == pg),
                            key=lambda sk: sgn * arc(slot[sk])) for pg in pg_names}
        idx = {sk: i for pg in pg_names for i, sk in enumerate(order[pg])}
        M = {pg: len(order[pg]) for pg in pg_names}
        n0, n1 = M[pg_names[0]], M[pg_names[1]]
        # The berths a berth cannot sit beside, as ONE number each: the
        # LOWEST-indexed berth that conflicts with it, on its own page and
        # on the other. Chosen indices only ever increase, so "the last one
        # taken is below that" proves no berth taken so far conflicts --
        # sound with a state that remembers only the last. (Checking the
        # last one alone is not: a conflict two berths back is invisible,
        # and then the choice is dropped at apply time and the greedy
        # fills it, which is exactly what broke the chain.)
        big = max(n0, n1) + 1
        minc, minx = {}, {}
        for pg in pg_names:
            o_, oth = order[pg], order[pg_names[1] if pg == pg_names[0] else pg_names[0]]
            mc = [big] * len(o_)
            mx = [big] * len(o_)
            for i_ in range(len(o_)):
                for j_ in range(len(o_)):
                    if j_ != i_ and pe.sm._conflict(slot[o_[i_]], slot[o_[j_]],
                                                    strict=False):
                        mc[i_] = min(mc[i_], j_)
                for j_ in range(len(oth)):
                    if pe.sm._conflict(slot[o_[i_]], slot[oth[j_]], strict=False):
                        mx[i_] = min(mx[i_], j_)
            minc[pg], minx[pg] = mc, mx
        # dp[a][b]: the best (count, -cost) whose last berth on page 0 is
        # order[0][a-1] and on page 1 is order[1][b-1]; 0 = none yet
        dp = [[None] * (n1 + 1) for _ in range(n0 + 1)]
        dp[0][0] = (0, 0.0)
        back, dps = [], [[row[:] for row in dp]]
        for nm in nets_o:
            cs = cand.get(nm) or {}
            cost = {sk: cost_of(nm, m) for sk, m in cs.items()}
            nd = [row[:] for row in dp]
            step = {}
            for sk in cs:
                pg = sk[3]
                k = idx[sk]
                p0 = (pg == pg_names[0])
                mc_, mx_ = minc[pg][k], minx[pg][k]
                for a_ in range(n0 + 1):
                    row = dp[a_]
                    for b_ in range(n1 + 1):
                        if row[b_] is None:
                            continue
                        last, oth_ = (a_, b_) if p0 else (b_, a_)
                        if k < last or last > mc_ or oth_ > mx_:
                            continue
                        na, nb = (k + 1, b_) if p0 else (a_, k + 1)
                        sc = (row[b_][0] + 1, row[b_][1] - cost[sk])
                        if nd[na][nb] is None or sc > nd[na][nb]:
                            nd[na][nb] = sc
                            step[(na, nb)] = ((a_, b_), nm, sk, sc)
            dp = nd
            back.append(step)
            dps.append([r[:] for r in dp])
        fin = max(((a_, b_) for a_ in range(n0 + 1) for b_ in range(n1 + 1)
                   if dp[a_][b_] is not None), key=lambda t: dp[t[0]][t[1]])
        # read the chain back. A state's score is matched against the
        # snapshot AT THAT STEP, never the final array: a state reached by
        # an assignment at step i can be improved later, and matching on
        # the final value would re-assign a net the chain never took.
        chain, cur = {}, fin
        for i_ in range(len(nets_o) - 1, -1, -1):
            hit = back[i_].get(cur)
            if hit and hit[3] == dps[i_ + 1][cur[0]][cur[1]]:
                chain[hit[1]] = slot[hit[2]]
                cur = hit[0]
        log(f'    dp: {len(nets_o)} net(s), {n0}+{n1} slot(s) on '
            f'{pg_names[0]}/{pg_names[1]}, chain {dp[fin[0]][fin[1]][0]}')
        for nm in nets_o:
            # the net's OWN move for that slot, never the representative.
            # `slot` keys on (x, y, direction, layer, kind) only, and menus
            # of different balls collide on that key constantly -- measured
            # on the bench, 11 of 15 seated berths were a FOREIGN net's
            # Move, one of them asking the engine to drill a via 9.6 mm
            # away inside another ball, because `site` and `legs` travel
            # with the Move into escape_dir_hints.
            m = chain.get(nm)
            if m is not None:
                m = (cand.get(nm) or {}).get(_sk_of(m), m)
            if m is not None and ok(m):
                out[nm] = m
                used.add((round(m.exit_pt[0], 2), round(m.exit_pt[1], 2)))
        for nm in nets_o:
            if nm in out:
                continue
            cs2 = [m for m in menu[nm] if ok(m)]
            if not cs2:
                continue
            m = min(cs2, key=lambda m: cost_of(nm, m))
            out[nm] = m
            used.add((round(m.exit_pt[0], 2), round(m.exit_pt[1], 2)))
            n_break += 1
        _seat_repair(out, menu, cost_of, log, launch)
        log(f'  pattern seed (dp): {len(out)}/{len(nets)} berth(s), '
            f'{n_break} pattern break(s)')
        return out

    if mode == 'face':
        # one run per (face, layer); the coordinate is only comparable to
        # berths on the SAME face, so that is the grouping too
        def key_of(nm):
            return (face[nm], lay[nm])

        def coord(nm, m):
            return along(m)

        def cands(nm, k):
            d, Lp = k
            return ([m for m in menu[nm] if m.direction == d and m.layer == Lp],
                    [m for m in menu[nm] if m.direction == d],
                    list(menu[nm]))

        def fits(nm, m, k):
            return m.direction == k[0] and m.layer == k[1]
    else:
        # one run per LAYER over ALL the faces, in the destination's own
        # global coordinate -- the braid's frame. A net is free to land on
        # whatever face the order takes it to.
        gco = o_of if mode in ('perp', 'faceperp') else None
        onface = mode in ('facearc', 'faceperp')
        # WINDOW: the face-free way to say what the face was really saying.
        # A face is a contiguous run of the arc, and restricting a net to
        # its own face does two things -- it fixes which way round the
        # array the net goes, and it stops the monotone walk dragging the
        # net right across the array to stay forward. The second is just
        # LOCALITY, and locality is a window: a net may take any berth
        # within DST_SEED_WIN radians of the one it would pick for itself,
        # whatever face that lands on.
        win = float(os.environ.get('DST_SEED_WIN', '0.9') or 0.9)
        home = {}
        if mode == 'win':
            for nm in nets:
                c_ = [m for m in menu[nm] if m.layer == lay[nm]] or list(menu[nm])
                home[nm] = arc(min(c_, key=lambda m: cost_of(nm, m)))

        def key_of(nm):
            return (lay[nm],)

        def coord(nm, m):
            return gco(m.exit_pt) if gco else arc(m)

        def cands(nm, k):
            # `face*` modes keep the face ASSIGNMENT (which way round the
            # array a net approaches -- a topological choice no 1-D
            # coordinate carries) and take only the ORDER globally
            if onface:
                return ([m for m in menu[nm] if m.layer == k[0] and m.direction == face[nm]],
                        [m for m in menu[nm] if m.direction == face[nm]],
                        [m for m in menu[nm] if m.layer == k[0]] or list(menu[nm]))
            if mode == 'win':
                near = [m for m in menu[nm]
                        if abs(_wrap(arc(m) - home[nm])) <= win]
                return ([m for m in near if m.layer == k[0]], near,
                        [m for m in menu[nm] if m.layer == k[0]] or list(menu[nm]))
            return ([m for m in menu[nm] if m.layer == k[0]], list(menu[nm]),
                    list(menu[nm]))

        def fits(nm, m, k):
            return m.layer == k[0]
    for k in sorted({key_of(nm) for nm in nets}):
        grp = sorted((nm for nm in nets if key_of(nm) == k),
                     key=lambda nm: o_of(launch[nm]))
        if not grp:
            continue
        # which way along the run the launch order naturally travels
        nat = {}
        for nm in grp:
            c_ = next((c for c in cands(nm, k) if c), [])
            nat[nm] = coord(nm, min(c_, key=lambda m: m.vias)) if c_ else 0.0

        def bad(sgn):
            v = [sgn * nat[nm] for nm in grp]
            return sum(1 for i in range(len(v)) for j in range(i + 1, len(v)) if v[i] > v[j])
        sgn = 1 if bad(1) <= bad(-1) else -1
        # 3. walk the group in launch order, only ever moving FORWARD
        last = None
        for nm in grp:
            cs = next((c2 for c2 in ([m for m in c if ok(m)] for c in cands(nm, k)) if c2), [])
            if not cs:
                continue
            fwd = [m for m in cs if last is None or sgn * coord(nm, m) > last]
            if not fwd:
                fwd = cs
                n_break += 1
            m = min(fwd, key=lambda m: (sgn * coord(nm, m), m.vias))
            out[nm] = m
            used.add((round(m.exit_pt[0], 2), round(m.exit_pt[1], 2)))
            if fits(nm, m, k):
                last = sgn * coord(nm, m)
    _seat_repair(out, menu, cost_of, log, launch)
    log(f'  pattern seed ({mode}): {len(out)}/{len(nets)} berth(s) over '
        f'{len({key_of(nm) for nm in nets})} run(s), {n_break} pattern break(s)')
    return out


def _wrap(a):
    """An angle difference folded into (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def lanes_cross(a, b, c, d):
    """Do the straight lanes a->b and c->d cross?"""
    return (_ccw(a, c, d) != _ccw(b, c, d)) and (_ccw(a, b, c) != _ccw(a, b, d))


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


# SRC_EXCHANGE (2026-09-12, README TODO 12 / audit item c1): the source
# 2-OPT, as a PROBE. Every arm runs SRC_ROUNDS=0, so U1's teeth are the
# bench's own fanout and the source disagreement with the human is an
# INPUT rather than a result -- but a tooth is physical copper, so a plan
# that exchanges two nets' launch points is not realizable without a
# re-fan, and three sessions of source arms have measured null.
# This asks the cheap question FIRST: under the braid's own judge, does
# ANY pairwise exchange of two nets' launch points improve the plan at
# all? If none does, the realize build is not worth writing and the source
# lever is dead on the merits rather than dead on the engine. If some do,
# the gains name exactly which pairs a re-fan should target.
# It changes NOTHING -- it judges and logs. Cost is one planner call per
# pair tried, so it is capped.
SRC_EXCHANGE = int(os.environ.get('SRC_EXCHANGE', '0') or 0)


# SF_ROUTE_SCREEN (2026-09-12): THE ROUTE AS THE PASS'S OWN JUDGE.
#
# The destination pass loop accepts and re-plans on `planner judge of the
# LAID board` -- the surrogate. Measured, that surrogate ranks plans at
# ~50% against the routed via count, has no resolution where the search
# works (28 of 39 K41 boards share ONE judged cost while spanning 54-86
# routed vias), and -- the part that matters -- is BLIND TO REFUSALS,
# which carry ~80% of its error. A lane the router refuses is where the
# model is cheapest and the copper is dearest, so optimising the surrogate
# walks INTO the refused region.
#
# The refusals are not mysterious: every one of 174 rip min-cut probes
# names a cut set of THIS RUN'S OWN LANES (mean 5.2), never static copper.
# The model's lanes coexist; the router lays them sequentially and the
# loser pays (+1.45 vias at K41 over its plan, 527 cases).
#
# So: run the real braid on the laid board and read its refusal set. Not
# the full braid -- ATTEMPT 0 ONLY (BRAID_ATTEMPTS=1), which is the first
# pass with no rip ladder and no re-plan. That is the cheap half: ~30 s at
# K41 against ~1.3 min for the full braid, and refusals are exactly what
# attempt 0 already reports.
#
# HOW THIS DIFFERS FROM replan.py, which also uses the route as judge:
# replan is an OUTER driver over a FINISHED board, moving ONE NET at a
# time and re-braiding to confirm. This is INSIDE the chain's own pass
# loop, judging the WHOLE PLAN, and it replaces the surrogate rather than
# repairing what the surrogate chose. They compose: fewer bad passes
# shipped means less for replan to repair.
#
# 1 = MEASURE ONLY: run the screen, log refused-vs-swimmers, change
#     nothing. What that answers before any behaviour depends on it: what
#     the screen costs, and whether its refusal set differs from the
#     planner's swimmer set usefully enough to be worth deciding on.
# 2 = DECIDE: also feed the refused lanes back as the nets to free.
# 0 = off (default).
SF_ROUTE_SCREEN = int(os.environ.get('SF_ROUTE_SCREEN', '0') or 0)
# work-based budget: screens, never seconds.
SF_ROUTE_SCREEN_CALLS = int(os.environ.get('SF_ROUTE_SCREEN_CALLS', '8'))
_SCREEN_CALLS = [0]


def route_screen(board_path, names, dref, log=print):
    """Braid `board_path` for ONE attempt and return the refused nets.

    Runs the production braid as a subprocess, exactly as the chain's own
    braid stage does, so the screen sees what the chain will see -- no
    second implementation to drift. Returns None when the budget is spent
    or the braid could not be read, so a caller can tell "no answer" from
    "nothing refused"."""
    import re, subprocess, tempfile   # `re` is not a module-level import here
    if _SCREEN_CALLS[0] >= SF_ROUTE_SCREEN_CALLS:
        return None
    _SCREEN_CALLS[0] += 1
    env = dict(os.environ)
    env['BRAID_ATTEMPTS'] = '1'          # attempt 0 only: no rip ladder
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'screen')
        r = subprocess.run(
            [sys.executable, '-u', os.path.join(HERE, 'braid.py'),
             '--board', board_path, '--dest', dref,
             '--nets', ','.join(names), '--out', out],
            capture_output=True, text=True, env=env)
    # SILENCE IS NOT SUCCESS, and here it is also not FAILURE. The braid
    # prints "REFUSED nets (left open): [...]" ONLY when something was
    # refused (braid.py:8005, guarded by `if refused:`), so an absent line
    # means EITHER a clean route OR a braid that died. Those must not
    # collapse to the same answer -- an empty refusal set is the best
    # possible result and a dead screen is no result at all. Decide on a
    # positive success marker ("wrote <board>:", printed on every completed
    # braid) and treat its absence as no verdict.
    ok = (r.returncode == 0) and ('wrote ' in r.stdout)
    if not ok:
        tail = [l for l in r.stdout.splitlines()[-3:] if l.strip()]
        log(f'    route screen: NO VERDICT (rc={r.returncode}) {tail}')
        return None
    m = re.findall(r'REFUSED nets \(left open\): \[(.*?)\]', r.stdout)
    if not m:
        return []                       # routed clean: nothing refused
    return [x.strip().strip("'\"") for x in m[-1].split(',') if x.strip()]
SRC_EXCHANGE_PAIRS = int(os.environ.get('SRC_EXCHANGE_PAIRS', '60'))


def _st_swap_src(st, a, b):
    """`st` with nets `a` and `b` trading launch points -- the same four
    fields _st_with_src touches, exchanged rather than replaced."""
    st2 = dict(st)
    for key in ('launch', 'tooth0', 'tooth_vias'):
        d = dict(st[key])
        if a in d and b in d:
            d[a], d[b] = d[b], d[a]
        st2[key] = d
    so = dict(st.get('src_over') or {})
    if a in so or b in so:
        so[a], so[b] = so.get(b), so.get(a)
        so = {k: v for k, v in so.items() if v is not None}
    st2['src_over'] = so
    return st2


def src_exchange_probe(st, choice, board, f0, log=print):
    """Judge every pairwise source exchange among the nets the schedule
    could not page. Reports the improving ones; changes nothing."""
    res = [nm for nm in choice if nm in st.get('launch', {})]
    gains = []
    tried = 0
    for i, a in enumerate(sorted(res)):
        for b in sorted(res)[i + 1:]:
            if tried >= SRC_EXCHANGE_PAIRS:
                break
            tried += 1
            try:
                f, _p, _bp, _pl = judge_by_braid(_st_swap_src(st, a, b), choice, board)
            except Exception as e:                      # a swap the planner refuses
                log(f'    source exchange: {a}<->{b} refused ({type(e).__name__})')
                continue
            if f < f0 - 1e-6:
                gains.append((f0 - f, a, b))
    gains.sort(reverse=True)
    if gains:
        log(f'  source exchange: {len(gains)} of {tried} pair(s) improve the judge; '
            + ', '.join(f'{a}<->{b} {g:+.2f}' for g, a, b in gains[:6]))
    else:
        log(f'  source exchange: NO pair of {tried} improves the judge '
            f'(judged {f0:.2f}) -- the source order is not the lever here')
    return gains


def _src_screen(st, nm, moves, cap, tabu):
    """The candidate teeth worth a planner call: one per distinct geometry
    (exit point, layer, face -- the kind and via site are not something the
    lane sees), cheapest by vias then by how far the tooth travels, capped.
    One per FACE first, as the destination screen does, since the face is
    what moves a net along the launch order and the gap only fine-tunes it."""
    cur = st['launch'][nm]
    by_geo = {}
    for m in moves:
        if (nm, sr.move_sig(m)) in tabu:
            continue
        g = (round(m.exit_pt[0], 3), round(m.exit_pt[1], 3), m.layer, m.direction)
        c = (m.vias, math.hypot(m.exit_pt[0] - cur[0], m.exit_pt[1] - cur[1]))
        if g not in by_geo or c < by_geo[g][0]:
            by_geo[g] = (c, m)
    by_f = {}
    for _g, (c, m) in by_geo.items():
        by_f.setdefault(m.direction, []).append((c, m))
    keep = []
    for d in by_f:
        by_f[d].sort(key=lambda t: t[0])
        keep.append(by_f[d].pop(0))
    keep.sort(key=lambda t: t[0])
    rest = sorted((t for v in by_f.values() for t in v), key=lambda t: t[0])
    return [m for _c, m in (keep + rest)[:cap]]


def src_replan_pick(st, choice, board, res, base_key, log=print, tabu=()):
    """The best candidate tooth among the residue nets, judged by re-planning
    the corridor with it. Returns (key, nm, move) or None if none beats
    `base_key` = (swimmers, cost) of the plan as it stands."""
    import time
    t0 = time.time()
    c0 = _spent()
    best = None
    n = 0
    for nm in res:
        moves = st['smenu'].get(nm, ())
        if not moves:
            continue
        for m in _src_screen(st, nm, moves, SRC_REPLAN_CANDS, set(tabu)):
            if _spent() - c0 > SRC_REPLAN_CALLS:
                log(f'    source re-plan: budget spent after {n} trial(s)')
                return best
            try:
                f2, _p2, bp2, _pl2 = judge_by_braid(_st_with_src(st, nm, m), choice, board)
            except Exception as e:
                log(f'    source re-plan: {nm} {m} not judgeable ({e})')
                continue
            n += 1
            key2 = (sum(1 for x in choice if bp2.get(x, {}).get('page') is None), f2)
            if key2 < base_key and (best is None or key2 < best[0]):
                best = (key2, nm, m)
                log(f'    source re-plan: {nm} -> {m}: swimmers {base_key[0]} -> '
                    f'{key2[0]}, cost {base_key[1]:.2f} -> {f2:.2f}  <- best')
    log(f'    source re-plan: {n} tooth candidate(s) judged in {time.time() - t0:.0f} s'
        + ('' if best else '; none beat the plan as it stands'))
    return best


def residue_choice(st, choice, board, log=print, sweeps=4, src_out=None):
    """DST_RESIDUE=2: see the flag. The planner is called once with every
    residue net's collision-free candidate berths (plan['alts'], costed on
    the judge's scale: the berth's own vias and its ride) and the pairs
    of candidates that cannot both be laid (plan['alt_excl']); its
    level-5 solve chooses; the chosen berths are applied together and
    the full judge (no candidates) confirms -- a joint move that is not
    better on it is retried one net at a time. Sweeps until nothing
    improves. Returns the choice."""
    import time
    t0 = time.time()
    c0 = _spent()
    bands = pe.sm.bands_of_boxes(st['dboxes']) if st['dboxes'] else []

    def judge(ch, alts=None, excl=None, xing=None):
        plan = braid_plan_of(st, ch, board)
        if alts:
            plan['alts'] = alts
        # `excl` is NOT part of the crossing term -- it is the rule that
        # two berths cannot both be laid. Nesting it under `if xing:` meant
        # that on the DEFAULT path (DST_XING=0) the level-5 solve received
        # ZERO exclusion rows while the log still printed "N exclusion(s)",
        # so it was free to choose two berths sharing a lane.
        if excl:
            plan['alt_excl'] = list(excl)
        if xing:
            plan['alt_xing'] = xing
        PLAN_CALLS[0] += 1
        bp = te.plan_braid(board, list(ch), st['dref'], plan)
        f, pred, _bp, _pl = judge_by_braid(st, ch, board, bp=bp)
        res = [nm for nm in ch if bp.get(nm, {}).get('page') is None]
        return (len(res), f), res, pred, bp

    def ride_of(nm, m):
        return pe.sm.ride_mm({nm: m}, st['launch'], st['dboxes'], st['sgrid'].bbox)
    # the room each candidate barrel takes from the other escapes
    _contend = (em.site_contention(st['dmenu'], pe.sm.VIA_NEED_SITE)
                if DST_CONTEND else {})

    def contend_of(nm, m):
        """How many OTHER nets want the site this move's via would take."""
        if not DST_CONTEND or not m.site:
            return 0
        return _contend.get(nm, {}).get(
            (round(m.site[0], 3), round(m.site[1], 3)), 0)

    def _page_of(o, bp_):
        return (bp_.get(o, {}).get('page') if bp_ else None) or st['tooth0'].get(o)

    def swim_of(nm, m, sel, bp_, skip=()):
        """Inversions this berth makes with the lanes on its OWN page --
        the crossings that consume page capacity, and so the ones that
        decide whether a lane can be scheduled at all.

        `skip` is NOT optional in practice: an inversion belongs to TWO
        lanes, so charging each candidate a delta against the held choice
        counts the pair twice when both nets move, and measures it against
        a berth the other net is about to leave. That is the exact defect
        the crossing term was rewritten to fix (it drove K51 352 -> 394),
        and this term shipped without the lesson -- nets whose berth is
        also being chosen are priced pairwise or not at all.

        The page is the one this BERTH would put the net on, not the one
        the incumbent plan holds: a candidate that changes layer changes
        the answer, which is the whole quantity being measured.
        """
        if not DST_SWIM:
            return 0
        pg = m.layer or _page_of(nm, bp_)
        lo = st['launch'][nm]
        n = 0
        for o, om in sel.items():
            if o == nm or o in skip:
                continue
            if (om.layer or _page_of(o, bp_)) != pg:
                continue
            olo = st['launch'][o]
            if (lo[1] - olo[1]) * (m.exit_pt[1] - om.exit_pt[1]) < 0:
                n += 1
        return n

    def xing_of(nm, m, sel, skip=()):
        """How many of the OTHER lanes this net's lane would cross if it
        berthed at `m` -- the page capacity it consumes (see DST_XING).
        `skip` are the nets whose berth is ALSO being chosen: their
        crossings with this one are pairwise (alt_xing) and must not be
        charged here as well, or the pair is counted twice."""
        if not DST_XING:
            return 0
        a, b = tuple(st['launch'][nm]), tuple(m.exit_pt)
        return sum(1 for o, om in sel.items()
                   if o != nm and o not in skip
                   and lanes_cross(a, b, tuple(st['launch'][o]),
                                   tuple(om.exit_pt)))
    key0, res, pred, bp = judge(choice)
    f0, n0 = key0[1], key0[0]
    log(f'  residue choice: {len(res)} residue net(s) {res}, judged {f0:.2f}')
    if SRC_EXCHANGE:
        src_exchange_probe(st, choice, board, f0, log=log)
    src_done = set()           # nets whose source move this search chose (realized by the caller)
    r0_of = {nm: ride_of(nm, choice[nm]) for nm in choice}
    n_moves = n_solves = n_judged = 0
    # a berth a net has held in this search is not offered to it again:
    # the solve holds the other lines, and once a net has moved the berth
    # it left can look better in the new geometry than the judge finds it
    # (K41 sweep 1: SA8 proposed back, the judge refused, sweep 2 again)
    tabu = {(nm, sr.move_sig(m)) for nm, m in choice.items()}
    for sw in range(sweeps):
        if not res or _spent() - c0 > DST_RESIDUE_CALLS:
            break
        cands = {}
        n_all = 0
        joint = DST_RESIDUE >= 3
        pool = list(choice) if joint else res
        if joint and DST_RESIDUE_POOL != 'all':
            # the residue nets' candidates first (occupied berths allowed),
            # then the occupants they would displace join the pool
            occ = set()
            for nm in res:
                for m in st['dmenu'].get(nm, ()):
                    if m is choice[nm] or (nm, sr.move_sig(m)) in tabu:
                        continue
                    for o in choice:
                        if o != nm and pe.sm._conflict(m, choice[o], strict=False):
                            occ.add(o)
            pool = list(res) + sorted(o for o in occ if o not in res)
        # the nets whose berth this solve is CHOOSING: their crossings with
        # each other are priced pairwise (alt_xing), so the per-candidate
        # linear term must skip them or every such pair is charged twice
        _movers = set(pool)
        # a net whose menu is its laid berth alone (kept as copper) cannot
        # move; a candidate through such a net's berth is no candidate
        fixed_nets = {nm for nm in choice if len(st['dmenu'].get(nm, ())) <= 1}
        for nm in pool:
            cs = [m for m in st['dmenu'].get(nm, ())
                  if m is not choice[nm] and (nm, sr.move_sig(m)) not in tabu
                  and (joint or (pe.sm.lanes_free(m, choice, nm, strict=False, bands=bands)
                                 and not any(o != nm and pe.sm._conflict(m, choice[o], strict=False)
                                             for o in choice)))]
            if joint:
                cs = [m for m in cs
                      if not any(o != nm and o in fixed_nets and pe.sm._conflict(m, choice[o], strict=False)
                                 for o in choice)]
            n_all += len(cs)
            # one lane per GEOMETRY (exit point, layer, face): the moves
            # through one gap differ by kind and via site only, which the
            # lane does not see -- the cheapest of them stands for the rest
            # (K41: 48 candidate moves, 30 distinct lanes)
            r0 = ride_of(nm, choice[nm])
            by_geo = {}
            for m in cs:
                g = (round(m.exit_pt[0], 3), round(m.exit_pt[1], 3), m.layer, m.direction)
                c = ((m.vias - choice[nm].vias)
                     + (ride_of(nm, m) - r0) / pe.sm.VIA_MM
                     + DST_CONTEND * (contend_of(nm, m)
                                      - contend_of(nm, choice[nm]))
                     + DST_XING * (xing_of(nm, m, choice, _movers)
                                   - xing_of(nm, choice[nm], choice, _movers))
                     + DST_SWIM * (swim_of(nm, m, choice, bp, _movers)
                                   - swim_of(nm, choice[nm], choice, bp, _movers)))
                if g not in by_geo or c < by_geo[g][0]:
                    by_geo[g] = (c, m)
            if joint and by_geo:
                # one per face first (the cheapest), then round-robin over
                # the faces by nearness to the held berth's exit, each
                # face's remaining moves cheapest first
                by_f = {}
                for g, (c, m) in by_geo.items():
                    by_f.setdefault(m.direction, []).append((c, m))
                # ...or, on DST_XING_SCREEN, by the CROSSINGS each berth
                # makes against every other lane as it stands -- the whole
                # menu judged on the objective before the cap takes four
                rank = ((lambda t: (xing_of(nm, t[1], choice), t[0]))
                        if DST_XING_SCREEN else (lambda t: t[0]))
                keep = []
                for d in by_f:
                    by_f[d].sort(key=rank)
                    keep.append(by_f[d].pop(0))
                ex = choice[nm].exit_pt
                if DST_XING_SCREEN:
                    near = sorted(by_f, key=lambda d: rank(by_f[d][0]) if by_f[d] else (1e9, 1e9))
                else:
                    near = sorted(by_f, key=lambda d: min(math.hypot(m.exit_pt[0] - ex[0], m.exit_pt[1] - ex[1])
                                                          for _c, m in by_f[d]) if by_f[d] else 1e9)
                while len(keep) < DST_RESIDUE_CANDS and any(by_f.values()):
                    for d in near:
                        if by_f[d] and len(keep) < DST_RESIDUE_CANDS:
                            keep.append(by_f[d].pop(0))
                by_geo = {i: v for i, v in enumerate(keep)}
            if by_geo:
                cands[nm] = [m for _c, m in by_geo.values()]
        if not cands:
            log('  residue choice: no candidate berth for any residue net')
            break
        alts = {}
        for nm, cs in cands.items():
            r0 = ride_of(nm, choice[nm])
            k0 = contend_of(nm, choice[nm])
            x0 = xing_of(nm, choice[nm], choice, _movers)
            s0 = swim_of(nm, choice[nm], choice, bp, _movers)
            alts[nm] = [{'exit': list(m.exit_pt), 'layer': m.layer, 'dir': list(DIRS[m.direction]),
                         'cost': ((m.vias - choice[nm].vias)
                                  + (ride_of(nm, m) - r0) / pe.sm.VIA_MM
                                  + DST_CONTEND * (contend_of(nm, m) - k0)
                                  + DST_XING * (xing_of(nm, m, choice, _movers) - x0)
                                  + DST_SWIM * (swim_of(nm, m, choice, bp, _movers) - s0))}
                        for m in cs]
        src_cands = {}
        if DST_RESIDUE_SRC:
            # the residue nets' SOURCE moves (see DST_RESIDUE_SRC): one per
            # (face, layer), cheapest by vias then by ride from the new tooth
            for nm in res:
                if nm in src_done:
                    continue
                # one per (face, layer, EXIT ROW): a climb's whole point is
                # the row it leaves at (the launch order), so the climbs are
                # candidates of their own, not folded into the pad-row exit
                # (they were: at K41/climb 4 not one east-face climb reached
                # the solve). Cheapest first, the pad-row exit always kept.
                by_fl = {}
                for m in st['smenu'].get(nm, ()):
                    if (nm, sr.move_sig(m)) in tabu:
                        continue
                    ride1 = pe.sm.ride_mm({nm: choice[nm]}, {nm: m.exit_pt}, st['dboxes'], st['sgrid'].bbox)
                    c = (m.vias - st['tooth_vias'].get(nm, 0)) + (ride1 - r0_of[nm]) / pe.sm.VIA_MM
                    ax = 1 if m.direction in ('left', 'right') else 0
                    k = (m.direction, m.layer, round(m.exit_pt[ax], 2))
                    if k not in by_fl or c < by_fl[k][0]:
                        by_fl[k] = (c, m)
                base_ = {}
                for (d, L, _row), v in by_fl.items():
                    if (d, L) not in base_ or v[0] < base_[(d, L)][0]:
                        base_[(d, L)] = v
                ms = list(base_.values())
                for v in sorted(by_fl.values(), key=lambda t: t[0]):
                    if len(ms) >= SRC_RESIDUE_CANDS:
                        break
                    if not any(v[1] is k_[1] for k_ in ms):
                        ms.append(v)
                if ms:
                    src_cands[nm] = [m for _c, m in ms]
                    alts.setdefault(nm, [])
                    alts[nm] += [{'end': 'src', 'exit': list(m.exit_pt), 'layer': m.layer,
                                  'dir': list(DIRS[m.direction]), 'cost': c} for c, m in ms]
                    cands.setdefault(nm, [])
        excl = []
        names = list(cands)
        for ia, a in enumerate(names):
            for b in names[ia + 1:]:
                for i, ma in enumerate(cands[a], 1):
                    for j, mb in enumerate(cands[b], 1):
                        if pe.sm._conflict(ma, mb, strict=False):
                            excl.append([a, i, b, j])
        if joint:
            # a candidate through a neighbour's LAID berth: only if the
            # neighbour leaves it (its own laid lane is candidate 0)
            for a in names:
                for i, ma in enumerate(cands[a], 1):
                    for b in names:
                        if b != a and pe.sm._conflict(ma, choice[b], strict=False):
                            excl.append([a, i, b, 0])
        # THE CROSSING TERM, PAIRWISE (DST_XING). Two lanes need different
        # pages exactly when they cross, so the capacity a plan has is set by
        # its crossings -- but a crossing belongs to TWO lanes, and charging
        # each candidate a delta against the held choice double-counts the
        # pair when both nets move (measured: it drove K51 352 -> 394). Here
        # the pool-vs-FIXED crossings stay in the candidate's own linear cost
        # (exact, one lane varies) and the pool-vs-POOL ones become pairwise
        # costs the solve carries as z >= ya + yb - 1.
        alt_xing = []
        if DST_XING:
            movers = [nm for nm in alts if alts[nm]]
            opt = {nm: [choice[nm]] + [m for m in cands[nm]] for nm in movers}
            varying = []
            for ia in range(len(movers)):
                for ib in range(ia + 1, len(movers)):
                    a_, b_ = movers[ia], movers[ib]
                    hits = [(i, j) for i, ma in enumerate(opt[a_])
                            for j, mb in enumerate(opt[b_])
                            if lanes_cross(tuple(st['launch'][a_]), tuple(ma.exit_pt),
                                           tuple(st['launch'][b_]), tuple(mb.exit_pt))]
                    n_pair_opts = len(opt[a_]) * len(opt[b_])
                    # n_pair_opts, not n_all: "does this pair's crossing
                    # status VARY across the pair's own combinations" is a
                    # question about len(opt[a]) * len(opt[b]). The two
                    # shared one name, so the solve log's "(N moves)" -- a
                    # measured number -- printed the last pair's product.
                    if not hits or len(hits) == n_pair_opts:
                        continue            # constant for this pair: no variable
                    varying.append((abs(len(hits) / n_pair_opts - 0.5), a_, b_, hits))
            varying.sort(key=lambda t: t[0])        # most discriminating first
            n_em = 0
            for _d, a_, b_, hits in varying:
                if n_em + len(hits) > DST_XING_PAIRS:
                    continue
                for i, j in hits:
                    alt_xing.append((a_, i, b_, j, DST_XING))
                n_em += len(hits)
            log(f'    residue choice: crossing term: {len(movers)} mover(s), '
                f'{len(varying)} varying pair(s), {n_em} pairwise cost(s) emitted'
                + (f' (capped at {DST_XING_PAIRS})' if n_em >= DST_XING_PAIRS else ''))
        log(f'    residue choice: sweep {sw}: solving {sum(len(v) for v in alts.values())} candidate berth(s) '
            f'({n_all} moves) for {len(cands)} net(s), {len(excl)} exclusion(s)...')
        _k, _r, _p, bp2 = judge(choice, alts, excl, alt_xing)
        n_solves += 1
        moves = {nm: bp2[nm].get('alt') for nm in cands if bp2.get(nm, {}).get('alt')}
        smoves = {nm: j - len(cands[nm]) for nm, j in moves.items() if j > len(cands[nm])}
        moves = {nm: j for nm, j in moves.items() if nm not in smoves}
        if smoves:
            log('    residue choice: SOURCE move(s) chosen: '
                + ', '.join(f'{nm} tooth -> {sr.fmt_ask(src_cands[nm][j - 1])}' for nm, j in sorted(smoves.items())))
            # ONE source move at a time, the one the solve un-swims first
            # (else the costliest): eight realized together changed every
            # lane's neighbours, the engine refused three, and the round
            # graded 15 -> 18 residue (K41). The caller realizes it,
            # re-chooses on the new board, and keeps it only if the full
            # judge agrees; the destination moves chosen beside it are
            # applied below like any other sweep's.
            if src_out is not None:
                unswum = [nm for nm in smoves if not bp2.get(nm, {}).get('alt_w', True)]
                pick = (sorted(unswum) or sorted(smoves, key=lambda n: -pred.get(n, 0)))[0]
                src_out[pick] = src_cands[pick][smoves[pick] - 1]
                src_done.add(pick)
                # the berth moves chosen BESIDE the tooth are not applied
                # here: the solve chose them together, the re-plan on the
                # realized board re-derives them against the new comb, and
                # applying them first charged the tooth against a base that
                # had already taken its gain (K41: SA11 un-swum by both a
                # berth move and SA1's tooth; the tooth then read as +9)
                log(f'  residue choice: {n0} -> {key0[0]} residue, judged {f0:.2f} -> {key0[1]:.2f}, '
                    f'{n_moves} move(s) + 1 source move to realize ({pick}), {n_solves} solve(s), '
                    f'{n_judged} confirmed, {time.time() - t0:.0f} s')
                return choice
            if not moves:
                log(f'  residue choice: {n0} -> {key0[0]} residue, judged {f0:.2f} -> {key0[1]:.2f}, '
                    f'{n_moves} move(s), {n_solves} solve(s), {n_judged} confirmed, {time.time() - t0:.0f} s')
                return choice
        obj = next((bp2[nm].get('alt_obj') for nm in cands if bp2.get(nm, {}).get('alt_obj')), None)
        log(f'    residue choice: sweep {sw}: {sum(len(v) for v in alts.values())} candidate berth(s) '
            f'({n_all} moves) for {len(cands)} net(s), {len(excl)} exclusion(s); solve objective '
            + (f'{obj[0]:.2f} -> {obj[1]:.2f}' if obj else 'n/a') + f'; {len(moves)} move(s)')
        if not moves:
            break
        trial = dict(choice)
        for nm, j in moves.items():
            trial[nm] = cands[nm][j - 1]
        key1, res1, pred1, bp1 = judge(trial)
        n_judged += 1
        log('    residue choice: ' + ', '.join(f'{nm} {sr.fmt_ask(choice[nm])} -> {sr.fmt_ask(trial[nm])}'
                                              for nm in sorted(moves))
            + f': residue {key0[0]} -> {key1[0]}, judged {key0[1]:.2f} -> {key1[1]:.2f}')
        taken = None
        if accept_key(key1, key0, plan_lis(bp1) if LIS_GUARD else None,
                      plan_lis(bp) if LIS_GUARD else None):
            taken = (trial, key1, res1, pred1, bp1, len(moves))
        elif len(moves) > 1:
            # the joint move held the neighbours' lines; one net at a time
            # against the full judge, the best single move taken
            for nm, j in sorted(moves.items()):
                t1 = dict(choice)
                t1[nm] = cands[nm][j - 1]
                k1, r1, p1, b1 = judge(t1)
                n_judged += 1
                if accept_key(k1, key0, plan_lis(b1) if LIS_GUARD else None,
                              plan_lis(bp) if LIS_GUARD else None) \
                        and (taken is None or k1 < taken[1]):
                    taken = (t1, k1, r1, p1, b1, 1)
            if taken is not None:
                nm_ = next(nm for nm in moves if taken[0][nm] is not choice[nm])
                log(f'    residue choice: singly, {nm_}: residue {key0[0]} -> {taken[1][0]}, '
                    f'judged {key0[1]:.2f} -> {taken[1][1]:.2f}')
        if taken is None:
            log('    residue choice: not better on the full judge -- kept')
            break
        choice, key0, res, pred, bp, k_ = taken
        tabu |= {(nm, sr.move_sig(m)) for nm, m in choice.items()}
        n_moves += k_
        if src_out:
            break          # the pending source move is realized first; the next search continues
    log(f'  residue choice: {n0} -> {key0[0]} residue, judged {f0:.2f} -> {key0[1]:.2f}, '
        f'{n_moves} move(s), {n_solves} solve(s), {n_judged} confirmed, {time.time() - t0:.0f} s')
    return choice


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
        if SRC_REPLAN and not src_out:
            # ITEM 3: pick the tooth by RE-PLANNING, not by held lines. The
            # winner is handed to the same realize-and-confirm loop below,
            # so the engine still has the last word on whether it can be
            # laid as asked.
            _f0, _p0, _bp0, _pl0 = judge_by_braid(st, dst_choice, board)
            _key0 = (sum(1 for nm in dst_choice
                         if _bp0.get(nm, {}).get('page') is None), _f0)
            _res0 = [nm for nm in dst_choice
                     if _bp0.get(nm, {}).get('page') is None]
            _pick = src_replan_pick(st, dst_choice, board, _res0, _key0,
                                    log=print, tabu=banned)
            if _pick:
                src_out[_pick[1]] = _pick[2]
        if src_out and (DST_RESIDUE_SRC or SRC_REPLAN):
            # the choice solve asked for source moves: realize them with the
            # engine, choose the destination again on the new board, and
            # KEEP the new board only if the full judge (residue, then
            # cost) says it is better -- the choice solve's model is the
            # held-lines one, and a realized tooth changes every lane's
            # neighbours (K35: three unconfirmed rounds took the judged
            # cost 141 -> 147). A rejected round's moves are banned.
            def _key(ch):
                f_, _p, bp_, _pl = judge_by_braid(st, ch, board)
                return (sum(1 for nm in ch if bp_.get(nm, {}).get('page') is None), f_)
            best_key = _key(dst_choice)
            for _k in range(SRC_RESIDUE_ROUNDS):
                new_board = f'{work}_srcres{r}_{_k}.kicad_pcb'
                _free = []
                if SRC_REFAN_JOINT:
                    _pcb_now = parse_kicad_pcb(board)
                    _pin = set()
                    for _nm, _mv in src_out.items():
                        _mov, _pinned = sr.blockers_of(
                            _pcb_now, _mv, st['byname'][_nm][0], st['byname'],
                            set(names) - set(src_out))
                        _free += [b for b in _mov if b not in _free]
                        _pin |= set(_pinned)
                    if len(_free) > SRC_REFAN_MAX:
                        print(f'    joint re-fan: {len(_free)} blocker(s), capping at '
                              f'{SRC_REFAN_MAX}: {_free[SRC_REFAN_MAX:]} left in place')
                        _free = _free[:SRC_REFAN_MAX]
                    print(f'    joint re-fan: blockers of {sorted(src_out)} = '
                          f'{_free or "none"}'
                          + (f'; PINNED (outside the run, immovable): {sorted(_pin)}'
                             if _pin else ''))
                res_r = sr.realize(board, src_out, st['src_pad'], st['byname'], st['sref'], new_board,
                                   guard_names=names, free=_free,
                                   strict=bool(SRC_REFAN_STRICT))
                realized.append(res_r)
                misses = [nm for nm, e in res_r['audit'].items() if not e['exact']]
                for nm in misses:
                    banned.add((nm, sr.move_sig(src_out[nm])))
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
                # fixed (a one-move menu, sched_first's contract), so the
                # choice solve frees only what it decides to. A re-plan
                # from scratch re-decided every berth on the new board and
                # the whole-plan comparison then charged the tooth for the
                # greedy's other 20 changes (K41: SA1's realized tooth,
                # laid exactly and scheduled by the plain judge, graded
                # 14 -> 16 residue against a from-scratch re-plan)
                keep_sig = {nm: sr.move_sig(m) for nm, m in dst_choice.items()
                            if nm not in src_out and nm in st2['dmenu']
                            and any(sr.move_sig(mm) == sr.move_sig(m) for mm in st2['dmenu'][nm])}
                ch2, un2 = dest_choice(st2, new_board, src_out=src2, fixed=keep_sig)
                if not ch2:
                    print(line + '; no destination choice on the new board -- reverted'); break
                f2, _p2, bp2, _pl2 = judge_by_braid(st2, ch2, new_board)
                key2 = (sum(1 for nm in ch2 if bp2.get(nm, {}).get('page') is None), f2)
                if key2 < best_key:
                    print(line + f'; judged residue {best_key[0]} -> {key2[0]}, cost {best_key[1]:.2f} -> {f2:.2f}: KEPT')
                    board, st, dst_choice, un, best_key, src_out = new_board, st2, ch2, un2, key2, src2
                    if SRC_REPLAN and not src_out:
                        _res = [nm for nm in dst_choice
                                if bp2.get(nm, {}).get('page') is None]
                        _pk = src_replan_pick(st, dst_choice, board, _res,
                                              best_key, log=print, tabu=banned)
                        if _pk:
                            src_out[_pk[1]] = _pk[2]
                else:
                    print(line + f'; judged residue {best_key[0]} -> {key2[0]}, cost {best_key[1]:.2f} -> {f2:.2f}: '
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
    names = coherent_nets(K, base)
    print('planning (source realized every round)...')
    work = out_path[:-len('.kicad_pcb')] if out_path.endswith('.kicad_pcb') else out_path
    choice, dst_pad, dref, byname, board, realized, banned = plan(base, names, work)
    lay = fanout_equivalent if SF_EQUIV else fanout_destination
    return lay(out_path, names, choice, dst_pad, dref, byname, board, realized, banned)


def fanout_destination(out_path, names, choice, dst_pad, dref, byname, board,
                       realized, banned):
    """Fan out DU1 to the plan, audit, and FEED BACK: a berth the engine
    could not lay as asked leaves that net's menu, the destination is
    re-selected against the same teeth, and the fanout runs again --
    until every berth is exactly the plan's, or nothing changes. The
    plan's own via model is printed for the plan that ships."""
    st = plan_state(parse_kicad_pcb(board), names, banned)
    laid_pass = None       # the LAST pass fanned out: (choice, st, achieved, ok)
    learned = set()        # SF_LEARN: pairs of moves the engine would not lay together
    for it in range(DST_ITERS):
        faces = [m.direction for m in choice.values()]
        print(f'\nplan (destination pass {it}): {len(choice)} berth escape directions '
              + ', '.join(f'{d}:{faces.count(d)}' for d in sorted(set(faces)))
              + f'  (source board: {os.path.basename(board)}, '
              f'{len(realized)} realized round(s), {len(banned)} banned move(s))')
        ask = None
        if DST_FACE_ASK:
            # DST_FACE_ASK (2026-09-10): a net the selector left without a
            # free lane reaches the engine as a PLANNED ball with a face-only
            # ask (the face most of its menu leaves by), laid in the plan-
            # follow order where an exact ask blocked by it negotiates --
            # rips it, lays exactly, re-lays it -- instead of in Phase A,
            # which escaped it BEFORE the planned balls and took a planned
            # gap (K41: SRAS's and SA12's asked lanes, 3 passes for 1 net)
            ask = {}
            for nm in names:
                if nm in choice or nm not in st['dmenu'] or not st['dmenu'][nm]:
                    continue
                faces = collections.Counter(m.direction for m in st['dmenu'][nm] if m.vias == 0) \
                    or collections.Counter(m.direction for m in st['dmenu'][nm])
                ask[nm] = faces.most_common(1)[0][0]
            if ask:
                print(f'  face asks for the unplanned: {ask}')
        laid, audit_d, ok = fanout_once(out_path, names, choice, dst_pad, dref,
                                        byname, board, face_asks=ask or None)
        if os.environ.get('DST_KEEP_PASSES'):
            # each pass's fanout board kept beside the output (diagnosis of
            # the asks the engine would not lay: what occupied the gap)
            shutil.copy(out_path, out_path[:-len('.kicad_pcb')] + f'_pass{it}.kicad_pcb')
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
        paired = []
        for nm in misses:
            m = choice[nm]
            if SCHED_FIRST and SF_LEARN:
                # SF_LEARN (2026-09-11): a refusal is a PAIR when the asked
                # exit gap is held by another net's laid stub -- the planner
                # then avoids the two moves TOGETHER and keeps both; banning
                # the move alone threw away a berth that was fine with other
                # neighbours (K41: 12 refusals on the 8th pass, half of them
                # gaps another net had taken)
                ax = 0 if m.direction in ('up', 'down') else 1
                got = getattr(fanout_once, 'achieved', {}) or {}
                occ = [o for o, g in got.items() if o != nm and o in choice and g
                       and g['direction'] == m.direction
                       and abs(g['tooth'][ax] - m.exit_pt[ax]) <= sr.GAP_TOL   # the audit's own 'same gap'
                       and audit_d.get(o, {}).get('exact')]
                if occ:
                    for o in occ:
                        learned.add(frozenset((sr.move_sig(m), sr.move_sig(choice[o]))))
                    paired.append(f'{nm}x{"/".join(occ)}')
                    continue
            banned.add((nm, sr.move_sig(m)))
        print(f'  destination pass {it}: {len(misses)} berth(s) not laid as asked '
              f'-> banned, re-planning the destination: {misses}'
              + (f'; learned as pairs: {paired}' if paired else ''))
        st = plan_state(parse_kicad_pcb(board), names, banned)
        # the berths laid exactly stay as laid (schedule-first re-plans
        # the refused nets around them; the greedy re-selects everything)
        laid_ok = {nm: sr.move_sig(choice[nm]) for nm in choice
                   if audit_d.get(nm, {}).get('exact')}
        new_choice, un = dest_choice(st, board,
                                     fixed=laid_ok if (SCHED_FIRST and SF_FIXED) else None,
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
    return 0 if ok_l else 1


def achieved_move(st, nm, g):
    """The berth a fanout LAID, as a Move the planner can price: the menu
    move of its class (kind, face, layer) nearest its stub end along the
    face (_menu_match -- a berth the engine walked a gap is the menu's move
    at that gap, so its signature is one the planner knows), else a move
    made of what was measured (one leg, ball to stub end) for copper the
    menu does not name. None for a net with no berth."""
    if not g:
        return None
    m = _menu_match(st['dmenu'].get(nm, []), g)
    if m is not None:
        return m
    p = st['dst_pad'][nm]
    return em.Move(net=nm, kind=g['kind'], direction=g['direction'], layer=g['layer'],
                   exit_pt=tuple(g['tooth']), vias=g['vias'],
                   legs=[((p.global_x, p.global_y), tuple(g['tooth']), g['layer'])],
                   site=(tuple(g['site']) if g.get('site') else None))


def menus_against(st, board_path, free, banned):
    """The freed nets' menus enumerated against the LAID board: every
    other net's copper -- the kept berths first of all -- is an obstacle,
    the freed nets' own stubs are not (the relay strips them). The same
    obstacle model the plan prices every move with (braid.build_obstacles:
    foreign pads, segments and vias inflated by clearance and half a
    track); the dogbone sites are checked the same way. plan_state's
    destination menu is the BARE array's, and a re-plan from it kept
    asking for moves across the kept stubs -- K41: SDQ15 refused on all 8
    passes, 25 moves banned one at a time (2026-09-11)."""
    pcb = parse_kicad_pcb(board_path)
    byname = st['byname']
    kids = {byname[n][0] for n in free}
    cache = {}

    def obs(nid, layer):
        key = (nid, layer)
        if key not in cache:
            cache[key] = te.build_obstacles(pcb, nid, kids | {nid}, layer)
        return cache[key]
    out = {}
    for nm in free:
        nid = byname[nm][0]
        pad = st['dst_pad'][nm]
        grid = em.grid_of(pcb.footprints[pad.component_ref])
        moves = em.enumerate_moves(
            pad, grid, LAYERS,
            lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
            lambda p, L, _n=nid: not (obs(_n, L).point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0],
            walk=DST_WALK, walk_off=DST_WALK_OFF if DST_WALK else 0)
        out[nm] = _force(FORCE_DST, nm,
                         [m for m in moves if (nm, sr.move_sig(m)) not in banned], 'destination')
    return out


def fanout_equivalent(out_path, names, choice, dst_pad, dref, byname, board,
                      realized, banned):
    """SF_EQUIV (2026-09-11): the destination laid to be EQUIVALENT to the
    plan under the PLANNER'S OWN judge, not exact gap by gap.

    Pass 0 fans out the whole plan. Every pass is then judged by
    sched_first.judge on the berths the board actually holds (matched to
    menu moves): the least cost of seating every laid berth on the front
    chain, the back chain or a swim -- the objective the planner minimised
    -- so the plan and the copper are read on ONE scale. A berth the judge
    SEATS is kept: fixed as laid, copper the next pass cannot displace. Only
    the nets the judge swims, the nets with no copper and the nets in a DRC
    pair are FREED: re-planned by the same planner with the kept berths
    fixed (a one-move menu each), then re-laid alone on the kept board (the
    incremental relay pass, which strips and re-fans those nets only).

    Why this and not the exact-gap loop (fanout_destination): that loop
    banned every berth the audit called inexact and re-fanned the whole
    array from bare, so a berth the engine walked one gap along its face
    (schedule-equivalent) was thrown away, a berth ripped by a neighbour's
    negotiation trade was banned as if the plan were infeasible, and each
    pass's engine run reshuffled the rest (K41 joint plan: 8-20 refusals
    on every one of 8 passes, the shipped plan the greedy's).

    Rules that make it converge: a freed net's ask that a RELAY pass could
    not lay exactly leaves its menu (the refusal is against static copper
    now, the kept berths included); on pass 0 nothing is banned (a miss
    there may be the engine's own trade, retried once with the neighbours
    fixed); a DRC pair between two of the run's nets is learned as a move
    pair the planner avoids (both freed), a pair with foreign copper bans
    the move; a net the judge swims at its own exact berth after a re-plan
    is accepted as the plan-equivalent outcome (the DP had every
    alternative and kept it); a re-plan that changes nothing stops the
    loop. The last pass's board ships, with its plan sidecar."""
    import sched_first
    JUDGE = os.environ.get('SF_JUDGE', 'plan')

    def judge_of(st_, moves, achieved):
        """The loop's judge of one assignment. 'plan' (default): the
        planner's own objective, sched_first.judge. 'braid': the braid's
        own planner (judge_by_braid -- its corridors, orders, pages and
        per-net vias), the model that tracks the routed copper within 3
        vias at K15/K28 where the planner's sat 10-16 below it (measured
        2026-09-11: greedy K28 laid exactly, planner judge 7 swimmers,
        re-berthed 5, braid 36 -> 40 vias). Same keys either way."""
        if JUDGE != 'braid':
            return sched_first.judge(st_, moves)
        mv = {nm: m for nm, m in moves.items() if m is not None}
        cost, pred, bp, _plan = judge_by_braid(st_, mv, board, achieved)
        return {'cost': cost, 'page': {nm: bp[nm]['page'] for nm in mv if nm in bp},
                'skipped': [nm for nm in mv if nm in bp and bp[nm]['page'] is None],
                'vias': sum(pred.values()), 'n': len(mv),
                'unlaid': [nm for nm, m in moves.items() if m is None]}
    st = plan_state(parse_kicad_pcb(board), names, banned)
    plan_j = judge_of(st, {nm: choice.get(nm) for nm in names}, None)
    print(f'  planner judge of the PLAN: cost {plan_j["cost"]:.1f}, '
          f'{len(plan_j["skipped"])} swimmer(s) {plan_j["skipped"]}, vias {plan_j["vias"]}'
          + (f', {len(plan_j["unlaid"])} unplanned {plan_j["unlaid"]}' if plan_j['unlaid'] else ''))
    learned = set()           # {frozenset({sig_a, sig_b})}: move pairs the board refused (a DRC pair)
    kept = {}                 # net -> the laid Move the judge seated
    accepted = set()          # nets kept at their laid berth whatever the judge says next
    free = list(names)
    ask_choice = dict(choice)
    prev = None
    final = None
    for it in range(DST_ITERS):
        sub = {nm: ask_choice[nm] for nm in free if nm in ask_choice}
        ask = None
        if DST_FACE_ASK:
            ask = {}
            for nm in free:
                if nm in sub or not st['dmenu'].get(nm):
                    continue
                fs = (collections.Counter(m.direction for m in st['dmenu'][nm] if m.vias == 0)
                      or collections.Counter(m.direction for m in st['dmenu'][nm]))
                ask[nm] = fs.most_common(1)[0][0]
        faces = [m.direction for m in sub.values()]
        print(f'\nplan (destination pass {it}): {len(sub)} berth(s) to lay '
              + ', '.join(f'{d}:{faces.count(d)}' for d in sorted(set(faces)))
              + f'; {len(kept)} kept as laid, {len(banned)} banned move(s), '
              f'{len(learned)} learned pair(s)'
              + (f'; face asks for the unplanned: {ask}' if ask else ''))
        laid, audit_d, ok = fanout_once(out_path, names, sub, dst_pad, dref, byname, board,
                                        relay=(free if it > 0 else None),
                                        already=tuple(kept), face_asks=ask or None)
        if os.environ.get('DST_KEEP_PASSES'):
            shutil.copy(out_path, out_path[:-len('.kicad_pcb')] + f'_pass{it}.kicad_pcb')
        got = getattr(fanout_once, 'achieved', None) or {}
        ach = {nm: achieved_move(st, nm, got.get(nm)) for nm in names}
        j = judge_of(st, ach, got)
        drc_pairs = getattr(fanout_once, 'drc_pairs', set())
        drc = {nm for pr in drc_pairs for nm in pr if nm in ach}
        print(f'  destination pass {it}: planner judge of the LAID board: cost {j["cost"]:.1f} '
              f'(plan {plan_j["cost"]:.1f}), {len(j["skipped"])} swimmer(s) {j["skipped"]}, '
              f'vias {j["vias"]} (plan {plan_j["vias"]})'
              + (f'; unlaid {j["unlaid"]}' if j['unlaid'] else '')
              + (f'; DRC nets {sorted(drc)}' if drc else ''))
        if SF_ROUTE_SCREEN:
            # THE ROUTE'S OWN VERDICT on the board this pass just laid --
            # attempt 0 of the real braid, no rip ladder. Measure-only at
            # SF_ROUTE_SCREEN=1: what the surrogate calls a swimmer and
            # what the router actually REFUSES are different sets, and the
            # refused set is the one carrying ~80% of the judge's error.
            _ref = route_screen(out_path, names, dref)
            if _ref is None:
                print(f'  destination pass {it}: route screen unavailable '
                      f'(budget {_SCREEN_CALLS[0]}/{SF_ROUTE_SCREEN_CALLS})')
            else:
                _sw = set(j['skipped'])
                _rf = set(_ref)
                print(f'  destination pass {it}: ROUTE SCREEN refused {len(_rf)} '
                      f'{sorted(_rf)}; planner swimmers {len(_sw)}; '
                      f'refused-not-swimmer {sorted(_rf - _sw)}; '
                      f'swimmer-not-refused {sorted(_sw - _rf)}')
        final = (dict(kept), dict(sub), st, got, ok)
        # AN ASK THE ENGINE ANSWERED WITH A STRUCTURALLY DIFFERENT BERTH is
        # banned here, whatever the judge then makes of the net. Until now a
        # non-exact ask was only banned when the net ALSO came back a
        # swimmer, so a berth the engine had already called "infeasible even
        # alone" was asked for again every pass -- K51 asked SDQ11 and SDQM1
        # for the same dogbone on B three passes running, got the same
        # surface stub on F a layer and a kind away each time, and the lane
        # that had to reach that degraded berth was one of the three left
        # open. Losing only the GAP is a berth a hair along its own face and
        # is not banned; losing the LAYER or the KIND is a different berth.
        if DST_ASK_BAN:
            for nm in names:
                a_ = sub.get(nm)
                lost = audit_d.get(nm, {}).get('lost') or []
                if a_ is not None and ({'layer', 'kind', 'face'} & set(lost)):
                    sig = sr.move_sig(a_)
                    if (nm, sig) not in banned:
                        banned.add((nm, sig))
                        print(f'  destination pass {it}: {nm} asked a berth the engine '
                              f'answered a {"/".join(lost)} away -- not asked again')
        new_free, why = [], {}
        for nm in names:
            asked = sub.get(nm)
            exact = bool(audit_d.get(nm, {}).get('exact'))
            if ach[nm] is None:
                new_free.append(nm); why[nm] = 'no copper'
                accepted.discard(nm)
                if asked is not None and it > 0:
                    banned.add((nm, sr.move_sig(asked)))
            elif nm in drc:
                new_free.append(nm); why[nm] = 'DRC'
                accepted.discard(nm)
            elif nm in j['skipped']:
                if nm in accepted:
                    why[nm] = 'swims, accepted earlier'
                elif asked is not None and exact and it > 0:
                    why[nm] = 'swims at its exact berth after a re-plan: accepted'
                    accepted.add(nm)
                else:
                    new_free.append(nm)
                    why[nm] = 'judged a swimmer' + ('' if asked is None or exact else ' (laid off its ask)')
                    if asked is not None and not exact and it > 0:
                        banned.add((nm, sr.move_sig(asked)))
        for pr in drc_pairs:
            ns = [nm for nm in pr if nm in ach and ach[nm] is not None]
            if len(ns) == 2 and SCHED_FIRST:
                learned.add(frozenset((sr.move_sig(ach[ns[0]]), sr.move_sig(ach[ns[1]]))))
            else:
                for nm in ns:
                    banned.add((nm, sr.move_sig(ach[nm])))
        kept = {nm: ach[nm] for nm in names if ach[nm] is not None and nm not in new_free}
        acc_now = [nm for nm in names if why.get(nm, '') == 'swims at its exact berth after a re-plan: accepted']
        if acc_now:
            print(f'  destination pass {it}: accepted as swimmers at their own berths: {acc_now}')
        if not new_free:
            print(f'  destination pass {it}: the laid board is plan-equivalent by the judge -- done')
            break
        print(f'  destination pass {it}: {len(new_free)} net(s) freed -- '
              + ', '.join(f'{nm} ({why[nm]})' for nm in new_free)
              + f'; {len(kept)} kept as laid')
        st = plan_state(parse_kicad_pcb(board), names, banned)
        for nm, m in kept.items():
            st['dmenu'][nm] = [m]      # a kept berth is its net's whole menu
        # the freed nets' menus against the kept copper, not the bare array
        st['dmenu'].update(menus_against(st, out_path, new_free, banned))
        fixed = {nm: sr.move_sig(m) for nm, m in kept.items()}
        new_choice, un = dest_choice(st, board, fixed=fixed, learned=learned)
        ask_choice = {nm: new_choice[nm] for nm in new_free if nm in new_choice}
        # a freed net the re-plan seats at the berth it already holds is
        # laid already: accepted, not re-laid
        same = [nm for nm in new_free if ach[nm] is not None and nm not in drc
                and nm in ask_choice and sr.move_sig(ask_choice[nm]) == sr.move_sig(ach[nm])]
        for nm in same:
            accepted.add(nm)
            kept[nm] = ach[nm]
        relay = [nm for nm in new_free if nm not in same]
        key = (tuple(relay), tuple(sorted((nm, sr.move_sig(ask_choice[nm])) for nm in relay if nm in ask_choice)))
        rj = judge_of(st, {nm: ({**kept, **ask_choice}).get(nm) for nm in names},
                      {nm: got[nm] for nm in kept if got.get(nm)})
        print(f'  destination re-plan: {len(ask_choice)} of {len(new_free)} freed net(s) placed'
              + (f', unplaced {[n for n in new_free if n not in ask_choice]}'
                 if any(n not in ask_choice for n in new_free) else '')
              + (f'; {len(same)} re-seated where laid {same}' if same else '')
              + f'; planner judge cost {rj["cost"]:.1f}, {len(rj["skipped"])} swimmer(s) {rj["skipped"]}, '
              f'vias {rj["vias"]}')
        if not relay:
            print(f'  destination pass {it}: nothing left to re-lay -- done')
            break
        if key == prev:
            print('  destination: the re-plan changed nothing -- stopping')
            break
        prev = key
        free = relay
    kept_l, sub_l, st_l, got_l, ok_l = final
    choice_l = {**kept_l, **sub_l}
    # the planner's judge per net on the SHIPPED berths, beside the braid's
    # own model printed by explain_plan: where the two disagree is where
    # the copper leaves the plan
    ach_l = {nm: achieved_move(st_l, nm, got_l.get(nm)) for nm in names}
    jl = judge_of(st_l, ach_l, got_l)
    print(f'  planner judge of the SHIPPED board: cost {jl["cost"]:.1f}, vias {jl["vias"]}, '
          f'{len(jl["skipped"])} swimmer(s) {jl["skipped"]}')
    print('    judge page F: ' + ' '.join(nm for nm in names if jl['page'].get(nm) == 'F.Cu'))
    print('    judge page B: ' + ' '.join(nm for nm in names if jl['page'].get(nm) == 'B.Cu'))
    explain_plan(choice_l, st_l, names, out_path, out_path, achieved=got_l)
    complete = all(got_l.get(nm) for nm in names)
    return 0 if (ok_l and complete) else 1


def block_view(pcb, dref, block):
    """The board as the engine must see ONE block of `dref`: the
    footprint under `dref` holds the block's pads alone (its grid, its
    boundary, its faces), and the other blocks' balls stand beside it as
    a part of their own (`<dref>~rest`) -- foreign copper a via must
    clear and a track must not cross, which is what they are to this
    block's escapes. Returns the two footprints to restore."""
    import copy as _copy
    fp = pcb.footprints[dref]
    mine = {id(p) for p in block.pads}
    vfp = _copy.copy(fp)
    vfp.pads = [p for p in fp.pads if id(p) in mine]
    rest = _copy.copy(fp)
    rest.reference = f'{dref}~rest'
    rest.pads = [p for p in fp.pads if id(p) not in mine]
    pcb.footprints[dref] = vfp
    pcb.footprints[rest.reference] = rest
    return fp, rest.reference


def fanout_blocks(pcb, dref, blocks, targets, dst_pad, hints, src_file, out_path):
    """The destination fanned out in phases (SPLIT_BLOCKS): first the
    WHOLE array for every net whose move is not a band move (the outer
    faces and the through-runs, the engine's deepest-first claim order
    as ever), then each BLOCK for the nets whose planned exit lies in a
    band -- the block is the array of that call, on a board that carries
    the copper laid before it (written and parsed back between calls:
    the engine reads its obstacles off the board it is given). Returns
    the union of the calls' results."""
    all_t, all_va, all_vr, all_f = [], [], [], []
    pos = {nm: (round(p.global_x, 3), round(p.global_y, 3)) for nm, p in dst_pad.items()}
    ax0, ay0, ax1, ay1 = em.grid_of(pcb.footprints[dref]).bbox

    def in_band(h):
        e = h.get('exit') if isinstance(h, dict) else None
        return e is not None and ax0 < e[0] < ax1 and ay0 < e[1] < ay1
    band_pos = {q for q, h in hints.items() if in_band(h)}
    band_nets = [nm for nm in targets if pos.get(nm) in band_pos]
    rest_nets = [nm for nm in targets if nm not in band_nets]
    stage = out_path[:-len('.kicad_pcb')] + '.block.tmp'
    n2n = {i: n.name for i, n in pcb.nets.items()}
    cur = pcb
    if rest_nets:
        print(f'  whole array: {len(rest_nets)} net(s) {rest_nets}')
        t, va, vr, f = generate_bga_fanout(
            cur.footprints[dref], cur, net_filter=rest_nets, layers=list(LAYERS),
            track_width=0.1, clearance=0.1, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
            exit_margin=0.5, escape_method='underpad', plane_drop='off',
            escape_dir_hints={q: h for q, h in hints.items() if q not in band_pos})
        all_t += t
        all_va += va
        all_vr += vr
        all_f += f
    for block in blocks:
        mine = {(round(p.global_x, 3), round(p.global_y, 3)) for p in block.pads}
        nets = [nm for nm in band_nets if pos.get(nm) in mine]
        if not nets:
            continue
        if all_t:
            # the copper laid so far, on the board the engine reads
            add_tracks_and_vias_to_pcb(src_file, stage, all_t, all_va, all_vr,
                                       net_id_to_name=n2n)
            cur = parse_kicad_pcb(stage)
            cur._fanout_all_foreign_immovable = True
        blk = em.block_of(block.pads[0], em.blocks_of(cur.footprints[dref]))
        fp0, rest_ref = block_view(cur, dref, blk)
        print(f'  block {block.cell}: {len(blk.pads)} balls, {len(nets)} band net(s) '
              f'{nets}')
        try:
            t, va, vr, f = generate_bga_fanout(
                cur.footprints[dref], cur, net_filter=nets, layers=list(LAYERS),
                track_width=0.1, clearance=0.1, via_size=te.VIA_SIZE,
                via_drill=te.VIA_DRILL, exit_margin=BAND_EXIT,
                escape_method='underpad', plane_drop='off',
                escape_dir_hints={q: h for q, h in hints.items() if q in mine})
        finally:
            cur.footprints[dref] = fp0
            cur.footprints.pop(rest_ref, None)
        all_t += t
        all_va += va
        all_vr += vr
        all_f += f
    if os.path.exists(stage):
        os.remove(stage)
    return all_t, all_va, all_vr, all_f


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
            if SF_EQUIV >= 2:
                # STRICT plan-follow (underpad._follow_plan): a negotiation
                # is kept only when the count of exact berths rises, and a
                # ball with no berth on its asked face is left unescaped
                # for the planner's re-plan, never dumped on another face
                hints[(round(p.global_x, 3), round(p.global_y, 3))]['strict'] = True
        elif face_asks and nm in face_asks:
            p = dst_pad[nm]
            # a face-only ask. As a bare string the engine escapes the ball in
            # its generic Phase A, BEFORE the planned balls (K41: SBA2's 8.5 mm
            # band-edge walk blocked seven planned exact legs); under
            # DST_FACE_ASK it goes as a dict hint, a PLANNED ball with no exit,
            # which the plan-follow lays after every exact ask
            hints[(round(p.global_x, 3), round(p.global_y, 3))] = (
                {'face': face_asks[nm]} if DST_FACE_ASK else face_asks[nm])
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
    blocks = em.blocks_of(pcb.footprints[dref]) if SPLIT_BLOCKS else []
    if len(blocks) > 1:
        tracks, vias_add, vias_rm, failed = fanout_blocks(
            pcb, dref, blocks, targets, dst_pad, hints, src_file, out_path)
    else:
        tracks, vias_add, vias_rm, failed = generate_bga_fanout(
            pcb.footprints[dref], pcb, net_filter=targets, layers=list(LAYERS),
            track_width=0.1, clearance=0.1, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
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
    drc_pairs = set()       # the PAIRS too (SF_EQUIV learns a pair of the run's nets as a move pair)
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
