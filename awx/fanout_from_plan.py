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
       fanout_from_plan.py OUT.kicad_pcb --hold    (FANOUT_JOINT: OUT's joint copper held to its passives, joint_hold)
"""
import math
import contextlib
import io
import json
import os
import awx_settings
import shutil
import subprocess
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
import ship_vias  # noqa: E402  a via in a pad declares Type VII (#962)
from bga_fanout import generate_bga_fanout  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
import detect_buses as db  # noqa: E402
import plan_ends as pe  # noqa: E402
import source_realize as sr  # noqa: E402
from coherent_nets import coherent_nets  # noqa: E402
import rules as _rules  # noqa: E402  ONE source for every design rule
import route_layers  # noqa: E402  the routing layers: the escapes' runs may take any of them

# SRC_CLIMB=k (2026-09-10): the SOURCE menu also offers CLIMBS -- a dog-bone
# or via-in-pad whose run first travels up to k pitches along a gap under
# the array and leaves the face at a chosen row or column (the human's
# north riders at K51; escape_moves.enumerate_moves climb=). 0 = off, the
# menu byte-identical. replan.py runs with 14.
SRC_CLIMB = int(awx_settings.get('SRC_CLIMB', '0'))
# PLAN_PAGES=1 (2026-09-13): the PAGES-FIRST planner (pages_first.py) chooses
# BOTH ends and the page of every net in one CP-SAT with hard two-page
# planarity, so no net needs more than two vias by construction. 0 = the
# recorded planner, byte-identical. DST_CLIMB=k enumerates destination
# dog-bones whose run climbs along the array up to k pitches before it
# leaves (escape_moves climb=), the class that makes a B berth's rank free.
PLAN_PAGES = int(awx_settings.get('PLAN_PAGES', '0'))
DST_CLIMB = int(awx_settings.get('DST_CLIMB', '0'))
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
PLAN_JUDGE = awx_settings.get('PLAN_JUDGE', '')
# DST_STREET=k: destination STREET dog-bones (escape_moves street=) -- a via in an empty band of the destination
# array, on a lane a track pitch from the next, at k sites along it, the run leaving toward the source. On (2) under
# the whole route's ends: the braid they give is far simpler (K35: 132 crossings against 162, the solve 10 s against
# 121, the whole run 287 s against 606), at 58 vias against 52; 0 = off
DST_STREET = int(awx_settings.get('DST_STREET', '2' if PLAN_JUDGE == 'ends' else '0'))
PLAN_JUDGE_RIDE = int(awx_settings.get('PLAN_JUDGE_RIDE', '1') or 0)
# PLAN_JUDGE_LEN: the length ESTIMATOR the judge prices at VIA_MM -- 'lane' (the
# braid's planned polylines + berth runs; default) or 'ride' (the around-box
# ride from launch to berth exit, ride_mm: the jcr arm, 3x over on the K35 batch)
PLAN_JUDGE_LEN = awx_settings.get('PLAN_JUDGE_LEN', 'ride')
if PLAN_JUDGE not in ('', 'count', 'flat', 'ends'):
    raise SystemExit(f'PLAN_JUDGE={PLAN_JUDGE!r}: expected count | flat | ends | unset')
# PLAN_JUDGE=ends: the whole route's own ENDS model (whole_ends.py) both CHOOSES the berths and the teeth to move (in
# place of pages_first) and JUDGES a candidate on the ends model's objective (whole_ends: the ends' vias and the
# route's, estimated from the ends and ranked exact on their orders, the ride, congestion and feedback), in seconds,
# with no whole solve and no braid planner. A realized board is judged on its teeth AS LAID.


from escape_moves import DIRS, LAYERS  # noqa: E402,F401  -- ONE source


FAST_PRO = False   # True (a probe's intermediate boards): the sidecar copied, not re-scanned


def copy_pro(src_board, dst_board):
    # every sibling: the project, and the .kicad_dru's per-layer rules (#498) a bus step on inner layers must keep
    from copy_board import copy_siblings
    copy_siblings(src_board, dst_board)
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
    if awx_settings.get('AWX_STAMP_PRO', '1') == '0':   # the flag-off parity control
        return
    try:
        from fix_kicad_drc_settings import fix_project_for_output
        fix_project_for_output(dst_board, src_board, clearance=te.SPEC_CLEARANCE,
                               track_width=sr.FAN_TRACK,
                               via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL,
                               verbose=False)
    except Exception as e:
        print(f'  project floor NOT stamped: {e}', flush=True)


ROUNDS = int(awx_settings.get('SRC_ROUNDS', '8'))   # realized source rounds (feasibility bans need re-plans); 0 = the teeth as they stand
DST_ITERS = 8  # destination select -> fan out -> audit -> ban -> re-select


def _load_force(var):
    """PLAN_FORCE_DST / PLAN_FORCE_SRC (a PROBE, 2026-09-11, ported back
    from handoff_0910b): a JSON file {net: {"direction": .., "layer": ..,
    "kind": ..}} restricts that net's menu at that end to the class named
    (any subset of the three keys). Written by tmp/human_sides.py off the
    human's copper, it measures the plan's headroom -- what the braid does
    on our teeth with the human's destination classes -- not a mechanism."""
    path = awx_settings.get(var, '')
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


def via_escapes(moves, end):
    """`moves` (at `end`, 'src' or 'dest') less the surface escapes under ESCAPE_VIAS (route_layers.escape_vias) -- all
    of them, where none is left"""
    import route_layers
    if not route_layers.escape_vias(end):
        return moves
    return [m for m in moves if m.kind != 'surface'] or moves


def plan_state(pcb, names, banned=frozenset()):
    """Everything the plan reads off ONE board: the menus of legal escapes
    at both ends, the launch points (the source teeth AS THEY ARE on this
    board), the layer each tooth ends on, the taut-path buses. `banned`
    holds (net, move signature) pairs the fanout has REFUSED to lay as
    asked: the plan's model said they were possible, the engine said no,
    and the engine is the authority -- they leave the menus."""
    plan_state._pair_legs = None
    _nol = sorted(set(route_layers.layers()) - set(pcb.board_info.copper_layers or ()))
    if _nol:
        raise SystemExit(f'fanout: ROUTE_LAYERS names {", ".join(_nol)}, which the board has not (its copper layers: '
                         f'{", ".join(pcb.board_info.copper_layers or ())})')
    # the joint fanout (FANOUT_JOINT): the arrays' other balls keep their own via sites, which the menus leave free
    import joint_escape as _je
    _je.reserve_ball_vias(pcb)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ends = te.endpoints(pcb, names, byname)
    kids = {byname[n][0] for n in names}
    # a pair's exit room is checked against copper OUTSIDE the run under the ends model, whose search moves the run's
    # stubs and tests their options against the pair's (whole_ends); the other judges read the laid stubs as fixed
    run_free = kids if PLAN_JUDGE == 'ends' else ()
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

    # a bus escape on the outer layers (route_layers.escape_layers: a via's run on an inner layer is the solve's), its
    # via clear on every copper layer it stands through where the run may take an inner one (an earlier round's run
    # moved there stands in it)
    via_layers = (list(pcb.board_info.copper_layers or ()) if len(route_layers.layers()) > 2 else None)

    def menu(pad, grid, nid, own_only=False, climb=0, street=0, street_dirs=None):
        # (ends) the straight escape along the ball's own line too (escape_moves `straight`)
        return em.enumerate_moves(
            pad, grid, route_layers.escape_layers(pcb.board_info.copper_layers),
            lambda p, q, L, _n=nid: obs(_n, L, own_only).seg_clear(p, q),
            lambda p, L, _n=nid: not any((obs(_n, L_, own_only).point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0] for L_ in (via_layers or (L,))),
            climb=climb, straight=(PLAN_JUDGE == 'ends'),
            street=street, street_pitch=pe.sm._STACK_PITCH + 1e-4, street_dirs=street_dirs)
    dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
    dref = ends[names[0]][2]
    dgrid = em.grid_of(pcb.footprints[dref])
    dboxes = dgrid.bbox
    # the destination's FAR face, the one facing away from the source: under the whole route's ends a far-face move
    # is offered only to a ball in the array's half beside that face -- in ten fanouts over K15-K41 the ends model
    # chose 3 of 341 berths there, one net one column from the far face (K41 SA11), from 518 of 2269 menu moves
    # (every dog-bone and via-in-pad among them unchosen)
    _sc = [sum(ends[n][0][k] for n in names) / len(names) for k in (0, 1)]
    _dc = ((dboxes[0] + dboxes[2]) / 2, (dboxes[1] + dboxes[3]) / 2)
    far = max(em.DIRS, key=lambda d: em.DIRS[d][0] * (_dc[0] - _sc[0]) + em.DIRS[d][1] * (_dc[1] - _sc[1]))
    toward = max(em.DIRS, key=lambda d: em.DIRS[d][0] * (_sc[0] - _dc[0]) + em.DIRS[d][1] * (_sc[1] - _dc[1]))
    _fax = 0 if em.DIRS[far][0] else 1

    def far_half(pad):
        mid = (dboxes[_fax] + dboxes[_fax + 2]) / 2
        return ((pad.global_x, pad.global_y)[_fax] - mid) * em.DIRS[far][_fax] >= -1e-6
    for nm in names:
        nid, net = byname[nm]
        fp = pcb.footprints[ends[nm][2]]
        bx, by = ends[nm][1]
        pad = min(fp.pads, key=lambda p: (p.global_x - bx) * (p.global_x - bx)
                  + (p.global_y - by) * (p.global_y - by))
        dst_pad[nm] = pad
        moves = dedupe_climbs(menu(pad, em.grid_of(fp), nid, climb=DST_CLIMB,
                                   street=DST_STREET, street_dirs=(toward,)))
        if PLAN_JUDGE == 'ends' and not far_half(pad):
            moves = [m for m in moves if m.direction != far]
        dmenu[nm] = [m for m in moves if (nm, sr.move_sig(m)) not in banned
                     and (nm, 'class', sr.move_class(m)) not in banned]
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
            if int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0):
                for _b, (_pn, _nn) in _pairs.pair_names(names).items():
                    _plegs[_pn], _plegs[_nn] = _nn, _pn
            plan_state._pair_legs = _plegs
        if nm in _plegs and _plegs[nm] in byname:
            keep = [m for m in dmenu[nm] if pair_exit_clear(pcb, nid, byname[_plegs[nm]][0], m, free=run_free)]
            if len(keep) < len(dmenu[nm]):
                print(f'  {nm}: {len(dmenu[nm]) - len(keep)} of {len(dmenu[nm])} berth moves have no room '
                      f'for the pair at the exit -- dropped')
            if keep:
                dmenu[nm] = keep
        dmenu[nm] = _force(FORCE_DST, nm, dmenu[nm], 'destination')
        dmenu[nm] = via_escapes(dmenu[nm], 'dest')
        launch[nm] = ends[nm][0]
        others = [p for p in net.pads if p.component_ref != ends[nm][2]]
        if len(others) > 1:
            # the source pad is the one the net's source stub leaves (its copper walked from the launch point), not
            # the first other pad: a pair's termination part between the arrays carries a pad of the net too (the
            # zynq's R20 on CK), and first in the net's pad order it took the source's place -- no source menu, no
            # laid tooth, and the ends model had no option for the pair
            own = te._owner(ends[nm][0], [s for s in pcb.segments if s.net_id == nid], others)
            others.sort(key=lambda p: p.component_ref != own)
        src_pad[nm] = others[0] if others else None
    refs = {}
    for nm in names:
        if src_pad[nm] is not None:
            refs[src_pad[nm].component_ref] = refs.get(
                src_pad[nm].component_ref, 0) + 1
    sref = max(refs, key=refs.get)
    sgrid = em.grid_of(pcb.footprints[sref])
    smenu = {}
    # PLAN_JUDGE=ends: no tooth move to the source's FAR face (the one facing away from the destination) -- the whole
    # route has no way round the source (whole_frame), so such a move is no candidate
    far_dir = None
    if PLAN_JUDGE == 'ends':
        _dp = [(p_.global_x, p_.global_y) for p_ in pcb.footprints[ends[names[0]][2]].pads]
        _sp = [(p_.global_x, p_.global_y) for p_ in pcb.footprints[sref].pads]
        _fx = sum(x for x, _y in _dp) / len(_dp) - sum(x for x, _y in _sp) / len(_sp)
        _fy = sum(y for _x, y in _dp) / len(_dp) - sum(y for _x, y in _sp) / len(_sp)
        far_dir = min(DIRS, key=lambda d_: DIRS[d_][0] * _fx + DIRS[d_][1] * _fy)
        near_dir = max(DIRS, key=lambda d_: DIRS[d_][0] * _fx + DIRS[d_][1] * _fy)
    # ...and a tooth on a SIDE face only from a ball in the array's half beside that face: in eleven fanouts over K15-K41
    # every one of the 40 side-face teeth the ends model asked came from its face's half, and the north face's 387
    # menu moves (the bus in the source's south half) were never asked
    _sb = sgrid.bbox

    def side_half(pad_, d_):
        ax = 0 if DIRS[d_][0] else 1
        return ((pad_.global_x, pad_.global_y)[ax] - (_sb[ax] + _sb[ax + 2]) / 2) * DIRS[d_][ax] >= -1e-6
    for nm in names:
        p = src_pad[nm]
        if p is None or p.component_ref != sref:
            continue
        # the ends model prices its tooth moves against copper OUTSIDE the run, as the berths are: a move into a run
        # net's laid tooth TRACKS is a conflict with that tooth (sblock, below), not a refusal -- so teeth can swap and
        # rotate (a run net's laid via still refuses it). The other judges keep them priced against the run's laid
        # stubs (own_only)
        smenu[nm] = [m for m in dedupe_climbs(menu(p, sgrid, byname[nm][0], own_only=(PLAN_JUDGE != 'ends'),
                                                   climb=SRC_CLIMB))
                     if (nm, sr.move_sig(m)) not in banned and m.direction != far_dir
                     and (nm, 'class', sr.move_class(m)) not in banned
                     and (far_dir is None or m.direction == near_dir or side_half(p, m.direction))]
        _plegs = getattr(plan_state, '_pair_legs', None) or {}
        if nm in _plegs and _plegs[nm] in byname:
            keep = [m for m in smenu[nm] if pair_exit_clear(pcb, byname[nm][0], byname[_plegs[nm]][0], m, free=run_free)]
            if len(keep) < len(smenu[nm]):
                print(f'  {nm}: {len(smenu[nm]) - len(keep)} of {len(smenu[nm])} tooth moves have no room '
                      f'for the pair at the exit -- dropped')
            smenu[nm] = keep
        smenu[nm] = via_escapes(smenu[nm], 'src')
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
    # (ends) the run nets whose LAID copper each tooth move passes through (source_realize.blockers_of, over the
    # run's own copper): the move and that net's laid tooth are one conflict (whole_ends) -- the laid tooth is a move
    # with no legs (pages_first.current_tooth), so the lane test alone would plan a move through a tooth that stays
    sblock = {}
    if PLAN_JUDGE == 'ends':
        class _RunCopper:
            segments = [sg for sg in pcb.segments if sg.net_id in kids]
            vias = [v for v in pcb.vias if v.net_id in kids]
        for nm, ms in smenu.items():
            for m in ms:
                mv_, _pin = sr.blockers_of(_RunCopper, m, byname[nm][0], byname, set(names))
                if mv_:
                    sblock[(nm, id(m))] = mv_
    # (ends) an INCREMENTAL fanout (INCREMENTAL= the previous round's sidecar, FEEDBACK= its findings): only the ends
    # the feedback names move -- the teeth it names are free, every other tooth stands as laid on this board, and every
    # berth it does not name is held at the previous round's (the menu move at that berth's point and layer); what
    # worked stays, as replan.py's rounds kept the unmoved ends
    incr = None
    if PLAN_JUDGE == 'ends' and awx_settings.get('INCREMENTAL') and awx_settings.get('FEEDBACK'):
        import pairs as _pairs_i
        import whole_ends as _we
        prev = json.load(open(awx_settings.req('INCREMENTAL')))
        fb = json.load(open(awx_settings.req('FEEDBACK')))
        items = [e for pr in fb.get('pairs', ()) for e in pr] + list(fb.get('avoid', ()))
        # (a lane the whole solve could not keep to two vias -- 'over', whole_route.add_over -- has both its ends freed:
        # its tooth and its berth are where the promise it could not keep was made)
        over = set(fb.get('over') or {})
        free_t = {e['lane'] for e in items if e['end'] == 0} | over
        free_b = {e['lane'] for e in items if e['end'] == 1} | over
        pr_i = _pairs_i.pair_names(list(names))
        legs_b = {l_ for ln in free_b for l_ in (pr_i.get(ln) or (ln,)) if l_ in dmenu}
        pe_, pl_ = dict(prev['ends']), dict(prev['dest_layer'])
        fixed_b = {}
        for nm in names:
            if nm in legs_b or nm not in pe_ or nm not in dmenu:
                continue
            bx_, by_ = pe_[nm][1]
            # (held as LAID: the move the sidecar records it was laid by, exactly -- the nearest menu move on its layer
            # can be another kind or direction to the same exit, whose lanes or site conflict with a neighbour the laid
            # berth did not; that is the fallback for a sidecar that records none)
            sig_ = (prev.get('berth_sig') or {}).get(nm)
            m_ = next((m for m in dmenu[nm] if json.dumps(sr.move_sig(m)) == json.dumps(sig_)), None) if sig_ else None
            if m_ is not None and math.hypot(m_.exit_pt[0] - bx_, m_.exit_pt[1] - by_) <= _we.DUP_TOL:
                fixed_b[nm] = sr.move_sig(m_)
                continue
            m_ = min((m for m in dmenu[nm] if m.layer == pl_.get(nm)), default=None,
                     key=lambda m: math.hypot(m.exit_pt[0] - bx_, m.exit_pt[1] - by_))
            if m_ is not None and math.hypot(m_.exit_pt[0] - bx_, m_.exit_pt[1] - by_) <= _we.DUP_TOL:
                fixed_b[nm] = sr.move_sig(m_)
        incr = {'free_teeth': free_t, 'fixed': fixed_b}
    return _hold_berths({
            'banned': banned,          # the feasibility ledger, for a proposal
                                       # that enumerates its own moves
            'byname': byname, 'dmenu': dmenu, 'smenu': smenu, 'sblock': sblock, 'launch': launch,
            # the whole route's feedback (whole_feedback.py): ends its audits found crowded, priced by whole_ends
            'feedback': json.load(open(awx_settings.req('FEEDBACK'))) if awx_settings.get('FEEDBACK') else None,
            'incr': incr,
            'tooth0': tooth0, 'tooth_vias': tooth_vias, 'src_pad': src_pad,
            'dst_pad': dst_pad, 'sref': sref, 'dref': dref, 'sgrid': sgrid,
            'bundle_layer': bundle_layer, 'chi': chi,
            'dgrid': dgrid, 'dboxes': dboxes,
            'buses': buses, 'obs': obs, 'pcb': pcb,
            'pads_of': {ref: [(p.global_x, p.global_y) for p in fp.pads]
                        for ref, fp in pcb.footprints.items()}})


# THE DESTINATION'S BERTHS HELD (the joint fanout): {net: Move} -- the berths a joint plan of the destination array
# places nearest the ends model's first choice, every ball of the array served and every pair whole (joint_berths),
# held through the fanout's plan: each plan state carries them in its menu, held (`held`), so the ends model chooses
# the teeth against berths the destination will lay, and the joint destination, preferring them, lays them. The ends
# model's own berth menu (no climbs) has no conflict-free choice on the zynq's U5 for the AD9364's LVDS bus (33 lanes,
# CP-SAT infeasible); chosen there, 17 of its berths conflicted, the joint destination moved 23 of 47, RX_D1's from the
# up face to the left, 3.7 mm from U1, and the solve found no room for the crossover its plan needed
_HELD = {}


def _hold_berths(st):
    """`st` with the held berths in its destination menu and named in st['held'] {net: move signature}"""
    st['held'] = {}
    for nm, m in _HELD.items():
        if nm not in st['dmenu']:
            continue
        sig = sr.move_sig(m)
        if not any(sr.move_sig(x) == sig for x in st['dmenu'][nm]):
            st['dmenu'][nm] = list(st['dmenu'][nm]) + [m]
        st['held'][nm] = sig
    return st


def pair_travel(da, db):
    """the direction a pair whose legs leave by faces `da` and `db` (escape names) travels: their face's, or -- a pair
    round a corner, one leg out of each face -- the diagonal between them; None for two opposite faces (no hand)"""
    import pairs as _pairs
    if da == db:
        return da
    va, vb = _pairs._DIRS.get(da), _pairs._DIRS.get(db)
    if va is None or vb is None or (va[0] + vb[0], va[1] + vb[1]) == (0, 0):
        return None
    return (va[0] + vb[0], va[1] + vb[1])


def teeth_hands(pcb, names, byname, sref):
    """{a pair's P leg (short name): (hand, True)}: the HAND each pair's teeth leave the array `sref` with on `pcb`
    (pairs.hand, the teeth as they stand -- the destination not yet fanned, each net's one free end is its tooth),
    the hand its berths are to ARRIVE with: a coupled route keeps P on one side of its travel from end to end"""
    import joint_escape as je
    import pairs as _pairs
    out = {}
    for _b, (pn, nn) in _pairs.pair_names(list(names)).items():
        g = {}
        for nm in (pn, nn):
            pad = (next((p for p in pcb.footprints[sref].pads if p.net_id == byname[nm][0]), None)
                   if nm in byname else None)
            with contextlib.redirect_stdout(io.StringIO()):
                g[nm] = sr.measure_tooth(pcb, nm, pad, byname) if pad is not None else None
        if not g[pn] or not g[nn]:
            continue                # (a leg with no tooth -- bare: the other end's hand is the pair's)
        h = _pairs.hand(pair_travel(g[pn]['direction'], g[nn]['direction']), g[pn]['tooth'], g[nn]['tooth'])
        if h:
            out[je.short_name(pn)] = (h, True)
    return out


def berth_hands(choice):
    """{a pair's P leg (short name): (hand, False)}: the hand each pair's berths in `choice` {net: Move} arrive with
    (pairs.hand), the hand its teeth are to LEAVE with"""
    import joint_escape as je
    import pairs as _pairs
    out = {}
    for _b, (pn, nn) in _pairs.pair_names(list(choice)).items():
        a, b = choice.get(pn), choice.get(nn)
        if a is None or b is None:
            continue
        h = _pairs.hand(pair_travel(a.direction, b.direction), a.exit_pt, b.exit_pt, arriving=True)
        if h:
            out[je.short_name(pn)] = (h, False)
    return out


def joint_berths(st, board, choice, log=print):
    """{net: Move}: the destination array planned jointly (joint_escape.plan_array: its bus, other nets and plane
    balls, as the joint destination plans them) on `board`, the bus preferring `choice` (the ends model's berths), each
    pair arriving with the hand its teeth leave by (teeth_hands) -- the berth each bus net gets"""
    import joint_escape as je
    spec = json.load(open(awx_settings.req('FANOUT_JOINT')))
    dref = st['dref']
    a = next((x for x in spec['arrays'] if x['ref'] == dref), {'others': [], 'drops': []})
    prefer = {je.short_name(nm): {'tooth': tuple(m.exit_pt), 'direction': m.direction, 'layer': m.layer,
                                  'kind': m.kind} for nm, m in choice.items()}
    names = [st['byname'][nm][1].name for nm in choice if nm in st['byname']]
    pcb = parse_kicad_pcb(board)
    hands = teeth_hands(pcb, list(choice), st['byname'], st['sref'])
    with contextlib.redirect_stdout(io.StringIO()):
        _h, rep = je.plan_array(pcb, dref, names, a['others'], spec['layers'], prefer=prefer,
                                drops=a['drops'], debug=True, log=lambda *a_: None, hands=hands,
                                vias_only=route_layers.escape_vias('dest'), exit_rays=True)
    out = {k.split('#')[0]: o for k, (kind, o) in (rep.get('debug', {}).get('chosen') or {}).items()
           if kind == 'escape' and k.split('#')[0] in choice}
    same = sum(1 for nm, o in out.items() if sr.move_sig(o) == sr.move_sig(choice[nm]))
    log(f'  berths held (joint plan of {dref}, preferring the ends model\'s): {len(out)}/{len(choice)} placed, '
        f'{same} the ends model\'s own; pairs {rep.get("pairs_escaped")}/{rep.get("pairs")}, held to their teeth\'s '
        f'hand {len(rep.get("hands_held") or ())}/{len(rep.get("hands_held") or ()) + len(rep.get("hands_free") or ())}'
        + (f' (NO berth pair of it: {rep["hands_free"]})' if rep.get('hands_free') else '')
        + f', phase 1 {rep.get("phase1_tiers")}')
    return out


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
        plan.setdefault('berth_sig', {})[nm] = sr.move_sig(m)     # the move it was laid by (an incremental round's hold)
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


PAIR_BAD_W = float(awx_settings.get('PLAN_PAIR_BAD_W', '50') or 0)


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
    if not int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0) or PAIR_BAD_W <= 0:
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
    if PLAN_JUDGE == 'ends':
        import whole_ends
        v, parts = whole_ends.judge(st, choice)
        judge_by_braid.ends = parts
        return v, {}, {}, plan
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


def _pair_legs_of(base, names):
    """The two nets of pair `base` among `names`."""
    import pairs as _pairs
    return _pairs.pair_names(list(names)).get(base, ())


def drc_side_net(side):
    """The short net name of one side of a check_drc violation line: the kind prefix (`Seg:`, `Pad:`) and the
    TRAILING annotations stripped (`(REF.PAD)`, `[SHORT]`, `(drill hole clearance)`) -- never split on whitespace:
    this board's nets are `/DDR3 16x1/SDQ2`, so a space split yields `DDR3`."""
    import re as _re
    side = side.strip()
    if ':' in side[:6]:
        side = side.split(':', 1)[1]
    for _ in range(3):
        side = _re.sub(r'\s*\[[^\]]*\]\s*$', '', side)
        side = _re.sub(r'\s*\([^)]*\)\s*$', '', side)
    return side.strip().split('/')[-1]


def drc_line_nets(line):
    """(net, net) of a check_drc violation line `A <-> B`, or None"""
    import re as _re
    mt = _re.match(r'^\s*(.+?) <-> (.+?)\s*$', line)
    if not mt:
        return None
    a, b = drc_side_net(mt.group(1)), drc_side_net(mt.group(2))
    return (a, b) if a and b else None


def ban_moves(banned, moves, nets, names):
    """Ban `nets`' moves in `moves` {net: Move} (the fanout would not lay them as asked). A PAIR's two legs are ONE
    joint move: a leg whose partner's move is in `moves` too bans the pair's move as a unit -- ('pair', P, N, P's
    signature, N's signature), which whole_ends drops from the pair's options -- never a leg's move alone (with
    another partner move it is another joint move); a leg moved alone is banned alone. Only under PLAN_JUDGE=ends,
    whose ends model reads a joint ban: every other judge filters its menus leg by leg (plan_state), and bans each.
    Under PLAN_JUDGE=ends a ban also covers the move's CLASS (source_realize.move_class: every leg variant of the same
    exit), which the engine refuses for the same reason -- banned one variant at a time, the re-plans asked them in turn"""
    import pairs as _pairs
    legs = {}
    if PLAN_JUDGE == 'ends' and int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0):
        for pn, nn in _pairs.pair_names(list(names)).values():
            legs[pn] = legs[nn] = (pn, nn)
    by_class = PLAN_JUDGE == 'ends'
    for nm in nets:
        pr = legs.get(nm)
        if pr and pr[0] in moves and pr[1] in moves:
            banned.add(('pair', pr[0], pr[1], sr.move_sig(moves[pr[0]]), sr.move_sig(moves[pr[1]])))
            if by_class:
                banned.add(('pairclass', pr[0], pr[1], sr.move_class(moves[pr[0]]), sr.move_class(moves[pr[1]])))
        else:
            banned.add((nm, sr.move_sig(moves[nm])))
            if by_class:
                banned.add((nm, 'class', sr.move_class(moves[nm])))


def split_pairs(st):
    """(count, [pair]) of the differential pairs whose TEETH, as laid, stand on
    different faces or layers of the source array -- a pair that cannot be
    launched coupled. Zero without PLAN_PAIRS / BRAID_PAIRS."""
    if not int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0):
        return 0, []
    import pairs as _pairs
    import pages_first
    out = []
    for base, (pn, nn) in _pairs.pair_names(list(st['launch'])).items():
        a, b = pages_first.current_tooth(st, pn), pages_first.current_tooth(st, nn)
        if a is not None and b is not None and (a.direction, a.layer) != (b.direction, b.layer):
            out.append(base)
    return len(out), sorted(out)

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


SRC_RESIDUE_ROUNDS = int(awx_settings.get('SRC_RESIDUE_ROUNDS', '8'))   # one tooth realized -> re-chosen, at most this often a round


def total(dst_c, st, cache, buses=None):
    """What the plan is judged on (plan_ends.judged_cost): the vias the
    plan's own model implies, both escapes' vias included, plus the ride
    round both arrays at VIA_MM per via -- keepers judged within the
    corridors the braid will form (planned_buses)."""
    return pe.judged_cost(dst_c, st['launch'], st['dboxes'], cache,
                          st['sgrid'].bbox, st['tooth0'], st['tooth_vias'],
                          buses if buses is not None else planned_buses(st, dst_c),
                          chi=st['chi'])


def dest_choice(st, board, log=print, fixed=None, learned=None, src_out=None, seed=None):
    """The destination choice on a plan state: the greedy selector, then
    the pages-first planner over it (the greedy's move stays the fallback
    for a net the planner leaves out). ONE function for the first plan and
    for every re-plan after a refused berth. Returns (choice, unplaced)."""
    pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y)
            for nm in st['dst_pad']}
    choice, un = pe.sm.select(st['dmenu'], st['launch'],
                              keep_out=st['dboxes'], buses=st['buses'],
                              tooth_layer=st['tooth0'], log=None, pads=pads, chi=st['chi'])
    if choice and PLAN_JUDGE == 'ends':
        # the whole route's ends model: every berth (and, with `src_out`, the teeth to move) from `seed` (a previous
        # choice) or the greedy's
        import whole_ends
        inc = st.get('incr') or {}
        ch, src = whole_ends.choose(st, log=log or (lambda *a: None), src_free=(src_out is not None),
                                    fixed=(fixed if fixed is not None else (st.get('held') or inc.get('fixed'))),
                                    seed=seed or choice,
                                    learned=learned, free_teeth=inc.get('free_teeth'))
        for nm, mv in choice.items():
            ch.setdefault(nm, mv)
        un = [nm for nm in un if nm not in ch]
        choice = ch
        if src_out is not None:
            src_out.update(src)
    elif choice and PLAN_PAGES:
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
SRC_REFAN_JOINT = int(awx_settings.get('SRC_REFAN_JOINT', '0'))
# cap on how many blockers may be re-fanned with one tooth: the region the
# engine is asked to re-solve, not the whole array
SRC_REFAN_MAX = int(awx_settings.get('SRC_REFAN_MAX', '6'))


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
    _HELD.clear()           # (a fanout run in the chain's own process: the last round's berths are not this one's)
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
        if PLAN_JUDGE == 'ends' and awx_settings.get('FANOUT_JOINT') and not _HELD:
            # the joint fanout: the berths the destination's joint plan places nearest these, held from here on --
            # and the ends chosen again on them (_HELD)
            _HELD.update(joint_berths(st, board, dst_choice))
            if _HELD:
                _hold_berths(st)
                src_out = {}
                dst_choice, un = dest_choice(st, board, src_out=src_out)
                if not dst_choice:
                    print(f'  round {r}: no destination choice on the held berths'); break
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
            if PLAN_JUDGE == 'ends':
                # the ends model judges a board on its teeth AS LAID (the berths searched against them)
                import whole_ends
                best_key = pf_key(dst_choice, {}, whole_ends.choose.last['laid'])
            else:
                best_key = _key(dst_choice)
            best_split = split_pairs(st)

            def _trial(moves, new_board):
                """Lay `moves` on `board` with the engine, choose the destination
                again on the new board, judge it. (result, line) -- result None
                when it was not laid as asked or has no destination choice."""
                _free = []
                if SRC_REFAN_JOINT:
                    _free = _blockers_for(parse_kicad_pcb(board), moves, st,
                                          set(names) - set(moves), log=print)
                res_r = sr.realize(board, moves, st['src_pad'], st['byname'],
                                   st['sref'], new_board, guard_names=names,
                                   free=_free, hands=berth_hands(dst_choice))
                realized.append(res_r)
                misses = [nm for nm, e in res_r['audit'].items() if not e['exact']]
                # the JOINT realize (FANOUT_JOINT) plans the asked teeth with every other ball of the array and lays its
                # plan exactly: a tooth off its ask is that plan's choice, not an engine's refusal -- the board is
                # judged as any other (kept when better), and nothing banned. Treated as a refusal, the zynq LVDS bus's
                # source on three layers was asked 22 moves, laid with every face and every rank along each face kept
                # and 8 of them a gap over, the board thrown away and the 8 banned -- six passes, nothing kept, until
                # a net had no option left
                joint_ = bool(res_r.get('joint'))
                if not joint_:
                    ban_moves(banned, moves, misses, names)
                how_ = "the joint plan's" if joint_ else 'banned'
                line = (f'  round {r}: source residue move(s) realized: {sorted(moves)}'
                        + (f'; not laid as asked ({how_}): {misses}' if misses else '')
                        + (f'; REJECTED ({res_r["rejected"]})' if res_r['rejected'] else ''))
                _trial.missed = misses if PLAN_JUDGE == 'ends' and not joint_ else []
                if _trial.missed and res_r['rejected']:
                    # (ends) the trial is returned unjudged below, so the moves its DRC names are banned here too: a
                    # rejected trial's moves were never banned, and the next round asked them again
                    hit = sorted({n for ln in res_r.get('pairs') or () for n in (drc_line_nets(ln) or ())} & set(moves))
                    if hit:
                        ban_moves(banned, moves, hit, names)
                if _trial.missed:
                    # (ends) a board whose teeth are not the plan's is NOT kept: what the engine laid in place of an
                    # asked move is a tooth nobody chose (K35: SA12's dogbone to B laid as a surface tooth on F,
                    # squeezed 0.20 from SDQ0's, and the board kept for judging better than the one before)
                    return None, line
                if res_r['rejected']:
                    # the moves of the nets its DRC pairs name (all of them when no line names one)
                    hit = sorted({n for ln in res_r.get('pairs') or () for n in (drc_line_nets(ln) or ())} & set(moves))
                    ban_moves(banned, moves, hit or list(moves), names)
                    return None, line
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
                            if nm not in moves and nm in st2['dmenu']
                            and any(sr.move_sig(mm) == sr.move_sig(m) for mm in st2['dmenu'][nm])}
                if PLAN_JUDGE == 'ends':
                    # the ends model searches every berth again, from the previous choice
                    ch2, un2 = dest_choice(st2, new_board, src_out=src2, seed=dst_choice)
                    if not ch2:
                        return None, line + '; no destination choice on the new board -- reverted'
                    import whole_ends
                    return (new_board, st2, ch2, un2, pf_key(ch2, {}, whole_ends.choose.last['laid']), src2,
                            split_pairs(st2)), line
                ch2, un2 = dest_choice(st2, new_board, src_out=src2, fixed=keep_sig)
                if not ch2:
                    return None, line + '; no destination choice on the new board -- reverted'
                f2, _p2, bp2, _pl2 = judge_by_braid(st2, ch2, new_board)
                mv2 = None
                if PLAN_PAGES and not PLAN_JUDGE:
                    import pages_first
                    mv2 = getattr(pages_first.choose, 'last', {}).get('vias', f2)
                return (new_board, st2, ch2, un2, pf_key(ch2, bp2, f2, mv2), src2, split_pairs(st2)), line
            for _k in range(SRC_RESIDUE_ROUNDS):
                # the moves IN THE PLAN'S ORDER: it is the order
                # `_blockers_for` walks (and its `free` list is capped) and
                # the order the engine receives its hints in, so a rebuilt
                # dict gives the engine a different call and different
                # copper (measured at K35: 69 segments different, judged 198
                # against 181)
                rest = dict(src_out)
                res, line = _trial(rest, f'{work}_srcres{r}_{_k}.kicad_pcb')
                if res is None and getattr(_trial, 'missed', None):
                    # the engine did not lay every move as asked: the board stays as it was, the misses banned, and
                    # the plan is chosen again on it without them
                    print(line + '; the board NOT kept -- planned again without them')
                    st = plan_state(parse_kicad_pcb(board), names, banned)
                    src_out = {}
                    dst_choice, un = dest_choice(st, board, src_out=src_out, seed=dst_choice)
                    import whole_ends
                    best_key = pf_key(dst_choice, {}, whole_ends.choose.last['laid'])
                    best_split = split_pairs(st)
                    if not src_out:
                        break
                    continue
                if res is None:
                    print(line)
                    break
                sp2 = res[6]
                # A PAIR'S TEETH ARE ONE UNIT: a board with fewer pairs split
                # across faces or layers wins before any count, and one that
                # splits a pair never does. The moves are judged as a SET, so
                # the move uniting a pair was reverted and banned with the
                # others (zynq K44, branch judge: DQS0_N's tooth left on the
                # BGA's south face, DQS0_P's on the east -- the pair refused
                # at its source in every braid); a set that unites a pair but
                # judges worse is tried again with the pair legs' moves alone.
                if sp2[0] < best_split[0] and not pf_better(res[4], best_key):
                    legs = {nm for b_ in best_split[1] for nm in _pair_legs_of(b_, names)}
                    sub = {nm: m for nm, m in rest.items() if nm in legs}
                    if sub and len(sub) < len(rest):
                        res_s, line_s = _trial(sub, f'{work}_srcres{r}_{_k}p.kicad_pcb')
                        if res_s is not None and res_s[6][0] < best_split[0] \
                                and (res_s[6][0] < sp2[0] or pf_better(res_s[4], res[4])):
                            print(line + f'; {pf_fmt(best_key, res[4])}, pairs split {best_split[0]} -> {sp2[0]}'
                                  f' -- the pair legs\' moves alone instead')
                            res, line, sp2 = res_s, line_s, res_s[6]
                if sp2[0] < best_split[0] or (sp2[0] == best_split[0] and pf_better(res[4], best_key)):
                    print(line + f'; {pf_fmt(best_key, res[4])}'
                          + (f', pairs split {best_split[0]} -> {sp2[0]}' if sp2[0] != best_split[0] else '')
                          + ': KEPT')
                    board, st, dst_choice, un, best_key, src_out = res[0], res[1], res[2], res[3], res[4], res[5]
                    best_split = sp2
                else:
                    print(line + f'; {pf_fmt(best_key, res[4])}'
                          + (f', pairs split {best_split[0]} -> {sp2[0]}' if sp2[0] != best_split[0] else '')
                          + ': not better -- reverted')
                    # (nothing banned: the engine laid them as asked -- a ban is the engine's refusal, not the judge's)
                    break
                if not src_out:
                    break
            if PLAN_JUDGE == 'ends':
                # the berths for the board KEPT, against its teeth as laid (a choice made with tooth moves that
                # were then not kept is not this board's) -- unless the last choice was made on this very board and
                # asked no tooth move: that choice is the one
                _L = getattr(whole_ends.choose, 'last', {})
                if not (_L.get('st') == id(st) and not _L.get('moved')):
                    dst_choice, un = dest_choice(st, board, seed=dst_choice)
        pb = planned_buses(st, dst_choice)
        # (the fast proxy is the braid's two-page planner's: on more routing layers it has none to give)
        f_fast = total(dst_choice, st, cache, pb) if len(route_layers.layers()) == 2 else float('nan')
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
                         st['sref'], new_board, guard_names=names, hands=berth_hands(dst_choice))
        realized.append(res)
        # FEEDBACK: every asked move the engine did not lay exactly leaves
        # that net's menu; the next round plans over what is achievable
        misses = [nm for nm, e in res['audit'].items() if not e['exact']]
        ban_moves(banned, src_choice, misses, names)
        new_bans = len(misses)
        if misses:
            print(f'  round {r}: {len(misses)} asked source move(s) not laid as '
                  f'asked -> banned for re-planning: {misses}')
        if res['rejected']:
            print(f'  round {r}: realized board REJECTED ({res["rejected"]}); '
                  f'keeping {os.path.basename(board)}')
            ban_moves(banned, src_choice, list(src_choice), names)
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
    if PLAN_JUDGE == 'ends':
        import whole_ends
        print(f'  plan model (the whole route\'s ends): {whole_ends._fmt(judge_by_braid.ends)}')
    else:
        print(f'  plan model total predicted vias: {sum(pred.values())} over {len(pred)} nets '
              + (f'(PLAN_JUDGE={PLAN_JUDGE}: the braid\'s count {cost:.0f})' if PLAN_JUDGE else
                 f'(braid-judged cost {cost:.2f} incl. ride)'))
    if out_path:
        side = os.path.splitext(out_path)[0] + '.plan.json'
        if PLAN_PAGES:
            plan['pages_first'] = True      # the braid stage pages this plan EXACTLY
        plan['nets'] = list(names)          # the run, in its order: an INCREMENTAL round routes the same nets
        if PLAN_JUDGE == 'ends':
            # the ends model's reading of each lane on these ends (whole_ends: its nets over two, its load on the
            # trunk, its crossings) -- the lanes a round the whole route lays nothing on frees next (whole_feedback
            # --name) -- and the far face's cut it read them with: the solve's frame takes that one (braid.setup ->
            # whole_frame), where recomputed from the laid stubs, a hair off the menu's exits, two near-equal gaps could
            # split the face the other way and hand the solve crossings the model never priced
            pe_ = judge_by_braid.ends
            plan['ends_model'] = {k: pe_.get('lane_' + k, {}) for k in ('over', 'load', 'x', 'front')}
            if pe_.get('cut') is not None:
                c_ = pe_['cut']
                # (a far-face cut its y, as it always was; one off the far face -- winding -- its face and coordinate)
                plan['dest_cut'] = [c_[0], float(c_[1])] if isinstance(c_, (list, tuple)) else float(c_)
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
    if '--hold' in sys.argv[2:]:
        # (OUT_TAG is the round's fanned board, its passives just moved by the cap step: whole_route)
        joint_hold(out_path)
        return 0
    # the run's nets: coherent on the base -- or, an INCREMENTAL round, the previous round's (its base is that round's
    # source board, on which the coherent K can be other nets: K35's lost its three pairs to six others)
    prev_nets = (json.load(open(awx_settings.req('INCREMENTAL'))).get('nets')
                 if PLAN_JUDGE == 'ends' and awx_settings.get('INCREMENTAL') else None)
    if PLAN_JUDGE == 'ends' and awx_settings.get('INCREMENTAL') and not prev_nets:
        print(f'WARNING: INCREMENTAL={awx_settings.req("INCREMENTAL")} records no nets: the run is the coherent {K} on '
              f'{os.path.basename(base)}, which can be other nets than the previous round\'s')
    if PLAN_JUDGE == 'ends' and bool(awx_settings.get('INCREMENTAL')) != bool(awx_settings.get('FEEDBACK')):
        print('WARNING: an incremental round needs both INCREMENTAL and FEEDBACK: this round chooses every end afresh')
    # (FANOUT_NETS: the run's nets as the chain fixed them, whole_route -- never re-picked off a later round's board)
    run_nets = [n for n in (awx_settings.get('FANOUT_NETS') or '').split(',') if n]
    names = prev_nets or run_nets or coherent_nets(K, base)
    print('planning (source realized every round)...')
    work = out_path[:-len('.kicad_pcb')] if out_path.endswith('.kicad_pcb') else out_path
    choice, dst_pad, dref, byname, board, realized, banned = plan(base, names, work)
    # the joint fanout's destination from the array's JOINT plan with more routing layers than two (the zynq U5's
    # LVDS berths: the ends model's own held 7 conflicts and 3 split pairs); on two the bus's berths are laid as the
    # chain lays them and the array's other balls planned round them after (joint_others) -- the joint plan of the
    # whole array chose berths for every ball's sake, and the zynq DDR's ends crossed 260 times to the chain's 202
    joint_dest = bool(awx_settings.get('FANOUT_JOINT')) and len(route_layers.layers()) > 2
    if joint_dest:
        rc = joint_destination(out_path, names, choice, dref, byname, board, banned)
    else:
        rc = fanout_destination(out_path, names, choice, dst_pad, dref, byname, board, realized, banned)
    # THE FANOUT AUDIT on the board handed back: a fanout NEVER hands back one the whole route would refuse -- a pair
    # split the destination's re-plans could not clear (a tooth's, or a berth's with nothing left to ban), or anything
    # else its frame stops on. Such a board is set aside, named, and no board goes to the stage after
    sp, far, stops = fanout_audit(out_path, names, dref) if os.path.isfile(out_path) else ([], [], [])
    if sp or far or stops:
        held = out_path[:-len('.kicad_pcb')] + '.refused.kicad_pcb'
        os.replace(out_path, held)
        # (the findings, for the chain's feedback: the next round frees the ends named -- whole_feedback --refused)
        with open(out_path[:-len('.kicad_pcb')] + '.refused.json', 'w', encoding='utf-8') as f_:
            json.dump({'splits': [[pr_, end_, list(ls_)] for pr_, end_, ls_ in sp], 'far': list(far),
                       'stops': stops}, f_, indent=1)
        print(f'REFUSED by the fanout audit: {_audit_line(sp, far, stops)} -- the board set aside as '
              f'{os.path.basename(held)}')
        return 3
    print('fanout audit: clean')
    joint_others(out_path, skip={dref} if joint_dest else ())
    return rc


def joint_destination(out_path, names, choice, dref, byname, board, banned, log=print):
    """FANOUT_JOINT (route_bus --joint-fanout): the destination's WHOLE array laid from one joint plan
    (joint_escape.fan_array) -- every ball of it: the bus's berths, each PREFERRING the ends model's choice for it
    (`choice`: its exit, face, layer and kind, joint_escape's deviation costs) and moved only where those cannot all
    be laid together (two in conflict, a pair split: the plan holds a pair's two legs on one face and layer, their exits
    neighbours), the other nets' escapes and straps, and the plane balls' drops -- laid exactly as planned. The ends
    model chooses berths ball by ball and repairs pairs after; on the zynq's U5 for the AD9364's LVDS bus its first
    choice already held 7 conflicting berths and 3 split pairs (the TX pairs, staircased on diagonal balls), and eight
    engine passes left 8 to 20 of them not as asked, round after round refused; the joint plan places all 47 berths,
    every pair whole. The plan sidecar is written from the berths laid. Returns 0, or 1 where a bus ball is left bare."""
    import joint_escape as je
    spec = json.load(open(awx_settings.req('FANOUT_JOINT')))
    a = next((x for x in spec['arrays'] if x['ref'] == dref), {'others': [], 'drops': []})
    prefer = {je.short_name(nm): {'tooth': tuple(m.exit_pt), 'direction': m.direction, 'layer': m.layer,
                                  'kind': m.kind} for nm, m in choice.items()}
    # each pair's berths arriving with the hand its teeth leave by (teeth_hands, on the source as laid)
    sref = next((x['ref'] for x in spec['arrays'] if x['ref'] != dref), None)
    # the source board, named as fanout_destination names it: the whole route's next round starts from it
    # (whole_route.advance). Unnamed, every round after the first started again from the bench's own source fanout,
    # its pairs split as that pair-blind fanout left them (zynq LVDS on four layers: 11 pairs split at U1, every round 2)
    log(f'\nplan (joint destination {dref}): {len(choice)} berth(s)  (source board: {os.path.basename(board)}, '
        f'{len(banned)} banned move(s))')
    pcb = parse_kicad_pcb(board)
    hands = teeth_hands(pcb, list(names), byname, sref) if sref in pcb.footprints else {}
    rung, reps = je.fan_array(board, out_path, dref, list(names), a['others'], spec['layers'], prefer=prefer,
                              drops=a['drops'], log=log, debug=True, hands=hands,
                              vias_only=route_layers.escape_vias('dest'), exit_rays=True)
    kept = next(r for r in reps if (r['track'], r['via'], r['drill']) == (rung.fan_track, rung.via_size,
                                                                          rung.via_drill))
    bus_s = {je.short_name(nm) for nm in names}
    laid = {k.split('#')[0]: o for k, (kind, o) in (kept.get('chosen') or {}).items()
            if kind == 'escape' and k.split('#')[0] in bus_s}
    moved = sorted(nm for nm, o in laid.items() if nm in prefer and (
        o.direction != prefer[nm]['direction'] or o.layer != prefer[nm]['layer']
        or math.hypot(o.exit_pt[0] - prefer[nm]['tooth'][0], o.exit_pt[1] - prefer[nm]['tooth'][1]) > sr.GAP_TOL))
    bare_bus = sorted(nm for nm in bus_s if nm not in laid)
    log(f'  joint destination {dref}: {len(laid)}/{len(bus_s)} berths planned and laid ({len(laid) - len(moved)} as the '
        f'ends model chose them, {len(moved)} moved: {moved}), pairs {kept.get("pairs_escaped")}/{kept.get("pairs")}, '
        f'others {kept["planned_others"]}/{len(a["others"])} planned, plane drops {kept["planned_drops"]}; '
        f'{len(kept["bare"])} ball(s) bare, {kept["undropped"]} plane ball(s) undropped; phase 1 {kept.get("tiers")}'
        + (f'; pairs held to their teeth\'s hand {len(kept.get("hands_held") or ())}/'
           f'{len(kept.get("hands_held") or ()) + len(kept.get("hands_free") or ())}'
           + (f' (NO berth pair of it: {kept["hands_free"]})' if kept.get('hands_free') else '') if hands else '')
        + (f'; BUS BALLS BARE {bare_bus}' if bare_bus else ''))
    # the DESTINATION AUDIT, as the source's (source_realize.audit): each berth the ends model asked against the one
    # laid -- a moved face or a pair's hand turned is a route the ends model did not plan (zynq U5: RX_D1's asked on the
    # up face, laid on the left 3.7 mm from U1, its planned crossover left no room)
    sh = lambda o: f'{o.kind}/{o.direction}/{o.layer[0]} exit=({o.exit_pt[0]:.2f},{o.exit_pt[1]:.2f})'   # noqa: E731
    for nm in sorted(moved):
        o, g = laid[nm], prefer[nm]
        bad = [w for w, ok in (('FACE', o.direction == g['direction']), ('LAYER', o.layer == g['layer']),
                               ('KIND', o.kind == g['kind'])) if not ok]
        gap = math.hypot(o.exit_pt[0] - g['tooth'][0], o.exit_pt[1] - g['tooth'][1])
        log(f'    {nm:14s} asked {g["kind"]}/{g["direction"]}/{g["layer"][0]} exit=({g["tooth"][0]:.2f},'
            f'{g["tooth"][1]:.2f})  laid {sh(o)}  ' + (', '.join(bad) + ', ' if bad else '') + f'{gap:.2f} mm off')
    # the sidecar from the berths LAID: the whole route reads a planned net's ends from it -- each berth's end as the
    # copper stands (braid_plan_of's `achieved`, as fanout_destination measures it), never the planned exit: the engine's
    # stub ends past it (zynq's U5, 25 um), and the relayer, walking from the planned point along a stub it lies in the
    # middle of, found no run to a via at any of the 25 berths it was to move onto the solve's layers
    st = plan_state(parse_kicad_pcb(board), names, banned)
    ch_laid = {nm: laid[je.short_name(nm)] for nm in choice if je.short_name(nm) in laid}
    with contextlib.redirect_stdout(io.StringIO()):
        pcb_out = parse_kicad_pcb(out_path)
        achieved = {nm: sr.measure_tooth(pcb_out, nm, st['dst_pad'][nm], st['byname'], dest_ref=dref)
                    for nm in ch_laid if st['dst_pad'].get(nm) is not None}
    achieved = {nm: g for nm, g in achieved.items() if g}
    try:
        explain_plan(ch_laid, st, names, out_path, out_path, achieved=achieved)
    except Exception as e:
        log(f'  joint destination: the plan sidecar NOT written ({type(e).__name__}: {e})')
    # the TIE VIAS plan_state promised (a ball with a pad of its own net under it gets no via-in-pad berth: a via at
    # the ball serves the pad, tie_vias_under), on the shipped board as fanout_destination adds them -- the joint path
    # left those pads open
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb_t = parse_kicad_pcb(out_path)
        ties = tie_vias_under(pcb_t, list(names), byname, st['dst_pad'], [], log=log)
        if ties:
            tmp = out_path[:-len('.kicad_pcb')] + '_tie.kicad_pcb'
            add_tracks_and_vias_to_pcb(out_path, tmp, [], ties, [],
                                       net_id_to_name={i: n.name for i, n in pcb_t.nets.items()})
            os.replace(tmp, out_path)
            ship_vias.stamp(out_path, 'fanout ties', log)
            log(f'  joint destination: {len(ties)} tie via(s) for pads under balls')
    except Exception as e:      # a tie must not lose the board
        log(f'  tie vias NOT added: {e}')
    return 1 if bare_bus else 0


def joint_others(out_path, log=print, skip=()):
    """FANOUT_JOINT (route_bus --joint-fanout): a JSON file naming the arrays whose OTHER nets and plane balls are
    fanned with the bus's -- {"layers": [the others' layers], "arrays": [{"ref", "others", "drops"}, ...]}. The bus
    is laid at both arrays first, by the bus-only engine call above, exactly as without it; then the others.

    The first round plans and lays them jointly round that copper (joint_escape.fan_array: one plan, the whole
    fanout stepped down the fab ladder together only while a ball is left), and records the size it kept beside the
    board (<OUT>.joint.json). A later round -- INCREMENTAL, whose fanout moves only the bus stubs its feedback names
    and holds the rest -- holds these the same way: each ball's piece of the previous round's copper stands if the
    round's bus copper leaves it room (joint_escape.carry), and only the balls whose piece does not, or that had
    none, are planned again, at the first round's size."""
    path = awx_settings.get('FANOUT_JOINT')
    if not path or not os.path.isfile(out_path):
        return
    import shutil
    import joint_escape as je
    spec = json.load(open(path))
    stem = out_path[:-len('.kicad_pcb')] if out_path.endswith('.kicad_pcb') else out_path
    prev = awx_settings.get('INCREMENTAL') if PLAN_JUDGE == 'ends' else None
    prev_stem = prev[:-len('.plan.json')] if prev and prev.endswith('.plan.json') else None
    prev_board, prev_side = (f'{prev_stem}.kicad_pcb', f'{prev_stem}.joint.json') if prev_stem else (None, None)
    held = json.load(open(prev_side)) if prev_side and os.path.isfile(prev_side) and os.path.isfile(prev_board) \
        else None
    kept = {}
    cur = out_path
    for a in spec['arrays']:
        ref = a['ref']
        if ref in skip:
            continue                # (laid already, the whole array with the bus: joint_destination)
        nxt = f'{stem}.joint_{ref}.kicad_pcb'
        if held and ref in held:
            hold_array(prev_board, cur, nxt, a, held[ref], spec['layers'], f'{stem}.', 'held from the previous round',
                       log=log)
            rung = None
        else:
            rung, reps = je.fan_array(cur, nxt, ref, [], a['others'], spec['layers'], drops=a['drops'], log=log)
            last = next(r for r in reps if (r['track'], r['via'], r['drill']) ==
                        (rung.fan_track, rung.via_size, rung.via_drill))
            log(f'  joint fanout of {ref}: {len(a["others"])} other nets and the plane balls of '
                f'{len(a["drops"])} nets at track {rung.fan_track} / via {rung.via_size}/{rung.via_drill}: '
                f'{len(last["bare"])} ball(s) bare, {last["undropped"]} plane ball(s) undropped')
        kept[ref] = held[ref] if rung is None else \
            {'track': rung.fan_track, 'via': rung.via_size, 'drill': rung.via_drill}
        cur = nxt
    if cur != out_path:
        shutil.copy(cur, out_path)
        copy_pro(cur, out_path)
    with open(f'{stem}.joint.json', 'w', encoding='utf-8') as f:
        json.dump(kept, f, indent=1)


def hold_array(prev_board, cur, nxt, a, size, layers, stem, why, log=print, in_place=False):
    """The array `a`'s other nets and plane balls (a FANOUT_JOINT array: {"ref", "others", "drops"}) as PREV_BOARD laid
    them, held on CUR: each ball's piece that still stands there kept (joint_escape.carry), and only the balls whose
    piece does not, or that had none, planned again at `size` ({"track", "via", "drill"}, the first round's) -- once
    more with a stuck ball's neighbours released, kept only if that serves more. NXT is the board. `in_place`: the
    pieces are on CUR already (PREV_BOARD is CUR), the ones that no longer stand taken off it."""
    import dataclasses
    import shutil
    import joint_escape as je
    import rules as _rules
    ref = a['ref']
    rung = dataclasses.replace(_rules.active(), fan_track=size['track'], via_size=size['via'], via_drill=size['drill'])
    carried = f'{stem}carried_{ref}.kicad_pcb'
    again, n_kept, n_moved = je.carry(prev_board, cur, carried, ref, a['others'], a['drops'], in_place=in_place)
    log(f'  joint fanout of {ref}, {why}: {n_kept} piece(s) stand, {n_moved} moved; '
        f'{len(again)} ball(s) to plan again at track {rung.fan_track} / via {rung.via_size}/{rung.via_drill}'
        + (f': {again}' if again and len(again) <= 12 else ''))
    if not again:
        shutil.copy(carried, nxt)
        copy_pro(carried, nxt)
        return

    def replan(board, out_b, keys):
        nets_k = {k.split('#')[0] for k in keys}
        o_k = sorted(n for n in a['others'] if je.short_name(n) in nets_k)
        d_k = sorted(n for n in a['drops'] if je.short_name(n) in nets_k)
        # (the array's other nets are the engine's filter: it leaves the carried balls as they stand)
        return je.fan_array(board, out_b, ref, [], o_k, layers, drops=d_k, log=log,
                            only=set(keys), rungs=[rung], filter_nets=a['others'])[1][-1]
    tw = rung.fan_track

    def unserved(board):
        return je.bare_balls(board, ref, a['others'], tw) + je.undropped_balls(board, ref, a['drops'], tw)
    replan(carried, nxt, again)
    left = unserved(nxt)
    stuck = [k for k in left if k in set(again)]
    log(f'  joint fanout of {ref}, planned again: {len(left)} ball(s) unserved' + (f' {left}' if left else ''))
    if stuck:
        # once more with the stuck balls' neighbours released from their held copper, the neighbourhood planned
        # together -- kept only if it serves more
        carried2, nxt2 = f'{stem}carried2_{ref}.kicad_pcb', f'{stem}joint2_{ref}.kicad_pcb'
        again2, k2, m2 = je.carry(prev_board, cur, carried2, ref, a['others'], a['drops'], release_near=stuck,
                                  in_place=in_place)
        replan(carried2, nxt2, again2)
        left2 = unserved(nxt2)
        log(f'  joint fanout of {ref}, {stuck} with their neighbours released ({len(again2)} ball(s) planned '
            f'together): {len(left2)} unserved' + (f' {left2}' if left2 else '')
            + (' -- kept' if len(left2) < len(left) else ' -- not kept'))
        if len(left2) < len(left):
            shutil.copy(nxt2, nxt)
            copy_pro(nxt2, nxt)


def joint_hold(board, log=print):
    """FANOUT_JOINT, the first round's copper HELD to the passives its cap step has just put back over it: the cap
    step nudges the movable passives off the joint fanout's copper (whole_route: --beneath-only, so a part beneath a
    BGA stays beneath it), and the passives stand from then on (FANOUT_PASSIVES_FIXED, which the caller sets); a part
    with nowhere beneath its BGA to go is left on copper it cannot clear -- the zynq's decoupling caps under U1, on B.Cu
    across the others' escapes there. Each array's other nets and plane balls are held on BOARD as a later round holds
    them (hold_array, in place): what stands clear of the passives stays, and what does not is planned again round
    them, at the size BOARD's round kept (<BOARD>.joint.json). BOARD is rewritten."""
    import shutil
    path = awx_settings.get('FANOUT_JOINT')
    stem = board[:-len('.kicad_pcb')] if board.endswith('.kicad_pcb') else board
    side = f'{stem}.joint.json'
    if not path or not os.path.isfile(side):
        log('  joint hold: no joint fanout to hold')
        return
    spec, sizes = json.load(open(path)), json.load(open(side))
    cur = board
    for a in spec['arrays']:
        if a['ref'] not in sizes:
            continue
        nxt = f'{stem}.hold_{a["ref"]}.kicad_pcb'
        hold_array(cur, cur, nxt, a, sizes[a['ref']], spec['layers'], f'{stem}.hold_',
                   'held to the passives where the cap step left them', log=log, in_place=True)
        cur = nxt
    if cur != board:
        shutil.copy(cur, board)
        copy_pro(cur, board)


PAIR_EXIT_REACH = float(awx_settings.get('PLAN_PAIR_EXIT_REACH', '1.2') or 0)


def pair_exit_clear(pcb, nid, other, m, reach=None, free=(), layer=None):
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
    pose at every setback. `free`: nets whose segments are no obstacle
    here -- the run's own stubs, which move with the ends search and whose
    options it tests against the pair's itself. `layer`: the layer to
    check (the move's own by default). On more routing layers than two a
    VIA move's run is laid on the layer the solve gives it (the relayer):
    room on any one of them is room."""
    import pairs as _pairs
    if reach is None:
        reach = PAIR_EXIT_REACH
    if reach <= 0 or getattr(m, 'exit_pt', None) is None:
        return True
    if layer is None and getattr(m, 'kind', 'surface') != 'surface':
        rl_ = route_layers.layers()
        if len(rl_) > 2:
            return any(pair_exit_clear(pcb, nid, other, m, reach, free, layer=L_)
                       for L_ in [m.layer] + [L_ for L_ in rl_ if L_ != m.layer])
    mp = te.build_obstacles(pcb, nid, {nid, other} | set(free), layer or m.layer)
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
            if awx_settings.get('PLAN_PAIR_EXIT_DEBUG'):
                print(f'    exit blocked: {m.net} {m.kind}/{m.direction}/{m.layer[0]} at {0.1 * k:.1f} mm on its own line')
            return False
        for sg in (1, -1):
            if side_ok[sg] and mp.point_violation((x + n[0] * pitch * sg, y + n[1] * pitch * sg)):
                side_ok[sg] = False
        if not (side_ok[1] or side_ok[-1]):
            if awx_settings.get('PLAN_PAIR_EXIT_DEBUG'):
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


def _split_legs(names):
    """{a pair's lane name: its legs} as the whole route names a pair (pairs.pair_names), for the run's nets"""
    import pairs as _pairs_s
    return {b: tuple(pr) for b, pr in _pairs_s.pair_names(list(names)).items()}


def fanout_audit(board, names, dest):
    """THE FANOUT AUDIT, run on every board a fanout hands back (and on each destination pass's): what the whole
    route would refuse on it, by its own tests on the bench the board becomes -- whole_ctx's reading of it (its
    frame, every run net's ends) and whole_frame.build, which stops on a pair SPLIT (another lane's end on the
    pair's layer between its tips, at its tooth or its berth: K51's second round, SDQ3 between SDQS0's berths).
    (splits, far, stops): splits [(pair, 'tooth' | 'berth', [the lanes between its tips])], far [the lanes whose
    tooth stands on the source's far face], stops [why] for anything else the whole route would stop on there. All
    empty: the board may be handed on."""
    import whole_ctx
    import whole_frame
    with awx_settings.given({**awx_settings.environ(), 'BENCH': board, 'NETS': ','.join(names), 'DEST': dest}):
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                ctx, _cs = whole_ctx.plan()
                whole_frame.build(ctx, dest)
        except whole_frame.PairSplit as e:
            return e.splits, [], []
        except whole_frame.FarFace as e:
            return [], e.lanes, []
        except SystemExit as e:
            return [], [], [str(e)]
    return [], [], []


def _audit_line(splits, far, stops):
    return '; '.join([f'{pr_} split at its {end_} (round {", ".join(ls_)})' for pr_, end_, ls_ in splits]
                     + ([f'{", ".join(far)} launch from the source\'s far face'] if far else []) + stops)


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
    best_pass = None       # (ends) the best pass laid: (key, laid_pass, pass)
    learned = set()        # berth pairs the planner must avoid together: laid exactly, in violation of each other
    # PAIR BERTHS (pairs.harmonise, PLAN_PAIRS): a differential pair's two
    # berths are made one move -- same face, layer and kind, neighbouring
    # exits -- before the engine lays them, every pass. Off unless the
    # braid routes pairs as members (BRAID_PAIRS), so the chain is
    # byte-identical without it; inert on a run with no pairs.
    _harm = int(awx_settings.get('PLAN_PAIRS', awx_settings.get('BRAID_PAIRS', '0')) or 0)
    _pitch = 0.0
    if _harm:
        import pairs as _pairs
        _g = em.grid_of(_pcb0.footprints[dref])
        _pitch = max(_g.pitch_x, _g.pitch_y)
    for it in range(DST_ITERS):
        if _harm:
            _conf = pe.sm._conflict
            if PLAN_JUDGE == 'ends':
                # the ends model's own test, as its berths were chosen: strict, and an F exit over a B one no conflict
                import pages_first as _pf
                _conf = lambda m, om, strict=False: pe.sm._conflict(m, om, strict=bool(_pf.PAGES_STRICT), stack=True)
            _pairs.harmonise(choice, st['dmenu'], names, _pitch, _conf, print)
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
        if PLAN_JUDGE == 'ends':
            # every pass graded -- its berths not as asked or in a DRC violation, then the ends model's objective on
            # what it asked -- and the best one's board kept beside: it ships, not merely the last (a pass's re-plan
            # can ask worse ends than the pass before, and the passes can run out with a better one unlaid)
            try:
                _v = judge_by_braid(st, choice, board)[0]
            except Exception:
                _v = float('inf')
            _k = (len(misses) + len(drc_nets), _v)
            if best_pass is None or _k < best_pass[0]:
                for ext_ in ('.kicad_pcb', '.kicad_pro'):
                    src_ = out_path[:-len('.kicad_pcb')] + ext_
                    if os.path.isfile(src_):
                        shutil.copyfile(src_, out_path[:-len('.kicad_pcb')] + '.bestpass' + ext_)
                best_pass = (_k, laid_pass, it)
            print(f'  destination pass {it}: graded {_k[0]} not as asked, objective {_k[1]:.2f}'
                  + (' (the best so far)' if best_pass[2] == it else ''))
        pair_only = []
        if drc_nets and (PLAN_PAGES or PLAN_JUDGE == 'ends'):
            # two berths laid as asked but in violation of EACH OTHER: the pair of moves is learned (the planner
            # avoids them together), neither banned -- each may be fine beside another neighbour. A berth in
            # violation of anything else (static copper, a net outside the choice, a berth not laid as asked) is
            # refused as before
            dp_ = [tuple(pr) for pr in getattr(fanout_once, 'drc_pairs', ()) if len(pr) == 2]
            for nm in drc_nets:
                partners = [b if a == nm else a for a, b in dp_ if nm in (a, b)]
                if partners and all(q in drc_nets for q in partners):
                    pair_only.append(nm)
            for a, b in dp_:
                if a in pair_only and b in pair_only:
                    learned.add(frozenset((sr.move_sig(choice[a]), sr.move_sig(choice[b]))))
            if pair_only:
                print(f'  destination pass {it}: {len(pair_only)} berth(s) laid as asked but in violation of each '
                      f'other -> the pairs learned, re-planned: {pair_only}')
            drc_nets = [nm for nm in drc_nets if nm not in pair_only]
        if drc_nets:
            print(f'  destination pass {it}: {len(drc_nets)} berth(s) laid as asked but in a '
                  f'DRC violation -> treated as refused: {drc_nets}')
            misses += drc_nets
        if not misses and not pair_only:
            # ...and no pair SPLIT on the board laid: a fanout never hands back a pair with another lane's end
            # between its tips on its layer (the whole route stops there: K51's second round, SDQ3 between SDQS0's
            # berths). The lanes between a pair's berths are re-planned as a berth not laid as asked is
            sp, far, stops = fanout_audit(out_path, names, dref)
            legs_ = _split_legs(names)
            bad = sorted({l_ for _pr, end_, lanes_ in sp if end_ == 'berth' for ln in lanes_
                          for l_ in legs_.get(ln, (ln,)) if l_ in choice})
            if not sp and not far and not stops:
                print(f'  destination pass {it}: every berth laid as planned')
                break
            print(f'  destination pass {it}: every berth laid as planned, but the fanout audit finds: '
                  + _audit_line(sp, far, stops))
            if not bad:
                break
            misses = bad
        if misses:
            ban_moves(banned, choice, misses, names)
            print(f'  destination pass {it}: {len(misses)} berth(s) not laid as asked '
                  f'-> banned, re-planning the destination: {misses}')
        st = plan_state(parse_kicad_pcb(board), names, banned)
        # the berths laid exactly stay as laid (but those of a learned pair, which are re-planned)
        laid_ok = {nm: sr.move_sig(choice[nm]) for nm in choice
                   if audit_d.get(nm, {}).get('exact') and nm not in pair_only}
        # pages-first: the berths laid exactly stay FIXED, only the missed
        # nets are re-planned -- a re-plan from scratch asked for 5-6 new
        # berths every pass and never converged (K28, 8 passes, 27 bans).
        # (ends) The re-plan starts from this pass's choice, and a lane it
        # still finds in a CONFLICT has its held neighbours freed with it,
        # rather than a known conflict laid: a lane freed alone with no
        # clean option kept its old berth beside a held one
        new_choice, un = dest_choice(st, board,
                                     fixed=laid_ok if PLAN_PAGES else None,
                                     learned=learned, seed=(dict(choice) if PLAN_JUDGE == 'ends' else None))
        if PLAN_JUDGE == 'ends' and PLAN_PAGES:
            import whole_ends as _we2
            _cl = (_we2.choose.last.get('laid_parts') or {}).get('conf_lanes') or []
            _legs2 = _split_legs(names)
            _unfix = sorted({l_ for ln in _cl for l_ in _legs2.get(ln, (ln,)) if l_ in laid_ok})
            if _unfix:
                print(f'  destination re-plan: still a conflict ({", ".join(_cl)}) -- re-planned with {_unfix} freed')
                for l_ in _unfix:
                    laid_ok.pop(l_, None)
                new_choice, un = dest_choice(st, board, fixed=laid_ok, learned=learned, seed=dict(choice))
        if not new_choice or new_choice == choice:
            print('  destination: the re-plan changed nothing -- stopping')
            break
        f, _p, _bp, _pl = judge_by_braid(st, new_choice, board)
        print(f'  destination re-plan: braid-judged {f:.2f}, {len(new_choice)} placed'
              + (f', {len(un)} unplaced' if un else ''))
        choice, dst_pad = new_choice, st['dst_pad']
    # The LAST PASS SHIPS for the braid's judges, and its sidecar is its
    # own: the choice that was fanned out and audited, never the re-plan
    # after it (which is a choice no board was laid to). Measured over
    # every K41 pass board (2026-09-07, the braid on each): passes 0..7
    # graded 6/3/4/5/3/1/1/1 open at 86/108/92/74/81/98/102/78 vias -- the
    # last pass best, and neither the audit's exact count (pass 5: 38/40 vs
    # 34/40, 1 open at 98) nor the judge's cost (pass 6 the lowest, 1 open
    # at 102 with 6 DRC) picks a better one. Judged on the WRITTEN fanout
    # board: the same copper the braid will read, so its taut paths are
    # the memo's. Under the ENDS judge the BEST pass ships, graded above.
    if best_pass is not None and best_pass[1] is not laid_pass:
        # the best pass's board back in place of the last's
        for ext_ in ('.kicad_pcb', '.kicad_pro'):
            src_ = out_path[:-len('.kicad_pcb')] + '.bestpass' + ext_
            if os.path.isfile(src_):
                shutil.copyfile(src_, out_path[:-len('.kicad_pcb')] + ext_)
        print(f'  destination: pass {best_pass[2]} ships (the best laid, {best_pass[0][0]} not as asked, objective '
              f'{best_pass[0][1]:.2f}), not the last')
        laid_pass = best_pass[1]
    for ext_ in ('.kicad_pcb', '.kicad_pro'):       # (the best pass's copy, kept beside while the passes ran)
        if os.path.isfile(out_path[:-len('.kicad_pcb')] + '.bestpass' + ext_):
            os.remove(out_path[:-len('.kicad_pcb')] + '.bestpass' + ext_)
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
            ship_vias.stamp(out_path, 'fanout ties', print)
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
    import joint_escape as _je
    _je.reserve_ball_vias(pcb)          # the joint fanout: the other balls' own via sites stay free
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        pcb.footprints[dref], pcb, net_filter=targets, layers=route_layers.stacked(pcb.board_info.copper_layers),
        track_width=sr.FAN_TRACK, clearance=sr.FAN_CLEAR, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
        exit_margin=0.5, escape_method='underpad', plane_drop='off',
        escape_dir_hints=hints)
    if tracks:
        add_tracks_and_vias_to_pcb(
            src_file, out_path, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb.nets.items()})
        ship_vias.stamp(out_path, 'fanout', print)
    else:
        shutil.copy(src_file, out_path)
    if relay is not None:
        os.remove(src_file)
    copy_pro(board, out_path)
    r = subprocess.run([sys.executable,
                        os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                        out_path, '--clearance', str(te.SPEC_CLEARANCE),
                        '--clearance-margin', '0.1',
                        # or check_drc truncates each category at 20 and the
                        # nets beyond that are never banned, never freed
                        '--max-print', '0',
                        # the violations of the run's nets, as the chain grades
                        # its routed board: a board the chain hands on carries
                        # its earlier steps' violations between other nets (the
                        # zynq's NetC146_2 endpoint gap, an earlier step's),
                        # which no fanout of the run's nets made or can mend,
                        # and read whole it failed every fanout of the run
                        '--nets'] + [f'*{nm}' for nm in names],
                       capture_output=True, text=True, env=awx_settings.environ())
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
    for a, b in _re.findall(r'^\s+(.+?) <-> (.+?)\s*$', _drc_txt, flags=_re.M):
        a, b = drc_side_net(a), drc_side_net(b)
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
