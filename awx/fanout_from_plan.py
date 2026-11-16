#!/usr/bin/env python3
"""Fan out the destination array in the directions the PLAN chose.

The plan (the whole route's ends model, whole_ends) picks, per ball, an
escape move, and hands it to the production fanout engine (generate_bga_fanout) as
escape_dir_hints as FULL moves (face, exit gap, layer, kind, dog-bone
site). A hint accepted is not a move achieved, so the copper that
actually leaves each ball is measured against what was asked for, per
dimension and as an order along each face (source_realize.audit).

The source array arrives already fanned out (the bench): the ends are
chosen against the teeth as they are on the board, the teeth the ends
ask REALIZED by re-fanning those balls with the same engine
(source_realize: every tooth audited, original vs asked vs achieved),
a realized board kept only when it judges better on its teeth as laid,
and the destination chosen against the teeth the kept board's copper
has. That board is the one the destination fanout runs on.

usage: fanout_from_plan.py OUT.kicad_pcb K --board=BASE.kicad_pcb
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
from coherent_nets import coherent_nets, net_patterns, net_keys, engine_filter  # noqa: E402
import rules as _rules  # noqa: E402  ONE source for every design rule
import route_layers  # noqa: E402  the routing layers: the escapes' runs may take any of them

# SRC_CLIMB=k: the SOURCE menu also offers CLIMBS -- a dog-bone or via-in-pad
# whose run first travels up to k pitches along a gap under the array and
# leaves the face at a chosen row or column (escape_moves.enumerate_moves
# climb=); 0 = none. With SRC_RUN_INNER, six: a ball in the middle of a
# shared array whose only ways toward its destination were a surface run
# out of the far half, through another bus's balls, leaves by a dog-bone and
# a run on an inner layer under the array, as the human's does.
SRC_CLIMB = int(awx_settings.get('SRC_CLIMB', '6'))
# SRC_RUN_INNER=1 (more routing layers than two): a source via's run -- in its pad, a dog-bone, a climb -- may lie on
# the board's inner SIGNAL layers where it cannot on B.Cu (escape_moves run_layers): the human's escapes reorder the
# lanes under the array on the free inner layers, where the back layer holds the passives under it. 0 = the menu
# as on two layers
SRC_RUN_INNER = (awx_settings.get('SRC_RUN_INNER') or '1') == '1'
# (the whole route's ends) a destination ROW PART's pad also berths on a via INSIDE the part, behind its row --
# between a two-row connector's rows -- its lane out under the rows by the face toward the source (escape_moves.Inward)
# DST_ALONG=k: ...and its SURFACE lanes along that strip out of either end of the part, k of them (escape_moves.Inward
# `along`). 0 = none
DST_ALONG = int(awx_settings.get('DST_ALONG') or 3)
# NO BALL WALLED IN (the whole route's ends): a source realize whose teeth WALL IN a ball of the source on no bus net
# (the step's own cage test, route_bus.walls_made) is not kept -- the move walling it banned, the plan chosen again
# without it. Two F.Cu runs from an inner via to the face, either side of a row, took every via site of the ball
# between them, and its net shipped open. And a destination pass whose berths wall in a ball of a destination part
# likewise: the walling berths re-planned as a berth not laid as asked is (fanout_destination) -- a BGA's berths' inner
# runs round an RF ground ball, twelve lanes' copper about it on every layer
SRC_WALLS = True            # (the test is always on here; escape_plan.run reads this)
WALL_RETRIES = 3            # a walling realize: the rest of its moves realized without the walling ones, so often
# DST_CLIMB=k enumerates destination dog-bones whose run climbs along the
# array up to k pitches before it leaves (escape_moves climb=), the class
# that makes a B berth's rank free.
DST_CLIMB = int(awx_settings.get('DST_CLIMB', '0'))
# DST_STREET: destination STREET dog-bones (escape_moves street=) -- a via in an empty band of the destination
# array, on a lane a track pitch from the next, at DST_STREET sites along it, the run leaving toward the source: the
# braid they give is far simpler (K35: 132 crossings against 162, the solve 10 s against 121, the whole run 287 s
# against 606), at 58 vias against 52
DST_STREET = 2
# The whole route's own ENDS model (whole_ends.py) both CHOOSES the berths and the teeth to move and JUDGES a
# candidate on the ends model's objective (whole_ends: the ends' vias and the route's, estimated from the ends and
# ranked exact on their orders, the ride, congestion and feedback), in seconds, with no whole solve and no braid
# planner. A realized board is judged on its teeth AS LAID.


from escape_moves import DIRS, LAYERS  # noqa: E402,F401  -- ONE source


def copy_pro(src_board, dst_board):
    # every sibling: the project, and the .kicad_dru's per-layer rules (#498) a bus step on inner layers must keep
    from copy_board import copy_siblings
    copy_siblings(src_board, dst_board)
    # ...and stamp the floor this stage fans out at (0.1 / 0.1 / the braid's
    # via, the numbers fanout_once is called with below), lower-only, as the
    # production CLIs do -- see braid.write_out for why a bare copy was not
    # enough (a Default class clearance of 0.0 rode down every chain step).
    try:
        from fix_kicad_drc_settings import fix_project_for_output
        fix_project_for_output(dst_board, src_board, clearance=te.SPEC_CLEARANCE,
                               track_width=sr.FAN_TRACK,
                               via_diameter=te.VIA_SIZE, via_drill=te.VIA_DRILL,
                               verbose=False)
    except Exception as e:
        print(f'  project floor NOT stamped: {e}', flush=True)


def _drop_tie_siblings(tmp):
    # the writer seeds a temp board's siblings beside it (#1286); the shipped board keeps its own
    from copy_board import SIBLING_EXTS
    for ext in SIBLING_EXTS:
        try:
            os.unlink(tmp[:-len('.kicad_pcb')] + ext)
        except OSError:
            pass


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


def unbanned_or_all(nm, moves, banned, what):
    """`nm`'s moves less those the fanout refused (`banned`: by signature, and by class) -- or, where that leaves none,
    all of them, said: a net with no move stops the ends model's whole search (the zynq's USB bus on the six-rail board
    with the LVDS step's field: OTG_NXT's one tooth class banned when the engine laid it as a via in pad, its laid tooth
    on an inner layer no move is, and the fanout died, the bus laid nothing), where a refused move is only a move the
    engine may lay otherwise"""
    import source_realize as sr
    ok = [m for m in moves if (nm, sr.move_sig(m)) not in banned and (nm, 'class', sr.move_class(m)) not in banned]
    if moves and not ok:
        print(f'  {nm}: every {what} move refused once -- offered again, the net has no other')
        return list(moves)
    return ok


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
    from coherent_nets import by_key
    byname = by_key(pcb)
    # (each net's two ends told apart by the run's destination, DEST: on a board both arrays are fanned on -- a round's
    # bench, its crowded ends escaped again -- the source's free end is no longer the only one, and the first found was
    # taken for the tooth: a memory bus's data line re-escaped at its berth, its tooth asked off F.Cu searched among the
    # destination's escapes and none found. Not yet fanned, the destination has no free end and the source's stand)
    ends = te.endpoints(pcb, names, byname, dest_ref=awx_settings.get('DEST') or None)
    kids = {byname[n][0] for n in names}
    # a pair's exit room is checked against copper OUTSIDE the run: the ends model's search moves the run's stubs and
    # tests their options against the pair's (whole_ends)
    run_free = kids
    cache = {}

    def obs(nid, layer):
        # `kids` (every net of the run) is excluded for the taut paths and both menus: the destination's a bare
        # array, the source's priced against copper outside the run (src_moves, below)
        key = (nid, layer)
        if key not in cache:
            cache[key] = te.build_obstacles(pcb, nid, kids, layer)
        return cache[key]

    # a bus escape on the outer layers (route_layers.escape_layers: a via's run on an inner layer is the solve's), its
    # via clear on every copper layer it stands through where the run may take an inner one (an earlier round's run
    # moved there stands in it)
    via_layers = (list(pcb.board_info.copper_layers or ()) if len(route_layers.layers()) > 2 else None)

    key_of_id = {i: k for k, (i, _n) in byname.items()}

    def menu(pad, grid, nid, climb=0, street=0, street_dirs=None, run_layers=(), row_inward=None):
        # the straight escape along the ball's own line too (escape_moves `straight`)
        return em.enumerate_moves(
            pad, grid, route_layers.escape_layers(pcb.board_info.copper_layers),
            lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
            lambda p, L, _n=nid: not any((obs(_n, L_).point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0] for L_ in (via_layers or (L,))),
            climb=climb, straight=True,
            street=street, street_pitch=pe.sm._STACK_PITCH + 1e-4, street_dirs=street_dirs, net=key_of_id.get(nid),
            run_layers=run_layers, row_inward=row_inward)
    # the inner signal layers a via's run may take, the routing layers no plane fills: a source's (SRC_RUN_INNER), a
    # row part's via inside it (escape_moves.Inward)
    inner_runs = ()
    if len(route_layers.layers()) > 2:
        kinds_ = route_layers.plane_kinds(pcb, route_layers.layers())
        inner_runs = tuple(L for L in route_layers.layers()
                           if L not in ('F.Cu', 'B.Cu') and kinds_.get(L, '-') == '-')
    src_runs = inner_runs if SRC_RUN_INNER else ()
    # ...and a ball with NO via escape on those (another step's copper across every via's run on the outer layers): its
    # via's run on every inner routing layer, a plane's too -- the solve and the ends price a run there (route_layers.
    # run_price) -- where its one surface escape runs into copper nothing may move (an earlier bus's locked escape across
    # the channel at the array's edge)
    all_inner = tuple(L for L in route_layers.layers() if L not in ('F.Cu', 'B.Cu'))

    def hemmed(nm, moves, again, what):
        """`moves`, and where none is a via's, the via escapes `again(run_layers)` finds with the run on an inner layer"""
        if not all_inner or any(m.vias for m in moves):
            return moves
        more = [m for m in again(all_inner) if m.vias and any(L in all_inner for _a, _b, L in m.legs)]
        if more:
            print(f'  {nm}: no via {what} move on the outer layers -- {len(more)} with the run on an inner layer offered')
        return moves + more
    dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
    dref = ends[names[0]][2]
    # (DEST naming several parts, one source and several ends: ONE destination, the GROUP of them -- whole_frame.Group;
    # the first is the plan's destination array, each net's berth laid at its own part)
    _dl = te.dest_refs(awx_settings.get('DEST') or '')
    if len(_dl) > 1:
        dref = _dl[0]
    dgrid = em.grid_of(pcb.footprints[dref])
    dboxes = dgrid.bbox
    _dests = [dref] + [r_ for r_ in _dl[1:] if r_ != dref] if len(_dl) > 1 else [dref]
    _dgrid_of = {r_: (dgrid if r_ == dref else em.grid_of(pcb.footprints[r_])) for r_ in _dests}
    # the destination's FAR face, the one facing away from the source: under the whole route's ends a far-face move
    # is offered only to a ball in the array's half beside that face -- in ten fanouts over K15-K41 the ends model
    # chose 3 of 341 berths there, one net one column from the far face (K41 SA11), from 518 of 2269 menu moves
    # (every dog-bone and via-in-pad among them unchosen). Each destination's own, where DEST names several
    _sc = [sum(ends[n][0][k] for n in names) / len(names) for k in (0, 1)]
    _fd = {}

    def faces_of(ref):
        """(far, toward, the far face's axis, the box) of destination `ref`"""
        if ref not in _fd:
            b_ = dboxes if ref == dref else em.grid_of(pcb.footprints[ref]).bbox
            c_ = ((b_[0] + b_[2]) / 2, (b_[1] + b_[3]) / 2)
            f_ = max(em.DIRS, key=lambda d: em.DIRS[d][0] * (c_[0] - _sc[0]) + em.DIRS[d][1] * (c_[1] - _sc[1]))
            t_ = max(em.DIRS, key=lambda d: em.DIRS[d][0] * (_sc[0] - c_[0]) + em.DIRS[d][1] * (_sc[1] - c_[1]))
            _fd[ref] = (f_, t_, 0 if em.DIRS[f_][0] else 1, b_, c_)
        return _fd[ref]

    def far_half(pad):
        far, _t, _fax, b_, _c = faces_of(pad.component_ref)
        mid = (b_[_fax] + b_[_fax + 2]) / 2
        return ((pad.global_x, pad.global_y)[_fax] - mid) * em.DIRS[far][_fax] >= -1e-6
    for nm in names:
        nid, net = byname[nm]
        fp = pcb.footprints[ends[nm][2]]
        bx, by = ends[nm][1]
        pad = min(fp.pads, key=lambda p: (p.global_x - bx) * (p.global_x - bx)
                  + (p.global_y - by) * (p.global_y - by))
        dst_pad[nm] = pad
        far, toward, _fax, _b, _c = faces_of(ends[nm][2])
        # (the ends: a row part's pad also berths on a via inside the part, its lane out by the face toward the
        # source)
        iw_ = em.Inward(toward, te.VIA_SIZE, sr.FAN_CLEAR, sr.FAN_TRACK, inner_runs, along=DST_ALONG,
                        along_pitch=te.MINP) \
            if em.row_part(fp) else None
        def dst_moves(run_layers=()):
            ms_ = dedupe_climbs(menu(pad, em.grid_of(fp), nid, climb=DST_CLIMB, street=DST_STREET,
                                     street_dirs=(toward,), run_layers=run_layers, row_inward=iw_))
            return ms_ if far_half(pad) else [m for m in ms_ if m.direction != far]
        moves = hemmed(nm, dst_moves(), dst_moves, 'berth')
        dmenu[nm] = unbanned_or_all(nm, moves, banned, 'berth')
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
    # a TIE VIA's site (tie_site: a ball with a pad of its own net under it, tied by a via there once its berth is laid)
    # is a via of its net's from the start: another run net's berth runs and stands off it by the fanout's rule. The
    # menus above are built with the run's own copper out (a bare array), and the zynq7020's DDR_CK_P was offered -- and
    # chosen -- a via in its pad running along its ball row across DDR_CK_N's ball, where the tie via then stood
    ties = {nm: t for nm in names if (t := tie_site(dst_pad[nm], byname[nm][1])) is not None}
    if ties:
        from routing_defaults import HOLE_TO_HOLE_CLEARANCE
        run_bar = te.VIA_SIZE / 2 + sr.FAN_CLEAR + sr.FAN_TRACK / 2
        via_bar = max(te.VIA_SIZE + sr.FAN_CLEAR, te.VIA_DRILL + HOLE_TO_HOLE_CLEARANCE)

        def on_tie(m, t):
            return any(_pairs_m._pt_seg(t, a, b) < run_bar - 1e-6 for a, b, _L in (m.legs or ())) or \
                (m.site is not None and math.hypot(m.site[0] - t[0], m.site[1] - t[1]) < via_bar - 1e-6)
        import pairs as _pairs_m
        for nm in names:
            sites = [t for o, t in ties.items() if o != nm]
            if not sites:
                continue
            keep = [m for m in dmenu[nm] if not any(on_tie(m, t) for t in sites)]
            if len(keep) < len(dmenu[nm]):
                print(f'  {nm}: {len(dmenu[nm]) - len(keep)} of {len(dmenu[nm])} berth moves run or stand on another '
                      f'net\'s tie via site -- dropped')
                if keep:
                    dmenu[nm] = keep
    refs = {}
    for nm in names:
        if src_pad[nm] is not None:
            refs[src_pad[nm].component_ref] = refs.get(
                src_pad[nm].component_ref, 0) + 1
    sref = max(refs, key=refs.get)
    sgrid = em.grid_of(pcb.footprints[sref])
    smenu = {}
    # a tooth move to the source's FAR face (the one facing away from the destination) only from a ball in the array's
    # half beside it, as a side face's (side_half, below) -- the whole route takes such a lane round the source on a
    # lead-in (whole_frame), and the ends model prices it past any via count (whole_ends.FAR_TOOTH), so it is chosen
    # only by a lane with no other way out. (Every destination's pads, where DEST names several: the source's far face
    # faces away from them all)
    _dp = [(p_.global_x, p_.global_y) for r_ in sorted({ends[n][2] for n in names})
           for p_ in pcb.footprints[r_].pads]
    _sp = [(p_.global_x, p_.global_y) for p_ in pcb.footprints[sref].pads]
    _fx = sum(x for x, _y in _dp) / len(_dp) - sum(x for x, _y in _sp) / len(_sp)
    _fy = sum(y for _x, y in _dp) / len(_dp) - sum(y for _x, y in _sp) / len(_sp)
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
        # rotate (a run net's laid via still refuses it)
        def src_moves(run_layers):
            return [m for m in dedupe_climbs(menu(p, sgrid, byname[nm][0], climb=SRC_CLIMB, run_layers=run_layers))
                    if m.direction == near_dir or side_half(p, m.direction)]
        smenu[nm] = unbanned_or_all(nm, hemmed(nm, src_moves(src_runs), src_moves, 'tooth'), banned, 'tooth')
        _plegs = getattr(plan_state, '_pair_legs', None) or {}
        if nm in _plegs and _plegs[nm] in byname:
            keep = [m for m in smenu[nm] if pair_exit_clear(pcb, byname[nm][0], byname[_plegs[nm]][0], m, free=run_free)]
            if len(keep) < len(smenu[nm]):
                print(f'  {nm}: {len(smenu[nm]) - len(keep)} of {len(smenu[nm])} tooth moves have no room '
                      f'for the pair at the exit -- dropped' + ('' if keep else ', none left: the menu kept whole'))
            # (all of them where none has, as the berths' menu keeps its moves: emptied, a leg had no tooth at all --
            # four of four, eight of eight on an escape bus's clock and data pairs -- and an empty menu stops the
            # ends' search, unbanned_or_all)
            if keep:
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
    # the run nets whose LAID copper each tooth move passes through (source_realize.blockers_of, over the run's own
    # copper): the move and that net's laid tooth are one conflict (whole_ends) -- the laid tooth is a move with no legs
    # (source_realize.current_tooth), so the lane test alone would plan a move through a tooth that stays
    sblock = {}

    class _RunCopper:
        segments = [sg for sg in pcb.segments if sg.net_id in kids]
        vias = [v for v in pcb.vias if v.net_id in kids]
    for nm, ms in smenu.items():
        for m in ms:
            mv_, _pin = sr.blockers_of(_RunCopper, m, byname[nm][0], byname, set(names))
            if mv_:
                sblock[(nm, id(m))] = mv_
    # an INCREMENTAL fanout (INCREMENTAL= the previous round's sidecar, FEEDBACK= its findings): only the ends the
    # feedback names move -- the teeth it names are free, every other tooth stands as laid on this board, and every
    # berth it does not name is held at the previous round's (the menu move at that berth's point and layer): what
    # worked stays
    incr = None
    if awx_settings.get('INCREMENTAL') and awx_settings.get('FEEDBACK'):
        import pairs as _pairs_i
        import whole_ends as _we
        prev = json.load(open(awx_settings.req('INCREMENTAL')))
        fb = json.load(open(awx_settings.req('FEEDBACK')))
        items = [e for pr in fb.get('pairs', ()) for e in pr] + list(fb.get('avoid', ()))
        items += list(fb.get('ban', ())) + list(fb.get('pin', ()))       # (the round edits' ends: freed too)
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
    import whole_ends as _we_fb
    return {
            'banned': banned,          # the feasibility ledger, for a proposal
                                       # that enumerates its own moves
            'byname': byname, 'dmenu': dmenu, 'smenu': smenu, 'sblock': sblock, 'launch': launch,
            # the whole route's feedback (whole_feedback.py): ends its audits found crowded, and the run's blocked
            # teeth (ENDS_BLOCKED), priced by whole_ends
            'feedback': _we_fb.round_feedback(),
            'incr': incr,
            'tooth0': tooth0, 'tooth_vias': tooth_vias, 'src_pad': src_pad,
            'dst_pad': dst_pad, 'sref': sref, 'dref': dref, 'sgrid': sgrid,
            'bundle_layer': bundle_layer, 'chi': chi,
            'dgrid': dgrid, 'dboxes': dboxes,
            # (DEST naming several parts, a GROUP: each lane's own part -- whole_ends reads each lane's own)
            'dests': _dests, 'dgrid_of': _dgrid_of, 'dbox_of': {r_: g_.bbox for r_, g_ in _dgrid_of.items()},
            'buses': buses, 'obs': obs, 'pcb': pcb,
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


def judge_by_braid(st, choice, board, achieved=None):
    """THE judgment of a candidate plan: the whole route's ends model on
    the plan's ends (whole_ends.judge: the ends' vias and the route's,
    estimated from the ends and ranked exact on their orders, the ride,
    congestion and feedback), its parts kept in `judge_by_braid.ends`.
    Returns (cost, {}, {}, the plan dict as the braid's planner takes it)."""
    plan = braid_plan_of(st, choice, board, achieved)
    import whole_ends
    v, parts = whole_ends.judge(st, choice)
    judge_by_braid.ends = parts
    return v, {}, {}, plan


def _pair_legs_of(base, names):
    """The two nets of pair `base` among `names`."""
    import pairs as _pairs
    return _pairs.pair_names(list(names)).get(base, ())


def drc_side_net(side, keys=None):
    """The net of one side of a check_drc violation line, by the whole route's name (`keys`, coherent_nets.net_keys),
    else by its last part: the kind prefix (`Seg:`, `Pad:`) and the TRAILING annotations stripped (`(REF.PAD)`,
    `[SHORT]`, `(drill hole clearance)`) -- never split on whitespace: a board's nets may be `/DDR3 16x1/SDQ2`, so a
    space split yields `DDR3`."""
    import re as _re
    from coherent_nets import key_of
    side = side.strip()
    if ':' in side[:6]:
        side = side.split(':', 1)[1]
    for _ in range(3):
        side = _re.sub(r'\s*\[[^\]]*\]\s*$', '', side)
        side = _re.sub(r'\s*\([^)]*\)\s*$', '', side)
    return key_of(side.strip(), keys)


def drc_line_nets(line, keys=None):
    """(net, net) of a check_drc violation line `A <-> B`, or None (drc_side_net)"""
    import re as _re
    mt = _re.match(r'^\s*(.+?) <-> (.+?)\s*$', line)
    if not mt:
        return None
    a, b = drc_side_net(mt.group(1), keys), drc_side_net(mt.group(2), keys)
    return (a, b) if a and b else None


def ban_moves(banned, moves, nets, names):
    """Ban `nets`' moves in `moves` {net: Move} (the fanout would not lay them as asked). A PAIR's two legs are ONE
    joint move: a leg whose partner's move is in `moves` too bans the pair's move as a unit -- ('pair', P, N, P's
    signature, N's signature), which whole_ends drops from the pair's options -- never a leg's move alone (with
    another partner move it is another joint move); a leg moved alone is banned alone. A ban also covers the move's
    CLASS (source_realize.move_class: every leg variant of the same exit), which the engine refuses for the same
    reason -- banned one variant at a time, the re-plans asked them in turn"""
    import pairs as _pairs
    legs = {}
    for pn, nn in _pairs.pair_names(list(names)).values():
        legs[pn] = legs[nn] = (pn, nn)
    for nm in nets:
        pr = legs.get(nm)
        if pr and pr[0] in moves and pr[1] in moves:
            banned.add(('pair', pr[0], pr[1], sr.move_sig(moves[pr[0]]), sr.move_sig(moves[pr[1]])))
            banned.add(('pairclass', pr[0], pr[1], sr.move_class(moves[pr[0]]), sr.move_class(moves[pr[1]])))
        else:
            banned.add((nm, sr.move_sig(moves[nm])))
            banned.add((nm, 'class', sr.move_class(moves[nm])))


def split_pairs(st):
    """(count, [pair]) of the differential pairs whose TEETH, as laid, stand on
    different faces or layers of the source array -- a pair that cannot be
    launched coupled."""
    import pairs as _pairs
    out = []
    for base, (pn, nn) in _pairs.pair_names(list(st['launch'])).items():
        a, b = sr.current_tooth(st, pn), sr.current_tooth(st, nn)
        if a is not None and b is not None and (a.direction, a.layer) != (b.direction, b.layer):
            out.append(base)
    return len(out), sorted(out)

def pf_key(choice, bp, cost):
    """The KEY a realize-and-confirm site compares plans on: (the ends
    model's cost, residue -- the nets `bp` pages none of): the cost decides,
    the residue is the tie-break."""
    resid = sum(1 for nm in choice if bp.get(nm, {}).get('page') is None)
    return (cost, resid)


def pf_better(new, best):
    """Is key `new` better than `best`? Lexicographic."""
    return new < best


def pf_fmt(k0, k1):
    """`judged count X -> Y, residue A -> B`"""
    return f'judged count {k0[0]:.0f} -> {k1[0]:.0f}, residue {k0[1]} -> {k1[1]}'


SRC_RESIDUE_ROUNDS = 8      # one tooth realized -> re-chosen, at most this often a round
TEETH_AHEAD = 3             # a realized board worse as laid: the teeth its own choice asks realized too, this often


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
    the whole route's ends model from it (the greedy's move stays the
    fallback for a net the model leaves out). ONE function for the first
    plan and for every re-plan after a refused berth. Returns (choice,
    unplaced)."""
    pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y)
            for nm in st['dst_pad']}
    choice, un = pe.sm.select(st['dmenu'], st['launch'],
                              keep_out=st['dboxes'], buses=st['buses'],
                              tooth_layer=st['tooth0'], log=None, pads=pads, chi=st['chi'])
    if choice:
        # the whole route's ends model: every berth (and, with `src_out`, the teeth to move) from `seed` (a previous
        # choice) or the greedy's
        import whole_ends
        inc = st.get('incr') or {}
        ch, src = whole_ends.choose(st, log=log or (lambda *a: None), src_free=(src_out is not None),
                                    fixed=(fixed if fixed is not None else inc.get('fixed')),
                                    seed=seed or choice,
                                    learned=learned, free_teeth=inc.get('free_teeth'))
        for nm, mv in choice.items():
            ch.setdefault(nm, mv)
        un = [nm for nm in un if nm not in ch]
        choice = ch
        if src_out is not None:
            src_out.update(src)
    if choice and fixed:
        # the berths a caller holds fixed (a re-plan on a realized source
        # board keeps the previous choice) bind the greedy's answer too
        for nm, sig in fixed.items():
            m = next((mm for mm in st['dmenu'].get(nm, ()) if sr.move_sig(mm) == sig), None)
            if m is not None:
                choice[nm] = m
    return choice, un


def plan(base, names, work):
    """The plan: the ends model chooses the destination escapes, and the
    teeth to move, against the source teeth AS THEY ARE on the board; the
    teeth it asks are REALIZED -- those nets' source copper stripped and
    re-fanned with the production engine in the asked faces
    (source_realize, which audits every tooth: original vs asked vs
    achieved) -- and a realized board is kept only when the judge, on its
    teeth as laid, says it is better; the destination is then chosen
    against the teeth the kept board's copper has. The board and the
    destination choice are what the destination fanout then runs on."""
    board = base
    realized = []
    banned = set()          # (net, move signature) the fanout refused
    r = 0                   # (the one round: its boards and lines are numbered 0)
    st = plan_state(parse_kicad_pcb(board), names, banned)
    cache = {}
    src_out = {}
    dst_choice, un = dest_choice(st, board, src_out=src_out)
    if not dst_choice:
        print(f'  round {r}: no destination choice')
        # (no round chose the destination's ends at all: no plan to keep -- said so, the fanout's own exit, not a crash)
        raise SystemExit('fanout_from_plan: no round chose the destination\'s ends -- no plan')
    if src_out:
        # the ends asked for source moves: realize them with the engine,
        # choose the destination again on the new board, and KEEP the new
        # board only if the judge says it is better -- a realized tooth
        # changes every lane's neighbours. A rejected round's moves are
        # banned. (The ends model judges a board on its teeth AS LAID: the
        # berths searched against them)
        import whole_ends
        best_key = pf_key(dst_choice, {}, whole_ends.choose.last['laid'])
        best_split = split_pairs(st)

        def _trial(moves, new_board):
            """Lay `moves` on `board` with the engine, choose the destination
            again on the new board, judge it. (result, line) -- result None
            when it was not laid as asked or has no destination choice."""
            # every asked tooth laid on its own legs or left as it stood -- never laid another way
            # (source_realize exact_only): the board is then one of the plan's teeth and the teeth it had
            res_r = sr.realize(board, moves, st['src_pad'], st['byname'],
                               st['sref'], new_board, guard_names=names,
                               exact_only=True)
            realized.append(res_r)
            misses = [nm for nm, e in res_r['audit'].items() if not e['exact']]
            ban_moves(banned, moves, misses, names)
            line = (f'  round {r}: source residue move(s) realized: {sorted(moves)}'
                    + (f'; not laid as asked (banned): {misses}' if misses else '')
                    + (f'; REJECTED ({res_r["rejected"]})' if res_r['rejected'] else ''))
            # a miss the engine left as it stood is no tooth nobody chose: the board keeps it, its move
            # banned above; one laid some other way (a ladder the realize did not cap) still throws the board out
            restored_ = set(res_r.get('restored') or ())
            if restored_ & set(misses):
                line += f'; left as they stood: {sorted(restored_ & set(misses))}'
            _trial.missed = [nm for nm in misses if nm not in restored_]
            if _trial.missed and res_r['rejected']:
                # the trial is returned unjudged below, so the moves its DRC names are banned here too: a
                # rejected trial's moves were never banned, and the next round asked them again
                hit = sorted({n for ln in res_r.get('pairs') or ()
                              for n in (drc_line_nets(ln, net_keys(st['byname'])) or ())} & set(moves))
                if hit:
                    ban_moves(banned, moves, hit, names)
            if _trial.missed:
                # a board whose teeth are not the plan's is NOT kept: what the engine laid in place of an
                # asked move is a tooth nobody chose (K35: SA12's dogbone to B laid as a surface tooth on F,
                # squeezed 0.20 from SDQ0's, and the board kept for judging better than the one before)
                return None, line
            if res_r['rejected']:
                # the moves of the nets its DRC pairs name (all of them when no line names one) -- banned, and the
                # plan chosen again without them (the loop below), as for a move not laid as asked
                hit = sorted({n for ln in res_r.get('pairs') or ()
                              for n in (drc_line_nets(ln, net_keys(st['byname'])) or ())} & set(moves))
                ban_moves(banned, moves, hit or list(moves), names)
                _trial.rejected = True
                return None, line
            # a board whose teeth wall in a ball no bus net is on is not kept: the moves walling one
            # banned, as a refusal is, and the plan chosen again without them
            import route_bus
            full_ = {n: st['byname'][n][1].name for n in names if n in st['byname']}
            walls_ = route_bus.walls_made(board, new_board, [st['sref']], list(full_.values()),
                                          [full_[n] for n in moves if n in full_])
            short_ = {v: k for k, v in full_.items()}
            by_ = {k: sorted(short_[x] for x in v if short_.get(x) in moves) for k, v in walls_.items()}
            wall_nets = sorted({n for v in by_.values() for n in v})
            if walls_:
                line += '; WALLS IN ' + ', '.join(f'{k} ({"/".join(v) or "none of its moves alone"})'
                                                  for k, v in by_.items())
            if wall_nets:
                # banned, and the REST of the moves realized without them at once -- their teeth left as
                # they stood -- rather than the ends searched again for each: a run along a channel walls the
                # row beside it with the copper across the row, and one class banned a search was another class
                # of the same net's the next (three searches, eleven balls)
                ban_moves(banned, moves, wall_nets, names)
                sub_ = {nm: m for nm, m in moves.items() if nm not in wall_nets}
                depth_ = getattr(_trial, 'depth', 0)
                if sub_ and depth_ < WALL_RETRIES:
                    _trial.depth = depth_ + 1
                    try:
                        res_w, line_w = _trial(sub_, new_board.replace('.kicad_pcb', 'w.kicad_pcb'))
                    finally:
                        _trial.depth = depth_
                    return res_w, line + ' -- banned; the rest realized without them:\n' + line_w
                _trial.missed = sorted(set(_trial.missed) | set(wall_nets))
                return None, line + ' -- banned'
            st2 = plan_state(parse_kicad_pcb(new_board), names, banned)
            src2 = {}
            # the ends model searches every berth again, from the previous choice
            ch2, un2 = dest_choice(st2, new_board, src_out=src2, seed=dst_choice)
            if not ch2:
                return None, line + '; no destination choice on the new board -- reverted'
            import whole_ends
            # (and what the teeth the new board's own choice asks would make of it: _ahead)
            _trial.free = whole_ends.choose.last.get('free') if src2 else None
            return (new_board, st2, ch2, un2, pf_key(ch2, {}, whole_ends.choose.last['laid']), src2,
                    split_pairs(st2)), line
        for _k in range(SRC_RESIDUE_ROUNDS):
            # the moves IN THE PLAN'S ORDER: it is the order the engine
            # receives its hints in, so a rebuilt dict gives the engine a
            # different call and different copper (measured at K35: 69
            # segments different, judged 198 against 181)
            rest = dict(src_out)
            _trial.rejected = False
            res, line = _trial(rest, f'{work}_srcres{r}_{_k}.kicad_pcb')
            if res is None and (getattr(_trial, 'missed', None) or _trial.rejected):
                # the engine did not lay every move as asked, or its board broke the rules (its moves the DRC
                # names banned): the board stays as it was, and the plan is chosen again on it without them. A
                # rejected board ended the round, and the teeth as they stood were kept: the joint ends' teeth,
                # one walls retry's restored tooth clashing with its new neighbours, never asked again
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
            # (fewer pairs split wins, then the judge's count)
            if sp2[0] < best_split[0] or (sp2[0] == best_split[0] and pf_better(res[4], best_key)):
                print(line + f'; {pf_fmt(best_key, res[4])}'
                      + (f', pairs split {best_split[0]} -> {sp2[0]}' if sp2[0] != best_split[0] else '')
                      + ': KEPT')
                board, st, dst_choice, un, best_key, src_out = res[0], res[1], res[2], res[3], res[4], res[5]
                best_split = sp2
            else:
                # THE TEETH AHEAD: a board judged worse on its teeth as laid whose own choice asks teeth that
                # make it better than the board kept is a step, not the end -- those teeth realized too, once, from
                # it, and the board they make judged against the one kept. The zynq DDR3 bus with its reset's pull
                # laid: the moves realized met two berths in conflict as laid (one split pair before), the 13 teeth
                # its choice then asked 74 vias and none -- reverted, the round laid the split pair's ends, 326
                # crossings, 104 vias where the bus without the reset lays 64
                # The steps go on while each is better as laid than the one before (a conflict fewer: 200306 ->
                # 100299 -> ...), TEETH_AHEAD at most
                free_ = getattr(_trial, 'free', None)
                ahead = (res[5] and free_ is not None and sp2[0] <= best_split[0]
                         and free_ < best_key[0])
                steps_, won_ = [], None
                if ahead:
                    kept_ = (board, st, dst_choice)
                    cur_ = res
                    for a_ in range(TEETH_AHEAD):
                        board, st, dst_choice = cur_[0], cur_[1], cur_[2]
                        nxt_, line_n = _trial(dict(cur_[5]), f'{work}_srcres{r}_{_k}a{a_}.kicad_pcb')
                        if nxt_ is None:
                            steps_.append('not laid')
                            break
                        steps_.append(pf_fmt(best_key, nxt_[4]).replace('judged ', ''))
                        if nxt_[6][0] < best_split[0] or (nxt_[6][0] == best_split[0]
                                                           and pf_better(nxt_[4], best_key)):
                            won_ = (nxt_, line_n)
                            break
                        nfree_ = getattr(_trial, 'free', None)
                        if not (nxt_[5] and nfree_ is not None and nfree_ < best_key[0]
                                and nxt_[6][0] <= cur_[6][0] and pf_better(nxt_[4], cur_[4])):
                            break
                        cur_ = nxt_
                    if won_ is not None:
                        res2, line2 = won_
                        print(line + f'; {pf_fmt(best_key, res[4])}: worse as laid, its own teeth ahead '
                              f'{free_:.0f} -- realized too, {len(steps_)} step(s): {"; ".join(steps_)}')
                        print(line2 + f'; {pf_fmt(best_key, res2[4])}'
                              + (f', pairs split {best_split[0]} -> {res2[6][0]}' if res2[6][0] != best_split[0]
                                 else '') + ': KEPT')
                        board, st, dst_choice, un, best_key, src_out = (res2[0], res2[1], res2[2], res2[3],
                                                                        res2[4], res2[5])
                        best_split = res2[6]
                        if not src_out:
                            break
                        continue
                    board, st, dst_choice = kept_
                print(line + f'; {pf_fmt(best_key, res[4])}'
                      + (f', pairs split {best_split[0]} -> {sp2[0]}' if sp2[0] != best_split[0] else '')
                      + (f', its own teeth ahead {free_:.0f} realized: {"; ".join(steps_)}' if ahead else '')
                      + ': not better -- reverted')
                # (nothing banned: the engine laid them as asked -- a ban is the engine's refusal, not the judge's)
                break
            if not src_out:
                break
        # the berths for the board KEPT, against its teeth as laid (a choice made with tooth moves that
        # were then not kept is not this board's) -- unless the last choice was made on this very board and
        # asked no tooth move: that choice is the one
        _L = getattr(whole_ends.choose, 'last', {})
        if not (_L.get('st') is st and not _L.get('moved')):
            dst_choice, un = dest_choice(st, board, seed=dst_choice)
    pb = planned_buses(st, dst_choice)
    # (the fast proxy is the braid's two-page planner's: on more routing layers it has none to give)
    f_fast = total(dst_choice, st, cache, pb) if len(route_layers.layers()) == 2 else float('nan')
    f, _pred, bp, _plan = judge_by_braid(st, dst_choice, board)
    n_corr = len({v['corridor'] for v in bp.values()})
    line = (f'  round {r}: destination vs the teeth ON {os.path.basename(board)}: '
            f'braid-judged {f:.2f} (fast proxy {f_fast:.2f}), {len(dst_choice)} placed'
            + (f', {len(un)} unplaced' if un else ''))
    line += '   <- best'
    line += f'  ({n_corr} corridor(s) by the braid\'s planner)'
    print(line)
    print(f'  kept round {r}: floor {f:.2f} on {os.path.basename(board)}')
    return dst_choice, st['dst_pad'], st['dref'], st['byname'], board, realized, banned


def explain_plan(choice, st, names, out_path=None, board=None, achieved=None):
    """The plan that ships, as the ends model reads it (judge_by_braid), and
    with `out_path` written beside the fanout board as `<board>.plan.json`:
    the braid reads it and builds its corridors from the identical inputs."""
    import json
    cost, pred, bp, plan = judge_by_braid(st, choice, board, achieved)
    print('  planner (the braid\'s own, on the plan\'s ends) per net: corridor, '
          'launch/target index, page, tooth layer+vias, berth escape, predicted vias')
    import whole_ends
    print(f'  plan model (the whole route\'s ends): {whole_ends._fmt(judge_by_braid.ends)}')
    if out_path:
        side = os.path.splitext(out_path)[0] + '.plan.json'
        plan['pages_first'] = True          # the braid stage pages this plan EXACTLY
        plan['nets'] = list(names)          # the run, in its order: an INCREMENTAL round routes the same nets
        if isinstance(st, dict) and len(st.get('dests') or ()) > 1:
            # (DEST naming several parts, one source and several ends: each net's own -- the feedback's berth boxes,
            # whole_feedback)
            plan['dest_of'] = {nm: st['dst_pad'][nm].component_ref for nm in plan['ends'] if nm in st['dst_pad']}
        # the ends model's reading of each lane on these ends (whole_ends: its nets over two, its load on the
        # trunk, its crossings) -- the lanes a round the whole route lays nothing on frees next (whole_feedback
        # --name) -- and the far face's cut it read them with: the solve's frame takes that one (braid.setup ->
        # whole_frame), where recomputed from the laid stubs, a hair off the menu's exits, two near-equal gaps could
        # split the face the other way and hand the solve crossings the model never priced
        pe_ = judge_by_braid.ends
        plan['ends_model'] = {k: pe_.get('lane_' + k, {}) for k in ('over', 'load', 'x', 'front')}
        if pe_.get('cut') is not None:
            c_ = pe_['cut']
            # (a far-face cut its y, as it always was; one off the far face -- winding -- its face and coordinate;
            # a GROUP's ('P', x, y), the point on its walk)
            plan['dest_cut'] = [c_[0]] + [float(v) for v in c_[1:]] if isinstance(c_, (list, tuple)) else float(c_)
        if pe_.get('entry'):
            # (a GROUP's ways in: each lane's place on its walk, whole_frame.Group -- the frame takes the way nearest)
            plan['dest_entry'] = {l_: [float(v) for v in p_] for l_, p_ in pe_['entry'].items()}
        if pe_.get('scut') is not None:
            plan['src_cut'] = float(pe_['scut'])       # (the source's far face's: a far-face tooth's side round it)
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
    if awx_settings.get('GROUP_ENDS'):
        # THE GROUP'S ENDS (a bus step on a group's field, group_ends.py): no fanout and no end chosen -- the board is
        # the round's as it stands, its ends and their plan the group's model's (group_ends.bus_stage)
        import group_ends
        return group_ends.bus_stage(out_path, base)
    # the run's nets: coherent on the base -- or, an INCREMENTAL round, the previous round's (its base is that round's
    # source board, on which the coherent K can be other nets: K35's lost its three pairs to six others)
    prev_nets = (json.load(open(awx_settings.req('INCREMENTAL'))).get('nets')
                 if awx_settings.get('INCREMENTAL') else None)
    if awx_settings.get('INCREMENTAL') and not prev_nets:
        print(f'WARNING: INCREMENTAL={awx_settings.req("INCREMENTAL")} records no nets: the run is the coherent {K} on '
              f'{os.path.basename(base)}, which can be other nets than the previous round\'s')
    if bool(awx_settings.get('INCREMENTAL')) != bool(awx_settings.get('FEEDBACK')):
        print('WARNING: an incremental round needs both INCREMENTAL and FEEDBACK: this round chooses every end afresh')
    # (FANOUT_NETS: the run's nets as the chain fixed them, whole_route -- never re-picked off a later round's board)
    run_nets = [n for n in (awx_settings.get('FANOUT_NETS') or '').split(',') if n]
    names = prev_nets or run_nets or coherent_nets(K, base)
    print('planning (source realized every round)...')
    work = out_path[:-len('.kicad_pcb')] if out_path.endswith('.kicad_pcb') else out_path
    choice, dst_pad, dref, byname, board, realized, banned = plan(base, names, work)
    rc = fanout_destination(out_path, names, choice, dst_pad, dref, byname, board, realized, banned)
    return _after_destination(out_path, names, dref, rc)


def _after_destination(out_path, names, dref, rc):
    """the fanout's last step after the destination's: the audit"""
    # THE FANOUT AUDIT on the board handed back: a fanout NEVER hands back one the whole route would refuse -- a pair
    # split the destination's re-plans could not clear (a tooth's, or a berth's with nothing left to ban), or anything
    # else its frame stops on. Such a board is set aside, named, and no board goes to the stage after
    sp, far, stops = fanout_audit(out_path, names, dref) if os.path.isfile(out_path) else ([], [], [])
    if sp or far or stops:
        held = set_aside_refused(out_path, sp, far, stops)
        print(f'REFUSED by the fanout audit: {_audit_line(sp, far, stops)} -- the board set aside as '
              f'{os.path.basename(held)}')
        return 3
    print('fanout audit: clean')
    return rc


PAIR_EXIT_REACH = 1.2


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


def tie_stand_ins(pcb, names, byname, dst_pad):
    """the TIE VIAS' sites (tie_vias_under: a ball with a pad of its own net under it is tied by a via at the ball once
    its berth is laid) as stand-in vias, each its net's, for a plan of the destination's escapes (group_ends: the
    field's menus measured round them) -- the other nets' escapes go round each as round any via.
    Unknown to the plans, the zynq7020's DDR_CK_P escaped by a via in its pad and a run along its ball row across
    DDR_CK_N's ball, the tie via was laid on that run, and the bus handed on a short"""
    from kicad_parser import Via
    with contextlib.redirect_stdout(io.StringIO()):
        sites = tie_vias_under(pcb, list(names), byname, dst_pad, [], log=lambda *a_: None)
    return [Via(x=t['x'], y=t['y'], size=t['size'], drill=t['drill'], layers=['F.Cu', 'B.Cu'], net_id=t['net_id'],
                locked=True) for t in sites]


def tie_site(ball, net):
    """(x, y): where the TIE VIA of destination ball `ball` stands (tie_vias_under) -- at the ball, nudged toward the
    first pad of its `net` under it within the ball's own slack, so the barrel's far annulus lies in the pad's copper;
    None where no pad of its net lies under it"""
    import pairs as _pairs
    if ball is None:
        return None
    under = [q for q in net.pads if _pairs.under_pad(ball, q, te.VIA_SIZE)]
    if not under:
        return None
    q = under[0]
    r_ball = min(ball.size_x, ball.size_y) / 2
    bx, by = ball.global_x, ball.global_y
    slack = max(r_ball - te.VIA_SIZE / 2, 0.0)
    dx, dy = q.global_x - bx, q.global_y - by
    d = math.hypot(dx, dy)
    if d <= 1e-9:
        return bx, by
    m = min(slack, d)
    return bx + dx / d * m, by + dy / d * m


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
        x, y = tie_site(ball, net)
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
                ex, ey = _pairs.into_pad(x - p.global_x, y - p.global_y, _pairs.pad_tilt(p))
                ex, ey = max(abs(ex) - p.size_x / 2, 0.0), max(abs(ey) - p.size_y / 2, 0.0)
                if math.hypot(ex, ey) < need - 1e-6:
                    bad = f'pad {fp.reference}.{p.pad_number}'
                    break
            if bad:
                break
        if not bad:
            # (a track of another net, on any layer: the via is a barrel on every one -- unchecked, the zynq7020's
            # DDR_CK_N tie was laid on DDR_CK_P's escape run along its ball row, a short the bus handed on)
            for sg in pcb.segments:
                if sg.net_id != nid and getattr(sg, 'graphic', False) is False and \
                        _pairs._pt_seg((x, y), (sg.start_x, sg.start_y), (sg.end_x, sg.end_y)) \
                        < need + sg.width / 2 - 1e-6:
                    bad = f'a track of {pcb.nets[sg.net_id].name if sg.net_id in pcb.nets else sg.net_id}'
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


def set_aside_refused(out_path, splits, far, stops):
    """A fanout board the audit refuses, set aside: OUT.kicad_pcb becomes OUT.refused.kicad_pcb, with what a board
    carries beside it -- the whole route's last resort lays this board:
    - its project and rules (#441: a board with no project beside it is laid under a stock one, which the step's OUT
      then carries -- the board's DRC exclusions, rule severities and edge clearance lost);
    - its plan sidecar (OUT.plan.json, and the taut cache): the solve reads the ends model's fronts from it, the braid
      its corridors, and on more than two layers the via ends are moved onto the solve's layers by it -- without it
      they stay as laid, on layers the plan does not route them on, and the band meets no tooth (TOOTH-OUT). Copied,
      not moved: the last resort reads the round's own OUT.plan.json too;
    and the findings beside it in OUT.refused.json, for the chain's feedback (whole_feedback --refused frees the ends
    named). Returns the set-aside board's path"""
    from copy_board import copy_siblings
    base = out_path[:-len('.kicad_pcb')]
    held = base + '.refused.kicad_pcb'
    os.replace(out_path, held)
    copy_siblings(out_path, held)
    for ext in ('.plan.json', '.taut.json'):
        if os.path.isfile(base + ext):
            shutil.copy2(base + ext, base + '.refused' + ext)
    with open(base + '.refused.json', 'w', encoding='utf-8') as f_:
        json.dump({'splits': [[pr_, end_, list(ls_)] for pr_, end_, ls_ in splits], 'far': list(far),
                   'stops': stops}, f_, indent=1)
    return held


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
    _d = awx_settings.get('DEST') or ''
    if len(te.dest_refs(_d)) > 1 and te.dest_refs(_d)[0] == dest:
        dest = _d                       # (DEST naming several parts, `dest` the first: the frame of the GROUP)
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


def _berth_walls(board, out_path, names, choice, dst_pad, byname):
    """(SRC_WALLS) {'REF.PAD': [net]}: the balls of the destination parts on no bus net that the pass's berths
    (`out_path` against `board`, the board it laid them on) wall in -- route_bus.walls_made, the step's own cage test --
    each with the run's nets of `choice` that wall it, by their short names"""
    import route_bus
    full_ = {n: byname[n][1].name for n in names if n in byname}
    refs = sorted({dst_pad[n].component_ref for n in names if dst_pad.get(n) is not None})
    walls_ = route_bus.walls_made(board, out_path, refs, list(full_.values()),
                                  [full_[n] for n in choice if n in full_])
    short_ = {v: k for k, v in full_.items()}
    return {k: sorted(short_[x] for x in v if short_.get(x) in choice) for k, v in walls_.items()}


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
    best_pass = None       # the best pass laid: (key, laid_pass, pass)
    learned = set()        # berth pairs the planner must avoid together: laid exactly, in violation of each other
    for it in range(DST_ITERS):
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
        # every pass graded -- its berths not as asked or in a DRC violation, then the ends model's objective on
        # what it asked -- and the best one's board kept beside: it ships, not merely the last (a pass's re-plan
        # can ask worse ends than the pass before, and the passes can run out with a better one unlaid)
        try:
            _v = judge_by_braid(st, choice, board)[0]
        except Exception:
            _v = float('inf')
        # ...the fanout audit FIRST: a pass whose board it refuses (a pair split, a tooth on the far face) is no
        # board to hand on, whatever it laid -- a negotiated escape bus's pass 5 shipped as the best laid, 3 not as
        # asked, with a pair split at its berths, and the audit refused it: the round's last resorts, 26 open
        aud_ = fanout_audit(out_path, names, dref)
        _k = (bool(aud_[0] or aud_[1] or aud_[2]), len(misses) + len(drc_nets), _v)
        if best_pass is None or _k < best_pass[0]:
            for ext_ in ('.kicad_pcb', '.kicad_pro'):
                src_ = out_path[:-len('.kicad_pcb')] + ext_
                if os.path.isfile(src_):
                    shutil.copyfile(src_, out_path[:-len('.kicad_pcb')] + '.bestpass' + ext_)
            best_pass = (_k, laid_pass, it)
        print(f'  destination pass {it}: graded {_k[1]} not as asked, objective {_k[2]:.2f}'
              + (', the fanout audit refuses it' if _k[0] else '')
              + (' (the best so far)' if best_pass[2] == it else ''))
        pair_only = []
        if drc_nets:
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
            sp, far, stops = aud_
            legs_ = _split_legs(names)
            bad = sorted({l_ for _pr, end_, lanes_ in sp if end_ == 'berth' for ln in lanes_
                          for l_ in legs_.get(ln, (ln,)) if l_ in choice})
            walled_ = _berth_walls(board, out_path, names, choice, dst_pad, byname) \
                if not sp and not far and not stops and SRC_WALLS else {}
            if not sp and not far and not stops and not walled_:
                print(f'  destination pass {it}: every berth laid as planned')
                break
            if walled_:
                wall_nets = sorted({n_ for v_ in walled_.values() for n_ in v_})
                print(f'  destination pass {it}: every berth laid as planned, but they WALL IN '
                      + ', '.join(f'{k_} ({"/".join(v_) or "none of its berths alone"})' for k_, v_ in walled_.items())
                      + (' -> re-planned' if wall_nets else ''))
                if not wall_nets:
                    break
                misses = wall_nets             # (banned and re-planned below, as a berth not laid as asked)
            else:
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
        # the berths laid exactly stay FIXED, only the missed nets are
        # re-planned -- a re-plan from scratch asked for 5-6 new berths
        # every pass and never converged (K28, 8 passes, 27 bans). The
        # re-plan starts from this pass's choice, and a lane it still finds
        # in a CONFLICT has its held neighbours freed with it, rather than a
        # known conflict laid: a lane freed alone with no clean option kept
        # its old berth beside a held one
        new_choice, un = dest_choice(st, board, fixed=laid_ok, learned=learned, seed=dict(choice))
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
    # The BEST pass ships (graded above), and its sidecar is its own: the
    # choice that was fanned out and audited, never the re-plan after it
    # (which is a choice no board was laid to). Judged on the WRITTEN
    # fanout board: the same copper the braid will read.
    if best_pass is not None and best_pass[1] is not laid_pass:
        # the best pass's board back in place of the last's
        for ext_ in ('.kicad_pcb', '.kicad_pro'):
            src_ = out_path[:-len('.kicad_pcb')] + '.bestpass' + ext_
            if os.path.isfile(src_):
                shutil.copyfile(src_, out_path[:-len('.kicad_pcb')] + ext_)
        print(f'  destination: pass {best_pass[2]} ships (the best laid, {best_pass[0][1]} not as asked, objective '
              f'{best_pass[0][2]:.2f}' + (', the fanout audit refusing every pass' if best_pass[0][0] else '')
              + '), not the last')
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
            _drop_tie_siblings(tmp)
            ship_vias.stamp(out_path, 'fanout ties', print)
    except Exception as e:      # a tie must not lose the board
        print(f'  tie vias NOT added: {e}')
    return 0 if ok_l else 1


def row_berths(pcb, dref, targets, choice, dst_pad, byname):
    """(tracks, vias, removed vias, failed nets) for the berths at a ROW PART (escape_moves.row_part): each target's
    planned move laid on its own legs at the fanout's track; a target the plan left unplaced takes its pad's one
    escape, straight out of its row (escape_moves), where its row's face is clear of everything but the run's nets"""
    grid = em.grid_of(pcb.footprints[dref])
    kids = {byname[nm][0] for nm in targets}
    tracks, vias, failed = [], [], []
    for nm in targets:
        nid = byname[nm][0]
        m = choice.get(nm)
        if m is None and nm in dst_pad:
            own = em.enumerate_moves(dst_pad[nm], grid, route_layers.escape_layers(pcb.board_info.copper_layers),
                                     lambda p, q, L, _n=nid: te.build_obstacles(pcb, _n, kids, L).seg_clear(p, q),
                                     net=nm)
            m = own[0] if own else None
        if m is None:
            failed.append(nm)
            continue
        tracks += [{'start': tuple(a), 'end': tuple(b), 'width': sr.FAN_TRACK, 'layer': L, 'net_id': nid}
                   for a, b, L in m.legs]
        if m.site is not None:
            vias.append({'x': m.site[0], 'y': m.site[1], 'size': te.VIA_SIZE, 'drill': te.VIA_DRILL,
                         'layers': ['F.Cu', 'B.Cu'], 'net_id': nid})
    return tracks, vias, [], failed


def fanout_once(out_path, names, choice, dst_pad, dref, byname, board):
    """One destination fanout to `choice`, written to out_path and audited:
    every net of the run is fanned out from `board` (the source board, its
    destination bare). Returns (laid nets, audit dict, clean-and-complete).
    (DEST naming several parts, one source and several ends: each net's
    berth laid at its own -- dst_pad's part -- by that part's engine.)"""
    multi = len(te.dest_refs(awx_settings.get('DEST') or '')) > 1
    dof = (lambda nm: dst_pad[nm].component_ref if nm in dst_pad else dref) if multi else (lambda nm: dref)
    targets = list(names)
    pcb = parse_kicad_pcb(board)
    src_file = board
    hints = {}
    for nm in targets:
        if nm in choice:
            p = dst_pad[nm]
            hints[(round(p.global_x, 3), round(p.global_y, 3))] = sr.full_move(choice[nm])
            # STRICT plan-follow (underpad._follow_plan): a negotiation
            # is kept only when the count of exact berths rises, and a
            # ball with no berth on its asked face is left unescaped
            # for the planner's re-plan, never dumped on another face
            hints[(round(p.global_x, 3), round(p.global_y, 3))]['strict'] = True
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
    # (the ends keep each plane ball a way down of its own -- whole_ends.plane_ways -- so the engine reserves no via in
    # every plane ball's pad, as at the source: source_realize)
    pcb._fanout_plane_ways = True
    tracks, vias_add, vias_rm, failed = [], [], [], []
    for r_ in [dref] + sorted({dof(nm) for nm in targets} - {dref}):
        tg_ = [nm for nm in targets if dof(nm) == r_]
        if not tg_ and r_ != dref:
            continue
        if em.row_part(pcb.footprints[r_]):
            # a ROW PART (a QFN): each berth laid as planned, on its own legs -- a pad's one escape, straight out of
            # its row (escape_moves); the array engine follows a plan over a ball grid, and on zynq U4 laid 1 of 8
            got_ = row_berths(pcb, r_, tg_, choice, dst_pad, byname)
        else:
            got_ = generate_bga_fanout(
                pcb.footprints[r_], pcb, net_filter=engine_filter(tg_, pcb),
                layers=route_layers.stacked(pcb.board_info.copper_layers),
                track_width=sr.FAN_TRACK, clearance=sr.FAN_CLEAR, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
                exit_margin=0.5, escape_method='underpad', plane_drop='off',
                escape_dir_hints=hints, net_clearances={})   # class-blind, as the plan (#1529)
        for acc_, add_ in zip((tracks, vias_add, vias_rm, failed), got_):
            acc_ += list(add_)
    # a PAIR's berths laid together or not at all, as source_realize's teeth: a leg the engine could not lay leaves its
    # partner's new berth out too -- laid alone it stood where the pair was asked, between another pair's legs on its
    # layer (an escape bus negotiated: DATA_CLK_P unlaid, DATA_CLK_N on the face between TX_FRAME's berths, and the
    # fanout audit refused the board). Both are then berths not laid as asked, banned and re-planned together
    nid_ = {nm: byname[nm][0] for nm in names if nm in byname}
    new_ = {t['net_id'] for t in tracks} | {v['net_id'] for v in vias_add}
    lone_ = []
    for a_, b_ in _split_legs(names).values():
        has_ = [nid_.get(x_) in new_ for x_ in (a_, b_)]
        if has_[0] != has_[1]:
            k_ = a_ if has_[0] else b_
            if k_ in targets and nid_.get(k_) in new_:
                lone_.append(k_)
    if lone_:
        ids_ = {nid_[k_] for k_ in lone_}
        tracks = [t for t in tracks if t['net_id'] not in ids_]
        vias_add = [v for v in vias_add if v['net_id'] not in ids_]
        vias_rm = [v for v in vias_rm if not isinstance(v, dict) or v.get('net_id') not in ids_]
        failed += lone_
        print(f'  a pair\'s berths together or not at all: {", ".join(lone_)} left out, its partner not laid')
    if tracks:
        add_tracks_and_vias_to_pcb(
            src_file, out_path, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb.nets.items()})
        ship_vias.stamp(out_path, 'fanout', print)
    else:
        shutil.copy(src_file, out_path)
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
                        '--nets'] + net_patterns(names, net_keys(pcb)),
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
    keys_ = net_keys(pcb)
    for a, b in _re.findall(r'^\s+(.+?) <-> (.+?)\s*$', _drc_txt, flags=_re.M):
        a, b = drc_side_net(a, keys_), drc_side_net(b, keys_)
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
    have = {nm for nm in names if byname[nm][0] in got}
    laid = [nm for nm in choice if nm in have]
    achieved = {nm: sr.measure_tooth(pcb_out, nm, dst_pad[nm], byname, dest_ref=dof(nm))
                for nm in laid}
    audit_d, _counts = sr.audit(choice, achieved, None, laid, print, 'berth')
    # the berth of a net the plan left UNPLACED, laid by the engine's own
    # choice: measured too, so the loop can keep it (the audit above is
    # over the asked berths only)
    for nm in names:
        if nm not in choice and nm in have and nm in dst_pad:
            achieved[nm] = sr.measure_tooth(pcb_out, nm, dst_pad[nm], byname, dest_ref=dof(nm))
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
