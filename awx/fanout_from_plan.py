#!/usr/bin/env python3
"""Fan out the berth array in the directions the PLAN chose.

The plan picks, per ball, an escape direction and a travel layer. Until
now it picked them for nobody -- the production fanout was never told,
and chose its own with `preferred_escape_dirs`, which guesses "toward
this net's nearest off-footprint pad" one net at a time, blind to the
other nets and to the corridor they share.

This hands the plan's directions to `generate_bga_fanout` through
escape_dir_hints and then CHECKS THE COPPER, because a hint accepted is
not a hint obeyed: the fanout still has to find a free channel, and
when it cannot it silently takes another direction (or drops the ball).
The direction of each emitted escape is measured from the track that
actually leaves the pad, and compared with what was asked for.

The negative control is the same run with the hints withheld
(--no-hints), which is what the production fanout does today.

usage: fanout_from_plan.py OUT.kicad_pcb [K] [--no-hints]
"""
import math
import os
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
import flow_frame as ff  # noqa: E402

# Face names are FLOW-FRAME names: 'left' is the face toward the source,
# 'down' the face on the side of the source's flank teeth, whatever the
# board's own axes say. They are mapped to board directions through the
# same flow angle the braid rotates by, so a rotated bench gets the same
# plan -- which the rotation gate (chain_rot.sh) checks.
_DIRV = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}


def board_dirs(flow_names, theta):
    """Flow-frame direction names -> board direction names, for a flow
    frame that rotates the board by `theta` degrees (flow_frame.rotate_pcb's
    convention: flow = R(theta) . board, so board = R(-theta) . flow)."""
    r = math.radians(-theta)
    c, s = math.cos(r), math.sin(r)
    out = set()
    for nm in flow_names:
        vx, vy = _DIRV[nm]
        bx, by = c * vx - s * vy, s * vx + c * vy
        out.add(min(_DIRV, key=lambda k: (_DIRV[k][0] - bx) ** 2
                    + (_DIRV[k][1] - by) ** 2))
    return out

out_path = sys.argv[1]
rest = [a for a in sys.argv[2:] if not a.startswith('-')]
K = rest[0] if rest else '21'
NO_HINTS = '--no-hints' in sys.argv
METHOD = next((a.split('=', 1)[1] for a in sys.argv
               if a.startswith('--escape-method=')), 'auto')
# Restrict the plan to one exit side. The braid delivers from the WEST
# splice line and has no mechanism to reach any other face, so a chain
# that means to hand off to it has to plan within that limit; without
# this the plan spreads over all four sides and most nets have no
# corridor at all. Not a fact about this board -- a fact about what the
# braid can currently do.
ONLY = next((set(a.split('=', 1)[1].split(',')) for a in sys.argv
             if a.startswith('--dirs=')), None)
# The plane-drop pass collides with the decoupling caps under the array
# on this bench -- 4 pad-via violations present with hints, without
# hints, and with no braid at all. It is a defect of that pass, not of
# anything here, so it gets a switch: with it off, what the DRC count
# reports is the escapes and the braid alone.
NO_DROP = '--no-plane-drop' in sys.argv
# Negative control for the exit-line hint specifically: directions
# still applied, gaps left to the fanout.
NO_LINES = '--no-lines' in sys.argv
# Apply the plan's SOURCE choices too: the bench's source array comes
# fanned out, and the plan's source refinement (plan_ends.refine_source)
# only ever chose better teeth on paper. With --source the chosen nets'
# source copper is stripped and re-fanned in the planned direction and
# KIND (surface -> channel escape on the pad's layer, dogbone -> dogbone,
# via-in-pad -> underpad), so a tooth arrives on the layer the corridor
# delivers on -- a B tooth the corridor must bring back to F costs the
# braid a via the plan already knew it did not want (K15 SRAS).
SOURCE = '--source' in sys.argv
# --order-model: the braid's own launch and exit rules
# (plan_order.BraidOrder) score the JOINERS' face choice by the
# schedule the braid will lay -- corridor vias (divers, exit-leg
# crossings, in-flight surfacings) and columns against the corridor's
# capacity -- after select() has placed everything by its projection.
# Opt-in: it moves SWE down at K21 and K28 as the human does (vias
# 34 -> 32, 75 -> 68) and the braid then refuses two exit-block lanes
# behind it each time (SA7/SA9, SA1/SA7) -- a braid defect to chase
# before this becomes the default. Off, the chain is bit-identical.
ORDER_MODEL = '--order-model' in sys.argv
# --two-page: STEP 3 of the two-page design -- escapes BY PAGE. After
# the plan, the braid's own orders give each net its page (schedule.
# Schedule under TWO_PAGE), and every page net's escape is re-picked to
# a menu move on its PAGE LAYER at both ends (same direction; the
# fanout then applies the chosen KIND per net, dogbones and via-in-pads
# first, like the source apply). A B-page net fanned to B at both ends
# costs the corridor nothing and takes its birth/landing copper off F
# -- the layer whose saturation refuses the ribbon's swimmers. Implies
# --order-model; pair with SOURCE=1 and TWO_PAGE=1 in the chain so the
# source is applied and the braid lays the ribbon.
TWO_PAGE_PLAN = '--two-page' in sys.argv
# --pages-json=FILE: the ROUND-TRIP page source. The plan's own
# transverse projection and the braid's real schedule disagree for a
# few nets per rung (the tooth-layer/ride mismatch: a B dogbone whose
# lane rides F pays 2 vias for nothing). With this flag the escape
# re-pick uses the BRAID'S OWN pages (a plan-only braid dump on a
# first fanout), and the sidecar is PINNED to the same map so the
# final braid agrees by construction.
PAGES_JSON = next((a.split('=', 1)[1] for a in sys.argv
                   if a.startswith('--pages-json=')), None)
if TWO_PAGE_PLAN:
    os.environ['TWO_PAGE'] = '1'
    # NOT forcing --order-model any more: the measured verdict (t4..t7)
    # is that the order model's face refinement moves flank joiners the
    # braid then refuses, and the plain plan + ribbon braid is the arm
    # to beat (K28 46/5). Without it the pages come from the plan's own
    # transverse projection -- the minimal escapes-by-page arm.
base = next((a.split('=', 1)[1] for a in sys.argv
             if a.startswith('--board=')),
            os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))

names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(base)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
cache = {}


def obs(nid, layer):
    if (nid, layer) not in cache:
        cache[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
    return cache[(nid, layer)]


LAYERS = ('F.Cu', 'B.Cu')
dmenu, launch, src_pad, dst_pad = {}, {}, {}, {}
for nm in names:
    nid, net = byname[nm]
    fp = pcb.footprints[ends[nm][2]]
    bx, by = ends[nm][1]
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)
    dst_pad[nm] = pad
    dmenu[nm] = em.enumerate_moves(
        pad, em.grid_of(fp), LAYERS,
        lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
        lambda p, L, _n=nid: not (obs(_n, L).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
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
    nid = byname[nm][0]
    smenu[nm] = em.enumerate_moves(
        p, sgrid, LAYERS,
        lambda a, b, L, _n=nid: obs(_n, L).seg_clear(a, b),
        lambda a, L, _n=nid: not (obs(_n, L).point_violation(
            a, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])

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
    direction = min(_DIRV, key=lambda k: (_DIRV[k][0] * math.hypot(*d) - d[0]) ** 2
                    + (_DIRV[k][1] * math.hypot(*d) - d[1]) ** 2)
    site = (near_vias[0].x, near_vias[0].y) if near_vias else None
    src_seed[nm] = em.Move(nm, 'dogbone' if site else 'surface', direction,
                           tooth0[nm], tooth, len(near_vias),
                           [((p.global_x, p.global_y), tooth, tooth0[nm])],
                           site=site)

print('planning...')
if ONLY:
    _theta = ff.flow_angle([ends[n][0] for n in names],
                           [ends[n][1] for n in names])
    _only_board = board_dirs(ONLY, _theta)
    if _theta:
        print(f'  flow frame {_theta:.0f} deg: faces {sorted(ONLY)} '
              f'(flow) -> {sorted(_only_board)} (board)')
    ONLY = _only_board
    dmenu = {n: [m for m in ms if m.direction in ONLY]
             for n, ms in dmenu.items()}
    _empty = [n for n, ms in dmenu.items() if not ms]
    if _empty:
        print(f'  {len(_empty)} net(s) have NO move on {",".join(ONLY)}: '
              + ','.join(_empty[:8]))
model = None
if ORDER_MODEL:
    import plan_order as po
    model = po.build_model(pcb, names, ends, byname, obs, paths, launch, tooth0,
                           dref, sref, dgrid.bbox)
    print(f'  order model: {len(model.joiners)} joiner(s) '
          f'{sorted(model.joiners)}, s0 {model.s0:.2f}')
schoice, choice, lp, report = pe.plan_ends(
    smenu, dmenu, launch, sgrid.bbox, dgrid.bbox, buses=buses,
    tooth_layer0=tooth0, src_seed=src_seed,
    # the 'spend' objective only when the source moves will be APPLIED
    # SRC_OBJECTIVE=floor|spend overrides the SOURCE-implied objective:
    # measured 0903 on a fresh seed, 'spend' alone (no apply) cost K28
    # 34 -> 48 and K35 84 -> 96, while the apply recovered 14 at K35
    objective=os.environ.get('SRC_OBJECTIVE') or ('spend' if SOURCE else 'floor'),
    model=model)
if TWO_PAGE_PLAN and choice:
    # STEP 3: pages from the braid's own orders for THIS choice, and
    # each page net's escape re-picked to its page layer at both ends
    # -- iterated to a FIXED POINT with the orders. A re-pick moves
    # the exit (and, with SOURCE, the tooth), which shifts the very
    # permutation the pages were computed from: computed once, K11's
    # SDQ12 was planned B-page and braided as a swimmer with both
    # ends on B, and K28's plan said F18/B7/sw3 where the braid then
    # found F17/B5/sw5.
    from schedule import Schedule
    import select_moves as sm_mod
    import itertools

    def _legs_cross(ma, mb):
        # PERPENDICULAR same-layer leg crossings: _conflict keys lanes
        # on shared gaps and sites, so a long re-picked B run across
        # the array crossing another move's B leg slipped through
        # (t8 K28: SA0's left exit over SDQM1/SRAS's down legs, 2 DRC)
        for (a1, b1, L1) in ma.legs:
            for (a2, b2, L2) in mb.legs:
                if L1 == L2 and sm_mod._proper_cross(a1, b1, a2, b2):
                    return True
        return False

    def _clashes(m, sel, me):
        return (not sm_mod.lanes_free(m, sel, me, strict=True)
                or any(_legs_cross(m, om) for o, om in sel.items()
                       if o != me))
    unresolved = set()
    repicked = set()          # berths actually moved by this block
    relay_cand = {}           # surgical scope: net -> ranked B moves
    split_targets = []        # split scope: nets whose berth goes B
    # ONE round, not a fixed point: iterating re-picks against the
    # re-shifted orders converged to WORSE pages (K28 sw3 -> sw5, K21
    # split into two corridors as de-conflict moved faces). Instead
    # the assignment is written BESIDE the board (below) and the braid
    # uses it verbatim, so plan and braid cannot disagree.
    for _round in range(1):
        grp = [n for n in names if n in choice]
        tooth_now = dict(tooth0)
        launch_now = dict(launch)
        for n, m in schoice.items():
            tooth_now[n] = m.layer
            launch_now[n] = m.exit_pt
        if model is not None:
            geo_now = (model if launch_now == model.launch
                       else model.rebased(launch_now))
            lo_ = geo_now.order(grp, choice)
            tgt_ = geo_now.target_order(grp, choice)
        else:
            # no order model: both orders from the plan's own
            # transverse projection (the base Corridor's axis), the
            # same geometry select() and the floor price with
            geo_now = pe.sm.Corridor(dgrid.bbox, launch_now, cache=cache)
            t_ = geo_now.axis(grp, choice)
            lo_ = sorted(grp, key=lambda n: geo_now.launch_key(n, t_))
            tgt_ = sorted(grp, key=lambda n: geo_now.exit_key(n, choice[n], t_))
        sched = Schedule(lo_, tgt_, tooth_now,
                         dest_layer={n: choice[n].layer for n in grp})
        if PAGES_JSON:
            import json as _json
            _pg = _json.load(open(PAGES_JSON))
            sched.page = {n: _pg.get(n) for n in grp}
            sched.b_page = {n for n in grp
                            if _pg.get(n) == 'B.Cu'}
            sched.swimmers = {n for n in grp if _pg.get(n) is None}
            print(f'  pages from {os.path.basename(PAGES_JSON)} '
                  '(braid round-trip), not the plan projection')
        n_f = sum(1 for n in grp if sched.page.get(n) == 'F.Cu')
        print(('\n' if not _round else '')
              + f'two-page plan round {_round}: pages F {n_f} / B '
              f'{len(sched.b_page)} / swimmers {len(sched.swimmers)}')
        re_d = re_s = 0
        # TP_RELAY_NETS (surgical scope): re-lay THESE nets' escapes
        # instead of the plan-predicted B page. The plan's orders are
        # not the braid's (measured: at K21 the plan's B set shares 3
        # of 6 members with the braid's own page B), so the honest
        # target set is the braid's round-1 derivation, grepped from
        # its log by the driver and passed here for round 2.
        _tp_env = os.environ.get('TP_RELAY_NETS')
        relay_targets = (set(_tp_env.split(',')) & set(grp) if _tp_env
                         else {n for n in grp
                               if sched.page.get(n) == 'B.Cu'})
        # TP_SCOPE=swim: re-pick ONLY the predicted SWIMMERS' escapes,
        # to the smaller page's layer, and leave every page net on the
        # plain plan. Rationale: no page POLICY completes K28 (lis/
        # wmax 49v/3 open SDQ0,SDQ15,SWE; worst 46v/4 open -- the
        # demoted risers), so the swimmers are a PHYSICAL wall: they
        # weave against the all-F escape saturation. Moving all ~20
        # page escapes (the default scope) costs vias and DRC (t10:
        # 69v/4 open/2 DRC vs the plain plan's 49/3/0); moving only
        # the ~5 swimmers removes F copper exactly where they weave.
        scope = os.environ.get('TP_SCOPE', 'pages')
        n_b = len(sched.b_page)
        minor = 'B.Cu' if n_f >= n_b else 'F.Cu'
        for n in grp:
            P = sched.page.get(n)
            if scope == 'swim':
                if P is not None:
                    continue
                P = minor
            elif scope == 'split':
                # PAGE-SPLIT FANOUT (built for K35, where the all-F
                # fanout SATURATES: channel drops 11 of 35 balls and
                # the under-pad rescue punts 10 "elsewhere", through
                # the decoupling caps -- 15 DRC). Only the B PAGE's
                # escapes are re-picked here (and laid verbatim,
                # dogbone preferred, before the engine runs); the F
                # page and the swimmers stay with the engine, which
                # then fans a load the faces can hold. The census of
                # the HUMAN's own DU1 fanout is the blueprint: 22 of
                # 35 nets travel B, the down face splits 10 B / 4 F,
                # dogbones in the gaps, NO via-in-pad. SWIMMERS
                # offload to B too (the human's ratio is 22 of 35
                # travelling B; the B page alone is 7). No menu move
                # is needed: the apply places the VIA verbatim and
                # ROUTES the B run (a straight-leg B move exists for
                # almost nobody -- the cap field is in the way, which
                # is exactly why the runs must be routed).
                if P is None or P == 'B.Cu':
                    split_targets.append(n)
                continue
            elif scope == 'surgical':
                # the target set may include nets the plan's own
                # Schedule called swimmers: with TP_RELAY_NETS the
                # braid's round-1 page B is the authority, not the
                # plan's projection
                if n not in relay_targets:
                    continue
                P = 'B.Cu'
            elif not P:
                continue
            m0 = choice[n]
            if scope == 'surgical':
                # ENGINE-FIRST SURGICAL RE-LAY. `choice` is left alone:
                # the engine lays EVERY net from the plain plan first
                # (the all-verbatim apply was the confound in every
                # escapes-by-page loss -- it starved the flank the
                # engine routes fine). Only the B-page nets' B-layer
                # menu moves are RECORDED here, ranked; the apply rips
                # each such net's engine escape and lays the first
                # candidate that clears the engine's REAL copper.
                if P == 'B.Cu':
                    # EVERY B-page net is a re-lay target -- also one
                    # whose plain move is already B-layer: the engine
                    # never honours a layer, so its real copper is F
                    # regardless of what the plan chose
                    cand = [m for m in dmenu.get(n, ()) if m.layer == P]
                    if cand:
                        relay_cand[n] = sorted(cand, key=lambda m: (
                            m.direction != m0.direction,
                            m.kind != 'dogbone', m.vias,
                            abs(m.exit_pt[0] - m0.exit_pt[0])
                            + abs(m.exit_pt[1] - m0.exit_pt[1])))
                    else:
                        print(f'    surgical: {n} is B-page but its menu '
                              f'holds NO B move '
                              f'({len(dmenu.get(n, ()))} move(s) total)')
            elif m0.layer != P:
                # lanes_free re-checked STRICTLY against the WHOLE
                # choice: the re-pick happens after plan_ends, and an
                # unchecked gap via landed in the street a kept
                # surface move uses (K11 SDQ11 under SDQ14, 6 DRC)
                # split: the F moves are the ENGINE's, not verbatim
                # copper, so a B re-pick need only clear the OTHER
                # re-picked moves (checked vs the whole choice, 5 of
                # K35's 7 B-page nets found no candidate at all)
                sel_chk = ({k: choice[k] for k in repicked}
                           if scope == 'split' else choice)
                cand = [m for m in dmenu.get(n, ())
                        if m.layer == P
                        and (m.direction == m0.direction
                             or scope in ('swim', 'split'))
                        and not _clashes(m, sel_chk, n)]
                if cand:
                    # dogbone preferred over via_in_pad: the moves are
                    # laid VERBATIM, and a gap via is a standard
                    # barrel where a via-in-pad needs the pad clamp
                    # and the IPC-4761 burden. Swim scope relaxes the
                    # direction to a PREFERENCE (t12: with it required,
                    # 0 of 7 swimmers had a B candidate at all).
                    choice[n] = min(cand, key=lambda m: (
                        m.direction != m0.direction,
                        m.kind != 'dogbone', m.vias,
                        abs(m.exit_pt[0] - m0.exit_pt[0])
                        + abs(m.exit_pt[1] - m0.exit_pt[1])))
                    repicked.add(n)
                    re_d += 1
            if SOURCE and n in smenu:
                cur = schoice.get(n) or src_seed.get(n)
                if cur is not None and cur.layer != P:
                    # checked against the UNMOVED seeds too: a re-pick
                    # that only saw the moved nets shared a gap with a
                    # stub the plan left alone
                    merged = {**src_seed, **schoice}
                    cand = [m for m in smenu[n]
                            if m.layer == P and m.direction == cur.direction
                            and not _clashes(m, merged, n)]
                    if cand:
                        schoice[n] = min(cand, key=lambda m: m.vias)
                        re_s += 1
        # DE-CONFLICT under the STRICT test: the moves are laid
        # VERBATIM, so what select() tolerated (it only ever handed
        # the fanout a direction; the engine re-assigned gaps) must
        # hold as real geometry. The loser of a conflicting pair moves
        # to a free same-layer move -- same direction preferred, any
        # allowed direction as the fallback (an unresolved conflict
        # ships as real DRC: K21 SCKE0's column run under SRAS/SODT0;
        # a moved face is absorbed by the next round's page recompute).
        n_fix = 0
        for _r in range(4):
            moved_any = False
            for a_, b_ in itertools.combinations(
                    [n for n in names if n in choice], 2):
                if scope == 'split' and (a_ not in repicked
                                         or b_ not in repicked):
                    # split: only verbatim-vs-verbatim pairs are ours
                    # to de-conflict; a B move vs an engine net's
                    # paper move is the engine's business
                    continue
                if scope in ('swim', 'surgical') \
                        and a_ not in repicked and b_ not in repicked:
                    # swim scope: un-re-picked nets go to the ENGINE,
                    # which resolves its own geometry -- de-conflicting
                    # them here verbatim-lays moves that did not need
                    # to move (t12: 0 re-picks, 6 'de-conflicted' nets
                    # laid verbatim, braid 15 open vs the plain 3)
                    continue
                if not (sm_mod._conflict(choice[a_], choice[b_], strict=True)
                        or _legs_cross(choice[a_], choice[b_])):
                    continue
                done = False
                for n2 in (b_, a_):
                    m0 = choice[n2]
                    cand = [m for m in dmenu.get(n2, ())
                            if m.layer == m0.layer and m is not m0
                            and not _clashes(m, choice, n2)]
                    if cand:
                        choice[n2] = min(cand, key=lambda m: (
                            m.direction != m0.direction,
                            m.kind != m0.kind, m.vias,
                            abs(m.exit_pt[0] - m0.exit_pt[0])
                            + abs(m.exit_pt[1] - m0.exit_pt[1])))
                        repicked.add(n2)
                        n_fix += 1
                        moved_any = done = True
                        break
                if not done:
                    unresolved.add((a_, b_))
            if not moved_any:
                break
        if re_d or re_s or n_fix:
            print(f'  round {_round}: re-picked {re_d} berth / {re_s} '
                  f'source escape(s), de-conflicted {n_fix}')
        else:
            break
    for a_, b_ in sorted(unresolved):
        print(f'  UNRESOLVED lane conflict: {a_} vs {b_}')
    import json
    pages_path = os.path.splitext(out_path)[0] + '.pages.json'
    if PAGES_JSON:
        # round-trip: PIN the braid's own pages as the sidecar, so
        # the final braid uses the exact map the escapes were re-
        # picked for. (Pinning the PLAN's pages was the measured t15
        # mistake; pinning the braid's own is the 44v-record recipe.)
        _pg = json.load(open(PAGES_JSON))
        with open(pages_path, 'w', encoding='utf-8') as f:
            json.dump({n: _pg.get(n) for n in names}, f, indent=1)
        print('  pages sidecar PINNED from the braid round-trip')
    elif scope in ('surgical', 'split'):
        # NO sidecar: the plan's orders are not the braid's (t15: 7 of
        # 25 planned page members crossed their own page under the
        # braid's real orders and were demoted -- 46v/5 vs plain 49/3).
        # The braid derives its own pages, and its ends weighting
        # (schedule.on) adopts whichever teeth actually arrive on B.
        if os.path.exists(pages_path):
            os.remove(pages_path)
        if scope == 'surgical':
            print(f'  surgical: {len(relay_cand)} B-page escape(s) to '
                  f're-lay after the engine: {sorted(relay_cand)} '
                  '(no pages sidecar)')
        else:
            print(f'  split: {len(split_targets)} berth(s) to fan out '
                  f'on B {sorted(split_targets)} (no pages sidecar)')
    else:
        with open(pages_path, 'w', encoding='utf-8') as f:
            json.dump({n: sched.page.get(n) for n in grp}, f)
        print(f'  pages -> {os.path.basename(pages_path)}')
# only the nets the plan actually MOVED are re-fanned: a move of the
# same kind, side, layer and gap as the stub already on the board IS
# that stub (the menu's exit x differs from the tooth's by the array
# margin, and identity let every such net be re-fanned for nothing)


def _same_as_seed(nm, m):
    s = src_seed.get(nm)
    if s is None or m is s:
        return m is s
    if (m.kind, m.direction, m.layer) != (s.kind, s.direction, s.layer):
        return False
    ax = 1 if m.direction in ('left', 'right') else 0
    if abs(m.exit_pt[ax] - s.exit_pt[ax]) > 0.16:
        return False
    if m.site is not None and s.site is not None and \
            math.hypot(m.site[0] - s.site[0], m.site[1] - s.site[1]) > 0.16:
        return False
    return True


schoice = {nm: m for nm, m in schoice.items() if not _same_as_seed(nm, m)}
for line in report:
    print(line)

hints, lines = {}, {}
for nm, m in choice.items():
    p = dst_pad[nm]
    k = (round(p.global_x, 3), round(p.global_y, 3))
    hints[k] = m.direction
    # the exit LINE: which row gap (left/right) or column gap (up/down)
    # the plan put this net in. The side alone leaves the fanout free to
    # pick the gap, and the gap is what the launch->exit permutation --
    # and so the via floor -- is actually made of.
    lines[k] = m.exit_pt[1] if m.direction in ('left', 'right') \
        else m.exit_pt[0]
# ASKS DUMP: what the plan asked per net at each array -- side, gap
# line, layer, page -- next to the fanout board ({stem}_asks.json).
# plan_search reads it to tell a deviation whose plan is identical to
# another's (a no-op) from a real alternative; audits grade the
# fanout's obedience against it.
try:
    import json as _json
    _pp = (globals().get('PLAN_PAGES_ROUNDS') or [{}])[-1]
    _asks = {}
    for nm, m in choice.items():
        _asks.setdefault(nm, {})[dref] = dict(
            side=m.direction, line=(m.exit_pt[1] if m.direction in ('left', 'right')
                                    else m.exit_pt[0]),
            layer=m.layer, kind=m.kind, page=_pp.get(nm))
    for nm, m in schoice.items():
        _asks.setdefault(nm, {})[sref] = dict(
            side=m.direction, line=(m.exit_pt[1] if m.direction in ('left', 'right')
                                    else m.exit_pt[0]),
            layer=m.layer, kind=m.kind, page=_pp.get(nm))
    _ap = os.path.splitext(out_path)[0] + '_asks.json'
    with open(_ap, 'w', encoding='utf-8') as _f:
        _json.dump(_asks, _f, indent=1)
    print(f'asks dumped: {_ap} ({len(_asks)} nets)')
except Exception as _e:
    print(f'asks dump skipped ({_e})')
print(f'\nplan: {len(hints)} berth escape directions '
      + ', '.join(f'{d}:{sum(1 for v in hints.values() if v == d)}'
                  for d in sorted(set(hints.values()))))
if os.environ.get('PLAN_DUMP_BUSES'):
    import select_moves as _sm2
    _sm2.dump_buses()
if os.environ.get('PLAN_ONLY') == '1':
    # the plan only (plan_search.py enumerates candidates from the
    # bus dump without paying for the fanout)
    sys.exit(0)

DIRS = {'right': (1, 0), 'left': (-1, 0), 'up': (0, -1), 'down': (0, 1)}


def obeyed(tracks, chosen, pads, label):
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


def pad_key(p):
    return (round(p.global_x, 3), round(p.global_y, 3))


def copy_pro(src_board, dst_board):
    import shutil
    pro = os.path.splitext(src_board)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, os.path.splitext(dst_board)[0] + '.kicad_pro')


board = base
if SOURCE and schoice and os.environ.get('SRC_APPLY') != 'none':
    # SRC_APPLY=none: the SOURCE objective ('spend') without the apply --
    # the A/B that separates the plan's source DECISIONS from the
    # objective switch that comes with SOURCE=1
    import statistics
    from collections import Counter
    src_nets = [nm for nm in schoice if nm in src_pad and src_pad[nm] is not None]
    src_ids = {byname[nm][0] for nm in src_nets}
    src_names_full = {byname[nm][1].name for nm in src_nets}
    # the re-fanout uses the bench's OWN source fanout geometry
    widths = [s.width for s in pcb.segments if s.net_id in src_ids]
    vsz = [(v.size, v.drill) for v in pcb.vias if v.net_id in src_ids]
    tw = statistics.median(widths) if widths else 0.1
    vs, vd = Counter(vsz).most_common(1)[0][0] if vsz else (0.25, 0.15)
    txt = open(base, encoding='utf-8').read()
    txt = te.strip_net_items(txt, 'segment', src_ids, src_names_full)
    txt = te.strip_net_items(txt, 'via', src_ids, src_names_full)
    stem = os.path.splitext(out_path)[0]
    board = stem + '_src0.kicad_pcb'
    with open(board, 'w', encoding='utf-8') as f:
        f.write(txt)
    copy_pro(base, board)
    kinds = Counter(schoice[nm].kind for nm in src_nets)
    print(f'\nsource: re-fanning {len(src_nets)} of {sref}\'s nets per the '
          f'plan ({", ".join(f"{k}:{v}" for k, v in sorted(kinds.items()))}), '
          f'track {tw} via {vs}/{vd}')
    for nm in src_nets:
        print(f'    {nm}: {src_seed[nm]} -> {schoice[nm]}')
    src_tracks = []
    if os.environ.get('SRC_APPLY', 'engine') == 'engine':
        # WHOLE-ARRAY ENGINE PASS (0903): the source apply the way the
        # dest side has always worked. The incremental form above/below
        # (strip the CHANGED nets, lay them against frozen neighbours)
        # was measured harmful on a sound seed (fresh-fanned U1: K28 34
        # -> 49v/11 drc, K35 84/1 -> 74/6 open/14 drc, K41 2 open ->
        # 4 open/10 drc) for a structural reason: the plan's obstacle
        # model EXCLUDES every planned net (assumed re-laid), so a
        # menu never sees an unmoved neighbour's real stub -- SCAS's
        # re-slot landed on SA7's stub, all 11 K28 violations. Here
        # EVERY planned net's source stub is stripped and the array is
        # fanned in ONE engine pass with the plan's direction, exit
        # line and layer as hints (changed nets from their choice,
        # unchanged from their seed), so legality comes from the
        # engine's own checks; a net the engine cannot deliver gets
        # its seed stub back (rip only what you re-lay). SRC_APPLY=
        # incremental keeps the old path for A/B.
        import re as _re
        src_all = [nm for nm in smenu if src_pad.get(nm) is not None
                   and src_pad[nm].component_ref == sref]
        choice_of = {nm: (schoice.get(nm) or src_seed.get(nm))
                     for nm in src_all}
        src_all = [nm for nm in src_all if choice_of[nm] is not None]
        fp_s = pcb.footprints[sref]
        _xs = [q.global_x for q in fp_s.pads]
        _ys = [q.global_y for q in fp_s.pads]
        _RW = (min(_xs) - 1.6, min(_ys) - 1.6, max(_xs) + 1.6, max(_ys) + 1.6)

        def _in_rw(x, y):
            return _RW[0] <= x <= _RW[2] and _RW[1] <= y <= _RW[3]

        def _strip_window(txt_, token, nid_, full_):
            out_, i_ = [], 0
            while True:
                j_ = txt_.find('(' + token, i_)
                if j_ < 0:
                    out_.append(txt_[i_:])
                    break
                k_, d_ = j_, 0
                while True:
                    ch = txt_[k_]
                    if ch == '(':
                        d_ += 1
                    elif ch == ')':
                        d_ -= 1
                        if d_ == 0:
                            break
                    k_ += 1
                blk = txt_[j_:k_ + 1]
                m1 = _re.search(r'\(net (\d+)\)', blk)
                m2 = _re.search(r'\(net "([^"]+)"\)', blk)
                hit = (m1 and int(m1.group(1)) == nid_) or \
                      (m2 and m2.group(1) == full_)
                pts = _re.findall(r'\((?:start|end|at) ([-\d.]+) ([-\d.]+)',
                                  blk)
                if hit and pts and all(_in_rw(float(x), float(y))
                                       for x, y in pts):
                    out_.append(txt_[i_:j_].rstrip(' \t'))
                    e_ = k_ + 1
                    if e_ < len(txt_) and txt_[e_] == '\n':
                        e_ += 1
                    i_ = e_
                else:
                    out_.append(txt_[i_:k_ + 1])
                    i_ = k_ + 1
            return ''.join(out_)
        txt = open(base, encoding='utf-8').read()
        for nm in src_all:
            nid_, net_ = byname[nm]
            for token in ('segment', 'via'):
                txt = _strip_window(txt, token, nid_, net_.name)
        board = stem + '_src0.kicad_pcb'
        with open(board, 'w', encoding='utf-8') as f:
            f.write(txt)
        copy_pro(base, board)
        pcb_s = parse_kicad_pcb(board)
        fp_s = pcb_s.footprints[sref]
        hints_s = {pad_key(src_pad[nm]): choice_of[nm].direction
                   for nm in src_all}
        lines_s = {pad_key(src_pad[nm]): (
            choice_of[nm].exit_pt[1]
            if choice_of[nm].direction in ('left', 'right')
            else choice_of[nm].exit_pt[0]) for nm in src_all}
        layers_s = {pad_key(src_pad[nm]): choice_of[nm].layer
                    for nm in src_all}
        n_changed = sum(1 for nm in src_all if nm in schoice)
        print(f'  engine pass: {len(src_all)} source net(s) in one '
              f'fanout ({n_changed} changed by the plan)')
        tr, va, vr, fl = generate_bga_fanout(
            fp_s, pcb_s, net_filter=src_all, layers=['F.Cu', 'B.Cu'],
            track_width=tw, clearance=0.1, via_size=vs, via_drill=vd,
            exit_margin=0.5, escape_method='auto', plane_drop='off',
            escape_dir_hints=hints_s, escape_line_hints=lines_s,
            escape_layer_hints=layers_s)
        nxt = stem + '_src_engine.kicad_pcb'
        if tr:
            add_tracks_and_vias_to_pcb(
                board, nxt, tr, va, vr,
                net_id_to_name={i: n.name for i, n in pcb_s.nets.items()})
        else:
            import shutil
            shutil.copy(board, nxt)
        copy_pro(board, nxt)
        fl = set(fl)
        if fl:
            # a net the engine could not deliver as ASKED gets a
            # hint-free engine escape on the new board first (legal by
            # construction against the copper just laid); only if that
            # fails does the seed's stub come back. Measured: a blind
            # seed restore put SBA0's old stub on SCAS's new one -- the
            # incremental path's collision class, 10 drc at K35.
            fl_short = [x.rsplit('/', 1)[-1] for x in fl]
            pcb_f = parse_kicad_pcb(nxt)
            tr2, va2, vr2, fl2 = generate_bga_fanout(
                pcb_f.footprints[sref], pcb_f, net_filter=fl_short,
                layers=['F.Cu', 'B.Cu'], track_width=tw, clearance=0.1,
                via_size=vs, via_drill=vd, exit_margin=0.5,
                escape_method='auto', plane_drop='off')
            if tr2:
                nxt2 = stem + '_src_free.kicad_pcb'
                add_tracks_and_vias_to_pcb(
                    nxt, nxt2, tr2, va2, vr2,
                    net_id_to_name={i: n.name for i, n in pcb_f.nets.items()})
                copy_pro(nxt, nxt2)
                nxt = nxt2
                src_tracks.extend(tr2)
            print(f'  engine fallback (hint-free) for {sorted(fl_short)}: '
                  f'{len(tr2)} tracks, {len(fl2)} still failed')
            import surgical as _sg
            for nm in sorted(set(x.rsplit('/', 1)[-1] for x in fl2)):
                if nm not in byname:
                    continue
                nxt2 = stem + f'_src_restore_{nm}.kicad_pcb'
                _sg.swap_stub(nxt, base, nm, nxt2)
                copy_pro(nxt, nxt2)
                nxt = nxt2
            fl = set(fl2)
            # VERIFY the fallback copper: the engine's hint-free leg can
            # still cross a neighbour's fresh stub inside the ball's own
            # keep-out lens (measured: SBA0's fallback across SDQ6, 20
            # drc). A fallback/restored net that check_drc pairs with
            # anything is stripped back to its bare ball -- an honest
            # open the braid's retry can close, never a shipped overlap.
            chk = [x.rsplit('/', 1)[-1] for x in fl_short]
            bad = [nm for nm in chk if nm in byname and _sg.drc_partners(nxt, nm)]
            if bad:
                pcb_b = parse_kicad_pcb(nxt)
                for nm in bad:
                    nid_b, net_b = byname[nm]
                    txt_b = open(nxt, encoding='utf-8').read()
                    for token in ('segment', 'via'):
                        txt_b = _strip_window(txt_b, token, nid_b, net_b.name)
                    nxt2 = stem + f'_src_unfanned_{nm}.kicad_pcb'
                    with open(nxt2, 'w', encoding='utf-8') as f:
                        f.write(txt_b)
                    copy_pro(nxt, nxt2)
                    nxt = nxt2
                print(f'  fallback copper collides for {bad}: left UNFANNED '
                      f'(open) rather than shipped overlapping')
                del pcb_b
        print(f'  engine -> {len(tr)} tracks, {len(va)} vias, '
              f'{len(fl)} failed (seed restored)'
              + (f': {", ".join(sorted(x.rsplit("/", 1)[-1] for x in fl))}'
                 if fl else ''))
        src_tracks.extend(tr)
        board = nxt
        obeyed(src_tracks, {nm: choice_of[nm] for nm in src_all}, src_pad,
               'source')
    else:
        src_tracks = []
        # dogbones and via-in-pads FIRST: their stubs are short and sit
        # against the pad, and the dogbone engine does not keep its 45-degree
        # stub clear of an earlier pass's surface escape running through the
        # same row gap (K15: 9 grazes SDQM0/SDQ15). The channel escapes go
        # last and route round whatever copper is there.
        for kind, method in (('via_in_pad', 'underpad'), ('dogbone', 'dogbone'),
                             ('surface', 'channel')):
            nets_k = [nm for nm in src_nets if schoice[nm].kind == kind]
            if not nets_k:
                continue
            pcb_k = parse_kicad_pcb(board)
            fp_k = pcb_k.footprints[sref]
            if kind != 'surface' or TWO_PAGE_PLAN:
                # DIRECT EMIT. The escape engines optimise for "escaped",
                # not "delivered on layer L": measured at K11 the underpad
                # passes escaped every ball ON F with 0 vias -- the
                # planned B tooth never existed and the braid opened
                # exactly those nets. A via/dogbone MOVE carries its own
                # geometry (legs + via site), clear()-checked against the
                # static copper and lanes_free-checked against the other
                # moves, so it is laid verbatim.
                tr, va, vr, fl = [], [], [], []
                for nm in nets_k:
                    m = schoice[nm]
                    nid = byname[nm][0]
                    for (a_, b_, L_) in m.legs:
                        tr.append({'start': a_, 'end': b_, 'width': tw,
                                   'layer': L_, 'net_id': nid})
                    if m.site is not None:
                        va.append({'x': m.site[0], 'y': m.site[1], 'size': vs,
                                   'drill': vd, 'layers': ['F.Cu', 'B.Cu'],
                                   'net_id': nid})
            else:
                hints_k = {pad_key(src_pad[nm]): schoice[nm].direction
                           for nm in nets_k}
                # the plan's exit LINE too: it checked that exact gap against
                # the static copper (a foreign via in the next gap grazed the
                # engine's own pick of gap -- K15 SDQ8 vs SDQS1N)
                lines_k = {pad_key(src_pad[nm]): (schoice[nm].exit_pt[1]
                                                  if schoice[nm].direction in ('left', 'right')
                                                  else schoice[nm].exit_pt[0])
                           for nm in nets_k}
                tr, va, vr, fl = generate_bga_fanout(
                    fp_k, pcb_k, net_filter=nets_k, layers=['F.Cu', 'B.Cu'],
                    track_width=tw, clearance=0.1, via_size=vs, via_drill=vd,
                    exit_margin=0.5, escape_method=method, plane_drop='off',
                    escape_dir_hints=hints_k, escape_line_hints=lines_k)
            nxt = f'{stem}_src_{kind}.kicad_pcb'
            if tr:
                add_tracks_and_vias_to_pcb(
                    board, nxt, tr, va, vr,
                    net_id_to_name={i: n.name for i, n in pcb_k.nets.items()})
            else:
                import shutil
                shutil.copy(board, nxt)
            copy_pro(board, nxt)
            print(f'  {kind} -> {method}: {len(nets_k)} net(s), {len(tr)} tracks, '
                  f'{len(va)} vias, {len(fl)} failed'
                  + (f' ({", ".join(sorted(fl)[:6])})' if fl else ''))
            src_tracks.extend(tr)
            board = nxt
        obeyed(src_tracks, {nm: schoice[nm] for nm in src_nets}, src_pad,
               'source')
    pcb = parse_kicad_pcb(board)

fp = pcb.footprints[dref]
print(f'\nfanning out {dref} ({len(fp.pads)} pads), method {METHOD}'
      + ('  WITHOUT hints (negative control)' if NO_HINTS else
         '  with the plan\'s directions'))
if TWO_PAGE_PLAN and not NO_HINTS \
        and os.environ.get('TP_SCOPE') == 'surgical':
    # ENGINE-FIRST SURGICAL RE-LAY. Pass 1 is the production fanout for
    # EVERY net, hints and lines from the PLAIN choice (the re-lay
    # candidates never touched `choice`), so the engine's world is the
    # plain arm's world, flank included. Pass 2 rips ONLY the B-page
    # nets' engine escapes and lays their planned B moves, each
    # candidate checked against the engine's REAL copper (own net
    # excluded -- its escape is what is being ripped) and against the
    # moves already re-laid this pass.
    #
    # VERDICT (2026-08-30, fresh chains, paired vs plain 49/3, 32/0,
    # 22/1 at K28/21/15): MEASURED OUT. Best form (verbatim emission +
    # braid-owned TP_RELAY_NETS + skip-engine-B + engine-exit ranking)
    # grades 55/1, 32/0, 27/0 -- the same completion the braid's own
    # LAST CALL pass reaches alone (52/1, 32/0, 24/0), at +3 vias.
    # The page decision cannot live in escape copper: moving an exit
    # shifts the very permutation the pages came from (sg6: SDQM1's
    # own re-lay demoted it). Kept opt-in for future study.
    import shutil
    stem2 = os.path.splitext(out_path)[0]
    relay_cand = globals().get('relay_cand', {})
    pcb_e = parse_kicad_pcb(board)
    fp_e = pcb_e.footprints[dref]
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        fp_e, pcb_e, net_filter=names, layers=['F.Cu', 'B.Cu'],
        track_width=0.1, clearance=0.1, via_size=0.45, via_drill=0.25,
        exit_margin=0.5, escape_method=METHOD,
        plane_drop=('off' if NO_DROP else 'auto'),
        escape_dir_hints=hints,
        escape_line_hints=(None if NO_LINES else lines) or None)
    failed = set(failed)
    eng = f'{stem2}_dst_engine.kicad_pcb'
    if tracks:
        add_tracks_and_vias_to_pcb(
            board, eng, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb_e.nets.items()})
    else:
        shutil.copy(board, eng)
    copy_pro(board, eng)
    print(f'  engine (all {len(names)} nets): {len(tracks)} tracks, '
          f'{len(vias_add)} vias, {len(failed)} failed')
    pcb_w = parse_kicad_pcb(eng)
    wobs = {}
    rip_now = set()

    def obs_w(nid, L):
        k = (nid, L, tuple(sorted(rip_now)))
        if k not in wobs:
            wobs[k] = te.build_obstacles(pcb_w, nid, set(rip_now), L)
        return wobs[k]

    laid = {}

    def _emit_legs(m):
        # TP_EMIT=whisker: emit only the DOGBONE UNIT (pad->site stub,
        # via, 0.35mm whisker) and leave the run through the array to
        # the braid's lane router. Measured WORSE than the full run in
        # BOTH target regimes (sg4 K28 38v/9 open, sg5 K21 23v/6 vs
        # plain 49/3, 32/0): a berth deep in the array is a structural
        # perturbation -- endpoints move, corridor clustering resplits
        # (sg4 K28 split SA8/SA6 off and stranded SA0), pages scramble
        # -- and the lanes cannot reach it. Default: the move's own
        # full geometry, whose defect is merely local (it walls the
        # flank when the TARGETS are wrong -- sg2).
        if os.environ.get('TP_EMIT', 'full') == 'full':
            return m.legs
        legs = [m.legs[0]] if m.kind == 'dogbone' else []
        sx, sy = m.site
        dx, dy = _DIRV[m.direction]
        legs.append(((sx, sy), (sx + 0.35 * dx, sy + 0.35 * dy), m.layer))
        return legs

    # the engine's REAL delivery per target: the far end of its emitted
    # copper and the layer there. A target the engine already delivers
    # on B keeps its engine escape (sg6 K21: SRAS's engine dogbone WAS
    # on B; ripping it for a planned via-in-pad broke SRAS and SCAS),
    # and candidates are ranked by closeness to the engine's exit so
    # the re-lay preserves the round-1 permutation the pages were
    # computed from (sg6 K28: SDQM1's moved exit demoted it to a
    # swimmer of its own page).
    eng_far, eng_lay = {}, {}
    for n in relay_cand:
        _nid = byname[n][0]
        p = dst_pad[n]
        pts = [(t['end'], t['layer']) for t in tracks if t['net_id'] == _nid]
        pts += [(t['start'], t['layer']) for t in tracks if t['net_id'] == _nid]
        if pts:
            (fx, fy), fl_ = max(
                pts, key=lambda q: (q[0][0] - p.global_x) ** 2
                + (q[0][1] - p.global_y) ** 2)
            eng_far[n] = (fx, fy)
            eng_lay[n] = fl_
    pend = []
    for x in names:
        if x not in relay_cand:
            continue
        if eng_lay.get(x) == 'B.Cu':
            print(f'    surgical: {x} already delivered on B by the '
                  'engine -- kept')
            continue
        pend.append(x)
    fails = []
    for _phase in (0, 1):
        # phase 0 checks against the WHOLE engine board; phase 1
        # retries the failures with the already-ripped nets' engine
        # segments excluded (phase 0 conservatively kept them as
        # obstacles, and a failed candidate may only have collided
        # with copper that is coming out anyway)
        rip_now = {byname[x][0] for x in laid}
        fails = []
        for n in pend:
            nid = byname[n][0]
            got, why = None, []
            ef = eng_far.get(n)
            cands = relay_cand[n] if ef is None else sorted(
                relay_cand[n], key=lambda m: (
                    m.kind != 'dogbone',
                    abs(m.exit_pt[0] - ef[0]) + abs(m.exit_pt[1] - ef[1])))
            for m in cands:
                # the emitted via is 0.45 (0.35 in-pad), wider than
                # the TRACK the obstacle margins were built for: pad
                # the check out to the real barrel radius, both layers
                pad_r = ((0.35 if m.kind == 'via_in_pad' else 0.45)
                         - te.TRACK) / 2
                if not all(obs_w(nid, L_).seg_clear(a_, b_)
                           for (a_, b_, L_) in _emit_legs(m)):
                    why.append('leg')
                    continue
                if m.site is not None and any(
                        obs_w(nid, L_).point_violation(m.site, pad=pad_r)
                        for L_ in LAYERS):
                    why.append('site')
                    continue
                if not sm_mod.lanes_free(m, laid, n, strict=True) or any(
                        _legs_cross(m, om) for om in laid.values()):
                    why.append('move')
                    continue
                got = m
                break
            if got is not None:
                laid[n] = got
            else:
                fails.append((n, why))
        pend = [n for n, _ in fails]
        if not pend or not laid:
            break
    for n, why in fails:
        print(f'    surgical: NO clear B move for {n} -- engine kept '
              f'(candidates refused: {",".join(why)})')
    if laid:
        rip = {byname[n][0] for n in laid}
        tracks = [t for t in tracks if t['net_id'] not in rip]
        vias_add = [v for v in vias_add if v['net_id'] not in rip]
        for n, m in laid.items():
            nid = byname[n][0]
            for (a_, b_, L_) in _emit_legs(m):
                tracks.append({'start': a_, 'end': b_, 'width': 0.127,
                               'layer': L_, 'net_id': nid})
            if m.site is not None:
                in_pad = m.kind == 'via_in_pad'
                vias_add.append({'x': m.site[0], 'y': m.site[1],
                                 'size': 0.35 if in_pad else 0.45,
                                 'drill': 0.2 if in_pad else 0.25,
                                 'layers': ['F.Cu', 'B.Cu'],
                                 'net_id': nid})
        print(f'  surgical re-lay: {len(laid)} of {len(relay_cand)} '
              'B-page escape(s) ripped and re-laid on B: '
              + ', '.join(f'{n}({laid[n].kind[0]}/{laid[n].direction[0]})'
                          for n in sorted(laid)))
    if tracks:
        add_tracks_and_vias_to_pcb(
            board, out_path, tracks, vias_add, vias_rm,
            net_id_to_name={i: n.name for i, n in pcb_e.nets.items()})
    else:
        shutil.copy(board, out_path)
    copy_pro(board, out_path)
elif TWO_PAGE_PLAN and not NO_HINTS \
        and os.environ.get('TP_SCOPE') == 'split':
    # PAGE-SPLIT FANOUT, engine-honest form. The engines cannot be
    # told a layer (a layers=['B.Cu'] underpad probe escaped 17/17
    # with ZERO vias -- disconnected B copper), and a verbatim
    # straight B leg clears almost nowhere (the decoupling caps live
    # on B under the array). So: VERBATIM what must be exact -- the
    # dogbone via at the ball, clearance-checked -- and ROUTED what
    # must negotiate -- the B run from the via to the planned face,
    # by the braid's own router around the cap field. The F rest then
    # goes to the engine, which routes round the B copper. Blueprint:
    # the HUMAN's own DU1 fanout (22 of 35 nets travel B, dogbones in
    # the gaps, no via-in-pad).
    import shutil
    from kicad_parser import Segment, Via
    stem2 = os.path.splitext(out_path)[0]
    split_targets = list(globals().get('split_targets', []))
    # TP_SPLIT_NETS: probe knob -- force named nets into the dest B
    # pass even when the plan's schedule called them F-page (SDQ14:
    # the braid refuses it on F at K32 AND K35, margin 6, no rip
    # victim; the human's answer is a B dogbone at the ball).
    for x in os.environ.get('TP_SPLIT_NETS', '').split(','):
        if x and x in names and x not in split_targets:
            split_targets.append(x)
    # CAPACITY GATE: try the PLAIN engine first and LOOK at what it
    # did. At K21/K28 the all-F fanout is clean and complete, and the
    # split only perturbs it (sp3 K28: split stubs starved the
    # channel, the under-pad rescue went dirty, braid 8 open); at
    # K32/K35 the plain fanout ships cap-pad grazes and elsewhere-
    # escapes. Engine for everyone -> adopt iff its copper is
    # DRC-clean and nothing failed; else the B pass proceeds from the
    # bare board.
    import subprocess as _sp
    pcb_p = parse_kicad_pcb(board)
    fp_p = pcb_p.footprints[dref]
    tr_p, va_p, vr_p, fl_p = generate_bga_fanout(
        fp_p, pcb_p, net_filter=names, layers=['F.Cu', 'B.Cu'],
        track_width=0.1, clearance=0.1, via_size=0.45, via_drill=0.25,
        exit_margin=0.5, escape_method=METHOD,
        plane_drop=('off' if NO_DROP else 'auto'),
        escape_dir_hints=hints,
        escape_line_hints=(None if NO_LINES else lines) or None)
    probe = f'{stem2}_dst_probe.kicad_pcb'
    if tr_p:
        add_tracks_and_vias_to_pcb(
            board, probe, tr_p, va_p, vr_p,
            net_id_to_name={i: n_.name for i, n_ in pcb_p.nets.items()})
    else:
        shutil.copy(board, probe)
    copy_pro(board, probe)
    r_ = _sp.run([sys.executable,
                  os.path.join(HERE, '..', 'py_router', 'check_drc.py'),
                  probe, '--clearance', '0.1',
                  '--clearance-margin', '0.1'],
                 capture_output=True, text=True)
    clean_ = 'NO DRC VIOLATIONS' in (r_.stdout + r_.stderr)
    if clean_ and not fl_p:
        print('  split GATE: plain engine fanout is DRC-clean and '
              'complete -- adopted, no B pass needed')
        # ...but a FORCED source split still applies. The K28 flank
        # economy question lives exactly here: the gate adopts and
        # exits, so TP_SRC_B_NETS could never reach a board whose
        # plain fanout is clean -- yet the source copper is
        # independent of the dest fanout. Same tooth walk as the
        # split-path source pass below, against the ADOPTED board.
        force_a = ([x for x in os.environ.get('TP_SRC_B_NETS', '')
                    .split(',') if x and x in names]
                   if os.environ.get('TP_SRC_B') else [])
        tr_a, va_a = [], []
        if force_a:
            pcb_a = parse_kicad_pcb(probe)
            oba = {}

            def ob_a(nid, L):
                if (nid, L) not in oba:
                    oba[(nid, L)] = te.build_obstacles(pcb_a, nid,
                                                       set(), L)
                return oba[(nid, L)]

            cxa = (dgrid.bbox[0] + dgrid.bbox[2]) / 2.0
            cya = (dgrid.bbox[1] + dgrid.bbox[3]) / 2.0
            pra = (te.VIA_SIZE - te.TRACK) / 2
            laid_a = []
            for n in force_a:
                nid = byname[n][0]
                t0 = launch[n]
                tl = tooth0.get(n, 'F.Cu')
                if tl == 'B.Cu':
                    continue
                h = math.hypot(cxa - t0[0], cya - t0[1]) or 1.0
                u = ((cxa - t0[0]) / h, (cya - t0[1]) / h)
                for j in range(4):
                    site = (t0[0] + 0.4 * j * u[0],
                            t0[1] + 0.4 * j * u[1])
                    bend = (site[0] + 0.6 * u[0],
                            site[1] + 0.6 * u[1])
                    if any(ob_a(nid, L_).point_violation(site, pad=pra)
                           is not None for L_ in LAYERS):
                        continue
                    if j and not ob_a(nid, tl).seg_clear(t0, site):
                        continue
                    if not ob_a(nid, 'B.Cu').seg_clear(site, bend):
                        continue
                    if j:
                        tr_a.append({'start': t0, 'end': site,
                                     'width': te.TRACK, 'layer': tl,
                                     'net_id': nid})
                    tr_a.append({'start': site, 'end': bend,
                                 'width': te.TRACK, 'layer': 'B.Cu',
                                 'net_id': nid})
                    va_a.append({'x': site[0], 'y': site[1],
                                 'size': te.VIA_SIZE,
                                 'drill': te.VIA_DRILL,
                                 'layers': ['F.Cu', 'B.Cu'],
                                 'net_id': nid})
                    laid_a.append(n)
                    break
            if laid_a:
                print('  split SOURCE pass (adopted board): '
                      + ','.join(laid_a))
        if tr_a:
            add_tracks_and_vias_to_pcb(
                probe, out_path, tr_a, va_a, [],
                net_id_to_name={i: n_.name
                                for i, n_ in pcb_p.nets.items()})
        else:
            shutil.copy(probe, out_path)
        copy_pro(probe, out_path)
        print(f'\nwrote {out_path}: {len(tr_p)} tracks, {len(va_p)} '
              f'vias, {len(set(fl_p))} failed nets')
        obeyed(tr_p, choice, dst_pad, 'berth')
        sys.exit(0)
    print('  split GATE: plain engine fanout is '
          + ('INCOMPLETE' if fl_p else 'DIRTY') + ' -- B pass proceeds')
    pcb_w = parse_kicad_pcb(board)
    cfg_w = te.cn.make_config(pcb_w, te.TRACK, te.CLEAR, te.VIA_SIZE,
                              te.VIA_DRILL, grid_step=0.025)
    wobs = {}
    n_laid = [0]

    def obs_w(nid, L):
        k = (nid, L, n_laid[0])
        if k not in wobs:
            wobs[k] = te.build_obstacles(pcb_w, nid, set(), L)
        return wobs[k]

    laid_b, b_tracks, b_vias = [], [], []
    hx, hy = dgrid.pitch_x / 2.0, dgrid.pitch_y / 2.0
    pad_r = (te.VIA_SIZE - te.TRACK) / 2
    used_exit = []          # B stub ends already landed on a face
    fail1 = []              # nets no dest-side lay could serve
    # TWO ROUNDS: the plain pass first for everybody (bit-identical to
    # the pre-rescue arm), then far sites + slot spread ONLY for its
    # refusals. Measured all-at-once (q4, 0831): the slot cascade
    # re-landed even the healthy nets (a newly-landed net buried SA1's
    # base exit before its turn) and the braid got WORSE -- K35 87v/3
    # -> 107v/4, K32 86v/4 -> 99v/6+1drc. Additive-only is the honest
    # form: the baseline's copper lays first and identically.
    queue = [[x, False] for x in names if x in set(split_targets)]
    for n, rescue in queue:
        nid = byname[n][0]
        p = dst_pad[n]
        m0 = choice.get(n)
        if m0 is None:
            continue
        bx, by = p.global_x, p.global_y
        dv = _DIRV[m0.direction]
        # candidate via sites, nearest first: the four diagonal cells
        # (exit-aligned first -- the original menu), then the HUMAN's
        # far-via idiom: walk the escape gap outward in the planned
        # direction, one cell corner at a time, out to the face line.
        # The human's own DU1 fanout puts 12 of its 35 vias 1.1-3.8 mm
        # from the ball (census 0831); the diagonals are the menu's
        # first page, not the menu.
        cands = []
        for sx, sy in sorted([(sx, sy) for sx in (-1, 1) for sy in (-1, 1)],
                             key=lambda s: -(s[0] * dv[0] + s[1] * dv[1])):
            site = (bx + sx * hx, by + sy * hy)
            cands.append((site, [((bx, by), site)]))
        if rescue:
            gx0, gy0, gx1, gy1 = dgrid.bbox
            for side in (-1, 1):
                if dv[0]:
                    gate = (bx + dv[0] * hx, by + side * hy)
                else:
                    gate = (bx + side * hx, by + dv[1] * hy)
                for k in range(1, 12):
                    if dv[0]:
                        site = (bx + dv[0] * (2 * k + 1) * hx, gate[1])
                        if not (gx0 - hx - 1e-6 <= site[0]
                                <= gx1 + hx + 1e-6):
                            break
                    else:
                        site = (gate[0], by + dv[1] * (2 * k + 1) * hy)
                        if not (gy0 - hy - 1e-6 <= site[1]
                                <= gy1 + hy + 1e-6):
                            break
                    cands.append((site, [((bx, by), gate), (gate, site)]))
        # exit SLOTS: the diagnosed failure mode (TP_DEBUG, 0831) is
        # not the site menu -- it is the LANDING. Same-gap nets get
        # near-identical face exit points, and the first-laid B run
        # buries every later one (backward frontier stillborn: "8/8
        # neighbors blocked" by an earlier split net's tracks). So the
        # exact exit point is negotiable and the face is not: shift
        # along the face tangent to the nearest slot >= 0.35 mm from
        # every landed stub end.
        ex0 = m0.exit_pt
        if rescue:
            tang = (0.0, 1.0) if dv[0] else (1.0, 0.0)
            exits = []
            for t_ in (0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2):
                ex = (ex0[0] + tang[0] * t_, ex0[1] + tang[1] * t_)
                if all(math.hypot(ex[0] - u[0], ex[1] - u[1]) >= 0.35
                       for u in used_exit):
                    exits.append(ex)
            exits = exits[:4] or [ex0]
        else:
            exits = [ex0]
        # every geometrically-clear site gets its RUN tried (the
        # first-clear site's run refusing does not mean the others'
        # would), each at two margins -- the last call's own lesson
        res, got, legs_got, ex_got = None, None, None, None
        dbg = os.environ.get('TP_DEBUG')
        for site, legs in cands:
            bad = next((L_ for L_ in LAYERS
                        if obs_w(nid, L_).point_violation(site, pad=pad_r)
                        is not None), None)
            if bad is not None:
                if dbg:
                    print(f'      {n} site ({site[0] - bx:+.2f},'
                          f'{site[1] - by:+.2f}): via blocked on {bad}')
                continue
            if not all(obs_w(nid, 'F.Cu').seg_clear(a_, b_)
                       for a_, b_ in legs):
                if dbg:
                    print(f'      {n} site ({site[0] - bx:+.2f},'
                          f'{site[1] - by:+.2f}): F leg blocked')
                continue
            for a_, b_ in legs:
                pcb_w.segments.append(Segment(a_[0], a_[1], b_[0], b_[1],
                                              te.TRACK, 'F.Cu', nid))
            pcb_w.vias.append(Via(site[0], site[1], te.VIA_SIZE,
                                  te.VIA_DRILL, ['F.Cu', 'B.Cu'], nid))
            for ex in exits:
                for mg in (1.2, 2.5):
                    res = te.cn.connect(pcb_w, nid, site, 'B.Cu',
                                        ex, 'B.Cu', cfg_w,
                                        band=None, margin=mg)
                    if res is not None:
                        break
                if res is not None:
                    ex_got = ex
                    break
            if res is not None:
                got, legs_got = site, legs
                break
            if dbg:
                print(f'      {n} site ({site[0] - bx:+.2f},'
                      f'{site[1] - by:+.2f}): B run refused')
            for _ in legs:
                pcb_w.segments.pop()
            pcb_w.vias.pop()
        if got is None:
            if not rescue and os.environ.get('TP_FAR'):
                # dest-side far-site + slot rescue: measured WORSE in
                # both forms (q4 all-at-once K35 87v/3 -> 107v/4; q5
                # additive-only 87v/8 + 11 drc) -- the extra dest-B
                # cohort is the regression, not the lay order. Kept as
                # a knob; the mechanism itself works (no-site 11 -> 3).
                queue.append([n, True])
                continue
            fail1.append(n)
            print(f'    split: no site/B run for {n} '
                  '-- engine keeps it')
            continue
        if math.hypot(got[0] - bx, got[1] - by) > 2 * hx + 1e-6:
            print(f'    split: far site for {n} at '
                  f'({got[0] - bx:+.2f},{got[1] - by:+.2f})')
        if ex_got is not None and (abs(ex_got[0] - ex0[0]) > 1e-6
                                   or abs(ex_got[1] - ex0[1]) > 1e-6):
            print(f'    split: {n} lands at a shifted slot '
                  f'({ex_got[0] - ex0[0]:+.2f},{ex_got[1] - ex0[1]:+.2f})')
        if ex_got is not None:
            used_exit.append(ex_got)
        segs_o, vias_o = res
        pcb_w.segments.extend(segs_o)
        pcb_w.vias.extend(vias_o)
        n_laid[0] += 1
        laid_b.append(n)
        for a_, b_ in legs_got:
            b_tracks.append({'start': a_, 'end': b_,
                             'width': te.TRACK, 'layer': 'F.Cu',
                             'net_id': nid})
        for s_ in segs_o:
            b_tracks.append({'start': (s_.start_x, s_.start_y),
                             'end': (s_.end_x, s_.end_y),
                             'width': s_.width, 'layer': s_.layer,
                             'net_id': nid})
        b_vias.append({'x': got[0], 'y': got[1], 'size': te.VIA_SIZE,
                       'drill': te.VIA_DRILL,
                       'layers': ['F.Cu', 'B.Cu'], 'net_id': nid})
        for v_ in vias_o:
            b_vias.append({'x': v_.x, 'y': v_.y, 'size': v_.size,
                           'drill': v_.drill,
                           'layers': ['F.Cu', 'B.Cu'], 'net_id': nid})
    # TP_SRC_B_NETS: probe knob -- force named nets into the source
    # pass even when the dest loop never saw them (an F-page net the
    # braid refuses, e.g. SDQ14, is invisible to split_targets).
    force_s = [x for x in os.environ.get('TP_SRC_B_NETS', '').split(',')
               if x and x in names and x not in fail1]
    if os.environ.get('TP_SRC_B') and (fail1 or force_s):
        # SOURCE-SIDE SPLIT: a net whose berth cannot reach B at the
        # destination gets its layer change at the SOURCE instead --
        # the human's own idiom for exactly this class (census 0831:
        # SA8/SBA1 dive inside the U1 window, ride B end-to-end,
        # surface past the array; 2 vias, constant layer between).
        # The stub is EXTENDED: via on the tooth line toward the
        # field, then a short B whisker, because endpoint discovery
        # requires a free end OUTSIDE every barrel -- the whisker is
        # what makes the B end discoverable. The lane is then born on
        # B with its one required change already paid.
        cx = (dgrid.bbox[0] + dgrid.bbox[2]) / 2.0
        cy = (dgrid.bbox[1] + dgrid.bbox[3]) / 2.0
        laid_s = []
        for n in fail1 + force_s:
            nid = byname[n][0]
            t0 = launch[n]
            tl = tooth0.get(n, 'F.Cu')
            if tl == 'B.Cu':
                continue
            h = math.hypot(cx - t0[0], cy - t0[1]) or 1.0
            u = ((cx - t0[0]) / h, (cy - t0[1]) / h)
            done = False
            for j in range(4):
                site = (t0[0] + 0.4 * j * u[0], t0[1] + 0.4 * j * u[1])
                bend = (site[0] + 0.6 * u[0], site[1] + 0.6 * u[1])
                if any(obs_w(nid, L_).point_violation(site, pad=pad_r)
                       is not None for L_ in LAYERS):
                    continue
                if j and not obs_w(nid, tl).seg_clear(t0, site):
                    continue
                if not obs_w(nid, 'B.Cu').seg_clear(site, bend):
                    continue
                add_t = ([{'start': t0, 'end': site, 'width': te.TRACK,
                           'layer': tl, 'net_id': nid}] if j else []) \
                    + [{'start': site, 'end': bend, 'width': te.TRACK,
                        'layer': 'B.Cu', 'net_id': nid}]
                for t_ in add_t:
                    pcb_w.segments.append(Segment(
                        t_['start'][0], t_['start'][1], t_['end'][0],
                        t_['end'][1], t_['width'], t_['layer'], nid))
                pcb_w.vias.append(Via(site[0], site[1], te.VIA_SIZE,
                                      te.VIA_DRILL, ['F.Cu', 'B.Cu'],
                                      nid))
                b_tracks.extend(add_t)
                b_vias.append({'x': site[0], 'y': site[1],
                               'size': te.VIA_SIZE,
                               'drill': te.VIA_DRILL,
                               'layers': ['F.Cu', 'B.Cu'],
                               'net_id': nid})
                n_laid[0] += 1
                laid_s.append(n)
                print(f'    split: {n} tooth taken to B at '
                      f'({site[0]:.2f},{site[1]:.2f})'
                      + (f' +{0.4 * j:.1f} along' if j else ''))
                done = True
                break
            if not done:
                print(f'    split: no source-side site for {n}')
        if laid_s:
            print(f'  split SOURCE pass: {len(laid_s)} tooth(s) '
                  'taken to B: ' + ','.join(laid_s))
    print(f'  split B pass: {len(laid_b)} of {len(split_targets)} '
          'berth(s) via-and-routed on B: ' + ','.join(laid_b))
    binter = f'{stem2}_dst_bsplit.kicad_pcb'
    if b_tracks:
        add_tracks_and_vias_to_pcb(
            board, binter, b_tracks, b_vias, [],
            net_id_to_name={i: n_.name for i, n_ in pcb_w.nets.items()})
    else:
        shutil.copy(board, binter)
    copy_pro(board, binter)
    pcb_e = parse_kicad_pcb(binter)
    fp_e = pcb_e.footprints[dref]
    rest = [nm for nm in names if nm not in set(laid_b)]
    hints_r = {pad_key(dst_pad[nm]): choice[nm].direction
               for nm in rest if nm in choice}
    lines_r = {pad_key(dst_pad[nm]): (choice[nm].exit_pt[1]
                                      if choice[nm].direction in ('left', 'right')
                                      else choice[nm].exit_pt[0])
               for nm in rest if nm in choice}
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        fp_e, pcb_e, net_filter=rest, layers=['F.Cu', 'B.Cu'],
        track_width=0.1, clearance=0.1, via_size=0.45, via_drill=0.25,
        exit_margin=0.5, escape_method=METHOD,
        plane_drop=('off' if NO_DROP else 'auto'),
        escape_dir_hints=hints_r,
        escape_line_hints=(None if NO_LINES else lines_r) or None)
    failed = set(failed)
    if tracks:
        add_tracks_and_vias_to_pcb(
            binter, out_path, tracks, vias_add, vias_rm,
            net_id_to_name={i: n_.name for i, n_ in pcb_e.nets.items()})
    else:
        shutil.copy(binter, out_path)
    copy_pro(binter, out_path)
    tracks = b_tracks + tracks
    vias_add = b_vias + vias_add
elif TWO_PAGE_PLAN and not NO_HINTS:
    # apply the plan's KIND per net, like the source apply: dogbones
    # and via-in-pads first (short stubs against the pad), channel
    # escapes last, routing round whatever copper is there
    import shutil
    stem2 = os.path.splitext(out_path)[0]
    kind_of = {nm: choice[nm].kind for nm in names if nm in choice}
    if os.environ.get('TP_SCOPE') in ('swim', 'split'):
        # swimmer scope: only the RE-PICKED berths are laid verbatim;
        # every untouched net goes through the production engine pass
        # below ('unplanned', with the plan's hints and lines), i.e.
        # the plain-plan fanout the 49-via arm graded. t11 laid ALL 28
        # moves verbatim and the braid opened 13 -- the verbatim apply
        # is a worse fanout than the engine for nets that didn't move.
        kind_of = {nm: k for nm, k in kind_of.items()
                   if nm in globals().get('repicked', set())}
    passes = []
    for kind, method in (('via_in_pad', 'underpad'), ('dogbone', 'dogbone'),
                         ('surface', 'channel')):
        nets_k = [nm for nm in names if kind_of.get(nm) == kind]
        if nets_k:
            passes.append((kind, method, nets_k))
    rest_k = [nm for nm in names if nm not in kind_of]
    if rest_k:
        passes.append(('unplanned', METHOD, rest_k))
    tracks, vias_add, vias_rm, failed = [], [], [], set()
    cur = board
    for i, (kind, method, nets_k) in enumerate(passes):
        pcb_k = parse_kicad_pcb(cur)
        fp_k = pcb_k.footprints[dref]
        last = i == len(passes) - 1
        if kind != 'unplanned':
            # DIRECT EMIT -- see the source apply: the engines do not
            # honour a target layer, the move's own geometry does. A
            # via-in-pad barrel is clamped for the 0.8 mm ball
            # (advanced fab tier); a gap-site dogbone keeps the
            # standard via.
            tr, va, vr, fl = [], [], [], []
            for nm in nets_k:
                m = choice[nm]
                nid = byname[nm][0]
                for (a_, b_, L_) in m.legs:
                    tr.append({'start': a_, 'end': b_, 'width': 0.127,
                               'layer': L_, 'net_id': nid})
                if m.site is not None:
                    in_pad = m.kind == 'via_in_pad'
                    va.append({'x': m.site[0], 'y': m.site[1],
                               'size': 0.35 if in_pad else 0.45,
                               'drill': 0.2 if in_pad else 0.25,
                               'layers': ['F.Cu', 'B.Cu'], 'net_id': nid})
        else:
            hints_k = {pad_key(dst_pad[nm]): choice[nm].direction
                       for nm in nets_k if nm in choice}
            lines_k = {pad_key(dst_pad[nm]): (choice[nm].exit_pt[1]
                                              if choice[nm].direction in ('left', 'right')
                                              else choice[nm].exit_pt[0])
                       for nm in nets_k if nm in choice}
            tr, va, vr, fl = generate_bga_fanout(
                fp_k, pcb_k, net_filter=nets_k, layers=['F.Cu', 'B.Cu'],
                track_width=0.1, clearance=0.1, via_size=0.45, via_drill=0.25,
                exit_margin=0.5, escape_method=method,
                plane_drop=(('off' if NO_DROP else 'auto') if last else 'off'),
                escape_dir_hints=hints_k,
                escape_line_hints=(None if NO_LINES else lines_k) or None)
        nxt = f'{stem2}_dst_{kind}.kicad_pcb'
        if tr:
            add_tracks_and_vias_to_pcb(
                cur, nxt, tr, va, vr,
                net_id_to_name={ii: n.name for ii, n in pcb_k.nets.items()})
        else:
            shutil.copy(cur, nxt)
        copy_pro(cur, nxt)
        print(f'  {kind} -> {method}: {len(nets_k)} net(s), {len(tr)} tracks, '
              f'{len(va)} vias, {len(fl)} failed'
              + (f' ({", ".join(sorted(fl)[:6])})' if fl else ''))
        tracks.extend(tr)
        vias_add.extend(va)
        vias_rm.extend(vr)
        failed |= set(fl)
        cur = nxt
    shutil.copy(cur, out_path)
    copy_pro(cur, out_path)
else:
    tracks, vias_add, vias_rm, failed = generate_bga_fanout(
        fp, pcb,
        net_filter=names,
        layers=['F.Cu', 'B.Cu'],
        track_width=0.1, clearance=0.1,
        via_size=0.45, via_drill=0.25,
        exit_margin=0.5,
        escape_method=METHOD,
        plane_drop=('off' if NO_DROP else 'auto'),
        escape_dir_hints=(None if NO_HINTS else hints),
        escape_line_hints=(None if (NO_HINTS or NO_LINES) else lines),
    )

    net_names = {nid: n.name for nid, n in pcb.nets.items()}
    if tracks:
        add_tracks_and_vias_to_pcb(board, out_path, tracks, vias_add, vias_rm,
                                   net_id_to_name=net_names)
    else:
        import shutil
        shutil.copy(board, out_path)
    copy_pro(board, out_path)

print(f'\nwrote {out_path}: {len(tracks)} tracks, {len(vias_add)} vias, '
      f'{len(failed)} failed nets')
obeyed(tracks, choice, dst_pad, 'berth')
if failed:
    print(f'  failed nets: {", ".join(sorted(failed)[:10])}')
