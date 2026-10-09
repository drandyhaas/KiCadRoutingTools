"""whole_solve.py OUT.json -- the WHOLE-ROUTE crossing and layer solve (CP-SAT), one permutation: the launch order
-> the berths' order round the destination's pad box (unrolled from a cut on its far face between the branches).
Each lane's route is ONE
coordinate u: trunk s from its entry (its tooth's projection) to the handoff, then its ring (u = handoff + s_b -
ring start) to its berth; a head lane's trunk s to its berth. Every inverted pair crosses once, at a u both share
(a pair in one branch may cross on the ring, up to the earlier leg); the braid triple rule over ALL triples;
each lane's crossings a pitch apart along its track (a stayer), or less for a sweep (a mover), and a pair's two
crossings of opposite ways its turning run and a pitch apart (no zigzag it cannot turn); up to KMAX changes per lane,
each a via's half-room from its own crossings and a via's room apart, a single's a change's room from both its ends
(END_ROOM) and a pair's its dive room; no crossing and no change in the band along the source's near face
(FACE_ROOM), where the teeth stand; neighbouring lanes' changes -- neighbours at either end, or two that cross --
staggered along the route so their vias clear; tooth layer at the start, berth layer at the end. Objective: the nets
over two vias first, then the vias, then HISTORY congestion (the crossings and changes in the places earlier rounds'
audits found the plan short, HIST).

The bench from BENCH / NETS / DEST (whole_ctx). HINT=SOLVE.json warm-starts from an earlier solve; CUTS=GEO.json,..
adds the geometry's cuts (whole_geo: the islands a lane could not be kept off, the changes it could not give their
room). HIST=HOT.json,.. prices the places earlier audits found short (whole_gate --hot). The solve is bounded in WORK,
not time: a count of CP-SAT's interleaved batches, its workers pinned and sharing no clauses, so the same model gives
the same answer on every run, later on a slower machine (WHOLE_SOLVE_BATCHES sets the budget). It stops sooner when
the vias are proved (the plan's vias no more than the bound's whole vias) or, once it has a plan, when it STALLS
(SOLVE_STALL of its own model reductions in a row with no better plan or bound: events of the search, never a clock)
-- all but the fallback, the plan-finding workers' last try, which runs its whole budget. Only a plan PROVED optimal in its vias is written;
one the search could not prove is no plan -- except under SOLVE_UNPROVED=1 (a round's first solve in whole_route,
which never ends with nothing): the best plan found is written marked 'proved': false, with the nets it leaves over
two vias ('over_nets'), and never as a floor for a later solve. Under SOLVE_CROWD=1 (that solve too) a model with NO
plan is solved again as its CROWD diagnosis, and the lanes whose crossings the ends leave no room for are written to
OUT.crowded.json (solve(crowd=True))."""
import sys, os, re, itertools, collections, json, math, hashlib
import awx_settings
import whole_ctx
import whole_frame
import route_layers
from ortools.sat.python import cp_model
import braid as bd
import pairs as _pairs
# every length below is in the design rules' own units: track, clearance, via size, a via's room, the lane pitch
TRK, CLR, VIA, VNEED, PITCH = bd.TRACK, bd.CLEAR, bd.VIA_SIZE, bd.VIA_NEED, bd.LANE_MIN
LEGROOM = PITCH                        # a peeling leg crosses, then still runs a pitch to its berth
VR_STAY = 1.1                          # a change's room from a stayer's crossing: its via is passed at an angle
KMAX = 3                               # layer changes per lane at most (every proved plan K15-K51 has 2 at most: a
                                       # fourth only widened the model -- K51 proved in 37 s at 3 with SUBSOLVERS)
G = PITCH / 5                          # the solve's time grid: a fifth of the lane pitch
MARG = CLR                             # a crossing starts a clearance past both lanes' terminals (a lane leaving its tooth may cross at once)
# a crossing's room along each lane: a STAYER's crossings a pitch apart, a MOVER's (a sweep crossing a bundle nearly
# across the spine, its slope up to K_SWEEP) PITCH / sqrt(1 + K^2) apart along s; the geometry keeps the real pitch
K_SWEEP = 4.0
W_V = 10 ** 6                          # per via: vias first (an integer: the objective stays CP-SAT's exact one)
W_OVER_X = 5                           # each via a net carries past two costs this many vias MORE (whole_ends.W_OVER)
SOLVE_BATCHES = int(awx_settings.get('WHOLE_SOLVE_BATCHES', '100'))  # CP-SAT interleaved batches: the work budget
SOLVE_WORKERS = 4
# ...running these: two LP workers (the default and the strongest relaxation), core-based search and the objective's
# lower-bound search. The default four ran nothing that raises the bound, and a plan's vias are proved from below: K51's
# first solve stopped unproved at 421 s (best 56 vias, bound 40), with these it proves 42 in 141 s, K41 in 17 s (38)
SUBSOLVERS = ['default_lp', 'max_lp', 'core', 'objective_lb_search']
# ...and when they cannot prove it, the PLAN-FINDING workers the first four leave out, with core-based search to close
# the proof from their plan (the finders alone found K51-on-the-human's-fanout's plan and left its bound where it was)
FALLBACK = ['quick_restart', 'no_lp', 'core']
FACE_ROOM = 2 * VNEED                  # the band along the source's near face: a change's room along its lane
SOLVE_STALL = 3                        # the search's model reductions in a row with no progress: stalled
WARM_WORK = 30.0                       # deterministic work to lay a re-solve's warm start out whole (one worker)
W_FIRM = 100 * W_V                     # a broken FIRM cut (CUTS_FIRM=1, more layers than two): above everything else a
#                                        plan can buy -- the solve holds every cut it can and breaks one only where no
#                                        plan holds them all, in ONE solve (whole_route's loop: no hard pass first)
STATUS = {}                            # the last solve's outcome: 'plan', whether its search found any plan at all
W_SOFT = 3 * W_V                       # a broken SOFT cut: above a dive's two vias, below a net over two -- whole vias,
#                                        as the search's proof reads them (at two and a half, a plan holding a cut two
#                                        vias dearer than one breaking it floored to the same whole number, proved)


def solve(ctx, dest, cuts=(), hist=(), hint=None, soft_cuts=(), crowd=False):
    """the solve of the bench ctx (whole_ctx.plan()) round the destination part `dest`: the JSON
    whole_geo reads (crossings, changes, each lane's route coordinate), or None when CP-SAT finds no plan or none it
    proves optimal in its vias. `cuts`
    names the geometry's cut files (whole_geo), `hist` the audits' hot files (whole_gate --hot), `hint` an earlier
    solve to warm-start from. `crowd`: the CROWD diagnosis of a model with no plan -- each lane's crossings may break
    their room along it (their spacing, and the stack of crossers at one point), the plan's only price the lanes that
    do, so it breaks it on the fewest lanes there can be, named in the JSON's 'crowded': the lanes whose crossings the
    ends leave no room for. Its plan is no plan to lay"""
    Fr = whole_frame.build(ctx, dest)          # the whole route's own frame (whole_frame.py)
    prs = getattr(ctx, 'pairs', {}) or {}
    M = list(Fr.M)
    F = lambda x: 1 if x == 'B.Cu' else 0
    tl = {n: F(ctx.tooth_layer[n]) for n in M}
    dl = {n: F(ctx.dest_layer[n]) for n in M}
    # the ROUTING LAYERS (route_layers: F.Cu and B.Cu, and the inner ones ROUTE_LAYERS adds). On two a lane's layer is
    # one bit, flipped by each change; on more each run between changes has a layer of its own (YL, below), a change
    # going to any other, the end runs on the stubs' layers
    LAYN = route_layers.layers()
    NL = len(LAYN)
    tli = {n: LAYN.index(ctx.tooth_layer[n]) for n in M} if NL > 2 else tl
    dli = {n: LAYN.index(ctx.dest_layer[n]) for n in M} if NL > 2 else dl
    YL = {}
    # ...and VIA ENDS (more layers than two, VIA_ENDS=1, the default there): an end whose stub carries a via inside its
    # array -- a dog-bone's, a via in its pad, each leg of a pair -- can run its last stretch, from that via out to the
    # array's edge, on any routing layer its run fits on (below), and the relayer moves it there (whole_route): that
    # end's layer is the solve's to choose. So the human runs the AD9364's LVDS bus: a dog-bone at both arrays, each
    # net on one of two inner pages from its via out. On its neck's own layer (F.Cu) the via joins nothing, and the
    # relayer drops it: a synth bus whose crossings have no 2-colouring on the inner pages and no room for a change in
    # its 3 mm channel (shuffle seed 4) had no plan while F.Cu was refused there
    import relayer as _relayer

    def via_end(n, k_):
        """each leg's stub at end k_ (0 the tooth, 1 the berth) carries a via inside its array, and its end stands on
        that via or runs to it on one layer -- what the relayer can move (relayer.run_to_via): a via of the net in the
        box its stub does not reach would be an end the plan moves and the copper leaves where it was"""
        box = (Fr.SB, Fr.DB)[k_]
        if n in prs:
            (sp_, sn_), (tp_, tn_) = ctx.pair_ends[n]
            legs_ = ((prs[n][0], (sp_, tp_)[k_], (ctx.tooth_layer, ctx.dest_layer)[k_][n]),
                     (prs[n][1], (sn_, tn_)[k_], ctx.pair_layers[n][k_]))
        else:
            legs_ = ((n, ctx.ends[n][k_], (ctx.tooth_layer, ctx.dest_layer)[k_][n]),)
        for leg, pt_, L0_ in legs_:
            nid = ctx.byname[leg][0]
            vs_ = [v for v in ctx.base_vias if v.net_id == nid and box[0] - 1e-6 <= v.x <= box[2] + 1e-6
                   and box[1] - 1e-6 <= v.y <= box[3] + 1e-6]
            if not vs_:
                return False
            if not any(math.hypot(v.x - pt_[0], v.y - pt_[1]) < v.size / 2 for v in vs_) \
                    and not _relayer.run_to_via(ctx.pcb, nid, pt_, L0_):
                return False
        return True
    VEND = {}
    if NL > 2 and (awx_settings.get('VIA_ENDS') or '1') == '1':
        VEND = {(n, k_): True for n in M for k_ in (0, 1) if via_end(n, k_)}
        print(f'   {NL} routing layers {",".join(LAYN)}; via ends, their layers the solve\'s: '
              f'{sum(1 for k_ in VEND if k_[1] == 0)} teeth, {sum(1 for k_ in VEND if k_[1] == 1)} berths of {len(M)} lanes')
    # ...where their runs can lie: a via end's run (its stub from the via out, as the relayer moves it) on another
    # layer lies beside the copper there -- another net's stub or pad within the rule bans that layer, and two runs
    # within the rule of each other (crossing, laid on two layers) end on different layers (the ns3_rev synth: two
    # berths' runs crossing on In2.Cu and B.Cu, both moved onto In2.Cu, shorted)
    VBAN, VSEP = {}, []
    if VEND:
        import rules as _rules
        runs_ = {}
        for (n, k_) in sorted(VEND):
            if n in prs:
                (sp_, sn_), (tp_, tn_) = ctx.pair_ends[n]
                legs_ = ((prs[n][0], (sp_, tp_)[k_], (ctx.tooth_layer, ctx.dest_layer)[k_][n]),
                         (prs[n][1], (sn_, tn_)[k_], ctx.pair_layers[n][k_]))
            else:
                legs_ = ((n, ctx.ends[n][k_], (ctx.tooth_layer, ctx.dest_layer)[k_][n]),)
            segs_, nids_ = [], set()
            for leg, pt_, L0_ in legs_:
                nid = ctx.byname[leg][0]
                nids_.add(nid)
                segs_ += _relayer.run_to_via(ctx.pcb, nid, pt_, L0_) or []
            if segs_:
                runs_[(n, k_)] = (nids_, segs_)
        VBAN, VSEP = _relayer.clashes(ctx.pcb, runs_, LAYN, _rules.active().fan_clear)
        # (never the layer a run is laid on, nor two already laid on one: the fanout's own, as it stands)
        on_ = lambda key: (ctx.tooth_layer, ctx.dest_layer)[key[1]][key[0]]
        VBAN = {k_: v_ - {on_(k_)} for k_, v_ in VBAN.items() if v_ - {on_(k_)}}
        VSEP = [(a_, b_) for a_, b_ in VSEP if on_(a_) != on_(b_)]
        if VBAN or VSEP:
            print(f'   via ends\' runs: {sum(len(v_) for v_ in VBAN.values())} layer(s) banned by the copper there, '
                  f'{len(VSEP)} pair(s) of runs on different layers')
    Dv = {n: 2 * VNEED + (_pairs.pitch(TRK) if n in prs else 0.0) for n in M}     # a change's room along its lane
    Q = lambda s: int(round(s / G))
    QU = lambda s: int(math.ceil(s / G - 1e-9))
    # ---- the whole route of each lane in u: its trunk s from its entry, and for a ring lane (bname: its class, the
    # face of the destination it berths on) past the arrival line H0 its ring's
    spine, bname, H0, ring_of, Hk = Fr.spine, Fr.cls, Fr.H0, Fr.rings, Fr.Hk
    Hn = {n: Hk[bname[n]] for n in bname}         # where each ring lane leaves the trunk: its ring's start
    # a ring's route coordinate starts where the geometry starts it: ahead of every class lane's handoff point, on a
    # geometry column (two solve steps) -- ONE origin for the solve and the layout
    GG = 2 * G
    rs = {}
    for k_, b_ in ring_of.items():
        sbs = [b_.project_pt(spine.xy(Hk[k_], Fr.o_h[n]))[0] for n in M if bname.get(n) == k_]
        rs[k_] = math.ceil(max(sbs) / GG - 1e-9) * GG
    entry = {n: Fr.st[n][0] for n in M}
    def u_ring(n, s_b):
        return Hn[n] + (s_b - rs[bname[n]])
    end = {}
    for n in M:
        if n in bname:
            end[n] = u_ring(n, ring_of[bname[n]].project_pt(Fr.land[n])[0])     # where it lands (whole_frame)
        else:
            end[n] = spine.project_pt(Fr.land[n])[0]     # its landing: its berth, or a trunk pair's across its face
    tend = {n: (Hn[n] if n in bname else end[n]) for n in M}      # where the lane leaves the TRUNK frame
    # a PAIR's end: no CROSSING inside its END CONNECTOR (pairs.end_connector: the legs from its tips to the pose where the
    # pair router takes over; SDQS1 once crossed SDQ15 and SDQ13 in the last 0.3 mm before their berths), and no CHANGE of
    # its own nearer than its DIVE ROOM (pairs.dive_room: that connector, then the router's straight from the pose into the
    # via). A crossing lane is on the other layer there; only the pair's own dive has to stand beyond its pose
    RIN0 = {n: (_pairs.end_connector(ctx.cfg, ctx.pair_ends[n][0]) if n in prs else 0.0) for n in M}   # at the tooth end
    RIN1 = {n: (_pairs.end_connector(ctx.cfg, ctx.pair_ends[n][1]) if n in prs else 0.0) for n in M}   # ... the berth end
    _axis = lambda u: u if u is not None else (1.0, 0.0)
    # ...and a SINGLE's change stands a change's room from its own tooth and berth (as two of its changes stand apart):
    # there its neighbours have not spread from the array's edge yet, on either layer, and a via planned 0.2 past a
    # tooth had no room beside it (K28: SDQ11 0.47 and SCKE1 0.59 past theirs, each moved by a via cut a round later).
    # Its crossings keep their own windows
    END_ROOM = 2 * VNEED
    VIN0 = {n: (_pairs.dive_room(ctx.cfg, ctx.pair_ends[n][0], _axis(ctx.tooth_dir.get(n))) if n in prs else END_ROOM)
            for n in M}
    VIN1 = {n: (_pairs.dive_room(ctx.cfg, ctx.pair_ends[n][1], _axis(ctx.stub_dir.get(n))) if n in prs else END_ROOM)
            for n in M}
    # ...a CROSSED pair's (pairs.opposite_hands: its legs swap once, at its first dive, a crossover) by the crossover's
    # own runs (pairs.crossover_room) -- its first change from its tooth, and at its berth where that change is its only
    # one; its other changes are plain dives, and its changes' cuts are the crossover's, the longer
    # a single's end whose stub runs into copper a via's room past its exit (the ends model's BLOCKED FRONT, priced as the
    # change before it: whole_ends FRONT_VIA; the fanout's plan sidecar, ends_model.front): the lane stands on the OTHER
    # layer across that copper (a LAYER cut, below), its change between the stub and the copper -- its end room there a
    # via's own. A ring lane's berth stub runs across its ring, not along its route: its front is the geometry's
    FRONT = {}
    try:
        _side = os.path.splitext(awx_settings.req('BENCH'))[0] + '.plan.json'
        FRONT = (json.load(open(_side)).get('ends_model') or {}).get('front') or {} if os.path.isfile(_side) else {}
    except Exception:
        FRONT = {}
    LAYER_CUTS = []
    for n, ends_ in sorted(FRONT.items()):
        if n not in M or n in prs:
            continue
        # (from the exit, as the ends model measured it: the span where its TRACK meets the copper's clearance, the
        # stretch before it where a via of its own fits -- the change stands there, the lane's end room waived down to
        # it, and nowhere between it and the far side of the copper, a via's half width more than a track's)
        for k_, sp_ in sorted(ends_.items()):
            k_, vm_ = int(k_), (VIA - TRK) / 2
            if k_ == 1 and n in bname or len(sp_) != 4:
                continue
            v0_, v1_, d0_, d1_ = sp_
            if k_ == 1:
                LAYER_CUTS.append((n, k_, 1 - dl[n], end[n] - d1_, end[n] - d0_, end[n] - d1_ - vm_, end[n] - v1_ - G / 2,
                                   end[n] - v1_, end[n] - v0_))
                VIN1[n] = min(VIN1[n], v0_)
            else:
                LAYER_CUTS.append((n, k_, 1 - tl[n], entry[n] + d0_, entry[n] + d1_, entry[n] + v1_ + G / 2,
                                   entry[n] + d1_ + vm_, entry[n] + v0_, entry[n] + v1_))
                VIN0[n] = min(VIN0[n], v0_)
    # ...and a single lane the loop found blocked by an island on one layer -- its audit against it, or its snap stuck
    # beside it (whole_route's LAYER cuts, `lcuts` in the cut files: a wall the lanes cannot go round): on the other layer
    # across the island's box on the trunk, a track's clearance either side, its changes a via's clearance off it
    CHAN_CUTS, CHAN_T0 = set(), set()
    CHAN_BLK = {}             # (lane, 'chan ISLAND') -> the island's layers (more than two: the run held off each)
    # (each cut once: one the cut files carry twice -- a hard round's in CUTS and again in SOFT_CUTS -- read a second
    # time after the first had waived its lane's end room, found no window, and the two sorted None against a number)
    _seen_lc = set()
    for fn_ in list(cuts) + list(soft_cuts):
        for c_ in json.load(open(fn_)).get('lcuts', []):
            k_lc = (c_['lane'], c_['island'], int(c_['layer']), tuple(round(float(v_), 6) for v_ in c_['box']),
                    tuple(c_.get('span') or ()))
            if k_lc in _seen_lc:
                continue
            _seen_lc.add(k_lc)
            n_ = c_['lane']
            # (a pair's lane too on more layers than two: its legs on one layer, held off the island's as a single's)
            if n_ not in M or (n_ in prs and NL == 2):
                continue
            L_ = int(c_['layer'])
            blk_ = sorted(int(b_) for b_ in c_.get('blocked', [1 - L_])) if NL > 2 else None
            x0_, y0_, x1_, y1_ = c_['box']
            cnr_ = ((x0_, y0_), (x0_, y1_), (x1_, y0_), (x1_, y1_))
            ss_ = [float(spine.project_pt(q_)[0]) for q_ in cnr_]
            spans_ = []
            if c_.get('span'):
                # (the stretch of the lane's own route that met the island, in route u: whole_route.own_spans)
                lo_s, hi_s = (float(v_) for v_ in c_['span'])
                if hi_s >= entry[n_] and lo_s <= end[n_]:
                    spans_.append((max(lo_s, entry[n_]), min(hi_s, end[n_])))
            elif not (max(ss_) < entry[n_] or min(ss_) > tend[n_]):
                spans_.append((min(ss_), max(ss_)))              # (on the lane's trunk)
            if NL > 2 and n_ in bname and not c_.get('span'):
                # (...and on its ring, past its handoff: the island's span along the ring's spine, in the lane's u)
                us_ = [u_ring(n_, float(ring_of[bname[n_]].project_pt(q_)[0])) for q_ in cnr_]
                if max(us_) > Hn[n_] and min(us_) < end[n_]:
                    spans_.append((max(min(us_), Hn[n_]), max(us_)))
            for s0_, s1_ in spans_:
                t_, v_ = TRK / 2 + CLR, VIA / 2 + CLR
                lo_c, hi_c, w0_, w1_ = s0_ - v_, s1_ + v_, None, None
                # (an island so near the lane's tooth -- or a trunk lane's berth -- that its end room leaves no change
                # before it -- after it: the change from the stub's end to the island, its end room and the face band
                # waived and no stagger there, its neighbours still a ball pitch apart, as at a blocked front. On more
                # layers than two: a single lane whose end there is fixed -- no via end -- on one of the island's layers)
                if NL == 2:
                    off0, off1 = tl[n_] != L_, dl[n_] != L_
                else:
                    off0 = n_ not in prs and (n_, 0) not in VEND and tli[n_] in blk_
                    off1 = n_ not in prs and (n_, 1) not in VEND and dli[n_] in blk_
                if off0 and lo_c < entry[n_] + VIN0[n_] + G:
                    w0_, w1_ = entry[n_], lo_c
                    VIN0[n_] = 0.0
                    CHAN_T0.add(n_)
                elif n_ not in bname and off1 and hi_c > end[n_] - VIN1[n_] - G:
                    w0_, w1_ = hi_c, end[n_]
                    VIN1[n_] = 0.0
                CHAN_CUTS.add((n_, 'chan ' + c_['island'], L_, round(s0_ - t_, 4), round(s1_ + t_, 4), round(lo_c, 4),
                               round(hi_c, 4), w0_, w1_))
                if blk_ is not None:
                    CHAN_BLK[(n_, 'chan ' + c_['island'])] = blk_
    CHAN_CUTS = sorted(CHAN_CUTS)
    XO = {n for n in prs if n in M and _pairs.opposite_hands(ctx, n)}
    for n in XO:
        VIN0[n] = _pairs.crossover_room(ctx.cfg, ctx.pair_ends[n][0], _axis(ctx.tooth_dir.get(n)), 0)
        VIN1[n] = max(VIN1[n], _pairs.crossover_room(ctx.cfg, ctx.pair_ends[n][1], _axis(ctx.stub_dir.get(n)), 1))
    # a PAIR's KNOWN TURNS: the pair router neither turns at its via nor within its straight run of one, so its changes
    # stay out of every stretch of its route where the frames already turn (below, as built-in via cuts) -- and, where an
    # end's stub stands more than its connector's 45 degrees off the route's own way there, beyond the turn onto that way
    # as well. A turn is one the router must make: half a router step or more. The rest -- a lane's own sweep onto its
    # berth -- only the geometry knows, and it and the polish send those (whole_geo / whole_polish vcuts)
    TURN_DEG = 22.5
    L_DIVE = _pairs.via_straight(ctx.cfg, (math.sqrt(0.5), math.sqrt(0.5))) + ctx.cfg.grid_step   # the longer (diagonal) run
    _xs = _pairs.crossover_shape(ctx.cfg, (math.sqrt(0.5), math.sqrt(0.5)))
    L_XO = (max(_xs[1], _xs[2]) + ctx.cfg.grid_step) if _xs else L_DIVE                        # ... a crossover's
    LD = lambda n: L_XO if n in XO else L_DIVE


    def turn_room(deg):
        """half the stretch a pair's turn of `deg` takes: its 45-degree turns, a turning radius's straight run apart"""
        return _pairs.turn_straight_steps(ctx.cfg) * ctx.cfg.grid_step * max(0, math.ceil(abs(deg) / 45.0 - 1e-9) - 1) / 2


    def _deg(a, b):
        return math.degrees(math.acos(max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (math.hypot(*a) * math.hypot(*b))))))


    def route_dir(n, u):
        """the unit way lane n's route runs at u: its trunk's spine, or past the handoff its ring's"""
        if n in bname and u > Hn[n]:
            sp_ = ring_of[bname[n]]
            return tuple(sp_.d[sp_.seg_of(u - Hn[n] + rs[bname[n]])])
        return tuple(spine.d[spine.seg_of(u)])


    for n in prs:
        if n not in M:
            continue
        for k_, (u_e, esc, sg) in enumerate(((entry[n], ctx.tooth_dir.get(n), 1.0), (end[n], ctx.stub_dir.get(n), -1.0))):
            if esc is None:
                continue
            rd = route_dir(n, u_e)
            th = _deg((esc[0] * sg, esc[1] * sg), rd) - 45.0         # what the connector's 45 degrees leave to turn
            if th >= TURN_DEG:
                if k_ == 0:
                    VIN0[n] += 2 * turn_room(th) + LD(n)
                else:
                    VIN1[n] += 2 * turn_room(th) + LD(n)
    # ...and TOOTH VIAS (more routing layers than two): a lane whose tooth is no via end -- its stub laid on the surface
    # by the source's own fanout -- may change layer right at its tooth, a via at its stub's end on the array's edge, as
    # at a blocked front: no end room (a pair keeps its dive room), the face band waived, and in its window no stagger
    # (its neighbours a ball pitch across along the face) and no built-in via cut -- at a via's cost, as any change. So
    # the lanes leave the source's surface escapes for whichever layer the plan gives them where the gap between the
    # arrays has no room for their changes (the zynq's U1 to U5)
    TVIA = {n for n in M if not via_end(n, 0)} if NL > 2 else set()     # (the bench's own: VIA_ENDS holds no tooth)
    for n in TVIA:
        if n not in prs:
            VIN0[n] = 0.0

    def site_clear(n, k_):
        """a through via at single lane n's stub end k_ (0 its tooth, 1 its berth) clear of every other net's copper on
        every layer by a via's room -- measured, as the ends model measures a blocked front's"""
        pt_ = ctx.ends[n][k_]
        nid = ctx.byname[n][0]
        r_ = bd.VIA_SIZE / 2 + bd.CLEAR
        if any(s_.net_id != nid and _pairs._pt_seg(pt_, (s_.start_x, s_.start_y), (s_.end_x, s_.end_y))
               < r_ + s_.width / 2 for s_ in ctx.base_segments):
            return False
        if any(v_.net_id != nid and math.hypot(v_.x - pt_[0], v_.y - pt_[1]) < r_ + v_.size / 2 for v_ in ctx.base_vias):
            return False
        from routing_utils import point_to_pad_rect_dist
        return not any(p_.net_id != nid and point_to_pad_rect_dist(pt_[0], pt_[1], p_) < r_
                       for fp_ in ctx.pcb.footprints.values() for p_ in fp_.pads
                       if abs(p_.global_x - pt_[0]) < 3 and abs(p_.global_y - pt_[1]) < 3)
    # ...and BERTH VIAS, the same at the destination: a single lane whose berth is a surface escape may change layer
    # right at it, where a via at its stub's end is measured clear -- no end room, and in its window no stagger and no
    # built-in via cut (a surface berth kept its whole end room, and its lane's last change stood that far back)
    BVIA = {n for n in M if n not in prs and not via_end(n, 1) and site_clear(n, 1)} if NL > 2 else set()
    for n in BVIA:
        VIN1[n] = 0.0
    print('classes:', dict(collections.Counter(bname.get(n, 'W') for n in M)), 'W ends', sorted(round(end[n], 2) for n in M if n not in bname))
    Ln, Fn = list(Fr.launch), list(Fr.final)                  # both north to south (whole_frame)
    li = {n: i for i, n in enumerate(Ln)}; fi = {n: i for i, n in enumerate(Fn)}
    inv = lambda a, b: (li[a] < li[b]) == (fi[a] > fi[b])
    pairs = [(a, b) for a, b in itertools.combinations(Ln, 2) if inv(a, b)]
    same = lambda a, b: a in bname and b in bname and bname[a] == bname[b]
    # the FACE BAND: no crossing and no change within a change's room (FACE_ROOM) of the source's near face line --
    # lanes leaving its corner and its side faces (still running along them, beside its pad box) have not spread
    # there, and the geometry folded or squeezed what the solve put in it (K28: SCKE1's via beside the SCK pair at the
    # south-east corner; K35: SA9 and SCK short of their pitch along the south face). A side face's lanes run in the
    # band and cross beyond it
    S_FACE = float(spine.project_pt((Fr.SB[2], (Fr.SB[1] + Fr.SB[3]) / 2))[0])
    BAND = S_FACE + FACE_ROOM
    win = {}
    # (more routing layers than two: two lanes cross on different layers, always -- a crossing in the band is one run
    # over another, which the geometry orders and spaces per layer; the band holds the CHANGES, whose vias stand on
    # every layer, alone. The band's evidence is two-layer: a pitch folded on one layer, K28/K35)
    BAND_X = BAND if NL == 2 else 0.0
    n_empty = 0
    for a, b in pairs:
        lo = max(entry[a] + max(MARG, RIN0[a]), entry[b] + max(MARG, RIN0[b]), BAND_X)
        # a crossing on the ring leaves the earlier lane's leg LEGROOM to reach its berth after it
        hi = min(end[a] - max(LEGROOM, RIN1[a]), end[b] - max(LEGROOM, RIN1[b])) if same(a, b) else \
            min(tend[a] - (max(MARG, RIN1[a]) if a not in bname else MARG), tend[b] - (max(MARG, RIN1[b]) if b not in bname else MARG))
        n_empty += hi < lo + G
        win[(a, b)] = (lo, max(hi, lo + G))
    if n_empty:
        # (an inverted pair with no room to cross is given a grid step at its window's start: a plan the geometry then
        # pays for -- the frame's rings started at the source on zynq's U1 to U5)
        print(f'   crossing windows EMPTY, held open a grid step: {n_empty} of {len(win)}')
    m = cp_model.CpModel()
    t = {k: m.NewIntVar(Q(lo), Q(hi), f't_{k[0]}_{k[1]}') for k, (lo, hi) in win.items()}
    CROWD = {n: m.NewBoolVar(f'crowd_{n}') for n in M} if crowd else {}

    def present(lit, n):
        """a crossing's room along lane n is there when `lit` -- and, in the CROWD diagnosis, n is not crowded"""
        if not CROWD:
            return lit
        q_ = m.NewBoolVar('')
        m.AddBoolAnd([lit, CROWD[n].Not()]).OnlyEnforceIf(q_)
        m.AddBoolOr([lit.Not(), CROWD[n], q_])
        return q_
    # ---- geometry cuts (whole_geo.py's islands a lane could not be kept off): none of that lane's crossings in the span
    CUTS = []
    for fn_ in cuts:
        CUTS += json.load(open(fn_)).get('cuts', [])
    ncut = 0
    # (CUTS_FIRM=1: each of the CUTS files' cuts held unless its BROKEN flag, priced W_FIRM -- held wherever a plan
    # holds them all, broken only where none does)
    FIRM = awx_settings.get('CUTS_FIRM') == '1'
    firm_broken = {}
    for cu_ in sorted({(c_['lane'], round(c_['u_lo'], 3), round(c_['u_hi'], 3)) for c_ in CUTS}):
        n_, lo_, hi_ = cu_
        brk_ = []
        if FIRM:
            firm_broken[('island',) + cu_] = m.NewBoolVar('')
            brk_ = [firm_broken[('island',) + cu_].Not()]
        for key in t:
            if n_ not in key:
                continue
            a_ = m.NewBoolVar('')
            m.Add(t[key] <= Q(lo_)).OnlyEnforceIf([a_] + brk_)
            m.Add(t[key] >= Q(hi_) + 1).OnlyEnforceIf([a_.Not()] + brk_)
            ncut += 1
    if CUTS:
        print(f'   geometry cuts: {len(CUTS)} island spans, {ncut} crossing constraints')
    VCUTS = []
    for fn_ in cuts:
        VCUTS += json.load(open(fn_)).get('vcuts', [])
    # ...and a pair's built-in via cuts, at its route's known turns: its trunk's spine corners, its ring's (the pad box's
    # corners) and the handoff from the one onto the other
    NVC0 = len(VCUTS)
    for n in prs:
        if n not in M:
            continue
        turns = [(s_, d_) for _i, s_, d_ in spine.corners(TURN_DEG) if entry[n] < s_ < tend[n]]
        if n in bname:
            sp_ = ring_of[bname[n]]
            turns += [(u_, d_) for _i, sb_, d_ in sp_.corners(TURN_DEG) for u_ in [u_ring(n, sb_)] if Hn[n] < u_ < end[n]]
            dh = _deg(route_dir(n, Hn[n] - G), route_dir(n, Hn[n] + G))
            if dh >= TURN_DEG and entry[n] < Hn[n] < end[n]:
                turns.append((Hn[n], dh))
        VCUTS += [{'lane': n, 'u': u_, 'w': LD(n) + turn_room(d_)} for u_, d_ in turns]
    if len(VCUTS) > NVC0:
        print(f'   built-in via cuts at the pairs\' known turns: {len(VCUTS) - NVC0}')
    # ...and at the OTHER PARTS' PADS on the trunk and the TEETH of the nets outside the bus (whole_ctx.foreign_teeth:
    # their stubs' ends on the arrays' box lines): a lane whose reference (its taut path, whole_frame.ref) passes within
    # a via's reach of one -- its half size, a via's copper and clearance, a pair's barrel offset, and a lane pitch the
    # geometry may move it -- keeps its changes off that stretch (K35: SDQS0's dive planned beside C5's pad, 0.127 from
    # it where 0.242 was asked, and the geometry folded the pair round it). A PAIR's stretch is longer by its dive's
    # straight run (L_DIVE) either side: its lane bends round the item, and the pair router neither turns at its via
    # nor within that run of one (K41: SDQS0's dive planned 0.6 past its tooth, where its lane bent round SDQ4's tooth
    # beside it, turned 75 degrees at the via)
    NVC1 = len(VCUTS)
    off_ = {n: (_pairs.dive_offset(ctx.cfg, _pairs.pitch(TRK) / 2) if n in prs else 0.0) for n in M}
    items = [((pd_.global_x, pd_.global_y), max(pd_.size_x, pd_.size_y) / 2)
             for ref_, fp_ in ctx.pcb.footprints.items() if ref_ not in (Fr.src, Fr.dst) for pd_ in fp_.pads
             if pd_.pad_type != 'np_thru_hole' and any(L.endswith('.Cu') for L in pd_.layers)]
    items += [((x_, y_), r_) for (x_, y_, r_, _L, _nm) in whole_ctx.foreign_teeth(ctx, M, (Fr.SB, Fr.DB))]
    for (px_, py_), rp_ in items:
        sp_, op_ = (float(v_) for v_ in spine.project_pt((px_, py_)))
        for n in M:
            w_ = rp_ + VIA / 2 + CLR + (LD(n) if n in prs else 0.0)
            if not (entry[n] < sp_ + w_ and sp_ - w_ < tend[n]):       # the stretch reaches into the trunk's route
                continue
            if abs(whole_frame.ref(Fr, n, sp_) - op_) < rp_ + VIA / 2 + CLR + off_[n] + PITCH:
                VCUTS.append({'lane': n, 'u': sp_, 'w': w_, 'built': 1})
    if len(VCUTS) > NVC1:
        print(f'   built-in via cuts at other parts\' pads and the teeth outside the bus: {len(VCUTS) - NVC1}')
    # ---- the braid rule over every triple
    nt = 0
    for i, j, k in itertools.combinations(Ln, 3):
        ij, ik, jk = (i, j) in t, (i, k) in t, (j, k) in t
        if ij and ik and jk:
            b1 = m.NewBoolVar('')
            m.Add(t[(i, j)] < t[(i, k)]).OnlyEnforceIf(b1); m.Add(t[(i, k)] < t[(j, k)]).OnlyEnforceIf(b1)
            m.Add(t[(j, k)] < t[(i, k)]).OnlyEnforceIf(b1.Not()); m.Add(t[(i, k)] < t[(i, j)]).OnlyEnforceIf(b1.Not())
            nt += 1
        elif ij and ik:
            m.Add(t[(i, j)] < t[(i, k)]); nt += 1
        elif ik and jk:
            m.Add(t[(j, k)] < t[(i, k)]); nt += 1
        elif ij and jk:
            raise SystemExit(f'inconsistent triple {i} {j} {k}')
    ev = collections.defaultdict(list)
    for key in t:
        for n in key: ev[n].append(key)
    P_STAY = PITCH                        # along a STAYER two crossings sit a pitch apart (its crossers are parallel there)
    MV = {}
    # (the mover / stayer model) every crossing has a MOVER (the steep lane, crossing over) and a STAYER; along a stayer
    # its crossings are P_STAY apart in s, along a mover PITCH / sqrt(1 + K_SWEEP^2) (a sweep): the mover chosen by the solve
    ivs_of = collections.defaultdict(list)
    w_move, w_stay = max(1, QU(PITCH / math.sqrt(1 + K_SWEEP * K_SWEEP))), max(1, QU(P_STAY))
    for key in t:
        a, b = key
        mv = m.NewBoolVar('')                           # True: a moves, b stays
        MV[key] = mv
        if NL > 2:
            continue                    # (spaced per the crosser's layer, below, where the runs' layers are known)
        ivs_of[a].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_move, present(mv, a), ''))
        ivs_of[a].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_stay, present(mv.Not(), a), ''))
        ivs_of[b].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_move, present(mv.Not(), b), ''))
        ivs_of[b].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_stay, present(mv, b), ''))
    for n, ivs in ivs_of.items():
        m.AddNoOverlap(ivs)
    # a PAIR does not ZIGZAG: two of its crossings that pass lanes in OPPOSITE directions (one taking it north of a lane,
    # the other south of one) stand a pair's turn apart -- two 45-degree bends, each with its straight run, and its own
    # width -- or the geometry folds it between them (K35: SDQS0 run north on B to cross some lanes and back south to
    # cross others, a V in half a millimetre)
    zq = QU(2 * _pairs.turn_straight_steps(ctx.cfg) * ctx.cfg.grid_step + _pairs.pitch(TRK))
    for n in M:
        if n not in prs:
            continue
        ks = [k for k in t if n in k]
        way = {k: (1 if li[n] > li[k[1] if k[0] == n else k[0]] else -1) for k in ks}
        for k1, k2 in itertools.combinations(ks, 2):
            if way[k1] != way[k2]:
                zb = m.NewBoolVar('')
                m.Add(t[k1] + zq <= t[k2]).OnlyEnforceIf(zb); m.Add(t[k2] + zq <= t[k1]).OnlyEnforceIf(zb.Not())
    # ---- layer changes
    cost = []
    chg, tot = {}, {}
    T0 = {c_[0] for c_ in LAYER_CUTS if c_[1] == 0} | CHAN_T0 | TVIA   # (a blocked tooth's change may stand in the face's band)
    for n in M:
        # (more layers than two: never before the lane's own entry -- a tooth via's window starts AT the tooth, no end
        # room, and rounded to the nearest step it opened half a step before it: the change laid ahead of the tooth,
        # the band TOOTH-OUT)
        lo_n = Q(max(entry[n] + VIN0[n], BAND if n not in T0 else entry[n] + VIN0[n]))
        if NL > 2:
            lo_n = max(lo_n, QU(entry[n]))
        hi_n = Q(end[n] - VIN1[n])
        # (a lane whose end rooms overlap has no room for a change: its changes stand inactive -- a domain from past its
        # end was no domain at all, and CP-SAT refused the whole model. The zynq LVDS on the human's ends: RX_D1, a pair
        # 1.14 mm of route u from its tooth to its berth, a dive room of 0.69 at each end)
        lo_n = min(lo_n, hi_n + 1)
        cs_ = [m.NewIntVar(lo_n, hi_n + 1, f'c_{n}_{k}') for k in range(KMAX)]
        act = [m.NewBoolVar('') for _ in range(KMAX)]
        for k in range(KMAX):
            m.Add(cs_[k] <= hi_n).OnlyEnforceIf(act[k]); m.Add(cs_[k] == hi_n + 1).OnlyEnforceIf(act[k].Not())
            if k:
                m.Add(cs_[k] >= cs_[k - 1] + Q(Dv[n])).OnlyEnforceIf(act[k]); m.AddImplication(act[k], act[k - 1])
        # a change's room from each of its lane's crossings, along s: a STAYER's via is passed by a steep mover at an
        # angle (VR_STAY x the room), a MOVER's via sits on its own steep track (the room itself) -- the room of both
        # lanes there, a PAIR crossing the via's lane the wider by its second leg: sized by the via's lane alone, SDQ13's
        # via stood 0.40 before SDQS1 swept across it, and the pair folded round it twice (K41)
        before = {}
        for key in ev[n]:
            wide = Dv[n] + (_pairs.pitch(TRK) if (key[1] if key[0] == n else key[0]) in prs else 0.0)
            h_st, h_mv = QU(VR_STAY * wide / 2), QU(wide / 2)               # a room rounds UP to the grid
            stay = MV[key].Not() if key[0] == n else MV[key]
            bits = []
            for k in range(KMAX):
                bb = m.NewBoolVar('')
                for h_, lit in ((h_st, stay), (h_mv, stay.Not())):
                    m.Add(cs_[k] + h_ <= t[key]).OnlyEnforceIf([bb, lit])
                    m.Add(cs_[k] - h_ >= t[key]).OnlyEnforceIf([bb.Not(), act[k], lit])
                m.AddImplication(bb, act[k]); bits.append(bb)
            before[key] = bits
        if NL > 2:
            # (more layers than two: run k's layer y[k], after the lane's k-th change -- a change goes to another
            # layer, an unused one keeps it; the first run on its tooth's layer, the last on its berth's -- a via
            # end's on any its run fits on)
            y_ = [m.NewIntVar(0, NL - 1, f'y_{n}_{k}') for k in range(KMAX + 1)]
            for y_e, L_e, k_e in ((y_[0], tli[n], 0), (y_[KMAX], dli[n], 1)):
                if (n, k_e) in VEND:
                    for L_b in sorted(VBAN.get((n, k_e), ())):
                        m.Add(y_e != LAYN.index(L_b))
                else:
                    m.Add(y_e == L_e)
            for k in range(KMAX):
                m.Add(y_[k + 1] != y_[k]).OnlyEnforceIf(act[k])
                m.Add(y_[k + 1] == y_[k]).OnlyEnforceIf(act[k].Not())
            YL[n] = y_
        else:
            m.AddBoolXOr(act + ([m.NewConstant(1)] if tl[n] == dl[n] else []))
        tot[n] = sum(act); chg[n] = (cs_, act)
        ev[n] = before
    # an OPPOSITE-HANDS pair (pairs.opposite_hands) swaps its legs at a dive, a crossover: it changes layer at least once.
    # On two layers, where its tooth and berth are on different layers the berth rule's parity already asks an odd
    # number; only where they share one could it plan none (and be laid uncrossed) -- the rule is added there alone, since
    # an added constraint that binds nothing still moves the solver to another of its equal optima. On more, a via end's
    # layer is the solve's: a pair from an F tooth to a via berth could end on F with no change at all, and no crossover
    # would be laid (whole_geo's XO_AT) -- the rule for every one
    for n in M:
        if n in prs and (NL > 2 or tl[n] == dl[n]) and _pairs.opposite_hands(ctx, n):
            m.Add(chg[n][1][0] == 1)
    # two lanes' changes apart along one frame far enough that two on NEIGHBOURING lanes -- a lane pitch across -- clear
    # the via-to-via rule as the geometry plans it (a grid step over it): a via apart along left them 0.36 where the rule
    # is 0.38, for the geometry to spread. Only lanes that can BE neighbours: adjacent at the launch or at the berths,
    # or crossing each other (adjacent where they cross). Vias on lanes far apart across a frame stand side by side,
    # as the human's do; held single file along the whole frame, K35's trunk (6.5 mm between the arrays, a change
    # every 0.28) could not hold its changes. The geometry keeps the real via-to-via rule between every two vias, and
    # sends back a via cut where it cannot
    from fab_tiers import min_via_center_distance
    _VV = min_via_center_distance(VIA, CLR, ctx.cfg.via_drill, getattr(ctx.cfg, 'hole_to_hole_clearance', 0.0) or 0.0) \
        + ctx.cfg.grid_step
    STAGGER = max(VIA, math.sqrt(max(_VV * _VV - PITCH * PITCH, 0.0)))
    fr_ivs = collections.defaultdict(lambda: collections.defaultdict(list))      # frame -> lane -> its changes there
    w_s = max(1, Q(STAGGER))
    # (more layers than two: two CROSSING lanes stand stacked where they cross, not a lane pitch across -- their
    # changes apart by the whole via-to-via rule)
    fr_vv = collections.defaultdict(lambda: collections.defaultdict(list))
    w_v = max(1, QU(_VV))
    # (a change in its blocked end's window, between the stub and the copper, is a dog-bone's via at the array's edge,
    # its neighbours' a ball pitch across: no stagger along)
    DOG = collections.defaultdict(list)
    for (n_, k_, _L, lo_u, hi_u, lo_c, hi_c, w0, w1) in LAYER_CUTS + [c_ for c_ in CHAN_CUTS if c_[7] is not None]:
        DOG[n_].append((Q(w0), Q(w1)))
    for n in sorted(TVIA):
        DOG[n].append((Q(entry[n] + VIN0[n]), Q(entry[n] + VIN0[n] + VNEED)))
    # (a berth via's window, the same at its end -- for a single lane whose site is measured clear, site_clear)
    for n in sorted(BVIA):
        DOG[n].append((Q(end[n] - VIN1[n] - VNEED), Q(end[n] - VIN1[n])))
    INWIN = {}                                          # (lane, change) -> it stands in its blocked end's window
    for n in M:
        cs_, act = chg[n]
        for i_x, (x, a_) in enumerate(zip(cs_, act)):
            if DOG.get(n):
                ins_ = []
                for lo_w, hi_w in DOG[n]:
                    w1, w2, w_ = m.NewBoolVar(''), m.NewBoolVar(''), m.NewBoolVar('')
                    m.Add(x >= lo_w).OnlyEnforceIf(w1); m.Add(x < lo_w).OnlyEnforceIf(w1.Not())
                    m.Add(x <= hi_w).OnlyEnforceIf(w2); m.Add(x > hi_w).OnlyEnforceIf(w2.Not())
                    m.AddBoolAnd([w1, w2]).OnlyEnforceIf(w_); m.AddBoolOr([w1.Not(), w2.Not(), w_])
                    ins_.append(w_)
                a2 = m.NewBoolVar('')                        # active and in no window: staggered
                m.AddImplication(a2, a_)
                for w_ in ins_:
                    m.AddImplication(a2, w_.Not())
                m.AddBoolOr([a_.Not(), a2] + ins_)
                INWIN[(n, i_x)] = ins_
                a_ = a2
            if n in bname:
                # its frame is the trunk before the handoff, its ring after
                inT, inR = m.NewBoolVar(''), m.NewBoolVar('')
                m.Add(x <= Q(Hn[n])).OnlyEnforceIf(inT); m.Add(x > Q(Hn[n])).OnlyEnforceIf(inR)
                m.AddBoolOr([inT.Not(), a_]); m.AddBoolOr([inR.Not(), a_])
                m.Add(inT + inR == 1).OnlyEnforceIf(a_); m.Add(inT + inR == 0).OnlyEnforceIf(a_.Not())
                fr_ivs['T'][n].append(m.NewOptionalFixedSizeIntervalVar(x, w_s, inT, ''))
                fr_ivs[bname[n]][n].append(m.NewOptionalFixedSizeIntervalVar(x, w_s, inR, ''))
                if NL > 2:
                    fr_vv['T'][n].append(m.NewOptionalFixedSizeIntervalVar(x, w_v, inT, ''))
                    fr_vv[bname[n]][n].append(m.NewOptionalFixedSizeIntervalVar(x, w_v, inR, ''))
            else:
                fr_ivs['T'][n].append(m.NewOptionalFixedSizeIntervalVar(x, w_s, a_, ''))
                if NL > 2:
                    fr_vv['T'][n].append(m.NewOptionalFixedSizeIntervalVar(x, w_v, a_, ''))
    nbr = {frozenset(p_) for p_ in zip(Ln, Ln[1:])} | {frozenset(p_) for p_ in zip(Fn, Fn[1:])} | \
        {frozenset(k) for k in t}
    crossing_ = {frozenset(k) for k in t}
    for f_, by_lane in fr_ivs.items():
        for p_ in sorted(nbr, key=lambda p_: sorted(p_)):
            a_, b_ = sorted(p_)
            src_ = fr_vv[f_] if NL > 2 and p_ in crossing_ else by_lane
            if src_.get(a_) and src_.get(b_):
                m.AddNoOverlap(src_[a_] + src_[b_])
    # a PAIR's tips stand together (whole_frame: no other lane's end between them on the pair's layer): where the solve
    # chooses the ends' layers (more routing layers than two, via ends), every lane with an end between a pair's two
    # tips is on another layer than the pair at that end -- the relayer then lays them so (on the zynq's U1 the solve
    # put SPI_DI's tooth on RX_D4's layer between its tips, and the frame refused the pair split)
    if NL > 2:
        end_y = lambda n, k_: YL[n][0] if k_ == 0 else YL[n][KMAX]
        for (n, k1_), (o_, k2_) in VSEP:
            m.Add(end_y(n, k1_) != end_y(o_, k2_))
        for (n, k_), between_ in sorted(Fr.tip_between.items()):
            for o_ in between_:
                if n in YL and o_ in YL:
                    m.Add(end_y(n, k_) != end_y(o_, k_))

    def run_at(n, before):
        """(more layers than two) the layer of lane n's run where `before` -- its changes before a point, in order --
        says it is: the run after the last of them"""
        i_ = m.NewIntVar(0, KMAX, '')
        m.Add(i_ == sum(before))
        L_ = m.NewIntVar(0, NL - 1, '')
        m.AddElement(i_, YL[n], L_)
        return L_
    # (more layers than two) a lane's crossings are spaced along it only among the crossers ON ONE LAYER: two lanes on
    # two other layers cross it at one point, stacked, as a human's families on B and In1 run over one another. Spaced
    # whatever the crossers' layers, as two layers need it (every crosser on the other one), the zynq LVDS bus's round 1
    # (215 crossings, U5 3.7 mm from U1) was infeasible on three layers and on four; spaced per layer, four layers
    # planned it at 28 vias. A change is a via, through every layer: its room from each crossing stands as before
    LS = collections.defaultdict(list)              # (lane, the crosser's layer) -> its crossings' rooms there
    # ...and STACKED crossers clear of each other's vias: a crosser's change is a through via, its room kept from its own
    # lane's crossings only -- a crosser on another layer stacked at the same point stood over it. A crosser with an
    # active change within two via rooms of its crossing takes the lane's whole stacking room there (a cumulative per
    # crossed lane, capacity NL - 1: a via-free crosser one unit, so up to NL - 1 stack; one with a via near, all)
    CU = collections.defaultdict(lambda: ([], []))   # crossed lane -> (its crossings' rooms, their demands)
    DQ = QU(2 * VNEED)

    def via_near(c_, key):
        nears = []
        for x_, a_ in zip(*chg[c_]):
            lo_b, hi_b, nb = m.NewBoolVar(''), m.NewBoolVar(''), m.NewBoolVar('')
            m.Add(x_ - t[key] <= DQ).OnlyEnforceIf(lo_b); m.Add(x_ - t[key] > DQ).OnlyEnforceIf(lo_b.Not())
            m.Add(t[key] - x_ <= DQ).OnlyEnforceIf(hi_b); m.Add(t[key] - x_ > DQ).OnlyEnforceIf(hi_b.Not())
            m.AddBoolAnd([a_, lo_b, hi_b]).OnlyEnforceIf(nb); m.AddBoolOr([a_.Not(), lo_b.Not(), hi_b.Not(), nb])
            nears.append(nb)
        vb = m.NewBoolVar('')
        m.AddMaxEquality(vb, nears)
        return vb
    for key in t:
        a, b = key
        if NL > 2:
            # crossing lanes DIFFER: the run each is on there (its changes before the crossing: they stand in order)
            La_, Lb_ = run_at(a, ev[a][key]), run_at(b, ev[b][key])
            m.Add(La_ != Lb_)
            for lane_, c_, mv_ in ((a, b, MV[key]), (b, a, MV[key].Not())):
                vb = via_near(c_, key)
                for w_, pres_ in ((w_move, mv_), (w_stay, mv_.Not())):
                    CU[lane_][0].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_, present(pres_, lane_), ''))
                    CU[lane_][1].append(1 + (NL - 2) * vb)
            for l_ in range(NL):
                for lane_, Lx_, mv_ in ((a, Lb_, MV[key]), (b, La_, MV[key].Not())):
                    on_l = m.NewBoolVar('')
                    m.Add(Lx_ == l_).OnlyEnforceIf(on_l); m.Add(Lx_ != l_).OnlyEnforceIf(on_l.Not())
                    for w_, pres_ in ((w_move, mv_), (w_stay, mv_.Not())):
                        p_ = m.NewBoolVar('')
                        m.AddBoolAnd([pres_, on_l]).OnlyEnforceIf(p_); m.AddBoolOr([pres_.Not(), on_l.Not(), p_])
                        LS[(lane_, l_)].append(m.NewOptionalFixedSizeIntervalVar(t[key], w_, present(p_, lane_), ''))
            continue
        # crossing lanes DIFFER: tl_a ^ tl_b ^ Ca ^ Cb == 1, i.e. XOR(parity bits [+ 1 when the teeth differ]) == 1
        lits = ev[a][key] + ev[b][key]
        if tl[a] ^ tl[b] == 1: lits = lits + [m.NewConstant(1)]
        m.AddBoolXOr(lits)
    for _k in sorted(LS):
        if len(LS[_k]) > 1:
            m.AddNoOverlap(LS[_k])
    for _k in sorted(CU):
        if len(CU[_k][0]) > 1:
            m.AddCumulative(CU[_k][0], CU[_k][1], NL - 1)
    # ---- via cuts (whole_geo.py: a change the geometry could not give its room): that lane's changes stay out of the window
    # (a BUILT-IN cut gives way in a lane's blocked end's window: the ends model measured a via there clear of every
    # copper on every layer, where the built-in cut only reads a pad within reach of the lane's reference path)
    filed = {(c_['lane'], round(c_['u'], 3), round(c_['w'], 3)) for c_ in VCUTS[:NVC0]}     # (the cut files')
    for vc_ in sorted({(c_['lane'], round(c_['u'], 3), round(c_['w'], 3), c_.get('built', 0)) for c_ in VCUTS}):
        n_, u_, w_, bi_ = vc_
        if n_ not in chg:
            continue
        brk_ = []
        if FIRM and not bi_ and vc_[:3] in filed:
            firm_broken[('via',) + vc_[:3]] = m.NewBoolVar('')
            brk_ = [firm_broken[('via',) + vc_[:3]]]
        cs_v, act_v = chg[n_]
        for i_x, (x_, a_) in enumerate(zip(cs_v, act_v)):
            lo_b, hi_b = m.NewBoolVar(''), m.NewBoolVar('')
            m.Add(x_ <= Q(u_ - w_)).OnlyEnforceIf(lo_b); m.Add(x_ >= Q(u_ + w_)).OnlyEnforceIf(hi_b)
            m.AddBoolOr([lo_b, hi_b, a_.Not()] + (INWIN.get((n_, i_x), []) if bi_ else []) + brk_)
    if VCUTS:
        print(f'   via cuts: {len(VCUTS)}')
    # ---- SOFT cuts (SOFT_CUTS=GEO.json,..: the geometry's cuts, as CUTS reads them, that left no plan as hard ones --
    # whole_route's fallback): each its own BROKEN flag, its constraints held while the flag is off, the flag priced
    # W_SOFT vias -- above a dive's two, so the solve still buys the vias to keep a lane off an island or a change out of
    # a window as a hard cut would, but takes the break where nothing else gives a plan; below a net over two (W_OVER)
    SOFT = []
    for fn_ in soft_cuts:
        j_ = json.load(open(fn_))
        SOFT += [('island', c_['lane'], round(c_['u_lo'], 3), round(c_['u_hi'], 3)) for c_ in j_.get('cuts', [])]
        SOFT += [('via', c_['lane'], round(c_['u'], 3), round(c_['w'], 3)) for c_ in j_.get('vcuts', [])]
    soft_broken = {}
    for sc_ in sorted(set(SOFT)):
        kind_, n_, p_, q_ = sc_
        br_ = m.NewBoolVar('')
        if kind_ == 'island':
            hit_ = False
            for key in t:
                if n_ not in key:
                    continue
                a_ = m.NewBoolVar('')
                m.Add(t[key] <= Q(p_)).OnlyEnforceIf([a_, br_.Not()])
                m.Add(t[key] >= Q(q_) + 1).OnlyEnforceIf([a_.Not(), br_.Not()])
                hit_ = True
        else:
            hit_ = n_ in chg
            if hit_:
                for x_, a_ in zip(*chg[n_]):
                    lo_b, hi_b = m.NewBoolVar(''), m.NewBoolVar('')
                    m.Add(x_ <= Q(p_ - q_)).OnlyEnforceIf(lo_b); m.Add(x_ >= Q(p_ + q_)).OnlyEnforceIf(hi_b)
                    m.AddBoolOr([lo_b, hi_b, a_.Not(), br_])
        if hit_:
            soft_broken[sc_] = br_
        else:
            m.Add(br_ == 0)
    if SOFT:
        print(f'   soft cuts: {len(soft_broken)} ({sum(1 for s_ in soft_broken if s_[0] == "island")} island, '
              f'{sum(1 for s_ in soft_broken if s_[0] == "via")} via), each {W_SOFT / W_V:g} vias when broken')
    # ---- LAYER cuts (the blocked fronts, above): no change of the lane inside the span, and its layer there -- its end's
    # layer flipped by the changes before the span -- the other; soft as the geometry's cuts are, priced W_SOFT broken
    for (n_, k_, L_, lo_u, hi_u, lo_c, hi_c, w0, w1) in LAYER_CUTS + CHAN_CUTS:
        cs_l, act_l = chg[n_]
        br_ = m.NewBoolVar('')
        bef = []
        for x_, a_ in zip(cs_l, act_l):
            lo_b, hi_b, b_ = m.NewBoolVar(''), m.NewBoolVar(''), m.NewBoolVar('')
            m.Add(x_ <= Q(lo_c)).OnlyEnforceIf(lo_b); m.Add(x_ >= QU(hi_c)).OnlyEnforceIf(hi_b)
            m.AddBoolOr([lo_b, hi_b, a_.Not(), br_])
            m.Add(x_ <= Q(lo_u)).OnlyEnforceIf(b_); m.Add(x_ > Q(lo_u)).OnlyEnforceIf(b_.Not())
            bef.append(b_)
        if NL > 2:
            # (more layers than two: off the BLOCKED layers across the span -- a front's, its end's own; an island's,
            # every layer it stands on -- on any other)
            blks_ = [(tli[n_] if k_ == 0 else dli[n_])] if k_ in (0, 1) else CHAN_BLK.get((n_, k_), [1 - L_])
            run_ = run_at(n_, bef)
            for blk_ in blks_:
                m.Add(run_ != blk_).OnlyEnforceIf(br_.Not())
            soft_broken[('layer', n_, str(k_), round(lo_u, 3))] = br_     # (k_ a front's end, or an island's name)
            continue
        par = m.NewBoolVar('')
        m.AddBoolXOr(bef + [par.Not()])                        # par: an odd number of changes before the span
        m.Add(par == (tl[n_] ^ L_)).OnlyEnforceIf(br_.Not())
        soft_broken[('layer', n_, str(k_), round(lo_u, 3))] = br_
    # (a cut's layer as the model holds it: on two layers the one the lane is held ON; on more, the one it is held OFF)
    held = lambda c_: (f'on {"FB"[c_[2]]}' if NL == 2 else
                       'off ' + ','.join(LAYN[b_] for b_ in ([tli[c_[0]] if c_[1] == 0 else dli[c_[0]]] if c_[1] in (0, 1)
                                                            else CHAN_BLK.get((c_[0], c_[1]), [1 - c_[2]]))))
    if LAYER_CUTS:
        print(f'   layer cuts (blocked fronts): {len(LAYER_CUTS)} -- ' + ', '.join(
            f'{c_[0]} {"tooth" if c_[1] == 0 else "berth"} {held(c_)} over {c_[3]:.2f}..{c_[4]:.2f}'
            for c_ in LAYER_CUTS))
    if CHAN_CUTS:
        print(f'   layer cuts (islands in the way): {len(CHAN_CUTS)} -- ' + ', '.join(
            f'{c_[0]} {held(c_)} under {c_[1][5:]} over {c_[3]:.2f}..{c_[4]:.2f}' for c_ in CHAN_CUTS))
    # ---- PART GATES (more routing layers than two, SOLVE_GATES, on there): a part whose pads stand on some routing
    # layers only is no wall the frame goes round (whole_frame._rings_round) -- the lanes pass it on the others -- so
    # across it the lanes ON its layers must fit in what those layers have free. Each such ISLAND (whole_ctx.part_islands:
    # pads no lane passes between, one box, as the geometry holds it), in each frame it stands in: the frame's columns at
    # its two edges and its middle, board edge to board edge (a ring's on its own side of the destination), less every
    # island and the arrays' boxes on the layer, each grown by a lane's bar, read as lane pitches; every lane of that
    # frame there, on that layer there, counts its pitch (a pair's, its legs' too). A lane over it is priced W_FIRM: the
    # geometry has nowhere to lay it. The frame lays its rings over such a part and the solve, blind to it, put every lane
    # on its layer there -- synth wwP's twelve lanes all on B under two walls of 0402s on B, a plan no geometry lays,
    # left to the loop to undo a cut at a time
    GATE_OVER = []
    if NL > 2 and (awx_settings.get('SOLVE_GATES') or '1') == '1':
        import numpy as np
        g2_ = ctx.cfg.grid_step / 2
        BAR_ = TRK / 2 + CLR + g2_                     # a lane's centreline off copper (whole_geo's LANE_ST)
        PMIN_ = max(PITCH, TRK + CLR + 2 * g2_)        # two lanes' centrelines apart (whole_geo's P_MIN)
        wid_ = {n: PMIN_ + (_pairs.pitch(TRK) if n in prs else 0.0) for n in M}
        LIDX_ = {L_: i_ for i_, L_ in enumerate(LAYN)}
        ALL_ = frozenset(range(NL))
        ISLB = {}                                      # (island, its layers) -> its box
        for (ref_, i_), lab_ in whole_ctx.part_islands(ctx, skip=(Fr.src, Fr.dst)).items():
            p_ = ctx.pcb.footprints[ref_].pads[i_]
            if p_.pad_type == 'np_thru_hole':
                hx_ = hy_ = (p_.drill or 0) / 2
                ls_ = ALL_
            else:
                hx_, hy_ = p_.size_x / 2, p_.size_y / 2
                ls_ = ALL_ if ((p_.drill or 0) > 0 or '*.Cu' in p_.layers) else \
                    frozenset(LIDX_[L_] for L_ in p_.layers if L_ in LIDX_)
            if not ls_:
                continue
            b_ = ISLB.setdefault((lab_, ls_), [math.inf, math.inf, -math.inf, -math.inf])
            b_[0], b_[1] = min(b_[0], p_.global_x - hx_), min(b_[1], p_.global_y - hy_)
            b_[2], b_[3] = max(b_[2], p_.global_x + hx_), max(b_[3], p_.global_y + hy_)
        BX0_, BY0_, BX1_, BY1_ = ctx.pcb.board_info.board_bounds
        EDGE_ = (float(getattr(ctx.cfg, 'board_edge_clearance', 0.0) or 0.0) or ctx.cfg.clearance) + TRK / 2
        SPAN_ = max(BX1_ - BX0_, BY1_ - BY0_)
        OS_ = np.arange(-SPAN_, SPAN_, PMIN_ / 8)
        dcen_ = ((Fr.DB[0] + Fr.DB[2]) / 2, (Fr.DB[1] + Fr.DB[3]) / 2)

        def column(sp_, s_):
            k_ = sp_.seg_of(s_)
            t_ = s_ - sp_.S[k_]
            return (sp_.P[k_, 0] + t_ * sp_.d[k_, 0] + OS_ * sp_.nrm[k_, 0],
                    sp_.P[k_, 1] + t_ * sp_.d[k_, 1] + OS_ * sp_.nrm[k_, 1])

        def free_room(sp_, s_, L_, ring_):
            """lane pitches layer L_ has free across frame sp_'s column at s_ (a ring's: outside the destination)"""
            X_, Y_ = column(sp_, s_)
            ok_ = (X_ >= BX0_ + EDGE_) & (X_ <= BX1_ - EDGE_) & (Y_ >= BY0_ + EDGE_) & (Y_ <= BY1_ - EDGE_)
            for b_ in [Fr.SB, Fr.DB] + [b2_ for (_l, ls2_), b2_ in ISLB.items() if L_ in ls2_]:
                ok_ &= ~((X_ > b_[0] - BAR_) & (X_ < b_[2] + BAR_) & (Y_ > b_[1] - BAR_) & (Y_ < b_[3] + BAR_))
            oc_ = float(sp_.project_pt(dcen_)[1]) if ring_ else None
            room_, i_ = 0.0, 0
            while i_ < len(OS_):
                if not ok_[i_]:
                    i_ += 1
                    continue
                j_ = i_
                while j_ + 1 < len(OS_) and ok_[j_ + 1]:
                    j_ += 1
                a_, b_ = float(OS_[i_]), float(OS_[j_])
                if not ring_ or ((a_ + b_) / 2 - oc_) * (0.0 - oc_) > 0:
                    room_ += b_ - a_ + PMIN_                  # (centrelines a pitch apart from a_ to b_)
                i_ = j_ + 1
            return room_
        frames_ = [('T', spine, None)] + [(k_, sp_, k_) for k_, sp_ in sorted(ring_of.items())]
        # (a lane held off the layer from a VIA'S ROOM before the island to one past it: its change is a via, standing
        # on every layer -- held only across the island's own span, a lane came back to B 0.03 mm past synth wwP's
        # wall of 0402s, its via and its track inside the wall's clearance, and the audit found it under the wall)
        VROOM_ = bd.VIA_SIZE / 2 + CLR + g2_
        seen_ = set()
        for (lab_, ls_), b_ in sorted(ISLB.items(), key=lambda kv: (kv[0][0], sorted(kv[0][1]))):
            if ls_ == ALL_:
                continue                                    # (a wall on every layer: the frame goes round it)
            cnr_ = ((b_[0], b_[1]), (b_[0], b_[3]), (b_[2], b_[1]), (b_[2], b_[3]))
            for fk_, sp_, ring_ in frames_:
                ss_ = [float(sp_.project_pt(q_)[0]) for q_ in cnr_]
                s0_, s1_ = max(min(ss_), 0.0), min(max(ss_), sp_.L)
                if s1_ < s0_:
                    continue
                # the island's room on each of its layers: the least across its span
                rooms_ = {L_: min(free_room(sp_, s_, L_, ring_) for s_ in (s0_, (s0_ + s1_) / 2, s1_)) for L_ in ls_}
                for s_ in sorted({round(max(s0_ - VROOM_, 0.0), 3), round((s0_ + s1_) / 2, 3),
                                  round(min(s1_ + VROOM_, sp_.L), 3)}):
                    if ring_ is None:
                        pres_ = [(n, s_) for n in M if entry[n] < s_ < tend[n]]
                    else:
                        pres_ = [(n, u_ring(n, s_)) for n in M if bname.get(n) == ring_
                                 and Hn[n] < u_ring(n, s_) < end[n]]
                    if not pres_:
                        continue
                    need_ = sum(wid_[n] for n, _u in pres_)
                    for L_ in sorted(ls_):
                        if (fk_, lab_, L_, s_) in seen_:
                            continue
                        seen_.add((fk_, lab_, L_, s_))
                        room_ = rooms_[L_]
                        if need_ <= room_:
                            continue                        # (it cannot bind: no row)
                        terms_ = []
                        for n, u_ in pres_:
                            bef_ = []
                            for x_, a_ in zip(*chg[n]):
                                c_ = m.NewBoolVar('')
                                m.Add(x_ <= Q(u_)).OnlyEnforceIf(c_); m.Add(x_ > Q(u_)).OnlyEnforceIf(c_.Not())
                                bef_.append(c_)
                            on_ = m.NewBoolVar('')
                            r_ = run_at(n, bef_)
                            m.Add(r_ == L_).OnlyEnforceIf(on_); m.Add(r_ != L_).OnlyEnforceIf(on_.Not())
                            terms_.append((int(round(wid_[n] * 1000)), on_))
                        ov_ = m.NewIntVar(0, 4 * len(pres_), '')
                        m.Add(sum(c_ * v_ for c_, v_ in terms_) <= int(round(room_ * 1000)) + int(round(PMIN_ * 1000)) * ov_)
                        GATE_OVER.append(((fk_, lab_, LAYN[L_], round(s_, 2), round(room_ / PMIN_, 1),
                                           [n for n, _u in pres_]), ov_))
        if GATE_OVER:
            print(f'   part gates: {len(GATE_OVER)} -- ' + ', '.join(
                f'{k_[1][:28]} {k_[2]} {k_[0]} s {k_[3]:.2f}: room {k_[4]:.1f} for {len(k_[5])}' for k_, _v in GATE_OVER[:10])
                + (' ...' if len(GATE_OVER) > 10 else ''))
    # ---- HISTORY congestion (negotiated, as PathFinder prices a resource that was overused before): HIST=HOT.json,.. are
    # the audits' findings of earlier rounds (whole_gate --hot: where a plan was short -- a dive, a pitch, a static, a
    # shape), one file per audit. A finding marks the bins of route within a via's room of it, on the frame whose spine is
    # nearest; a bin's price is the number of audits it was hot in. Only those bins carry terms: a crossing in one pays a
    # crossing's copper area (a pitch across, a pitch along), a change a via's patch (a pair's two) -- after the vias and
    # the nets over two, so it moves crossings and changes out of the places the plan could not fit, never adds a via
    LB = 2 * PITCH                         # a history bin: two lane pitches of route
    R_HOT = 2 * VNEED                      # a finding marks the bins within a via's room of it
    SC = 1000.0                            # objective units per mm^2 of copper area
    A_V, A_X = 2 * (2 * VNEED) * (2 * VNEED), PITCH * PITCH
    # a frame's bins from its own origin: the trunk's from H0, a ring's from where it leaves the trunk (Hk)
    origin = lambda fr: H0 if fr == 'T' else Hk[fr]
    kof = lambda fr, u: int(math.floor((u - origin(fr)) / LB + 1e-9))
    HOT = collections.Counter()
    # (more layers than two: the lanes each bin's findings named -- every finding's, and a DIVE's, whose vias stand on
    # every layer -- for the terms below: two lanes stacked on other layers are not the ones a finding was about)
    HLANE, HDIVE = collections.defaultdict(set), collections.defaultdict(set)
    HFILES = list(hist)
    for fn_ in HFILES:
        marked = set()
        for x_, y_, *_k in json.load(open(fn_)).get('hot', []):
            kind_ = _k[0] if _k else ''
            lanes_ = set(_k[1]) if len(_k) > 1 and isinstance(_k[1], list) else set()
            # the frame whose spine the finding stands nearest -- a ring's only past where its lanes leave the trunk (a
            # ring's spine runs back along the trunk before that, and a finding at the source's face, nearer it than the
            # trunk's, was priced on the ring before any of its lanes is on it: nothing, K41's SDQS0 dive)
            fr_u = [('T',) + tuple(spine.project_pt((x_, y_)))]
            fr_u += [e_ for k_ in ring_of for e_ in [(lambda so: (k_, Hk[k_] + so[0] - rs[k_], so[1]))(
                ring_of[k_].project_pt((x_, y_)))] if e_[1] >= Hk[k_]]
            fr, u, _o = min(fr_u, key=lambda e_: abs(e_[2]))
            bins_ = {(fr, k) for k in range(kof(fr, u - R_HOT), kof(fr, u + R_HOT) + 1)}
            marked |= bins_
            for b_ in bins_:
                HLANE[b_] |= lanes_
                if kind_ == 'DIVE':
                    HDIVE[b_] |= lanes_
        HOT.update(marked)


    def in_bin(v_, u0, u1, lit=None):
        """a bool that is 1 whenever v_ lies in [u0, u1) (and lit holds): it carries a positive price, so it is 1 only then"""
        x_, lo_b, hi_b = m.NewBoolVar(''), m.NewBoolVar(''), m.NewBoolVar('')
        m.Add(v_ >= Q(u0)).OnlyEnforceIf(lo_b); m.Add(v_ < Q(u0)).OnlyEnforceIf(lo_b.Not())
        m.Add(v_ < Q(u1)).OnlyEnforceIf(hi_b); m.Add(v_ >= Q(u1)).OnlyEnforceIf(hi_b.Not())
        m.AddBoolOr([lo_b.Not(), hi_b.Not(), x_] + ([lit.Not()] if lit is not None else []))
        return x_


    def in_frame(fr, n, u0, u1):
        """the part of frame fr's bin [u0, u1) where lane n's route is in that frame -- a ring lane's is its ring's past
        where it leaves the trunk (Hn), the trunk's before it -- or None"""
        if fr == 'T':
            lo_, hi_ = u0, (min(u1, Hn[n]) if n in bname else u1)
        elif bname.get(n) == fr:
            lo_, hi_ = max(u0, Hn[n]), u1
        else:
            return None
        return (lo_, hi_) if lo_ < hi_ else None

    nh_x = nh_v = 0
    # (more routing layers than two) a RUN of neighbouring bins of one price priced ONCE: a crossing or a change lies in
    # one bin at most, so the run's one term is the same price wherever in it it lies -- the same objective. Bin by bin,
    # a finding marking every bin within a via's room of it, zynq's LVDS bus on four layers had 5053 terms, the model
    # 28986 variables against 12588 without them, and its plan-finding workers past 1.5 GB; on two the bins are priced
    # one by one, as every ladder was measured
    runs = [((fr, k), h_, 1) for (fr, k), h_ in sorted(HOT.items(), key=lambda e_: (str(e_[0][0]), e_[0][1]))]
    if NL > 2:
        # (one price AND the same lanes named: the run's one term the bins' own)
        merged = []
        for (fr, k), h_, w_ in runs:
            p_ = merged[-1] if merged else None
            if p_ and p_[0][0] == fr and p_[1] == h_ and p_[0][1] + p_[2] == k \
                    and HLANE[(fr, k)] == HLANE[p_[0]] and HDIVE[(fr, k)] == HDIVE[p_[0]]:
                merged[-1] = (p_[0], h_, p_[2] + 1)
            else:
                merged.append(((fr, k), h_, w_))
        runs = merged
    for (fr, k), h_, w_ in runs:
        u0, u1 = origin(fr) + k * LB, origin(fr) + (k + w_) * LB
        named, dived = (HLANE[(fr, k)], HDIVE[(fr, k)]) if NL > 2 else (set(), set())
        for key, (lo, hi) in win.items():
            a, b = key
            # (a crossing is on a ring only between two lanes of that ring)
            sp_ = in_frame(fr, a, u0, u1) if same(a, b) else ((u0, u1) if fr == 'T' else None)
            if sp_ is None or hi < sp_[0] or lo >= sp_[1] or (named and not ({a, b} & named)):
                continue
            cost.append(int(round(SC * A_X * h_)) * in_bin(t[key], *sp_)); nh_x += 1
        for n in M:
            sp_ = in_frame(fr, n, u0, u1)
            if sp_ is None or end[n] < sp_[0] or entry[n] >= sp_[1] or (named and n not in (dived or named)):
                continue
            cs_, act = chg[n]
            for cv_, a_ in zip(cs_, act):
                cost.append(int(round(SC * A_V * (2 if n in prs else 1) * h_)) * in_bin(cv_, *sp_, a_)); nh_v += 1
    if HFILES:
        print(f'   history: {len(HFILES)} audit(s), {len(HOT)} hot bin(s) (hottest {max(HOT.values(), default=0)})'
              + (f' in {len(runs)} run(s)' if len(runs) != len(HOT) else '') + f', {nh_x} crossing and {nh_v} change terms')
    if hint:
        Hj = json.load(open(hint))
        nh = 0
        for k_, v_ in Hj['cross'].items():
            a_, b_ = k_.split('|')
            key_ = (a_, b_) if (a_, b_) in t else (b_, a_)
            if key_ in t:
                m.AddHint(t[key_], int(round(v_['u'] / G))); nh += 1
        for n_, chs_ in Hj['changes'].items():
            cs_h, act_h = chg[n_]
            for i_ in range(KMAX):
                if i_ < len(chs_):
                    m.AddHint(cs_h[i_], int(round(chs_[i_] / G))); m.AddHint(act_h[i_], 1)
                else:
                    m.AddHint(act_h[i_], 0)
        for n_, ls_ in (Hj.get('layers') or {}).items():
            if n_ in YL and all(L_ in LAYN for L_ in ls_):
                for i_ in range(KMAX + 1):
                    m.AddHint(YL[n_][i_], LAYN.index(ls_[min(i_, len(ls_) - 1)]))
        print(f'   warm start from {os.path.basename(hint)}: {nh} crossing hints')
    # ---- no more than TWO VIAS on a net where that can be had (Andy, 2026-09-25): a net's vias on the board are its stubs'
    # own (the bench's copper) and its lane's changes -- a pair's leg a barrel at each dive. Each via past two costs
    # W_OVER_X vias more, then the vias, then congestion -- a price, never a cap, so a board that cannot keep it still
    # plans. A TIE via (a ball's via to a pad of its own under it, laid by the fanout) is not counted toward the two:
    # it serves that pad, not the lane -- per leg, as the ends model counts it, and only where the board has one
    VIA_PREF = 2

    def _leg_vias(leg):
        nid = ctx.byname[leg][0]
        vs = [v for v in ctx.base_vias if v.net_id == nid]
        pads = ctx.pcb.nets[nid].pads
        ties = [d_ for d_ in pads if d_.component_ref == dest and any(_pairs.under_pad(d_, q_, bd.VIA_SIZE) for q_ in pads)]
        tie = any(abs(v.x - d_.global_x) <= d_.size_x / 2 and abs(v.y - d_.global_y) <= d_.size_y / 2
                  for v in vs for d_ in ties)
        return len(vs) - int(tie), tie
    LV = {leg: _leg_vias(leg) for n in M for leg in (prs[n] if n in prs else (n,))}
    SV = {n: max(LV[leg][0] for leg in (prs[n] if n in prs else (n,))) for n in M}
    if any(t_ for _v, t_ in LV.values()):
        print(f"   tie vias, not counted toward two: {sorted(leg for leg, (_v, t_) in LV.items() if t_)}")
    # ...less the via the relayer DROPS (more layers than two): a via end planned on its NECK's own layer -- the layer of
    # the ball's pad its via joins, a dog-bone's neck or the pad a via-in-pad stands in -- has all its copper at the via
    # on one layer, and the via joins nothing (relayer.relayer). The solve chose it: a via saved, on the board and
    # toward the two
    DROP = collections.defaultdict(list)
    for (n, k_) in sorted(VEND):
        necks = set()
        for leg in (prs[n] if n in prs else (n,)):
            ref_ = ctx.src_ref.get(leg) if k_ == 0 else dest
            cu_ = {L for p_ in ctx.pcb.nets[ctx.byname[leg][0]].pads if p_.component_ref == ref_
                   for L in p_.layers if L in LAYN}
            necks |= cu_ if len(cu_) == 1 else {None}
        if len(necks) == 1 and None not in necks:
            y_e, Ln_ = (YL[n][0] if k_ == 0 else YL[n][KMAX]), LAYN.index(next(iter(necks)))
            d_ = m.NewBoolVar('')
            m.Add(y_e == Ln_).OnlyEnforceIf(d_); m.Add(y_e != Ln_).OnlyEnforceIf(d_.Not())
            DROP[n].append(d_)
    dr = {n: sum(DROP[n]) for n in M}
    over = {n: m.NewIntVar(0, KMAX + SV[n], f'over_{n}') for n in M}
    for n in M:
        m.Add(over[n] >= SV[n] - dr[n] + tot[n] - VIA_PREF)
    VIAS = sum(tot.values()) - sum(dr.values()) if DROP else sum(tot.values())   # the route's vias, less those dropped
    W_OVER = W_V * W_OVER_X
    # ---- the ROOT's proof, a floor for every re-solve of the bench: the first solve (no geometry cuts) proved the least
    # nets over two and vias there are; a re-solve only adds cuts to it (a flip drops only an island cut a re-solve
    # added) and history below a via, so it can do no better -- and a plan it finds AT the root's is proved at once. A
    # re-solve left short of that proof ran out its budget on the proof alone (K51: best 1 over two and 42 vias, the
    # root's own, bound 36, 193 s unproved, and the loop stopped). The root is carried in each solve's JSON and read from
    # the warm start's, and taken only for the same model: its lanes, their orders, end layers, stub vias, KMAX and the
    # built-in cuts (the bench's own), by a signature
    sig = hashlib.sha1(json.dumps([sorted(M), list(Ln), list(Fn), [(n, tl[n], dl[n], SV[n]) for n in sorted(M)], KMAX,
                                   VIA_PREF, [(c_['lane'], round(c_['u'], 4), round(c_['w'], 4)) for c_ in VCUTS[NVC0:]]]
                                  + ([list(LAYN), [(n, tli[n], dli[n]) for n in sorted(M)],
                                      # (the via ends' model: which ends, their banned layers, the runs kept apart, the
                                      # tooth vias -- a floor proved on another is none here)
                                      sorted(VEND), sorted((list(k_), sorted(v_)) for k_, v_ in VBAN.items()),
                                      sorted(list(map(list, p_)) for p_ in VSEP), sorted(TVIA), sorted(BVIA)] if NL > 2 else []),
                                  sort_keys=True).encode()).hexdigest()
    root = None
    if hint:
        r_ = json.load(open(hint)).get('root')
        if r_ and r_.get('sig') == sig:
            root = r_
            m.Add(W_OVER * sum(over.values()) + W_V * VIAS >= W_OVER * r_['over'] + W_V * r_['vias'])
            print(f"   the root's proof as a floor: {r_['over']} via(s) over two, {r_['vias']} vias")
    OBJ = W_OVER * sum(over.values()) + W_V * VIAS + W_SOFT * sum(soft_broken.values()) + sum(cost) \
        + W_FIRM * sum(firm_broken.values()) + W_FIRM * sum(v_ for _k, v_ in GATE_OVER)
    if CROWD:
        # (the diagnosis asks only for the fewest crowded lanes -- a whole via each, so the search stops, proved, as
        # soon as their count is)
        OBJ = W_V * sum(CROWD.values())
    m.Minimize(OBJ)
    # ...and STOPPED when it STALLS: once it has a plan, SOLVE_STALL of the search's own model reductions in a row with
    # no better plan and no better bound (its log's '#Model' against '#n' and '#Bound' lines, events of the
    # deterministic search, never a clock). K35: its one plan at 46 s, then 145 s with neither -- the budget spent on
    # nothing. Before its first plan the budget decides: K41's re-solve, stopped at 31 s with none, proved a plan of
    # 33 vias by 255 s. And DONE once the vias
    # are PROVED, read off the same lines (best, and the bound below it): the plan's vias no more than the bound's
    # whole vias -- the nets over two and the vias settled, only the history's tie-break open (K35's re-solve: 167 s,
    # most of it on the tie-break). (A gap limit of a via would stop short of that proof: a plan of 24 vias against a
    # bound of 23 and half a via's history is within one via, and a 23-via plan may still exist.)
    def run(subs):
        s_ = cp_model.CpSolver()
        s_.parameters.num_workers = SOLVE_WORKERS
        s_.parameters.subsolvers.extend(subs)
        # REPRODUCIBLE: stopped by a count of interleaved batches, the workers sharing no clauses. Measured on this
        # model (OR-tools 9.15): bounded by deterministic time, four solves of one model gave four answers (36051281 ..
        # 36051642); by batches with clause sharing on, two gave two; by batches with sharing off, two concurrent
        # solves agree exactly
        s_.parameters.interleave_search = True
        s_.parameters.max_num_deterministic_batches = SOLVE_BATCHES
        s_.parameters.share_glue_clauses = False
        s_.parameters.share_binary_clauses = False
        stall = {'n': 0, 'plan': False}

        def _progress(line):
            if line.startswith('#Bound') or re.match(r'#\d+\s', line):
                stall['n'] = 0
                stall['plan'] = stall['plan'] or not line.startswith('#Bound')
                m_ = re.search(r'best:(\S+)\s+next:\[([^,\]]+)', line)
                if m_ and float(m_.group(1)) < math.inf and \
                        math.floor(float(m_.group(1)) / W_V) <= math.floor(float(m_.group(2)) / W_V):
                    s_.StopSearch()
            # (not the FALLBACK's: started from the first run's plan, it has one from its first line, and a stall there is
            # the last chance lost -- K51 on the human's fanout on Linux stopped so, unproved, holding the plan it proves
            # optimal when it runs on, and the bench had no plan)
            elif line.startswith('#Model') and stall['plan'] and subs is not FALLBACK:
                stall['n'] += 1
                if stall['n'] >= SOLVE_STALL:
                    s_.StopSearch()
        s_.parameters.log_search_progress = True
        s_.parameters.log_to_stdout = False
        s_.log_callback = _progress
        st_ = s_.Solve(m)
        pr_ = st_ in (cp_model.OPTIMAL, cp_model.FEASIBLE) and \
            math.floor(s_.ObjectiveValue() / W_V) <= math.floor(s_.BestObjectiveBound() / W_V)
        return s_, st_, pr_
    # ---- the WARM START, laid out whole: a re-solve starts from the round before's plan, and when that plan is one of
    # this model (no cut the round added excludes it) at the root's floor, it is PROVED -- the floor is the root's own
    # proof. The search is given it as a hint and may still lose it (K41 with the stub check per layer, a round whose
    # solve heard of the geometry by its history alone: its warm start at the floor, the search's best eleven nets
    # over two, no plan). Laid out here -- every hinted value held, the rest completed by one worker -- it is kept,
    # and taken only when the search proves no plan of its own: where the search proves one, nothing changes
    warm = None
    if hint and root is not None:
        sw = cp_model.CpSolver()
        sw.parameters.num_workers = 1
        sw.parameters.fix_variables_to_their_hinted_value = True
        sw.parameters.max_deterministic_time = WARM_WORK
        if sw.Solve(m) in (cp_model.OPTIMAL, cp_model.FEASIBLE) and \
                math.floor(sw.ObjectiveValue() / W_V) <= (W_OVER * root['over'] + W_V * root['vias']) // W_V:
            warm = sw
    sv, st, proved = run(SUBSOLVERS)
    if not proved and warm is not None:
        print(f'   bound-raising workers: [{sv.StatusName(st)}] {sv.WallTime():.0f}s, no proved plan -- the warm start, '
              f'at the root\'s floor ({warm.ObjectiveValue():.0f})')
        sv, st, proved = warm, cp_model.FEASIBLE, True
    if not proved:
        # the FALLBACK: the bound-raising workers could not prove it (the plan they found was not good enough to meet
        # their bound: K51 on the human's fanout, best 2 over two + 38 vias against 0 + 32, 124 s). The PLAN-FINDING
        # workers run on from there -- its best plan as their start, and its bound, proved on this very model, a
        # constraint (no worker proves it again)
        print(f'   bound-raising workers: [{sv.StatusName(st)}] {sv.WallTime():.0f}s' +
              (f', best {sv.ObjectiveValue():.0f}' if st in (cp_model.OPTIMAL, cp_model.FEASIBLE) else ', no plan') +
              f', bound {sv.BestObjectiveBound():.0f} -- the plan-finding workers on from there')
        if st in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            m.ClearHints()
            for i_ in range(len(m.Proto().variables)):
                v_ = m.GetIntVarFromProtoIndex(i_)
                m.AddHint(v_, sv.Value(v_))
        m.Add(OBJ >= int(math.ceil(sv.BestObjectiveBound() - 1e-6)))
        t_first = sv.WallTime()
        sv, st, proved = run(FALLBACK)
        print(f'   plan-finding workers: [{sv.StatusName(st)}] {sv.WallTime():.0f}s (after {t_first:.0f}s)')
    print(f'whole_solve: {len(t)} crossings ({sum(1 for k in t if same(*k))} same-branch), {nt} triples, K<={KMAX}, '
          f'mover/stayer (stay {P_STAY}), stagger {STAGGER:.3f}, MARG {MARG}: [{sv.StatusName(st)}] {sv.WallTime():.0f}s', end=' ')
    # only a plan PROVED optimal in its vias -- the nets over two and the vias, whole multiples of W_V; the history
    # terms below one are congestion's tie-break -- goes on to the geometry: one the search could not prove is a plan
    # whose ends it found hard, and the geometry would be laid on a guess (K35's round 2: best and bound a thousandth
    # of a via apart, refused for the tie-break alone)
    # ...except where the caller takes an UNPROVED plan (SOLVE_UNPROVED=1: whole_route's first solve of a round, which
    # never ends with nothing -- the round lays it, and the nets it leaves over two go back to the ends, which had
    # counted on keeping them at two: K51's ends promised 0 over two, the search held 1 over and 51 vias against a
    # bound of 0 and 45, and the round stopped with nothing). It is marked so, and no later solve takes it as a floor
    STATUS['plan'] = st in (cp_model.OPTIMAL, cp_model.FEASIBLE)
    keep = not proved and st == cp_model.FEASIBLE and (awx_settings.get('SOLVE_UNPROVED') == '1' or bool(CROWD))
    if not proved and not keep:
        print(('(not proved optimal: best ' + f'{sv.ObjectiveValue():.0f}, bound {sv.BestObjectiveBound():.0f} -- no plan)')
              if st == cp_model.FEASIBLE else '')
        return None
    if keep:
        print(f'(not proved optimal: best {sv.ObjectiveValue():.0f}, bound {sv.BestObjectiveBound():.0f} -- kept UNPROVED) ',
              end='')
    per = {n: int(sv.Value(tot[n])) for n in M}
    drp = {n: int(sum(sv.Value(d_) for d_ in DROP[n])) for n in M}
    ov = [n for n in M if SV[n] - drp[n] + per[n] > VIA_PREF]
    print(f'vias {sum(per.values())}' + (f' (and {sum(drp.values())} via end(s) dropped)' if any(drp.values()) else '')
          + f', nets over {VIA_PREF} vias on the board: {len(ov)} {ov}, changes per lane {dict(sorted(collections.Counter(per.values()).items()))}, obj {sv.ObjectiveValue():.0f} bound {sv.BestObjectiveBound():.0f}')
    # verify: every crossing on two layers, every lane on its berth layer
    lay = lambda n, u: tl[n] ^ (sum(1 for x, a_ in zip(*chg[n]) if sv.Value(a_) and sv.Value(x) * G < u) & 1)
    if NL > 2:
        lay = lambda n, u: sv.Value(YL[n][sum(1 for x, a_ in zip(*chg[n]) if sv.Value(a_) and sv.Value(x) * G < u)])
    bad = [(a, b) for (a, b), v in t.items() if lay(a, sv.Value(v) * G) == lay(b, sv.Value(v) * G)]
    badb = [n for n in M if lay(n, 1e9) != dli[n] and (n, 1) not in VEND]
    print(f'   check: crossings on one layer {len(bad)}, lanes off their berth layer {len(badb)}')
    J = {'H0': H0, 'Hk': Hk, 'rs': rs, 'cross': {}, 'changes': {}, 'entry': entry, 'end': end, 'tend': tend, 'branch': bname,
         'final': Fn, 'launch': Ln}
    for (a, b), v in t.items():
        J['cross'][f'{a}|{b}'] = {'u': sv.Value(v) * G, 'ring': same(a, b) and sv.Value(v) * G > Hn[a] + 1e-9}
    for n in M:
        cs_, act = chg[n]
        J['changes'][n] = [sv.Value(x) * G for x, a_ in zip(cs_, act) if sv.Value(a_)]
        if NL > 2:
            # (more layers than two: each run's layer, one more than its changes -- on two they alternate from its
            # tooth's)
            J.setdefault('layers', {})[n] = [LAYN[sv.Value(YL[n][k])] for k in range(len(J['changes'][n]) + 1)]
    # (the root: this solve's own proof when it has no geometry cuts, else the one it was floored by -- never an
    # unproved plan's, which proves nothing)
    J['root'] = root if root is not None else \
        ({'over': int(sum(sv.Value(over[n]) for n in M)), 'vias': int(sum(per.values()) - sum(drp.values())), 'sig': sig}
         if not CUTS and not SOFT and NVC0 == 0 and proved else None)
    J['proved'] = bool(proved)
    if CROWD:
        J['crowded'] = [n for n in M if sv.Value(CROWD[n])]
        print(f"   crowded: {len(J['crowded'])} lane(s) whose crossings break their room -- {', '.join(J['crowded'])}")
    if GATE_OVER:
        J['gate_over'] = [[k_[0], k_[1], k_[2], k_[3], int(sv.Value(v_))] for k_, v_ in GATE_OVER if sv.Value(v_)]
        print(f"   part gates over: {len(J['gate_over'])} of {len(GATE_OVER)}"
              + (f" -- {', '.join(f'{g_[1][:28]} {g_[2]} {g_[0]} s {g_[3]} +{g_[4]}' for g_ in J['gate_over'])}"
                 if J['gate_over'] else ''))
    if firm_broken:
        J['firm_broken'] = [list(s_) for s_, b_ in sorted(firm_broken.items()) if sv.Value(b_)]
        print(f"   firm cuts broken: {len(J['firm_broken'])} of {len(firm_broken)}"
              + (f" -- {', '.join(f'{s_[1]} {s_[0]} {s_[2]}' for s_ in J['firm_broken'])}" if J['firm_broken'] else ''))
    if soft_broken:
        J['soft_broken'] = [list(s_) for s_, b_ in sorted(soft_broken.items()) if sv.Value(b_)]
        print(f"   soft cuts broken: {len(J['soft_broken'])} of {len(soft_broken)}")
    # each lane over two vias on the board, by how many: an unproved plan's go back to the ends (whole_route)
    J['over_nets'] = {n: SV[n] - drp[n] + per[n] - VIA_PREF for n in ov}
    if not proved:
        J['unproved'] = {'best': sv.ObjectiveValue(), 'bound': sv.BestObjectiveBound(),
                         'over': int(sum(sv.Value(over[n]) for n in M)),
                         'bound_over': int(math.floor(sv.BestObjectiveBound() / W_OVER))}
    return J


def main():
    files = lambda var: [x for x in awx_settings.get(var, '').split(',') if x]
    out = sys.argv[1] if len(sys.argv) > 1 else '/dev/null'
    ctx, _cs = whole_ctx.plan()
    J = solve(ctx, awx_settings.req('DEST'), files('CUTS'), files('HIST'), awx_settings.get('HINT') or None,
              soft_cuts=files('SOFT_CUTS'))
    if J is None:
        # (SOLVE_CROWD=1, a round's first solve in whole_route: a model with NO plan is solved again as its CROWD
        # diagnosis, and the lanes it names -- whose crossings the ends leave no room for -- go to OUT.crowded.json,
        # which the round's log reads: zynq LVDS round 1's TX_D4 and DATA_CLK, round 3's DATA_CLK)
        if awx_settings.get('SOLVE_CROWD') == '1' and STATUS.get('plan') is False and out.endswith('.json'):
            ctx, _cs = whole_ctx.plan()
            Jc = solve(ctx, awx_settings.req('DEST'), files('CUTS'), files('HIST'), None,
                       soft_cuts=files('SOFT_CUTS'), crowd=True)
            if Jc is not None and Jc.get('crowded'):
                json.dump({'crowded': Jc['crowded']}, open(out[:-len('.json')] + '.crowded.json', 'w'), indent=0)
        sys.exit(1)
    json.dump(J, open(out, 'w'), indent=0)


if __name__ == '__main__':
    main()
