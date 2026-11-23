#!/usr/bin/env python3
"""group_ends.py CAPTURES OUT --board=BOARD [--from=A_RUN_S_OUT [--feedback=ITS_BUSES_OUT]] [--log] [--verify] [--full] --
THE GROUP'S ENDS: the ends of every bus off
one shared array chosen in ONE model -- the shared array's fanout and every bus's berths together -- every ball of an
array offered the same choices.

AN ARRAY'S MENU is one list, on the bare board (joint_escape.build_menus, the whole-array fanout's own, every
face): each ball's escapes from every via site it could take -- its four gap points, its pad, none -- by every face of
the array, a via move's run on no one layer yet (joint_escape.RUN: the layers it clears are the move's `runs_on`, and
which of them it takes is the model's choice). The shared array's is one such list, and so is each destination's that
is a ball array. No bus's fanout mode builds it, and no bus's own run does: what one ball is offered, every ball is.

THE PATTERN is what each ball is offered of it first, and all that is built of it at first (build_menus `pattern`): a
dog-bone to the gap point toward its nearest corner and a run straight out along its own row or column, by whichever
face; a surface escape from the two outer rows -- a handful of moves a ball, the fanout a hand lays. A ball is offered
its WHOLE menu (its other sites, its side-steps), built then (widen), only where the pattern meets a problem: the
model with no state names the lanes its rules cannot hold together, those lanes' balls are offered the whole menu, and
it is built again -- until a state stands, or the lanes named have their whole menus already (then that is the answer,
and the run stops). --full: every ball its whole menu from the start.

THE TIERS: every BUS ball's whole menu is built, each move tagged with the first tier that
holds it -- 0 the pattern, 1 the pattern from the next gap point that stands clear (a pair's balls: the lines too), 2
the straight moves toward the bus's other part from every clear gap point and from a via in the pad, 3 those by every
face, 4 the lines, 5 the rest. THE MENUS BY NEED AND BY USE: the model is offered a bus ball's first TIER_MENU tiers
(0 to 3, about a fifth of its moves); its whole menu where a refusal's rules name it (need, as the pattern's balls
are widened) and for one EPOCH of the search (use), each ball's whole menu kept (whole_opts) and its first tiers
restored after, with the move its lane took. A four-bus group on its whole menus was 371k variables and 1.24 million
rows, its first state three minutes in; on its first tiers it is 137k and 389k, a state in about one. The array's
other balls keep the pattern, widened as above.

THE MODEL is each bus's ends model (whole_ends._joint_model: an option a lane and end, each end on a layer its stub
can take, the crossings the two orders make, each priced by the layers its two lanes share -- JOINT_X_LAYERS), its
ends at an array chosen from that array's menu, and at each array ONE set of rules over every lane there alike:

  * VIAS: two moves one of whose vias stands on the other's site or within its reach of the other's run (a via is a
    barrel through every layer: conflict_groups' site and reach parts) are never both chosen;
  * RUNS: a point of the array's lines carries one lane on one layer -- two runs through one point (along one line, or
    crossing there, or leaving by one exit) stand on different layers;
  * WAYS OUT: no via stands in a bus lane's way out of its exit -- the lane goes on from there, straight out past the
    face, at its own track and the router's clearance (joint_escape.exit_ray_conflicts): a plane ball's drop, or the
    via of another lane's escape -- and a PAIR's way out from the middle of its two exits on, the pair's width, where
    the route lays it as one lane;
  * PAIRS WHOLE: no other lane's exit lies between a pair's two exits on the pair's layer there -- a pair is one lane
    to the route (whole_frame's PairSplit);
  * REGIONS (an array several buses leave): on each layer, a bus's exits are ONE stretch round the array with no
    other bus's among them -- every other bus's exits there lie outside the stretch its own span, or its own lie round
    theirs, outside the stretch they span;
  * MEETINGS BETWEEN BUSES: each lane runs round an array from its exit to where it is HEADED (between_rows: toward
    its destination's face, or the corner of the side of it the lane goes round -- its berth's face, and the cut for
    one on the far face), and two lanes of two buses MEET where those stretches overlap: they cross, or one runs past
    the other's exit beside it. Each bus's lanes are planned by its own solve, which nests its own lanes side by side
    and knows nothing of another bus's, so a meeting is what a crossing is within a bus: a crossing's price, and two
    that meet on one layer change layer, one at least -- priced, in the objective. One rule in it: two that meet with
    their exits NEAR each other round the array (NEAR_EXITS) leave on two layers -- they meet right at the exits,
    before any via fits;
  * NO BUS END ON A GROUND PLANE'S LAYER, where the board has an inner layer without a plane; a run on a POWER
    plane's at its price (route_layers.run_price), ranked with the changes of layer;
  * A POCKET'S MOUTH HOLDS ITS LANES: a group's lanes that enter a pocket between two of its parts by one mouth are
    no more than land across it a lane pitch apart (whole_frame's Group.landings, which lands every lane of a mouth
    side by side whatever its layer); the rest go in by the other mouth, or straight under a part;
  * NO FAR-FACE TOOTH for a lane still on the pattern (a lane on its whole menu may take one, at its price);
  * a tooth on the source's FAR FACE -- and a PAIR's berth on its destination's far face, each leg's -- and THE WAY
    ROUND an array, priced as the ends model prices them: the far face past any count of vias, yet no refusal
    (whole_ends.FAR_TOOTH), ranked with the changes of layer; a lane that
    leaves by a face turned from its other end, or arrives by one, the length it travels round the array to the face
    toward it at the one exchange rate (select_moves.VIA_MM of track a via), with the vias;
  * EVERY BALL SERVED: the array's other balls are lanes there as the buses' are, offered the same menu under the same
    rules -- another net's ball fanned out, a plane's ball (its net a pour's) dropped to its plane by a via
    (joint_escape._drops, held clear of every other option by the whole-array fanout's own test, _hit), else
    strapped to a ball of its net beside it that is itself joined to the plane (joint_escape._straps), and fanned out
    as a signal is only where neither fits (PLANE_FAN, with the changes of layer).

EVERY rule is hard. Where no state holds them all the rules that cannot hold together are named, and the run stops.

THE SEARCH (tier_search): THE START, from the first tiers' own state, EACH BUS'S ENDS IN TURN (end_turns): one bus's
teeth freed, every berth and every other bus's ends held, then its berths (a run of exits slid along a face, which no
round of a few lanes does) -- first with each lane priced past its first two tiers, the state nearest the pattern the
rules allow, and rounds about the costly balls that leaves; then unpriced, until a pass of the buses gains too
little. Then THE EPOCHS: the lanes round the source in STRETCHES of some EPOCH_LANES
(lane_stretches, by their balls' angles about the array's centre); a stretch's balls offered their whole menus, and
the balls round each COSTLY BALL with them (a plane ball fanned out; the source ball of a lane with a tooth on its far
face, the dearest end); the model built again from the state, which stands on it as it was (every lane keeps the move
it took; where a rebuilt bus model refuses a few ends, the state nearest it, the ends moved named); rounds that free
lanes of the stretch -- about a costly ball first, else at random or about one lane and its neighbours -- each other
lane held at the best state, until EPOCH_STALL rounds gain less than a via. A SWEEP is every stretch once; one that
gained a hundredth of the objective is followed by another, its stretches drawn at random (EPOCH_SWEEPS at most).
ON TWO ROUTING LAYERS a state is judged with what the model does not count: the changes of layer the ORDER of its
crossings forces (whole_ends.route_floor -- a lane with both ends on one layer goes down and back under one it meets
there); a round's model is given the dives it can see (whole_ends.dive_cuts), and two rounds in three free a GROUP BY
WHAT CROSSES -- the lanes most of the state's dives are owed to together (dive_cover), or a lane and those it crosses.
Every lane's wide moves are worth the having -- the search held to the first four tiers ended 8 and 11 % dearer on two
boards -- but never all at once: on its whole menus at once, a four-bus group's search took 12 GB and had not ended
when it was stopped.
Each solve bounded by deterministic batches, never a clock. A model solved again after a check or a lesson of the
field laid (a repair, holding most lanes) that leaves a costly ball searches the lanes nearest it once more, as an
epoch (COSTLY_TRIES). (No bus ball tiered -- --full, every ball on its whole menu from the start: the weighted search,
then the rest alone under the upper rank it reached.)
Its state is CHECKED BY THE WHOLE
SOLVE, bus by bus, as a bus's own ends were (whole_ends.joint_choose): a refusal teaches -- lanes held to one layer,
crossings held apart, the lanes' state refused -- the lanes named get their whole menus, and the model is solved again
(whole_ends.JOINT_TRIES at most; ends with no plan are no answer, exit 5). A bus's FIRST refusal, on the pattern,
gives its balls THE LINES instead: from the pattern's via site, every line it can leave along (some thirty moves a
ball). Each model starts from the last one's state.
The objective is the buses' own, summed: a walled ball (or a plane ball fanned out) first; then the rest, a change of
layer CHANGE_VIAS vias in it and a run on a plane's layer at its price in changes.

CAPTURES: a directory of DEST.ends.pkl, each bus's ends model as its own run built it (whole_ends.choose under
ENDS_CAPTURE): its plan state -- the frame, the cut, a row part's berths' menu -- and its run's settings; its menus at
the arrays and the copper its own fanout laid there are left out, each array bare. BOARD: the bare board (the frame the
arrays are measured in, and what the menus are measured on). OUT: group.json (each bus's state), a directory an array
(exits.json, drops.json: each ball's choice, in the board's frame), and shared_field.kicad_pcb -- the bare board with
the arrays fanned out exactly as the model chose, every ball's copper. --verify: the runs' rule sampled against the
whole route's own conflict test. --plans: each bus's first plan (whole_solve) from the ends the model chose it, teeth
and berths, on the board as the model fanned it out -- no bus step between. --log: the search's own log. --source-field: OUT/source/shared_field.kicad_pcb too,
the shared array alone fanned out (a bench's: each bus's own step fans out its destination from it)."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import bisect
import collections
import contextlib
import copy
import gc
import io
import itertools
import json
import math
import os
import pickle
import shutil
import random
import re
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
import awx_settings  # noqa: E402
import bus_events  # noqa: E402

# CP-SAT's interleaved batches (reproducible): a four-bus model's search fell to an eighth of its 80th batch's objective
# between its 80th and 130th, and stood still from there to its 320th
BATCHES = 160
# the search's full workers: none with the linear relaxation -- that one found none of 146 improvements and took 130 s
# of a 195 s search's wall time, each of its batches 8.7 s for a second of work -- and TWO of them, so that two of
# the four workers are the neighbourhood searches (CP-SAT gives those what the full workers leave): with three full
# workers and one for the neighbourhoods a four-bus model's search stood still from its 170th second at 2.3 times the
# objective these two reach in 80 batches, and took 1.7 times as long
SEARCH = ['no_lp', 'core']
SEARCH_STALL = 8    # a search from a searched state: stopped at this many model reductions in a row with no better one
WIDE_A_ROUND = 16   # a large set of rules that cannot hold together: its balls widened this many a round, at most
ROWS = 2           # the pattern's surface escapes: from this many outer rows (build_menus `pattern`)
OTHER = 'array'     # an array's balls that are no bus's, in the model's names: lanes at the array alone
START_WORK = 120.0  # the start's deterministic seconds, one worker
BARE = 2.0          # mm round an array's balls: a bus's own fanout's copper there is not the group's
EPS = 1e-4
# two lanes of two buses that meet with their exits nearer than this round the array (mm) leave on two layers: the
# meeting comes within a few mm of an exit, before a via fits, and no change of layer mends it (a negotiated run's
# couplings: with the exits under 8 mm apart 34 of 35 such crossings stayed broken, 8 to 16 mm half, beyond 16 none)
NEAR_EXITS = 16.0
# a PLANE ball is served, in this order: DROPPED to its plane by a via; else STRAPPED to a ball of its net beside it
# that is itself joined to the plane (joint_escape._straps, the whole-array fanout's: a hand joins two such balls
# round one via); and FANNED OUT as a signal is -- a stub left for a later step to join -- only where neither fits:
# priced with the changes of layer, this many vias. Nothing ranked a drop above a fan-out, and 35 of a board's 153
# plane balls on its bus chips were left fanned out, joined to no plane
PLANE_FAN = 100.0
STRAP_VIAS = 2.0    # a strap, in vias: more than the drop it does without (1), so a drop is taken where one fits
# an exit's FRONT read this far (mm): its straight way out clear past the solve's no-change zone and a via's room --
# at the ends model's own 1.2 a pair left on the back layer 2 mm short of a part's pad, no room to change before it
EXIT_REACH = 2.0
SNAP = 0.02         # mm: two coordinates of an array's lines this close are one (_snap)
DROP_CELL = 1.0     # mm: drop_hits' grid, a drop tested against the escapes whose boxes meet its cells
# THE TIERS: every bus ball's whole menu built, each move tagged with the first tier that holds it -- 0 the pattern,
# 1 the pattern from the next clear site (a pair's balls: the lines too), 2 the straight moves toward the bus's other
# part from every clear site and from a via in the pad, 3 those by every face, 4 the lines, 5 the rest -- the model
# offered its first TIER_MENU tiers (below), and the search's start (tier_search) each lane on its first TIER_START
# tiers where a state allows
TIER_START = 1      # the start: each lane on its first tiers up to this one wherever a state allows
TIER_OVER = 50.0    # ...a lane past them priced this many vias in the start alone
# THE MENUS BY NEED AND BY USE: a bus ball is offered its first TIER_MENU tiers in the model; its whole menu where a
# refusal names it (need), and for one EPOCH of the search at a time (use): a stretch of EPOCH_LANES lanes round the
# source offered their whole menus, the model built again from the state, the search freeing lanes of the stretch until
# EPOCH_STALL rounds gain less than a via -- then the stretch back on its first tiers, each lane keeping the move it
# took, and the next stretch. Every lane on its whole menu at once, the tiers' four-bus model was 371k variables and
# 1.24M rows (some 4 GB built) where its first three tiers are 137k and 389k; the lanes' wide moves earned most of the
# search's gain on two boards (capped at the third tier, 372 -> 366 and 545 -> 523 where they reached 339 and 469)
TIER_MENU = 3
EPOCH_LANES = 16
EPOCH_STALL = 12
EPOCH_SWEEPS = 12   # a sweep that gained a hundredth of the objective is followed by another, its stretches drawn
EPOCH_GAIN = 0.01
COSTLY_TRIES = 2    # a costly ball left by a later model (a repair, a lesson of the field laid): its lanes searched again,
#                     this many times a ball
END_BATCHES = 24    # a bus's ends in turn, after the start (tier_search.end_turns): a solve's batches at most, a
#                     round's own (LNS_BATCHES)
LNS_K = 8           # a round of the search frees this many lanes on their whole menus
LNS_NEAR = 2.5      # mm: another ball this near a freed lane's balls, or the costly ball a round is about, is freed too
LNS_STALL = 12      # the search ends where this many rounds in a row gained less than a via's worth together
LNS_BATCHES = 24    # a round's deterministic batches (a round frees a few lanes, not the model)
LNS_HOT = 3         # rounds about one costly ball (fanned out, walled) with no gain before the search lets it be
# THE LAYERS A ROUND LEAVES FREE: the freed lanes' and those of the LNS_LAYERS lanes nearest them by their balls -- every
# other lane's two ends keep the layers they stand on, as they keep their moves. With every lane's layers free a round
# was the whole group's layers solved again with a few lanes' moves: of one stretch's 28 rounds on a four-bus group 22
# ended at their batches with nothing proved, 251 s; with all but these held the same stretch's 28 rounds were each
# proved best, 63 s, and ended within a via and a half of the same state (24 nearest: 38 s and a fifth of the gain
# short). Every lane's layers together are solved at each stretch's start, its ends held -- a second's solve
LNS_LAYERS = 48
# a lane's CHANGE OF LAYER, in vias -- in the rest's own currency (a run on a plane's layer at its price in changes).
# Ranked above everything below it, one change outweighed any number of crossings and any length
CHANGE_VIAS = 2.0
# (a bus's own run's settings that name ITS fanout: not the group's)
OWN_KEYS = ('FANOUT_NETS', 'DEST', 'ENDS_CAPTURE', 'ENDS_ONLY', bus_events.BUS)


def option_tier(b, k_, l_, i_, tier_of):
    """the tier of bus b's lane l_'s option i_ at its end k_: its moves' widest (tier_of: (array, ball) -> each move's
    of its menu); 0 where none of its moves is tiered"""
    ts_ = [tier_of[g_[:2]][g_[2]] for g_ in (getattr(mv_, 'gid', None) for mv_ in (b.E.T, b.E.B)[k_][l_][i_][0])
           if g_ and g_[:2] in tier_of]
    return max(ts_) if ts_ else 0


def lane_stretches(buses, draw=None):
    """every bus lane in STRETCHES round its source array, some EPOCH_LANES a stretch -- the lanes in the order of
    their balls' angles about the array's centre, cut evenly; `draw` (a seed): the lanes DRAWN at random instead, each
    stretch some EPOCH_LANES from anywhere round the array -- two lanes far apart that cross are freed together, as the
    whole menus' search's random rounds free them: the search's epochs, a list of [(bus, lane)]"""
    at = []
    for b in buses:
        A = b.arr[0]
        cx, cy = (A.box[0] + A.box[2]) / 2, (A.box[1] + A.box[3]) / 2
        for l_ in b.J.lanes:
            legs = [leg for leg in (b.E.legs_of.get(l_) or (l_,)) if leg in A.ball_of]
            p = A.bm.items[A.ball_of[legs[0]]][1] if legs else None
            ang = math.atan2(p.global_y - cy, p.global_x - cx) if p is not None else 0.0
            at.append((ang, b.name, str(l_)))
    at.sort()
    n = max(1, round(len(at) / EPOCH_LANES))
    size = math.ceil(len(at) / n)
    if draw is not None:
        random.Random(draw).shuffle(at)
    return [[(bn, l_) for _a, bn, l_ in at[i:i + size]] for i in range(0, len(at), size)]


def stretch_balls(buses, stretch):
    """the balls, at either end's array, of the lanes of `stretch` [(bus, lane)]: {(array, ball)}"""
    st, out = set(stretch), set()
    for b in buses:
        for l_ in b.J.lanes:
            if (b.name, str(l_)) not in st:
                continue
            for k_ in (0, 1):
                A = b.arr[k_]
                if A is None:
                    continue
                out.update((A.ref, A.ball_of[leg]) for leg in (b.E.legs_of.get(l_) or (l_,)) if leg in A.ball_of)
    return out


def lane_points(b, l_):
    """the places of lane `l_`'s balls at bus `b`'s arrays, an end each: what the search measures the distance between
    two lanes by (tier_search's `at`)"""
    at = [[], []]
    for k_ in (0, 1):
        A = b.arr[k_]
        if A is not None:
            for leg_ in (b.E.legs_of.get(l_) or (l_,)):
                if leg_ in A.ball_of:
                    p_ = A.bm.items[A.ball_of[leg_]][1]
                    at[k_].append([p_.global_x, p_.global_y])
    return at


def stretch_reach(buses, stretch, costly=()):
    """THE LANES A STRETCH'S ROUNDS CAN FREE (tier_search with `pool` the stretch): the stretch's own, each one's
    LNS_K - 1 nearest by their balls -- a round frees lanes of the stretch, or one of them and its nearest -- and the
    LNS_K nearest each costly ball (`costly`: their places). Every other lane keeps its ends through the stretch
    whatever its rounds draw, and its model is built with those lanes HELD on them (whole_ends._joint_model's `only`).
    {(bus, lane)}; the lanes in the search's own order, a tie at a distance broken as the search breaks it"""
    names = [(b.name, str(l_)) for b in buses for l_, _lg in b.E.lanes]
    pts = [[tuple(p) for e_ in lane_points(b, l_) for p in e_] for b in buses for l_, _lg in b.E.lanes]
    dist = lambda ps, qs: min((math.hypot(p[0] - q[0], p[1] - q[1]) for p in ps for q in qs), default=1e9)  # noqa: E731

    def nearest(ps, k, but=()):
        return [j for _d, j in sorted((dist(pts[j], ps), j) for j in range(len(names)) if j not in but)[:k]]
    st, out = set(stretch), set()
    for j0, nm in enumerate(names):
        if nm in st:
            out.add(j0)
            out.update(nearest(pts[j0], LNS_K - 1, but=(j0,)))
    for p in costly:
        out.update(nearest([tuple(p)], LNS_K))
    return {names[j] for j in out}


class StateRead:
    """a state of model `m` -- a value per variable, a solver's own answer on it -- read as a solver's answer is:
    Value(a variable, a literal or a linear expression), ObjectiveValue(), StatusName()"""

    def __init__(self, m, vals):
        ob = m.Proto().objective
        self.vals = vals
        self.objective = float(sum(c * (vals[v] if v >= 0 else 1 - vals[-v - 1]) for v, c in zip(ob.vars, ob.coeffs))
                               + ob.offset) * (ob.scaling_factor or 1.0)

    def Value(self, e):
        if isinstance(e, int):
            return e
        ix = getattr(e, 'Index', None)
        if ix is not None:
            i = ix()
            return self.vals[i] if i >= 0 else 1 - self.vals[-i - 1]
        from ortools.sat.python import cp_model_helper
        f = cp_model_helper.FlatIntExpr(e)
        return f.offset + sum(c * self.vals[v.index] for v, c in zip(f.vars, f.coeffs))

    def ObjectiveValue(self):
        return self.objective

    @staticmethod
    def StatusName(status):
        return status.name


def tier_search(m, meta, log, seed=1, hint=None, pool=None):
    """THE SEARCH ON TIERED MENUS of model `m`, its objective set. `meta`: 'lanes' [{'x': [the option
    variables of each end], 'tier': [their tiers], 'at': [the places of its balls at each end]}], 'others' [{'vars',
    'at'}] (the arrays' other balls), 'W_TOP' (a walled ball's, or a plane ball fanned out's, price).
    THE START, from `hint` (the first tiers' own state: main): EACH BUS'S ENDS AT ONE SIDE IN TURN (end_turns), each
    lane priced TIER_OVER vias past its first TIER_START tiers -- a state as near the pattern as the rules allow, a
    lane taken past it only where none holds or a walled ball would stand -- then rounds about the costly balls that
    pass left, the two again while they lower the count of those; then the turns unpriced, to their own end. The
    whole model searched at once from that state spent two thirds of its batches clearing walled balls one at a time
    and stood, at its last batch, wherever its path had brought it -- a four-bus group's at 3073 in 654 s, and the
    same model with its rules said in fewer rows at 7626 -- and the turns after it brought either to the state these
    reach alone. With no state to start from, the whole model as before. THEN THE
    NEIGHBOURHOOD SEARCH: rounds that free LNS_K lanes on their whole menus -- about a costly ball still in the state
    (fanned out, walled: the lanes nearest it), else a random LNS_K, else a lane and its nearest -- every other ball
    within LNS_NEAR of theirs freed with them, everything else held at the best state, that state hinted; a better state
    kept. It ends where LNS_STALL rounds in a row gained less than a via's worth together. Reproducible: one seed, each
    solve bounded by deterministic batches, never by a clock. On two boards' models, searched 240 s each way, such
    rounds ended 2 and 5 % below one search of the whole menus, and on one of them 16 % below the pattern's alone; freeing the
    costliest lane and those it crosses instead, a tier at a time or all at once, ended between the two.
    `pool` (an EPOCH, main's: a stretch of lanes on their whole menus, the rest on their first tiers): None, the start
    and rounds over every lane; [] the start alone; lane indices, no start -- `hint` is the state -- and rounds that
    free lanes of the pool (and those nearest them), ending where EPOCH_STALL rounds in a row gained less than a via.
    Returns (the state, a value per variable, or None; a line for the log)"""
    from ortools.sat.python import cp_model
    import cp_seed
    import whole_solve
    P = m.Proto()
    nv = len(P.variables)
    ob = P.objective
    obj = list(zip(ob.vars, ob.coeffs))

    def value(vs):
        return sum(c * (vs[v] if v >= 0 else 1 - vs[-v - 1]) for v, c in obj)
    # (the objective's constant, said with every value: a model built with lanes HELD counts their crossings there,
    # and a stretch's values read without it stood lower than the very state's in the model before)
    off = int(round(ob.offset))
    more = meta.get('more')         # what the model does not count of a state: main's `true_more`
    n_more = [0, 0]
    lanes, others, W_TOP = meta['lanes'], meta['others'], meta['W_TOP']
    two = max((len(e['y'][0]) for e in lanes), default=0) <= 2      # (two routing layers: `run`'s `lns`)
    pts_of = [[tuple(p) for k_ in (0, 1) for p in e['at'][k_]] for e in lanes]
    at_of = {}
    for j, e in enumerate(lanes):
        for k_ in (0, 1):
            for v in e['x'][k_]:
                if v >= 0:
                    at_of.setdefault(v, pts_of[j])
    for o in others:
        for v in o['vars']:
            at_of.setdefault(v, [tuple(o['at'])])
    # (...and a tooth on the source's FAR FACE: the dearest end there is, a hand never lays one)
    top = sorted({v for v, c in obj if v >= 0 and c >= W_TOP and v in at_of} | {v for v in meta.get('far', ()) if v in at_of})

    dist = lambda ps, qs: min((math.hypot(p[0] - q[0], p[1] - q[1]) for p in ps for q in qs), default=1e9)  # noqa: E731

    def nearest(ps, k, but=()):
        return [j for _d, j in sorted((dist(pts_of[j], ps), j) for j in range(len(lanes)) if j not in but)[:k]]

    def show(v):
        # (a state's value, its costly balls apart)
        c_, r_ = divmod(v + off, W_TOP)
        return (f'{c_} costly ball(s) + ' if c_ else '') + f'{r_ / 1000:.1f}'

    def run(fix, hint, batches, extra=None, label='a round of the search', once=False, lns=True):
        mc = m.Clone()
        Pc = mc.Proto()
        for v, x in fix.items():
            d = Pc.variables[v].domain
            d.clear()
            d.extend([x, x])
        if hint is not None:
            Pc.clear_solution_hint()
            Pc.solution_hint.vars.extend(range(len(hint)))
            Pc.solution_hint.values.extend(hint)
        if extra is not None:
            extra(mc)
        s = cp_seed.solver()
        s.parameters.num_workers = whole_solve.SOLVE_WORKERS
        s.parameters.subsolvers.extend(SEARCH)
        s.parameters.interleave_search = True
        s.parameters.max_num_deterministic_batches = batches
        s.parameters.share_glue_clauses = False
        s.parameters.share_binary_clauses = False
        if not lns:
            # (NONE OF THE SOLVER'S OWN NEIGHBOURHOOD SEARCHES in a ROUND, which is one neighbourhood of this search
            # already: its batches go to the two full strategies, which PROVE it. With them a batch was twelve
            # tasks of which the solver's seven neighbourhood searches took all but one in five: the rounds of
            # one two-layer bus's search that ended at their batches unproved, 8 to 10 s each, each held the
            # optimum from their first seconds (two from their hint) and a bound a part in a thousand short of it
            # -- the one strategy that moves the bound had run five times in 288 tasks. Without them the same
            # rounds were proved, at the same answers, in 4 to 8 s, and a round that was proved in its first
            # batches -- 97 in 100 -- is what it was. On TWO routing layers (`two`), where a round is a few
            # hundred variables once the solver has read it. NOT on more: a round there leaves the layers of
            # its nearest lanes free too (LNS_LAYERS) -- ten times the variables, a round in seven unproved either
            # way, and without the neighbourhood searches each round a third more work for states no better. And
            # NOT a bus's ends in turn: a whole side of a bus free is a model of ten thousand variables no
            # strategy proves in its batches, and there the neighbourhood searches are what finds its state --
            # without them such a solve took five times as long for a state no better)
            s.parameters.use_lns = False
        if once:
            # (a round's model, most of it held: one pass of the solver's presolve leaves what three do -- with the
            # layers held a round's search is short, and the presolve was near half its time)
            s.parameters.max_presolve_iterations = 1
        bus_events.watch_solve(s, 'ends', label, 1000, bus=None)
        r = s.Solve(mc)
        return (list(s.ResponseProto().solution)[:nv] if r in (cp_model.OPTIMAL, cp_model.FEASIBLE) else None), \
            s.StatusName(r)

    def over(mc):
        # (the start's price on a lane past its first tiers: a literal each, set by any option past them)
        Pc = mc.Proto()
        for e in lanes:
            vs = [v for k_ in (0, 1) for v, t in zip(e['x'][k_], e['tier'][k_]) if v >= 0 and t > TIER_START]
            if vs:
                o = mc.NewBoolVar('')
                for v in vs:
                    mc.AddImplication(mc.GetBoolVarFromProtoIndex(v), o)
                Pc.objective.vars.append(o.Index())
                Pc.objective.coeffs.append(int(round(1000 * TIER_OVER)))
    def end_turns(inc, best, priced=False, passes=None):
        """EACH BUS'S ENDS AT ONE SIDE IN TURN, after the start: every lane of ONE bus freed on its menu at its teeth,
        with the other balls near them -- every berth, and every other bus's ends, held -- then at its berths, its
        teeth held: the crossings between a bus's two rows of ends lowered a row at a time. With the other row a
        constant such a solve is an easy one (on a two-layer bus proved best in a second or two); a round of LNS_K
        lanes cannot slide a long run of exits along a face -- each lane of the run must move, and none gains alone
        -- and this can: a start whose address lanes left the source among its data lanes, each crossing that whole
        group, kept every one of those crossings through three sweeps of the rounds and ended with no plan.
        A BUS at a time, not an array: every bus's ends at a shared source at once were 227 lanes' teeth in one solve
        of a four-bus group -- ten of them, 82 s each, none proved, the last gaining nothing -- and from where they
        stopped one pass of these, eight solves in 96 s, took 8 % off the state; a row part's berths, on no array,
        had no turn at all (a fifth of that pass's gain). It ends where a pass of every bus gained less than a via,
        or than EPOCH_GAIN of the state below its walled balls (the second pass there gained 0.7 %, the third 0.3).
        Returns (the state, its value, the solves)"""
        order_ = list(dict.fromkeys(e['bus'] for e in lanes))
        n_, t1, n_pass = 0, time.time(), 0
        while order_ and (passes is None or n_pass < passes):
            n_pass += 1
            was_ = best
            for bus_ in order_:
                for k_e, side_ in ((0, 'teeth'), (1, 'berths')):
                    mine_ = {j for j, e in enumerate(lanes) if e['bus'] == bus_
                             and sum(1 for v in e['x'][k_e] if v >= 0) > 1}
                    if not mine_:
                        continue
                    fix = {v: inc[v] for j, e in enumerate(lanes) for k_ in (0, 1) if j not in mine_ or k_ != k_e
                           for v in e['x'][k_] if v >= 0}
                    ps_ = [tuple(p) for j in mine_ for p in lanes[j]['at'][k_e]]
                    fix.update((v, inc[v]) for o in others if not ps_ or dist([tuple(o['at'])], ps_) > LNS_NEAR
                               for v in o['vars'])
                    got, st_ = run(fix, inc, END_BATCHES, extra=over if priced else None,
                                   label=f"{bus_}'s {side_}, the rest held")
                    n_ += 1
                    # (`priced`: the state is the solver's, better on the priced objective it was asked for)
                    if got is not None and (priced or value(got) < best):
                        inc, best = got, value(got)
                    log(f'    {bus_}\'s {side_}, the rest held [{st_}]: {show(best)} ({time.time() - t1:.0f} s)')
            if was_ - best < max(1000, EPOCH_GAIN * ((was_ + off) % W_TOP)):
                break
        return inc, best, n_
    t0 = time.time()
    if pool:
        # (the state before, every lane's ends and every other ball's choice held, the rest of it solved for the
        # objective: what the model built before it read as derived terms are set at their least)
        held = {v: hint[v] for e in lanes for k_ in (0, 1) for v in e['x'][k_] if v >= 0}
        held.update((v, hint[v]) for o in others for v in o['vars'])
        got, _st = run(held, hint, LNS_BATCHES, lns=not two)
        inc = got if got is not None else list(hint)
        best = value(inc)
        if more:
            x_ = more(inc)
            best_m = math.inf if x_ is None else x_
    else:
        if hint is None:
            inc, st = run({}, hint, BATCHES, extra=over, label='the start')
            if inc is None:
                return None, f'no state [{st}]'
            best = value(inc)
        else:
            st = 'FEASIBLE'
            inc = list(hint)
            best = b0 = value(inc)
            n_pr = n_h0 = 0
            while True:
                c0 = sum(1 for v in top if inc[v])
                inc, best, n_ = end_turns(inc, best, priced=True, passes=1)
                n_pr += n_
                for try_ in range(LNS_HOT):
                    for v0 in [v for v in top if inc[v]]:
                        if not inc[v0]:
                            continue
                        centre = list(at_of[v0])
                        free = set(nearest(centre, LNS_K * (try_ + 1)))
                        fix = {v: inc[v] for j, e in enumerate(lanes) if j not in free for k_ in (0, 1)
                               for v in e['x'][k_] if v >= 0}
                        ps = [p for j in free for p in pts_of[j]] + centre
                        fix.update((v, inc[v]) for o in others if dist([tuple(o['at'])], ps) > LNS_NEAR
                                   for v in o['vars'])
                        got, _st = run(fix, inc, LNS_BATCHES, label='a round about a costly ball', lns=not two)
                        n_h0 += 1
                        if got is not None and value(got) < best:
                            inc, best = got, value(got)
                    if not any(inc[v] for v in top):
                        break
                c1 = sum(1 for v in top if inc[v])
                if not c1 or c1 >= c0:
                    break
            log(f'  the start, each bus\'s ends in turn and each lane priced past its first {TIER_START + 1} tiers, {n_pr} '
                f'solve(s), and {n_h0} round(s) about a costly ball: {show(b0)} -> {show(best)} '
                f'({time.time() - t0:.0f} s)')
        n_past = sum(1 for e in lanes if any(inc[v] for k_ in (0, 1) for v, t in zip(e['x'][k_], e['tier'][k_])
                                             if v >= 0 and t > TIER_START))
        log(f'  the start, each lane on its first {TIER_START + 1} tiers where a state allows ({n_past} of {len(lanes)} '
            f'past them) [{st}]: {show(best)} ({time.time() - t0:.0f} s)')
        t1, b0 = time.time(), best
        inc, best, n_end = end_turns(inc, best)
        log(f'  each bus\'s ends in turn, {n_end} solve(s): {show(b0)} -> {show(best)} ({time.time() - t1:.0f} s)')
        if pool is not None:
            return inc, f'the start alone: {show(best)} ({time.time() - t0:.0f} s)'
    rng = random.Random(seed)
    hist, rnd, hot_n, n_hot = [best], 0, collections.Counter(), 0
    groups, gcache, n_wise = (meta.get('groups') if pool else None), {'of': None, 'x': None, 'dv': None, 'cu': [],
                                                                      'fx': []}, [0]
    among = list(pool) if pool else list(range(len(lanes)))

    # (THE LAYERS A ROUND LEAVES FREE, LNS_LAYERS: not on two routing layers, where a round frees a lane with what it
    # crosses and the two must trade layers wherever they stand)
    hold_y = LNS_LAYERS > 0 and not groups and max((len(e['y'][0]) for e in lanes), default=0) > 2
    # (a round asked again of the state it was asked of -- the same lanes freed about the same place -- is the same
    # solve with the same answer, which bettered nothing: counted as the round it is, not solved. A round about a
    # costly ball that gains nothing frees the same nearest lanes at its next two tries)
    asked, n_same = set(), 0
    while True:
        rnd += 1
        hot = [v for v in top if inc[v] and hot_n[v] < LNS_HOT]
        centre = []
        if hot:
            hot_n[hot[0]] += 1
            n_hot += 1
            centre = list(at_of[hot[0]])
            free = set(nearest(centre, LNS_K))
        elif groups and rng.random() < 2 / 3:
            # (two routing layers) A GROUP BY WHAT CROSSES: a lane the order of the crossings forces to dive, or a
            # lane of the stretch drawn by how many it crosses -- and the lanes it crosses, as many as the round
            # frees (drawn among them where they are more), the nearest by their balls where they are fewer
            if gcache['of'] is not inc:
                gcache['of'] = inc
                gcache['x'], gcache['dv'], gcache['cu'], gcache['fx'] = groups(inc)
            X_, dv_ = gcache['x'], [j for j in gcache['dv'] if gcache['x'].get(j)]
            if gcache['cu'] and rng.random() < 0.5:
                # (THE LANES THE DIVES ARE OWED TO, together: a dive stands while any one of the lanes that force it
                # still does, so no round that moves one of them alone gains -- as many of them as the round frees,
                # those in the most of the state's forced threes: whole_ends.dive_cover)
                import whole_ends as we_
                cov_ = we_.dive_cover(gcache['cu'], LNS_K, rng)
                free, j0 = set(cov_), cov_[0]
            else:
                if dv_ and rng.random() < 0.5:
                    j0 = rng.choice(dv_)
                else:
                    j0 = rng.choices(among, weights=[len(X_.get(j, ())) + 1 for j in among])[0]
                xs_ = sorted(set(X_.get(j0, ())) - {j0})
                rng.shuffle(xs_)
                free = {j0} | set(xs_[:LNS_K - 1])
            if len(free) < LNS_K:
                free |= set(nearest(pts_of[j0], LNS_K - len(free), but=tuple(free)))
            n_wise[0] += 1
        elif rng.random() < 0.5:
            free = set(rng.sample(among, min(LNS_K, len(among))))
        else:
            j0 = rng.choice(among)
            free = {j0} | set(nearest(pts_of[j0], LNS_K - 1, but=(j0,)))
        # (every lane's ends but the freed ones', and every other ball's choice but those near them, at the state's)
        fix = {}
        for j, e in enumerate(lanes):
            if j not in free:
                for k_ in (0, 1):
                    for v in e['x'][k_]:
                        if v >= 0:
                            fix[v] = inc[v]
        ps = [p for j in free for p in pts_of[j]] + centre
        for o in others:
            if dist([tuple(o['at'])], ps) > LNS_NEAR:
                for v in o['vars']:
                    fix[v] = inc[v]
        if hold_y:
            lay_ = free | set(nearest(ps, LNS_LAYERS, but=tuple(free)))
            for j, e in enumerate(lanes):
                if j not in lay_:
                    for k_ in (0, 1):
                        for v in e['y'][k_]:
                            if v >= 0:
                                fix[v] = inc[v]
        ask_ = (frozenset(free), tuple(centre))
        if ask_ in asked:
            got, n_same = None, n_same + 1
        else:
            asked.add(ask_)
            xcut = meta.get('cuts') if pool else None      # (the dives a round can see: main's `dive_cuts`)
            got, _st = run(fix, inc, LNS_BATCHES, once=hold_y, lns=not two, extra=(
                lambda mc_: xcut(mc_, {(lanes[j_]['bus'], lanes[j_]['lane']) for j_ in free}, inc)) if xcut else None)
        if got is not None and more and pool:
            # (a state is taken where it is better WITH what the model does not count)
            if value(got) < best + (0 if best_m == math.inf else best_m):
                x_ = more(got)
                n_more[0] += 1
                if x_ is not None and (best_m == math.inf or value(got) + x_ < best + best_m):
                    inc, best, best_m = got, value(got), x_
                    n_more[1] += 1
                    hot_n.clear()
                    asked.clear()
        elif got is not None and value(got) < best:
            inc, best = got, value(got)
            hot_n.clear()
            asked.clear()
        hist.append(best + (best_m if more and pool and best_m != math.inf else 0))
        stall = EPOCH_STALL if pool else LNS_STALL
        if len(hist) > stall and hist[-1 - stall] - hist[-1] < 1000:
            break
    n_top = sum(inc[v] for v in top)
    return inc, (f'{rnd} rounds of {LNS_K} lanes ({n_hot} about a costly ball'
                 + (f', {n_same} asked before and not solved again' if n_same else '')
                 + f'): {(hist[0] + off) / 1000:.1f} -> '
                 f'{(best + off) / 1000:.1f}, {n_top} costly balls left ({time.time() - t0:.0f} s)'
                 + (f'; with what the model does not count {(hist[-1] + off) / 1000:.1f}, {n_more[1]} of {n_more[0]} states '
                    f'asked taken' if more and pool else '')
                 + (f'; {n_wise[0]} rounds about a lane and what it crosses' if n_wise[0] else ''))


def frame_map(pads_f, pads_c):
    """a frame onto another: the quarter turn and shift that carry the array's pads `pads_f` {pad number: (x, y)}
    onto `pads_c` -- a namespace of `pt` (a point's map) and `dirn` (a face's name's)"""
    import flow_frame as ff
    from escape_moves import DIRS
    common = sorted(set(pads_f) & set(pads_c))
    a = common[0]
    b = max(common, key=lambda n: math.hypot(pads_f[n][0] - pads_f[a][0], pads_f[n][1] - pads_f[a][1]))
    for k in range(4):
        ra = ff._q(pads_f[a][0], pads_f[a][1], k)
        t = (pads_c[a][0] - ra[0], pads_c[a][1] - ra[1])
        if all(math.hypot(ff._q(*pads_f[n], k)[0] + t[0] - pads_c[n][0],
                          ff._q(*pads_f[n], k)[1] + t[1] - pads_c[n][1]) < 2e-3 for n in (b, common[len(common) // 2])):
            def pt(p, k=k, t=t):
                x, y = ff._q(p[0], p[1], k)
                return (x + t[0], y + t[1])
            names = {}
            for d, u in DIRS.items():
                v = ff._q(u[0], u[1], k)
                names[d] = next(d2 for d2, u2 in DIRS.items() if abs(u2[0] - v[0]) < 1e-9 and abs(u2[1] - v[1]) < 1e-9)
            return types.SimpleNamespace(pt=pt, dirn=names.get, k=k)
    raise SystemExit('group_ends: a bus\'s frame is no quarter turn of the board\'s')


def turn_move(m, F):
    """escape move `m` in another frame (a copy: its points and face turned, nothing else changed)"""
    t = copy.copy(m)
    t.__dict__.pop('_spans_c', None)
    t.direction = F.dirn(m.direction)
    t.exit_pt = F.pt(m.exit_pt)
    t.legs = [(F.pt(a), F.pt(b), L) for a, b, L in (m.legs or [])]
    t.site = F.pt(m.site) if m.site is not None else None
    return t


def other_nets(pcb, ref, bus_nets):
    """(signals, planes): the nets with a ball on the array that are no bus's -- `planes` those a pour carries,
    `signals` the others with a pad elsewhere (a net of one pad needs no way out)"""
    from coherent_nets import namer
    sn = namer(pcb)
    poured = {z.net_id for z in (pcb.zones or []) if z.net_id}
    sig, pl = set(), set()
    for p in pcb.footprints[ref].pads:
        if not p.net_id or not p.net_name or p.pad_type == 'np_thru_hole' or sn(p.net_name) in bus_nets:
            continue
        if p.net_id in poured:
            pl.add(sn(p.net_name))
        elif len(pcb.nets[p.net_id].pads) > 1:
            sig.add(sn(p.net_name))
    return sig, pl


def served_balls(pcb, ref, bus_nets):
    """the balls NET#PAD of array `ref` that are no bus's (`bus_nets`) and carry copper of their own net already --
    a via that reaches the ball, a track that ends on it (bga_fanout.ball_has_copper): SERVED. A board an earlier
    step fanned out or routed, or the board a group's escalation keeps while the buses that failed are routed again:
    their copper stands as any other on the board, which every ball's menu is measured against. Offered a menu
    again, each was a lane of the model and laid a second time beside its own copper"""
    import joint_escape as je
    from bga_fanout import ball_has_copper
    from coherent_nets import namer
    sn = namer(pcb)
    foot = pcb.footprints[ref]
    vias = [{'x': v.x, 'y': v.y, 'size': v.size, 'net_id': v.net_id} for v in pcb.vias]
    tracks = [{'start': (s_.start_x, s_.start_y), 'end': (s_.end_x, s_.end_y), 'layer': s_.layer, 'net_id': s_.net_id}
              for s_ in pcb.segments if not getattr(s_, 'graphic', False)]
    tw = je._sizes(pcb, foot)['tw']
    return {f'{sn(p.net_name)}#{p.pad_number}' for p in foot.pads
            if p.net_id and p.net_name and sn(p.net_name) not in bus_nets and ball_has_copper(p, vias, tracks, tw)}


def via_groups(bm):
    """(cliques, bicliques) of the menu's moves (ball, index) that can never both be laid whatever layers their runs
    take: one via site, or one's via within its reach of the other's run -- conflict_groups' site and reach parts,
    at the sizes the array is laid at"""
    import conflict_groups as cg
    tags = {}
    cl, bi, _pr = cg.conflict_groups(bm.menu, stack=True, stack_pitch=bm.sz['stack'],
                                     via_r=lambda m: getattr(m, 'via_r', bm.sz['vr']),
                                     reach_extra=bm.sz['cl'] + bm.sz['tw'] / 2, xing=2, tags=tags)
    via = lambda g: any(t[0] in ('site', 'reach') for t in tags[g])   # noqa: E731
    # ...and two moves of two nets whose VIAS stand nearer than their two radii and a clearance (bm.n_near of them):
    # the relation's parts know one site shared, and a via within its reach of a RUN -- two vias on neighbouring
    # sites, neither's run beside the other's, are neither (a connector's pins at a pitch finer than a via and a
    # clearance each took one, laid into each other pair after pair)
    rmax = max([bm.sz['vr']] + [getattr(m, 'via_r', bm.sz['vr']) for ms in bm.menu.values() for m in ms])
    G_ = 2 * rmax + bm.sz['cl']
    cells = collections.defaultdict(list)
    for k in sorted(bm.menu):
        for i in range(len(bm.menu[k])):
            for c, r, _dr in bm.opts[k][i][2].vias:
                cells[(math.floor(c[0] / G_), math.floor(c[1] / G_))].append((k, i, c, r))
    near = set()
    for (cx, cy), here in cells.items():
        for k, i, c, r in here:
            for k2, i2, c2, r2 in (e for dx in (-1, 0, 1) for dy in (-1, 0, 1) for e in cells.get((cx + dx, cy + dy), ())):
                if k < k2 and bm.items[k][1].net_id != bm.items[k2][1].net_id and \
                        math.hypot(c[0] - c2[0], c[1] - c2[1]) < r + r2 + bm.sz['cl'] - 1e-6:
                    near.add(((k, i), (k2, i2)))
    # (a pair is held apart already where one group holds both: each move's groups, and the pair's two shared --
    # asked of every group in turn, a three-array model spent 7 s of every build here)
    of_ = collections.defaultdict(set)
    for n_, g in enumerate(g for g in cl if via(g)):
        for mbr in g:
            of_[tuple(mbr)].add(n_)
    near = sorted(pr for pr in near if not (of_.get(pr[0], set()) & of_.get(pr[1], set())))
    bm.n_near = len(near)
    return sorted([sorted(g) for g in cl if via(g)] + [list(pr) for pr in near], key=repr), \
        sorted(((sorted(a), sorted(b)) for a, b in bi if via((a, b))), key=repr)


def _snap(vals):
    """{value: the first of its cluster}: sorted, a value within SNAP of the one before it is the same coordinate --
    an array's balls are written to the micron, and one line reckoned from two of its balls differs by one or two"""
    out, first, last = {}, None, None
    for v in sorted(set(vals)):
        if last is None or v - last > SNAP:
            first = v
        out[v], last = first, v
    return out


def run_points(bm):
    """{(x, y): {layer: [(ball, index)]}}: the points of the array's lines each move's run stands on, by the layer it
    runs on (joint_escape.RUN: no one layer yet). A run is its lane spans (select_moves._lane_spans: a stretch along a
    row or a column line); the points of a line are its spans' ends and where a line of the other axis crosses it.
    Two spans along one line that meet share a point (the end of one within the other), a row's and a column's that
    cross share the crossing, two that leave by one exit share it: one lane a point, on a layer. Coordinates within
    SNAP of each other are one (_snap)"""
    import select_moves as sm
    spans = collections.defaultdict(list)       # layer -> [(axis, line, a, b, member)]
    for k in sorted(bm.menu):
        for i, mv in enumerate(bm.menu[k]):
            for key, a, b in sm._lane_spans(mv):
                spans[key[2]].append((key[0], key[1], round(a, 3), round(b, 3), (k, i)))
    every = [s_ for ss in spans.values() for s_ in ss]
    # (the x's: the columns' lines and the rows' spans' ends; the y's: the rows' lines and the columns' spans' ends)
    sx = _snap([s_[1] for s_ in every if s_[0] == 'col'] + [v for s_ in every if s_[0] == 'row' for v in s_[2:4]])
    sy = _snap([s_[1] for s_ in every if s_[0] == 'row'] + [v for s_ in every if s_[0] == 'col' for v in s_[2:4]])
    out = collections.defaultdict(lambda: collections.defaultdict(list))
    for L, ss in sorted(spans.items()):
        # (each span: its line, and its ends along it, at their clusters' coordinates)
        ss = [(ax, (sy if ax == 'row' else sx)[c], (sx if ax == 'row' else sy)[a], (sx if ax == 'row' else sy)[b], mbr)
              for ax, c, a, b, mbr in ss]
        lines = {'row': sorted({s_[1] for s_ in ss if s_[0] == 'row'}), 'col': sorted({s_[1] for s_ in ss if s_[0] == 'col'})}
        ends = collections.defaultdict(set)
        for ax, c, a, b, _m in ss:
            ends[(ax, c)].update((a, b))
        marks = {(ax, c): sorted(ends[(ax, c)] | set(lines['col' if ax == 'row' else 'row'])) for (ax, c) in ends}
        for ax, c, a, b, mbr in ss:
            for v in marks[(ax, c)]:
                if a - EPS <= v <= b + EPS:
                    out[(v, c) if ax == 'row' else (c, v)][L].append(mbr)
    return out


def verify_runs(bm, pts, cliques, bicliques, n=200000, seed=1):
    """an array's rules (the vias' groups, a point one lane a layer) against the whole route's own conflict test,
    select_moves._conflict, on `n` pairs of moves of two balls: a pair it holds apart is one the rules hold apart (on
    any layers by a via's group, on one layer by a shared point), and no other is"""
    import joint_escape as je
    import select_moves as sm
    at = collections.defaultdict(set)           # move -> its (point, layer)s
    for p, by_l in pts.items():
        for L, ms in by_l.items():
            for mbr in ms:
                at[mbr].add((p, L))
    hard = set()
    for g in cliques:
        hard.update((a, b) for a, b in itertools.combinations(g, 2))
    for a_, b_ in bicliques:
        hard.update((min(a, b), max(a, b)) for a in a_ for b in b_ if a != b)
    rnd = random.Random(seed)
    # (half the pairs drawn from one point's or one group's members -- the pairs that matter -- half anywhere)
    pools = [sorted({m_ for ms in by_l.values() for m_ in ms}) for by_l in pts.values()]
    pools = [pl for pl in pools if len({m_[0] for m_ in pl}) > 1] + [g for g in cliques] + [a + b for a, b in bicliques]
    every = [(k, i) for k in sorted(bm.menu) for i in range(len(bm.menu[k]))]
    bad = collections.Counter()
    seen = 0
    for t in range(n):
        a, b = rnd.sample(pools[rnd.randrange(len(pools))] if t % 2 == 0 else every, 2)
        if a[0] == b[0]:
            continue
        seen += 1
        ma, mb = bm.menu[a[0]][a[1]], bm.menu[b[0]][b[1]]
        ra, rb = (getattr(mm, 'via_r', bm.sz['vr']) for mm in (ma, mb))
        # (each via at its own reach, as the rules' groups price it: a via-in-pad's finer via at its own radius -- one
        # reach for both, the larger, called 0.4 % of a memory array's whole-menu pairs held apart that are clear)
        ext = bm.sz['cl'] + bm.sz['tw'] / 2
        want = sm._conflict(ma, mb, strict=True, stack=True, xing=2, reach=(ra + ext, rb + ext))
        got = (min(a, b), max(a, b)) in hard or bool(at[a] & at[b])
        if want != got:
            bad[('the test alone' if want else 'the rules alone', ma.layer == je.RUN, mb.layer == je.RUN)] += 1
    return seen, dict(bad)


def _in_poly(p, poly):
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > p[1]) != (y1 > p[1]) and p[0] < x0 + (p[1] - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def keep_outs(pcb):
    """(track_banned(a, b, layer, half width), via_banned(point, radius)): the board's keep-out rule areas
    (board_info.keepouts) as two tests -- copper of a track from a to b on `layer`, or of a via at a point (a barrel
    through every layer), standing in an area that forbids it there"""
    import joint_escape as je
    from mask_artwork import routing_keepouts
    areas = []
    for k in routing_keepouts(pcb):         # (the router's: the rule areas and the mask artwork, plan_audit.keepout_areas)
        poly = [tuple(q[:2]) for q in (k.get('polygon') or ())]
        if len(poly) >= 3:
            xs, ys = [q[0] for q in poly], [q[1] for q in poly]
            areas.append((poly, set(k.get('layers') or ()), k.get('tracks_allowed', True), k.get('vias_allowed', True),
                          (min(xs), min(ys), max(xs), max(ys))))
    on = lambda Ls, L: L in Ls or '*.Cu' in Ls or 'F&B.Cu' in Ls and L in ('F.Cu', 'B.Cu')   # noqa: E731

    def near(a, b, poly, box, r):
        # (its box first: an array's moves stand nowhere near most areas)
        if max(a[0], b[0]) < box[0] - r or min(a[0], b[0]) > box[2] + r or \
                max(a[1], b[1]) < box[1] - r or min(a[1], b[1]) > box[3] + r:
            return False
        return _in_poly(a, poly) or _in_poly(b, poly) or \
            any(je._seg_seg(a, b, u, v) < r for u, v in zip(poly, poly[1:] + poly[:1]))

    def track_banned(a, b, layer, hw):
        return any(not tr and on(Ls, layer) and near(a, b, poly, box, hw) for poly, Ls, tr, _vi, box in areas)

    def via_banned(c, r):
        return any(not vi and near(c, c, poly, box, r) for poly, _Ls, _tr, vi, box in areas)
    return track_banned, via_banned


def _menus(pcb, ref, nets, only=None, pattern=True, lines=False):
    """joint_escape.build_menus for the balls of `nets` on array `ref`, every face -- the pattern's moves, or (`only`:
    those balls) their whole menus"""
    import joint_escape as je
    with contextlib.redirect_stdout(io.StringIO()):
        return je.build_menus(pcb, ref, nets, [], [], all_faces=True, escape_to={n: ref for n in nets}, only=only,
                              pattern=ROWS if pattern else False, lines=lines)


def build_array(pcb, ref, bus_of):
    """an ARRAY of the group, on the bare board `pcb`: every ball's PATTERN moves by every face
    (joint_escape.build_menus `pattern`: `bm.menu` {NET#PAD: [Move]}, `bm.opts` each move with its copper, `bm.sz` the
    sizes they are laid at) -- the buses' balls (`bus_of` {net: its bus}), the other signals' and the planes' alike --
    each plane ball's drops {NET#PAD: [(Drop, its copper)]}, and the rules' data (array_rules). A ball's WHOLE menu is
    built only when it is asked for (widen)"""
    import joint_escape as je
    sig, pl = other_nets(pcb, ref, set(bus_of))
    nets = sorted(set(bus_of) | sig | pl)
    # (the balls SERVED ALREADY are no lanes here -- served_balls; `only` the rest, None on a bare array)
    from coherent_nets import namer
    sn_ = namer(pcb)
    gone = served_balls(pcb, ref, set(bus_of))
    only = None if not gone else \
        {f'{sn_(q.net_name)}#{q.pad_number}' for q in pcb.footprints[ref].pads if q.net_id and q.net_name} - gone
    bm = _menus(pcb, ref, nets, only=only)
    with contextlib.redirect_stdout(io.StringIO()):
        dm = je.build_menus(pcb, ref, [], [], [], drops=sorted(pl), only=only)
    track_banned, via_banned = keep_outs(pcb)
    hw = bm.sz['tw'] / 2
    pads = {p.pad_number: (p.global_x, p.global_y) for p in pcb.footprints[ref].pads}
    xs_, ys_ = [p[0] for p in pads.values()], [p[1] for p in pads.values()]
    A = types.SimpleNamespace(
        ref=ref, bm=bm, nets=nets, pads=pads, box=(min(xs_), min(ys_), max(xs_), max(ys_)), wide=set(),
        drops={k: [(d, je.Piece(dm.items[k][0], [d.stub + (d.layer,)] if d.stub else [], [(d.site, d.r, d.dr)],
                                [dm.balls[k]])) for d in ds if not via_banned(d.site, d.r)
                   and not (d.stub and track_banned(d.stub[0], d.stub[1], d.layer, hw))] for k, ds in dm.dmenu.items()},
        straps={k: [(s_, je.Piece(bm.items[k][0], [(s_.a, s_.b, s_.layer)], [], [s_.a, s_.b]))
                    for s_ in (bm.straps.get(k) or ()) if s_.to in dm.dmenu
                    and not track_banned(s_.a, s_.b, s_.layer, hw)] for k in dm.dmenu},
        ball_of={nm: k for k, (nm, _p) in bm.items.items() if nm in bus_of}, bus_of=dict(bus_of),
        okeys=sorted(k for k, (nm, _p) in bm.items.items() if nm not in bus_of), served=sorted(gone))
    array_rules(pcb, A)
    return A


def widen(pcb, A, keys):
    """the balls `keys` of array `A` offered their WHOLE menus from here: built now (every site, every side-step), in
    place of their pattern's moves, and the array's rules' data made again"""
    wm = _menus(pcb, A.ref, A.nets, only=set(keys), pattern=False)
    for k in sorted(keys):
        A.bm.menu[k], A.bm.opts[k] = wm.menu[k], wm.opts[k]
    A.wide |= set(keys)
    array_rules(pcb, A)


def expand(pcb, A, keys):
    """the balls `keys` of array `A` offered THE LINES from here: from the pattern's one via site, every line it can
    leave along by each face (joint_escape.escape_field_moves `lines`: some thirty moves a ball, a bus's own escape
    menu's size) -- built now, in place of their pattern's moves"""
    lm = _menus(pcb, A.ref, A.nets, only=set(keys), lines=True)
    for k in sorted(keys):
        A.bm.menu[k], A.bm.opts[k] = lm.menu[k], lm.opts[k]
    array_rules(pcb, A)


def array_rules(pcb, A):
    """array `A`'s rules' data over the moves its balls are offered now: each move's `gid` (the array, its ball, its
    index); the board's KEEP-OUT areas, which the menus' own copper models do not hold -- a run keeps the layers it
    stands clear of them on, and a move with a leg or a via in one, or a run with no layer left, is no option (`dead`);
    `offer` {ball: the indices offered}; the vias' groups (via_groups) and the points of its lines (run_points)"""
    import joint_escape as je
    bm = A.bm
    track_banned, via_banned = keep_outs(pcb)
    hw, dead = bm.sz['tw'] / 2, set()
    for k, ms in bm.menu.items():
        for i, mv in enumerate(ms):
            mv.gid = (A.ref, k, i)
            if mv.layer == je.RUN:
                mv.runs_on = tuple(L for L in mv.runs_on
                                   if not any(track_banned(a, b, L, hw) for a, b, Lg in mv.legs if Lg == je.RUN))
            if (mv.layer == je.RUN and not mv.runs_on) or \
                    any(track_banned(a, b, Lg, hw) for a, b, Lg in mv.legs if Lg != je.RUN) or \
                    any(via_banned(c, r) for c, r, _dr in bm.opts[k][i][2].vias):
                dead.add((k, i))
    A.dead = dead
    if getattr(A, 'hits', None) is None:
        A.hits = {}        # (drop_hits' answers, by the copper's own pieces: kept as menus change)
    A.offer = {k: [i for i in range(len(ms)) if (k, i) not in dead] for k, ms in bm.menu.items()}
    A.cliques, A.bicliques = via_groups(bm)
    A.pts = run_points(bm)


def drop_hits(A, offered):
    """{(ball, drop index): ({(ball, move index)}, {(ball, drop index)})}: the escapes offered (`offered` {ball: menu
    indices}) and the other balls' drops each drop of array `A` cannot be laid with -- joint_escape._hit, the whole-array
    fanout's own test of two options' copper at the sizes they are laid at (two plane balls of one net at one site share
    its via). An array's drops are fixed: each drop's against the other balls' drops are tested once (A.hits['dd']), and
    each escape's copper once against the drops near it, the drops it hits kept with it (A.hits['esc'], by the piece:
    held beside its id, so no id is met again for another) -- whatever menus the balls are offered since. Keyed by a
    move's index on its ball's menu, the answers went with every model built again: one board's six models tested 3.3
    million pairs, 70 of its 154 s; kept a pair at a time, 8 million of them held 0.4 GB"""
    import joint_escape as je
    bm, sz = A.bm, A.bm.sz
    # (a plane ball's drops and, after them, its straps: one list, as array_rows' literals are)
    drops = {k: list(A.drops[k]) + list(A.straps.get(k, ())) for k in A.drops}
    C = A.hits
    geo = C.setdefault('geo', {})
    apart = lambda a, b: a[0] > b[2] or b[0] > a[2] or a[1] > b[3] or b[1] > a[3]    # noqa: E731

    def cells(b):
        return [(gx, gy) for gx in range(math.floor(b[0] / DROP_CELL), math.floor(b[2] / DROP_CELL) + 1)
                for gy in range(math.floor(b[1] / DROP_CELL), math.floor(b[3] / DROP_CELL) + 1)]
    if 'dd' not in C:
        # (the drops by the cells of a coarse grid their boxes meet -- two boxes that meet share a cell, so a drop
        # met through its cells is every one a whole list would give -- and each drop's against the drops of the
        # balls after its own, once)
        dbox, dgrid, dd = {}, collections.defaultdict(list), {}
        for k in sorted(drops):
            for j, (_d, pc) in enumerate(drops[k]):
                dbox[(k, j)] = je._box(pc, sz['grow'])
                for c_ in cells(dbox[(k, j)]):
                    dgrid[c_].append((k, j))
        for k in sorted(drops):
            net = bm.items[k][0]
            for j, (_d, pc) in enumerate(drops[k]):
                b0, drs, seen = dbox[(k, j)], set(), set()
                for c_ in cells(b0):
                    for k2, j2 in dgrid.get(c_, ()):
                        if k2 <= k or (k2, j2) in seen:
                            continue
                        seen.add((k2, j2))
                        if not apart(b0, dbox[(k2, j2)]) and \
                                je._hit(pc, drops[k2][j2][1], bm.items[k2][0] == net, sz, geo):
                            drs.add((k2, j2))
                dd[(k, j)] = drs
        C['dbox'], C['dgrid'], C['dd'] = dbox, dgrid, dd
    dbox, dgrid, dd = C['dbox'], C['dgrid'], C['dd']
    esc_hits = C.setdefault('esc', {})
    out = {kj: (set(), set(drs)) for kj, drs in dd.items()}
    for k2 in sorted(offered):
        net2 = bm.items[k2][0]
        for i in offered[k2]:
            pc2 = bm.opts[k2][i][2]
            g = esc_hits.get(id(pc2))
            if g is None or g[0] is not pc2:
                b2, hs, seen = je._box(pc2, sz['grow']), [], set()
                for c_ in cells(b2):
                    for k, j in dgrid.get(c_, ()):
                        if k == k2 or (k, j) in seen:
                            continue
                        seen.add((k, j))
                        if not apart(dbox[(k, j)], b2) and \
                                je._hit(drops[k][j][1], pc2, bm.items[k][0] == net2, sz, geo):
                            hs.append((k, j))
                g = esc_hits[id(pc2)] = (pc2, tuple(sorted(hs)))
            for kj in g[1]:
                out[kj][0].add((k2, i))
    return out


def rect_seg_gap(px, py, hx, hy, a, b):
    """the gap between a pad's rectangle (its centre, its half sizes) and the centre line of the run a-b: 0 where the
    run crosses the rectangle"""
    (ax, ay), (bx, by) = a, b
    t0, t1 = 0.0, 1.0           # (the run clipped to the rectangle: any of it inside?)
    for p_, q_ in ((ax - bx, ax - (px - hx)), (bx - ax, (px + hx) - ax), (ay - by, ay - (py - hy)),
                   (by - ay, (py + hy) - ay)):
        if abs(p_) < 1e-12:
            if q_ < 0:
                break
        elif p_ < 0:
            t0 = max(t0, q_ / p_)
        else:
            t1 = min(t1, q_ / p_)
    else:
        if t0 <= t1:
            return 0.0
    L2 = (bx - ax) ** 2 + (by - ay) ** 2

    def to_run(x, y):
        t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / L2))
        return math.hypot(ax + t * (bx - ax) - x, ay + t * (by - ay) - y)
    # (apart, the nearest two points of a rectangle and a run have a corner of the one or an end of the other)
    return min([math.hypot(max(abs(x - px) - hx, 0.0), max(abs(y - py) - hy, 0.0)) for x, y in (a, b)]
               + [to_run(px + sx * hx, py + sy * hy) for sx in (-1, 1) for sy in (-1, 1)])


def pad_on(ly, L):
    """whether a held pad of layers `ly` (() where they are not known: every layer) has copper on the layer L"""
    return not ly or L in ly or '*.Cu' in ly


def held_hits(held, pc, net, tw, cl, run, way=None):
    """WHAT A PART HELD WHERE IT CAME RULES OUT of one option's copper `pc` (joint_escape.Piece) of the net `net`:
    (whether the option cannot stand whatever layer its run takes -- a via within the clearance `cl` of a pad of
    another net, or a leg on a layer of its own, `tw` wide, that near a pad with copper there -- and the layers [a
    tuple a pad, pad_on] of the pads a leg on its lane's own layer (`run`: joint_escape.RUN) comes that near, or its
    lane's way out `way` ((its start, its end), its width: way_of)). `held` [(a pad's centre x, y, its half sizes,
    its net, its layers)]"""
    dead, on = False, []
    for px, py, hx, hy, pn, *ly in held:
        if pn == net:
            continue
        ly = tuple(ly[0]) if ly else ()
        if any(math.hypot(max(abs(c[0] - px) - hx, 0.0), max(abs(c[1] - py) - hy, 0.0)) - r < cl - 1e-6
               for c, r, _dr in pc.vias):
            dead = True
        for a, b, Lg in pc.legs:
            if rect_seg_gap(px, py, hx, hy, a, b) - tw / 2 < cl - 1e-6:
                if Lg == run:
                    on.append(ly)
                elif pad_on(ly, Lg):
                    dead = True
        if way is not None and rect_seg_gap(px, py, hx, hy, *way[0]) - way[1] / 2 < cl - 1e-6:
            on.append(ly)
    return dead, on


def standing(pads, laid, ways, name_of, buses, cl):
    """THE LANES THAT STAND ON A PART HELD WHERE IT CAME: {a bus's name: its lanes} whose copper AS LAID (`laid`
    [(a net's id, its copper, its run's layer, its track, its array)]: the field's pieces) or whose way out (`ways`
    [(a net's id, an exit, its direction, its layer)]) the part's pads `pads` rule out (held_hits) -- the repair's
    to free. `name_of`: a net's name by its id; a bus's lanes by its nets' names (its `leg_lane`). Read off what was
    laid and not off the menus: a ball's menu is narrowed back to its first tiers once the search ends, and a
    state's option was then no index of it"""
    import joint_escape as je
    on_it = set()
    for nid, pc, L, tw, _ref in laid:
        dead_, on_ = held_hits(pads, pc, name_of(nid), tw, cl, je.RUN)
        if dead_ or any(pad_on(ly, L) for ly in on_):
            on_it.add(name_of(nid))
    for nid, ex, d, L in ways:
        w = wayout_runs([(nid, tuple(ex), d, L)], [L])
        if w and any(pad_on(ly, L) for ly in held_hits(pads, je.Piece(None, [], [], []), name_of(nid), 0.0, cl,
                                                       je.RUN, ((w[0]['start'], w[0]['end']), w[0]['width']))[1]):
            on_it.add(name_of(nid))
    return {b.name: {b.leg_lane[n_] for n_ in on_it if n_ in b.leg_lane} for b in buses}


WALL_VIAS = 2       # (field_walls) the most of the field's vias round a walled pad one lesson gives other sites
WALL_CANDS = 10     # (field_walls) the field's vias nearest such a pad asked of, alone and two at a time


def copper_pads(pcb, ref):
    """the pads of part `ref` that are copper: a paste aperture drawn as a pad of its own (no copper layer) is none"""
    cu = set(pcb.board_info.copper_layers or ())
    return [p for p in pcb.footprints[ref].pads if '*.Cu' in p.layers or cu & set(p.layers)]


def walled_on(before, after, refs, bus, prot):
    """(the walls' nets of `after`, the rung, ['REF.PAD']): the pads later routes must reach beside the arrays `refs`
    and beneath them (route_bus._cage_pads, `after`'s copper the copper near) that the step's own cage test
    (route_bus._caged, at the fab ladder's finest rung) finds walled in on `after` and free on `before` (both parsed).
    The walls are the bus's nets (`bus`, full names), the protected ones (`prot`) and the poured ones"""
    import route_bus as rb
    bus, prot = set(bus), set(prot)

    def fixed_of(pcb):
        return {i for i, nn in pcb.nets.items() if nn.name in bus | prot} | \
            {z.net_id for z in (pcb.zones or []) if z.net_id}
    ff, fb = fixed_of(after), fixed_of(before)
    rung = rb._cage_rung(after)
    asked = {r: [p.pad_number for p in ps] for r, ps in rb._cage_pads(after, refs, bus, after.segments,
                                                                         after.vias).items()}
    now = rb._caged_on(after, asked, ff, rung)
    if not now:
        return ff, rung, []
    was_k = collections.defaultdict(list)
    for k in now:
        was_k[k.split('.', 1)[0]].append(k.split('.', 1)[1])
    was = set(rb._caged_on(before, dict(was_k), fb, rung))
    return ff, rung, [k for k in now if k not in was]


def field_walls(bare, bare_path, field, refs, bus, held=()):
    """THE PADS THE FIELD WALLS IN, and each one's lesson: the pads later routes must reach beside the arrays `refs`
    and beneath them (route_bus._cage_pads -- a decoupling capacitor's, a test point's) that the step's own cage test
    (route_bus._caged, at the fab ladder's finest rung) finds walled in on the laid `field` and free on the bare board
    `bare` (parsed; its path `bare_path`, its project's protected nets walls as the bus step's grade reads them). The
    walls are the bus's nets (`bus`, full names), the poured nets and the protected -- under an array, the lattice of
    the plane balls' drops, each a via on a site between four balls: two vias a pitch apart leave less room than a
    track and its clearances, and a capacitor's supply pad in a cell of them with no via of its own net beside it was
    reached from nowhere, its net shipped open. The menus measure their vias as if the movable passives were not there.
    The human board runs a short track from such a pad through sites its drops leave empty to a via of the pad's own
    net half a millimetre to a millimetre off. Each such pad's lesson, the first that frees it:
    'hold' -- its part MOVED by the nudge and free where it came, with no copper of another net on its pads there (as
    a part held where it came has none, array_rows): the part is held there (`what`: its reference). Parts in `held`
    are held already;
    'sites' -- the fewest of the field's vias of other nets round it (WALL_CANDS nearest within WALL_NEAR, one, else
    WALL_VIAS of them, the nearest first; no bus net's, no locked one) whose sites, left to no other net's drop, free it
    (`what`: their centres) -- the balls dropped there take another site, share a via of their net or are strapped;
    None -- neither frees it (`what`: why).
    [(the pad 'REF.PAD', its net, kind, what)]"""
    import copy
    import itertools
    import route_bus as rb
    from kicad_parser import parse_kicad_pcb, is_immutable_copper
    from protected_nets import read_protected_nets, pro_path_for_board
    with contextlib.redirect_stdout(io.StringIO()):
        f = parse_kicad_pcb(field)
    bus = set(bus)
    ff, rung, walled = walled_on(bare, f, refs, bus, read_protected_nets(pro_path_for_board(bare_path)))
    cl = rung[2]

    def gap(o, p):
        """the copper of track or via `o` to pad `p`'s rectangle"""
        hx, hy = p.size_x / 2, p.size_y / 2
        if hasattr(o, 'start_x'):
            return rect_seg_gap(p.global_x, p.global_y, hx, hy, (o.start_x, o.start_y),
                                (o.end_x, o.end_y)) - o.width / 2
        return math.hypot(max(abs(o.x - p.global_x) - hx, 0.0), max(abs(o.y - p.global_y) - hy, 0.0)) - o.size / 2

    out = []
    for k in walled:
        r, n = k.split('.', 1)
        pad = next(p for p in f.footprints[r].pads if p.pad_number == n)
        ff_, fb_ = f.footprints[r], bare.footprints.get(r)
        if fb_ is not None and r not in held and (abs(ff_.x - fb_.x) + abs(ff_.y - fb_.y) > 1e-3
                                                  or abs((ff_.rotation - fb_.rotation) % 360) > 1e-3):
            # (the part where it came, as a hold leaves it: no other net's via on its pads, nor track on their layer)
            c = copy.copy(rb._cropped(f, fb_))
            c.footprints = dict(c.footprints)
            c.footprints[r] = fb_
            cps = copper_pads(bare, r)
            c.vias = [v for v in c.vias if not any(q.net_id != v.net_id and gap(v, q) < cl for q in cps)]
            c.segments = [s for s in c.segments
                          if not any(q.net_id != s.net_id and (s.layer in q.layers or '*.Cu' in q.layers)
                                     and gap(s, q) < cl for q in cps)]
            q0 = next((q for q in fb_.pads if q.pad_number == n), None)
            if q0 is not None and not rb._caged(rb._walls(c, ff, q0.net_id, is_immutable_copper), q0, *rung):
                out.append((k, pad.net_name, 'hold', r))
                continue
        crop = rb._cropped(f, ff_)
        cands = sorted(((gap(v, pad), i) for i, v in enumerate(crop.vias)
                        if v.net_id != pad.net_id and v.net_id in ff and f.nets[v.net_id].name not in bus
                        and not is_immutable_copper(v)), key=lambda t: t[0])
        cands = [crop.vias[i] for d_, i in cands if d_ <= rb.WALL_NEAR][:WALL_CANDS]
        combos = sorted((c_ for size in range(1, WALL_VIAS + 1)
                         for c_ in itertools.combinations(range(len(cands)), size)), key=lambda c_: (len(c_), sum(c_)))
        hit = None
        for c_ in combos:
            gone = {id(cands[i]) for i in c_}
            w = copy.copy(crop)
            w.vias = [v for v in crop.vias if id(v) not in gone]
            if not rb._caged(rb._walls(w, ff, pad.net_id, is_immutable_copper), pad, *rung):
                hit = [(round(cands[i].x, 4), round(cands[i].y, 4)) for i in c_]
                break
        out.append((k, pad.net_name, 'sites', hit) if hit else
                   (k, pad.net_name, None, f'no {WALL_VIAS} of the {len(cands)} vias of other nets round it free it'
                    + (' (nor its part where it came)' if fb_ is not None and r not in held
                       and abs(ff_.x - fb_.x) + abs(ff_.y - fb_.y) > 1e-3 else '')))
    return out


def tie_walls(untied, tied, ties, refs, bus, prot):
    """THE FIELD'S TIES THAT WALL A PAD IN: (the indices of `ties`' tracks, of its vias, [(the pad 'REF.PAD', why)])
    -- the pieces (field_ties.pieces) of the ties laid on the field at `tied` (`ties`: field_ties.lay's, its 'tracks'
    and 'vs') to take out again, each one whose copper alone gone frees a pad later routes must reach that the
    untied field (`untied`) leaves free (walled_on: the step's own cage test). A rail's tie run round another cap
    under the array, from a via on one side of it back along the other, closed that cap's pad off from every way
    out; its net shipped open past copper the bus step protects. The pad nothing frees is named"""
    import copy
    import types
    import field_ties
    import route_bus as rb
    from kicad_parser import parse_kicad_pcb, is_immutable_copper
    with contextlib.redirect_stdout(io.StringIO()):
        a, b = parse_kicad_pcb(untied), parse_kicad_pcb(tied)
    ff, rung, walled = walled_on(a, b, refs, bus, prot)
    if not walled:
        return set(), set(), []
    ts = [types.SimpleNamespace(start_x=t['start'][0], start_y=t['start'][1], end_x=t['end'][0], end_y=t['end'][1],
                                width=t['width'], layer=t['layer'], net_id=t['net_id'])
          for t in ties.get('tracks') or ()]
    vs = [types.SimpleNamespace(x=v['x'], y=v['y'], size=v['size'], net_id=v['net_id']) for v in ties.get('vs') or ()]
    its = field_ties.pieces(ts, vs)
    by_pc = collections.defaultdict(lambda: (set(), set()))
    for i, (k_, o, pc) in enumerate(its):
        by_pc[pc][0 if k_ == 's' else 1].add(i if k_ == 's' else i - len(ts))
    sk = lambda o: (o.layer, round(o.start_x, 3), round(o.start_y, 3),         # noqa: E731
                    round(o.end_x, 3), round(o.end_y, 3))
    vk = lambda o: (round(o.x, 3), round(o.y, 3))       # noqa: E731
    out_s, out_v, named = set(), set(), []
    for key in walled:
        r, n = key.split('.', 1)
        pad = next(p for p in b.footprints[r].pads if p.pad_number == n)
        crop = rb._cropped(b, b.footprints[r])
        near = sorted(by_pc, key=lambda pc_: min(
            [math.hypot((ts[i].start_x + ts[i].end_x) / 2 - pad.global_x,
                        (ts[i].start_y + ts[i].end_y) / 2 - pad.global_y) for i in by_pc[pc_][0]]
            + [math.hypot(vs[j].x - pad.global_x, vs[j].y - pad.global_y) for j in by_pc[pc_][1]] + [9e9]))
        hit = None
        for pc_ in near:
            gone_s = {sk(ts[i]) for i in by_pc[pc_][0]}
            gone_v = {vk(vs[j]) for j in by_pc[pc_][1]}
            w = copy.copy(crop)
            w.segments = [s_ for s_ in crop.segments if sk(s_) not in gone_s]
            w.vias = [v_ for v_ in crop.vias if vk(v_) not in gone_v]
            if not rb._caged(rb._walls(w, ff, pad.net_id, is_immutable_copper), pad, *rung):
                hit = pc_
                break
        if hit is None:
            named.append((key, 'no one tie alone gone frees it'))
            continue
        out_s |= by_pc[hit][0]
        out_v |= by_pc[hit][1]
        named.append((key, f'a tie of {b.nets[ts[next(iter(by_pc[hit][0]))].net_id].name if by_pc[hit][0] else "its net"} '
                           'round it taken out'))
    return out_s, out_v, named


def way_of(g0, L):
    """((its start, its end), its width) of the way out of the menu move `g0`'s exit on the layer L, as the nudge is
    given it (wayout_runs), or None where its exit has no direction"""
    w = wayout_runs([(0, tuple(g0.exit_pt), g0.direction, L)], [L])
    return ((w[0]['start'], w[0]['end']), w[0]['width']) if w else None


def wayout_runs(wayouts, layers, reach=None):
    """THE LANES' WAYS OUT as copper: [{'start', 'end', 'width', 'layer', 'net_id'}], one for each of `wayouts`
    [(net id, an exit, its direction, its layer)] on one of the board's copper `layers` -- the exit's straight run out
    to where its lane can turn, two lanes' bars on, of the lane's own net; or `reach` mm on"""
    import rules as _rules
    from escape_moves import DIRS
    import pairs as _pairs
    R_ = _rules.active()
    # (as wide as the bar the loop's audit holds a lane to beside a pad -- plan_audit.check_static: the lay's own
    # clearance, the hug, where the nudge keeps the spec's; half a grid step either side for a piece off the grid;
    # and the router's corner buffer in a pad's corner zone. A track and a grid step wide, a decap was parked
    # 0.017 mm inside that bar beside a berth, and the bus's loop stopped at once on a finding no solve moves)
    w_wo = R_.track + R_.grid + 2 * (R_.hug - R_.clearance) + 2 * _pairs.corner_buffer(R_.grid)
    reach_wo = 2 * (R_.track / 2 + R_.clearance + R_.grid / 2) if reach is None else reach
    wo = []
    for nid_, (ex_, ey_), dir_, L_ in sorted(set(wayouts)):
        d_ = DIRS.get(dir_)
        if d_ is None or L_ not in layers:
            continue
        wo.append({'start': (round(ex_, 4), round(ey_, 4)), 'width': w_wo, 'layer': L_, 'net_id': nid_,
                   'end': (round(ex_ + d_[0] * reach_wo, 4), round(ey_ + d_[1] * reach_wo, 4))})
    return wo


def stuck_in_ways(field, caps, wayouts):
    """THE PARTS THE NUDGE LEFT IN THE LANES' WAYS OUT, as findings: [{'type': 'way-out', 'layer', 'net1', 'net2': None,
    'pad_ref', 'closest_pt'}], one for each lane's way out (`wayouts`, wayout_runs) a pad of such a part stands in
    on the laid `field` -- the parts the nudge's own lines (`caps`: write_field's) name as stuck or unresolved, a pad
    within the clearance of the way out's run on its layer. The ways out are laid for the nudge alone and taken up
    again: a part left standing in them breaks no rule of the board's, and the field's check finds nothing"""
    import rules as _rules
    from kicad_parser import parse_kicad_pcb
    refs = sorted({r_ for ln in caps if ln.startswith(('Stuck', 'Unresolved')) and ':' in ln
                   for r_ in re.findall(r'[A-Za-z_]+\d+\w*', ln.rsplit(':', 1)[1])})
    if not refs or not wayouts:
        return []
    with contextlib.redirect_stdout(io.StringIO()):
        pf = parse_kicad_pcb(field)
    cu = pf.board_info.copper_layers
    cl = _rules.active().clearance
    out = []
    for w in wayout_runs(wayouts, cu):
        (ax, ay), (bx, by) = w['start'], w['end']
        L2 = (bx - ax) ** 2 + (by - ay) ** 2
        for r_ in refs:
            for p_ in (pf.footprints[r_].pads if r_ in pf.footprints else ()):
                if p_.net_id == w['net_id'] or not (w['layer'] in p_.layers or '*.Cu' in p_.layers):
                    continue
                # (the pad's rectangle to the run's line: the nearest of the run's points to the pad's centre)
                t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p_.global_x - ax) * (bx - ax)
                                                             + (p_.global_y - ay) * (by - ay)) / L2))
                d = math.hypot(max(abs(ax + t * (bx - ax) - p_.global_x) - p_.size_x / 2, 0.0),
                               max(abs(ay + t * (by - ay) - p_.global_y) - p_.size_y / 2, 0.0)) - w['width'] / 2
                if d < cl - 1e-6:
                    out.append({'type': 'way-out', 'layer': w['layer'], 'net1': pf.nets[w['net_id']].name,
                                'net2': None, 'pad_ref': f'{r_}.{p_.pad_number}', 'closest_pt': [ax, ay]})
                    break
    return out


def write_field(board, out, laid, ties_of=(), tie_sites=(), pulls=(), wayouts=(), locked=()):
    """the arrays FANNED OUT as the model chose them, on a copy of the bare `board` (its project beside it): each
    piece of `laid` [(its net's id, its copper: joint_escape.Piece, the layer its run takes, the fan track)] as its own
    legs and its vias at the sizes the menu measured them at -- nothing planned again, nothing moved; two balls of one
    net dropped at one site share its via. With them THE TIE VIAS at the sites the menus were measured round
    (`tie_sites` [(the stand-in via, its ball (x, y, radius))]: fanout_from_plan.tie_stand_ins), each unless a via of
    its net is laid in its ball. Then the passives beneath the arrays are nudged off it (place_fanout_clearance
    --beneath-only), THE BUSES' PULL PADS OFF THEIR NETS meanwhile (`pulls`: route_bus.pulled_nets' stub pads, as in
    each bus's own step), and off THE LANES' WAYS OUT (`wayouts` [(net id, an exit, its direction, its layer)]: each
    exit's straight run out to where its lane can turn -- two lanes' bars on, its stub's mouth and a bar past it --
    laid for the nudge alone at the route's track and a grid step, a lane's bar off it at the clearance, and taken up
    after); the parts `locked` (main's `held`: the model keeps every option off their pads where they came) are left
    where they are. Returns (tracks written, vias written, the nudge's lines)"""
    import joint_escape as je
    import fanout_from_plan as fp
    import ship_vias
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    tracks, vias, seen = [], [], []
    for nid, pc, L, tw, _ref in laid:
        for a, b, Lg in pc.legs:
            tracks.append({'start': tuple(a), 'end': tuple(b), 'width': tw, 'layer': L if Lg == je.RUN else Lg,
                           'net_id': nid})
        for c, r, dr in pc.vias:
            # (two balls of one net dropped at one site share its via: joint_escape._hit's own tolerance)
            if not any(n2 == nid and math.hypot(c[0] - c2[0], c[1] - c2[1]) < je.SHARE_TOL for n2, c2 in seen):
                seen.append((nid, c))
                vias.append({'x': c[0], 'y': c[1], 'size': 2 * r, 'drill': 2 * dr, 'layers': ['F.Cu', 'B.Cu'],
                             'net_id': nid})
    # (THE TIE VIAS, laid here at the sites every menu was measured round, before the passives are nudged: laid after
    # it at a site read again from the nudged pad, one stood on a neighbour's run -- the site the plans kept clear was
    # the other one. One whose ball has a via of its net laid in it is tied by that via)
    for v_, (bx_, by_, rb_) in tie_sites:
        if not any(n2 == v_.net_id and math.hypot(c2[0] - bx_, c2[1] - by_) <= rb_ for n2, c2 in seen):
            vias.append({'x': v_.x, 'y': v_.y, 'size': v_.size, 'drill': v_.drill, 'layers': ['F.Cu', 'B.Cu'],
                         'net_id': v_.net_id})
    # (CHECKED before it is written: a pad some track of its net ends on has one of them on a layer it has copper on,
    # or a via of the net standing on it -- a stub on any other layer alone joins its pad to nothing, and no clearance
    # check sees it: the nets it left open were found by the buses' routes, half an hour on. A run on another layer
    # that only turns under its own ball is none of it: the ball's own stub is there too)
    # (...by PLACE: a ball over a pad of its own net on the other side is one place, either's layer joins it)
    pads_ = collections.defaultdict(list)
    for f_ in pcb.footprints.values():
        for p_ in f_.pads:
            if p_.net_id and p_.pad_type != 'np_thru_hole':
                pads_[p_.net_id].append(p_)
    place = {}                                     # (net, a pad's centre) -> [its pads there, the layers of the tracks ending there]
    for t_ in tracks:
        for e_ in (t_['start'], t_['end']):
            for p_ in pads_.get(t_['net_id'], ()):
                if math.hypot(e_[0] - p_.global_x, e_[1] - p_.global_y) < 1e-3:
                    k_ = (t_['net_id'], round(p_.global_x, 3), round(p_.global_y, 3))
                    if k_ not in place:
                        place[k_] = ([q_ for q_ in pads_[t_['net_id']]
                                      if math.hypot(q_.global_x - p_.global_x, q_.global_y - p_.global_y) < 1e-3], set())
                    place[k_][1].add(t_['layer'])
    loose = []
    for (nid_, x_, y_), (ps_, ls_) in sorted(place.items(), key=lambda t: t[0]):
        if not any('*.Cu' in q_.layers or ls_ & set(q_.layers) for q_ in ps_) \
                and not any(v_['net_id'] == nid_ and math.hypot(v_['x'] - x_, v_['y'] - y_) < v_['size'] / 2
                            for v_ in vias):
            loose.append(f'{ps_[0].component_ref}.{ps_[0].pad_number} ({ps_[0].net_name}) on {"/".join(sorted(ls_))}')
    if loose:
        raise SystemExit(f'group_ends: write_field: {len(loose)} pad(s) whose only stub runs on a layer the pad has no '
                         f'copper on, no via there: ' + ', '.join(sorted(loose)[:12]))
    # (THE LANES' WAYS OUT, for the nudge alone: a decap the nudge moved off a stub moved 0.2 mm into the straight run
    # out of the next ball's exit, which the ends had been chosen clear of -- the lane stood 0.021 mm short of its bar
    # there, in every round of its bus's loop. Held off the whole front a single lane's end is priced on, 1.2 mm, a
    # decap under the array had nowhere to go but onto the next balls' stubs: five lessons, a field laid six times)
    wo = wayout_runs(wayouts, pcb.board_info.copper_layers)
    w_wo = wo[0]['width'] if wo else None
    with contextlib.redirect_stdout(io.StringIO()):
        add_tracks_and_vias_to_pcb(board, out, tracks + wo, vias, [],
                                   net_id_to_name={i: n.name for i, n in pcb.nets.items()})
        fp.copy_pro(board, out)
        ship_vias.stamp(out, 'group ends')      # (a via in a pad or under solder declared filled and capped)
    # (the passives beneath the arrays nudged off the fanout, as every fanout's step does after it: the menus' vias were
    # measured as if those parts were not there -- joint_escape.movable_refs, the cap step's own set of them)
    # (...THE BUSES' PULL PADS OFF THEIR NETS meanwhile, as in each bus's own step (route_bus: a pulled net is laid
    # two-ended, its stub the rest of the chain's): on its net, a pull's pad was drawn by the nudge under its own
    # net's ball 1.4 mm off -- the net then read as one served by a via at that ball, which no plan had kept a site
    # for; the tie stood on a neighbour's run, was not laid, and the bus's route graded the net open)
    import subprocess
    import rules as _rules
    import route_bus as rb
    held = []
    if pulls:
        with open(out, encoding='utf-8') as f:
            txt_ = f.read()
        txt_, held = rb.unpull(txt_, pulls)
        with open(out, 'w', encoding='utf-8') as f:
            f.write(txt_)
    r = subprocess.run([sys.executable, os.path.join(HERE, '..', 'py_router', 'place_fanout_clearance.py'), out, out,
                        '--clearance', str(_rules.active().clearance), '--beneath-only']
                       # (a part HELD where it came stays there: moved for a neighbour's sake, it left the pads the
                       # model had kept clear and stood 1.4 mm off, on copper no rule had kept from it)
                       + (['--lock'] + sorted(locked) if locked else []),
                       env=awx_settings.environ(), capture_output=True, text=True)
    with open(out[:-len('.kicad_pcb')] + '.caps.log', 'w', encoding='utf-8') as f:
        f.write(r.stdout + r.stderr)
    if wo:
        # (...the ways out taken up again, every one of them, or the run stops: they are no copper of the field's)
        from kicad_writer import remove_segments_from_content
        with open(out, encoding='utf-8') as f:
            txt_ = f.read()
        txt_, n_wo = remove_segments_from_content(
            txt_, [types.SimpleNamespace(start_x=t_['start'][0], start_y=t_['start'][1], end_x=t_['end'][0],
                                         end_y=t_['end'][1], layer=t_['layer'], net_id=t_['net_id'], width=w_wo)
                   for t_ in wo], net_id_to_name={i: n.name for i, n in pcb.nets.items()}, match_width=True)
        if n_wo != len(wo):
            raise SystemExit(f'group_ends: write_field: {len(wo) - n_wo} of the {len(wo)} ways out laid for the '
                             f'nudge not found again to take up')
        with open(out, 'w', encoding='utf-8') as f:
            f.write(txt_)
    if held:
        with open(out, encoding='utf-8') as f:
            txt_ = f.read()
        txt_, missed_ = rb.repull(txt_, held)
        if missed_:
            raise SystemExit(f'group_ends: write_field: the pull pads {missed_} not found again to put back on their nets')
        with open(out, 'w', encoding='utf-8') as f:
            f.write(txt_)
    caps = [ln.strip() for ln in (r.stdout + r.stderr).splitlines() if ln.startswith(('Moved', 'Stuck', '  Unresolved',
                                                                                     'WARNING'))]
    # (...and THE TIE VIAS, last: a destination ball with a pad of its OWN net under it on the other side -- a
    # termination's under a clock ball -- is tied to it by a via at the ball, as every fanout's destination pass ties
    # them on the board it ships (fanout_from_plan.tie_vias_under: checked clear of foreign copper and drills, a
    # via-in-pad escape ties it by itself). `ties_of` [(the bus's nets, its destination array)]. Without them a bus's
    # clock pair and its reset were laid end to end and left open at their terminations, three nets of the board)
    ties = []
    if ties_of:
        from coherent_nets import by_key
        with contextlib.redirect_stdout(io.StringIO()):
            pf = parse_kicad_pcb(out)
        byk = by_key(pf)
        for names_, dref in ties_of:
            nms_ = [n for n in names_ if n in byk]
            dpad_ = {n: next((p for p in byk[n][1].pads if p.component_ref == dref), None) for n in nms_}
            ties += fp.tie_vias_under(pf, nms_, byk, dpad_, ties, log=lambda *a: caps.append(' '.join(map(str, a)).strip()))
        if ties:
            tmp = out[:-len('.kicad_pcb')] + '_tie.kicad_pcb'
            with contextlib.redirect_stdout(io.StringIO()):
                add_tracks_and_vias_to_pcb(out, tmp, [], ties, [], net_id_to_name={i: n.name for i, n in pf.nets.items()})
            os.replace(tmp, out)
            fp._drop_tie_siblings(tmp)
            with contextlib.redirect_stdout(io.StringIO()):
                ship_vias.stamp(out, 'group ties')
    return len(tracks), len(vias) + len(ties), caps + ([f'the nudge exited {r.returncode}'] if r.returncode else [])


def place_round(box, d, p):
    """where an exit `p` on face `d` lies round `box`, clockwise from its north-west corner (thousandths of a mm)"""
    x0, y0, x1, y1 = box
    W, H = x1 - x0, y1 - y0
    x, y = min(max(p[0], x0), x1), min(max(p[1], y0), y1)
    s = {'up': x - x0, 'right': W + (y - y0), 'down': W + H + (x1 - x), 'left': 2 * W + H + (y1 - y)}[d]
    return int(round(s * 1000))


CHECK_DUMP = None           # (main: GROUP_CHECK_DUMP) a directory each check's bench is written to, or None
ROW_STAGGERS = 1            # a fine-pitch row part's fan-out zone: this many rows of vias past the stubs' ends


def row_zone(grid, direction):
    """A FINE-PITCH ROW PART's fan-out zone along a pad's escape `direction` -- (stagger, out) in mm past the stub's end
    its own menu gives it, or None where its pads stand a via and a clearance (and a grid step) apart or more: `stagger`
    the least step along the stubs at which two neighbours' vias clear each other by a grid step, `out` the reach of
    ROW_STAGGERS rows of vias and a via's room for a lane at the route's own track beyond the last -- the room the
    part's fan (qfn_berths) and the vias at its tips take, which the prepared board's passives are moved off (main:
    route_bus.clear_row_escapes)"""
    import braid as bd
    import escape_moves as em
    import rules as _rules
    rl = _rules.active()
    gap, step = bd.VIA_SIZE + rl.clearance, rl.grid
    pitch = grid.pitch_y if em.DIRS[direction][0] else grid.pitch_x
    if pitch >= gap + step:
        return None
    up = lambda v: math.ceil(v / step - 1e-9) * step   # noqa: E731
    stagger = up(math.sqrt((gap + step) ** 2 - pitch ** 2))
    return stagger, ROW_STAGGERS * stagger + up(bd.VIA_SIZE / 2 + rl.clearance + bd.TRACK / 2 + step)


def _stub_path(segs, p0):
    """a fanned stub's segments [(a, b, layer)] as one polyline from the end nearest its pad's centre p0 (none: [])"""
    left = [(tuple(a), tuple(b)) for a, b, _L in segs]
    if not left:
        return []
    d = lambda q: math.hypot(q[0] - p0[0], q[1] - p0[1])        # noqa: E731
    a, b = min(left, key=lambda s: min(d(s[0]), d(s[1])))
    left.remove((a, b))
    path = [a, b] if d(a) <= d(b) else [b, a]
    while left:
        nx = next(((s, s[1] if math.dist(s[0], path[-1]) < 1e-6 else s[0]) for s in left
                   if min(math.dist(s[0], path[-1]), math.dist(s[1], path[-1])) < 1e-6), None)
        if nx is None:
            return []
        left.remove(nx[0])
        path.append(nx[1])
    return path


def _steps(length, h):
    """0, h, 2h, ... up to `length`, and `length`"""
    k = max(1, int(math.ceil(length / h - 1e-9)))
    return [length * i / k for i in range(k + 1)]


def _walk_back(path, h):
    """(point, the outward heading there) every h back from the polyline's far end to its first bend, the end first"""
    for i in range(len(path) - 1, 1, -1):
        a, b = path[i - 1], path[i]
        L = math.dist(a, b)
        if L < 1e-9:
            continue
        u = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
        for r in _steps(L, h):
            yield (b[0] - u[0] * r, b[1] - u[1] * r), u


def _cut_path(path, q, layer):
    """the polyline's segments from its start to point q on it (as [(a, b, layer)])"""
    out = []
    for a, b in zip(path, path[1:]):
        if _pt_seg_d(q, a, b) < 1e-6:
            if math.dist(a, q) > 1e-9:
                out.append((a, q, layer))
            return out
        out.append((a, b, layer))
    return out


def _pt_seg_d(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L2 = ax * ax + ay * ay
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L2))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def qfn_berths(st, pcb):
    """a bus's destination menu (`st['dmenu']`, its own run's) with each ROW PART FANNED OUT FIRST BY THE QFN FANOUT
    (py_router/qfn_fanout, its surface fan: a straight stub and a diverging 45-degree fan at the fanout's track, each
    stub checked against the board and the others, the tips spread so that vias fit between them) -- the tool the
    chain fans a QFN out with, where a row pad's own two moves here (its stub straight out, that stub with a via at
    its end) leave a row of lanes that arrive on other layers nowhere to change: two neighbours' vias at their stubs'
    ends cannot both be laid at a fine pitch, and the solve's own changes there stand a few hundredths of a mm from
    their bars. Each net the fan escapes has, in place of its pad's own moves, its FANNED stub as its berth: ending at
    the stub's tip, plain or with a via there where one has room on every copper layer (its lane then leaves on any
    layer). The stub's own copper rides on the move (`fan_legs`: laid with the berth, bus_plan) and not in its legs
    -- the ends' conflict test reads axis-aligned lanes, and the fan's stubs are already clear of each other. A pair's
    legs keep their own moves (the fan would part them), as does a net the fan could not escape, and every net of a
    part whose pads have more moves of their own than those two. The fan is held inside the board's edge rule (the
    fanout's own edge model, given the board's rule), and so is a tip's via. A tip is one the lane can LEAVE: it, and
    its stub's line on for a lane's bar, stand that bar off other copper on its layer -- else it is walked back along
    its own diagonal to where they do (and a via there held off the fan's other stubs)"""
    import dataclasses
    import braid as bd
    import pairs as _pairs
    import route_layers
    import rules as _rules
    from qfn_fanout import generate_qfn_fanout
    rl = _rules.active()
    RL = route_layers.layers()
    cu = list(pcb.board_info.copper_layers or ())
    kids = {st['byname'][n][0] for n in st['launch']}
    legs_ = {leg for pr in _pairs.pair_names(list(st['launch'])).values() for leg in pr}
    out = {n: list(v) for n, v in st['dmenu'].items()}
    by_ref = collections.defaultdict(list)
    for n in st['launch']:
        pad = st['dst_pad'].get(n)
        grid = ((st.get('dgrid_of') or {}).get(pad.component_ref) or st['dgrid']) if pad is not None else None
        if grid is not None and getattr(grid, 'row', False) and n not in legs_ and out.get(n):
            by_ref[pad.component_ref].append(n)
    obs_ = {}

    def obs(nid, L):
        if (nid, L) not in obs_:
            obs_[(nid, L)] = bd.build_obstacles(pcb, nid, kids, L)
        return obs_[(nid, L)]
    # (THE BOARD'S EDGE RULE, given to the fanout -- which shortens a fan that would pass it -- and held by the via
    # at a stub's tip here: asked with none, the fanout held its stubs off the edge by the track clearance alone,
    # and a part whose row faces the board's edge 1.6 mm off had five stubs and their vias laid past the board's
    # own rule -- every berth of those nets ruled out once the field was measured, and the model no state)
    cfg = bd.run_config(pcb, st.get('base'))
    edge_ = float(getattr(cfg, 'board_edge_clearance', 0.0) or 0.0) or cfg.clearance
    on_ = lambda q, half: bd._on_board(pcb, q[0], q[1], edge_ + half)       # noqa: E731
    # (a lane's bar off copper where it lies off the grid: its track's half, the clearance, half a grid step -- the
    # audit's, plan_audit.static)
    lane_need = cfg.track_width / 2 + cfg.clearance + cfg.grid_step / 2
    for ref, names in sorted(by_ref.items()):
        # (ONLY A PART WHOSE PADS HAVE NO MOVE OF THEIR OWN but a stub and that stub with a via. One with more -- a
        # two-row connector with room for vias between its rows, a dozen moves a pad -- keeps them: fanned too, its
        # lanes arrived among its fanned stubs and its own pads, 38 findings in a bus's first audit for 6 and 13 of
        # a board's 78 bus nets open for none)
        if any(len(out[n]) > 2 for n in names):
            continue
        foot = pcb.footprints[ref]
        # (the board without the run's own copper beside the part: the fan lays every one of its nets' stubs anew)
        xs, ys = [p.global_x for p in foot.pads], [p.global_y for p in foot.pads]
        box = (min(xs) - 3.0, min(ys) - 3.0, max(xs) + 3.0, max(ys) + 3.0)
        inb = lambda x, y: box[0] <= x <= box[2] and box[1] <= y <= box[3]     # noqa: E731
        bare = copy.copy(pcb)
        bare.segments = [s_ for s_ in pcb.segments if not (s_.net_id in kids and inb(s_.start_x, s_.start_y))]
        bare.vias = [v_ for v_ in pcb.vias if not (v_.net_id in kids and inb(v_.x, v_.y))]
        full = [st['byname'][n][1].name for n in names]
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            tracks, _vias, failed = generate_qfn_fanout(
                foot, bare, net_filter=full, layer=foot.layer, track_width=rl.fan_track, clearance=rl.clearance,
                grid_step=rl.grid, escape_method='stub', via_size=bd.VIA_SIZE, via_drill=bd.VIA_DRILL,
                net_clearances={}, clearance_ceiling=rl.clearance, board_edge_clearance=edge_)
        by_net = collections.defaultdict(list)
        for t_ in tracks:
            by_net[t_['net_id']].append((tuple(t_['start']), tuple(t_['end']), t_['layer']))
        for n in names:
            nid, pad = st['byname'][n][0], st['dst_pad'][n]
            segs = by_net.get(nid)
            if not segs or st['byname'][n][1].name in set(failed or ()):
                continue
            p0 = (pad.global_x, pad.global_y)
            pts = [q for a, b, _L in segs for q in (a, b)]
            tip = max(pts, key=lambda q: math.hypot(q[0] - p0[0], q[1] - p0[1]))
            b0 = out[n][0]
            home = foot.layer
            moved = False
            # (A TIP THE LANE CAN LEAVE: the fan shortens a stub at the clearance from what it meets, and a stub
            # turned 45 degrees toward a passive beside the part ended facing its pad at the fan's clearance -- the
            # lane, at its own track and its bar off the grid, could neither go on nor turn there, and the loop found
            # it short at that berth in every round. Walked back along its own diagonal to where the tip, and its line
            # on for a lane's bar, stand that bar off other copper on its layer; none: the fan's tip as it was)
            path = _stub_path(segs, p0)
            if len(path) > 2:
                obs_h = obs(nid, home)
                grow = lane_need - (bd.CLEAR + bd.TRACK / 2)
                free_ = lambda q: not (obs_h.point_violation(q, pad=grow) or [0])[0]           # noqa: E731
                for q, u in _walk_back(path, rl.grid / 2):
                    if all(free_((q[0] + u[0] * r, q[1] + u[1] * r)) for r in _steps(lane_need, rl.grid / 2)):
                        if q != tip:
                            segs, tip, moved = _cut_path(path, q, home), q, True
                        break
            ms = dataclasses.replace(b0, kind='surface', layer=home, exit_pt=tip, vias=0, legs=[], site=None)
            ms.fan_legs = list(segs)
            new = [ms]
            vl = cu if len(RL) > 2 else [home]
            # (a via at a tip walked back stands nearer the fan's other stubs than the fan spread it from: held off them)
            via_off = (not moved) or all(
                _pt_seg_d(tip, a_, b_) >= bd.VIA_SIZE / 2 + rl.clearance + rl.fan_track / 2
                for t_ in tracks if t_['net_id'] != nid for a_, b_ in [(tuple(t_['start']), tuple(t_['end']))]
                if t_['layer'] == home)
            if on_(tip, bd.VIA_SIZE / 2) and via_off and \
                    not any((obs(nid, L).point_violation(tip, pad=(bd.VIA_SIZE - bd.TRACK) / 2) or [0])[0] for L in vl):
                mv = dataclasses.replace(b0, kind='dogbone', layer=home, exit_pt=tip, vias=1, legs=[], site=tip)
                mv.fan_legs = list(segs)
                new.append(mv)
            out[n] = new
    return out


def kept_copper(bpcb, cpcb, arrays):
    """{array: ([segments], [vias])} of the board `bpcb` wholly inside each of the group's `arrays` (their pads' box
    and BARE round it) that `cpcb`, the board the buses' ends models were captured on, has not: a group's escalation
    captures the buses it routes again on the kept board with their arrays bare (route_bus._bare_arrays), and what
    stands there -- the other buses' escapes, the field's stubs of the other balls -- is the board's all the same"""
    r3 = lambda q: (round(q[0], 3), round(q[1], 3))        # noqa: E731
    nm = lambda p_, i_: p_.nets[i_].name if i_ in p_.nets else None     # noqa: E731
    had = {(frozenset((r3((s_.start_x, s_.start_y)), r3((s_.end_x, s_.end_y)))), s_.layer, nm(cpcb, s_.net_id))
           for s_ in cpcb.segments}
    hadv = {(r3((v_.x, v_.y)), nm(cpcb, v_.net_id)) for v_ in cpcb.vias}
    out = {}
    for ref, A in arrays.items():
        x0, y0, x1, y1 = A.box[0] - BARE, A.box[1] - BARE, A.box[2] + BARE, A.box[3] + BARE
        inb = lambda x, y, x0=x0, y0=y0, x1=x1, y1=y1: x0 <= x <= x1 and y0 <= y <= y1      # noqa: E731
        out[ref] = (
            [s_ for s_ in bpcb.segments if not getattr(s_, 'graphic', False)
             and inb(s_.start_x, s_.start_y) and inb(s_.end_x, s_.end_y)
             and (frozenset((r3((s_.start_x, s_.start_y)), r3((s_.end_x, s_.end_y)))), s_.layer,
                  nm(bpcb, s_.net_id)) not in had],
            [v_ for v_ in bpcb.vias if inb(v_.x, v_.y) and (r3((v_.x, v_.y)), nm(bpcb, v_.net_id)) not in hadv])
    return out


def load_bus(name, cap, arrays, log, cut=None, rank=False, feedback=None, bare_ids=None, board_cu=None):
    """a bus's ends model from its capture, under its own run's settings, its ends at the group's ARRAYS (`arrays`
    {ref: build_array's}: its source, and its destination where that is one) chosen from their menus: each such array
    bare in its board (its own fanout's copper there left out), the moves its balls are offered there (A.offer: the
    pattern's, a widened ball's whole menu) each turned into the bus's frame. Its start: the ends it chose alone, each
    the nearest of the moves offered. `bare`: its (lane or net, end)s with no clean option among them"""
    with awx_settings.given(cap['env']):
        import rules as _rules
        _rules.install_defaults()
        import whole_ends as we
        t0 = time.time()
        st = dict(cap['st'])
        arr = {0: arrays[st['sref']], 1: arrays.get(st['dref']) if len(st.get('dests') or [st['dref']]) == 1 else None}
        pads = {p.pad_number: (p.global_x, p.global_y) for p in st['pcb'].footprints[st['sref']].pads}
        F, G = frame_map(pads, arr[0].pads), frame_map(arr[0].pads, pads)   # (the bus's frame to the board's, and back)
        pcb = copy.copy(st['pcb'])
        n0 = (len(pcb.segments), len(pcb.vias))
        for A in arr.values():
            if A is None:
                continue
            ps = [(p.global_x, p.global_y) for p in pcb.footprints[A.ref].pads]
            x0, y0 = min(p[0] for p in ps) - BARE, min(p[1] for p in ps) - BARE
            x1, y1 = max(p[0] for p in ps) + BARE, max(p[1] for p in ps) + BARE
            inb = lambda x, y, x0=x0, y0=y0, x1=x1, y1=y1: x0 <= x <= x1 and y0 <= y <= y1      # noqa: E731
            pcb.segments = [s for s in pcb.segments if not (inb(s.start_x, s.start_y) and inb(s.end_x, s.end_y))]
            pcb.vias = [v for v in pcb.vias if not inb(v.x, v.y)]
        # (...and THE BOARD'S OWN COPPER THERE THE CAPTURE'S BOARD HAS NOT, in the bus's view (`board_cu`,
        # kept_copper): turned into the bus's frame, each piece on its net there. Unknown to it, a berth's run was
        # planned across the stub of a ball the board had fanned out already, at the array's rim -- the same finding
        # at that end every round of the loop, and the ball walled in)
        ids_ = {n_.name: i_ for i_, n_ in pcb.nets.items() if n_.name}
        for A in arr.values():
            if A is None or not (board_cu or {}).get(A.ref):
                continue
            more_s, more_v = [], []
            for s_, n_ in board_cu[A.ref][0]:
                if n_ in ids_:
                    t_ = copy.copy(s_)
                    (t_.start_x, t_.start_y), (t_.end_x, t_.end_y) = G.pt((s_.start_x, s_.start_y)), \
                        G.pt((s_.end_x, s_.end_y))
                    t_.net_id = ids_[n_]
                    more_s.append(t_)
            for v_, n_ in board_cu[A.ref][1]:
                if n_ in ids_:
                    t_ = copy.copy(v_)
                    t_.x, t_.y = G.pt((v_.x, v_.y))
                    t_.net_id = ids_[n_]
                    more_v.append(t_)
            pcb.segments, pcb.vias = list(pcb.segments) + more_s, list(pcb.vias) + more_v
        # (the FRONTS read on this board -- whole_ends._front's, kept per exit -- held from one building of the bus's
        # model to the next: the board is the capture's with the same copper left out each time, read again only if
        # it is not. Read afresh at every building, a 53-lane bus's fronts were a fifth of each)
        of_ = (len(pcb.segments), len(pcb.vias),
               hash(tuple((s.start_x, s.start_y, s.end_x, s.end_y, s.layer, s.net_id) for s in pcb.segments)),
               hash(tuple((v.x, v.y, v.net_id) for v in pcb.vias)),
               hash(tuple((p.global_x, p.global_y, p.net_id) for f in pcb.footprints.values() for p in f.pads)))
        if cap.get('_fronts_of') != of_:
            cap['_fronts_of'], cap['_fronts'], cap['_ends_memo'] = of_, {}, {}
        pcb.__dict__['_exit_front'] = cap['_fronts']
        st['pcb'] = pcb
        # (...and, held the same way, what the ends model works out of an end's moves alone -- which of them cannot be
        # laid together, which ways down of the plane balls each takes: whole_ends' `ends_memo`, read again only while
        # that end's moves are the very ones. A row part's berths are: its own run's menu at every building)
        st['ends_memo'] = cap.setdefault('_ends_memo', {})
        for k_, A in arr.items():
            if A is None:
                continue
            missing = sorted(n for n in st['launch'] if n not in A.ball_of)
            if missing:
                raise SystemExit(f'group_ends: {name}: no ball on {A.ref} in its menu for {missing}')
            menu = {}
            for n in st['launch']:
                k = A.ball_of[n]
                menu[n] = [turn_move(A.bm.menu[k][i], G) for i in A.offer[k]]
            # (a SINGLE's run, on no one layer yet, keeps the layers its exit's FRONT is clear on -- the straight run
            # out of it at the lane's own size, whole_ends._front, layer by layer: the ends model passes a via end whose
            # front is clear on ANY layer, the route's to choose there, and here the layer is the model's. A berth's run
            # put on the back layer 0.038 mm short of a resistor's pad in front of it, and its net laid open)
            import joint_escape as je
            import pairs as _pairs
            legs_ = {leg for pr in _pairs.pair_names(list(st['launch'])).values() for leg in pr}
            kids_ = {st['byname'][n][0] for n in st['launch']}
            for n in sorted(menu):
                if n in legs_:
                    continue
                for mv in menu[n]:
                    if mv.layer == je.RUN:
                        mv.runs_on = tuple(L for L in mv.runs_on if we._front(
                            pcb, st['byname'][n][0], types.SimpleNamespace(
                                direction=mv.direction, exit_pt=mv.exit_pt, layer=L, kind='surface'), kids_)[0] == 0)
                menu[n] = [mv for mv in menu[n] if mv.layer != je.RUN or mv.runs_on]    # (no layer left: no option)
            st[('smenu', 'dmenu')[k_]] = menu
            st[('src_shared', 'dst_shared')[k_]] = True
        if arr[1] is None:
            st['dmenu'] = qfn_berths(st, pcb)
        st['sblock'] = {}
        if feedback is not None:
            # (what the bus's own step found crowded at these ends, whole_feedback's file: the ends model's own
            # tables of it -- E.fb_avoid, E.fb_pairs -- which the group rules out where the place is the very one)
            st['feedback'] = feedback
        with contextlib.redirect_stdout(io.StringIO()):
            E = we.Ends(st, src_free=True, fixed=None if arr[1] is not None else cap.get('fixed'),
                        learned=None if arr[1] is not None else cap.get('learned'))
        t_e = time.time()
        last = cap.get('last') or {}
        # (the destination's cut: a ball array's far face's middle, as the source's is read -- each lane round it the
        # nearer way. A row part's, or a group's, the capture's own. Left as a bus's own run had chosen it for its own
        # ends, a winding cut sent the group's berths the long way round an array: 119 places two of the bus's lanes
        # lay on one line there)
        E.wcut = (E.dbox[1] + E.dbox[3]) / 2 if arr[1] is not None else last.get('cut')
        E._nogoods, E._nochg, E._nocross = [], set(), set()
        B = types.SimpleNamespace(name=name, cap=cap, env=cap['env'], E=E, F=F, G=G, arr=arr)
        # (a net's id on the bus's own board against its id on the group's bare one, BY NAME: a capture's board may
        # number its nets otherwise -- taken as one, a bus's berths at a row part were laid each on the next net of
        # the row, and the arrays' copper put on its check's bench under other nets' ids)
        B.to_bare = {i: bare_ids[n.name] for i, n in st['pcb'].nets.items() if n.name and n.name in (bare_ids or {})}
        B.to_cap = {j: i for i, j in B.to_bare.items()}
        B.leg_lane = {st['byname'][leg][1].name: lane for lane, lg in E.lanes for leg in lg if leg in st['byname']}
        laned = {l_ for _l, lg in E.lanes for l_ in lg}
        B.bare = [(lane, 0) for lane, _lg in E.lanes if all(o[4] for o in E.T[lane])]
        B.bare += [(lane, 1) for lane, _lg in E.lanes if arr[1] is not None and all(o[4] for o in E.B[lane])]
        # (a net of the bus the ends hold no lane for: no option at all at an end among the moves offered)
        B.bare += [(n, 1 if not st['dmenu'].get(n) else 0) for n in st['launch'] if n not in laned]
        if B.bare:
            return B
        want = {0: cap.get('teeth') or {}, 1: cap.get('berths') or {}}
        je_ = last.get('joint_ends') or {}
        own = {}
        for lane, lg in E.lanes:
            ij = []
            for k_, opts in ((0, E.T[lane]), (1, E.B[lane])):
                w_ = [want[k_].get(l_) for l_ in lg]
                clean = [i for i, o in enumerate(opts) if not o[4]]
                i = None
                if arr[k_] is None:
                    # (a row part's berth: the option of its signature -- a GROUP berth's way in with its moves, the
                    # same move being offered once per way -- or the very move)
                    if je_.get(lane):
                        sig = json.dumps(je_[lane][k_])
                        i = next((i for i in clean if json.dumps(we._opt_sig(opts[i])) == sig), None)
                    if i is None:
                        i = next((i for i in clean if len(opts[i][0]) == len(w_)
                                  and all(a is b for a, b in zip(opts[i][0], w_))), None)
                elif all(w is not None for w in w_):
                    i = min(clean, key=lambda i: (sum(m.direction != w.direction for m, w in zip(opts[i][0], w_)),
                                                  sum(math.hypot(m.exit_pt[0] - w.exit_pt[0], m.exit_pt[1] - w.exit_pt[1])
                                                      for m, w in zip(opts[i][0], w_))))
                ij.append(i if i is not None else min(clean, key=lambda i: opts[i][3]))
            own[lane] = tuple(ij)
        B.own, B.boxes = own, we.joint_boxes(E, own)
        # (a row part's or a group's CUT where the capture records none -- one taken before the bus's own ends were
        # searched (negotiated_buses.route_grouped): the cut the ends' own score takes for the ends as they start)
        if E.wcut is None and arr[1] is None:
            # (...UNTIL THE BUS HAS A STATE, as an array destination's below (`cut`: that state's): scored again at each
            # later building -- on a stretch's menus the start's ends are other options here and there -- a row part's
            # cut moved between two models of one search, 4 mm along its walk, and each building paid the score's
            # whole kernel for it, two to three seconds of a 74-lane bus's)
            E.wcut = cut[0] if cut is not None else E.score(own)[1].get('cut')
        # (what the group prices beside a bus's own ends, for its descents: a tooth on the shared source's FAR FACE --
        # the bus's own ends price it as a side face there, the group past any count of vias (main: `far`). Without
        # it the descents' ends put nine lanes of a four-bus group on far-face teeth, the start's with them)
        B.dear = {(lane, 0): {i: we.FAR_TOOTH * len(lg) for i, o in enumerate(E.T[lane])
                              if o[0] and o[0][0].direction == 'left'} for lane, lg in E.lanes}
        if arr[1] is not None:
            # (THE START, as a bus's own ends chose theirs: its own run's ends DESCENDED at the cut
            # (whole_ends.joint_descent). THE CUT round a ball array destination: its far face's middle -- each lane
            # round it the nearer way -- until the whole solve refuses the bus's ends there; then, with the lines
            # (`rank`), the far face's and one in each gap between the start's berths, each ranked by its descent, the
            # best taken (whole_ends.ranked_cuts): at the far face's alone an escape bus's ends had twice and more the
            # crossings of its own run's and no plan. Ranked from the start for every bus, a bus that shares its source
            # took a winding cut its descent -- which sees no other bus -- read as free of crossings, and had half
            # as many crossings again in the group. Chosen until the bus has a state (`cut`: that state's, a ball or two of its
            # menu since widened): chosen again at each later building, it moved between two models of one search)
            fc = lambda c_: (f'{c_[0]} {float(c_[1]):.2f}' if isinstance(c_, (tuple, list))     # noqa: E731
                             else f'far face {float(c_):.2f}')
            if cut is not None or not rank:
                E.wcut = cut[0] if cut is not None else E.wcut
                B.own = we.joint_descent(E, own, B.boxes, E.wcut, dear=B.dear)[0]
            else:
                ranked = we.ranked_cuts(E, own, B.boxes, E.wcut, dear=B.dear)
                _c, _q, E.wcut, B.own, _nx = ranked[0]
                log(f'  {name}: its cut round {st["dref"]} by the descents: '
                    + ', '.join(f'{fc(c_)} {nx_} x' for _c2, _q2, c_, _s2, nx_ in ranked[:6])
                    + (f', ... ({len(ranked)} cuts)' if len(ranked) > 6 else ''))
        nt = sorted(len(E.T[l_]) for l_, _g in E.lanes)
        nb = sorted(len(E.B[l_]) for l_, _g in E.lanes)
        log(f'  {name}: {len(E.lanes)} lanes ({sum(1 for _l, g in E.lanes if len(g) > 1)} pairs), {sum(nt)} tooth '
            f'options (a lane {nt[0]} to {nt[-1]}) and {sum(nb)} berth options (a lane {nb[0]} to {nb[-1]}'
            + (f', {st["dref"]}\'s own menu' if arr[1] is not None else ', its own run\'s menu')
            + f'); its own fanout\'s {n0[0] - len(pcb.segments)} tracks and {n0[1] - len(pcb.vias)} vias at the arrays '
            f'left out; built in {time.time() - t0:.0f} s (its ends model {t_e - t0:.0f} s)')
        return B


def group_walk(E, edge, bar, pitch, step=0.25):
    """A row-part GROUP's WALK off its front (whole_frame.Group: from the front's south end f1 round to its north end
    f0), read for its LENGTH and its ROOM: (gl, way_mm, rooms). `gl` the walk's length off the front; `way_mm(o)` the
    length a berth option `o` (its way its sixth element) adds -- an 'on' berth off the front its walk from the front's
    nearer end, as the bus's own model rides it (Ends.ride), any other way its own length; `rooms` [(u, lanes)] at each
    `step` of the walk (u from f1): the lanes that stand a `pitch` apart between the parts' copper -- `bar` off it --
    and the board's edge (braid._on_board, `edge` in), round to their berths there"""
    import braid as bd
    from shapely.geometry import Point
    G, pcb = E.group, E.st['pcb']
    gl = (G.f0 - G.f1) % G.L

    def way_mm(o):
        w_ = o[5]
        if w_[0] != 'on':
            return float(w_[3])
        if G.in_front(w_[1]):
            return 0.0
        u_ = (w_[1] - G.f1) % G.L
        return min(u_, gl - u_)
    rooms = []
    for i in range(int(gl / step) + 1):
        u = min(gl, i * step)
        p, a, c = G.pt(G.f1 + u), G.pt(G.f1 + u - 0.05), G.pt(G.f1 + u + 0.05)
        tl = math.hypot(c[0] - a[0], c[1] - a[1]) or 1.0
        nx, ny = -(c[1] - a[1]) / tl, (c[0] - a[0]) / tl
        if G.U.contains(Point(p[0] + 0.1 * nx, p[1] + 0.1 * ny)):
            nx, ny = -nx, -ny                       # (outward)
        r = 0.0
        while r < 3.0 and bd._on_board(pcb, p[0] + (r + 0.05) * nx, p[1] + (r + 0.05) * ny, edge):
            r += 0.05
        rooms.append((u, int(math.floor((r - bar) / pitch + 1e-9)) + 1 if r >= bar else 0))
    return gl, way_mm, rooms


def walk_lanes(E, o, gl):
    """the stretch (u0, u1) of a GROUP's walk off its front (u from f1) that a berth option `o` rides to its place -- an
    'on' berth off the front, from the front's nearer end -- or None"""
    G, w_ = E.group, o[5]
    if w_[0] != 'on' or G.in_front(w_[1]):
        return None
    u_ = (w_[1] - G.f1) % G.L
    return (0.0, u_) if u_ <= gl - u_ else (u_, gl)


def way_round(E, k_, pos, boxes, cut):
    """the length (thousandths of a mm) a lane travels ROUND an array from an end at `pos` -- its place round the box as
    the bus's own model unrolls it (whole_frame.perim_s at the source, perim_c at the cut round the destination) -- to
    the face toward its other end: nothing on that face, else the way to its nearer corner, never across the cut"""
    import whole_frame
    Q = lambda v_: int(round(v_ * 1000))        # noqa: E731
    SB, DB = boxes
    if k_ == 0:
        fc = [Q(whole_frame.perim_s(c_, SB, E.shared_scut() if E.src_shared else None))
              for c_ in ((SB[2], SB[1]), (SB[2], SB[3]))]
    else:
        fc = [Q(whole_frame.perim_c(c_, DB, cut)) for c_ in ((DB[0], DB[1]), (DB[0], DB[3]))]
    return max(0, min(fc) - pos, pos - max(fc))


def lane_run(b, l_, i, j):
    """(kind, length) of lane `l_` of bus `b` on tooth option i and berth option j: its kind -- one net or a pair, the
    face its tooth leaves by, the face its berth is reached by -- and the length (mm) from its tooth's exit to its
    berth's along the way from the source's middle to the destination's. The whole solve gives a lane's change of
    layer a room at each end; a lane shorter than the two has none, and says so (its check's `nochange`): the
    longest such length of a kind is what the group's model learns no change fits in (learned 'room')"""
    (sx0, sy0, sx1, sy1), (dx0, dy0, dx1, dy1) = b.boxes
    ux, uy = (dx0 + dx1 - sx0 - sx1) / 2, (dy0 + dy1 - sy0 - sy1) / 2
    n_ = math.hypot(ux, uy) or 1.0
    ot, ob = b.E.T[l_][i], b.E.B[l_][j]
    return (len(ot[0]), ot[0][0].direction, ob[0][0].direction), \
        ((ob[1][0] - ot[1][0]) * ux + (ob[1][1] - ot[1][1]) * uy) / n_


def array_rows(m, A, users, gate, RL, pk, off, log, wide=(), dropped=False, oth_ban=(), held=()):
    """ONE ARRAY's rules, added to the model `m`: `users` [(bus, end)] the buses with lanes at array `A` and by which
    end (0 their teeth, 1 their berths). Every lane at the array alike -- the buses', and the array's other balls',
    made lanes here (each served: fanned out by a move of the menu, a plane's ball that or dropped) -- is held by the
    VIAS' groups and a POINT one lane a layer, and a drop clear of every option it cannot be laid beside. `gate(*name,
    lanes=)`: a rule's assumption. Returns a namespace: `others` {ball: (its literals by menu index, its layer's, its
    drops')}, `cost` and `plane` (the other balls' terms of the objective), `lits` (sets of literals of which one is
    taken: the weights' bound), `stuck` (the bus lanes (bus, lane, end) with no option left among the moves offered)"""
    import joint_escape as je
    import whole_ends as we
    bm, NL, oname = A.bm, len(RL), f'{OTHER}:{A.ref}'
    xs, ys, dom, where = {}, {}, {}, collections.defaultdict(list)
    exits = collections.defaultdict(list)       # a lane's option -> [where each of its legs' exits lies round the array]
    # (...and ALONG its face, the exit's own coordinate there: an exit past the face's end -- a side-stepped escape at a
    # corner -- lies round the box AT the corner, with every other exit past it)
    along = collections.defaultdict(list)
    along_ = lambda d_, p_: int(round((p_[1] if d_ in ('left', 'right') else p_[0]) * 1000))     # noqa: E731
    dead = set()
    for b, k_ in users:
        for lane, _lg in b.E.lanes:
            key = (b.name, lane, k_)
            xs[key], ys[key] = b.J.xs[k_][lane], b.J.y[k_][lane]
            opts = (b.E.T if k_ == 0 else b.E.B)[lane]
            for i in xs[key]:
                dom[(key, i)] = b.J.dom[(k_, lane, i)]
                for mv in opts[i][0]:
                    where[mv.gid[1:]].append((key, i))
                    g0 = bm.menu[mv.gid[1]][mv.gid[2]]
                    exits[(key, i)].append((place_round(A.box, g0.direction, g0.exit_pt), g0.direction))
                    along[(key, i)].append((g0.direction, along_(g0.direction, g0.exit_pt)))
            # (not offered: a single's exit blocked in front on every layer it could take with no room for a via
            # before the block -- priced past any via count for a bus alone, which keeps it where a lane has no other)
            dead.update((key, i) for i, c_ in (b.E.front.get((lane, k_)) or {}).items()
                        if c_ == we.FRONT_BLOCKED and i in xs[key])
    # ---- the array's OTHER balls: lanes at the array as the buses' are
    others, cost, plane, lits, fans = {}, [], [], [], []
    for k in A.okeys:
        key = (oname, k, 0)
        offered = A.offer[k]
        xs[key] = {i: m.NewBoolVar('') for i in offered}
        ys[key] = [m.NewBoolVar('') for _ in range(NL)]
        for i in offered:
            mv = bm.menu[k][i]
            # (a move on ONE layer -- a surface escape, or a via move where the routing layers are two and its run's
            # layer is the other one -- fixed there; a run on no one layer yet (joint_escape.RUN) on those it clears.
            # Read as a RUN whatever its layer, every dog-bone of a two-layer board's array had no `runs_on`)
            dom[(key, i)] = (RL.index(mv.layer), None, None) if mv.layer != je.RUN else \
                (None, None, frozenset(RL.index(L_) for L_ in mv.runs_on if L_ in RL))
            where[(k, i)].append((key, i))
            exits[(key, i)].append((place_round(A.box, mv.direction, mv.exit_pt), mv.direction))
            along[(key, i)].append((mv.direction, along_(mv.direction, mv.exit_pt)))
        # (a plane ball's drops, then its straps)
        dv = [m.NewBoolVar('') for _ in list(A.drops.get(k, ())) + list(A.straps.get(k, ()))]
        m.Add(sum(xs[key].values()) + sum(dv) <= 1)
        m.Add(sum(xs[key].values()) + sum(dv) >= 1).OnlyEnforceIf(gate('served', A.ref, k, lanes={key}))
        m.Add(sum(ys[key]) == sum(xs[key].values()))
        for q in range(NL):
            m.Add(ys[key][q] <= sum(v for i, v in xs[key].items()
                                    if (dom[(key, i)][0] == q if dom[(key, i)][0] is not None else q in dom[(key, i)][2])))
        # (a via each; a plane's ball fanned out still goes down to its plane somewhere: one more)
        cost += [1000 * (bm.menu[k][i].vias + (1 if k in A.drops else 0)) * v for i, v in xs[key].items()]
        nd_ = len(A.drops.get(k, ()))
        cost += [(1000 if j < nd_ else int(round(1000 * STRAP_VIAS))) * v for j, v in enumerate(dv)]
        plane += [(pk[q], ys[key][q]) for q in range(NL) if pk[q]]
        if k in A.drops:        # (a plane ball fanned out: the fallback, with the changes of layer)
            plane += [(int(round(1000 * PLANE_FAN)), v) for v in xs[key].values()]
            fans += list(xs[key].values())
            # ...and NO FAN-OUT for a plane ball still on the pattern, a rule behind its own assumption (`wide`: the
            # balls on their whole menus): where no state holds it, it is named with what stands in its drops' way,
            # those lanes' balls are widened first, and only a plane ball on its whole menu may be fanned out, at
            # its price. Priced alone, a search that did not converge left three plane balls of a four-bus board
            # fanned out, joined to no pour, where a state with every one dropped stood
            # (asked for with --dropped: on a three-bus array the state solve neither found a state holding it nor ruled
            # one out in its time, where without it a state stood in seconds)
            if (A.ref, k) not in wide and dropped:
                m.Add(sum(xs[key].values()) == 0).OnlyEnforceIf(gate('dropped', A.ref, k, lanes={key}))
        lits += [list(xs[key].values()) + dv, list(ys[key])]
        others[k] = (xs[key], ys[key], dv)
        # (...its choice whose copper, laid, broke the board's rules -- main's teach_laid: not again)
        for r_, k2_, how_, what_ in oth_ban:
            if r_ == A.ref and k2_ == k:
                import source_realize as sr_
                for v in ([v_ for i, v_ in xs[key].items() if sr_.move_sig(bm.menu[k][i]) == what_] if how_ == 'x'
                          else dv[what_:what_ + 1]):
                    m.Add(v == 0).OnlyEnforceIf(gate('laid', A.ref, k, lanes={key}))

    # ---- A PART HELD WHERE IT CAME (`held` [(a pad's centre x, y, its half sizes, its net, its layers)]: main's
    # teach_laid -- a passive beneath the array the nudge could not clear of the fanout): no copper within the
    # clearance of a pad of its of another net -- a move's via, every lane's alike, or a drop's; a leg on the pad's
    # layer; and a bus lane's WAY OUT of its exit there, which the nudge keeps the passives off as it does the copper
    # (held_hits). The menus measure their copper as if the movable passives were not there; one the nudge cannot
    # move clear is copper, as any fixed pad is. Held against the vias alone, a capacitor in two lanes' ways out was
    # moved off them at every laying, into a nook where it grazed a third lane's run: that lane's end ruled out,
    # another took its place, and the model stopped on a field that broke the board's rules
    if held:
        import rules as _rules
        cl_, tw_ = _rules.active().clearance, bm.sz['tw']
        for (k, i), us in where.items():
            hits = {}
            for key, i_ in us:
                bus_ = key[0] != oname          # (an other ball's escape ends at its exit: no way out is laid for it)
                if bus_ not in hits:
                    hits[bus_] = held_hits(held, bm.opts[k][i][2], bm.items[k][1].net_name, tw_, cl_, je.RUN,
                                           way_of(bm.menu[k][i], RL[0]) if bus_ else None)
                dead_, on_ = hits[bus_]
                qs = [] if dead_ else [q for q in range(NL) if any(pad_on(ly, RL[q]) for ly in on_)]
                if dead_ or qs:
                    g_ = gate('held', A.ref, k, lanes={key})
                    if dead_:
                        m.Add(xs[key][i_] == 0).OnlyEnforceIf(g_)
                    for q in qs:                # (the option, but not with its lane's end on the pad's layer)
                        m.Add(xs[key][i_] + ys[key][q] <= 1).OnlyEnforceIf(g_)
        for k in A.okeys:
            for j, (_s, pc_) in enumerate(list(A.drops.get(k, ())) + list(A.straps.get(k, ()))):
                dead_, on_ = held_hits(held, pc_, bm.items[k][1].net_name, tw_, cl_, je.RUN)
                if dead_ or on_:
                    m.Add(others[k][2][j] == 0).OnlyEnforceIf(gate('held', A.ref, k, lanes={(oname, k, 0)}))

    # ---- a STRAP joins a ball that is itself joined to the plane -- dropped, or strapped on -- and a chain of straps
    # climbs to a dropped ball, never round in a ring (joint_escape.plan_array's own rule)
    lvl = {k: m.NewIntVar(0, len(A.okeys), '') for k in A.okeys if A.straps.get(k)}
    for k in sorted(lvl):
        nd_ = len(A.drops.get(k, ()))
        for j, (s_, _pc) in enumerate(A.straps[k]):
            v, to = others[k][2][nd_ + j], others.get(s_.to)
            if to is None:
                m.Add(v == 0)
                continue
            m.Add(v <= sum(to[2]))
            m.Add(lvl[k] >= (lvl[s_.to] if s_.to in lvl else 0) + 1).OnlyEnforceIf(v)

    def by_lane(members):
        """{lane: {option: the members it lays}} of a set of menu moves"""
        d = collections.defaultdict(lambda: collections.defaultdict(set))
        for g in members:
            for key, i in where.get(tuple(g), ()):
                d[key][i].add(tuple(g))
        return d
    lane_sum = lambda key, o: sum(xs[key][i] for i in sorted(o) if (key, i) not in dead)   # noqa: E731
    tag = lambda key: f'{key[0]}:{key[1]}'                   # noqa: E731
    # ---- VIAS: two moves that cannot both stand, whatever layers their runs take
    via_rows = []
    for g in ([] if 'vias' in off else A.cliques):
        d = by_lane(g)
        dead.update((key, i) for key, o in d.items() for i, ms in o.items() if len(ms) > 1)
        if len(d) > 1:
            via_rows.append(('clique', d, None))
    for a_, b_ in ([] if 'vias' in off else A.bicliques):
        da, db = by_lane(a_), by_lane(b_)
        dead.update((key, i) for key in set(da) & set(db) for i in set(da[key]) & set(db[key])
                    if len(da[key][i] | db[key][i]) > 1)
        if len(set(da) | set(db)) > 1:
            via_rows.append(('biclique', da, db))
    # ---- RUNS: a point of the array's lines carries one lane on one layer
    run_rows = []
    for p, by_l in ([] if 'runs' in off else sorted(A.pts.items())):
        d = {L: by_lane(ms) for L, ms in by_l.items()}
        for L, dl in d.items():
            dead.update((key, i) for key, o in dl.items() for i, ms in o.items() if len(ms) > 1)
        if len({key for dl in d.values() for key in dl}) > 1:
            run_rows.append((p, d))
    for key, i in sorted(dead):
        m.Add(xs[key][i] == 0)
    # (a trace's: each lane's options left alive here, and each via group's lanes with their options in it -- read
    # back when a core names a group: which lanes it holds with EVERY option they have left)
    A.trace_live = {key: sorted(i for i in xs[key] if (key, i) not in dead) for key in xs}
    A.trace_vias = {}
    stuck = sorted(key for key in xs if key[0] != oname and all((key, i) in dead for i in xs[key]))
    n_via = 0
    for kind, da, db in via_rows:
        A.trace_vias.setdefault(' / '.join(sorted(tag(k) for k in (set(da) | set(db or {})))), []).append((
            kind, {key: sorted(o) for key, o in da.items()}, {key: sorted(o) for key, o in (db or {}).items()}))
        if kind == 'clique':
            m.Add(sum(lane_sum(key, o) for key, o in sorted(da.items())) <= 1).OnlyEnforceIf(
                gate('vias', A.ref, ' / '.join(sorted(tag(k) for k in da)), lanes=da))
            n_via += 1
            continue
        # (every move of one side against every move of the other: a side's literal, raised by any lane on it; a move
        # on both sides stands against both)
        both = {(key, i) for key in set(da) & set(db) for i in set(da[key]) & set(db[key])}
        sides = []
        for d in (da, db):
            ls = [(key, [i for i in o if (key, i) not in both and (key, i) not in dead]) for key, o in sorted(d.items())]
            ls = [(key, o) for key, o in ls if o]
            if len(ls) == 1:
                sides.append(lane_sum(*ls[0]))
            elif ls:
                t = m.NewBoolVar('')
                for key, o in ls:
                    m.Add(lane_sum(key, o) <= t)
                sides.append(t)
        m.Add(sum(sides) + sum(xs[key][i] for key, i in sorted(both) if (key, i) not in dead) <= 1).OnlyEnforceIf(
            gate('vias', A.ref, ' / '.join(sorted(tag(k) for k in set(da) | set(db))), lanes=set(da) | set(db)))
        n_via += 1
    used = {}                   # (lane, its options through a point) -> [the lane takes one of them]
    occ = {}                    # (that, layer) -> [the lane's run stands at the point on the layer]

    def at_point(key, on):
        if (key, on) not in used:
            if len(on) == 1:
                used[(key, on)] = xs[key][next(iter(on))]
            else:
                u = used[(key, on)] = m.NewBoolVar('')
                m.Add(lane_sum(key, on) <= u)
        return used[(key, on)]
    n_pt = 0
    for p, d in run_rows:
        for q in range(NL):
            # (a run on its own layer: a surface escape's)
            terms = [lane_sum(key, o) for key, o in sorted(d.get(RL[q], {}).items())]
            # (a run on no one layer yet: there once its lane's end takes this layer)
            for key, o in sorted(d.get(je.RUN, {}).items()):
                on = frozenset(i for i in o if (key, i) not in dead and dom[(key, i)][0] is None and q in dom[(key, i)][2])
                if not on:
                    continue
                if (key, on, q) not in occ:
                    w = occ[(key, on, q)] = m.NewBoolVar('')
                    m.AddBoolOr([at_point(key, on).Not(), ys[key][q].Not(), w])
                terms.append(occ[(key, on, q)])
            if len(terms) > 1:
                m.Add(sum(terms) <= 1).OnlyEnforceIf(gate('runs', A.ref, RL[q], f'({p[0]:.2f}, {p[1]:.2f})',
                                                         lanes={key for dl in d.values() for key in dl}))
                n_pt += 1
    # ---- a plane ball's DROP: never with an option its copper cannot be laid beside
    offered_all = collections.defaultdict(set)
    for k2, i in where:
        offered_all[k2].add(i)
    n_dh = 0
    for (k, j), (esc_, drs_) in sorted(drop_hits(A, {k2: sorted(v) for k2, v in offered_all.items()}).items()):
        for key, o in sorted(by_lane(esc_).items()):
            m.Add(others[k][2][j] + lane_sum(key, o) <= 1).OnlyEnforceIf(
                gate('drop', A.ref, k, tag(key), lanes={key, (oname, k, 0)}))
            n_dh += 1
        for k2, j2 in sorted(drs_):
            m.AddBoolOr([others[k][2][j].Not(), others[k2][2][j2].Not()])
            n_dh += 1
    # ---- WAYS OUT: no via in a bus lane's way out of its tooth -- the lane goes on from its exit, straight out past
    # the face, at the lane's own track and the router's clearance (joint_escape.exit_ray_conflicts, the whole-array
    # fanout's own measure): a plane ball's drop, or the via of another lane's escape, a barrel through every layer
    n_way = 0
    if 'ways' not in off:
        esc = {k2: sorted(v) for k2, v in offered_all.items()}
        opts_ = {k2: [('escape', bm.menu[k2][i], None) for i in esc.get(k2, ())]
                 + [('drop', d, None) for d, _pc in A.drops.get(k2, ())] for k2 in bm.items}
        blocked = collections.defaultdict(set)          # a via's option -> the bus escapes whose way out it stands in
        for k, i, k2, j in je.exit_ray_conflicts(opts_, {k2: bm.items[k2][0] for k2 in bm.items}, set(A.bus_of),
                                                 bm.sz['vr'], bm.grid, {n: n for n in A.bus_of}):
            blocked[(k, i)].add((k2, esc[k2][j]))
        for (k, i), ms in sorted(blocked.items()):
            ne = len(esc.get(k, ()))
            # (the via's own literal: a drop's, or its lanes' options that lay the escape)
            left = [((oname, k, 0), others[k][2][i - ne])] if i >= ne else \
                [(key, lane_sum(key, o)) for key, o in sorted(by_lane({(k, esc[k][i])}).items())]
            for key1, lit in left:
                for key2, o in sorted(by_lane(ms).items()):
                    if key2 == key1:
                        continue            # (a pair's two legs: its own options', laid together)
                    m.Add(lit + lane_sum(key2, o) <= 1).OnlyEnforceIf(
                        gate('ways', A.ref, tag(key1), tag(key2), lanes={key1, key2}))
                    n_way += 1
        # ...and a PAIR's way out: one lane to the route, from the middle of its two exits on, its two tracks and their
        # gap wide -- the legs' own rays pass either side of a via standing between them. A GND drop 0.135 mm inside
        # the berths' line, midway between a strobe pair's two berths 0.8 mm apart, was 0.4 mm from each leg's ray and
        # 0.006 mm short of the pair's own bar, the bus's loop stopped on it two rounds running
        import braid as te
        import escape_plan as ep
        import rules as _rules
        from escape_moves import DIRS
        bar_p = _rules.active().pair_gap / 2 + te.TRACK + te.CLEAR + te.GRID / 2
        vias_ = [(k2, i2, o_.site, r_) for k2 in sorted(opts_) for i2, (kind_, o_, _pc) in enumerate(opts_[k2])
                 for r_ in ([o_.r] if kind_ == 'drop' else [bm.sz['vr']] if getattr(o_, 'site', None) is not None
                            and getattr(o_, 'vias', 0) else [])]
        # (one row per pair's lane, via and the lane it is another's of: the pair's options it stands in the way of
        # summed -- a lane takes one option, so that is each of them alone. One row per option and via, four buses'
        # pairs at one array had 361 thousand)
        # (each via's reach for a lane worked out ONCE, and a via whose reach ends short of the way's own line passed
        # over before the distance to the way is measured: the way runs straight out along one axis, so a via is no
        # nearer it than it stands off that line. Asked of every via for every option of every pair, the reach and
        # the distance were eleven million questions a search, most of an array's rules' time)
        ring_ = {}
        for _k2, _i2, site_, r_ in vias_:
            if (site_[0], site_[1], r_) not in ring_:
                ring_[(site_[0], site_[1], r_)] = je._ring(site_, r_)
        agg_ = {}
        for b, k_ in users:
            opts = b.E.T if k_ == 0 else b.E.B
            for lane, lg in b.E.lanes:
                key = (b.name, lane, k_)
                if len(lg) != 2:
                    continue
                for i in sorted(xs[key]):
                    if (key, i) in dead:
                        continue
                    g_ = [bm.menu[mv.gid[1]][mv.gid[2]] for mv in opts[lane][i][0]]
                    if len(g_) != 2 or g_[0].direction != g_[1].direction:
                        continue
                    u = DIRS[g_[0].direction]
                    mid = ((g_[0].exit_pt[0] + g_[1].exit_pt[0]) / 2, (g_[0].exit_pt[1] + g_[1].exit_pt[1]) / 2)
                    back = (ep.PAST - 0.5) * (bm.grid.pitch_x if u[0] else bm.grid.pitch_y)
                    a = (mid[0] - u[0] * back, mid[1] - u[1] * back)
                    e = (mid[0] + u[0] * je.LANE_REACH, mid[1] + u[1] * je.LANE_REACH)
                    reach_ = max(back, je.LANE_REACH) + bar_p + 1.0
                    own_ = {mv.gid[1] for mv in opts[lane][i][0]}
                    ax_ = 1 if u[0] else 0          # (the axis ACROSS the way)
                    for k2, i2, site, r_ in vias_:
                        if k2 in own_ or abs(site[0] - mid[0]) > reach_ or abs(site[1] - mid[1]) > reach_:
                            continue
                        rg_ = ring_[(site[0], site[1], r_)]
                        if abs(site[ax_] - mid[ax_]) - rg_ >= bar_p + 1e-6 or \
                                je._pt_seg(site, a, e) - rg_ >= bar_p - 1e-9:
                            continue
                        ne = len(esc.get(k2, ()))
                        left = [((oname, k2, 0), ('drop', k2, i2 - ne))] if i2 >= ne else \
                            [(key2, ('lane', tuple(sorted(o)))) for key2, o in
                             sorted(by_lane({(k2, esc[k2][i2])}).items())]
                        for key2, lid in left:
                            if key2 != key:
                                agg_.setdefault((key, key2, lid), set()).add(i)
        for (key, key2, lid), is_ in sorted(agg_.items(), key=repr):
            lit = others[lid[1]][2][lid[2]] if lid[0] == 'drop' else lane_sum(key2, set(lid[1]))
            m.Add(sum(xs[key][i] for i in sorted(is_)) + lit <= 1).OnlyEnforceIf(
                gate('ways', A.ref, tag(key), tag(key2), lanes={key, key2}))
            n_way += 1
    # ---- PAIRS WHOLE: no other lane's exit between a pair's two exits on the pair's layer there -- a pair is one lane
    # to the route (whole_frame's PairSplit: a memory bus's data net left on the surface between a strobe pair's two
    # surface teeth, and the bus's step refused the board)
    n_pw = 0
    if 'pairs' not in off:
        lay_of = lambda key, i: {dom[(key, i)][0]} if dom[(key, i)][0] is not None else set(dom[(key, i)][2])   # noqa: E731
        every_ = sorted((p_, key, i) for (key, i), es in exits.items() if (key, i) not in dead for p_, _d in es)
        pos_ = [e_[0] for e_ in every_]
        # (a pair's options by the two exits they leave by: many share them -- other sites, other side-steps -- and
        # the rule is the exits')
        spans = collections.defaultdict(list)               # (the pair's lane, its two exits) -> its options there
        for (key, i), es in sorted(exits.items()):
            if len(es) == 2 and (key, i) not in dead and es[0][1] == es[1][1]:
                spans[(key,) + tuple(sorted(e_[0] for e_ in es))
                      + tuple(sorted(a_ for _d, a_ in along[(key, i)]))].append(i)
        # (...and between them ALONG THEIR FACE, where one of the two lies past the face's end: round the box both
        # that exit and any beside it stand at the corner, one place, and nothing was read as between them -- a
        # pair's legs a ball pitch apart at an array's corner with another net's surface tooth between, and the bus's
        # step refused the board)
        by_face = collections.defaultdict(list)             # a face -> [(along it, lane, option)] of every exit on it
        for (key, i), es in along.items():
            if (key, i) not in dead:
                for d_, a_ in es:
                    by_face[d_].append((a_, key, i))
        for d_ in by_face:
            by_face[d_].sort()
        at_face = {d_: [e_[0] for e_ in es_] for d_, es_ in by_face.items()}
        for (key, lo, hi, a0_, a1_), mine in sorted(spans.items()):
            between = collections.defaultdict(set)          # another lane -> its options with an exit between the two
            for _p, k2, j in every_[bisect.bisect_right(pos_, lo):bisect.bisect_left(pos_, hi)]:
                if k2 != key:
                    between[k2].add(j)
            d_ = along[(key, mine[0])][0][0]
            for _a, k2, j in by_face[d_][bisect.bisect_right(at_face[d_], a0_):bisect.bisect_left(at_face[d_], a1_)]:
                if k2 != key:
                    between[k2].add(j)
            for k2, js in sorted(between.items()):
                for q in range(NL):
                    on = [j for j in sorted(js) if q in lay_of(k2, j)]
                    here = [i for i in mine if q in lay_of(key, i)]
                    if on and here:
                        m.Add(sum(xs[key][i] for i in here) + sum(xs[k2][j] for j in on) + ys[key][q] + ys[k2][q]
                              <= 3).OnlyEnforceIf(gate('pairs', A.ref, tag(key), tag(k2), lanes={key, k2}))
                        n_pw += 1
    # ---- A BLOCKED FRONT'S VIA: a single lane's exit that meets copper straight ahead on its layer changes layer
    # BEFORE it (whole_ends FRONT_VIA; bus_plan tells the solve where) -- a via in its own way out, a barrel through
    # every layer, right outside the array. No other lane leaves by an exit within a via's room of that way out, on
    # any layer: its straight run past the face is where the barrel stands. A surface tooth told to dive 0.6 mm out
    # of an exit it shared with another lane's inner-layer run (two lanes leaving one point on two layers) was
    # planned through that run, and again beside the exits a ball pitch either side
    n_fv = 0
    if 'fronts' not in off:
        # (a via's radius, the clearance and half the lane's track: the array's own sizes -- bm.sz, the menus')
        R_ = int(round(1000 * (bm.sz['vr'] + bm.sz['cl'] + bm.sz['tw'] / 2)))
        every_ = sorted((d_, p_, key, i) for (key, i), es in exits.items() if (key, i) not in dead for p_, d_ in es)
        for b, k_ in users:
            opts = b.E.T if k_ == 0 else b.E.B
            for lane, lg in b.E.lanes:
                key = (b.name, lane, k_)
                if len(lg) != 1:
                    continue
                for i, c_ in sorted((b.E.front.get((lane, k_)) or {}).items()):
                    if c_ != we.FRONT_VIA or i not in xs[key] or (key, i) in dead or len(exits[(key, i)]) != 1:
                        continue
                    if we.front_span(b.E.st['pcb'], b.E.st['byname'][lg[0]][0], opts[lane][i][0][0],
                                     b.E.run_ids) is None:
                        continue            # (no via there: the lane goes round the block on its own layer)
                    p0, d0 = exits[(key, i)][0]
                    beside = collections.defaultdict(set)
                    for d_, p_, key2, j in every_:
                        if d_ == d0 and key2 != key and abs(p_ - p0) < R_:
                            beside[key2].add(j)
                    for key2, js in sorted(beside.items()):
                        m.Add(xs[key][i] + lane_sum(key2, js) <= 1).OnlyEnforceIf(
                            gate('fronts', A.ref, tag(key), tag(key2), lanes={key, key2}))
                        n_fv += 1
    log(f'  {A.ref}: {sum(len(b.E.lanes) for b, _k in users)} bus lanes and {len(A.okeys)} other balls, each served -- '
        f'{n_via} groups of vias, {n_pt} points on a layer one lane each ({len(used)} lane-at-point literals), '
        f'{sum(len(o[2]) for o in others.values())} drops in {n_dh} rows, {n_way} ways out kept clear of a via, '
        f'{n_fv} rows keeping a blocked front\'s via clear of the exits beside it, '
        f'{n_pw} rows keeping a pair\'s exits together, {len(dead)} options not offered')
    return types.SimpleNamespace(others=others, cost=cost, plane=plane, lits=lits, stuck=stuck, fans=fans)


def region_rows(m, A, users, gate, RL, P):
    """THE REGIONS at array `A`, added to `m`: on each layer, each bus's exits ONE stretch round the array with no
    other bus's among them -- every other bus's exits there outside the stretch its own span, or its own round theirs,
    outside the stretch they span. Returns the number of rules"""
    NL = len(RL)
    place = {}
    for b, k_ in users:
        opts = b.E.T if k_ == 0 else b.E.B
        for l_ in b.J.lanes:
            pv = {i: place_round(A.box, b.F.dirn(opts[l_][i][0][0].direction), b.F.pt(opts[l_][i][1]))
                  for i in b.J.xs[k_][l_]}
            v = m.NewIntVar(min(pv.values()), max(pv.values()), '')
            m.Add(v == sum(pv[i] * x for i, x in b.J.xs[k_][l_].items()))
            place[(b.name, l_)] = v
    span = {(b.name, q): (m.NewIntVar(-1, P, ''), m.NewIntVar(-1, P, '')) for b, _k in users for q in range(NL)}
    rest = {(b.name, q): (m.NewIntVar(-1, P, ''), m.NewIntVar(-1, P, '')) for b, _k in users for q in range(NL)}
    for b, k_ in users:
        for l_ in b.J.lanes:
            for q in range(NL):
                for o_, _ko in users:
                    lo, hi = (span if o_ is b else rest)[(o_.name, q)]
                    m.Add(place[(b.name, l_)] >= lo).OnlyEnforceIf(b.J.y[k_][l_][q])
                    m.Add(place[(b.name, l_)] <= hi).OnlyEnforceIf(b.J.y[k_][l_][q])
    n = 0
    for B_, _kB in users:
        for q in range(NL):
            g = m.NewBoolVar('')
            n += 1
            for b, k_ in users:
                lo, hi = (rest if b is B_ else span)[(B_.name, q)]
                on = g.Not() if b is B_ else g
                for l_ in b.J.lanes:
                    sd = m.NewBoolVar('')
                    gl = gate('region', A.ref, B_.name, RL[q], f'{b.name}:{l_}', lanes={(b.name, l_, k_)})
                    m.Add(place[(b.name, l_)] < lo).OnlyEnforceIf([b.J.y[k_][l_][q], on, gl, sd])
                    m.Add(place[(b.name, l_)] > hi).OnlyEnforceIf([b.J.y[k_][l_][q], on, gl, sd.Not()])
    return n


def lane_heads(m, A, b, k_, way_b, P):
    """{lane: [(the place round array `A` it is HEADED for, from its bus's face -- thousandths of a mm, the nearer way
    -- and a literal or 1: [it is headed there])]} of bus `b`'s lanes at `A`, its end `k_`. Where a lane is headed is
    the place where the ray from A's centre toward the point it makes for leaves A's box. At its SOURCE that point is
    on its destination's box (the bus's frame, turned to the board's): the middle of the face toward the source for a
    berth on that face, else the corner of the side of the destination the lane goes round -- a berth on the north
    face round the north, one on the south face round the south, one on the far face round the side the cut sends
    it. A lane at its destination, and one headed for a GROUP of parts, is headed for its bus's face (0).
    A part beside the array is seen from it at an angle: four buses off one array, the face toward one bus's part
    was the middle of the array's west face, the ray to the part left it 2.8 mm north of that, and the side the
    bus's cut sent its far berths round 3.9 mm north -- along the face past another bus's exits, which every lane of
    the bus headed for the face's middle never reached"""
    import whole_frame as wf
    Q = lambda v_: int(round(v_ * 1000))        # noqa: E731
    lanes = b.J.lanes
    DB = getattr(b.J, 'DB', None)
    if k_ != 0 or b.E.group is not None or DB is None:
        return {l_: [(0, 1)] for l_ in lanes}
    x0, y0, x1, y1 = A.box
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

    def toward(p):
        q = b.F.pt(p)
        dx, dy = q[0] - cx, q[1] - cy
        hits = [(t_, d_) for t_, d_ in (((x1 - cx) / dx if dx > 0 else None, 'right'),
                                        ((x0 - cx) / dx if dx < 0 else None, 'left'),
                                        ((y1 - cy) / dy if dy > 0 else None, 'down'),
                                        ((y0 - cy) / dy if dy < 0 else None, 'up')) if t_ is not None]
        if not hits:
            return 0
        t_, d_ = min(hits)
        return (place_round(A.box, d_, (cx + dx * t_, cy + dy * t_)) - way_b + P // 2) % P - P // 2
    head = {'F': toward((DB[0], (DB[1] + DB[3]) / 2)), 'N': toward((DB[0], DB[1])), 'S': toward((DB[0], DB[3]))}
    tNW, tSW = Q(wf.perim_c((DB[0], DB[1]), DB, b.J.cut)), Q(wf.perim_c((DB[0], DB[3]), DB, b.J.cut))
    out = {}
    for l_ in lanes:
        conds = collections.defaultdict(list)       # heading -> [the literals that, all held, send the lane there]
        for j, x_ in b.J.xs[1][l_].items():
            t = b.J.pos[(1, l_, j)]
            if tNW <= t <= tSW:
                conds['F'].append([x_])
            else:
                conds['N' if t < tNW else 'S'].append([x_])
        by = collections.defaultdict(list)
        for c_, cs in conds.items():
            for cl_ in cs:
                if not any(isinstance(e_, int) and e_ == 0 for e_ in cl_):
                    by[head[c_]].append([e_ for e_ in cl_ if not isinstance(e_, int)])
        if len(by) == 1:
            out[l_] = [(next(iter(by)), 1)]
            continue
        out[l_] = []
        for h_, cs in sorted(by.items()):
            g_ = m.NewBoolVar('')
            for cl_ in cs:
                m.AddBoolOr([g_] + [e_.Not() for e_ in cl_])
            out[l_].append((h_, g_))
    return out


def between_rows(m, A, users, gate, RL):
    """THE MEETINGS BETWEEN BUSES at array `A`, added to `m`. Each lane runs round `A` from its exit to where it is
    headed (lane_heads), the nearer way from its own bus's face (as the bus's own model unrolls its exits from the
    middle of the face away from its part); two lanes of two buses MEET where those two stretches overlap -- they
    cross (their order leaving opposite to their order where they are headed, as a bus counts its own crossings), or
    one runs past the other's exit beside it. A bus's own lanes side by side are nested by its own solve; two buses'
    are planned by two solves, each knowing nothing of the other's, and laid out they lie on one another's -- four
    buses off one array, one bus's cut sent its far berths round the side of its part toward another bus's, along the
    face past that bus's exits on their layers: 436 places the two buses' own plans ran together, 2573 couplings to
    settle. So a meeting is counted as a bus counts a crossing of its own: two that meet on one layer there change
    layer, one of them at least, and each is priced by whether it is clearly free of a via -- X_TRUNK where the two
    lanes share no layer end to end, JOINT_X where they stand on one layer somewhere.
    With `D` the way round from one bus's face to the other's, `s` a lane's exit's place from its own bus's and `t`
    its heading's: lane b's stretch, seen from a's bus's face, is the one between D + s_b and D + t_b (or a whole way
    round less), and the two meet where the stretches overlap by more than a tie.
    `gate(*name, lanes=)`: the assumption two meeting lanes' layers stand behind, so that where no state holds them
    the two lanes are named. Returns (the meetings {(bus, lane, bus, lane): a literal or 1}, their terms of the
    objective, its constant)"""
    import whole_ends as we
    Q = lambda v_: int(round(v_ * 1000))        # noqa: E731
    TQ, W_X, NL = Q(we.TIE_MM), Q(we.JOINT_X), len(RL)
    x0, y0, x1, y1 = A.box
    P = Q(2 * ((x1 - x0) + (y1 - y0)))
    mid = {'up': ((x0 + x1) / 2, y0), 'right': (x1, (y0 + y1) / 2), 'down': ((x0 + x1) / 2, y1),
           'left': (x0, (y0 + y1) / 2)}
    from ortools.sat.python import cp_model
    way, vals, H, PV = {}, {}, {}, {}
    for b, k_ in users:
        # (the face toward its other end, in the bus's own frame the east at its source and the west at its end)
        face = b.F.dirn('right' if k_ == 0 else 'left')
        way[b.name] = place_round(A.box, face, mid[face])
        opts = b.E.T if k_ == 0 else b.E.B
        for l_ in b.J.lanes:
            sv = {i: (place_round(A.box, b.F.dirn(opts[l_][i][0][0].direction), b.F.pt(opts[l_][i][1]))
                      - way[b.name] + P // 2) % P - P // 2 for i in b.J.xs[k_][l_]}
            vs = sorted(set(sv.values()))
            h_ = [1] + [m.NewBoolVar('') for _ in vs[1:]] + [0]         # (order-encoded: [its place >= vs[t]])
            for t_, v_ in enumerate(vs):
                m.Add(h_[t_] - h_[t_ + 1] == sum(x for i, x in b.J.xs[k_][l_].items() if sv[i] == v_))
            vals[(b.name, l_)], H[(b.name, l_)] = vs, h_
            # (...and the place itself, an integer on exactly those values: two lanes' places compared by one row
            # each way -- gt. Compared clause by clause over every place of the other, four buses at one array took
            # 1.1 million rows, two of every five of their model's)
            pv_ = m.NewIntVarFromDomain(cp_model.Domain.FromValues(vs), '')
            m.Add(pv_ == vs[0] + sum((vs[t_] - vs[t_ - 1]) * h_[t_] for t_ in range(1, len(vs))))
            PV[(b.name, l_)] = pv_
    heads = {(b.name, l_): hs for b, k_ in users for l_, hs in lane_heads(m, A, b, k_, way[b.name], P).items()}

    def ge(key, v_):
        t_ = bisect.bisect_left(vals[key], v_)
        return H[key][t_] if t_ < len(vals[key]) else 0
    neg = lambda e_: 1 - e_ if isinstance(e_, int) else e_.Not()     # noqa: E731

    X, cost, const = {}, [], 0
    GT = {}

    def gt(ka, kb, d):
        """a literal, or 0/1: [a's place - b's > d] -- both ways, on the two places' integers"""
        if d >= P:
            return 0
        if d < -P:
            return 1
        if (ka, kb, d) not in GT:
            if vals[ka][0] - vals[kb][-1] > d:
                g = 1
            elif vals[ka][-1] - vals[kb][0] <= d:
                g = 0
            else:
                g = m.NewBoolVar('')
                m.Add(PV[ka] - PV[kb] >= d + 1).OnlyEnforceIf(g)
                m.Add(PV[ka] - PV[kb] <= d).OnlyEnforceIf(g.Not())
            GT[(ka, kb, d)] = g
        return GT[(ka, kb, d)]
    ANY = {}

    def any_of(ts):
        """a literal, or 0/1: [one of `ts` holds] -- each a thunk giving a literal or 0/1, asked only until one is 1"""
        ls = []
        for f_ in ts:
            e_ = f_()
            if isinstance(e_, int):
                if e_:
                    return 1
                continue
            ls.append(e_)
        if len(ls) <= 1:
            return ls[0] if ls else 0
        key_ = tuple(sorted(e_.Index() for e_ in ls))
        if key_ not in ANY:
            g_ = m.NewBoolVar('')
            for e_ in ls:
                m.AddImplication(e_, g_)
            ANY[key_] = g_
        return ANY[key_]

    def meet(ka, kb, D):
        """a literal, or 0/1: [lanes `ka` and `kb` meet round A] -- for each heading of each and b's stretch a whole
        way round less or not, the two stretches [lo, hi] overlap: lo_a < hi_b and lo_b < hi_a, by more than a tie"""
        amin, amax, bmin, bmax = vals[ka][0], vals[ka][-1], vals[kb][0], vals[kb][-1]
        rows = []
        for ta, ha in heads[ka]:
            for tb0, hb in heads[kb]:
                for kk in (0, -1):
                    off = D + kk * P
                    tb = tb0 + off
                    if max(min(amin, ta), min(bmin + off, tb)) + TQ >= min(max(amax, ta), max(bmax + off, tb)):
                        continue                # (the two can overlap at no place either may leave by)
                    # (a's stretch starts before b's ends: a's exit or heading before b's exit or heading)
                    lo_a = any_of([lambda: int(ta + TQ < tb), lambda: neg(ge(ka, tb - TQ)),
                                   lambda: ge(kb, ta + TQ - off + 1), lambda: neg(gt(ka, kb, off - TQ - 1))])
                    # (b's starts before a's ends)
                    lo_b = any_of([lambda: int(tb + TQ < ta), lambda: ge(ka, tb + TQ + 1),
                                   lambda: neg(ge(kb, ta - TQ - off)), lambda: gt(ka, kb, off + TQ)])
                    row = (lo_a, lo_b, ha, hb)
                    if any(isinstance(e_, int) and e_ == 0 for e_ in row):
                        continue
                    row = [e_ for e_ in row if not isinstance(e_, int)]
                    if not row:
                        return 1
                    rows.append(row)
        if not rows:
            return 0
        xv = m.NewBoolVar('')
        for row in rows:
            m.AddBoolOr([xv] + [e_.Not() for e_ in row])
        return xv
    R_ = Q(NEAR_EXITS)
    W_E = Q(we.X_TRUNK)             # (a crossing clear of any via: the ends model's price of one on the trunk)
    W_X = max(W_X, W_E)
    uses = {}                       # a lane -> [it stands on the layer at one end or the other]

    def on(b, l_):
        if (b.name, l_) not in uses:
            us = [m.NewBoolVar('') for _ in range(NL)]
            for q in range(NL):
                m.AddMaxEquality(us[q], [b.J.y[0][l_][q], b.J.y[1][l_][q]])
            uses[(b.name, l_)] = us
        return uses[(b.name, l_)]
    for n_, (ba, ka_) in enumerate(users):
        for bb, kb_ in users[n_ + 1:]:
            D = (way[bb.name] - way[ba.name]) % P
            for la in ba.J.lanes:
                for lb in bb.J.lanes:
                    ka, kb = (ba.name, la), (bb.name, lb)
                    xv = meet(ka, kb, D)
                    if isinstance(xv, int) and xv == 0:
                        continue
                    if isinstance(xv, int):
                        const += W_E
                    else:
                        cost.append(W_E * xv)
                    # (CLEARLY free of a via only where the two lanes share no layer, end to end: a meeting on two
                    # layers is no trouble, one between lanes that stand on one layer somewhere is a via to plan --
                    # a crossing's own price, JOINT_X, is that one's)
                    nv = m.NewBoolVar('')
                    ua, ub = on(ba, la), on(bb, lb)
                    for q in range(NL):
                        m.AddBoolOr([nv, ua[q].Not(), ub[q].Not()] + ([] if isinstance(xv, int) else [xv.Not()]))
                    cost.append((W_X - W_E) * nv)
                    X[(ba.name, la, bb.name, lb)] = (xv, nv)
                    gl = gate('between', A.ref, f'{ba.name}:{la}', f'{bb.name}:{lb}',
                              lanes={(ba.name, la, ka_), (bb.name, lb, kb_)})
                    # (NEAR: their exits within NEAR_EXITS round the array, either way -- the two places' difference
                    # within it of the faces' own, a whole turn more or less)
                    xl = [] if isinstance(xv, int) else [xv.Not()]
                    # (ONE literal for [the two meet and leave on one layer here], set by each layer they both take:
                    # the two rules below are then a row each. Said a layer at a time they were some twenty-two rows a
                    # pair of lanes, for every pair of two buses' lanes -- a third of a four-bus group's model)
                    sl = m.NewBoolVar('')
                    for q in range(NL):
                        m.AddBoolOr([sl, ba.J.y[ka_][la][q].Not(), bb.J.y[kb_][lb][q].Not()] + xl)
                    for k__ in (-2, -1, 0, 1, 2):
                        lo_, hi_ = D + k__ * P - R_, D + k__ * P + R_
                        if hi_ <= -P or lo_ >= P:
                            continue
                        g_lo, g_hi = gt(ka, kb, lo_), gt(ka, kb, hi_ - 1)
                        if (isinstance(g_lo, int) and g_lo == 0) or (isinstance(g_hi, int) and g_hi == 1):
                            continue
                        cl_ = ([] if isinstance(g_lo, int) else [g_lo.Not()]) + ([] if isinstance(g_hi, int) else [g_hi])
                        m.AddBoolOr([sl.Not()] + cl_).OnlyEnforceIf(gl)
                    m.AddBoolOr([sl.Not(), ba.J.chg[la], bb.J.chg[lb]]).OnlyEnforceIf(gl)
    return X, cost, const


def bus_dest(st):
    """the destination the whole route's frame reads a bus by: its part, or every part of its GROUP ('A+B':
    whole_frame.Group, the frame's one destination)"""
    return '+'.join(st['dests']) if len(st.get('dests') or ()) > 1 else st['dref']


def bus_plan(b, state, lay, RL):
    """(plan, copper) of bus `b` in the group's state -- `state` {lane: (tooth, berth)}, `lay` {lane: (its tooth's
    layer, its berth's)} -- in the bus's own frame: the plan sidecar's keys as the whole route reads them (braid.setup:
    each leg's ends at its moves' exits, the layers the model gave them, the escape directions, the pair frame, the
    bus's cut, its nets in the run's order), and a row part's berths' own copper [(net id, legs [(a, b, layer)], its
    via's site or None)], each run on the layer the model gave it -- an array's copper is the array's (main's `laid`)"""
    import joint_escape as je
    import whole_ends as we
    st, E = b.E.st, b.E
    plan = {'ends': {}, 'tooth_layer': {}, 'dest_layer': {}, 'tooth_dir': {}, 'stub_dir': {}, 'chi': int(st['chi']),
            'end_layers': {}}
    copper = []
    for lane, lg in E.lanes:
        to_, bo_ = E.T[lane][state[lane][0]], E.B[lane][state[lane][1]]
        for leg, tm, bm_ in zip(lg, to_[0], bo_[0]):
            plan['ends'][leg] = [[float(v) for v in tm.exit_pt], [float(v) for v in bm_.exit_pt]]
            # (...and THE LAYERS EACH END HAS ROOM ON: a move whose run is laid on a layer to choose keeps the layers
            # its exit's front is clear on -- a single's straight run out, a pair leg's with room for the pair
            # (`runs_on`: load_bus, joint_escape) -- and an end whose layer the bus's solve may choose is held to
            # them, whole_solve. Free of them, a solve moved a pair's berths onto the back layer, a capacitor 0.7 mm
            # in front of one leg there, and the pair could not be laid; None: the end's one layer, or no reading)
            plan['end_layers'][leg] = [sorted(m_.runs_on) if m_.layer == je.RUN and getattr(m_, 'runs_on', None)
                                       else None for m_ in (tm, bm_)]
            plan['tooth_layer'][leg], plan['dest_layer'][leg] = RL[lay[lane][0]], RL[lay[lane][1]]
            plan['tooth_dir'][leg], plan['stub_dir'][leg] = list(we.FAR[tm.direction]), list(we.FAR[bm_.direction])
            if b.arr[1] is None:
                # (a berth with a VIA at its stub's end: the stub stays on its pad's own layer and the lane's layer
                # is the via's far side. Laid on the lane's layer, it left its pad joined to nothing: seven nets of
                # a bus open at one row part, each a berth with a via)
                # (...and a berth the QFN fanout laid -- qfn_berths -- has its stub's copper beside it, on its own layer)
                # (...and a berth whose via stands INSIDE its part -- between a two-row connector's rows -- has a run
                # from that via out to its exit: the via's far side too, laid on the lane's layer. Left on the move's
                # own layer, the run ended at the exit on the front layer and the lane arrived there on an inner one,
                # no via between them: a dozen nets of a bus open at one connector, each 0.000 mm from its lane)
                fan_ = getattr(bm_, 'fan_legs', None)
                site_ = tuple(bm_.site) if bm_.vias and bm_.site is not None else None
                at_ = lambda p_: site_ is not None and math.hypot(p_[0] - site_[0], p_[1] - site_[1]) < 1e-6  # noqa: E731
                legs, past_ = [], False
                for p_, q_, Lg in (() if fan_ else bm_.legs):
                    past_ = past_ or at_(p_)
                    if math.hypot(q_[0] - p_[0], q_[1] - p_[1]) > 1e-6:
                        legs.append((tuple(p_), tuple(q_), RL[lay[lane][1]]
                                     if Lg == bm_.layer and (past_ or not bm_.vias) else Lg))
                    past_ = past_ or at_(q_)
                if fan_:
                    legs = [(tuple(p_), tuple(q_), Lg) for p_, q_, Lg in fan_]
                copper.append((st['byname'][leg][0], legs,
                               tuple(bm_.site) if bm_.vias and bm_.site is not None else None))
    c_ = getattr(b, 'cut', None)
    c_ = E.wcut if c_ is None else c_
    if c_ is not None:
        plan['dest_cut'] = [c_[0]] + [float(v) for v in c_[1:]] if isinstance(c_, (list, tuple)) else float(c_)
    if E.group is not None:
        # (a GROUP's ways in -- each lane's place on its walk, a point: the frame takes the way nearest it. Without
        # them the frame chose each lane's way itself, another reading of the berths than the ends model's)
        plan['dest_entry'] = {l_: list(p_) for l_, p_ in E._entry_out(
            {lane: E.B[lane][state[lane][1]] for lane, _lg in E.lanes}).items()}
    if b.arr[0] is not None:
        # (the source's far face's cut as the model read it -- its middle, each lane round the array the nearer way,
        # a tooth at the very middle by the one side every reader takes (whole_ends.SRC_TIE): the bus's own frame
        # takes the cut it is given, and would else choose one for this bus alone)
        plan['src_cut'] = float(E.shared_scut())
    plan['nets'] = [nm for nm in st['launch'] if nm in plan['ends']]
    # THE BLOCKED FRONTS the model chose: a single lane's end whose straight run out meets copper on its layer, with
    # room for a via before it (whole_ends FRONT_VIA) -- the model counts that lane as changing layer there, and the
    # solve is TOLD: the span of the copper and where the via stands before it, {lane: {end: (via_near, via_far, near,
    # far)}}, its LAYER cut (whole_solve FRONT) -- for the model's own check of these ends and for the bus's rounds
    # (the plan sidecar's ends_model.front). Untold, the check passed and the rounds planned eleven surface teeth
    # along one face of an array with a flash part's pads 0.85 mm in front of five of them, each lane on its tooth's
    # layer across the pads, and the audits found them there round after round
    plan['front'] = {}
    for lane, lg in E.lanes:
        if len(lg) != 1:
            continue
        for k_, opts in ((0, E.T[lane]), (1, E.B[lane])):
            if (E.front.get((lane, k_)) or {}).get(state[lane][k_]) == we.FRONT_VIA:
                sp_ = we.front_span(st['pcb'], st['byname'][lg[0]][0], opts[state[lane][k_]][0][0], E.run_ids)
                if sp_ is not None:
                    plan['front'].setdefault(lane, {})[str(k_)] = [float(v) for v in sp_]
    return plan, copper


def bus_bench(b, laid, RL):
    """the bench bus `b`'s ends make as the model chose them (b.plan: bus_plan's), with no bus step between
    (whole_ctx.candidate): the bus's own board with everything the model laid (`laid`: every array's copper and every
    row part's berths', each piece turned into the bus's frame). None for a plan in the turned-over pair frame"""
    import braid as bd
    import joint_escape as je
    import whole_ctx
    from kicad_parser import Segment, Via
    st = b.E.st
    pcb = copy.copy(st['pcb'])
    segs, vias, seen = [], [], []

    def add_via(nid, c, size, drill):
        if not any(n2 == nid and math.hypot(c[0] - c2[0], c[1] - c2[1]) < je.SHARE_TOL for n2, c2 in seen):
            seen.append((nid, c))
            vias.append(Via(x=c[0], y=c[1], size=size, drill=drill, layers=[RL[0], RL[-1]], net_id=nid))
    for nid, pc, L, tw, _ref in laid:
        nid = b.to_cap.get(nid, -nid)       # (laid under the bare board's ids: on this bus's board, its own)
        for a_, b_, Lg in pc.legs:
            (x0, y0), (x1, y1) = b.G.pt(a_), b.G.pt(b_)
            segs.append(Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=tw, layer=L if Lg == je.RUN else Lg,
                                net_id=nid))
        for c, r, dr in pc.vias:
            add_via(nid, b.G.pt(c), 2 * r, 2 * dr)
    pcb.segments = list(pcb.segments) + segs
    pcb.vias = list(pcb.vias) + vias
    if CHECK_DUMP:              # (a bench's: the check's bench as asked, to ask it again alone)
        d_ = CHECK_DUMP
        os.makedirs(d_, exist_ok=True)
        n_ = sum(f.startswith(b.name + '.') for f in os.listdir(d_))
        with open(os.path.join(d_, f'{b.name}.{n_}.pkl'), 'wb') as f:
            pickle.dump({'pcb': pcb, 'nets': b.plan['nets'], 'dest': bus_dest(st), 'plan': b.plan,
                         'base': st.get('base'), 'env': dict(b.env)}, f)
    ctx = whole_ctx.candidate(pcb, b.plan['nets'], bus_dest(st), b.plan, [], cfg=bd.run_config(pcb, st.get('base')),
                              board=st.get('base'))
    if ctx is not None:
        ctx.escape = True       # (ends laid exactly as their moves are, as an escape's)
    return ctx


# the model's exit where a bus has NO ROOM AT THESE SIZES and a rung below has (main: --rungs-below)
NO_ROOM = 7


def room_on_a_rung(room, rungs):
    """whether a lane with no room for a change of layer at these sizes has room for one ON A RUNG BELOW. `room` the
    solve's own measure of the lane (whole_solve's `rooms`: its entry and its end along the frame, the room each of
    its ends keeps, ..., the step its changes stand on); `rungs` [(the lanes' track, the via)] of the rungs below.
    Each end's room is a via's need (rules.via_need: twice it a single's, a pair's dive room read as scaling the
    same way), and a via's need is its sizes'"""
    import dataclasses
    import rules as _rules
    r0 = _rules.active()
    span, need = room[1] - room[0], room[2] + room[3]
    return any(span - need * dataclasses.replace(r0, track=t_, via_size=v_).via_need / r0.via_need >= room[7]
               for t_, v_ in rungs)


def short_named(chk, state, rungs):
    """the lanes a refusal `chk` (bus_check's verdict) NAMES that are too short between their two ends for a change of
    layer at these sizes and have room for one on a rung below (`rungs`, room_on_a_rung) -- of the lanes in `state`.
    A lane short and not named holds one layer, and nothing says the refusal is of that"""
    rooms = chk.get('rooms') or {}
    return {l_ for l_ in chk.get('short') or () if l_ in state and l_ in (chk.get('lanes') or ())
            and rooms.get(l_) and room_on_a_rung(rooms[l_], rungs)}


def bus_check(b, laid, RL, save=None, plans=None):
    """THE WHOLE SOLVE'S VERDICT on bus `b`'s ends as the model chose them (whole_ends.solve_check's, `exact`, on the
    group's own bench -- bus_bench): {'plan', 'status', 'core' (pairs of lanes whose crossing has no window), 'lanes'
    (the lanes its rules cannot hold together), 'rules' ((lane, the rule's family) as its rules core names them),
    'ends_y' (with a plan: {lane: [its tooth's layer, its berth's]} as the solve took them),
    'hands' (the lanes named for their hands: a pair's crossover with no place), 'endlay' ((lane, end): an end whose
    own layer cannot hold), 'nochange' (lanes no change fits in
    on these ends: their window empty, or named for their change's room among their crossings), 'short' (those of
    them whose own window is empty -- too short between their two ends for a change, whatever the rest), 'crossing'
    (the refusal read from its crossings with no window alone: `lanes` theirs, no rules asked), 'at'}; None where it
    cannot be asked. `plans` {bus: the plan its check before found}: asked first (whole_solve.solve's `hint` under
    `check`), and this check's plan kept there for the next"""
    import tempfile
    import whole_frame
    import whole_solve
    ctx = bus_bench(b, laid, RL)
    if ctx is None:
        return None
    dest_ = bus_dest(b.E.st)
    buf_ = io.StringIO()
    hint_ = None
    if plans is not None and plans.get(b.name):
        fd_, hint_ = tempfile.mkstemp(suffix='.json', prefix='check_plan_')
        with os.fdopen(fd_, 'w') as fh_:
            json.dump(plans[b.name], fh_)
    try:
        chk = _bus_check(b, ctx, dest_, RL, buf_, whole_frame, whole_solve, hint_)
        if plans is not None and chk and chk.get('hint'):
            plans[b.name] = chk['hint']
        if chk:
            chk.pop('hint', None)
        return chk
    finally:
        if hint_:
            os.remove(hint_)
        if save:
            # (the check's own solve, as it said it: a check's trace -- group_ends' checks.jsonl names the file)
            os.makedirs(os.path.dirname(save), exist_ok=True)
            open(save, 'w', encoding='utf-8').write(buf_.getvalue())


def _bus_check(b, ctx, dest_, RL, buf_, whole_frame, whole_solve, hint_=None):
    """bus_check's verdict, the solve's console in `buf_`"""
    with contextlib.redirect_stdout(buf_):
        try:
            J = whole_solve.solve(ctx, dest_, front=b.plan.get('front') or {}, check=True, core=True, hint=hint_,
                                  cores=True)
        except whole_frame.PairSplit as e:
            at_ = {}
            for p_, w_, bs_ in e.splits:
                for l_ in [p_] + list(bs_):
                    k_ = 0 if w_ == 'tooth' else 1
                    at_[l_] = k_ if at_.get(l_, k_) == k_ else None
            return {'plan': False, 'status': 'SPLIT', 'core': [], 'lanes': sorted(at_), 'at': at_, 'nochange': []}
        except whole_frame.FarFace as e:
            return {'plan': False, 'status': 'FARFACE', 'core': [], 'lanes': sorted(getattr(e, 'lanes', ()) or ()),
                    'nochange': []}
        out = {'plan': bool(J and J.get('plan')), 'status': (J or {}).get('status'),
               'core': [tuple(k_) for k_ in (J or {}).get('core') or []], 'lanes': [],
               'nochange': list((J or {}).get('nochange') or []), 'ends_y': (J or {}).get('ends_y') or {},
               'short': list((J or {}).get('nochange') or [])}
        if (J or {}).get('hint'):
            out['hint'] = J['hint']         # (the plan itself: bus_check keeps it for the bus's next check)
        out['vban'] = [(n_, int(k_), list(ls_)) for n_, k_, ls_ in (J or {}).get('vban') or []]
        out['vend'] = {(n_, int(k_)) for n_, k_ in (J or {}).get('vend') or []}
        out['vban_why'] = [(n_, int(k_), L_, w_) for n_, k_, L_, w_ in (J or {}).get('vban_why') or []]
        out['vsep'] = [(tuple(a_), tuple(c_)) for a_, c_ in (J or {}).get('vsep') or []]
        out['rooms'] = (J or {}).get('rooms') or {}
        if int(b.plan.get('chi') or 1) < 0:
            # (a plan in the turned-over pair frame is solved on the board turned over: its layers named back --
            # read as they came, every front end of such a bus was "on another layer" than the model's, the back)
            import braid as bd
            out['ends_y'] = {n_: [bd.other_layer(L_) for L_ in ls_] for n_, ls_ in out['ends_y'].items()}
            out['vban'] = [(n_, k_, [bd.other_layer(L_) for L_ in ls_]) for n_, k_, ls_ in out['vban']]
            out['vban_why'] = [(n_, k_, bd.other_layer(L_), w_) for n_, k_, L_, w_ in out['vban_why']]
        if not out['plan'] and out['status'] == 'INFEASIBLE' and out['core']:
            # (THE CROSSINGS WITH NO WINDOW name the lanes at fault, and the refusal is read from them alone: the
            # lanes' core and the rules' below each build the solve's model again and take their core apart a rule at
            # a time -- 109 s and 123 s where this verdict took 9, on a 53-lane bus -- and are asked only where no
            # crossing is named)
            out['lanes'] = sorted({l_ for k_ in out['core'] for l_ in k_})
            out['rules'], out['hands'], out['endlay'], out['crossing'] = [], [], [], True
            # (EVERY core of crossings the state holds, the solve's `cores`: each one's lanes apart -- each is a
            # refused state of its own)
            out['cores'] = [[tuple(k_) for k_ in c_] for c_ in (J or {}).get('cores') or [out['core']]]
            out['groups'] = [sorted({l_ for k_ in c_ for l_ in k_}) for c_ in out['cores']]
        elif not out['plan'] and out['status'] == 'INFEASIBLE':
            # (no crossing named with the windows alone: THE RULES that cannot hold together, each lane's by FAMILY
            # -- which rule of a lane cannot hold, a pair's hands among them -- and the crossings their core names
            # with them. Asked once: the lanes' own core, every rule of a lane as one, was a second solve as long
            # that named lanes this one names by their rules)
            Jr = whole_solve.solve(ctx, dest_, front=b.plan.get('front') or {}, check=True, core=True, rules=True)
            out['core'] = [tuple(k_) for k_ in (Jr or {}).get('core') or []]
            out['rules'] = sorted({(r_[0], r_[1]) for r_ in (Jr or {}).get('rules') or []})
            out['lanes'] = sorted({l_ for l_, _f in out['rules']} | {l_ for k_ in out['core'] for l_ in k_})
            out['hands'] = sorted({l_ for l_, f_ in out['rules'] if f_ == 'hands'})
            # (an end whose own layer cannot hold -- the solve's 'toothlay' / 'berthlay': (lane, end))
            out['endlay'] = sorted({(l_, ('toothlay', 'berthlay').index(f_)) for l_, f_ in out['rules']
                                    if f_ in ('toothlay', 'berthlay')})
            # (a lane named for the ROOM of its change among its crossings has none for a change, as one whose own
            # window is empty: the solve's rule, read as its `nochange` is)
            out['nochange'] = sorted(set(out['nochange']) | {l_ for l_, f_ in out['rules'] if f_ == 'room'})
    return out


def repair_holds(mg, buses, fresh, prev_x, held_idx, let_go=True, refused=False, work=20.0):
    """THE REPAIR'S HELD ENDS, after a check's refusal: every lane the refusal did not name -- nor that crossed one it
    named -- held on the ends its last state had (`held_idx`), on model `mg` (every rule's gate set). The held lanes
    that CANNOT stand with what the refusal taught are let go too: asked with each held end an assumption, a "no
    state" names the ones in its way -- held as rules, the repair of a refusal that named one lane had no state in a
    second, three checks in four. ASKED FIRST WITH THE HELD ENDS SET, not assumed: a state there is the answer the
    assumptions give -- the same ends held, none let go -- and the solver reads a model with those ends set down to
    the lanes left free, where under assumptions it searches the whole of it; the assumptions are asked only where
    that has no state, for the names -- and not at all without `let_go` (a model whose held lanes are offered the one
    option: none can be let go on it). `refused`: the held ends are KNOWN to have no state (the repair's own search
    proved it): the names asked for at once, each solve in `work` deterministic seconds. Returns (the lanes left free, {a held end's variable: its (bus, lane)}, the
    lanes let go, {bus: {lane: why it is free}}, the last answer's solver, its status)"""
    from ortools.sat.python import cp_model
    import cp_seed
    n_free, hold_, info = 0, {}, {}
    for b in buses:
        fr_ = set(fresh.get(b.name, ()))
        ri_ = info.setdefault(b.name, {})
        ri_.update({l_: 'named' for l_ in fr_ if l_ in b.J.lanes})
        for pr_ in prev_x.get(b.name, ()):
            if fr_ & set(pr_):
                for l2_ in pr_:
                    if l2_ not in fr_ and l2_ in b.J.lanes:
                        ri_.setdefault(l2_, 'crossed ' + '/'.join(sorted(set(pr_) & fr_)))
        fr_ |= {l2_ for pr_ in prev_x.get(b.name, ()) if fr_ & set(pr_) for l2_ in pr_}
        n_free += len(fr_ & set(b.J.lanes))
        for l_ in b.J.lanes:
            if l_ in fr_:
                continue
            for k_ in (0, 1):
                i = held_idx[b.name][l_][k_]
                if i in b.J.xs[k_][l_]:
                    hold_[b.J.xs[k_][l_][i].Index()] = (b.name, l_)
    n_let, sf, st = 0, None, cp_model.UNKNOWN
    for _t in range(200):           # (each pass lets at least one lane go: it ends)
        if not (refused and _t == 0):
            mq = mg.Clone()
            mq.ClearObjective()
            pq_ = mq.Proto()
            for i_ in sorted(hold_):
                d_ = pq_.variables[i_].domain
                d_.clear()
                d_.extend([1, 1])
            sf = cp_seed.solver()
            sf.parameters.num_workers = 1
            sf.parameters.max_deterministic_time = work
            st = sf.Solve(mq)
            if st in (cp_model.OPTIMAL, cp_model.FEASIBLE) or not let_go:
                break
        mf = mg.Clone()
        mf.ClearObjective()
        mf.AddAssumptions([mf.GetBoolVarFromProtoIndex(i_) for i_ in sorted(hold_)])
        sf = cp_seed.solver()
        sf.parameters.num_workers = 1
        sf.parameters.max_deterministic_time = work
        st = sf.Solve(mf)
        if st != cp_model.INFEASIBLE:
            break
        core_l = {hold_[i_] for i_ in sf.SufficientAssumptionsForInfeasibility() if i_ in hold_}
        if not core_l:
            break
        hold_ = {i_: bl_ for i_, bl_ in hold_.items() if bl_ not in core_l}
        n_let += len(core_l)
        for bn_, l_ in core_l:
            info.setdefault(bn_, {}).setdefault(l_, f'let go: held, it could not stand with the rest (pass {_t + 1})')
    return n_free, hold_, n_let, info, sf, st


def objective_split(sv, buses, rows, between_cost, ride, walls, fans, chg, plane, far):
    """THE OBJECTIVE A SOLVE PAID, by tier, by family and by lane (a trace's -- checks.jsonl): the top tiers as counts
    (walled balls, plane balls fanned out, legs changing layer, teeth on a far face) and the vias' worth of runs on a
    plane's layer; the rest -- `cost`, below one change -- in vias, by what each term is (whole_ends._joint_model's
    cost_tag: an option's fanout vias, blocked front, feedback and tooth price; crossings, the learned price on them;
    the feedback's pairs; the arrays' rows, the meetings between buses, the ways round)"""
    val = sv.Value
    fam = collections.Counter()
    lane = collections.defaultdict(collections.Counter)
    for b in buses:
        for t_, (f_, ls_, parts) in zip(b.J.cost, b.J.cost_tag):
            v_ = val(t_)
            if not v_:
                continue
            if f_ == 'option':
                for p_, c_ in parts.items():
                    if p_ not in ('end', 'opt') and c_:
                        fam[p_] += c_
                        lane[(b.name, ls_[0])][p_] += c_
            elif f_ == 'crossing':
                fam['crossing'] += parts['crossing']
                fam['crossing_learned'] += parts['learned']
                for l_ in ls_:
                    lane[(b.name, l_)]['crossings'] += 1
                    lane[(b.name, l_)]['crossing_learned'] += parts['learned']
            else:
                fam[f_] += v_
                for l_ in ls_:
                    lane[(b.name, l_)][f_] += v_
        for t_ in b.J.cost[len(b.J.cost_tag):]:
            fam['untagged'] += val(t_)
    for ref, r_ in rows.items():
        fam['array_rows'] += sum(val(t_) for t_ in r_.cost)
    fam['between'] += sum(val(t_) for t_ in between_cost)
    fam['ride'] += sum(c_ * val(x_) for c_, x_ in ride)
    by_lane = {}
    for (bn_, l_), c_ in sorted(lane.items()):
        by_lane.setdefault(bn_, {})[l_] = {k_: (v_ if k_ == 'crossings' else v_ / 1000) for k_, v_ in c_.items()}
    return {'walls': int(sum(val(w_) for w_ in walls) - sum(val(v_) for v_ in fans)),
            'fans': int(sum(val(v_) for v_ in fans)), 'changes': int(sum(val(c_) for c_ in chg)),
            'plane_runs_vias': (sum(c_ * val(v_) for c_, v_ in plane) - sum(c_ * val(v_) for c_, v_ in far)) / 1000,
            'far_teeth': int(sum(val(v_) for _c, v_ in far)),
            'rest_vias': {k_: v_ / 1000 for k_, v_ in sorted(fam.items())}, 'lanes': by_lane}


def frame_check(b, laid, RL):
    """None, or why the whole route's FRAME refuses bus `b`'s ends as the model chose them (whole_frame.build on
    bus_bench's bench: a pair split by another lane's end, a tooth on the far face with no way round) -- the test a bus
    step's fanout audit makes of its board, made here before any step runs"""
    import whole_frame
    ctx = bus_bench(b, laid, RL)
    if ctx is None:
        return None
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            whole_frame.build(ctx, bus_dest(b.E.st))
    except SystemExit as e:
        return str(e)
    return None


def first_plan(b, laid, RL):
    """THE WHOLE SOLVE's plan of bus `b` from the ends the model chose it, on bus_bench's bench. Returns the solve's
    own line (its crossings, its vias, whether it proved the plan) and its seconds"""
    import whole_solve
    ctx = bus_bench(b, laid, RL)
    if ctx is None:
        return 'no bench (a plan in the turned-over pair frame)', 0.0
    t0, buf = time.time(), io.StringIO()
    with contextlib.redirect_stdout(buf):
        J = whole_solve.solve(ctx, bus_dest(b.E.st), front=b.plan.get('front') or {})
    line = next((ln.strip() for ln in reversed(buf.getvalue().splitlines()) if ln.strip().startswith('whole_solve:')),
                f'no plan (status {(J or {}).get("status")})')
    return line, time.time() - t0


def bus_key(dest):
    """a bus's name in the group's model by its destination as a bus step names it: a bus to several parts ('A+B') is
    'A_B' there, as its capture's file is named"""
    return dest.replace('+', '_')


def bus_nets(group, dest):
    """the nets the group's model (its OUT directory `group`) gave bus `dest` its ends, in the model's order"""
    g = json.load(open(os.path.join(group, 'group.json'))).get('buses') or {}
    dest = bus_key(dest)
    if dest not in g or not g[dest].get('plan'):
        raise SystemExit(f'group_ends: {os.path.join(group, "group.json")} holds no plan for the bus to {dest}')
    return list(g[dest]['plan']['nets'])


def layers_free(group, dest):
    """the group's model (its OUT directory `group`) left the layers of bus `dest`'s ends to the bus's own solve, but
    the lanes its plan holds (`held_layers`: they cross another bus's lane, or run along one under an array)"""
    g = json.load(open(os.path.join(group, 'group.json'))).get('buses') or {}
    return ((g.get(bus_key(dest)) or {}).get('plan') or {}).get('held_layers') is not None


def bus_stage(out_path, base, log=print):
    """A BUS STEP'S FANOUT STAGE under the group's ends (GROUP_ENDS=DIR, the group's OUT; fanout_from_plan's main):
    nothing is fanned out and no end is chosen. BASE -- the group's field, in the bus's frame -- is the round's board
    as it stands, each of the bus's ends on it where the group's model laid it, and the plan sidecar written beside it
    is the model's own (bus_plan): the ends, the layers the model gave them, the escape directions, the cut. It STOPS,
    naming why, where the bench is not the board the model read: another frame, a net of the run with no ends in the
    model, an end with none of its net's copper at its exit. Then the fanout audit, as every fanout's"""
    import fanout_from_plan as fp
    from coherent_nets import by_key
    from kicad_parser import parse_kicad_pcb
    group, dest = awx_settings.req('GROUP_ENDS'), awx_settings.req('DEST')
    g = json.load(open(os.path.join(group, 'group.json')))
    gb = (g.get('buses') or {}).get(bus_key(dest))
    if not gb or not gb.get('plan'):
        raise SystemExit(f'group ends: {os.path.join(group, "group.json")} holds no plan for the bus to {dest}')
    plan = gb['plan']
    names = [n for n in (awx_settings.get('FANOUT_NETS') or '').split(',') if n] or list(plan['nets'])
    miss = [n for n in names if n not in plan['ends']]
    if miss:
        raise SystemExit(f'group ends: {dest}: the run\'s nets with no ends in the group\'s model: {", ".join(miss)}')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(base)
    pads = {p.pad_number: (p.global_x, p.global_y) for p in pcb.footprints[gb['src']].pads}
    off = [n for n, q in gb['frame'].items() if n not in pads or math.hypot(pads[n][0] - q[0], pads[n][1] - q[1]) > 2e-3]
    if off:
        raise SystemExit(f'group ends: {dest}: {os.path.basename(base)} is not in the frame the group\'s model read '
                         f'the bus in ({gb["src"]}\'s pads {", ".join(off)} stand elsewhere)')
    byname = by_key(pcb)
    bare = []
    for nm in names:
        nid = byname[nm][0]
        pts = [q for s_ in pcb.segments if s_.net_id == nid for q in ((s_.start_x, s_.start_y), (s_.end_x, s_.end_y))]
        pts += [(v.x, v.y) for v in pcb.vias if v.net_id == nid]
        bare += [f'{nm} at its {("tooth", "berth")[k_]}' for k_ in (0, 1)
                 if not any(math.hypot(q[0] - plan['ends'][nm][k_][0], q[1] - plan['ends'][nm][k_][1]) < 2e-3 for q in pts)]
    if bare:
        raise SystemExit(f'group ends: {dest}: no copper of its net at the end the group\'s model laid for '
                         + ', '.join(bare))
    shutil.copyfile(base, out_path)
    fp.copy_pro(base, out_path)
    side = {k: {n: plan[k][n] for n in names} for k in ('ends', 'tooth_layer', 'dest_layer', 'tooth_dir', 'stub_dir')}
    side.update(nets=names, chi=int(plan['chi']), pages_first=False, escape=True,
                ends_model={k: {} for k in ('over', 'load', 'x', 'front')})
    # (the blocked fronts the model chose, for the rounds' solves as for the model's own check: bus_plan)
    side['ends_model']['front'] = {n: v for n, v in (plan.get('front') or {}).items() if n in names}
    # (a bus to SEVERAL PARTS: each net's own part, as the bus's own fanout writes it -- the berths' box of each part
    # by itself. With none, the round's feedback and its ends' repair read ONE box round every part's berths, the
    # gap between the parts in it: a finding out between two parts was read as at the berths, a loop stopped for
    # its ends on findings its solve had still to answer, and the model was told of pairs of ends never crowded
    # together)
    parts_ = [r_ for r_ in dest.split('+') if r_]
    if len(parts_) > 1:
        side['dest_of'] = {}
        for nm in names:
            ps_ = [p_ for p_ in byname[nm][1].pads if p_.component_ref in parts_]
            if ps_:
                e_ = plan['ends'][nm][1]
                side['dest_of'][nm] = min(ps_, key=lambda p_: math.hypot(p_.global_x - e_[0],
                                                                         p_.global_y - e_[1])).component_ref
    # (the cuts the model read the ends at, and a group's ways in: the frame takes the ones it is given)
    for k in ('dest_cut', 'src_cut', 'dest_entry'):
        if plan.get(k) is not None:
            side[k] = plan[k]
    if plan.get('held_layers') is not None:     # (the lanes whose ends' layers stay the model's: layers_free)
        side['held_layers'] = [n for n in plan['held_layers'] if n in names]
    if plan.get('end_layers'):                  # (the layers each end has room on: bus_plan)
        side['end_layers'] = {n: plan['end_layers'][n] for n in names if n in plan['end_layers']}
    with open(out_path[:-len('.kicad_pcb')] + '.plan.json', 'w', encoding='utf-8') as f:
        json.dump(side, f, indent=1)
    by = collections.Counter((('tooth', 'berth')[k_], side[('tooth_layer', 'dest_layer')[k_]][n]) for n in names
                             for k_ in (0, 1))
    log(f'group ends: {len(names)} nets to {dest}, each end as the group\'s model laid it on '
        f'{os.path.basename(base)} -- nothing fanned out, no end chosen')
    log(f'  plan model (the group\'s ends): {gb.get("crossings")} crossings, {gb.get("changes")} lanes changing layer; '
        + ', '.join(f'{e_} {L_} {n_}' for (e_, L_), n_ in sorted(by.items())))
    sp, far, stops = fp.fanout_audit(out_path, names, dest)
    if sp or far or stops:
        held = fp.set_aside_refused(out_path, sp, far, stops)
        log(f'REFUSED by the fanout audit: {fp._audit_line(sp, far, stops)} -- the board set aside as '
            f'{os.path.basename(held)}')
        return 3
    log('fanout audit: clean')
    return 0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    flags = [a for a in sys.argv[1:] if a.startswith('--')]
    if '--help' in flags or '-h' in sys.argv[1:] or len(args) < 2:
        print('usage: ' + __doc__.split(' -- ', 1)[0].strip())
        sys.exit(0 if '--help' in flags or '-h' in sys.argv[1:] else 2)
    cap_dir, out = args[0], args[1]
    board = next((a.split('=', 1)[1] for a in flags if a.startswith('--board=')), None)
    if not board:
        raise SystemExit('group_ends: --board=BOARD, the bare board the arrays\' menus are measured on')
    off = {f[len('--no-'):] for f in flags if f.startswith('--no-')}       # (a rule left out: --no-vias/runs/region)
    os.makedirs(os.path.join(out, 'source'), exist_ok=True)
    log = lambda *a: print(*a, flush=True)   # noqa: E731
    est_parts = {}              # (two routing layers) bus -> its state's estimate: its end vias and its forced changes
    caps = {}
    for f in sorted(os.listdir(cap_dir)):
        if f.endswith('.ends.pkl'):
            with open(os.path.join(cap_dir, f), 'rb') as fh:
                caps[f[:-len('.ends.pkl')]] = pickle.load(fh)
    t_all = time.time()
    log(f'group ends: {len(caps)} buses -- {", ".join(caps)}')
    # (WHAT THE RUN SAVES for a person to read: the code that ran, OUT/run.json; and the model's every step, a record a
    # line in OUT/checks.jsonl -- each solve's objective by family and lane, each bus's state and the ends that moved
    # since the last, each check's whole verdict with its solve's console, each lesson it taught: bus_trace.py reads it)
    import run_trace
    run_trace.provenance(out, {'stage': 'group_ends', 'caps': os.path.abspath(cap_dir),
                               'board': os.path.abspath(board), 'flags': flags})
    TR = run_trace.Trace(os.path.join(out, 'checks.jsonl'))
    trace_prev = {}             # bus -> {lane: ((its tooth's signature, layer), (its berth's ...))}: the state before
    first = next(iter(caps.values()))
    srefs = {cap['st']['sref'] for cap in caps.values()}
    if len(srefs) != 1:
        raise SystemExit(f'group_ends: the buses share one source, not {sorted(srefs)}')
    sref = srefs.pop()
    bus_events.for_group(sref)
    # (the group's own settings: the first bus's run's, without what names that bus's own fanout)
    genv = {k: v for k, v in first['env'].items() if k not in OWN_KEYS}
    with awx_settings.given(genv):
        import rules as _rules
        _rules.install_defaults()
        # (the run's sizes -- the lanes' track, the escapes', the via: kept with its end, resume.pkl)
        sizes_now = tuple(round(float(v_), 6) for v_ in (_rules.active().track, _rules.active().fan_track,
                                                         _rules.active().via_size, _rules.active().via_drill))
        import whole_ends as we
        import whole_solve
        import cp_seed
        import escape_moves as em
        import joint_escape as je
        import route_layers
        from kicad_parser import parse_kicad_pcb
        from ortools.sat.python import cp_model
        we.OPP_SOFT = True          # (a pair that must cross over pays for it, ranked as a change)
        we.JOINT_X_LAYERS = True    # (a crossing priced by the layers its two lanes share: whole_ends.JOINT_X_LAYERS)
        import fanout_from_plan as fp_
        we.FRONT_REACH = fp_.PAIR_EXIT_REACH = EXIT_REACH       # (this run's alone: a single's front, a pair's)
        # ---- THE BOARD PREPARED: the movable passives beside each bus's ROW PART destinations moved off its escapes
        # there, as every bus's own step does before its fanout (route_bus.clear_row_escapes: a pad of a row part has
        # one escape, straight out of its row, and a cap standing across it leaves its lane no end) -- once, for the
        # group, on a copy (OUT/prepared.kicad_pcb), which is then the board the menus are measured on and the field
        # is laid on. Left where they stood, a three-part bus's berths' vias were laid onto a connector's decaps
        import route_bus as rb
        with contextlib.redirect_stdout(io.StringIO()):
            pcb0 = parse_kicad_pcb(board)
        by0 = {n_.name: n_ for n_ in pcb0.nets.values() if n_.name}
        prep, moved_ = os.path.join(out, 'prepared.kicad_pcb'), []
        os.makedirs(os.path.join(out, 'prepare'), exist_ok=True)
        shutil.copyfile(board, prep)
        fp_.copy_pro(board, prep)
        for nm, cap in caps.items():
            st = cap['st']
            full = [st['byname'][n][1].name for n in st['launch'] if n in st['byname']]
            keep = {q_.component_ref for n_ in full if n_ in by0 for q_ in by0[n_].pads}
            for d_ in st.get('dests') or [st['dref']]:
                with awx_settings.given(cap['env']):
                    moved_ += rb.clear_row_escapes(prep, d_, full, sorted(keep - {sref, d_}),
                                                   os.path.join(out, 'prepare'), lambda *a: None, zone=row_zone) or []
        if moved_:
            log(f'  the board prepared: {len(set(moved_))} passive(s) moved off the row parts\' escapes -- '
                + ', '.join(sorted(set(moved_))[:16]) + f' -> {prep}')
            board = prep
        with contextlib.redirect_stdout(io.StringIO()):
            bpcb = parse_kicad_pcb(board)
        # (THE TIE VIAS' SITES, stood in on the board the menus are measured on -- each its own net's via, so that
        # the other nets' escapes go round it as round any via; the vias themselves are laid with the field,
        # write_field. Unknown to the plans, a tie via was once laid on its neighbour's escape run: a short)
        from coherent_nets import by_key
        byk0 = by_key(bpcb)
        ties_of, tie_sites = [], []
        # (THE PULLS: each bus's nets with other pads beside their two ends, as its own step reads them
        # (route_bus.pulled_nets) -- their stub pads off their nets while the field's passives are nudged, write_field)
        import make_bench as mb
        pulls = []
        for nm, cap in caps.items():
            st = cap['st']
            mine_ = {byk0[n][1].name for n in st['launch'] if n in byk0}
            for d_ in st.get('dests') or [st['dref']]:
                for n_, ps_ in sorted(rb.pulled_nets(bpcb, sref, d_, mb.pair_nets(bpcb, sref, d_)).items()):
                    if n_ in mine_:
                        pulls += [p_ for p_ in ps_ if p_ not in pulls]
        if pulls:
            log(f'  {len(pulls)} pull pad(s) of the buses\' nets, off their nets while the field\'s passives are nudged: '
                + ', '.join(sorted(f'{p_.component_ref}.{p_.pad_number}' for p_ in pulls)[:24]))
        for nm, cap in caps.items():
            st = cap['st']
            if len(st.get('dests') or [st['dref']]) == 1 and not em.row_part(bpcb.footprints[st['dref']]):
                nms_ = [n for n in st['launch'] if n in byk0]
                dpad_ = {n: next((p for p in byk0[n][1].pads if p.component_ref == st['dref']), None) for n in nms_}
                stand_ = fp_.tie_stand_ins(bpcb, nms_, byk0, dpad_)
                ties_of.append((nms_, st['dref']))
                ball_ = {byk0[n][0]: dpad_[n] for n in nms_ if dpad_.get(n) is not None}
                tie_sites += [(v_, (ball_[v_.net_id].global_x, ball_[v_.net_id].global_y,
                                    min(ball_[v_.net_id].size_x, ball_[v_.net_id].size_y) / 2)) for v_ in stand_]
                if stand_:
                    bpcb.vias = list(bpcb.vias) + stand_
                    log(f'  {nm}: {len(stand_)} ball(s) of {st["dref"]} with a pad of their own net under them: a tie via '
                        f'each, its site kept clear')
        at = collections.defaultdict(dict)          # array -> {net: its bus}
        for nm, cap in caps.items():
            st = cap['st']
            at[sref].update({n: nm for n in st['launch']})
            if len(st.get('dests') or [st['dref']]) == 1 and not em.row_part(bpcb.footprints[st['dref']]):
                at[st['dref']].update({n: nm for n in st['launch']})
        arrays = {}
        for ref, bus_of in at.items():
            t0 = time.time()
            A = arrays[ref] = build_array(bpcb, ref, bus_of)
            n0_ = sorted(len(A.offer[k]) for k in A.bm.items)
            log(f'  {ref}: {len(A.ball_of)} bus balls, {sum(1 for k in A.okeys if k not in A.drops)} other signal '
                f'balls' + (f' ({len(A.served)} more served already, their copper the board\'s)' if A.served else '')
                + f', {len(A.drops)} plane balls with {sum(len(v) for v in A.drops.values())} drops; the pattern '
                f'{sum(n0_)} moves (a ball {n0_[0]} to {n0_[-1]}, median {n0_[len(n0_) // 2]}); {len(A.cliques)} + '
                f'{len(A.bicliques)} groups of vias ({A.bm.n_near} of them two vias too near), {len(A.pts)} points of '
                f'its lines; {len(A.dead)} moves in a '
                f'keep-out area left out ({time.time() - t0:.0f} s)')
            if '--verify' in flags:
                seen, bad = verify_runs(A.bm, A.pts, A.cliques, A.bicliques)
                log(f'    verified against the whole route\'s conflict test on {seen} pairs of moves: '
                    + (f'{sum(bad.values())} DIFFER {bad}' if bad else 'the same on every one'))
    # (THE CAPTURES' BOARD where it is not BOARD -- --captured-on: what BOARD holds inside the arrays that it has
    # not is put in each bus's view of its board, load_bus)
    board_cu = None
    cap_on = next((a.split('=', 1)[1] for a in flags if a.startswith('--captured-on=')), None)
    if cap_on:
        with contextlib.redirect_stdout(io.StringIO()):
            cpcb_ = parse_kicad_pcb(cap_on)
        nm_ = lambda i_: bpcb.nets[i_].name if i_ in bpcb.nets else None        # noqa: E731
        board_cu = {ref_: ([(s_, nm_(s_.net_id)) for s_ in ss_], [(v_, nm_(v_.net_id)) for v_ in vs_])
                    for ref_, (ss_, vs_) in kept_copper(bpcb, cpcb_, arrays).items()}
        log('  the board\'s copper inside the arrays that the captures\' board has not, in each bus\'s view: '
            + ', '.join(f'{ref_} {len(c_[0])} tracks and {len(c_[1])} vias' for ref_, c_ in sorted(board_cu.items())))
    # ---- THE PATTERN first: each ball its pattern's moves, its whole menu only where the pattern meets a problem
    every = {(A.ref, k) for A in arrays.values() for k in A.bm.items}

    # (THE MENUS BY NEED AND BY USE: `narrowed` the bus balls on their first TIER_MENU tiers now; `whole_opts` each bus
    # ball's whole menu, kept; `tier_sig` its moves' tiers by signature; `mver` each ball's menu's version -- a bus's
    # ends model is built again when a menu of its balls changes)
    narrowed, whole_opts, tier_sig, mver = set(), {}, {}, collections.Counter()

    def retier(ref, ks):
        for k in ks:
            if (ref, k) in tier_sig:
                S_k, sig_k = tier_sig[(ref, k)]
                tier_of[(ref, k)] = [S_k.get(sig_k(o_), 5) for o_ in arrays[ref].bm.menu[k]]

    def whole(more):
        """the balls `more` {(array, ball)} offered their whole menus: built now (widen) -- a bus ball's, kept, back"""
        for ref in sorted({r_ for r_, _k in more}):
            t0, ks = time.time(), {k for r_, k in more if r_ == ref}
            A_ = arrays[ref]
            back_ = {k for k in ks if (ref, k) in whole_opts}
            for k in back_:
                A_.bm.opts[k] = list(whole_opts[(ref, k)])
                A_.bm.menu[k] = [o_ for kd_, o_, _pc in A_.bm.opts[k] if kd_ == 'escape']
            if ks - back_:
                with awx_settings.given(genv):
                    widen(bpcb, A_, ks - back_)
            else:
                array_rules(bpcb, A_)
            for k in ks:
                mver[(ref, k)] += 1
                narrowed.discard((ref, k))
            retier(ref, ks)
            log(f'    {ref}: {len(ks)} balls\' whole menus built, {sum(len(A_.bm.menu[k]) for k in ks)} moves '
                f'({time.time() - t0:.0f} s)')

    def release(bs):
        """the model built last let go before the next is built: each bus's joint model (b.J) holds its CpModel. Held
        on, a stretch's model was built beside the last one -- a three-array group's footprint 9.1 GB"""
        for b_ in bs:
            b_.J = None
        gc.collect()

    def narrow(keys, keep=None, rules=True):
        """bus balls `keys` {(array, ball)} back on their first TIER_MENU tiers -- and each on the move `keep` names
        ({key: its signature}: the one its lane took). `rules` False: the arrays' rules' data NOT made again here --
        the arrays named are returned, for a caller that changes their menus once more before anything reads it"""
        refs = sorted({r_ for r_, _k in keys})
        for ref in refs:
            A_ = arrays[ref]
            ks = sorted(k for r_, k in keys if r_ == ref)
            for k in ks:
                S_k, sig_k = tier_sig[(ref, k)]
                kp_ = (keep or {}).get((ref, k))
                A_.bm.opts[k] = [x_ for x_ in whole_opts[(ref, k)] if x_[0] != 'escape'
                                 or S_k.get(sig_k(x_[1]), 5) <= TIER_MENU or (kp_ is not None and sig_k(x_[1]) == kp_)]
                A_.bm.menu[k] = [o_ for kd_, o_, _pc in A_.bm.opts[k] if kd_ == 'escape']
                mver[(ref, k)] += 1
                narrowed.add((ref, k))
            if rules:
                array_rules(bpcb, A_)
            retier(ref, ks)
        return set() if rules else set(refs)
    wide = set(every) if '--full' in flags else {(A.ref, k) for A in arrays.values() for k in A.bm.items
                                                 if not A.offer[k]}
    if wide:
        log(f'  {len(wide)} balls offered their whole menu from the start')
        whole(wide)
    # THE TIERS: every bus ball on its WHOLE menu, each move tagged with the first tier that holds it --
    # 0 the pattern, 1 the pattern from the next clear site (a pair's balls: the lines too), 2 the straight moves toward
    # the bus's other part from every clear site and from a via in the pad, 3 those by every face, 4 the lines, 5 the
    # rest of the whole menu. The search starts on the first tiers and widens where it pays (tier_search). A ball
    # whose pattern's gap point stood blocked by a part on the other side had no dog-bone at all; on the pattern alone
    # a bus's lanes ran down inside an FPGA to its far face, a pair round the back of the memory across the whole bus
    tier_of = {}                # (array, ball) -> the tier of each move of its menu

    def tbox_(ref_):
        if ref_ in arrays:
            return arrays[ref_].box
        xs_ = [p_.global_x for p_ in bpcb.footprints[ref_].pads]
        ys_ = [p_.global_y for p_ in bpcb.footprints[ref_].pads]
        return (min(xs_), min(ys_), max(xs_), max(ys_))

    def tface_(ref_, other_):
        # (the face of `ref_` turned toward `other_`)
        a_, b_ = tbox_(ref_), tbox_(other_)
        cx_, cy_ = (b_[0] + b_[2] - a_[0] - a_[2]) / 2, (b_[1] + b_[3] - a_[1] - a_[3]) / 2
        return ('right' if cx_ > 0 else 'left') if abs(cx_) >= abs(cy_) else ('down' if cy_ > 0 else 'up')
    tsig_ = lambda o_: (o_.kind, o_.direction, tuple(round(v_, 4) for v_ in (o_.site or ())),     # noqa: E731
                        tuple((tuple(round(v_, 4) for v_ in a_), tuple(round(v_, 4) for v_ in b_))
                              for a_, b_, _l in o_.legs))
    for ref in sorted(arrays):
        A_ = arrays[ref]
        ks_ = sorted(set(A_.ball_of.values()) - {k for r_, k in wide if r_ == ref})
        if not ks_:
            continue
        t0 = time.time()
        net_of_ = {k: n_ for n_, k in A_.ball_of.items()}
        pr_ = je.array_pairs(bpcb.footprints[ref], set(A_.bus_of), je.namer(bpcb))
        pk_ = {A_.ball_of[leg] for legs_ in pr_.values() for leg in legs_ if leg in A_.ball_of}
        face_ = {}
        for k in ks_:
            dref_ = caps[A_.bus_of[net_of_[k]]]['st']['dref']
            face_[k] = tface_(ref, dref_ if ref != dref_ else sref)
        S_ = {k: {tsig_(o_): 0 for o_ in A_.bm.menu[k]} for k in ks_}
        ent_ = {k: {tsig_(x_[1]): x_ for x_ in A_.bm.opts[k] if x_[0] == 'escape'} for k in ks_}

        def tadd_(ns_, t_, only_=None, toward_=False):
            for k in ks_:
                if only_ is not None and k not in only_:
                    continue
                for x_ in ns_.opts[k]:
                    if x_[0] != 'escape' or (toward_ and x_[1].direction != face_[k]):
                        continue
                    S_[k].setdefault(tsig_(x_[1]), t_)
                    ent_[k].setdefault(tsig_(x_[1]), x_)
        try:
            je.PATTERN_NEXT_SITE = True
            with awx_settings.given(genv):
                tadd_(_menus(bpcb, ref, A_.nets, only=set(ks_)), 1)
                n4_ = _menus(bpcb, ref, A_.nets, only=set(ks_), lines=True)
                tadd_(n4_, 1, only_=pk_)
                je.PATTERN_ALL_SITES = True
                n3_ = _menus(bpcb, ref, A_.nets, only=set(ks_))
                je.PATTERN_ALL_SITES = False
                tadd_(n3_, 2, toward_=True)
                tadd_(n3_, 3)
                tadd_(n4_, 4)
        finally:
            je.PATTERN_ALL_SITES = je.PATTERN_NEXT_SITE = False
        more_ = {(ref, k) for k in ks_}
        whole(more_)
        wide |= more_
        n_miss = 0
        for k in ks_:
            # (a narrower tier's move the whole menu does not hold -- its site from another gap point -- added)
            have_ = {tsig_(x_[1]) for x_ in A_.bm.opts[k] if x_[0] == 'escape'}
            add_ = [x_ for sg_, x_ in ent_[k].items() if sg_ not in have_]
            if add_:
                A_.bm.opts[k] = list(A_.bm.opts[k]) + add_
                A_.bm.menu[k] = [o_ for kd_, o_, _pc in A_.bm.opts[k] if kd_ == 'escape']
                n_miss += len(add_)
        for k in ks_:
            tier_sig[(ref, k)] = (S_[k], tsig_)
            whole_opts[(ref, k)] = list(A_.bm.opts[k])
        cnt_ = collections.Counter(S_[k].get(tsig_(o_), 5) for k in ks_ for o_ in A_.bm.menu[k])
        narrow({(ref, k) for k in ks_})
        log(f'  {ref}: {len(ks_)} bus balls, their whole menus\' moves by tier {dict(sorted(cnt_.items()))}'
            + f' -- the first {TIER_MENU + 1} tiers offered, {sum(len(A_.bm.menu[k]) for k in ks_)} moves'
            + f'; {n_miss} of the narrower tiers\' moves the whole menu lacked, added ({time.time() - t0:.0f} s)')
    # THE LINES are a bus's when the whole solve refuses its pattern ends (the check below), and with them its cut
    # round its destination ranked (load_bus): the pattern alone ties a ball's exit to its own row or column, the
    # exits' order round the array then the balls' own -- a bus whose balls stand in another order than its berths
    # ended with three times the crossings of its own run's ends, and no plan. From the start for every bus they cost
    # a four-bus model minutes to its first state
    lined = set()
    # THE LAST STATE, carried to the model built next as its start (each end by its signature, each other ball's
    # choice): a model solved again starts where the last one stood, not anywhere
    prev_bus, prev_oth = {}, {}
    built = {}                  # (bus, its balls offered their whole menu) -> its ends model
    cuts = {}                   # (bus, on the lines) -> (its cut round its destination,), the one its first state had
    layers_want = {}            # bus -> {lane: (its tooth's layer, its berth's)}: the whole solve's own, taken (the check)
    searched = set()            # (bus, on the lines): its ends searched whole on that tier of its menus
    # WHAT THE WHOLE SOLVE'S CHECKS TAUGHT, a bus: the lanes held to one layer, the crossings held apart, the states
    # refused (each lane's two ends by their signature, so that they stand on a menu built again)
    learned = {}
    oth_ban = set()             # (array, ball, 'x' a fan-out / 'd' a drop, which): an other ball's choice ruled out
    from_first = False          # (--from) the first model built from the run before: its last state the ends wanted
    held = {}                   # a part the nudge could not clear -> its pads where it came: no copper on them (array_rows)
    # ---- FROM A RUN BEFORE (--from=ITS OUT) and WHAT ITS BUSES' STEPS FOUND (--feedback=THEIR OUT): the model as that
    # run left it -- the balls widened, the buses on the lines, what the checks taught, each bus's cut, the last state
    # -- solved again with the ends a step's audits found crowded RULED OUT: a lane's end at the very place (and, an
    # end the route cannot move to another layer, on the very layer) its step named, two lanes' ends it named together
    # not both. The lanes named are asked nothing of where they stood; every other end is wanted where it was, so the
    # state nearest the last one that answers the findings is the one taken. A bus step cannot change the group's
    # ends, and a step that left nets open on them had nowhere to send what it found
    frm = next((a.split('=', 1)[1] for a in flags if a.startswith('--from=')), None)
    # (a bench's: FROM a state kept in a search.pkl, straight to the search's sweeps -- --at-start, the state its start's
    # -- or to its checks, --at-search, the state its search's end. And a RUNG's (route_bus: the group escalated):
    # --at-end, from the run's END (its resume.pkl), its step's findings answered by the repair and every other lane
    # held -- on the rung's finer menus, which hold every move of the coarser ones -- to the checks at the rung's sizes:
    # searched again from nothing, a rung cost what the first run had, 51 to 87 minutes of a group's model)
    at_point = next((f_ for f_ in ('--at-start', '--at-search', '--at-end') if f_ in flags), None) if frm else None
    fbd = next((a.split('=', 1)[1] for a in flags if a.startswith('--feedback=')), None)
    # (the group's escalation's, --rungs-below=TRACK:VIA,...: the lanes' track and the via of each rung below these
    # sizes -- NO ROOM AT THESE SIZES, at the checks)
    rungs_below = [tuple(float(v_) for v_ in r_.split(':')) for r_ in
                   next((a.split('=', 1)[1] for a in flags if a.startswith('--rungs-below=')), '').split(',') if r_]
    feedback, fed = {}, {}      # bus -> its step's findings (whole_feedback's file); bus -> its lanes they name
    found_at = {}               # bus -> the state its step was laid on, which its findings are of: the run before's
    check_plans = {}            # bus -> the plan its last check with one found: its next check asks it first
    hard = {}                   # bus -> its step's findings that STAND (group_findings_hard.json): ruled out
    if fbd:
        import glob as _glob
        for nm, cap in caps.items():
            # (a bus's step leaves its findings in its own run: `<source>-<bus>.bus/run` where the step is the bus's
            # alone, `<bus>/bus.bus/run` where the buses are laid on the group's field, each in a directory of its
            # name. Read by the first form alone, a lay on the field handed the model nothing -- the round after a
            # lay that left nets open searched again from its state, told of none of them)
            fs = sorted(f_ for pat_ in {nm, nm.replace('_', '+'), '+'.join(cap['st'].get('dests') or [nm])}
                        for g_ in (os.path.join(fbd, '**', f'*-{pat_}.bus', 'run', 'group_findings.json'),
                                   os.path.join(fbd, '**', pat_, 'bus.bus', 'run', 'group_findings.json'))
                        for f_ in _glob.glob(g_, recursive=True))
            if fs:
                feedback[nm] = json.load(open(fs[-1]))
                fb_ = feedback[nm]
                # (...ruled out: the ones that STAND -- the step's own file of them; every one is priced, by the ends
                # model's own tables of the whole file)
                fh_ = fs[-1][:-len('.json')] + '_hard.json'
                hard[nm] = json.load(open(fh_)) if os.path.isfile(fh_) else {'avoid': [], 'pairs': []}
                fed[nm] = {it_['lane'] for it_ in (hard[nm].get('avoid') or []) + (fb_.get('ban') or [])} | \
                    {e_['lane'] for pr_ in hard[nm].get('pairs') or [] for e_ in pr_}
                log(f'  {nm}: its step\'s findings ({os.path.relpath(fs[-1], fbd)}): {len(fb_.get("avoid") or [])} end(s) '
                    f'to avoid and {len(fb_.get("pairs") or [])} pair(s) of ends priced; of them '
                    f'{len(hard[nm].get("avoid") or [])} end(s) and {len(hard[nm].get("pairs") or [])} pair(s) stood two '
                    f'audits running and are ruled out, naming {len(fed[nm])} lane(s): ' + ', '.join(sorted(fed[nm])[:16]))
    if fbd and not feedback:
        log(f'  --feedback: no bus\'s step left findings under {fbd} -- the model is told nothing')
    if frm:
        R_ = pickle.load(open(os.path.join(frm, 'search.pkl' if at_point in ('--at-start', '--at-search')
                                           else 'resume.pkl'), 'rb'))
        more_ = set(R_['wide']) - wide
        if more_:
            wide |= more_
            whole(more_)
        lined |= set(R_['lined'])
        for nm in sorted(lined):
            st_ = caps[nm]['st']
            for ref in (sref, st_['dref']):
                if ref in arrays:
                    A_ = arrays[ref]
                    ks_ = {A_.ball_of[n_] for n_ in st_['launch'] if n_ in A_.ball_of and (ref, A_.ball_of[n_]) not in wide}
                    if ks_:
                        with awx_settings.given(genv):
                            expand(bpcb, A_, ks_)
        # (THE MENUS AS THE RUN LEFT THEM: each bus ball on its first tiers and the move its lane holds, or whole
        # where a need had widened it -- a bench's FROM A RUN'S SEARCH, its search.pkl, and a run FROM A RUN'S END.
        # A ball's move past its first tiers is on its menu because its lane took it (narrow's `keep`); built
        # again on the first tiers alone, 105 of a four-bus group's 454 ends were not on their menus, each was
        # read as the nearest option there, and the state so read broke the arrays' own rules: asked about two
        # lanes, the repair let 48 others go in four buses and every bus's check refused the answer; asked about
        # four, the model went on for seven checks and moved 160 of 227 lanes' ends)
        nar_ = set(narrowed) if R_.get('narrowed') is None else set(R_['narrowed'])
        keep_ = R_.get('keep')
        if keep_ is None:
            # (...the moves not kept with it -- a run's end, or a state put together from a run's trace: each
            # narrowed bus ball on the move of its whole menu that a lane's end in the state names)
            # (...read in its bus's own frame: a lane's end is recorded as its bus's ends model sees it, a quarter
            # turn of the board's for a bus off another face)
            import source_realize as sr_
            sigs_ = {nm_: {sg_ for e_ in pb_.values() for end_ in e_ for sg_ in end_[0] if isinstance(sg_, tuple)}
                     for nm_, pb_ in R_['prev_bus'].items()}
            turn_ = {}
            for nm_, cap_ in caps.items():
                st_ = cap_['st']
                turn_[nm_] = frame_map(arrays[st_['sref']].pads, {
                    p_.pad_number: (p_.global_x, p_.global_y) for p_ in st_['pcb'].footprints[st_['sref']].pads})
            keep_ = {}
            for key_ in sorted(nar_):
                A_ = arrays[key_[0]]
                nm_ = next((A_.bus_of[n_] for n_, k_ in A_.ball_of.items() if k_ == key_[1]), None)
                if nm_ not in sigs_:
                    continue
                x_ = next((x_ for x_ in whole_opts.get(key_, ()) if x_[0] == 'escape'
                           and sr_.move_sig(turn_move(x_[1], turn_[nm_])) in sigs_[nm_]), None)
                if x_ is not None:
                    keep_[key_] = tier_sig[key_][1](x_[1])
            log(f'  from {frm}: {len(keep_)} of {len(nar_)} narrowed balls keep the move their lane holds')
        go_ = {key_ for key_ in narrowed if key_ not in nar_}
        if go_:
            whole(go_)
        narrow({key_ for key_ in nar_ if key_ in tier_sig}, keep_)
        prev_bus, prev_oth, learned = R_['prev_bus'], R_['prev_oth'], R_['learned']
        # (WHAT A CHECK TAUGHT OF A LANE'S ROOM FOR A CHANGE OF LAYER IS ITS SIZES': from a run at other sizes -- a
        # rung's start -- the lanes held to one layer and the lengths no change fits in are not kept. The rung is
        # routed to give that room, and its own checks say what has none there)
        if R_.get('sizes') is not None and tuple(R_['sizes']) != sizes_now:
            n_l = sum(len(L_.get('nochg') or ()) for L_ in learned.values())
            n_k = sum(len(L_.get('room') or ()) for L_ in learned.values())
            for L_ in learned.values():
                L_['nochg'], L_['room'] = set(), {}
            if n_l or n_k:
                log(f'  from {frm}: a run at other sizes -- the {n_l} lane(s) it held to one layer and the {n_k} '
                    f'length(s) it found too short for a change of layer are not held here')
        found_at = dict(prev_bus)       # (each bus's state as the run before left it: `prev_bus` moves on with this run)
        layers_want = R_['layers_want']
        oth_ban |= set(R_.get('oth_ban') or ())
        held.update(R_.get('held') or {})
        cuts.update(R_['cuts'])
        if not feedback or at_point == '--at-end':
            searched |= set(R_['searched'])     # (with findings to answer, the ends are searched whole again: many
            #                                     lanes may have to move for a good state under what they rule out.
            #                                     A RUNG's are not -- the repair answers them: read as not searched,
            #                                     each bus's ends were descended again for the model after the
            #                                     repair, the lanes it held stood elsewhere than wanted, and the
            #                                     state nearest moved 24 ends of a bus no finding named)
        # (...from the LAST STATE all the same, the group's: the first model built from it wants every end where that
        # state had it, none descended again a bus at a time -- from_first. Each bus's ends descended alone for its
        # own crossings, a four-bus group's start stood 76 of 200 ends from its last state to rule out ONE end, and
        # the search from there ended at four times the upper rank that state had)
        from_first = True
        # (...and the findings of the runs before stand with this one's)
        for nm, fb0 in sorted((R_.get('feedback') or {}).items()):
            fb1 = feedback.get(nm) or {}
            feedback[nm] = {k_: list(fb0.get(k_) or []) + [it_ for it_ in fb1.get(k_) or [] if it_ not in (fb0.get(k_) or [])]
                            for k_ in ('avoid', 'pairs', 'ban', 'pin', 'named')}
            feedback[nm]['over'] = {**(fb0.get('over') or {}), **(fb1.get('over') or {})}
        for nm, h0 in sorted((R_.get('hard') or {}).items()):
            h1 = hard.get(nm) or {}
            hard[nm] = {k_: list(h0.get(k_) or []) + [it_ for it_ in h1.get(k_) or [] if it_ not in (h0.get(k_) or [])]
                        for k_ in ('avoid', 'pairs')}
        log(f'  from {frm}: {len(wide)} balls on their whole menu, {len(lined)} bus(es) on the lines, the last state '
            f'of {len(prev_bus)} bus(es) and what their checks taught')
        trace_prev = {nm: {l_: tuple((str(e_[0]), int(e_[1])) for e_ in v_) for l_, v_ in pb_.items()}
                      for nm, pb_ in prev_bus.items()}
        TR.rec('from', run=os.path.abspath(frm), feedback={nm: {k_: len(v_ or ()) for k_, v_ in fb_.items()}
                                                           for nm, fb_ in feedback.items()},
               hard={nm: {k_: len(v_ or ()) for k_, v_ in h_.items()} for nm, h_ in hard.items()},
               fed={nm: sorted(v_) for nm, v_ in fed.items()})

    def teach(b):
        L_ = learned.get(b.name)
        if not L_:
            return
        b.E._nochg, b.E._nocross, b.E._nogoods = set(L_['nochg']), set(L_['nocross']), []
        b.E._samehands = set(L_['hands'])
        for ng_ in L_['nogoods']:
            e_ = {}
            for l_, sg_ in ng_.items():
                ij_ = []
                for k_, opts_ in enumerate((b.E.T, b.E.B)):
                    ij_.append(None if sg_[k_] is None else
                               next((i for i, o in enumerate(opts_.get(l_) or ()) if we._opt_sig(o) == sg_[k_]), -1))
                if -1 in ij_:
                    e_ = None           # (an end the lane is no longer offered: that state cannot come again)
                    break
                e_[l_] = tuple(ij_)
            if e_:
                b.E._nogoods.append(e_)
    # ---- THE FIELD LAID AND MEASURED: the arrays fanned out as the state chose, then the board's own check on it
    # (check_drc at the project's floor) against the bare board's. The arrays' rules say no two pieces the model laid
    # clash; this reads the copper itself -- a via on a part the nudge could not move, a berth's on a neighbour's pad
    # -- and A VIOLATION THE FANOUT MADE IS THE MODEL'S TO ANSWER: the bus lane's end at fault is not taken again
    # (learned 'endban': the option it had, by its signature), the lane re-solved, the field laid and measured again
    import whole_route as wr
    measured = {}

    def field_drc(path, sub_):
        jp = os.path.join(out, f'field_drc_{sub_}.json')
        if os.path.exists(jp):
            os.remove(jp)
        wr.run([os.path.join(HERE, '..', 'py_router', 'check_drc.py'), path, '--clearance-margin', '0.1',
                '--max-print', '0', '--json', jp], awx_settings.environ(), log_path=jp[:-len('.json')] + '.log',
               own_process=True)
        return wr.drc_count(jp)[0], jp

    def lay_and_measure(laid, wayouts=()):
        with awx_settings.given(genv):
            nt_, nv_, caps_ = write_field(board, os.path.join(out, 'shared_field.kicad_pcb'), laid, ties_of=ties_of,
                                          tie_sites=tie_sites, pulls=pulls, wayouts=wayouts, locked=sorted(held))
            d_f, jp_f = field_drc(os.path.join(out, 'shared_field.kicad_pcb'), 'field')
            if 'bare' not in measured:
                measured['bare'] = field_drc(board, 'bare')[0]
        d_b = measured['bare']
        bad_ = d_f is None or d_b is None or d_f > d_b
        items = [i_ for i_ in (json.load(open(jp_f)).get('items') or []) if not i_.get('accepted')] if bad_ and d_f else []
        # (...and A PART THE NUDGE COULD NOT MOVE CLEAR OF THE LANES' WAYS OUT: each lane whose way out it is left
        # standing in, a finding as a violation is -- that lane's end at fault, teach_laid. The nudge said so and
        # nothing read it: a memory bus's decoupling capacitor in front of its array's face, the corner it had gone
        # to on other ends taken by a via on these, was left in three lanes' ways out with the field's check clean;
        # the lanes were planned through it, the loop stopped with its ends crowded, and one net shipped open)
        with awx_settings.given(genv):
            items += stuck_in_ways(os.path.join(out, 'shared_field.kicad_pcb'), caps_, wayouts)
        moved = set()               # (the parts the nudge moved from where the board has them)
        if items:
            if 'poses' not in measured:
                measured['poses'] = wr.footprint_poses(board)
            now_ = wr.footprint_poses(os.path.join(out, 'shared_field.kicad_pcb'))
            moved = {r_ for r_, p_ in now_.items() if measured['poses'].get(r_, p_) != p_}
        return dict(nt=nt_, nv=nv_, caps=caps_, d_f=d_f, d_b=d_b, bad=bad_, items=items, moved=moved)

    def teach_laid(items, buses, moved=(), laid_=(), ways_=()):
        """how many lessons the violations `items` (check_drc's) newly give: for each, the bus lanes of its nets --
        but a pad's own net, which stood there before the fanout -- each at its end nearer the violation; but a
        violation at a pad of a part the nudge MOVED (`moved`) holds that part where it came instead, the lanes
        that stand on it there (`laid_`, `ways_`: the field's copper and the ways out, as laid) the repair's"""
        n, new_held = 0, set()
        for it in items:
            sl_ = it.get('seg_loc')
            # (...two tracks' own: where they cross, else where they come nearest -- read without these, two runs
            # shorted on one layer taught nothing, and the model stopped on a field it could have answered)
            P = it.get('via_loc') or ([(sl_[0] + sl_[2]) / 2, (sl_[1] + sl_[3]) / 2] if sl_ else None) \
                or it.get('closest_pt') or it.get('pad_loc') or it.get('cross_point') or it.get('closest_pt1')
            # (...two HOLES' own, a drill within the hole rule of another: between the two -- its item names its two
            # places and no other, and read without them a tooth's via a quarter millimetre off its own lane's berth
            # via, two parts a millimetre apart, taught nothing: the model stopped on a field one ban answers)
            if P is None and it.get('loc1') and it.get('loc2'):
                P = [(it['loc1'][0] + it['loc2'][0]) / 2, (it['loc1'][1] + it['loc2'][1]) / 2]
            if P is None:
                P = it.get('loc1') or it.get('loc2')
            if P is None:
                continue
            static = None
            if str(it.get('type', '')).startswith('pad-') and it.get('pad_ref') and '.' in it['pad_ref']:
                ref_, num_ = it['pad_ref'].rsplit('.', 1)
                f_ = bpcb.footprints.get(ref_)
                static = next((p_.net_name for p_ in f_.pads if p_.pad_number == num_), None) if f_ else None
                # (...a PART THE NUDGE MOVED and could not clear: what moved it is the fanout's copper on its pads
                # where it came -- a via, a track, a lane's way out -- not the copper it was moved against: that
                # lane ruled out, another's took its place and the part was moved onto it again, a lap of the model
                # a net. The part is HELD where it came: no copper and no way out within the clearance of its pads
                # from here on, any lane's -- array_rows -- and the lanes that stand on it now are the repair's to
                # free, `standing`)
                # (held already and moved again: its lanes' ends are what is left to answer, as before)
                if f_ and ref_ in moved and ref_ not in held:
                    held[ref_] = [(p_.global_x, p_.global_y, p_.size_x / 2, p_.size_y / 2, p_.net_name,
                                   tuple(p_.layers)) for p_ in f_.pads]
                    new_held.add(ref_)
                    n += 1
                    import rules as _rules
                    with awx_settings.given(genv):
                        on_it = standing(held[ref_], laid_, ways_, lambda i_: bpcb.nets[i_].name if i_ in bpcb.nets
                                         else None, buses, _rules.active().clearance)
                    for bn_, ls_ in on_it.items():
                        fresh.setdefault(bn_, set()).update(ls_)
                if ref_ in new_held:
                    continue
            for b in buses:
                for nm_ in (it.get('net1'), it.get('net2')):
                    l_ = b.leg_lane.get(nm_)
                    if l_ is None or nm_ == static:
                        continue
                    k_ = min((0, 1), key=lambda k: min(
                        math.hypot(b.F.pt(m_.exit_pt)[0] - P[0], b.F.pt(m_.exit_pt)[1] - P[1])
                        for m_ in (b.E.T, b.E.B)[k][l_][b.state[l_][k]][0]))
                    e_ = (l_, k_, we._opt_sig((b.E.T, b.E.B)[k_][l_][b.state[l_][k_]]))
                    L_ = learned.setdefault(b.name, {'nochg': set(), 'nocross': set(), 'nogoods': [], 'xl': {},
                                                     'hands': set(), 'endlay': []})
                    if e_ not in L_.setdefault('endban', []):
                        L_['endban'].append(e_)
                        fresh.setdefault(b.name, set()).add(l_)
                        n += 1
            # (...and an array's OTHER ball of one of its nets -- the nearest such ball: the way it was served, a
            # fan-out or a drop, not again. A ball's via laid on a part the nudge could not move was the one
            # violation left on a board, and nothing was the model's to change)
            for nm_ in (it.get('net1'), it.get('net2')):
                if nm_ == static or any(nm_ in b.leg_lane for b in buses):
                    continue
                near_ = [(math.hypot(A.bm.balls[k][0] - P[0], A.bm.balls[k][1] - P[1]), ref, k)
                         for ref, A in arrays.items() for k in A.okeys
                         if A.bm.items[k][1].net_name == nm_ and (ref, k) in prev_oth]
                if near_:
                    _d, ref, k = min(near_, key=lambda t: t[0])
                    e_ = (ref, k, prev_oth[(ref, k)][0], prev_oth[(ref, k)][1])
                    if e_ not in oth_ban:
                        oth_ban.add(e_)
                        n += 1
        return n
    def teach_walled(buses, laid_=(), ways_=()):
        """how many lessons THE PADS THE FIELD WALLS IN newly give (field_walls, on the field as laid): a part the nudge
        moved into the cage is HELD where it came, as teach_laid holds one (its lanes standing on it the repair's to
        free); else the sites of the field's vias round the pad that free it are left to no other net's drop -- every
        drop of another net's ball there, and an escape whose via stands there, not again ('laid', as a choice whose
        copper broke the board's rules) -- and those balls take another site, share a via of their net or are strapped.
        A pad neither frees is named, and the field goes on as laid"""
        import source_realize as sr_
        bus_ = sorted({cap['st']['byname'][n_][1].name for cap in caps.values() for n_ in cap['st']['launch']
                       if n_ in cap['st']['byname']})
        with awx_settings.given(genv):
            got = field_walls(bpcb, board, os.path.join(out, 'shared_field.kicad_pcb'), sorted(arrays), bus_,
                              held=set(held))
        n, said = 0, []
        for key_, net_, kind_, what_ in got:
            if kind_ == 'hold':
                held[what_] = [(p_.global_x, p_.global_y, p_.size_x / 2, p_.size_y / 2, p_.net_name, tuple(p_.layers))
                               for p_ in copper_pads(bpcb, what_)]
                import rules as _rules
                with awx_settings.given(genv):
                    on_it = standing(held[what_], laid_, ways_, lambda i_: bpcb.nets[i_].name if i_ in bpcb.nets
                                     else None, buses, _rules.active().clearance)
                for bn_, ls_ in on_it.items():
                    fresh.setdefault(bn_, set()).update(ls_)
                n += 1
                said.append(f'{key_} ({net_}): {what_} held where it came')
                continue
            if kind_ != 'sites':
                said.append(f'{key_} ({net_}): NO WAY KEPT -- {what_}')
                continue
            at_ = lambda c_: any(math.hypot(c_[0] - x_, c_[1] - y_) < 0.01 for x_, y_ in what_)   # noqa: E731
            new_ = set()
            for ref, A in arrays.items():
                for k, ds in A.drops.items():
                    if A.bm.items[k][1].net_name != net_:
                        new_ |= {(ref, k, 'd', j) for j, (d, _pc) in enumerate(ds) if at_(d.site)}
                for k in A.okeys:
                    ch_ = prev_oth.get((ref, k))
                    if ch_ and ch_[0] == 'x' and A.bm.items[k][1].net_name != net_:
                        i_ = next((i for i, mv in enumerate(A.bm.menu[k]) if sr_.move_sig(mv) == ch_[1]), None)
                        if i_ is not None and any(at_(c_) for c_, _r, _dr in A.bm.opts[k][i_][2].vias):
                            new_.add((ref, k, 'x', ch_[1]))
            new_ -= oth_ban
            if not new_:
                said.append(f'{key_} ({net_}): its sites {what_} ruled out already, walled all the same')
                continue
            oth_ban.update(new_)
            n += 1
            said.append(f'{key_} ({net_}): the site(s) ' + ', '.join(f'({x_:.2f}, {y_:.2f})' for x_, y_ in what_)
                        + f' left to no other net\'s drop ({len(new_)} choice(s) ruled out)')
        if got:
            log(f'  the field walls in {len(got)} pad(s) later routes must reach -- ' + '; '.join(said))
        TR.rec('walled', check=chk_try + 1, pads=[list(g_[:3]) for g_ in got], lessons=n)
        return n
    no_plan = []
    # (GROUP_LAYERS=free: a bus ALONE on its arrays -- no other bus of the group at either -- has its via ends' layers
    # chosen by its own solve, among the routing layers at their prices; the model's layers are where it starts)
    # (GROUP_LAYERS=free: a lane's ends' layers are its own bus's solve's to choose -- the model's are where it starts
    # -- unless the lane CROSSES another bus's, or its run under an array lies along another bus's: those two lanes'
    # layers are what keeps them apart, and stay the model's (the plan's `held_layers`). A bus alone on its arrays has
    # none held)
    FREE = awx_settings.get('GROUP_LAYERS') == 'free'
    global CHECK_DUMP
    CHECK_DUMP = awx_settings.get('GROUP_CHECK_DUMP') or None
    # (a check's refusal of a bus's PATTERN ends, which gives it the lines, is none of its JOINT_TRIES: those are its
    # ends' on the lines, as a bus's own ends had them)
    chk_try, spent, n_taught, adopts = -1, 0, 0, 0
    fresh, prev_x = {}, {}      # bus -> the lanes its last check's refusal named; bus -> its last state's crossings
    if frm and any(fed.values()):
        # (A RUN'S FINDINGS ARE ANSWERED BY THE REPAIR, as a check's refusal is: every lane held on its last state's
        # ends but those the findings name and those that crossed one. Started near that state and searched whole
        # again, a one-bus model moved 30 of its 78 other ends to answer 8 findings -- ends its audit had found
        # nothing against -- and took the minutes of a first model)
        fresh = {nm: set(fed.get(nm) or ()) for nm in caps}
        prev_x = {nm: {tuple(pr_) for pr_ in x_} for nm, x_ in (R_.get('prev_x') or {}).items()}
    elif frm and at_point:
        # (a bench's: every lane held on the state's ends -- then its checks, --at-search, or its sweeps, --at-start;
        # a rung's with no findings to answer, --at-end: its checks)
        fresh = {nm: set() for nm in caps}
    laid_try, M_ = 0, None      # the field's violations answered so far; the last laying's measure
    no_room = {}                # bus -> its lanes with no room for a change here and room on a rung below
    tight = {}                  # bus -> the lanes a refusal of this run named so: named again, the bus has no room
    # (THE EPOCHS of the search -- TIER_MENU: `stretches` the lanes in stretches round the source, `epoch` the one on
    # its whole menus now, `in_epoch` a model built for one)
    stretches, epoch, in_epoch, sweep, sweep_v0, epoch_balls = None, -1, False, 0, None, set()
    epoch_fan = []              # (the costly balls of the state a stretch starts from: its rounds' reach, stretch_reach)
    kept_last = None            # (each bus ball's move in the search's last state, by its tier signature)
    more_last = 0               # (two routing layers) what the model does not count of the search's last state
    costly_tries = collections.Counter()    # (a costly ball (array, ball) -> the searches after a repair it has had)
    # (the last state's price below its costly balls -- a run from another's end: that run's last -- a repair's is
    # held against it: A DEAR REPAIR, below)
    v_last = R_.get('below') if frm else None
    mouth_said = None          # (the mouths offered to more lanes than they hold: said once a count)
    walk_said = None            # (the places on a group's walk offered to more lanes than they hold: the same)
    whole_q = False             # (a repair had no state: the whole model's question asked, the model built again)
    again, small_r = False, True    # (the same check's model built again; the repair's model built small, first)
    while True:
        if not again:
            small_r = True
        chk_try += 0 if in_epoch or again else 1
        again = False
        rnd, deferred = 0, False
        while True:
            rnd += 1
            buses, more = [], set()
            for nm, cap in caps.items():
                st = cap['st']
                # (its balls' menus as they stand: a ball's version -- widened, or back on its first tiers -- each)
                mine = frozenset((ref, arrays[ref].ball_of[n], mver[(ref, arrays[ref].ball_of[n])])
                                 for ref in (sref, st['dref']) if ref in arrays
                                 for n in st['launch'] if (ref, arrays[ref].ball_of.get(n)) in wide)
                if (nm, mine, nm in lined) not in built:
                    for key_ in [k_ for k_ in built if k_[0] == nm and k_[2] == (nm in lined)]:
                        del built[key_]         # (a menu since changed: not built on again)
                    built[(nm, mine, nm in lined)] = load_bus(
                        nm, cap, arrays, log, cut=cuts.get((nm, nm in lined)),
                        rank=nm in lined, feedback=feedback.get(nm),
                        bare_ids={n_.name: i_ for i_, n_ in bpcb.nets.items() if n_.name}, board_cu=board_cu)
                b = built[(nm, mine, nm in lined)]
                teach(b)
                buses.append(b)
                # (a lane with no clean option among the moves offered: its balls' whole menus)
                # (...where the end is an array's: a row part's or a group's berths are its own run's menu, with
                # none wider to offer -- such a lane is named below)
                more |= {(b.arr[k_].ref, b.arr[k_].ball_of[leg]) for l_, k_ in b.bare if b.arr[k_] is not None
                         for leg in (b.E.legs_of.get(l_) or (l_,))} - (wide - narrowed)
            if more:
                log(f'  round {rnd}: no option in the pattern for {sorted(f"{b.name}:{l_}" for b in buses for l_, _k in b.bare)}'
                    f' -- {len(more)} balls offered their whole menu')
                wide |= more
                release(buses)
                m = mc = rows = gates = None
                whole(more)
                continue
            stuck = sorted(f'{b.name}:{l_}' for b in buses for l_, _k in b.bare)
            if stuck:
                raise SystemExit(f'group_ends: no option in the whole menu for {stuck}')
            # ---- ONE model: each bus's own, then each array's rules over every lane there alike. Every rule is HARD --
            # nothing is priced in place of a rule that cannot hold. Each stands behind an assumption for one purpose: where
            # no state holds them all, the rules that cannot hold together are NAMED
            m = cp_model.CpModel()
            room_rows, endlay_rows = [], []
            # (A STRETCH'S MODEL, built small: a round of its search frees lanes of the stretch, or one of them and its
            # nearest -- stretch_reach -- and every other lane keeps its two ends through the stretch whatever the
            # rounds draw. Those lanes are HELD: offered the one option they have at each end, so no row or term of
            # their other options is built. Built with every lane on its whole first tiers, a four-bus stretch's model
            # was 347k variables and 1.36M rows, and each of its rounds -- eight lanes freed -- paid the solver's
            # reading of all of it. On more routing layers than two: on two, a round also frees a lane with whatever
            # it crosses, any lane of its bus)
            reach, small_now = None, False
            if in_epoch and stretches and 0 <= epoch < len(stretches) and stretches[epoch] \
                    and len(buses[0].E.RL) > 2 and prev_bus:
                reach = stretch_reach(buses, stretches[epoch], [arrays[r_].bm.balls[k] for r_, k in epoch_fan])
            elif fresh and small_r and not in_epoch and not whole_q and all(prev_bus.get(nm_) for nm_ in caps):
                # (A REPAIR'S MODEL, built small as a stretch's: the repair holds every lane on its ends but those a
                # refusal named and those that crossed one (repair_holds) -- so the held ones are offered the one
                # option they have at each end, the same state asked of a model without the rows and terms of the
                # options they cannot take. Built whole, a three-bus group's model after a refusal was 191 to 283
                # thousand variables, 22 to 31 s in the making, and its repairs 150 to 362 s each -- eleven of them
                # 1420 s of a run's 5229. A held lane the lessons leave no state must be LET GO, and here it has no
                # other option to go to: with no state this way the model is built whole, and asked as before)
                reach = set()
                for nm_ in caps:
                    fr_ = set(fresh.get(nm_) or ())
                    fr_ |= {l2_ for pr_ in prev_x.get(nm_, ()) if fr_ & set(pr_) for l2_ in pr_}
                    reach |= {(nm_, str(l_)) for l_ in fr_}
                small_now = True
            n_only = 0
            for b in buses:
                only_ = None
                if reach is not None and prev_bus.get(b.name):
                    only_ = {}
                    for l_, _lg in b.E.lanes:
                        P_ = prev_bus[b.name].get(l_)
                        if (b.name, str(l_)) in reach or not P_:
                            continue
                        ij_ = [next((i for i, o in enumerate(opts_[l_]) if not o[4] and we._opt_sig(o) == P_[k_][0]),
                                    None) for k_, opts_ in enumerate((b.E.T, b.E.B))]
                        if None not in ij_:
                            only_[l_] = tuple(ij_)
                    n_only += len(only_)
                with awx_settings.given(b.env):
                    b.J = we._joint_model(b.E, b.own, (learned.get(b.name) or {}).get('xl'), lambda *a: None, b.boxes,
                                          (), False, m=m, only=only_)
                    if isinstance(b.J, tuple):
                        raise SystemExit(f'group_ends: {b.name}: {b.J[1]}')
                # (what the checks taught of a lane's LENGTH: no change of layer on ends that leave it no longer than
                # the longest lane of its kind the solve found no room in -- behind an assumption, as every rule)
                rm_ = (learned.get(b.name) or {}).get('room') or {}
                n_rm = 0
                for l_ in (b.J.lanes if rm_ else ()):
                    for i, xt_ in b.J.xs[0][l_].items():
                        js_ = []
                        for j in b.J.xs[1][l_]:
                            kd_, ln_ = lane_run(b, l_, i, j)
                            if kd_ in rm_ and ln_ <= rm_[kd_] + 1e-6:
                                js_.append(j)
                        if js_:
                            room_rows.append((b, l_, xt_ + sum(b.J.xs[1][l_][j] for j in js_) + b.J.chg[l_] <= 2))
                            n_rm += 1
                # (what a check taught of an END on its layer: not that end on that layer again -- the whole solve
                # named the lane for it, and a state refused by its two ends together came back with the one moved
                # that was not at fault, three checks running)
                for l_, k_, sg_, q_ in (learned.get(b.name) or {}).get('endlay', ()):
                    opts_ = (b.E.T, b.E.B)[k_]
                    for i, v in (b.J.xs[k_].get(l_) or {}).items():
                        if we._opt_sig(opts_[l_][i]) == sg_:
                            endlay_rows.append((b, l_, k_, [v.Not(), b.J.y[k_][l_][q_].Not()]))
            if small_now:
                log(f'  the repair\'s model: {len(reach)} lanes a refusal named or that crossed one, the other {n_only} '
                    f'each held on its two ends')
            elif reach is not None and rnd == 1:
                log(f'  the stretch\'s model: {len(reach)} lanes within its rounds\' reach, the other {n_only} each '
                    f'held on its two ends')
            by_name = {b.name: b for b in buses}
            RL = buses[0].E.RL
            NL = len(RL)
            # (a run on a plane's layer at its price, as the exact route prices it -- route_layers.run_price)
            with awx_settings.given(genv):
                pk = [int(round(1000 * route_layers.run_price(buses[0].E.PK[q], buses[0].E.inner_signal))) for q in range(NL)]
            gates, gate_lanes = {}, collections.defaultdict(set)

            def gate(*name, lanes=()):
                if name not in gates:
                    gates[name] = m.NewBoolVar('')
                gate_lanes[name].update(lanes)
                return gates[name]
            for b_, l_, row_ in room_rows:
                m.Add(row_).OnlyEnforceIf(gate('room', b_.name, l_, lanes={(b_.name, l_, 0), (b_.name, l_, 1)}))
            # (...each behind an assumption, as every rule: with none, the model built after a check's lessons had no
            # state and no rule to name for it)
            for b_, l_, k_, cl_ in endlay_rows:
                m.AddBoolOr(cl_).OnlyEnforceIf(gate('endlay', b_.name, l_, k_, lanes={(b_.name, l_, k_)}))
            # (an end whose copper, laid, broke the board's rules -- teach_laid: not that option again)
            for b in buses:
                for l_, k_, sg_ in (learned.get(b.name) or {}).get('endban', ()):
                    for i, v in (b.J.xs[k_].get(l_) or {}).items():
                        if we._opt_sig((b.E.T, b.E.B)[k_][l_][i]) == sg_:
                            m.Add(v == 0).OnlyEnforceIf(gate('laid', b.name, l_, k_, lanes={(b.name, l_, k_)}))
            if room_rows and rnd == 1:
                log(f'  no change of layer on ends that leave a lane as short as the solve found too short: '
                    f'{len(room_rows)} tooth options of {len({(b_.name, l_) for b_, l_, _r in room_rows})} lanes')
            # WHAT A BUS'S STEP FOUND at its ends (--feedback), ruled out: an end at the very place it was found crowded
            # (the ends model's own table of the findings, E.fb_avoid: a weight of 1 or more is the place itself, on
            # the layer named unless the route chooses that end's layer), and two ends found crowded together
            # (E.fb_pairs) not both -- each behind its assumption, so that where no state holds them the lanes are
            # named and their balls widened, as for every rule. Nearer places keep the ends model's price
            for b in buses:
                if b.name not in feedback:
                    continue
                n_av = n_pr = n_oth = 0
                ruled_ = collections.defaultdict(list)      # (lane, end) -> the options ruled out (a trace's)
                hav_, hpr_ = we.fb_tables(hard.get(b.name) or {}, b.E.T, b.E.B)
                for (l_, k_), ws_ in sorted(hav_.items()):
                    at_ = [i for i, w_ in sorted(ws_.items()) if w_ >= 1.0 - 1e-9 and i in b.J.xs[k_].get(l_, {})]
                    # (...and the very option the step was laid on there, whatever the tables weigh it: the finding
                    # is of that end as it was laid. A row part's berth on a stub and a via, its lane on an inner layer,
                    # weighed half -- "another layer at the place" -- and seven of eleven such ends were not ruled out.
                    # The state the step was LAID on, the run before's (`found_at`), for the lanes this run's findings
                    # name -- not the state this run has now: read off the last state, the end the model had just
                    # given a named lane in answer was ruled out in the model built after the next check, the lane
                    # moved again and seven of its bus's lanes with it, and a bus whose check had just found a plan
                    # was left on ends with none)
                    sg0_ = ((found_at.get(b.name) or {}).get(l_) or ((None,), (None,)))[k_][0] \
                        if l_ in fed.get(b.name, ()) else None
                    at_ = sorted(set(at_) | {i for i in b.J.xs[k_].get(l_, {})
                                             if sg0_ is not None and we._opt_sig((b.E.T, b.E.B)[k_][l_][i]) == sg0_})
                    if at_ and len(at_) >= len(b.J.xs[k_][l_]):
                        # (an end with NO OTHER PLACE -- a row part's pad has one escape: its lane is answered at its
                        # OTHER end instead, the option it had there not taken again. Ruled out where it stood, the
                        # one rule had no state: a bus's eleven open nets, one of them on such a pad, and no model)
                        sg_ = ((found_at.get(b.name) or {}).get(l_) or ((None,), (None,)))[1 - k_][0]
                        at_, k_ = [i for i in b.J.xs[1 - k_][l_] if len(b.J.xs[1 - k_][l_]) > 1
                                   and we._opt_sig((b.E.T, b.E.B)[1 - k_][l_][i]) == sg_], 1 - k_
                        n_oth += bool(at_)
                    for i in at_:
                        m.Add(b.J.xs[k_][l_][i] == 0).OnlyEnforceIf(
                            gate('found', b.name, l_, k_, lanes={(b.name, l_, k_)}))
                        n_av += 1
                        o_ = (b.E.T, b.E.B)[k_][l_][i]
                        ruled_[(l_, k_)].append({'opt': i, 'xy': [list(m_.exit_pt) for m_ in o_[0]],
                                                 'dir': o_[0][0].direction if o_[0] else None, 'layer': o_[2],
                                                 'via': we._via_end(o_)})
                for a_, b2_, k_, ia_, ib_ in hpr_:
                    for i, wa_ in sorted(ia_.items()):
                        for j, wb_ in sorted(ib_.items()):
                            if wa_ * wb_ >= 1.0 - 1e-9 and i in b.J.xs[k_].get(a_, {}) and j in b.J.xs[k_].get(b2_, {}):
                                m.AddBoolOr([b.J.xs[k_][a_][i].Not(), b.J.xs[k_][b2_][j].Not()]).OnlyEnforceIf(
                                    gate('found', b.name, f'{a_}+{b2_}', k_, lanes={(b.name, a_, k_), (b.name, b2_, k_)}))
                                n_pr += 1
                if rnd == 1:
                    log(f'  {b.name}: its step\'s findings ruled out: {n_av} option(s) at an end found crowded, {n_pr} '
                        f'pair(s) of options at two ends found crowded together'
                        + (f'; {n_oth} lane(s) with no other place at the end named, answered at their other end'
                           if n_oth else ''))
                    # (...and what the rest of them PRICE, in the cost -- below one change of layer: a tie-break)
                    log(f'  {b.name}: its step\'s findings priced (in the cost, below a change of layer): '
                        f'{sum(1 for ws_ in b.E.fb_avoid.values() for w_ in ws_.values() if w_ > 0)} option(s) at '
                        f'{len(b.E.fb_avoid)} end(s), {len(b.E.fb_pairs)} pair(s) of ends')
                    TR.rec('feedback', check=chk_try + 1, bus=b.name,
                           ruled_out={f'{l_}:{("tooth", "berth")[k_]}': v_ for (l_, k_), v_ in sorted(ruled_.items())},
                           ruled_pairs=n_pr, answered_at_other_end=n_oth,
                           priced={f'{l_}:{("tooth", "berth")[k_]}': {'options': sum(1 for w_ in ws_.values() if w_ > 0),
                                                                     'max_w': max(ws_.values(), default=0)}
                                   for (l_, k_), ws_ in sorted(b.E.fb_avoid.items())},
                           priced_pairs=[[a_, b2_, k_] for a_, b2_, k_, _ia, _ib in b.E.fb_pairs])
            t0 = time.time()
            rows, n_reg = {}, 0
            between, between_cost, between_const = {}, [], 0    # (the crossings between buses, an array several leave)
            for ref, A in arrays.items():
                users = [(b, k_) for b in buses for k_ in (0, 1) if b.arr[k_] is A]
                with awx_settings.given(genv):
                    rows[ref] = array_rows(m, A, users, gate, RL, pk, off, log, wide=wide,
                                           dropped='--dropped' in flags, oth_ban=sorted(oth_ban, key=repr),
                                           held=[p_ for r_ in sorted(held) for p_ in held[r_]])
                if len(users) > 1 and 'between' not in off:
                    bx_, bc_, bk_ = between_rows(m, A, users, gate, RL)
                    between[ref], between_cost, between_const = bx_, between_cost + bc_, between_const + bk_
                if len(users) > 1 and 'region' not in off:
                    n_reg += region_rows(m, A, users, gate, RL,
                                         int(round(2 * ((A.box[2] - A.box[0]) + (A.box[3] - A.box[1])) * 1000)) + 4000)
            # LANES OF ONE BUS HEADED FOR DIFFERENT PARTS (a bus to a group of parts) ARE TO EACH OTHER AS LANES OF TWO
            # BUSES AT THEIR SOURCE: where two of them cross they leave it on different layers. They part where the
            # trunk ends, so they cross before it does -- within a few mm of their exits where the parts stand close
            # by, before a via fits -- and the bus's own rule (one of the two changes layer) asks for a change there
            # is no place for: two lanes from surface ends 3.6 mm apart on one face, one to the part above and one to
            # the part below across a 3 mm channel, were refused by the solve at two checks running, their teeth's
            # layer and their crossing's the whole of its core
            n_px = 0
            if 'parts' not in off:
                for b in buses:
                    dp_ = b.E.st.get('dst_pad') or {}
                    part_ = {l_: {getattr(dp_.get(leg), 'component_ref', None) for leg in (b.E.legs_of.get(l_) or (l_,))}
                             for l_ in b.J.lanes}
                    if len(set().union(*part_.values()) - {None}) < 2:
                        continue
                    for (a_, c_), xv_ in b.J.X.items():
                        if (isinstance(xv_, int) and not xv_) or part_[a_] & part_[c_]:
                            continue
                        gl_ = gate('parts', b.name, a_, c_, lanes={(b.name, a_, 0), (b.name, c_, 0)})
                        for q in range(NL):
                            m.AddBoolOr([b.J.y[0][a_][q].Not(), b.J.y[0][c_][q].Not()]
                                        + ([] if isinstance(xv_, int) else [xv_.Not()])).OnlyEnforceIf(gl_)
                        n_px += 1
                if n_px and rnd == 1:
                    log(f'  lanes of one bus headed for different parts leave on different layers where they cross: '
                        f'{n_px} pair(s) of lanes that may')
            # (a bus lane with no option left at an array among the moves offered -- each blocked in front, or its own
            # legs' in each other's way: its balls' whole menus, and the model built again)
            stuck = [(ref, key) for ref, r_ in rows.items() for key in r_.stuck]
            if stuck:
                more = {(ref, arrays[ref].ball_of[leg]) for ref, (owner, lane, _k) in stuck
                        for leg in by_name[owner].E.legs_of[lane]} - (wide - narrowed)
                names_ = ', '.join(f'{owner}:{lane} at {ref}' for ref, (owner, lane, _k) in stuck)
                if not more:
                    raise SystemExit(f'group_ends: no option in the whole menu for {names_}')
                log(f'  round {rnd}: no option left in the pattern for {names_} -- {len(more)} balls offered their whole menu')
                wide |= more
                release(buses)
                m = mc = rows = gates = None
                whole(more)
                continue
            # (a bus lane's run on a plane's layer: its run out of its tooth, and its run into its berth where it changes
            # layer between them)
            plane = [t for r_ in rows.values() for t in r_.plane]
            # ...and beside them a tooth on the source's FAR FACE, at the ends model's own price (whole_ends.FAR_TOOTH: past
            # any count of vias, yet no refusal -- for a lane with no other way out): in the lower rank it was taken to save
            # one change of layer, eighteen of one bus's teeth where its own run alone had two, its lanes round the array
            # through the other buses' fanout
            far_w = we.FAR_TOOTH
            far = []
            for b in buses:
                if b.arr[0] is None:
                    continue
                for l_ in b.J.lanes:
                    far += [(int(round(1000 * far_w)) * b.J.npair[l_], x_) for i, x_ in b.J.xs[0][l_].items()
                            if b.E.T[l_][i][0] and b.E.T[l_][i][0][0].direction == 'left']
            # ...and a PAIR's berth on a ball array destination's FAR FACE at the same price: a pair laid round its
            # destination to the face turned from its source passes the faces between on its layer, in front of their
            # berths, and turns two corners as a pair -- a memory bus's clock pair taken round to its DDR's far face
            # was short of its pitch beside every berth of the face it passed, the whole route's loop stopped with its
            # ends crowded and 19 nets open. A single lane goes round as before (two did, the same bus routed)
            for b in buses:
                if b.arr[1] is None:
                    continue
                for l_ in b.J.lanes:
                    if b.J.npair[l_] < 2:
                        continue
                    far += [(int(round(1000 * far_w)) * b.J.npair[l_], x_) for j, x_ in b.J.xs[1][l_].items()
                            if b.E.B[l_][j][0] and b.E.B[l_][j][0][0].direction == 'right']
            plane += [t_ for t_ in far if t_[0]]
            # A POCKET'S MOUTH HOLDS ITS LANES A LANE PITCH APART, a rule behind its own assumption. A group's lanes that
            # enter a pocket between two of its parts by a mouth land across that mouth in their order, a lane pitch
            # apart from the wall of their own part (whole_frame's Group.landings) -- every lane of the mouth, whatever
            # its layer -- and a mouth too short for them is shared evenly, its lanes nearer than a pitch. Unheld, seven
            # lanes were sent through a mouth 1.2 mm wide that lands two: those on one layer were laid on top of each
            # other at their berths in every round, and the loop stopped with its ends crowded. A lane past a mouth's
            # count goes in by another way: the pocket's other mouth, or straight under a part on a layer it leaves open
            n_mouth = 0
            for b in buses:
                if getattr(b.E, 'group', None) is None:
                    continue
                with awx_settings.given(b.env):
                    import braid as bd
                    room_ = bd.LPITCH
                by_m_ = collections.defaultdict(list)
                for l_ in b.J.lanes:
                    for j, x_ in b.J.xs[1][l_].items():
                        w_ = b.E.B[l_][j][5]
                        if w_[0] == 'mouth':
                            by_m_[tuple(round(v_, 4) for p_ in w_[2] for v_ in p_)].append((l_, x_))
                for mk_, mem_ in sorted(by_m_.items()):
                    # (as the landings read it: k lanes stand a pitch apart where k + 1 pitches lie within the mouth)
                    cap_ = int(math.floor(math.hypot(mk_[2] - mk_[0], mk_[3] - mk_[1]) / room_ + 1e-9)) - 1
                    lanes_ = sorted({l_ for l_, _x in mem_})
                    if sum(b.J.npair[l_] for l_ in lanes_) <= cap_:
                        continue
                    m.Add(sum(b.J.npair[l_] * x_ for l_, x_ in mem_) <= max(0, cap_)).OnlyEnforceIf(
                        gate('mouth', b.name, mk_, lanes={(b.name, l_, 1) for l_ in lanes_}))
                    n_mouth += 1
            if n_mouth and n_mouth != mouth_said:
                mouth_said = n_mouth
                log(f'  a pocket\'s mouth holds its lanes a lane pitch apart: {n_mouth} mouth(s) offered to more lanes '
                    f'than they hold')
            # NO FAR-FACE TOOTH for a lane still on the pattern, a rule behind its own assumption: where no state holds it
            # the lane is named with the rules it meets (the ball behind it on their one via site, say), their balls get
            # their whole menus, and a lane on its whole menu may take the far face, at its price
            for b in buses:
                if b.arr[0] is None:
                    continue
                for l_ in b.J.lanes:
                    fx = [x_ for i, x_ in b.J.xs[0][l_].items()
                          if b.E.T[l_][i][0] and b.E.T[l_][i][0][0].direction == 'left']
                    if fx and not {(b.arr[0].ref, b.arr[0].ball_of[leg])
                                   for leg in (b.E.legs_of.get(l_) or (l_,))} <= wide:
                        m.Add(sum(fx) == 0).OnlyEnforceIf(gate('far', b.name, l_, lanes={(b.name, l_, 0)}))
            # THE WAY ROUND an array, with the vias (the ends model's ride, at its one exchange rate -- select_moves.VIA_MM
            # of track a via): a lane that leaves an array by a face turned from its other end, or arrives by one, travels
            # round the array to the face toward it, a leg each
            import select_moves as sm_
            ride = []
            for b in buses:
                for k_ in (0, 1):
                    if b.arr[k_] is None:
                        continue
                    for l_ in b.J.lanes:
                        for i, x_ in b.J.xs[k_][l_].items():
                            c_ = int(round(b.J.npair[l_] * way_round(b.E, k_, b.J.pos[(k_, l_, i)], (b.J.SB, b.J.DB), b.J.cut)
                                           / sm_.VIA_MM))                   # (thousandths of a via)
                            if c_:
                                ride.append((c_, x_))
            # ...and THE LENGTH INSIDE AN ARRAY, each leg's stub from its ball to its exit along its legs (the ends
            # model's own measure, Ends._stub_len, at the same rate): unpriced, an exit that ran millimetres between
            # the balls to leave by another face cost what the straight way out of the ball's own row did
            n_stub = 0
            for b in buses:
                for k_ in (0, 1):
                    if b.arr[k_] is None:
                        continue
                    pad_ = b.E.st[('src_pad', 'dst_pad')[k_]]
                    for l_, lg in b.E.lanes:
                        for i, x_ in b.J.xs[k_][l_].items():
                            mm_ = sum(b.E._stub_len(mv, pad_[leg]) for leg, mv in zip(lg, (b.E.T, b.E.B)[k_][l_][i][0])
                                      if leg in pad_)
                            c_ = int(round(1000 * mm_ / sm_.VIA_MM))
                            if c_:
                                ride.append((c_, x_))
                                n_stub += 1
            # ...and THE WAY INTO A GROUP OF ROW PARTS, its length at the same rate (group_walk: an 'on' berth off the
            # group's front its walk round the outline, any other way its own length) -- and the lanes walking past
            # each place held to the ROOM beside the outline there, to the board's edge, a lane pitch apart. Unpriced
            # and unheld, a connector's far-row berth by a surface stub on its own layer costs nothing beside a via at
            # the pin, and lanes go round its end along a strip at the board's edge that holds fewer of them
            n_walk = 0
            for b in buses:
                if b.arr[1] is not None or getattr(b.E, 'group', None) is None:
                    continue
                with awx_settings.given(b.env):
                    import braid as bd
                    cfg_ = bd.run_config(b.E.st['pcb'], b.E.st.get('base'))
                    pitch_ = bd.LPITCH
                g2_ = cfg_.grid_step / 2
                edge_ = (float(getattr(cfg_, 'board_edge_clearance', 0.0) or 0.0) or cfg_.clearance) + \
                    cfg_.track_width / 2 + g2_
                gl_, way_mm_, rooms_ = group_walk(b.E, edge_, cfg_.track_width / 2 + cfg_.clearance + g2_, pitch_)
                on_ = collections.defaultdict(list)     # u -> [(npair, literal, lane)] of the options walking past it
                for l_ in b.J.lanes:
                    for j, x_ in b.J.xs[1][l_].items():
                        o_ = b.E.B[l_][j]
                        c_ = int(round(b.J.npair[l_] * 1000 * way_mm_(o_) / sm_.VIA_MM))
                        if c_:
                            ride.append((c_, x_))
                        st_ = walk_lanes(b.E, o_, gl_)
                        if st_ is not None:
                            for u_, _k in rooms_:
                                if st_[0] - 1e-9 <= u_ <= st_[1] + 1e-9:
                                    on_[u_].append((b.J.npair[l_], x_, l_))
                seen_ = set()
                for u_, k_ in rooms_:
                    mem_ = on_.get(u_) or []
                    if sum(n_ for n_, _x, _l in mem_) <= k_:
                        continue
                    key_ = (k_, frozenset(id(x_) for _n, x_, _l in mem_))
                    if key_ in seen_:
                        continue
                    seen_.add(key_)
                    m.Add(sum(n_ * x_ for n_, x_, _l in mem_) <= k_).OnlyEnforceIf(
                        gate('walk', b.name, round(u_, 2), lanes={(b.name, l_, 1) for _n, _x, l_ in mem_}))
                    n_walk += 1
            if n_walk and n_walk != walk_said:
                walk_said = n_walk
                log(f'  a group\'s walk holds the lanes the room beside it does: {n_walk} place(s) offered to more '
                    f'lanes than they hold')
            for b in buses:
                for l_ in b.J.lanes:
                    for q in range(NL):
                        if pk[q]:
                            z_ = m.NewBoolVar('')
                            m.AddBoolOr([b.J.y[1][l_][q].Not(), b.J.chg[l_].Not(), z_])
                            plane += [(b.J.npair[l_] * pk[q], b.J.y[0][l_][q]), (b.J.npair[l_] * pk[q], z_)]
            # NO BUS END ON A GROUND PLANE'S LAYER, where the board has an inner layer that carries no plane to run on
            # instead (route_layers.inner_signal): a run there cuts the plane under its neighbours, and the search left
            # lanes on one where a state without any stood at no more changes
            for b in buses:
                for l_ in b.J.lanes:
                    for k_ in (0, 1):
                        for q in range(NL):
                            if b.E.PK[q] == 'gnd' and b.E.inner_signal:
                                m.Add(b.J.y[k_][l_][q] == 0).OnlyEnforceIf(
                                    gate('ground', b.name, l_, k_, lanes={(b.name, l_, k_)}))
            log(f'  round {rnd}: the model {len(m.Proto().variables)} variables, {len(m.Proto().constraints)} rows, '
                f'{n_reg} regions\' rules ({time.time() - t0:.0f} s)')
            # ---- a state at all? Every rule assumed: with none, the rules that cannot hold together are named
            import source_realize as sr_
            # (THE ENDS WANTED, a bus: its last state's -- each end by its signature on the menu as it now stands;
            # descended again (whole_ends.joint_descent, as its own ends' search) where a check's core priced a crossing
            # or its menus are of a new tier -- else its start's (load_bus: its own run's ends descended at its cut).
            # The state looked for, and the start, near them)
            wanted, held_idx, lost_ = {}, {}, []
            probe = None
            for b in buses:
                P_ = prev_bus.get(b.name)
                s_ = dict(b.own)
                if P_:
                    for l_ in b.J.lanes:
                        ij_ = []
                        for k_, opts_ in enumerate((b.E.T, b.E.B)):
                            sg_, _q, geo_ = P_[l_][k_] if l_ in P_ else (None, None, ())
                            i = next((i for i in b.J.xs[k_][l_] if we._opt_sig(opts_[l_][i]) == sg_), None)
                            if i is None and sg_ is not None and in_epoch:
                                lost_.append(f'{b.name}:{l_}:{("tooth", "berth")[k_]}')
                            if i is None and geo_:
                                # (the end is not on the menu as it now stands -- its ball since offered its whole
                                # menu, other moves: the nearest of them, as a bus's own ends are read onto the group's)
                                i = min((i for i in b.J.xs[k_][l_] if len(opts_[l_][i][0]) == len(geo_)), default=None,
                                        key=lambda i: (sum(m_.direction != d_ for m_, (d_, _p) in zip(opts_[l_][i][0], geo_)),
                                                       sum(math.hypot(m_.exit_pt[0] - p_[0], m_.exit_pt[1] - p_[1])
                                                           for m_, (_d, p_) in zip(opts_[l_][i][0], geo_))))
                            ij_.append(i)
                        s_[l_] = tuple(b.own[l_][k_] if i is None else i for k_, i in enumerate(ij_))
                    held_idx[b.name] = dict(s_)        # (the last state's own ends on this menu: the repair's)
                    xl_ = (learned.get(b.name) or {}).get('xl')
                    # (...never for AN EPOCH's model: that one starts from the state before AS IT STOOD -- each end
                    # held where it was, below. Descended again there -- a bus a check's core had priced a crossing
                    # of -- the ends wanted were no longer the state's: every model of a costly ball's stretch after
                    # a repair said "the state before breaks", spent 86 to 125 s on the state nearest ends the state
                    # never had, 44 to 56 of some 220 moved, and where that had none in its work the run stopped
                    # with no state. Nor for A REPAIR's: it holds every lane but a few on the state refused, and
                    # starts from that state -- the ends descended again are another bus-wide arrangement, which the
                    # held lanes contradict end by end)
                    if (xl_ or (b.name, b.name in lined) not in searched) and not from_first and not in_epoch \
                            and not (fresh and not whole_q):
                        with awx_settings.given(b.env):
                            s_ = we.joint_descent(b.E, s_, b.boxes, b.E.wcut, xl_, dear=b.dear)[0]
                wanted[b.name] = s_
                for l_ in b.J.lanes:
                    for k_ in (0, 1):
                        for i, v in b.J.xs[k_][l_].items():
                            m.AddHint(v, int(i == s_[l_][k_]))
                        q_ = (P_ or {}).get(l_, ((None, None, ()), (None, None, ())))[k_][1]
                        if l_ in layers_want.get(b.name, ()):
                            q_ = layers_want[b.name][l_][k_]
                        if q_ is not None:
                            for q in range(NL):
                                m.AddHint(b.J.y[k_][l_][q], int(q == q_))
            from_first = False
            for ref, r_ in rows.items():
                for k, (xo_, yo_, dv) in r_.others.items():
                    # (a plane's ball starts DROPPED, where no state before says otherwise)
                    h_ = prev_oth.get((ref, k)) or (('d', 0) if dv else None)
                    if not h_:
                        continue
                    if h_[0] == 'x':
                        i_ = next((i for i in xo_ if sr_.move_sig(arrays[ref].bm.menu[k][i]) == h_[1]), None)
                        if i_ is None:
                            continue
                        for i, v in xo_.items():
                            m.AddHint(v, int(i == i_))
                        for q in range(NL):
                            m.AddHint(yo_[q], int(q == h_[2]))
                        for v in dv:
                            m.AddHint(v, 0)
                    elif h_[1] < len(dv):
                        for i, v in xo_.items():
                            m.AddHint(v, 0)
                        for q in range(NL):
                            m.AddHint(yo_[q], 0)
                        for j, v in enumerate(dv):
                            m.AddHint(v, int(j == h_[1]))
            t0 = time.time()
            if lost_:
                log(f'  round {rnd}: {len(lost_)} end(s) of the state before not on the menus as built again (read '
                    f'onto the nearest): ' + ', '.join(lost_[:12]))
            if in_epoch:
                # (AN EPOCH of the search: the state before stands on the menus as they now are -- every lane keeps
                # the move it took -- so it is not looked for: each end and ball held where it was, the rest of it
                # completed. Should it not stand, it is looked for as ever)
                mf_ = m.Clone()
                pf_ = mf_.Proto()
                for v_, x_ in zip(list(pf_.solution_hint.vars), list(pf_.solution_hint.values)):
                    d_ = pf_.variables[v_].domain
                    d_.clear()
                    d_.extend([x_, x_])
                for g_ in gates.values():
                    mf_.Add(mf_.GetBoolVarFromProtoIndex(g_.Index()) == 1)
                mf_.ClearObjective()
                sc = cp_seed.solver()
                sc.parameters.num_workers = 1
                sc.parameters.max_deterministic_time = START_WORK
                qc = sc.Solve(mf_)
                if qc not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    # (...not whole: the state NEAREST it -- the most of its choices where they stood, the lanes'
                    # ends, their layers and every other ball's alike -- and the ends that moved named. Its lanes'
                    # ends alone kept, the other balls came back anywhere: forty plane balls fanned out)
                    # (first, the rules it breaks there: each behind its assumption, the state held)
                    mr_ = m.Clone()
                    pr2_ = mr_.Proto()
                    for v_, x_ in zip(list(pr2_.solution_hint.vars), list(pr2_.solution_hint.values)):
                        d_ = pr2_.variables[v_].domain
                        d_.clear()
                        d_.extend([x_, x_])
                    mr_.ClearObjective()
                    mr_.AddAssumptions([mr_.GetBoolVarFromProtoIndex(g_.Index()) for g_ in gates.values()])
                    sr2_ = cp_seed.solver()
                    sr2_.parameters.num_workers = 1
                    sr2_.parameters.max_deterministic_time = START_WORK
                    if sr2_.Solve(mr_) == cp_model.INFEASIBLE:
                        by_ = {g_.Index(): name for name, g_ in gates.items()}
                        core_ = sorted(by_[i_] for i_ in sr2_.SufficientAssumptionsForInfeasibility() if i_ in by_)
                        log(f'  round {rnd}: the state before breaks, on the menus as they are: '
                            + ('; '.join(' / '.join(str(x) for x in n_)[:160] for n_ in core_[:6])
                               or 'no rule of the arrays (a bus\'s own model)'))
                        if not core_:
                            # (...a bus's own: which of its choices cannot stand together -- each lane's end and
                            # layer held by an assumption of its own, the core named)
                            nm_ = {}
                            for b in buses:
                                for l_ in b.J.lanes:
                                    for k_ in (0, 1):
                                        e_ = ("tooth", "berth")[k_]
                                        for i, v in b.J.xs[k_][l_].items():
                                            nm_[v.Index()] = f'{b.name}:{l_}:{e_} #{i}'
                                        for q, v in enumerate(b.J.y[k_][l_]):
                                            if hasattr(v, 'Index'):
                                                nm_[v.Index()] = f'{b.name}:{l_}:{e_} on {RL[q]}'
                            ma_ = m.Clone()
                            ma_.ClearObjective()
                            for g_ in gates.values():
                                ma_.Add(ma_.GetBoolVarFromProtoIndex(g_.Index()) == 1)
                            hv2_ = dict(zip(ma_.Proto().solution_hint.vars, ma_.Proto().solution_hint.values))
                            lits_ = {}
                            for i_, x_ in sorted(hv2_.items()):
                                if i_ in nm_:
                                    v_ = ma_.GetBoolVarFromProtoIndex(i_)
                                    lits_[i_] = v_ if x_ else v_.Not()
                            ma_.AddAssumptions(list(lits_.values()))
                            sa_ = cp_seed.solver()
                            sa_.parameters.num_workers = 1
                            sa_.parameters.max_deterministic_time = START_WORK
                            if sa_.Solve(ma_) == cp_model.INFEASIBLE:
                                ci_ = set(sa_.SufficientAssumptionsForInfeasibility())
                                bad_ = sorted(f'{nm_[i_]}={hv2_[i_]}' for i_ in lits_ if lits_[i_].Index() in ci_)
                                log(f'  round {rnd}: ...its choices that cannot stand together ({len(bad_)}): '
                                    + ', '.join(bad_[:16]))
                    hv_ = dict(zip(pf_.solution_hint.vars, pf_.solution_hint.values))
                    on_ = [(b, l_, k_, v) for b in buses for k_ in (0, 1) for l_ in b.J.lanes
                           for v in b.J.xs[k_][l_].values() if hv_.get(v.Index()) == 1]
                    mn_ = m.Clone()
                    for g_ in gates.values():
                        mn_.Add(mn_.GetBoolVarFromProtoIndex(g_.Index()) == 1)
                    ones_ = [i_ for i_, x_ in sorted(hv_.items()) if x_ == 1
                             and list(m.Proto().variables[i_].domain) == [0, 1]]
                    mn_.Minimize(sum(1 - mn_.GetBoolVarFromProtoIndex(i_) for i_ in ones_))
                    sc = cp_seed.solver()
                    sc.parameters.num_workers = 1
                    sc.parameters.max_deterministic_time = START_WORK
                    qc = sc.Solve(mn_)
                    if qc in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                        mv_ = [f'{b.name}:{l_}:{("tooth", "berth")[k_]}' for b, l_, k_, v in on_
                               if not sc.Value(mn_.GetBoolVarFromProtoIndex(v.Index()))]
                        log(f'  round {rnd}: the state before does not stand on the menus as they are -- the nearest '
                            f'[{sc.StatusName(qc)}]: {len(mv_)} of {len(on_)} ends moved: ' + ', '.join(mv_[:12]))
                if qc in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    narrow_vals = list(sc.ResponseProto().solution)[:len(m.Proto().variables)]
                    state_known = True
                    break
                log(f'  round {rnd}: the state before does not stand on the menus as they are [{sc.StatusName(qc)}] '
                    f'-- looked for')
            # (AFTER A REFUSAL, THE REPAIR'S OWN QUESTION FIRST: the lanes it will hold, held -- a state there holds every
            # rule, and is the one the repair searches from. Asked after the first tiers' state, that state was looked
            # for at every refusal -- half of the time between two checks of a four-bus group, 93 to 337 s at each of
            # a three-bus group's four -- and then not read: the repair had its own. With none this way, as before)
            if fresh and not in_epoch and not whole_q and all(b.name in held_idx for b in buses):
                mg_ = m.Clone()
                for g_ in gates.values():
                    mg_.Add(mg_.GetBoolVarFromProtoIndex(g_.Index()) == 1)
                got_ = repair_holds(mg_, buses, fresh, prev_x, held_idx, let_go=not small_now)
                if got_[5] in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    probe, sc, qc = got_[:4], got_[4], got_[5]
                    narrow_vals = list(sc.ResponseProto().solution)[:len(m.Proto().variables)]
                    state_known = True
                    break
                if small_now and got_[5] != cp_model.UNKNOWN:
                    # (...the small model has NO state: a held lane must be let go, and the whole model names which)
                    log(f'  round {rnd}: the repair\'s model has no state with the other lanes held '
                        f'[{got_[4].StatusName(got_[5])}] -- built whole')
                    small_r = False
                    release(buses)
                    m = mc = rows = gates = None
                    continue
                # (...NEITHER A STATE NOR A REFUSAL IN ITS WORK: THE REPAIR ITSELF NEXT -- the same question, the
                # objective with it and the search's workers -- and the whole model's question only where the repair
                # has no state either. Asked between the two, the first tiers' and the whole model's question stood
                # unanswered through their work at three refusals of a three-bus group running, 276 to 399 s each,
                # and each time the repair then had its state in 161 to 301 s: the answer waited for was not read)
                if got_[5] == cp_model.UNKNOWN:
                    probe, sc, qc = got_[:4], got_[4], got_[5]
                    narrow_vals, state_known, deferred = None, False, True
                    break
            mc = m.Clone()
            mc.ClearObjective()
            mc.AddAssumptions([mc.GetBoolVarFromProtoIndex(g_.Index()) for g_ in gates.values()])
            if '--model' in flags:      # (a bench's: the model a state is looked for in, its rules' assumptions in it)
                mc.ExportToFile(os.path.join(out, f'state_{chk_try}_{rnd}.pb.txt'))
            # (THE FIRST TIERS FIRST, on tiered menus: the same question with every lane's options past its first
            # TIER_START tiers held off -- a state there holds every rule of the whole model, and starts the search
            # (tier_search's hint). Asked of the whole menus at once, a four-bus model of 2.7 million rows found no
            # state in its work, where the pattern's alone had one in seconds. None there: the whole model asked)
            # A lane's end with no option on its first tiers keeps its whole menu (the start's price, past them only
            # where nothing nearer is), and the ends a refusal's rules name are given theirs, asked again
            narrow_vals, qc = None, cp_model.UNKNOWN
            held_ = {}              # (bus, lane, end) -> its options' variables past its first tiers, held off
            for b in (buses if tier_of else ()):
                for l_ in b.J.lanes:
                    for k_ in (0, 1):
                        ts_ = {i_: option_tier(b, k_, l_, i_, tier_of) for i_, x_ in b.J.xs[k_][l_].items()
                               if hasattr(x_, 'Index')}
                        if ts_ and min(ts_.values()) <= TIER_START:
                            past_ = [b.J.xs[k_][l_][i_].Index() for i_, t_ in sorted(ts_.items()) if t_ > TIER_START]
                            if past_:
                                held_[(b.name, l_, k_)] = past_
            # (each end held by an ASSUMPTION of its own, as each rule is: a refusal names the ends it cannot do
            # without -- an end whose first tiers' options other rules rule out, say -- and those are let go)
            n_held, n_freed, nv_m = len(held_), 0, len(mc.Proto().variables)
            # (the holds added to the model once, each asked by its assumption while it is held -- one end let go is
            # one assumption dropped. A copy of the model a try, a three-array model's footprint rose 0.6 GB in it)
            gl_ = [mc.GetBoolVarFromProtoIndex(g_.Index()) for g_ in gates.values()]
            hold_of = {}
            for key in sorted(held_, key=repr):
                h_ = mc.NewBoolVar('')
                for v_ in held_[key]:
                    mc.AddImplication(h_, mc.GetBoolVarFromProtoIndex(v_).Not())
                hold_of[h_.Index()] = key
            lit_of = {key: i_ for i_, key in hold_of.items()}

            def ask_(lin_):
                # (one worker, its work counted: the same answer whatever else runs beside it)
                s_ = cp_seed.solver()
                s_.parameters.num_workers = 1
                s_.parameters.max_deterministic_time = START_WORK
                if lin_ is not None:
                    s_.parameters.linearization_level = lin_
                return s_
            # (THE TWO QUESTIONS AT ONCE, where ends are held: THE WHOLE MODEL's -- every rule assumed, no end held,
            # on a copy of this one -- asked beside the first tiers', a worker each. Its refusal is the first tiers'
            # too, which are then not waited for; a state on the first tiers ends its question; and with neither, its
            # answer is the one it gave when asked after them, the same model under the same count of work. Asked
            # one after the other, a group's six rounds with no state spent 229 s on first tiers the whole model
            # then refused in 60)
            # (without the LP first -- a four-bus model's state in a quarter of the time with it -- then with it: a
            # bus held to what four checks had taught had neither a state nor its refusal without, in the same work)
            both_ = None
            if held_:
                both_ = {'m': mc.Clone(), 'ask': [ask_(0), ask_(None)], 'sc': None, 'qc': cp_model.UNKNOWN,
                         'refused': False, 'stop': False, 'first': None, 'err': None}

                def ask_whole(w_=both_):
                    try:
                        for s_ in w_['ask']:
                            w_['sc'] = s_
                            if w_['stop']:
                                return
                            w_['qc'] = s_.Solve(w_['m'])
                            if w_['qc'] != cp_model.UNKNOWN or w_['stop']:
                                break
                        if w_['qc'] == cp_model.INFEASIBLE and not w_['stop']:
                            w_['refused'] = True
                            if w_['first'] is not None:
                                w_['first'].StopSearch()
                    except BaseException as e_:         # noqa: B902 (raised again where it is waited for)
                        w_['err'] = e_
                both_['thread'] = threading.Thread(target=ask_whole, daemon=True)
                both_['thread'].start()
            refused_ = lambda: both_ is not None and both_['refused']      # noqa: E731
            while held_:
                mc.ClearAssumptions()
                mc.AddAssumptions(gl_ + [mc.GetBoolVarFromProtoIndex(lit_of[key]) for key in sorted(held_, key=repr)])
                for lin_ in (0, None):
                    sc = ask_(lin_)
                    both_['first'] = sc
                    if refused_():
                        break
                    qc = sc.Solve(mc)
                    if qc != cp_model.UNKNOWN or refused_():
                        break
                if refused_():
                    break
                if qc in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    narrow_vals = list(sc.ResponseProto().solution)[:nv_m]
                    break
                if qc != cp_model.INFEASIBLE:
                    break
                keys_ = sorted({hold_of[i_] for i_ in sc.SufficientAssumptionsForInfeasibility() if i_ in hold_of},
                               key=repr)
                if not keys_:
                    break                   # (the rules alone refuse: the whole model's answer names them)
                for key in keys_:
                    del held_[key]
                n_freed += len(keys_)
            if n_held:
                log(f'  round {rnd}: a state on the lanes\' first {TIER_START + 1} tiers ({n_held} ends held to them, '
                    f'{n_freed} let go where a refusal named them): '
                    + ('not waited for, the whole model refusing' if refused_() else f'[{sc.StatusName(qc)}]')
                    + f' ({time.time() - t0:.0f} s)')
            mc.ClearAssumptions()
            mc.AddAssumptions(gl_)          # (the whole model: every rule assumed, no end held)
            if both_ is not None:
                if narrow_vals is not None:
                    both_['stop'] = True
                while both_['thread'].is_alive():
                    if both_['stop'] and both_['sc'] is not None:
                        both_['sc'].StopSearch()
                    both_['thread'].join(0.2)
                if both_['err'] is not None:
                    raise both_['err']
                if narrow_vals is None:
                    sc, qc = both_['sc'], both_['qc']
                both_ = None
            else:
                for lin_ in (0, None):
                    sc = ask_(lin_)
                    qc = sc.Solve(mc)
                    if qc != cp_model.UNKNOWN:
                        break
            if qc == cp_model.INFEASIBLE:
                by_ = {g_.Index(): name for name, g_ in gates.items()}
                core_ = sorted(by_[i_] for i_ in sc.SufficientAssumptionsForInfeasibility() if i_ in by_)
                # (a LARGE core is asked again with its own rules alone assumed, while that names fewer: the solver
                # gives some set that cannot hold, not a small one. A core of 135 rules naming 203 lanes had 140
                # balls offered their whole menus at once, and the search then ran on 205 of an array's 364 where
                # 60 had served)
                idx_ = {name: g_.Index() for name, g_ in gates.items()}
                for _shrink in range(8):
                    if len(core_) <= 12:
                        break
                    ms_ = m.Clone()
                    ms_.ClearObjective()
                    ms_.AddAssumptions([ms_.GetBoolVarFromProtoIndex(idx_[name]) for name in core_])
                    ss_ = cp_seed.solver()
                    ss_.parameters.num_workers = 1
                    ss_.parameters.max_deterministic_time = START_WORK
                    ss_.parameters.linearization_level = 0
                    if ss_.Solve(ms_) != cp_model.INFEASIBLE:
                        break
                    less_ = sorted(by_[i_] for i_ in ss_.SufficientAssumptionsForInfeasibility() if i_ in by_)
                    if not less_ or len(less_) >= len(core_):
                        break
                    core_ = less_
                # (the lanes TWO of its rules name first: a rule names every lane of its row -- a group of vias its
                # whole line of balls -- and the ones the rules cannot hold together are those they share. Every lane
                # of every rule widened, a core of two rules over two of a bus's balls took sixteen balls' whole
                # menus, the next eighteen, and 152 of an array's 364 before a state stood)
                cnt_ = collections.Counter(key for name in core_ for key in gate_lanes[name])
                more = set()
                # (...and a plane ball named for its drop last of all: what stands in its drops' way moves first)
                drp_ = {key for name in core_ if name[0] == 'dropped' for key in gate_lanes[name]}
                for named in ({key for key, n_ in cnt_.items() if n_ > 1} - drp_, set(cnt_) - drp_, set(cnt_)):
                    for owner, lane, k_ in named:
                        if owner.startswith(OTHER + ':'):
                            more.add((owner.split(':', 1)[1], lane))
                        elif by_name[owner].arr[k_] is not None:
                            A = by_name[owner].arr[k_]
                            more.update((A.ref, A.ball_of[leg]) for leg in by_name[owner].E.legs_of[lane])
                    more -= (wide - narrowed)
                    if more:
                        break
                # (a LARGE set's balls a few at a time, those whose lanes most of its rules name first: the solver's
                # set is not a small one, and where the rules cannot hold for want of room over a whole face -- two
                # dozen lanes that cross each other on four layers -- no smaller set exists (deletion left 138 of
                # 264 rules in four minutes). All of them widened at once, a three-bus model had 190 balls' whole
                # menus in its second round and one bus 10819 tooth options for 180)
                if len(core_) > 12 and len(more) > WIDE_A_ROUND:
                    rank_ = collections.Counter()
                    for owner, lane, k_ in named:
                        if owner.startswith(OTHER + ':'):
                            rank_[(owner.split(':', 1)[1], lane)] += cnt_[(owner, lane, k_)]
                        elif by_name[owner].arr[k_] is not None:
                            A = by_name[owner].arr[k_]
                            for leg in by_name[owner].E.legs_of[lane]:
                                rank_[(A.ref, A.ball_of[leg])] += cnt_[(owner, lane, k_)]
                    more = set(sorted(more, key=lambda rk: (-rank_[rk], rk))[:WIDE_A_ROUND])
                if more:
                    kinds_ = collections.Counter(name[0] for name in core_)
                    log(f'  round {rnd}: no state in the pattern ({time.time() - t0:.0f} s) -- {len(core_)} rules that cannot '
                        f'hold together ({", ".join(f"{k} {n}" for k, n in sorted(kinds_.items()))}) name {len(named)} '
                        f'lanes; {len(more)} more balls offered their whole menu: '
                        + ', '.join(f'{r_}:{k}' for r_, k in sorted(more)))
                    if len(core_) <= 12:
                        for name in core_:
                            log('      ' + name[0] + ': ' + ' / '.join(str(x) for x in name[1:]))
                    wide |= more
                    release(buses)
                    m = mc = rows = gates = None
                    whole(more)
                    whole_q = False         # (other menus: the repair's own question is asked of them again)
                    continue
                if not core_:
                    # (no array's rule in it: a BUS's own model has no state on the moves its balls are offered -- its
                    # lanes' ends on one layer each and no order of them free of crossings, say -- found by asking each bus
                    # alone; that bus's balls at the arrays are offered their whole menus)
                    alone = []
                    for b in buses:
                        with awx_settings.given(b.env):
                            mb_ = cp_model.CpModel()
                            jb_ = we._joint_model(b.E, b.own, None, lambda *a: None, b.boxes, (), False, m=mb_)
                        sb_ = cp_seed.solver()
                        sb_.parameters.num_workers = 1
                        sb_.parameters.max_deterministic_time = START_WORK
                        if isinstance(jb_, tuple) or sb_.Solve(mb_) == cp_model.INFEASIBLE:
                            alone.append(b)
                    more = {(A.ref, A.ball_of[n]) for b in alone for A in b.arr.values() if A is not None
                            for n in b.E.st['launch'] if n in A.ball_of} - (wide - narrowed)
                    if more:
                        log(f'  round {rnd}: no state in the pattern ({time.time() - t0:.0f} s) -- no array\'s rule named: '
                            f'{", ".join(b.name for b in alone)} alone has none on the moves offered; its {len(more)} '
                            f'balls offered their whole menu')
                        wide |= more
                        release(buses)
                        m = mc = rows = gates = None
                        whole(more)
                        whole_q = False
                        continue
                # (every rule named a LESSON of a check -- an end taken off its layer -- and nothing left to widen: the
                # lesson cannot be held, and is taken back. It was a reading of one refusal, no rule of the board's: a
                # lane's berth on a row part, fixed on its pad's layer, was taught "not on that layer again", the
                # lane had no other end, and the model stopped. The state refused stays refused)
                if core_ and all(n_[0] == 'endlay' for n_ in core_):
                    for _kd, bn_, l_, k_ in core_:
                        learned[bn_]['endlay'] = [e_ for e_ in learned[bn_]['endlay'] if (e_[0], e_[1]) != (l_, k_)]
                    log(f'  round {rnd}: no state ({time.time() - t0:.0f} s) -- what a check taught cannot be held, and '
                        'is taken back: ' + ', '.join(f'{bn_}:{l_} its {("tooth", "berth")[k_]} off its layer'
                                                      for _kd, bn_, l_, k_ in core_))
                    whole_q = False
                    continue
                log(f'group ends: NO STATE holds every rule ({time.time() - t0:.0f} s), the lanes named offered their '
                    'whole menus. The rules that cannot hold together:')
                for name in core_:
                    log('    ' + name[0] + ': ' + ' / '.join(str(x) for x in name[1:]))
                    # (a VIA GROUP named: each of its lanes' options IN it against those it has left at all -- a lane
                    # with every one in it is FORCED in; two forced, the group alone has no state)
                    tv_ = getattr(arrays.get(name[1]), 'trace_vias', {}) if name[0] == 'vias' else {}
                    for kd_, da_, db_ in (tv_.get(name[2], []) if name[0] == 'vias' and len(name) > 2 else []):
                        # (every via row under that one name: a gate is shared by the rows of one lane set)
                        live_ = arrays[name[1]].trace_live
                        rows_ = []
                        for side_, d_ in (('', da_), ('other side: ', db_)):
                            for key_, o_ in sorted(d_.items()):
                                lv_ = live_.get(key_, [])
                                in_ = sorted(set(o_) & set(lv_))
                                rows_.append((len(in_) == len(lv_) and bool(lv_), f'{side_}{key_[0]}:{key_[1]}',
                                              len(in_), len(lv_)))
                        frc_ = [r_[1] for r_ in rows_ if r_[0]]
                        log(f'      ({kd_}) lanes FORCED into it (every option left in it): {len(frc_)} -- '
                            + ', '.join(frc_[:12]) + '; with some option in it: '
                            + ', '.join(f'{r_[1]} {r_[2]}/{r_[3]}' for r_ in rows_ if not r_[0])[:900])
                        # (...and each BUS lane's options there, one by one: in the group or not (which side), refused
                        # by its bus's own model (o[4]: a pair split by other copper between its tips, a far face ...),
                        # its legs' exits and via sites -- what is left to a lane the group leaves no room)
                        opt_rows_ = {}
                        for key_ in sorted(set(da_) | set(db_)):
                            b_ = next((x_ for x_ in buses if x_.name == key_[0]), None)
                            if b_ is None:
                                continue
                            k_e = key_[2]
                            A_e = arrays[name[1]]
                            for i_ in live_.get(key_, []):
                                o_ = (b_.E.T, b_.E.B)[k_e][key_[1]][i_]
                                g_ = [A_e.bm.menu[m_.gid[1]][m_.gid[2]] if getattr(m_, 'gid', None) else m_ for m_ in o_[0]]
                                cf_ = sorted(f'{o2_}#{j_}' for o2_, j_ in (b_.E.oconf.get((key_[1], k_e, i_)) or ())
                                             if any(o2_ == kk_[1] and kk_[0] == key_[0] for kk_ in set(da_) | set(db_)))
                                opt_rows_.setdefault(f'{key_[0]}:{key_[1]}', []).append({
                                    'opt': i_, 'side': 0 if i_ in da_.get(key_, ()) else 1 if i_ in db_.get(key_, ()) else None,
                                    'refused': bool(o_[4]), 'layer': o_[2], 'oconf': cf_,
                                    'legs': [{'kind': g0_.kind, 'dir': g0_.direction,
                                              'exit': [round(c_, 3) for c_ in g0_.exit_pt],
                                              'site': [round(c_, 3) for c_ in g0_.site] if g0_.site else None}
                                             for g0_ in g_]})
                        for ln_, os_ in opt_rows_.items():
                            log(f'      {ln_}: ' + '; '.join(
                                f'#{o_["opt"]} side {o_["side"]}{" REFUSED" if o_["refused"] else ""} '
                                + ' + '.join(f'{lg_["kind"]}/{lg_["dir"]} exit {lg_["exit"]} site {lg_["site"]}'
                                             for lg_ in o_['legs'])
                                + (f' -- its bus model\'s conflicts with {", ".join(o_["oconf"])}' if o_['oconf'] else '')
                                for o_ in os_))
                        TR.rec('core_vias', check=chk_try + 1, ref=name[1], kind=kd_, forced=frc_,
                               lanes={r_[1]: [r_[2], r_[3]] for r_ in rows_}, bus_options=opt_rows_)
                if not core_:
                    log('    none of the arrays\' rules: a bus\'s own model has no state')
                    # (...WHICH of what its refusals taught it cannot hold: each kind of lesson lifted alone, and
                    # each lane of the kind that frees it -- the rules named, as the arrays' are)
                    for b in buses:
                        def own_state_(b=b):
                            with awx_settings.given(b.env):
                                mb_ = cp_model.CpModel()
                                jb_ = we._joint_model(b.E, b.own, None, lambda *a: None, b.boxes, (), False, m=mb_)
                            sb_ = cp_seed.solver()
                            sb_.parameters.num_workers = 1
                            sb_.parameters.max_deterministic_time = START_WORK
                            return not isinstance(jb_, tuple) and sb_.Solve(mb_) != cp_model.INFEASIBLE
                        if own_state_():
                            continue
                        keep_ = {a_: getattr(b.E, a_) for a_ in ('_nochg', '_nocross', '_nogoods', '_samehands')
                                 if hasattr(b.E, a_)}
                        for a_, v_ in sorted(keep_.items()):
                            setattr(b.E, a_, type(v_)())
                            free_ = own_state_()
                            setattr(b.E, a_, v_)
                            if not free_:
                                continue
                            which_ = []
                            for it_ in (sorted(v_, key=repr) if not isinstance(v_, list) else list(v_)):
                                setattr(b.E, a_, type(v_)(x_ for x_ in v_ if x_ != it_))
                                if own_state_():
                                    which_.append(it_)
                                setattr(b.E, a_, v_)
                            log(f'    {b.name}: a state without what it was taught as {a_[1:]} ({len(v_)}); without '
                                f'one of them alone: ' + (', '.join(str(sorted(x_) if isinstance(x_, frozenset) else x_)
                                                                    for x_ in which_)[:600] or 'none'))
                json.dump({'status': 'INFEASIBLE', 'core': [list(n_) for n_ in core_]},
                          open(os.path.join(out, 'group.json'), 'w'), indent=1)
                raise SystemExit(3)
            # (neither found nor ruled out in its time: after a refusal there is a state at hand -- the one refused --
            # for THE REPAIR to start from, which asks the same question with most lanes held; only with none is
            # this the end. A three-bus model stopped here at its fourth re-solve, the repair never asked)
            state_known = qc in (cp_model.OPTIMAL, cp_model.FEASIBLE)
            if not state_known and not (fresh and all(b.name in held_idx for b in buses)):
                raise SystemExit(f'group_ends: a state not found nor ruled out [{sc.StatusName(qc)}] in '
                                 f'{time.time() - t0:.0f} s')
            break
        log(f'  round {rnd}: ' + (f'a state holding every rule: [{sc.StatusName(qc)}]' if state_known else
                                  f'a state neither found nor ruled out [{sc.StatusName(qc)}] -- left to the repair')
            + f' ({time.time() - t0:.0f} s); {len(wide)} of {len(every)} balls on their whole menu')
        for g_ in gates.values():
            m.Add(g_ == 1)
        # ---- the objective: the buses' own, summed -- a walled ball first; then the changes of layer and the runs on a
        # plane's layer, together, a run there at the router's own price in vias (route_layers.run_price: it cuts the plane
        # under its neighbours, and is taken only where it saves that many vias -- ranked below the changes, a lane rode a
        # ground plane end to end to save one); then the rest
        cost = [t for b in buses for t in b.J.cost] + [t for r_ in rows.values() for t in r_.cost] + between_cost + \
            [c_ * x_ for c_, x_ in ride]
        const = sum(b.J.const for b in buses) + between_const
        chg = [t for b in buses for t in b.J.chg_cost]
        # (...and a PLANE BALL FANNED OUT with the walled ones, the top rank: a stub left for a later step is a ball
        # on no pour of its net, as a walled one is -- the board's grade counts both. Ranked with the changes of layer,
        # at a hundred of them, a search that ended short of its optimum left one fanned out on a board it routed with
        # no net open)
        fans = [v for r_ in rows.values() for v in r_.fans]
        walls = [t for b in buses for t in b.J.walls] + fans
        # (a change of layer is priced CHANGE_VIAS vias, a run on a plane's layer at the router's price, both in the
        # rest's own currency: everything below a walled ball is one sum)
        W_LEX = int(round(1000 * CHANGE_VIAS))
        below = W_LEX * sum(chg) + sum(c_ * (W_LEX // 1000) * v_ for c_, v_ in plane) + sum(cost) + const
        m.Minimize(below)
        # (A WALLED BALL OUTWEIGHS EVERYTHING BELOW IT -- the most that sum can COME TO, and no more: of the options of
        # one lane's end, of its layers, of another ball's moves or layers, of one pair of ends' priced meetings, one
        # is taken, so the dearest of each counts once; every other term whole; and the start's price on a lane past
        # its first tiers (tier_search's `over`), added to this objective there, a lane each. A state's value read
        # by that weight -- its walled balls the quotient, the rest the remainder -- needs the rest below it, never
        # more. Weighed as if every change of layer and every run on a plane cost what all the rest together could,
        # the weight stood at 2e14 to 8e14, and on menus a little wider -- 334 of 522 balls whole -- that weight on
        # each option of each plane ball overflowed the objective's sum: the solver called the model invalid, and
        # the run stopped with no state)
        one_of = {}
        groups = [list(b.J.xs[k_][l_].values()) for b in buses for k_ in (0, 1) for l_ in b.J.lanes] + \
            [list(b.J.y[k_][l_]) for b in buses for k_ in (0, 1) for l_ in b.J.lanes] + \
            [g_ for r_ in rows.values() for g_ in r_.lits] + [g_ for b in buses for g_ in b.J.fb_one]
        for n_, g_ in enumerate(groups):
            for v_ in g_:
                one_of[v_.Index()] = n_
        pr_ = m.Proto()
        ob_ = pr_.floating_point_objective if len(pr_.floating_point_objective.vars) else pr_.objective
        top_, low_, more_, less_ = collections.defaultdict(float), collections.defaultdict(float), 0.0, 0.0
        for v_, c_ in zip(ob_.vars, ob_.coeffs):
            i_, c_ = (v_, c_) if v_ >= 0 else (-v_ - 1, -c_)
            g_ = one_of.get(i_)
            if g_ is None:
                more_, less_ = more_ + max(c_, 0), less_ + min(c_, 0)
            else:
                top_[g_], low_[g_] = max(top_[g_], c_), min(low_[g_], c_)
        hi_ = ob_.offset + more_ + sum(top_.values()) + \
            int(round(1000 * TIER_OVER)) * sum(len(b.J.lanes) for b in buses)
        lo_ = ob_.offset + less_ + sum(low_.values())
        W_TOP = int(math.ceil(max(hi_, hi_ - lo_))) + 1
        W_TOP += -W_TOP % 1000              # (a thousandth of it whole: the prices are in thousandths of a via)
        m.Minimize(W_TOP * sum(walls) + below)
        log(f'  the objective: a change {W_LEX}, a walled ball {W_TOP}')
        def _search(stall=None):
            # (reproducible, as the joint ends' own: a count of interleaved batches, no clauses shared)
            s_ = cp_seed.solver()
            s_.parameters.num_workers = whole_solve.SOLVE_WORKERS
            s_.parameters.subsolvers.extend(SEARCH)
            s_.parameters.interleave_search = True
            s_.parameters.max_num_deterministic_batches = BATCHES
            s_.parameters.share_glue_clauses = False
            s_.parameters.share_binary_clauses = False
            s_.parameters.log_search_progress = '--log' in flags
            s_.stalled = None
            if stall:
                # (...and STOPPED WHERE IT STALLS, a search that starts from a state already searched -- the rest with
                # the ends held, the rest alone under the upper rank's cap, a repair: `stall` of the search's own model
                # reductions in a row with no better state (its log's '#Model' against '#n' lines -- events of the
                # deterministic search, never a clock; the whole solve's own rule). A one-bus model's search stood
                # within a fortieth of its last state from its 80th second to its 199th; each of these ran as long)
                at_ = {'n': 0, 'k': 0, 'have': False}

                def _progress(line):
                    if '--log' in flags:
                        print(line, flush=True)
                    if re.match(r'#\d+\s', line):
                        at_['n'], at_['have'] = 0, True
                    elif line.startswith('#Model'):
                        at_['k'] += 1
                        at_['n'] += at_['have']
                        if at_['n'] >= stall:
                            s_.stalled = at_['k']
                            s_.StopSearch()
                s_.parameters.log_search_progress = True
                s_.parameters.log_to_stdout = False
                s_.log_callback = bus_events.solver_progress('ends', 'the model', 1000, then=_progress, bus=None)
            else:
                bus_events.watch_solve(s_, 'ends', 'the model', 1000, bus=None)
            return s_
        # ---- THE REPAIR, after a check's refusal on this tier: every lane the refusal did not name HELD on the ends
        # it had -- but the lanes that crossed one it named -- and the rest solved for the objective: the state the
        # check refused, changed only where it was refused. Asked for "the state nearest the ends wanted" instead, the
        # start's one worker came back with 25 to 46 of a bus's 76 ends moved after a refusal naming one to three
        # lanes; the whole search then ran again, the state went elsewhere, and the next check found other lanes
        # short of room -- six checks, each naming the same few lanes' neighbours, and no plan. With no state that
        # way, the start and the search as before
        repaired = False
        repair_info = {}                # bus -> {lane: why it was free in the repair} (a trace's)
        if fresh and not whole_q and all(b.name in held_idx for b in buses):
            t0 = time.time()
            # (the held ends: the ones the state above was asked with, where it was the repair's own)
            n_free, hold_, n_let, repair_info = probe or repair_holds(m, buses, fresh, prev_x, held_idx)[:4]
            mh = m.Clone()
            for i_ in sorted(hold_):
                mh.Add(mh.GetBoolVarFromProtoIndex(i_) == 1)
            # (...AND WHAT A ROUND OF THE SEARCH HOLDS BESIDE THEM, first: the layers of every lane but the free ones
            # and the LNS_LAYERS nearest them, and every other ball farther than LNS_NEAR from them, where the state
            # refused had them. With only the ends held the repair was the whole group's layers and other balls
            # searched again for a few lanes' ends: a four-bus group's two repairs, of six and seven lanes, took 36
            # and 48 s, one stopped where it stalled; held like this 4 and 3 s, both proved, and ended within a via
            # of the same states. With no state that way, the ends alone held, as before)
            hv_ = dict(zip(m.Proto().solution_hint.vars, m.Proto().solution_hint.values))
            pts_ = {(b.name, l_): [tuple(p_) for e_ in lane_points(b, l_) for p_ in e_] for b in buses
                    for l_ in b.J.lanes}
            held_l = set(hold_.values())
            fp_ = [p_ for key_ in sorted(pts_, key=repr) if key_ not in held_l for p_ in pts_[key_]]
            dist_ = lambda ps, qs: min((math.hypot(p_[0] - q_[0], p_[1] - q_[1]) for p_ in ps for q_ in qs),   # noqa: E731
                                       default=1e9)
            near_l = set(sorted((key_ for key_ in pts_ if key_ in held_l),
                                key=lambda key_: (dist_(pts_[key_], fp_), repr(key_)))[:LNS_LAYERS])
            fixn = {}
            for b in buses:
                for l_ in b.J.lanes:
                    if (b.name, l_) in held_l and (b.name, l_) not in near_l:
                        fixn.update((v_.Index(), hv_[v_.Index()]) for k_ in (0, 1) for v_ in b.J.y[k_][l_]
                                    if hasattr(v_, 'Index') and v_.Index() in hv_)
            for ref, r_ in rows.items():
                for k, (xo_, yo_, dv) in r_.others.items():
                    p_ = arrays[ref].bm.items[k][1]
                    if dist_([(p_.global_x, p_.global_y)], fp_) > LNS_NEAR:
                        fixn.update((v_.Index(), hv_[v_.Index()])
                                    for v_ in list(xo_.values() if isinstance(xo_, dict) else xo_) + list(yo_) + list(dv)
                                    if hasattr(v_, 'Index') and v_.Index() in hv_)
            sh = rh = None
            if fp_ and fixn:
                mn = mh.Clone()
                Pn_ = mn.Proto()
                for v_, x_ in fixn.items():
                    d_ = Pn_.variables[v_].domain
                    d_.clear()
                    d_.extend([x_, x_])
                sn = _search(stall=SEARCH_STALL)
                rn = sn.Solve(mn)
                if rn in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    sh, rh = sn, rn
                else:
                    log(f'  the repair with the far lanes\' layers and the far balls held too: no state that way '
                        f'[{sn.StatusName(rn)}] ({time.time() - t0:.0f} s) -- the ends alone held')
            if sh is None:
                sh = _search(stall=SEARCH_STALL)
                rh = sh.Solve(mh)
            if rh == cp_model.INFEASIBLE and not small_now:
                # (THE HELD ENDS PROVED UNABLE TO STAND -- by the repair's own search, where the question asked of
                # them first had neither a state nor a refusal in its work: the lanes in their way are named -- the
                # same question, each held end an assumption, in the start's work -- let go, and the repair asked
                # once more. Left there, a refusal naming two lanes ended a run: the holds had no state in five
                # seconds of the search, the question first asked of them had none either way in its twenty, no lane
                # was let go, and the whole model's question after it stood unanswered through its work)
                t1_ = time.time()
                got2_ = repair_holds(m, buses, fresh, prev_x, held_idx, refused=True, work=START_WORK)
                if got2_[2]:
                    n_free, hold_, n_let, repair_info = got2_[:4]
                    log(f'  the repair: the held ends proved unable to stand -- {n_let} lane(s) in their way let go '
                        f'({time.time() - t1_:.0f} s), asked again')
                    mh = m.Clone()
                    for i_ in sorted(hold_):
                        mh.Add(mh.GetBoolVarFromProtoIndex(i_) == 1)
                    sh = _search(stall=SEARCH_STALL)
                    rh = sh.Solve(mh)
            repaired = rh in (cp_model.OPTIMAL, cp_model.FEASIBLE)
            log(f'  the repair: every lane held on its ends but the {n_free} the refusal named or that crossed one'
                + (f' and {n_let} more that could not stand with what it taught' if n_let else '') + ': '
                + (f'[{sh.StatusName(rh)}]' if repaired else f'no state that way [{sh.StatusName(rh)}], searched whole')
                + (f', stopped where it stalled (its {sh.stalled}th reduction)' if sh.stalled else '')
                + f' ({time.time() - t0:.0f} s)')
        whole_q = False
        if repaired:
            sv, r, quick = sh, rh, True
        elif small_now:
            # (the repair's small model, searched, had no state: the whole model, as before)
            log(f'  the repair\'s model had no state [{sh.StatusName(rh)}]: built whole')
            release(buses)
            m = mc = mh = sv = rows = gates = None
            small_r, again = False, True
            continue
        elif not state_known and deferred:
            # (...the whole model's question after all: its refusal names the rules, its state starts the search)
            log(f'  the repair had no state [{sh.StatusName(rh)}]'
                + (f' ({m.Validate()[:160]})' if rh == cp_model.MODEL_INVALID else '')
                + ': the whole model\'s question asked, the model built again')
            release(buses)
            m = mc = mh = sv = rows = gates = None
            whole_q = again = True
            continue
        elif not state_known:
            raise SystemExit(f'group_ends: a state not found nor ruled out, and none by the repair '
                             f'[{sh.StatusName(rh)}]')
        elif tier_of:
            # ---- THE SEARCH ON THE TIERS (tier_search): the start on each lane's first tiers, then the rounds that
            # free a few lanes on their whole menus -- each lane's option variables with their tiers and the places of
            # its balls, each other ball's variables and its place
            t0 = time.time()
            ix_ = lambda v_: v_.Index() if hasattr(v_, 'Index') else -1         # noqa: E731
            meta_ = {'W_LEX': W_LEX, 'W_TOP': W_TOP, 'lanes': [], 'others': [],
                     'far': [x_.Index() for c_, x_ in far if c_ and hasattr(x_, 'Index')]}
            for b in buses:
                for l_ in b.J.lanes:
                    e_ = {'bus': b.name, 'lane': str(l_), 'npair': b.J.npair[l_], 'chg': ix_(b.J.chg[l_]),
                          'x': [[], []], 'tier': [[], []], 'at': lane_points(b, l_),
                          'ref': [b.arr[k_].ref if b.arr[k_] is not None else None for k_ in (0, 1)],
                          'y': [[ix_(v_) for v_ in b.J.y[k_][l_]] for k_ in (0, 1)]}
                    for k_ in (0, 1):
                        for i_, x_ in sorted(b.J.xs[k_][l_].items()):
                            e_['x'][k_].append(ix_(x_))
                            e_['tier'][k_].append(option_tier(b, k_, l_, i_, tier_of))
                    meta_['lanes'].append(e_)
            for ref_, r_ in rows.items():
                for k, (xs_o, ys_o, dv_o) in r_.others.items():
                    p_ = arrays[ref_].bm.items[k][1]
                    vs_ = list(xs_o.values() if isinstance(xs_o, dict) else xs_o) + list(ys_o) + list(dv_o)
                    meta_['others'].append({'ref': ref_, 'ball': k, 'at': [p_.global_x, p_.global_y],
                                            'vars': [ix_(v_) for v_ in vs_ if ix_(v_) >= 0]})
            true_more = None
            if NL == 2:
                # (two routing layers) what the model does not count of a state, in its objective's units: the changes
                # of layer the order of its crossings forces past the model's own (whole_ends.route_floor), a change
                # each; None where no order holds. The model counts a lane's change where its ends' layers differ
                # and nothing where they agree; a search judged by it alone ended, by its seed, on states of 80 to
                # 102 vias' worth that the route then laid with 86 to 132, or had no plan for
                def true_more(vals__, buses=buses, W_LEX=W_LEX):
                    tot_ = 0
                    for b_ in buses:
                        fl__ = we.route_floor(b_.J, lambda v_: v_ if isinstance(v_, int) else vals__[v_.Index()])
                        if fl__ is None:
                            return None
                        tot_ += W_LEX * sum(b_.J.npair[l_] * (t_ - t_ % 2) for l_, t_ in fl__[2].items())
                    return tot_
                idx_ = {(e_['bus'], e_['lane']): j_ for j_, e_ in enumerate(meta_['lanes'])}

                def groups_of(vals__, buses=buses, idx_=idx_):
                    # (what each lane crosses in the state, the lanes its crossings' order forces to dive, and the
                    # forced threes themselves -- whole_ends.dive_triples -- by the lanes' places in the search's list)
                    X_, dv_, cu_, fx_ = collections.defaultdict(list), [], [], []
                    for b_ in buses:
                        val_ = lambda v_: v_ if isinstance(v_, int) else vals__[v_.Index()]     # noqa: E731
                        for (a_, c_), x_ in b_.J.X.items():
                            ja_, jc_ = idx_.get((b_.name, str(a_))), idx_.get((b_.name, str(c_)))
                            if ja_ is not None and jc_ is not None and val_(x_):
                                X_[ja_].append(jc_)
                                X_[jc_].append(ja_)
                        fl__ = we.route_floor(b_.J, val_)
                        if fl__ is not None:
                            dv_ += [idx_[(b_.name, str(l_))] for l_, t_ in sorted(fl__[2].items())
                                    if t_ >= 2 and (b_.name, str(l_)) in idx_]
                        for t_ in we.dive_triples(b_.J, val_):
                            js_ = tuple(idx_.get((b_.name, str(l_))) for l_ in t_)
                            if None not in js_:
                                cu_.append(js_)
                    return X_, dv_, cu_, fx_

                def dive_cuts(mc_, free_, vals__, buses=buses, W_LEX=W_LEX):
                    # (the dives a round of the search can see: whole_ends.dive_cuts, a dive two changes)
                    return sum(we.dive_cuts(b_.J, mc_, {l_ for bn_, l_ in free_ if bn_ == b_.name}, vals__,
                                            2 * W_LEX) for b_ in buses)
                meta_['groups'], meta_['more'], meta_['cuts'] = groups_of, true_more, dive_cuts
            if '--model' in flags:
                # (a bench's: the model the search takes, and which of its variables are which lane's and ball's)
                m.ExportToFile(os.path.join(out, 'model.pb.txt'))
                json.dump(meta_, open(os.path.join(out, 'tiers.json'), 'w'))
            # (THE EPOCHS, at the first check: the start alone on the first tiers, then each stretch's rounds)
            pool_ = None
            if in_epoch and narrow_vals is not None:
                st_ = set(stretches[epoch])
                pool_ = [j_ for j_, e_ in enumerate(meta_['lanes']) if (e_['bus'], e_['lane']) in st_]
            elif chk_try == 0:
                pool_ = []
            vals_, line_ = tier_search(m, meta_, log, hint=narrow_vals, pool=pool_)
            if vals_ is None:
                raise SystemExit(f'group_ends: no state on the tiers -- {line_}')
            log(f'  the search: {line_}')
            if true_more is not None:
                more_last = true_more(vals_) or 0
            # (the state as a solver's answer: read, not solved for -- it is a solver's own answer on this model, every
            # variable's value. Held in a copy of the model and solved once more to be read, and hinted back into a
            # model that is let go before any solve reads it, each stretch paid a second for what it had)
            sv, r, quick = StateRead(m, vals_), cp_model.OPTIMAL, True
            searched.update((b.name, b.name in lined) for b in buses)
        else:
            # ---- THE START, as a bus's own ends' search starts (whole_ends.nearest_start): the state NEAREST the ends wanted
            # -- the fewest of them moved -- then, those ends HELD, the rest searched for the objective (the other balls'
            # moves and drops, every lane's layers), hinted whole. From any state the rules allow, the search spent its
            # budget coming down and each model solved again started somewhere else (a bus's crossings over five checks:
            # 191, 141, 60, 138, 146). None near them in its time: the state found
            t0 = time.time()
            hints_ = m.Proto().solution_hint
            hv_ = dict(zip(hints_.vars, hints_.values))
            ends_ = [v for b in buses for k_ in (0, 1) for l_ in b.J.lanes for v in b.J.xs[k_][l_].values()]
            ei_ = {v.Index() for v in ends_}
            # (...but a lane its bus's step named in its findings: where it stood is what was found crowded)
            want_ = [b.J.xs[k_][l_][wanted[b.name][l_][k_]] for b in buses for k_ in (0, 1) for l_ in b.J.lanes
                     if wanted[b.name][l_][k_] in b.J.xs[k_][l_] and l_ not in fed.get(b.name, ())]
            # (...and the layers the whole solve's plan gave a bus's ends, wanted as the ends are)
            for b in buses:
                for l_, qs_ in sorted(layers_want.get(b.name, {}).items()):
                    if l_ in b.J.y[0]:
                        for k_ in (0, 1):
                            ends_ += list(b.J.y[k_][l_])
                            want_.append(b.J.y[k_][l_][qs_[k_]])
            ei_ = {v.Index() for v in ends_}
            m.ClearHints()
            # (the nearest states ranked by the walled balls and the upper rank: an end that has to move is else put
            # anywhere -- on a far face, seven of a four-bus group's)
            upper_ = 1000 * sum(chg) + sum(c_ * v_ for c_, v_ in plane)
            from ortools.sat.python import cp_model_helper
            then_ = upper_ + (int(sum(abs(c_) for c_ in cp_model_helper.FlatIntExpr(upper_).coeffs)) + 1) * sum(walls)
            got = we.nearest_start(m, want_, ends_, hints=[(m.GetIntVarFromProtoIndex(i_), x_) for i_, x_ in hv_.items()
                                                           if i_ not in ei_], work=START_WORK, lin=0, rest=False,
                                   then=then_, export=os.path.join(out, f'near_{chk_try}.pb.txt')
                                   if '--model' in flags else None)

            if isinstance(got, str):
                log(f'  the start: {got} ({time.time() - t0:.0f} s) -- the state found')
                for i_ in range(len(mc.Proto().variables)):
                    m.AddHint(m.GetIntVarFromProtoIndex(i_), sc.Value(mc.GetIntVarFromProtoIndex(i_)))
            else:
                vals_ = got[2]
                mh = m.Clone()
                for v in ends_:
                    mh.Add(mh.GetBoolVarFromProtoIndex(v.Index()) == vals_[v.Index()])
                for i_, v_ in enumerate(vals_):
                    mh.AddHint(mh.GetIntVarFromProtoIndex(i_), v_)
                sh = _search(stall=SEARCH_STALL)
                rh = sh.Solve(mh)
                if rh in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    vals_ = [sh.Value(mh.GetIntVarFromProtoIndex(i_)) for i_ in range(len(vals_))]
                ev_ = lambda e_: sum(c_ * vals_[v_.Index()] for c_, v_ in e_)      # noqa: E731
                log(f'  the start: the state nearest the ends wanted, {got[0]} of {len(want_)} ends moved, the rest searched '
                    f'with them held [{sh.StatusName(rh)}]'
                    + (f', stopped where it stalled (its {sh.stalled}th reduction)' if sh.stalled else '')
                    + f' ({time.time() - t0:.0f} s) -- '
                    f'{sum(vals_[c_.Index()] for b in buses for c_ in b.J.chg.values())} lanes changing layer, '
                    f'{(ev_(plane) - ev_(far)) / 1000:.1f} vias\' worth of runs on a plane\'s layer, '
                    f'{sum(vals_[v_.Index()] for _c, v_ in far)} teeth on a far face')
                for i_, v_ in enumerate(vals_):
                    m.AddHint(m.GetIntVarFromProtoIndex(i_), v_)
            if '--model' in flags:
                m.ExportToFile(os.path.join(out, 'model.pb.txt'))   # (a bench's: the model as the search takes it, its start in it)
                # (...and which of its variables are which bus's lanes' ends, for a bench that holds some and frees others)
                json.dump({'buses': {b.name: {l_: [[v.Index() for v in b.J.xs[k_][l_].values()] for k_ in (0, 1)]
                                              for l_ in b.J.lanes} for b in buses}},
                          open(os.path.join(out, 'model_groups.json'), 'w'))
            # ---- the search -- or none: a model solved again after a check's refusal, every bus's ends searched before on
            # the menus' tier it stands on (`searched`), takes the START as its answer -- the state nearest the one refused
            # that holds what the refusal taught, the rest searched with those ends held -- where that state IS near: no
            # more ends moved than the lanes the refusal named have (`n_taught` of them, two ends each). Searched whole
            # again each time, a bus's ends took four minutes a check, each naming one lane
            t0 = time.time()
            quick = not isinstance(got, str) and rh in (cp_model.OPTIMAL, cp_model.FEASIBLE) and \
                all((b.name, b.name in lined) in searched for b in buses) and got[0] <= 2 * n_taught
            if quick:
                sv, r = sh, rh
                log('  no search: the state nearest the one refused stands, as the start found it')
            else:
                sv = _search()
                r = sv.Solve(m)
            if r in (cp_model.OPTIMAL, cp_model.FEASIBLE) and not quick:
                searched.update((b.name, b.name in lined) for b in buses)
                # ...THEN THE REST ALONE, under the walled balls and the upper rank it reached, from its state: the weighted
                # search is still moving the upper rank when its budget ends, each such move dealing the crossings again -- a
                # bus's ends had three times the crossings its own run's ends had
                ua_, wa_ = int(round(sv.Value(upper_))), sum(sv.Value(w) for w in walls)
                ra_ = sv.Value(sum(cost) + const)
                va_ = [sv.Value(m.GetIntVarFromProtoIndex(i_)) for i_ in range(len(m.Proto().variables))]
                m.ClearHints()
                for i_, v_ in enumerate(va_):
                    m.AddHint(m.GetIntVarFromProtoIndex(i_), v_)
                m.Add(upper_ <= ua_)
                m.Add(sum(walls) <= wa_)
                m.Minimize(sum(cost) + const)
                t1_ = time.time()
                s2 = _search(stall=SEARCH_STALL)
                r2 = s2.Solve(m)
                if r2 in (cp_model.OPTIMAL, cp_model.FEASIBLE):
                    sv = s2
                log(f'  the search: the upper rank {ua_ / 1000:.1f} vias\' worth, the rest {ra_ / 1000:.1f} '
                    f'({t1_ - t0:.0f} s); then the rest alone under that cap: '
                    f'{sv.Value(sum(cost) + const) / 1000:.1f}'
                    + (f', stopped where it stalled (its {s2.stalled}th reduction)' if s2.stalled else '')
                    + f' ({time.time() - t1_:.0f} s)')
        if r not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            raise SystemExit(f'group_ends: no state [{sv.StatusName(r)}] in {time.time() - t0:.0f} s')
        log(f'group ends: [{sv.StatusName(r)}] in {time.time() - t0:.0f} s, the rest '
            f'{sv.Value(sum(cost) + const) / 1000:.1f}'
            + ('' if quick else f' (bound {sv.BestObjectiveBound() / 1000:.1f})')
            + f'; {sum(sv.Value(w) for w in walls) - sum(sv.Value(v) for v in fans)} walled balls, '
            f'{sum(sv.Value(v) for v in fans)} plane balls fanned out, '
            f'{sum(sv.Value(c_) for b in buses for c_ in b.J.chg.values())} lanes changing layer, '
            f'{(sum(c_ * sv.Value(v_) for c_, v_ in plane) - sum(c_ * sv.Value(v_) for c_, v_ in far)) / 1000:.1f} '
            f'vias\' worth of runs on a '
            f'plane\'s layer, {sum(sv.Value(v_) for _c, v_ in far)} ends on a far face (teeth, pairs\' berths), '
            f'{sum(c_ * sv.Value(v_) for c_, v_ in ride) / 1000:.1f} vias\' worth of ways round the arrays and '
            f'inside them')
        bus_events.emit('ends', 'solve', group=sref, bus=None, check=chk_try + 1, status=sv.StatusName(r),
                        secs=round(time.time() - t0), rest=round(sv.Value(sum(cost) + const) / 1000, 1),
                        bound=None if quick else round(sv.BestObjectiveBound() / 1000, 1),
                        walled=sum(sv.Value(w) for w in walls) - sum(sv.Value(v) for v in fans),
                        changing_layer=sum(sv.Value(c_) for b in buses for c_ in b.J.chg.values()))
        TR.rec('model', check=chk_try + 1, rnd=rnd, how='repair' if repaired else ('tiers' if tier_of else 'start+search'),
               status=sv.StatusName(r), secs=round(time.time() - t0, 1), W_LEX=W_LEX, W_TOP=W_TOP,
               free={bn_: dict(sorted(v_.items())) for bn_, v_ in repair_info.items()},
               objective=objective_split(sv, buses, rows, between_cost, ride, walls, fans, chg, plane, far))
        for ref, bx_ in between.items():
            by_ = collections.Counter()
            shared_ = collections.Counter()
            for (a_, _la, b_, _lb), (x_, n_) in bx_.items():
                v_ = 1 if isinstance(x_, int) else sv.Value(x_)
                by_[f'{a_}/{b_}'] += v_
                shared_[f'{a_}/{b_}'] += v_ and sv.Value(n_)
            log(f'  meetings between buses at {ref} (crossings, and one lane run past another\'s exit): '
                f'{sum(by_.values())}, {sum(shared_.values())} of them between lanes that '
                f'share a layer -- ' + (', '.join(f'{k_} {n_} ({shared_[k_]})' for k_, n_ in sorted(by_.items()) if n_)
                                        or 'none'))
        # ---- each bus's ends in the group, and each array as the model fanned it out
        rep = {'buses': {}, 'arrays': {ref: {'exits': {}, 'drops': []} for ref in arrays}, 'status': sv.StatusName(r)}
        laid = []                   # (a net's id, its copper, its run's layer, the fan track, its array)
        wayouts = []                # (a net's id, an exit at an array, its direction, its lane's layer there)
        row_ways = []               # (...the same of a row part's berths: the ties' to keep clear, no nudge's)
        for b in buses:
            s = {l_: (next(i for i, v in b.J.xs[0][l_].items() if sv.Value(v)),
                      next(j for j, v in b.J.xs[1][l_].items() if sv.Value(v))) for l_ in b.J.lanes}
            lay = {l_: tuple(next(q for q in range(NL) if sv.Value(b.J.y[k_][l_][q])) for k_ in (0, 1)) for l_ in b.J.lanes}
            b.state, b.lay = s, lay
            cuts[(b.name, b.name in lined)] = (b.E.wcut,)
            b.cut = None
            prev_bus[b.name] = {l_: tuple((we._opt_sig(o_), lay[l_][k_],
                                           tuple((m_.direction, tuple(m_.exit_pt)) for m_ in o_[0]))
                                          for k_, o_ in enumerate((b.E.T[l_][s[l_][0]], b.E.B[l_][s[l_][1]])))
                                for l_ in b.J.lanes}
            with awx_settings.given(b.env):
                import braid as bd
                b.plan, row = bus_plan(b, s, lay, RL)
                # (a row part's berths, in the board's frame: laid with the arrays)
                for nid, legs_, site in row:
                    laid.append((b.to_bare[nid], je.Piece(None, [(b.F.pt(p_), b.F.pt(q_), L_) for p_, q_, L_ in legs_],
                                               [(b.F.pt(site), bd.VIA_SIZE / 2, bd.VIA_DRILL / 2)] if site is not None
                                               else [], []), None, _rules.active().fan_track, b.E.st['dref']))
                if b.arr[1] is None:
                    for l_, lg in b.E.lanes:
                        for leg, mv in zip(lg, b.E.B[l_][s[l_][1]][0]):
                            row_ways.append((b.to_bare[b.E.st['byname'][leg][0]], tuple(b.F.pt(mv.exit_pt)),
                                             b.F.dirn(mv.direction), RL[lay[l_][1]]))
            nx = sum(1 if isinstance(x_, int) else sv.Value(x_) for x_ in b.J.X.values())
            prev_x[b.name] = {pr_ for pr_, x_ in b.J.X.items() if (x_ if isinstance(x_, int) else sv.Value(x_))}
            nchg = sum(sv.Value(c_) for c_ in b.J.chg.values())
            vias = sum(b.E.T[l_][s[l_][0]][3] + b.E.B[l_][s[l_][1]][3] for l_ in b.J.lanes)
            ends = {}
            for k_ in (0, 1):
                opts = b.E.T if k_ == 0 else b.E.B
                ends[k_] = collections.Counter((b.F.dirn(opts[l_][s[l_][k_]][0][0].direction), RL[lay[l_][k_]])
                                               for l_ in b.J.lanes)
                A = b.arr[k_]
                if A is None:
                    continue
                for l_, lg in b.E.lanes:
                    for leg, mv in zip(lg, opts[l_][s[l_][k_]][0]):
                        _r, k, i = mv.gid
                        g0, L_ = A.bm.menu[k][i], RL[lay[l_][k_]]
                        laid.append((A.bm.items[k][1].net_id, A.bm.opts[k][i][2], L_, A.bm.sz['tw'], A.ref))
                        wayouts.append((A.bm.items[k][1].net_id, tuple(g0.exit_pt), g0.direction, L_))
                        rep['arrays'][A.ref]['exits'][leg] = {
                            'tooth': list(g0.exit_pt), 'direction': g0.direction, 'layer': L_, 'kind': g0.kind,
                            'site': list(g0.site) if g0.site is not None else None, 'bus': b.name,
                            'legs': [[list(a_), list(b_), (L_ if Lg == je.RUN else Lg)] for a_, b_, Lg in g0.legs]}
            log(f'  {b.name}: {len(b.J.lanes)} lanes -- {nx} crossings, {nchg} changing layer, {vias} fanout vias\n'
                f'      its teeth by face and layer: ' + ', '.join(f'{d} {L} {n}' for (d, L), n in sorted(ends[0].items()))
                + '\n      its berths: ' + ', '.join(f'{d} {L} {n}' for (d, L), n in sorted(ends[1].items())))
            # (the bus's plan for its own step, bus_stage: in its frame, which two of the source's pads place -- frame)
            two = sorted(b.arr[0].pads)[:1] + sorted(b.arr[0].pads)[-1:]
            rep['buses'][b.name] = {
                'state': {l_: [we._opt_sig(b.E.T[l_][s[l_][0]]), we._opt_sig(b.E.B[l_][s[l_][1]])] for l_ in b.J.lanes},
                'crossings': int(nx), 'changes': int(nchg), 'vias': int(vias), 'src': sref, 'plan': b.plan,
                'frame': {n_: list(b.G.pt(b.arr[0].pads[n_])) for n_ in two}}
            # (THE STATE, a trace's: each lane's two ends as chosen, its change, the lanes it crosses, and the ends
            # that moved since the state before -- this model's last, or the run's it started from)
            lanes_rec, moved_ = {}, {}
            pv_ = trace_prev.get(b.name) or {}
            for l_ in b.J.lanes:
                r_ = {}
                for k_, nm_ in ((0, 'tooth'), (1, 'berth')):
                    o_ = (b.E.T, b.E.B)[k_][l_][s[l_][k_]]
                    A_ = b.arr[k_]
                    g_ = [A_.bm.menu[m_.gid[1]][m_.gid[2]] if A_ is not None and getattr(m_, 'gid', None) else m_
                          for m_ in o_[0]]
                    r_[nm_] = {'opt': s[l_][k_], 'sig': str(we._opt_sig(o_)), 'layer': RL[lay[l_][k_]],
                               'xy': [list(m_.exit_pt) for m_ in o_[0]],
                               'dir': o_[0][0].direction if o_[0] else None, 'via': we._via_end(o_),
                               'runs_on': sorted({L_ for m_ in o_[0] for L_ in (getattr(m_, 'runs_on', None) or ())}),
                               'legs': [{'kind': getattr(g0_, 'kind', None),
                                         'site': list(g0_.site) if getattr(g0_, 'site', None) is not None else None}
                                        for g0_ in g_]}
                c_ = b.J.chg.get(l_)
                r_['change'] = None if c_ is None else int(c_ if isinstance(c_, int) else sv.Value(c_))
                r_['crosses'] = sorted(c2_ if a2_ == l_ else a2_ for (a2_, c2_), x_ in b.J.X.items() if l_ in (a2_, c2_)
                                       and (x_ if isinstance(x_, int) else sv.Value(x_)))
                lanes_rec[l_] = r_
                now_ = tuple((r_[e_]['sig'], lay[l_][k_]) for k_, e_ in enumerate(('tooth', 'berth')))
                if l_ in pv_ and pv_[l_] != now_:
                    moved_[l_] = [e_ for k_, e_ in enumerate(('tooth', 'berth')) if pv_[l_][k_] != now_[k_]]
            trace_prev[b.name] = {l_: tuple((r_[e_]['sig'], lay[l_][k_]) for k_, e_ in enumerate(('tooth', 'berth')))
                                  for l_, r_ in lanes_rec.items()}
            TR.rec('state', check=chk_try + 1, bus=b.name, cut=b.cut, crossings=int(nx), changes=int(nchg),
                   vias=int(vias), n_moved=len(moved_), moved=moved_, lanes=lanes_rec)
            if NL == 2:
                t0_ = time.time()
                fl_ = we.route_floor(b.J, sv.Value)
                est_parts[b.name] = None if fl_ is None else int(vias) + int(fl_[0])
                log(f'  {b.name}: by the order of its crossings alone: ' + ('no order found' if fl_ is None else
                    f'{fl_[0]} changes of layer at the least ({int(nchg)} the model\'s), {fl_[1]} via(s) past two')
                    + f' ({time.time() - t0_:.1f} s)')
        if FREE:
            held_ = collections.defaultdict(set)
            for ref, bx_ in between.items():
                for (a_, la_, b_, lb_), (x_, _n) in bx_.items():
                    if x_ if isinstance(x_, int) else sv.Value(x_):
                        held_[a_].add(la_)
                        held_[b_].add(lb_)
            n_x = sum(len(v_) for v_ in held_.values())
            with awx_settings.given(genv):
                gap_ = _rules.active().fan_clear
            for ref, A in arrays.items():
                runs_ = []
                for b in buses:
                    for k_ in (0, 1):
                        if b.arr[k_] is not A:
                            continue
                        for l_, lg in b.E.lanes:
                            sg_ = [(tuple(p_), tuple(q_)) for mv in (b.E.T, b.E.B)[k_][l_][b.state[l_][k_]][0]
                                   for p_, q_, Lg in A.bm.menu[mv.gid[1]][mv.gid[2]].legs if Lg == je.RUN]
                            if sg_:
                                runs_.append((b.name, l_, sg_))
                for (na_, la_, sa_), (nb_, lb_, sb_) in itertools.combinations(runs_, 2):
                    if na_ != nb_ and any(we._seg_d(p_, q_, u_, v_) < A.bm.sz['tw'] + gap_
                                          for p_, q_ in sa_ for u_, v_ in sb_):
                        held_[na_].add(la_)
                        held_[nb_].add(lb_)
            for b in buses:
                b.plan['held_layers'] = sorted(leg for l_ in held_[b.name] for leg in (b.E.legs_of.get(l_) or (l_,)))
                # (...and how many pairs of them cross within the bus on a layer they share: their bus's solve cannot
                # move their ends' layers, and has to place a change before each such crossing)
                n_sh = sum(1 for (a_, c_), x_ in b.J.X.items()
                           if a_ in held_[b.name] and c_ in held_[b.name] and (x_ if isinstance(x_, int) else sv.Value(x_))
                           and set(b.lay[a_]) & set(b.lay[c_]))
                log(f'  {b.name}: its ends\' layers its own solve\'s, but {len(held_[b.name])} of {len(b.J.lanes)} lanes\' '
                    f'that cross another bus\'s lane or run along one under an array'
                    + (f' -- {n_sh} pair(s) of them cross each other on a layer they share' if n_sh else ''))
            if len(buses) > 1:
                log(f'  lanes held for a crossing between buses: {n_x}; with those along another bus\'s run: '
                    f'{sum(len(v_) for v_ in held_.values())}')
        # (each bus's CUT round an array destination against the best one for the berths chosen -- one in each gap between
        # two of them, whole_frame.cut_gaps: what choosing the cut with the ends would save)
        import whole_frame
        for b in buses:
            if b.arr[1] is None:
                continue
            E_, J_ = b.E, b.J
            TQ_ = int(round(we.TIE_MM * 1000))
            tp_ = [J_.pos[(0, l_, b.state[l_][0])] for l_ in J_.lanes]
            bo_ = [E_.B[l_][b.state[l_][1]] for l_ in J_.lanes]

            def crossings(cut, tp_=tp_, bo_=bo_, E_=E_, J_=J_, TQ_=TQ_):
                bp_ = [int(round(we._place_b(E_, o_, J_.DB, cut) * 1000)) for o_ in bo_]
                return sum(1 for i_, j_ in itertools.combinations(range(len(tp_)), 2)
                           if (tp_[i_] < tp_[j_] - TQ_ and bp_[j_] < bp_[i_] - TQ_)
                           or (tp_[j_] < tp_[i_] - TQ_ and bp_[i_] < bp_[j_] - TQ_))
            cands = whole_frame.cut_gaps([o_[1] for o_ in bo_], J_.DB,
                                         avoid=[m_.exit_pt for l_ in J_.lanes for o_ in E_.B[l_] for m_ in o_[0]])
            own_ = J_.cut if b.cut is None else b.cut
            best = min([own_] + list(cands), key=crossings)        # (its own among them: first, so kept on a tie)
            fmt = lambda c_: f'{c_[0]} {float(c_[1]):.2f}' if isinstance(c_, (tuple, list)) else f'far face {float(c_):.2f}'   # noqa: E731
            log(f'  {b.name}: its cut round {b.E.st["dref"]} ({fmt(own_)}): {crossings(own_)} crossings; the best of '
                f'{len(cands) + 1} cuts for these berths ({fmt(best)}): {crossings(best)}')
        for ref, A in arrays.items():
            how = collections.Counter()
            for k, (xo_, yo_, dv) in rows[ref].others.items():
                kind_ = 'planes' if k in A.drops else 'other signals'
                i = next((i for i, v in xo_.items() if sv.Value(v)), None)
                if i is not None:
                    g0, L_ = A.bm.menu[k][i], RL[next(q for q in range(NL) if sv.Value(yo_[q]))]
                    prev_oth[(ref, k)] = ('x', sr_.move_sig(g0), RL.index(L_))
                    how[(kind_, 'fanned out')] += 1
                    laid.append((A.bm.items[k][1].net_id, A.bm.opts[k][i][2], L_, A.bm.sz['tw'], ref))
                    rep['arrays'][ref]['exits'][k] = {
                        'tooth': list(g0.exit_pt), 'direction': g0.direction, 'layer': L_, 'kind': g0.kind,
                        'site': list(g0.site) if g0.site is not None else None, 'bus': kind_,
                        'legs': [[list(a_), list(b_), (L_ if Lg == je.RUN else Lg)] for a_, b_, Lg in g0.legs]}
                else:
                    j = next(j for j, v in enumerate(dv) if sv.Value(v))
                    prev_oth[(ref, k)] = ('d', j)
                    nd_ = len(A.drops.get(k, ()))
                    if j >= nd_:        # (strapped to a ball of its net beside it)
                        s_, pc_ = A.straps[k][j - nd_]
                        how[(kind_, 'strapped')] += 1
                        laid.append((A.bm.items[k][1].net_id, pc_, None, A.bm.sz['tw'], ref))
                        rep['arrays'][ref].setdefault('straps', []).append(
                            {'ball': k, 'net': A.bm.items[k][0], 'at': list(A.bm.balls[k]), 'to': s_.to})
                        continue
                    how[(kind_, 'dropped')] += 1
                    laid.append((A.bm.items[k][1].net_id, A.drops[k][j][1], None, A.bm.sz['tw'], ref))
                    rep['arrays'][ref]['drops'].append({'ball': k, 'net': A.bm.items[k][0], 'at': list(A.bm.balls[k]),
                                                        'site': list(A.drops[k][j][0].site),
                                                        'in_pad': bool(A.drops[k][j][0].inpad)})
            log(f'  {ref}\'s other balls: ' + (', '.join(f'{n} {a} {b_}' for (a, b_), n in sorted(how.items())) or 'none'))
            os.makedirs(os.path.join(out, ref), exist_ok=True)
            json.dump(rep['arrays'][ref]['exits'], open(os.path.join(out, ref, 'exits.json'), 'w'), indent=0)
            json.dump(rep['arrays'][ref]['drops'], open(os.path.join(out, ref, 'drops.json'), 'w'), indent=0)
        # ---- THE EPOCHS (TIER_MENU, at the first check): the stretch just searched back on its first tiers, each lane
        # on the move it took, and the next stretch of lanes round the source offered their whole menus -- with every
        # ball near a plane ball still fanned out, whose drops something stands in the way of -- and the model built
        # again from this state. The last stretch searched, the state is checked
        def costly_now():
            # (the COSTLY BALLS of the state: a plane ball fanned out, and the source's ball of every lane with a tooth
            # on its far face, the dearest end)
            fan_ = [(ref, k) for ref, A in arrays.items() for k in rows[ref].others if k in A.drops
                    and (prev_oth.get((ref, k)) or ('d',))[0] == 'x']
            farl_ = [(b, l_) for b in buses if b.arr[0] is not None for l_ in b.J.lanes
                     if b.E.T[l_][b.state[l_][0]][0] and b.E.T[l_][b.state[l_][0]][0][0].direction == 'left']
            return fan_ + [(b.arr[0].ref, b.arr[0].ball_of[leg]) for b, l_ in farl_
                           for leg in (b.E.legs_of.get(l_) or (l_,)) if leg in b.arr[0].ball_of]

        def near_of(fan_):
            # (...and the balls round each, not yet on their whole menus)
            return {(ref, k2) for ref, k in fan_ for k2 in arrays[ref].bm.items
                    if math.dist(arrays[ref].bm.balls[k], arrays[ref].bm.balls[k2]) <= LNS_NEAR
                    and ((ref, k2) not in wide or (ref, k2) in narrowed)}
        def sweep_value():
            # (a state's price for the sweeps' rule: its walled balls and plane balls fanned out as a COUNT, and the
            # rest of its objective apart -- the weight on one such ball, W_TOP, is each model's own, grown with the
            # menus it was built on, so two builds' whole objectives differ by it where one such ball stands in both)
            n_ = sum(sv.Value(w_) for w_ in walls)
            return n_, sv.ObjectiveValue() - W_TOP * n_ + more_last
        # (...the state's price below its costly balls, held against the state before's -- A DEAR REPAIR, below)
        v_now_ = sweep_value()[1]
        dear_was_ = v_last
        dear_ = bool(repaired and v_last is not None and v_now_ - v_last >= W_LEX)
        v_last = v_now_
        if chk_try == 0 and tier_of and (not repaired or at_point == '--at-start'):
            fresh = {}
            stale_ = set()          # (the arrays whose menus the stretch searched was taken back on, their rules' data due)
            if stretches is None:
                stretches, sweep_v0 = lane_stretches(buses), sweep_value()
            if 0 <= epoch < len(stretches):
                keep_ = {}
                for b in buses:
                    for k_ in (0, 1):
                        A_ = b.arr[k_]
                        if A_ is None:
                            continue
                        for l_, lg in b.E.lanes:
                            for leg, mv in zip(lg, (b.E.T, b.E.B)[k_][l_][b.state[l_][k_]][0]):
                                key_ = (A_.ref, mv.gid[1])
                                if key_ in tier_sig:
                                    keep_[key_] = tier_sig[key_][1](A_.bm.menu[mv.gid[1]][mv.gid[2]])
                # (...every bus ball the epoch offered its whole menu -- its stretch's, and those round a costly ball:
                # kept whole, the balls round nine far-face teeth made every later model of a four-bus group the
                # larger, its footprint 8.8 GB where the stretches alone peaked at 4.7)
                back_ = (epoch_balls | stretch_balls(buses, stretches[epoch])) & (wide - narrowed) & set(keep_)
                kept_last = dict(keep_)     # (every bus ball's move in the state: the search's end, search.pkl)
                if back_:
                    # (the arrays' rules' data made once for the two changes of menu a switch of stretch is -- this
                    # one's balls back, the next one's whole -- by `whole` below: made here too, each switch worked
                    # out an array's groups of vias and its lines' points for menus no model was built on)
                    stale_ = narrow(back_, keep_, rules=False)
            epoch += 1
            now_ = sweep_value()
            if epoch == len(stretches) and sweep + 1 < EPOCH_SWEEPS and len(stretches) > 1 \
                    and (now_[0] < sweep_v0[0]
                         or (now_[0] == sweep_v0[0] and sweep_v0[1] - now_[1] >= EPOCH_GAIN * now_[1])):
                # (a sweep that gained -- a walled ball or a plane ball fanned out the fewer, or with as many the
                # rest of the objective down by EPOCH_GAIN of itself: another, its stretches drawn at random)
                top_ = (lambda n_: f' with {n_} ball(s) walled or fanned out' if n_ else '')   # noqa: E731
                log(f'  the search\'s sweep {sweep + 1}: {sweep_v0[1] / 1000:.1f}{top_(sweep_v0[0])} -> '
                    f'{now_[1] / 1000:.1f}{top_(now_[0])} -- another, its stretches drawn at random')
                sweep, epoch, sweep_v0 = sweep + 1, 0, now_
                stretches = lane_stretches(buses, draw=sweep)
            if epoch < len(stretches):
                more_ = stretch_balls(buses, stretches[epoch]) & narrowed
                fan_ = costly_now()
                if NL == 2:
                    # ...and the balls of the lanes the state's dives are owed to (whole_ends.dive_triples):
                    # on their first tiers alone outside their own stretch, they had nowhere else to go
                    for b in buses:
                        for l_ in we.dive_cover(we.dive_triples(b.J, sv.Value), LNS_K):
                            for A_ in b.arr.values():
                                if A_ is not None:
                                    fan_ += [(A_.ref, A_.ball_of[leg]) for leg in (b.E.legs_of.get(l_) or (l_,))
                                             if leg in A_.ball_of]
                near_ = near_of(fan_)
                log(f'  the search\'s stretch {epoch + 1} of {len(stretches)}: {len(stretches[epoch])} lanes, '
                    f'{len(more_)} balls offered their whole menus'
                    + (f'; {len(near_)} more round {len(fan_)} costly ball(s) (plane balls fanned out, teeth on a '
                       f'far face)' if near_ else ''))
                wide |= near_
                release(buses)
                m = mc = mh = sv = rows = gates = None
                whole(more_ | near_)
                for ref in sorted(stale_ - {r_ for r_, _k in more_ | near_}):
                    array_rules(bpcb, arrays[ref])
                epoch_balls = more_ | near_
                epoch_fan = list(fan_)
                in_epoch = True
                continue
            for ref in sorted(stale_):
                array_rules(bpcb, arrays[ref])
            in_epoch = False
        elif in_epoch:
            in_epoch = False            # (a costly ball's stretch searched: on to the checks)
        elif tier_of:
            # ---- A COSTLY BALL LEFT by a later model -- a repair after a check, the model solved again for a lesson
            # of the field laid, each holding most lanes on their ends: the lanes nearest it, one stretch, their balls
            # and the balls round it offered their whole menus, and searched from this state as an epoch is. A repair
            # that held every lane round a plane ball whose drop a lesson had taken left it fanned out. COSTLY_TRIES
            # A BALL, not a run: a search that clears its ball has spent nothing a later repair's new one needs --
            # two tries a run, each clearing its ball, left the last repair's three plane balls fanned out, one of
            # them walled in by the lanes laid round its stub
            fan_ = [b_ for b_ in costly_now() if costly_tries[b_] < COSTLY_TRIES]
            if fan_:
                costly_tries.update(fan_)
                dl_ = {}
                for b in buses:
                    for l_ in b.J.lanes:
                        for k_ in (0, 1):
                            A_ = b.arr[k_]
                            if A_ is None:
                                continue
                            for leg in (b.E.legs_of.get(l_) or (l_,)):
                                if leg in A_.ball_of:
                                    for ref, k in fan_:
                                        if ref == A_.ref:
                                            d_ = math.dist(A_.bm.balls[A_.ball_of[leg]], arrays[ref].bm.balls[k])
                                            key_ = (b.name, str(l_))
                                            dl_[key_] = min(dl_.get(key_, d_), d_)
                stretches, epoch = [sorted(sorted(dl_, key=lambda kk: (dl_[kk], kk))[:EPOCH_LANES])], 0
                more_ = stretch_balls(buses, stretches[0]) & narrowed
                near_ = near_of(fan_)
                log(f'  {len(fan_)} costly ball(s) left (plane balls fanned out, teeth on a far face): the '
                    f'{len(stretches[0])} lanes nearest them searched again, {len(more_) + len(near_)} balls offered '
                    f'their whole menus')
                wide |= near_
                release(buses)
                m = mc = mh = sv = rows = gates = None
                whole(more_ | near_)
                epoch_balls = more_ | near_
                epoch_fan = list(fan_)
                fresh = {}
                in_epoch = True
                continue
            # ---- A DEAR REPAIR: its state dearer than the state before it by a change of layer or more. The repair
            # holds every lane it was not told of on its ends, and solves the few left for the objective: what a
            # lesson rules out of an end it can only answer with that lane's next way, across whatever the held lanes
            # leave it -- three lanes of a two-layer memory bus moved off a capacitor went from 167 crossings to 211,
            # the bus's own solve had no plan of them, and four nets shipped open where one had. The lanes the repair
            # freed -- the ones named first, then those it let go, then those that crossed one -- are searched again
            # as a stretch from that state, their balls on their whole menus: the lanes round them may move too
            if dear_:
                rk_ = lambda w_: 0 if w_ == 'named' else (2 if str(w_).startswith('crossed') else 1)    # noqa: E731
                free_ = sorted((rk_(w_), bn_, str(l_)) for bn_, ri_ in repair_info.items() for l_, w_ in ri_.items())
                if free_:
                    stretches, epoch = [sorted((bn_, l_) for _r, bn_, l_ in free_[:EPOCH_LANES])], 0
                    more_ = stretch_balls(buses, stretches[0]) & narrowed
                    log(f'  the repair\'s state is dearer than the state before it by {(v_now_ - dear_was_) / 1000:.1f} '
                        f'vias\' worth: the {len(stretches[0])} lanes it freed searched again, {len(more_)} balls '
                        f'offered their whole menus')
                    release(buses)
                    m = mc = mh = sv = rows = gates = None
                    whole(more_)
                    epoch_balls = set(more_)
                    epoch_fan = []
                    fresh = {}
                    in_epoch = True
                    continue
        # ---- CHECKED BY THE WHOLE SOLVE, as a bus's own ends were (whole_ends.joint_choose): each bus's ends given to
        # the solve under its strict rules, on the board as the model fanned it out; a refusal TEACHES -- the lanes no
        # change fits in held to one layer (and offered their whole menus), the crossings with no window held apart and
        # priced, the lanes' state refused -- and the model is solved again from the state it refused
        if '--search-only' in flags:
            # (a bench's: the search's state and, on two layers, its estimate -- no check, no field)
            est_ = None if not est_parts or any(v_ is None for v_ in est_parts.values()) else sum(est_parts.values())
            with open(os.path.join(out, 'search.json'), 'w', encoding='utf-8') as fh:
                json.dump({'estimate': est_, 'seed': awx_settings.get('BUS_SEED'), 'buses': est_parts}, fh)
            log('group ends: the search alone -- its estimate ' + ('none' if est_ is None else f'{est_} vias'))
            return
        if chk_try == 0 and tier_of and (not frm or at_point == '--at-start'):
            # (THE SEARCH'S END, kept for a bench of the checks FROM it -- --from=OUT --at-search: the state, the
            # menus as the search left them -- each bus ball narrowed to its first tiers and the move its lane holds,
            # as its last stretch read them -- and nothing yet taught. A check's repair is then asked of this state
            # in minutes, where the search that leads to it is most of the model's time)
            pickle.dump({'wide': set(wide), 'lined': set(lined), 'prev_bus': prev_bus, 'prev_oth': prev_oth,
                         'learned': copy.deepcopy(learned), 'cuts': dict(cuts),
                         'layers_want': copy.deepcopy(layers_want), 'searched': set(searched),
                         'feedback': feedback, 'hard': hard, 'oth_ban': set(oth_ban),
                         'held': dict(held), 'prev_x': {nm: sorted(x_) for nm, x_ in prev_x.items()},
                         'narrowed': set(narrowed), 'keep': kept_last},
                        open(os.path.join(out, 'search.pkl'), 'wb'))
        unplanned, taught, more, newly, n_taught, adopted = [], False, set(), False, 0, False
        no_room = {}
        fresh, progressed = {}, False
        for b in buses:
            t0 = time.time()
            L0_ = copy.deepcopy(learned.get(b.name) or {})         # (what it knew before this check: a trace's)
            # (asked first with a via end's layer the solve's to choose -- a refusal there is the ends' own, and what
            # it teaches below -- then as the bus's own step will ask it: under the group's ends each end's layer is
            # the model's, HELD (route_bus's VIA_ENDS 0). Asked the first way alone, a bus's ends had a plan at the
            # check and none in its step, 24 of its 54 nets open; held from the start, each check named other lanes
            # for their ends' layers. With a plan the first way and none the second, THE SOLVE'S LAYERS ARE TAKEN:
            # each lane's ends wanted on the layers its plan gave them, the model solved again -- the arrays' rules
            # the judge of two lanes on one line there -- and asked again)
            sl_ = os.path.join(out, 'checks', f'{b.name}.{chk_try + 1}.free.solve.log')
            with awx_settings.given(b.env):
                chk = bus_check(b, laid, RL, save=sl_, plans=check_plans)
            TR.rec('verdict', check=chk_try + 1, bus=b.name, mode='free', secs=round(time.time() - t0, 1),
                   solve_log=os.path.relpath(sl_, out), verdict=chk)
            if chk is not None and chk.get('plan') and FREE:
                # (GROUP_LAYERS=free: asked as its step will ask it -- its via ends' layers the solve's own but the
                # lanes its plan holds, `held_layers`)
                dif_ = sorted(l_ for l_, ls_ in (chk.get('ends_y') or {}).items()
                              if l_ in b.lay and [RL[q_] for q_ in b.lay[l_]] != list(ls_))
                by_ = collections.Counter(L_ for ls_ in (chk.get('ends_y') or {}).values() for L_ in ls_)
                log(f'  {b.name}: the solve\'s check: a plan, its ends\' layers the solve\'s -- {len(dif_)} lane(s) on '
                    f'other layers than the model\'s; its ends by layer: '
                    + ', '.join(f'{L_} {n_}' for L_, n_ in sorted(by_.items())) + f' ({time.time() - t0:.0f} s)')
                continue
            if chk is not None and chk.get('plan'):
                free_ = chk
                t1_ = time.time()
                sl_ = os.path.join(out, 'checks', f'{b.name}.{chk_try + 1}.held.solve.log')
                with awx_settings.given({**b.env, 'VIA_ENDS': '0'}):
                    chk = bus_check(b, laid, RL, save=sl_)
                TR.rec('verdict', check=chk_try + 1, bus=b.name, mode='held', secs=round(time.time() - t1_, 1),
                       solve_log=os.path.relpath(sl_, out), verdict=chk)
                if chk is not None and not chk.get('plan') and free_.get('ends_y'):
                    new_ = {l_: tuple(RL.index(L_) for L_ in ls_) for l_, ls_ in free_['ends_y'].items()
                            if l_ in b.lay and all(L_ in RL for L_ in ls_)}
                    dif_ = sorted(l_ for l_, qs_ in new_.items() if tuple(b.lay[l_]) != qs_)
                    if dif_:
                        layers_want.setdefault(b.name, {}).update(new_)
                        unplanned.append(b.name)
                        taught, n_taught, adopted = True, n_taught + len(dif_), True
                        log(f'  {b.name}: the solve\'s check: a plan with its ends\' layers the solve\'s, none with '
                            f'the model\'s held -- the solve\'s layers taken, {len(dif_)} lane(s) on other layers: '
                            + ', '.join(dif_[:12]) + (' ...' if len(dif_) > 12 else '') + f' ({time.time() - t0:.0f} s)')
                        continue
            if chk is None or chk.get('plan'):
                log(f'  {b.name}: the solve\'s check: ' + ('not asked' if chk is None else 'a plan')
                    + f' ({time.time() - t0:.0f} s)')
                continue
            if chk.get('status') == 'UNKNOWN' and not chk.get('lanes') and not chk.get('core'):
                # (NEITHER a plan NOR a refusal in the check's time: it has said nothing against these ends, and
                # they stand for the bus's own step, whose solve has the time the check has not -- as a bus's own
                # ends did. Read as "no plan, nothing to learn", a 53-lane bus's model stopped on a timeout)
                log(f'  {b.name}: the solve\'s check: UNDECIDED in its time [UNKNOWN] -- neither a plan nor a refusal; '
                    f'the ends stand for the step\'s own solve ({time.time() - t0:.0f} s)')
                continue
            unplanned.append(b.name)
            s_ = b.state
            log(f'  {b.name}: the solve\'s check: NO plan [{chk.get("status")}], core {chk.get("core")}, lanes '
                f'{chk.get("lanes")}'
                + (f' (the rules: {", ".join(f"{l_} {f_}" for l_, f_ in chk["rules"])})' if chk.get('rules') else '')
                + f', no room for a change {chk.get("nochange")} ({time.time() - t0:.0f} s)')
            # (NO ROOM AT THESE SIZES, and room on a rung below -- the group's escalation's, --rungs-below: a lane the
            # refusal NAMES that is too short between its two ends for a change of layer here and has room for one at
            # a rung's sizes (room_on_a_rung, on the solve's own measure of the lane), named so AGAIN: the model
            # answered the first -- other ends, the lanes round it moved -- and the solve names the same lane for the
            # same want. What stands in its way is the size of its vias, which a rung answers and the model's ends
            # cannot. Answered here all the same -- another state, the solve asked again -- a 38-lane bus 3 mm from
            # its array was refused at twelve checks running, a pair of it named so at seven of them, each repair four to
            # five minutes of a model half on its whole menus: 65 minutes, and no plan. Not on a lane named ONCE: a
            # bus whose second refusal named a lane so had its plan at the third check, at those sizes. Nor on a lane
            # short and not named: it holds one layer, and nothing says that is what the refusal is of)
            short_ = short_named(chk, s_, rungs_below)
            again_ = sorted(short_ & tight.get(b.name, set()))
            tight.setdefault(b.name, set()).update(short_)
            if again_:
                no_room[b.name] = again_
                continue
            # (each lane it names, end by end: its layer the model's, whether the solve may move it, where it may not)
            ve_, vb_ = chk.get('vend') or set(), {(l_, k_): ls_ for l_, k_, ls_ in chk.get('vban') or ()}
            for l_ in chk.get('lanes') or ():
                if l_ in s_:
                    log(f'    {l_}: ' + ', '.join(
                        f'{("tooth", "berth")[k_]} on {RL[b.lay[l_][k_]]} '
                        + (('the solve\'s to move' + (f', banned from {"/".join(vb_[(l_, k_)])}' if (l_, k_) in vb_ else ''))
                           if (l_, k_) in ve_ else 'fixed') for k_ in (0, 1))
                        + ('' if l_ not in (chk.get('nochange') or ()) else '; no room for a change'))
            if b.name not in lined:
                # (its pattern ends refused: its balls offered THE LINES)
                lined.add(b.name)
                taught = newly = True
                for A_ in b.arr.values():
                    ks_ = set() if A_ is None else {A_.ball_of[n_] for n_ in b.E.st['launch'] if n_ in A_.ball_of
                                                    and (A_.ref, A_.ball_of[n_]) not in wide}
                    if ks_:
                        n0_ = sum(len(A_.bm.menu[k]) for k in ks_)
                        with awx_settings.given(genv):
                            expand(bpcb, A_, ks_)
                        log(f'    {A_.ref}: {b.name}\'s {len(ks_)} balls offered their site\'s every line, {n0_} -> '
                            f'{sum(len(A_.bm.menu[k]) for k in ks_)} moves')
                # (...answered by THE REPAIR on the lines: the lanes the refusal named, and those it found no room for
                # a change in, free on their wider menus, every other lane held where the search put it. Searched
                # whole again on the new tier, each bus's first refusal cost a second search as long as the first --
                # five to ten minutes of a model's fifteen -- to move the few lanes it named)
                fresh[b.name] = {l_ for l_ in chk.get('lanes') or () if l_ in s_} | \
                    {l_ for l_ in chk.get('nochange') or () if l_ in s_}
                # (...AND TAUGHT what the refusal says, as every later one: its lessons are of the ends refused, by
                # their signatures, and stand on the lines' menus. Offered the lines and taught nothing, two buses
                # were each refused a second time for the very lanes and rules of the first -- a check and a repair
                # of four minutes each -- before anything was held)
            # (...a refusal that names no lane of a rule still teaches the lanes NO CHANGE FITS IN: read only with
            # the lanes named, such a refusal -- four lanes with no room for a change, none named -- taught nothing,
            # and the model stopped with no plan and nothing to learn)
            if chk.get('lanes') or [l_ for l_ in chk.get('nochange') or () if l_ in s_]:
                L_ = learned.setdefault(b.name, {'nochg': set(), 'nocross': set(), 'nogoods': [], 'xl': {},
                                                 'hands': set(), 'endlay': []})
                # (NO ROOM FOR A CHANGE is a lane's LENGTH's doing where its own WINDOW for one is empty (the check's
                # `short`): of every such lane -- named by the refusal or not -- the longest of its kind, and from
                # here no lane of that kind as short changes layer, on any ends: lane_run. Learned a lane at a time,
                # a bus with ten lanes across a 3 mm gap between its two parts was refused for one or two of them at
                # each of six checks. NOT a lane the refusal names for its change's room AMONG ITS CROSSINGS: that is
                # those crossings' doing, and the lane is held to one layer for it (below) -- read as its length's,
                # a lane 31 mm from its tooth to its berth taught that none of its kind changes layer, three checks
                # banned a change on 1400 tooth options of 48 lanes, and the model had no state)
                rm_ = L_.setdefault('room', {})
                rm_new = False
                for l_ in chk.get('short') or ():
                    if l_ in s_:
                        kd_, ln_ = lane_run(b, l_, s_[l_][0], s_[l_][1])
                        if ln_ > rm_.get(kd_, -1e9) + 1e-6:
                            rm_[kd_], rm_new = ln_, True
                el_new_ = False
                for l_, k_ in chk.get('endlay') or ():
                    e_ = (l_, k_, we._opt_sig((b.E.T, b.E.B)[k_][l_][s_[l_][k_]]), b.lay[l_][k_]) if l_ in s_ else None
                    if e_ is not None and e_ not in L_['endlay']:
                        L_['endlay'].append(e_)
                        el_new_ = True
                # (...and, of a lane the refusal NAMES, each end's layers the copper beside its run BANS it (the
                # solve's own list, whatever layer the model had it on): that end not on those layers again. Named
                # for "vban" alone, the model learned the lane and not which layers, and gave it another banned one)
                vb_n = 0
                for l_, k_, ls_ in chk.get('vban') or ():
                    if l_ in s_ and l_ in chk['lanes']:
                        for L_b in ls_:
                            if L_b in RL:
                                e_ = (l_, k_, we._opt_sig((b.E.T, b.E.B)[k_][l_][s_[l_][k_]]), RL.index(L_b))
                                if e_ not in L_['endlay']:
                                    L_['endlay'].append(e_)
                                    el_new_, vb_n = True, vb_n + 1
                # (no room for a change: the solve's own word for it, and a lane it NAMES that changes layer -- named
                # for all its rules as one, and what it cannot hold with a change it is first asked without. A pair
                # 3 mm from its tooth to its berth, a change between them, was named at four checks running, its window
                # for the change never empty, and came back each time with one end moved and the change kept)
                # (...but not the lanes of a refusal read from its CROSSINGS alone (the check's `crossing`): what it
                # says is that those crossings have no windows together, and they are not all crossed again below --
                # held to one layer besides, a lane from a surface end on one face to a part on the other had no end
                # left, and its bus's own model no state)
                nochg_ = {l_ for l_ in chk.get('nochange') or () if l_ in s_} | \
                    ({l_ for l_ in chk['lanes'] if l_ in s_ and b.lay[l_][0] != b.lay[l_][1]}
                     if not chk.get('crossing') else set())
                n_taught += len((nochg_ - L_['nochg']) | {l_ for l_ in chk['lanes'] if l_ in s_})
                new_ = bool(chk.get('lanes')) or bool(nochg_ - L_['nochg']) or rm_new
                fresh[b.name] = {l_ for l_ in chk['lanes'] if l_ in s_} | nochg_
                # (a refusal that teaches a LANE something new -- held to one layer, to its hands, an end off its
                # layer -- is progress, of which there is only so much: it is none of the tries)
                # (each core of crossings: not all of them again -- whole_ends._joint_model)
                cores_ = {we.core_of(c_) for c_ in chk.get('cores') or [chk.get('core') or ()] if c_}
                progressed = progressed or el_new_ or rm_new or bool(nochg_ - L_['nochg']) or \
                    bool({l_ for l_ in chk.get('hands') or () if l_ in s_} - L_['hands']) or \
                    bool(cores_ - L_['nocross'])
                L_['nochg'] |= nochg_
                L_['hands'] |= {l_ for l_ in chk.get('hands') or () if l_ in s_}
                L_['nocross'] |= cores_
                for k_ in chk.get('core') or ():        # (its core's crossings priced, doubled when named again)
                    k_ = frozenset(k_)
                    L_['xl'][k_] = 2 * L_['xl'][k_] if k_ in L_['xl'] else we.JOINT_LEARN
                at_ = chk.get('at') or {}
                for g_ in chk.get('groups') or [list(chk['lanes'])]:
                    ng_ = {l_: tuple(None if at_.get(l_) not in (None, k_) else
                                     we._opt_sig((b.E.T, b.E.B)[k_][l_][s_[l_][k_]]) for k_ in (0, 1))
                           for l_ in g_ if l_ in s_}
                    if ng_:
                        L_['nogoods'].append(ng_)
                taught = taught or new_
                # (a lane held to one layer is offered its whole menu: on the lines it has its pattern's one via site,
                # and two such lanes side by side had no end each on one layer -- the model no state, its rules naming
                # 39 lanes for it)
                for A_ in b.arr.values():
                    if A_ is not None:
                        more |= {(A_.ref, A_.ball_of[leg]) for l_ in nochg_
                                 for leg in (b.E.legs_of.get(l_) or (l_,)) if leg in A_.ball_of} - (wide - narrowed)
                if L_.get('room'):
                    log(f'  {b.name}: no change of layer from here on a lane as short as: '
                        + ', '.join(f'{("a single", "a pair")[kd_[0] - 1]} {kd_[1]} to {kd_[2]} {ln_:.2f} mm'
                                    for kd_, ln_ in sorted(L_['room'].items())))
                log(f'  {b.name}: held from here: {len(L_["nochg"])} lane(s) to one layer, {len(L_["hands"])} pair(s) '
                    f'to their hands, {len(L_["endlay"])} end(s) off their layer, {len(L_["nocross"])} core(s) of '
                    f'crossings not all again, {len(L_["nogoods"])} refused state(s)')
                # (WHAT THIS CHECK TAUGHT, a trace's: each lesson new since it was asked, and the lanes the repair
                # is to free for it)
                new_l_ = {}
                for k_, v_ in L_.items():
                    v0_ = L0_.get(k_)
                    if isinstance(v_, set):
                        d_ = v_ - (v0_ or set())
                    elif isinstance(v_, list):
                        d_ = [x_ for x_ in v_ if x_ not in (v0_ or [])]
                    elif isinstance(v_, dict):
                        d_ = {kk_: vv_ for kk_, vv_ in v_.items() if (v0_ or {}).get(kk_) != vv_}
                    else:
                        d_ = v_ if v_ != v0_ else None
                    if d_:
                        new_l_[k_] = d_
                TR.rec('lessons', check=chk_try + 1, bus=b.name, new=new_l_, free_next=sorted(fresh.get(b.name, ())),
                       totals={k_: len(v_) if hasattr(v_, '__len__') else v_ for k_, v_ in L_.items()})
        if no_room:
            for nm_, ls_ in sorted(no_room.items()):
                log(f'group ends: NO ROOM AT THESE SIZES for {nm_} after {chk_try + 1} check(s) -- {len(ls_)} lane(s) '
                    f'named again, the model\'s answer between, as too short between their two ends for a change of '
                    f'layer here, with room for one on a rung below: ' + ', '.join(ls_[:12])
                    + (' ...' if len(ls_) > 12 else ''))
            bus_events.emit('ends', 'no_room', group=sref, bus=None, check=chk_try + 1, buses=sorted(no_room))
            no_plan = unplanned
            break
        if not unplanned:
            M_ = lay_and_measure(laid, wayouts)
            n_held = len(held)
            n_ban = teach_laid(M_['items'], buses, M_['moved'], laid, wayouts) \
                if M_['items'] and laid_try < we.JOINT_TRIES else 0
            # (...and on a field that breaks none of the board's rules, THE PADS IT WALLS IN: each one's way out kept
            # -- teach_walled -- answered by the same repair)
            if not M_['items'] and not M_['bad'] and laid_try < we.JOINT_TRIES:
                n_ban += teach_walled(buses, laid, wayouts)
            TR.rec('field', check=chk_try + 1, drc_field=M_['d_f'], drc_bare=M_['d_b'], items=M_['items'][:60],
                   moved=M_['moved'], lessons=n_ban, held_parts=sorted(held))
            if not n_ban:
                break
            # (a lesson of the laid field is answered by THE REPAIR, as a check's refusal is: every lane held on its
            # ends but the ones it named, and whatever stands on what it ruled out let go by the rules themselves --
            # a held part, another ball's choice name no lane. With none named the model was solved whole again, a
            # start and a search of five to ten minutes for one capacitor)
            for b in buses:
                fresh.setdefault(b.name, set())
            laid_try += 1
            bus_events.emit('ends', 'field', group=sref, bus=None, check=chk_try + 1, drc=M_['d_f'], drc_bare=M_['d_b'],
                            lessons=n_ban)
            log(f'  the field as laid, measured: DRC {M_["d_f"]} (the bare board {M_["d_b"]}) -- {n_ban} lesson(s): '
                f'{n_ban - (len(held) - n_held)} end(s) or other ball\'s choice(s) at fault ruled out'
                + (f', {len(held) - n_held} part(s) the nudge could not clear held where they came, no via, track or '
                   f'way out on their pads again ({", ".join(sorted(held))} held in all)' if len(held) > n_held else '')
                + '; the model solved again')
            for it in M_['items'][:12]:
                log('    ' + ', '.join(f'{k_} {it[k_]}' for k_ in ('type', 'layer', 'net1', 'net2', 'pad_ref')
                                       if it.get(k_) is not None)[:200])
            release(buses)
            m = mc = mh = sv = rows = gates = None
            continue
        # (...nor is a pass that took the solve's layers: its ends have a plan, and it has JOINT_TRIES of its own)
        adopts += adopted
        spent += not (newly or adopted or progressed)
        if not taught or spent == we.JOINT_TRIES or adopts > we.JOINT_TRIES:
            bus_events.emit('ends', 'no_plan', group=sref, bus=None, check=chk_try + 1, buses=list(unplanned))
            log(f'group ends: NO PLAN from the whole solve for {", ".join(unplanned)} after {chk_try + 1} check(s)'
                + ('' if taught else ' -- and nothing to learn from the last'))
            no_plan = unplanned
            break
        release(buses)
        m = mc = mh = sv = rows = gates = None
        if more:
            log(f'  check {chk_try + 1}: the {len(more)} balls of the lanes held to one layer offered their whole menu')
            wide |= more
            whole(more)
        log(f'  check {chk_try + 1}: {", ".join(unplanned)} refused -- the model solved again')
        bus_events.emit('ends', 'check', group=sref, bus=None, check=chk_try + 1, refused=list(unplanned))
    # (every lane's way out of its exit, arrays' and row parts': a bench's, to lay the field's ties again)
    rep['ways_out'] = [[n_, list(e_), d_, L_] for n_, e_, d_, L_ in wayouts + row_ways]
    json.dump(rep, open(os.path.join(out, 'group.json'), 'w'), indent=1, default=str)
    TR.rec('done', checks=chk_try + 1, no_plan=no_plan or [], secs=round(time.time() - t_all, 1))
    # (the run as it ends, for a run FROM it: --from)
    pickle.dump({'wide': set(wide), 'lined': set(lined), 'prev_bus': prev_bus, 'prev_oth': prev_oth, 'learned': learned,
                 'cuts': dict(cuts), 'layers_want': layers_want, 'searched': set(searched),
                 'feedback': feedback, 'hard': hard, 'oth_ban': set(oth_ban),
                 'held': dict(held), 'prev_x': {nm: sorted(x_) for nm, x_ in prev_x.items()},
                 'narrowed': set(narrowed), 'below': v_last, 'sizes': list(sizes_now)},
                open(os.path.join(out, 'resume.pkl'), 'wb'))
    if no_room:
        # (...the group's escalation's to answer, from this end -- the last state and what the checks before this one
        # taught: no field is laid on ends the solve refused)
        bus_events.emit('ends', 'done', group=sref, bus=None, secs=round(time.time() - t_all), refused=[],
                        no_plan=list(no_plan or []), no_room=sorted(no_room))
        log(f'group ends: {time.time() - t_all:.0f} s in all -- no answer at these sizes: a rung below')
        raise SystemExit(NO_ROOM)
    if M_ is None or no_plan:
        M_ = lay_and_measure(laid, wayouts)
    log(f'  the arrays fanned out as chosen: {M_["nt"]} tracks and {M_["nv"]} vias -> '
        f'{os.path.join(out, "shared_field.kicad_pcb")}')
    for ln in M_['caps']:
        log('    caps: ' + ln[:200])
    field_bad = M_['bad']
    bus_events.emit('ends', 'field', group=sref, bus=None, drc=M_['d_f'], drc_bare=M_['d_b'], bad=bool(field_bad),
                    tracks=M_['nt'], vias=M_['nv'])
    log(f'  the field as laid, measured: DRC {M_["d_f"]} (the bare board {M_["d_b"]})'
        + ('' if not field_bad else ' -- THE FANOUT BREAKS THE BOARD\'S RULES:' if M_['d_f'] is not None
           and M_['d_b'] is not None else ' -- no verdict from check_drc'))
    for it in M_['items'][:40]:
        log('    ' + ', '.join(f'{k_} {it[k_]}' for k_ in ('type', 'layer', 'net1', 'net2', 'pad_ref')
                               if it.get(k_) is not None)[:260])
    # ---- THE FIELD'S TIES: the power pads beneath the group's parts tied before a lane is planned (field_ties)
    if not field_bad and not no_plan:
        import field_ties
        fpath_ = os.path.join(out, 'shared_field.kicad_pcb')
        parts_ = {sref} | {d_ for cap in caps.values() for d_ in (cap['st'].get('dests') or [cap['st']['dref']])}
        shutil.copy(fpath_, fpath_ + '.untied')
        with awx_settings.given(genv):
            # (the ways out as far as the model reads an exit's front, EXIT_REACH: at the nudge's two bars a tie left
            # an array round the end of a tooth's way out and back in, across the lane's own run on its layer)
            T_ = field_ties.lay(fpath_, parts_, wayout_runs(wayouts + row_ways, bpcb.board_info.copper_layers,
                                                            reach=EXIT_REACH), log=log)
            d_t = field_drc(fpath_, 'tied')[0] if T_ and (T_['segments'] or T_['vias']) else M_['d_f']
        if T_ is None:
            log('  field ties: no pad on a poured net beneath the group\'s parts')
        elif d_t is None or M_['d_f'] is None or d_t > M_['d_f']:
            # (ties that break a rule are no ties: the field goes on as it was measured, and the run says so)
            shutil.copy(fpath_ + '.untied', fpath_)
            log(f'  field ties: {T_["segments"]} tracks and {T_["vias"]} vias laid for {len(T_["tied"])} pad(s) measured '
                f'DRC {d_t} against the field\'s {M_["d_f"]} -- TAKEN OUT AGAIN, the field goes on untied '
                f'(field_drc_tied.json)')
        else:
            # (...and a tie that WALLS IN a pad later routes must reach, the untied field leaving it free, is no tie
            # either: taken out again (tie_walls) -- the pad's way kept, and the tie the chain's to lay)
            from protected_nets import read_protected_nets, pro_path_for_board
            with awx_settings.given(genv):
                out_s, out_v, named_ = tie_walls(fpath_ + '.untied', fpath_, T_, sorted(arrays), sorted(
                    {cap['st']['byname'][n_][1].name for cap in caps.values() for n_ in cap['st']['launch']
                     if n_ in cap['st']['byname']}), read_protected_nets(pro_path_for_board(board)))
            if out_s or out_v:
                import ship_vias
                from kicad_writer import add_tracks_and_vias_to_pcb
                T_ = dict(T_, tracks=[t_ for i_, t_ in enumerate(T_['tracks']) if i_ not in out_s],
                          vs=[v_ for j_, v_ in enumerate(T_['vs']) if j_ not in out_v])
                shutil.copy(fpath_ + '.untied', fpath_)
                with contextlib.redirect_stdout(io.StringIO()):
                    add_tracks_and_vias_to_pcb(fpath_, fpath_, [dict(t_) for t_ in T_['tracks']],
                                               [dict(v_) for v_ in T_['vs']], [],
                                               net_id_to_name={i_: n_.name for i_, n_ in bpcb.nets.items()})
                ship_vias.stamp(fpath_, 'group ends')
                T_['segments'], T_['vias'] = len(T_['tracks']), len(T_['vs'])
                # (the pads the copper kept still reaches: field_ties' own test of a piece on a pad)
                import types
                kept_ = [('s', types.SimpleNamespace(start_x=t_['start'][0], start_y=t_['start'][1], end_x=t_['end'][0],
                                                     end_y=t_['end'][1], layer=t_['layer'], net_id=t_['net_id']))
                         for t_ in T_['tracks']] + [('v', types.SimpleNamespace(x=v_['x'], y=v_['y'], net_id=v_['net_id']))
                                                    for v_ in T_['vs']]
                T_['tied'] = sorted({(p_.component_ref, p_.pad_number) for p_ in T_['pads'] for k_, o_ in kept_
                                     if o_.net_id == p_.net_id and field_ties._at(k_, o_, p_)})
            if named_:
                log('  field ties: walling a pad in -- ' + '; '.join(f'{k_}: {w_}' for k_, w_ in named_)
                    + (f' ({len(out_s)} track(s) and {len(out_v)} via(s) not kept)' if out_s or out_v else ''))
            json.dump({'tracks': T_['tracks'], 'vias': T_['vs']}, open(os.path.join(out, 'field_ties.json'), 'w'))
            log(f'  field ties: {len(T_["tied"])} of the {len(T_["pads"])} pad(s) on a poured net beneath '
                f'{", ".join(sorted(parts_))} tied before the lanes -- {T_["segments"]} tracks, {T_["vias"]} vias, '
                f'{T_["secs"]:.0f} s; DRC {d_t}'
                + (f'; {T_["strayed"]} tie(s) that left their part\'s fan or the outer layers not kept'
                   if T_.get('strayed') else ''))
        os.remove(fpath_ + '.untied')
    if '--source-field' in flags:
        # (a bench's: the shared array alone fanned out, each bus's own step to fan out its destination from it)
        with awx_settings.given(genv):
            write_field(board, os.path.join(out, 'source', 'shared_field.kicad_pcb'), [p_ for p_ in laid if p_[4] == sref],
                        locked=sorted(held))
    # ---- CHECKED: each bus's ends read by the whole route's own frame, as its step's fanout audit will read its
    # board -- an end the route would refuse is the model's to answer, here, not a bus step's to find
    bad = []
    for b in buses:
        with awx_settings.given(b.env):
            why = frame_check(b, laid, RL)
        if why:
            bad.append(f'{b.name}: {why}')
    for ln in bad:
        log('  REFUSED by the whole route\'s frame -- ' + ln[:400])
    bus_events.emit('ends', 'done', group=sref, bus=None, secs=round(time.time() - t_all), refused=bad,
                    no_plan=list(no_plan or []))
    log(f'group ends: {time.time() - t_all:.0f} s in all -> {os.path.join(out, "group.json")}'
        + ('' if bad else '; every bus\'s ends read by the whole route\'s frame'))
    if bad:
        raise SystemExit(4)
    if no_plan:
        raise SystemExit(5)         # (ends the whole solve has no plan for are no answer)
    if field_bad:
        raise SystemExit(6)         # (a field that breaks the board's rules as laid is no answer either)
    if '--plans' in flags:
        # (a bench's: each bus's FIRST PLAN from the model's own ends, teeth and berths, no bus step between)
        for b in buses:
            with awx_settings.given({**b.env, 'VIA_ENDS': '0'}):
                line, secs = first_plan(b, laid, RL)
            log(f'  {b.name}: its first plan from the model\'s ends ({secs:.0f} s) -- {line[:330]}')


if __name__ == '__main__':
    main()
