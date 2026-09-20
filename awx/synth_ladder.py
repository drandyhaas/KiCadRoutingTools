#!/usr/bin/env python3
"""synth_ladder.py -- run the chain over generated cases and grade it
against their KNOWN ANSWERS (#622).

    python3 synth_ladder.py --batch first          # 3 cases at K=8, the proof
    python3 synth_ladder.py --batch b1             # patterns x K, open arrays
    python3 synth_ladder.py --batch b2             # the same, interiors CLOSED
    python3 synth_ladder.py --batch b3             # the corridor-obstacle ladder
    python3 synth_ladder.py --batch b1 --only sorted_k8,blocks_k15
    python3 synth_ladder.py --batch b1 --regrade   # no build, no chain

Per case: `synth_bus.py` writes the board and its truth, `make_bench.py`
prepares it the way it prepares a corpus pair (source fanned out, DRC
floor stamped, coherent ladder written), `chain_k.sh` runs plan + fanout +
braid, and the result is graded against the KNOWN ANSWER:

    PLANNER gap  = the braid's plan-implied count   - the known optimum
    BRAID   gap  = the routed vias                  - the plan-implied count
    DP      gap  = the routed vias                  - the known optimum
    DETOUR       = routed copper mm / the straight-line lower bound

plus `open` (completion), `drc`, and `inband/offered` -- the braid's own
first-attempt in-band lane count, read from its `lanes: a/b routed` lines,
because "all lanes in band" is the objective the planner is actually
optimising and a case can route perfectly while leaving it.

The optimum is re-derived FROM THE BOARD (`truth_from_board`) rather than
trusted from the generator's sidecar, so a case whose ladder does not hand
the chain the K that was asked for -- `coherent_nets` counts whole RIVERS,
so it can return fewer -- is still graded against its own real problem;
`k_asked` and `k_real` are printed side by side and a disagreement is a
harness bug, not a routing result.

THE THREE COLUMNS THAT SAY WHETHER A NUMBER MEANS ANYTHING
----------------------------------------------------------
* `lb` / `opt` / `dp`: the LIS lower bound, the whole-lane optimum, and
  the exact optimum over pages AND mid-channel changes. `dp` is the one
  to grade against; `opt` is blank when the crossing graph has an odd
  cycle and `dp` is blank past the DP's K cap.
* `thru`: lanes with copper INSIDE an array. The optimum is
  channel-confined, so a case with `thru > 0` had a cheaper topology
  available than the model describes and its negative gap is NOT a win.
  The b1/b2 pair exists exactly to separate this: same cases, b2 with the
  array interiors closed by fat unused balls.
* `slot_cap`: on an obstacle case, how many lanes can physically pass the
  obstruction by counting. Without it `open=4` cannot be read -- it is a
  router defect if the lanes fit and a correct refusal if they do not.

Times are printed as an OBSERVATION. Nothing here budgets on a clock.
"""
from __future__ import annotations

KRT_TOOL = {'scope': [], 'kind': 'driver'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import synth_bus as sy                                        # noqa: E402

PY = sys.executable
SRC, DST = 'SU1', 'SD1'

# ---------------------------------------------------------------------------
# The batches. A case is geometry + a pin pattern; nothing here names a
# board, a net or a face, and every field is a generator argument.
# ---------------------------------------------------------------------------


def case(k, pattern, **kw):
    c = dict(k=k, pattern=pattern, seed=0, blocks=2, inversions=None,
             cols=4, rows=None, depth=1, gap=12.0, dst_rot=0.0, caps=0,
             pad=0.4, pad_inner=None, margin_y=6.0, fanout_layers='F.Cu',
             obstacle_w=0.0, obstacle_h=0.0, obstacle_x=0.5, obstacle_y=0.0,
             row_offset=0, pairs=0)
    c.update(kw)
    bits = [pattern, f'k{k}']
    for key, dflt in (('seed', 0), ('blocks', 2), ('gap', 12.0), ('depth', 1),
                      ('caps', 0), ('dst_rot', 0.0), ('cols', 4),
                      ('pad', 0.4), ('margin_y', 6.0), ('row_offset', 0)):
        if c[key] != dflt:
            bits.append(f'{key[:3]}{c[key]:g}' if isinstance(c[key], (int, float))
                        else f'{key[:3]}{c[key]}')
    if c['pad_inner']:
        bits.append(f'pi{c["pad_inner"]:g}')
    if c['inversions'] is not None:
        bits.append(f'inv{c["inversions"]:g}')       # 0 is a real value here
    if c['obstacle_h']:
        bits.append(f'obs{c["obstacle_w"]:g}x{c["obstacle_h"]:g}'
                    + (f'y{c["obstacle_y"]:g}' if c['obstacle_y'] else '')
                    + (f'x{c["obstacle_x"]:g}' if c['obstacle_x'] != 0.5 else ''))
    if c['fanout_layers'] != 'F.Cu':
        bits.append('fb')
    if c['pairs']:
        bits.append(f'pr{c["pairs"]}')
    c['tag'] = '_'.join(bits)
    return c


BATCHES = {
    # the proof: three planted optima at one K, every one exact
    'first': [case(8, 'sorted'), case(8, 'blocks'), case(8, 'interleave')],
    # DIFFERENTIAL PAIRS (2026-09-20): two pairs among sixteen, balls
    # neighbouring at both ends; the chain runs with BRAID_PAIRS=1
    # PLAN_PAIRS=1 (override in the environment for the pairs-off arm)
    'pairs': [case(16, 'sorted', pairs=2), case(16, 'blocks', pairs=2),
              case(16, 'interleave', pairs=2)],

    # the first real batch: the patterns crossed with K, plus the axes
    # that are geometry rather than pattern (gap, rotation, foreign
    # parts, escape depth, a two-layer source fanout)
    'b1': (
        [case(k, p) for k in (8, 15, 28)
         for p in ('sorted', 'blocks', 'interleave', 'riffle', 'reversed')]
        + [case(15, 'riffle', seed=s) for s in (1, 2)]
        + [case(15, 'blocks', blocks=3)]
        + [case(15, 'interleave', gap=6.0)]
        + [case(15, 'interleave', dst_rot=180.0)]
        + [case(15, 'interleave', caps=6)]
        + [case(15, 'interleave', depth=2)]
        + [case(15, 'interleave', fanout_layers='F.Cu,B.Cu')]
    ),
}

# b2 is b1 with the array interior CLOSED to F traffic, which is what makes
# the channel-confined optimum an optimum (see synth_bus --pad-inner, and
# the README section). Same cases, so the two tables are paired.
BATCHES['b2'] = [case(**{**{k: v for k, v in c.items() if k != 'tag'},
                         'pad_inner': 0.6}) for c in BATCHES['b1']]

# b3: the OBSTACLE ladder. A through-hole part sits mid-channel and gets
# taller, so the bus has to fan in past it through a shrinking slot at
# each end -- parallel bends at the lane pitch, which is the geometry the
# session-11 island-stack finding says goes infeasible. The optimum
# printed is the CLEAR-CHANNEL one, so the excess over it is the price of
# the obstruction, and `open` is whether the chain got through at all.
# Every case is `pad_inner=0.6` (the interior closed) so a lane cannot
# answer the obstacle by going through an array instead.
#
# Room past the obstacle, for K=15 (17 rows, margin 6): the outline is
# +-12.4 mm, so h=12 leaves 6.4 mm each side, h=18 leaves 3.4, h=20
# leaves 2.4 and h=22 leaves 1.4 -- against the 0.2 mm lane pitch, 15
# lanes need 3.0 mm of width split between the two slots.
BATCHES['b3'] = (
    [case(15, p, pad_inner=0.6, obstacle_w=2.0, obstacle_h=h)
     for p in ('sorted', 'interleave')
     for h in (12.0, 18.0, 20.0, 22.0)]
    # K=8 -- and the h=18 row is a NEGATIVE CONTROL, deliberately kept: the
    # K=8 article is shorter (10 rows), so an 18 mm blocker SEALS the
    # channel and `slot_cap` comes out 0. The chain leaves 8 nets open and
    # that is a correct refusal, not a defect. It is the proof that
    # `slot_cap` discriminates -- without a case where the answer is
    # "impossible", an `open` count cannot be read at all.
    + [case(8, 'interleave', pad_inner=0.6, obstacle_w=2.0, obstacle_h=h)
       for h in (12.0, 18.0)]
    + [case(15, 'blocks', pad_inner=0.6, obstacle_w=2.0, obstacle_h=18.0)]
    + [case(15, 'riffle', pad_inner=0.6, obstacle_w=2.0, obstacle_h=18.0)]
    # a WIDE obstacle at the same height: length of the squeeze, not depth
    + [case(15, 'interleave', pad_inner=0.6, obstacle_w=6.0, obstacle_h=18.0)]
    # and one OFF-CENTRE, so the two slots are unequal and the fan-in has
    # to be asymmetric -- an equal split is then the wrong answer
    + [case(15, 'interleave', pad_inner=0.6, obstacle_w=2.0, obstacle_h=18.0,
            obstacle_y=4.0)]
)


# b4: the BUNDLE ladder -- the room an END-OF-FACE CLIMB needs, and its
# negative control. Every case above fills the facing column from the first
# non-corner row to the last (`rows` is derived as K + 2), so a launch has
# NOWHERE along the face to move to: the end-of-face climb
# (fanout_from_plan SRC_CLIMB_END) and the group move built on it
# (pages_first PLAN_PAGES_GROUP) cannot exist on any b1/b2/b3 case, and the
# harness is inert for them however they are flagged. `row_offset=3` leaves
# three free ball rows at EACH end of the facing column and changes nothing
# else -- the permutation, the crossing graph and all three planted answers
# are identical -- so each pair below (offset 3 against offset 0) measures
# exactly what the room buys, on a case whose optimum is known.
#
# What the optimum is NOT, and it is worth saying plainly: **a clear
# two-layer channel cannot make the group move pay.** A crossing costs its
# lane 2 vias (one at each end, both pads being on F), and a climb that
# re-orders a launch costs the same 2 -- it dives to the other layer to run
# along the face and must come back for its F pad. So on these cases the
# answer with the room is the answer without it, and what the pair grades is
# that the chain still REACHES the optimum when the extra moves are on the
# menu -- a change detector for the group machinery, not a claim that it
# wins here. The climb's value is congestion relief, which is why the
# obstacle rows are here too: with a blocker mid-channel the excess over the
# clear-channel optimum is the measure, and re-ordering the fan-in at the
# face is a thing that can reduce it.
BATCHES['b4'] = (
    [case(k, p, row_offset=o, pad_inner=0.6)
     for k in (15, 28)
     for p in ('blocks', 'reversed', 'interleave')
     for o in (3, 0)]
    # and the congested pair: the same blocker as b3's ladder, where the
    # excess over the clear-channel optimum is what a re-ordered fan-in can
    # actually move
    + [case(15, p, row_offset=o, pad_inner=0.6, obstacle_w=2.0, obstacle_h=18.0)
       for p in ('blocks', 'interleave')
       for o in (3, 0)]
)


# b5: the MODEL ladder -- the cases where the planner's own two-page model
# cannot express the answer, which b1..b4 do not contain.
#
# Measured before this batch existed (`synth_bus --self-test`, and the sweep
# in the README): on `sorted`, `blocks`, `interleave`, `riffle` and even
# `reversed` the two-page model's best plan costs EXACTLY the optimum, so
# every one of those cases grades the search and the braid and says nothing
# about the model. The model error lives on IRREGULAR permutations, where
# the model swims lanes it cannot page and, at a swim price of 100 vias
# against a swimmer's true cost of 2, buys paged lanes at any price:
# `shuffle` K=12 seed 0 is +14 vias (34 against 20), K=15 +10, K=18 +4.
#
# So this batch is `shuffle` at several seeds and K -- the only family whose
# crossing graph is a general permutation graph -- with the interiors CLOSED
# so the channel-confined optimum really is one, plus the INVERSION
# dose-response the first batches have no curve for (the patterns are
# corners of the space, not a sweep). `inversions` is a controlled crossing
# count at fixed K, so `model_err` against it is the shape of the defect
# rather than one number.
def _no_duplicate_tags():
    """A case's tag is its FILENAME, so two cases sharing one overwrite each
    other's boards and the table reports the second twice. That is silent --
    the run looks complete -- so it is checked at import. It has bitten once
    already: `inversions` was not in the tag, which collapsed b5's whole
    dose-response onto one name."""
    for b, cs in BATCHES.items():
        seen = {}
        for c in cs:
            if c['tag'] in seen:
                raise SystemExit(
                    f'batch {b}: two cases share the tag {c["tag"]!r} -- they '
                    f'would write the same files. Differing keys: '
                    + ', '.join(sorted(k for k in c
                                       if k != 'tag' and c[k] != seen[c['tag']][k])))
            seen[c['tag']] = c


BATCHES['b5'] = (
    [case(k, 'shuffle', seed=s, pad_inner=0.6)
     for k in (12, 15, 18)
     for s in (0, 1, 2)]
    # the dose-response: K fixed, crossings swept from none to nearly all
    # (K=12 has 66 pairs, so 0 is `sorted` and 66 is `reversed`)
    + [case(12, 'shuffle', seed=0, inversions=v, pad_inner=0.6)
       for v in (0, 6, 12, 22, 33, 44, 55, 66)]
)


# b6: CALIBRATED TO THE BENCH, for the rungs that matter (K41, K51).
#
# `b5` measured that a swim price of 2 beats the shipped 100 by 20 vias --
# and on the bench it LOSES on all three rungs. The reason is a property of
# b5 and not of the price: every b5 case is `pad_inner=0.6`, interiors
# CLOSED, so a lane cannot leave the channel and the case is confined to the
# homotopy class the truth model describes. **The bench is not.** Measured on
# the shipped boards, lanes with copper past the deepest bus ball at one end:
# 5 of 28 at K28, 8 of 35 at K35, 8 of 41 at K41 -- a fifth of the bus goes
# through or around an array, which un-crosses pairs for free. It is why K28
# routes 34 against a channel bound of 44.
#
# So a batch meant to PREDICT the bench must leave the interiors OPEN, and
# must be at the bench's own K. Two things make this a calibration and not a
# fit to one board: the permutation family is `shuffle`, chosen because the
# bench's own bus IS statistically a uniform random permutation (inversions
# 195/302/399 against a random mean of 189/298/410 at K28/35/41, LIS within
# one standard deviation at every rung), and nothing here copies a ball map,
# a net name or a pitch.
#
# `depth=2` draws the bus from two columns, so the teeth are a real escape
# field with some launches on B -- the bench carries 6 teeth on B at K41 and
# 8 at K51, which `depth=1` cannot produce.
#
# CALIBRATION, measured before the batch was run (8 seeds a rung, against the
# bench's own numbers) on the four quantities the PLANNER consumes:
#
#              crossings      LIS      paged by the model   swimmers
#   K28      177 / 195      8 / 7          15 / 13          13 / 15
#   K35      303 / 302      9 / 9          16 / 15          19 / 20
#   K41      406 / 399     10 / 9          18 / 16          23 / 25
#   K51      664 / 539     10 / 11         19 / 20          32 / 28
#
# K41 is a tight match on all four, which is the rung this batch is for.
#
# K51 is NOT, and the obvious repair does not work: the bench's K51 bus is
# more ordered than a uniform shuffle (539 crossings against 655), but pinning
# the count with `--inversions 539` buys the crossings at the cost of the
# structure -- it lands at **LIS 6.2 +- 0.6 where the bench is 11**, because
# the generator's accept-while-it-moves-toward-the-target walk does not sample
# uniformly among permutations of that inversion count. LIS is the quantity
# that sets both the floor (`2*(K - LIS)`) and the largest page a plan can
# have, so it is the one to match: plain `shuffle` gives LIS 10.5 +- 1.6
# against the bench's 11, and 655 crossings against 539. Those cases are
# therefore a HARDER article than the bench's K51, not a model of it -- read
# them as a stress rung, and calibrate K51 properly only with a generator
# that can hit a target LIS.
BATCHES['b6'] = (
    [case(41, 'shuffle', seed=s) for s in (0, 1, 2)]
    + [case(41, 'shuffle', seed=0, depth=2)]
    + [case(51, 'shuffle', seed=s) for s in (0, 1)]
)


# cal: the POROSITY calibration, and it is a prerequisite for b6 rather than
# a result of its own.
#
# `thru` -- lanes whose copper passes the deepest bus ball at one end, so they
# reached their pad through or around an array -- is the one property that
# decides whether the channel model applies at all, and the bench's value is
# **about a fifth**: 5 of 28, 8 of 35, 8 of 41 on the shipped boards. The
# harness's two settings bracket that and neither hits it: `pad_inner=0.6`
# CLOSES the interior (thru 0 by construction, which is what b5 measured and
# why its answer did not transfer), while the default 0.4 leaves a 0.4 mm gap
# between balls at 0.8 pitch -- wide open to a 0.127 track, and more porous
# than a real BGA, which carries its own escape copper in there.
#
# So sweep the ball diameter at a K that runs in a minute and read `thru` off
# the routed board. Porosity is geometry, so the setting transfers to K41
# where a probe costs the best part of an hour.
BATCHES['cal'] = [case(15, 'shuffle', seed=0, pad_inner=pi)
                  for pi in (0.40, 0.45, 0.50, 0.55, 0.60)]


_no_duplicate_tags()


# ---------------------------------------------------------------------------
# The truth, re-derived from the built boards
# ---------------------------------------------------------------------------

def _axis(pcb, src, dst):
    """(u, v): the unit vector source -> destination, and its left normal.
    Read off the two arrays, so a rotated or mirrored pose is the same
    computation -- nothing here assumes the channel runs along +x."""
    a, b = pcb.footprints[src], pcb.footprints[dst]
    dx, dy = b.x - a.x, b.y - a.y
    n = math.hypot(dx, dy) or 1.0
    u = (dx / n, dy / n)
    return u, (-u[1], u[0])


# the bench's 0.1 mm process, which is what make_bench stamps as the DRC
# floor and what chain_k.sh grades at. Named here because the capacity
# bound below is arithmetic on them, not a guess.
TRACK, CLEAR, EDGE = 0.1, 0.1, 0.2
# `pages_first.PAGES_SWIM` -- read from the planner rather than re-spelled,
# so the model column is priced at what the chain actually charges
SWIM_DEFAULT = float(os.environ.get('PLAN_PAGES_SWIM', '100'))


def slot_capacity(pcb, v, src, dst):
    """How many lanes can PHYSICALLY pass a corridor obstacle, by counting.

    Without this an obstacle case cannot be read: 4 open nets is a router
    defect if the lanes fit and a correct refusal if they do not, and the
    two look identical in the grade. So: take the obstacle's copper extent
    across the channel, the room left to the outline on each side, subtract
    the edge clearance and one clearance off the obstacle, and fit lanes at
    `TRACK + CLEAR` pitch -- n lanes need `n*TRACK + (n-1)*CLEAR`. Both
    copper layers are available in the slot (the obstacle blocks both, the
    slot beside it blocks neither), so the capacity is doubled.

    An UPPER bound on what any router could do, and deliberately generous:
    it ignores that a lane must also bend into the slot and back out, which
    is the very thing that goes infeasible. A case routed short of this
    number has room the chain did not use."""
    obs = [f for r, f in pcb.footprints.items()
           if r not in (src, dst)
           and any(p.drill and p.drill > 0 and not p.net_id for p in f.pads)]
    if not obs or not pcb.board_info.board_bounds:
        return None
    x0, y0, x1, y1 = pcb.board_info.board_bounds

    def pv(x, y):
        return x * v[0] + y * v[1]
    lo = hi = None
    for f in obs:
        for p in f.pads:
            rad = max(p.size_x, p.size_y) / 2
            q = pv(p.global_x, p.global_y)
            lo = q - rad if lo is None else min(lo, q - rad)
            hi = q + rad if hi is None else max(hi, q + rad)
    corners = [pv(a, b) for a in (x0, x1) for b in (y0, y1)]
    bmin, bmax = min(corners), max(corners)

    def fit(room):
        usable = room - EDGE - CLEAR
        if usable < TRACK:
            return 0
        return int((usable + CLEAR) // (TRACK + CLEAR))
    a, b = fit(lo - bmin), fit(bmax - hi)
    return {'room': (round(lo - bmin, 2), round(bmax - hi, 2)),
            'per_layer': (a, b), 'capacity': 2 * (a + b)}


def truth_from_board(bench, names, src=SRC, dst=DST, swim_price=None):
    """The known answer for the nets the chain is ACTUALLY given, read off
    the prepared bench: each lane's source tooth (the free stub end and
    the layer its copper sits on) and its destination pad, ordered along
    the channel's cross-axis. Returns the crossing count, LIS, the lower
    bound, the optimum and the teeth."""
    from kicad_parser import parse_kicad_pcb
    import braid as te
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(bench)
        bn = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
        ends = te.endpoints(pcb, names, bn)
    u, v = _axis(pcb, src, dst)
    tooth, key_s, key_d = {}, {}, {}
    for nm in names:
        s_pt, t_pt, _ref = ends[nm]
        key_s[nm] = s_pt[0] * v[0] + s_pt[1] * v[1]
        key_d[nm] = t_pt[0] * v[0] + t_pt[1] * v[1]
        nid = bn[nm][0]
        # the tooth's layer: the layer of the source stub segment that
        # owns the free end (that is the layer the lane launches on)
        best, bl = None, 'F.Cu'
        for sg in pcb.segments:
            if sg.net_id != nid:
                continue
            for p in ((sg.start_x, sg.start_y), (sg.end_x, sg.end_y)):
                d = math.hypot(p[0] - s_pt[0], p[1] - s_pt[1])
                if best is None or d < best:
                    best, bl = d, sg.layer
        tooth[nm] = bl
    order_src = sorted(names, key=lambda n: key_s[n])
    order_dst = sorted(names, key=lambda n: key_d[n])
    edges = sy.crossing_edges(order_src, order_dst)
    # the berth is the chain's own choice; the OPTIMUM assumes the best
    # one available, which on these articles is a straight escape on the
    # array's own layer at no via
    berth = {n: 'F.Cu' for n in names}
    opt, _comps, how = sy.optimum(names, edges, tooth_layer=tooth, berth_layer=berth)
    lb, lis = sy.lower_bound(order_src, order_dst)
    # the EXACT answer: pages AND mid-channel layer changes, over the
    # crossing order the real end positions give. This is the number the
    # whole-lane `opt` cannot produce for a non-bipartite case, and it is
    # <= `opt` whenever both exist -- the driver asserts that below.
    dp, dp_how = sy.exact_dp(names, key_s, key_d, tooth_layer=tooth,
                             berth_layer=berth)
    if any(l != 'F.Cu' for l in tooth.values()):
        # a tooth already on B makes the LIS bound (which assumes every
        # lane starts and ends on F) inapplicable -- say so rather than
        # print a number that is not a bound
        lb = None
    mmlb = 0.0
    for nm in names:
        nid, net = bn[nm]
        ps = [p for p in net.pads]
        if len(ps) >= 2:
            mmlb += math.hypot(ps[0].global_x - ps[1].global_x,
                               ps[0].global_y - ps[1].global_y)
    # The two lines the channel-confined model assumes a lane never passes:
    # half a pitch beyond the deepest bus ball at each end. A lane with
    # copper past one of them reached its ball THROUGH the array, which is a
    # topology the permutation's crossing graph does not describe -- it can
    # un-cross a pair with no via at all, so the optimum stops being one.
    def proj(p):
        return p[0] * u[0] + p[1] * u[1]
    sp = [proj((p.global_x, p.global_y)) for nm in names
          for p in bn[nm][1].pads if p.component_ref == src]
    dp_ = [proj((p.global_x, p.global_y)) for nm in names
           for p in bn[nm][1].pads if p.component_ref == dst]
    # the line sits at the deepest bus ball's own copper edge plus a hug
    # clearance: an arrival stub that lands ON its ball stops at the ball
    # centre and never reaches it, so only copper that went PAST the ball
    # to approach from behind trips the count. (Half a pitch was too far:
    # the measured escape ran at exactly grid+pitch/2 and read as 0.)
    rad = max((max(p.size_x, p.size_y) / 2 for nm in names
               for p in bn[nm][1].pads), default=0.2)
    half = rad + 0.105
    assert dp is None or opt is None or dp <= opt, \
        f'exact {dp} above the whole-lane optimum {opt}: one of them is wrong'
    # ---- the PLANNER'S OWN model, solved exactly, and what its answer
    # really costs. `planner gap` alone cannot say whether the plan is off
    # because the search stopped early or because the model cannot express
    # the answer; these two columns split it. `m_dp` pins the model's paged
    # lanes to their pages and re-runs the exact DP, so `model_err =
    # m_dp - dp` is measured in the same units and homotopy class as the
    # optimum. `m2_dp` is the same at a swim price of 2 -- what a lane that
    # leaves layer F and comes back actually costs -- so the pair prices
    # the shipped PLAN_PAGES_SWIM against the truth on this case.
    # ---- the CHANNEL FLOOR, available at EVERY K. `dp` is exact but costs
    # 2**(free lanes), so it is blank past the cap -- and the rungs that
    # matter, K41 and K51, have never carried a reference number at all.
    # This one is a true lower bound (synth_bus.channel_lower_bound) and
    # costs milliseconds. It bounds the CHANNEL-CONFINED class only, so on
    # a case with `thru > 0` it is the floor for the lanes that stayed in.
    clb = sy.channel_lower_bound(order_src, order_dst, tooth_layer=tooth,
                                 berth_layer=berth)
    model = {}
    # the price the CHAIN is running at, not this process's environment --
    # the driver passes the chain's own `--env` through, or a bench run
    # under one price would be graded against the model at another
    swim_p = SWIM_DEFAULT if swim_price is None else float(swim_price)
    for tag, price in (('m', swim_p), ('m2', 2.0)):
        mm_ = sy.pages_model(order_src, order_dst, tooth_layer=tooth,
                             berth_layer=berth, swim=price)
        fixed = {n: l for n, l in mm_['page'].items() if l != 'swim'}
        v_, _how = sy.exact_dp(names, key_s, key_d, tooth_layer=tooth,
                               berth_layer=berth, fixed=fixed)
        model[tag] = mm_
        model[tag + '_dp'] = v_
    return {'xing': len(edges), 'lis': lis, 'lb': lb, 'opt': opt, 'how': how,
            'dp': dp, 'dp_how': dp_how, 'slot': slot_capacity(pcb, v, src, dst),
            'teeth_b': sum(1 for l in tooth.values() if l != 'F.Cu'),
            'mm_lb': mmlb, 'u': u,
            'bound': clb['cost'],
            'm_obj': model['m']['cost'], 'paged': model['m']['paged'],
            'm_swim': model['m']['n_swim'], 'm_dp': model['m_dp'],
            'm2_swim': model['m2']['n_swim'], 'm2_dp': model['m2_dp'],
            'swim_price': swim_p,
            'model_err': (None if (model['m_dp'] is None or dp is None)
                          else model['m_dp'] - dp),
            'swim_price_cost': (None if (model['m_dp'] is None
                                         or model['m2_dp'] is None)
                                else model['m_dp'] - model['m2_dp']),
            's_in': (min(sp) - half) if sp else None,
            'd_in': (max(dp_) + half) if dp_ else None}


def through_array(board, names, u, s_in, d_in):
    """How many of `names` have copper INSIDE either array -- past the lines
    `truth_from_board` computed. Printed as `thru`: a non-zero count is the
    warning that this case's routed number may legitimately beat its
    channel-confined optimum, and it is the mechanism the b1/b2 pair of
    tables exists to show."""
    from kicad_parser import parse_kicad_pcb
    if s_in is None or d_in is None:
        return ''
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    ids = {i: n.name.split('/')[-1] for i, n in pcb.nets.items()
           if n.name.split('/')[-1] in names}
    hit = set()
    for s in pcb.segments:
        nm = ids.get(s.net_id)
        if nm is None:
            continue
        for p in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            q = p[0] * u[0] + p[1] * u[1]
            if q > d_in or q < s_in:
                hit.add(nm)
    return len(hit)


def vias_mm(board, names):
    """(vias, copper mm) on `names` for a written board."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    ids = {i for i, n in pcb.nets.items() if n.name.split('/')[-1] in names}
    mm = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in pcb.segments if s.net_id in ids)
    return sum(1 for v in pcb.vias if v.net_id in ids), mm


def plan_count(fo_board, names, dest):
    """The braid's PLAN-IMPLIED via count for the fanout board's plan --
    `judge_gate.grade_one`'s `c_sw`: the ends as laid, every page lane's
    `changes`, every swimmer's `swim_changes`, and `cross_vias`."""
    import braid as te
    sidecar = fo_board.replace('.kicad_pcb', '.plan.json')
    if not os.path.isfile(sidecar):
        return None, None
    plan = json.load(open(sidecar))
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        bp = te.plan_braid(fo_board, list(names), dest, plan)
    ends, _ = vias_mm(fo_board, names)
    ch = sw = xv = resid = 0
    for nm in names:
        b = bp.get(nm, {})
        xv += b.get('cross_vias', 0) or 0
        if b.get('page') is None:
            resid += 1
            sw += b.get('swim_changes') or 0
        else:
            ch += b.get('changes') or 0
    return ends + ch + sw + xv, resid


# ---------------------------------------------------------------------------
# The stages
# ---------------------------------------------------------------------------

def gen_and_bench(c, outdir, log=print):
    """synth_bus -> make_bench. Returns (bench path, real net list)."""
    raw = os.path.join(outdir, c['tag'] + '.kicad_pcb')
    bench = os.path.join(outdir, c['tag'] + '_b.kicad_pcb')
    argv = [PY, 'synth_bus.py', raw, '--k', str(c['k']), '--pattern', c['pattern'],
            '--seed', str(c['seed']), '--blocks', str(c['blocks']),
            '--cols', str(c['cols']), '--depth', str(c['depth']),
            '--gap', str(c['gap']), '--dst-rot', str(c['dst_rot']),
            '--caps', str(c['caps']), '--src', SRC, '--dst', DST,
            '--pad', str(c['pad']), '--margin-y', str(c['margin_y'])]
    if c['pad_inner']:
        argv += ['--pad-inner', str(c['pad_inner'])]
    if c['obstacle_h'] and c['obstacle_w']:
        argv += ['--obstacle-w', str(c['obstacle_w']),
                 '--obstacle-h', str(c['obstacle_h']),
                 '--obstacle-x', str(c['obstacle_x']),
                 '--obstacle-y', str(c['obstacle_y'])]
    if c['rows']:
        argv += ['--rows', str(c['rows'])]
    if c['row_offset']:
        argv += ['--row-offset', str(c['row_offset'])]
    if c['inversions'] is not None:
        argv += ['--inversions', str(c['inversions'])]
    if c.get('pairs'):
        argv += ['--pairs', str(c['pairs'])]
    r = subprocess.run(argv, capture_output=True, text=True)
    if r.returncode:
        log(f'  {c["tag"]}: GENERATOR FAILED\n' + (r.stdout + r.stderr)[-600:])
        return None, []
    log('  ' + r.stdout.strip())
    for ext in ('.kicad_pcb', '.kicad_pro', '.ladder.txt'):
        with contextlib.suppress(FileNotFoundError):
            os.remove(bench.replace('.kicad_pcb', ext))
    argv = [PY, 'make_bench.py', raw, SRC, DST, bench,
            '--fanout-layers', c['fanout_layers']]
    r = subprocess.run(argv, capture_output=True, text=True)
    out = r.stdout + r.stderr
    if r.returncode or not os.path.isfile(bench):
        log(f'  {c["tag"]}: MAKE_BENCH FAILED (rc={r.returncode})\n' + out[-800:])
        return None, []
    for line in r.stdout.splitlines():
        if re.search(r'fanned out|DRC|ladder|refused', line):
            log('  ' + line.strip())
    return bench, coherent(bench, c['k'])


def coherent(bench, K):
    r = subprocess.run([PY, 'coherent_nets.py', str(K), f'--board={bench}'],
                       capture_output=True, text=True)
    return [n for n in r.stdout.strip().split(',') if n]


def run_chain(c, bench, tag, env_extra=None, log=print):
    env = dict(os.environ, BASE=bench, DEST=DST, PLAN_PAGES='1')
    if c.get('pairs'):
        env.setdefault('BRAID_PAIRS', '1')
        env.setdefault('PLAN_PAIRS', '1')
    env.update(env_extra or {})
    t0 = time.time()
    r = subprocess.run(['bash', 'chain_k.sh', tag, str(c['k'])],
                       capture_output=True, text=True, env=env)
    dt = time.time() - t0
    out = r.stdout + r.stderr
    for line in out.splitlines():
        if re.search(r'GRADE|NO BRAID|NO FANOUT|FLOW FRAME|Traceback', line):
            log('  ' + line.strip())
    return dt, out


# ---------------------------------------------------------------------------

def inband_lanes(log_path):
    """(lanes routed IN BAND on the first attempt, lanes offered), summed
    over the braid's corridors, from its own `lanes: a/b routed` lines.

    This is the braid's OWN objective -- "all lanes in band" -- and it is
    reported apart from the via count because a case can be routed
    perfectly and still have left the band, which is the chain telling us
    the plan it was handed did not fit."""
    if not os.path.isfile(log_path):
        return '', ''
    a = b = 0
    for line in open(log_path, errors='replace'):
        m = re.search(r'lanes: (\d+)/(\d+) routed', line)
        if m:
            a += int(m.group(1))
            b += int(m.group(2))
    return (a, b) if b else ('', '')


def planner_ran(log_path):
    """Which planner the chain actually used, read off the FANOUT stage's
    log -- which is where `pages_first` prints, and NOT the braid's (that
    mistake is why this function's first version reported 16 of 17 cases
    as planner-less on a batch where it had run every time).

    The model columns in this table describe `pages_first`'s model, so a
    case where that planner did not run is a case they do not describe.
    The driver ASKS for it (`run_chain` sets `PLAN_PAGES=1`), and asking is
    not the same as it happening: `chain_k.sh` does not set the variable
    itself, so a chain run by hand gets the OLD planner and grades
    identically in both arms of a swim-price A/B -- measured, a whole
    vacuous K28/K35/K41 pair.

    Returns 'pages-first', or 'NO pages-first LINE' -- which is ambiguous
    on purpose, because an INFEASIBLE solve prints no such line either and
    reads exactly like a planner that never ran (README TODO item 8)."""
    if not os.path.isfile(log_path):
        return ''
    for line in open(log_path, errors='replace'):
        if 'pages-first' in line:
            return 'pages-first'
    return 'NO pages-first LINE'


def swim_price_real(log_path):
    """What a SWIMMER actually cost the braid, from its own report:
    `swimmer SYN00: 8 page crossing(s), 6 change(s), ...`.

    This is the number no offline analysis can supply. The harness's exact
    DP prices a swimmer at its BEST possible routing (2 vias for an F-to-F
    lane), because the unpinned lanes are free in it; `pages_first` charges
    a flat `PLAN_PAGES_SWIM`, 100 as shipped. The braid prints what it
    really paid, and on the generated cases the answer is neither.

    Returns (swimmers, total changes) over the braid's LAST reported block
    -- it re-plans, so an earlier block is superseded.
    """
    if not os.path.isfile(log_path):
        return '', ''
    runs, cur = [], []
    for line in open(log_path, errors='replace'):
        m = re.search(r'swimmer (\S+): (\d+) page crossing\(s\), (\d+) change',
                      line)
        if m:
            cur.append(int(m.group(3)))
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    if not runs:
        return '', ''
    return len(runs[-1]), sum(runs[-1])


FIELDS = ['tag', 'k_asked', 'k_real', 'pattern', 'xing', 'lis',
          'lb', 'bound', 'total_bound', 'opt', 'dp', 'teeth_b', 'bench_vias',
          'total_opt',
          'total_dp', 'thru',
          'plan', 'routed', 'open', 'drc', 'segs', 'resid',
          'inband', 'offered', 'slot_cap', 'slot_room',
          'planner_gap', 'braid_gap', 'total_gap', 'dp_gap',
          'planner', 'swimmers', 'swim_changes',
          'paged', 'm_swim', 'm_obj', 'm_dp', 'model_err',
          'm2_swim', 'm2_dp', 'swim_price', 'swim_price_cost', 'solve_vs_model',
          'mm', 'mm_lb', 'detour', 'sec', 'how', 'dp_how']


def grade(c, bench, tag, dt, outdir, log=print, swim_price=None):
    K = c['k']
    names = coherent(bench, K)
    fo = f'{tag}_fo_k{K}.kicad_pcb'
    rt = f'{tag}_k{K}.kicad_pcb'
    row = {f: '' for f in FIELDS}
    row.update(tag=c['tag'], k_asked=K, k_real=len(names), pattern=c['pattern'],
               sec=round(dt, 1))
    if not names:
        row['how'] = 'NO NETS -- the ladder gave the chain nothing'
        return row
    t = truth_from_board(bench, names, swim_price=swim_price)
    bench_vias, _ = vias_mm(bench, names)
    row['bound'] = t['bound']
    row['total_bound'] = t['bound'] + bench_vias
    row.update(xing=t['xing'], lis=t['lis'], lb=t['lb'], opt=t['opt'],
               dp=t['dp'], teeth_b=t['teeth_b'], bench_vias=bench_vias,
               mm_lb=round(t['mm_lb'], 1), how=t['how'], dp_how=t['dp_how'])
    row.update(paged=t['paged'], m_swim=t['m_swim'],
               m_obj=round(t['m_obj'], 1), m_dp=t['m_dp'],
               model_err=t['model_err'], m2_swim=t['m2_swim'],
               m2_dp=t['m2_dp'], swim_price=t['swim_price'],
               swim_price_cost=t['swim_price_cost'])
    if t['slot']:
        row['slot_cap'] = t['slot']['capacity']
        row['slot_room'] = '/'.join(str(r) for r in t['slot']['room'])
    if t['opt'] is not None:
        row['total_opt'] = t['opt'] + bench_vias
    if t['dp'] is not None:
        row['total_dp'] = t['dp'] + bench_vias
    if not os.path.isfile(rt):
        row['how'] = (f'NO ROUTED BOARD at {os.path.basename(rt)} -- the chain '
                      f'produced none (its known answer was lb={t["lb"]} '
                      f'exact={t["dp"]})')
        return row
    g = subprocess.run([PY, 'grade_k.py', rt, ','.join(names)],
                       capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+) segs=(\d+)', g)
    if not m:
        row['how'] = 'GRADE BROKEN: ' + g.strip()[-160:]
        return row
    row.update(open=int(m.group(1)), drc=int(m.group(2)),
               routed=int(m.group(3)), segs=int(m.group(4)))
    _, mm = vias_mm(rt, names)
    row['mm'] = round(mm, 1)
    row['thru'] = through_array(rt, names, t['u'], t['s_in'], t['d_in'])
    row['detour'] = round(mm / t['mm_lb'], 2) if t['mm_lb'] else ''
    row['inband'], row['offered'] = inband_lanes(f'{tag}_k{K}.log')
    row['planner'] = planner_ran(f'{tag}_fo_k{K}.log')
    row['swimmers'], row['swim_changes'] = swim_price_real(f'{tag}_k{K}.log')
    pc, resid = plan_count(fo, names, DST)
    if pc is not None:
        row['plan'], row['resid'] = pc, resid
        row['braid_gap'] = row['routed'] - pc
        # the PLANNER gap is measured against the EXACT optimum where
        # there is one, and only falls back to the whole-lane answer
        # when K is past the DP's reach
        ref = row['total_dp'] if row['total_dp'] != '' else row['total_opt']
        if ref != '':
            row['planner_gap'] = pc - ref
        # the plan this solve FOUND against the best its own model can
        # express. Do not read it as a search gap: it is signed, and a
        # NEGATIVE value is the interesting one -- the solve stopped
        # somewhere its objective ranks worse and the truth ranks better,
        # which is the anti-correlation, per case, with a known answer.
        if t['m_dp'] is not None:
            row['solve_vs_model'] = pc - (t['m_dp'] + bench_vias)
    if row['total_opt'] != '':
        row['total_gap'] = row['routed'] - row['total_opt']
    if row['total_dp'] != '':
        row['dp_gap'] = row['routed'] - row['total_dp']
    return row


def print_table(rows, log=print):
    cols = ['tag', 'k_real', 'xing', 'lis', 'total_bound', 'opt', 'dp',
            'total_dp', 'plan',
            'routed', 'open', 'drc', 'thru', 'slot_cap', 'inband', 'offered',
            'swimmers', 'swim_changes',
            'paged', 'm_swim', 'm_dp', 'model_err', 'm2_dp',
            'swim_price_cost',
            'planner_gap', 'solve_vs_model', 'braid_gap', 'dp_gap', 'detour',
            'sec']
    w = {c: max(len(c), *(len(str(r.get(c, ''))) for r in rows)) for c in cols}
    log('  '.join(c.rjust(w[c]) if c != 'tag' else c.ljust(w[c]) for c in cols))
    for r in rows:
        log('  '.join((str(r.get(c, '')).rjust(w[c]) if c != 'tag'
                       else str(r.get(c, '')).ljust(w[c])) for c in cols))
    # a case with no answer AND a case that never routed are both excluded;
    # `.get(k, '')` is deliberate, because a row from a failed build carries
    # only a tag and a reason (that is how this summary once crashed)
    ok = [r for r in rows
          if r.get('total_dp', '') != '' and r.get('routed', '') != '']
    broke = [r for r in rows if r.get('routed', '') == '']
    if ok:
        conf = [r for r in ok if r.get('thru') == 0]
        log(f'\n{len(ok)} case(s) with an EXACT answer: '
            f'routed {sum(r["routed"] for r in ok)} vias against an optimum of '
            f'{sum(r["total_dp"] for r in ok)}; '
            f'{sum(1 for r in ok if r["dp_gap"] == 0)} exactly optimal, '
            f'{sum(1 for r in ok if r.get("open"))} with open nets, '
            f'{sum(1 for r in ok if r.get("drc"))} with DRC.')
        if conf:
            log(f'{len(conf)} of them CHANNEL-CONFINED (thru=0), where the '
                f'optimum really is one: routed {sum(r["routed"] for r in conf)} '
                f'against {sum(r["total_dp"] for r in conf)}, '
                f'{sum(1 for r in conf if r["dp_gap"] == 0)} exact. The rest had '
                'copper inside an array, so a cheaper topology than the model '
                'describes was available to them.')
        # the planner gap, SPLIT. `model_err` is what the two-page whole-lane
        # model costs AT ITS OWN OPTIMUM, and it answers the question a bare
        # planner gap cannot: a model error means more solving is wasted,
        # because the answer is not in the model to be found.
        # a case whose planner did not run is a case the model columns do
        # not describe -- named, not silently averaged in
        off = [r for r in ok if r.get('planner') == 'NO pages-first LINE']
        if off:
            log(f'\n{len(off)} case(s) printed NO pages-first line, so the '
                f'model columns below describe a planner that may not have '
                f'run (it is also what an INFEASIBLE solve prints): '
                + ', '.join(r['tag'] for r in off[:6])
                + (', ...' if len(off) > 6 else ''))
        sp = [r for r in ok if r.get('model_err', '') != ''
              and r.get('solve_vs_model', '') != '']
        if sp:
            me = sum(r['model_err'] for r in sp)
            se = sum(r['solve_vs_model'] for r in sp)
            worse = sum(1 for r in sp if r['solve_vs_model'] < 0)
            log(f'The planner gap SPLIT over {len(sp)} case(s): MODEL error '
                f'{me:+d} vias -- the best plan the two-page model can express, '
                f'priced by the exact DP, against the optimum. '
                f'{sum(1 for r in sp if r["model_err"] > 0)} case(s) where the '
                f'model cannot express the optimum at all.')
            log(f'The solve landed {se:+d} vias from its own model\'s optimum '
                f'over those cases, BETTER on {worse} of them -- a plan its '
                f'objective ranks worse and the truth ranks better is the '
                f'anti-correlation, per case, against a known answer.')
        real = [r for r in ok if r.get('swimmers', '') not in ('', 0)]
        if real:
            n_ = sum(r['swimmers'] for r in real)
            c_ = sum(r['swim_changes'] for r in real)
            log(f'What a SWIMMER really cost the braid, from its own report: '
                f'{c_} layer change(s) over {n_} swimmer(s) = {c_ / n_:.1f} '
                f'each, on {len(real)} case(s). The exact DP prices one at 2 '
                f'(it routes the free lanes optimally) and PLAN_PAGES_SWIM '
                f'charges {real[0].get("swim_price", "?")}; neither is this '
                f'number, and this is the one the board paid.')
        sw = [r for r in ok if r.get('swim_price_cost', '') != '']
        if sw:
            c = sum(r['swim_price_cost'] for r in sw)
            log(f'The SWIM PRICE costs {c:+d} vias over {len(sw)} case(s): the '
                f'plan the model picks at PLAN_PAGES_SWIM='
                f'{sw[0]["swim_price"]:g} against the one it '
                f'picks at 2, both priced by the exact DP. '
                f'{sum(1 for r in sw if r["swim_price_cost"] > 0)} case(s) where '
                f'the shipped price picks the worse plan, '
                f'{sum(1 for r in sw if r["swim_price_cost"] < 0)} the better.')
        ib = [r for r in ok if r.get('offered', '') != '']
        if ib:
            log(f'In band on the first attempt (the braid\'s own objective): '
                f'{sum(r["inband"] for r in ib)}/{sum(r["offered"] for r in ib)} '
                f'lanes over {len(ib)} case(s); '
                f'{sum(1 for r in ib if r["inband"] < r["offered"])} case(s) '
                'left the band.')
    if broke:
        log(f'\n{len(broke)} case(s) produced NO ROUTED BOARD:')
        for r in broke:
            log(f'  {r.get("tag", "?")}: {r.get("how", "(no reason recorded)")}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--batch', default='first', choices=sorted(BATCHES))
    ap.add_argument('--outdir', default=None, help='default tmp/synth_<batch>')
    ap.add_argument('--only', help='comma-separated case tags')
    ap.add_argument('--regrade', action='store_true',
                    help='no generate, no bench, no chain -- grade what is there')
    ap.add_argument('--no-chain', action='store_true')
    ap.add_argument('--env', default='', help='extra chain env, K=V,K=V')
    a = ap.parse_args(argv)
    outdir = a.outdir or os.path.join('tmp', f'synth_{a.batch}')
    os.makedirs(outdir, exist_ok=True)
    cases = BATCHES[a.batch]
    if a.only:
        want = set(a.only.split(','))
        cases = [c for c in cases if c['tag'] in want]
        if not cases:
            raise SystemExit(f'--only {a.only}: no such case in batch {a.batch}')
    env_extra = dict(kv.split('=', 1) for kv in a.env.split(',') if kv)
    rows = []
    for c in cases:
        tag = os.path.join(outdir, c['tag'])
        bench = os.path.join(outdir, c['tag'] + '_b.kicad_pcb')
        print(f'=== {c["tag"]}  ({c["pattern"]}, K={c["k"]}, gap={c["gap"]}, '
              f'depth={c["depth"]}, rot={c["dst_rot"]}, caps={c["caps"]})')
        dt = 0.0
        if not a.regrade:
            bench, _ = gen_and_bench(c, outdir)
            if bench is None:
                r = {f: '' for f in FIELDS}
                r.update(tag=c['tag'], k_asked=c['k'], pattern=c['pattern'],
                         how='BUILD FAILED -- the generator or make_bench '
                             'refused; see the log above')
                rows.append(r)
                continue
            if not a.no_chain:
                for ext in ('.kicad_pcb', '.kicad_pro', '.plan.json', '.log'):
                    for stem in (f'{tag}_fo_k{c["k"]}', f'{tag}_k{c["k"]}'):
                        with contextlib.suppress(FileNotFoundError):
                            os.remove(stem + ext)
                dt, _ = run_chain(c, bench, tag, env_extra)
        rows.append(grade(c, bench, tag, dt, outdir,
                          swim_price=env_extra.get('PLAN_PAGES_SWIM')))
    print()
    print_table(rows)
    tsv = os.path.join(outdir, 'ladder.tsv')
    with open(tsv, 'w', encoding='utf-8') as f:
        f.write('\t'.join(FIELDS) + '\n')
        for r in rows:
            f.write('\t'.join(str(r.get(k, '')) for k in FIELDS) + '\n')
    print(f'\nrows -> {tsv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
