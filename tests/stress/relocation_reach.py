#!/usr/bin/env python3
"""How far can a placement block actually move toward its connectivity target?

#554 asks for "bounded block relocation as a constraint solve": move one diagnosed
block toward where its nets want it, keeping the relative order of everything else
as a hard constraint, minimising displacement. Before any solver is worth writing,
one number decides whether the feature can exist at all:

    reach_mm   -- the furthest the block travels when neighbours may YIELD,
                  in preserved relative order
    frozen_mm  -- the furthest it travels with every non-member FROZEN

`reach_mm / frozen_mm` is the whole reason a constraint solve would beat the rigid
translate `quench` already ships (#538).

THE COMPARISON IS PAIRED, AND IT CANNOT COME OUT NEGATIVE
---------------------------------------------------------
Both arms run `relocate.reach` -- the same graph, the same feasibility rule, the
same solver -- and differ in exactly one thing: `frozen_mm` calls it on
`relocate.pin_all_but(units, block)`, so every other unit is a constant.

That makes `reach_mm >= frozen_mm` true **by construction**: pinning only removes
variables, so it can only shrink the feasible set. **"reach beat frozen on 17 of 17
cells" is therefore not evidence of anything**, and this script never reports it as
such. The evidence is the MAGNITUDE and the count of cells where the gap is
material -- which is why `reach_over_frozen` is reported per row, why cells that
move nothing in either arm are counted separately as `both_zero`, and why the
summary leads with `cells_material` rather than with a win rate.

WHAT `reach_mm` IS NOT: CLOSURE ON THE TARGET
----------------------------------------------
The direction comes from `net_centroid`, the centroid of pads on parts OUTSIDE
the block -- and the yielding arm MOVES those parts. So a block that travels
`reach_mm` does not close `reach_mm` of the gap to what it connects to: the
target drifts with the corridor. These are board-frame travel numbers, and the
distinction is not academic. Bounded by LP over the same envelope, the true
closure is smaller on most cells and LARGER on a few (a neighbour yielding can
carry the target toward the block):

    kit-dev-coldfire net:JTAG    travels 16.78   closes at most 13.05
    esp_prog decap:Y1            travels  3.41   closes at most  1.18
    splitflap decap:U10          travels  0.00   closes at most  5.72
    watchy decap:J3              travels  0.00   closes at most  4.65

**Those four rows come from a review-time probe that is NOT in this repo, and
they are not re-derived by anything.** Computing closure needs a second LP per
cell (maximise `u . (delta_centroid - delta_net_centroid)` over the same
envelope), and it is not implemented here. Treat the four numbers as an
order-of-magnitude illustration of the direction and size of the gap, not as a
measurement this tree can reproduce -- everything else in this file's output IS
re-derived by `tests/test_554_reach_regen.py`, and this paragraph deliberately
is not.

What the probe did establish, and what matters for reading the table: the
substitution leaves the aggregate verdict standing -- material either way on
essentially the same count -- so MECHANISM HOLDS is not withdrawn. But no single
cell's number here is the closure, and at least two of the `both_zero` cells,
which this file calls "the honest limit of the feature", do have room to close.
The mechanism question -- can neighbours yielding buy travel a frozen board
cannot -- is answered by travel, which is what is measured.

`slide_frozen_mm` is a THIRD number and a different question, kept because it is
what the rigid translate actually does: slide-until-contact under the block's TOTAL
violation being no worse. It is a LOOSER rule than the order graph's per-pair one
(one pair may worsen while another improves, and nothing forbids two parts
exchanging sides), so it can and does exceed `frozen_mm` on some cells. Neither is
wrong; only `frozen_mm` is paired with `reach_mm`, and only that pair may be
compared.

WHAT THIS MEASURES, AND THE INSTRUMENT THAT GOT IT WRONG FIRST
--------------------------------------------------------------
The obvious instrument is `QuenchState.group_move_valid`, which is what the quench
itself gates block moves on. Run that way, the first version of this sweep reported
**0.00 mm of feasible travel on nearly every board** -- and the number was an
artefact, because `group_move_valid` is an ABSOLUTE test and its zero-offset control
FAILED on 9 of 11 cells. Human placements routinely sit below the 0.25 mm courtyard
clearance the quench asks for (`py_placer/placement/README.md` records watchy seeding
81 of 82 parts in violation), so the incumbent board is already "invalid" and every
offset including no offset at all is refused. A sweep whose control fails is not
measuring room; it is measuring the clearance knob.

So the shipped rule here is **baseline-relative**: an offset is feasible when it
leaves the block's total violation NO WORSE than the incumbent board's. That is not
a softer rule invented for this script -- it is character-for-character what
`LegalityContext.pads_ok` applies ("no worse than the SEED baseline",
`placement/legality.py`) and what `candidate_valid`'s off-board unfreeze branch
applies. Both numbers are reported per row (`frozen_mm` baseline-relative,
`frozen_abs_mm` absolute) so the two instruments can never be confused again, and
every row carries its own control (`base_violation`, `control_ok`).

Intra-block pairs are excluded from the violation sum, for `group_move_valid`'s own
recorded reason: under a rigid translate the block's internal geometry is invariant,
so those pairs contribute exactly what they did before the move, and on a real board
members routinely sit at sub-clearance courtyard gaps already.

TWO MORE WAYS TO MEASURE THE WRONG THING, BOTH AVOIDED HERE
-----------------------------------------------------------
* **Rails in the centroid.** `block_displacements` without `ignore_net_ids`
  degenerates into "distance from the middle of the board" -- GND owns 96 of ulx3s's
  parts. Rails are cut (`--ignore-nets`; see IGNORE_NETS for how it relates to
  `perturb_batch.py`'s list, which it is not identical to).
* **The wrong direction.** `perturb.max_feasible_dose` is used by the damage rig to
  slide a block AWAY from its net centroid, off the board. Relocation goes TOWARD
  it, and the two are not the same number: measured on the same boards, toward is
  routinely larger, because a shipped board's empty space is on the outside.
  `docs/placement-optimization.md`'s recorded feasible doses are the AWAY direction.

Usage
-----
    python3 -X utf8 tests/stress/relocation_reach.py --out wk/554
    python3 -X utf8 tests/stress/relocation_reach.py --from-rows wk/554/rows.jsonl
    python3 -X utf8 tests/stress/relocation_reach.py --self-test
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_here, '..', '..'))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

SCHEMA = 1

#: Rails, so the connectivity target does not degenerate into the board centre.
#:
#: A SUPERSET of `tests/stress/perturb_batch.py`'s eight -- it adds VSS, AGND,
#: VBUS and +3.3V. An earlier comment here claimed the two lists were identical
#: "deliberately", which was false. Measured, the difference is inert on this
#: corpus except for `slide_frozen_mm_median`, which moves 0.05 -> 0.10; so the
#: two studies do not disagree about any number either of them reports, but they
#: are not the same list and this file no longer says they are.
IGNORE_NETS = ['GND', 'VCC', 'VDD', 'VSS', '+3V3', '+5V', '+1V8', '+1V1', 'GNDA',
               'AGND', 'VBUS', '+3.3V']

#: Block sources. NOT `auto` (= kicad,sheet): measured, that derives ZERO blocks on
#: five of the six boards this repo grades placement on, so a sweep over it would
#: report an empty table and look like a result.
GROUP_SOURCES = ('kicad', 'sheet', 'netprefix', 'decap')

#: Coarse-then-fine slide. A flat 0.1 mm walk over a 69 mm want on an 83-member
#: block is ~4M violation evaluations; this is ~65 and lands on the same 0.1 mm
#: grid, because the predicate is monotone in the offset (slide-until-contact,
#: never largest-feasible-k -- `perturb.max_feasible_dose` makes the same choice
#: for the same reason: a notch must not let the search jump a hole).
COARSE_STEPS = 40
FINE_STEP_MM = 0.1

#: A ranking over one candidate made no choice. Reported, never counted.
MIN_CANDIDATES = 2

BOARDS = ['esp_prog', 'splitflap_driver', 'watchy', 'tigard', 'sonde_u',
          'glasgow_revC', 'ulx3s', 'orangecrab_ext_pll',
          'kit-dev-coldfire-xilinx_5213', 'rp2350_fpga_eensy_prePlane']


# ---------------------------------------------------------------------------
# the measurement
# ---------------------------------------------------------------------------

def block_violation(state, members, dx, dy):
    """Total board+overlap violation of the block, translated rigidly by (dx, dy).

    Intra-block pairs excluded (`exclude=members`): a rigid translate leaves them
    exactly as they were, and on a real board they are routinely sub-clearance
    already -- `quench.group_move_valid` excludes them for that recorded reason,
    and a sweep that did not would veto every candidate a block ever has.
    """
    ex = set(members)
    total = 0.0
    for ref in members:
        p = state.parts[ref]
        board, overlap = state.violation_parts(ref, p.x + dx, p.y + dy, p.rot,
                                               exclude=ex)
        total += board + overlap
    return total


def slide_reach(state, members, u, want_mm, *, absolute=False, tol=1e-9):
    """Slide-until-contact toward `u`; return how far the block got, in mm.

    `absolute=False` (the shipped rule): feasible means "violation no worse than at
    zero offset". `absolute=True`: feasible means `group_move_valid`, i.e. every
    member fully legal -- reported alongside so the two instruments stay visible.

    Monotone by construction: the walk stops at the first infeasible point and
    never resumes, so a notch or an interior cutout cannot let it jump a hole and
    report a distance the block could not actually travel.
    """
    if want_mm <= 0:
        return 0.0
    ux, uy = u

    if absolute:
        def ok(d):
            return state.group_move_valid(list(members), ux * d, uy * d)
    else:
        base = block_violation(state, members, 0.0, 0.0)

        def ok(d):
            return block_violation(state, members, ux * d, uy * d) <= base + tol

    coarse = max(FINE_STEP_MM, want_mm / COARSE_STEPS)
    best = 0.0
    d = coarse
    while d <= want_mm + tol:
        if not ok(d):
            break
        best = d
        d += coarse
    # Refine inside the bracket [best, best + coarse) at the routing grid.
    d = best + FINE_STEP_MM
    limit = min(want_mm, best + coarse)
    while d <= limit + tol:
        if not ok(d):
            break
        best = d
        d += FINE_STEP_MM
    return round(best, 4)


def board_rows(name, *, ignore_nets=IGNORE_NETS, sources=GROUP_SOURCES,
               top_k=None, want_absolute=True):
    """One row per derivable block on one board. Pure measurement, no IO."""
    from kicad_parser import parse_kicad_pcb
    import pose_score
    from placement import groups as G, routability as R, diagnosis as D
    from placement import relocate as RL

    path = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
    if not os.path.exists(path):
        return [], {'board': name, 'reason': 'board file not present'}

    pcb = parse_kicad_pcb(path)
    # move_refs is deliberately NOT passed. `QuenchState.__init__` turns every ref
    # OUTSIDE move_refs into a locked part, so handing it the block would freeze the
    # whole board and this sweep would measure the frozen case while claiming to
    # measure room. The trap is invisible in the output, so it is avoided by
    # construction here and asserted in the self-test.
    state = pose_score.make_state(pcb, path)
    ids = D.ignore_net_ids(pcb, ignore_nets)
    blocks = G.derive_groups(pcb, sources)
    disps = R.block_displacements(state, blocks, ignore_net_ids=ids)
    # A block with no foreign pads is OMITTED by block_displacements rather than
    # scored 0.0, and that omission is carried through here rather than repaired:
    # "connects to nothing outside itself" and "sits exactly on its partners" are
    # different facts.
    ranked = sorted(disps, key=lambda d: (-d.distance_mm, d.block))
    # A NOTE, not a skip. An earlier version printed "SKIP <board>" here and
    # then measured and COUNTED the board's rows anyway -- and on sonde_u that
    # silently-kept row supplied the 11th material cell and the 6th material
    # board, which are exactly the floors the regen test pins. A run that says
    # it skipped a board and counts it is worse than either choice.
    #
    # The row is kept, because it is a valid measurement: this study asks how
    # far ONE block travels, which is well defined on a board with one block.
    # The <2-candidate gate belongs to a SELECTION study (#553's recall), where
    # a ranking over one candidate made no choice. Different question.
    skipped = None
    if len(ranked) < MIN_CANDIDATES:
        skipped = {'board': name, 'kind': 'note', 'counted': True,
                   'reason': f'{len(ranked)} ranked block(s). Its rows ARE '
                             f'measured and counted -- this note exists because '
                             f'a board with one block cannot support a SELECTION '
                             f'claim, not because its reach is unmeasurable',
                   'blocks_derived': len(blocks)}
    if top_k:
        ranked = ranked[:top_k]

    # Built ONCE per board and shared by both arms, so the two cannot differ by
    # anything but which units are pinned.
    RL.assert_relocatable_state(state)
    units = RL.rigid_units(state, blocks)
    edges = RL.order_graph(state, units)
    identity_bad = RL.identity_violations(edges)

    rows = []
    for bd in ranked:
        members = sorted(bd.members)
        ux = bd.net_centroid[0] - bd.centroid[0]
        uy = bd.net_centroid[1] - bd.centroid[1]
        norm = math.hypot(ux, uy)
        if norm <= 0:
            rows.append({'schema': SCHEMA, 'board': name, 'block': bd.block,
                         'members': members, 'n': len(members),
                         'want_mm': 0.0, 'skipped':
                             'the block already sits on its connectivity target'})
            continue
        u = (ux / norm, uy / norm)
        base = block_violation(state, members, 0.0, 0.0)
        slide = slide_reach(state, members, u, bd.distance_mm)
        slide_abs = (slide_reach(state, members, u, bd.distance_mm, absolute=True)
                     if want_absolute else None)
        r_yield = RL.reach(state, units, edges, bd.block, (ux, uy))
        r_frozen = RL.reach(state, RL.pin_all_but(units, bd.block), edges,
                            bd.block, (ux, uy))
        ratio = None
        if r_yield.refusal or r_frozen.refusal:
            pass
        elif r_frozen.reach_mm > 0:
            ratio = round(r_yield.reach_mm / r_frozen.reach_mm, 4)
        elif r_yield.reach_mm > 0:
            ratio = math.inf        # 0 -> something: the frozen arm cannot move
        rows.append({
            'schema': SCHEMA,
            'board': name,
            'block': bd.block,
            'members': members,
            'n': len(members),
            'want_mm': round(bd.distance_mm, 4),
            'foreign_pads': bd.foreign_pads,
            'nets': bd.nets,
            'nets_ignored': bd.nets_ignored,
            # The control. A row whose incumbent is already violating is still a
            # valid baseline-relative measurement -- the rule is "no worse" -- but
            # it is NOT a valid absolute one, and `frozen_abs_mm` on such a row is
            # 0.0 for a reason that has nothing to do with room.
            'base_violation': round(base, 6),
            'control_ok': base <= 1e-9,
            # The slide, under the looser total-violation rule. A DIFFERENT
            # question from the pair below, and never compared with it.
            'slide_frozen_mm': slide,
            'slide_frozen_abs_mm': slide_abs,
            # THE PAIR. Same graph, same rule, same solver; the only difference is
            # whether the neighbours are variables.
            'frozen_mm': round(r_frozen.reach_mm, 4),
            'reach_mm': round(r_yield.reach_mm, 4),
            'reach_over_frozen': (None if ratio is None else
                                  ('inf' if ratio == math.inf else ratio)),
            'binding_axis': r_yield.binding_axis,
            # The chain on the axis that actually BOUND. Recording the x
            # chain unconditionally named the wrong parts on 6 of 24 cells.
            'binding': [list(x) for x in r_yield.binding_chain][:6],
            'refusal': r_yield.refusal or r_frozen.refusal,
            # Must always be 0. A non-zero count means the incumbent board does
            # not satisfy its own constraints, and every number above is void.
            'identity_violations': len(identity_bad),
        })
    return rows, skipped


# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------

#: A cell counts as MATERIAL when yielding neighbours buys at least this much
#: extra travel. 1.0 mm is the smallest move worth the machinery: `place_optimize`
#: recommends a 3 mm nudge cap and `recovery.HOME_TOLERANCE_MM` is 2.0, so a
#: sub-millimetre gain is inside the noise of every consumer downstream.
MATERIAL_MM = 1.0


def summarise(rows, skipped=()):
    live = [r for r in rows if 'skipped' not in r and not r.get('refusal')]
    refused = [r for r in rows if r.get('refusal')]
    gains = [round(r['reach_mm'] - r['frozen_mm'], 4) for r in live]
    material = [r for r, g in zip(live, gains) if g >= MATERIAL_MM]
    both_zero = [r for r in live if r['reach_mm'] <= 0 and r['frozen_mm'] <= 0]
    ratios = [r['reach_mm'] / r['frozen_mm'] for r in live if r['frozen_mm'] > 0]
    unlocked = [r for r in live if r['frozen_mm'] <= 0 < r['reach_mm']]
    out = {
        'schema': SCHEMA,
        'rows': len(live),
        'boards': sorted({r['board'] for r in live}),
        'controls_ok': sum(1 for r in live if r['control_ok']),
        'controls_failed': sum(1 for r in live if not r['control_ok']),
        'identity_violations': sum(r.get('identity_violations', 0) for r in live),
        'want_mm_median': _median([r['want_mm'] for r in live]),
        # THE PAIR.
        'frozen_mm_median': _median([r['frozen_mm'] for r in live]),
        'reach_mm_median': _median([r['reach_mm'] for r in live]),
        'gain_mm_median': _median(gains),
        'gain_mm_max': max(gains) if gains else None,
        # The headline. NOT a win rate: `reach >= frozen` holds by construction
        # (pinning only removes variables), so counting wins would be counting a
        # theorem. What is measurable is how many cells the gain is big enough to
        # matter on, and how many boards those cells span.
        'cells_material': len(material),
        'cells_material_boards': sorted({r['board'] for r in material}),
        'cells_frozen_cannot_move_but_reach_can': len(unlocked),
        'cells_both_zero': len(both_zero),
        'cells_both_zero_boards': sorted({r['board'] for r in both_zero}),
        'ratio_median_where_frozen_moves': _median(ratios) if ratios else None,
        # The slide, a DIFFERENT rule, reported so it is never mistaken for the
        # paired arm. On a board whose incumbent already violates, the absolute
        # form reads 0.00 for a reason that is not about room.
        'slide_frozen_mm_median': _median([r['slide_frozen_mm'] for r in live]),
        'slide_frozen_abs_mm_median': _median(
            [r['slide_frozen_abs_mm'] for r in live
             if r['slide_frozen_abs_mm'] is not None]),
        'refused': [{'board': r['board'], 'block': r['block'],
                     'refusal': r['refusal']} for r in refused],
        'skipped': list(skipped),
    }
    out['verdict'] = _verdict(out)
    return out


def _verdict(s):
    if s['identity_violations']:
        return ('VOID: %d edge(s) refuse the incumbent board, so every number '
                'here describes a system the shipped placement does not satisfy.'
                % s['identity_violations'])
    if not s['rows']:
        return 'NO ROWS: nothing was measured.'
    n, boards = s['cells_material'], len(s['cells_material_boards'])
    if n == 0:
        return ('NO MECHANISM: yielding neighbours bought less than %.1f mm of '
                'extra travel on every one of the %d cell(s) measured. A '
                'constraint solve has nothing to offer over the rigid translate '
                'that already ships.' % (MATERIAL_MM, s['rows']))
    return ('MECHANISM HOLDS: yielding neighbours bought >= %.1f mm on %d of %d '
            'cell(s), spanning %d board(s); median gain %.2f mm, max %.2f mm. '
            'Note reach >= frozen is TRUE BY CONSTRUCTION -- the evidence is the '
            'magnitude and the spread, never the win rate. This says nothing '
            'about whether a relocated board ROUTES better.'
            % (MATERIAL_MM, n, s['rows'], boards, s['gain_mm_median'],
               s['gain_mm_max']))


def _median(vals):
    v = sorted(x for x in vals if x is not None)
    if not v:
        return None
    m = len(v) // 2
    return round(v[m] if len(v) % 2 else (v[m - 1] + v[m]) / 2.0, 4)


def format_table(rows):
    live = [r for r in rows if 'skipped' not in r]
    if not live:
        return '(no rows)'
    head = ('%-24s %-24s %4s %7s %7s %7s %7s %6s %s'
            % ('board', 'block', 'n', 'want', 'frozen', 'reach', 'gain',
               'slide', 'bound by'))
    out = [head, '-' * len(head)]
    for r in live:
        if r.get('refusal'):
            out.append('%-24s %-24s %4d  REFUSED %s'
                       % (r['board'][:24], r['block'][:24], r['n'],
                          r['refusal'][:48]))
            continue
        gain = r['reach_mm'] - r['frozen_mm']
        bound = ' -> '.join(b[0] for b in (r.get('binding') or [])[:3]) or '-'
        out.append('%-24s %-24s %4d %7.2f %7.2f %7.2f %7.2f %6.2f %s%s'
                   % (r['board'][:24], r['block'][:24], r['n'], r['want_mm'],
                      r['frozen_mm'], r['reach_mm'], gain, r['slide_frozen_mm'],
                      bound, '' if r['control_ok'] else '  [incumbent violates]'))
    return '\n'.join(out)


# ---------------------------------------------------------------------------
# self-test -- the instrument, before any board
# ---------------------------------------------------------------------------

def self_test():
    """Milliseconds. Checks the two things this script has already got wrong."""
    fails = []

    # 1. The slide is monotone and stops at the first refusal -- it must not jump
    #    a hole. Feasible on [0, 3] and again on [7, 10]: the answer is 3, not 10.
    class _S:
        def group_move_valid(self, refs, dx, dy):
            d = math.hypot(dx, dy)
            return d <= 3.0 + 1e-9 or 7.0 - 1e-9 <= d <= 10.0 + 1e-9
    got = slide_reach(_S(), ['A'], (1.0, 0.0), 10.0, absolute=True)
    if not (2.9 <= got <= 3.0):
        fails.append(f'slide jumped a hole: got {got}, want ~3.0')

    # 2. A VIOLATING incumbent must still measure room under the shipped rule, and
    #    must measure 0.0 under the absolute one. This is the artefact that made
    #    the first version of this sweep report a phantom null.
    class _P:
        def __init__(self):
            self.x = self.y = self.rot = 0.0

    class _V:
        """Incumbent violates by 1.0; sliding adds nothing until 5 mm."""
        parts = {'A': _P()}

        def violation_parts(self, ref, x, y, rot, exclude=None):
            d = math.hypot(x, y)
            return (0.0, 1.0 if d <= 5.0 else 1.0 + d)

        def group_move_valid(self, refs, dx, dy):
            return False        # absolute test: the incumbent is already illegal
    rel = slide_reach(_V(), ['A'], (1.0, 0.0), 8.0)
    ab = slide_reach(_V(), ['A'], (1.0, 0.0), 8.0, absolute=True)
    if not (4.9 <= rel <= 5.0):
        fails.append(f'baseline-relative rule refused a violating incumbent: '
                     f'got {rel}, want ~5.0')
    if ab != 0.0:
        fails.append(f'absolute rule should read 0.0 on a violating incumbent, '
                     f'got {ab}')

    # 3. `make_state` must not be handed move_refs. THE trap: every ref outside
    #    move_refs becomes locked, which would freeze the board this sweep is
    #    measuring room in -- silently, and in the direction that confirms the
    #    null.
    #
    #    Asserted on the parsed SHAPE of the call, not on a substring. The first
    #    version grepped the function body for the literal `move_refs=`, and
    #    `move_refs = _mr` (spaces) or `**{'move' + '_refs': ...}` both walked
    #    straight past it -- including versions that lock half the board. An
    #    absence guard has to anticipate every spelling; a shape guard does not.
    import ast
    src = open(os.path.abspath(__file__), encoding='utf-8').read()
    fn = next((n for n in ast.walk(ast.parse(src))
               if isinstance(n, ast.FunctionDef) and n.name == 'board_rows'),
              None)
    if fn is None:
        fails.append('board_rows not found: this check cannot be evaluated')
    else:
        calls = [c for c in ast.walk(fn) if isinstance(c, ast.Call)
                 and getattr(c.func, 'attr', None) == 'make_state']
        if not calls:
            fails.append('board_rows makes no make_state call, so the sweep is '
                         'not building the state this check is about')
        for c in calls:
            names = [k.arg for k in c.keywords]
            if 'move_refs' in names:
                fails.append('board_rows passes move_refs to make_state: every '
                             'non-member becomes locked and the sweep measures '
                             'the frozen case while claiming to measure room')
            if any(k.arg is None for k in c.keywords):
                fails.append('board_rows splats **kwargs into make_state, so this '
                             'check can no longer see whether move_refs is passed')

    # 4. The verdict must key on MAGNITUDE, not on a win rate. `reach >= frozen`
    #    is a theorem (pinning only removes variables), so a summary that called
    #    17-of-17 a result would be reporting arithmetic. A table where every gain
    #    is real but tiny must read NO MECHANISM.
    def _row(**kw):
        base = {'board': 'b', 'block': 'k', 'n': 2, 'want_mm': 5.0,
                'frozen_mm': 1.0, 'reach_mm': 1.2, 'slide_frozen_mm': 1.0,
                'slide_frozen_abs_mm': 1.0, 'base_violation': 0.0,
                'control_ok': True, 'identity_violations': 0}
        base.update(kw)
        return base

    tiny = summarise([_row(), _row(board='c', reach_mm=1.3)])
    if 'NO MECHANISM' not in (tiny.get('verdict') or ''):
        fails.append('a sub-%.1fmm gain on every cell did not read NO MECHANISM: %r'
                     % (MATERIAL_MM, tiny.get('verdict')))
    if tiny['cells_material'] != 0:
        fails.append('cells_material counted a sub-threshold gain')

    big = summarise([_row(reach_mm=6.0), _row(board='c', reach_mm=1.05)])
    if 'MECHANISM HOLDS' not in (big.get('verdict') or ''):
        fails.append('a 5 mm gain did not read MECHANISM HOLDS')
    if big['cells_material'] != 1 or big['cells_material_boards'] != ['b']:
        fails.append('cells_material miscounted: %r / %r'
                     % (big['cells_material'], big['cells_material_boards']))
    if 'BY CONSTRUCTION' not in (big.get('verdict') or ''):
        fails.append('the positive verdict omitted the by-construction caveat, '
                     'which is the only thing stopping it being read as a win rate')

    # 5. An identity violation VOIDS the run. Every number rests on `s = 0` being
    #    feasible, and it shipped false once (unclamped wall edges).
    void = summarise([_row(identity_violations=3, reach_mm=99.0)])
    if not (void.get('verdict') or '').startswith('VOID'):
        fails.append('identity violations did not void the summary')

    for f in fails:
        print('SELF-TEST FAIL: ' + f)
    print('self-test: %s (%d check groups)' % ('FAIL' if fails else 'ok', 5))
    return 1 if fails else 0


# ---------------------------------------------------------------------------

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--boards', nargs='+', default=BOARDS)
    p.add_argument('--sources', default=','.join(GROUP_SOURCES),
                   help='groups.derive_groups sources (default: every source; '
                        '`auto` derives NOTHING on 5 of the 6 graded boards)')
    p.add_argument('--ignore-nets', nargs='+', default=IGNORE_NETS)
    p.add_argument('--top-k', type=int, default=3,
                   help='blocks per board, by descending want_mm (0 = all)')
    p.add_argument('--no-absolute', action='store_true',
                   help='skip the absolute group_move_valid column (faster)')
    p.add_argument('--out', default=None, metavar='DIR')
    p.add_argument('--from-rows', default=None, metavar='JSONL',
                   help='re-summarise recorded rows; measures nothing')
    p.add_argument('--self-test', action='store_true')
    a = p.parse_args(argv)

    if a.self_test:
        return self_test()

    if a.from_rows:
        rows = [json.loads(x) for x in open(a.from_rows, encoding='utf-8')
                if x.strip()]
        print(format_table(rows))
        print()
        print(json.dumps(summarise(rows), indent=2, sort_keys=True))
        return 0

    if self_test():
        print('refusing to measure with a failing instrument')
        return 1

    sources = tuple(s for s in a.sources.split(',') if s)
    rows, skipped = [], []
    for b in a.boards:
        r, sk = board_rows(b, ignore_nets=a.ignore_nets, sources=sources,
                           top_k=(a.top_k or None),
                           want_absolute=not a.no_absolute)
        rows.extend(r)
        if sk:
            skipped.append(sk)
            print('NOTE %s: %s' % (sk['board'], sk['reason']))
    print()
    print(format_table(rows))
    summary = summarise(rows, skipped)
    print()
    print(json.dumps(summary, indent=2, sort_keys=True))

    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, 'rows.jsonl'), 'w', encoding='utf-8') as fh:
            for r in rows:
                fh.write(json.dumps(r, sort_keys=True) + '\n')
        with open(os.path.join(a.out, 'summary.json'), 'w', encoding='utf-8') as fh:
            json.dump(summary, fh, indent=2, sort_keys=True)
        print('\nwrote %s/rows.jsonl and summary.json' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
