#!/usr/bin/env python3
"""Score a `route.py --quick 2` board: the seed judge's score.

    quick_score.py FAST.kicad_pcb [--intent INTENT] [--baseline PILE] [--json OUT]

- **blocking** is the board's own connectivity, graded the way `board_score`
  grades a full route (`check_connected`: unrouted nets plus the joins each
  broken net still needs), over the nets WITHOUT a zone. Level 2 skips the
  plane finalize, the step that taps a poured net's pads into its pour, so a
  poured net's open pads say what the fast route skipped, not what the
  placement allows: they are reported as `poured` and left out of blocking.
  The board is graded, not the route step's tally, because a chain's later
  route steps each tally only their own `--nets` (a one-net retry reports
  `already_connected` and nothing about the rest of the board).
- **floorplan**, with `--intent`, is `board_score`'s own floorplan component:
  `check_floorplan --intent` on the board, its ERROR findings counted (a
  decap past its pin limit, a claim past its `max_mm`), summed into
  `blocking` as board_score sums it. A route cannot clear such a finding, so
  a seed that carries one cannot be DONE as placed (#1296).
- **assembly**, with `--baseline` (the board the seed was placed from), is
  `board_score --baseline`'s own assembly component: check_assembly against
  that board, its blocking count (1 at least when NOT BUILDABLE) summed into
  `blocking`. No route makes a NOT BUILDABLE placement buildable either.
- **quality** (vias, copper_mm, segments) is `board_score`'s own, counted on
  FAST.kicad_pcb.

DRC is left out: level 2 skips the passes that fix grazes, so DRC on its
board measures the polish it skipped, not the placement. The keys
are `board_score --json`'s (`blocking`, `blocking_by`, `quality`), so a
seed's `s<S>_fast.score.json` reads the same either way, and what it leaves
out is listed in `ungraded`, by board_score's own component names (#1323): a
converge ledger holds quick and full rows side by side, and it compares two
rows' graded sets before it compares their `blocking`.

Rank seeds by `judge_key`: the placement's own defects first (floorplan
errors plus the assembly count; every seed that grades clean before any that
does not), then blocking, then `joins_needed` (the
joins the open nets still need, board_score's own, #1340), then vias,
copper_mm, segments. Never grade a board to ship with it: route that one in
full and run `board_score`.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'instrument'}

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))


def connectivity(board):
    """(blocking, blocking_by, poured, joins_needed) from
    `board_score.score_connectivity`, the nets with a zone set apart.
    `joins_needed` is board_score's own (#1340) over the same nets: N - 1 per
    unrouted net of N pads, plus the broken joins. None for blocking and
    joins when check_connected gave no summary."""
    from board_score import score_connectivity
    c = score_connectivity(ROOT, board)
    if not c.get('ran'):
        return None, {'unrouted': None, 'broken': None}, {'reason': c.get('reason')}, None
    zoned = set(c.get('poured_nets') or ())
    unrouted = [n for n in c.get('unrouted_net_names') or () if n not in zoned]
    pads = c.get('unrouted_pads') or {}
    detail = c.get('broken_detail') or {}
    broken = {n: v.get('joins_needed') or 0 for n, v in detail.items() if n not in zoned}
    poured = {n: v.get('joins_needed') or 0 for n, v in detail.items() if n in zoned}
    poured_unrouted = [n for n in c.get('unrouted_net_names') or () if n in zoned]
    blocking = len(unrouted) + sum(broken.values())
    # an unrouted net's pad count, else 2 (the least check_connected calls
    # unrouted): one join, as `blocking` counts it
    joins = sum(max(1, pads.get(n, 2) - 1) for n in unrouted) + sum(broken.values())
    return (blocking,
            {'unrouted': len(unrouted), 'broken': sum(broken.values()),
             'nets': sorted(set(unrouted) | {n for n, j in broken.items() if j})},
            {'unrouted': len(poured_unrouted), 'broken': sum(poured.values()),
             'nets': sorted(set(poured_unrouted) | {n for n, j in poured.items() if j})},
            joins)


#: What this score leaves out of board_score's `blocking`, by board_score's
#: component names, and why. `converge.ungraded_set` reads a payload's
#: `ungraded` list and `components[k].reason`; without them a quick row read
#: as grading everything, and the first board_score row after it -- which
#: honestly lists impedance, length and net_widths -- was refused as an
#: improvement that "graded FEWER components" (#1323). `assembly.courtyard_
#: gating` is board_score's own entry for check_assembly's moved-part gate;
#: `connectivity.poured_nets` is the half of `unrouted`/`broken` this score
#: sets apart (`poured`).
UNGRADED = {
    'drc': '--quick 2 leaves the grazes for the full route\'s cleanup; DRC is '
           'graded on the full route',
    'undersized': '--quick 2 leaves the copper polish for the full route; '
                  'sizes are graded on the full route',
    'floorplan': 'no --intent given; the floorplan is ungraded',
    'assembly': 'no --baseline given; the assembly is ungraded',
    'assembly.courtyard_gating': 'no --baseline given: check_assembly\'s '
                                 'moved-part courtyard gate is not armed',
    'impedance': 'quick_score grades no impedance',
    'length': 'quick_score grades no length matching',
    'net_widths': 'quick_score grades no per-net widths',
    'connectivity.poured_nets': '--quick 2 skips the plane finalize that joins '
                                'a poured net\'s pads, so their open pads are '
                                'reported as `poured`, not graded',
}


def quick_score(board, intent=None, baseline=None):
    """The score document. With `intent`, the floorplan is graded as
    `board_score --intent` grades it (`score_floorplan`: check_floorplan's
    ERROR findings) and summed into `blocking`; without it, or when
    check_floorplan could not grade, `floorplan` is listed in `ungraded`.
    With `baseline` (the board the seed was placed from), the assembly is
    graded as `board_score --baseline` grades it (`score_assembly`) and its
    count summed into `blocking` the same way."""
    import tempfile
    import board_score
    blk, by, poured, joins = connectivity(board)
    reasons = dict(UNGRADED)
    components = {}
    if intent:
        with tempfile.TemporaryDirectory(prefix='quick_score_') as tmp:
            fp = board_score.score_floorplan(ROOT, board, intent, tmp)
        if fp.get('ran'):
            reasons.pop('floorplan')
            components['floorplan'] = fp
            by['floorplan'] = fp['count']
            if blk is not None:
                blk += fp['count']
        else:
            reasons['floorplan'] = fp.get('reason') or reasons['floorplan']
    if baseline:
        with tempfile.TemporaryDirectory(prefix='quick_score_') as tmp:
            asm = board_score.score_assembly(ROOT, board, intent, tmp,
                                             baseline=baseline)
        if asm.get('ran'):
            reasons.pop('assembly')
            reasons.pop('assembly.courtyard_gating')
            components['assembly'] = asm
            by['assembly'] = asm['count']
            if blk is not None:
                blk += asm['count']
        else:
            reasons['assembly'] = asm.get('reason') or reasons['assembly']
    ungraded = sorted(reasons)
    components.update({k: {'ran': False, 'reason': reasons[k]} for k in ungraded})
    return {'blocking': blk, 'blocking_by': by, 'joins_needed': joins, 'poured': poured,
            'quality': board_score.quality(board), 'ungraded': ungraded,
            'components': components,
            'score': 'quick_score: connectivity of the nets without a zone (route.py --quick 2)'
                     + (', and the floorplan' if 'floorplan' not in reasons else '')
                     + (', and the assembly' if 'assembly' not in reasons else '')}


def judge_key(score):
    """How the seed judge ranks scored seeds, best first (sort ascending).

    The placement's own defects come first, ahead of the route: a seed
    check_floorplan grades with an ERROR (a decap past its pin limit), or
    check_assembly grades NOT BUILDABLE, cannot be DONE as placed whatever
    its route does, so every seed that grades clean ranks above every seed
    that does not (#1296). The floorplan errors and the assembly count are
    summed, as board_score sums them into `blocking`. Then `blocking`, then
    `joins_needed` (#1340): `blocking` counts an unrouted net as 1 whatever
    its size, so between equal blockings a board missing a 17-pad rail (16
    joins) must lose to one with a single open pad (1 join) -- by vias it
    won, its missing rail reading as fewer vias. Only then vias, copper_mm,
    segments. A floorplan or assembly nobody graded (no --intent, no
    --baseline) counts as 0, so a run scored without them ranks as it did
    before; `joins_needed` absent
    (a score written before it existed) is 0 at blocking 0 and last
    otherwise; any other value that is not a number sorts last."""
    def num(v):
        return (v if isinstance(v, (int, float)) and not isinstance(v, bool)
                else float('inf'))
    by = score.get('blocking_by') if isinstance(score.get('blocking_by'), dict) else {}
    q = score.get('quality') if isinstance(score.get('quality'), dict) else {}
    fp, asm = by.get('floorplan'), by.get('assembly')
    blk = num(score.get('blocking'))
    joins = score.get('joins_needed')
    return ((0 if fp is None else num(fp)) + (0 if asm is None else num(asm)), blk,
            0 if joins is None and blk == 0 else num(joins),
            num(q.get('vias')), num(q.get('copper_mm')), num(q.get('segments')))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('board')
    ap.add_argument('--intent',
                    help="the run's intent: grade the floorplan as `board_score --intent` "
                         "does (check_floorplan's errors, summed into blocking)")
    ap.add_argument('--baseline',
                    help="the board the seed was placed from (the pile): grade the assembly "
                         "as `board_score --baseline` does (check_assembly's count, summed "
                         "into blocking)")
    ap.add_argument('--json', help='write the score here')
    a = ap.parse_args(argv)
    sc = quick_score(a.board, intent=a.intent, baseline=a.baseline)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(sc, fh, indent=1)
    q = sc['quality']
    p = sc['poured']
    fp = sc['blocking_by'].get('floorplan')
    print(f"blocking={sc['blocking']} joins_needed={sc['joins_needed']} vias={q.get('vias')} "
          f"copper_mm={q.get('copper_mm')} segments={q.get('segments')} "
          f"(poured, not counted: {p.get('broken')} joins)"
          + (f" floorplan={fp}" if fp is not None else '')
          + (f" assembly={sc['blocking_by']['assembly']}"
             if sc['blocking_by'].get('assembly') is not None else ''))
    if fp:
        rules = sorted(set(sc['components']['floorplan'].get('violations') or ()))
        print(f"FLOORPLAN: {fp} error(s) ({', '.join(map(str, rules))}): a route cannot "
              f"clear them, so this placement cannot be DONE as it stands")
    if sc['blocking_by'].get('assembly'):
        asm = sc['components']['assembly']
        print(f"ASSEMBLY: NOT BUILDABLE ({asm.get('count_basis')}): a route cannot "
              f"change that, so this placement cannot be DONE as it stands")
    print(f"UNGRADED (the full route's board_score grades these): {', '.join(sc['ungraded'])}")
    return 2 if sc['blocking'] is None else (4 if sc['blocking'] else 0)


if __name__ == '__main__':
    sys.exit(main())
