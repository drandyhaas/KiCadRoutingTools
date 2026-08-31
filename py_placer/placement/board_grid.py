"""The lattice a board was laid out on, inferred from the board (#708).

`snap_to_grid` quantizes; this decides WHAT to quantize to. The two are
separate on purpose: `grid_step` is the router's raster, a setting no board
declares (`py_tools/check_channels.py` says so in as many words), whereas a
placement lattice IS declared by a board, implicitly, in the coordinates of its
own footprints. Reading it off is what lets the placer move a part by a whole
number of the designer's grid units instead of an arbitrary residue.

Two rules here are not the obvious ones, and both come from measurement over
the tracked corpus (`tests/measure_708_lattice.py`, table C).

**The tie-break is the FINEST rung within `TIE_SLACK` of the maximum, not the
argmax.** The ladder contains divisibility chains -- 0.3175 | 0.635 | 1.27 |
2.54, and 0.05 | 0.1 | 0.2, 0.05 | 0.25 | 0.5 | 1.0 -- and occupancy is monotone
non-increasing along one: if `d` divides `s` then every on-`s` point is on-`d`,
so `occ(d) >= occ(s)`. Ties are therefore STRUCTURAL rather than accidental, and
an argmax is undefined exactly where the answer matters. Measured:
`splitflap_driver` ties 0.92 at both 0.3175 and 0.635; `sonde_u` ties 0.78 at
0.3175, 0.635 AND 1.27. Taking the coarsest would infer a 1.27 mm grid for
`sonde_u` on the strength of a tie.

`TIE_SLACK` is a tolerance on that comparison and is currently INERT: measured
over the admitting corpus boards, the nearest rung OUTSIDE each tie group sits
0.075 (`esp_prog`) to 0.460 (`interf_u_unrouted_placed`) below it, so nothing
is within 0.02 of the boundary either way. It exists so the rule survives
someone adding a rung to the ladder, not because it is load-bearing today.

A useful consequence, worth knowing before someone "simplifies" this: only
0.05 and 0.3175 have no proper divisor in the ladder, so those are the ONLY two
values this function can ever return. The issue's fear that a 2.54 mm inference
would quantize the search into uselessness cannot happen -- not because it is
clamped, but because the tie-break cannot select it. `test_708_board_grid.py`
pins that as a property of the ladder.

**A rate needs a denominator.** `cap_chain` (4 parts) and the `qfn_*` fixtures
(1-2 parts) score 1.00 at six rungs from 0.05 to 1.0, because coordinates
authored by hand are round by construction rather than by a design grid. Below
`MIN_PARTS` there is no answer to give and the caller gets `None`.

`OCCUPANCY_FLOOR` is the MIDPOINT of a real population gap, and it is not a
round number for a reason worth keeping. Sorted best-occupancy over the tracked
corpus (parts >= MIN_PARTS), the widest gap by a factor of two is

    0.700000  interf_u_unrouted            <- admitted
    0.642424  orangecrab_ext_pll           <- rejected

and every other adjacent pair differs by at most 0.05. A floor of 0.70 would
therefore sit EXACTLY on a corpus board: `>=` admits `interf_u_unrouted` and
`>` rejects it, on a float equality, which is the most fragile place a
threshold can be. 0.67 is 0.030 clear of the nearest admitted board and 0.028
clear of the nearest rejected one. The floor is far above the chance rate at
every rung --
with a 1 um tolerance a uniformly random coordinate lands on a multiple of `g`
with probability `2e-3/g`, i.e. 4.0% at 0.05 falling to 0.08% at 2.54 -- so a
single flat floor is safe HERE even though the null rate is 50x higher at the
fine end than the coarse end. Anyone lowering the floor toward 0.1 must
subtract the per-rung null rate first; at 0.67 it does not bite.
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

# The ladder issue #708 proposes, kept verbatim so this answers the issue's own
# question rather than a reshaped one.
GRID_LADDER: Tuple[float, ...] = (0.05, 0.1, 0.2, 0.25, 0.3175, 0.5, 0.635,
                                  1.0, 1.27, 2.54)
TOL_MM = 0.001            # 1 um: the issue's own tolerance
OCCUPANCY_FLOOR = 0.67    # MIDPOINT of the population gap; see the docstring
MIN_PARTS = 8             # below this a rate has no denominator
TIE_SLACK = 0.02          # inert on today's corpus; worst margin 0.075


def occupancy(values: Sequence[float], step: float,
              tol_mm: float = TOL_MM) -> float:
    """Fraction of `values` within `tol_mm` of a multiple of `step`."""
    if not values or step <= 0:
        return 0.0
    hits = sum(1 for v in values
               if abs(v / step - round(v / step)) * step <= tol_mm)
    return hits / len(values)


def board_coordinates(pcb_data) -> Tuple[list, int]:
    """(pooled x and y of every footprint ORIGIN, footprint count).

    Origins, not pads: a pad sits at its origin plus a package pitch (0.5 mm,
    0.65 mm, 1.27 mm BGA) that is on no board lattice, so a pad sample measures
    the footprint library rather than the layout.

    Every footprint, unconditionally -- not the movable set, not lock-filtered.
    The lattice must be a property of the FILE: `--lock` globs and `move_refs`
    differ between `place_optimize`, `place_seed` and `place_portfolio`, and a
    filtered sample would let the same board infer different lattices in
    different tools.
    """
    fps = list(getattr(pcb_data, 'footprints', {}).values())
    xs = [f.x for f in fps if f.x is not None and math.isfinite(f.x)]
    ys = [f.y for f in fps if f.y is not None and math.isfinite(f.y)]
    return xs + ys, len(fps)


def infer_board_grid(pcb_data, *, ladder: Sequence[float] = GRID_LADDER,
                     floor: float = OCCUPANCY_FLOOR,
                     min_parts: int = MIN_PARTS,
                     tol_mm: float = TOL_MM,
                     tie_slack: float = TIE_SLACK) -> Dict:
    """The lattice this board appears to be laid out on.

    Always returns a dict; `step is None` means "no lattice found" and `reason`
    says which test failed. Never a bare `None`: "the inference declined" and
    "the inference never ran" must not look the same to a caller, and a caller
    that wants the terse form can read `d['step']`.
    """
    for s in ladder:
        if s <= 0:
            raise ValueError("board grid ladder must be positive: %r" % (s,))
    values, n_parts = board_coordinates(pcb_data)
    out: Dict = {'step': None, 'occupancy': None, 'n_parts': n_parts,
                 'profile': {}, 'ties': (), 'reason': ''}
    if not values:
        out['reason'] = 'no footprints'
        return out
    out['profile'] = {s: occupancy(values, s, tol_mm) for s in ladder}
    if n_parts < min_parts:
        out['reason'] = 'n_parts %d < %d' % (n_parts, min_parts)
        return out
    best = max(out['profile'].values())
    ties = tuple(s for s in ladder if out['profile'][s] >= best - tie_slack)
    out['ties'] = ties
    if best < floor:
        out['reason'] = 'best occupancy %.3f < floor %.2f' % (best, floor)
        return out
    # FINEST within the slack, never the argmax -- see the module docstring.
    out['step'] = ties[0]
    out['occupancy'] = out['profile'][ties[0]]
    out['reason'] = 'inferred'
    return out


def resolve_snap_lattice(pcb_data, grid_step: float, *,
                         override: Optional[float] = None
                         ) -> Tuple[float, Dict]:
    """The lattice a candidate OFFSET should be a multiple of.

    Falls back to `grid_step`, which is exactly today's offset granularity, so
    a board with no inferable lattice keeps the step sizes it has always had.
    There is deliberately no on/off switch: the fallback IS the off state, it
    is reached by the board rather than by a flag, and a run discloses which
    branch it took through the returned evidence.
    """
    if override is not None:
        if not (override > 0):
            raise ValueError("board grid override must be positive: %r"
                             % (override,))
        return override, {'source': 'explicit', 'step': override,
                          'reason': 'override'}
    ev = infer_board_grid(pcb_data)
    if ev['step'] is None:
        ev['source'] = 'grid_step'
        ev['resolved'] = grid_step
        return grid_step, ev
    ev['source'] = 'inferred'
    ev['resolved'] = ev['step']
    return ev['step'], ev


def describe(evidence: Dict) -> str:
    """One line, in the `value (source)` idiom the placement CLIs already use."""
    if evidence.get('source') == 'explicit':
        return "placement lattice %g mm (given)" % evidence['step']
    if evidence.get('step') is None:
        return ("placement lattice %g mm (the --grid-step raster: %s)"
                % (evidence.get('resolved', 0.0), evidence.get('reason', '')))
    return ("placement lattice %g mm (inferred, %.0f%% of %d footprint "
            "coordinates%s)"
            % (evidence['step'], 100.0 * evidence['occupancy'],
               2 * evidence['n_parts'],
               '' if len(evidence.get('ties', ())) < 2
               else '; ties with ' + ', '.join('%g' % t
                                               for t in evidence['ties'][1:])))
