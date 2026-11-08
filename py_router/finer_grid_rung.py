"""The final reconciliation's finer-grid rung (#1763).

A fine-pitch net whose pads sit off the run's grid can stay boxed in through
every in-run retry: the per-net rescue's finer grid (net_rescue rung 0) routes
through free space only, the terminal escalation keeps the run's grid, and the
reconciliation laps keep it too -- so the copper this same run laid around the
pads (or an earlier step's) walls them in at a grid that never changes.

This rung is ONE more reconciliation lap, after the ordinary laps, for the nets
still open whose failure was the static box-in verdict: it re-routes them at
HALF the run's grid, with rip authority over the blockers the last lap named
for them. The authority is filtered by the reconciliation's own guard (the
caller passes it in): protected nets are never granted, nets negated by plan
and plane/zone-backed nets are skipped, and locked copper is dropped by the
sub-run's own rip filter, exactly as for the #103 escalation.

The lap is whole-board (the reconciliation re-invokes batch_route on the
board), so it is bounded by a cell budget instead of a window: a board whose
half-grid map would exceed FINER_GRID_RUNG_MAX_CELLS (summed over the routing
layers) skips the rung and says so (#105: halving the grid quadruples the
cells).
"""

import routing_defaults as defaults


def rung_cells(board_bounds, grid_step: float, n_layers: int) -> float:
    """Cells of a whole-board map at `grid_step` over `n_layers` layers."""
    if not board_bounds or grid_step <= 0:
        return 0.0
    w = max(0.0, board_bounds[2] - board_bounds[0])
    h = max(0.0, board_bounds[3] - board_bounds[1])
    return (w / grid_step) * (h / grid_step) * max(1, n_layers)


def boxed_in_names(open_targets: dict, outer_boxed: set) -> list:
    """The open targets that qualify: their last lap (or the outer pass)
    recorded the static box-in verdict for them."""
    return [n for n, rec in (open_targets or {}).items()
            if (rec or {}).get('boxed_in') or n in outer_boxed]


def rung_blockers(open_targets: dict, names) -> list:
    """The blockers the last lap named for `names`, first-seen order."""
    out = []
    for n in names:
        for b in ((open_targets or {}).get(n) or {}).get('blockers') or []:
            if b not in out and b not in names:
                out.append(b)
    return out


def plan(open_targets: dict, outer_boxed: set, grid_step: float,
         board_bounds, n_layers: int):
    """Decide the rung. Returns (names, fine_grid, cells, reason): names is
    empty when the rung does not run, and `reason` then says why (None when
    there was simply nothing to do)."""
    names = boxed_in_names(open_targets, outer_boxed)
    if not names:
        return [], None, 0.0, None
    fine = grid_step / 2.0
    cells = rung_cells(board_bounds, fine, n_layers)
    if cells > defaults.FINER_GRID_RUNG_MAX_CELLS:
        return [], fine, cells, (
            f"a whole-board grid-{fine:g} map is ~{cells / 1e6:.0f}M cells "
            f"over {n_layers} layer(s), past the "
            f"{defaults.FINER_GRID_RUNG_MAX_CELLS / 1e6:.0f}M budget")
    return names, fine, cells, None
