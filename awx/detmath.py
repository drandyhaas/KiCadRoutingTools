"""detmath -- one answer per linear program. The geometry's LP and the polish's are degenerate: a face of equal optima,
and which of its vertices the solver lands on is decided by its own rounding, so a change that should not matter -- a
row order, an input's last bit -- lands the same model on another point (K28: the geometry LP's input identical, its
answer 10 mm apart). lp_tie_break decides the model's exact ties by a fixed cost per column (small beside every real
cost, large beside the solver's tolerances), and lp_round takes the solver's last bits off, so a solution is a
function of the model alone.

(This module also held fdlibm's transcendental functions, swapped in for math's and numpy's in every chain stage so a
Mac and a Linux box gave the same bits. The whole route asks for the same answer on each machine type, not across
them, and the swap is gone: it replaced math's and numpy's functions for the whole process, which a routing call
inside KiCad's own process cannot do to everything else it runs.)
"""

# ---- linear programs: one answer per model, whatever the solver's rounding
LP_TIE = 1e-4           # per unit of a column: below any real cost's step, above HiGHS's 1e-7 tolerances
LP_QUANT = 2.0 ** -24   # a solution rounded to this: ~6e-8 (mm), exact in binary, far below any geometry


def lp_tie_break(n):
    """a fixed cost per column (LP_TIE times 0.5 .. 1.5, the golden-ratio sequence): with it, the LP's optimum is one
    point, not a face whose vertex the solver's rounding picks"""
    import numpy as np
    return LP_TIE * (0.5 + (np.arange(1, n + 1, dtype=float) * 0.6180339887498949) % 1.0)


def lp_round(x):
    """the solution without the solver's last bits (a power-of-two grid: the scaling is exact)"""
    import numpy as np
    return np.round(np.asarray(x, float) / LP_QUANT) * LP_QUANT
