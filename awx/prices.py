"""ONE price for a swimmer -- a lane the schedule gives no page -- in vias,
the same number everywhere it is charged: the plan's judge (plan_ends) and
the pages-first model (schedule). One number, calibrated on the ladder.

    SWIM    the price (2)
    JUDGE   plan_ends' reading of it
    PAGES   schedule's reading of it
"""

SWIM = 2.0
JUDGE = SWIM        # plan_ends: accepts/rejects a batch
PAGES = SWIM        # schedule: a page lane's swimmer
