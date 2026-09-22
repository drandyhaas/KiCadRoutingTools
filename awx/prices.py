"""ONE price for a swimmer -- a lane the schedule gives no page -- in vias,
the same number everywhere it is charged: the plan's judge (plan_ends) and
the pages-first model (schedule). One number, calibrated on the ladder;
the two site names exist so a disagreement can be DIAGNOSED by setting one,
never to ship different prices at different sites.

    SWIM_PRICE   the price (default 2)
    SWIM_VIAS    plan_ends' reading of it
    BRAID_SWIM_W schedule's reading of it
"""
import os as _os

SWIM = float(_os.environ.get('SWIM_PRICE', '2'))
JUDGE = float(_os.environ.get('SWIM_VIAS', SWIM))       # plan_ends: accepts/rejects a batch
PAGES = float(_os.environ.get('BRAID_SWIM_W', SWIM))    # schedule: a page lane's swimmer
