#!/usr/bin/env python3
"""ONE price for a swimmer. The same number everywhere, because it is the
same quantity everywhere.

A lane the schedule gives no page -- a SWIMMER -- was priced in five
places with five different numbers, all of them a via count, all of them
used the same way:

    plan_ends       pred[n] = SWIM_VIAS + berth vias        2
    braid           cvec[v] = RESIDUE_W   (level-4/5 MILP)  6
    braid           per ride + SWIM_W per swim              2.5
    braid           + SWIM_PRICE for a band-over lane       2
    sched_first     a misfit skipped at SF_SWIM             12

Nothing modelled the differences; they are five accidents. And the
disagreement was a live defect: the level-5 MILP avoided swimmers at 6
while the outer loop ACCEPTED the batches it proposed at 2, so a re-plan
that ADDED swimmers was waved through whenever the cheap arithmetic said
the total had fallen --

    RESIDUE_W=6:   residue 12 -> 14,  judged 132.67 -> 130.83   ACCEPTED
    RESIDUE_W=10:  residue 12 -> 13,  judged 137.10 -> 131.66   ACCEPTED

-- which is why raising the MILP's charge produced MORE swimmers, not
fewer.

WHAT THE NUMBER SHOULD BE is a measurement, not a preference. On routed
boards a swimmer costs 2.50 / 3.08 / 3.74 vias at K35 / K41 / K51
against a scheduled lane's 1.90 / 2.13 / 2.29, and the plan's flat 2
under-predicts a swimmer by +1.55 (K51) to +2.92 (K41). Calibrate the
ONE number on the ladder; do not re-introduce per-site values.

The per-site overrides below exist to DIAGNOSE a disagreement (to ask
"is this site the one that matters?"), never to ship one.
"""
import os as _os

# THE price of a swimmer, in vias. One number, everywhere.
SWIM = float(_os.environ.get('SWIM_PRICE', '2'))

# Per-site overrides: for diagnosis only. Unset, every one of these IS
# SWIM -- that is the point of the file.
JUDGE = float(_os.environ.get('SWIM_VIAS', SWIM))            # plan_ends: accepts/rejects a batch
PLANNER = float(_os.environ.get('BRAID_SWIM_PRICE', SWIM))   # braid: a band-over lane
PAGES = float(_os.environ.get('BRAID_SWIM_W', SWIM))         # braid: per swim in the ride cost
MILP = float(_os.environ.get('BRAID_RESIDUE_W', SWIM))       # braid: the level-4/5 residue charge
CHAIN = float(_os.environ.get('SF_SWIM', SWIM))              # sched_first: off its page's chain
