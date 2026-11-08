"""cp_seed.solver(): a CP-SAT solver under the bus step's seed.

Every solve of a whole-route round -- the joint ends and their checks, the escape plans, the round's solve and the
loop's re-solves -- searches under ONE seed, BUS_SEED, so a round is one draw: the same board and seed give the same
plan, and another seed another plan. A round that leaves nets open is followed by round 1 again under the next seed
(whole_route.chain). Unset, a solver keeps CP-SAT's own default seed, and round 1 is what it has always been.

CP-SAT is imported by solver() itself: a driver that reads only DEFAULT (whole_route) imports this module before
its dependency gate (startup_checks.check_bus_dependencies) has run, and a missing ortools must reach that gate."""
import awx_settings

DEFAULT = 1             # CP-SAT's own random_seed when none is given: round r runs under DEFAULT + r - 1


def solver():
    """a CpSolver, its random_seed BUS_SEED's when the step gives one"""
    from ortools.sat.python import cp_model
    s = cp_model.CpSolver()
    v = awx_settings.get('BUS_SEED')
    if v not in (None, ''):
        s.parameters.random_seed = int(v)
    return s
