"""The plan executor appends a final plane-verify step (late-pinch guard).

A route/route_diff/fanout step that runs AFTER the last plane step can sever
plane fill (ch32v203's In1.Cu GND shipped severed behind an all-green chain);
parse_plan_result must append one final repair_planes step cloned from the
last plane step -- and must NOT append when the plan already ends with a
plane step or has no plane steps at all. No wx required (stubbed).
"""
import json
import os
import sys
import types

sys.modules.setdefault('wx', types.ModuleType('wx'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'kicad_routing_plugin'))

from claude_plan import parse_plan_result  # noqa: E402

ASSIGN = [{'nets': ['GND'], 'layer': 'In1.Cu'}]


def _steps(*actions_and_extras):
    out = []
    for a in actions_and_extras:
        out.append(dict(a))
    return {'steps': out}


def main():
    # 1. Copper after the plane steps -> verify appended, cloning the repair
    #    step's assignments and params.
    plan = _steps(
        {'action': 'route', 'nets': ['*', '!GND']},
        {'action': 'route_planes', 'assignments': ASSIGN},
        {'action': 'repair_planes', 'assignments': ASSIGN,
         'params': {'rip_blocker_nets': True, 'grid_step': 0.1}},
        {'action': 'route', 'nets': ['*']},
    )
    steps, errors = parse_plan_result(json.dumps(plan))
    assert steps[-1]['action'] == 'repair_planes', steps[-1]
    assert steps[-1]['assignments'] == ASSIGN
    assert steps[-1]['params'] == {'rip_blocker_nets': True, 'grid_step': 0.1}
    assert len(steps) == 5, len(steps)

    # 2. Plan already ends with a plane step -> unchanged.
    plan2 = _steps(
        {'action': 'route', 'nets': ['*', '!GND']},
        {'action': 'route_planes', 'assignments': ASSIGN},
        {'action': 'repair_planes', 'assignments': ASSIGN},
    )
    steps2, _ = parse_plan_result(json.dumps(plan2))
    assert len(steps2) == 3, len(steps2)

    # 3. No plane steps -> unchanged.
    plan3 = _steps({'action': 'route', 'nets': ['*']})
    steps3, _ = parse_plan_result(json.dumps(plan3))
    assert len(steps3) == 1 and steps3[-1]['action'] == 'route'

    # 4. route_planes only (no repair step recorded) -> verify cloned from the
    #    route_planes step's assignments.
    plan4 = _steps(
        {'action': 'route_planes', 'assignments': ASSIGN},
        {'action': 'route', 'nets': ['*']},
    )
    steps4, _ = parse_plan_result(json.dumps(plan4))
    assert steps4[-1]['action'] == 'repair_planes'
    assert steps4[-1]['assignments'] == ASSIGN

    print("PASS: final plane-verify auto-append (late-pinch guard)")


if __name__ == '__main__':
    main()
