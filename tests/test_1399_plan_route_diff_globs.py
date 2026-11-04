#!/usr/bin/env python3
"""#1399: a GUI plan's route_diff step hands the engine its RAW pair patterns.

route_diff.py passes its --nets patterns as `net_name_patterns`, the #521
override list: a glob is no override, so `--force-reroute` keeps a protected
(already routed) pair. The GUI plan executor clears `dialog._plan_net_globs`
for every step and used to set it only for route steps, so a route_diff step
reached `batch_route_diff_pairs` with None -- read as "every selected net
named exactly" -- and a converted `--force-reroute` step ripped the protected
pairs the CLI kept. ai_plan imports wx, so this reads its source: the
route_diff branch of `apply_step_selection` must assign the step's patterns.

    python3 tests/test_1399_plan_route_diff_globs.py
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'kicad_routing_plugin', 'ai_plan.py')


def branch_assigns(src, action, attr):
    """Does the `action == "<action>"` branch of apply_step_selection assign
    `dialog.<attr>`? None when the branch is not found."""
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == 'apply_step_selection')
    for node in ast.walk(fn):
        if not isinstance(node, ast.If):
            continue
        t = node.test
        if (isinstance(t, ast.Compare) and isinstance(t.left, ast.Name)
                and t.left.id == 'action' and len(t.comparators) == 1
                and isinstance(t.comparators[0], ast.Constant)
                and t.comparators[0].value == action):
            body = ast.Module(body=node.body, type_ignores=[])
            return any(isinstance(n, ast.Assign) and any(
                isinstance(tg, ast.Attribute) and tg.attr == attr
                and isinstance(tg.value, ast.Name) and tg.value.id == 'dialog'
                for tg in n.targets) for n in ast.walk(body))
    return None


def main():
    src = open(SRC, encoding='utf-8').read()
    failures = []
    for action in ('route', 'route_diff'):
        got = branch_assigns(src, action, '_plan_net_globs')
        ok = got is True
        print(f"  [{'ok' if ok else 'FAIL'}] the {action} branch sets "
              f"dialog._plan_net_globs: {got}")
        if not ok:
            failures.append(action)
    # non-vacuity: a branch that never touches it reads False, not True
    ctl = branch_assigns(src, 'fanout', '_plan_net_globs')
    print(f"  [{'ok' if ctl is False else 'FAIL'}] control: the fanout branch "
          f"does not: {ctl}")
    if ctl is not False:
        failures.append('control')
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
