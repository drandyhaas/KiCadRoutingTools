#!/usr/bin/env python3
"""make_plan / run_plan: the CLI front doors (#506, #507).

Two small tools that expose what the stress harness already did:

  * ``make_plan.py``   -- a GUI-loadable plan JSON from a recorded command chain
  * ``run_plan.py``    -- run that plan headless through the real plugin

What is checked here is everything that does NOT need wx/pcbnew: the manifest ->
plan conversion (including that it still matches ``manifest_to_plan.py`` exactly
after both were pointed at one shared function), input resolution and
step-range parsing.
"""
import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURE = os.path.join(ROOT, 'tests', 'gui_parity', 'fixtures',
                       'sample_redo_commands.sh')

failures = []


def check(name, ok, detail=''):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f": {detail}" if detail else ''))
    if not ok:
        failures.append(name)


def test_make_plan(tmp):
    print("1. make_plan: a recorded manifest -> a GUI-loadable plan")
    import make_plan

    rundir = os.path.join(tmp, 'myboard')
    os.makedirs(rundir)
    shutil.copy(FIXTURE, os.path.join(rundir, 'redo_commands.sh'))

    out, steps, _skipped = make_plan.make_plan(rundir)
    check("default output is <dir>/<dirname>_plan.json",
          os.path.basename(out) == 'myboard_plan.json', os.path.basename(out))
    check("the plan file parses", os.path.exists(out))
    plan = json.load(open(out))
    actions = [s['action'] for s in plan['steps']]
    check("every recorded routing command became a step",
          actions == ['fanout', 'route_diff', 'route', 'route_planes',
                      'repair_planes'], str(actions))
    check("params are carried from the CLI flags",
          plan['steps'][2]['params'].get('clearance') == 0.09
          and plan['steps'][2]['params'].get('max_ripup') == 10,
          str(plan['steps'][2].get('params')))
    check("the plan carries no private _files keys",
          all('_files' not in s for s in plan['steps']))
    check("a directory and its manifest resolve the same",
          make_plan.resolve_manifest(rundir)
          == make_plan.resolve_manifest(os.path.join(rundir, 'redo_commands.sh')))

    # The converter now has ONE implementation (manifest_to_plan.
    # plan_steps_from_manifest) shared by make_plan.py, manifest_to_plan.main()
    # and the parity harness. Prove the front doors agree.
    sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
    import manifest_to_plan as M
    direct = os.path.join(tmp, 'direct_plan.json')
    argv = sys.argv
    try:
        sys.argv = ['manifest_to_plan.py', os.path.join(rundir, 'redo_commands.sh'),
                    direct]
        M.main()
    finally:
        sys.argv = argv
    check("make_plan matches manifest_to_plan byte for byte",
          open(direct, 'rb').read() == open(out, 'rb').read())
    check("keep_files=True keeps the chain files the parity harness needs",
          all('_files' in s for s in M.plan_steps_from_manifest(
              os.path.join(rundir, 'redo_commands.sh'), keep_files=True)[0]))

    missing = os.path.join(tmp, 'no_manifest_here')
    os.makedirs(missing)
    try:
        make_plan.resolve_manifest(missing)
        check("a directory with no manifest is an error", False)
    except FileNotFoundError:
        check("a directory with no manifest is an error", True)


def test_run_plan_selection():
    print("2. run_plan: --steps selection")
    import run_plan

    check("no selection = every step",
          run_plan.parse_step_selection(None, 4) == [0, 1, 2, 3])
    check("ranges are 1-based and inclusive",
          run_plan.parse_step_selection('1,3-5', 6) == [0, 2, 3, 4])
    check("duplicates collapse, order is kept",
          run_plan.parse_step_selection('3,1,3', 3) == [2, 0])
    for spec in ('0', '7', '2-9'):
        try:
            run_plan.parse_step_selection(spec, 5)
            check(f"out-of-range '{spec}' is refused", False)
        except ValueError:
            check(f"out-of-range '{spec}' is refused", True)


def main():
    tmp = tempfile.mkdtemp(prefix='plan_tools_')
    try:
        test_make_plan(tmp)
        test_run_plan_selection()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    if failures:
        print(f"FAILED ({len(failures)}): {failures}")
        return 1
    print("PASS: make_plan / run_plan behave")
    return 0


if __name__ == '__main__':
    sys.exit(main())
