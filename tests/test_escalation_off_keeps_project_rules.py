#!/usr/bin/env python3
"""--escalation off never lowers a declared rule to the input's own objects.

The writeback (#160) lowers each project floor to the smallest object on the
board, the board's own input objects included: a 0.2 mm drill under a 0.3 mm
hole floor rewrote the floor to 0.2, and KiCad's DRC then graded that drill
clean. Under --escalation off the run promised the input's rules, so the
writeback (which still runs) takes its size minima only from the objects the
run ADDED (fix_kicad_drc_settings.RunMinima), while what the run explicitly
asked for is still recorded -- an explicit --clearance below the Default
class is drawn, so it must be written, or KiCad flags the router's own
copper:

1. set_policy_from_args keeps the writeback under off; off is the promised
   policy, fab is not.
2. fix_project_for_output under off, on an output that keeps the input's
   0.2 mm drill and adds a 0.25 mm via:
   - min_through_hole_diameter goes to the run's 0.25, not the input's 0.2;
   - the input's 0.2 drill is reported below the rule;
   - an explicit clearance 0.15 under the 0.2 class is written to the rule
     and the Default class (the --clearance contract);
   - the in-run sync (apply_routed_floors) still writes the routed floors;
   - compute_targets handed a whole-board scan (the GUI path) takes only
     the pad-override cap from it.
3. Control: under fab the same writeback lowers the hole floor to 0.2.

    python3 tests/test_escalation_off_keeps_project_rules.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from synth import RoutingBoard                                  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


PROJECT = {"board": {"design_settings": {
               "rules": {"min_clearance": 0.2, "min_hole_clearance": 0.25,
                         "min_track_width": 0.15, "min_via_diameter": 0.5,
                         "min_via_drill": 0.3,
                         "min_through_hole_diameter": 0.3},
               "rule_severities": {}}},
           "meta": {"filename": "b.kicad_pro", "version": 1},
           "net_settings": {"classes": [{"name": "Default", "clearance": 0.2,
                                         "track_width": 0.2, "via_diameter": 0.6,
                                         "via_drill": 0.3}],
                            "meta": {"version": 3}}}


def _board(tmp, name, extra_via=False):
    """`name`.kicad_pcb (+ .kicad_pro): an INPUT via with a 0.2 mm drill under
    the 0.3 mm floors, and with `extra_via` the run's own 0.6/0.25 via."""
    pcb = os.path.join(tmp, f'{name}.kicad_pcb')
    b = RoutingBoard(w=20, h=20)
    b.pad('C1', 5, 5, 'GND')
    b.via(10, 10, 'GND', size=0.6, drill=0.2)
    if extra_via:
        b.via(14, 14, 'GND', size=0.6, drill=0.25)
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b.text())
    pro = os.path.join(tmp, f'{name}.kicad_pro')
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(PROJECT, f, indent=2)
    return pcb, pro


def _rules(pro):
    with open(pro, encoding='utf-8') as f:
        return json.load(f)['board']['design_settings']['rules']


def _default_class(pro):
    with open(pro, encoding='utf-8') as f:
        classes = json.load(f)['net_settings']['classes']
    return next(c for c in classes if c['name'] == 'Default')['clearance']


def main():
    import fab_tiers
    from fix_kicad_drc_settings import (fix_project_for_output,
                                        apply_routed_floors, compute_targets,
                                        scan_board_minima)
    prev = fab_tiers.get_escalation_policy()
    try:
        print("1. the CLI policy keeps the writeback under off")
        args = SimpleNamespace(escalation='off', no_fix_drc_settings=False)
        with contextlib.redirect_stdout(io.StringIO()):
            fab_tiers.set_policy_from_args(args)
        check("off keeps the writeback",
              args.no_fix_drc_settings is False and fab_tiers.project_writeback())
        check("off promises the input's rules", fab_tiers.rules_are_promised())
        args = SimpleNamespace(escalation='fab', no_fix_drc_settings=False)
        with contextlib.redirect_stdout(io.StringIO()):
            fab_tiers.set_policy_from_args(args)
        check("control: fab promises nothing", not fab_tiers.rules_are_promised())

        print("2. the writeback under off")
        with tempfile.TemporaryDirectory(prefix='esc_off_') as tmp:
            inp, _ = _board(tmp, 'in')
            out, pro = _board(tmp, 'out', extra_via=True)
            fab_tiers.set_escalation_policy('off')
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                fix_project_for_output(out, input_pcb=inp, clearance=0.2)
            log = buf.getvalue()
            got = _rules(pro).get('min_through_hole_diameter')
            check("off: the hole floor follows the run's own 0.25 mm via, not "
                  "the input's 0.2 mm drill",
                  got is not None and abs(got - 0.25) < 1e-9, str(got))
            check("off: the input drill below the rule is reported",
                  'rules.min_through_hole_diameter 0.25 mm: 1 object(s) below, '
                  'smallest 0.2 mm' in log, log.strip()[-600:])

            out2, pro2 = _board(tmp, 'out2')
            with contextlib.redirect_stdout(io.StringIO()):
                fix_project_for_output(out2, input_pcb=inp, clearance=0.15)
            r = _rules(pro2)
            check("off: an explicit clearance 0.15 is written to the rule",
                  abs(r.get('min_clearance') - 0.15) < 1e-9, str(r.get('min_clearance')))
            check("off: ... and to the Default class",
                  abs(_default_class(pro2) - 0.15) < 1e-9, str(_default_class(pro2)))
            check("off: the input's 0.2 mm drill lowers nothing",
                  abs(r.get('min_through_hole_diameter') - 0.3) < 1e-9,
                  str(r.get('min_through_hole_diameter')))

            with contextlib.redirect_stdout(io.StringIO()):
                ch = apply_routed_floors(out2, clearance=0.1, hole_clearance=0.1)
            check("off: the in-run floor sync writes the routed floors", bool(ch), str(ch))

            t = compute_targets(clearance=0.2, via_drill=0.3,
                                minima=scan_board_minima(out))
            check("off: a whole-board scan (the GUI path) lowers no size floor",
                  abs(t.get('min_through_hole_diameter') - 0.3) < 1e-9,
                  str(t.get('min_through_hole_diameter')))

            print("3. control: fab")
            out3, pro3 = _board(tmp, 'out3', extra_via=True)
            fab_tiers.set_escalation_policy('fab')
            with contextlib.redirect_stdout(io.StringIO()):
                fix_project_for_output(out3, input_pcb=inp, clearance=0.2)
            got = _rules(pro3).get('min_through_hole_diameter')
            check("control: fab lowers the hole floor to the input's drill",
                  got is not None and abs(got - 0.2) < 1e-9, str(got))
    finally:
        fab_tiers.set_escalation_policy(*prev)
    if FAILS:
        print(f"\nFAILED {len(FAILS)}: " + "; ".join(FAILS))
        sys.exit(1)
    print("\nALL PASS")


if __name__ == '__main__':
    main()
