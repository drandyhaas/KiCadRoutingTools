#!/usr/bin/env python3
"""--escalation off ships the project's DRC rules as declared.

The writeback (#160) lowers each project floor to the smallest object on the
board, the board's own input objects included: a 0.2 mm drill under a 0.3 mm
hole floor rewrote the floor to 0.2, and KiCad's DRC then graded that drill
clean. Under --escalation off the run promised the project's rules, so:

- set_policy_from_args turns the writeback off (as --no-fix-drc-settings)
  and says so;
- fix_project_for_output and the in-run apply_routed_floors write no floor,
  whoever calls them;
- report_below_project_rules names the objects below the declared rules,
  which DRC then reports.

Control: under --escalation fab the same writeback still lowers the floor.

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


def _board(tmp):
    """A board whose INPUT via has a 0.2 mm drill, under the 0.3 mm floors."""
    pcb = os.path.join(tmp, 'b.kicad_pcb')
    b = RoutingBoard(w=20, h=20)
    b.pad('C1', 5, 5, 'GND')
    b.via(10, 10, 'GND', size=0.6, drill=0.2)
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b.text())
    pro = os.path.join(tmp, 'b.kicad_pro')
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(PROJECT, f, indent=2)
    return pcb, pro


def _rule(pro, key):
    with open(pro, encoding='utf-8') as f:
        return json.load(f)['board']['design_settings']['rules'].get(key)


def main():
    import fab_tiers
    from fix_kicad_drc_settings import (fix_project_for_output,
                                        apply_routed_floors)
    prev = fab_tiers.get_escalation_policy()
    try:
        print("1. the CLI policy turns the writeback off")
        args = SimpleNamespace(escalation='off', no_fix_drc_settings=False)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            fab_tiers.set_policy_from_args(args)
        check("off implies --no-fix-drc-settings", args.no_fix_drc_settings is True)
        check("off: the run's writeback is off", fab_tiers.project_writeback() is False)
        check("off: the console says the rules ship as declared",
              'ship as declared' in buf.getvalue(), buf.getvalue().strip())
        args = SimpleNamespace(escalation='fab', no_fix_drc_settings=False)
        with contextlib.redirect_stdout(io.StringIO()):
            fab_tiers.set_policy_from_args(args)
        check("control: fab keeps the writeback",
              args.no_fix_drc_settings is False and fab_tiers.project_writeback())

        print("2. the writers write no floor under off")
        with tempfile.TemporaryDirectory(prefix='esc_off_') as tmp:
            pcb, pro = _board(tmp)
            before = open(pro, encoding='utf-8').read()
            fab_tiers.set_escalation_policy('off')
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                fix_project_for_output(pcb, clearance=0.2, via_drill=0.3)
                ch = apply_routed_floors(pcb, clearance=0.1, hole_clearance=0.1)
            log = buf.getvalue()
            check("off: the project file is untouched",
                  open(pro, encoding='utf-8').read() == before)
            check("off: the in-run floor sync writes nothing", ch == [], str(ch))
            check("off: the input drill below the hole floor is reported",
                  'rules.min_through_hole_diameter 0.3 mm: 1 object(s) below, '
                  'smallest 0.2 mm' in log, log.strip())

            fab_tiers.set_escalation_policy('fab')
            with contextlib.redirect_stdout(io.StringIO()):
                fix_project_for_output(pcb, clearance=0.2, via_drill=0.3)
            got = _rule(pro, 'min_through_hole_diameter')
            check("control: fab lowers the hole floor to the drill",
                  got is not None and abs(got - 0.2) < 1e-9, str(got))
    finally:
        fab_tiers.set_escalation_policy(*prev)
    if FAILS:
        print(f"\nFAILED {len(FAILS)}: " + "; ".join(FAILS))
        sys.exit(1)
    print("\nALL PASS")


if __name__ == '__main__':
    main()
