#!/usr/bin/env python3
"""A via a courtyard-scoped `.kicad_dru` rule sizes is graded at that rule by
the fan-out, and the project writeback does not lower the board-wide minimum
to it (#1972).

The board declares Board Setup min_via_annular_width 0.13 mm, and one rule
relaxes the ring to 0.07 mm for vias inside one QFN's courtyard
(`A.Type == 'Via' && A.insideCourtyard('U3')`). An under-pad fan-out of that
QFN lays 0.30/0.15 vias (ring 0.075), every one inside the courtyard, which
KiCad's DRC and check_drc both accept. The fan-out used to grade each via
with no position, so the rule never bound it: with the writeback off it
printed that KiCad flags all of them under the Board Setup ring, and with
the writeback on it lowered the WHOLE board's min_via_annular_width to 0.075.

  1. writeback on: the written project keeps min_via_annular_width 0.13 and
     the run says nothing about relaxing the ring;
  2. writeback off: no "KiCad flags them" warning, and JSON_SUMMARY
     `board_setup_vias` carries no annular row;
  3. control, the same runs with the rules file removed: the warning names
     the vias below 0.13 and the writeback lowers the ring to 0.075 -- so 1
     and 2 are the rule's doing, not a run that laid nothing;
  4. units: a via dict inside the courtyard is graded at the rule and one
     outside it at the Board Setup ring (`annular_grade` with the step's
     courtyards), and `via_rule_held_test` says the same per via.

    python3 tests/test_1972_fanout_courtyard_via_rule.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
PART = 'U3'
PRO = {"board": {"design_settings": {"rules": {
    "min_via_annular_width": 0.13, "min_clearance": 0.1, "min_track_width": 0.1,
    "min_via_diameter": 0.3, "min_through_hole_diameter": 0.15}}},
    "net_settings": {"classes": [{"name": "Default", "clearance": 0.1,
                                  "track_width": 0.1, "via_diameter": 0.3,
                                  "via_drill": 0.15}]},
    "meta": {"version": 3}}
DRU = f"""(version 1)
(rule small_ring_in_part
  (condition "A.Type == 'Via' && A.insideCourtyard('{PART}')")
  (constraint annular_width (min 0.07mm)))
"""
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def make_board(d, with_rule=True):
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'in.kicad_pcb')
    shutil.copyfile(SRC, pcb)
    with open(os.path.join(d, 'in.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    if with_rule:
        with open(os.path.join(d, 'in.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(DRU)
    return pcb


def fanout(pcb, *extra):
    out = os.path.join(os.path.dirname(pcb), 'out.kicad_pcb')
    r = run_utils.check([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py', pcb,
                         '--component', PART, '--escape-method', 'underpad',
                         '--nets', '*', '-o', out, *extra], accept=True)
    text = r.stdout + r.stderr
    m = re.search(r'Underpad via-drop: (\d+) vias placed', text)
    summary = None
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            summary = json.loads(line.split(':', 1)[1])
    return out, text, int(m.group(1)) if m else 0, summary


def ring_written(out):
    pro = os.path.splitext(run_utils.evidence(out, 'fan-out board'))[0] + '.kicad_pro'
    with open(run_utils.evidence(pro, 'written project'), encoding='utf-8') as fh:
        return json.load(fh)['board']['design_settings']['rules'].get('min_via_annular_width')


def annular_rows(summary):
    return [r for r in (summary or {}).get('board_setup_vias') or []
            if 'smallest_ring' in r]


def main():
    tmp = tempfile.mkdtemp(prefix='t1972_')
    try:
        print("1. writeback on, the rule beside the board")
        out, text, n, _s = fanout(make_board(os.path.join(tmp, 'on')))
        ok("the fan-out laid vias", n > 0, f"{n}")
        ok("the written project keeps min_via_annular_width 0.13",
           ring_written(out) == 0.13, f"{ring_written(out)}")
        ok("no ring relaxation is reported", 'via annular ring:' not in text)

        print("2. writeback off, the rule beside the board")
        out, text, n, s = fanout(make_board(os.path.join(tmp, 'off')),
                                 '--no-fix-drc-settings')
        ok("the fan-out laid vias", n > 0, f"{n}")
        ok("no 'KiCad flags them' warning", 'KiCad flags them' not in text)
        ok("JSON_SUMMARY board_setup_vias has no annular row",
           s is not None and not annular_rows(s), f"{(s or {}).get('board_setup_vias')}")

        print("3. control: the same runs without the rules file")
        out, text, n, _s = fanout(make_board(os.path.join(tmp, 'on_c'), with_rule=False))
        ok("the writeback lowers the ring to the vias' 0.075",
           ring_written(out) == 0.075, f"{ring_written(out)}")
        ok("and says so", 'via annular ring: 0.13 -> 0.075 mm' in text)
        out, text, n, s = fanout(make_board(os.path.join(tmp, 'off_c'), with_rule=False),
                                 '--no-fix-drc-settings')
        ok("the warning names the vias below the Board Setup ring",
           f'{n} BELOW it (smallest ring 0.075 mm); KiCad flags them' in text)
        ok("JSON_SUMMARY board_setup_vias carries the annular row",
           [r['below'] for r in annular_rows(s)] == [n], f"{annular_rows(s)}")

        print("4. units: inside and outside the courtyard")
        from kicad_parser import parse_kicad_pcb
        from design_rules import DesignRules, grade_courtyards, via_rule_held_test
        pcb_path = make_board(os.path.join(tmp, 'unit'))
        pcb = parse_kicad_pcb(pcb_path)
        fp = pcb.footprints[PART]
        net = next(p.net_id for p in fp.pads if p.net_id)
        inside = {'x': fp.x, 'y': fp.y, 'size': 0.3, 'drill': 0.15, 'net_id': net}
        outside = dict(inside, x=fp.x + 15.0)
        dr = DesignRules.from_project(pcb, pcb_path)
        cy = grade_courtyards(dr, pcb, pcb_path)
        ok("the step's courtyards are measured", cy is not None)
        rows = {r['rule']: r for r in dr.annular_grade([inside, outside], {'annular': 0.13}, cy)}
        ok("the via inside is graded at the rule and meets it",
           rows.get('small_ring_in_part', {}).get('vias') == 1
           and rows['small_ring_in_part']['below'] == 0, f"{rows}")
        setup = rows.get('Board Setup min_via_annular_width', {})
        ok("the via outside is graded at the Board Setup ring and is below it",
           setup.get('vias') == 1 and setup.get('below') == 1, f"{setup}")
        held = via_rule_held_test(pcb, pcb_path)
        from types import SimpleNamespace
        ok("via_rule_held_test: the rule holds the ring inside, not outside",
           held is not None and held(SimpleNamespace(**inside), 'annular_width')
           and not held(SimpleNamespace(**outside), 'annular_width'))
        ok("via_rule_held_test: no rule sizes the diameter",
           held is not None and not held(SimpleNamespace(**inside), 'via_diameter'))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAILED: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
