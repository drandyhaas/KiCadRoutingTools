#!/usr/bin/env python3
"""The rule guard lists every rule KiCad grades that the router does not
model, and the DRC audit sorts KiCad's violations by whether the router knew
the rule.

A synthetic board with one project of each kind:

* a .kicad_dru rule whose condition the resolver cannot evaluate and that
  TIGHTENS a limit (clearance 0.5 inside a rule area): listed, can add DRC,
  and KICAD_RULES_STRICT=1 refuses the run;
* the same predicate on a rule that only RELAXES (edge clearance 0): listed,
  tagged no DRC risk, strict passes it;
* a .kicad_pro net-class key the resolver does not read: listed;
* a stale KiCad 6 Board Setup key (min_via_annulus) next to its modern name:
  not listed, KiCad grades only the modern key.

Then drc_rule_audit on a hand-written KiCad DRC report: a track narrower than
a rule the resolver knows is "bypassed", a clearance from a rule the resolver
cannot evaluate is "unknown", an unconnected pair is "other".
"""
import io
import json
import os
import contextlib
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from synth import RoutingBoard                                  # noqa: E402

DRU = """(version 1)
(rule "W" (condition "A.NetName == 'SIG'")
    (constraint track_width (min 0.18mm)))
(rule "TIGHT" (condition "A.insideArea('keep')")
    (constraint clearance (min 0.5mm)))
(rule "LOOSE" (condition "A.insideArea('edge')")
    (constraint edge_clearance (min 0mm)))
"""
PRO = {
    "board": {"design_settings": {"rules": {
        "min_clearance": 0.1, "min_track_width": 0.1, "min_copper_edge_clearance": 0.3,
        "min_via_annular_width": 0.1, "min_via_annulus": 0.05}}},
    "net_settings": {"classes": [
        {"name": "Default", "clearance": 0.2, "track_width": 0.2, "via_diameter": 0.6,
         "via_drill": 0.3, "priority": 2147483647, "pcb_color": "rgba(0,0,0,0)",
         "wire_width": 6, "future_copper_gap": 0.25}]},
}


def build(td):
    b = RoutingBoard(w=20, h=10)
    b.pad('A1', 2, 5.0, 'SIG', sx=0.6, sy=0.6)
    b.pad('A2', 18, 5.0, 'SIG', sx=0.6, sy=0.6)
    b.pad('B1', 2, 8.0, 'GND', sx=0.6, sy=0.6)
    path = os.path.join(td, 'g.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b.text())
    with open(os.path.join(td, 'g.kicad_dru'), 'w', encoding='utf-8') as f:
        f.write(DRU)
    with open(os.path.join(td, 'g.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(PRO, f)
    return path


def main():
    from kicad_parser import parse_kicad_pcb
    from design_rules import DesignRules, guard_unmodelled_rules
    import drc_rule_audit
    fails = []
    with tempfile.TemporaryDirectory() as td:
        path = build(td)
        with contextlib.redirect_stdout(io.StringIO()):
            rules = DesignRules.from_project(parse_kicad_pcb(path), path)
        lines = rules.unmodelled_rules()
        text = '\n'.join(lines)
        tight = [l for l in lines if "'TIGHT'" in l]
        loose = [l for l in lines if "'LOOSE'" in l]
        if not tight or tight[0].endswith('no DRC risk]'):
            fails.append(f"TIGHT must be listed as a DRC risk: {tight}")
        if not loose or not loose[0].endswith('no DRC risk]'):
            fails.append(f"LOOSE must be listed as relax-only: {loose}")
        if "future_copper_gap" not in text:
            fails.append("unread net-class key future_copper_gap not listed")
        for k in ('pcb_color', 'wire_width', 'min_via_annulus', "'W'"):
            if k in text:
                fails.append(f"{k} listed but it is modelled or not a copper rule")
        os.environ['KICAD_RULES_STRICT'] = '1'
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                guard_unmodelled_rules(rules)
            fails.append("KICAD_RULES_STRICT=1 did not refuse an unmodelled tightening rule")
        except SystemExit:
            pass
        rules.rules = [r for r in rules.rules if r.name != 'TIGHT']
        rules.unread_keys = []
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                guard_unmodelled_rules(rules)
        except SystemExit:
            fails.append("strict refused a board whose only unmodelled rule relaxes")
        finally:
            del os.environ['KICAD_RULES_STRICT']

        drc = {"violations": [
            {"type": "track_width", "severity": "error",
             "description": "Track width (rule 'W' min width 0.1800 mm; actual 0.1016 mm)",
             "items": [{"description": "Track [SIG] on F.Cu, length 1.0000 mm"}]},
            {"type": "clearance", "severity": "error",
             "description": "Clearance violation (rule 'TIGHT' clearance 0.5000 mm; "
                            "actual 0.3000 mm)",
             "items": [{"description": "Track [SIG] on F.Cu, length 1.0000 mm"},
                       {"description": "Pad 1 [GND] of B1 on F.Cu"}]},
            {"type": "clearance", "severity": "warning",
             "description": "Clearance violation (rule 'TIGHT' clearance 0.5000 mm; "
                            "actual 0.3000 mm)", "items": []},
        ], "unconnected_items": [{"type": "unconnected_items", "items": []}]}
        dp = os.path.join(td, 'g.drc.json')
        with open(dp, 'w', encoding='utf-8') as f:
            json.dump(drc, f)
        with contextlib.redirect_stdout(io.StringIO()):
            r = drc_rule_audit.audit(path, dp)
        want = {'unknown': 1, 'bypassed': 1, 'other': 1}
        if r['counts'] != want or r['ruleUnknown'] != 2:
            fails.append(f"audit counts {r['counts']} (ruleUnknown {r['ruleUnknown']}), "
                         f"want {want} / 2")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print(f"PASS: guard listed {len(lines)} unmodelled rule(s), strict refuses only "
          f"DRC risks; audit sorted unknown/bypassed/other")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
